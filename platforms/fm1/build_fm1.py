#!/usr/bin/env python3
"""Build Ruido for M-VAVE FM-1 (AC791N / pi32v2) and run Gate 4.1 + 4.2 + 4.3 verification.

Supports fast `-MD` + SHA-256 incremental `.o` caching (`--rebuild` to force cold compile) and
`--skip-emu` (`make fm1-build`) for rapid ~6-8 s link + size + disassembly checks, while `make fm1-check`
also runs all 5 `fm1-emu` dual-core SoC hardware gates in parallel.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
import pathlib
import re
import subprocess
import sys

from fm1_audit import audit_firmware_image
from fm1_common import (
    APP_XIP,
    C1_OBJ_DIR,
    FM1_DIR,
    GEN_DIR,
    MAKEFILE,
    OUT_DIR,
    PRODUCT_DEV,
    RAW_OBJ_DIR,
    SDK_SHA256,
    CompileSpec,
    collect_c1_weak_symbols,
    compile_cached_jobs,
    delegate_to_docker,
    have_local_toolchain,
    parse_makefile_var,
    relocate_obj_to_c1_sram,
    run_cmd,
    write_bytes_if_changed,
    write_text_if_changed,
)
from fm1_emu_check import run_emu_gates
from fm1_stage_loader import build_loader, stage_platform_sources


def detect_cxx_std(clang_bin: pathlib.Path) -> str:
    for flag in ("-std=c++17", "-std=c++1z"):
        res = subprocess.run(
            [str(clang_bin), "-target", "pi32v2", "-x", "c++", flag, "-fsyntax-only", "-"],
            input="inline constexpr int kTest = 1;\n",
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode == 0:
            return flag
    raise SystemExit("[FM-1 FAIL] pi32v2 clang supports neither -std=c++17 nor -std=c++1z")


def main(argv: list[str]) -> int:
    if not have_local_toolchain():
        return delegate_to_docker(argv)

    ap = argparse.ArgumentParser()
    ap.add_argument("--dsp-opt", default="-O3", choices=["-Os", "-O2", "-O3"])
    ap.add_argument("--ui-opt", default="-O2", choices=["-Os", "-O2", "-O3"])
    ap.add_argument("--skip-emu", action="store_true", help="Run build + static audits without fm1-emu")
    ap.add_argument("--rebuild", action="store_true", help="Ignore `.o` cache and recompile all TUs")
    args = ap.parse_args(argv)

    tc = pathlib.Path(os.environ.get("JIELI_TOOLCHAIN", "/opt/jieli/toolchain")).resolve()
    sdk = pathlib.Path(os.environ.get("AC79_SDK", "/opt/jieli/ac79-sdk")).resolve()
    dinghy_dir = pathlib.Path(os.environ.get("DINGHY_DIR", "/opt/dinghy")).resolve()

    sys.path.insert(0, str(dinghy_dir / "tools"))
    import fm1pkg_make  # type: ignore
    import lz4blk  # type: ignore

    fm1pkg_make.set_board("fm1")
    fm1pkg_make.SDK = sdk
    app_slot = fm1pkg_make.APP_SLOT

    for rel, expected_sha in SDK_SHA256.items():
        actual_sha = hashlib.sha256(fm1pkg_make.sdk_file(rel)).hexdigest()
        if actual_sha != expected_sha:
            raise SystemExit(f"[FM-1 FAIL] SDK SHA-256 mismatch on {rel}: {actual_sha}")

    clang = tc / "pi32v2" / "bin" / "clang"
    ld = tc / "pi32v2" / "bin" / "ld"
    objcopy = tc / "common" / "bin" / "objcopy"
    objdump = tc / "common" / "bin" / "objdump"
    compiler_rt = tc / "pi32v2" / "lib" / "r3" / "libcompiler_rt.a"

    cxx_std = detect_cxx_std(clang)
    mk_text = MAKEFILE.read_text(encoding="utf-8")
    dsp_srcs = parse_makefile_var(mk_text, "DSP_SRCS")
    ui_srcs = parse_makefile_var(mk_text, "UI_SRCS")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    RAW_OBJ_DIR.mkdir(parents=True, exist_ok=True)
    C1_OBJ_DIR.mkdir(parents=True, exist_ok=True)

    file_sha_cache: dict[pathlib.Path, str] = {}
    fw = dinghy_dir / "firmware"
    stage_platform_sources(fw)
    ota, ldr_hits, ldr_built = build_loader(
        tc, dinghy_dir, lz4blk, cxx_std, file_sha_cache, rebuild=args.rebuild
    )

    # Keep platform.c / usb.c strictly at dinghy's hardware-verified C flags (-Os, no -O2 USB babble)
    plat_cflags = [
        "-target",
        "pi32v2",
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-builtin",
        "-Wall",
        "-Wno-unused-function",
        f"-I{GEN_DIR}",
        f"-I{fw / 'hal'}",
        f"-I{fw / 'src'}",
        f"-I{fw / 'gen'}",
        f"-I{FM1_DIR}",
        f'-DFELUCCA_ID="{PRODUCT_DEV}"',
        '-DAPP_VER="1.0"',
    ]

    common_cxxflags = [
        "-target",
        "pi32v2",
        "-mcpu=r3",
        "-mfprev1",
        cxx_std,
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wno-missing-braces",
        "-Wdouble-promotion",
        "-Wshadow",
        "-Wglobal-constructors",
        "-Wexit-time-destructors",
        "-fno-exceptions",
        "-fno-rtti",
        "-fno-math-errno",
        "-ffp-contract=off",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-threadsafe-statics",
        "-nostdinc",
        "-nostdinc++",
        "-isystem",
        "platforms/fm1/libc",
        "-Isrc",
        f"-I{FM1_DIR}",
    ]

    c1_rt_cxxflags = [*common_cxxflags, "-fno-builtin", args.dsp_opt]
    c1_dsp_cxxflags = [*common_cxxflags, args.dsp_opt]

    specs: list[CompileSpec] = [
        CompileSpec(
            fw / "crt0.S",
            RAW_OBJ_DIR / "crt0.o",
            [str(clang), "-target", "pi32v2", "-mcpu=r3", "-c", str(fw / "crt0.S"), "-o", str(RAW_OBJ_DIR / "crt0.o")],
            use_depfile=False,
        ),
        CompileSpec(
            fw / "hal" / "fm1_vec.S",
            RAW_OBJ_DIR / "fm1_vec.o",
            [
                str(clang),
                "-target",
                "pi32v2",
                "-mcpu=r3",
                "-c",
                str(fw / "hal" / "fm1_vec.S"),
                "-o",
                str(RAW_OBJ_DIR / "fm1_vec.o"),
            ],
            use_depfile=False,
        ),
        CompileSpec(
            fw / "hal" / "fm1_isr.S",
            RAW_OBJ_DIR / "fm1_isr.o",
            [
                str(clang),
                "-target",
                "pi32v2",
                "-mcpu=r3",
                "-c",
                str(fw / "hal" / "fm1_isr.S"),
                "-o",
                str(RAW_OBJ_DIR / "fm1_isr.o"),
            ],
            use_depfile=False,
        ),
        CompileSpec(
            FM1_DIR / "fm1_cpu1.S",
            RAW_OBJ_DIR / "fm1_cpu1_asm.o",
            [
                str(clang),
                "-target",
                "pi32v2",
                "-mcpu=r3",
                "-c",
                str(FM1_DIR / "fm1_cpu1.S"),
                "-o",
                str(RAW_OBJ_DIR / "fm1_cpu1_asm.o"),
            ],
            use_depfile=False,
        ),
        CompileSpec(
            FM1_DIR / "fm1_cpu1.c",
            RAW_OBJ_DIR / "fm1_cpu1.o",
            [
                str(clang),
                *plat_cflags,
                "-Wextra",
                "-Werror",
                "-c",
                str(FM1_DIR / "fm1_cpu1.c"),
                "-o",
                str(RAW_OBJ_DIR / "fm1_cpu1.o"),
            ],
        ),
        CompileSpec(
            FM1_DIR / "fm1_main.c",
            RAW_OBJ_DIR / "fm1_main.o",
            [str(clang), *plat_cflags, "-c", str(FM1_DIR / "fm1_main.c"), "-o", str(RAW_OBJ_DIR / "fm1_main.o")],
        ),
        CompileSpec(
            FM1_DIR / "ruido_fm1_c1.cpp",
            RAW_OBJ_DIR / "ruido_fm1_c1.o",
            [
                str(clang),
                *c1_rt_cxxflags,
                "-c",
                str(FM1_DIR / "ruido_fm1_c1.cpp"),
                "-o",
                str(RAW_OBJ_DIR / "ruido_fm1_c1.o"),
            ],
        ),
        CompileSpec(
            FM1_DIR / "ruido_fm1_bridge.cpp",
            RAW_OBJ_DIR / "ruido_fm1_bridge.o",
            [
                str(clang),
                *common_cxxflags,
                args.ui_opt,
                "-c",
                str(FM1_DIR / "ruido_fm1_bridge.cpp"),
                "-o",
                str(RAW_OBJ_DIR / "ruido_fm1_bridge.o"),
            ],
        ),
        CompileSpec(
            FM1_DIR / "fm1_ops_screen.cpp",
            RAW_OBJ_DIR / "fm1_ops_screen.o",
            [
                str(clang),
                *common_cxxflags,
                args.ui_opt,
                "-c",
                str(FM1_DIR / "fm1_ops_screen.cpp"),
                "-o",
                str(RAW_OBJ_DIR / "fm1_ops_screen.o"),
            ],
        ),
    ]

    raw_dsp_objs: list[pathlib.Path] = []
    for s in dsp_srcs:
        out_o = RAW_OBJ_DIR / (pathlib.Path(s).stem + ".o")
        raw_dsp_objs.append(out_o)
        specs.append(CompileSpec( OUT_DIR.parents[1] / s, out_o, [str(clang), *c1_dsp_cxxflags, "-c", s, "-o", str(out_o)]))

    ui_objs: list[pathlib.Path] = []
    for s in ui_srcs:
        out_o = RAW_OBJ_DIR / (pathlib.Path(s).stem + ".o")
        ui_objs.append(out_o)
        specs.append(
            CompileSpec(OUT_DIR.parents[1] / s, out_o, [str(clang), *common_cxxflags, args.ui_opt, "-c", s, "-o", str(out_o)])
        )

    app_hits, app_built = compile_cached_jobs(specs, file_sha_cache, rebuild=args.rebuild)

    # Relocate pristine `obj/raw/*.o` Core 1 TUs into `obj/c1/*.o` (never mutates cached raw `.o` files!)
    raw_c1_cpp_objs = [RAW_OBJ_DIR / "ruido_fm1_c1.o", *raw_dsp_objs]
    c1_weak_names = collect_c1_weak_symbols(raw_c1_cpp_objs)
    c1_cpp_objs: list[pathlib.Path] = []
    for raw_o in raw_c1_cpp_objs:
        dst_o = C1_OBJ_DIR / raw_o.name
        relocate_obj_to_c1_sram(raw_o, dst_o, c1_weak_names)
        c1_cpp_objs.append(dst_o)

    all_objs = [
        RAW_OBJ_DIR / "crt0.o",
        RAW_OBJ_DIR / "fm1_vec.o",
        RAW_OBJ_DIR / "fm1_isr.o",
        RAW_OBJ_DIR / "fm1_cpu1_asm.o",
        *c1_cpp_objs,
        RAW_OBJ_DIR / "fm1_cpu1.o",
        RAW_OBJ_DIR / "fm1_main.o",
        RAW_OBJ_DIR / "fm1_ops_screen.o",
        RAW_OBJ_DIR / "ruido_fm1_bridge.o",
        *ui_objs,
    ]

    elf = OUT_DIR / "ruido.elf"
    link_cmd = [
        str(ld),
        "--gc-sections",
        "-e",
        "_start",
        "-T",
        str(FM1_DIR / "ruido_fm1.ld"),
        *[str(o) for o in all_objs],
    ]
    if compiler_rt.is_file():
        link_cmd += ["--start-group", str(compiler_rt), "--end-group"]
    link_cmd += ["-o", str(elf)]
    run_cmd(link_cmd)

    for name in ("text.bin", "ramtext.bin", "data.bin", "c1text.bin"):
        (OUT_DIR / name).unlink(missing_ok=True)

    # Run all `objcopy` + `objdump` passes concurrently across CPU cores
    extract_cmds = [
        [str(objcopy), "-O", "binary", "-j", ".text", str(elf), str(OUT_DIR / "text.bin")],
        [str(objcopy), "-O", "binary", "-j", ".ram_text", str(elf), str(OUT_DIR / "ramtext.bin")],
        [str(objcopy), "-O", "binary", "-j", ".data", str(elf), str(OUT_DIR / "data.bin")],
        [str(objcopy), "-O", "binary", "-j", ".c1_text", str(elf), str(OUT_DIR / "c1text.bin")],
        [str(objdump), "-t", str(elf)],
        [str(objdump), "-h", str(elf)],
        [str(objdump), "--mcpu=r3", "-d", str(elf)],
    ]
    with ThreadPoolExecutor(max_workers=len(extract_cmds)) as pool:
        extract_outs = list(pool.map(run_cmd, extract_cmds))
    syms, hdrs, dis = extract_outs[4], extract_outs[5], extract_outs[6]
    write_text_if_changed(OUT_DIR / "ruido.dis", dis)

    def sym_addr(sym_name: str) -> int:
        m = re.search(r"^([0-9a-f]+)\s+.*\s" + re.escape(sym_name) + r"$", syms, re.MULTILINE)
        if not m:
            raise SystemExit(f"[FM-1 FAIL] missing linker symbol {sym_name}")
        return int(m.group(1), 16)

    img = bytearray((OUT_DIR / "text.bin").read_bytes())
    for sect_file, load_sym in (
        ("ramtext.bin", "_rt_load"),
        ("data.bin", "_data_load"),
        ("c1text.bin", "_c1_load"),
    ):
        load_addr = sym_addr(load_sym)
        if load_addr % 4 != 0:
            raise SystemExit(f"[FM-1 FAIL] {load_sym} ({load_addr:#x}) is not 4-byte aligned")
        blob = (OUT_DIR / sect_file).read_bytes() if (OUT_DIR / sect_file).is_file() else b""
        if blob:
            if load_addr - APP_XIP < len(img):
                raise SystemExit(f"[FM-1 FAIL] {load_sym} ({load_addr:#x}) overlaps preceding image")
            img += b"\xff" * (load_addr - APP_XIP - len(img))
            img += blob
    img += b"\xff" * (-len(img) % 4)
    img_bytes = bytes(img)
    write_bytes_if_changed(OUT_DIR / "ruido.bin", img_bytes)

    raw_flash_head = fm1pkg_make.flash_image(img_bytes, fm1pkg_make.KEY)
    fwsc_bytes = fm1pkg_make.ufw(raw_flash_head, ota, PRODUCT_DEV)
    fwsc_path = OUT_DIR / "ruido.fwsc"
    write_bytes_if_changed(fwsc_path, fwsc_bytes)

    install_tool = dinghy_dir / "tools" / "fm1_install.py"
    if install_tool.is_file():
        write_bytes_if_changed(OUT_DIR / "fm1_install.py", install_tool.read_bytes())

    errors = audit_firmware_image(
        fwsc_path=fwsc_path,
        fwsc_bytes=fwsc_bytes,
        img=img_bytes,
        app_slot=app_slot,
        syms=syms,
        hdrs=hdrs,
        dis=dis,
        sym_addr=sym_addr,
        c1_objs=[RAW_OBJ_DIR / "fm1_cpu1_asm.o", *c1_cpp_objs],
        ui_audit_objs=[RAW_OBJ_DIR / "ruido_fm1_bridge.o", RAW_OBJ_DIR / "fm1_ops_screen.o", *ui_objs],
        cxx_std=cxx_std,
        dsp_opt=args.dsp_opt,
        ui_opt=args.ui_opt,
        cache_hits=ldr_hits + app_hits,
        cache_built=ldr_built + app_built,
    )

    if not args.skip_emu:
        errors.extend(
            run_emu_gates(
                fwsc_path=fwsc_path,
                raw_flash_head=raw_flash_head,
                img=img_bytes,
                status_addr=sym_addr("g_ruido_fm1_status"),
                bootguard_addr=sym_addr("bootguard"),
            )
        )

    if errors:
        print("=== FM-1 Gate 4.3 FAILED ===", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    if args.skip_emu:
        print("[FM-1 Build OK] Static Gate 4.1 + 4.2 + 600 KiB package audit passed (--skip-emu).")
    else:
        print("[FM-1 Gate 4.3 OK] Timed SPI1 DMA, 42-LED ΔΣ, panel/pressure, MIDI clock & USR NVS gates passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
