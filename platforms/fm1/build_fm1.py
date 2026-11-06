#!/usr/bin/env python3
"""Build Ruido for M-VAVE FM-1 (AC791N / pi32v2) and run Gate 4.1 verification.

Runs directly inside `ruido-fm1-toolchain:latest` (Linux amd64) or automatically delegates
to a single `docker run` invocation when invoked from macOS / non-toolchain hosts.

Gate 4.1 Automated Checks:
  1. Flash & SRAM Bounds: `_start == 0x02000120` (>= 0x4000, zero boot-head writes),
     `ruido.bin <= 0x8DFBC` (567.9 KiB APP_SLOT), `ruido.fwsc < 600 KiB`,
     `.data + .bss <= 96 KiB`, `.pool <= 336 KiB`, call-free `.ram_text`.
  2. Anti-Brick Pre-Boot Safety: zero `.init_array` / `.ctors` / `.dtors` sections,
     zero `0xFFC00000..0xFFCFFFFF` chip-ROM immediates or call targets.
  3. Hardware `-mcpu=r3 -mfprev1` FPU & Division Audit (`objdump --mcpu=r3 -d`):
     native single-precision FPU emission (`(f)` / `iff`), zero `soft-double` symbols,
     zero 64-bit integer division (`__udivdi3` / `__divdi3` / `__aeabi_ldivmod`),
     and exact `fdiv` vs `__divsf3` tally + per-layer `.text + .rodata` footprint.
  4. Headless `fm1-emu` SoC Verification:
     - Normal boot (`900 ms`): `RUIDO_FM1_STATUS_MAGIC`, live stereo PCM audio,
       240x240 scanline LCD frames, and measured 64-frame DSP cycle cost.
     - Power-on `Hold OCT-` USB Rescue (`--keys 0-150:OCTDN`): `bootguard.pending == 2`
       and `status.magic == 0` (zero C++ engine execution before `recovery_main()`).
"""

from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import platform
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT = pathlib.Path(__file__).resolve().parents[2]
FM1_DIR = ROOT / "platforms" / "fm1"
MAKEFILE = ROOT / "Makefile"
OUT_DIR = ROOT / "build" / "fm1"
OBJ_DIR = OUT_DIR / "obj"
LDR_DIR = OUT_DIR / "loader"

DOCKER_IMAGE = os.environ.get("RUIDO_FM1_DOCKER_IMAGE", "ruido-fm1-toolchain:latest")
APP_XIP = 0x02000120
LOADER_LOAD = 0x01C0A800
LOADER_NAME = b"usb_hid_ota.bin"
SCREEN_NAME = "RUIDO"
SCREEN_COLOUR = 0xFFFF  # Monochrome white
PRODUCT_DEV = "FM-1_990"
MAX_FWSC_BYTES = 600 * 1024

SDK_SHA256 = {
    "uboot.boot": "4e3b4c220dc96641cb5a723f41e68ce41d5261ae9434bb33fbd7f2c59976ded4",
    "cfg_tool.bin": "276579954f076886a6a7694f65dc71c034a63a2c204b76749065c0ac7b010d1b",
    "cfg/eq_cfg_hw.bin": "41167491bffed4651750719c973d2758adeb9021a5670d02d6a53c85ed80ea7d",
}

FORBIDDEN_SOFT_DOUBLE = {
    "__adddf3",
    "__subdf3",
    "__muldf3",
    "__divdf3",
    "__extendsfdf2",
    "__truncdfsf2",
    "__fixdfsi",
    "__fixunsdfsi",
    "__floatsidf",
    "__floatunsidf",
    "__eqdf2",
    "__nedf2",
    "__ltdf2",
    "__ledf2",
    "__gtdf2",
    "__gedf2",
}

FORBIDDEN_INT64_DIV = {
    "__udivdi3",
    "__divdi3",
    "__umoddi3",
    "__moddi3",
    "__udivmoddi4",
    "__divmoddi4",
    "__aeabi_ldivmod",
    "__aeabi_uldivmod",
}

DIS_LINE_RE = re.compile(r"^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2} )+)\s*\t(.*)$")


def parse_makefile_var(text: str, name: str) -> list[str]:
    pattern = re.compile(rf"^{re.escape(name)}\s*:=\s*((?:.*\\\n)*.*)$", re.MULTILINE)
    match = pattern.search(text)
    if not match:
        return []
    raw = match.group(1).replace("\\\n", " ")
    return [tok for tok in raw.split() if tok]


def have_local_toolchain() -> bool:
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        return False
    tc = pathlib.Path(os.environ.get("JIELI_TOOLCHAIN", "/opt/jieli/toolchain"))
    return (tc / "pi32v2" / "bin" / "clang").is_file()


def delegate_to_docker(argv: list[str]) -> int:
    cmd = [
        "docker",
        "run",
        "--rm",
        "--platform",
        "linux/amd64",
        "-v",
        f"{ROOT}:/work",
        "-w",
        "/work",
        DOCKER_IMAGE,
        "python3",
        "platforms/fm1/build_fm1.py",
        *argv,
    ]
    return subprocess.call(cmd, cwd=ROOT)


def run_cmd(cmd: list[str], env: dict[str, str] | None = None) -> str:
    res = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, check=False)
    if res.returncode != 0:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit(f"[FM-1 FAIL] command failed ({res.returncode}): {' '.join(cmd[:4])}...")
    return res.stdout


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


def crc16_ccitt(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def ldr_head(dcrc: int, a: int, b: int, attr: int, name: bytes) -> bytes:
    import struct

    body = struct.pack("<HIIBBH16s", dcrc, a, b, attr, 0, 0, name)
    return struct.pack("<H", crc16_ccitt(body)) + body


def build_loader(tc: pathlib.Path, dinghy_dir: pathlib.Path, lz4blk_mod) -> bytes:
    import struct

    LDR_DIR.mkdir(parents=True, exist_ok=True)
    clang = str(tc / "pi32v2" / "bin" / "clang")
    ld = str(tc / "pi32v2" / "bin" / "ld")
    objcopy = str(tc / "common" / "bin" / "objcopy")
    objdump = str(tc / "common" / "bin" / "objdump")
    fw = dinghy_dir / "firmware"
    ldr_src = fw / "loader"

    cflags = [
        "-target",
        "pi32v2",
        "-Os",
        "-ffunction-sections",
        "-fno-builtin",
        "-Wall",
        "-Wno-unused-function",
        f"-I{fw / 'hal'}",
        f"-I{fw / 'src'}",
        f'-DLDR_NAME="{SCREEN_NAME}"',
        f"-DLDR_COLOUR={SCREEN_COLOUR:#x}u",
    ]
    run_cmd([clang, "-target", "pi32v2", "-c", str(ldr_src / "crt0_ldr.S"), "-o", str(LDR_DIR / "crt0_ldr.o")])
    run_cmd([clang, *cflags, "-c", str(ldr_src / "loader.c"), "-o", str(LDR_DIR / "loader.o")])
    elf = LDR_DIR / "loader.elf"
    run_cmd([
        ld,
        "--gc-sections",
        "-e",
        "_start",
        "-T",
        str(ldr_src / "loader.ld"),
        str(LDR_DIR / "crt0_ldr.o"),
        str(LDR_DIR / "loader.o"),
        "-o",
        str(elf),
    ])
    hdr = run_cmd([objdump, "-h", str(elf)])
    for ln in hdr.splitlines():
        parts = ln.split()
        if len(parts) > 4 and parts[1].startswith(".") and int(parts[3], 16) >= 0x02000000 and int(parts[2], 16):
            raise SystemExit(f"[FM-1 FAIL] loader section {parts[1]} placed in XIP ({parts[3]})")
    ldr_bin = LDR_DIR / "loader.bin"
    run_cmd([objcopy, "-O", "binary", "-j", ".text", str(elf), str(ldr_bin)])
    image = ldr_bin.read_bytes()
    if not image or len(image) > 0x14000:
        raise SystemExit(f"[FM-1 FAIL] loader size invalid ({len(image)} B)")

    blocks = bytearray()
    for i in range(0, len(image), 4096):
        raw = image[i : i + 4096]
        comp = lz4blk_mod.compress(raw)
        if len(comp) > 4096 or lz4blk_mod.decompress(comp) != raw:
            raise SystemExit(f"[FM-1 FAIL] loader LZ4 block {i // 4096} failed round-trip")
        blocks += struct.pack("<II", len(comp), len(raw)) + comp
    inner = ldr_head(crc16_ccitt(image), len(image), LOADER_LOAD, 0, LOADER_NAME)
    body = inner + bytes(blocks)
    ota = ldr_head(crc16_ccitt(body), 0x20, len(body), 0x41, LOADER_NAME) + body
    (LDR_DIR / "ota.bin").write_bytes(ota)
    return ota


def section_bytes_of_obj(objdump: str, obj_path: pathlib.Path) -> tuple[int, int, list[str]]:
    hdr = run_cmd([objdump, "-h", str(obj_path)])
    text_b = 0
    rodata_b = 0
    bad_ctors: list[str] = []
    for ln in hdr.splitlines():
        p = ln.split()
        if len(p) > 3 and p[0].isdigit():
            sec = p[1]
            sz = int(p[2], 16)
            if sec.startswith(".text"):
                text_b += sz
            elif sec.startswith(".rodata"):
                rodata_b += sz
            if any(sec.startswith(bad) for bad in (".init_array", ".ctors", ".dtors", ".fini_array")) and sz > 0:
                bad_ctors.append(f"{obj_path.name}:{sec}({sz}B)")
    return text_b, rodata_b, bad_ctors


def parse_dump_words(emu_out: str) -> list[int]:
    words: list[int] = []
    in_dump = False
    for ln in emu_out.splitlines():
        if ln.startswith("dump "):
            in_dump = True
            continue
        if in_dump and ln.startswith("  0x"):
            _, rhs = ln.split(":", 1)
            for tok in rhs.split():
                words.append(int(tok, 16))
    return words


def main(argv: list[str]) -> int:
    if not have_local_toolchain():
        return delegate_to_docker(argv)

    ap = argparse.ArgumentParser()
    ap.add_argument("--dsp-opt", default="-O2", choices=["-Os", "-O2", "-O3"])
    ap.add_argument("--ui-opt", default="-Os", choices=["-Os", "-O2", "-O3"])
    ap.add_argument("--skip-emu", action="store_true")
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
    OBJ_DIR.mkdir(parents=True, exist_ok=True)

    ota = build_loader(tc, dinghy_dir, lz4blk)

    fw = dinghy_dir / "firmware"
    # Keep platform.c / usb.c strictly at dinghy's hardware-verified C flags (-Os, no -O2 USB babble)
    plat_cflags = [
        "-target",
        "pi32v2",
        "-Os",
        "-ffunction-sections",
        "-fno-builtin",
        "-Wall",
        "-Wno-unused-function",
        f"-I{fw / 'hal'}",
        f"-I{fw / 'src'}",
        f"-I{fw / 'gen'}",
        f"-I{FM1_DIR}",
        f'-DFELUCCA_ID="{PRODUCT_DEV}"',
        '-DAPP_VER="0.1"',
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

    compile_jobs: list[list[str]] = [
        [str(clang), "-target", "pi32v2", "-c", str(fw / "crt0.S"), "-o", str(OBJ_DIR / "crt0.o")],
        [str(clang), "-target", "pi32v2", "-c", str(fw / "hal" / "fm1_vec.S"), "-o", str(OBJ_DIR / "fm1_vec.o")],
        [str(clang), "-target", "pi32v2", "-c", str(fw / "hal" / "fm1_isr.S"), "-o", str(OBJ_DIR / "fm1_isr.o")],
        [str(clang), *plat_cflags, "-c", str(FM1_DIR / "fm1_main.c"), "-o", str(OBJ_DIR / "fm1_main.o")],
        [
            str(clang),
            *common_cxxflags,
            args.ui_opt,
            "-c",
            str(FM1_DIR / "ruido_fm1_bridge.cpp"),
            "-o",
            str(OBJ_DIR / "ruido_fm1_bridge.o"),
        ],
    ]

    dsp_objs: list[pathlib.Path] = []
    for s in dsp_srcs:
        out_o = OBJ_DIR / (pathlib.Path(s).stem + ".o")
        dsp_objs.append(out_o)
        compile_jobs.append([str(clang), *common_cxxflags, args.dsp_opt, "-c", s, "-o", str(out_o)])

    ui_objs: list[pathlib.Path] = []
    for s in ui_srcs:
        out_o = OBJ_DIR / (pathlib.Path(s).stem + ".o")
        ui_objs.append(out_o)
        compile_jobs.append([str(clang), *common_cxxflags, args.ui_opt, "-c", s, "-o", str(out_o)])

    with ThreadPoolExecutor(max_workers=min(8, len(compile_jobs))) as pool:
        list(pool.map(run_cmd, compile_jobs))

    all_objs = [
        OBJ_DIR / "crt0.o",
        OBJ_DIR / "fm1_vec.o",
        OBJ_DIR / "fm1_isr.o",
        OBJ_DIR / "fm1_main.o",
        OBJ_DIR / "ruido_fm1_bridge.o",
        *dsp_objs,
        *ui_objs,
    ]

    elf = OUT_DIR / "ruido.elf"
    link_cmd = [
        str(ld),
        "--gc-sections",
        "-e",
        "_start",
        "-T",
        str(fw / "app.ld"),
        *[str(o) for o in all_objs],
    ]
    if compiler_rt.is_file():
        link_cmd += ["--start-group", str(compiler_rt), "--end-group"]
    link_cmd += ["-o", str(elf)]
    run_cmd(link_cmd)

    # Extract sections & disassembly (--mcpu=r3 decodes 3f e5 / 3f f5 hardware FPU ops!)
    for name in ("text.bin", "data.bin", "ramtext.bin"):
        (OUT_DIR / name).unlink(missing_ok=True)
    run_cmd([str(objcopy), "-O", "binary", "-j", ".text", str(elf), str(OUT_DIR / "text.bin")])
    run_cmd([str(objcopy), "-O", "binary", "-j", ".data", str(elf), str(OUT_DIR / "data.bin")])
    run_cmd([str(objcopy), "-O", "binary", "-j", ".ram_text", str(elf), str(OUT_DIR / "ramtext.bin")])

    syms = run_cmd([str(objdump), "-t", str(elf)])
    hdrs = run_cmd([str(objdump), "-h", str(elf)])
    dis = run_cmd([str(objdump), "--mcpu=r3", "-d", str(elf)])
    rt_dis = run_cmd([str(objdump), "--mcpu=r3", "-d", "-j", ".ram_text", str(elf)])
    (OUT_DIR / "ruido.dis").write_text(dis, encoding="utf-8")

    def sym_addr(sym_name: str) -> int:
        m = re.search(r"^([0-9a-f]+)\s+.*\s" + re.escape(sym_name) + r"$", syms, re.MULTILINE)
        if not m:
            raise SystemExit(f"[FM-1 FAIL] missing linker symbol {sym_name}")
        return int(m.group(1), 16)

    img = bytearray((OUT_DIR / "text.bin").read_bytes())
    for sect_file, load_sym in (("ramtext.bin", "_rt_load"), ("data.bin", "_data_load")):
        load_addr = sym_addr(load_sym)
        if load_addr % 4 != 0:
            raise SystemExit(f"[FM-1 FAIL] {load_sym} ({load_addr:#x}) is not 4-byte aligned")
        blob = (OUT_DIR / sect_file).read_bytes() if (OUT_DIR / sect_file).is_file() else b""
        if blob:
            if load_addr - APP_XIP < len(img):
                raise SystemExit(f"[FM-1 FAIL] {load_sym} overlaps preceding image")
            img += b"\xff" * (load_addr - APP_XIP - len(img))
            img += blob
    img += b"\xff" * (-len(img) % 4)
    app_bin = OUT_DIR / "ruido.bin"
    app_bin.write_bytes(img)

    fwsc_bytes = fm1pkg_make.ufw(fm1pkg_make.flash_image(bytes(img), fm1pkg_make.KEY), ota, PRODUCT_DEV)
    fwsc_path = OUT_DIR / "ruido.fwsc"
    fwsc_path.write_bytes(fwsc_bytes)

    # --- Gate 4.1 Audit ---
    errors: list[str] = []

    if sym_addr("_start") != APP_XIP:
        errors.append(f"_start != {APP_XIP:#x}")
    if bytes(img[:4]) != bytes.fromhex("04818000"):
        errors.append(f"entry stub mismatch: {bytes(img[:4]).hex()}")
    if len(img) > app_slot:
        errors.append(f"ruido.bin ({len(img)} B) exceeds APP_SLOT ({app_slot} B)")
    if len(fwsc_bytes) >= MAX_FWSC_BYTES:
        errors.append(f"ruido.fwsc ({len(fwsc_bytes)} B) exceeds 600 KiB ceiling")

    rt_calls = [ln for ln in rt_dis.splitlines() if re.search(r"\bcall\b", ln)]
    if rt_calls:
        errors.append(f".ram_text contains call instructions: {rt_calls[:2]}")

    # Check ELF & per-TU global constructor sections
    bad_sections: list[str] = []
    for ln in hdrs.splitlines():
        p = ln.split()
        if len(p) > 3 and p[0].isdigit():
            sec, sz = p[1], int(p[2], 16)
            if any(sec.startswith(b) for b in (".init_array", ".ctors", ".dtors", ".fini_array")) and sz > 0:
                bad_sections.append(f"ELF:{sec}({sz}B)")

    dsp_text = dsp_rodata = 0
    for o in dsp_objs:
        t, r, bad = section_bytes_of_obj(str(objdump), o)
        dsp_text += t
        dsp_rodata += r
        bad_sections.extend(bad)

    ui_text = ui_rodata = 0
    for o in [OBJ_DIR / "ruido_fm1_bridge.o", *ui_objs]:
        t, r, bad = section_bytes_of_obj(str(objdump), o)
        ui_text += t
        ui_rodata += r
        bad_sections.extend(bad)

    if bad_sections:
        errors.append(f"pre-boot global constructor sections found: {bad_sections}")

    # Check chip-ROM address trap & count native hardware r3 FPU instructions
    fpu_add = fpu_sub = fpu_mul = fpu_div = fpu_cmp = fpu_conv = 0
    for ln in dis.splitlines():
        mm = DIS_LINE_RE.match(ln)
        if not mm:
            continue
        asm = mm.group(3)
        for v in re.findall(r"= (-?\d+) <|call -?\d+ <[^:>]*: ([0-9a-f]+) >", asm):
            val = (int(v[0]) & 0xFFFFFFFF) if v[0] else (int(v[1], 16) & 0xFFFFFFFF)
            if 0xFFC00000 <= val < 0xFFD00000:
                errors.append(f"chip-ROM reference {val:#010x} at {mm.group(1)}")
        if "(f)" in asm:
            if " / " in asm:
                fpu_div += 1
            elif " * " in asm:
                fpu_mul += 1
            elif " + " in asm:
                fpu_add += 1
            elif " - " in asm:
                fpu_sub += 1
            else:
                fpu_conv += 1
        elif asm.startswith("iff") or " iff" in asm:
            fpu_cmp += 1
        elif any(op in asm for op in ("ftoi", "ftou", "itof", "utof", "fmin", "fmax")):
            fpu_conv += 1

    # Check DSP-only fdiv count
    dsp_fdiv = 0
    for o in dsp_objs:
        o_dis = run_cmd([str(objdump), "--mcpu=r3", "-d", str(o)])
        for ln in o_dis.splitlines():
            mm = DIS_LINE_RE.match(ln)
            if mm and "(f)" in mm.group(3) and " / " in mm.group(3):
                dsp_fdiv += 1

    # Symbol table audit for soft-double, int64 division, libcompiler_rt __sync_* traps, and soft-float helpers
    defined_syms: set[str] = set()
    soft_single_syms: list[str] = []
    bad_sync_syms: list[str] = []
    for ln in syms.splitlines():
        parts = ln.split()
        if len(parts) >= 6:
            sym = parts[-1]
            defined_syms.add(sym)
            if sym in ("__divsf3", "__addsf3", "__subsf3", "__mulsf3"):
                soft_single_syms.append(sym)
            if sym.startswith("__sync_"):
                bad_sync_syms.append(sym)

    bad_double = sorted(defined_syms & FORBIDDEN_SOFT_DOUBLE)
    bad_i64div = sorted(defined_syms & FORBIDDEN_INT64_DIV)
    if bad_double:
        errors.append(f"forbidden soft-double symbols linked: {bad_double}")
    if bad_i64div:
        errors.append(f"forbidden 64-bit integer division symbols linked: {bad_i64div}")
    if bad_sync_syms:
        errors.append(f"forbidden libcompiler_rt __sync_* atomic helpers linked: {bad_sync_syms}")
    if (fpu_add + fpu_sub + fpu_mul) == 0:
        errors.append("zero hardware r3 FPU arithmetic instructions detected in disassembly")

    ram_bytes = sym_addr("_bss_end") - 0x01C08000
    pool_bytes = sym_addr("_pool_end") - 0x01C20000
    if ram_bytes > 96 * 1024:
        errors.append(f".data+.bss ({ram_bytes} B) > 96 KiB")
    if pool_bytes > 0x54000:
        errors.append(f".pool ({pool_bytes} B) > 336 KiB")

    print("=== Ruido FM-1 Gate 4.1 Toolchain & ISA Audit ===")
    print(f"  Compiler          : pi32v2 clang ({cxx_std}, DSP={args.dsp_opt}, UI={args.ui_opt}, -mcpu=r3 -mfprev1)")
    print(f"  App XIP (ruido.bin): {len(img):,} B / {app_slot:,} B ({100.0 * len(img) / app_slot:.1f}% of 567.9 KiB APP_SLOT)")
    print(f"  Package (.fwsc)   : {len(fwsc_bytes):,} B / {MAX_FWSC_BYTES:,} B ({len(fwsc_bytes) / 1024.0:.1f} KiB)")
    print(f"  DSP Core1 Preview : {dsp_text + dsp_rodata:,} B (.text={dsp_text:,} B + .rodata={dsp_rodata:,} B)")
    print(f"  UI + Bridge Code  : {ui_text + ui_rodata:,} B (.text={ui_text:,} B + .rodata={ui_rodata:,} B)")
    print(f"  SRAM .data + .bss : {ram_bytes:,} B / {96 * 1024:,} B ({ram_bytes / 1024.0:.1f} KiB)")
    print(f"  SRAM .pool        : {pool_bytes:,} B / {0x54000:,} B ({pool_bytes / 1024.0:.1f} KiB)")
    print(
        f"  Hardware r3 FPU   : fadd={fpu_add}, fsub={fpu_sub}, fmul={fpu_mul}, "
        f"fdiv={fpu_div} (DSP TUs={dsp_fdiv}), fcmp={fpu_cmp}, fconv/minmax={fpu_conv}"
    )
    print(f"  Soft-float calls  : {soft_single_syms if soft_single_syms else 'NONE (100% native r3 hardware FPU)'}")
    print("  Anti-Brick Checks : _start=0x02000120, 0 global ctors, 0 __sync_*, 0 soft-double, 0 int64-div, 0 ROM traps")

    if not args.skip_emu:
        status_addr = sym_addr("g_ruido_fm1_status")
        bootguard_addr = sym_addr("bootguard")
        wav_out = OUT_DIR / "emu_boot.wav"
        if wav_out.exists():
            wav_out.unlink()

        # 1. Normal boot verification (950 ms: clears 50 ms OCT- scan + ~350 ms ST7789 init + ~550 ms synth/UI)
        boot_out = run_cmd([
            "fm1-emu",
            "--fwsc",
            str(fwsc_path),
            "--ms",
            "950",
            "--wav",
            str(wav_out),
            "--dump",
            f"{status_addr:#x}",
            "8",
        ])
        sw = parse_dump_words(boot_out)
        if len(sw) < 8:
            errors.append(f"fm1-emu normal boot dump incomplete: {sw}")
        else:
            (
                magic,
                audio_blocks,
                lcd_frames,
                bench_idle_ticks,
                bench_full_ticks,
                cal_8k_ticks,
                last256_ticks,
                peak_q15,
            ) = sw[:8]
            insns_per_tick = 8000.0 / max(1, cal_8k_ticks)
            idle_insns_f = (bench_idle_ticks * insns_per_tick) / 64.0
            full_insns_f = (bench_full_ticks * insns_per_tick) / 64.0
            dma_insns_f = (last256_ticks * insns_per_tick) / 256.0
            cyc_budget_240 = 240_000_000.0 / 44_100.0
            cyc_budget_320 = 320_000_000.0 / 44_100.0
            wav_bytes = wav_out.stat().st_size if wav_out.exists() else 0
            print(
                f"  fm1-emu Boot (.95s): magic={magic:#010x}, audio_blocks={audio_blocks}, "
                f"lcd_frames={lcd_frames}, peak_q15={peak_q15}, wav={wav_bytes:,} B"
            )
            print(
                f"  DSP Idle 64f Bench: ~{idle_insns_f:.0f} insns/frame "
                f"({100.0 * idle_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
                f"{100.0 * idle_insns_f / cyc_budget_320:.1f}% @ 320 MHz; "
                f"cal={insns_per_tick:.2f} insn/tick, DMA256=~{dma_insns_f:.0f} insn/f)"
            )
            print(
                f"  DSP Full 64f Stress: ~{full_insns_f:.0f} insns/frame "
                f"({100.0 * full_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
                f"{100.0 * full_insns_f / cyc_budget_320:.1f}% @ 320 MHz; "
                f"4v HP+LP ZDF self-osc + drive + 8x LFO)"
            )
            if magic != 0x52554944:
                errors.append(f"fm1-emu normal boot did not reach ruido_fm1_init (magic={magic:#x})")
            if audio_blocks == 0 or peak_q15 == 0:
                errors.append(f"fm1-emu normal boot produced silent/zero audio (blocks={audio_blocks}, peak={peak_q15})")
            if wav_bytes <= 44:
                errors.append(f"fm1-emu did not write valid audio WAV ({wav_bytes} B)")
            if lcd_frames == 0:
                errors.append("fm1-emu normal boot rendered 0 LCD frames")

        # 2. Power-on Hold OCT- USB Rescue verification (must enter recovery_main BEFORE any C++ engine init!)
        rescue_out = run_cmd([
            "fm1-emu",
            "--fwsc",
            str(fwsc_path),
            "--ms",
            "600",
            "--quiet",
            "--keys",
            "0-150:OCTDN",
            "--dump",
            f"{bootguard_addr:#x}",
            "3",
        ])
        bw = parse_dump_words(rescue_out)
        rescue_status_out = run_cmd([
            "fm1-emu",
            "--fwsc",
            str(fwsc_path),
            "--ms",
            "250",
            "--quiet",
            "--keys",
            "0-150:OCTDN",
            "--dump",
            f"{status_addr:#x}",
            "2",
        ])
        rsw = parse_dump_words(rescue_status_out)
        if len(bw) < 3 or bw[0] != 0x42475232 or bw[2] != 2:
            errors.append(f"Hold OCT- USB rescue failed bootguard check (got {bw}, expected pending=2)")
        elif len(rsw) < 2 or rsw[0] != 0 or rsw[1] != 0:
            errors.append(f"Hold OCT- USB rescue leaked into C++ init/audio (status={rsw})")
        else:
            print(
                f"  fm1-emu OCT- Rescue: PASS (bootguard.pending={bw[2]}, "
                f"cpp_magic={rsw[0]:#x}, audio_blocks={rsw[1]})"
            )

    if errors:
        print("=== FM-1 Gate 4.1 FAILED ===", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print("[FM-1 Gate 4.1 OK] Firmware package & hardware/emulator gates passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
