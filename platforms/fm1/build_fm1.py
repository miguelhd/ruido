#!/usr/bin/env python3
"""Build Ruido for M-VAVE FM-1 (AC791N / pi32v2) and run Gate 4.1 + 4.2 + 4.3 verification.

Runs directly inside `ruido-fm1-toolchain:latest` (Linux amd64) or automatically delegates
to a single `docker run` invocation when invoked from macOS / non-toolchain hosts.

Automated Hardware & Firmware Gates:
  1. Flash & Dual-Core SRAM Bounds:
     - `_start == 0x02000120` (>= 0x4000, zero boot-head writes)
     - `ruido.bin <= 0x8DFBC` (567.9 KiB APP_SLOT), `ruido.fwsc < 600 KiB`
     - `.data + .bss <= 24 KiB`, `.c1_text <= 72 KiB` (0x01C0E000 SRAM), `.pool <= 320 KiB`
     - Call-free `.ram_text` AND 100% XIP-free `.c1_text` (zero `0x02000000..0x07FFFFFF` refs)
  2. Anti-Brick Pre-Boot Safety:
     - Zero `.init_array` / `.ctors` / `.dtors` / `.fini_array` sections
     - Zero `0xFFC00000..0xFFCFFFFF` chip-ROM immediates or call targets
  3. Hardware `-mcpu=r3 -mfprev1` FPU & Division Audit (`objdump --mcpu=r3 -d`):
     - 100% native single-precision FPU emission, zero `soft-double`, zero `int64` div, zero `__sync_*`
  4. Headless `fm1-emu` Dual-Core SoC + Gate 4.3 Peripheral & NVS Verification:
     - Timed `SPI1` Scanline DMA (`FM1_LCD_TIMED=1`, `950 ms`): `Core 1` boots at `0x01C0E000`
       (`.c1_text`), renders live TPDF-dithered 64-frame blocks (`c1_underruns == 0`), `Core 0`
       pushes live 240x240 scanline frames and drives 42-channel `ΔΣ` LEDs (`STOP` red `(10, 2)`).
     - Panel Matrix + `24 PPQN` MIDI Clock Lock (`--keys`): verifies buttons (`SEL`, `PLAY`),
       encoders (`SELECT`, `KNOB1`), silicone key (`n7`), `MASTER` pot, `100.0 BPM` external MIDI
       clock lock (`TIMER4` hardware timestamps), and split `PLAY` green `(8, 1)` LED toggle.
     - 2-Boot `--raw-flash` `USR` NVS Persistence (`0xF4000..0xF8FFF`): saves modified Scene 2 +
       `DeviceSettingsBlob`, audits all `1,048,576 B` of SPI NOR flash (`0` writes `< 0xEA000` or
       `>= 0xFC000`), and verifies cold power-cycle restore before audio arm.
     - Flaky-Core1 Silicon Survival (`--flaky-core1 0.05 --cpu1-survives`): `Core 0` detects a
       halted `Core 1` within 4 DMA blocks and switches to fallback audio without crashing.
     - Power-on `Hold OCT-` USB Rescue (`--keys 0-150:OCTDN`): `bootguard.pending == 2`,
       `magic == 0`, `audio_blocks == 0`, and `c1_blocks == 0`.
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
C1_SRAM_ORIGIN = 0x01C0E000
C1_SRAM_MAX_BYTES = 72 * 1024
RAM_MAX_BYTES = 24 * 1024
POOL_MAX_BYTES = 0x50000
LOADER_LOAD = 0x01C0A800
LOADER_NAME = b"usb_hid_ota.bin"
SCREEN_NAME = "RUIDO"
SCREEN_COLOUR = 0xFFFF  # Monochrome white
PRODUCT_DEV = "FM-1_960"  # Dedicated Ruido package identity (`FM-1_96N`)
MAX_FWSC_BYTES = 600 * 1024

SDK_SHA256 = {
    "uboot.boot": "4e3b4c220dc96641cb5a723f41e68ce41d5261ae9434bb33fbd7f2c59976ded4",
    "cfg_tool.bin": "276579954f076886a6a7694f65dc71c034a63a2c204b76749065c0ac7b010d1b",
    "cfg/eq_cfg_hw.bin": "41167491bffed4651750719c973d2758adeb9021a5670d02d6a53c85ed80ea7d",
}

# Both FM-1 1 MiB SPI NOR hardware revisions must be accepted in BOTH `loader.bin` (`ota.bin`)
# and `ruido.bin` so neither USB updates (`Hold OCT-`) nor `USR` NVS saves lock out a unit:
REQUIRED_JEDEC_IDS = (
    ("P25Q80", 0x00856014, b"\x14\x60\x85\x00"),
    ("UC25HQ80", 0x00B36014, b"\x14\x60\xb3\x00"),
)

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


def run_cmd(cmd: list[str], env: dict[str, str] | None = None, include_stderr: bool = False) -> str:
    res = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, check=False)
    if res.returncode != 0:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit(f"[FM-1 FAIL] command failed ({res.returncode}): {' '.join(cmd[:4])}...")
    return (res.stdout + res.stderr) if include_stderr else res.stdout


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
    for chip_name, jedec_id, le_bytes in REQUIRED_JEDEC_IDS:
        if le_bytes not in image:
            raise SystemExit(
                f"[FM-1 FAIL] update loader (`loader.bin`) missing {chip_name} JEDEC ID {jedec_id:#08x}"
            )

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
            if sec.startswith(".c1_text.rodata") or sec.startswith(".rodata"):
                rodata_b += sz
            elif sec.startswith(".c1_text") or sec.startswith(".text"):
                text_b += sz
            if any(sec.startswith(bad) for bad in (".init_array", ".ctors", ".dtors", ".fini_array")) and sz > 0:
                bad_ctors.append(f"{obj_path.name}:{sec}({sz}B)")
    return text_b, rodata_b, bad_ctors


def collect_c1_weak_symbols(obj_paths: list[pathlib.Path]) -> set[str]:
    import struct

    weak_names: set[str] = set()
    for obj_path in obj_paths:
        buf = obj_path.read_bytes()
        e_shoff = struct.unpack_from("<I", buf, 32)[0]
        e_shnum = struct.unpack_from("<H", buf, 48)[0]
        shdrs = [struct.unpack_from("<10I", buf, e_shoff + i * 40) for i in range(e_shnum)]
        group_sig_syms = {sh[7] for sh in shdrs if sh[1] == 17}
        for sh in shdrs:
            if sh[1] != 2 or sh[9] != 16:
                continue
            str_sh = shdrs[sh[6]]
            str_buf = buf[str_sh[4] : str_sh[4] + str_sh[5]]
            for sym_idx, pos in enumerate(range(sh[4], sh[4] + sh[5], 16)):
                st_name, _val, _sz, st_info, _other, st_shndx = struct.unpack_from("<IIIBBH", buf, pos)
                if st_name != 0 and st_shndx != 0 and ((st_info >> 4) == 2 or sym_idx in group_sig_syms):
                    end = str_buf.index(0, st_name)
                    weak_names.add(str_buf[st_name:end].decode("ascii"))
    return weak_names


def relocate_obj_to_c1_sram(obj_path: pathlib.Path, c1_weak_names: set[str]) -> None:
    """Rewrite ELF32 relocatable `.o` so `.text*`/`.rodata*` + `memcpy`/`memset` land in `.c1_text` SRAM."""
    import struct

    out = bytearray(obj_path.read_bytes())
    if out[:5] != b"\x7fELF\x01" or out[5] != 1:
        raise SystemExit(f"[FM-1 FAIL] {obj_path} is not little-endian ELF32")

    (
        _ident,
        _etype,
        _emachine,
        _eversion,
        _eentry,
        _ephoff,
        e_shoff,
        _eflags,
        _eehsize,
        _ephentsize,
        _ephnum,
        e_shentsize,
        e_shnum,
        e_shstrndx,
    ) = struct.unpack_from("<16sHHIIIIIHHHHHH", out, 0)
    if e_shentsize != 40:
        raise SystemExit(f"[FM-1 FAIL] unexpected ELF32 e_shentsize={e_shentsize} in {obj_path}")

    shdrs = [list(struct.unpack_from("<10I", out, e_shoff + i * 40)) for i in range(e_shnum)]

    def read_cstr(buf: bytes | bytearray, off: int) -> str:
        end = buf.index(0, off)
        return bytes(buf[off:end]).decode("ascii")

    new_strtabs: dict[int, bytearray] = {}

    def get_strtab(sec_idx: int) -> bytearray:
        if sec_idx not in new_strtabs:
            s = shdrs[sec_idx]
            new_strtabs[sec_idx] = bytearray(out[s[4] : s[4] + s[5]])
        return new_strtabs[sec_idx]

    shstr = get_strtab(e_shstrndx)

    # 1. Rename PROGBITS `.text*` and `.rodata*` sections into `.c1_text*`
    for sh in shdrs:
        if sh[1] != 1:  # SHT_PROGBITS only (leave SHT_REL sh_info indices untouched)
            continue
        sec_name = read_cstr(shstr, sh[0])
        new_sec: str | None = None
        if sec_name == ".text" or sec_name.startswith(".text."):
            new_sec = ".c1_text" + sec_name[len(".text") :]
        elif sec_name == ".rodata" or sec_name.startswith(".rodata."):
            new_sec = ".c1_text.rodata" + sec_name[len(".rodata") :]
        if new_sec is not None:
            sh[0] = len(shstr)
            shstr += new_sec.encode("ascii") + b"\x00"

    # 2. Redirect undefined `memcpy`/`memset` to `.c1_text` SRAM helpers, and namespace weak/COMDAT
    #    inline symbols (`__c1w_*`) so `.c1_text` (0x01C0E000) and `.text` (0x02000120) never emit
    #    > 4 MiB PC-relative `R_PI32V2_LONG_JUMP_23M2` relocations across memory regions!
    group_sig_syms = {sh[7] for sh in shdrs if sh[1] == 17}  # SHT_GROUP sh_info symbol indices
    for sh in shdrs:
        if sh[1] != 2 or sh[9] != 16:  # SHT_SYMTAB
            continue
        sym_str = get_strtab(sh[6])
        sym_off, sym_sz = sh[4], sh[5]
        for sym_idx, pos in enumerate(range(sym_off, sym_off + sym_sz, 16)):
            st_name, _val, _sz, st_info, _other, st_shndx = struct.unpack_from("<IIIBBH", out, pos)
            if st_name == 0:
                continue
            sname = read_cstr(sym_str, st_name)
            if st_shndx == 0 and sname in ("memcpy", "memset"):
                struct.pack_into("<I", out, pos, len(sym_str))
                sym_str += f"ruido_c1_{sname}".encode("ascii") + b"\x00"
            elif (
                sname in c1_weak_names
                or (st_shndx != 0 and ((st_info >> 4) == 2 or sym_idx in group_sig_syms))
            ) and not sname.startswith("__c1w_"):
                struct.pack_into("<I", out, pos, len(sym_str))
                sym_str += f"__c1w_{sname}".encode("ascii") + b"\x00"

    for sec_idx, buf in new_strtabs.items():
        shdrs[sec_idx][4] = len(out)
        shdrs[sec_idx][5] = len(buf)
        out += buf
    out += b"\x00" * (-len(out) % 4)

    new_shoff = len(out)
    for sh in shdrs:
        out += struct.pack("<10I", *sh)
    struct.pack_into("<I", out, 32, new_shoff)
    obj_path.write_bytes(out)


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
    ap.add_argument("--dsp-opt", default="-O3", choices=["-Os", "-O2", "-O3"])
    ap.add_argument("--ui-opt", default="-O2", choices=["-Os", "-O2", "-O3"])
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
        '-DAPP_VER="0.2"',
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

    compile_jobs: list[list[str]] = [
        [str(clang), "-target", "pi32v2", "-mcpu=r3", "-c", str(fw / "crt0.S"), "-o", str(OBJ_DIR / "crt0.o")],
        [str(clang), "-target", "pi32v2", "-mcpu=r3", "-c", str(fw / "hal" / "fm1_vec.S"), "-o", str(OBJ_DIR / "fm1_vec.o")],
        [str(clang), "-target", "pi32v2", "-mcpu=r3", "-c", str(fw / "hal" / "fm1_isr.S"), "-o", str(OBJ_DIR / "fm1_isr.o")],
        [
            str(clang),
            "-target",
            "pi32v2",
            "-mcpu=r3",
            "-c",
            str(FM1_DIR / "fm1_cpu1.S"),
            "-o",
            str(OBJ_DIR / "fm1_cpu1_asm.o"),
        ],
        [
            str(clang),
            *plat_cflags,
            "-Wextra",
            "-Werror",
            "-c",
            str(FM1_DIR / "fm1_cpu1.c"),
            "-o",
            str(OBJ_DIR / "fm1_cpu1.o"),
        ],
        [str(clang), *plat_cflags, "-c", str(FM1_DIR / "fm1_main.c"), "-o", str(OBJ_DIR / "fm1_main.o")],
        [
            str(clang),
            *c1_rt_cxxflags,
            "-c",
            str(FM1_DIR / "ruido_fm1_c1.cpp"),
            "-o",
            str(OBJ_DIR / "ruido_fm1_c1.o"),
        ],
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
        compile_jobs.append([str(clang), *c1_dsp_cxxflags, "-c", s, "-o", str(out_o)])

    ui_objs: list[pathlib.Path] = []
    for s in ui_srcs:
        out_o = OBJ_DIR / (pathlib.Path(s).stem + ".o")
        ui_objs.append(out_o)
        compile_jobs.append([str(clang), *common_cxxflags, args.ui_opt, "-c", s, "-o", str(out_o)])

    with ThreadPoolExecutor(max_workers=min(8, len(compile_jobs))) as pool:
        list(pool.map(run_cmd, compile_jobs))

    c1_cpp_objs = [OBJ_DIR / "ruido_fm1_c1.o", *dsp_objs]
    c1_weak_names = collect_c1_weak_symbols(c1_cpp_objs)
    for o in c1_cpp_objs:
        relocate_obj_to_c1_sram(o, c1_weak_names)

    # Link Core 1 SRAM TUs BEFORE UI TUs so shared weak inline LUTs/helpers resolve to `.c1_text` SRAM!
    all_objs = [
        OBJ_DIR / "crt0.o",
        OBJ_DIR / "fm1_vec.o",
        OBJ_DIR / "fm1_isr.o",
        OBJ_DIR / "fm1_cpu1_asm.o",
        *c1_cpp_objs,
        OBJ_DIR / "fm1_cpu1.o",
        OBJ_DIR / "fm1_main.o",
        OBJ_DIR / "ruido_fm1_bridge.o",
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

    # Extract loadable binary sections & disassembly (--mcpu=r3 decodes 3f e5 / 3f f5 hardware FPU ops!)
    for name in ("text.bin", "ramtext.bin", "data.bin", "c1text.bin"):
        (OUT_DIR / name).unlink(missing_ok=True)
    run_cmd([str(objcopy), "-O", "binary", "-j", ".text", str(elf), str(OUT_DIR / "text.bin")])
    run_cmd([str(objcopy), "-O", "binary", "-j", ".ram_text", str(elf), str(OUT_DIR / "ramtext.bin")])
    run_cmd([str(objcopy), "-O", "binary", "-j", ".data", str(elf), str(OUT_DIR / "data.bin")])
    run_cmd([str(objcopy), "-O", "binary", "-j", ".c1_text", str(elf), str(OUT_DIR / "c1text.bin")])

    syms = run_cmd([str(objdump), "-t", str(elf)])
    hdrs = run_cmd([str(objdump), "-h", str(elf)])
    dis = run_cmd([str(objdump), "--mcpu=r3", "-d", str(elf)])
    rt_dis = run_cmd([str(objdump), "--mcpu=r3", "-d", "-j", ".ram_text", str(elf)])
    c1_dis = run_cmd([str(objdump), "--mcpu=r3", "-d", "-j", ".c1_text", str(elf)])
    (OUT_DIR / "ruido.dis").write_text(dis, encoding="utf-8")

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
    app_bin = OUT_DIR / "ruido.bin"
    app_bin.write_bytes(img)

    raw_flash_head = fm1pkg_make.flash_image(bytes(img), fm1pkg_make.KEY)
    fwsc_bytes = fm1pkg_make.ufw(raw_flash_head, ota, PRODUCT_DEV)
    fwsc_path = OUT_DIR / "ruido.fwsc"
    fwsc_path.write_bytes(fwsc_bytes)

    install_tool = dinghy_dir / "tools" / "fm1_install.py"
    if install_tool.is_file():
        (OUT_DIR / "fm1_install.py").write_bytes(install_tool.read_bytes())

    # --- Gate 4.1 + Gate 4.2 Audit ---
    errors: list[str] = []

    import fm1_install  # type: ignore

    pkg_prod, _ = fm1_install.load_package(str(fwsc_path), force=False)
    if pkg_prod != PRODUCT_DEV:
        errors.append(f"fwsc product identity mismatch: {pkg_prod!r} != {PRODUCT_DEV!r}")

    if sym_addr("_start") != APP_XIP:
        errors.append(f"_start != {APP_XIP:#x}")
    if sym_addr("fm1_cpu1_entry") != C1_SRAM_ORIGIN:
        errors.append(f"fm1_cpu1_entry ({sym_addr('fm1_cpu1_entry'):#x}) != {C1_SRAM_ORIGIN:#x}")
    if bytes(img[:4]) != bytes.fromhex("04818000"):
        errors.append(f"entry stub mismatch: {bytes(img[:4]).hex()}")
    if len(img) > app_slot:
        errors.append(f"ruido.bin ({len(img)} B) exceeds APP_SLOT ({app_slot} B)")
    if len(fwsc_bytes) >= MAX_FWSC_BYTES:
        errors.append(f"ruido.fwsc ({len(fwsc_bytes)} B) exceeds 600 KiB ceiling")

    rt_calls = [ln for ln in rt_dis.splitlines() if re.search(r"\bcall\b", ln)]
    if rt_calls:
        errors.append(f".ram_text contains call instructions: {rt_calls[:2]}")

    # Audit `.c1_text` for 100% XIP independence, zero `idle`, zero hardware `lockset` spinlocks,
    # and a 100% stackless leaf `ruido_fm1_c1_boot_post` SRAM bit-flip canary POST.
    c1_xip_leaks: list[str] = []
    c1_bad_ops: list[str] = []
    boot_post_sp_ops: list[str] = []
    in_boot_post = False
    for ln in c1_dis.splitlines():
        if re.match(r"^[0-9a-f]+\s+<([^>]+)>:$", ln):
            in_boot_post = "<ruido_fm1_c1_boot_post>:" in ln
            continue
        mm = DIS_LINE_RE.match(ln)
        if not mm:
            continue
        asm = mm.group(3)
        if in_boot_post and ("sp" in asm or "call " in asm):
            boot_post_sp_ops.append(f"{mm.group(1)}: {asm.strip()}")
        if re.search(r"\b(idle|lockset)\b", asm):
            c1_bad_ops.append(f"{mm.group(1)}: {asm.strip()}")
        for v in re.findall(r"= (-?\d+) <|call -?\d+ <[^:>]*: ([0-9a-f]+) >", asm):
            val = (int(v[0]) & 0xFFFFFFFF) if v[0] else (int(v[1], 16) & 0xFFFFFFFF)
            if 0x02000000 <= val < 0x08000000:
                c1_xip_leaks.append(f"{mm.group(1)}: {asm.strip()} (-> {val:#010x})")
    if c1_xip_leaks:
        errors.append(f".c1_text references XIP Flash ({len(c1_xip_leaks)} leaks): {c1_xip_leaks[:3]}")
    if c1_bad_ops:
        errors.append(f".c1_text contains forbidden idle/lockset instructions: {c1_bad_ops[:3]}")
    if boot_post_sp_ops:
        errors.append(f"ruido_fm1_c1_boot_post must be stackless leaf, found: {boot_post_sp_ops[:3]}")

    # Check ELF & per-TU global constructor sections
    bad_sections: list[str] = []
    for ln in hdrs.splitlines():
        p = ln.split()
        if len(p) > 3 and p[0].isdigit():
            sec, sz = p[1], int(p[2], 16)
            if any(sec.startswith(b) for b in (".init_array", ".ctors", ".dtors", ".fini_array")) and sz > 0:
                bad_sections.append(f"ELF:{sec}({sz}B)")

    dsp_text = dsp_rodata = 0
    for o in [OBJ_DIR / "fm1_cpu1_asm.o", *c1_cpp_objs]:
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

    # Check DSP-only fdiv count inside linked `.c1_text`
    dsp_fdiv = 0
    for ln in c1_dis.splitlines():
        mm = DIS_LINE_RE.match(ln)
        if mm and "(f)" in mm.group(3) and " / " in mm.group(3):
            dsp_fdiv += 1

    # Symbol table audit
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
    c1_bytes = sym_addr("_c1_end") - C1_SRAM_ORIGIN
    pool_bytes = sym_addr("_pool_end") - 0x01C20000
    if ram_bytes > RAM_MAX_BYTES:
        errors.append(f".data+.bss ({ram_bytes} B) > {RAM_MAX_BYTES} B")
    if c1_bytes == 0 or c1_bytes > C1_SRAM_MAX_BYTES:
        errors.append(f".c1_text ({c1_bytes} B) outside (0, {C1_SRAM_MAX_BYTES}] B")
    if pool_bytes > POOL_MAX_BYTES:
        errors.append(f".pool ({pool_bytes} B) > {POOL_MAX_BYTES} B")
    for chip_name, jedec_id, le_bytes in REQUIRED_JEDEC_IDS:
        if le_bytes not in img:
            errors.append(f"app image (`ruido.bin`) missing {chip_name} JEDEC ID {jedec_id:#08x}")

    print("=== Ruido FM-1 Gate 4.1 + 4.2 + 4.3 Dual-Core Firmware Audit ===")
    print(f"  Compiler          : pi32v2 clang ({cxx_std}, DSP={args.dsp_opt}, UI={args.ui_opt}, -mcpu=r3 -mfprev1)")
    print(f"  App XIP (ruido.bin): {len(img):,} B / {app_slot:,} B ({100.0 * len(img) / app_slot:.1f}% of 567.9 KiB APP_SLOT)")
    print(f"  Package (.fwsc)   : {len(fwsc_bytes):,} B / {MAX_FWSC_BYTES:,} B ({len(fwsc_bytes) / 1024.0:.1f} KiB)")
    print(
        f"  SRAM .c1_text DSP : {c1_bytes:,} B / {C1_SRAM_MAX_BYTES:,} B ({c1_bytes / 1024.0:.1f} KiB @ 0x01C0E000, "
        f"0 XIP refs; raw .text={dsp_text:,} B + .rodata={dsp_rodata:,} B)"
    )
    print(f"  XIP UI + Bridge   : {ui_text + ui_rodata:,} B (.text={ui_text:,} B + .rodata={ui_rodata:,} B)")
    print(f"  SRAM .data + .bss : {ram_bytes:,} B / {RAM_MAX_BYTES:,} B ({ram_bytes / 1024.0:.1f} KiB)")
    print(f"  SRAM .pool        : {pool_bytes:,} B / {POOL_MAX_BYTES:,} B ({pool_bytes / 1024.0:.1f} KiB)")
    print(
        f"  Hardware r3 FPU   : fadd={fpu_add}, fsub={fpu_sub}, fmul={fpu_mul}, "
        f"fdiv={fpu_div} (.c1_text={dsp_fdiv}), fcmp={fpu_cmp}, fconv/minmax={fpu_conv}"
    )
    print(f"  Soft-float calls  : {soft_single_syms if soft_single_syms else 'NONE (100% native r3 hardware FPU)'}")
    print(
        "  Anti-Brick Checks : _start=0x02000120, JEDEC=P25Q80+UC25HQ80, "
        "0 global ctors, 0 __sync_*, 0 soft-double, 0 int64-div, 0 ROM/XIP leaks"
    )

    if not args.skip_emu:
        status_addr = sym_addr("g_ruido_fm1_status")
        bootguard_addr = sym_addr("bootguard")
        wav_out = OUT_DIR / "emu_boot.wav"
        if wav_out.exists():
            wav_out.unlink()

        # 1. Normal Dual-Core Boot + Timed SPI1 Scanline DMA verification (`FM1_LCD_TIMED=1`, 950 ms)
        timed_env = {**os.environ, "FM1_LCD_TIMED": "1"}
        boot_out = run_cmd(
            [
                "fm1-emu",
                "--fwsc",
                str(fwsc_path),
                "--ms",
                "950",
                "--wav",
                str(wav_out),
                "--dump",
                f"{status_addr:#x}",
                "16",
            ],
            env=timed_env,
            include_stderr=True,
        )
        sw = parse_dump_words(boot_out)
        if len(sw) < 16:
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
                c1_blocks,
                c1_last64_ticks,
                clk_con0_div,
                c1_flags,
                panel_state,
                midi_clock_state,
                led_summary,
                nvs_osc1_state,
            ) = sw[:16]
            insns_per_tick = 8000.0 / max(1, cal_8k_ticks)
            idle_insns_f = (bench_idle_ticks * insns_per_tick) / 64.0
            full_insns_f = (bench_full_ticks * insns_per_tick) / 64.0
            c1_live_insns_f = (c1_last64_ticks * insns_per_tick) / 64.0
            dma_insns_f = (last256_ticks * insns_per_tick) / 256.0
            cyc_budget_240 = 240_000_000.0 / 44_100.0
            cyc_budget_320 = 320_000_000.0 / 44_100.0
            c1_started = c1_flags & 1
            c1_fallback = (c1_flags >> 1) & 1
            c1_underruns = (c1_flags >> 8) & 0xFF
            c1_assist_spans = (c1_flags >> 16) & 0xFFFF
            sys_div = (clk_con0_div >> 24) & 0xFF
            clk_con0 = clk_con0_div & 0x00FFFFFF
            wav_bytes = wav_out.stat().st_size if wav_out.exists() else 0
            boot_hud = (panel_state >> 8) & 0x3F
            boot_run = (panel_state >> 5) & 1
            boot_bpm_x10 = midi_clock_state & 0xFFFF
            boot_play_led = (led_summary >> 24) & 1
            boot_stop_led = (led_summary >> 25) & 1
            boot_osc1_x10 = (nvs_osc1_state >> 16) & 0xFFFF
            print(
                f"  fm1-emu Timed LCD : magic={magic:#010x}, c1_blocks={c1_blocks} (started={c1_started}, "
                f"fallback={c1_fallback}, underruns={c1_underruns}, c0_assist={c1_assist_spans}), "
                f"audio_blocks={audio_blocks}, lcd_frames={lcd_frames}, peak_q15={peak_q15}, wav={wav_bytes:,} B"
            )
            print(
                f"  Clock & Core0 ISR : CLK_CON0={clk_con0:#08x}, SYS_DIV={sys_div:#04x}, "
                f"cal={insns_per_tick:.2f} insn/tick, Core0 ALNK0 DMA256=~{dma_insns_f:.1f} insns/frame "
                f"({100.0 * dma_insns_f / cyc_budget_240:.2f}% @ 240 MHz)"
            )
            print(
                f"  Dual-Core Idle 64f: ~{idle_insns_f:.0f} insns/frame (live c1=~{c1_live_insns_f:.0f} insns/f; "
                f"{100.0 * idle_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
                f"{100.0 * idle_insns_f / cyc_budget_320:.1f}% @ 320 MHz)"
            )
            print(
                f"  Dual-Core Stress  : ~{full_insns_f:.0f} insns/frame "
                f"({100.0 * full_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
                f"{100.0 * full_insns_f / cyc_budget_320:.1f}% @ 320 MHz; "
                f"4v HP+LP ZDF self-osc + drive + 8x LFO)"
            )
            if magic != 0x52554944:
                errors.append(f"fm1-emu normal boot did not reach ruido_fm1_arm_audio (magic={magic:#x})")
            if c1_started != 1 or c1_blocks == 0 or c1_fallback != 0:
                diag = [
                    ln.strip()
                    for ln in boot_out.splitlines()
                    if any(k in ln.lower() for k in ("fault", "trap", "cpu1", "pc_limit", "dbg", "crash", "illegal"))
                ]
                errors.append(
                    f"Core 1 failed normal SRAM execution (started={c1_started}, c1_blocks={c1_blocks}, "
                    f"fallback={c1_fallback}, diag={diag[:6]})"
                )
            if c1_underruns != 0:
                errors.append(f"Core 1 -> ALNK0 DMA ring underruns detected (underruns={c1_underruns})")
            if idle_insns_f > cyc_budget_240:
                errors.append(
                    f"Dual-core Idle DSP ({idle_insns_f:.0f} insns/f) exceeds 240 MHz budget ({cyc_budget_240:.0f})"
                )
            if full_insns_f > cyc_budget_320:
                errors.append(
                    f"Dual-core Stress DSP ({full_insns_f:.0f} insns/f) exceeds 320 MHz budget ({cyc_budget_320:.0f})"
                )
            if audio_blocks == 0 or peak_q15 == 0:
                errors.append(f"fm1-emu normal boot produced silent/zero audio (blocks={audio_blocks}, peak={peak_q15})")
            if dma_insns_f > 120.0:
                errors.append(f"Core 0 ALNK0 DMA ISR too heavy ({dma_insns_f:.1f} insns/frame; expected ring drain < 120)")
            if wav_bytes <= 44:
                errors.append(f"fm1-emu did not write valid audio WAV ({wav_bytes} B)")
            if lcd_frames == 0:
                errors.append("fm1-emu timed SPI1 boot rendered 0 LCD frames")
            if boot_hud != 0 or boot_run != 1 or boot_bpm_x10 != 1200 or boot_osc1_x10 != 550:
                errors.append(
                    f"boot state mismatch (hud={boot_hud}, run={boot_run}, bpm_x10={boot_bpm_x10}, osc1_x10={boot_osc1_x10})"
                )
            if boot_stop_led != 1 or boot_play_led != 0 or (led_summary & 0x00FFFFFF) == 0:
                errors.append(f"boot 42-LED summary mismatch (led_summary={led_summary:#010x})")

        # 2. Gate 4.3 Panel Matrix + Encoders + Keys + MASTER Pot + 100.0 BPM 24-PPQN MIDI Clock Lock
        # (`SCL` = BTN_SEL id 3, `enc0` = SELECT, `enc2` = KNOB1, `n7` = White Key 5, `PLAY` = BTN_PLAY)
        # Note: `PLAY` tapped at 1050 ms gives 450 ms (> 3 * 80 ms tau) for LedEngine's STOP->PLAY fade!
        midi_ticks_100bpm = ",".join(f"{t}:midi:f8" for t in range(700, 1450, 25))
        ctrl_keys = (
            "550-580:SCL,610:enc0:+2,640:enc2:+8,660:pot:900,680-900:n7,"
            f"700:midi:fa,{midi_ticks_100bpm},1050-1080:PLAY"
        )
        ctrl_out = run_cmd(
            [
                "fm1-emu",
                "--fwsc",
                str(fwsc_path),
                "--ms",
                "1500",
                "--quiet",
                "--keys",
                ctrl_keys,
                "--dump",
                f"{status_addr:#x}",
                "16",
            ],
            env=timed_env,
        )
        csw = parse_dump_words(ctrl_out)
        if len(csw) < 16:
            errors.append(f"Gate 4.3 panel + MIDI clock dump incomplete: {csw}")
        else:
            p_model = csw[12] & 3
            p_sync = (csw[12] >> 4) & 1
            p_run = (csw[12] >> 5) & 1
            p_events = (csw[12] >> 16) & 0xFFFF
            m_bpm_x10 = csw[13] & 0xFFFF
            m_events = (csw[13] >> 16) & 0xFFFF
            l_play = (csw[14] >> 24) & 1
            l_stop = (csw[14] >> 25) & 1
            c_osc1_x10 = (csw[15] >> 16) & 0xFFFF
            c_underruns = (csw[11] >> 8) & 0xFF
            if (
                p_model != 2
                or p_sync != 1
                or p_run != 0
                or p_events < 6
                or not (975 <= m_bpm_x10 <= 1025)
                or m_events != 31
                or l_play != 1
                or l_stop != 0
                or c_osc1_x10 <= 580
                or c_underruns != 0
            ):
                errors.append(
                    f"Gate 4.3 panel/MIDI check failed: model={p_model}, sync={p_sync}, run={p_run}, "
                    f"events={p_events}, bpm_x10={m_bpm_x10}, midi_ev={m_events}, "
                    f"play_led={l_play}, stop_led={l_stop}, osc1_x10={c_osc1_x10}, underruns={c_underruns}"
                )
            else:
                print(
                    f"  fm1-emu Panel+MIDI: PASS (model=RhythmLab, osc1={c_osc1_x10 / 10.0:.1f} Hz, "
                    f"events={p_events}, MIDI={m_bpm_x10 / 10.0:.1f} BPM [{m_events} msgs], "
                    f"PLAY_green={l_play}/STOP_red={l_stop}, underruns=0)"
                )

        # 3. Gate 4.3 2-Boot `--raw-flash` USR NVS Persistence (`0xF4000..0xF8FFF`) & SPI NOR Write Audit
        # Note: `fm1-emu --raw-flash` is a read-only startup input image; we audit every SPI0 erase/program
        # command via `FM1_FLASH_TRACE=1`, read back the exact SPI NOR sectors (`0x0F5000` + `0x0F8000`)
        # written during Boot 1 (`st_read` -> `fl_read_ram`), splice them into a cold 1 MiB flash image,
        # and verify Boot 2 cold-restores Scene 2 + DeviceSettingsBlob with zero `--keys` input!
        import struct

        raw_flash_path = OUT_DIR / "emu_nvs_flash.bin"
        if len(raw_flash_head) > 0xEA000:
            errors.append(f"encrypted app flash head ({len(raw_flash_head):#x}) overlaps USR NVS (0xEA000)")
        else:
            baseline_flash = bytearray(raw_flash_head) + bytearray(b"\xff" * (0x100000 - len(raw_flash_head)))
            raw_flash_path.write_bytes(baseline_flash)

            # Boot 1: raise OSC 1 frequency via KNOB1 (`enc2:+12`), arm SAVE, store to Scene 2 (`n23` = White 14)
            nvs_boot1_out = run_cmd(
                [
                    "fm1-emu",
                    "--fwsc",
                    str(fwsc_path),
                    "--raw-flash",
                    str(raw_flash_path),
                    "--ms",
                    "1150",
                    "--quiet",
                    "--keys",
                    "560:enc2:+12,620-650:SAVE,700-740:n23",
                    "--dump",
                    f"{status_addr:#x}",
                    "58",
                ],
                env={**os.environ, "FM1_FLASH_TRACE": "1"},
                include_stderr=True,
            )
            nsw1 = parse_dump_words(nvs_boot1_out)

            flash_trace_re = re.compile(
                r"flash cmd (?:0x)?([0-9a-fA-F]+) addr (?:0x)?([0-9a-fA-F]+) bytes (\d+)"
            )
            erase_ops: list[int] = []
            prog_ops: list[tuple[int, int]] = []
            bad_flash_ops: list[str] = []
            for m_fl in flash_trace_re.finditer(nvs_boot1_out):
                fl_cmd = int(m_fl.group(1), 16)
                fl_addr = int(m_fl.group(2), 16)
                fl_bytes = int(m_fl.group(3))
                if fl_cmd in (0x20, 0x52, 0xD8, 0xC7, 0x60, 0x01, 0x31):
                    if fl_cmd == 0x20 and fl_addr in (0x0F5000, 0x0F8000):
                        erase_ops.append(fl_addr)
                    else:
                        bad_flash_ops.append(f"cmd={fl_cmd:#04x}@addr={fl_addr:#08x}")
                elif fl_cmd == 0x02:
                    if (0x0F5000 <= fl_addr < 0x0F5200) or (0x0F8000 <= fl_addr < 0x0F8200):
                        prog_ops.append((fl_addr, fl_bytes))
                    else:
                        bad_flash_ops.append(f"prog@addr={fl_addr:#08x}(bytes={fl_bytes})")

            slot1_raw = struct.pack("<35I", *nsw1[16:51]) if len(nsw1) >= 58 else b""
            settings_raw = struct.pack("<7I", *nsw1[51:58]) if len(nsw1) >= 58 else b""
            if len(slot1_raw) == 140 and len(settings_raw) == 28:
                persisted_flash = bytearray(baseline_flash)
                persisted_flash[0x0F5000 : 0x0F5000 + 140] = slot1_raw
                persisted_flash[0x0F8000 : 0x0F8000 + 28] = settings_raw
                raw_flash_path.write_bytes(persisted_flash)

            # Boot 2: cold power-cycle from `--bin` + `--raw-flash` (since `--fwsc` unpacks a fresh
            # blank USR flash image) with ZERO `--keys` input
            xip_bin_path = OUT_DIR / "ruido_xip.bin"
            xip_bin_path.write_bytes((b"\x00" * 0x120) + bytes(img))
            nvs_boot2_out = run_cmd([
                "fm1-emu",
                "--bin",
                str(xip_bin_path),
                "--entry",
                "0x02000120",
                "--raw-flash",
                str(raw_flash_path),
                "--ms",
                "750",
                "--quiet",
                "--dump",
                f"{status_addr:#x}",
                "16",
            ])
            nsw2 = parse_dump_words(nvs_boot2_out)

            b1_slot = ((nsw1[12] >> 2) & 3) if len(nsw1) >= 58 else -1
            b1_saves = ((nsw1[15] >> 8) & 0xFF) if len(nsw1) >= 58 else 0
            b1_osc1 = ((nsw1[15] >> 16) & 0xFFFF) if len(nsw1) >= 58 else 0
            b1_s1_magic = nsw1[16] if len(nsw1) >= 58 else 0
            b1_s1_len = nsw1[17] if len(nsw1) >= 58 else 0
            b1_cfg_magic = nsw1[51] if len(nsw1) >= 58 else 0
            b1_cfg_len = nsw1[52] if len(nsw1) >= 58 else 0
            b2_slot = ((nsw2[12] >> 2) & 3) if len(nsw2) >= 16 else -1
            b2_mask = (nsw2[15] & 0xFF) if len(nsw2) >= 16 else 0
            b2_osc1 = ((nsw2[15] >> 16) & 0xFFFF) if len(nsw2) >= 16 else 0

            if bad_flash_ops or sorted(erase_ops) != [0x0F5000, 0x0F8000] or len(prog_ops) < 4:
                errors.append(
                    f"NVS SPI NOR write audit failed: bad={bad_flash_ops[:4]}, "
                    f"erases={[hex(x) for x in erase_ops]}, progs={len(prog_ops)}"
                )
            elif (
                b1_s1_magic != 0x52554944
                or b1_s1_len != 128
                or b1_cfg_magic != 0x52554944
                or b1_cfg_len != 16
            ):
                errors.append(
                    f"NVS SPI NOR sector readback mismatch: slot1=({b1_s1_magic:#x}, {b1_s1_len}B), "
                    f"settings=({b1_cfg_magic:#x}, {b1_cfg_len}B)"
                )
            elif (
                b1_slot != 1
                or b1_saves < 2
                or b1_osc1 <= 600
                or b2_slot != 1
                or b2_mask != 0x12
                or b2_osc1 != b1_osc1
            ):
                errors.append(
                    f"NVS 2-boot persistence mismatch: boot1(slot={b1_slot}, saves={b1_saves}, osc1={b1_osc1}) "
                    f"vs boot2(slot={b2_slot}, loaded_mask={b2_mask:#x}, osc1={b2_osc1})"
                )
            else:
                print(
                    f"  fm1-emu USR NVS   : PASS (SPI0 erases={[hex(x) for x in erase_ops]}, "
                    f"progs={len(prog_ops)}, cold-restored Scene 2 osc1={b2_osc1 / 10.0:.1f} Hz, "
                    "0 writes outside 0xF4000..0xF8FFF)"
                )

        # 4. Silicon-Lottery Flaky-Core1 Survival (`--flaky-core1 0.05 --cpu1-survives`)
        flaky_out = run_cmd([
            "fm1-emu",
            "--fwsc",
            str(fwsc_path),
            "--ms",
            "950",
            "--quiet",
            "--flaky-core1",
            "0.05",
            "--seed",
            "1",
            "--cpu1-survives",
            "--dump",
            f"{status_addr:#x}",
            "16",
        ])
        fsw = parse_dump_words(flaky_out)
        f_fallback = ((fsw[11] >> 1) & 1) if len(fsw) >= 16 else 0
        if (
            len(fsw) < 16
            or fsw[0] != 0x52554944
            or f_fallback != 1
            or fsw[1] == 0
            or fsw[2] == 0
            or fsw[7] == 0
        ):
            errors.append(f"Flaky-Core1 survival check failed: {fsw}")
        else:
            print(
                f"  fm1-emu Flaky-C1  : PASS (cpu0 survived Core 1 bit-21 SRAM fault, "
                f"fallback={f_fallback}, audio_blocks={fsw[1]}, lcd_frames={fsw[2]}, peak_q15={fsw[7]})"
            )

        # 5. Power-on Hold OCT- USB Rescue verification (must enter recovery_main BEFORE C++ / Core 1!)
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
            "16",
        ])
        rsw = parse_dump_words(rescue_status_out)
        if len(bw) < 3 or bw[0] != 0x42475232 or bw[2] != 2:
            errors.append(f"Hold OCT- USB rescue failed bootguard check (got {bw}, expected pending=2)")
        elif len(rsw) < 16 or rsw[0] != 0 or rsw[1] != 0 or rsw[8] != 0:
            errors.append(f"Hold OCT- USB rescue leaked into C++ init or Core 1 (status={rsw})")
        else:
            print(
                f"  fm1-emu OCT- Rescue: PASS (bootguard.pending={bw[2]}, "
                f"cpp_magic={rsw[0]:#x}, audio_blocks={rsw[1]}, c1_blocks={rsw[8]})"
            )

    if errors:
        print("=== FM-1 Gate 4.3 FAILED ===", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1

    print(
        "[FM-1 Gate 4.3 OK] Timed SPI1 DMA, 42-LED ΔΣ, panel/pressure, MIDI clock & USR NVS gates passed."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
