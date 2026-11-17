"""Shared constants, incremental `-MD` + SHA-256 compiler cache, and ELF32 helpers for FM-1."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import dataclasses
import hashlib
import os
import pathlib
import platform
import re
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
FM1_DIR = ROOT / "platforms" / "fm1"
MAKEFILE = ROOT / "Makefile"
OUT_DIR = ROOT / "build" / "fm1"
OBJ_DIR = OUT_DIR / "obj"
RAW_OBJ_DIR = OBJ_DIR / "raw"
C1_OBJ_DIR = OBJ_DIR / "c1"
LDR_DIR = OUT_DIR / "loader"
GEN_DIR = OUT_DIR / "gen"

DOCKER_IMAGE = os.environ.get("RUIDO_FM1_DOCKER_IMAGE", "ruido-fm1-toolchain:latest")
APP_XIP = 0x02000120
C1_SRAM_ORIGIN = 0x01C0E000
C1_SRAM_MAX_BYTES = 72 * 1024
RAM_MAX_BYTES = 24 * 1024
POOL_MAX_BYTES = 0x50000
LOADER_LOAD = 0x01C0A800
LOADER_NAME = b"usb_hid_ota.bin"
OTA_MAX_BYTES = 0x4F00  # 0xE0000..0xE4EFF below OTA_RES @ 0xE4F00
PRODUCT_DEV = "FM-1_312"  # Dedicated Ruido package identity (`FM-1_312`)
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


@dataclasses.dataclass(frozen=True)
class CompileSpec:
    src: pathlib.Path
    out_obj: pathlib.Path
    cmd: list[str]
    use_depfile: bool = True


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


def write_text_if_changed(path: pathlib.Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and path.read_text(encoding="utf-8") == content:
        return
    path.write_text(content, encoding="utf-8")


def write_bytes_if_changed(path: pathlib.Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and path.read_bytes() == content:
        return
    path.write_bytes(content)


def crc16_ccitt(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def ldr_head(dcrc: int, a: int, b: int, attr: int, name: bytes) -> bytes:
    body = struct.pack("<HIIBBH16s", dcrc, a, b, attr, 0, 0, name)
    return struct.pack("<H", crc16_ccitt(body)) + body


def parse_depfile(dep_path: pathlib.Path) -> list[pathlib.Path] | None:
    if not dep_path.is_file():
        return None
    raw = dep_path.read_text(encoding="utf-8").replace("\\\n", " ")
    if ":" not in raw:
        return None
    _, rhs = raw.split(":", 1)
    tokens = [tok.replace("\\ ", " ") for tok in re.split(r"(?<!\\)\s+", rhs.strip()) if tok]
    if not tokens:
        return None
    deps: list[pathlib.Path] = []
    for tok in tokens:
        p = pathlib.Path(tok)
        if not p.is_absolute():
            p = ROOT / p
        if not p.is_file():
            return None
        deps.append(p.resolve())
    return deps


def compute_compile_digest(
    spec: CompileSpec,
    deps: list[pathlib.Path],
    file_sha_cache: dict[pathlib.Path, str],
) -> str:
    h = hashlib.sha256()
    h.update("\x00".join(spec.cmd).encode("utf-8"))
    h.update(b"\x00")
    extra_bins: list[pathlib.Path] = []
    if spec.cmd:
        compiler_fp = pathlib.Path(spec.cmd[0])
        if compiler_fp.is_file():
            extra_bins.append(compiler_fp.resolve())
    all_files = sorted({spec.src.resolve(), *extra_bins, *(d.resolve() for d in deps)})
    for fp in all_files:
        digest = file_sha_cache.get(fp)
        if digest is None:
            digest = hashlib.sha256(fp.read_bytes()).hexdigest()
            file_sha_cache[fp] = digest
        h.update(str(fp).encode("utf-8"))
        h.update(b"=")
        h.update(digest.encode("ascii"))
        h.update(b"\n")
    return h.hexdigest()


def compile_cached_jobs(
    specs: list[CompileSpec],
    file_sha_cache: dict[pathlib.Path, str],
    rebuild: bool = False,
) -> tuple[int, int]:
    """Compile `specs` in parallel, skipping up-to-date `.o` targets via `-MD` + SHA-256."""
    misses: list[tuple[CompileSpec, pathlib.Path, pathlib.Path]] = []
    hits = 0
    for spec in specs:
        spec.out_obj.parent.mkdir(parents=True, exist_ok=True)
        dep_path = spec.out_obj.with_suffix(".d")
        stamp_path = spec.out_obj.with_suffix(".sha256")
        if not rebuild and spec.out_obj.is_file() and stamp_path.is_file():
            deps = parse_depfile(dep_path) if spec.use_depfile else [spec.src]
            if deps is not None:
                expected = compute_compile_digest(spec, deps, file_sha_cache)
                if stamp_path.read_text(encoding="ascii").strip() == expected:
                    hits += 1
                    continue
        misses.append((spec, dep_path, stamp_path))

    def build_one(item: tuple[CompileSpec, pathlib.Path, pathlib.Path]) -> tuple[CompileSpec, pathlib.Path, pathlib.Path]:
        spec, dep_path, stamp_path = item
        full_cmd = [*spec.cmd, "-MD", "-MF", str(dep_path)] if spec.use_depfile else list(spec.cmd)
        run_cmd(full_cmd)
        return item

    if misses:
        with ThreadPoolExecutor(max_workers=min(8, len(misses))) as pool:
            built = list(pool.map(build_one, misses))
        for spec, dep_path, stamp_path in built:
            deps = parse_depfile(dep_path) if spec.use_depfile else [spec.src]
            if deps is None:
                raise SystemExit(f"[FM-1 FAIL] missing/invalid depfile after compiling {spec.src}")
            digest = compute_compile_digest(spec, deps, file_sha_cache)
            stamp_path.write_text(digest + "\n", encoding="ascii")

    return hits, len(misses)


def elf32_section_bytes(obj_path: pathlib.Path) -> tuple[int, int, list[str]]:
    """Pure-Python ELF32 section reader (avoids spawning x86_64 `objdump -h` 27x under QEMU)."""
    buf = obj_path.read_bytes()
    e_shoff = struct.unpack_from("<I", buf, 32)[0]
    e_shnum, e_shstrndx = struct.unpack_from("<HH", buf, 48)
    shdrs = [struct.unpack_from("<10I", buf, e_shoff + i * 40) for i in range(e_shnum)]
    shstr_sh = shdrs[e_shstrndx]
    shstr = buf[shstr_sh[4] : shstr_sh[4] + shstr_sh[5]]

    text_b = 0
    rodata_b = 0
    bad_ctors: list[str] = []
    for sh in shdrs:
        end = shstr.index(0, sh[0])
        sec = shstr[sh[0] : end].decode("ascii", "replace")
        sz = sh[5]
        if sz == 0:
            continue
        if sec.startswith(".c1_text.rodata") or sec.startswith(".rodata"):
            rodata_b += sz
        elif sec.startswith(".c1_text") or sec.startswith(".text"):
            text_b += sz
        if any(sec.startswith(bad) for bad in (".init_array", ".ctors", ".dtors", ".fini_array")):
            bad_ctors.append(f"{obj_path.name}:{sec}({sz}B)")
    return text_b, rodata_b, bad_ctors


def collect_c1_weak_symbols(obj_paths: list[pathlib.Path]) -> set[str]:
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


# Cold Core-0-only init/constructor/reset/bench `.text.*` sections in DSP/C1 TUs that are
# never called from Core 1 or real-time DSP and should stay in XIP `.text` to save `.c1_text` SRAM.
C1_COLD_XIP_TEXT_SECTIONS: frozenset[str] = frozenset(
    {
        ".text.ruido_fm1_c1_construct",
        ".text.ruido_fm1_set_bench_stress",
        ".text.ruido_fm1_c1_snap_latest",
        ".text.ruido_fm1_c1_prime_blocks",
        ".text.ruido_fm1_c1_enter_solo_fallback",
        ".text.ruido_fm1_c1_step_solo_bench64",
        ".text.ruido_fm1_c1_set_helper_armed",
        ".text._ZN5ruido15DroneEngineCoreC2Ev",
        ".text._ZN5ruido15DroneEngineCoreC1Ev",
        ".text._ZN5ruido15DroneEngineCore4initERNS_17EngineSharedBusesEf",
        ".text._ZN5ruido15DroneEngineCore22snapParametersToLatestERNS_17EngineSharedBusesE",
        ".text._ZN5ruido12VoiceLfoBank5resetEf",
        ".text._ZN5ruido12VoiceLfoBankC2Ev",
        ".text._ZN5ruido12VoiceLfoBankC1Ev",
        ".text._ZNK5ruido12VoiceLfoBank21writeInitialTelemetryERNS_15EngineTelemetryE",
        ".text._ZN5ruido21SchmittOscillatorBank5resetEf",
        ".text._ZN5ruido21SchmittOscillatorBankC2Ev",
        ".text._ZN5ruido21SchmittOscillatorBankC1Ev",
        ".text._ZN5ruido16RhythmClockState4initEff",
        ".text._ZN5ruido17Fm1AudioBlockRing5resetEv",
        ".text._ZN5ruido18LockFreeControlBusC2Ev",
        ".text._ZN5ruido18LockFreeControlBusC1Ev",
    }
)

C1_COLD_XIP_SYMBOLS: frozenset[str] = frozenset(
    sec[len(".text.") :] for sec in C1_COLD_XIP_TEXT_SECTIONS if sec.startswith(".text.")
)


def relocate_obj_to_c1_sram(
    src_obj_path: pathlib.Path,
    dst_obj_path: pathlib.Path,
    c1_weak_names: set[str],
) -> None:
    """Copy `src_obj_path` -> `dst_obj_path`, relocating hot `.text*`/`.rodata*` into `.c1_text` SRAM."""
    out = bytearray(src_obj_path.read_bytes())
    if out[:5] != b"\x7fELF\x01" or out[5] != 1:
        raise SystemExit(f"[FM-1 FAIL] {src_obj_path} is not little-endian ELF32")

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
        raise SystemExit(f"[FM-1 FAIL] unexpected ELF32 e_shentsize={e_shentsize} in {src_obj_path}")

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
    for sh in shdrs:
        if sh[1] != 1:
            continue
        sec_name = read_cstr(shstr, sh[0])
        new_sec: str | None = None
        if (sec_name == ".text" or sec_name.startswith(".text.")) and (
            sec_name not in C1_COLD_XIP_TEXT_SECTIONS
        ):
            new_sec = ".c1_text" + sec_name[len(".text") :]
        elif sec_name == ".rodata" or sec_name.startswith(".rodata."):
            new_sec = ".c1_text.rodata" + sec_name[len(".rodata") :]
        if new_sec is not None:
            sh[0] = len(shstr)
            shstr += new_sec.encode("ascii") + b"\x00"

    group_sig_syms = {sh[7] for sh in shdrs if sh[1] == 17}
    for sh in shdrs:
        if sh[1] != 2 or sh[9] != 16:
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
                (
                    sname in c1_weak_names
                    or (st_shndx != 0 and ((st_info >> 4) == 2 or sym_idx in group_sig_syms))
                )
                and sname not in C1_COLD_XIP_SYMBOLS
                and not sname.startswith("__c1w_")
            ):
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
    write_bytes_if_changed(dst_obj_path, bytes(out))
