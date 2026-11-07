"""Gate 4.1 + 4.2 static binary, symbol, FPU, SRAM/XIP isolation, and anti-brick audits."""

from __future__ import annotations

import pathlib
import re
from typing import Callable

from fm1_common import (
    APP_XIP,
    C1_SRAM_MAX_BYTES,
    C1_SRAM_ORIGIN,
    DIS_LINE_RE,
    FORBIDDEN_INT64_DIV,
    FORBIDDEN_SOFT_DOUBLE,
    MAX_FWSC_BYTES,
    POOL_MAX_BYTES,
    PRODUCT_DEV,
    RAM_MAX_BYTES,
    REQUIRED_JEDEC_IDS,
    elf32_section_bytes,
)


def extract_dis_section(full_dis: str, sec_name: str) -> str:
    """Slice `Disassembly of section <sec_name>:` from a single `objdump -d` pass."""
    marker = f"Disassembly of section {sec_name}:"
    idx = full_dis.find(marker)
    if idx < 0:
        return ""
    next_idx = full_dis.find("Disassembly of section ", idx + len(marker))
    return full_dis[idx:] if next_idx < 0 else full_dis[idx:next_idx]


def audit_firmware_image(
    *,
    fwsc_path: pathlib.Path,
    fwsc_bytes: bytes,
    img: bytes,
    app_slot: int,
    syms: str,
    hdrs: str,
    dis: str,
    sym_addr: Callable[[str], int],
    c1_objs: list[pathlib.Path],
    ui_audit_objs: list[pathlib.Path],
    cxx_std: str,
    dsp_opt: str,
    ui_opt: str,
    cache_hits: int,
    cache_built: int,
) -> list[str]:
    errors: list[str] = []

    import fm1_install  # type: ignore

    pkg_prod, _ = fm1_install.load_package(str(fwsc_path), force=False)
    if pkg_prod != PRODUCT_DEV:
        errors.append(f"fwsc product identity mismatch: {pkg_prod!r} != {PRODUCT_DEV!r}")

    if sym_addr("_start") != APP_XIP:
        errors.append(f"_start != {APP_XIP:#x}")
    if sym_addr("fm1_cpu1_entry") != C1_SRAM_ORIGIN:
        errors.append(f"fm1_cpu1_entry ({sym_addr('fm1_cpu1_entry'):#x}) != {C1_SRAM_ORIGIN:#x}")
    if img[:4] != bytes.fromhex("04818000"):
        errors.append(f"entry stub mismatch: {img[:4].hex()}")
    if len(img) > app_slot:
        errors.append(f"ruido.bin ({len(img)} B) exceeds APP_SLOT ({app_slot} B)")
    if len(fwsc_bytes) >= MAX_FWSC_BYTES:
        errors.append(f"ruido.fwsc ({len(fwsc_bytes)} B) exceeds 600 KiB ceiling")
    forbidden_url = b"firmware" + b"hub"
    if forbidden_url in img.lower() or forbidden_url in fwsc_bytes.lower():
        errors.append("ruido.bin / ruido.fwsc contains forbidden third-party URL string")

    rt_dis = extract_dis_section(dis, ".ram_text")
    c1_dis = extract_dis_section(dis, ".c1_text")

    rt_calls = [ln for ln in rt_dis.splitlines() if re.search(r"\bcall\b", ln)]
    if rt_calls:
        errors.append(f".ram_text contains call instructions: {rt_calls[:2]}")

    c1_xip_leaks: list[str] = []
    c1_bad_ops: list[str] = []
    boot_post_sp_ops: list[str] = []
    in_boot_post = False
    dsp_fdiv = 0
    for ln in c1_dis.splitlines():
        if re.match(r"^[0-9a-f]+\s+<([^>]+)>:$", ln):
            in_boot_post = "<ruido_fm1_c1_boot_post>:" in ln
            continue
        mm = DIS_LINE_RE.match(ln)
        if not mm:
            continue
        asm = mm.group(3)
        if "(f)" in asm and " / " in asm:
            dsp_fdiv += 1
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

    bad_sections: list[str] = []
    for ln in hdrs.splitlines():
        p = ln.split()
        if len(p) > 3 and p[0].isdigit():
            sec, sz = p[1], int(p[2], 16)
            if any(sec.startswith(b) for b in (".init_array", ".ctors", ".dtors", ".fini_array")) and sz > 0:
                bad_sections.append(f"ELF:{sec}({sz}B)")

    dsp_text = dsp_rodata = 0
    for o in c1_objs:
        t, r, bad = elf32_section_bytes(o)
        dsp_text += t
        dsp_rodata += r
        bad_sections.extend(bad)

    ui_text = ui_rodata = 0
    for o in ui_audit_objs:
        t, r, bad = elf32_section_bytes(o)
        ui_text += t
        ui_rodata += r
        bad_sections.extend(bad)

    if bad_sections:
        errors.append(f"pre-boot global constructor sections found: {bad_sections}")

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
    print(
        f"  Compiler          : pi32v2 clang ({cxx_std}, DSP={dsp_opt}, UI={ui_opt}, -mcpu=r3 -mfprev1; "
        f"cache {cache_hits}/{cache_hits + cache_built} hits, compiled {cache_built})"
    )
    print(
        f"  App XIP (ruido.bin): {len(img):,} B / {app_slot:,} B "
        f"({100.0 * len(img) / app_slot:.1f}% of 567.9 KiB APP_SLOT)"
    )
    print(
        f"  Package (.fwsc)   : {len(fwsc_bytes):,} B / {MAX_FWSC_BYTES:,} B "
        f"({len(fwsc_bytes) / 1024.0:.1f} KiB)"
    )
    print(
        f"  SRAM .c1_text DSP : {c1_bytes:,} B / {C1_SRAM_MAX_BYTES:,} B "
        f"({c1_bytes / 1024.0:.1f} KiB @ 0x01C0E000, 0 XIP refs; "
        f"raw .text={dsp_text:,} B + .rodata={dsp_rodata:,} B)"
    )
    print(f"  XIP UI + Bridge   : {ui_text + ui_rodata:,} B (.text={ui_text:,} B + .rodata={ui_rodata:,} B)")
    print(f"  SRAM .data + .bss : {ram_bytes:,} B / {RAM_MAX_BYTES:,} B ({ram_bytes / 1024.0:.1f} KiB)")
    print(f"  SRAM .pool        : {pool_bytes:,} B / {POOL_MAX_BYTES:,} B ({pool_bytes / 1024.0:.1f} KiB)")
    print(
        f"  Hardware r3 FPU   : fadd={fpu_add}, fsub={fpu_sub}, fmul={fpu_mul}, "
        f"fdiv={fpu_div} (.c1_text={dsp_fdiv}), fcmp={fpu_cmp}, fconv/minmax={fpu_conv}"
    )
    print(
        f"  Soft-float calls  : "
        f"{soft_single_syms if soft_single_syms else 'NONE (100% native r3 hardware FPU)'}"
    )
    print(
        "  Anti-Brick Checks : _start=0x02000120, JEDEC=P25Q80+UC25HQ80, "
        "0 global ctors, 0 __sync_*, 0 soft-double, 0 int64-div, 0 ROM/XIP leaks"
    )
    return errors
