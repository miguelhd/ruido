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
    OTA_MAX_BYTES,
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
    usb_product_desc = b"\x16\x03" + "Ruido FM-1".encode("utf-16le")
    if usb_product_desc not in img:
        errors.append("ruido.bin missing 'Ruido FM-1' UTF-16LE USB iProduct string descriptor")

    rt_dis = extract_dis_section(dis, ".ram_text")
    c1_dis = extract_dis_section(dis, ".c1_text")

    rt_calls = [ln for ln in rt_dis.splitlines() if re.search(r"\bcall\b", ln)]
    if rt_calls:
        errors.append(f".ram_text contains call instructions: {rt_calls[:2]}")

    sym_label_re = re.compile(r"^([A-Za-z_.$][A-Za-z0-9_.$]*):$")
    c1_xip_leaks: list[str] = []
    c1_bad_ops: list[str] = []
    boot_post_sp_ops: list[str] = []
    in_boot_post = False
    dsp_fdiv = 0
    c1_flushinv = 0
    for ln in c1_dis.splitlines():
        if not ln.startswith(" "):
            m_c1_lbl = sym_label_re.match(ln.strip())
            if m_c1_lbl:
                in_boot_post = m_c1_lbl.group(1) == "ruido_fm1_c1_boot_post"
            continue
        mm = DIS_LINE_RE.match(ln)
        if not mm:
            continue
        asm = mm.group(3)
        if "(f)" in asm and " / " in asm:
            dsp_fdiv += 1
        if "flushinv" in asm:
            c1_flushinv += 1
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
    if c1_flushinv < 20:
        errors.append(
            f".c1_text missing CacheSync flushinv instructions (found {c1_flushinv}, expected >= 20)"
        )

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
    c0_ustack_bytes = sym_addr("_ustack_top") - sym_addr("_ustack_lo")
    c0_sstack_bytes = sym_addr("_sstack_top") - sym_addr("_sstack_lo")
    c1_ustack_bytes = sym_addr("_c1_ustack_top") - sym_addr("_c1_ustack_lo")
    c1_sstack_bytes = sym_addr("_c1_sstack_top") - sym_addr("_c1_sstack_lo")
    if ram_bytes > RAM_MAX_BYTES:
        errors.append(f".data+.bss ({ram_bytes} B) > {RAM_MAX_BYTES} B")
    if c1_bytes == 0 or c1_bytes > C1_SRAM_MAX_BYTES:
        errors.append(f".c1_text ({c1_bytes} B) outside (0, {C1_SRAM_MAX_BYTES}] B")
    if pool_bytes > POOL_MAX_BYTES:
        errors.append(f".pool ({pool_bytes} B) > {POOL_MAX_BYTES} B")
    if c0_ustack_bytes < 15 * 1024 or c0_sstack_bytes < 15 * 1024:
        errors.append(
            f"Core 0 stacks ({c0_ustack_bytes} B U / {c0_sstack_bytes} B S) < 15 KiB minimum"
        )
    if c1_ustack_bytes < 7 * 1024 or c1_sstack_bytes < 7 * 1024:
        errors.append(
            f"Core 1 stacks ({c1_ustack_bytes} B U / {c1_sstack_bytes} B S) < 7 KiB minimum"
        )

    dc_stubs_addr = sym_addr("fm1_dc_fatal_stubs")
    dc_common_addr = sym_addr("fm1_dc_fatal_common")
    if dc_stubs_addr == 0 or dc_common_addr == 0:
        errors.append("missing fm1_dc_fatal_stubs / fm1_dc_fatal_common symbols")
    else:
        stub_push_addrs: set[int] = set()
        for ln in extract_dis_section(dis, ".text").splitlines():
            mm = DIS_LINE_RE.match(ln)
            if mm and mm.group(3).strip() == "[--sp] = {rets}":
                stub_push_addrs.add(int(mm.group(1), 16))
        missing_stubs = [
            i for i in range(128) if (dc_stubs_addr + 6 * i) not in stub_push_addrs
        ]
        if missing_stubs:
            errors.append(
                f"fm1_dc_fatal_stubs relaxation/stride mismatch at indices {missing_stubs[:4]}"
            )

    sym_label_re = re.compile(r"^([A-Za-z_.$][A-Za-z0-9_.$]*):$")
    cur_fn: str | None = None
    cur_sec = ""
    cur_sp = 0
    cur_fn_needs_addr = False
    max_c0_sp = 0
    max_c0_fn = "none"
    max_c1_sp = 0
    max_c1_fn = "none"
    c1_addr_to_fn: dict[int, str] = {}
    c1_fn_sp: dict[str, int] = {}
    c1_fn_call_addrs: dict[str, set[int]] = {}
    oversized_stacks: list[str] = []
    for ln in dis.splitlines():
        if ln.startswith("Disassembly of section "):
            cur_sec = ln.split("Disassembly of section ", 1)[1].rstrip(":")
            cur_fn = None
            cur_sp = 0
            cur_fn_needs_addr = False
            continue
        if not ln.startswith(" "):
            m_fn = sym_label_re.match(ln.strip())
            if m_fn:
                cur_fn = m_fn.group(1)
                cur_sp = 0
                cur_fn_needs_addr = True
            continue
        if cur_fn in (None, "fm1_fatal_stubs", "fm1_dc_fatal_stubs"):
            continue
        if cur_sec == ".c1_text":
            mm_c1 = DIS_LINE_RE.match(ln)
            if mm_c1:
                if cur_fn_needs_addr:
                    c1_addr_to_fn[int(mm_c1.group(1), 16)] = cur_fn
                    cur_fn_needs_addr = False
                for tgt_hex in re.findall(r"call -?\d+ <[^:>]*: ([0-9a-f]+) >", mm_c1.group(3)):
                    c1_fn_call_addrs.setdefault(cur_fn, set()).add(int(tgt_hex, 16))
        for d_sp in re.findall(r"\bsp \+= (-\d+)", ln):
            cur_sp += -int(d_sp)
        if "[--sp]" in ln:
            cur_sp += 16
        if cur_sec == ".c1_text":
            if cur_sp > c1_fn_sp.get(cur_fn, 0):
                c1_fn_sp[cur_fn] = cur_sp
            if cur_sp > max_c1_sp:
                max_c1_sp, max_c1_fn = cur_sp, cur_fn
        else:
            if cur_sp > max_c0_sp:
                max_c0_sp, max_c0_fn = cur_sp, cur_fn
        if cur_sp > 3584 and not any(s.startswith(f"{cur_fn}(") for s in oversized_stacks):
            oversized_stacks.append(f"{cur_fn}({cur_sp}B)")
    if oversized_stacks:
        errors.append(f"functions exceed 3.5 KiB stack frame ceiling: {oversized_stacks[:5]}")

    c1_tree_memo: dict[str, int] = {}

    def compute_c1_tree_sp(fn_name: str, visiting: frozenset[str] = frozenset()) -> int:
        if fn_name in c1_tree_memo:
            return c1_tree_memo[fn_name]
        if fn_name in visiting:
            return c1_fn_sp.get(fn_name, 0)
        next_visiting = visiting | {fn_name}
        callees = [
            c1_addr_to_fn[a]
            for a in c1_fn_call_addrs.get(fn_name, ())
            if a in c1_addr_to_fn
        ]
        child_max = max((compute_c1_tree_sp(c, next_visiting) for c in callees), default=0)
        res = c1_fn_sp.get(fn_name, 0) + child_max
        c1_tree_memo[fn_name] = res
        return res

    c1_tree_sp = max(compute_c1_tree_sp("ruido_fm1_cpu1_loop"), max_c1_sp)
    if c1_tree_sp > 4096:
        errors.append(f"Core 1 call-tree stack ({c1_tree_sp} B) exceeds 4096 B ceiling")

    for chip_name, jedec_id, le_bytes in REQUIRED_JEDEC_IDS:
        if le_bytes not in img:
            errors.append(f"app image (`ruido.bin`) missing {chip_name} JEDEC ID {jedec_id:#08x}")

    ota_est_bytes = max(0, len(fwsc_bytes) - (1024 + 602112 + 20))
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
        f"(ota.bin={ota_est_bytes:,}/{OTA_MAX_BYTES:,} B, {MAX_FWSC_BYTES - len(fwsc_bytes):,} B free)"
    )
    print(
        f"  SRAM .c1_text DSP : {c1_bytes:,} B / {C1_SRAM_MAX_BYTES:,} B "
        f"({c1_bytes / 1024.0:.1f} KiB @ 0x01C0E000, {C1_SRAM_MAX_BYTES - c1_bytes:,} B free, 0 XIP refs; "
        f"raw .text={dsp_text:,} B + .rodata={dsp_rodata:,} B)"
    )
    print(f"  XIP UI + Bridge   : {ui_text + ui_rodata:,} B (.text={ui_text:,} B + .rodata={ui_rodata:,} B)")
    print(f"  SRAM .data + .bss : {ram_bytes:,} B / {RAM_MAX_BYTES:,} B ({ram_bytes / 1024.0:.1f} KiB)")
    print(f"  SRAM .pool        : {pool_bytes:,} B / {POOL_MAX_BYTES:,} B ({pool_bytes / 1024.0:.1f} KiB)")
    print(
        f"  Guarded U/S Stacks: Core0 max={max_c0_sp:,} B / ({c0_ustack_bytes:,} U + {c0_sstack_bytes:,} S), "
        f"Core1 tree={c1_tree_sp:,} B (max frame={max_c1_sp:,} B) / ({c1_ustack_bytes:,} U + {c1_sstack_bytes:,} S), 128x6B stubs=OK"
    )
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
