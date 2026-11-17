"""Gate 4.3 headless `fm1-emu` dual-core SoC verification (parallel 5-gate execution)."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import os
import pathlib
import re
import struct

from fm1_common import OUT_DIR, run_cmd


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


def _gate_timed_lcd(fwsc_path: pathlib.Path, status_addr: int) -> tuple[list[str], list[str]]:
    logs: list[str] = []
    errs: list[str] = []
    wav_out = OUT_DIR / "emu_boot.wav"
    wav_out.unlink(missing_ok=True)
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
        errs.append(f"fm1-emu normal boot dump incomplete: {sw}")
        return logs, errs

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
    logs.append(
        f"  fm1-emu Timed LCD : magic={magic:#010x}, c1_blocks={c1_blocks} (started={c1_started}, "
        f"fallback={c1_fallback}, underruns={c1_underruns}, c0_assist={c1_assist_spans}), "
        f"audio_blocks={audio_blocks}, lcd_frames={lcd_frames}, peak_q15={peak_q15}, wav={wav_bytes:,} B"
    )
    logs.append(
        f"  Clock & Core0 ISR : CLK_CON0={clk_con0:#08x}, SYS_DIV={sys_div:#04x}, "
        f"cal={insns_per_tick:.2f} insn/tick, Core0 ALNK0 DMA256=~{dma_insns_f:.1f} insns/frame "
        f"({100.0 * dma_insns_f / cyc_budget_240:.2f}% @ 240 MHz)"
    )
    logs.append(
        f"  Dual-Core Idle 64f: ~{idle_insns_f:.0f} insns/frame (live c1=~{c1_live_insns_f:.0f} insns/f; "
        f"{100.0 * idle_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
        f"{100.0 * idle_insns_f / cyc_budget_320:.1f}% @ 320 MHz)"
    )
    logs.append(
        f"  Dual-Core Stress  : ~{full_insns_f:.0f} insns/frame "
        f"({100.0 * full_insns_f / cyc_budget_240:.1f}% @ 240 MHz, "
        f"{100.0 * full_insns_f / cyc_budget_320:.1f}% @ 320 MHz; "
        f"4v HP+LP ZDF self-osc + drive + 8x LFO)"
    )
    if magic != 0x52554944:
        errs.append(f"fm1-emu normal boot did not reach ruido_fm1_arm_audio (magic={magic:#x})")
    if c1_started != 1 or c1_blocks == 0 or c1_fallback != 0:
        diag = [
            ln.strip()
            for ln in boot_out.splitlines()
            if any(k in ln.lower() for k in ("fault", "trap", "cpu1", "pc_limit", "dbg", "crash", "illegal"))
        ]
        errs.append(
            f"Core 1 failed normal SRAM execution (started={c1_started}, c1_blocks={c1_blocks}, "
            f"fallback={c1_fallback}, diag={diag[:6]})"
        )
    if c1_underruns != 0:
        errs.append(f"Core 1 -> ALNK0 DMA ring underruns detected (underruns={c1_underruns})")
    if idle_insns_f > cyc_budget_240:
        errs.append(f"Dual-core Idle DSP ({idle_insns_f:.0f} insns/f) exceeds 240 MHz budget ({cyc_budget_240:.0f})")
    if full_insns_f > cyc_budget_320:
        errs.append(f"Dual-core Stress DSP ({full_insns_f:.0f} insns/f) exceeds 320 MHz budget ({cyc_budget_320:.0f})")
    if audio_blocks == 0 or peak_q15 == 0:
        errs.append(f"fm1-emu normal boot produced silent/zero audio (blocks={audio_blocks}, peak={peak_q15})")
    if dma_insns_f > 120.0:
        errs.append(f"Core 0 ALNK0 DMA ISR too heavy ({dma_insns_f:.1f} insns/frame; expected ring drain < 120)")
    if wav_bytes <= 44:
        errs.append(f"fm1-emu did not write valid audio WAV ({wav_bytes} B)")
    if lcd_frames == 0:
        errs.append("fm1-emu timed SPI1 boot rendered 0 LCD frames")
    if boot_hud != 0 or boot_run != 1 or boot_bpm_x10 != 1200 or boot_osc1_x10 != 654:
        errs.append(
            f"boot state mismatch (hud={boot_hud}, run={boot_run}, bpm_x10={boot_bpm_x10}, osc1_x10={boot_osc1_x10})"
        )
    if boot_stop_led != 1 or boot_play_led != 0 or (led_summary & 0x00FFFFFF) == 0:
        errs.append(f"boot 42-LED summary mismatch (led_summary={led_summary:#010x})")
    return logs, errs


def _gate_panel_midi(fwsc_path: pathlib.Path, status_addr: int) -> tuple[list[str], list[str]]:
    logs: list[str] = []
    errs: list[str] = []
    timed_env = {**os.environ, "FM1_LCD_TIMED": "1"}
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
        errs.append(f"Gate 4.3 panel + MIDI clock dump incomplete: {csw}")
        return logs, errs

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
        errs.append(
            f"Gate 4.3 panel/MIDI check failed: model={p_model}, sync={p_sync}, run={p_run}, "
            f"events={p_events}, bpm_x10={m_bpm_x10}, midi_ev={m_events}, "
            f"play_led={l_play}, stop_led={l_stop}, osc1_x10={c_osc1_x10}, underruns={c_underruns}"
        )
    else:
        logs.append(
            f"  fm1-emu Panel+MIDI: PASS (model=RhythmLab, osc1={c_osc1_x10 / 10.0:.1f} Hz, "
            f"events={p_events}, MIDI={m_bpm_x10 / 10.0:.1f} BPM [{m_events} msgs], "
            f"PLAY_green={l_play}/STOP_red={l_stop}, underruns=0)"
        )
    return logs, errs


def _gate_usr_nvs(
    fwsc_path: pathlib.Path,
    raw_flash_head: bytes,
    img: bytes,
    status_addr: int,
) -> tuple[list[str], list[str]]:
    logs: list[str] = []
    errs: list[str] = []
    if len(raw_flash_head) > 0xEA000:
        errs.append(f"encrypted app flash head ({len(raw_flash_head):#x}) overlaps USR NVS (0xEA000)")
        return logs, errs

    raw_flash_path = OUT_DIR / "emu_nvs_flash.bin"
    baseline_flash = bytearray(raw_flash_head) + bytearray(b"\xff" * (0x100000 - len(raw_flash_head)))
    raw_flash_path.write_bytes(baseline_flash)

    nvs_boot1_out = run_cmd(
        [
            "fm1-emu",
            "--fwsc",
            str(fwsc_path),
            "--raw-flash",
            str(raw_flash_path),
            "--ms",
            "1250",
            "--quiet",
            "--keys",
            "520:enc2:+12,720-750:SAVE,800-840:n16",
            "--dump",
            f"{status_addr:#x}",
            "84",
        ],
        env={**os.environ, "FM1_FLASH_TRACE": "1"},
        include_stderr=True,
    )
    nsw1 = parse_dump_words(nvs_boot1_out)

    flash_trace_re = re.compile(r"flash cmd (?:0x)?([0-9a-fA-F]+) addr (?:0x)?([0-9a-fA-F]+) bytes (\d+)")
    erase_ops: list[int] = []
    prog_ops: list[tuple[int, int]] = []
    bad_flash_ops: list[str] = []
    for m_fl in flash_trace_re.finditer(nvs_boot1_out):
        fl_cmd = int(m_fl.group(1), 16)
        fl_addr = int(m_fl.group(2), 16)
        fl_bytes = int(m_fl.group(3))
        if fl_cmd in (0x20, 0x52, 0xD8, 0xC7, 0x60, 0x01, 0x31):
            erase_ops.append(fl_addr)
        elif fl_cmd == 0x02:
            if fl_addr in (0x0F5100, 0x0F8020):
                prog_ops.append((fl_addr, fl_bytes))
            else:
                bad_flash_ops.append(f"prog@addr={fl_addr:#08x}(bytes={fl_bytes})")

    persisted_flash = bytearray(baseline_flash)
    slot1_raw = struct.pack("<32I", *nsw1[16:48]) if len(nsw1) >= 84 else b""
    settings_raw = struct.pack("<4I", *nsw1[48:52]) if len(nsw1) >= 84 else b""
    if len(slot1_raw) == 128 and len(settings_raw) == 16:
        persisted_flash[0x0F5100 : 0x0F5100 + 128] = slot1_raw
        persisted_flash[0x0F8020 : 0x0F8020 + 16] = settings_raw
        raw_flash_path.write_bytes(persisted_flash)

    xip_bin_path = OUT_DIR / "ruido_xip.bin"
    xip_bin_path.write_bytes((b"\x00" * 0x120) + img)
    nvs_boot2_out = run_cmd(
        [
            "fm1-emu",
            "--bin",
            str(xip_bin_path),
            "--entry",
            "0x02000120",
            "--raw-flash",
            str(raw_flash_path),
            "--ms",
            "2700",
            "--quiet",
            "--keys",
            "560-590:SCL,620-650:SCL,680-710:HOME,740:enc2:+16",
            "--dump",
            f"{status_addr:#x}",
            "84",
        ],
        env={**os.environ, "FM1_FLASH_TRACE": "1"},
        include_stderr=True,
    )
    nsw2 = parse_dump_words(nvs_boot2_out)
    b2_erase_ops: list[int] = []
    b2_prog_ops: list[tuple[int, int]] = []
    b2_bad_ops: list[str] = []
    for m_fl2 in flash_trace_re.finditer(nvs_boot2_out):
        fl_cmd = int(m_fl2.group(1), 16)
        fl_addr = int(m_fl2.group(2), 16)
        fl_bytes = int(m_fl2.group(3))
        if fl_cmd in (0x20, 0x52, 0xD8, 0xC7, 0x60, 0x01, 0x31):
            b2_erase_ops.append(fl_addr)
        elif fl_cmd == 0x02:
            if (fl_addr == 0x0F8030 and fl_bytes == 20) or (fl_addr == 0x0F9100 and fl_bytes == 132):
                b2_prog_ops.append((fl_addr, fl_bytes))
            else:
                b2_bad_ops.append(f"prog@addr={fl_addr:#08x}(bytes={fl_bytes})")

    # Persist the 16 B Settings journal entry (0x0F8030) + 128 B LIVE scratchpad journal entry
    # (0x0F9100), then cold-boot a 3rd time to prove Scene 2 + CircuitLab + dirty LIVE drone
    # tweaks restore across power-off with ZERO sector erases!
    b3_model = -1
    b3_slot = -1
    b3_dirty = 0
    b3_mask = 0
    b3_osc1 = 0
    if len(nsw2) >= 84:
        persisted_flash[0x0F8030 : 0x0F8030 + 16] = struct.pack("<4I", *nsw2[48:52])
        persisted_flash[0x0F9100 : 0x0F9100 + 128] = struct.pack("<32I", *nsw2[52:84])
        raw_flash_path.write_bytes(persisted_flash)
        nvs_boot3_out = run_cmd([
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
        nsw3 = parse_dump_words(nvs_boot3_out)
        if len(nsw3) >= 16:
            b3_model = nsw3[12] & 3
            b3_slot = (nsw3[12] >> 2) & 3
            b3_dirty = (nsw3[12] >> 6) & 1
            b3_mask = nsw3[15] & 0xFF
            b3_osc1 = (nsw3[15] >> 16) & 0xFFFF

    b1_slot = ((nsw1[12] >> 2) & 3) if len(nsw1) >= 84 else -1
    b1_dirty = ((nsw1[12] >> 6) & 1) if len(nsw1) >= 84 else 1
    b1_saves = ((nsw1[15] >> 8) & 0xFF) if len(nsw1) >= 84 else 0
    b1_osc1 = ((nsw1[15] >> 16) & 0xFFFF) if len(nsw1) >= 84 else 0
    b1_underruns = ((nsw1[11] >> 8) & 0xFF) if len(nsw1) >= 84 else 99
    b2_model = (nsw2[12] & 3) if len(nsw2) >= 84 else -1
    b2_slot = ((nsw2[12] >> 2) & 3) if len(nsw2) >= 84 else -1
    b2_dirty = ((nsw2[12] >> 6) & 1) if len(nsw2) >= 84 else 0
    b2_mask = (nsw2[15] & 0xFF) if len(nsw2) >= 84 else 0
    b2_saves = ((nsw2[15] >> 8) & 0xFF) if len(nsw2) >= 84 else 0
    b2_osc1 = ((nsw2[15] >> 16) & 0xFFFF) if len(nsw2) >= 84 else 0
    b2_underruns = ((nsw2[11] >> 8) & 0xFF) if len(nsw2) >= 84 else 99

    # Note: fm1-emu `FM1_FLASH_TRACE` reports total SPI frame bytes (4B cmd+addr + payload)
    if (
        bad_flash_ops
        or erase_ops
        or sorted(prog_ops) != [(0x0F5100, 132), (0x0F8020, 20)]
        or b1_underruns != 0
    ):
        errs.append(
            f"NVS Boot1 live Scene journal failed: bad={bad_flash_ops[:4]}, "
            f"erases={[hex(x) for x in erase_ops]}, "
            f"progs={[(hex(a), n) for a, n in prog_ops]}, underruns={b1_underruns}, "
            f"len_nsw1={len(nsw1)}, nsw1={[hex(x) for x in nsw1[:16]]}"
        )
    elif (
        b2_bad_ops
        or b2_erase_ops
        or sorted(b2_prog_ops) != [(0x0F8030, 20), (0x0F9100, 132)]
        or b2_underruns != 0
    ):
        errs.append(
            f"NVS Boot2 live Settings+LIVE journal failed: bad={b2_bad_ops[:4]}, "
            f"erases={[hex(x) for x in b2_erase_ops]}, "
            f"progs={[(hex(a), n) for a, n in b2_prog_ops]}, underruns={b2_underruns}, "
            f"nsw2={[hex(x) for x in nsw2[:16]]}"
        )
    elif (
        b1_slot != 1
        or b1_dirty != 0
        or b1_saves != 2
        or b1_osc1 <= 600
        or b2_model != 1
        or b2_slot != 1
        or b2_dirty != 1
        or b2_mask != 0x12
        or b2_saves != 2
        or b2_osc1 <= b1_osc1 + 50
        or b3_model != 1
        or b3_slot != 1
        or b3_dirty != 1
        or b3_mask != 0x32
        or abs(b3_osc1 - b2_osc1) > 2
    ):
        errs.append(
            f"NVS 3-boot persistence mismatch: boot1(slot={b1_slot}, dirty={b1_dirty}, saves={b1_saves}, osc1={b1_osc1}) "
            f"vs boot2(model={b2_model}, slot={b2_slot}, dirty={b2_dirty}, mask={b2_mask:#x}, "
            f"saves={b2_saves}, osc1={b2_osc1}) vs boot3(model={b3_model}, slot={b3_slot}, "
            f"dirty={b3_dirty}, mask={b3_mask:#x}, osc1={b3_osc1})"
        )
    else:
        logs.append(
            f"  fm1-emu USR NVS   : PASS (boot1 S2=0xf5100+cfg=0xf8020 [erases=0, underruns=0], "
            f"boot2 cfg=0xf8030+live=0xf9100 [erases=0, underruns=0], "
            f"boot3 restored model={b3_model}/S2+LIVE osc1={b3_osc1 / 10.0:.1f} Hz)"
        )
    return logs, errs


def _gate_flaky_c1(fwsc_path: pathlib.Path, status_addr: int) -> tuple[list[str], list[str]]:
    logs: list[str] = []
    errs: list[str] = []
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
    if len(fsw) < 16 or fsw[0] != 0x52554944 or f_fallback != 1 or fsw[1] == 0 or fsw[2] == 0 or fsw[7] == 0:
        errs.append(f"Flaky-Core1 survival check failed: {fsw}")
    else:
        logs.append(
            f"  fm1-emu Flaky-C1  : PASS (cpu0 survived Core 1 bit-21 SRAM fault, "
            f"fallback={f_fallback}, audio_blocks={fsw[1]}, lcd_frames={fsw[2]}, peak_q15={fsw[7]})"
        )
    return logs, errs


def _gate_oct_rescue(
    fwsc_path: pathlib.Path,
    bootguard_addr: int,
    status_addr: int,
) -> tuple[list[str], list[str]]:
    logs: list[str] = []
    errs: list[str] = []
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
        errs.append(f"Hold OCT- USB rescue failed bootguard check (got {bw}, expected pending=2)")
    elif len(rsw) < 16 or rsw[0] != 0 or rsw[1] != 0 or rsw[8] != 0:
        errs.append(f"Hold OCT- USB rescue leaked into C++ init or Core 1 (status={rsw})")
    else:
        logs.append(
            f"  fm1-emu OCT- Rescue: PASS (bootguard.pending={bw[2]}, "
            f"cpp_magic={rsw[0]:#x}, audio_blocks={rsw[1]}, c1_blocks={rsw[8]})"
        )
    return logs, errs


def run_emu_gates(
    *,
    fwsc_path: pathlib.Path,
    raw_flash_head: bytes,
    img: bytes,
    status_addr: int,
    bootguard_addr: int,
) -> list[str]:
    """Run all 5 disjoint `fm1-emu` SoC gates concurrently and print deterministic results."""
    tasks = [
        lambda: _gate_timed_lcd(fwsc_path, status_addr),
        lambda: _gate_panel_midi(fwsc_path, status_addr),
        lambda: _gate_usr_nvs(fwsc_path, raw_flash_head, img, status_addr),
        lambda: _gate_flaky_c1(fwsc_path, status_addr),
        lambda: _gate_oct_rescue(fwsc_path, bootguard_addr, status_addr),
    ]
    with ThreadPoolExecutor(max_workers=len(tasks)) as pool:
        results = list(pool.map(lambda fn: fn(), tasks))

    errors: list[str] = []
    for logs, errs in results:
        for ln in logs:
            print(ln)
        errors.extend(errs)
    return errors
