/* FM-1 target entry point for Ruido (built against zvenson/dinghy's GPL-3.0-only platform.c).
 * Keeps dinghy's bootguard, Hold OCT- USB rescue, OTA SysEx updater, and USB-MIDI/Audio intact
 * while constructing all C++17 engine objects and launching Core 1 strictly AFTER `recovery_key()`
 * clears. */

#include "core/Version.h"

#define APP_NAME "RUIDO"
#define APP_VER RUIDO_VERSION_STR
#define STORE_MAGIC 0x52554944u /* "RUID": distinct NVS magic from stock dinghy forks */
#define CV_MAX (240u * 16u)     /* Unused gfx.c canvas shrunk (GC'd by --gc-sections) */
#define LCD_BAUD 1u /* Fast ~30 MHz SPI1 ST7789 scanline DMA (vs default BAUD 4 = 12 MHz) */

#include "fm1_ops_screen.h"

/* Redirect dinghy's 1-byte-at-a-time libc.c memset/memcpy & wrap TIMER5 ISR cleanly */
#define memset dinghy_byte_memset
#define memcpy dinghy_byte_memcpy
#define fm1_timer5_irq dinghy_timer5_irq

static void app_idle(void);
static void ruido_fm1_step_power_rails(void);
static void ruido_fm1_plat_fault_quiesce(void);
static void ruido_fm1_plat_ota_show(uint32_t step, int32_t code);
static void ruido_fm1_plat_rescue_show(uint8_t is_flash_ok);
static void ruido_fm1_plat_uboot_show(void);
static void ruido_fm1_plat_fault_show(uint32_t vec, uint32_t pc, uint32_t rets);
#define LCD_IDLE app_idle
#include "platform_ruido.c"

#undef fm1_timer5_irq
#undef memset
#undef memcpy

#include "fm1_cpu1.h"
#include "fm1_libc_fast.h"
#include "ruido_fm1_bridge.h"

#define RUIDO_UNITY_MASTER_Q12 4096
#define RUIDO_LCD_FRAME_PERIOD_MS 16u
#define RUIDO_LED_BG_PULSE_NS 2000u

/* AC791N P33 voltage-rail registers & V15 360 MHz profile (`VDC14 -> 7` / 1.60 V, `SYSVDD -> 15`
 * / 1.38 V) */
#define RUIDO_P3_ANA_CON6 0x06u
#define RUIDO_P3_ANA_CON9 0x09u
#define RUIDO_VDC14_MASK 0x07u
#define RUIDO_SYSVDD_MASK 0x0Fu
#define RUIDO_VDC14_TARGET_LEVEL 7u
#define RUIDO_SYSVDD_TARGET_LEVEL 15u
#define RUIDO_RAIL_STEP_SETTLE_US 100u

RuidoBootDiag g_ruido_boot_diag __attribute__((section(".noinit")));
static volatile uint8_t g_in_platform_poll = 0u;
static volatile uint8_t g_adc_inflight = 0u;
static uint8_t g_adc_inflight_note = 0xFFu;
static uint8_t g_adc_inflight_pin = 7u;
static uint32_t g_idle_assist_ok = 0u;
static uint8_t g_in_app_idle = 0u;
static uint8_t g_lcd_inited = 0u;
static uint8_t g_last_ota_kind = 0xFFu;

#include "fm1_usb_diag.h"

static inline void mark_boot_stage(uint16_t stage) {
    g_ruido_boot_diag.magic = RUIDO_BOOT_DIAG_MAGIC;
    g_ruido_boot_diag.stage = stage;
    g_ruido_boot_diag.uptime_ms = fm1_ms;
    ruido_fm1_diag_sync_c1();
    fm1_wdt_feed();
}

static void ruido_fm1_plat_blit_ops_screen(const RuidoOpsState* st, int use_ota_idle) {
    uint16_t* ops_strip_be = ruido_fm1_acquire_ops_strip();
    uint16_t y = 0u;
    g_idle_assist_ok = 0u;
    for (y = 0u; y < 240u; y = (uint16_t)(y + 16u)) {
        if (use_ota_idle) {
            ota_idle();
        } else {
            fm1_wdt_feed();
        }
        lcd_sync();
        ruido_ops_render_strip_be(st, y, 16u, ops_strip_be);
        lcd_blit(0u, y, 240u, 16u, ops_strip_be);
        lcd_sync();
    }
}

static void ruido_fm1_plat_rescue_show(uint8_t is_flash_ok) {
    uint32_t diag_code = 0u;
    uint32_t c_pc = 0u;
    uint32_t c_rets = 0u;
    uint32_t c_emu = 0u;
    uint32_t c_dbg = 0u;
    uint32_t c_sp = 0u;
    const int has_crash = (fm1_crash.magic == FM1_CRASH_MAGIC);
    if (is_flash_ok &&
        ((bootguard.magic == BOOTGUARD_MAGIC && bootguard.failed >= 2u) || has_crash)) {
        const uint16_t stg = (g_ruido_boot_diag.magic == RUIDO_BOOT_DIAG_MAGIC)
                                 ? (uint16_t)g_ruido_boot_diag.stage
                                 : 0xFFFFu;
        diag_code = ((uint32_t)fm1_boot.p3_rst << 24u) | ((uint32_t)bootguard.failed << 16u) | stg;
        if (diag_code == 0u) {
            diag_code = 0x0002FFFFu;
        }
        if (has_crash) {
            c_pc = fm1_crash.pc;
            c_rets = fm1_crash.rets;
            c_emu = fm1_crash.emu;
            c_dbg = fm1_crash.dbg;
            c_sp = ((fm1_crash.vec & 0xFFu) << 24u) | (fm1_crash.sp & 0x00FFFFFFu);
        }
    }
    RuidoOpsState st = {
        (uint8_t)(is_flash_ok ? RUIDO_OPS_RESCUE_READY : RUIDO_OPS_RESCUE_BADFLASH),
        0u,
        0u,
        diag_code,
        c_pc,
        c_rets,
        c_emu,
        c_dbg,
        c_sp,
    };
    g_lcd_inited = 1u;
    g_last_ota_kind = 0xFFu;
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

static void ruido_fm1_plat_ota_show(uint32_t step, int32_t code) {
    RuidoOpsState st = {RUIDO_OPS_OTA_CHECK, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    if (step <= 2u) {
        st.kind = RUIDO_OPS_OTA_CHECK;
    } else if (step <= 4u) {
        st.kind = RUIDO_OPS_OTA_PREPARE;
    } else if (step == 5u) {
        st.kind = RUIDO_OPS_OTA_ENTER;
    } else {
        st.kind = (uint8_t)((code == 1) ? RUIDO_OPS_OTA_VERIFIED : RUIDO_OPS_OTA_CANCELLED);
    }
    if (step < 9u && st.kind == g_last_ota_kind) {
        ota_idle();
        return;
    }
    g_last_ota_kind = st.kind;
    ruido_fm1_plat_blit_ops_screen(&st, 1);

    if (step >= 9u) {
        uint32_t t0 = fm1_ticks();
        g_last_ota_kind = 0xFFu;
        while ((uint32_t)(fm1_ticks() - t0) < (1800u * 1000u * FM1_TICKS_PER_US)) {
            ota_idle();
        }
        if (recovery_active) {
            ruido_fm1_plat_rescue_show(flash_ok);
        }
    }
}

static void ruido_fm1_plat_uboot_show(void) {
    RuidoOpsState st = {RUIDO_OPS_UBOOT, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

static void ruido_fm1_plat_fault_quiesce(void) {
    /* Pause Core 1 and disarm shared JL_DBG / Core 0 EMU traps so rendering the 15-strip
     * SYSTEM FAULT screen over SPI1 DMA or resetting into uboot.boot (0x01E00004) never trips
     * a secondary JL_DBG / PC_LIMIT exception. */
    *(volatile uint32_t*)0x01EEE004u |= 2u;  /* hold Core 1 (AC79 SDK): bit 1 set ... */
    *(volatile uint32_t*)0x01EEE004u &= ~8u; /* ... then bit 3 (enable) cleared */
    if ((*(volatile uint32_t*)0x01EEE240u & 1u) == 0u) {
        *(volatile uint32_t*)0x01EEE240u = 0xE7u;
    }
    __asm__ volatile("csync" ::: "memory");
    *(volatile uint32_t*)0x01EEE380u = 0xFFFFFFFFu;
    *(volatile uint32_t*)0x01EEE384u = 0u;
    *(volatile uint32_t*)0x01EEE388u = 0xFFFFFFFFu;
    *(volatile uint32_t*)0x01EEE38Cu = 0u;
    *(volatile uint32_t*)0x01EEE340u = 0u;
    *(volatile uint32_t*)0x01EEE348u = 0u;
    *(volatile uint32_t*)0x01EEE248u = 0xFFFFFFFFu;
    *(volatile uint32_t*)0x01EEF0D0u = 0u;
    *(volatile uint32_t*)0x01EEF0D4u = 0xFFFFFFFFu;
    *(volatile uint32_t*)0x01EEF1A4u = 0xFFu;
    __asm__ volatile("csync" ::: "memory");
}

static void ruido_fm1_plat_fault_show(uint32_t vec, uint32_t pc, uint32_t rets) {
    RuidoOpsState st = {
        RUIDO_OPS_SYSTEM_FAULT, 0u, 0u, vec, pc, rets, fm1_crash.emu, fm1_crash.dbg, fm1_crash.sp,
    };
    g_idle_assist_ok = 0u;
    if (!g_lcd_inited) {
        lcd_init();
        g_lcd_inited = 1u;
    }
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

static void wait_us_bounded(uint32_t us) {
    const uint32_t t0 = fm1_ticks();
    const uint32_t span = us * FM1_TICKS_PER_US;
    while ((uint32_t)(fm1_ticks() - t0) < span) {
    }
}

/* 10 kHz TIMER5 wrapper:
 * 1. Harvests any non-blocking PA7/PA8 DRIVE-2 SARADC conversion launched on the previous 100 us
 *    tick (`0x204Fu` = delay 2, divider 7 -> ~57 us hardware conversion, zero ISR busy-spin) and
 *    restores `PA7`/`PA8` digital pull-up input before `fm1_input_tick()` reads `fm1__rows()`.
 * 2. Runs dinghy's column scan, 1 ms tick, USB poll, and 2 kHz TRS UART1 MIDI DMA poll.
 * 3. Steps our 42-channel staggered-phase Delta-Sigma LED modulator at 909.1 Hz (`fm1_in.frames`).
 * 4. Every 2nd scan frame (454.5 Hz), if `g_in_platform_poll == 0` and the newly latched 74HC595
 *    column matches the requested held PA7/PA8 key, blanks LED anodes on that column, drives the
 *    row high with `DRIVE 2` (`HD0=1, HD=0`, `R_s ~ 132 ohm`), and starts a non-blocking 10-bit
 *    SARADC read across the 100 us column dwell. */
void fm1_timer5_irq(void) {
    static uint32_t adc_sampled_frame = 0xFFFFFFFFu;
    uint32_t prev_frames = 0u;

    if (g_adc_inflight != 0u) {
        const uint32_t m = 1u << g_adc_inflight_pin;
        int32_t val = -2;
        if ((FM1_ADC_CON & 0x80u) != 0u) {
            val = (int32_t)(FM1_ADC_RES & 0x3FFu);
        }
        FM1_ADC_CON = 0x40u;
        FM1_ADC_CON = 0u;

        FM1_PR(FM1_PA, FM1_DIR) |= m;
        FM1_PR(FM1_PA, FM1_OUT) &= ~m;
        FM1_PR(FM1_PA, FM1_HD0) &= ~m;
        FM1_PR(FM1_PA, FM1_HD) &= ~m;
        FM1_PR(FM1_PA, FM1_PU) |= m;
        FM1_PR(FM1_PA, FM1_DIE) |= m;
        g_adc_inflight = 0u;
        if (((g_virt_notes >> g_adc_inflight_note) & 1u) == 0u) {
            ruido_fm1_adc_put_sample(g_adc_inflight_note, val);
        }
    }

    prev_frames = fm1_in.frames;
    dinghy_timer5_irq();

    if (fm1_in.frames != prev_frames) {
        ruido_fm1_led_step_909hz(fm1_led, fm1_led_dim, fm1_led_bg);
    }

    if (!g_in_platform_poll && (uint32_t)(fm1_in.frames - adc_sampled_frame) >= 2u) {
        uint8_t req_note = 0u;
        uint8_t req_col = 0u;
        uint8_t req_ch = 0u;
        uint8_t req_pin = 0u;
        if (ruido_fm1_adc_get_req(&req_note, &req_col, &req_ch, &req_pin) != 0u &&
            fm1__tick_col == req_col) {
            const uint32_t m = 1u << req_pin;
            adc_sampled_frame = fm1_in.frames;
            fm1__led_lines(0u);

            FM1_PR(FM1_PA, FM1_DIE) &= ~m;
            FM1_PR(FM1_PA, FM1_PU) &= ~m;
            FM1_PR(FM1_PA, FM1_PD) &= ~m;
            FM1_PR(FM1_PA, FM1_HD) &= ~m;
            FM1_PR(FM1_PA, FM1_HD0) |= m;
            FM1_PR(FM1_PA, FM1_OUT) |= m;
            FM1_PR(FM1_PA, FM1_DIR) &= ~m;

            FM1_ADC_CON = 0u;
            if (FM1_WLA_CON0 & (1u << 14)) {
                FM1_WLA_CON0 &= ~(1u << 14);
            }
            FM1_ADC_CON = 0x204Fu | (((uint32_t)req_ch & 0xFu) << 8u);
            FM1_ADC_CON |= 0x10u;
            FM1_ADC_CON |= 0x40u;

            g_adc_inflight_note = req_note;
            g_adc_inflight_pin = req_pin;
            g_adc_inflight = 1u;
        }
    }
}

static void run_platform_poll_safe(void) {
    const uint32_t t0 = fm1_ticks();
    g_in_platform_poll = 1u;
    while (g_adc_inflight != 0u && (uint32_t)(fm1_ticks() - t0) < (150u * FM1_TICKS_PER_US)) {
    }
    platform_poll();
    g_in_platform_poll = 0u;
}

/* Read dinghy's `[0..4088]` integer `ruido_raw_master_q12` on Core 0 main loop so `MASTER` pot
 * scaling happens smoothly in float32 inside `DroneEngineCore` BEFORE TPDF quantization, while
 * `platform.c`'s `master_q12` remains permanently locked at unity (4096) with zero ISR race. */
static inline void sync_master_pot_target(void) {
    int32_t mq = ruido_raw_master_q12;
    ruido_fm1_set_master_q12((uint32_t)(mq > 0 ? mq : 0));
}

static void poll_midi_clock(void) {
    uint8_t st = 0u;
    uint8_t d1 = 0u;
    uint8_t d2 = 0u;
    int src = 0;
    while ((src = midi_get(&st, &d1, &d2)) != 0) {
        if (st == 0xF8u || st == 0xFAu || st == 0xFBu || st == 0xFCu) {
            ruido_fm1_on_midi_rt(st);
        } else if (src == 1 && st == 0xBFu) {
            handle_usb_diag_cc(d1, d2);
        }
    }
}

static void poll_panel_inputs(uint32_t now_ms) {
    static uint32_t last_scan_frame = 0xFFFFFFFFu;
    const uint32_t cur_frame = fm1_in.frames;
    uint32_t released_btns = 0u;
    uint32_t pressed_btns = 0u;
    uint32_t cur_btns = 0u;
    uint32_t note_edges = 0u;
    uint32_t cur_notes = 0u;
    int32_t enc_steps[FM1_NENC];
    uint32_t lock_k = 0u;
    uint32_t e = 0u;

    /* Debounced keys, buttons, and quadrature detents update at 909.1 Hz (`fm1_in.frames`).
     * Snapshot edges + masks + detents inside ONE atomic critical section so `TIMER5` can never
     * split `fm1_in.pressed` from `fm1_in.buttons` across two frames. */
    if (cur_frame == last_scan_frame) {
        return;
    }
    last_scan_frame = cur_frame;

    lock_k = fm1__lock();
    pressed_btns = fm1_in.pressed;
    released_btns = fm1_in.released;
    fm1_in.pressed = 0u;
    fm1_in.released = 0u;
    cur_btns = fm1_in.buttons;
    note_edges = fm1_in.notes_pressed;
    fm1_in.notes_pressed = 0u;
    cur_notes = fm1_in.notes;
    for (e = 0u; e < FM1_NENC; ++e) {
        enc_steps[e] = fm1_in.enc_steps[e];
        fm1_in.enc_steps[e] = 0;
    }
    fm1__unlock(lock_k);

    apply_usb_diag_virtual_inputs(&pressed_btns, &released_btns, &cur_btns, &note_edges, &cur_notes,
                                  enc_steps);
    ruido_fm1_poll_panel(now_ms, pressed_btns, released_btns, cur_btns, enc_steps, cur_notes,
                         note_edges);
}

#include "fm1_nvs_glue.h"

static void app_idle(void) {
    if (g_idle_assist_ok && !g_in_app_idle && !in_audio) {
        g_in_app_idle = 1u;
        ruido_fm1_assist_dsp();
        poll_midi_clock();
        poll_panel_inputs(fm1_ms);
        g_in_app_idle = 0u;
    }
}

static void app_audio(int32_t* out, uint32_t n) {
    uint32_t t0 = fm1_ticks();
    ruido_fm1_audio(out, n);
    g_ruido_fm1_status.last_audio256_ticks = fm1_ticks() - t0;

#if FELUCCA_UAC
    if (uac.feed && uac.go) {
        uint32_t w = ua_w;
        uint32_t r = ua_r;
        int32_t fill = (int32_t)(w - r);
        int32_t target = (int32_t)UA_PRIME;
#if FELUCCA_UAC_48K
        if (uac.ring48) {
            target = (int32_t)UA_PRIME48;
        }
#endif
        uint32_t free_slots = UA_N - (w - r);
        uint32_t stream_seq =
            uac.starts ^ (uac.rate_sets * 0x9E3779B9u) ^ ((uint32_t)uac.ring48 << 31u);
        uint32_t written = ruido_fm1_usb_feed(out, n, uac.ring48, fill, target, ua_ring, UA_N - 1u,
                                              w, free_slots, stream_seq);
        if (written > 0u) {
            RING_PUBLISH();
            ua_w = w + written;
        } else {
            uac.overruns++;
        }
        /* Bypass stock non-interpolated uac_tap; our Hermite + TPDF resampler already fed ua_ring
         */
        uac.feed = 0;
    }
#endif
}

static inline void call_c1_set_bench_stress(uint32_t stress_mode) {
    void (*volatile fn)(uint32_t) = ruido_fm1_set_bench_stress;
    fn(stress_mode);
}

static uint32_t calibrate_8k_insns_ticks(void) {
    uint32_t iters = 1000u;
    uint32_t irq_f = irq_save();
    uint32_t t0 = fm1_ticks();
    do {
        __asm__ volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop" ::: "memory");
    } while (--iters != 0u);
    uint32_t dt = fm1_ticks() - t0;
    irq_restore(irq_f);
    return (dt > 0u) ? dt : 1u;
}

static void step_p33_rail_up(uint32_t reg, uint8_t mask, uint8_t target) {
    const uint32_t tmo0 = fm1_p33_timeouts;
    uint32_t irq_f = irq_save();
    const uint8_t val = fm1_p33_read(reg);
    irq_restore(irq_f);
    if (fm1_p33_timeouts != tmo0) {
        return;
    }
    uint8_t lvl = (uint8_t)(val & mask);
    const uint8_t upper = (uint8_t)(val & (uint8_t)~mask);
    while (lvl < target) {
        lvl = (uint8_t)(lvl + 1u);
        irq_f = irq_save();
        fm1_p33_write(reg, (uint8_t)(upper | (lvl & mask)));
        irq_restore(irq_f);
        if (fm1_p33_timeouts != tmo0) {
            return;
        }
        wait_us_bounded(RUIDO_RAIL_STEP_SETTLE_US);
    }
}

static void ruido_fm1_step_power_rails(void) {
    step_p33_rail_up(RUIDO_P3_ANA_CON6, RUIDO_VDC14_MASK, RUIDO_VDC14_TARGET_LEVEL);
    step_p33_rail_up(RUIDO_P3_ANA_CON9, RUIDO_SYSVDD_MASK, RUIDO_SYSVDD_TARGET_LEVEL);
    wait_us_bounded(RUIDO_RAIL_STEP_SETTLE_US);
    fm1_wdt_feed();
}

static void app_main(void) {
    Fm1ClockSnapshot clk_snap = {0u, 0u, 0u, 0u};
    uint32_t init_mq = 2048u;
    uint32_t allow_core1 = 1u;
    uint32_t last_lcd_ms = 0u;
    uint32_t last_lcd_done_ms = 0u;
    uint32_t last_poll_ms = 0u;
    uint32_t cached_cushion = 32u;
    uint32_t first_frame = 1u;
    uint8_t lcd_slot = 0u;

    /* Reachable ONLY after fm1_cstart() cleared bootguard AND Hold OCT- USB rescue! */
    g_ruido_boot_diag.c1_code = 0u;
    g_ruido_boot_diag.c1_aux = 0u;
    g_ruido_boot_diag.c1_state = 0u;
    g_lcd_inited = 1u;
    fm1_led_bg_ns = RUIDO_LED_BG_PULSE_NS;
    mark_boot_stage(0x0010u);

    /* Step `VDC14` (`P3_ANA_CON6`) then `SYSVDD` (`P3_ANA_CON9`) up to factory V15 360 MHz levels
     * (`1.60 V` / `1.38 V`, 1 step per 100 us) before SRAM DSP load, NVS reads, and Core 1 wake. */
    mark_boot_stage(0x0018u);
    ruido_fm1_step_power_rails();

    /* Settle platform.c's 1-pole `knob_avg` IIR (`tau = 8` samples from 512) to the true physical
     * PB6 MASTER pot voltage (~0.3 ms) so initial `init_mq` never triggers a phantom boot toast. */
    g_in_platform_poll = 1u;
    for (uint32_t settle = 0u; settle < 48u; ++settle) {
        platform_poll();
    }
    g_in_platform_poll = 0u;
    if (ruido_raw_master_q12 >= 0) {
        init_mq = (uint32_t)ruido_raw_master_q12;
    }
    master_q12 = RUIDO_UNITY_MASTER_Q12;

    /* 1. Copy Core 1 hot DSP `.text` + `.rodata` (`~60 KiB`) from XIP into 0x01C0E000 SRAM */
    mark_boot_stage(0x0020u);
    fm1_cpu1_load_sram();

    /* 2. Placement-new construct C++17 engine arenas in `.pool` SRAM (audio still disarmed) */
    mark_boot_stage(0x0030u);
    ruido_fm1_construct(init_mq);

    /* 3. Restore CRC-verified Scenes (0xF4000..0xF7FFF), Settings (0xF8000), & LIVE (0xF9000)
     * BEFORE starting Core 1 (`SYS_CON` bit 3 == 0) so `fl_cache_idle()` (`0x01EEE008`) has zero
     *    cross-core bus contention — identical to single-core `ota_boot_cleanup()`! */
    mark_boot_stage(0x0040u);
    (void)ruido_fm1_nvs_load_boot(blob_load, nvs_tail_read_boot, blob_save);

    /* 4. Snapshot AC791N clock dividers, check silicon-lottery early-crash flag, and boot Core 1 */
    mark_boot_stage(0x0050u);
    g_ruido_fm1_status.cal_8k_insns_ticks = calibrate_8k_insns_ticks();
    fm1_clock_snapshot(&clk_snap);
    g_ruido_fm1_status.clk_con0_div =
        ((clk_snap.sys_div & 0xFFu) << 24u) | (clk_snap.clk_con0 & 0x00FFFFFFu);

    /* Silicon-lottery auto-heal: if the previous boot failed (`bootguard.failed > 0`) or tripped
     * an early fault before 3.5 s, skip Core 1 (`SYS_CON` bit 3 stays 0) and boot directly into
     * Solo Half-Rate Mode so Boot #2 always clears `bootguard`. */
    if (bootguard.failed > 0u || (fm1_crash.magic == FM1_CRASH_MAGIC && fm1_crash.early > 0u)) {
        allow_core1 = 0u;
    }
    fm1_crash.magic = 0u;
    fm1_crash.early = 0u;
    if (allow_core1 != 0u) {
        mark_boot_stage(0x0051u);
        allow_core1 = ruido_fm1_start_core1();
        mark_boot_stage(0x0052u);
    } else {
        ruido_fm1_enter_solo_mode();
    }

    /* 5. Measure `.c1_text` SRAM 64-frame cycle cost (dual-core 2+2 Idle vs Full ZDF Stress)
     *    and re-publish the NVS-restored controller patch cleanly into `controlBus`. */
    mark_boot_stage(0x0060u);
    call_c1_set_bench_stress(2u);
    mark_boot_stage(0x0061u);
    g_ruido_fm1_status.bench_dsp64_ticks = ruido_fm1_measure_bench64_ticks();
    mark_boot_stage(0x0062u);
    call_c1_set_bench_stress(1u);
    mark_boot_stage(0x0063u);
    g_ruido_fm1_status.bench_full64_ticks = ruido_fm1_measure_bench64_ticks();
    mark_boot_stage(0x0064u);
    ruido_fm1_restore_default_patch();
    mark_boot_stage(0x0065u);

    /* 6. Prime 512-frame audio ring and arm ALNK0 consumer + live Core 1 worker */
    mark_boot_stage(0x0070u);
    ruido_fm1_arm_audio(allow_core1);
    g_idle_assist_ok = 1u;
    mark_boot_stage(0x0080u);

    last_lcd_ms = fm1_ms;
    last_lcd_done_ms = last_lcd_ms;
    last_poll_ms = last_lcd_ms - 1u;
    for (;;) {
        uint32_t now_ms = fm1_ms;

        ruido_fm1_assist_dsp();
        poll_midi_clock();
        poll_panel_inputs(now_ms);
        if (now_ms >= 3500u && fm1_crash.early != 0u) {
            fm1_crash.early = 0u;
        }

        if (now_ms != last_poll_ms || usb.ota_req || usb.uboot_req) {
            const uint8_t was_ota_req = usb.ota_req;
            last_poll_ms = now_ms;
            if (was_ota_req || usb.uboot_req) {
                /* Flush any pending Scene / Config NVS save, pause Core 1 into clean solo fallback,
                 * finish in-flight SPI1 LCD DMA, and silence ALNK0 DMA halves before
                 * `ota_session()` erases 5 x 4 KiB staging sectors. */
                g_idle_assist_ok = 0u;
                g_last_ota_kind = 0xFFu;
                fm1_crash.magic = 0u;
                fm1_crash.early = 0u;
                (void)ruido_fm1_nvs_poll_save(now_ms, 1u, nvs_save_c1_safe, nvs_tail_write_c1_safe,
                                              prepare_for_nvs_erase);
                ruido_fm1_quiesce_for_ota();
                prepare_for_nvs_erase();
            }
            run_platform_poll_safe();
            if (was_ota_req) {
                ruido_fm1_resume_after_ota();
                g_idle_assist_ok = 1u;
                last_lcd_ms = fm1_ms;
                first_frame = 1u;
            }
            sync_master_pot_target();
            g_idle_assist_ok = 0u;
            (void)ruido_fm1_nvs_poll_save(now_ms, 0u, nvs_save_c1_safe, nvs_tail_write_c1_safe,
                                          prepare_for_nvs_erase);
            g_idle_assist_ok = 1u;
            cached_cushion = ruido_fm1_audio_cushion_blocks();
            ruido_fm1_assist_dsp();
        }

        const uint32_t req_lcd_period = (cached_cushion >= 24u)
                                            ? RUIDO_LCD_FRAME_PERIOD_MS
                                            : ((cached_cushion >= 16u) ? 28u : 48u);
        const uint32_t req_lcd_gap = (cached_cushion >= 24u) ? 2u : 12u;
        if (first_frame || ((uint32_t)(now_ms - last_lcd_ms) >= req_lcd_period &&
                            (uint32_t)(now_ms - last_lcd_done_ms) >= req_lcd_gap)) {
            uint32_t elapsed_ms = first_frame ? RUIDO_LCD_FRAME_PERIOD_MS : (now_ms - last_lcd_ms);
            uint16_t y = 0u;

            first_frame = 0u;
            last_lcd_ms = now_ms;

            if (g_ruido_fm1_status.lcd_frames == 0u) {
                mark_boot_stage(0x0088u);
            }
            ruido_fm1_tick_and_prepare_ms(elapsed_ms);
            if (g_ruido_fm1_status.lcd_frames == 0u) {
                mark_boot_stage(0x0089u);
            }

            /* Persistent cross-frame ping-pong: Strip 14 DMA stays in-flight across the frame
             * boundary while `app_main` polls MIDI/panel and renders the next frame's Strip 0 into
             * `lcd_slot ^ 1`! */
            for (y = 0u; y < 240u; y = (uint16_t)(y + 16u)) {
                ruido_fm1_assist_dsp();
                poll_midi_clock();
                poll_panel_inputs(fm1_ms);
                if (fm1_ms != last_poll_ms && !usb.ota_req && !usb.uboot_req) {
                    last_poll_ms = fm1_ms;
                    run_platform_poll_safe();
                    sync_master_pot_target();
                    cached_cushion = ruido_fm1_audio_cushion_blocks();
                }
                const uint16_t* strip_be = ruido_fm1_render_strip_be(y, lcd_slot);
                ruido_fm1_assist_dsp();
                lcd_blit(0u, y, 240u, 16u, strip_be);
                lcd_slot ^= 1u;
            }
            cached_cushion = ruido_fm1_audio_cushion_blocks();
            last_lcd_done_ms = fm1_ms;
            if (g_ruido_fm1_status.lcd_frames == 0u) {
                mark_boot_stage(0x0090u);
            } else {
                /* Stage 0x0100 + elapsed seconds while running: shows how long boot #1 survived */
                g_ruido_boot_diag.stage = 0x0100u + (fm1_ms / 1000u);
                g_ruido_boot_diag.uptime_ms = fm1_ms;
                ruido_fm1_diag_sync_c1();
            }
            g_ruido_fm1_status.lcd_frames = g_ruido_fm1_status.lcd_frames + 1u;
            if (g_ruido_fm1_status.lcd_frames == 8u && bootguard.pending != 0u) {
                bootguard_clear(&bootguard);
            }
        }
    }
}
