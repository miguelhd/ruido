/* FM-1 target entry point for Ruido (built against zvenson/dinghy's GPL-3.0-only platform.c).
 * Keeps dinghy's bootguard, Hold OCT- USB rescue, OTA SysEx updater, and USB-MIDI/Audio intact
 * while constructing all C++17 engine objects and launching Core 1 strictly AFTER `recovery_key()`
 * clears. */

#define APP_NAME "RUIDO"
#define APP_VER "1.0"
#define STORE_MAGIC 0x52554944u /* "RUID": distinct NVS magic from stock dinghy forks */
#define CV_MAX (240u * 16u)     /* Unused gfx.c canvas shrunk (GC'd by --gc-sections) */
#define LCD_BAUD 1u /* Fast ~30 MHz SPI1 ST7789 scanline DMA (vs default BAUD 4 = 12 MHz) */

#include "fm1_ops_screen.h"

/* Redirect dinghy's 1-byte-at-a-time libc.c memset/memcpy & wrap TIMER5 ISR cleanly */
#define memset dinghy_byte_memset
#define memcpy dinghy_byte_memcpy
#define fm1_timer5_irq dinghy_timer5_irq

static void app_idle(void);
static void ruido_fm1_plat_ota_show(uint32_t step, int32_t code);
static void ruido_fm1_plat_rescue_show(uint8_t is_flash_ok);
static void ruido_fm1_plat_uboot_show(void);
static void ruido_fm1_plat_fault_show(uint32_t vec, uint32_t pc, uint32_t rets);
#define LCD_IDLE app_idle
#include "platform_ruido.c"

#undef fm1_timer5_irq
#undef memset
#undef memcpy

void* memset(void* dst, int c, unsigned n) {
    uint8_t* p = (uint8_t*)dst;
    const uint8_t b = (uint8_t)c;
    while (n > 0u && (((uintptr_t)p & 3u) != 0u)) {
        *p++ = b;
        --n;
    }
    if (n >= 16u) {
        const uint32_t w =
            (uint32_t)b | ((uint32_t)b << 8u) | ((uint32_t)b << 16u) | ((uint32_t)b << 24u);
        uint32_t* wp = (uint32_t*)p;
        while (n >= 16u) {
            wp[0] = w;
            wp[1] = w;
            wp[2] = w;
            wp[3] = w;
            wp += 4;
            n -= 16u;
        }
        while (n >= 4u) {
            *wp++ = w;
            n -= 4u;
        }
        p = (uint8_t*)wp;
    }
    while (n-- > 0u) {
        *p++ = b;
    }
    return dst;
}

void* memcpy(void* dst, const void* src, unsigned n) {
    uint8_t* d = (uint8_t*)dst;
    const uint8_t* s = (const uint8_t*)src;
    if (n >= 16u && (((uintptr_t)d & 3u) == ((uintptr_t)s & 3u))) {
        while (((uintptr_t)d & 3u) != 0u) {
            *d++ = *s++;
            --n;
        }
        uint32_t* wd = (uint32_t*)d;
        const uint32_t* ws = (const uint32_t*)s;
        while (n >= 16u) {
            wd[0] = ws[0];
            wd[1] = ws[1];
            wd[2] = ws[2];
            wd[3] = ws[3];
            wd += 4;
            ws += 4;
            n -= 16u;
        }
        while (n >= 4u) {
            *wd++ = *ws++;
            n -= 4u;
        }
        d = (uint8_t*)wd;
        s = (const uint8_t*)ws;
    }
    while (n-- > 0u) {
        *d++ = *s++;
    }
    return dst;
}

#include "fm1_cpu1.h"
#include "ruido_fm1_bridge.h"

#define RUIDO_UNITY_MASTER_Q12 4096
#define RUIDO_LCD_FRAME_PERIOD_MS 16u
#define RUIDO_LED_BG_PULSE_NS 2000u

static volatile uint8_t g_in_platform_poll = 0u;
static uint32_t g_idle_assist_ok = 0u;
static uint8_t g_in_app_idle = 0u;
static uint8_t g_lcd_inited = 0u;
static uint8_t g_last_ota_kind = 0xFFu;
static uint16_t g_ops_strip_be[240u * 16u] __attribute__((section(".pool")));

static void ruido_fm1_plat_blit_ops_screen(const RuidoOpsState* st, int use_ota_idle) {
    uint16_t y = 0u;
    g_idle_assist_ok = 0u;
    for (y = 0u; y < 240u; y = (uint16_t)(y + 16u)) {
        if (use_ota_idle) {
            ota_idle();
        } else {
            fm1_wdt_feed();
        }
        lcd_sync();
        ruido_ops_render_strip_be(st, y, 16u, g_ops_strip_be);
        lcd_blit(0u, y, 240u, 16u, g_ops_strip_be);
    }
    lcd_sync();
}

static void ruido_fm1_plat_rescue_show(uint8_t is_flash_ok) {
    RuidoOpsState st = {
        (uint8_t)(is_flash_ok ? RUIDO_OPS_RESCUE_READY : RUIDO_OPS_RESCUE_BADFLASH),
        0u,
        0u,
        0u,
        0u,
        0u,
    };
    g_lcd_inited = 1u;
    g_last_ota_kind = 0xFFu;
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

static void ruido_fm1_plat_ota_show(uint32_t step, int32_t code) {
    RuidoOpsState st = {RUIDO_OPS_OTA_CHECK, 0u, 0u, 0u, 0u, 0u};
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
    RuidoOpsState st = {RUIDO_OPS_UBOOT, 0u, 0u, 0u, 0u, 0u};
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

static void ruido_fm1_plat_fault_show(uint32_t vec, uint32_t pc, uint32_t rets) {
    RuidoOpsState st = {RUIDO_OPS_SYSTEM_FAULT, 0u, 0u, vec, pc, rets};
    g_idle_assist_ok = 0u;
    if (!g_lcd_inited) {
        lcd_init();
        g_lcd_inited = 1u;
    }
    ruido_fm1_plat_blit_ops_screen(&st, 0);
}

/* Fast bounded 10-bit SARADC single conversion for ISR carbon-pill sampling (`ch 0` PA7 / `ch 1`
 * PA8). Uses a tight 1,024-iteration timeout (~12 us ceiling) so ISR execution stays strictly
 * bounded. */
static int32_t sample_saradc_10b_isr(uint32_t ch) {
    uint32_t t = 0u;
    uint32_t v = 0u;
    FM1_ADC_CON = 0u;
    if (FM1_WLA_CON0 & (1u << 14)) {
        FM1_WLA_CON0 &= ~(1u << 14);
    }
    FM1_ADC_CON = 0xF04Eu | ((ch & 0xFu) << 8u);
    FM1_ADC_CON |= 0x10u;
    FM1_ADC_CON |= 0x40u;
    for (t = 0u; t < 1024u; ++t) {
        if (FM1_ADC_CON & 0x80u) {
            break;
        }
    }
    v = FM1_ADC_RES & 0x3FFu;
    FM1_ADC_CON = 0x40u;
    FM1_ADC_CON = 0u;
    return (t == 1024u) ? -1 : (int32_t)v;
}

/* 10 kHz TIMER5 wrapper:
 * 1. Runs dinghy's column scan, 1 ms tick, USB poll, and 2 kHz TRS UART1 MIDI DMA poll.
 * 2. Steps our 42-channel staggered-phase Delta-Sigma LED modulator at 909.1 Hz (`fm1_in.frames`).
 * 3. Samples one held PA7/PA8 silicone carbon-pill key right after its 74HC595 column latches low
 *    (with `g_in_platform_poll == 0` so `FM1_ADC_CON` never collides with `PB6` MASTER pot reads,
 *    LED anodes blanked for zero sink offset, dummy S&H discharge read, and > 55 us RC recovery
 *    before the next digital `fm1__rows()` read). */
void fm1_timer5_irq(void) {
    static uint32_t adc_sampled_frame = 0xFFFFFFFFu;
    const uint32_t t_enter = fm1_ticks();
    const uint32_t prev_frames = fm1_in.frames;

    dinghy_timer5_irq();

    if (fm1_in.frames != prev_frames) {
        ruido_fm1_led_step_909hz(fm1_led, fm1_led_dim, fm1_led_bg);
    }

    if (!g_in_platform_poll && !in_audio && adc_sampled_frame != fm1_in.frames &&
        (uint32_t)(fm1_ticks() - t_enter) < (40u * FM1_TICKS_PER_US)) {
        uint8_t req_note = 0u;
        uint8_t req_col = 0u;
        uint8_t req_ch = 0u;
        uint8_t req_pin = 0u;
        if (ruido_fm1_adc_get_req(&req_note, &req_col, &req_ch, &req_pin) != 0u &&
            fm1__tick_col == req_col) {
            const uint32_t m = 1u << req_pin;
            int32_t val = -1;

            adc_sampled_frame = fm1_in.frames;

            /* Blank column LED anodes during the ~8 us read so 74HC595 sink sits at 0.00V GND */
            fm1__led_lines(0u);

            /* Weak 2.4 mA high-side pull + disable Schmitt input (`DIE=0`) on PA7/PA8 */
            FM1_PR(FM1_PA, FM1_DIE) &= ~m;
            FM1_PR(FM1_PA, FM1_HD0) &= ~m;
            FM1_PR(FM1_PA, FM1_HD) &= ~m;
            FM1_PR(FM1_PA, FM1_PU) |= m;
            FM1_PR(FM1_PA, FM1_OUT) |= m;
            FM1_PR(FM1_PA, FM1_DIR) &= ~m;

            /* Dummy conversion discharges PB6 MASTER pot charge on S&H cap; 2nd read is settled */
            (void)sample_saradc_10b_isr(req_ch);
            val = sample_saradc_10b_isr(req_ch);

            /* Restore digital pull-up matrix row input + column LED state (> 50 us before next
             * tick) */
            FM1_PR(FM1_PA, FM1_DIR) |= m;
            FM1_PR(FM1_PA, FM1_OUT) &= ~m;
            FM1_PR(FM1_PA, FM1_PU) |= m;
            FM1_PR(FM1_PA, FM1_DIE) |= m;
            fm1__led_lines(fm1_led[fm1__tick_col]);

            if (val >= 0) {
                ruido_fm1_adc_put_sample(req_note, val);
            }
        }
    }
}

/* Intercept dinghy's `[0..4088]` integer `master_q12` on Core 0 main loop so `MASTER` pot scaling
 * happens smoothly in float32 inside `DroneEngineCore` BEFORE TPDF quantization, while `platform.c`
 * sees exact unity (4096). */
static inline void sync_master_pot_target(void) {
    int32_t mq = master_q12;
    if (mq != RUIDO_UNITY_MASTER_Q12) {
        master_q12 = RUIDO_UNITY_MASTER_Q12;
        ruido_fm1_set_master_q12((uint32_t)(mq > 0 ? mq : 0));
    }
}

static void poll_midi_clock(void) {
    uint8_t st = 0u;
    uint8_t d1 = 0u;
    uint8_t d2 = 0u;
    while (midi_get(&st, &d1, &d2) != 0) {
        if (st == 0xF8u || st == 0xFAu || st == 0xFBu || st == 0xFCu) {
            ruido_fm1_on_midi_rt(st);
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

    ruido_fm1_poll_panel(now_ms, pressed_btns, released_btns, cur_btns, enc_steps, cur_notes,
                         note_edges);
}

static void prepare_for_nvs_erase(void) {
    /* Finish any in-flight SPI1 LCD DMA strip and silence ALNK0 DMA halves so the ~35 ms IRQ-masked
     * 4 KiB sector erase outputs clean silence instead of looping an 11.6 ms (86 Hz) grain. */
    lcd_sync();
    memset(abuf, 0, sizeof(abuf));
}

static void app_idle(void) {
    if (g_idle_assist_ok && !g_in_app_idle) {
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

static void app_main(void) {
    Fm1ClockSnapshot clk_snap = {0u, 0u, 0u, 0u};
    uint32_t init_mq = 2048u;
    uint32_t allow_core1 = 1u;
    uint32_t last_lcd_ms = 0u;
    uint32_t last_poll_ms = 0u;
    uint32_t first_frame = 1u;
    uint8_t lcd_slot = 0u;

    /* Reachable ONLY after fm1_cstart() cleared bootguard AND Hold OCT- USB rescue! */
    fm1_led_bg_ns = RUIDO_LED_BG_PULSE_NS;

    /* Settle platform.c's 1-pole `knob_avg` IIR (`tau = 8` samples from 512) to the true physical
     * PB6 MASTER pot voltage (~0.3 ms) so initial `init_mq` never triggers a phantom boot toast. */
    g_in_platform_poll = 1u;
    for (uint32_t settle = 0u; settle < 48u; ++settle) {
        platform_poll();
    }
    g_in_platform_poll = 0u;
    if (master_q12 >= 0) {
        init_mq = (uint32_t)master_q12;
    }
    master_q12 = RUIDO_UNITY_MASTER_Q12;

    /* 1. Copy Core 1 hot DSP `.text` + `.rodata` (`~60 KiB`) from XIP into 0x01C0E000 SRAM */
    fm1_cpu1_load_sram();

    /* 2. Placement-new construct C++17 engine arenas in `.pool` SRAM (audio still disarmed) */
    ruido_fm1_construct(init_mq);

    /* 3. Snapshot AC791N clock dividers, check silicon-lottery early-crash flag, and boot Core 1 */
    g_ruido_fm1_status.cal_8k_insns_ticks = calibrate_8k_insns_ticks();
    fm1_clock_snapshot(&clk_snap);
    g_ruido_fm1_status.clk_con0_div =
        ((clk_snap.sys_div & 0xFFu) << 24u) | (clk_snap.clk_con0 & 0x00FFFFFFu);

    if (fm1_crash.magic == FM1_CRASH_MAGIC && fm1_crash.early > 0u) {
        allow_core1 = 0u;
    }

    if (allow_core1 != 0u) {
        ruido_fm1_set_coproc_enabled(1u);
        fm1_cpu1_start(fm1_cpu1_entry);
        g_ruido_fm1_status.c1_flags = g_ruido_fm1_status.c1_flags | 1u;
    }

    /* 4. Measure `.c1_text` SRAM 64-frame cycle cost (dual-core 2+2 Idle vs Full ZDF Stress) */
    g_ruido_fm1_status.bench_dsp64_ticks = ruido_fm1_measure_bench64_ticks();
    call_c1_set_bench_stress(1u);
    g_ruido_fm1_status.bench_full64_ticks = ruido_fm1_measure_bench64_ticks();
    ruido_fm1_restore_default_patch();

    /* 5. Restore CRC-verified Scenes (0xF4000..0xF7FFF) & DeviceSettings (0xF8000) before audio */
    (void)ruido_fm1_nvs_load_boot(blob_load);

    /* 6. Prime 512-frame audio ring and arm ALNK0 consumer + live Core 1 worker */
    ruido_fm1_arm_audio(allow_core1);
    g_lcd_inited = 1u;
    g_idle_assist_ok = 1u;

    last_lcd_ms = fm1_ms;
    last_poll_ms = last_lcd_ms - 1u;
    for (;;) {
        uint32_t now_ms = fm1_ms;

        ruido_fm1_assist_dsp();
        poll_midi_clock();
        poll_panel_inputs(now_ms);

        if (now_ms != last_poll_ms) {
            const uint8_t was_ota_req = usb.ota_req;
            last_poll_ms = now_ms;
            if (was_ota_req || usb.uboot_req) {
                /* Flush any pending Scene / Config NVS save, pause Core 1 into clean solo fallback,
                 * finish in-flight SPI1 LCD DMA, and silence ALNK0 DMA halves before
                 * `ota_session()` erases 5 x 4 KiB staging sectors. */
                g_idle_assist_ok = 0u;
                g_last_ota_kind = 0xFFu;
                (void)ruido_fm1_nvs_poll_save(now_ms, 1u, blob_save, prepare_for_nvs_erase);
                ruido_fm1_quiesce_for_ota();
                prepare_for_nvs_erase();
            }
            g_in_platform_poll = 1u;
            platform_poll();
            g_in_platform_poll = 0u;
            if (was_ota_req) {
                g_idle_assist_ok = 1u;
                last_lcd_ms = fm1_ms;
                first_frame = 1u;
            }
            sync_master_pot_target();
            if (ruido_fm1_nvs_poll_save(now_ms, 0u, blob_save, prepare_for_nvs_erase) > 0u) {
                uint32_t rb_slot1[35];
                uint32_t rb_settings[7];
                uint32_t w = 0u;
                if (st_read(BLOBS_AT + 1u * 0x1000u, rb_slot1, sizeof(rb_slot1)) == 0) {
                    for (w = 0u; w < 35u; ++w) {
                        g_ruido_fm1_status.nvs_flash_readback[w] = rb_slot1[w];
                    }
                }
                if (st_read(BLOBS_AT + 4u * 0x1000u, rb_settings, sizeof(rb_settings)) == 0) {
                    for (w = 0u; w < 7u; ++w) {
                        g_ruido_fm1_status.nvs_flash_readback[35u + w] = rb_settings[w];
                    }
                }
            }
            ruido_fm1_assist_dsp();
        }

        if (first_frame || (uint32_t)(now_ms - last_lcd_ms) >= RUIDO_LCD_FRAME_PERIOD_MS) {
            uint32_t elapsed_ms = first_frame ? RUIDO_LCD_FRAME_PERIOD_MS : (now_ms - last_lcd_ms);
            uint16_t y = 0u;

            first_frame = 0u;
            last_lcd_ms = now_ms;

            ruido_fm1_tick_and_prepare_ms(elapsed_ms);

            /* Persistent cross-frame ping-pong: Strip 14 DMA stays in-flight across the frame
             * boundary while `app_main` polls MIDI/panel and renders the next frame's Strip 0 into
             * `lcd_slot ^ 1`! */
            for (y = 0u; y < 240u; y = (uint16_t)(y + 16u)) {
                ruido_fm1_assist_dsp();
                poll_midi_clock();
                const uint16_t* strip_be = ruido_fm1_render_strip_be(y, lcd_slot);
                ruido_fm1_assist_dsp();
                lcd_blit(0u, y, 240u, 16u, strip_be);
                lcd_slot ^= 1u;
            }
            g_ruido_fm1_status.lcd_frames = g_ruido_fm1_status.lcd_frames + 1u;
        }
    }
}
