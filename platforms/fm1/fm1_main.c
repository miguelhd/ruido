/* FM-1 target entry point for Ruido (built against zvenson/dinghy's GPL-3.0-only platform.c).
 * Keeps dinghy's bootguard, Hold OCT- USB rescue, OTA SysEx updater, and USB-MIDI/Audio intact
 * while constructing all C++17 engine objects and launching Core 1 strictly AFTER `recovery_key()`
 * clears. */

#define APP_NAME "RUIDO"
#define STORE_MAGIC 0x52554944u /* "RUID": distinct NVS magic from stock dinghy forks */
#define CV_MAX (240u * 32u)     /* 15 KiB canvas for platform rescue/OTA FONT_S/FONT_L text boxes */

/* Redirect dinghy's 1-byte-at-a-time libc.c memset/memcpy so our 32-bit word versions link */
#define memset dinghy_byte_memset
#define memcpy dinghy_byte_memcpy

static void app_idle(void);
#define LCD_IDLE app_idle
#include "platform.c"

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

static uint32_t g_idle_assist_ok = 0u;

/* Intercept dinghy's `[0..4088]` integer `master_q12` so `MASTER` pot scaling happens smoothly in
 * float32 BEFORE 1-LSB TPDF quantization on Core 1, while `platform.c` sees exact unity (4096). */
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

static void app_idle(void) {
    if (g_idle_assist_ok) {
        ruido_fm1_assist_dsp();
        poll_midi_clock();
    }
}

static void app_audio(int32_t* out, uint32_t n) {
    uint32_t t0 = fm1_ticks();
    sync_master_pot_target();
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

    /* Reachable ONLY after fm1_cstart() cleared bootguard AND Hold OCT- USB rescue! */
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

    /* 5. Prime 256-frame audio ring and arm ALNK0 consumer + live Core 1 worker */
    ruido_fm1_arm_audio(allow_core1);

    last_lcd_ms = fm1_ms;
    last_poll_ms = last_lcd_ms - 1u;
    for (;;) {
        uint32_t now_ms = fm1_ms;

        ruido_fm1_assist_dsp();
        poll_midi_clock();

        if (now_ms != last_poll_ms) {
            last_poll_ms = now_ms;
            platform_poll();
            sync_master_pot_target();
            ruido_fm1_assist_dsp();
        }

        if (first_frame || (uint32_t)(now_ms - last_lcd_ms) >= RUIDO_LCD_FRAME_PERIOD_MS) {
            uint32_t elapsed_ms = first_frame ? RUIDO_LCD_FRAME_PERIOD_MS : (now_ms - last_lcd_ms);
            uint16_t y = 0u;
            uint8_t slot = 0u;

            first_frame = 0u;
            last_lcd_ms = now_ms;

            ruido_fm1_tick_and_prepare_ms(elapsed_ms);

            g_idle_assist_ok = 1u;
            for (y = 0u, slot = 0u; y < 240u; y = (uint16_t)(y + 16u), slot ^= 1u) {
                ruido_fm1_assist_dsp();
                const uint16_t* strip_be = ruido_fm1_render_strip_be(y, slot);
                ruido_fm1_assist_dsp();
                lcd_blit(0u, y, 240u, 16u, strip_be);
            }
            lcd_sync();
            g_idle_assist_ok = 0u;
            g_ruido_fm1_status.lcd_frames = g_ruido_fm1_status.lcd_frames + 1u;
        }
    }
}
