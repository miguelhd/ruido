/* FM-1 target entry point for Ruido (built against zvenson/dinghy's GPL-3.0-only platform.c).
 * Keeps dinghy's bootguard, Hold OCT- USB rescue, OTA SysEx updater, and USB-MIDI/Audio intact
 * while constructing all C++17 engine objects strictly AFTER recovery_key() clears. */

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

#include "ruido_fm1_bridge.h"

static uint32_t g_idle_midi_ok = 0u;

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
    if (g_idle_midi_ok) {
        poll_midi_clock();
    }
}

static void app_audio(int32_t* out, uint32_t n) {
    uint32_t t0 = fm1_ticks();
    ruido_fm1_audio(out, n);
    g_ruido_fm1_status.last_audio256_ticks = fm1_ticks() - t0;
}

static uint32_t measure_bench64_ticks(void) {
    uint32_t best_ticks = 0xFFFFFFFFu;
    uint32_t pass = 0u;
    for (pass = 0u; pass < 3u; ++pass) {
        uint32_t irq_f = irq_save();
        uint32_t t0 = fm1_ticks();
        ruido_fm1_step_bench64();
        uint32_t dt = fm1_ticks() - t0;
        irq_restore(irq_f);
        if (dt > 0u && dt < best_ticks) {
            best_ticks = dt;
        }
    }
    return best_ticks;
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
    uint32_t last_ms = 0u;

    /* Reachable ONLY after fm1_cstart() cleared bootguard AND Hold OCT- USB rescue! */
    ruido_fm1_init();
    g_ruido_fm1_status.cal_8k_insns_ticks = calibrate_8k_insns_ticks();
    g_ruido_fm1_status.bench_dsp64_ticks = measure_bench64_ticks();
    ruido_fm1_set_bench_stress(1u);
    g_ruido_fm1_status.bench_full64_ticks = measure_bench64_ticks();
    ruido_fm1_set_bench_stress(0u);

    last_ms = fm1_ms;
    for (;;) {
        uint32_t now_ms = fm1_ms;
        uint32_t elapsed_ms = now_ms - last_ms;
        uint16_t y = 0u;
        uint8_t slot = 0u;

        platform_poll();
        poll_midi_clock();

        if (elapsed_ms > 0u) {
            last_ms = now_ms;
        }

        ruido_fm1_tick_and_prepare_ms(elapsed_ms);

        g_idle_midi_ok = 1u;
        for (y = 0u, slot = 0u; y < 240u; y = (uint16_t)(y + 16u), slot ^= 1u) {
            const uint16_t* strip_be = ruido_fm1_render_strip_be(y, slot);
            lcd_blit(0u, y, 240u, 16u, strip_be);
        }
        lcd_sync();
        g_idle_midi_ok = 0u;
        g_ruido_fm1_status.lcd_frames = g_ruido_fm1_status.lcd_frames + 1u;
    }
}
