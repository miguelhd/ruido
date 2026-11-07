/* First-party standalone RAM update loader screen for Ruido (`loader.bin` @ 0x01C0A800).
 * Included directly into staged `loader_ruido.c` in place of Dinghy's `ldr_screen.c`, rendering
 * Ruido's anti-aliased monochrome sans-serif UI with zero external website URLs and zero XIP reads.
 */
#pragma once

#include "fm1_ops_screen.h"

static void ldr_poll(void);

static uint16_t ldr_strip_be[240u * 16u];
static uint16_t ldr_last_px = 0xFFFFu;
static uint8_t ldr_last_pct = 0xFFu;
static uint8_t ldr_writing = 0u;
static uint8_t ldr_idle = 1u;
static uint8_t ldr_glider_dir = 0u;
static uint16_t ldr_glider_pos = 0u;
static uint32_t ldr_next_anim_ms = 0u;

/* Exact bounded integer quotient min(num / den, max_q) without __udivsi3 libcalls (`max_q <= 255`).
 */
static uint32_t ldr_udiv_bounded(uint32_t num, uint32_t den, uint32_t max_q) {
    uint32_t q = 0u;
    int bit = 7;
    if (den == 0u) {
        return max_q;
    }
    for (; bit >= 0; --bit) {
        uint32_t cand = q | (1u << (unsigned)bit);
        if (cand <= max_q && num >= cand * den) {
            q = cand;
        }
    }
    return q;
}

static void ldr_blit_strip(const RuidoOpsState* st, uint16_t y0) {
    lcd_sync();
    ruido_ops_render_strip_be(st, y0, 16u, ldr_strip_be);
    lcd_blit(0u, y0, 240u, 16u, ldr_strip_be);
}

static void ldr_blit_full(const RuidoOpsState* st, int poll_usb) {
    uint16_t y = 0u;
    for (y = 0u; y < 240u; y = (uint16_t)(y + 16u)) {
        if (poll_usb) {
            ldr_poll();
        } else {
            fm1_wdt_feed();
        }
        ldr_blit_strip(st, y);
    }
    lcd_sync();
}

static void ldr_screen(void) {
    RuidoOpsState st = {RUIDO_OPS_LDR_WAITING, 0u, 0u, 0u, 0u, 0u};
    lcd_init();
    ldr_writing = 0u;
    ldr_idle = 1u;
    ldr_glider_dir = 0u;
    ldr_glider_pos = 0u;
    ldr_last_px = 0u;
    ldr_last_pct = 0xFFu;
    ldr_next_anim_ms = 0u;
    ldr_blit_full(&st, 0);
}

static void ldr_anim(uint32_t ms) {
    RuidoOpsState st = {RUIDO_OPS_LDR_WAITING, 0u, 0u, 0u, 0u, 0u};
    if (!ldr_idle || (int32_t)(ms - ldr_next_anim_ms) < 0) {
        return;
    }
    ldr_next_anim_ms = ms + 24u;
    /* 184 px travel each way in 4 px steps -> ~2.2 s smooth back-and-forth glide, 0 divisions */
    if (ldr_glider_dir == 0u) {
        if (ldr_glider_pos + 4u >= 184u) {
            ldr_glider_pos = 184u;
            ldr_glider_dir = 1u;
        } else {
            ldr_glider_pos = (uint16_t)(ldr_glider_pos + 4u);
        }
    } else {
        if (ldr_glider_pos <= 4u) {
            ldr_glider_pos = 0u;
            ldr_glider_dir = 0u;
        } else {
            ldr_glider_pos = (uint16_t)(ldr_glider_pos - 4u);
        }
    }
    if (ldr_glider_pos == ldr_last_px) {
        return;
    }
    ldr_last_px = ldr_glider_pos;
    st.bar_px = ldr_glider_pos;
    ldr_blit_strip(&st, 112u);
    lcd_sync();
}

static void ldr_bar(uint32_t done, uint32_t total) {
    uint32_t safe_total = (total > 0u) ? total : 1u;
    uint16_t fill_px =
        (done >= safe_total) ? 220u : (uint16_t)ldr_udiv_bounded(done * 220u, safe_total, 220u);
    uint8_t pct =
        (done >= safe_total) ? 100u : (uint8_t)ldr_udiv_bounded(done * 100u, safe_total, 100u);
    RuidoOpsState st = {RUIDO_OPS_LDR_WRITING, pct, fill_px, 0u, 0u, 0u};

    if (!ldr_writing || fill_px < ldr_last_px) {
        ldr_idle = 0u;
        ldr_writing = 1u;
        ldr_last_px = fill_px;
        ldr_last_pct = pct;
        ldr_blit_full(&st, 1);
        return;
    }

    if (pct != ldr_last_pct) {
        ldr_last_pct = pct;
        ldr_blit_strip(&st, 0u);
        ldr_poll();
        ldr_blit_strip(&st, 16u);
    }
    if (fill_px != ldr_last_px) {
        ldr_last_px = fill_px;
        ldr_blit_strip(&st, 112u);
    }
    lcd_sync();
}

static void ldr_failed(void) {
    RuidoOpsState st = {RUIDO_OPS_LDR_FAILED, 0u, 0u, 0u, 0u, 0u};
    ldr_writing = 0u;
    ldr_idle = 0u;
    ldr_last_px = 0xFFFFu;
    ldr_last_pct = 0xFFu;
    ldr_blit_full(&st, 1);
}
