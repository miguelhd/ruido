#pragma once

/* Locked USB-MIDI diagnostic telemetry & virtual-panel test interface (included by `fm1_main.c`
 * after `platform_ruido.c`). Strictly inert until unlocked over USB cable 0 via `CC 119 = 0x52`,
 * `CC 119 = 0x55` ("RU"), enabling automated real-silicon underrun & CPU sweeps across all modes.
 */

static uint8_t g_diag_unlock_step = 0u;
static uint8_t g_virt_pressure_7b = 127u;
static uint32_t g_virt_buttons = 0u;
static uint32_t g_prev_virt_buttons = 0u;
static uint32_t g_virt_notes = 0u;
static uint32_t g_prev_virt_notes = 0u;
static int32_t g_virt_enc[FM1_NENC];

static uint32_t read_diag_word(uint32_t idx) {
    ruido_fm1_diag_sync_c1();
    if (idx < 16u) {
        return ((const volatile uint32_t*)&g_ruido_fm1_status)[idx];
    }
    switch (idx) {
    case 16u:
        return g_ruido_boot_diag.c1_state;
    case 17u:
        return g_ruido_boot_diag.c1_code;
    case 18u:
        return ruido_fm1_diag_max_c1_ticks();
    case 19u:
#if FELUCCA_UAC
        return (uac.underruns & 0xFFFFu) | ((uac.overruns & 0xFFFFu) << 16u);
#else
        return 0u;
#endif
    case 20u:
#if FELUCCA_UAC
        return (uac.missed & 0xFFFFu) | (((uint32_t)uac.r48 & 1u) << 24u) |
               (((uint32_t)uac.flowing & 1u) << 25u);
#else
        return 0u;
#endif
    case 21u:
        return g_ruido_boot_diag.stage;
    default:
        return 0u;
    }
}

static void handle_usb_diag_cc(uint8_t cc, uint8_t val) {
    if (cc == 119u) {
        if (val == 0x52u) {
            g_diag_unlock_step = 1u;
            return;
        }
        if (val == 0x55u && g_diag_unlock_step == 1u) {
            g_diag_unlock_step = 2u;
            return;
        }
        if (val == 0u) {
            g_diag_unlock_step = 0u;
            g_virt_buttons = 0u;
            g_virt_notes = 0u;
            return;
        }
        if (g_diag_unlock_step == 2u && val == 2u) {
            ruido_fm1_diag_clear_counters();
#if FELUCCA_UAC
            uac.underruns = 0u;
            uac.overruns = 0u;
            uac.missed = 0u;
#endif
            return;
        }
        if (g_diag_unlock_step == 2u && val == 3u) {
            g_virt_buttons = 0u;
            g_virt_notes = 0u;
            return;
        }
        if (g_diag_unlock_step == 1u) {
            g_diag_unlock_step = 0u;
        }
        return;
    }
    if (g_diag_unlock_step != 2u) {
        return;
    }
    if (cc == 118u) {
        const uint32_t w = read_diag_word(val);
        midi_send(0xBFu, 115u, val & 0x7Fu);
        midi_send(0xBFu, 110u, w & 0x7Fu);
        midi_send(0xBFu, 111u, (w >> 7u) & 0x7Fu);
        midi_send(0xBFu, 112u, (w >> 14u) & 0x7Fu);
        midi_send(0xBFu, 113u, (w >> 21u) & 0x7Fu);
        midi_send(0xBFu, 114u, (w >> 28u) & 0x0Fu);
    } else if (cc == 101u) {
        g_virt_pressure_7b = val & 0x7Fu;
    } else if (cc >= 102u && cc <= 105u) {
        const uint32_t shift = (uint32_t)(cc - 102u) * 7u;
        const uint32_t mask = 0x7Fu << shift;
        g_virt_notes =
            ((g_virt_notes & ~mask) | (((uint32_t)(val & 0x7Fu)) << shift)) & ((1u << 27u) - 1u);
    } else if (cc == 106u || cc == 107u) {
        const uint32_t shift = (uint32_t)(cc - 106u) * 7u;
        const uint32_t mask = 0x7Fu << shift;
        g_virt_buttons =
            ((g_virt_buttons & ~mask) | (((uint32_t)(val & 0x7Fu)) << shift)) & ((1u << 14u) - 1u);
    } else if (cc == 108u) {
        const uint32_t enc = (val >> 4u) & 7u;
        const int32_t delta = (int32_t)(val & 0x0Fu) - 8;
        if (enc < FM1_NENC) {
            g_virt_enc[enc] += delta;
        }
    }
}

static inline void apply_usb_diag_virtual_inputs(uint32_t* pressed_btns, uint32_t* released_btns,
                                                 uint32_t* cur_btns, uint32_t* note_edges,
                                                 uint32_t* cur_notes, int32_t enc_steps[FM1_NENC]) {
    uint32_t e = 0u;
    if (g_virt_buttons != 0u || g_prev_virt_buttons != 0u) {
        *pressed_btns |= (g_virt_buttons & ~g_prev_virt_buttons);
        *released_btns |= (~g_virt_buttons & g_prev_virt_buttons);
        *cur_btns |= g_virt_buttons;
        g_prev_virt_buttons = g_virt_buttons;
    }
    if (g_virt_notes != 0u || g_prev_virt_notes != 0u) {
        const int32_t v_adc = 815 - (((int32_t)g_virt_pressure_7b * 305) / 127);
        uint32_t n = 0u;
        *note_edges |= (g_virt_notes & ~g_prev_virt_notes);
        *cur_notes |= g_virt_notes;
        g_prev_virt_notes = g_virt_notes;
        for (n = 0u; n < 27u; ++n) {
            if (((g_virt_notes >> n) & 1u) != 0u) {
                ruido_fm1_adc_put_sample((uint8_t)n, v_adc);
            }
        }
    }
    for (e = 0u; e < FM1_NENC; ++e) {
        if (g_virt_enc[e] != 0) {
            enc_steps[e] += g_virt_enc[e];
            g_virt_enc[e] = 0;
        }
    }
}
