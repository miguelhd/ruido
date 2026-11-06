#pragma once

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define RUIDO_FM1_STATUS_MAGIC 0x52554944u /* "RUID" */

typedef struct {
    volatile uint32_t magic;
    volatile uint32_t audio_blocks;
    volatile uint32_t lcd_frames;
    volatile uint32_t bench_dsp64_ticks;
    volatile uint32_t bench_full64_ticks;
    volatile uint32_t cal_8k_insns_ticks;
    volatile uint32_t last_audio256_ticks;
    volatile int32_t peak_abs_q15;
} RuidoFm1Status;

extern RuidoFm1Status g_ruido_fm1_status;

/* Placement-new construct DroneEngine, RuidoController, and ScopeHudRenderer in .pool SRAM.
 * MUST be invoked strictly AFTER dinghy's power-on Hold OCT- USB rescue check clears. */
void ruido_fm1_init(void);

/* Configure a 4-voice nonlinear MS-20 self-oscillation + overdrive + 8x LFO stress patch, or
 * restore clean boot defaults (`stress_mode == 0`) after benchmarking. */
void ruido_fm1_set_bench_stress(uint32_t stress_mode);

/* Render one 64-frame 4x-oversampled DSP block into scratch buffers for cycle benchmarking. */
void ruido_fm1_step_bench64(void);

/* Fill `num_frames` stereo interleaved Q15 frames (`out_lr_q15` is pre-zeroed by platform.c). */
void ruido_fm1_audio(int32_t* out_lr_q15, uint32_t num_frames);

/* Route 1-byte MIDI System Real-Time clock/transport (0xF8/0xFA/0xFB/0xFC) to MidiClockTracker. */
void ruido_fm1_on_midi_rt(uint8_t status_byte);

/* Advance controller state, ingest scope ring, and prepare coupled 240x240 frame curves. */
void ruido_fm1_tick_and_prepare_ms(uint32_t elapsed_ms);

/* Render a 240x16 scanline strip at `y_start` into `.pool` ping-pong slot `slot & 1` and swap
 * RGB565 pixels to big-endian for ST7789 SPI1 DMA (`0 B` stack copy). */
const uint16_t* ruido_fm1_render_strip_be(uint16_t y_start, uint8_t slot);

#ifdef __cplusplus
}
#endif
