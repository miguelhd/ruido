#pragma once

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define RUIDO_FM1_STATUS_MAGIC 0x52554944u /* "RUID" */

typedef struct {
    volatile uint32_t magic;               /* [0] 0x52554944 once init + bench + arm complete */
    volatile uint32_t audio_blocks;        /* [1] Core 0 ALNK0 256-frame DMA callbacks serviced */
    volatile uint32_t lcd_frames;          /* [2] Core 0 240x240 scanline frames pushed */
    volatile uint32_t bench_dsp64_ticks;   /* [3] .c1_text SRAM 64-frame Idle Drone ticks */
    volatile uint32_t bench_full64_ticks;  /* [4] .c1_text SRAM 64-frame Full ZDF Stress ticks */
    volatile uint32_t cal_8k_insns_ticks;  /* [5] 8,000-instruction calibration in TIMER4 ticks */
    volatile uint32_t last_audio256_ticks; /* [6] Core 0 ALNK0 256-frame ISR ring-drain ticks */
    volatile int32_t peak_abs_q15;         /* [7] Peak |sample| in [-32767, +32767] */
    volatile uint32_t c1_blocks;           /* [8] 64-frame DSP blocks rendered by Core 1 */
    volatile uint32_t c1_last64_ticks;     /* [9] Live Core 1 64-frame render duration (ticks) */
    volatile uint32_t clk_con0_div;        /* [10] (sys_div << 24) | (clk_con0 & 0x00FFFFFF) */
    volatile uint32_t c1_flags; /* [11] b0=started, b1=fallback, [15:8]=underruns, [31:16]=assist */
} RuidoFm1Status;

extern RuidoFm1Status g_ruido_fm1_status;

/* Placement-new construct shared buses, Core 1 DSP state, Core 0 controller/HUD, and USB drift
 * resampler in `.pool` SRAM. MUST be called strictly AFTER Hold OCT- USB rescue clears and AFTER
 * `fm1_cpu1_load_sram()` populates `.c1_text`. Leaves `engineReady == 0` during boot benchmarks. */
void ruido_fm1_construct(uint32_t initial_master_q12);

/* Configure a 4-voice nonlinear MS-20 self-oscillation + overdrive + 8x LFO stress patch, or
 * restore clean boot defaults (`stress_mode == 0`). */
void ruido_fm1_set_bench_stress(uint32_t stress_mode);

/* Enable or disable the 2+2 `VoiceFilterCoProcessor` attachment prior to launching Core 1. */
void ruido_fm1_set_coproc_enabled(uint32_t enabled);

/* Measure one 64-frame `.c1_text` SRAM DSP block in TIMER4 ticks (runs dual-core 2+2 parallel
 * timing when Core 1 is active, or falls back automatically to solo timing if Core 1 faulted). */
uint32_t ruido_fm1_measure_bench64_ticks(void);

/* Reset controller, DSP core, and HUD cleanly after boot benchmarks complete. */
void ruido_fm1_restore_default_patch(void);

/* Pre-prime `Fm1AudioBlockRing` with 4 blocks (256 frames) and enable audio + Core 1 worker. */
void ruido_fm1_arm_audio(uint32_t enable_core1);

/* Invoked from `fm1_dc_fatal_stubs` if Core 1 takes a silicon exception: detaches the co-processor
 * and switches `ruido_fm1_audio` seamlessly to Core 0 fallback rendering. */
void ruido_fm1_on_c1_fault(void);

/* Opportunistically assist Core 1 by claiming a pending `Voices 2-3` 4x ZDF filter span in
 * `.c1_text` SRAM (`0` branch cost when no span is waiting). Safe from `app_idle` & main loop. */
void ruido_fm1_assist_dsp(void);

/* Update pre-dither float32 MASTER pot gain target from dinghy's `[0..4088]` quadratic pot curve.
 */
void ruido_fm1_set_master_q12(uint32_t master_q12);

/* Core 0 ALNK0 24 MHz crystal DMA callback: drain 64-frame TPDF blocks from `Fm1AudioBlockRing`. */
void ruido_fm1_audio(int32_t* out_lr_q15, uint32_t num_frames);

/* Fractional Hermite + 1-LSB TPDF USB UAC1 clock-drift bridge (`44.1 kHz` / `48 kHz` +-50 ppm). */
uint32_t ruido_fm1_usb_feed(const int32_t* in_lr_q15, uint32_t num_in_frames, uint32_t rate_48k,
                            int32_t fill_frames, int32_t target_fill, uint32_t* ua_ring_buf,
                            uint32_t ua_ring_mask, uint32_t write_pos, uint32_t free_slots,
                            uint32_t stream_seq);

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
