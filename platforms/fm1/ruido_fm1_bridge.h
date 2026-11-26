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
    volatile uint32_t panel_state;       /* [12] [1:0]=model, [3:2]=slot, b4=sync, b5=run, b6=dirty,
                                            [13:8]=hud, [31:16]=events */
    volatile uint32_t midi_clock_state;  /* [13] [15:0]=bpm_x10, [31:16]=midi_rt_events */
    volatile uint32_t led_state_summary; /* [14] [7:0]=lit, [15:8]=dim, [23:16]=bg, b24=play,
                                            b25=stop */
    volatile uint32_t nvs_and_osc1_state;     /* [15] [7:0]=boot_mask, [15:8]=saves_ok,
                                                 [31:16]=osc1_hz_x10 */
    volatile uint32_t nvs_flash_readback[68]; /* [16..83] 128B S2 + 16B Cfg + 128B Live SPI read */
} RuidoFm1Status;

extern RuidoFm1Status g_ruido_fm1_status;

/* Reset-surviving (`.noinit`) boot breadcrumb. Core 0 writes `stage`; Core 1's fault stubs in
 * `fm1_cpu1.S` write `c1_code` / `c1_aux` directly so the record survives a Core 0 hang + WDT. */
#define RUIDO_BOOT_DIAG_MAGIC 0x52424432u /* "RBD2" */
typedef struct {
    volatile uint32_t magic;    /* [0]  */
    volatile uint32_t stage;    /* [4]  last boot stage reached by Core 0 */
    volatile uint32_t c1_code;  /* [8]  DBG_MSG (PC-limit trap) | 0xC1xxxxxx (C1 vector) | 0xCA.. */
    volatile uint32_t c1_aux;   /* [12] interrupted Core 0 PC (trap) or Core 1 RETI (C1 vector) */
    volatile uint32_t c1_state; /* [16] [7:0]=solo reason, b8=c1SelfFault seen, b9=c1Booted */
    volatile uint32_t uptime_ms;
} RuidoBootDiag;

extern RuidoBootDiag g_ruido_boot_diag;

/* Snapshot live Core 1 fault words into `g_ruido_boot_diag` (cheap; call per frame / stage). */
void ruido_fm1_diag_sync_c1(void);
uint32_t ruido_fm1_diag_max_c1_ticks(void);
void ruido_fm1_diag_clear_counters(void);

typedef int (*RuidoFm1BlobLoadFn)(uint32_t idx, void* dst, uint32_t len);
typedef int (*RuidoFm1BlobSaveFn)(uint32_t idx, const void* src, uint32_t len);
typedef int (*RuidoFm1TailReadFn)(uint32_t sector_id, uint32_t off_in_sector, void* dst,
                                  uint32_t len);
typedef int (*RuidoFm1TailWriteFn)(uint32_t sector_id, uint32_t off_in_sector, const void* src,
                                   uint32_t len);
typedef void (*RuidoFm1PreEraseFn)(void);

/* Placement-new construct shared buses, Core 1 DSP state, Core 0 controller/HUD, panel/LED I/O,
 * and USB drift resampler in `.pool` SRAM. MUST be called strictly AFTER Hold OCT- USB rescue
 * clears and AFTER `fm1_cpu1_load_sram()` populates `.c1_text`. Leaves `engineReady == 0` during
 * boot benchmarks. */
void ruido_fm1_construct(uint32_t initial_master_q12);

/* Configure a 4-voice nonlinear MS-20 self-oscillation + overdrive + 8x LFO stress patch, or
 * restore clean boot defaults (`stress_mode == 0`). */
void ruido_fm1_set_bench_stress(uint32_t stress_mode);

/* Enable or disable the 2+2 `VoiceFilterCoProcessor` attachment prior to launching Core 1. */
void ruido_fm1_set_coproc_enabled(uint32_t enabled);

/* Measure one 64-frame `.c1_text` SRAM DSP block in TIMER4 ticks (runs dual-core 2+2 parallel
 * timing when Core 1 is active, or falls back automatically to solo timing if Core 1 faulted). */
uint32_t ruido_fm1_measure_bench64_ticks(void);

/* Reset controller, DSP core, panel I/O, and HUD cleanly after boot benchmarks complete. */
void ruido_fm1_restore_default_patch(void);

/* Load CRC-verified `PatchSnapshot`s (`sector 0..3` @ `0xF4000..0xF7FFF`), `DeviceSettingsBlob`
 * (`sector 4` @ `0xF8000`), and `LIVE` scratchpad (`blob 5` @ `0xF9000`), scanning all
 * append-only tail journals and compacting silently at cold boot while Core 1 & audio are off. */
uint32_t ruido_fm1_nvs_load_boot(RuidoFm1BlobLoadFn load_fn, RuidoFm1TailReadFn tail_read_fn,
                                 RuidoFm1BlobSaveFn boot_compact_save_fn);

/* Persist newly saved/deleted Scene slots immediately (`SAVE` + White Key 9-12) and debounce
 * Global Config + `LIVE` drone scratchpad edits (`1,500 ms` idle with no keys held) via erase-free
 * 128 B / 16 B journal appends (~150-500 us page program, zero sector erase, zero audio dropout).
 */
uint32_t ruido_fm1_nvs_poll_save(uint32_t now_ms, uint32_t force_flush, RuidoFm1BlobSaveFn save_fn,
                                 RuidoFm1TailWriteFn tail_write_fn,
                                 RuidoFm1PreEraseFn pre_erase_fn);

/* Pre-prime `Fm1AudioBlockRing` with 8 blocks (512 frames) and enable audio + Core 1 worker. */
void ruido_fm1_arm_audio(uint32_t enable_core1);

/* Launch Core 1 safely across AC791N Mask ROM + `uboot.boot` (`0x01E00000`) -> `fm1_cpu1_boot_xip`
 * -> `fm1_cpu1_entry`, returning 1 if Core 1 reached `.c1_text` SRAM or 0 if solo fallback engaged.
 */
uint32_t ruido_fm1_start_core1(void);

/* Explicitly switch `DroneEngineCore` into 22.05 kHz half-rate solo fallback when `allow_core1 ==
 * 0`. */
void ruido_fm1_enter_solo_mode(void);

/* Cleanly pause Core 1 and mute DSP block synthesis ahead of a live USB-MIDI OTA session or
 * UBOOT soft-key jump; `ruido_fm1_resume_after_ota()` re-arms audio if an OTA dry-run cancels. */
void ruido_fm1_quiesce_for_ota(void);
void ruido_fm1_resume_after_ota(void);

/* Park/unpark Core 1 cleanly inside a tight `.c1_text` internal SRAM spinloop around runtime SPI0
 * NOR flash saves/reads without touching the one-way `0x01EEE004` (`C1_CON`) hardware latch. */
void ruido_fm1_flash_hold_begin(void);
void ruido_fm1_flash_hold_end(void);

/* Opportunistically assist Core 1 by claiming a pending `Voices 2-3` 4x ZDF filter span in
 * `.c1_text` SRAM (`0` branch cost when no span is waiting). Safe from `app_idle` & main loop. */
void ruido_fm1_assist_dsp(void);

/* Query ready 64-frame blocks in `Fm1AudioBlockRing` (`0..16`, or `16` when muted / solo / pre-arm)
 * so Core 0 soft-real-time LCD strip pacing yields cleanly whenever the 16-block audio cushion dips
 * below `min_blocks`. */
uint32_t ruido_fm1_audio_cushion_blocks(void);

/* Route dinghy's `[0..4088]` quadratic `MASTER` pot reading on Core 0 into `RuidoController`. */
void ruido_fm1_set_master_q12(uint32_t master_q12);

/* Dispatch 14 tactile buttons, 7 quadrature encoders, and 27 silicone keys + PA7/PA8 pressure. */
void ruido_fm1_poll_panel(uint32_t now_ms, uint32_t pressed_btns, uint32_t released_btns,
                          uint32_t cur_btns, const int32_t enc_steps[7], uint32_t cur_notes,
                          uint32_t note_press_edges);

/* Query active round-robin PA7/PA8 carbon-pill SARADC column sample request from `TIMER5` ISR. */
uint32_t ruido_fm1_adc_get_req(uint8_t* out_note, uint8_t* out_col, uint8_t* out_ch,
                               uint8_t* out_pin_bit);

/* Record settled 10-bit SARADC reading for `note_bit` from `TIMER5` ISR. */
void ruido_fm1_adc_put_sample(uint8_t note_bit, int32_t adc_val);

/* Step 42-channel staggered-phase 909.1 Hz Delta-Sigma LED modulator inside `TIMER5` ISR. */
void ruido_fm1_led_step_909hz(uint8_t out_lit[11], uint8_t out_dim[11], uint8_t out_bg[11]);

/* Core 0 ALNK0 24 MHz crystal DMA callback: drain 64-frame TPDF blocks from `Fm1AudioBlockRing`. */
void ruido_fm1_audio(int32_t* out_lr_q15, uint32_t num_frames);

/* Fractional Hermite + 1-LSB TPDF USB UAC1 clock-drift bridge (`44.1 kHz` / `48 kHz` +-50 ppm). */
uint32_t ruido_fm1_usb_feed(const int32_t* in_lr_q15, uint32_t num_in_frames, uint32_t rate_48k,
                            int32_t fill_frames, int32_t target_fill, uint32_t* ua_ring_buf,
                            uint32_t ua_ring_mask, uint32_t write_pos, uint32_t free_slots,
                            uint32_t stream_seq);

/* Route 1-byte MIDI System Real-Time clock/transport (0xF8/0xFA/0xFB/0xFC) to MidiClockTracker
 * stamped with sub-microsecond 24 MHz `TIMER4` hardware time. */
void ruido_fm1_on_midi_rt(uint8_t status_byte);

/* Advance controller state, ingest scope ring, update LED targets, and prepare 240x240 curves. */
void ruido_fm1_tick_and_prepare_ms(uint32_t elapsed_ms);

/* Render a 240x16 scanline strip at `y_start` into `.pool` ping-pong slot `slot & 1` in
 * big-endian RGB565 for ST7789 SPI1 DMA (`0 B` stack copy). */
const uint16_t* ruido_fm1_render_strip_be(uint16_t y_start, uint8_t slot);

/* Return `.pool` strip buffer `g_strip_ping_pong[0]` (`240 x 16` uint16_t) for lifecycle
 * operational screens (`OTA`, `USB RESCUE`, `SYSTEM FAULT`) when normal rendering is quiesced. */
uint16_t* ruido_fm1_acquire_ops_strip(void);

#ifdef __cplusplus
}
#endif
