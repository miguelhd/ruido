#include "ruido_fm1_bridge.h"

#include "core/RuidoController.h"
#include "fm1_cpu1.h"
#include "fm1_panel_io.h"
#include "ruido_fm1_state.h"
#include "ui/ScopeHudRenderer.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

inline void* operator new(size_t, void* p) noexcept {
    return p;
}

extern "C" RuidoFm1Status g_ruido_fm1_status{};

/* Cache-line-isolated `.pool` arenas (shared IPC vs Core 1 DSP vs Core 0 UI/Panel/USB) */
alignas(64) uint8_t g_ruido_shared_buses_arena[sizeof(ruido::EngineSharedBuses)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_ruido_audio_ring_arena[sizeof(ruido::Fm1AudioBlockRing)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_ruido_coproc_arena[sizeof(ruido::VoiceFilterCoProcessor)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_ruido_dsp_core_arena[sizeof(ruido::DroneEngineCore)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_ruido_c1_runtime_arena[sizeof(ruido::fm1::Fm1C1DspRuntime)]
    __attribute__((section(".pool")));
ruido::fm1::Fm1InterCoreControl g_ruido_ic_ctrl{};

namespace {

constexpr size_t kAudioChunkFrames = ruido::kBlockFrames;
constexpr size_t kStripPixels = ruido::kDisplayWidth * ruido::kScanlineStripHeight;
constexpr float kMaxPlatformMasterQ12 = 4088.0f; // (1023 * 1023) >> 8 in platform.c
constexpr uint32_t kC1StallFallbackCallbacks = 3u;
constexpr uint32_t kC1BenchSpinLimit = 40000u;
constexpr uint32_t kTicksPer64Sec = 1536000000u; // 64 s * 24,000,000 Hz TIMER4 crystal
constexpr float kInvTimer4Hz = 1.0f / 24000000.0f;
constexpr uint32_t kNvsSettingsDebounceMs = 1500u;
constexpr uint32_t kNvsSettingsBlobIdx = 4u; // 0xF8000 (blobs 0..3 = 0xF4000..0xF7FFF scenes)

volatile uint32_t* const kTimer4CntReg = reinterpret_cast<volatile uint32_t*>(0x00010804u);

alignas(64) uint8_t g_ctrl_arena[sizeof(ruido::RuidoController)] __attribute__((section(".pool")));
alignas(64) uint8_t g_renderer_arena[sizeof(ruido::ScopeHudRenderer)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_usb_resampler_arena[sizeof(ruido::Fm1UsbDriftResampler)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_panel_arena[sizeof(ruido::fm1::Fm1PanelState)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_led_mod_arena[sizeof(ruido::fm1::Fm1LedModulator)]
    __attribute__((section(".pool")));
alignas(64) uint16_t g_strip_ping_pong[2][kStripPixels] __attribute__((section(".pool")));

uint32_t g_last_seen_c1_blocks = 0u;
uint32_t g_c1_stall_callbacks = 0u;
float g_latest_master_gain = 0.85f;
uint8_t g_panel_constructed = 0u;

uint8_t g_midi_t4_anchored = 0u;
uint32_t g_midi_t4_prev = 0u;
uint32_t g_midi_t4_mod64 = 0u;
uint32_t g_midi_rt_events = 0u;

uint32_t g_last_panel_touch_ms = 0u;
uint8_t g_pending_slot_mask = 0u;
uint8_t g_pending_settings_save = 0u;
uint8_t g_nvs_boot_loaded_mask = 0u;
uint8_t g_nvs_saves_ok = 0u;

inline ruido::RuidoController* controller() noexcept {
    return reinterpret_cast<ruido::RuidoController*>(g_ctrl_arena);
}

inline ruido::ScopeHudRenderer* renderer() noexcept {
    return reinterpret_cast<ruido::ScopeHudRenderer*>(g_renderer_arena);
}

inline ruido::Fm1UsbDriftResampler* usbResampler() noexcept {
    return reinterpret_cast<ruido::Fm1UsbDriftResampler*>(g_usb_resampler_arena);
}

inline ruido::fm1::Fm1PanelState* panel() noexcept {
    return reinterpret_cast<ruido::fm1::Fm1PanelState*>(g_panel_arena);
}

inline ruido::fm1::Fm1LedModulator* ledMod() noexcept {
    return reinterpret_cast<ruido::fm1::Fm1LedModulator*>(g_led_mod_arena);
}

inline uint32_t floatToBits(float f) noexcept {
    union {
        float f;
        uint32_t u;
    } conv{f};
    return conv.u;
}

inline float masterQ12ToGain(uint32_t master_q12) noexcept {
    return ruido::clampf(static_cast<float>(master_q12) * (1.0f / kMaxPlatformMasterQ12), 0.0f,
                         1.0f);
}

inline void refreshGate43StatusWords() noexcept {
    const auto* ctrl = controller();
    const auto& hud = ctrl->hud();
    const auto& st = ctrl->state();

    const uint32_t modelBits = static_cast<uint32_t>(hud.keyboardModel) & 3u;
    const uint32_t slotBits = static_cast<uint32_t>(ctrl->snapshots().activeSlot()) & 3u;
    const uint32_t syncBit = hud.midiClockSynced ? 1u : 0u;
    const uint32_t runBit = st.global.masterRun ? 1u : 0u;
    const uint32_t hudBits = static_cast<uint32_t>(hud.mode) & 0x3Fu;
    const uint32_t evBits = static_cast<uint32_t>(panel()->inputEventCount());

    g_ruido_fm1_status.panel_state = modelBits | (slotBits << 2u) | (syncBit << 4u) |
                                     (runBit << 5u) | (hudBits << 8u) | (evBits << 16u);

    const float clampedBpm = ruido::clampf(hud.effectiveClockBpm, 0.0f, 6500.0f);
    const auto bpmX10 = static_cast<uint32_t>(clampedBpm * 10.0f + 0.5f) & 0xFFFFu;
    g_ruido_fm1_status.midi_clock_state = bpmX10 | ((g_midi_rt_events & 0xFFFFu) << 16u);

    g_ruido_fm1_status.led_state_summary = ledMod()->lastSummary();

    const float clampedHz = ruido::clampf(st.voices[0].frequencyHz, 0.0f, 6550.0f);
    const auto osc1HzX10 = static_cast<uint32_t>(clampedHz * 10.0f + 0.5f) & 0xFFFFu;
    g_ruido_fm1_status.nvs_and_osc1_state = static_cast<uint32_t>(g_nvs_boot_loaded_mask) |
                                            (static_cast<uint32_t>(g_nvs_saves_ok) << 8u) |
                                            (osc1HzX10 << 16u);
}

inline void copyStereoBlockToDma(const ruido::Fm1AudioBlock& src, int32_t* dst,
                                 uint32_t frames) noexcept {
    for (uint32_t i = 0u; i < frames; ++i) {
        dst[2u * i] = static_cast<int32_t>(src.lr[2u * i]);
        dst[2u * i + 1u] = static_cast<int32_t>(src.lr[2u * i + 1u]);
    }
}

inline void engageCore1Fallback() noexcept {
    g_ruido_ic_ctrl.c1RunEnable.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_relaxed);
    fm1_cpu1_pause();
    ruidoSharedBuses()->controlBus.abortFromFault();
    ruidoCoProc()->abortFromFault();
    ruidoDspCore()->attachFilterCoProcessor(nullptr);
    g_ruido_ic_ctrl.c1FallbackActive.store(1u, std::memory_order_release);
    g_ruido_fm1_status.c1_flags = g_ruido_fm1_status.c1_flags | 2u;
}

inline bool checkCore1SelfFault() noexcept {
    if (g_ruido_ic_ctrl.c1SelfFault.load(std::memory_order_acquire) != 0u) {
        if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_relaxed) == 0u) {
            engageCore1Fallback();
        }
        return true;
    }
    return g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) != 0u;
}

uint32_t measureSoloBench64Ticks() noexcept {
    const uint32_t passes = checkCore1SelfFault() ? 1u : 3u;
    uint32_t best = 0xFFFFFFFFu;
    for (uint32_t pass = 0u; pass < passes; ++pass) {
        uint32_t saved_icfg = 0u;
        __asm__ volatile("%0 = icfg\n\tcli" : "=r"(saved_icfg)::"memory");
        const uint32_t t0 = *kTimer4CntReg;
        ruido::fm1::callC1Far(ruido_fm1_c1_step_solo_bench64);
        const uint32_t dt = *kTimer4CntReg - t0;
        __asm__ volatile("csync\n\ticfg = %0" ::"r"(saved_icfg) : "memory");
        if (dt > 0u && dt < best) {
            best = dt;
        }
    }
    return best;
}

} // namespace

extern "C" {

void ruido_fm1_construct(uint32_t initial_master_q12) {
    g_ruido_ic_ctrl.sramCanaries[0] = 0u;
    g_ruido_ic_ctrl.sramCanaries[1] = 0xFFFFFFFFu;
    g_ruido_ic_ctrl.c1SelfFault.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.engineReady.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1RunEnable.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FallbackActive.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchTicks.store(0u, std::memory_order_relaxed);

    g_latest_master_gain = masterQ12ToGain(initial_master_q12);
    /* Core 1 quantizer runs at 1.0f unity; MASTER pot scales inside DroneEngineCore::masterTrim */
    g_ruido_ic_ctrl.masterGainBits.store(floatToBits(1.0f), std::memory_order_relaxed);

    ruido::fm1::callC1Far(ruido_fm1_c1_construct, 1.0f);

    auto* ctrl = new (g_ctrl_arena) ruido::RuidoController();
    auto* ui = new (g_renderer_arena) ruido::ScopeHudRenderer();
    auto* usbRs = new (g_usb_resampler_arena) ruido::Fm1UsbDriftResampler();
    auto* pnl = new (g_panel_arena) ruido::fm1::Fm1PanelState();
    auto* leds = new (g_led_mod_arena) ruido::fm1::Fm1LedModulator();

    ctrl->attachControlBus(&ruidoSharedBuses()->controlBus);
    pnl->reset(g_latest_master_gain);
    pnl->syncMasterPot(*ctrl, g_latest_master_gain);
    leds->reset();
    leds->updateTargets(ctrl->leds());
    g_panel_constructed = 1u;

    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    ui->reset();
    usbRs->reset();

    g_last_seen_c1_blocks = 0u;
    g_c1_stall_callbacks = 0u;
    g_midi_t4_anchored = 0u;
    g_midi_t4_prev = 0u;
    g_midi_t4_mod64 = 0u;
    g_midi_rt_events = 0u;
    g_last_panel_touch_ms = 0u;
    g_pending_slot_mask = 0u;
    g_pending_settings_save = 0u;
    g_nvs_boot_loaded_mask = 0u;
    g_nvs_saves_ok = 0u;

    refreshGate43StatusWords();
}

void ruido_fm1_set_coproc_enabled(uint32_t enabled) {
    auto* coproc = ruidoCoProc();
    coproc->reset();
    if (enabled != 0u) {
        ruido::fm1::callC1Far(ruido_fm1_c1_set_helper_armed, 1u);
        ruidoDspCore()->attachFilterCoProcessor(coproc);
    } else {
        ruidoDspCore()->attachFilterCoProcessor(nullptr);
    }
    ruido::fm1::hwMemoryBarrier();
}

uint32_t ruido_fm1_measure_bench64_ticks(void) {
    if ((g_ruido_fm1_status.c1_flags & 1u) == 0u || checkCore1SelfFault()) {
        return measureSoloBench64Ticks();
    }

    uint32_t best = 0xFFFFFFFFu;
    for (uint32_t pass = 0u; pass < 3u; ++pass) {
        g_ruido_ic_ctrl.c1BenchTicks.store(0u, std::memory_order_relaxed);
        g_ruido_ic_ctrl.c1BenchReq.store(1u, std::memory_order_release);

        uint32_t spins = 0u;
        while (g_ruido_ic_ctrl.c1BenchReq.load(std::memory_order_acquire) != 0u) {
            if (checkCore1SelfFault() || ++spins >= kC1BenchSpinLimit) {
                engageCore1Fallback();
                return measureSoloBench64Ticks();
            }
            if (ruidoCoProc()->hasPendingWork()) {
                ruido::fm1::callC1Far(ruido_fm1_c1_assist_dsp);
            }
        }
        const uint32_t dt = g_ruido_ic_ctrl.c1BenchTicks.load(std::memory_order_relaxed);
        if (dt > 0u && dt < best) {
            best = dt;
        }
    }
    return best;
}

void ruido_fm1_restore_default_patch(void) {
    auto* ctrl = controller();
    ruido::fm1::callC1Far(ruido_fm1_set_bench_stress, 0u);
    ctrl->reset();
    panel()->reset(g_latest_master_gain);
    panel()->syncMasterPot(*ctrl, g_latest_master_gain);
    (void)ctrl->consumeSavedSlotMask();
    (void)ctrl->consumeSettingsDirty();
    ledMod()->reset();
    ledMod()->updateTargets(ctrl->leds());
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    renderer()->reset();
    g_midi_t4_anchored = 0u;
    g_midi_rt_events = 0u;
    g_pending_slot_mask = 0u;
    g_pending_settings_save = 0u;
    refreshGate43StatusWords();
}

uint32_t ruido_fm1_nvs_load_boot(RuidoFm1BlobLoadFn load_fn) {
    if (load_fn == nullptr || g_panel_constructed == 0u) {
        return 0u;
    }
    auto* ctrl = controller();
    uint32_t loadedMask = 0u;

    /* 1. Load up to 4 CRC-verified 128-byte Scene slots (`0xF4000..0xF7FFF`) */
    for (uint8_t s = 0u; s < ruido::kNumSnapshots; ++s) {
        ruido::PatchSnapshot snap{};
        if (load_fn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0 &&
            ctrl->loadSnapshotSlot(s, snap)) {
            loadedMask |= (1u << s);
        }
    }

    /* 2. Load CRC-verified 16-byte DeviceSettingsBlob (`0xF8000`) and activate saved Scene */
    ruido::DeviceSettingsBlob settings{};
    if (load_fn(kNvsSettingsBlobIdx, &settings, static_cast<uint32_t>(sizeof(settings))) == 0 &&
        ctrl->loadSettings(settings)) {
        loadedMask |= (1u << kNvsSettingsBlobIdx);
    }

    /* 3. Re-apply live physical MASTER pot position & clear boot dirty flags */
    panel()->reset(g_latest_master_gain);
    panel()->syncMasterPot(*ctrl, g_latest_master_gain);
    (void)ctrl->consumeSavedSlotMask();
    (void)ctrl->consumeSettingsDirty();

    ledMod()->updateTargets(ctrl->leds());
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);

    g_nvs_boot_loaded_mask = static_cast<uint8_t>(loadedMask & 0xFFu);
    refreshGate43StatusWords();
    return loadedMask;
}

uint32_t ruido_fm1_nvs_poll_save(uint32_t now_ms, uint32_t force_flush, RuidoFm1BlobSaveFn save_fn,
                                 RuidoFm1PreEraseFn pre_erase_fn) {
    if (save_fn == nullptr || g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return 0u;
    }

    auto* ctrl = controller();
    g_pending_slot_mask = static_cast<uint8_t>(g_pending_slot_mask | ctrl->consumeSavedSlotMask());
    if (ctrl->consumeSettingsDirty()) {
        g_pending_settings_save = 1u;
    }

    const bool saveSlotsNow = (g_pending_slot_mask != 0u);
    const auto& hud = ctrl->hud();
    const bool keysIdle = (hud.heldBlackMask == 0u) && (hud.heldWhiteMask == 0u);
    const bool saveSettingsNow =
        (g_pending_settings_save != 0u) &&
        (saveSlotsNow || force_flush != 0u ||
         (keysIdle && (now_ms - g_last_panel_touch_ms) >= kNvsSettingsDebounceMs));

    if (!saveSlotsNow && !saveSettingsNow) {
        return 0u;
    }

    if (pre_erase_fn != nullptr) {
        pre_erase_fn();
    }

    uint32_t writes = 0u;
    if (saveSlotsNow) {
        for (uint8_t s = 0u; s < ruido::kNumSnapshots; ++s) {
            const uint8_t sBit = static_cast<uint8_t>(1u << s);
            if ((g_pending_slot_mask & sBit) == 0u) {
                continue;
            }
            const ruido::PatchSnapshot snap = ctrl->snapshots().slotBinary(s);
            if (save_fn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0) {
                g_pending_slot_mask = static_cast<uint8_t>(g_pending_slot_mask & ~sBit);
                if (g_nvs_saves_ok < 255u) {
                    ++g_nvs_saves_ok;
                }
                ++writes;
            }
        }
    }

    if (saveSettingsNow) {
        const ruido::DeviceSettingsBlob blob = ctrl->exportSettings();
        if (save_fn(kNvsSettingsBlobIdx, &blob, static_cast<uint32_t>(sizeof(blob))) == 0) {
            g_pending_settings_save = 0u;
            if (g_nvs_saves_ok < 255u) {
                ++g_nvs_saves_ok;
            }
            ++writes;
        }
    }

    refreshGate43StatusWords();
    return writes;
}

void ruido_fm1_arm_audio(uint32_t enable_core1) {
    const bool useCore1 = (enable_core1 != 0u) && !checkCore1SelfFault();

    /* Pre-fill all 8 x 64-frame blocks (512 frames = two full 256-frame ALNK0 DMA halves) */
    ruido::fm1::callC1Far(ruido_fm1_c1_prime_blocks, ruido::Fm1AudioBlockRing::kNumBlocks,
                          useCore1 ? 1u : 0u);

    if (useCore1) {
        g_ruido_ic_ctrl.c1RunEnable.store(1u, std::memory_order_release);
    } else {
        engageCore1Fallback();
    }

    ruido::fm1::hwMemoryBarrier();
    g_ruido_ic_ctrl.engineReady.store(1u, std::memory_order_release);
    g_ruido_fm1_status.magic = RUIDO_FM1_STATUS_MAGIC;
    refreshGate43StatusWords();
}

void ruido_fm1_on_c1_fault(void) {
    engageCore1Fallback();
    ruido::fm1::hwMemoryBarrier();
    for (;;) {
        __asm__ volatile("nop" ::: "memory");
    }
}

void ruido_fm1_assist_dsp(void) {
    if (checkCore1SelfFault()) {
        return;
    }
    if (ruidoCoProc()->hasPendingWork()) {
        ruido::fm1::callC1Far(ruido_fm1_c1_assist_dsp);
    }
}

void ruido_fm1_set_master_q12(uint32_t master_q12) {
    g_latest_master_gain = masterQ12ToGain(master_q12);
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return;
    }
    panel()->syncMasterPot(*controller(), g_latest_master_gain);
}

void ruido_fm1_poll_panel(uint32_t now_ms, uint32_t pressed_btns, uint32_t released_btns,
                          uint32_t cur_btns, const int32_t enc_steps[7], uint32_t cur_notes,
                          uint32_t note_press_edges) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u || enc_steps == nullptr) {
        return;
    }
    auto* ctrl = controller();
    auto* pnl = panel();
    const bool btnOrEnc =
        pnl->pollButtonsAndEncoders(*ctrl, pressed_btns, released_btns, cur_btns, enc_steps);
    const bool keyTouched = pnl->pollKeysAndPressure(*ctrl, cur_notes, note_press_edges);
    if (btnOrEnc || keyTouched) {
        g_last_panel_touch_ms = now_ms;
    }
}

uint32_t ruido_fm1_adc_get_req(uint8_t* out_note, uint8_t* out_col, uint8_t* out_ch,
                               uint8_t* out_pin_bit) {
    if (g_panel_constructed == 0u) {
        return 0u;
    }
    const auto req = panel()->currentAdcRequest();
    if (req.active == 0u) {
        return 0u;
    }
    *out_note = req.noteBit;
    *out_col = req.col;
    *out_ch = req.saradcCh;
    *out_pin_bit = req.paPinBit;
    return 1u;
}

void ruido_fm1_adc_put_sample(uint8_t note_bit, int32_t adc_val) {
    if (g_panel_constructed == 0u) {
        return;
    }
    panel()->recordAdcSample(note_bit, adc_val);
}

void ruido_fm1_led_step_909hz(uint8_t out_lit[11], uint8_t out_dim[11], uint8_t out_bg[11]) {
    if (g_panel_constructed == 0u) {
        return;
    }
    ledMod()->stepFrame909Hz(out_lit, out_dim, out_bg);
    g_ruido_fm1_status.led_state_summary = ledMod()->lastSummary();
}

void ruido_fm1_audio(int32_t* out_lr_q15, uint32_t num_frames) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u ||
        out_lr_q15 == nullptr) {
        return;
    }

    (void)checkCore1SelfFault();
    auto* ring = ruidoAudioRing();
    int32_t peak = g_ruido_fm1_status.peak_abs_q15;
    bool hadMiss = false;
    uint32_t offset = 0u;

    while (offset < num_frames) {
        const uint32_t chunk =
            ((num_frames - offset) > kAudioChunkFrames) ? kAudioChunkFrames : (num_frames - offset);
        int32_t* dst = out_lr_q15 + 2u * offset;

        if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) != 0u) {
            ruido::Fm1AudioBlock fallbackBlk{};
            ruido::fm1::callC1Far(ruido_fm1_c1_render_one_block, &fallbackBlk);
            copyStereoBlockToDma(fallbackBlk, dst, chunk);
            if (fallbackBlk.peakAbsQ15 > peak) {
                peak = fallbackBlk.peakAbsQ15;
            }
        } else {
            const ruido::Fm1AudioBlock* blk = ring->beginPop();
            if (blk != nullptr) {
                ruido::fm1::hwMemoryBarrier();
                copyStereoBlockToDma(*blk, dst, chunk);
                if (blk->peakAbsQ15 > peak) {
                    peak = blk->peakAbsQ15;
                }
                ruido::fm1::hwMemoryBarrier();
                ring->commitPop();
            } else if (checkCore1SelfFault()) {
                ruido::Fm1AudioBlock fallbackBlk{};
                ruido::fm1::callC1Far(ruido_fm1_c1_render_one_block, &fallbackBlk);
                copyStereoBlockToDma(fallbackBlk, dst, chunk);
                if (fallbackBlk.peakAbsQ15 > peak) {
                    peak = fallbackBlk.peakAbsQ15;
                }
            } else {
                hadMiss = true;
            }
        }
        offset += chunk;
    }

    if (hadMiss && !checkCore1SelfFault()) {
        const uint32_t curFlags = g_ruido_fm1_status.c1_flags;
        const uint32_t und = (curFlags >> 8u) & 0xFFu;
        if (und < 255u) {
            g_ruido_fm1_status.c1_flags = (curFlags & ~0xFF00u) | ((und + 1u) << 8u);
        }

        const uint32_t curC1Blocks = g_ruido_fm1_status.c1_blocks;
        if (curC1Blocks == g_last_seen_c1_blocks) {
            ++g_c1_stall_callbacks;
            if (g_c1_stall_callbacks >= kC1StallFallbackCallbacks) {
                /* Automatic silicon-lottery survival: Core 1 halted -> switch to Core 0 fallback */
                engageCore1Fallback();
            }
        } else {
            g_last_seen_c1_blocks = curC1Blocks;
            g_c1_stall_callbacks = 0u;
        }
    } else {
        g_last_seen_c1_blocks = g_ruido_fm1_status.c1_blocks;
        g_c1_stall_callbacks = 0u;
    }

    g_ruido_fm1_status.peak_abs_q15 = peak;
    g_ruido_fm1_status.audio_blocks = g_ruido_fm1_status.audio_blocks + 1u;
}

uint32_t ruido_fm1_usb_feed(const int32_t* in_lr_q15, uint32_t num_in_frames, uint32_t rate_48k,
                            int32_t fill_frames, int32_t target_fill, uint32_t* ua_ring_buf,
                            uint32_t ua_ring_mask, uint32_t write_pos, uint32_t free_slots,
                            uint32_t stream_seq) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return 0u;
    }
    return usbResampler()->processBlockToRing(in_lr_q15, num_in_frames, rate_48k != 0u, fill_frames,
                                              target_fill, ua_ring_buf, ua_ring_mask, write_pos,
                                              free_slots, stream_seq);
}

void ruido_fm1_on_midi_rt(uint8_t status_byte) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return;
    }
    const uint32_t nowT4 = *kTimer4CntReg;
    if (g_midi_t4_anchored == 0u) {
        g_midi_t4_anchored = 1u;
        g_midi_t4_prev = nowT4;
        g_midi_t4_mod64 = 0u;
    } else {
        uint32_t delta = nowT4 - g_midi_t4_prev;
        g_midi_t4_prev = nowT4;
        while (delta >= kTicksPer64Sec) {
            delta -= kTicksPer64Sec;
        }
        uint32_t nextMod = g_midi_t4_mod64 + delta;
        if (nextMod >= kTicksPer64Sec) {
            nextMod -= kTicksPer64Sec;
        }
        g_midi_t4_mod64 = nextMod;
    }
    const float tsSec = static_cast<float>(g_midi_t4_mod64) * kInvTimer4Hz;
    controller()->onMidiRealtime(status_byte, tsSec);
    ++g_midi_rt_events;
    refreshGate43StatusWords();
}

void ruido_fm1_tick_and_prepare_ms(uint32_t elapsed_ms) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return;
    }
    const uint32_t clamped_ms =
        (elapsed_ms == 0u) ? 16u : ((elapsed_ms > 100u) ? 100u : elapsed_ms);
    const float dt_sec = static_cast<float>(clamped_ms) * 0.001f;
    auto* buses = ruidoSharedBuses();
    auto* ctrl = controller();
    auto* ui = renderer();

    ruido_fm1_assist_dsp();
    ctrl->tick(dt_sec, buses->telemetry);
    ledMod()->updateTargets(ctrl->leds());
    ruido_fm1_assist_dsp();
    ui->ingestScopeRing(buses->scopeBuffer);
    ruido_fm1_assist_dsp();
    ui->prepareFrame(dt_sec, ctrl->state(), ctrl->hud(), buses->telemetry);
    ruido_fm1_assist_dsp();

    const uint32_t assisted = ruidoCoProc()->assistedSpans();
    const uint32_t clampedAssist = (assisted > 0xFFFFu) ? 0xFFFFu : assisted;
    g_ruido_fm1_status.c1_flags =
        (g_ruido_fm1_status.c1_flags & 0x0000FFFFu) | (clampedAssist << 16u);
    refreshGate43StatusWords();
}

const uint16_t* ruido_fm1_render_strip_be(uint16_t y_start, uint8_t slot) {
    uint16_t* strip = g_strip_ping_pong[slot & 1u];
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return strip;
    }
    if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_relaxed) != 0u) {
        renderer()->renderStrip(y_start, ruido::kScanlineStripHeight, strip);
        auto* stripWords = reinterpret_cast<uint32_t*>(strip);
        for (size_t w = 0u; w < (kStripPixels / 2u); ++w) {
            const uint32_t pair = stripWords[w];
            stripWords[w] = ((pair >> 8u) & 0x00FF00FFu) | ((pair & 0x00FF00FFu) << 8u);
        }
        return strip;
    }
    constexpr uint16_t kSliceRows = 2u;
    constexpr size_t kSliceWords = (kSliceRows * ruido::kDisplayWidth) / 2u;
    for (uint16_t rowOff = 0u; rowOff < ruido::kScanlineStripHeight; rowOff += kSliceRows) {
        ruido_fm1_assist_dsp();
        uint16_t* sliceDst = strip + static_cast<size_t>(rowOff) * ruido::kDisplayWidth;
        renderer()->renderStrip(static_cast<uint16_t>(y_start + rowOff), kSliceRows, sliceDst);
        auto* sliceWords = reinterpret_cast<uint32_t*>(sliceDst);
        for (size_t w = 0u; w < kSliceWords; ++w) {
            const uint32_t pair = sliceWords[w];
            sliceWords[w] = ((pair >> 8u) & 0x00FF00FFu) | ((pair & 0x00FF00FFu) << 8u);
        }
    }
    ruido_fm1_assist_dsp();
    return strip;
}

} // extern "C"
