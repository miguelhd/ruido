#include "ruido_fm1_bridge.h"

#include "core/RuidoController.h"
#include "fm1_audio_io.h"
#include "fm1_cpu1.h"
#include "fm1_panel_io.h"
#include "fm1_storage_io.h"
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
constexpr uint32_t kC1StallFallbackCallbacks = 16u;
constexpr uint32_t kC1BenchTimeoutTicks = 20000u * 24u; // 20 ms @ 24 MHz TIMER4
constexpr uint32_t kC1BenchMaxSpins = 5000000u;
constexpr uint32_t kC1FlashHoldTimeoutTicks = 10000u * 24u; // 10 ms @ 24 MHz TIMER4
constexpr uint32_t kTicksPer64Sec = 1536000000u;            // 64 s * 24,000,000 Hz TIMER4 crystal
constexpr float kInvTimer4Hz = 1.0f / 24000000.0f;
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
int32_t g_last_out_l = 0;
int32_t g_last_out_r = 0;
float g_latest_master_gain = 0.85f;
uint8_t g_panel_constructed = 0u;
uint8_t g_midi_t4_anchored = 0u;
uint32_t g_midi_t4_prev = 0u;
uint32_t g_midi_t4_mod64 = 0u;
uint32_t g_midi_rt_events = 0u;
ruido::fm1::Fm1NvsRuntimeState g_nvs_state{};
uint32_t g_solo_reason = 0u;
volatile uint8_t g_c0_in_assist = 0u;
uint32_t g_last_empty_assist_t4 = 0u;
uint8_t g_assist_t4_valid = 0u;

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
inline float masterQ12ToGain(uint32_t master_q12) noexcept {
    return ruido::fm1::masterQ12ToGain(master_q12);
}

inline void syncCore1StatsToStatus() noexcept {
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c1Blocks);
    g_ruido_fm1_status.c1_blocks = g_ruido_ic_ctrl.c1Blocks.load(std::memory_order_acquire);
    g_ruido_fm1_status.c1_last64_ticks =
        g_ruido_ic_ctrl.c1Last64Ticks.load(std::memory_order_relaxed);
}

inline void refreshGate43StatusWords() noexcept {
    syncCore1StatsToStatus();
    const auto* ctrl = controller();
    const auto& hud = ctrl->hud();
    const auto& st = ctrl->state();
    const uint32_t modelBits = static_cast<uint32_t>(hud.keyboardModel) & 3u;
    const uint32_t slotBits = static_cast<uint32_t>(ctrl->snapshots().activeSlot()) & 3u;
    const uint32_t syncBit = hud.midiClockSynced ? 1u : 0u;
    const uint32_t runBit = st.global.masterRun ? 1u : 0u;
    const uint32_t dirtyBit = hud.snapshotDirty ? 1u : 0u;
    const uint32_t hudBits = static_cast<uint32_t>(hud.mode) & 0x3Fu;
    const uint32_t evBits = static_cast<uint32_t>(panel()->inputEventCount());
    g_ruido_fm1_status.panel_state = modelBits | (slotBits << 2u) | (syncBit << 4u) |
                                     (runBit << 5u) | (dirtyBit << 6u) | (hudBits << 8u) |
                                     (evBits << 16u);
    const float clampedBpm = ruido::clampf(hud.effectiveClockBpm, 0.0f, 6500.0f);
    const auto bpmX10 = static_cast<uint32_t>(clampedBpm * 10.0f + 0.5f) & 0xFFFFu;
    g_ruido_fm1_status.midi_clock_state = bpmX10 | ((g_midi_rt_events & 0xFFFFu) << 16u);
    g_ruido_fm1_status.led_state_summary = ledMod()->lastSummary();
    const float clampedHz = ruido::clampf(st.voices[0].frequencyHz, 0.0f, 6550.0f);
    const auto osc1HzX10 = static_cast<uint32_t>(clampedHz * 10.0f + 0.5f) & 0xFFFFu;
    g_ruido_fm1_status.nvs_and_osc1_state = static_cast<uint32_t>(g_nvs_state.bootLoadedMask) |
                                            (static_cast<uint32_t>(g_nvs_state.savesOk) << 8u) |
                                            (osc1HzX10 << 16u);
}

[[gnu::noinline]] void engageCore1Fallback(uint32_t reason) noexcept {
    g_solo_reason = (g_solo_reason != 0u) ? g_solo_reason : reason;
    g_ruido_ic_ctrl.c1RunEnable.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FallbackActive.store(1u, std::memory_order_release);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
    fm1_cpu1_pause();
    ruidoSharedBuses()->controlBus.abortFromFault();
    ruidoCoProc()->abortFromFault();
    ruido::fm1::callC1Far(ruido_fm1_c1_enter_solo_fallback);
    g_ruido_fm1_status.c1_flags = g_ruido_fm1_status.c1_flags | 2u;
}

inline bool checkCore1SelfFault() noexcept {
    if (g_ruido_ic_ctrl.c1SelfFault.load(std::memory_order_acquire) != 0u) {
        if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_relaxed) == 0u) {
            engageCore1Fallback(3u);
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
    g_ruido_ic_ctrl.c1Booted = 0u;
    g_ruido_ic_ctrl.c1FaultCode = 0u;
    g_ruido_ic_ctrl.c1FaultAux = 0u;
    __asm__ volatile("flushinv [%0]\n\tssync\n\tcsync" ::"r"(&g_ruido_ic_ctrl) : "memory");
    g_ruido_ic_ctrl.engineReady.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1RunEnable.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FallbackActive.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1BenchTicks.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FlashHold.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FlashAck.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1Blocks.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1Last64Ticks.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1Max64Ticks.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c0ResetMaxReq.store(0u, std::memory_order_relaxed);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c1Blocks);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c0ResetMaxReq);

    g_latest_master_gain = masterQ12ToGain(initial_master_q12);
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
    ui->armBootIntro();
    usbRs->reset();

    g_last_seen_c1_blocks = 0u;
    g_c1_stall_callbacks = 0u;
    g_last_out_l = 0;
    g_last_out_r = 0;
    g_midi_t4_anchored = 0u;
    g_midi_t4_prev = 0u;
    g_midi_t4_mod64 = 0u;
    g_midi_rt_events = 0u;
    g_last_empty_assist_t4 = 0u;
    g_assist_t4_valid = 0u;
    g_nvs_state.reset();
    refreshGate43StatusWords();
}

void ruido_fm1_diag_sync_c1(void) {
    const uint32_t code = g_ruido_ic_ctrl.c1FaultCode;
    if (code != 0u) {
        g_ruido_boot_diag.c1_code = code;
        g_ruido_boot_diag.c1_aux = g_ruido_ic_ctrl.c1FaultAux;
    }
    const uint32_t selfBit =
        (g_ruido_ic_ctrl.c1SelfFault.load(std::memory_order_relaxed) != 0u) ? 0x100u : 0u;
    const uint32_t bootBit = (g_ruido_ic_ctrl.c1Booted != 0u) ? 0x200u : 0u;
    g_ruido_boot_diag.c1_state = (g_solo_reason & 0xFFu) | selfBit | bootBit;
}

uint32_t ruido_fm1_diag_max_c1_ticks(void) {
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c1Blocks);
    return g_ruido_ic_ctrl.c1Max64Ticks.load(std::memory_order_acquire);
}

void ruido_fm1_diag_clear_counters(void) {
    g_ruido_fm1_status.c1_flags = g_ruido_fm1_status.c1_flags & ~0xFF00u;
    g_ruido_fm1_status.peak_abs_q15 = 0;
    const uint32_t nextReq = g_ruido_ic_ctrl.c0ResetMaxReq.load(std::memory_order_relaxed) + 1u;
    g_ruido_ic_ctrl.c0ResetMaxReq.store(nextReq, std::memory_order_release);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c0ResetMaxReq);
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
        ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
        const uint32_t t0 = *kTimer4CntReg;
        uint32_t spins = 0u;
        while (g_ruido_ic_ctrl.c1BenchReq.load(std::memory_order_acquire) != 0u) {
            if ((spins & 63u) == 0u) {
                ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl);
                ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
            }
            if (checkCore1SelfFault() || (*kTimer4CntReg - t0) >= kC1BenchTimeoutTicks ||
                ++spins >= kC1BenchMaxSpins) {
                engageCore1Fallback(4u);
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
    auto* buses = ruidoSharedBuses();
    /* Re-publish NVS-restored controller state and snap Core 1 smoothers after boot benchmarks */
    buses->controlBus.publish(ctrl->state());
    ruido::fm1::callC1Far(ruido_fm1_set_bench_stress, 0u);
    panel()->reset(g_latest_master_gain);
    panel()->syncMasterPot(*ctrl, g_latest_master_gain);
    (void)ctrl->consumeSavedSlotMask();
    (void)ctrl->consumeSettingsDirty();
    (void)ctrl->consumeLivePatchDirty();
    ledMod()->reset();
    ledMod()->updateTargets(ctrl->leds());
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    renderer()->reset();
    renderer()->armBootIntro();
    g_last_out_l = 0;
    g_last_out_r = 0;
    g_midi_t4_anchored = 0u;
    g_midi_rt_events = 0u;
    g_nvs_state.clearPendingAfterRestore();
    refreshGate43StatusWords();
}

uint32_t ruido_fm1_nvs_load_boot(RuidoFm1BlobLoadFn load_fn, RuidoFm1TailReadFn tail_read_fn,
                                 RuidoFm1BlobSaveFn boot_compact_save_fn) {
    if (load_fn == nullptr || g_panel_constructed == 0u) {
        return 0u;
    }
    const uint32_t loadedMask = ruido::fm1::loadBootNvsBlobs(
        load_fn, *controller(), *panel(), *ledMod(), g_latest_master_gain, g_nvs_state,
        tail_read_fn, boot_compact_save_fn);
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    refreshGate43StatusWords();
    return loadedMask;
}

uint32_t ruido_fm1_nvs_poll_save(uint32_t now_ms, uint32_t force_flush, RuidoFm1BlobSaveFn save_fn,
                                 RuidoFm1TailWriteFn tail_write_fn,
                                 RuidoFm1PreEraseFn pre_erase_fn) {
    if (save_fn == nullptr || g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return 0u;
    }
    const uint32_t availBlocks = ruidoAudioRing()->availableBlocks();
    const uint32_t writes =
        ruido::fm1::pollAndSaveNvsBlobs(now_ms, force_flush, save_fn, pre_erase_fn, *controller(),
                                        g_nvs_state, tail_write_fn, availBlocks);
    if (writes > 0u) {
        refreshGate43StatusWords();
    }
    return writes;
}

void ruido_fm1_arm_audio(uint32_t enable_core1) {
    const bool useCore1 = (enable_core1 != 0u) && !checkCore1SelfFault();
    if (!useCore1) {
        engageCore1Fallback(5u);
    }
    ruido::fm1::callC1Far(ruido_fm1_c1_prime_blocks, ruido::Fm1AudioBlockRing::kNumBlocks,
                          useCore1 ? 1u : 0u);
    if (useCore1) {
        g_ruido_ic_ctrl.c1RunEnable.store(1u, std::memory_order_release);
    }
    g_ruido_ic_ctrl.engineReady.store(1u, std::memory_order_release);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
    g_ruido_fm1_status.magic = RUIDO_FM1_STATUS_MAGIC;
    refreshGate43StatusWords();
}

uint32_t ruido_fm1_start_core1(void) {
    ruido_fm1_set_coproc_enabled(1u);
    if (fm1_cpu1_start(fm1_cpu1_boot_xip, &g_ruido_ic_ctrl.c1Booted) != 0) {
        g_ruido_fm1_status.c1_flags = g_ruido_fm1_status.c1_flags | 1u;
        return 1u;
    }
    ruido_fm1_set_coproc_enabled(0u);
    engageCore1Fallback(1u);
    return 0u;
}

void ruido_fm1_enter_solo_mode(void) {
    ruido_fm1_set_coproc_enabled(0u);
    engageCore1Fallback(2u);
}

void ruido_fm1_quiesce_for_ota(void) {
    g_ruido_ic_ctrl.engineReady.store(0u, std::memory_order_release);
    g_ruido_ic_ctrl.c1FlashHold.store(0u, std::memory_order_relaxed);
    if (g_panel_constructed != 0u) {
        ledMod()->updateTargets(ruido::LedState{});
    }
    if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) == 0u) {
        engageCore1Fallback(6u);
    }
    ruido::fm1::hwMemoryBarrier();
}

void ruido_fm1_resume_after_ota(void) {
    g_ruido_ic_ctrl.engineReady.store(1u, std::memory_order_release);
    ruido::fm1::hwMemoryBarrier();
}

void ruido_fm1_flash_hold_begin(void) {
    if ((g_ruido_fm1_status.c1_flags & 1u) == 0u || checkCore1SelfFault()) {
        return;
    }
    g_ruido_ic_ctrl.c1FlashAck.store(0u, std::memory_order_relaxed);
    g_ruido_ic_ctrl.c1FlashHold.store(1u, std::memory_order_release);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
    const uint32_t t0 = *kTimer4CntReg;
    uint32_t spins = 0u;
    while (g_ruido_ic_ctrl.c1FlashAck.load(std::memory_order_acquire) == 0u) {
        if ((spins & 63u) == 0u) {
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl);
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
        }
        if (checkCore1SelfFault()) {
            break;
        }
        if ((*kTimer4CntReg - t0) >= kC1FlashHoldTimeoutTicks || ++spins >= 500000u) {
            engageCore1Fallback(7u);
            break;
        }
        if (ruidoCoProc()->hasPendingWork()) {
            ruido::fm1::callC1Far(ruido_fm1_c1_assist_dsp);
        }
    }
    ruido::fm1::hwMemoryBarrier();
}

void ruido_fm1_flash_hold_end(void) {
    g_c1_stall_callbacks = 0u;
    g_ruido_ic_ctrl.c1FlashHold.store(0u, std::memory_order_release);
    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
}

void ruido_fm1_assist_dsp(void) {
    if (g_c0_in_assist != 0u) {
        return;
    }
    if (checkCore1SelfFault()) {
        if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_relaxed) != 0u) {
            const uint32_t avail = ruidoAudioRing()->availableBlocks();
            const uint32_t pumpCount = (avail < 12u) ? 6u : 2u;
            g_c0_in_assist = 1u;
            ruido::fm1::callC1Far(ruido_fm1_c1_pump_solo_ring, pumpCount);
            g_c0_in_assist = 0u;
        }
        return;
    }
    const uint32_t now_t4 = *kTimer4CntReg;
    if (g_assist_t4_valid != 0u && (now_t4 - g_last_empty_assist_t4) < (24u * 24u)) {
        return;
    }
    if (ruidoCoProc()->hasPendingWork()) {
        g_assist_t4_valid = 0u;
        g_c0_in_assist = 1u;
        ruido::fm1::callC1Far(ruido_fm1_c1_assist_dsp);
        g_c0_in_assist = 0u;
    } else {
        g_last_empty_assist_t4 = *kTimer4CntReg;
        g_assist_t4_valid = 1u;
    }
}

uint32_t ruido_fm1_audio_cushion_blocks(void) {
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_relaxed) == 0u ||
        g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_relaxed) != 0u ||
        g_ruido_ic_ctrl.c1SelfFault.load(std::memory_order_relaxed) != 0u) {
        return ruido::Fm1AudioBlockRing::kNumBlocks;
    }
    if (!controller()->state().global.masterRun) {
        return ruido::Fm1AudioBlockRing::kNumBlocks;
    }
    return ruidoAudioRing()->availableBlocksOnConsumer();
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
    ruido_fm1_assist_dsp();
    const bool btnOrEnc =
        pnl->pollButtonsAndEncoders(*ctrl, pressed_btns, released_btns, cur_btns, enc_steps);
    ruido_fm1_assist_dsp();
    const bool keyTouched = pnl->pollKeysAndPressure(*ctrl, cur_notes, note_press_edges);
    ruido_fm1_assist_dsp();
    if (btnOrEnc || keyTouched) {
        g_nvs_state.lastPanelTouchMs = now_ms;
        renderer()->notifyUserInput();
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
    auto* ring = ruidoAudioRing();
    int32_t peak = g_ruido_fm1_status.peak_abs_q15;
    bool hadMiss = false;
    uint32_t offset = 0u;

    while (offset < num_frames) {
        const uint32_t chunk =
            ((num_frames - offset) > kAudioChunkFrames) ? kAudioChunkFrames : (num_frames - offset);
        int32_t* dst = out_lr_q15 + 2u * offset;
        const ruido::Fm1AudioBlock* blk = ring->beginPop();
        if (blk != nullptr) {
            ruido::fm1::hwMemoryBarrier();
            ruido::fm1::copyStereoBlockToDma(*blk, dst, chunk, g_last_out_l, g_last_out_r);
            if (blk->peakAbsQ15 > peak) {
                peak = blk->peakAbsQ15;
            }
            ruido::fm1::hwMemoryBarrier();
            ring->commitPop();
        } else {
            ruido::fm1::slewStereoBlockToZero(dst, chunk, g_last_out_l, g_last_out_r);
            hadMiss = true;
        }
        offset += chunk;
    }

    syncCore1StatsToStatus();
    const bool c1Faulted = (g_ruido_ic_ctrl.c1SelfFault.load(std::memory_order_acquire) != 0u) ||
                           (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) != 0u);
    if (hadMiss && !c1Faulted &&
        g_ruido_ic_ctrl.c1FlashHold.load(std::memory_order_relaxed) == 0u) {
        const uint32_t curFlags = g_ruido_fm1_status.c1_flags;
        const uint32_t und = (curFlags >> 8u) & 0xFFu;
        if (und < 255u) {
            g_ruido_fm1_status.c1_flags = (curFlags & ~0xFF00u) | ((und + 1u) << 8u);
        }

        const uint32_t curC1Blocks = g_ruido_fm1_status.c1_blocks;
        if (curC1Blocks == g_last_seen_c1_blocks) {
            ++g_c1_stall_callbacks;
            if (g_c1_stall_callbacks >= kC1StallFallbackCallbacks) {
                fm1_cpu1_pause();
                g_solo_reason = (g_solo_reason != 0u) ? g_solo_reason : 7u;
                g_ruido_ic_ctrl.c1SelfFault.store(1u, std::memory_order_release);
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
    const ruido::fm1::RuidoUsbFeedParams params{
        in_lr_q15,   num_in_frames, rate_48k,  fill_frames, target_fill,
        ua_ring_buf, ua_ring_mask,  write_pos, free_slots,  stream_seq,
    };
    return ruido::fm1::callC1Far(ruido_fm1_c1_usb_feed, usbResampler(), &params);
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
        (elapsed_ms == 0u) ? 16u : ((elapsed_ms > 250u) ? 250u : elapsed_ms);
    const float dt_sec = static_cast<float>(clamped_ms) * 0.001f;
    auto* buses = ruidoSharedBuses();
    auto* ctrl = controller();
    auto* ui = renderer();

    ruido_fm1_assist_dsp();
    ruido::hwCacheFlushInvalidateRange(&buses->telemetry, sizeof(buses->telemetry));
    ctrl->tick(dt_sec, buses->telemetry, []() noexcept { ruido_fm1_assist_dsp(); });
    ruido_fm1_assist_dsp();
    ledMod()->updateTargets(ctrl->leds());
    ruido_fm1_assist_dsp();
    ui->ingestScopeRing(buses->scopeBuffer, []() noexcept { ruido_fm1_assist_dsp(); });
    ruido_fm1_assist_dsp();
    ui->prepareFrame(dt_sec, ctrl->state(), ctrl->hud(), buses->telemetry,
                     []() noexcept { ruido_fm1_assist_dsp(); });
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
    /* Drain Core 1's 44.1 kHz 2048-frame scope ring 4x per frame so zero samples are ever dropped
     */
    if ((y_start & 63u) == 0u && y_start != 0u) {
        renderer()->ingestScopeRing(ruidoSharedBuses()->scopeBuffer,
                                    []() noexcept { ruido_fm1_assist_dsp(); });
    }
    constexpr uint16_t kSliceRows = 4u;
    static_assert(ruido::kScanlineStripHeight % kSliceRows == 0u, "kSliceRows must divide strip");
    for (uint16_t rowOff = 0u; rowOff < ruido::kScanlineStripHeight; rowOff += kSliceRows) {
        ruido_fm1_assist_dsp();
        uint16_t* sliceDst = strip + static_cast<size_t>(rowOff) * ruido::kDisplayWidth;
        renderer()->renderStrip(static_cast<uint16_t>(y_start + rowOff), kSliceRows, sliceDst, true,
                                []() noexcept { ruido_fm1_assist_dsp(); });
    }
    ruido_fm1_assist_dsp();
    return strip;
}

uint16_t* ruido_fm1_acquire_ops_strip(void) {
    return g_strip_ping_pong[0];
}

} // extern "C"
