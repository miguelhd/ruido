#include "ruido_fm1_bridge.h"

#include "core/RuidoController.h"
#include "fm1_cpu1.h"
#include "ruido_fm1_state.h"
#include "ui/ScopeHudRenderer.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

inline void* operator new(size_t, void* p) noexcept {
    return p;
}

extern "C" RuidoFm1Status g_ruido_fm1_status = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0, 0u, 0u, 0u, 0u};

/* Cache-line-isolated `.pool` arenas (shared IPC vs Core 1 DSP vs Core 0 UI/USB) */
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

volatile uint32_t* const kTimer4CntReg = reinterpret_cast<volatile uint32_t*>(0x00010804u);

alignas(64) uint8_t g_ctrl_arena[sizeof(ruido::RuidoController)] __attribute__((section(".pool")));
alignas(64) uint8_t g_renderer_arena[sizeof(ruido::ScopeHudRenderer)]
    __attribute__((section(".pool")));
alignas(64) uint8_t g_usb_resampler_arena[sizeof(ruido::Fm1UsbDriftResampler)]
    __attribute__((section(".pool")));
alignas(64) uint16_t g_strip_ping_pong[2][kStripPixels] __attribute__((section(".pool")));

uint32_t g_last_seen_c1_blocks = 0u;
uint32_t g_c1_stall_callbacks = 0u;

inline ruido::RuidoController* controller() noexcept {
    return reinterpret_cast<ruido::RuidoController*>(g_ctrl_arena);
}

inline ruido::ScopeHudRenderer* renderer() noexcept {
    return reinterpret_cast<ruido::ScopeHudRenderer*>(g_renderer_arena);
}

inline ruido::Fm1UsbDriftResampler* usbResampler() noexcept {
    return reinterpret_cast<ruido::Fm1UsbDriftResampler*>(g_usb_resampler_arena);
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

    const float initGain = masterQ12ToGain(initial_master_q12);
    g_ruido_ic_ctrl.masterGainBits.store(floatToBits(initGain), std::memory_order_relaxed);

    ruido::fm1::callC1Far(ruido_fm1_c1_construct, initGain);

    auto* ctrl = new (g_ctrl_arena) ruido::RuidoController();
    auto* ui = new (g_renderer_arena) ruido::ScopeHudRenderer();
    auto* usbRs = new (g_usb_resampler_arena) ruido::Fm1UsbDriftResampler();

    ctrl->attachControlBus(&ruidoSharedBuses()->controlBus);
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    ui->reset();
    usbRs->reset();

    g_last_seen_c1_blocks = 0u;
    g_c1_stall_callbacks = 0u;
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
    ruido::fm1::callC1Far(ruido_fm1_set_bench_stress, 0u);
    controller()->reset();
    ruido::fm1::callC1Far(ruido_fm1_c1_snap_latest);
    renderer()->reset();
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
    const float gain = masterQ12ToGain(master_q12);
    g_ruido_ic_ctrl.masterGainBits.store(floatToBits(gain), std::memory_order_relaxed);
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
    controller()->onMidiRealtime(status_byte, -1.0f);
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
    ruido_fm1_assist_dsp();
    ui->ingestScopeRing(buses->scopeBuffer);
    ruido_fm1_assist_dsp();
    ui->prepareFrame(dt_sec, ctrl->state(), ctrl->hud(), buses->telemetry);
    ruido_fm1_assist_dsp();

    const uint32_t assisted = ruidoCoProc()->assistedSpans();
    const uint32_t clampedAssist = (assisted > 0xFFFFu) ? 0xFFFFu : assisted;
    g_ruido_fm1_status.c1_flags =
        (g_ruido_fm1_status.c1_flags & 0x0000FFFFu) | (clampedAssist << 16u);
}

const uint16_t* ruido_fm1_render_strip_be(uint16_t y_start, uint8_t slot) {
    uint16_t* strip = g_strip_ping_pong[slot & 1u];
    if (g_ruido_ic_ctrl.engineReady.load(std::memory_order_acquire) == 0u) {
        return strip;
    }
    constexpr uint16_t kSliceRows = 4u;
    for (uint16_t rowOff = 0u; rowOff < ruido::kScanlineStripHeight; rowOff += kSliceRows) {
        ruido_fm1_assist_dsp();
        uint16_t* sliceDst = strip + static_cast<size_t>(rowOff) * ruido::kDisplayWidth;
        renderer()->renderStrip(static_cast<uint16_t>(y_start + rowOff), kSliceRows, sliceDst);
    }
    ruido_fm1_assist_dsp();
    for (size_t i = 0u; i < kStripPixels; ++i) {
        const uint16_t px = strip[i];
        strip[i] = static_cast<uint16_t>((px >> 8u) | (px << 8u));
    }
    return strip;
}

} // extern "C"
