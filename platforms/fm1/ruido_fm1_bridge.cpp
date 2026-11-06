#include "ruido_fm1_bridge.h"

#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/ScopeHudRenderer.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

inline void* operator new(size_t, void* p) noexcept {
    return p;
}

extern "C" RuidoFm1Status g_ruido_fm1_status = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0};

namespace {

constexpr size_t kAudioChunkFrames = ruido::kBlockFrames;
constexpr size_t kStripPixels = ruido::kDisplayWidth * ruido::kScanlineStripHeight;
// Full-scale +-1.45f dacSafetyClip ceiling mapped inside signed Q15 [-32767, +32767]
constexpr float kFloatToQ15Scale = 22500.0f;

alignas(64) uint8_t g_dsp_arena[sizeof(ruido::DroneEngine)] __attribute__((section(".pool")));
alignas(64) uint8_t g_ctrl_arena[sizeof(ruido::RuidoController)] __attribute__((section(".pool")));
alignas(64) uint8_t g_renderer_arena[sizeof(ruido::ScopeHudRenderer)]
    __attribute__((section(".pool")));
alignas(32) uint16_t g_strip_ping_pong[2][kStripPixels] __attribute__((section(".pool")));

std::atomic<uint32_t> g_engine_ready{0u};

inline ruido::DroneEngine* dspEngine() noexcept {
    return reinterpret_cast<ruido::DroneEngine*>(g_dsp_arena);
}

inline ruido::RuidoController* controller() noexcept {
    return reinterpret_cast<ruido::RuidoController*>(g_ctrl_arena);
}

inline ruido::ScopeHudRenderer* renderer() noexcept {
    return reinterpret_cast<ruido::ScopeHudRenderer*>(g_renderer_arena);
}

inline int32_t clampQ15(float sample) noexcept {
    const float scaled = sample * kFloatToQ15Scale;
    const auto q = static_cast<int32_t>(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    if (q > 32767) {
        return 32767;
    }
    if (q < -32767) {
        return -32767;
    }
    return q;
}

} // namespace

extern "C" {

void ruido_fm1_init(void) {
    auto* dsp = new (g_dsp_arena) ruido::DroneEngine();
    auto* ctrl = new (g_ctrl_arena) ruido::RuidoController();
    auto* ui = new (g_renderer_arena) ruido::ScopeHudRenderer();

    // Hardware ALNK0 I2S runs at 44,100 Hz (`FS` in platform.c)
    dsp->init(44100.0f);
    ctrl->attachControlBus(&dsp->controlBus());
    dsp->snapParametersToLatest();
    ui->reset();

    g_engine_ready.store(1u, std::memory_order_release);
    g_ruido_fm1_status.magic = RUIDO_FM1_STATUS_MAGIC;
}

void ruido_fm1_set_bench_stress(uint32_t stress_mode) {
    if (g_engine_ready.load(std::memory_order_acquire) == 0u) {
        return;
    }
    auto* dsp = dspEngine();
    auto* ctrl = controller();
    auto* ui = renderer();
    if (stress_mode != 0u) {
        ruido::EngineControlState st{};
        for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
            auto& v = st.voices[i];
            v.frequencyHz = 110.0f * static_cast<float>(i + 1u);
            v.pulseWidth = 0.32f;
            v.drift = 0.35f;
            v.hpCutoffHz = 220.0f;
            v.hpResonance = 0.55f;
            v.lpCutoffHz = 1100.0f;
            v.lpResonance = 0.72f;
            v.amplitude = 0.88f; // Active MS-20 input overdrive (> 0.70 knee)
            v.pan = (i & 1u) != 0u ? 0.45f : -0.45f;
            v.lfoTarget = ruido::LfoTarget::LpCutoff;
            v.lfoShape = 0.65f;
            v.lfoRateHz = 3.5f;
            v.lfoDepth = 0.55f;
            v.lfoBTarget = ruido::LfoTarget::Pitch;
            v.lfoBShape = 0.90f;
            v.lfoBRateHz = 7.0f;
            v.lfoBDepth = 0.35f;
        }
        dsp->controlBus().publish(st);
        dsp->snapParametersToLatest();
    } else {
        dsp->init(44100.0f);
        ctrl->reset();
        dsp->snapParametersToLatest();
        ui->reset();
    }
}

void ruido_fm1_step_bench64(void) {
    if (g_engine_ready.load(std::memory_order_acquire) == 0u) {
        return;
    }
    float left[kAudioChunkFrames];
    float right[kAudioChunkFrames];
    dspEngine()->processBlock(left, right, kAudioChunkFrames);
}

void ruido_fm1_audio(int32_t* out_lr_q15, uint32_t num_frames) {
    if (g_engine_ready.load(std::memory_order_acquire) == 0u || out_lr_q15 == nullptr) {
        return;
    }

    auto* dsp = dspEngine();
    float left[kAudioChunkFrames];
    float right[kAudioChunkFrames];
    int32_t peak = g_ruido_fm1_status.peak_abs_q15;

    uint32_t offset = 0u;
    while (offset < num_frames) {
        const uint32_t chunk =
            ((num_frames - offset) > kAudioChunkFrames) ? kAudioChunkFrames : (num_frames - offset);
        dsp->processBlock(left, right, chunk);
        int32_t* dst = out_lr_q15 + 2u * offset;
        for (uint32_t i = 0u; i < chunk; ++i) {
            const int32_t sL = clampQ15(left[i]);
            const int32_t sR = clampQ15(right[i]);
            dst[2u * i] = sL;
            dst[2u * i + 1u] = sR;
            const int32_t aL = (sL < 0) ? -sL : sL;
            const int32_t aR = (sR < 0) ? -sR : sR;
            if (aL > peak) {
                peak = aL;
            }
            if (aR > peak) {
                peak = aR;
            }
        }
        offset += chunk;
    }

    g_ruido_fm1_status.peak_abs_q15 = peak;
    g_ruido_fm1_status.audio_blocks = g_ruido_fm1_status.audio_blocks + 1u;
}

void ruido_fm1_on_midi_rt(uint8_t status_byte) {
    if (g_engine_ready.load(std::memory_order_acquire) == 0u) {
        return;
    }
    controller()->onMidiRealtime(status_byte, -1.0f);
}

void ruido_fm1_tick_and_prepare_ms(uint32_t elapsed_ms) {
    if (g_engine_ready.load(std::memory_order_acquire) == 0u) {
        return;
    }
    const uint32_t clamped_ms =
        (elapsed_ms == 0u) ? 16u : ((elapsed_ms > 100u) ? 100u : elapsed_ms);
    const float dt_sec = static_cast<float>(clamped_ms) * 0.001f;
    auto* dsp = dspEngine();
    auto* ctrl = controller();
    auto* ui = renderer();

    ctrl->tick(dt_sec, dsp->telemetry());
    ui->ingestScopeRing(dsp->scopeBuffer());
    ui->prepareFrame(dt_sec, ctrl->state(), ctrl->hud(), dsp->telemetry());
}

const uint16_t* ruido_fm1_render_strip_be(uint16_t y_start, uint8_t slot) {
    uint16_t* strip = g_strip_ping_pong[slot & 1u];
    if (g_engine_ready.load(std::memory_order_acquire) == 0u) {
        return strip;
    }
    renderer()->renderStrip(y_start, ruido::kScanlineStripHeight, strip);
    for (size_t i = 0u; i < kStripPixels; ++i) {
        const uint16_t px = strip[i];
        strip[i] = static_cast<uint16_t>((px >> 8u) | (px << 8u));
    }
    return strip;
}

} // extern "C"
