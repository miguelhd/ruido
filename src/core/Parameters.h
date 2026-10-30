#pragma once

#include "core/Config.h"
#include "core/DspMath.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Per-voice LFO modulation destinations (1-to-1 with VoiceParams circuit parameters).
enum class LfoTarget : uint8_t {
    Pitch = 0,       // PITCH
    LpCutoff = 1,    // LPF (default)
    LpResonance = 2, // LP RES
    HpCutoff = 3,    // HPF
    HpResonance = 4, // HP RES
    Amplitude = 5,   // AMP
    Pan = 6,         // PAN
    Count = 7
};

// User-facing per-oscillator state (serializable into <= 128B quantized snapshots in Milestone 3).
struct VoiceParams {
    float frequencyHz = 110.0f;                // [0.05, 20000] Hz unbroken sub-cycle -> audio range
    float hpCutoffHz = kFilterCutoffMinHz;     // [2.0, 28000.0] Hz (2 Hz + 0% RES = parked flat)
    float hpResonance = 0.0f;                  // [0.0, 1.0] Rev 2 (0-60%) -> Korg35 (60-100%) Peak
    float lpCutoffHz = kFilterCutoffMaxHz;     // [2.0, 28000.0] Hz (28 kHz + 0% RES = parked flat)
    float lpResonance = 0.0f;                  // [0.0, 1.0] Rev 2 (0-60%) -> Korg35 (60-100%) Peak
    float amplitude = 0.65f;                   // [0.0, 0.70] clean VCA; (0.70, 1.0] MS-20 overdrive
    float pan = 0.0f;                          // [-1.0 (Left), +1.0 (Right)]
    float lfoRateHz = 0.25f;                   // [0.01, 2000.0] Hz ultra-slow -> audio-rate FM/AM
    float lfoShape = 0.0f;                     // [0.0, 1.0] Sine -> Tri -> Saw -> Square -> Drift
    float lfoDepth = 0.0f;                     // [0.0, 1.0] LFO modulation amount (0 = off)
    LfoTarget lfoTarget = LfoTarget::LpCutoff; // KNOB1 LFO destination
    bool muted = false;                        // White Keys 1-4 latching mute
};

// EDIT page two-zone channel gain law:
//  - [0.00, 0.70]: clean post-filter VCA attenuation (0.0 -> 0.72x clean ceiling, preDrive = 1.0x)
//  - (0.70, 1.00]: 1.0x -> 4.5x pre-filter MS-20 input overdrive + 0.72x -> 1.15x post-filter trim
inline constexpr float kCleanAmpKnee = 0.70f;

struct VoiceChannelGain {
    float preDrive;
    float postVca;
};

inline constexpr VoiceChannelGain computeVoiceChannelGain(float amplitude) noexcept {
    const float a = clampf(amplitude, 0.0f, 1.0f);
    if (a <= kCleanAmpKnee) {
        return {1.0f, a * (0.72f / kCleanAmpKnee)};
    }
    const float u = (a - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee));
    const float preDrive = 1.0f + 3.5f * u * (0.65f + 0.35f * u);
    const float postVca = 0.72f + 0.43f * u;
    return {preDrive, postVca};
}

// Global circuit & master routing state (Left encoders + SEL + MONO/POLY + PLAY/STOP).
struct GlobalParams {
    float circuitCoupling = 0.25f;  // ALGORITHM knob: [0.0, 1.0] rail sag -> starved-rail sync
    float masterTrim = 0.85f;       // Software headroom trim before DAC / pedals
    bool monoCollapse = false;      // MONO vs POLY (true = Dual-Mono L==R)
    bool masterRun = true;          // PLAY/STOP button (clickless master kill/run)
    bool pitchPressureMode = false; // SEL button: false = MS-20 Filter/LPG, true = Pitch Bend
};

// Effective filter + VCA parameters produced by White Keys 5-8 continuous pressure:
//  - Filter/LPG mode (pitchMode = 0): zero pitch shift; active voices sweep MS-20 HPF/LPF Accent
//    + full Rev 2 / Korg35 HP/LP Resonance (free to cross HPF > LPF); muted voices swell via LPG.
//  - Pitch Bend mode (pitchMode = 1): filter stays at base settings while pressure bends pitch
//    upward (+24 semitones / +2 octaves at full pressure) and swells muted voices via VCA gate.
struct ModulatedFilterParams {
    float hpCutoffHz;
    float hpResonance;
    float lpCutoffHz;
    float lpResonance;
    float effectiveGate;
};

inline float computeVoicePitchRatio(float pressure, float pitchMode) noexcept {
    const float pp = clampf(pressure, 0.0f, 1.0f) * clampf(pitchMode, 0.0f, 1.0f);
    return (pp > 0.0f) ? fastExp2(2.0f * pp * pp) : 1.0f;
}

inline ModulatedFilterParams computeVoicePressureMacro(float hpBase, float hpResBase, float lpBase,
                                                       float lpResBase, float pressure,
                                                       float activeGate,
                                                       float pitchMode = 0.0f) noexcept {
    const float p = clampf(pressure, 0.0f, 1.0f);
    const float g = clampf(activeGate, 0.0f, 1.0f);
    if (p <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, g};
    }

    const float pm = clampf(pitchMode, 0.0f, 1.0f);
    const float fp = p * (1.0f - pm);

    const float lpgGate = p * (0.65f + 0.35f * p);
    const float effGate = clampf(g + (1.0f - g) * lpgGate, 0.0f, 1.0f);
    if (fp <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, effGate};
    }

    // 1. Active Voice: Uncapped MS-20 Filter Accent + Full Korg35 HP/LP Resonance
    // log2(2500 Hz) = 11.287712f; 3.5-octave transition up to 28,000 Hz
    const float lpOpenness = smoothstep01((fastLog2(lpBase) - 11.287712f) * (1.0f / 3.5f));
    const float lpOctDelta = (4.8f * (1.0f - lpOpenness) - 1.65f * lpOpenness) * fp;
    const float lpAccent =
        clampf(lpBase * fastExp2(lpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float hpOctDelta = 7.2f * lpOpenness * fp;
    const float hpAccent =
        clampf(hpBase * fastExp2(hpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float hpResAccent = clampf(hpResBase + 0.72f * fp, 0.0f, 1.0f);
    const float lpResAccent = clampf(lpResBase + 0.78f * fp, 0.0f, 1.0f);

    // 2. Muted Voice: Vactrol Low-Pass Gate (warm 85 Hz sub murmur -> full base LPF harmonics)
    const float lpgCeil = (lpBase > 85.0f) ? lpBase : 85.0f;
    const float lpgSpanOct = fastLog2(lpgCeil * (1.0f / 85.0f));
    const float lpLpg =
        clampf(85.0f * fastExp2(lpgSpanOct * fp * (0.48f + 0.52f * fp)), 85.0f, kFilterCutoffMaxHz);
    const float hpLpg = hpBase;
    const float hpResLpg = hpResBase;
    const float lpResLpg = clampf(lpResBase + 0.55f * fp * fp, 0.0f, 1.0f);

    return {
        hpLpg + g * (hpAccent - hpLpg),
        hpResLpg + g * (hpResAccent - hpResLpg),
        lpLpg + g * (lpAccent - lpLpg),
        lpResLpg + g * (lpResAccent - lpResLpg),
        effGate,
    };
}

// Complete patch state + momentary SARADC finger-pressure modulation from Core 0 -> Core 1.
struct EngineControlState {
    std::array<VoiceParams, kNumOscillators> voices{};
    GlobalParams global{};
    // White Keys 5-8 per-voice continuous SARADC carbon-pill pressure in [0.0, 1.0]
    std::array<float, kNumOscillators> voicePressure{0.0f, 0.0f, 0.0f, 0.0f};

    static EngineControlState makeDefault() noexcept {
        EngineControlState s{};
        // Musical open-fifth / octave starting chord with gentle stereo spread
        constexpr std::array<float, kNumOscillators> kInitFreqs = {55.0f, 82.41f, 110.15f, 164.81f};
        constexpr std::array<float, kNumOscillators> kInitPans = {-0.35f, 0.35f, -0.65f, 0.65f};
        for (size_t i = 0; i < kNumOscillators; ++i) {
            s.voices[i].frequencyHz = kInitFreqs[i];
            s.voices[i].hpCutoffHz = kFilterCutoffMinHz;
            s.voices[i].hpResonance = 0.0f;
            s.voices[i].lpCutoffHz = kFilterCutoffMaxHz;
            s.voices[i].lpResonance = 0.0f;
            s.voices[i].amplitude = 0.65f;
            s.voices[i].pan = kInitPans[i];
            s.voices[i].lfoRateHz = 0.25f;
            s.voices[i].lfoShape = 0.0f;
            s.voices[i].lfoDepth = 0.0f;
            s.voices[i].lfoTarget = LfoTarget::LpCutoff;
            s.voices[i].muted = false;
        }
        return s;
    }
};

// Verify lock-free atomic invariants mandated by src/dsp/AGENTS.md Rule #3
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "uint32_t atomics must be lock-free on target architecture");
static_assert(std::atomic<size_t>::is_always_lock_free,
              "size_t atomics must be lock-free on target architecture");
static_assert(std::atomic<float>::is_always_lock_free,
              "float atomics must be lock-free on target architecture");

// Lock-free triple-buffered control bus (Core 0 writer -> Core 1 audio reader).
// Never blocks either core and guarantees tear-free multi-parameter reads.
class LockFreeControlBus {
  public:
    LockFreeControlBus() noexcept {
        const EngineControlState init = EngineControlState::makeDefault();
        buffers_[0] = init;
        buffers_[1] = init;
        buffers_[2] = init;
    }

    // Called from Core 0 (UI / GPIO / SARADC thread). Single-producer only.
    void publish(const EngineControlState& state) noexcept {
        buffers_[writeIdx_] = state;
        // Atomically swap writeIdx_ with the clean middle slot holding latest data.
        // Uses acq_rel so reclaiming readyIdx synchronizes-with Core 1's prior release.
        uint32_t prevReady = sharedState_.load(std::memory_order_relaxed);
        uint32_t nextState = 0u;
        do {
            const uint32_t readyIdx = prevReady & 0x3u;
            nextState = (writeIdx_ & 0x3u) | kNewDataBit;
            if (sharedState_.compare_exchange_weak(prevReady, nextState, std::memory_order_acq_rel,
                                                   std::memory_order_relaxed)) {
                writeIdx_ = readyIdx;
                break;
            }
        } while (true);
    }

    // Called from Core 1 at block start (Audio thread). Single-consumer only.
    const EngineControlState& consumeLatest() noexcept {
        uint32_t current = sharedState_.load(std::memory_order_acquire);
        if ((current & kNewDataBit) != 0u) {
            do {
                const uint32_t newReadIdx = current & 0x3u;
                // Return old readIdx_ to pool and clear kNewDataBit
                const uint32_t replacement = readIdx_ & 0x3u;
                if (sharedState_.compare_exchange_weak(current, replacement,
                                                       std::memory_order_acq_rel,
                                                       std::memory_order_acquire)) {
                    readIdx_ = newReadIdx;
                    break;
                }
            } while ((current & kNewDataBit) != 0u);
        }
        return buffers_[readIdx_];
    }

  private:
    static constexpr uint32_t kNewDataBit = 0x4u;

    std::array<EngineControlState, 3> buffers_{};
    alignas(64) uint32_t writeIdx_ = 0u;
    alignas(64) uint32_t readIdx_ = 1u;
    alignas(64) std::atomic<uint32_t> sharedState_{2u}; // Slot 2 initially parked in middle
};

// Quantize per-voice scope tap in [-3.5, +3.5] to signed int16 (1/8192 ~= 0.0095 px step)
inline int16_t quantizeVoiceScopeQ13(float sample) noexcept {
    const float scaled = clampf(sample, -3.5f, 3.5f) * 8192.0f;
    const int32_t q = static_cast<int32_t>(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    if (q < -28672) {
        return -28672;
    }
    if (q > 28672) {
        return 28672;
    }
    return static_cast<int16_t>(q);
}

inline constexpr float unquantizeVoiceScopeQ13(int16_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 8192.0f);
}

// Decimated stereo + 4-voice scope frame & rising-edge trigger flags (Core 1 -> Core 0).
struct ScopeFrame {
    float left = 0.0f;
    float right = 0.0f;
    uint8_t voiceRisingMask = 0u;
    std::array<int16_t, kNumOscillators> voiceQ13{0, 0, 0, 0};
};

// Lock-free SPSC ring buffer feeding the 240x240 CRT scope on Core 0.
class ScopeRingBuffer {
  public:
    bool push(const ScopeFrame& frame) noexcept {
        const size_t w = writePos_.load(std::memory_order_relaxed);
        const size_t r = readPos_.load(std::memory_order_acquire);
        if ((w - r) >= kScopeRingCapacity) {
            return false; // Full: drop gracefully without blocking audio
        }
        buffer_[w & kScopeRingMask] = frame;
        writePos_.store(w + 1u, std::memory_order_release);
        return true;
    }

    size_t popBatch(ScopeFrame* dst, size_t maxCount) noexcept {
        const size_t r = readPos_.load(std::memory_order_relaxed);
        const size_t w = writePos_.load(std::memory_order_acquire);
        const size_t avail = w - r;
        const size_t count = (avail < maxCount) ? avail : maxCount;
        if (count > 0u) {
            for (size_t i = 0u; i < count; ++i) {
                dst[i] = buffer_[(r + i) & kScopeRingMask];
            }
            readPos_.store(r + count, std::memory_order_release);
        }
        return count;
    }

    size_t available() const noexcept {
        const size_t r = readPos_.load(std::memory_order_acquire);
        const size_t w = writePos_.load(std::memory_order_acquire);
        const size_t diff = w - r;
        return (diff <= kScopeRingCapacity) ? diff : 0u;
    }

  private:
    std::array<ScopeFrame, kScopeRingCapacity> buffer_{};
    alignas(64) std::atomic<size_t> writePos_{0u};
    alignas(64) std::atomic<size_t> readPos_{0u};
};

// Real-time acoustic telemetry for Core 0 varibright LED pulsing & headroom indicator.
struct EngineTelemetry {
    std::atomic<float> peakLeft{0.0f};
    std::atomic<float> peakRight{0.0f};
    // Instantaneous [0.0, 1.0] gate state per Schmitt trigger (for < 20 Hz key LED pulse)
    std::array<std::atomic<float>, kNumOscillators> oscGateHigh{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
    // Instantaneous [-1.0, +1.0] bipolar LFO wave output per voice (HUD animation & LFO LED)
    std::array<std::atomic<float>, kNumOscillators> lfoOutput{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
    // Instantaneous [0.0, 1.0) LFO phase per voice (anchors moving optical phase bead on HUD)
    std::array<std::atomic<float>, kNumOscillators> lfoPhase{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
    // Packed 5 x 6-bit signed C^2 B-spline drift knots per voice for tear-free HUD drift curve
    std::array<std::atomic<uint32_t>, kNumOscillators> lfoDriftPacked{
        std::atomic<uint32_t>{0u},
        std::atomic<uint32_t>{0u},
        std::atomic<uint32_t>{0u},
        std::atomic<uint32_t>{0u},
    };
    // Post-VCA per-voice peak amplitude [0.0, ~2.0] for EDIT page plasma & overdrive LED flare
    std::array<std::atomic<float>, kNumOscillators> voicePeak{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
};

} // namespace ruido
