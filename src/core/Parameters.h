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
    LpCutoff = 0,    // LPF (default)
    LpResonance = 1, // LP RES
    HpCutoff = 2,    // HPF
    HpResonance = 3, // HP RES
    Amplitude = 4,   // AMP
    Pitch = 5,       // PITCH
    Pan = 6,         // PAN
    Count = 7
};

// User-facing per-oscillator state (serializable into <= 128B quantized snapshots in Milestone 3).
struct VoiceParams {
    float frequencyHz = 110.0f;  // [20, 4000] Hz audio oscillator range
    float amplitude = 0.70f;     // [0.0, 1.0] post-filter mix level
    float pan = 0.0f;            // [-1.0 (Left), +1.0 (Right)]
    float hpCutoffHz = 20.0f;    // [20.0, 20000.0] Hz (20 Hz = parked flat)
    float hpResonance = 0.0f;    // [0.0, 1.0] Smart-Park independent MS-20 HPF Peak
    float lpCutoffHz = 20000.0f; // [20.0, 20000.0] Hz (20 kHz = parked flat)
    float lpResonance = 0.0f;    // [0.0, 1.0] Smart-Park independent MS-20 LPF Peak
    float lfoRateHz = 0.25f;     // [0.02, 30.0] Hz dedicated per-voice LFO speed
    float lfoShape = 0.0f;       // [0.0, 1.0] Sine -> Tri -> Saw -> Slewed Square -> Smooth Drift
    float lfoDepth = 0.0f;       // [0.0, 1.0] Unipolar LFO modulation amount (0 = off)
    LfoTarget lfoTarget = LfoTarget::LpCutoff; // KNOB1 LFO destination
    bool muted = false;                        // White Keys 1-4 latching mute
};

// Global circuit & master routing state (Left encoders + SEL + MONO/POLY + PLAY/STOP).
struct GlobalParams {
    float circuitCoupling = 0.25f;  // ALGORITHM knob: [0.0, 1.0] CD40106 Vdd rail sag
    float masterTrim = 0.85f;       // Software headroom trim before DAC / pedals
    bool monoCollapse = false;      // MONO vs POLY (true = Dual-Mono L==R)
    bool masterRun = true;          // PLAY/STOP button (clickless master kill/run)
    bool pitchPressureMode = false; // SEL button: false = MS-20 Filter/LPG, true = Pitch Bend
};

// Effective filter + VCA parameters produced by White Keys 5-8 continuous pressure:
//  - Filter/LPG mode (pitchMode = 0): zero pitch shift; active voices sweep MS-20 HPF/LPF Accent
//    + independent Korg35 HP/LP Resonance; muted voices swell through a Vactrol LPG.
//  - Pitch Bend mode (pitchMode = 1): filter stays at base settings while pressure bends pitch
//    upward (+6 semitones at full pressure) and swells muted voices open via the VCA gate.
struct ModulatedFilterParams {
    float hpCutoffHz;
    float hpResonance;
    float lpCutoffHz;
    float lpResonance;
    float effectiveGate;
};

inline float computeVoicePitchRatio(float pressure, float pitchMode) noexcept {
    const float pp = clampf(pressure, 0.0f, 1.0f) * clampf(pitchMode, 0.0f, 1.0f);
    return (pp > 0.0f) ? fastExp2(0.50f * pp * pp) : 1.0f;
}

inline ModulatedFilterParams computeVoicePressureMacro(float hpBase, float hpResBase,
                                                       float lpBase, float lpResBase,
                                                       float pressure, float activeGate,
                                                       float pitchMode = 0.0f) noexcept {
    const float p = clampf(pressure, 0.0f, 1.0f);
    const float g = clampf(activeGate, 0.0f, 1.0f);
    if (p <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, g};
    }

    const float pm = clampf(pitchMode, 0.0f, 1.0f);
    const float fp = p * (1.0f - pm);

    const float lpgGate = p * (0.42f + 0.58f * p);
    const float effGate = clampf(g + (1.0f - g) * lpgGate, 0.0f, 1.0f);
    if (fp <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, effGate};
    }

    // 1. Active Voice: MS-20 Filter Accent + Independent Korg35 HP/LP Resonance
    // log2(2500 Hz) = 11.287712f; 3.0-octave transition up to 20,000 Hz
    const float lpOpenness = smoothstep01((fastLog2(lpBase) - 11.287712f) * (1.0f / 3.0f));
    const float lpOctDelta = (4.0f * (1.0f - lpOpenness) - 1.35f * lpOpenness) * fp;
    const float lpAccent =
        clampf(lpBase * fastExp2(lpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float hpOctDelta = 5.4f * lpOpenness * fp;
    const float maxSafeHp = clampf(lpAccent * 0.72f, kFilterCutoffMinHz, 12000.0f);
    const float hpCeil = (hpBase > maxSafeHp) ? hpBase : maxSafeHp;
    const float hpAccent = clampf(hpBase * fastExp2(hpOctDelta), kFilterCutoffMinHz, hpCeil);

    const float hpResAccent = clampf(hpResBase + 0.58f * fp, 0.0f, 0.93f);
    const float lpResAccent = clampf(lpResBase + 0.62f * fp, 0.0f, 0.93f);

    // 2. Muted Voice: Vactrol Low-Pass Gate (warm 95 Hz sub murmur -> full open harmonics)
    const float invFp = 1.0f - fp;
    const float lpLpg =
        clampf(lpBase * fastExp2(-7.0f * invFp * invFp), 95.0f, kFilterCutoffMaxHz);
    const float hpLpg = hpBase;
    const float hpResLpg = hpResBase;
    const float lpResLpg = clampf(lpResBase + 0.38f * fp * fp, 0.0f, 0.90f);

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
        constexpr std::array<float, kNumOscillators> kInitFreqs = {
            55.0f, 82.41f, 110.15f, 164.81f};
        constexpr std::array<float, kNumOscillators> kInitPans = {
            -0.35f, 0.35f, -0.65f, 0.65f};
        for (size_t i = 0; i < kNumOscillators; ++i) {
            s.voices[i].frequencyHz = kInitFreqs[i];
            s.voices[i].amplitude = 0.65f;
            s.voices[i].pan = kInitPans[i];
            s.voices[i].hpCutoffHz = kFilterCutoffMinHz;
            s.voices[i].hpResonance = 0.0f;
            s.voices[i].lpCutoffHz = kFilterCutoffMaxHz;
            s.voices[i].lpResonance = 0.0f;
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
            if (sharedState_.compare_exchange_weak(
                    prevReady, nextState, std::memory_order_acq_rel, std::memory_order_relaxed)) {
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
    uint8_t osc1RisingEdge = 0u;
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
};

} // namespace ruido
