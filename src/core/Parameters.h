#pragma once

#include "core/Config.h"
#include "core/DspMath.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Per-voice LFO modulation destinations (LFO A: Pitch..Pan; LFO B adds LfoARate & LfoADepth).
enum class LfoTarget : uint8_t {
    Pitch = 0,       // PITCH
    PulseWidth = 1,  // WIDTH (band-limited 5-95% Schmitt PWM)
    LpCutoff = 2,    // LPF (default)
    LpResonance = 3, // LP RES
    HpCutoff = 4,    // HPF
    HpResonance = 5, // HP RES
    Amplitude = 6,   // AMP
    Pan = 7,         // PAN
    LfoARate = 8,    // A RATE  (LFO B only: modulates LFO A speed)
    LfoADepth = 9,   // A DEPTH (LFO B only: swells/fades LFO A strength)
    Count = 10
};

inline constexpr size_t kNumLfoATargets = 8u; // Pitch .. Pan
inline constexpr size_t kNumLfoBTargets = static_cast<size_t>(LfoTarget::Count);
using LfoSignalArray = std::array<float, kNumVoiceLfos>;
using LfoDriftArray = std::array<uint32_t, kNumVoiceLfos>;

// Bank index inside LfoSignalArray / EngineTelemetry: [0..3] = Osc 1..4 LFO A, [4..7] = Osc 1..4
// LFO B
inline constexpr size_t lfoBankIndex(size_t voiceIdx, size_t subIdx) noexcept {
    return (subIdx & 1u) * kNumOscillators + (voiceIdx & 3u);
}
// Decode SELECT focus slot 0..7 (LFO A OSC 1, LFO B OSC 1 .. LFO A OSC 4, LFO B OSC 4)
inline constexpr uint8_t lfoFocusVoice(uint8_t slotFocus) noexcept {
    return static_cast<uint8_t>((slotFocus >> 1u) & 3u);
}
inline constexpr uint8_t lfoFocusSub(uint8_t slotFocus) noexcept {
    return static_cast<uint8_t>(slotFocus & 1u);
}

// User-facing per-oscillator state (serializable into <= 128B quantized snapshots in Milestone 3).
struct VoiceParams {
    float frequencyHz = 110.0f;            // [0.05, 20000] Hz unbroken sub-cycle -> audio range
    float pulseWidth = 0.50f;              // [0.05, 0.95] CMOS Schmitt duty cycle (50% = square)
    float drift = kDefaultVoiceDrift;      // [0.00, 1.00] 1/f analog thermal pitch instability
    float hpCutoffHz = kFilterCutoffMinHz; // [2.0, 28000.0] Hz (2 Hz + 0% RES = parked flat)
    float hpResonance = 0.0f;              // [0.0, 1.0] Rev 2 (0-60%) -> Korg35 (60-100%) Peak
    float lpCutoffHz = kFilterCutoffMaxHz; // [2.0, 28000.0] Hz (28 kHz + 0% RES = parked flat)
    float lpResonance = 0.0f;              // [0.0, 1.0] Rev 2 (0-60%) -> Korg35 (60-100%) Peak
    float amplitude = kDefaultVoiceAmp;    // [0.0, 0.70] clean VCA; (0.70, 1.0] MS-20 overdrive
    float pan = 0.0f;                      // [-1.0 (Left), +1.0 (Right)]
    float lfoRateHz = 0.25f;               // [0.01, 2000.0] Hz LFO A rate
    float lfoShape = 0.0f;                 // [0.0, 1.0] LFO A Sine->Tri->Saw->Square->Drift
    float lfoDepth = 0.0f;                 // [0.0, 1.0] LFO A modulation amount (0 = off)
    LfoTarget lfoTarget = LfoTarget::LpCutoff;  // KNOB1 LFO A destination (Pitch..Pan)
    float lfoBRateHz = 0.25f;                   // [0.01, 2000.0] Hz LFO B rate
    float lfoBShape = 0.0f;                     // [0.0, 1.0] LFO B shape
    float lfoBDepth = 0.0f;                     // [0.0, 1.0] LFO B modulation amount (0 = off)
    LfoTarget lfoBTarget = LfoTarget::LpCutoff; // KNOB1 LFO B destination (Pitch..LfoADepth)
    bool muted = false;                         // White Keys 1-4 latching mute
};

inline constexpr float lfoSlotRateHz(const VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoRateHz : vp.lfoBRateHz;
}
inline constexpr float lfoSlotShape(const VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoShape : vp.lfoBShape;
}
inline constexpr float lfoSlotDepth(const VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoDepth : vp.lfoBDepth;
}
inline constexpr LfoTarget lfoSlotTarget(const VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoTarget : vp.lfoBTarget;
}
inline float& lfoSlotRateHzRef(VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoRateHz : vp.lfoBRateHz;
}
inline float& lfoSlotShapeRef(VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoShape : vp.lfoBShape;
}
inline float& lfoSlotDepthRef(VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoDepth : vp.lfoBDepth;
}
inline LfoTarget& lfoSlotTargetRef(VoiceParams& vp, size_t sub) noexcept {
    return ((sub & 1u) == 0u) ? vp.lfoTarget : vp.lfoBTarget;
}

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
    bool monoCollapse = false;      // MONO vs POLY (true = MONO L==R)
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

    // 1. Active Voice: Full-Throw [0..1] MS-20 Vocal Formant + Korg35 HP/LP Resonance
    // Avoids wasting the first 50% of pressure below 24 Hz when HPF is parked at 2.0 Hz,
    // and sweeps open 28 kHz LPF down into the vocal formant band across the entire throw.
    const float lpOpenness = smoothstep01((fastLog2(lpBase) - 11.287712f) * (1.0f / 3.5f));
    const float lpOctDelta = (4.8f * (1.0f - lpOpenness) - 3.80f * lpOpenness) * fp;
    const float lpAccent =
        clampf(lpBase * fastExp2(lpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float hpFloorHz =
        (hpBase < 22.0f) ? (hpBase + (22.0f - hpBase) * smoothstep01(fp * 4.5f)) : hpBase;
    const float hpOctDelta = 4.65f * lpOpenness * fp;
    const float hpAccent =
        clampf(hpFloorHz * fastExp2(hpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float resCurve = fp * (0.65f + 0.35f * fp);
    const float hpResAccent = clampf(hpResBase + 0.80f * resCurve, 0.0f, 1.0f);
    const float lpResAccent = clampf(lpResBase + 0.85f * resCurve, 0.0f, 1.0f);

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

// Live performance keyboard interaction models selected via SEL button + SELECT encoder.
enum class KeyboardModel : uint8_t {
    LiveMixer = 0,  // 1/4: 4-Oscillator Live Mixer, FX & 4-Slot Scenes (default)
    CircuitLab = 1, // 2/4: Circuit-Bent CMOS Patch Bay
    Harmonium = 2,  // 3/4: Just-Intonation Harmonium / Monochord (framework slot)
    RhythmLab = 3,  // 4/4: Polyrhythmic Gate & Chopper Matrix (framework slot)
    Count = 4
};

// Non-destructive momentary punch-in modulation overlaid onto the steady knob patch (Core 0 -> 1).
struct KeyboardModState {
    std::array<float, kNumOscillators> voiceGainMul{1.0f, 1.0f, 1.0f, 1.0f};
    std::array<float, kNumOscillators> voicePitchMul{1.0f, 1.0f, 1.0f, 1.0f};
    std::array<float, kNumOscillators> voicePwDelta{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> stutterRateHz{6.0f, 6.0f, 6.0f, 6.0f};
    std::array<float, kNumOscillators> stutterDepth{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> xmodDepth{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> foldDepth{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> crushDepth{0.0f, 0.0f, 0.0f, 0.0f};
    float overloadSurge = 0.0f;   // [0, 1] -> 4.5x MS-20 drive + 100% starved-rail sync
    float dubLpfDrop = 0.0f;      // [0, 1] -> master resonant MS-20 LPF curtain
    float spatialFocus = 0.0f;    // -1.0 = center focus, +1.0 = hyper-wide spread
    float ratioLockAmount = 0.0f; // [0, 1] thermal drift damping
    float ringModMix = 0.0f;      // [0, 1] 4x paired ring/XOR sideband crossfade
    float hardSyncAmount = 0.0f;  // [0, 1] band-limited OSC 1 -> OSC 2..4 hard sync
    float lfoTurbo = 0.0f;        // [0, 1] 1x -> 32x LFO rate overclock + dry-patch floor
    float filterScream = 0.0f;    // [0, 1] twin Korg35 HPF + LPF self-oscillation sweep
    uint8_t soloMask = 0u;        // 4-bit mask of soloed oscillators (bit v = OSC v+1 solo active)
    bool stasisFreeze = false;    // 11th tall key: freezes 8 LFOs + thermal drift + scope
};

// Complete patch state + momentary SARADC finger-pressure modulation from Core 0 -> Core 1.
struct EngineControlState {
    std::array<VoiceParams, kNumOscillators> voices{};
    GlobalParams global{};
    // Per-voice MS-20 formant/LPG pressure in [0.0, 1.0] (driven by KeyboardEngine)
    std::array<float, kNumOscillators> voicePressure{0.0f, 0.0f, 0.0f, 0.0f};
    KeyboardModState keyMod{};

    static EngineControlState makeDefault() noexcept {
        EngineControlState s{};
        // Musical open-fifth / octave starting chord with gentle stereo spread
        constexpr std::array<float, kNumOscillators> kInitFreqs = {55.0f, 82.41f, 110.15f, 164.81f};
        constexpr std::array<float, kNumOscillators> kInitPans = {-0.35f, 0.35f, -0.65f, 0.65f};
        for (size_t i = 0; i < kNumOscillators; ++i) {
            s.voices[i].frequencyHz = kInitFreqs[i];
            s.voices[i].pulseWidth = 0.50f;
            s.voices[i].drift = kDefaultVoiceDrift;
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
            s.voices[i].lfoBRateHz = 0.25f;
            s.voices[i].lfoBShape = 0.0f;
            s.voices[i].lfoBDepth = 0.0f;
            s.voices[i].lfoBTarget = LfoTarget::LpCutoff;
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
    uint32_t writeIdx_ = 0u;
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
    // Instantaneous [-1.0, +1.0] bipolar LFO wave output ([0..3]=Osc 1..4 A, [4..7]=Osc 1..4 B)
    std::array<std::atomic<float>, kNumVoiceLfos> lfoOutput{
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
    };
    // Instantaneous [0.0, 1.0) LFO phase ([0..3]=Osc 1..4 A, [4..7]=Osc 1..4 B)
    std::array<std::atomic<float>, kNumVoiceLfos> lfoPhase{
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
        std::atomic<float>{0.0f}, std::atomic<float>{0.0f},
    };
    // Packed 5 x 6-bit signed C^2 B-spline drift knots ([0..3]=Osc 1..4 A, [4..7]=Osc 1..4 B)
    std::array<std::atomic<uint32_t>, kNumVoiceLfos> lfoDriftPacked{
        std::atomic<uint32_t>{0u}, std::atomic<uint32_t>{0u}, std::atomic<uint32_t>{0u},
        std::atomic<uint32_t>{0u}, std::atomic<uint32_t>{0u}, std::atomic<uint32_t>{0u},
        std::atomic<uint32_t>{0u}, std::atomic<uint32_t>{0u},
    };
    // Post-VCA per-voice peak amplitude [0.0, ~2.0] for EDIT page plasma & overdrive LED flare
    std::array<std::atomic<float>, kNumOscillators> voicePeak{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
    // Instantaneous [-1.0, +1.0] normalized 1/f thermal pitch drift per voice (EDIT -> DRIFT)
    std::array<std::atomic<float>, kNumOscillators> voiceDriftNorm{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
    // Sample-accurate 4-voice stutter chopper open-gate mask (bit v = 1 open, 0 chopped silent)
    std::atomic<uint8_t> stutterGateMask{0x0Fu};
};

} // namespace ruido
