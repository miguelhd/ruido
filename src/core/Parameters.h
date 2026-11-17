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
    float lfoBRateHz = 0.25f;              // [0.01, 2000.0] Hz LFO B rate
    float lfoBShape = 0.0f;                // [0.0, 1.0] LFO B shape
    float lfoBDepth = 0.0f;                // [0.0, 1.0] LFO B modulation amount (0 = off)
    LfoTarget lfoTarget = LfoTarget::LpCutoff;  // KNOB1 LFO A destination (Pitch..Pan)
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
    if (a <= kSilentAmpThreshold) {
        return {1.0f, 0.0f};
    }
    if (a <= kCleanAmpKnee) {
        return {1.0f, a * (0.72f / kCleanAmpKnee)};
    }
    const float u = (a - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee));
    const float preDrive = 1.0f + 3.5f * u * (0.65f + 0.35f * u);
    const float postVca = 0.72f + 0.43f * u;
    return {preDrive, postVca};
}

// Global circuit & master routing state (Left encoders + SEL + MONO/POLY + PLAY/STOP + MIDI clock).
struct GlobalParams {
    float circuitCoupling = 0.25f;  // ALGORITHM knob: [0.0, 1.0] rail sag -> starved-rail sync
    float masterTrim = 0.85f;       // Software headroom trim before DAC / pedals
    float clockBpm = 120.0f;        // Master rhythm clock BPM (120 default, or 30..300 MIDI clock)
    uint8_t clockResetSeq = 0u;     // Incremented on MIDI 0xFA Start to snap bar phase to step 0
    bool monoCollapse = false;      // MONO vs POLY (true = MONO L==R)
    bool masterRun = true;          // PLAY/STOP button (clickless master kill/run)
    bool pitchPressureMode = false; // SEL button: false = MS-20 Filter/LPG, true = Pitch Bend
};

// White Keys 5-8 continuous pitch-bend ratio (+24 semitones / +2 octaves at full pressure
// when SEL pitchPressureMode is active; 1.0f otherwise).
RUIDO_ALWAYS_INLINE float computeVoicePitchRatio(float pressure, float pitchMode) noexcept {
    const float pp = clampf(pressure, 0.0f, 1.0f) * clampf(pitchMode, 0.0f, 1.0f);
    return (pp > 0.0f) ? fastExp2(2.0f * pp * pp) : 1.0f;
}

// Live performance keyboard interaction models selected via SEL button + SELECT encoder.
enum class KeyboardModel : uint8_t {
    LiveMixer = 0,  // 1/3: 4-Oscillator Live Mixer, FX & 4-Slot Scenes (default)
    CircuitLab = 1, // 2/3: Circuit-Bent CMOS Patch Bay
    RhythmLab = 2,  // 3/3: Stutter, Clock Dividers & Euclidean Grooves
    Count = 3
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
    std::array<uint8_t, kNumOscillators> rhythmMode{0u, 0u, 0u, 0u};  // VoiceRhythmMode per osc
    std::array<uint8_t, kNumOscillators> rhythmParam{0u, 0u, 0u, 0u}; // Div/Euclid index per osc
    float overloadSurge = 0.0f;   // [0, 1] -> 4.5x MS-20 drive + 100% starved-rail sync
    float dubLpfDrop = 0.0f;      // [0, 1] -> master resonant MS-20 LPF curtain
    float masterHpfSweep = 0.0f;  // [0, 1] -> master resonant MS-20 HPF bass-cut sweep
    float spatialFocus = 0.0f;    // -1.0 = center focus, +1.0 = hyper-wide spread
    float ratioLockAmount = 0.0f; // [0, 1] thermal drift damping
    float ringModMix = 0.0f;      // [0, 1] 4x paired ring/XOR sideband crossfade
    float hardSyncAmount = 0.0f;  // [0, 1] band-limited OSC 1 -> OSC 2..4 hard sync
    float lfoTurbo = 0.0f;        // [0, 1] 1x -> 32x LFO rate overclock + dry-patch floor
    float filterScream = 0.0f;    // [0, 1] twin Korg35 HPF + LPF self-oscillation sweep
    float rhythmPump = 0.0f;      // [0, 1] 4-voice sidechain pump ducking swell
    float rhythmChase = 0.0f;     // [0, 1] 1->2->3->4 step loop sequencer
    float rhythmTempo = 0.0f;     // [0, 1] 1x -> 4x master rhythm clock speed
    float rhythmPluck = 0.0f;     // [0, 1] percussive MS-20 low-pass filter pluck
    float rhythmShort = 0.0f;     // [0, 1] 50% -> 6% pinpoint gate length
    float rhythmSwing = 0.0f;     // [0, 1] pure 16th-note timing shuffle groove
    float rhythmPingPong = 0.0f;  // [0, 1] alternating L/R stereo gate bounce
    float rhythmRoll = 0.0f;      // [0, 1] momentary x2..x8 fast ratchet gate fill
    uint8_t soloMask = 0u;        // 4-bit mask of soloed oscillators (bit v = OSC v+1 solo active)
    uint8_t muteMask = 0u;        // 4-bit mask of muted oscillators (bit v = OSC v+1 mute active)
    bool stasisFreeze = false;    // DSP modulation freeze flag (unit-test & engine stasis hook)
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

// Verify lock-free atomic & POD bus invariants mandated by src/core/AGENTS.md & src/dsp/AGENTS.md
static_assert(__is_trivially_copyable(VoiceParams), "VoiceParams must remain trivially copyable");
static_assert(__is_trivially_copyable(GlobalParams), "GlobalParams must remain trivially copyable");
static_assert(__is_trivially_copyable(KeyboardModState),
              "KeyboardModState must remain trivially copyable");
static_assert(__is_trivially_copyable(EngineControlState),
              "EngineControlState must remain trivially copyable for lock-free bus");
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "uint32_t atomics must be lock-free on target architecture");
static_assert(std::atomic<size_t>::is_always_lock_free,
              "size_t atomics must be lock-free on target architecture");
static_assert(std::atomic<float>::is_always_lock_free,
              "float atomics must be lock-free on target architecture");

// Lock-free, 100% CAS-free / spinlock-free triple-buffered SPSC control bus
// (Core 0 writer -> Core 1 audio reader). Uses Dekker store-load ordering on the 3-slot index
// handoff so neither core ever executes `compare_exchange` or hardware `lockset` spinlocks, and
// Core 1 (`consumeLatest`) executes zero loops.
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
        c0Swapping_.store(1u, std::memory_order_seq_cst);
        uint32_t spins = 0u;
        while (c1Swapping_.load(std::memory_order_seq_cst) != 0u &&
               c1Faulted_.load(std::memory_order_relaxed) == 0u && ++spins < 8388608u) {
        }
        const uint32_t prevReady = readySlot_.load(std::memory_order_acquire);
        readySlot_.store(writeIdx_, std::memory_order_release);
        hasNew_.store(1u, std::memory_order_release);
        writeIdx_ = prevReady;
        c0Swapping_.store(0u, std::memory_order_release);
    }

    void abortFromFault() noexcept { c1Faulted_.store(1u, std::memory_order_release); }

    // Called from Core 1 at block start (Audio thread). Single-consumer only, O(1) wait-free.
    const EngineControlState& consumeLatest() noexcept {
        if (hasNew_.load(std::memory_order_acquire) != 0u) {
            c1Swapping_.store(1u, std::memory_order_seq_cst);
            if (c0Swapping_.load(std::memory_order_seq_cst) == 0u &&
                hasNew_.load(std::memory_order_acquire) != 0u) {
                const uint32_t nextRead = readySlot_.load(std::memory_order_acquire);
                readySlot_.store(readIdx_, std::memory_order_release);
                hasNew_.store(0u, std::memory_order_release);
                readIdx_ = nextRead;
                if (++readSeq_ == 0u) {
                    readSeq_ = 1u;
                }
            }
            c1Swapping_.store(0u, std::memory_order_release);
        }
        return buffers_[readIdx_];
    }

    // Monotonic non-zero consumer generation counter (changes only when `consumeLatest` swaps).
    uint32_t readSeq() const noexcept { return readSeq_; }

  private:
    std::array<EngineControlState, 3> buffers_{};
    uint32_t writeIdx_ = 0u;
    alignas(64) uint32_t readIdx_ = 1u;
    uint32_t readSeq_ = 1u;
    std::atomic<uint32_t> c1Swapping_{0u};
    alignas(64) std::atomic<uint32_t> c0Swapping_{0u};
    std::atomic<uint32_t> readySlot_{2u}; // Slot 2 initially parked in middle
    std::atomic<uint32_t> hasNew_{0u};
    std::atomic<uint32_t> c1Faulted_{0u};
};

// Quantize per-voice scope tap in [-3.5, +3.5] to signed int16 (1/8192 ~= 0.0095 px step)
inline int16_t quantizeVoiceScopeQ13(float sample) noexcept {
    const float scaled = clampf(sample * 8192.0f, -28672.0f, 28672.0f);
    return static_cast<int16_t>(static_cast<int32_t>(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f)));
}

inline constexpr float unquantizeVoiceScopeQ13(int16_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 8192.0f);
}

// Decimated stereo + 4-voice scope frame & rising-edge trigger flags (Core 1 -> Core 0, 14 bytes).
struct ScopeFrame {
    int16_t leftQ13 = 0;
    int16_t rightQ13 = 0;
    std::array<int16_t, kNumOscillators> voiceQ13{0, 0, 0, 0};
    uint8_t voiceRisingMask = 0u;
};
static_assert(sizeof(ScopeFrame) == 14u, "ScopeFrame must pack tightly into 14 bytes");
static_assert(__is_trivially_copyable(ScopeFrame), "ScopeFrame must remain trivially copyable");

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

    // Batch SPSC span writer: amortizes acquire/release memory barriers across 16-frame DSP spans.
    size_t beginPushSpan(size_t maxCount, size_t& outWritePos) const noexcept {
        const size_t w = writePos_.load(std::memory_order_relaxed);
        const size_t r = readPos_.load(std::memory_order_acquire);
        outWritePos = w;
        const size_t used = w - r;
        if (used >= kScopeRingCapacity) {
            return 0u;
        }
        const size_t freeSlots = kScopeRingCapacity - used;
        return (freeSlots < maxCount) ? freeSlots : maxCount;
    }

    ScopeFrame& slotAt(size_t pos) noexcept { return buffer_[pos & kScopeRingMask]; }

    void commitPushSpan(size_t newWritePos) noexcept {
        writePos_.store(newWritePos, std::memory_order_release);
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

    size_t writePosRelaxed() const noexcept { return writePos_.load(std::memory_order_relaxed); }
    size_t readPosRelaxed() const noexcept { return readPos_.load(std::memory_order_relaxed); }
    const void* writePosAddr() const noexcept { return &writePos_; }
    const void* readPosAddr() const noexcept { return &readPos_; }

  private:
    std::array<ScopeFrame, kScopeRingCapacity> buffer_{};
    alignas(64) std::atomic<size_t> writePos_{0u};
    alignas(64) std::atomic<size_t> readPos_{0u};
};

// Real-time acoustic telemetry for Core 0 varibright LED pulsing & headroom indicator.
struct alignas(64) EngineTelemetry {
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
    // Sample-accurate 4-voice rhythm/stutter open-gate mask (bit v = 1 open, 0 chopped silent)
    std::atomic<uint8_t> stutterGateMask{0x0Fu};
    // Instantaneous [0.0, 1.0) swung master 16-step bar phase from Core 1 -> Core 0 LEDs/HUD
    std::atomic<float> rhythmPhase{0.0f};
};

// Cache-line-isolated Core 0 <-> Core 1 shared IPC buses.
// On dual-core `AC791N` (`fm1_cpu1`), `EngineSharedBuses` can be placed in uncached shared SRAM
// while `DroneEngineCore` stays in Core 1 L1-cached SRAM with zero false sharing.
struct alignas(64) EngineSharedBuses {
    LockFreeControlBus controlBus{};
    ScopeRingBuffer scopeBuffer{};
    EngineTelemetry telemetry{};
};

} // namespace ruido
