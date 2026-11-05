#pragma once

#include "core/Config.h"
#include "core/ParameterSmoother.h"
#include "core/Parameters.h"
#include "dsp/CircuitPatchMatrix.h"
#include "dsp/HalfBandDecimator.h"
#include "dsp/Ms20Filter.h"
#include "dsp/RhythmGateMatrix.h"
#include "dsp/SchmittOscillatorBank.h"
#include "dsp/VoiceLfoBank.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Hot L1-cacheable Core 1 synthesis & filtering state (~5.9 KB), decoupled from the 42.9 KB
// shared Core 0 <-> Core 1 IPC buses (`EngineSharedBuses`) so dual-core FM-1 (`fm1_cpu1`) can
// allocate `EngineSharedBuses` in uncached SRAM while keeping `DroneEngineCore` in Core 1 L1.
class DroneEngineCore {
  public:
    DroneEngineCore() noexcept = default;

    void init(EngineSharedBuses& buses, float baseSampleRate = kBaseSampleRate) noexcept;
    void reset(EngineSharedBuses& buses) noexcept;

    void setAnalogCharacterEnabled(bool enableDrift, bool enableRcTilt) noexcept {
        oscBank_.setAnalogCharacterEnabled(enableDrift, enableRcTilt);
        lfoBank_.setAnalogCharacterEnabled(enableDrift);
    }

    void snapParametersToLatest(EngineSharedBuses& buses) noexcept;

    void processBlock(EngineSharedBuses& buses, float* outLeft, float* outRight,
                      size_t numFrames) noexcept;

  private:
    struct VoiceSmoothers {
        LogFreqSmoother freqHz;
        LogFreqSmoother hpCutoffHz;
        LogFreqSmoother lpCutoffHz;
        LogFreqSmoother keyPitchMul;
        CriticallyDampedSmootherBank<6> potSmoothers; // [0..5] pw, drift, hpRes, lpRes, amp, pan
        VactrolSmoother pressure;
        OnePoleSmoother stutterGate;
        OnePoleSmootherBank<3> vactrolMods;   // [0]=vactrolGate, [1]=keyGainMul, [2]=stutterDepth
        OnePoleSmootherBank<3> circuitDepths; // [0]=xmodDepth, [1]=foldDepth, [2]=crushDepth
    };

    struct VoiceCoeffCache {
        float hpHz = -1.0f;
        float hpRes = -1.0f;
        float lpHz = -1.0f;
        float lpRes = -1.0f;
        float preDrive = -1.0f;
        float panEff = -99.0f;
        float panL = 0.70710678f;
        float panR = 0.70710678f;
    };

    void updateTargetsFromState(const EngineControlState& state, bool snapImmediate) noexcept;

    StereoHalfBandDecimator4x decimator_{};
    float baseSampleRate_ = kBaseSampleRate;
    float oversampledRate_ = kOversampledRate;
    float invOversampledRate_ = kInvOversampledRate;
    float hzToBasePhaseU32_ = 4294967296.0f / kBaseSampleRate;
    float hzToOversampledPhaseU32_ = 4294967296.0f / kOversampledRate;

    SchmittOscillatorBank oscBank_{};
    VoiceLfoBank lfoBank_{};
    std::array<Ms20DualFilter, kNumOscillators> filters_{};
    std::array<Ms20DualFilter::Coeffs, kNumOscillators> filterCoeffs_{};

    std::array<VoiceSmoothers, kNumOscillators> voiceSmoothers_{};
    std::array<VoiceCoeffCache, kNumOscillators> coeffCache_{};
    std::array<VoiceCrushState, kNumOscillators> crushStates_{};
    RhythmClockState rhythmState_{};
    std::array<uint32_t, kNumOscillators> stutterPhases_{0x80000000u, 0x80000000u, 0x80000000u,
                                                         0x80000000u};
    std::array<float, kNumOscillators> stutterRatesHz_{6.0f, 6.0f, 6.0f, 6.0f};
    std::array<float, kNumOscillators> prevOscFreqHz_{55.0f, 82.41f, 110.15f, 164.81f};
    std::array<float, kNumOscillators> prevOscSamples_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> prevCapRamp_{0.0f, 0.0f, 0.0f, 0.0f};
    CriticallyDampedSmoother couplingSmoother_{};
    CriticallyDampedSmoother masterTrimSmoother_{};
    OnePoleSmoother monoMixSmoother_{};          // 0.0 = Stereo (POLY), 1.0 = Dual-Mono (MONO)
    OnePoleSmoother masterRunSmoother_{};        // 0.0 = Muted (STOP), 1.0 = Active (PLAY)
    OnePoleSmoother pitchModeSmoother_{};        // 0.0 = MS-20 Filter/LPG, 1.0 = Pitch Bend
    OnePoleSmootherBank<6> punchBusSmoothers_{}; // [0..5] surge, dubLpf, spatial, ring, sync, scrm
    bool stasisFreeze_ = false;

    // 36-frame delay aligning 4-voice rising edges & scope taps with PolyBLEP + FIR delay
    static constexpr uint32_t kScopeAlignDelaySlots = 37u;
    std::array<uint64_t, kNumOscillators> voiceEdgeDelayLines_{0u, 0u, 0u, 0u};
    std::array<std::array<int16_t, kNumOscillators>, kScopeAlignDelaySlots> voiceScopeDelay_{};
    uint32_t voiceScopeDelayWrite_ = 0u;
};

// Complete Core 1 portable synthesis & filtering engine composing `EngineSharedBuses` +
// `DroneEngineCore` for single-object use in unit tests, macOS/Wasm simulators, and single-core
// `dinghy` targets.
class DroneEngine {
  public:
    DroneEngine() noexcept { core_.init(buses_, kBaseSampleRate); }

    void init(float baseSampleRate = kBaseSampleRate) noexcept {
        core_.init(buses_, baseSampleRate);
    }

    void reset() noexcept { core_.reset(buses_); }

    EngineSharedBuses& sharedBuses() noexcept { return buses_; }
    const EngineSharedBuses& sharedBuses() const noexcept { return buses_; }

    DroneEngineCore& core() noexcept { return core_; }
    const DroneEngineCore& core() const noexcept { return core_; }

    // Control bus (Core 0 calls controlBus().publish(state))
    LockFreeControlBus& controlBus() noexcept { return buses_.controlBus; }

    // Scope SPSC ring buffer (Core 0 calls scopeBuffer().popBatch(...))
    ScopeRingBuffer& scopeBuffer() noexcept { return buses_.scopeBuffer; }

    // Atomic LED & peak telemetry (Core 0 reads without locking)
    const EngineTelemetry& telemetry() const noexcept { return buses_.telemetry; }

    // Toggle internal analog drift / RC roof tilt (default: both enabled).
    void setAnalogCharacterEnabled(bool enableDrift, bool enableRcTilt) noexcept {
        core_.setAnalogCharacterEnabled(enableDrift, enableRcTilt);
    }

    // Snap all parameter smoothers immediately to the latest published control state.
    void snapParametersToLatest() noexcept { core_.snapParametersToLatest(buses_); }

    // Render `numFrames` stereo samples at baseSampleRate (64 frames per block on FM-1).
    void processBlock(float* outLeft, float* outRight, size_t numFrames) noexcept {
        core_.processBlock(buses_, outLeft, outRight, numFrames);
    }

  private:
    EngineSharedBuses buses_{};
    DroneEngineCore core_{};
};

} // namespace ruido
