#pragma once

#include "core/Config.h"
#include "core/ParameterSmoother.h"
#include "core/Parameters.h"
#include "dsp/HalfBandDecimator.h"
#include "dsp/Ms20Filter.h"
#include "dsp/SchmittOscillatorBank.h"
#include "dsp/VoiceLfoBank.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Complete Core 1 portable synthesis & filtering engine:
//  - Unified 4x oversampled (176.472 kHz) voice, rail-coupling, MS-20 ZDF filter & master bus
//  - Clickless per-sample parameter & vactrol mute/pressure slewing
//  - Lock-free Core 0 -> Core 1 control consumption & Core 1 -> Core 0 scope/LED telemetry
class DroneEngine {
  public:
    DroneEngine() noexcept;

    void init(float baseSampleRate = kBaseSampleRate) noexcept;
    void reset() noexcept;

    // Control bus (Core 0 calls controlBus().publish(state))
    LockFreeControlBus& controlBus() noexcept { return controlBus_; }

    // Scope SPSC ring buffer (Core 0 calls scopeBuffer().popBatch(...))
    ScopeRingBuffer& scopeBuffer() noexcept { return scopeBuffer_; }

    // Atomic LED & peak telemetry (Core 0 reads without locking)
    const EngineTelemetry& telemetry() const noexcept { return telemetry_; }

    // Toggle internal analog drift / RC roof tilt (default: both enabled).
    void setAnalogCharacterEnabled(bool enableDrift, bool enableRcTilt) noexcept {
        oscBank_.setAnalogCharacterEnabled(enableDrift, enableRcTilt);
        lfoBank_.setAnalogCharacterEnabled(enableDrift);
    }

    // Snap all parameter smoothers immediately to the latest published control state
    // (useful on boot or inside deterministic unit tests).
    void snapParametersToLatest() noexcept;

    // Render `numFrames` stereo samples at baseSampleRate (64 frames per block on FM-1).
    void processBlock(float* outLeft, float* outRight, size_t numFrames) noexcept;

  private:
    struct VoiceSmoothers {
        LogFreqSmoother freqHz;
        CriticallyDampedSmoother pulseWidth;
        CriticallyDampedSmoother drift;
        LogFreqSmoother hpCutoffHz;
        CriticallyDampedSmoother hpResonance;
        LogFreqSmoother lpCutoffHz;
        CriticallyDampedSmoother lpResonance;
        CriticallyDampedSmoother amplitude;
        CriticallyDampedSmoother pan;
        OnePoleSmoother vactrolGate;
        VactrolSmoother pressure;
        OnePoleSmoother keyGainMul;
        LogFreqSmoother keyPitchMul;
        OnePoleSmoother stutterDepth;
        OnePoleSmoother stutterGate;
        uint32_t stutterPhase = 0x80000000u;
        float stutterRateHz = 6.0f;
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

    float baseSampleRate_ = kBaseSampleRate;
    float oversampledRate_ = kOversampledRate;
    float invOversampledRate_ = kInvOversampledRate;
    float hzToBasePhaseU32_ = 4294967296.0f / kBaseSampleRate;

    LockFreeControlBus controlBus_{};
    ScopeRingBuffer scopeBuffer_{};
    EngineTelemetry telemetry_{};

    SchmittOscillatorBank oscBank_{};
    VoiceLfoBank lfoBank_{};
    std::array<Ms20DualFilter, kNumOscillators> filters_{};
    std::array<Ms20DualFilter::Coeffs, kNumOscillators> filterCoeffs_{};
    StereoHalfBandDecimator4x decimator_{};

    std::array<VoiceSmoothers, kNumOscillators> voiceSmoothers_{};
    std::array<VoiceCoeffCache, kNumOscillators> coeffCache_{};
    std::array<float, kNumOscillators> prevOscFreqHz_{55.0f, 82.41f, 110.15f, 164.81f};
    CriticallyDampedSmoother couplingSmoother_{};
    CriticallyDampedSmoother masterTrimSmoother_{};
    OnePoleSmoother monoMixSmoother_{};   // 0.0 = Stereo (POLY), 1.0 = Dual-Mono (MONO)
    OnePoleSmoother masterRunSmoother_{}; // 0.0 = Muted (STOP), 1.0 = Active (PLAY)
    OnePoleSmoother pitchModeSmoother_{}; // 0.0 = MS-20 Filter/LPG, 1.0 = Pitch Bend
    OnePoleSmoother overloadSurgeSmoother_{};
    OnePoleSmoother dubLpfDropSmoother_{};
    OnePoleSmoother spatialFocusSmoother_{};
    bool stasisFreeze_ = false;

    // 36-frame delay aligning 4-voice rising edges & scope taps with PolyBLEP + FIR delay
    std::array<uint64_t, kNumOscillators> voiceEdgeDelayLines_{0u, 0u, 0u, 0u};
    std::array<std::array<int16_t, kNumOscillators>, 64> voiceScopeDelay_{};
    uint32_t voiceScopeDelayWrite_ = 0u;
};

} // namespace ruido
