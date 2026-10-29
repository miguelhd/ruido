#pragma once

#include "core/Config.h"
#include "core/ParameterSmoother.h"
#include "core/Parameters.h"
#include "dsp/HalfBandDecimator.h"
#include "dsp/Ms20Filter.h"
#include "dsp/SchmittOscillatorBank.h"

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
    LockFreeControlBus& controlBus() noexcept {
        return controlBus_;
    }

    // Scope SPSC ring buffer (Core 0 calls scopeBuffer().popBatch(...))
    ScopeRingBuffer& scopeBuffer() noexcept {
        return scopeBuffer_;
    }

    // Atomic LED & peak telemetry (Core 0 reads without locking)
    const EngineTelemetry& telemetry() const noexcept {
        return telemetry_;
    }

    // Toggle internal analog drift / RC roof tilt (default: both enabled).
    void setAnalogCharacterEnabled(bool enableDrift, bool enableRcTilt) noexcept {
        oscBank_.setAnalogCharacterEnabled(enableDrift, enableRcTilt);
    }

    // Snap all parameter smoothers immediately to the latest published control state
    // (useful on boot or inside deterministic unit tests).
    void snapParametersToLatest() noexcept;

    // Render `numFrames` stereo samples at baseSampleRate (64 frames per block on FM-1).
    void processBlock(float* outLeft, float* outRight, size_t numFrames) noexcept;

private:
    struct VoiceSmoothers {
        OnePoleSmoother freqHz;
        OnePoleSmoother amplitude;
        OnePoleSmoother pan;
        OnePoleSmoother hpCutoffHz;
        OnePoleSmoother lpCutoffHz;
        OnePoleSmoother resonance;
        OnePoleSmoother drive;
        OnePoleSmoother vactrolGate;
        OnePoleSmoother pressure;
    };

    struct VoiceCoeffCache {
        float hpHz = -1.0f;
        float lpHz = -1.0f;
        float reso = -1.0f;
        float drv = -1.0f;
        Ms20DualFilter::Coeffs filter{};
        float panEff = -99.0f;
        float panL = 0.70710678f;
        float panR = 0.70710678f;
    };

    void updateTargetsFromState(const EngineControlState& state, bool snapImmediate) noexcept;

    float baseSampleRate_ = kBaseSampleRate;
    float oversampledRate_ = kOversampledRate;
    float invOversampledRate_ = kInvOversampledRate;

    LockFreeControlBus controlBus_{};
    ScopeRingBuffer scopeBuffer_{};
    EngineTelemetry telemetry_{};

    SchmittOscillatorBank oscBank_{};
    std::array<Ms20DualFilter, kNumOscillators> filters_{};
    StereoHalfBandDecimator4x decimator_{};

    std::array<VoiceSmoothers, kNumOscillators> voiceSmoothers_{};
    std::array<VoiceCoeffCache, kNumOscillators> coeffCache_{};
    OnePoleSmoother couplingSmoother_{};
    OnePoleSmoother spreadSmoother_{};
    OnePoleSmoother masterTrimSmoother_{};
    OnePoleSmoother monoMixSmoother_{};   // 0.0 = Stereo (POLY), 1.0 = Dual-Mono (MONO)
    OnePoleSmoother masterRunSmoother_{}; // 0.0 = Muted (STOP), 1.0 = Active (PLAY)
    OnePoleSmoother pitchModeSmoother_{}; // 0.0 = Filter Pressure, 1.0 = Pitch Pressure (PIT)

    // 36-frame delay aligning Osc-1 scope trigger with PolyBLEP + FIR group delay
    uint64_t osc1EdgeDelayLine_ = 0u;
};

} // namespace ruido
