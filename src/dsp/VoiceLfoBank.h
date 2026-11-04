#pragma once

#include "core/Config.h"
#include "core/ParameterSmoother.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

struct VoiceLfoFrame {
    float raw = 0.0f;     // [-1.0, +1.0] bipolar LFO wave output
    float phase01 = 0.0f; // [0.0, 1.0) instantaneous phase
    uint32_t driftPacked = 0u;
    // Per-target effective depths in [0.0, 1.0] (clicklessly crossfaded on live target changes)
    std::array<float, static_cast<size_t>(LfoTarget::Count)> targetDepths{};
};

// 8 independent analog-modeled voice LFOs (Osc 1..4 x LFO A + LFO B) at baseSampleRate (44.1 kHz):
//  - Slots [0..3] = Osc 1..4 LFO A; Slots [4..7] = Osc 1..4 LFO B (stepped first for 0-delay B->A)
//  - 32-bit unsigned integer phase accumulators (~10.3 uHz resolution, zero float32 stall)
//  - Stepless waveform morph: Sine -> Triangle -> Sawtooth -> Slewed Square -> C^2 Smooth Drift
//  - Phase-clocked 5-knot Uniform Cubic B-Spline drift with Sloth anti-clump zero-mean feedback
//  - Clickless 12 ms destination crossfade weights (incl. LFO B -> LFO A Rate & Depth cross-mod)
//  - Subtle polystyrene timing-capacitor tolerance & slow thermal drift per LFO
class VoiceLfoBank {
  public:
    static constexpr size_t kNumTargets = static_cast<size_t>(LfoTarget::Count);

    VoiceLfoBank() noexcept;

    void reset(float sampleRate = kBaseSampleRate) noexcept;

    void setAnalogCharacterEnabled(bool enableDrift) noexcept { enableDrift_ = enableDrift; }

    void updateTargets(const EngineControlState& state, bool snapImmediate) noexcept;

    std::array<VoiceLfoFrame, kNumVoiceLfos> stepFrame(bool freeze = false) noexcept;

    void writeInitialTelemetry(EngineTelemetry& telemetry) const noexcept;

  private:
    struct VoiceState {
        uint32_t phase = 0u;
        uint32_t rngState = 1u;
        uint32_t thermalRng = 1u;
        float prevKnotNorm = 0.0f;
        float knotMean = 0.0f;
        std::array<int8_t, 5> knots{0, 0, 0, 0, 0};
        uint32_t packedKnots = 0u;

        // Slow analog timing-capacitor thermal drift
        float thermalState = 0.0f;
        uint32_t thermalCounter = 0u;
        float cachedRateScale = 1.0f;

        LogFreqSmoother rateHz{};
        CriticallyDampedSmoother shape{};
        CriticallyDampedSmoother depth{};
        std::array<OnePoleSmoother, kNumTargets> targetWeights{};
        uint8_t activeTarget = 0u;
        bool crossfading = false;
    };

    static float nextUniformBipolar(uint32_t& rng) noexcept;
    static int8_t nextDriftKnot(VoiceState& v) noexcept;
    void stepSlot(size_t idx, float rateModDepth, float depthModDepth, float modBipolar,
                  bool freeze, VoiceLfoFrame& out) noexcept;

    std::array<VoiceState, kNumVoiceLfos> voices_{};
    float sampleRate_ = kBaseSampleRate;
    float hzToPhaseU32_ = 4294967296.0f / kBaseSampleRate;
    bool enableDrift_ = true;
};

} // namespace ruido
