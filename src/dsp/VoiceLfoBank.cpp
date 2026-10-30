#include "dsp/VoiceLfoBank.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kLfoPotPoleTauSec = 0.0140f;      // 2-pole C^1 pot glide (~28 ms, 0% overshoot)
constexpr float kTargetCrossfadeTauSec = 0.0042f; // ~12.5 ms clickless target crossfade
constexpr float kInv24Bit = 1.0f / 16777216.0f;

// 1% polystyrene timing capacitor tolerances per voice (deterministic 1.0 when drift disabled)
constexpr std::array<float, kNumOscillators> kCapTolerance = {
    1.0000f,
    1.0063f,
    0.9943f,
    1.0109f,
};

constexpr std::array<uint32_t, kNumOscillators> kInitPhases = {
    0x00000000u,
    0x40000000u,
    0x80000000u,
    0xC0000000u,
};

constexpr std::array<uint32_t, kNumOscillators> kInitSeeds = {
    0x9E3779B9u,
    0x85EBCA6Bu,
    0xC2B2AE35u,
    0x27D4EB2Fu,
};

} // namespace

VoiceLfoBank::VoiceLfoBank() noexcept {
    reset(kBaseSampleRate);
}

float VoiceLfoBank::nextUniformBipolar(uint32_t& rng) noexcept {
    uint32_t x = (rng != 0u) ? rng : 0x6D2B79F5u;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    rng = x;
    const float u01 = static_cast<float>(x >> 8u) * kInv24Bit;
    return u01 * 2.0f - 1.0f;
}

int8_t VoiceLfoBank::nextDriftKnot(VoiceState& v) noexcept {
    const float u1 = nextUniformBipolar(v.rngState);
    const float u2 = nextUniformBipolar(v.rngState);
    const float tri = 0.70f * u1 + 0.30f * u2;

    // Sloth-inspired anti-clump + DC-centering feedback for wide, organic zero-mean excursions
    const float driven =
        clampf((tri - 0.28f * v.prevKnotNorm - 0.24f * v.knotMean) * 1.18f, -1.0f, 1.0f);
    v.prevKnotNorm = driven;
    v.knotMean = sanitizeState(v.knotMean + 0.18f * (driven - v.knotMean));

    const float scaled = driven * 31.0f;
    int32_t q = static_cast<int32_t>(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    if (q < -31) {
        q = -31;
    } else if (q > 31) {
        q = 31;
    }
    return static_cast<int8_t>(q);
}

void VoiceLfoBank::reset(float sampleRate) noexcept {
    sampleRate_ = (sampleRate > 8000.0f) ? sampleRate : kBaseSampleRate;
    hzToPhaseU32_ = 4294967296.0f / sampleRate_;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        v.phase = kInitPhases[i];
        v.rngState = kInitSeeds[i];
        v.thermalRng = kInitSeeds[i] ^ 0xA5A55A5Au;
        v.prevKnotNorm = 0.0f;
        v.knotMean = 0.0f;
        for (size_t k = 0u; k < 5u; ++k) {
            v.knots[k] = nextDriftKnot(v);
        }
        v.packedKnots = packDriftKnots(v.knots);

        v.thermalState = 0.0f;
        v.thermalCounter = static_cast<uint32_t>(i * 64u);
        v.cachedRateScale = enableDrift_ ? kCapTolerance[i] : 1.0f;

        v.rateHz.init(0.25f, kLfoPotPoleTauSec, sampleRate_);
        v.shape.init(0.0f, kLfoPotPoleTauSec, sampleRate_);
        v.depth.init(0.0f, kLfoPotPoleTauSec, sampleRate_);

        v.activeTarget = static_cast<uint8_t>(LfoTarget::LpCutoff);
        v.crossfading = false;
        for (size_t t = 0u; t < kNumTargets; ++t) {
            const float initWeight = (t == v.activeTarget) ? 1.0f : 0.0f;
            v.targetWeights[t].init(initWeight, kTargetCrossfadeTauSec, sampleRate_);
        }
    }
}

void VoiceLfoBank::updateTargets(const EngineControlState& state, bool snapImmediate) noexcept {
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = state.voices[i];
        VoiceState& v = voices_[i];

        const float rate = clampf(vp.lfoRateHz, kLfoFreqMinHz, kLfoFreqMaxHz);
        const float shape = clampf(vp.lfoShape, 0.0f, 1.0f);
        const float depth = clampf(vp.lfoDepth, 0.0f, 1.0f);
        uint8_t targetIdx = static_cast<uint8_t>(vp.lfoTarget);
        if (targetIdx >= kNumTargets) {
            targetIdx = 0u;
        }

        if (snapImmediate) {
            v.rateHz.snapTo(rate);
            v.shape.snapTo(shape);
            v.depth.snapTo(depth);
            v.activeTarget = targetIdx;
            v.crossfading = false;
            for (size_t t = 0u; t < kNumTargets; ++t) {
                v.targetWeights[t].snapTo((t == targetIdx) ? 1.0f : 0.0f);
            }
        } else {
            v.rateHz.setTarget(rate);
            v.shape.setTarget(shape);
            v.depth.setTarget(depth);
            if (targetIdx != v.activeTarget) {
                v.activeTarget = targetIdx;
                v.crossfading = true;
                for (size_t t = 0u; t < kNumTargets; ++t) {
                    v.targetWeights[t].setTarget((t == targetIdx) ? 1.0f : 0.0f);
                }
            }
        }
    }
}

std::array<VoiceLfoFrame, kNumOscillators> VoiceLfoBank::stepFrame() noexcept {
    std::array<VoiceLfoFrame, kNumOscillators> frames{};

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        VoiceLfoFrame& out = frames[i];

        const float curRate = v.rateHz.next();
        const float curShape = v.shape.next();
        const float curDepth = v.depth.next();

        if (enableDrift_) {
            if ((v.thermalCounter++ & 255u) == 0u) {
                const float n = nextUniformBipolar(v.thermalRng);
                v.thermalState = sanitizeState(v.thermalState + 0.002f * (n - v.thermalState));
                v.cachedRateScale = kCapTolerance[i] * (1.0f + 0.0025f * v.thermalState);
            }
        } else {
            v.cachedRateScale = 1.0f;
        }

        const float phase01 = static_cast<float>(v.phase >> 8u) * kInv24Bit;
        out.phase01 = phase01;
        out.driftPacked = v.packedKnots;
        out.raw = evalMorphedLfoWave(phase01, curShape, curRate, v.knots);

        if (v.crossfading) {
            bool allSettled = true;
            for (size_t t = 0u; t < kNumTargets; ++t) {
                const float w = v.targetWeights[t].next();
                out.targetDepths[t] = curDepth * w;
                if (w != v.targetWeights[t].target()) {
                    allSettled = false;
                }
            }
            if (allSettled) {
                v.crossfading = false;
            }
        } else {
            out.targetDepths[v.activeTarget] = curDepth;
        }

        const float effHz = clampf(curRate * v.cachedRateScale, 0.005f, kLfoFreqMaxHz * 1.1f);
        const uint32_t inc = static_cast<uint32_t>(effHz * hzToPhaseU32_ + 0.5f);
        const uint32_t nextPhase = v.phase + inc;
        if (nextPhase < v.phase) {
            v.knots[0] = v.knots[2];
            v.knots[1] = v.knots[3];
            v.knots[2] = v.knots[4];
            v.knots[3] = nextDriftKnot(v);
            v.knots[4] = nextDriftKnot(v);
            v.packedKnots = packDriftKnots(v.knots);
        }
        v.phase = nextPhase;
    }

    return frames;
}

void VoiceLfoBank::writeInitialTelemetry(EngineTelemetry& telemetry) const noexcept {
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceState& v = voices_[i];
        const float phase01 = static_cast<float>(v.phase >> 8u) * kInv24Bit;
        const float raw =
            evalMorphedLfoWave(phase01, v.shape.current(), v.rateHz.current(), v.knots);
        telemetry.lfoOutput[i].store(raw, std::memory_order_relaxed);
        telemetry.lfoPhase[i].store(phase01, std::memory_order_relaxed);
        telemetry.lfoDriftPacked[i].store(v.packedKnots, std::memory_order_relaxed);
    }
}

} // namespace ruido
