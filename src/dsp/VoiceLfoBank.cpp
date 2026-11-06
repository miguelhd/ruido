#include "dsp/VoiceLfoBank.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kLfoPotPoleTauSec = 0.0140f;      // 2-pole C^1 pot glide (~28 ms, 0% overshoot)
constexpr float kTargetCrossfadeTauSec = 0.0042f; // ~12.5 ms clickless target crossfade
constexpr float kInv24Bit = 1.0f / 16777216.0f;

// 1% polystyrene timing capacitor tolerances per LFO ([0..3]=LFO A, [4..7]=LFO B)
constexpr std::array<float, kNumVoiceLfos> kCapTolerance = {
    1.0000f, 1.0063f, 0.9943f, 1.0109f, 0.9918f, 1.0087f, 1.0031f, 0.9964f,
};

constexpr std::array<uint32_t, kNumVoiceLfos> kInitPhases = {
    0x00000000u, 0x40000000u, 0x80000000u, 0xC0000000u,
    0x80000000u, 0xC0000000u, 0x20000000u, 0x60000000u,
};

constexpr std::array<uint32_t, kNumVoiceLfos> kInitSeeds = {
    0x9E3779B9u, 0x85EBCA6Bu, 0xC2B2AE35u, 0x27D4EB2Fu,
    0x165667B1u, 0xD3A2646Cu, 0xFD7046C5u, 0xB55A4F09u,
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
    turboSmoother_.init(0.0f, kLfoPotPoleTauSec, sampleRate_);

    for (size_t i = 0u; i < kNumVoiceLfos; ++i) {
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
        v.thermalCounter = static_cast<uint32_t>(i * 32u);
        v.cachedRateScale = enableDrift_ ? kCapTolerance[i] : 1.0f;

        v.rateHz.init(0.25f, kLfoPotPoleTauSec, sampleRate_);
        v.shape.init(0.0f, kLfoPotPoleTauSec, sampleRate_);
        v.depth.init(0.0f, kLfoPotPoleTauSec, sampleRate_);

        v.activeTarget = static_cast<uint8_t>(LfoTarget::LpCutoff);
        v.crossfading = false;
        v.targetWeights.setTimeConstant(kTargetCrossfadeTauSec, sampleRate_);
        for (size_t t = 0u; t < kNumTargets; ++t) {
            const float initWeight = (t == v.activeTarget) ? 1.0f : 0.0f;
            v.targetWeights.snapChannel(t, initWeight);
        }
    }
}

void VoiceLfoBank::updateTargets(const EngineControlState& state, bool snapImmediate) noexcept {
    for (size_t sub = 0u; sub < kLfosPerVoice; ++sub) {
        const size_t maxAllowed = (sub == 0u) ? kNumLfoATargets : kNumLfoBTargets;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const VoiceParams& vp = state.voices[i];
            VoiceState& v = voices_[lfoBankIndex(i, sub)];

            const float rate = clampf(lfoSlotRateHz(vp, sub), kLfoFreqMinHz, kLfoFreqMaxHz);
            const float shape = clampf(lfoSlotShape(vp, sub), 0.0f, 1.0f);
            const float depth = clampf(lfoSlotDepth(vp, sub), 0.0f, 1.0f);
            uint8_t targetIdx = static_cast<uint8_t>(lfoSlotTarget(vp, sub));
            if (targetIdx >= maxAllowed) {
                targetIdx = static_cast<uint8_t>(LfoTarget::LpCutoff);
            }

            if (snapImmediate) {
                v.rateHz.snapTo(rate);
                v.shape.snapTo(shape);
                v.depth.snapTo(depth);
                v.activeTarget = targetIdx;
                v.crossfading = false;
                for (size_t t = 0u; t < kNumTargets; ++t) {
                    v.targetWeights.snapChannel(t, (t == targetIdx) ? 1.0f : 0.0f);
                }
            } else {
                v.rateHz.setTarget(rate);
                v.shape.setTarget(shape);
                v.depth.setTarget(depth);
                if (targetIdx != v.activeTarget) {
                    v.activeTarget = targetIdx;
                    v.crossfading = true;
                    for (size_t t = 0u; t < kNumTargets; ++t) {
                        v.targetWeights.setTarget(t, (t == targetIdx) ? 1.0f : 0.0f);
                    }
                }
            }
        }
    }
    const float turboTarget = clampf(state.keyMod.lfoTurbo, 0.0f, 1.0f);
    if (snapImmediate) {
        turboSmoother_.snapTo(turboTarget);
    } else {
        turboSmoother_.setTarget(turboTarget);
    }
}

bool VoiceLfoBank::hasFastModulation() const noexcept {
    if (turboSmoother_.current() > 1.0e-4f || turboSmoother_.target() > 1.0e-4f) {
        return true;
    }
    for (size_t i = 0u; i < kNumVoiceLfos; ++i) {
        const VoiceState& v = voices_[i];
        const bool active =
            (v.depth.current() > 0.0f) || (v.depth.target() > 0.0f) || v.crossfading;
        if (active && (v.rateHz.current() > 45.0f || v.rateHz.target() > 45.0f)) {
            return true;
        }
    }
    return false;
}

void VoiceLfoBank::stepSlot(size_t idx, uint32_t stride, float rateModDepth, float depthModDepth,
                            float modBipolar, float turbo, float turboMul, bool freeze,
                            bool forceWaveEval, VoiceLfoFrame& out) noexcept {
    VoiceState& v = voices_[idx];

    const float baseRate = v.rateHz.nextN(stride);
    const float curShape = v.shape.nextN(stride);
    const float baseDepth = v.depth.nextN(stride);

    const bool modInactive = (rateModDepth <= 0.0f && depthModDepth <= 0.0f && turbo <= 1.0e-4f);
    const float curRate =
        modInactive ? baseRate
                    : clampf(applyLfoModRateHz(baseRate, rateModDepth, modBipolar) * turboMul,
                             kLfoFreqMinHz, kLfoFreqMaxHz);
    float curDepth =
        modInactive ? baseDepth : applyLfoModDepth(baseDepth, depthModDepth, modBipolar);
    if (!modInactive && idx < kNumOscillators && turbo > 1.0e-4f &&
        voices_[4u + idx].depth.current() < 0.05f) {
        const float turboFloor = 0.42f * turbo;
        if (turboFloor > curDepth) {
            curDepth = turboFloor;
        }
    }

    if (enableDrift_) {
        if (!freeze && stride > 0u) {
            const uint32_t prevTc = v.thermalCounter;
            v.thermalCounter = prevTc + stride;
            if (((prevTc & 255u) == 0u) || (((prevTc & 255u) + stride) > 256u)) {
                const float n = nextUniformBipolar(v.thermalRng);
                v.thermalState = sanitizeState(v.thermalState + 0.002f * (n - v.thermalState));
                v.cachedRateScale = kCapTolerance[idx] * (1.0f + 0.0025f * v.thermalState);
            }
        }
    } else {
        v.cachedRateScale = 1.0f;
    }

    const float phase01 = static_cast<float>(v.phase >> 8u) * kInv24Bit;
    out.phase01 = phase01;
    out.driftPacked = v.packedKnots;
    const bool slotActive = (curDepth > 0.0f) || v.crossfading;
    out.activeDepth = slotActive ? (curDepth > 0.0f ? curDepth : 1.0e-6f) : 0.0f;
    out.raw = (forceWaveEval || slotActive)
                  ? evalMorphedLfoWave(phase01, curShape, curRate, v.knots)
                  : 0.0f;

    if (v.crossfading) {
        bool allSettled = true;
        for (size_t t = 0u; t < kNumTargets; ++t) {
            const float w = v.targetWeights.nextN(t, stride);
            out.targetDepths[t] = curDepth * w;
            if (w != v.targetWeights.target(t)) {
                allSettled = false;
            }
        }
        if (allSettled) {
            v.crossfading = false;
        }
    } else if (curDepth > 0.0f) {
        for (size_t t = 0u; t < kNumTargets; ++t) {
            out.targetDepths[t] = 0.0f;
        }
        out.targetDepths[v.activeTarget] = curDepth;
    }

    if (freeze || stride == 0u) {
        return;
    }
    const float effHz = clampf(curRate * v.cachedRateScale, 0.005f, kLfoFreqMaxHz * 1.1f);
    const uint32_t inc = static_cast<uint32_t>(effHz * hzToPhaseU32_ + 0.5f);
    const uint32_t nextPhase = v.phase + inc * stride;
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

void VoiceLfoBank::stepSubBlock(std::array<VoiceLfoFrame, kNumVoiceLfos>& frames, uint32_t stride,
                                bool freeze, bool forceWaveEval) noexcept {
    const float turbo = turboSmoother_.nextN(stride);
    const float turboMul = (turbo > 1.0e-4f) ? fastExp2(5.0f * turbo) : 1.0f;

    // 1. Step LFO B [4..7] first so LFO B -> LFO A Rate/Depth cross-modulation has zero delay
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        stepSlot(4u + i, stride, 0.0f, 0.0f, 0.0f, turbo, turboMul, freeze, forceWaveEval,
                 frames[4u + i]);
    }
    // 2. Step LFO A [0..3] with instantaneous LFO B Rate/Depth cross-modulation
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceLfoFrame& bOut = frames[4u + i];
        const float rMod = (bOut.activeDepth > 0.0f)
                               ? bOut.targetDepths[static_cast<size_t>(LfoTarget::LfoARate)]
                               : 0.0f;
        const float dMod = (bOut.activeDepth > 0.0f)
                               ? bOut.targetDepths[static_cast<size_t>(LfoTarget::LfoADepth)]
                               : 0.0f;
        stepSlot(i, stride, rMod, dMod, bOut.raw, turbo, turboMul, freeze, forceWaveEval,
                 frames[i]);
    }
}

std::array<VoiceLfoFrame, kNumVoiceLfos> VoiceLfoBank::stepFrame(bool freeze) noexcept {
    std::array<VoiceLfoFrame, kNumVoiceLfos> frames{};
    stepFrame(frames, freeze, true);
    return frames;
}

void VoiceLfoBank::writeInitialTelemetry(EngineTelemetry& telemetry) const noexcept {
    for (size_t i = 0u; i < kNumVoiceLfos; ++i) {
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
