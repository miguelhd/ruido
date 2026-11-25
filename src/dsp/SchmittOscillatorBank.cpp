#include "dsp/SchmittOscillatorBank.h"

#include "core/DspMath.h"

namespace ruido {

SchmittOscillatorBank::SchmittOscillatorBank() noexcept {
    reset(kOversampledRate);
}

void SchmittOscillatorBank::reset(float sampleRate) noexcept {
    sampleRate_ = (sampleRate > 1000.0f) ? sampleRate : kOversampledRate;
    hzToPhaseU32_ = 4294967296.0f / sampleRate_;

    // 1.5 Hz AC coupling / capacitor roof-tilt corner:
    // preserves 99.5% weight on 15 Hz sub-bass while sculpting < 2 Hz into sharp relaxation pulses.
    rcHighpassPole_ = fastExp(-kTwoPi * 1.5f / sampleRate_);
    // ~0.45 ms shared Vdd rail decoupling recovery time constant
    railDecayCoeff_ = fastExp(-1.0f / (0.00045f * sampleRate_));
    const float rd2 = railDecayCoeff_ * railDecayCoeff_;
    railDecayCoeff4x_ = rd2 * rd2;
    prevRisingMask_ = 0u;

    for (size_t i = 0; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        v.phase = kInitialPhases[i];
        v.currentLevel = (v.phase < kHalfPhase) ? 1.0f : -1.0f;
        v.naiveD1 = v.currentLevel;
        v.naiveD2 = v.currentLevel;
        v.blepResidual = {0.0f, 0.0f, 0.0f, 0.0f};
        v.rcCapState = 0.0f;
        v.rcPrevInput = v.currentLevel;
        v.rngState = kInitialRngSeeds[i];
        v.driftFast = 0.0f;
        v.driftSlow = 0.0f;
        v.driftCounter = static_cast<uint32_t>(i * 16u);
        v.cachedDriftRatio = 1.0f;
        v.cachedDriftNorm = 0.0f;
        v.lastEdgePulse = 0.0f;
        v.cachedPw = -1.0f;
        v.cachedMinSafeDuty = -1.0f;
        v.cachedHalfInvHigh = 1.0f;
        v.cachedHalfInvLow = 1.0f;
        v.blepPending = 0u;
        voiceRailSag_[i] = 0.0f;
    }
}

void SchmittOscillatorBank::addCubicBsplineBlep(VoiceState& v, float d, float delta) noexcept {
    auto& res = v.blepResidual;
    v.blepPending = 4u;
    const float dc = clampf(d, 0.0f, 1.0f);
    const float d2 = dc * dc;
    const float u1 = 1.0f - dc;
    const float u1_2 = u1 * u1;

    // 4-point integrated 3rd-order B-spline step residual at [n-2, n-1, n, n+1]
    const float rM2 = (d2 * d2) * (1.0f / 24.0f);
    const float x = dc - 1.0f;
    const float x2 = x * x;
    const float rM1 = 0.5f + x * (0.66666667f - x2 * 0.33333333f - (x2 * x) * 0.125f);
    const float r0 = (0.5f + dc * (0.66666667f - d2 * 0.33333333f + (d2 * dc) * 0.125f)) - 1.0f;
    const float rP1 = -(u1_2 * u1_2) * (1.0f / 24.0f);

    res[0] += delta * rM2;
    res[1] += delta * rM1;
    res[2] += delta * r0;
    res[3] += delta * rP1;
}

RUIDO_NOINLINE void SchmittOscillatorBank::applyMasterHardSync(VoiceState& v, uint32_t incHighU32,
                                                               uint32_t incLowU32,
                                                               float osc0WrapD) noexcept {
    const float preFrac = 1.0f - osc0WrapD;
    if (v.currentLevel < 0.0f) {
        const uint32_t remToWrap = 0u - v.phase;
        const float preLow = preFrac * static_cast<float>(incLowU32);
        if (static_cast<float>(remToWrap) < preLow && incLowU32 > 0u) {
            const float dNat =
                1.0f - static_cast<float>(remToWrap) * fastRecipPos(static_cast<float>(incLowU32));
            addCubicBsplineBlep(v, dNat, 2.0f);
        } else {
            addCubicBsplineBlep(v, osc0WrapD, 2.0f);
        }
    } else {
        const uint32_t remToHalf = kHalfPhase - v.phase;
        const float preHigh = preFrac * static_cast<float>(incHighU32);
        if (static_cast<float>(remToHalf) < preHigh && incHighU32 > 0u) {
            const float dFall =
                1.0f - static_cast<float>(remToHalf) * fastRecipPos(static_cast<float>(incHighU32));
            addCubicBsplineBlep(v, dFall, -2.0f);
            addCubicBsplineBlep(v, osc0WrapD, 2.0f);
        }
    }
    v.lastEdgePulse += 0.22f;
    uint32_t postHigh = static_cast<uint32_t>(osc0WrapD * static_cast<float>(incHighU32) + 0.5f);
    if (postHigh >= kHalfPhase) {
        postHigh = kHalfPhase - 1u;
    }
    v.phase = postHigh;
    v.currentLevel = 1.0f;
}

float SchmittOscillatorBank::nextWhiteNoise(uint32_t& state) noexcept {
    uint32_t x = (state != 0u) ? state : 0x6D2B79F5u;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    state = x;
    return static_cast<float>(static_cast<int32_t>(x)) * 4.65661287e-10f;
}

void SchmittOscillatorBank::stepVoiceThermalDrift(VoiceState& v, float dAmt,
                                                  bool enableDrift) noexcept {
    if (!enableDrift) {
        v.cachedDriftRatio = 1.0f;
        v.cachedDriftNorm = 0.0f;
        return;
    }
    const float wn = nextWhiteNoise(v.rngState);
    v.driftFast = sanitizeState(0.996f * v.driftFast + 0.004f * wn);
    v.driftSlow = sanitizeState(0.9997f * v.driftSlow + 0.0003f * v.driftFast);
    if (dAmt > 1.0e-5f) {
        const float wanderNorm =
            clampf(0.35f * (v.driftFast * 12.5f) + 0.65f * (v.driftSlow * 45.0f), -1.0f, 1.0f);
        const float maxDriftOct = dAmt * (0.018f + 0.082f * dAmt * dAmt);
        v.cachedDriftRatio = fastExp2(wanderNorm * maxDriftOct);
        v.cachedDriftNorm = wanderNorm * dAmt;
    } else {
        v.cachedDriftRatio = 1.0f;
        v.cachedDriftNorm = 0.0f;
    }
}

void SchmittOscillatorBank::processSample(
    OscillatorBankOutput& __restrict out,
    const std::array<float, kNumOscillators>& __restrict freqHz,
    const std::array<float, kNumOscillators>& __restrict pulseWidth,
    const std::array<float, kNumOscillators>& __restrict drift, float circuitCoupling,
    bool freezeDrift, float hardSync) noexcept {
    out.risingMask = 0u;
    const CouplingParams cp = computeCouplingParams(circuitCoupling);
    const float syncAmt = clampf(hardSync, 0.0f, 1.0f);
    const uint32_t hardSyncThreshold = computeHardSyncThreshold(syncAmt);
    bool osc0Wrapped = false;
    float osc0WrapD = 0.0f;
    const uint8_t lastRising = prevRisingMask_;

    float totalRailSag = 0.0f;
    float totalGateLoad = 0.0f;
    for (size_t i = 0; i < kNumOscillators; ++i) {
        voiceRailSag_[i] =
            sanitizeState(voiceRailSag_[i] * railDecayCoeff_ + voices_[i].lastEdgePulse);
        voices_[i].lastEdgePulse = 0.0f;
        totalRailSag += voiceRailSag_[i];
        totalGateLoad += voices_[i].currentLevel;
    }

    for (size_t i = 0; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        if (!freezeDrift && ((v.driftCounter++ & 63u) == 0u)) {
            stepVoiceThermalDrift(v, clampf(drift[i], 0.0f, 1.0f), enableDrift_);
        }

        const float railPullRatio = computeVoiceRailPull<true>(
            v.phase, v.currentLevel, totalRailSag - voiceRailSag_[i], totalGateLoad, cp);
        const float targetHz =
            clampf(freqHz[i] * v.cachedDriftRatio * railPullRatio, 0.005f, 21200.0f);
        const float baseIncFloat = targetHz * hzToPhaseU32_;

        const float minSafeDuty =
            clampf(baseIncFloat * (0.5f / static_cast<float>(kMaxPhaseInc)), kPulseWidthMin, 0.49f);
        if (pulseWidth[i] != v.cachedPw || minSafeDuty != v.cachedMinSafeDuty) {
            v.cachedPw = pulseWidth[i];
            v.cachedMinSafeDuty = minSafeDuty;
            const float pwClamped = clampf(pulseWidth[i], kPulseWidthMin, kPulseWidthMax);
            const float trimWeight = clampf(1.0f - absf(2.0f * pwClamped - 1.0f), 0.0f, 1.0f);
            const float effDuty = clampf(pwClamped + kInverterDutyTrim[i] * trimWeight, minSafeDuty,
                                         1.0f - minSafeDuty);
            v.cachedHalfInvHigh = 0.5f * fastRecipPos(effDuty);
            v.cachedHalfInvLow = 0.5f * fastRecipPos(1.0f - effDuty);
        }

        const uint32_t incHighU32 = static_cast<uint32_t>(static_cast<int32_t>(
            clampf(baseIncFloat * v.cachedHalfInvHigh, 1.0f, kMaxPhaseIncFloat)));
        const uint32_t incLowU32 = static_cast<uint32_t>(static_cast<int32_t>(
            clampf(baseIncFloat * v.cachedHalfInvLow, 1.0f, kMaxPhaseIncFloat)));

        const uint8_t neighborRose =
            static_cast<uint8_t>((lastRising | out.risingMask) & static_cast<uint8_t>(~(1u << i)));
        if (i > 0u && osc0Wrapped && syncAmt > 1.0e-4f && v.phase >= hardSyncThreshold) {
            applyMasterHardSync(v, incHighU32, incLowU32, osc0WrapD);
            out.risingMask |= static_cast<uint8_t>(1u << i);
        } else if (cp.chaos > 0.02f && neighborRose != 0u && v.currentLevel < 0.0f &&
                   v.phase >= cp.syncCapturePhase) {
            v.phase = (incHighU32 >> 1u);
            addCubicBsplineBlep(v, 0.5f, 2.0f);
            v.lastEdgePulse += 0.22f;
            v.currentLevel = 1.0f;
            out.risingMask |= static_cast<uint8_t>(1u << i);
            if (i == 0u) {
                osc0Wrapped = true;
                osc0WrapD = 0.5f;
            }
        } else if (v.phase < kHalfPhase) {
            const uint32_t remToHalf = kHalfPhase - v.phase;
            if (incHighU32 < remToHalf) {
                v.phase += incHighU32;
            } else {
                v.phase = fastFallingEdge(v, incHighU32 - remToHalf, incHighU32, incLowU32);
                v.currentLevel = -1.0f;
            }
        } else {
            const uint32_t remToWrap = 0u - v.phase;
            if (incLowU32 < remToWrap) {
                v.phase += incLowU32;
            } else {
                const uint32_t overLow = incLowU32 - remToWrap;
                if (i == 0u) {
                    osc0Wrapped = true;
                    osc0WrapD =
                        static_cast<float>(overLow) * fastRecipPos(static_cast<float>(incLowU32));
                }
                v.phase = fastRisingEdge(v, overLow, incLowU32, incHighU32);
                v.currentLevel = 1.0f;
                out.risingMask |= static_cast<uint8_t>(1u << i);
            }
        }

        float bandLimitedSquare = v.naiveD2;
        if (v.blepPending != 0u) {
            bandLimitedSquare += v.blepResidual[0];
            v.blepResidual[0] = v.blepResidual[1];
            v.blepResidual[1] = v.blepResidual[2];
            v.blepResidual[2] = v.blepResidual[3];
            v.blepResidual[3] = 0.0f;
            --v.blepPending;
        }
        v.naiveD2 = v.naiveD1;
        v.naiveD1 = v.currentLevel;

        float voiceSample = bandLimitedSquare;
        if (enableRcTilt_) {
            const float rcOut =
                rcHighpassPole_ * (v.rcCapState + bandLimitedSquare - v.rcPrevInput);
            v.rcPrevInput = bandLimitedSquare;
            v.rcCapState = sanitizeState(rcOut);
            voiceSample = v.rcCapState;
        }

        out.samples[i] = voiceSample;
        out.capRamp[i] = phaseToCapRamp(v.phase);
        out.gateHigh[i] = (v.currentLevel > 0.0f) ? 1.0f : 0.0f;
        out.driftNorm[i] = v.cachedDriftNorm;
    }

    prevRisingMask_ = out.risingMask;
}

RUIDO_NOINLINE uint32_t SchmittOscillatorBank::fastFallingEdge(VoiceState& v, uint32_t overHigh,
                                                               uint32_t incHigh,
                                                               uint32_t incLow) noexcept {
    const float d = static_cast<float>(overHigh) * fastRecipPos(static_cast<float>(incHigh));
    addCubicBsplineBlep(v, d, -2.0f);
    v.lastEdgePulse -= 0.16f;
    uint32_t postLow = static_cast<uint32_t>(d * static_cast<float>(incLow) + 0.5f);
    if (postLow >= kHalfPhase) {
        postLow = kHalfPhase - 1u;
    }
    return kHalfPhase + postLow;
}

RUIDO_NOINLINE uint32_t SchmittOscillatorBank::fastRisingEdge(VoiceState& v, uint32_t overLow,
                                                              uint32_t incLow,
                                                              uint32_t incHigh) noexcept {
    const float d = static_cast<float>(overLow) * fastRecipPos(static_cast<float>(incLow));
    addCubicBsplineBlep(v, d, 2.0f);
    v.lastEdgePulse += 0.22f;
    uint32_t postHigh = static_cast<uint32_t>(d * static_cast<float>(incHigh) + 0.5f);
    if (postHigh >= kHalfPhase) {
        postHigh = kHalfPhase - 1u;
    }
    return postHigh;
}

} // namespace ruido
