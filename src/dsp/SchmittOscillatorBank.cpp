#include "dsp/SchmittOscillatorBank.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

// Subtle per-inverter CMOS hysteresis threshold duty offsets around 50% modeled after real
// CD40106 manufacturing spread (~49.2% to ~50.9%), yielding organic ~-35 dB 2nd/4th harmonics.
constexpr std::array<float, kNumOscillators> kInverterDutyTrim = {
    -0.0080f, // ~0.4920 at 50%
    +0.0070f, // ~0.5070 at 50%
    -0.0040f, // ~0.4960 at 50%
    +0.0090f, // ~0.5090 at 50%
};

constexpr std::array<uint32_t, kNumOscillators> kInitialRngSeeds = {0x13579BDFu, 0x2468ACE1u,
                                                                    0x9E3779B9u, 0x7F4A7C15u};
constexpr std::array<uint32_t, kNumOscillators> kInitialPhases = {0x00000000u, 0x3504F333u,
                                                                  0x6A09E667u, 0xBB67AE85u};

constexpr float kSignedPhaseToBipolar = 4.6566128730773926e-10f; // 1.0 / 2^31
constexpr float kHalfPhaseToBipolar = 9.31322575e-10f;           // 2.0 / 2^31
constexpr uint32_t kHalfPhase = 0x80000000u;
constexpr uint32_t kMaxPhaseInc = 0x1F000000u; // >= 4.13 ticks per half-cycle (< fs/8 safe BLEP)
constexpr float kMaxPhaseIncFloat = static_cast<float>(kMaxPhaseInc);

struct FastDroneTag {
    static constexpr bool kFast = true;
};
struct FullSyncTag {
    static constexpr bool kFast = false;
};

struct CouplingParams {
    float chaos;
    float couplingGain;
    float minPull;
    float maxPull;
    uint32_t syncCapturePhase;
};

inline CouplingParams computeCouplingParams(float circuitCoupling) noexcept {
    const float coupling = clampf(circuitCoupling, 0.0f, 1.0f);
    // Two-regime ALGORITHM coupling:
    //  - [0.00, 0.45]: smooth CD40106 Vdd rail sag & subtle organic chorus shimmer (25% default)
    //  - (0.45, 1.00]: starved-rail voltage collapse -> PWM duty warp, subharmonic sync locks,
    //                  and hard phase-lock tearing across the 4 inverters
    const float chaos = smoothstep01((coupling - 0.45f) * (1.0f / 0.55f));
    const float couplingGain = coupling * (0.10f + 0.65f * coupling) + 4.80f * chaos * chaos;
    const float minPull = 0.55f - 0.43f * chaos;
    const float maxPull = 1.75f + 2.75f * chaos;
    const uint32_t syncCapturePhase =
        0xFFFFFFFFu - static_cast<uint32_t>(chaos * 601295424.0f); // upper 14% Vt+ threshold window
    return {chaos, couplingGain, minPull, maxPull, syncCapturePhase};
}

uint32_t computeHardSyncThreshold(float hardSync) noexcept {
    const float syncShaped = smoothstep01(clampf(hardSync, 0.0f, 1.0f) * 1.45f);
    if (syncShaped <= 1.0e-4f) {
        return 0xFFFFFFFFu;
    }
    return (syncShaped >= 0.9999f) ? 0u
                                   : static_cast<uint32_t>((1.0f - syncShaped) * 4294967040.0f);
}

inline float phaseToCapRamp(uint32_t phase) noexcept {
    return (phase < kHalfPhase)
               ? (static_cast<float>(phase) * kHalfPhaseToBipolar - 1.0f)
               : (1.0f - static_cast<float>(phase - kHalfPhase) * kHalfPhaseToBipolar);
}

template <bool IncludeChaosLoad>
inline float computeVoiceRailPull(uint32_t phase, float currentLevel, float neighborSag,
                                  float totalGateLoad, const CouplingParams& cp) noexcept {
    if (cp.couplingGain <= 0.0f) {
        return 1.0f;
    }
    const float dHalf =
        absf(static_cast<float>(static_cast<int32_t>(phase ^ kHalfPhase)) * kSignedPhaseToBipolar);
    const float signedPrc = dHalf * dHalf * (3.0f - 2.0f * dHalf) * 2.0f - 1.0f;
    float pull = 1.0f - cp.couplingGain * neighborSag * signedPrc;
    if constexpr (IncludeChaosLoad) {
        if (cp.chaos > 0.0f) {
            const float neighborLoad = (totalGateLoad - currentLevel) * (1.0f / 3.0f);
            const float asymSign = (currentLevel > 0.0f) ? 0.36f : -0.36f;
            pull += cp.chaos * asymSign * neighborLoad;
        }
    }
    return clampf(pull, cp.minPull, cp.maxPull);
}

} // namespace

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

void SchmittOscillatorBank::applyMasterHardSync(VoiceState& v, uint32_t incHighU32,
                                                uint32_t incLowU32, float osc0WrapD) noexcept {
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
    // xorshift32 PRNG mapped to [-1.0f, +1.0f]
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
        // 0% = 0c locked, 10% default = ~+-2.2c analog warmth, 100% = ~+-120c drift
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

        const float pwClamped = clampf(pulseWidth[i], kPulseWidthMin, kPulseWidthMax);
        const float trimWeight = clampf(1.0f - absf(2.0f * pwClamped - 1.0f), 0.0f, 1.0f);
        const float minSafeDuty =
            clampf(baseIncFloat * (0.5f / static_cast<float>(kMaxPhaseInc)), kPulseWidthMin, 0.49f);
        const float effDuty =
            clampf(pwClamped + kInverterDutyTrim[i] * trimWeight, minSafeDuty, 1.0f - minSafeDuty);

        const float halfBaseInc = 0.5f * baseIncFloat;
        const uint32_t incHighU32 = static_cast<uint32_t>(
            static_cast<int32_t>(clampf(halfBaseInc / effDuty, 1.0f, kMaxPhaseIncFloat)));
        const uint32_t incLowU32 = static_cast<uint32_t>(
            static_cast<int32_t>(clampf(halfBaseInc / (1.0f - effDuty), 1.0f, kMaxPhaseIncFloat)));

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
                const uint32_t overHigh = incHighU32 - remToHalf;
                const float d =
                    static_cast<float>(overHigh) * fastRecipPos(static_cast<float>(incHighU32));
                addCubicBsplineBlep(v, d, -2.0f);
                v.lastEdgePulse -= 0.16f;
                uint32_t postLow = static_cast<uint32_t>(d * static_cast<float>(incLowU32) + 0.5f);
                if (postLow >= kHalfPhase) {
                    postLow = kHalfPhase - 1u;
                }
                v.phase = kHalfPhase + postLow;
                v.currentLevel = -1.0f;
            }
        } else {
            const uint32_t remToWrap = 0u - v.phase;
            if (incLowU32 < remToWrap) {
                v.phase += incLowU32;
            } else {
                const uint32_t overLow = incLowU32 - remToWrap;
                const float d =
                    static_cast<float>(overLow) * fastRecipPos(static_cast<float>(incLowU32));
                addCubicBsplineBlep(v, d, 2.0f);
                v.lastEdgePulse += 0.22f;
                out.risingMask |= static_cast<uint8_t>(1u << i);
                if (i == 0u) {
                    osc0Wrapped = true;
                    osc0WrapD = d;
                }
                uint32_t postHigh =
                    static_cast<uint32_t>(d * static_cast<float>(incHighU32) + 0.5f);
                if (postHigh >= kHalfPhase) {
                    postHigh = kHalfPhase - 1u;
                }
                v.phase = postHigh;
                v.currentLevel = 1.0f;
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

void SchmittOscillatorBank::processSpan4x(
    std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
    uint8_t* __restrict spanRising, std::array<uint8_t, kNumOscillators>& totalRisingCounts,
    size_t numFrames, const std::array<float, kNumOscillators>& __restrict startFreqHz,
    const std::array<float, kNumOscillators>& __restrict endFreqHz,
    const std::array<float, kNumOscillators>& __restrict startPw,
    const std::array<float, kNumOscillators>& __restrict endPw,
    const std::array<float, kNumOscillators>& __restrict drift, float circuitCoupling,
    bool freezeDrift, float hardSync, std::array<float, kNumOscillators>& outLastSamples,
    std::array<float, kNumOscillators>& outLastCapRamp,
    std::array<float, kNumOscillators>* optGateHigh,
    std::array<float, kNumOscillators>* optDriftNorm) noexcept {
    static_assert(kOversampleFactor == 4u, "processSpan4x expects 4x oversampling");
    totalRisingCounts = {0u, 0u, 0u, 0u};
    if (numFrames == 0u || numFrames > 16u) {
        return;
    }

    const float invSpan =
        (numFrames == 16u) ? 0.0625f
                           : ((numFrames == 8u) ? 0.125f : (1.0f / static_cast<float>(numFrames)));
    const float invSpan4x = 0.25f * invSpan;
    const CouplingParams cp = computeCouplingParams(circuitCoupling);
    const float syncAmt = clampf(hardSync, 0.0f, 1.0f);
    const bool syncActive = (syncAmt > 1.0e-4f);
    const uint32_t hardSyncThreshold = syncActive ? computeHardSyncThreshold(syncAmt) : 0xFFFFFFFFu;
    const float hzToPhase = hzToPhaseU32_;
    const float minBaseInc = 0.005f * hzToPhase;
    const float maxBaseInc = 21200.0f * hzToPhase;

    std::array<float, kNumOscillators> curFreq{};
    std::array<float, kNumOscillators> freqStep4x{};
    std::array<float, kNumOscillators> curPw{};
    std::array<float, kNumOscillators> pwStep{};
    std::array<float, kNumOscillators> maxRawHz{};
    std::array<float, kNumOscillators> curBaseScale{};
    std::array<float, kNumOscillators> halfInvHigh{};
    std::array<float, kNumOscillators> halfInvLow{};
    std::array<float, kNumOscillators> dAmtClamped{};
    bool anyPwSweep = false;
    bool safeFreqRange = true;

    const auto refreshPwm = [&](size_t i, VoiceState& v, float pw) noexcept {
        const float peakHz = clampf(maxRawHz[i] * v.cachedDriftRatio *
                                        ((cp.couplingGain > 0.0f) ? cp.maxPull : 1.0f),
                                    0.005f, 21200.0f);
        const float minSafeDuty =
            clampf((peakHz * hzToPhase) * (0.5f / static_cast<float>(kMaxPhaseInc)), kPulseWidthMin,
                   0.49f);
        if (pw != v.cachedPw || minSafeDuty != v.cachedMinSafeDuty) {
            v.cachedPw = pw;
            v.cachedMinSafeDuty = minSafeDuty;
            const float pwClamped = clampf(pw, kPulseWidthMin, kPulseWidthMax);
            const float trimWeight = clampf(1.0f - absf(2.0f * pwClamped - 1.0f), 0.0f, 1.0f);
            const float effDuty = clampf(pwClamped + kInverterDutyTrim[i] * trimWeight, minSafeDuty,
                                         1.0f - minSafeDuty);
            v.cachedHalfInvHigh = 0.5f * fastRecipPos(effDuty);
            v.cachedHalfInvLow = 0.5f * fastRecipPos(1.0f - effDuty);
        }
        halfInvHigh[i] = v.cachedHalfInvHigh;
        halfInvLow[i] = v.cachedHalfInvLow;
    };

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        const float sHz = startFreqHz[i];
        const float eHz = endFreqHz[i];
        curFreq[i] = sHz;
        freqStep4x[i] = (eHz - sHz) * invSpan4x;
        const float minHz = (eHz < sHz) ? eHz : sHz;
        maxRawHz[i] = (eHz > sHz) ? eHz : sHz;
        if (minHz < 0.05f || maxRawHz[i] > 8000.0f) {
            safeFreqRange = false;
        }
        curPw[i] = startPw[i];
        pwStep[i] = (endPw[i] - startPw[i]) * invSpan;
        if (pwStep[i] != 0.0f) {
            anyPwSweep = true;
        }
        dAmtClamped[i] = clampf(drift[i], 0.0f, 1.0f);
        curBaseScale[i] = v.cachedDriftRatio * hzToPhase;
        if (!anyPwSweep) {
            refreshPwm(i, v, startPw[i]);
        }
    }

    const auto stepFrameDriftAndPw = [&]() noexcept {
        if (!freezeDrift) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                VoiceState& v = voices_[i];
                const uint32_t prevDc = v.driftCounter;
                v.driftCounter = prevDc + 4u;
                if (((prevDc & 63u) == 0u) || (((prevDc & 63u) + 4u) > 64u)) {
                    stepVoiceThermalDrift(v, dAmtClamped[i], enableDrift_);
                    curBaseScale[i] = v.cachedDriftRatio * hzToPhase;
                }
            }
        }
        if (anyPwSweep) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                curPw[i] += pwStep[i];
                refreshPwm(i, voices_[i], curPw[i]);
            }
        }
    };

    const auto toPhaseIncU32 = [](float scaled) noexcept -> uint32_t {
        return static_cast<uint32_t>(static_cast<int32_t>(clampf(scaled, 1.0f, kMaxPhaseIncFloat)));
    };
    const auto toPhaseIncFast = [](float scaled) noexcept -> uint32_t {
        return static_cast<uint32_t>(static_cast<int32_t>(scaled));
    };

    const auto advanceNormalPhase = [&](auto modeTag, VoiceState& v, size_t i, float baseInc,
                                        float invH, float invL, uint8_t& subRising,
                                        bool* osc0WrapPtr, float* osc0WrapDPtr) noexcept {
        constexpr bool kFast = decltype(modeTag)::kFast;
        if (v.phase < kHalfPhase) {
            const uint32_t incHighU32 =
                kFast ? toPhaseIncFast(baseInc * invH) : toPhaseIncU32(baseInc * invH);
            const uint32_t remToHalf = kHalfPhase - v.phase;
            if (incHighU32 < remToHalf) {
                v.phase += incHighU32;
            } else {
                const uint32_t incLowU32 =
                    kFast ? toPhaseIncFast(baseInc * invL) : toPhaseIncU32(baseInc * invL);
                const uint32_t overHigh = incHighU32 - remToHalf;
                const float d =
                    static_cast<float>(overHigh) * fastRecipPos(static_cast<float>(incHighU32));
                addCubicBsplineBlep(v, d, -2.0f);
                v.lastEdgePulse -= 0.16f;
                uint32_t postLow = static_cast<uint32_t>(d * static_cast<float>(incLowU32) + 0.5f);
                if (postLow >= kHalfPhase) {
                    postLow = kHalfPhase - 1u;
                }
                v.phase = kHalfPhase + postLow;
                v.currentLevel = -1.0f;
            }
        } else {
            const uint32_t incLowU32 =
                kFast ? toPhaseIncFast(baseInc * invL) : toPhaseIncU32(baseInc * invL);
            const uint32_t remToWrap = 0u - v.phase;
            if (incLowU32 < remToWrap) {
                v.phase += incLowU32;
            } else {
                const uint32_t incHighU32 =
                    kFast ? toPhaseIncFast(baseInc * invH) : toPhaseIncU32(baseInc * invH);
                const uint32_t overLow = incLowU32 - remToWrap;
                const float d =
                    static_cast<float>(overLow) * fastRecipPos(static_cast<float>(incLowU32));
                addCubicBsplineBlep(v, d, 2.0f);
                v.lastEdgePulse += 0.22f;
                subRising |= static_cast<uint8_t>(1u << i);
                ++totalRisingCounts[i];
                if (osc0WrapPtr != nullptr && i == 0u) {
                    *osc0WrapPtr = true;
                    *osc0WrapDPtr = d;
                }
                uint32_t postHigh =
                    static_cast<uint32_t>(d * static_cast<float>(incHighU32) + 0.5f);
                if (postHigh >= kHalfPhase) {
                    postHigh = kHalfPhase - 1u;
                }
                v.phase = postHigh;
                v.currentLevel = 1.0f;
            }
        }
    };

    const float rcPole = rcHighpassPole_;
    const float railDecay = railDecayCoeff_;
    const bool doRcTilt = enableRcTilt_;
    uint8_t lastRising = prevRisingMask_;

    const auto runSpan = [&](auto modeTag) noexcept {
        constexpr bool kFast = decltype(modeTag)::kFast;
        for (size_t k = 0u; k < numFrames; ++k) {
            stepFrameDriftAndPw();
            uint8_t frameRising = 0u;
            for (size_t sub = 0u; sub < 4u; ++sub) {
                uint8_t subRising = 0u;
                [[maybe_unused]] bool osc0Wrapped = false;
                [[maybe_unused]] float osc0WrapD = 0.0f;
                float totalRailSag = 0.0f;
                [[maybe_unused]] float totalGateLoad = 0.0f;
                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    voiceRailSag_[i] = voiceRailSag_[i] * railDecay + voices_[i].lastEdgePulse;
                    voices_[i].lastEdgePulse = 0.0f;
                    totalRailSag += voiceRailSag_[i];
                    if constexpr (!kFast) {
                        totalGateLoad += voices_[i].currentLevel;
                    }
                }

                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    VoiceState& v = voices_[i];
                    curFreq[i] += freqStep4x[i];

                    const float railPullRatio = computeVoiceRailPull<!kFast>(
                        v.phase, v.currentLevel, totalRailSag - voiceRailSag_[i], totalGateLoad,
                        cp);
                    const float rawBaseInc = curFreq[i] * curBaseScale[i] * railPullRatio;
                    const float baseIncFloat =
                        kFast ? rawBaseInc : clampf(rawBaseInc, minBaseInc, maxBaseInc);

                    if constexpr (kFast) {
                        advanceNormalPhase(modeTag, v, i, baseIncFloat, halfInvHigh[i],
                                           halfInvLow[i], subRising, nullptr, nullptr);
                    } else {
                        const uint8_t neighborRose = static_cast<uint8_t>(
                            (lastRising | subRising) & static_cast<uint8_t>(~(1u << i)));
                        if (i > 0u && osc0Wrapped && syncActive && v.phase >= hardSyncThreshold) {
                            const uint32_t incHighU32 =
                                toPhaseIncU32(baseIncFloat * halfInvHigh[i]);
                            const uint32_t incLowU32 = toPhaseIncU32(baseIncFloat * halfInvLow[i]);
                            applyMasterHardSync(v, incHighU32, incLowU32, osc0WrapD);
                            subRising |= static_cast<uint8_t>(1u << i);
                            ++totalRisingCounts[i];
                        } else if (cp.chaos > 0.02f && neighborRose != 0u &&
                                   v.currentLevel < 0.0f && v.phase >= cp.syncCapturePhase) {
                            const uint32_t incHighU32 =
                                toPhaseIncU32(baseIncFloat * halfInvHigh[i]);
                            v.phase = (incHighU32 >> 1u);
                            addCubicBsplineBlep(v, 0.5f, 2.0f);
                            v.lastEdgePulse += 0.22f;
                            v.currentLevel = 1.0f;
                            subRising |= static_cast<uint8_t>(1u << i);
                            ++totalRisingCounts[i];
                            if (i == 0u) {
                                osc0Wrapped = true;
                                osc0WrapD = 0.5f;
                            }
                        } else {
                            advanceNormalPhase(modeTag, v, i, baseIncFloat, halfInvHigh[i],
                                               halfInvLow[i], subRising, &osc0Wrapped, &osc0WrapD);
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
                    if (doRcTilt) {
                        v.rcCapState = rcPole * (v.rcCapState + bandLimitedSquare - v.rcPrevInput);
                        v.rcPrevInput = bandLimitedSquare;
                        voiceSample = v.rcCapState;
                    }
                    spanSub[i][k][sub] = voiceSample;
                }
                frameRising |= subRising;
                lastRising = subRising;
            }
            spanRising[k] = frameRising;
        }
    };

    if (!syncActive && cp.chaos == 0.0f && safeFreqRange) {
        runSpan(FastDroneTag{});
    } else {
        runSpan(FullSyncTag{});
    }

    prevRisingMask_ = lastRising;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        voiceRailSag_[i] = sanitizeState(voiceRailSag_[i]);
        VoiceState& v = voices_[i];
        v.rcCapState = sanitizeState(v.rcCapState);
        outLastSamples[i] = spanSub[i][numFrames - 1u][3];
        outLastCapRamp[i] = phaseToCapRamp(v.phase);
    }
    if (optGateHigh != nullptr) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            (*optGateHigh)[i] = (voices_[i].currentLevel > 0.0f) ? 1.0f : 0.0f;
        }
    }
    if (optDriftNorm != nullptr) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            (*optDriftNorm)[i] = voices_[i].cachedDriftNorm;
        }
    }
}

} // namespace ruido
