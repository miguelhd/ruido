#include "core/DspMath.h"
#include "dsp/SchmittOscillatorBank.h"

namespace ruido {

void SchmittOscillatorBank::initSpanVoiceTables(size_t i, float sHz, float eHz, float sPw,
                                                float ePw, float driftAmt, float maxPullScale,
                                                float invSpan, float maxSafePeakHz,
                                                float& outCurBaseScale, float& outHalfInvHigh,
                                                float& outHalfInvLow, float& outStepInvHigh,
                                                float& outStepInvLow, bool& ioAnyPwSweep) noexcept {
    VoiceState& v = voices_[i];
    (void)driftAmt;
    outCurBaseScale = v.cachedDriftRatio * hzToPhaseU32_;

    const float rawMaxHz = (eHz > sHz) ? eHz : sHz;
    const float peakHz = clampf((rawMaxHz > maxSafePeakHz ? rawMaxHz : maxSafePeakHz) *
                                    v.cachedDriftRatio * maxPullScale,
                                0.005f, 21200.0f);
    const float minSafeDuty =
        clampf((peakHz * hzToPhaseU32_) * (0.5f / static_cast<float>(kMaxPhaseInc)), kPulseWidthMin,
               0.49f);

    const auto evalDutyInv = [&](float pw, float& hOut, float& lOut) noexcept {
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
        hOut = v.cachedHalfInvHigh;
        lOut = v.cachedHalfInvLow;
    };

    evalDutyInv(sPw, outHalfInvHigh, outHalfInvLow);
    outStepInvHigh = 0.0f;
    outStepInvLow = 0.0f;
    if (ePw != sPw) {
        ioAnyPwSweep = true;
        const float h0 = outHalfInvHigh;
        const float l0 = outHalfInvLow;
        float h1 = h0;
        float l1 = l0;
        evalDutyInv(ePw, h1, l1);
        outStepInvHigh = (h1 - h0) * invSpan;
        outStepInvLow = (l1 - l0) * invSpan;
        outHalfInvHigh = h0;
        outHalfInvLow = l0;
    }
}

RUIDO_NOINLINE void SchmittOscillatorBank::processFastDroneSpan4x(
    std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
    uint8_t* __restrict spanRising, std::array<uint8_t, kNumOscillators>& totalRisingCounts,
    size_t numFrames, float invSpan,
    const std::array<float, kNumOscillators>& __restrict startFreqHz,
    const std::array<float, kNumOscillators>& __restrict endFreqHz,
    const std::array<float, kNumOscillators>& __restrict startPw,
    const std::array<float, kNumOscillators>& __restrict endPw,
    const std::array<float, kNumOscillators>& __restrict drift, float couplingGain, float minPull,
    float maxPull, bool freezeDrift) noexcept {
    const float hzToPhase = hzToPhaseU32_;
    const float rcPole = rcHighpassPole_;
    const float railDecay4 = railDecayCoeff4x_;
    const bool doRcTilt = enableRcTilt_;
    const CouplingParams cp{0.0f, couplingGain, minPull, maxPull, 0xFFFFFFFFu};
    const float pullBound = (couplingGain > 0.0f) ? maxPull : 1.0f;

    std::array<float, kNumOscillators> curFreq{};
    std::array<float, kNumOscillators> freqStep{};
    std::array<float, kNumOscillators> curBaseScale{};
    std::array<float, kNumOscillators> halfInvHigh{};
    std::array<float, kNumOscillators> halfInvLow{};
    std::array<float, kNumOscillators> stepInvHigh{};
    std::array<float, kNumOscillators> stepInvLow{};
    std::array<float, kNumOscillators> dAmtClamped{};
    bool anyPwSweep = false;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const float sHz = startFreqHz[i];
        const float eHz = endFreqHz[i];
        const float stepHz = (eHz - sHz) * invSpan;
        freqStep[i] = stepHz;
        curFreq[i] = sHz - 0.375f * stepHz;
        dAmtClamped[i] = clampf(drift[i], 0.0f, 1.0f);
        initSpanVoiceTables(i, sHz, eHz, startPw[i], endPw[i], dAmtClamped[i], pullBound, invSpan,
                            0.0f, curBaseScale[i], halfInvHigh[i], halfInvLow[i], stepInvHigh[i],
                            stepInvLow[i], anyPwSweep);
    }

    const float maxBaseInc = 21200.0f * hzToPhase;
    uint8_t lastSubRising = prevRisingMask_;
    for (size_t k = 0u; k < numFrames; ++k) {
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
                halfInvHigh[i] += stepInvHigh[i];
                halfInvLow[i] += stepInvLow[i];
            }
        }

        float totalRailSag = 0.0f;
        if (couplingGain > 0.0f) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                voiceRailSag_[i] = voiceRailSag_[i] * railDecay4 + voices_[i].lastEdgePulse;
                voices_[i].lastEdgePulse = 0.0f;
                totalRailSag += voiceRailSag_[i];
            }
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                voices_[i].lastEdgePulse = 0.0f;
                voiceRailSag_[i] = 0.0f;
            }
        }

        uint8_t frameRising = 0u;
        uint8_t sub3Rising = 0u;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceState& v = voices_[i];
            curFreq[i] += freqStep[i];
            uint32_t phase = v.phase;
            float level = v.currentLevel;
            const float railPullRatio =
                (couplingGain > 0.0f) ? computeVoiceRailPull<false>(
                                            phase, level, totalRailSag - voiceRailSag_[i], 0.0f, cp)
                                      : 1.0f;
            const float rawBaseInc = curFreq[i] * curBaseScale[i] * railPullRatio;
            const float baseInc =
                (rawBaseInc > maxBaseInc) ? maxBaseInc : ((rawBaseInc > 1.0f) ? rawBaseInc : 1.0f);
            const auto incHigh = static_cast<uint32_t>(
                static_cast<int32_t>(clampf(baseInc * halfInvHigh[i], 1.0f, kMaxPhaseIncFloat)));
            const auto incLow = static_cast<uint32_t>(
                static_cast<int32_t>(clampf(baseInc * halfInvLow[i], 1.0f, kMaxPhaseIncFloat)));

            float naiveD1 = v.naiveD1;
            float naiveD2 = v.naiveD2;
            float rcCap = v.rcCapState;
            float rcPrev = v.rcPrevInput;
            uint32_t blepPending = v.blepPending;
            auto& dst4 = spanSub[i][k];

            for (size_t sub = 0u; sub < 4u; ++sub) {
                if (phase < kHalfPhase) {
                    const uint32_t remToHalf = kHalfPhase - phase;
                    if (incHigh < remToHalf) {
                        phase += incHigh;
                    } else {
                        phase = fastFallingEdge(v, incHigh - remToHalf, incHigh, incLow);
                        blepPending = 4u;
                        level = -1.0f;
                    }
                } else {
                    const uint32_t remToWrap = 0u - phase;
                    if (incLow < remToWrap) {
                        phase += incLow;
                    } else {
                        phase = fastRisingEdge(v, incLow - remToWrap, incLow, incHigh);
                        blepPending = 4u;
                        level = 1.0f;
                        const uint8_t vBit = static_cast<uint8_t>(1u << i);
                        frameRising |= vBit;
                        sub3Rising =
                            (sub == 3u) ? static_cast<uint8_t>(sub3Rising | vBit) : sub3Rising;
                        ++totalRisingCounts[i];
                    }
                }

                float sq = naiveD2;
                if (blepPending != 0u) {
                    sq += v.blepResidual[0];
                    v.blepResidual[0] = v.blepResidual[1];
                    v.blepResidual[1] = v.blepResidual[2];
                    v.blepResidual[2] = v.blepResidual[3];
                    v.blepResidual[3] = 0.0f;
                    --blepPending;
                }
                naiveD2 = naiveD1;
                naiveD1 = level;
                if (doRcTilt) {
                    rcCap = rcPole * (rcCap + sq - rcPrev);
                    rcPrev = sq;
                    sq = rcCap;
                }
                dst4[sub] = sq;
            }

            v.phase = phase;
            v.currentLevel = level;
            v.naiveD1 = naiveD1;
            v.naiveD2 = naiveD2;
            v.rcCapState = rcCap;
            v.rcPrevInput = rcPrev;
            v.blepPending = static_cast<uint8_t>(blepPending);
        }
        spanRising[k] = frameRising;
        lastSubRising = sub3Rising;
    }
    prevRisingMask_ = lastSubRising;
}

// Fast 4x sub-stepped span synthesis when `SURGE` (`cp.chaos > 0`) or `HARD SYNC` (`syncActive`)
// is active: preserves exact sub-sample cross-voice `syncCapturePhase` & Osc 0 hard-sync edges
// while hoisting coupling/sync thresholds, endpoint PWM duty reciprocals, thermal drift, and
// state sanitization out of the 64-subsample loop (~5x faster than 64x `processSample` calls).
RUIDO_NOINLINE void SchmittOscillatorBank::processCoupledSyncSpan4x(
    std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
    uint8_t* __restrict spanRising, std::array<uint8_t, kNumOscillators>& totalRisingCounts,
    size_t numFrames, float invSpan,
    const std::array<float, kNumOscillators>& __restrict startFreqHz,
    const std::array<float, kNumOscillators>& __restrict endFreqHz,
    const std::array<float, kNumOscillators>& __restrict startPw,
    const std::array<float, kNumOscillators>& __restrict endPw,
    const std::array<float, kNumOscillators>& __restrict drift, const CouplingParams& cp,
    bool freezeDrift, bool syncActive, uint32_t hardSyncThreshold) noexcept {
    const float hzToPhase = hzToPhaseU32_;
    const float rcPole = rcHighpassPole_;
    const float railDecay = railDecayCoeff_;
    const bool doRcTilt = enableRcTilt_;
    const bool chaosSync = (cp.chaos > 0.02f);
    const float maxBaseInc = 21200.0f * hzToPhase;
    const float invSpan4x = 0.25f * invSpan;

    std::array<float, kNumOscillators> curFreq{};
    std::array<float, kNumOscillators> freqStep4x{};
    std::array<float, kNumOscillators> curBaseScale{};
    std::array<float, kNumOscillators> halfInvHigh{};
    std::array<float, kNumOscillators> halfInvLow{};
    std::array<float, kNumOscillators> stepInvHigh{};
    std::array<float, kNumOscillators> stepInvLow{};
    std::array<float, kNumOscillators> dAmtClamped{};
    bool anyPwSweep = false;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const float sHz = startFreqHz[i];
        const float eHz = endFreqHz[i];
        curFreq[i] = sHz;
        freqStep4x[i] = (eHz - sHz) * invSpan4x;
        dAmtClamped[i] = clampf(drift[i], 0.0f, 1.0f);
        initSpanVoiceTables(i, sHz, eHz, startPw[i], endPw[i], dAmtClamped[i], cp.maxPull, invSpan,
                            0.0f, curBaseScale[i], halfInvHigh[i], halfInvLow[i], stepInvHigh[i],
                            stepInvLow[i], anyPwSweep);
    }

    uint8_t prevSubRising = prevRisingMask_;
    for (size_t k = 0u; k < numFrames; ++k) {
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
                halfInvHigh[i] += stepInvHigh[i];
                halfInvLow[i] += stepInvLow[i];
            }
        }

        uint8_t frameRising = 0u;
        for (size_t sub = 0u; sub < 4u; ++sub) {
            float totalRailSag = 0.0f;
            float totalGateLoad = 0.0f;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                voiceRailSag_[i] = voiceRailSag_[i] * railDecay + voices_[i].lastEdgePulse;
                voices_[i].lastEdgePulse = 0.0f;
                totalRailSag += voiceRailSag_[i];
                totalGateLoad += voices_[i].currentLevel;
            }

            uint8_t subRising = 0u;
            bool osc0Wrapped = false;
            float osc0WrapD = 0.0f;

            for (size_t i = 0u; i < kNumOscillators; ++i) {
                VoiceState& v = voices_[i];
                curFreq[i] += freqStep4x[i];
                const float railPullRatio = computeVoiceRailPull<true>(
                    v.phase, v.currentLevel, totalRailSag - voiceRailSag_[i], totalGateLoad, cp);
                const float rawBaseInc = curFreq[i] * curBaseScale[i] * railPullRatio;
                const float baseInc = (rawBaseInc > maxBaseInc)
                                          ? maxBaseInc
                                          : ((rawBaseInc > 1.0f) ? rawBaseInc : 1.0f);
                const auto incHigh = static_cast<uint32_t>(static_cast<int32_t>(
                    clampf(baseInc * halfInvHigh[i], 1.0f, kMaxPhaseIncFloat)));
                const auto incLow = static_cast<uint32_t>(
                    static_cast<int32_t>(clampf(baseInc * halfInvLow[i], 1.0f, kMaxPhaseIncFloat)));

                const uint8_t vBit = static_cast<uint8_t>(1u << i);
                const uint8_t neighborRose =
                    static_cast<uint8_t>((prevSubRising | subRising) & static_cast<uint8_t>(~vBit));
                if (i > 0u && osc0Wrapped && syncActive && v.phase >= hardSyncThreshold) {
                    applyMasterHardSync(v, incHigh, incLow, osc0WrapD);
                    subRising |= vBit;
                    ++totalRisingCounts[i];
                } else if (chaosSync && neighborRose != 0u && v.currentLevel < 0.0f &&
                           v.phase >= cp.syncCapturePhase) {
                    v.phase = (incHigh >> 1u);
                    addCubicBsplineBlep(v, 0.5f, 2.0f);
                    v.lastEdgePulse += 0.22f;
                    v.currentLevel = 1.0f;
                    subRising |= vBit;
                    ++totalRisingCounts[i];
                    if (i == 0u) {
                        osc0Wrapped = true;
                        osc0WrapD = 0.5f;
                    }
                } else if (v.phase < kHalfPhase) {
                    const uint32_t remToHalf = kHalfPhase - v.phase;
                    if (incHigh < remToHalf) {
                        v.phase += incHigh;
                    } else {
                        v.phase = fastFallingEdge(v, incHigh - remToHalf, incHigh, incLow);
                        v.currentLevel = -1.0f;
                    }
                } else {
                    const uint32_t remToWrap = 0u - v.phase;
                    if (incLow < remToWrap) {
                        v.phase += incLow;
                    } else {
                        const uint32_t overLow = incLow - remToWrap;
                        if (i == 0u) {
                            osc0Wrapped = true;
                            osc0WrapD = static_cast<float>(overLow) *
                                        fastRecipPos(static_cast<float>(incLow));
                        }
                        v.phase = fastRisingEdge(v, overLow, incLow, incHigh);
                        v.currentLevel = 1.0f;
                        subRising |= vBit;
                        ++totalRisingCounts[i];
                    }
                }

                float sq = v.naiveD2;
                if (v.blepPending != 0u) {
                    sq += v.blepResidual[0];
                    v.blepResidual[0] = v.blepResidual[1];
                    v.blepResidual[1] = v.blepResidual[2];
                    v.blepResidual[2] = v.blepResidual[3];
                    v.blepResidual[3] = 0.0f;
                    --v.blepPending;
                }
                v.naiveD2 = v.naiveD1;
                v.naiveD1 = v.currentLevel;
                if (doRcTilt) {
                    v.rcCapState = rcPole * (v.rcCapState + sq - v.rcPrevInput);
                    v.rcPrevInput = sq;
                    sq = v.rcCapState;
                }
                spanSub[i][k][sub] = sq;
            }
            frameRising |= subRising;
            prevSubRising = subRising;
        }
        spanRising[k] = frameRising;
    }
    prevRisingMask_ = prevSubRising;
}

void SchmittOscillatorBank::preparePatchedSpan(
    PatchedSpanState& st, float invSpan, const std::array<float, kNumOscillators>& startFreqHz,
    const std::array<float, kNumOscillators>& endFreqHz,
    const std::array<float, kNumOscillators>& startPw,
    const std::array<float, kNumOscillators>& endPw,
    const std::array<float, kNumOscillators>& drift, float circuitCoupling, bool freezeDrift,
    float hardSync, uint8_t needCapRampMask) noexcept {
    st.cp = computeCouplingParams(circuitCoupling);
    const float syncAmt = clampf(hardSync, 0.0f, 1.0f);
    st.syncActive = (syncAmt > 1.0e-4f);
    st.hardSyncThreshold = computeHardSyncThreshold(syncAmt);
    st.maxBaseInc = 21200.0f * hzToPhaseU32_;
    st.needCapRampMask = needCapRampMask;
    st.anyPwSweep = false;
    st.freezeDrift = freezeDrift;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        st.driftClamped[i] = clampf(drift[i], 0.0f, 1.0f);
        // XMod can sweep up to 21.2 kHz dynamically, so pass 21200 Hz when XMod touches voice i
        const float xmodPeakHz = (needCapRampMask != 0u) ? 21200.0f : 0.0f;
        initSpanVoiceTables(i, startFreqHz[i], endFreqHz[i], startPw[i], endPw[i],
                            st.driftClamped[i], st.cp.maxPull, invSpan, xmodPeakHz,
                            st.curBaseScale[i], st.halfInvHigh[i], st.halfInvLow[i],
                            st.stepInvHigh[i], st.stepInvLow[i], st.anyPwSweep);
    }
}

void SchmittOscillatorBank::beginPatchedFrame(PatchedSpanState& st) noexcept {
    if (!st.freezeDrift) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceState& v = voices_[i];
            const uint32_t prevDc = v.driftCounter;
            v.driftCounter = prevDc + 4u;
            if (((prevDc & 63u) == 0u) || (((prevDc & 63u) + 4u) > 64u)) {
                stepVoiceThermalDrift(v, st.driftClamped[i], enableDrift_);
                st.curBaseScale[i] = v.cachedDriftRatio * hzToPhaseU32_;
            }
        }
    }
    if (st.anyPwSweep) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            st.halfInvHigh[i] += st.stepInvHigh[i];
            st.halfInvLow[i] += st.stepInvLow[i];
        }
    }
}

uint8_t SchmittOscillatorBank::stepPatchedSubSample(
    const PatchedSpanState& st, const std::array<float, kNumOscillators>& subFreqHz,
    std::array<float, kNumOscillators>& outSamples,
    std::array<float, kNumOscillators>& outCapRamp) noexcept {
    const uint8_t lastRising = prevRisingMask_;
    const float railDecay = railDecayCoeff_;
    const float rcPole = rcHighpassPole_;
    const bool doRcTilt = enableRcTilt_;
    const bool chaosSync = (st.cp.chaos > 0.02f);

    float totalRailSag = 0.0f;
    float totalGateLoad = 0.0f;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        voiceRailSag_[i] = voiceRailSag_[i] * railDecay + voices_[i].lastEdgePulse;
        voices_[i].lastEdgePulse = 0.0f;
        totalRailSag += voiceRailSag_[i];
        totalGateLoad += voices_[i].currentLevel;
    }

    uint8_t risingMask = 0u;
    bool osc0Wrapped = false;
    float osc0WrapD = 0.0f;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        const float railPullRatio = computeVoiceRailPull<true>(
            v.phase, v.currentLevel, totalRailSag - voiceRailSag_[i], totalGateLoad, st.cp);
        const float rawBaseInc = subFreqHz[i] * st.curBaseScale[i] * railPullRatio;
        const float baseInc = (rawBaseInc > st.maxBaseInc)
                                  ? st.maxBaseInc
                                  : ((rawBaseInc > 1.0f) ? rawBaseInc : 1.0f);
        const auto incHigh = static_cast<uint32_t>(
            static_cast<int32_t>(clampf(baseInc * st.halfInvHigh[i], 1.0f, kMaxPhaseIncFloat)));
        const auto incLow = static_cast<uint32_t>(
            static_cast<int32_t>(clampf(baseInc * st.halfInvLow[i], 1.0f, kMaxPhaseIncFloat)));

        const uint8_t vBit = static_cast<uint8_t>(1u << i);
        const uint8_t neighborRose =
            static_cast<uint8_t>((lastRising | risingMask) & static_cast<uint8_t>(~vBit));
        if (i > 0u && osc0Wrapped && st.syncActive && v.phase >= st.hardSyncThreshold) {
            applyMasterHardSync(v, incHigh, incLow, osc0WrapD);
            risingMask |= vBit;
        } else if (chaosSync && neighborRose != 0u && v.currentLevel < 0.0f &&
                   v.phase >= st.cp.syncCapturePhase) {
            v.phase = (incHigh >> 1u);
            addCubicBsplineBlep(v, 0.5f, 2.0f);
            v.lastEdgePulse += 0.22f;
            v.currentLevel = 1.0f;
            risingMask |= vBit;
            if (i == 0u) {
                osc0Wrapped = true;
                osc0WrapD = 0.5f;
            }
        } else if (v.phase < kHalfPhase) {
            const uint32_t remToHalf = kHalfPhase - v.phase;
            if (incHigh < remToHalf) {
                v.phase += incHigh;
            } else {
                v.phase = fastFallingEdge(v, incHigh - remToHalf, incHigh, incLow);
                v.currentLevel = -1.0f;
            }
        } else {
            const uint32_t remToWrap = 0u - v.phase;
            if (incLow < remToWrap) {
                v.phase += incLow;
            } else {
                const uint32_t overLow = incLow - remToWrap;
                if (i == 0u) {
                    osc0Wrapped = true;
                    osc0WrapD =
                        static_cast<float>(overLow) * fastRecipPos(static_cast<float>(incLow));
                }
                v.phase = fastRisingEdge(v, overLow, incLow, incHigh);
                v.currentLevel = 1.0f;
                risingMask |= vBit;
            }
        }

        float sq = v.naiveD2;
        if (v.blepPending != 0u) {
            sq += v.blepResidual[0];
            v.blepResidual[0] = v.blepResidual[1];
            v.blepResidual[1] = v.blepResidual[2];
            v.blepResidual[2] = v.blepResidual[3];
            v.blepResidual[3] = 0.0f;
            --v.blepPending;
        }
        v.naiveD2 = v.naiveD1;
        v.naiveD1 = v.currentLevel;
        if (doRcTilt) {
            v.rcCapState = rcPole * (v.rcCapState + sq - v.rcPrevInput);
            v.rcPrevInput = sq;
            sq = v.rcCapState;
        }
        outSamples[i] = sq;
        if ((st.needCapRampMask & vBit) != 0u) {
            outCapRamp[i] = phaseToCapRamp(v.phase);
        }
    }
    prevRisingMask_ = risingMask;
    return risingMask;
}

void SchmittOscillatorBank::finishPatchedSpan(
    std::array<float, kNumOscillators>* optGateHigh,
    std::array<float, kNumOscillators>* optDriftNorm) noexcept {
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        voiceRailSag_[i] = sanitizeState(voiceRailSag_[i]);
        voices_[i].rcCapState = sanitizeState(voices_[i].rcCapState);
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
    const CouplingParams cp = computeCouplingParams(circuitCoupling);
    const float syncAmt = clampf(hardSync, 0.0f, 1.0f);
    const bool syncActive = (syncAmt > 1.0e-4f);

    bool safeMinHz = true;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const float minHz = (endFreqHz[i] < startFreqHz[i]) ? endFreqHz[i] : startFreqHz[i];
        if (minHz < 0.05f) {
            safeMinHz = false;
            break;
        }
    }

    if (safeMinHz) {
        if (!syncActive && cp.chaos == 0.0f) {
            processFastDroneSpan4x(spanSub, spanRising, totalRisingCounts, numFrames, invSpan,
                                   startFreqHz, endFreqHz, startPw, endPw, drift, cp.couplingGain,
                                   cp.minPull, cp.maxPull, freezeDrift);
        } else {
            processCoupledSyncSpan4x(spanSub, spanRising, totalRisingCounts, numFrames, invSpan,
                                     startFreqHz, endFreqHz, startPw, endPw, drift, cp, freezeDrift,
                                     syncActive, computeHardSyncThreshold(syncAmt));
        }
        finishPatchedSpan(optGateHigh, optDriftNorm);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            outLastSamples[i] = spanSub[i][numFrames - 1u][3];
            outLastCapRamp[i] = phaseToCapRamp(voices_[i].phase);
        }
        return;
    }

    const float invSpan4x = 0.25f * invSpan;
    std::array<float, kNumOscillators> curFreq{};
    std::array<float, kNumOscillators> freqStep4x{};
    std::array<float, kNumOscillators> curPw{};
    std::array<float, kNumOscillators> pwStep{};
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        curFreq[i] = startFreqHz[i];
        freqStep4x[i] = (endFreqHz[i] - startFreqHz[i]) * invSpan4x;
        curPw[i] = startPw[i];
        pwStep[i] = (endPw[i] - startPw[i]) * invSpan;
    }

    OscillatorBankOutput subOut{};
    for (size_t k = 0u; k < numFrames; ++k) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            curPw[i] += pwStep[i];
        }
        uint8_t frameRising = 0u;
        for (size_t sub = 0u; sub < 4u; ++sub) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                curFreq[i] += freqStep4x[i];
            }
            processSample(subOut, curFreq, curPw, drift, circuitCoupling, freezeDrift, hardSync);
            frameRising |= subOut.risingMask;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                spanSub[i][k][sub] = subOut.samples[i];
                if ((subOut.risingMask & static_cast<uint8_t>(1u << i)) != 0u) {
                    ++totalRisingCounts[i];
                }
            }
        }
        spanRising[k] = frameRising;
    }
    finishPatchedSpan(optGateHigh, optDriftNorm);
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        outLastSamples[i] = spanSub[i][numFrames - 1u][3];
        outLastCapRamp[i] = phaseToCapRamp(voices_[i].phase);
    }
}

} // namespace ruido
