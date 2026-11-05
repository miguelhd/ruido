#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/GlobalControl.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"

namespace ruido {

namespace {

void finalizeSpread(GlobalColumnSummary& s) noexcept {
    float lo = s.voiceSigned[0];
    float hi = s.voiceSigned[0];
    for (size_t i = 1u; i < kNumOscillators; ++i) {
        if (s.voiceSigned[i] < lo) {
            lo = s.voiceSigned[i];
        }
        if (s.voiceSigned[i] > hi) {
            hi = s.voiceSigned[i];
        }
    }
    s.hasVoiceSpread = (hi - lo) > 0.045f;
}

} // namespace

GlobalColumnSummary evaluateGlobalColumn(KnobBankMode bank, uint8_t knobIdx,
                                         const EngineControlState& control,
                                         const LfoSignalArray& lfoOutput,
                                         uint8_t stutterGateMask) noexcept {
    GlobalColumnSummary s{};
    const uint8_t k = knobIdx & 3u;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;

    if (bank == KnobBankMode::GlobalAll) {
        if (k == 0u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalFreq, 0u, control, lfoOutput,
                                        stutterGateMask);
        }
        if (k == 1u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalFilter, 2u, control, lfoOutput,
                                        stutterGateMask);
        }
        if (k == 2u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalLfo, 3u, control, lfoOutput,
                                        stutterGateMask);
        }
        return evaluateGlobalColumn(KnobBankMode::GlobalAmp, 0u, control, lfoOutput,
                                    stutterGateMask);
    }

    if (bank == KnobBankMode::GlobalFreq) {
        std::array<float, kNumOscillators> logs{};
        float meanLog = 0.0f;
        float minLog = 100.0f;
        float maxLog = -100.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const VoiceParams& vp = control.voices[i];
            const float pMul = computeVoicePitchRatio(control.voicePressure[i], pitchMode) *
                               control.keyMod.voicePitchMul[i];
            const float lfoHz = computeLiveVoicePitchHz(vp, lfoOutput[i], lfoOutput[4u + i]);
            const float liveHz = clampf(lfoHz * pMul, kAudioFreqMinHz, kAudioFreqMaxHz);
            logs[i] = fastLog2(liveHz);
            meanLog += logs[i];
            if (logs[i] < minLog) {
                minLog = logs[i];
            }
            if (logs[i] > maxLog) {
                maxLog = logs[i];
            }
            const float vNorm = clampf((logs[i] - kGcAudioLogMin) / kGcAudioLogSpan, 0.0f, 1.0f);
            s.voiceSigned[i] = 2.0f * vNorm - 1.0f;
        }
        meanLog *= 0.25f;

        if (k == 0u) {
            s.primaryValue = fastExp2(meanLog);
            const float norm = clampf((meanLog - kGcAudioLogMin) / kGcAudioLogSpan, 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
            finalizeSpread(s);
        } else if (k == 1u) {
            const float spanSt = (maxLog - minLog) * 12.0f;
            s.primaryValue = spanSt;
            const float norm = clampf(spanSt * (1.0f / 36.0f), 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
        } else if (k == 2u) {
            s.bipolar = true;
            const float detOct = 0.25f * (logs[0] - logs[1] - logs[2] + logs[3]);
            const float cents = clampf(detOct * 1200.0f, -100.0f, 100.0f);
            s.primaryValue = cents;
            s.posSigned = clampf(cents * 0.01f, -1.0f, 1.0f);
        } else {
            s.bipolar = true;
            const float tiltOct =
                0.45f * (-0.3333333f * logs[0] + logs[1] - logs[2] + 0.3333333f * logs[3]);
            const float tiltSt = clampf(tiltOct * 12.0f, -6.0f, 6.0f);
            s.primaryValue = tiltSt;
            s.posSigned = clampf(tiltSt * (1.0f / 6.0f), -1.0f, 1.0f);
        }
        return s;
    }

    if (bank == KnobBankMode::GlobalFilter) {
        float sumVal = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const VoiceParams& vp = control.voices[i];
            const float pluckEnv = (((stutterGateMask >> i) & 1u) != 0u) ? 0.55f : 0.0f;
            const ModulatedFilterParams mod = computeLiveVoiceFilter(
                vp, control.voicePressure[i], pitchMode, lfoOutput[i], lfoOutput[4u + i],
                FilterMacroContext::fromKeyMod(control.keyMod, pluckEnv));
            float vNorm = 0.0f;
            if (k == 0u) {
                const float lg = fastLog2(mod.hpCutoffHz);
                sumVal += lg;
                vNorm = clampf((lg - kGcFilterLogLo) / kGcFilterLogRange, 0.0f, 1.0f);
            } else if (k == 1u) {
                sumVal += mod.hpResonance;
                vNorm = clampf(mod.hpResonance, 0.0f, 1.0f);
            } else if (k == 2u) {
                const float lg = fastLog2(mod.lpCutoffHz);
                sumVal += lg;
                vNorm = clampf((lg - kGcFilterLogLo) / kGcFilterLogRange, 0.0f, 1.0f);
            } else {
                sumVal += mod.lpResonance;
                vNorm = clampf(mod.lpResonance, 0.0f, 1.0f);
            }
            s.voiceSigned[i] = 2.0f * vNorm - 1.0f;
        }
        sumVal *= 0.25f;
        if (k == 0u || k == 2u) {
            s.primaryValue = fastExp2(sumVal);
            const float norm = clampf((sumVal - kGcFilterLogLo) / kGcFilterLogRange, 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
        } else {
            s.primaryValue = clampf(sumVal, 0.0f, 1.0f);
            s.posSigned = 2.0f * s.primaryValue - 1.0f;
        }
        finalizeSpread(s);
        return s;
    }

    if (bank == KnobBankMode::GlobalLfo) {
        constexpr float kInvMaxTgt = 1.0f / static_cast<float>(kNumLfoATargets - 1u);
        bool anyActive = false;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            if (control.voices[i].lfoDepth > 0.002f || control.voices[i].lfoBDepth > 0.002f) {
                anyActive = true;
                break;
            }
        }

        if (k == 0u) {
            bool haveRef = false;
            LfoTarget ref = control.voices[0].lfoTarget;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                const size_t domSub =
                    (vp.lfoBDepth > vp.lfoDepth && vp.lfoBDepth > 0.002f) ? 1u : 0u;
                const LfoTarget domTgt = lfoSlotTarget(vp, domSub);
                const float vn = clampf(
                    static_cast<float>(static_cast<uint8_t>(domTgt)) * kInvMaxTgt, 0.0f, 1.0f);
                s.voiceSigned[i] = 2.0f * vn - 1.0f;

                for (size_t sub = 0u; sub < kLfosPerVoice; ++sub) {
                    if (anyActive ? (lfoSlotDepth(vp, sub) <= 0.002f) : (sub != 0u)) {
                        continue;
                    }
                    const LfoTarget t = lfoSlotTarget(vp, sub);
                    if (!haveRef) {
                        ref = t;
                        haveRef = true;
                    } else if (t != ref) {
                        s.lfoTargetMixed = true;
                    }
                }
            }
            s.primaryValue = static_cast<float>(static_cast<uint8_t>(ref));
            const float norm =
                clampf(static_cast<float>(static_cast<uint8_t>(ref)) * kInvMaxTgt, 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
            finalizeSpread(s);
        } else if (k == 1u) {
            float lo = 1.0f;
            float hi = 0.0f;
            float sum = 0.0f;
            uint32_t count = 0u;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                const size_t domSub =
                    (vp.lfoBDepth > vp.lfoDepth && vp.lfoBDepth > 0.002f) ? 1u : 0u;
                s.voiceSigned[i] = 2.0f * clampf(lfoSlotShape(vp, domSub), 0.0f, 1.0f) - 1.0f;
                for (size_t sub = 0u; sub < kLfosPerVoice; ++sub) {
                    if (anyActive ? (lfoSlotDepth(vp, sub) <= 0.002f) : (sub != 0u)) {
                        continue;
                    }
                    const float shp = clampf(lfoSlotShape(vp, sub), 0.0f, 1.0f);
                    sum += shp;
                    ++count;
                    if (shp < lo) {
                        lo = shp;
                    }
                    if (shp > hi) {
                        hi = shp;
                    }
                }
            }
            s.lfoShapeMixed = (count > 1u) && ((hi - lo) > 0.05f);
            s.primaryValue = (count > 0u) ? (sum / static_cast<float>(count)) : 0.0f;
            s.posSigned = 2.0f * s.primaryValue - 1.0f;
            finalizeSpread(s);
        } else if (k == 2u) {
            float sumLog = 0.0f;
            uint32_t count = 0u;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                const size_t domSub =
                    (vp.lfoBDepth > vp.lfoDepth && vp.lfoBDepth > 0.002f) ? 1u : 0u;
                const float domLg = fastLog2(lfoSlotRateHz(vp, domSub));
                const float vn = clampf((domLg - kGcLfoLogLo) / kGcLfoLogRange, 0.0f, 1.0f);
                s.voiceSigned[i] = 2.0f * vn - 1.0f;
                for (size_t sub = 0u; sub < kLfosPerVoice; ++sub) {
                    if (anyActive ? (lfoSlotDepth(vp, sub) <= 0.002f) : (sub != 0u)) {
                        continue;
                    }
                    sumLog += fastLog2(lfoSlotRateHz(vp, sub));
                    ++count;
                }
            }
            const float meanLog =
                (count > 0u) ? (sumLog / static_cast<float>(count)) : fastLog2(0.25f);
            s.primaryValue = fastExp2(meanLog);
            const float norm = clampf((meanLog - kGcLfoLogLo) / kGcLfoLogRange, 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
            finalizeSpread(s);
        } else {
            float sum = 0.0f;
            uint32_t count = 0u;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                const float maxV = (vp.lfoBDepth > vp.lfoDepth) ? vp.lfoBDepth : vp.lfoDepth;
                s.voiceSigned[i] = 2.0f * clampf(maxV, 0.0f, 1.0f) - 1.0f;
                for (size_t sub = 0u; sub < kLfosPerVoice; ++sub) {
                    if (anyActive ? (lfoSlotDepth(vp, sub) <= 0.002f) : (sub != 0u)) {
                        continue;
                    }
                    sum += clampf(lfoSlotDepth(vp, sub), 0.0f, 1.0f);
                    ++count;
                }
            }
            s.primaryValue = (count > 0u) ? (sum / static_cast<float>(count)) : 0.0f;
            s.posSigned = 2.0f * s.primaryValue - 1.0f;
            finalizeSpread(s);
        }
        return s;
    }

    // KnobBankMode::GlobalAmp
    std::array<float, kNumOscillators> amps{};
    std::array<float, kNumOscillators> pans{};
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = control.voices[i];
        amps[i] =
            evaluateLiveOscAmplitude(control, i, lfoOutput[i], lfoOutput[4u + i], stutterGateMask);
        const float livePan = computeLiveVoicePan(vp, lfoOutput[i], lfoOutput[4u + i]);
        pans[i] = applySpatialFocusPan(livePan, i, control.global.monoCollapse,
                                       control.keyMod.spatialFocus);
    }

    if (k == 0u) {
        float sum = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            sum += amps[i];
            s.voiceSigned[i] = 2.0f * amps[i] - 1.0f;
        }
        s.primaryValue = sum * 0.25f;
        s.posSigned = 2.0f * s.primaryValue - 1.0f;
        finalizeSpread(s);
    } else if (k == 1u) {
        s.bipolar = true;
        s.primaryValue = clampf(0.5f * ((amps[2] + amps[3]) - (amps[0] + amps[1])), -1.0f, 1.0f);
        s.posSigned = s.primaryValue;
    } else if (k == 2u) {
        const float meanAmp = 0.25f * (amps[0] + amps[1] + amps[2] + amps[3]);
        float sumSq = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const float dev = amps[i] - meanAmp;
            sumSq += dev * dev;
            s.voiceSigned[i] = 2.0f * amps[i] - 1.0f;
        }
        const float rms = (sumSq > 1.0e-8f) ? (sumSq * 0.25f * fastInvSqrt(sumSq * 0.25f)) : 0.0f;
        s.primaryValue = clampf(rms * (1.0f / kRefAmpSpreadRms), 0.0f, 1.0f);
        s.posSigned = 2.0f * s.primaryValue - 1.0f;
        finalizeSpread(s);
    } else if (control.global.monoCollapse) {
        s.disabled = true;
    } else {
        s.bipolar = true;
        float sumPan = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            sumPan += pans[i];
            s.voiceSigned[i] = pans[i];
        }
        s.primaryValue = clampf(sumPan * 0.25f, -1.0f, 1.0f);
        s.posSigned = s.primaryValue;
        finalizeSpread(s);
    }
    return s;
}

} // namespace ruido
