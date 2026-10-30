#include "core/GlobalControl.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kGcAudioLogMin = cxLog2Pos(kAudioFreqMinHz);
constexpr float kGcAudioLogMax = cxLog2Pos(kAudioFreqMaxHz);
constexpr float kGcAudioLogSpan = kGcAudioLogMax - kGcAudioLogMin;
constexpr float kGcFilterLogLo = cxLog2Pos(kFilterCutoffMinHz);
constexpr float kGcFilterLogHi = cxLog2Pos(kFilterCutoffMaxHz);
constexpr float kGcFilterLogRange = kGcFilterLogHi - kGcFilterLogLo;
constexpr float kGcLfoLogLo = cxLog2Pos(kLfoFreqMinHz);
constexpr float kGcLfoLogRange = cxLog2Pos(kLfoFreqMaxHz) - kGcLfoLogLo;
constexpr float kRefPanRms = 0.74759615f; // RMS of {-1.0, -0.34, +0.34, +1.0}

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

void applyGlobalFreqKnob(uint8_t knobIdx, int32_t ticks, KnobRate rate,
                         EngineControlState& state) noexcept {
    std::array<float, kNumOscillators> logs{};
    float meanLog = 0.0f;
    float minLog = 100.0f;
    float maxLog = -100.0f;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        logs[i] = fastLog2(clampf(state.voices[i].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz));
        meanLog += logs[i];
        if (logs[i] < minLog) {
            minLog = logs[i];
        }
        if (logs[i] > maxLog) {
            maxLog = logs[i];
        }
    }
    meanLog *= 0.25f;

    if (knobIdx == 0u) {
        const float octPerStep = (rate == KnobRate::Fine)     ? 0.00055f
                                 : (rate == KnobRate::Coarse) ? 0.06000f
                                                              : 0.02850f;
        const float rawDelta = static_cast<float>(ticks) * octPerStep;
        const float delta = clampf(rawDelta, kGcAudioLogMin - minLog, kGcAudioLogMax - maxLog);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            state.voices[i].frequencyHz =
                clampf(fastExp2(logs[i] + delta), kAudioFreqMinHz, kAudioFreqMaxHz);
        }
        return;
    }

    if (knobIdx == 1u) {
        std::array<float, kNumOscillators> dev{};
        float sumSq = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            dev[i] = logs[i] - meanLog;
            sumSq += dev[i] * dev[i];
        }
        const float rms = (sumSq > 1.0e-8f) ? (sumSq * 0.25f * fastInvSqrt(sumSq * 0.25f)) : 0.0f;
        std::array<float, kNumOscillators> unit{-1.3416408f, -0.4472136f, 0.4472136f, 1.3416408f};
        if (rms > 0.0015f) {
            const float invRms = 1.0f / rms;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                unit[i] = dev[i] * invRms;
            }
        }
        const float stepOct = (rate == KnobRate::Fine)     ? 0.00045f
                              : (rate == KnobRate::Coarse) ? 0.02800f
                                                           : 0.01200f;
        float maxScale = 2.20f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            if (unit[i] > 0.01f) {
                const float cap = (kGcAudioLogMax - meanLog) / unit[i];
                if (cap < maxScale) {
                    maxScale = cap;
                }
            } else if (unit[i] < -0.01f) {
                const float cap = (kGcAudioLogMin - meanLog) / unit[i];
                if (cap < maxScale) {
                    maxScale = cap;
                }
            }
        }
        if (maxScale < 0.0f) {
            maxScale = 0.0f;
        }
        const float nextRms = clampf(rms + static_cast<float>(ticks) * stepOct, 0.0f, maxScale);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            state.voices[i].frequencyHz =
                clampf(fastExp2(meanLog + unit[i] * nextRms), kAudioFreqMinHz, kAudioFreqMaxHz);
        }
        return;
    }

    if (knobIdx == 2u) {
        constexpr std::array<float, kNumOscillators> kDetuneBasis{1.0f, -1.0f, -1.0f, 1.0f};
        const float curOct = 0.25f * (logs[0] - logs[1] - logs[2] + logs[3]);
        const float stepOct = (rate == KnobRate::Fine)     ? 0.00018f
                              : (rate == KnobRate::Coarse) ? 0.00420f
                                                           : 0.00160f;
        constexpr float kMaxDetuneOct = 1.0f / 12.0f; // +-100 cents
        const float nextOct =
            clampf(curOct + static_cast<float>(ticks) * stepOct, -kMaxDetuneOct, kMaxDetuneOct);
        const float dOct = nextOct - curOct;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            state.voices[i].frequencyHz = clampf(fastExp2(logs[i] + kDetuneBasis[i] * dOct),
                                                 kAudioFreqMinHz, kAudioFreqMaxHz);
        }
        return;
    }

    constexpr std::array<float, kNumOscillators> kTiltBasis{-0.3333333f, 1.0f, -1.0f, 0.3333333f};
    const float curTilt = 0.45f * (kTiltBasis[0] * logs[0] + kTiltBasis[1] * logs[1] +
                                   kTiltBasis[2] * logs[2] + kTiltBasis[3] * logs[3]);
    const float stepOct = (rate == KnobRate::Fine)     ? 0.00035f
                          : (rate == KnobRate::Coarse) ? 0.01350f
                                                       : 0.00550f;
    const float nextTilt = clampf(curTilt + static_cast<float>(ticks) * stepOct, -0.50f, 0.50f);
    const float dTilt = nextTilt - curTilt;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        state.voices[i].frequencyHz =
            clampf(fastExp2(logs[i] + kTiltBasis[i] * dTilt), kAudioFreqMinHz, kAudioFreqMaxHz);
    }
}

void applyGlobalAmpKnob(uint8_t knobIdx, int32_t ticks, KnobRate rate,
                        EngineControlState& state) noexcept {
    if (knobIdx == 0u) {
        bool anyUnmuted = false;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            if (!state.voices[i].muted) {
                anyUnmuted = true;
            }
        }
        if (!anyUnmuted) {
            if (ticks > 0) {
                const float seeded = adjustLinear01(0.0f, ticks, rate, 0.0f, 1.0f);
                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    state.voices[i].muted = false;
                    state.voices[i].amplitude = seeded;
                }
            }
            return;
        }
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            if (!state.voices[i].muted) {
                state.voices[i].amplitude =
                    adjustLinear01(state.voices[i].amplitude, ticks, rate, 0.0f, 1.0f);
            }
        }
        return;
    }

    if (knobIdx == 1u) {
        const float delta = adjustLinear01(0.5f, ticks, rate, 0.0f, 1.0f) - 0.5f;
        state.voices[0].amplitude = clampf(state.voices[0].amplitude - delta, 0.0f, 1.0f);
        state.voices[1].amplitude = clampf(state.voices[1].amplitude - delta, 0.0f, 1.0f);
        state.voices[2].amplitude = clampf(state.voices[2].amplitude + delta, 0.0f, 1.0f);
        state.voices[3].amplitude = clampf(state.voices[3].amplitude + delta, 0.0f, 1.0f);
        return;
    }

    if (knobIdx == 2u) {
        float sumSq = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            sumSq += state.voices[i].pan * state.voices[i].pan;
        }
        const float rms = (sumSq > 1.0e-8f) ? (sumSq * 0.25f * fastInvSqrt(sumSq * 0.25f)) : 0.0f;
        const float curWidth = clampf(rms * (1.0f / kRefPanRms), 0.0f, 1.0f);
        const float nextWidth = adjustLinear01(curWidth, ticks, rate, 0.0f, 1.0f);
        if (rms > 0.005f) {
            const float scale = (nextWidth * kRefPanRms) / rms;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].pan = clampf(state.voices[i].pan * scale, -1.0f, 1.0f);
            }
        } else if (ticks > 0) {
            constexpr std::array<float, kNumOscillators> kSeedPan{-1.0f, -0.34f, 0.34f, 1.0f};
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].pan = clampf(kSeedPan[i] * nextWidth, -1.0f, 1.0f);
            }
        }
        return;
    }

    const float delta = adjustLinear01(0.5f, ticks, rate, 0.0f, 1.0f) - 0.5f;
    state.voices[0].amplitude = clampf(state.voices[0].amplitude - delta, 0.0f, 1.0f);
    state.voices[1].amplitude = clampf(state.voices[1].amplitude + delta, 0.0f, 1.0f);
    state.voices[2].amplitude = clampf(state.voices[2].amplitude - delta, 0.0f, 1.0f);
    state.voices[3].amplitude = clampf(state.voices[3].amplitude + delta, 0.0f, 1.0f);
}

} // namespace

void applyGlobalKnobDelta(GlobalTarget target, uint8_t knobIdx, int32_t ticks, KnobRate rate,
                          int32_t& targetTickAccum, EngineControlState& state) noexcept {
    const uint8_t k = knobIdx & 3u;
    switch (target) {
    case GlobalTarget::Freq:
        applyGlobalFreqKnob(k, ticks, rate, state);
        break;
    case GlobalTarget::Filter:
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceParams& vp = state.voices[i];
            if (k == 0u) {
                vp.hpCutoffHz = adjustFilterCutoff(vp.hpCutoffHz, ticks, rate);
            } else if (k == 1u) {
                vp.hpResonance = adjustLinear01(vp.hpResonance, ticks, rate, 0.0f, 1.0f);
            } else if (k == 2u) {
                vp.lpCutoffHz = adjustFilterCutoff(vp.lpCutoffHz, ticks, rate);
            } else {
                vp.lpResonance = adjustLinear01(vp.lpResonance, ticks, rate, 0.0f, 1.0f);
            }
        }
        break;
    case GlobalTarget::Lfo:
        if (k == 0u) {
            const int32_t curTgt = static_cast<int32_t>(state.voices[0].lfoTarget);
            const int32_t maxTgt = static_cast<int32_t>(LfoTarget::Count) - 1;
            const auto nextTgt = static_cast<LfoTarget>(
                stepClampedDetent(curTgt, maxTgt, ticks, rate, targetTickAccum));
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoTarget = nextTgt;
            }
        } else if (k == 1u) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoShape =
                    adjustLinear01(state.voices[i].lfoShape, ticks, rate, 0.0f, 1.0f);
            }
        } else if (k == 2u) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoRateHz = adjustLfoRate(state.voices[i].lfoRateHz, ticks, rate);
            }
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoDepth =
                    adjustLinear01(state.voices[i].lfoDepth, ticks, rate, 0.0f, 1.0f);
            }
        }
        break;
    case GlobalTarget::Amp:
        applyGlobalAmpKnob(k, ticks, rate, state);
        break;
    case GlobalTarget::All:
    case GlobalTarget::Count:
        if (k == 0u) {
            applyGlobalFreqKnob(0u, ticks, rate, state);
        } else if (k == 1u) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lpCutoffHz =
                    adjustFilterCutoff(state.voices[i].lpCutoffHz, ticks, rate);
            }
        } else if (k == 2u) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoDepth =
                    adjustLinear01(state.voices[i].lfoDepth, ticks, rate, 0.0f, 1.0f);
            }
        } else {
            applyGlobalAmpKnob(0u, ticks, rate, state);
        }
        break;
    }
}

GlobalColumnSummary
evaluateGlobalColumn(KnobBankMode bank, uint8_t knobIdx, const EngineControlState& control,
                     const std::array<float, kNumOscillators>& lfoOutput) noexcept {
    GlobalColumnSummary s{};
    const uint8_t k = knobIdx & 3u;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;

    if (bank == KnobBankMode::GlobalAll) {
        if (k == 0u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalFreq, 0u, control, lfoOutput);
        }
        if (k == 1u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalFilter, 2u, control, lfoOutput);
        }
        if (k == 2u) {
            return evaluateGlobalColumn(KnobBankMode::GlobalLfo, 3u, control, lfoOutput);
        }
        return evaluateGlobalColumn(KnobBankMode::GlobalAmp, 0u, control, lfoOutput);
    }

    if (bank == KnobBankMode::GlobalFreq) {
        std::array<float, kNumOscillators> logs{};
        float meanLog = 0.0f;
        float minLog = 100.0f;
        float maxLog = -100.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const VoiceParams& vp = control.voices[i];
            const float pMul = computeVoicePitchRatio(control.voicePressure[i], pitchMode);
            const float lfoHz = (vp.lfoTarget == LfoTarget::Pitch)
                                    ? applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoOutput[i])
                                    : vp.frequencyHz;
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
            const float hpBase = (vp.lfoTarget == LfoTarget::HpCutoff)
                                     ? applyLfoCutoffHz(vp.hpCutoffHz, vp.lfoDepth, lfoOutput[i])
                                     : vp.hpCutoffHz;
            const float hpResBase =
                (vp.lfoTarget == LfoTarget::HpResonance)
                    ? applyLfoResonance(vp.hpResonance, vp.lfoDepth, lfoOutput[i])
                    : vp.hpResonance;
            const float lpBase = (vp.lfoTarget == LfoTarget::LpCutoff)
                                     ? applyLfoCutoffHz(vp.lpCutoffHz, vp.lfoDepth, lfoOutput[i])
                                     : vp.lpCutoffHz;
            const float lpResBase =
                (vp.lfoTarget == LfoTarget::LpResonance)
                    ? applyLfoResonance(vp.lpResonance, vp.lfoDepth, lfoOutput[i])
                    : vp.lpResonance;
            const ModulatedFilterParams mod = computeVoicePressureMacro(
                hpBase, hpResBase, lpBase, lpResBase, control.voicePressure[i],
                vp.muted ? 0.0f : 1.0f, pitchMode);
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
        if (k == 0u) {
            const LfoTarget ref = control.voices[0].lfoTarget;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                if (control.voices[i].lfoTarget != ref) {
                    s.lfoTargetMixed = true;
                }
                const float vn =
                    static_cast<float>(static_cast<uint8_t>(control.voices[i].lfoTarget)) *
                    (1.0f / 6.0f);
                s.voiceSigned[i] = 2.0f * vn - 1.0f;
            }
            const float norm = static_cast<float>(static_cast<uint8_t>(ref)) * (1.0f / 6.0f);
            s.posSigned = 2.0f * norm - 1.0f;
            finalizeSpread(s);
        } else if (k == 1u) {
            float lo = control.voices[0].lfoShape;
            float hi = lo;
            float sum = 0.0f;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const float shp = clampf(control.voices[i].lfoShape, 0.0f, 1.0f);
                sum += shp;
                if (shp < lo) {
                    lo = shp;
                }
                if (shp > hi) {
                    hi = shp;
                }
                s.voiceSigned[i] = 2.0f * shp - 1.0f;
            }
            s.lfoShapeMixed = (hi - lo) > 0.05f;
            s.primaryValue = sum * 0.25f;
            s.posSigned = 2.0f * s.primaryValue - 1.0f;
            finalizeSpread(s);
        } else if (k == 2u) {
            float sumLog = 0.0f;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const float lg = fastLog2(control.voices[i].lfoRateHz);
                sumLog += lg;
                const float vn = clampf((lg - kGcLfoLogLo) / kGcLfoLogRange, 0.0f, 1.0f);
                s.voiceSigned[i] = 2.0f * vn - 1.0f;
            }
            sumLog *= 0.25f;
            s.primaryValue = fastExp2(sumLog);
            const float norm = clampf((sumLog - kGcLfoLogLo) / kGcLfoLogRange, 0.0f, 1.0f);
            s.posSigned = 2.0f * norm - 1.0f;
            finalizeSpread(s);
        } else {
            float sum = 0.0f;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const float d = clampf(control.voices[i].lfoDepth, 0.0f, 1.0f);
                sum += d;
                s.voiceSigned[i] = 2.0f * d - 1.0f;
            }
            s.primaryValue = sum * 0.25f;
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
        const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                  ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[i])
                                  : vp.amplitude;
        amps[i] = vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
        const float livePan = (vp.lfoTarget == LfoTarget::Pan)
                                  ? applyLfoPan(vp.pan, vp.lfoDepth, lfoOutput[i])
                                  : vp.pan;
        pans[i] = clampf(livePan * panScale, -1.0f, 1.0f);
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
        float sumSq = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            sumSq += pans[i] * pans[i];
            s.voiceSigned[i] = pans[i];
        }
        const float rms = (sumSq > 1.0e-8f) ? (sumSq * 0.25f * fastInvSqrt(sumSq * 0.25f)) : 0.0f;
        s.primaryValue = clampf(rms * (1.0f / kRefPanRms), 0.0f, 1.0f);
        s.posSigned = 2.0f * s.primaryValue - 1.0f;
    } else {
        s.bipolar = true;
        s.primaryValue = clampf(0.5f * ((amps[1] + amps[3]) - (amps[0] + amps[2])), -1.0f, 1.0f);
        s.posSigned = s.primaryValue;
    }
    return s;
}

} // namespace ruido
