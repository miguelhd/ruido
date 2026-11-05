#include "core/GlobalControl.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"

namespace ruido {

namespace {

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
                                 : (rate == KnobRate::Coarse) ? 0.14500f
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
                              : (rate == KnobRate::Coarse) ? 0.04500f
                                                           : 0.01500f;
        float maxScale = 6.0f;
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
        float meanAmp = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            meanAmp += state.voices[i].amplitude;
        }
        meanAmp *= 0.25f;
        std::array<float, kNumOscillators> dev{};
        float sumSq = 0.0f;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            dev[i] = state.voices[i].amplitude - meanAmp;
            sumSq += dev[i] * dev[i];
        }
        const float rms = (sumSq > 1.0e-8f) ? (sumSq * 0.25f * fastInvSqrt(sumSq * 0.25f)) : 0.0f;
        const float curSpread = clampf(rms * (1.0f / kRefAmpSpreadRms), 0.0f, 1.0f);
        const float nextSpread = adjustLinear01(curSpread, ticks, rate, 0.0f, 1.0f);
        std::array<float, kNumOscillators> unit{-1.3376206f, -0.4547910f, 0.4547910f, 1.3376206f};
        if (rms > 0.002f) {
            const float invRms = 1.0f / rms;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                unit[i] = dev[i] * invRms;
            }
        } else if (ticks <= 0) {
            return;
        }
        const float nextRms = nextSpread * kRefAmpSpreadRms;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            state.voices[i].amplitude = clampf(meanAmp + unit[i] * nextRms, 0.0f, 1.0f);
        }
        return;
    }

    if (state.global.monoCollapse) {
        return;
    }
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        state.voices[i].pan = adjustLinear01(state.voices[i].pan, ticks, rate, -1.0f, 1.0f);
    }
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
            LfoTarget refTgt = state.voices[0].lfoTarget;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                if (state.voices[i].lfoDepth > 1.0e-4f) {
                    refTgt = state.voices[i].lfoTarget;
                    break;
                }
                if (state.voices[i].lfoBDepth > 1.0e-4f &&
                    state.voices[i].lfoBTarget < LfoTarget::LfoARate) {
                    refTgt = state.voices[i].lfoBTarget;
                    break;
                }
            }
            const int32_t curTgt = static_cast<int32_t>(refTgt);
            const int32_t maxTgt = static_cast<int32_t>(kNumLfoATargets) - 1;
            const auto nextTgt = static_cast<LfoTarget>(
                stepClampedDetent(curTgt, maxTgt, ticks, rate, targetTickAccum));
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoTarget = nextTgt;
                if (state.voices[i].lfoBTarget < LfoTarget::LfoARate) {
                    state.voices[i].lfoBTarget = nextTgt;
                }
            }
        } else if (k == 1u) {
            bool anyActive = false;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                if (state.voices[i].lfoDepth > 1.0e-4f || state.voices[i].lfoBDepth > 1.0e-4f) {
                    anyActive = true;
                    break;
                }
            }
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                VoiceParams& vp = state.voices[i];
                if (!anyActive || vp.lfoDepth > 1.0e-4f) {
                    vp.lfoShape = adjustLinear01(vp.lfoShape, ticks, rate, 0.0f, 1.0f);
                }
                if (!anyActive || vp.lfoBDepth > 1.0e-4f) {
                    vp.lfoBShape = adjustLinear01(vp.lfoBShape, ticks, rate, 0.0f, 1.0f);
                }
            }
        } else if (k == 2u) {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                state.voices[i].lfoRateHz = adjustLfoRate(state.voices[i].lfoRateHz, ticks, rate);
                state.voices[i].lfoBRateHz = adjustLfoRate(state.voices[i].lfoBRateHz, ticks, rate);
            }
        } else {
            bool anyActive = false;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                if (state.voices[i].lfoDepth > 1.0e-4f || state.voices[i].lfoBDepth > 1.0e-4f) {
                    anyActive = true;
                    break;
                }
            }
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                VoiceParams& vp = state.voices[i];
                if (anyActive) {
                    if (vp.lfoDepth > 1.0e-4f) {
                        vp.lfoDepth = adjustLinear01(vp.lfoDepth, ticks, rate, 0.0f, 1.0f);
                    }
                    if (vp.lfoBDepth > 1.0e-4f) {
                        vp.lfoBDepth = adjustLinear01(vp.lfoBDepth, ticks, rate, 0.0f, 1.0f);
                    }
                } else if (ticks > 0) {
                    vp.lfoDepth = adjustLinear01(vp.lfoDepth, ticks, rate, 0.0f, 1.0f);
                }
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
            bool anyActive = false;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                if (state.voices[i].lfoDepth > 1.0e-4f || state.voices[i].lfoBDepth > 1.0e-4f) {
                    anyActive = true;
                    break;
                }
            }
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                VoiceParams& vp = state.voices[i];
                if (anyActive) {
                    if (vp.lfoDepth > 1.0e-4f) {
                        vp.lfoDepth = adjustLinear01(vp.lfoDepth, ticks, rate, 0.0f, 1.0f);
                    }
                    if (vp.lfoBDepth > 1.0e-4f) {
                        vp.lfoBDepth = adjustLinear01(vp.lfoBDepth, ticks, rate, 0.0f, 1.0f);
                    }
                } else if (ticks > 0) {
                    vp.lfoDepth = adjustLinear01(vp.lfoDepth, ticks, rate, 0.0f, 1.0f);
                }
            }
        } else {
            applyGlobalAmpKnob(0u, ticks, rate, state);
        }
        break;
    }
}

} // namespace ruido
