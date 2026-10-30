#include "ui/HudGraphs.h"

#include "core/HudEnvelope.h"

#include <cstring>

namespace ruido {

void HudGraphs::reset() noexcept {
    for (auto& fc : filterCurvesFixed_) {
        fc.fill(static_cast<int16_t>(132 * 64));
    }
    for (auto& cc : filterCompCurvesFixed_) {
        cc.fill(static_cast<int16_t>(132 * 64));
    }
    filterCompWeights_ = {0.0f, 0.0f};
    filterDivergenceSmooth_ = 0.0f;
    filterHpBeadX_ = 10.0f;
    filterHpBeadY_ = 132.0f;
    filterHpBeadInt_ = 0.0f;
    filterHpBeadRad_ = 2.0f;
    filterLpBeadX_ = 230.0f;
    filterLpBeadY_ = 132.0f;
    filterLpBeadInt_ = 0.0f;
    filterLpBeadRad_ = 2.0f;
    filterGraphPhase_ = 0.0f;
    filterBackdropDim_ = 0.0f;

    const auto lfoZeroFixed = static_cast<int16_t>(kLfoYZero * 64.0f + 0.5f);
    for (auto& lc : lfoCurvesFixed_) {
        lc.fill(lfoZeroFixed);
    }
    lfoBeadIntensity_ = {0.0f, 0.0f, 0.0f, 0.0f};
    lfoDivergenceSmooth_ = 0.0f;
    lfoGraphPhase_ = 0.0f;
}

void HudGraphs::clearImmediate() noexcept {
    filterGraphPhase_ = 0.0f;
    filterBackdropDim_ = 0.0f;
    lfoGraphPhase_ = 0.0f;
}

void HudGraphs::stepFadeOut(float dtClamped) noexcept {
    filterGraphPhase_ = stepHudFadeLinear(filterGraphPhase_, false, dtClamped);
    filterBackdropDim_ = stepHudFadeLinear(filterBackdropDim_, false, dtClamped);
    lfoGraphPhase_ = stepHudFadeLinear(lfoGraphPhase_, false, dtClamped);
}

void HudGraphs::updateFilterGraph(float dtClamped, bool snapImmediate,
                                  const std::array<ModulatedFilterParams, kNumOscillators>& mods,
                                  bool unified) noexcept {
    std::array<std::array<int16_t, kFilterSweepPts>, kNumOscillators> tgtCurves{};
    std::array<std::array<int16_t, kFilterSweepPts>, 2> tgtComp{};
    std::array<float, 2> tgtCompWeights{0.0f, 0.0f};
    float tgtHpX = filterHpBeadX_;
    float tgtHpY = filterHpBeadY_;
    float tgtHpInt = 0.0f;
    float tgtHpRad = filterHpBeadRad_;
    float tgtLpX = filterLpBeadX_;
    float tgtLpY = filterLpBeadY_;
    float tgtLpInt = 0.0f;
    float tgtLpRad = filterLpBeadRad_;

    if (unified) {
        const DualMs20Coeffs mainC = makeDualMs20Coeffs(mods[0].hpCutoffHz, mods[0].hpResonance,
                                                        mods[0].lpCutoffHz, mods[0].lpResonance);
        const float resEnergy = 0.5f * (mainC.rHp * mainC.hpAct + mainC.rLp * mainC.lpAct);
        const DualMs20Coeffs comp1C = makeDualMs20Coeffs(
            mainC.clampedHp * 1.03f, mainC.rHp * 0.76f, mainC.clampedLp * 0.97f, mainC.rLp * 0.76f);
        const DualMs20Coeffs comp2C = makeDualMs20Coeffs(
            mainC.clampedHp * 1.07f, mainC.rHp * 0.54f, mainC.clampedLp * 0.93f, mainC.rLp * 0.54f);
        const float drop1Db = 1.1f + 1.8f * resEnergy;
        const float drop2Db = 2.3f + 3.0f * resEnergy;

        for (size_t i = 0u; i < kFilterSweepPts; ++i) {
            const float freq = kFilterSweepFreqs[i];
            const int16_t yMain = dbToFilterYFixed(evalDualMs20Db(freq, mainC));
            tgtCurves[0][i] = yMain;
            tgtCurves[1][i] = yMain;
            tgtCurves[2][i] = yMain;
            tgtCurves[3][i] = yMain;
            tgtComp[0][i] = dbToFilterYFixed(evalDualMs20Db(freq, comp1C) - drop1Db);
            tgtComp[1][i] = dbToFilterYFixed(evalDualMs20Db(freq, comp2C) - drop2Db);
        }
        tgtCompWeights[0] = 0.28f + 0.24f * resEnergy;
        tgtCompWeights[1] = 0.18f + 0.22f * resEnergy;

        if (mainC.hpAct > 0.02f) {
            const float uHp = clampf(
                (fastLog2(mainC.clampedHp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtHpX =
                static_cast<float>(kFilterXStart) + uHp * static_cast<float>(kFilterSweepPts - 1u);
            const float hpDb = evalDualMs20Db(mainC.clampedHp, mainC);
            tgtHpY = clampf(kFilterYZeroDb - hpDb * kFilterDbToPx, kFilterYTop, kFilterYBot);
            tgtHpInt = (0.78f + 0.22f * mainC.rHp) * mainC.hpAct;
            tgtHpRad = 2.0f + 1.35f * mainC.rHp;
        }
        if (mainC.lpAct > 0.02f) {
            const float uLp = clampf(
                (fastLog2(mainC.clampedLp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtLpX =
                static_cast<float>(kFilterXStart) + uLp * static_cast<float>(kFilterSweepPts - 1u);
            const float lpDb = evalDualMs20Db(mainC.clampedLp, mainC);
            tgtLpY = clampf(kFilterYZeroDb - lpDb * kFilterDbToPx, kFilterYTop, kFilterYBot);
            tgtLpInt = (0.78f + 0.22f * mainC.rLp) * mainC.lpAct;
            tgtLpRad = 2.0f + 1.35f * mainC.rLp;
        }
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const DualMs20Coeffs c = makeDualMs20Coeffs(mods[v].hpCutoffHz, mods[v].hpResonance,
                                                        mods[v].lpCutoffHz, mods[v].lpResonance);
            for (size_t i = 0u; i < kFilterSweepPts; ++i) {
                tgtCurves[v][i] = dbToFilterYFixed(evalDualMs20Db(kFilterSweepFreqs[i], c));
            }
        }
        tgtComp[0] = tgtCurves[0];
        tgtComp[1] = tgtCurves[0];
    }

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));
    const float alphaDiv = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.055f)));

    filterDivergenceSmooth_ =
        slewScalar(filterDivergenceSmooth_, unified ? 0.0f : 1.0f, alphaDiv, 0.001f);
    filterCompWeights_[0] = slewScalar(filterCompWeights_[0], tgtCompWeights[0], alphaCurve);
    filterCompWeights_[1] = slewScalar(filterCompWeights_[1], tgtCompWeights[1], alphaCurve);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        for (size_t i = 0u; i < kFilterSweepPts; ++i) {
            filterCurvesFixed_[v][i] =
                slewQ8_6(filterCurvesFixed_[v][i], tgtCurves[v][i], alphaCurve, snapImmediate);
        }
    }
    for (size_t c = 0u; c < 2u; ++c) {
        for (size_t i = 0u; i < kFilterSweepPts; ++i) {
            filterCompCurvesFixed_[c][i] =
                slewQ8_6(filterCompCurvesFixed_[c][i], tgtComp[c][i], alphaCurve, snapImmediate);
        }
    }

    if (tgtHpInt > 0.001f) {
        const bool snapPos = snapImmediate || (filterHpBeadInt_ <= 0.005f);
        filterHpBeadX_ = snapPos ? tgtHpX : slewScalar(filterHpBeadX_, tgtHpX, alphaCurve, 0.01f);
        filterHpBeadY_ = snapPos ? tgtHpY : slewScalar(filterHpBeadY_, tgtHpY, alphaCurve, 0.01f);
        filterHpBeadRad_ =
            snapPos ? tgtHpRad : slewScalar(filterHpBeadRad_, tgtHpRad, alphaCurve, 0.005f);
    }
    filterHpBeadInt_ =
        snapImmediate ? tgtHpInt : slewScalar(filterHpBeadInt_, tgtHpInt, alphaCurve, 0.002f);

    if (tgtLpInt > 0.001f) {
        const bool snapPos = snapImmediate || (filterLpBeadInt_ <= 0.005f);
        filterLpBeadX_ = snapPos ? tgtLpX : slewScalar(filterLpBeadX_, tgtLpX, alphaCurve, 0.01f);
        filterLpBeadY_ = snapPos ? tgtLpY : slewScalar(filterLpBeadY_, tgtLpY, alphaCurve, 0.01f);
        filterLpBeadRad_ =
            snapPos ? tgtLpRad : slewScalar(filterLpBeadRad_, tgtLpRad, alphaCurve, 0.005f);
    }
    filterLpBeadInt_ =
        snapImmediate ? tgtLpInt : slewScalar(filterLpBeadInt_, tgtLpInt, alphaCurve, 0.002f);
}

void HudGraphs::updateLfoGraph(
    float dtClamped, bool snapImmediate, bool showAllVoices, const EngineControlState& control,
    const HudState& effectiveHud,
    const std::array<uint32_t, kNumOscillators>& lfoDriftPacked) noexcept {
    std::array<std::array<int16_t, kTracePoints>, kNumOscillators> tgtCurves{};
    std::array<float, kNumOscillators> tgtBeadInt{0.0f, 0.0f, 0.0f, 0.0f};
    bool unified = true;

    constexpr float kInvPts = 1.0f / static_cast<float>(kTracePoints - 1u);
    static constexpr std::array<std::array<int8_t, 5>, kNumOscillators> kFallbackDriftKnots = {{
        {-18, 24, -12, 27, -21},
        {22, -19, 25, -14, 19},
        {-24, 16, -26, 21, -15},
        {17, -25, 14, -22, 26},
    }};

    if (!showAllVoices) {
        const uint8_t v = effectiveHud.lfoVoiceFocus & 3u;
        const VoiceParams& vp = control.voices[v];
        const float d = clampf(vp.lfoDepth, 0.0f, 1.0f);
        const float shapedDepth = d * (1.35f - 0.35f * d);
        const float ampPx = kLfoMaxAmpPx * (0.46f + 0.54f * shapedDepth);
        const std::array<int8_t, 5> knots = (lfoDriftPacked[v] != 0u)
                                                ? unpackDriftKnots(lfoDriftPacked[v])
                                                : kFallbackDriftKnots[v];

        for (size_t i = 0u; i < kTracePoints; ++i) {
            const float u = static_cast<float>(i) * kInvPts;
            const float w = evalMorphedLfoWave(u, vp.lfoShape, vp.lfoRateHz, knots);
            const float yMain = clampf(kLfoYZero - ampPx * w, kLfoYTop, kLfoYBot);
            const auto qMain = static_cast<int16_t>(yMain * 64.0f + 0.5f);
            tgtCurves[0][i] = qMain;
            tgtCurves[1][i] = qMain;
            tgtCurves[2][i] = qMain;
            tgtCurves[3][i] = qMain;
        }
        tgtBeadInt[v] = 0.88f + 0.12f * smoothstep01(d * 3.0f);
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const VoiceParams& vp = control.voices[v];
            const float d = clampf(vp.lfoDepth, 0.0f, 1.0f);
            const float shapedDepth = d * (1.35f - 0.35f * d);
            const float ampPx = kLfoMaxAmpPx * (0.46f + 0.54f * shapedDepth);
            const std::array<int8_t, 5> knots = (lfoDriftPacked[v] != 0u)
                                                    ? unpackDriftKnots(lfoDriftPacked[v])
                                                    : kFallbackDriftKnots[v];
            for (size_t i = 0u; i < kTracePoints; ++i) {
                const float u = static_cast<float>(i) * kInvPts;
                const float w = evalMorphedLfoWave(u, vp.lfoShape, vp.lfoRateHz, knots);
                const float yMain = clampf(kLfoYZero - ampPx * w, kLfoYTop, kLfoYBot);
                tgtCurves[v][i] = static_cast<int16_t>(yMain * 64.0f + 0.5f);
            }
            tgtBeadInt[v] = 0.76f + 0.16f * smoothstep01(d * 3.0f);
        }
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            for (size_t i = 0u; i < kTracePoints; i += 8u) {
                if (tgtCurves[v][i] - tgtCurves[0][i] > 16 ||
                    tgtCurves[0][i] - tgtCurves[v][i] > 16) {
                    unified = false;
                    break;
                }
            }
            if (!unified) {
                break;
            }
        }
    }

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));
    const float alphaDiv = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.055f)));

    lfoDivergenceSmooth_ =
        slewScalar(lfoDivergenceSmooth_, unified ? 0.0f : 1.0f, alphaDiv, 0.001f);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        for (size_t i = 0u; i < kTracePoints; ++i) {
            lfoCurvesFixed_[v][i] =
                slewQ8_6(lfoCurvesFixed_[v][i], tgtCurves[v][i], alphaCurve, snapImmediate);
        }
        lfoBeadIntensity_[v] =
            snapImmediate ? tgtBeadInt[v]
                          : slewScalar(lfoBeadIntensity_[v], tgtBeadInt[v], alphaCurve, 0.002f);
    }
}

void HudGraphs::update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                       const HudState& effectiveHud, bool extraCenterViewActive,
                       const std::array<float, kNumOscillators>& lfoOutput,
                       const std::array<uint32_t, kNumOscillators>& lfoDriftPacked) noexcept {
    const bool hudVisible = (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None);
    const bool rawFilterActive =
        hudVisible &&
        (rawHud.knobBank == KnobBankMode::VoiceFilter ||
         rawHud.knobBank == KnobBankMode::GlobalFilter ||
         rawHud.knobBank == KnobBankMode::GlobalAll ||
         (rawHud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode));
    const bool rawLfoGraphActive = !rawFilterActive && hudVisible &&
                                   ((rawHud.lfoPageOpen && rawHud.knobBank == KnobBankMode::Lfo) ||
                                    rawHud.knobBank == KnobBankMode::GlobalLfo);

    if (rawFilterActive) {
        const bool snapFilter = (filterGraphPhase_ <= 0.015f);
        filterGraphPhase_ = stepHudFadeLinear(filterGraphPhase_, true, dtClamped);

        const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
        std::array<ModulatedFilterParams, kNumOscillators> mods{};
        bool unified = true;

        if (rawHud.knobBank == KnobBankMode::VoiceFilter ||
            (rawHud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode)) {
            const uint8_t v = rawHud.focusedVoice & 3u;
            const VoiceParams& vp = control.voices[v];
            const ModulatedFilterParams mod =
                computeLiveVoiceFilter(vp, control.voicePressure[v], pitchMode, lfoOutput[v]);
            mods = {mod, mod, mod, mod};
            unified = true;
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                mods[i] =
                    computeLiveVoiceFilter(vp, control.voicePressure[i], pitchMode, lfoOutput[i]);
            }
            for (size_t i = 1u; i < kNumOscillators; ++i) {
                if (absf(mods[i].hpCutoffHz - mods[0].hpCutoffHz) >
                        0.005f * mods[0].hpCutoffHz + 0.05f ||
                    absf(mods[i].lpCutoffHz - mods[0].lpCutoffHz) >
                        0.005f * mods[0].lpCutoffHz + 0.05f ||
                    absf(mods[i].hpResonance - mods[0].hpResonance) > 0.005f ||
                    absf(mods[i].lpResonance - mods[0].lpResonance) > 0.005f) {
                    unified = false;
                    break;
                }
            }
        }
        updateFilterGraph(dtClamped, snapFilter, mods, unified);
    } else {
        filterGraphPhase_ = stepHudFadeLinear(filterGraphPhase_, false, dtClamped);
    }

    if (rawLfoGraphActive) {
        const bool snapLfo = (lfoGraphPhase_ <= 0.015f);
        lfoGraphPhase_ = stepHudFadeLinear(lfoGraphPhase_, true, dtClamped);
        const bool showAll = (rawHud.knobBank == KnobBankMode::GlobalLfo);
        updateLfoGraph(dtClamped, snapLfo, showAll, control, effectiveHud, lfoDriftPacked);
    } else {
        lfoGraphPhase_ = stepHudFadeLinear(lfoGraphPhase_, false, dtClamped);
    }

    const bool wantBackdrop = rawFilterActive || rawLfoGraphActive || extraCenterViewActive;
    filterBackdropDim_ = stepHudFadeLinear(filterBackdropDim_, wantBackdrop, dtClamped);
}

void HudGraphs::compositeLfoStrip(
    int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf, uint16_t* linScratch,
    float effLfoAlpha01, uint8_t lfoAlpha8, const EngineControlState& control, const HudState& hud,
    const std::array<float, kNumOscillators>& lfoPhase) const noexcept {
    if (lfoAlpha8 == 0u || y1 <= 12 || y0 >= 206) {
        return;
    }
    constexpr float kFixedToPx = 1.0f / 64.0f;
    const MonoStripCanvas lfoCv{lumBuf, y0, y1, lfoAlpha8};
    const float div = clampf(lfoDivergenceSmooth_, 0.0f, 1.0f);
    const bool multiCurves = (div > 0.001f);
    const size_t curveCount = multiCurves ? kNumOscillators : 1u;

    float maxD = 0.0f;
    if (hud.knobBank == KnobBankMode::GlobalLfo || hud.mode == HudMode::GlobalLfo) {
        for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
            if (control.voices[vi].lfoDepth > maxD) {
                maxD = control.voices[vi].lfoDepth;
            }
        }
    } else {
        maxD = control.voices[hud.lfoVoiceFocus & 3u].lfoDepth;
    }
    const float lfoEnergy = smoothstep01(clampf(maxD, 0.0f, 1.0f) * 3.0f);
    const float spineModeScale = 0.58f + 0.42f * lfoEnergy;
    const float fillModeScale = 0.34f + 0.66f * lfoEnergy;

    lfoCv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), kLfoYZero, 38.0f,
                                 0.50f, false);

    const float fillScale = (multiCurves ? (0.24f + 0.05f * div) : 1.0f) * fillModeScale;
    if (y1 > static_cast<int>(kLfoYTop) - 1 && y0 <= static_cast<int>(kLfoYBot) + 1) {
        for (size_t vc = 0u; vc < curveCount; ++vc) {
            const auto& pts = lfoCurvesFixed_[vc];
            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                const float yA = static_cast<float>(pts[colX]) * kFixedToPx;
                const float yB = static_cast<float>(pts[colX + 1u]) * kFixedToPx;
                const float dy = yB - yA;
                const float absDy = absf(dy);
                const float yMid = 0.5f * (yA + yB);

                float envLo = (yA < yB) ? yA : yB;
                if (kLfoYZero < envLo) {
                    envLo = kLfoYZero;
                }
                float envHi = (yA > yB) ? yA : yB;
                if (kLfoYZero > envHi) {
                    envHi = kLfoYZero;
                }
                if (envHi - envLo <= 0.08f) {
                    continue;
                }

                int pyStart = static_cast<int>(envLo - 0.5f);
                if (pyStart < static_cast<int>(kLfoYTop) - 1) {
                    pyStart = static_cast<int>(kLfoYTop) - 1;
                }
                if (pyStart < y0) {
                    pyStart = y0;
                }
                int pyEnd = static_cast<int>(envHi + 0.5f) + 1;
                if (pyEnd > static_cast<int>(kLfoYBot) + 1) {
                    pyEnd = static_cast<int>(kLfoYBot) + 1;
                }
                if (pyEnd > y1 - 1) {
                    pyEnd = y1 - 1;
                }
                if (pyStart > pyEnd) {
                    continue;
                }

                const float spanA = absf(kLfoYZero - yA);
                const float spanB = absf(kLfoYZero - yB);
                const float maxSpan = (spanA > spanB) ? spanA : spanB;
                const float crestBoost =
                    1.0f + 0.34f * clampf(maxSpan * (1.0f / kLfoMaxAmpPx), 0.0f, 1.0f);
                const float colScale = fillScale * effLfoAlpha01;
                const float glowAmp = 68.0f * crestBoost;
                const float invL2 = 1.0f / (1.0f + dy * dy);
                const bool steepSeg = (absDy > 1.5f);

                for (int py = pyStart; py <= pyEnd; ++py) {
                    const float pyF = static_cast<float>(py);
                    const float boxLo = pyF - 0.5f;
                    const float boxHi = pyF + 0.5f;

                    float subEdge = 0.0f;
                    if (!steepSeg) {
                        const float segLo = (yMid < kLfoYZero) ? yMid : kLfoYZero;
                        const float segHi = (yMid > kLfoYZero) ? yMid : kLfoYZero;
                        const float ovLo = (boxLo > segLo) ? boxLo : segLo;
                        const float ovHi = (boxHi < segHi) ? boxHi : segHi;
                        if (ovHi > ovLo) {
                            subEdge = ovHi - ovLo;
                        }
                    } else {
                        for (int s = 0; s < 4; ++s) {
                            const float u = (static_cast<float>(s) + 0.5f) * 0.25f;
                            const float ySub = yA + u * dy;
                            const float segLo = (ySub < kLfoYZero) ? ySub : kLfoYZero;
                            const float segHi = (ySub > kLfoYZero) ? ySub : kLfoYZero;
                            const float ovLo = (boxLo > segLo) ? boxLo : segLo;
                            const float ovHi = (boxHi < segHi) ? boxHi : segHi;
                            if (ovHi > ovLo) {
                                subEdge += (ovHi - ovLo) * 0.25f;
                            }
                        }
                    }
                    if (subEdge <= 0.004f) {
                        continue;
                    }

                    const float vy = pyF - yMid;
                    const float segT = clampf(vy * dy * invL2, -0.5f, 0.5f);
                    const float ey = vy - dy * segT;
                    const float d2 = segT * segT + ey * ey;
                    const float perpDist = d2 * fastInvSqrt(d2 + 0.25f);
                    const float crestGlow = glowAmp * fastExpNeg(perpDist * 0.052f);
                    int alpha = static_cast<int>((18.0f + crestGlow) * colScale * subEdge + 0.5f);
                    if (alpha > 215) {
                        alpha = 215;
                    }
                    if (alpha > 0) {
                        uint8_t& dst = lumBuf[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
                        dst = blendLum8(dst, 240u, static_cast<uint8_t>(alpha));
                    }
                }
            }
        }
    }

    if (y1 > static_cast<int>(kLfoYTop) - 4 && y0 <= static_cast<int>(kLfoYBot) + 4) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float mainCurveWgt =
            (multiCurves ? (0.28f + 0.28f * div) : 1.04f) * spineModeScale * effLfoAlpha01;
        if (mainCurveWgt > 0.004f) {
            for (size_t vc = 0u; vc < curveCount; ++vc) {
                const auto& pts = lfoCurvesFixed_[vc];
                for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                    splatPhotonSegment(static_cast<int>(colX),
                                       static_cast<float>(pts[colX]) * kFixedToPx,
                                       static_cast<float>(pts[colX + 1u]) * kFixedToPx,
                                       mainCurveWgt, 0.64f, y0, y1, linScratch);
                }
            }
        }
        screenBlendLinearStrip(stripHeight, 0u, kDisplayWidth, linScratch, lumBuf);
    }

    for (size_t bv = 0u; bv < kNumOscillators; ++bv) {
        const float lfoVis = lfoBeadIntensity_[bv];
        if (lfoVis <= 0.005f) {
            continue;
        }
        const float ph = clampf(lfoPhase[bv], 0.0f, 0.999999f);
        const float uIdx = ph * static_cast<float>(kDisplayWidth);
        const size_t i0 = (static_cast<size_t>(uIdx) < kDisplayWidth) ? static_cast<size_t>(uIdx)
                                                                      : (kDisplayWidth - 1u);
        const size_t i1 = i0 + 1u;
        const float frac = uIdx - static_cast<float>(i0);
        const size_t curveIdx = multiCurves ? bv : 0u;
        const float y0Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i0]) * kFixedToPx;
        const float y1Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i1]) * kFixedToPx;
        const float beadX = clampf(uIdx, 0.0f, static_cast<float>(kDisplayWidth - 1u));
        const float beadY = y0Px + frac * (y1Px - y0Px);
        lfoCv.drawLfoPhaseBeacon(beadX, beadY, lfoVis, control.voices[bv].lfoDepth);
    }
}

void HudGraphs::compositeFilterStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                     uint16_t* linScratch, float effFilterAlpha01,
                                     uint8_t filterAlpha8) const noexcept {
    if (filterAlpha8 == 0u || y1 <= 94 || y0 >= 189) {
        return;
    }
    constexpr float kFixedToPx = 1.0f / 64.0f;
    const MonoStripCanvas filterCv{lumBuf, y0, y1, filterAlpha8};
    const float div = clampf(filterDivergenceSmooth_, 0.0f, 1.0f);
    const bool multiCurves = (div > 0.001f);
    const size_t curveCount = multiCurves ? kNumOscillators : 1u;

    filterCv.drawHorizontalFilament(
        static_cast<float>(kFilterXStart),
        static_cast<float>(kFilterXStart + static_cast<int>(kFilterSweepPts) - 1), kFilterYBot,
        28.0f, 0.48f, true);

    const float fillScale = multiCurves ? (0.25f + 0.05f * div) : 1.0f;
    if (y1 > static_cast<int>(kFilterYTop) && y0 <= static_cast<int>(kFilterYBot)) {
        for (size_t vc = 0u; vc < curveCount; ++vc) {
            const auto& pts = filterCurvesFixed_[vc];
            for (size_t i = 0u; i < kFilterSweepPts; ++i) {
                const int px = kFilterXStart + static_cast<int>(i);
                const size_t iNext = (i + 1u < kFilterSweepPts) ? (i + 1u) : i;
                const float yCurve = 0.5f *
                                     (static_cast<float>(pts[i]) + static_cast<float>(pts[iNext])) *
                                     kFixedToPx;
                if (kFilterYBot - yCurve <= 0.35f) {
                    continue;
                }
                int pyStart = static_cast<int>(yCurve);
                if (pyStart < static_cast<int>(kFilterYTop)) {
                    pyStart = static_cast<int>(kFilterYTop);
                }
                if (pyStart < y0) {
                    pyStart = y0;
                }
                int pyEnd = static_cast<int>(kFilterYBot);
                if (pyEnd > y1 - 1) {
                    pyEnd = y1 - 1;
                }
                if (pyStart > pyEnd) {
                    continue;
                }

                const int distL = static_cast<int>(i);
                const int distR = static_cast<int>(kFilterSweepPts - 1u - i);
                const int edgeDist = (distL < distR) ? distL : distR;
                const float edgeTaper =
                    0.65f + 0.35f * smoothstep01(static_cast<float>(edgeDist) * (1.0f / 6.0f));
                const float hornBoost =
                    1.0f + 0.40f * clampf((kFilterYZeroDb - yCurve) * (1.0f / 28.0f), 0.0f, 1.0f);
                const float colScale = fillScale * edgeTaper * effFilterAlpha01;
                const float glowAmp = 76.0f * hornBoost;

                for (int py = pyStart; py <= pyEnd; ++py) {
                    const float pyF = static_cast<float>(py);
                    const float subEdge =
                        (pyF < yCurve) ? clampf(1.0f - (yCurve - pyF), 0.0f, 1.0f) : 1.0f;
                    const float dist = pyF - yCurve;
                    const float crestGlow =
                        glowAmp * fastExpNeg(((dist > 0.0f) ? dist : 0.0f) * 0.095f);
                    int alpha = static_cast<int>((26.0f + crestGlow) * colScale * subEdge + 0.5f);
                    if (alpha > 220) {
                        alpha = 220;
                    }
                    if (alpha > 0) {
                        uint8_t& dst = lumBuf[static_cast<size_t>(py - y0) * kDisplayWidth +
                                              static_cast<size_t>(px)];
                        dst = blendLum8(dst, 240u, static_cast<uint8_t>(alpha));
                    }
                }
            }
        }
    }

    std::memset(linScratch, 0, static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
    auto addCurvePhoton = [&](const std::array<int16_t, kFilterSweepPts>& pts,
                              float weight) noexcept {
        if (weight <= 0.004f) {
            return;
        }
        for (size_t i = 0u; i < kFilterSweepPts; ++i) {
            const int px = kFilterXStart + static_cast<int>(i);
            const size_t iNext = (i + 1u < kFilterSweepPts) ? (i + 1u) : i;
            splatPhotonSegment(px, static_cast<float>(pts[i]) * kFixedToPx,
                               static_cast<float>(pts[iNext]) * kFixedToPx, weight, 0.62f, y0, y1,
                               linScratch);
        }
    };

    const float compVis = 1.0f - div;
    if (compVis > 0.004f) {
        addCurvePhoton(filterCompCurvesFixed_[1],
                       filterCompWeights_[1] * compVis * effFilterAlpha01);
        addCurvePhoton(filterCompCurvesFixed_[0],
                       filterCompWeights_[0] * compVis * effFilterAlpha01);
    }
    const float mainCurveWgt = (multiCurves ? (0.245f + 0.275f * div) : 0.98f) * effFilterAlpha01;
    for (size_t vc = 0u; vc < curveCount; ++vc) {
        addCurvePhoton(filterCurvesFixed_[vc], mainCurveWgt);
    }

    screenBlendLinearStrip(stripHeight, static_cast<size_t>(kFilterXStart),
                           static_cast<size_t>(kFilterXStart) + kFilterSweepPts, linScratch,
                           lumBuf);

    const float hpBeadVis = filterHpBeadInt_ * compVis;
    if (hpBeadVis > 0.005f) {
        filterCv.drawOpticalBead(filterHpBeadX_, filterHpBeadY_, hpBeadVis, filterHpBeadRad_);
    }
    const float lpBeadVis = filterLpBeadInt_ * compVis;
    if (lpBeadVis > 0.005f) {
        filterCv.drawOpticalBead(filterLpBeadX_, filterLpBeadY_, lpBeadVis, filterLpBeadRad_);
    }
}

} // namespace ruido
