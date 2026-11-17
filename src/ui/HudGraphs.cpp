#include "ui/HudGraphs.h"

#include "core/HudEnvelope.h"
#include "core/RhythmMath.h"

#include <cstring>

namespace ruido {

void HudGraphs::reset() noexcept {
    const auto fltZeroFixed = static_cast<int16_t>(kFilterYZeroDb * 64.0f + 0.5f);
    for (auto& fc : filterCurvesFixed_) {
        fc.fill(fltZeroFixed);
    }
    filterDivergenceSmooth_ = 0.0f;
    filterHpBeadX_ = 0.0f;
    filterHpBeadY_ = kFilterYZeroDb;
    filterHpBeadInt_ = 0.0f;
    filterHpBeadRad_ = 2.0f;
    filterLpBeadX_ = static_cast<float>(kDisplayWidth - 1u);
    filterLpBeadY_ = kFilterYZeroDb;
    filterLpBeadInt_ = 0.0f;
    filterLpBeadRad_ = 2.0f;
    filterGraphPhase_ = 0.0f;
    filterBackdropDim_ = 0.0f;

    const auto lfoZeroFixed = static_cast<int16_t>(kLfoYZero * 64.0f + 0.5f);
    for (auto& lc : lfoCurvesFixed_) {
        lc.fill(lfoZeroFixed);
    }
    lfoBeadIntensity_ = 0.0f;
    lfoBeadBankIdx_ = {0u, 4u};
    lfoSiblingWeight_ = 0.0f;
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
    float tgtHpX = filterHpBeadX_;
    float tgtHpY = filterHpBeadY_;
    float tgtHpInt = 0.0f;
    float tgtHpRad = filterHpBeadRad_;
    float tgtLpX = filterLpBeadX_;
    float tgtLpY = filterLpBeadY_;
    float tgtLpInt = 0.0f;
    float tgtLpRad = filterLpBeadRad_;

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));
    const float alphaDiv = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.055f)));
    filterDivergenceSmooth_ =
        slewScalar(filterDivergenceSmooth_, unified ? 0.0f : 1.0f, alphaDiv, 0.001f);

    if (unified) {
        const DualMs20Coeffs mainC = makeDualMs20Coeffs(mods[0].hpCutoffHz, mods[0].hpResonance,
                                                        mods[0].lpCutoffHz, mods[0].lpResonance);

        for (size_t i = 0u; i < kFilterSweepPts; ++i) {
            const float freq = kFilterSweepFreqs[i];
            const int16_t yMain = dbToFilterYFixed(evalDualMs20Db(freq, mainC));
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                filterCurvesFixed_[v][i] =
                    slewQ8_6(filterCurvesFixed_[v][i], yMain, alphaCurve, snapImmediate);
            }
        }

        if (mainC.hpAct > 0.02f) {
            const float uHp = clampf(
                (fastLog2(mainC.clampedHp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtHpX = filterNormToScreenX(uHp);
            const float hpDb = evalDualMs20Db(mainC.clampedHp, mainC);
            tgtHpY = clampf(kFilterYZeroDb - hpDb * kFilterDbToPx, kFilterYTop, kFilterYBot);
            tgtHpInt = (0.78f + 0.22f * mainC.rHp) * mainC.hpAct;
            tgtHpRad = 2.0f + 1.35f * mainC.rHp;
        }
        if (mainC.lpAct > 0.02f) {
            const float uLp = clampf(
                (fastLog2(mainC.clampedLp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtLpX = filterNormToScreenX(uLp);
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
                const int16_t yTgt = dbToFilterYFixed(evalDualMs20Db(kFilterSweepFreqs[i], c));
                filterCurvesFixed_[v][i] =
                    slewQ8_6(filterCurvesFixed_[v][i], yTgt, alphaCurve, snapImmediate);
            }
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

void HudGraphs::updateLfoGraph(float dtClamped, bool snapImmediate,
                               const EngineControlState& control, const HudState& effectiveHud,
                               const LfoSignalArray& lfoOutput,
                               const LfoDriftArray& lfoDriftPacked) noexcept {
    std::array<int16_t, kTracePoints> curve0{};
    std::array<int16_t, kTracePoints> curveN{};
    float tgtBeadInt = 0.0f;
    float tgtSibWeight = 0.0f;

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));

    constexpr float kInvPts = 1.0f / static_cast<float>(kTracePoints - 1u);
    static constexpr std::array<std::array<int8_t, 5>, kNumOscillators> kFallbackDriftKnots = {{
        {-18, 24, -12, 27, -21},
        {22, -19, 25, -14, 19},
        {-24, 16, -26, 21, -15},
        {17, -25, 14, -22, 26},
    }};
    const auto buildSlotCurve = [&](size_t voiceIdx, size_t subIdx,
                                    std::array<int16_t, kTracePoints>& dst,
                                    float& outLiveDepth) noexcept {
        const VoiceParams& vp = control.voices[voiceIdx & 3u];
        const float lfoBRaw = lfoOutput[lfoBankIndex(voiceIdx, 1u)];
        const float d =
            clampf((subIdx == 0u) ? computeLiveLfoADepth(vp, lfoBRaw) : vp.lfoBDepth, 0.0f, 1.0f);
        const float r = (subIdx == 0u) ? computeLiveLfoARateHz(vp, lfoBRaw) : vp.lfoBRateHz;
        const float sh = lfoSlotShape(vp, subIdx);
        const float shapedDepth = d * (1.35f - 0.35f * d);
        const float ampPx = kLfoMaxAmpPx * (0.46f + 0.54f * shapedDepth);
        const size_t bankIdx = lfoBankIndex(voiceIdx, subIdx);
        const std::array<int8_t, 5> knots = (lfoDriftPacked[bankIdx] != 0u)
                                                ? unpackDriftKnots(lfoDriftPacked[bankIdx])
                                                : kFallbackDriftKnots[bankIdx & 3u];
        for (size_t i = 0u; i < kTracePoints; ++i) {
            const float u = static_cast<float>(i) * kInvPts;
            const float w = evalMorphedLfoWave(u, sh, r, knots);
            const float yMain = clampf(kLfoYZero - ampPx * w, kLfoYTop, kLfoYBot);
            dst[i] = static_cast<int16_t>(yMain * 64.0f + 0.5f);
        }
        outLiveDepth = d;
    };
    const auto slewVoiceCurve = [&](size_t v, const std::array<int16_t, kTracePoints>& tgt) {
        for (size_t i = 0u; i < kTracePoints; ++i) {
            lfoCurvesFixed_[v][i] =
                slewQ8_6(lfoCurvesFixed_[v][i], tgt[i], alphaCurve, snapImmediate);
        }
    };

    const uint8_t slot = effectiveHud.lfoVoiceFocus & 7u;
    const uint8_t v = lfoFocusVoice(slot);
    const uint8_t sub = lfoFocusSub(slot);
    const uint8_t sibSub = static_cast<uint8_t>(sub ^ 1u);
    float dMain = 0.0f;
    float dSib = 0.0f;
    buildSlotCurve(v, sub, curve0, dMain);
    buildSlotCurve(v, sibSub, curveN, dSib);
    slewVoiceCurve(0u, curve0);
    slewVoiceCurve(1u, curveN);

    lfoBeadBankIdx_[0] = static_cast<uint8_t>(lfoBankIndex(v, sub));
    lfoBeadBankIdx_[1] = static_cast<uint8_t>(lfoBankIndex(v, sibSub));
    tgtBeadInt = control.global.masterRun ? (0.88f + 0.12f * smoothstep01(dMain * 3.0f)) : 0.0f;
    tgtSibWeight = (lfoSlotDepth(control.voices[v], sibSub) > 0.002f) ? 0.34f : 0.0f;

    lfoSiblingWeight_ = snapImmediate
                            ? tgtSibWeight
                            : slewScalar(lfoSiblingWeight_, tgtSibWeight, alphaCurve, 0.002f);
    lfoBeadIntensity_ =
        snapImmediate ? tgtBeadInt : slewScalar(lfoBeadIntensity_, tgtBeadInt, alphaCurve, 0.002f);
}

void HudGraphs::update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                       const HudState& effectiveHud, bool extraCenterViewActive,
                       const LfoSignalArray& lfoOutput, const LfoDriftArray& lfoDriftPacked,
                       float rhythmPhase01, uint8_t stutterGateMask) noexcept {
    const bool hudVisible =
        (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None) && !rawHud.shiftHeld;
    const bool rawFilterActive =
        hudVisible &&
        (rawHud.knobBank == KnobBankMode::VoiceFilter ||
         rawHud.knobBank == KnobBankMode::GlobalFilter ||
         (rawHud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode));
    const bool rawLfoGraphActive = !rawFilterActive && hudVisible && rawHud.lfoPageOpen &&
                                   rawHud.knobBank == KnobBankMode::Lfo;

    if (rawFilterActive) {
        const bool snapFilter = (filterGraphPhase_ <= 0.015f);
        filterGraphPhase_ = stepHudFadeLinear(filterGraphPhase_, true, dtClamped);

        const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
        std::array<ModulatedFilterParams, kNumOscillators> mods{};
        bool unified = true;

        if (rawHud.knobBank == KnobBankMode::VoiceFilter ||
            (rawHud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode)) {
            const uint8_t v =
                (rawHud.mode == HudMode::KeyboardPlay &&
                 rawHud.activePunch == KeyPunchKind::VoiceFormant)
                    ? (rawHud.activePunchVoice & 3u)
                : (rawHud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode)
                    ? (rawHud.focusedVoice & 3u)
                    : (effectiveHud.fxVoiceFocus & 3u);
            const VoiceParams& vp = control.voices[v];
            const float pluckEnv =
                control.global.masterRun
                    ? evalVoicePluckPreviewEnv01(control.keyMod.rhythmPluck,
                                                 control.keyMod.rhythmMode[v],
                                                 control.keyMod.rhythmParam[v], v, rhythmPhase01,
                                                 ((stutterGateMask >> v) & 1u) != 0u)
                    : 0.0f;
            const ModulatedFilterParams mod = computeLiveVoiceFilter(
                vp, control.voicePressure[v], pitchMode, lfoOutput[lfoBankIndex(v, 0u)],
                lfoOutput[lfoBankIndex(v, 1u)],
                FilterMacroContext::fromKeyMod(control.keyMod, pluckEnv));
            mods = {mod, mod, mod, mod};
            unified = true;
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                const float pluckEnv =
                    control.global.masterRun
                        ? evalVoicePluckPreviewEnv01(
                              control.keyMod.rhythmPluck, control.keyMod.rhythmMode[i],
                              control.keyMod.rhythmParam[i], static_cast<uint8_t>(i), rhythmPhase01,
                              ((stutterGateMask >> i) & 1u) != 0u)
                        : 0.0f;
                mods[i] = computeLiveVoiceFilter(
                    vp, control.voicePressure[i], pitchMode, lfoOutput[lfoBankIndex(i, 0u)],
                    lfoOutput[lfoBankIndex(i, 1u)],
                    FilterMacroContext::fromKeyMod(control.keyMod, pluckEnv));
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
        updateLfoGraph(dtClamped, snapLfo, control, effectiveHud, lfoOutput, lfoDriftPacked);
    } else {
        lfoGraphPhase_ = stepHudFadeLinear(lfoGraphPhase_, false, dtClamped);
    }

    const bool wantBackdrop = rawFilterActive || rawLfoGraphActive || extraCenterViewActive;
    filterBackdropDim_ = stepHudFadeLinear(filterBackdropDim_, wantBackdrop, dtClamped);
}

void HudGraphs::compositeLfoStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                  uint16_t* linScratch, float effLfoAlpha01, uint8_t lfoAlpha8,
                                  const EngineControlState& control, const HudState& hud,
                                  const LfoSignalArray& lfoPhase) const noexcept {
    if (lfoAlpha8 == 0u || y1 <= 12 || y0 >= 206) {
        return;
    }
    constexpr float kFixedToPx = 1.0f / 64.0f;
    const MonoStripCanvas lfoCv{lumBuf, y0, y1, lfoAlpha8};

    const uint8_t slot = hud.lfoVoiceFocus & 7u;
    const float maxD = lfoSlotDepth(control.voices[lfoFocusVoice(slot)], lfoFocusSub(slot));
    const float lfoEnergy = smoothstep01(clampf(maxD, 0.0f, 1.0f) * 3.0f);
    const float spineModeScale = 0.58f + 0.42f * lfoEnergy;
    const float fillModeScale = 0.34f + 0.66f * lfoEnergy;

    lfoCv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), kLfoYZero, 38.0f,
                                 0.50f, false);

    const float fillScale = fillModeScale;
    if (y1 > static_cast<int>(kLfoYTop) - 1 && y0 <= static_cast<int>(kLfoYBot) + 1) {
        const auto& pts = lfoCurvesFixed_[0];
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
            const float invL2 = fastRecipPos(1.0f + dy * dy);
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

    if (y1 > static_cast<int>(kLfoYTop) - 4 && y0 <= static_cast<int>(kLfoYBot) + 4) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        if (lfoSiblingWeight_ > 0.005f) {
            const float sibWgt = lfoSiblingWeight_ * effLfoAlpha01;
            const auto& sibPts = lfoCurvesFixed_[1];
            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                splatPhotonSegment(static_cast<int>(colX),
                                   static_cast<float>(sibPts[colX]) * kFixedToPx,
                                   static_cast<float>(sibPts[colX + 1u]) * kFixedToPx, sibWgt,
                                   0.60f, y0, y1, linScratch);
            }
        }
        const float mainCurveWgt = 1.04f * spineModeScale * effLfoAlpha01;
        if (mainCurveWgt > 0.004f) {
            const auto& pts = lfoCurvesFixed_[0];
            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                splatPhotonSegment(static_cast<int>(colX),
                                   static_cast<float>(pts[colX]) * kFixedToPx,
                                   static_cast<float>(pts[colX + 1u]) * kFixedToPx, mainCurveWgt,
                                   0.64f, y0, y1, linScratch);
            }
        }
        screenBlendLinearStrip(stripHeight, 0u, kDisplayWidth, linScratch, lumBuf);
    }

    const auto evalCurveBeadPos = [&](size_t curveIdx, float phase01, float& outX,
                                      float& outY) noexcept {
        const float ph = clampf(phase01, 0.0f, 0.999999f);
        const float uIdx = ph * static_cast<float>(kDisplayWidth);
        const size_t i0 = (static_cast<size_t>(uIdx) < kDisplayWidth) ? static_cast<size_t>(uIdx)
                                                                      : (kDisplayWidth - 1u);
        const float frac = uIdx - static_cast<float>(i0);
        const float y0Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i0]) * kFixedToPx;
        const float y1Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i0 + 1u]) * kFixedToPx;
        outX = clampf(uIdx, 0.0f, static_cast<float>(kDisplayWidth - 1u));
        outY = y0Px + frac * (y1Px - y0Px);
    };

    if (control.global.masterRun && lfoSiblingWeight_ > 0.005f) {
        float sibX = 0.0f;
        float sibY = 0.0f;
        evalCurveBeadPos(1u, lfoPhase[lfoBeadBankIdx_[1] & 7u], sibX, sibY);
        lfoCv.drawOpticalBead(sibX, sibY, clampf(lfoSiblingWeight_ * 1.45f, 0.0f, 0.55f), 1.65f);
    }

    if (lfoBeadIntensity_ > 0.005f) {
        const size_t bankIdx = lfoBeadBankIdx_[0] & 7u;
        float beadX = 0.0f;
        float beadY = 0.0f;
        evalCurveBeadPos(0u, lfoPhase[bankIdx], beadX, beadY);
        const float dSlot = lfoSlotDepth(control.voices[bankIdx & 3u], bankIdx >> 2u);
        lfoCv.drawLfoPhaseBeacon(beadX, beadY, lfoBeadIntensity_, dSlot);
    }
}

void HudGraphs::compositeFilterStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                     uint16_t* linScratch, float effFilterAlpha01,
                                     uint8_t filterAlpha8) const noexcept {
    if (filterAlpha8 == 0u || y1 <= kGraphStripMinY1 || y0 >= kGraphStripMaxY0) {
        return;
    }
    constexpr float kFixedToPx = 1.0f / 64.0f;
    const MonoStripCanvas filterCv{lumBuf, y0, y1, filterAlpha8};
    const float div = clampf(filterDivergenceSmooth_, 0.0f, 1.0f);
    const bool multiCurves = (div > 0.001f);
    const size_t curveCount = multiCurves ? kNumOscillators : 1u;

    drawGraphRefHorizon(filterCv, kFilterYZeroDb);
    drawGraphBaseline(filterCv);

    const float fillScale = multiCurves ? (0.25f + 0.05f * div) : 1.0f;
    const float colScale = fillScale * effFilterAlpha01;
    if (colScale > 0.004f && y1 > static_cast<int>(kFilterYTop) &&
        y0 <= static_cast<int>(kFilterYBot)) {
        for (size_t vc = 0u; vc < curveCount; ++vc) {
            const auto& pts = filterCurvesFixed_[vc];
            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                const float yA = static_cast<float>(pts[colX]) * kFixedToPx;
                const float yB = static_cast<float>(pts[colX + 1u]) * kFixedToPx;
                shadeUnipolarGraphColumn(colX, yA, yB, kFilterYZeroDb, 76.0f, colScale, y0, y1,
                                         lumBuf);
            }
        }
    }

    if (y1 > static_cast<int>(kFilterYTop) - 4 && y0 <= static_cast<int>(kFilterYBot) + 5) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float mainCurveWgt =
            (multiCurves ? (0.245f + 0.275f * div) : 0.98f) * effFilterAlpha01;
        if (mainCurveWgt > 0.004f) {
            for (size_t vc = 0u; vc < curveCount; ++vc) {
                const auto& pts = filterCurvesFixed_[vc];
                for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                    const float yA = static_cast<float>(pts[colX]) * kFixedToPx;
                    const float yB = static_cast<float>(pts[colX + 1u]) * kFixedToPx;
                    splatUnipolarGraphSegment(colX, yA, yB, mainCurveWgt, y0, y1, linScratch);
                }
            }
        }

        screenBlendLinearStrip(stripHeight, 0u, kDisplayWidth, linScratch, lumBuf);
    }

    const float singleVis = 1.0f - div;
    const float hpBeadVis = filterHpBeadInt_ * singleVis;
    if (hpBeadVis > 0.005f) {
        filterCv.drawOpticalBead(filterHpBeadX_, filterHpBeadY_, hpBeadVis, filterHpBeadRad_);
    }
    const float lpBeadVis = filterLpBeadInt_ * singleVis;
    if (lpBeadVis > 0.005f) {
        filterCv.drawOpticalBead(filterLpBeadX_, filterLpBeadY_, lpBeadVis, filterLpBeadRad_);
    }
}

} // namespace ruido
