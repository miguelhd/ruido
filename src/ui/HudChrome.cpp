#include "ui/HudChrome.h"

#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/KeyPunchSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/FontSansAA.h"
#include "ui/GraphStyle.h"
#include "ui/HudColumnText.h"
#include "ui/HudGraphs.h"

namespace ruido {

HudGaugeTargets computeHudGaugeTargets(const EngineControlState& control, const HudState& hud,
                                       const LfoSignalArray& lfoOutput,
                                       uint8_t stutterGateMask) noexcept {
    HudGaugeTargets out{};

    switch (hud.mode) {
    case HudMode::MasterLevel:
        out.headerBarNorm = clampf(control.global.masterTrim, 0.0f, 1.0f);
        out.hasHeaderBar = true;
        break;
    case HudMode::RailCoupling:
        out.headerBarNorm = clampf(control.global.circuitCoupling, 0.0f, 1.0f);
        out.hasHeaderBar = true;
        break;
    case HudMode::MasterTempo:
        out.headerBarNorm = normalizeClockBpmLog01(hud.effectiveClockBpm);
        out.hasHeaderBar = true;
        break;
    case HudMode::ScopeTimebase:
        out.headerBarNorm = scopeTimebaseNorm01(hud.scopeTimebase);
        out.hasHeaderBar = true;
        break;
    case HudMode::VoicePressure: {
        out.headerBarNorm = clampf(control.voicePressure[hud.focusedVoice & 3u], 0.0f, 1.0f);
        out.hasHeaderBar = true;
        break;
    }
    case HudMode::KeyboardPlay:
        out.hasHeaderBar = evalKeyPunchHeaderBar(hud, out.headerBarNorm);
        break;
    case HudMode::ShiftMap:
        if (hud.shiftFocus == ShiftFocus::Tempo) {
            out.headerBarNorm = normalizeClockBpmLog01(hud.effectiveClockBpm);
            out.hasHeaderBar = true;
        } else {
            out.hasHeaderBar = evalKeyPunchHeaderBar(hud, out.headerBarNorm);
        }
        break;
    default:
        break;
    }

    if (hud.knobBank == KnobBankMode::None || HudGlobalViews::isRailCouplingActive(hud)) {
        return out;
    }

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    ModulatedFilterParams voiceFlt{};
    if (hud.knobBank == KnobBankMode::VoiceFilter) {
        const uint8_t refV = resolveFilterViewVoice(hud);
        voiceFlt = computeLiveVoiceFilter(control.voices[refV], control.voicePressure[refV],
                                          pitchMode, lfoOutput[lfoBankIndex(refV, 0u)],
                                          lfoOutput[lfoBankIndex(refV, 1u)],
                                          FilterMacroContext::fromKeyMod(control.keyMod));
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        out.focused[k] = (hud.focusedKnob == k) && isKnobFocusableMode(hud.mode);

        if (hud.knobBank == KnobBankMode::GlobalFilter) {
            const GlobalColumnSummary gs =
                evaluateGlobalFilterColumn(k, control, lfoOutput, stutterGateMask);
            out.posSigned[k] = gs.posSigned;
            out.dimAmount[k] = 0.0f;
            continue;
        }
        if (hud.knobBank == KnobBankMode::Panning && control.global.monoCollapse &&
            control.keyMod.spatialFocus <= 0.001f) {
            out.focused[k] = false;
        }

        const VoiceCircuitState vc = isPerOscKnobBank(hud.knobBank)
                                         ? evaluateVoiceCircuit(control, k, stutterGateMask)
                                         : VoiceCircuitState{};
        const float lfoA = lfoOutput[lfoBankIndex(k, 0u)];
        const float lfoB = lfoOutput[lfoBankIndex(k, 1u)];
        float tgtSigned = 0.0f;
        switch (hud.knobBank) {
        case KnobBankMode::Frequency: {
            const VoiceParams& vp = control.voices[k];
            const float pMul = computeVoicePitchRatio(control.voicePressure[k], pitchMode) *
                               control.keyMod.voicePitchMul[k];
            const float lfoHz = computeLiveVoicePitchHz(vp, lfoA, lfoB);
            const float norm = normalizeOscFreqIndicator01(lfoHz * pMul);
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        case KnobBankMode::PulseWidth: {
            const VoiceParams& vp = control.voices[k];
            const float livePw =
                computeLiveVoicePulseWidth(vp, lfoA, lfoB, control.keyMod.voicePwDelta[k]);
            tgtSigned = clampf((livePw - 0.50f) * (1.0f / 0.45f), -1.0f, 1.0f);
            break;
        }
        case KnobBankMode::VoiceDrift: {
            const VoiceParams& vp = control.voices[k];
            tgtSigned = 2.0f * clampf(vp.drift, 0.0f, 1.0f) - 1.0f;
            break;
        }
        case KnobBankMode::VoiceFilter: {
            float norm = 0.0f;
            if (k == 0u) {
                norm = clampf((fastLog2(voiceFlt.hpCutoffHz) - kFilterLogMin) *
                                  (1.0f / kFilterLogSpan),
                              0.0f, 1.0f);
            } else if (k == 1u) {
                norm = clampf(voiceFlt.hpResonance, 0.0f, 1.0f);
            } else if (k == 2u) {
                norm = clampf((fastLog2(voiceFlt.lpCutoffHz) - kFilterLogMin) *
                                  (1.0f / kFilterLogSpan),
                              0.0f, 1.0f);
            } else {
                norm = clampf(voiceFlt.lpResonance, 0.0f, 1.0f);
            }
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        case KnobBankMode::Amplitude: {
            const float norm = evaluateLiveOscAmplitude(control, k, lfoA, lfoB, stutterGateMask);
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        case KnobBankMode::Panning: {
            const VoiceParams& vp = control.voices[k];
            tgtSigned =
                applySpatialFocusPan(computeLiveVoicePan(vp, lfoA, lfoB), k,
                                     control.global.monoCollapse, control.keyMod.spatialFocus);
            break;
        }
        case KnobBankMode::Lfo: {
            const uint8_t slot = hud.lfoVoiceFocus & 7u;
            const uint8_t lv = lfoFocusVoice(slot);
            const uint8_t sub = lfoFocusSub(slot);
            const VoiceParams& vp = control.voices[lv];
            const float lfoBRaw = lfoOutput[lfoBankIndex(lv, 1u)];
            const float maxTgt =
                static_cast<float>((sub == 0u ? kNumLfoATargets : kNumLfoBTargets) - 1u);
            float norm = 0.0f;
            if (k == 0u) {
                norm = clampf(static_cast<float>(static_cast<uint8_t>(lfoSlotTarget(vp, sub))) /
                                  maxTgt,
                              0.0f, 1.0f);
            } else if (k == 1u) {
                norm = clampf(lfoSlotShape(vp, sub), 0.0f, 1.0f);
            } else if (k == 2u) {
                const float liveRate =
                    (sub == 0u) ? computeLiveLfoARateHz(vp, lfoBRaw) : vp.lfoBRateHz;
                norm = clampf((fastLog2(liveRate) - kLfoLogMin) * (1.0f / kLfoLogSpan), 0.0f, 1.0f);
            } else {
                const float liveDepth =
                    (sub == 0u) ? computeLiveLfoADepth(vp, lfoBRaw) : vp.lfoBDepth;
                norm = clampf(liveDepth, 0.0f, 1.0f);
            }
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        default:
            break;
        }

        out.posSigned[k] = tgtSigned;
        out.dimAmount[k] = vc.inaudibleDim;
    }

    return out;
}

namespace {

constexpr bool shouldShowOscModRow(const HudState& hud) noexcept {
    return (hud.freqPageOpen || hud.editPageOpen || hud.panPageOpen) &&
           isPerOscKnobBank(hud.knobBank) && hud.mode != HudMode::KeyboardPlay;
}

constexpr bool doesLfoTargetKnobBank(LfoTarget tgt, KnobBankMode bank) noexcept {
    switch (bank) {
    case KnobBankMode::Frequency:
        return tgt == LfoTarget::Pitch;
    case KnobBankMode::PulseWidth:
        return tgt == LfoTarget::PulseWidth;
    case KnobBankMode::Amplitude:
        return tgt == LfoTarget::Amplitude;
    case KnobBankMode::Panning:
        return tgt == LfoTarget::Pan;
    default:
        return false;
    }
}

} // namespace

void prepareHudChrome(PreparedHudChrome& out, const EngineControlState& control,
                      const HudState& hud, const LfoSignalArray& lfoOutput,
                      uint8_t stutterGateMask) noexcept {
    if (hud.knobBank == KnobBankMode::None || HudGlobalViews::isRailCouplingActive(hud)) {
        return;
    }
    if (hud.knobBank == KnobBankMode::Panning && control.global.monoCollapse &&
        control.keyMod.spatialFocus <= 0.001f) {
        return;
    }
    const bool showOscModRow = shouldShowOscModRow(hud);
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const FilterMacroContext fltMacros = FilterMacroContext::fromKeyMod(control.keyMod);

    for (uint8_t k = 0u; k < 4u; ++k) {
        PreparedHudColumn& col = out.cols[k];
        col = PreparedHudColumn{};
        GlobalColumnSummary gs{};
        formatColumnRoleAndValue(control, hud, lfoOutput, stutterGateMask, k, col.roleLbl,
                                 sizeof(col.roleLbl), col.valStr, sizeof(col.valStr), gs);
        col.voiceSigned = gs.voiceSigned;
        col.bipolar = gs.bipolar;
        col.disabled = gs.disabled;
        col.hasVoiceSpread = gs.hasVoiceSpread;
        if (!col.disabled) {
            col.valTrk = (measureSansText(kFontSansMedium, col.valStr, 0) > 58) ? -1 : 0;
            if (showOscModRow) {
                const VoiceParams& vp = control.voices[k];
                const float lfoARaw = lfoOutput[lfoBankIndex(k, 0u)];
                const float lfoBRaw = lfoOutput[lfoBankIndex(k, 1u)];
                const float p = clampf(control.voicePressure[k], 0.0f, 1.0f);
                const ModulatedFilterParams flt =
                    computeLiveVoiceFilter(vp, p, pitchMode, lfoARaw, lfoBRaw, fltMacros);
                constexpr float kHpMinOct = cxLog2Pos(kFilterCutoffMinHz);
                constexpr float kLpMaxOct = cxLog2Pos(kFilterCutoffMaxHz);
                const float hpEng =
                    (flt.hpCutoffHz > 2.05f)
                        ? clampf((fastLog2(flt.hpCutoffHz) - kHpMinOct) * (1.0f / 11.0f), 0.0f,
                                 1.0f)
                        : 0.0f;
                const float lpEng =
                    (flt.lpCutoffHz < 27900.0f)
                        ? clampf((kLpMaxOct - fastLog2(flt.lpCutoffHz)) * (1.0f / 11.0f), 0.0f,
                                 1.0f)
                        : 0.0f;
                const float cutEng = (hpEng > lpEng) ? hpEng : lpEng;
                const float resEng =
                    clampf((flt.hpResonance > flt.lpResonance) ? flt.hpResonance : flt.lpResonance,
                           0.0f, 1.0f);
                const float fltEng = clampf((cutEng > resEng) ? (cutEng + 0.35f * resEng)
                                                              : (resEng + 0.35f * cutEng),
                                            0.0f, 1.0f);
                col.fxOn = isVoiceFilterActive(vp, p, pitchMode) || (fltEng > 1.0e-3f);
                col.fxLevel01 = col.fxOn ? clampf(0.28f + 0.72f * fltEng, 0.0f, 1.0f) : 0.0f;

                const float liveDepthA = computeLiveLfoADepth(vp, lfoBRaw);
                col.lfoAOn = doesLfoTargetKnobBank(vp.lfoTarget, hud.knobBank) &&
                             ((vp.lfoDepth > 0.002f) || (liveDepthA > 0.002f));
                if (col.lfoAOn && control.global.masterRun) {
                    const float waveA01 = 0.5f * (clampf(lfoARaw, -1.0f, 1.0f) + 1.0f);
                    col.lfoALevel01 = waveA01 * (0.35f + 0.65f * clampf(liveDepthA, 0.0f, 1.0f));
                }
                col.lfoBOn =
                    doesLfoTargetKnobBank(vp.lfoBTarget, hud.knobBank) && (vp.lfoBDepth > 0.002f);
                if (col.lfoBOn && control.global.masterRun) {
                    const float waveB01 = 0.5f * (clampf(lfoBRaw, -1.0f, 1.0f) + 1.0f);
                    col.lfoBLevel01 = waveB01 * (0.35f + 0.65f * clampf(vp.lfoBDepth, 0.0f, 1.0f));
                }
            }
        }
    }
}

void compositeBottomKnobStrip(const MonoStripCanvas& cv, const PreparedHudChrome& prep,
                              const EngineControlState& control, const HudState& hud,
                              const std::array<float, kNumOscillators>& knobPosSignedSmooth,
                              const std::array<float, kNumOscillators>& knobDimSmooth,
                              const std::array<float, kNumOscillators>& knobFocusSmooth) noexcept {
    if (cv.y1 <= 188) {
        return;
    }
    if (HudGlobalViews::isRailCouplingActive(hud)) {
        const float effCoupling =
            clampf(control.global.circuitCoupling + control.keyMod.overloadSurge, 0.0f, 1.0f);
        const char* leftPart = (effCoupling <= 0.005f)  ? "ISOLATED"
                               : (effCoupling <= 0.45f) ? "SHARED POWER"
                                                        : "STARVED POWER";
        const char* rightPart = (effCoupling <= 0.005f)  ? "FREE RUNNING"
                                : (effCoupling <= 0.45f) ? "SOFT PULL"
                                                         : "HARD SYNC";
        const int leftW = measureSansText(kFontSansSmall, leftPart, 1);
        const int rightW = measureSansText(kFontSansSmall, rightPart, 1);
        constexpr int kDotGapPx = 14;
        const int totalW = leftW + kDotGapPx + rightW;
        const int startX = 120 - (totalW / 2);
        cv.drawSans(kFontSansSmall, startX, 206, leftPart, 152u, 1);
        cv.drawOpticalBead(static_cast<float>(startX + leftW + (kDotGapPx / 2)), 211.2f, 0.48f,
                           0.92f);
        cv.drawSans(kFontSansSmall, startX + leftW + kDotGapPx, 206, rightPart, 152u, 1);
        return;
    }
    if (hud.knobBank == KnobBankMode::None) {
        return;
    }
    if (hud.knobBank == KnobBankMode::Panning && control.global.monoCollapse &&
        control.keyMod.spatialFocus <= 0.001f) {
        cv.drawSansCentered(kFontSansSmall, 120, 205, "OUTPUT ROUTING SET TO MONO", 154u, 1);
        return;
    }

    const bool showOscModRow = shouldShowOscModRow(hud);

    for (uint8_t k = 0u; k < 4u; ++k) {
        const int cx = hudColumnCenterX(k);
        const PreparedHudColumn& col = prep.cols[k];
        const bool bipolar = col.bipolar;

        const float dim = clampf(knobDimSmooth[k], 0.0f, 1.0f);
        if (col.disabled) {
            const auto disLbl = static_cast<uint8_t>(88.0f + (64.0f - 88.0f) * dim + 0.5f);
            const auto disVal = static_cast<uint8_t>(104.0f + (76.0f - 104.0f) * dim + 0.5f);
            cv.drawSansCentered(kFontSansSmall, cx, kHudBotLabelY, col.roleLbl, disLbl, 1);
            cv.drawHorizontalFilament(static_cast<float>(cx) - 20.0f,
                                      static_cast<float>(cx) + 20.0f, kHudBotTrackY, 22.0f, 0.58f,
                                      true);
            cv.drawSansCentered(kFontSansSmall, cx, kHudBotDisabledValueY, col.valStr, disVal, 0);
            continue;
        }
        const float foc = clampf(knobFocusSmooth[k], 0.0f, 1.0f);
        const float effFoc = foc * (1.0f - dim);
        const float posSigned = clampf(knobPosSignedSmooth[k], -1.0f, 1.0f);

        const float baseLbl = 154.0f + (72.0f - 154.0f) * dim;
        const auto lblGray = static_cast<uint8_t>(baseLbl + (255.0f - baseLbl) * effFoc + 0.5f);
        cv.drawSansCentered(kFontSansSmall, cx, kHudBotLabelY, col.roleLbl, lblGray, 1);

        const float cxF = static_cast<float>(cx);
        const float bxf = cxF - 20.0f;
        const float bwf = 40.0f;
        const float byf = kHudBotTrackY;
        const float trackLum = 42.0f + (22.0f - 42.0f) * dim;
        const float unfocLum = 178.0f + (64.0f - 178.0f) * dim;
        const float activeLum = unfocLum + (235.0f - unfocLum) * effFoc;
        const float targetX = cxF + posSigned * 20.0f;

        cv.drawHorizontalFilament(bxf, bxf + bwf, byf, trackLum, 0.58f, true);
        if (col.hasVoiceSpread) {
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                const float vx = cxF + clampf(col.voiceSigned[vi], -1.0f, 1.0f) * 20.0f;
                cv.drawOpticalBead(vx, byf, 0.44f, 1.15f);
            }
        }
        if (bipolar) {
            cv.drawVerticalSlit(cxF, byf - 2.2f, byf + 2.2f, 85.0f + (42.0f - 85.0f) * dim);
            if (absf(targetX - cxF) > 0.4f) {
                cv.drawHorizontalFilament(cxF, targetX, byf, activeLum, 0.68f, false);
            }
            const float beadIntUnfoc = 0.82f + (0.30f - 0.82f) * dim;
            const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * effFoc;
            const float beadRad = 1.90f + (2.25f - 1.90f) * effFoc;
            cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
        } else {
            if (targetX - bxf > 0.32f) {
                cv.drawHorizontalFilament(bxf, targetX, byf, activeLum, 0.68f, false);
            }
            const float beadIntUnfoc = 0.80f + (0.28f - 0.80f) * dim;
            const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * effFoc;
            const float beadRad = 1.85f + (2.30f - 1.85f) * effFoc;
            cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
        }

        const float baseVal = 224.0f + (96.0f - 224.0f) * dim;
        const auto valGray = static_cast<uint8_t>(baseVal + (255.0f - baseVal) * effFoc + 0.5f);
        cv.drawSansCentered(kFontSansMedium, cx, kHudBotValueY, col.valStr, valGray, col.valTrk);

        if (showOscModRow) {
            constexpr int kTotalSpan = kHudHomeFxLfoGapPx + kHudHomeLfoPairGapPx;
            const int fxX = cx - (kTotalSpan / 2);
            const int lfoAX = fxX + kHudHomeFxLfoGapPx;
            const int lfoBX = lfoAX + kHudHomeLfoPairGapPx;
            cv.drawModulatorSlot(fxX, kHudHomeModRowY, col.fxOn, col.fxLevel01, dim);
            cv.drawModulatorSlot(lfoAX, kHudHomeModRowY, col.lfoAOn, col.lfoALevel01, dim);
            cv.drawModulatorSlot(lfoBX, kHudHomeModRowY, col.lfoBOn, col.lfoBLevel01, dim);
        }
    }
}

} // namespace ruido
