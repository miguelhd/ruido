#include "ui/HudChrome.h"

#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/GlobalControl.h"
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
    case HudMode::VoicePressure: {
        out.headerBarNorm = clampf(control.voicePressure[hud.focusedVoice & 3u], 0.0f, 1.0f);
        out.hasHeaderBar = true;
        break;
    }
    case HudMode::SnapshotSlot:
        if (hud.snapshotGliding) {
            out.headerBarNorm = clampf(hud.morphPosition, 0.0f, 1.0f);
            out.hasHeaderBar = true;
        }
        break;
    case HudMode::KeyboardPlay:
        out.hasHeaderBar = evalKeyPunchHeaderBar(hud, out.headerBarNorm);
        break;
    default:
        break;
    }

    if (hud.knobBank == KnobBankMode::None) {
        return out;
    }

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    ModulatedFilterParams voiceFlt{};
    if (hud.knobBank == KnobBankMode::VoiceFilter) {
        const uint8_t refV = resolveFilterViewVoice(hud);
        const FilterMacroContext fltMacros{
            control.keyMod.dubLpfDrop,
            control.keyMod.overloadSurge,
            control.keyMod.filterScream,
        };
        voiceFlt = computeLiveVoiceFilter(control.voices[refV], control.voicePressure[refV],
                                          pitchMode, lfoOutput[lfoBankIndex(refV, 0u)],
                                          lfoOutput[lfoBankIndex(refV, 1u)], fltMacros);
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        out.focused[k] = (hud.focusedKnob == k) && isKnobFocusableMode(hud.mode);

        if (isGlobalKnobBank(hud.knobBank)) {
            const GlobalColumnSummary gs =
                evaluateGlobalColumn(hud.knobBank, k, control, lfoOutput, stutterGateMask);
            out.posSigned[k] = gs.posSigned;
            out.dimAmount[k] = 0.0f;
            if (gs.disabled) {
                out.focused[k] = false;
            }
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
            const float norm = clampf((fastLog2(lfoHz * pMul) - fastLog2(kAudioFreqMinHz)) /
                                          (fastLog2(kAudioFreqMaxHz) - fastLog2(kAudioFreqMinHz)),
                                      0.0f, 1.0f);
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
        case KnobBankMode::Snapshot: {
            const float w = clampf(hud.snapshotWeights[k], 0.0f, 1.0f);
            out.posSigned[k] = 2.0f * w - 1.0f;
            out.focused[k] = (k == (hud.snapshotSlot & 3u));
            out.dimAmount[k] = hud.saveArmed ? 0.20f : (0.72f * (1.0f - w));
            continue;
        }
        default:
            break;
        }

        out.posSigned[k] = tgtSigned;
        out.dimAmount[k] = vc.inaudibleDim;
    }

    return out;
}

void compositeBottomKnobStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                              const HudState& hud, const LfoSignalArray& lfoOutput,
                              uint8_t stutterGateMask,
                              const std::array<float, kNumOscillators>& knobPosSignedSmooth,
                              const std::array<float, kNumOscillators>& knobDimSmooth,
                              const std::array<float, kNumOscillators>& knobFocusSmooth) noexcept {
    if (cv.y1 <= 188 || hud.knobBank == KnobBankMode::None) {
        return;
    }
    if (hud.knobBank == KnobBankMode::Panning && control.global.monoCollapse &&
        control.keyMod.spatialFocus <= 0.001f) {
        cv.drawSansCentered(kFontSansSmall, 120, 205, "OUTPUT ROUTING SET TO MONO", 154u, 1);
        return;
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        const int cx = hudColumnCenterX(k);
        char roleLbl[12]{};
        char valStr[16]{};
        GlobalColumnSummary gs{};
        formatColumnRoleAndValue(control, hud, lfoOutput, stutterGateMask, k, roleLbl,
                                 sizeof(roleLbl), valStr, sizeof(valStr), gs);
        const bool bipolar = gs.bipolar;

        const float dim = clampf(knobDimSmooth[k], 0.0f, 1.0f);
        if (gs.disabled) {
            const auto disLbl = static_cast<uint8_t>(88.0f + (64.0f - 88.0f) * dim + 0.5f);
            const auto disVal = static_cast<uint8_t>(104.0f + (76.0f - 104.0f) * dim + 0.5f);
            cv.drawSansCentered(kFontSansSmall, cx, kHudBotLabelY, roleLbl, disLbl, 1);
            cv.drawHorizontalFilament(static_cast<float>(cx) - 20.0f,
                                      static_cast<float>(cx) + 20.0f, kHudBotTrackY, 22.0f, 0.58f,
                                      true);
            cv.drawSansCentered(kFontSansSmall, cx, kHudBotDisabledValueY, valStr, disVal, 0);
            continue;
        }
        const float foc = clampf(knobFocusSmooth[k], 0.0f, 1.0f);
        const float effFoc = foc * (1.0f - dim);
        const float posSigned = clampf(knobPosSignedSmooth[k], -1.0f, 1.0f);

        const float baseLbl = 154.0f + (72.0f - 154.0f) * dim;
        const auto lblGray = static_cast<uint8_t>(baseLbl + (255.0f - baseLbl) * effFoc + 0.5f);
        cv.drawSansCentered(kFontSansSmall, cx, kHudBotLabelY, roleLbl, lblGray, 1);

        const float cxF = static_cast<float>(cx);
        const float bxf = cxF - 20.0f;
        const float bwf = 40.0f;
        const float byf = kHudBotTrackY;
        const float trackLum = 42.0f + (22.0f - 42.0f) * dim;
        const float unfocLum = 178.0f + (64.0f - 178.0f) * dim;
        const float activeLum = unfocLum + (235.0f - unfocLum) * effFoc;
        const float targetX = cxF + posSigned * 20.0f;

        cv.drawHorizontalFilament(bxf, bxf + bwf, byf, trackLum, 0.58f, true);
        if (gs.hasVoiceSpread) {
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                const float vx = cxF + clampf(gs.voiceSigned[vi], -1.0f, 1.0f) * 20.0f;
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
        cv.drawSansCentered(kFontSansMedium, cx, kHudBotValueY, valStr, valGray, 0);

        if (hud.knobBank == KnobBankMode::Frequency) {
            constexpr int kTotalSpan = kHudHomeFxLfoGapPx + kHudHomeLfoPairGapPx;
            const int fxX = cx - (kTotalSpan / 2);
            const int lfoAX = fxX + kHudHomeFxLfoGapPx;
            const int lfoBX = lfoAX + kHudHomeLfoPairGapPx;
            const VoiceParams& vp = control.voices[k];
            const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
            const bool fxOn = isVoiceFilterActive(vp, control.voicePressure[k], pitchMode);
            cv.drawModulatorSlot(fxX, kHudHomeModRowY, fxOn, hud.fxLedSync01, dim);
            cv.drawModulatorSlot(lfoAX, kHudHomeModRowY, vp.lfoDepth > 0.002f, hud.lfoLedSync01,
                                 dim);
            cv.drawModulatorSlot(lfoBX, kHudHomeModRowY, vp.lfoBDepth > 0.002f, hud.lfoLedSync01,
                                 dim);
        }
    }
}

} // namespace ruido
