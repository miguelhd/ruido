#include "ui/HudChrome.h"

#include "core/DspMath.h"
#include "core/GlobalConfig.h"
#include "core/GlobalControl.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"
#include "ui/HudColumnText.h"
#include "ui/HudFormat.h"
#include "ui/HudGraphs.h"

namespace ruido {

HudGaugeTargets
computeHudGaugeTargets(const EngineControlState& control, const HudState& hud,
                       const std::array<float, kNumOscillators>& lfoOutput) noexcept {
    HudGaugeTargets out{};

    if (!hud.rateHoldActive) {
        switch (hud.mode) {
        case HudMode::MasterLevel:
            out.headerBarNorm = clampf(control.global.masterTrim, 0.0f, 1.0f);
            out.hasHeaderBar = true;
            break;
        case HudMode::RailCoupling:
            out.headerBarNorm = clampf(control.global.circuitCoupling, 0.0f, 1.0f);
            out.hasHeaderBar = true;
            break;
        case HudMode::VoicePressure:
            out.headerBarNorm = clampf(control.voicePressure[hud.focusedVoice & 3u], 0.0f, 1.0f);
            out.hasHeaderBar = true;
            break;
        default:
            break;
        }
    }

    if (hud.knobBank == KnobBankMode::None) {
        return out;
    }

    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    ModulatedFilterParams voiceFlt{};
    if (hud.knobBank == KnobBankMode::VoiceFilter) {
        const uint8_t refV = hud.focusedVoice & 3u;
        voiceFlt = computeLiveVoiceFilter(control.voices[refV], control.voicePressure[refV],
                                          pitchMode, lfoOutput[refV]);
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        out.focused[k] = (hud.focusedKnob == k) && isKnobFocusableMode(hud.mode);

        if (isGlobalKnobBank(hud.knobBank)) {
            const GlobalColumnSummary gs =
                evaluateGlobalColumn(hud.knobBank, k, control, lfoOutput);
            out.posSigned[k] = gs.posSigned;
            out.dimmed[k] = false;
            continue;
        }

        float tgtSigned = 0.0f;
        bool dimmed = false;
        switch (hud.knobBank) {
        case KnobBankMode::Frequency: {
            const VoiceParams& vp = control.voices[k];
            const float pMul = computeVoicePitchRatio(control.voicePressure[k], pitchMode);
            const float lfoHz = (vp.lfoTarget == LfoTarget::Pitch)
                                    ? applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoOutput[k])
                                    : vp.frequencyHz;
            const float norm = clampf((fastLog2(lfoHz * pMul) - fastLog2(kAudioFreqMinHz)) /
                                          (fastLog2(kAudioFreqMaxHz) - fastLog2(kAudioFreqMinHz)),
                                      0.0f, 1.0f);
            tgtSigned = 2.0f * norm - 1.0f;
            dimmed = vp.muted && (control.voicePressure[k] <= 0.001f);
            break;
        }
        case KnobBankMode::Amplitude: {
            const VoiceParams& vp = control.voices[k];
            const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                      ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[k])
                                      : vp.amplitude;
            const float norm = vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
            tgtSigned = 2.0f * norm - 1.0f;
            dimmed = vp.muted;
            break;
        }
        case KnobBankMode::Panning: {
            const VoiceParams& vp = control.voices[k];
            const float livePan = (vp.lfoTarget == LfoTarget::Pan)
                                      ? applyLfoPan(vp.pan, vp.lfoDepth, lfoOutput[k])
                                      : vp.pan;
            tgtSigned = clampf(livePan * panScale, -1.0f, 1.0f);
            dimmed = vp.muted;
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
        case KnobBankMode::Lfo: {
            const VoiceParams& vp = control.voices[hud.lfoVoiceFocus & 3u];
            float norm = 0.0f;
            if (k == 0u) {
                norm = static_cast<float>(static_cast<uint8_t>(vp.lfoTarget)) * (1.0f / 6.0f);
            } else if (k == 1u) {
                norm = clampf(vp.lfoShape, 0.0f, 1.0f);
            } else if (k == 2u) {
                norm = clampf((fastLog2(vp.lfoRateHz) - kLfoLogMin) * (1.0f / kLfoLogSpan), 0.0f,
                              1.0f);
            } else {
                norm = clampf(vp.lfoDepth, 0.0f, 1.0f);
            }
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        default:
            break;
        }

        out.posSigned[k] = tgtSigned;
        out.dimmed[k] = dimmed;
    }

    return out;
}

void compositeIdlePeakMeters(const MonoStripCanvas& idleCv, float smoothedPeakL,
                             float smoothedPeakR) noexcept {
    const float nl = clampf(smoothedPeakL * (1.0f / 1.25f), 0.0f, 1.0f);
    const float nr = clampf(smoothedPeakR * (1.0f / 1.25f), 0.0f, 1.0f);
    const float actL = smoothstep01(smoothedPeakL * (1.0f / 0.055f));
    const float actR = smoothstep01(smoothedPeakR * (1.0f / 0.055f));
    const float hotL = smoothstep01((smoothedPeakL - 0.94f) * (1.0f / 0.16f));
    const float hotR = smoothstep01((smoothedPeakR - 0.94f) * (1.0f / 0.16f));

    idleCv.drawHorizontalFilament(202.0f, 230.0f, 8.8f, 22.0f, 0.55f, true);
    idleCv.drawHorizontalFilament(202.0f, 230.0f, 12.8f, 22.0f, 0.55f, true);
    if (nl > 0.004f) {
        idleCv.drawHorizontalFilament(202.0f, 202.0f + nl * 28.0f, 8.8f,
                                      (168.0f + 77.0f * hotL) * actL, 0.62f, false);
    }
    if (nr > 0.004f) {
        idleCv.drawHorizontalFilament(202.0f, 202.0f + nr * 28.0f, 12.8f,
                                      (168.0f + 77.0f * hotR) * actR, 0.62f, false);
    }
    const float beadIntL = 0.22f + (0.50f + 0.28f * hotL) * actL;
    const float beadIntR = 0.22f + (0.50f + 0.28f * hotR) * actR;
    idleCv.drawOpticalBead(202.0f + nl * 28.0f, 8.8f, beadIntL, 1.65f);
    idleCv.drawOpticalBead(202.0f + nr * 28.0f, 12.8f, beadIntR, 1.65f);
}

void compositeTopHeaderStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                             const HudState& hud,
                             const std::array<float, kNumOscillators>& lfoOutput,
                             float headerBarSmooth) noexcept {
    if (cv.y0 >= 26) {
        return;
    }
    const uint8_t v = hud.focusedVoice & 3u;
    const VoiceParams& focVp = control.voices[v];
    char leftMain[24]{};
    char leftSuffix[16]{};
    char rightText[32]{};
    bool showHeaderBar = false;
    float headerBarNorm = 0.0f;

    if (hud.rateHoldActive) {
        appendStr(leftMain, 0u, sizeof(leftMain),
                  (hud.knobRate == KnobRate::Fine) ? "FINE" : "COARSE");
        formatRateOverrideRightReadout(control, hud, lfoOutput, rightText, sizeof(rightText));
    } else {
        switch (hud.mode) {
        case HudMode::None:
            break;
        case HudMode::VoiceFrequency: {
            appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            const float pitchMul = computeVoicePitchRatio(
                control.voicePressure[v], control.global.pitchPressureMode ? 1.0f : 0.0f);
            const float liveHz =
                (focVp.lfoTarget == LfoTarget::Pitch)
                    ? applyLfoPitchHz(focVp.frequencyHz, focVp.lfoDepth, lfoOutput[v])
                    : focVp.frequencyHz;
            appendPrecisionHz(rightText, 0u, sizeof(rightText), liveHz * pitchMul);
            break;
        }
        case HudMode::Amplitude:
            appendStr(leftMain, 0u, sizeof(leftMain), "AMPLITUDE");
            appendStr(rightText, 0u, sizeof(rightText), "POST-FILTER VCA");
            break;
        case HudMode::Panning:
            appendStr(leftMain, 0u, sizeof(leftMain), "PANNING");
            appendStr(rightText, 0u, sizeof(rightText),
                      control.global.monoCollapse ? "DUAL MONO" : "STEREO");
            break;
        case HudMode::VoiceFilter: {
            appendStr(leftMain, 0u, sizeof(leftMain), "FILTER");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            appendStr(rightText, 0u, sizeof(rightText), "12 dB HP -> LP");
            break;
        }
        case HudMode::MasterLevel:
            appendStr(leftMain, 0u, sizeof(leftMain), "MASTER LEVEL");
            headerBarNorm = clampf(control.global.masterTrim, 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        case HudMode::RailCoupling:
            appendStr(leftMain, 0u, sizeof(leftMain), "RAIL COUPLING");
            headerBarNorm = clampf(control.global.circuitCoupling, 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        case HudMode::UnassignedEncoder:
            appendStr(leftMain, 0u, sizeof(leftMain),
                      (hud.lastEncoder == EncoderId::Presets) ? "PRESETS ENCODER"
                                                              : "SELECT ENCODER");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        case HudMode::KnobRate: {
            const char* rateStr = (hud.knobRate == KnobRate::Fine)     ? "FINE"
                                  : (hud.knobRate == KnobRate::Coarse) ? "COARSE"
                                                                       : "STANDARD";
            appendStr(leftMain, 0u, sizeof(leftMain), rateStr);
            break;
        }
        case HudMode::VoiceGate: {
            appendStr(leftMain, 0u, sizeof(leftMain), "GATE");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            appendStr(rightText, 0u, sizeof(rightText), focVp.muted ? "MUTED" : "ACTIVE");
            break;
        }
        case HudMode::VoicePressure: {
            if (control.global.pitchPressureMode) {
                appendStr(leftMain, 0u, sizeof(leftMain), "PITCH BEND");
            } else {
                appendStr(leftMain, 0u, sizeof(leftMain), focVp.muted ? "LPG SWELL" : "FILTER EXP");
            }
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            headerBarNorm = clampf(control.voicePressure[v], 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        }
        case HudMode::PressureTarget:
            appendStr(leftMain, 0u, sizeof(leftMain), "PRESSURE TARGET");
            appendStr(rightText, 0u, sizeof(rightText),
                      control.global.pitchPressureMode ? "PITCH BEND" : "MS-20 FILTER");
            break;
        case HudMode::SnapshotSlot: {
            size_t p = appendStr(leftMain, 0u, sizeof(leftMain), "SNAPSHOT ");
            appendUInt(leftMain, p, sizeof(leftMain),
                       static_cast<uint32_t>((hud.snapshotSlot & 7u) + 1u));
            appendStr(rightText, 0u, sizeof(rightText), "EMPTY SLOT");
            break;
        }
        case HudMode::UnassignedKey:
            appendStr(leftMain, 0u, sizeof(leftMain), "PIT KEY");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        case HudMode::OutputRouting:
            appendStr(leftMain, 0u, sizeof(leftMain), "OUTPUT ROUTING");
            appendStr(rightText, 0u, sizeof(rightText),
                      control.global.monoCollapse ? "DUAL MONO" : "STEREO");
            break;
        case HudMode::AudioOutput:
            appendStr(leftMain, 0u, sizeof(leftMain), "AUDIO OUTPUT");
            appendStr(rightText, 0u, sizeof(rightText), control.global.masterRun ? "ON" : "OFF");
            break;
        case HudMode::SaveArmed:
            appendStr(leftMain, 0u, sizeof(leftMain), "STORE SNAPSHOT");
            appendStr(rightText, 0u, sizeof(rightText),
                      hud.saveArmed ? "TAP KEY 9-16" : "CANCELLED");
            break;
        case HudMode::ScopeFreeze:
            appendStr(leftMain, 0u, sizeof(leftMain), "SCOPE FREEZE");
            appendStr(rightText, 0u, sizeof(rightText), "HOLDING");
            break;
        case HudMode::VoiceLfo: {
            const uint8_t lv = hud.lfoVoiceFocus & 3u;
            const VoiceParams& lvp = control.voices[lv];
            appendStr(leftMain, 0u, sizeof(leftMain), "LFO");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(lv + 1u));
            appendStr(rightText, 0u, sizeof(rightText),
                      (lvp.lfoDepth > 0.002f) ? "ACTIVE" : "BYPASSED");
            break;
        }
        case HudMode::GlobalFreq: {
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " FREQ");
            const GlobalColumnSummary gs =
                evaluateGlobalColumn(KnobBankMode::GlobalFreq, 0u, control, lfoOutput);
            char hzBuf[16]{};
            formatCompactHz(hzBuf, sizeof(hzBuf), gs.primaryValue);
            size_t p = appendStr(rightText, 0u, sizeof(rightText), "MEAN ");
            appendStr(rightText, p, sizeof(rightText), hzBuf);
            break;
        }
        case HudMode::GlobalFilter:
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " FILTER");
            appendStr(rightText, 0u, sizeof(rightText), "12 dB HP -> LP");
            break;
        case HudMode::GlobalLfo: {
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " LFO");
            uint32_t activeLfos = 0u;
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                if (control.voices[vi].lfoDepth > 0.002f) {
                    ++activeLfos;
                }
            }
            if (activeLfos == 0u) {
                appendStr(rightText, 0u, sizeof(rightText), "BYPASSED");
            } else {
                size_t p = appendUInt(rightText, 0u, sizeof(rightText), activeLfos);
                appendStr(rightText, p, sizeof(rightText), "/4 ACTIVE");
            }
            break;
        }
        case HudMode::GlobalAmp:
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " AMP");
            appendStr(rightText, 0u, sizeof(rightText),
                      control.global.monoCollapse ? "DUAL MONO" : "STEREO");
            break;
        case HudMode::GlobalAll:
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " ALL");
            appendStr(rightText, 0u, sizeof(rightText),
                      (hud.config.gloRelease == GloReleaseMode::SnapBack) ? "SNAP BACK" : "LATCH");
            break;
        case HudMode::GlobalConfig: {
            appendStr(leftMain, 0u, sizeof(leftMain), "GLOBAL");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " CONFIG");
            const uint32_t cur = static_cast<uint32_t>(
                (hud.configCursor < kNumGlobalConfigItems) ? (hud.configCursor + 1u) : 1u);
            size_t p = appendUInt(rightText, 0u, sizeof(rightText), cur);
            p = appendStr(rightText, p, sizeof(rightText), " / ");
            appendUInt(rightText, p, sizeof(rightText),
                       static_cast<uint32_t>(kNumGlobalConfigItems));
            break;
        }
        case HudMode::UnassignedButton: {
            const char* swName = "PANEL";
            switch (hud.lastButton) {
            case ButtonId::Fx:
                swName = "FX";
                break;
            case ButtonId::Env:
                swName = "ENV";
                break;
            case ButtonId::Edit:
                swName = "EDIT";
                break;
            case ButtonId::Arp:
                swName = "ARP";
                break;
            case ButtonId::Seq:
                swName = "SEQ";
                break;
            case ButtonId::Rec:
                swName = "REC";
                break;
            default:
                break;
            }
            size_t p = appendStr(leftMain, 0u, sizeof(leftMain), swName);
            appendStr(leftMain, p, sizeof(leftMain), " SWITCH");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        }
        }
    }

    if (hud.mode != HudMode::None && !showHeaderBar && rightText[0] == '\0') {
        const char* defStatus = !control.global.masterRun     ? "AUDIO OFF"
                                : control.global.monoCollapse ? "DUAL MONO"
                                                              : "STEREO";
        appendStr(rightText, 0u, sizeof(rightText), defStatus);
    }

    cv.drawSans(kFontSansBold, 10, 6, leftMain, 255u, 0);
    if (leftSuffix[0] != '\0') {
        const int suffixX = 10 + measureSansText(kFontSansBold, leftMain, 0);
        cv.drawSans(kFontSansBold, suffixX, 6, leftSuffix, 118u, 0);
    }

    if (showHeaderBar) {
        const int txtW = measureSansText(kFontSansMedium, rightText, 0);
        cv.drawSans(kFontSansMedium, 230 - txtW, 6, rightText, 255u, 0);

        const float barWf = 34.0f;
        const float barXf = static_cast<float>(230 - txtW - 8) - barWf;
        const float barNorm = clampf(headerBarSmooth, 0.0f, 1.0f);
        cv.drawHorizontalFilament(barXf, barXf + barWf, 13.5f, 42.0f, 0.58f, true);
        if (barNorm > 0.008f) {
            cv.drawHorizontalFilament(barXf, barXf + barNorm * barWf, 13.5f, 225.0f, 0.68f, false);
        }
        cv.drawOpticalBead(barXf + barNorm * barWf, 13.5f, 0.95f, 2.1f);
    } else if (rightText[0] != '\0') {
        const uint8_t rightGray = hud.rateHoldActive ? 228u : 168u;
        cv.drawSansRight(kFontSansSmall, 230, 6, rightText, rightGray, 1);
    }
}

void compositeBottomKnobStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                              const HudState& hud,
                              const std::array<float, kNumOscillators>& lfoOutput,
                              const std::array<float, kNumOscillators>& knobPosSignedSmooth,
                              const std::array<float, kNumOscillators>& knobDimSmooth,
                              const std::array<float, kNumOscillators>& knobFocusSmooth) noexcept {
    if (cv.y1 <= 188 || hud.knobBank == KnobBankMode::None) {
        return;
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        const int cx = 30 + static_cast<int>(k) * 60;
        char roleLbl[12]{};
        char valStr[16]{};
        GlobalColumnSummary gs{};
        formatColumnRoleAndValue(control, hud, lfoOutput, k, roleLbl, sizeof(roleLbl), valStr,
                                 sizeof(valStr), gs);
        const bool bipolar = gs.bipolar;

        const float dim = clampf(knobDimSmooth[k], 0.0f, 1.0f);
        const float foc = clampf(knobFocusSmooth[k], 0.0f, 1.0f);
        const float posSigned = clampf(knobPosSignedSmooth[k], -1.0f, 1.0f);

        const float baseLbl = 154.0f + (96.0f - 154.0f) * dim;
        const auto lblGray = static_cast<uint8_t>(baseLbl + (255.0f - baseLbl) * foc + 0.5f);
        cv.drawSansCentered(kFontSansSmall, cx, 191, roleLbl, lblGray, 1);

        const float cxF = static_cast<float>(cx);
        const float bxf = cxF - 20.0f;
        const float bwf = 40.0f;
        const float byf = 207.5f;
        const float trackLum = 42.0f + (24.0f - 42.0f) * dim;
        const float unfocLum = 178.0f + (72.0f - 178.0f) * dim;
        const float activeLum = unfocLum + (235.0f - unfocLum) * (1.0f - dim) * foc;
        const float targetX = cxF + posSigned * 20.0f;

        cv.drawHorizontalFilament(bxf, bxf + bwf, byf, trackLum, 0.58f, true);
        if (gs.hasVoiceSpread) {
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                const float vx = cxF + clampf(gs.voiceSigned[vi], -1.0f, 1.0f) * 20.0f;
                cv.drawOpticalBead(vx, byf, 0.44f, 1.15f);
            }
        }
        if (bipolar) {
            cv.drawVerticalSlit(cxF, byf - 2.2f, byf + 2.2f, 85.0f);
            if (absf(targetX - cxF) > 0.4f) {
                cv.drawHorizontalFilament(cxF, targetX, byf, activeLum, 0.68f, false);
            }
            const float beadIntUnfoc = 0.82f + (0.36f - 0.82f) * dim;
            const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * (1.0f - dim) * foc;
            const float beadRad = 1.90f + (2.25f - 1.90f) * foc;
            cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
        } else {
            if (targetX - bxf > 0.32f) {
                cv.drawHorizontalFilament(bxf, targetX, byf, activeLum, 0.68f, false);
            }
            const float beadIntUnfoc = 0.80f + (0.34f - 0.80f) * dim;
            const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * (1.0f - dim) * foc;
            const float beadRad = 1.85f + (2.30f - 1.85f) * foc;
            cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
        }

        const float baseVal = 224.0f + (112.0f - 224.0f) * dim;
        const auto valGray =
            static_cast<uint8_t>(baseVal + (255.0f - baseVal) * (1.0f - dim) * foc + 0.5f);
        cv.drawSansCentered(kFontSansMedium, cx, 214, valStr, valGray, 0);
    }
}

} // namespace ruido
