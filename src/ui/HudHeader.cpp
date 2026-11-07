#include "ui/HudHeader.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/GlobalConfig.h"
#include "core/KeyPunchSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/FontSansAA.h"
#include "ui/HudColumnText.h"
#include "ui/HudFormat.h"
#include "ui/HudKeybedView.h"

namespace ruido {

namespace {

constexpr int minHeaderBarSlotWidth(const HudState& hud) noexcept {
    constexpr int kPctSlotW = measureSansText(kFontSansMedium, "100%", 0);
    constexpr int kBpmSlotW = measureSansText(kFontSansMedium, "9999 BPM", 0);
    constexpr int kBpmExtSlotW = measureSansText(kFontSansMedium, "999 BPM EXT", 0);
    constexpr int kBpmMultExtSlotW = measureSansText(kFontSansMedium, "9999 BPM EXT", 0);
    constexpr int kScopeSlotW = []() constexpr noexcept {
        int maxW = 0;
        for (uint8_t i = 0u; i < static_cast<uint8_t>(ScopeTimebase::Count); ++i) {
            const int w = measureSansText(kFontSansMedium,
                                          scopeTimebaseLabel(static_cast<ScopeTimebase>(i)), 0);
            if (w > maxW) {
                maxW = w;
            }
        }
        return maxW;
    }();

    switch (hud.mode) {
    case HudMode::MasterLevel:
    case HudMode::RailCoupling:
    case HudMode::VoicePressure:
    case HudMode::SnapshotSlot:
        return kPctSlotW;
    case HudMode::MasterTempo:
        return hud.midiClockSynced ? kBpmExtSlotW : kBpmSlotW;
    case HudMode::ScopeTimebase:
        return kScopeSlotW;
    case HudMode::KeyboardPlay:
        switch (keyPunchSpec(hud.activePunch).readoutKind) {
        case PunchReadoutKind::Percent:
        case PunchReadoutKind::SwingPct:
        case PunchReadoutKind::Snapshot:
            return kPctSlotW;
        case PunchReadoutKind::SubOctave:
            return measureSansText(kFontSansMedium, "-2.0 OCT", 0);
        case PunchReadoutKind::PosOctave:
            return measureSansText(kFontSansMedium, "+2.0 OCT", 0);
        case PunchReadoutKind::JustRatio:
            return measureSansText(kFontSansMedium, "9:8", 0);
        case PunchReadoutKind::StutterHz:
            return measureSansText(kFontSansMedium, "24.0 Hz", 0);
        case PunchReadoutKind::FoldStages:
            return measureSansText(kFontSansMedium, "6.2x", 0);
        case PunchReadoutKind::CrushHz:
            return measureSansText(kFontSansMedium, "9.9 kHz", 0);
        case PunchReadoutKind::TurboMult:
            return measureSansText(kFontSansMedium, "32.0x", 0);
        case PunchReadoutKind::ClockDiv:
            return measureSansText(kFontSansMedium, "X8", 0);
        case PunchReadoutKind::EuclidHits:
            return measureSansText(kFontSansMedium, "16/16", 0);
        case PunchReadoutKind::TempoBpm:
            return hud.midiClockSynced ? kBpmMultExtSlotW : kBpmSlotW;
        case PunchReadoutKind::TapBpm:
            return hud.midiClockSynced ? kBpmExtSlotW : kBpmSlotW;
        default:
            return 0;
        }
    default:
        return 0;
    }
}

} // namespace

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
                             const HudState& hud, const LfoSignalArray& lfoOutput,
                             uint8_t stutterGateMask, float headerBarSmooth, float finePhase,
                             float coarsePhase) noexcept {
    if (cv.y0 >= 26) {
        return;
    }
    const uint8_t v = hud.focusedVoice & 3u;
    char leftMain[24]{};
    char leftSuffix[16]{};
    char rightText[32]{};
    bool showHeaderBar = false;
    float headerBarNorm = 0.0f;
    float voiceLiveHz = 0.0f;
    const int minBarSlotW = minHeaderBarSlotWidth(hud);

    const float hdrPitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const auto voiceAud = [&](size_t vi) noexcept {
        return evaluateVoiceCircuit(control, vi, stutterGateMask);
    };
    const auto livePwAt = [&](size_t vi) noexcept {
        return computeLiveVoicePulseWidth(control.voices[vi], lfoOutput[lfoBankIndex(vi, 0u)],
                                          lfoOutput[lfoBankIndex(vi, 1u)],
                                          control.keyMod.voicePwDelta[vi]);
    };
    const auto liveAmpAt = [&](size_t vi) noexcept {
        return evaluateLiveOscAmplitude(control, vi, lfoOutput[lfoBankIndex(vi, 0u)],
                                        lfoOutput[lfoBankIndex(vi, 1u)], stutterGateMask);
    };

    switch (hud.mode) {
    case HudMode::None:
        break;
    case HudMode::VoiceFrequency: {
        appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(kv + 1u));
            const float pitchMul = computeVoicePitchRatio(control.voicePressure[kv], hdrPitchMode);
            const float liveHz =
                computeLiveVoicePitchHz(control.voices[kv], lfoOutput[lfoBankIndex(kv, 0u)],
                                        lfoOutput[lfoBankIndex(kv, 1u)]);
            voiceLiveHz = liveHz * pitchMul;
            appendPrecisionHz(rightText, 0u, sizeof(rightText), voiceLiveHz);
        }
        break;
    }
    case HudMode::PulseWidth: {
        appendStr(leftMain, 0u, sizeof(leftMain), "DUTY");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(kv + 1u));
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(rightText, 0u, sizeof(rightText), "MUTED");
            } else {
                appendStr(rightText, 0u, sizeof(rightText),
                          (absf(livePwAt(kv) - 0.50f) <= 0.015f) ? "SQUARE" : "PULSE");
            }
        } else {
            bool anyActive = false;
            bool anyPulse = false;
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                if (voiceAud(vi).muteBlend < 0.999f) {
                    anyActive = true;
                    if (absf(livePwAt(vi) - 0.50f) > 0.015f) {
                        anyPulse = true;
                    }
                }
            }
            appendStr(rightText, 0u, sizeof(rightText),
                      !anyActive ? "MUTED" : (anyPulse ? "PULSE" : "SQUARE"));
        }
        break;
    }
    case HudMode::VoiceDrift: {
        appendStr(leftMain, 0u, sizeof(leftMain), "DRIFT");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(kv + 1u));
            const VoiceParams& dvp = control.voices[kv];
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(rightText, 0u, sizeof(rightText), "MUTED");
            } else {
                appendStr(rightText, 0u, sizeof(rightText),
                          (dvp.drift <= 0.004f) ? "LOCKED" : "ANALOG 1/F");
            }
        } else {
            bool anyActive = false;
            bool anyDrifting = false;
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                if (voiceAud(vi).muteBlend < 0.999f) {
                    anyActive = true;
                    if (control.voices[vi].drift > 0.004f) {
                        anyDrifting = true;
                    }
                }
            }
            appendStr(rightText, 0u, sizeof(rightText),
                      !anyActive ? "MUTED" : (anyDrifting ? "ANALOG 1/F" : "LOCKED"));
        }
        break;
    }
    case HudMode::VoiceFilter: {
        const uint8_t fv = hud.fxVoiceFocus & 3u;
        appendStr(leftMain, 0u, sizeof(leftMain), "FILTER");
        size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(fv + 1u));
        appendStr(rightText, 0u, sizeof(rightText), "12 dB HP -> LP");
        break;
    }
    case HudMode::Amplitude: {
        appendStr(leftMain, 0u, sizeof(leftMain), "LEVEL");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(kv + 1u));
            if (voiceAud(kv).isMuted) {
                appendStr(rightText, 0u, sizeof(rightText), "MUTED");
            } else {
                formatHeaderDriveValue(liveAmpAt(kv), rightText, sizeof(rightText));
            }
        } else {
            formatHeaderEnsembleDrive(control, lfoOutput, stutterGateMask, rightText,
                                      sizeof(rightText));
        }
        break;
    }
    case HudMode::Panning:
        appendStr(leftMain, 0u, sizeof(leftMain), "PAN");
        formatHeaderPanMode(control, hud, stutterGateMask, leftSuffix, sizeof(leftSuffix),
                            rightText, sizeof(rightText));
        break;
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
    case HudMode::MasterTempo: {
        appendStr(leftMain, 0u, sizeof(leftMain), "TEMPO");
        headerBarNorm = normalizeClockBpmLog01(hud.effectiveClockBpm);
        const bool showTenths =
            !hud.midiClockSynced && (hud.rateHoldActive && hud.knobRate == KnobRate::Fine);
        formatClockBpm(rightText, sizeof(rightText), hud.effectiveClockBpm, hud.midiClockSynced,
                       showTenths);
        showHeaderBar = true;
        break;
    }
    case HudMode::ScopeTimebase:
        appendStr(leftMain, 0u, sizeof(leftMain), "OSCILLOSCOPE");
        headerBarNorm = scopeTimebaseNorm01(hud.scopeTimebase);
        appendStr(rightText, 0u, sizeof(rightText), scopeTimebaseLabel(hud.scopeTimebase));
        showHeaderBar = true;
        break;
    case HudMode::KnobRate:
        appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
        appendStr(rightText, 0u, sizeof(rightText), "STANDARD");
        break;
    case HudMode::VoiceGate: {
        appendStr(leftMain, 0u, sizeof(leftMain), "GATE");
        size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
        const VoiceCircuitState ga = voiceAud(v);
        const char* gateStatus =
            ga.isMuted ? "MUTED" : ((ga.subAudioBlend >= 0.5f) ? "SUB-AUDIO" : "ACTIVE");
        appendStr(rightText, 0u, sizeof(rightText), gateStatus);
        break;
    }
    case HudMode::VoicePressure: {
        appendStr(leftMain, 0u, sizeof(leftMain),
                  control.global.pitchPressureMode ? "PITCH BEND" : "PRESSURE");
        size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
        headerBarNorm = clampf(control.voicePressure[v], 0.0f, 1.0f);
        formatPercent(rightText, sizeof(rightText), headerBarNorm);
        showHeaderBar = true;
        break;
    }
    case HudMode::SnapshotSlot:
    case HudMode::SaveArmed:
        formatSnapshotHeader(hud, leftMain, sizeof(leftMain), leftSuffix, sizeof(leftSuffix),
                             rightText, sizeof(rightText), showHeaderBar, headerBarNorm);
        break;
    case HudMode::OutputRouting:
        appendStr(leftMain, 0u, sizeof(leftMain), "OUTPUT ROUTING");
        appendStr(rightText, 0u, sizeof(rightText),
                  control.global.monoCollapse ? "MONO" : "STEREO");
        break;
    case HudMode::AudioOutput:
        appendStr(leftMain, 0u, sizeof(leftMain), "AUDIO OUTPUT");
        appendStr(rightText, 0u, sizeof(rightText), control.global.masterRun ? "ON" : "OFF");
        break;
    case HudMode::ScopeFreeze:
        appendStr(leftMain, 0u, sizeof(leftMain), "SCOPE FREEZE");
        appendStr(rightText, 0u, sizeof(rightText), "HOLDING");
        break;
    case HudMode::VoiceLfo: {
        const uint8_t slot = hud.lfoVoiceFocus & 7u;
        const uint8_t lv = lfoFocusVoice(slot);
        const uint8_t sub = lfoFocusSub(slot);
        const VoiceParams& lvp = control.voices[lv];
        appendStr(leftMain, 0u, sizeof(leftMain), (sub == 0u) ? "LFO A" : "LFO B");
        size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(lv + 1u));
        appendStr(rightText, 0u, sizeof(rightText),
                  (lfoSlotDepth(lvp, sub) > 0.002f) ? "ACTIVE" : "BYPASSED");
        break;
    }
    case HudMode::GlobalConfig: {
        switch (hud.configSubView) {
        case ConfigSubView::FirmwareUpdate:
            appendStr(leftMain, 0u, sizeof(leftMain), "FIRMWARE");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " UPDATE");
            appendStr(rightText, 0u, sizeof(rightText), "AUDIO MUTED");
            break;
        case ConfigSubView::FactoryResetConfirm:
            appendStr(leftMain, 0u, sizeof(leftMain), "FACTORY");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " RESET");
            appendStr(rightText, 0u, sizeof(rightText), "CONFIRM");
            break;
        case ConfigSubView::FactoryResetDone:
            appendStr(leftMain, 0u, sizeof(leftMain), "FACTORY");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " RESET");
            appendStr(rightText, 0u, sizeof(rightText), "COMPLETE");
            break;
        case ConfigSubView::Credits: {
            appendStr(leftMain, 0u, sizeof(leftMain), "CREDITS");
            size_t p = appendStr(rightText, 0u, sizeof(rightText), "v");
            appendStr(rightText, p, sizeof(rightText), kRuidoVersionStr);
            break;
        }
        case ConfigSubView::List:
        default: {
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
        }
        break;
    }
    case HudMode::KeyboardModel:
    case HudMode::KeyboardPlay:
        formatKeybedTopHeader(hud, leftMain, sizeof(leftMain), leftSuffix, sizeof(leftSuffix),
                              rightText, sizeof(rightText), showHeaderBar, headerBarNorm);
        break;
    case HudMode::Count:
        break;
    }

    if (hud.mode != HudMode::None && !showHeaderBar && rightText[0] == '\0') {
        const char* defStatus = !control.global.masterRun     ? "AUDIO OFF"
                                : control.global.monoCollapse ? "MONO"
                                                              : "STEREO";
        appendStr(rightText, 0u, sizeof(rightText), defStatus);
    }

    cv.drawSans(kFontSansBold, 10, 6, leftMain, 255u, 0);
    if (leftSuffix[0] != '\0') {
        const int suffixX = 10 + measureSansText(kFontSansBold, leftMain, 0);
        cv.drawSans(kFontSansBold, suffixX, 6, leftSuffix, 118u, 0);
    }

    const auto drawNormalRight = [&](const MonoStripCanvas& dstCv) noexcept {
        if (showHeaderBar) {
            const int barTxtW = measureSansText(kFontSansMedium, rightText, 0);
            dstCv.drawSans(kFontSansMedium, 230 - barTxtW, 6, rightText, 255u, 0);

            const int slotW = (barTxtW > minBarSlotW) ? barTxtW : minBarSlotW;
            const float barWf = 34.0f;
            const float barXf = static_cast<float>(230 - slotW - 8) - barWf;
            const float barNorm = clampf(headerBarSmooth, 0.0f, 1.0f);
            dstCv.drawHorizontalFilament(barXf, barXf + barWf, 13.5f, 42.0f, 0.58f, true);
            if (barNorm > 0.008f) {
                dstCv.drawHorizontalFilament(barXf, barXf + barNorm * barWf, 13.5f, 225.0f, 0.68f,
                                             false);
            }
            dstCv.drawOpticalBead(barXf + barNorm * barWf, 13.5f, 0.95f, 2.1f);
        } else if (rightText[0] != '\0') {
            dstCv.drawSansRight(kFontSansSmall, 230, 6, rightText, 168u, 1);
        }
    };

    const float fLin = clampf(finePhase, 0.0f, 1.0f);
    const float cLin = clampf(coarsePhase, 0.0f, 1.0f);
    const float maxRateLin = (fLin > cLin) ? fLin : cLin;
    if (maxRateLin <= 0.001f) {
        drawNormalRight(cv);
        return;
    }

    const auto shapedLayerAlpha8 = [&](float linPhase) noexcept -> uint8_t {
        const float shaped = smoothstep01(clampf((linPhase - 0.24f) * (1.0f / 0.76f), 0.0f, 1.0f));
        return static_cast<uint8_t>(static_cast<float>(cv.masterAlpha8) * shaped + 0.5f);
    };

    const float baseLin = (hud.mode == HudMode::KnobRate) ? 0.0f : (1.0f - maxRateLin);
    const uint8_t baseAlpha8 = shapedLayerAlpha8(baseLin);
    if (baseAlpha8 > 0u) {
        const MonoStripCanvas baseCv{cv.lum, cv.y0, cv.y1, baseAlpha8};
        drawNormalRight(baseCv);
    }

    const auto drawRateBadge = [&](const char* badgeStr, bool isCoarse, float linPhase) noexcept {
        if (linPhase <= 0.001f) {
            return;
        }
        const uint8_t rateAlpha8 = shapedLayerAlpha8(linPhase);
        int badgeRightX = 230;
        char rateValBuf[24]{};

        if (hud.mode == HudMode::VoiceFrequency && hud.focusedKnob < kNumOscillators) {
            if (isCoarse) {
                formatCompactHz(rateValBuf, sizeof(rateValBuf), voiceLiveHz);
            } else {
                appendPrecisionHz(rateValBuf, 0u, sizeof(rateValBuf), voiceLiveHz);
            }
        } else if (showHeaderBar && rightText[0] != '\0') {
            appendStr(rateValBuf, 0u, sizeof(rateValBuf), rightText);
        }

        if (rateValBuf[0] != '\0') {
            const int valW = measureSansText(kFontSansSmall, rateValBuf, 0);
            badgeRightX = 230 - valW - 8;
            if (rateAlpha8 > 0u) {
                const MonoStripCanvas rateCv{cv.lum, cv.y0, cv.y1, rateAlpha8};
                rateCv.drawSans(kFontSansSmall, 230 - valW, 6, rateValBuf, 176u, 0);
                rateCv.drawOpticalBead(static_cast<float>(230 - valW - 4), 12.2f, 0.52f, 0.95f);
            }
        }

        const int badgeTrk = (badgeRightX == 230) ? 1 : 0;
        const int badgeW = measureSansText(kFontSansSmall, badgeStr, badgeTrk);
        if (rateAlpha8 > 0u) {
            const MonoStripCanvas rateCv{cv.lum, cv.y0, cv.y1, rateAlpha8};
            rateCv.drawSans(kFontSansSmall, badgeRightX - badgeW, 6, badgeStr, 236u, badgeTrk);
        }
    };

    drawRateBadge("FINE", false, fLin);
    drawRateBadge("COARSE", true, cLin);
}

} // namespace ruido
