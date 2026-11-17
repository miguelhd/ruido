#include "ui/HudHeader.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/GlobalConfig.h"
#include "core/KeyPunchSpec.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/BootIntroData.h"
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
        return kPctSlotW;
    case HudMode::MasterTempo:
        return hud.midiClockSynced ? kBpmExtSlotW : kBpmSlotW;
    case HudMode::ScopeTimebase:
        return kScopeSlotW;
    case HudMode::ShiftMap:
        if (hud.shiftFocus == ShiftFocus::Tempo) {
            return hud.midiClockSynced ? kBpmExtSlotW : kBpmSlotW;
        }
        return 0;
    case HudMode::KeyboardPlay:
        switch (keyPunchSpec(hud.activePunch).readoutKind) {
        case PunchReadoutKind::Percent:
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

// 3x3 supersampled (63x16 px) RUIDO vector wordmark derived directly from `kLogoCore4Bit` (`0 B`
// extra Flash or SRAM), aligned flush at `leftX = 10, topY = 5` (`y = 5..20`).
void drawHeaderRuidoWordmark(const MonoStripCanvas& cv, int leftX, int topY) noexcept {
    constexpr int kOutW = 63;
    constexpr int kOutH = 16;
    if (cv.masterAlpha8 == 0u || topY + kOutH <= cv.y0 || topY >= cv.y1) {
        return;
    }
    const int hyStart = (topY < cv.y0) ? (cv.y0 - topY) : 0;
    const int hyEnd = (topY + kOutH > cv.y1) ? (cv.y1 - topY) : kOutH;
    for (int hy = hyStart; hy < hyEnd; ++hy) {
        const int py = topY + hy;
        uint8_t* dstRow = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        const int syBase = (95 - intro_data::kCoreY0) + hy * 3;
        for (int hx = 0; hx < kOutW; ++hx) {
            const int sxBase = 16 + hx * 3;
            uint32_t sum9 = 0u;
            for (int dy = 0; dy < 3; ++dy) {
                const uint8_t* srcRow =
                    &intro_data::kLogoCore4Bit[static_cast<size_t>(syBase + dy) *
                                               static_cast<size_t>(intro_data::kCoreRowBytes)];
                for (int dx = 0; dx < 3; ++dx) {
                    const int sx = sxBase + dx;
                    const uint8_t b = srcRow[sx >> 1];
                    sum9 += ((sx & 1) == 0) ? static_cast<uint32_t>(b >> 4u)
                                            : static_cast<uint32_t>(b & 0x0Fu);
                }
            }
            if (sum9 != 0u) {
                const uint32_t cov8 = (sum9 * 255u + 67u) / 135u;
                const auto a8 = static_cast<uint8_t>(
                    (cov8 * static_cast<uint32_t>(cv.masterAlpha8) + 127u) / 255u);
                dstRow[leftX + hx] = blendLum8(dstRow[leftX + hx], 255u, a8);
            }
        }
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

void prepareHudHeader(PreparedHudHeader& out, const EngineControlState& control,
                      const HudState& hud, const LfoSignalArray& lfoOutput, uint8_t stutterGateMask,
                      float finePhase, float coarsePhase) noexcept {
    out = PreparedHudHeader{};
    if (hud.mode == HudMode::None) {
        return;
    }
    const uint8_t v = hud.focusedVoice & 3u;
    float headerBarNorm = 0.0f;
    float voiceLiveHz = 0.0f;
    out.minBarSlotW = static_cast<int16_t>(minHeaderBarSlotWidth(hud));

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
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "FREQUENCY");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
            appendUInt(out.leftSuffix, lp, sizeof(out.leftSuffix), static_cast<uint32_t>(kv + 1u));
            const float pitchMul = computeVoicePitchRatio(control.voicePressure[kv], hdrPitchMode);
            const float liveHz =
                computeLiveVoicePitchHz(control.voices[kv], lfoOutput[lfoBankIndex(kv, 0u)],
                                        lfoOutput[lfoBankIndex(kv, 1u)]);
            voiceLiveHz = liveHz * pitchMul;
            appendPrecisionHz(out.rightText, 0u, sizeof(out.rightText), voiceLiveHz);
        }
        break;
    }
    case HudMode::PulseWidth: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "DUTY");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
            appendUInt(out.leftSuffix, lp, sizeof(out.leftSuffix), static_cast<uint32_t>(kv + 1u));
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(out.rightText, 0u, sizeof(out.rightText), "MUTED");
            } else {
                appendStr(out.rightText, 0u, sizeof(out.rightText),
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
            appendStr(out.rightText, 0u, sizeof(out.rightText),
                      !anyActive ? "MUTED" : (anyPulse ? "PULSE" : "SQUARE"));
        }
        break;
    }
    case HudMode::VoiceDrift: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "DRIFT");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
            appendUInt(out.leftSuffix, lp, sizeof(out.leftSuffix), static_cast<uint32_t>(kv + 1u));
            const VoiceParams& dvp = control.voices[kv];
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(out.rightText, 0u, sizeof(out.rightText), "MUTED");
            } else {
                appendStr(out.rightText, 0u, sizeof(out.rightText),
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
            appendStr(out.rightText, 0u, sizeof(out.rightText),
                      !anyActive ? "MUTED" : (anyDrifting ? "ANALOG 1/F" : "LOCKED"));
        }
        break;
    }
    case HudMode::VoiceFilter: {
        const uint8_t fv = hud.fxVoiceFocus & 3u;
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "FILTER");
        size_t p = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
        appendUInt(out.leftSuffix, p, sizeof(out.leftSuffix), static_cast<uint32_t>(fv + 1u));
        appendStr(out.rightText, 0u, sizeof(out.rightText), "12 dB HP -> LP");
        break;
    }
    case HudMode::Amplitude: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "LEVEL");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
            appendUInt(out.leftSuffix, lp, sizeof(out.leftSuffix), static_cast<uint32_t>(kv + 1u));
            if (voiceAud(kv).isMuted) {
                appendStr(out.rightText, 0u, sizeof(out.rightText), "MUTED");
            } else {
                formatHeaderDriveValue(liveAmpAt(kv), out.rightText, sizeof(out.rightText));
            }
        } else {
            formatHeaderEnsembleDrive(control, lfoOutput, stutterGateMask, out.rightText,
                                      sizeof(out.rightText));
        }
        break;
    }
    case HudMode::Panning:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "PAN");
        formatHeaderPanMode(control, hud, stutterGateMask, out.leftSuffix, sizeof(out.leftSuffix),
                            out.rightText, sizeof(out.rightText));
        break;
    case HudMode::MasterLevel:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "MASTER LEVEL");
        headerBarNorm = clampf(control.global.masterTrim, 0.0f, 1.0f);
        formatPercent(out.rightText, sizeof(out.rightText), headerBarNorm);
        out.showHeaderBar = true;
        break;
    case HudMode::RailCoupling:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "RAIL COUPLING");
        headerBarNorm = clampf(control.global.circuitCoupling, 0.0f, 1.0f);
        formatPercent(out.rightText, sizeof(out.rightText), headerBarNorm);
        out.showHeaderBar = true;
        break;
    case HudMode::MasterTempo: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "TEMPO");
        headerBarNorm = normalizeClockBpmLog01(hud.effectiveClockBpm);
        const bool showTenths =
            !hud.midiClockSynced && (hud.rateHoldActive && hud.knobRate == KnobRate::Fine);
        formatClockBpm(out.rightText, sizeof(out.rightText), hud.effectiveClockBpm,
                       hud.midiClockSynced, showTenths);
        out.showHeaderBar = true;
        break;
    }
    case HudMode::ScopeTimebase:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "OSCILLOSCOPE");
        headerBarNorm = scopeTimebaseNorm01(hud.scopeTimebase);
        appendStr(out.rightText, 0u, sizeof(out.rightText), scopeTimebaseLabel(hud.scopeTimebase));
        out.showHeaderBar = true;
        break;
    case HudMode::KnobRate:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "FREQUENCY");
        appendStr(out.rightText, 0u, sizeof(out.rightText), "STANDARD");
        break;
    case HudMode::VoiceGate: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "GATE");
        size_t p = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
        appendUInt(out.leftSuffix, p, sizeof(out.leftSuffix), static_cast<uint32_t>(v + 1u));
        const VoiceCircuitState ga = voiceAud(v);
        const char* gateStatus =
            ga.isMuted ? "MUTED" : ((ga.subAudioBlend >= 0.5f) ? "SUB-AUDIO" : "ACTIVE");
        appendStr(out.rightText, 0u, sizeof(out.rightText), gateStatus);
        break;
    }
    case HudMode::VoicePressure: {
        appendStr(out.leftMain, 0u, sizeof(out.leftMain),
                  control.global.pitchPressureMode ? "PITCH BEND" : "PRESSURE");
        size_t p = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
        appendUInt(out.leftSuffix, p, sizeof(out.leftSuffix), static_cast<uint32_t>(v + 1u));
        headerBarNorm = clampf(control.voicePressure[v], 0.0f, 1.0f);
        formatPercent(out.rightText, sizeof(out.rightText), headerBarNorm);
        out.showHeaderBar = true;
        break;
    }
    case HudMode::SnapshotSlot:
    case HudMode::SaveArmed:
        formatSnapshotHeader(hud, out.leftMain, sizeof(out.leftMain), out.leftSuffix,
                             sizeof(out.leftSuffix), out.rightText, sizeof(out.rightText),
                             out.showHeaderBar, headerBarNorm, &out.showSceneDots);
        break;
    case HudMode::OutputRouting:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "OUTPUT ROUTING");
        appendStr(out.rightText, 0u, sizeof(out.rightText),
                  control.global.monoCollapse ? "MONO" : "STEREO");
        break;
    case HudMode::AudioOutput:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "AUDIO OUTPUT");
        appendStr(out.rightText, 0u, sizeof(out.rightText),
                  control.global.masterRun ? "ON" : "OFF");
        break;
    case HudMode::ScopeFreeze:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "SCOPE FREEZE");
        appendStr(out.rightText, 0u, sizeof(out.rightText), "HOLDING");
        break;
    case HudMode::KeyHold:
        appendStr(out.leftMain, 0u, sizeof(out.leftMain), "LATCH");
        appendStr(out.rightText, 0u, sizeof(out.rightText),
                  hud.keyHoldLatched ? ((hud.keyHoldCount > 0u) ? "LATCHED" : "ARMED")
                                     : "RELEASED");
        break;
    case HudMode::VoiceLfo: {
        const uint8_t slot = hud.lfoVoiceFocus & 7u;
        const uint8_t lv = lfoFocusVoice(slot);
        const uint8_t sub = lfoFocusSub(slot);
        const VoiceParams& lvp = control.voices[lv];
        size_t p =
            appendStr(out.leftMain, 0u, sizeof(out.leftMain), (sub == 0u) ? "LFO A" : "LFO B");
        appendUInt(out.leftMain, p, sizeof(out.leftMain), static_cast<uint32_t>(lv + 1u));
        size_t sp = appendStr(out.leftSuffix, 0u, sizeof(out.leftSuffix), " OSC ");
        appendUInt(out.leftSuffix, sp, sizeof(out.leftSuffix), static_cast<uint32_t>(lv + 1u));
        appendStr(out.rightText, 0u, sizeof(out.rightText),
                  (lfoSlotDepth(lvp, sub) > 0.002f) ? "ACTIVE" : "BYPASSED");
        break;
    }
    case HudMode::GlobalConfig: {
        switch (hud.configSubView) {
        case ConfigSubView::FactoryResetConfirm:
        case ConfigSubView::CleanInitConfirm:
        case ConfigSubView::FactoryResetDone:
        case ConfigSubView::CleanInitDone: {
            appendStr(out.leftMain, 0u, sizeof(out.leftMain), "FACTORY RESET");
            const bool secondRow = (hud.configSubView == ConfigSubView::CleanInitConfirm ||
                                    hud.configSubView == ConfigSubView::CleanInitDone);
            appendStr(out.rightText, 0u, sizeof(out.rightText), secondRow ? "2 / 2" : "1 / 2");
            break;
        }
        case ConfigSubView::Credits: {
            appendStr(out.leftMain, 0u, sizeof(out.leftMain), "CREDITS");
            size_t p = appendStr(out.rightText, 0u, sizeof(out.rightText), "v");
            appendStr(out.rightText, p, sizeof(out.rightText), kRuidoVersionStr);
            break;
        }
        case ConfigSubView::List:
        default: {
            appendStr(out.leftMain, 0u, sizeof(out.leftMain), "GLOBAL CONFIG");
            const uint32_t cur = static_cast<uint32_t>(
                (hud.configCursor < kNumGlobalConfigItems) ? (hud.configCursor + 1u) : 1u);
            size_t p = appendUInt(out.rightText, 0u, sizeof(out.rightText), cur);
            p = appendStr(out.rightText, p, sizeof(out.rightText), " / ");
            appendUInt(out.rightText, p, sizeof(out.rightText),
                       static_cast<uint32_t>(kNumGlobalConfigItems));
            break;
        }
        }
        break;
    }
    case HudMode::KeyboardModel:
    case HudMode::KeyboardPlay:
    case HudMode::ShiftMap:
        formatKeybedTopHeader(hud, out.leftMain, sizeof(out.leftMain), out.leftSuffix,
                              sizeof(out.leftSuffix), out.rightText, sizeof(out.rightText),
                              out.showHeaderBar, headerBarNorm, &out.showSceneDots);
        break;
    case HudMode::Count:
        break;
    }

    if (hud.mode != HudMode::None && !out.showHeaderBar && !out.showSceneDots &&
        out.rightText[0] == '\0') {
        const char* defStatus = !control.global.masterRun     ? "AUDIO OFF"
                                : control.global.monoCollapse ? "MONO"
                                                              : "STEREO";
        appendStr(out.rightText, 0u, sizeof(out.rightText), defStatus);
    }

    out.isRuidoAbout =
        (hud.mode == HudMode::GlobalConfig) && (hud.configSubView == ConfigSubView::Credits);
    out.isStepBadgeText = (hud.mode == HudMode::GlobalConfig) && !out.showHeaderBar;

    if (out.isRuidoAbout) {
        out.leftEndX = 10 + 63;
    } else {
        size_t idx0 = 0u;
        bool acute0 = false;
        const int xOff0 =
            (out.leftMain[0] != '\0')
                ? static_cast<int>(kFontSansBold
                                       .metrics[static_cast<size_t>(
                                           decodeSansGlyphChar(out.leftMain, idx0, acute0) - 32u)]
                                       .xOffset)
                : 0;
        out.mainX = static_cast<int16_t>(10 - xOff0);
        const int mainW = measureSansText(kFontSansBold, out.leftMain, 0);
        out.leftEndX = static_cast<int16_t>(out.mainX + mainW);
        out.suffixX = out.leftEndX;
        if (out.leftSuffix[0] != '\0') {
            out.leftEndX = static_cast<int16_t>(out.leftEndX +
                                                measureSansText(kFontSansBold, out.leftSuffix, 0));
        } else if (out.showSceneDots) {
            out.leftEndX = static_cast<int16_t>(out.leftEndX + 29);
        }
    }

    if (out.showHeaderBar) {
        out.rightTxtW = static_cast<int16_t>(measureSansText(kFontSansMedium, out.rightText, 0));
    }

    const float fLin = clampf(finePhase, 0.0f, 1.0f);
    const float cLin = clampf(coarsePhase, 0.0f, 1.0f);
    if (fLin <= 0.001f && cLin <= 0.001f) {
        return;
    }

    const bool isSnapHdr = (hud.mode == HudMode::SnapshotSlot) ||
                           ((hud.mode == HudMode::KeyboardPlay || hud.mode == HudMode::ShiftMap) &&
                            hud.activePunch == KeyPunchKind::SnapshotRecall);
    const auto prepRateBadge = [&](const char* badgeStr, bool isCoarse, float linPhase,
                                   char (&valBuf)[24], int16_t& valW, int8_t& badgeTrk,
                                   int16_t& badgeW) noexcept {
        if (linPhase <= 0.001f) {
            return;
        }
        if (hud.mode == HudMode::VoiceFrequency && hud.focusedKnob < kNumOscillators) {
            if (isCoarse) {
                formatCompactHz(valBuf, sizeof(valBuf), voiceLiveHz);
            } else {
                appendPrecisionHz(valBuf, 0u, sizeof(valBuf), voiceLiveHz);
            }
        } else if ((out.showHeaderBar || isSnapHdr) && out.rightText[0] != '\0') {
            const int candValW = measureSansText(kFontSansSmall, out.rightText, 0);
            const int candBadgeW = measureSansText(kFontSansSmall, badgeStr, 0);
            if ((230 - candValW - 8 - candBadgeW) >= (out.leftEndX + 8)) {
                appendStr(valBuf, 0u, sizeof(valBuf), out.rightText);
            }
        }
        int badgeRightX = 230;
        if (valBuf[0] != '\0') {
            valW = static_cast<int16_t>(measureSansText(kFontSansSmall, valBuf, 0));
            badgeRightX = 230 - valW - 8;
        }
        badgeTrk = (badgeRightX == 230) ? 1 : 0;
        badgeW = static_cast<int16_t>(measureSansText(kFontSansSmall, badgeStr, badgeTrk));
    };

    prepRateBadge("FINE", false, fLin, out.fineRateVal, out.fineValW, out.fineBadgeTrk,
                  out.fineBadgeW);
    prepRateBadge("COARSE", true, cLin, out.coarseRateVal, out.coarseValW, out.coarseBadgeTrk,
                  out.coarseBadgeW);
}

void compositeTopHeaderStrip(const MonoStripCanvas& cv, const PreparedHudHeader& prep,
                             const HudState& hud, float headerBarSmooth, float finePhase,
                             float coarsePhase) noexcept {
    if (cv.y0 >= 26) {
        return;
    }

    if (prep.isRuidoAbout) {
        drawHeaderRuidoWordmark(cv, 10, 5);
    } else {
        cv.drawSans(kFontSansBold, prep.mainX, 6, prep.leftMain, 255u, 0);
        if (prep.showSceneDots && prep.leftSuffix[0] == '\0') {
            const float dialCx = static_cast<float>(prep.suffixX) + 15.0f;
            constexpr float kDialRadiusPx = 12.5f;
            const float knobTurns = wrapLfoPhase01(prep.sceneDialPhase * 0.25f);
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                float dTurns = static_cast<float>(s) * 0.25f - knobTurns;
                if (dTurns > 0.5f) {
                    dTurns -= 1.0f;
                } else if (dTurns < -0.5f) {
                    dTurns += 1.0f;
                }
                if (absf(dTurns) >= 0.244f) {
                    continue;
                }
                const float sinA = evalAnalogSine(wrapLfoPhase01(dTurns));
                const float cosA =
                    clampf(evalAnalogSine(wrapLfoPhase01(dTurns + 0.25f)), 0.0f, 1.0f);
                const float limb = cosA * (0.35f + 0.65f * cosA);
                const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
                const float focus = cosA * cosA;
                const float baseLum = occ ? (116.0f + 98.0f * focus * focus) : 64.0f;
                const auto lum8 = static_cast<uint8_t>(clampf(baseLum * limb + 0.5f, 0.0f, 255.0f));
                if (lum8 < 6u) {
                    continue;
                }
                cv.drawSansCharScaledX(kFontSansBold, static_cast<char>('1' + s),
                                       dialCx + kDialRadiusPx * sinA, 6, 0.38f + 0.62f * cosA,
                                       lum8);
            }
        } else if (prep.leftSuffix[0] != '\0') {
            cv.drawSans(kFontSansBold, prep.suffixX, 6, prep.leftSuffix, 118u, 0);
        }
    }

    const auto drawNormalRight = [&](const MonoStripCanvas& dstCv) noexcept {
        if (prep.showSceneDots) {
            constexpr float kFirstDotX = 180.0f;
            constexpr float kDotStrideX = 16.0f;
            constexpr float kDotY = 13.5f;
            for (size_t s = 0u; s + 1u < kNumSnapshots; ++s) {
                const float x0 = kFirstDotX + static_cast<float>(s) * kDotStrideX + 4.0f;
                const float x1 = kFirstDotX + static_cast<float>(s + 1u) * kDotStrideX - 4.0f;
                const float w0 = prep.sceneDotWeights[s];
                const float w1 = prep.sceneDotWeights[s + 1u];
                const float bridge = (w0 < w1) ? w0 : w1;
                const float filLum = 38.0f + 128.0f * clampf(bridge * 2.0f, 0.0f, 1.0f);
                dstCv.drawHorizontalFilament(x0, x1, kDotY, filLum, 0.55f, true);
            }
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                const float cx = kFirstDotX + static_cast<float>(s) * kDotStrideX;
                const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
                if (!occ) {
                    dstCv.drawOpticalBead(cx, kDotY, 0.22f, 1.15f);
                } else {
                    const float w = clampf(prep.sceneDotWeights[s], 0.0f, 1.0f);
                    dstCv.drawOpticalBead(cx, kDotY, 0.48f + 0.52f * w, 1.45f + 0.70f * w);
                }
            }
        } else if (prep.showHeaderBar) {
            const int barTxtW = prep.rightTxtW;
            dstCv.drawSans(kFontSansMedium, 230 - barTxtW, 6, prep.rightText, 255u, 0);

            const int slotW = (barTxtW > prep.minBarSlotW) ? barTxtW : prep.minBarSlotW;
            const float barWf = 34.0f;
            const float barXf = static_cast<float>(230 - slotW - 8) - barWf;
            const float barNorm = clampf(headerBarSmooth, 0.0f, 1.0f);
            dstCv.drawHorizontalFilament(barXf, barXf + barWf, 13.5f, 42.0f, 0.58f, true);
            if (barNorm > 0.008f) {
                dstCv.drawHorizontalFilament(barXf, barXf + barNorm * barWf, 13.5f, 225.0f, 0.68f,
                                             false);
            }
            dstCv.drawOpticalBead(barXf + barNorm * barWf, 13.5f, 0.95f, 2.1f);
        } else if (prep.rightText[0] != '\0') {
            if (prep.isStepBadgeText) {
                dstCv.drawSansRight(kFontSansMedium, 230, 6, prep.rightText, 174u, 0);
            } else {
                dstCv.drawSansRight(kFontSansSmall, 230, 6, prep.rightText, 168u, 1);
            }
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

    const auto drawRateBadge = [&](const char* badgeStr, float linPhase, const char* rateValBuf,
                                   int16_t valW, int8_t badgeTrk, int16_t badgeW) noexcept {
        if (linPhase <= 0.001f) {
            return;
        }
        const uint8_t rateAlpha8 = shapedLayerAlpha8(linPhase);
        int badgeRightX = 230;
        if (rateValBuf[0] != '\0') {
            badgeRightX = 230 - valW - 8;
            if (rateAlpha8 > 0u) {
                const MonoStripCanvas rateCv{cv.lum, cv.y0, cv.y1, rateAlpha8};
                rateCv.drawSans(kFontSansSmall, 230 - valW, 6, rateValBuf, 176u, 0);
                rateCv.drawOpticalBead(static_cast<float>(230 - valW - 4), 12.2f, 0.52f, 0.95f);
            }
        }
        if (rateAlpha8 > 0u) {
            const MonoStripCanvas rateCv{cv.lum, cv.y0, cv.y1, rateAlpha8};
            rateCv.drawSans(kFontSansSmall, badgeRightX - badgeW, 6, badgeStr, 236u, badgeTrk);
        }
    };

    drawRateBadge("FINE", fLin, prep.fineRateVal, prep.fineValW, prep.fineBadgeTrk,
                  prep.fineBadgeW);
    drawRateBadge("COARSE", cLin, prep.coarseRateVal, prep.coarseValW, prep.coarseBadgeTrk,
                  prep.coarseBadgeW);
}

} // namespace ruido
