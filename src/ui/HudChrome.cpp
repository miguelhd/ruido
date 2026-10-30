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

    if (hud.knobBank == KnobBankMode::None) {
        return out;
    }

    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    ModulatedFilterParams voiceFlt{};
    if (hud.knobBank == KnobBankMode::VoiceFilter) {
        const uint8_t refV = (hud.mode == HudMode::VoicePressure) ? (hud.focusedVoice & 3u)
                                                                  : (hud.fxVoiceFocus & 3u);
        voiceFlt = computeLiveVoiceFilter(control.voices[refV], control.voicePressure[refV],
                                          pitchMode, lfoOutput[refV]);
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        out.focused[k] = (hud.focusedKnob == k) && isKnobFocusableMode(hud.mode);

        if (isGlobalKnobBank(hud.knobBank)) {
            const GlobalColumnSummary gs =
                evaluateGlobalColumn(hud.knobBank, k, control, lfoOutput);
            out.posSigned[k] = gs.posSigned;
            out.dimAmount[k] = 0.0f;
            continue;
        }

        const VoiceAudibility aud =
            isPerOscKnobBank(hud.knobBank)
                ? computeVoiceAudibility(control.voices[k], control.voicePressure[k], pitchMode)
                : VoiceAudibility{0.0f, 0.0f, 0.0f};
        float tgtSigned = 0.0f;
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
            break;
        }
        case KnobBankMode::PulseWidth: {
            const VoiceParams& vp = control.voices[k];
            tgtSigned = clampf((vp.pulseWidth - 0.50f) * (1.0f / 0.45f), -1.0f, 1.0f);
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
            const VoiceParams& vp = control.voices[k];
            const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                      ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[k])
                                      : vp.amplitude;
            const float norm = (aud.muteBlend >= 0.999f) ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        case KnobBankMode::Panning: {
            const VoiceParams& vp = control.voices[k];
            const float livePan = (vp.lfoTarget == LfoTarget::Pan)
                                      ? applyLfoPan(vp.pan, vp.lfoDepth, lfoOutput[k])
                                      : vp.pan;
            tgtSigned = clampf(livePan * panScale, -1.0f, 1.0f);
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
        out.dimAmount[k] = aud.inaudibleDim;
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
                             float headerBarSmooth, float finePhase, float coarsePhase) noexcept {
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
    float voiceLiveHz = 0.0f;

    const float hdrPitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const auto voiceAud = [&](size_t vi) noexcept {
        return computeVoiceAudibility(control.voices[vi], control.voicePressure[vi], hdrPitchMode);
    };

    switch (hud.mode) {
    case HudMode::None:
        break;
    case HudMode::VoiceFrequency: {
        appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
        size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
        const float pitchMul = computeVoicePitchRatio(control.voicePressure[v], hdrPitchMode);
        const float liveHz = (focVp.lfoTarget == LfoTarget::Pitch)
                                 ? applyLfoPitchHz(focVp.frequencyHz, focVp.lfoDepth, lfoOutput[v])
                                 : focVp.frequencyHz;
        voiceLiveHz = liveHz * pitchMul;
        appendPrecisionHz(rightText, 0u, sizeof(rightText), voiceLiveHz);
        break;
    }
    case HudMode::PulseWidth: {
        appendStr(leftMain, 0u, sizeof(leftMain), "WIDTH");
        if (hud.focusedKnob < kNumOscillators) {
            const uint8_t kv = hud.focusedKnob & 3u;
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix), static_cast<uint32_t>(kv + 1u));
            const VoiceParams& wvp = control.voices[kv];
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(rightText, 0u, sizeof(rightText), "MUTED");
            } else {
                appendStr(rightText, 0u, sizeof(rightText),
                          (absf(wvp.pulseWidth - 0.50f) <= 0.015f) ? "SQUARE" : "PULSE");
            }
        } else {
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " ALL");
            bool anyActive = false;
            bool anyPulse = false;
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                if (voiceAud(vi).muteBlend < 0.999f) {
                    anyActive = true;
                    if (absf(control.voices[vi].pulseWidth - 0.50f) > 0.015f) {
                        anyPulse = true;
                    }
                }
            }
            appendStr(rightText, 0u, sizeof(rightText),
                      !anyActive ? "ALL MUTED" : (anyPulse ? "PULSE" : "SQUARE"));
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
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " ALL");
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
                      !anyActive ? "ALL MUTED" : (anyDrifting ? "ANALOG 1/F" : "LOCKED"));
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
            const VoiceParams& avp = control.voices[kv];
            const float liveAmp =
                (avp.lfoTarget == LfoTarget::Amplitude)
                    ? applyLfoAmplitude(avp.amplitude, avp.lfoDepth, lfoOutput[kv])
                    : avp.amplitude;
            if (voiceAud(kv).muteBlend >= 0.999f) {
                appendStr(rightText, 0u, sizeof(rightText), "MUTED");
            } else if (liveAmp > kCleanAmpKnee + 0.003f) {
                const VoiceChannelGain cg = computeVoiceChannelGain(liveAmp);
                char multBuf[12]{};
                formatMultiplierX(multBuf, sizeof(multBuf), cg.preDrive);
                size_t rp = appendStr(rightText, 0u, sizeof(rightText), "DRIVE ");
                appendStr(rightText, rp, sizeof(rightText), multBuf);
            } else {
                appendStr(rightText, 0u, sizeof(rightText), "CLEAN VCA");
            }
        } else {
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " ALL");
            bool anyActive = false;
            float maxAmp = 0.0f;
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                const VoiceParams& avp = control.voices[vi];
                if (voiceAud(vi).muteBlend < 0.999f) {
                    anyActive = true;
                    const float la =
                        (avp.lfoTarget == LfoTarget::Amplitude)
                            ? applyLfoAmplitude(avp.amplitude, avp.lfoDepth, lfoOutput[vi])
                            : avp.amplitude;
                    if (la > maxAmp) {
                        maxAmp = la;
                    }
                }
            }
            if (!anyActive) {
                appendStr(rightText, 0u, sizeof(rightText), "ALL MUTED");
            } else if (maxAmp > kCleanAmpKnee + 0.003f) {
                const VoiceChannelGain cg = computeVoiceChannelGain(maxAmp);
                char multBuf[12]{};
                formatMultiplierX(multBuf, sizeof(multBuf), cg.preDrive);
                size_t rp = appendStr(rightText, 0u, sizeof(rightText), "DRIVE ");
                appendStr(rightText, rp, sizeof(rightText), multBuf);
            } else {
                appendStr(rightText, 0u, sizeof(rightText), "CLEAN VCA");
            }
        }
        break;
    }
    case HudMode::Panning:
        appendStr(leftMain, 0u, sizeof(leftMain), "PANNING");
        if (hud.focusedKnob < kNumOscillators) {
            size_t lp = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, lp, sizeof(leftSuffix),
                       static_cast<uint32_t>((hud.focusedKnob & 3u) + 1u));
        } else if (hud.editPageOpen && !hud.modifierHeld) {
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " ALL");
        }
        appendStr(rightText, 0u, sizeof(rightText),
                  control.global.monoCollapse ? "DUAL MONO" : "STEREO");
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
    case HudMode::UnassignedEncoder:
        appendStr(leftMain, 0u, sizeof(leftMain),
                  (hud.lastEncoder == EncoderId::Presets) ? "PRESETS ENCODER" : "SELECT ENCODER");
        appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
        break;
    case HudMode::KnobRate:
        appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
        appendStr(rightText, 0u, sizeof(rightText), "STANDARD");
        break;
    case HudMode::VoiceGate: {
        appendStr(leftMain, 0u, sizeof(leftMain), "GATE");
        size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
        appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
        const VoiceAudibility ga = voiceAud(v);
        const char* gateStatus = (ga.muteBlend >= 0.999f)
                                     ? "MUTED"
                                     : ((ga.subAudioBlend >= 0.5f) ? "SUB-AUDIO" : "ACTIVE");
        appendStr(rightText, 0u, sizeof(rightText), gateStatus);
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
    case HudMode::SnapshotSlot: {
        size_t p = appendStr(leftMain, 0u, sizeof(leftMain), "SNAPSHOT ");
        appendUInt(leftMain, p, sizeof(leftMain),
                   static_cast<uint32_t>((hud.snapshotSlot & 7u) + 1u));
        appendStr(rightText, 0u, sizeof(rightText), "EMPTY SLOT");
        break;
    }
    case HudMode::UnassignedKey: {
        if (hud.lastBlackKey <= BlackKeyId::Op5) {
            size_t p = appendStr(leftMain, 0u, sizeof(leftMain), "OP");
            p = appendUInt(leftMain, p, sizeof(leftMain),
                           static_cast<uint32_t>(hud.lastBlackKey) + 1u);
            appendStr(leftMain, p, sizeof(leftMain), " KEY");
        } else {
            appendStr(leftMain, 0u, sizeof(leftMain), "PIT KEY");
        }
        appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
        break;
    }
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
        appendStr(rightText, 0u, sizeof(rightText), hud.saveArmed ? "TAP KEY 9-16" : "CANCELLED");
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
        appendUInt(rightText, p, sizeof(rightText), static_cast<uint32_t>(kNumGlobalConfigItems));
        break;
    }
    case HudMode::UnassignedButton: {
        const char* swName = "PANEL";
        switch (hud.lastButton) {
        case ButtonId::Sel:
            swName = "SEL";
            break;
        case ButtonId::Env:
            swName = "ENV";
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

    const auto drawNormalRight = [&](const MonoStripCanvas& dstCv) noexcept {
        if (showHeaderBar) {
            const int barTxtW = measureSansText(kFontSansMedium, rightText, 0);
            dstCv.drawSans(kFontSansMedium, 230 - barTxtW, 6, rightText, 255u, 0);

            const float barWf = 34.0f;
            const float barXf = static_cast<float>(230 - barTxtW - 8) - barWf;
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

        if (hud.mode == HudMode::VoiceFrequency) {
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
        const float effFoc = foc * (1.0f - dim);
        const float posSigned = clampf(knobPosSignedSmooth[k], -1.0f, 1.0f);

        const float baseLbl = 154.0f + (72.0f - 154.0f) * dim;
        const auto lblGray = static_cast<uint8_t>(baseLbl + (255.0f - baseLbl) * effFoc + 0.5f);
        cv.drawSansCentered(kFontSansSmall, cx, 191, roleLbl, lblGray, 1);

        const float cxF = static_cast<float>(cx);
        const float bxf = cxF - 20.0f;
        const float bwf = 40.0f;
        const float byf = 207.5f;
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
        cv.drawSansCentered(kFontSansMedium, cx, 214, valStr, valGray, 0);
    }
}

} // namespace ruido
