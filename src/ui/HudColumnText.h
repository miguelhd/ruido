#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/GlobalControl.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/HudFormat.h"
#include "ui/HudGraphs.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

constexpr bool isKnobFocusableMode(HudMode mode) noexcept {
    return mode == HudMode::VoiceFrequency || mode == HudMode::PulseWidth ||
           mode == HudMode::VoiceDrift || mode == HudMode::VoiceFilter ||
           mode == HudMode::Amplitude || mode == HudMode::Panning || mode == HudMode::VoiceGate ||
           mode == HudMode::VoicePressure || mode == HudMode::VoiceLfo || isGlobalHudMode(mode);
}

inline void formatHeaderDriveValue(float ampVal, char* rightText, size_t rightCap) noexcept {
    if (ampVal > kCleanAmpKnee + 0.003f) {
        const VoiceChannelGain cg = computeVoiceChannelGain(ampVal);
        char multBuf[12]{};
        formatMultiplierX(multBuf, sizeof(multBuf), cg.preDrive);
        size_t rp = appendStr(rightText, 0u, rightCap, "DRIVE ");
        appendStr(rightText, rp, rightCap, multBuf);
    } else {
        appendStr(rightText, 0u, rightCap, "CLEAN VCA");
    }
}

constexpr const char* unassignedButtonName(ButtonId button) noexcept {
    switch (button) {
    case ButtonId::Env:
        return "ENV";
    case ButtonId::Arp:
        return "ARP";
    case ButtonId::Seq:
        return "SEQ";
    case ButtonId::Rec:
        return "REC";
    default:
        return "PANEL";
    }
}

inline void formatHeaderEnsembleDrive(const EngineControlState& control,
                                      const LfoSignalArray& lfoOutput, uint8_t stutterGateMask,
                                      char* rightText, size_t rightCap) noexcept {
    bool anyActive = false;
    float maxAmp = 0.0f;
    for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
        if (!evaluateVoiceCircuit(control, vi, stutterGateMask).isMuted) {
            anyActive = true;
            const float la =
                evaluateLiveOscAmplitude(control, vi, lfoOutput[lfoBankIndex(vi, 0u)],
                                         lfoOutput[lfoBankIndex(vi, 1u)], stutterGateMask);
            if (la > maxAmp) {
                maxAmp = la;
            }
        }
    }
    if (!anyActive) {
        appendStr(rightText, 0u, rightCap, "MUTED");
    } else {
        formatHeaderDriveValue(maxAmp, rightText, rightCap);
    }
}

inline void formatSnapshotHeader(const HudState& hud, char* leftMain, size_t leftMainCap,
                                 char* leftSuffix, size_t suffixCap, char* rightText,
                                 size_t rightCap, bool& outShowBar, float& outBarNorm) noexcept {
    outShowBar = false;
    outBarNorm = 0.0f;
    if (hud.mode == HudMode::SaveArmed) {
        appendStr(leftMain, 0u, leftMainCap, "STORE SCENE");
        appendStr(rightText, 0u, rightCap, hud.saveArmed ? "TAP SCENE 1-4" : "CANCELLED");
        return;
    }
    appendStr(leftMain, 0u, leftMainCap, "SCENE");
    size_t sp = appendStr(leftSuffix, 0u, suffixCap, " ");
    appendUInt(leftSuffix, sp, suffixCap, static_cast<uint32_t>((hud.snapshotSlot & 3u) + 1u));
    if (hud.snapshotSavedToast) {
        appendStr(rightText, 0u, rightCap, "SAVED");
    } else if (hud.snapshotGliding) {
        outBarNorm = clampf(hud.morphPosition, 0.0f, 1.0f);
        formatPercent(rightText, rightCap, outBarNorm);
        outShowBar = true;
    } else {
        appendStr(rightText, 0u, rightCap, hud.snapshotDirty ? "EDITED" : "ACTIVE");
    }
}

inline void formatHeaderPanMode(const EngineControlState& control, const HudState& hud,
                                uint8_t stutterGateMask, char* leftSuffix, size_t suffixCap,
                                char* rightText, size_t rightCap) noexcept {
    if (!control.global.monoCollapse && hud.focusedKnob < kNumOscillators) {
        const uint8_t kv = hud.focusedKnob & 3u;
        size_t lp = appendStr(leftSuffix, 0u, suffixCap, " OSC ");
        appendUInt(leftSuffix, lp, suffixCap, static_cast<uint32_t>(kv + 1u));
        const bool muted = evaluateVoiceCircuit(control, kv, stutterGateMask).isMuted;
        appendStr(rightText, 0u, rightCap, muted ? "MUTED" : "STEREO");
        return;
    }
    appendStr(rightText, 0u, rightCap, control.global.monoCollapse ? "MONO" : "STEREO");
}

inline void formatColumnRoleAndValue(const EngineControlState& control, const HudState& hud,
                                     const LfoSignalArray& lfoOutput, uint8_t stutterGateMask,
                                     uint8_t k, char* roleLbl, size_t roleCap, char* valStr,
                                     size_t valCap, GlobalColumnSummary& outGlobal) noexcept {
    outGlobal = {};
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;

    if (isGlobalKnobBank(hud.knobBank)) {
        outGlobal = evaluateGlobalColumn(hud.knobBank, k, control, lfoOutput, stutterGateMask);
        switch (hud.knobBank) {
        case KnobBankMode::GlobalFreq:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "FREQ");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 1u) {
                appendStr(roleLbl, 0u, roleCap, "SPREAD");
                formatSemitones(valStr, valCap, outGlobal.primaryValue, false);
            } else if (k == 2u) {
                appendStr(roleLbl, 0u, roleCap, "DETUNE");
                formatSignedCents(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "TILT");
                formatSemitones(valStr, valCap, outGlobal.primaryValue, true);
            }
            return;
        case KnobBankMode::GlobalFilter:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "HPF");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 1u) {
                appendStr(roleLbl, 0u, roleCap, "HP RES");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 2u) {
                appendStr(roleLbl, 0u, roleCap, "LPF");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "LP RES");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            }
            return;
        case KnobBankMode::GlobalLfo:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "TARGET");
                const auto uniTgt =
                    static_cast<LfoTarget>(static_cast<uint8_t>(outGlobal.primaryValue + 0.5f));
                appendStr(valStr, 0u, valCap,
                          outGlobal.lfoTargetMixed ? "MIXED" : lfoTargetName(uniTgt));
            } else if (k == 1u) {
                appendStr(roleLbl, 0u, roleCap, "SHAPE");
                appendStr(valStr, 0u, valCap,
                          outGlobal.lfoShapeMixed ? "MIXED"
                                                  : lfoShapeLabel(outGlobal.primaryValue));
            } else if (k == 2u) {
                appendStr(roleLbl, 0u, roleCap, "RATE");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "DEPTH");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            }
            return;
        case KnobBankMode::GlobalAmp:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "LEVEL");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 1u) {
                appendStr(roleLbl, 0u, roleCap, "TILT");
                formatSignedPercent(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 2u) {
                appendStr(roleLbl, 0u, roleCap, "SPREAD");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "PAN");
                if (control.global.monoCollapse) {
                    appendStr(valStr, 0u, valCap, "LOCKED");
                } else {
                    formatPanValue(valStr, valCap, outGlobal.primaryValue);
                }
            }
            return;
        case KnobBankMode::GlobalAll:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "FREQ");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 1u) {
                appendStr(roleLbl, 0u, roleCap, "FILTER");
                formatCompactHz(valStr, valCap, outGlobal.primaryValue);
            } else if (k == 2u) {
                appendStr(roleLbl, 0u, roleCap, "LFO");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "LEVEL");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            }
            return;
        default:
            return;
        }
    }

    const float lfoA = lfoOutput[lfoBankIndex(k, 0u)];
    const float lfoB = lfoOutput[lfoBankIndex(k, 1u)];

    switch (hud.knobBank) {
    case KnobBankMode::Frequency: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        const float pMul = computeVoicePitchRatio(control.voicePressure[k], pitchMode) *
                           control.keyMod.voicePitchMul[k];
        formatCompactHz(
            valStr, valCap,
            clampf(computeLiveVoicePitchHz(vp, lfoA, lfoB) * pMul, 0.005f, kAudioFreqMaxHz * 4.0f));
        break;
    }
    case KnobBankMode::PulseWidth: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        outGlobal.bipolar = true;
        formatPercent(
            valStr, valCap,
            clampf(computeLiveVoicePulseWidth(vp, lfoA, lfoB, control.keyMod.voicePwDelta[k]),
                   kPulseWidthMin, kPulseWidthMax));
        break;
    }
    case KnobBankMode::VoiceDrift: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        formatPercent(valStr, valCap, clampf(vp.drift, 0.0f, 1.0f));
        break;
    }
    case KnobBankMode::VoiceFilter: {
        const uint8_t refV = resolveFilterViewVoice(hud);
        const ModulatedFilterParams mod = computeLiveVoiceFilter(
            control.voices[refV], control.voicePressure[refV], pitchMode,
            lfoOutput[lfoBankIndex(refV, 0u)], lfoOutput[lfoBankIndex(refV, 1u)],
            control.keyMod.dubLpfDrop, control.keyMod.overloadSurge, control.keyMod.filterScream);
        if (k == 0u) {
            appendStr(roleLbl, 0u, roleCap, "HPF");
            formatCompactHz(valStr, valCap, mod.hpCutoffHz);
        } else if (k == 1u) {
            appendStr(roleLbl, 0u, roleCap, "HP RES");
            formatPercent(valStr, valCap, clampf(mod.hpResonance, 0.0f, 1.0f));
        } else if (k == 2u) {
            appendStr(roleLbl, 0u, roleCap, "LPF");
            formatCompactHz(valStr, valCap, mod.lpCutoffHz);
        } else {
            appendStr(roleLbl, 0u, roleCap, "LP RES");
            formatPercent(valStr, valCap, clampf(mod.lpResonance, 0.0f, 1.0f));
        }
        break;
    }
    case KnobBankMode::Amplitude: {
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        formatPercent(valStr, valCap,
                      evaluateLiveOscAmplitude(control, k, lfoA, lfoB, stutterGateMask));
        break;
    }
    case KnobBankMode::Panning: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        if (control.global.monoCollapse && control.keyMod.spatialFocus <= 0.001f) {
            outGlobal.disabled = true;
            appendStr(valStr, 0u, valCap, "LOCKED");
        } else {
            outGlobal.bipolar = true;
            const float effPan =
                applySpatialFocusPan(computeLiveVoicePan(vp, lfoA, lfoB), k,
                                     control.global.monoCollapse, control.keyMod.spatialFocus);
            formatPanValue(valStr, valCap, effPan);
        }
        break;
    }
    case KnobBankMode::Lfo: {
        const uint8_t slot = hud.lfoVoiceFocus & 7u;
        const uint8_t lv = lfoFocusVoice(slot);
        const uint8_t sub = lfoFocusSub(slot);
        const VoiceParams& vp = control.voices[lv];
        const float lfoBRaw = lfoOutput[lfoBankIndex(lv, 1u)];
        if (k == 0u) {
            appendStr(roleLbl, 0u, roleCap, "TARGET");
            appendStr(valStr, 0u, valCap, lfoTargetName(lfoSlotTarget(vp, sub)));
        } else if (k == 1u) {
            appendStr(roleLbl, 0u, roleCap, "SHAPE");
            appendStr(valStr, 0u, valCap, lfoShapeLabel(lfoSlotShape(vp, sub)));
        } else if (k == 2u) {
            appendStr(roleLbl, 0u, roleCap, "RATE");
            const float liveRate = (sub == 0u) ? computeLiveLfoARateHz(vp, lfoBRaw) : vp.lfoBRateHz;
            formatCompactHz(valStr, valCap, liveRate);
        } else {
            appendStr(roleLbl, 0u, roleCap, "DEPTH");
            const float liveDepth = (sub == 0u) ? computeLiveLfoADepth(vp, lfoBRaw) : vp.lfoBDepth;
            formatPercent(valStr, valCap, liveDepth);
        }
        break;
    }
    case KnobBankMode::Snapshot: {
        size_t p = appendStr(roleLbl, 0u, roleCap, "SCENE ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        formatPercent(valStr, valCap, clampf(hud.snapshotWeights[k], 0.0f, 1.0f));
        break;
    }
    default:
        break;
    }
}

} // namespace ruido
