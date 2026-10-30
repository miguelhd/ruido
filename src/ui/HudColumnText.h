#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/GlobalControl.h"
#include "core/LfoMath.h"
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

inline void formatColumnRoleAndValue(const EngineControlState& control, const HudState& hud,
                                     const LfoSignalArray& lfoOutput, uint8_t k, char* roleLbl,
                                     size_t roleCap, char* valStr, size_t valCap,
                                     GlobalColumnSummary& outGlobal) noexcept {
    outGlobal = {};
    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;

    if (isGlobalKnobBank(hud.knobBank)) {
        outGlobal = evaluateGlobalColumn(hud.knobBank, k, control, lfoOutput);
        switch (hud.knobBank) {
        case KnobBankMode::GlobalFreq:
            if (k == 0u) {
                appendStr(roleLbl, 0u, roleCap, "PITCH");
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
                appendStr(roleLbl, 0u, roleCap, "WIDTH");
                formatPercent(valStr, valCap, outGlobal.primaryValue);
            } else {
                appendStr(roleLbl, 0u, roleCap, "BALANCE");
                formatSignedPercent(valStr, valCap, outGlobal.primaryValue);
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
                appendStr(roleLbl, 0u, roleCap, "AMP");
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
        const float pMul = computeVoicePitchRatio(control.voicePressure[k], pitchMode);
        formatCompactHz(valStr, valCap, computeLiveVoicePitchHz(vp, lfoA, lfoB) * pMul);
        break;
    }
    case KnobBankMode::PulseWidth: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        outGlobal.bipolar = true;
        formatPercent(
            valStr, valCap,
            clampf(computeLiveVoicePulseWidth(vp, lfoA, lfoB), kPulseWidthMin, kPulseWidthMax));
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
        const uint8_t refV = (hud.mode == HudMode::VoicePressure) ? (hud.focusedVoice & 3u)
                                                                  : (hud.fxVoiceFocus & 3u);
        const ModulatedFilterParams mod = computeLiveVoiceFilter(
            control.voices[refV], control.voicePressure[refV], pitchMode,
            lfoOutput[lfoBankIndex(refV, 0u)], lfoOutput[lfoBankIndex(refV, 1u)]);
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
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        const VoiceAudibility aud = computeVoiceAudibility(vp, control.voicePressure[k], pitchMode);
        const float liveAmp = computeLiveVoiceAmplitude(vp, lfoA, lfoB);
        formatPercent(valStr, valCap,
                      (aud.muteBlend >= 0.999f) ? 0.0f : clampf(liveAmp, 0.0f, 1.0f));
        break;
    }
    case KnobBankMode::Panning: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        outGlobal.bipolar = true;
        const float livePan = computeLiveVoicePan(vp, lfoA, lfoB);
        formatPanValue(valStr, valCap, clampf(livePan * panScale, -1.0f, 1.0f));
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
    default:
        break;
    }
}

} // namespace ruido
