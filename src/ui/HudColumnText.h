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
                                     const std::array<float, kNumOscillators>& lfoOutput, uint8_t k,
                                     char* roleLbl, size_t roleCap, char* valStr, size_t valCap,
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
                appendStr(valStr, 0u, valCap,
                          outGlobal.lfoTargetMixed ? "MIXED"
                                                   : lfoTargetName(control.voices[0].lfoTarget));
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

    switch (hud.knobBank) {
    case KnobBankMode::Frequency: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        const float pMul = computeVoicePitchRatio(control.voicePressure[k], pitchMode);
        const float lfoHz = (vp.lfoTarget == LfoTarget::Pitch)
                                ? applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoOutput[k])
                                : vp.frequencyHz;
        formatCompactHz(valStr, valCap, lfoHz * pMul);
        break;
    }
    case KnobBankMode::PulseWidth: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        outGlobal.bipolar = true;
        formatPercent(valStr, valCap, clampf(vp.pulseWidth, kPulseWidthMin, kPulseWidthMax));
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
            control.voices[refV], control.voicePressure[refV], pitchMode, lfoOutput[refV]);
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
        const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                  ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[k])
                                  : vp.amplitude;
        formatPercent(valStr, valCap,
                      (aud.muteBlend >= 0.999f) ? 0.0f : clampf(liveAmp, 0.0f, 1.0f));
        break;
    }
    case KnobBankMode::Panning: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        outGlobal.bipolar = true;
        const float livePan = (vp.lfoTarget == LfoTarget::Pan)
                                  ? applyLfoPan(vp.pan, vp.lfoDepth, lfoOutput[k])
                                  : vp.pan;
        formatPanValue(valStr, valCap, clampf(livePan * panScale, -1.0f, 1.0f));
        break;
    }
    case KnobBankMode::Lfo: {
        const VoiceParams& vp = control.voices[hud.lfoVoiceFocus & 3u];
        if (k == 0u) {
            appendStr(roleLbl, 0u, roleCap, "TARGET");
            appendStr(valStr, 0u, valCap, lfoTargetName(vp.lfoTarget));
        } else if (k == 1u) {
            appendStr(roleLbl, 0u, roleCap, "SHAPE");
            appendStr(valStr, 0u, valCap, lfoShapeLabel(vp.lfoShape));
        } else if (k == 2u) {
            appendStr(roleLbl, 0u, roleCap, "RATE");
            formatCompactHz(valStr, valCap, vp.lfoRateHz);
        } else {
            appendStr(roleLbl, 0u, roleCap, "DEPTH");
            formatPercent(valStr, valCap, vp.lfoDepth);
        }
        break;
    }
    default:
        break;
    }
}

} // namespace ruido
