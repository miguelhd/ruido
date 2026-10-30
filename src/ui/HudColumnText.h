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
    return mode == HudMode::VoiceFrequency || mode == HudMode::Amplitude ||
           mode == HudMode::Panning || mode == HudMode::VoiceFilter || mode == HudMode::VoiceGate ||
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
    case KnobBankMode::Amplitude: {
        const VoiceParams& vp = control.voices[k];
        size_t p = appendStr(roleLbl, 0u, roleCap, "OSC ");
        appendUInt(roleLbl, p, roleCap, static_cast<uint32_t>(k + 1u));
        const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                  ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[k])
                                  : vp.amplitude;
        formatPercent(valStr, valCap, vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f));
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
    case KnobBankMode::VoiceFilter: {
        const uint8_t refV = hud.focusedVoice & 3u;
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

inline void formatRateOverrideRightReadout(const EngineControlState& control, const HudState& hud,
                                           const std::array<float, kNumOscillators>& lfoOutput,
                                           char* rightText, size_t cap) noexcept {
    const uint8_t v = hud.focusedVoice & 3u;
    const VoiceParams& focVp = control.voices[v];
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;

    if (hud.mode == HudMode::VoiceFrequency && hud.focusedKnob < kNumOscillators) {
        size_t p = appendStr(rightText, 0u, cap, "OSC ");
        p = appendUInt(rightText, p, cap, static_cast<uint32_t>(v + 1u));
        p = appendStr(rightText, p, cap, "  ");
        const float pMul = computeVoicePitchRatio(control.voicePressure[v], pitchMode);
        const float liveHz = (focVp.lfoTarget == LfoTarget::Pitch)
                                 ? applyLfoPitchHz(focVp.frequencyHz, focVp.lfoDepth, lfoOutput[v])
                                 : focVp.frequencyHz;
        appendPrecisionHz(rightText, p, cap, liveHz * pMul);
        return;
    }
    if (hud.mode == HudMode::MasterLevel) {
        size_t p = appendStr(rightText, 0u, cap, "MASTER  ");
        char pct[8]{};
        formatPercent(pct, sizeof(pct), control.global.masterTrim);
        appendStr(rightText, p, cap, pct);
        return;
    }
    if (hud.mode == HudMode::RailCoupling) {
        size_t p = appendStr(rightText, 0u, cap, "COUPLING  ");
        char pct[8]{};
        formatPercent(pct, sizeof(pct), control.global.circuitCoupling);
        appendStr(rightText, p, cap, pct);
        return;
    }
    if (hud.focusedKnob < kNumOscillators && hud.knobBank != KnobBankMode::None) {
        char roleLbl[12]{};
        char valStr[16]{};
        GlobalColumnSummary gs{};
        formatColumnRoleAndValue(control, hud, lfoOutput, hud.focusedKnob, roleLbl, sizeof(roleLbl),
                                 valStr, sizeof(valStr), gs);
        if (roleLbl[0] != '\0') {
            size_t p = appendStr(rightText, 0u, cap, roleLbl);
            p = appendStr(rightText, p, cap, "  ");
            appendStr(rightText, p, cap, valStr);
            return;
        }
    }
    if (isGlobalHudMode(hud.mode)) {
        size_t p = appendStr(rightText, 0u, cap, "GLOBAL");
        appendStr(rightText, p, cap, globalTargetSuffix(hud.globalTarget));
        return;
    }
    if (hud.mode == HudMode::VoiceFilter) {
        size_t p = appendStr(rightText, 0u, cap, "FILTER OSC ");
        appendUInt(rightText, p, cap, static_cast<uint32_t>(v + 1u));
        return;
    }
    if (hud.mode == HudMode::VoiceLfo) {
        size_t p = appendStr(rightText, 0u, cap, "LFO OSC ");
        appendUInt(rightText, p, cap, static_cast<uint32_t>((hud.lfoVoiceFocus & 3u) + 1u));
        return;
    }
    if (hud.mode == HudMode::GlobalConfig) {
        appendStr(rightText, 0u, cap, "GLOBAL CONFIG");
        return;
    }
}

} // namespace ruido
