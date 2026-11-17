#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/KeyPunchSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/HudFormat.h"
#include "ui/HudGlobalViews.h"
#include "ui/HudGraphs.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

struct GlobalColumnSummary {
    float posSigned = 0.0f;
    std::array<float, kNumOscillators> voiceSigned{0.0f, 0.0f, 0.0f, 0.0f};
    bool bipolar = false;
    bool disabled = false;
    bool hasVoiceSpread = false;
    float primaryValue = 0.0f;
};

inline GlobalColumnSummary evaluateGlobalFilterColumn(uint8_t knobIdx,
                                                      const EngineControlState& control,
                                                      const LfoSignalArray& lfoOutput,
                                                      uint8_t stutterGateMask = 0x0Fu) noexcept {
    GlobalColumnSummary s{};
    const uint8_t k = knobIdx & 3u;
    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    constexpr float kFltLogLo = cxLog2Pos(kFilterCutoffMinHz);
    constexpr float kInvFltLogRange = 1.0f / (cxLog2Pos(kFilterCutoffMaxHz) - kFltLogLo);
    float sumVal = 0.0f;
    float lo = 1.0f;
    float hi = -1.0f;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = control.voices[i];
        const float pluckEnv = (((stutterGateMask >> i) & 1u) != 0u) ? 0.55f : 0.0f;
        const ModulatedFilterParams mod = computeLiveVoiceFilter(
            vp, control.voicePressure[i], pitchMode, lfoOutput[i], lfoOutput[4u + i],
            FilterMacroContext::fromKeyMod(control.keyMod, pluckEnv));
        float vNorm = 0.0f;
        if (k == 0u) {
            const float lg = fastLog2(mod.hpCutoffHz);
            sumVal += lg;
            vNorm = clampf((lg - kFltLogLo) * kInvFltLogRange, 0.0f, 1.0f);
        } else if (k == 1u) {
            sumVal += mod.hpResonance;
            vNorm = clampf(mod.hpResonance, 0.0f, 1.0f);
        } else if (k == 2u) {
            const float lg = fastLog2(mod.lpCutoffHz);
            sumVal += lg;
            vNorm = clampf((lg - kFltLogLo) * kInvFltLogRange, 0.0f, 1.0f);
        } else {
            sumVal += mod.lpResonance;
            vNorm = clampf(mod.lpResonance, 0.0f, 1.0f);
        }
        const float vs = 2.0f * vNorm - 1.0f;
        s.voiceSigned[i] = vs;
        if (vs < lo) {
            lo = vs;
        }
        if (vs > hi) {
            hi = vs;
        }
    }
    sumVal *= 0.25f;
    if (k == 0u || k == 2u) {
        s.primaryValue = fastExp2(sumVal);
        const float norm = clampf((sumVal - kFltLogLo) * kInvFltLogRange, 0.0f, 1.0f);
        s.posSigned = 2.0f * norm - 1.0f;
    } else {
        s.primaryValue = clampf(sumVal, 0.0f, 1.0f);
        s.posSigned = 2.0f * s.primaryValue - 1.0f;
    }
    s.hasVoiceSpread = (hi - lo) > 0.045f;
    return s;
}

constexpr bool isKnobFocusableMode(HudMode mode) noexcept {
    return mode == HudMode::VoiceFrequency || mode == HudMode::PulseWidth ||
           mode == HudMode::VoiceDrift || mode == HudMode::VoiceFilter ||
           mode == HudMode::Amplitude || mode == HudMode::Panning || mode == HudMode::VoiceGate ||
           mode == HudMode::VoicePressure || mode == HudMode::VoiceLfo;
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
                                 size_t rightCap, bool& outShowBar, float& outBarNorm,
                                 bool* outShowSceneDots = nullptr) noexcept {
    outShowBar = false;
    outBarNorm = 0.0f;
    if (outShowSceneDots != nullptr) {
        *outShowSceneDots = false;
    }
    if (leftMainCap > 0u) {
        leftMain[0] = '\0';
    }
    if (suffixCap > 0u) {
        leftSuffix[0] = '\0';
    }
    if (rightCap > 0u) {
        rightText[0] = '\0';
    }
    appendStr(leftMain, 0u, leftMainCap, "SCENE");
    if (hud.mode == HudMode::SaveArmed) {
        appendStr(leftSuffix, 0u, suffixCap, hud.saveDeleteMode ? " 2/2" : " 1/2");
        return;
    }
    const bool isKeyRecall =
        ((hud.mode == HudMode::KeyboardPlay || hud.mode == HudMode::ShiftMap) &&
         hud.activePunch == KeyPunchKind::SnapshotRecall);
    if (hud.snapshotOccupiedMask == 0u && !isKeyRecall && !hud.snapshotDeletedToast) {
        appendStr(rightText, 0u, rightCap, "ALL EMPTY");
        return;
    }

    uint8_t activeWeights = 0u;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (((hud.snapshotOccupiedMask & (1u << s)) != 0u) && hud.snapshotWeights[s] > 0.01f) {
            ++activeWeights;
        }
    }

    const uint8_t activeSlot = hud.snapshotSlot & 3u;
    const bool activeOcc = (hud.snapshotOccupiedMask & (1u << activeSlot)) != 0u;
    const uint8_t dispSlot =
        isKeyRecall ? (hud.activePunchVoice & 3u)
                    : ((hud.snapshotSavedToast || hud.snapshotDeletedToast || !activeOcc)
                           ? (hud.snapshotToastSlot & 3u)
                           : activeSlot);
    const bool dispOcc = (hud.snapshotOccupiedMask & (1u << dispSlot)) != 0u;

    if (hud.snapshotDeletedToast) {
        size_t sp = appendStr(leftSuffix, 0u, suffixCap, " ");
        appendUInt(leftSuffix, sp, suffixCap, static_cast<uint32_t>(dispSlot + 1u));
        appendStr(rightText, 0u, rightCap, "DELETED");
        return;
    }
    if (hud.snapshotSavedToast) {
        size_t sp = appendStr(leftSuffix, 0u, suffixCap, " ");
        appendUInt(leftSuffix, sp, suffixCap, static_cast<uint32_t>(dispSlot + 1u));
        appendStr(rightText, 0u, rightCap, "SAVED");
        return;
    }
    if (!dispOcc && (isKeyRecall || activeWeights == 0u)) {
        size_t sp = appendStr(leftSuffix, 0u, suffixCap, " ");
        appendUInt(leftSuffix, sp, suffixCap, static_cast<uint32_t>(dispSlot + 1u));
        appendStr(rightText, 0u, rightCap, "EMPTY");
        return;
    }

    if (outShowSceneDots != nullptr) {
        *outShowSceneDots = true;
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

    if (hud.knobBank == KnobBankMode::GlobalFilter) {
        outGlobal = evaluateGlobalFilterColumn(k, control, lfoOutput, stutterGateMask);
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
            FilterMacroContext::fromKeyMod(control.keyMod));
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
            const LfoTarget tgt = lfoSlotTarget(vp, sub);
            if (tgt == LfoTarget::LfoARate || tgt == LfoTarget::LfoADepth) {
                valStr[0] = 'A';
                valStr[1] = static_cast<char>('1' + (lv & 3u));
                appendStr(valStr, 2u, valCap, (tgt == LfoTarget::LfoARate) ? " RATE" : " DEPTH");
            } else {
                appendStr(valStr, 0u, valCap, lfoTargetName(tgt));
            }
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
