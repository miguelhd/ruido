#include "core/LedEngine.h"

#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

namespace {
// Exact integer multiples of 1000.0 s animPhase wrap (zero wrap discontinuity)
constexpr float kSubAudioBreathHz = 0.35f;
constexpr float kFxBreathHz = 0.45f;
} // namespace

void LedSmoothState::reset(const EngineControlState& state) noexcept {
    animPhase = 0.0f;
    playStopRunSmooth = state.global.masterRun ? 1.0f : 0.0f;
    fxSync01 = 0.0f;
    lfoSync01 = 0.0f;
    subBreathWave01 = 0.5f;
    savePulse01 = 0.5f;
    glintChannels.fill(0xFFu);
    glintPhaseQ8.fill(0u);
    nextGlintSlot = 0u;
    lastPhantomSeq = 0u;
    presetSceneLedQ16 = 0u;
}

void stepAndComputePanelLeds(float dtSec, const LedInputContext& ctx, LedSmoothState& smooth,
                             LedState& outLeds) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    smooth.animPhase += dt;
    if (smooth.animPhase >= 1000.0f) {
        smooth.animPhase -= 1000.0f;
    }

    const float runTarget = ctx.state.global.masterRun ? 1.0f : 0.0f;
    if (dt > 0.0f) {
        smooth.playStopRunSmooth +=
            (runTarget - smooth.playStopRunSmooth) * (1.0f - fastExp(-dt * 12.5f));
        if (absf(runTarget - smooth.playStopRunSmooth) < 1.0e-4f) {
            smooth.playStopRunSmooth = runTarget;
        }
    }

    smooth.subBreathWave01 =
        0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(smooth.animPhase * kSubAudioBreathHz)));
    const SavePageMode saveMode = ctx.hud.saveArmed ? ctx.hud.savePageMode : SavePageMode::Store;
    smooth.savePulse01 = evalSaveArmedPulse01(saveMode, smooth.animPhase);

    const float pitchMode = ctx.state.global.pitchPressureMode ? 1.0f : 0.0f;
    const uint8_t stutMask = ctx.telemetry.stutterGateMask.load(std::memory_order_relaxed);
    float maxLfoDepth = 0.0f;
    size_t dominantLfoBankIdx = 0u;
    float maxFilterEngagement = 0.0f;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = ctx.state.voices[i];
        const VoiceCircuitState vc = evaluateVoiceCircuit(ctx.state, i, stutMask);
        const float lfoARaw = ctx.telemetry.lfoOutput[i].load(std::memory_order_relaxed);
        const float lfoBRaw = ctx.telemetry.lfoOutput[4u + i].load(std::memory_order_relaxed);
        const float p = clampf(ctx.state.voicePressure[i], 0.0f, 1.0f);

        if (vp.lfoDepth > maxLfoDepth) {
            maxLfoDepth = vp.lfoDepth;
            dominantLfoBankIdx = i;
        }
        if (vp.lfoBDepth > maxLfoDepth) {
            maxLfoDepth = vp.lfoBDepth;
            dominantLfoBankIdx = 4u + i;
        }
        if (!vc.isMuted) {
            const FilterMacroContext fltMacros = FilterMacroContext::fromKeyMod(ctx.state.keyMod);
            const ModulatedFilterParams flt =
                computeLiveVoiceFilter(vp, p, pitchMode, lfoARaw, lfoBRaw, fltMacros);
            constexpr float kHpMinOct = cxLog2Pos(kFilterCutoffMinHz);
            constexpr float kLpMaxOct = cxLog2Pos(kFilterCutoffMaxHz);
            const float hpEng =
                (flt.hpCutoffHz > 2.05f)
                    ? clampf((fastLog2(flt.hpCutoffHz) - kHpMinOct) * (1.0f / 11.0f), 0.0f, 1.0f)
                    : 0.0f;
            const float lpEng =
                (flt.lpCutoffHz < 27900.0f)
                    ? clampf((kLpMaxOct - fastLog2(flt.lpCutoffHz)) * (1.0f / 11.0f), 0.0f, 1.0f)
                    : 0.0f;
            const float cutEng = (hpEng > lpEng) ? hpEng : lpEng;
            const float resEng =
                clampf((flt.hpResonance > flt.lpResonance) ? flt.hpResonance : flt.lpResonance,
                       0.0f, 1.0f);
            const float fltEng =
                clampf((cutEng > resEng) ? (cutEng + 0.35f * resEng) : (resEng + 0.35f * cutEng),
                       0.0f, 1.0f);
            if (fltEng > maxFilterEngagement) {
                maxFilterEngagement = fltEng;
            }
        }
    }

    if (ctx.state.keyMod.dubLpfDrop > maxFilterEngagement) {
        maxFilterEngagement = ctx.state.keyMod.dubLpfDrop;
    }
    if (ctx.state.keyMod.masterHpfSweep > maxFilterEngagement) {
        maxFilterEngagement = ctx.state.keyMod.masterHpfSweep;
    }
    if (ctx.state.keyMod.filterScream > maxFilterEngagement) {
        maxFilterEngagement = ctx.state.keyMod.filterScream;
    }

    // Unassigned controls rest at 0% dark; tapping any unassigned button or key brings its LED up
    // to the exact 15% steady glow of assigned controls, then fades dark in 120 ms.
    constexpr float kPhantomGlintDurationSec = 0.120f;
    if (dt > 0.0f) {
        const uint32_t decQ8 = static_cast<uint32_t>(
            clampf(dt * (1.0f / kPhantomGlintDurationSec), 0.0f, 1.0f) * 255.0f + 0.99f);
        for (uint8_t& ph : smooth.glintPhaseQ8) {
            ph = (ph > decQ8) ? static_cast<uint8_t>(ph - decQ8) : 0u;
        }
    }
    if (ctx.hud.phantomPinSeq != smooth.lastPhantomSeq) {
        smooth.lastPhantomSeq = ctx.hud.phantomPinSeq;
        const uint8_t ch = ctx.hud.phantomPinIndex;
        if (ch < kNumPhantomChannels) {
            size_t slotIdx = smooth.nextGlintSlot & 3u;
            bool reused = false;
            for (size_t i = 0u; i < smooth.glintChannels.size(); ++i) {
                if (smooth.glintChannels[i] == ch) {
                    slotIdx = i;
                    reused = true;
                    break;
                }
            }
            smooth.glintChannels[slotIdx] = ch;
            smooth.glintPhaseQ8[slotIdx] = 255u;
            if (!reused) {
                smooth.nextGlintSlot = static_cast<uint8_t>((smooth.nextGlintSlot + 1u) & 3u);
            }
        }
    }

    outLeds.buttonLeds.fill(0.0f);
    outLeds.stopLed = 0.0f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] =
        (ctx.hud.knobRate == KnobRate::Fine) ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] =
        (ctx.hud.knobRate == KnobRate::Coarse) ? 1.0f : 0.15f;

    const bool nonHomePageOpen = ctx.hud.fxPageOpen || ctx.hud.selPageOpen || ctx.hud.lfoPageOpen ||
                                 ctx.hud.editPageOpen || ctx.hud.panPageOpen ||
                                 ctx.hud.configPageOpen || ctx.hud.saveArmed;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Home)] = nonHomePageOpen ? 0.15f : 1.0f;
    if (ctx.hud.saveArmed) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] =
            computeSaveArmedButtonLed(ctx.hud.savePageMode, smooth.savePulse01);
    } else if (ctx.hud.snapshotDirty) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] =
            ctx.state.global.masterRun ? (0.12f + 0.16f * smooth.subBreathWave01) : 0.15f;
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] = 0.0f;
    }
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::GloRight)] =
        ctx.hud.configPageOpen ? 1.0f : 0.15f;

    // FX button (MS-20 Dual Filter Page): 100% open, 15% parked rest, 21%..37% tertiary breathing
    // when closed (matched 1-to-1 with HOME per-osc FX modulator dots)
    const float fxBreath01 =
        0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(smooth.animPhase * kFxBreathHz)));
    smooth.fxSync01 = (ctx.state.global.masterRun && maxFilterEngagement > 1.0e-3f)
                          ? clampf(0.70f * fxBreath01 + 0.30f * maxFilterEngagement, 0.0f, 1.0f)
                          : 0.0f;
    if (ctx.hud.fxPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] = 1.0f;
    } else if (ctx.state.global.masterRun && maxFilterEngagement > 1.0e-3f) {
        const float engGate = clampf(maxFilterEngagement * 50.0f, 0.0f, 1.0f);
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] =
            0.15f + engGate * (0.06f + 0.16f * smooth.fxSync01);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] = 0.15f;
    }

    // SEL button (Keyboard Model Status & Guidance):
    // 100% while SEL guidance page is open, live flare during keyboard punch-ins, 15% at rest
    if (ctx.hud.selPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] = 1.0f;
    } else if (ctx.state.global.masterRun && ctx.keyPunchEnergy01 > 1.0e-3f) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] =
            clampf(0.15f + 0.75f * ctx.keyPunchEnergy01, 0.15f, 0.90f);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] = 0.15f;
    }

    // EDIT button: 100% across EDIT Page 1 (Channel Mix) AND Page 2 (Panning), 15% when closed
    const bool editActive = (ctx.hud.editPageOpen || ctx.hud.panPageOpen) &&
                            (ctx.hud.knobBank != KnobBankMode::GlobalFilter) &&
                            !ctx.hud.scopeFrozen;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Edit)] = editActive ? 1.0f : 0.15f;

    float lfoWave01 = 0.0f;
    if (ctx.state.global.masterRun && maxLfoDepth > 1.0e-4f) {
        const uint8_t slotFocus = static_cast<uint8_t>(ctx.hud.lfoVoiceFocus & 7u);
        const size_t focV = lfoFocusVoice(slotFocus);
        const size_t focSub = lfoFocusSub(slotFocus);
        const size_t focusBankIdx = (lfoSlotDepth(ctx.state.voices[focV], focSub) > 1.0e-4f)
                                        ? lfoBankIndex(focV, focSub)
                                        : dominantLfoBankIdx;
        const float raw = ctx.telemetry.lfoOutput[focusBankIdx].load(std::memory_order_relaxed);
        lfoWave01 = 0.5f * (clampf(raw, -1.0f, 1.0f) + 1.0f);
    }
    // LFO button: 100% open, 15% parked rest, 21%..37% tertiary pulse when closed (matched 1-to-1
    // with HOME per-osc LFO A/B modulator dots)
    smooth.lfoSync01 = lfoWave01;
    if (ctx.hud.lfoPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 1.0f;
    } else if (ctx.state.global.masterRun && maxLfoDepth > 1.0e-4f) {
        const float depthGate = clampf(maxLfoDepth * 20.0f, 0.0f, 1.0f);
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] =
            0.15f + depthGate * (0.06f + 0.16f * smooth.lfoSync01);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 0.15f;
    }

    // Dual-channel PLAY/STOP split cap (ADR-026 action affordance)
    const float runSmooth = clampf(smooth.playStopRunSmooth, 0.0f, 1.0f);
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::PlayStop)] = 1.0f - runSmooth;
    outLeds.stopLed = runSmooth;
}

void applyPresetKnobSceneLeds(float dtSec, const HudState& hud, bool toastHoldActive,
                              LedSmoothState& smooth, LedState& ioLeds) noexcept {
    if (hud.shiftHeld || hud.saveArmed) {
        smooth.presetSceneLedQ16 = 0u;
        return;
    }
    const bool active =
        toastHoldActive && !hasSnapshotResultToast(hud) && (hud.mode == HudMode::SnapshotSlot);
    const float cur01 = static_cast<float>(smooth.presetSceneLedQ16) * (1.0f / 65535.0f);
    const float next01 = stepHudFadeLinear(cur01, active, clampf(dtSec, 0.0f, 0.25f));
    smooth.presetSceneLedQ16 = static_cast<uint16_t>(next01 * 65535.0f + 0.5f);
    if (smooth.presetSceneLedQ16 == 0u) {
        return;
    }
    const float blend = smoothstep01(next01);
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        const size_t wIdx = kSceneWhiteKeyBase + s;
        if (((hud.heldWhiteMask >> wIdx) & 1u) != 0u) {
            continue;
        }
        const float baseLed = ioLeds.whiteKeyLeds[wIdx];
        const float sceneLed = computeSceneKeyBrightness(s, hud, true);
        ioLeds.whiteKeyLeds[wIdx] = baseLed + (sceneLed - baseLed) * blend;
    }
}

void applyUnassignedLedGlints(const LedSmoothState& smooth, LedState& ioLeds) noexcept {
    for (size_t i = 0u; i < smooth.glintChannels.size(); ++i) {
        const uint8_t phQ8 = smooth.glintPhaseQ8[i];
        const uint8_t ch = smooth.glintChannels[i];
        if (phQ8 == 0u || ch >= kNumPhantomChannels) {
            continue;
        }
        const float glint =
            kLedAssignedRest * smoothstep01(static_cast<float>(phQ8) * (1.0f / 255.0f));
        if (ch < kPhantomBlackKeyBase) {
            const ButtonId btn = kPhantomButtonOrder[ch];
            float& dst = ioLeds.buttonLeds[static_cast<size_t>(btn)];
            if (glint > dst) {
                dst = glint;
            }
        } else if (ch < kPhantomWhiteKeyBase) {
            float& dst = ioLeds.blackKeyLeds[static_cast<size_t>(ch - kPhantomBlackKeyBase)];
            if (glint > dst) {
                dst = glint;
            }
        } else {
            float& dst = ioLeds.whiteKeyLeds[static_cast<size_t>(ch - kPhantomWhiteKeyBase)];
            if (glint > dst) {
                dst = glint;
            }
        }
    }
}

} // namespace ruido
