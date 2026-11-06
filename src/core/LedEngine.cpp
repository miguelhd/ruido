#include "core/LedEngine.h"

#include "core/DspMath.h"
#include "core/FilterModMath.h"
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
    phantomGlint.fill(0.0f);
    lastPhantomSeq = 0u;
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
            const FilterMacroContext fltMacros{
                ctx.state.keyMod.dubLpfDrop,
                ctx.state.keyMod.overloadSurge,
                ctx.state.keyMod.filterScream,
            };
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
    if (ctx.state.keyMod.filterScream > maxFilterEngagement) {
        maxFilterEngagement = ctx.state.keyMod.filterScream;
    }

    // Panel Tactile Buttons (unconnected switches ENV/ARP/SEQ/REC rest at 0% dark; tapping one
    // brings its LED up to the exact 15% steady glow of assigned buttons, then fades dark in 120
    // ms)
    constexpr float kPhantomGlintDurationSec = 0.120f;
    constexpr float kAssignedRestGlow = 0.15f;
    if (dt > 0.0f) {
        const float step = dt * (1.0f / kPhantomGlintDurationSec);
        for (float& g : smooth.phantomGlint) {
            g = (g > step) ? (g - step) : 0.0f;
        }
    }
    if (ctx.hud.phantomPinSeq != smooth.lastPhantomSeq) {
        smooth.lastPhantomSeq = ctx.hud.phantomPinSeq;
        smooth.phantomGlint[ctx.hud.phantomPinIndex & 3u] = 1.0f;
    }

    outLeds.buttonLeds.fill(0.0f);
    outLeds.stopLed = 0.0f;
    for (size_t i = 0u; i < kPhantomPinButtons.size(); ++i) {
        outLeds.buttonLeds[static_cast<size_t>(kPhantomPinButtons[i])] =
            kAssignedRestGlow * smoothstep01(smooth.phantomGlint[i]);
    }
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] =
        (ctx.hud.knobRate == KnobRate::Fine) ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] =
        (ctx.hud.knobRate == KnobRate::Coarse) ? 1.0f : 0.15f;

    const bool homeFreqActive = ctx.hud.freqPageOpen &&
                                (ctx.hud.knobBank != KnobBankMode::GlobalFilter) &&
                                !ctx.hud.scopeFrozen;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Home)] =
        homeFreqActive ? 1.0f : (0.12f + 0.23f * ctx.hud.opacity);
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] = ctx.hud.saveArmed ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::GloRight)] =
        ctx.hud.configPageOpen ? 1.0f : 0.15f;

    // FX button (MS-20 Dual Filter Page): 100% open, 15% parked rest, 21%..37% tertiary breathing
    // when closed (matched 1-to-1 with HOME per-osc FX modulator dots)
    const float fxBreath01 =
        0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(smooth.animPhase * kFxBreathHz)));
    smooth.fxSync01 = (maxFilterEngagement > 1.0e-3f)
                          ? clampf(0.70f * fxBreath01 + 0.30f * maxFilterEngagement, 0.0f, 1.0f)
                          : 0.0f;
    if (ctx.hud.fxPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] = 1.0f;
    } else if (maxFilterEngagement > 1.0e-3f) {
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
    } else if (ctx.keyPunchEnergy01 > 1.0e-3f) {
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
    if (maxLfoDepth > 1.0e-4f) {
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
    } else if (maxLfoDepth > 1.0e-4f) {
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

} // namespace ruido
