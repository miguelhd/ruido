#include "core/LedEngine.h"

#include "core/DspMath.h"
#include "core/GlobalControl.h"
#include "core/LfoMath.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

namespace {
// Exact integer multiples of 1000.0 s animPhase wrap (1500 & 350 cycles -> zero wrap discontinuity)
constexpr float kMutedPulseHz = 1.50f;
constexpr float kSubAudioBreathHz = 0.35f;
} // namespace

void LedSmoothState::reset(const EngineControlState& state) noexcept {
    animPhase = 0.0f;
    playStopRunSmooth = state.global.masterRun ? 1.0f : 0.0f;
    const float pitchMode = state.global.pitchPressureMode ? 1.0f : 0.0f;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceAudibility aud =
            computeVoiceAudibility(state.voices[i], state.voicePressure[i], pitchMode);
        muteBlendSmooth[i] = aud.muteBlend;
        subAudioBlendSmooth[i] = aud.subAudioBlend;
    }
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

    // 1. Raised Black Keys (11 monochrome white LEDs)
    // OP1..OP6 & PIT unassigned: 0% dark; Global (GLO): 25% rest, 100% held
    for (size_t i = 0u; i <= static_cast<size_t>(BlackKeyId::Pit); ++i) {
        outLeds.blackKeyLeds[i] = 0.0f;
    }
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Glo)] =
        ctx.blackKeyHeld[static_cast<size_t>(BlackKeyId::Glo)] ? 1.0f : 0.25f;
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Mono)] =
        ctx.state.global.monoCollapse ? 1.0f : 0.15f;
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Poly)] =
        ctx.state.global.monoCollapse ? 0.15f : 1.0f;
    // 11th black key (Scope Freeze): 0% at rest, 100% while freezing waveform
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)] =
        ctx.hud.scopeFrozen ? 1.0f : 0.0f;

    // 2. Bottom White Keys (16 monochrome white LEDs)
    const float pitchMode = ctx.state.global.pitchPressureMode ? 1.0f : 0.0f;
    const float muteAlpha = (dt > 0.0f) ? (1.0f - fastExp(-dt * 12.5f)) : 0.0f;
    const float subAlpha = (dt > 0.0f) ? (1.0f - fastExp(-dt * 9.0f)) : 0.0f;

    // Smooth C^2 analog sine waves:
    // - User-activated Muted beacon: 1.50 Hz (4% <-> 54%) to clearly signal why voice is silent
    // - Sub-audio (< 20 Hz) breathing: 0.35 Hz (~2.86 s calm breath, 4.3x slower than Muted)
    const float muteWave01 =
        0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(smooth.animPhase * kMutedPulseHz)));
    const float mutePulseLed = 0.04f + 0.50f * muteWave01;
    const float breathWave01 =
        0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(smooth.animPhase * kSubAudioBreathHz)));

    float maxLfoDepth = 0.0f;
    size_t dominantLfoBankIdx = 0u;
    float maxFilterEngagement = 0.0f;
    float maxPanEngagement = 0.0f;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = ctx.state.voices[i];
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
        if (!vp.muted || p > 0.001f) {
            const ModulatedFilterParams flt =
                computeLiveVoiceFilter(vp, p, pitchMode, lfoARaw, lfoBRaw);
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

        // Taper visual LFO modulation on key LEDs above 2..7 Hz so audio-rate FM/AM never strobes
        const float effRateA = computeLiveLfoARateHz(vp, lfoBRaw);
        const float effDepthA = computeLiveLfoADepth(vp, lfoBRaw);
        const float lfoVisWeightA = 1.0f - smoothstep01((effRateA - 2.0f) * 0.20f);
        const float lfoVisWeightB = 1.0f - smoothstep01((vp.lfoBRateHz - 2.0f) * 0.20f);
        if ((!vp.muted || p > 0.001f) && !ctx.state.global.monoCollapse) {
            float visPan = (vp.lfoTarget == LfoTarget::Pan && effDepthA > 0.0f)
                               ? applyLfoPan(vp.pan, effDepthA * lfoVisWeightA, lfoARaw)
                               : vp.pan;
            if (vp.lfoBTarget == LfoTarget::Pan && vp.lfoBDepth > 0.0f) {
                visPan = applyLfoPan(visPan, vp.lfoBDepth * lfoVisWeightB, lfoBRaw);
            }
            const float wideEng = clampf((absf(visPan) - 0.65f) * (1.0f / 0.35f), 0.0f, 1.0f);
            const float panModEng = ((vp.lfoTarget == LfoTarget::Pan && effDepthA > 1.0e-3f) ||
                                     (vp.lfoBTarget == LfoTarget::Pan && vp.lfoBDepth > 1.0e-3f))
                                        ? clampf(0.15f + 0.85f * absf(visPan), 0.0f, 1.0f)
                                        : 0.0f;
            const float panEng = (wideEng > panModEng) ? wideEng : panModEng;
            if (panEng > maxPanEngagement) {
                maxPanEngagement = panEng;
            }
        }
        float visAmp = (vp.lfoTarget == LfoTarget::Amplitude && effDepthA > 0.0f)
                           ? applyLfoAmplitude(vp.amplitude, effDepthA * lfoVisWeightA, lfoARaw)
                           : vp.amplitude;
        if (vp.lfoBTarget == LfoTarget::Amplitude && vp.lfoBDepth > 0.0f) {
            visAmp = applyLfoAmplitude(visAmp, vp.lfoBDepth * lfoVisWeightB, lfoBRaw);
        }
        const float visDepthA = (vp.lfoTarget == LfoTarget::Amplitude)
                                    ? clampf(effDepthA * lfoVisWeightA, 0.0f, 1.0f)
                                    : 0.0f;
        const float visDepthB = (vp.lfoBTarget == LfoTarget::Amplitude)
                                    ? clampf(vp.lfoBDepth * lfoVisWeightB, 0.0f, 1.0f)
                                    : 0.0f;
        const float visAmpDepth = (visDepthB > visDepthA) ? visDepthB : visDepthA;

        const VoiceAudibility aud = computeVoiceAudibility(vp, p, pitchMode);
        smooth.subAudioBlendSmooth[i] +=
            (aud.subAudioBlend - smooth.subAudioBlendSmooth[i]) * subAlpha;
        if (absf(aud.subAudioBlend - smooth.subAudioBlendSmooth[i]) < 1.0e-4f) {
            smooth.subAudioBlendSmooth[i] = aud.subAudioBlend;
        }

        smooth.muteBlendSmooth[i] += (aud.muteBlend - smooth.muteBlendSmooth[i]) * muteAlpha;
        if (absf(aud.muteBlend - smooth.muteBlendSmooth[i]) < 1.0e-4f) {
            smooth.muteBlendSmooth[i] = aud.muteBlend;
        }

        const float subBlend = smoothstep01(smooth.subAudioBlendSmooth[i]);
        const float muteBlend = smoothstep01(smooth.muteBlendSmooth[i]);

        const float restFloor = 0.60f - 0.42f * visAmpDepth;
        const float steadyAudibleLed = restFloor + (1.0f - restFloor) * visAmp;
        const float subBreathCrest = 0.52f + 0.28f * clampf(vp.amplitude, 0.0f, 1.0f);
        const float subBreathLed = 0.14f + (subBreathCrest - 0.14f) * breathWave01;
        const float unmutedLed = steadyAudibleLed + (subBreathLed - steadyAudibleLed) * subBlend;
        outLeds.whiteKeyLeds[i] = unmutedLed + (mutePulseLed - unmutedLed) * muteBlend;

        outLeds.whiteKeyLeds[4u + i] = kPadFilterBaseLed + (1.0f - kPadFilterBaseLed) * p;
    }

    float savePulse = 0.0f;
    if (ctx.hud.saveArmed) {
        const float cyc = smooth.animPhase * 2.5f -
                          static_cast<float>(static_cast<int32_t>(smooth.animPhase * 2.5f));
        const float tri = 1.0f - absf(2.0f * cyc - 1.0f);
        savePulse = 0.12f + 0.48f * smoothstep01(tri);
    }
    for (size_t s = 8u; s < 16u; ++s) {
        outLeds.whiteKeyLeds[s] = ctx.whiteKeyHeld[s] ? 0.75f : savePulse;
    }

    // 3. Panel Tactile Buttons (unused switches ENV/ARP/SEQ/REC strictly 0% dark)
    outLeds.buttonLeds.fill(0.0f);
    outLeds.stopLed = 0.0f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] =
        (ctx.hud.knobRate == KnobRate::Fine) ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] =
        (ctx.hud.knobRate == KnobRate::Coarse) ? 1.0f : 0.15f;

    const bool homeFreqActive =
        ctx.hud.freqPageOpen && !isGlobalKnobBank(ctx.hud.knobBank) && !ctx.hud.scopeFrozen;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Home)] =
        homeFreqActive ? 1.0f : (0.12f + 0.23f * ctx.hud.opacity);
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] = ctx.hud.saveArmed ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::GloRight)] =
        ctx.hud.configPageOpen ? 1.0f : 0.15f;

    // FX button (MS-20 Dual Filter Page): 100% open, 15% parked rest, 15%..85% engaged filter glow
    if (ctx.hud.fxPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] = 1.0f;
    } else if (maxFilterEngagement > 1.0e-3f) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] =
            clampf(0.15f + 0.70f * maxFilterEngagement, 0.15f, 0.85f);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Fx)] = 0.15f;
    }

    // SEL button (Stereo Panning Page): 100% open, 15% rest, 15%..75% wide/LFO stereo glow
    if (ctx.hud.panPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] = 1.0f;
    } else if (maxPanEngagement > 1.0e-3f) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] =
            clampf(0.15f + 0.60f * maxPanEngagement, 0.15f, 0.75f);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] = 0.15f;
    }

    // EDIT button: 100% strictly while EDIT is active, 15% dim whenever closed or in another mode
    const bool editActive =
        ctx.hud.editPageOpen && !isGlobalKnobBank(ctx.hud.knobBank) && !ctx.hud.scopeFrozen;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Edit)] = editActive ? 1.0f : 0.15f;

    if (ctx.hud.lfoPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 1.0f;
    } else if (maxLfoDepth > 1.0e-4f) {
        const uint8_t slotFocus = static_cast<uint8_t>(ctx.hud.lfoVoiceFocus & 7u);
        const size_t focV = lfoFocusVoice(slotFocus);
        const size_t focSub = lfoFocusSub(slotFocus);
        const size_t focusBankIdx = (lfoSlotDepth(ctx.state.voices[focV], focSub) > 1.0e-4f)
                                        ? lfoBankIndex(focV, focSub)
                                        : dominantLfoBankIdx;
        const float raw = ctx.telemetry.lfoOutput[focusBankIdx].load(std::memory_order_relaxed);
        const float u = 0.5f * (clampf(raw, -1.0f, 1.0f) + 1.0f);
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] =
            0.15f + (0.08f + 0.37f * clampf(maxLfoDepth, 0.0f, 1.0f)) * u;
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 0.15f;
    }

    // Dual-channel PLAY/STOP split cap (ADR-026 action affordance):
    // steady Green PLAY when quiet (OFF), steady Red STOP when sound is ON,
    // crossfading smoothly through warm Yellow/Amber during transitions
    const float runSmooth = clampf(smooth.playStopRunSmooth, 0.0f, 1.0f);
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::PlayStop)] = 1.0f - runSmooth;
    outLeds.stopLed = runSmooth;
}

} // namespace ruido
