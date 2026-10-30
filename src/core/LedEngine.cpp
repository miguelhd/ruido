#include "core/LedEngine.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

void computePanelAndKeybedLeds(const LedInputContext& ctx, LedState& outLeds) noexcept {
    // 1. Raised Black Keys (11 monochrome white LEDs)
    // Contiguous 4-key per-voice MS-20 filter family (OP1..OP4): 15% rest, 100% held
    for (size_t i = 0u; i < 4u; ++i) {
        outLeds.blackKeyLeds[i] = ctx.blackKeyHeld[i] ? 1.0f : 0.15f;
    }
    // Spatial (OP6 Panning) & Global (GLO) landmarks: 25% rest, 100% held; OP5 & PIT unassigned 0%
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op5)] = 0.0f;
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op6)] =
        ctx.blackKeyHeld[static_cast<size_t>(BlackKeyId::Op6)] ? 1.0f : 0.25f;
    outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Pit)] = 0.0f;
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
    const float padBaseLed =
        ctx.state.global.pitchPressureMode ? kPadPitchBaseLed : kPadFilterBaseLed;
    float maxPadPressure = 0.0f;

    float maxLfoDepth = 0.0f;
    size_t dominantLfoVoice = 0u;
    float maxOverdriveNorm = 0.0f;
    float maxOverdrivePeak = 0.0f;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = ctx.state.voices[i];
        const float lfoRaw = ctx.telemetry.lfoOutput[i].load(std::memory_order_relaxed);
        const float effHz = (vp.lfoTarget == LfoTarget::Pitch && vp.lfoDepth > 0.0f)
                                ? applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoRaw)
                                : vp.frequencyHz;
        const float effAmp = (vp.lfoTarget == LfoTarget::Amplitude && vp.lfoDepth > 0.0f)
                                 ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoRaw)
                                 : vp.amplitude;
        const float ampDepth =
            (vp.lfoTarget == LfoTarget::Amplitude) ? clampf(vp.lfoDepth, 0.0f, 1.0f) : 0.0f;
        const float gate = ctx.telemetry.oscGateHigh[i].load(std::memory_order_relaxed);
        const float vPeak = ctx.telemetry.voicePeak[i].load(std::memory_order_relaxed);

        if (vp.lfoDepth > maxLfoDepth) {
            maxLfoDepth = vp.lfoDepth;
            dominantLfoVoice = i;
        }
        if (!vp.muted && effAmp > kCleanAmpKnee) {
            const float driveNorm =
                clampf((effAmp - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee)), 0.0f, 1.0f);
            if (driveNorm > maxOverdriveNorm) {
                maxOverdriveNorm = driveNorm;
            }
            if (vPeak > maxOverdrivePeak) {
                maxOverdrivePeak = vPeak;
            }
        }

        if (vp.muted) {
            outLeds.whiteKeyLeds[i] = 0.10f;
        } else if (effHz < 20.0f) {
            const float peakBright = 0.60f + 0.40f * effAmp;
            outLeds.whiteKeyLeds[i] = 0.12f + (peakBright - 0.12f) * clampf(gate, 0.0f, 1.0f);
        } else {
            const float restFloor = 0.60f - 0.42f * ampDepth;
            outLeds.whiteKeyLeds[i] = restFloor + (1.0f - restFloor) * effAmp;
        }

        const float p = clampf(ctx.state.voicePressure[i], 0.0f, 1.0f);
        if (p > maxPadPressure) {
            maxPadPressure = p;
        }
        outLeds.whiteKeyLeds[4u + i] = padBaseLed + (1.0f - padBaseLed) * p;
    }

    float savePulse = 0.0f;
    if (ctx.hud.saveArmed) {
        const float cyc =
            ctx.animPhase * 2.5f - static_cast<float>(static_cast<int32_t>(ctx.animPhase * 2.5f));
        const float tri = 1.0f - absf(2.0f * cyc - 1.0f);
        savePulse = 0.12f + 0.48f * smoothstep01(tri);
    }
    for (size_t s = 8u; s < 16u; ++s) {
        outLeds.whiteKeyLeds[s] = ctx.whiteKeyHeld[s] ? 0.75f : savePulse;
    }

    // 3. Panel Tactile Buttons (unused FM/seq buttons strictly 0% dark)
    outLeds.buttonLeds.fill(0.0f);
    outLeds.stopLed = 0.0f;
    // Visually couple SEL to White Keys 5-8 via identical rest brightness & live touch flare
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Sel)] =
        padBaseLed + (1.0f - padBaseLed) * maxPadPressure;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] =
        (ctx.hud.knobRate == KnobRate::Fine) ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] =
        (ctx.hud.knobRate == KnobRate::Coarse) ? 1.0f : 0.15f;

    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Home)] = 0.12f + 0.23f * ctx.hud.opacity;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::Save)] = ctx.hud.saveArmed ? 1.0f : 0.15f;
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::GloRight)] =
        ctx.hud.configPageOpen ? 1.0f : 0.15f;

    // EDIT button (Channel Level & MS-20 Drive): 100% when open, 15% clean rest, live flare > 70%
    if (ctx.hud.editPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Edit)] = 1.0f;
    } else if (maxOverdriveNorm > 1.0e-4f) {
        const float peakNorm = clampf(maxOverdrivePeak, 0.0f, 1.0f);
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Edit)] =
            clampf(0.15f + (0.10f + 0.60f * peakNorm) * maxOverdriveNorm, 0.15f, 0.85f);
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Edit)] = 0.15f;
    }

    if (ctx.hud.lfoPageOpen) {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 1.0f;
    } else if (maxLfoDepth > 1.0e-4f) {
        const size_t focusV = (ctx.hud.lfoVoiceFocus < kNumOscillators &&
                               ctx.state.voices[ctx.hud.lfoVoiceFocus].lfoDepth > 1.0e-4f)
                                  ? static_cast<size_t>(ctx.hud.lfoVoiceFocus)
                                  : dominantLfoVoice;
        const float raw = ctx.telemetry.lfoOutput[focusV].load(std::memory_order_relaxed);
        const float u = 0.5f * (clampf(raw, -1.0f, 1.0f) + 1.0f);
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] =
            0.15f + (0.08f + 0.37f * clampf(maxLfoDepth, 0.0f, 1.0f)) * u;
    } else {
        outLeds.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 0.15f;
    }

    // Dual-channel PLAY/STOP split cap (ADR-026 action affordance):
    // steady Green PLAY when quiet (OFF), steady Red STOP when sound is ON,
    // crossfading smoothly through warm Yellow/Amber during transitions
    const float runSmooth = clampf(ctx.playStopRunSmooth, 0.0f, 1.0f);
    outLeds.buttonLeds[static_cast<size_t>(ButtonId::PlayStop)] = 1.0f - runSmooth;
    outLeds.stopLed = runSmooth;
}

} // namespace ruido
