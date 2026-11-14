#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

// Canonical per-oscillator acoustic circuit state shared across EVERY device surface:
// physical SOLO/MUTE key LEDs, panel FX/LFO LEDs, SEL keybed preview, main-screen OSC 1-4
// columns/filaments/beads/readouts, LEVEL stems, PAN bells, and oscilloscope trigger/presence.
struct VoiceCircuitState {
    float subAudioBlend = 0.0f; // 0.0 >= 25 Hz -> 1.0 <= 15 Hz (below human pitch fusion)
    float muteBlend = 0.0f;     // 0.0 audible -> 1.0 silent from ANY gate (patch/mute/solo/stutter)
    float inaudibleDim = 0.0f;  // max(muteBlend, subAudioBlend) in [0.0, 1.0]
    float gateGainMul = 1.0f;   // Combined post-VCA keybed + stutter gate multiplier in [0.0, 1.5]
    bool isMuted = false;       // True whenever no sound passes through this oscillator's VCA gate
    bool isSoloed = false;      // True iff solo is active on this oscillator AND it is audible
};

inline VoiceCircuitState evaluateVoiceCircuit(const EngineControlState& state, size_t voiceIdx,
                                              uint8_t stutterGateMask) noexcept {
    const size_t v = voiceIdx & 3u;
    const VoiceParams& vp = state.voices[v];
    const KeyboardModState& km = state.keyMod;
    const float pitchMode = state.global.pitchPressureMode ? 1.0f : 0.0f;
    const float p = clampf(state.voicePressure[v], 0.0f, 1.0f);

    const float effHz =
        clampf(vp.frequencyHz * computeVoicePitchRatio(p, pitchMode) * km.voicePitchMul[v],
               kAudioFreqMinHz * 0.25f, kAudioFreqMaxHz * 4.0f);
    const float subAudioBlend =
        1.0f - smoothstep01((fastLog2(effHz) - kSubAudioLoOct) * kInvSubAudioSpanOct);

    const bool hasAudibleAmp =
        (vp.amplitude > kSilentAmpThreshold) || (km.overloadSurge > kSilentAmpThreshold) ||
        (vp.lfoTarget == LfoTarget::Amplitude && vp.lfoDepth > kSilentAmpThreshold) ||
        (vp.lfoBTarget == LfoTarget::Amplitude && vp.lfoBDepth > kSilentAmpThreshold);

    float muteBlend = 0.0f;
    if (!hasAudibleAmp) {
        muteBlend = 1.0f;
    } else if (vp.muted) {
        const float lpgGate = (1.0f - pitchMode) * p * (0.65f + 0.35f * p);
        muteBlend = clampf(1.0f - lpgGate, 0.0f, 1.0f);
    }

    const float keyGain = clampf(km.voiceGainMul[v], 0.0f, 1.5f);
    if (keyGain < 0.999f) {
        const float keyDuck = 1.0f - clampf(keyGain, 0.0f, 1.0f);
        muteBlend = clampf(muteBlend + (1.0f - muteBlend) * keyDuck, 0.0f, 1.0f);
    }

    const float stutDepth = clampf(km.stutterDepth[v], 0.0f, 1.0f);
    const bool stutClosed = (stutDepth > 1.0e-4f) && (((stutterGateMask >> v) & 1u) == 0u);
    const float stutMul = stutClosed ? (1.0f - stutDepth) : 1.0f;
    if (stutMul < 0.999f) {
        const float stutDuck = 1.0f - stutMul;
        muteBlend = clampf(muteBlend + (1.0f - muteBlend) * stutDuck, 0.0f, 1.0f);
    }

    const float inaudibleDim = (muteBlend > subAudioBlend) ? muteBlend : subAudioBlend;
    const float gateGainMul = clampf(keyGain * stutMul, 0.0f, 1.5f);
    const bool isMuted = (muteBlend >= 0.98f);
    const bool isSoloed = (((km.soloMask >> v) & 1u) != 0u) && !isMuted;

    return {subAudioBlend, muteBlend, inaudibleDim, gateGainMul, isMuted, isSoloed};
}

inline float evaluateLiveOscAmplitude(const EngineControlState& state, size_t voiceIdx,
                                      float lfoARaw, float lfoBRaw,
                                      uint8_t stutterGateMask) noexcept {
    const size_t v = voiceIdx & 3u;
    const VoiceCircuitState vc = evaluateVoiceCircuit(state, v, stutterGateMask);
    if (vc.isMuted) {
        return 0.0f;
    }
    const float rawAmp =
        computeLiveVoiceAmplitude(state.voices[v], lfoARaw, lfoBRaw, state.keyMod.overloadSurge);
    return clampf(rawAmp * clampf(vc.gateGainMul, 0.0f, 1.0f), 0.0f, 1.0f);
}

// Single source of truth for Console SOLO (PIT..POLY, Black 6-9) & MUTE (Keys 13-16, White 12-15)
// LEDs across both physical PWM LEDs and the on-screen SEL keybed preview:
//   - Strictly binary switch state (ON = 100%, OFF = 0% dark) — never modulated by voice level,
//     sub-audio breathing, LFOs, peer solo ducking, or stutter gates.
inline void evaluateVoiceConsoleLeds(const KeyboardModState& km, size_t voiceIdx, float& outSoloLed,
                                     float& outMuteLed) noexcept {
    const uint8_t bit = static_cast<uint8_t>(1u << (voiceIdx & 3u));
    const bool muteOn = (km.muteMask & bit) != 0u;
    const bool soloOn = !muteOn && ((km.soloMask & bit) != 0u);
    outSoloLed = soloOn ? 1.0f : 0.0f;
    outMuteLed = muteOn ? 1.0f : 0.0f;
}

} // namespace ruido
