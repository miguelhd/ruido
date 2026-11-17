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

constexpr bool hasAudibleVoiceDrive(const VoiceParams& vp) noexcept {
    return (vp.amplitude > kSilentAmpThreshold) ||
           (vp.lfoTarget == LfoTarget::Amplitude && vp.lfoDepth > kSilentAmpThreshold) ||
           (vp.lfoBTarget == LfoTarget::Amplitude && vp.lfoBDepth > kSilentAmpThreshold);
}

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

    const bool hasAudibleAmp = hasAudibleVoiceDrive(vp) || (km.overloadSurge > kSilentAmpThreshold);

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

// Steady-state per-oscillator sound status LED for Keys 13-16 (`MUTE 1-4` & `SHIFT + SOLO 1-4`):
//   - `0.00f` (0% dark):    `PLAY/STOP` is OFF, `LEVEL == 0%`, muted (`MUTE`), or cut by `SOLO`.
//   - `0.50f` (50% half):   unmuted & active, base `frequencyHz < 20.0f` (sub-audio).
//   - `0.80f` (80% bright): unmuted & active, base `frequencyHz >= 20.0f` (capped at 80% so `SHIFT`
//                           at 100% remains unmistakably the brightest key on the right bay).
// Never dances with LFO waves, rhythm/stutter gates, or momentary pressure bends.
inline float evaluateVoiceSoundLedSteady(const EngineControlState& state,
                                         size_t voiceIdx) noexcept {
    if (!state.global.masterRun) {
        return 0.0f;
    }
    const size_t v = voiceIdx & 3u;
    const VoiceParams& vp = state.voices[v];
    const KeyboardModState& km = state.keyMod;
    const uint8_t bit = static_cast<uint8_t>(1u << v);
    if (vp.muted || !hasAudibleVoiceDrive(vp) || ((km.muteMask & bit) != 0u)) {
        return 0.0f;
    }
    if (km.soloMask != 0u && ((km.soloMask & bit) == 0u)) {
        return 0.0f;
    }
    return (vp.frequencyHz < 20.0f) ? 0.50f : 0.80f;
}

} // namespace ruido
