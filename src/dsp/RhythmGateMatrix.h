#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/ParameterSmoother.h"
#include "core/Parameters.h"
#include "core/RhythmMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Core 1 audio-rate (44.118 kHz) master rhythm clock, clock dividers, 16-step Euclidean gates,
// free-speed stutter choppers, sidechain pump, round-robin chase, and percussive pluck envelopes.
struct RhythmClockState {
    uint32_t barPhaseU32 = 0u;
    float clockBpm = 120.0f;
    float warpedBar01 = 0.0f;
    std::array<uint8_t, kNumOscillators> voiceMode{0u, 0u, 0u, 0u};
    std::array<uint8_t, kNumOscillators> voiceParam{0u, 0u, 0u, 0u};
    OnePoleSmootherBank<6> macroSmoothers{}; // [0..5] = pump, chase, tempo, pluck, short, swing
    OnePoleSmoother swingPanSmoother{};
    uint8_t lastResetSeq = 0u;

    void init(float sampleRate, float slewSec) noexcept {
        barPhaseU32 = 0u;
        clockBpm = 120.0f;
        warpedBar01 = 0.0f;
        voiceMode = {0u, 0u, 0u, 0u};
        voiceParam = {0u, 0u, 0u, 0u};
        lastResetSeq = 0u;
        macroSmoothers.setTimeConstant(slewSec, sampleRate);
        for (size_t k = 0u; k < 6u; ++k) {
            macroSmoothers.snapChannel(k, 0.0f);
        }
        swingPanSmoother.init(0.0f, 0.0045f, sampleRate);
    }

    void updateTargets(const EngineControlState& state, bool snapImmediate) noexcept {
        clockBpm = clampf(state.global.clockBpm, kMinInternalClockBpm, kMaxInternalClockBpm);
        if (state.global.clockResetSeq != lastResetSeq) {
            lastResetSeq = state.global.clockResetSeq;
            barPhaseU32 = 0u;
        }
        voiceMode = state.keyMod.rhythmMode;
        voiceParam = state.keyMod.rhythmParam;

        const std::array<float, 6> targets = {
            clampf(state.keyMod.rhythmPump, 0.0f, 1.0f),
            clampf(state.keyMod.rhythmChase, 0.0f, 1.0f),
            clampf(state.keyMod.rhythmTempo, 0.0f, 1.0f),
            clampf(state.keyMod.rhythmPluck, 0.0f, 1.0f),
            clampf(state.keyMod.rhythmShort, 0.0f, 1.0f),
            clampf(state.keyMod.rhythmSwing, 0.0f, 1.0f),
        };

        if (snapImmediate) {
            for (size_t k = 0u; k < 6u; ++k) {
                macroSmoothers.snapChannel(k, targets[k]);
            }
            swingPanSmoother.snapTo(0.0f);
        } else {
            for (size_t k = 0u; k < 6u; ++k) {
                macroSmoothers.setTarget(k, targets[k]);
            }
        }
    }
};

struct RhythmFrameResult {
    std::array<float, kNumOscillators> vcaGateTarget{1.0f, 1.0f, 1.0f, 1.0f};
    std::array<float, kNumOscillators> pluckEnv01{0.0f, 0.0f, 0.0f, 0.0f};
    float pluckDepth = 0.0f;
    float swingPanOffset = 0.0f;
    uint8_t gateMask = 0x0Fu;
};

inline RhythmFrameResult stepRhythmFrame(RhythmClockState& rs,
                                         std::array<uint32_t, kNumOscillators>& stutterPhases,
                                         const std::array<float, kNumOscillators>& stutterRatesHz,
                                         const std::array<float, kNumOscillators>& stutterDepths,
                                         float hzToBasePhaseU32, bool stasisFreeze) noexcept {
    RhythmFrameResult out{};

    const float pump = rs.macroSmoothers.next(0u);
    const float chase = rs.macroSmoothers.next(1u);
    const float tempo = rs.macroSmoothers.next(2u);
    const float pluck = rs.macroSmoothers.next(3u);
    const float shrt = rs.macroSmoothers.next(4u);
    const float swing = rs.macroSmoothers.next(5u);
    out.pluckDepth = pluck;

    // 1. Advance 4-beat bar clock (1 bar = 4 quarter-note beats = 240 / BPM seconds)
    const float effBpm = clampf(rs.clockBpm * computeTempoMultiplier(tempo), kMinInternalClockBpm,
                                kMaxEffectiveClockBpm);
    const float barHz = effBpm * (1.0f / 240.0f);
    if (!stasisFreeze) {
        const auto barInc = static_cast<uint32_t>(barHz * hzToBasePhaseU32 + 0.5f);
        rs.barPhaseU32 += barInc;
    }

    const float rawBar01 = static_cast<float>(rs.barPhaseU32) * (1.0f / 4294967296.0f);
    const float bar01 = warpSwingBarPhase01(rawBar01, swing);
    rs.warpedBar01 = bar01;

    const float step16Scaled = bar01 * 16.0f;
    const auto step16Idx = static_cast<uint32_t>(step16Scaled);
    const uint8_t step16 = static_cast<uint8_t>(step16Idx & 15u);
    const float step16Frac = step16Scaled - static_cast<float>(step16Idx);

    // Stereo ping-pong bounce on odd swung 16th off-beats
    const float panTarget =
        (swing > 1.0e-4f) ? (((step16 & 1u) != 0u ? 0.56f : -0.56f) * swing) : 0.0f;
    rs.swingPanSmoother.setTarget(panTarget);
    out.swingPanOffset = rs.swingPanSmoother.next();

    // Check active gate families
    const bool anyVoiceMode = (rs.voiceMode[0] != 0u) || (rs.voiceMode[1] != 0u) ||
                              (rs.voiceMode[2] != 0u) || (rs.voiceMode[3] != 0u);
    const bool hasChase = (chase > 0.02f);
    const bool hasPump = (pump > 1.0e-4f);
    const bool soloTopPulse =
        !anyVoiceMode && !hasChase && !hasPump &&
        ((tempo > 0.02f) || (pluck > 0.02f) || (shrt > 0.02f) || (swing > 0.02f));

    // Gate duty: 50% standard -> 6% pinpoint click with MONO SHORT (or 88% decay window on PLUCK)
    const float baseDuty = 0.50f - 0.44f * shrt;
    const float effDuty = (shrt > 0.02f) ? baseDuty : (baseDuty + 0.38f * pluck);

    // Optional CHASE step index (16th-note chase below 55% pressure, 32nd-note chase above 55%)
    const float chaseScaled = bar01 * ((chase >= 0.55f) ? 32.0f : 16.0f);
    const auto chaseStepIdx = static_cast<uint32_t>(chaseScaled);
    const uint8_t chaseVoice = static_cast<uint8_t>(chaseStepIdx & 3u);
    const float chaseFrac = chaseScaled - static_cast<float>(chaseStepIdx);

    // Optional PUMP sidechain gain (4-on-the-floor, or 8/bar double-time above 72% pressure)
    float pumpGain = 1.0f;
    bool pumpDucked = false;
    if (hasPump) {
        const float pumpScaled = bar01 * ((pump >= 0.72f) ? 8.0f : 4.0f);
        const float pumpFrac = pumpScaled - static_cast<float>(static_cast<uint32_t>(pumpScaled));
        const float swell = smoothstep01(pumpFrac * 1.22f);
        pumpGain = 1.0f - (0.95f * pump) * (1.0f - swell);
        pumpDucked = ((1.0f - swell) * pump > 0.52f);
    }

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        auto mode = static_cast<VoiceRhythmMode>(rs.voiceMode[v]);
        const float rawStutDepth = clampf(stutterDepths[v], 0.0f, 1.0f);
        if (mode == VoiceRhythmMode::Off && !hasChase && !hasPump && !soloTopPulse &&
            rawStutDepth > 1.0e-4f) {
            mode = VoiceRhythmMode::Stutter;
        }
        bool hasGate = false;
        bool stepActive = true;
        float gateDepth = 1.0f;
        float subFrac = 0.0f;

        if (mode == VoiceRhythmMode::Stutter) {
            hasGate = true;
            gateDepth = (rs.voiceMode[v] != 0u) ? 1.0f : rawStutDepth;
            if (!stasisFreeze) {
                const auto inc = static_cast<uint32_t>(stutterRatesHz[v] * hzToBasePhaseU32 + 0.5f);
                stutterPhases[v] += inc;
            }
            subFrac = static_cast<float>(stutterPhases[v]) * (1.0f / 4294967296.0f);
        } else {
            stutterPhases[v] = 0x80000000u;
            if (mode == VoiceRhythmMode::Divide) {
                hasGate = true;
                const float cyc = kClockDivCyclesPerBar[rs.voiceParam[v] % kNumClockDivisions];
                const float divScaled = bar01 * cyc;
                subFrac = divScaled - static_cast<float>(static_cast<uint32_t>(divScaled));
            } else if (mode == VoiceRhythmMode::Euclid) {
                hasGate = true;
                const uint16_t mask16 = voiceEuclidMask16(v, rs.voiceParam[v]);
                stepActive = (((mask16 >> step16) & 1u) != 0u);
                subFrac = step16Frac;
            }
        }

        if (hasChase) {
            if (hasGate) {
                stepActive = stepActive && (chaseVoice == v);
            } else {
                hasGate = true;
                stepActive = (chaseVoice == v);
                subFrac = chaseFrac;
            }
        } else if (!hasGate && soloTopPulse) {
            hasGate = true;
            stepActive = true;
            subFrac = step16Frac;
        }

        float voiceGate = 1.0f;
        if (hasGate) {
            const bool gateOpen = stepActive && (subFrac < effDuty);
            if (!gateOpen) {
                out.gateMask &= static_cast<uint8_t>(~(1u << v));
                voiceGate = 1.0f - gateDepth;
            } else if (pluck > 1.0e-4f) {
                const float env = fastExpNeg(subFrac * (3.0f + 6.5f * pluck));
                out.pluckEnv01[v] = env;
                voiceGate = (1.0f - 0.88f * pluck) + 0.88f * pluck * env;
            }
        } else if (pumpDucked) {
            out.gateMask &= static_cast<uint8_t>(~(1u << v));
        }

        out.vcaGateTarget[v] = clampf(voiceGate * pumpGain, 0.0f, 1.0f);
    }

    return out;
}

} // namespace ruido
