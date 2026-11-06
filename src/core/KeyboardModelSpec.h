#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/Parameters.h"
#include "core/VoiceCircuit.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Reusable declarative metadata & varibright LED zone map shared across all swappable
// 27-key interaction models (consumed by both Core 0 physical LEDs and the SEL guidance HUD).
struct KeyboardModelSpec {
    const char* title;
    const char* subtitle;
    std::array<const char*, 4> topZones;
    std::array<const char*, 4> bottomZones;
};

inline constexpr std::array<KeyboardModelSpec, static_cast<size_t>(KeyboardModel::Count)>
    kKeyboardModelSpecs = {{
        // 1/3: LIVE MIXER (4-Oscillator Mix, FX & 4-Slot Scenes)
        {
            "LIVE MIXER",
            "MIX, FX & SCENES",
            {"SOLO", "DRIVE", "SUB/LPF", "SPREAD"},
            {"MUTE", "FILTER", "OCTAVE", "SCENE"},
        },
        // 2/3: CIRCUIT LAB (CMOS Patch Bay)
        {
            "CIRCUIT LAB",
            "CMOS PATCH BAY",
            {"SOLO", "STARVE", "RING/MOD", "SYNC"},
            {"MUTE", "CROSS", "FOLD", "CRUSH"},
        },
        // 3/3: RHYTHM LAB (Stutter, Clock Dividers & Euclidean Grooves)
        {
            "RHYTHM LAB",
            "CLOCK, DIVIDE & EUCLID",
            {"SOLO", "PUMP", "CLOCK", "SWING"},
            {"MUTE", "STUTTER", "DIVIDE", "EUCLID"},
        },
    }};

constexpr const KeyboardModelSpec& keyboardModelSpec(KeyboardModel model) noexcept {
    const auto idx = static_cast<size_t>(model);
    return (idx < kKeyboardModelSpecs.size()) ? kKeyboardModelSpecs[idx] : kKeyboardModelSpecs[0];
}

// Single source of truth for the 27-key varibright zone gradient, Console SOLO/MUTE state,
// and SNAP 1-4 status across all keyboard models (shared 1-to-1 by Core 0 PWM LEDs and SEL map).
inline void computeKeyboardModelRestLeds(KeyboardModel model, const EngineControlState& state,
                                         const HudState& hud, uint8_t stutterGateMask,
                                         float subBreathWave01, float savePulse01,
                                         std::array<float, kNumBlackKeys>& outBlack,
                                         std::array<float, kNumWhiteKeys>& outWhite) noexcept {
    // Shared left-to-right group brightness gradient across both physical rows:
    //   Top 11 (5 groups, L->R):    64% -> 46% -> 30% -> 18% -> 8%
    //   Bottom 16 (4 groups, L->R): 64% -> 42% -> 22% -> 8%
    outBlack[static_cast<size_t>(BlackKeyId::Op5)] = 0.46f;
    outBlack[static_cast<size_t>(BlackKeyId::Op6)] = 0.30f;
    outBlack[static_cast<size_t>(BlackKeyId::Pit)] = 0.30f;
    outBlack[static_cast<size_t>(BlackKeyId::Glo)] = 0.30f;
    outBlack[static_cast<size_t>(BlackKeyId::Mono)] = 0.18f;
    outBlack[static_cast<size_t>(BlackKeyId::Poly)] = 0.18f;
    outBlack[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)] = 0.08f;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        outWhite[4u + v] = 0.42f;
        outWhite[8u + v] = 0.22f;
        const bool occ = (hud.snapshotOccupiedMask & (1u << v)) != 0u;
        outWhite[12u + v] = (model == KeyboardModel::LiveMixer && !occ) ? 0.0f : 0.08f;
    }

    if (hud.saveArmed) {
        // Store mode pulses all 4 Scene keys; Delete mode pulses only occupied scenes (empty =
        // dark)
        const float pulse = 0.25f + 0.70f * clampf(savePulse01, 0.0f, 1.0f);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
            if (hud.saveDeleteMode) {
                outWhite[12u + s] = occ ? pulse : 0.0f;
            } else {
                outWhite[12u + s] = (occ && s == (hud.snapshotSlot & 3u)) ? 1.0f : pulse;
            }
        }
    } else if (model == KeyboardModel::LiveMixer) {
        const bool snapBanner =
            hud.snapshotGliding || (hud.mode == HudMode::SnapshotSlot) ||
            (hud.mode == HudMode::KeyboardPlay && hud.activePunch == KeyPunchKind::SnapshotRecall);
        if (snapBanner) {
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
                const float w = clampf(hud.snapshotWeights[s], 0.0f, 1.0f);
                outWhite[12u + s] = occ ? (0.08f + 0.92f * w) : 0.0f;
            }
        } else if (hud.config.snapMode == KeySwitchMode::Latching && !hud.snapshotDirty &&
                   ((hud.snapshotOccupiedMask & (1u << (hud.snapshotSlot & 3u))) != 0u)) {
            // Subtle active-slot landmark (< 22% of OCTAVE group so L->R gradient stays monotonic)
            outWhite[12u + (hud.snapshotSlot & 3u)] = 0.16f;
        }
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceCircuitState vc = evaluateVoiceCircuit(state, v, stutterGateMask);
        evaluateVoiceConsoleLeds(vc, subBreathWave01, outBlack[v], outWhite[v]);
    }
}

} // namespace ruido
