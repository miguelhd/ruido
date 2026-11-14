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
            {"DRIVE", "SUB/LPF", "SPREAD", "SOLO"},
            {"FILTER", "OCTAVE", "SCENE", "MUTE"},
        },
        // 2/3: CIRCUIT LAB (CMOS Patch Bay)
        {
            "CIRCUIT LAB",
            "CMOS PATCH BAY",
            {"STARVE", "RING/MOD", "SYNC", "SOLO"},
            {"CROSS", "FOLD", "CRUSH", "MUTE"},
        },
        // 3/3: RHYTHM LAB (Stutter, Clock Dividers & Euclidean Grooves)
        {
            "RHYTHM LAB",
            "CLOCK, DIVIDE & EUCLID",
            {"PUMP", "CLOCK", "SWING", "SOLO"},
            {"STUTTER", "DIVIDE", "EUCLID", "MUTE"},
        },
    }};

constexpr const KeyboardModelSpec& keyboardModelSpec(KeyboardModel model) noexcept {
    const auto idx = static_cast<size_t>(model);
    return (idx < kKeyboardModelSpecs.size()) ? kKeyboardModelSpecs[idx] : kKeyboardModelSpecs[0];
}

// Single source of truth for the 27-key varibright zone gradient, Console SOLO/MUTE state,
// and SCENE 1-4 status across all keyboard models (shared 1-to-1 by Core 0 PWM LEDs and SEL map).
inline void computeKeyboardModelRestLeds(KeyboardModel model, const EngineControlState& state,
                                         const HudState& hud, uint8_t stutterGateMask,
                                         float subBreathWave01, float savePulse01,
                                         std::array<float, kNumBlackKeys>& outBlack,
                                         std::array<float, kNumWhiteKeys>& outWhite) noexcept {
    (void)stutterGateMask;
    (void)subBreathWave01;
    // High-contrast bright -> dark left-to-right group gradient across both physical rows
    // (terminating at 0% dark on right-hand SOLO 1-4 and MUTE 1-4 when off, 100% when active):
    //   Top 11 (L->R):    86% (OP1) -> 50% (OP2-4) -> 22% (OP5-6) -> 0% (SOLO) | 12% (11th)
    //   Bottom 16 (L->R): 82% (1-4) -> 48% (5-8)   -> 16% (9-12)  -> 0% (MUTE 13-16)
    outBlack[static_cast<size_t>(BlackKeyId::Op1)] = 0.86f;
    outBlack[static_cast<size_t>(BlackKeyId::Op2)] = 0.50f;
    outBlack[static_cast<size_t>(BlackKeyId::Op3)] = 0.50f;
    outBlack[static_cast<size_t>(BlackKeyId::Op4)] = 0.50f;
    outBlack[static_cast<size_t>(BlackKeyId::Op5)] = 0.22f;
    outBlack[static_cast<size_t>(BlackKeyId::Op6)] = 0.22f;
    outBlack[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)] = 0.12f;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        outWhite[kBankAWhiteKeyBase + v] = 0.82f;
        outWhite[kBankBWhiteKeyBase + v] = 0.48f;
        const bool occ = (hud.snapshotOccupiedMask & (1u << v)) != 0u;
        outWhite[kSceneWhiteKeyBase + v] =
            (model == KeyboardModel::LiveMixer) ? (occ ? 0.16f : 0.0f) : 0.16f;
    }

    if (hud.saveArmed) {
        // Store mode pulses all 4 Scene keys (9-12); Delete mode pulses only occupied scenes
        const float pulse = 0.25f + 0.70f * clampf(savePulse01, 0.0f, 1.0f);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
            if (hud.saveDeleteMode) {
                outWhite[kSceneWhiteKeyBase + s] = occ ? pulse : 0.0f;
            } else {
                outWhite[kSceneWhiteKeyBase + s] =
                    (occ && s == (hud.snapshotSlot & 3u)) ? 1.0f : pulse;
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
                outWhite[kSceneWhiteKeyBase + s] = occ ? (0.24f + 0.76f * w) : 0.0f;
            }
        } else if (hud.config.snapMode == KeySwitchMode::Latching && !hud.snapshotDirty &&
                   ((hud.snapshotOccupiedMask & (1u << (hud.snapshotSlot & 3u))) != 0u)) {
            outWhite[kSceneWhiteKeyBase + (hud.snapshotSlot & 3u)] = 1.0f;
        }
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        evaluateVoiceConsoleLeds(state.keyMod, v, outBlack[kSoloBlackKeyBase + v],
                                 outWhite[kMuteWhiteKeyBase + v]);
    }
}

} // namespace ruido
