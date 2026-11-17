#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"
#include "core/RhythmMath.h"
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
    std::array<const char*, 3> topZones;    // Left 8 pressure keys in physical 3 · 2 · 3 clusters
    std::array<const char*, 4> bottomZones; // 12 pressure keys (3x4) + 4 permanent MUTE switches
};

inline constexpr float kLedAssignedRest = 0.15f; // Exact 2.0 us `fm1_led_bg` assigned-ready glow
inline constexpr float kLedZone1Rest = 0.65f;    // Group 1 (`OP1-3` & `Keys 1-4`): ~29.3% duty
inline constexpr float kLedZone2Rest = 0.40f;    // Group 2 (`OP4-5` & `Keys 5-8`): ~7.9% duty
inline constexpr float kLedZone3Rest = 0.15f;    // Group 3 (`OP6-GLO` & `Keys 9-12`): ~2.1% duty

// Assigned `SCENE` (`Keys 9-12`) 3-tier LED ladder (`0%` reserved strictly for unassigned keys;
// `80%` active ceiling matches `MUTE`/`SOLO` so `SHIFT` at `100%` remains the brightest beacon):
inline constexpr float kLedSceneEmpty = kLedAssignedRest; // 15% assigned rest (empty slot)
inline constexpr float kLedSceneStored = kLedZone2Rest;   // 40% occupied stored slot
inline constexpr float kLedSceneActive = 0.80f;           // 80% active scene ceiling

inline constexpr bool isSceneLiveMorphActive(const HudState& hud) noexcept {
    if (hud.saveArmed || hud.snapshotSavedToast || hud.snapshotDeletedToast) {
        return false;
    }
    return hud.snapshotGliding || hud.mode == HudMode::SnapshotSlot ||
           (hud.shiftHeld && hud.shiftFocus == ShiftFocus::Scene) ||
           ((hud.mode == HudMode::KeyboardPlay || hud.mode == HudMode::ShiftMap) &&
            hud.activePunch == KeyPunchKind::SnapshotRecall && (hud.heldWhiteMask & 0x0F00u) != 0u);
}

inline constexpr float computeSceneKeyBrightness(size_t slot, const HudState& hud,
                                                 bool morphActive) noexcept {
    const uint8_t s = static_cast<uint8_t>(slot & 3u);
    if ((hud.snapshotOccupiedMask & (1u << s)) == 0u) {
        return kLedSceneEmpty;
    }
    if (morphActive) {
        const float w = clampf(hud.snapshotWeights[s], 0.0f, 1.0f);
        return kLedSceneStored + (kLedSceneActive - kLedSceneStored) * w;
    }
    return ((hud.snapshotSlot & 3u) == s) ? kLedSceneActive : kLedSceneStored;
}

inline constexpr std::array<KeyboardModelSpec, static_cast<size_t>(KeyboardModel::Count)>
    kKeyboardModelSpecs = {{
        // 1/3: LIVE MIXER (4-Oscillator Mix, Filter, Harmony & Duty)
        {
            "LIVE MIXER",
            "MIX, FILTER & HARMONY",
            {"TONE", "FILTER", "WIDTH"},
            {"FILTER", "OCTAVE", "DUTY", "MUTE"},
        },
        // 2/3: CIRCUIT LAB (CMOS Patch Bay)
        {
            "CIRCUIT LAB",
            "CMOS PATCH BAY",
            {"MOD", "LOCK", "BENDS"},
            {"CROSS", "FOLD", "CRUSH", "MUTE"},
        },
        // 3/3: RHYTHM LAB (Stutter, Clock Dividers & Euclidean Grooves)
        {
            "RHYTHM LAB",
            "CLOCK, DIVIDE & EUCLID",
            {"PULSE", "GATE", "FX"},
            {"STUTTER", "DIVIDE", "EUCLID", "MUTE"},
        },
    }};

constexpr const KeyboardModelSpec& keyboardModelSpec(KeyboardModel model) noexcept {
    const auto idx = static_cast<size_t>(model);
    return (idx < kKeyboardModelSpecs.size()) ? kKeyboardModelSpecs[idx] : kKeyboardModelSpecs[0];
}

constexpr float saveArmedPulseHz(bool deleteMode) noexcept {
    return deleteMode ? 5.00f : 2.50f;
}

inline float computeLatchKeyLed(const HudState& hud) noexcept {
    if ((hud.heldBlackMask & (1u << kHoldBlackKeyIdx)) != 0u) {
        return 1.0f;
    }
    if (!hud.keyHoldLatched) {
        return 0.15f;
    }
    const float p = static_cast<float>(hud.savePulseQ8) * (1.0f / 255.0f);
    return 0.08f + 0.92f * (p * p);
}

// Single source of truth for the 20+7 keybed LED grammar across physical PWM LEDs and SEL/SHIFT UI:
//  - Functional States: `0%` unassigned/silent, `15%` assigned rest (`EMPTY` scene / `LATCH`),
//    `40%` `STORED` scene, `50%` sub-audio `< 20 Hz`, `80%` `ACTIVE` scene & audible `MUTE`/`SOLO`,
//    `100%` `SHIFT` beacon / squeezed pad, `2.5 Hz` blink when `LATCH` is active.
//  - Pure-Blinking `SAVE`/`DELETE` Rule: When `saveArmed`, ONLY actionable `SCENE` keys blink and
//    all other 23 keyboard keys rest at `0%`.
inline void computeKeyboardModelRestLeds(KeyboardModel model, const EngineControlState& state,
                                         const HudState& hud,
                                         std::array<float, kNumBlackKeys>& outBlack,
                                         std::array<float, kNumWhiteKeys>& outWhite) noexcept {
    outBlack.fill(0.0f);
    outWhite.fill(0.0f);

    if (hud.saveArmed) {
        const float savePulse01 = static_cast<float>(hud.savePulseQ8) * (1.0f / 255.0f);
        const float pulse = 0.25f + 0.70f * clampf(savePulse01, 0.0f, 1.0f);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const bool occ = (hud.snapshotOccupiedMask & (1u << s)) != 0u;
            outWhite[kSceneWhiteKeyBase + s] = (!hud.saveDeleteMode || occ) ? pulse : 0.0f;
        }
        return;
    }

    if (hud.shiftHeld) {
        // Universal SHIFT layer: light SHIFT (100%), OP1..OP3 (direct Keyboard 1-3 select w/ 0.40
        // Hz slow blink on active keyboard), SCENE 9-12 (15%/40%/80%), and SOLO 13-16 (<= 80%)
        outBlack[kShiftBlackKeyIdx] = 1.0f;
        const float blink01 = static_cast<float>(hud.shiftBlinkQ8) * (1.0f / 255.0f);
        const auto activeModelIdx = static_cast<size_t>(model);
        for (size_t m = 0u; m < static_cast<size_t>(KeyboardModel::Count); ++m) {
            if (((hud.heldBlackMask >> m) & 1u) != 0u) {
                outBlack[m] = 1.0f;
            } else if (m == activeModelIdx) {
                const float shaped = blink01 * blink01;
                outBlack[m] = 0.08f + 0.92f * shaped;
            } else {
                outBlack[m] = kLedAssignedRest;
            }
        }

        const bool snapMorph = isSceneLiveMorphActive(hud);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            outWhite[kSceneWhiteKeyBase + s] = computeSceneKeyBrightness(s, hud, snapMorph);
        }
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            outWhite[kMuteWhiteKeyBase + v] = evaluateVoiceSoundLedSteady(state, v);
        }
        return;
    }

    // Calibrated 3-group left-to-right gradient (`65% -> 40% -> 15%`) across all 20 pressure pads:
    outBlack[static_cast<size_t>(BlackKeyId::Op1)] = kLedZone1Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Op2)] = kLedZone1Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Op3)] = kLedZone1Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Op4)] = kLedZone2Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Op5)] = kLedZone2Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Op6)] = kLedZone3Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Pit)] = kLedZone3Rest;
    outBlack[static_cast<size_t>(BlackKeyId::Glo)] = kLedZone3Rest;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        outWhite[kBankAWhiteKeyBase + v] = kLedZone1Rest;
        outWhite[kBankBWhiteKeyBase + v] = kLedZone2Rest;
        outWhite[kSceneWhiteKeyBase + v] = kLedZone3Rest;
    }

    // Highlight any expressive pads currently held or latched hands-free by LATCH (`MONO`) at 100%
    const uint16_t litBlack = static_cast<uint16_t>(hud.latchedBlackMask | hud.heldBlackMask);
    const uint16_t litWhite = static_cast<uint16_t>(hud.latchedWhiteMask | hud.heldWhiteMask);
    for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
        if (((litBlack >> i) & 1u) != 0u) {
            outBlack[i] = 1.0f;
        }
    }
    for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
        if (((litWhite >> i) & 1u) != 0u) {
            outWhite[i] = 1.0f;
        }
    }

    // Right 7 Permanent Utility Switches (`LATCH`, `TAP`, `SHIFT` + `MUTE 1-4`):
    outBlack[kHoldBlackKeyIdx] = computeLatchKeyLed(hud);
    outBlack[kTapBlackKeyIdx] = 0.0f; // Active only in 3/3 RHYTHM LAB (pulsed in KeyboardEngine)
    outBlack[kShiftBlackKeyIdx] = 1.0f;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        outWhite[kMuteWhiteKeyBase + v] = evaluateVoiceSoundLedSteady(state, v);
    }
}

inline float computeTapTempoBeatGlow(const EngineControlState& state, float rhythmPhase) noexcept {
    if (!state.global.masterRun) {
        return kLedAssignedRest;
    }
    const float effBpm = state.global.clockBpm * computeTempoMultiplier(state.keyMod.rhythmTempo);
    const float beatScaled = wrapLfoPhase01(rhythmPhase) * 4.0f;
    const float beatFrac = beatScaled - static_cast<float>(static_cast<uint32_t>(beatScaled));
    const float beatFlash = fastExpNeg(beatFrac * 6.0f);
    const float beatSmooth = smoothstep01(clampf((effBpm - 900.0f) * (1.0f / 600.0f), 0.0f, 1.0f));
    const float pulse = beatFlash * (1.0f - beatSmooth) + 0.35f * beatSmooth;
    return kLedAssignedRest + (1.0f - kLedAssignedRest) * pulse;
}

} // namespace ruido
