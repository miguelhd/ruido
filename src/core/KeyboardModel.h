#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/KeyboardModelSpec.h"
#include "core/Parameters.h"
#include "core/VoiceCircuit.h"

#include <array>
#include <cstdint>

namespace ruido {

// Shared right-hand 2x6 panel button grid tables & LED helpers for keyboard model choreographers
inline constexpr std::array<ButtonId, 6> kPanelGridTop = {
    ButtonId::Fx, ButtonId::Sel, ButtonId::Env, ButtonId::Lfo, ButtonId::Edit, ButtonId::GloRight,
};
inline constexpr std::array<ButtonId, 6> kPanelGridBot = {
    ButtonId::Home, ButtonId::Save, ButtonId::Arp, ButtonId::Seq, ButtonId::PlayStop, ButtonId::Rec,
};

inline void setPanelBtnLed(LedState& leds, ButtonId id, float val) noexcept {
    leds.buttonLeds[static_cast<size_t>(id)] = clampf(val, 0.0f, 1.0f);
}

inline void maxPanelBtnLed(LedState& leds, ButtonId id, float val) noexcept {
    const float v = clampf(val, 0.0f, 1.0f);
    float& dst = leds.buttonLeds[static_cast<size_t>(id)];
    if (v > dst) {
        dst = v;
    }
}

inline void scalePanelGrid(LedState& leds, float factor) noexcept {
    leds.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] *= factor;
    leds.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] *= factor;
    for (size_t c = 0u; c < 6u; ++c) {
        leds.buttonLeds[static_cast<size_t>(kPanelGridTop[c])] *= factor;
        if (kPanelGridBot[c] != ButtonId::PlayStop && kPanelGridBot[c] != ButtonId::Rec) {
            leds.buttonLeds[static_cast<size_t>(kPanelGridBot[c])] *= factor;
        }
    }
}

// Persistent console switch state across the right-hand 7-switch bay.
struct KeyboardLatchState {
    uint8_t muteMask = 0u;        // 4-bit latched oscillator mute mask (Keys 13-16)
    uint8_t preSoloMuteMask = 0u; // 4-bit muteMask saved before latching SHIFT + SOLO
    bool preSoloValid = false;    // True while in a latched SHIFT + SOLO isolate state
};

// Effective 27-key view passed from KeyboardEngine into the active KeyboardModelOps evaluator
// (combines live physical unshifted key presses with any hands-free HOLD-latched expressive pads).
struct KeyboardInputView {
    const std::array<bool, kNumBlackKeys>& blackHeld;
    const std::array<bool, kNumWhiteKeys>& whiteHeld;
    const std::array<float, kNumBlackKeys>& blackPressure;
    const std::array<float, kNumWhiteKeys>& whitePressure;
    const std::array<uint16_t, kNumBlackKeys>& blackPressSeq;
    const std::array<uint16_t, kNumWhiteKeys>& whitePressSeq;
    uint16_t heldBlackMask = 0u;
    uint16_t heldWhiteMask = 0u;
    uint8_t muteHeldMask = 0u; // Bits 0..3 = Keys 13-16 held unshifted (MUTE 1-4)
    uint8_t soloHeldMask = 0u; // Bits 0..3 = Keys 13-16 held under SHIFT (SOLO 1-4)
    KeyboardLatchState latch{};
    GlobalConfigState config{};
};

// LIFO most-recently-pressed key punch tracker for HudMode::KeyboardPlay & SEL/SHIFT inspectors.
struct PunchTracker {
    uint16_t latestSeq = 0u;
    KeyPunchKind latestKind = KeyPunchKind::None;
    uint8_t latestVoice = 0u;
    float latestValue = 0.0f;

    void note(uint16_t seq, KeyPunchKind kind, uint8_t vIdx, float val) noexcept {
        if (seq > 0u && seq >= latestSeq) {
            latestSeq = seq;
            latestKind = kind;
            latestVoice = vIdx;
            latestValue = val;
        }
    }

    void commitTo(HudState& hud) const noexcept {
        hud.activePunch = latestKind;
        hud.activePunchVoice = latestVoice;
        hud.activePunchValue = latestValue;
    }
};

// Single-state Console MUTE 1-4 (Keys 13-16 unshifted) & Exclusive Isolate SOLO 1-4 (SHIFT + Keys
// 13-16) with pre-Solo mix memory in LATCHING mode.
inline void onConsoleSoloMuteKeyPress(bool shiftHeld, uint8_t whiteKeyIdx,
                                      uint8_t shiftSoloHeldMask, const GlobalConfigState& config,
                                      KeyboardLatchState& latch) noexcept {
    if (whiteKeyIdx < kMuteWhiteKeyBase || whiteKeyIdx >= kMuteWhiteKeyBase + kNumOscillators) {
        return;
    }
    if (config.muteMode != KeySwitchMode::Latching) {
        return;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << (whiteKeyIdx - kMuteWhiteKeyBase));
    if (!shiftHeld) {
        latch.muteMask ^= bit;
        latch.preSoloValid = false;
        return;
    }

    const uint8_t chordBits = static_cast<uint8_t>((shiftSoloHeldMask | bit) & 0x0Fu);
    const uint8_t singleIsoMute = static_cast<uint8_t>((~bit) & 0x0Fu);
    if (chordBits == bit && latch.muteMask == singleIsoMute) {
        // Already isolated on this sole oscillator: restore pre-Solo mix (or unmute all 4)
        latch.muteMask = (latch.preSoloValid && latch.preSoloMuteMask != singleIsoMute)
                             ? latch.preSoloMuteMask
                             : 0u;
        latch.preSoloValid = false;
        return;
    }

    if (!latch.preSoloValid) {
        latch.preSoloMuteMask = latch.muteMask;
        latch.preSoloValid = true;
    }
    latch.muteMask = static_cast<uint8_t>((~chordBits) & 0x0Fu);
}

inline void evaluateConsoleSoloMute(const KeyboardInputView& in, KeyboardModState& mod,
                                    PunchTracker& punch) noexcept {
    const bool muteLatch = (in.config.muteMode == KeySwitchMode::Latching);
    uint8_t muteMask = muteLatch ? (in.latch.muteMask & 0x0Fu) : (in.muteHeldMask & 0x0Fu);
    uint8_t soloMask = muteLatch ? 0u : static_cast<uint8_t>((in.soloHeldMask & 0x0Fu) & ~muteMask);

    mod.soloMask = soloMask;
    mod.muteMask = muteMask;
    if (soloMask != 0u) {
        for (uint8_t v = 0u; v < kNumOscillators; ++v) {
            if (((soloMask >> v) & 1u) == 0u) {
                mod.voiceGainMul[v] = 0.0f;
            }
        }
    }

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        const size_t mIdx = kMuteWhiteKeyBase + v;
        const uint8_t bit = static_cast<uint8_t>(1u << v);
        const bool isChMuted = ((muteMask & bit) != 0u);
        if (isChMuted) {
            mod.voiceGainMul[v] = 0.0f;
        }
        if ((in.muteHeldMask & bit) != 0u) {
            punch.note(in.whitePressSeq[mIdx], KeyPunchKind::VoiceCut, v, isChMuted ? 1.0f : 0.0f);
        }
        if ((in.soloHeldMask & bit) != 0u) {
            const bool isSoloed =
                muteLatch ? (in.latch.preSoloValid && !isChMuted) : ((soloMask & bit) != 0u);
            punch.note(in.whitePressSeq[mIdx], KeyPunchKind::SpotlightSolo, v,
                       isSoloed ? 1.0f : 0.0f);
        }
    }
}

inline void applyAuthoritativeConsoleLeds(const EngineControlState& state, const HudState& hud,
                                          LedState& outLeds) noexcept {
    outLeds.blackKeyLeds[kShiftBlackKeyIdx] = 1.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        outLeds.whiteKeyLeds[kMuteWhiteKeyBase + v] = evaluateVoiceSoundLedSteady(state, v);
    }
    if (hud.shiftHeld && !hud.saveArmed) {
        return;
    }
    outLeds.blackKeyLeds[kHoldBlackKeyIdx] = computeLatchKeyLed(hud);
    if (hud.keyboardModel != KeyboardModel::RhythmLab) {
        outLeds.blackKeyLeds[kTapBlackKeyIdx] = 0.0f;
    } else if ((hud.heldBlackMask & (1u << kTapBlackKeyIdx)) != 0u) {
        outLeds.blackKeyLeds[kTapBlackKeyIdx] = 1.0f;
    }
}

// Pluggable behavior contract for each 27-key interaction model (LiveMixer, CircuitLab, RhythmLab)
struct KeyboardModelOps {
    void (*evaluate)(const KeyboardInputView& input, EngineControlState& state,
                     HudState& hud) noexcept;
    void (*choreographActiveLeds)(const KeyboardInputView& input, float animPhase,
                                  const EngineControlState& state, const HudState& hud,
                                  uint8_t stutterGateMask, float rhythmPhase01,
                                  LedState& outLeds) noexcept;
};

void evaluateLiveMixerModel(const KeyboardInputView& input, EngineControlState& state,
                            HudState& hud) noexcept;
void choreographLiveMixerLeds(const KeyboardInputView& input, float animPhase,
                              const EngineControlState& state, const HudState& hud,
                              uint8_t stutterGateMask, float rhythmPhase01,
                              LedState& outLeds) noexcept;

void evaluateCircuitLabModel(const KeyboardInputView& input, EngineControlState& state,
                             HudState& hud) noexcept;
void choreographCircuitLabLeds(const KeyboardInputView& input, float animPhase,
                               const EngineControlState& state, const HudState& hud,
                               uint8_t stutterGateMask, float rhythmPhase01,
                               LedState& outLeds) noexcept;

void evaluateRhythmLabModel(const KeyboardInputView& input, EngineControlState& state,
                            HudState& hud) noexcept;
void choreographRhythmLabLeds(const KeyboardInputView& input, float animPhase,
                              const EngineControlState& state, const HudState& hud,
                              uint8_t stutterGateMask, float rhythmPhase01,
                              LedState& outLeds) noexcept;

const KeyboardModelOps& keyboardModelOps(KeyboardModel model) noexcept;

} // namespace ruido
