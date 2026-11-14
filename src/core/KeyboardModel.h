#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
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

// Per-model latched key switch state (managed by KeyboardEngine, cleared on model swap or reset).
struct KeyboardLatchState {
    uint8_t muteMask = 0u; // 4-bit latched oscillator mute mask (bit v = OSC v+1)
    uint8_t soloMask = 0u; // 4-bit latched oscillator solo mask (bit v = OSC v+1)
};

// Zero-copy read-only snapshot of the 27-key gates, continuous pressures, and LIFO press order
// passed from KeyboardEngine into whichever KeyboardModelOps plugin is active.
struct KeyboardInputView {
    const std::array<bool, kNumBlackKeys>& blackHeld;
    const std::array<bool, kNumWhiteKeys>& whiteHeld;
    const std::array<float, kNumBlackKeys>& blackPressure;
    const std::array<float, kNumWhiteKeys>& whitePressure;
    const std::array<uint16_t, kNumBlackKeys>& blackPressSeq;
    const std::array<uint16_t, kNumWhiteKeys>& whitePressSeq;
    uint16_t heldBlackMask = 0u;
    uint16_t heldWhiteMask = 0u;
    KeyboardLatchState latch{};
    GlobalConfigState config{};
};

// LIFO most-recently-pressed key punch tracker for HudMode::KeyboardPlay headers.
struct PunchTracker {
    uint16_t latestSeq = 0u;
    KeyPunchKind latestKind = KeyPunchKind::None;
    uint8_t latestVoice = 0u;
    float latestValue = 0.0f;

    void note(uint16_t seq, KeyPunchKind kind, uint8_t vIdx, float val) noexcept {
        if (seq >= latestSeq) {
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

// Shared Momentary/Latching Console SOLO 1-4 (PIT..POLY, Black 6-9) & MUTE 1-4 (Keys 13-16,
// White 12-15) switch logic on the right-hand non-pressure keys.
inline void onConsoleSoloMuteKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                                      KeyboardLatchState& latch) noexcept {
    if (isBlack) {
        if (keyIdx < kSoloBlackKeyBase || keyIdx >= kSoloBlackKeyBase + kNumOscillators) {
            return;
        }
        const uint8_t bit = static_cast<uint8_t>(1u << (keyIdx - kSoloBlackKeyBase));
        if (config.soloMode == KeySwitchMode::Latching) {
            latch.soloMask ^= bit;
            if ((latch.soloMask & bit) != 0u) {
                latch.muteMask &= static_cast<uint8_t>(~bit);
            }
        }
    } else {
        if (keyIdx < kMuteWhiteKeyBase || keyIdx >= kMuteWhiteKeyBase + kNumOscillators) {
            return;
        }
        const uint8_t bit = static_cast<uint8_t>(1u << (keyIdx - kMuteWhiteKeyBase));
        if (config.muteMode == KeySwitchMode::Latching) {
            latch.muteMask ^= bit;
            if ((latch.muteMask & bit) != 0u) {
                latch.soloMask &= static_cast<uint8_t>(~bit);
            }
        }
    }
}

inline void evaluateConsoleSoloMute(const KeyboardInputView& in, KeyboardModState& mod,
                                    PunchTracker& punch) noexcept {
    const bool soloLatch = (in.config.soloMode == KeySwitchMode::Latching);
    const bool muteLatch = (in.config.muteMode == KeySwitchMode::Latching);
    uint8_t soloMask = soloLatch ? in.latch.soloMask : 0u;
    uint8_t muteMask = muteLatch ? in.latch.muteMask : 0u;

    uint16_t latestSoloSeq = 0u;
    uint8_t latestSoloVoice = 0u;
    bool latestSoloOn = true;

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        const size_t sIdx = kSoloBlackKeyBase + v;
        const size_t mIdx = kMuteWhiteKeyBase + v;
        const uint8_t bit = static_cast<uint8_t>(1u << v);
        if (!soloLatch && in.blackHeld[sIdx]) {
            soloMask |= bit;
        }
        if (!muteLatch && in.whiteHeld[mIdx]) {
            muteMask |= bit;
        }
        if (!soloLatch && muteLatch && in.blackHeld[sIdx]) {
            muteMask &= static_cast<uint8_t>(~bit);
        }
        if (!soloLatch && !muteLatch && in.blackHeld[sIdx] && in.whiteHeld[mIdx]) {
            if (in.blackPressSeq[sIdx] > in.whitePressSeq[mIdx]) {
                muteMask &= static_cast<uint8_t>(~bit);
            } else {
                soloMask &= static_cast<uint8_t>(~bit);
            }
        }
        if ((muteMask & bit) != 0u) {
            soloMask &= static_cast<uint8_t>(~bit);
        }
        if (in.blackHeld[sIdx] && in.blackPressSeq[sIdx] >= latestSoloSeq) {
            latestSoloSeq = in.blackPressSeq[sIdx];
            latestSoloVoice = v;
            latestSoloOn = ((soloMask & bit) != 0u);
        }
    }

    mod.soloMask = soloMask;
    mod.muteMask = muteMask;
    if (soloMask != 0u) {
        for (uint8_t v = 0u; v < kNumOscillators; ++v) {
            if (((soloMask >> v) & 1u) == 0u) {
                mod.voiceGainMul[v] = 0.0f;
            }
        }
    }
    if (latestSoloSeq > 0u) {
        punch.note(latestSoloSeq, KeyPunchKind::SpotlightSolo, latestSoloVoice,
                   latestSoloOn ? 1.0f : 0.0f);
    }

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        const size_t mIdx = kMuteWhiteKeyBase + v;
        const uint8_t bit = static_cast<uint8_t>(1u << v);
        const bool isChMuted = ((muteMask & bit) != 0u);
        if (isChMuted) {
            mod.voiceGainMul[v] = 0.0f;
        }
        if (in.whiteHeld[mIdx]) {
            punch.note(in.whitePressSeq[mIdx], KeyPunchKind::VoiceCut, v, isChMuted ? 1.0f : 0.0f);
        }
    }
}

inline void evaluateUniversalFreeze(const KeyboardInputView& in, KeyboardModState& mod,
                                    HudState& hud, PunchTracker& punch) noexcept {
    constexpr size_t kFreezeIdx = static_cast<size_t>(BlackKeyId::ScopeFreezeKey);
    if (in.blackHeld[kFreezeIdx]) {
        mod.stasisFreeze = true;
        hud.scopeFrozen = true;
        punch.note(in.blackPressSeq[kFreezeIdx], KeyPunchKind::StasisFreeze, 0u, 1.0f);
    }
}

inline void applyFreezeStillFrameLeds(const KeyboardInputView& in, LedState& outLeds) noexcept {
    constexpr size_t kFreezeIdx = static_cast<size_t>(BlackKeyId::ScopeFreezeKey);
    if (!in.blackHeld[kFreezeIdx]) {
        return;
    }
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        outLeds.blackKeyLeds[i] = 0.52f;
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        outLeds.whiteKeyLeds[i] = 0.52f;
    }
    outLeds.blackKeyLeds[kFreezeIdx] = 1.0f;
    setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.48f);
    setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.48f);
    for (size_t c = 0u; c < 6u; ++c) {
        setPanelBtnLed(outLeds, kPanelGridTop[c], 0.48f);
        if (c < 4u) {
            setPanelBtnLed(outLeds, kPanelGridBot[c], 0.48f);
        }
    }
}

inline void applyAuthoritativeConsoleLeds(const EngineControlState& state,
                                          LedState& outLeds) noexcept {
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        evaluateVoiceConsoleLeds(state.keyMod, v, outLeds.blackKeyLeds[kSoloBlackKeyBase + v],
                                 outLeds.whiteKeyLeds[kMuteWhiteKeyBase + v]);
    }
}

// Pluggable behavior contract for each 27-key interaction model (LiveMixer, CircuitLab, RhythmLab)
struct KeyboardModelOps {
    void (*onKeyPress)(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                       KeyboardLatchState& latch) noexcept;
    void (*evaluate)(const KeyboardInputView& input, EngineControlState& state,
                     HudState& hud) noexcept;
    void (*choreographActiveLeds)(const KeyboardInputView& input, float animPhase,
                                  float subBreathWave01, const EngineControlState& state,
                                  const HudState& hud, uint8_t stutterGateMask, float rhythmPhase01,
                                  LedState& outLeds) noexcept;
};

void onLiveMixerKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                         KeyboardLatchState& latch) noexcept;
void evaluateLiveMixerModel(const KeyboardInputView& input, EngineControlState& state,
                            HudState& hud) noexcept;
void choreographLiveMixerLeds(const KeyboardInputView& input, float animPhase,
                              float subBreathWave01, const EngineControlState& state,
                              const HudState& hud, uint8_t stutterGateMask, float rhythmPhase01,
                              LedState& outLeds) noexcept;

void onCircuitLabKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                          KeyboardLatchState& latch) noexcept;
void evaluateCircuitLabModel(const KeyboardInputView& input, EngineControlState& state,
                             HudState& hud) noexcept;
void choreographCircuitLabLeds(const KeyboardInputView& input, float animPhase,
                               float subBreathWave01, const EngineControlState& state,
                               const HudState& hud, uint8_t stutterGateMask, float rhythmPhase01,
                               LedState& outLeds) noexcept;

void onRhythmLabKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                         KeyboardLatchState& latch) noexcept;
void evaluateRhythmLabModel(const KeyboardInputView& input, EngineControlState& state,
                            HudState& hud) noexcept;
void choreographRhythmLabLeds(const KeyboardInputView& input, float animPhase,
                              float subBreathWave01, const EngineControlState& state,
                              const HudState& hud, uint8_t stutterGateMask, float rhythmPhase01,
                              LedState& outLeds) noexcept;

const KeyboardModelOps& keyboardModelOps(KeyboardModel model) noexcept;

} // namespace ruido
