#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/Parameters.h"

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
    const std::array<uint32_t, kNumBlackKeys>& blackPressSeq;
    const std::array<uint32_t, kNumWhiteKeys>& whitePressSeq;
    uint16_t heldBlackMask = 0u;
    uint16_t heldWhiteMask = 0u;
    KeyboardLatchState latch{};
    GlobalConfigState config{};
};

// Pluggable behavior contract for each 27-key interaction model (LiveMixer, Harmonium, CircuitLab)
struct KeyboardModelOps {
    void (*onKeyPress)(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                       KeyboardLatchState& latch) noexcept;
    void (*evaluate)(const KeyboardInputView& input, EngineControlState& state,
                     HudState& hud) noexcept;
    void (*choreographActiveLeds)(const KeyboardInputView& input, float animPhase,
                                  float subBreathWave01, const EngineControlState& state,
                                  const HudState& hud, uint8_t stutterGateMask,
                                  LedState& outLeds) noexcept;
};

void onLiveMixerKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                         KeyboardLatchState& latch) noexcept;
void evaluateLiveMixerModel(const KeyboardInputView& input, EngineControlState& state,
                            HudState& hud) noexcept;
void choreographLiveMixerLeds(const KeyboardInputView& input, float animPhase,
                              float subBreathWave01, const EngineControlState& state,
                              const HudState& hud, uint8_t stutterGateMask,
                              LedState& outLeds) noexcept;

const KeyboardModelOps& keyboardModelOps(KeyboardModel model) noexcept;

} // namespace ruido
