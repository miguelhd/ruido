#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/MonoCanvas.h"

#include <array>
#include <cstdint>

namespace ruido {

struct HudGaugeTargets {
    std::array<float, kNumOscillators> posSigned{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> dimAmount{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<bool, kNumOscillators> focused{false, false, false, false};
    float headerBarNorm = 0.0f;
    bool hasHeaderBar = false;
};

struct PreparedHudColumn {
    char roleLbl[12]{};
    char valStr[16]{};
    std::array<float, kNumOscillators> voiceSigned{0.0f, 0.0f, 0.0f, 0.0f};
    float fxLevel01 = 0.0f;
    float lfoALevel01 = 0.0f;
    float lfoBLevel01 = 0.0f;
    int8_t valTrk = 0;
    bool bipolar = false;
    bool disabled = false;
    bool hasVoiceSpread = false;
    bool fxOn = false;
    bool lfoAOn = false;
    bool lfoBOn = false;
};

struct PreparedHudChrome {
    std::array<PreparedHudColumn, kNumOscillators> cols{};
};

// Pure target-gauge calculator for top header bar & bottom 4-knob physical bead strip.
HudGaugeTargets computeHudGaugeTargets(const EngineControlState& control, const HudState& hud,
                                       const LfoSignalArray& lfoOutput,
                                       uint8_t stutterGateMask) noexcept;

void prepareHudChrome(PreparedHudChrome& out, const EngineControlState& control,
                      const HudState& hud, const LfoSignalArray& lfoOutput,
                      uint8_t stutterGateMask) noexcept;

void compositeBottomKnobStrip(const MonoStripCanvas& cv, const PreparedHudChrome& prep,
                              const EngineControlState& control, const HudState& hud,
                              const std::array<float, kNumOscillators>& knobPosSignedSmooth,
                              const std::array<float, kNumOscillators>& knobDimSmooth,
                              const std::array<float, kNumOscillators>& knobFocusSmooth) noexcept;

} // namespace ruido
