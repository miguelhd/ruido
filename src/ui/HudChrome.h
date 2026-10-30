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

// Pure target-gauge calculator for top header bar & bottom 4-knob physical bead strip.
HudGaugeTargets
computeHudGaugeTargets(const EngineControlState& control, const HudState& hud,
                       const std::array<float, kNumOscillators>& lfoOutput) noexcept;

void compositeIdlePeakMeters(const MonoStripCanvas& idleCv, float smoothedPeakL,
                             float smoothedPeakR) noexcept;

void compositeTopHeaderStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                             const HudState& hud,
                             const std::array<float, kNumOscillators>& lfoOutput,
                             float headerBarSmooth, float finePhase, float coarsePhase) noexcept;

void compositeBottomKnobStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                              const HudState& hud,
                              const std::array<float, kNumOscillators>& lfoOutput,
                              const std::array<float, kNumOscillators>& knobPosSignedSmooth,
                              const std::array<float, kNumOscillators>& knobDimSmooth,
                              const std::array<float, kNumOscillators>& knobFocusSmooth) noexcept;

} // namespace ruido
