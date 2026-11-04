#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>

namespace ruido {

struct LedSmoothState {
    float animPhase = 0.0f;
    float playStopRunSmooth = 1.0f;
    float fxSync01 = 0.0f;
    float lfoSync01 = 0.0f;
    float subBreathWave01 = 0.5f;

    void reset(const EngineControlState& state) noexcept;
};

struct LedInputContext {
    const EngineControlState& state;
    const HudState& hud;
    const EngineTelemetry& telemetry;
    float keyPunchEnergy01 = 0.0f;
};

// Advance smooth LED transition states by `dtSec` and compute all 42 hardware software-PWM
// channels (27 white keys + 12 white buttons + dual Green/Red-Orange PLAY/STOP + Red REC).
void stepAndComputePanelLeds(float dtSec, const LedInputContext& ctx, LedSmoothState& smooth,
                             LedState& outLeds) noexcept;

} // namespace ruido
