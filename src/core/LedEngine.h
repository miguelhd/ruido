#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>

namespace ruido {

struct LedInputContext {
    const EngineControlState& state;
    const HudState& hud;
    const EngineTelemetry& telemetry;
    const std::array<bool, kNumBlackKeys>& blackKeyHeld;
    const std::array<bool, kNumWhiteKeys>& whiteKeyHeld;
    float animPhase;
    float playStopRunSmooth;
};

// Pure 42-channel hardware software-PWM brightness calculator (27 white keys + 12 white buttons +
// dual-channel Green/Red-Orange PLAY/STOP + Red REC).
void computePanelAndKeybedLeds(const LedInputContext& ctx, LedState& outLeds) noexcept;

} // namespace ruido
