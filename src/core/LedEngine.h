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
    float savePulse01 = 0.5f;
    std::array<uint8_t, 4> glintChannels{0xFFu, 0xFFu, 0xFFu, 0xFFu};
    std::array<uint8_t, 4> glintPhaseQ8{0u, 0u, 0u, 0u};
    uint8_t nextGlintSlot = 0u;
    uint8_t lastPhantomSeq = 0u;
    uint16_t presetSceneLedQ16 = 0u;

    void reset(const EngineControlState& state) noexcept;
};

struct LedInputContext {
    const EngineControlState& state;
    const HudState& hud;
    const EngineTelemetry& telemetry;
    float keyPunchEnergy01 = 0.0f;
};

// Advance smooth LED transition states by `dtSec` and compute panel button PWM channels.
void stepAndComputePanelLeds(float dtSec, const LedInputContext& ctx, LedSmoothState& smooth,
                             LedState& outLeds) noexcept;

// Mirror live `SCENE 1-4` weights (`15% / 40%..80%`) onto `Keys 9-12` strictly while turning the
// `PRESETS` scene knob, fading smoothly back to normal keyboard LEDs once the turn toast ends.
void applyPresetKnobSceneLeds(float dtSec, const HudState& hud, bool toastHoldActive,
                              LedSmoothState& smooth, LedState& ioLeds) noexcept;

// Overlay active 120 ms `0% -> 15% -> 0%` glints onto any unassigned button or key channel.
void applyUnassignedLedGlints(const LedSmoothState& smooth, LedState& ioLeds) noexcept;

} // namespace ruido
