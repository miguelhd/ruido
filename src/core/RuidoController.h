#pragma once

#include "core/ControlSurface.h"
#include "core/HudEnvelope.h"
#include "core/LedEngine.h"
#include "core/ModifierTracker.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// Stateless two-hand instrument controller, 3-gear linear X4 quadrature encoder scaling,
// centralized 150 ms S-curve HUD envelope, and 27-key + 14-button varibright LED engine.
class RuidoController {
  public:
    RuidoController() noexcept;

    void reset() noexcept;

    // Bind to Core 1's lock-free control bus; publishes updated EngineControlState automatically.
    void attachControlBus(LockFreeControlBus* bus) noexcept {
        controlBus_ = bus;
        publishState();
    }

    // Endless encoder rotation: `ticks` signed X4 quadrature steps (96 steps = 360 deg turn).
    // Speed is strictly constant/linear per gear (Standard, Hold OCT- Fine, Hold OCT+ Coarse).
    void onEncoderDelta(EncoderId encoder, int32_t ticks) noexcept;

    // Analog MASTER volume potentiometer [0.0f, 1.0f]
    void onMasterPot(float normalizedLevel) noexcept;

    // Tactile panel buttons (14 physical switches: OCT-/OCT+ + right 2x6 grid)
    void onButtonEvent(ButtonId button, bool pressed) noexcept;

    // Raised black keys (OP1-OP6, PIT, GLO, MONO, POLY, ScopeFreezeKey)
    void onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept;

    // Bottom 16 white keys: digital gate + continuous SARADC carbon-pill pressure [0.0f, 1.0f]
    void onWhiteKeyGate(uint8_t whiteKeyIndex, bool pressed) noexcept;
    void onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept;

    // Advance 150 ms HUD envelope & refresh all 42 varibright PWM LED levels.
    void tick(float dtSec, const EngineTelemetry& telemetry) noexcept;

    const EngineControlState& state() const noexcept { return state_; }

    const HudState& hud() const noexcept { return hud_; }

    const LedState& leds() const noexcept { return leds_; }

  private:
    KnobBankMode baseKnobBank() const noexcept;
    void closeAllPages() noexcept;
    void syncModifierHud() noexcept;
    void restoreOrDissolveHud() noexcept;
    void triggerHud(HudMode mode, uint8_t voiceIndex = 0u,
                    uint8_t knobIndex = kNoFocusedKnob) noexcept;
    void latchSwitchHud(HudMode mode, ButtonId button = ButtonId::Count) noexcept;
    void publishState() noexcept;
    void updateLeds(float dtSec, const EngineTelemetry& telemetry) noexcept;

    LockFreeControlBus* controlBus_ = nullptr;
    EngineControlState state_{};
    HudState hud_{};
    LedState leds_{};
    std::array<VoiceParams, kNumOscillators> gloPreHoldVoices_{};

    ModifierKeyTracker keys_{};
    HudEnvelopeTracker env_{};
    LedSmoothState ledSmooth_{};
    bool knobTurnedDuringRateHold_ = false;
    bool gloSnapshotValid_ = false;
    float masterPot_ = 0.85f;
    float masterPotHudAnchor_ = 0.85f;
    HudMode latchedHudMode_ = HudMode::None;
    ButtonId latchedButton_ = ButtonId::Count;
    int32_t selectTickAccum_ = 0;
    int32_t targetTickAccum_ = 0;
    int32_t configTickAccum_ = 0;
};

} // namespace ruido
