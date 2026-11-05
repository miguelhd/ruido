#pragma once

#include "core/ControlSurface.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardEngine.h"
#include "core/LedEngine.h"
#include "core/MidiClock.h"
#include "core/ModifierTracker.h"
#include "core/Parameters.h"
#include "core/SnapshotBank.h"

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

    // Raised black keys (OP1-OP6, PIT, GLO, MONO, POLY, 11th key): gate + continuous pressure
    void onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept;
    void onBlackKeyPressure(BlackKeyId key, float pressure) noexcept;

    // Bottom 16 white keys: digital gate + continuous SARADC carbon-pill pressure [0.0f, 1.0f]
    void onWhiteKeyGate(uint8_t whiteKeyIndex, bool pressed) noexcept;
    void onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept;

    // Transport-agnostic 1-byte MIDI System Real-Time clock sync (0xF8 / 0xFA / 0xFB / 0xFC)
    // across TRS UART, USB-MIDI, and Bluetooth BLE-MIDI (`msgTimestampSec < 0` uses internal time).
    void onMidiRealtime(uint8_t statusByte, float msgTimestampSec = -1.0f) noexcept;

    // Channel filter query (`MIDI CHANNEL IN`: `OMNI` or `CH 1..16`) for incoming MIDI messages.
    bool acceptsMidiChannel(uint8_t statusByte) const noexcept {
        return ruido::acceptsMidiChannel(hud_.config.midiChannel, statusByte);
    }

    // Advance 150 ms HUD envelope, MIDI clock timeout & refresh all 42 varibright PWM LED levels.
    void tick(float dtSec, const EngineTelemetry& telemetry) noexcept;

    const EngineControlState& state() const noexcept { return state_; }

    const HudState& hud() const noexcept { return hud_; }

    const LedState& leds() const noexcept { return leds_; }

    const SnapshotBank& snapshots() const noexcept { return snapshots_; }

    bool consumeStateDirty() noexcept {
        const bool dirty = stateDirty_;
        stateDirty_ = false;
        return dirty;
    }

    // Bidirectional `zvenson/dinghy` NVS flash ports (`slot_*` 128 B & `settings_*` 16 B):
    // Pure in-RAM CRC-32 POD transfers with zero raw SPI NOR flash register access in `src/`.
    bool loadSnapshotSlot(uint8_t slot, const PatchSnapshot& snap) noexcept;
    uint8_t consumeSavedSlotMask() noexcept {
        const uint8_t mask = savedSlotMask_;
        savedSlotMask_ = 0u;
        return mask;
    }
    DeviceSettingsBlob exportSettings() const noexcept;
    bool loadSettings(const DeviceSettingsBlob& blob) noexcept;
    bool consumeSettingsDirty() noexcept {
        const bool dirty = settingsDirty_;
        settingsDirty_ = false;
        return dirty;
    }

  private:
    KnobBankMode baseKnobBank() const noexcept;
    bool anyPageOpen() const noexcept {
        return hud_.freqPageOpen || hud_.fxPageOpen || hud_.panPageOpen || hud_.lfoPageOpen ||
               hud_.editPageOpen || hud_.configPageOpen || hud_.selPageOpen;
    }
    void closeAllPages() noexcept {
        hud_.freqPageOpen = hud_.fxPageOpen = hud_.panPageOpen = false;
        hud_.lfoPageOpen = hud_.editPageOpen = hud_.configPageOpen = hud_.selPageOpen = false;
    }
    void resetPageNav() noexcept;
    void setLatchedPage(HudMode mode, ButtonId button) noexcept;
    void restoreLatchedFields() noexcept;
    bool syncClockState() noexcept;
    bool tryInterceptSnapSaveKey(uint8_t whiteKeyIndex, bool active) noexcept;
    void applyKeyboardPerformanceChange() noexcept;
    void syncModifierHud() noexcept;
    void restoreOrDissolveHud() noexcept;
    void triggerHud(HudMode mode, uint8_t voiceIndex = 0u,
                    uint8_t knobIndex = kNoFocusedKnob) noexcept;
    void latchSwitchHud(HudMode mode, ButtonId button = ButtonId::Count) noexcept;
    void latchKnobEdit(KnobBankMode bank, HudMode mode, uint8_t knobIdx, uint8_t voiceIdx) noexcept;
    void publishState() noexcept;
    void updateLeds(float dtSec, const EngineTelemetry& telemetry) noexcept;

    LockFreeControlBus* controlBus_ = nullptr;
    EngineControlState state_{};
    HudState hud_{};
    LedState leds_{};

    KeyboardEngine keyboard_{};
    SnapshotBank snapshots_{};
    MidiClockTracker midiClock_{};
    ModifierKeyTracker keys_{};
    HudEnvelopeTracker env_{};
    LedSmoothState ledSmooth_{};
    bool knobTurnedDuringRateHold_ = false;
    bool dismissedByHome_ = false;
    bool stateDirty_ = false;
    bool settingsDirty_ = false;
    uint8_t saveSwallowedMask_ = 0u;
    uint8_t savedSlotMask_ = 0u;
    int8_t selectTickAccum_ = 0;
    int8_t presetTickAccum_ = 0;
    int8_t targetTickAccum_ = 0;
    int8_t configTickAccum_ = 0;
    HudMode latchedHudMode_ = HudMode::None;
    ButtonId latchedButton_ = ButtonId::Count;
    float masterPot_ = 0.85f;
    float masterPotHudAnchor_ = 0.85f;
};

} // namespace ruido
