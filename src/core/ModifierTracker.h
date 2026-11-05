#pragma once

#include "core/ControlSurface.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

enum class HeldModifier : uint8_t { None = 0, Global = 1 };

constexpr bool isModifierBlackKey(BlackKeyId) noexcept {
    return false; // All 11 black keys belong to the momentary Keyboard Performance surface
}

// LIFO momentary key & knob-rate tracker for two-hand chording without dynamic containers.
struct ModifierKeyTracker {
    uint32_t pressSeq = 0u;
    uint32_t octMinusOrder = 0u;
    uint32_t octPlusOrder = 0u;
    bool octMinusHeld = false;
    bool octPlusHeld = false;
    bool anyPerfKeyHeld = false;

    void reset() noexcept {
        pressSeq = 0u;
        octMinusOrder = 0u;
        octPlusOrder = 0u;
        octMinusHeld = false;
        octPlusHeld = false;
        anyPerfKeyHeld = false;
    }

    bool setRateButton(ButtonId button, bool pressed) noexcept {
        const bool wasRateHeld = octMinusHeld || octPlusHeld;
        if (button == ButtonId::OctMinus) {
            octMinusHeld = pressed;
            if (pressed) {
                octMinusOrder = ++pressSeq;
            }
        } else if (button == ButtonId::OctPlus) {
            octPlusHeld = pressed;
            if (pressed) {
                octPlusOrder = ++pressSeq;
            }
        }
        return wasRateHeld;
    }

    bool isRateHeld() const noexcept { return octMinusHeld || octPlusHeld; }

    HeldModifier activeModifier() const noexcept { return HeldModifier::None; }

    bool isAnyPerformanceKeyHeld() const noexcept { return anyPerfKeyHeld; }

    KnobRate activeKnobRate() const noexcept {
        if (octMinusHeld && octPlusHeld) {
            return (octPlusOrder >= octMinusOrder) ? KnobRate::Coarse : KnobRate::Fine;
        }
        if (octMinusHeld) {
            return KnobRate::Fine;
        }
        if (octPlusHeld) {
            return KnobRate::Coarse;
        }
        return KnobRate::Standard;
    }

    bool isAnyMomentaryHudHoldActive(bool scopeFrozen) const noexcept {
        return scopeFrozen || isRateHeld() || isAnyPerformanceKeyHeld();
    }
};

} // namespace ruido
