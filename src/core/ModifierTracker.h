#pragma once

#include "core/ControlSurface.h"

#include <cstdint>

namespace ruido {

// LIFO momentary OCT-/OCT+ knob-rate & performance-key hold tracker without dynamic containers.
struct ModifierKeyTracker {
    uint16_t pressSeq = 0u;
    uint16_t octMinusOrder = 0u;
    uint16_t octPlusOrder = 0u;
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
                octMinusOrder = nextOrder();
            }
        } else if (button == ButtonId::OctPlus) {
            octPlusHeld = pressed;
            if (pressed) {
                octPlusOrder = nextOrder();
            }
        }
        if (!octMinusHeld && !octPlusHeld) {
            pressSeq = 0u;
            octMinusOrder = 0u;
            octPlusOrder = 0u;
        }
        return wasRateHeld;
    }

    uint16_t nextOrder() noexcept {
        if (pressSeq == 0xFFFFu) {
            if (octMinusOrder <= octPlusOrder) {
                octMinusOrder = octMinusHeld ? 1u : 0u;
                octPlusOrder = octPlusHeld ? 2u : 0u;
            } else {
                octPlusOrder = octPlusHeld ? 1u : 0u;
                octMinusOrder = octMinusHeld ? 2u : 0u;
            }
            pressSeq = 2u;
        }
        return ++pressSeq;
    }

    bool isRateHeld() const noexcept { return octMinusHeld || octPlusHeld; }

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
