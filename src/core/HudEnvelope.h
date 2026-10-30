#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"

namespace ruido {

inline constexpr float kHudBloomSec = 0.200f;
inline constexpr float kHudHoldSec = 1.500f;
inline constexpr float kHudDissolveTau = 0.250f;

// Centralized HUD fade-in / hold / fade-out envelope state for Core 0 controller ticks.
struct HudEnvelopeTracker {
    float rawLevel = 0.0f;
    float holdRemainingSec = 0.0f;
    bool active = false;

    void reset() noexcept {
        rawLevel = 0.0f;
        holdRemainingSec = 0.0f;
        active = false;
    }

    void triggerToast() noexcept {
        active = true;
        holdRemainingSec = kHudHoldSec;
    }

    void latchPage() noexcept {
        active = true;
        holdRemainingSec = 0.0f;
    }

    void step(float dt, bool momentaryPinned, bool switchLatched, bool& outToastExpired,
              bool& outDissolvedToIdle, float& outOpacity) noexcept {
        outToastExpired = false;
        outDissolvedToIdle = false;

        if (momentaryPinned) {
            holdRemainingSec = kHudHoldSec;
            active = true;
        } else if (switchLatched) {
            active = true;
        }

        if (!active) {
            outOpacity = 0.0f;
            return;
        }

        if (momentaryPinned) {
            rawLevel = clampf(rawLevel + dt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
        } else if (switchLatched) {
            rawLevel = clampf(rawLevel + dt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
            if (holdRemainingSec > 0.0f) {
                holdRemainingSec -= dt;
                if (holdRemainingSec <= 0.0f) {
                    holdRemainingSec = 0.0f;
                    outToastExpired = true;
                }
            }
        } else {
            float remDt = dt;
            if (holdRemainingSec > 0.0f && rawLevel < 1.0f) {
                const float needBloom = (1.0f - rawLevel) * kHudBloomSec;
                const float bloomDt = (remDt < needBloom) ? remDt : needBloom;
                rawLevel = clampf(rawLevel + bloomDt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
                remDt -= bloomDt;
            }
            if (remDt > 0.0f && holdRemainingSec > 0.0f) {
                const float holdDt = (remDt < holdRemainingSec) ? remDt : holdRemainingSec;
                holdRemainingSec -= holdDt;
                remDt -= holdDt;
            }
            if (remDt > 0.0f && holdRemainingSec <= 0.0f) {
                const float decay = fastExp(-remDt * (1.0f / kHudDissolveTau));
                rawLevel = rawLevel * decay - 0.035f * remDt;
                if (rawLevel <= 0.005f) {
                    rawLevel = 0.0f;
                    active = false;
                    outDissolvedToIdle = true;
                }
            }
        }
        outOpacity = smoothstep01(rawLevel);
    }
};

} // namespace ruido
