#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"

namespace ruido {

// Centralized 150 ms HUD fade-in / fade-out duration and 1.5 s transient toast hold.
// Shared across controller opacity, HOME LED, center acoustic graphs, and 72% backdrop dim.
inline constexpr float kHudFadeSec = 0.150f;
inline constexpr float kHudToastHoldSec = 1.500f;

// Advance any [0, 1] linear phase toward 1.0 (active=true) or 0.0 (active=false) in 150 ms.
inline float stepHudFadeLinear(float level, bool active, float dtSec) noexcept {
    const float delta = dtSec * (1.0f / kHudFadeSec);
    return active ? clampf(level + delta, 0.0f, 1.0f) : clampf(level - delta, 0.0f, 1.0f);
}

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
        holdRemainingSec = kHudToastHoldSec;
    }

    void latchPage() noexcept {
        active = true;
        holdRemainingSec = 0.0f;
    }

    // Begin smooth 150 ms fade-out from current rawLevel (or deactivate immediately if unlit).
    bool beginFadeOut() noexcept {
        holdRemainingSec = 0.0f;
        if (rawLevel <= 0.001f) {
            rawLevel = 0.0f;
            active = false;
            return true;
        }
        active = true;
        return false;
    }

    void step(float dt, bool momentaryPinned, bool switchLatched, bool& outToastExpired,
              bool& outDissolvedToIdle, float& outOpacity) noexcept {
        outToastExpired = false;
        outDissolvedToIdle = false;

        if (momentaryPinned) {
            // Do not overwrite holdRemainingSec if already zeroed by rate-only tap;
            // pinned state keeps HUD blooming toward 1.0 while held.
            active = true;
        } else if (switchLatched) {
            active = true;
        }

        if (!active) {
            outOpacity = 0.0f;
            return;
        }

        if (momentaryPinned) {
            rawLevel = stepHudFadeLinear(rawLevel, true, dt);
        } else if (switchLatched) {
            rawLevel = stepHudFadeLinear(rawLevel, true, dt);
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
                const float needBloom = (1.0f - rawLevel) * kHudFadeSec;
                const float bloomDt = (remDt < needBloom) ? remDt : needBloom;
                rawLevel = stepHudFadeLinear(rawLevel, true, bloomDt);
                remDt -= bloomDt;
            }
            if (remDt > 0.0f && holdRemainingSec > 0.0f) {
                const float holdDt = (remDt < holdRemainingSec) ? remDt : holdRemainingSec;
                holdRemainingSec -= holdDt;
                remDt -= holdDt;
            }
            if (remDt > 0.0f && holdRemainingSec <= 0.0f) {
                rawLevel = stepHudFadeLinear(rawLevel, false, remDt);
                if (rawLevel <= 0.001f) {
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
