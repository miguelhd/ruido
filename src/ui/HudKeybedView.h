#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Format top HUD header strings & optional bar gauge for KeyboardModel (SEL) and
// KeyboardPlay (momentary 27-key punch-ins routed 1-to-1 to native Ruido views).
void formatKeybedTopHeader(const HudState& hud, char* leftMain, size_t leftMainCap,
                           char* leftSuffix, size_t leftSuffixCap, char* rightText, size_t rightCap,
                           bool& outShowBar, float& outBarNorm) noexcept;

// Reusable scanline-composited Keyboard Model Status & Guidance View (active strictly on SEL /
// HudMode::KeyboardModel; 0 B extra framebuffer RAM). Renders the swappable model carousel,
// anatomically accurate SDF rounded-capsule FM-1 keybed map with live varibright zone slits,
// and non-overlapping model zone guidance.
class HudKeybedView {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    bool isActive(const HudState& hud) const noexcept {
        return hud.opacity > 0.004f && hud.mode == HudMode::KeyboardModel;
    }

    bool hasResidualPhase() const noexcept { return phase_ > 0.002f; }

    float phase() const noexcept { return phase_; }

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const std::array<float, kNumOscillators>& voicePeak,
                uint8_t stutterGateMask) noexcept;

    void compositeStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                        const HudState& frameHud) const noexcept;

  private:
    std::array<float, kNumBlackKeys> blackGlow_{};
    std::array<float, kNumWhiteKeys> whiteGlow_{};
    float phase_ = 0.0f;
    float breathPhase_ = 0.0f;
    bool primed_ = false;
};

} // namespace ruido
