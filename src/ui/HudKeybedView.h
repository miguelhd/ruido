#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

struct MonoStripCanvas;

// Format top HUD header strings & optional bar/scene gauge for KeyboardModel (SEL),
// ShiftMap (universal SHIFT layer), and KeyboardPlay (momentary 27-key punch-ins).
void formatKeybedTopHeader(const HudState& hud, char* leftMain, size_t leftMainCap,
                           char* leftSuffix, size_t leftSuffixCap, char* rightText, size_t rightCap,
                           bool& outShowBar, float& outBarNorm,
                           bool* outShowSceneDots = nullptr) noexcept;

// Shared physical FM-1 Two-Bay (`20 + 7`) keybed & mode-carousel layout constants:
//  - Left 20-Pad Analog Pressure Bay (`x = 7.0 .. 166.5`): `OP1..GLO` + `Keys 1-12` (`4 · 4 · 4`)
//  - Right 7-Switch Digital Bay (`x = 170.5 .. 233.0`): `HOLD, TAP, SHIFT` + `MUTE 1-4`
inline constexpr int kKeybedCarouselArrowLeftX = 18;
inline constexpr int kKeybedCarouselArrowRightX = 222;
inline constexpr int kKeybedCarouselArrowY = 34;
inline constexpr int kKeybedCarouselTitleY = 33;
inline constexpr float kKeybedDiagramTopY = 92.0f;
inline constexpr float kKeybedBracketOverhangX = 3.2f;
inline constexpr float kKeybedBracketTickTopY = 167.4f;
inline constexpr float kKeybedBracketY = 170.0f;
inline constexpr int kKeybedBracketLabelY = 175;
inline constexpr int kKeybedFooterLine1Y = 202;
inline constexpr int kKeybedFooterLine2Y = 217;

inline constexpr float whiteKeyDiagramCenterX(size_t whiteIdx) noexcept {
    return (whiteIdx < kMuteWhiteKeyBase)
               ? (17.5f + static_cast<float>(whiteIdx) * 12.2f +
                  static_cast<float>(whiteIdx >> 2u) * 2.0f)
               : (181.5f + static_cast<float>(whiteIdx - kMuteWhiteKeyBase) * 13.5f);
}

inline constexpr float blackKeyDiagramCenterX(size_t blackIdx) noexcept {
    constexpr std::array<float, kNumBlackKeys> kCenters = {
        23.60f,  35.80f,  48.00f,  // OP1, OP2, OP3 (gaps 0-1, 1-2, 2-3)
        74.40f,  86.60f,           // OP4, OP5      (gaps 4-5, 5-6)
        112.00f, 125.20f, 137.40f, // OP6, PIT, GLO (gaps 7-8, 8-9, 9-10)
        181.50f, 195.00f,          // MONO (HOLD), POLY (TAP)
        221.50f,                   // 11th key (SHIFT)
    };
    return (blackIdx < kNumBlackKeys) ? kCenters[blackIdx] : kCenters[kNumBlackKeys - 1u];
}

void compositeFm1KeybedDiagram(const MonoStripCanvas& cv, float topY,
                               const std::array<float, kNumBlackKeys>& blackGlow,
                               const std::array<float, kNumWhiteKeys>& whiteGlow,
                               const HudState* liveHud = nullptr,
                               uint16_t activeBlackMask = 0x07FFu,
                               uint16_t activeWhiteMask = 0xFFFFu,
                               void (*coProcYield)() noexcept = nullptr) noexcept;

// Reusable scanline-composited Two-Bay Keyboard Explorer (`SEL`) & Universal Full-Chassis
// `SHIFT` Map (`HudMode::ShiftMap`; `0 B` extra framebuffer RAM).
class HudKeybedView {
  public:
    static constexpr float kIdleHoldSec = 5.0f;
    static constexpr float kIdleDimSec = 5.0f;
    static constexpr float kWakeRiseSec = 0.15f;
    static constexpr float kDimFloorBrightness = 0.50f;

    void reset() noexcept;
    void clearImmediate() noexcept;

    bool isActive(const HudState& hud) const noexcept {
        if (hud.opacity <= 0.004f || hud.shiftHeld) {
            return false;
        }
        return hud.mode != HudMode::ShiftMap &&
               (hud.selPageOpen || (hud.mode == HudMode::KeyboardModel && hud.opacity >= 0.99f));
    }

    bool isShiftActive(const HudState& hud) const noexcept {
        return hud.opacity > 0.004f &&
               (hud.shiftHeld || (hud.mode == HudMode::ShiftMap && hud.opacity >= 0.99f));
    }

    bool hasResidualPhase() const noexcept { return phase_ > 0.002f || shiftPhase_ > 0.002f; }

    float phase() const noexcept { return phase_; }

    float shiftPhase() const noexcept { return shiftPhase_; }

    float brightnessScale() const noexcept;

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const std::array<float, kNumOscillators>& voicePeak, uint8_t stutterGateMask,
                float rhythmPhase01 = 0.0f) noexcept;

    void compositeStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8, const HudState& frameHud,
                        void (*coProcYield)() noexcept = nullptr) const noexcept;

    void compositeShiftStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                             const EngineControlState& control, const HudState& frameHud,
                             void (*coProcYield)() noexcept = nullptr) const noexcept;

  private:
    std::array<float, kNumBlackKeys> blackGlow_{};
    std::array<float, kNumWhiteKeys> whiteGlow_{};
    float phase_ = 0.0f;
    float shiftPhase_ = 0.0f;
    float idleTimerSec_ = 0.0f;
    float brightPhase_ = 1.0f;
    uint8_t lastWakeSeq_ = 0u;
    bool primed_ = false;
    bool wasSelPageOpen_ = false;
};

} // namespace ruido
