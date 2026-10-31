#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/GraphStyle.h"
#include "ui/MonoCanvas.h"
#include "ui/ScopeTraceMath.h"

#include <array>
#include <cstdint>

namespace ruido {

// Full-screen (x = 0..239, y = 26..188) 4-channel oscillator visualizations:
//  - LEVEL: Clean optical VCA stems (0-70%) & 3A narrow triangular overdrive cap (> 70%)
//           shared across EDIT -> LEVEL, GLOBAL LEVEL (K1-K3), and GLOBAL ALL (K4).
//  - DUTY:  4-column CMOS Schmitt single-cycle pulse duty cells (5% needle .. 50% sq .. 95% wide)
//  - DRIFT: 4-column thermal excursion brackets + live Core 1 1/f pitch wander beacons
class HudAmpGraph {
  public:
    static constexpr float kColYTop = 36.0f;
    static constexpr float kColYBot = kGraphBaselineY;                    // 186.0f
    static constexpr float kColYSpan = kColYBot - kColYTop;               // 150.0f
    static constexpr float kKneeY = kColYBot - kCleanAmpKnee * kColYSpan; // 81.0f

    void reset() noexcept;
    void clearImmediate() noexcept;
    void stepFadeOut(float dtClamped) noexcept;

    static constexpr bool isLevelViewActive(const HudState& rawHud) noexcept {
        return rawHud.opacity > 0.001f &&
               ((rawHud.editPageOpen && !rawHud.modifierHeld &&
                 rawHud.knobBank == KnobBankMode::Amplitude) ||
                (rawHud.knobBank == KnobBankMode::GlobalAmp && rawHud.focusedKnob != 3u) ||
                (rawHud.knobBank == KnobBankMode::GlobalAll && rawHud.focusedKnob == 3u));
    }

    bool isActive(const HudState& rawHud) const noexcept {
        const bool editVis = rawHud.opacity > 0.001f && rawHud.editPageOpen && !rawHud.modifierHeld;
        return isLevelViewActive(rawHud) ||
               (editVis && (rawHud.knobBank == KnobBankMode::PulseWidth ||
                            rawHud.knobBank == KnobBankMode::VoiceDrift));
    }

    bool hasResidualPhase() const noexcept {
        return phase_ > 0.002f || widthPhase_ > 0.002f || driftPhase_ > 0.002f;
    }

    float maxPhase() const noexcept {
        const float hi = (phase_ > widthPhase_) ? phase_ : widthPhase_;
        return (hi > driftPhase_) ? hi : driftPhase_;
    }

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const LfoSignalArray& lfoOutput,
                const std::array<float, kNumOscillators>& voiceDriftNorm,
                const OverdriveHairTable& overdriveHair) noexcept;

    void compositeStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf, uint16_t* linScratch,
                        float hudAlpha01, const HudState& hud) const noexcept;

  private:
    void compositeLevelStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                             uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                             const HudState& hud) const noexcept;
    void compositeWidthStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                             uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                             const HudState& hud) const noexcept;
    void compositeDriftStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                             uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                             const HudState& hud) const noexcept;

    std::array<float, kNumOscillators> faderY_{kColYBot, kColYBot, kColYBot, kColYBot};
    std::array<float, kNumOscillators> driveNorm_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> pulseWidth_{0.50f, 0.50f, 0.50f, 0.50f};
    std::array<float, kNumOscillators> driftAmt_{0.10f, 0.10f, 0.10f, 0.10f};
    std::array<float, kNumOscillators> driftLive_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> muteDim_{0.0f, 0.0f, 0.0f, 0.0f};
    OverdriveHairTable overdriveHair_{};
    float phase_ = 0.0f;
    float widthPhase_ = 0.0f;
    float driftPhase_ = 0.0f;
    bool primed_ = false;
};

} // namespace ruido
