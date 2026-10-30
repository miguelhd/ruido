#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// Full-screen (x = 0..239, y = 26..188) 4-channel Optical VCA Level & MS-20 Pre-Filter Drive
// visualization for the latching EDIT page:
//  - 4 vertical optical columns aligned directly above KNOB 1-4 (cx = 30, 90, 150, 210)
//  - 0% floor horizon (y = 180) and 70% (1.0x Unity / MS-20 Overdrive Knee) horizon (y = 76.4)
//  - Live slewed post-filter acoustic plasma columns driven by EngineTelemetry::voicePeak[0..3]
//  - Symmetrical linear-photon saturation wings + incandescent beacon corona above 70% drive knee
class HudAmpGraph {
  public:
    static constexpr float kColYTop = 36.0f;
    static constexpr float kColYBot = 180.0f;
    static constexpr float kColYSpan = kColYBot - kColYTop;               // 144.0f
    static constexpr float kKneeY = kColYBot - kCleanAmpKnee * kColYSpan; // 79.2f

    void reset() noexcept;
    void clearImmediate() noexcept;
    void stepFadeOut(float dtClamped) noexcept;

    bool isActive(const HudState& rawHud) const noexcept {
        return rawHud.opacity > 0.001f && rawHud.editPageOpen &&
               rawHud.knobBank == KnobBankMode::Amplitude;
    }

    bool hasResidualPhase() const noexcept { return phase_ > 0.002f; }

    float phase() const noexcept { return phase_; }

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const std::array<float, kNumOscillators>& lfoOutput,
                const std::array<float, kNumOscillators>& voicePeak) noexcept;

    void compositeStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf, uint16_t* linScratch,
                        float effAlpha01, uint8_t alpha8, const HudState& hud) const noexcept;

  private:
    std::array<float, kNumOscillators> faderY_{kColYBot, kColYBot, kColYBot, kColYBot};
    std::array<float, kNumOscillators> driveNorm_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> driveRatio_{1.0f, 1.0f, 1.0f, 1.0f};
    std::array<float, kNumOscillators> peakY_{kColYBot, kColYBot, kColYBot, kColYBot};
    std::array<float, kNumOscillators> muteDim_{0.0f, 0.0f, 0.0f, 0.0f};
    float phase_ = 0.0f;
    bool primed_ = false;
};

} // namespace ruido
