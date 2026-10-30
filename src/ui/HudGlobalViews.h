#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// Custom center-screen compositors for GLOBAL FREQ, GLOBAL AMP, GLOBAL ALL stem overlay,
// and the scalable vertical GLOBAL CONFIG list menu.
class HudGlobalViews {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const std::array<float, kNumOscillators>& lfoOutput) noexcept;

    float freqPhase() const noexcept { return freqPhase_; }

    float ampPhase() const noexcept { return ampPhase_; }

    float allOverlayPhase() const noexcept { return allOverlayPhase_; }

    float configPhase() const noexcept { return configPhase_; }

    bool anyActiveTarget(const HudState& rawHud) const noexcept;
    bool hasResidualPhase() const noexcept;

    void compositeGlobalFreqStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8) const noexcept;
    void compositeGlobalAmpStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8) const noexcept;
    void compositeGlobalAllOverlayStrip(int y0, int y1, uint8_t* lumBuf,
                                        uint8_t alpha8) const noexcept;
    void compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                    const EngineControlState& control,
                                    const HudState& hud) const noexcept;

  private:
    std::array<float, kNumOscillators> freqStemX_{40.0f, 90.0f, 140.0f, 190.0f};
    std::array<float, kNumOscillators> freqStemTopY_{180.0f, 180.0f, 180.0f, 180.0f};
    std::array<float, kNumOscillators> freqStemInt_{0.0f, 0.0f, 0.0f, 0.0f};

    std::array<float, kNumOscillators> ampStemX_{60.0f, 100.0f, 140.0f, 180.0f};
    std::array<float, kNumOscillators> ampStemTopY_{180.0f, 180.0f, 180.0f, 180.0f};
    std::array<float, kNumOscillators> ampStemInt_{0.0f, 0.0f, 0.0f, 0.0f};

    std::array<float, kNumOscillators> allStemX_{40.0f, 70.0f, 100.0f, 130.0f};
    std::array<float, kNumOscillators> allStemTopY_{186.0f, 186.0f, 186.0f, 186.0f};
    std::array<float, kNumOscillators> allStemInt_{0.0f, 0.0f, 0.0f, 0.0f};

    float freqPhase_ = 0.0f;
    float ampPhase_ = 0.0f;
    float allOverlayPhase_ = 0.0f;
    float configPhase_ = 0.0f;
};

} // namespace ruido
