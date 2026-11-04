#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// Edge-to-edge Stereo Panorama Curve (SEL -> PAN, GLOBAL LEVEL K4; HudGraphs sister to FX & LFO)
// and flat vertical GLOBAL CONFIG list menu (right GLO).
class HudGlobalViews {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const LfoSignalArray& lfoOutput,
                const std::array<float, kNumOscillators>& voicePeak,
                uint8_t stutterGateMask) noexcept;

    float panPhase() const noexcept { return panPhase_; }

    float configPhase() const noexcept { return configPhase_; }

    bool anyActiveTarget(const HudState& rawHud) const noexcept;
    bool hasResidualPhase() const noexcept;

    void compositePanStageStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                uint16_t* linScratch, float effPanAlpha01, uint8_t alpha8,
                                const HudState& hud) const noexcept;
    void compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                    const EngineControlState& control,
                                    const HudState& hud) const noexcept;

  private:
    static constexpr bool isPanStageActive(const HudState& rawHud) noexcept {
        return (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None) &&
               (rawHud.knobBank == KnobBankMode::Panning ||
                (rawHud.knobBank == KnobBankMode::GlobalAmp && rawHud.focusedKnob == 3u));
    }

    std::array<float, kNumOscillators> panX_{78.0f, 162.0f, 48.0f, 192.0f};
    std::array<float, kNumOscillators> panBaseX_{78.0f, 162.0f, 48.0f, 192.0f};
    std::array<float, kNumOscillators> orbitXMin_{78.0f, 162.0f, 48.0f, 192.0f};
    std::array<float, kNumOscillators> orbitXMax_{78.0f, 162.0f, 48.0f, 192.0f};
    std::array<float, kNumOscillators> panLfoDepth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> apexH_{88.0f, 88.0f, 88.0f, 88.0f};
    std::array<float, kNumOscillators> invWidth2_{0.003f, 0.0035f, 0.004f, 0.005f};
    std::array<float, kNumOscillators> focusWeight_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> muteDim_{0.0f, 0.0f, 0.0f, 0.0f};

    float hasFocusSmooth_ = 0.0f;
    float panPhase_ = 0.0f;
    float configPhase_ = 0.0f;
    bool monoCollapse_ = false;
    bool primed_ = false;
};

} // namespace ruido
