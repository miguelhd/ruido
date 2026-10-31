#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// 4-lane Binaural Constant-Power Stereo Soundstage (SEL -> PAN, GLOBAL LEVEL K4)
// and flat vertical GLOBAL CONFIG list menu (right GLO).
class HudGlobalViews {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const LfoSignalArray& lfoOutput,
                const std::array<float, kNumOscillators>& voicePeak) noexcept;

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

    std::array<float, kNumOscillators> panSigned_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> panBase_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> panOrbitMin_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> panOrbitMax_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> panLfoDepth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> voicePresence_{0.5f, 0.5f, 0.5f, 0.5f};
    std::array<float, kNumOscillators> muteDim_{0.0f, 0.0f, 0.0f, 0.0f};

    float panPhase_ = 0.0f;
    float configPhase_ = 0.0f;
};

} // namespace ruido
