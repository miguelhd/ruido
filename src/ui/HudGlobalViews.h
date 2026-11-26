#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

class ScopeTraceEngine;

// Edge-to-edge Stereo Panorama Curve (SEL -> PAN, GLOBAL LEVEL K4; HudGraphs sister to FX & LFO)
// and flat vertical GLOBAL CONFIG list menu (right GLO).
class HudGlobalViews {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const LfoSignalArray& lfoOutput,
                const std::array<float, kNumOscillators>& voicePeak,
                const std::array<float, kNumOscillators>& voiceDriftNorm, uint8_t stutterGateMask,
                const ScopeTraceEngine& traceEngine,
                void (*coProcYield)() noexcept = nullptr) noexcept;

    float panPhase() const noexcept { return panPhase_; }

    float couplingPhase() const noexcept { return couplingPhase_; }

    bool noPressureNotice() const noexcept { return noPressureHoldSec_ > 0.0f; }

    float configPhase() const noexcept { return configPhase_; }

    static constexpr bool isRailCouplingActive(const HudState& rawHud) noexcept {
        return (rawHud.opacity > 0.004f) && !hasNonHomePageOpen(rawHud) && !rawHud.shiftHeld &&
               !rawHud.saveArmed &&
               (rawHud.mode == HudMode::RailCoupling ||
                (rawHud.mode == HudMode::KeyboardPlay &&
                 rawHud.activePunch == KeyPunchKind::RailStarve));
    }

    bool anyActiveTarget(const HudState& rawHud) const noexcept;
    bool hasResidualPhase() const noexcept;

    void compositePanStageStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                uint16_t* linScratch, float effPanAlpha01, uint8_t alpha8,
                                const HudState& hud,
                                void (*coProcYield)() noexcept = nullptr) const noexcept;
    void compositeRailCouplingStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                    uint16_t* linScratch, float effCouplingAlpha01, uint8_t alpha8,
                                    void (*coProcYield)() noexcept = nullptr) const noexcept;
    void compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                    const EngineControlState& control, const HudState& hud,
                                    void (*coProcYield)() noexcept = nullptr) const noexcept;

  private:
    static constexpr bool isPanStageActive(const HudState& rawHud) noexcept {
        return (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None) && !rawHud.shiftHeld &&
               (rawHud.panPageOpen ||
                (!hasPinnedPageOpen(rawHud) && rawHud.knobBank == KnobBankMode::Panning)) &&
               !isRailCouplingActive(rawHud);
    }

    std::array<float, kNumOscillators> panX_{78.0f, 162.0f, 48.0f, 192.0f};
    std::array<float, kNumOscillators> panLfoDepth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> apexH_{88.0f, 88.0f, 88.0f, 88.0f};
    std::array<float, kNumOscillators> invWidth2_{0.003f, 0.0035f, 0.004f, 0.005f};
    std::array<float, kNumOscillators> focusWeight_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> muteDim_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> voicePhase01_{0.0f, 0.21f, 0.42f, 0.73f};
    std::array<float, kNumOscillators> capAct_{0.72f, 0.72f, 0.72f, 0.72f};
    std::array<float, kNumOscillators> couplingFullSpanSmooth_{800.0f, 534.0f, 400.0f, 267.0f};
    std::array<uint8_t, 181u> couplingSagQ8_{};
    std::array<std::array<uint8_t, 181u>, kNumOscillators> couplingRampQ8_{};
    std::array<std::array<uint8_t, 180u>, kNumOscillators> couplingWrapPacked_{};
    std::array<uint8_t, 180u> couplingSyncMask_{};

    void updatePressureMeter(float dtClamped, const HudState& rawHud, bool cfgActive) noexcept;

    // GLOBAL CONFIG > Key Pressure hairline meter (live analog key pressure + brief peak hold).
    float pressLive01_ = 0.0f;
    float pressPeak01_ = 0.0f;
    float pressPeakHoldSec_ = 0.0f;
    float pressMeterPhase_ = 0.0f;
    float noPressureHoldSec_ = 0.0f; // > 0 while a non-pressure key was just touched on this row
    float hasFocusSmooth_ = 0.0f;
    float panPhase_ = 0.0f;
    float couplingPhase_ = 0.0f;
    float couplingSmooth_ = 0.0f;
    float couplingChaos_ = 0.0f;
    float couplingWinSamplesSmooth_ = 1800.0f;
    float configPhase_ = 0.0f;
    bool monoCollapse_ = false;
    bool primed_ = false;
    bool couplingPrimed_ = false;
};

} // namespace ruido
