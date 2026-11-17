#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/BootIntroRender.h"
#include "ui/HudAmpGraph.h"
#include "ui/HudChrome.h"
#include "ui/HudGlobalViews.h"
#include "ui/HudGraphs.h"
#include "ui/HudHeader.h"
#include "ui/HudKeybedView.h"
#include "ui/HudSceneView.h"
#include "ui/ScopeTraceEngine.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Portable 240x16 RGB565 scanline strip monochrome 4-voice caustic ribbon oscilloscope +
// soft-feathered anti-aliased sans-serif HUD compositor.
class ScopeHudRenderer {
  public:
    ScopeHudRenderer() noexcept;

    void reset() noexcept;

    // Arm the cold-boot CRT `RUIDO` power-on intro sequence.
    void armBootIntro() noexcept { bootIntro_.arm(); }

    // Smoothly fast-dissolve (or skip before frame 1) the boot intro on hardware user input.
    void notifyUserInput() noexcept { bootIntro_.notifyUserInput(); }

    bool isBootIntroActive() const noexcept { return bootIntro_.isActive(); }

    // Drain available audio scope samples from Core 1's lock-free SPSC ring buffer.
    void ingestScopeRing(ScopeRingBuffer& ring) noexcept;

    // Directly push a batch of ScopeFrames (used when simulator bridges frames over IPC).
    void pushScopeSamples(const ScopeFrame* frames, size_t count) noexcept;

    // Prepare coupled 4-voice caustic strands, 60x60 separable bloom & HUD curves (~50-60 FPS).
    void prepareFrame(float dtSec, const EngineControlState& control, const HudState& hud,
                      const EngineTelemetry& telemetry,
                      void (*coProcYield)() noexcept = nullptr) noexcept;

    // Render `stripHeight` scanlines starting at `yStart` into `rgb565Out` (`240 * stripHeight`).
    // Pass `bigEndianRgb565 = true` on big-endian SPI displays (e.g., ST7789) for single-pass pack.
    void renderStrip(uint16_t yStart, uint16_t stripHeight, uint16_t* rgb565Out,
                     bool bigEndianRgb565 = false,
                     void (*coProcYield)() noexcept = nullptr) const noexcept;

    // Convenience helper to render all 15 strips (`15 x 16 = 240`) into a 240x240 RGB565 surface.
    void renderFullScreenStrips(uint16_t* fullRgb565Out) const noexcept {
        for (uint16_t y = 0u; y < kDisplayHeight;
             y = static_cast<uint16_t>(y + kScanlineStripHeight)) {
            renderStrip(y, kScanlineStripHeight,
                        fullRgb565Out + static_cast<size_t>(y) * kDisplayWidth);
        }
    }

  private:
    uint32_t computeFrameBgKeepQ8() const noexcept;
    void updateTraceForFrame(float dtSec, const EngineControlState& control, const HudState& hud,
                             void (*coProcYield)() noexcept = nullptr) noexcept;
    int computeScanlineSyncShift(int py) const noexcept;
    void updateHudGauges(float dtClamped, bool snapImmediate) noexcept;
    void compositeHudStrip(uint16_t yStart, uint16_t stripHeight, uint8_t* lumBuf,
                           uint16_t* linScratch, uint8_t hudAlpha8) const noexcept;

    static constexpr uint16_t kSubStripRows = 8u;

    BootIntroRenderer bootIntro_{};
    ScopeTraceEngine traceEngine_{};
    HudGraphs graphs_{};
    HudAmpGraph ampGraph_{};
    HudGlobalViews globalViews_{};
    HudKeybedView keybedView_{};
    HudSceneView sceneView_{};

    mutable std::array<uint16_t, kDisplayWidth * kSubStripRows> stripLinScratch_{};
    mutable std::array<uint8_t, kDisplayWidth * kSubStripRows> stripLumScratch_{};

    EngineControlState frameControl_{};
    HudState frameHud_{};
    HudState prevRawHud_{};
    HudState releaseLatchHud_{};
    PreparedHudHeader preparedHeader_{};
    PreparedHudChrome preparedChrome_{};
    LfoSignalArray frameLfoOutput_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LfoSignalArray frameLfoPhase_{0.0f, 0.25f, 0.50f, 0.75f, 0.50f, 0.75f, 0.125f, 0.375f};
    LfoDriftArray frameLfoDriftPacked_{0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    std::array<float, kNumOscillators> frameVoicePeak_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> frameVoiceDrift_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> prevVoiceFreqHz_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobPosSignedSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobDimSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobFocusSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumSnapshots> sceneDotsSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    float headerBarSmooth_ = 0.0f;
    float fineOverlayPhase_ = 0.0f;
    float coarseOverlayPhase_ = 0.0f;
    float sceneDialPhaseSmooth_ = 0.0f;
    float renderedOpacityLevel_ = 0.0f;
    float smoothedPeakL_ = 0.0f;
    float smoothedPeakR_ = 0.0f;
    float frameRhythmPhase_ = 0.0f;
    float syncRecoilEnv_ = 0.0f;
    float syncRecoilPhase_ = 0.0f;
    HudMode prevHeaderBarMode_ = HudMode::None;
    KnobBankMode prevKnobBank_ = KnobBankMode::Frequency;
    uint32_t frameBgKeepQ8_ = 256u;
    uint8_t frameStutterGateMask_ = 0x0Fu;
    uint8_t lastPhantomSeq_ = 0u;
    uint8_t phantomPin_ = 0u;
    bool wasShowingSceneDots_ = false;
    bool releaseLatchActive_ = false;
    bool hudGaugesPrimed_ = false;
};

} // namespace ruido
