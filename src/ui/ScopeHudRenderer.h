#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/HudAmpGraph.h"
#include "ui/HudGlobalViews.h"
#include "ui/HudGraphs.h"
#include "ui/HudKeybedView.h"
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

    // Drain available audio scope samples from Core 1's lock-free SPSC ring buffer.
    void ingestScopeRing(ScopeRingBuffer& ring) noexcept;

    // Directly push a batch of ScopeFrames (used when simulator bridges frames over IPC).
    void pushScopeSamples(const ScopeFrame* frames, size_t count) noexcept;

    // Prepare coupled 4-voice caustic strands, 60x60 separable bloom & HUD curves (~50-60 FPS).
    void prepareFrame(float dtSec, const EngineControlState& control, const HudState& hud,
                      const EngineTelemetry& telemetry) noexcept;

    // Render `stripHeight` scanlines starting at `yStart` into `rgb565Out` (`240 * stripHeight`).
    void renderStrip(uint16_t yStart, uint16_t stripHeight, uint16_t* rgb565Out) const noexcept;

    // Convenience helper to render all 15 strips (`15 x 16 = 240`) into a 240x240 RGB565 surface.
    void renderFullScreenStrips(uint16_t* fullRgb565Out) const noexcept {
        for (uint16_t y = 0u; y < kDisplayHeight;
             y = static_cast<uint16_t>(y + kScanlineStripHeight)) {
            renderStrip(y, kScanlineStripHeight,
                        fullRgb565Out + static_cast<size_t>(y) * kDisplayWidth);
        }
    }

  private:
    void updateHudGauges(float dtClamped, bool snapImmediate) noexcept;
    void compositeHudStrip(uint16_t yStart, uint16_t stripHeight, uint8_t* lumBuf,
                           uint16_t* linScratch, uint8_t hudAlpha8) const noexcept;

    ScopeTraceEngine traceEngine_{};
    HudGraphs graphs_{};
    HudAmpGraph ampGraph_{};
    HudGlobalViews globalViews_{};
    HudKeybedView keybedView_{};

    EngineControlState frameControl_{};
    HudState frameHud_{};
    HudState prevRawHud_{};
    HudState releaseLatchHud_{};
    LfoSignalArray frameLfoOutput_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LfoSignalArray frameLfoPhase_{0.0f, 0.25f, 0.50f, 0.75f, 0.50f, 0.75f, 0.125f, 0.375f};
    LfoDriftArray frameLfoDriftPacked_{0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    std::array<float, kNumOscillators> frameVoicePeak_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> frameVoiceDrift_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> prevVoiceFreqHz_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobPosSignedSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobDimSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobFocusSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    float headerBarSmooth_ = 0.0f;
    float fineOverlayPhase_ = 0.0f;
    float coarseOverlayPhase_ = 0.0f;
    float renderedOpacityLevel_ = 0.0f;
    float smoothedPeakL_ = 0.0f;
    float smoothedPeakR_ = 0.0f;
    HudMode prevHeaderBarMode_ = HudMode::None;
    KnobBankMode prevKnobBank_ = KnobBankMode::Frequency;
    uint8_t frameStutterGateMask_ = 0x0Fu;
    bool releaseLatchActive_ = false;
    bool hudGaugesPrimed_ = false;
};

} // namespace ruido
