#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr uint16_t kDisplayWidth = 240u;
inline constexpr uint16_t kDisplayHeight = 240u;
inline constexpr uint16_t kScanlineStripHeight = 16u;

// Portable 240x16 RGB565 scanline strip monochrome 4-voice caustic ribbon oscilloscope +
// soft-feathered anti-aliased sans-serif HUD compositor.
// Renders 14 coupled per-oscillator + filter-capacitor strands with a 4096-sample super-period
// similarity trigger, continuous phosphor persistence, a 60x60 quarter-res separable bloom grid,
// and 4x4 ordered Bayer RGB565 dithering (zero 240x240 framebuffer), matching Core 0 SPI DMA
// ping-pong strips pixel-for-pixel.
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
    static constexpr size_t kHistorySize = 4096u;
    static constexpr size_t kHistoryMask = kHistorySize - 1u;
    static constexpr size_t kTracePoints = kDisplayWidth + 1u; // 241 segment endpoints
    static constexpr size_t kNumStrands = 14u;                 // 4 voices x 3 + 2 stereo ribs
    static constexpr size_t kTileCols = 8u;
    static constexpr size_t kNumTiles = kDisplayWidth / kTileCols; // 30 horizontal 8-px tiles
    static constexpr size_t kTrigSigTaps = 12u;
    static constexpr size_t kTrigSigPoints = kTrigSigTaps * kNumOscillators;
    static constexpr size_t kBloomWidth = kDisplayWidth / 4u;   // 60
    static constexpr size_t kBloomHeight = kDisplayHeight / 4u; // 60
    static constexpr size_t kFilterPoints = 221u;               // x = 10..230 (220 1-px segments)

    struct CompactScopeSample {
        int16_t leftQ12 = 0;
        int16_t rightQ12 = 0;
        // Upper 15 bits = signed Q2.12 voice sample, bit 0 = delayed rising-edge flag
        std::array<int16_t, kNumOscillators> voiceEdgeQ12{0, 0, 0, 0};
    };

    void rebuildTilesAndBloom() noexcept;
    void buildWaveformTrace(float dtSec, const EngineControlState& control) noexcept;
    void updateFilterGraph(float dtClamped, bool snapImmediate,
                           const std::array<ModulatedFilterParams, kNumOscillators>& mods,
                           bool unified) noexcept;
    void updateLfoGraph(float dtClamped, bool snapImmediate, bool showAllVoices) noexcept;
    void updateHudGauges(float dtClamped, bool snapImmediate) noexcept;
    void compositeHudStrip(uint16_t yStart, uint16_t stripHeight, uint8_t* lumBuf,
                           uint16_t* linScratch, uint8_t hudAlpha8) const noexcept;

    // 4096-sample rolling audio + 4-voice scope history (~93 ms @ 44.1 kHz, locks down to 20 Hz)
    std::array<CompactScopeSample, kHistorySize> history_{};
    size_t historyWrite_ = 0u;
    size_t historyCount_ = 0u;
    bool bassZoom2x_ = true;
    bool scopeValid_ = false;
    bool trigSigValid_ = false;
    bool phosphorPrimed_ = false;
    float smoothStride_ = 2.0f;
    float timeSinceScopePushSec_ = 0.0f;
    float thermalPhase_ = 0.0f;
    float busPresenceSmooth_ = 0.0f;
    float soloFactorSmooth_ = 0.0f;
    float monoCollapseSmooth_ = 0.0f;
    std::array<float, kNumOscillators> voicePresenceSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kTrigSigPoints> prevTrigSig_{};

    // Continuous phosphor / galvanometer persistence states (replaces discrete frame-ring jumps)
    std::array<std::array<float, kTracePoints>, kNumStrands> strandSmoothY_{};
    std::array<std::array<int16_t, kTracePoints>, kNumOscillators> voiceTrailQ12_{};
    std::array<int16_t, kTracePoints> busTrailLQ12_{};
    std::array<int16_t, kTracePoints> busTrailRQ12_{};

    // Prepared open edge-to-edge screen-space strands (Q8.6 Y coordinates) + 8-px tile Y-bounds
    std::array<std::array<int16_t, kTracePoints>, kNumStrands> strandYFixed_{};
    std::array<uint16_t, kNumStrands> strandWeightQ8_{};
    std::array<std::array<uint8_t, kNumTiles>, kNumStrands> strandTileMinY_{};
    std::array<std::array<uint8_t, kNumTiles>, kNumStrands> strandTileMaxY_{};

    // Quarter-resolution (60x60) pre-blurred caustic bloom grid (scaled linear photon units)
    std::array<uint16_t, kBloomWidth * kBloomHeight> bloomGrid_{};

    // Concept A3 1/3-screen (80 px: y = 102..182, x = 10..230) dual-resonance MS-20 filter graph:
    // - filterCurvesFixed_[0..3] = slewed Voice 1-4 spines (identical when unified)
    // - filterCompCurvesFixed_[0..1] = slewed MS-20 companion harmonic contours
    std::array<std::array<int16_t, kFilterPoints>, kNumOscillators> filterCurvesFixed_{};
    std::array<std::array<int16_t, kFilterPoints>, 2> filterCompCurvesFixed_{};
    std::array<float, 2> filterCompWeights_{0.0f, 0.0f};
    float filterDivergenceSmooth_ = 0.0f;
    float filterHpBeadX_ = 10.0f;
    float filterHpBeadY_ = 132.0f;
    float filterHpBeadInt_ = 0.0f;
    float filterHpBeadRad_ = 2.0f;
    float filterLpBeadX_ = 230.0f;
    float filterLpBeadY_ = 132.0f;
    float filterLpBeadInt_ = 0.0f;
    float filterLpBeadRad_ = 2.0f;
    float filterGraphPhase_ = 0.0f;
    float filterBackdropDim_ = 0.0f;

    // Full-screen (x = 0..239 edge-to-edge, y = 26..188 centered at y = 107) per-voice LFO graph:
    // - lfoCurvesFixed_[0..3] = slewed 241-pt Voice 1-4 LFO waveforms (zero fake inner contours)
    std::array<std::array<int16_t, kTracePoints>, kNumOscillators> lfoCurvesFixed_{};
    float lfoDivergenceSmooth_ = 0.0f;
    float lfoGraphPhase_ = 0.0f;
    std::array<float, kNumOscillators> frameLfoOutput_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> frameLfoPhase_{0.0f, 0.25f, 0.50f, 0.75f};
    std::array<uint32_t, kNumOscillators> frameLfoDriftPacked_{0u, 0u, 0u, 0u};
    std::array<float, kNumOscillators> lfoBeadIntensity_{0.0f, 0.0f, 0.0f, 0.0f};

    // Latched & smoothly slewed HUD / gauge state (zero hard-cut mode/fade/bead transitions)
    EngineControlState frameControl_{};
    HudState frameHud_{};
    HudState prevRawHud_{};
    HudState releaseLatchHud_{};
    std::array<float, kNumOscillators> prevVoiceFreqHz_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobPosSignedSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobDimSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> knobFocusSmooth_{0.0f, 0.0f, 0.0f, 0.0f};
    float headerBarSmooth_ = 0.0f;
    float renderedHudOpacity_ = 0.0f;
    float smoothedPeakL_ = 0.0f;
    float smoothedPeakR_ = 0.0f;
    HudMode prevHeaderBarMode_ = HudMode::None;
    bool releaseLatchActive_ = false;
    bool hudGaugesPrimed_ = false;
};

} // namespace ruido
