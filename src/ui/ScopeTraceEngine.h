#pragma once

#include "core/Parameters.h"
#include "ui/MonoCanvas.h"
#include "ui/ScopeTraceMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// 4-voice coupled caustic ribbon oscilloscope engine with a 4096-sample super-period similarity
// trigger, continuous phosphor persistence, and a 60x60 quarter-res separable bloom grid.
class ScopeTraceEngine {
  public:
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

    void reset() noexcept;

    void advanceWatchdog(float dtClamped, bool scopeFrozen) noexcept {
        if (!scopeFrozen) {
            timeSinceScopePushSec_ += dtClamped;
            if (timeSinceScopePushSec_ > 10.0f) {
                timeSinceScopePushSec_ = 10.0f;
            }
        }
    }

    bool isStreamStalled(bool scopeFrozen) const noexcept {
        return (!scopeFrozen) && (timeSinceScopePushSec_ > 0.065f);
    }

    void ingestScopeRing(ScopeRingBuffer& ring, bool scopeFrozen) noexcept;
    void pushScopeSamples(const ScopeFrame* frames, size_t count, bool scopeFrozen) noexcept;

    void buildWaveformTrace(float dtSec, const EngineControlState& control, bool scopeFrozen,
                            uint8_t stutterGateMask = 0x0Fu) noexcept;

    // Rasterize <= 8-row strip chunk of 14 caustic strands + bilinear 60x60 bloom + tonemap.
    void renderBackgroundChunk(int y0, uint16_t chunkH, uint32_t bgKeepQ8, uint16_t* linBuf,
                               uint8_t* lumBuf) const noexcept;

    const OverdriveHairTable& overdriveHair() const noexcept { return overdriveHair_; }

  private:
    void rebuildTilesAndBloom() noexcept;

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

    // Continuous phosphor / galvanometer persistence states
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

    // 48-point centered bipolar live transient window per voice for HudAmpGraph 3A overdrive cap
    OverdriveHairTable overdriveHair_{};
};

} // namespace ruido
