#pragma once

#include "ui/MonoCanvas.h"

#include <array>
#include <cstdint>

namespace ruido {

// Stateless-friendly CRT power-on boot intro renderer (`240x16` scanline strip compatible,
// `~13.4 KiB` XIP `.rodata`, `0 B` framebuffer SRAM).
class BootIntroRenderer {
  public:
    BootIntroRenderer() noexcept { reset(); }

    void reset() noexcept;

    // Arm the 2.00 s CRT power-on sequence on cold boot (`ruido_fm1_construct` / `wasm_ui_init`).
    void arm() noexcept;

    // Smoothly fast-dissolve (or skip if 0 frames rendered) when any hardware control is touched.
    void notifyUserInput() noexcept;

    bool isActive() const noexcept { return active_; }
    float scopeVisibility01() const noexcept { return active_ ? scopeAlpha_ : 1.0f; }

    void step(float dtSec, bool hudActive) noexcept;

    // Composite (or write directly when `blendOverScope == false`) `chunkH` scanlines at `y0`
    // into 8-bit neutral luminance buffer `lumBuf` (`chunkH * 240` bytes).
    void compositeStripChunk(int y0, uint16_t chunkH, uint8_t* lumBuf,
                             bool blendOverScope) const noexcept;

  private:
    void evaluateAtMs(float tMs) noexcept;

    float elapsedMs_ = 0.0f;
    float scopeAlpha_ = 1.0f;
    float beamRx_ = 1.12f;
    float beamRy_ = 1.9f;
    float beamOp_ = 0.0f;
    float beamCenterIdx_ = 119.5f;
    int32_t invCoreSxQ16_ = 65536;
    int32_t invCoreSyQ16_ = 65536;
    int32_t invBloomSyQ16_ = 65536;
    int32_t centerXQ16_ = 120 * 65536;
    uint16_t logoAlphaQ8_ = 0u;
    uint16_t renderedFrames_ = 0u;
    uint8_t logoQuantLum8_ = 255u;
    bool active_ = false;
    bool fastDismiss_ = false;
    bool beamVisible_ = false;
    bool logoVisible_ = false;
    std::array<uint8_t, kDisplayWidth> beamTightX_{};
    std::array<uint8_t, kDisplayWidth> beamMedX_{};
    std::array<uint8_t, kDisplayWidth> beamWideX_{};
    std::array<uint8_t, kDisplayWidth> beamCoreHalfHQ4_{};
};

} // namespace ruido
