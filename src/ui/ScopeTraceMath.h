#pragma once

#include "core/DspMath.h"
#include "ui/MonoCanvas.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Linear CRT deflection projection: never shifts or compresses when HUD fades in, and lets
// large resonant peaks deflect naturally past the top/bottom bezel without flat-topping.
inline float sampleToScreenY(float sample) noexcept {
    const float y = 120.0f - sample * 112.0f;
    return clampf(y, -360.0f, 480.0f);
}

inline int16_t screenYToFixed(float y) noexcept {
    const float clamped = clampf(y, -360.0f, 480.0f);
    const float scaled = clamped * 64.0f;
    return static_cast<int16_t>((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

// 4-point Catmull-Rom cubic Hermite interpolation for silky sub-sample trigger lock
inline float catmullRom(float y0, float y1, float y2, float y3, float t) noexcept {
    const float a0 = -0.5f * y0 + 1.5f * y1 - 1.5f * y2 + 0.5f * y3;
    const float a1 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    const float a2 = -0.5f * y0 + 0.5f * y2;
    return ((a0 * t + a1) * t + a2) * t + y1;
}

inline int16_t packQ3_12(float s) noexcept {
    const float clamped = clampf(s, -3.8f, 3.8f);
    const float scaled = clamped * 4096.0f;
    return static_cast<int16_t>((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

inline float unpackQ3_12(int16_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 4096.0f);
}

inline int16_t packVoiceEdgeQ12(int16_t voiceQ13, bool risingEdge) noexcept {
    int32_t q12 = static_cast<int32_t>(voiceQ13) / 2;
    if (q12 < -16000) {
        q12 = -16000;
    } else if (q12 > 16000) {
        q12 = 16000;
    }
    const uint16_t u =
        static_cast<uint16_t>((static_cast<uint32_t>(q12 * 2) & 0xFFFEu) | (risingEdge ? 1u : 0u));
    return static_cast<int16_t>(u);
}

inline float unpackVoiceEdgeSample(int16_t packed) noexcept {
    const int32_t q12 = (packed >= 0) ? (static_cast<int32_t>(packed) >> 1)
                                      : ((static_cast<int32_t>(packed) - 1) / 2);
    return static_cast<float>(q12) * (1.0f / 4096.0f);
}

inline bool unpackVoiceEdgeFlag(int16_t packed) noexcept {
    return (static_cast<uint16_t>(packed) & 1u) != 0u;
}

// Zero-libm full-wave sin(2 * pi * phase01) via 5th-order minimax polynomial
inline float fastSinPhase01(float phase01) noexcept {
    float p = phase01 - static_cast<float>(static_cast<int>(phase01));
    if (p < 0.0f) {
        p += 1.0f;
    }
    float sign = 1.0f;
    if (p >= 0.5f) {
        p -= 0.5f;
        sign = -1.0f;
    }
    const float q = p * 2.0f;
    const float u = (q <= 0.5f) ? q : (1.0f - q);
    const float a = u * kPi;
    const float a2 = a * a;
    return sign * a * (1.0f - a2 * (0.16666667f - 0.0083143f * a2));
}

// Forward-backward zero-phase 1-pole smoother for silky inner harmonic curves
inline void smoothZeroPhase(const float* src, float* dst, size_t n, float alpha,
                            size_t passes) noexcept {
    for (size_t i = 0u; i < n; ++i) {
        dst[i] = src[i];
    }
    for (size_t p = 0u; p < passes; ++p) {
        float z = dst[0];
        for (size_t i = 1u; i < n; ++i) {
            z += alpha * (dst[i] - z);
            dst[i] = z;
        }
        z = dst[n - 1u];
        for (size_t i = n - 1u; i > 0u; --i) {
            const size_t idx = i - 1u;
            z += alpha * (dst[idx] - z);
            dst[idx] = z;
        }
    }
}

// Causal 2-pole dispersive lag + continuous fractional sample offset for ribbon separation
inline void dispersiveShell(const float* src, float* dst, size_t n, float alpha,
                            float shiftPts) noexcept {
    float z1 = src[0];
    float z2 = src[0];
    const float maxIdxF = static_cast<float>(n - 1u);
    for (size_t i = 0u; i < n; ++i) {
        const float pos = clampf(static_cast<float>(i) - shiftPts, 0.0f, maxIdxF);
        const size_t i0 = static_cast<size_t>(pos);
        const size_t i1 = (i0 + 1u < n) ? (i0 + 1u) : (n - 1u);
        const float frac = pos - static_cast<float>(i0);
        const float x = src[i0] + frac * (src[i1] - src[i0]);
        z1 += alpha * (x - z1);
        z2 += alpha * (z1 - z2);
        dst[i] = z2;
    }
}

// 2-pass in-place separable [1, 3, 4, 3, 1]/12 blur on 60x60 quarter-res bloom grid
template <size_t Width, size_t Height>
inline void blurSeparableBloomGrid(std::array<uint16_t, Width * Height>& grid) noexcept {
    std::array<uint16_t, Width> lineTmp{};
    for (size_t pass = 0u; pass < 2u; ++pass) {
        for (size_t by = 0u; by < Height; ++by) {
            uint16_t* row = grid.data() + by * Width;
            for (size_t bx = 0u; bx < Width; ++bx) {
                const size_t x0 = (bx >= 2u) ? (bx - 2u) : 0u;
                const size_t x1 = (bx >= 1u) ? (bx - 1u) : 0u;
                const size_t x3 = (bx + 1u < Width) ? (bx + 1u) : (Width - 1u);
                const size_t x4 = (bx + 2u < Width) ? (bx + 2u) : (Width - 1u);
                const uint32_t sum =
                    static_cast<uint32_t>(row[x0]) + 3u * static_cast<uint32_t>(row[x1]) +
                    4u * static_cast<uint32_t>(row[bx]) + 3u * static_cast<uint32_t>(row[x3]) +
                    static_cast<uint32_t>(row[x4]);
                lineTmp[bx] = static_cast<uint16_t>((sum + 6u) / 12u);
            }
            for (size_t bx = 0u; bx < Width; ++bx) {
                row[bx] = lineTmp[bx];
            }
        }

        for (size_t bx = 0u; bx < Width; ++bx) {
            for (size_t by = 0u; by < Height; ++by) {
                const size_t y0 = (by >= 2u) ? (by - 2u) : 0u;
                const size_t y1 = (by >= 1u) ? (by - 1u) : 0u;
                const size_t y3 = (by + 1u < Height) ? (by + 1u) : (Height - 1u);
                const size_t y4 = (by + 2u < Height) ? (by + 2u) : (Height - 1u);
                const uint32_t sum = static_cast<uint32_t>(grid[y0 * Width + bx]) +
                                     3u * static_cast<uint32_t>(grid[y1 * Width + bx]) +
                                     4u * static_cast<uint32_t>(grid[by * Width + bx]) +
                                     3u * static_cast<uint32_t>(grid[y3 * Width + bx]) +
                                     static_cast<uint32_t>(grid[y4 * Width + bx]);
                if (pass == 0u) {
                    lineTmp[by] = static_cast<uint16_t>((sum + 6u) / 12u);
                } else {
                    const uint32_t scaled = (sum * 990u + 32768u) >> 16u;
                    lineTmp[by] = static_cast<uint16_t>((scaled > 65535u) ? 65535u : scaled);
                }
            }
            for (size_t by = 0u; by < Height; ++by) {
                grid[by * Width + bx] = lineTmp[by];
            }
        }
    }
}

// Bilinear upsample of 60x60 bloom grid + linear photon sum + Reinhard/gamma tonemap for 1 row
template <size_t DisplayW, size_t BloomW, size_t BloomH>
inline void tonemapBloomScanline(int py, uint32_t bgKeepQ8, const uint16_t* bloomGrid,
                                 const uint16_t* linRow, uint8_t* lumRow) noexcept {
    const int numY8 = 2 * py - 3;
    size_t by0 = 0u;
    size_t by1 = 0u;
    uint32_t wy8 = 0u;
    if (numY8 > 0) {
        const uint32_t uy8 = static_cast<uint32_t>(numY8);
        by0 = uy8 >> 3u;
        if (by0 >= BloomH) {
            by0 = BloomH - 1u;
        }
        by1 = (by0 + 1u < BloomH) ? (by0 + 1u) : (BloomH - 1u);
        wy8 = uy8 & 7u;
    }
    const uint32_t invWy8 = 8u - wy8;
    const uint16_t* bRow0 = bloomGrid + by0 * BloomW;
    const uint16_t* bRow1 = bloomGrid + by1 * BloomW;

    for (size_t colX = 0u; colX < DisplayW; ++colX) {
        const int numX8 = 2 * static_cast<int>(colX) - 3;
        size_t bx0 = 0u;
        size_t bx1 = 0u;
        uint32_t wx8 = 0u;
        if (numX8 > 0) {
            const uint32_t ux8 = static_cast<uint32_t>(numX8);
            bx0 = ux8 >> 3u;
            if (bx0 >= BloomW) {
                bx0 = BloomW - 1u;
            }
            bx1 = (bx0 + 1u < BloomW) ? (bx0 + 1u) : (BloomW - 1u);
            wx8 = ux8 & 7u;
        }
        const uint32_t invWx8 = 8u - wx8;
        const uint32_t bTop =
            static_cast<uint32_t>(bRow0[bx0]) * invWx8 + static_cast<uint32_t>(bRow0[bx1]) * wx8;
        const uint32_t bBot =
            static_cast<uint32_t>(bRow1[bx0]) * invWx8 + static_cast<uint32_t>(bRow1[bx1]) * wx8;
        const uint32_t bloomE = (bTop * invWy8 + bBot * wy8 + 256u) >> 9u;

        uint32_t totalE = static_cast<uint32_t>(linRow[colX]) + bloomE;
        if (totalE > 4095u) {
            totalE = 4095u;
        }
        uint32_t lum = kTonemapLut[totalE >> 2u];
        if (bgKeepQ8 < 256u) {
            lum = (lum * bgKeepQ8 + 128u) >> 8u;
        }
        lumRow[colX] = static_cast<uint8_t>(lum);
    }
}

} // namespace ruido
