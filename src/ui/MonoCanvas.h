#pragma once

#include "core/DspMath.h"
#include "ui/FontSansAA.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr uint16_t kDisplayWidth = 240u;
inline constexpr uint16_t kDisplayHeight = 240u;
inline constexpr uint16_t kScanlineStripHeight = 16u;

// Compile-time elementary math helpers for zero-SRAM XIP Flash optical lookup tables
constexpr float cxExpPos(float u) noexcept {
    const float s = u * 0.125f;
    float r = 1.0f;
    for (int k = 10; k >= 1; --k) {
        r = 1.0f + (r * s) / static_cast<float>(k);
    }
    r = r * r;
    r = r * r;
    return r * r;
}

constexpr float cxExp(float x) noexcept {
    return (x >= 0.0f) ? cxExpPos(x) : (1.0f / cxExpPos(-x));
}

constexpr float cxLnUnit(float x) noexcept {
    float m = x;
    int p2 = 0;
    while (m < 0.5f && p2 > -24) {
        m *= 2.0f;
        --p2;
    }
    const float y = (m - 1.0f) / (m + 1.0f);
    const float y2 = y * y;
    const float series =
        2.0f * y *
        (1.0f +
         y2 * ((1.0f / 3.0f) + y2 * ((1.0f / 5.0f) + y2 * ((1.0f / 7.0f) + y2 * (1.0f / 9.0f)))));
    return series + static_cast<float>(p2) * 0.69314718f;
}

constexpr std::array<uint16_t, 64> makeGaussLut() noexcept {
    std::array<uint16_t, 64> lut{};
    for (size_t i = 0u; i < 64u; ++i) {
        const float d2 = (static_cast<float>(i) * (1.0f / 64.0f)) * 4.0f;
        lut[i] = static_cast<uint16_t>(256.0f * cxExp(-d2 * 1.85f) + 0.5f);
    }
    return lut;
}

// Soft-knee Reinhard + 0.82 CRT gamma LUT mapping linear photon energy [0..4095] (>>2) -> [0..255]
constexpr std::array<uint8_t, 1024> makeTonemapLut() noexcept {
    std::array<uint8_t, 1024> lut{};
    lut[0] = 0u;
    for (size_t i = 1u; i < 1024u; ++i) {
        const float rawE = static_cast<float>(i * 4u) * (1.0f / 540.0f);
        // Smooth optical black-floor toe keeps distant ultra-faint bloom tails pure #000000
        const float e = (rawE > 0.012f) ? ((rawE - 0.012f) * (1.0f / 0.988f)) : 0.0f;
        if (e <= 0.0f) {
            lut[i] = 0u;
            continue;
        }
        const float mapped = clampf((e * (1.0f + 0.18f * e)) / (1.0f + e), 0.0f, 1.0f);
        const float gamma = (mapped <= 1.0e-5f) ? 0.0f : cxExp(0.82f * cxLnUnit(mapped));
        const float out = clampf(gamma * 255.0f + 0.5f, 0.0f, 255.0f);
        lut[i] = static_cast<uint8_t>(out);
    }
    return lut;
}

inline constexpr std::array<uint16_t, 64> kGaussLut = makeGaussLut();
inline constexpr std::array<uint8_t, 1024> kTonemapLut = makeTonemapLut();

inline float slewScalar(float cur, float tgt, float alpha, float snapEps = 1.0e-4f) noexcept {
    const float diff = tgt - cur;
    if (absf(diff) <= snapEps) {
        return tgt;
    }
    const float next = cur + diff * alpha;
    return (absf(tgt - next) <= snapEps) ? tgt : next;
}

inline int16_t slewQ8_6(int16_t cur, int16_t tgt, float alpha, bool snapImmediate) noexcept {
    if (snapImmediate || cur == tgt) {
        return tgt;
    }
    const int32_t diff = static_cast<int32_t>(tgt) - static_cast<int32_t>(cur);
    int32_t step = static_cast<int32_t>(static_cast<float>(diff) * alpha);
    if (step == 0) {
        step = (diff > 0) ? 1 : -1;
    }
    return static_cast<int16_t>(static_cast<int32_t>(cur) + step);
}

// Pack 8-bit monochrome luminance [0..255] into strictly neutral RGB565 (R5==B5, G6==2*R5)
// with 4x4 ordered Bayer dithering (~496 effective shades) so subtle 25% caustic bloom halos
// and soft HUD vignettes melt into #000000 with zero 5-bit contour banding or tint.
inline uint16_t packNeutralGrayRgb565(uint8_t lum, int x, int y) noexcept {
    if (lum <= 1u) {
        return 0x0000u;
    }
    if (lum >= 253u) {
        return 0xFFFFu;
    }
    constexpr uint8_t kBayer4x4[16] = {
        0u, 8u, 2u, 10u, 12u, 4u, 14u, 6u, 3u, 11u, 1u, 9u, 15u, 7u, 13u, 5u,
    };
    const uint32_t dither = kBayer4x4[((y & 3) << 2) | (x & 3)];
    uint32_t g5 = (static_cast<uint32_t>(lum) * 31u + dither * 16u + 8u) / 255u;
    if (g5 > 31u) {
        g5 = 31u;
    }
    const uint32_t g6 = (g5 << 1u) | (g5 >> 4u);
    return static_cast<uint16_t>((g5 << 11u) | (g6 << 5u) | g5);
}

inline uint8_t blendLum8(uint8_t bg, uint8_t fg, uint8_t alpha8) noexcept {
    if (alpha8 == 0u) {
        return bg;
    }
    if (alpha8 == 255u) {
        return fg;
    }
    const uint32_t a = static_cast<uint32_t>(alpha8);
    const uint32_t inv = 255u - a;
    return static_cast<uint8_t>((inv * bg + a * fg + 127u) / 255u);
}

struct MonoStripCanvas {
    uint8_t* lum;
    int y0;
    int y1;
    uint8_t masterAlpha8;

    // Sub-pixel incandescent focal bead with soft Gaussian halo
    void drawOpticalBead(float cx, float cy, float intensity, float radius) const noexcept {
        const float effIntensity = intensity * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
        if (effIntensity <= 0.004f || radius <= 0.1f) {
            return;
        }
        const float reach = radius + 2.5f;
        int pxStart = static_cast<int>(cx - reach);
        int pxEnd = static_cast<int>(cx + reach) + 1;
        if (pxStart < 0) {
            pxStart = 0;
        }
        if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
            pxEnd = static_cast<int>(kDisplayWidth) - 1;
        }
        int pyStart = static_cast<int>(cy - reach);
        int pyEnd = static_cast<int>(cy + reach) + 1;
        if (pyStart < y0) {
            pyStart = y0;
        }
        if (pyEnd > y1 - 1) {
            pyEnd = y1 - 1;
        }
        if (pxStart > pxEnd || pyStart > pyEnd) {
            return;
        }

        const float invR2 = 1.0f / (radius * radius);
        for (int py = pyStart; py <= pyEnd; ++py) {
            const float dy = static_cast<float>(py) - cy;
            const float dy2 = dy * dy;
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (int px = pxStart; px <= pxEnd; ++px) {
                const float dx = static_cast<float>(px) - cx;
                const float r2 = dx * dx + dy2;
                const float core = fastExpNeg(r2 * 1.45f);
                const float halo = 0.44f * fastExpNeg(r2 * invR2 * 1.55f);
                const float a = clampf((core + halo) * effIntensity, 0.0f, 1.0f);
                const uint32_t a8 = static_cast<uint32_t>(a * 255.0f + 0.5f);
                if (a8 >= 2u) {
                    row[px] = blendLum8(row[px], 255u, static_cast<uint8_t>(a8));
                }
            }
        }
    }

    // Crisp optical phase bead with pure-white pinpoint core and tight phosphor halo
    // (proportioned to match the MS-20 filter cutoff beads on the 240x240 display).
    void drawLfoPhaseBeacon(float cx, float cy, float intensity, float depth01) const noexcept {
        const float effIntensity = intensity * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
        if (effIntensity <= 0.004f) {
            return;
        }
        const float d = clampf(depth01, 0.0f, 1.0f);
        const float coreR = 0.95f + 0.15f * d;
        const float coronaR = 1.85f + 0.40f * d;
        const float bloomR = 3.10f + 0.65f * d;
        const float reach = bloomR + 1.6f;
        const float reach2 = reach * reach;
        const float fadeStart2 = (bloomR * 0.78f) * (bloomR * 0.78f);
        const float invFadeSpan2 = 1.0f / (reach2 - fadeStart2 + 1.0e-4f);

        int pxStart = static_cast<int>(cx - reach);
        int pxEnd = static_cast<int>(cx + reach) + 1;
        if (pxStart < 0) {
            pxStart = 0;
        }
        if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
            pxEnd = static_cast<int>(kDisplayWidth) - 1;
        }
        int pyStart = static_cast<int>(cy - reach);
        int pyEnd = static_cast<int>(cy + reach) + 1;
        if (pyStart < y0) {
            pyStart = y0;
        }
        if (pyEnd > y1 - 1) {
            pyEnd = y1 - 1;
        }
        if (pxStart > pxEnd || pyStart > pyEnd) {
            return;
        }

        const float invCore2 = 1.0f / (coreR * coreR);
        const float invCorona2 = 1.0f / (coronaR * coronaR);
        const float invBloom2 = 1.0f / (bloomR * bloomR);

        for (int py = pyStart; py <= pyEnd; ++py) {
            const float dy = static_cast<float>(py) - cy;
            const float dy2 = dy * dy;
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (int px = pxStart; px <= pxEnd; ++px) {
                const float dx = static_cast<float>(px) - cx;
                const float r2 = dx * dx + dy2;
                if (r2 >= reach2) {
                    continue;
                }
                const float outerWin =
                    (r2 <= fadeStart2) ? 1.0f
                                       : (1.0f - smoothstep01((r2 - fadeStart2) * invFadeSpan2));
                const float core = clampf(1.28f * fastExpNeg(r2 * invCore2 * 1.40f), 0.0f, 1.0f);
                const float corona = 0.40f * fastExpNeg(r2 * invCorona2 * 1.35f);
                const float bloom = 0.12f * fastExpNeg(r2 * invBloom2 * 1.40f);
                const float glow =
                    clampf((core + corona + bloom) * outerWin * effIntensity, 0.0f, 1.0f);
                const uint32_t add8 = static_cast<uint32_t>(glow * 255.0f + 0.5f);
                if (add8 > 0u) {
                    const uint32_t bg = row[px];
                    row[px] = static_cast<uint8_t>(255u - (((255u - bg) * (255u - add8)) >> 8u));
                }
            }
        }
    }

    // Sub-pixel horizontal optical filament with optional feathered endpoints
    void drawHorizontalFilament(float x0, float x1, float yCenter, float peakLum,
                                float halfSigma = 0.65f, bool taperEnds = true) const noexcept {
        const float effPeak = peakLum * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
        if (effPeak <= 0.5f) {
            return;
        }
        const float lo = (x0 < x1) ? x0 : x1;
        const float hi = (x0 > x1) ? x0 : x1;
        int pxStart = static_cast<int>(lo) - 2;
        int pxEnd = static_cast<int>(hi) + 2;
        if (pxStart < 0) {
            pxStart = 0;
        }
        if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
            pxEnd = static_cast<int>(kDisplayWidth) - 1;
        }
        int pyStart = static_cast<int>(yCenter) - 3;
        int pyEnd = static_cast<int>(yCenter) + 3;
        if (pyStart < y0) {
            pyStart = y0;
        }
        if (pyEnd > y1 - 1) {
            pyEnd = y1 - 1;
        }
        if (pxStart > pxEnd || pyStart > pyEnd) {
            return;
        }

        const float span = hi - lo;
        const float inv2Sig2 = 1.0f / (2.0f * halfSigma * halfSigma);
        for (int py = pyStart; py <= pyEnd; ++py) {
            const float dy = static_cast<float>(py) - yCenter;
            const float wy = fastExpNeg(dy * dy * inv2Sig2);
            if (wy < 0.01f) {
                continue;
            }
            const float rowPeak = effPeak * wy;
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (int px = pxStart; px <= pxEnd; ++px) {
                const float pxF = static_cast<float>(px);
                float wx = 1.0f;
                if (pxF < lo) {
                    const float d = lo - pxF;
                    wx = fastExpNeg(d * d * 1.8f);
                } else if (pxF > hi) {
                    const float d = pxF - hi;
                    wx = fastExpNeg(d * d * 1.8f);
                } else if (taperEnds && span > 8.0f) {
                    const float dL = pxF - lo;
                    const float dR = hi - pxF;
                    const float edgeDist = (dL < dR) ? dL : dR;
                    if (edgeDist < 3.0f) {
                        wx = 0.35f + 0.65f * (edgeDist * (1.0f / 3.0f));
                    }
                }
                const uint32_t a8 = static_cast<uint32_t>(rowPeak * wx + 0.5f);
                if (a8 > 0u) {
                    row[px] =
                        blendLum8(row[px], 255u, static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
                }
            }
        }
    }

    // Sub-pixel vertical optical slit / stem filament
    void drawVerticalSlit(float xCenter, float yTop, float yBot, float peakLum) const noexcept {
        const float effPeak = peakLum * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
        if (effPeak <= 0.5f) {
            return;
        }
        int pxStart = static_cast<int>(xCenter) - 2;
        int pxEnd = static_cast<int>(xCenter) + 2;
        if (pxStart < 0) {
            pxStart = 0;
        }
        if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
            pxEnd = static_cast<int>(kDisplayWidth) - 1;
        }
        int pyStart = static_cast<int>(yTop);
        int pyEnd = static_cast<int>(yBot + 0.999f);
        if (pyStart < y0) {
            pyStart = y0;
        }
        if (pyEnd > y1 - 1) {
            pyEnd = y1 - 1;
        }
        if (pxStart > pxEnd || pyStart > pyEnd) {
            return;
        }
        for (int py = pyStart; py <= pyEnd; ++py) {
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (int px = pxStart; px <= pxEnd; ++px) {
                const float dx = static_cast<float>(px) - xCenter;
                const float wx = fastExpNeg(dx * dx * 2.1f);
                const uint32_t a8 = static_cast<uint32_t>(effPeak * wx + 0.5f);
                if (a8 > 0u) {
                    row[px] =
                        blendLum8(row[px], 255u, static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
                }
            }
        }
    }

    // Soft feathered black scrim (zero hard borders or divider lines)
    void applyVerticalScrim(int yTop, int yBot, float peakDim, bool topDown) const noexcept {
        const int ys = (yTop < y0) ? y0 : yTop;
        const int ye = (yBot > y1) ? y1 : yBot;
        if (ys >= ye || yBot <= yTop) {
            return;
        }
        const float invSpan = 1.0f / static_cast<float>(yBot - yTop);
        const float masterF = static_cast<float>(masterAlpha8) * (1.0f / 255.0f);
        for (int py = ys; py < ye; ++py) {
            const float u = (static_cast<float>(py - yTop) + 0.5f) * invSpan;
            const float profile = topDown ? (1.0f - smoothstep01(u)) : smoothstep01(u);
            const float keep = 1.0f - clampf(peakDim * profile * masterF, 0.0f, 0.94f);
            const uint32_t keepQ8 = static_cast<uint32_t>(keep * 256.0f + 0.5f);
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (size_t x = 0u; x < kDisplayWidth; ++x) {
                row[x] = static_cast<uint8_t>((static_cast<uint32_t>(row[x]) * keepQ8) >> 8u);
            }
        }
    }

    // Render 4-bit anti-aliased Helvetica Neue sans-serif text
    void drawSans(const SansFont& font, int tx, int ty, const char* str, uint8_t gray,
                  int tracking = 0) const noexcept {
        if (str == nullptr || masterAlpha8 == 0u) {
            return;
        }
        if (ty + static_cast<int>(font.lineHeight) <= y0 || ty >= y1) {
            return;
        }
        int penX = tx;
        for (size_t i = 0u; str[i] != '\0'; ++i) {
            bool hasAcute = false;
            const unsigned char baseUc = decodeSansGlyphChar(str, i, hasAcute);
            const size_t idx = static_cast<size_t>(baseUc - 32u);
            const GlyphMetrics& gm = font.metrics[idx];

            const int gx0 = penX + static_cast<int>(gm.xOffset);
            const int gy0 = ty + static_cast<int>(gm.yOffset);
            const int gw = static_cast<int>(gm.width);
            const int gh = static_cast<int>(gm.height);

            if (gw > 0 && gh > 0 && gy0 + gh > y0 && gy0 < y1 && gx0 + gw > 0 &&
                gx0 < static_cast<int>(kDisplayWidth)) {
                const int ryStart = (gy0 < y0) ? (y0 - gy0) : 0;
                const int ryEnd = (gy0 + gh > y1) ? (y1 - gy0) : gh;

                for (int ry = ryStart; ry < ryEnd; ++ry) {
                    const int py = gy0 + ry;
                    uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
                    for (int rx = 0; rx < gw; ++rx) {
                        const int px = gx0 + rx;
                        if (px < 0 || px >= static_cast<int>(kDisplayWidth)) {
                            continue;
                        }
                        const size_t nibIdx = static_cast<size_t>(ry * gw + rx);
                        const uint8_t packed = font.bitmaps[gm.dataOffset + (nibIdx >> 1u)];
                        const uint8_t nib =
                            ((nibIdx & 1u) == 0u) ? (packed >> 4u) : (packed & 0x0Fu);
                        if (nib != 0u) {
                            const uint32_t cov8 = static_cast<uint32_t>(nib) * 17u; // 0..255
                            const auto a8 = static_cast<uint8_t>(
                                (cov8 * static_cast<uint32_t>(masterAlpha8) + 127u) / 255u);
                            row[px] = blendLum8(row[px], gray, a8);
                        }
                    }
                }
            }
            if (hasAcute) {
                constexpr uint8_t kAcute3x3[3][3] = {
                    {0u, 10u, 15u},
                    {4u, 15u, 6u},
                    {13u, 5u, 0u},
                };
                const int ax0 = penX + (static_cast<int>(gm.advance) / 2) - 1;
                const int ayRaw = gy0 - 4;
                const int ay0 = (ayRaw < ty) ? ty : ayRaw;
                if (ay0 + 3 > y0 && ay0 < y1) {
                    for (int ary = 0; ary < 3; ++ary) {
                        const int py = ay0 + ary;
                        if (py < y0 || py >= y1) {
                            continue;
                        }
                        uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
                        for (int arx = 0; arx < 3; ++arx) {
                            const int px = ax0 + arx;
                            const uint8_t nib = kAcute3x3[ary][arx];
                            if (nib != 0u && px >= 0 && px < static_cast<int>(kDisplayWidth)) {
                                const uint32_t cov8 = static_cast<uint32_t>(nib) * 17u;
                                const auto a8 = static_cast<uint8_t>(
                                    (cov8 * static_cast<uint32_t>(masterAlpha8) + 127u) / 255u);
                                row[px] = blendLum8(row[px], gray, a8);
                            }
                        }
                    }
                }
            }
            penX += static_cast<int>(gm.advance) + tracking;
        }
    }

    void drawSansCentered(const SansFont& font, int centerX, int ty, const char* str, uint8_t gray,
                          int tracking = 0) const noexcept {
        const int w = measureSansText(font, str, tracking);
        drawSans(font, centerX - (w / 2), ty, str, gray, tracking);
    }

    void drawSansRight(const SansFont& font, int rightX, int ty, const char* str, uint8_t gray,
                       int tracking = 0) const noexcept {
        const int w = measureSansText(font, str, tracking);
        drawSans(font, rightX - w, ty, str, gray, tracking);
    }

    // Two-tier tertiary monochrome indicator slot (HOME / FREQUENCY per-osc FX + LFO A/B row):
    //  - Inactive (!active): crisp 1x1 integer pixel dot at 16% (matching passive track, zero AA)
    //  - Active (active): calm tertiary circle (r = 1.50) capped strictly at 21%-37% (well below
    //    60% secondary column labels and 88%-100% primary readouts)
    void drawModulatorSlot(int cx, int cy, bool active, float sync01,
                           float colDim01) const noexcept {
        if (masterAlpha8 == 0u || cx < 0 || cx >= static_cast<int>(kDisplayWidth)) {
            return;
        }
        const float masterF = static_cast<float>(masterAlpha8) * (1.0f / 255.0f);
        const float dim = clampf(colDim01, 0.0f, 1.0f);
        if (!active) {
            if (cy < y0 || cy >= y1) {
                return;
            }
            const float phAlpha = 0.16f * (1.0f - 0.35f * dim) * masterF;
            const auto a8 = static_cast<uint8_t>(phAlpha * 255.0f + 0.5f);
            if (a8 > 0u) {
                uint8_t* row = lum + static_cast<size_t>(cy - y0) * kDisplayWidth;
                row[cx] = blendLum8(row[cx], 255u, a8);
            }
            return;
        }

        constexpr int kReach = 3;
        if (cy + kReach < y0 || cy - kReach >= y1) {
            return;
        }
        const int pxStart = (cx - kReach < 0) ? 0 : (cx - kReach);
        const int pxEnd = (cx + kReach >= static_cast<int>(kDisplayWidth))
                              ? (static_cast<int>(kDisplayWidth) - 1)
                              : (cx + kReach);
        const int pyStart = (cy - kReach < y0) ? y0 : (cy - kReach);
        const int pyEnd = (cy + kReach >= y1) ? (y1 - 1) : (cy + kReach);

        const float s = clampf(sync01, 0.0f, 1.0f);
        const float effSync = s * (1.0f - dim) + 0.35f * dim;
        const float dimKeep = 1.0f - 0.45f * dim;
        const float circleAlpha = (0.21f + 0.16f * effSync) * dimKeep;
        const float haloPeak = effSync * 0.08f * dimKeep;

        constexpr float kCircleR = 1.50f;
        constexpr float kHaloInnerR = 1.40f;
        constexpr float kHaloOuterR = 2.70f;
        constexpr float kInvHaloSpan = 1.0f / (kHaloOuterR - kHaloInnerR);

        for (int py = pyStart; py <= pyEnd; ++py) {
            const float dy = static_cast<float>(py - cy);
            const float dy2 = dy * dy;
            uint8_t* row = lum + static_cast<size_t>(py - y0) * kDisplayWidth;
            for (int px = pxStart; px <= pxEnd; ++px) {
                const float dx = static_cast<float>(px - cx);
                const float r2 = dx * dx + dy2;
                if (r2 >= kHaloOuterR * kHaloOuterR) {
                    continue;
                }
                const float r = (r2 > 1.0e-8f) ? (r2 * fastInvSqrt(r2)) : 0.0f;
                const float circleCov = 1.0f - smoothstep01(r - (kCircleR - 0.50f));
                const float haloT = clampf((r - kHaloInnerR) * kInvHaloSpan, 0.0f, 1.0f);
                const float haloA = haloPeak * (1.0f - haloT) * (1.0f - 0.65f * haloT);
                const float a = (circleCov * circleAlpha + (1.0f - circleCov) * haloA) * masterF;
                const auto a8 = static_cast<uint8_t>(clampf(a, 0.0f, 1.0f) * 255.0f + 0.5f);
                if (a8 > 0u) {
                    row[px] = blendLum8(row[px], 255u, a8);
                }
            }
        }
    }
};

// Splat a single anti-aliased 1-px horizontal Gaussian photon segment into `linScratch`
inline void splatPhotonSegment(int colX, float yA, float yB, float weight, float minDwell, int y0,
                               int y1, uint16_t* linScratch) noexcept {
    const float yLo = (yA < yB) ? yA : yB;
    const float yHi = (yA > yB) ? yA : yB;
    const int segTop = static_cast<int>(yLo) - 3;
    const int segBot = static_cast<int>(yHi) + 4;
    if (segBot <= y0 || segTop >= y1) {
        return;
    }
    const float dy = yB - yA;
    const float invL2 = 1.0f / (1.0f + dy * dy);
    const float dwell =
        (minDwell + (1.0f - minDwell) * fastInvSqrt(1.0f + 0.14f * dy * dy)) * weight;
    const float yMid = 0.5f * (yA + yB);
    const int pyS = (segTop < y0) ? y0 : segTop;
    const int pyE = (segBot > y1) ? y1 : segBot;
    for (int py = pyS; py < pyE; ++py) {
        const float vy = static_cast<float>(py) - yMid;
        const float segT = clampf(vy * dy * invL2, -0.5f, 0.5f);
        const float ey = vy - dy * segT;
        const float d2 = segT * segT + ey * ey;
        if (d2 < 3.95f) {
            const size_t lutIdx = static_cast<size_t>(d2 * 16.0f);
            const uint32_t add =
                static_cast<uint32_t>(static_cast<float>(kGaussLut[lutIdx]) * dwell);
            uint16_t& cell = linScratch[static_cast<size_t>(py - y0) * kDisplayWidth +
                                        static_cast<size_t>(colX)];
            const uint32_t next = static_cast<uint32_t>(cell) + add;
            cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
        }
    }
}

// Tonemap linear photon strip [xStart..xEnd) and screen-blend into 8-bit luminance buffer
inline void screenBlendLinearStrip(uint16_t stripHeight, size_t xStart, size_t xEnd,
                                   const uint16_t* linScratch, uint8_t* lumBuf) noexcept {
    for (uint16_t r = 0u; r < stripHeight; ++r) {
        const uint16_t* linRow = linScratch + static_cast<size_t>(r) * kDisplayWidth;
        uint8_t* lumRow = lumBuf + static_cast<size_t>(r) * kDisplayWidth;
        for (size_t x = xStart; x < xEnd; ++x) {
            const uint16_t e = linRow[x];
            if (e > 0u) {
                const uint32_t addLum = kTonemapLut[e >> 2u];
                const uint32_t bg = lumRow[x];
                lumRow[x] = static_cast<uint8_t>(255u - (((255u - bg) * (255u - addLum)) >> 8u));
            }
        }
    }
}

} // namespace ruido
