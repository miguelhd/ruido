#include "ui/ScopeHudRenderer.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"

#include <cstring>

namespace ruido {

namespace {

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
        (1.0f + y2 * ((1.0f / 3.0f) +
                      y2 * ((1.0f / 5.0f) + y2 * ((1.0f / 7.0f) + y2 * (1.0f / 9.0f)))));
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

// Concept A3 1/3-screen (80 px: y = 102..182, x = 10..230) filter graph & backdrop dim constants
inline constexpr int kFilterXStart = 10;
inline constexpr size_t kFilterSweepPts = 221u;
inline constexpr float kFilterLogMin = 4.32192809f;  // log2(20 Hz)
inline constexpr float kFilterLogSpan = 9.96578428f; // log2(20000 / 20)
inline constexpr float kFilterYTop = 102.0f;
inline constexpr float kFilterYZeroDb = 132.0f;
inline constexpr float kFilterYBot = 182.0f;
inline constexpr float kFilterDbToPx = 1.68f;
inline constexpr float kFilterBgDimDepth = 0.72f; // 72% BG waveform dimming behind active graphs

// Full-screen (x = 0..239 edge-to-edge, y = 26..188) bipolar per-voice LFO visualization
inline constexpr float kLfoYTop = 26.0f;
inline constexpr float kLfoYZero = 107.0f;        // Midpoint between top header and bottom strip
inline constexpr float kLfoYBot = 188.0f;
inline constexpr float kLfoMaxAmpPx = 75.0f;      // +-75 px (150 px peak-to-peak) excursion
inline constexpr float kLfoLogMin = -5.64385619f; // log2(0.02 Hz)
inline constexpr float kLfoLogSpan = 10.55074679f; // log2(30.0 / 0.02)

constexpr std::array<float, kFilterSweepPts> makeFilterSweepFreqs() noexcept {
    std::array<float, kFilterSweepPts> freqs{};
    for (size_t i = 0u; i < kFilterSweepPts; ++i) {
        const float u = static_cast<float>(i) * (1.0f / static_cast<float>(kFilterSweepPts - 1u));
        const float log2Freq = kFilterLogMin + u * kFilterLogSpan;
        freqs[i] = cxExpPos(log2Freq * 0.69314718f);
    }
    return freqs;
}

inline constexpr std::array<float, kFilterSweepPts> kFilterSweepFreqs = makeFilterSweepFreqs();

inline float fastExpNeg(float x) noexcept {
    const float u = (x > 0.0f) ? x : 0.0f;
    return 1.0f / (1.0f + u + 0.48f * u * u + 0.235f * u * u * u);
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
        0u,  8u,  2u,  10u,
        12u, 4u,  14u, 6u,
        3u,  11u, 1u,  9u,
        15u, 7u,  13u, 5u,
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
        const float effIntensity =
            intensity * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
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
        const float effIntensity =
            intensity * (static_cast<float>(masterAlpha8) * (1.0f / 255.0f));
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
                    (r2 <= fadeStart2) ?
                        1.0f :
                        (1.0f - smoothstep01((r2 - fadeStart2) * invFadeSpan2));
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
                    row[px] = blendLum8(row[px], 255u,
                                        static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
                }
            }
        }
    }

    // Sub-pixel vertical optical slit for bipolar pan center ticks
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
                    row[px] = blendLum8(row[px], 255u,
                                        static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
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
            const unsigned char uc = static_cast<unsigned char>(str[i]);
            const size_t idx = (uc >= 32u && uc <= 126u) ? (uc - 32u) : 0u;
            const GlyphMetrics& gm = font.metrics[idx];

            const int gx0 = penX + static_cast<int>(gm.xOffset);
            const int gy0 = ty + static_cast<int>(gm.yOffset);
            const int gw = static_cast<int>(gm.width);
            const int gh = static_cast<int>(gm.height);

            if (gw > 0 && gh > 0 && gy0 + gh > y0 && gy0 < y1 &&
                gx0 + gw > 0 && gx0 < static_cast<int>(kDisplayWidth)) {
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
};

// Zero-allocation fixed-buffer technical string formatters
size_t appendUInt(char* dst, size_t pos, size_t cap, uint32_t val) noexcept {
    char rev[12]{};
    size_t n = 0u;
    do {
        rev[n++] = static_cast<char>('0' + (val % 10u));
        val /= 10u;
    } while (val > 0u && n < sizeof(rev));
    while (n > 0u && pos + 1u < cap) {
        dst[pos++] = rev[--n];
    }
    dst[pos] = '\0';
    return pos;
}

size_t appendStr(char* dst, size_t pos, size_t cap, const char* s) noexcept {
    for (size_t i = 0u; s[i] != '\0' && pos + 1u < cap; ++i) {
        dst[pos++] = s[i];
    }
    dst[pos] = '\0';
    return pos;
}

// Compact technical frequency formatter for 60px 4-knob columns
void formatCompactHz(char* buf, size_t cap, float hz) noexcept {
    size_t p = 0u;
    const float f = clampf(hz, 0.001f, 99999.0f);
    if (f < 1.0f) {
        const uint32_t centi = static_cast<uint32_t>(f * 100.0f + 0.5f);
        p = appendUInt(buf, p, cap, centi / 100u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, (centi / 10u) % 10u);
        p = appendUInt(buf, p, cap, centi % 10u);
        appendStr(buf, p, cap, " Hz");
    } else if (f < 100.0f) {
        const uint32_t deci = static_cast<uint32_t>(f * 10.0f + 0.5f);
        p = appendUInt(buf, p, cap, deci / 10u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, deci % 10u);
        appendStr(buf, p, cap, " Hz");
    } else if (f < 1000.0f) {
        p = appendUInt(buf, p, cap, static_cast<uint32_t>(f + 0.5f));
        appendStr(buf, p, cap, " Hz");
    } else if (f < 10000.0f) {
        const uint32_t centiK = static_cast<uint32_t>(f * 0.01f + 0.5f);
        p = appendUInt(buf, p, cap, centiK / 10u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, centiK % 10u);
        appendStr(buf, p, cap, " kHz");
    } else {
        const uint32_t intK = static_cast<uint32_t>(f * 0.001f + 0.5f);
        p = appendUInt(buf, p, cap, intK);
        appendStr(buf, p, cap, " kHz");
    }
}

void formatPercent(char* buf, size_t cap, float norm01) noexcept {
    const uint32_t pct = static_cast<uint32_t>(clampf(norm01, 0.0f, 1.0f) * 100.0f + 0.5f);
    size_t p = appendUInt(buf, 0u, cap, pct);
    appendStr(buf, p, cap, "%");
}

void formatPanValue(char* buf, size_t cap, float pan) noexcept {
    const int pInt = static_cast<int>(pan * 100.0f + ((pan >= 0.0f) ? 0.5f : -0.5f));
    if (pInt > -2 && pInt < 2) {
        appendStr(buf, 0u, cap, "C");
        return;
    }
    size_t p = 0u;
    if (pInt < 0) {
        p = appendStr(buf, p, cap, "L");
        appendUInt(buf, p, cap, static_cast<uint32_t>(-pInt));
    } else {
        p = appendStr(buf, p, cap, "R");
        appendUInt(buf, p, cap, static_cast<uint32_t>(pInt));
    }
}

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
    const int32_t q12 = (packed >= 0) ? (static_cast<int32_t>(packed) >> 1) :
                                        ((static_cast<int32_t>(packed) - 1) / 2);
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

struct DualMs20Coeffs {
    float invEffHp;
    float dampHp2;
    float invEffLp;
    float dampLp2;
    float hpAct;
    float lpAct;
    float clampedHp;
    float clampedLp;
    float rHp;
    float rLp;
};

inline DualMs20Coeffs makeDualMs20Coeffs(float hpHz, float hpRes, float lpHz,
                                         float lpRes) noexcept {
    const float clampedHp = clampf(hpHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float clampedLp = clampf(lpHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float rHp = clampf(hpRes, 0.0f, 1.0f);
    const float rLp = clampf(lpRes, 0.0f, 1.0f);

    constexpr float kButterK = 2.0f - kSqrt2;
    const float targetKHp = kButterK + rHp * (1.42f - 0.42f * rHp) * (2.82f - kButterK);
    const float targetKLp = kButterK + rLp * (1.42f - 0.42f * rLp) * (2.82f - kButterK);
    const float hpAct = smoothstep01((clampedHp - 20.0f) * 0.25f);
    const float lpAct = smoothstep01((20000.0f - clampedLp) * (1.0f / 1200.0f));

    const float dampHp = clampf(2.0f - (kButterK + hpAct * (targetKHp - kButterK)), 0.14f, 1.42f);
    const float dampLp = clampf(2.0f - (kButterK + lpAct * (targetKLp - kButterK)), 0.14f, 1.42f);
    const float effHp = 2.5f + hpAct * (clampedHp - 2.5f);
    const float effLp = 28000.0f + lpAct * (clampedLp - 28000.0f);

    return {
        1.0f / effHp,
        dampHp * dampHp,
        1.0f / effLp,
        dampLp * dampLp,
        hpAct,
        lpAct,
        clampedHp,
        clampedLp,
        rHp,
        rLp,
    };
}

inline float evalDualMs20Db(float freq, const DualMs20Coeffs& c) noexcept {
    const float rHp = freq * c.invEffHp;
    const float rHp2 = rHp * rHp;
    const float oneMinusHp2 = 1.0f - rHp2;
    const float denHp = oneMinusHp2 * oneMinusHp2 + c.dampHp2 * rHp2;
    const float mag2Hp = (rHp2 * rHp2) / (denHp + 1.0e-9f);

    const float rLp = freq * c.invEffLp;
    const float rLp2 = rLp * rLp;
    const float oneMinusLp2 = 1.0f - rLp2;
    const float denLp = oneMinusLp2 * oneMinusLp2 + c.dampLp2 * rLp2;
    const float mag2Lp = 1.0f / (denLp + 1.0e-9f);

    return 3.0103f * fastLog2(clampf(mag2Hp * mag2Lp, 1.0e-6f, 64.0f));
}

inline int16_t dbToFilterYFixed(float db) noexcept {
    const float yPx = clampf(kFilterYZeroDb - db * kFilterDbToPx, kFilterYTop, kFilterYBot);
    return static_cast<int16_t>(yPx * 64.0f + 0.5f);
}

inline ModulatedFilterParams computeLiveVoiceFilter(const VoiceParams& vp, float pressure,
                                                    float pitchMode, float lfoRaw) noexcept {
    const float hpBase =
        (vp.lfoTarget == LfoTarget::HpCutoff) ?
            applyLfoCutoffHz(vp.hpCutoffHz, vp.lfoDepth, lfoRaw) :
            vp.hpCutoffHz;
    const float hpResBase =
        (vp.lfoTarget == LfoTarget::HpResonance) ?
            applyLfoResonance(vp.hpResonance, vp.lfoDepth, lfoRaw) :
            vp.hpResonance;
    const float lpBase =
        (vp.lfoTarget == LfoTarget::LpCutoff) ?
            applyLfoCutoffHz(vp.lpCutoffHz, vp.lfoDepth, lfoRaw) :
            vp.lpCutoffHz;
    const float lpResBase =
        (vp.lfoTarget == LfoTarget::LpResonance) ?
            applyLfoResonance(vp.lpResonance, vp.lfoDepth, lfoRaw) :
            vp.lpResonance;
    return computeVoicePressureMacro(hpBase, hpResBase, lpBase, lpResBase, pressure,
                                     vp.muted ? 0.0f : 1.0f, pitchMode);
}

} // namespace

ScopeHudRenderer::ScopeHudRenderer() noexcept {
    reset();
}

void ScopeHudRenderer::reset() noexcept {
    history_.fill({});
    historyWrite_ = 0u;
    historyCount_ = 0u;
    bassZoom2x_ = true;
    scopeValid_ = false;
    trigSigValid_ = false;
    phosphorPrimed_ = false;
    smoothStride_ = 2.0f;
    timeSinceScopePushSec_ = 0.0f;
    thermalPhase_ = 0.0f;
    busPresenceSmooth_ = 0.0f;
    soloFactorSmooth_ = 0.0f;
    monoCollapseSmooth_ = 0.0f;
    voicePresenceSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prevTrigSig_.fill(0.0f);

    for (size_t s = 0u; s < kNumStrands; ++s) {
        strandSmoothY_[s].fill(120.0f);
        strandYFixed_[s].fill(static_cast<int16_t>(120 * 64));
        strandWeightQ8_[s] = 0u;
        strandTileMinY_[s].fill(255u);
        strandTileMaxY_[s].fill(0u);
    }
    for (auto& vt : voiceTrailQ12_) {
        vt.fill(0);
    }
    busTrailLQ12_.fill(0);
    busTrailRQ12_.fill(0);

    strandWeightQ8_[12] = 112u;
    rebuildTilesAndBloom();
    scopeValid_ = true;

    for (auto& fc : filterCurvesFixed_) {
        fc.fill(static_cast<int16_t>(132 * 64));
    }
    for (auto& cc : filterCompCurvesFixed_) {
        cc.fill(static_cast<int16_t>(132 * 64));
    }
    filterCompWeights_ = {0.0f, 0.0f};
    filterDivergenceSmooth_ = 0.0f;
    filterHpBeadX_ = 10.0f;
    filterHpBeadY_ = 132.0f;
    filterHpBeadInt_ = 0.0f;
    filterHpBeadRad_ = 2.0f;
    filterLpBeadX_ = 230.0f;
    filterLpBeadY_ = 132.0f;
    filterLpBeadInt_ = 0.0f;
    filterLpBeadRad_ = 2.0f;
    filterGraphPhase_ = 0.0f;
    filterBackdropDim_ = 0.0f;

    const auto lfoZeroFixed = static_cast<int16_t>(kLfoYZero * 64.0f + 0.5f);
    for (auto& lc : lfoCurvesFixed_) {
        lc.fill(lfoZeroFixed);
    }
    lfoDivergenceSmooth_ = 0.0f;
    lfoGraphPhase_ = 0.0f;
    frameLfoOutput_ = {0.0f, 0.0f, 0.0f, 0.0f};
    frameLfoPhase_ = {0.0f, 0.25f, 0.50f, 0.75f};
    frameLfoDriftPacked_ = {0u, 0u, 0u, 0u};
    lfoBeadIntensity_ = {0.0f, 0.0f, 0.0f, 0.0f};

    frameControl_ = EngineControlState::makeDefault();
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        prevVoiceFreqHz_[v] = frameControl_.voices[v].frequencyHz;
    }
    frameHud_ = {};
    prevRawHud_ = {};
    releaseLatchHud_ = {};
    knobPosSignedSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobDimSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobFocusSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    headerBarSmooth_ = 0.0f;
    renderedHudOpacity_ = 0.0f;
    smoothedPeakL_ = 0.0f;
    smoothedPeakR_ = 0.0f;
    prevHeaderBarMode_ = HudMode::None;
    releaseLatchActive_ = false;
    hudGaugesPrimed_ = false;
}

void ScopeHudRenderer::ingestScopeRing(ScopeRingBuffer& ring) noexcept {
    std::array<ScopeFrame, 256> scratch{};
    while (ring.available() > 0u) {
        const size_t n = ring.popBatch(scratch.data(), scratch.size());
        if (n == 0u) {
            break;
        }
        pushScopeSamples(scratch.data(), n);
    }
}

void ScopeHudRenderer::pushScopeSamples(const ScopeFrame* frames, size_t count) noexcept {
    if (frames == nullptr || count == 0u) {
        return;
    }
    timeSinceScopePushSec_ = 0.0f;
    for (size_t i = 0u; i < count; ++i) {
        if (!frameHud_.scopeFrozen) {
            CompactScopeSample packed{};
            packed.leftQ12 = packQ3_12(frames[i].left);
            packed.rightQ12 = packQ3_12(frames[i].right);
            const uint8_t mask = frames[i].voiceRisingMask;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const bool edge = ((mask >> v) & 1u) != 0u;
                packed.voiceEdgeQ12[v] = packVoiceEdgeQ12(frames[i].voiceQ13[v], edge);
            }
            history_[historyWrite_] = packed;
            historyWrite_ = (historyWrite_ + 1u) & kHistoryMask;
            if (historyCount_ < kHistorySize) {
                ++historyCount_;
            }
        }
    }
}

void ScopeHudRenderer::rebuildTilesAndBloom() noexcept {
    // Populate 8-column tile Y-bounds AND splat analytical photon energy into 60x60 bloom grid
    bloomGrid_.fill(0u);
    constexpr float kFixedToPx = 1.0f / 64.0f;

    for (size_t s = 0u; s < kNumStrands; ++s) {
        if (strandWeightQ8_[s] == 0u) {
            strandTileMinY_[s].fill(255u);
            strandTileMaxY_[s].fill(0u);
            continue;
        }
        const float wgt = static_cast<float>(strandWeightQ8_[s]) * (1.0f / 256.0f);
        const auto& yFixed = strandYFixed_[s];

        for (size_t tile = 0u; tile < kNumTiles; ++tile) {
            int tileMin = static_cast<int>(kDisplayHeight);
            int tileMax = -1;
            const size_t colBase = tile * kTileCols;

            for (size_t c = 0u; c < kTileCols; ++c) {
                const size_t col = colBase + c;
                const float yA = static_cast<float>(yFixed[col]) * kFixedToPx;
                const float yB = static_cast<float>(yFixed[col + 1u]) * kFixedToPx;
                const float yLo = (yA < yB) ? yA : yB;
                const float yHi = (yA > yB) ? yA : yB;

                const int rawTop = static_cast<int>(yLo + 512.0f) - 512 - 3;
                const int rawBot = static_cast<int>(yHi + 512.0f) - 512 + 4;
                if (rawBot >= 0 && rawTop < static_cast<int>(kDisplayHeight)) {
                    const int segTop = (rawTop < 0) ? 0 : rawTop;
                    const int segBot = (rawBot >= static_cast<int>(kDisplayHeight)) ?
                                           (static_cast<int>(kDisplayHeight) - 1) :
                                           rawBot;
                    if (segTop < tileMin) {
                        tileMin = segTop;
                    }
                    if (segBot > tileMax) {
                        tileMax = segBot;
                    }
                }

                // Analytical column photon integral stored in Q12.4 (*16) for blur precision.
                // Segments deflecting off-screen only deposit photons into visible [0..59] cells.
                const int byStart = static_cast<int>((yLo + 512.0f) * 0.25f) - 128;
                const int byEnd = static_cast<int>((yHi + 512.0f) * 0.25f) - 128;
                if (byEnd >= 0 && byStart < static_cast<int>(kBloomHeight)) {
                    const float dy = yB - yA;
                    const float len2 = 1.0f + dy * dy;
                    const float segLen = clampf(len2 * fastInvSqrt(len2), 1.0f, 4.2f);
                    const float dwell =
                        (0.58f + 0.42f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * wgt;
                    const uint32_t energyQ4 = static_cast<uint32_t>(1174.4f * dwell * segLen);

                    const size_t bx = col >> 2u;
                    const uint32_t cellCount = static_cast<uint32_t>(byEnd - byStart + 1);
                    const uint32_t addPerCell = energyQ4 / cellCount;
                    const size_t by0 = (byStart < 0) ? 0u : static_cast<size_t>(byStart);
                    const size_t by1 = (byEnd >= static_cast<int>(kBloomHeight)) ?
                                           (kBloomHeight - 1u) :
                                           static_cast<size_t>(byEnd);
                    for (size_t by = by0; by <= by1; ++by) {
                        uint16_t& cell = bloomGrid_[by * kBloomWidth + bx];
                        const uint32_t next = static_cast<uint32_t>(cell) + addPerCell;
                        cell = static_cast<uint16_t>((next > 65535u) ? 65535u : next);
                    }
                }
            }

            if (tileMin <= tileMax) {
                strandTileMinY_[s][tile] = static_cast<uint8_t>(tileMin);
                strandTileMaxY_[s][tile] = static_cast<uint8_t>(tileMax);
            } else {
                strandTileMinY_[s][tile] = 255u;
                strandTileMaxY_[s][tile] = 0u;
            }
        }
    }

    // 2-pass in-place separable [1, 3, 4, 3, 1]/12 blur on 60x60 grid (120-byte stack scratch),
    // baking calibrated 25% Caustic Bloom (0.3625) into Q13.3 (*8) on the final vertical pass.
    std::array<uint16_t, kBloomWidth> lineTmp{};
    for (size_t pass = 0u; pass < 2u; ++pass) {
        for (size_t by = 0u; by < kBloomHeight; ++by) {
            uint16_t* row = bloomGrid_.data() + by * kBloomWidth;
            for (size_t bx = 0u; bx < kBloomWidth; ++bx) {
                const size_t x0 = (bx >= 2u) ? (bx - 2u) : 0u;
                const size_t x1 = (bx >= 1u) ? (bx - 1u) : 0u;
                const size_t x3 = (bx + 1u < kBloomWidth) ? (bx + 1u) : (kBloomWidth - 1u);
                const size_t x4 = (bx + 2u < kBloomWidth) ? (bx + 2u) : (kBloomWidth - 1u);
                const uint32_t sum = static_cast<uint32_t>(row[x0]) +
                                     3u * static_cast<uint32_t>(row[x1]) +
                                     4u * static_cast<uint32_t>(row[bx]) +
                                     3u * static_cast<uint32_t>(row[x3]) +
                                     static_cast<uint32_t>(row[x4]);
                lineTmp[bx] = static_cast<uint16_t>((sum + 6u) / 12u);
            }
            for (size_t bx = 0u; bx < kBloomWidth; ++bx) {
                row[bx] = lineTmp[bx];
            }
        }

        for (size_t bx = 0u; bx < kBloomWidth; ++bx) {
            for (size_t by = 0u; by < kBloomHeight; ++by) {
                const size_t y0 = (by >= 2u) ? (by - 2u) : 0u;
                const size_t y1 = (by >= 1u) ? (by - 1u) : 0u;
                const size_t y3 = (by + 1u < kBloomHeight) ? (by + 1u) : (kBloomHeight - 1u);
                const size_t y4 = (by + 2u < kBloomHeight) ? (by + 2u) : (kBloomHeight - 1u);
                const uint32_t sum =
                    static_cast<uint32_t>(bloomGrid_[y0 * kBloomWidth + bx]) +
                    3u * static_cast<uint32_t>(bloomGrid_[y1 * kBloomWidth + bx]) +
                    4u * static_cast<uint32_t>(bloomGrid_[by * kBloomWidth + bx]) +
                    3u * static_cast<uint32_t>(bloomGrid_[y3 * kBloomWidth + bx]) +
                    static_cast<uint32_t>(bloomGrid_[y4 * kBloomWidth + bx]);
                if (pass == 0u) {
                    lineTmp[by] = static_cast<uint16_t>((sum + 6u) / 12u);
                } else {
                    // Q12.4 (*16) -> Q13.3 (*8) * 0.3625 / 12 = 0.01510417 ~= 990 / 65536
                    const uint32_t scaled = (sum * 990u + 32768u) >> 16u;
                    lineTmp[by] = static_cast<uint16_t>((scaled > 65535u) ? 65535u : scaled);
                }
            }
            for (size_t by = 0u; by < kBloomHeight; ++by) {
                bloomGrid_[by * kBloomWidth + bx] = lineTmp[by];
            }
        }
    }
}

void ScopeHudRenderer::buildWaveformTrace(float dtSec, const EngineControlState& control) noexcept {
    if (frameHud_.scopeFrozen && scopeValid_) {
        return;
    }

    const float dtClamped = clampf(dtSec, 0.002f, 0.080f);

    // Select primary trigger anchor (Osc 1 if active audio-rate, else first unmuted audio voice)
    size_t anchorVoice = 0u;
    bool hasAudioAnchor = false;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (!control.voices[v].muted && control.voices[v].frequencyHz >= kAudioFreqMinHz) {
            anchorVoice = v;
            hasAudioAnchor = true;
            break;
        }
    }

    if (hasAudioAnchor) {
        const float anchorHz = control.voices[anchorVoice].frequencyHz;
        if (bassZoom2x_ && anchorHz > 92.0f) {
            bassZoom2x_ = false;
        } else if (!bassZoom2x_ && anchorHz < 78.0f) {
            bassZoom2x_ = true;
        }
    }

    const float targetStride = bassZoom2x_ ? 2.0f : 1.0f;
    if (!phosphorPrimed_) {
        smoothStride_ = targetStride;
    } else {
        smoothStride_ = slewScalar(
            smoothStride_, targetStride, 1.0f - fastExpNeg(dtClamped * (1.0f / 0.160f)), 0.001f);
    }

    constexpr size_t kTrigPt = 120u;
    constexpr size_t kLeftSpan = kTrigPt * 2u + 8u;
    constexpr size_t kRightSpan = (kTracePoints - kTrigPt) * 2u + 44u;

    if (historyCount_ < kLeftSpan + kRightSpan + 8u) {
        // Pre-audio boot state: prime flatline at y=120 so first audio start blooms smoothly out
        phosphorPrimed_ = true;
        scopeValid_ = true;
        return;
    }

    const bool streamStalled = (timeSinceScopePushSec_ > 0.065f);

    std::array<float, kTracePoints> liveL{};
    std::array<float, kTracePoints> liveR{};
    std::array<std::array<float, kTracePoints>, kNumOscillators> voiceCur{};
    std::array<float, kNumOscillators> voicePeak{0.0f, 0.0f, 0.0f, 0.0f};
    float busPeak = 0.0f;
    size_t chosenTrigInt = (historyWrite_ - kRightSpan) & kHistoryMask;
    float totalShift = 0.0f;

    if (!streamStalled) {
        const size_t latestEdgeLimit = (historyWrite_ - kRightSpan) & kHistoryMask;
        const size_t maxBack = historyCount_ - kLeftSpan - kRightSpan;
        const size_t searchLen = (maxBack > 3480u) ? 3480u : maxBack;

        struct TrigCandidate {
            size_t trigInt = 0u;
            float trigFrac = 0.0f;
        };
        constexpr size_t kMaxCandidates = 8u;
        std::array<TrigCandidate, kMaxCandidates> candidates{};
        size_t numCand = 0u;
        size_t minNextBack = 0u;

        if (hasAudioAnchor && searchLen > 0u) {
            for (size_t back = 0u; back < searchLen && numCand < kMaxCandidates; ++back) {
                if (back < minNextBack) {
                    continue;
                }
                const size_t edgePos = (latestEdgeLimit - back) & kHistoryMask;
                if (!unpackVoiceEdgeFlag(history_[edgePos].voiceEdgeQ12[anchorVoice])) {
                    continue;
                }

                // Refine oscillator period marker to exact continuous post-MS-20 rising zero-cross
                size_t candInt = edgePos;
                float candFrac = 0.0f;
                for (int off = -3; off <= 32; ++off) {
                    const size_t p1 =
                        (edgePos + static_cast<size_t>(off + static_cast<int>(kHistorySize))) &
                        kHistoryMask;
                    const size_t p0 = (p1 - 1u) & kHistoryMask;
                    const float v0 = unpackVoiceEdgeSample(history_[p0].voiceEdgeQ12[anchorVoice]);
                    const float v1 = unpackVoiceEdgeSample(history_[p1].voiceEdgeQ12[anchorVoice]);
                    const float slope = v1 - v0;
                    if (v0 <= 0.0f && v1 > 0.0f && slope > 0.002f) {
                        candInt = p0;
                        candFrac = clampf(-v0 / slope, 0.0f, 1.0f);
                        break;
                    }
                }
                candidates[numCand++] = {candInt, candFrac};
                minNextBack = back + 16u;
            }
        }

        chosenTrigInt = latestEdgeLimit;
        float chosenTrigFrac = 0.0f;
        if (numCand == 1u || (numCand > 1u && !trigSigValid_)) {
            chosenTrigInt = candidates[0].trigInt;
            chosenTrigFrac = candidates[0].trigFrac;
        } else if (numCand > 1u) {
            // Multi-cycle per-voice super-period lock: prevents 30 Hz flip on 3:2 / 4:3 intervals
            float bestScore = 1.0e9f;
            size_t bestC = 0u;
            for (size_t c = 0u; c < numCand; ++c) {
                float err = 0.0f;
                const size_t cTrig = candidates[c].trigInt;
                const float cFrac = candidates[c].trigFrac;
                for (size_t k = 0u; k < kTrigSigTaps; ++k) {
                    const float relOff =
                        cFrac + static_cast<float>(static_cast<int>(10u + k * 20u) -
                                                   static_cast<int>(kTrigPt)) *
                                    smoothStride_;
                    const int relFloor = static_cast<int>(relOff + 512.0f) - 512;
                    const float relFrac = clampf(relOff - static_cast<float>(relFloor), 0.0f, 1.0f);
                    const size_t p0 =
                        (cTrig + static_cast<size_t>(relFloor + static_cast<int>(kHistorySize))) &
                        kHistoryMask;
                    const size_t p1 = (p0 + 1u) & kHistoryMask;
                    for (size_t v = 0u; v < kNumOscillators; ++v) {
                        const float v0 = unpackVoiceEdgeSample(history_[p0].voiceEdgeQ12[v]);
                        const float v1 = unpackVoiceEdgeSample(history_[p1].voiceEdgeQ12[v]);
                        const float sv = v0 + relFrac * (v1 - v0);
                        err += absf(sv - prevTrigSig_[k * kNumOscillators + v]);
                    }
                }
                const float score = (err * (1.0f / static_cast<float>(kTrigSigPoints))) +
                                    0.008f * static_cast<float>(c);
                if (score < bestScore) {
                    bestScore = score;
                    bestC = c;
                }
            }
            chosenTrigInt = candidates[bestC].trigInt;
            chosenTrigFrac = candidates[bestC].trigFrac;
        }

        // Ultra-slow (~15 s) sub-pixel analog breath keeps crystal-locked solo tones gently alive
        thermalPhase_ += clampf(dtSec, 0.0f, 0.10f) * 0.065f;
        while (thermalPhase_ >= 1.0f) {
            thermalPhase_ -= 1.0f;
        }
        const float slowBreathSamples =
            0.52f *
            (fastSinPhase01(thermalPhase_) + 0.45f * fastSinPhase01(thermalPhase_ * 2.0f + 0.19f));
        totalShift = clampf(chosenTrigFrac + slowBreathSamples, -2.0f, 2.0f);

        // Sample 241 sub-pixel Catmull-Rom points along continuous `smoothStride_` timebase
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float relOff =
                totalShift + (static_cast<float>(pt) - static_cast<float>(kTrigPt)) * smoothStride_;
            const int offFloor = static_cast<int>(relOff + 512.0f) - 512;
            const float frac = clampf(relOff - static_cast<float>(offFloor), 0.0f, 1.0f);
            const size_t base =
                (chosenTrigInt + static_cast<size_t>(offFloor + static_cast<int>(kHistorySize))) &
                kHistoryMask;
            const size_t i0 = (base - 1u) & kHistoryMask;
            const size_t i1 = base;
            const size_t i2 = (base + 1u) & kHistoryMask;
            const size_t i3 = (base + 2u) & kHistoryMask;

            const float sL = catmullRom(unpackQ3_12(history_[i0].leftQ12),
                                        unpackQ3_12(history_[i1].leftQ12),
                                        unpackQ3_12(history_[i2].leftQ12),
                                        unpackQ3_12(history_[i3].leftQ12), frac) *
                             1.50f;
            const float sR = catmullRom(unpackQ3_12(history_[i0].rightQ12),
                                        unpackQ3_12(history_[i1].rightQ12),
                                        unpackQ3_12(history_[i2].rightQ12),
                                        unpackQ3_12(history_[i3].rightQ12), frac) *
                             1.50f;
            liveL[pt] = sL;
            liveR[pt] = sR;
            const float pkBus = (absf(sL) > absf(sR)) ? absf(sL) : absf(sR);
            if (pkBus > busPeak) {
                busPeak = pkBus;
            }

            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float sv = catmullRom(unpackVoiceEdgeSample(history_[i0].voiceEdgeQ12[v]),
                                            unpackVoiceEdgeSample(history_[i1].voiceEdgeQ12[v]),
                                            unpackVoiceEdgeSample(history_[i2].voiceEdgeQ12[v]),
                                            unpackVoiceEdgeSample(history_[i3].voiceEdgeQ12[v]),
                                            frac);
                voiceCur[v][pt] = sv;
                const float av = absf(sv);
                if (av > voicePeak[v]) {
                    voicePeak[v] = av;
                }
            }
        }
    }

    // Continuous presence envelopes (eliminates 1-frame snap to/from y=120 flatline & voice mutes)
    std::array<float, kNumOscillators> rawVoicePres{};
    float rawTotalVoicePres = 0.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        rawVoicePres[v] = smoothstep01(voicePeak[v] * (1.0f / 0.028f));
        rawTotalVoicePres += rawVoicePres[v];
    }
    const float rawBusPres = smoothstep01(busPeak * (1.0f / 0.022f));
    const float rawVoiceMixPres = smoothstep01(rawTotalVoicePres * 0.5f);
    const float rawAnyPres = (rawBusPres > rawVoiceMixPres) ? rawBusPres : rawVoiceMixPres;

    if (!phosphorPrimed_) {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            voicePresenceSmooth_[v] = rawVoicePres[v];
        }
        busPresenceSmooth_ = rawAnyPres;
        soloFactorSmooth_ = clampf(2.0f - rawTotalVoicePres, 0.0f, 1.0f);
        monoCollapseSmooth_ = control.global.monoCollapse ? 1.0f : 0.0f;
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float tauV = (rawVoicePres[v] >= voicePresenceSmooth_[v]) ? 0.065f : 0.210f;
            voicePresenceSmooth_[v] = slewScalar(
                voicePresenceSmooth_[v], rawVoicePres[v],
                1.0f - fastExpNeg(dtClamped * (1.0f / tauV)), 0.002f);
        }
        const float tauBus = (rawAnyPres >= busPresenceSmooth_) ? 0.065f : 0.220f;
        busPresenceSmooth_ = slewScalar(busPresenceSmooth_, rawAnyPres,
                                        1.0f - fastExpNeg(dtClamped * (1.0f / tauBus)), 0.002f);
        if (rawTotalVoicePres > 0.35f) {
            const float targetSolo = clampf(2.0f - rawTotalVoicePres, 0.0f, 1.0f);
            soloFactorSmooth_ = slewScalar(soloFactorSmooth_, targetSolo,
                                           1.0f - fastExpNeg(dtClamped * (1.0f / 0.110f)), 0.002f);
        }
        monoCollapseSmooth_ = slewScalar(monoCollapseSmooth_,
                                         control.global.monoCollapse ? 1.0f : 0.0f,
                                         1.0f - fastExpNeg(dtClamped * (1.0f / 0.060f)), 0.002f);
    }

    // Update per-voice super-period reference signature only while active audio is present
    if (rawAnyPres > 0.15f) {
        for (size_t k = 0u; k < kTrigSigTaps; ++k) {
            const size_t pt = 10u + k * 20u;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float sv = voiceCur[v][pt];
                float& ref = prevTrigSig_[k * kNumOscillators + v];
                ref = (!trigSigValid_) ? sv : (ref + 0.25f * (sv - ref));
            }
        }
        trigSigValid_ = true;
    } else if (busPresenceSmooth_ <= 0.005f) {
        trigSigValid_ = false;
    }

    // Poised galvanometer relaxation when collapsing toward silence, tight tracking while active
    const float stoppingBlend = clampf((busPresenceSmooth_ - rawAnyPres) * 1.5f, 0.0f, 1.0f);
    const float alphaMid =
        1.0f - fastExpNeg(dtClamped * (1.0f / (0.085f + 0.055f * stoppingBlend)));
    const float alphaSlow =
        1.0f - fastExpNeg(dtClamped * (1.0f / (0.135f + 0.055f * stoppingBlend)));

    // Continuous phosphor stereo bus trail (replaces discrete 3-frame ring jumps)
    std::array<float, kTracePoints> trailBusL{};
    std::array<float, kTracePoints> trailBusR{};
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        const float prevL = unpackQ3_12(busTrailLQ12_[pt]);
        const float prevR = unpackQ3_12(busTrailRQ12_[pt]);
        trailBusL[pt] = (!phosphorPrimed_) ? liveL[pt] : (prevL + alphaMid * (liveL[pt] - prevL));
        trailBusR[pt] = (!phosphorPrimed_) ? liveR[pt] : (prevR + alphaMid * (liveR[pt] - prevR));
        busTrailLQ12_[pt] = packQ3_12(trailBusL[pt]);
        busTrailRQ12_[pt] = packQ3_12(trailBusR[pt]);
    }

    const float soloFactor = soloFactorSmooth_;
    const float voiceVisGain = 1.12f + 0.24f * soloFactor;
    const float busSympathy = 0.22f * (1.0f - soloFactor);

    const float startRelOff = totalShift - static_cast<float>(kTrigPt) * smoothStride_;
    const int startOffFloor = static_cast<int>(startRelOff + 512.0f) - 512;
    const size_t startIdx =
        (chosenTrigInt + static_cast<size_t>(startOffFloor + static_cast<int>(kHistorySize))) &
        kHistoryMask;
    const size_t oldestHistIdx = (historyWrite_ - historyCount_) & kHistoryMask;
    const size_t preAvailSamples =
        streamStalled ? 0u : ((startIdx - oldestHistIdx) & kHistoryMask);
    size_t preWarmSteps =
        (preAvailSamples > 6u) ? static_cast<size_t>(static_cast<float>(preAvailSamples - 4u) /
                                                     smoothStride_) :
                                 0u;
    if (preWarmSteps > 40u) {
        preWarmSteps = 40u;
    }
    const float gCap = 0.017095f * smoothStride_;
    const float kCapG = gCap / (1.0f + gCap);

    std::array<float, kTracePoints> voiceCap{};
    std::array<float, kTracePoints> scratchA{};
    std::array<float, kTracePoints> scratchB{};

    auto applyStrandPhosphor = [this](size_t s, const float* targetY, float alpha,
                                      bool snapImmediate) noexcept {
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            float y = targetY[pt];
            if (!snapImmediate) {
                y = strandSmoothY_[s][pt] + alpha * (y - strandSmoothY_[s][pt]);
                if (absf(y - 120.0f) < 0.008f && absf(targetY[pt] - 120.0f) < 0.001f) {
                    y = 120.0f;
                }
            }
            strandSmoothY_[s][pt] = y;
            strandYFixed_[s][pt] = screenYToFixed(y);
        }
    };

    // Build 12 coupled per-voice circuit strands (4 voices x 3 strands, open edge-to-edge)
    constexpr float kBaseDispersiveShifts[kNumOscillators] = {-5.0f, -2.0f, 2.0f, 5.0f};
    constexpr float kSoloDispersiveShifts[kNumOscillators] = {-5.0f, -5.0f, 5.0f, 5.0f};
    const bool snapColdBoot = !phosphorPrimed_;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const size_t baseStrand = v * 3u;
        const float pres = voicePresenceSmooth_[v];
        if (pres <= 0.001f && rawVoicePres[v] <= 0.001f) {
            voiceTrailQ12_[v].fill(0);
            for (size_t k = 0u; k < 3u; ++k) {
                strandSmoothY_[baseStrand + k].fill(120.0f);
                strandYFixed_[baseStrand + k].fill(static_cast<int16_t>(120 * 64));
                strandWeightQ8_[baseStrand + k] = 0u;
                strandTileMinY_[baseStrand + k].fill(255u);
                strandTileMaxY_[baseStrand + k].fill(0u);
            }
            continue;
        }

        const float vStop = clampf((pres - rawVoicePres[v]) * 1.5f, 0.0f, 1.0f);
        const float vAlphaFast =
            1.0f - fastExpNeg(dtClamped * (1.0f / (0.038f + 0.057f * vStop)));
        const float vAlphaMid =
            1.0f - fastExpNeg(dtClamped * (1.0f / (0.085f + 0.055f * vStop)));
        const float vAlphaSlow =
            1.0f - fastExpNeg(dtClamped * (1.0f / (0.135f + 0.055f * vStop)));

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float curScaled = voiceCur[v][pt] * voiceVisGain;
            voiceCur[v][pt] = curScaled;
            float tv = curScaled;
            if (!snapColdBoot) {
                const float prevTv = unpackQ3_12(voiceTrailQ12_[v][pt]);
                tv = prevTv + vAlphaMid * (curScaled - prevTv);
            }
            voiceTrailQ12_[v][pt] = packQ3_12(tv);
            scratchA[pt] = tv;
        }

        // Pre-warm 240 Hz TPT leaky-capacitor state from history so x=0 is transient-free
        float capState = voiceCur[v][0] * 0.35f;
        if (preWarmSteps > 0u) {
            const int firstBack =
                static_cast<int>(static_cast<float>(preWarmSteps) * smoothStride_ + 0.5f);
            const size_t firstWarm =
                (startIdx + static_cast<size_t>(static_cast<int>(kHistorySize) - firstBack)) &
                kHistoryMask;
            capState =
                unpackVoiceEdgeSample(history_[firstWarm].voiceEdgeQ12[v]) * voiceVisGain * 0.35f;
            for (size_t step = preWarmSteps; step >= 1u; --step) {
                const int backSamples =
                    static_cast<int>(static_cast<float>(step) * smoothStride_ + 0.5f);
                const size_t wIdx =
                    (startIdx + static_cast<size_t>(static_cast<int>(kHistorySize) - backSamples)) &
                    kHistoryMask;
                const float xWarm =
                    unpackVoiceEdgeSample(history_[wIdx].voiceEdgeQ12[v]) * voiceVisGain;
                const float vCap = (xWarm - capState) * kCapG;
                const float yCap = vCap + capState;
                capState = yCap + vCap;
            }
        }
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float vCap = (voiceCur[v][pt] - capState) * kCapG;
            const float yCap = vCap + capState;
            capState = yCap + vCap;
            voiceCap[pt] = yCap * 1.32f;
        }

        const auto& busRef = ((v & 1u) != 0u) ? liveR : liveL;
        const auto& prevBusRef = ((v & 1u) != 0u) ? trailBusR : trailBusL;
        const float vSympathy = busSympathy * rawVoicePres[v];

        // Strand A: 240 Hz TPT capacitor trajectory + calm phosphor persistence
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float coupledCap =
                (1.0f - vSympathy) * voiceCap[pt] + vSympathy * prevBusRef[pt];
            scratchB[pt] = sampleToScreenY(coupledCap * 0.76f);
        }
        applyStrandPhosphor(baseStrand + 0u, scratchB.data(), vAlphaSlow, snapColdBoot);
        strandWeightQ8_[baseStrand + 0u] =
            static_cast<uint16_t>((0.34f + 0.16f * soloFactor) * pres * 256.0f + 0.5f);

        // Strand B: Continuous phosphor-trailed dispersive harmonic shell
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchA[pt] = (1.0f - vSympathy * 1.15f) * scratchA[pt] +
                           (vSympathy * 1.15f) * prevBusRef[pt];
        }
        const float dispShift =
            kBaseDispersiveShifts[v] +
            (kSoloDispersiveShifts[v] - kBaseDispersiveShifts[v]) * soloFactor;
        dispersiveShell(scratchA.data(), scratchB.data(), kTracePoints, 0.30f, dispShift);
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchB[pt] = sampleToScreenY(scratchB[pt] * 0.92f);
        }
        applyStrandPhosphor(baseStrand + 1u, scratchB.data(), vAlphaMid, snapColdBoot);
        strandWeightQ8_[baseStrand + 1u] =
            static_cast<uint16_t>((0.37f + 0.19f * soloFactor) * pres * 256.0f + 0.5f);

        // Strand C: Zero-phase smoothed live post-MS-20 voice output + responsive phosphor lock
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchA[pt] = (1.0f - vSympathy * 0.85f) * voiceCur[v][pt] +
                           (vSympathy * 0.85f) * busRef[pt];
        }
        smoothZeroPhase(scratchA.data(), scratchB.data(), kTracePoints, 0.36f, 2u);
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchB[pt] = sampleToScreenY(scratchB[pt] * 1.04f);
        }
        applyStrandPhosphor(baseStrand + 2u, scratchB.data(), vAlphaFast, snapColdBoot);
        strandWeightQ8_[baseStrand + 2u] =
            static_cast<uint16_t>((0.45f + 0.27f * soloFactor) * pres * 256.0f + 0.5f);
    }

    // Strands 12 & 13: Master stereo bus inner harmonic ribs (Strand 12 settles into y=120 horizon)
    smoothZeroPhase(liveL.data(), scratchB.data(), kTracePoints, 0.24f, 2u);
    const float s12Gain = 0.56f - 0.12f * soloFactor;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchB[pt] = sampleToScreenY(scratchB[pt] * s12Gain);
    }
    applyStrandPhosphor(12u, scratchB.data(), alphaSlow, snapColdBoot);
    const float w12 = 0.4375f + (0.40f - 0.4375f) * busPresenceSmooth_;
    strandWeightQ8_[12] = static_cast<uint16_t>(w12 * 256.0f + 0.5f);

    const float soloRibBlend = smoothstep01((soloFactor - 0.45f) * (1.0f / 0.45f));
    const float trailRibMix =
        (monoCollapseSmooth_ > soloRibBlend) ? monoCollapseSmooth_ : soloRibBlend;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchA[pt] = liveR[pt] + trailRibMix * (trailBusL[pt] - liveR[pt]);
    }
    smoothZeroPhase(scratchA.data(), scratchB.data(), kTracePoints, 0.28f, 2u);
    const float s13Gain = 0.82f - 0.20f * soloFactor;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchB[pt] = sampleToScreenY(scratchB[pt] * s13Gain);
    }
    applyStrandPhosphor(13u, scratchB.data(), alphaMid, snapColdBoot);
    strandWeightQ8_[13] = static_cast<uint16_t>(0.46f * busPresenceSmooth_ * 256.0f + 0.5f);
    phosphorPrimed_ = true;

    rebuildTilesAndBloom();
    scopeValid_ = true;
}

void ScopeHudRenderer::updateFilterGraph(
    float dtClamped, bool snapImmediate,
    const std::array<ModulatedFilterParams, kNumOscillators>& mods, bool unified) noexcept {
    std::array<std::array<int16_t, kFilterPoints>, kNumOscillators> tgtCurves{};
    std::array<std::array<int16_t, kFilterPoints>, 2> tgtComp{};
    std::array<float, 2> tgtCompWeights{0.0f, 0.0f};
    float tgtHpX = filterHpBeadX_;
    float tgtHpY = filterHpBeadY_;
    float tgtHpInt = 0.0f;
    float tgtHpRad = filterHpBeadRad_;
    float tgtLpX = filterLpBeadX_;
    float tgtLpY = filterLpBeadY_;
    float tgtLpInt = 0.0f;
    float tgtLpRad = filterLpBeadRad_;

    if (unified) {
        const DualMs20Coeffs mainC = makeDualMs20Coeffs(mods[0].hpCutoffHz, mods[0].hpResonance,
                                                        mods[0].lpCutoffHz, mods[0].lpResonance);
        const float resEnergy = 0.5f * (mainC.rHp * mainC.hpAct + mainC.rLp * mainC.lpAct);
        const DualMs20Coeffs comp1C =
            makeDualMs20Coeffs(mainC.clampedHp * 1.03f, mainC.rHp * 0.76f, mainC.clampedLp * 0.97f,
                               mainC.rLp * 0.76f);
        const DualMs20Coeffs comp2C =
            makeDualMs20Coeffs(mainC.clampedHp * 1.07f, mainC.rHp * 0.54f, mainC.clampedLp * 0.93f,
                               mainC.rLp * 0.54f);
        const float drop1Db = 1.1f + 1.8f * resEnergy;
        const float drop2Db = 2.3f + 3.0f * resEnergy;

        for (size_t i = 0u; i < kFilterPoints; ++i) {
            const float freq = kFilterSweepFreqs[i];
            const int16_t yMain = dbToFilterYFixed(evalDualMs20Db(freq, mainC));
            tgtCurves[0][i] = yMain;
            tgtCurves[1][i] = yMain;
            tgtCurves[2][i] = yMain;
            tgtCurves[3][i] = yMain;
            tgtComp[0][i] = dbToFilterYFixed(evalDualMs20Db(freq, comp1C) - drop1Db);
            tgtComp[1][i] = dbToFilterYFixed(evalDualMs20Db(freq, comp2C) - drop2Db);
        }
        tgtCompWeights[0] = 0.28f + 0.24f * resEnergy;
        tgtCompWeights[1] = 0.18f + 0.22f * resEnergy;

        if (mainC.hpAct > 0.02f) {
            const float uHp = clampf(
                (fastLog2(mainC.clampedHp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtHpX =
                static_cast<float>(kFilterXStart) + uHp * static_cast<float>(kFilterPoints - 1u);
            const float hpDb = evalDualMs20Db(mainC.clampedHp, mainC);
            tgtHpY = clampf(kFilterYZeroDb - hpDb * kFilterDbToPx, kFilterYTop, kFilterYBot);
            tgtHpInt = (0.78f + 0.22f * mainC.rHp) * mainC.hpAct;
            tgtHpRad = 2.0f + 1.35f * mainC.rHp;
        }
        if (mainC.lpAct > 0.02f) {
            const float uLp = clampf(
                (fastLog2(mainC.clampedLp) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            tgtLpX =
                static_cast<float>(kFilterXStart) + uLp * static_cast<float>(kFilterPoints - 1u);
            const float lpDb = evalDualMs20Db(mainC.clampedLp, mainC);
            tgtLpY = clampf(kFilterYZeroDb - lpDb * kFilterDbToPx, kFilterYTop, kFilterYBot);
            tgtLpInt = (0.78f + 0.22f * mainC.rLp) * mainC.lpAct;
            tgtLpRad = 2.0f + 1.35f * mainC.rLp;
        }
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const DualMs20Coeffs c = makeDualMs20Coeffs(mods[v].hpCutoffHz, mods[v].hpResonance,
                                                        mods[v].lpCutoffHz, mods[v].lpResonance);
            for (size_t i = 0u; i < kFilterPoints; ++i) {
                tgtCurves[v][i] = dbToFilterYFixed(evalDualMs20Db(kFilterSweepFreqs[i], c));
            }
        }
        tgtComp[0] = tgtCurves[0];
        tgtComp[1] = tgtCurves[0];
    }

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));
    const float alphaDiv =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.055f)));

    filterDivergenceSmooth_ =
        slewScalar(filterDivergenceSmooth_, unified ? 0.0f : 1.0f, alphaDiv, 0.001f);
    filterCompWeights_[0] = slewScalar(filterCompWeights_[0], tgtCompWeights[0], alphaCurve);
    filterCompWeights_[1] = slewScalar(filterCompWeights_[1], tgtCompWeights[1], alphaCurve);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        for (size_t i = 0u; i < kFilterPoints; ++i) {
            filterCurvesFixed_[v][i] =
                slewQ8_6(filterCurvesFixed_[v][i], tgtCurves[v][i], alphaCurve, snapImmediate);
        }
    }
    for (size_t c = 0u; c < 2u; ++c) {
        for (size_t i = 0u; i < kFilterPoints; ++i) {
            filterCompCurvesFixed_[c][i] =
                slewQ8_6(filterCompCurvesFixed_[c][i], tgtComp[c][i], alphaCurve, snapImmediate);
        }
    }

    if (tgtHpInt > 0.001f) {
        const bool snapPos = snapImmediate || (filterHpBeadInt_ <= 0.005f);
        filterHpBeadX_ = snapPos ? tgtHpX : slewScalar(filterHpBeadX_, tgtHpX, alphaCurve, 0.01f);
        filterHpBeadY_ = snapPos ? tgtHpY : slewScalar(filterHpBeadY_, tgtHpY, alphaCurve, 0.01f);
        filterHpBeadRad_ =
            snapPos ? tgtHpRad : slewScalar(filterHpBeadRad_, tgtHpRad, alphaCurve, 0.005f);
    }
    filterHpBeadInt_ =
        snapImmediate ? tgtHpInt : slewScalar(filterHpBeadInt_, tgtHpInt, alphaCurve, 0.002f);

    if (tgtLpInt > 0.001f) {
        const bool snapPos = snapImmediate || (filterLpBeadInt_ <= 0.005f);
        filterLpBeadX_ = snapPos ? tgtLpX : slewScalar(filterLpBeadX_, tgtLpX, alphaCurve, 0.01f);
        filterLpBeadY_ = snapPos ? tgtLpY : slewScalar(filterLpBeadY_, tgtLpY, alphaCurve, 0.01f);
        filterLpBeadRad_ =
            snapPos ? tgtLpRad : slewScalar(filterLpBeadRad_, tgtLpRad, alphaCurve, 0.005f);
    }
    filterLpBeadInt_ =
        snapImmediate ? tgtLpInt : slewScalar(filterLpBeadInt_, tgtLpInt, alphaCurve, 0.002f);
}

void ScopeHudRenderer::updateLfoGraph(float dtClamped, bool snapImmediate,
                                      bool showAllVoices) noexcept {
    std::array<std::array<int16_t, kTracePoints>, kNumOscillators> tgtCurves{};
    std::array<float, kNumOscillators> tgtBeadInt{0.0f, 0.0f, 0.0f, 0.0f};
    bool unified = true;

    constexpr float kInvPts = 1.0f / static_cast<float>(kTracePoints - 1u);
    static constexpr std::array<std::array<int8_t, 5>, kNumOscillators> kFallbackDriftKnots = {{
        {-18, 24, -12, 27, -21},
        {22, -19, 25, -14, 19},
        {-24, 16, -26, 21, -15},
        {17, -25, 14, -22, 26},
    }};

    if (!showAllVoices) {
        const uint8_t v = frameHud_.lfoVoiceFocus & 3u;
        const VoiceParams& vp = frameControl_.voices[v];
        const float d = clampf(vp.lfoDepth, 0.0f, 1.0f);
        const float shapedDepth = d * (1.35f - 0.35f * d);
        const float ampPx = kLfoMaxAmpPx * (0.46f + 0.54f * shapedDepth);
        const std::array<int8_t, 5> knots = (frameLfoDriftPacked_[v] != 0u) ?
                                                unpackDriftKnots(frameLfoDriftPacked_[v]) :
                                                kFallbackDriftKnots[v];

        for (size_t i = 0u; i < kTracePoints; ++i) {
            const float u = static_cast<float>(i) * kInvPts;
            const float w = evalMorphedLfoWave(u, vp.lfoShape, vp.lfoRateHz, knots);
            const float yMain = clampf(kLfoYZero - ampPx * w, kLfoYTop, kLfoYBot);
            const auto qMain = static_cast<int16_t>(yMain * 64.0f + 0.5f);
            tgtCurves[0][i] = qMain;
            tgtCurves[1][i] = qMain;
            tgtCurves[2][i] = qMain;
            tgtCurves[3][i] = qMain;
        }
        tgtBeadInt[v] = 0.88f + 0.12f * smoothstep01(d * 3.0f);
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const VoiceParams& vp = frameControl_.voices[v];
            const float d = clampf(vp.lfoDepth, 0.0f, 1.0f);
            const float shapedDepth = d * (1.35f - 0.35f * d);
            const float ampPx = kLfoMaxAmpPx * (0.46f + 0.54f * shapedDepth);
            const std::array<int8_t, 5> knots = (frameLfoDriftPacked_[v] != 0u) ?
                                                    unpackDriftKnots(frameLfoDriftPacked_[v]) :
                                                    kFallbackDriftKnots[v];
            for (size_t i = 0u; i < kTracePoints; ++i) {
                const float u = static_cast<float>(i) * kInvPts;
                const float w = evalMorphedLfoWave(u, vp.lfoShape, vp.lfoRateHz, knots);
                const float yMain = clampf(kLfoYZero - ampPx * w, kLfoYTop, kLfoYBot);
                tgtCurves[v][i] = static_cast<int16_t>(yMain * 64.0f + 0.5f);
            }
            tgtBeadInt[v] = 0.76f + 0.16f * smoothstep01(d * 3.0f);
        }
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            for (size_t i = 0u; i < kTracePoints; i += 8u) {
                if (tgtCurves[v][i] - tgtCurves[0][i] > 16 ||
                    tgtCurves[0][i] - tgtCurves[v][i] > 16) {
                    unified = false;
                    break;
                }
            }
            if (!unified) {
                break;
            }
        }
    }

    const float alphaCurve =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));
    const float alphaDiv =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.055f)));

    lfoDivergenceSmooth_ =
        slewScalar(lfoDivergenceSmooth_, unified ? 0.0f : 1.0f, alphaDiv, 0.001f);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        for (size_t i = 0u; i < kTracePoints; ++i) {
            lfoCurvesFixed_[v][i] =
                slewQ8_6(lfoCurvesFixed_[v][i], tgtCurves[v][i], alphaCurve, snapImmediate);
        }
        lfoBeadIntensity_[v] = snapImmediate ?
                                   tgtBeadInt[v] :
                                   slewScalar(lfoBeadIntensity_[v], tgtBeadInt[v], alphaCurve,
                                              0.002f);
    }
}

void ScopeHudRenderer::updateHudGauges(float dtClamped, bool snapImmediate) noexcept {
    const float alphaGauge =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.045f)));

    // 1. Top header bar norm (MasterLevel, RailCoupling, SnapshotMorph, VoicePressure)
    bool hasHeaderBar = false;
    float tgtHeaderBar = headerBarSmooth_;
    switch (frameHud_.mode) {
    case HudMode::MasterLevel:
        tgtHeaderBar = clampf(frameControl_.global.masterTrim, 0.0f, 1.0f);
        hasHeaderBar = true;
        break;
    case HudMode::RailCoupling:
        tgtHeaderBar = clampf(frameControl_.global.circuitCoupling, 0.0f, 1.0f);
        hasHeaderBar = true;
        break;
    case HudMode::SnapshotMorph:
        tgtHeaderBar = clampf(frameHud_.morphPosition, 0.0f, 1.0f);
        hasHeaderBar = true;
        break;
    case HudMode::VoicePressure:
        tgtHeaderBar = clampf(frameControl_.voicePressure[frameHud_.focusedVoice & 3u], 0.0f, 1.0f);
        hasHeaderBar = true;
        break;
    default:
        break;
    }
    if (hasHeaderBar) {
        const bool snapHeader = snapImmediate || (prevHeaderBarMode_ != frameHud_.mode);
        headerBarSmooth_ =
            snapHeader ? tgtHeaderBar : slewScalar(headerBarSmooth_, tgtHeaderBar, alphaGauge);
        prevHeaderBarMode_ = frameHud_.mode;
    } else {
        prevHeaderBarMode_ = HudMode::None;
    }

    // 2. Bottom 4-knob physical bead coordinates ([-1..+1]), dim factor, and focus highlight
    const float panScale = frameControl_.global.monoCollapse ? 0.0f : 1.0f;
    const float pitchMode = frameControl_.global.pitchPressureMode ? 1.0f : 0.0f;

    float filterShowHp = 20.0f;
    float filterShowHpRes = 0.0f;
    float filterShowLp = 20000.0f;
    float filterShowLpRes = 0.0f;
    if (frameHud_.knobBank == KnobBankMode::GlobalFilter) {
        float sumLogHp = 0.0f;
        float sumLogLp = 0.0f;
        for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
            const VoiceParams& vp = frameControl_.voices[vi];
            const ModulatedFilterParams mod = computeLiveVoiceFilter(
                vp, frameControl_.voicePressure[vi], pitchMode, frameLfoOutput_[vi]);
            sumLogHp += fastLog2(mod.hpCutoffHz);
            filterShowHpRes += mod.hpResonance;
            sumLogLp += fastLog2(mod.lpCutoffHz);
            filterShowLpRes += mod.lpResonance;
        }
        filterShowHp = fastExp2(sumLogHp * 0.25f);
        filterShowHpRes *= 0.25f;
        filterShowLp = fastExp2(sumLogLp * 0.25f);
        filterShowLpRes *= 0.25f;
    } else if (frameHud_.knobBank == KnobBankMode::VoiceFilter) {
        const uint8_t refV = frameHud_.focusedVoice & 3u;
        const VoiceParams& vp = frameControl_.voices[refV];
        const ModulatedFilterParams mod = computeLiveVoiceFilter(
            vp, frameControl_.voicePressure[refV], pitchMode, frameLfoOutput_[refV]);
        filterShowHp = mod.hpCutoffHz;
        filterShowHpRes = mod.hpResonance;
        filterShowLp = mod.lpCutoffHz;
        filterShowLpRes = mod.lpResonance;
    }

    for (uint8_t k = 0u; k < 4u; ++k) {
        const bool isFocused =
            (frameHud_.focusedKnob == k) &&
            (frameHud_.mode == HudMode::VoiceFrequency || frameHud_.mode == HudMode::Amplitude ||
             frameHud_.mode == HudMode::Panning || frameHud_.mode == HudMode::VoiceFilter ||
             frameHud_.mode == HudMode::GlobalFilter || frameHud_.mode == HudMode::VoiceGate ||
             frameHud_.mode == HudMode::VoicePressure || frameHud_.mode == HudMode::VoiceLfo ||
             frameHud_.mode == HudMode::GlobalLfo);

        float tgtSigned = 0.0f;
        bool dimmed = false;
        switch (frameHud_.knobBank) {
        case KnobBankMode::Frequency: {
            const VoiceParams& vp = frameControl_.voices[k];
            const float pitchMul =
                computeVoicePitchRatio(frameControl_.voicePressure[k], pitchMode);
            const float lfoHz =
                (vp.lfoTarget == LfoTarget::Pitch) ?
                    applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, frameLfoOutput_[k]) :
                    vp.frequencyHz;
            const float liveHz = lfoHz * pitchMul;
            const float norm =
                clampf((fastLog2(liveHz) - fastLog2(kAudioFreqMinHz)) /
                           (fastLog2(kAudioFreqMaxHz) - fastLog2(kAudioFreqMinHz)),
                       0.0f, 1.0f);
            tgtSigned = 2.0f * norm - 1.0f;
            dimmed = vp.muted && (frameControl_.voicePressure[k] <= 0.001f);
            break;
        }
        case KnobBankMode::Amplitude: {
            const VoiceParams& vp = frameControl_.voices[k];
            const float liveAmp =
                (vp.lfoTarget == LfoTarget::Amplitude) ?
                    applyLfoAmplitude(vp.amplitude, vp.lfoDepth, frameLfoOutput_[k]) :
                    vp.amplitude;
            const float norm = vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
            tgtSigned = 2.0f * norm - 1.0f;
            dimmed = vp.muted;
            break;
        }
        case KnobBankMode::Panning: {
            const VoiceParams& vp = frameControl_.voices[k];
            const float livePan =
                (vp.lfoTarget == LfoTarget::Pan) ?
                    applyLfoPan(vp.pan, vp.lfoDepth, frameLfoOutput_[k]) :
                    vp.pan;
            tgtSigned = clampf(livePan * panScale, -1.0f, 1.0f);
            dimmed = vp.muted;
            break;
        }
        case KnobBankMode::VoiceFilter:
        case KnobBankMode::GlobalFilter: {
            float norm = 0.0f;
            if (k == 0u) {
                norm = clampf((fastLog2(filterShowHp) - kFilterLogMin) * (1.0f / kFilterLogSpan),
                              0.0f, 1.0f);
            } else if (k == 1u) {
                norm = clampf(filterShowHpRes, 0.0f, 1.0f);
            } else if (k == 2u) {
                norm = clampf((fastLog2(filterShowLp) - kFilterLogMin) * (1.0f / kFilterLogSpan),
                              0.0f, 1.0f);
            } else {
                norm = clampf(filterShowLpRes, 0.0f, 1.0f);
            }
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        case KnobBankMode::Lfo: {
            float norm = 0.0f;
            if (frameHud_.lfoVoiceFocus >= kNumOscillators) {
                if (k == 0u) {
                    const uint8_t tgtIdx =
                        static_cast<uint8_t>(frameControl_.voices[0].lfoTarget);
                    norm = static_cast<float>(tgtIdx) * (1.0f / 6.0f);
                } else if (k == 1u) {
                    float sumShape = 0.0f;
                    for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                        sumShape += frameControl_.voices[vi].lfoShape;
                    }
                    norm = clampf(sumShape * 0.25f, 0.0f, 1.0f);
                } else if (k == 2u) {
                    float sumLogRate = 0.0f;
                    for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                        sumLogRate += fastLog2(frameControl_.voices[vi].lfoRateHz);
                    }
                    norm = clampf((sumLogRate * 0.25f - kLfoLogMin) * (1.0f / kLfoLogSpan), 0.0f,
                                  1.0f);
                } else {
                    float sumDepth = 0.0f;
                    for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                        sumDepth += frameControl_.voices[vi].lfoDepth;
                    }
                    norm = clampf(sumDepth * 0.25f, 0.0f, 1.0f);
                }
            } else {
                const VoiceParams& vp = frameControl_.voices[frameHud_.lfoVoiceFocus & 3u];
                if (k == 0u) {
                    norm = static_cast<float>(static_cast<uint8_t>(vp.lfoTarget)) * (1.0f / 6.0f);
                } else if (k == 1u) {
                    norm = clampf(vp.lfoShape, 0.0f, 1.0f);
                } else if (k == 2u) {
                    norm = clampf((fastLog2(vp.lfoRateHz) - kLfoLogMin) * (1.0f / kLfoLogSpan),
                                  0.0f, 1.0f);
                } else {
                    norm = clampf(vp.lfoDepth, 0.0f, 1.0f);
                }
            }
            tgtSigned = 2.0f * norm - 1.0f;
            break;
        }
        }

        knobPosSignedSmooth_[k] = slewScalar(knobPosSignedSmooth_[k], tgtSigned, alphaGauge);
        knobDimSmooth_[k] = slewScalar(knobDimSmooth_[k], dimmed ? 1.0f : 0.0f, alphaGauge);
        knobFocusSmooth_[k] = slewScalar(knobFocusSmooth_[k], isFocused ? 1.0f : 0.0f, alphaGauge);
    }
}

void ScopeHudRenderer::prepareFrame(float dtSec, const EngineControlState& control,
                                    const HudState& hud,
                                    const EngineTelemetry& telemetry) noexcept {
    frameControl_ = control;
    frameHud_.scopeFrozen = hud.scopeFrozen;

    const float dtClamped = clampf(dtSec, 0.001f, 0.250f);
    if (!hud.scopeFrozen) {
        timeSinceScopePushSec_ += dtClamped;
        if (timeSinceScopePushSec_ > 10.0f) {
            timeSinceScopePushSec_ = 10.0f;
        }
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            frameLfoOutput_[v] = telemetry.lfoOutput[v].load(std::memory_order_relaxed);
            frameLfoPhase_[v] =
                wrapLfoPhase01(telemetry.lfoPhase[v].load(std::memory_order_relaxed));
            frameLfoDriftPacked_[v] = telemetry.lfoDriftPacked[v].load(std::memory_order_relaxed);
        }
    }

    const bool streamStalled = (!hud.scopeFrozen) && (timeSinceScopePushSec_ > 0.065f);
    const float pL = streamStalled ? 0.0f : telemetry.peakLeft.load(std::memory_order_relaxed);
    const float pR = streamStalled ? 0.0f : telemetry.peakRight.load(std::memory_order_relaxed);
    const float alphaPeakAtk = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.028f));
    const float alphaPeakRel = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.155f));
    smoothedPeakL_ =
        slewScalar(smoothedPeakL_, pL, (pL >= smoothedPeakL_) ? alphaPeakAtk : alphaPeakRel);
    smoothedPeakR_ =
        slewScalar(smoothedPeakR_, pR, (pR >= smoothedPeakR_) ? alphaPeakAtk : alphaPeakRel);

    buildWaveformTrace(dtSec, control);

    bool freqChanged = false;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (absf(control.voices[v].frequencyHz - prevVoiceFreqHz_[v]) > 1.0e-4f) {
            freqChanged = true;
        }
        prevVoiceFreqHz_[v] = control.voices[v].frequencyHz;
    }

    // Immediate clean-slate path for idle frames or synthetic frozen-scope A/B snapshots
    if (hud.mode == HudMode::None && hud.opacity <= 0.001f) {
        releaseLatchActive_ = false;
        prevRawHud_ = hud;
        if (hud.scopeFrozen || (renderedHudOpacity_ <= 0.004f && filterGraphPhase_ <= 0.004f &&
                                filterBackdropDim_ <= 0.004f)) {
            frameHud_ = hud;
            renderedHudOpacity_ = 0.0f;
            filterGraphPhase_ = 0.0f;
            filterBackdropDim_ = 0.0f;
            hudGaugesPrimed_ = false;
            return;
        }

        // Live HOME dismissal: brisk, smooth ~150 ms dissolve (zero 1-frame pop)
        const float homeDecay = fastExpNeg(dtClamped * (1.0f / 0.065f));
        renderedHudOpacity_ *= homeDecay;
        filterGraphPhase_ *= homeDecay;
        filterBackdropDim_ *= homeDecay;
        if (renderedHudOpacity_ < 0.004f) {
            renderedHudOpacity_ = 0.0f;
            frameHud_.mode = HudMode::None;
            hudGaugesPrimed_ = false;
        }
        if (filterGraphPhase_ < 0.004f) {
            filterGraphPhase_ = 0.0f;
        }
        if (filterBackdropDim_ < 0.004f) {
            filterBackdropDim_ = 0.0f;
        }
        frameHud_.opacity = renderedHudOpacity_;
        return;
    }

    // Latch outgoing momentary HUD view during passive release dissolve so labels/gauges
    // never flash to FREQUENCY for ~850 ms when releasing OP1-6, GLO, OCT-/+, or ScopeFreeze
    const bool prevWasMomentary =
        prevRawHud_.modifierHeld || prevRawHud_.mode == HudMode::ScopeFreeze ||
        prevRawHud_.mode == HudMode::VoiceFilter || prevRawHud_.mode == HudMode::GlobalFilter ||
        prevRawHud_.mode == HudMode::Amplitude || prevRawHud_.mode == HudMode::Panning ||
        prevRawHud_.mode == HudMode::KnobRate || prevRawHud_.mode == HudMode::VoicePressure ||
        prevRawHud_.mode == HudMode::VoiceLfo || prevRawHud_.mode == HudMode::GlobalLfo;
    const bool isPassiveFallback = !hud.modifierHeld && (hud.mode == HudMode::VoiceFrequency) &&
                                   (hud.knobBank == KnobBankMode::Frequency) && !freqChanged &&
                                   (hud.opacity > 0.004f) &&
                                   (hud.opacity <= prevRawHud_.opacity + 0.002f);

    if (!releaseLatchActive_ && prevWasMomentary && prevRawHud_.opacity > 0.02f &&
        isPassiveFallback) {
        releaseLatchActive_ = true;
        releaseLatchHud_ = prevRawHud_;
    } else if (releaseLatchActive_ && !isPassiveFallback) {
        releaseLatchActive_ = false;
    }

    HudState effectiveHud = hud;
    if (releaseLatchActive_) {
        effectiveHud.mode = releaseLatchHud_.mode;
        effectiveHud.knobBank = releaseLatchHud_.knobBank;
        effectiveHud.focusedVoice = releaseLatchHud_.focusedVoice;
        effectiveHud.focusedKnob = releaseLatchHud_.focusedKnob;
        effectiveHud.lfoVoiceFocus = releaseLatchHud_.lfoVoiceFocus;
        effectiveHud.lfoPageOpen = releaseLatchHud_.lfoPageOpen;
        effectiveHud.knobRate = releaseLatchHud_.knobRate;
    }

    frameHud_ = effectiveHud;
    renderedHudOpacity_ = effectiveHud.opacity;
    prevRawHud_ = hud;

    const bool snapGauges = !hudGaugesPrimed_;
    hudGaugesPrimed_ = (renderedHudOpacity_ > 0.004f);
    if (hudGaugesPrimed_) {
        updateHudGauges(dtClamped, snapGauges);
    }

    const bool rawFilterActive =
        (hud.opacity > 0.004f) && (hud.mode != HudMode::None) &&
        (hud.knobBank == KnobBankMode::VoiceFilter ||
         hud.knobBank == KnobBankMode::GlobalFilter ||
         (hud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode));
    const bool rawLfoGraphActive =
        !rawFilterActive && (hud.opacity > 0.004f) && (hud.mode != HudMode::None) &&
        (hud.mode == HudMode::VoiceLfo || hud.mode == HudMode::GlobalLfo ||
         hud.knobBank == KnobBankMode::Lfo);

    if (rawFilterActive) {
        const bool snapFilter = (filterGraphPhase_ <= 0.015f);
        filterGraphPhase_ +=
            (1.0f - filterGraphPhase_) * (1.0f - fastExpNeg(dtClamped * (1.0f / 0.090f)));

        const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
        std::array<ModulatedFilterParams, kNumOscillators> mods{};
        bool unified = true;

        if (hud.knobBank == KnobBankMode::VoiceFilter ||
            (hud.mode == HudMode::VoicePressure && !control.global.pitchPressureMode)) {
            const uint8_t v = hud.focusedVoice & 3u;
            const VoiceParams& vp = control.voices[v];
            const ModulatedFilterParams mod =
                computeLiveVoiceFilter(vp, control.voicePressure[v], pitchMode, frameLfoOutput_[v]);
            mods = {mod, mod, mod, mod};
            unified = true;
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const VoiceParams& vp = control.voices[i];
                mods[i] = computeLiveVoiceFilter(vp, control.voicePressure[i], pitchMode,
                                                 frameLfoOutput_[i]);
            }
            for (size_t i = 1u; i < kNumOscillators; ++i) {
                if (absf(mods[i].hpCutoffHz - mods[0].hpCutoffHz) >
                        0.005f * mods[0].hpCutoffHz + 0.05f ||
                    absf(mods[i].lpCutoffHz - mods[0].lpCutoffHz) >
                        0.005f * mods[0].lpCutoffHz + 0.05f ||
                    absf(mods[i].hpResonance - mods[0].hpResonance) > 0.005f ||
                    absf(mods[i].lpResonance - mods[0].lpResonance) > 0.005f) {
                    unified = false;
                    break;
                }
            }
        }
        updateFilterGraph(dtClamped, snapFilter, mods, unified);
    } else {
        filterGraphPhase_ *= fastExpNeg(dtClamped * (1.0f / 0.180f));
        if (filterGraphPhase_ < 0.003f) {
            filterGraphPhase_ = 0.0f;
        }
    }

    if (rawLfoGraphActive) {
        const bool snapLfo = (lfoGraphPhase_ <= 0.015f);
        lfoGraphPhase_ +=
            (1.0f - lfoGraphPhase_) * (1.0f - fastExpNeg(dtClamped * (1.0f / 0.090f)));
        updateLfoGraph(dtClamped, snapLfo, hud.lfoVoiceFocus >= kNumOscillators);
    } else {
        lfoGraphPhase_ *= fastExpNeg(dtClamped * (1.0f / 0.180f));
        if (lfoGraphPhase_ < 0.003f) {
            lfoGraphPhase_ = 0.0f;
        }
    }

    if (rawFilterActive || rawLfoGraphActive) {
        filterBackdropDim_ +=
            (1.0f - filterBackdropDim_) * (1.0f - fastExpNeg(dtClamped * (1.0f / 0.110f)));
    } else {
        filterBackdropDim_ *= fastExpNeg(dtClamped * (1.0f / 0.240f));
        if (filterBackdropDim_ < 0.003f) {
            filterBackdropDim_ = 0.0f;
        }
    }
}

void ScopeHudRenderer::renderStrip(uint16_t yStart, uint16_t stripHeight,
                                   uint16_t* rgb565Out) const noexcept {
    if (rgb565Out == nullptr || yStart >= kDisplayHeight || stripHeight == 0u) {
        return;
    }

    const uint16_t totalEnd = (yStart + stripHeight > kDisplayHeight) ?
                                  kDisplayHeight :
                                  static_cast<uint16_t>(yStart + stripHeight);

    // Smooth 70% background waveform dimming strictly when Filter Graph is active/dissolving
    const float bgDim01 = smoothstep01(clampf(filterBackdropDim_, 0.0f, 1.0f));
    const uint32_t bgKeepQ8 =
        static_cast<uint32_t>((1.0f - kFilterBgDimDepth * bgDim01) * 256.0f + 0.5f);

    // Process in <= 8-row sub-chunks so stack photon + luminance buffers stay 5,760 bytes total
    // while keeping strip heights 8, 16, and 24 100% bit-for-bit identical.
    constexpr uint16_t kSubStripRows = 8u;
    std::array<uint16_t, kDisplayWidth * kSubStripRows> linBuf{};
    std::array<uint8_t, kDisplayWidth * kSubStripRows> lumBuf{};
    uint16_t curY = yStart;

    while (curY < totalEnd) {
        const uint16_t chunkH = (totalEnd - curY > kSubStripRows) ?
                                    kSubStripRows :
                                    static_cast<uint16_t>(totalEnd - curY);
        const int y0 = static_cast<int>(curY);
        const int y1 = y0 + static_cast<int>(chunkH);

        // 1. Clear linear photon accumulator for pitch-black #000000 canvas
        std::memset(linBuf.data(), 0,
                    static_cast<size_t>(chunkH) * kDisplayWidth * sizeof(uint16_t));

        // 2. Tile-culled 14-strand open-edge caustic capsule rasterization
        if (scopeValid_) {
            constexpr float kFixedToPx = 1.0f / 64.0f;
            for (size_t s = 0u; s < kNumStrands; ++s) {
                const uint32_t wgtQ8 = strandWeightQ8_[s];
                if (wgtQ8 == 0u) {
                    continue;
                }
                const float wgt = static_cast<float>(wgtQ8) * (1.0f / 256.0f);
                const auto& yFixed = strandYFixed_[s];
                const auto& tMin = strandTileMinY_[s];
                const auto& tMax = strandTileMaxY_[s];

                for (size_t tile = 0u; tile < kNumTiles; ++tile) {
                    if (static_cast<int>(tMax[tile]) < y0 || static_cast<int>(tMin[tile]) >= y1) {
                        continue;
                    }
                    const size_t colBase = tile * kTileCols;
                    for (size_t c = 0u; c < kTileCols; ++c) {
                        const size_t colX = colBase + c;
                        const float yA = static_cast<float>(yFixed[colX]) * kFixedToPx;
                        const float yB = static_cast<float>(yFixed[colX + 1u]) * kFixedToPx;
                        const int segTop = static_cast<int>(((yA < yB) ? yA : yB) + 512.0f) - 515;
                        const int segBot = static_cast<int>(((yA > yB) ? yA : yB) + 512.0f) - 508;
                        if (segBot <= y0 || segTop >= y1) {
                            continue;
                        }

                        const float yMid = 0.5f * (yA + yB);
                        const float dy = yB - yA;
                        const float invL2 = 1.0f / (1.0f + dy * dy);
                        const float dwell =
                            (0.58f + 0.42f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * wgt;
                        const int pyS = (segTop < y0) ? y0 : segTop;
                        const int pyE = (segBot > y1) ? y1 : segBot;

                        for (int py = pyS; py < pyE; ++py) {
                            const float vy = static_cast<float>(py) - yMid;
                            const float segT = clampf(vy * dy * invL2, -0.5f, 0.5f);
                            const float ey = vy - dy * segT;
                            const float d2 = segT * segT + ey * ey;
                            if (d2 < 3.95f) {
                                const size_t lutIdx = static_cast<size_t>(d2 * 16.0f);
                                const uint32_t add = static_cast<uint32_t>(
                                    static_cast<float>(kGaussLut[lutIdx]) * dwell);
                                uint16_t& cell =
                                    linBuf[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
                                const uint32_t next = static_cast<uint32_t>(cell) + add;
                                cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                            }
                        }
                    }
                }
            }
        }

        // 3. Exact eighth-step bilinear 60x60 bloom upsample + soft-knee Reinhard tonemap
        //    + smooth 70% background dim transition behind active filter graphs
        for (uint16_t rowIdx = 0u; rowIdx < chunkH; ++rowIdx) {
            const int py = y0 + static_cast<int>(rowIdx);
            const int numY8 = 2 * py - 3;
            size_t by0 = 0u;
            size_t by1 = 0u;
            uint32_t wy8 = 0u;
            if (numY8 > 0) {
                const uint32_t uy8 = static_cast<uint32_t>(numY8);
                by0 = uy8 >> 3u;
                if (by0 >= kBloomHeight) {
                    by0 = kBloomHeight - 1u;
                }
                by1 = (by0 + 1u < kBloomHeight) ? (by0 + 1u) : (kBloomHeight - 1u);
                wy8 = uy8 & 7u;
            }
            const uint32_t invWy8 = 8u - wy8;
            const uint16_t* bRow0 = bloomGrid_.data() + by0 * kBloomWidth;
            const uint16_t* bRow1 = bloomGrid_.data() + by1 * kBloomWidth;
            const uint16_t* linRow = linBuf.data() + static_cast<size_t>(rowIdx) * kDisplayWidth;
            uint8_t* lumRow = lumBuf.data() + static_cast<size_t>(rowIdx) * kDisplayWidth;

            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                const int numX8 = 2 * static_cast<int>(colX) - 3;
                size_t bx0 = 0u;
                size_t bx1 = 0u;
                uint32_t wx8 = 0u;
                if (numX8 > 0) {
                    const uint32_t ux8 = static_cast<uint32_t>(numX8);
                    bx0 = ux8 >> 3u;
                    if (bx0 >= kBloomWidth) {
                        bx0 = kBloomWidth - 1u;
                    }
                    bx1 = (bx0 + 1u < kBloomWidth) ? (bx0 + 1u) : (kBloomWidth - 1u);
                    wx8 = ux8 & 7u;
                }
                const uint32_t invWx8 = 8u - wx8;
                const uint32_t bTop = static_cast<uint32_t>(bRow0[bx0]) * invWx8 +
                                      static_cast<uint32_t>(bRow0[bx1]) * wx8;
                const uint32_t bBot = static_cast<uint32_t>(bRow1[bx0]) * invWx8 +
                                      static_cast<uint32_t>(bRow1[bx1]) * wx8;
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

        // 4. Idle top-right stereo peak optical-bead filaments when HUD is inactive/fading
        const uint8_t hudAlpha8 =
            static_cast<uint8_t>(clampf(frameHud_.opacity, 0.0f, 1.0f) * 255.0f + 0.5f);
        if (hudAlpha8 < 255u && y0 < 18 && y1 > 4) {
            const uint8_t idleA = static_cast<uint8_t>(255u - hudAlpha8);
            const MonoStripCanvas idleCv{lumBuf.data(), y0, y1, idleA};
            const float nl = clampf(smoothedPeakL_ * (1.0f / 1.25f), 0.0f, 1.0f);
            const float nr = clampf(smoothedPeakR_ * (1.0f / 1.25f), 0.0f, 1.0f);
            const float actL = smoothstep01(smoothedPeakL_ * (1.0f / 0.055f));
            const float actR = smoothstep01(smoothedPeakR_ * (1.0f / 0.055f));
            const float hotL = smoothstep01((smoothedPeakL_ - 0.94f) * (1.0f / 0.16f));
            const float hotR = smoothstep01((smoothedPeakR_ - 0.94f) * (1.0f / 0.16f));

            idleCv.drawHorizontalFilament(202.0f, 230.0f, 8.8f, 22.0f, 0.55f, true);
            idleCv.drawHorizontalFilament(202.0f, 230.0f, 12.8f, 22.0f, 0.55f, true);
            if (nl > 0.004f) {
                idleCv.drawHorizontalFilament(202.0f, 202.0f + nl * 28.0f, 8.8f,
                                              (168.0f + 77.0f * hotL) * actL, 0.62f, false);
            }
            if (nr > 0.004f) {
                idleCv.drawHorizontalFilament(202.0f, 202.0f + nr * 28.0f, 12.8f,
                                              (168.0f + 77.0f * hotR) * actR, 0.62f, false);
            }
            const float beadIntL = 0.22f + (0.50f + 0.28f * hotL) * actL;
            const float beadIntR = 0.22f + (0.50f + 0.28f * hotR) * actR;
            idleCv.drawOpticalBead(202.0f + nl * 28.0f, 8.8f, beadIntL, 1.65f);
            idleCv.drawOpticalBead(202.0f + nr * 28.0f, 12.8f, beadIntR, 1.65f);
        }

        // 5. Divider-free soft-feathered monochrome sans-serif + Concept A3 optical HUD
        if (hudAlpha8 > 0u && frameHud_.mode != HudMode::None) {
            compositeHudStrip(curY, chunkH, lumBuf.data(), linBuf.data(), hudAlpha8);
        }

        // 6. Pack 8-bit monochrome strip into dithered neutral-gray RGB565
        uint16_t* dstBase = rgb565Out + static_cast<size_t>(curY - yStart) * kDisplayWidth;
        for (uint16_t r = 0u; r < chunkH; ++r) {
            const int py = y0 + static_cast<int>(r);
            const uint8_t* srcRow = lumBuf.data() + static_cast<size_t>(r) * kDisplayWidth;
            uint16_t* dstRow = dstBase + static_cast<size_t>(r) * kDisplayWidth;
            for (size_t x = 0u; x < kDisplayWidth; ++x) {
                dstRow[x] = packNeutralGrayRgb565(srcRow[x], static_cast<int>(x), py);
            }
        }

        curY = static_cast<uint16_t>(curY + chunkH);
    }
}

void ScopeHudRenderer::compositeHudStrip(uint16_t yStart, uint16_t stripHeight, uint8_t* lumBuf,
                                         uint16_t* linScratch, uint8_t hudAlpha8) const noexcept {
    const int y0 = static_cast<int>(yStart);
    const int y1 = y0 + static_cast<int>(stripHeight);
    const MonoStripCanvas cv{lumBuf, y0, y1, hudAlpha8};

    const float hudAlpha01 = static_cast<float>(hudAlpha8) * (1.0f / 255.0f);
    const float filterEnv01 = smoothstep01(clampf(filterGraphPhase_, 0.0f, 1.0f));
    const float effFilterAlpha01 = filterEnv01 * hudAlpha01;
    const auto filterAlpha8 = static_cast<uint8_t>(effFilterAlpha01 * 255.0f + 0.5f);
    const float lfoEnv01 = smoothstep01(clampf(lfoGraphPhase_, 0.0f, 1.0f));
    const float effLfoAlpha01 = lfoEnv01 * hudAlpha01;
    const auto lfoAlpha8 = static_cast<uint8_t>(effLfoAlpha01 * 255.0f + 0.5f);

    // Soft feathered black vignettes (zero hard borders, zero divider lines, wave never compressed)
    cv.applyVerticalScrim(0, 34, 0.80f, true);
    const float graphEnvMax = (filterEnv01 > lfoEnv01) ? filterEnv01 : lfoEnv01;
    const int botScrimTop = 178 - static_cast<int>(8.0f * graphEnvMax + 0.5f);
    cv.applyVerticalScrim(botScrimTop, 240, 0.90f, false);

    // ========================================================================
    // LFO. Full-Screen Edge-to-Edge Bipolar Wave (x = 0..239, y = 26..188) + Phase Beacon
    // ========================================================================
    if (lfoAlpha8 > 0u && y1 > 12 && y0 < 206) {
        constexpr float kFixedToPx = 1.0f / 64.0f;
        const MonoStripCanvas lfoCv{lumBuf, y0, y1, lfoAlpha8};
        const float div = clampf(lfoDivergenceSmooth_, 0.0f, 1.0f);
        const bool multiCurves = (div > 0.001f);
        const size_t curveCount = multiCurves ? kNumOscillators : 1u;

        float maxD = 0.0f;
        if (frameHud_.lfoVoiceFocus >= kNumOscillators) {
            for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                if (frameControl_.voices[vi].lfoDepth > maxD) {
                    maxD = frameControl_.voices[vi].lfoDepth;
                }
            }
        } else {
            maxD = frameControl_.voices[frameHud_.lfoVoiceFocus & 3u].lfoDepth;
        }
        const float lfoEnergy = smoothstep01(clampf(maxD, 0.0f, 1.0f) * 3.0f);
        const float spineModeScale = 0.58f + 0.42f * lfoEnergy;
        const float fillModeScale = 0.34f + 0.66f * lfoEnergy;

        // 1. Edge-to-edge bipolar zero-crossing horizon filament (y = 107.0)
        lfoCv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), kLfoYZero,
                                     38.0f, 0.50f, false);

        // 2. Full-height bipolar caustic veil between LFO spine and zero-line (zero ghost contours,
        //    4x sub-column box-integrated on steep Sawtooth/Square edges + 2D crest glow)
        const float fillScale = (multiCurves ? (0.24f + 0.05f * div) : 1.0f) * fillModeScale;
        if (y1 > static_cast<int>(kLfoYTop) - 1 && y0 <= static_cast<int>(kLfoYBot) + 1) {
            for (size_t vc = 0u; vc < curveCount; ++vc) {
                const auto& pts = lfoCurvesFixed_[vc];
                for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                    const float yA = static_cast<float>(pts[colX]) * kFixedToPx;
                    const float yB = static_cast<float>(pts[colX + 1u]) * kFixedToPx;
                    const float dy = yB - yA;
                    const float absDy = absf(dy);
                    const float yMid = 0.5f * (yA + yB);

                    float envLo = (yA < yB) ? yA : yB;
                    if (kLfoYZero < envLo) {
                        envLo = kLfoYZero;
                    }
                    float envHi = (yA > yB) ? yA : yB;
                    if (kLfoYZero > envHi) {
                        envHi = kLfoYZero;
                    }
                    if (envHi - envLo <= 0.08f) {
                        continue;
                    }

                    int pyStart = static_cast<int>(envLo - 0.5f);
                    if (pyStart < static_cast<int>(kLfoYTop) - 1) {
                        pyStart = static_cast<int>(kLfoYTop) - 1;
                    }
                    if (pyStart < y0) {
                        pyStart = y0;
                    }
                    int pyEnd = static_cast<int>(envHi + 0.5f) + 1;
                    if (pyEnd > static_cast<int>(kLfoYBot) + 1) {
                        pyEnd = static_cast<int>(kLfoYBot) + 1;
                    }
                    if (pyEnd > y1 - 1) {
                        pyEnd = y1 - 1;
                    }
                    if (pyStart > pyEnd) {
                        continue;
                    }

                    const float spanA = absf(kLfoYZero - yA);
                    const float spanB = absf(kLfoYZero - yB);
                    const float maxSpan = (spanA > spanB) ? spanA : spanB;
                    const float crestBoost =
                        1.0f + 0.34f * clampf(maxSpan * (1.0f / kLfoMaxAmpPx), 0.0f, 1.0f);
                    const float colScale = fillScale * effLfoAlpha01;
                    const float glowAmp = 68.0f * crestBoost;
                    const float invL2 = 1.0f / (1.0f + dy * dy);
                    const bool steepSeg = (absDy > 1.5f);

                    for (int py = pyStart; py <= pyEnd; ++py) {
                        const float pyF = static_cast<float>(py);
                        const float boxLo = pyF - 0.5f;
                        const float boxHi = pyF + 0.5f;

                        float subEdge = 0.0f;
                        if (!steepSeg) {
                            const float segLo = (yMid < kLfoYZero) ? yMid : kLfoYZero;
                            const float segHi = (yMid > kLfoYZero) ? yMid : kLfoYZero;
                            const float ovLo = (boxLo > segLo) ? boxLo : segLo;
                            const float ovHi = (boxHi < segHi) ? boxHi : segHi;
                            if (ovHi > ovLo) {
                                subEdge = ovHi - ovLo;
                            }
                        } else {
                            for (int s = 0; s < 4; ++s) {
                                const float u = (static_cast<float>(s) + 0.5f) * 0.25f;
                                const float ySub = yA + u * dy;
                                const float segLo = (ySub < kLfoYZero) ? ySub : kLfoYZero;
                                const float segHi = (ySub > kLfoYZero) ? ySub : kLfoYZero;
                                const float ovLo = (boxLo > segLo) ? boxLo : segLo;
                                const float ovHi = (boxHi < segHi) ? boxHi : segHi;
                                if (ovHi > ovLo) {
                                    subEdge += (ovHi - ovLo) * 0.25f;
                                }
                            }
                        }
                        if (subEdge <= 0.004f) {
                            continue;
                        }

                        const float vy = pyF - yMid;
                        const float segT = clampf(vy * dy * invL2, -0.5f, 0.5f);
                        const float ey = vy - dy * segT;
                        const float d2 = segT * segT + ey * ey;
                        const float perpDist = d2 * fastInvSqrt(d2 + 0.25f);
                        const float crestGlow = glowAmp * fastExpNeg(perpDist * 0.052f);
                        int alpha =
                            static_cast<int>((18.0f + crestGlow) * colScale * subEdge + 0.5f);
                        if (alpha > 215) {
                            alpha = 215;
                        }
                        if (alpha > 0) {
                            uint8_t& dst =
                                lumBuf[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
                            dst = blendLum8(dst, 240u, static_cast<uint8_t>(alpha));
                        }
                    }
                }
            }
        }

        // 3. Linear-photon incandescent LFO wave spine (x = 0..239, zero decorative inner lines)
        if (y1 > static_cast<int>(kLfoYTop) - 4 && y0 <= static_cast<int>(kLfoYBot) + 4) {
            std::memset(linScratch, 0,
                        static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
            auto addLfoPhoton = [&](const std::array<int16_t, kTracePoints>& pts,
                                    float weight) noexcept {
                if (weight <= 0.004f) {
                    return;
                }
                for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                    const float yA = static_cast<float>(pts[colX]) * kFixedToPx;
                    const float yB = static_cast<float>(pts[colX + 1u]) * kFixedToPx;
                    const float yLo = (yA < yB) ? yA : yB;
                    const float yHi = (yA > yB) ? yA : yB;
                    const int segTop = static_cast<int>(yLo) - 3;
                    const int segBot = static_cast<int>(yHi) + 4;
                    if (segBot <= y0 || segTop >= y1) {
                        continue;
                    }
                    const float dy = yB - yA;
                    const float invL2 = 1.0f / (1.0f + dy * dy);
                    const float dwell =
                        (0.64f + 0.36f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * weight;
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
                                static_cast<uint32_t>(static_cast<float>(kGaussLut[lutIdx]) *
                                                      dwell);
                            uint16_t& cell =
                                linScratch[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
                            const uint32_t next = static_cast<uint32_t>(cell) + add;
                            cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                        }
                    }
                }
            };

            const float mainCurveWgt =
                (multiCurves ? (0.28f + 0.28f * div) : 1.04f) * spineModeScale * effLfoAlpha01;
            for (size_t vc = 0u; vc < curveCount; ++vc) {
                addLfoPhoton(lfoCurvesFixed_[vc], mainCurveWgt);
            }

            for (uint16_t r = 0u; r < stripHeight; ++r) {
                const uint16_t* linRow = linScratch + static_cast<size_t>(r) * kDisplayWidth;
                uint8_t* lumRow = lumBuf + static_cast<size_t>(r) * kDisplayWidth;
                for (size_t x = 0u; x < kDisplayWidth; ++x) {
                    const uint16_t e = linRow[x];
                    if (e > 0u) {
                        const uint32_t addLum = kTonemapLut[e >> 2u];
                        const uint32_t bg = lumRow[x];
                        lumRow[x] =
                            static_cast<uint8_t>(255u - (((255u - bg) * (255u - addLum)) >> 8u));
                    }
                }
            }
        }

        // 4. Radiant CRT phosphor phase beacon(s) tracking live LFO period across x = 0..239
        for (size_t bv = 0u; bv < kNumOscillators; ++bv) {
            const float lfoVis = lfoBeadIntensity_[bv];
            if (lfoVis <= 0.005f) {
                continue;
            }
            const float ph = clampf(frameLfoPhase_[bv], 0.0f, 0.999999f);
            const float uIdx = ph * static_cast<float>(kDisplayWidth);
            const size_t i0 =
                (static_cast<size_t>(uIdx) < kDisplayWidth) ? static_cast<size_t>(uIdx) :
                                                              (kDisplayWidth - 1u);
            const size_t i1 = i0 + 1u;
            const float frac = uIdx - static_cast<float>(i0);
            const size_t curveIdx = multiCurves ? bv : 0u;
            const float y0Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i0]) * kFixedToPx;
            const float y1Px = static_cast<float>(lfoCurvesFixed_[curveIdx][i1]) * kFixedToPx;
            const float beadX = clampf(uIdx, 0.0f, static_cast<float>(kDisplayWidth - 1u));
            const float beadY = y0Px + frac * (y1Px - y0Px);
            lfoCv.drawLfoPhaseBeacon(beadX, beadY, lfoVis, frameControl_.voices[bv].lfoDepth);
        }
    }

    // ========================================================================
    // A. Top Function Header (y = 6..22): 13pt Bold Two-Tone Title + Top-Aligned Right Info
    // ========================================================================
    if (y0 < 26) {
        const uint8_t v = frameHud_.focusedVoice & 3u;
        const VoiceParams& focVp = frameControl_.voices[v];
        char leftMain[24]{};
        char leftSuffix[16]{};
        char rightText[24]{};
        bool showHeaderBar = false;
        float headerBarNorm = 0.0f;

        switch (frameHud_.mode) {
        case HudMode::None:
            break;
        case HudMode::VoiceFrequency:
            appendStr(leftMain, 0u, sizeof(leftMain), "FREQUENCY");
            break;
        case HudMode::Amplitude:
            appendStr(leftMain, 0u, sizeof(leftMain), "AMPLITUDE");
            appendStr(rightText, 0u, sizeof(rightText), "POST-FILTER VCA");
            break;
        case HudMode::Panning:
            appendStr(leftMain, 0u, sizeof(leftMain), "PANNING");
            appendStr(rightText, 0u, sizeof(rightText),
                      frameControl_.global.monoCollapse ? "DUAL MONO" : "EQUAL POWER");
            break;
        case HudMode::VoiceFilter: {
            appendStr(leftMain, 0u, sizeof(leftMain), "FILTER");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            appendStr(rightText, 0u, sizeof(rightText), "24 dB/OCT ZDF");
            break;
        }
        case HudMode::GlobalFilter:
            appendStr(leftMain, 0u, sizeof(leftMain), "FILTER");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " GLOBAL");
            appendStr(rightText, 0u, sizeof(rightText), "ALL 4 VOICES");
            break;
        case HudMode::MasterLevel:
            appendStr(leftMain, 0u, sizeof(leftMain), "MASTER LEVEL");
            headerBarNorm = clampf(frameControl_.global.masterTrim, 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        case HudMode::RailCoupling:
            appendStr(leftMain, 0u, sizeof(leftMain), "RAIL COUPLING");
            headerBarNorm = clampf(frameControl_.global.circuitCoupling, 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        case HudMode::UnassignedEncoder:
            appendStr(leftMain, 0u, sizeof(leftMain), "SELECT ENCODER");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        case HudMode::SnapshotMorph:
            appendStr(leftMain, 0u, sizeof(leftMain), "SNAPSHOT MORPH");
            headerBarNorm = clampf(frameHud_.morphPosition, 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        case HudMode::KnobRate: {
            appendStr(leftMain, 0u, sizeof(leftMain), "KNOB RATE");
            const char* rateStr = (frameHud_.knobRate == KnobRate::Fine) ? "FINE" :
                                  (frameHud_.knobRate == KnobRate::Fast) ? "FAST" :
                                                                           "STANDARD";
            appendStr(rightText, 0u, sizeof(rightText), rateStr);
            break;
        }
        case HudMode::VoiceGate: {
            appendStr(leftMain, 0u, sizeof(leftMain), "GATE");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            appendStr(rightText, 0u, sizeof(rightText), focVp.muted ? "MUTED" : "ACTIVE");
            break;
        }
        case HudMode::VoicePressure: {
            if (frameControl_.global.pitchPressureMode) {
                appendStr(leftMain, 0u, sizeof(leftMain), "PITCH BEND");
            } else {
                appendStr(leftMain, 0u, sizeof(leftMain),
                          focVp.muted ? "LPG SWELL" : "FILTER EXP");
            }
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(v + 1u));
            headerBarNorm = clampf(frameControl_.voicePressure[v], 0.0f, 1.0f);
            formatPercent(rightText, sizeof(rightText), headerBarNorm);
            showHeaderBar = true;
            break;
        }
        case HudMode::PressureTarget:
            appendStr(leftMain, 0u, sizeof(leftMain), "PRESSURE TARGET");
            appendStr(rightText, 0u, sizeof(rightText),
                      frameControl_.global.pitchPressureMode ? "PITCH BEND" : "MS-20 FILTER");
            break;
        case HudMode::SnapshotSlot: {
            size_t p = appendStr(leftMain, 0u, sizeof(leftMain), "SNAPSHOT ");
            appendUInt(leftMain, p, sizeof(leftMain),
                       static_cast<uint32_t>((frameHud_.snapshotSlot & 7u) + 1u));
            appendStr(rightText, 0u, sizeof(rightText), "EMPTY SLOT");
            break;
        }
        case HudMode::UnassignedKey:
            appendStr(leftMain, 0u, sizeof(leftMain), "PIT KEY");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        case HudMode::OutputRouting:
            appendStr(leftMain, 0u, sizeof(leftMain), "OUTPUT ROUTING");
            appendStr(rightText, 0u, sizeof(rightText),
                      frameControl_.global.monoCollapse ? "DUAL MONO" : "STEREO");
            break;
        case HudMode::AudioOutput:
            appendStr(leftMain, 0u, sizeof(leftMain), "AUDIO OUTPUT");
            appendStr(rightText, 0u, sizeof(rightText),
                      frameControl_.global.masterRun ? "ACTIVE" : "MUTED");
            break;
        case HudMode::SaveArmed:
            appendStr(leftMain, 0u, sizeof(leftMain), "STORE SNAPSHOT");
            appendStr(rightText, 0u, sizeof(rightText),
                      frameHud_.saveArmed ? "TAP KEY 9-16" : "CANCELLED");
            break;
        case HudMode::ScopeFreeze:
            appendStr(leftMain, 0u, sizeof(leftMain), "SCOPE FREEZE");
            appendStr(rightText, 0u, sizeof(rightText), "HOLDING");
            break;
        case HudMode::VoiceLfo: {
            const uint8_t lv = frameHud_.lfoVoiceFocus & 3u;
            const VoiceParams& lvp = frameControl_.voices[lv];
            appendStr(leftMain, 0u, sizeof(leftMain), "LFO");
            size_t p = appendStr(leftSuffix, 0u, sizeof(leftSuffix), " OSC ");
            appendUInt(leftSuffix, p, sizeof(leftSuffix), static_cast<uint32_t>(lv + 1u));
            appendStr(rightText, 0u, sizeof(rightText),
                      (lvp.lfoDepth > 0.002f) ? "ACTIVE" : "BYPASSED");
            break;
        }
        case HudMode::GlobalLfo:
            appendStr(leftMain, 0u, sizeof(leftMain), "LFO");
            appendStr(leftSuffix, 0u, sizeof(leftSuffix), " GLOBAL");
            appendStr(rightText, 0u, sizeof(rightText), "ALL 4 VOICES");
            break;
        // Remaining right-grid switches (FX, ENV, EDIT, GloRight, ARP, SEQ, REC)
        case HudMode::UnassignedButton: {
            const char* swName = "PANEL";
            switch (frameHud_.lastButton) {
            case ButtonId::Fx: swName = "FX"; break;
            case ButtonId::Env: swName = "ENV"; break;
            case ButtonId::Edit: swName = "EDIT"; break;
            case ButtonId::GloRight: swName = "GLO"; break;
            case ButtonId::Arp: swName = "ARP"; break;
            case ButtonId::Seq: swName = "SEQ"; break;
            case ButtonId::Rec: swName = "REC"; break;
            default: break;
            }
            size_t p = appendStr(leftMain, 0u, sizeof(leftMain), swName);
            appendStr(leftMain, p, sizeof(leftMain), " SWITCH");
            appendStr(rightText, 0u, sizeof(rightText), "UNASSIGNED");
            break;
        }
        }

        cv.drawSans(kFontSansBold, 10, 6, leftMain, 255u, 0);
        if (leftSuffix[0] != '\0') {
            const int suffixX = 10 + measureSansText(kFontSansBold, leftMain, 0);
            cv.drawSans(kFontSansBold, suffixX, 6, leftSuffix, 118u, 0);
        }

        if (showHeaderBar) {
            const int txtW = measureSansText(kFontSansMedium, rightText, 0);
            cv.drawSans(kFontSansMedium, 230 - txtW, 6, rightText, 255u, 0);

            const float barWf = 34.0f;
            const float barXf = static_cast<float>(230 - txtW - 8) - barWf;
            const float barNorm = clampf(headerBarSmooth_, 0.0f, 1.0f);
            cv.drawHorizontalFilament(barXf, barXf + barWf, 13.5f, 42.0f, 0.58f, true);
            if (barNorm > 0.008f) {
                cv.drawHorizontalFilament(barXf, barXf + barNorm * barWf, 13.5f, 225.0f, 0.68f,
                                          false);
            }
            cv.drawOpticalBead(barXf + barNorm * barWf, 13.5f, 0.95f, 2.1f);
        } else if (rightText[0] != '\0') {
            // Top-aligned with kFontSansBold at ty = 6 (both share yOffset = 3 -> cap top y = 9)
            cv.drawSansRight(kFontSansSmall, 230, 6, rightText, 168u, 1);
        }
    }

    // ========================================================================
    // B. Concept A3 1/3-Screen (80 px: y = 102..182) Dual-Resonance MS-20 Graph
    // ========================================================================
    if (filterAlpha8 > 0u && y1 > 94 && y0 < 189) {
        constexpr float kFixedToPx = 1.0f / 64.0f;
        const MonoStripCanvas filterCv{lumBuf, y0, y1, filterAlpha8};
        const float div = clampf(filterDivergenceSmooth_, 0.0f, 1.0f);
        const bool multiCurves = (div > 0.001f);
        const size_t curveCount = multiCurves ? kNumOscillators : 1u;

        // 1. Subtle floor horizon filament (y = 182)
        filterCv.drawHorizontalFilament(
            static_cast<float>(kFilterXStart),
            static_cast<float>(kFilterXStart + static_cast<int>(kFilterPoints) - 1), kFilterYBot,
            28.0f, 0.48f, true);

        // 2. Option A3 Passband Silhouette + Caustic Crest Glow
        const float fillScale = multiCurves ? (0.25f + 0.05f * div) : 1.0f;
        if (y1 > static_cast<int>(kFilterYTop) && y0 <= static_cast<int>(kFilterYBot)) {
            for (size_t vc = 0u; vc < curveCount; ++vc) {
                const auto& pts = filterCurvesFixed_[vc];
                for (size_t i = 0u; i < kFilterPoints; ++i) {
                    const int px = kFilterXStart + static_cast<int>(i);
                    const size_t iNext = (i + 1u < kFilterPoints) ? (i + 1u) : i;
                    const float yCurve =
                        0.5f * (static_cast<float>(pts[i]) + static_cast<float>(pts[iNext])) *
                        kFixedToPx;
                    if (kFilterYBot - yCurve <= 0.35f) {
                        continue;
                    }
                    int pyStart = static_cast<int>(yCurve);
                    if (pyStart < static_cast<int>(kFilterYTop)) {
                        pyStart = static_cast<int>(kFilterYTop);
                    }
                    if (pyStart < y0) {
                        pyStart = y0;
                    }
                    int pyEnd = static_cast<int>(kFilterYBot);
                    if (pyEnd > y1 - 1) {
                        pyEnd = y1 - 1;
                    }
                    if (pyStart > pyEnd) {
                        continue;
                    }

                    const int distL = static_cast<int>(i);
                    const int distR = static_cast<int>(kFilterPoints - 1u - i);
                    const int edgeDist = (distL < distR) ? distL : distR;
                    const float edgeTaper =
                        0.65f +
                        0.35f * smoothstep01(static_cast<float>(edgeDist) * (1.0f / 6.0f));
                    const float hornBoost =
                        1.0f + 0.40f * clampf((kFilterYZeroDb - yCurve) * (1.0f / 28.0f), 0.0f,
                                              1.0f);
                    const float colScale = fillScale * edgeTaper * effFilterAlpha01;
                    const float glowAmp = 76.0f * hornBoost;

                    for (int py = pyStart; py <= pyEnd; ++py) {
                        const float pyF = static_cast<float>(py);
                        const float subEdge =
                            (pyF < yCurve) ? clampf(1.0f - (yCurve - pyF), 0.0f, 1.0f) : 1.0f;
                        const float dist = pyF - yCurve;
                        const float crestGlow =
                            glowAmp * fastExpNeg(((dist > 0.0f) ? dist : 0.0f) * 0.095f);
                        int alpha =
                            static_cast<int>((26.0f + crestGlow) * colScale * subEdge + 0.5f);
                        if (alpha > 220) {
                            alpha = 220;
                        }
                        if (alpha > 0) {
                            uint8_t& dst = lumBuf[static_cast<size_t>(py - y0) * kDisplayWidth +
                                                  static_cast<size_t>(px)];
                            dst = blendLum8(dst, 240u, static_cast<uint8_t>(alpha));
                        }
                    }
                }
            }
        }

        // 3. Linear-photon incandescent passband spine + MS-20 companion contours
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        auto addCurvePhoton = [&](const std::array<int16_t, kFilterPoints>& pts,
                                  float weight) noexcept {
            if (weight <= 0.004f) {
                return;
            }
            for (size_t i = 0u; i < kFilterPoints; ++i) {
                const int px = kFilterXStart + static_cast<int>(i);
                const size_t iNext = (i + 1u < kFilterPoints) ? (i + 1u) : i;
                const float yA = static_cast<float>(pts[i]) * kFixedToPx;
                const float yB = static_cast<float>(pts[iNext]) * kFixedToPx;
                const float yLo = (yA < yB) ? yA : yB;
                const float yHi = (yA > yB) ? yA : yB;
                const int segTop = static_cast<int>(yLo) - 3;
                const int segBot = static_cast<int>(yHi) + 4;
                if (segBot <= y0 || segTop >= y1) {
                    continue;
                }
                const float dy = yB - yA;
                const float invL2 = 1.0f / (1.0f + dy * dy);
                const float dwell =
                    (0.62f + 0.38f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * weight;
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
                                                    static_cast<size_t>(px)];
                        const uint32_t next = static_cast<uint32_t>(cell) + add;
                        cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                    }
                }
            }
        };

        const float compVis = 1.0f - div;
        if (compVis > 0.004f) {
            addCurvePhoton(filterCompCurvesFixed_[1],
                           filterCompWeights_[1] * compVis * effFilterAlpha01);
            addCurvePhoton(filterCompCurvesFixed_[0],
                           filterCompWeights_[0] * compVis * effFilterAlpha01);
        }
        const float mainCurveWgt =
            (multiCurves ? (0.245f + 0.275f * div) : 0.98f) * effFilterAlpha01;
        for (size_t vc = 0u; vc < curveCount; ++vc) {
            addCurvePhoton(filterCurvesFixed_[vc], mainCurveWgt);
        }

        for (uint16_t r = 0u; r < stripHeight; ++r) {
            const uint16_t* linRow = linScratch + static_cast<size_t>(r) * kDisplayWidth;
            uint8_t* lumRow = lumBuf + static_cast<size_t>(r) * kDisplayWidth;
            for (size_t x = static_cast<size_t>(kFilterXStart);
                 x < static_cast<size_t>(kFilterXStart) + kFilterPoints; ++x) {
                const uint16_t e = linRow[x];
                if (e > 0u) {
                    const uint32_t addLum = kTonemapLut[e >> 2u];
                    const uint32_t bg = lumRow[x];
                    lumRow[x] =
                        static_cast<uint8_t>(255u - (((255u - bg) * (255u - addLum)) >> 8u));
                }
            }
        }

        // 4. Dual HP/LP cutoff focal beads
        const float hpBeadVis = filterHpBeadInt_ * compVis;
        if (hpBeadVis > 0.005f) {
            filterCv.drawOpticalBead(filterHpBeadX_, filterHpBeadY_, hpBeadVis, filterHpBeadRad_);
        }
        const float lpBeadVis = filterLpBeadInt_ * compVis;
        if (lpBeadVis > 0.005f) {
            filterCv.drawOpticalBead(filterLpBeadX_, filterLpBeadY_, lpBeadVis, filterLpBeadRad_);
        }
    }

    // ========================================================================
    // C. Reserved Bottom 4-Knob Strip (y = 190..235, Concept A Optical Beads)
    // ========================================================================
    if (y1 > 188) {
        const float panScale = frameControl_.global.monoCollapse ? 0.0f : 1.0f;

        for (uint8_t k = 0u; k < 4u; ++k) {
            const int cx = 30 + static_cast<int>(k) * 60;
            char roleLbl[12]{};
            char valStr[16]{};
            bool bipolar = false;

            switch (frameHud_.knobBank) {
            case KnobBankMode::Frequency: {
                const VoiceParams& vp = frameControl_.voices[k];
                size_t p = appendStr(roleLbl, 0u, sizeof(roleLbl), "OSC ");
                appendUInt(roleLbl, p, sizeof(roleLbl), static_cast<uint32_t>(k + 1u));

                const float pitchMul = computeVoicePitchRatio(
                    frameControl_.voicePressure[k],
                    frameControl_.global.pitchPressureMode ? 1.0f : 0.0f);
                const float lfoHz =
                    (vp.lfoTarget == LfoTarget::Pitch) ?
                        applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, frameLfoOutput_[k]) :
                        vp.frequencyHz;
                formatCompactHz(valStr, sizeof(valStr), lfoHz * pitchMul);
                break;
            }
            case KnobBankMode::Amplitude: {
                const VoiceParams& vp = frameControl_.voices[k];
                size_t p = appendStr(roleLbl, 0u, sizeof(roleLbl), "OSC ");
                appendUInt(roleLbl, p, sizeof(roleLbl), static_cast<uint32_t>(k + 1u));
                const float liveAmp =
                    (vp.lfoTarget == LfoTarget::Amplitude) ?
                        applyLfoAmplitude(vp.amplitude, vp.lfoDepth, frameLfoOutput_[k]) :
                        vp.amplitude;
                const float norm = vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
                formatPercent(valStr, sizeof(valStr), norm);
                break;
            }
            case KnobBankMode::Panning: {
                const VoiceParams& vp = frameControl_.voices[k];
                size_t p = appendStr(roleLbl, 0u, sizeof(roleLbl), "OSC ");
                appendUInt(roleLbl, p, sizeof(roleLbl), static_cast<uint32_t>(k + 1u));
                bipolar = true;
                const float livePan =
                    (vp.lfoTarget == LfoTarget::Pan) ?
                        applyLfoPan(vp.pan, vp.lfoDepth, frameLfoOutput_[k]) :
                        vp.pan;
                formatPanValue(valStr, sizeof(valStr), clampf(livePan * panScale, -1.0f, 1.0f));
                break;
            }
            case KnobBankMode::VoiceFilter:
            case KnobBankMode::GlobalFilter: {
                float showHp = 20.0f;
                float showHpRes = 0.0f;
                float showLp = 20000.0f;
                float showLpRes = 0.0f;
                const float pitchMode = frameControl_.global.pitchPressureMode ? 1.0f : 0.0f;
                if (frameHud_.knobBank == KnobBankMode::GlobalFilter) {
                    float sumLogHp = 0.0f;
                    float sumLogLp = 0.0f;
                    for (size_t vi = 0u; vi < kNumOscillators; ++vi) {
                        const VoiceParams& vp = frameControl_.voices[vi];
                        const ModulatedFilterParams mod = computeLiveVoiceFilter(
                            vp, frameControl_.voicePressure[vi], pitchMode, frameLfoOutput_[vi]);
                        sumLogHp += fastLog2(mod.hpCutoffHz);
                        showHpRes += mod.hpResonance;
                        sumLogLp += fastLog2(mod.lpCutoffHz);
                        showLpRes += mod.lpResonance;
                    }
                    showHp = fastExp2(sumLogHp * 0.25f);
                    showHpRes *= 0.25f;
                    showLp = fastExp2(sumLogLp * 0.25f);
                    showLpRes *= 0.25f;
                } else {
                    const uint8_t refV = frameHud_.focusedVoice & 3u;
                    const VoiceParams& vp = frameControl_.voices[refV];
                    const ModulatedFilterParams mod = computeLiveVoiceFilter(
                        vp, frameControl_.voicePressure[refV], pitchMode, frameLfoOutput_[refV]);
                    showHp = mod.hpCutoffHz;
                    showHpRes = mod.hpResonance;
                    showLp = mod.lpCutoffHz;
                    showLpRes = mod.lpResonance;
                }
                if (k == 0u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "HPF");
                    formatCompactHz(valStr, sizeof(valStr), showHp);
                } else if (k == 1u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "HP RES");
                    formatPercent(valStr, sizeof(valStr), clampf(showHpRes, 0.0f, 1.0f));
                } else if (k == 2u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "LPF");
                    formatCompactHz(valStr, sizeof(valStr), showLp);
                } else {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "LP RES");
                    formatPercent(valStr, sizeof(valStr), clampf(showLpRes, 0.0f, 1.0f));
                }
                break;
            }
            case KnobBankMode::Lfo: {
                const bool showAll = (frameHud_.lfoVoiceFocus >= kNumOscillators);
                const VoiceParams& vp = frameControl_.voices[frameHud_.lfoVoiceFocus & 3u];
                if (k == 0u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "TARGET");
                    bool sameTarget = true;
                    if (showAll) {
                        for (size_t vi = 1u; vi < kNumOscillators; ++vi) {
                            if (frameControl_.voices[vi].lfoTarget !=
                                frameControl_.voices[0].lfoTarget) {
                                sameTarget = false;
                                break;
                            }
                        }
                    }
                    const LfoTarget tgt =
                        showAll ? frameControl_.voices[0].lfoTarget : vp.lfoTarget;
                    appendStr(valStr, 0u, sizeof(valStr),
                              sameTarget ? lfoTargetName(tgt) : "MIXED");
                } else if (k == 1u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "SHAPE");
                    float shp = vp.lfoShape;
                    if (showAll) {
                        shp = 0.25f * (frameControl_.voices[0].lfoShape +
                                       frameControl_.voices[1].lfoShape +
                                       frameControl_.voices[2].lfoShape +
                                       frameControl_.voices[3].lfoShape);
                    }
                    appendStr(valStr, 0u, sizeof(valStr), lfoShapeLabel(shp));
                } else if (k == 2u) {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "RATE");
                    float rateHz = vp.lfoRateHz;
                    if (showAll) {
                        const float avgLog = 0.25f * (fastLog2(frameControl_.voices[0].lfoRateHz) +
                                                      fastLog2(frameControl_.voices[1].lfoRateHz) +
                                                      fastLog2(frameControl_.voices[2].lfoRateHz) +
                                                      fastLog2(frameControl_.voices[3].lfoRateHz));
                        rateHz = fastExp2(avgLog);
                    }
                    formatCompactHz(valStr, sizeof(valStr), rateHz);
                } else {
                    appendStr(roleLbl, 0u, sizeof(roleLbl), "DEPTH");
                    float depth = vp.lfoDepth;
                    if (showAll) {
                        depth = 0.25f * (frameControl_.voices[0].lfoDepth +
                                         frameControl_.voices[1].lfoDepth +
                                         frameControl_.voices[2].lfoDepth +
                                         frameControl_.voices[3].lfoDepth);
                    }
                    formatPercent(valStr, sizeof(valStr), depth);
                }
                break;
            }
            }

            const float dim = clampf(knobDimSmooth_[k], 0.0f, 1.0f);
            const float foc = clampf(knobFocusSmooth_[k], 0.0f, 1.0f);
            const float posSigned = clampf(knobPosSignedSmooth_[k], -1.0f, 1.0f);

            // 1. Technical Knob Role Label (y = 191)
            const float baseLbl = 154.0f + (96.0f - 154.0f) * dim;
            const auto lblGray = static_cast<uint8_t>(baseLbl + (255.0f - baseLbl) * foc + 0.5f);
            cv.drawSansCentered(kFontSansSmall, cx, 191, roleLbl, lblGray, 1);

            // 2. Concept A Tapered Optical Filament + Sub-Pixel Focal Bead (y = 207.5)
            const float cxF = static_cast<float>(cx);
            const float bxf = cxF - 20.0f;
            const float bwf = 40.0f;
            const float byf = 207.5f;
            const float trackLum = 42.0f + (24.0f - 42.0f) * dim;
            const float unfocLum = 178.0f + (72.0f - 178.0f) * dim;
            const float activeLum = unfocLum + (235.0f - unfocLum) * (1.0f - dim) * foc;
            const float targetX = cxF + posSigned * 20.0f;

            cv.drawHorizontalFilament(bxf, bxf + bwf, byf, trackLum, 0.58f, true);
            if (bipolar) {
                cv.drawVerticalSlit(cxF, byf - 2.2f, byf + 2.2f, 85.0f);
                if (absf(targetX - cxF) > 0.4f) {
                    cv.drawHorizontalFilament(cxF, targetX, byf, activeLum, 0.68f, false);
                }
                const float beadIntUnfoc = 0.82f + (0.36f - 0.82f) * dim;
                const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * (1.0f - dim) * foc;
                const float beadRad = 1.90f + (2.25f - 1.90f) * foc;
                cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
            } else {
                if (targetX - bxf > 0.32f) {
                    cv.drawHorizontalFilament(bxf, targetX, byf, activeLum, 0.68f, false);
                }
                const float beadIntUnfoc = 0.80f + (0.34f - 0.80f) * dim;
                const float beadInt = beadIntUnfoc + (1.0f - beadIntUnfoc) * (1.0f - dim) * foc;
                const float beadRad = 1.85f + (2.30f - 1.85f) * foc;
                cv.drawOpticalBead(targetX, byf, beadInt, beadRad);
            }

            // 3. Tabular Sans-Serif Value Readout (y = 214)
            const float baseVal = 224.0f + (112.0f - 224.0f) * dim;
            const auto valGray =
                static_cast<uint8_t>(baseVal + (255.0f - baseVal) * (1.0f - dim) * foc + 0.5f);
            cv.drawSansCentered(kFontSansMedium, cx, 214, valStr, valGray, 0);
        }
    }
}

} // namespace ruido
