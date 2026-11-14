#include "ui/BootIntroRender.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "ui/BootIntroData.h"
#include "ui/ScopeTraceMath.h"

#include <cstring>

namespace ruido {

namespace {

// User-tuned Physical CRT Boot Intro Spec (1.00 s total):
constexpr float kBeamPeakMs = 90.0f;
constexpr float kBeamHandoffMs = 60.0f;
constexpr float kUnfoldDelayMs = 100.0f;
constexpr float kUnfoldDurMs = 200.0f;
constexpr float kHoldDurMs = 520.0f;
constexpr float kHoldEndMs = kUnfoldDelayMs + kUnfoldDurMs + kHoldDurMs; // 820 ms
constexpr float kExitDurMs = 180.0f;
constexpr float kIntroTotalMs = kHoldEndMs + kExitDurMs; // 1000 ms
constexpr float kFastDismissSpeed = 2.0f;                // ~90 ms smooth collapse on user touch

constexpr float kBzX1 = 0.12f;
constexpr float kBzY1 = 1.03f;
constexpr float kBzX2 = 0.24f;
constexpr float kBzY2 = 1.00f;

constexpr float kUnfoldStartSx = 1.04f;
constexpr float kUnfoldStartSy = 0.018f;

constexpr float kBeamRx = 112.0f;
constexpr float kBeamRy = 1.4f;
constexpr float kSpotRadiusPx = 2.0f;
constexpr float kAnodeSagPct = 4.0f;
constexpr float kHsyncJitterPx = 2.0f;

constexpr float kCorePeakLum = 255.0f;
constexpr float kCoreSettledLum = 205.0f;
constexpr float kBloomPeakOp = 1.00f;
constexpr float kBloomSettledOp = 0.32f;
constexpr float kPhosphorDecayMs = 140.0f;

constexpr float kBlurTightX = 1.6f;
constexpr float kBlurTightY = 1.6f * 0.62f;
constexpr float kBlurMedX = 1.5f;
constexpr float kBlurMedY = 1.5f * 0.58f;
constexpr float kBlurWideX = 1.0f;
constexpr float kBlurWideY = 0.80f;

// Scanline mask depth = 0.00 -> even rows = 1.00 (256), odd rows = 1.00 (256 / 256)
constexpr uint32_t kOddRowKeepQ8 = 256u;

inline constexpr std::array<uint8_t, 32> kCrtQuant32 = {
    0u,   8u,   16u,  25u,  33u,  41u,  49u,  58u,  66u,  74u,  82u,  90u,  99u,  107u, 115u, 123u,
    132u, 140u, 148u, 156u, 165u, 173u, 181u, 189u, 197u, 205u, 214u, 222u, 230u, 239u, 247u, 255u,
};

float solveCubicBezier(float x) noexcept {
    if (x <= 0.0f) {
        return 0.0f;
    }
    if (x >= 1.0f) {
        return 1.0f;
    }
    constexpr float cx = 3.0f * kBzX1;
    constexpr float bx = 3.0f * (kBzX2 - kBzX1) - cx;
    constexpr float ax = 1.0f - cx - bx;
    constexpr float cy = 3.0f * kBzY1;
    constexpr float by = 3.0f * (kBzY2 - kBzY1) - cy;
    constexpr float ay = 1.0f - cy - by;

    float t = x;
    for (int i = 0; i < 6; ++i) {
        const float xt = ((ax * t + bx) * t + cx) * t - x;
        const float dx = (3.0f * ax * t + 2.0f * bx) * t + cx;
        if (absf(dx) < 1.0e-5f) {
            break;
        }
        t = clampf(t - xt / dx, 0.0f, 1.0f);
    }
    return ((ay * t + by) * t + cy) * t;
}

// Normal CDF Phi(z) via logistic approximation (max abs error < 0.009, zero libm).
inline float fastNormalCdf(float z) noexcept {
    if (z >= 0.0f) {
        return 1.0f / (1.0f + fastExpNeg(1.702f * z));
    }
    const float e = fastExpNeg(-1.702f * z);
    return e / (1.0f + e);
}

// 1D box [-halfExtent, +halfExtent] convolved with N(0, sigma^2) evaluated at distance `absDist`.
inline float boxConvolvedGauss1D(float absDist, float halfExtent, float invSigma) noexcept {
    const float hi = (halfExtent - absDist) * invSigma;
    const float lo = (-halfExtent - absDist) * invSigma;
    return clampf(fastNormalCdf(hi) - fastNormalCdf(lo), 0.0f, 1.0f);
}

inline uint8_t readCoreNibbleQ8(int cx, int cy) noexcept {
    using namespace intro_data;
    if (cx < kCoreX0 || cx >= (kCoreX0 + kCoreW) || cy < kCoreY0 || cy >= (kCoreY0 + kCoreH)) {
        return 0u;
    }
    const int rx = cx - kCoreX0;
    const int ry = cy - kCoreY0;
    const uint8_t packed = kLogoCore4Bit[static_cast<size_t>(ry * kCoreRowBytes + (rx >> 1))];
    const uint8_t nib = ((rx & 1) == 0) ? (packed >> 4u) : (packed & 0x0Fu);
    return static_cast<uint8_t>(nib * 17u);
}

inline uint32_t sampleCoreBilinearQ8(int32_t uxQ16, int32_t uyQ16) noexcept {
    using namespace intro_data;
    const int x0 = uxQ16 >> 16;
    const int y0 = uyQ16 >> 16;
    if (x0 < (kCoreX0 - 1) || x0 >= (kCoreX0 + kCoreW) || y0 < (kCoreY0 - 1) ||
        y0 >= (kCoreY0 + kCoreH)) {
        return 0u;
    }
    const uint32_t fx = static_cast<uint32_t>((uxQ16 >> 8) & 0xFF);
    const uint32_t fy = static_cast<uint32_t>((uyQ16 >> 8) & 0xFF);
    if ((fx | fy) == 0u) {
        return readCoreNibbleQ8(x0, y0);
    }
    const uint32_t v00 = readCoreNibbleQ8(x0, y0);
    const uint32_t v10 = readCoreNibbleQ8(x0 + 1, y0);
    const uint32_t v01 = readCoreNibbleQ8(x0, y0 + 1);
    const uint32_t v11 = readCoreNibbleQ8(x0 + 1, y0 + 1);
    const uint32_t top = (v00 * (256u - fx) + v10 * fx) >> 8u;
    const uint32_t bot = (v01 * (256u - fx) + v11 * fx) >> 8u;
    return (top * (256u - fy) + bot * fy) >> 8u;
}

inline uint32_t sampleGridBilinearQ8(const uint8_t* grid, int cols, int rows, int32_t gxQ8,
                                     int32_t gyQ8) noexcept {
    if (gyQ8 <= -256 || gyQ8 >= (rows << 8) || gxQ8 <= -256 || gxQ8 >= (cols << 8)) {
        return 0u;
    }
    const int c0 = gxQ8 >> 8;
    const int r0 = gyQ8 >> 8;
    const uint32_t fx = static_cast<uint32_t>(gxQ8 & 0xFF);
    const uint32_t fy = static_cast<uint32_t>(gyQ8 & 0xFF);

    const auto fetch = [&](int c, int r) noexcept -> uint32_t {
        if (c < 0 || c >= cols || r < 0 || r >= rows) {
            return 0u;
        }
        return grid[static_cast<size_t>(r * cols + c)];
    };

    const uint32_t v00 = fetch(c0, r0);
    const uint32_t v10 = fetch(c0 + 1, r0);
    const uint32_t v01 = fetch(c0, r0 + 1);
    const uint32_t v11 = fetch(c0 + 1, r0 + 1);
    const uint32_t top = (v00 * (256u - fx) + v10 * fx) >> 8u;
    const uint32_t bot = (v01 * (256u - fx) + v11 * fx) >> 8u;
    return (top * (256u - fy) + bot * fy) >> 8u;
}

} // namespace

void BootIntroRenderer::reset() noexcept {
    elapsedMs_ = 0.0f;
    scopeAlpha_ = 1.0f;
    beamRx_ = 1.12f;
    beamRy_ = kBeamRy;
    beamOp_ = 0.0f;
    beamCenterIdx_ = 119.5f;
    invCoreSxQ16_ = 65536;
    invCoreSyQ16_ = 65536;
    invBloomSyQ16_ = 65536;
    centerXQ16_ = 120 * 65536 - 32768;
    logoAlphaQ8_ = 0u;
    renderedFrames_ = 0u;
    logoQuantLum8_ = 255u;
    active_ = false;
    fastDismiss_ = false;
    beamVisible_ = false;
    logoVisible_ = false;
}

void BootIntroRenderer::arm() noexcept {
    reset();
    active_ = true;
    scopeAlpha_ = 0.0f;
    evaluateAtMs(12.0f);
}

void BootIntroRenderer::notifyUserInput() noexcept {
    if (!active_) {
        return;
    }
    if (renderedFrames_ == 0u) {
        active_ = false;
        scopeAlpha_ = 1.0f;
        return;
    }
    if (!fastDismiss_) {
        fastDismiss_ = true;
        if (elapsedMs_ < kHoldEndMs) {
            elapsedMs_ = kHoldEndMs;
        }
    }
}

void BootIntroRenderer::step(float dtSec, bool hudActive) noexcept {
    if (!active_) {
        return;
    }
    if (hudActive) {
        notifyUserInput();
        if (!active_) {
            return;
        }
    }

    const float dtMs = clampf(dtSec, 0.001f, 0.100f) * 1000.0f;
    elapsedMs_ += fastDismiss_ ? (dtMs * kFastDismissSpeed) : dtMs;
    if (elapsedMs_ >= kIntroTotalMs) {
        active_ = false;
        scopeAlpha_ = 1.0f;
        beamVisible_ = false;
        logoVisible_ = false;
        return;
    }

    if (renderedFrames_ < 65535u) {
        ++renderedFrames_;
    }
    evaluateAtMs(elapsedMs_);
}

void BootIntroRenderer::evaluateAtMs(float tMs) noexcept {
    // 1. Damped H-Sync Lock Oscillation (`tMs > kBeamPeakMs * 0.5f`)
    float jitterX = 0.0f;
    if (tMs > (kBeamPeakMs * 0.5f)) {
        const float dtS = (tMs - (kBeamPeakMs * 0.5f)) * 0.001f;
        const float env = fastExpNeg(dtS * 10.5f);
        if (env > 0.005f) {
            const float ph = wrapLfoPhase01(dtS * (96.0f * (1.0f / (2.0f * kPi))));
            jitterX = kHsyncJitterPx * env * fastSinPhase01(ph);
        }
    }

    // 2. Exit / Scope Handoff (`collapse-y` vertical collapse with 1/sy brightness surge)
    float exitFade = 1.0f;
    float exitScaleY = 1.0f;
    float exitBloomBoost = 1.0f;
    float collapseSurge = 0.0f;
    scopeAlpha_ = 0.0f;
    if (tMs > kHoldEndMs) {
        const float uExit = clampf((tMs - kHoldEndMs) * (1.0f / kExitDurMs), 0.0f, 1.0f);
        const float remExit = 1.0f - uExit;
        // (1 - uExit)^1.7 approximated via remExit * (0.3f * remExit + 0.7f * sqrt(remExit))
        const float sqrtRem = (remExit > 1.0e-5f) ? (remExit * fastInvSqrt(remExit)) : 0.0f;
        exitScaleY = clampf(remExit * (0.30f * remExit + 0.70f * sqrtRem), 0.018f, 1.0f);
        const float uExit2 = uExit * uExit;
        exitFade = clampf(1.0f - uExit2 * (0.80f + 0.20f * uExit), 0.0f, 1.0f);
        collapseSurge = fastSinPhase01(0.5f * uExit);
        exitBloomBoost = 1.0f + 0.65f * collapseSurge;
        scopeAlpha_ = smoothstep01(clampf((uExit - 0.18f) * (1.0f / 0.82f), 0.0f, 1.0f));
    }

    // 3. Flat Horizontal Scanline + Center Cathode Strike Spot (NO vertical oval inflation!)
    const float uH = smoothstep01(clampf(tMs * (1.0f / kBeamPeakMs), 0.0f, 1.0f));
    const float sqrtUH = (uH > 1.0e-5f) ? (uH * fastInvSqrt(uH)) : 0.0f;
    const float beamSx = 0.015f + 0.985f * (0.90f * sqrtUH + 0.10f * uH);

    float beamLineOp = 0.0f;
    if (tMs <= kUnfoldDelayMs) {
        beamLineOp = clampf(tMs * (1.0f / (kBeamPeakMs * 0.25f)), 0.0f, 1.0f);
    } else if (tMs < (kUnfoldDelayMs + kBeamHandoffMs)) {
        const float uHand = clampf((tMs - kUnfoldDelayMs) * (1.0f / kBeamHandoffMs), 0.0f, 1.0f);
        const float remHand = 1.0f - uHand;
        beamLineOp = remHand * remHand;
    }

    float spotOp = 0.0f;
    float spotRx = kSpotRadiusPx;
    float spotRy = kSpotRadiusPx;
    if (tMs < (kBeamPeakMs * 1.1f)) {
        const float uSpot = clampf(tMs * (1.0f / (kBeamPeakMs * 0.85f)), 0.0f, 1.0f);
        const float remSpot = 1.0f - uSpot;
        spotOp = remSpot * (0.60f * remSpot + 0.40f);
        spotRx = kSpotRadiusPx * (1.0f + 2.8f * uSpot);
        spotRy = kSpotRadiusPx * clampf(1.0f - 0.55f * uSpot, 0.35f, 1.0f);
    }

    const float maxBeamOp = ((beamLineOp > spotOp) ? beamLineOp : spotOp) * exitFade;
    beamOp_ = clampf(maxBeamOp, 0.0f, 1.0f);
    beamVisible_ = (beamOp_ > 0.003f);

    if (beamVisible_) {
        beamRx_ = clampf(kBeamRx * beamSx, 1.5f, 120.0f);
        beamRy_ = kBeamRy; // Stays flat (`1.4 px`) — never inflates vertically!
        beamCenterIdx_ = 119.5f + jitterX;
        const float invRx = 1.0f / beamRx_;
        const float flatHalfH = 0.5f * kBeamRy;
        for (size_t x = 0u; x < kDisplayWidth; ++x) {
            const float dx = absf(static_cast<float>(x) - beamCenterIdx_);
            const float q = dx * invRx;
            // Flat horizontal scanline gradient (`beam-h-grad`: 1.0 plateau across 84%, soft tips)
            float hGrad = 0.0f;
            if (q <= 0.68f) {
                hGrad = 1.0f;
            } else if (q <= 0.86f) {
                hGrad = 1.0f - (q - 0.68f) * (0.35f / 0.18f);
            } else if (q < 1.0f) {
                hGrad = 0.65f * (1.0f - (q - 0.86f) * (1.0f / 0.14f));
            }
            const float lineEnv = hGrad * beamLineOp;

            // Radial cathode strike spot at center (`0 .. 99 ms`)
            float spotEnv = 0.0f;
            float spotHalfH = 0.0f;
            if (spotOp > 0.005f && dx < spotRx) {
                const float sq = dx * (1.0f / spotRx);
                const float rem2 = 1.0f - sq * sq;
                spotEnv = spotOp * rem2;
                spotHalfH = spotRy * rem2 * fastInvSqrt(rem2) * spotOp;
            }

            const float envX = clampf((lineEnv > spotEnv) ? lineEnv : spotEnv, 0.0f, 1.0f);
            beamTightX_[x] = static_cast<uint8_t>(
                boxConvolvedGauss1D(dx, beamRx_, 1.0f / kBlurTightX) * envX * 255.0f + 0.5f);
            beamMedX_[x] = static_cast<uint8_t>(
                boxConvolvedGauss1D(dx, beamRx_, 1.0f / kBlurMedX) * envX * 255.0f + 0.5f);
            beamWideX_[x] = static_cast<uint8_t>(
                boxConvolvedGauss1D(dx, beamRx_, 1.0f / kBlurWideX) * envX * 255.0f + 0.5f);

            const float coreHalfH =
                ((flatHalfH * lineEnv) > spotHalfH) ? (flatHalfH * lineEnv) : spotHalfH;
            beamCoreHalfHQ4_[x] =
                static_cast<uint8_t>(clampf(coreHalfH * 16.0f + 0.5f, 0.0f, 255.0f));
        }
    }

    // 4. Unified RUIDO Raster (pre-unfold slit + V-Yoke Unfold + Anode Sag + 1/sy Energy)
    logoVisible_ = false;
    logoAlphaQ8_ = 0u;
    if (exitFade > 0.002f && tMs > (kBeamPeakMs * 0.35f)) {
        float relSx = kUnfoldStartSx;
        float relSy = kUnfoldStartSy;
        float coreOp = 0.0f;
        float bloomOp = 0.0f;
        float coreLum = kCorePeakLum;
        float uUnfold = 0.0f;

        if (tMs <= kUnfoldDelayMs) {
            const float preSweep = smoothstep01((tMs - kBeamPeakMs * 0.35f) *
                                                (1.0f / (kUnfoldDelayMs - kBeamPeakMs * 0.35f)));
            relSx = clampf(beamSx * kUnfoldStartSx, 0.20f, 2.0f);
            relSy = kUnfoldStartSy;
            coreOp = 0.85f * preSweep;
            bloomOp = kBloomPeakOp * preSweep;
            coreLum = kCorePeakLum;
        } else {
            const float dtUnfold = tMs - kUnfoldDelayMs;
            uUnfold = clampf(dtUnfold * (1.0f / kUnfoldDurMs), 0.0f, 1.0f);
            const float eUnfold = solveCubicBezier(uUnfold);

            const float swellPhase =
                0.5f * clampf(dtUnfold * (1.0f / (kUnfoldDurMs * 0.85f)), 0.0f, 1.0f);
            const float anodeSwell = 1.0f + (kAnodeSagPct * 0.01f) * fastSinPhase01(swellPhase) *
                                                fastExpNeg(dtUnfold * (1.0f / 240.0f));

            const float sxLerp = clampf(uUnfold * 1.25f, 0.0f, 1.0f);
            relSx = clampf((kUnfoldStartSx + (1.0f - kUnfoldStartSx) * sxLerp) * anodeSwell, 0.25f,
                           2.0f);
            relSy = clampf((kUnfoldStartSy + (1.0f - kUnfoldStartSy) * eUnfold) * anodeSwell,
                           0.018f, 2.0f);
            coreOp = 1.0f;

            // Physical 1/sy energy density + exponential phosphor persistence tail
            const float clampedSy = clampf(eUnfold, 0.02f, 1.0f);
            const float remSy = 1.0f - clampedSy;
            const float sqrtRemSy = (remSy > 1.0e-5f) ? (remSy * fastInvSqrt(remSy)) : 0.0f;
            const float slitComp = remSy * sqrtRemSy; // (1 - sy)^1.5
            const float phosphorTail = fastExpNeg(dtUnfold * (1.0f / kPhosphorDecayMs));
            const float energyEnv =
                clampf(((slitComp * 0.85f) > phosphorTail) ? (slitComp * 0.85f) : phosphorTail,
                       0.0f, 1.0f);

            bloomOp = kBloomSettledOp + (kBloomPeakOp - kBloomSettledOp) * energyEnv;
            coreLum = kCoreSettledLum + (kCorePeakLum - kCoreSettledLum) * energyEnv;
        }

        if (collapseSurge > 0.0f) {
            coreLum = clampf(coreLum + (255.0f - coreLum) * collapseSurge, 0.0f, 255.0f);
        }

        const float totalSy = clampf(relSy * exitScaleY, 0.04f, 2.0f);
        const float bloomSy = clampf(0.18f + 0.82f * totalSy, 0.18f, 2.0f);

        const float aCore = clampf(coreOp * exitFade, 0.0f, 1.0f);
        const float aBloom = clampf(bloomOp * exitBloomBoost * exitFade, 0.0f, 1.0f);
        const float aSrc = aCore + (1.0f - aCore) * aBloom;

        if (aSrc > 0.003f) {
            const float cSrc = aCore * (coreLum * (1.0f / 255.0f)) + (1.0f - aCore) * aBloom;
            const float unpre01 = clampf(cSrc / aSrc, 0.0f, 1.0f);
            const int qIdx = (unpre01 >= 1.0f) ? 31 : static_cast<int>(unpre01 * 32.0f);
            logoQuantLum8_ = kCrtQuant32[static_cast<size_t>(qIdx & 31)];
            logoAlphaQ8_ = static_cast<uint16_t>(aSrc * 256.0f + 0.5f);
            if (logoAlphaQ8_ > 256u) {
                logoAlphaQ8_ = 256u;
            }

            if (uUnfold >= 0.999f && tMs <= kHoldEndMs && absf(jitterX) < 0.01f &&
                absf(relSy - 1.0f) < 0.003f) {
                invCoreSxQ16_ = 65536;
                invCoreSyQ16_ = 65536;
                invBloomSyQ16_ = 65536;
                centerXQ16_ = (119 * 65536) + 32768;
            } else {
                invCoreSxQ16_ = static_cast<int32_t>((1.0f / relSx) * 65536.0f + 0.5f);
                invCoreSyQ16_ = static_cast<int32_t>((1.0f / totalSy) * 65536.0f + 0.5f);
                invBloomSyQ16_ = static_cast<int32_t>((1.0f / bloomSy) * 65536.0f + 0.5f);
                centerXQ16_ = static_cast<int32_t>((119.5f + jitterX) * 65536.0f + 0.5f);
            }
            logoVisible_ = (logoAlphaQ8_ > 0u);
        }
    }
}

void BootIntroRenderer::compositeStripChunk(int y0, uint16_t chunkH, uint8_t* lumBuf,
                                            bool blendOverScope) const noexcept {
    if (!active_) {
        return;
    }
    if (!blendOverScope) {
        std::memset(lumBuf, 0, static_cast<size_t>(chunkH) * kDisplayWidth);
    }
    if (!beamVisible_ && !logoVisible_) {
        return;
    }

    constexpr int32_t kPivotQ16 = (119 * 65536) + 32768; // 119.5 in Q16
    using namespace intro_data;

    for (uint16_t r = 0u; r < chunkH; ++r) {
        const int py = y0 + static_cast<int>(r);
        uint8_t* dstRow = lumBuf + static_cast<size_t>(r) * kDisplayWidth;
        const uint32_t scanKeepQ8 = ((py & 1) == 0) ? 256u : kOddRowKeepQ8;

        // Evaluate per-row vertical beam factors (`0` outside beam window)
        uint32_t beamYtQ8 = 0u;
        uint32_t beamYmQ8 = 0u;
        uint32_t beamYwQ8 = 0u;
        uint32_t absDyQ4 = 4096u;
        const uint32_t beamOpQ8 = static_cast<uint32_t>(beamOp_ * 256.0f + 0.5f);
        bool rowHasBeam = false;
        if (beamVisible_) {
            const float absDy = absf(static_cast<float>(py) - 119.5f);
            if (absDy < beamRy_ + 11.0f) {
                const float effHalfY = 0.85f * beamRy_;
                const float yt = boxConvolvedGauss1D(absDy, effHalfY, 1.0f / kBlurTightY);
                const float ym = boxConvolvedGauss1D(absDy, effHalfY, 1.0f / kBlurMedY);
                const float yw = boxConvolvedGauss1D(absDy, effHalfY, 1.0f / kBlurWideY);
                beamYtQ8 = (static_cast<uint32_t>(yt * 256.0f + 0.5f) * beamOpQ8) >> 8u;
                beamYmQ8 = (static_cast<uint32_t>(ym * 256.0f + 0.5f) * beamOpQ8) >> 8u;
                beamYwQ8 = (static_cast<uint32_t>(yw * 256.0f + 0.5f) * beamOpQ8) >> 8u;
                absDyQ4 = static_cast<uint32_t>(absDy * 16.0f + 0.5f);
                rowHasBeam = (beamYtQ8 | beamYmQ8 | beamYwQ8) != 0u || (absDy <= beamRy_ + 1.5f);
            }
        }

        // Evaluate per-row logo Y coordinates (`0` outside [78..161] settled / unfolded band)
        int32_t coreUyQ16 = 0;
        int32_t nearGyQ8 = -1024;
        int32_t wideGyQ8 = -1024;
        bool rowHasLogo = false;
        if (logoVisible_) {
            const int32_t dyQ8 = (py << 8) - ((119 << 8) + 128);
            coreUyQ16 = kPivotQ16 + (dyQ8 * (invCoreSyQ16_ >> 8));
            const int32_t bloomUyQ16 = kPivotQ16 + (dyQ8 * (invBloomSyQ16_ >> 8));
            nearGyQ8 = ((bloomUyQ16 >> 8) - (kNearY0 << 8)) >> 1;
            wideGyQ8 = ((bloomUyQ16 >> 8) - (kWideY0 << 8)) >> 1;
            if (wideGyQ8 > -256 && wideGyQ8 < (kWideRows << 8)) {
                rowHasLogo = true;
            }
        }

        if (!rowHasBeam && !rowHasLogo) {
            continue;
        }

        const int32_t xOffQ8 = -(centerXQ16_ >> 8);
        int32_t uxQ16 = kPivotQ16 + (xOffQ8 * (invCoreSxQ16_ >> 8));
        const uint32_t logoAQ8 = logoAlphaQ8_;
        const uint32_t logoLum8 = logoQuantLum8_;

        for (size_t x = 0u; x < kDisplayWidth; ++x, uxQ16 += invCoreSxQ16_) {
            uint32_t aLogoMergedQ8 = 0u;
            if (rowHasLogo) {
                const uint32_t m0 = sampleCoreBilinearQ8(uxQ16, coreUyQ16);
                const int32_t uxQ8 = uxQ16 >> 8;
                const uint32_t bn = sampleGridBilinearQ8(kBloomNear8Bit.data(), kNearCols,
                                                         kNearRows, uxQ8 >> 1, nearGyQ8);
                const uint32_t bw = sampleGridBilinearQ8(kBloomWide8Bit.data(), kWideCols,
                                                         kWideRows, uxQ8 >> 2, wideGyQ8);

                const uint32_t a0 = (logoAQ8 * m0) >> 8u;
                const uint32_t an = (logoAQ8 * bn) >> 8u;
                const uint32_t aw = (logoAQ8 * bw) >> 8u;
                const uint32_t inv01 = ((256u - a0) * (256u - an)) >> 8u;
                const uint32_t invAll = (inv01 * (256u - aw)) >> 8u;
                aLogoMergedQ8 = 256u - invAll;
            }

            uint32_t aTotalQ8 = aLogoMergedQ8;
            uint32_t unpreLum8 = logoLum8;

            if (rowHasBeam) {
                uint32_t aCoreBeamQ8 = 0u;
                const uint32_t halfHQ4 = beamCoreHalfHQ4_[x];
                if (halfHQ4 > 0u && absDyQ4 < halfHQ4 + 16u) {
                    const uint32_t edgeCovQ4 =
                        (absDyQ4 + 16u <= halfHQ4) ? 16u : (halfHQ4 + 16u - absDyQ4);
                    aCoreBeamQ8 = (beamOpQ8 * edgeCovQ4) >> 4u;
                }
                const uint32_t abt = (beamYtQ8 * beamTightX_[x]) >> 8u;
                const uint32_t abm = (beamYmQ8 * beamMedX_[x]) >> 8u;
                const uint32_t abw = (beamYwQ8 * beamWideX_[x]) >> 8u;
                const uint32_t invB0 = ((256u - aCoreBeamQ8) * (256u - abt)) >> 8u;
                const uint32_t invB1 = ((256u - abm) * (256u - abw)) >> 8u;
                const uint32_t aBeamMergedQ8 = 256u - ((invB0 * invB1) >> 8u);

                if (aBeamMergedQ8 > 0u) {
                    aTotalQ8 = 256u - (((256u - aBeamMergedQ8) * (256u - aLogoMergedQ8)) >> 8u);
                    unpreLum8 = 255u;
                }
            }

            if (aTotalQ8 == 0u) {
                continue;
            }

            const uint32_t aMaskedQ8 = (aTotalQ8 * scanKeepQ8) >> 8u;
            const uint32_t cIntro8 = (unpreLum8 * aMaskedQ8) >> 8u;

            if (!blendOverScope) {
                dstRow[x] = static_cast<uint8_t>((cIntro8 > 255u) ? 255u : cIntro8);
            } else {
                const uint32_t bg = dstRow[x];
                const uint32_t out = cIntro8 + (((256u - aMaskedQ8) * bg) >> 8u);
                dstRow[x] = static_cast<uint8_t>((out > 255u) ? 255u : out);
            }
        }
    }
}

} // namespace ruido
