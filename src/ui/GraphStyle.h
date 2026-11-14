#pragma once

#include "core/DspMath.h"
#include "ui/MonoCanvas.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

// Canonical full-screen graph viewport & optical reference styling (shared across FILTER, PAN,
// LEVEL, DUTY, DRIFT, and LFO so baseline position, opacity, and guide filaments never drift).
inline constexpr float kGraphTopY = 26.0f;
inline constexpr float kGraphBaselineY = 186.0f;
inline constexpr float kGraphBaselinePeakLum = 50.0f;
inline constexpr float kGraphBaselineHalfSigma = 0.50f;
inline constexpr float kGraphRefLinePeakLum = 22.0f;
inline constexpr float kGraphRefLineHalfSigma = 0.48f;
inline constexpr float kGraphGuideSlitLum = 22.0f;
inline constexpr int kGraphStripMinY1 = 16;
inline constexpr int kGraphStripMaxY0 = 198;

// Canonical 4-column HUD layout aligned 1-to-1 with physical KNOB 1-4 / OSC 1-4 columns
inline constexpr int kHudColStartX = 30;
inline constexpr int kHudColStrideX = 60;
inline constexpr int kHudBotLabelY = 191;
inline constexpr float kHudBotTrackY = 207.5f;
inline constexpr int kHudBotValueY = 214;
inline constexpr int kHudBotDisabledValueY = 216;
inline constexpr int kHudHomeModRowY = 234;
inline constexpr int kHudHomeFxLfoGapPx = 10;
inline constexpr int kHudHomeLfoPairGapPx = 6;

inline constexpr int hudColumnCenterX(size_t col) noexcept {
    return kHudColStartX + static_cast<int>(col) * kHudColStrideX;
}

inline constexpr float hudColumnCenterXF(size_t col) noexcept {
    return static_cast<float>(hudColumnCenterX(col));
}

// Shared S-curve sub-view envelope clamped by master HUD opacity
inline uint8_t resolveSubViewAlpha(float linearPhase, float hudAlpha01,
                                   float& outAlpha01) noexcept {
    const float env01 = smoothstep01(clampf(linearPhase, 0.0f, 1.0f));
    outAlpha01 = (env01 < hudAlpha01) ? env01 : hudAlpha01;
    return static_cast<uint8_t>(outAlpha01 * 255.0f + 0.5f);
}

// Canonical edge-to-edge unipolar graph baseline (y = 186, shared by FILTER, PAN, and LEVEL)
inline void drawGraphBaseline(const MonoStripCanvas& cv) noexcept {
    cv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), kGraphBaselineY,
                              kGraphBaselinePeakLum, kGraphBaselineHalfSigma, false);
}

// Canonical edge-to-edge secondary reference horizon (e.g., FILTER 0 dB & LEVEL 70% Drive Knee)
inline void drawGraphRefHorizon(const MonoStripCanvas& cv, float yCenter) noexcept {
    cv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), yCenter,
                              kGraphRefLinePeakLum, kGraphRefLineHalfSigma, false);
}

// Canonical vertical guide hairline stopping 1 px above kGraphBaselineY so baseline opacity stays
// 100% uniform across all 240 columns
inline void drawGraphGuideSlit(const MonoStripCanvas& cv, float xCenter, float yTop) noexcept {
    cv.drawVerticalSlit(xCenter, yTop, kGraphBaselineY - 1.0f, kGraphGuideSlitLum);
}

// Shared 4x-subsampled directional vertical-decay skirt shader for unipolar full-screen curves
// (used by both MS-20 FILTER passbands and Cauchy-Lorentzian PAN soundstage bells).
inline void shadeUnipolarGraphColumn(size_t colX, float yA, float yB, float zeroRefY,
                                     float baseGlowAmp, float colScale, int y0, int y1,
                                     uint8_t* lumBuf) noexcept {
    if (colScale <= 0.004f) {
        return;
    }
    const float yTopSeg = (yA < yB) ? yA : yB;
    if (kGraphBaselineY - yTopSeg <= 0.25f) {
        return;
    }
    int pyStart = static_cast<int>(yTopSeg - 0.5f);
    if (pyStart < static_cast<int>(kGraphTopY)) {
        pyStart = static_cast<int>(kGraphTopY);
    }
    if (pyStart < y0) {
        pyStart = y0;
    }
    int pyEnd = static_cast<int>(kGraphBaselineY) - 1;
    if (pyEnd > y1 - 1) {
        pyEnd = y1 - 1;
    }
    if (pyStart > pyEnd) {
        return;
    }

    const float dy = yB - yA;
    const float absDy = absf(dy);
    const float yMid = 0.5f * (yA + yB);
    const float hornBoost =
        1.0f + 0.42f * clampf((zeroRefY - yTopSeg) * (1.0f / 56.0f), 0.0f, 1.0f);
    const float glowAmp = baseGlowAmp * hornBoost;
    const float invL2 = fastRecipPos(1.0f + dy * dy);
    const bool steepSeg = (absDy > 1.5f);

    for (int py = pyStart; py <= pyEnd; ++py) {
        const float pyF = static_cast<float>(py);
        float subEdge = 0.0f;
        if (!steepSeg) {
            subEdge = clampf((pyF + 0.5f) - yMid, 0.0f, 1.0f);
        } else {
            for (int s = 0; s < 4; ++s) {
                const float u = (static_cast<float>(s) + 0.5f) * 0.25f;
                const float ySub = yA + u * dy;
                subEdge += clampf((pyF + 0.5f) - ySub, 0.0f, 1.0f) * 0.25f;
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
        const float vDist = (vy > 0.0f) ? vy : 0.0f;
        const float effDist = 0.65f * vDist + 0.35f * perpDist;
        const float baseTaper = smoothstep01((kGraphBaselineY - pyF) * (1.0f / 6.0f));
        const float crestGlow = glowAmp * fastExpNeg(effDist * 0.048f) * baseTaper;
        int alpha = static_cast<int>((22.0f + crestGlow) * colScale * subEdge + 0.5f);
        if (alpha > 220) {
            alpha = 220;
        }
        if (alpha > 0) {
            uint8_t& dst = lumBuf[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
            dst = blendLum8(dst, 240u, static_cast<uint8_t>(alpha));
        }
    }
}

// Splat a unipolar curve spine segment with smooth baseline-toe feathering so resting stopbands
// and distant tails never brighten the canonical y = 186 baseline filament.
inline void splatUnipolarGraphSegment(size_t colX, float yA, float yB, float weight, int y0, int y1,
                                      uint16_t* linScratch) noexcept {
    const float yTopSeg = (yA < yB) ? yA : yB;
    const float tailFade = smoothstep01((kGraphBaselineY - yTopSeg - 1.2f) * (1.0f / 4.5f));
    if (tailFade > 0.004f) {
        splatPhotonSegment(static_cast<int>(colX), yA, yB, weight * tailFade, 0.62f, y0, y1,
                           linScratch);
    }
}

} // namespace ruido
