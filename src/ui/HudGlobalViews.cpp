#include "ui/HudGlobalViews.h"

#include "core/DspMath.h"
#include "core/GlobalConfig.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"
#include "ui/MonoCanvas.h"

#include <cstring>

namespace ruido {

namespace {

constexpr float kOriginX = 120.0f;
constexpr float kOriginY = 168.0f;
constexpr float kStageRadius = 112.0f;
constexpr float kRayInnerR = 11.0f;
// sin(0.9333f) * 112.0f = 90.0f -> aligns L100/R100 horizon ends with OSC 1/4 columns (x = 30, 210)
constexpr float kMaxPolarAngleRad = 0.9333f;

// Zero-libm polar unit direction from pan in [-1, +1] (0 = straight up: (0, -1))
inline void fastPolarDir(float panNorm, float& dirX, float& dirY) noexcept {
    const float theta = clampf(panNorm, -1.0f, 1.0f) * kMaxPolarAngleRad;
    const float a = absf(theta);
    const float a2 = a * a;
    const float s = a * (1.0f - a2 * (0.16666667f - 0.0083143f * a2));
    const float b = kHalfPi - a;
    const float b2 = b * b;
    const float c = b * (1.0f - b2 * (0.16666667f - 0.0083143f * b2));
    dirX = (theta < 0.0f) ? -s : s;
    dirY = -c;
}

// Sub-pixel arbitrary-angle optical line segment matching MonoStripCanvas::drawVerticalSlit
inline void drawPolarSegmentFilament(const MonoStripCanvas& cv, float x0, float y0Seg, float x1,
                                     float y1Seg, float peakLum, float halfSigma = 0.52f) noexcept {
    const float effPeak = peakLum * (static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f));
    if (effPeak <= 0.5f) {
        return;
    }
    const float minXF = (x0 < x1) ? x0 : x1;
    const float maxXF = (x0 > x1) ? x0 : x1;
    const float minYF = (y0Seg < y1Seg) ? y0Seg : y1Seg;
    const float maxYF = (y0Seg > y1Seg) ? y0Seg : y1Seg;

    int pxStart = static_cast<int>(minXF) - 3;
    int pxEnd = static_cast<int>(maxXF) + 3;
    if (pxStart < 0) {
        pxStart = 0;
    }
    if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
        pxEnd = static_cast<int>(kDisplayWidth) - 1;
    }
    int pyStart = static_cast<int>(minYF) - 3;
    int pyEnd = static_cast<int>(maxYF) + 3;
    if (pyStart < cv.y0) {
        pyStart = cv.y0;
    }
    if (pyEnd > cv.y1 - 1) {
        pyEnd = cv.y1 - 1;
    }
    if (pxStart > pxEnd || pyStart > pyEnd) {
        return;
    }

    const float vx = x1 - x0;
    const float vy = y1Seg - y0Seg;
    const float len2 = vx * vx + vy * vy;
    const float invLen2 = (len2 > 1.0e-5f) ? (1.0f / len2) : 0.0f;
    const float inv2Sig2 = 1.0f / (2.0f * halfSigma * halfSigma);

    for (int py = pyStart; py <= pyEnd; ++py) {
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        const float wy = static_cast<float>(py) - y0Seg;
        for (int px = pxStart; px <= pxEnd; ++px) {
            const float wx = static_cast<float>(px) - x0;
            const float t =
                (invLen2 > 0.0f) ? clampf((wx * vx + wy * vy) * invLen2, 0.0f, 1.0f) : 0.0f;
            const float ddx = wx - t * vx;
            const float ddy = wy - t * vy;
            const float d2 = ddx * ddx + ddy * ddy;
            if (d2 < 4.5f) {
                const uint32_t a8 =
                    static_cast<uint32_t>(effPeak * fastExpNeg(d2 * inv2Sig2) + 0.5f);
                if (a8 > 0u) {
                    row[px] =
                        blendLum8(row[px], 255u, static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
                }
            }
        }
    }
}

// Analytical circular-arc optical filament centered at (kOriginX, kOriginY)
inline void drawPolarArcFilament(const MonoStripCanvas& cv, float radius, float pan0, float pan1,
                                 float peakLum, float halfSigma = 0.52f,
                                 bool taperEnds = true) noexcept {
    const float effPeak = peakLum * (static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f));
    if (effPeak <= 0.5f || radius <= 2.0f) {
        return;
    }
    const float pLo = (pan0 < pan1) ? pan0 : pan1;
    const float pHi = (pan0 > pan1) ? pan0 : pan1;

    float d0X = 0.0f;
    float d0Y = -1.0f;
    float d1X = 0.0f;
    float d1Y = -1.0f;
    fastPolarDir(pLo, d0X, d0Y);
    fastPolarDir(pHi, d1X, d1Y);

    const float end0X = kOriginX + radius * d0X;
    const float end0Y = kOriginY + radius * d0Y;
    const float end1X = kOriginX + radius * d1X;
    const float end1Y = kOriginY + radius * d1Y;

    float minYF = (end0Y < end1Y) ? end0Y : end1Y;
    if (pLo <= 0.0f && pHi >= 0.0f) {
        const float apexY = kOriginY - radius;
        if (apexY < minYF) {
            minYF = apexY;
        }
    }
    const float maxYF = (end0Y > end1Y) ? end0Y : end1Y;

    int pxStart = static_cast<int>(end0X) - 3;
    int pxEnd = static_cast<int>(end1X) + 3;
    if (pxStart < 0) {
        pxStart = 0;
    }
    if (pxEnd > static_cast<int>(kDisplayWidth) - 1) {
        pxEnd = static_cast<int>(kDisplayWidth) - 1;
    }
    int pyStart = static_cast<int>(minYF) - 3;
    int pyEnd = static_cast<int>(maxYF) + 3;
    if (pyStart < cv.y0) {
        pyStart = cv.y0;
    }
    if (pyEnd > cv.y1 - 1) {
        pyEnd = cv.y1 - 1;
    }
    if (pxStart > pxEnd || pyStart > pyEnd) {
        return;
    }

    const float rMin = radius - 2.6f;
    const float rMax = radius + 2.6f;
    const float rMin2 = rMin * rMin;
    const float rMax2 = rMax * rMax;
    const float inv2Sig2 = 1.0f / (2.0f * halfSigma * halfSigma);

    for (int py = pyStart; py <= pyEnd; ++py) {
        const float pyF = static_cast<float>(py);
        const float dy = pyF - kOriginY;
        if (dy >= 0.0f) {
            continue;
        }
        const float dy2 = dy * dy;
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        for (int px = pxStart; px <= pxEnd; ++px) {
            const float pxF = static_cast<float>(px);
            const float dx = pxF - kOriginX;
            const float r2 = dx * dx + dy2;
            if (r2 < rMin2 || r2 > rMax2) {
                continue;
            }
            const float invR = fastInvSqrt(r2);
            const float r = r2 * invR;
            const float sinTheta = dx * invR;

            float d2 = 0.0f;
            float taper = 1.0f;
            if (sinTheta < d0X) {
                const float ex = pxF - end0X;
                const float ey = pyF - end0Y;
                d2 = ex * ex + ey * ey;
                if (taperEnds) {
                    taper = 0.35f;
                }
            } else if (sinTheta > d1X) {
                const float ex = pxF - end1X;
                const float ey = pyF - end1Y;
                d2 = ex * ex + ey * ey;
                if (taperEnds) {
                    taper = 0.35f;
                }
            } else {
                const float dr = r - radius;
                d2 = dr * dr;
                if (taperEnds) {
                    const float e0x = pxF - end0X;
                    const float e0y = pyF - end0Y;
                    const float e1x = pxF - end1X;
                    const float e1y = pyF - end1Y;
                    const float c0 = e0x * e0x + e0y * e0y;
                    const float c1 = e1x * e1x + e1y * e1y;
                    const float minC2 = (c0 < c1) ? c0 : c1;
                    if (minC2 < 16.0f) {
                        taper = 0.35f + 0.65f * (minC2 * (1.0f / 16.0f));
                    }
                }
            }
            if (d2 < 4.5f) {
                const uint32_t a8 =
                    static_cast<uint32_t>(effPeak * taper * fastExpNeg(d2 * inv2Sig2) + 0.5f);
                if (a8 > 0u) {
                    row[px] =
                        blendLum8(row[px], 255u, static_cast<uint8_t>((a8 > 255u) ? 255u : a8));
                }
            }
        }
    }
}

} // namespace

void HudGlobalViews::reset() noexcept {
    panSigned_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panBase_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panOrbitMin_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panOrbitMax_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panLfoDepth_.fill(0.0f);
    rayLen_.fill(80.0f);
    waveAmp_.fill(0.0f);
    muteDim_.fill(0.0f);
    clearImmediate();
}

void HudGlobalViews::clearImmediate() noexcept {
    panPhase_ = 0.0f;
    configPhase_ = 0.0f;
    primed_ = false;
}

bool HudGlobalViews::anyActiveTarget(const HudState& rawHud) const noexcept {
    if (rawHud.opacity <= 0.004f || rawHud.mode == HudMode::None) {
        return false;
    }
    return isPanStageActive(rawHud) || rawHud.configPageOpen ||
           (rawHud.mode == HudMode::GlobalConfig && rawHud.opacity >= 0.99f);
}

bool HudGlobalViews::hasResidualPhase() const noexcept {
    return panPhase_ > 0.002f || configPhase_ > 0.002f;
}

void HudGlobalViews::update(float dtClamped, const EngineControlState& control,
                            const HudState& rawHud, const LfoSignalArray& lfoOutput,
                            const std::array<float, kNumOscillators>& voicePeak) noexcept {
    const bool hudVisible = (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None);
    const bool panActive = isPanStageActive(rawHud);
    const bool cfgActive =
        hudVisible && (rawHud.configPageOpen ||
                       (rawHud.mode == HudMode::GlobalConfig && rawHud.opacity >= 0.99f));
    const bool wasPanInvisible = (panPhase_ <= 0.002f);

    panPhase_ = stepHudFadeLinear(panPhase_, panActive, dtClamped);
    configPhase_ = stepHudFadeLinear(configPhase_, cfgActive, dtClamped);

    if (!panActive) {
        if (panPhase_ <= 0.002f) {
            primed_ = false;
        }
        return;
    }

    const bool snapPan = !primed_ || wasPanInvisible;
    primed_ = true;

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float alphaPos = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.032f)));
    const float alphaAtk = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.024f)));
    const float alphaRel = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.125f)));

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
        const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
        const VoiceAudibility aud = computeVoiceAudibility(vp, control.voicePressure[v], pitchMode);
        const bool silentMute = aud.muteBlend >= 0.999f;

        const float liveAmpRaw = computeLiveVoiceAmplitude(vp, lfoA, lfoB);
        const float effAmp = silentMute ? 0.0f : clampf(liveAmpRaw, 0.0f, 1.0f);
        const float livePan = clampf(computeLiveVoicePan(vp, lfoA, lfoB) * panScale, -1.0f, 1.0f);
        const float basePan = clampf(vp.pan * panScale, -1.0f, 1.0f);

        const float dA = computeLiveLfoADepth(vp, lfoB);
        float pLo = (vp.lfoTarget == LfoTarget::Pan) ? applyLfoPan(vp.pan, dA, -1.0f) : vp.pan;
        float pHi = (vp.lfoTarget == LfoTarget::Pan) ? applyLfoPan(vp.pan, dA, +1.0f) : vp.pan;
        float modDepth = (vp.lfoTarget == LfoTarget::Pan) ? dA : 0.0f;
        if (vp.lfoBTarget == LfoTarget::Pan) {
            pLo = applyLfoPan(pLo, vp.lfoBDepth, -1.0f);
            pHi = applyLfoPan(pHi, vp.lfoBDepth, +1.0f);
            modDepth = clampf(modDepth + vp.lfoBDepth, 0.0f, 1.0f);
        }
        pLo = clampf(pLo * panScale, -1.0f, 1.0f);
        pHi = clampf(pHi * panScale, -1.0f, 1.0f);
        if (pLo > pHi) {
            const float tmp = pLo;
            pLo = pHi;
            pHi = tmp;
        }
        modDepth *= panScale;

        const float peakRaw = silentMute ? 0.0f : clampf(voicePeak[v], 0.0f, 1.50f);
        const float peakNorm = clampf(peakRaw * (1.0f / 1.15f), 0.0f, 1.0f);
        const float drvNorm =
            (effAmp > kCleanAmpKnee)
                ? clampf((effAmp - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee)), 0.0f, 1.0f)
                : 0.0f;

        const float tgtRayLen =
            silentMute ? 42.0f : clampf(46.0f + 48.0f * effAmp + 13.0f * peakNorm, 42.0f, 108.0f);
        const float audibleScale = 1.0f - aud.subAudioBlend;
        const float tgtWaveAmp =
            (silentMute || peakRaw < 0.005f)
                ? 0.0f
                : ((1.50f + 2.15f * peakNorm + 1.35f * drvNorm) * audibleScale);

        panSigned_[v] = slewScalar(panSigned_[v], livePan, alphaPos, 0.001f);
        panBase_[v] = slewScalar(panBase_[v], basePan, alphaPos, 0.001f);
        panOrbitMin_[v] = slewScalar(panOrbitMin_[v], pLo, alphaPos, 0.001f);
        panOrbitMax_[v] = slewScalar(panOrbitMax_[v], pHi, alphaPos, 0.001f);
        panLfoDepth_[v] = slewScalar(panLfoDepth_[v], modDepth, alphaPos, 0.002f);
        rayLen_[v] = slewScalar(rayLen_[v], tgtRayLen, alphaPos, 0.02f);

        const float aWave = (tgtWaveAmp >= waveAmp_[v]) ? alphaAtk : alphaRel;
        waveAmp_[v] = slewScalar(waveAmp_[v], tgtWaveAmp, aWave, 0.01f);
        muteDim_[v] = slewScalar(muteDim_[v], aud.inaudibleDim, alphaPos, 0.005f);
    }
}

void HudGlobalViews::compositePanStageStrip(
    int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf, uint16_t* linScratch,
    float effPanAlpha01, uint8_t alpha8, const HudState& hud,
    const OverdriveHairTable& overdriveHair) const noexcept {
    if (alpha8 == 0u || y1 <= 36 || y0 >= 176) {
        return;
    }

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};

    // 1. Restrained optical stage reticle (zero text labels; matches HudAmpGraph 22..30 lum guides)
    drawPolarArcFilament(cv, kStageRadius, -1.0f, 1.0f, 30.0f, 0.52f, true);
    cv.drawVerticalSlit(kOriginX, kOriginY - kStageRadius, kOriginY - kRayInnerR, 24.0f);

    float leftDirX = 0.0f;
    float leftDirY = -1.0f;
    fastPolarDir(-1.0f, leftDirX, leftDirY);
    drawPolarSegmentFilament(cv, kOriginX + leftDirX * kRayInnerR, kOriginY + leftDirY * kRayInnerR,
                             kOriginX + leftDirX * kStageRadius, kOriginY + leftDirY * kStageRadius,
                             22.0f, 0.50f);
    drawPolarSegmentFilament(cv, kOriginX - leftDirX * kRayInnerR, kOriginY + leftDirY * kRayInnerR,
                             kOriginX - leftDirX * kStageRadius, kOriginY + leftDirY * kStageRadius,
                             22.0f, 0.50f);
    cv.drawOpticalBead(kOriginX, kOriginY, 0.44f, 1.50f);

    // 2. 4 Polar Live-Audio Voice Filaments (1px CRT focus matching HudAmpGraph 3A, zero fog bloom)
    const bool globalPanAll = (hud.knobBank == KnobBankMode::GlobalAmp) && (hud.focusedKnob == 3u);
    const int pyStart = (y0 > 38) ? y0 : 38;
    const int pyEnd = (y1 < 172) ? y1 : 172;
    if (effPanAlpha01 > 0.004f && pyStart < pyEnd) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float masterE = clampf(effPanAlpha01, 0.0f, 1.0f);
        constexpr float kHairMaxIdx = static_cast<float>(kOverdriveHairPoints - 1u);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float dim = clampf(muteDim_[v], 0.0f, 1.0f);
            const float activeW = 1.0f - 0.78f * dim;
            const bool focused = globalPanAll || (hud.focusedKnob == static_cast<uint8_t>(v));
            const float effFoc = (focused ? 1.0f : 0.0f) * (1.0f - dim);
            const float focBoost = 1.0f + 0.24f * effFoc;

            float dirX = 0.0f;
            float dirY = -1.0f;
            fastPolarDir(panSigned_[v], dirX, dirY);
            const float normX = -dirY;
            const float normY = dirX;

            const float rEnd = rayLen_[v];
            const float spanR = rEnd - kRayInnerR;
            if (spanR <= 1.0f) {
                continue;
            }
            int nPts = static_cast<int>(spanR * 0.62f + 0.5f);
            if (nPts < 18) {
                nPts = 18;
            } else if (nPts > 60) {
                nPts = 60;
            }
            const float invNPts = 1.0f / static_cast<float>(nPts);
            const float wAmp = waveAmp_[v];
            const bool liveWave = (wAmp > 0.02f) && (dim < 0.92f);

            OverdriveHairArray companion{};
            if (liveWave) {
                smoothZeroPhase(overdriveHair[v].data(), companion.data(), kOverdriveHairPoints,
                                0.44f, 2u);
            }
            const size_t strandCount = liveWave ? 2u : 1u;
            const float* strands[2] = {overdriveHair[v].data(), companion.data()};
            const float strandWaveScale[2] = {wAmp, wAmp * 0.48f};
            const float strandBaseE[2] = {liveWave ? 215.0f : 250.0f, 105.0f};

            for (size_t s = 0u; s < strandCount; ++s) {
                const float layerE = strandBaseE[s] * activeW * focBoost * masterE;
                const float wScale = liveWave ? strandWaveScale[s] : 0.0f;
                float px0 = kOriginX + dirX * kRayInnerR;
                float py0 = kOriginY + dirY * kRayInnerR;

                for (int i = 1; i <= nPts; ++i) {
                    const float u = static_cast<float>(i) * invNPts;
                    const float r = kRayInnerR + u * spanR;
                    float waveOff = 0.0f;
                    if (wScale > 0.0f && i < nPts) {
                        const float idxF = u * kHairMaxIdx;
                        const size_t i0 = static_cast<size_t>(idxF);
                        const size_t i1 = (i0 + 1u < kOverdriveHairPoints) ? (i0 + 1u) : i0;
                        const float rawVal = strands[s][i0] + (idxF - static_cast<float>(i0)) *
                                                                  (strands[s][i1] - strands[s][i0]);
                        const float spindleEnv = fastSinPhase01(0.5f * u);
                        waveOff = clampf(rawVal, -1.0f, 1.0f) * wScale * spindleEnv;
                    }
                    const float px1 = kOriginX + dirX * r + normX * waveOff;
                    const float py1 = kOriginY + dirY * r + normY * waveOff;

                    const float innerTaper =
                        (u < 0.14f) ? (0.38f + 0.62f * (u * (1.0f / 0.14f))) : 1.0f;
                    const float segE = layerE * innerTaper;

                    const float yLoF = (py0 < py1) ? py0 : py1;
                    const float yHiF = (py0 > py1) ? py0 : py1;
                    int segY0 = static_cast<int>(yLoF) - 2;
                    int segY1 = static_cast<int>(yHiF) + 3;
                    if (segY0 < pyStart) {
                        segY0 = pyStart;
                    }
                    if (segY1 > pyEnd) {
                        segY1 = pyEnd;
                    }
                    if (segY0 < segY1) {
                        const float xLoF = (px0 < px1) ? px0 : px1;
                        const float xHiF = (px0 > px1) ? px0 : px1;
                        int segX0 = static_cast<int>(xLoF) - 2;
                        int segX1 = static_cast<int>(xHiF) + 2;
                        if (segX0 < 2) {
                            segX0 = 2;
                        }
                        if (segX1 > 237) {
                            segX1 = 237;
                        }
                        const float vx = px1 - px0;
                        const float vy = py1 - py0;
                        const float len2 = vx * vx + vy * vy;
                        const float invLen2 = (len2 > 1.0e-5f) ? (1.0f / len2) : 0.0f;

                        for (int py = segY0; py < segY1; ++py) {
                            uint16_t* linRow =
                                linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;
                            const float wy = static_cast<float>(py) - py0;
                            for (int px = segX0; px <= segX1; ++px) {
                                const float wx = static_cast<float>(px) - px0;
                                const float t =
                                    (invLen2 > 0.0f)
                                        ? clampf((wx * vx + wy * vy) * invLen2, 0.0f, 1.0f)
                                        : 0.0f;
                                const float ddx = wx - t * vx;
                                const float ddy = wy - t * vy;
                                const float d2 = ddx * ddx + ddy * ddy;
                                if (d2 < 3.6f) {
                                    const uint32_t add =
                                        static_cast<uint32_t>(segE * fastExpNeg(d2 * 2.05f) + 0.5f);
                                    if (add > 0u) {
                                        const uint32_t next =
                                            static_cast<uint32_t>(linRow[px]) + add;
                                        linRow[px] =
                                            static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                                    }
                                }
                            }
                        }
                    }

                    px0 = px1;
                    py0 = py1;
                }
            }
        }

        screenBlendLinearStrip(stripHeight, 2u, 238u, linScratch, lumBuf);
    }

    // 3. Foreground LFO -> PAN Polar Excursion Brackets & Crisp Tip Focal Beads
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float dim = clampf(muteDim_[v], 0.0f, 1.0f);
        const bool focused = globalPanAll || (hud.focusedKnob == static_cast<uint8_t>(v));
        const float effFoc = (focused ? 1.0f : 0.0f) * (1.0f - dim);
        const float rEnd = rayLen_[v];

        float dirX = 0.0f;
        float dirY = -1.0f;
        fastPolarDir(panSigned_[v], dirX, dirY);
        const float tipX = kOriginX + dirX * rEnd;
        const float tipY = kOriginY + dirY * rEnd;

        const float orbSpan = panOrbitMax_[v] - panOrbitMin_[v];
        if (orbSpan > 0.025f && dim < 0.85f) {
            const float arcLum = (104.0f + 38.0f * effFoc) * (1.0f - 0.65f * dim);
            const float tickLum = (128.0f + 42.0f * effFoc) * (1.0f - 0.65f * dim);
            drawPolarArcFilament(cv, rEnd, panOrbitMin_[v], panOrbitMax_[v], arcLum, 0.54f, false);

            float loX = 0.0f;
            float loY = -1.0f;
            float hiX = 0.0f;
            float hiY = -1.0f;
            float bsX = 0.0f;
            float bsY = -1.0f;
            fastPolarDir(panOrbitMin_[v], loX, loY);
            fastPolarDir(panOrbitMax_[v], hiX, hiY);
            fastPolarDir(panBase_[v], bsX, bsY);

            drawPolarSegmentFilament(cv, kOriginX + loX * (rEnd - 2.8f),
                                     kOriginY + loY * (rEnd - 2.8f), kOriginX + loX * (rEnd + 2.8f),
                                     kOriginY + loY * (rEnd + 2.8f), tickLum, 0.50f);
            drawPolarSegmentFilament(cv, kOriginX + hiX * (rEnd - 2.8f),
                                     kOriginY + hiY * (rEnd - 2.8f), kOriginX + hiX * (rEnd + 2.8f),
                                     kOriginY + hiY * (rEnd + 2.8f), tickLum, 0.50f);
            drawPolarSegmentFilament(cv, kOriginX + bsX * (rEnd - 1.8f),
                                     kOriginY + bsY * (rEnd - 1.8f), kOriginX + bsX * (rEnd + 1.8f),
                                     kOriginY + bsY * (rEnd + 1.8f), 84.0f * (1.0f - 0.65f * dim),
                                     0.50f);

            const float beaconInt = (0.82f + 0.18f * effFoc) * (1.0f - 0.60f * dim);
            cv.drawLfoPhaseBeacon(tipX, tipY, beaconInt, 0.30f + 0.70f * panLfoDepth_[v]);
        } else {
            const float beadInt = (0.80f + (0.30f - 0.80f) * dim) + 0.18f * effFoc;
            const float beadRad = 1.90f + 0.40f * effFoc;
            cv.drawOpticalBead(tipX, tipY, clampf(beadInt, 0.25f, 1.0f), beadRad);
        }
    }
}

void HudGlobalViews::compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                                const EngineControlState& control,
                                                const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 38 || y0 >= 232) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    constexpr size_t kMaxVisibleRows = 6u;
    constexpr int kRowStartY = 44;
    constexpr int kRowPitch = 28;

    const size_t total = kNumGlobalConfigItems;
    const size_t cursor = (hud.configCursor < total) ? static_cast<size_t>(hud.configCursor) : 0u;
    size_t firstRow = 0u;
    if (total > kMaxVisibleRows && cursor >= kMaxVisibleRows - 1u) {
        firstRow = cursor - (kMaxVisibleRows - 2u);
        if (firstRow + kMaxVisibleRows > total) {
            firstRow = total - kMaxVisibleRows;
        }
    }
    const size_t visibleCount =
        (total - firstRow > kMaxVisibleRows) ? kMaxVisibleRows : (total - firstRow);

    for (size_t r = 0u; r < visibleCount; ++r) {
        const size_t itemIdx = firstRow + r;
        const auto itemId = static_cast<ConfigItemId>(itemIdx);
        const int ry = kRowStartY + static_cast<int>(r) * kRowPitch;
        if (ry + kRowPitch < y0 - 2 || ry > y1 + 2) {
            continue;
        }

        const bool selected = (itemIdx == cursor);
        if (selected) {
            const int fillTop = (ry < y0) ? y0 : ry;
            const int fillBot = (ry + 24 > y1) ? y1 : (ry + 24);
            const auto fillAlpha =
                static_cast<uint8_t>((30u * static_cast<uint32_t>(alpha8) + 127u) / 255u);
            if (fillAlpha > 0u && fillTop < fillBot) {
                for (int py = fillTop; py < fillBot; ++py) {
                    uint8_t* row = lumBuf + static_cast<size_t>(py - y0) * kDisplayWidth;
                    for (int px = 10; px <= 230; ++px) {
                        row[px] = blendLum8(row[px], 255u, fillAlpha);
                    }
                }
            }
        }

        const char* itemLbl = globalConfigItemLabel(itemId);
        const char* valLbl = globalConfigValueLabel(itemId, hud.config, control.global);
        if (selected) {
            cv.drawSans(kFontSansMedium, 16, ry + 5, itemLbl, 255u, 0);
            cv.drawSansRight(kFontSansMedium, 224, ry + 5, valLbl, 255u, 0);
        } else {
            cv.drawSans(kFontSansSmall, 16, ry + 6, itemLbl, 138u, 1);
            cv.drawSansRight(kFontSansMedium, 224, ry + 5, valLbl, 174u, 0);
        }
    }
}

} // namespace ruido
