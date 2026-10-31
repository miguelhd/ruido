#include "ui/HudAmpGraph.h"

#include "core/DspMath.h"
#include "core/GlobalControl.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "ui/MonoCanvas.h"

#include <cstring>

namespace ruido {

void HudAmpGraph::reset() noexcept {
    faderY_.fill(kColYBot);
    driveNorm_.fill(0.0f);
    pulseWidth_.fill(0.50f);
    driftAmt_.fill(kDefaultVoiceDrift);
    driftLive_.fill(0.0f);
    muteDim_.fill(0.0f);
    overdriveHair_ = {};
    phase_ = 0.0f;
    widthPhase_ = 0.0f;
    driftPhase_ = 0.0f;
    primed_ = false;
}

void HudAmpGraph::clearImmediate() noexcept {
    phase_ = 0.0f;
    widthPhase_ = 0.0f;
    driftPhase_ = 0.0f;
    primed_ = false;
}

void HudAmpGraph::stepFadeOut(float dtClamped) noexcept {
    phase_ = stepHudFadeLinear(phase_, false, dtClamped);
    widthPhase_ = stepHudFadeLinear(widthPhase_, false, dtClamped);
    driftPhase_ = stepHudFadeLinear(driftPhase_, false, dtClamped);
    if (!hasResidualPhase()) {
        primed_ = false;
    }
}

void HudAmpGraph::update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                         const LfoSignalArray& lfoOutput,
                         const std::array<float, kNumOscillators>& voiceDriftNorm,
                         const OverdriveHairTable& overdriveHair) noexcept {
    const bool editVisible = rawHud.opacity > 0.001f && rawHud.editPageOpen && !rawHud.modifierHeld;
    const bool levelAct = isLevelViewActive(rawHud);
    const bool widthAct = editVisible && (rawHud.knobBank == KnobBankMode::PulseWidth);
    const bool driftAct = editVisible && (rawHud.knobBank == KnobBankMode::VoiceDrift);
    const bool anyAct = levelAct || widthAct || driftAct;
    const bool wasInvisible = !hasResidualPhase();

    phase_ = stepHudFadeLinear(phase_, levelAct, dtClamped);
    widthPhase_ = stepHudFadeLinear(widthPhase_, widthAct, dtClamped);
    driftPhase_ = stepHudFadeLinear(driftPhase_, driftAct, dtClamped);
    if (!anyAct && !hasResidualPhase()) {
        primed_ = false;
        return;
    }

    const bool snap = !primed_ || (anyAct && wasInvisible);
    primed_ = true;
    overdriveHair_ = overdriveHair;

    const float alphaFader = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.036f)));
    const float alphaDrift = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.065f)));

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
        const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
        const VoiceAudibility aud = computeVoiceAudibility(vp, control.voicePressure[v], pitchMode);
        const bool silentMute = aud.muteBlend >= 0.999f;
        const float liveAmp = computeLiveVoiceAmplitude(vp, lfoA, lfoB);
        const float effAmp = silentMute ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);

        const float tgtY = clampf(kColYBot - effAmp * kColYSpan, kColYTop, kColYBot);
        const float tgtDrive =
            (effAmp > kCleanAmpKnee)
                ? clampf((effAmp - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee)), 0.0f, 1.0f)
                : 0.0f;
        const float tgtPw =
            clampf(computeLiveVoicePulseWidth(vp, lfoA, lfoB), kPulseWidthMin, kPulseWidthMax);
        const float tgtDrAmt = clampf(vp.drift, 0.0f, 1.0f);
        const float tgtDrLive = silentMute ? 0.0f : clampf(voiceDriftNorm[v], -1.0f, 1.0f);
        const float tgtDim = aud.inaudibleDim;

        faderY_[v] = slewScalar(faderY_[v], tgtY, alphaFader, 0.01f);
        driveNorm_[v] = slewScalar(driveNorm_[v], tgtDrive, alphaFader, 0.002f);
        pulseWidth_[v] = slewScalar(pulseWidth_[v], tgtPw, alphaFader, 0.002f);
        driftAmt_[v] = slewScalar(driftAmt_[v], tgtDrAmt, alphaFader, 0.002f);
        driftLive_[v] = slewScalar(driftLive_[v], tgtDrLive, alphaDrift, 0.002f);
        muteDim_[v] = slewScalar(muteDim_[v], tgtDim, alphaFader, 0.005f);
    }
}

void HudAmpGraph::compositeStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                 uint16_t* linScratch, float hudAlpha01,
                                 const HudState& hud) const noexcept {
    if (hudAlpha01 <= 0.002f || y1 <= 16 || y0 >= 198) {
        return;
    }

    const auto resolveSubAlpha = [&](float linPhase, float& outE) noexcept -> uint8_t {
        const float env01 = smoothstep01(clampf(linPhase, 0.0f, 1.0f));
        outE = (env01 < hudAlpha01) ? env01 : hudAlpha01;
        return static_cast<uint8_t>(outE * 255.0f + 0.5f);
    };

    if (widthPhase_ > 0.002f) {
        float wAlpha01 = 0.0f;
        const uint8_t wAlpha8 = resolveSubAlpha(widthPhase_, wAlpha01);
        if (wAlpha8 > 0u) {
            compositeWidthStrip(y0, y1, stripHeight, lumBuf, linScratch, wAlpha01, wAlpha8, hud);
        }
    }
    if (driftPhase_ > 0.002f) {
        float dAlpha01 = 0.0f;
        const uint8_t dAlpha8 = resolveSubAlpha(driftPhase_, dAlpha01);
        if (dAlpha8 > 0u) {
            compositeDriftStrip(y0, y1, stripHeight, lumBuf, linScratch, dAlpha01, dAlpha8, hud);
        }
    }
    if (phase_ > 0.002f) {
        float lAlpha01 = 0.0f;
        const uint8_t lAlpha8 = resolveSubAlpha(phase_, lAlpha01);
        if (lAlpha8 > 0u) {
            compositeLevelStrip(y0, y1, stripHeight, lumBuf, linScratch, lAlpha01, lAlpha8, hud);
        }
    }
}

void HudAmpGraph::compositeLevelStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                      uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                                      const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 16 || y0 >= 198) {
        return;
    }

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};

    // 1. Canonical edge-to-edge 0% baseline (y = 186, shared with FILTER & PAN) + 70% Drive Knee
    cv.drawGraphBaseline();
    cv.drawHorizontalFilament(0.0f, static_cast<float>(kDisplayWidth - 1u), kKneeY, 22.0f, 0.48f,
                              false);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        cv.drawVerticalSlit(cx, kColYTop, kColYBot - 1.0f, 22.0f);
    }

    // 2. Linear-photon columns: clean VCA stem (0-70%) & 3A narrow hairy drive cap (>70%)
    const bool perVoiceFocus = !isGlobalKnobBank(hud.knobBank);
    const int pyStart = (y0 > 32) ? y0 : 32;
    const int pyEnd = (y1 < 188) ? y1 : 188;
    if (pyStart < pyEnd) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float masterE = clampf(effAlpha01, 0.0f, 1.0f);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const int cxInt = 30 + static_cast<int>(v) * 60;
            const float cx = static_cast<float>(cxInt);
            const float fy = faderY_[v];
            const float level01 = clampf((kColYBot - fy) * (1.0f / kColYSpan), 0.0f, 1.0f);
            const float drv = driveNorm_[v];
            const float dim = muteDim_[v];
            const float activeW = 1.0f - 0.78f * dim;
            const float effFoc =
                (perVoiceFocus && hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - dim);
            const float focBoost = 1.0f + 0.18f * effFoc;

            // Level-scaled brightness: clearly visible ~47% silver-white floor at lowest audible
            // level, brightening monotonically across 0-70% clean VCA into 100% incandescent white.
            const float levelCurve = level01 * (0.58f + 0.42f * level01);
            const float stemBaseE = (240.0f + 840.0f * levelCurve) * activeW * focBoost;

            const float stemTop = (drv > 0.005f) ? kKneeY : fy;
            const int xLo = cxInt - 4;
            const int xHi = cxInt + 4;

            for (int py = pyStart; py < pyEnd; ++py) {
                const float pyF = static_cast<float>(py);
                if (pyF < stemTop - 0.5f || pyF >= kColYBot) {
                    continue;
                }
                const float topCov = clampf((pyF + 0.5f) - stemTop, 0.0f, 1.0f);
                const float botCov = clampf(kColYBot - pyF, 0.0f, 1.0f);
                const float rowE = stemBaseE * topCov * botCov;
                if (rowE < 1.0f) {
                    continue;
                }
                uint16_t* linRow = linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;
                for (int px = xLo; px <= xHi; ++px) {
                    const float dx = absf(static_cast<float>(px) - cx);
                    const float edgeD = (dx > 0.52f) ? (dx - 0.52f) : 0.0f;
                    const float energy = rowE * fastExpNeg(edgeD * edgeD * 2.10f);
                    if (energy >= 1.0f) {
                        const uint32_t add = static_cast<uint32_t>(energy * masterE + 0.5f);
                        const uint32_t next = static_cast<uint32_t>(linRow[px]) + add;
                        linRow[px] = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                    }
                }
            }

            const float hZone = kKneeY - fy;
            if (dim < 0.92f && drv > 0.005f && hZone > 1.2f) {
                const float peakW = 3.8f * fastExp2(0.78f * fastLog2(clampf(drv, 0.001f, 1.0f)));
                const int kneeClipY = static_cast<int>(kKneeY) + 1;
                const int driveYEnd = (pyEnd < kneeClipY) ? pyEnd : kneeClipY;

                // Subtle CRT phosphor bloom confined strictly to the > 70% V-cone drive region
                int auraY0 = static_cast<int>(fy - 3.0f);
                if (auraY0 < pyStart) {
                    auraY0 = pyStart;
                }
                if (auraY0 < driveYEnd) {
                    const float invHZone = 1.0f / hZone;
                    const float auraBaseE = (30.0f + 48.0f * drv) * activeW * focBoost * masterE;
                    for (int py = auraY0; py < driveYEnd; ++py) {
                        const float pyF = static_cast<float>(py);
                        const float u = clampf((kKneeY - pyF) * invHZone, 0.0f, 1.0f);
                        const float tipFade =
                            (pyF < fy) ? fastExpNeg((fy - pyF) * (fy - pyF) * 0.42f) : 1.0f;
                        const float rowE = auraBaseE * u * tipFade;
                        if (rowE < 1.5f) {
                            continue;
                        }
                        const float halfSpan = 0.95f + u * (peakW * 0.62f + 0.85f);
                        const float invFalloff = 1.0f / (halfSpan * halfSpan * 1.35f);
                        const int axRange = static_cast<int>(halfSpan * 2.1f) + 2;
                        int ax0 = cxInt - axRange;
                        int ax1 = cxInt + axRange;
                        if (ax0 < 2) {
                            ax0 = 2;
                        }
                        if (ax1 > 237) {
                            ax1 = 237;
                        }
                        uint16_t* linRow =
                            linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;
                        for (int px = ax0; px <= ax1; ++px) {
                            const float dx = static_cast<float>(px) - cx;
                            const uint32_t add = static_cast<uint32_t>(
                                rowE * fastExpNeg(dx * dx * invFalloff) + 0.5f);
                            if (add > 0u) {
                                const uint32_t next = static_cast<uint32_t>(linRow[px]) + add;
                                linRow[px] = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                            }
                        }
                    }
                }

                OverdriveHairArray layer1{};
                OverdriveHairArray layer2{};
                smoothZeroPhase(overdriveHair_[v].data(), layer1.data(), kOverdriveHairPoints,
                                0.42f, 1u);
                smoothZeroPhase(overdriveHair_[v].data(), layer2.data(), kOverdriveHairPoints,
                                0.45f, 3u);
                const float* layers[3] = {overdriveHair_[v].data(), layer1.data(), layer2.data()};
                constexpr float kLayerBaseE[3] = {350.0f, 205.0f, 130.0f};

                int nPts = static_cast<int>(hZone * 1.45f + 0.5f);
                if (nPts < 12) {
                    nPts = 12;
                } else if (nPts > 64) {
                    nPts = 64;
                }
                const float invNPts = 1.0f / static_cast<float>(nPts);
                constexpr float kHairMaxIdx = static_cast<float>(kOverdriveHairPoints - 1u);

                for (size_t s = 0u; s < 3u; ++s) {
                    const float layerE =
                        kLayerBaseE[s] * (1.00f + 0.46f * drv) * activeW * focBoost * masterE;
                    const float layerGain =
                        (1.15f + 2.35f * drv) * (1.0f - 0.14f * static_cast<float>(s));
                    float px0 = cx;
                    float py0 = kKneeY;

                    for (int i = 1; i <= nPts; ++i) {
                        const float u = static_cast<float>(i) * invNPts;
                        const float cy = kKneeY - u * hZone;
                        const float idxF = u * kHairMaxIdx;
                        const size_t i0 = static_cast<size_t>(idxF);
                        const size_t i1 = (i0 + 1u < kOverdriveHairPoints) ? (i0 + 1u) : i0;
                        const float rawVal = layers[s][i0] + (idxF - static_cast<float>(i0)) *
                                                                 (layers[s][i1] - layers[s][i0]);
                        const float clipped = clampf(rawVal * layerGain, -1.0f, 1.0f);
                        const float cx1 = cx + clipped * (u * peakW);
                        const float segE = layerE * ((absf(clipped) >= 0.93f) ? 1.48f : 1.0f);
                        const float segBloomE = segE * (0.14f + 0.14f * drv) * u;
                        const float baseR2 = 0.27f * (1.0f - u);
                        const float coreFalloff = 0.95f + 1.00f * u;

                        const float yLoF = (py0 < cy) ? py0 : cy;
                        const float yHiF = (py0 > cy) ? py0 : cy;
                        int segY0 = static_cast<int>(yLoF) - 3;
                        int segY1 = static_cast<int>(yHiF) + 4;
                        if (segY0 < pyStart) {
                            segY0 = pyStart;
                        }
                        if (segY1 > driveYEnd) {
                            segY1 = driveYEnd;
                        }
                        if (segY0 < segY1) {
                            const float xLoF = (px0 < cx1) ? px0 : cx1;
                            const float xHiF = (px0 > cx1) ? px0 : cx1;
                            int segX0 = static_cast<int>(xLoF) - 4;
                            int segX1 = static_cast<int>(xHiF) + 4;
                            if (segX0 < 2) {
                                segX0 = 2;
                            }
                            if (segX1 > 237) {
                                segX1 = 237;
                            }
                            const float vx = cx1 - px0;
                            const float vy = cy - py0;
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
                                    if (d2 < 12.5f) {
                                        const float effD2 = (d2 > baseR2) ? (d2 - baseR2) : 0.0f;
                                        const float core =
                                            (effD2 < 3.8f)
                                                ? (segE * fastExpNeg(effD2 * coreFalloff))
                                                : 0.0f;
                                        const float bloom = segBloomE * fastExpNeg(d2 * 0.28f);
                                        const uint32_t add =
                                            static_cast<uint32_t>(core + bloom + 0.5f);
                                        if (add > 0u) {
                                            const uint32_t next =
                                                static_cast<uint32_t>(linRow[px]) + add;
                                            linRow[px] = static_cast<uint16_t>(
                                                (next > 4095u) ? 4095u : next);
                                        }
                                    }
                                }
                            }
                        }
                        px0 = cx1;
                        py0 = cy;
                    }
                }
            }
        }

        screenBlendLinearStrip(stripHeight, 2u, 238u, linScratch, lumBuf);
    }

    // 3. Clean optical crest bead strictly within the clean VCA zone (<= 70%), matched in width
    //    to the 3-px capsule stem and scaling smoothly in brightness with level.
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (driveNorm_[v] > 0.006f) {
            continue;
        }
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        const float fy = faderY_[v];
        const float cleanFrac =
            clampf((kColYBot - fy) * (1.0f / (kCleanAmpKnee * kColYSpan)), 0.0f, 1.0f);
        const float dim = muteDim_[v];
        const float effFoc = (perVoiceFocus && hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - dim);
        const float audibleBeadInt = 0.56f + 0.36f * cleanFrac + 0.12f * effFoc;
        const float beadInt = audibleBeadInt + (0.28f - audibleBeadInt) * dim;
        const float beadRad = 1.80f + 0.25f * effFoc;
        cv.drawOpticalBead(cx, fy, clampf(beadInt, 0.24f, 1.0f), beadRad);
    }
}

void HudAmpGraph::compositeWidthStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                      uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                                      const HudState& hud) const noexcept {
    constexpr float kYHigh = 74.0f;
    constexpr float kYMid = 114.0f;
    constexpr float kYLow = 154.0f;
    constexpr float kHalfSpanX = 21.0f;

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    cv.drawHorizontalFilament(10.0f, 230.0f, kYMid, 28.0f, 0.48f, true);
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        cv.drawVerticalSlit(cx, kYHigh - 6.0f, kYLow + 6.0f, 22.0f);
    }

    const int pyStart = (y0 > 70) ? y0 : 70;
    const int pyEnd = (y1 < 158) ? y1 : 158;
    if (pyStart < pyEnd) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float masterE = clampf(effAlpha01, 0.0f, 1.0f);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const int cxInt = 30 + static_cast<int>(v) * 60;
            const float cx = static_cast<float>(cxInt);
            const float x0 = cx - kHalfSpanX;
            const float x1 = cx + kHalfSpanX;
            const float xEdge =
                x0 + clampf(pulseWidth_[v], kPulseWidthMin, kPulseWidthMax) * (2.0f * kHalfSpanX);
            const float activeW = 1.0f - 0.78f * muteDim_[v];
            const float effFoc = (hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - muteDim_[v]);
            const float focBoost = 1.0f + 0.18f * effFoc;
            const int pxLo = cxInt - 23;
            const int pxHi = cxInt + 23;

            for (int py = pyStart; py < pyEnd; ++py) {
                const float pyF = static_cast<float>(py);
                uint16_t* linRow = linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;
                const bool highHalf = (pyF >= kYHigh - 1.0f && pyF <= kYMid);
                const bool lowHalf = (pyF > kYMid && pyF <= kYLow + 1.0f);
                if (!highHalf && !lowHalf) {
                    continue;
                }
                const float vDecay = highHalf
                                         ? fastExpNeg((pyF - kYHigh) * (1.0f / 26.0f))
                                         : (0.48f * fastExpNeg((kYLow - pyF) * (1.0f / 22.0f)));

                for (int px = pxLo; px <= pxHi; ++px) {
                    const float pxF = static_cast<float>(px);
                    const bool inside =
                        highHalf ? (pxF >= x0 && pxF <= xEdge) : (pxF >= xEdge && pxF <= x1);
                    if (!inside) {
                        continue;
                    }
                    const float energy = 150.0f * vDecay * activeW * focBoost;
                    if (energy >= 1.0f) {
                        const uint32_t add = static_cast<uint32_t>(energy * masterE + 0.5f);
                        const uint32_t next = static_cast<uint32_t>(linRow[px]) + add;
                        linRow[px] = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                    }
                }
            }
        }
        screenBlendLinearStrip(stripHeight, 2u, 238u, linScratch, lumBuf);
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        const float x0 = cx - kHalfSpanX;
        const float x1 = cx + kHalfSpanX;
        const float xEdge =
            x0 + clampf(pulseWidth_[v], kPulseWidthMin, kPulseWidthMax) * (2.0f * kHalfSpanX);
        const float dim = muteDim_[v];
        const float effFoc = (hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - dim);
        const float lineLum = (182.0f + 48.0f * effFoc) * (1.0f - 0.65f * dim);
        const float lowLum = lineLum * 0.72f;

        cv.drawVerticalSlit(x0, kYHigh, kYLow, lineLum);
        cv.drawHorizontalFilament(x0, xEdge, kYHigh, lineLum, 0.68f, false);
        cv.drawVerticalSlit(xEdge, kYHigh, kYLow, lineLum);
        cv.drawHorizontalFilament(xEdge, x1, kYLow, lowLum, 0.62f, false);
        cv.drawVerticalSlit(x1, kYMid, kYLow, lowLum);

        const float beadInt = (0.82f + 0.18f * effFoc) * (1.0f - 0.60f * dim);
        cv.drawOpticalBead(xEdge, kYMid, clampf(beadInt, 0.25f, 1.0f), 1.90f + 0.35f * effFoc);
    }
}

void HudAmpGraph::compositeDriftStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                      uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                                      const HudState& hud) const noexcept {
    constexpr float kYZero = 108.0f;
    constexpr float kMaxSpanPx = 56.0f;

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    cv.drawHorizontalFilament(10.0f, 230.0f, kYZero, 36.0f, 0.50f, true);
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        cv.drawVerticalSlit(cx, kYZero - kMaxSpanPx - 4.0f, kYZero + kMaxSpanPx + 4.0f, 24.0f);
    }

    const int pyStart = (y0 > 46) ? y0 : 46;
    const int pyEnd = (y1 < 170) ? y1 : 170;
    if (pyStart < pyEnd) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float masterE = clampf(effAlpha01, 0.0f, 1.0f);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const int cxInt = 30 + static_cast<int>(v) * 60;
            const float cx = static_cast<float>(cxInt);
            const float spanPx = clampf(driftAmt_[v], 0.0f, 1.0f) * kMaxSpanPx;
            const float beaconY = clampf(kYZero - driftLive_[v] * kMaxSpanPx, kYZero - kMaxSpanPx,
                                         kYZero + kMaxSpanPx);
            const float stemLo = (beaconY < kYZero) ? beaconY : kYZero;
            const float stemHi = (beaconY > kYZero) ? beaconY : kYZero;
            const float activeW = 1.0f - 0.78f * muteDim_[v];
            const float effFoc = (hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - muteDim_[v]);
            const float focBoost = 1.0f + 0.18f * effFoc;

            for (int py = pyStart; py < pyEnd; ++py) {
                const float pyF = static_cast<float>(py);
                const float dyZero = absf(pyF - kYZero);
                const bool inEnv = (spanPx > 0.6f && dyZero <= spanPx + 0.5f);
                const bool inStem = (pyF >= stemLo - 0.5f && pyF <= stemHi + 0.5f);
                if (!inEnv && !inStem) {
                    continue;
                }
                uint16_t* linRow = linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;
                for (int px = cxInt - 14; px <= cxInt + 14; ++px) {
                    const float dx = absf(static_cast<float>(px) - cx);
                    float energy = 0.0f;
                    if (inEnv && dx <= 11.0f) {
                        const float envFall = 1.0f - 0.45f * (dyZero / (spanPx + 0.5f));
                        energy += 95.0f * fastExpNeg(dx * dx * 0.085f) * envFall * activeW;
                    }
                    if (inStem && dx < 4.5f) {
                        energy += 250.0f * fastExpNeg(dx * dx * 1.65f) * activeW * focBoost;
                    }
                    if (energy >= 1.0f) {
                        const uint32_t add = static_cast<uint32_t>(energy * masterE + 0.5f);
                        const uint32_t next = static_cast<uint32_t>(linRow[px]) + add;
                        linRow[px] = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                    }
                }
            }
        }
        screenBlendLinearStrip(stripHeight, 2u, 238u, linScratch, lumBuf);
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        const float dAmt = clampf(driftAmt_[v], 0.0f, 1.0f);
        const float spanPx = dAmt * kMaxSpanPx;
        const float beaconY =
            clampf(kYZero - driftLive_[v] * kMaxSpanPx, kYZero - kMaxSpanPx, kYZero + kMaxSpanPx);
        const float dim = muteDim_[v];
        const float effFoc = (hud.focusedKnob == v ? 1.0f : 0.0f) * (1.0f - dim);

        if (spanPx > 0.8f) {
            const float brkLum = (118.0f + 44.0f * effFoc) * (1.0f - 0.65f * dim);
            cv.drawHorizontalFilament(cx - 10.0f, cx + 10.0f, kYZero - spanPx, brkLum, 0.58f, true);
            cv.drawHorizontalFilament(cx - 10.0f, cx + 10.0f, kYZero + spanPx, brkLum, 0.58f, true);
        }
        if (dAmt > 0.02f && dim < 0.85f) {
            const float beaconInt = (0.82f + 0.18f * effFoc) * (1.0f - 0.60f * dim);
            cv.drawLfoPhaseBeacon(cx, beaconY, beaconInt, 0.30f + 0.70f * dAmt);
        } else {
            const float beadInt = (0.78f + 0.20f * effFoc) * (1.0f - 0.60f * dim);
            cv.drawOpticalBead(cx, beaconY, clampf(beadInt, 0.25f, 1.0f), 1.85f + 0.40f * effFoc);
        }
    }
}

} // namespace ruido
