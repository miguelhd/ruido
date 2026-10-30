#include "ui/HudAmpGraph.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"
#include "ui/HudFormat.h"
#include "ui/MonoCanvas.h"

#include <cstring>

namespace ruido {

void HudAmpGraph::reset() noexcept {
    faderY_.fill(kColYBot);
    driveNorm_.fill(0.0f);
    driveRatio_.fill(1.0f);
    peakY_.fill(kColYBot);
    muteDim_.fill(0.0f);
    phase_ = 0.0f;
    primed_ = false;
}

void HudAmpGraph::clearImmediate() noexcept {
    phase_ = 0.0f;
    primed_ = false;
}

void HudAmpGraph::stepFadeOut(float dtClamped) noexcept {
    phase_ = stepHudFadeLinear(phase_, false, dtClamped);
    if (phase_ <= 0.001f) {
        primed_ = false;
    }
}

void HudAmpGraph::update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                         const std::array<float, kNumOscillators>& lfoOutput,
                         const std::array<float, kNumOscillators>& voicePeak) noexcept {
    const bool active = isActive(rawHud);
    const bool wasInvisible = (phase_ <= 0.002f);
    phase_ = stepHudFadeLinear(phase_, active, dtClamped);
    if (!active && phase_ <= 0.001f) {
        primed_ = false;
        return;
    }

    const bool snap = !primed_ || (active && wasInvisible);
    primed_ = true;

    const float alphaFader = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.036f)));
    const float alphaPeakAtk = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.022f)));
    const float alphaPeakRel = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.135f)));

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const float liveAmp = (vp.lfoTarget == LfoTarget::Amplitude)
                                  ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[v])
                                  : vp.amplitude;
        const float effAmp = vp.muted ? 0.0f : clampf(liveAmp, 0.0f, 1.0f);
        const VoiceChannelGain chGain = computeVoiceChannelGain(effAmp);

        const float tgtY = clampf(kColYBot - effAmp * kColYSpan, kColYTop, kColYBot);
        const float tgtDrive =
            (effAmp > kCleanAmpKnee)
                ? clampf((effAmp - kCleanAmpKnee) * (1.0f / (1.0f - kCleanAmpKnee)), 0.0f, 1.0f)
                : 0.0f;
        const float tgtDim = vp.muted ? 1.0f : 0.0f;

        const float pNorm = clampf(voicePeak[v] * (1.0f / 1.55f), 0.0f, 1.0f);
        const float tgtPeakY = clampf(kColYBot - pNorm * kColYSpan, kColYTop, kColYBot);

        faderY_[v] = slewScalar(faderY_[v], tgtY, alphaFader, 0.01f);
        driveNorm_[v] = slewScalar(driveNorm_[v], tgtDrive, alphaFader, 0.002f);
        driveRatio_[v] = slewScalar(driveRatio_[v], chGain.preDrive, alphaFader, 0.005f);
        muteDim_[v] = slewScalar(muteDim_[v], tgtDim, alphaFader, 0.005f);

        const float alphaPeak = (tgtPeakY <= peakY_[v]) ? alphaPeakAtk : alphaPeakRel;
        peakY_[v] = slewScalar(peakY_[v], tgtPeakY, alphaPeak, 0.02f);
    }
}

void HudAmpGraph::compositeStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                 uint16_t* linScratch, float effAlpha01, uint8_t alpha8,
                                 const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 24 || y0 >= 188) {
        return;
    }

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};

    // 1. Optical reference horizons: 0% floor (y = 180) and 70% / 1.0x Drive Knee (y = 79.2)
    cv.drawHorizontalFilament(10.0f, 230.0f, kColYBot, 40.0f, 0.52f, true);
    cv.drawHorizontalFilament(10.0f, 230.0f, kKneeY, 26.0f, 0.48f, true);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float cx = 30.0f + static_cast<float>(v) * 60.0f;
        cv.drawVerticalSlit(cx, kColYTop, kColYBot + 2.5f, 24.0f);
        cv.drawHorizontalFilament(cx - 15.0f, cx - 6.0f, kKneeY, 64.0f, 0.54f, true);
        cv.drawHorizontalFilament(cx + 6.0f, cx + 15.0f, kKneeY, 64.0f, 0.54f, true);
    }

    // 2. Linear-photon caustic columns: clean VCA stem, live acoustic plasma & MS-20 drive wings
    const int pyStart = (y0 > 32) ? y0 : 32;
    const int pyEnd = (y1 < 183) ? y1 : 183;
    if (pyStart < pyEnd) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        const float masterE = clampf(effAlpha01, 0.0f, 1.0f);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const int cxInt = 30 + static_cast<int>(v) * 60;
            const float cx = static_cast<float>(cxInt);
            const float fy = faderY_[v];
            const float pyPeak = peakY_[v];
            const float drv = driveNorm_[v];
            const float activeW = 1.0f - 0.78f * muteDim_[v];
            const float focBoost = (hud.focusedKnob == v) ? 1.18f : 1.0f;
            const int xLo = (cxInt - 22 > 2) ? (cxInt - 22) : 2;
            const int xHi = (cxInt + 22 < 237) ? (cxInt + 22) : 237;

            for (int py = pyStart; py < pyEnd; ++py) {
                const float pyF = static_cast<float>(py);
                uint16_t* linRow = linScratch + static_cast<size_t>(py - y0) * kDisplayWidth;

                const bool inStem = (pyF >= fy - 0.5f && pyF <= kColYBot + 0.5f);
                const bool inPlasma = (pyF >= pyPeak && pyF <= kColYBot);
                const bool inWings = (drv > 0.005f && pyF >= fy - 0.5f && pyF <= kKneeY + 1.0f);
                if (!inStem && !inPlasma && !inWings) {
                    continue;
                }

                float wingHalfW = 1.2f;
                float uOver = 0.0f;
                if (inWings) {
                    uOver = clampf((kKneeY - pyF) * (1.0f / (kKneeY - kColYTop)), 0.0f, 1.0f);
                    wingHalfW =
                        1.5f + (16.5f * uOver * (0.58f + 0.42f * uOver)) * (0.32f + 0.68f * drv);
                }

                for (int px = xLo; px <= xHi; ++px) {
                    const float dx = absf(static_cast<float>(px) - cx);
                    float energy = 0.0f;

                    if (inStem && dx < 4.5f) {
                        const float stemCore = fastExpNeg(dx * dx * 1.65f);
                        energy += 265.0f * stemCore * activeW * focBoost;
                    }
                    if (inPlasma && dx < 9.5f) {
                        const float sigInv2 = (pyF < kKneeY) ? 0.065f : 0.115f;
                        const float plasma = fastExpNeg(dx * dx * sigInv2);
                        energy += 155.0f * plasma * activeW;
                    }
                    if (inWings && dx <= wingHalfW + 3.2f) {
                        const float fill =
                            (dx <= wingHalfW)
                                ? (0.36f + 0.44f * (dx / (wingHalfW + 0.1f)))
                                : (0.80f * fastExpNeg((dx - wingHalfW) * (dx - wingHalfW) * 1.4f));
                        const float rimDist = dx - wingHalfW;
                        const float rim = fastExpNeg(rimDist * rimDist * 1.35f);
                        const float capDist = pyF - fy;
                        const float cap = (capDist >= -0.6f && capDist <= 2.2f && dx <= wingHalfW)
                                              ? fastExpNeg(capDist * capDist * 0.85f)
                                              : 0.0f;
                        energy += (185.0f * fill + 275.0f * rim + 210.0f * cap) *
                                  (0.35f + 0.65f * drv) * activeW * focBoost;
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

    // 3. Crest optical beads / overdrive coronas & compact drive-multiplier callouts
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const int cxInt = 30 + static_cast<int>(v) * 60;
        const float cx = static_cast<float>(cxInt);
        const float fy = faderY_[v];
        const float drv = driveNorm_[v];
        const float dim = muteDim_[v];
        const bool focused = (hud.focusedKnob == v);

        if (drv > 0.02f && dim < 0.85f) {
            const float beaconInt = (0.84f + 0.16f * (focused ? 1.0f : 0.0f)) * (1.0f - 0.6f * dim);
            cv.drawLfoPhaseBeacon(cx, fy, beaconInt, drv);

            char multStr[12]{};
            formatMultiplierX(multStr, sizeof(multStr), driveRatio_[v]);
            int lblY = static_cast<int>(fy) - 13;
            if (lblY < 25) {
                lblY = 25;
            }
            const uint8_t lblGray = focused ? 242u : 186u;
            cv.drawSansCentered(kFontSansSmall, cxInt, lblY, multStr, lblGray, 1);
        } else {
            const float beadInt = (0.80f + (0.34f - 0.80f) * dim) + (focused ? 0.18f : 0.0f);
            const float beadRad = focused ? 2.35f : 1.95f;
            cv.drawOpticalBead(cx, fy, clampf(beadInt, 0.25f, 1.0f), beadRad);
        }
    }
}

} // namespace ruido
