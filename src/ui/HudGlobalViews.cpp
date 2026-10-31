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

constexpr float kCenterAxisX = 120.0f;
constexpr float kPanHalfSpanPx = 94.0f;
constexpr float kRailLeftX = kCenterAxisX - kPanHalfSpanPx;  // 26.0f
constexpr float kRailRightX = kCenterAxisX + kPanHalfSpanPx; // 214.0f

constexpr std::array<float, kNumOscillators> kLaneY = {54.0f, 88.0f, 122.0f, 156.0f};
constexpr std::array<const char*, kNumOscillators> kLaneNumeral = {"1", "2", "3", "4"};

inline void addLinearEnergy(uint16_t* linScratch, int rowRel, int px, float energy) noexcept {
    if (energy <= 0.5f) {
        return;
    }
    const auto add = static_cast<uint32_t>(energy + 0.5f);
    uint16_t& cell =
        linScratch[static_cast<size_t>(rowRel) * kDisplayWidth + static_cast<size_t>(px)];
    const uint32_t next = static_cast<uint32_t>(cell) + add;
    cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
}

} // namespace

void HudGlobalViews::reset() noexcept {
    panSigned_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panBase_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panOrbitMin_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panOrbitMax_ = {-0.65f, -0.22f, 0.22f, 0.65f};
    panLfoDepth_.fill(0.0f);
    voicePresence_.fill(0.55f);
    muteDim_.fill(0.0f);
    clearImmediate();
}

void HudGlobalViews::clearImmediate() noexcept {
    panPhase_ = 0.0f;
    configPhase_ = 0.0f;
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

    const bool snapPan = (panPhase_ <= 0.015f);
    panPhase_ = stepHudFadeLinear(panPhase_, panActive, dtClamped);
    configPhase_ = stepHudFadeLinear(configPhase_, cfgActive, dtClamped);

    if (!panActive) {
        return;
    }

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float alphaPos = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.032f));
    const float alphaAtk = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.025f));
    const float alphaRel = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.120f));

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
        const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
        const VoiceAudibility aud = computeVoiceAudibility(vp, control.voicePressure[v], pitchMode);
        const bool silentMute = aud.muteBlend >= 0.999f;

        const float liveAmpRaw = computeLiveVoiceAmplitude(vp, lfoA, lfoB);
        const float liveAmp = silentMute ? 0.0f : clampf(liveAmpRaw, 0.0f, 1.0f);
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

        const float peak01 = silentMute ? 0.0f : clampf(voicePeak[v], 0.0f, 1.25f);
        const float tgtPresence =
            silentMute ? 0.16f : clampf(0.36f + 0.42f * liveAmp + 0.34f * peak01, 0.0f, 1.0f);

        if (snapPan) {
            panSigned_[v] = livePan;
            panBase_[v] = basePan;
            panOrbitMin_[v] = pLo;
            panOrbitMax_[v] = pHi;
            panLfoDepth_[v] = modDepth;
            voicePresence_[v] = tgtPresence;
            muteDim_[v] = aud.inaudibleDim;
        } else {
            panSigned_[v] = slewScalar(panSigned_[v], livePan, alphaPos, 0.001f);
            panBase_[v] = slewScalar(panBase_[v], basePan, alphaPos, 0.001f);
            panOrbitMin_[v] = slewScalar(panOrbitMin_[v], pLo, alphaPos, 0.001f);
            panOrbitMax_[v] = slewScalar(panOrbitMax_[v], pHi, alphaPos, 0.001f);
            panLfoDepth_[v] = slewScalar(panLfoDepth_[v], modDepth, alphaPos, 0.002f);
            const float aPres = (tgtPresence >= voicePresence_[v]) ? alphaAtk : alphaRel;
            voicePresence_[v] = slewScalar(voicePresence_[v], tgtPresence, aPres, 0.002f);
            muteDim_[v] = slewScalar(muteDim_[v], aud.inaudibleDim, alphaPos, 0.002f);
        }
    }
}

void HudGlobalViews::compositePanStageStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                            uint16_t* linScratch, float effPanAlpha01,
                                            uint8_t alpha8, const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 26 || y0 >= 182) {
        return;
    }

    // 1. Linear-photon Binaural Radiation, Bipolar Center Wake & Constant-Power Speaker-Rail Glow
    if (effPanAlpha01 > 0.004f && y1 > 38 && y0 < 174) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));

        const bool globalPanAll =
            (hud.knobBank == KnobBankMode::GlobalAmp) && (hud.focusedKnob == 3u);

        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float laneY = kLaneY[v];
            const int bandTop = static_cast<int>(laneY) - 14;
            const int bandBot = static_cast<int>(laneY) + 15;
            if (bandBot <= y0 || bandTop >= y1) {
                continue;
            }

            const float visScale = 1.0f - 0.72f * muteDim_[v];
            const bool focused = globalPanAll || (hud.focusedKnob == static_cast<uint8_t>(v));
            const float focusGain = focused ? 1.24f : 1.0f;
            const float presence = voicePresence_[v];
            const float pan = panSigned_[v];
            const float panX = kCenterAxisX + pan * kPanHalfSpanPx;

            float gainL = 0.7071f;
            float gainR = 0.7071f;
            equalPowerPan(pan, gainL, gainR);

            const float eScale = effPanAlpha01 * visScale * focusGain;
            const float halfHL = 2.0f + 10.5f * gainL * presence;
            const float halfHR = 2.0f + 10.5f * gainR * presence;

            // Asymmetric binaural lobe widths weighted by DSP equal-power speaker gains
            const float sigXL = 5.5f + 12.5f * gainL * presence;
            const float sigXR = 5.5f + 12.5f * gainR * presence;
            const float inv2SigXL2 = 1.0f / (2.0f * sigXL * sigXL);
            const float inv2SigXR2 = 1.0f / (2.0f * sigXR * sigXR);
            const float sigY = 3.0f + 4.4f * presence;
            const float inv2SigY2 = 1.0f / (2.0f * sigY * sigY);

            const float wakeLo = (panX < kCenterAxisX) ? panX : kCenterAxisX;
            const float wakeHi = (panX > kCenterAxisX) ? panX : kCenterAxisX;
            const float wakeSpan = wakeHi - wakeLo;
            const float invWakeSpan = (wakeSpan > 1.0f) ? (1.0f / wakeSpan) : 0.0f;

            const int pyS = (bandTop < y0) ? y0 : bandTop;
            const int pyE = (bandBot > y1) ? y1 : bandBot;

            const int auraX0 = (static_cast<int>(panX - 2.6f * sigXL) > 20)
                                   ? static_cast<int>(panX - 2.6f * sigXL)
                                   : 20;
            const int auraX1 = (static_cast<int>(panX + 2.6f * sigXR) + 1 < 220)
                                   ? (static_cast<int>(panX + 2.6f * sigXR) + 1)
                                   : 220;
            const int wakeX0 = (static_cast<int>(wakeLo) > 26) ? static_cast<int>(wakeLo) : 26;
            const int wakeX1 =
                (static_cast<int>(wakeHi) + 1 < 215) ? (static_cast<int>(wakeHi) + 1) : 215;

            for (int py = pyS; py < pyE; ++py) {
                const int rowRel = py - y0;
                const float dy = static_cast<float>(py) - laneY;
                const float absDy = absf(dy);
                const float dy2 = dy * dy;

                // (a) Left & Right Speaker-Rail Constant-Power Projection Glows (x = 26, x = 214)
                if (absDy <= halfHL + 3.0f && gainL > 0.02f) {
                    const float overL = (absDy > halfHL) ? (absDy - halfHL) : 0.0f;
                    const float wyL = fastExpNeg(overL * overL * 0.75f);
                    const float peakEL = (85.0f + 310.0f * gainL * presence) * wyL * eScale;
                    for (int px = 22; px <= 30; ++px) {
                        const float dx = static_cast<float>(px) - kRailLeftX;
                        addLinearEnergy(linScratch, rowRel, px,
                                        peakEL * fastExpNeg(dx * dx * 0.55f));
                    }
                }
                if (absDy <= halfHR + 3.0f && gainR > 0.02f) {
                    const float overR = (absDy > halfHR) ? (absDy - halfHR) : 0.0f;
                    const float wyR = fastExpNeg(overR * overR * 0.75f);
                    const float peakER = (85.0f + 310.0f * gainR * presence) * wyR * eScale;
                    for (int px = 210; px <= 218; ++px) {
                        const float dx = static_cast<float>(px) - kRailRightX;
                        addLinearEnergy(linScratch, rowRel, px,
                                        peakER * fastExpNeg(dx * dx * 0.55f));
                    }
                }

                // (b) Tapered Bipolar Displacement Wake from Phantom Center (x = 120) to panX
                if (wakeSpan > 1.5f && absDy <= 4.5f) {
                    const float wyWake = fastExpNeg(dy2 * 0.42f);
                    const float wakeBase = (64.0f + 165.0f * presence) * wyWake * eScale;
                    for (int px = wakeX0; px < wakeX1; ++px) {
                        const float distFromCenter = absf(static_cast<float>(px) - kCenterAxisX);
                        const float t = clampf(distFromCenter * invWakeSpan, 0.0f, 1.0f);
                        addLinearEnergy(linScratch, rowRel, px, wakeBase * (0.22f + 0.78f * t * t));
                    }
                }

                // (c) Asymmetric Binaural Acoustic Radiation Field around (panX, laneY)
                const float wyAura = fastExpNeg(dy2 * inv2SigY2);
                if (wyAura > 0.015f) {
                    const float auraPeak = (120.0f + 340.0f * presence) * wyAura * eScale;
                    for (int px = auraX0; px < auraX1; ++px) {
                        const float dx = static_cast<float>(px) - panX;
                        const float wx =
                            fastExpNeg(dx * dx * ((dx < 0.0f) ? inv2SigXL2 : inv2SigXR2));
                        addLinearEnergy(linScratch, rowRel, px, auraPeak * wx);
                    }
                }
            }
        }

        screenBlendLinearStrip(stripHeight, 0u, kDisplayWidth, linScratch, lumBuf);
    }

    // 2. Crisp Foreground Landmarks, Constant-Power Rail Cores, LFO Stereo Orbits & Focal Beads
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    cv.drawSansCentered(kFontSansSmall, static_cast<int>(kRailLeftX), 28, "L", 136u, 0);
    cv.drawSansCentered(kFontSansSmall, static_cast<int>(kCenterAxisX), 28, "C", 136u, 0);
    cv.drawSansCentered(kFontSansSmall, static_cast<int>(kRailRightX), 28, "R", 136u, 0);

    cv.drawVerticalSlit(kCenterAxisX, 40.0f, 170.0f, 36.0f);
    cv.drawVerticalSlit(kRailLeftX, 40.0f, 170.0f, 26.0f);
    cv.drawVerticalSlit(kRailRightX, 40.0f, 170.0f, 26.0f);

    const bool globalPanAll = (hud.knobBank == KnobBankMode::GlobalAmp) && (hud.focusedKnob == 3u);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float laneY = kLaneY[v];
        const float visScale = 1.0f - 0.72f * muteDim_[v];
        const bool focused = globalPanAll || (hud.focusedKnob == static_cast<uint8_t>(v));
        const float presence = voicePresence_[v];
        const float pan = panSigned_[v];
        const float panX = kCenterAxisX + pan * kPanHalfSpanPx;

        float gainL = 0.7071f;
        float gainR = 0.7071f;
        equalPowerPan(pan, gainL, gainR);

        // Left-margin lane numeral ("1".."4") dimmed 1-to-1 with voice audibility
        const auto lblLum =
            static_cast<uint8_t>(clampf((focused ? 238.0f : 152.0f) * visScale, 28.0f, 255.0f));
        cv.drawSans(kFontSansSmall, 8, static_cast<int>(laneY) - 3, kLaneNumeral[v], lblLum, 0);

        // Subtle horizontal stereo track guide
        cv.drawHorizontalFilament(kRailLeftX, kRailRightX, laneY,
                                  (focused ? 32.0f : 20.0f) * visScale, 0.50f, false);

        // Razor-sharp constant-power L/R speaker projection cores
        const float halfHL = 1.6f + 10.0f * gainL * presence;
        const float halfHR = 1.6f + 10.0f * gainR * presence;
        if (gainL > 0.02f) {
            cv.drawVerticalSlit(kRailLeftX, laneY - halfHL, laneY + halfHL,
                                (64.0f + 186.0f * gainL * presence) * visScale);
        }
        if (gainR > 0.02f) {
            cv.drawVerticalSlit(kRailRightX, laneY - halfHR, laneY + halfHR,
                                (64.0f + 186.0f * gainR * presence) * visScale);
        }

        // LFO -> PAN stereo orbit bracket or static optical focal bead
        const float orbitX0 = kCenterAxisX + panOrbitMin_[v] * kPanHalfSpanPx;
        const float orbitX1 = kCenterAxisX + panOrbitMax_[v] * kPanHalfSpanPx;
        const float beadInt = clampf((focused ? 1.0f : 0.88f) * visScale, 0.14f, 1.0f);

        if (orbitX1 - orbitX0 > 1.5f) {
            const float baseX = kCenterAxisX + panBase_[v] * kPanHalfSpanPx;
            cv.drawHorizontalFilament(orbitX0, orbitX1, laneY, 132.0f * visScale, 0.70f, false);
            cv.drawVerticalSlit(orbitX0, laneY - 3.5f, laneY + 3.5f, 148.0f * visScale);
            cv.drawVerticalSlit(orbitX1, laneY - 3.5f, laneY + 3.5f, 148.0f * visScale);
            cv.drawVerticalSlit(baseX, laneY - 2.2f, laneY + 2.2f, 96.0f * visScale);
            cv.drawLfoPhaseBeacon(panX, laneY, beadInt, panLfoDepth_[v]);
        } else {
            cv.drawOpticalBead(panX, laneY, beadInt, (focused ? 2.35f : 1.95f) + 0.50f * presence);
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
