#include "ui/HudKeybedView.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/KeyPunchSpec.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"
#include "ui/HudFormat.h"
#include "ui/HudKeybedInspector.h"
#include "ui/HudShiftMap.h"
#include "ui/MonoCanvas.h"

namespace ruido {

namespace {

// Hardware-faithful rounded-capsule half-width shared identically across all 27 keys (`8.4 px`).
constexpr float kKeyHalfW = 4.2f;

// 4 piano natural-gap registration dots (`3` in Left 20-Pad Bay + `1` in Right 7-Switch Bay).
constexpr std::array<float, 4> kRegDotCenterX = {61.20f, 98.80f, 149.60f, 208.25f};

struct TopZoneSpan {
    float xL;
    float xR;
    int labelX;
    uint8_t firstKey;
    uint8_t keyCount;
};

// 3 model-specific pressure clusters across `OP1..GLO` (`Black 0..7`) in the Left 20-Pad Bay:
constexpr std::array<TopZoneSpan, 3> kLeftTopZoneSpans = {{
    {blackKeyDiagramCenterX(0u) - 3.2f, blackKeyDiagramCenterX(2u) + 3.2f, 36, 0u, 3u},  // OP1..OP3
    {blackKeyDiagramCenterX(3u) - 3.2f, blackKeyDiagramCenterX(4u) + 3.2f, 81, 3u, 2u},  // OP4..OP5
    {blackKeyDiagramCenterX(5u) - 3.2f, blackKeyDiagramCenterX(7u) + 3.2f, 125, 5u, 3u}, // OP6..GLO
}};

} // namespace

void compositeFm1KeybedDiagram(const MonoStripCanvas& cv, float topY,
                               const std::array<float, kNumBlackKeys>& blackGlow,
                               const std::array<float, kNumWhiteKeys>& whiteGlow,
                               const HudState* liveHud, uint16_t activeBlackMask,
                               uint16_t activeWhiteMask) noexcept {
    const float botY = topY + 72.0f;
    if (cv.y1 <= static_cast<int>(topY) - 2 || cv.y0 >= static_cast<int>(botY) + 3) {
        return;
    }

    // Two Separate Stadium Enclosures: Left 20-Pad Pressure Bay | Right 7-Switch Permanent Bay
    fillKeybedStadiumSDF(cv, 7.0f, topY, 166.5f, botY, 12.5f, 13u, 74u, 242u);
    fillKeybedStadiumSDF(cv, 170.5f, topY, 233.0f, botY, 12.5f, 13u, 74u, 242u);

    const uint16_t liveBlackMask =
        (liveHud != nullptr)
            ? static_cast<uint16_t>(liveHud->heldBlackMask | liveHud->latchedBlackMask)
            : 0u;
    const uint16_t liveWhiteMask =
        (liveHud != nullptr)
            ? static_cast<uint16_t>(liveHud->heldWhiteMask | liveHud->latchedWhiteMask)
            : 0u;

    const float bTop = topY + 6.0f;
    const float bBot = topY + 32.0f;
    const float bMidY = 0.5f * (bTop + bBot);
    if (cv.y1 > static_cast<int>(bTop) - 3 && cv.y0 <= static_cast<int>(bBot) + 3) {
        for (size_t i = 0u; i < kNumBlackKeys; ++i) {
            const float cx = blackKeyDiagramCenterX(i);
            const bool activeKey = ((activeBlackMask >> i) & 1u) != 0u;
            const float g = activeKey ? clampf(blackGlow[i], 0.0f, 1.0f) : 0.0f;
            const bool isPressurePad = isAnalogPressureBlackKey(i);
            const bool analogLatched =
                activeKey && isPressurePad && (((liveBlackMask >> i) & 1u) != 0u);
            const bool holdLatched = activeKey && (i == kHoldBlackKeyIdx) &&
                                     (liveHud != nullptr && liveHud->keyHoldLatched);
            const float pFill =
                (analogLatched && liveHud != nullptr)
                    ? (static_cast<float>(liveHud->blackPressureQ8[i]) * (1.0f / 255.0f))
                    : -1.0f;
            const bool physHeld =
                activeKey && (liveHud != nullptr) && (((liveHud->heldBlackMask >> i) & 1u) != 0u);
            const uint8_t fillGray = !activeKey ? 13u
                                     : physHeld ? 32u
                                     : (holdLatched && !physHeld)
                                         ? static_cast<uint8_t>(11.0f + 27.0f * g + 0.5f)
                                         : (analogLatched ? 24u : 19u);
            const uint8_t rimGray = !activeKey ? 34u
                                    : physHeld ? 250u
                                    : (holdLatched && !physHeld)
                                        ? static_cast<uint8_t>(52.0f + 200.0f * g + 0.5f)
                                        : (analogLatched ? 212u : 132u);
            fillKeybedStadiumSDF(cv, cx - kKeyHalfW, bTop, cx + kKeyHalfW, bBot, kKeyHalfW,
                                 fillGray, rimGray, 248u);
            if (isPressurePad) {
                fillKeybedSlitSDF(cv, cx, bTop + 4.2f, bBot - 4.2f, 1.08f, g, pFill, activeKey);
            } else {
                fillKeybedSwitchDotSDF(cv, cx, bMidY, g, activeKey);
                if (holdLatched && !physHeld && g > 0.25f) {
                    cv.drawOpticalBead(cx, bMidY, (g - 0.25f) * 0.85f, 2.5f);
                }
            }
        }
    }

    if (cv.y1 > static_cast<int>(topY) + 31 && cv.y0 <= static_cast<int>(topY) + 41) {
        for (float dotX : kRegDotCenterX) {
            cv.drawOpticalBead(dotX, topY + 36.0f, 0.38f, 0.95f);
        }
    }

    const float wTop = topY + 40.0f;
    const float wBot = topY + 66.0f;
    const float wMidY = 0.5f * (wTop + wBot);
    if (cv.y1 > static_cast<int>(wTop) - 3 && cv.y0 <= static_cast<int>(wBot) + 3) {
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float cx = whiteKeyDiagramCenterX(i);
            const bool activeKey = ((activeWhiteMask >> i) & 1u) != 0u;
            const float g = activeKey ? clampf(whiteGlow[i], 0.0f, 1.0f) : 0.0f;
            const bool isPressurePad = isAnalogPressureWhiteKey(i);
            const bool analogLatched =
                activeKey && isPressurePad && (((liveWhiteMask >> i) & 1u) != 0u);
            const float pFill =
                (analogLatched && liveHud != nullptr)
                    ? (static_cast<float>(liveHud->whitePressureQ8[i]) * (1.0f / 255.0f))
                    : -1.0f;
            const bool physHeld =
                activeKey && (liveHud != nullptr) && (((liveHud->heldWhiteMask >> i) & 1u) != 0u);
            const uint8_t fillGray =
                !activeKey ? 13u : (physHeld ? 32u : (analogLatched ? 24u : 19u));
            const uint8_t rimGray =
                !activeKey ? 34u : (physHeld ? 250u : (analogLatched ? 212u : 132u));
            fillKeybedStadiumSDF(cv, cx - kKeyHalfW, wTop, cx + kKeyHalfW, wBot, kKeyHalfW,
                                 fillGray, rimGray, 248u);
            if (isPressurePad) {
                fillKeybedSlitSDF(cv, cx, wTop + 4.2f, wBot - 4.2f, 1.12f, g, pFill, activeKey);
            } else {
                fillKeybedSwitchDotSDF(cv, cx, wMidY, g, activeKey);
            }
        }
    }
}

void formatKeybedTopHeader(const HudState& hud, char* leftMain, size_t leftMainCap,
                           char* leftSuffix, size_t leftSuffixCap, char* rightText, size_t rightCap,
                           bool& outShowBar, float& outBarNorm, bool* outShowSceneDots) noexcept {
    outShowBar = false;
    outBarNorm = 0.0f;
    if (outShowSceneDots != nullptr) {
        *outShowSceneDots = false;
    }

    if (hud.mode == HudMode::KeyboardModel) {
        if (hud.activePunch != KeyPunchKind::None) {
            formatKeyPunchReadout(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                                  rightCap, outShowBar, outBarNorm, outShowSceneDots);
            outShowBar = false;
            return;
        }
        appendStr(leftMain, 0u, leftMainCap, "KEYBOARD");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " ");
        sp = appendUInt(leftSuffix, sp, leftSuffixCap,
                        static_cast<uint32_t>(hud.keyboardModel) + 1u);
        sp = appendStr(leftSuffix, sp, leftSuffixCap, "/");
        appendUInt(leftSuffix, sp, leftSuffixCap, static_cast<uint32_t>(KeyboardModel::Count));
        return;
    }

    if (hud.mode == HudMode::ShiftMap) {
        if (hud.shiftFocus == ShiftFocus::Tempo) {
            appendStr(leftMain, 0u, leftMainCap, "TEMPO");
            outBarNorm = normalizeClockBpmLog01(hud.effectiveClockBpm);
            outShowBar = true;
            const bool showTenths =
                !hud.midiClockSynced && (hud.rateHoldActive && hud.knobRate == KnobRate::Fine);
            formatClockBpm(rightText, rightCap, hud.effectiveClockBpm, hud.midiClockSynced,
                           showTenths);
            return;
        }
        if (hud.shiftFocus == ShiftFocus::Scene ||
            hud.activePunch == KeyPunchKind::SnapshotRecall) {
            formatSnapshotHeader(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                                 rightCap, outShowBar, outBarNorm, outShowSceneDots);
            return;
        }
        if (hud.shiftFocus == ShiftFocus::Model) {
            appendStr(leftMain, 0u, leftMainCap, "KEYBOARD");
            size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " ");
            sp = appendUInt(leftSuffix, sp, leftSuffixCap,
                            static_cast<uint32_t>(hud.keyboardModel) + 1u);
            sp = appendStr(leftSuffix, sp, leftSuffixCap, "/");
            appendUInt(leftSuffix, sp, leftSuffixCap, static_cast<uint32_t>(KeyboardModel::Count));
            appendStr(rightText, 0u, rightCap, keyboardModelTitle(hud.keyboardModel));
            return;
        }
        if (hud.activePunch == KeyPunchKind::SpotlightSolo) {
            formatKeyPunchReadout(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                                  rightCap, outShowBar, outBarNorm, outShowSceneDots);
            return;
        }
        appendStr(leftMain, 0u, leftMainCap, "SHIFT");
        return;
    }

    formatKeyPunchReadout(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                          rightCap, outShowBar, outBarNorm, outShowSceneDots);
}

void HudKeybedView::reset() noexcept {
    blackGlow_.fill(0.0f);
    whiteGlow_.fill(0.0f);
    phase_ = 0.0f;
    shiftPhase_ = 0.0f;
    idleTimerSec_ = 0.0f;
    brightPhase_ = 1.0f;
    lastWakeSeq_ = 0u;
    primed_ = false;
    wasSelPageOpen_ = false;
}

void HudKeybedView::clearImmediate() noexcept {
    phase_ = 0.0f;
    shiftPhase_ = 0.0f;
    idleTimerSec_ = 0.0f;
    brightPhase_ = 1.0f;
    primed_ = false;
    wasSelPageOpen_ = false;
}

float HudKeybedView::brightnessScale() const noexcept {
    return kDimFloorBrightness + (1.0f - kDimFloorBrightness) * smoothstep01(brightPhase_);
}

void HudKeybedView::update(float dtClamped, const EngineControlState& control,
                           const HudState& rawHud,
                           const std::array<float, kNumOscillators>& voicePeak,
                           uint8_t stutterGateMask, float rhythmPhase01) noexcept {
    (void)voicePeak;
    (void)stutterGateMask;

    const bool anyKeyDown = (rawHud.heldBlackMask | rawHud.heldWhiteMask) != 0u;
    const bool selLikeOpen = rawHud.selPageOpen && !rawHud.shiftHeld;
    if (selLikeOpen && !wasSelPageOpen_) {
        idleTimerSec_ = 0.0f;
        brightPhase_ = 1.0f;
        lastWakeSeq_ = rawHud.selWakeSeq;
    } else if (selLikeOpen && (rawHud.selWakeSeq != lastWakeSeq_ || anyKeyDown)) {
        idleTimerSec_ = 0.0f;
        lastWakeSeq_ = rawHud.selWakeSeq;
    }
    wasSelPageOpen_ = selLikeOpen;

    if (selLikeOpen) {
        if (idleTimerSec_ < kIdleHoldSec) {
            idleTimerSec_ += dtClamped;
            if (brightPhase_ < 1.0f) {
                brightPhase_ = clampf(brightPhase_ + dtClamped * (1.0f / kWakeRiseSec), 0.0f, 1.0f);
            }
        } else if (brightPhase_ > 0.0f) {
            brightPhase_ = clampf(brightPhase_ - dtClamped * (1.0f / kIdleDimSec), 0.0f, 1.0f);
        }
    }

    const bool active = isActive(rawHud);
    const bool shiftAct = isShiftActive(rawHud);
    const bool wasInvisible = (phase_ <= 0.002f);
    phase_ = stepHudFadeLinear(phase_, active, dtClamped);
    shiftPhase_ = stepHudFadeLinear(shiftPhase_, shiftAct, dtClamped);
    if (!active && !shiftAct && !hasResidualPhase()) {
        primed_ = false;
        return;
    }

    std::array<float, kNumBlackKeys> tgtBlack{};
    std::array<float, kNumWhiteKeys> tgtWhite{};
    computeKeyboardModelRestLeds(rawHud.keyboardModel, control, rawHud, tgtBlack, tgtWhite);
    if (!rawHud.shiftHeld && rawHud.keyboardModel == KeyboardModel::RhythmLab) {
        tgtBlack[kTapBlackKeyIdx] = computeTapTempoBeatGlow(control, rhythmPhase01);
    }

    const uint16_t activeBlack =
        static_cast<uint16_t>(rawHud.heldBlackMask | rawHud.latchedBlackMask);
    const uint16_t activeWhite =
        static_cast<uint16_t>(rawHud.heldWhiteMask | rawHud.latchedWhiteMask);
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        if (i == kTapBlackKeyIdx && rawHud.keyboardModel != KeyboardModel::RhythmLab) {
            continue;
        }
        if (((activeBlack >> i) & 1u) != 0u) {
            tgtBlack[i] = 1.0f;
        }
    }
    for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
        if (((activeWhite >> i) & 1u) != 0u) {
            tgtWhite[i] = 1.0f;
        }
    }

    const bool snap = !primed_ || (active && wasInvisible);
    primed_ = true;
    const float alpha = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.020f)));
    const float alphaChop = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.004f)));
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        blackGlow_[i] = slewScalar(blackGlow_[i], tgtBlack[i], alpha, 0.002f);
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        const bool isChopKey =
            (i < kMuteWhiteKeyBase) && (control.keyMod.stutterDepth[i & 3u] > 1.0e-4f);
        whiteGlow_[i] =
            slewScalar(whiteGlow_[i], tgtWhite[i], isChopKey ? alphaChop : alpha, 0.002f);
    }
}

void HudKeybedView::compositeStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                   const HudState& frameHud) const noexcept {
    if (alpha8 == 0u || y1 <= 28 || y0 >= 230) {
        return;
    }
    const auto effAlpha8 =
        static_cast<uint8_t>(static_cast<float>(alpha8) * brightnessScale() + 0.5f);
    if (effAlpha8 == 0u) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, effAlpha8};
    const KeyboardModelSpec& spec = keyboardModelSpec(frameHud.keyboardModel);
    const auto modelIdx = static_cast<uint8_t>(frameHud.keyboardModel);
    constexpr uint8_t kModelCount = static_cast<uint8_t>(KeyboardModel::Count);

    // 1. Swappable Model Carousel & Clean Subtitle (`y = 33..69`)
    if (y1 > 30 && y0 < 72) {
        if (modelIdx > 0u) {
            cv.drawSans(kFontSansMedium, kKeybedCarouselArrowLeftX, kKeybedCarouselArrowY, "<",
                        132u, 0);
        }
        if (modelIdx + 1u < kModelCount) {
            cv.drawSansRight(kFontSansMedium, kKeybedCarouselArrowRightX, kKeybedCarouselArrowY,
                             ">", 132u, 0);
        }
        cv.drawSansCentered(kFontSansBold, 120, kKeybedCarouselTitleY, spec.title, 255u, 0);
        cv.drawSansCentered(kFontSansSmall, 120, 50, spec.subtitle, 168u, 1);

        constexpr float kBeadSpacing = 14.0f;
        const float firstX = 120.0f - 0.5f * static_cast<float>(kModelCount - 1u) * kBeadSpacing;
        for (uint8_t m = 0u; m + 1u < kModelCount; ++m) {
            const float fx0 = firstX + static_cast<float>(m) * kBeadSpacing + 3.0f;
            const float fx1 = firstX + static_cast<float>(m + 1u) * kBeadSpacing - 3.0f;
            cv.drawHorizontalFilament(fx0, fx1, 66.0f, 40.0f, 0.55f, true);
        }
        for (uint8_t m = 0u; m < kModelCount; ++m) {
            const float sx = firstX + static_cast<float>(m) * kBeadSpacing;
            const bool sel = (m == modelIdx);
            cv.drawOpticalBead(sx, 66.0f, sel ? 0.98f : 0.28f, sel ? 2.05f : 1.10f);
        }
    }

    // 2. Top-Row Brackets: 3 Pressure Clusters across Left Bay + LATCH / TAP / SHIFT above Right
    // Bay
    if (y1 > 70 && y0 < 92) {
        for (size_t z = 0u; z < kLeftTopZoneSpans.size(); ++z) {
            const TopZoneSpan& span = kLeftTopZoneSpans[z];
            float glowSum = 0.0f;
            for (uint8_t k = 0u; k < span.keyCount; ++k) {
                glowSum += blackGlow_[span.firstKey + k];
            }
            const float zoneGlow = glowSum * (1.0f / static_cast<float>(span.keyCount));
            const float brkLum = 56.0f + 92.0f * clampf(zoneGlow, 0.0f, 1.0f);
            cv.drawSansCentered(kFontSansSmall, span.labelX, 72, spec.topZones[z], 214u, 0);
            cv.drawHorizontalFilament(span.xL, span.xR, 87.0f, brkLum, 0.58f, true);
            cv.drawVerticalSlit(span.xL, 87.0f, 89.6f, brkLum);
            cv.drawVerticalSlit(span.xR, 87.0f, 89.6f, brkLum);
        }

        const float latchKeyX = blackKeyDiagramCenterX(kHoldBlackKeyIdx);
        const float latchLum = 56.0f + 92.0f * clampf(blackGlow_[kHoldBlackKeyIdx], 0.0f, 1.0f);
        const float shiftKeyX = blackKeyDiagramCenterX(kShiftBlackKeyIdx);
        const float shiftLum = 56.0f + 92.0f * clampf(blackGlow_[kShiftBlackKeyIdx], 0.0f, 1.0f);
        if (frameHud.keyboardModel == KeyboardModel::RhythmLab) {
            const float tapKeyX = blackKeyDiagramCenterX(kTapBlackKeyIdx);
            const float tapLum = 56.0f + 92.0f * clampf(blackGlow_[kTapBlackKeyIdx], 0.0f, 1.0f);
            cv.drawSansCentered(kFontSansSmall, 159, 72, "LATCH", 214u, 0);
            cv.drawVerticalSlit(159.0f, 84.5f, 87.0f, latchLum);
            cv.drawHorizontalFilament(159.0f, latchKeyX, 87.0f, latchLum, 0.58f, false);
            cv.drawVerticalSlit(latchKeyX, 87.0f, 89.6f, latchLum);

            cv.drawSansCentered(kFontSansSmall, 193, 72, "TAP", 214u, 0);
            cv.drawVerticalSlit(tapKeyX, 84.5f, 89.6f, tapLum);
        } else {
            cv.drawSansCentered(kFontSansSmall, static_cast<int>(latchKeyX + 0.5f), 72, "LATCH",
                                214u, 0);
            cv.drawVerticalSlit(latchKeyX, 84.5f, 89.6f, latchLum);
        }
        cv.drawSansCentered(kFontSansSmall, static_cast<int>(shiftKeyX + 0.5f), 72, "SHIFT", 214u,
                            0);
        cv.drawVerticalSlit(shiftKeyX, 84.5f, 89.6f, shiftLum);
    }

    // 3. Two-Bay Rounded-Capsule FM-1 Keybed Map (`y = 92..164`: lines for pads, dots for switches)
    const uint16_t activeBlackMask =
        (frameHud.keyboardModel == KeyboardModel::RhythmLab) ? 0x07FFu : 0x05FFu;
    compositeFm1KeybedDiagram(cv, kKeybedDiagramTopY, blackGlow_, whiteGlow_, &frameHud,
                              activeBlackMask, 0xFFFFu);

    // 4. Bottom 4 Zone Brackets & Labels (`y = 167..190`: 3 Pressure Banks + Permanent MUTE)
    if (y1 > 166 && y0 < 192) {
        for (size_t z = 0u; z < 4u; ++z) {
            const float xL = whiteKeyDiagramCenterX(z * 4u) - kKeybedBracketOverhangX;
            const float xR = whiteKeyDiagramCenterX(z * 4u + 3u) + kKeybedBracketOverhangX;
            const float cxF = 0.5f * (xL + xR);
            const float zoneGlow = 0.25f * (whiteGlow_[z * 4u] + whiteGlow_[z * 4u + 1u] +
                                            whiteGlow_[z * 4u + 2u] + whiteGlow_[z * 4u + 3u]);
            const float brkLum = 56.0f + 92.0f * clampf(zoneGlow, 0.0f, 1.0f);
            cv.drawHorizontalFilament(xL, xR, kKeybedBracketY, brkLum, 0.58f, true);
            cv.drawVerticalSlit(xL, kKeybedBracketTickTopY, kKeybedBracketY, brkLum);
            cv.drawVerticalSlit(xR, kKeybedBracketTickTopY, kKeybedBracketY, brkLum);
            const char* lbl = spec.bottomZones[z];
            const int trk = (measureSansText(kFontSansSmall, lbl, 0) > 46) ? -1 : 0;
            cv.drawSansCentered(kFontSansSmall, static_cast<int>(cxF + 0.5f), kKeybedBracketLabelY,
                                lbl, 222u, trk);
        }
    }

    // 5. Full-Width Horizontal Pressure Bar + Bottom Guidance / Live Effect Description
    if (y1 > 192 && y0 < 226 && frameHud.knobBank == KnobBankMode::None) {
        float press01 = 0.0f;
        const bool showPressBar = evalKeybedBottomPressureBar(frameHud, press01);
        if (showPressBar) {
            drawKeybedBottomPressureBar(cv, press01);
        }
        const char* desc = (frameHud.activePunch != KeyPunchKind::None)
                               ? keyPunchGuidanceDesc(frameHud.activePunch)
                               : "";
        if (desc[0] != '\0') {
            cv.drawSansCentered(kFontSansSmall, 120, showPressBar ? 208 : 206, desc, 195u, 0);
        } else {
            cv.drawSansCentered(kFontSansSmall, 120, 206, "TAP SEL OR TURN SELECT", 138u, 0);
        }
    }
}

void HudKeybedView::compositeShiftStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                        const EngineControlState& control,
                                        const HudState& frameHud) const noexcept {
    if (alpha8 == 0u || y1 <= 26 || y0 >= 230) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    compositeFullChassisShiftStrip(cv, control, frameHud);
}

} // namespace ruido
