#include "ui/HudSceneView.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardModelSpec.h"
#include "ui/FontSansAA.h"
#include "ui/HudKeybedInspector.h"
#include "ui/HudKeybedView.h"
#include "ui/MonoCanvas.h"

namespace ruido {

namespace {

void compositeSceneKeybedAndBracket(const MonoStripCanvas& cv,
                                    const std::array<float, kNumSnapshots>& sceneGlow,
                                    uint16_t activeWhiteMask, const HudState* liveHud) noexcept {
    std::array<float, kNumBlackKeys> blackGlow{};
    std::array<float, kNumWhiteKeys> whiteGlow{};
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        whiteGlow[kSceneWhiteKeyBase + s] = sceneGlow[s];
    }
    compositeFm1KeybedDiagram(cv, kKeybedDiagramTopY, blackGlow, whiteGlow, liveHud, 0x0000u,
                              activeWhiteMask);

    if (cv.y1 > 166 && cv.y0 < 194) {
        const float xL = whiteKeyDiagramCenterX(kSceneWhiteKeyBase) - kKeybedBracketOverhangX;
        const float xR = whiteKeyDiagramCenterX(kSceneWhiteKeyBase + kNumSnapshots - 1u) +
                         kKeybedBracketOverhangX;
        const float cxF = 0.5f * (xL + xR);
        const float zoneGlow = 0.25f * (sceneGlow[0] + sceneGlow[1] + sceneGlow[2] + sceneGlow[3]);
        const float brkLum = 56.0f + 92.0f * clampf(zoneGlow, 0.0f, 1.0f);
        cv.drawHorizontalFilament(xL, xR, kKeybedBracketY, brkLum, 0.58f, true);
        cv.drawVerticalSlit(xL, kKeybedBracketTickTopY, kKeybedBracketY, brkLum);
        cv.drawVerticalSlit(xR, kKeybedBracketTickTopY, kKeybedBracketY, brkLum);
        cv.drawSansCentered(kFontSansSmall, static_cast<int>(cxF + 0.5f), kKeybedBracketLabelY,
                            "SCENE", 222u, 0);
    }
}

} // namespace

void HudSceneView::reset() noexcept {
    sceneKeyGlow_.fill(0.0f);
    savePhase_ = 0.0f;
    morphPhase_ = 0.0f;
    latchedOccupiedMask_ = 0x0Fu;
    latchedSlot_ = 0u;
    latchedDeleteMode_ = false;
    primed_ = false;
}

void HudSceneView::clearImmediate() noexcept {
    savePhase_ = 0.0f;
    morphPhase_ = 0.0f;
    primed_ = false;
}

void HudSceneView::update(float dtClamped, const HudState& effectiveHud) noexcept {
    const bool saveAct = isSaveActive(effectiveHud);
    const bool morphAct = isPresetSceneActive(effectiveHud);
    const bool wasInvisible = !hasResidualPhase();

    savePhase_ = stepHudFadeLinear(savePhase_, saveAct, dtClamped);
    morphPhase_ = stepHudFadeLinear(morphPhase_, morphAct, dtClamped);

    if (!saveAct && !morphAct) {
        if (!hasResidualPhase()) {
            primed_ = false;
        }
        return;
    }

    const bool snap = !primed_ || wasInvisible;
    primed_ = true;
    const float alpha = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.016f)));

    if (saveAct) {
        latchedOccupiedMask_ = effectiveHud.snapshotOccupiedMask;
        latchedSlot_ = effectiveHud.snapshotSlot & 3u;
        latchedDeleteMode_ = effectiveHud.saveDeleteMode;

        const float savePulse01 = static_cast<float>(effectiveHud.savePulseQ8) * (1.0f / 255.0f);
        const float pulseGlow = 0.25f + 0.70f * clampf(savePulse01, 0.0f, 1.0f);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const bool occ = (effectiveHud.snapshotOccupiedMask & (1u << s)) != 0u;
            const float tgt = (!effectiveHud.saveDeleteMode || occ) ? pulseGlow : 0.0f;
            sceneKeyGlow_[s] = slewScalar(sceneKeyGlow_[s], tgt, alpha, 0.002f);
        }
        return;
    }

    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        const float tgt = computeSceneKeyBrightness(s, effectiveHud, true);
        sceneKeyGlow_[s] = slewScalar(sceneKeyGlow_[s], tgt, alpha, 0.002f);
    }
}

void HudSceneView::compositeSaveStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                      const HudState& frameHud) const noexcept {
    if (alpha8 == 0u || y1 <= 30 || y0 >= 234) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    const bool liveSave = isSaveActive(frameHud);
    const uint8_t occMask = liveSave ? frameHud.snapshotOccupiedMask : latchedOccupiedMask_;
    const uint8_t actSlot = liveSave ? (frameHud.snapshotSlot & 3u) : latchedSlot_;
    const bool deleteMode = liveSave ? frameHud.saveDeleteMode : latchedDeleteMode_;

    // 1. SEL-Matched Mode Carousel Banner (`y = 33..49`: `STORE SCENE >` <-> `< DELETE SCENE`)
    if (y1 > 30 && y0 < 52) {
        if (deleteMode) {
            cv.drawSans(kFontSansMedium, kKeybedCarouselArrowLeftX, kKeybedCarouselArrowY, "<",
                        132u, 0);
        } else {
            cv.drawSansRight(kFontSansMedium, kKeybedCarouselArrowRightX, kKeybedCarouselArrowY,
                             ">", 132u, 0);
        }
        cv.drawSansCentered(kFontSansBold, 120, kKeybedCarouselTitleY,
                            deleteMode ? "DELETE SCENE" : "STORE SCENE", 255u, 0);
    }

    // 2. Open, Container-Free 4-Slot Status Columns (`ACTIVE` / `STORED` / `EMPTY`)
    if (y1 > 52 && y0 < 84) {
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const int cx = static_cast<int>(kSceneNodeX[s] + 0.5f);
            const bool occ = (occMask & (1u << s)) != 0u;
            const char numStr[2] = {static_cast<char>('1' + s), '\0'};
            const char* statusStr = !occ ? "EMPTY" : ((s == actSlot) ? "ACTIVE" : "STORED");
            uint8_t numGray = 72u;
            uint8_t statusGray = 64u;
            if (deleteMode) {
                if (occ) {
                    numGray = (s == actSlot) ? 255u : 224u;
                    statusGray = (s == actSlot) ? 242u : 206u;
                }
            } else if (!occ) {
                numGray = 148u;
                statusGray = 132u;
            } else if (s == actSlot) {
                numGray = 255u;
                statusGray = 242u;
            } else {
                numGray = 204u;
                statusGray = 188u;
            }
            cv.drawSansCentered(kFontSansSmall, cx, 55, numStr, numGray, 0);
            cv.drawSansCentered(kFontSansSmall, cx, 69, statusStr, statusGray, 0);
        }
    }

    // 3. Shared 27-Key FM-1 Hardware Diagram + `"SCENE"` Bracket (pure blinking, zero key-hold
    // snap)
    const uint16_t activeWhiteMask =
        deleteMode ? static_cast<uint16_t>((occMask & 0x0Fu) << kSceneWhiteKeyBase) : 0x0F00u;
    compositeSceneKeybedAndBracket(cv, sceneKeyGlow_, activeWhiteMask, nullptr);

    // 4. Calm 2-Line Footer (`y = 202..228`; suppressed when `KNOB 1-4` strip wakes)
    if (y1 > 200 && y0 < 234 && frameHud.knobBank == KnobBankMode::None) {
        const char* line1 = !deleteMode       ? "TAP SCENE KEY TO STORE"
                            : (occMask == 0u) ? "NO STORED SCENES"
                                              : "TAP SCENE KEY TO DELETE";
        const char* line2 =
            !deleteMode ? "TAP SAVE OR SELECT TO DELETE" : "TAP SAVE OR HOME TO EXIT";
        cv.drawSansCentered(kFontSansSmall, 120, kKeybedFooterLine1Y, line1, 148u, 0);
        cv.drawSansCentered(kFontSansSmall, 120, kKeybedFooterLine2Y, line2, 122u, 0);
    }
}

void HudSceneView::compositePresetSceneStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                             const HudState& frameHud) const noexcept {
    if (alpha8 == 0u || y1 <= 54 || y0 >= 224) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};

    // 1. Only the 4 Scene Keys (`1 · 2 · 3 · 4`) mimicking real FM-1 LED-slit keys (zero pressure
    // bars, zero non-Scene keyboard keys)
    constexpr std::array<float, kNumSnapshots> kPresetKeyX = {66.0f, 102.0f, 138.0f, 174.0f};
    constexpr float kPresetKeyTopY = 80.0f;
    constexpr float kPresetKeyBotY = 128.0f;
    constexpr float kPresetKeyHalfW = 7.2f;

    if (y1 > 56 && y0 < 134) {
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const float cx = kPresetKeyX[s];
            const bool occ = (frameHud.snapshotOccupiedMask & (1u << s)) != 0u;
            const float g = clampf(sceneKeyGlow_[s], 0.0f, 1.0f);
            const float w01 = occ ? clampf((g - 0.40f) * (1.0f / 0.40f), 0.0f, 1.0f) : 0.0f;
            const char numStr[2] = {static_cast<char>('1' + s), '\0'};
            const auto numGray = !occ ? static_cast<uint8_t>(68u)
                                      : static_cast<uint8_t>(120.0f + 135.0f * w01 + 0.5f);
            cv.drawSansCentered(kFontSansSmall, static_cast<int>(cx + 0.5f), 62, numStr, numGray,
                                0);

            const auto fillGray =
                !occ ? static_cast<uint8_t>(11u) : static_cast<uint8_t>(16.0f + 14.0f * w01 + 0.5f);
            const auto rimGray = !occ ? static_cast<uint8_t>(56u)
                                      : static_cast<uint8_t>(110.0f + 138.0f * w01 + 0.5f);
            fillKeybedStadiumSDF(cv, cx - kPresetKeyHalfW, kPresetKeyTopY, cx + kPresetKeyHalfW,
                                 kPresetKeyBotY, kPresetKeyHalfW, fillGray, rimGray, 248u);
            fillKeybedSlitSDF(cv, cx, kPresetKeyTopY + 7.5f, kPresetKeyBotY - 7.5f, 1.35f,
                              !occ ? 0.15f : (0.38f + 0.62f * w01), -1.0f, true);
        }
    }

    // 2. Calm 2-line SHIFT + hardware location guidance below
    if (y1 > 184 && y0 < 224 && frameHud.knobBank == KnobBankMode::None) {
        cv.drawSansCentered(kFontSansSmall, 120, 188, "HOLD SHIFT FOR SCENE KEYS", 158u, 0);
        cv.drawSansCentered(kFontSansSmall, 120, 203, "SHIFT = LIT TOP-RIGHT KEY", 116u, 0);
    }
}

} // namespace ruido
