#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/FontSansAA.h"
#include "ui/HudKeybedInspector.h"
#include "ui/HudKeybedView.h"
#include "ui/MonoCanvas.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Anti-aliased angled encoder pointer slit (`turns01`: `0.00` = 12 o'clock, `0.25` = 3 o'clock).
inline void drawShiftEncoderPointer(const MonoStripCanvas& cv, float cx, float cy, float turns01,
                                    float peakLum) noexcept {
    const float effPeak = peakLum * (static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f));
    if (effPeak <= 0.5f || cv.y1 <= static_cast<int>(cy) - 6 || cv.y0 > static_cast<int>(cy) + 6) {
        return;
    }
    const float p = wrapLfoPhase01(turns01);
    const float dx = evalAnalogSine(p);
    const float dy = -evalAnalogSine(wrapLfoPhase01(p + 0.25f));
    constexpr float kR0 = 1.0f;
    constexpr float kR1 = 4.6f;
    constexpr int kSteps = 6;
    for (int s = 0; s <= kSteps; ++s) {
        const float r = kR0 + (kR1 - kR0) * (static_cast<float>(s) * (1.0f / kSteps));
        const float sx = cx + dx * r;
        const float sy = cy + dy * r;
        const int py0 = (static_cast<int>(sy) - 1 < cv.y0) ? cv.y0 : (static_cast<int>(sy) - 1);
        const int py1 =
            (static_cast<int>(sy) + 1 > cv.y1 - 1) ? (cv.y1 - 1) : (static_cast<int>(sy) + 1);
        const int px0 = (static_cast<int>(sx) - 1 < 0) ? 0 : (static_cast<int>(sx) - 1);
        const int px1 = (static_cast<int>(sx) + 1 > static_cast<int>(kDisplayWidth) - 1)
                            ? (static_cast<int>(kDisplayWidth) - 1)
                            : (static_cast<int>(sx) + 1);
        for (int py = py0; py <= py1; ++py) {
            uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
            const float ey = static_cast<float>(py) - sy;
            for (int px = px0; px <= px1; ++px) {
                const float ex = static_cast<float>(px) - sx;
                const float w = fastExpNeg((ex * ex + ey * ey) * 2.2f);
                const uint32_t a8 = static_cast<uint32_t>(effPeak * w * 0.62f + 0.5f);
                if (a8 > 0u) {
                    row[px] = blendLum8(row[px], 255u, static_cast<uint8_t>(a8 > 255u ? 255u : a8));
                }
            }
        }
    }
}

// Hardware-accurate Full-Chassis M-VAVE FM-1 `SHIFT` Map (`HudMode::ShiftMap`, `0 B` extra RAM):
// Keeps exact `1.75:1` FM-1 proportions (`224 x 128 px`), lighting & labeling strictly controls
// with dedicated `SHIFT` functions directly over the dim ghosted chassis (zero knockout boxes):
//  1. `SELECT` (`KEYBOARD`) & `ALGORITHM` (`TEMPO`) encoders (`MASTER` & `PRESETS` stay ghosted)
//  2. `OP1-OP3` (`KEYBOARD` centered directly above `OP1-OP3` with downward bracket)
//  3. `Keys 9-12` (`SCENE` pressure pads) & `Keys 13-16` (`SOLO` switches) with bottom brackets
//  4. `11th` corner key (`SHIFT` switch at `100%`) + `220 ms` `modeWipeQ8` LED wave sync
inline void compositeFullChassisShiftStrip(const MonoStripCanvas& cv,
                                           const EngineControlState& control, const HudState& hud,
                                           void (*coProcYield)() noexcept = nullptr) noexcept {
    const bool modelHot = (hud.shiftFocus == ShiftFocus::Model);
    const bool tempoHot = (hud.shiftFocus == ShiftFocus::Tempo);
    const bool sceneHot =
        (hud.shiftFocus == ShiftFocus::Scene) || (hud.activePunch == KeyPunchKind::SnapshotRecall);
    const bool soloHot = (hud.activePunch == KeyPunchKind::SpotlightSolo);
    const float wipe = static_cast<float>(hud.modeWipeQ8) * (1.0f / 255.0f);
    const float wipeProg = 1.0f - wipe;

    // 1. Outer FM-1 hardware chassis (`224 x 128 px` = `1.75 : 1` true FM-1 aspect ratio)
    if (cv.y1 > 28 && cv.y0 < 161) {
        fillKeybedStadiumSDF(cv, 8.0f, 30.0f, 232.0f, 158.0f, 11.5f, 8u, 48u, 246u);
        if (coProcYield != nullptr) {
            coProcYield();
        }
    }

    // 2. Upper Panel (`y = 34..85`): ghosted non-SHIFT hardware drawn first, then dedicated SHIFT
    // encoders (`SELECT` & `ALGORITHM`) + direct foreground labels!
    if (cv.y1 > 33 && cv.y0 < 87) {
        fillKeybedStadiumSDF(cv, 22.8f, 39.8f, 33.2f, 50.2f, 5.2f, 12u, 32u, 242u); // MASTER
        fillKeybedStadiumSDF(cv, 22.3f, 56.8f, 33.7f, 68.2f, 5.7f, 12u, 32u, 242u); // PRESETS

        // Recessed `[OCT- | OCT+]` bezel directly under `[PRESETS | ALGORITHM]`
        fillKeybedStadiumSDF(cv, 20.0f, 73.0f, 58.0f, 83.0f, 2.6f, 9u, 30u, 244u);
        const bool fineOn = hud.rateHoldActive && hud.knobRate == KnobRate::Fine;
        const bool coarseOn = hud.rateHoldActive && hud.knobRate == KnobRate::Coarse;
        fillKeybedStadiumSDF(cv, 21.8f, 74.5f, 38.2f, 81.5f, 1.8f, fineOn ? 44u : 12u,
                             fineOn ? 204u : 28u, 244u);
        fillKeybedStadiumSDF(cv, 39.8f, 74.5f, 56.2f, 81.5f, 1.8f, coarseOn ? 44u : 12u,
                             coarseOn ? 204u : 28u, 244u);

        // Ghosted center-left `1:1` square screen glass bezel (`48x48`) + inner LCD (`34x34`)
        fillKeybedStadiumSDF(cv, 66.0f, 36.0f, 114.0f, 84.0f, 5.5f, 9u, 40u, 246u);
        fillKeybedStadiumSDF(cv, 73.0f, 43.0f, 107.0f, 77.0f, 1.8f, 10u, 30u, 244u);
        if (coProcYield != nullptr) {
            coProcYield();
        }

        // Ghosted right `4` encoders (`KNOB 1-4`) + recessed `2 x 6` button grid bezel
        for (int k = 0; k < 4; ++k) {
            const float kx = 134.0f + static_cast<float>(k) * 24.0f;
            fillKeybedStadiumSDF(cv, kx - 5.2f, 39.8f, kx + 5.2f, 50.2f, 5.2f, 12u, 30u, 240u);
        }
        fillKeybedStadiumSDF(cv, 123.0f, 56.0f, 219.0f, 84.0f, 3.2f, 9u, 30u, 244u);
        for (int c = 0; c < 6; ++c) {
            const float bx = 132.25f + static_cast<float>(c) * 15.5f;
            fillKeybedStadiumSDF(cv, bx - 6.0f, 58.5f, bx + 6.0f, 68.5f, 1.8f, 12u, 26u, 238u);
            fillKeybedStadiumSDF(cv, bx - 6.0f, 71.5f, bx + 6.0f, 81.5f, 1.8f, 12u, 26u, 238u);
            if (coProcYield != nullptr && c == 2) {
                coProcYield();
            }
        }
        if (coProcYield != nullptr) {
            coProcYield();
        }

        // Dedicated SHIFT encoders: SELECT (`KEYBOARD`) & ALGORITHM (`TEMPO`)
        const float selTurns =
            (static_cast<float>(static_cast<uint8_t>(hud.keyboardModel)) - 1.0f) * 0.111f;
        fillKeybedStadiumSDF(cv, 44.3f, 39.3f, 55.7f, 50.7f, 5.7f, modelHot ? 48u : 19u,
                             modelHot ? 255u : 156u, 248u);
        drawShiftEncoderPointer(cv, 50.0f, 45.0f, selTurns, modelHot ? 255.0f : 184.0f);
        if (modelHot) {
            cv.drawOpticalBead(50.0f, 45.0f, 0.92f, 1.85f);
        }
        cv.drawHorizontalFilament(57.5f, 65.5f, 45.0f, modelHot ? 242.0f : 152.0f, 0.58f, false);
        cv.drawSans(kFontSansSmall, 68, 40, "KEYBOARD", modelHot ? 255u : 236u, 0);

        const float algoTurns = -0.375f + 0.75f * normalizeClockBpmLog01(hud.effectiveClockBpm);
        fillKeybedStadiumSDF(cv, 44.3f, 56.8f, 55.7f, 68.2f, 5.7f, tempoHot ? 48u : 19u,
                             tempoHot ? 255u : 156u, 250u);
        drawShiftEncoderPointer(cv, 50.0f, 62.5f, algoTurns, tempoHot ? 255.0f : 184.0f);
        if (tempoHot) {
            cv.drawOpticalBead(50.0f, 62.5f, 0.94f, 1.90f);
        }
        cv.drawHorizontalFilament(57.5f, 65.5f, 62.5f, tempoHot ? 242.0f : 152.0f, 0.58f, false);
        cv.drawSans(kFontSansSmall, 68, 57, "TEMPO", tempoHot ? 255u : 236u, 0);
    }

    // 3. Single Continuous Full-Width 27-Key Stadium Well (`x = 16..224, y = 92..151`)
    constexpr float kWhiteBaseX = 28.5f;
    constexpr float kWhitePitchX = 12.2f;
    constexpr float kCapsuleHalfW = 4.1f;
    constexpr std::array<uint8_t, kNumBlackKeys> kBlackPianoGap = {
        0u, 1u, 2u, 4u, 5u, 7u, 8u, 9u, 11u, 12u, 14u,
    };
    constexpr std::array<uint8_t, 4> kNaturalDotGap = {3u, 6u, 10u, 13u};
    const auto shiftWhiteX = [](size_t wIdx) noexcept -> float {
        return kWhiteBaseX + static_cast<float>(wIdx) * kWhitePitchX;
    };
    const auto shiftBlackX = [&](size_t bIdx) noexcept -> float {
        return kWhiteBaseX + (static_cast<float>(kBlackPianoGap[bIdx]) + 0.5f) * kWhitePitchX;
    };

    if (cv.y1 > 70 && cv.y0 < 186) {
        if (cv.y1 > 90 && cv.y0 < 153) {
            fillKeybedStadiumSDF(cv, 16.0f, 92.0f, 224.0f, 151.0f, 11.5f, 9u, 46u, 246u);
        }

        // Top-row `KEYBOARD` (`OP1-OP3`) & `SHIFT` (`11th` key) callouts
        const float kbL = shiftBlackX(0u) - 3.2f;
        const float kbR = shiftBlackX(2u) + 3.2f;
        const int kbMid = static_cast<int>(0.5f * (kbL + kbR) + 0.5f);
        cv.drawSansCentered(kFontSansSmall, kbMid, 74, "KEYBOARD", modelHot ? 255u : 236u, 0);
        const float kbBrkLum = modelHot ? 235.0f : 148.0f;
        cv.drawHorizontalFilament(kbL, kbR, 89.2f, kbBrkLum, 0.58f, true);
        cv.drawVerticalSlit(kbL, 89.2f, 94.2f, kbBrkLum);
        cv.drawVerticalSlit(kbR, 89.2f, 94.2f, kbBrkLum);

        const float shiftKeyX = shiftBlackX(kShiftBlackKeyIdx);
        cv.drawSansCentered(kFontSansSmall, static_cast<int>(shiftKeyX + 0.5f), 74, "SHIFT", 242u,
                            0);
        cv.drawVerticalSlit(shiftKeyX, 86.5f, 94.2f, 214.0f);

        // Top 11 raised keys (`3 · 2 · 3 · 2 · 1`): `0..2` = `KEYBOARD` switches, `10` = `SHIFT`
        constexpr float kBTop = 96.5f;
        constexpr float kBBot = 119.0f;
        constexpr float kBMidY = 0.5f * (kBTop + kBBot);
        const float blink01 = static_cast<float>(hud.shiftBlinkQ8) * (1.0f / 255.0f);
        if (cv.y1 > static_cast<int>(kBTop) - 3 && cv.y0 <= static_cast<int>(kBBot) + 3) {
            for (size_t i = 0u; i < kNumBlackKeys; ++i) {
                const float cx = shiftBlackX(i);
                const float wPulse =
                    (wipe > 0.002f)
                        ? (clampf(1.0f -
                                      absf(wipeProg * 1.25f - static_cast<float>(i) * 0.10f) * 4.0f,
                                  0.0f, 1.0f) *
                           wipe)
                        : 0.0f;
                if (i < 3u) {
                    const bool selModel = (i == static_cast<size_t>(hud.keyboardModel));
                    const bool held = ((hud.heldBlackMask >> i) & 1u) != 0u;
                    if (held || selModel) {
                        const float p = held ? 1.0f : clampf(blink01 + wPulse, 0.0f, 1.0f);
                        const auto fillG = static_cast<uint8_t>(11.0f + 29.0f * p + 0.5f);
                        const auto rimG = static_cast<uint8_t>(48.0f + 207.0f * p + 0.5f);
                        fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kBTop, cx + kCapsuleHalfW,
                                             kBBot, kCapsuleHalfW, fillG, rimG, 248u);
                        fillKeybedSwitchDotSDF(cv, cx, kBMidY, 0.04f + 0.96f * p, true);
                        if (p > 0.22f) {
                            cv.drawOpticalBead(cx, kBMidY, (p - 0.22f) * 0.90f, 2.55f);
                        }
                    } else {
                        const float g = clampf(0.15f + wPulse, 0.0f, 1.0f);
                        const auto rimG = static_cast<uint8_t>(120.0f + 120.0f * wPulse + 0.5f);
                        fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kBTop, cx + kCapsuleHalfW,
                                             kBBot, kCapsuleHalfW, 16u, rimG, 248u);
                        fillKeybedSwitchDotSDF(cv, cx, kBMidY, g, true);
                    }
                } else if (i == kShiftBlackKeyIdx) {
                    fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kBTop, cx + kCapsuleHalfW, kBBot,
                                         kCapsuleHalfW, 34u, 252u, 248u);
                    fillKeybedSwitchDotSDF(cv, cx, kBMidY, 1.0f, true);
                } else {
                    const auto wRim = static_cast<uint8_t>(30.0f + 120.0f * wPulse + 0.5f);
                    fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kBTop, cx + kCapsuleHalfW, kBBot,
                                         kCapsuleHalfW, 13u, wRim, 246u);
                    if (isAnalogPressureBlackKey(i)) {
                        fillKeybedSlitSDF(cv, cx, kBTop + 3.6f, kBBot - 3.6f, 0.95f, wPulse, -1.0f,
                                          wPulse > 0.03f);
                    } else {
                        fillKeybedSwitchDotSDF(cv, cx, kBMidY, wPulse, wPulse > 0.03f);
                    }
                }
                if (coProcYield != nullptr && (i & 3u) == 3u) {
                    coProcYield();
                }
            }
        }

        // 4 piano natural-gap registration dots (`B-C` / `E-F`) along the row midline
        if (cv.y1 > 118 && cv.y0 < 126) {
            for (uint8_t gap : kNaturalDotGap) {
                const float dotX = kWhiteBaseX + (static_cast<float>(gap) + 0.5f) * kWhitePitchX;
                cv.drawOpticalBead(dotX, 121.5f, 0.34f, 0.90f);
            }
        }
        if (coProcYield != nullptr) {
            coProcYield();
        }

        // Bottom 16 evenly spaced keys: `0..7` ghosted, `8..11` = `SCENE` pads, `12..15` = `SOLO`
        constexpr float kWTop = 124.0f;
        constexpr float kWBot = 146.5f;
        constexpr float kWMidY = 0.5f * (kWTop + kWBot);
        const bool snapLiveMorph = isSceneLiveMorphActive(hud);
        if (cv.y1 > static_cast<int>(kWTop) - 3 && cv.y0 <= static_cast<int>(kWBot) + 3) {
            for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
                const float cx = shiftWhiteX(i);
                float pos01 = static_cast<float>(i) * (1.0f / 15.0f);
                if (hud.keyboardModel == KeyboardModel::CircuitLab) {
                    pos01 = static_cast<float>((i * 7u + 3u) & 15u) * (1.0f / 15.0f);
                } else if (hud.keyboardModel == KeyboardModel::RhythmLab) {
                    pos01 = static_cast<float>(((i & 3u) * 4u) + (i >> 2u)) * (1.0f / 15.0f);
                }
                const float wPulse =
                    (wipe > 0.002f)
                        ? (clampf(1.0f - absf(wipeProg * 1.25f - pos01) * 4.2f, 0.0f, 1.0f) * wipe)
                        : 0.0f;

                if (i < kSceneWhiteKeyBase) {
                    const auto wRim = static_cast<uint8_t>(30.0f + 120.0f * wPulse + 0.5f);
                    fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kWTop, cx + kCapsuleHalfW, kWBot,
                                         kCapsuleHalfW, 13u, wRim, 246u);
                    fillKeybedSlitSDF(cv, cx, kWTop + 3.6f, kWBot - 3.6f, 1.0f, wPulse, -1.0f,
                                      wPulse > 0.03f);
                } else if (i < kMuteWhiteKeyBase) {
                    const uint8_t slot = static_cast<uint8_t>(i - kSceneWhiteKeyBase);
                    const bool occ = (hud.snapshotOccupiedMask & (1u << slot)) != 0u;
                    const bool held = ((hud.heldWhiteMask >> i) & 1u) != 0u;
                    const float g = clampf(
                        computeSceneKeyBrightness(slot, hud, snapLiveMorph) + wPulse, 0.0f, 1.0f);
                    const float pFill =
                        (held && occ)
                            ? (static_cast<float>(hud.whitePressureQ8[i]) * (1.0f / 255.0f))
                            : -1.0f;
                    const auto rim = static_cast<uint8_t>(held ? 252u : (occ ? 168u : 132u));
                    fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kWTop, cx + kCapsuleHalfW, kWBot,
                                         kCapsuleHalfW, held ? 32u : 19u, rim, 248u);
                    fillKeybedSlitSDF(cv, cx, kWTop + 3.6f, kWBot - 3.6f, 1.08f, g, pFill, true);
                } else {
                    const uint8_t v = static_cast<uint8_t>(i - kMuteWhiteKeyBase);
                    const bool held = ((hud.heldWhiteMask >> i) & 1u) != 0u;
                    const float g =
                        clampf(evaluateVoiceSoundLedSteady(control, v) + wPulse, 0.0f, 1.0f);
                    fillKeybedStadiumSDF(cv, cx - kCapsuleHalfW, kWTop, cx + kCapsuleHalfW, kWBot,
                                         kCapsuleHalfW, held ? 32u : 19u,
                                         static_cast<uint8_t>(held ? 252u : 148u), 248u);
                    fillKeybedSwitchDotSDF(cv, cx, kWMidY, g, true);
                }
                if (coProcYield != nullptr && (i & 3u) == 3u) {
                    coProcYield();
                }
            }
        }

        // Upward callout brackets below `SCENE` (`Keys 9-12`) & `SOLO` (`Keys 13-16`)
        const auto drawShiftZoneBracket = [&](float xL, float xR, const char* lbl,
                                              bool hot) noexcept {
            const float brkLum = hot ? 235.0f : 148.0f;
            cv.drawHorizontalFilament(xL, xR, 165.0f, brkLum, 0.58f, true);
            cv.drawVerticalSlit(xL, 162.5f, 165.0f, brkLum);
            cv.drawVerticalSlit(xR, 162.5f, 165.0f, brkLum);
            cv.drawSansCentered(kFontSansSmall, static_cast<int>(0.5f * (xL + xR) + 0.5f), 169, lbl,
                                hot ? 255u : 242u, 0);
        };
        drawShiftZoneBracket(shiftWhiteX(kSceneWhiteKeyBase) - 3.2f,
                             shiftWhiteX(kSceneWhiteKeyBase + 3u) + 3.2f, "SCENE", sceneHot);
        drawShiftZoneBracket(shiftWhiteX(kMuteWhiteKeyBase) - 3.2f,
                             shiftWhiteX(kMuteWhiteKeyBase + 3u) + 3.2f, "SOLO", soloHot);
    }

    // 4. Full-Width Horizontal Pressure Bar + Bottom Guidance / Live Effect Description
    if (cv.y1 > 192 && cv.y0 < 226) {
        float press01 = 0.0f;
        const bool showPressBar = evalKeybedBottomPressureBar(hud, press01);
        if (showPressBar) {
            drawKeybedBottomPressureBar(cv, press01);
        }
        const char* desc = tempoHot   ? "MASTER CLOCK SPEED (20-3000 BPM)"
                           : modelHot ? "SWAP ACTIVE KEYBOARD MODEL (1-3)"
                           : sceneHot ? keyPunchGuidanceDesc(KeyPunchKind::SnapshotRecall)
                           : soloHot  ? keyPunchGuidanceDesc(KeyPunchKind::SpotlightSolo)
                                      : "USE LIT CONTROLS WHILE HOLDING SHIFT";
        const uint8_t descLum = (tempoHot || modelHot || sceneHot || soloHot) ? 195u : 138u;
        cv.drawSansCentered(kFontSansSmall, 120, showPressBar ? 208 : 206, desc, descLum, 0);
    }
}

} // namespace ruido
