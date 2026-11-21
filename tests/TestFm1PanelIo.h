#pragma once

#include "../platforms/fm1/fm1_ops_screen.h"
#include "../platforms/fm1/fm1_panel_io.h"
#include "../platforms/fm1/fm1_storage_io.h"
#include "core/RuidoController.h"
#include "ui/HudFormat.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ruido_test {

template <typename CheckFn> inline void runFm1PanelIoAndLedModulatorTests(CheckFn checkFn) {
    std::printf("[Test 14] FM-1 Gate 4.3 Panel Matrix, SARADC Pressure & 42-LED Modulator...\n");
#define RUIDO_FM1_CHECK(expr) checkFn((expr), #expr, __LINE__)

    using namespace ruido;
    using namespace ruido::fm1;

    // 1. Verify all 42 matrix LED nodes (41 key/button IDs + PLAY green at {8, 1}) are
    // collision-free
    //    and all 27 silicone keys map bijectively onto 16 white + 11 black keys (20 analog + 7
    //    PA6).
    bool occupied[kFm1NumCols][5]{};
    for (size_t id = 0u; id < kFm1KeyIdCoord.size(); ++id) {
        const MatrixCoord mc = kFm1KeyIdCoord[id];
        RUIDO_FM1_CHECK(mc.col < kFm1NumCols);
        RUIDO_FM1_CHECK(mc.row >= 1u && mc.row <= 4u);
        RUIDO_FM1_CHECK(!occupied[mc.col][mc.row]);
        occupied[mc.col][mc.row] = true;
    }
    RUIDO_FM1_CHECK(!occupied[kPlayGreenCol][kPlayGreenRow]);
    occupied[kPlayGreenCol][kPlayGreenRow] = true;
    RUIDO_FM1_CHECK(kFm1KeyIdCoord[static_cast<size_t>(ButtonId::PlayStop)].col == kStopRedCol);
    RUIDO_FM1_CHECK(kFm1KeyIdCoord[static_cast<size_t>(ButtonId::PlayStop)].row == kStopRedRow);

    uint32_t whiteSeenMask = 0u;
    uint32_t blackSeenMask = 0u;
    uint32_t analogCount = 0u;
    uint32_t digitalOnlyCount = 0u;
    for (uint8_t n = 0u; n < kFm1NumNoteKeys; ++n) {
        const NoteKeyMapping& km = kNoteKeyTable[n];
        if (km.isBlack) {
            RUIDO_FM1_CHECK(km.subIndex < kNumBlackKeys);
            blackSeenMask |= (1u << km.subIndex);
        } else {
            RUIDO_FM1_CHECK(km.subIndex < kNumWhiteKeys);
            whiteSeenMask |= (1u << km.subIndex);
        }
        if (km.saradcCh >= 0) {
            RUIDO_FM1_CHECK(km.saradcCh == 0 || km.saradcCh == 1);
            RUIDO_FM1_CHECK(km.paPinBit == (km.saradcCh == 0 ? 7u : 8u));
            ++analogCount;
        } else {
            RUIDO_FM1_CHECK(km.paPinBit == 6u);
            ++digitalOnlyCount;
        }
    }
    RUIDO_FM1_CHECK(whiteSeenMask == ((1u << kNumWhiteKeys) - 1u));
    RUIDO_FM1_CHECK(blackSeenMask == ((1u << kNumBlackKeys) - 1u));
    RUIDO_FM1_CHECK(analogCount == 20u);
    RUIDO_FM1_CHECK(digitalOnlyCount == 7u);

    // 2. Verify quiet boot MASTER pot sync, 7 hardware encoders (`steps * 4`), & intra-poll fast
    // tap
    RuidoController ctrl;
    Fm1PanelState panel;
    panel.reset(0.50f);
    panel.syncMasterPot(ctrl, 0.50f);
    RUIDO_FM1_CHECK(std::fabs(ctrl.state().global.masterTrim - 0.50f) < 1.0e-4f);
    RUIDO_FM1_CHECK(ctrl.hud().mode == HudMode::None);
    RUIDO_FM1_CHECK(ctrl.hud().opacity == 0.0f);

    // Thermal 1-LSB noise stays silent; deliberate pot turn triggers MasterLevel HUD toast
    panel.syncMasterPot(ctrl, 0.501f);
    RUIDO_FM1_CHECK(ctrl.hud().mode == HudMode::None);
    panel.syncMasterPot(ctrl, 0.78f);
    RUIDO_FM1_CHECK(std::fabs(ctrl.state().global.masterTrim - 0.78f) < 1.0e-4f);
    RUIDO_FM1_CHECK(ctrl.hud().mode == HudMode::MasterLevel);

    const float baseOsc1Hz = ctrl.state().voices[0].frequencyHz;
    int32_t encSteps[kFm1NumEncoders]{0, 0, 3, 0,
                                      0, 0, 12}; // KNOB1 +3 detents, PRESETS +12 detents
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, 0u, 0u, 0u, encSteps));
    RUIDO_FM1_CHECK(ctrl.snapshots().activeSlot() == 1u);

    int32_t knobOnly[kFm1NumEncoders]{0, 0, 4, 0, 0, 0, 0};
    const float slot1Osc1Hz = ctrl.state().voices[0].frequencyHz;
    (void)baseOsc1Hz;
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, 0u, 0u, 0u, knobOnly));
    RUIDO_FM1_CHECK(ctrl.state().voices[0].frequencyHz > slot1Osc1Hz);

    // Intra-poll fast press+release on PLAY/STOP (`bit 12`) toggles masterRun `false -> true`
    int32_t zeroEnc[kFm1NumEncoders]{};
    const uint32_t playBit = 1u << static_cast<uint32_t>(ButtonId::PlayStop);
    RUIDO_FM1_CHECK(!ctrl.state().global.masterRun);
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, playBit, playBit, 0u, zeroEnc));
    RUIDO_FM1_CHECK(ctrl.state().global.masterRun);

    // 3. Verify PA7/PA8 DRIVE-2 (`R_s = 132 ohm`) wide `[528..815]` carbon-pill curve & diagnostics
    RUIDO_FM1_CHECK(adcToCarbonPressure01(-2) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(0) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(119) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(1023) < 0.0f);
    // High closed-key readings (`815..1022`) must stay at 4% feather touch, NEVER `-1 -> 100%`!
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(1015) - 0.04f) < 1.0e-4f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(815) - 0.04f) < 1.0e-4f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(670) > 0.18f && adcToCarbonPressure01(670) < 0.28f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(555) > 0.74f && adcToCarbonPressure01(555) < 0.84f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(528) - 1.0f) < 1.0e-4f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(400) - 1.0f) < 1.0e-4f);

    // Key Pressure presets: explicit MEDIUM == default; lighter presets read strictly higher at the
    // same mid squeeze, firmer presets strictly lower; VERY LIGHT maxes out by ADC 600 while VERY
    // FIRM still reaches 100% above the ~490 PCB series-resistance floor.
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(670, PressureTouch::Medium) -
                              adcToCarbonPressure01(670)) < 1.0e-6f);
    for (uint8_t lvl = 1u; lvl < kNumPressureTouchLevels; ++lvl) {
        const auto lighter = static_cast<PressureTouch>(lvl - 1u);
        const auto firmer = static_cast<PressureTouch>(lvl);
        RUIDO_FM1_CHECK(adcToCarbonPressure01(640, lighter) > adcToCarbonPressure01(640, firmer));
        RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(815, firmer) - 0.04f) < 1.0e-4f);
    }
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(690, PressureTouch::VeryLight) - 1.0f) <
                    1.0e-4f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(600, PressureTouch::VeryFirm) < 0.35f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(515, PressureTouch::VeryFirm) - 1.0f) <
                    1.0e-4f);
    // Presets must be clearly distinct at a light-medium squeeze (ADC 640): >= 8 points apart.
    for (uint8_t lvl = 1u; lvl < kNumPressureTouchLevels; ++lvl) {
        RUIDO_FM1_CHECK(adcToCarbonPressure01(640, static_cast<PressureTouch>(lvl - 1u)) >=
                        adcToCarbonPressure01(640, static_cast<PressureTouch>(lvl)) + 0.08f);
    }
    for (uint8_t lvl = 0u; lvl < kNumPressureTouchLevels; ++lvl) {
        float prev = 0.04f;
        for (int32_t adc = 815; adc >= 480; adc -= 5) {
            const float p = adcToCarbonPressure01(adc, static_cast<PressureTouch>(lvl));
            RUIDO_FM1_CHECK(p >= prev && p <= 1.0f);
            prev = p;
        }
    }

    float prevP = 0.04f;
    for (int32_t adc = 815; adc >= 480; adc -= 20) {
        const float p = adcToCarbonPressure01(adc);
        RUIDO_FM1_CHECK(p >= prevP);
        prevP = p;
    }

    ctrl.reset();
    panel.reset(0.85f);
    panel.syncMasterPot(ctrl, 0.85f);

    // Digital fallback (`rawAdc == -1`): holding n0 (White Key 1 = OSC 1 filter pad) immediately
    // applies 100% pressure and schedules PA8 (`ch 1`, `col 3`).
    const uint32_t n0Mask = 1u << 0u;
    const uint32_t n9Mask = 1u << 9u;
    RUIDO_FM1_CHECK(panel.pollKeysAndPressure(ctrl, n0Mask, n0Mask));
    RUIDO_FM1_CHECK(ctrl.state().voicePressure[0] > 0.99f);
    const auto req1 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req1.active == 1u && req1.noteBit == 0u && req1.col == 3u &&
                    req1.saradcCh == 1u);

    // Hold both n0 (PA8 ch1 col 3) and n9 (PA7 ch0 col 0): round-robin alternates 0 <-> 9
    (void)panel.pollKeysAndPressure(ctrl, n0Mask | n9Mask, n9Mask);
    const auto req2 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req2.active == 1u && req2.noteBit == 9u && req2.col == 0u &&
                    req2.saradcCh == 0u);
    (void)panel.pollKeysAndPressure(ctrl, n0Mask | n9Mask, 0u);
    const auto req3 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req3.active == 1u && req3.noteBit == 0u);

    // Feed physical `DRIVE 2` (`R_s = 132 ohm`) readings on n0: soft (670) -> firm (520)
    (void)panel.pollKeysAndPressure(ctrl, 0u, 0u);
    panel.recordAdcSample(0u, 670);
    (void)panel.pollKeysAndPressure(ctrl, n0Mask, n0Mask);
    const float softPress = ctrl.state().voicePressure[0];
    RUIDO_FM1_CHECK(softPress >= 0.18f && softPress < 0.28f);

    panel.recordAdcSample(0u, 520);
    for (int step = 0; step < 12; ++step) {
        (void)panel.pollKeysAndPressure(ctrl, n0Mask, 0u);
    }
    RUIDO_FM1_CHECK(ctrl.state().voicePressure[0] > 0.99f);

    // 4. Verify 42-channel multi-tier 909.1 Hz Delta-Sigma LED modulator (`PLAY` green vs `STOP`
    // red)
    ctrl.reset();
    const EngineTelemetry zeroTel{};
    ctrl.tick(0.016f, zeroTel);

    Fm1LedModulator ledMod;
    ledMod.reset();
    ledMod.updateTargets(ctrl.leds());

    uint8_t lit[kFm1NumCols]{};
    uint8_t dim[kFm1NumCols]{};
    uint8_t bg[kFm1NumCols]{};
    ledMod.stepFrame909Hz(lit, dim, bg);

    // Cold-boot stopped state (`masterRun == false`): PLAY green (col 8, row 1) lights, STOP red &
    // MUTE 1-4 stay 0% dark
    RUIDO_FM1_CHECK((lit[kPlayGreenCol] & (1u << kPlayGreenRow)) != 0u);
    RUIDO_FM1_CHECK((lit[kStopRedCol] & (1u << kStopRedRow)) == 0u);
    RUIDO_FM1_CHECK(((lit[1u] | dim[1u] | bg[1u]) & (1u << 2u)) == 0u);

    // Start audio via PLAY/STOP (`masterRun == true`): STOP red (col 10, row 2) lights, PLAY green
    // (col 8, row 1) goes dark
    ctrl.onButtonEvent(ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ButtonId::PlayStop, false);
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.025f, zeroTel);
    }
    ledMod.updateTargets(ctrl.leds());
    ledMod.stepFrame909Hz(lit, dim, bg);
    RUIDO_FM1_CHECK((lit[kStopRedCol] & (1u << kStopRedRow)) != 0u);
    RUIDO_FM1_CHECK((lit[kPlayGreenCol] & (1u << kPlayGreenRow)) == 0u);
    RUIDO_FM1_CHECK((bg[kPlayGreenCol] & (1u << kPlayGreenRow)) == 0u);
    // Rest panel buttons (SEL = 15% perceptual -> `fm1_led_bg`; active boot FX -> `bg|dim`)
    const MatrixCoord selMc = kFm1KeyIdCoord[static_cast<size_t>(ButtonId::Sel)];
    RUIDO_FM1_CHECK((bg[selMc.col] & (1u << selMc.row)) != 0u);
    RUIDO_FM1_CHECK(((bg[6u] | dim[6u]) & (1u << 1u)) != 0u);
    // Left-bright -> right-dark gradient: FILTER 1 (n0 at col 3, row 4 = 65% -> `dim|lit`),
    // OCTAVE 1 (n7 at col 9, row 4 = 40% -> `dim|lit`), DUTY 1 (n14 at col 5, row 3 = 15% -> `bg`),
    // DUTY 2 (n16 at col 7, row 3 = 15% -> `bg`), and audible MUTE 1 (n21 at col 1, row 2 = 100%
    // -> `lit`):
    RUIDO_FM1_CHECK(((dim[3u] | lit[3u]) & (1u << 4u)) != 0u);
    RUIDO_FM1_CHECK(((dim[9u] | lit[9u]) & (1u << 4u)) != 0u);
    RUIDO_FM1_CHECK((bg[5u] & (1u << 3u)) != 0u);
    RUIDO_FM1_CHECK((bg[7u] & (1u << 3u)) != 0u);
    RUIDO_FM1_CHECK((lit[1u] & (1u << 2u)) != 0u);

    // Verify mid-scale Delta-Sigma duty accuracy AND spatial phase staggering across columns:
    LedState customLeds{};
    customLeds.whiteKeyLeds[0] = 0.65f; // col 3, row 4 (n0)
    customLeds.whiteKeyLeds[1] = 0.65f; // col 5, row 4 (n2)
    ledMod.reset();
    ledMod.updateTargets(customLeds);

    const float expectedDuty = perceptualToPhysicalLedDuty(0.65f);
    double dutySumCol3 = 0.0;
    uint32_t diffPhaseFrames = 0u;
    for (uint32_t f = 0u; f < 256u; ++f) {
        ledMod.stepFrame909Hz(lit, dim, bg);
        const bool lit3 = (lit[3u] & (1u << 4u)) != 0u;
        const bool lit5 = (lit[5u] & (1u << 4u)) != 0u;
        dutySumCol3 += lit3 ? 1.0 : static_cast<double>(kLedTier2Duty);
        if (lit3 != lit5) {
            ++diffPhaseFrames;
        }
    }
    const double measuredDuty = dutySumCol3 / 256.0;
    RUIDO_FM1_CHECK(std::fabs(measuredDuty - static_cast<double>(expectedDuty)) < 0.01);
    RUIDO_FM1_CHECK(diffPhaseFrames > 32u); // Staggered VDDIO switching across columns!

    // 5. Verify all 12 FM-1 Hardware Operational Screens (monochrome neutrality, strip invariance,
    //    10 px left/right bezel margins, big-endian SPI1 swap, and 1 px bar accuracy):
    static std::array<uint16_t, 240u * 240u> opsFullLe{};
    static std::array<uint16_t, 240u * 240u> opsStrips16Le{};
    static std::array<uint16_t, 240u * 16u> opsStripBe{};
    for (uint8_t k = 0u; k <= static_cast<uint8_t>(RUIDO_OPS_UBOOT); ++k) {
        RuidoOpsState st{};
        st.kind = k;
        st.percent = 57u;
        st.bar_px = (k == RUIDO_OPS_LDR_WAITING) ? 84u : 125u;
        st.fault_vec = 0x00000004u;
        st.fault_pc = 0x0201A4C8u;
        st.fault_rets = 0x0201A310u;

        renderOpsScreenStripImpl(st, 0u, 240u, opsFullLe.data(), false);
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 16u)) {
            renderOpsScreenStripImpl(st, y, 16u, opsStrips16Le.data() + y * 240u, false);
        }
        renderOpsScreenStripImpl(st, 112u, 16u, opsStripBe.data(), true);

        bool stripsEq = true;
        bool monoOk = true;
        bool marginsClean = true;
        uint32_t litPixels = 0u;
        for (size_t y = 0u; y < 240u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                const uint16_t px = opsFullLe[y * 240u + x];
                if (px != opsStrips16Le[y * 240u + x]) {
                    stripsEq = false;
                }
                const uint16_t r5 = (px >> 11u) & 0x1Fu;
                const uint16_t g6 = (px >> 5u) & 0x3Fu;
                const uint16_t b5 = px & 0x1Fu;
                if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
                    monoOk = false;
                }
                if ((x < 9u || x > 231u) && px != 0u) {
                    marginsClean = false;
                }
                if (px != 0u) {
                    ++litPixels;
                }
            }
        }
        bool beSwapOk = true;
        for (size_t i = 0u; i < 240u * 16u; ++i) {
            const uint16_t le = opsFullLe[112u * 240u + i];
            const auto expectedBe = static_cast<uint16_t>((le >> 8u) | (le << 8u));
            if (opsStripBe[i] != expectedBe) {
                beSwapOk = false;
            }
        }
        RUIDO_FM1_CHECK(stripsEq);
        RUIDO_FM1_CHECK(monoOk);
        RUIDO_FM1_CHECK(marginsClean);
        RUIDO_FM1_CHECK(beSwapOk);
        RUIDO_FM1_CHECK(litPixels > 350u);
    }

    // Verify 1 px progress bar accuracy at y = 112..115 across 0%, 50% (110 px), and 100% (220 px)
    for (uint16_t fill : {uint16_t{0u}, uint16_t{110u}, uint16_t{220u}}) {
        RuidoOpsState wrSt{RUIDO_OPS_LDR_WRITING,
                           static_cast<uint8_t>((fill * 100u) / 220u),
                           fill,
                           0u,
                           0u,
                           0u,
                           0u,
                           0u,
                           0u};
        renderOpsScreenStripImpl(wrSt, 112u, 16u, opsStripBe.data(), false);
        uint32_t whiteCols = 0u;
        uint32_t trackCols = 0u;
        for (size_t x = 10u; x < 230u; ++x) {
            const uint16_t px = opsStripBe[1u * 240u + x]; // y = 113
            if (px == 0xFFFFu) {
                ++whiteCols;
            } else if (px != 0u) {
                ++trackCols;
            }
        }
        RUIDO_FM1_CHECK(whiteCols == fill);
        RUIDO_FM1_CHECK(whiteCols + trackCols == 220u);
    }

    // Verify standalone RAM `loader.bin` renderer (`renderOpsScreenStripLdrImpl` + sparse 2-bit
    // SoA font tables) matches full `FontSansAA` on every solid stroke, background, and progress
    // bar pixel, with bounded 2-bit vs 4-bit edge AA delta (`<= 4` in 5-bit RGB565 channel)
    for (uint8_t ldrKind :
         {static_cast<uint8_t>(RUIDO_OPS_LDR_WAITING), static_cast<uint8_t>(RUIDO_OPS_LDR_WRITING),
          static_cast<uint8_t>(RUIDO_OPS_LDR_FAILED)}) {
        for (uint8_t pct : {uint8_t{0u}, uint8_t{64u}, uint8_t{100u}}) {
            RuidoOpsState ldrSt{ldrKind, pct, 110u, 0u, 0u, 0u, 0u, 0u, 0u};
            renderOpsScreenStripFullImpl(ldrSt, 0u, 240u, opsFullLe.data(), false);
            renderOpsScreenStripLdrImpl(ldrSt, 0u, 240u, opsStrips16Le.data(), false);
            int maxChanDiff = 0;
            for (size_t i = 0u; i < opsFullLe.size(); ++i) {
                const uint16_t a = opsFullLe[i];
                const uint16_t b = opsStrips16Le[i];
                const size_t y = i / 240u;
                if (a == 0xFFFFu || (y >= 112u && y < 116u)) {
                    RUIDO_FM1_CHECK(a == b);
                }
                const int ga = static_cast<int>(a & 0x1Fu);
                const int gb = static_cast<int>(b & 0x1Fu);
                const int d = (ga > gb) ? (ga - gb) : (gb - ga);
                if (d > maxChanDiff) {
                    maxChanDiff = d;
                }
            }
            RUIDO_FM1_CHECK(maxChanDiff <= 4);
        }
    }

    // 7. Verify erase-free 128 B Scene / LIVE journals & 16 B Settings journal + cold compaction
    {
        struct MockNvsFlash {
            std::array<PatchSnapshot, kNumSnapshots> slots{};
            std::array<bool, kNumSnapshots> slotValid{};
            PatchSnapshot baseLive{};
            bool baseLiveValid = false;
            DeviceSettingsBlob baseSettings{};
            bool baseSettingsValid = false;
            std::array<std::array<uint8_t, 0x1000u>, 6u> sectors{};
            uint32_t eraseCount = 0u;
            uint32_t baseSaveCount = 0u;
            uint32_t journalWriteCount = 0u;

            void initBlank() noexcept {
                slotValid.fill(false);
                baseLiveValid = false;
                baseSettingsValid = false;
                for (auto& sec : sectors) {
                    sec.fill(0xFFu);
                }
                eraseCount = 0u;
                baseSaveCount = 0u;
                journalWriteCount = 0u;
            }
        };
        static MockNvsFlash gMockNvs{};
        gMockNvs.initBlank();

        const RuidoFm1BlobLoadFn loadFn = [](uint32_t idx, void* dst, uint32_t len) -> int {
            if (idx < kNumSnapshots) {
                if (!gMockNvs.slotValid[idx] || len != sizeof(PatchSnapshot)) {
                    return -1;
                }
                std::memcpy(dst, &gMockNvs.slots[idx], len);
                return 0;
            }
            if (idx == kNvsSettingsBlobIdx) {
                if (!gMockNvs.baseSettingsValid || len != sizeof(DeviceSettingsBlob)) {
                    return -1;
                }
                std::memcpy(dst, &gMockNvs.baseSettings, len);
                return 0;
            }
            if (idx == kNvsLivePatchIdx) {
                if (!gMockNvs.baseLiveValid || len != sizeof(PatchSnapshot)) {
                    return -1;
                }
                std::memcpy(dst, &gMockNvs.baseLive, len);
                return 0;
            }
            return -1;
        };
        const RuidoFm1BlobSaveFn saveFn = [](uint32_t idx, const void* src, uint32_t len) -> int {
            if (idx >= gMockNvs.sectors.size()) {
                return -1;
            }
            if (idx < kNumSnapshots && len == sizeof(PatchSnapshot)) {
                std::memcpy(&gMockNvs.slots[idx], src, len);
                gMockNvs.slotValid[idx] = true;
                gMockNvs.sectors[idx].fill(0xFFu);
                std::memcpy(gMockNvs.sectors[idx].data() + 12u, src, len);
                ++gMockNvs.baseSaveCount;
                return 0;
            }
            if (idx == kNvsSettingsBlobIdx && len == sizeof(DeviceSettingsBlob)) {
                std::memcpy(&gMockNvs.baseSettings, src, len);
                gMockNvs.baseSettingsValid = true;
                gMockNvs.sectors[idx].fill(0xFFu);
                std::memcpy(gMockNvs.sectors[idx].data() + 12u, src, len);
                ++gMockNvs.baseSaveCount;
                return 0;
            }
            if (idx == kNvsLivePatchIdx && len == sizeof(PatchSnapshot)) {
                std::memcpy(&gMockNvs.baseLive, src, len);
                gMockNvs.baseLiveValid = true;
                gMockNvs.sectors[idx].fill(0xFFu);
                std::memcpy(gMockNvs.sectors[idx].data() + 12u, src, len);
                ++gMockNvs.baseSaveCount;
                return 0;
            }
            return -1;
        };
        const RuidoFm1PreEraseFn preEraseFn = []() { ++gMockNvs.eraseCount; };
        const RuidoFm1TailReadFn tailReadFn = [](uint32_t sec, uint32_t off, void* dst,
                                                 uint32_t len) -> int {
            const uint32_t minOff =
                (sec == kNvsSettingsBlobIdx) ? kSettingsJournalBaseOff : kPatchJournalBaseOff;
            if (sec >= gMockNvs.sectors.size() || off < minOff ||
                off + len > gMockNvs.sectors[sec].size()) {
                return -1;
            }
            std::memcpy(dst, gMockNvs.sectors[sec].data() + off, len);
            return 0;
        };
        const RuidoFm1TailWriteFn tailWriteFn = [](uint32_t sec, uint32_t off, const void* src,
                                                   uint32_t len) -> int {
            const uint32_t minOff =
                (sec == kNvsSettingsBlobIdx) ? kSettingsJournalBaseOff : kPatchJournalBaseOff;
            const uint32_t reqLen =
                (sec == kNvsSettingsBlobIdx) ? kSettingsJournalEntryBytes : kPatchJournalEntryBytes;
            if (sec >= gMockNvs.sectors.size() || off < minOff || len != reqLen ||
                (off & (reqLen - 1u)) != 0u || off + len > gMockNvs.sectors[sec].size()) {
                return -1;
            }
            const auto* inBytes = static_cast<const uint8_t*>(src);
            for (uint32_t i = 0u; i < len; ++i) {
                // Enforce real SPI NOR flash physics: Page Program can only clear bits (1 -> 0)!
                if ((gMockNvs.sectors[sec][off + i] & inBytes[i]) != inBytes[i]) {
                    return -4;
                }
                gMockNvs.sectors[sec][off + i] =
                    static_cast<uint8_t>(gMockNvs.sectors[sec][off + i] & inBytes[i]);
            }
            ++gMockNvs.journalWriteCount;
            return 0;
        };

        RuidoController nvsCtrl{};
        Fm1PanelState nvsPanel{};
        Fm1LedModulator nvsLeds{};
        Fm1NvsRuntimeState nvsRt{};
        const EngineTelemetry dummyTel{};
        RUIDO_FM1_CHECK(loadBootNvsBlobs(loadFn, nvsCtrl, nvsPanel, nvsLeds, 0.6f, nvsRt,
                                         tailReadFn, saveFn) == 0u);
        RUIDO_FM1_CHECK(nvsRt.nextJournalSlot == 0u);
        RUIDO_FM1_CHECK(nvsRt.nextLiveSlot == 0u);
        RUIDO_FM1_CHECK(nvsRt.nextPatchSlot[1] == 0u);

        // Live Mixer pressure-key bounce `Scene 1 -> Scene 2 -> Scene 1` before 1.5 s deduplicates
        // to ZERO flash operations (neither erase nor journal write)!
        nvsCtrl.onWhiteKeyGate(9u, true); // Scene 2
        nvsCtrl.onWhiteKeyGate(9u, false);
        nvsCtrl.onWhiteKeyGate(8u, true); // Back to Scene 1
        nvsCtrl.onWhiteKeyGate(8u, false);
        for (int i = 0; i < 40; ++i) {
            nvsCtrl.tick(0.02f, dummyTel);
        }
        nvsRt.lastPanelTouchMs = 100u;
        RUIDO_FM1_CHECK(
            pollAndSaveNvsBlobs(1700u, 0u, saveFn, preEraseFn, nvsCtrl, nvsRt, tailWriteFn) == 0u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u && gMockNvs.journalWriteCount == 0u);

        // Live `SAVE + Key 10` (`Scene 2`) writes 128 B Scene 2 journal + 16 B Settings journal
        // with ZERO `preEraseFn` / sector erases!
        nvsCtrl.onEncoderDelta(EncoderId::Knob1, 24);
        const float savedScene2Hz = nvsCtrl.state().voices[0].frequencyHz;
        nvsCtrl.onButtonEvent(ButtonId::Save, true);
        nvsCtrl.onButtonEvent(ButtonId::Save, false);
        nvsCtrl.onWhiteKeyGate(9u, true); // Store Scene 2
        nvsCtrl.onWhiteKeyGate(9u, false);
        RUIDO_FM1_CHECK(
            pollAndSaveNvsBlobs(1800u, 0u, saveFn, preEraseFn, nvsCtrl, nvsRt, tailWriteFn) == 2u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u && gMockNvs.baseSaveCount == 0u);
        RUIDO_FM1_CHECK(nvsRt.nextPatchSlot[1] == 1u && nvsRt.nextJournalSlot == 1u);

        // Tweak `KNOB 1` live WITHOUT saving: after 1.5 s idle, `LIVE` scratchpad appends 128 B
        // with ZERO sector erases and restores across power-off with 50% Home Base LED!
        nvsCtrl.onEncoderDelta(EncoderId::Knob1, 32);
        const float liveTweakedHz = nvsCtrl.state().voices[0].frequencyHz;
        nvsRt.lastPanelTouchMs = 2000u;
        RUIDO_FM1_CHECK(
            pollAndSaveNvsBlobs(3600u, 0u, saveFn, preEraseFn, nvsCtrl, nvsRt, tailWriteFn) == 1u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u && nvsRt.nextLiveSlot == 1u);

        // Simulate a torn/corrupt Settings slot 1 at `0x030`: cold boot recovers Scene 2 + LIVE
        // tweak and advances `nextJournalSlot` past the dirty slot (`2`)!
        gMockNvs.sectors[kNvsSettingsBlobIdx][kSettingsJournalBaseOff + 16u + 4u] = 0x01u;
        gMockNvs.sectors[kNvsSettingsBlobIdx][kSettingsJournalBaseOff + 16u + 0u] = 0xAAu;

        RuidoController bootCtrl2{};
        Fm1PanelState bootPanel2{};
        Fm1LedModulator bootLeds2{};
        Fm1NvsRuntimeState bootRt2{};
        RUIDO_FM1_CHECK(loadBootNvsBlobs(loadFn, bootCtrl2, bootPanel2, bootLeds2, 0.6f, bootRt2,
                                         tailReadFn, saveFn) ==
                        ((1u << 1u) | (1u << kNvsSettingsBlobIdx) | (1u << kNvsLivePatchIdx)));
        RUIDO_FM1_CHECK(bootCtrl2.snapshots().activeSlot() == 1u);
        RUIDO_FM1_CHECK(bootCtrl2.snapshots().isDirty());
        bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, true);
        bootCtrl2.tick(0.016f, dummyTel);
        RUIDO_FM1_CHECK(absf(bootCtrl2.leds().whiteKeyLeds[9] - 0.80f) < 1.0e-4f);
        bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, false);
        RUIDO_FM1_CHECK(absf(bootCtrl2.state().voices[0].frequencyHz - liveTweakedHz) < 0.2f);
        // Cold boot compacts LIVE (`afterLiveNonFf >= 1`) so all 30 journal slots start free (`0`)!
        RUIDO_FM1_CHECK(bootRt2.nextJournalSlot == 2u && bootRt2.nextLiveSlot == 0u);
        RUIDO_FM1_CHECK(gMockNvs.baseSaveCount == 1u);

        // 1-Tap Snap-Back on `SHIFT + Key 10` (`Scene 2`) glides back to `savedScene2Hz` and
        // appends an `EMPTY` marker to `LIVE` scratchpad (`sector 5`) with ZERO erases!
        bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, true);
        bootCtrl2.onWhiteKeyPressure(9u, 1.0f);
        bootCtrl2.onWhiteKeyGate(9u, true);
        bootCtrl2.onWhiteKeyGate(9u, false);
        bootCtrl2.onWhiteKeyPressure(9u, 0.0f);
        bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, false);
        for (int i = 0; i < 20; ++i) {
            bootCtrl2.tick(0.02f, dummyTel);
        }
        RUIDO_FM1_CHECK(!bootCtrl2.snapshots().isDirty());
        bootRt2.lastPanelTouchMs = 4000u;
        RUIDO_FM1_CHECK(pollAndSaveNvsBlobs(5600u, 0u, saveFn, preEraseFn, bootCtrl2, bootRt2,
                                            tailWriteFn) == 1u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u && bootRt2.nextLiveSlot == 1u);

        // Fill Settings journal past `kSettingsJournalBootCompactThreshold` (64) & Scene 2 journal
        // past `kPatchJournalBootCompactThreshold` (8) with zero live erases, then verify cold-boot
        // silent compaction resets both back to `0`!
        for (uint32_t i = 0u; i < kPatchJournalBootCompactThreshold; ++i) {
            bootCtrl2.onButtonEvent(ButtonId::Save, true);
            bootCtrl2.onButtonEvent(ButtonId::Save, false);
            bootCtrl2.onWhiteKeyGate(9u, true);
            bootCtrl2.onWhiteKeyGate(9u, false);
            (void)pollAndSaveNvsBlobs(6000u + i * 100u, 0u, saveFn, preEraseFn, bootCtrl2, bootRt2,
                                      tailWriteFn);
        }
        for (uint32_t i = 0u; i < kSettingsJournalBootCompactThreshold; ++i) {
            const uint8_t targetKey = static_cast<uint8_t>(8u + (i & 3u));
            bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, true);
            bootCtrl2.onWhiteKeyGate(targetKey, true);
            bootCtrl2.onWhiteKeyGate(targetKey, false);
            bootCtrl2.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, false);
            for (int t = 0; t < 35; ++t) {
                bootCtrl2.tick(0.02f, dummyTel);
            }
            bootRt2.lastPanelTouchMs = 8000u + i * 2000u;
            (void)pollAndSaveNvsBlobs(9600u + i * 2000u, 0u, saveFn, preEraseFn, bootCtrl2, bootRt2,
                                      tailWriteFn);
        }
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u);
        const uint8_t expectedFinalSlot = bootCtrl2.snapshots().activeSlot();

        RuidoController bootCtrl3{};
        Fm1PanelState bootPanel3{};
        Fm1LedModulator bootLeds3{};
        Fm1NvsRuntimeState bootRt3{};
        (void)loadBootNvsBlobs(loadFn, bootCtrl3, bootPanel3, bootLeds3, 0.6f, bootRt3, tailReadFn,
                               saveFn);
        RUIDO_FM1_CHECK(bootCtrl3.snapshots().activeSlot() == expectedFinalSlot);
        RUIDO_FM1_CHECK(!bootCtrl3.snapshots().isDirty());
        bootCtrl3.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, true);
        bootCtrl3.onWhiteKeyGate(9u, true);
        bootCtrl3.onWhiteKeyGate(9u, false);
        bootCtrl3.onBlackKeyEvent(BlackKeyId::ScopeFreezeKey, false);
        for (int t = 0; t < 35; ++t) {
            bootCtrl3.tick(0.02f, dummyTel);
        }
        RUIDO_FM1_CHECK(absf(bootCtrl3.state().voices[0].frequencyHz - savedScene2Hz) < 0.2f);
        RUIDO_FM1_CHECK(gMockNvs.baseSaveCount == 4u);
        RUIDO_FM1_CHECK(bootRt3.nextPatchSlot[1] == 0u && bootRt3.nextJournalSlot == 0u &&
                        bootRt3.nextLiveSlot == 0u);

        // Dirty mid-journal NOR slot (`0x00` byte -> `tailWriteFn` `-4`) steps past the dirty slot
        // into the next clean `0xFF` slot with ZERO live sector erases!
        bootRt3.lastPanelTouchMs = 195000u;
        (void)pollAndSaveNvsBlobs(197000u, 0u, saveFn, preEraseFn, bootCtrl3, bootRt3, tailWriteFn);
        RUIDO_FM1_CHECK(bootRt3.nextLiveSlot == 0u);
        gMockNvs.sectors[kNvsLivePatchIdx][kPatchJournalBaseOff] = 0x00u;
        bootCtrl3.onEncoderDelta(EncoderId::Knob1, 8);
        bootRt3.lastPanelTouchMs = 200000u;
        RUIDO_FM1_CHECK(pollAndSaveNvsBlobs(202000u, 0u, saveFn, preEraseFn, bootCtrl3, bootRt3,
                                            tailWriteFn) == 1u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u && bootRt3.nextLiveSlot == 2u);

        // Exhausting all 30 `LIVE` slots while `masterRun == true` performs ZERO mid-drone sector
        // erases (`pendingLiveSave` stays armed until `PLAY/STOP` mute or cold boot)!
        bootCtrl3.onButtonEvent(ButtonId::PlayStop, true);
        bootCtrl3.onButtonEvent(ButtonId::PlayStop, false);
        for (uint32_t i = 0u; i < kPatchJournalMaxSlots; ++i) {
            bootCtrl3.onEncoderDelta(EncoderId::Knob1, (i & 1u) != 0u ? -4 : 4);
            bootRt3.lastPanelTouchMs = 210000u + i * 2000u;
            (void)pollAndSaveNvsBlobs(211600u + i * 2000u, 0u, saveFn, preEraseFn, bootCtrl3,
                                      bootRt3, tailWriteFn);
        }
        RUIDO_FM1_CHECK(bootCtrl3.state().global.masterRun);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 0u);
        RUIDO_FM1_CHECK(bootRt3.nextLiveSlot == kPatchJournalMaxSlots &&
                        bootRt3.pendingLiveSave == 1u);

        // Toggling `PLAY/STOP` (`masterRun == false`) flushes and compacts `LIVE` safely!
        bootCtrl3.onButtonEvent(ButtonId::PlayStop, true);
        bootCtrl3.onButtonEvent(ButtonId::PlayStop, false);
        RUIDO_FM1_CHECK(!bootCtrl3.state().global.masterRun);
        RUIDO_FM1_CHECK(pollAndSaveNvsBlobs(280000u, 0u, saveFn, preEraseFn, bootCtrl3, bootRt3,
                                            tailWriteFn) == 1u);
        RUIDO_FM1_CHECK(gMockNvs.eraseCount == 1u && bootRt3.nextLiveSlot == 0u &&
                        bootRt3.pendingLiveSave == 0u);

        // Full-sector foreign non-`0xFF` bytes (`afterNonFf == MaxSlots`) with `tailReadOk == true`
        // clean at cold boot, whereas a `tailReadFn` read failure (`tailReadOk == false`) never
        // overwrites flash!
        gMockNvs.slotValid[2] = false;
        gMockNvs.sectors[2].fill(0xA5u);
        gMockNvs.slotValid[3] = false;
        gMockNvs.sectors[3].fill(0xA5u);
        RuidoController bootCtrl4{};
        Fm1PanelState bootPanel4{};
        Fm1LedModulator bootLeds4{};
        Fm1NvsRuntimeState bootRt4{};
        (void)loadBootNvsBlobs(loadFn, bootCtrl4, bootPanel4, bootLeds4, 0.6f, bootRt4, tailReadFn,
                               saveFn);
        RUIDO_FM1_CHECK(bootRt4.nextPatchSlot[2] == 0u &&
                        gMockNvs.sectors[2][kPatchJournalBaseOff] == 0xFFu);

        gMockNvs.slotValid[3] = false;
        gMockNvs.sectors[3].fill(0xA5u);
        const RuidoFm1TailReadFn failReadFn = [](uint32_t, uint32_t, void*, uint32_t) -> int {
            return -1;
        };
        RuidoController bootCtrl5{};
        Fm1PanelState bootPanel5{};
        Fm1LedModulator bootLeds5{};
        Fm1NvsRuntimeState bootRt5{};
        (void)loadBootNvsBlobs(loadFn, bootCtrl5, bootPanel5, bootLeds5, 0.6f, bootRt5, failReadFn,
                               saveFn);
        RUIDO_FM1_CHECK(bootRt5.nextPatchSlot[3] == kPatchJournalMaxSlots &&
                        gMockNvs.sectors[3][kPatchJournalBaseOff] == 0xA5u);
    }

    // 7. True-zero MASTER volume & honest "0%" readout regression:
    //    - `masterQ12ToGain` maps physical CCW pot offset (`master_q12 <= 36` / `k10 <= 96`) to
    //      exact `0.0f` and scales `(36..4088]` continuously up to `1.0f`.
    //    - `platform_poll()`'s 1-ms exponential `knob_avg` decay into `syncMasterPot` reaches
    //      exact `0.0f` (never trapped below the `0.0025f` jitter deadband) without idle re-wakes.
    //    - `formatPercent` reserves `"0%"` strictly for `<= kSilentAmpThreshold` (`0.0f` silence).
    {
        RUIDO_FM1_CHECK(masterQ12ToGain(0u) == 0.0f);
        RUIDO_FM1_CHECK(masterQ12ToGain(kMinPlatformMasterQ12) == 0.0f);
        RUIDO_FM1_CHECK(masterQ12ToGain(kMinPlatformMasterQ12 + 1u) > 0.0f);
        RUIDO_FM1_CHECK(masterQ12ToGain(kMinPlatformMasterQ12 + 1u) < 0.001f);
        RUIDO_FM1_CHECK(masterQ12ToGain(kMaxPlatformMasterQ12) == 1.0f);
        float prevG = 0.0f;
        for (uint32_t mq = kMinPlatformMasterQ12 + 1u; mq <= kMaxPlatformMasterQ12; ++mq) {
            const float g = masterQ12ToGain(mq);
            RUIDO_FM1_CHECK(g > prevG);
            prevG = g;
        }

        for (int32_t ccwAdc : {0, 48, 88}) {
            RuidoController zCtrl{};
            Fm1PanelState zPanel{};
            int32_t knobAvg = 512 * 16;
            const uint32_t initK10 = static_cast<uint32_t>(knobAvg / 16);
            const uint32_t initMq = (initK10 * initK10) >> 8u;
            zPanel.reset(masterQ12ToGain(initMq));
            zPanel.syncMasterPot(zCtrl, masterQ12ToGain(initMq));
            RUIDO_FM1_CHECK(zCtrl.state().global.masterTrim > 0.24f);

            for (int ms = 0; ms < 120; ++ms) {
                knobAvg += (ccwAdc * 16 - knobAvg) / 8;
                const auto k10 = static_cast<uint32_t>(knobAvg / 16);
                const uint32_t mq = (k10 * k10) >> 8u;
                zPanel.syncMasterPot(zCtrl, masterQ12ToGain(mq));
            }
            RUIDO_FM1_CHECK(zCtrl.state().global.masterTrim == 0.0f);
            RUIDO_FM1_CHECK(zCtrl.hud().mode == HudMode::MasterLevel);
            const uint32_t settledEvents = zPanel.inputEventCount();
            for (int ms = 0; ms < 20; ++ms) {
                zPanel.syncMasterPot(zCtrl, 0.0f);
            }
            RUIDO_FM1_CHECK(zPanel.inputEventCount() == settledEvents);
        }

        char pctBuf[12]{};
        formatPercent(pctBuf, sizeof(pctBuf), 0.0f);
        RUIDO_FM1_CHECK(std::strcmp(pctBuf, "0%") == 0);
        formatPercent(pctBuf, sizeof(pctBuf), 0.5f * kSilentAmpThreshold);
        RUIDO_FM1_CHECK(std::strcmp(pctBuf, "0%") == 0);
        formatPercent(pctBuf, sizeof(pctBuf), 0.0036f); // 1 Fine detent above 0 -> never lies "0%"
        RUIDO_FM1_CHECK(std::strcmp(pctBuf, "1%") == 0);
        RUIDO_FM1_CHECK(computeVoiceChannelGain(0.0f).postVca == 0.0f);
        RUIDO_FM1_CHECK(computeVoiceChannelGain(kSilentAmpThreshold).postVca == 0.0f);
    }

#undef RUIDO_FM1_CHECK
}

} // namespace ruido_test
