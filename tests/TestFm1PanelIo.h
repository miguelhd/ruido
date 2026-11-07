#pragma once

#include "../platforms/fm1/fm1_ops_screen.h"
#include "../platforms/fm1/fm1_panel_io.h"
#include "core/RuidoController.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

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
    int32_t encSteps[kFm1NumEncoders]{0, 0, 3, 0, 0, 0, 1}; // KNOB1 +3 detents, PRESETS +1 detent
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, 0u, 0u, 0u, encSteps));
    RUIDO_FM1_CHECK(ctrl.snapshots().activeSlot() == 1u);

    int32_t knobOnly[kFm1NumEncoders]{0, 0, 4, 0, 0, 0, 0};
    const float slot1Osc1Hz = ctrl.state().voices[0].frequencyHz;
    (void)baseOsc1Hz;
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, 0u, 0u, 0u, knobOnly));
    RUIDO_FM1_CHECK(ctrl.state().voices[0].frequencyHz > slot1Osc1Hz);

    // Intra-poll fast press+release on PLAY/STOP (`bit 12`) toggles masterRun `true -> false`
    int32_t zeroEnc[kFm1NumEncoders]{};
    const uint32_t playBit = 1u << static_cast<uint32_t>(ButtonId::PlayStop);
    RUIDO_FM1_CHECK(ctrl.state().global.masterRun);
    RUIDO_FM1_CHECK(panel.pollButtonsAndEncoders(ctrl, playBit, playBit, 0u, zeroEnc));
    RUIDO_FM1_CHECK(!ctrl.state().global.masterRun);

    // 3. Verify PA7/PA8 carbon-pill SARADC curve, digital fallback (`-1`), & round-robin scheduling
    RUIDO_FM1_CHECK(adcToCarbonPressure01(0) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(119) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(921) < 0.0f);
    RUIDO_FM1_CHECK(adcToCarbonPressure01(1023) < 0.0f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(800) - 0.18f) < 1.0e-4f);
    RUIDO_FM1_CHECK(std::fabs(adcToCarbonPressure01(240) - 1.0f) < 1.0e-4f);

    float prevP = 0.18f;
    for (int32_t adc = 770; adc >= 250; adc -= 20) {
        const float p = adcToCarbonPressure01(adc);
        RUIDO_FM1_CHECK(p >= prevP);
        prevP = p;
    }

    ctrl.reset();
    panel.reset(0.85f);
    panel.syncMasterPot(ctrl, 0.85f);

    // Digital fallback (`rawAdc == -1` as in `fm1-emu`): holding n7 (White Key 5 = OSC 1 filter
    // pad) immediately applies 100% pressure, and schedules PA8 (`ch 1`, `col 9`) for SARADC check.
    const uint32_t n7Mask = 1u << 7u;
    const uint32_t n9Mask = 1u << 9u;
    RUIDO_FM1_CHECK(panel.pollKeysAndPressure(ctrl, n7Mask, n7Mask));
    RUIDO_FM1_CHECK(ctrl.state().voicePressure[0] > 0.99f);
    const auto req1 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req1.active == 1u && req1.noteBit == 7u && req1.col == 9u &&
                    req1.saradcCh == 1u);

    // Hold both n7 (PA8 ch1 col 9) and n9 (PA7 ch0 col 0): round-robin alternates 7 <-> 9
    (void)panel.pollKeysAndPressure(ctrl, n7Mask | n9Mask, n9Mask);
    const auto req2 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req2.active == 1u && req2.noteBit == 9u && req2.col == 0u &&
                    req2.saradcCh == 0u);
    (void)panel.pollKeysAndPressure(ctrl, n7Mask | n9Mask, 0u);
    const auto req3 = panel.currentAdcRequest();
    RUIDO_FM1_CHECK(req3.active == 1u && req3.noteBit == 7u);

    // Feed real physical carbon-pill readings on n7: soft touch (760) -> firm press (260)
    (void)panel.pollKeysAndPressure(ctrl, 0u, 0u);
    panel.recordAdcSample(7u, 760);
    (void)panel.pollKeysAndPressure(ctrl, n7Mask, n7Mask);
    const float softPress = ctrl.state().voicePressure[0];
    RUIDO_FM1_CHECK(softPress >= 0.18f && softPress < 0.30f);

    panel.recordAdcSample(7u, 260);
    for (int step = 0; step < 12; ++step) {
        (void)panel.pollKeysAndPressure(ctrl, n7Mask, 0u);
    }
    RUIDO_FM1_CHECK(ctrl.state().voicePressure[0] > 0.95f);

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

    // Running state (`masterRun == true`): STOP red (col 10, row 2) is lit, PLAY green (col 8, row
    // 1) dark
    RUIDO_FM1_CHECK((lit[kStopRedCol] & (1u << kStopRedRow)) != 0u);
    RUIDO_FM1_CHECK((lit[kPlayGreenCol] & (1u << kPlayGreenRow)) == 0u);
    RUIDO_FM1_CHECK((bg[kPlayGreenCol] & (1u << kPlayGreenRow)) == 0u);
    // Rest panel buttons (e.g. FX at col 6, row 1 = 15% perceptual) sit cleanly on `fm1_led_bg`
    RUIDO_FM1_CHECK((bg[6u] & (1u << 1u)) != 0u);
    // White Keys 5-8 filter landmarks (e.g. n7 at col 9, row 4 = 24% perceptual) sit on
    // `fm1_led_dim`
    RUIDO_FM1_CHECK((dim[9u] & (1u << 4u)) != 0u);

    // Stop audio via PLAY/STOP: PLAY green (col 8, row 1) lights, STOP red (col 10, row 2) goes
    // dark
    ctrl.onButtonEvent(ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ButtonId::PlayStop, false);
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.025f, zeroTel);
    }
    ledMod.updateTargets(ctrl.leds());
    ledMod.stepFrame909Hz(lit, dim, bg);
    RUIDO_FM1_CHECK((lit[kPlayGreenCol] & (1u << kPlayGreenRow)) != 0u);
    RUIDO_FM1_CHECK((lit[kStopRedCol] & (1u << kStopRedRow)) == 0u);

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
        RuidoOpsState wrSt{
            RUIDO_OPS_LDR_WRITING, static_cast<uint8_t>((fill * 100u) / 220u), fill, 0u, 0u, 0u};
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
            RuidoOpsState ldrSt{ldrKind, pct, 110u, 0u, 0u, 0u};
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

#undef RUIDO_FM1_CHECK
}

} // namespace ruido_test
