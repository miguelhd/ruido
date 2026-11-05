#pragma once

#include "core/FilterModMath.h"
#include "core/GlobalConfig.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardModelSpec.h"
#include "core/Parameters.h"
#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/FontSansAA.h"
#include "ui/GraphStyle.h"
#include "ui/HudColumnText.h"
#include "ui/ScopeHudRenderer.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ruido_test {

template <typename CheckFn>
inline void runUnified150msFadesAndRateSplitHeaderTests(CheckFn checkFn) {
    std::printf(
        "[Test 8] Unified 150 ms Fades, Option A FINE/COARSE Header & Bug Regressions...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    ruido::EngineTelemetry tel{};

    // 1. Boot & mode-switch focus hygiene: no phantom knob column highlight until a knob is turned
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 4);
    RTEST_CHECK(ctrl.hud().focusedKnob == 2u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().fxVoiceFocus == 1u);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);

    // 2. Discrete notched detents (SELECT, TARGET, GLOBAL CONFIG) step 1 item per 4 ticks
    //    across all 3 gears (Standard, Fine, Coarse) across all 8 LFO slots (LFO A/B OSC 1..4)
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 0u); // LFO A OSC 1

    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -4); // LpCutoff(2) - 1 -> PulseWidth(1)
    RTEST_CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::PulseWidth);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -4); // PulseWidth(1) - 1 -> Pitch(0)
    RTEST_CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::Pitch);

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // FINE
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);    // 0 (LFO A OSC 1) -> 1 (LFO B OSC 1)
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 1u);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4); // LFO B OSC 1: LpCutoff(2) + 1 -> LpRes(3)
    RTEST_CHECK(ctrl.state().voices[0].lfoBTarget == ruido::LfoTarget::LpResonance);
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);

    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true); // COARSE
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);   // 1 (LFO B OSC 1) -> 2 (LFO A OSC 2)
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 2u);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4); // LFO A OSC 2: LpCutoff(2) + 1 -> LpRes(3)
    RTEST_CHECK(ctrl.state().voices[1].lfoTarget == ruido::LfoTarget::LpResonance);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);

    // 3. Anchored Left Title + Smooth 150 ms Top-Right FINE/COARSE Transition:
    //    (a) Idle OCT- hold shows "FREQUENCY" on left + "FINE" on right; turning Knob1 keeps
    //        "FREQUENCY" anchored on left while expanding top-right to "FINE · ... Hz"!
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().rateHoldActive);
    RTEST_CHECK(ctrl.hud().knobRate == ruido::KnobRate::Fine);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));

    std::array<uint16_t, 240u * 24u> hdrRateOnly{};
    std::array<uint16_t, 240u * 24u> hdrRateKnob{};
    ruido::EngineControlState silentSt = ctrl.state();
    silentSt.global.masterRun = false;
    silentSt.global.masterTrim = 0.0f;

    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateOnly.data());

    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 8);
    RTEST_CHECK(ctrl.hud().rateHoldActive);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    silentSt = ctrl.state();
    silentSt.global.masterRun = false;
    silentSt.global.masterTrim = 0.0f;
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateKnob.data());

    bool leftFreqLocked = true;
    size_t rightReadoutDiffs = 0u;
    for (size_t y = 6u; y <= 19u; ++y) {
        for (size_t x = 10u; x <= 80u; ++x) {
            if (hdrRateOnly[y * 240u + x] != hdrRateKnob[y * 240u + x]) {
                leftFreqLocked = false;
            }
        }
        for (size_t x = 140u; x <= 230u; ++x) {
            if (hdrRateOnly[y * 240u + x] != hdrRateKnob[y * 240u + x]) {
                ++rightReadoutDiffs;
            }
        }
    }
    RTEST_CHECK(leftFreqLocked);
    RTEST_CHECK(rightReadoutDiffs > 40u);

    // Releasing OCT- after turning a knob preserves the remaining 1.5 s toast hold!
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    RTEST_CHECK(!ctrl.hud().rateHoldActive);
    for (int i = 0; i < 10; ++i) {
        ctrl.tick(0.100f, tel);
    }
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));
    for (int i = 0; i < 10; ++i) {
        ctrl.tick(0.100f, tel);
    }
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));

    //    (b) On an open tool page (LFO OSC 1), holding OCT- keeps the entire left title
    //        ("LFO OSC 1", x = 10..120) 100% stationary while top-right transitions smoothly
    //        over 150 ms into "FINE" and back!
    ctrl.reset();
    renderer.reset();
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    ctrl.tick(0.200f, tel);
    silentSt = ctrl.state();
    silentSt.global.masterRun = false;
    silentSt.global.masterTrim = 0.0f;

    std::array<uint16_t, 240u * 24u> hdrPageBase{};
    std::array<uint16_t, 240u * 24u> hdrPageMid{};
    std::array<uint16_t, 240u * 24u> hdrPageFine{};
    std::array<uint16_t, 240u * 24u> hdrPageRestored{};
    renderer.prepareFrame(0.200f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrPageBase.data());

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
    ctrl.tick(0.075f, tel);
    renderer.prepareFrame(0.075f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrPageMid.data());

    ctrl.tick(0.120f, tel);
    renderer.prepareFrame(0.120f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrPageFine.data());

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    ctrl.tick(0.200f, tel);
    renderer.prepareFrame(0.200f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrPageRestored.data());

    bool pageLeftStationary = true;
    size_t midVsBaseRight = 0u;
    size_t midVsFineRight = 0u;
    bool rightRestoredCleanly = true;
    for (size_t y = 6u; y <= 20u; ++y) {
        for (size_t x = 10u; x <= 120u; ++x) {
            const size_t idx = y * 240u + x;
            if (hdrPageBase[idx] != hdrPageMid[idx] || hdrPageBase[idx] != hdrPageFine[idx]) {
                pageLeftStationary = false;
            }
        }
        for (size_t x = 155u; x <= 232u; ++x) {
            const size_t idx = y * 240u + x;
            if (hdrPageMid[idx] != hdrPageBase[idx]) {
                ++midVsBaseRight;
            }
            if (hdrPageMid[idx] != hdrPageFine[idx]) {
                ++midVsFineRight;
            }
            if (hdrPageRestored[idx] != hdrPageBase[idx]) {
                rightRestoredCleanly = false;
            }
        }
    }
    RTEST_CHECK(pageLeftStationary);
    RTEST_CHECK(midVsBaseRight > 10u && midVsFineRight > 10u);
    RTEST_CHECK(rightRestoredCleanly);

    // Verify ZERO underline pixels below the top-right FINE/COARSE badge baseline (y = 18..20)
    uint16_t maxUnderlineG6 = 0u;
    for (size_t y = 18u; y <= 20u; ++y) {
        for (size_t x = 155u; x <= 232u; ++x) {
            const uint16_t g6 = (hdrPageFine[y * 240u + x] >> 5u) & 0x3Fu;
            if (g6 > maxUnderlineG6) {
                maxUnderlineG6 = g6;
            }
        }
    }
    RTEST_CHECK(maxUnderlineG6 == 0u);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.200f, tel);

    // Pressing HOME while OCT+ is physically held preserves COARSE hold cleanly
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.tick(0.200f, tel);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.050f, tel);
    RTEST_CHECK(!ctrl.hud().lfoPageOpen);
    RTEST_CHECK(ctrl.hud().rateHoldActive);
    RTEST_CHECK(ctrl.hud().knobRate == ruido::KnobRate::Coarse);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KnobRate);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    ctrl.tick(0.075f, tel); // Halfway through 150 ms fade-out
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, tel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));

    // 4. Bug-fix regressions:
    // (a) Slow MASTER pot sweep (0.0006 per call) wakes MasterLevel once cumulative delta > 0.002
    ctrl.reset();
    float pot = ctrl.state().global.masterTrim;
    for (int i = 0; i < 6; ++i) {
        pot -= 0.0006f;
        ctrl.onMasterPot(pot);
    }
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterLevel);

    // (b) SELECT unassigned when idle vs PRESETS stepped SNAP 1..4 recall & glide
    ctrl.reset();
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedEncoder);
    RTEST_CHECK(ctrl.hud().lastEncoder == ruido::EncoderId::Select);
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateOnly.data());

    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4); // Step SNAP 1 -> SNAP 2
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::SnapshotSlot);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Snapshot);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(ctrl.hud().snapshotGliding);
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateKnob.data());
    size_t encTitleDiffs = 0u;
    for (size_t i = 0u; i < hdrRateOnly.size(); ++i) {
        if (hdrRateOnly[i] != hdrRateKnob[i]) {
            ++encTitleDiffs;
        }
    }
    RTEST_CHECK(encTitleDiffs > 25u);
    ctrl.reset();

    // (c) Armed SAVE cancelled by unassigned switch (SEL) clears orphaned saveArmed LED state
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(!ctrl.hud().saveArmed);

    // (d) HOME from active LFO page fades out in 150 ms with zero ghost LFO wave
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 200); // High depth LFO
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, silentSt, ctrl.hud(), tel);
    }
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.075f, tel); // Mid-150ms fade
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    renderer.prepareFrame(0.075f, silentSt, ctrl.hud(), tel);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, silentSt, ctrl.hud(), tel);
    }
    static std::array<uint16_t, 240u * 240u> settledFrame{};
    renderer.renderFullScreenStrips(settledFrame.data());
    size_t nonHorizonLit = 0u;
    for (size_t y = 30u; y <= 105u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            if (((settledFrame[y * 240u + x] >> 5u) & 0x3Fu) > 2u) {
                ++nonHorizonLit;
            }
        }
    }
    RTEST_CHECK(nonHorizonLit == 0u);
#undef RTEST_CHECK
}

template <typename CheckFn> inline void runGlobalFiveModesAndConfigMenuTests(CheckFn checkFn) {
    std::printf("[Test 9] GlobalFilter 4-Voice Spread & Clean GLOBAL CONFIG Menu...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    ruido::EngineTelemetry tel{};
    ruido::LfoSignalArray zeroLfo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // 1. GlobalFilter 4-voice micro-bead spread evaluation (shared across GLO LOW PASS / SCREAM /
    // PLUCK)
    ruido::EngineControlState gloSt = ruido::EngineControlState::makeDefault();
    gloSt.voices[0].lpCutoffHz = 4000.0f;
    const auto fltCol = ruido::evaluateGlobalFilterColumn(2u, gloSt, zeroLfo);
    RTEST_CHECK(fltCol.hasVoiceSpread);

    static std::array<uint16_t, 240u * 240u> s16{};
    static std::array<uint16_t, 240u * 240u> s8{};
    static std::array<uint16_t, 240u * 240u> s24{};

    const auto verifyViewStripsAndCenter = [&](bool expectBottomStrip) {
        renderer.renderFullScreenStrips(s16.data());
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
            renderer.renderStrip(y, 8u, s8.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
            renderer.renderStrip(y, 24u, s24.data() + static_cast<size_t>(y) * 240u);
        }
        bool eq = true;
        bool mono = true;
        size_t centerPx = 0u;
        size_t botPx = 0u;
        for (size_t y = 0u; y < 240u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                const size_t i = y * 240u + x;
                const uint16_t px = s16[i];
                if (px != s8[i] || px != s24[i]) {
                    eq = false;
                }
                const uint16_t r5 = (px >> 11u) & 0x1Fu;
                const uint16_t g6 = (px >> 5u) & 0x3Fu;
                const uint16_t b5 = px & 0x1Fu;
                if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
                    mono = false;
                }
                if (y >= 42u && y <= 182u && y != 120u && g6 >= 10u) {
                    ++centerPx;
                }
                const size_t botCheckY = expectBottomStrip ? 191u : 210u;
                if (y >= botCheckY && y <= 232u && g6 >= 8u) {
                    ++botPx;
                }
            }
        }
        RTEST_CHECK(eq);
        RTEST_CHECK(mono);
        RTEST_CHECK(centerPx > 120u);
        if (expectBottomStrip) {
            RTEST_CHECK(botPx > 150u);
        } else {
            RTEST_CHECK(botPx == 0u);
        }
    };

    // 2. Right-panel GLO = GLOBAL CONFIG menu (OUTPUT ROUTING clean at Row 0)
    ctrl.reset();
    RTEST_CHECK(
        nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)], 0.15f));
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(ctrl.hud().configPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    RTEST_CHECK(ctrl.hud().configCursor == 0u);

    ruido::EngineControlState quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    renderer.reset();
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    }
    RTEST_CHECK(
        nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)], 1.0f));
    verifyViewStripsAndCenter(false); // Bottom 4-knob strip strictly hidden!

    uint16_t maxConfigSepG6 = 0u;
    uint16_t maxSelectedLeftEdgeG6 = 0u;
    for (size_t x = 14u; x <= 226u; ++x) {
        const uint16_t g6Sep0 = (s16[69u * 240u + x] >> 5u) & 0x3Fu;
        if (g6Sep0 > maxConfigSepG6) {
            maxConfigSepG6 = g6Sep0;
        }
    }
    for (size_t y = 44u; y < 68u; ++y) {
        for (size_t x = 10u; x <= 13u; ++x) {
            const uint16_t g6Edge = (s16[y * 240u + x] >> 5u) & 0x3Fu;
            if (g6Edge > maxSelectedLeftEdgeG6) {
                maxSelectedLeftEdgeG6 = g6Edge;
            }
        }
    }
    RTEST_CHECK(maxConfigSepG6 == 0u);
    RTEST_CHECK(maxSelectedLeftEdgeG6 <= 10u);

    // Knob1 turns Osc 1 pitch without altering OUTPUT ROUTING, then returns to GlobalConfig
    const float osc1BeforeMenuTurn = ctrl.state().voices[0].frequencyHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 8);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > osc1BeforeMenuTurn * 1.10f);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    for (int i = 0; i < 18; ++i) {
        ctrl.tick(0.100f, tel);
    }
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);

    // Row 0: OUTPUT ROUTING -> step CW to MONO via ALGORITHM, then back to STEREO
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::OutputRouting,
                                                          ctrl.hud().config, ctrl.state().global),
                            "STEREO") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::OutputRouting,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MONO") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);

    // Row 1 (MUTE MODE) & Row 2 (SOLO MODE): MOMENTARY (default) <-> LATCHING via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 1u);
    RTEST_CHECK(
        std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MuteMode), "MUTE MODE") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MuteMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MOMENTARY") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.muteMode == ruido::KeySwitchMode::Latching);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MuteMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "LATCHING") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(ctrl.hud().config.muteMode == ruido::KeySwitchMode::Momentary);

    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 2u);
    RTEST_CHECK(
        std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::SoloMode), "SOLO MODE") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::SoloMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MOMENTARY") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.soloMode == ruido::KeySwitchMode::Latching);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::SoloMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "LATCHING") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(ctrl.hud().config.soloMode == ruido::KeySwitchMode::Momentary);

    // Row 3 (SCENE MODE): LATCHING (default) <-> MOMENTARY via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 3u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::SnapMode),
                            "SCENE MODE") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::SnapMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "LATCHING") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(ctrl.hud().config.snapMode == ruido::KeySwitchMode::Momentary);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::SnapMode,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MOMENTARY") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.snapMode == ruido::KeySwitchMode::Latching);

    // Row 4 (MIDI CLOCK IN): AUTO (default) <-> OFF via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 4u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::ClockSync),
                            "MIDI CLOCK IN") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::ClockSync,
                                                          ctrl.hud().config, ctrl.state().global),
                            "AUTO") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(ctrl.hud().config.clockSync == ruido::ClockSyncMode::Internal);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::ClockSync,
                                                          ctrl.hud().config, ctrl.state().global),
                            "OFF") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.clockSync == ruido::ClockSyncMode::Auto);

    // Row 5 (MIDI TRANSPORT IN): PHASE (default) <-> RUN/STOP via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 5u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MidiTransport),
                            "MIDI TRANSPORT IN") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiTransport,
                                                          ctrl.hud().config, ctrl.state().global),
                            "PHASE") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::RunStop);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiTransport,
                                                          ctrl.hud().config, ctrl.state().global),
                            "RUN/STOP") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::PhaseOnly);

    // Row 6 (MIDI CHANNEL IN): OMNI (default) <-> CH 1..CH 16, clamped at last row (6) + scrolled
    // 6-row viewport check
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 6u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8); // Clamped at last row (6)
    RTEST_CHECK(ctrl.hud().configCursor == 6u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MidiChannel),
                            "MIDI CHANNEL IN") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiChannel,
                                                          ctrl.hud().config, ctrl.state().global),
                            "OMNI") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.midiChannel == 1u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiChannel,
                                                          ctrl.hud().config, ctrl.state().global),
                            "CH 1") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 80); // Clamped at CH 16
    RTEST_CHECK(ctrl.hud().config.midiChannel == 16u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiChannel,
                                                          ctrl.hud().config, ctrl.state().global),
                            "CH 16") == 0);
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false); // Scrolled 6-row viewport renders cleanly without bottom bar
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -80); // Clamped back to OMNI (0)
    RTEST_CHECK(ctrl.hud().config.midiChannel == 0u);

    // Close GLOBAL CONFIG via 2nd GloRight tap -> 150 ms smooth fade-out
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(!ctrl.hud().configPageOpen);
    ctrl.tick(0.075f, tel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, tel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));
#undef RTEST_CHECK
}

template <typename CheckFn> inline void runEditChannelDrivePageAndAmpGraphTests(CheckFn checkFn) {
    std::printf(
        "[Test 10] 4-Mode Latching EDIT Page (WIDTH · DRIFT · LEVEL/Drive · PAN) & Visuals...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::DroneEngine engine;
    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    engine.init(ruido::kBaseSampleRate);
    ctrl.attachControlBus(&engine.controlBus());
    engine.snapParametersToLatest();

    // 1. Tap EDIT: opens latched at EditTarget::Level (Channel Level & 4.5x Drive),
    //    mutually exclusive with FX/LFO/CONFIG/SAVE
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    RTEST_CHECK(ctrl.hud().fxPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    RTEST_CHECK(ctrl.hud().lfoPageOpen);
    RTEST_CHECK(!ctrl.hud().fxPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(ctrl.hud().editPageOpen);
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Level);
    RTEST_CHECK(!ctrl.hud().fxPageOpen);
    RTEST_CHECK(!ctrl.hud().lfoPageOpen);
    RTEST_CHECK(!ctrl.hud().configPageOpen);
    RTEST_CHECK(!ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);

    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, engine.telemetry());
    }
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 1.0f));

    // 2. Discrete SELECT steps across WIDTH(0) · DRIFT(1) · LEVEL(2) · PAN(3) (1 item / 4 ticks)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -8); // Level(2) -> Width(0)
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Width);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::PulseWidth);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::PulseWidth);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -80);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 80);
    RTEST_CHECK(ctrl.state().voices[0].pulseWidth < 0.25f);
    RTEST_CHECK(ctrl.state().voices[3].pulseWidth > 0.75f);

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // Gear-invariant Fine detent
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);    // Width(0) -> Drift(1)
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Drift);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceDrift);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceDrift);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -80); // Lock Osc 1 to 0% drift
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 120); // Boost Osc 2 thermal drift
    RTEST_CHECK(nearF(ctrl.state().voices[0].drift, 0.0f));
    RTEST_CHECK(ctrl.state().voices[1].drift > 0.55f);

    // 2nd EDIT tap advances to Page 2 (Spatial PAN); 3rd tap closes; 1st tap re-opens Page 1
    // remembering last EditTarget (Drift -> step to Level)!
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 60);
    RTEST_CHECK(ctrl.state().voices[2].pan > -0.20f);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Drift);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Drift(1) -> Level(2)
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Level);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);

    // KNOB1 -> 100% (4.5x Drive kept 100% intact!), mute Voice 1 via 0% Level lower rail
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 200);
    RTEST_CHECK(nearF(ctrl.state().voices[0].amplitude, 1.0f));
    RTEST_CHECK(nearF(ruido::computeVoiceChannelGain(ctrl.state().voices[0].amplitude).preDrive,
                      4.5f, 1.0e-3f));
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -240);
    RTEST_CHECK(ctrl.state().voices[1].muted);

    // 3. Momentary Keybed punch-in overrides EDIT temporarily and returns seamlessly on release
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    RTEST_CHECK(ctrl.hud().editPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));

    // 4. HudAmpGraph 8/16/24-row strip equivalence, strict monochrome neutrality & drive wings
    ruido::EngineControlState quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    ruido::EngineTelemetry driveTel{};
    driveTel.voicePeak[0].store(0.92f, std::memory_order_relaxed);
    driveTel.voicePeak[2].store(0.45f, std::memory_order_relaxed);

    static std::array<uint16_t, 240u * 240u> amp16{};
    static std::array<uint16_t, 240u * 240u> amp8{};
    static std::array<uint16_t, 240u * 240u> amp24{};
    static std::array<uint16_t, 240u * 240u> clean16{};

    for (int f = 0; f < 12; ++f) {
        renderer.prepareFrame(0.032f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(amp16.data());
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
        renderer.renderStrip(y, 8u, amp8.data() + static_cast<size_t>(y) * 240u);
    }
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
        renderer.renderStrip(y, 24u, amp24.data() + static_cast<size_t>(y) * 240u);
    }

    bool eqStrips = true;
    bool monoOk = true;
    for (size_t i = 0u; i < amp16.size(); ++i) {
        const uint16_t px = amp16[i];
        if (px != amp8[i] || px != amp24[i]) {
            eqStrips = false;
        }
        const uint16_t r5 = (px >> 11u) & 0x1Fu;
        const uint16_t g6 = (px >> 5u) & 0x3Fu;
        const uint16_t b5 = px & 0x1Fu;
        if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
            monoOk = false;
        }
    }
    RTEST_CHECK(eqStrips);
    RTEST_CHECK(monoOk);

    // Verify 0-70% clean stem (quiet background) stays razor-sharp on Column 0 with zero wide
    // post-filter `voicePeak` plasma halo to the left or right of the stem!
    const auto col0X = static_cast<size_t>(ruido::hudColumnCenterX(0u));
    const auto cleanStemTopY = static_cast<size_t>(ruido::HudAmpGraph::kKneeY) + 4u;
    const auto cleanStemBotY = static_cast<size_t>(ruido::HudAmpGraph::kColYBot) - 11u;
    size_t cleanStemSideHaloPx = 0u;
    for (size_t y = cleanStemTopY; y <= cleanStemBotY; ++y) {
        if (y == 120u) {
            continue; // quiet scope flatline horizon
        }
        for (size_t x = col0X - 10u; x <= col0X - 5u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                ++cleanStemSideHaloPx;
            }
        }
        for (size_t x = col0X + 5u; x <= col0X + 10u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                ++cleanStemSideHaloPx;
            }
        }
    }
    RTEST_CHECK(cleanStemSideHaloPx == 0u);

    // Compare Column 0 upper overdrive zone between 100% (4.5x drive, Option 3A narrow triangular
    // hairy CRT cap) and clean 65% VCA:
    //  - Narrow V-triangle lights center hair while keeping wide outer regions 100% clear.
    static ruido::DroneEngine drvEng;
    static std::array<float, 128> drvOutL{};
    static std::array<float, 128> drvOutR{};
    drvEng.init(44100.0f);
    ruido::EngineControlState hotLiveSt = quietSt;
    hotLiveSt.global.masterRun = true;
    hotLiveSt.global.masterTrim = 0.85f;
    drvEng.controlBus().publish(hotLiveSt);
    drvEng.snapParametersToLatest();
    for (int b = 0; b < 36; ++b) {
        drvEng.processBlock(drvOutL.data(), drvOutR.data(), 128u);
        renderer.ingestScopeRing(drvEng.scopeBuffer());
    }
    for (int f = 0; f < 12; ++f) {
        renderer.prepareFrame(0.032f, hotLiveSt, ctrl.hud(), drvEng.telemetry());
    }
    renderer.renderFullScreenStrips(amp16.data());

    ruido::EngineControlState cleanSt = hotLiveSt;
    cleanSt.voices[0].amplitude = 0.65f;
    for (int f = 0; f < 12; ++f) {
        renderer.prepareFrame(0.032f, cleanSt, ctrl.hud(), drvEng.telemetry());
    }
    renderer.renderFullScreenStrips(clean16.data());

    const auto drvTopY = static_cast<size_t>(ruido::HudAmpGraph::kColYTop);
    const auto drvBotY = static_cast<size_t>(ruido::HudAmpGraph::kKneeY) - 5u;
    size_t driveZonePxHot = 0u;
    size_t driveZonePxClean = 0u;
    size_t wideOuterPxHot = 0u;
    for (size_t y = drvTopY; y <= drvBotY; ++y) {
        for (size_t x = col0X - 6u; x <= col0X + 6u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                ++driveZonePxHot;
            }
            if (((clean16[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                ++driveZonePxClean;
            }
        }
        for (size_t x = col0X - 22u; x <= col0X - 8u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
                ++wideOuterPxHot;
            }
        }
        for (size_t x = col0X + 8u; x <= col0X + 22u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
                ++wideOuterPxHot;
            }
        }
    }
    RTEST_CHECK(driveZonePxHot > driveZonePxClean + 30u);
    RTEST_CHECK(wideOuterPxHot == 0u);

    // Verify locked-in nomenclature strings across LFO, GLOBAL, and GLOBAL CONFIG
    RTEST_CHECK(std::strcmp(ruido::lfoTargetName(ruido::LfoTarget::Pitch), "FREQ") == 0);
    RTEST_CHECK(std::strcmp(ruido::lfoTargetName(ruido::LfoTarget::PulseWidth), "DUTY") == 0);
    RTEST_CHECK(std::strcmp(ruido::lfoTargetName(ruido::LfoTarget::Amplitude), "LEVEL") == 0);
    RTEST_CHECK(std::strcmp(ruido::lfoShapeLabel(0.88f), "SQR-RND") == 0);
    RTEST_CHECK(std::strcmp(ruido::lfoShapeLabel(1.00f), "RANDOM") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::OutputRouting),
                            "OUTPUT ROUTING") == 0);

    // Verify 240x8 == 240x16 == 240x24 strip equivalence & monochrome neutrality across all 3 EDIT
    // modes (DUTY · DRIFT · LEVEL) + EDIT Page 2 (PAN)
    driveTel.voiceDriftNorm[2].store(0.35f, std::memory_order_relaxed);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -16); // Clamp to Width(0)
    for (int em = 0; em < 4; ++em) {
        if (em == 3) {
            ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
            ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
        }
        renderer.reset();
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.025f, driveTel);
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
        }
        renderer.renderFullScreenStrips(amp16.data());
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
            renderer.renderStrip(y, 8u, amp8.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
            renderer.renderStrip(y, 24u, amp24.data() + static_cast<size_t>(y) * 240u);
        }
        bool mEq = true;
        bool mMono = true;
        size_t mCenterPx = 0u;
        for (size_t y = 0u; y < 240u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                const size_t idx = y * 240u + x;
                const uint16_t px = amp16[idx];
                if (px != amp8[idx] || px != amp24[idx]) {
                    mEq = false;
                }
                const uint16_t r5 = (px >> 11u) & 0x1Fu;
                const uint16_t g6 = (px >> 5u) & 0x3Fu;
                const uint16_t b5 = px & 0x1Fu;
                if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
                    mMono = false;
                }
                if (y >= 46u && y <= 178u && y != 120u && g6 >= 8u) {
                    ++mCenterPx;
                }
            }
        }
        RTEST_CHECK(mEq);
        RTEST_CHECK(mMono);
        RTEST_CHECK(mCenterPx > 80u);
        if (em < 2) {
            ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
        }
    }

    // Close EDIT Page 2 (PAN) via 3rd EDIT tap -> smooth 150 ms fade-out & EDIT LED returns to 15%!
    RTEST_CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    ctrl.tick(0.200f, driveTel);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 1.0f));
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    ctrl.tick(0.075f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));

    // Closed EDIT LED returns strictly to 15% dim (even when Osc 1 is at 100% / 4.5x overdrive)
    driveTel.voicePeak[0].store(0.95f, std::memory_order_relaxed);
    ctrl.tick(0.016f, driveTel);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 0.15f));

    // 6. 1-to-1 Lower-Rail OSC 1-4 Dimming & 0% / Sub-Audio Alignment (Zero Screen Pulsing):
    ctrl.reset();
    renderer.reset();
    const auto maxOscLabelG6 = [&](const std::array<uint16_t, 240u * 240u>& img, size_t col) {
        uint16_t maxG6 = 0u;
        const auto colCx = static_cast<size_t>(ruido::hudColumnCenterX(col));
        const size_t x0 = colCx - 22u;
        const size_t x1 = colCx + 22u;
        const auto lblY0 = static_cast<size_t>(ruido::kHudBotLabelY);
        for (size_t y = lblY0; y <= lblY0 + 10u; ++y) {
            for (size_t x = x0; x <= x1; ++x) {
                const uint16_t g6 = (img[y * 240u + x] >> 5u) & 0x3Fu;
                if (g6 > maxG6) {
                    maxG6 = g6;
                }
            }
        }
        return maxG6;
    };

    // (a) Mute Osc 1 via EDIT Level 0% (focusedKnob == 0): OSC 1 text dims immediately (never 255
    //     white!) and exhibits ZERO pulsing across frames!
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -240);
    RTEST_CHECK(ctrl.state().voices[0].muted);
    RTEST_CHECK(ctrl.hud().focusedKnob == 0u);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(amp16.data());
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) <= 22u);
    RTEST_CHECK(maxOscLabelG6(amp16, 2u) >= 35u);

    for (int f = 0; f < 15; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(clean16.data());
    bool lowerRailSteadyOnMute = true;
    for (size_t y = 190u; y < 240u; ++y) {
        for (size_t x = 0u; x < 60u; ++x) {
            if (amp16[y * 240u + x] != clean16[y * 240u + x]) {
                lowerRailSteadyOnMute = false;
            }
        }
    }
    RTEST_CHECK(lowerRailSteadyOnMute);

    // (b) Unmute Osc 1 on EDIT Level, return to Frequency, then sweep Osc 1 into sub-audio (< 20
    //     Hz, down to 0.05 Hz lower rail): White Key 1 breathes at 0.35 Hz while OSC 1 text dims!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 140);
    RTEST_CHECK(!ctrl.state().voices[0].muted);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -120);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, ruido::kAudioFreqMinHz));
    RTEST_CHECK(ctrl.hud().focusedKnob == 0u);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 30; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    const float ledSubA = ctrl.leds().whiteKeyLeds[0];
    renderer.renderFullScreenStrips(amp16.data());
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) <= 22u);
    RTEST_CHECK(maxOscLabelG6(amp16, 2u) >= 35u);

    for (int f = 0; f < 20; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    const float ledSubB = ctrl.leds().whiteKeyLeds[0];
    RTEST_CHECK(std::fabs(ledSubB - ledSubA) > 0.008f);
    renderer.renderFullScreenStrips(clean16.data());
    bool lowerRailSteadySubAudio = true;
    for (size_t y = 190u; y < 240u; ++y) {
        for (size_t x = 0u; x < 60u; ++x) {
            if (amp16[y * 240u + x] != clean16[y * 240u + x]) {
                lowerRailSteadySubAudio = false;
            }
        }
    }
    RTEST_CHECK(lowerRailSteadySubAudio);

    // Sweep Osc 1 back >= 25 Hz -> OSC 1 undims to crisp focused white!
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 80);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > 50.0f);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(amp16.data());
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) >= 60u);

    // (c) EDIT Level 0% lower rail auto-mutes & Fine +1 CW unmutes cleanly:
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -240);
    RTEST_CHECK(nearF(ctrl.state().voices[0].amplitude, 0.0f));
    RTEST_CHECK(ctrl.state().voices[0].muted);

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // Fine gear: +1 tick = 0.00090 > 1e-4
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 1);
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    RTEST_CHECK(!ctrl.state().voices[0].muted);
    RTEST_CHECK(ctrl.state().voices[0].amplitude > 0.0005f);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 140); // Restore ~65% audible level

    // (d) EDIT Page 2 (PAN) Edge-to-Edge Stereo Panorama Curve (2nd EDIT tap):
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.tick(0.016f, driveTel);
    RTEST_CHECK(ctrl.hud().panPageOpen && !ctrl.hud().editPageOpen);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 1.0f));

    // SELECT switches STEREO (0) <-> MONO (1) right on SEL while keeping HudMode::Panning ("PAN"
    // title stays steady, never flashing "OUTPUT ROUTING") and preserving stored voice pan values!
    const float preSelectPan0 = ctrl.state().voices[0].pan;
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // +1 detent -> MONO
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning &&
                ctrl.hud().knobBank == ruido::KnobBankMode::Panning &&
                ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // -1 detent -> STEREO
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning &&
                ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    RTEST_CHECK(nearF(ctrl.state().voices[0].pan, preSelectPan0));

    // Pan all 4 voices hard Left (L100 = -1.0) first to verify crisp Left-stage curve & zero text
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -240);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -240);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -240);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -240);
    RTEST_CHECK(nearF(ctrl.state().voices[0].pan, -1.0f));
    RTEST_CHECK(nearF(ctrl.state().voices[3].pan, -1.0f));

    renderer
        .reset(); // Clear residual background scope history so silent render has pure #000000 bg
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.025f, driveTel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(amp16.data());

    const auto sumBoxG6 = [&](const std::array<uint16_t, 240u * 240u>& img, size_t x0, size_t x1,
                              size_t y0, size_t y1) {
        uint32_t sum = 0u;
        for (size_t y = y0; y <= y1; ++y) {
            for (size_t x = x0; x <= x1; ++x) {
                sum += (img[y * 240u + x] >> 5u) & 0x3Fu;
            }
        }
        return sum;
    };
    const auto maxBoxG6 = [&](const std::array<uint16_t, 240u * 240u>& img, size_t x0, size_t x1,
                              size_t y0, size_t y1) {
        uint16_t maxG = 0u;
        for (size_t y = y0; y <= y1; ++y) {
            for (size_t x = x0; x <= x1; ++x) {
                const uint16_t g6 = (img[y * 240u + x] >> 5u) & 0x3Fu;
                if (g6 > maxG) {
                    maxG = g6;
                }
            }
        }
        return maxG;
    };

    const auto baseY = static_cast<size_t>(ruido::kGraphBaselineY);
    const auto graphTopY = static_cast<size_t>(ruido::kGraphTopY);
    const uint32_t leftPanorama = sumBoxG6(amp16, 4u, 108u, 44u, baseY - 6u);
    const uint32_t rightPanorama = sumBoxG6(amp16, 132u, 235u, 44u, baseY - 6u);
    RTEST_CHECK(leftPanorama > rightPanorama * 4u);
    // Edge-to-edge baseline filament is present on both sides; zero floating top text!
    RTEST_CHECK(maxBoxG6(amp16, 200u, 239u, baseY - 1u, baseY + 1u) >= 6u);
    RTEST_CHECK(maxBoxG6(amp16, 4u, 235u, graphTopY, graphTopY + 7u) == 0u);

    // Spread Osc 1 -> L100, Osc 2 -> C + LFO->PAN, Osc 3 -> R45, Osc 4 -> R100 (focusedKnob = 3):
    // verify Osc 4 spotlight outshines unfocused Osc 1 companion & 240x8 == 240x16 == 240x24!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 90);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 180);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 260);
    RTEST_CHECK(nearF(ctrl.state().voices[3].pan, 1.0f));
    RTEST_CHECK(ctrl.hud().focusedKnob == 3u);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    quietSt.voices[0].amplitude = 0.65f;
    quietSt.voices[3].amplitude = 0.65f;
    quietSt.voices[1].lfoTarget = ruido::LfoTarget::Pan;
    quietSt.voices[1].lfoDepth = 0.55f;
    for (int f = 0; f < 16; ++f) {
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(amp16.data());
    const uint32_t unfocusedL100Box = sumBoxG6(amp16, 4u, 32u, 70u, 180u);
    const uint32_t spotlightR100Box = sumBoxG6(amp16, 207u, 235u, 70u, 180u);
    RTEST_CHECK(spotlightR100Box > unfocusedL100Box * 2u);

    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
        renderer.renderStrip(y, 8u, amp8.data() + static_cast<size_t>(y) * 240u);
    }
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
        renderer.renderStrip(y, 24u, amp24.data() + static_cast<size_t>(y) * 240u);
    }
    bool panPanoramaStripsEq = true;
    for (size_t i = 0u; i < amp16.size(); ++i) {
        if (amp16[i] != amp8[i] || amp16[i] != amp24[i]) {
            panPanoramaStripsEq = false;
        }
    }
    RTEST_CHECK(panPanoramaStripsEq);

    // Turning SELECT CW on SEL switches STEREO -> MONO while keeping the top-left title steady as
    // "PAN" ("PAN · MONO", x = 48..126 empty immediately!), replaces OSC 1-4 bottom columns with
    // centered "OUTPUT ROUTING SET TO MONO", strictly locks KNOB 1-4 (zero state or pixel change),
    // and preserves stereo pan positions intact when SELECT switches back to STEREO!
    const std::array<float, 4> savedStereoPan = {
        ctrl.state().voices[0].pan,
        ctrl.state().voices[1].pan,
        ctrl.state().voices[2].pan,
        ctrl.state().voices[3].pan,
    };
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Turn SELECT CW -> MONO!
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning &&
                ctrl.hud().knobBank == ruido::KnobBankMode::Panning &&
                ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);

    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 16; ++f) {
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
    }
    renderer.renderFullScreenStrips(clean16.data());
    RTEST_CHECK(maxBoxG6(clean16, 48u, 126u, 6u, 21u) == 0u); // Title stays short "PAN"!

    const ruido::LfoSignalArray zeroLfo{};
    char hdrSuffix[16]{};
    char hdrRight[24]{};
    ruido::formatHeaderPanMode(ctrl.state(), ctrl.hud(), 0x0Fu, hdrSuffix, sizeof(hdrSuffix),
                               hdrRight, sizeof(hdrRight));
    RTEST_CHECK(hdrSuffix[0] == '\0' && std::strcmp(hdrRight, "MONO") == 0);

    // Render settled MONO SEL view: left header is short "PAN" (x = 48..126 empty), no OSC 1-4 row
    // (y = 191..202 empty) or 12pt value row (y = 219..232 empty), centered "OUTPUT ROUTING SET TO
    // MONO" caption at y = 206..217, and centered converged mono soundstage curve!
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
    renderer.renderFullScreenStrips(clean16.data());
    RTEST_CHECK(maxBoxG6(clean16, 48u, 126u, 6u, 21u) == 0u);
    RTEST_CHECK(maxBoxG6(clean16, 0u, 239u, 191u, 202u) == 0u);
    RTEST_CHECK(maxBoxG6(clean16, 0u, 239u, 219u, 232u) == 0u);
    RTEST_CHECK(maxBoxG6(clean16, 0u, 22u, 190u, 235u) == 0u);
    RTEST_CHECK(maxBoxG6(clean16, 217u, 239u, 190u, 235u) == 0u);
    RTEST_CHECK(maxBoxG6(clean16, 26u, 214u, 206u, 217u) >= 35u);
    RTEST_CHECK(sumBoxG6(clean16, 26u, 214u, 206u, 217u) > 6000u);

    const uint32_t monoLeftHalf = sumBoxG6(clean16, 10u, 96u, 50u, 180u);
    const uint32_t monoRightHalf = sumBoxG6(clean16, 143u, 229u, 50u, 180u);
    const uint32_t monoCenterPeak = sumBoxG6(clean16, 98u, 142u, 50u, 180u);
    const int32_t monoHalfDiff =
        static_cast<int32_t>(monoLeftHalf) - static_cast<int32_t>(monoRightHalf);
    RTEST_CHECK(std::abs(monoHalfDiff) <= 16);
    RTEST_CHECK(monoCenterPeak > monoLeftHalf * 3u);

    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
        renderer.renderStrip(y, 8u, amp8.data() + static_cast<size_t>(y) * 240u);
    }
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
        renderer.renderStrip(y, 24u, amp24.data() + static_cast<size_t>(y) * 240u);
    }
    bool monoPanStripsEq = true;
    for (size_t i = 0u; i < clean16.size(); ++i) {
        if (clean16[i] != amp8[i] || clean16[i] != amp24[i]) {
            monoPanStripsEq = false;
        }
    }
    RTEST_CHECK(monoPanStripsEq);

    // Turning KNOB 1-4 in MONO acts strictly locked: zero pan change, zero focus, and 100%
    // pixel-identical frame (never lights OSC 1-4 or flashes OUTPUT ROUTING)!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 120);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 64);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -80);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -120);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning &&
                ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].pan, savedStereoPan[v]));
    }
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), driveTel);
    renderer.renderFullScreenStrips(amp16.data());
    bool lockedTurnNoPixelDiff = true;
    for (size_t i = 0u; i < clean16.size(); ++i) {
        if (clean16[i] != amp16[i]) {
            lockedTurnNoPixelDiff = false;
        }
    }
    RTEST_CHECK(lockedTurnNoPixelDiff);

    // Verify EDIT Page 2 (PAN) in MONO formats columns as disabled "LOCKED"
    {
        ruido::HudState panHud = ctrl.hud();
        panHud.knobBank = ruido::KnobBankMode::Panning;
        char roleBuf[12]{};
        char valBuf[16]{};
        ruido::GlobalColumnSummary colGs{};
        ruido::formatColumnRoleAndValue(ctrl.state(), panHud, zeroLfo, 0x0Fu, 3u, roleBuf,
                                        sizeof(roleBuf), valBuf, sizeof(valBuf), colGs);
        RTEST_CHECK(colGs.disabled && std::strcmp(roleBuf, "OSC 4") == 0 &&
                    std::strcmp(valBuf, "LOCKED") == 0);
    }

    // Turning SELECT CCW on EDIT Page 2 (PAN) switches MONO -> STEREO ("PAN · STEREO") with
    // original stereo pan positions intact, and 3rd EDIT tap dissolves in 150 ms!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(!ctrl.state().global.monoCollapse && ctrl.hud().mode == ruido::HudMode::Panning);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].pan, savedStereoPan[v]));
    }
    ctrl.tick(0.025f, driveTel);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().panPageOpen && !ctrl.hud().editPageOpen);
    ctrl.tick(0.075f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 0.15f));

    // (e) No redundant/confusing "ALL" suffix on any 4-oscillator bottom-ledge screen
    //     (FREQUENCY, DUTY, DRIFT, LEVEL, PAN): unfocused header stops cleanly after leftMain,
    //     while focusing a knob appends " OSC 1..4".
    struct PerOscScreenCase {
        ruido::HudMode mode;
        ruido::KnobBankMode bank;
        const char* title;
    };
    const std::array<PerOscScreenCase, 5> perOscScreens{{
        {ruido::HudMode::VoiceFrequency, ruido::KnobBankMode::Frequency, "FREQUENCY"},
        {ruido::HudMode::PulseWidth, ruido::KnobBankMode::PulseWidth, "DUTY"},
        {ruido::HudMode::VoiceDrift, ruido::KnobBankMode::VoiceDrift, "DRIFT"},
        {ruido::HudMode::Amplitude, ruido::KnobBankMode::Amplitude, "LEVEL"},
        {ruido::HudMode::Panning, ruido::KnobBankMode::Panning, "PAN"},
    }};
    quietSt.global.monoCollapse = false;
    for (const auto& sc : perOscScreens) {
        renderer.reset();
        ruido::HudState testHud{};
        testHud.mode = sc.mode;
        testHud.knobBank = sc.bank;
        testHud.focusedKnob = ruido::kNoFocusedKnob;
        testHud.panPageOpen = (sc.mode == ruido::HudMode::Panning);
        testHud.freqPageOpen = (sc.mode == ruido::HudMode::VoiceFrequency);
        testHud.editPageOpen =
            (sc.mode != ruido::HudMode::Panning && sc.mode != ruido::HudMode::VoiceFrequency);
        testHud.opacity = 1.0f;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, quietSt, testHud, driveTel);
        }
        renderer.renderFullScreenStrips(amp16.data());
        const auto suffixStart =
            static_cast<size_t>(10 + ruido::measureSansText(ruido::kFontSansBold, sc.title, 0) + 3);
        RTEST_CHECK(maxBoxG6(amp16, suffixStart, suffixStart + 36u, 6u, 21u) == 0u);

        testHud.focusedKnob = 0u;
        renderer.prepareFrame(0.025f, quietSt, testHud, driveTel);
        renderer.renderFullScreenStrips(amp16.data());
        RTEST_CHECK(maxBoxG6(amp16, suffixStart, suffixStart + 36u, 6u, 21u) >= 20u);
    }

    // (f) Single pure contour per oscillator on FX (`HudMode::VoiceFilter`): zero fake secondary
    //     echo ridges below the passband spine (monotonic vertical skirt decay on same-phase 4x4
    //     Bayer rows above and below the y = 120 background scope baseline).
    {
        renderer.reset();
        ruido::EngineTelemetry silentTel{};
        ruido::EngineControlState fltSt{};
        fltSt.global.masterRun = false;
        fltSt.global.masterTrim = 0.0f;
        fltSt.voices[0].hpCutoffHz = 110.0f;
        fltSt.voices[0].hpResonance = 0.78f;
        fltSt.voices[0].lpCutoffHz = 2600.0f;
        fltSt.voices[0].lpResonance = 0.78f;
        ruido::HudState fltHud{};
        fltHud.mode = ruido::HudMode::VoiceFilter;
        fltHud.knobBank = ruido::KnobBankMode::VoiceFilter;
        fltHud.fxVoiceFocus = 0u;
        fltHud.fxPageOpen = true;
        fltHud.opacity = 1.0f;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, fltSt, fltHud, silentTel);
        }
        renderer.renderFullScreenStrips(amp16.data());
        const auto pxG6 = [&](size_t y, size_t x) -> uint16_t {
            return (amp16[y * 240u + x] >> 5u) & 0x3Fu;
        };
        uint16_t maxPassbandSpineG6 = 0u;
        for (size_t y = 74u; y <= 87u; ++y) {
            const uint16_t g = pxG6(y, 120u);
            if (g > maxPassbandSpineG6) {
                maxPassbandSpineG6 = g;
            }
        }
        RTEST_CHECK(maxPassbandSpineG6 >= 32u); // Crisp single Reinhard-tonemapped passband spine
        bool monotonicSkirt = true;
        for (size_t y = 90u; y <= 110u; ++y) {
            if (pxG6(y + 4u, 120u) > pxG6(y, 120u)) {
                monotonicSkirt = false;
            }
        }
        for (size_t y = 128u; y <= 176u; ++y) {
            if (pxG6(y + 4u, 120u) > pxG6(y, 120u)) {
                monotonicSkirt = false;
            }
        }
        RTEST_CHECK(monotonicSkirt);
    }

    // (g) LEVEL screen (`HudAmpGraph::compositeLevelStrip`): 3-px dot-matched stem width around
    //     Column 0 center and monotonic level-brightness scaling across `3% -> 35% -> 68% -> 100%`,
    //     with clearly visible brightness even at the lowest audible level.
    {
        ruido::EngineTelemetry silentTel{};
        ruido::HudState lvlHud{};
        lvlHud.mode = ruido::HudMode::Amplitude;
        lvlHud.knobBank = ruido::KnobBankMode::Amplitude;
        lvlHud.editTarget = ruido::EditTarget::Level;
        lvlHud.editPageOpen = true;
        lvlHud.opacity = 1.0f;

        const auto lvlBaseY = static_cast<size_t>(ruido::HudAmpGraph::kColYBot);
        const size_t stemNearBaseY = lvlBaseY - 2u;
        const size_t stemMidY = (lvlBaseY - 44u) & ~size_t{3u};

        const std::array<float, 4> testLevels{0.03f, 0.35f, 0.68f, 1.00f};
        std::array<uint32_t, 4> stemBodySums{};
        for (size_t li = 0u; li < testLevels.size(); ++li) {
            renderer.reset();
            ruido::EngineControlState lvlSt{};
            lvlSt.global.masterRun = false;
            lvlSt.global.masterTrim = 0.0f;
            lvlSt.voices[0].amplitude = testLevels[li];
            for (int f = 0; f < 8; ++f) {
                renderer.prepareFrame(0.025f, lvlSt, lvlHud, silentTel);
            }
            renderer.renderFullScreenStrips(amp16.data());
            const auto pxG6 = [&](size_t y, size_t x) -> uint16_t {
                return (amp16[y * 240u + x] >> 5u) & 0x3Fu;
            };
            if (li == 0u) {
                uint16_t lowMaxG6 = 0u;
                for (size_t y = lvlBaseY - 8u; y <= lvlBaseY + 1u; ++y) {
                    for (size_t x = col0X - 2u; x <= col0X + 2u; ++x) {
                        const uint16_t g = pxG6(y, x);
                        if (g > lowMaxG6) {
                            lowMaxG6 = g;
                        }
                    }
                }
                RTEST_CHECK(lowMaxG6 >= 28u);
                RTEST_CHECK(pxG6(stemNearBaseY, col0X - 1u) >= 16u &&
                            pxG6(stemNearBaseY, col0X) >= 26u &&
                            pxG6(stemNearBaseY, col0X + 1u) >= 16u);
                RTEST_CHECK(pxG6(stemNearBaseY, col0X - 5u) == 0u &&
                            pxG6(stemNearBaseY, col0X + 5u) == 0u);
            }
            if (li == 2u) {
                RTEST_CHECK(pxG6(stemMidY, col0X - 1u) >= 34u && pxG6(stemMidY, col0X) >= 46u &&
                            pxG6(stemMidY, col0X + 1u) >= 34u);
                RTEST_CHECK(pxG6(stemMidY, col0X - 4u) == 0u && pxG6(stemMidY, col0X + 4u) == 0u);
            }
            uint32_t bodySum = 0u;
            for (size_t y = lvlBaseY - 3u; y <= lvlBaseY - 1u; ++y) {
                for (size_t x = col0X - 1u; x <= col0X + 1u; ++x) {
                    bodySum += pxG6(y, x);
                }
            }
            stemBodySums[li] = bodySum;
        }
        RTEST_CHECK(stemBodySums[0] < stemBodySums[1] && stemBodySums[1] < stemBodySums[2] &&
                    stemBodySums[2] < stemBodySums[3]);
    }

    // (h) Shared `kGraphBaselineY` edge-to-edge baseline (`drawGraphBaseline`) bit-exact opacity &
    //     position consistency across FILTER (`FX`), PAN (`SEL` STEREO & MONO), and LEVEL (`EDIT`):
    {
        ruido::EngineTelemetry silentTel{};
        ruido::EngineControlState baseSt = ruido::EngineControlState::makeDefault();
        baseSt.global.masterRun = false;
        baseSt.global.masterTrim = 0.0f;

        // 1. FILTER (`HudMode::VoiceFilter`, open passband)
        renderer.reset();
        ruido::HudState fltH{};
        fltH.mode = ruido::HudMode::VoiceFilter;
        fltH.knobBank = ruido::KnobBankMode::VoiceFilter;
        fltH.fxPageOpen = true;
        fltH.opacity = 1.0f;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, baseSt, fltH, silentTel);
        }
        renderer.renderFullScreenStrips(amp8.data());

        // 2. PAN (`HudMode::Panning`, default STEREO spread)
        renderer.reset();
        ruido::HudState panH{};
        panH.mode = ruido::HudMode::Panning;
        panH.knobBank = ruido::KnobBankMode::Panning;
        panH.panPageOpen = true;
        panH.opacity = 1.0f;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, baseSt, panH, silentTel);
        }
        renderer.renderFullScreenStrips(amp16.data());

        // 3. PAN in MONO (`monoCollapse = true`)
        renderer.reset();
        ruido::EngineControlState monoSt = baseSt;
        monoSt.global.monoCollapse = true;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, monoSt, panH, silentTel);
        }
        renderer.renderFullScreenStrips(clean16.data());

        // 4. LEVEL (`HudMode::Amplitude`)
        renderer.reset();
        ruido::HudState lvlH{};
        lvlH.mode = ruido::HudMode::Amplitude;
        lvlH.knobBank = ruido::KnobBankMode::Amplitude;
        lvlH.editTarget = ruido::EditTarget::Level;
        lvlH.editPageOpen = true;
        lvlH.opacity = 1.0f;
        for (int f = 0; f < 8; ++f) {
            renderer.prepareFrame(0.025f, baseSt, lvlH, silentTel);
        }
        renderer.renderFullScreenStrips(amp24.data());

        const auto rowSumG6 = [](const std::array<uint16_t, 240u * 240u>& img, size_t y, size_t x0,
                                 size_t x1) {
            uint32_t s = 0u;
            for (size_t x = x0; x <= x1; ++x) {
                s += (img[y * 240u + x] >> 5u) & 0x3Fu;
            }
            return s;
        };

        // Full 240-px edge-to-edge baseline at kGraphBaselineY with zero overhang 3 px below
        RTEST_CHECK(rowSumG6(amp8, baseY, 0u, 3u) >= 40u);
        RTEST_CHECK(rowSumG6(amp8, baseY, 236u, 239u) >= 40u);
        RTEST_CHECK(rowSumG6(amp8, baseY + 3u, 0u, 239u) == 0u);

        // Every single pixel x = 0..239 across rows [baseY .. baseY + 3] has 100% bit-exact
        // identical baseline opacity across FILTER, PAN (STEREO), PAN (MONO), and LEVEL!
        bool allBaselinesBitExact = true;
        for (size_t y = baseY; y <= baseY + 3u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                const size_t idx = y * 240u + x;
                if (amp16[idx] != amp8[idx] || clean16[idx] != amp8[idx] ||
                    amp24[idx] != amp8[idx]) {
                    allBaselinesBitExact = false;
                }
            }
        }
        RTEST_CHECK(allBaselinesBitExact);
    }

    // 10l. HOME (`FREQUENCY`) Per-Oscillator `[FX]  [LFO A] [LFO B]` Monochrome Status Row &
    //      1-to-1 Hardware `FX` / `LFO` Button LED Sync:
    {
        ctrl.reset();
        renderer.reset();
        // Silence audio trace so background behind bottom HUD strip is pure pitch-black (#000000)
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
        ruido::EngineTelemetry modTel{};
        for (int f = 0; f < 30; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        // Dismiss the latched AUDIO OUTPUT · OFF banner, let 150 ms dissolve finish, then latch
        // HOME (`FREQUENCY`) HUD open and settle its 150 ms S-curve fade-in
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(ctrl.hud().freqPageOpen);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }

        renderer.renderFullScreenStrips(amp8.data());
        ctrl.tick(0.05f, modTel);
        renderer.prepareFrame(0.05f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(amp16.data());

        using FrameImg = std::array<uint16_t, 240u * 240u>;
        const auto pxAt = [](const FrameImg& img, int x, int y) -> uint16_t {
            return img[static_cast<size_t>(y) * 240u + static_cast<size_t>(x)];
        };
        const auto g6At = [&](const FrameImg& img, int x, int y) -> uint32_t {
            return (pxAt(img, x, y) >> 5u) & 0x3Fu;
        };
        const auto neighborSum = [&](const FrameImg& img, int cx, int cy) -> uint32_t {
            uint32_t sum = 0u;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    sum += g6At(img, cx + dx, cy + dy);
                }
            }
            return sum;
        };

        // 1. Dry boot patch: FX & LFO buttons rest at 15%, all 12 placeholders are crisp 1x1 dots
        //    at y = 234 (`cx - 8`, `cx + 2`, `cx + 8`) with zero AA fringe and zero idle pulsing
        RTEST_CHECK(std::fabs(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] -
                              0.15f) <= 1.0e-4f);
        RTEST_CHECK(std::fabs(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)] -
                              0.15f) <= 1.0e-4f);
        constexpr int kModY = ruido::kHudHomeModRowY;
        for (int col = 0; col < 4; ++col) {
            const int cx = 30 + col * 60;
            const int slots[3] = {cx - 8, cx + 2, cx + 8};
            for (int sx : slots) {
                const uint32_t g = g6At(amp8, sx, kModY);
                RTEST_CHECK(g >= 14u && g <= 18u);
                RTEST_CHECK(neighborSum(amp8, sx, kModY) == 0u);
            }
        }
        bool dryZeroDiff = true;
        for (int y = 230; y < 240; ++y) {
            for (int x = 0; x < 240; ++x) {
                if (pxAt(amp8, x, y) != pxAt(amp16, x, y)) {
                    dryZeroDiff = false;
                }
            }
        }
        RTEST_CHECK(dryZeroDiff);

        // Non-FREQUENCY pages (`EDIT`) hide the modulator status row cleanly
        ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
        ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) == 0u);
        RTEST_CHECK(g6At(clean16, 32, kModY) == 0u);
        RTEST_CHECK(g6At(clean16, 38, kModY) == 0u);

        // Return to latched HOME (`FREQUENCY`)
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(ctrl.hud().freqPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }

        // 2. Dial OSC 1 LPF below 28 kHz with 0% LFO depth -> FX button LED breathes steadily at
        //    0.25 Hz and ONLY OSC 1's left (`FX`) slot promotes to a tertiary circle (`g6 <= 44`)
        ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
        ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
        ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -32);
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(ctrl.hud().freqPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        RTEST_CHECK(ctrl.state().voices[0].lpCutoffHz < 20000.0f);
        RTEST_CHECK(ctrl.state().voices[0].lfoDepth == 0.0f);

        float minFxLed = 1.0f;
        float maxFxLed = 0.0f;
        uint32_t minFxG6 = 63u;
        uint32_t maxFxG6 = 0u;
        for (int step = 0; step < 40; ++step) {
            ctrl.tick(0.10f, modTel);
            renderer.prepareFrame(0.10f, ctrl.state(), ctrl.hud(), modTel);
            renderer.renderFullScreenStrips(amp24.data());
            const float fxLed = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)];
            RTEST_CHECK(fxLed > 0.15f && fxLed <= 0.85f);
            if (fxLed < minFxLed) {
                minFxLed = fxLed;
            }
            if (fxLed > maxFxLed) {
                maxFxLed = fxLed;
            }
            const uint32_t cG6 = g6At(amp24, 22, kModY);
            if (cG6 < minFxG6) {
                minFxG6 = cG6;
            }
            if (cG6 > maxFxG6) {
                maxFxG6 = cG6;
            }
            // Active tertiary cap: solid multi-pixel circle, strictly capped well below 100% white
            RTEST_CHECK(cG6 >= 24u && cG6 <= 44u);
            RTEST_CHECK(neighborSum(amp24, 22, kModY) >= 40u);
            // OSC 1 LFO A/B and OSC 2-4 all remain crisp 1x1 muted dots
            RTEST_CHECK(neighborSum(amp24, 32, kModY) == 0u);
            RTEST_CHECK(neighborSum(amp24, 38, kModY) == 0u);
            for (int col = 1; col < 4; ++col) {
                const int cx = 30 + col * 60;
                RTEST_CHECK(neighborSum(amp24, cx - 8, kModY) == 0u);
                RTEST_CHECK(neighborSum(amp24, cx + 2, kModY) == 0u);
                RTEST_CHECK(neighborSum(amp24, cx + 8, kModY) == 0u);
            }
        }
        RTEST_CHECK(maxFxLed - minFxLed >= 0.08f);
        RTEST_CHECK(maxFxG6 > minFxG6 + 4u);

        // 3. Enable OSC 2 LFO B (`slot 3`) -> ONLY `(98, 234)` promotes & syncs with
        // `ButtonId::Lfo`
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 12); // slot 0 -> slot 3 (`LFO B OSC 2`)
        ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 80);  // depth > 25%
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(ctrl.hud().freqPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        RTEST_CHECK(ctrl.state().voices[1].lfoDepth == 0.0f);
        RTEST_CHECK(ctrl.state().voices[1].lfoBDepth > 0.25f);

        modTel.lfoOutput[5].store(-1.0f, std::memory_order_relaxed);
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(amp8.data());
        const float lfoLedLo = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

        modTel.lfoOutput[5].store(1.0f, std::memory_order_relaxed);
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(amp16.data());
        const float lfoLedHi = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

        RTEST_CHECK(lfoLedHi > lfoLedLo + 0.12f);
        RTEST_CHECK(g6At(amp16, 98, kModY) > g6At(amp8, 98, kModY) + 6u);
        RTEST_CHECK(g6At(amp16, 98, kModY) <= 44u);
        RTEST_CHECK(neighborSum(amp16, 98, kModY) >= 40u);
        // Sibling `LFO A` on OSC 2 (`92, 234`) stays a crisp 1x1 placeholder dot
        RTEST_CHECK(neighborSum(amp16, 92, kModY) == 0u);

        // 4. Strip-height invariance (`8 == 16 == 24` rows) & strict neutral monochrome (`R5 ==
        // B5`)
        for (uint16_t y = 0u; y < 240u; y += 8u) {
            renderer.renderStrip(y, 8u, amp24.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y += 24u) {
            renderer.renderStrip(y, 24u, clean16.data() + static_cast<size_t>(y) * 240u);
        }
        bool stripsEqual = true;
        bool neutralMono = true;
        for (size_t i = 0u; i < 240u * 240u; ++i) {
            if (amp24[i] != amp16[i] || clean16[i] != amp16[i]) {
                stripsEqual = false;
            }
            const uint16_t p = amp16[i];
            if (((p >> 11u) & 0x1Fu) != (p & 0x1Fu)) {
                neutralMono = false;
            }
        }
        RTEST_CHECK(stripsEqual);
        RTEST_CHECK(neutralMono);
    }
#undef RTEST_CHECK
}

template <typename CheckFn> inline void runKeyboardModelAndLiveMixerTests(CheckFn checkFn) {
    std::printf("[Test 11] LIVE MIXER, UNISON/SPREAD, 4-Slot Snapshots & Circuit Lab Stutter...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    ruido::EngineTelemetry tel{};

    // 1. Persistent SEL guidance screen + SELECT knob model picker + 220 ms signature LED wipe
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    RTEST_CHECK(ctrl.hud().fxPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(!ctrl.hud().fxPageOpen && !ctrl.hud().panPageOpen && !ctrl.hud().editPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);

    // Stays latched open indefinitely (past 6.0 s) until 2nd SEL tap or HOME
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel);
    }
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 1.0f));

    // Turning KNOB 1 while SEL is open still adjusts OSC 1 Frequency, then returns to SEL!
    const float hz0Before = ctrl.state().voices[0].frequencyHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 8);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > hz0Before);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    for (int i = 0; i < 20; ++i) {
        ctrl.tick(0.100f, tel);
    }
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -8);
    for (int i = 0; i < 20; ++i) {
        ctrl.tick(0.100f, tel);
    }

    // Pressing a key while SEL is open jumps live to native KeyboardPlay and returns on release!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 0.70f);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);

    // Step SELECT CW across LiveMixer(0) -> CircuitLab(1) -> RhythmLab(2)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.tick(0.050f, tel); // Mid-220ms wipe boosts key LEDs above rest!
    float maxWipeLed = 0.0f;
    for (float l : ctrl.leds().whiteKeyLeds) {
        maxWipeLed = std::max(maxWipeLed, l);
    }
    RTEST_CHECK(maxWipeLed > 0.55f);

    ctrl.onEncoderDelta(ruido::EncoderId::Select, 200); // Clamps at RhythmLab(2)
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "KEYBOARD") == 0 && std::strcmp(ls, " 3/3") == 0);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // Zero windup -> CircuitLab(1)
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // -> LiveMixer(0)
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);

    // 2nd SEL tap dissolves in 150 ms S-curve fade while keeping chosen model active
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(!ctrl.hud().selPageOpen);
    for (int i = 0; i < 4; ++i) {
        ctrl.tick(0.050f, tel);
    }
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::None);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 0.15f));

    // 2. Mode 1 (LIVE MIXER) Punch-Ins, MONO/POLY Unison/Spread & Non-Destructive Restoration
    const ruido::EngineControlState baseline = ctrl.state();
    const auto verifyRestored = [&]() {
        RTEST_CHECK(!ctrl.hud().scopeFrozen);
        RTEST_CHECK(!ctrl.state().keyMod.stasisFreeze);
        RTEST_CHECK(nearF(ctrl.state().keyMod.overloadSurge, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.dubLpfDrop, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.ringModMix, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.hardSyncAmount, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.lfoTurbo, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.filterScream, 0.0f));
        RTEST_CHECK(ctrl.state().global.circuitCoupling == baseline.global.circuitCoupling);
        RTEST_CHECK(ctrl.state().global.monoCollapse == baseline.global.monoCollapse);
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            RTEST_CHECK(ctrl.state().voices[v].frequencyHz == baseline.voices[v].frequencyHz);
            RTEST_CHECK(ctrl.state().voices[v].amplitude == baseline.voices[v].amplitude);
            RTEST_CHECK(ctrl.state().voices[v].pan == baseline.voices[v].pan);
            RTEST_CHECK(ctrl.state().voices[v].lpCutoffHz == baseline.voices[v].lpCutoffHz);
            RTEST_CHECK(ctrl.state().voices[v].hpCutoffHz == baseline.voices[v].hpCutoffHz);
            RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 1.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[v], 1.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.voicePwDelta[v], 0.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.stutterDepth[v], 0.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.xmodDepth[v], 0.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.foldDepth[v], 0.0f));
            RTEST_CHECK(nearF(ctrl.state().keyMod.crushDepth[v], 0.0f));
            RTEST_CHECK(nearF(ctrl.state().voicePressure[v], 0.0f));
        }
    };

    // OP1..OP4 Immediate Console Solo -> OP_v = 100% (MUTE_v = 0%), peers MUTE = 100% (OP = 0%),
    // with zero extraneous highlights on Keys 5-16 or FX/HOME!
    for (uint8_t v = 0u; v < 4u; ++v) {
        const auto key = static_cast<ruido::BlackKeyId>(v);
        const size_t peer = (v + 1u) & 3u;
        ctrl.onBlackKeyPressure(key, 0.20f);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::SpotlightSolo);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(ctrl.state().keyMod.voiceGainMul[v] >= 1.0f);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[peer], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[v], 1.0f));
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[peer], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[v], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[peer], 1.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[4u + v], 0.42f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8u + v], 0.22f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12u + v], 0.08f));
        RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 0.15f));
        ctrl.onBlackKeyEvent(key, false);
        ctrl.tick(0.016f, tel);
        verifyRestored();
    }

    // OP5 Drive -> native Amplitude view + full-device VU bar + red REC clip beacon
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 0.92f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    RTEST_CHECK(nearF(ctrl.state().keyMod.overloadSurge, 0.92f));
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[0] > 0.70f && ctrl.leds().whiteKeyLeds[15] > 0.70f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)] > 0.65f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)], 0.0f));
    verifyRestored();

    // OP6 Sub Octave (continuous 0.0 -> -2.0 Oct dive across [0, 1] + OCT- beacon)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 0.25f);
    const float sub25 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 0.50f);
    const float sub50 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 0.75f);
    const float sub75 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.00f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::SubDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(sub25 < 0.90f && sub25 > sub50);
    RTEST_CHECK(nearF(sub50, 0.50f, 1.0e-3f));
    RTEST_CHECK(sub75 < sub50 && sub75 > 0.28f);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[0], 0.25f, 1.0e-3f));
    RTEST_CHECK(
        nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // PIT Ratio Lock (pulls detuned Osc 2 = 82.41 Hz onto pure 3/2 = 82.50 Hz relative to 55 Hz)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RatioMagnet);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    const float lockedHz1 =
        ctrl.state().voices[1].frequencyHz * ctrl.state().keyMod.voicePitchMul[1];
    RTEST_CHECK(nearF(lockedHz1, 82.50f, 0.05f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);
    verifyRestored();

    // GLO Low Pass -> native 4-voice GlobalFilter MS-20 passband curve + right-to-left curtain LEDs
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 0.85f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.dubLpfDrop, 0.85f));
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[2] > ctrl.leds().whiteKeyLeds[14] + 0.40f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // MONO (UNISON: collapses Osc 2-4 pitches onto Osc 1 ~55 Hz + centers pan, undimmed scope!)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Mono, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::MonoFocus);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "UNISON") == 0);
        RTEST_CHECK(std::strcmp(rt, "100%") == 0 && sb && nearF(bn, 1.0f));
    }
    for (size_t v = 1u; v < 4u; ++v) {
        const float uniHz =
            ctrl.state().voices[v].frequencyHz * ctrl.state().keyMod.voicePitchMul[v];
        RTEST_CHECK(std::fabs(uniHz - ctrl.state().voices[0].frequencyHz) < 0.25f);
    }
    RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, -1.0f));
    RTEST_CHECK(ctrl.state().keyMod.ratioLockAmount > 0.70f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[7] > 0.90f && ctrl.leds().whiteKeyLeds[0] < 0.05f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)] > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    verifyRestored();

    // POLY (DETUNE SPREAD: detuned pitch swarm + Schmitt PWM duty shifts + wide pan)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Poly, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::PolySpread);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "DETUNE SPREAD") == 0);
        RTEST_CHECK(std::strcmp(rt, "100%") == 0 && sb && nearF(bn, 1.0f));
    }
    for (const auto& spec : ruido::kKeyboardModelSpecs) {
        for (const char* tz : spec.topZones) {
            RTEST_CHECK(std::strcmp(tz, "FREEZE") != 0);
        }
    }
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[0] < 0.98f);
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[3] > 1.06f);
    RTEST_CHECK(ctrl.state().keyMod.voicePwDelta[0] < -0.15f);
    RTEST_CHECK(ctrl.state().keyMod.voicePwDelta[3] > 0.20f);
    RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 1.0f));
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[0] > 0.90f && ctrl.leds().whiteKeyLeds[7] < 0.06f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)] > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    verifyRestored();

    // 11th Key Stasis Freeze -> Scope
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::ScopeFreezeKey, 1.0f);
    RTEST_CHECK(ctrl.hud().scopeFrozen && ctrl.state().keyMod.stasisFreeze);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // Bottom Keys on LIVE MIXER: Mute (0..3), Filter (4..7), Octave (8..11)
    for (uint8_t v = 0u; v < 4u; ++v) {
        // MUTE v: immediate full mute on touch -> MUTE_v = 100%, OP_v = 0%, zero FX/HOME side-glow
        ctrl.onWhiteKeyPressure(v, 0.20f);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceCut);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[v], 1.0f));
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[v], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 0.15f));
        ctrl.onWhiteKeyGate(v, false);

        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(4u + v), 0.80f);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceFormant);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
        RTEST_CHECK(ctrl.hud().focusedVoice == v);
        RTEST_CHECK(nearF(ctrl.state().voicePressure[v], 0.80f));
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(4u + v), false);

        // OCTAVE v (Keys 8..11): continuous +0.0 -> +2.0 Oct siren climb (+1 Oct at 0.50, +2 at 1)
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(8u + v), 0.50f);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceSiren);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[v], 2.0f, 1.0e-3f));
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(8u + v), 1.00f);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[v], 4.0f, 1.0e-3f));
        RTEST_CHECK(ctrl.state().voicePressure[v] > 0.65f);
        RTEST_CHECK(
            nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 1.0f));
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(8u + v), false);
        ctrl.tick(0.016f, tel);

        verifyRestored();
    }

    // 3. Strip bit-exactness (240x8 == 240x16 == 240x24), monochrome neutrality & SDF geometry
    static std::array<uint16_t, 240u * 240u> kb16{};
    static std::array<uint16_t, 240u * 240u> kb8{};
    static std::array<uint16_t, 240u * 240u> kb24{};
    const auto g6Px = [&](size_t x, size_t y) -> uint16_t {
        return (kb16[y * 240u + x] >> 5u) & 0x3Fu;
    };
    ruido::EngineControlState quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;

    const auto checkStripAndMono = [&]() {
        renderer.renderFullScreenStrips(kb16.data());
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
            renderer.renderStrip(y, 8u, kb8.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
            renderer.renderStrip(y, 24u, kb24.data() + static_cast<size_t>(y) * 240u);
        }
        bool eq = true;
        bool mono = true;
        size_t litPx = 0u;
        for (size_t y = 0u; y < 240u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                const size_t i = y * 240u + x;
                const uint16_t px = kb16[i];
                if (px != kb8[i] || px != kb24[i]) {
                    eq = false;
                }
                const uint16_t r5 = (px >> 11u) & 0x1Fu;
                const uint16_t g6 = (px >> 5u) & 0x3Fu;
                const uint16_t b5 = px & 0x1Fu;
                if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
                    mono = false;
                }
                if (y >= 32u && y <= 232u && y != 120u && g6 >= 10u) {
                    ++litPx;
                }
            }
        }
        RTEST_CHECK(eq);
        RTEST_CHECK(mono);
        RTEST_CHECK(litPx > 400u);
    };

    // Verify all 3 SEL KeyboardModel guidance screens + SDF rounded corners + 7-tier zone slits
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    for (int m = 0; m < 3; ++m) {
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.020f, tel);
            renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
        }
        checkStripAndMono();
        if (m == 0) {
            // Rounded stadium corners: enclosure bbox corner (8, 92) is pure black (0), whereas
            // top flat edge (30, 92) is lit border; OP1 corner (23, 98) is dark bed vs arc (28, 98)
            RTEST_CHECK(g6Px(8u, 92u) == 0u);
            RTEST_CHECK(g6Px(30u, 92u) >= 8u);
            RTEST_CHECK(g6Px(23u, 98u) <= 5u);
            RTEST_CHECK(g6Px(28u, 98u) >= 9u);
            // 4-tier bottom slits step brightest -> faintest left-to-right: MUTE > FILTER > OCTAVE
            // > SNAP
            const uint16_t gMute = g6Px(21u, 145u);
            const uint16_t gFilt = g6Px(74u, 145u);
            const uint16_t gOct = g6Px(127u, 145u);
            const uint16_t gSnap = g6Px(179u, 145u);
            RTEST_CHECK(gMute > gFilt && gFilt > gOct && gOct > gSnap);
        }
        if (m < 2) {
            ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
        }
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -8); // Back to LiveMixer
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel); // t = 0.20 s after Select wake
    }
    checkStripAndMono();

    // Verify whole SEL illustration + instructions hold at 100% for 5.0 s, slowly dim over 5.0 s
    // down to 50% (never 0%!), and wake back to 100% when interacting with SELECT!
    const auto maxInstructG6 = [&]() -> uint16_t {
        uint16_t hi = 0u;
        for (size_t y = 202u; y <= 228u; ++y) {
            for (size_t x = 20u; x <= 220u; ++x) {
                const uint16_t g = g6Px(x, y);
                if (g > hi) {
                    hi = g;
                }
            }
        }
        return hi;
    };
    const uint16_t instructInitG6 = maxInstructG6();
    const uint16_t slitInitG6 = g6Px(21u, 145u);
    RTEST_CHECK(instructInitG6 >= 30u);
    RTEST_CHECK(slitInitG6 >= 45u);

    for (int f = 0; f < 73; ++f) {
        ctrl.tick(0.10f, tel);
        renderer.prepareFrame(0.10f, quietSt, ctrl.hud(), tel); // +7.30 s -> t = 7.50 s (mid-dim)
    }
    checkStripAndMono();
    const uint16_t instructMidG6 = maxInstructG6();
    const uint16_t slitMidG6 = g6Px(21u, 145u);
    RTEST_CHECK(instructMidG6 < instructInitG6 - 4u);
    RTEST_CHECK(slitMidG6 < slitInitG6 - 6u);

    for (int f = 0; f < 30; ++f) {
        ctrl.tick(0.10f, tel);
        renderer.prepareFrame(0.10f, quietSt, ctrl.hud(),
                              tel); // +3.00 s -> t = 10.50 s (50% floor)
    }
    checkStripAndMono();
    const uint16_t instructDimG6 = maxInstructG6();
    const uint16_t slitDimG6 = g6Px(21u, 145u);
    RTEST_CHECK(instructDimG6 >= 14u && instructDimG6 < instructMidG6 - 3u);
    RTEST_CHECK(slitDimG6 >= 22u && slitDimG6 < slitMidG6 - 5u);

    // Turning SELECT (even clamped at 1/3 end stop) wakes whole view back to 100% brightness!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono();
    RTEST_CHECK(maxInstructG6() == instructInitG6);
    RTEST_CHECK(g6Px(21u, 145u) >= slitInitG6 - 1u);

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false); // Dismiss SEL

    // Chording multiple keys routes to most-recent press & peels back LIFO on release!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 0.80f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 0.65f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono();

    ctrl.onWhiteKeyPressure(9u, 0.90f); // OCTAVE 2 on LIVE MIXER
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceSiren);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);
    verifyRestored();

    // 4. Unified VoiceCircuit mute consistency across main-screen HUD columns, SOLO+STUTTER (on
    //    3/3 CIRCUIT LAB), and EDIT LEVEL=0%
    const auto maxHudColG6 = [&](uint8_t colIdx) -> uint16_t {
        const size_t cx = static_cast<size_t>(ruido::hudColumnCenterX(colIdx));
        uint16_t hi = 0u;
        for (size_t y = 191u; y <= 232u; ++y) {
            for (size_t x = cx - 22u; x <= cx + 22u; ++x) {
                const uint16_t g = g6Px(x, y);
                if (g > hi) {
                    hi = g;
                }
            }
        }
        return hi;
    };

    // Switch to 2/4 CIRCUIT LAB
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> CircuitLab (1)
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    for (int f = 0; f < 14; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Verify VoiceCircuit stutterGateMask main-screen column dimming (shared ready for RhythmLab):
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.160f, tel);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    quietSt.keyMod.stutterDepth[1] = 1.0f;
    tel.stutterGateMask.store(static_cast<uint8_t>(~(1u << 1u) & 0x0Fu), std::memory_order_relaxed);
    renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), tel);
    renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), tel);
    renderer.renderFullScreenStrips(kb16.data());
    RTEST_CHECK(maxHudColG6(1u) <= 32u); // Chopped OSC 2 column dimmed cleanly on main screen!
    RTEST_CHECK(maxHudColG6(0u) >= 45u); // Unchopped OSC 1 column stays full brightness!

    tel.stutterGateMask.store(0x0Fu, std::memory_order_relaxed);
    renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), tel);
    renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), tel);
    renderer.renderFullScreenStrips(kb16.data());
    RTEST_CHECK(maxHudColG6(1u) >= 45u); // Open chop restores OSC 2 column immediately!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.020f, tel);
    }
    verifyRestored();

    // Comprehensive 2/4 CIRCUIT LAB (CMOS PATCH BAY) 27-Key Bends, Headers & 42-LED Light Shows:
    // (a) CROSS-PIN 1-4 (Keys 5-8, white 4..7): neighbor pre-mute FM/XOR into Voice v + 42-LED arc
    for (uint8_t v = 0u; v < 4u; ++v) {
        const size_t nbr = (v + 1u) & 3u;
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(4u + v), 0.85f);
        ctrl.tick(0.025f, tel);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceXMod);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.xmodDepth[v], 0.85f));
        RTEST_CHECK(ctrl.state().voicePressure[v] > 0.60f); // Modulates Voice v itself!
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[4u + v], 1.0f));
        RTEST_CHECK(ctrl.leds().whiteKeyLeds[8u + nbr] > 0.35f ||
                    ctrl.leds().whiteKeyLeds[8u + v] > 0.35f);
        RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)] > 0.20f);
        {
            char lm[24]{}, ls[16]{}, rt[32]{};
            bool sb = false;
            float bn = 0.0f;
            ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt),
                                         sb, bn);
            RTEST_CHECK(std::strcmp(lm, "CROSS-PIN") == 0);
            RTEST_CHECK(sb && nearF(bn, 0.85f, 1.0e-3f));
        }
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(4u + v), false);
        ctrl.tick(0.016f, tel);
        verifyRestored();
    }

    // (b) FOLD 1-4 (Keys 9-12, white 8..11): 1.0x -> 6.2x West-Coast wavefolder + standing crests
    ctrl.onWhiteKeyPressure(8u, 1.0f); // FOLD 1 full pressure -> 6.2x
    ctrl.tick(0.025f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceFold);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(nearF(ctrl.state().keyMod.foldDepth[0], 1.0f));
    RTEST_CHECK(ctrl.state().voicePressure[0] > 0.40f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8u], 1.0f));
    RTEST_CHECK(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op6)] > 0.10f ||
                ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Pit)] > 0.10f);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "FOLD") == 0 && std::strcmp(ls, " OSC 1") == 0);
        RTEST_CHECK(std::strcmp(rt, "6.2x") == 0 && sb && nearF(bn, 1.0f));
    }
    ctrl.onWhiteKeyGate(8u, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (c) CRUSH 1-4 (Keys 13-16, white 12..15): CD4040 sub-XOR + CD4013 S&H clock + 16-bit LED bus
    ctrl.onWhiteKeyPressure(12u, 1.0f); // CRUSH 1 full pressure -> ~165 Hz S&H clock
    ctrl.tick(0.045f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceCrush);
    RTEST_CHECK(nearF(ctrl.state().keyMod.crushDepth[0], 1.0f));
    RTEST_CHECK(ctrl.state().voicePressure[0] > 0.65f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12u], 1.0f));
    {
        // Verify binary ripple divider lights crisp hi/lo contrast across OP5..POLY
        bool anyHi = false;
        for (size_t b = 4u; b < 10u; ++b) {
            if (ctrl.leds().blackKeyLeds[b] > 0.80f) {
                anyHi = true;
            }
        }
        RTEST_CHECK(anyHi);
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "CRUSH") == 0 && std::strcmp(ls, " OSC 1") == 0);
        RTEST_CHECK(std::strcmp(rt, "165 Hz") == 0 && sb && nearF(bn, 1.0f));
    }
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (c) OP5 STARVE: rail overload/coupling surge + pitch sag + PWM squeeze + brownout dim
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 0.90f);
    ctrl.tick(0.025f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RailStarve);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(nearF(ctrl.state().keyMod.overloadSurge, 0.90f));
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[0] < 0.56f);
    RTEST_CHECK(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op5)] > 0.85f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[15u] < 0.25f); // Brownout dims passive pads!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (d) OP6 RING MOD: balanced 4-quadrant / CMOS XOR mix + checkerboard alternating keys
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RingXor);
    RTEST_CHECK(nearF(ctrl.state().keyMod.ringModMix, 1.0f));
    RTEST_CHECK(std::fabs(ctrl.leds().whiteKeyLeds[4u] - ctrl.leds().whiteKeyLeds[5u]) > 0.50f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (e) PIT TURBO LFO: 1x -> 32x LFO overclock ("32.0x") + LFO button strobe
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::LfoTurbo);
    RTEST_CHECK(nearF(ctrl.state().keyMod.lfoTurbo, 1.0f));
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "TURBO LFO") == 0);
        RTEST_CHECK(std::strcmp(rt, "32.0x") == 0 && sb && nearF(bn, 1.0f));
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (f) GLO SCREAM: opens GlobalFilter MS-20 passband curve + twin Korg35 peaks + red REC flare!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::FilterScream);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.filterScream, 1.0f));
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.85f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)] > 0.80f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[6u] > 0.70f && ctrl.leds().whiteKeyLeds[9u] > 0.70f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[4u] < 0.25f);
    {
        const ruido::ModulatedFilterParams vf = ruido::computeLiveVoiceFilter(
            ctrl.state().voices[0], 0.0f, 0.0f, 0.0f, 0.0f,
            ruido::FilterMacroContext::fromKeyMod(ctrl.state().keyMod));
        RTEST_CHECK(vf.hpCutoffHz > 380.0f && vf.lpCutoffHz < 950.0f);
        RTEST_CHECK(vf.hpResonance > 0.90f && vf.lpResonance > 0.90f);
    }
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 6; ++f) {
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (g) MONO HARD SYNC: OSC 1 master reset + left-column console anchor (FX + OCT-)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Mono, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::HardSync);
    RTEST_CHECK(nearF(ctrl.state().keyMod.hardSyncAmount, 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[4u], 1.0f));
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.90f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)] > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (h) POLY SCATTER: S&H stepped chaos across pitch + PWM + stereo pan
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Poly, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::PhaseScatter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 1.0f));
    RTEST_CHECK(std::fabs(ctrl.state().keyMod.voicePitchMul[0] - 1.0f) > 0.01f ||
                std::fabs(ctrl.state().keyMod.voicePitchMul[1] - 1.0f) > 0.01f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (i) Universal 11th Key FREEZE on CIRCUIT LAB freezes scope + LFO stasis cleanly
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::ScopeFreezeKey, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().scopeFrozen && ctrl.state().keyMod.stasisFreeze);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // Return to 1/3 LIVE MIXER
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    for (int f = 0; f < 14; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Turning Osc 1 LEVEL down to 0% in EDIT lights MUTE 1 = 100% and turns SOLO 1 = 0% dark, and
    // pressing SOLO 1 while Osc 1 is at 0% never lights SOLO 1!
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -400);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().voices[0].amplitude <= 1.0e-4f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.0f));
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.80f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);

    // 5. Latching MUTE MODE & SOLO MODE via GLOBAL CONFIG across LEDs, DSP & HUD:
    ctrl.reset();
    renderer.reset();
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);    // Row 1: MUTE MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // MOMENTARY -> LATCHING
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false); // Close GLOBAL CONFIG
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Tap & release MUTE 1 (white key 0) -> Osc 1 latches muted after release!
    ctrl.onWhiteKeyGate(0u, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 1.0f)); // Header shows "MUTED"
    ctrl.onWhiteKeyGate(0u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.0f));

    // Momentary SOLO 1 (OP1) temporarily overrides latched mute while held, then returns on
    // release!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.60f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().keyMod.voiceGainMul[0] >= 1.0f);
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 0.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.0f));

    // Second tap on MUTE 1 toggles Osc 1 back ON ("UNMUTED" while held, 64% rest on release)
    ctrl.onWhiteKeyGate(0u, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 0.0f)); // Header shows "UNMUTED"
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 1.0f));
    ctrl.onWhiteKeyGate(0u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 0.64f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.64f));

    // Enable SOLO MODE = LATCHING alongside MUTE MODE = LATCHING
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);    // Row 2: SOLO MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // MOMENTARY -> LATCHING
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);

    // Tap & release OP2 -> latches SOLO 2 on (OP2 = 100%, MUTE 2 = 0%, peers MUTE = 100%, OP = 0%)
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 1.0f)); // Header shows "SOLOED"
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().keyMod.soloMask == (1u << 1u));
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[1], 1.0f));
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[1], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[1], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.0f));

    // Open HOME (latched FREQUENCY HUD) -> main screen reflects latched SOLO 2 (OSC 2 bright, peers
    // dimmed) with zero keys held!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    renderer.renderFullScreenStrips(kb16.data());
    RTEST_CHECK(maxHudColG6(1u) >= 45u);
    RTEST_CHECK(maxHudColG6(0u) <= 32u);

    // Tap & release OP3 -> duet solo (OSC 2 + OSC 3); then tap MUTE 2 -> clears solo on OSC 2 &
    // latches OSC 2 muted while OSC 3 stays soloed!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().keyMod.soloMask == ((1u << 1u) | (1u << 2u)));
    ctrl.onWhiteKeyGate(1u, true);
    ctrl.onWhiteKeyGate(1u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().keyMod.soloMask == (1u << 2u));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[2], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[1], 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[1], 0.0f));

    // Switching SOLO MODE & MUTE MODE back to MOMENTARY in GLOBAL CONFIG clears all latches cleanly
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // Row 2 SOLO MODE -> MOMENTARY
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);    // Row 1 MUTE MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // Row 1 MUTE MODE -> MOMENTARY
    ctrl.tick(0.016f, tel);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 1.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[v], 0.64f));
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[v], 0.64f));
    }

    // 6. 4-Slot 128-Byte SnapshotBank: CRC-32, SAVE + Keys 13-16, LATCHING/MOMENTARY & PRESETS:
    static_assert(sizeof(ruido::PatchSnapshot) == 128u, "PatchSnapshot must be 128 bytes");
    ctrl.reset();
    renderer.reset();

    // (a) Binary quantization + CRC-32 tamper verification
    for (size_t s = 0u; s < ruido::kNumSnapshots; ++s) {
        RTEST_CHECK(ruido::verifyPatchSnapshotCrc(ctrl.snapshots().slotBinary(s)));
    }
    ruido::PatchSnapshot tampered = ctrl.snapshots().slotBinary(1u);
    tampered.voices[0].freqLogQ16 ^= 0x00FFu;
    RTEST_CHECK(!ruido::verifyPatchSnapshotCrc(tampered));

    // (b) Custom patch -> SAVE arms Keys 13-16 -> tap Key 14 (SNAP 2, white 13) stores & swallows
    //     release -> manual knob turn marks EDITED -> recall glides back to stored patch!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 48); // Raise Osc 1 well above 55 Hz
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 40);
    const float customHz0 = ctrl.state().voices[0].frequencyHz;
    const float customCoupling = ctrl.state().global.circuitCoupling;
    RTEST_CHECK(customHz0 > 90.0f);
    RTEST_CHECK(ctrl.hud().snapshotDirty);

    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Snapshot);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "STORE SCENE") == 0);
        RTEST_CHECK(std::strcmp(rt, "TAP SCENE 1-4") == 0);
        RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[0].bottomZones[3], "SCENE") == 0);
    }
    ctrl.tick(0.050f, tel);
    // Armed SAVE pulses SCENE 1-4 (white keys 12..15) unmistakably above 25%!
    for (size_t s = 0u; s < 4u; ++s) {
        RTEST_CHECK(ctrl.leds().whiteKeyLeds[12u + s] >= 0.25f);
    }

    ctrl.onWhiteKeyPressure(13u, 0.85f); // Store into SCENE 2 (Key 14)
    RTEST_CHECK(!ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(!ctrl.hud().snapshotDirty);
    RTEST_CHECK(ctrl.hud().snapshotSavedToast);
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    {
        char lm[24]{}, ls[16]{}, rt[32]{}, rl[12]{}, vs[16]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::GlobalColumnSummary gs{};
        ruido::LfoSignalArray zeroLfo{};
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, " 2") == 0 &&
                    std::strcmp(rt, "SAVED") == 0);
        ruido::formatColumnRoleAndValue(ctrl.state(), ctrl.hud(), zeroLfo, 0x0Fu, 1u, rl,
                                        sizeof(rl), vs, sizeof(vs), gs);
        RTEST_CHECK(std::strcmp(rl, "SCENE 2") == 0);
    }
    // Releasing Key 14 after saving is swallowed cleanly (never re-triggers recall!)
    ctrl.onWhiteKeyGate(13u, false);
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, customHz0, 0.05f));

    // Render Snapshot HUD & verify 240x8 == 240x16 == 240x24 bit-exact monochrome strips
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono();

    // Tweak Knob 1 away -> marks EDITED; tapping Key 14 (SNAP 2, LATCHING default) glides back!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -60);
    RTEST_CHECK(ctrl.hud().snapshotDirty);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz < 70.0f);
    ctrl.onWhiteKeyPressure(13u, 1.0f); // Firm press (~80ms fast glide)
    ctrl.onWhiteKeyGate(13u, false);
    RTEST_CHECK(ctrl.hud().snapshotGliding);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(!ctrl.hud().snapshotDirty);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, customHz0, 0.08f));
    RTEST_CHECK(nearF(ctrl.state().global.circuitCoupling, customCoupling, 1.0e-3f));

    // (c) Multi-key LATCHING pressure crossfade: hold SNAP 1 (Key 13) + SNAP 3 (Key 15) -> parks at
    //     sculpted hybrid patch on release!
    ctrl.onWhiteKeyPressure(12u, 0.50f); // SNAP 1 (55 Hz)
    ctrl.onWhiteKeyPressure(14u, 0.50f); // SNAP 3 (27.5 Hz)
    for (int f = 0; f < 30; ++f) {
        ctrl.tick(0.025f, tel);
    }
    const float hybridHz = ctrl.state().voices[0].frequencyHz;
    // Equal log-octave midpoint between 55 Hz and 27.5 Hz is 55 / sqrt(2) ~= 38.89 Hz!
    RTEST_CHECK(std::fabs(hybridHz - 38.89f) < 0.50f);
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.onWhiteKeyGate(14u, false);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.025f, tel);
    }
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, hybridHz, 0.05f));
    RTEST_CHECK(ctrl.hud().snapshotDirty); // Sculpted hybrid is marked EDITED

    // (d) SNAP MODE = MOMENTARY: non-destructive pressure morph & bit-exact release home!
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);    // Row 3: SNAP MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // LATCHING -> MOMENTARY
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    const ruido::EngineControlState momHome = ctrl.state();

    ctrl.onWhiteKeyPressure(15u, 1.0f); // Hold SNAP 4 at 100% pressure
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(std::fabs(ctrl.state().global.circuitCoupling - 0.68f) < 0.03f);
    ctrl.onWhiteKeyGate(15u, false); // Release -> melts back to bit-exact momHome!
    for (int f = 0; f < 50; ++f) {
        ctrl.tick(0.025f, tel);
    }
    RTEST_CHECK(ctrl.state().global.circuitCoupling == momHome.global.circuitCoupling);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(ctrl.state().voices[v].frequencyHz == momHome.voices[v].frequencyHz);
        RTEST_CHECK(ctrl.state().voices[v].pulseWidth == momHome.voices[v].pulseWidth);
        RTEST_CHECK(ctrl.state().voices[v].lpCutoffHz == momHome.voices[v].lpCutoffHz);
    }

    // (e) PRESETS encoder steps SNAP 1..4 even while in CIRCUIT LAB / RHYTHM LAB + OCT-/OCT+ rates
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> CircuitLab
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);  // Coarse = 0.10 s fast glide
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, -16); // Clamp -> SNAP 1 (slot 0)
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, 55.0f, 0.05f));
#undef RTEST_CHECK
}

template <typename CheckFn> inline void runRhythmLabAndMidiClockTests(CheckFn checkFn) {
    std::printf(
        "[Test 12] 3/3 RHYTHM LAB (Stutter/Divide/Euclid/Pluck/Swing) & 24-PPQN MIDI Clock...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::LockFreeControlBus bus;
    ruido::RuidoController ctrl;
    ctrl.attachControlBus(&bus);
    static ruido::ScopeHudRenderer renderer;
    renderer.reset();

    static std::array<uint16_t, 240u * 240u> s8{};
    static std::array<uint16_t, 240u * 240u> s16{};
    static std::array<uint16_t, 240u * 240u> s24{};
    const auto checkStripAndMono = [&]() {
        renderer.renderFullScreenStrips(s16.data());
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
            renderer.renderStrip(y, 8u, s8.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
            renderer.renderStrip(y, 24u, s24.data() + static_cast<size_t>(y) * 240u);
        }
        bool eq = true;
        bool mono = true;
        for (size_t i = 0u; i < 240u * 240u; ++i) {
            const uint16_t px = s16[i];
            if (px != s8[i] || px != s24[i]) {
                eq = false;
            }
            const uint16_t r5 = (px >> 11u) & 0x1Fu;
            const uint16_t g6 = (px >> 5u) & 0x3Fu;
            const uint16_t b5 = px & 0x1Fu;
            if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
                mono = false;
            }
        }
        RTEST_CHECK(eq);
        RTEST_CHECK(mono);
    };

    ruido::EngineTelemetry tel;
    tel.stutterGateMask.store(0x05u, std::memory_order_relaxed);
    tel.rhythmPhase.store(0.25f, std::memory_order_relaxed);

    // 1. Select 3/3 RHYTHM LAB via SEL + SELECT (+8) and verify SEL map labels & strip parity
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].title, "RHYTHM LAB") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].topZones[1], "PUMP") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].bottomZones[2], "DIVIDE") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].bottomZones[3], "EUCLID") == 0);
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();

    // Turning ALGORITHM while SEL 3/3 RHYTHM LAB is open adjusts Master Tempo logarithmically
    // (+48 ticks = +1.0 octave -> 240.0 BPM), keeps SEL keybed map active underneath, and shows
    // MasterTempo header!
    const float couplingBeforeRhythmAlg = ctrl.state().global.circuitCoupling;
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 48); // +1.0 oct -> ~240.0 BPM
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterTempo);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 240.0f) < 0.25f);
    RTEST_CHECK(std::fabs(ctrl.state().global.clockBpm - 240.0f) < 0.25f);
    RTEST_CHECK(nearF(ctrl.state().global.circuitCoupling, couplingBeforeRhythmAlg));
    for (int f = 0; f < 4; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -48); // Back to ~120.0 BPM
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(!ctrl.hud().selPageOpen);

    // Verify 3-gear logarithmic ALGORITHM tempo scaling in RHYTHM LAB across [20.0, 3000.0] BPM:
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // Fine (~0.2 BPM / detent @ 120 BPM)
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8); // +2 detents -> ~120.4 BPM
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 120.4f) < 0.1f);
    for (int f = 0; f < 4; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);

    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);    // Coarse (6.0 oct/turn)
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 200); // Clamp at 3,000.0 BPM!
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 3000.0f));
    // Verify high-speed 3,000 BPM LED anti-strobe choreography stays strictly in [0, 1]
    ctrl.onWhiteKeyPressure(12u, 0.70f); // EUCLID @ 3,000 BPM
    ctrl.tick(0.016f, tel);
    for (float l : ctrl.leds().whiteKeyLeds) {
        RTEST_CHECK(l >= 0.0f && l <= 1.0f);
    }
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -300); // Clamp at 20.0 BPM!
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 20.0f));
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 16); // +1.0 oct -> 40.0 BPM
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 40.0f) < 0.15f);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);

    // Verify 11th Tall Key = TAP TEMPO in RHYTHM LAB (never freezes scope, aligns Beat 1 on Tap 1,
    // rejects <40 ms switch bounce, holds MasterTempo toast between taps, and locks 120.0 BPM!):
    const uint8_t seqBeforeTap = ctrl.state().global.clockResetSeq;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::ScopeFreezeKey, 0.80f); // Held pressure: no 2x fire!
    RTEST_CHECK(!ctrl.hud().scopeFrozen);
    RTEST_CHECK(!ctrl.state().keyMod.stasisFreeze);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::TapTempo);
    RTEST_CHECK(ctrl.state().global.clockResetSeq == static_cast<uint8_t>(seqBeforeTap + 1u));
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "TAP TEMPO") == 0 && sb);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterTempo); // Steady toast between taps!

    // < 40 ms switch bounce after Tap 1 must be ignored without corrupting BPM:
    ctrl.tick(0.020f, tel);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 40.0f) < 0.15f);

    // Advance remaining 0.480 s (total 0.500 s since Tap 1 = 120.0 BPM) and tap twice @ 0.500 s:
    for (int f = 0; f < 24; ++f) {
        ctrl.tick(0.020f, tel);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 120.0f) < 0.5f);
    for (int f = 0; f < 25; ++f) {
        ctrl.tick(0.020f, tel); // 0.500 s -> Tap 3
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 120.0f));

    // 2. Exercise STUTTER, DIVIDE, EUCLID, PUMP, CHASE, TEMPO, PLUCK, SHORT, SWING + HUD headers
    ctrl.onWhiteKeyPressure(4u, 0.50f); // Key 5: STUTTER OSC 1
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceStutter);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[0] ==
                static_cast<uint8_t>(ruido::VoiceRhythmMode::Stutter));
    ctrl.onWhiteKeyGate(4u, false);

    ctrl.onWhiteKeyPressure(11u, 1.0f); // Key 12: DIVIDE OSC 4 @ full pressure -> X8 (index 5)
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceDivide);
    RTEST_CHECK(ctrl.state().keyMod.rhythmParam[3] == 5u);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "DIVIDE") == 0 && std::strcmp(ls, " OSC 4") == 0 &&
                    std::strcmp(rt, "X8") == 0 && sb);
    }
    ctrl.onWhiteKeyGate(11u, false);

    ctrl.onWhiteKeyPressure(12u, 0.45f); // Key 13: EUCLID OSC 1 -> 5/16
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceEuclid);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[0] ==
                static_cast<uint8_t>(ruido::VoiceRhythmMode::Euclid));
    ctrl.tick(0.016f, tel);
    for (float l : ctrl.leds().whiteKeyLeds) {
        RTEST_CHECK(l >= 0.0f && l <= 1.0f);
    }
    ctrl.onWhiteKeyGate(12u, false);

    // SAVE + Key 14 (white 13) stores SCENE 2 cleanly even inside RHYTHM LAB without firing EUCLID!
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed);
    ctrl.onWhiteKeyPressure(13u, 0.90f);
    RTEST_CHECK(!ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[1] == 0u);
    ctrl.onWhiteKeyGate(13u, false);

    // GLO PLUCK opens live MS-20 Filter Pluck graph & maps knobs to GlobalFilter
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 0.85f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmPluck);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmPluck, 0.85f));
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // OP5 PUMP, OP6 CHASE, MONO SHORT, POLY SWING
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 0.80f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmPump);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 0.70f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmChase);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Mono, 0.90f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmShort);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Poly, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmSwing);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "SWING") == 0 && std::strcmp(rt, "72%") == 0);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);

    // 3. Port-agnostic 24-PPQN MIDI Clock In (TRS / USB / BLE-MIDI burst jitter immunity)
    RTEST_CHECK(!ctrl.hud().midiClockSynced);
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 120.0f));
    const uint8_t seqBeforeStart = ctrl.state().global.clockResetSeq;
    ctrl.onMidiRealtime(0xFAu, 1.000f); // MIDI Start
    RTEST_CHECK(ctrl.state().global.clockResetSeq == static_cast<uint8_t>(seqBeforeStart + 1u));

    // Steady 140.0 BPM USB/TRS clock (dt = 60 / (140 * 24) = 0.01785714 s)
    float tMsg = 1.000f;
    const float dt140 = 60.0f / (140.0f * 24.0f);
    for (int k = 0; k < 36; ++k) {
        ctrl.onMidiRealtime(0xF8u, tMsg);
        tMsg += dt140;
    }
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 140.0f) < 0.4f);
    RTEST_CHECK(std::fabs(ctrl.state().global.clockBpm - 140.0f) < 0.4f);

    // Hold PIT TEMPO at minimal pressure -> header shows "140 BPM EXT"!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 0.002f);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "TEMPO") == 0 && std::strcmp(rt, "140 BPM EXT") == 0);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);

    // Turning ALGORITHM OR tapping 11th key while externally MIDI-synced shows "140 BPM EXT"
    // without mutating internal fallback BPM or breaking external lock (tap aligns Beat 1)!
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 24);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterTempo);
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 140.0f) < 0.4f);
    const uint8_t seqBeforeExtTap = ctrl.state().global.clockResetSeq;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(ctrl.state().global.clockResetSeq == static_cast<uint8_t>(seqBeforeExtTap + 1u));
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 140.0f) < 0.4f);
    for (int f = 0; f < 4; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();

    // Simulate Bluetooth BLE-MIDI 15 ms connection-interval packet clumping @ 125.0 BPM
    // (true tick period = 20.0 ms, arriving quantized into 15.0 ms BLE connection events)
    ctrl.onMidiRealtime(0xFAu, 10.000f);
    for (int tick = 0; tick < 72; ++tick) {
        const float idealSec = 10.000f + static_cast<float>(tick) * 0.020f;
        const float blePacketSec =
            10.000f + std::ceil((idealSec - 10.000f) / 0.015f) * 0.015f; // 0ms clumps + 15ms jumps
        ctrl.onMidiRealtime(0xF8u, blePacketSec);
    }
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 125.0f) < 1.5f);

    // GLOBAL CONFIG -> MIDI CLOCK IN: OFF ignores external clock immediately
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 16); // Row 4: MIDI CLOCK IN
    RTEST_CHECK(ctrl.hud().configCursor == 4u);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // AUTO -> OFF
    RTEST_CHECK(!ctrl.hud().midiClockSynced);
    RTEST_CHECK(nearF(ctrl.state().global.clockBpm, 120.0f));
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // OFF -> AUTO
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);

    // 1.6 s cable-unplug timeout falls back cleanly to 120 BPM; 0xFC Stop releases lock while
    // PHASE (default) leaves masterRun untouched
    for (int i = 0; i < 10; ++i) {
        ctrl.tick(0.20f, tel);
    }
    RTEST_CHECK(!ctrl.hud().midiClockSynced);
    RTEST_CHECK(nearF(ctrl.state().global.clockBpm, 120.0f));

    for (int k = 0; k < 12; ++k) {
        ctrl.onMidiRealtime(0xF8u, 30.0f + static_cast<float>(k) * 0.020f);
    }
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(ctrl.state().global.masterRun);
    ctrl.onMidiRealtime(0xFCu, 30.30f); // MIDI Stop in PHASE mode -> drone keeps running!
    RTEST_CHECK(!ctrl.hud().midiClockSynced);
    RTEST_CHECK(ctrl.state().global.masterRun);
    RTEST_CHECK(nearF(ctrl.state().global.clockBpm, 120.0f));

    // MIDI TRANSPORT IN == RUN/STOP couples 0xFA/0xFB/0xFC to masterRun when MIDI CLOCK IN == AUTO
    RTEST_CHECK(ctrl.acceptsMidiChannel(0x90u)); // Default OMNI accepts CH 1..16
    RTEST_CHECK(ctrl.acceptsMidiChannel(0x9Fu));
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);     // Row 5: MIDI TRANSPORT IN
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);  // PHASE -> RUN/STOP
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);     // Row 6: MIDI CHANNEL IN
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 16); // OMNI -> CH 4
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::RunStop);
    RTEST_CHECK(ctrl.hud().config.midiChannel == 4u);
    RTEST_CHECK(!ctrl.acceptsMidiChannel(0x90u)); // CH 1 rejected
    RTEST_CHECK(ctrl.acceptsMidiChannel(0x93u));  // CH 4 accepted (0-indexed 3)
    RTEST_CHECK(ctrl.acceptsMidiChannel(0xF8u));  // System Real-Time always accepted

    ctrl.onMidiRealtime(0xFCu, 31.0f); // Stop -> mutes masterRun
    RTEST_CHECK(!ctrl.state().global.masterRun);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::AudioOutput);
    ctrl.onMidiRealtime(0xFBu, 31.2f); // Continue -> resumes masterRun
    RTEST_CHECK(ctrl.state().global.masterRun);
    ctrl.onMidiRealtime(0xFCu, 31.4f);
    RTEST_CHECK(!ctrl.state().global.masterRun);
    ctrl.onMidiRealtime(0xFAu, 31.6f); // Start -> resumes masterRun + resets phase
    RTEST_CHECK(ctrl.state().global.masterRun);

    // 4. Multi-day continuous drone uptime immunity (no float32 timer stall after 80+ hours)
    ruido::MidiClockTracker longRunClock{};
    longRunClock.reset();
    for (int i = 0; i < 1200000; ++i) {
        (void)longRunClock.step(0.25f, ruido::ClockSyncMode::Auto); // 300,000 s (~83.3 hours!)
    }
    for (int k = 0; k < 36; ++k) {
        (void)longRunClock.step(0.020f, ruido::ClockSyncMode::Auto);
        (void)longRunClock.onMidiRealtime(0xF8u, ruido::ClockSyncMode::Auto, -1.0f);
    }
    RTEST_CHECK(longRunClock.isExternalSynced(ruido::ClockSyncMode::Auto));
    RTEST_CHECK(std::fabs(longRunClock.bpm(ruido::ClockSyncMode::Auto) - 125.0f) < 0.25f);

    // Also verify 64-second timestamp wraparound across 63.95 s -> 0.45 s boundary @ 120.0 BPM
    longRunClock.reset();
    for (int k = 0; k < 36; ++k) {
        const float rawTs = 63.90f + static_cast<float>(k) * (60.0f / (120.0f * 24.0f));
        const float wrappedTs = (rawTs >= 64.0f) ? (rawTs - 64.0f) : rawTs;
        (void)longRunClock.onMidiRealtime(0xF8u, ruido::ClockSyncMode::Auto, wrappedTs);
    }
    RTEST_CHECK(longRunClock.isExternalSynced(ruido::ClockSyncMode::Auto));
    RTEST_CHECK(std::fabs(longRunClock.bpm(ruido::ClockSyncMode::Auto) - 120.0f) < 0.25f);

    // 5. Bidirectional `dinghy` NVS flash ports (`PatchSnapshot` 128 B & `DeviceSettingsBlob` 16 B)
    ctrl.reset();
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == 0u);
    RTEST_CHECK(!ctrl.consumeSettingsDirty());
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 24);
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.onWhiteKeyGate(14u, true); // Save into slot 2 (Key 15)
    ctrl.onWhiteKeyGate(14u, false);
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == (1u << 2u));
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == 0u);
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    const float savedOsc1Hz = ctrl.state().voices[0].frequencyHz;
    const ruido::PatchSnapshot savedSlot2 = ctrl.snapshots().slotBinary(2u);

    // Switch to RhythmLab, dial ~1,920 BPM (+4 octaves = +192 ticks) via logarithmic ALGORITHM,
    // then switch to CircuitLab + Latching MuteMode and export 16 B settings blob!
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);      // -> RhythmLab
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 192); // +4.0 octaves -> ~1920.0 BPM
    const float dialedHighBpm = ctrl.hud().effectiveClockBpm;
    RTEST_CHECK(std::fabs(dialedHighBpm - 1920.0f) < 2.0f);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // -> CircuitLab (ALGORITHM = Rail Coupling)
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    const float couplingBeforeCircuitAlg = ctrl.state().global.circuitCoupling;
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    RTEST_CHECK(ctrl.state().global.circuitCoupling > couplingBeforeCircuitAlg);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Row 1: MuteMode
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 16);    // Row 5: MidiTransport
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);  // -> RunStop
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);     // Row 6: MidiChannel
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 28); // -> CH 7
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    const ruido::DeviceSettingsBlob savedSettings = ctrl.exportSettings();

    // Fresh boot restore + corrupt-CRC rejection (never bricks on uninitialized 0xFF flash!)
    ruido::RuidoController bootCtrl{};
    ruido::PatchSnapshot corruptSnap = savedSlot2;
    corruptSnap.crc32 ^= 0xDEADBEEFu;
    RTEST_CHECK(!bootCtrl.loadSnapshotSlot(2u, corruptSnap));

    ruido::DeviceSettingsBlob erasedFlashSettings{};
    std::memset(&erasedFlashSettings, 0xFF, sizeof(erasedFlashSettings));
    RTEST_CHECK(!bootCtrl.loadSettings(erasedFlashSettings));

    RTEST_CHECK(bootCtrl.loadSnapshotSlot(2u, savedSlot2));
    RTEST_CHECK(bootCtrl.loadSettings(savedSettings));
    RTEST_CHECK(bootCtrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    RTEST_CHECK(bootCtrl.hud().config.muteMode == ruido::KeySwitchMode::Latching);
    RTEST_CHECK(bootCtrl.hud().config.midiTransport == ruido::MidiTransportMode::RunStop);
    RTEST_CHECK(bootCtrl.hud().config.midiChannel == 7u);
    RTEST_CHECK(std::fabs(bootCtrl.hud().effectiveClockBpm - dialedHighBpm) < 0.1f);
    RTEST_CHECK(bootCtrl.snapshots().activeSlot() == 2u);
    RTEST_CHECK(std::fabs(bootCtrl.state().voices[0].frequencyHz - savedOsc1Hz) < 0.25f);
#undef RTEST_CHECK
}

} // namespace ruido_test
