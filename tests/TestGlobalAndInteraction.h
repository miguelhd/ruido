#pragma once

#include "core/GlobalConfig.h"
#include "core/GlobalControl.h"
#include "core/HudEnvelope.h"
#include "core/Parameters.h"
#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/ScopeHudRenderer.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

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
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, true);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 4);
    RTEST_CHECK(ctrl.hud().focusedKnob == 2u);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, true);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);

    // 2. Discrete notched detents (SELECT, TARGET, GLOBAL CONFIG) step 1 item per 4 ticks
    //    across all 3 gears (Standard, Fine, Coarse)
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 0u);

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // FINE
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 1u);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
    RTEST_CHECK(ctrl.state().voices[1].lfoTarget == ruido::LfoTarget::LpResonance);
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);

    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true); // COARSE
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().lfoVoiceFocus == 2u);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
    RTEST_CHECK(ctrl.state().voices[2].lfoTarget == ruido::LfoTarget::LpResonance);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);

    // 3. Option A Split Top-Bar: holding OCT-/OCT+ overrides top-left to FINE/COARSE across modes
    //    while updating top-right readout live as knobs turn
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().rateHoldActive);
    RTEST_CHECK(ctrl.hud().knobRate == ruido::KnobRate::Fine);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 1.0f));

    // Render top header strip in pure FINE hold vs FINE + Knob1 turn -> top-left stays identical
    // ("FINE") while top-right changes to live "OSC 1  ... Hz"!
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

    bool leftFineLocked = true;
    size_t rightReadoutDiffs = 0u;
    for (size_t y = 6u; y <= 19u; ++y) {
        for (size_t x = 10u; x <= 60u; ++x) {
            if (hdrRateOnly[y * 240u + x] != hdrRateKnob[y * 240u + x]) {
                leftFineLocked = false;
            }
        }
        for (size_t x = 120u; x <= 230u; ++x) {
            if (hdrRateOnly[y * 240u + x] != hdrRateKnob[y * 240u + x]) {
                ++rightReadoutDiffs;
            }
        }
    }
    RTEST_CHECK(leftFineLocked);
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

    // (b) PRESETS vs SELECT unassigned encoder distinct labels
    ctrl.reset();
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedEncoder);
    RTEST_CHECK(ctrl.hud().lastEncoder == ruido::EncoderId::Select);
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateOnly.data());

    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedEncoder);
    RTEST_CHECK(ctrl.hud().lastEncoder == ruido::EncoderId::Presets);
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateKnob.data());
    size_t encTitleDiffs = 0u;
    for (size_t i = 0u; i < hdrRateOnly.size(); ++i) {
        if (hdrRateOnly[i] != hdrRateKnob[i]) {
            ++encTitleDiffs;
        }
    }
    RTEST_CHECK(encTitleDiffs > 25u);

    // (c) Armed SAVE cancelled by unassigned switch clears orphaned saveArmed LED state
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
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
    std::printf(
        "[Test 9] 5-Mode Keybed GLOBAL (FREQ/FILTER/LFO/AMP/ALL) & Scalable GLOBAL CONFIG...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    ruido::EngineTelemetry tel{};
    std::array<float, ruido::kNumOscillators> zeroLfo{0.0f, 0.0f, 0.0f, 0.0f};

    // 1. Keybed GLO defaults to GlobalTarget::All on boot, steps 0..4 via SELECT, and remembers
    //    last target across releases
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::All);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAll);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalAll);

    ctrl.onEncoderDelta(ruido::EncoderId::Select, -16); // Step to GlobalTarget::Freq (0)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Freq);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalFreq);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Freq);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalFreq);

    // 2. GLOBAL FREQ orthogonality (PITCH | SPREAD | DETUNE | TILT) & unison recovery
    const auto evalFreq = [&](uint8_t col) {
        return ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalFreq, col, ctrl.state(),
                                           zeroLfo);
    };
    const float spread0 = evalFreq(1u).primaryValue;
    const float detune0 = evalFreq(2u).primaryValue;
    const float tilt0 = evalFreq(3u).primaryValue;
    RTEST_CHECK(std::fabs(detune0) < 5.0f); // Default boot chord ~centered (-2c)

    // Turn PITCH (Knob1): transposes mean Hz while SPREAD, DETUNE, and TILT stay invariant
    const float mean0 = evalFreq(0u).primaryValue;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 24);
    RTEST_CHECK(evalFreq(0u).primaryValue > mean0 * 1.45f);
    RTEST_CHECK(nearF(evalFreq(1u).primaryValue, spread0, 0.05f));
    RTEST_CHECK(nearF(evalFreq(2u).primaryValue, detune0, 0.5f));
    RTEST_CHECK(nearF(evalFreq(3u).primaryValue, tilt0, 0.05f));

    // Turn DETUNE (Knob3): mean Hz & TILT stay invariant while beating cents increase
    const float meanAfterPitch = evalFreq(0u).primaryValue;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 24);
    RTEST_CHECK(evalFreq(2u).primaryValue > detune0 + 25.0f);
    RTEST_CHECK(nearF(evalFreq(0u).primaryValue, meanAfterPitch, 0.25f));
    RTEST_CHECK(nearF(evalFreq(3u).primaryValue, tilt0, 0.05f));

    // Turn TILT (Knob4): mean Hz & DETUNE stay invariant while inner voices skew
    const float detuneAfter = evalFreq(2u).primaryValue;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 24);
    RTEST_CHECK(evalFreq(3u).primaryValue > tilt0 + 0.5f);
    RTEST_CHECK(nearF(evalFreq(0u).primaryValue, meanAfterPitch, 0.25f));
    RTEST_CHECK(nearF(evalFreq(2u).primaryValue, detuneAfter, 0.6f));

    // Collapse SPREAD (Knob2) to 0.0 st unison, then re-fan CW -> recovers wide chord!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -200);
    RTEST_CHECK(evalFreq(1u).primaryValue < 0.05f);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 60);
    RTEST_CHECK(evalFreq(1u).primaryValue > 8.0f);

    // 3. Step SELECT -> GLOBAL FILTER (1): relative deltas + 4-voice micro-bead spread
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.reset();
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -40); // Offset Voice 0 LPF from Voices 1-3
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -12); // All(4) - 3 = Filter(1)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Filter);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalFilter);
    const auto fltCol =
        ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalFilter, 2u, ctrl.state(), zeroLfo);
    RTEST_CHECK(fltCol.hasVoiceSpread);

    // 4. Step SELECT -> GLOBAL LFO (2): "MIXED" shape detection & 4-voice uniform alignment
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 120); // Voice 0 shape > 0.45, Voices 1-3 = 0.0
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Filter(1) -> Lfo(2)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Lfo);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalLfo);
    const auto lfoShapeMixed =
        ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalLfo, 1u, ctrl.state(), zeroLfo);
    RTEST_CHECK(lfoShapeMixed.lfoShapeMixed);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 260); // Drive all 4 shapes to 1.0 -> uniform!
    const auto lfoShapeUniform =
        ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalLfo, 1u, ctrl.state(), zeroLfo);
    RTEST_CHECK(!lfoShapeUniform.lfoShapeMixed);

    // 5. Step SELECT -> GLOBAL AMP (3): LEVEL, TILT, WIDTH collapse/re-fan, BALANCE
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Lfo(2) -> Amp(3)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Amp);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAmp);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -240); // Collapse WIDTH -> 0%
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        RTEST_CHECK(std::fabs(ctrl.state().voices[v].pan) < 0.02f);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 120); // Re-fan WIDTH CW -> wide stereo!
    RTEST_CHECK(ctrl.state().voices[0].pan < -0.35f);
    RTEST_CHECK(ctrl.state().voices[3].pan > 0.35f);

    // 6. Step SELECT -> GLOBAL ALL (4): 4-stage macro edits all 4 voices simultaneously
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Amp(3) -> All(4)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::All);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAll);
    const float v2HzBefore = ctrl.state().voices[2].frequencyHz;
    const float v2LpBefore = ctrl.state().voices[2].lpCutoffHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 16);  // FREQ macro
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -24); // FILTER macro
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 80);  // LFO depth macro
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -20); // AMP level macro
    RTEST_CHECK(ctrl.state().voices[2].frequencyHz > v2HzBefore * 1.2f);
    RTEST_CHECK(ctrl.state().voices[2].lpCutoffHz < v2LpBefore * 0.85f);
    RTEST_CHECK(ctrl.state().voices[2].lfoDepth > 0.25f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // 7. Render all 5 GLOBAL custom center graphs & GLOBAL CONFIG menu:
    //    verify 240x8 == 240x16 == 240x24 strip equivalence, 100% R==G==B neutrality,
    //    non-empty custom center visuals, and hidden bottom strip in GLOBAL CONFIG!
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
                if (y >= 191u && y <= 232u && g6 >= 8u) {
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

    ruido::EngineControlState quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -20); // Start at Freq(0)
    for (int m = 0; m < 5; ++m) {
        renderer.reset();
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.025f, tel);
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        verifyViewStripsAndCenter(true);
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // 8. Right-panel GLO = scalable GLOBAL CONFIG menu + SNAP BACK end-to-end verification
    ctrl.reset();
    RTEST_CHECK(
        nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)], 0.15f));
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(ctrl.hud().configPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    RTEST_CHECK(ctrl.hud().configCursor == 0u);

    quietSt = ctrl.state();
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

    // Row 0: GLO RELEASE -> step CW to SNAP BACK via Knob2
    RTEST_CHECK(ctrl.hud().config.gloRelease == ruido::GloReleaseMode::Latch);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 4);
    RTEST_CHECK(ctrl.hud().config.gloRelease == ruido::GloReleaseMode::SnapBack);

    // Step SELECT -> Row 1: OUTPUT ROUTING -> step CW to DUAL MONO via Knob4, synced with MONO/POLY
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 1u);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 4);
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);

    // Close GLOBAL CONFIG via 2nd GloRight tap -> 150 ms smooth fade-out
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(!ctrl.hud().configPageOpen);
    ctrl.tick(0.075f, tel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, tel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));

    // Verify SNAP BACK in performance: hold keybed GLO, mangle all 4 voices, release -> exact snap!
    const auto preHoldVoices = ctrl.state().voices;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 36);  // Transpose pitch
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -48); // Darken LPF
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -36); // Drop amplitude
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > preHoldVoices[0].frequencyHz * 1.4f);
    RTEST_CHECK(ctrl.state().voices[0].lpCutoffHz < preHoldVoices[0].lpCutoffHz * 0.7f);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].frequencyHz, preHoldVoices[v].frequencyHz));
        RTEST_CHECK(nearF(ctrl.state().voices[v].lpCutoffHz, preHoldVoices[v].lpCutoffHz));
        RTEST_CHECK(nearF(ctrl.state().voices[v].amplitude, preHoldVoices[v].amplitude));
    }
#undef RTEST_CHECK
}

} // namespace ruido_test
