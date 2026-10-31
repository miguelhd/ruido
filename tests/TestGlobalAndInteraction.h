#pragma once

#include "core/GlobalConfig.h"
#include "core/GlobalControl.h"
#include "core/HudEnvelope.h"
#include "core/Parameters.h"
#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/FontSansAA.h"
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
    std::printf(
        "[Test 9] 5-Mode Keybed GLOBAL (FREQ/FILTER/LFO/AMP/ALL) & Scalable GLOBAL CONFIG...\n");
    const auto nearF = [](float a, float b, float eps = 1.0e-4f) {
        return std::fabs(a - b) <= eps;
    };
#define RTEST_CHECK(expr) checkFn((expr), #expr, __LINE__)

    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;
    ruido::EngineTelemetry tel{};
    ruido::LfoSignalArray zeroLfo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

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
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -40); // Offset Voice 0 LPF from Voices 1-3
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);

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

    // 5. Step SELECT -> GLOBAL LEVEL (3): LEVEL, TILT, SPREAD (amplitude contrast), PAN (stereo
    // shift)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Lfo(2) -> Amp(3)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Amp);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAmp);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 120); // Expand SPREAD CW -> terraced amp contrast!
    RTEST_CHECK(ctrl.state().voices[0].amplitude + 0.25f < ctrl.state().voices[3].amplitude);
    const auto ampSpreadOpen =
        ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalAmp, 2u, ctrl.state(), zeroLfo);
    RTEST_CHECK(ampSpreadOpen.primaryValue > 0.35f);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -240); // Collapse SPREAD CCW -> uniform levels!
    for (size_t v = 1u; v < ruido::kNumOscillators; ++v) {
        RTEST_CHECK(std::fabs(ctrl.state().voices[v].amplitude - ctrl.state().voices[0].amplitude) <
                    0.01f);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 260); // Shift ensemble PAN Right -> +1.0!
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        RTEST_CHECK(ctrl.state().voices[v].pan > 0.90f);
    }
    const auto panRightSummary =
        ruido::evaluateGlobalColumn(ruido::KnobBankMode::GlobalAmp, 3u, ctrl.state(), zeroLfo);
    RTEST_CHECK(panRightSummary.bipolar && panRightSummary.primaryValue > 0.90f);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -520); // Shift ensemble PAN Left -> -1.0!
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        RTEST_CHECK(ctrl.state().voices[v].pan < -0.90f);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 120); // Return toward center stereo field

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

    static ruido::DroneEngine liveEng;
    static std::array<float, 128> liveOutL{};
    static std::array<float, 128> liveOutR{};
    liveEng.init(44100.0f);
    ruido::EngineControlState liveSt = ctrl.state();
    liveSt.global.masterRun = true;
    liveSt.global.masterTrim = 0.85f;
    liveEng.controlBus().publish(liveSt);
    liveEng.snapParametersToLatest();
    for (int b = 0; b < 36; ++b) {
        liveEng.processBlock(liveOutL.data(), liveOutR.data(), 128u);
    }

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -20); // Start at Freq(0)
    for (int m = 0; m < 5; ++m) {
        renderer.reset();
        for (int b = 0; b < 36; ++b) {
            liveEng.processBlock(liveOutL.data(), liveOutR.data(), 128u);
            renderer.ingestScopeRing(liveEng.scopeBuffer());
        }
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.025f, liveEng.telemetry());
            renderer.prepareFrame(0.025f, liveSt, ctrl.hud(), liveEng.telemetry());
        }
        verifyViewStripsAndCenter(true);
        if (m == 3) {
            // GLOBAL LEVEL Knob 4 (PAN) switches cleanly to unified PAN stereo stage
            ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 4);
            for (int f = 0; f < 10; ++f) {
                ctrl.tick(0.025f, liveEng.telemetry());
                renderer.prepareFrame(0.025f, liveSt, ctrl.hud(), liveEng.telemetry());
            }
            verifyViewStripsAndCenter(true);
        } else if (m == 4) {
            // GLOBAL ALL K1..K4 route cleanly across live scope, Filter, LFO, and LEVEL
            for (int gk = 0; gk < 4; ++gk) {
                const auto enc = static_cast<ruido::EncoderId>(
                    static_cast<uint8_t>(ruido::EncoderId::Knob1) + gk);
                ctrl.onEncoderDelta(enc, 4);
                for (int f = 0; f < 10; ++f) {
                    ctrl.tick(0.025f, liveEng.telemetry());
                    renderer.prepareFrame(0.025f, liveSt, ctrl.hud(), liveEng.telemetry());
                }
                verifyViewStripsAndCenter(true);
            }
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    ruido::EngineControlState quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;

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

    // Verify GLOBAL CONFIG renders zero row separators (y = 69, 97) and zero leftmost white slit
    // (x = 10..13 on selected row y = 44..67 has only subtle ~12% highlight tint, never white bar)
    uint16_t maxConfigSepG6 = 0u;
    uint16_t maxSelectedLeftEdgeG6 = 0u;
    for (size_t x = 14u; x <= 226u; ++x) {
        const uint16_t g6Sep0 = (s16[69u * 240u + x] >> 5u) & 0x3Fu;
        const uint16_t g6Sep1 = (s16[97u * 240u + x] >> 5u) & 0x3Fu;
        if (g6Sep0 > maxConfigSepG6) {
            maxConfigSepG6 = g6Sep0;
        }
        if (g6Sep1 > maxConfigSepG6) {
            maxConfigSepG6 = g6Sep1;
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

    // Row 0: GLO RELEASE -> step CW to SNAP BACK via ALGORITHM encoder (Knob1 must NOT edit menu!)
    RTEST_CHECK(ctrl.hud().config.gloRelease == ruido::GloReleaseMode::Latch);
    const float osc1BeforeMenuTurn = ctrl.state().voices[0].frequencyHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 8);
    RTEST_CHECK(ctrl.hud().config.gloRelease == ruido::GloReleaseMode::Latch);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > osc1BeforeMenuTurn * 1.10f);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    for (int i = 0; i < 18; ++i) {
        ctrl.tick(0.100f, tel);
    }
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);

    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.gloRelease == ruido::GloReleaseMode::SnapBack);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);

    // Step SELECT -> Row 1: OUTPUT ROUTING -> step CW to MONO via ALGORITHM, synced with
    // MONO/POLY
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 1u);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::OutputRouting,
                                                          ctrl.hud().config, ctrl.state().global),
                            "STEREO") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::OutputRouting,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MONO") == 0);
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

    // Close & re-open EDIT -> remembers last selected EditTarget (Drift -> step to Level),
    // and SEL opens dedicated latching PAN page (panPageOpen)
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().panPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 60);
    RTEST_CHECK(ctrl.state().voices[2].pan > -0.20f);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().panPageOpen);
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Drift);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Drift(1) -> Level(2)
    RTEST_CHECK(ctrl.hud().editTarget == ruido::EditTarget::Level);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);

    // KNOB1 -> 100% (4.5x Drive kept 100% intact!), mute Voice 1 via White Key 2
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 200);
    RTEST_CHECK(nearF(ctrl.state().voices[0].amplitude, 1.0f));
    RTEST_CHECK(nearF(ruido::computeVoiceChannelGain(ctrl.state().voices[0].amplitude).preDrive,
                      4.5f, 1.0e-3f));

    ctrl.onWhiteKeyGate(1u, true);
    ctrl.onWhiteKeyGate(1u, false);
    RTEST_CHECK(ctrl.state().voices[1].muted);

    // 3. Momentary Keybed GLO overrides EDIT temporarily and returns seamlessly on release
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAll);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalAll);
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

    // Verify 0-70% clean stem (`y = 85..175`, quiet background) stays razor-sharp (`cx = 30`) with
    // zero wide post-filter `voicePeak` plasma halo in `x = 20..25` / `x = 35..40`!
    size_t cleanStemSideHaloPx = 0u;
    for (size_t y = 85u; y <= 175u; ++y) {
        if (y == 120u) {
            continue; // quiet scope flatline horizon
        }
        for (size_t x = 20u; x <= 25u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                ++cleanStemSideHaloPx;
            }
        }
        for (size_t x = 35u; x <= 40u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                ++cleanStemSideHaloPx;
            }
        }
    }
    RTEST_CHECK(cleanStemSideHaloPx == 0u);

    // Compare Column 0 upper overdrive zone (y = 36..76) between 100% (4.5x drive, Option 3A
    // narrow triangular hairy CRT cap) and clean 65% VCA:
    //  - Narrow V-triangle lights center hair (`x = 24..36`) while keeping wide outer regions
    //    (`x = 8..22` and `x = 38..52`) 100% clear (zero wide wings or floating text!).
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

    size_t driveZonePxHot = 0u;
    size_t driveZonePxClean = 0u;
    size_t wideOuterPxHot = 0u;
    for (size_t y = 36u; y <= 76u; ++y) {
        for (size_t x = 24u; x <= 36u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                ++driveZonePxHot;
            }
            if (((clean16[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                ++driveZonePxClean;
            }
        }
        for (size_t x = 8u; x <= 22u; ++x) {
            if (((amp16[y * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
                ++wideOuterPxHot;
            }
        }
        for (size_t x = 38u; x <= 52u; ++x) {
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
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::GloRelease),
                            "GLOBAL RELEASE") == 0);
    RTEST_CHECK(std::strcmp(ruido::globalTargetSuffix(ruido::GlobalTarget::Amp), " LEVEL") == 0);

    // Verify 240x8 == 240x16 == 240x24 strip equivalence & monochrome neutrality across all 3 EDIT
    // modes (DUTY · DRIFT · LEVEL) + dedicated SEL (PAN) page
    driveTel.voiceDriftNorm[2].store(0.35f, std::memory_order_relaxed);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -16); // Clamp to Width(0)
    for (int em = 0; em < 4; ++em) {
        if (em == 3) {
            ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
            ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
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

    // Reopen EDIT (closing SEL PAN), then close EDIT via 2nd EDIT tap -> smooth 150 ms fade-out &
    // live EDIT LED overdrive flare!
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    ctrl.tick(0.200f, driveTel);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen);
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
        const size_t x0 = col * 60u + 8u;
        const size_t x1 = col * 60u + 52u;
        for (size_t y = 191u; y <= 201u; ++y) {
            for (size_t x = x0; x <= x1; ++x) {
                const uint16_t g6 = (img[y * 240u + x] >> 5u) & 0x3Fu;
                if (g6 > maxG6) {
                    maxG6 = g6;
                }
            }
        }
        return maxG6;
    };

    // (a) Mute Osc 1 via White Key 1 (focusedKnob == 0): OSC 1 text dims immediately (never 255
    //     white!) and exhibits ZERO pulsing across frames while White Key 1 pulses at 1.50 Hz!
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
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
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) <= 22u); // Dimmed (~72/255 -> g6 ~18), even when focused!
    RTEST_CHECK(maxOscLabelG6(amp16, 2u) >= 35u); // Audible unfocused OSC 3 stays bright (~154/255)

    float ledMuteA = ctrl.leds().whiteKeyLeds[0];
    for (int f = 0; f < 15; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    float ledMuteB = ctrl.leds().whiteKeyLeds[0];
    renderer.renderFullScreenStrips(clean16.data());
    RTEST_CHECK(std::fabs(ledMuteB - ledMuteA) > 0.05f); // Physical button pulses!
    bool lowerRailSteadyOnMute = true;
    for (size_t y = 190u; y < 240u; ++y) {
        for (size_t x = 0u; x < 60u; ++x) {
            if (amp16[y * 240u + x] != clean16[y * 240u + x]) {
                lowerRailSteadyOnMute = false;
            }
        }
    }
    RTEST_CHECK(lowerRailSteadyOnMute); // Zero pulsing on screen!

    // (b) Unmute Osc 1, then sweep Osc 1 into sub-audio (< 20 Hz, down to 0.05 Hz lower rail):
    //     White Key 1 breathes at 0.35 Hz while OSC 1 text dims steadily with zero screen pulsing!
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    RTEST_CHECK(!ctrl.state().voices[0].muted);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -120); // Clamps at 0.05 Hz lower rail!
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
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) <= 22u); // Sub-audio OSC 1 dimmed even while focused!
    RTEST_CHECK(maxOscLabelG6(amp16, 2u) >= 35u);

    for (int f = 0; f < 20; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    const float ledSubB = ctrl.leds().whiteKeyLeds[0];
    RTEST_CHECK(std::fabs(ledSubB - ledSubA) > 0.015f); // White Key 1 breathes at 0.35 Hz!
    renderer.renderFullScreenStrips(clean16.data());
    bool lowerRailSteadySubAudio = true;
    for (size_t y = 190u; y < 240u; ++y) {
        for (size_t x = 0u; x < 60u; ++x) {
            if (amp16[y * 240u + x] != clean16[y * 240u + x]) {
                lowerRailSteadySubAudio = false;
            }
        }
    }
    RTEST_CHECK(lowerRailSteadySubAudio); // Zero pulsing on screen in sub-audio!

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

    // (c) EDIT Level 0% lower rail auto-mutes, Fine +1 CW unmutes, and White Key 1 unmute recovers
    //     default 65% audible level instead of staying stuck silent at 0%!
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

    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -20); // Back to 0% lower rail -> muted
    RTEST_CHECK(ctrl.state().voices[0].muted);
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false); // Tap pulsing White Key 1 -> restores 65% audible level!
    RTEST_CHECK(!ctrl.state().voices[0].muted);
    RTEST_CHECK(nearF(ctrl.state().voices[0].amplitude, ruido::kDefaultVoiceAmp));

    // (d) SEL (PAN) Edge-to-Edge Stereo Panorama Curve (HudGraphs sister to FX & LFO):
    //     SELECT switches STEREO <-> MONO while keeping the "PAN" title steady,
    //     left/right panorama steering, spotlight vs 30% silver companion contrast, LFO -> PAN
    //     strip equivalence, MONO center convergence + strictly locked KNOB 1-4 + centered
    //     "OUTPUT ROUTING SET TO MONO" bottom message (no OSC 1-4 columns), and 150 ms dissolve!
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.tick(0.016f, driveTel);
    RTEST_CHECK(ctrl.hud().panPageOpen && !ctrl.hud().editPageOpen);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 1.0f));

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

    const uint32_t leftPanorama = sumBoxG6(amp16, 4u, 108u, 44u, 180u);
    const uint32_t rightPanorama = sumBoxG6(amp16, 132u, 235u, 44u, 180u);
    RTEST_CHECK(leftPanorama > rightPanorama * 4u);
    // Edge-to-edge y = 186 baseline filament is present on both sides; zero floating top text!
    RTEST_CHECK(maxBoxG6(amp16, 200u, 239u, 185u, 187u) >= 6u);
    RTEST_CHECK(maxBoxG6(amp16, 4u, 235u, 26u, 33u) == 0u);

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
    ruido::formatHeaderPanMode(ctrl.state(), ctrl.hud(), hdrSuffix, sizeof(hdrSuffix), hdrRight,
                               sizeof(hdrRight));
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

    // Hold GLO -> GLOBAL LEVEL -> KNOB 4 (PAN) in MONO also acts strictly locked (does not focus
    // K4 or alter pan) and formats as disabled "LOCKED"
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -20); // Home to GlobalTarget::Freq (0)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);  // Step +3 detents -> GlobalTarget::Amp (3)
    RTEST_CHECK(ctrl.hud().globalTarget == ruido::GlobalTarget::Amp);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -80);
    RTEST_CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].pan, savedStereoPan[v]));
    }
    {
        char roleBuf[12]{};
        char valBuf[16]{};
        ruido::GlobalColumnSummary colGs{};
        ruido::formatColumnRoleAndValue(ctrl.state(), ctrl.hud(), zeroLfo, 3u, roleBuf,
                                        sizeof(roleBuf), valBuf, sizeof(valBuf), colGs);
        RTEST_CHECK(colGs.disabled && std::strcmp(roleBuf, "PAN") == 0 &&
                    std::strcmp(valBuf, "LOCKED") == 0);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // Turning SELECT CCW on SEL switches MONO -> STEREO ("PAN · STEREO") with original stereo pan
    // positions intact!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(!ctrl.state().global.monoCollapse && ctrl.hud().mode == ruido::HudMode::Panning);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].pan, savedStereoPan[v]));
    }
    ctrl.tick(0.025f, driveTel);

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(!ctrl.hud().panPageOpen);
    ctrl.tick(0.075f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, driveTel);
    RTEST_CHECK(nearF(ctrl.hud().opacity, 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 0.75f));

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

    // (g) LEVEL screen (`HudAmpGraph::compositeLevelStrip`): 3-px dot-matched stem width (`x =
    //     29..31` solid body at `cx = 30`) and monotonic level-brightness scaling across `3% ->
    //     35% -> 68% -> 100%`, with clearly visible brightness even at the lowest audible level.
    {
        ruido::EngineTelemetry silentTel{};
        ruido::HudState lvlHud{};
        lvlHud.mode = ruido::HudMode::Amplitude;
        lvlHud.knobBank = ruido::KnobBankMode::Amplitude;
        lvlHud.editTarget = ruido::EditTarget::Level;
        lvlHud.editPageOpen = true;
        lvlHud.opacity = 1.0f;

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
                // Lowest audible level (3%): clearly visible bead/stem (>= 28 G6) and thicker
                // 3-px body at y = 184 (`x = 29, 30, 31` all >= 16 G6; outer `x <= 25` == 0).
                uint16_t lowMaxG6 = 0u;
                for (size_t y = 178u; y <= 187u; ++y) {
                    for (size_t x = 28u; x <= 32u; ++x) {
                        const uint16_t g = pxG6(y, x);
                        if (g > lowMaxG6) {
                            lowMaxG6 = g;
                        }
                    }
                }
                RTEST_CHECK(lowMaxG6 >= 28u);
                RTEST_CHECK(pxG6(184u, 29u) >= 16u && pxG6(184u, 30u) >= 26u &&
                            pxG6(184u, 31u) >= 16u);
                RTEST_CHECK(pxG6(184u, 25u) == 0u && pxG6(184u, 35u) == 0u);
            }
            if (li == 2u) {
                // At 68% clean VCA, `x = 29..31` mid-stem (`y = 140`) forms a bright 3-px bar
                // matching the dot width, with crisp zero halo outside `x = 26..34`.
                RTEST_CHECK(pxG6(140u, 29u) >= 34u && pxG6(140u, 30u) >= 46u &&
                            pxG6(140u, 31u) >= 34u);
                RTEST_CHECK(pxG6(140u, 26u) == 0u && pxG6(140u, 34u) == 0u);
            }
            uint32_t bodySum = 0u;
            for (size_t y = 183u; y <= 185u; ++y) {
                for (size_t x = 29u; x <= 31u; ++x) {
                    bodySum += pxG6(y, x);
                }
            }
            stemBodySums[li] = bodySum;
        }
        RTEST_CHECK(stemBodySums[0] < stemBodySums[1] && stemBodySums[1] < stemBodySums[2] &&
                    stemBodySums[2] < stemBodySums[3]);
    }

    // (h) Shared `y = 186` edge-to-edge baseline (`kGraphBaselineY`, `0..239`, `50.0f`, `0.50f`)
    //     bit-exact opacity & position consistency across FILTER (`FX`), PAN (`SEL` STEREO & MONO),
    //     and LEVEL (`EDIT`):
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

        // Full 240-px edge-to-edge baseline at y = 186 (`G6 ~ 12`) with zero overhang at y >= 189
        RTEST_CHECK(rowSumG6(amp8, 186u, 0u, 3u) >= 40u);
        RTEST_CHECK(rowSumG6(amp8, 186u, 236u, 239u) >= 40u);
        RTEST_CHECK(rowSumG6(amp8, 189u, 0u, 239u) == 0u);

        // Every single pixel x = 0..239 across rows y = 186..189 has 100% bit-exact identical
        // baseline opacity across FILTER, PAN (STEREO), PAN (MONO), and LEVEL!
        bool allBaselinesBitExact = true;
        for (size_t y = 186u; y <= 189u; ++y) {
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
#undef RTEST_CHECK
}

} // namespace ruido_test
