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
#include "ui/HudKeybedInspector.h"
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
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4); // LFO B OSC 1: PulseWidth(1) + 1 -> LpCut(2)
    RTEST_CHECK(ctrl.state().voices[0].lfoBTarget == ruido::LfoTarget::LpCutoff);
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

    // (b) Idle SELECT steps ScopeTimebase (4X..1/8X) with stationary scroll indicator + CRT recoil
    ctrl.reset();
    RTEST_CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Auto);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Zoom4x), "4X") == 0);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Zoom2x), "2X") == 0);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Auto), "AUTO") == 0);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Wide2x), "1/2X") == 0);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Wide4x), "1/4X") == 0);
    RTEST_CHECK(std::strcmp(ruido::scopeTimebaseLabel(ruido::ScopeTimebase::Wide8x), "1/8X") == 0);

    const auto findHeaderBarLeftX = [&](const ruido::HudState& h, int scanStartX) -> int {
        ruido::ScopeHudRenderer tbRend;
        tbRend.prepareFrame(0.016f, silentSt, h, tel);
        std::array<uint16_t, 240u * 24u> strip{};
        tbRend.renderStrip(0u, 24u, strip.data());
        for (int x = scanStartX; x < 200; ++x) {
            if (((strip[13u * 240u + static_cast<size_t>(x)] >> 5u) & 0x3Fu) > 0u) {
                return x;
            }
        }
        return -1;
    };
    const auto findScopeBarLeftX = [&](ruido::ScopeTimebase tb) -> int {
        ruido::HudState tbHud = ctrl.hud();
        tbHud.mode = ruido::HudMode::ScopeTimebase;
        tbHud.opacity = 1.0f;
        tbHud.scopeTimebase = tb;
        return findHeaderBarLeftX(tbHud, 135);
    };
    const int barLeft2x = findScopeBarLeftX(ruido::ScopeTimebase::Zoom2x);
    const int barLeftAuto = findScopeBarLeftX(ruido::ScopeTimebase::Auto);
    const int barLeftHalf = findScopeBarLeftX(ruido::ScopeTimebase::Wide2x);
    const int barLeftEighth = findScopeBarLeftX(ruido::ScopeTimebase::Wide8x);
    RTEST_CHECK(barLeftAuto > 140);
    RTEST_CHECK(barLeft2x == barLeftAuto);
    RTEST_CHECK(barLeftHalf == barLeftAuto);
    RTEST_CHECK(barLeftEighth == barLeftAuto);

    // Scene glide ("20%" vs "100%"), MasterTempo ("25.5 BPM" vs "1920 BPM"), and KeyboardPlay
    // ("2.4 Hz" vs "24.0 Hz", "9.9 kHz" vs "400 Hz", "/2" vs "X8", "3/16" vs "16/16") keep barXf
    // stationary too:
    {
        ruido::HudState snapA = ctrl.hud();
        snapA.mode = ruido::HudMode::SnapshotSlot;
        snapA.opacity = 1.0f;
        snapA.snapshotGliding = true;
        snapA.morphPosition = 0.20f;
        ruido::HudState snapB = snapA;
        snapB.morphPosition = 1.00f;
        RTEST_CHECK(findHeaderBarLeftX(snapA, 120) == findHeaderBarLeftX(snapB, 120));

        ruido::HudState bpmA = ctrl.hud();
        bpmA.mode = ruido::HudMode::MasterTempo;
        bpmA.opacity = 1.0f;
        bpmA.effectiveClockBpm = 55.5f;
        ruido::HudState bpmB = bpmA;
        bpmB.effectiveClockBpm = 120.0f;
        ruido::HudState bpmC = bpmA;
        bpmC.effectiveClockBpm = 1920.0f;
        const int bpmLeftA = findHeaderBarLeftX(bpmA, 90);
        RTEST_CHECK(bpmLeftA == findHeaderBarLeftX(bpmB, 90));
        RTEST_CHECK(bpmLeftA == findHeaderBarLeftX(bpmC, 90));

        const auto checkPunchStationary = [&](ruido::KeyPunchKind kind, float vLow, float vHigh,
                                              int scanX) {
            ruido::HudState pA = ctrl.hud();
            pA.mode = ruido::HudMode::KeyboardPlay;
            pA.opacity = 1.0f;
            pA.activePunch = kind;
            pA.activePunchValue = vLow;
            ruido::HudState pB = pA;
            pB.activePunchValue = vHigh;
            RTEST_CHECK(findHeaderBarLeftX(pA, scanX) == findHeaderBarLeftX(pB, scanX));
        };
        checkPunchStationary(ruido::KeyPunchKind::VoiceStutter, 0.20f, 1.00f, 125);
        checkPunchStationary(ruido::KeyPunchKind::VoiceCrush, 0.20f, 0.90f, 125);
        checkPunchStationary(ruido::KeyPunchKind::VoiceDivide, 0.20f, 0.90f, 140);
        checkPunchStationary(ruido::KeyPunchKind::VoiceEuclid, 0.20f, 1.00f, 130);
    }

    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ScopeTimebase);
    RTEST_CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Wide2x);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    RTEST_CHECK(ctrl.hud().lastEncoder == ruido::EncoderId::Select);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 16);
    RTEST_CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Wide8x);
    renderer.prepareFrame(0.016f, silentSt, ctrl.hud(), tel);
    renderer.renderStrip(0u, 24u, hdrRateOnly.data());

    // Phantom pin touch on flatline (ENV..REC): deflects CRT trace, keeps 8/16-row strip parity,
    // and settles back to exact flatline after ~0.40 s without waking any HUD mode!
    {
        ruido::ScopeHudRenderer pinRendA;
        ruido::ScopeHudRenderer pinRendB;
        ruido::HudState pinHud{};
        std::array<uint16_t, 240u * 32u> baseBand{};
        std::array<uint16_t, 240u * 32u> rippleBand16{};
        std::array<uint16_t, 240u * 32u> rippleBand8{};
        std::array<uint16_t, 240u * 32u> settledBand{};
        pinRendA.prepareFrame(0.016f, silentSt, pinHud, tel);
        pinRendA.renderStrip(104u, 32u, baseBand.data());

        pinHud.phantomPinIndex = 2u; // SEQ
        pinHud.phantomPinSeq = 1u;
        pinRendA.prepareFrame(0.016f, silentSt, pinHud, tel);
        pinRendB.prepareFrame(0.016f, silentSt, pinHud, tel);
        pinRendA.renderStrip(104u, 16u, rippleBand16.data());
        pinRendA.renderStrip(120u, 16u, rippleBand16.data() + 240u * 16u);
        for (uint16_t s = 0u; s < 4u; ++s) {
            pinRendB.renderStrip(static_cast<uint16_t>(104u + s * 8u), 8u,
                                 rippleBand8.data() + static_cast<size_t>(s) * 240u * 8u);
        }
        size_t rippleDiffs = 0u;
        for (size_t i = 0u; i < baseBand.size(); ++i) {
            RTEST_CHECK(rippleBand16[i] == rippleBand8[i]);
            if (rippleBand16[i] != baseBand[i]) {
                ++rippleDiffs;
            }
        }
        RTEST_CHECK(rippleDiffs > 120u);

        for (int f = 0; f < 24; ++f) {
            pinRendA.prepareFrame(0.020f, silentSt, pinHud, tel);
        }
        pinRendA.renderStrip(104u, 32u, settledBand.data());
        for (size_t i = 0u; i < baseBand.size(); ++i) {
            RTEST_CHECK(settledBand[i] == baseBand[i]);
        }
    }

    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24); // Midpoint morph SCENE 1 -> SCENE 2
    ctrl.tick(0.200f, tel);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::SnapshotSlot);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
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
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24); // +48 total -> 100% pure SCENE 2!
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(!ctrl.hud().snapshotGliding && !ctrl.hud().snapshotDirty);
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

    // Row 1 (Key Sensitivity): MEDIUM (default) <-> VERY HIGH .. VERY LOW, label fits beside value
    static_assert(ruido::kNumGlobalConfigItems == 9u, "GLOBAL CONFIG must have 9 items");
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 1u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::PressureTouch),
                            "Key Sensitivity") == 0);
    RTEST_CHECK(ctrl.hud().config.pressureTouch == ruido::PressureTouch::Medium);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::PressureTouch,
                                                          ctrl.hud().config, ctrl.state().global),
                            "MEDIUM") == 0);
    (void)ctrl.consumeSettingsDirty();
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -40); // Clamped at VERY HIGH
    RTEST_CHECK(ctrl.hud().config.pressureTouch == ruido::PressureTouch::VeryLight);
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    for (const char* lbl : ruido::kPressureTouchLabels) {
        RTEST_CHECK(ruido::measureSansText(ruido::kFontSansMedium, "Key Sensitivity", 0) +
                        ruido::measureSansText(ruido::kFontSansMedium, lbl, 0) + 12 <
                    224 - 16);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 40); // Clamped at VERY LOW
    RTEST_CHECK(ctrl.hud().config.pressureTouch == ruido::PressureTouch::VeryFirm);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::PressureTouch,
                                                          ctrl.hud().config, ctrl.state().global),
                            "VERY LOW") == 0);
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -8); // Back to MEDIUM default
    RTEST_CHECK(ctrl.hud().config.pressureTouch == ruido::PressureTouch::Medium);

    // Key Sensitivity row (Row 1, ry = 44 + 28 = 72) doubles as a touch test above the 6-row fold:
    // holding an analog key keeps the list on screen and draws a hairline meter at y = 72 + 22 = 94
    {
        const auto countMeterPx = [&]() {
            size_t lit = 0u;
            for (size_t y = 93u; y <= 95u; ++y) {
                for (size_t x = 20u; x <= 220u; ++x) {
                    if (((s16[y * 240u + x] >> 5u) & 0x3Fu) >= 40u) {
                        ++lit;
                    }
                }
            }
            return lit;
        };
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        renderer.renderFullScreenStrips(s16.data());
        const size_t idleMeterPx = countMeterPx();
        ctrl.onWhiteKeyGate(0u, true);
        ctrl.onWhiteKeyPressure(0u, 1.0f);
        ctrl.tick(0.016f, ruido::EngineTelemetry{});
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
        for (int f = 0; f < 10; ++f) {
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        renderer.renderFullScreenStrips(s16.data());
        const size_t fullMeterPx = countMeterPx();
        RTEST_CHECK(fullMeterPx > idleMeterPx + 150u);
        ctrl.onWhiteKeyGate(0u, false);
        ctrl.tick(0.016f, ruido::EngineTelemetry{});
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
        for (int f = 0; f < 80; ++f) { // 1.2 s peak hold + 150 ms fade
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        renderer.renderFullScreenStrips(s16.data());
        RTEST_CHECK(countMeterPx() <= idleMeterPx);

        // Touching an on/off key replaces the whole row with "Not pressure sensitive" (flush-left,
        // 134 px), so the right-aligned value column (x = 184..224) goes dark while held.
        RTEST_CHECK(ruido::measureSansText(ruido::kFontSansMedium, "Not pressure sensitive", 0) <
                    184 - 16);
        const auto valueAreaLit = [&]() {
            size_t lit = 0u;
            for (size_t y = 76u; y <= 91u; ++y) {
                for (size_t x = 184u; x <= 224u; ++x) {
                    if (((s16[y * 240u + x] >> 5u) & 0x3Fu) >= 30u) {
                        ++lit;
                    }
                }
            }
            return lit;
        };
        renderer.renderFullScreenStrips(s16.data());
        const size_t normalValuePx = valueAreaLit();
        ctrl.onWhiteKeyGate(12u, true); // Key 13 = MUTE 1 (digital-only PA6)
        ctrl.tick(0.016f, ruido::EngineTelemetry{});
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::GlobalConfig);
        for (int f = 0; f < 4; ++f) {
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        renderer.renderFullScreenStrips(s16.data());
        RTEST_CHECK(normalValuePx > 20u);
        RTEST_CHECK(valueAreaLit() + 20u < normalValuePx); // value replaced by the notice
        ctrl.onWhiteKeyGate(12u, false);
        ctrl.tick(0.016f, ruido::EngineTelemetry{});
        for (int f = 0; f < 60; ++f) {
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        renderer.renderFullScreenStrips(s16.data());
        RTEST_CHECK(valueAreaLit() + 4u >= normalValuePx); // "MEDIUM" value restored
    }

    // Row 2 (Mute Mode): MOMENTARY (default) <-> LATCHING via SELECT/ALGORITHM (single-state MUTE +
    // SHIFT + SOLO; separate SoloMode removed -> 9 rows total: 0..8)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 2u);
    RTEST_CHECK(
        std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MuteMode), "Mute Mode") == 0);
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

    // Row 3 (Scene Mode): LATCHING (default) <-> MOMENTARY via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 3u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::SnapMode),
                            "Scene Mode") == 0);
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

    // Row 4 (MIDI Clock In): AUTO (default) <-> OFF via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 4u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::ClockSync),
                            "MIDI Clock In") == 0);
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

    // Row 5 (MIDI Transport In): OFF (default) -> PHASE -> RUN/STOP via SELECT/ALGORITHM
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 5u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MidiTransport),
                            "MIDI Transport In") == 0);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::Off);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiTransport,
                                                          ctrl.hud().config, ctrl.state().global),
                            "OFF") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::PhaseOnly);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiTransport,
                                                          ctrl.hud().config, ctrl.state().global),
                            "PHASE") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::RunStop);
    RTEST_CHECK(std::strcmp(ruido::globalConfigValueLabel(ruido::ConfigItemId::MidiTransport,
                                                          ctrl.hud().config, ctrl.state().global),
                            "RUN/STOP") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -8);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::Off);

    // Row 6 (MIDI Channel In): OMNI (default) <-> CH 1..CH 16 + scrolled 6-row viewport check
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 6u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::MidiChannel),
                            "MIDI Channel In") == 0);
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

    // Rows 7..8: Operational sub-screens (Factory Reset, Ruido)
    // 7a. FACTORY RESET requires 2 CW detents (8 ticks), cancels on CCW, and resets NVS + patch
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    RTEST_CHECK(ctrl.hud().configCursor == 7u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::FactoryReset),
                            "Factory Reset") == 0);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::FactoryResetConfirm);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // 1 detent (50% progress) -> NOT reset yet
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::FactoryResetConfirm);
    RTEST_CHECK(ctrl.hud().configResetTicks == 4u);
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    {
        // Inline row hairline at y = 66 (44 + 22) is lit on left half (50%) and dim on right half
        uint32_t leftLumSum = 0u;
        uint32_t rightLumSum = 0u;
        for (size_t x = 20u; x < 112u; ++x) {
            leftLumSum += (s16[66u * 240u + x] & 0x1Fu);
        }
        for (size_t x = 128u; x < 220u; ++x) {
            rightLumSum += (s16[66u * 240u + x] & 0x1Fu);
        }
        RTEST_CHECK(leftLumSum > rightLumSum * 3u);
        RTEST_CHECK(s16[120u * 240u + 4u] == 0u); // Zero background scope flatline bleed
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -8); // CCW cancels back to list safely
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::List);

    // Mutate config + delete Scene 1, then run full 2-detent FACTORY RESET confirmation
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // DELETE SCENE
    ctrl.onWhiteKeyGate(8u, true);
    ctrl.onWhiteKeyGate(8u, false);
    RTEST_CHECK(!ctrl.snapshots().isSlotOccupied(0u));
    (void)ctrl.consumeSavedSlotMask();
    (void)ctrl.consumeSettingsDirty();

    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Back to row 7 if needed (preserved at 7)
    RTEST_CHECK(ctrl.hud().configCursor >= 7u);
    if (ctrl.hud().configCursor != 7u) {
        ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    }
    RTEST_CHECK(ctrl.hud().configCursor == 7u);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // Enter confirm
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8); // 2 CW detents -> confirm reset!
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::FactoryResetDone);
    RTEST_CHECK(ctrl.snapshots().occupiedMask() == 0x0Fu);
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == 0x0Fu);
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::List);

    // 7a-bis. CLEAN INIT selector inside Factory Reset: SELECT switches mode (resetting progress),
    //         restores original startup sound (55/82.41/110.15/164.81 Hz, 50% duty, open filters,
    //         0% LFOs), clears Scenes 1-4 to EMPTY, and preserves GLOBAL CONFIG (Output Routing,
    //         MIDI Channel, Key Sensitivity)!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -28); // Row 0: Output Routing
    RTEST_CHECK(ctrl.hud().configCursor == 0u);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // Set Output Routing = MONO
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 24); // Row 6: MIDI Channel In
    RTEST_CHECK(ctrl.hud().configCursor == 6u);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 12); // Set MIDI Channel = CH 3
    RTEST_CHECK(ctrl.hud().config.midiChannel == 3u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Row 7: Factory Reset
    RTEST_CHECK(ctrl.hud().configCursor == 7u);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // Enter Factory Reset
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::FactoryResetConfirm);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // Half-arm (4 ticks = 50%)
    RTEST_CHECK(ctrl.hud().configResetTicks == 4u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // Switch to CLEAN INIT -> resets progress!
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::CleanInitConfirm);
    RTEST_CHECK(ctrl.hud().configResetTicks == 0u);
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    (void)ctrl.consumeSavedSlotMask();
    (void)ctrl.consumeSettingsDirty();
    (void)ctrl.consumeLivePatchDirty();
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8); // 2 CW detents -> confirm CLEAN INIT!
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::CleanInitDone);
    RTEST_CHECK(ctrl.snapshots().occupiedMask() == 0x00u);
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == 0x00u);
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == 0x0Fu);
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    RTEST_CHECK(ctrl.consumeLivePatchDirty());
    RTEST_CHECK(ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.hud().config.midiChannel == 3u);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, 55.0f, 0.05f));
    RTEST_CHECK(nearF(ctrl.state().voices[1].frequencyHz, 82.41f, 0.08f));
    RTEST_CHECK(nearF(ctrl.state().voices[2].frequencyHz, 110.15f, 0.08f));
    RTEST_CHECK(nearF(ctrl.state().voices[3].frequencyHz, 164.81f, 0.12f));
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().voices[v].pulseWidth, 0.50f, 1.0e-3f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].hpCutoffHz, ruido::kFilterCutoffMinHz, 0.05f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].lpCutoffHz, ruido::kFilterCutoffMaxHz, 10.0f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].hpResonance, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].lpResonance, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].lfoDepth, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].lfoBDepth, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().voices[v].amplitude, 0.65f, 1.0e-3f));
    }
    RTEST_CHECK(nearF(ctrl.state().global.circuitCoupling, 0.25f, 1.0e-3f));
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    // Verify NVS cold-reboot round-trip preserves empty Scenes 1-4 + startup sound + Global Config!
    {
        ruido::RuidoController rebootCtrl;
        for (uint8_t s = 0u; s < 4u; ++s) {
            RTEST_CHECK(rebootCtrl.loadSnapshotSlot(s, ctrl.snapshots().slotBinary(s)));
        }
        RTEST_CHECK(rebootCtrl.loadSettings(ctrl.exportSettings()));
        RTEST_CHECK(rebootCtrl.loadLivePatch(ctrl.exportLivePatch()));
        RTEST_CHECK(rebootCtrl.snapshots().occupiedMask() == 0x00u);
        RTEST_CHECK(rebootCtrl.state().global.monoCollapse);
        RTEST_CHECK(rebootCtrl.hud().config.midiChannel == 3u);
        RTEST_CHECK(nearF(rebootCtrl.state().voices[0].frequencyHz, 55.0f, 0.05f));
        RTEST_CHECK(nearF(rebootCtrl.state().voices[3].frequencyHz, 164.81f, 0.12f));
    }
    // Restore Factory Defaults cleanly before proceeding to Credits test
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // CCW BACK: Exit CleanInitDone -> List
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::List);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // Enter Factory Reset
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8); // Confirm Factory Presets
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::FactoryResetDone);
    RTEST_CHECK(ctrl.snapshots().occupiedMask() == 0x0Fu);
    RTEST_CHECK(!ctrl.state().global.monoCollapse);
    RTEST_CHECK(ctrl.hud().config.midiChannel == 0u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::List);

    // 7b. ABOUT credits (last row = 8): clamped at bottom, UTF-8 'á' acute width,
    //     flush-left x=10 alignment across header wordmark & body rows, and monochrome render
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 16); // Clamped at last row (8)
    RTEST_CHECK(ctrl.hud().configCursor == 8u);
    RTEST_CHECK(std::strcmp(ruido::globalConfigItemLabel(ruido::ConfigItemId::Credits), "About") ==
                0);
    RTEST_CHECK(std::strcmp(ruido::kRuidoVersionStr, "0.7.0") == 0);
    RTEST_CHECK(ruido::measureSansText(ruido::kFontSansMedium, "Hern\xC3\xA1ndez", 0) ==
                ruido::measureSansText(ruido::kFontSansMedium, "Hernandez", 0));
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    RTEST_CHECK(ctrl.hud().configSubView == ruido::ConfigSubView::Credits);
    renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    verifyViewStripsAndCenter(false);
    {
        static std::array<uint16_t, 240u * 240u> creditsFrame{};
        renderer.renderFullScreenStrips(creditsFrame.data());
        const auto firstLitXInBand = [&](size_t yStart, size_t yEnd) -> int {
            int minX = 240;
            for (size_t y = yStart; y <= yEnd; ++y) {
                for (int x = 0; x < 40; ++x) {
                    if (creditsFrame[y * 240u + static_cast<size_t>(x)] != 0u && x < minX) {
                        minX = x;
                    }
                }
            }
            return minX;
        };
        RTEST_CHECK(firstLitXInBand(5u, 22u) == 10);    // RUIDO header wordmark
        RTEST_CHECK(firstLitXInBand(46u, 60u) == 16);   // 4-Oscillator Drone Synthesizer
        RTEST_CHECK(firstLitXInBand(78u, 92u) == 16);   // By Miguel Hernández Domenech
        RTEST_CHECK(firstLitXInBand(102u, 116u) == 16); // GPL-3.0  /  2026  /  Dinghy HAL
        RTEST_CHECK(firstLitXInBand(126u, 140u) == 16); // github.com/miguelhd/ruido
        RTEST_CHECK(firstLitXInBand(150u, 164u) == 240);
        RTEST_CHECK(creditsFrame[120u * 240u + 4u] == 0u);
    }

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

    // 2. HOME 4-Step Oscillator Core Cycle (Clean -> FREQUENCY -> DUTY -> DRIFT -> Clean) & SELECT:
    //    1st HOME tap from EDIT escapes to Clean Scope; next HOME taps step FREQUENCY(0) -> DUTY(1)
    //    -> DRIFT(2) -> Clean Scope!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().freqPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Frequency &&
                ctrl.hud().mode == ruido::HudMode::VoiceFrequency);

    ctrl.onButtonEvent(ruido::ButtonId::Home, true); // 2nd HOME tap -> DUTY(1)
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Width);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::PulseWidth);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::PulseWidth);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -80);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 80);
    RTEST_CHECK(ctrl.state().voices[0].pulseWidth < 0.25f);
    RTEST_CHECK(ctrl.state().voices[3].pulseWidth > 0.75f);

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true); // Gear-invariant Fine detent on HOME
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);    // DUTY(1) -> DRIFT(2)
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    RTEST_CHECK(ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Drift);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceDrift);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceDrift);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -80); // Lock Osc 1 to 0% drift
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 120); // Boost Osc 2 thermal drift
    RTEST_CHECK(nearF(ctrl.state().voices[0].drift, 0.0f));
    RTEST_CHECK(ctrl.state().voices[1].drift > 0.55f);

    // 4th HOME press from DRIFT(2) returns to Clean Scope (freqPageOpen == false)!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(!ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Frequency);

    // Pure 2-mode EDIT Console Mixer: toggles LEVEL <-> PAN and remembers PAN on HOME close!
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 60);
    RTEST_CHECK(ctrl.state().voices[2].pan > -0.20f);

    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    RTEST_CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
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
                            "Output Routing") == 0);
    {
        ruido::EngineControlState lfoColSt = quietSt;
        lfoColSt.voices[0].lfoBTarget = ruido::LfoTarget::LfoARate;
        lfoColSt.voices[3].lfoBTarget = ruido::LfoTarget::LfoADepth;
        ruido::HudState lfoColHud{};
        lfoColHud.mode = ruido::HudMode::VoiceLfo;
        lfoColHud.knobBank = ruido::KnobBankMode::Lfo;
        lfoColHud.lfoPageOpen = true;
        lfoColHud.opacity = 1.0f;
        ruido::LfoSignalArray zeroLfos{};
        char rBuf[12]{};
        char vBuf[16]{};
        ruido::GlobalColumnSummary gs{};

        lfoColHud.lfoVoiceFocus = 1u; // LFO B1 -> "A1 RATE"
        ruido::formatColumnRoleAndValue(lfoColSt, lfoColHud, zeroLfos, 0x0Fu, 0u, rBuf,
                                        sizeof(rBuf), vBuf, sizeof(vBuf), gs);
        RTEST_CHECK(std::strcmp(vBuf, "A1 RATE") == 0);

        lfoColHud.lfoVoiceFocus = 7u; // LFO B4 -> "A4 DEPTH" (fits cleanly inside 60 px Col 0!)
        std::memset(rBuf, 0, sizeof(rBuf));
        std::memset(vBuf, 0, sizeof(vBuf));
        ruido::formatColumnRoleAndValue(lfoColSt, lfoColHud, zeroLfos, 0x0Fu, 0u, rBuf,
                                        sizeof(rBuf), vBuf, sizeof(vBuf), gs);
        RTEST_CHECK(std::strcmp(vBuf, "A4 DEPTH") == 0);
    }

    // Verify 240x8 == 240x16 == 240x24 strip equivalence & monochrome neutrality across HOME
    // (DUTY · DRIFT) and EDIT (LEVEL · PAN):
    driveTel.voiceDriftNorm[2].store(0.35f, std::memory_order_relaxed);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true); // Escape EDIT -> Clean Scope
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true); // 1st -> FREQUENCY
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true); // 2nd -> DUTY
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int em = 0; em < 4; ++em) {
        if (em == 1) {
            ctrl.onButtonEvent(ruido::ButtonId::Home, true); // 3rd -> DRIFT
            ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        } else if (em == 2 || em == 3) {
            ctrl.onButtonEvent(ruido::ButtonId::Edit, true); // em=2 -> LEVEL, em=3 -> PAN
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
    }

    // Close EDIT PAN via HOME -> smooth 150 ms fade-out & EDIT LED returns to 15%!
    RTEST_CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    ctrl.tick(0.200f, driveTel);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 1.0f));
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
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
    const float ledSubA = ctrl.leds().whiteKeyLeds[12];
    renderer.renderFullScreenStrips(amp16.data());
    RTEST_CHECK(maxOscLabelG6(amp16, 0u) <= 22u);
    RTEST_CHECK(maxOscLabelG6(amp16, 2u) >= 35u);

    for (int f = 0; f < 20; ++f) {
        ctrl.tick(0.016f, driveTel);
        renderer.prepareFrame(0.016f, quietSt, ctrl.hud(), driveTel);
    }
    const float ledSubB = ctrl.leds().whiteKeyLeds[12];
    RTEST_CHECK(nearF(ledSubA, 0.50f) && nearF(ledSubB, 0.50f));
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

    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
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
        testHud.editPageOpen = (sc.mode == ruido::HudMode::Amplitude);
        testHud.freqPageOpen = !testHud.panPageOpen && !testHud.editPageOpen;
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
        ruido::EngineTelemetry modTel{};
        // Verify first-load boot patch (Scene 1) showcases active modifiers across all 4 voices:
        ctrl.tick(0.05f, modTel);
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            const auto& vp = ctrl.state().voices[v];
            RTEST_CHECK(std::fabs(vp.pulseWidth - 0.50f) > 0.05f);
            RTEST_CHECK(vp.drift >= 0.09f);
            RTEST_CHECK(std::fabs(vp.pan) >= 0.15f);
            RTEST_CHECK(vp.lfoDepth >= 0.20f && vp.lfoBDepth >= 0.20f);
        }
        RTEST_CHECK(ctrl.state().voices[0].lpCutoffHz <= 1200.0f &&
                    ctrl.state().voices[0].lpResonance >= 0.35f);
        RTEST_CHECK(ctrl.state().voices[1].lpCutoffHz <= 2000.0f &&
                    ctrl.state().voices[1].lpResonance >= 0.40f);
        RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.15f &&
                    ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 0.375f);
        RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)] > 0.15f &&
                    ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)] <= 0.375f);

        // Load clean unmodulated baseline to isolate 1x1 placeholder -> single FX/LFO promotion:
        RTEST_CHECK(ctrl.loadLivePatch(
            ruido::encodePatchSnapshot(ruido::EngineControlState::makeDefault())));
        // Reset scope history so background behind bottom HUD strip is pure pitch-black (#000000)
        // while keeping masterRun = true so live FX/LFO modulator LEDs & dots animate!
        renderer.reset();
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
        //    at y = 230 (`cx - 8`, `cx + 2`, `cx + 8`) with zero AA fringe and zero idle pulsing
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
                RTEST_CHECK(g >= 7u && g <= 11u);
                RTEST_CHECK(neighborSum(amp8, sx, kModY) == 0u);
            }
        }
        bool dryZeroDiff = true;
        for (int y = 226; y < 240; ++y) {
            for (int x = 0; x < 240; ++x) {
                if (pxAt(amp8, x, y) != pxAt(amp16, x, y)) {
                    dryZeroDiff = false;
                }
            }
        }
        RTEST_CHECK(dryZeroDiff);

        // Shared component across `EDIT -> LEVEL` and `EDIT -> PAN` (stereo), while unlatched
        // `KNOB 1` turns (`freqPageOpen == false`) and keybed punch-ins (`MUTE` / `SOLO`) hide it:
        ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
        ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
        RTEST_CHECK(ctrl.hud().editPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) >= 7u && g6At(clean16, 32, kModY) >= 7u &&
                    g6At(clean16, 38, kModY) >= 7u);

        ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
        ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
        RTEST_CHECK(ctrl.hud().panPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) >= 7u && g6At(clean16, 32, kModY) >= 7u &&
                    g6At(clean16, 38, kModY) >= 7u);

        // Close EDIT -> idle -> turn KNOB 1 (unlatched VoiceFrequency toast, freqPageOpen == false)
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
        RTEST_CHECK(!ctrl.hud().freqPageOpen &&
                    ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) == 0u);
        RTEST_CHECK(g6At(clean16, 32, kModY) == 0u);
        RTEST_CHECK(g6At(clean16, 38, kModY) == 0u);

        // Dismiss toast -> Return to latched HOME (`FREQUENCY`)
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

        // Pressing MUTE 1 (white 12) or SOLO 1 — even while HOME is latched open — hides the
        // modulator dots on the MUTE/SOLO screen and restores them upon key release:
        ctrl.onWhiteKeyPressure(12u, 0.80f); // MUTE 1
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) == 0u && g6At(clean16, 32, kModY) == 0u &&
                    g6At(clean16, 38, kModY) == 0u);
        ctrl.onWhiteKeyGate(12u, false);

        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onWhiteKeyGate(12u, true); // SHIFT + Key 13 = SOLO 1
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) == 0u && g6At(clean16, 32, kModY) == 0u &&
                    g6At(clean16, 38, kModY) == 0u);
        ctrl.onWhiteKeyGate(12u, false);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(clean16.data());
        RTEST_CHECK(g6At(clean16, 22, kModY) >= 7u);

        // 2. Dial OSC 1 LPF below 28 kHz with 0% LFO depth -> FX button LED breathes subtly, while
        //    OSC 1's left (`FX`) dot holds a steady, clear per-voice brightness (no fake sine):
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
            RTEST_CHECK(fxLed > 0.15f && fxLed <= 0.375f);
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
            RTEST_CHECK(cG6 >= 24u);
            RTEST_CHECK(neighborSum(amp24, 22, kModY) >= 80u);
            // OSC 1 LFO A/B and OSC 2-4 all remain crisp 1x1 inactive dots
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
        RTEST_CHECK(minFxG6 == maxFxG6); // Static filter = steady dot (zero fake breathing)

        // 3. Target-matched LFO dots: when `OSC 1 LFO A` (`slot 0`) & `OSC 2 LFO B` (`slot 3`)
        //    have `TARGET = LPF` (default), they do NOT animate on `HOME -> FREQUENCY`, `LEVEL`,
        //    or `PAN`; setting `TARGET = FREQ` animates them strictly on `HOME -> FREQUENCY`!
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
        ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 120); // OSC 1 LFO A depth > 50% (TARGET = LPF)
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 12); // slot 0 -> slot 3 (`LFO B OSC 2`)
        ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 120); // OSC 2 LFO B depth > 50% (TARGET = LPF)
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 12; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(ctrl.hud().freqPageOpen);
        modTel.lfoOutput[0].store(1.0f, std::memory_order_relaxed);
        modTel.lfoOutput[5].store(1.0f, std::memory_order_relaxed);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(amp8.data());
        // With TARGET = LPF, LFO dots on FREQUENCY stay calm 1x1 rest dots (`neighborSum == 0`)!
        RTEST_CHECK(neighborSum(amp8, 32, kModY) == 0u);
        RTEST_CHECK(neighborSum(amp8, 98, kModY) == 0u);

        // Point both `OSC 1 LFO A` and `OSC 2 LFO B` at `FREQ` (`LfoTarget::Pitch` = -2 detents):
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);    // Opens remembered slot 3 (`LFO B2`)
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -8);   // `LPF` (2) -> `FREQ` (0)
        ctrl.onEncoderDelta(ruido::EncoderId::Select, -12); // slot 3 -> slot 0 (`LFO A1`)
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -8);   // `LPF` (2) -> `FREQ` (0)
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);  // restore focus to slot 3 (`LFO B2`)
        RTEST_CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::Pitch &&
                    ctrl.state().voices[1].lfoBTarget == ruido::LfoTarget::Pitch);

        // On `EDIT -> PAN` and `EDIT -> LEVEL`, `TARGET = FREQ` LFO dots stay calm 1x1 rest dots!
        ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
        ctrl.onButtonEvent(ruido::ButtonId::Edit, false); // Re-opens remembered `PAN`
        RTEST_CHECK(ctrl.hud().panPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(amp8.data());
        RTEST_CHECK(neighborSum(amp8, 32, kModY) == 0u && neighborSum(amp8, 98, kModY) == 0u);

        ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
        ctrl.onButtonEvent(ruido::ButtonId::Edit, false); // Toggles `PAN` -> `LEVEL`
        RTEST_CHECK(ctrl.hud().editPageOpen);
        for (int f = 0; f < 16; ++f) {
            ctrl.tick(0.02f, modTel);
            renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        }
        renderer.renderFullScreenStrips(amp8.data());
        RTEST_CHECK(neighborSum(amp8, 32, kModY) == 0u && neighborSum(amp8, 98, kModY) == 0u);

        // Return to `HOME -> FREQUENCY` -> now `TARGET = FREQ` dots animate independently!
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
        RTEST_CHECK(ctrl.state().voices[0].lfoDepth > 0.45f);
        RTEST_CHECK(ctrl.state().voices[1].lfoDepth == 0.0f);
        RTEST_CHECK(ctrl.state().voices[1].lfoBDepth > 0.45f);

        modTel.lfoOutput[0].store(1.0f, std::memory_order_relaxed);  // OSC 1 LFO A at +1.0 crest
        modTel.lfoOutput[5].store(-1.0f, std::memory_order_relaxed); // OSC 2 LFO B at -1.0 trough
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(amp8.data());
        const float lfoLedLo = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];
        RTEST_CHECK(g6At(amp8, 32, kModY) > g6At(amp8, 98, kModY) + 18u);

        modTel.lfoOutput[0].store(-1.0f, std::memory_order_relaxed); // OSC 1 LFO A at -1.0 trough
        modTel.lfoOutput[5].store(1.0f, std::memory_order_relaxed);  // OSC 2 LFO B at +1.0 crest
        ctrl.tick(0.02f, modTel);
        renderer.prepareFrame(0.02f, ctrl.state(), ctrl.hud(), modTel);
        renderer.renderFullScreenStrips(amp16.data());
        const float lfoLedHi = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

        RTEST_CHECK(lfoLedLo >= 0.15f && lfoLedHi <= 0.375f);
        RTEST_CHECK(lfoLedHi > lfoLedLo + 0.12f);
        RTEST_CHECK(g6At(amp16, 98, kModY) > g6At(amp8, 98, kModY) + 18u);
        RTEST_CHECK(g6At(amp16, 98, kModY) >= 44u);
        RTEST_CHECK(neighborSum(amp16, 98, kModY) >= 80u);
        // Sibling `LFO A` on OSC 2 (`92, 230`) stays a crisp 1x1 placeholder dot
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

    // Turning KNOB 1 while SEL is open adjusts OSC 1 Frequency as a 1.5 s header-only toast
    // (`knobBank == None` so SEL diagram + bottom inspector stay intact), then returns to SEL!
    const float hz0Before = ctrl.state().voices[0].frequencyHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 8);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz > hz0Before);
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
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

    // Pressing any key while SEL is open keeps SEL pinned on screen (`HudMode::KeyboardModel`,
    // `knobBank == None`) while populating `activePunch` for the bottom Live Guidance Inspector!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.70f);
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
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

    // Repeated SEL taps wrap 0 -> 1 -> 2 -> 0; HOME dissolves in 150 ms while keeping model active
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().selPageOpen &&
                ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().selPageOpen &&
                ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().selPageOpen &&
                ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
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
        RTEST_CHECK(nearF(ctrl.state().keyMod.masterHpfSweep, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.ringModMix, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.hardSyncAmount, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.lfoTurbo, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.filterScream, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmPingPong, 0.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmRoll, 0.0f));
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

    // Universal SHIFT + Keys 13..16 (`White 12..15`) Immediate Console Solo -> SOLO_v = 80% max
    // (keeping SHIFT at 100% brightest), silenced peers = 0%, and SHIFT + OP1..OP3 / SELECT!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    for (uint8_t v = 0u; v < 4u; ++v) {
        const auto wKey = static_cast<uint8_t>(ruido::kMuteWhiteKeyBase + v);
        const size_t peer = (v + 1u) & 3u;
        ctrl.onWhiteKeyGate(wKey, true);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().shiftHeld);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::SpotlightSolo);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 1.0f));
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[peer], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[wKey], 0.80f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kMuteWhiteKeyBase + peer], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kShiftBlackKeyIdx], 1.0f));
        ctrl.onWhiteKeyGate(wKey, false);
        ctrl.tick(0.016f, tel);
    }
    // Direct Keyboard Select while holding SHIFT (`SHIFT + OP2` -> CIRCUIT LAB, `OP3` -> RHYTHM
    // LAB, `OP1` -> LIVE MIXER) + `SHIFT + SELECT` stays on `ShiftMap` & fires the 220 ms LED wave!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, true);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    RTEST_CHECK(ctrl.hud().modeWipeQ8 > 220u); // 220 ms wave active immediately on key select!
    RTEST_CHECK(ctrl.hud().shiftFocus == ruido::ShiftFocus::Model && !ctrl.hud().selPageOpen &&
                ctrl.hud().mode == ruido::HudMode::ShiftMap);
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[1], 1.0f));
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "KEYBOARD") == 0 && std::strcmp(ls, " 2/3") == 0 &&
                    std::strcmp(rt, "CIRCUIT LAB") == 0);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);
    ctrl.tick(0.016f, tel);
    const float op2BlinkA = ctrl.leds().blackKeyLeds[1]; // Immediately at ~100% crest
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.21f, tel); // +1.26 s (~half of 2.50 s period, wipe settled!) -> near 15% trough
    }
    const float op2BlinkB = ctrl.leds().blackKeyLeds[1];
    RTEST_CHECK(ctrl.hud().modeWipeQ8 == 0u);
    RTEST_CHECK(op2BlinkA > 0.95f && op2BlinkB < 0.20f && (op2BlinkA - op2BlinkB) > 0.70f);
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[0], 0.15f) &&
                nearF(ctrl.leds().blackKeyLeds[2], 0.15f));
    // Twisting SELECT while holding SHIFT stays 100% on `HudMode::ShiftMap` AND fires the exact
    // same 220 ms keyboard-change LED wave across the keybed before settling back to SHIFT LEDs!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> RhythmLab on ShiftMap!
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().shiftHeld &&
                ctrl.hud().shiftFocus == ruido::ShiftFocus::Model);
    RTEST_CHECK(ctrl.hud().modeWipeQ8 > 220u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -8); // -> LiveMixer on ShiftMap!
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer &&
                ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().modeWipeQ8 > 220u);
    {
        ruido::RuidoController waveCtrl{};
        waveCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        waveCtrl.onEncoderDelta(ruido::EncoderId::Select, 4);
        waveCtrl.tick(0.080f, tel); // Mid-wave: ghosted keys light up with kinetic wave!
        RTEST_CHECK(
            waveCtrl.hud().mode == ruido::HudMode::ShiftMap &&
            (waveCtrl.leds().whiteKeyLeds[2] > 0.15f || waveCtrl.leds().whiteKeyLeds[5] > 0.15f));
        for (int f = 0; f < 5; ++f) {
            waveCtrl.tick(0.050f, tel); // Settle back onto SHIFT rest LEDs
        }
        RTEST_CHECK(waveCtrl.hud().modeWipeQ8 == 0u &&
                    nearF(waveCtrl.leds().whiteKeyLeds[0], 0.0f));
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab &&
                ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().modeWipeQ8 == 255u);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    for (int f = 0; f < 5; ++f) {
        ctrl.tick(0.050f, tel); // Let 150 ms ShiftMap release dissolve cleanly
    }
    verifyRestored();

    // TAP TEMPO (`POLY`, Black 9) is 0% unassigned and clock-inert in LIVE MIXER, firing the
    // unified unassigned feedback (CRT ripple + 120 ms 15% glint -> 0% dark, zero HUD text):
    {
        const uint8_t seqBeforeInertTap = ctrl.state().global.clockResetSeq;
        const uint8_t phantomBeforeTap = ctrl.hud().phantomPinSeq;
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kTapBlackKeyIdx], 0.0f));
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
        RTEST_CHECK(ctrl.hud().phantomPinSeq == static_cast<uint8_t>(phantomBeforeTap + 1u));
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::None);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::None);
        RTEST_CHECK(ctrl.state().global.clockResetSeq == seqBeforeInertTap);
        RTEST_CHECK(ctrl.leds().blackKeyLeds[ruido::kTapBlackKeyIdx] > 0.11f &&
                    ctrl.leds().blackKeyLeds[ruido::kTapBlackKeyIdx] <= 0.15f);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
        ctrl.tick(0.130f, tel);
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kTapBlackKeyIdx], 0.0f));
    }

    // OP1 Drive -> native Amplitude view + full-device VU bar + red REC clip beacon (MUTE 1-4 stay
    // steady 100% audible — never dancing with OP1 plasma flood)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.92f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    RTEST_CHECK(nearF(ctrl.state().keyMod.overloadSurge, 0.92f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12], 0.80f) && ctrl.leds().whiteKeyLeds[0] > 0.70f &&
                ctrl.leds().whiteKeyLeds[11] > 0.70f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)] > 0.65f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)], 0.0f));
    verifyRestored();

    // OP2 Sub Octave (continuous 0.0 -> -2.0 Oct dive across [0, 1] + OCT- beacon)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 0.25f);
    const float sub25 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 0.50f);
    const float sub50 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 0.75f);
    const float sub75 = ctrl.state().keyMod.voicePitchMul[0];
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 1.00f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::SubDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(sub25 < 0.90f && sub25 > sub50);
    RTEST_CHECK(nearF(sub50, 0.50f, 1.0e-3f));
    RTEST_CHECK(sub75 < sub50 && sub75 > 0.28f);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[0], 0.25f, 1.0e-3f));
    RTEST_CHECK(
        nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // OP3 Ratio Lock (pulls detuned Osc 2 = 131.06 Hz onto pure 2/1 = 130.82 Hz relative to 65.41
    // Hz)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op3, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RatioMagnet);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    const float lockedHz1 =
        ctrl.state().voices[1].frequencyHz * ctrl.state().keyMod.voicePitchMul[1];
    RTEST_CHECK(nearF(lockedHz1, 130.82f, 0.05f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);
    verifyRestored();

    // OP4 Low Pass -> native 4-voice GlobalFilter MS-20 passband curve + right-to-left curtain LEDs
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.65f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12], 0.80f) &&
                ctrl.leds().whiteKeyLeds[3] > ctrl.leds().whiteKeyLeds[11] + 0.60f);
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.85f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.dubLpfDrop, 0.85f));
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)] >
                ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)] + 0.70f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // OP5 (HIGH PASS: sweeps resonant MS-20 HPF bass cut from 20 Hz up to ~3.2 kHz!)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::HighPass);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.masterHpfSweep, 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // OP6 (UNISON: collapses Osc 2-4 pitches onto Osc 1 ~55 Hz + centers pan, undimmed scope!)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.0f);
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
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    verifyRestored();

    // PIT (DETUNE) & GLO (SPREAD)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DetuneChorus);
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[0] < 0.98f);
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[3] > 1.06f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);
    verifyRestored();

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::PolySpread);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "SPREAD") == 0);
        RTEST_CHECK(std::strcmp(rt, "100%") == 0 && sb && nearF(bn, 1.0f));
    }
    for (const auto& spec : ruido::kKeyboardModelSpecs) {
        for (const char* tz : spec.topZones) {
            RTEST_CHECK(std::strcmp(tz, "FREEZE") != 0);
        }
    }
    RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    verifyRestored();

    // Permanent Right-Bay MONO = HOLD: latches held analog pressure hands-free and blinks at 2.5 Hz
    // until tapped again (while lighting 100% while physically pressed)!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.80f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx], 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false); // Finger lifts off OP4!
    float latchLedMin = 1.0f;
    float latchLedMax = 0.0f;
    for (int step = 0; step < 26; ++step) {
        ctrl.tick(0.016f, tel);
        const float lv = ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx];
        if (lv < latchLedMin) {
            latchLedMin = lv;
        }
        if (lv > latchLedMax) {
            latchLedMax = lv;
        }
    }
    RTEST_CHECK(ctrl.hud().keyHoldLatched && ctrl.hud().keyHoldCount == 1u);
    RTEST_CHECK(nearF(ctrl.state().keyMod.dubLpfDrop, 0.80f));
    RTEST_CHECK(latchLedMin < 0.15f && latchLedMax > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false); // Release HOLD!
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(!ctrl.hud().keyHoldLatched && ctrl.hud().keyHoldCount == 0u);
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx], 0.15f));
    verifyRestored();

    // Tapping MONO (LATCH) with 0 held keys arms LATCH and blinks identically at 2.5 Hz!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    float armedMin = 1.0f;
    float armedMax = 0.0f;
    for (int step = 0; step < 26; ++step) {
        ctrl.tick(0.016f, tel);
        const float lv = ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx];
        if (lv < armedMin) {
            armedMin = lv;
        }
        if (lv > armedMax) {
            armedMax = lv;
        }
    }
    RTEST_CHECK(ctrl.hud().keyHoldLatched && ctrl.hud().keyHoldCount == 0u);
    RTEST_CHECK(armedMin < 0.15f && armedMax > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(!ctrl.hud().keyHoldLatched);
    verifyRestored();

    // Bottom Keys on LIVE MIXER: Filter (0..3), Octave (4..7), Duty (8..11), Mute (12..15)
    for (uint8_t v = 0u; v < 4u; ++v) {
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(ruido::kMuteWhiteKeyBase + v), 0.20f);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceCut);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 0.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kMuteWhiteKeyBase + v], 0.0f));
        RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 0.375f);
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(ruido::kMuteWhiteKeyBase + v), false);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kMuteWhiteKeyBase + v], 0.80f));

        ctrl.onWhiteKeyPressure(v, 0.80f);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceFormant);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
        RTEST_CHECK(ctrl.hud().focusedVoice == v);
        RTEST_CHECK(nearF(ctrl.state().voicePressure[v], 0.80f));
        ctrl.onWhiteKeyGate(v, false);

        // OCTAVE v (Keys 4..7): continuous +0.0 -> +2.0 Oct siren climb (+1 Oct at 0.50, +2 at 1)
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(4u + v), 0.50f);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceSiren);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[v], 2.0f, 1.0e-3f));
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(4u + v), 1.00f);
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(nearF(ctrl.state().keyMod.voicePitchMul[v], 4.0f, 1.0e-3f));
        RTEST_CHECK(ctrl.state().voicePressure[v] > 0.65f);
        RTEST_CHECK(
            nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 1.0f));
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(4u + v), false);
        ctrl.tick(0.016f, tel);

        // DUTY v (Keys 8..11): continuous Schmitt PWM duty narrow morph + PulseWidth visualizer
        ctrl.onWhiteKeyPressure(static_cast<uint8_t>(8u + v), 1.0f);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceDuty);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::PulseWidth);
        RTEST_CHECK(std::fabs(ctrl.state().keyMod.voicePwDelta[v]) > 0.35f);
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

    const auto checkStripAndMono = [&](size_t minLitPx = 400u) {
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
        RTEST_CHECK(litPx >= minLitPx);
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
            // Two-Bay rounded stadium corners + dark 4px gap between Left 20-Pad Bay & Right
            // 7-Switch Bay:
            RTEST_CHECK(g6Px(7u, 92u) == 0u);
            RTEST_CHECK(g6Px(30u, 92u) >= 8u);
            RTEST_CHECK(g6Px(168u, 128u) == 0u); // Clean gap between Left & Right stadium bays!
            // Left 3 pressure pad banks draw vertical lines stepping 65% -> 40% -> 15%, while
            // Right switches draw centered optical dots (unlit seat on MUTE at x=182 when
            // masterRun=false, bright 100% dot on SHIFT at x=222, y=111)!
            const uint16_t gFilt = g6Px(18u, 150u);
            const uint16_t gOct = g6Px(68u, 150u);
            const uint16_t gDuty = g6Px(119u, 150u);
            const uint16_t gMuteDotCenter = g6Px(182u, 145u);
            const uint16_t gMuteUpperEmpty = g6Px(182u, 138u);
            const uint16_t gShiftDotCenter = g6Px(222u, 111u);
            const uint16_t gShiftUpperEmpty = g6Px(222u, 102u);
            RTEST_CHECK(gFilt > gOct && gOct > gDuty);
            RTEST_CHECK(gMuteDotCenter > gMuteUpperEmpty + 5u);
            RTEST_CHECK(gShiftDotCenter > gShiftUpperEmpty + 35u);
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
    const uint16_t slitInitG6 = g6Px(18u, 150u);
    RTEST_CHECK(instructInitG6 >= 30u);
    RTEST_CHECK(slitInitG6 >= 42u);

    for (int f = 0; f < 73; ++f) {
        ctrl.tick(0.10f, tel);
        renderer.prepareFrame(0.10f, quietSt, ctrl.hud(), tel); // +7.30 s -> t = 7.50 s (mid-dim)
    }
    checkStripAndMono();
    const uint16_t instructMidG6 = maxInstructG6();
    const uint16_t slitMidG6 = g6Px(18u, 150u);
    RTEST_CHECK(instructMidG6 < instructInitG6 - 4u);
    RTEST_CHECK(slitMidG6 < slitInitG6 - 6u);

    for (int f = 0; f < 30; ++f) {
        ctrl.tick(0.10f, tel);
        renderer.prepareFrame(0.10f, quietSt, ctrl.hud(),
                              tel); // +3.00 s -> t = 10.50 s (50% floor)
    }
    checkStripAndMono();
    const uint16_t instructDimG6 = maxInstructG6();
    const uint16_t slitDimG6 = g6Px(18u, 150u);
    RTEST_CHECK(instructDimG6 >= 14u && instructDimG6 < instructMidG6 - 3u);
    RTEST_CHECK(slitDimG6 >= 18u && slitDimG6 < slitMidG6 - 5u);

    // Turning SELECT (even clamped at 1/3 end stop) wakes whole view back to 100% brightness!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono();
    RTEST_CHECK(maxInstructG6() == instructInitG6);
    RTEST_CHECK(g6Px(18u, 150u) >= slitInitG6 - 1u);

    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false); // Dismiss SEL

    // Chording multiple keys routes to most-recent press & peels back LIFO on release!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.80f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.65f);
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

    ctrl.onWhiteKeyPressure(5u, 0.90f); // OCTAVE 2 on LIVE MIXER
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceSiren);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.onWhiteKeyGate(5u, false);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::DubLpfDrop);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::OverloadSurge);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
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

    // Switch to 2/3 CIRCUIT LAB via SEL taps + HOME dismiss
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int f = 0; f < 14; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Verify VoiceCircuit stutterGateMask main-screen column dimming (shared ready for RhythmLab):
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.160f, tel);
    renderer.reset();
    quietSt = ctrl.state();
    quietSt.global.masterRun = true;
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
    while (ctrl.hud().freqPageOpen) {
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    }
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.020f, tel);
    }
    verifyRestored();

    // Comprehensive 2/3 CIRCUIT LAB (CMOS PATCH BAY) 27-Key Bends, Headers & 42-LED Light Shows:
    // (a) CROSS-PIN 1-4 (Keys 1-4, white 0..3): neighbor pre-mute FM/XOR into Voice v + 42-LED arc
    for (uint8_t v = 0u; v < 4u; ++v) {
        const size_t nbr = (v + 1u) & 3u;
        ctrl.onWhiteKeyPressure(v, 0.85f);
        ctrl.tick(0.025f, tel);
        RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceXMod);
        RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        RTEST_CHECK(nearF(ctrl.state().keyMod.xmodDepth[v], 0.85f));
        RTEST_CHECK(ctrl.state().voicePressure[v] > 0.60f); // Modulates Voice v itself!
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[v], 1.0f));
        RTEST_CHECK(ctrl.leds().whiteKeyLeds[4u + nbr] > 0.35f ||
                    ctrl.leds().whiteKeyLeds[4u + v] > 0.35f);
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
        ctrl.onWhiteKeyGate(v, false);
        ctrl.tick(0.016f, tel);
        verifyRestored();
    }

    // (b) FOLD 1-4 (Keys 5-8, white 4..7): 1.0x -> 6.2x West-Coast wavefolder + standing crests
    ctrl.onWhiteKeyPressure(4u, 1.0f); // FOLD 1 full pressure -> 6.2x
    ctrl.tick(0.025f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceFold);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(nearF(ctrl.state().keyMod.foldDepth[0], 1.0f));
    RTEST_CHECK(ctrl.state().voicePressure[0] > 0.40f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[4u], 1.0f));
    RTEST_CHECK(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op2)] > 0.10f ||
                ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op3)] > 0.10f);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "FOLD") == 0 && std::strcmp(ls, " OSC 1") == 0);
        RTEST_CHECK(std::strcmp(rt, "6.2x") == 0 && sb && nearF(bn, 1.0f));
    }
    ctrl.onWhiteKeyGate(4u, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (c) CRUSH 1-4 (Keys 9-12, white 8..11): CD4040 sub-XOR + CD4013 S&H clock + 16-bit LED bus
    ctrl.onWhiteKeyPressure(8u, 1.0f); // CRUSH 1 full pressure -> ~165 Hz S&H clock
    ctrl.tick(0.045f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceCrush);
    RTEST_CHECK(nearF(ctrl.state().keyMod.crushDepth[0], 1.0f));
    RTEST_CHECK(ctrl.state().voicePressure[0] > 0.65f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8u], 1.0f));
    {
        // Verify binary ripple divider lights crisp hi/lo contrast across OP1..OP6
        bool anyHi = false;
        for (size_t b = 0u; b < ruido::kNumTopMacroKeys; ++b) {
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
    ctrl.onWhiteKeyGate(8u, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (c) OP1 STARVE: rail overload/coupling surge + pitch sag + PWM squeeze + brownout dim
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.90f);
    ctrl.tick(0.025f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RailStarve);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    RTEST_CHECK(nearF(ctrl.state().keyMod.overloadSurge, 0.90f));
    RTEST_CHECK(ctrl.state().keyMod.voicePitchMul[0] < 0.56f);
    RTEST_CHECK(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op1)] > 0.85f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[11u] < 0.25f); // Brownout dims passive pads!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (d) OP2 RING MOD: balanced 4-quadrant / CMOS XOR mix + checkerboard alternating keys
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RingXor);
    RTEST_CHECK(nearF(ctrl.state().keyMod.ringModMix, 1.0f));
    RTEST_CHECK(std::fabs(ctrl.leds().whiteKeyLeds[0u] - ctrl.leds().whiteKeyLeds[1u]) > 0.50f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (e) OP3 TURBO LFO: 1x -> 32x LFO overclock ("32.0x") + LFO button strobe
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op3, 1.0f);
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
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (f) OP4 SCREAM: opens GlobalFilter MS-20 passband curve + twin Korg35 peaks + red REC flare!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::FilterScream);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.filterScream, 1.0f));
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.85f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)] > 0.80f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[5u] > 0.60f && ctrl.leds().whiteKeyLeds[6u] > 0.60f);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[0u] < 0.25f);
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
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (g) OP5 HARD SYNC: OSC 1 master reset + left-column console anchor (FX + OCT-)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::HardSync);
    RTEST_CHECK(nearF(ctrl.state().keyMod.hardSyncAmount, 1.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0u], 1.0f));
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.90f);
    RTEST_CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)] > 0.90f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (h) OP6 SCATTER: S&H stepped chaos across pitch + PWM + stereo pan
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.0f);
    ctrl.tick(0.020f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::PhaseScatter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.spatialFocus, 1.0f));
    RTEST_CHECK(std::fabs(ctrl.state().keyMod.voicePitchMul[0] - 1.0f) > 0.01f ||
                std::fabs(ctrl.state().keyMod.voicePitchMul[1] - 1.0f) > 0.01f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // (i) PIT SUB-XOR & GLO FEEDBACK on CIRCUIT LAB + Full-Chassis SHIFT Map rendering!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::SubXor);
    RTEST_CHECK(ctrl.state().keyMod.crushDepth[0] >= 0.45f &&
                ctrl.state().keyMod.ringModMix >= 0.50f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 1.0f);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::FeedbackRoar);
    RTEST_CHECK(ctrl.state().keyMod.filterScream >= 0.80f &&
                ctrl.state().keyMod.overloadSurge >= 0.65f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // Verify Full-Chassis SHIFT Map strip parity & unified unassigned animation (CRT ripple +
    // 120 ms 15% glint, zero `"UNASSIGNED"` text!) on CIRCUIT LAB:
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    const uint8_t seqBeforeUnassigned = ctrl.hud().phantomPinSeq;
    ctrl.onWhiteKeyGate(0u, true); // Unassigned under SHIFT -> CRT ripple + 15% glint!
    RTEST_CHECK(ctrl.hud().phantomPinSeq == static_cast<uint8_t>(seqBeforeUnassigned + 1u));
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::None);
    ctrl.tick(0.005f, tel);
    RTEST_CHECK(ctrl.leds().whiteKeyLeds[0u] > 0.13f && ctrl.leds().whiteKeyLeds[0u] <= 0.15f);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::None);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[0u], 0.0f));
    checkStripAndMono(400u);
    ctrl.onWhiteKeyGate(0u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);
    verifyRestored();

    // Return to 1/3 LIVE MIXER
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int f = 0; f < 14; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Turning Osc 1 LEVEL down to 0% in EDIT dims MUTE 1 to 0% (silent), while audible MUTE 2-4
    // remain at 80%:
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -400);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.state().voices[0].amplitude <= 1.0e-4f);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[13], 0.80f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyGate(13u, true); // SHIFT + Key 14 = SOLO 2
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[13], 0.80f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[14], 0.0f));
    ctrl.onWhiteKeyGate(13u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);

    // 5. Single-State Latching MUTE & Exclusive-Isolate SHIFT + SOLO (with Pre-Solo Mix Memory &
    //    Release-Order Guard) via GLOBAL CONFIG Row 1:
    ctrl.reset();
    renderer.reset();
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);    // Row 2: MUTE MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // MOMENTARY -> LATCHING
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false); // Close GLOBAL CONFIG
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.020f, tel);
    }

    // Tap & release MUTE 1 (white key 12) -> Osc 1 latches muted (LED -> 0% dark, peers stay 80%)
    ctrl.onWhiteKeyGate(12u, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 1.0f)); // Header shows "MUTED"
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[13], 0.80f));

    // Second tap on MUTE 1 toggles Osc 1 back ON ("UNMUTED", LED -> 80% audible)
    ctrl.onWhiteKeyGate(12u, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 0.0f)); // Header shows "UNMUTED"
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 1.0f));
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[12], 0.80f));

    // Build custom mix [1, 2 ON; 3, 4 MUTED], then SHIFT + Key 14 (SOLO 2) -> saves pre-Solo mix,
    // isolates OSC 2 alone, keeps SHIFT at steady 100% (no warning blink), and reflects on HOME!
    ctrl.onWhiteKeyGate(14u, true);
    ctrl.onWhiteKeyGate(14u, false);
    ctrl.onWhiteKeyGate(15u, true);
    ctrl.onWhiteKeyGate(15u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(
        nearF(ctrl.leds().whiteKeyLeds[12], 0.80f) && nearF(ctrl.leds().whiteKeyLeds[13], 0.80f) &&
        nearF(ctrl.leds().whiteKeyLeds[14], 0.0f) && nearF(ctrl.leds().whiteKeyLeds[15], 0.0f));

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyGate(13u, true);
    RTEST_CHECK(nearF(ctrl.hud().activePunchValue, 1.0f)); // Header shows "SOLOED"
    ctrl.onWhiteKeyGate(13u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[1], 1.0f));
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kShiftBlackKeyIdx], 1.0f));
    RTEST_CHECK(
        nearF(ctrl.leds().whiteKeyLeds[12], 0.0f) && nearF(ctrl.leds().whiteKeyLeds[13], 0.80f) &&
        nearF(ctrl.leds().whiteKeyLeds[14], 0.0f) && nearF(ctrl.leds().whiteKeyLeds[15], 0.0f));

    // Open HOME (latched FREQUENCY HUD) -> main screen reflects isolated OSC 2 (bright) vs peers!
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

    // Walk SHIFT + Key 15 (SOLO 3 -> [3 ON]) -> walk back SHIFT + Key 14 (SOLO 2 -> [2 ON]) ->
    // tap SHIFT + Key 14 again -> restores pre-Solo mix [1, 2 ON; 3, 4 MUTED]!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyGate(14u, true);
    ctrl.onWhiteKeyGate(14u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[2], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[1], 0.0f));
    ctrl.onWhiteKeyGate(13u, true);
    ctrl.onWhiteKeyGate(13u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[1], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[2], 0.0f));
    ctrl.onWhiteKeyGate(13u, true); // Toggle off Solo 2 -> restores [1, 2 ON; 3, 4 MUTED]!
    ctrl.onWhiteKeyGate(13u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[1], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[2], 0.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[3], 0.0f));

    // Release-Order Guard: holding SHIFT + Key 15 (SOLO 3) then lifting SHIFT BEFORE Key 15 must
    // NEVER re-trigger an accidental MUTE on OSC 3; and unshifted tap on dark Key 13 brings OSC 1
    // directly back in ([1, 3 ON])!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyGate(14u, true);
    ctrl.tick(0.016f, tel);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false); // Release SHIFT first!
    ctrl.tick(0.016f, tel);
    ctrl.onWhiteKeyGate(14u, false); // Then release Key 15 -> stays ON!
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[2], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[0], 0.0f));
    ctrl.onWhiteKeyGate(12u, true); // Direct unshifted 1-tap unmute on dark Key 13 -> [1, 3 ON]!
    ctrl.onWhiteKeyGate(12u, false);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[0], 1.0f) &&
                nearF(ctrl.state().keyMod.voiceGainMul[2], 1.0f));

    // Switching MUTE MODE back to MOMENTARY in GLOBAL CONFIG clears all latches cleanly
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // Row 1 MUTE MODE -> MOMENTARY
    ctrl.tick(0.016f, tel);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(nearF(ctrl.state().keyMod.voiceGainMul[v], 1.0f));
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kMuteWhiteKeyBase + v], 0.80f));
    }
    RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[ruido::kShiftBlackKeyIdx], 1.0f));

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

    const auto checkBottomKnobZoneClear = [&]() {
        size_t bottomHudPx = 0u;
        for (size_t y = 188u; y <= 235u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                if (g6Px(x, y) >= 6u) {
                    ++bottomHudPx;
                }
            }
        }
        RTEST_CHECK(bottomHudPx == 0u);
    };

    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed && !ctrl.hud().saveDeleteMode);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0);
        RTEST_CHECK(std::strcmp(ls, " 1/2") == 0);
        RTEST_CHECK(std::strcmp(rt, "") == 0 && !sb);
        RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[0].bottomZones[2], "DUTY") == 0);
    }
    ctrl.tick(0.050f, tel);
    // Pure-blinking SAVE rule: in 1/2 STORE SCENE ONLY the 4 Scene keys flash in unison while all
    // other 23 keyboard keys rest at 0% dark:
    const float storePulseT0 = ctrl.leds().whiteKeyLeds[ruido::kSceneWhiteKeyBase + 0u];
    RTEST_CHECK(storePulseT0 >= 0.25f && storePulseT0 <= 0.95f);
    for (size_t s = 1u; s < 4u; ++s) {
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kSceneWhiteKeyBase + s], storePulseT0));
    }
    for (size_t b = 0u; b < ruido::kNumBlackKeys; ++b) {
        RTEST_CHECK(nearF(ctrl.leds().blackKeyLeds[b], 0.0f));
    }
    for (size_t w = 0u; w < ruido::kNumWhiteKeys; ++w) {
        if (w < ruido::kSceneWhiteKeyBase || w >= ruido::kMuteWhiteKeyBase) {
            RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[w], 0.0f));
        }
    }
    ctrl.tick(0.200f, tel); // Half-cycle at 2.50 Hz -> opposite phase of flash!
    const float storePulseT1 = ctrl.leds().whiteKeyLeds[ruido::kSceneWhiteKeyBase + 0u];
    RTEST_CHECK(std::fabs(storePulseT1 - storePulseT0) > 0.25f);
    for (size_t s = 1u; s < 4u; ++s) {
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[ruido::kSceneWhiteKeyBase + s], storePulseT1));
    }
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono(750u);
    // Zero UI containers between open status columns (`x = 72, 120, 168`) & SEL-matched `topY = 92`
    RTEST_CHECK(g6Px(72u, 62u) == 0u && g6Px(120u, 62u) == 0u && g6Px(168u, 62u) == 0u);
    RTEST_CHECK(g6Px(120u, 88u) == 0u && g6Px(120u, 95u) > 0u);

    // Turning KNOB 1 while SAVE is armed cleanly disarms SAVE and edits Osc 1 speed immediately:
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
    RTEST_CHECK(!ctrl.hud().saveArmed);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -4); // Restore customHz0
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);

    ctrl.onWhiteKeyPressure(9u, 0.85f); // Store into SCENE 2 (Key 10)
    RTEST_CHECK(!ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(!ctrl.hud().snapshotDirty);
    RTEST_CHECK(ctrl.hud().snapshotSavedToast);
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, " 2") == 0 &&
                    std::strcmp(rt, "SAVED") == 0);
    }
    // Releasing Key 10 after saving is swallowed cleanly (never re-triggers recall!)
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, customHz0, 0.05f));

    // `SAVED` confirmation dissolves `SAVE` straight to the clean scope (header-only toast, zero
    // upper morph overlay collision), while turning `PRESETS` opens the borderless 4-node track!
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 9; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono(0u);
    RTEST_CHECK(g6Px(48u, 54u) == 0u);
    checkBottomKnobZoneClear();

    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 8);
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono(250u);
    RTEST_CHECK(g6Px(180u, 13u) >= 24u); // Top-Bar 4-dot Scene strip!
    RTEST_CHECK(g6Px(66u, 104u) >= 16u &&
                g6Px(102u, 104u) >= 16u); // 4 Scene keys (zero 27-key bed)
    RTEST_CHECK(g6Px(24u, 104u) == 0u && g6Px(216u, 104u) == 0u);
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, -8);

    // Tweak Knob 1 away -> marks internal snapshotDirty; active SCENE 2 LED stays at 80% ACTIVE
    // (`STORED` = 40%, `EMPTY` = 15%), and SHIFT + Key 10 (1-Tap Snap-Back) glides back cleanly!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -60);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(ctrl.hud().snapshotDirty);
    RTEST_CHECK(ctrl.state().voices[0].frequencyHz < 70.0f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.tick(0.016f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8u], 0.40f));
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[9u], 0.80f));
    ctrl.onWhiteKeyPressure(9u, 1.0f); // Firm press (~80ms fast glide)
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(ctrl.hud().snapshotGliding);
    for (int f = 0; f < 10; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(!ctrl.hud().snapshotDirty);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[9u], 0.80f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, customHz0, 0.08f));
    RTEST_CHECK(nearF(ctrl.state().global.circuitCoupling, customCoupling, 1.0e-3f));

    // (c) Continuous multi-key LATCHING pressure sculpting & staggered chord-release freeze under
    // SHIFT:
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyPressure(8u, 0.50f);  // SCENE 1 (65.41 Hz)
    ctrl.onWhiteKeyPressure(10u, 0.50f); // SCENE 3 (55.00 Hz)
    for (int f = 0; f < 30; ++f) {
        ctrl.tick(0.025f, tel);
    }
    const float hybridHz = ctrl.state().voices[0].frequencyHz;
    RTEST_CHECK(std::fabs(hybridHz - 59.98f) < 0.50f);
    // Keep holding AFTER initial glide completes and lean heavily into SCENE 1 -> continues live
    // sculpting toward 65.41 Hz!
    ctrl.onWhiteKeyPressure(8u, 0.90f);
    ctrl.onWhiteKeyPressure(10u, 0.10f);
    for (int f = 0; f < 14; ++f) {
        ctrl.tick(0.025f, tel);
    }
    const float sculptedToward1Hz = ctrl.state().voices[0].frequencyHz;
    RTEST_CHECK(sculptedToward1Hz > 62.8f && sculptedToward1Hz < 65.3f);
    // Staggered release: lift Key 9 FIRST while Key 11 is still briefly down -> freezes sculpted
    // hybrid instead of collapsing onto Scene 3!
    ctrl.onWhiteKeyGate(8u, false);
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.025f, tel);
    }
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, sculptedToward1Hz, 0.05f));
    ctrl.onWhiteKeyGate(10u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.025f, tel);
    }
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, sculptedToward1Hz, 0.05f));
    RTEST_CHECK(ctrl.hud().snapshotDirty);

    // Verify LIVE scratchpad persistence (`exportLivePatch` / `loadLivePatch`) survives reboot:
    {
        RTEST_CHECK(ctrl.consumeLivePatchDirty());
        const ruido::PatchSnapshot liveSnap = ctrl.exportLivePatch();
        RTEST_CHECK(ruido::verifyPatchSnapshotCrc(liveSnap) &&
                    ruido::isPatchSnapshotOccupied(liveSnap));
        ruido::RuidoController rebootCtrl;
        RTEST_CHECK(rebootCtrl.loadLivePatch(liveSnap));
        rebootCtrl.tick(0.016f, tel);
        RTEST_CHECK(rebootCtrl.hud().snapshotDirty);
        RTEST_CHECK(nearF(rebootCtrl.state().voices[0].frequencyHz, sculptedToward1Hz, 0.08f));
        RTEST_CHECK(
            rebootCtrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)] >= 0.12f &&
            rebootCtrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)] <= 0.28f);
    }

    // (d) SNAP MODE = MOMENTARY: non-destructive pressure morph & bit-exact release home across ALL
    //     models (tested here inside 2/3 CIRCUIT LAB via SHIFT + Key 12)!
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> CircuitLab!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);    // Row 3: SNAP MODE
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // LATCHING -> MOMENTARY
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    const ruido::EngineControlState momHome = ctrl.state();

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyPressure(11u,
                            1.0f); // SHIFT + Key 12 = SNAP 4 at 100% pressure inside CircuitLab!
    for (int f = 0; f < 16; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(std::fabs(ctrl.state().global.circuitCoupling - 0.16f) < 0.03f);
    ctrl.onWhiteKeyGate(11u, false); // Release -> melts back to bit-exact momHome!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    for (int f = 0; f < 50; ++f) {
        ctrl.tick(0.025f, tel);
    }
    RTEST_CHECK(ctrl.state().global.circuitCoupling == momHome.global.circuitCoupling);
    for (size_t v = 0u; v < 4u; ++v) {
        RTEST_CHECK(ctrl.state().voices[v].frequencyHz == momHome.voices[v].frequencyHz);
        RTEST_CHECK(ctrl.state().voices[v].pulseWidth == momHome.voices[v].pulseWidth);
        RTEST_CHECK(ctrl.state().voices[v].lpCutoffHz == momHome.voices[v].lpCutoffHz);
    }

    // (e) PRESETS OCT+ Coarse circular steps (1 click = 1 scene around 1 -> 2 -> 3 -> 4 -> 1)
    //     even while in CIRCUIT LAB / RHYTHM LAB:
    RTEST_CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    // Currently activeSlot_ == 0 (SCENE 1); -4 ticks (1 CCW Coarse click) wraps 1 -> SCENE 4!
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, -4);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 3u);
    // +4 ticks (1 CW Coarse click) wraps SCENE 4 -> SCENE 1!
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.020f, tel);
    }
    RTEST_CHECK(!ctrl.hud().snapshotGliding);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, 65.41f, 0.05f));
    RTEST_CHECK(nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)], 0.0f));

    // (f) Endless 360-deg circular PRESETS morphing (`1 -> 2 -> 3 -> 4 -> 1` CW & `1 -> 4` CCW):
    //     hits 100% pure Scene at ALL 4 nodes (zero 3rd-scene bleed) + smooth takeover from EDITED:
    ctrl.reset();
    {
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
        RTEST_CHECK(ctrl.hud().snapshotGliding);
        RTEST_CHECK(ctrl.hud().snapshotWeights[0] > 0.90f && ctrl.hud().snapshotWeights[0] < 1.0f);
        RTEST_CHECK(ctrl.hud().snapshotWeights[1] > 0.0f && ctrl.hud().snapshotWeights[1] < 0.10f);
        RTEST_CHECK(ctrl.hud().snapshotWeights[2] == 0.0f && ctrl.hud().snapshotWeights[3] == 0.0f);

        // +44 more ticks (48 total = 12 detents) seats at 100% pure SCENE 2!
        float prevMorphPos = ctrl.hud().morphPosition;
        for (int step = 0; step < 11; ++step) {
            ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
            RTEST_CHECK(ctrl.hud().morphPosition >= prevMorphPos);
            prevMorphPos = ctrl.hud().morphPosition;
        }
        RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
        RTEST_CHECK(!ctrl.hud().snapshotGliding && !ctrl.hud().snapshotDirty);
        RTEST_CHECK(nearF(ctrl.hud().snapshotWeights[1], 1.0f, 1.0e-4f));
        RTEST_CHECK(ctrl.hud().snapshotWeights[0] == 0.0f && ctrl.hud().snapshotWeights[2] == 0.0f);

        // Continue CW: +48 -> pure SCENE 3, +48 -> pure SCENE 4, +24 -> 50/50 wrap 4 -> 1, +24 ->
        // back at 100% pure SCENE 1!
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 48);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 2u && !ctrl.hud().snapshotDirty);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 48);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 3u && !ctrl.hud().snapshotDirty);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24);
        RTEST_CHECK(ctrl.hud().snapshotGliding);
        RTEST_CHECK(nearF(ctrl.hud().snapshotWeights[3], 0.50f, 1.0e-3f) &&
                    nearF(ctrl.hud().snapshotWeights[0], 0.50f, 1.0e-3f));
        {
            char lm[24]{}, ls[16]{}, rt[32]{};
            bool sb = false;
            bool sd = false;
            float bn = 0.0f;
            ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt),
                                        sb, bn, &sd);
            RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, "") == 0 &&
                        std::strcmp(rt, "") == 0 && !sb && sd);
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 0u && !ctrl.hud().snapshotDirty);

        // CCW from SCENE 1 (-48 ticks) wraps directly 1 -> 4 to 100% pure SCENE 4!
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, -48);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 3u && !ctrl.hud().snapshotDirty);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 48); // Back to SCENE 1

        // Holding SHIFT + twisting PRESETS stays anchored in ShiftMap (`ShiftFocus::Scene`),
        // renders the Top-Bar 4-dot Scene graphic, and mirrors 40%..80% Scene weights onto Keys
        // 9-12 LEDs!
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24); // 50/50 blend between Scene 1 & 2
        for (int f = 0; f < 8; ++f) {
            ctrl.tick(0.025f, tel);
            renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
        }
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().shiftHeld &&
                    ctrl.hud().shiftFocus == ruido::ShiftFocus::Scene);
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8], 0.60f, 0.01f) &&
                    nearF(ctrl.leds().whiteKeyLeds[9], 0.60f, 0.01f) &&
                    nearF(ctrl.leds().whiteKeyLeds[10], 0.40f, 0.01f) &&
                    nearF(ctrl.leds().whiteKeyLeds[11], 0.40f, 0.01f));
        checkStripAndMono(400u);
        RTEST_CHECK(g6Px(180u, 13u) >= 24u && g6Px(196u, 13u) >= 24u); // Top-Bar 4-dot Scene strip!
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, -24);           // Back to 100% pure Scene 1!
        ctrl.tick(0.016f, tel);
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8], 0.80f, 0.01f) &&
                    nearF(ctrl.leds().whiteKeyLeds[9], 0.40f, 0.01f));
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.020f, tel);
        }

        // Twisting PRESETS while FX Filter graph is open keeps the Filter passband visible while
        // showing the Top-Bar spinning disk + 4-dot strip; unshifted Keys 9-12 LEDs stay on Zone 3!
        ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
        ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 24);
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.020f, tel);
            renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
        }
        RTEST_CHECK(ctrl.hud().fxPageOpen);
        RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[8], 0.15f, 0.01f) &&
                    nearF(ctrl.leds().whiteKeyLeds[9], 0.15f, 0.01f));
        checkStripAndMono(400u);
        RTEST_CHECK(g6Px(180u, 13u) >= 24u && g6Px(196u, 13u) >= 24u);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, -24);
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        for (int f = 0; f < 10; ++f) {
            ctrl.tick(0.020f, tel);
            renderer.prepareFrame(0.020f, quietSt, ctrl.hud(), tel);
        }

        // Smooth takeover from EDITED: tweak Osc 1 to >130 Hz, then turn PRESETS +4 ticks -> does
        // NOT jump abruptly back to 65.41 Hz on click 1; smoothly crossfades onto loop over 6
        // clicks!
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 80);
        const float editedHz = ctrl.state().voices[0].frequencyHz;
        RTEST_CHECK(editedHz > 130.0f);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
        const float firstClickHz = ctrl.state().voices[0].frequencyHz;
        RTEST_CHECK(firstClickHz > editedHz * 0.85f && firstClickHz > 110.0f);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 44); // Finish span -> pure SCENE 2
        RTEST_CHECK(ctrl.hud().snapshotSlot == 1u && !ctrl.hud().snapshotDirty);
        RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, 87.31f, 0.15f) &&
                    nearF(ctrl.state().voices[1].frequencyHz, 131.35f, 0.25f));
    }

    // (g) Self-discoverable 2-step SAVE (`1st press = STORE SCENE 1/2` -> `2nd press = DELETE SCENE
    //     2/2`), safe no-op on EMPTY in Delete mode, skipping EMPTY on circular loop, & NVS reload:
    ctrl.reset();
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed && !ctrl.hud().saveDeleteMode);
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed && ctrl.hud().saveDeleteMode);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0);
        RTEST_CHECK(std::strcmp(ls, " 2/2") == 0);
        RTEST_CHECK(std::strcmp(rt, "") == 0 && !sb);
    }
    // Delete SCENE 2 (white key 9) -> slot 1 becomes EMPTY with valid CRC-32
    ctrl.onWhiteKeyGate(9u, true);
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(!ctrl.hud().saveArmed && !ctrl.hud().saveDeleteMode);
    RTEST_CHECK(ctrl.hud().snapshotDeletedToast);
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == 0x0Du); // Scenes 1, 3, 4 occupied
    RTEST_CHECK(ruido::verifyPatchSnapshotCrc(ctrl.snapshots().slotBinary(1u)));
    RTEST_CHECK(!ruido::isPatchSnapshotOccupied(ctrl.snapshots().slotBinary(1u)));
    RTEST_CHECK((ctrl.consumeSavedSlotMask() & (1u << 1u)) != 0u);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, " 2") == 0 &&
                    std::strcmp(rt, "DELETED") == 0 && !sb);
    }
    quietSt = ctrl.state();
    quietSt.global.masterRun = false;
    quietSt.global.masterTrim = 0.0f;
    for (int f = 0; f < 9; ++f) {
        ctrl.tick(0.025f, tel);
        renderer.prepareFrame(0.025f, quietSt, ctrl.hud(), tel);
    }
    checkStripAndMono(0u);
    RTEST_CHECK(g6Px(48u, 54u) == 0u);
    checkBottomKnobZoneClear();

    // Verify 1/2 flashes ALL 4 saveable keys (Empty & Stored), while 2/2 flashes ONLY occupied
    // deletable keys and keeps EMPTY Key 10 strictly 0% dark & safe no-op!
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.tick(0.050f, tel);
    const float storeBothPulse = ctrl.leds().whiteKeyLeds[9u];
    RTEST_CHECK(storeBothPulse >= 0.25f);
    RTEST_CHECK(
        nearF(ctrl.leds().whiteKeyLeds[10u], storeBothPulse)); // Stored also flashes in 1/2!
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.tick(0.050f, tel);
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[9u], 0.0f)); // Empty is 0% dark in 2/2 DELETE
    const float delPulseT0 = ctrl.leds().whiteKeyLeds[10u];
    RTEST_CHECK(delPulseT0 >= 0.25f); // Occupied flashes at 2x (5.00 Hz) in 2/2 DELETE
    ctrl.tick(0.100f, tel);           // 100 ms = half-cycle at 5.00 Hz (2x speed)!
    const float delPulseHalf = ctrl.leds().whiteKeyLeds[10u];
    RTEST_CHECK(std::fabs(delPulseHalf - delPulseT0) > 0.25f);
    ctrl.tick(0.100f, tel); // +100 ms = full 200 ms cycle at 5.00 Hz -> back to delPulseT0!
    RTEST_CHECK(nearF(ctrl.leds().whiteKeyLeds[10u], delPulseT0, 1.0e-3f));
    ctrl.onWhiteKeyGate(9u, true); // Tapping EMPTY in Delete = safe no-op
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(ctrl.hud().saveArmed && ctrl.hud().saveDeleteMode);
    ctrl.onButtonEvent(ruido::ButtonId::Save, true); // 3rd press exits SAVE
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(!ctrl.hud().saveArmed);

    // Sweeping PRESETS +12 detents (+48 ticks) from SCENE 1 skips deleted SCENE 2 (`weight == 0`)
    // and lands cleanly at 100% pure SCENE 3!
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 48);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 2u);
    RTEST_CHECK(ctrl.hud().snapshotWeights[1] == 0.0f);
    RTEST_CHECK(nearF(ctrl.hud().snapshotWeights[2], 1.0f, 1.0e-4f));
    const float scene3Hz = ctrl.state().voices[0].frequencyHz;
    RTEST_CHECK(nearF(scene3Hz, 55.00f, 0.15f));

    // Tapping empty SCENE 2 key (SHIFT + white 9) shows "SCENE 2 · EMPTY" without altering live
    // Scene 3 audio!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onWhiteKeyPressure(9u, 0.90f);
    ctrl.tick(0.016f, tel);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, " 2") == 0 &&
                    std::strcmp(rt, "EMPTY") == 0 && !sb);
    }
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, scene3Hz, 0.05f));
    ctrl.onWhiteKeyGate(9u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);

    // Deleting the currently active scene (SCENE 3) leaves live panel audio uninterrupted, and
    // deleting remaining scenes (1 & 4) reaches "SCENES · ALL EMPTY" cleanly:
    for (uint8_t s : {2u, 0u, 3u}) {
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(ruido::kSceneWhiteKeyBase + s), true);
        ctrl.onWhiteKeyGate(static_cast<uint8_t>(ruido::kSceneWhiteKeyBase + s), false);
    }
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == 0u);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, scene3Hz, 0.05f));
    ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                    bn);
        RTEST_CHECK(std::strcmp(lm, "SCENE") == 0 && std::strcmp(ls, "") == 0 &&
                    std::strcmp(rt, "ALL EMPTY") == 0 && !sb);
    }

    // Re-saving into SCENE 2 restores occupation, and NVS boot reload preserves deleted slots:
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.onWhiteKeyGate(9u, true);
    ctrl.onWhiteKeyGate(9u, false);
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == (1u << 1u));
    const std::array<ruido::PatchSnapshot, 4> nvsSlots = {
        ctrl.snapshots().slotBinary(0u), ctrl.snapshots().slotBinary(1u),
        ctrl.snapshots().slotBinary(2u), ctrl.snapshots().slotBinary(3u)};
    const ruido::DeviceSettingsBlob nvsBlob = ctrl.exportSettings();

    ctrl.reset();
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == 0x0Fu);
    for (uint8_t s = 0u; s < 4u; ++s) {
        RTEST_CHECK(ctrl.loadSnapshotSlot(s, nvsSlots[s]));
    }
    RTEST_CHECK(ctrl.loadSettings(nvsBlob));
    RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == (1u << 1u));
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, scene3Hz, 0.05f));

    // Post-audit regressions: Home Base weight reset on manual edit, smooth Snap-Back takeover,
    // 3rd-press SAVE cancel under momentary holds, PLAY/STOP preserving saveArmed, 2.50 Hz pulse
    // sync without Env/Save leak on Key 9 hold, CW S3->S1 coarse wrap, and chord bead suppression.
    {
        auto strEq = [](const char* a, const char* b) {
            if (!a || !b)
                return false;
            while (*a && *b && *a == *b) {
                ++a;
                ++b;
            }
            return *a == '\0' && *b == '\0';
        };
        auto tapSceneKey = [&](uint8_t slot) {
            const uint8_t key = static_cast<uint8_t>(ruido::kSceneWhiteKeyBase + slot);
            ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
            ctrl.onWhiteKeyGate(key, true);
            ctrl.onWhiteKeyPressure(key, 0.7f);
            ctrl.tick(0.02f, tel);
            ctrl.onWhiteKeyPressure(key, 0.0f);
            ctrl.onWhiteKeyGate(key, false);
            ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
            ctrl.tick(0.04f, tel);
        };
        auto saveSlot = [&](uint8_t slot) {
            ctrl.onButtonEvent(ruido::ButtonId::Save, true);
            ctrl.onButtonEvent(ruido::ButtonId::Save, false);
            ctrl.tick(0.02f, tel);
            const uint8_t key = static_cast<uint8_t>(ruido::kSceneWhiteKeyBase + slot);
            ctrl.onWhiteKeyGate(key, true);
            ctrl.onWhiteKeyPressure(key, 0.7f);
            ctrl.tick(0.02f, tel);
            ctrl.onWhiteKeyPressure(key, 0.0f);
            ctrl.onWhiteKeyGate(key, false);
            ctrl.tick(0.04f, tel);
        };

        const auto emptySnap = nvsSlots[0]; // verified empty slot from NVS test above
        ctrl.reset();
        for (uint8_t s = 0u; s < 4u; ++s) {
            RTEST_CHECK(ctrl.loadSnapshotSlot(s, emptySnap));
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -30);
        ctrl.tick(0.02f, tel);
        const float s1Hz = ctrl.state().voices[0].frequencyHz;
        saveSlot(0u);

        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 45);
        ctrl.tick(0.02f, tel);
        const float s2Hz = ctrl.state().voices[0].frequencyHz;
        RTEST_CHECK(s2Hz > s1Hz + 20.0f);
        saveSlot(1u);
        RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == 0x03u);

        // Recall Scene 1 and settle, then bend +24 PRESETS ticks and tweak Knob1.
        tapSceneKey(0u);
        for (int i = 0; i < 40; ++i)
            ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
        RTEST_CHECK(nearF(ctrl.state().voices[0].frequencyHz, s1Hz, 0.25f));

        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 16);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.state().voices[0].frequencyHz > s1Hz + 2.0f);

        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 20);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
        RTEST_CHECK(nearF(ctrl.hud().snapshotWeights[0], 1.0f, 1e-4f));
        RTEST_CHECK(nearF(ctrl.hud().snapshotWeights[1], 0.0f, 1e-4f));

        char leftMain[16]{};
        char leftSub[16]{};
        char right[16]{};
        bool showBar = false;
        bool showDots = false;
        float barNorm = 0.0f;
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(strEq(leftMain, "SCENE"));
        RTEST_CHECK(strEq(leftSub, ""));
        RTEST_CHECK(strEq(right, "") && !showBar && showDots);

        // 1-Tap Snap-Back uses Top-Bar 4-dot Scene graphic (zero text % or EDITED), and PRESETS
        // takes over smoothly.
        tapSceneKey(0u);
        ctrl.tick(0.04f, tel);
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(!showBar && showDots && strEq(right, ""));
        const float midSnapHz = ctrl.state().voices[0].frequencyHz;
        RTEST_CHECK(midSnapHz > s1Hz + 5.0f);

        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
        ctrl.tick(0.02f, tel);
        const float takeoverHz = ctrl.state().voices[0].frequencyHz;
        RTEST_CHECK(takeoverHz > s1Hz + 5.0f);
        RTEST_CHECK(nearF(takeoverHz, midSnapHz, 12.0f));
        for (int i = 0; i < 30; ++i)
            ctrl.tick(0.02f, tel);

        // 3rd-press SAVE cancel while holding OctMinus must exit HudMode::SaveArmed immediately.
        tapSceneKey(0u);
        for (int i = 0; i < 40; ++i)
            ctrl.tick(0.02f, tel);
        ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
        ctrl.tick(0.02f, tel);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().saveArmed && !ctrl.hud().saveDeleteMode);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().saveArmed && ctrl.hud().saveDeleteMode);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(!ctrl.hud().saveArmed);
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::KnobRate);
        ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
        ctrl.tick(0.02f, tel);

        // PLAY/STOP while SAVE is armed keeps saveArmed and returns to SaveArmed after toast.
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().saveArmed);
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().saveArmed);
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::AudioOutput);
        for (int i = 0; i < 12; ++i)
            ctrl.tick(0.20f, tel);
        RTEST_CHECK(ctrl.hud().saveArmed);
        RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);

        // 2.50 Hz pulse lockstep between ButtonId::Save LED and HudState::savePulseQ8 over 12 s.
        const auto checkSavePulseSync = [&]() {
            const float ledPulse01 =
                (ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)] - 0.25f) /
                0.75f;
            const float hudPulse01 = static_cast<float>(ctrl.hud().savePulseQ8) / 255.0f;
            RTEST_CHECK(nearF(ledPulse01, hudPulse01, 0.02f));
        };
        checkSavePulseSync();
        const uint8_t q8AtStart = ctrl.hud().savePulseQ8;
        for (int i = 0; i < 600; ++i)
            ctrl.tick(0.02f, tel); // 12.0 s = 30 cycles at 2.50 Hz
        checkSavePulseSync();
        RTEST_CHECK(
            std::abs(static_cast<int>(ctrl.hud().savePulseQ8) - static_cast<int>(q8AtStart)) <= 4);

        // Cancel SAVE cleanly.
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(!ctrl.hud().saveArmed);

        // Holding SHIFT + Key 9 cleanly leaves Env == 0 and clean Save == 0.
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.9f);
        ctrl.tick(0.05f, tel);
        RTEST_CHECK(
            nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.0f, 1e-4f));
        RTEST_CHECK(
            nearF(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)], 0.0f, 1e-4f));
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, false);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        for (int i = 0; i < 40; ++i)
            ctrl.tick(0.02f, tel);

        // S3 -> S1 CW Coarse wrap when S2 & S4 are EMPTY + 2-key chord traveler suppression.
        RTEST_CHECK(ctrl.loadSnapshotSlot(1u, emptySnap));
        saveSlot(2u);
        RTEST_CHECK(ctrl.hud().snapshotOccupiedMask == ((1u << 0u) | (1u << 2u)));
        RTEST_CHECK(ctrl.hud().snapshotSlot == 2u);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
        for (int i = 0; i < 8; ++i)
            ctrl.tick(0.02f, tel); // HUD wakes to 1.0f while OCT+ is held
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
        ctrl.tick(0.045f, tel);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 0u);
        RTEST_CHECK(ctrl.hud().snapshotLoopPhase > 2.0f);
        for (int f = 0; f < 6; ++f) {
            renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        }
        checkStripAndMono(80u);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
        for (int i = 0; i < 40; ++i)
            ctrl.tick(0.02f, tel);

        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.8f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 2u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 2u, 0.8f);
        for (int i = 0; i < 6; ++i)
            ctrl.tick(0.03f, tel);
        ruido::HudState chordHud = ctrl.hud();
        chordHud.shiftHeld = false;
        chordHud.mode = ruido::HudMode::SnapshotSlot;
        for (int f = 0; f < 6; ++f) {
            renderer.prepareFrame(0.030f, quietSt, chordHud, tel);
        }
        checkStripAndMono(80u);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, false);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 2u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 2u, false);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);

        // O(1) circular phase folding handles NaN, +/-Inf, huge floats, and negative values
        // cleanly.
        const float nanVal = std::nanf("");
        const float infVal = INFINITY;
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(nanVal, 4.0f), 0.0f));
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(infVal, 4.0f), 0.0f));
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(-infVal, 4.0f), 0.0f));
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(5.0e10f, 4.0f), 0.0f));
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(-0.25f, 4.0f), 3.75f));
        RTEST_CHECK(nearF(ruido::wrapCircularPhase(7.5f, 4.0f), 3.5f));

        // Orphan delete + recall: deleting active S2 (`1u`) then recalling S4 (`3u`) must never
        // pack a phantom `S2 -> S4` pair or snap `snapshotLoopPhase` to `S1` (`0.0f`), and the
        // `150 ms` `2/2 DELETE SCENE` crossfade renders strip-identically without text collision!
        saveSlot(1u);
        saveSlot(3u);
        tapSceneKey(1u);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().saveDeleteMode);
        for (int f = 0; f < 4; ++f) {
            renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        }
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 1u, true);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 1u, false);
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(!ctrl.hud().saveArmed);
        RTEST_CHECK((ctrl.hud().snapshotOccupiedMask & (1u << 1u)) == 0u);
        // Mid-fade (`150 ms`) after deleting S2: strip & monochrome invariant holds.
        renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        checkStripAndMono(180u);

        // Exiting SAVE via KNOB 1 (`VoiceFrequency`, `knobBank != None`) suppresses SAVE bottom
        // text cleanly during the `150 ms` crossfade.
        ctrl.onButtonEvent(ruido::ButtonId::Save, true);
        ctrl.onButtonEvent(ruido::ButtonId::Save, false);
        ctrl.tick(0.02f, tel);
        for (int f = 0; f < 4; ++f) {
            renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 2);
        ctrl.tick(0.02f, tel);
        renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        checkStripAndMono(180u);

        // Recall S4 from orphaned deleted S2: loop phase lands at S4 (`3.0f`) with Top-Bar 4-dot
        // Scene graphic (zero ASCII arrows or PRESSURE MIX).
        tapSceneKey(3u);
        ctrl.tick(0.03f, tel);
        RTEST_CHECK(nearF(ctrl.hud().snapshotLoopPhase, 3.0f, 1.0e-3f));
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(strEq(leftMain, "SCENE"));
        RTEST_CHECK(strEq(leftSub, ""));
        RTEST_CHECK(strEq(right, "") && showDots);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(strEq(right, "") && showDots);

        // Rapid 2-click `OCT+` + `PRESETS` (`1 -> 2 -> 3`) renders Top-Bar spinning disk + 4-dot
        // Scene graphic cleanly with zero ASCII `"1 -> 3"` / `"2 -> 3"` arrows!
        saveSlot(1u);
        tapSceneKey(0u);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
        for (int i = 0; i < 6; ++i)
            ctrl.tick(0.02f, tel);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4); // 1 -> 2
        ctrl.tick(0.02f, tel);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4); // 20 ms flick -> 3 (still 87% S1)
        ctrl.tick(0.02f, tel);
        RTEST_CHECK(ctrl.hud().snapshotSlot == 2u);
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(strEq(leftMain, "SCENE") && strEq(leftSub, "") && strEq(right, "") && showDots);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);
        tapSceneKey(0u);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4); // 1 -> 2
        ctrl.tick(0.06f, tel);                             // Past midpoint (74% S2)
        ctrl.onEncoderDelta(ruido::EncoderId::Presets, 4); // 2 -> 3
        ctrl.tick(0.02f, tel);
        ruido::formatSnapshotHeader(ctrl.hud(), leftMain, sizeof(leftMain), leftSub,
                                    sizeof(leftSub), right, sizeof(right), showBar, barNorm,
                                    &showDots);
        RTEST_CHECK(strEq(leftMain, "SCENE") && strEq(leftSub, "") && strEq(right, "") && showDots);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
        for (int i = 0; i < 35; ++i)
            ctrl.tick(0.02f, tel);

        // 3-key chord `"PRESSURE MIX"` + `OCT+` (`COARSE` badge) under SHIFT renders cleanly
        // without header overlap.
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.8f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 1u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 1u, 0.8f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 2u, true);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 2u, 0.8f);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
        for (int i = 0; i < 6; ++i)
            ctrl.tick(0.02f, tel);
        for (int f = 0; f < 4; ++f) {
            renderer.prepareFrame(0.030f, quietSt, ctrl.hud(), tel);
        }
        checkStripAndMono(80u);
        ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, false);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 1u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 1u, false);
        ctrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 2u, 0.0f);
        ctrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 2u, false);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    }
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
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[0].topZones[0], "TONE") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[0].topZones[1], "FILTER") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[0].topZones[2], "WIDTH") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[1].topZones[0], "MOD") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[1].topZones[1], "LOCK") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[1].topZones[2], "BENDS") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].topZones[0], "PULSE") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].topZones[1], "GATE") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].topZones[2], "FX") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].bottomZones[1], "DIVIDE") == 0);
    RTEST_CHECK(std::strcmp(ruido::kKeyboardModelSpecs[2].bottomZones[2], "EUCLID") == 0);
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    // Verify both LATCH (`x ~ 172`) and TAP (`x ~ 203`) top-row legends render cleanly on RHYTHM
    // LAB SEL with clear separation between them (`x = 190`):
    {
        uint32_t latchLegendMax = 0u;
        uint32_t tapLegendMax = 0u;
        uint32_t gapMax = 0u;
        for (size_t y = 72u; y <= 82u; ++y) {
            for (size_t x = 164u; x <= 180u; ++x) {
                const uint32_t g = (s16[y * 240u + x] >> 5u) & 0x3Fu;
                if (g > latchLegendMax)
                    latchLegendMax = g;
            }
            for (size_t x = 196u; x <= 210u; ++x) {
                const uint32_t g = (s16[y * 240u + x] >> 5u) & 0x3Fu;
                if (g > tapLegendMax)
                    tapLegendMax = g;
            }
            const uint32_t gapG = (s16[y * 240u + 190u] >> 5u) & 0x3Fu;
            if (gapG > gapMax)
                gapMax = gapG;
        }
        RTEST_CHECK(latchLegendMax >= 28u && tapLegendMax >= 28u && gapMax == 0u);
        RTEST_CHECK(std::strcmp(ruido::keyPunchSpec(ruido::KeyPunchKind::KeyHoldSwitch).title,
                                "LATCH") == 0);
    }

    // Unshifted ALGORITHM is now consistently RAIL COUPLING across ALL 3 keyboard models (including
    // 3/3 RHYTHM LAB!), toasting 1.5 s in the top header while SEL stays pinned open underneath:
    const float couplingBeforeRhythmAlg = ctrl.state().global.circuitCoupling;
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 24);
    RTEST_CHECK(ctrl.hud().selPageOpen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    RTEST_CHECK(ctrl.state().global.circuitCoupling > couplingBeforeRhythmAlg + 0.10f);
    for (int f = 0; f < 4; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -24);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    RTEST_CHECK(!ctrl.hud().selPageOpen);

    // Universal SHIFT + ALGORITHM adjusts Master Tempo logarithmically (+48 ticks = +1.0 octave ->
    // 240.0 BPM) across 3 gears [20.0, 3000.0] BPM while showing ShiftMap + TEMPO inspector!
    const float couplingBeforeShiftTempo = ctrl.state().global.circuitCoupling;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 48); // +1.0 oct -> ~240.0 BPM
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap &&
                ctrl.hud().shiftFocus == ruido::ShiftFocus::Tempo);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 240.0f) < 0.25f);
    RTEST_CHECK(std::fabs(ctrl.state().global.clockBpm - 240.0f) < 0.25f);
    RTEST_CHECK(nearF(ctrl.state().global.circuitCoupling, couplingBeforeShiftTempo));
    for (int f = 0; f < 8; ++f) {
        ctrl.tick(0.025f, tel);
        ruido::EngineControlState silentShiftSt = ctrl.state();
        silentShiftSt.global.masterRun = false;
        silentShiftSt.global.masterTrim = 0.0f;
        renderer.prepareFrame(0.025f, silentShiftSt, ctrl.hud(), tel);
    }
    checkStripAndMono();
    // Verify Option A SHIFT screen:
    //  - KEYBOARD is centered directly above OP1-OP3 (`y = 74..85`, ZERO legs below chassis at
    //    `y = 154`),
    //  - SELECT -> KEYBOARD (`y = 40..50`) and ALGORITHM -> TEMPO (`y = 57..67`) callouts render
    //    over `#000000` knockout while PRESETS (`28, 62.5`) stays ghosted dark (`< 12`),
    //  - Bottom description line (`y = 206..216`) shows `"MASTER CLOCK SPEED (20-3000 BPM)"` with
    //    zero duplicate parameter readout bar above it (`y = 190..202`), and
    //  - ALGORITHM knob pointer slit rotates with live tempo!
    {
        size_t dupBarPx = 0u;
        size_t descLinePx = 0u;
        uint32_t topKbLabelMax = 0u;
        uint32_t tempoCalloutMax = 0u;
        for (size_t y = 190u; y <= 202u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                if (((s16[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                    ++dupBarPx;
                }
            }
        }
        for (size_t y = 206u; y <= 216u; ++y) {
            for (size_t x = 40u; x <= 200u; ++x) {
                if (((s16[y * 240u + x] >> 5u) & 0x3Fu) >= 24u) {
                    ++descLinePx;
                }
            }
        }
        for (size_t y = 74u; y <= 84u; ++y) {
            for (size_t x = 24u; x <= 70u; ++x) {
                const uint32_t g = (s16[y * 240u + x] >> 5u) & 0x3Fu;
                if (g > topKbLabelMax)
                    topKbLabelMax = g;
            }
        }
        for (size_t y = 57u; y <= 67u; ++y) {
            for (size_t x = 68u; x <= 104u; ++x) {
                const uint32_t g = (s16[y * 240u + x] >> 5u) & 0x3Fu;
                if (g > tempoCalloutMax)
                    tempoCalloutMax = g;
            }
        }
        RTEST_CHECK(dupBarPx == 0u);
        RTEST_CHECK(descLinePx >= 40u);
        RTEST_CHECK(topKbLabelMax >= 36u && tempoCalloutMax >= 36u);
        // Zero bottom legs under OP1-OP3 at y = 162 (`x = 35..59`) & PRESETS center (`28, 62`)
        // stays ghosted!
        RTEST_CHECK(((s16[162u * 240u + 47u] >> 5u) & 0x3Fu) == 0u);
        RTEST_CHECK(((s16[62u * 240u + 28u] >> 5u) & 0x3Fu) <= 6u);
        std::array<uint16_t, 121u> algoDial240{};
        size_t di = 0u;
        for (size_t y = 57u; y <= 67u; ++y) {
            for (size_t x = 45u; x <= 55u; ++x) {
                algoDial240[di++] = s16[y * 240u + x];
            }
        }
        ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 96); // +2.0 oct -> ~960 BPM
        ruido::EngineControlState silentShiftSt2 = ctrl.state();
        silentShiftSt2.global.masterRun = false;
        silentShiftSt2.global.masterTrim = 0.0f;
        renderer.prepareFrame(0.020f, silentShiftSt2, ctrl.hud(), tel);
        renderer.renderFullScreenStrips(s8.data());
        bool dialRotated = false;
        di = 0u;
        for (size_t y = 57u; y <= 67u; ++y) {
            for (size_t x = 45u; x <= 55u; ++x) {
                if (s8[y * 240u + x] != algoDial240[di++]) {
                    dialRotated = true;
                }
            }
        }
        RTEST_CHECK(dialRotated);
        ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -96);
    }
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -48); // Back to ~120.0 BPM

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
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    // Verify high-speed 3,000 BPM LED anti-strobe choreography stays strictly in [0, 1]
    ctrl.onWhiteKeyPressure(8u, 0.70f); // EUCLID @ 3,000 BPM
    ctrl.tick(0.016f, tel);
    for (float l : ctrl.leds().whiteKeyLeds) {
        RTEST_CHECK(l >= 0.0f && l <= 1.0f);
    }
    ctrl.onWhiteKeyGate(8u, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -300); // Clamp at 20.0 BPM!
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 20.0f));
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 16); // +1.0 oct -> 40.0 BPM
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 40.0f) < 0.15f);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);

    // Verify Permanent POLY Key (`BlackKeyId::Poly`) = TAP TEMPO across ALL models (aligns Beat 1
    // on Tap 1, rejects <40 ms switch bounce, holds MasterTempo toast between taps, locks 120
    // BPM!):
    const uint8_t seqBeforeTap = ctrl.state().global.clockResetSeq;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Poly, 0.80f); // Held pressure: no 2x fire!
    RTEST_CHECK(!ctrl.hud().scopeFrozen);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterTempo);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::TapTempo);
    RTEST_CHECK(ctrl.state().global.clockResetSeq == static_cast<uint8_t>(seqBeforeTap + 1u));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::MasterTempo); // Steady toast between taps!

    // < 40 ms switch bounce after Tap 1 must be ignored without corrupting BPM:
    ctrl.tick(0.020f, tel);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 40.0f) < 0.15f);

    // Advance remaining 0.480 s (total 0.500 s since Tap 1 = 120.0 BPM) and tap twice @ 0.500 s:
    for (int f = 0; f < 24; ++f) {
        ctrl.tick(0.020f, tel);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 120.0f) < 0.5f);
    for (int f = 0; f < 25; ++f) {
        ctrl.tick(0.020f, tel); // 0.500 s -> Tap 3
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 120.0f));

    // 2. Exercise STUTTER, DIVIDE, EUCLID, PUMP, CHASE, TEMPO, PLUCK, SHORT, SWING + HUD headers
    ctrl.onWhiteKeyPressure(0u, 0.50f); // Key 1: STUTTER OSC 1
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceStutter);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[0] ==
                static_cast<uint8_t>(ruido::VoiceRhythmMode::Stutter));
    ctrl.onWhiteKeyGate(0u, false);

    ctrl.onWhiteKeyPressure(7u, 1.0f); // Key 8: DIVIDE OSC 4 @ full pressure -> X8 (index 5)
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
    ctrl.onWhiteKeyGate(7u, false);

    ctrl.onWhiteKeyPressure(8u, 0.45f); // Key 9: EUCLID OSC 1 -> 5/16
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::VoiceEuclid);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[0] ==
                static_cast<uint8_t>(ruido::VoiceRhythmMode::Euclid));
    ctrl.tick(0.016f, tel);
    for (float l : ctrl.leds().whiteKeyLeds) {
        RTEST_CHECK(l >= 0.0f && l <= 1.0f);
    }
    ctrl.onWhiteKeyGate(8u, false);

    // SAVE + Key 10 (white 9) stores SCENE 2 cleanly even inside RHYTHM LAB without firing EUCLID!
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    RTEST_CHECK(ctrl.hud().saveArmed);
    ctrl.onWhiteKeyPressure(9u, 0.90f);
    RTEST_CHECK(!ctrl.hud().saveArmed);
    RTEST_CHECK(ctrl.hud().snapshotSlot == 1u);
    RTEST_CHECK(ctrl.state().keyMod.rhythmMode[1] == 0u);
    ctrl.onWhiteKeyGate(9u, false);

    // OP4 PLUCK opens live MS-20 Filter Pluck graph & maps knobs to GlobalFilter
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op4, 0.85f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmPluck);
    RTEST_CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmPluck, 0.85f));
    for (int f = 0; f < 6; ++f) {
        ctrl.tick(0.020f, tel);
        renderer.prepareFrame(0.020f, ctrl.state(), ctrl.hud(), tel);
    }
    checkStripAndMono();
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);

    // OP1 PUMP, OP2 CHASE, OP5 SHORT, OP6 SWING
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, 0.80f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmPump);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op2, 0.70f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmChase);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op2, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op5, 0.90f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmShort);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op5, false);

    // OP6 SHUFFLE (0-100% pure timing warp!), PIT PING-PONG (0-100% stereo bounce), GLO ROLL
    // (0-100%)
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmSwing);
    RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmSwing, 1.0f) &&
                nearF(ctrl.state().keyMod.rhythmPingPong, 0.0f));
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "SHUFFLE") == 0 && std::strcmp(rt, "100%") == 0 && sb &&
                    nearF(bn, 1.0f));
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Pit, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmPingPong);
    RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmPingPong, 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 1.0f);
    RTEST_CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RhythmRoll);
    RTEST_CHECK(nearF(ctrl.state().keyMod.rhythmRoll, 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // 3. Port-agnostic 24-PPQN MIDI Clock In (TRS / USB / BLE-MIDI burst jitter immunity)
    RTEST_CHECK(!ctrl.hud().midiClockSynced);
    RTEST_CHECK(nearF(ctrl.hud().effectiveClockBpm, 120.0f));
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::Off);
    const uint8_t seqBeforeOffStart = ctrl.state().global.clockResetSeq;
    ctrl.onMidiRealtime(0xFAu, 0.900f); // Default OFF ignores 0xFA Start (bar phase untouched)
    RTEST_CHECK(ctrl.state().global.clockResetSeq == seqBeforeOffStart);

    // Switch MIDI TRANSPORT IN -> PHASE and verify 0xFA Start resets Beat-1 bar phase
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 20);   // Row 5: MIDI TRANSPORT IN
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4); // OFF -> PHASE
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    RTEST_CHECK(ctrl.hud().config.midiTransport == ruido::MidiTransportMode::PhaseOnly);

    const uint8_t seqBeforeStart = ctrl.state().global.clockResetSeq;
    ctrl.onMidiRealtime(0xFAu, 1.000f); // MIDI Start in PHASE mode
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

    // Hold OP3 SPEED at minimal pressure -> header shows "140 BPM EXT"!
    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op3, 0.002f);
    {
        char lm[24]{}, ls[16]{}, rt[32]{};
        bool sb = false;
        float bn = 0.0f;
        ruido::formatKeybedTopHeader(ctrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt, sizeof(rt), sb,
                                     bn);
        RTEST_CHECK(std::strcmp(lm, "SPEED") == 0 && std::strcmp(rt, "140 BPM EXT") == 0);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);

    // Turning SHIFT + ALGORITHM OR tapping POLY while externally MIDI-synced shows "140 BPM EXT"
    // without mutating internal fallback BPM or breaking external lock (tap aligns Beat 1)!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 24);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap &&
                ctrl.hud().shiftFocus == ruido::ShiftFocus::Tempo);
    RTEST_CHECK(ctrl.hud().midiClockSynced);
    RTEST_CHECK(std::fabs(ctrl.hud().effectiveClockBpm - 140.0f) < 0.4f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    const uint8_t seqBeforeExtTap = ctrl.state().global.clockResetSeq;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
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
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // Row 5 -> Row 4: MIDI CLOCK IN
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
    ctrl.onWhiteKeyGate(10u, true); // Save into slot 2 (Key 11)
    ctrl.onWhiteKeyGate(10u, false);
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == (1u << 2u));
    RTEST_CHECK(ctrl.consumeSavedSlotMask() == 0u);
    RTEST_CHECK(ctrl.consumeSettingsDirty());
    const float savedOsc1Hz = ctrl.state().voices[0].frequencyHz;
    const ruido::PatchSnapshot savedSlot2 = ctrl.snapshots().slotBinary(2u);

    // Switch to RhythmLab, dial ~1,920 BPM (+4 octaves = +192 ticks) via SHIFT + ALGORITHM,
    // then switch to CircuitLab + Latching MuteMode and export 16 B settings blob!
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8); // -> RhythmLab
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 192); // +4.0 octaves -> ~1920.0 BPM
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    const float dialedHighBpm = ctrl.hud().effectiveClockBpm;
    RTEST_CHECK(std::fabs(dialedHighBpm - 1920.0f) < 2.0f);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4); // -> CircuitLab (ALGORITHM = Rail Coupling)
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    const float couplingBeforeCircuitAlg = ctrl.state().global.circuitCoupling;
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8);
    RTEST_CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    RTEST_CHECK(ctrl.state().global.circuitCoupling > couplingBeforeCircuitAlg);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, true);
    ctrl.onButtonEvent(ruido::ButtonId::GloRight, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);     // Row 1: Key Sensitivity
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -4); // MEDIUM -> LIGHT
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);     // Row 2: MuteMode
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);    // Row 5: MidiTransport
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8);  // OFF -> RunStop (+2 detents)
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
    RTEST_CHECK(bootCtrl.hud().config.pressureTouch == ruido::PressureTouch::Light);
    {
        // Legacy v1 blobs (byte 13 == 0, written before Key Pressure existed) decode as MEDIUM.
        ruido::DeviceSettingsBlob legacy = savedSettings;
        legacy.pressureTouchPlus1 = 0u;
        const auto* payload = reinterpret_cast<const uint8_t*>(&legacy) + sizeof(uint32_t);
        legacy.crc32 = ruido::computeCrc32Bytes(payload, sizeof(legacy) - sizeof(uint32_t));
        ruido::RuidoController legacyCtrl{};
        RTEST_CHECK(legacyCtrl.loadSettings(legacy));
        RTEST_CHECK(legacyCtrl.hud().config.pressureTouch == ruido::PressureTouch::Medium);
    }
    RTEST_CHECK(std::fabs(bootCtrl.hud().effectiveClockBpm - dialedHighBpm) < 0.1f);
    RTEST_CHECK(bootCtrl.snapshots().activeSlot() == 2u);
    RTEST_CHECK(std::fabs(bootCtrl.state().voices[0].frequencyHz - savedOsc1Hz) < 0.25f);

    // 6. Code-Review Regressions:
    //  (a) Analog `onBlackKeyPressure(ScopeFreezeKey, 1.0f)` snaps active OP1..OP3 blink to 100%
    //  (b) Steady 80% MUTE LED when a voice is audible solely via LFO -> LEVEL (`amplitude = 0`)
    //  (c) Simultaneous 150 ms crossfade when closing `SEL`, `SHIFT`, or `SAVE` (no 150 ms black
    //  gap)
    {
        ruido::RuidoController regCtrl{};
        regCtrl.tick(1.25f, ruido::EngineTelemetry{}); // move 0.40 Hz blink away from peak
        regCtrl.onBlackKeyPressure(ruido::BlackKeyId::ScopeFreezeKey, 1.0f);
        regCtrl.tick(0.0f, ruido::EngineTelemetry{});
        RTEST_CHECK(std::fabs(regCtrl.leds().blackKeyLeds[0] - 1.0f) < 0.01f);
        regCtrl.onBlackKeyPressure(ruido::BlackKeyId::ScopeFreezeKey, 0.0f);
        regCtrl.tick(0.0f, ruido::EngineTelemetry{});

        // Voice 0 audible via LFO A -> Amplitude while base amplitude == 0.0f:
        ruido::EngineControlState lfoAmpState = regCtrl.state();
        lfoAmpState.voices[0].amplitude = 0.0f;
        lfoAmpState.voices[0].lfoTarget = ruido::LfoTarget::Amplitude;
        lfoAmpState.voices[0].lfoDepth = 0.50f;
        RTEST_CHECK(std::fabs(ruido::evaluateVoiceSoundLedSteady(lfoAmpState, 0u) - 0.80f) <
                    1.0e-4f);

        // Closing SHIFT from Clean Scope crossfades the waveform back in during the 150 ms
        // fade-out:
        for (int f = 0; f < 12; ++f) {
            regCtrl.tick(0.020f, tel);
        }
        RTEST_CHECK(regCtrl.hud().mode == ruido::HudMode::None);
        ruido::ScopeHudRenderer fadeRen{};
        std::array<ruido::ScopeFrame, 256> waveFrames{};
        for (size_t i = 0u; i < waveFrames.size(); ++i) {
            const float ph = static_cast<float>(i) * (1.0f / 64.0f);
            const float s = (ph - static_cast<float>(static_cast<int>(ph)) < 0.5f) ? 0.6f : -0.6f;
            const auto q = static_cast<int16_t>(s * 8192.0f);
            waveFrames[i] = {q, q, {2048, 2048, 2048, 2048}, 0u};
        }
        fadeRen.pushScopeSamples(waveFrames.data(), waveFrames.size());
        regCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        for (int f = 0; f < 12; ++f) {
            regCtrl.tick(0.020f, tel);
            fadeRen.prepareFrame(0.020f, regCtrl.state(), regCtrl.hud(), tel);
        }
        regCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        // Step halfway through the 150 ms release fade (~80 ms): both HUD and scope are visible!
        for (int f = 0; f < 4; ++f) {
            regCtrl.tick(0.020f, tel);
            fadeRen.prepareFrame(0.020f, regCtrl.state(), regCtrl.hud(), tel);
        }
        RTEST_CHECK(regCtrl.hud().opacity > 0.20f && regCtrl.hud().opacity < 0.80f);
        // After another 90 ms (total 170 ms), HUD fade is completely finished (`opacity == 0`)
        for (int f = 0; f < 5; ++f) {
            regCtrl.tick(0.020f, tel);
            fadeRen.prepareFrame(0.020f, regCtrl.state(), regCtrl.hud(), tel);
        }
        RTEST_CHECK(regCtrl.hud().opacity <= 0.001f);
        RTEST_CHECK(regCtrl.hud().mode == ruido::HudMode::None);

        // (d) Hardware Regressions:
        //  - Zero `"THIS OSC"` / `"THIS OSCILLATOR"` across all keyPunchGuidanceDesc strings
        //  - SEL held-key Top-Bar `<NAME> OSC 1..4` + `<VALUE>` + full-width bottom pressure bar
        //  - SHIFT zero black knockout halo + full-width bottom pressure bar on SCENE squeeze
        //  - RAIL COUPLING replaces all HOME screens (`FREQUENCY`, `DUTY`, `DRIFT`) and restores
        //  - Top-Bar horizontal spinning number disk shifts smoothly across `x = 55..82`
        for (size_t k = 0u; k < ruido::kNumKeyPunchKinds; ++k) {
            const char* d = ruido::keyPunchGuidanceDesc(static_cast<ruido::KeyPunchKind>(k));
            RTEST_CHECK(std::strstr(d, "THIS OSC") == nullptr);
        }

        ruido::RuidoController hwCtrl{};
        ruido::ScopeHudRenderer hwRen{};
        ruido::EngineControlState hwQuietSt = hwCtrl.state();
        hwQuietSt.global.masterRun = false;
        hwQuietSt.global.masterTrim = 0.0f;
        static std::array<uint16_t, 240u * 240u> hwImg{};
        const auto hwG6 = [&](size_t x, size_t y) -> uint16_t {
            return (hwImg[y * 240u + x] >> 5u) & 0x3Fu;
        };

        // SEL -> CIRCUIT LAB (`2/3`): hold Key 1 (`CROSS-PIN OSC 1`) -> Top-Bar `CROSS-PIN OSC 1` +
        // `%` value + full-width bottom pressure bar (`y = 198..199`)
        hwCtrl.onButtonEvent(ruido::ButtonId::Sel, true);
        hwCtrl.onButtonEvent(ruido::ButtonId::Sel, false);
        hwCtrl.onButtonEvent(ruido::ButtonId::Sel, true);
        hwCtrl.onButtonEvent(ruido::ButtonId::Sel, false);
        RTEST_CHECK(hwCtrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
        hwCtrl.onWhiteKeyPressure(0u, 0.72f);
        for (int f = 0; f < 10; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        {
            char lm[24]{}, ls[16]{}, rt[32]{};
            bool sb = true;
            float bn = 0.0f;
            ruido::formatKeybedTopHeader(hwCtrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt,
                                         sizeof(rt), sb, bn);
            RTEST_CHECK(std::strcmp(lm, "CROSS-PIN") == 0 && std::strcmp(ls, " OSC 1") == 0 &&
                        std::strstr(rt, "%") != nullptr && !sb);
        }
        hwRen.renderFullScreenStrips(hwImg.data());
        RTEST_CHECK(hwG6(60u, 198u) >= 24u && hwG6(60u, 199u) >= 24u); // Bottom pressure bar lit!
        hwCtrl.onWhiteKeyGate(0u, false);
        hwCtrl.tick(0.020f, tel);
        {
            char lm[24]{}, ls[16]{}, rt[32]{};
            bool sb = false;
            float bn = 0.0f;
            ruido::formatKeybedTopHeader(hwCtrl.hud(), lm, sizeof(lm), ls, sizeof(ls), rt,
                                         sizeof(rt), sb, bn);
            RTEST_CHECK(std::strcmp(lm, "KEYBOARD") == 0 && std::strcmp(ls, " 2/3") == 0);
        }

        // SHIFT: chassis fill directly left of `"KEYBOARD"` (`x = 64, y = 48`) is NOT erased to 0,
        // and holding `SHIFT + Key 9` (`SCENE 1` squeeze) renders the bottom pressure bar!
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        hwCtrl.onWhiteKeyPressure(ruido::kSceneWhiteKeyBase + 0u, 0.75f);
        for (int f = 0; f < 10; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        hwRen.renderFullScreenStrips(hwImg.data());
        RTEST_CHECK(hwImg[48u * 240u + 76u] != 0u); // Inner LCD fill (`10u`) intact behind label!
        RTEST_CHECK(hwG6(80u, 198u) >= 24u);        // Bottom pressure bar lit on SHIFT + SCENE!
        hwCtrl.onWhiteKeyGate(ruido::kSceneWhiteKeyBase + 0u, false);
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);

        // RAIL COUPLING replaces HOME (`FREQUENCY`, `DUTY`, `DRIFT`) and restores HOME on timeout:
        hwCtrl.reset();
        hwCtrl.onButtonEvent(ruido::ButtonId::Home, true);
        hwCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        hwCtrl.onButtonEvent(ruido::ButtonId::Home, true);
        hwCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        RTEST_CHECK(hwCtrl.hud().freqPageOpen &&
                    hwCtrl.hud().homeTarget == ruido::HomeTarget::Width);
        hwCtrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8);
        hwCtrl.tick(0.020f, tel);
        RTEST_CHECK(hwCtrl.hud().mode == ruido::HudMode::RailCoupling &&
                    hwCtrl.hud().knobBank == ruido::KnobBankMode::None &&
                    ruido::HudGlobalViews::isRailCouplingActive(hwCtrl.hud()));
        for (int f = 0; f < 120; ++f) {
            hwCtrl.tick(0.025f, tel);
        }
        RTEST_CHECK(hwCtrl.hud().freqPageOpen && hwCtrl.hud().mode == ruido::HudMode::PulseWidth &&
                    hwCtrl.hud().knobBank == ruido::KnobBankMode::PulseWidth);

        // Top-Bar spinning number disk: parked Scene 1 vs 50/50 morph shifts horizontal centroid:
        hwCtrl.reset();
        hwRen.reset();
        hwCtrl.onEncoderDelta(ruido::EncoderId::Presets, 4);
        hwCtrl.onEncoderDelta(ruido::EncoderId::Presets, -4); // Pure Scene 1 (`loopPhase == 0`)
        for (int f = 0; f < 8; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        hwRen.renderFullScreenStrips(hwImg.data());
        uint32_t pureDialLum = 0u;
        for (size_t y = 6u; y <= 18u; ++y) {
            for (size_t x = 55u; x <= 82u; ++x) {
                pureDialLum += hwG6(x, y);
            }
        }
        RTEST_CHECK(pureDialLum > 40u);
        hwCtrl.onEncoderDelta(ruido::EncoderId::Presets, 24); // 50/50 morph (`loopPhase == 0.5`)
        for (int f = 0; f < 10; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        hwRen.renderFullScreenStrips(hwImg.data());
        // Center `x = 67` between `1` (`~58`) and `2` (`~76`) stays clear (zero vertical tick line)
        RTEST_CHECK(hwG6(67u, 12u) == 0u && hwG6(58u, 12u) > 0u && hwG6(76u, 12u) > 0u);

        // (e) Real-hardware SARADC unloading on LATCH release + SHIFT label & slow SAVE-style OP1-3
        // pulse:
        hwCtrl.reset();
        // Gesture 1: Hold Key 1 at 84% -> tap LATCH (`MONO`) -> gradual carbon-pill finger lift
        hwCtrl.onWhiteKeyPressure(0u, 0.84f);
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
        for (float unloadP : {0.55f, 0.25f, 0.08f, 0.015f, 0.003f}) {
            hwCtrl.onWhiteKeyPressure(0u, unloadP);
            RTEST_CHECK(nearF(hwCtrl.state().voicePressure[0], 0.84f, 0.01f));
        }
        hwCtrl.onWhiteKeyGate(0u, false);
        RTEST_CHECK(nearF(hwCtrl.state().voicePressure[0], 0.84f, 0.01f));

        // Gesture 2: While LATCH stays ARMED/LATCHED, press OP1 (`0.06 -> 0.90 -> unload -> 0`)
        for (float squeezeP : {0.06f, 0.35f, 0.70f, 0.90f, 0.45f, 0.12f, 0.01f}) {
            hwCtrl.onBlackKeyPressure(ruido::BlackKeyId::Op1, squeezeP);
        }
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::Op1, false);
        RTEST_CHECK(nearF(hwCtrl.state().keyMod.overloadSurge, 0.90f, 0.01f));

        // Re-press already-latched Key 1 at a lighter 42% squeeze (`0.06 -> 0.42 -> unload -> 0`):
        for (float rePressP : {0.06f, 0.22f, 0.42f, 0.18f, 0.02f}) {
            hwCtrl.onWhiteKeyPressure(0u, rePressP);
        }
        hwCtrl.onWhiteKeyGate(0u, false);
        RTEST_CHECK(nearF(hwCtrl.state().voicePressure[0], 0.42f, 0.01f));

        // SHIFT screen: `"SHIFT"` label above 11th key (`x = 192..218, y = 74..85`) & high-contrast
        // 0.50 Hz (`2.0 s`) OP1 pulse between trough (`t = 1.0 s`) and crest (`t = 2.0 s`):
        hwCtrl.reset();
        hwRen.reset();
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        hwCtrl.tick(0.010f, tel);
        RTEST_CHECK(hwCtrl.leds().blackKeyLeds[0] > 0.95f); // Snaps to peak on SHIFT press!
        // Step to t = 1.00 s (HUD 100% faded in, 0.50 Hz pulse at deep trough):
        for (int f = 0; f < 49; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        hwCtrl.tick(0.010f, tel);
        hwRen.prepareFrame(0.010f, hwQuietSt, hwCtrl.hud(), tel);
        RTEST_CHECK(hwCtrl.leds().blackKeyLeds[0] < 0.12f);
        hwRen.renderFullScreenStrips(hwImg.data());
        uint32_t shiftLblLum = 0u;
        uint32_t op1TroughLum = 0u;
        for (size_t y = 74u; y <= 85u; ++y) {
            for (size_t x = 192u; x <= 218u; ++x) {
                shiftLblLum += hwG6(x, y);
            }
        }
        for (size_t y = 96u; y <= 119u; ++y) {
            for (size_t x = 30u; x <= 40u; ++x) {
                op1TroughLum += hwG6(x, y);
            }
        }
        RTEST_CHECK(shiftLblLum > 80u);
        // Step another 1.00 s (`t = 2.00 s` = full crest at 100% HUD opacity):
        for (int f = 0; f < 50; ++f) {
            hwCtrl.tick(0.020f, tel);
            hwRen.prepareFrame(0.020f, hwQuietSt, hwCtrl.hud(), tel);
        }
        RTEST_CHECK(hwCtrl.leds().blackKeyLeds[0] > 0.95f);
        hwRen.renderFullScreenStrips(hwImg.data());
        uint32_t op1CrestLum = 0u;
        for (size_t y = 96u; y <= 119u; ++y) {
            for (size_t x = 30u; x <= 40u; ++x) {
                op1CrestLum += hwG6(x, y);
            }
        }
        RTEST_CHECK(op1CrestLum > op1TroughLum * 2u);
        hwCtrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    }
#undef RTEST_CHECK
}

} // namespace ruido_test
