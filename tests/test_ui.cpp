#include "TestGlobalAndInteraction.h"
#include "core/Parameters.h"
#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/ScopeHudRenderer.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

int gFailures = 0;

void check(bool cond, const char* expr, int line) {
    if (!cond) {
        std::fprintf(stderr, "  [FAIL] test_ui.cpp:%d: %s\n", line, expr);
        ++gFailures;
    }
}

bool near(float a, float b, float eps = 1.0e-4f) {
    return std::fabs(a - b) <= eps;
}

#define CHECK(expr) check((expr), #expr, __LINE__)

void testMemoryFootprint() {
    std::printf("[Test 1] Static RAM Footprint & Zero-Framebuffer Guarantee...\n");
    std::printf("  sizeof(RuidoController)  = %zu bytes\n", sizeof(ruido::RuidoController));
    std::printf("  sizeof(ScopeHudRenderer) = %zu bytes\n", sizeof(ruido::ScopeHudRenderer));
    CHECK(sizeof(ruido::RuidoController) < 2048u);
    // Full 240x240 RGB565 framebuffer would be 115,200 B; 4096-sample super-period history +
    // continuous phosphor persistence + 60x60 bloom + full-screen LFO stays < 86 KB without FB.
    CHECK(sizeof(ruido::ScopeHudRenderer) < 88064u);
}

void testTwoHandErgonomicsAndLinearGears() {
    std::printf("[Test 2] Stateless Two-Hand Modifiers & 3-Gear Linear Quadrature Encoders...\n");
    ruido::LockFreeControlBus bus;
    ruido::RuidoController ctrl;
    ctrl.attachControlBus(&bus);

    // 1. Strictly linear 3-gear X4 quadrature rates (96 steps/turn, ZERO velocity acceleration)
    const float f0 = ctrl.state().voices[0].frequencyHz;
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Standard);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 16);
    const float fFirst = ctrl.state().voices[0].frequencyHz;
    const float stepRatio = fFirst / f0;
    CHECK(stepRatio > 1.35f && stepRatio < 1.40f);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    CHECK(ctrl.hud().focusedVoice == 0u);

    ctrl.reset();
    ctrl.attachControlBus(&bus);
    for (int i = 0; i < 16; ++i) {
        ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 1); // 16 x 1-tick == 1 x 16-tick linear!
    }
    const float fAccum = ctrl.state().voices[0].frequencyHz;
    CHECK(near(fAccum, fFirst, 0.02f));

    // Momentary OCT- Fine hold: surgical ~0.66-cent steps while held, Standard on release
    ctrl.reset();
    ctrl.attachControlBus(&bus);
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Fine);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 16);
    const float fineRatio = ctrl.state().voices[0].frequencyHz / f0;
    CHECK(fineRatio > 1.005f && fineRatio < 1.008f);
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Standard);

    // Momentary OCT+ Coarse hold: full end-to-end sweep in ~1.33 turns (<= 132 X4 steps @ 96/turn),
    // including simultaneous OP1 + OCT+ chording, returning immediately to Standard on release
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Coarse);

    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -132);
    CHECK(near(ctrl.state().voices[0].frequencyHz, ruido::kAudioFreqMinHz));
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 132);
    CHECK(near(ctrl.state().voices[0].frequencyHz, ruido::kAudioFreqMaxHz));

    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 132);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 132);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 132);
    CHECK(near(ctrl.state().voices[0].hpCutoffHz, ruido::kFilterCutoffMaxHz));
    CHECK(near(ctrl.state().voices[0].hpResonance, 1.0f));
    CHECK(near(ctrl.state().voices[0].lpResonance, 1.0f));
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Coarse);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Standard);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(!ctrl.hud().fxPageOpen);

    ctrl.reset();
    ctrl.attachControlBus(&bus);

    // 2. Latching FX Filter Page + SELECT per-voice focus (OSC 1..4) + LIFO OP6/GLO chording
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
    CHECK(ctrl.hud().fxVoiceFocus == 0u);
    CHECK(ctrl.hud().focusedVoice == 0u);

    // Step SELECT +2 detents -> OSC 3 (voice 2); clamp at OSC 4 (voice 3) with zero windup
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);
    CHECK(ctrl.hud().fxVoiceFocus == 3u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 120);
    CHECK(ctrl.hud().fxVoiceFocus == 3u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    CHECK(ctrl.hud().fxVoiceFocus == 2u);
    CHECK(ctrl.hud().focusedVoice == 2u);

    // Turn KNOB1..4 while FX focuses OSC 3 -> updates Voice 2 HPF, HP RES, LPF, LP RES only
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 20);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 30);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -20);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 30);
    CHECK(ctrl.state().voices[2].hpCutoffHz > 2.2f);
    CHECK(ctrl.state().voices[2].hpResonance > 0.10f);
    CHECK(ctrl.state().voices[2].lpCutoffHz < 27000.0f);
    CHECK(ctrl.state().voices[2].lpResonance > 0.10f);
    CHECK(near(ctrl.state().voices[0].hpCutoffHz, ruido::kFilterCutoffMinHz));

    // Close FX page via 2nd FX tap -> returns KNOB1-4 to Frequency
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(!ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);

    // LIFO chording across remaining momentary black keys (OP6 Panning + Keybed GLO)
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAll);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    CHECK(!ctrl.hud().modifierHeld);

    // 3. EDIT Page (Channel Level & MS-20 Drive), OP5 (Unassigned), OP6 (Panning), Keybed GLO
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::GlobalAll);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalAll);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -12); // Step All(4) -> Filter(1)
    CHECK(ctrl.hud().mode == ruido::HudMode::GlobalFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::GlobalFilter);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 30);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 30);
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        CHECK(ctrl.state().voices[i].hpResonance > 0.08f);
        CHECK(ctrl.state().voices[i].lpResonance > 0.08f);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -30);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    CHECK(ctrl.state().voices[3].pan < 0.65f);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);

    // OP1..OP5 are all unassigned (show UnassignedKey, leave KNOB1-4 on Frequency)
    for (uint8_t op = 0u; op <= static_cast<uint8_t>(ruido::BlackKeyId::Op5); ++op) {
        const auto key = static_cast<ruido::BlackKeyId>(op);
        ctrl.onBlackKeyEvent(key, true);
        CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedKey);
        CHECK(ctrl.hud().lastBlackKey == key);
        CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
        ctrl.onBlackKeyEvent(key, false);
    }
    CHECK(near(ctrl.state().voices[1].amplitude, 0.65f));

    // Tap EDIT button -> opens latching Channel Level & MS-20 Overdrive page
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(ctrl.hud().editPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -30);
    CHECK(ctrl.state().voices[1].amplitude < 0.65f);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(!ctrl.hud().editPageOpen);

    // 4. Mute memory + EDIT Amplitude/Drive page 0% display & fade-in unmute:
    const float rememberedAmp = ctrl.state().voices[0].amplitude;
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(ctrl.state().voices[0].muted);
    CHECK(near(ctrl.state().voices[0].amplitude, rememberedAmp));

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -3);
    CHECK(ctrl.state().voices[0].muted);
    CHECK(near(ctrl.state().voices[0].amplitude, rememberedAmp));
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);

    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(!ctrl.state().voices[0].muted);
    CHECK(near(ctrl.state().voices[0].amplitude, rememberedAmp));

    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(ctrl.state().voices[0].muted);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
    CHECK(!ctrl.state().voices[0].muted);
    CHECK(ctrl.state().voices[0].amplitude > 0.005f && ctrl.state().voices[0].amplitude < 0.05f);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);

    // Verify lock-free bus received latest published state
    const ruido::EngineControlState& snapped = bus.consumeLatest();
    CHECK(near(snapped.voices[2].hpCutoffHz, ctrl.state().voices[2].hpCutoffHz));
}

void testAllControlsDisplayOnScreen() {
    std::printf("[Test 3] All 27 Keys, 14 Panel Switches & 8 Knobs Display HUD Feedback...\n");
    ruido::RuidoController ctrl;

    // 1. Every single Black Key (all 11: OP1..6, PIT, GLO, MONO, POLY, ScopeFreezeKey)
    for (uint8_t b = 0u; b < static_cast<uint8_t>(ruido::BlackKeyId::Count); ++b) {
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        CHECK(ctrl.hud().mode == ruido::HudMode::None);

        const auto key = static_cast<ruido::BlackKeyId>(b);
        ctrl.onBlackKeyEvent(key, true);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
        ctrl.onBlackKeyEvent(key, false);
    }

    // 2. Every single White Key (all 16: Gates 1..4, Pressure Pads 5..8, Snapshots 9..16)
    for (uint8_t w = 0u; w < ruido::kNumWhiteKeys; ++w) {
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        CHECK(ctrl.hud().mode == ruido::HudMode::None);

        ctrl.onWhiteKeyGate(w, true);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
        ctrl.onWhiteKeyGate(w, false);
    }

    // 3. Every single Panel Switch (except HOME, whose role is dismissing the HUD)
    for (uint8_t btn = 0u; btn < static_cast<uint8_t>(ruido::ButtonId::Count); ++btn) {
        const auto button = static_cast<ruido::ButtonId>(btn);
        if (button == ruido::ButtonId::Home) {
            continue;
        }
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        ctrl.onButtonEvent(button, true);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
        ctrl.onButtonEvent(button, false);
    }

    // 4. All 7 Encoders + MASTER Potentiometer
    for (uint8_t e = 0u; e < static_cast<uint8_t>(ruido::EncoderId::Count); ++e) {
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        ctrl.onEncoderDelta(static_cast<ruido::EncoderId>(e), 2);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
    }
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onMasterPot(0.42f);
    CHECK(ctrl.hud().mode == ruido::HudMode::MasterLevel);
}

void testButtonsOctavesKeybedAndPressure() {
    std::printf("[Test 4] Global Knobs, Octaves, Single-Taps & Keybed Pressure...\n");
    ruido::RuidoController ctrl;

    // Global encoders & unassigned SELECT encoder reservation
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 40);
    CHECK(ctrl.state().global.circuitCoupling > 0.30f);

    ctrl.onEncoderDelta(ruido::EncoderId::Select, -40);
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedEncoder);

    // Momentary OCT- / OCT+ Knob Rate holds, LIFO chording, LEDs & immediate release dissolve
    ruido::EngineTelemetry tel{};
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Standard);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 0.15f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 0.15f));

    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, true);
    ctrl.tick(0.250f, tel);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Fine);
    CHECK(ctrl.hud().mode == ruido::HudMode::KnobRate);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 0.15f));

    // LIFO chording: pressing OCT+ while OCT- is held promotes to Coarse; releasing OCT+ restores
    // Fine
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Coarse);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 0.15f));

    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Fine);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctPlus)], 0.15f));

    // Releasing OCT- immediately returns to Standard, dims LED to 15%, and dissolves in 150 ms
    ctrl.onButtonEvent(ruido::ButtonId::OctMinus, false);
    CHECK(ctrl.hud().knobRate == ruido::KnobRate::Standard);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::OctMinus)], 0.15f));
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));

    // LFO and FX buttons open their latching pages without altering base voice frequency;
    // unassigned switches (SEL, ENV, ARP, SEQ, REC) display UnassignedButton:
    const float v1BeforeLfo = ctrl.state().voices[1].frequencyHz;
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(ctrl.hud().lfoPageOpen);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    CHECK(near(ctrl.state().voices[1].frequencyHz, v1BeforeLfo));
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    CHECK(!ctrl.hud().lfoPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(!ctrl.hud().fxPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Env, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedButton);
    ctrl.onButtonEvent(ruido::ButtonId::Env, false);

    // PIT & SEL (both unassigned), MONO, POLY, PLAY/STOP
    const float v0BeforePit = ctrl.state().voices[0].frequencyHz;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedKey);
    CHECK(!ctrl.state().global.pitchPressureMode);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Pit, false);

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(!ctrl.state().global.pitchPressureMode);
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedButton);
    CHECK(ctrl.hud().lastButton == ruido::ButtonId::Sel);

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    CHECK(ctrl.state().global.monoCollapse);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, true);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Poly, false);
    CHECK(!ctrl.state().global.monoCollapse);

    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    CHECK(!ctrl.state().global.masterRun);

    // White Keys 0..3 latching gates & White Keys 4..7 continuous filter/LPG pressure (0 pitch mod)
    CHECK(!ctrl.state().voices[0].muted);
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(ctrl.state().voices[0].muted);

    ctrl.onWhiteKeyGate(5u, true);
    CHECK(ctrl.state().voicePressure[1] > 0.0f);
    ctrl.onWhiteKeyPressure(5u, 0.82f);
    CHECK(near(ctrl.state().voicePressure[1], 0.82f));
    CHECK(near(ctrl.state().voices[0].frequencyHz, v0BeforePit));
    ctrl.onWhiteKeyGate(5u, false);
    CHECK(near(ctrl.state().voicePressure[1], 0.0f));
}

void testHudEnvelopeAndVaribrightLeds() {
    std::printf("[Test 5] Soft S-Curve HUD Envelope & 42-Channel Hardware LED Engine...\n");
    ruido::RuidoController ctrl;
    ruido::EngineTelemetry tel{};

    // Hold OP6: unified 150 ms S-curve fade-in (smoothstep01), pinned indefinitely while held
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    ctrl.tick(0.030f, tel); // 20% of 150 ms -> smoothstep01(0.2) = 0.104 (eased start!)
    CHECK(ctrl.hud().opacity > 0.08f && ctrl.hud().opacity < 0.14f);
    ctrl.tick(0.045f, tel); // 50% of 150 ms (75 ms total) -> smoothstep01(0.5) = 0.500
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    for (int i = 0; i < 10; ++i) {
        ctrl.tick(0.050f, tel);
    }
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // Release OP6: momentary hold dissolves smoothly in 150 ms without 1.5 s freeze
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.tick(0.075f, tel); // 50% of 150 ms fade-out -> 0.50
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));

    ctrl.tick(0.100f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);

    // Instantaneous encoder turn: blooms in 150 ms, holds readable for 1.5 s, dissolves in 150 ms
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 2);
    for (int i = 0; i < 16; ++i) {
        ctrl.tick(0.100f, tel); // 0.15 s bloom + 1.45 s hold -> still 1.0!
    }
    CHECK(near(ctrl.hud().opacity, 1.0f));
    ctrl.tick(0.125f, tel); // 0.05 s remaining hold + 0.075 s fade -> halfway dissolved (0.50)
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));

    // HOME button fades active HUD out smoothly in 150 ms (including HOME LED)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 3);
    ctrl.tick(0.200f, tel);
    CHECK(ctrl.hud().opacity > 0.99f);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)] > 0.20f &&
          ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)] < 0.27f);
    ctrl.tick(0.100f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);

    // Right 2x6 pages & switches (FX, LFO, EDIT, GLO, SAVE, unassigned SEL/ENV) stay latched until
    // closed or HOME:
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    for (int i = 0; i < 60; ++i) {
        ctrl.tick(0.200f, tel); // 12.0 s elapsed -> FX Filter page stays latched at 1.0f!
    }
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 1.0f));

    // Holding & releasing a momentary modifier (OP6 / GLO / Pad 5) returns to latched FX page:
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    ctrl.onWhiteKeyGate(4u, true);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoicePressure);
    ctrl.onWhiteKeyGate(4u, false);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // Tap SEL (unassigned) -> closes FX page and latches UnassignedButton until HOME:
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(!ctrl.hud().fxPageOpen);
    CHECK(!ctrl.state().global.pitchPressureMode);
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedButton);

    // Standalone knob tweak while switch is latched shows tweak for 1.5 s then returns to switch:
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 2);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    for (int i = 0; i < 16; ++i) {
        ctrl.tick(0.100f, tel);
    }
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedButton);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // LFO, ENV, and SAVE also stay latched until HOME or SAVE completion/cancel:
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> LFO page stays latched!
    }
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    ctrl.onButtonEvent(ruido::ButtonId::Env, true);
    ctrl.onButtonEvent(ruido::ButtonId::Env, false);
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> switch HUD still latched!
    }
    CHECK(ctrl.hud().mode == ruido::HudMode::UnassignedButton);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> SAVE stays armed & latched!
    }
    CHECK(ctrl.hud().saveArmed);
    CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // Second tap on SAVE shows 1.5 s CANCELLED toast then fades in 150 ms; HOME fades out in 150 ms
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    CHECK(!ctrl.hud().saveArmed);
    CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);
    for (int i = 0; i < 14; ++i) {
        ctrl.tick(0.100f, tel);
    }
    CHECK(near(ctrl.hud().opacity, 1.0f));
    ctrl.tick(0.175f, tel); // 0.10 s remaining hold + 0.075 s fade -> ~0.50
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.160f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);

    // Verify unassigned buttons (incl. SEL), OP1..OP5/PIT keys, & resting 11th black key are
    // strictly 0% dark, while assigned FX, LFO, EDIT & GLO right buttons rest at 15%:
    const auto& leds = ctrl.leds();
    for (size_t op = 0u; op <= static_cast<size_t>(ruido::BlackKeyId::Op5); ++op) {
        CHECK(near(leds.blackKeyLeds[op], 0.0f));
    }
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Pit)], 0.0f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::ScopeFreezeKey)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Arp)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Seq)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)], 0.0f));

    // White Keys 5..8 rest at kPadFilterBaseLed (0.24) and flare under pressure while SEL stays 0%
    // and FX LED flares dynamically with live filter engagement!
    for (size_t i = 0u; i < 4u; ++i) {
        CHECK(near(leds.whiteKeyLeds[4u + i], ruido::kPadFilterBaseLed));
    }
    ctrl.onWhiteKeyPressure(4u, 0.75f);
    ctrl.tick(0.016f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 0.0f));
    CHECK(ctrl.leds().whiteKeyLeds[4u] > 0.80f);
    CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.45f &&
          ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 0.85f);
    ctrl.onWhiteKeyGate(4u, false);
    ctrl.tick(0.016f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 0.15f));

    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.tick(0.016f, tel);
    CHECK(near(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::ScopeFreezeKey)],
               1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);

    // Verify OP6 & GLO landmarks rest at 25%; OP1..OP5 = 0%;
    // and verify White Keys 1-4 smooth Mute pulse (1.50 Hz, 4..54%) vs slow Sub-Audio breath (0.35
    // Hz, 14..70%):
    CHECK(near(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op1)], 0.0f));
    CHECK(near(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op5)], 0.0f));
    CHECK(near(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op6)], 0.25f));
    CHECK(near(ctrl.leds().blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Glo)], 0.25f));
    const float steadyAudible0 = 0.60f + 0.40f * ctrl.state().voices[0].amplitude;
    CHECK(near(ctrl.leds().whiteKeyLeds[0], steadyAudible0));

    // Tap White Key 1 -> user-activated Mute: S-curve onset (no harsh 1-frame snap!) into 1.50 Hz
    // pulse
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(ctrl.state().voices[0].muted);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.leds().whiteKeyLeds[0] > steadyAudible0 - 0.12f); // Smooth onset, zero harsh jump!

    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.016f, tel); // Settle onto 1.50 Hz mute wave (~0.48 s)
    }
    float muteMin = 1.0f;
    float muteMax = 0.0f;
    float maxMuteStep = 0.0f;
    float prevMuteLed = ctrl.leds().whiteKeyLeds[0];
    for (int i = 0; i < 90; ++i) { // ~1.44 s (> 2 full 1.50 Hz cycles)
        ctrl.tick(0.016f, tel);
        const float v = ctrl.leds().whiteKeyLeds[0];
        const float d = std::fabs(v - prevMuteLed);
        if (v < muteMin) {
            muteMin = v;
        }
        if (v > muteMax) {
            muteMax = v;
        }
        if (d > maxMuteStep) {
            maxMuteStep = d;
        }
        prevMuteLed = v;
    }
    CHECK(muteMin < 0.07f);
    CHECK(muteMax > 0.50f && muteMax < 0.58f);
    CHECK(maxMuteStep > 0.030f && maxMuteStep < 0.055f); // Smooth continuous C^2 sine, zero snaps

    // Unmute White Key 1 -> smooth bloom back to steady audible glow, then sweep < 20 Hz into slow
    // 0.35 Hz breath!
    ctrl.onWhiteKeyGate(0u, true);
    ctrl.onWhiteKeyGate(0u, false);
    CHECK(!ctrl.state().voices[0].muted);
    const float preUnmuteStepLed = ctrl.leds().whiteKeyLeds[0];
    ctrl.tick(0.016f, tel);
    CHECK(std::fabs(ctrl.leds().whiteKeyLeds[0] - preUnmuteStepLed) < 0.12f);
    for (int i = 0; i < 40; ++i) {
        ctrl.tick(0.016f, tel);
    }
    CHECK(near(ctrl.leds().whiteKeyLeds[0], steadyAudible0, 1.0e-3f));

    // Sweep Osc 1 down below hearing range (~1 Hz) via COARSE + Knob1 -> slow 0.35 Hz breathing!
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -110);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    CHECK(ctrl.state().voices[0].frequencyHz < 5.0f);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.leds().whiteKeyLeds[0] > steadyAudible0 - 0.10f); // Smooth frequency-threshold slew!
    for (int i = 0; i < 45; ++i) {
        ctrl.tick(0.016f, tel);
    }
    float breathMin = 1.0f;
    float breathMax = 0.0f;
    float maxBreathStep = 0.0f;
    float prevBreathLed = ctrl.leds().whiteKeyLeds[0];
    for (int i = 0; i < 200; ++i) { // ~3.2 s (> 1 full 2.86 s breath cycle)
        ctrl.tick(0.016f, tel);
        const float v = ctrl.leds().whiteKeyLeds[0];
        const float d = std::fabs(v - prevBreathLed);
        if (v < breathMin) {
            breathMin = v;
        }
        if (v > breathMax) {
            breathMax = v;
        }
        if (d > maxBreathStep) {
            maxBreathStep = d;
        }
        prevBreathLed = v;
    }
    CHECK(breathMin > 0.12f && breathMin < 0.18f);
    CHECK(breathMax > 0.66f && breathMax < 0.74f);
    CHECK(maxBreathStep < 0.025f); // >3x slower & gentler than the 1.50 Hz user-activated Mute!

    // Verify dual-channel PLAY/STOP split cap:
    // - Sound ON (AUDIO OUTPUT = ON): PLAY (Green) = 0.0, STOP (Red) = 1.0
    // - Mid-transition (~32 ms): both channels > 0 (smooth Yellow/Amber optical crossfade)
    // - Quiet (AUDIO OUTPUT = OFF settled): PLAY (Green) = 1.0, STOP (Red) = 0.0
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::PlayStop)], 0.0f));
    CHECK(near(ctrl.leds().stopLed, 1.0f));

    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    ctrl.tick(0.032f, tel);
    CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::PlayStop)] > 0.15f &&
          ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::PlayStop)] < 0.85f);
    CHECK(ctrl.leds().stopLed > 0.15f && ctrl.leds().stopLed < 0.85f);

    for (int i = 0; i < 20; ++i) {
        ctrl.tick(0.050f, tel);
    }
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::PlayStop)], 1.0f));
    CHECK(near(ctrl.leds().stopLed, 0.0f));

    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    for (int i = 0; i < 20; ++i) {
        ctrl.tick(0.050f, tel);
    }
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::PlayStop)], 0.0f));
    CHECK(near(ctrl.leds().stopLed, 1.0f));
}

void testMonochromeCrtScanlineAndUncompressedWave() {
    std::printf("[Test 6] Monochrome CRT Equivalence, Pure B&W & Zero Wave Compression...\n");
    ruido::DroneEngine engine;
    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;

    engine.init(ruido::kBaseSampleRate);
    ctrl.attachControlBus(&engine.controlBus());

    // Open the latching FX Filter page so live CRT beam + HUD + MS-20 filter curve render
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 32);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 90);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -32);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 120);

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    for (int f = 0; f < 6; ++f) {
        for (int b = 0; b < 10; ++b) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            renderer.ingestScopeRing(engine.scopeBuffer());
        }
        ctrl.tick(0.050f, engine.telemetry());
        renderer.prepareFrame(0.016f, ctrl.state(), ctrl.hud(), engine.telemetry());
    }

    // 1. Render via 15 x (240x16) strips vs 30 x (240x8) vs 10 x (240x24)
    static std::array<uint16_t, 240u * 240u> img16{};
    static std::array<uint16_t, 240u * 240u> img8{};
    static std::array<uint16_t, 240u * 240u> img24{};

    renderer.renderFullScreenStrips(img16.data());
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
        renderer.renderStrip(y, 8u, img8.data() + static_cast<size_t>(y) * 240u);
    }
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
        renderer.renderStrip(y, 24u, img24.data() + static_cast<size_t>(y) * 240u);
    }

    bool identical8 = true;
    bool identical24 = true;
    bool strictlyMonochrome = true;
    size_t litPixels = 0u;
    size_t pitchBlackPixels = 0u;

    for (size_t i = 0u; i < img16.size(); ++i) {
        const uint16_t px = img16[i];
        if (px != img8[i]) {
            identical8 = false;
        }
        if (px != img24[i]) {
            identical24 = false;
        }
        const uint16_t r5 = (px >> 11u) & 0x1Fu;
        const uint16_t g6 = (px >> 5u) & 0x3Fu;
        const uint16_t b5 = px & 0x1Fu;
        if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
            strictlyMonochrome = false;
        }
        if (px == 0x0000u) {
            ++pitchBlackPixels;
        }
        if (px > 0x18E3u) {
            ++litPixels;
        }
    }
    CHECK(identical8);
    CHECK(identical24);
    CHECK(strictlyMonochrome);
    CHECK(pitchBlackPixels > 25000u); // Pitch-black #000000 background preserved
    CHECK(litPixels > 1200u);

    // 2. Verify zero vertical waveform compression when non-filter HUD opens vs closes:
    // Close FX page, let the filter backdrop dim recovery settle cleanly, then compare ScopeFreeze
    // HUD open (opacity = 1.0) vs HUD dismissed (opacity = 0.0) on the exact frozen trace!
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.tick(0.250f, engine.telemetry());
    for (int settle = 0; settle < 16; ++settle) {
        renderer.prepareFrame(0.150f, ctrl.state(), ctrl.hud(), engine.telemetry());
    }
    renderer.renderFullScreenStrips(img16.data()); // ScopeFreeze HUD opacity == 1.0

    ruido::HudState hiddenHud = ctrl.hud();
    hiddenHud.opacity = 0.0f;
    hiddenHud.mode = ruido::HudMode::None;
    hiddenHud.scopeFrozen = true;
    renderer.prepareFrame(0.016f, ctrl.state(), hiddenHud, engine.telemetry());
    renderer.renderFullScreenStrips(img8.data()); // HUD opacity == 0.0

    bool centerWaveUncompressed = true;
    size_t centerLit = 0u;
    for (size_t y = 48u; y < 136u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            const uint16_t pHud = img16[y * 240u + x];
            const uint16_t pIdle = img8[y * 240u + x];
            if (pHud != pIdle) {
                centerWaveUncompressed = false;
            }
            if (pIdle > 0x1082u) {
                ++centerLit;
            }
        }
    }
    CHECK(centerWaveUncompressed);
    CHECK(centerLit > 400u);

    // 3. Verify idle CRT trace spans all 240 horizontal columns edge-to-edge
    size_t colsWithTrace = 0u;
    for (size_t x = 0u; x < 240u; ++x) {
        bool colLit = false;
        for (size_t y = 4u; y < 236u; ++y) {
            if (img8[y * 240u + x] > 0x1082u) {
                colLit = true;
                break;
            }
        }
        if (colLit) {
            ++colsWithTrace;
        }
    }
    CHECK(colsWithTrace == 240u);

    // 4. Verify GLO mixer renders 0% identically for muted voices regardless of remembered level
    ruido::EngineControlState stMuteA = ctrl.state();
    stMuteA.voices[0].muted = true;
    stMuteA.voices[0].amplitude = 0.85f;
    ruido::EngineControlState stMuteB = stMuteA;
    stMuteB.voices[0].amplitude = 0.18f;

    ruido::HudState gloHud = ctrl.hud();
    gloHud.mode = ruido::HudMode::Amplitude;
    gloHud.knobBank = ruido::KnobBankMode::Amplitude;
    gloHud.opacity = 1.0f;
    gloHud.scopeFrozen = true;

    renderer.prepareFrame(0.016f, stMuteA, gloHud, engine.telemetry());
    renderer.renderFullScreenStrips(img16.data());
    renderer.prepareFrame(0.016f, stMuteB, gloHud, engine.telemetry());
    renderer.renderFullScreenStrips(img8.data());

    bool mutedMixerShowsZero = true;
    for (size_t y = 190u; y < 236u; ++y) {
        for (size_t x = 0u; x < 60u; ++x) {
            if (img16[y * 240u + x] != img8[y * 240u + x]) {
                mutedMixerShowsZero = false;
            }
        }
    }
    CHECK(mutedMixerShowsZero);

    // 5. Verify coupled 4-voice caustic ribbon responds to muting Osc 2-4 (solo Osc 1)
    // while keeping open edge-to-edge 240-column coverage and organic frame-to-frame motion.
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    renderer.prepareFrame(0.016f, ctrl.state(), ctrl.hud(), engine.telemetry());
    renderer.renderFullScreenStrips(img24.data()); // Live 4-voice caustic mesh

    for (uint8_t w = 1u; w < 4u; ++w) {
        ctrl.onWhiteKeyGate(w, true);
        ctrl.onWhiteKeyGate(w, false);
    }
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);

    for (int b = 0; b < 18; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        renderer.ingestScopeRing(engine.scopeBuffer());
    }
    renderer.prepareFrame(0.032f, ctrl.state(), ctrl.hud(), engine.telemetry());
    renderer.renderFullScreenStrips(img16.data()); // Solo Osc 1 frame A

    for (int b = 0; b < 10; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        renderer.ingestScopeRing(engine.scopeBuffer());
    }
    renderer.prepareFrame(0.032f, ctrl.state(), ctrl.hud(), engine.telemetry());
    renderer.renderFullScreenStrips(img8.data()); // Solo Osc 1 frame B

    size_t chordVsSoloDiffs = 0u;
    size_t soloFrameMotionDiffs = 0u;
    size_t soloColsLit = 0u;
    for (size_t x = 0u; x < 240u; ++x) {
        bool colLit = false;
        for (size_t y = 20u; y < 220u; ++y) {
            const size_t idx = y * 240u + x;
            if (img24[idx] != img16[idx]) {
                ++chordVsSoloDiffs;
            }
            if (img16[idx] != img8[idx]) {
                ++soloFrameMotionDiffs;
            }
            if (img16[idx] > 0x1082u) {
                colLit = true;
            }
        }
        if (colLit) {
            ++soloColsLit;
        }
    }
    CHECK(chordVsSoloDiffs > 500u);
    CHECK(soloFrameMotionDiffs > 40u);
    CHECK(soloColsLit == 240u);

    // 6. Verify 4096-sample super-period lock & phosphor persistence eliminate 30 Hz chord tremble:
    // Part A: Pure 3:2 fifth + 2:1 + 3:1 lock (55, 82.5, 110, 165 Hz) advancing by ~17.4 ms
    // (~0.96 cycles of 55 Hz, where naive single-cycle triggering flips Osc 2 upside-down!)
    ruido::DroneEngine calmEngine;
    ruido::RuidoController calmCtrl;
    ruido::ScopeHudRenderer calmRenderer;
    calmEngine.init(ruido::kBaseSampleRate);
    calmEngine.setAnalogCharacterEnabled(false, true);

    ruido::EngineControlState lockedSt = ruido::EngineControlState::makeDefault();
    lockedSt.voices[0].frequencyHz = 55.0f;
    lockedSt.voices[1].frequencyHz = 82.5f;
    lockedSt.voices[2].frequencyHz = 110.0f;
    lockedSt.voices[3].frequencyHz = 165.0f;
    calmEngine.controlBus().publish(lockedSt);
    calmEngine.snapParametersToLatest();

    // Settle initial boot AC-coupling capacitor transient (~0.5 s) before measuring scope lock
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> scopeDrain{};
    for (int b = 0; b < 360; ++b) {
        calmEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        calmEngine.scopeBuffer().popBatch(scopeDrain.data(), scopeDrain.size());
    }

    for (int f = 0; f < 10; ++f) {
        for (int b = 0; b < 12; ++b) {
            calmEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            calmRenderer.ingestScopeRing(calmEngine.scopeBuffer());
        }
        calmRenderer.prepareFrame(0.0174f, lockedSt, calmCtrl.hud(), calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img16.data());

    for (int b = 0; b < 12; ++b) {
        calmEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        calmRenderer.ingestScopeRing(calmEngine.scopeBuffer());
    }
    calmRenderer.prepareFrame(0.0174f, lockedSt, calmCtrl.hud(), calmEngine.telemetry());
    calmRenderer.renderFullScreenStrips(img8.data());

    auto measureMeanGrayDelta = [&]() -> double {
        double sum = 0.0;
        for (size_t i = 0u; i < img16.size(); ++i) {
            const int gA = static_cast<int>((img16[i] >> 5u) & 0x3Fu) * 4;
            const int gB = static_cast<int>((img8[i] >> 5u) & 0x3Fu) * 4;
            sum += static_cast<double>((gA >= gB) ? (gA - gB) : (gB - gA));
        }
        return sum / static_cast<double>(img16.size());
    };
    const double lockedDelta = measureMeanGrayDelta();

    // Part B: Default free-beating boot chord (55 + 82.41 + 110.15 + 164.81 Hz + analog drift)
    calmEngine.setAnalogCharacterEnabled(true, true);
    calmEngine.controlBus().publish(calmCtrl.state());
    calmEngine.snapParametersToLatest();
    calmRenderer.reset();
    for (int f = 0; f < 10; ++f) {
        for (int b = 0; b < 12; ++b) {
            calmEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            calmRenderer.ingestScopeRing(calmEngine.scopeBuffer());
        }
        calmRenderer.prepareFrame(0.0174f, calmCtrl.state(), calmCtrl.hud(),
                                  calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img16.data());
    for (int b = 0; b < 12; ++b) {
        calmEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        calmRenderer.ingestScopeRing(calmEngine.scopeBuffer());
    }
    calmRenderer.prepareFrame(0.0174f, calmCtrl.state(), calmCtrl.hud(), calmEngine.telemetry());
    calmRenderer.renderFullScreenStrips(img8.data());
    const double beatingDelta = measureMeanGrayDelta();

    std::printf("  Super-period stability |dLum|: locked 3:2 fifth = %.3f, free-beating = %.3f\n",
                lockedDelta, beatingDelta);
    CHECK(lockedDelta < 0.8);
    CHECK(beatingDelta < 7.0);

    // 7. Verify Concept A3 1/3-Screen (y = 102..182, 80 px) Dual-Resonance Filter Graph,
    //    0% Voice Offset on OP5, and smooth 70% BG Wave Dimming transition on OP1 hold/release:
    ruido::RuidoController a3Ctrl;
    ruido::EngineControlState a3State = ruido::EngineControlState::makeDefault();
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        a3State.voices[v].hpCutoffHz = 150.0f;
        a3State.voices[v].hpResonance = 0.72f;
        a3State.voices[v].lpCutoffHz = 1500.0f;
        a3State.voices[v].lpResonance = 0.78f;
    }

    // Freeze calmRenderer's scope trace so background luminance is 100% deterministic across steps
    ruido::HudState idleFrozen{};
    idleFrozen.mode = ruido::HudMode::None;
    idleFrozen.opacity = 0.0f;
    idleFrozen.scopeFrozen = true;
    calmRenderer.prepareFrame(0.016f, a3State, idleFrozen, calmEngine.telemetry());
    calmRenderer.renderFullScreenStrips(img24.data()); // Baseline 100% undimmed background

    auto sumUpperBandGray = [](const std::array<uint16_t, 240u * 240u>& img) -> double {
        double sum = 0.0;
        // Measure y = 40..92 (strictly outside top header 0..34 and filter graph 95..188)
        for (size_t y = 40u; y < 92u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                sum += static_cast<double>((img[y * 240u + x] >> 5u) & 0x3Fu) * 4.0;
            }
        }
        return sum;
    };
    const double lumIdle = sumUpperBandGray(img24);
    CHECK(lumIdle > 5000.0);

    // Open FX Filter page: step 64 ms (4 x 16 ms) -> mid-attack dimming; then settle -> ~72% dimmed
    a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    for (int step = 0; step < 4; ++step) {
        a3Ctrl.tick(0.016f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.016f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img16.data());
    const double lumMidAttack = sumUpperBandGray(img16);

    for (int step = 0; step < 20; ++step) {
        a3Ctrl.tick(0.080f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.080f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img16.data());
    const double lumSettledFilter = sumUpperBandGray(img16);
    const double dimRatio = lumSettledFilter / lumIdle;
    CHECK(lumMidAttack < lumIdle * 0.96);
    CHECK(lumMidAttack > lumSettledFilter * 1.15);
    CHECK(dimRatio > 0.24 && dimRatio < 0.36); // 28-30% kept luminance == ~72% BG Wave Dimming

    // Verify 80 px 1/3-screen filter zone: resonant horns reach y = 102..126 & A3 fill reaches 182
    size_t hornPixels = 0u;
    size_t a3FillPixels = 0u;
    for (size_t x = 20u; x < 220u; ++x) {
        for (size_t y = 102u; y < 126u; ++y) {
            if (img16[y * 240u + x] > 0x3186u) {
                ++hornPixels;
            }
        }
        for (size_t y = 148u; y <= 182u; ++y) {
            if (img16[y * 240u + x] > img24[y * 240u + x]) {
                ++a3FillPixels;
            }
        }
    }
    CHECK(hornPixels > 80u);
    CHECK(a3FillPixels > 1200u);

    // Verify 0% Voice Offset: GLO GlobalFilter matches FX VoiceFilter pixel-for-pixel (y=102..182)
    a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    a3Ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    a3Ctrl.onEncoderDelta(ruido::EncoderId::Select, -12); // GlobalTarget::Filter (1)
    for (int step = 0; step < 12; ++step) {
        a3Ctrl.tick(0.080f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.080f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img8.data());
    bool zeroVoiceOffsetIdentical = true;
    for (size_t y = 102u; y <= 182u; ++y) {
        for (size_t x = 10u; x <= 230u; ++x) {
            if (img16[y * 240u + x] != img8[y * 240u + x]) {
                zeroVoiceOffsetIdentical = false;
            }
        }
    }
    CHECK(zeroVoiceOffsetIdentical);

    // Release GLO: verify smooth 150 ms recovery transition (no 1-frame snap) back to 100%
    a3Ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    for (int step = 0; step < 4; ++step) { // 4 x 16 ms = 64 ms (~43% of 150 ms fade)
        a3Ctrl.tick(0.016f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.016f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img8.data());
    const double lumMidRelease = sumUpperBandGray(img8);
    CHECK(lumMidRelease > lumSettledFilter * 1.12);
    CHECK(lumMidRelease < lumIdle * 0.96);

    for (int step = 0; step < 25; ++step) {
        a3Ctrl.tick(0.100f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.100f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img8.data());
    const double lumRecovered = sumUpperBandGray(img8);
    CHECK(near(static_cast<float>(lumRecovered / lumIdle), 1.0f, 0.01f));

    // 8. Verify Zero Hard-Cut Transitions:
    //    (a) Active wave -> y=120 flatline on PLAY/STOP off (no 1-frame snap)
    //    (b) y=120 flatline -> active wave on PLAY/STOP resume (smooth expansion)
    //    (c) Stream stall (paused audio engine) -> y=120 flatline
    //    (d) Momentary modifier release dissolves outgoing HUD view without FREQUENCY flash
    //    (e) Live HOME dismissal dissolves smoothly without 1-frame backdrop snap
    ruido::DroneEngine trEngine;
    ruido::RuidoController trCtrl;
    ruido::ScopeHudRenderer trRenderer;
    trEngine.init(ruido::kBaseSampleRate);
    trCtrl.attachControlBus(&trEngine.controlBus());
    trEngine.snapParametersToLatest();

    for (int f = 0; f < 16; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.016f, trEngine.telemetry());
        trRenderer.prepareFrame(0.016f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img24.data());

    // Measure energy-weighted RMS vertical radius around y=120 & core filament span (g6 >= 24)
    // using pure scope frames (idleHud) so HUD text at top/bottom never clips y=4..236!
    ruido::HudState pureScopeHud{};
    pureScopeHud.mode = ruido::HudMode::None;
    pureScopeHud.opacity = 0.0f;

    auto measureWaveMetrics = [](const std::array<uint16_t, 240u * 240u>& img, int& coreSpanOut,
                                 int& minCoreOut, int& maxCoreOut) -> double {
        int minY = 240;
        int maxY = -1;
        double sumWeightedY2 = 0.0;
        double sumWeight = 0.0;
        // Scan y = 20..220 to exclude the idle top-right stereo peak meter at y = 8..13.
        for (int y = 20; y <= 220; ++y) {
            const double dy = static_cast<double>(y) - 120.0;
            for (size_t x = 0u; x < 240u; ++x) {
                const uint16_t g6 = (img[static_cast<size_t>(y) * 240u + x] >> 5u) & 0x3Fu;
                if (g6 >= 8u) {
                    const double w = static_cast<double>(g6);
                    sumWeightedY2 += w * dy * dy;
                    sumWeight += w;
                }
                if (g6 >= 14u) {
                    if (y < minY) {
                        minY = y;
                    }
                    if (y > maxY) {
                        maxY = y;
                    }
                }
            }
        }
        minCoreOut = minY;
        maxCoreOut = maxY;
        coreSpanOut = (maxY >= minY) ? (maxY - minY + 1) : 0;
        return (sumWeight > 0.0) ? std::sqrt(sumWeightedY2 / sumWeight) : 0.0;
    };

    trRenderer.prepareFrame(0.016f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    trRenderer.renderFullScreenStrips(img24.data());
    int spanActive = 0;
    int minAct = 0;
    int maxAct = 0;
    const double rmsYActive = measureWaveMetrics(img24, spanActive, minAct, maxAct);
    CHECK(spanActive > 125);
    CHECK(rmsYActive > 35.0);

    // (a) Stop audio via PLAY/STOP: after 80 ms (5 frames, past DSP mute + ring window delay),
    //     waveform must smoothly contract toward y=120 rather than snapping in 1 frame!
    trCtrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    trCtrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    for (int f = 0; f < 5; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.016f, trEngine.telemetry());
        trRenderer.prepareFrame(0.016f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    int spanStop80ms = 0;
    int min80 = 0;
    int max80 = 0;
    const double rmsYStop80ms = measureWaveMetrics(img16, spanStop80ms, min80, max80);
    CHECK(rmsYStop80ms > 12.0);
    CHECK(rmsYStop80ms < rmsYActive * 0.90);

    // Step another 112 ms (7 frames -> 192 ms total): monotonically narrower ribbon
    for (int f = 0; f < 7; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.016f, trEngine.telemetry());
        trRenderer.prepareFrame(0.016f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    int spanStop192ms = 0;
    int min192 = 0;
    int max192 = 0;
    const double rmsYStop192ms = measureWaveMetrics(img16, spanStop192ms, min192, max192);
    CHECK(rmsYStop192ms > 3.5);
    CHECK(rmsYStop192ms < rmsYStop80ms * 0.80);

    // Settle to 1.15 s: cleanly converged onto y=120 horizon across all 240 columns
    for (int f = 0; f < 30; ++f) {
        for (int b = 0; b < 24; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.032f, trEngine.telemetry());
        trRenderer.prepareFrame(0.032f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img8.data());
    int spanFlatline = 0;
    int minFlat = 0;
    int maxFlat = 0;
    const double rmsYFlatline = measureWaveMetrics(img8, spanFlatline, minFlat, maxFlat);
    std::printf("  Wave->Flatline RMS-Y: active=%.1fpx -> 80ms=%.1fpx -> 192ms=%.1fpx -> "
                "flat=%.2fpx (coreSpan=%d)\n",
                rmsYActive, rmsYStop80ms, rmsYStop192ms, rmsYFlatline, spanFlatline);
    CHECK(spanFlatline >= 1 && spanFlatline <= 6);
    CHECK(minFlat >= 117 && maxFlat <= 123);
    CHECK(rmsYFlatline < 3.0);
    size_t horizonCols = 0u;
    for (size_t x = 0u; x < 240u; ++x) {
        if (((img8[120u * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
            ++horizonCols;
        }
    }
    CHECK(horizonCols == 240u);

    // (b) Resume audio via PLAY/STOP: after 48 ms, wave expands smoothly out of y=120 (no snap)
    trCtrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    trCtrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    for (int f = 0; f < 3; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.016f, trEngine.telemetry());
        trRenderer.prepareFrame(0.016f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    int spanResume48ms = 0;
    int minRes48 = 0;
    int maxRes48 = 0;
    const double rmsYResume48ms = measureWaveMetrics(img16, spanResume48ms, minRes48, maxRes48);
    CHECK(rmsYResume48ms > rmsYFlatline + 5.0);
    CHECK(rmsYResume48ms < rmsYActive * 0.92);

    // Settle back to full active wave
    for (int f = 0; f < 24; ++f) {
        for (int b = 0; b < 16; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.025f, trEngine.telemetry());
        trRenderer.prepareFrame(0.025f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    int spanResumedFull = 0;
    int minFull = 0;
    int maxFull = 0;
    const double rmsYResumedFull = measureWaveMetrics(img16, spanResumedFull, minFull, maxFull);
    CHECK(rmsYResumedFull > 20.0);

    // (c) Simulate paused audio engine (zero scope pushes for > 65 ms): smooth collapse to flatline
    for (int f = 0; f < 3; ++f) {
        trCtrl.tick(0.040f, trEngine.telemetry());
        trRenderer.prepareFrame(0.040f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    int spanStallMid = 0;
    int minStallMid = 0;
    int maxStallMid = 0;
    const double rmsYStallMid = measureWaveMetrics(img16, spanStallMid, minStallMid, maxStallMid);
    CHECK(rmsYStallMid > 8.0 && rmsYStallMid < rmsYResumedFull * 0.88);

    for (int f = 0; f < 28; ++f) {
        trCtrl.tick(0.035f, trEngine.telemetry());
        trRenderer.prepareFrame(0.035f, trCtrl.state(), pureScopeHud, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img8.data());
    int spanStallEnd = 0;
    int minStallEnd = 0;
    int maxStallEnd = 0;
    const double rmsYStallEnd = measureWaveMetrics(img8, spanStallEnd, minStallEnd, maxStallEnd);
    CHECK(spanStallEnd >= 1 && spanStallEnd <= 6);
    CHECK(rmsYStallEnd < 3.5);

    // Clear latched PlayStop switch HUD via HOME and resume live scope feed before testing
    // unlatched momentary HUD dissolve transitions
    trCtrl.onButtonEvent(ruido::ButtonId::Home, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int f = 0; f < 16; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.020f, trEngine.telemetry());
        trRenderer.prepareFrame(0.020f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
    }

    // (d) Momentary OP6 release holds PANNING labels smoothly during dissolve (no FREQUENCY snap)
    trCtrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    for (int f = 0; f < 12; ++f) {
        trCtrl.tick(0.030f, trEngine.telemetry());
        ruido::HudState h = trCtrl.hud();
        h.scopeFrozen = true;
        trRenderer.prepareFrame(0.030f, trCtrl.state(), h, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());

    trCtrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    CHECK(trCtrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    trCtrl.tick(0.016f, trEngine.telemetry());
    {
        ruido::HudState h = trCtrl.hud();
        h.scopeFrozen = true;
        trRenderer.prepareFrame(0.016f, trCtrl.state(), h, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img8.data());

    double botStripDelta = 0.0;
    size_t botStripPx = 0u;
    for (size_t y = 212u; y < 232u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            const int gA = static_cast<int>((img16[y * 240u + x] >> 5u) & 0x3Fu) * 4;
            const int gB = static_cast<int>((img8[y * 240u + x] >> 5u) & 0x3Fu) * 4;
            botStripDelta += static_cast<double>((gA >= gB) ? (gA - gB) : (gB - gA));
            ++botStripPx;
        }
    }
    botStripDelta /= static_cast<double>(botStripPx);
    CHECK(botStripDelta < 2.5); // Text readout stays PANNING ("75L..") on frame 1 of fade-out!

    // (e) Live HOME dismissal from FX Filter page smoothly restores background brightness
    trCtrl.onButtonEvent(ruido::ButtonId::Fx, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Fx, false);
    for (int f = 0; f < 16; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.040f, trEngine.telemetry());
        trRenderer.prepareFrame(0.040f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());
    const double lumLiveDimmed = sumUpperBandGray(img16);

    trCtrl.onButtonEvent(ruido::ButtonId::Home, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int b = 0; b < 12; ++b) {
        trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        trRenderer.ingestScopeRing(trEngine.scopeBuffer());
    }
    trCtrl.tick(0.024f, trEngine.telemetry());
    trRenderer.prepareFrame(0.024f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
    trRenderer.renderFullScreenStrips(img8.data());
    const double lumLiveHomeMid = sumUpperBandGray(img8);

    for (int f = 0; f < 18; ++f) {
        for (int b = 0; b < 12; ++b) {
            trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            trRenderer.ingestScopeRing(trEngine.scopeBuffer());
        }
        trCtrl.tick(0.040f, trEngine.telemetry());
        trRenderer.prepareFrame(0.040f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img24.data());
    const double lumLiveHomeSettled = sumUpperBandGray(img24);
    CHECK(lumLiveHomeMid > lumLiveDimmed * 1.12);
    CHECK(lumLiveHomeMid < lumLiveHomeSettled * 0.90);

    // 9. Verify 13 pt Bold top-left title ("FILTER OSC 1"): bright "FILTER" + dimmed "OSC 1"
    ruido::EngineControlState silentCtrl = ruido::EngineControlState::makeDefault();
    silentCtrl.global.masterRun = false;
    silentCtrl.global.masterTrim = 0.0f;
    ruido::ScopeHudRenderer hdrRenderer;
    ruido::HudState filterHud{};
    filterHud.mode = ruido::HudMode::VoiceFilter;
    filterHud.knobBank = ruido::KnobBankMode::VoiceFilter;
    filterHud.focusedVoice = 0u;
    filterHud.opacity = 1.0f;
    filterHud.modifierHeld = true;
    hdrRenderer.prepareFrame(0.016f, silentCtrl, filterHud, trEngine.telemetry());
    hdrRenderer.renderFullScreenStrips(img16.data());

    uint16_t maxG6Main = 0u;
    uint16_t maxG6Suffix = 0u;
    int leftTopY = 240;
    int leftBotY = -1;
    int rightTopY = 240;
    for (int y = 6; y <= 22; ++y) {
        const size_t rowBase = static_cast<size_t>(y) * 240u;
        for (int x = 10; x <= 54; ++x) {
            const uint16_t g6 = (img16[rowBase + static_cast<size_t>(x)] >> 5u) & 0x3Fu;
            if (g6 > maxG6Main) {
                maxG6Main = g6;
            }
            if (g6 >= 16u) {
                if (y < leftTopY) {
                    leftTopY = y;
                }
                if (y > leftBotY) {
                    leftBotY = y;
                }
            }
        }
        for (int x = 59; x <= 100; ++x) {
            const uint16_t g6 = (img16[rowBase + static_cast<size_t>(x)] >> 5u) & 0x3Fu;
            if (g6 > maxG6Suffix) {
                maxG6Suffix = g6;
            }
        }
        // Check flat-cap "LP" word at right edge (x = 208..230) to avoid 'O'/'C' curve overshoot
        for (int x = 208; x <= 230; ++x) {
            const uint16_t g6 = (img16[rowBase + static_cast<size_t>(x)] >> 5u) & 0x3Fu;
            if (g6 >= 16u && y < rightTopY) {
                rightTopY = y;
            }
        }
    }
    const int hdrCapSpan = (leftBotY >= leftTopY) ? (leftBotY - leftTopY + 1) : 0;
    CHECK(hdrCapSpan == 10);                         // 13 pt Bold caps == 10 px (matches numbers)
    CHECK(maxG6Main == 63u);                         // "FILTER" emphasized first at 100% white
    CHECK(maxG6Suffix >= 24u && maxG6Suffix <= 36u); // Trailing "OSC 1" dimmed (~46% silver)
    CHECK(leftTopY == 9 && rightTopY == 9);          // Pixel-exact top alignment (y = 9)!

    // Verify EVERY active HudMode renders non-empty top-right header feedback (x = 170..230)
    {
        ruido::ScopeHudRenderer hdrRend;
        ruido::EngineControlState hdrCtrl = ruido::EngineControlState::makeDefault();
        ruido::EngineTelemetry hdrTel{};
        std::array<uint16_t, 240u * 24u> hdrStrip{};
        for (uint8_t m = 1u; m <= static_cast<uint8_t>(ruido::HudMode::GlobalConfig); ++m) {
            ruido::HudState hs{};
            hs.mode = static_cast<ruido::HudMode>(m);
            hs.opacity = 1.0f;
            hdrRend.prepareFrame(0.016f, hdrCtrl, hs, hdrTel);
            hdrRend.renderStrip(0u, 24u, hdrStrip.data());
            uint16_t maxRightG6 = 0u;
            for (int y = 7; y <= 19; ++y) {
                for (int x = 170; x <= 230; ++x) {
                    const uint16_t g6 =
                        (hdrStrip[static_cast<size_t>(y) * 240u + static_cast<size_t>(x)] >> 5u) &
                        0x3Fu;
                    if (g6 > maxRightG6) {
                        maxRightG6 = g6;
                    }
                }
            }
            CHECK(maxRightG6 >= 24u);
        }
    }

    // 10. Verify uncompressed CRT deflection: hot resonant MS-20 peaks cross y=0 and y=239 cleanly
    //     without piling up a clamped horizontal rail inside the bezel (y=4 or y=235)
    ruido::DroneEngine hotEngine;
    ruido::ScopeHudRenderer hotRenderer;
    hotEngine.init(ruido::kBaseSampleRate);
    ruido::EngineControlState hotState = ruido::EngineControlState::makeDefault();
    hotState.global.masterTrim = 1.15f;
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        hotState.voices[v].amplitude = 1.0f;
        hotState.voices[v].muted = (v != 0u);
        hotState.voices[v].lpCutoffHz = 220.0f;
        hotState.voices[v].lpResonance = 0.88f;
    }
    hotEngine.controlBus().publish(hotState);
    hotEngine.snapParametersToLatest();
    for (int f = 0; f < 10; ++f) {
        for (int b = 0; b < 14; ++b) {
            hotEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            hotRenderer.ingestScopeRing(hotEngine.scopeBuffer());
        }
        hotRenderer.prepareFrame(0.016f, hotState, pureScopeHud, hotEngine.telemetry());
    }
    hotRenderer.renderFullScreenStrips(img16.data());
    size_t topEdgeLit = 0u;
    size_t botEdgeLit = 0u;
    for (size_t x = 0u; x < 190u; ++x) { // Exclude idle top-right stereo meter at x >= 200
        if (((img16[0u * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
            ++topEdgeLit;
        }
        if (((img16[239u * 240u + x] >> 5u) & 0x3Fu) >= 12u) {
            ++botEdgeLit;
        }
    }
    CHECK(topEdgeLit > 0u);
    CHECK(botEdgeLit > 0u);
}

void testPerVoiceLfoPageAndHudGraph() {
    std::printf("[Test 7] Per-Voice LFO Page, SELECT Voice Focus, Beads & Strip Equivalence...\n");
    ruido::DroneEngine engine;
    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;

    engine.init(ruido::kBaseSampleRate);
    ctrl.attachControlBus(&engine.controlBus());
    engine.snapParametersToLatest();

    // 1. Open LFO Page via ButtonId::Lfo: latched indefinitely, LED 100%, KNOB1-4 = Lfo bank
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    CHECK(ctrl.hud().lfoPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Lfo);
    CHECK(ctrl.hud().lfoVoiceFocus == 0u);

    for (int i = 0; i < 40; ++i) {
        ctrl.tick(0.250f, engine.telemetry()); // 10.0 s elapsed -> stays latched at 1.0f!
    }
    CHECK(near(ctrl.hud().opacity, 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)], 1.0f));

    // 2. SELECT encoder steps strictly per-voice OSC 1 (0) -> OSC 2 (1) -> OSC 3 (2) -> OSC 4 (3)
    //    (4-voice macro LFO lives on Keybed GLO + SELECT -> GLOBAL LFO), clamped with zero windup!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    CHECK(ctrl.hud().lfoVoiceFocus == 1u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 12);
    CHECK(ctrl.hud().lfoVoiceFocus == 3u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);

    // Wind +200 extra CW ticks at OSC 4 rail -> stays at 3, single -4 CCW detent steps to 2 (OSC 3)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 200);
    CHECK(ctrl.hud().lfoVoiceFocus == 3u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    CHECK(ctrl.hud().lfoVoiceFocus == 2u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);

    // Wind -200 CCW ticks past OSC 1 -> clamps at 0, single +4 CW detent steps to 1 (OSC 2)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -200);
    CHECK(ctrl.hud().lfoVoiceFocus == 0u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    CHECK(ctrl.hud().lfoVoiceFocus == 1u);

    // 3. Per-voice editing on OSC 2 (KNOB 1-4 = TARGET | SHAPE | RATE | DEPTH) leaves OSC 1/3/4
    //    untouched, while Keybed GLO -> GLOBAL LFO edits all 4 voices simultaneously!
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 16);  // +4 steps from LpCutoff(1) -> Amplitude(5)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 60);  // SHAPE
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 40);  // RATE
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 180); // DEPTH
    CHECK(ctrl.state().voices[1].lfoTarget == ruido::LfoTarget::Amplitude);
    CHECK(ctrl.state().voices[1].lfoShape > 0.20f);
    CHECK(ctrl.state().voices[1].lfoRateHz > 0.60f);
    CHECK(ctrl.state().voices[1].lfoDepth > 0.70f);
    CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::LpCutoff);
    CHECK(near(ctrl.state().voices[0].lfoDepth, 0.0f));
    CHECK(near(ctrl.state().voices[2].lfoDepth, 0.0f));

    // Hold Keybed GLO, step SELECT to GLOBAL LFO (2), and turn KNOB1 + KNOB4 -> updates all 4!
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -8); // All(4) - 2 = Lfo(2)
    CHECK(ctrl.hud().mode == ruido::HudMode::GlobalLfo);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -16); // Clamps at first option: Pitch (0)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 150);
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        CHECK(ctrl.state().voices[v].lfoTarget == ruido::LfoTarget::Pitch);
        CHECK(ctrl.state().voices[v].lfoDepth > 0.60f);
    }
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);

    // 4. Momentary OP6 Panning hold overrides LFO Page temporarily and returns cleanly on release!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -16); // Focus OSC 1
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    CHECK(ctrl.hud().lfoPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Lfo);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // 5. Render LFO Page via 16-row, 8-row, and 24-row strips: verify 100% bit-identical monochrome
    //    and verify live Optical Phase Bead moves across x = 10..230 as lfoPhase advances!
    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    for (int f = 0; f < 10; ++f) {
        for (int b = 0; b < 8; ++b) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            renderer.ingestScopeRing(engine.scopeBuffer());
        }
        ctrl.tick(0.032f, engine.telemetry());
        renderer.prepareFrame(0.032f, ctrl.state(), ctrl.hud(), engine.telemetry());
    }

    static std::array<uint16_t, 240u * 240u> lfo16{};
    static std::array<uint16_t, 240u * 240u> lfo8{};
    static std::array<uint16_t, 240u * 240u> lfo24{};
    renderer.renderFullScreenStrips(lfo16.data());
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
        renderer.renderStrip(y, 8u, lfo8.data() + static_cast<size_t>(y) * 240u);
    }
    for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
        renderer.renderStrip(y, 24u, lfo24.data() + static_cast<size_t>(y) * 240u);
    }

    bool stripIdentical = true;
    bool monoGray = true;
    for (size_t i = 0u; i < lfo16.size(); ++i) {
        const uint16_t px = lfo16[i];
        if (px != lfo8[i] || px != lfo24[i]) {
            stripIdentical = false;
        }
        const uint16_t r5 = (px >> 11u) & 0x1Fu;
        const uint16_t g6 = (px >> 5u) & 0x3Fu;
        const uint16_t b5 = px & 0x1Fu;
        if (r5 != b5 || g6 != ((r5 << 1u) | (r5 >> 4u))) {
            monoGray = false;
        }
    }
    CHECK(stripIdentical);
    CHECK(monoGray);

    // Use a silent scope-free renderer and move lfoPhase[0] from 0.15 (left) to 0.80 (right)
    // to verify full-screen LFO geometry (x = 0..239, y = 26..188) and radiant Phase Beacon!
    ruido::ScopeHudRenderer beadRenderer;
    ruido::EngineControlState beadCtrl = ctrl.state();
    beadCtrl.global.masterRun = false;
    beadCtrl.global.masterTrim = 0.0f;
    beadCtrl.voices[0].lfoDepth = 1.0f;
    beadCtrl.voices[0].lfoShape = 0.0f; // Full-amplitude Sine spanning upper & lower viewport
    ruido::EngineTelemetry customTel{};
    customTel.lfoPhase[0].store(0.15f, std::memory_order_relaxed);
    customTel.lfoOutput[0].store(0.8f, std::memory_order_relaxed);
    for (int f = 0; f < 10; ++f) {
        beadRenderer.prepareFrame(0.050f, beadCtrl, ctrl.hud(), customTel);
    }
    beadRenderer.renderFullScreenStrips(lfo16.data());

    customTel.lfoPhase[0].store(0.80f, std::memory_order_relaxed);
    beadRenderer.prepareFrame(0.016f, beadCtrl, ctrl.hud(), customTel);
    beadRenderer.renderFullScreenStrips(lfo8.data());

    // Verify LFO wave spans upper viewport (y = 32..80), lower viewport (y = 135..182),
    // and both left/right screen edges (x = 0..3 and x = 236..239)
    size_t upperHalfLit = 0u;
    size_t lowerHalfLit = 0u;
    size_t leftEdgeLit = 0u;
    size_t rightEdgeLit = 0u;
    for (size_t y = 30u; y <= 184u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            const uint16_t g6 = (lfo16[y * 240u + x] >> 5u) & 0x3Fu;
            if (g6 >= 12u) {
                if (y <= 80u) {
                    ++upperHalfLit;
                }
                if (y >= 135u) {
                    ++lowerHalfLit;
                }
                if (x <= 3u) {
                    ++leftEdgeLit;
                }
                if (x >= 236u) {
                    ++rightEdgeLit;
                }
            }
        }
    }
    CHECK(upperHalfLit > 150u);
    CHECK(lowerHalfLit > 150u);
    CHECK(leftEdgeLit > 4u);
    CHECK(rightEdgeLit > 4u);

    double leftBeadDiff = 0.0;
    double rightBeadDiff = 0.0;
    uint16_t leftBeadMaxG6 = 0u;
    uint16_t rightBeadMaxG6 = 0u;
    for (size_t y = 24u; y <= 190u; ++y) {
        for (size_t x = 24u; x <= 48u; ++x) {
            const uint16_t g6A = (lfo16[y * 240u + x] >> 5u) & 0x3Fu;
            const uint16_t g6B = (lfo8[y * 240u + x] >> 5u) & 0x3Fu;
            if (g6A > leftBeadMaxG6) {
                leftBeadMaxG6 = g6A;
            }
            leftBeadDiff +=
                static_cast<double>(static_cast<int>(g6A) - static_cast<int>(g6B)) * 4.0;
        }
        for (size_t x = 180u; x <= 204u; ++x) {
            const uint16_t g6A = (lfo16[y * 240u + x] >> 5u) & 0x3Fu;
            const uint16_t g6B = (lfo8[y * 240u + x] >> 5u) & 0x3Fu;
            if (g6B > rightBeadMaxG6) {
                rightBeadMaxG6 = g6B;
            }
            rightBeadDiff +=
                static_cast<double>(static_cast<int>(g6B) - static_cast<int>(g6A)) * 4.0;
        }
    }
    CHECK(leftBeadMaxG6 == 63u); // Crisp #FFFFFF pure-white pinpoint core
    CHECK(rightBeadMaxG6 == 63u);
    CHECK(leftBeadDiff > 250.0 && leftBeadDiff < 2600.0); // Compact, non-bloated optical halo
    CHECK(rightBeadDiff > 250.0 && rightBeadDiff < 2600.0);

    // Verify shaded LFO interior (around positive Sine crest x = 60..63 in lfo16, y = 40..96)
    // decays smoothly with zero decorative inner contour lines!
    bool monotonicShade = true;
    bool firstTile = true;
    uint32_t prevTileSum = 0u;
    for (size_t ty = 40u; ty <= 96u; ty += 4u) {
        uint32_t tileSum = 0u;
        for (size_t dy = 0u; dy < 4u; ++dy) {
            for (size_t x = 60u; x < 64u; ++x) {
                tileSum += (lfo16[(ty + dy) * 240u + x] >> 5u) & 0x3Fu;
            }
        }
        if (!firstTile && tileSum > prevTileSum + 2u) {
            monotonicShade = false;
        }
        prevTileSum = tileSum;
        firstTile = false;
    }
    CHECK(monotonicShade);

    // 6. Close LFO Page via LFO tap (or HOME): active LFOs keep running and LFO LED pulses live!
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    CHECK(!ctrl.hud().lfoPageOpen);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);

    customTel.lfoOutput[0].store(-1.0f, std::memory_order_relaxed);
    for (int v = 1; v < 4; ++v) {
        customTel.lfoOutput[v].store(-1.0f, std::memory_order_relaxed);
    }
    ctrl.tick(0.016f, customTel);
    const float ledTrough = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

    customTel.lfoOutput[0].store(1.0f, std::memory_order_relaxed);
    ctrl.tick(0.016f, customTel);
    const float ledCrest = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

    CHECK(ledTrough >= 0.11f && ledTrough < 0.25f);
    CHECK(ledCrest > 0.45f && ledCrest <= 0.61f);
    CHECK(ledCrest > ledTrough + 0.25f);
}

} // namespace

int main() {
    std::printf("=== Ruido Milestone 2 Monochrome CRT & Sans-Serif HUD Verification ===\n");
    testMemoryFootprint();
    testTwoHandErgonomicsAndLinearGears();
    testAllControlsDisplayOnScreen();
    testButtonsOctavesKeybedAndPressure();
    testHudEnvelopeAndVaribrightLeds();
    testMonochromeCrtScanlineAndUncompressedWave();
    testPerVoiceLfoPageAndHudGraph();
    ruido_test::runUnified150msFadesAndRateSplitHeaderTests(check);
    ruido_test::runGlobalFiveModesAndConfigMenuTests(check);
    ruido_test::runEditChannelDrivePageAndAmpGraphTests(check);

    if (gFailures == 0) {
        std::printf("\nALL UI & CONTROLLER VERIFICATION GATES PASSED (0 failures).\n");
        return 0;
    }
    std::fprintf(stderr, "\nFAILED with %d error(s).\n", gFailures);
    return 1;
}
