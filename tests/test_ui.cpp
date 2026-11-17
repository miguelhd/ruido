#include "TestFm1PanelIo.h"
#include "TestGlobalAndInteraction.h"
#include "core/EncoderMath.h"
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
    // Full 240x240 RGB565 framebuffer would be 115,200 B on top of history/persistence state;
    // 4096-sample super-period history + continuous phosphor persistence + 60x60 bloom + 20.2 KB
    // per-column segment cache + arena trace/strip scratch stays < 124 KB without allocating a
    // full-screen framebuffer (and keeps all MCU stack frames < 2 KB).
    CHECK(sizeof(ruido::ScopeHudRenderer) < 126976u);
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

    // Subsonic boundary (20 Hz) sits at 16% (< 1/4) of the horizontal oscillator frequency bar
    CHECK(near(ruido::normalizeOscFreqIndicator01(ruido::kAudioFreqMinHz), 0.0f, 1.0e-3f));
    CHECK(near(ruido::normalizeOscFreqIndicator01(20.0f), 0.16f, 1.0e-2f));
    CHECK(ruido::normalizeOscFreqIndicator01(20.0f) < 0.25f);
    CHECK(near(ruido::normalizeOscFreqIndicator01(ruido::kAudioFreqMaxHz), 1.0f, 1.0e-3f));

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
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
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
    const ruido::VoiceParams v0BeforeFx = ctrl.state().voices[0];
    const ruido::VoiceParams v2BeforeFx = ctrl.state().voices[2];
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 20);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 30);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -20);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 30);
    CHECK(ctrl.state().voices[2].hpCutoffHz > v2BeforeFx.hpCutoffHz);
    CHECK(ctrl.state().voices[2].hpResonance > v2BeforeFx.hpResonance);
    CHECK(ctrl.state().voices[2].lpCutoffHz < v2BeforeFx.lpCutoffHz);
    CHECK(ctrl.state().voices[2].lpResonance > v2BeforeFx.lpResonance);
    CHECK(near(ctrl.state().voices[0].hpCutoffHz, v0BeforeFx.hpCutoffHz));

    // Repeated FX taps cycle OSC focus with continuous wrap (2 -> 3 -> 0 -> 1); HOME closes &
    // reopening FX remembers last focused voice
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen && ctrl.hud().fxVoiceFocus == 3u && ctrl.hud().focusedVoice == 3u);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen && ctrl.hud().fxVoiceFocus == 0u && ctrl.hud().focusedVoice == 0u);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen && ctrl.hud().fxVoiceFocus == 1u && ctrl.hud().focusedVoice == 1u);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen && ctrl.hud().fxVoiceFocus == 1u);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().fxPageOpen);

    // 3. EDIT 2-Step Console Mixer Cycle (LEVEL <-> PAN), remembered PAN sub-page on HOME close,
    //    HOME 4-Step Oscillator Core Cycle (Clean -> FREQUENCY -> DUTY -> DRIFT -> Clean),
    //    multi-press SEL model cycle, and Mode 1 LIVE MIXER punch-ins:
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::Amplitude);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Amplitude);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -30);
    CHECK(ctrl.state().voices[1].amplitude < 0.65f);

    // 2nd EDIT tap -> Spatial PAN
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::Panning);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Panning);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -30);
    CHECK(ctrl.state().voices[3].pan < 0.65f);

    // Closing from PAN via HOME remembers PAN on next EDIT press, then toggles PAN <-> LEVEL
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(!ctrl.hud().editPageOpen && ctrl.hud().panPageOpen);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen &&
          ctrl.hud().mode == ruido::HudMode::Amplitude);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().editPageOpen && !ctrl.hud().panPageOpen);

    // Persistent SEL guidance screen: 1st tap shows active model; subsequent taps wrap 1/3 -> 2/3
    // -> 3/3 -> 1/3 alongside SELECT knob; HOME dismisses!
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(ctrl.hud().selPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(ctrl.hud().selPageOpen && ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(ctrl.hud().selPageOpen && ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(ctrl.hud().selPageOpen && ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4);
    CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::CircuitLab);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().selPageOpen);

    // OP1..GLO (top 8 analog pressure keys) trigger momentary KeyboardPlay punch-ins mapped 1-to-1
    // to native Ruido views
    const float savedAmp1 = ctrl.state().voices[1].amplitude;
    for (uint8_t op = 0u; op <= static_cast<uint8_t>(ruido::BlackKeyId::Glo); ++op) {
        const auto key = static_cast<ruido::BlackKeyId>(op);
        ctrl.onBlackKeyEvent(key, true);
        CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
        CHECK(ctrl.hud().lastBlackKey == key);
        const auto expectedBank = (key == ruido::BlackKeyId::Op1) ? ruido::KnobBankMode::Amplitude
                                  : (key == ruido::BlackKeyId::Op4 || key == ruido::BlackKeyId::Op5)
                                      ? ruido::KnobBankMode::GlobalFilter
                                  : (key == ruido::BlackKeyId::Glo)
                                      ? ruido::KnobBankMode::Panning
                                      : ruido::KnobBankMode::Frequency;
        CHECK(ctrl.hud().knobBank == expectedBank);
        ctrl.onBlackKeyEvent(key, false);
    }
    CHECK(near(ctrl.state().voices[1].amplitude, savedAmp1));

    // EDIT Level 0% auto-mute & CW fade-in unmute:
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -240);
    CHECK(ctrl.state().voices[0].muted);
    CHECK(near(ctrl.state().voices[0].amplitude, 0.0f));
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4);
    CHECK(!ctrl.state().voices[0].muted);
    CHECK(ctrl.state().voices[0].amplitude > 0.005f && ctrl.state().voices[0].amplitude < 0.05f);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);

    // Verify lock-free bus received latest published state
    const ruido::EngineControlState& snapped = bus.consumeLatest();
    CHECK(near(snapped.voices[2].hpCutoffHz, ctrl.state().voices[2].hpCutoffHz));
}

void testAllControlsDisplayOnScreen() {
    std::printf("[Test 3] All 27 Keys, 14 Panel Switches & 8 Knobs Display HUD Feedback...\n");
    ruido::RuidoController ctrl;

    const auto clearHudIfActive = [&]() {
        if (ctrl.hud().freqPageOpen) {
            for (int step = 0; step < 3 && ctrl.hud().freqPageOpen; ++step) {
                ctrl.onButtonEvent(ruido::ButtonId::Home, true);
                ctrl.onButtonEvent(ruido::ButtonId::Home, false);
            }
        } else if (ctrl.hud().mode != ruido::HudMode::None) {
            ctrl.onButtonEvent(ruido::ButtonId::Home, true);
            ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        }
        CHECK(ctrl.hud().mode == ruido::HudMode::None);
    };

    // 1. Every single Black Key (OP1..6, PIT, GLO, MONO=HOLD, 11th=SHIFT in LiveMixer; POLY=TAP
    //    is strictly 0% inert in LiveMixer & wakes MasterTempo in RhythmLab!)
    for (uint8_t b = 0u; b < static_cast<uint8_t>(ruido::BlackKeyId::Count); ++b) {
        clearHudIfActive();

        const auto key = static_cast<ruido::BlackKeyId>(b);
        ctrl.onBlackKeyEvent(key, true);
        if (key == ruido::BlackKeyId::Poly) {
            CHECK(ctrl.hud().mode == ruido::HudMode::None);
        } else {
            CHECK(ctrl.hud().mode != ruido::HudMode::None);
        }
        ctrl.onBlackKeyEvent(key, false);
    }
    if (ctrl.hud().keyHoldLatched) {
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, true);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::Mono, false);
    }

    // 2. Every single White Key (all 16: Gates 1..4, Pressure Pads 5..8, Snapshots 9..16)
    for (uint8_t w = 0u; w < ruido::kNumWhiteKeys; ++w) {
        clearHudIfActive();

        ctrl.onWhiteKeyGate(w, true);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
        ctrl.onWhiteKeyGate(w, false);
    }

    // 3. All 14 Panel Switches: 10 functional switches wake HUD, 4 phantom-pin switches
    //    (ENV/ARP/SEQ/REC) leave HUD silent while pulsing phantomPinSeq
    for (uint8_t btn = 0u; btn < static_cast<uint8_t>(ruido::ButtonId::Count); ++btn) {
        const auto button = static_cast<ruido::ButtonId>(btn);
        clearHudIfActive();
        const uint8_t prevSeq = ctrl.hud().phantomPinSeq;
        ctrl.onButtonEvent(button, true);
        const bool isPhantom = (button == ruido::ButtonId::Env || button == ruido::ButtonId::Arp ||
                                button == ruido::ButtonId::Seq || button == ruido::ButtonId::Rec);
        if (isPhantom) {
            CHECK(ctrl.hud().mode == ruido::HudMode::None);
            CHECK(ctrl.hud().phantomPinSeq == static_cast<uint8_t>(prevSeq + 1u));
        } else {
            CHECK(ctrl.hud().mode != ruido::HudMode::None);
        }
        ctrl.onButtonEvent(button, false);
    }

    // 4. All 7 Encoders + MASTER Potentiometer
    for (uint8_t e = 0u; e < static_cast<uint8_t>(ruido::EncoderId::Count); ++e) {
        clearHudIfActive();
        ctrl.onEncoderDelta(static_cast<ruido::EncoderId>(e), 2);
        CHECK(ctrl.hud().mode != ruido::HudMode::None);
    }
    clearHudIfActive();
    ctrl.onMasterPot(0.42f);
    CHECK(ctrl.hud().mode == ruido::HudMode::MasterLevel);
}

void testButtonsOctavesKeybedAndPressure() {
    std::printf("[Test 4] Global Knobs, Octaves, Single-Taps & Keybed Pressure...\n");
    ruido::RuidoController ctrl;

    // Global encoders & idle SELECT CRT Timebase selector
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 40);
    CHECK(ctrl.state().global.circuitCoupling > 0.30f);

    CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Auto);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    CHECK(ctrl.hud().mode == ruido::HudMode::ScopeTimebase);
    CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Zoom2x);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::None);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -40);
    CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Zoom4x);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);
    CHECK(ctrl.hud().scopeTimebase == ruido::ScopeTimebase::Auto);

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
    // unconnected switches (ENV, ARP, SEQ, REC) leave HUD mode untouched and spark a dim LED glint:
    const float v1BeforeLfo = ctrl.state().voices[1].frequencyHz;
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(ctrl.hud().lfoPageOpen && ctrl.hud().lfoVoiceFocus == 0u);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    CHECK(near(ctrl.state().voices[1].frequencyHz, v1BeforeLfo));
    for (uint8_t step = 1u; step <= 8u; ++step) {
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
        ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
        CHECK(ctrl.hud().lfoPageOpen && ctrl.hud().lfoVoiceFocus == (step & 7u));
    }
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().lfoPageOpen);

    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().fxPageOpen);

    const uint8_t prevPhantomSeq = ctrl.hud().phantomPinSeq;
    ctrl.onButtonEvent(ruido::ButtonId::Env, true);
    ctrl.onButtonEvent(ruido::ButtonId::Env, false);
    CHECK(ctrl.hud().phantomPinSeq == static_cast<uint8_t>(prevPhantomSeq + 1u));
    CHECK(ctrl.hud().phantomPinIndex == 0u);
    ctrl.tick(0.016f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.15f));
    ctrl.tick(0.060f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.075f, 0.005f));
    ctrl.tick(0.065f, tel);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.0f));

    // OP3 Ratio Magnet, stateless SEL Model Toast, momentary OP5/OP6 spatial focus, PLAY/STOP
    const float v0BeforePit = ctrl.state().voices[0].frequencyHz;
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    CHECK(ctrl.hud().activePunch == ruido::KeyPunchKind::RatioMagnet);
    CHECK(!ctrl.state().global.pitchPressureMode);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op3, false);

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    CHECK(!ctrl.state().global.pitchPressureMode);
    CHECK(!ctrl.hud().panPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardModel);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Op6, 1.0f);
    CHECK(near(ctrl.state().keyMod.spatialFocus, -1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op6, false);
    CHECK(near(ctrl.state().keyMod.spatialFocus, 0.0f));

    ctrl.onBlackKeyPressure(ruido::BlackKeyId::Glo, 1.0f);
    CHECK(near(ctrl.state().keyMod.spatialFocus, 1.0f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Glo, false);
    CHECK(near(ctrl.state().keyMod.spatialFocus, 0.0f));

    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    CHECK(!ctrl.state().global.masterRun);

    // White Keys 12..15 momentary Mute & White Keys 0..3 continuous Formant (0 pitch mod)
    CHECK(!ctrl.state().voices[0].muted);
    ctrl.onWhiteKeyPressure(12u, 0.90f);
    CHECK(ctrl.state().keyMod.voiceGainMul[0] < 0.02f);
    ctrl.onWhiteKeyGate(12u, false);
    CHECK(near(ctrl.state().keyMod.voiceGainMul[0], 1.0f));

    ctrl.onWhiteKeyGate(1u, true);
    CHECK(ctrl.state().voicePressure[1] > 0.0f);
    ctrl.onWhiteKeyPressure(1u, 0.82f);
    CHECK(near(ctrl.state().voicePressure[1], 0.82f));
    CHECK(near(ctrl.state().voices[0].frequencyHz, v0BeforePit));
    ctrl.onWhiteKeyGate(1u, false);
    CHECK(near(ctrl.state().voicePressure[1], 0.0f));
}

void testHudEnvelopeAndVaribrightLeds() {
    std::printf("[Test 5] Soft S-Curve HUD Envelope & 42-Channel Hardware LED Engine...\n");
    ruido::RuidoController ctrl;
    ruido::EngineTelemetry tel{};

    // Hold OP4 (LOW PASS): unified 150 ms S-curve fade-in (smoothstep01), pinned while held
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, true);
    ctrl.tick(0.030f, tel); // 20% of 150 ms -> smoothstep01(0.2) = 0.104 (eased start!)
    CHECK(ctrl.hud().opacity > 0.08f && ctrl.hud().opacity < 0.14f);
    ctrl.tick(0.045f, tel); // 50% of 150 ms (75 ms total) -> smoothstep01(0.5) = 0.500
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    for (int i = 0; i < 10; ++i) {
        ctrl.tick(0.050f, tel);
    }
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // Release OP4: momentary hold dissolves smoothly in 150 ms without 1.5 s freeze
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
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

    // HOME button fades active HUD out smoothly in 150 ms while HOME LED stays 100% on Home screen
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 3);
    ctrl.tick(0.200f, tel);
    CHECK(ctrl.hud().opacity > 0.99f);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 1.0f));
    ctrl.tick(0.100f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 1.0f));

    // Pressing HOME when no HUD is active latches FREQUENCY HUD open; pressing again hides it
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(ctrl.hud().freqPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    CHECK(ctrl.hud().focusedKnob == ruido::kNoFocusedKnob);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> stays latched open!
    }
    CHECK(ctrl.hud().freqPageOpen);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 1.0f));

    // Turning KNOB 1-4 while FREQUENCY HUD is open keeps it latched (does not auto-hide in 1.5 s)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 2);
    CHECK(ctrl.hud().freqPageOpen);
    CHECK(ctrl.hud().focusedKnob == 1u);
    for (int i = 0; i < 20; ++i) {
        ctrl.tick(0.200f, tel); // 4.0 s elapsed -> still latched open!
    }
    CHECK(ctrl.hud().freqPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -2); // restore default Osc 2 pitch

    // 2nd & 3rd HOME taps step FREQUENCY -> DUTY -> DRIFT; 4th HOME tap hides smoothly over 150 ms
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Width);
    CHECK(ctrl.hud().mode == ruido::HudMode::PulseWidth &&
          ctrl.hud().knobBank == ruido::KnobBankMode::PulseWidth);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(ctrl.hud().freqPageOpen && ctrl.hud().homeTarget == ruido::HomeTarget::Drift);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceDrift &&
          ctrl.hud().knobBank == ruido::KnobBankMode::VoiceDrift);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().freqPageOpen);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.100f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 1.0f));

    // Right 2x6 pages & switches (FX, LFO, EDIT, GLO, SAVE, unassigned SEL/ENV) stay latched until
    // closed or HOME, and opening FX dims HOME LED cleanly to 15%:
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    for (int i = 0; i < 60; ++i) {
        ctrl.tick(0.200f, tel); // 12.0 s elapsed -> FX Filter page stays latched at 1.0f!
    }
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 1.0f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 0.15f));

    // Holding & releasing a momentary keybed punch-in (OP4 / Pad 1) returns to latched FX page:
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, true);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    ctrl.onWhiteKeyGate(0u, true);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    ctrl.onWhiteKeyGate(0u, false);
    ctrl.tick(0.050f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // Tap ENV/ARP/SEQ/REC (phantom pins) -> NEVER closes open FX page or alters HUD mode:
    ctrl.onButtonEvent(ruido::ButtonId::Env, true);
    ctrl.onButtonEvent(ruido::ButtonId::Env, false);
    CHECK(ctrl.hud().fxPageOpen);
    CHECK(!ctrl.state().global.pitchPressureMode);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFilter);

    // Close FX via HOME, latch PLAY/STOP AudioOutput switch, and verify knob tweak returns to it:
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    CHECK(ctrl.hud().mode == ruido::HudMode::AudioOutput);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 2);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceFrequency);
    for (int i = 0; i < 16; ++i) {
        ctrl.tick(0.100f, tel);
    }
    CHECK(ctrl.hud().mode == ruido::HudMode::AudioOutput);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);

    // LFO and SAVE stay latched across 6.0 s even if ARP/SEQ/REC are brushed:
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    ctrl.onButtonEvent(ruido::ButtonId::Arp, true);
    ctrl.onButtonEvent(ruido::ButtonId::Arp, false);
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> LFO page stays latched!
    }
    CHECK(ctrl.hud().lfoPageOpen);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    ctrl.onButtonEvent(ruido::ButtonId::Rec, true);
    ctrl.onButtonEvent(ruido::ButtonId::Rec, false);
    CHECK(ctrl.hud().saveArmed);
    for (int i = 0; i < 30; ++i) {
        ctrl.tick(0.200f, tel); // 6.0 s elapsed -> SAVE stays armed & latched!
    }
    CHECK(ctrl.hud().saveArmed);
    CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // 2nd tap on SAVE advances to 2/2 DELETE SCENE; 3rd tap (or HOME) exits cleanly in 150 ms
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    CHECK(ctrl.hud().saveArmed && ctrl.hud().saveDeleteMode);
    CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);
    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    CHECK(!ctrl.hud().saveArmed && !ctrl.hud().saveDeleteMode);
    CHECK(ctrl.hud().mode == ruido::HudMode::SaveArmed);
    ctrl.tick(0.075f, tel);
    CHECK(near(ctrl.hud().opacity, 0.50f, 0.03f));
    ctrl.tick(0.085f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);

    ctrl.onButtonEvent(ruido::ButtonId::Save, true);
    ctrl.onButtonEvent(ruido::ButtonId::Save, false);
    CHECK(ctrl.hud().saveArmed);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.160f, tel);
    CHECK(near(ctrl.hud().opacity, 0.0f));
    CHECK(ctrl.hud().mode == ruido::HudMode::None);

    // Verify unassigned buttons are strictly 0% dark, Mode 1 LIVE MIXER 20+7 keybed zones step in a
    // calibrated 65% -> 40% -> 15% left-to-right gradient, TAP=0% in LiveMixer, SHIFT=100% beacon,
    // HOME=100% on Clean Scope, and FX/SEL/LFO/EDIT/GLO rest at 15%:
    const auto& leds = ctrl.leds();
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op1)], 0.65f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op2)], 0.65f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op3)], 0.65f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op4)], 0.40f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op5)], 0.40f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Op6)], 0.15f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Pit)], 0.15f));
    CHECK(near(leds.blackKeyLeds[static_cast<size_t>(ruido::BlackKeyId::Glo)], 0.15f));
    CHECK(near(leds.blackKeyLeds[ruido::kHoldBlackKeyIdx], 0.15f));
    CHECK(near(leds.blackKeyLeds[ruido::kTapBlackKeyIdx], 0.0f));
    CHECK(near(leds.blackKeyLeds[ruido::kShiftBlackKeyIdx], 1.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Home)], 1.0f));
    CHECK(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.15f &&
          leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 0.375f);
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Sel)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Env)], 0.0f));
    CHECK(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)] > 0.15f &&
          leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)] <= 0.375f);
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Edit)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::GloRight)], 0.15f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Arp)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Seq)], 0.0f));
    CHECK(near(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Rec)], 0.0f));

    // Bottom 4x4 groups step 65% (FILTER) -> 40% (OCTAVE) -> 15% (DUTY) -> 80% (MUTE 1-4 audible
    // >= 20 Hz, leaving SHIFT at 100% brightest), while SAVE gently breathes (12%..28%) when
    // EDITED:
    for (size_t i = 0u; i < 4u; ++i) {
        CHECK(near(leds.whiteKeyLeds[i], 0.65f));
        CHECK(near(leds.whiteKeyLeds[4u + i], 0.40f));
        CHECK(near(leds.whiteKeyLeds[8u + i], 0.15f));
        CHECK(near(leds.whiteKeyLeds[12u + i], 0.80f));
    }
    CHECK(leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)] >= 0.12f &&
          leds.buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)] <= 0.28f);
    ctrl.onWhiteKeyPressure(0u, 0.75f);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.leds().whiteKeyLeds[0u] > 0.80f);
    CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.45f &&
          ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 1.0f);
    ctrl.onWhiteKeyGate(0u, false);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] > 0.15f &&
          ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)] <= 0.375f);

    // Universal 11th Key = SHIFT: opens ShiftMap, keeps SHIFT 100%, slow-blinks active model on OP1
    // (15%..100%) with OP2/OP3 at 15%, ghosts unassigned keys dark (0%), and lights SCENE & SOLO:
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
    ctrl.tick(0.016f, tel);
    CHECK(ctrl.hud().mode == ruido::HudMode::ShiftMap && ctrl.hud().shiftHeld);
    CHECK(near(ctrl.leds().blackKeyLeds[ruido::kShiftBlackKeyIdx], 1.0f));
    CHECK(near(ctrl.leds().whiteKeyLeds[0], 0.0f));
    CHECK(ctrl.leds().blackKeyLeds[0] >= 0.15f && ctrl.leds().blackKeyLeds[0] <= 1.0f);
    CHECK(near(ctrl.leds().blackKeyLeds[1], 0.15f));
    CHECK(near(ctrl.leds().blackKeyLeds[2], 0.15f));
    CHECK(near(ctrl.leds().blackKeyLeds[3], 0.0f));
    CHECK(near(ctrl.leds().whiteKeyLeds[8], 0.80f));
    CHECK(near(ctrl.leds().whiteKeyLeds[9], 0.40f));
    CHECK(near(ctrl.leds().whiteKeyLeds[12], 0.80f));
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
    ctrl.tick(0.016f, tel);

    // White Keys 13-16 (MUTE 1-4) reflect steady oscillator sound status (80% >= 20 Hz, 50% < 20
    // Hz, 0% muted/silent) and NEVER dance with sub-audio LFO breathing; HOLD rests at exact 15%:
    CHECK(near(ctrl.leds().whiteKeyLeds[12], 0.80f));
    CHECK(near(ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx], 0.15f));

    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, true);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -110);
    ctrl.onButtonEvent(ruido::ButtonId::OctPlus, false);
    CHECK(ctrl.state().voices[0].frequencyHz < 5.0f);
    bool consoleSteady = true;
    for (int i = 0; i < 200; ++i) {
        ctrl.tick(0.016f, tel);
        if (!near(ctrl.leds().whiteKeyLeds[12], 0.50f) ||
            !near(ctrl.leds().whiteKeyLeds[13], 0.80f) ||
            !near(ctrl.leds().blackKeyLeds[ruido::kHoldBlackKeyIdx], 0.15f)) {
            consoleSteady = false;
        }
    }
    CHECK(consoleSteady);

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
    CHECK(pitchBlackPixels > 16000u); // Pitch-black #000000 sky preserved above full-screen filter
    CHECK(litPixels > 2400u);

    // 2. Verify zero vertical waveform compression when non-filter HUD opens vs closes:
    // Close FX page, let the filter backdrop dim recovery settle cleanly, then compare Frequency
    // HUD open (opacity = 1.0) vs HUD dismissed (opacity = 0.0) on the exact frozen trace!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    ctrl.tick(0.250f, engine.telemetry());
    ruido::HudState freqOpenHud = ctrl.hud();
    freqOpenHud.mode = ruido::HudMode::VoiceFrequency;
    freqOpenHud.knobBank = ruido::KnobBankMode::Frequency;
    freqOpenHud.freqPageOpen = true;
    freqOpenHud.opacity = 1.0f;
    freqOpenHud.scopeFrozen = true;
    for (int settle = 0; settle < 16; ++settle) {
        renderer.prepareFrame(0.150f, ctrl.state(), freqOpenHud, engine.telemetry());
    }
    renderer.renderFullScreenStrips(img16.data()); // Frequency HUD opacity == 1.0

    ruido::HudState hiddenHud = ctrl.hud();
    hiddenHud.opacity = 0.0f;
    hiddenHud.mode = ruido::HudMode::None;
    hiddenHud.scopeFrozen = true;
    renderer.prepareFrame(0.016f, ctrl.state(), hiddenHud, engine.telemetry());
    renderer.renderFullScreenStrips(img8.data()); // HUD opacity == 0.0
    size_t idleBlackPixels = 0u;
    for (const uint16_t px : img8) {
        if (px == 0x0000u) {
            ++idleBlackPixels;
        }
    }
    CHECK(idleBlackPixels > 20000u); // Pitch-black #000000 idle CRT background preserved

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
    if (ctrl.hud().mode != ruido::HudMode::None) {
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    }
    renderer.prepareFrame(0.016f, ctrl.state(), ctrl.hud(), engine.telemetry());
    renderer.renderFullScreenStrips(img24.data()); // Live 4-voice caustic mesh

    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, -240);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, -240);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, -240);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int f = 0; f < 12; ++f) {
        ctrl.tick(0.025f, engine.telemetry());
    }

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
    CHECK(beatingDelta < 8.5);

    // 7. Verify Full-Screen (x = 0..239, y = 26..186, 160 px) Dual-Resonance MS-20 Filter Graph,
    //    0% Voice Offset on GLO GlobalFilter, and smooth 72% BG Wave Dimming on FX open/close:
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
        // Measure y = 38..80 in outer columns (x < 76 or x >= 196), strictly outside top header
        // scrim (0..34), 0 dB horizon (y = 86), and 150..1500 Hz resonant horns (x = 92..182)
        for (size_t y = 38u; y < 80u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                if (x >= 76u && x < 196u) {
                    continue;
                }
                sum += static_cast<double>((img[y * 240u + x] >> 5u) & 0x3Fu) * 4.0;
            }
        }
        return sum;
    };
    const double lumIdle = sumUpperBandGray(img24);
    CHECK(lumIdle > 2000.0);

    // Open FX Filter page: step 64 ms (4 x 16 ms) -> mid-attack fade-out; then settle -> 100%
    // hidden
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
    CHECK(lumMidAttack < lumIdle * 0.96 && lumMidAttack > 50.0);
    CHECK(lumSettledFilter == 0.0); // 100% hidden BG oscilloscope while FX menu is open

    // Verify full-screen 160 px filter zone: resonant horns reach upper half (y = 26..80)
    // and passband silhouette fills all the way down to y = 186!
    size_t hornPixels = 0u;
    size_t fullFillPixels = 0u;
    for (size_t x = 90u; x < 185u; ++x) {
        for (size_t y = 26u; y < 80u; ++y) {
            if (img16[y * 240u + x] > 0x3186u) {
                ++hornPixels;
            }
        }
        for (size_t y = 92u; y <= 186u; ++y) {
            if (((img16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                ++fullFillPixels;
            }
        }
    }
    CHECK(hornPixels > 160u);
    CHECK(fullFillPixels > 2400u);

    // Verify 0% Voice Offset: GlobalFilter matches FX VoiceFilter pixel-for-pixel across
    // the entire full-screen filter viewport (x = 0..239, y = 26..186)
    ruido::HudState gloFilterHud = a3Ctrl.hud();
    gloFilterHud.fxPageOpen = false;
    gloFilterHud.mode = ruido::HudMode::KeyboardPlay;
    gloFilterHud.activePunch = ruido::KeyPunchKind::DubLpfDrop;
    gloFilterHud.knobBank = ruido::KnobBankMode::GlobalFilter;
    gloFilterHud.scopeFrozen = true;
    for (int step = 0; step < 12; ++step) {
        calmRenderer.prepareFrame(0.080f, a3State, gloFilterHud, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img8.data());
    bool zeroVoiceOffsetIdentical = true;
    for (size_t y = 26u; y <= 186u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            if (img16[y * 240u + x] != img8[y * 240u + x]) {
                zeroVoiceOffsetIdentical = false;
            }
        }
    }
    CHECK(zeroVoiceOffsetIdentical);

    // Close FX: verify smooth 150 ms recovery transition (no 1-frame snap) back to 100%
    a3Ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    a3Ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    for (int step = 0; step < 4; ++step) { // 4 x 16 ms = 64 ms (~43% of 150 ms fade)
        a3Ctrl.tick(0.016f, calmEngine.telemetry());
        ruido::HudState h = a3Ctrl.hud();
        h.scopeFrozen = true;
        calmRenderer.prepareFrame(0.016f, a3State, h, calmEngine.telemetry());
    }
    calmRenderer.renderFullScreenStrips(img8.data());
    const double lumMidRelease = sumUpperBandGray(img8);
    CHECK(lumMidRelease > lumIdle * 0.10);
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

    // Verify edge-to-edge horizontal span (x = 0..3 and x = 236..239), top-of-viewport Korg35
    // self-oscillation crest (y = 26..36), and 8/16/24-row strip bit-exactness on FX page:
    {
        ruido::ScopeHudRenderer fltFullRenderer;
        ruido::EngineControlState fltFullSt = ruido::EngineControlState::makeDefault();
        fltFullSt.global.masterRun = false;
        fltFullSt.global.masterTrim = 0.0f;
        fltFullSt.voices[0].hpCutoffHz = ruido::kFilterCutoffMinHz;
        fltFullSt.voices[0].hpResonance = 0.0f;
        fltFullSt.voices[0].lpCutoffHz = ruido::kFilterCutoffMaxHz;
        fltFullSt.voices[0].lpResonance = 0.0f;
        a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
        a3Ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
        for (int step = 0; step < 12; ++step) {
            a3Ctrl.tick(0.032f, calmEngine.telemetry());
            fltFullRenderer.prepareFrame(0.032f, fltFullSt, a3Ctrl.hud(), calmEngine.telemetry());
        }
        fltFullRenderer.renderFullScreenStrips(img16.data());
        size_t leftEdgeFltLit = 0u;
        size_t rightEdgeFltLit = 0u;
        for (size_t y = 84u; y <= 186u; ++y) {
            for (size_t x = 0u; x <= 3u; ++x) {
                if (((img16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                    ++leftEdgeFltLit;
                }
            }
            for (size_t x = 236u; x <= 239u; ++x) {
                if (((img16[y * 240u + x] >> 5u) & 0x3Fu) >= 4u) {
                    ++rightEdgeFltLit;
                }
            }
        }
        CHECK(leftEdgeFltLit > 80u);
        CHECK(rightEdgeFltLit > 80u);

        // Crank LP RES -> 0.98 at 800 Hz: Korg35 horn reaches top viewport rows y = 26..36
        fltFullSt.voices[0].lpCutoffHz = 800.0f;
        fltFullSt.voices[0].lpResonance = 0.98f;
        for (int step = 0; step < 12; ++step) {
            fltFullRenderer.prepareFrame(0.040f, fltFullSt, a3Ctrl.hud(), calmEngine.telemetry());
        }
        fltFullRenderer.renderFullScreenStrips(img16.data());
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 8u)) {
            fltFullRenderer.renderStrip(y, 8u, img8.data() + static_cast<size_t>(y) * 240u);
        }
        for (uint16_t y = 0u; y < 240u; y = static_cast<uint16_t>(y + 24u)) {
            fltFullRenderer.renderStrip(y, 24u, img24.data() + static_cast<size_t>(y) * 240u);
        }
        bool fltStripEq = true;
        size_t topViewportHornLit = 0u;
        for (size_t i = 0u; i < img16.size(); ++i) {
            if (img16[i] != img8[i] || img16[i] != img24[i]) {
                fltStripEq = false;
            }
        }
        for (size_t y = 26u; y <= 36u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                if (((img16[y * 240u + x] >> 5u) & 0x3Fu) >= 16u) {
                    ++topViewportHornLit;
                }
            }
        }
        CHECK(fltStripEq);
        CHECK(topViewportHornLit > 20u);
    }

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
    CHECK(rmsYActive > 28.0);

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
    CHECK(rmsYStop80ms < rmsYActive * 0.94);

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

    // (d) Closing EDIT Page 2 (PAN) holds PAN labels smoothly during dissolve (no FREQUENCY snap)
    trCtrl.onButtonEvent(ruido::ButtonId::Edit, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Edit, false);
    trCtrl.onButtonEvent(ruido::ButtonId::Edit, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Edit, false);
    CHECK(trCtrl.hud().panPageOpen);
    for (int f = 0; f < 12; ++f) {
        trCtrl.tick(0.030f, trEngine.telemetry());
        ruido::HudState h = trCtrl.hud();
        h.scopeFrozen = true;
        trRenderer.prepareFrame(0.030f, trCtrl.state(), h, trEngine.telemetry());
    }
    trRenderer.renderFullScreenStrips(img16.data());

    trCtrl.onButtonEvent(ruido::ButtonId::Home, true);
    trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
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
    CHECK(lumLiveDimmed == 0.0);
    CHECK(lumLiveHomeMid > lumLiveHomeSettled * 0.02);
    CHECK(lumLiveHomeMid < lumLiveHomeSettled * 0.90);

    // (f) HOME -> DUTY & DRIFT keep translucent live waveform visible behind the 4 columns,
    //     whereas EDIT -> LEVEL fades background scope all the way to #000000:
    {
        auto sumInterColWaveGray = [](const std::array<uint16_t, 240u * 240u>& img) -> double {
            double sum = 0.0;
            // Sample inter-column gap x = 56..63 (strictly between Col 0 [9..51] & Col 1 [69..111])
            // away from LEVEL 70% knee line (y = 81), DUTY/DRIFT zero filaments (y = 104..118),
            // and shared baseline (y = 186):
            for (size_t y = 40u; y <= 170u; ++y) {
                if ((y >= 77u && y <= 85u) || (y >= 104u && y <= 118u)) {
                    continue;
                }
                for (size_t x = 56u; x <= 63u; ++x) {
                    sum += static_cast<double>((img[y * 240u + x] >> 5u) & 0x3Fu) * 4.0;
                }
            }
            return sum;
        };
        auto stepLiveFrames = [&](int frames) {
            for (int f = 0; f < frames; ++f) {
                for (int b = 0; b < 8; ++b) {
                    trEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                    trRenderer.ingestScopeRing(trEngine.scopeBuffer());
                }
                trCtrl.tick(0.032f, trEngine.telemetry());
                trRenderer.prepareFrame(0.032f, trCtrl.state(), trCtrl.hud(), trEngine.telemetry());
            }
        };

        trCtrl.onButtonEvent(ruido::ButtonId::Home, true); // 1st -> FREQUENCY (100% wave)
        trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        stepLiveFrames(10);
        trRenderer.renderFullScreenStrips(img16.data());
        const double lumWaveFreq = sumInterColWaveGray(img16);
        CHECK(lumWaveFreq > 400.0);

        const double keepFrac = 1.0 - static_cast<double>(ruido::kHomeSubBgDimDepth);

        trCtrl.onButtonEvent(ruido::ButtonId::Home, true); // 2nd -> DUTY (~12% translucent wave)
        trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        stepLiveFrames(10);
        trRenderer.renderFullScreenStrips(img16.data());
        const double lumWaveDuty = sumInterColWaveGray(img16);
        CHECK(lumWaveDuty > lumWaveFreq * (keepFrac * 0.35) &&
              lumWaveDuty < lumWaveFreq * (keepFrac * 2.20));

        trCtrl.onButtonEvent(ruido::ButtonId::Home, true); // 3rd -> DRIFT (~12% translucent wave)
        trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        stepLiveFrames(10);
        trRenderer.renderFullScreenStrips(img16.data());
        const double lumWaveDrift = sumInterColWaveGray(img16);
        CHECK(lumWaveDrift > lumWaveFreq * (keepFrac * 0.35) &&
              lumWaveDrift < lumWaveFreq * (keepFrac * 2.20));

        trCtrl.onButtonEvent(ruido::ButtonId::Edit, true); // EDIT -> LEVEL (0% opaque black BG)
        trCtrl.onButtonEvent(ruido::ButtonId::Edit, false);
        if (!trCtrl.hud().editPageOpen) {
            trCtrl.onButtonEvent(ruido::ButtonId::Edit, true);
            trCtrl.onButtonEvent(ruido::ButtonId::Edit, false);
        }
        CHECK(trCtrl.hud().editPageOpen);
        stepLiveFrames(10);
        trRenderer.renderFullScreenStrips(img16.data());
        const double lumWaveLevel = sumInterColWaveGray(img16);
        CHECK(lumWaveLevel == 0.0);

        trCtrl.onButtonEvent(ruido::ButtonId::Home, true); // Escape back to Clean Scope
        trCtrl.onButtonEvent(ruido::ButtonId::Home, false);
        stepLiveFrames(10);
    }

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
        for (uint8_t m = 1u; m < static_cast<uint8_t>(ruido::HudMode::Count); ++m) {
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

    // 2. SELECT encoder steps across all 8 Dual-LFO slots:
    //    LFO A OSC 1(0) · LFO B OSC 1(1) .. LFO A OSC 4(6) · LFO B OSC 4(7), clamped with 0 windup!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> 1 (LFO B OSC 1)
    CHECK(ctrl.hud().lfoVoiceFocus == 1u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    // Verify LFO B reaches exclusive A RATE (8) & A DEPTH (9) targets while LFO A clamps at Pan (7)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 40);
    CHECK(ctrl.state().voices[0].lfoBTarget == ruido::LfoTarget::LfoADepth);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -4);
    CHECK(ctrl.state().voices[0].lfoBTarget == ruido::LfoTarget::LfoARate);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -24); // Back to LpCutoff (2)
    CHECK(ctrl.state().voices[0].lfoBTarget == ruido::LfoTarget::LpCutoff);

    ctrl.onEncoderDelta(ruido::EncoderId::Select, 24); // -> 7 (LFO B OSC 4)
    CHECK(ctrl.hud().lfoVoiceFocus == 7u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);

    // Wind +200 extra CW ticks at LFO B OSC 4 rail -> stays at 7, -4 CCW detent steps to 6
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 200);
    CHECK(ctrl.hud().lfoVoiceFocus == 7u);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -4);
    CHECK(ctrl.hud().lfoVoiceFocus == 6u);
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);

    // Wind -200 CCW ticks past LFO A OSC 1 -> clamps at 0, +8 CW steps to 2 (LFO A OSC 2)
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -200);
    CHECK(ctrl.hud().lfoVoiceFocus == 0u);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1,
                        40); // LFO A clamps at Pan (7), never A RATE/DEPTH!
    CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::Pan);
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -20); // Back to LpCutoff (2)
    CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::LpCutoff);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);
    CHECK(ctrl.hud().lfoVoiceFocus == 2u); // LFO A OSC 2

    // 3. Per-slot editing on LFO A OSC 2 (KNOB 1-4 = TARGET | SHAPE | RATE | DEPTH) leaves OSC
    // 1/3/4 and LFO B OSC 2 untouched; Keybed GLO -> GLOBAL LFO scales only active LFOs when any >
    // 0%, and wakes LFO A (OSC 1..4) when all 8 LFOs are at 0%!
    const float v0ADepthBefore = ctrl.state().voices[0].lfoDepth;
    const float v2ADepthBefore = ctrl.state().voices[2].lfoDepth;
    const float v1BDepthBefore = ctrl.state().voices[1].lfoBDepth;
    const float v1ARateBefore = ctrl.state().voices[1].lfoRateHz;
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 16);  // +4 steps from LpCutoff(2) -> Amplitude(6)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2, 60);  // SHAPE
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 40);  // RATE
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 100); // DEPTH
    CHECK(ctrl.state().voices[1].lfoTarget == ruido::LfoTarget::Amplitude);
    CHECK(ctrl.state().voices[1].lfoShape > 0.40f);
    CHECK(ctrl.state().voices[1].lfoRateHz > v1ARateBefore * 1.5f);
    CHECK(ctrl.state().voices[1].lfoDepth > 0.55f);
    CHECK(near(ctrl.state().voices[1].lfoBDepth, v1BDepthBefore));
    CHECK(ctrl.state().voices[0].lfoTarget == ruido::LfoTarget::LpCutoff);
    CHECK(near(ctrl.state().voices[0].lfoDepth, v0ADepthBefore));
    CHECK(near(ctrl.state().voices[2].lfoDepth, v2ADepthBefore));

    // Also set OSC 2B to LfoARate at faster speed to verify smart GLOBAL LFO scaling + A:B lock!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 4); // -> 3 (OSC 2B)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 40); // Top rail LfoADepth(9)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, -4); // -> LfoARate(8)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 80); // Faster rate on 2B
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 60); // Active depth on 2B

    // 4. Momentary Keybed punch-in overrides LFO Page temporarily and returns cleanly on release!
    ctrl.onEncoderDelta(ruido::EncoderId::Select, -32); // Focus OSC 1A (0)
    ctrl.onEncoderDelta(ruido::EncoderId::Knob4, 150);  // Active depth on OSC 1A for telemetry test
    CHECK(ctrl.hud().mode == ruido::HudMode::VoiceLfo);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, true);
    CHECK(ctrl.hud().mode == ruido::HudMode::KeyboardPlay);
    ctrl.onBlackKeyEvent(ruido::BlackKeyId::Op4, false);
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
    beadCtrl.global.masterRun = true;
    beadCtrl.global.masterTrim = 0.0f;
    beadCtrl.voices[0].lfoDepth = 1.0f;
    beadCtrl.voices[0].lfoShape = 0.0f;  // Full-amplitude Sine spanning upper & lower viewport
    beadCtrl.voices[0].lfoBDepth = 0.0f; // Isolate solo LFO A1 before companion sibling test below
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

    // Verify companion sibling trace renders dimmed behind focused LFO A OSC 1 when LFO B > 0,
    // and verify header renders "LFO A OSC 1" / "LFO B OSC 1" / "LFO A OSC 2" cleanly!
    {
        ruido::EngineControlState sibCtrl = beadCtrl;
        sibCtrl.voices[0].lfoBDepth = 0.85f;
        sibCtrl.voices[0].lfoBShape = 0.75f; // Square sibling trace (differs from LFO A Sine!)
        customTel.lfoPhase[4].store(0.62f, std::memory_order_relaxed);
        for (int f = 0; f < 8; ++f) {
            beadRenderer.prepareFrame(0.050f, sibCtrl, ctrl.hud(), customTel);
        }
        beadRenderer.renderFullScreenStrips(lfo24.data());
        size_t companionDiffPixels = 0u;
        for (size_t y = 32u; y <= 182u; ++y) {
            for (size_t x = 0u; x < 240u; ++x) {
                if (lfo24[y * 240u + x] != lfo8[y * 240u + x]) {
                    ++companionDiffPixels;
                }
            }
        }
        CHECK(companionDiffPixels > 120u);

        // Compare top header (y = 6..20) across LFO A1 OSC 1 (slot 0), LFO B1 OSC 1 (slot 1),
        // and LFO A2 OSC 2 (slot 2), ensuring both "LFO A1..B4" and dimmed "OSC 1..4" render:
        ruido::HudState hudA1 = ctrl.hud();
        hudA1.lfoVoiceFocus = 0u;
        ruido::HudState hudB1 = ctrl.hud();
        hudB1.lfoVoiceFocus = 1u;
        ruido::HudState hudA2 = ctrl.hud();
        hudA2.lfoVoiceFocus = 2u;

        std::array<uint16_t, 240u * 24u> stripA1{};
        std::array<uint16_t, 240u * 24u> stripB1{};
        std::array<uint16_t, 240u * 24u> stripA2{};
        beadRenderer.prepareFrame(0.016f, sibCtrl, hudA1, customTel);
        beadRenderer.renderStrip(0u, 24u, stripA1.data());
        beadRenderer.prepareFrame(0.016f, sibCtrl, hudB1, customTel);
        beadRenderer.renderStrip(0u, 24u, stripB1.data());
        beadRenderer.prepareFrame(0.016f, sibCtrl, hudA2, customTel);
        beadRenderer.renderStrip(0u, 24u, stripA2.data());

        bool lfoPrefixMatches = true;
        size_t lfoABMainDiff = 0u;
        size_t osc12DigitDiff = 0u;
        size_t oscSuffixLit = 0u;
        size_t osc12SuffixDiff = 0u;
        for (size_t y = 6u; y <= 20u; ++y) {
            for (size_t x = 10u; x <= 35u; ++x) {
                const size_t idx = y * 240u + x;
                if (stripA1[idx] != stripA2[idx] || stripA1[idx] != stripB1[idx]) {
                    lfoPrefixMatches = false;
                }
            }
            for (size_t x = 36u; x <= 48u; ++x) {
                if (stripA1[y * 240u + x] != stripB1[y * 240u + x]) {
                    ++lfoABMainDiff;
                }
            }
            for (size_t x = 46u; x <= 66u; ++x) {
                if (stripA1[y * 240u + x] != stripA2[y * 240u + x]) {
                    ++osc12DigitDiff;
                }
            }
            for (size_t x = 68u; x <= 118u; ++x) {
                if (stripA1[y * 240u + x] != 0u) {
                    ++oscSuffixLit;
                }
                if (stripA1[y * 240u + x] != stripA2[y * 240u + x]) {
                    ++osc12SuffixDiff;
                }
            }
        }
        CHECK(lfoPrefixMatches);
        CHECK(lfoABMainDiff > 8u);
        CHECK(osc12DigitDiff > 8u);
        CHECK(oscSuffixLit > 20u);
        CHECK(osc12SuffixDiff > 8u);
    }

    // 6. Close LFO Page via HOME: active LFOs keep running and LFO LED pulses live!
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    CHECK(!ctrl.hud().lfoPageOpen);
    CHECK(ctrl.hud().knobBank == ruido::KnobBankMode::Frequency);

    for (size_t li = 0u; li < ruido::kNumVoiceLfos; ++li) {
        customTel.lfoOutput[li].store(-1.0f, std::memory_order_relaxed);
    }
    ctrl.tick(0.016f, customTel);
    const float ledTrough = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

    customTel.lfoOutput[0].store(1.0f, std::memory_order_relaxed);
    ctrl.tick(0.016f, customTel);
    const float ledCrest = ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)];

    CHECK(ledTrough >= 0.15f && ledTrough < 0.25f);
    CHECK(ledCrest > 0.34f && ledCrest <= 0.375f);
    CHECK(ledCrest > ledTrough + 0.12f);
}

void testBootIntroCrtSequence() {
    std::printf("[Test 8] Cold-Boot CRT RUIDO Intro Sequence, Scanlines & Fast-Dismiss...\n");
    static ruido::ScopeHudRenderer renderer{};
    ruido::EngineControlState ctrl = ruido::EngineControlState::makeDefault();
    ctrl.global.masterRun = false; // Silent flatline after intro so background check is isolated
    ruido::HudState hud{};
    ruido::EngineTelemetry tel{};
    static std::array<uint16_t, ruido::kDisplayWidth * ruido::kDisplayHeight> frame{};

    renderer.reset();
    CHECK(!renderer.isBootIntroActive());

    // 1. Arm cold-boot intro -> Phase 1 Beam Strike (~40 ms)
    renderer.armBootIntro();
    CHECK(renderer.isBootIntroActive());
    renderer.prepareFrame(0.040f, ctrl, hud, tel);
    renderer.renderFullScreenStrips(frame.data());

    bool neutralAll = true;
    uint32_t outerLit = 0u;
    for (size_t y = 0u; y < ruido::kDisplayHeight; ++y) {
        for (size_t x = 0u; x < ruido::kDisplayWidth; ++x) {
            const uint16_t px = frame[y * ruido::kDisplayWidth + x];
            const uint16_t r5 = (px >> 11u) & 0x1Fu;
            const uint16_t b5 = px & 0x1Fu;
            if (r5 != b5) {
                neutralAll = false;
            }
            if ((y < 64u || y >= 176u) && px != 0x0000u) {
                ++outerLit;
            }
        }
    }
    CHECK(neutralAll);
    CHECK(outerLit == 0u); // Pure #000000 background outside CRT beam
    const uint16_t beamCenterR5 = (frame[120u * 240u + 120u] >> 11u) & 0x1Fu;
    CHECK(beamCenterR5 >= 28u);

    // 2. Advance into settled RUIDO Hold (~690 ms): verify 1.00x wordmark, O counter hole,
    //    scanlineDepth = 0.0 (continuous vertical phosphor), and tight phosphor halo!
    for (int i = 0; i < 13; ++i) {
        renderer.prepareFrame(0.050f, ctrl, hud, tel);
    }
    CHECK(renderer.isBootIntroActive());
    renderer.renderFullScreenStrips(frame.data());

    uint32_t evenSum = 0u;
    uint32_t oddSum = 0u;
    for (size_t y = 94u; y < 144u; ++y) {
        for (size_t x = 0u; x < ruido::kDisplayWidth; ++x) {
            const uint16_t px = frame[y * 240u + x];
            if (((px >> 11u) & 0x1Fu) != (px & 0x1Fu)) {
                neutralAll = false;
            }
            const uint32_t r5 = (px >> 11u) & 0x1Fu;
            if ((y & 1u) == 0u) {
                evenSum += r5;
            } else {
                oddSum += r5;
            }
        }
    }
    CHECK(neutralAll);
    CHECK(evenSum > 1000u && oddSum > 1000u);

    // 'I' vertical stem around x=114, y=120 vs 'O' outer ring x=207, inner counter hole x=190
    const uint16_t stemI = (frame[120u * 240u + 114u] >> 11u) & 0x1Fu;
    const uint16_t ringO = (frame[120u * 240u + 207u] >> 11u) & 0x1Fu;
    const uint16_t holeO = (frame[120u * 240u + 190u] >> 11u) & 0x1Fu;
    CHECK(stemI == 25u); // Settled coreLum=205 -> exact 5-bit CRT level 25!
    CHECK(ringO == 25u);
    CHECK(holeO < ringO - 6u);

    // 3. Advance through +1.00 s extended hold + 180 ms collapse-y exit (2.00 s total) -> hands off
    for (int i = 0; i < 20; ++i) {
        renderer.prepareFrame(0.050f, ctrl, hud, tel);
    }
    CHECK(renderer.isBootIntroActive()); // Still active at ~1.69 s (< 1.82 s hold end)!
    for (int i = 0; i < 8; ++i) {
        renderer.prepareFrame(0.050f, ctrl, hud, tel);
    }
    CHECK(!renderer.isBootIntroActive());

    // 4. Touching any control mid-intro triggers smooth fast-collapse (not jarring cut)
    renderer.armBootIntro();
    renderer.prepareFrame(0.050f, ctrl, hud, tel);
    renderer.prepareFrame(0.050f, ctrl, hud, tel);
    renderer.notifyUserInput();
    CHECK(renderer.isBootIntroActive());
    for (int i = 0; i < 4; ++i) {
        renderer.prepareFrame(0.050f, ctrl, hud, tel);
    }
    CHECK(!renderer.isBootIntroActive());
}

void testRailCouplingHudAnalyzer() {
    std::printf(
        "[Test 8b] Rail Coupling HUD Analyzer (0%%, ~32%% Pull & ~78%% Starved Regimes)...\n");
    ruido::DroneEngine engine;
    ruido::RuidoController ctrl;
    ruido::ScopeHudRenderer renderer;

    engine.init(ruido::kBaseSampleRate);
    ctrl.attachControlBus(&engine.controlBus());
    engine.snapParametersToLatest();
    CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::LiveMixer);

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    static std::array<uint16_t, 240u * 240u> frameZero{};
    static std::array<uint16_t, 240u * 240u> framePull{};
    static std::array<uint16_t, 240u * 240u> frameStarved{};

    const auto renderCouplingState = [&](std::array<uint16_t, 240u * 240u>& outFrame) {
        engine.snapParametersToLatest();
        for (int f = 0; f < 10; ++f) {
            for (int b = 0; b < 8; ++b) {
                engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                renderer.ingestScopeRing(engine.scopeBuffer());
            }
            ctrl.tick(0.025f, engine.telemetry());
            renderer.prepareFrame(0.025f, ctrl.state(), ctrl.hud(), engine.telemetry());
        }
        renderer.renderFullScreenStrips(outFrame.data());
    };

    // 1. Turn ALGORITHM CCW in LiveMixer mode to 0% coupling baseline
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -100);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(near(ctrl.state().global.circuitCoupling, 0.0f));
    renderCouplingState(frameZero);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // 2. Turn ALGORITHM CW into the 0..45% sympathetic pull regime (~32%)
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 76);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(ctrl.state().global.circuitCoupling > 0.30f &&
          ctrl.state().global.circuitCoupling < 0.45f);
    renderCouplingState(framePull);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    // 3. Turn ALGORITHM CW into the >45% starved-rail regime (~78%)
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 110);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(ctrl.state().global.circuitCoupling > 0.75f &&
          ctrl.state().global.circuitCoupling < 0.82f);
    renderCouplingState(frameStarved);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(near(ctrl.hud().opacity, 1.0f));

    const auto verifyRegimeFrame = [&](const std::array<uint16_t, 240u * 240u>& img) {
        size_t upperLobeLit = 0u;
        size_t lowerLobeLit = 0u;
        size_t botCaptionLit = 0u;
        for (size_t y = 36u; y <= 108u; ++y) {
            for (size_t x = 10u; x < 230u; ++x) {
                if (((img[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                    ++upperLobeLit;
                }
            }
        }
        for (size_t y = 114u; y <= 185u; ++y) {
            for (size_t x = 10u; x < 230u; ++x) {
                if (((img[y * 240u + x] >> 5u) & 0x3Fu) >= 6u) {
                    ++lowerLobeLit;
                }
            }
        }
        for (size_t y = 198u; y <= 224u; ++y) {
            for (size_t x = 20u; x < 220u; ++x) {
                if (((img[y * 240u + x] >> 5u) & 0x3Fu) >= 8u) {
                    ++botCaptionLit;
                }
            }
        }
        CHECK(upperLobeLit > 600u);
        CHECK(lowerLobeLit > 600u);
        CHECK(botCaptionLit > 80u);
    };

    verifyRegimeFrame(frameZero);
    verifyRegimeFrame(framePull);
    verifyRegimeFrame(frameStarved);

    // 4. Verify live real-audio motion in BOTH Soft Pull (~24%) AND Starved Hard Sync (~80%):
    ctrl.onEncoderDelta(ruido::EncoderId::Knob2,
                        29); // Detune Osc 2 for lively multi-voice phase & sync interaction
    ctrl.onEncoderDelta(ruido::EncoderId::Knob3, 19);
    static std::array<uint16_t, 240u * 240u> frameLiveA{};
    static std::array<uint16_t, 240u * 240u> frameLiveB{};

    // First verify at ~80% Starved Hard Sync (must NEVER freeze while real oscillators slip/tear!)
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 8);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(ctrl.state().global.circuitCoupling > 0.78f);
    renderCouplingState(frameLiveA);
    renderCouplingState(frameLiveB);
    size_t hardSyncMotionDiffs = 0u;
    for (size_t y = 38u; y <= 182u; ++y) {
        for (size_t x = 48u; x < 228u; ++x) {
            if (frameLiveA[y * 240u + x] != frameLiveB[y * 240u + x]) {
                ++hardSyncMotionDiffs;
            }
        }
    }
    CHECK(hardSyncMotionDiffs > 100u);

    // Next verify at ~24% Soft Pull + 3.0s toast hold:
    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, -135);
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    renderCouplingState(frameLiveA);
    renderCouplingState(frameLiveB);
    size_t osc2BeatDiffs = 0u;
    for (size_t y = 104u; y <= 125u; ++y) {
        for (size_t x = 48u; x < 228u; ++x) {
            if (frameLiveA[y * 240u + x] != frameLiveB[y * 240u + x]) {
                ++osc2BeatDiffs;
            }
        }
    }
    CHECK(osc2BeatDiffs > 60u);

    // After 0.50 s already elapsed in renderCouplingState, tick another 2.20 s (2.70 s total < 3.0
    // s):
    for (int i = 0; i < 22; ++i) {
        ctrl.tick(0.100f, engine.telemetry());
    }
    CHECK(ctrl.hud().mode == ruido::HudMode::RailCoupling);
    CHECK(near(ctrl.hud().opacity, 1.0f));
    // Tick another 0.50 s (3.20 s total > 3.0 s + 0.15 s fade) -> cleanly faded out!
    for (int i = 0; i < 5; ++i) {
        ctrl.tick(0.100f, engine.telemetry());
    }
    CHECK(near(ctrl.hud().opacity, 0.0f));

    size_t pullVsZeroGraphDiffs = 0u;
    size_t starvedVsZeroGraphDiffs = 0u;
    size_t starvedVsPullGraphDiffs = 0u;
    size_t pullVsZeroBotDiffs = 0u;
    size_t starvedVsZeroBotDiffs = 0u;
    for (size_t y = 40u; y <= 185u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            const size_t idx = y * 240u + x;
            if (framePull[idx] != frameZero[idx]) {
                ++pullVsZeroGraphDiffs;
            }
            if (frameStarved[idx] != frameZero[idx]) {
                ++starvedVsZeroGraphDiffs;
            }
            if (frameStarved[idx] != framePull[idx]) {
                ++starvedVsPullGraphDiffs;
            }
        }
    }
    for (size_t y = 190u; y <= 230u; ++y) {
        for (size_t x = 0u; x < 240u; ++x) {
            const size_t idx = y * 240u + x;
            if (framePull[idx] != frameZero[idx]) {
                ++pullVsZeroBotDiffs;
            }
            if (frameStarved[idx] != frameZero[idx]) {
                ++starvedVsZeroBotDiffs;
            }
        }
    }
    CHECK(pullVsZeroGraphDiffs > 40u);
    CHECK(starvedVsZeroGraphDiffs > 80u);
    CHECK(starvedVsPullGraphDiffs > 40u);
    CHECK(pullVsZeroBotDiffs > 10u);
    CHECK(starvedVsZeroBotDiffs > 10u);
}

void testPlayStopFreezesAllVisualsAndLeds() {
    std::printf("[Test 8c] PLAY/STOP Visual & 42-LED Freeze Across All Screens...\n");
    static ruido::DroneEngine engine;
    static ruido::RuidoController ctrl;
    static ruido::ScopeHudRenderer renderer;
    static std::array<float, 128> outL{};
    static std::array<float, 128> outR{};
    static std::array<uint16_t, 240u * 240u> frameA{};
    static std::array<uint16_t, 240u * 240u> frameB{};

    engine.init(44100.0f);
    ctrl.reset();
    renderer.reset();

    // Switch to 3/3 RHYTHM LAB so TAP (`POLY`) beat-glow is also exercised
    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    ctrl.onEncoderDelta(ruido::EncoderId::Select, 8);
    CHECK(ctrl.hud().keyboardModel == ruido::KeyboardModel::RhythmLab);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);

    // Patch heavy live modulation across all 4 voices (Pitch, Duty, Level+Overdrive, Pan, LPF,
    // Drift, Rail Coupling, and snapshotDirty) so every screen has active motion while running.
    ruido::EngineControlState modPatch = ctrl.state();
    modPatch.global.circuitCoupling = 0.65f;
    modPatch.voices[0].amplitude = 1.00f;
    modPatch.voices[0].drift = 0.85f;
    modPatch.voices[0].lpCutoffHz = 1400.0f;
    modPatch.voices[0].lpResonance = 0.55f;
    modPatch.voices[0].lfoTarget = ruido::LfoTarget::Pan;
    modPatch.voices[0].lfoRateHz = 3.2f;
    modPatch.voices[0].lfoDepth = 0.85f;
    modPatch.voices[0].lfoBTarget = ruido::LfoTarget::Amplitude;
    modPatch.voices[0].lfoBRateHz = 2.4f;
    modPatch.voices[0].lfoBDepth = 0.75f;

    modPatch.voices[1].drift = 0.75f;
    modPatch.voices[1].lfoTarget = ruido::LfoTarget::Pitch;
    modPatch.voices[1].lfoRateHz = 4.1f;
    modPatch.voices[1].lfoDepth = 0.80f;
    modPatch.voices[1].lfoBTarget = ruido::LfoTarget::PulseWidth;
    modPatch.voices[1].lfoBRateHz = 2.9f;
    modPatch.voices[1].lfoBDepth = 0.70f;

    modPatch.voices[2].lfoTarget = ruido::LfoTarget::LpCutoff;
    modPatch.voices[2].lfoRateHz = 3.7f;
    modPatch.voices[2].lfoDepth = 0.85f;
    modPatch.voices[2].lfoShape = 0.95f; // S&H / Drift knots

    CHECK(ctrl.loadLivePatch(ruido::encodePatchSnapshot(modPatch)));
    ctrl.onEncoderDelta(ruido::EncoderId::Knob1, 4); // Mark snapshotDirty = true

    const auto stepLive = [&](int frames60Hz) {
        for (int f = 0; f < frames60Hz; ++f) {
            engine.controlBus().publish(ctrl.state());
            for (int b = 0; b < 6; ++b) {
                engine.processBlock(outL.data(), outR.data(), outL.size());
                renderer.ingestScopeRing(engine.scopeBuffer());
            }
            ctrl.tick(0.016f, engine.telemetry());
            renderer.prepareFrame(0.016f, ctrl.state(), ctrl.hud(), engine.telemetry());
        }
    };

    stepLive(18);
    CHECK(ctrl.state().global.masterRun);

    // Stop sound via PLAY/STOP button!
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    CHECK(!ctrl.state().global.masterRun);

    // Let 8 ms audio ramp + scope phosphor decay + LED smoothers settle completely (~900 ms)
    stepLive(60);

    const float frozenLfoPhase0 = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    const float frozenRhythmPhase = engine.telemetry().rhythmPhase.load(std::memory_order_relaxed);
    stepLive(12);
    CHECK(near(engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed), frozenLfoPhase0));
    CHECK(near(engine.telemetry().rhythmPhase.load(std::memory_order_relaxed), frozenRhythmPhase));
    CHECK(near(engine.telemetry().lfoOutput[0].load(std::memory_order_relaxed), 0.0f));
    CHECK(near(engine.telemetry().voicePeak[0].load(std::memory_order_relaxed), 0.0f));
    CHECK(engine.telemetry().stutterGateMask.load(std::memory_order_relaxed) == 0x0Fu);

    const auto assertScreenAndLedsMotionless = [&](const char* label) {
        (void)label;
        stepLive(24);
        renderer.renderFullScreenStrips(frameA.data());
        const ruido::LedState ledsA = ctrl.leds();

        stepLive(10);
        renderer.renderFullScreenStrips(frameB.data());
        const ruido::LedState ledsB = ctrl.leds();

        bool pixelsIdentical = true;
        for (size_t i = 0u; i < frameA.size(); ++i) {
            if (frameA[i] != frameB[i]) {
                pixelsIdentical = false;
                break;
            }
        }
        bool ledsIdentical = near(ledsA.stopLed, ledsB.stopLed);
        for (size_t i = 0u; i < ledsA.buttonLeds.size(); ++i) {
            if (!near(ledsA.buttonLeds[i], ledsB.buttonLeds[i])) {
                ledsIdentical = false;
            }
        }
        for (size_t i = 0u; i < ledsA.blackKeyLeds.size(); ++i) {
            if (!near(ledsA.blackKeyLeds[i], ledsB.blackKeyLeds[i])) {
                ledsIdentical = false;
            }
        }
        for (size_t i = 0u; i < ledsA.whiteKeyLeds.size(); ++i) {
            if (!near(ledsA.whiteKeyLeds[i], ledsB.whiteKeyLeds[i])) {
                ledsIdentical = false;
            }
        }
        CHECK(pixelsIdentical);
        CHECK(ledsIdentical);
    };

    // 1. Dismiss AUDIO OUTPUT OFF banner -> Idle Scope + Closed Panel LEDs (FX, LFO, SAVE, TAP)
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false);
    assertScreenAndLedsMotionless("IdleScope");
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Fx)], 0.15f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Lfo)], 0.15f));
    CHECK(near(ctrl.leds().buttonLeds[static_cast<size_t>(ruido::ButtonId::Save)], 0.15f));
    CHECK(near(ctrl.leds().blackKeyLeds[ruido::kTapBlackKeyIdx], 0.15f));

    // 2. LFO Screen (header reads "STOPPED" and curve has zero moving phase beacon)
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, true);
    ctrl.onButtonEvent(ruido::ButtonId::Lfo, false);
    assertScreenAndLedsMotionless("LFO");
    {
        ruido::LfoSignalArray zeroLfo{};
        ruido::PreparedHudHeader hdr{};
        ruido::prepareHudHeader(hdr, ctrl.state(), ctrl.hud(), zeroLfo, 0x0Fu, 0.0f, 0.0f);
        CHECK(std::strcmp(hdr.rightText, "STOPPED") == 0);
    }

    // 3. EDIT -> LEVEL (100% overdrive stem settled & motionless) and EDIT -> PAN
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    assertScreenAndLedsMotionless("LEVEL");
    ctrl.onButtonEvent(ruido::ButtonId::Edit, true);
    ctrl.onButtonEvent(ruido::ButtonId::Edit, false);
    assertScreenAndLedsMotionless("PAN");

    // 4. HOME -> FREQUENCY, DUTY, DRIFT
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false); // Dismiss PAN
    stepLive(12);
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false); // 1st -> FREQUENCY
    assertScreenAndLedsMotionless("FREQUENCY");
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false); // 2nd -> DUTY
    assertScreenAndLedsMotionless("DUTY");
    ctrl.onButtonEvent(ruido::ButtonId::Home, true);
    ctrl.onButtonEvent(ruido::ButtonId::Home, false); // 3rd -> DRIFT
    assertScreenAndLedsMotionless("DRIFT");

    // 5. FX Filter Screen & RAIL COUPLING Analyzer & SEL Guidance
    ctrl.onButtonEvent(ruido::ButtonId::Fx, true);
    ctrl.onButtonEvent(ruido::ButtonId::Fx, false);
    assertScreenAndLedsMotionless("FX");

    ctrl.onEncoderDelta(ruido::EncoderId::Algorithm, 4);
    assertScreenAndLedsMotionless("RAIL_COUPLING");

    ctrl.onButtonEvent(ruido::ButtonId::Sel, true);
    ctrl.onButtonEvent(ruido::ButtonId::Sel, false);
    assertScreenAndLedsMotionless("SEL");

    // 6. Resume via PLAY/STOP -> LFO phase & audio peaks immediately resume!
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
    ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
    CHECK(ctrl.state().global.masterRun);
    stepLive(8);
    CHECK(!near(engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed), frozenLfoPhase0));
    CHECK(engine.telemetry().peakLeft.load(std::memory_order_relaxed) > 0.05f);
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
    testBootIntroCrtSequence();
    testRailCouplingHudAnalyzer();
    testPlayStopFreezesAllVisualsAndLeds();
    ruido_test::runUnified150msFadesAndRateSplitHeaderTests(check);
    ruido_test::runGlobalFiveModesAndConfigMenuTests(check);
    ruido_test::runEditChannelDrivePageAndAmpGraphTests(check);
    ruido_test::runKeyboardModelAndLiveMixerTests(check);
    ruido_test::runRhythmLabAndMidiClockTests(check);
    ruido_test::runFm1PanelIoAndLedModulatorTests(check);

    if (gFailures == 0) {
        std::printf("\nALL UI & CONTROLLER VERIFICATION GATES PASSED (0 failures).\n");
        return 0;
    }
    std::fprintf(stderr, "\nFAILED with %d error(s).\n", gFailures);
    return 1;
}
