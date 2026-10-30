#pragma once

#include "core/Config.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr size_t kNumEncoders = 7u;
inline constexpr size_t kNumPanelButtons = 14u;
inline constexpr size_t kNumBlackKeys = 11u;
inline constexpr size_t kNumWhiteKeys = 16u;

// 7 endless quadrature encoders on the M-VAVE FM-1 front panel
enum class EncoderId : uint8_t {
    Select = 0,
    Presets = 1,
    Algorithm = 2,
    Knob1 = 3,
    Knob2 = 4,
    Knob3 = 5,
    Knob4 = 6,
    Count = 7
};

// 14 tactile panel buttons (left OCT-/OCT+ + right 2x6 grid)
enum class ButtonId : uint8_t {
    OctMinus = 0,
    OctPlus = 1,
    Fx = 2,
    Sel = 3,
    Env = 4,
    Lfo = 5,
    Edit = 6,
    GloRight = 7,
    Home = 8,
    Save = 9,
    Arp = 10,
    Seq = 11,
    PlayStop = 12,
    Rec = 13,
    Count = 14
};

// 11 raised black keys across the 27-key silicone bed (F#3 .. F#5, left to right)
enum class BlackKeyId : uint8_t {
    Op1 = 0,
    Op2 = 1,
    Op3 = 2,
    Op4 = 3,
    Op5 = 4,
    Op6 = 5,
    Pit = 6,
    Glo = 7,
    Mono = 8,
    Poly = 9,
    ScopeFreezeKey = 10,
    Count = 11
};

// Active mapping of top-right KNOB1..KNOB4 (always displayed in the 4 bottom HUD columns)
enum class KnobBankMode : uint8_t {
    Frequency = 0,
    Amplitude = 1,
    Panning = 2,
    VoiceFilter = 3,
    GlobalFilter = 4,
    Lfo = 5
};

// Encoder sweep rate selected via OCT- (Fine) and OCT+ (Fast: 1-1.5 turns full sweep)
enum class KnobRate : int8_t {
    Fine = -1,
    Standard = 0,
    Fast = 1
};

// Active function displayed in the top HUD header whenever any control or key is touched
enum class HudMode : uint8_t {
    None = 0,
    VoiceFrequency = 1,
    Amplitude = 2,
    Panning = 3,
    VoiceFilter = 4,
    GlobalFilter = 5,
    MasterLevel = 6,
    RailCoupling = 7,
    UnassignedEncoder = 8,
    SnapshotMorph = 9,
    KnobRate = 10,
    VoiceGate = 11,
    VoicePressure = 12,
    SnapshotSlot = 13,
    UnassignedKey = 14,
    OutputRouting = 15,
    AudioOutput = 16,
    SaveArmed = 17,
    ScopeFreeze = 18,
    UnassignedButton = 19,
    PressureTarget = 20,
    VoiceLfo = 21,
    GlobalLfo = 22
};

// Matched monochrome PWM LED rest brightness shared by SEL and White Keys 5-8
inline constexpr float kPadFilterBaseLed = 0.24f;
inline constexpr float kPadPitchBaseLed = 0.72f;

struct HudState {
    HudMode mode = HudMode::None;
    KnobBankMode knobBank = KnobBankMode::Frequency;
    KnobRate knobRate = KnobRate::Standard;
    ButtonId lastButton = ButtonId::Count;
    uint8_t focusedVoice = 0u;   // 0..3 oscillator or knob index
    uint8_t focusedKnob = 0u;    // 0..3 active column highlight
    uint8_t lfoVoiceFocus = 0u;  // 0..3 = OSC 1..4, 4 = ALL 4 (SELECT encoder on LFO Page)
    uint8_t snapshotSlot = 0u;   // 0..7 for White Keys 9..16
    float morphPosition = 0.0f;  // [0.0f, 1.0f] PRESETS scrub position
    float opacity = 0.0f;        // [0.0f, 1.0f] smooth S-curve alpha envelope
    bool modifierHeld = false;
    bool scopeFrozen = false;    // Momentary waveform freeze on 11th black key
    bool saveArmed = false;
    bool lfoPageOpen = false;    // Latching LFO page state (ButtonId::Lfo)
};

// Monochrome white software-PWM brightness [0.0f, 1.0f] for all 27 keys + 14 panel buttons
struct LedState {
    std::array<float, kNumBlackKeys> blackKeyLeds{};
    std::array<float, kNumWhiteKeys> whiteKeyLeds{};
    std::array<float, kNumPanelButtons> buttonLeds{};
};

} // namespace ruido
