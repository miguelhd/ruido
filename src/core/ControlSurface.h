#pragma once

#include "core/Config.h"
#include "core/GlobalConfig.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr size_t kNumEncoders = 7u;
inline constexpr size_t kNumPanelButtons = 14u;
inline constexpr size_t kNumBlackKeys = 11u;
inline constexpr size_t kNumWhiteKeys = 16u;
inline constexpr size_t kNumLedChannels =
    kNumBlackKeys + kNumWhiteKeys + kNumPanelButtons + 1u; // 42 physical LED channels

inline constexpr uint8_t kNoFocusedKnob = 0xFFu;

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

// 5 multi-voice destinations selected via SELECT while holding keybed GLO
enum class GlobalTarget : uint8_t { Freq = 0, Filter = 1, Lfo = 2, Amp = 3, All = 4, Count = 5 };

// Active mapping of top-right KNOB1..KNOB4 (displayed in the 4 bottom HUD columns unless None)
enum class KnobBankMode : uint8_t {
    Frequency = 0,
    VoiceFilter = 1,
    Amplitude = 2,
    Panning = 3,
    Lfo = 4,
    GlobalFreq = 5,
    GlobalFilter = 6,
    GlobalLfo = 7,
    GlobalAmp = 8,
    GlobalAll = 9,
    None = 10
};

// Encoder sweep rate selected via OCT- (Fine) and OCT+ (Coarse: ~1.3 turns full sweep)
enum class KnobRate : int8_t { Fine = -1, Standard = 0, Coarse = 1 };

// Active function displayed in the top HUD header whenever any control or key is touched
enum class HudMode : uint8_t {
    None = 0,
    VoiceFrequency = 1,
    VoiceFilter = 2,
    Amplitude = 3,
    Panning = 4,
    MasterLevel = 5,
    RailCoupling = 6,
    UnassignedEncoder = 7,
    KnobRate = 8,
    VoiceGate = 9,
    VoicePressure = 10,
    SnapshotSlot = 11,
    UnassignedKey = 12,
    OutputRouting = 13,
    AudioOutput = 14,
    SaveArmed = 15,
    ScopeFreeze = 16,
    UnassignedButton = 17,
    PressureTarget = 18,
    VoiceLfo = 19,
    GlobalFreq = 20,
    GlobalFilter = 21,
    GlobalLfo = 22,
    GlobalAmp = 23,
    GlobalAll = 24,
    GlobalConfig = 25
};

// Matched monochrome PWM LED rest brightness shared by SEL and White Keys 5-8
inline constexpr float kPadFilterBaseLed = 0.24f;
inline constexpr float kPadPitchBaseLed = 0.72f;

struct HudState {
    HudMode mode = HudMode::None;
    KnobBankMode knobBank = KnobBankMode::Frequency;
    KnobRate knobRate = KnobRate::Standard;
    GlobalTarget globalTarget = GlobalTarget::All;
    GlobalConfigState config{};
    ButtonId lastButton = ButtonId::Count;
    BlackKeyId lastBlackKey = BlackKeyId::Count;
    EncoderId lastEncoder = EncoderId::Select;
    uint8_t focusedVoice = 0u;            // 0..3 oscillator index
    uint8_t focusedKnob = kNoFocusedKnob; // 0..3 active column highlight, or kNoFocusedKnob
    uint8_t lfoVoiceFocus = 0u;           // 0..3 = OSC 1..4 (SELECT encoder on LFO Page)
    uint8_t configCursor = 0u;            // 0..kNumGlobalConfigItems-1 (SELECT in GLOBAL CONFIG)
    uint8_t snapshotSlot = 0u;            // 0..7 for White Keys 9..16
    float morphPosition = 0.0f;           // [0.0f, 1.0f] snapshot pressure morph position
    float opacity = 0.0f;                 // [0.0f, 1.0f] smooth 150 ms S-curve alpha envelope
    bool modifierHeld = false;
    bool rateHoldActive = false; // True whenever OCT- or OCT+ is physically held
    bool scopeFrozen = false;    // Momentary waveform freeze on 11th black key
    bool saveArmed = false;
    bool lfoPageOpen = false;    // Latching LFO page state (ButtonId::Lfo)
    bool editPageOpen = false;   // Latching Channel Level & Drive page (ButtonId::Edit)
    bool configPageOpen = false; // Latching GLOBAL CONFIG menu (ButtonId::GloRight)
};

// Software-PWM brightness [0.0f, 1.0f] across all 42 physical LED channels on the M-VAVE FM-1:
// - blackKeyLeds[0..10] & whiteKeyLeds[0..15]: 27 monochrome white silicone key LEDs
// - buttonLeds[0..11]: 12 monochrome white function/octave button LEDs (OCT-/+, FX..SEQ)
// - buttonLeds[12] (PlayStop): Green PLAY LED under the split PLAY/STOP cap (lit when quiet)
// - buttonLeds[13] (Rec): Red LED under the REC button
// - stopLed: Red STOP LED under the split PLAY/STOP cap (lit when sound is ON)
struct LedState {
    std::array<float, kNumBlackKeys> blackKeyLeds{};
    std::array<float, kNumWhiteKeys> whiteKeyLeds{};
    std::array<float, kNumPanelButtons> buttonLeds{};
    float stopLed = 0.0f;
};
static_assert(sizeof(LedState) == kNumLedChannels * sizeof(float),
              "LedState must pack contiguously as 42 floats for Wasm/DMA transport");

} // namespace ruido
