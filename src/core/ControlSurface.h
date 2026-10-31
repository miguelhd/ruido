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

// 3 per-oscillator modes selected via discrete SELECT on the latching EDIT page
// (signal-flow order: Osc Width -> Osc Drift -> Channel Level/Drive; default = Level)
enum class EditTarget : uint8_t { Width = 0, Drift = 1, Level = 2, Count = 3 };

// Active mapping of top-right KNOB1..KNOB4 (displayed in the 4 bottom HUD columns unless None)
enum class KnobBankMode : uint8_t {
    Frequency = 0,
    PulseWidth = 1,
    VoiceDrift = 2,
    VoiceFilter = 3,
    Amplitude = 4,
    Panning = 5,
    Lfo = 6,
    GlobalFreq = 7,
    GlobalFilter = 8,
    GlobalLfo = 9,
    GlobalAmp = 10,
    GlobalAll = 11,
    None = 12
};

// Encoder sweep rate selected via OCT- (Fine) and OCT+ (Coarse: ~1.3 turns full sweep)
enum class KnobRate : int8_t { Fine = -1, Standard = 0, Coarse = 1 };

// Active function displayed in the top HUD header whenever any control or key is touched
enum class HudMode : uint8_t {
    None = 0,
    VoiceFrequency = 1,
    PulseWidth = 2,
    VoiceDrift = 3,
    VoiceFilter = 4,
    Amplitude = 5,
    Panning = 6,
    MasterLevel = 7,
    RailCoupling = 8,
    UnassignedEncoder = 9,
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
    VoiceLfo = 20,
    GlobalFreq = 21,
    GlobalFilter = 22,
    GlobalLfo = 23,
    GlobalAmp = 24,
    GlobalAll = 25,
    GlobalConfig = 26
};

constexpr KnobBankMode knobBankForEditTarget(EditTarget target) noexcept {
    switch (target) {
    case EditTarget::Width:
        return KnobBankMode::PulseWidth;
    case EditTarget::Drift:
        return KnobBankMode::VoiceDrift;
    case EditTarget::Level:
    case EditTarget::Count:
        return KnobBankMode::Amplitude;
    }
    return KnobBankMode::Amplitude;
}

constexpr HudMode hudModeForEditTarget(EditTarget target) noexcept {
    switch (target) {
    case EditTarget::Width:
        return HudMode::PulseWidth;
    case EditTarget::Drift:
        return HudMode::VoiceDrift;
    case EditTarget::Level:
    case EditTarget::Count:
        return HudMode::Amplitude;
    }
    return HudMode::Amplitude;
}

constexpr bool isPerOscKnobBank(KnobBankMode bank) noexcept {
    return bank == KnobBankMode::Frequency || bank == KnobBankMode::PulseWidth ||
           bank == KnobBankMode::VoiceDrift || bank == KnobBankMode::Amplitude ||
           bank == KnobBankMode::Panning;
}

// Monochrome PWM LED rest brightness for White Keys 5-8 carbon-pill filter/LPG pads
inline constexpr float kPadFilterBaseLed = 0.24f;

struct HudState {
    HudMode mode = HudMode::None;
    KnobBankMode knobBank = KnobBankMode::Frequency;
    KnobRate knobRate = KnobRate::Standard;
    GlobalTarget globalTarget = GlobalTarget::All;
    EditTarget editTarget = EditTarget::Level;
    GlobalConfigState config{};
    ButtonId lastButton = ButtonId::Count;
    BlackKeyId lastBlackKey = BlackKeyId::Count;
    EncoderId lastEncoder = EncoderId::Select;
    uint8_t focusedVoice = 0u;            // 0..3 oscillator index
    uint8_t focusedKnob = kNoFocusedKnob; // 0..3 active column highlight, or kNoFocusedKnob
    uint8_t fxVoiceFocus = 0u;            // 0..3 = OSC 1..4 (SELECT encoder on FX Filter Page)
    uint8_t lfoVoiceFocus = 0u;           // 0..7 = LFO A/B OSC 1..4 (SELECT encoder on LFO Page)
    uint8_t configCursor = 0u;            // 0..kNumGlobalConfigItems-1 (SELECT in GLOBAL CONFIG)
    uint8_t snapshotSlot = 0u;            // 0..7 for White Keys 9..16
    float morphPosition = 0.0f;           // [0.0f, 1.0f] snapshot pressure morph position
    float opacity = 0.0f;                 // [0.0f, 1.0f] smooth 150 ms S-curve alpha envelope
    bool modifierHeld = false;
    bool rateHoldActive = false; // True whenever OCT- or OCT+ is physically held
    bool scopeFrozen = false;    // Momentary waveform freeze on 11th black key
    bool saveArmed = false;
    bool fxPageOpen = false;     // Latching MS-20 Filter page state (ButtonId::Fx)
    bool panPageOpen = false;    // Latching Stereo Panning page state (ButtonId::Sel)
    bool lfoPageOpen = false;     // Latching LFO page state (ButtonId::Lfo)
    bool editPageOpen = false;    // Latching Channel Level & Drive page (ButtonId::Edit)
    bool configPageOpen = false;  // Latching GLOBAL CONFIG menu (ButtonId::GloRight)
    bool panLockedNotice = false; // 1.5 s "MONO LOCKED" header feedback when turning PAN in MONO
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
