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
    Snapshot = 12,
    None = 13
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
    GlobalConfig = 26,
    KeyboardModel = 27, // Latching SEL Keyboard Model status & guidance screen
    KeyboardPlay = 28   // Momentary live punch-in routed 1-to-1 to native Ruido views
};

// Active momentary punch-in family displayed in the top header during HudMode::KeyboardPlay
enum class KeyPunchKind : uint8_t {
    None = 0,
    SpotlightSolo = 1,   // OP1-OP4: Solo OSC 1-4
    OverloadSurge = 2,   // LiveMixer OP5: Drive (4.5x MS-20 overdrive + rail coupling)
    SubDrop = 3,         // LiveMixer OP6: Sub Octave (-1 / -2 Oct)
    RatioMagnet = 4,     // LiveMixer PIT: Ratio Lock (Just Intonation)
    DubLpfDrop = 5,      // LiveMixer GLO: Low Pass sweep (28 kHz -> 31 Hz)
    MonoFocus = 6,       // LiveMixer MONO: 4-oscillator unison pitch collapse
    PolySpread = 7,      // LiveMixer POLY: Detuned + PWM swarm spread
    StasisFreeze = 8,    // 11th tall key: Freeze (8 LFOs + Drift + Scope)
    VoiceCut = 9,        // White Keys 1-4: Mute OSC 1-4
    VoiceFormant = 10,   // LiveMixer Keys 5-8: Filter OSC 1-4
    VoiceStutter = 11,   // RhythmLab: Stutter chopper gate OSC 1-4
    VoiceSiren = 12,     // LiveMixer Keys 9-12: Octave OSC 1-4 (+1 / +2 Oct)
    SnapshotRecall = 13, // LiveMixer Keys 13-16: Scene 1-4 recall / pressure morph
    VoiceXMod = 14,      // CircuitLab Keys 5-8: Cross-Pin neighbor FM/XOR into OSC 1-4
    VoiceFold = 15,      // CircuitLab Keys 9-12: 4x West-Coast capacitor wavefolder OSC 1-4
    RailStarve = 16,     // CircuitLab OP5: Starved 9V CMOS rail collapse
    RingXor = 17,        // CircuitLab OP6: 4x paired Ring/XOR sidebands
    LfoTurbo = 18,       // CircuitLab PIT: 1x -> 32x LFO overclock
    FilterScream = 19,   // CircuitLab GLO: Twin Korg35 HPF + LPF self-oscillation
    HardSync = 20,       // CircuitLab MONO: Band-limited OSC 1 -> 2..4 hard sync
    PhaseScatter = 21,   // CircuitLab POLY: Duty, pitch, filter & stereo scatter
    VoiceCrush = 22,     // CircuitLab Keys 13-16: CD4013 S&H clock crush + CD4040 ripple XOR
    VoiceDivide = 23,    // RhythmLab Keys 9-12: Master-clock-locked divider OSC 1-4 (/2..x8)
    VoiceEuclid = 24,    // RhythmLab Keys 13-16: 16-step Euclidean groove OSC 1-4 (2/16..16/16)
    RhythmPump = 25,     // RhythmLab OP5: 4-on-the-floor sidechain ducking pump
    RhythmChase = 26,    // RhythmLab OP6: 1->2->3->4 round-robin step loop
    RhythmTempo = 27,    // RhythmLab PIT: 1x -> 4x master clock speed
    RhythmPluck = 28,    // RhythmLab GLO: Percussive MS-20 low-pass filter pluck
    RhythmShort = 29,    // RhythmLab MONO: Pinpoint short gate duty
    RhythmSwing = 30,    // RhythmLab POLY: 50% -> 72% shuffle swing + stereo bounce
    Count = 31
};

struct PureRatioEntry {
    float ratio;
    uint8_t num;
    uint8_t den;
};

inline constexpr size_t kNumJustRatios = 19u;

inline constexpr std::array<PureRatioEntry, kNumJustRatios> kJustRatioTable = {{
    {0.2500000f, 1u, 4u}, {0.3333333f, 1u, 3u}, {0.3750000f, 3u, 8u}, {0.5000000f, 1u, 2u},
    {0.6250000f, 5u, 8u}, {0.6666667f, 2u, 3u}, {0.7500000f, 3u, 4u}, {0.8750000f, 7u, 8u},
    {1.0000000f, 1u, 1u}, {1.1250000f, 9u, 8u}, {1.2500000f, 5u, 4u}, {1.3333333f, 4u, 3u},
    {1.5000000f, 3u, 2u}, {1.6666667f, 5u, 3u}, {1.7500000f, 7u, 4u}, {2.0000000f, 2u, 1u},
    {2.5000000f, 5u, 2u}, {3.0000000f, 3u, 1u}, {4.0000000f, 4u, 1u},
}};

constexpr const char* keyboardModelTitle(KeyboardModel model) noexcept {
    switch (model) {
    case KeyboardModel::LiveMixer:
        return "LIVE MIXER";
    case KeyboardModel::CircuitLab:
        return "CIRCUIT LAB";
    case KeyboardModel::RhythmLab:
        return "RHYTHM LAB";
    case KeyboardModel::Count:
        break;
    }
    return "LIVE MIXER";
}

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
    uint8_t snapshotSlot = 0u;            // 0..3 active/selected Snapshot 1..4
    std::array<float, kNumSnapshots> snapshotWeights{1.0f, 0.0f, 0.0f, 0.0f};
    KeyboardModel keyboardModel = KeyboardModel::LiveMixer;
    KeyPunchKind activePunch = KeyPunchKind::None;
    uint8_t activePunchVoice = 0u; // 0..3 voice/slot index
    float activePunchValue = 0.0f; // Live acoustic readout (pressure 0..1, Hz, or octave/ratio)
    uint16_t heldBlackMask = 0u;   // Bitmask of held raised keys (bits 0..10)
    uint16_t heldWhiteMask = 0u;   // Bitmask of held bottom keys (bits 0..15)
    std::array<uint8_t, kNumBlackKeys> blackPressureQ8{};
    std::array<uint8_t, kNumWhiteKeys> whitePressureQ8{};
    float morphPosition = 1.0f;       // [0.0f, 1.0f] snapshot glide/pressure morph progress
    float opacity = 0.0f;             // [0.0f, 1.0f] smooth 150 ms S-curve alpha envelope
    float fxLedSync01 = 0.5f;         // [0.0f, 1.0f] Core 0 FX button breath/flare sync
    float lfoLedSync01 = 0.5f;        // [0.0f, 1.0f] Core 0 LFO button wave sync
    float effectiveClockBpm = 120.0f; // Live master clock BPM (internal 120 or synced MIDI BPM)
    bool midiClockSynced = false;     // True when locked to external 24-PPQN MIDI Clock
    bool modifierHeld = false;
    bool rateHoldActive = false; // True whenever OCT- or OCT+ is physically held
    bool scopeFrozen = false;    // Momentary waveform freeze on 11th black key
    bool saveArmed = false;
    bool snapshotDirty = false;      // True when knobs diverged from the recalled snapshot
    bool snapshotSavedToast = false; // True right after SAVE + Key 13-16 stores a slot
    bool snapshotGliding = false;    // True while actively gliding/morphing into a snapshot
    bool freqPageOpen = false;       // Latching Frequency HUD toggle state (ButtonId::Home)
    bool fxPageOpen = false;         // Latching MS-20 Filter page state (ButtonId::Fx)
    bool panPageOpen = false;    // Latching EDIT Page 2 Stereo Panning state (ButtonId::Edit x2)
    bool lfoPageOpen = false;    // Latching LFO page state (ButtonId::Lfo)
    bool editPageOpen = false;   // Latching EDIT Page 1 Channel Mix state (ButtonId::Edit x1)
    bool configPageOpen = false; // Latching GLOBAL CONFIG menu (ButtonId::GloRight)
    bool selPageOpen = false;    // Latching KEYBOARD MODEL status & guidance page (ButtonId::Sel)
};

constexpr uint8_t resolveFilterViewVoice(const HudState& hud) noexcept {
    if (hud.mode == HudMode::KeyboardPlay && hud.activePunch == KeyPunchKind::VoiceFormant) {
        return hud.activePunchVoice & 3u;
    }
    return (hud.mode == HudMode::VoicePressure) ? (hud.focusedVoice & 3u) : (hud.fxVoiceFocus & 3u);
}

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
static_assert(__is_trivially_copyable(HudState), "HudState must remain trivially copyable");
static_assert(__is_trivially_copyable(LedState), "LedState must remain trivially copyable");
static_assert(sizeof(LedState) == kNumLedChannels * sizeof(float),
              "LedState must pack contiguously as 42 floats for Wasm/DMA transport");

} // namespace ruido
