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

// Physical 27-key hardware split on the M-VAVE FM-1 PCB (`kFm1KeyIdCoord` in
// `fm1_led_modulator.h`):
//   - Left 20 Keys (`PA8` + `PA7` SARADC carbon-pill pressure):
//       Top 8 Black Keys:    `0..7`  (`OP1..OP6, PIT, GLO` in `3 · 2 · 3` clusters)
//       Bottom 12 White Keys:`0..11` (`Bank A 0..3` | `Bank B 4..7` | `Bank C 8..11`,
//       +`SHIFT=SCENE`)
//   - Right 7 Keys (`PA6` digital-only GPIO — permanent utility switches across all models):
//       Top 3 Black Keys:    `8` = `HOLD` (`MONO`), `9` = `TAP TEMPO` (`POLY`), `10` = `SHIFT`
//       (`11th`) Bottom 4 White Keys: `12..15` = `MUTE 1..4` (unshifted) / `SOLO 1..4` (`SHIFT`
//       held)
inline constexpr size_t kNumTopMacroKeys = 8u;   // Black 0..7 (OP1..GLO analog pressure macros)
inline constexpr size_t kHoldBlackKeyIdx = 8u;   // Black 8 (MONO): Key Hold latch switch
inline constexpr size_t kTapBlackKeyIdx = 9u;    // Black 9 (POLY): Tap Tempo switch
inline constexpr size_t kShiftBlackKeyIdx = 10u; // Black 10 (11th): Universal SHIFT modifier
inline constexpr size_t kBankAWhiteKeyBase = 0u; // White 0..3 (W01..W04): FILTER / CROSS / STUTTER
inline constexpr size_t kBankBWhiteKeyBase = 4u; // White 4..7 (W05..W08): OCTAVE / FOLD / DIVIDE
inline constexpr size_t kSceneWhiteKeyBase =
    8u; // White 8..11 (W09..W12): DUTY / CRUSH / EUCLID (+SHIFT SCENE)
inline constexpr size_t kMuteWhiteKeyBase = 12u; // White 12..15 (W13..W16): MUTE 1..4 (+SHIFT SOLO)

constexpr bool isAnalogPressureBlackKey(size_t blackIdx) noexcept {
    return blackIdx < kNumTopMacroKeys;
}

constexpr bool isAnalogPressureWhiteKey(size_t whiteIdx) noexcept {
    return whiteIdx < kMuteWhiteKeyBase;
}

// 3 per-oscillator modes cycled via HOME presses (returning to Clean Scope on 4th press)
// or SELECT on the HOME page (signal-flow order: Frequency -> Duty -> Drift)
enum class HomeTarget : uint8_t { Frequency = 0, Width = 1, Drift = 2, Count = 3 };

// Active mapping of top-right KNOB1..KNOB4 (displayed in the 4 bottom HUD columns unless None)
enum class KnobBankMode : uint8_t {
    Frequency = 0,
    PulseWidth = 1,
    VoiceDrift = 2,
    VoiceFilter = 3,
    Amplitude = 4,
    Panning = 5,
    Lfo = 6,
    GlobalFilter = 7,
    None = 8
};

// Encoder sweep rate selected via OCT- (Fine) and OCT+ (Coarse: ~1.3 turns full sweep)
enum class KnobRate : int8_t { Fine = -1, Standard = 0, Coarse = 1 };

// Notched CRT oscilloscope horizontal timebase selected via SELECT encoder when idle
enum class ScopeTimebase : uint8_t {
    Zoom4x = 0, // 0.25 samples/px (~1.4 ms window: 4x horizontal detail)
    Zoom2x = 1, // 0.50 samples/px (~2.7 ms window: 2x horizontal detail)
    Auto = 2,   // Boot default: auto 1.0 (5.4 ms) <-> 2.0 (11 ms) bass hysteresis
    Wide2x = 3, // 2.00 samples/px (~11 ms fixed window)
    Wide4x = 4, // 4.00 samples/px (~22 ms wide bass & chop window)
    Wide8x = 5, // 8.00 samples/px (~44 ms sub-bass & rhythm envelope window)
    Count = 6
};

constexpr const char* scopeTimebaseLabel(ScopeTimebase tb) noexcept {
    switch (tb) {
    case ScopeTimebase::Zoom4x:
        return "4X";
    case ScopeTimebase::Zoom2x:
        return "2X";
    case ScopeTimebase::Auto:
        return "AUTO";
    case ScopeTimebase::Wide2x:
        return "1/2X";
    case ScopeTimebase::Wide4x:
        return "1/4X";
    case ScopeTimebase::Wide8x:
        return "1/8X";
    case ScopeTimebase::Count:
        break;
    }
    return "AUTO";
}

constexpr float scopeTimebaseNorm01(ScopeTimebase tb) noexcept {
    const uint8_t idx = (tb < ScopeTimebase::Count) ? static_cast<uint8_t>(tb) : 2u;
    return static_cast<float>(idx) * 0.2f;
}

inline constexpr uint8_t kPhantomButtonBase = 0u;
inline constexpr uint8_t kPhantomBlackKeyBase = 14u;
inline constexpr uint8_t kPhantomWhiteKeyBase = 25u;
inline constexpr uint8_t kNumPhantomChannels = 41u;

inline constexpr std::array<ButtonId, 4> kPhantomPinButtons = {
    ButtonId::Env,
    ButtonId::Arp,
    ButtonId::Seq,
    ButtonId::Rec,
};

inline constexpr std::array<ButtonId, kNumPanelButtons> kPhantomButtonOrder = {
    ButtonId::Env,     ButtonId::Arp,  ButtonId::Seq,  ButtonId::Rec,      ButtonId::OctMinus,
    ButtonId::OctPlus, ButtonId::Fx,   ButtonId::Sel,  ButtonId::Lfo,      ButtonId::GloRight,
    ButtonId::Home,    ButtonId::Save, ButtonId::Edit, ButtonId::PlayStop,
};

constexpr uint8_t phantomPinSlot(ButtonId button) noexcept {
    for (uint8_t i = 0u; i < kPhantomButtonOrder.size(); ++i) {
        if (kPhantomButtonOrder[i] == button) {
            return i;
        }
    }
    return 0u;
}

constexpr uint8_t phantomBlackKeyChannel(size_t bIdx) noexcept {
    return static_cast<uint8_t>(kPhantomBlackKeyBase + (bIdx < kNumBlackKeys ? bIdx : 0u));
}

constexpr uint8_t phantomWhiteKeyChannel(size_t wIdx) noexcept {
    return static_cast<uint8_t>(kPhantomWhiteKeyBase + (wIdx < kNumWhiteKeys ? wIdx : 0u));
}

// Active SHIFT-layer encoder/key focus while 11th corner key (SHIFT) is held
enum class ShiftFocus : uint8_t {
    Idle = 0,
    Tempo = 1, // SHIFT + ALGORITHM (Master Tempo)
    Scene = 2, // SHIFT + PRESETS (Scene Morph)
    Model = 3, // SHIFT + SELECT or SHIFT + OP1..OP3 (Keyboard Model select)
};

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
    ScopeTimebase = 9,
    KnobRate = 10,
    VoiceGate = 11,
    VoicePressure = 12,
    SnapshotSlot = 13,
    OutputRouting = 14,
    AudioOutput = 15,
    SaveArmed = 16,
    ScopeFreeze = 17,
    VoiceLfo = 18,
    GlobalConfig = 19,
    KeyboardModel = 20, // Latching SEL Keyboard Model status & guidance screen
    KeyboardPlay = 21,  // Momentary live punch-in routed 1-to-1 to native Ruido views
    MasterTempo = 22,   // SHIFT + ALGORITHM / TAP TEMPO Master Tempo control
    KeyHold = 23,       // MONO switch Key Hold latch / release toast
    ShiftMap = 24,      // Universal Full-Chassis SHIFT Map overlay while 11th key is held
    Count = 25
};

// Active momentary punch-in family displayed in the top header during HudMode::KeyboardPlay
enum class KeyPunchKind : uint8_t {
    None = 0,
    SpotlightSolo = 1,   // SHIFT + Keys 13-16: Solo OSC 1-4
    OverloadSurge = 2,   // LiveMixer OP1: Drive (4.5x MS-20 overdrive + rail coupling)
    SubDrop = 3,         // LiveMixer OP2: Sub Octave (-1 / -2 Oct)
    RatioMagnet = 4,     // LiveMixer OP3: Ratio Lock (Just Intonation)
    DubLpfDrop = 5,      // LiveMixer OP4: Low Pass sweep (28 kHz -> 31 Hz)
    MonoFocus = 6,       // LiveMixer OP6: 4-oscillator unison pitch collapse
    PolySpread = 7,      // LiveMixer GLO: Wide stereo + PWM swarm spread
    StasisFreeze = 8,    // Legacy freeze slot
    VoiceCut = 9,        // Keys 13-16: Mute OSC 1-4
    VoiceFormant = 10,   // LiveMixer Keys 1-4: Filter OSC 1-4
    VoiceStutter = 11,   // RhythmLab Keys 1-4: Stutter chopper gate OSC 1-4
    VoiceSiren = 12,     // LiveMixer Keys 5-8: Octave OSC 1-4 (+1 / +2 Oct)
    SnapshotRecall = 13, // SHIFT + Keys 9-12: Scene 1-4 recall / pressure morph
    VoiceXMod = 14,      // CircuitLab Keys 1-4: Cross-Pin neighbor FM/XOR into OSC 1-4
    VoiceFold = 15,      // CircuitLab Keys 5-8: 4x West-Coast capacitor wavefolder OSC 1-4
    RailStarve = 16,     // CircuitLab OP1: Starved 9V CMOS rail collapse
    RingXor = 17,        // CircuitLab OP2: 4x paired Ring/XOR sidebands
    LfoTurbo = 18,       // CircuitLab OP3: 1x -> 32x LFO overclock
    FilterScream = 19,   // CircuitLab OP4: Twin Korg35 HPF + LPF self-oscillation
    HardSync = 20,       // CircuitLab OP5: Band-limited OSC 1 -> 2..4 hard sync
    PhaseScatter = 21,   // CircuitLab OP6: Duty, pitch, filter & stereo scatter
    VoiceCrush = 22,     // CircuitLab Keys 9-12: CD4013 S&H clock crush + CD4040 ripple XOR
    VoiceDivide = 23,    // RhythmLab Keys 5-8: Master-clock-locked divider OSC 1-4 (/2..x8)
    VoiceEuclid = 24,    // RhythmLab Keys 9-12: 16-step Euclidean groove OSC 1-4 (2/16..16/16)
    RhythmPump = 25,     // RhythmLab OP1: 4-on-the-floor sidechain ducking pump
    RhythmChase = 26,    // RhythmLab OP2: 1->2->3->4 round-robin step loop
    RhythmTempo = 27,    // RhythmLab OP3: 1x -> 4x master clock speed
    RhythmPluck = 28,    // RhythmLab OP4: Percussive MS-20 low-pass filter pluck
    RhythmShort = 29,    // RhythmLab OP5: Pinpoint short gate duty
    RhythmSwing = 30,    // RhythmLab OP6: 0-100% pure 16th-note shuffle groove
    TapTempo = 31,       // POLY switch: Universal Tap Tempo & Beat-1 downbeat sync
    HighPass = 32,       // LiveMixer OP5: Master resonant MS-20 HPF bass-cut sweep
    DetuneChorus = 33,   // LiveMixer PIT: 4-voice analog detune chorus
    VoiceDuty = 34,      // LiveMixer Keys 9-12: Per-voice Schmitt PWM duty sweep OSC 1-4
    SubXor = 35,         // CircuitLab PIT: Octave sub-divider + XOR sideband ring
    FeedbackRoar = 36,   // CircuitLab GLO: Cross-coupled rail + resonant filter feedback
    RhythmPingPong = 37, // RhythmLab PIT: Alternating L/R stereo gate bounce
    RhythmRoll = 38,     // RhythmLab GLO: Momentary x2..x8 fast ratchet fill
    KeyHoldSwitch = 39,  // MONO switch: LATCH pad capture / release inspection
    Count = 40
};

struct PureRatioEntry {
    float ratio;
    float log2Ratio;
    uint8_t num;
    uint8_t den;
};

inline constexpr size_t kNumJustRatios = 19u;

inline constexpr std::array<PureRatioEntry, kNumJustRatios> kJustRatioTable = {{
    {0.2500000f, -2.0000000f, 1u, 4u}, {0.3333333f, -1.5849625f, 1u, 3u},
    {0.3750000f, -1.4150375f, 3u, 8u}, {0.5000000f, -1.0000000f, 1u, 2u},
    {0.6250000f, -0.6780719f, 5u, 8u}, {0.6666667f, -0.5849625f, 2u, 3u},
    {0.7500000f, -0.4150375f, 3u, 4u}, {0.8750000f, -0.1926451f, 7u, 8u},
    {1.0000000f, 0.0000000f, 1u, 1u},  {1.1250000f, 0.1699250f, 9u, 8u},
    {1.2500000f, 0.3219281f, 5u, 4u},  {1.3333333f, 0.4150375f, 4u, 3u},
    {1.5000000f, 0.5849625f, 3u, 2u},  {1.6666667f, 0.7369656f, 5u, 3u},
    {1.7500000f, 0.8073549f, 7u, 4u},  {2.0000000f, 1.0000000f, 2u, 1u},
    {2.5000000f, 1.3219281f, 5u, 2u},  {3.0000000f, 1.5849625f, 3u, 1u},
    {4.0000000f, 2.0000000f, 4u, 1u},
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

constexpr KnobBankMode knobBankForHomeTarget(HomeTarget target) noexcept {
    switch (target) {
    case HomeTarget::Width:
        return KnobBankMode::PulseWidth;
    case HomeTarget::Drift:
        return KnobBankMode::VoiceDrift;
    case HomeTarget::Frequency:
    case HomeTarget::Count:
        return KnobBankMode::Frequency;
    }
    return KnobBankMode::Frequency;
}

constexpr HudMode hudModeForHomeTarget(HomeTarget target) noexcept {
    switch (target) {
    case HomeTarget::Width:
        return HudMode::PulseWidth;
    case HomeTarget::Drift:
        return HudMode::VoiceDrift;
    case HomeTarget::Frequency:
    case HomeTarget::Count:
        return HudMode::VoiceFrequency;
    }
    return HudMode::VoiceFrequency;
}

constexpr bool isPerOscKnobBank(KnobBankMode bank) noexcept {
    return bank == KnobBankMode::Frequency || bank == KnobBankMode::PulseWidth ||
           bank == KnobBankMode::VoiceDrift || bank == KnobBankMode::Amplitude ||
           bank == KnobBankMode::Panning;
}

enum class SavePageMode : uint8_t {
    Store = 0,
    Delete = 1,
    Init = 2,
    Count = 3,
};

enum class SnapshotToastKind : uint8_t {
    None = 0,
    Saved = 1,
    Initialized = 2,
    Deleted = 3,
};

enum class CapsuleToastKind : uint8_t {
    None = 0,
    LatchOn = 1,
    LatchOff = 2,
    Fine = 3,
    Coarse = 4,
};

struct HudState {
    GlobalConfigState config{};
    std::array<float, kNumSnapshots> snapshotWeights{1.0f, 0.0f, 0.0f, 0.0f};
    float activePunchValue = 0.0f;    // Live acoustic readout (pressure 0..1, Hz, or octave/ratio)
    float morphPosition = 1.0f;       // [0.0f, 1.0f] snapshot glide/pressure morph progress
    float snapshotLoopPhase = 0.0f;   // [0.0f, 4.0f) continuous circular position across S1..S4..S1
    float opacity = 0.0f;             // [0.0f, 1.0f] smooth 150 ms S-curve alpha envelope
    float effectiveClockBpm = 120.0f; // Live master clock BPM (internal 120 or synced MIDI BPM)
    uint16_t heldBlackMask = 0u;      // Bitmask of physically held raised keys (bits 0..10)
    uint16_t heldWhiteMask = 0u;      // Bitmask of physically held bottom keys (bits 0..15)
    uint16_t latchedBlackMask = 0u;   // Bitmask of raised keys latched by HOLD (bits 0..7)
    uint16_t latchedWhiteMask = 0u;   // Bitmask of bottom keys latched by HOLD (bits 0..11)
    std::array<uint8_t, kNumBlackKeys> blackPressureQ8{};
    std::array<uint8_t, kNumWhiteKeys> whitePressureQ8{};
    HudMode mode = HudMode::None;
    KnobBankMode knobBank = KnobBankMode::Frequency;
    KnobRate knobRate = KnobRate::Standard;
    HomeTarget homeTarget = HomeTarget::Frequency;
    ButtonId lastButton = ButtonId::Count;
    BlackKeyId lastBlackKey = BlackKeyId::Count;
    EncoderId lastEncoder = EncoderId::Select;
    uint8_t focusedVoice = 0u;            // 0..3 oscillator index
    uint8_t focusedKnob = kNoFocusedKnob; // 0..3 latest knob for Top-Bar readout, or kNoFocusedKnob
    uint8_t focusedColumnMask = 0u;       // Bits 0..3 = multi-knob / per-voice column spotlight
    uint8_t fxVoiceFocus = 0u;            // 0..3 = OSC 1..4 (SELECT encoder on FX Filter Page)
    uint8_t lfoVoiceFocus = 0u;           // 0..7 = LFO A/B OSC 1..4 (SELECT encoder on LFO Page)
    uint8_t configCursor = 0u;            // 0..kNumGlobalConfigItems-1 (SELECT in GLOBAL CONFIG)
    ConfigSubView configSubView = ConfigSubView::List; // Operational sub-screen in GLOBAL CONFIG
    uint8_t configResetTicks = 0u;        // 0..24 CW confirmation ticks in FactoryResetConfirm
    uint8_t snapshotSlot = 0u;            // 0..3 active/selected Snapshot 1..4
    uint8_t snapshotToastSlot = 0u;       // 0..3 target slot for SAVE / DELETE / EMPTY toast
    uint8_t snapshotOccupiedMask = 0x0Fu; // Bits 0..3 = Scene 1..4 occupied (0 = deleted/EMPTY)
    KeyboardModel keyboardModel = KeyboardModel::LiveMixer;
    KeyPunchKind activePunch = KeyPunchKind::None;
    ShiftFocus shiftFocus = ShiftFocus::Idle;
    uint8_t activePunchVoice = 0u; // 0..3 voice/slot index
    bool midiClockSynced = false;  // True when locked to external 24-PPQN MIDI Clock
    bool modifierHeld = false;
    bool rateHoldActive = false; // True whenever OCT- or OCT+ is physically held
    bool shiftHeld = false;      // True whenever 11th corner key (SHIFT) is physically held
    uint8_t modeWipeQ8 = 0u;     // [0..255] hardware-synced 220 ms keyboard-change LED wave phase
    bool keyHoldLatched = false; // True whenever MONO (LATCH) key-latch switch is engaged
    uint8_t keyHoldCount = 0u;   // Count of expressive pads currently captured by LATCH
    uint8_t shiftBlinkQ8 = 255u; // [0..255] hardware-synced 0.40 Hz slow blink for active OP1..3
    bool scopeFrozen = false;    // Legacy scope freeze flag
    bool saveArmed = false;
    SavePageMode savePageMode = SavePageMode::Store; // Active SAVE page (STORE / INIT / DELETE)
    bool snapshotDirty = false;                      // True when knobs diverged from recalled scene
    SnapshotToastKind snapshotToast = SnapshotToastKind::None; // Result toast after SAVE/INIT/DEL
    bool snapshotGliding = false; // True while actively gliding/morphing into a snapshot
    bool freqPageOpen = false;    // Latching Frequency HUD toggle state (ButtonId::Home)
    bool fxPageOpen = false;      // Latching MS-20 Filter page state (ButtonId::Fx)
    bool panPageOpen = false;     // Latching EDIT Stereo Panning sub-page state (ButtonId::Edit)
    bool lfoPageOpen = false;     // Latching LFO page state (ButtonId::Lfo)
    bool editPageOpen = false;    // Latching EDIT Channel Mix sub-page state (ButtonId::Edit)
    bool configPageOpen = false;  // Latching GLOBAL CONFIG menu (ButtonId::GloRight)
    bool selPageOpen = false;     // Latching KEYBOARD MODEL status & guidance page (ButtonId::Sel)
    uint8_t selWakeSeq = 0u;      // Incremented on SEL open or SELECT/ALGORITHM knob interaction
    ScopeTimebase scopeTimebase = ScopeTimebase::Auto; // CRT horizontal timebase (idle SELECT)
    uint8_t phantomPinSeq = 0u;   // Incremented when an unconnected switch (ENV/ARP/SEQ/REC) fires
    uint8_t phantomPinIndex = 0u; // 0 = ENV, 1 = ARP, 2 = SEQ, 3 = REC
    uint8_t savePulseQ8 = 128u;   // [0..255] hardware-synced SAVE / LATCH LED pulse phase
    CapsuleToastKind capsuleToast = CapsuleToastKind::None; // Momentary center stadium capsule
    uint8_t capsuleAlphaQ8 = 0u; // [0..255] 150 ms fade-in -> 1.0 s hold -> 150 ms fade-out
};

constexpr bool hasSnapshotResultToast(const HudState& hud) noexcept {
    return hud.snapshotToast != SnapshotToastKind::None;
}

constexpr bool hasNonHomePageOpen(const HudState& hud) noexcept {
    return hud.fxPageOpen || hud.panPageOpen || hud.lfoPageOpen || hud.editPageOpen ||
           hud.configPageOpen || hud.selPageOpen;
}

constexpr bool hasPinnedPageOpen(const HudState& hud) noexcept {
    return hud.freqPageOpen || hasNonHomePageOpen(hud);
}

constexpr uint8_t effectiveFocusedColumnMask(const HudState& hud) noexcept {
    const uint8_t singleBit =
        (hud.focusedKnob < kNumOscillators) ? static_cast<uint8_t>(1u << hud.focusedKnob) : 0u;
    return static_cast<uint8_t>((hud.focusedColumnMask | singleBit) & 0x0Fu);
}

constexpr bool isHudColumnFocused(const HudState& hud, size_t col) noexcept {
    return col < kNumOscillators && ((effectiveFocusedColumnMask(hud) >> col) & 1u) != 0u;
}

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
