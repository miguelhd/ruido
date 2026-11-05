#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

enum class PunchBarKind : uint8_t {
    None = 0,
    Unit01 = 1,        // Normalized [0, 1] directly from activePunchValue
    NegTwoOct = 2,     // [-2.0, 0.0] octave drop mapped to [0, 1]
    PosTwoOct = 3,     // [0.0, +2.0] octave rise mapped to [0, 1]
    SnapshotGlide = 4, // Active only while hud.snapshotGliding is true (morphPosition)
};

enum class PunchReadoutKind : uint8_t {
    ModelPreview = 0,
    Percent = 1,
    SoloState = 2,
    MuteState = 3,
    Holding = 4,
    SubOctave = 5,
    PosOctave = 6,
    JustRatio = 7,
    RawCutoffHz = 8,
    StutterHz = 9,
    FoldStages = 10,
    CrushHz = 11,
    TurboMult = 12,
    ClockDiv = 13,
    EuclidHits = 14,
    TempoBpm = 15,
    SwingPct = 16,
    Snapshot = 17,
};

// Contiguous position-independent ROM descriptor (20 B/row, zero pointer relocations in XIP Flash)
struct KeyPunchSpec {
    KeyPunchKind kind;
    KnobBankMode knobBank;
    PunchBarKind barKind;
    PunchReadoutKind readoutKind;
    bool showOscSuffix;
    char title[14];
};

inline constexpr size_t kNumKeyPunchKinds = static_cast<size_t>(KeyPunchKind::Count);

inline constexpr std::array<KeyPunchSpec, kNumKeyPunchKinds> kKeyPunchSpecs = {{
    {KeyPunchKind::None, KnobBankMode::None, PunchBarKind::None, PunchReadoutKind::ModelPreview,
     false, ""},
    {KeyPunchKind::SpotlightSolo, KnobBankMode::Frequency, PunchBarKind::None,
     PunchReadoutKind::SoloState, true, "SOLO"},
    {KeyPunchKind::OverloadSurge, KnobBankMode::Amplitude, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "DRIVE"},
    {KeyPunchKind::SubDrop, KnobBankMode::Frequency, PunchBarKind::NegTwoOct,
     PunchReadoutKind::SubOctave, false, "SUB OCTAVE"},
    {KeyPunchKind::RatioMagnet, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::JustRatio, false, "RATIO LOCK"},
    {KeyPunchKind::DubLpfDrop, KnobBankMode::GlobalFilter, PunchBarKind::None,
     PunchReadoutKind::RawCutoffHz, false, "LOW PASS"},
    {KeyPunchKind::MonoFocus, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "UNISON"},
    {KeyPunchKind::PolySpread, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "DETUNE SPREAD"},
    {KeyPunchKind::StasisFreeze, KnobBankMode::Frequency, PunchBarKind::None,
     PunchReadoutKind::Holding, false, "FREEZE"},
    {KeyPunchKind::VoiceCut, KnobBankMode::Frequency, PunchBarKind::None,
     PunchReadoutKind::MuteState, true, "MUTE"},
    {KeyPunchKind::VoiceFormant, KnobBankMode::VoiceFilter, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, true, "FILTER"},
    {KeyPunchKind::VoiceStutter, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::StutterHz, true, "STUTTER"},
    {KeyPunchKind::VoiceSiren, KnobBankMode::Frequency, PunchBarKind::PosTwoOct,
     PunchReadoutKind::PosOctave, true, "OCTAVE"},
    {KeyPunchKind::SnapshotRecall, KnobBankMode::Snapshot, PunchBarKind::SnapshotGlide,
     PunchReadoutKind::Snapshot, false, ""},
    {KeyPunchKind::VoiceXMod, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, true, "CROSS-PIN"},
    {KeyPunchKind::VoiceFold, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::FoldStages, true, "FOLD"},
    {KeyPunchKind::RailStarve, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "STARVE"},
    {KeyPunchKind::RingXor, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "RING MOD"},
    {KeyPunchKind::LfoTurbo, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::TurboMult, false, "TURBO LFO"},
    {KeyPunchKind::FilterScream, KnobBankMode::GlobalFilter, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "SCREAM"},
    {KeyPunchKind::HardSync, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "HARD SYNC"},
    {KeyPunchKind::PhaseScatter, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "SCATTER"},
    {KeyPunchKind::VoiceCrush, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::CrushHz, true, "CRUSH"},
    {KeyPunchKind::VoiceDivide, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::ClockDiv, true, "DIVIDE"},
    {KeyPunchKind::VoiceEuclid, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::EuclidHits, true, "EUCLID"},
    {KeyPunchKind::RhythmPump, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "PUMP"},
    {KeyPunchKind::RhythmChase, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "CHASE"},
    {KeyPunchKind::RhythmTempo, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::TempoBpm, false, "TEMPO"},
    {KeyPunchKind::RhythmPluck, KnobBankMode::GlobalFilter, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "PLUCK"},
    {KeyPunchKind::RhythmShort, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::Percent, false, "SHORT"},
    {KeyPunchKind::RhythmSwing, KnobBankMode::Frequency, PunchBarKind::Unit01,
     PunchReadoutKind::SwingPct, false, "SWING"},
}};

constexpr bool verifyKeyPunchSpecsAligned() noexcept {
    for (size_t i = 0u; i < kNumKeyPunchKinds; ++i) {
        if (kKeyPunchSpecs[i].kind != static_cast<KeyPunchKind>(i)) {
            return false;
        }
    }
    return true;
}
static_assert(verifyKeyPunchSpecsAligned(),
              "kKeyPunchSpecs row order must match KeyPunchKind enum values 1-to-1");

constexpr const KeyPunchSpec& keyPunchSpec(KeyPunchKind punch) noexcept {
    const auto idx = static_cast<size_t>(punch);
    return (idx < kNumKeyPunchKinds) ? kKeyPunchSpecs[idx] : kKeyPunchSpecs[0];
}

constexpr KnobBankMode knobBankForKeyPunch(KeyPunchKind punch, KnobBankMode fallback) noexcept {
    const KeyPunchSpec& spec = keyPunchSpec(punch);
    return (spec.kind == KeyPunchKind::None) ? fallback : spec.knobBank;
}

// Single source of truth for normalized [0, 1] top-header bar gauges across all keyboard models.
constexpr bool evalKeyPunchHeaderBar(const HudState& hud, float& outNorm) noexcept {
    outNorm = 0.0f;
    switch (keyPunchSpec(hud.activePunch).barKind) {
    case PunchBarKind::None:
        return false;
    case PunchBarKind::Unit01:
        outNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        return true;
    case PunchBarKind::NegTwoOct:
        outNorm = clampf(-hud.activePunchValue * 0.5f, 0.0f, 1.0f);
        return true;
    case PunchBarKind::PosTwoOct:
        outNorm = clampf(hud.activePunchValue * 0.5f, 0.0f, 1.0f);
        return true;
    case PunchBarKind::SnapshotGlide:
        if (!hud.snapshotGliding) {
            return false;
        }
        outNorm = clampf(hud.morphPosition, 0.0f, 1.0f);
        return true;
    }
    return false;
}

} // namespace ruido
