#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

struct GlobalColumnSummary {
    float posSigned = 0.0f;
    std::array<float, kNumOscillators> voiceSigned{0.0f, 0.0f, 0.0f, 0.0f};
    bool bipolar = false;
    bool hasVoiceSpread = false;
    bool lfoShapeMixed = false;
    bool lfoTargetMixed = false;
    float primaryValue = 0.0f;
};

constexpr HudMode hudModeForGlobalTarget(GlobalTarget target) noexcept {
    switch (target) {
    case GlobalTarget::Freq:
        return HudMode::GlobalFreq;
    case GlobalTarget::Filter:
        return HudMode::GlobalFilter;
    case GlobalTarget::Lfo:
        return HudMode::GlobalLfo;
    case GlobalTarget::Amp:
        return HudMode::GlobalAmp;
    case GlobalTarget::All:
    case GlobalTarget::Count:
        return HudMode::GlobalAll;
    }
    return HudMode::GlobalAll;
}

constexpr KnobBankMode knobBankForGlobalTarget(GlobalTarget target) noexcept {
    switch (target) {
    case GlobalTarget::Freq:
        return KnobBankMode::GlobalFreq;
    case GlobalTarget::Filter:
        return KnobBankMode::GlobalFilter;
    case GlobalTarget::Lfo:
        return KnobBankMode::GlobalLfo;
    case GlobalTarget::Amp:
        return KnobBankMode::GlobalAmp;
    case GlobalTarget::All:
    case GlobalTarget::Count:
        return KnobBankMode::GlobalAll;
    }
    return KnobBankMode::GlobalAll;
}

constexpr bool isGlobalHudMode(HudMode mode) noexcept {
    return mode == HudMode::GlobalFreq || mode == HudMode::GlobalFilter ||
           mode == HudMode::GlobalLfo || mode == HudMode::GlobalAmp || mode == HudMode::GlobalAll;
}

constexpr bool isGlobalKnobBank(KnobBankMode bank) noexcept {
    return bank == KnobBankMode::GlobalFreq || bank == KnobBankMode::GlobalFilter ||
           bank == KnobBankMode::GlobalLfo || bank == KnobBankMode::GlobalAmp ||
           bank == KnobBankMode::GlobalAll;
}

constexpr const char* globalTargetSuffix(GlobalTarget target) noexcept {
    switch (target) {
    case GlobalTarget::Freq:
        return " FREQ";
    case GlobalTarget::Filter:
        return " FILTER";
    case GlobalTarget::Lfo:
        return " LFO";
    case GlobalTarget::Amp:
        return " LEVEL";
    case GlobalTarget::All:
    case GlobalTarget::Count:
        return " ALL";
    }
    return " ALL";
}

// Apply 4-voice relative/orthogonal parameter deltas for any of the 5 GLOBAL destinations.
void applyGlobalKnobDelta(GlobalTarget target, uint8_t knobIdx, int32_t ticks, KnobRate rate,
                          int32_t& targetTickAccum, EngineControlState& state) noexcept;

// Evaluate normalized gauge position, per-voice spread beads, and acoustic readout value
// for column `knobIdx` (0..3) of any Global* KnobBankMode.
GlobalColumnSummary evaluateGlobalColumn(KnobBankMode bank, uint8_t knobIdx,
                                         const EngineControlState& control,
                                         const LfoSignalArray& lfoOutput) noexcept;

} // namespace ruido
