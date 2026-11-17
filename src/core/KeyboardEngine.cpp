#include "core/KeyboardEngine.h"

#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "core/RhythmMath.h"

namespace ruido {

namespace {

constexpr float kModeWipeDurationSec = 0.220f;
constexpr uint16_t kAnalogWhiteExprMask = 0x0FFFu;    // White 0..11
constexpr uint16_t kAnalogBlackExprMask = 0x00FFu;    // Black 0..7 (OP1..GLO)
constexpr uint16_t kRepressLatchThresholdQ16 = 6554u; // ~10% squeeze to re-latch already-held pad

inline void beginLatchedPress(uint16_t bit, uint16_t qVal, uint16_t& ioLatchedMask,
                              uint16_t& ioRepressMask, uint16_t& ioLatchedQ16) noexcept {
    if ((ioLatchedMask & bit) != 0u && qVal < kRepressLatchThresholdQ16) {
        ioRepressMask |= bit;
    } else {
        ioLatchedMask |= bit;
        ioRepressMask &= static_cast<uint16_t>(~bit);
        ioLatchedQ16 = qVal;
    }
}

inline void trackLatchedPeak(uint16_t bit, uint16_t qVal, uint16_t& ioLatchedMask,
                             uint16_t& ioRepressMask, uint16_t& ioLatchedQ16) noexcept {
    ioLatchedMask |= bit;
    if ((ioRepressMask & bit) != 0u) {
        if (qVal >= kRepressLatchThresholdQ16) {
            ioRepressMask &= static_cast<uint16_t>(~bit);
            ioLatchedQ16 = qVal;
        }
    } else if (qVal > ioLatchedQ16) {
        ioLatchedQ16 = qVal;
    }
}

inline uint16_t quantizePressureU16(float p) noexcept {
    return static_cast<uint16_t>(clampf(p, 0.0f, 1.0f) * 65535.0f + 0.5f);
}

inline float dequantizePressureU16(uint16_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 65535.0f);
}

inline uint8_t quantizeU16ToU8(uint16_t q) noexcept {
    return static_cast<uint8_t>((static_cast<uint32_t>(q) * 255u + 32767u) / 65535u);
}

inline uint8_t popcount16(uint16_t v) noexcept {
    uint8_t c = 0u;
    while (v != 0u) {
        v &= static_cast<uint16_t>(v - 1u);
        ++c;
    }
    return c;
}

constexpr std::array<KeyboardModelOps, static_cast<size_t>(KeyboardModel::Count)> kModelOpsTable = {
    {
        {&evaluateLiveMixerModel, &choreographLiveMixerLeds},
        {&evaluateCircuitLabModel, &choreographCircuitLabLeds},
        {&evaluateRhythmLabModel, &choreographRhythmLabLeds},
    }};

} // namespace

const KeyboardModelOps& keyboardModelOps(KeyboardModel model) noexcept {
    const auto idx = static_cast<size_t>(model);
    return (idx < kModelOpsTable.size()) ? kModelOpsTable[idx] : kModelOpsTable[0];
}

void KeyboardEngine::clearKeyHold() noexcept {
    holdActive_ = false;
    latchedBlackMask_ = 0u;
    latchedWhiteMask_ = 0u;
    repressBlackMask_ = 0u;
    repressWhiteMask_ = 0u;
    latchedBlackPressureQ16_.fill(0u);
    latchedWhitePressureQ16_.fill(0u);
}

void KeyboardEngine::reset() noexcept {
    model_ = KeyboardModel::LiveMixer;
    blackPressureQ16_.fill(0u);
    whitePressureQ16_.fill(0u);
    blackPressSeq_.fill(0u);
    whitePressSeq_.fill(0u);
    clearKeyHold();
    heldBlackMask_ = 0u;
    heldWhiteMask_ = 0u;
    shiftSwallowedBlackMask_ = 0u;
    shiftSwallowedWhiteMask_ = 0u;
    pressCounter_ = 0u;
    shiftModelSeq_ = 0u;
    unassignedChannelEvent_ = -1;
    shiftSceneMask_ = 0u;
    shiftSoloHeldMask_ = 0u;
    shiftModelEvent_ = -1;
    holdToggledEvent_ = false;
    tapTempoEvent_ = false;
    latch_ = {};
    config_ = {};
    wipePhase_ = 0.0f;
}

void KeyboardEngine::setModel(KeyboardModel nextModel) noexcept {
    if (static_cast<uint8_t>(nextModel) >= static_cast<uint8_t>(KeyboardModel::Count)) {
        return;
    }
    if (model_ != nextModel) {
        model_ = nextModel;
        clearKeyHold();
        latch_ = {};
        wipePhase_ = 1.0f;
    }
}

void KeyboardEngine::syncConfig(const GlobalConfigState& config) noexcept {
    config_ = config;
    if (config_.muteMode == KeySwitchMode::Momentary) {
        latch_ = {};
    }
}

float KeyboardEngine::maxHeldPressure() const noexcept {
    uint16_t qMax = 0u;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        if (isBlackHeld(i) && blackPressureQ16_[i] > qMax) {
            qMax = blackPressureQ16_[i];
        }
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        if (isWhiteHeld(i) && whitePressureQ16_[i] > qMax) {
            qMax = whitePressureQ16_[i];
        }
    }
    return dequantizePressureU16(qMax);
}

uint16_t KeyboardEngine::nextPressSeq() noexcept {
    if (pressCounter_ >= 0xFFF0u) {
        pressCounter_ = 4u;
        for (size_t i = 0u; i < kNumBlackKeys; ++i) {
            if (blackPressSeq_[i] != 0u) {
                blackPressSeq_[i] = ++pressCounter_;
            }
        }
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            if (whitePressSeq_[i] != 0u) {
                whitePressSeq_[i] = ++pressCounter_;
            }
        }
        if (shiftModelSeq_ != 0u) {
            shiftModelSeq_ = ++pressCounter_;
        }
    }
    return ++pressCounter_;
}

void KeyboardEngine::onBlackRisingEdge(size_t idx) noexcept {
    const uint16_t seq = nextPressSeq();
    blackPressSeq_[idx] = seq;

    if (idx == kShiftBlackKeyIdx) {
        shiftModelSeq_ = 0u;
        return;
    }

    if (isShiftHeld()) {
        shiftSwallowedBlackMask_ |= static_cast<uint16_t>(1u << idx);
        if (idx < static_cast<size_t>(KeyboardModel::Count)) {
            shiftModelEvent_ = static_cast<int8_t>(idx);
            shiftModelSeq_ = seq;
            return;
        }
        unassignedChannelEvent_ = static_cast<int16_t>(phantomBlackKeyChannel(idx));
        return;
    }

    if (idx == kHoldBlackKeyIdx) {
        if (holdActive_) {
            clearKeyHold();
        } else {
            holdActive_ = true;
            repressBlackMask_ = 0u;
            repressWhiteMask_ = 0u;
            for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
                const uint16_t bit = static_cast<uint16_t>(1u << k);
                if (isBlackHeld(k) && (shiftSwallowedBlackMask_ & bit) == 0u) {
                    latchedBlackMask_ |= bit;
                    latchedBlackPressureQ16_[k] = blackPressureQ16_[k];
                }
            }
            for (size_t w = 0u; w < kMuteWhiteKeyBase; ++w) {
                const uint16_t bit = static_cast<uint16_t>(1u << w);
                if (isWhiteHeld(w) && (shiftSwallowedWhiteMask_ & bit) == 0u) {
                    latchedWhiteMask_ |= bit;
                    latchedWhitePressureQ16_[w] = whitePressureQ16_[w];
                }
            }
        }
        holdToggledEvent_ = true;
        return;
    }

    if (idx == kTapBlackKeyIdx) {
        if (model_ == KeyboardModel::RhythmLab) {
            tapTempoEvent_ = true;
        } else {
            unassignedChannelEvent_ = static_cast<int16_t>(phantomBlackKeyChannel(idx));
        }
        return;
    }

    // Expressive top pad (OP1..GLO) pressed while LATCH is engaged -> latch (or arm re-press)!
    if (holdActive_ && idx < kNumTopMacroKeys) {
        beginLatchedPress(static_cast<uint16_t>(1u << idx), blackPressureQ16_[idx],
                          latchedBlackMask_, repressBlackMask_, latchedBlackPressureQ16_[idx]);
    }
}

void KeyboardEngine::onWhiteRisingEdge(uint8_t whiteIdx) noexcept {
    const uint16_t seq = nextPressSeq();
    whitePressSeq_[whiteIdx] = seq;
    const uint16_t wBit = static_cast<uint16_t>(1u << whiteIdx);

    if (isShiftHeld()) {
        shiftSwallowedWhiteMask_ |= wBit;
        if (whiteIdx >= kSceneWhiteKeyBase && whiteIdx < kSceneWhiteKeyBase + kNumSnapshots) {
            shiftSceneMask_ |= static_cast<uint8_t>(1u << (whiteIdx - kSceneWhiteKeyBase));
            shiftModelSeq_ = 0u;
            return;
        }
        if (whiteIdx >= kMuteWhiteKeyBase && whiteIdx < kMuteWhiteKeyBase + kNumOscillators) {
            onConsoleSoloMuteKeyPress(true, whiteIdx, shiftSoloHeldMask_, config_, latch_);
            shiftSoloHeldMask_ |= static_cast<uint8_t>(1u << (whiteIdx - kMuteWhiteKeyBase));
            shiftModelSeq_ = 0u;
            return;
        }
        unassignedChannelEvent_ = static_cast<int16_t>(phantomWhiteKeyChannel(whiteIdx));
        return;
    }

    if (whiteIdx >= kMuteWhiteKeyBase && whiteIdx < kMuteWhiteKeyBase + kNumOscillators) {
        onConsoleSoloMuteKeyPress(false, whiteIdx, 0u, config_, latch_);
        return;
    }

    if (holdActive_ && whiteIdx < kMuteWhiteKeyBase) {
        beginLatchedPress(wBit, whitePressureQ16_[whiteIdx], latchedWhiteMask_, repressWhiteMask_,
                          latchedWhitePressureQ16_[whiteIdx]);
    }
}

void KeyboardEngine::setBlackGate(BlackKeyId key, bool pressed) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys) {
        return;
    }
    const bool wasHeld = isBlackHeld(idx);
    const uint16_t bit = static_cast<uint16_t>(1u << idx);
    if (pressed) {
        heldBlackMask_ |= bit;
        if (blackPressureQ16_[idx] <= 66u) {
            blackPressureQ16_[idx] = quantizePressureU16(kDefaultGatePressure);
        }
        if (!wasHeld) {
            onBlackRisingEdge(idx);
        }
    } else {
        heldBlackMask_ &= static_cast<uint16_t>(~bit);
        shiftSwallowedBlackMask_ &= static_cast<uint16_t>(~bit);
        repressBlackMask_ &= static_cast<uint16_t>(~bit);
        blackPressureQ16_[idx] = 0u;
        blackPressSeq_[idx] = 0u;
        if (idx == kShiftBlackKeyIdx) {
            shiftSceneMask_ = 0u;
            shiftSoloHeldMask_ = 0u;
            shiftModelSeq_ = 0u;
        }
        if (!anyKeyHeld()) {
            pressCounter_ = 0u;
        }
    }
}

void KeyboardEngine::setBlackPressure(BlackKeyId key, float pressure) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys) {
        return;
    }
    const float clamped = clampf(pressure, 0.0f, 1.0f);
    const bool wasHeld = isBlackHeld(idx);
    const uint16_t qVal = quantizePressureU16(clamped);
    blackPressureQ16_[idx] = qVal;
    const bool held = (clamped > 0.001f);
    const uint16_t bit = static_cast<uint16_t>(1u << idx);
    if (held) {
        heldBlackMask_ |= bit;
        if (!wasHeld) {
            onBlackRisingEdge(idx);
        } else if (holdActive_ && !isShiftHeld() && idx < kNumTopMacroKeys &&
                   (shiftSwallowedBlackMask_ & bit) == 0u) {
            trackLatchedPeak(bit, qVal, latchedBlackMask_, repressBlackMask_,
                             latchedBlackPressureQ16_[idx]);
        }
    } else {
        heldBlackMask_ &= static_cast<uint16_t>(~bit);
        shiftSwallowedBlackMask_ &= static_cast<uint16_t>(~bit);
        repressBlackMask_ &= static_cast<uint16_t>(~bit);
        blackPressSeq_[idx] = 0u;
        if (idx == kShiftBlackKeyIdx) {
            shiftSceneMask_ = 0u;
            shiftSoloHeldMask_ = 0u;
            shiftModelSeq_ = 0u;
        }
        if (!anyKeyHeld()) {
            pressCounter_ = 0u;
        }
    }
}

void KeyboardEngine::setWhiteGate(uint8_t whiteIdx, bool pressed) noexcept {
    if (whiteIdx >= kNumWhiteKeys) {
        return;
    }
    const bool wasHeld = isWhiteHeld(whiteIdx);
    const uint16_t bit = static_cast<uint16_t>(1u << whiteIdx);
    if (pressed) {
        heldWhiteMask_ |= bit;
        if (whitePressureQ16_[whiteIdx] <= 66u) {
            whitePressureQ16_[whiteIdx] = quantizePressureU16(kDefaultGatePressure);
        }
        if (!wasHeld) {
            onWhiteRisingEdge(whiteIdx);
        }
    } else {
        heldWhiteMask_ &= static_cast<uint16_t>(~bit);
        shiftSwallowedWhiteMask_ &= static_cast<uint16_t>(~bit);
        repressWhiteMask_ &= static_cast<uint16_t>(~bit);
        if (whiteIdx >= kSceneWhiteKeyBase && whiteIdx < kSceneWhiteKeyBase + kNumSnapshots) {
            shiftSceneMask_ &= static_cast<uint8_t>(~(1u << (whiteIdx - kSceneWhiteKeyBase)));
        }
        if (whiteIdx >= kMuteWhiteKeyBase && whiteIdx < kMuteWhiteKeyBase + kNumOscillators) {
            shiftSoloHeldMask_ &= static_cast<uint8_t>(~(1u << (whiteIdx - kMuteWhiteKeyBase)));
        }
        whitePressureQ16_[whiteIdx] = 0u;
        whitePressSeq_[whiteIdx] = 0u;
        if (!anyKeyHeld()) {
            pressCounter_ = 0u;
        }
    }
}

void KeyboardEngine::setWhitePressure(uint8_t whiteIdx, float pressure) noexcept {
    if (whiteIdx >= kNumWhiteKeys) {
        return;
    }
    const float clamped = clampf(pressure, 0.0f, 1.0f);
    const bool wasHeld = isWhiteHeld(whiteIdx);
    const uint16_t qVal = quantizePressureU16(clamped);
    whitePressureQ16_[whiteIdx] = qVal;
    const bool held = (clamped > 0.001f);
    const uint16_t bit = static_cast<uint16_t>(1u << whiteIdx);
    if (held) {
        heldWhiteMask_ |= bit;
        if (!wasHeld) {
            onWhiteRisingEdge(whiteIdx);
        } else if (holdActive_ && !isShiftHeld() && whiteIdx < kMuteWhiteKeyBase &&
                   (shiftSwallowedWhiteMask_ & bit) == 0u) {
            trackLatchedPeak(bit, qVal, latchedWhiteMask_, repressWhiteMask_,
                             latchedWhitePressureQ16_[whiteIdx]);
        }
    } else {
        heldWhiteMask_ &= static_cast<uint16_t>(~bit);
        shiftSwallowedWhiteMask_ &= static_cast<uint16_t>(~bit);
        repressWhiteMask_ &= static_cast<uint16_t>(~bit);
        if (whiteIdx >= kSceneWhiteKeyBase && whiteIdx < kSceneWhiteKeyBase + kNumSnapshots) {
            shiftSceneMask_ &= static_cast<uint8_t>(~(1u << (whiteIdx - kSceneWhiteKeyBase)));
        }
        if (whiteIdx >= kMuteWhiteKeyBase && whiteIdx < kMuteWhiteKeyBase + kNumOscillators) {
            shiftSoloHeldMask_ &= static_cast<uint8_t>(~(1u << (whiteIdx - kMuteWhiteKeyBase)));
        }
        whitePressSeq_[whiteIdx] = 0u;
        if (!anyKeyHeld()) {
            pressCounter_ = 0u;
        }
    }
}

void KeyboardEngine::buildEffectiveInput(std::array<bool, kNumBlackKeys>& effBlackHeld,
                                         std::array<bool, kNumWhiteKeys>& effWhiteHeld,
                                         std::array<float, kNumBlackKeys>& effBlackPress,
                                         std::array<float, kNumWhiteKeys>& effWhitePress,
                                         std::array<uint16_t, kNumBlackKeys>& effBlackSeq,
                                         std::array<uint16_t, kNumWhiteKeys>& effWhiteSeq,
                                         uint16_t& outEffBlackMask, uint16_t& outEffWhiteMask,
                                         uint8_t& outMuteHeld,
                                         uint8_t& outSoloHeld) const noexcept {
    effBlackHeld.fill(false);
    effWhiteHeld.fill(false);
    effBlackPress.fill(0.0f);
    effWhitePress.fill(0.0f);
    effBlackSeq.fill(0u);
    effWhiteSeq.fill(0u);
    outEffBlackMask = 0u;
    outEffWhiteMask = 0u;
    outMuteHeld = 0u;
    outSoloHeld = isShiftHeld() ? shiftSoloHeldMask_ : 0u;

    for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
        const uint16_t bit = static_cast<uint16_t>(1u << i);
        const bool phys = isBlackHeld(i) && (shiftSwallowedBlackMask_ & bit) == 0u;
        const bool lat = holdActive_ && ((latchedBlackMask_ & bit) != 0u);
        if (phys || lat) {
            effBlackHeld[i] = true;
            outEffBlackMask |= bit;
            const uint16_t q =
                (lat && (!phys || latchedBlackPressureQ16_[i] > blackPressureQ16_[i]))
                    ? latchedBlackPressureQ16_[i]
                    : blackPressureQ16_[i];
            effBlackPress[i] = dequantizePressureU16(q);
            effBlackSeq[i] = phys ? (blackPressSeq_[i] + 4u) : 1u;
        }
    }

    for (size_t w = 0u; w < kMuteWhiteKeyBase; ++w) {
        const uint16_t bit = static_cast<uint16_t>(1u << w);
        const bool phys = isWhiteHeld(w) && (shiftSwallowedWhiteMask_ & bit) == 0u;
        const bool lat = holdActive_ && ((latchedWhiteMask_ & bit) != 0u);
        if (phys || lat) {
            effWhiteHeld[w] = true;
            outEffWhiteMask |= bit;
            const uint16_t q =
                (lat && (!phys || latchedWhitePressureQ16_[w] > whitePressureQ16_[w]))
                    ? latchedWhitePressureQ16_[w]
                    : whitePressureQ16_[w];
            effWhitePress[w] = dequantizePressureU16(q);
            effWhiteSeq[w] = phys ? (whitePressSeq_[w] + 4u) : 1u;
        }
    }

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        const size_t mIdx = kMuteWhiteKeyBase + v;
        const uint16_t bit = static_cast<uint16_t>(1u << mIdx);
        if (isWhiteHeld(mIdx) && (shiftSwallowedWhiteMask_ & bit) == 0u) {
            outMuteHeld |= static_cast<uint8_t>(1u << v);
            effWhiteHeld[mIdx] = true;
            outEffWhiteMask |= bit;
            effWhiteSeq[mIdx] = whitePressSeq_[mIdx] + 4u;
        }
        if ((outSoloHeld & (1u << v)) != 0u) {
            effWhiteSeq[mIdx] = whitePressSeq_[mIdx] + 4u;
        }
    }
}

void KeyboardEngine::evaluate(EngineControlState& state, HudState& hud) const noexcept {
    hud.keyboardModel = model_;
    hud.shiftHeld = isShiftHeld();
    hud.keyHoldLatched = holdActive_;
    hud.latchedBlackMask = holdActive_ ? (latchedBlackMask_ & kAnalogBlackExprMask) : 0u;
    hud.latchedWhiteMask = holdActive_ ? (latchedWhiteMask_ & kAnalogWhiteExprMask) : 0u;
    hud.keyHoldCount =
        static_cast<uint8_t>(popcount16(hud.latchedBlackMask) + popcount16(hud.latchedWhiteMask));
    hud.heldBlackMask = heldBlackMask_;
    hud.heldWhiteMask = heldWhiteMask_;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        const bool lat = (i < kNumTopMacroKeys) && ((hud.latchedBlackMask & (1u << i)) != 0u);
        const uint16_t q =
            (lat && (!isBlackHeld(i) || latchedBlackPressureQ16_[i] > blackPressureQ16_[i]))
                ? latchedBlackPressureQ16_[i]
                : blackPressureQ16_[i];
        hud.blackPressureQ8[i] = quantizeU16ToU8(q);
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        const bool lat = (i < kMuteWhiteKeyBase) && ((hud.latchedWhiteMask & (1u << i)) != 0u);
        const uint16_t q =
            (lat && (!isWhiteHeld(i) || latchedWhitePressureQ16_[i] > whitePressureQ16_[i]))
                ? latchedWhitePressureQ16_[i]
                : whitePressureQ16_[i];
        hud.whitePressureQ8[i] = quantizeU16ToU8(q);
    }

    state.keyMod = {};
    state.voicePressure = {0.0f, 0.0f, 0.0f, 0.0f};
    hud.modeWipeQ8 = modeWipeQ8();
    hud.scopeFrozen = false;
    hud.activePunch = KeyPunchKind::None;
    hud.activePunchVoice = 0u;
    hud.activePunchValue = 0.0f;

    std::array<bool, kNumBlackKeys> effBlackHeld{};
    std::array<bool, kNumWhiteKeys> effWhiteHeld{};
    std::array<float, kNumBlackKeys> effBlackPress{};
    std::array<float, kNumWhiteKeys> effWhitePress{};
    std::array<uint16_t, kNumBlackKeys> effBlackSeq{};
    std::array<uint16_t, kNumWhiteKeys> effWhiteSeq{};
    uint16_t effBlackMask = 0u;
    uint16_t effWhiteMask = 0u;
    uint8_t muteHeldMask = 0u;
    uint8_t soloHeldMask = 0u;
    buildEffectiveInput(effBlackHeld, effWhiteHeld, effBlackPress, effWhitePress, effBlackSeq,
                        effWhiteSeq, effBlackMask, effWhiteMask, muteHeldMask, soloHeldMask);

    const KeyboardInputView view{effBlackHeld, effWhiteHeld, effBlackPress, effWhitePress,
                                 effBlackSeq,  effWhiteSeq,  effBlackMask,  effWhiteMask,
                                 muteHeldMask, soloHeldMask, latch_,        config_};
    keyboardModelOps(model_).evaluate(view, state, hud);

    if (hud.shiftHeld && !hud.saveArmed) {
        // Report strictly SHIFT-layer actions (`KEYBOARD 1-3`, `SCENE 9-12`, or `SOLO 13-16`)
        PunchTracker sp{};
        const bool muteLatch = (config_.muteMode == KeySwitchMode::Latching);
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            if ((shiftSceneMask_ & (1u << s)) != 0u) {
                const size_t wIdx = kSceneWhiteKeyBase + s;
                sp.note(whitePressSeq_[wIdx], KeyPunchKind::SnapshotRecall, s,
                        dequantizePressureU16(whitePressureQ16_[wIdx]));
            }
        }
        for (uint8_t v = 0u; v < kNumOscillators; ++v) {
            if ((shiftSoloHeldMask_ & (1u << v)) != 0u) {
                const size_t mIdx = kMuteWhiteKeyBase + v;
                const bool isChMuted = ((state.keyMod.muteMask >> v) & 1u) != 0u;
                const bool isSoloed = muteLatch ? (latch_.preSoloValid && !isChMuted)
                                                : (((state.keyMod.soloMask >> v) & 1u) != 0u);
                sp.note(whitePressSeq_[mIdx], KeyPunchKind::SpotlightSolo, v,
                        isSoloed ? 1.0f : 0.0f);
            }
        }
        if (shiftModelSeq_ > 0u && shiftModelSeq_ >= sp.latestSeq) {
            hud.shiftFocus = ShiftFocus::Model;
            hud.activePunch = KeyPunchKind::None;
            return;
        }
        if (sp.latestSeq > 0u && hud.shiftFocus == ShiftFocus::Model) {
            hud.shiftFocus = ShiftFocus::Idle;
        }
        sp.commitTo(hud);
        return;
    }

    // Suppress hands-off HOLD-latched punches (`seq == 1`) from `hud.activePunch` so Clean Scope
    // returns to pure waveform when hands lift! Also inspect HOLD / TAP when physically held.
    if (!anyKeyHeld()) {
        hud.activePunch = KeyPunchKind::None;
        hud.activePunchVoice = 0u;
        hud.activePunchValue = 0.0f;
        return;
    }

    PunchTracker physPunch{};
    // Re-pick only physically held keys (seq >= 5) or HOLD / TAP switches
    if (hud.activePunch != KeyPunchKind::None) {
        uint16_t maxPhysExprSeq = 0u;
        for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
            if (isBlackHeld(i) && (shiftSwallowedBlackMask_ & (1u << i)) == 0u &&
                blackPressSeq_[i] > maxPhysExprSeq) {
                maxPhysExprSeq = blackPressSeq_[i];
            }
        }
        for (size_t w = 0u; w < kNumWhiteKeys; ++w) {
            if (isWhiteHeld(w) && (shiftSwallowedWhiteMask_ & (1u << w)) == 0u &&
                whitePressSeq_[w] > maxPhysExprSeq) {
                maxPhysExprSeq = whitePressSeq_[w];
            }
        }
        if (maxPhysExprSeq > 0u) {
            physPunch.note(maxPhysExprSeq, hud.activePunch, hud.activePunchVoice,
                           hud.activePunchValue);
        }
    }
    if (isBlackHeld(kHoldBlackKeyIdx) &&
        (shiftSwallowedBlackMask_ & (1u << kHoldBlackKeyIdx)) == 0u) {
        const float holdVal = holdActive_ ? ((hud.keyHoldCount > 0u) ? 1.0f : 0.5f) : 0.0f;
        physPunch.note(blackPressSeq_[kHoldBlackKeyIdx], KeyPunchKind::KeyHoldSwitch, 0u, holdVal);
    }
    if (model_ == KeyboardModel::RhythmLab && isBlackHeld(kTapBlackKeyIdx) &&
        (shiftSwallowedBlackMask_ & (1u << kTapBlackKeyIdx)) == 0u) {
        physPunch.note(blackPressSeq_[kTapBlackKeyIdx], KeyPunchKind::TapTempo, 0u, 1.0f);
    }
    physPunch.commitTo(hud);
}

void KeyboardEngine::stepAndComputeKeyLeds(float dtSec, float animPhase,
                                           const EngineControlState& state, const HudState& hud,
                                           const EngineTelemetry& telemetry,
                                           LedState& outLeds) noexcept {
    if (wipePhase_ > 0.0f) {
        wipePhase_ = clampf(wipePhase_ - dtSec * (1.0f / kModeWipeDurationSec), 0.0f, 1.0f);
    }

    const uint8_t stutMask = telemetry.stutterGateMask.load(std::memory_order_relaxed);
    const float rhythmPhase = telemetry.rhythmPhase.load(std::memory_order_relaxed);
    computeKeyboardModelRestLeds(model_, state, hud, outLeds.blackKeyLeds, outLeds.whiteKeyLeds);

    // Pure-blinking `SAVE`/`DELETE` rule: while `saveArmed`, keyboard LEDs convey ONLY the blinking
    // Scene target affordance (`Keys 9-12`) with zero other keybed state!
    if (hud.saveArmed) {
        return;
    }

    // In 3/3 RHYTHM LAB, TAP (`POLY`, Black 9) pulses on quarter notes (damped above 900 BPM).
    if (!hud.shiftHeld && model_ == KeyboardModel::RhythmLab) {
        outLeds.blackKeyLeds[kTapBlackKeyIdx] = computeTapTempoBeatGlow(state, rhythmPhase);
    }

    if (!hud.shiftHeld) {
        std::array<bool, kNumBlackKeys> effBlackHeld{};
        std::array<bool, kNumWhiteKeys> effWhiteHeld{};
        std::array<float, kNumBlackKeys> effBlackPress{};
        std::array<float, kNumWhiteKeys> effWhitePress{};
        std::array<uint16_t, kNumBlackKeys> effBlackSeq{};
        std::array<uint16_t, kNumWhiteKeys> effWhiteSeq{};
        uint16_t effBlackMask = 0u;
        uint16_t effWhiteMask = 0u;
        uint8_t muteHeldMask = 0u;
        uint8_t soloHeldMask = 0u;
        buildEffectiveInput(effBlackHeld, effWhiteHeld, effBlackPress, effWhitePress, effBlackSeq,
                            effWhiteSeq, effBlackMask, effWhiteMask, muteHeldMask, soloHeldMask);
        if (state.global.masterRun && (effBlackMask | effWhiteMask) != 0u) {
            const KeyboardInputView view{effBlackHeld, effWhiteHeld, effBlackPress, effWhitePress,
                                         effBlackSeq,  effWhiteSeq,  effBlackMask,  effWhiteMask,
                                         muteHeldMask, soloHeldMask, latch_,        config_};
            keyboardModelOps(model_).choreographActiveLeds(view, animPhase, state, hud, stutMask,
                                                           rhythmPhase, outLeds);
        }
    } else {
        // Dim top panel buttons subtly during SHIFT
        scalePanelGrid(outLeds, 0.22f);
        applyAuthoritativeConsoleLeds(state, hud, outLeds);
    }

    // Signature 220 ms Mode-Select LED Sweep across BOTH the 27-key bed & top panel switches
    // (consistent across SEL, SELECT, SHIFT + SELECT, and SHIFT + OP1..OP3)!
    if (wipePhase_ > 0.001f) {
        const float prog = 1.0f - wipePhase_;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            float pos01 = static_cast<float>(i) * (1.0f / 15.0f);
            if (model_ == KeyboardModel::CircuitLab) {
                pos01 = static_cast<float>((i * 7u + 3u) & 15u) * (1.0f / 15.0f);
            } else if (model_ == KeyboardModel::RhythmLab) {
                pos01 = static_cast<float>(((i & 3u) * 4u) + (i >> 2u)) * (1.0f / 15.0f);
            }
            const float pulse = clampf(1.0f - absf(prog * 1.25f - pos01) * 4.2f, 0.0f, 1.0f);
            outLeds.whiteKeyLeds[i] =
                clampf(outLeds.whiteKeyLeds[i] + pulse * wipePhase_, 0.0f, 1.0f);
        }
        for (size_t i = 0u; i < kNumBlackKeys; ++i) {
            const float pos01 = static_cast<float>(i) * (1.0f / 10.0f);
            const float pulse = clampf(1.0f - absf(prog * 1.25f - pos01) * 4.0f, 0.0f, 1.0f);
            outLeds.blackKeyLeds[i] =
                clampf(outLeds.blackKeyLeds[i] + pulse * wipePhase_, 0.0f, 1.0f);
        }
        for (size_t slot = 0u; slot < 8u; ++slot) {
            const float pos01 = static_cast<float>(slot) * (1.0f / 7.0f);
            const float pulse =
                clampf(1.0f - absf(prog * 1.25f - pos01) * 3.8f, 0.0f, 1.0f) * wipePhase_;
            if (slot == 0u) {
                maxPanelBtnLed(outLeds, ButtonId::OctMinus, pulse);
            } else if (slot == 1u) {
                maxPanelBtnLed(outLeds, ButtonId::OctPlus, pulse);
            } else {
                const size_t c = slot - 2u;
                maxPanelBtnLed(outLeds, kPanelGridTop[c], pulse);
                if (c < 4u) {
                    maxPanelBtnLed(outLeds, kPanelGridBot[c], pulse * 0.85f);
                }
            }
        }
    }

    if (hud.knobRate == KnobRate::Fine) {
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 1.0f);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.15f);
    } else if (hud.knobRate == KnobRate::Coarse) {
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.15f);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 1.0f);
    }
    if (!hud.shiftHeld) {
        const bool nonHomePage = hud.fxPageOpen || hud.selPageOpen || hud.lfoPageOpen ||
                                 hud.editPageOpen || hud.panPageOpen || hud.configPageOpen ||
                                 hud.saveArmed;
        if (!nonHomePage) {
            setPanelBtnLed(outLeds, ButtonId::Home, 1.0f);
        } else if (hud.selPageOpen) {
            setPanelBtnLed(outLeds, ButtonId::Sel, 1.0f);
        }
    }
}

} // namespace ruido
