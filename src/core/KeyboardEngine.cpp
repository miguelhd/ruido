#include "core/KeyboardEngine.h"

#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kModeWipeDurationSec = 0.220f;

inline uint8_t quantizePressureU8(float p) noexcept {
    return static_cast<uint8_t>(clampf(p, 0.0f, 1.0f) * 255.0f + 0.5f);
}

void onScaffoldKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                        KeyboardLatchState& latch) noexcept {
    (void)isBlack;
    (void)keyIdx;
    (void)config;
    (void)latch;
}

void evaluateScaffoldModel(const KeyboardInputView& input, EngineControlState& state,
                           HudState& hud) noexcept {
    (void)input;
    (void)state;
    (void)hud;
}

void choreographScaffoldLeds(const KeyboardInputView& input, float animPhase, float subBreathWave01,
                             const EngineControlState& state, const HudState& hud,
                             uint8_t stutterGateMask, LedState& outLeds) noexcept {
    (void)animPhase;
    (void)subBreathWave01;
    (void)state;
    (void)hud;
    (void)stutterGateMask;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        if (input.blackHeld[i]) {
            outLeds.blackKeyLeds[i] = 1.0f;
        }
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        if (input.whiteHeld[i]) {
            outLeds.whiteKeyLeds[i] = 1.0f;
        }
    }
}

void evaluateCircuitLabModel(const KeyboardInputView& in, EngineControlState& state,
                             HudState& hud) noexcept {
    KeyboardModState mod{};
    uint32_t latestSeq = 0u;
    KeyPunchKind latestKind = KeyPunchKind::None;
    uint8_t latestVoice = 0u;
    float latestValue = 0.0f;
    const auto notePunch = [&](uint32_t seq, KeyPunchKind kind, uint8_t vIdx, float val) noexcept {
        if (seq >= latestSeq) {
            latestSeq = seq;
            latestKind = kind;
            latestVoice = vIdx;
            latestValue = val;
        }
    };

    const bool soloLatch = (in.config.soloMode == KeySwitchMode::Latching);
    const bool muteLatch = (in.config.muteMode == KeySwitchMode::Latching);
    uint8_t soloMask = soloLatch ? in.latch.soloMask : 0u;
    uint8_t muteMask = muteLatch ? in.latch.muteMask : 0u;
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        const uint8_t bit = static_cast<uint8_t>(1u << v);
        if (!soloLatch && in.blackHeld[v]) {
            soloMask |= bit;
        }
        if (!muteLatch && in.whiteHeld[v]) {
            muteMask |= bit;
        }
        if (in.blackHeld[v]) {
            notePunch(in.blackPressSeq[v], KeyPunchKind::SpotlightSolo, v,
                      ((soloMask & bit) != 0u) ? 1.0f : 0.0f);
        }
    }
    mod.soloMask = soloMask;
    if (soloMask != 0u) {
        for (uint8_t v = 0u; v < kNumOscillators; ++v) {
            mod.voiceGainMul[v] = (((soloMask >> v) & 1u) != 0u) ? 1.0f : 0.0f;
        }
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)]) {
        const size_t idx = static_cast<size_t>(BlackKeyId::ScopeFreezeKey);
        mod.stasisFreeze = true;
        hud.scopeFrozen = true;
        notePunch(in.blackPressSeq[idx], KeyPunchKind::StasisFreeze, 0u, 1.0f);
    }
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        if (((muteMask >> v) & 1u) != 0u) {
            mod.voiceGainMul[v] = 0.0f;
        }
        if (in.whiteHeld[v]) {
            notePunch(in.whitePressSeq[v], KeyPunchKind::VoiceCut, v,
                      (((muteMask >> v) & 1u) != 0u) ? 1.0f : 0.0f);
        }
        // Circuit Lab Bank C (White 8..11): Stutter Chopper (1.2 Hz -> 20 Hz)
        if (in.whiteHeld[8u + v]) {
            const float p = in.whitePressure[8u + v];
            mod.stutterDepth[v] = 1.0f;
            mod.stutterRateHz[v] = 1.2f * fastExp2(4.0588937f * p);
            notePunch(in.whitePressSeq[8u + v], KeyPunchKind::VoiceStutter, v,
                      mod.stutterRateHz[v]);
        }
    }
    hud.activePunch = latestKind;
    hud.activePunchVoice = latestVoice;
    hud.activePunchValue = latestValue;
    state.keyMod = mod;
}

void choreographCircuitLabLeds(const KeyboardInputView& in, float animPhase, float subBreathWave01,
                               const EngineControlState& state, const HudState& hud,
                               uint8_t stutMask, LedState& outLeds) noexcept {
    (void)animPhase;
    (void)hud;
    choreographScaffoldLeds(in, animPhase, subBreathWave01, state, hud, stutMask, outLeds);
    bool anyChopOpen = false;
    bool anyChopClosed = false;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (in.whiteHeld[8u + v]) {
            const bool gateOpen = ((stutMask >> v) & 1u) != 0u;
            outLeds.whiteKeyLeds[8u + v] = gateOpen ? 1.0f : 0.03f;
            anyChopOpen = anyChopOpen || gateOpen;
            anyChopClosed = anyChopClosed || !gateOpen;
        }
        const VoiceCircuitState vc = evaluateVoiceCircuit(state, v, stutMask);
        evaluateVoiceConsoleLeds(vc, subBreathWave01, outLeds.blackKeyLeds[v],
                                 outLeds.whiteKeyLeds[v]);
    }
    if (anyChopOpen || anyChopClosed) {
        maxPanelBtnLed(outLeds, ButtonId::Env, anyChopOpen ? 0.92f : 0.02f);
        maxPanelBtnLed(outLeds, ButtonId::Arp, anyChopClosed ? 0.92f : 0.02f);
    }
}

constexpr std::array<KeyboardModelOps, static_cast<size_t>(KeyboardModel::Count)> kModelOpsTable = {
    {
        {&onLiveMixerKeyPress, &evaluateLiveMixerModel, &choreographLiveMixerLeds},
        {&onScaffoldKeyPress, &evaluateScaffoldModel, &choreographScaffoldLeds},
        {&onLiveMixerKeyPress, &evaluateCircuitLabModel, &choreographCircuitLabLeds},
    }};

} // namespace

const KeyboardModelOps& keyboardModelOps(KeyboardModel model) noexcept {
    const auto idx = static_cast<size_t>(model);
    return (idx < kModelOpsTable.size()) ? kModelOpsTable[idx] : kModelOpsTable[0];
}

void KeyboardEngine::reset() noexcept {
    model_ = KeyboardModel::LiveMixer;
    blackHeld_.fill(false);
    whiteHeld_.fill(false);
    blackPressure_.fill(0.0f);
    whitePressure_.fill(0.0f);
    blackPressSeq_.fill(0u);
    whitePressSeq_.fill(0u);
    pressCounter_ = 0u;
    heldBlackMask_ = 0u;
    heldWhiteMask_ = 0u;
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
        latch_ = {};
        wipePhase_ = 1.0f;
    }
}

void KeyboardEngine::syncConfig(const GlobalConfigState& config) noexcept {
    config_ = config;
    if (config_.muteMode == KeySwitchMode::Momentary) {
        latch_.muteMask = 0u;
    }
    if (config_.soloMode == KeySwitchMode::Momentary) {
        latch_.soloMask = 0u;
    }
}

float KeyboardEngine::maxHeldPressure() const noexcept {
    float pMax = 0.0f;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        if (blackHeld_[i] && blackPressure_[i] > pMax) {
            pMax = blackPressure_[i];
        }
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        if (whiteHeld_[i] && whitePressure_[i] > pMax) {
            pMax = whitePressure_[i];
        }
    }
    return clampf(pMax, 0.0f, 1.0f);
}

void KeyboardEngine::setBlackGate(BlackKeyId key, bool pressed) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys) {
        return;
    }
    const bool wasHeld = blackHeld_[idx];
    blackHeld_[idx] = pressed;
    if (pressed) {
        heldBlackMask_ |= static_cast<uint16_t>(1u << idx);
        if (!wasHeld) {
            blackPressSeq_[idx] = ++pressCounter_;
            keyboardModelOps(model_).onKeyPress(true, static_cast<uint8_t>(idx), config_, latch_);
        }
        if (blackPressure_[idx] <= 0.001f) {
            blackPressure_[idx] = kDefaultGatePressure;
        }
    } else {
        heldBlackMask_ &= static_cast<uint16_t>(~(1u << idx));
        blackPressure_[idx] = 0.0f;
        blackPressSeq_[idx] = 0u;
    }
}

void KeyboardEngine::setBlackPressure(BlackKeyId key, float pressure) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys) {
        return;
    }
    const float clamped = clampf(pressure, 0.0f, 1.0f);
    const bool wasHeld = blackHeld_[idx];
    blackPressure_[idx] = clamped;
    const bool held = (clamped > 0.001f);
    blackHeld_[idx] = held;
    if (held) {
        heldBlackMask_ |= static_cast<uint16_t>(1u << idx);
        if (!wasHeld) {
            blackPressSeq_[idx] = ++pressCounter_;
            keyboardModelOps(model_).onKeyPress(true, static_cast<uint8_t>(idx), config_, latch_);
        }
    } else {
        heldBlackMask_ &= static_cast<uint16_t>(~(1u << idx));
        blackPressSeq_[idx] = 0u;
    }
}

void KeyboardEngine::setWhiteGate(uint8_t whiteIdx, bool pressed) noexcept {
    if (whiteIdx >= kNumWhiteKeys) {
        return;
    }
    const bool wasHeld = whiteHeld_[whiteIdx];
    whiteHeld_[whiteIdx] = pressed;
    if (pressed) {
        heldWhiteMask_ |= static_cast<uint16_t>(1u << whiteIdx);
        if (!wasHeld) {
            whitePressSeq_[whiteIdx] = ++pressCounter_;
            keyboardModelOps(model_).onKeyPress(false, whiteIdx, config_, latch_);
        }
        if (whitePressure_[whiteIdx] <= 0.001f) {
            whitePressure_[whiteIdx] = kDefaultGatePressure;
        }
    } else {
        heldWhiteMask_ &= static_cast<uint16_t>(~(1u << whiteIdx));
        whitePressure_[whiteIdx] = 0.0f;
        whitePressSeq_[whiteIdx] = 0u;
    }
}

void KeyboardEngine::setWhitePressure(uint8_t whiteIdx, float pressure) noexcept {
    if (whiteIdx >= kNumWhiteKeys) {
        return;
    }
    const float clamped = clampf(pressure, 0.0f, 1.0f);
    const bool wasHeld = whiteHeld_[whiteIdx];
    whitePressure_[whiteIdx] = clamped;
    const bool held = (clamped > 0.001f);
    whiteHeld_[whiteIdx] = held;
    if (held) {
        heldWhiteMask_ |= static_cast<uint16_t>(1u << whiteIdx);
        if (!wasHeld) {
            whitePressSeq_[whiteIdx] = ++pressCounter_;
            keyboardModelOps(model_).onKeyPress(false, whiteIdx, config_, latch_);
        }
    } else {
        heldWhiteMask_ &= static_cast<uint16_t>(~(1u << whiteIdx));
        whitePressSeq_[whiteIdx] = 0u;
    }
}

void KeyboardEngine::evaluate(EngineControlState& state, HudState& hud) const noexcept {
    hud.keyboardModel = model_;
    hud.heldBlackMask = heldBlackMask_;
    hud.heldWhiteMask = heldWhiteMask_;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        hud.blackPressureQ8[i] = quantizePressureU8(blackPressure_[i]);
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        hud.whitePressureQ8[i] = quantizePressureU8(whitePressure_[i]);
    }

    state.keyMod = {};
    state.voicePressure = {0.0f, 0.0f, 0.0f, 0.0f};
    hud.scopeFrozen = false;
    hud.activePunch = KeyPunchKind::None;
    hud.activePunchVoice = 0u;
    hud.activePunchValue = 0.0f;

    const KeyboardInputView view{blackHeld_,     whiteHeld_,     blackPressure_, whitePressure_,
                                 blackPressSeq_, whitePressSeq_, heldBlackMask_, heldWhiteMask_,
                                 latch_,         config_};
    keyboardModelOps(model_).evaluate(view, state, hud);
}

void KeyboardEngine::stepAndComputeKeyLeds(float dtSec, float animPhase, float subBreathWave01,
                                           const EngineControlState& state, const HudState& hud,
                                           const EngineTelemetry& telemetry,
                                           LedState& outLeds) noexcept {
    if (wipePhase_ > 0.0f) {
        wipePhase_ = clampf(wipePhase_ - dtSec * (1.0f / kModeWipeDurationSec), 0.0f, 1.0f);
    }

    const uint8_t stutMask = telemetry.stutterGateMask.load(std::memory_order_relaxed);
    const float savePulse01 = 0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(animPhase * 2.5f)));
    computeKeyboardModelRestLeds(model_, state, hud, stutMask, subBreathWave01, savePulse01,
                                 outLeds.blackKeyLeds, outLeds.whiteKeyLeds);

    if (anyKeyHeld()) {
        const KeyboardInputView view{blackHeld_,     whiteHeld_,     blackPressure_, whitePressure_,
                                     blackPressSeq_, whitePressSeq_, heldBlackMask_, heldWhiteMask_,
                                     latch_,         config_};
        keyboardModelOps(model_).choreographActiveLeds(view, animPhase, subBreathWave01, state, hud,
                                                       stutMask, outLeds);
    }

    // Signature 220 ms Mode-Select LED Sweep across BOTH the 27-key bed & top panel switches!
    if (wipePhase_ > 0.001f) {
        const float prog = 1.0f - wipePhase_;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            float pos01 = static_cast<float>(i) * (1.0f / 15.0f);
            if (model_ == KeyboardModel::Harmonium) {
                pos01 = absf(static_cast<float>(i) - 7.5f) * (1.0f / 7.5f);
            } else if (model_ == KeyboardModel::CircuitLab) {
                pos01 = static_cast<float>((i * 7u + 3u) & 15u) * (1.0f / 15.0f);
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

    // Always honor explicit active rate-hold modifiers & latched SEL page indicator
    if (hud.knobRate == KnobRate::Fine) {
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 1.0f);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.15f);
    } else if (hud.knobRate == KnobRate::Coarse) {
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.15f);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 1.0f);
    }
    if (hud.selPageOpen) {
        setPanelBtnLed(outLeds, ButtonId::Sel, 1.0f);
    }
}

} // namespace ruido
