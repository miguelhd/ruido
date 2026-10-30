#include "core/RuidoController.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kHudBloomSec = 0.200f;    // Soft 200 ms S-curve fade-in on touch
constexpr float kHudHoldSec = 1.500f;     // 1.5 s hold after standalone tweaks
constexpr float kHudDissolveTau = 0.250f; // Smooth ~850 ms dissolve back to black

constexpr bool isModifierBlackKey(BlackKeyId k) noexcept {
    return k == BlackKeyId::Op1 || k == BlackKeyId::Op2 || k == BlackKeyId::Op3 ||
           k == BlackKeyId::Op4 || k == BlackKeyId::Op5 || k == BlackKeyId::Op6 ||
           k == BlackKeyId::Glo;
}

} // namespace

RuidoController::RuidoController() noexcept {
    reset();
}

void RuidoController::reset() noexcept {
    state_ = EngineControlState::makeDefault();
    masterPot_ = state_.global.masterTrim;
    hud_ = {};
    leds_ = {};
    blackKeyHeld_.fill(false);
    blackKeyOrder_.fill(0u);
    whiteKeyHeld_.fill(false);
    pressSeq_ = 0u;
    octMinusHeld_ = false;
    octPlusHeld_ = false;
    octMinusOrder_ = 0u;
    octPlusOrder_ = 0u;
    hudRawLevel_ = 0.0f;
    holdRemainingSec_ = 0.0f;
    animPhase_ = 0.0f;
    playStopRunSmooth_ = state_.global.masterRun ? 1.0f : 0.0f;
    latchedHudMode_ = HudMode::None;
    latchedButton_ = ButtonId::Count;
    selectTickAccum_ = 0;
    targetTickAccum_ = 0;
    hudActive_ = false;

    EngineTelemetry zeroTel{};
    updateLeds(zeroTel);
    publishState();
}

void RuidoController::publishState() noexcept {
    if (controlBus_ != nullptr) {
        controlBus_->publish(state_);
    }
}

RuidoController::HeldModifier RuidoController::activeModifier() const noexcept {
    // Most recently pressed still-held left/center black key wins deterministically
    uint32_t bestOrder = 0u;
    HeldModifier best = HeldModifier::None;

    const auto check = [&](BlackKeyId k, HeldModifier mod) {
        const size_t idx = static_cast<size_t>(k);
        if (blackKeyHeld_[idx] && blackKeyOrder_[idx] >= bestOrder) {
            bestOrder = blackKeyOrder_[idx];
            best = mod;
        }
    };

    check(BlackKeyId::Op1, HeldModifier::FilterVoice0);
    check(BlackKeyId::Op2, HeldModifier::FilterVoice1);
    check(BlackKeyId::Op3, HeldModifier::FilterVoice2);
    check(BlackKeyId::Op4, HeldModifier::FilterVoice3);
    check(BlackKeyId::Op5, HeldModifier::GlobalFilter);
    check(BlackKeyId::Op6, HeldModifier::Panning);
    check(BlackKeyId::Glo, HeldModifier::Amplitude);
    return best;
}

KnobRate RuidoController::activeKnobRate() const noexcept {
    if (octMinusHeld_ && octPlusHeld_) {
        return (octPlusOrder_ >= octMinusOrder_) ? KnobRate::Fast : KnobRate::Fine;
    }
    if (octMinusHeld_) {
        return KnobRate::Fine;
    }
    if (octPlusHeld_) {
        return KnobRate::Fast;
    }
    return KnobRate::Standard;
}

HudMode RuidoController::currentLfoHudMode() const noexcept {
    return (hud_.lfoVoiceFocus >= kNumOscillators) ? HudMode::GlobalLfo : HudMode::VoiceLfo;
}

bool RuidoController::isAnyMomentaryHudHoldActive() const noexcept {
    if (activeModifier() != HeldModifier::None || hud_.scopeFrozen || octMinusHeld_ ||
        octPlusHeld_) {
        return true;
    }
    for (size_t i = 4u; i < kNumWhiteKeys; ++i) {
        if (whiteKeyHeld_[i]) {
            return true;
        }
    }
    return false;
}

void RuidoController::triggerHud(HudMode mode, uint8_t voiceIndex, uint8_t knobIndex) noexcept {
    hud_.mode = mode;
    hud_.focusedVoice = static_cast<uint8_t>(voiceIndex & 3u);
    hud_.focusedKnob = static_cast<uint8_t>(knobIndex & 3u);
    hudActive_ = true;
    holdRemainingSec_ = kHudHoldSec;
}

void RuidoController::latchSwitchHud(HudMode mode, ButtonId button) noexcept {
    latchedHudMode_ = mode;
    latchedButton_ = button;
    if (button != ButtonId::Count) {
        hud_.lastButton = button;
    }
    hud_.mode = mode;
    hudActive_ = true;
    holdRemainingSec_ = 0.0f;
}

void RuidoController::restoreOrDissolveHud() noexcept {
    hud_.knobBank = hud_.lfoPageOpen ? KnobBankMode::Lfo : KnobBankMode::Frequency;
    for (size_t i = 4u; i < 8u; ++i) {
        if (whiteKeyHeld_[i]) {
            const auto v = static_cast<uint8_t>(i - 4u);
            triggerHud(HudMode::VoicePressure, v, v);
            return;
        }
    }
    for (size_t i = 8u; i < kNumWhiteKeys; ++i) {
        if (whiteKeyHeld_[i]) {
            hud_.snapshotSlot = static_cast<uint8_t>(i - 8u);
            triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, hud_.focusedKnob);
            return;
        }
    }
    if (latchedHudMode_ != HudMode::None) {
        hud_.mode = latchedHudMode_;
        if (hud_.lfoPageOpen && hud_.lfoVoiceFocus < kNumOscillators) {
            hud_.focusedVoice = hud_.lfoVoiceFocus;
        }
        if (latchedButton_ != ButtonId::Count) {
            hud_.lastButton = latchedButton_;
        }
        hudActive_ = true;
        holdRemainingSec_ = 0.0f;
        return;
    }
    // Momentary function released with no right-grid switch latched: start smooth dissolve
    if (hud_.mode == HudMode::VoiceFilter || hud_.mode == HudMode::GlobalFilter ||
        hud_.mode == HudMode::Amplitude || hud_.mode == HudMode::Panning ||
        hud_.mode == HudMode::ScopeFreeze || hud_.mode == HudMode::KnobRate ||
        hud_.mode == HudMode::VoicePressure || hud_.mode == HudMode::VoiceLfo ||
        hud_.mode == HudMode::GlobalLfo) {
        hud_.mode = HudMode::VoiceFrequency;
    }
    holdRemainingSec_ = 0.0f;
}

void RuidoController::syncModifierHud() noexcept {
    const HeldModifier mod = activeModifier();
    hud_.modifierHeld = (mod != HeldModifier::None);

    switch (mod) {
    case HeldModifier::FilterVoice0:
    case HeldModifier::FilterVoice1:
    case HeldModifier::FilterVoice2:
    case HeldModifier::FilterVoice3: {
        const uint8_t v = static_cast<uint8_t>(static_cast<uint8_t>(mod) -
                                               static_cast<uint8_t>(HeldModifier::FilterVoice0));
        hud_.knobBank = KnobBankMode::VoiceFilter;
        triggerHud(HudMode::VoiceFilter, v, hud_.focusedKnob);
        break;
    }
    case HeldModifier::Amplitude:
        hud_.knobBank = KnobBankMode::Amplitude;
        triggerHud(HudMode::Amplitude, hud_.focusedVoice, hud_.focusedKnob);
        break;
    case HeldModifier::Panning:
        hud_.knobBank = KnobBankMode::Panning;
        triggerHud(HudMode::Panning, hud_.focusedVoice, hud_.focusedKnob);
        break;
    case HeldModifier::GlobalFilter:
        hud_.knobBank = KnobBankMode::GlobalFilter;
        triggerHud(HudMode::GlobalFilter, 0u, hud_.focusedKnob);
        break;
    case HeldModifier::None:
        hud_.knobBank = hud_.lfoPageOpen ? KnobBankMode::Lfo : KnobBankMode::Frequency;
        if (hud_.scopeFrozen) {
            triggerHud(HudMode::ScopeFreeze, hud_.focusedVoice, hud_.focusedKnob);
        } else if (octMinusHeld_ || octPlusHeld_) {
            triggerHud(HudMode::KnobRate, hud_.focusedVoice, hud_.focusedKnob);
        } else {
            restoreOrDissolveHud();
        }
        break;
    }
}

void RuidoController::adjustVoiceFrequency(VoiceParams& vp, int32_t ticks,
                                           KnobRate rate) noexcept {
    // Strictly linear X4 quadrature scaling (96 steps = 360 deg; zero velocity acceleration):
    //   Fine (Hold OCT-) : ~63 cents/turn audio (~0.66 cents/step)
    //   Standard         : ~2.8 turns full sweep
    //   Fast (Hold OCT+) : ~1.3 turns full sweep
    const float steps = static_cast<float>(ticks);
    const float octPerStep = (rate == KnobRate::Fine) ? 0.00055f :
                             (rate == KnobRate::Fast) ? 0.06000f :
                                                        0.02850f;
    const float nextHz = vp.frequencyHz * fastExp2(steps * octPerStep);
    vp.frequencyHz = clampf(nextHz, kAudioFreqMinHz, kAudioFreqMaxHz);
}

float RuidoController::adjustFilterCutoff(float currentHz, int32_t ticks,
                                          KnobRate rate) noexcept {
    const float octPerStep = (rate == KnobRate::Fine) ? 0.00250f :
                             (rate == KnobRate::Fast) ? 0.07800f :
                                                        0.03650f;
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);
}

float RuidoController::adjustLfoRate(float currentHz, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep = (rate == KnobRate::Fine) ? 0.00250f :
                             (rate == KnobRate::Fast) ? 0.08200f :
                                                        0.03800f;
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kLfoFreqMinHz, kLfoFreqMaxHz);
}

int32_t RuidoController::stepClampedDetent(int32_t currentVal, int32_t maxVal, int32_t ticks,
                                           KnobRate rate, int32_t& accum) noexcept {
    const int32_t threshold = (rate == KnobRate::Fine) ? 8 : (rate == KnobRate::Fast) ? 2 : 4;
    if ((ticks > 0 && accum < 0) || (ticks < 0 && accum > 0)) {
        accum = 0;
    }
    accum += ticks;
    const int32_t steps = accum / threshold;
    if (steps == 0) {
        if ((currentVal <= 0 && accum < 0) || (currentVal >= maxVal && accum > 0)) {
            accum = 0;
        }
        return currentVal;
    }
    accum -= steps * threshold;
    int32_t next = currentVal + steps;
    if (next <= 0) {
        next = 0;
        if (accum < 0) {
            accum = 0;
        }
    } else if (next >= maxVal) {
        next = maxVal;
        if (accum > 0) {
            accum = 0;
        }
    }
    return next;
}

float RuidoController::adjustLinear01(float current, int32_t ticks, KnobRate rate, float lo,
                                      float hi) noexcept {
    const float stepFrac = (rate == KnobRate::Fine) ? 0.00090f :
                           (rate == KnobRate::Fast) ? 0.00800f :
                                                      0.00420f;
    const float span = hi - lo;
    const float delta = static_cast<float>(ticks) * stepFrac * span;
    return clampf(current + delta, lo, hi);
}

void RuidoController::onEncoderDelta(EncoderId encoder, int32_t ticks) noexcept {
    if (ticks == 0) {
        return;
    }
    const KnobRate rate = hud_.knobRate;

    if (encoder == EncoderId::Algorithm) {
        state_.global.circuitCoupling =
            adjustLinear01(state_.global.circuitCoupling, ticks, rate, 0.0f, 1.0f);
        triggerHud(HudMode::RailCoupling, hud_.focusedVoice, hud_.focusedKnob);
        publishState();
        return;
    }

    if (encoder == EncoderId::Select) {
        if (hud_.lfoPageOpen) {
            const int32_t nextFocus =
                stepClampedDetent(static_cast<int32_t>(hud_.lfoVoiceFocus),
                                  static_cast<int32_t>(kNumOscillators), ticks, rate,
                                  selectTickAccum_);
            hud_.lfoVoiceFocus = static_cast<uint8_t>(nextFocus);
            if (hud_.lfoVoiceFocus < kNumOscillators) {
                hud_.focusedVoice = hud_.lfoVoiceFocus;
            }
            latchedHudMode_ = currentLfoHudMode();
            if (activeModifier() == HeldModifier::None) {
                hud_.knobBank = KnobBankMode::Lfo;
                hud_.mode = latchedHudMode_;
                hudActive_ = true;
                holdRemainingSec_ = 0.0f;
            }
        } else {
            triggerHud(HudMode::UnassignedEncoder, hud_.focusedVoice, hud_.focusedKnob);
        }
        return;
    }

    if (encoder == EncoderId::Presets) {
        hud_.morphPosition = adjustLinear01(hud_.morphPosition, ticks, rate, 0.0f, 1.0f);
        triggerHud(HudMode::SnapshotMorph, hud_.focusedVoice, hud_.focusedKnob);
        return;
    }

    // Right-hand KNOB1..KNOB4 (always mapped according to left-hand black-key hold)
    const int knobIdx = static_cast<int>(encoder) - static_cast<int>(EncoderId::Knob1);
    if (knobIdx < 0 || knobIdx >= static_cast<int>(kNumOscillators)) {
        return;
    }
    const uint8_t kIdx = static_cast<uint8_t>(knobIdx);
    const HeldModifier mod = activeModifier();

    switch (mod) {
    case HeldModifier::None: {
        if (hud_.lfoPageOpen) {
            const bool editAll = (hud_.lfoVoiceFocus >= kNumOscillators);
            const size_t vStart = editAll ? 0u : static_cast<size_t>(hud_.lfoVoiceFocus);
            const size_t vEnd = editAll ? kNumOscillators : (vStart + 1u);

            if (kIdx == 0u) {
                const int32_t curTgt = static_cast<int32_t>(state_.voices[vStart].lfoTarget);
                const int32_t maxTgt = static_cast<int32_t>(LfoTarget::Count) - 1;
                const auto nextTgt = static_cast<LfoTarget>(
                    stepClampedDetent(curTgt, maxTgt, ticks, rate, targetTickAccum_));
                for (size_t i = vStart; i < vEnd; ++i) {
                    state_.voices[i].lfoTarget = nextTgt;
                }
            } else if (kIdx == 1u) {
                for (size_t i = vStart; i < vEnd; ++i) {
                    state_.voices[i].lfoShape =
                        adjustLinear01(state_.voices[i].lfoShape, ticks, rate, 0.0f, 1.0f);
                }
            } else if (kIdx == 2u) {
                for (size_t i = vStart; i < vEnd; ++i) {
                    state_.voices[i].lfoRateHz =
                        adjustLfoRate(state_.voices[i].lfoRateHz, ticks, rate);
                }
            } else {
                for (size_t i = vStart; i < vEnd; ++i) {
                    state_.voices[i].lfoDepth =
                        adjustLinear01(state_.voices[i].lfoDepth, ticks, rate, 0.0f, 1.0f);
                }
            }

            hud_.knobBank = KnobBankMode::Lfo;
            hud_.focusedKnob = kIdx;
            if (!editAll) {
                hud_.focusedVoice = hud_.lfoVoiceFocus;
            }
            latchedHudMode_ = currentLfoHudMode();
            hud_.mode = latchedHudMode_;
            hudActive_ = true;
            holdRemainingSec_ = 0.0f;
        } else {
            adjustVoiceFrequency(state_.voices[kIdx], ticks, rate);
            hud_.knobBank = KnobBankMode::Frequency;
            triggerHud(HudMode::VoiceFrequency, kIdx, kIdx);
        }
        break;
    }
    case HeldModifier::Amplitude: {
        VoiceParams& vp = state_.voices[kIdx];
        if (vp.muted) {
            if (ticks > 0) {
                vp.muted = false;
                vp.amplitude = adjustLinear01(0.0f, ticks, rate, 0.0f, 1.0f);
            }
        } else {
            vp.amplitude = adjustLinear01(vp.amplitude, ticks, rate, 0.0f, 1.0f);
        }
        hud_.knobBank = KnobBankMode::Amplitude;
        triggerHud(HudMode::Amplitude, kIdx, kIdx);
        break;
    }
    case HeldModifier::Panning: {
        state_.voices[kIdx].pan =
            adjustLinear01(state_.voices[kIdx].pan, ticks, rate, -1.0f, 1.0f);
        hud_.knobBank = KnobBankMode::Panning;
        triggerHud(HudMode::Panning, kIdx, kIdx);
        break;
    }
    case HeldModifier::FilterVoice0:
    case HeldModifier::FilterVoice1:
    case HeldModifier::FilterVoice2:
    case HeldModifier::FilterVoice3: {
        const uint8_t vIdx =
            static_cast<uint8_t>(static_cast<uint8_t>(mod) -
                                 static_cast<uint8_t>(HeldModifier::FilterVoice0));
        VoiceParams& vp = state_.voices[vIdx];
        if (kIdx == 0u) {
            vp.hpCutoffHz = adjustFilterCutoff(vp.hpCutoffHz, ticks, rate);
        } else if (kIdx == 1u) {
            vp.hpResonance = adjustLinear01(vp.hpResonance, ticks, rate, 0.0f, 1.0f);
        } else if (kIdx == 2u) {
            vp.lpCutoffHz = adjustFilterCutoff(vp.lpCutoffHz, ticks, rate);
        } else {
            vp.lpResonance = adjustLinear01(vp.lpResonance, ticks, rate, 0.0f, 1.0f);
        }
        hud_.knobBank = KnobBankMode::VoiceFilter;
        triggerHud(HudMode::VoiceFilter, vIdx, kIdx);
        break;
    }
    case HeldModifier::GlobalFilter: {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceParams& vp = state_.voices[i];
            if (kIdx == 0u) {
                vp.hpCutoffHz = adjustFilterCutoff(vp.hpCutoffHz, ticks, rate);
            } else if (kIdx == 1u) {
                vp.hpResonance = adjustLinear01(vp.hpResonance, ticks, rate, 0.0f, 1.0f);
            } else if (kIdx == 2u) {
                vp.lpCutoffHz = adjustFilterCutoff(vp.lpCutoffHz, ticks, rate);
            } else {
                vp.lpResonance = adjustLinear01(vp.lpResonance, ticks, rate, 0.0f, 1.0f);
            }
        }
        hud_.knobBank = KnobBankMode::GlobalFilter;
        triggerHud(HudMode::GlobalFilter, 0u, kIdx);
        break;
    }
    }

    publishState();
}

void RuidoController::onMasterPot(float normalizedLevel) noexcept {
    const float next = clampf(normalizedLevel, 0.0f, 1.0f);
    const bool moved = absf(next - masterPot_) > 0.002f;
    masterPot_ = next;
    state_.global.masterTrim = masterPot_;
    if (moved) {
        triggerHud(HudMode::MasterLevel, hud_.focusedVoice, hud_.focusedKnob);
    }
    publishState();
}

void RuidoController::onButtonEvent(ButtonId button, bool pressed) noexcept {
    // OCT- (Fine) and OCT+ (Fast) are strictly momentary left-hand rate modifiers
    if (button == ButtonId::OctMinus || button == ButtonId::OctPlus) {
        if (button == ButtonId::OctMinus) {
            octMinusHeld_ = pressed;
            if (pressed) {
                octMinusOrder_ = ++pressSeq_;
            }
        } else {
            octPlusHeld_ = pressed;
            if (pressed) {
                octPlusOrder_ = ++pressSeq_;
            }
        }
        hud_.knobRate = activeKnobRate();
        if (octMinusHeld_ || octPlusHeld_) {
            triggerHud(HudMode::KnobRate, hud_.focusedVoice, hud_.focusedKnob);
        } else {
            syncModifierHud();
        }
        return;
    }

    if (!pressed) {
        return;
    }

    if (button == ButtonId::Lfo) {
        selectTickAccum_ = 0;
        targetTickAccum_ = 0;
        if (!hud_.lfoPageOpen) {
            hud_.lfoPageOpen = true;
            hud_.saveArmed = false;
            if (hud_.lfoVoiceFocus < kNumOscillators) {
                hud_.focusedVoice = hud_.lfoVoiceFocus;
            }
            latchedHudMode_ = currentLfoHudMode();
            latchedButton_ = ButtonId::Lfo;
            hud_.lastButton = ButtonId::Lfo;
            syncModifierHud();
        } else {
            hud_.lfoPageOpen = false;
            latchedHudMode_ = HudMode::None;
            latchedButton_ = ButtonId::Count;
            syncModifierHud();
        }
        return;
    }

    if (button == ButtonId::PlayStop) {
        state_.global.masterRun = !state_.global.masterRun;
        if (hud_.lfoPageOpen) {
            hud_.lastButton = ButtonId::PlayStop;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, hud_.focusedKnob);
        } else {
            latchSwitchHud(HudMode::AudioOutput, ButtonId::PlayStop);
        }
        publishState();
        return;
    }

    if (button == ButtonId::Home) {
        hud_.lfoPageOpen = false;
        selectTickAccum_ = 0;
        targetTickAccum_ = 0;
        if (activeModifier() == HeldModifier::None) {
            hud_.knobBank = KnobBankMode::Frequency;
        }
        latchedHudMode_ = HudMode::None;
        latchedButton_ = ButtonId::Count;
        hudRawLevel_ = 0.0f;
        hud_.opacity = 0.0f;
        hud_.mode = HudMode::None;
        hudActive_ = false;
        holdRemainingSec_ = 0.0f;
        hud_.saveArmed = false;
        return;
    }

    if (button == ButtonId::Save) {
        hud_.lfoPageOpen = false;
        if (activeModifier() == HeldModifier::None) {
            hud_.knobBank = KnobBankMode::Frequency;
        }
        hud_.saveArmed = !hud_.saveArmed;
        if (hud_.saveArmed) {
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
        } else {
            latchedHudMode_ = HudMode::None;
            latchedButton_ = ButtonId::Count;
            holdRemainingSec_ = 0.0f;
        }
        return;
    }

    if (button == ButtonId::Sel) {
        state_.global.pitchPressureMode = !state_.global.pitchPressureMode;
        if (hud_.lfoPageOpen) {
            hud_.lastButton = ButtonId::Sel;
            triggerHud(HudMode::PressureTarget, hud_.focusedVoice, hud_.focusedKnob);
        } else {
            latchSwitchHud(HudMode::PressureTarget, ButtonId::Sel);
        }
        publishState();
        return;
    }

    // Remaining right-grid switches (FX, ENV, EDIT, GloRight, ARP, SEQ, REC)
    hud_.lfoPageOpen = false;
    if (activeModifier() == HeldModifier::None) {
        hud_.knobBank = KnobBankMode::Frequency;
    }
    latchSwitchHud(HudMode::UnassignedButton, button);
}

void RuidoController::onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys) {
        return;
    }

    blackKeyHeld_[idx] = pressed;
    if (pressed) {
        blackKeyOrder_[idx] = ++pressSeq_;
    }

    if (key == BlackKeyId::ScopeFreezeKey) {
        hud_.scopeFrozen = pressed;
        if (pressed) {
            triggerHud(HudMode::ScopeFreeze, hud_.focusedVoice, hud_.focusedKnob);
        } else {
            syncModifierHud();
        }
        return;
    }

    if (isModifierBlackKey(key)) {
        syncModifierHud();
        return;
    }

    if (!pressed) {
        return;
    }

    if (key == BlackKeyId::Pit) {
        triggerHud(HudMode::UnassignedKey, hud_.focusedVoice, hud_.focusedKnob);
    } else if (key == BlackKeyId::Mono) {
        state_.global.monoCollapse = true;
        triggerHud(HudMode::OutputRouting, hud_.focusedVoice, hud_.focusedKnob);
        publishState();
    } else if (key == BlackKeyId::Poly) {
        state_.global.monoCollapse = false;
        triggerHud(HudMode::OutputRouting, hud_.focusedVoice, hud_.focusedKnob);
        publishState();
    }
}

void RuidoController::onWhiteKeyGate(uint8_t whiteKeyIndex, bool pressed) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys) {
        return;
    }
    const bool wasPressed = whiteKeyHeld_[whiteKeyIndex];
    whiteKeyHeld_[whiteKeyIndex] = pressed;

    // White Keys 0..3 (Dot 1): Osc 1-4 Latching Gates / Mutes
    if (whiteKeyIndex < 4u) {
        if (pressed && !wasPressed) {
            state_.voices[whiteKeyIndex].muted = !state_.voices[whiteKeyIndex].muted;
            triggerHud(HudMode::VoiceGate, whiteKeyIndex, whiteKeyIndex);
            publishState();
        }
        return;
    }

    // White Keys 4..7 (Dot 2): Osc 1-4 Pressure Expression Pads (momentary)
    if (whiteKeyIndex < 8u) {
        const uint8_t vIdx = static_cast<uint8_t>(whiteKeyIndex - 4u);
        if (!pressed) {
            state_.voicePressure[vIdx] = 0.0f;
            syncModifierHud();
            publishState();
        } else {
            if (state_.voicePressure[vIdx] <= 0.0f) {
                state_.voicePressure[vIdx] = 0.25f;
            }
            triggerHud(HudMode::VoicePressure, vIdx, vIdx);
            publishState();
        }
        return;
    }

    // White Keys 8..15 (Dots 3 & 4): 8 Drone Snapshot Slots
    const uint8_t slot = static_cast<uint8_t>(whiteKeyIndex - 8u);
    if (pressed) {
        hud_.snapshotSlot = slot;
        if (hud_.saveArmed) {
            hud_.saveArmed = false;
            if (latchedHudMode_ == HudMode::SaveArmed) {
                latchedHudMode_ = HudMode::None;
                latchedButton_ = ButtonId::Count;
            }
        }
        triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, hud_.focusedKnob);
    } else if (activeModifier() != HeldModifier::None || hud_.scopeFrozen || octMinusHeld_ ||
               octPlusHeld_) {
        syncModifierHud();
    } else {
        holdRemainingSec_ = kHudHoldSec;
    }
}

void RuidoController::onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept {
    if (whiteKeyIndex >= 4u && whiteKeyIndex < 8u) {
        const uint8_t vIdx = static_cast<uint8_t>(whiteKeyIndex - 4u);
        const float clamped = clampf(pressure, 0.0f, 1.0f);
        const bool wasHeld = whiteKeyHeld_[whiteKeyIndex];
        state_.voicePressure[vIdx] = clamped;
        whiteKeyHeld_[whiteKeyIndex] = (clamped > 0.001f);
        if (clamped > 0.001f) {
            triggerHud(HudMode::VoicePressure, vIdx, vIdx);
        } else if (wasHeld) {
            syncModifierHud();
        }
        publishState();
    } else if (whiteKeyIndex >= 8u && whiteKeyIndex < kNumWhiteKeys) {
        const float clamped = clampf(pressure, 0.0f, 1.0f);
        const bool wasHeld = whiteKeyHeld_[whiteKeyIndex];
        whiteKeyHeld_[whiteKeyIndex] = (clamped > 0.001f);
        if (clamped > 0.001f) {
            hud_.snapshotSlot = static_cast<uint8_t>(whiteKeyIndex - 8u);
            hud_.morphPosition = clamped;
            triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, hud_.focusedKnob);
        } else if (wasHeld) {
            holdRemainingSec_ = kHudHoldSec;
        }
    }
}

void RuidoController::tick(float dtSec, const EngineTelemetry& telemetry) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    animPhase_ += dt;
    if (animPhase_ > 1000.0f) {
        animPhase_ -= 1000.0f;
    }

    // Soft S-curve HUD opacity envelope:
    // - Right 2x6 buttons stay latched until HOME (returning after any momentary hold or tweak)
    // - Momentary holds stay pinned while held and dissolve immediately on release
    // - Standalone knob turns & keybed taps hold readable for 1.5 s, then dissolve smoothly
    const bool modPinned =
        (activeModifier() != HeldModifier::None) || octMinusHeld_ || octPlusHeld_;
    const bool momentaryPinned = isAnyMomentaryHudHoldActive();
    const bool switchLatched = (latchedHudMode_ != HudMode::None);
    hud_.modifierHeld = modPinned;

    if (momentaryPinned) {
        holdRemainingSec_ = kHudHoldSec;
        hudActive_ = true;
    } else if (switchLatched) {
        hudActive_ = true;
    }

    if (hudActive_) {
        if (momentaryPinned) {
            hudRawLevel_ = clampf(hudRawLevel_ + dt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
        } else if (switchLatched) {
            hudRawLevel_ = clampf(hudRawLevel_ + dt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
            if (holdRemainingSec_ > 0.0f) {
                holdRemainingSec_ -= dt;
                if (holdRemainingSec_ <= 0.0f) {
                    holdRemainingSec_ = 0.0f;
                    hud_.mode = latchedHudMode_;
                    if (hud_.lfoPageOpen && activeModifier() == HeldModifier::None) {
                        hud_.knobBank = KnobBankMode::Lfo;
                        if (hud_.lfoVoiceFocus < kNumOscillators) {
                            hud_.focusedVoice = hud_.lfoVoiceFocus;
                        }
                    }
                    if (latchedButton_ != ButtonId::Count) {
                        hud_.lastButton = latchedButton_;
                    }
                }
            }
        } else {
            float remDt = dt;
            if (holdRemainingSec_ > 0.0f && hudRawLevel_ < 1.0f) {
                const float needBloom = (1.0f - hudRawLevel_) * kHudBloomSec;
                const float bloomDt = (remDt < needBloom) ? remDt : needBloom;
                hudRawLevel_ = clampf(hudRawLevel_ + bloomDt * (1.0f / kHudBloomSec), 0.0f, 1.0f);
                remDt -= bloomDt;
            }
            if (remDt > 0.0f && holdRemainingSec_ > 0.0f) {
                const float holdDt = (remDt < holdRemainingSec_) ? remDt : holdRemainingSec_;
                holdRemainingSec_ -= holdDt;
                remDt -= holdDt;
            }
            if (remDt > 0.0f && holdRemainingSec_ <= 0.0f) {
                const float decay = fastExp(-remDt * (1.0f / kHudDissolveTau));
                hudRawLevel_ = hudRawLevel_ * decay - 0.035f * remDt;
                if (hudRawLevel_ <= 0.005f) {
                    hudRawLevel_ = 0.0f;
                    hud_.mode = HudMode::None;
                    hudActive_ = false;
                }
            }
        }
        hud_.opacity = smoothstep01(hudRawLevel_);
    } else {
        hud_.opacity = 0.0f;
    }

    const float runTarget = state_.global.masterRun ? 1.0f : 0.0f;
    playStopRunSmooth_ += (runTarget - playStopRunSmooth_) * (1.0f - fastExp(-dt * 12.5f));
    if (absf(runTarget - playStopRunSmooth_) < 1.0e-4f) {
        playStopRunSmooth_ = runTarget;
    }

    updateLeds(telemetry);
}

void RuidoController::updateLeds(const EngineTelemetry& telemetry) noexcept {
    // 1. Raised Black Keys (11 monochrome white LEDs)
    // Contiguous 5-key filter family (OP1..OP4 per-voice + OP5 Global Filter): 15% rest, 100% held
    for (size_t i = 0u; i < 5u; ++i) {
        leds_.blackKeyLeds[i] = blackKeyHeld_[i] ? 1.0f : 0.15f;
    }
    // Mix / Spatial pair (OP6 Panning + GLO Amplitude): 25% landmark rest, 100% held
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op6)] =
        blackKeyHeld_[static_cast<size_t>(BlackKeyId::Op6)] ? 1.0f : 0.25f;
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::Pit)] = 0.0f;
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::Glo)] =
        blackKeyHeld_[static_cast<size_t>(BlackKeyId::Glo)] ? 1.0f : 0.25f;
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::Mono)] =
        state_.global.monoCollapse ? 1.0f : 0.15f;
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::Poly)] =
        state_.global.monoCollapse ? 0.15f : 1.0f;
    // 11th black key (Scope Freeze): 0% at rest, 100% while freezing waveform
    leds_.blackKeyLeds[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)] =
        hud_.scopeFrozen ? 1.0f : 0.0f;

    // 2. Bottom White Keys (16 monochrome white LEDs)
    const float padBaseLed =
        state_.global.pitchPressureMode ? kPadPitchBaseLed : kPadFilterBaseLed;
    float maxPadPressure = 0.0f;

    float maxLfoDepth = 0.0f;
    size_t dominantLfoVoice = 0u;

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = state_.voices[i];
        const float lfoRaw = telemetry.lfoOutput[i].load(std::memory_order_relaxed);
        const float effHz = (vp.lfoTarget == LfoTarget::Pitch && vp.lfoDepth > 0.0f) ?
                                applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoRaw) :
                                vp.frequencyHz;
        const float effAmp = (vp.lfoTarget == LfoTarget::Amplitude && vp.lfoDepth > 0.0f) ?
                                 applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoRaw) :
                                 vp.amplitude;
        const float ampDepth =
            (vp.lfoTarget == LfoTarget::Amplitude) ? clampf(vp.lfoDepth, 0.0f, 1.0f) : 0.0f;
        const float gate = telemetry.oscGateHigh[i].load(std::memory_order_relaxed);

        if (vp.lfoDepth > maxLfoDepth) {
            maxLfoDepth = vp.lfoDepth;
            dominantLfoVoice = i;
        }

        if (vp.muted) {
            leds_.whiteKeyLeds[i] = 0.10f;
        } else if (effHz < 20.0f) {
            const float peakBright = 0.60f + 0.40f * effAmp;
            leds_.whiteKeyLeds[i] = 0.12f + (peakBright - 0.12f) * clampf(gate, 0.0f, 1.0f);
        } else {
            const float restFloor = 0.60f - 0.42f * ampDepth;
            leds_.whiteKeyLeds[i] = restFloor + (1.0f - restFloor) * effAmp;
        }

        const float p = clampf(state_.voicePressure[i], 0.0f, 1.0f);
        if (p > maxPadPressure) {
            maxPadPressure = p;
        }
        leds_.whiteKeyLeds[4u + i] = padBaseLed + (1.0f - padBaseLed) * p;
    }

    float savePulse = 0.0f;
    if (hud_.saveArmed) {
        const float cyc =
            animPhase_ * 2.5f - static_cast<float>(static_cast<int32_t>(animPhase_ * 2.5f));
        const float tri = 1.0f - absf(2.0f * cyc - 1.0f);
        savePulse = 0.12f + 0.48f * smoothstep01(tri);
    }
    for (size_t s = 8u; s < 16u; ++s) {
        leds_.whiteKeyLeds[s] = whiteKeyHeld_[s] ? 0.75f : savePulse;
    }

    // 3. Panel Tactile Buttons (unused FM/seq buttons strictly 0% dark)
    leds_.buttonLeds.fill(0.0f);
    // Visually couple SEL to White Keys 5-8 via identical rest brightness & live touch flare
    leds_.buttonLeds[static_cast<size_t>(ButtonId::Sel)] =
        padBaseLed + (1.0f - padBaseLed) * maxPadPressure;
    leds_.buttonLeds[static_cast<size_t>(ButtonId::OctMinus)] =
        (hud_.knobRate == KnobRate::Fine) ? 1.0f : 0.15f;
    leds_.buttonLeds[static_cast<size_t>(ButtonId::OctPlus)] =
        (hud_.knobRate == KnobRate::Fast) ? 1.0f : 0.15f;

    leds_.buttonLeds[static_cast<size_t>(ButtonId::Home)] = 0.12f + 0.23f * hud_.opacity;
    leds_.buttonLeds[static_cast<size_t>(ButtonId::Save)] = hud_.saveArmed ? 1.0f : 0.15f;

    if (hud_.lfoPageOpen) {
        leds_.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] = 1.0f;
    } else if (maxLfoDepth > 1.0e-4f) {
        const size_t focusV =
            (hud_.lfoVoiceFocus < kNumOscillators &&
             state_.voices[hud_.lfoVoiceFocus].lfoDepth > 1.0e-4f) ?
                static_cast<size_t>(hud_.lfoVoiceFocus) :
                dominantLfoVoice;
        const float raw = telemetry.lfoOutput[focusV].load(std::memory_order_relaxed);
        const float u = 0.5f * (clampf(raw, -1.0f, 1.0f) + 1.0f);
        leds_.buttonLeds[static_cast<size_t>(ButtonId::Lfo)] =
            0.12f + (0.10f + 0.38f * clampf(maxLfoDepth, 0.0f, 1.0f)) * u;
    }

    const float cyc =
        animPhase_ * 1.2f - static_cast<float>(static_cast<int32_t>(animPhase_ * 1.2f));
    const float tri = 1.0f - absf(2.0f * cyc - 1.0f);
    const float standbyBreath = 0.15f + 0.55f * smoothstep01(tri);
    leds_.buttonLeds[static_cast<size_t>(ButtonId::PlayStop)] =
        standbyBreath + (1.0f - standbyBreath) * playStopRunSmooth_;
}

} // namespace ruido
