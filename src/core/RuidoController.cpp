#include "core/RuidoController.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/GlobalControl.h"
#include "core/LedEngine.h"

namespace ruido {

RuidoController::RuidoController() noexcept {
    reset();
}

void RuidoController::reset() noexcept {
    state_ = EngineControlState::makeDefault();
    masterPot_ = state_.global.masterTrim;
    masterPotHudAnchor_ = masterPot_;
    hud_ = {};
    leds_ = {};
    gloPreHoldVoices_ = state_.voices;
    keys_.reset();
    env_.reset();
    knobTurnedDuringRateHold_ = false;
    gloSnapshotValid_ = false;
    animPhase_ = 0.0f;
    playStopRunSmooth_ = state_.global.masterRun ? 1.0f : 0.0f;
    latchedHudMode_ = HudMode::None;
    latchedButton_ = ButtonId::Count;
    selectTickAccum_ = 0;
    targetTickAccum_ = 0;
    configTickAccum_ = 0;

    EngineTelemetry zeroTel{};
    updateLeds(zeroTel);
    publishState();
}

void RuidoController::publishState() noexcept {
    if (controlBus_ != nullptr) {
        controlBus_->publish(state_);
    }
}

KnobBankMode RuidoController::baseKnobBank() const noexcept {
    if (hud_.configPageOpen) {
        return KnobBankMode::None;
    }
    if (hud_.editPageOpen) {
        return KnobBankMode::Amplitude;
    }
    return hud_.lfoPageOpen ? KnobBankMode::Lfo : KnobBankMode::Frequency;
}

void RuidoController::triggerHud(HudMode mode, uint8_t voiceIndex, uint8_t knobIndex) noexcept {
    hud_.mode = mode;
    hud_.focusedVoice = static_cast<uint8_t>(voiceIndex & 3u);
    hud_.focusedKnob = (knobIndex < kNumOscillators) ? knobIndex : kNoFocusedKnob;
    env_.triggerToast();
}

void RuidoController::latchSwitchHud(HudMode mode, ButtonId button) noexcept {
    latchedHudMode_ = mode;
    latchedButton_ = button;
    if (button != ButtonId::Count) {
        hud_.lastButton = button;
    }
    hud_.focusedKnob = kNoFocusedKnob;
    hud_.mode = mode;
    env_.latchPage();
}

void RuidoController::restoreOrDissolveHud() noexcept {
    hud_.knobBank = baseKnobBank();
    for (size_t i = 4u; i < 8u; ++i) {
        if (keys_.whiteKeyHeld[i]) {
            const auto v = static_cast<uint8_t>(i - 4u);
            triggerHud(HudMode::VoicePressure, v, v);
            return;
        }
    }
    for (size_t i = 8u; i < kNumWhiteKeys; ++i) {
        if (keys_.whiteKeyHeld[i]) {
            hud_.snapshotSlot = static_cast<uint8_t>(i - 8u);
            triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, kNoFocusedKnob);
            return;
        }
    }
    if (latchedHudMode_ != HudMode::None) {
        hud_.mode = latchedHudMode_;
        if (hud_.lfoPageOpen) {
            hud_.focusedVoice = static_cast<uint8_t>(hud_.lfoVoiceFocus & 3u);
        }
        if (latchedButton_ != ButtonId::Count) {
            hud_.lastButton = latchedButton_;
        }
        hud_.focusedKnob = kNoFocusedKnob;
        env_.latchPage();
        return;
    }
    if (env_.beginFadeOut()) {
        hud_.mode = HudMode::None;
        hud_.opacity = 0.0f;
    }
}

void RuidoController::syncModifierHud() noexcept {
    const HeldModifier mod = keys_.activeModifier();
    hud_.modifierHeld = (mod != HeldModifier::None) || hud_.rateHoldActive;

    switch (mod) {
    case HeldModifier::FilterVoice0:
    case HeldModifier::FilterVoice1:
    case HeldModifier::FilterVoice2:
    case HeldModifier::FilterVoice3: {
        const uint8_t v = static_cast<uint8_t>(static_cast<uint8_t>(mod) -
                                               static_cast<uint8_t>(HeldModifier::FilterVoice0));
        hud_.knobBank = KnobBankMode::VoiceFilter;
        triggerHud(HudMode::VoiceFilter, v, kNoFocusedKnob);
        break;
    }
    case HeldModifier::Panning:
        hud_.knobBank = KnobBankMode::Panning;
        triggerHud(HudMode::Panning, hud_.focusedVoice, kNoFocusedKnob);
        break;
    case HeldModifier::Global:
        hud_.knobBank = knobBankForGlobalTarget(hud_.globalTarget);
        triggerHud(hudModeForGlobalTarget(hud_.globalTarget), hud_.focusedVoice, kNoFocusedKnob);
        break;
    case HeldModifier::None:
        hud_.knobBank = baseKnobBank();
        if (hud_.scopeFrozen) {
            triggerHud(HudMode::ScopeFreeze, hud_.focusedVoice, kNoFocusedKnob);
        } else if (hud_.rateHoldActive) {
            if (hud_.mode == HudMode::None) {
                hud_.mode = HudMode::KnobRate;
            }
            hud_.focusedKnob = kNoFocusedKnob;
            env_.active = true;
            if (!knobTurnedDuringRateHold_) {
                env_.holdRemainingSec = 0.0f;
            }
        } else {
            restoreOrDissolveHud();
        }
        break;
    }
}

void RuidoController::onEncoderDelta(EncoderId encoder, int32_t ticks) noexcept {
    if (ticks == 0) {
        return;
    }
    const KnobRate rate = hud_.knobRate;
    const HeldModifier mod = keys_.activeModifier();

    if (hud_.rateHoldActive) {
        knobTurnedDuringRateHold_ = true;
    }

    if (encoder == EncoderId::Algorithm) {
        state_.global.circuitCoupling =
            adjustLinear01(state_.global.circuitCoupling, ticks, rate, 0.0f, 1.0f);
        triggerHud(HudMode::RailCoupling, hud_.focusedVoice, kNoFocusedKnob);
        publishState();
        return;
    }

    if (encoder == EncoderId::Select) {
        if (mod == HeldModifier::Global) {
            const int32_t maxTgt = static_cast<int32_t>(GlobalTarget::Count) - 1;
            const int32_t nextTgt = stepClampedDetent(static_cast<int32_t>(hud_.globalTarget),
                                                      maxTgt, ticks, rate, selectTickAccum_);
            hud_.globalTarget = static_cast<GlobalTarget>(nextTgt);
            targetTickAccum_ = 0;
            hud_.knobBank = knobBankForGlobalTarget(hud_.globalTarget);
            triggerHud(hudModeForGlobalTarget(hud_.globalTarget), hud_.focusedVoice,
                       kNoFocusedKnob);
            return;
        }
        if (mod == HeldModifier::None && hud_.configPageOpen) {
            const int32_t maxItem = static_cast<int32_t>(kNumGlobalConfigItems) - 1;
            const int32_t nextRow = stepClampedDetent(static_cast<int32_t>(hud_.configCursor),
                                                      maxItem, ticks, rate, selectTickAccum_);
            hud_.configCursor = static_cast<uint8_t>(nextRow);
            configTickAccum_ = 0;
            hud_.focusedKnob = kNoFocusedKnob;
            hud_.knobBank = KnobBankMode::None;
            latchedHudMode_ = HudMode::GlobalConfig;
            hud_.mode = HudMode::GlobalConfig;
            env_.latchPage();
            return;
        }
        if (mod == HeldModifier::None && hud_.lfoPageOpen) {
            const int32_t maxVoice = static_cast<int32_t>(kNumOscillators) - 1;
            const int32_t nextFocus = stepClampedDetent(static_cast<int32_t>(hud_.lfoVoiceFocus),
                                                        maxVoice, ticks, rate, selectTickAccum_);
            hud_.lfoVoiceFocus = static_cast<uint8_t>(nextFocus);
            hud_.focusedVoice = hud_.lfoVoiceFocus;
            targetTickAccum_ = 0;
            hud_.focusedKnob = kNoFocusedKnob;
            latchedHudMode_ = HudMode::VoiceLfo;
            hud_.knobBank = KnobBankMode::Lfo;
            hud_.mode = HudMode::VoiceLfo;
            env_.latchPage();
            return;
        }
        if (mod == HeldModifier::None && hud_.editPageOpen) {
            applyGlobalKnobDelta(GlobalTarget::Amp, 0u, ticks, rate, targetTickAccum_, state_);
            hud_.knobBank = KnobBankMode::Amplitude;
            hud_.focusedKnob = kNoFocusedKnob;
            latchedHudMode_ = HudMode::Amplitude;
            hud_.mode = HudMode::Amplitude;
            env_.latchPage();
            publishState();
            return;
        }
        hud_.lastEncoder = EncoderId::Select;
        triggerHud(HudMode::UnassignedEncoder, hud_.focusedVoice, kNoFocusedKnob);
        return;
    }

    if (encoder == EncoderId::Presets) {
        hud_.lastEncoder = EncoderId::Presets;
        triggerHud(HudMode::UnassignedEncoder, hud_.focusedVoice, kNoFocusedKnob);
        return;
    }

    const int knobIdx = static_cast<int>(encoder) - static_cast<int>(EncoderId::Knob1);
    if (knobIdx < 0 || knobIdx >= static_cast<int>(kNumOscillators)) {
        return;
    }
    const uint8_t kIdx = static_cast<uint8_t>(knobIdx);

    switch (mod) {
    case HeldModifier::None: {
        if (hud_.configPageOpen) {
            const auto item = static_cast<ConfigItemId>(
                (hud_.configCursor < kNumGlobalConfigItems) ? hud_.configCursor : 0u);
            const int32_t curVal = globalConfigGetIndex(item, hud_.config, state_.global);
            const int32_t maxVal = globalConfigMaxIndex(item);
            const int32_t nextVal =
                stepClampedDetent(curVal, maxVal, ticks, rate, configTickAccum_);
            if (globalConfigSetIndex(item, nextVal, hud_.config, state_.global)) {
                publishState();
            }
            hud_.focusedKnob = kNoFocusedKnob;
            hud_.knobBank = KnobBankMode::None;
            latchedHudMode_ = HudMode::GlobalConfig;
            hud_.mode = HudMode::GlobalConfig;
            env_.latchPage();
            return;
        }
        if (hud_.lfoPageOpen) {
            const size_t vIdx = static_cast<size_t>(hud_.lfoVoiceFocus & 3u);
            VoiceParams& vp = state_.voices[vIdx];
            if (kIdx == 0u) {
                const int32_t curTgt = static_cast<int32_t>(vp.lfoTarget);
                const int32_t maxTgt = static_cast<int32_t>(LfoTarget::Count) - 1;
                vp.lfoTarget = static_cast<LfoTarget>(
                    stepClampedDetent(curTgt, maxTgt, ticks, rate, targetTickAccum_));
            } else if (kIdx == 1u) {
                vp.lfoShape = adjustLinear01(vp.lfoShape, ticks, rate, 0.0f, 1.0f);
            } else if (kIdx == 2u) {
                vp.lfoRateHz = adjustLfoRate(vp.lfoRateHz, ticks, rate);
            } else {
                vp.lfoDepth = adjustLinear01(vp.lfoDepth, ticks, rate, 0.0f, 1.0f);
            }
            hud_.knobBank = KnobBankMode::Lfo;
            hud_.focusedKnob = kIdx;
            hud_.focusedVoice = static_cast<uint8_t>(vIdx);
            latchedHudMode_ = HudMode::VoiceLfo;
            hud_.mode = HudMode::VoiceLfo;
            env_.latchPage();
        } else if (hud_.editPageOpen) {
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
            hud_.focusedKnob = kIdx;
            hud_.focusedVoice = kIdx;
            latchedHudMode_ = HudMode::Amplitude;
            hud_.mode = HudMode::Amplitude;
            env_.latchPage();
        } else {
            adjustVoiceFrequency(state_.voices[kIdx], ticks, rate);
            hud_.knobBank = KnobBankMode::Frequency;
            triggerHud(HudMode::VoiceFrequency, kIdx, kIdx);
        }
        break;
    }
    case HeldModifier::Panning: {
        state_.voices[kIdx].pan = adjustLinear01(state_.voices[kIdx].pan, ticks, rate, -1.0f, 1.0f);
        hud_.knobBank = KnobBankMode::Panning;
        triggerHud(HudMode::Panning, kIdx, kIdx);
        break;
    }
    case HeldModifier::FilterVoice0:
    case HeldModifier::FilterVoice1:
    case HeldModifier::FilterVoice2:
    case HeldModifier::FilterVoice3: {
        const uint8_t vIdx = static_cast<uint8_t>(static_cast<uint8_t>(mod) -
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
    case HeldModifier::Global: {
        applyGlobalKnobDelta(hud_.globalTarget, kIdx, ticks, rate, targetTickAccum_, state_);
        hud_.knobBank = knobBankForGlobalTarget(hud_.globalTarget);
        triggerHud(hudModeForGlobalTarget(hud_.globalTarget), hud_.focusedVoice, kIdx);
        break;
    }
    }

    publishState();
}

void RuidoController::onMasterPot(float normalizedLevel) noexcept {
    const float next = clampf(normalizedLevel, 0.0f, 1.0f);
    masterPot_ = next;
    state_.global.masterTrim = masterPot_;
    if (absf(next - masterPotHudAnchor_) > 0.002f) {
        masterPotHudAnchor_ = next;
        if (hud_.rateHoldActive) {
            knobTurnedDuringRateHold_ = true;
        }
        triggerHud(HudMode::MasterLevel, hud_.focusedVoice, kNoFocusedKnob);
    }
    publishState();
}

void RuidoController::onButtonEvent(ButtonId button, bool pressed) noexcept {
    if (button == ButtonId::OctMinus || button == ButtonId::OctPlus) {
        const bool wasRateHeld = keys_.setRateButton(button, pressed);
        const bool nowRateHeld = keys_.isRateHeld();
        hud_.knobRate = keys_.activeKnobRate();
        hud_.rateHoldActive = nowRateHeld;
        hud_.modifierHeld = (keys_.activeModifier() != HeldModifier::None) || nowRateHeld;

        if (nowRateHeld) {
            if (!wasRateHeld) {
                knobTurnedDuringRateHold_ = false;
                const bool unassignedToast = (hud_.mode == HudMode::UnassignedEncoder) ||
                                             (hud_.mode == HudMode::UnassignedKey);
                if ((env_.holdRemainingSec <= 0.0f || unassignedToast) &&
                    latchedHudMode_ == HudMode::None &&
                    keys_.activeModifier() == HeldModifier::None && !hud_.scopeFrozen) {
                    hud_.mode = HudMode::KnobRate;
                    hud_.focusedKnob = kNoFocusedKnob;
                }
            }
            if (hud_.mode == HudMode::None) {
                hud_.mode = HudMode::KnobRate;
                hud_.focusedKnob = kNoFocusedKnob;
            }
            env_.active = true;
        } else {
            const bool hadKnobToast = knobTurnedDuringRateHold_;
            knobTurnedDuringRateHold_ = false;
            if (keys_.activeModifier() != HeldModifier::None || hud_.scopeFrozen) {
                syncModifierHud();
            } else if (hadKnobToast && env_.holdRemainingSec > 0.0f) {
                hud_.knobBank = baseKnobBank();
            } else {
                restoreOrDissolveHud();
            }
        }
        return;
    }

    if (!pressed) {
        return;
    }

    if (button == ButtonId::Lfo || button == ButtonId::Edit || button == ButtonId::GloRight) {
        selectTickAccum_ = 0;
        targetTickAccum_ = 0;
        configTickAccum_ = 0;
        hud_.focusedKnob = kNoFocusedKnob;
        const bool isLfo = (button == ButtonId::Lfo);
        const bool isEdit = (button == ButtonId::Edit);
        const bool opening =
            isLfo ? !hud_.lfoPageOpen : (isEdit ? !hud_.editPageOpen : !hud_.configPageOpen);
        hud_.lfoPageOpen = isLfo && opening;
        hud_.editPageOpen = isEdit && opening;
        hud_.configPageOpen = (!isLfo && !isEdit) && opening;
        if (opening) {
            hud_.saveArmed = false;
            if (isLfo) {
                hud_.focusedVoice = static_cast<uint8_t>(hud_.lfoVoiceFocus & 3u);
            }
            latchedHudMode_ =
                isLfo ? HudMode::VoiceLfo : (isEdit ? HudMode::Amplitude : HudMode::GlobalConfig);
            latchedButton_ = button;
            hud_.lastButton = button;
        } else {
            latchedHudMode_ = HudMode::None;
            latchedButton_ = ButtonId::Count;
        }
        syncModifierHud();
        return;
    }

    if (button == ButtonId::PlayStop || button == ButtonId::Sel) {
        const bool isPlay = (button == ButtonId::PlayStop);
        if (isPlay) {
            state_.global.masterRun = !state_.global.masterRun;
        } else {
            state_.global.pitchPressureMode = !state_.global.pitchPressureMode;
        }
        const HudMode mode = isPlay ? HudMode::AudioOutput : HudMode::PressureTarget;
        if (hud_.lfoPageOpen || hud_.editPageOpen || hud_.configPageOpen) {
            hud_.lastButton = button;
            triggerHud(mode, hud_.focusedVoice, kNoFocusedKnob);
        } else {
            hud_.saveArmed = false;
            latchSwitchHud(mode, button);
        }
        publishState();
        return;
    }

    if (button == ButtonId::Home) {
        hud_.lfoPageOpen = false;
        hud_.editPageOpen = false;
        hud_.configPageOpen = false;
        hud_.saveArmed = false;
        latchedHudMode_ = HudMode::None;
        latchedButton_ = ButtonId::Count;
        selectTickAccum_ = 0;
        targetTickAccum_ = 0;
        configTickAccum_ = 0;
        hud_.focusedKnob = kNoFocusedKnob;
        knobTurnedDuringRateHold_ = false;
        if (keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen)) {
            if (keys_.activeModifier() == HeldModifier::None && !hud_.scopeFrozen &&
                hud_.rateHoldActive) {
                hud_.knobBank = KnobBankMode::Frequency;
                hud_.mode = HudMode::KnobRate;
                env_.holdRemainingSec = 0.0f;
                env_.active = true;
            } else {
                syncModifierHud();
            }
        } else {
            hud_.knobBank = KnobBankMode::Frequency;
            if (env_.beginFadeOut()) {
                hud_.mode = HudMode::None;
                hud_.opacity = 0.0f;
            }
        }
        return;
    }

    if (button == ButtonId::Save) {
        hud_.lfoPageOpen = false;
        hud_.editPageOpen = false;
        hud_.configPageOpen = false;
        if (keys_.activeModifier() == HeldModifier::None) {
            hud_.knobBank = KnobBankMode::Frequency;
        }
        hud_.saveArmed = !hud_.saveArmed;
        if (hud_.saveArmed) {
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
        } else {
            latchedHudMode_ = HudMode::None;
            latchedButton_ = ButtonId::Count;
            triggerHud(HudMode::SaveArmed, hud_.focusedVoice, kNoFocusedKnob);
        }
        return;
    }

    hud_.lfoPageOpen = false;
    hud_.editPageOpen = false;
    hud_.configPageOpen = false;
    hud_.saveArmed = false;
    if (keys_.activeModifier() == HeldModifier::None) {
        hud_.knobBank = KnobBankMode::Frequency;
    }
    latchSwitchHud(HudMode::UnassignedButton, button);
}

void RuidoController::onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys) {
        return;
    }
    const bool wasHeld = keys_.setBlackKey(key, pressed);

    if (key == BlackKeyId::Glo) {
        selectTickAccum_ = 0;
        targetTickAccum_ = 0;
        if (pressed && !wasHeld) {
            gloPreHoldVoices_ = state_.voices;
            gloSnapshotValid_ = true;
        } else if (!pressed && wasHeld) {
            if (gloSnapshotValid_ && hud_.config.gloRelease == GloReleaseMode::SnapBack) {
                state_.voices = gloPreHoldVoices_;
                publishState();
            }
            gloSnapshotValid_ = false;
        }
    }

    if (key == BlackKeyId::ScopeFreezeKey) {
        hud_.scopeFrozen = pressed;
        if (pressed) {
            triggerHud(HudMode::ScopeFreeze, hud_.focusedVoice, kNoFocusedKnob);
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

    if (key == BlackKeyId::Pit || key == BlackKeyId::Op5) {
        hud_.lastBlackKey = key;
        triggerHud(HudMode::UnassignedKey, hud_.focusedVoice, kNoFocusedKnob);
    } else if (key == BlackKeyId::Mono || key == BlackKeyId::Poly) {
        state_.global.monoCollapse = (key == BlackKeyId::Mono);
        triggerHud(HudMode::OutputRouting, hud_.focusedVoice, kNoFocusedKnob);
        publishState();
    }
}

void RuidoController::onWhiteKeyGate(uint8_t whiteKeyIndex, bool pressed) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys) {
        return;
    }
    const bool wasPressed = keys_.whiteKeyHeld[whiteKeyIndex];
    keys_.whiteKeyHeld[whiteKeyIndex] = pressed;

    if (whiteKeyIndex < 4u) {
        if (pressed && !wasPressed) {
            state_.voices[whiteKeyIndex].muted = !state_.voices[whiteKeyIndex].muted;
            triggerHud(HudMode::VoiceGate, whiteKeyIndex, whiteKeyIndex);
            publishState();
        }
        return;
    }

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
        triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, kNoFocusedKnob);
    } else if (keys_.activeModifier() != HeldModifier::None || hud_.scopeFrozen ||
               hud_.rateHoldActive) {
        syncModifierHud();
    } else {
        env_.holdRemainingSec = kHudToastHoldSec;
    }
}

void RuidoController::onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept {
    if (whiteKeyIndex >= 4u && whiteKeyIndex < 8u) {
        const uint8_t vIdx = static_cast<uint8_t>(whiteKeyIndex - 4u);
        const float clamped = clampf(pressure, 0.0f, 1.0f);
        const bool wasHeld = keys_.whiteKeyHeld[whiteKeyIndex];
        state_.voicePressure[vIdx] = clamped;
        keys_.whiteKeyHeld[whiteKeyIndex] = (clamped > 0.001f);
        if (clamped > 0.001f) {
            triggerHud(HudMode::VoicePressure, vIdx, vIdx);
        } else if (wasHeld) {
            syncModifierHud();
        }
        publishState();
    } else if (whiteKeyIndex >= 8u && whiteKeyIndex < kNumWhiteKeys) {
        const float clamped = clampf(pressure, 0.0f, 1.0f);
        const bool wasHeld = keys_.whiteKeyHeld[whiteKeyIndex];
        keys_.whiteKeyHeld[whiteKeyIndex] = (clamped > 0.001f);
        if (clamped > 0.001f) {
            hud_.snapshotSlot = static_cast<uint8_t>(whiteKeyIndex - 8u);
            hud_.morphPosition = clamped;
            triggerHud(HudMode::SnapshotSlot, hud_.focusedVoice, kNoFocusedKnob);
        } else if (wasHeld) {
            env_.holdRemainingSec = kHudToastHoldSec;
        }
    }
}

void RuidoController::tick(float dtSec, const EngineTelemetry& telemetry) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    animPhase_ += dt;
    if (animPhase_ > 1000.0f) {
        animPhase_ -= 1000.0f;
    }

    const bool modPinned = (keys_.activeModifier() != HeldModifier::None) || hud_.rateHoldActive;
    const bool momentaryPinned = keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen);
    const bool switchLatched = (latchedHudMode_ != HudMode::None);
    hud_.modifierHeld = modPinned;

    bool toastExpired = false;
    bool dissolvedToIdle = false;
    env_.step(dt, momentaryPinned, switchLatched, toastExpired, dissolvedToIdle, hud_.opacity);
    if (toastExpired) {
        hud_.mode = latchedHudMode_;
        hud_.focusedKnob = kNoFocusedKnob;
        if (keys_.activeModifier() == HeldModifier::None) {
            hud_.knobBank = baseKnobBank();
            if (hud_.lfoPageOpen) {
                hud_.focusedVoice = static_cast<uint8_t>(hud_.lfoVoiceFocus & 3u);
            }
        }
        if (latchedButton_ != ButtonId::Count) {
            hud_.lastButton = latchedButton_;
        }
    }
    if (dissolvedToIdle) {
        hud_.mode = HudMode::None;
        hud_.focusedKnob = kNoFocusedKnob;
    }

    const float runTarget = state_.global.masterRun ? 1.0f : 0.0f;
    playStopRunSmooth_ += (runTarget - playStopRunSmooth_) * (1.0f - fastExp(-dt * 12.5f));
    if (absf(runTarget - playStopRunSmooth_) < 1.0e-4f) {
        playStopRunSmooth_ = runTarget;
    }

    updateLeds(telemetry);
}

void RuidoController::updateLeds(const EngineTelemetry& telemetry) noexcept {
    const LedInputContext ctx{
        state_,
        hud_,
        telemetry,
        keys_.blackKeyHeld,
        keys_.whiteKeyHeld,
        animPhase_,
        playStopRunSmooth_,
    };
    computePanelAndKeybedLeds(ctx, leds_);
}

} // namespace ruido
