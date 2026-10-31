#include "core/RuidoController.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/GlobalControl.h"
#include "core/LedEngine.h"
#include "core/LfoMath.h"

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
    ledSmooth_.reset(state_);
    knobTurnedDuringRateHold_ = false;
    gloSnapshotValid_ = false;
    dismissedByHome_ = false;
    latchedHudMode_ = HudMode::None;
    latchedButton_ = ButtonId::Count;
    selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;

    EngineTelemetry zeroTel{};
    updateLeds(0.0f, zeroTel);
    publishState();
}

void RuidoController::publishState() noexcept {
    if (controlBus_ != nullptr) {
        controlBus_->publish(state_);
    }
}

KnobBankMode RuidoController::baseKnobBank() const noexcept {
    return hud_.configPageOpen ? KnobBankMode::None
           : hud_.fxPageOpen   ? KnobBankMode::VoiceFilter
           : hud_.panPageOpen  ? KnobBankMode::Panning
           : hud_.editPageOpen ? knobBankForEditTarget(hud_.editTarget)
           : hud_.lfoPageOpen  ? KnobBankMode::Lfo
                               : KnobBankMode::Frequency;
}

void RuidoController::closeAllPages() noexcept {
    hud_.freqPageOpen = hud_.fxPageOpen = hud_.panPageOpen = false;
    hud_.lfoPageOpen = hud_.editPageOpen = hud_.configPageOpen = false;
}

void RuidoController::triggerHud(HudMode mode, uint8_t voiceIndex, uint8_t knobIndex) noexcept {
    dismissedByHome_ = false;
    hud_.mode = mode;
    hud_.focusedVoice = static_cast<uint8_t>(voiceIndex & 3u);
    hud_.focusedKnob = (knobIndex < kNumOscillators) ? knobIndex : kNoFocusedKnob;
    env_.triggerToast();
}

void RuidoController::latchSwitchHud(HudMode mode, ButtonId button) noexcept {
    dismissedByHome_ = false;
    latchedHudMode_ = mode;
    latchedButton_ = button;
    if (button != ButtonId::Count) {
        hud_.lastButton = button;
    }
    hud_.focusedKnob = kNoFocusedKnob;
    hud_.mode = mode;
    env_.latchPage();
}

void RuidoController::latchKnobEdit(KnobBankMode bank, HudMode mode, uint8_t knobIdx,
                                    uint8_t voiceIdx) noexcept {
    dismissedByHome_ = false;
    hud_.knobBank = bank;
    hud_.focusedKnob = knobIdx;
    hud_.focusedVoice = static_cast<uint8_t>(voiceIdx & 3u);
    latchedHudMode_ = mode;
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
        if (hud_.fxPageOpen) {
            hud_.focusedVoice = static_cast<uint8_t>(hud_.fxVoiceFocus & 3u);
        } else if (hud_.lfoPageOpen) {
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
        if (mod == HeldModifier::None && hud_.configPageOpen) {
            const auto item = static_cast<ConfigItemId>(
                (hud_.configCursor < kNumGlobalConfigItems) ? hud_.configCursor : 0u);
            const int32_t curVal = globalConfigGetIndex(item, hud_.config, state_.global);
            const int32_t maxVal = globalConfigMaxIndex(item);
            const int32_t nextVal =
                stepClampedDetent(curVal, maxVal, ticks, rate, configTickAccum_);
            if (globalConfigSetIndex(item, nextVal, hud_.config, state_.global)) {
                publishState();
            }
            hud_.knobBank = KnobBankMode::None;
            latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
            return;
        }
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
            hud_.knobBank = KnobBankMode::None;
            latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
            return;
        }
        if (mod == HeldModifier::None && (hud_.fxPageOpen || hud_.lfoPageOpen)) {
            const bool isFx = hud_.fxPageOpen;
            uint8_t& focusRef = isFx ? hud_.fxVoiceFocus : hud_.lfoVoiceFocus;
            const int32_t maxFocus =
                static_cast<int32_t>(isFx ? kNumOscillators : kNumVoiceLfos) - 1;
            focusRef = static_cast<uint8_t>(stepClampedDetent(
                static_cast<int32_t>(focusRef), maxFocus, ticks, rate, selectTickAccum_));
            hud_.focusedVoice = isFx ? focusRef : lfoFocusVoice(focusRef);
            targetTickAccum_ = 0;
            hud_.knobBank = isFx ? KnobBankMode::VoiceFilter : KnobBankMode::Lfo;
            latchSwitchHud(isFx ? HudMode::VoiceFilter : HudMode::VoiceLfo,
                           isFx ? ButtonId::Fx : ButtonId::Lfo);
            return;
        }
        if (mod == HeldModifier::None && hud_.editPageOpen) {
            const int32_t maxEdit = static_cast<int32_t>(EditTarget::Count) - 1;
            hud_.editTarget = static_cast<EditTarget>(stepClampedDetent(
                static_cast<int32_t>(hud_.editTarget), maxEdit, ticks, rate, selectTickAccum_));
            targetTickAccum_ = 0;
            hud_.knobBank = knobBankForEditTarget(hud_.editTarget);
            latchSwitchHud(hudModeForEditTarget(hud_.editTarget), ButtonId::Edit);
            return;
        }
        if (mod == HeldModifier::None && hud_.panPageOpen) {
            const int32_t curIdx =
                globalConfigGetIndex(ConfigItemId::OutputRouting, hud_.config, state_.global);
            const int32_t nextIdx =
                stepClampedDetent(curIdx, globalConfigMaxIndex(ConfigItemId::OutputRouting), ticks,
                                  rate, selectTickAccum_);
            if (globalConfigSetIndex(ConfigItemId::OutputRouting, nextIdx, hud_.config,
                                     state_.global)) {
                publishState();
            }
            hud_.knobBank = KnobBankMode::Panning;
            triggerHud(HudMode::OutputRouting, hud_.focusedVoice, kNoFocusedKnob);
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

    if (mod == HeldModifier::Global) {
        if (hud_.globalTarget == GlobalTarget::Amp && kIdx == 3u && state_.global.monoCollapse) {
            return;
        }
        applyGlobalKnobDelta(hud_.globalTarget, kIdx, ticks, rate, targetTickAccum_, state_);
        publishState();
        hud_.knobBank = knobBankForGlobalTarget(hud_.globalTarget);
        triggerHud(hudModeForGlobalTarget(hud_.globalTarget), hud_.focusedVoice, kIdx);
        return;
    }

    if (hud_.fxPageOpen) {
        const size_t vIdx = static_cast<size_t>(hud_.fxVoiceFocus & 3u);
        applyVoiceFilterKnobDelta(state_.voices[vIdx], kIdx, ticks, rate);
        latchKnobEdit(KnobBankMode::VoiceFilter, HudMode::VoiceFilter, kIdx,
                      static_cast<uint8_t>(vIdx));
    } else if (hud_.lfoPageOpen) {
        const uint8_t slotFocus = static_cast<uint8_t>(hud_.lfoVoiceFocus & 7u);
        const size_t vIdx = lfoFocusVoice(slotFocus);
        applyVoiceLfoKnobDelta(state_.voices[vIdx], lfoFocusSub(slotFocus), kIdx, ticks, rate,
                               targetTickAccum_);
        latchKnobEdit(KnobBankMode::Lfo, HudMode::VoiceLfo, kIdx, static_cast<uint8_t>(vIdx));
    } else if (hud_.panPageOpen) {
        if (state_.global.monoCollapse) {
            return;
        }
        state_.voices[kIdx].pan = adjustLinear01(state_.voices[kIdx].pan, ticks, rate, -1.0f, 1.0f);
        latchKnobEdit(KnobBankMode::Panning, HudMode::Panning, kIdx, kIdx);
    } else if (hud_.editPageOpen) {
        applyEditKnobDelta(hud_.editTarget, state_.voices[kIdx], ticks, rate);
        latchKnobEdit(knobBankForEditTarget(hud_.editTarget), hudModeForEditTarget(hud_.editTarget),
                      kIdx, kIdx);
    } else {
        adjustVoiceFrequency(state_.voices[kIdx], ticks, rate);
        if (hud_.freqPageOpen) {
            latchKnobEdit(KnobBankMode::Frequency, HudMode::VoiceFrequency, kIdx, kIdx);
        } else {
            hud_.knobBank = KnobBankMode::Frequency;
            triggerHud(HudMode::VoiceFrequency, kIdx, kIdx);
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
            dismissedByHome_ = false;
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

    if (button == ButtonId::Fx || button == ButtonId::Sel || button == ButtonId::Lfo ||
        button == ButtonId::Edit || button == ButtonId::GloRight) {
        selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
        hud_.focusedKnob = kNoFocusedKnob;
        const bool isFx = (button == ButtonId::Fx);
        const bool isPan = (button == ButtonId::Sel);
        const bool isLfo = (button == ButtonId::Lfo);
        const bool isEdit = (button == ButtonId::Edit);
        const bool opening = isFx     ? !hud_.fxPageOpen
                             : isPan  ? !hud_.panPageOpen
                             : isLfo  ? !hud_.lfoPageOpen
                             : isEdit ? !hud_.editPageOpen
                                      : !hud_.configPageOpen;
        closeAllPages();
        if (opening) {
            dismissedByHome_ = false;
            hud_.fxPageOpen = isFx;
            hud_.panPageOpen = isPan;
            hud_.lfoPageOpen = isLfo;
            hud_.editPageOpen = isEdit;
            hud_.configPageOpen = (!isFx && !isPan && !isLfo && !isEdit);
            hud_.saveArmed = false;
            if (isFx) {
                hud_.focusedVoice = static_cast<uint8_t>(hud_.fxVoiceFocus & 3u);
                latchedHudMode_ = HudMode::VoiceFilter;
            } else if (isPan) {
                latchedHudMode_ = HudMode::Panning;
            } else if (isLfo) {
                hud_.focusedVoice = lfoFocusVoice(hud_.lfoVoiceFocus);
                latchedHudMode_ = HudMode::VoiceLfo;
            } else {
                latchedHudMode_ =
                    isEdit ? hudModeForEditTarget(hud_.editTarget) : HudMode::GlobalConfig;
            }
            latchedButton_ = button;
            hud_.lastButton = button;
        } else {
            latchedHudMode_ = HudMode::None;
            latchedButton_ = ButtonId::Count;
        }
        syncModifierHud();
        return;
    }

    if (button == ButtonId::PlayStop) {
        state_.global.masterRun = !state_.global.masterRun;
        if (hud_.freqPageOpen || hud_.fxPageOpen || hud_.panPageOpen || hud_.lfoPageOpen ||
            hud_.editPageOpen || hud_.configPageOpen) {
            hud_.lastButton = button;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, kNoFocusedKnob);
        } else {
            hud_.saveArmed = false;
            latchSwitchHud(HudMode::AudioOutput, button);
        }
        publishState();
        return;
    }

    if (button == ButtonId::Home) {
        const bool hudActive = (latchedHudMode_ != HudMode::None) || hud_.freqPageOpen ||
                               hud_.fxPageOpen || hud_.panPageOpen || hud_.lfoPageOpen ||
                               hud_.editPageOpen || hud_.configPageOpen || hud_.saveArmed ||
                               keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen) ||
                               (hud_.mode != HudMode::None && !dismissedByHome_);
        closeAllPages();
        hud_.saveArmed = false;
        selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
        hud_.focusedKnob = kNoFocusedKnob;
        knobTurnedDuringRateHold_ = false;
        if (!hudActive) {
            hud_.freqPageOpen = true;
            hud_.knobBank = KnobBankMode::Frequency;
            latchSwitchHud(HudMode::VoiceFrequency, ButtonId::Home);
            return;
        }
        dismissedByHome_ = true;
        latchedHudMode_ = HudMode::None;
        latchedButton_ = ButtonId::Count;
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
        closeAllPages();
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

    closeAllPages();
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

    if (key == BlackKeyId::Mono || key == BlackKeyId::Poly) {
        state_.global.monoCollapse = (key == BlackKeyId::Mono);
        triggerHud(HudMode::OutputRouting, hud_.focusedVoice, kNoFocusedKnob);
        publishState();
    } else {
        hud_.lastBlackKey = key;
        triggerHud(HudMode::UnassignedKey, hud_.focusedVoice, kNoFocusedKnob);
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
            toggleVoiceMuteGate(state_.voices[whiteKeyIndex]);
            const bool focusCol =
                isPerOscKnobBank(hud_.knobBank) &&
                !(hud_.knobBank == KnobBankMode::Panning && state_.global.monoCollapse);
            triggerHud(HudMode::VoiceGate, whiteKeyIndex,
                       focusCol ? whiteKeyIndex : kNoFocusedKnob);
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
            if (hud_.fxPageOpen) {
                hud_.focusedVoice = static_cast<uint8_t>(hud_.fxVoiceFocus & 3u);
            } else if (hud_.lfoPageOpen) {
                hud_.focusedVoice = static_cast<uint8_t>(hud_.lfoVoiceFocus & 3u);
            }
        }
        if (latchedButton_ != ButtonId::Count) {
            hud_.lastButton = latchedButton_;
        }
    }
    if (dissolvedToIdle) {
        dismissedByHome_ = false;
        hud_.mode = HudMode::None;
        hud_.focusedKnob = kNoFocusedKnob;
    }

    updateLeds(dt, telemetry);
}

void RuidoController::updateLeds(float dtSec, const EngineTelemetry& telemetry) noexcept {
    const LedInputContext ctx{
        state_, hud_, telemetry, keys_.blackKeyHeld, keys_.whiteKeyHeld,
    };
    stepAndComputePanelLeds(dtSec, ctx, ledSmooth_, leds_);
}

} // namespace ruido
