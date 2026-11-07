#include "core/EncoderMath.h"
#include "core/KeyPunchSpec.h"
#include "core/RuidoController.h"

namespace ruido {

KnobBankMode RuidoController::baseKnobBank() const noexcept {
    return (hud_.configPageOpen || hud_.selPageOpen) ? KnobBankMode::None
           : hud_.saveArmed                          ? KnobBankMode::Snapshot
           : hud_.fxPageOpen                         ? KnobBankMode::VoiceFilter
           : hud_.panPageOpen                        ? KnobBankMode::Panning
           : hud_.editPageOpen                       ? knobBankForEditTarget(hud_.editTarget)
           : hud_.lfoPageOpen                        ? KnobBankMode::Lfo
                                                     : KnobBankMode::Frequency;
}

void RuidoController::exitConfigSubView(bool restoreAudio) noexcept {
    hud_.configSubView = ConfigSubView::List;
    hud_.configResetTicks = 0u;
    configTickAccum_ = 0;
    selectTickAccum_ = 0;
    if (firmwareStandbyActive_) {
        firmwareStandbyActive_ = false;
        if (restoreAudio && state_.global.masterRun != firmwareStandbyPrevRun_) {
            state_.global.masterRun = firmwareStandbyPrevRun_;
            publishState();
        }
    }
}

void RuidoController::performFactoryReset() noexcept {
    exitConfigSubView(false);
    state_ = EngineControlState::makeDefault();
    state_.global.masterTrim = masterPot_;
    hud_.config = GlobalConfigState{};
    hud_.keyboardModel = KeyboardModel::LiveMixer;
    midiClock_.reset();
    keyboard_.reset();
    keyboard_.syncConfig(hud_.config);
    snapshots_.reset();
    snapshots_.syncHud(hud_);
    savedSlotMask_ = 0x0Fu;
    settingsDirty_ = true;
    (void)syncClockState();
    keyboard_.evaluate(state_, hud_);
    publishState();
}

void RuidoController::handleConfigEncoder(EncoderId encoder, int32_t ticks,
                                          KnobRate rate) noexcept {
    if (hud_.configSubView != ConfigSubView::List) {
        if (encoder == EncoderId::Select) {
            const int32_t nextSel = static_cast<int32_t>(selectTickAccum_) + ticks;
            if (nextSel >= 4 || nextSel <= -4) {
                exitConfigSubView(true);
            } else {
                selectTickAccum_ = static_cast<int8_t>(nextSel);
            }
        } else if (hud_.configSubView == ConfigSubView::FactoryResetConfirm) {
            const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
            if (nextAcc <= -4) {
                exitConfigSubView(true);
            } else if (nextAcc >= 8) {
                performFactoryReset();
                hud_.configSubView = ConfigSubView::FactoryResetDone;
                hud_.configResetTicks = 8u;
            } else {
                configTickAccum_ = static_cast<int8_t>(nextAcc);
                hud_.configResetTicks = (nextAcc > 0) ? static_cast<uint8_t>(nextAcc) : 0u;
            }
        } else if (hud_.configSubView == ConfigSubView::FactoryResetDone) {
            const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
            if (nextAcc >= 4 || nextAcc <= -4) {
                exitConfigSubView(true);
            } else {
                configTickAccum_ = static_cast<int8_t>(nextAcc);
            }
        } else {
            const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
            if (nextAcc <= -4) {
                exitConfigSubView(true);
            } else {
                configTickAccum_ = static_cast<int8_t>((nextAcc > 0) ? 0 : nextAcc);
            }
        }
        hud_.knobBank = KnobBankMode::None;
        latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
        return;
    }

    if (encoder == EncoderId::Select) {
        const int32_t maxItem = static_cast<int32_t>(kNumGlobalConfigItems) - 1;
        const int32_t nextRow = stepClampedDetent(static_cast<int32_t>(hud_.configCursor), maxItem,
                                                  ticks, rate, selectTickAccum_);
        hud_.configCursor = static_cast<uint8_t>(nextRow);
        configTickAccum_ = 0;
        hud_.knobBank = KnobBankMode::None;
        latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
        return;
    }

    const auto item = static_cast<ConfigItemId>(
        (hud_.configCursor < kNumGlobalConfigItems) ? hud_.configCursor : 0u);
    if (isGlobalConfigActionItem(item)) {
        const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
        if (nextAcc >= 4) {
            configTickAccum_ = 0;
            selectTickAccum_ = 0;
            hud_.configResetTicks = 0u;
            if (item == ConfigItemId::FirmwareUpdate) {
                firmwareStandbyPrevRun_ = state_.global.masterRun;
                firmwareStandbyActive_ = true;
                hud_.configSubView = ConfigSubView::FirmwareUpdate;
                if (state_.global.masterRun) {
                    state_.global.masterRun = false;
                    publishState();
                }
            } else if (item == ConfigItemId::FactoryReset) {
                hud_.configSubView = ConfigSubView::FactoryResetConfirm;
            } else {
                hud_.configSubView = ConfigSubView::Credits;
            }
        } else {
            configTickAccum_ = static_cast<int8_t>((nextAcc < 0) ? 0 : nextAcc);
        }
        hud_.knobBank = KnobBankMode::None;
        latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
        return;
    }

    const int32_t curVal = globalConfigGetIndex(item, hud_.config, state_.global);
    const int32_t maxVal = globalConfigMaxIndex(item);
    const int32_t nextVal = stepClampedDetent(curVal, maxVal, ticks, rate, configTickAccum_);
    if (globalConfigSetIndex(item, nextVal, hud_.config, state_.global)) {
        settingsDirty_ = true;
        keyboard_.syncConfig(hud_.config);
        snapshots_.cancelMomentaryIfActive(state_);
        snapshots_.syncHud(hud_);
        (void)syncClockState();
        keyboard_.evaluate(state_, hud_);
        publishState();
    }
    hud_.knobBank = KnobBankMode::None;
    latchSwitchHud(HudMode::GlobalConfig, ButtonId::GloRight);
}

void RuidoController::closeAllPages() noexcept {
    exitConfigSubView(true);
    hud_.freqPageOpen = hud_.fxPageOpen = hud_.panPageOpen = false;
    hud_.lfoPageOpen = hud_.editPageOpen = hud_.configPageOpen = hud_.selPageOpen = false;
}

void RuidoController::resetPageNav() noexcept {
    selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
    hud_.focusedKnob = kNoFocusedKnob;
    closeAllPages();
    hud_.saveArmed = false;
    hud_.saveDeleteMode = false;
}

void RuidoController::setLatchedPage(HudMode mode, ButtonId button) noexcept {
    if (mode != HudMode::None) {
        dismissedByHome_ = false;
        latchedHudMode_ = mode;
        latchedButton_ = button;
        hud_.lastButton = button;
    } else {
        latchedHudMode_ = HudMode::None;
        latchedButton_ = ButtonId::Count;
    }
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

void RuidoController::restoreLatchedFields() noexcept {
    hud_.mode = latchedHudMode_;
    hud_.knobBank = baseKnobBank();
    if (hud_.fxPageOpen) {
        hud_.focusedVoice = static_cast<uint8_t>(hud_.fxVoiceFocus & 3u);
    } else if (hud_.lfoPageOpen) {
        hud_.focusedVoice = lfoFocusVoice(hud_.lfoVoiceFocus);
    }
    if (latchedButton_ != ButtonId::Count) {
        hud_.lastButton = latchedButton_;
    }
}

void RuidoController::restoreOrDissolveHud() noexcept {
    if (keyboard_.anyKeyHeld()) {
        dismissedByHome_ = false;
        hud_.focusedKnob = kNoFocusedKnob;
        if (hud_.activePunch == KeyPunchKind::None && hud_.selPageOpen) {
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::KeyboardModel;
            env_.latchPage();
        } else {
            if (hud_.activePunch == KeyPunchKind::VoiceFormant) {
                hud_.focusedVoice = hud_.activePunchVoice & 3u;
            }
            hud_.knobBank = knobBankForKeyPunch(hud_.activePunch, KnobBankMode::Frequency);
            hud_.mode = HudMode::KeyboardPlay;
            env_.triggerToast();
        }
        return;
    }
    hud_.knobBank = baseKnobBank();
    if (hud_.mode == HudMode::KeyboardPlay && keyboard_.model() == KeyboardModel::RhythmLab &&
        hud_.lastBlackKey == BlackKeyId::ScopeFreezeKey) {
        triggerHud(HudMode::MasterTempo, hud_.focusedVoice, kNoFocusedKnob);
        return;
    }
    if (latchedHudMode_ != HudMode::None) {
        restoreLatchedFields();
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
    hud_.modifierHeld = hud_.rateHoldActive;
    if (keyboard_.anyKeyHeld()) {
        restoreOrDissolveHud();
    } else if (hud_.rateHoldActive) {
        hud_.knobBank = baseKnobBank();
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
}

void RuidoController::onButtonEvent(ButtonId button, bool pressed) noexcept {
    if (button == ButtonId::OctMinus || button == ButtonId::OctPlus) {
        const bool wasRateHeld = keys_.setRateButton(button, pressed);
        const bool nowRateHeld = keys_.isRateHeld();
        hud_.knobRate = keys_.activeKnobRate();
        hud_.rateHoldActive = nowRateHeld;
        hud_.modifierHeld = nowRateHeld;

        if (nowRateHeld) {
            dismissedByHome_ = false;
            if (!wasRateHeld) {
                knobTurnedDuringRateHold_ = false;
                const bool nonRateToast = (hud_.mode == HudMode::ScopeTimebase);
                if ((env_.holdRemainingSec <= 0.0f || nonRateToast) &&
                    latchedHudMode_ == HudMode::None && !keyboard_.anyKeyHeld() &&
                    !hud_.scopeFrozen) {
                    hud_.mode = HudMode::KnobRate;
                    hud_.knobBank = KnobBankMode::Frequency;
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
            if (keyboard_.anyKeyHeld() || hud_.scopeFrozen) {
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

    if (button == ButtonId::Sel) {
        const bool opening = !hud_.selPageOpen;
        resetPageNav();
        hud_.selPageOpen = opening;
        if (opening) {
            ++hud_.selWakeSeq;
        }
        setLatchedPage(opening ? HudMode::KeyboardModel : HudMode::None, ButtonId::Sel);
        syncModifierHud();
        return;
    }

    if (button == ButtonId::Edit) {
        const bool toMix = !hud_.editPageOpen && !hud_.panPageOpen;
        const bool toPan = hud_.editPageOpen;
        resetPageNav();
        hud_.editPageOpen = toMix;
        hud_.panPageOpen = toPan;
        setLatchedPage(toMix   ? hudModeForEditTarget(hud_.editTarget)
                       : toPan ? HudMode::Panning
                               : HudMode::None,
                       ButtonId::Edit);
        syncModifierHud();
        return;
    }

    if (button == ButtonId::Fx || button == ButtonId::Lfo || button == ButtonId::GloRight) {
        const bool isFx = (button == ButtonId::Fx);
        const bool isLfo = (button == ButtonId::Lfo);
        const bool opening = isFx    ? !hud_.fxPageOpen
                             : isLfo ? !hud_.lfoPageOpen
                                     : !hud_.configPageOpen;
        resetPageNav();
        if (opening) {
            hud_.fxPageOpen = isFx;
            hud_.lfoPageOpen = isLfo;
            hud_.configPageOpen = (!isFx && !isLfo);
            if (isFx || isLfo) {
                hud_.focusedVoice = isFx ? static_cast<uint8_t>(hud_.fxVoiceFocus & 3u)
                                         : lfoFocusVoice(hud_.lfoVoiceFocus);
            }
        }
        setLatchedPage(opening ? (isFx    ? HudMode::VoiceFilter
                                  : isLfo ? HudMode::VoiceLfo
                                          : HudMode::GlobalConfig)
                               : HudMode::None,
                       button);
        syncModifierHud();
        return;
    }

    if (button == ButtonId::PlayStop) {
        if (firmwareStandbyActive_) {
            exitConfigSubView(false);
        }
        state_.global.masterRun = !state_.global.masterRun;
        if (anyPageOpen()) {
            hud_.lastButton = button;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, kNoFocusedKnob);
        } else {
            hud_.saveArmed = false;
            hud_.saveDeleteMode = false;
            hud_.knobBank = KnobBankMode::Frequency;
            latchSwitchHud(HudMode::AudioOutput, button);
        }
        publishState();
        return;
    }

    if (button == ButtonId::Home) {
        const bool hudActive = (latchedHudMode_ != HudMode::None) || anyPageOpen() ||
                               hud_.saveArmed ||
                               keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen) ||
                               (hud_.mode != HudMode::None && !dismissedByHome_);
        resetPageNav();
        knobTurnedDuringRateHold_ = false;
        if (!hudActive) {
            hud_.freqPageOpen = true;
            hud_.knobBank = KnobBankMode::Frequency;
            latchSwitchHud(HudMode::VoiceFrequency, ButtonId::Home);
            return;
        }
        dismissedByHome_ = true;
        setLatchedPage(HudMode::None, ButtonId::Count);
        if (keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen)) {
            if (!keyboard_.anyKeyHeld() && !hud_.scopeFrozen && hud_.rateHoldActive) {
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
        hud_.snapshotSavedToast = false;
        hud_.snapshotDeletedToast = false;
        hud_.saveArmed = !hud_.saveArmed;
        hud_.knobBank = KnobBankMode::Snapshot;
        if (hud_.saveArmed) {
            hud_.saveDeleteMode = false;
            selectTickAccum_ = 0;
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
        } else {
            setLatchedPage(HudMode::None, ButtonId::Count);
            triggerHud(HudMode::SaveArmed, hud_.focusedVoice, kNoFocusedKnob);
        }
        return;
    }

    // Unconnected switches (ENV, ARP, SEQ, REC): leave open pages, armed SAVE, HUD mode, and audio
    // 100% undisturbed while firing a tactile capacitive CRT beam ripple + local LED glint.
    hud_.phantomPinIndex = phantomPinSlot(button);
    ++hud_.phantomPinSeq;
}

} // namespace ruido
