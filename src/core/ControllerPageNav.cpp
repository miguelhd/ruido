#include "core/EncoderMath.h"
#include "core/KeyPunchSpec.h"
#include "core/RuidoController.h"

namespace ruido {

KnobBankMode RuidoController::baseKnobBank() const noexcept {
    return (hud_.configPageOpen || hud_.selPageOpen || hud_.saveArmed) ? KnobBankMode::None
           : hud_.fxPageOpen                                           ? KnobBankMode::VoiceFilter
           : hud_.panPageOpen                                          ? KnobBankMode::Panning
           : hud_.editPageOpen                                         ? KnobBankMode::Amplitude
           : hud_.lfoPageOpen                                          ? KnobBankMode::Lfo
           : hud_.freqPageOpen ? knobBankForHomeTarget(hud_.homeTarget)
                               : KnobBankMode::Frequency;
}

void RuidoController::exitConfigSubView() noexcept {
    hud_.configSubView = ConfigSubView::List;
    hud_.configResetTicks = 0u;
    configTickAccum_ = 0;
    selectTickAccum_ = 0;
}

void RuidoController::performFactoryReset() noexcept {
    exitConfigSubView();
    state_ = EngineControlState::makeDefault();
    state_.global.masterTrim = masterPot_;
    hud_.config = GlobalConfigState{};
    hud_.keyboardModel = KeyboardModel::LiveMixer;
    hud_.homeTarget = HomeTarget::Frequency;
    editRememberPan_ = false;
    midiClock_.reset();
    keyboard_.reset();
    keyboard_.syncConfig(hud_.config);
    snapshots_.reset();
    applyPatchFrame(decodePatchSnapshot(snapshots_.slotBinary(0u)), state_);
    snapshots_.syncHud(hud_);
    ledSmooth_.reset(state_);
    savedSlotMask_ = 0x0Fu;
    settingsDirty_ = true;
    livePatchDirty_ = true;
    (void)syncClockState();
    keyboard_.evaluate(state_, hud_);
    publishState();
}

void RuidoController::performCleanInit() noexcept {
    const bool keepMono = state_.global.monoCollapse;
    exitConfigSubView();
    state_ = EngineControlState::makeDefault();
    state_.global.masterTrim = masterPot_;
    state_.global.monoCollapse = keepMono;
    hud_.keyboardModel = KeyboardModel::LiveMixer;
    hud_.homeTarget = HomeTarget::Frequency;
    editRememberPan_ = false;
    keyboard_.reset();
    keyboard_.syncConfig(hud_.config);
    snapshots_.resetEmpty(state_);
    snapshots_.syncHud(hud_);
    ledSmooth_.reset(state_);
    savedSlotMask_ = 0x0Fu;
    settingsDirty_ = true;
    livePatchDirty_ = true;
    (void)syncClockState();
    keyboard_.evaluate(state_, hud_);
    publishState();
}

void RuidoController::closeAllPages() noexcept {
    exitConfigSubView();
    hud_.homeTarget = HomeTarget::Frequency;
    hud_.freqPageOpen = hud_.fxPageOpen = hud_.panPageOpen = false;
    hud_.lfoPageOpen = hud_.editPageOpen = hud_.configPageOpen = hud_.selPageOpen = false;
}

void RuidoController::clearKnobFocus() noexcept {
    hud_.focusedKnob = kNoFocusedKnob;
    knobFocusMask_ = 0u;
    knobTurnHoldQ8_.fill(0u);
    syncFocusedColumnMask();
}

void RuidoController::recordKnobFocus(uint8_t knobIndex) noexcept {
    if (knobIndex >= kNumOscillators) {
        clearKnobFocus();
        return;
    }
    uint8_t liveTurningMask = 0u;
    for (uint8_t i = 0u; i < kNumOscillators; ++i) {
        if (knobTurnHoldQ8_[i] > 0u) {
            liveTurningMask |= static_cast<uint8_t>(1u << i);
        }
    }
    hud_.focusedKnob = knobIndex;
    knobFocusMask_ = static_cast<uint8_t>(liveTurningMask | (1u << knobIndex));
    knobTurnHoldQ8_[knobIndex] = 255u;
    syncFocusedColumnMask();
}

void RuidoController::syncFocusedColumnMask() noexcept {
    uint8_t mask = knobFocusMask_;
    if (!hud_.shiftHeld && !hud_.saveArmed && isPerOscKnobBank(hud_.knobBank)) {
        const uint16_t exprWhite = static_cast<uint16_t>(hud_.heldWhiteMask & 0x0FFFu);
        mask |= static_cast<uint8_t>((exprWhite & 0x0Fu) | ((exprWhite >> 4u) & 0x0Fu) |
                                     ((exprWhite >> 8u) & 0x0Fu));
    }
    hud_.focusedColumnMask = static_cast<uint8_t>(mask & 0x0Fu);
}

void RuidoController::resetPageNav() noexcept {
    selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
    closeAllPages();
    hud_.saveArmed = false;
    if (!hud_.rateHoldActive || (hud_.capsuleToast != CapsuleToastKind::Fine &&
                                 hud_.capsuleToast != CapsuleToastKind::Coarse)) {
        triggerCapsuleToast(CapsuleToastKind::None);
    }
    clearKnobFocus();
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
    recordKnobFocus(knobIndex);
    const float holdSec = (mode == HudMode::RailCoupling && !hasNonHomePageOpen(hud_))
                              ? kHudCouplingHoldSec
                              : kHudToastHoldSec;
    env_.triggerToast(holdSec);
}

void RuidoController::latchSwitchHud(HudMode mode, ButtonId button) noexcept {
    dismissedByHome_ = false;
    latchedHudMode_ = mode;
    latchedButton_ = button;
    if (button != ButtonId::Count) {
        hud_.lastButton = button;
    }
    hud_.mode = mode;
    clearKnobFocus();
    env_.latchPage();
}

void RuidoController::latchKnobEdit(KnobBankMode bank, HudMode mode, uint8_t knobIdx,
                                    uint8_t voiceIdx) noexcept {
    dismissedByHome_ = false;
    if (hud_.knobBank != bank) {
        knobTurnHoldQ8_.fill(0u);
    }
    hud_.knobBank = bank;
    hud_.focusedVoice = static_cast<uint8_t>(voiceIdx & 3u);
    latchedHudMode_ = mode;
    hud_.mode = mode;
    recordKnobFocus(knobIdx);
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
    if (keyboard_.isShiftHeld()) {
        dismissedByHome_ = false;
        hud_.knobBank = KnobBankMode::None;
        hud_.mode = HudMode::ShiftMap;
        clearKnobFocus();
        env_.latchPage();
        return;
    }
    if (keys_.isAnyPerformanceKeyHeld()) {
        dismissedByHome_ = false;
        clearKnobFocus();
        // GLOBAL CONFIG > Key Pressure doubles as a touch test: keys still play, but the list stays
        // on screen so the row's live pressure meter remains visible while squeezing.
        if (hud_.configPageOpen && hud_.configSubView == ConfigSubView::List &&
            hud_.configCursor == static_cast<uint8_t>(ConfigItemId::PressureTouch)) {
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::GlobalConfig;
            env_.latchPage();
            return;
        }
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::KeyboardModel;
            env_.latchPage();
            return;
        }
        if (hud_.activePunch != KeyPunchKind::None) {
            if (hud_.activePunch == KeyPunchKind::KeyHoldSwitch) {
                hud_.knobBank = anyPageOpen() ? baseKnobBank() : KnobBankMode::None;
                hud_.mode = HudMode::KeyHold;
                env_.triggerToast();
                return;
            }
            if (hud_.activePunch == KeyPunchKind::TapTempo) {
                hud_.knobBank = anyPageOpen() ? baseKnobBank() : KnobBankMode::None;
                hud_.mode = HudMode::MasterTempo;
                env_.triggerToast();
                return;
            }
            if (hud_.activePunch == KeyPunchKind::VoiceFormant) {
                hud_.focusedVoice = hud_.activePunchVoice & 3u;
            }
            hud_.knobBank = anyPageOpen()
                                ? baseKnobBank()
                                : knobBankForKeyPunch(hud_.activePunch, KnobBankMode::Frequency);
            hud_.mode = HudMode::KeyboardPlay;
            env_.triggerToast();
            return;
        }
    }
    if ((hud_.mode == HudMode::MasterTempo || hud_.mode == HudMode::KeyHold ||
         hud_.mode == HudMode::SnapshotSlot) &&
        env_.holdRemainingSec > 0.0f) {
        hud_.knobBank = anyPageOpen() ? baseKnobBank() : KnobBankMode::None;
        return;
    }
    hud_.knobBank = baseKnobBank();
    if (latchedHudMode_ != HudMode::None) {
        restoreLatchedFields();
        clearKnobFocus();
        env_.latchPage();
        return;
    }
    if (env_.beginFadeOut()) {
        hud_.mode = HudMode::None;
        hud_.opacity = 0.0f;
        clearKnobFocus();
    }
}

void RuidoController::syncModifierHud() noexcept {
    hud_.modifierHeld = hud_.rateHoldActive;
    restoreOrDissolveHud();
}

void RuidoController::onButtonEvent(ButtonId button, bool pressed) noexcept {
    if (button == ButtonId::OctMinus || button == ButtonId::OctPlus) {
        const KnobRate prevRate = keys_.activeKnobRate();
        (void)keys_.setRateButton(button, pressed);
        const bool nowRateHeld = keys_.isRateHeld();
        hud_.knobRate = keys_.activeKnobRate();
        hud_.rateHoldActive = nowRateHeld;
        hud_.modifierHeld = nowRateHeld;
        if (nowRateHeld && !keyboard_.isShiftHeld() && hud_.knobRate != prevRate &&
            (hud_.knobRate == KnobRate::Fine || hud_.knobRate == KnobRate::Coarse)) {
            triggerCapsuleToast((hud_.knobRate == KnobRate::Fine) ? CapsuleToastKind::Fine
                                                                  : CapsuleToastKind::Coarse);
        } else if (!nowRateHeld && (hud_.capsuleToast == CapsuleToastKind::Fine ||
                                    hud_.capsuleToast == CapsuleToastKind::Coarse)) {
            triggerCapsuleToast(CapsuleToastKind::None);
        }
        if (!nowRateHeld) {
            knobTurnedDuringRateHold_ = false;
        }
        if (keyboard_.isShiftHeld()) {
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::ShiftMap;
            env_.latchPage();
        }
        return;
    }

    if (!pressed) {
        return;
    }

    if (keyboard_.isShiftHeld()) {
        triggerUnassignedFeedback(phantomPinSlot(button));
        return;
    }

    if (button == ButtonId::Sel) {
        const bool wasOpen = hud_.selPageOpen;
        resetPageNav();
        if (wasOpen) {
            const auto nextIdx = (static_cast<uint8_t>(keyboard_.model()) + 1u) %
                                 static_cast<uint8_t>(KeyboardModel::Count);
            applyKeyboardModel(static_cast<KeyboardModel>(nextIdx));
        } else {
            hud_.selPageOpen = true;
            ++hud_.selWakeSeq;
            setLatchedPage(HudMode::KeyboardModel, ButtonId::Sel);
        }
        syncModifierHud();
        return;
    }

    if (button == ButtonId::Edit) {
        if (hud_.editPageOpen || hud_.panPageOpen) {
            editRememberPan_ = !hud_.panPageOpen;
        }
        resetPageNav();
        if (editRememberPan_) {
            hud_.panPageOpen = true;
            setLatchedPage(HudMode::Panning, ButtonId::Edit);
        } else {
            hud_.editPageOpen = true;
            setLatchedPage(HudMode::Amplitude, ButtonId::Edit);
        }
        syncModifierHud();
        return;
    }

    if (button == ButtonId::Fx || button == ButtonId::Lfo) {
        const bool isFx = (button == ButtonId::Fx);
        if (isFx ? hud_.fxPageOpen : hud_.lfoPageOpen) {
            if (isFx) {
                hud_.fxVoiceFocus = static_cast<uint8_t>((hud_.fxVoiceFocus + 1u) & 3u);
            } else {
                hud_.lfoVoiceFocus = static_cast<uint8_t>((hud_.lfoVoiceFocus + 1u) & 7u);
            }
        }
        resetPageNav();
        hud_.fxPageOpen = isFx;
        hud_.lfoPageOpen = !isFx;
        hud_.focusedVoice =
            isFx ? static_cast<uint8_t>(hud_.fxVoiceFocus & 3u) : lfoFocusVoice(hud_.lfoVoiceFocus);
        setLatchedPage(isFx ? HudMode::VoiceFilter : HudMode::VoiceLfo, button);
        syncModifierHud();
        return;
    }

    if (button == ButtonId::GloRight) {
        const bool opening = !hud_.configPageOpen;
        resetPageNav();
        hud_.configPageOpen = opening;
        setLatchedPage(opening ? HudMode::GlobalConfig : HudMode::None, ButtonId::GloRight);
        syncModifierHud();
        return;
    }

    if (button == ButtonId::PlayStop) {
        state_.global.masterRun = !state_.global.masterRun;
        if (anyPageOpen() || hud_.saveArmed) {
            hud_.lastButton = button;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, kNoFocusedKnob);
        } else {
            hud_.saveArmed = false;
            hud_.savePageMode = SavePageMode::Store;
            hud_.knobBank = KnobBankMode::Frequency;
            latchSwitchHud(HudMode::AudioOutput, button);
        }
        publishState();
        return;
    }

    if (button == ButtonId::Home) {
        const bool momentaryHeld = keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen);
        if (hud_.freqPageOpen && !momentaryHeld) {
            const auto curIdx = static_cast<uint8_t>(hud_.homeTarget);
            if (curIdx + 1u < static_cast<uint8_t>(HomeTarget::Count)) {
                const auto nextTgt = static_cast<HomeTarget>(curIdx + 1u);
                resetPageNav();
                knobTurnedDuringRateHold_ = false;
                hud_.freqPageOpen = true;
                hud_.homeTarget = nextTgt;
                hud_.knobBank = knobBankForHomeTarget(nextTgt);
                latchSwitchHud(hudModeForHomeTarget(nextTgt), ButtonId::Home);
                return;
            }
        }
        const bool hudActive = (latchedHudMode_ != HudMode::None) || anyPageOpen() ||
                               hud_.saveArmed || momentaryHeld ||
                               (hud_.mode != HudMode::None && !dismissedByHome_);
        if (latchedHudMode_ != HudMode::None && hud_.mode != latchedHudMode_) {
            restoreLatchedFields();
        }
        resetPageNav();
        knobTurnedDuringRateHold_ = false;
        if (!hudActive) {
            hud_.freqPageOpen = true;
            hud_.homeTarget = HomeTarget::Frequency;
            hud_.knobBank = KnobBankMode::Frequency;
            latchSwitchHud(HudMode::VoiceFrequency, ButtonId::Home);
            return;
        }
        dismissedByHome_ = true;
        setLatchedPage(HudMode::None, ButtonId::Count);
        if (momentaryHeld) {
            if (!keyboard_.anyKeyHeld() && !hud_.scopeFrozen && hud_.rateHoldActive) {
                hud_.knobBank = KnobBankMode::Frequency;
                hud_.mode = HudMode::KnobRate;
                env_.holdRemainingSec = 0.0f;
                env_.active = true;
            } else {
                syncModifierHud();
            }
        } else {
            const bool keepNoKnobBank =
                (hud_.mode == HudMode::GlobalConfig || hud_.mode == HudMode::KeyboardModel ||
                 hud_.mode == HudMode::SaveArmed || hud_.mode == HudMode::SnapshotSlot ||
                 hud_.mode == HudMode::ShiftMap || hud_.mode == HudMode::KeyHold ||
                 hud_.knobBank == KnobBankMode::None);
            if (env_.beginFadeOut()) {
                hud_.mode = HudMode::None;
                hud_.knobBank = KnobBankMode::Frequency;
                hud_.opacity = 0.0f;
            } else {
                hud_.knobBank = keepNoKnobBank ? KnobBankMode::None : KnobBankMode::Frequency;
            }
        }
        return;
    }

    if (button == ButtonId::Save) {
        closeAllPages();
        hud_.snapshotToast = SnapshotToastKind::None;
        selectTickAccum_ = 0;
        if (!hud_.saveArmed) {
            hud_.saveArmed = true;
            hud_.savePageMode = SavePageMode::Store;
            hud_.knobBank = KnobBankMode::None;
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
        } else if (static_cast<uint8_t>(hud_.savePageMode) + 1u <
                   static_cast<uint8_t>(SavePageMode::Count)) {
            hud_.savePageMode =
                static_cast<SavePageMode>(static_cast<uint8_t>(hud_.savePageMode) + 1u);
            hud_.knobBank = KnobBankMode::None;
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
        } else {
            hud_.saveArmed = false;
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
            } else if (env_.beginFadeOut()) {
                hud_.mode = HudMode::None;
                hud_.knobBank = KnobBankMode::Frequency;
                hud_.opacity = 0.0f;
            } else {
                hud_.knobBank = KnobBankMode::None;
            }
        }
        return;
    }

    // Unconnected switches (ENV, ARP, SEQ, REC): leave open pages, armed SAVE, HUD mode, and audio
    // 100% undisturbed while firing a tactile capacitive CRT beam ripple + local LED glint.
    triggerUnassignedFeedback(phantomPinSlot(button));
}

} // namespace ruido
