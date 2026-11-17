#include "core/DspMath.h"
#include "core/EncoderMath.h"
#include "core/LfoMath.h"
#include "core/RuidoController.h"

namespace ruido {

void RuidoController::handleConfigEncoder(EncoderId encoder, int32_t ticks,
                                          KnobRate rate) noexcept {
    if (hud_.configSubView != ConfigSubView::List) {
        if (isResetConfirmSubView(hud_.configSubView)) {
            if (encoder == EncoderId::Select) {
                const int32_t curMode =
                    (hud_.configSubView == ConfigSubView::CleanInitConfirm) ? 1 : 0;
                const int32_t nextMode =
                    stepClampedDetent(curMode, 1, ticks, rate, selectTickAccum_);
                const ConfigSubView nextView = (nextMode > 0) ? ConfigSubView::CleanInitConfirm
                                                              : ConfigSubView::FactoryResetConfirm;
                if (nextView != hud_.configSubView) {
                    hud_.configSubView = nextView;
                    hud_.configResetTicks = 0u;
                    configTickAccum_ = 0;
                }
            } else {
                const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
                if (nextAcc <= -4) {
                    exitConfigSubView();
                } else if (nextAcc >= 8) {
                    const bool cleanInit = (hud_.configSubView == ConfigSubView::CleanInitConfirm);
                    if (cleanInit) {
                        performCleanInit();
                        hud_.configSubView = ConfigSubView::CleanInitDone;
                    } else {
                        performFactoryReset();
                        hud_.configSubView = ConfigSubView::FactoryResetDone;
                    }
                    hud_.configResetTicks = 8u;
                } else {
                    configTickAccum_ = static_cast<int8_t>(nextAcc);
                    hud_.configResetTicks = (nextAcc > 0) ? static_cast<uint8_t>(nextAcc) : 0u;
                }
            }
        } else if (isResetDoneSubView(hud_.configSubView)) {
            if (encoder == EncoderId::Select) {
                const int32_t nextSel = static_cast<int32_t>(selectTickAccum_) + ticks;
                if (hud_.configSubView == ConfigSubView::FactoryResetDone && nextSel >= 4) {
                    hud_.configSubView = ConfigSubView::CleanInitConfirm;
                    hud_.configResetTicks = 0u;
                    configTickAccum_ = 0;
                    selectTickAccum_ = 0;
                } else if (hud_.configSubView == ConfigSubView::CleanInitDone && nextSel <= -4) {
                    hud_.configSubView = ConfigSubView::FactoryResetConfirm;
                    hud_.configResetTicks = 0u;
                    configTickAccum_ = 0;
                    selectTickAccum_ = 0;
                } else if (nextSel >= 4 || nextSel <= -4) {
                    exitConfigSubView();
                } else {
                    selectTickAccum_ = static_cast<int8_t>(nextSel);
                }
            } else {
                const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
                if (nextAcc <= -4) {
                    exitConfigSubView();
                } else {
                    configTickAccum_ = static_cast<int8_t>((nextAcc > 0) ? 0 : nextAcc);
                }
            }
        } else if (encoder == EncoderId::Select) {
            const int32_t nextSel = static_cast<int32_t>(selectTickAccum_) + ticks;
            if (nextSel >= 4 || nextSel <= -4) {
                exitConfigSubView();
            } else {
                selectTickAccum_ = static_cast<int8_t>(nextSel);
            }
        } else {
            const int32_t nextAcc = static_cast<int32_t>(configTickAccum_) + ticks;
            if (nextAcc <= -4) {
                exitConfigSubView();
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
            hud_.configSubView = (item == ConfigItemId::FactoryReset)
                                     ? ConfigSubView::FactoryResetConfirm
                                     : ConfigSubView::Credits;
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

void RuidoController::onEncoderDelta(EncoderId encoder, int32_t ticks) noexcept {
    if (ticks == 0) {
        return;
    }
    const KnobRate rate = hud_.knobRate;

    if (hud_.rateHoldActive) {
        knobTurnedDuringRateHold_ = true;
    }

    if (keyboard_.isShiftHeld()) {
        if (encoder == EncoderId::Algorithm) {
            if (!hud_.midiClockSynced) {
                const float prevBpm = midiClock_.internalBpm();
                const float nextBpm = adjustClockBpm(prevBpm, ticks, rate);
                if (absf(nextBpm - prevBpm) > 1.0e-5f) {
                    midiClock_.setInternalBpm(nextBpm);
                    settingsDirty_ = true;
                    (void)syncClockState();
                }
            }
            keyboard_.clearShiftNotices();
            hud_.shiftFocus = ShiftFocus::Tempo;
            keyboard_.evaluate(state_, hud_);
            hud_.activePunch = KeyPunchKind::None;
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::ShiftMap;
            env_.latchPage();
            publishState();
            return;
        }
        if (encoder == EncoderId::Select) {
            const int32_t maxModel = static_cast<int32_t>(KeyboardModel::Count) - 1;
            const int32_t nextIdx = stepClampedDetent(static_cast<int32_t>(keyboard_.model()),
                                                      maxModel, ticks, rate, selectTickAccum_);
            const auto nextModel = static_cast<KeyboardModel>(nextIdx);
            if (nextModel != keyboard_.model()) {
                keyboard_.setModel(nextModel);
                settingsDirty_ = true;
                snapshots_.cancelMomentaryIfActive(state_);
                snapshots_.syncHud(hud_);
            }
            snapShiftBlinkToPeak();
            if (hud_.selPageOpen) {
                ++hud_.selWakeSeq;
            }
            keyboard_.clearShiftNotices();
            hud_.shiftFocus = ShiftFocus::Model;
            keyboard_.evaluate(state_, hud_);
            hud_.activePunch = KeyPunchKind::None;
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::ShiftMap;
            env_.latchPage();
            publishState();
            return;
        }
        if (encoder == EncoderId::Presets) {
            hud_.lastEncoder = EncoderId::Presets;
            hud_.snapshotSavedToast = false;
            hud_.snapshotDeletedToast = false;
            const uint8_t prevSlot = snapshots_.activeSlot();
            if (snapshots_.morphByEncoder(ticks, rate, state_)) {
                if (snapshots_.activeSlot() != prevSlot) {
                    settingsDirty_ = true;
                }
                livePatchDirty_ = true;
                publishState();
            }
            snapshots_.syncHud(hud_);
            hud_.snapshotToastSlot = snapshots_.activeSlot();
            keyboard_.clearShiftNotices();
            hud_.shiftFocus = ShiftFocus::Scene;
            keyboard_.evaluate(state_, hud_);
            hud_.activePunch = KeyPunchKind::None;
            hud_.knobBank = KnobBankMode::None;
            hud_.mode = HudMode::ShiftMap;
            env_.latchPage();
            return;
        }
        triggerUnassignedFeedback(0xFFu);
        return;
    }

    if (encoder == EncoderId::Algorithm) {
        if (hud_.configPageOpen) {
            handleConfigEncoder(encoder, ticks, rate);
            return;
        }
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
        if (hud_.saveArmed) {
            hud_.saveArmed = false;
            hud_.saveDeleteMode = false;
            if (latchedHudMode_ == HudMode::SaveArmed) {
                setLatchedPage(HudMode::None, ButtonId::Count);
            }
        }
        state_.global.circuitCoupling =
            adjustLinear01(state_.global.circuitCoupling, ticks, rate, 0.0f, 1.0f);
        snapshots_.notifyManualEdit(state_);
        livePatchDirty_ = true;
        snapshots_.syncHud(hud_);
        hud_.knobBank = hasNonHomePageOpen(hud_) ? baseKnobBank() : KnobBankMode::None;
        triggerHud(HudMode::RailCoupling, hud_.focusedVoice, kNoFocusedKnob);
        publishState();
        return;
    }

    if (encoder == EncoderId::Select) {
        if (hud_.saveArmed) {
            hud_.lastEncoder = EncoderId::Select;
            const int32_t curMode = hud_.saveDeleteMode ? 1 : 0;
            const int32_t nextMode = stepClampedDetent(curMode, 1, ticks, rate, selectTickAccum_);
            hud_.saveDeleteMode = (nextMode != 0);
            hud_.knobBank = KnobBankMode::None;
            if (keyboard_.anyKeyHeld()) {
                keyboard_.evaluate(state_, hud_);
            }
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
            return;
        }
        if (hud_.selPageOpen || (hud_.mode == HudMode::KeyboardModel && !dismissedByHome_)) {
            const int32_t maxModel = static_cast<int32_t>(KeyboardModel::Count) - 1;
            const int32_t nextIdx = stepClampedDetent(static_cast<int32_t>(keyboard_.model()),
                                                      maxModel, ticks, rate, selectTickAccum_);
            applyKeyboardModel(static_cast<KeyboardModel>(nextIdx));
            return;
        }
        if (hud_.configPageOpen) {
            handleConfigEncoder(encoder, ticks, rate);
            return;
        }
        if (hud_.fxPageOpen || hud_.lfoPageOpen) {
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
        if (hud_.freqPageOpen) {
            const int32_t maxHome = static_cast<int32_t>(HomeTarget::Count) - 1;
            hud_.homeTarget = static_cast<HomeTarget>(stepClampedDetent(
                static_cast<int32_t>(hud_.homeTarget), maxHome, ticks, rate, selectTickAccum_));
            targetTickAccum_ = 0;
            hud_.knobBank = knobBankForHomeTarget(hud_.homeTarget);
            latchSwitchHud(hudModeForHomeTarget(hud_.homeTarget), ButtonId::Home);
            return;
        }
        if (hud_.editPageOpen) {
            targetTickAccum_ = 0;
            hud_.knobBank = KnobBankMode::Amplitude;
            latchSwitchHud(HudMode::Amplitude, ButtonId::Edit);
            return;
        }
        if (hud_.panPageOpen) {
            const int32_t curIdx =
                globalConfigGetIndex(ConfigItemId::OutputRouting, hud_.config, state_.global);
            const int32_t nextIdx =
                stepClampedDetent(curIdx, globalConfigMaxIndex(ConfigItemId::OutputRouting), ticks,
                                  rate, selectTickAccum_);
            if (globalConfigSetIndex(ConfigItemId::OutputRouting, nextIdx, hud_.config,
                                     state_.global)) {
                settingsDirty_ = true;
                publishState();
            }
            hud_.knobBank = KnobBankMode::Panning;
            latchSwitchHud(HudMode::Panning, ButtonId::Edit);
            return;
        }
        hud_.lastEncoder = EncoderId::Select;
        const int32_t maxTb = static_cast<int32_t>(ScopeTimebase::Count) - 1;
        const int32_t nextTb = stepClampedDetent(static_cast<int32_t>(hud_.scopeTimebase), maxTb,
                                                 ticks, rate, selectTickAccum_);
        hud_.scopeTimebase = static_cast<ScopeTimebase>(nextTb);
        hud_.knobBank = KnobBankMode::None;
        triggerHud(HudMode::ScopeTimebase, hud_.focusedVoice, kNoFocusedKnob);
        return;
    }

    if (encoder == EncoderId::Presets) {
        if (hud_.configSubView != ConfigSubView::List) {
            exitConfigSubView();
        }
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
        hud_.lastEncoder = EncoderId::Presets;
        if (hud_.saveArmed) {
            hud_.saveArmed = false;
            hud_.saveDeleteMode = false;
            if (latchedHudMode_ == HudMode::SaveArmed) {
                setLatchedPage(HudMode::None, ButtonId::Count);
            }
        }
        hud_.snapshotSavedToast = false;
        hud_.snapshotDeletedToast = false;

        const uint8_t prevSlot = snapshots_.activeSlot();
        if (snapshots_.morphByEncoder(ticks, rate, state_)) {
            if (snapshots_.activeSlot() != prevSlot) {
                settingsDirty_ = true;
            }
            livePatchDirty_ = true;
            if (keyboard_.anyKeyHeld()) {
                keyboard_.evaluate(state_, hud_);
            }
            publishState();
        }
        snapshots_.syncHud(hud_);
        hud_.snapshotToastSlot = snapshots_.activeSlot();
        hud_.knobBank = anyPageOpen() ? baseKnobBank() : KnobBankMode::None;
        triggerHud(HudMode::SnapshotSlot, snapshots_.activeSlot(), kNoFocusedKnob);
        return;
    }

    const int knobIdx = static_cast<int>(encoder) - static_cast<int>(EncoderId::Knob1);
    if (knobIdx < 0 || knobIdx >= static_cast<int>(kNumOscillators)) {
        return;
    }
    const uint8_t kIdx = static_cast<uint8_t>(knobIdx);
    if (hud_.configSubView != ConfigSubView::List) {
        exitConfigSubView();
    }
    if (hud_.saveArmed) {
        hud_.saveArmed = false;
        hud_.saveDeleteMode = false;
        if (latchedHudMode_ == HudMode::SaveArmed) {
            setLatchedPage(HudMode::None, ButtonId::Count);
        }
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
        applyVoiceLevelKnobDelta(state_.voices[kIdx], ticks, rate);
        latchKnobEdit(KnobBankMode::Amplitude, HudMode::Amplitude, kIdx, kIdx);
    } else if (hud_.freqPageOpen) {
        applyHomeKnobDelta(hud_.homeTarget, state_.voices[kIdx], ticks, rate);
        latchKnobEdit(knobBankForHomeTarget(hud_.homeTarget), hudModeForHomeTarget(hud_.homeTarget),
                      kIdx, kIdx);
    } else if (hud_.selPageOpen || hud_.configPageOpen) {
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
        adjustVoiceFrequency(state_.voices[kIdx], ticks, rate);
        hud_.knobBank = KnobBankMode::None;
        triggerHud(HudMode::VoiceFrequency, kIdx, kIdx);
    } else {
        adjustVoiceFrequency(state_.voices[kIdx], ticks, rate);
        hud_.knobBank = KnobBankMode::Frequency;
        triggerHud(HudMode::VoiceFrequency, kIdx, kIdx);
    }

    snapshots_.notifyManualEdit(state_);
    livePatchDirty_ = true;
    snapshots_.syncHud(hud_);
    if (keyboard_.anyKeyHeld()) {
        keyboard_.evaluate(state_, hud_);
    }
    publishState();
}

void RuidoController::onMasterPot(float normalizedLevel) noexcept {
    const float raw = clampf(normalizedLevel, 0.0f, 1.0f);
    const float next = (raw <= kSilentAmpThreshold) ? 0.0f : raw;
    masterPot_ = next;
    state_.global.masterTrim = masterPot_;
    const bool hitEndpoint = (next == 0.0f && masterPotHudAnchor_ > 0.0f) ||
                             (next == 1.0f && masterPotHudAnchor_ < 1.0f);
    if (hitEndpoint || absf(next - masterPotHudAnchor_) > 0.002f) {
        masterPotHudAnchor_ = next;
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
        if (!keyboard_.isShiftHeld()) {
            if (hud_.rateHoldActive) {
                knobTurnedDuringRateHold_ = true;
            }
            hud_.knobBank = baseKnobBank();
            triggerHud(HudMode::MasterLevel, hud_.focusedVoice, kNoFocusedKnob);
        }
    }
    publishState();
}

} // namespace ruido
