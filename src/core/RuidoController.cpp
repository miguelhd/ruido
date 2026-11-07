#include "core/RuidoController.h"

#include "core/DspMath.h"
#include "core/EncoderMath.h"
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
    keyboard_.reset();
    keyboard_.syncConfig(hud_.config);
    snapshots_.reset();
    snapshots_.syncHud(hud_);
    midiClock_.reset();
    (void)syncClockState();
    keys_.reset();
    env_.reset();
    ledSmooth_.reset(state_);
    knobTurnedDuringRateHold_ = false;
    dismissedByHome_ = false;
    settingsDirty_ = false;
    firmwareStandbyActive_ = false;
    firmwareStandbyPrevRun_ = true;
    saveSwallowedMask_ = 0u;
    savedSlotMask_ = 0u;
    latchedHudMode_ = HudMode::None;
    latchedButton_ = ButtonId::Count;
    selectTickAccum_ = presetTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;

    keyboard_.evaluate(state_, hud_);
    EngineTelemetry zeroTel{};
    updateLeds(0.0f, zeroTel);
    publishState();
}

void RuidoController::publishState() noexcept {
    stateDirty_ = true;
    if (controlBus_ != nullptr) {
        controlBus_->publish(state_);
    }
}

bool RuidoController::syncClockState() noexcept {
    const float nextBpm = midiClock_.bpm(hud_.config.clockSync);
    const bool nextSync = midiClock_.isExternalSynced(hud_.config.clockSync);
    const uint8_t nextSeq = midiClock_.resetSeq();
    hud_.effectiveClockBpm = nextBpm;
    hud_.midiClockSynced = nextSync;
    if (absf(state_.global.clockBpm - nextBpm) > 1.0e-4f ||
        state_.global.clockResetSeq != nextSeq) {
        state_.global.clockBpm = nextBpm;
        state_.global.clockResetSeq = nextSeq;
        return true;
    }
    return false;
}

void RuidoController::applyKeyboardPerformanceChange() noexcept {
    keys_.anyPerfKeyHeld = keyboard_.anyKeyHeld();
    if (keyboard_.model() == KeyboardModel::LiveMixer) {
        const uint8_t prevSlot = snapshots_.activeSlot();
        const auto sHeld = keyboard_.snapKeysHeld();
        snapshots_.onSnapKeysChanged(sHeld, keyboard_.snapKeysPressure(), hud_.config.snapMode,
                                     state_);
        if (hud_.config.snapMode == KeySwitchMode::Latching &&
            snapshots_.activeSlot() != prevSlot) {
            settingsDirty_ = true;
        }
        if (sHeld[0] || sHeld[1] || sHeld[2] || sHeld[3]) {
            hud_.snapshotSavedToast = false;
            hud_.snapshotDeletedToast = false;
        }
        snapshots_.syncHud(hud_);
    }
    keyboard_.evaluate(state_, hud_);
    syncModifierHud();
    publishState();
}

void RuidoController::onEncoderDelta(EncoderId encoder, int32_t ticks) noexcept {
    if (ticks == 0) {
        return;
    }
    const KnobRate rate = hud_.knobRate;

    if (hud_.rateHoldActive) {
        knobTurnedDuringRateHold_ = true;
    }

    if (encoder == EncoderId::Algorithm) {
        if (hud_.configPageOpen) {
            handleConfigEncoder(encoder, ticks, rate);
            return;
        }
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
        if (keyboard_.model() == KeyboardModel::RhythmLab) {
            if (!hud_.midiClockSynced) {
                const float prevBpm = midiClock_.internalBpm();
                const float nextBpm = adjustClockBpm(prevBpm, ticks, rate);
                if (absf(nextBpm - prevBpm) > 1.0e-5f) {
                    midiClock_.setInternalBpm(nextBpm);
                    settingsDirty_ = true;
                    (void)syncClockState();
                    if (keyboard_.anyKeyHeld()) {
                        keyboard_.evaluate(state_, hud_);
                    }
                }
            }
            hud_.knobBank = baseKnobBank();
            triggerHud(HudMode::MasterTempo, hud_.focusedVoice, kNoFocusedKnob);
            publishState();
            return;
        }
        state_.global.circuitCoupling =
            adjustLinear01(state_.global.circuitCoupling, ticks, rate, 0.0f, 1.0f);
        snapshots_.notifyManualEdit(state_);
        snapshots_.syncHud(hud_);
        hud_.knobBank = baseKnobBank();
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
            hud_.knobBank = KnobBankMode::Snapshot;
            if (keyboard_.anyKeyHeld()) {
                keyboard_.evaluate(state_, hud_);
            }
            latchSwitchHud(HudMode::SaveArmed, ButtonId::Save);
            return;
        }
        if (hud_.selPageOpen || (hud_.mode == HudMode::KeyboardModel && !dismissedByHome_)) {
            const KeyboardModel prevModel = keyboard_.model();
            const int32_t maxModel = static_cast<int32_t>(KeyboardModel::Count) - 1;
            const int32_t nextIdx = stepClampedDetent(static_cast<int32_t>(prevModel), maxModel,
                                                      ticks, rate, selectTickAccum_);
            keyboard_.setModel(static_cast<KeyboardModel>(nextIdx));
            if (keyboard_.model() != prevModel) {
                settingsDirty_ = true;
            }
            snapshots_.cancelMomentaryIfActive(state_);
            snapshots_.syncHud(hud_);
            keyboard_.evaluate(state_, hud_);
            hud_.selPageOpen = true;
            ++hud_.selWakeSeq;
            hud_.knobBank = KnobBankMode::None;
            latchSwitchHud(HudMode::KeyboardModel, ButtonId::Sel);
            publishState();
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
        if (hud_.editPageOpen) {
            const int32_t maxEdit = static_cast<int32_t>(EditTarget::Count) - 1;
            hud_.editTarget = static_cast<EditTarget>(stepClampedDetent(
                static_cast<int32_t>(hud_.editTarget), maxEdit, ticks, rate, selectTickAccum_));
            targetTickAccum_ = 0;
            hud_.knobBank = knobBankForEditTarget(hud_.editTarget);
            latchSwitchHud(hudModeForEditTarget(hud_.editTarget), ButtonId::Edit);
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
        hud_.knobBank = hud_.saveArmed      ? KnobBankMode::Snapshot
                        : hud_.freqPageOpen ? KnobBankMode::Frequency
                                            : KnobBankMode::None;
        triggerHud(HudMode::ScopeTimebase, hud_.focusedVoice, kNoFocusedKnob);
        return;
    }

    if (encoder == EncoderId::Presets) {
        if (hud_.configSubView != ConfigSubView::List) {
            exitConfigSubView(true);
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

        std::array<uint8_t, kNumSnapshots> occSlots{};
        int32_t occCount = 0;
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            if (snapshots_.isSlotOccupied(s)) {
                occSlots[static_cast<size_t>(occCount++)] = s;
            }
        }
        if (occCount == 0) {
            (void)stepClampedDetent(0, 0, ticks, rate, presetTickAccum_);
            snapshots_.syncHud(hud_);
            hud_.knobBank = KnobBankMode::Snapshot;
            triggerHud(HudMode::SnapshotSlot, 0u, kNoFocusedKnob);
            return;
        }

        const uint8_t curSlot = snapshots_.activeSlot() & 3u;
        int32_t curOrd = -1;
        for (int32_t i = 0; i < occCount; ++i) {
            if (occSlots[static_cast<size_t>(i)] == curSlot) {
                curOrd = i;
                break;
            }
        }
        uint8_t targetSlot = curSlot;
        bool shouldRecall = false;
        if (curOrd >= 0) {
            const int32_t prevAccum = presetTickAccum_;
            const int32_t nextOrd =
                stepClampedDetent(curOrd, occCount - 1, ticks, rate, presetTickAccum_);
            const bool steppedAtRail = snapshots_.isDirty() && (nextOrd == curOrd) &&
                                       ((prevAccum + ticks >= 4) || (prevAccum + ticks <= -4));
            targetSlot = occSlots[static_cast<size_t>(nextOrd)];
            shouldRecall = (nextOrd != curOrd) || steppedAtRail;
        } else {
            presetTickAccum_ += ticks;
            if (presetTickAccum_ >= 4) {
                presetTickAccum_ %= 4;
                targetSlot = occSlots[static_cast<size_t>(occCount - 1)];
                for (int32_t i = 0; i < occCount; ++i) {
                    if (occSlots[static_cast<size_t>(i)] > curSlot) {
                        targetSlot = occSlots[static_cast<size_t>(i)];
                        break;
                    }
                }
                shouldRecall = true;
            } else if (presetTickAccum_ <= -4) {
                presetTickAccum_ %= 4;
                targetSlot = occSlots[0];
                for (int32_t i = occCount - 1; i >= 0; --i) {
                    if (occSlots[static_cast<size_t>(i)] < curSlot) {
                        targetSlot = occSlots[static_cast<size_t>(i)];
                        break;
                    }
                }
                shouldRecall = true;
            }
        }
        if (shouldRecall) {
            if (targetSlot != curSlot) {
                settingsDirty_ = true;
            }
            snapshots_.recallSlotStepped(targetSlot, rate, state_);
            if (keyboard_.anyKeyHeld()) {
                keyboard_.evaluate(state_, hud_);
            }
            publishState();
        }
        snapshots_.syncHud(hud_);
        hud_.snapshotToastSlot = snapshots_.activeSlot();
        hud_.knobBank = KnobBankMode::Snapshot;
        triggerHud(HudMode::SnapshotSlot, snapshots_.activeSlot(), kNoFocusedKnob);
        return;
    }

    const int knobIdx = static_cast<int>(encoder) - static_cast<int>(EncoderId::Knob1);
    if (knobIdx < 0 || knobIdx >= static_cast<int>(kNumOscillators)) {
        return;
    }
    const uint8_t kIdx = static_cast<uint8_t>(knobIdx);
    if (hud_.configSubView != ConfigSubView::List) {
        exitConfigSubView(true);
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

    snapshots_.notifyManualEdit(state_);
    snapshots_.syncHud(hud_);
    if (keyboard_.anyKeyHeld()) {
        keyboard_.evaluate(state_, hud_);
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
        hud_.knobBank = baseKnobBank();
        triggerHud(HudMode::MasterLevel, hud_.focusedVoice, kNoFocusedKnob);
    }
    publishState();
}

void RuidoController::onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys) {
        return;
    }
    const bool wasHeld = keyboard_.blackHeld()[static_cast<size_t>(key)];
    keyboard_.setBlackGate(key, pressed);
    if (!wasHeld && keyboard_.blackHeld()[static_cast<size_t>(key)] &&
        key == BlackKeyId::ScopeFreezeKey && keyboard_.model() == KeyboardModel::RhythmLab) {
        if (midiClock_.onTapTempo(hud_.config.clockSync)) {
            settingsDirty_ = true;
        }
        (void)syncClockState();
    }
    hud_.lastBlackKey = key;
    applyKeyboardPerformanceChange();
}

void RuidoController::onBlackKeyPressure(BlackKeyId key, float pressure) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys) {
        return;
    }
    const bool wasHeld = keyboard_.blackHeld()[static_cast<size_t>(key)];
    keyboard_.setBlackPressure(key, pressure);
    if (!wasHeld && keyboard_.blackHeld()[static_cast<size_t>(key)] &&
        key == BlackKeyId::ScopeFreezeKey && keyboard_.model() == KeyboardModel::RhythmLab) {
        if (midiClock_.onTapTempo(hud_.config.clockSync)) {
            settingsDirty_ = true;
        }
        (void)syncClockState();
    }
    hud_.lastBlackKey = key;
    applyKeyboardPerformanceChange();
}

bool RuidoController::tryInterceptSnapSaveKey(uint8_t whiteKeyIndex, bool active) noexcept {
    if (whiteKeyIndex < 12u) {
        return false;
    }
    const uint8_t slot = static_cast<uint8_t>(whiteKeyIndex - 12u);
    const uint8_t sBit = static_cast<uint8_t>(1u << slot);
    if ((saveSwallowedMask_ & sBit) != 0u) {
        if (!active) {
            saveSwallowedMask_ &= static_cast<uint8_t>(~sBit);
        }
        return true;
    }
    if (!active || !hud_.saveArmed) {
        return false;
    }
    saveSwallowedMask_ |= sBit;
    const bool deleting = hud_.saveDeleteMode;
    hud_.saveArmed = false;
    hud_.saveDeleteMode = false;
    setLatchedPage(HudMode::None, ButtonId::Count);
    hud_.snapshotToastSlot = slot;
    if (deleting) {
        const bool deleted = snapshots_.deleteSlot(slot, state_);
        hud_.snapshotSavedToast = false;
        hud_.snapshotDeletedToast = deleted;
        if (deleted) {
            savedSlotMask_ |= sBit;
            settingsDirty_ = true;
        }
    } else {
        snapshots_.saveSlot(slot, state_);
        hud_.snapshotSavedToast = true;
        hud_.snapshotDeletedToast = false;
        savedSlotMask_ |= sBit;
        settingsDirty_ = true;
    }
    snapshots_.syncHud(hud_);
    keyboard_.evaluate(state_, hud_);
    hud_.knobBank = KnobBankMode::Snapshot;
    triggerHud(HudMode::SnapshotSlot, slot, kNoFocusedKnob);
    publishState();
    return true;
}

bool RuidoController::loadSnapshotSlot(uint8_t slot, const PatchSnapshot& snap) noexcept {
    if (!snapshots_.loadSlotBinary(slot, snap, state_)) {
        return false;
    }
    snapshots_.syncHud(hud_);
    keyboard_.evaluate(state_, hud_);
    publishState();
    return true;
}

DeviceSettingsBlob RuidoController::exportSettings() const noexcept {
    return encodeDeviceSettings(hud_.config, state_.global, keyboard_.model(),
                                snapshots_.activeSlot(), midiClock_.internalBpm());
}

bool RuidoController::loadSettings(const DeviceSettingsBlob& blob) noexcept {
    GlobalConfigState nextCfg = hud_.config;
    GlobalParams nextGlobal = state_.global;
    KeyboardModel nextModel = keyboard_.model();
    uint8_t nextSlot = snapshots_.activeSlot();
    float nextInternalBpm = midiClock_.internalBpm();
    if (!decodeDeviceSettings(blob, nextCfg, nextGlobal, nextModel, nextSlot, &nextInternalBpm)) {
        return false;
    }
    hud_.config = nextCfg;
    state_.global = nextGlobal;
    midiClock_.setInternalBpm(nextInternalBpm);
    keyboard_.setModel(nextModel);
    keyboard_.syncConfig(hud_.config);
    snapshots_.selectActiveSlotImmediate(nextSlot, state_);
    snapshots_.syncHud(hud_);
    (void)syncClockState();
    keyboard_.evaluate(state_, hud_);
    publishState();
    return true;
}

void RuidoController::onWhiteKeyGate(uint8_t whiteKeyIndex, bool pressed) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys || tryInterceptSnapSaveKey(whiteKeyIndex, pressed)) {
        return;
    }
    keyboard_.setWhiteGate(whiteKeyIndex, pressed);
    applyKeyboardPerformanceChange();
}

void RuidoController::onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys ||
        tryInterceptSnapSaveKey(whiteKeyIndex, pressure > 0.001f)) {
        return;
    }
    keyboard_.setWhitePressure(whiteKeyIndex, pressure);
    applyKeyboardPerformanceChange();
}

void RuidoController::onMidiRealtime(uint8_t statusByte, float msgTimestampSec) noexcept {
    if ((statusByte == 0xFAu || statusByte == 0xFBu || statusByte == 0xFCu) &&
        hud_.config.midiTransport == MidiTransportMode::Off) {
        return;
    }
    (void)midiClock_.onMidiRealtime(statusByte, hud_.config.clockSync, msgTimestampSec);
    bool dirty = syncClockState();
    if (hud_.config.clockSync == ClockSyncMode::Auto &&
        hud_.config.midiTransport == MidiTransportMode::RunStop) {
        if ((statusByte == 0xFAu || statusByte == 0xFBu) && !state_.global.masterRun) {
            state_.global.masterRun = true;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, kNoFocusedKnob);
            dirty = true;
        } else if (statusByte == 0xFCu && state_.global.masterRun) {
            state_.global.masterRun = false;
            triggerHud(HudMode::AudioOutput, hud_.focusedVoice, kNoFocusedKnob);
            dirty = true;
        }
    }
    if (dirty) {
        publishState();
    }
}

void RuidoController::tick(float dtSec, const EngineTelemetry& telemetry) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    (void)midiClock_.step(dt, hud_.config.clockSync);
    if (syncClockState()) {
        publishState();
    }
    const std::array<bool, kNumSnapshots> snapHeld =
        (keyboard_.model() == KeyboardModel::LiveMixer)
            ? keyboard_.snapKeysHeld()
            : std::array<bool, kNumSnapshots>{false, false, false, false};
    const std::array<float, kNumSnapshots> snapPress =
        (keyboard_.model() == KeyboardModel::LiveMixer)
            ? keyboard_.snapKeysPressure()
            : std::array<float, kNumSnapshots>{0.0f, 0.0f, 0.0f, 0.0f};
    const bool wasTransitioning = snapshots_.isTransitioning();
    if (snapshots_.step(dt, snapHeld, snapPress, hud_.config.snapMode, state_)) {
        snapshots_.syncHud(hud_);
        keyboard_.evaluate(state_, hud_);
        publishState();
    } else if (wasTransitioning != snapshots_.isTransitioning()) {
        snapshots_.syncHud(hud_);
    }

    const bool modPinned = hud_.rateHoldActive;
    const bool momentaryPinned = keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen);
    const bool switchLatched = (latchedHudMode_ != HudMode::None);
    hud_.modifierHeld = modPinned;

    bool toastExpired = false;
    bool dissolvedToIdle = false;
    env_.step(dt, momentaryPinned, switchLatched, toastExpired, dissolvedToIdle, hud_.opacity);
    if (toastExpired) {
        hud_.focusedKnob = kNoFocusedKnob;
        if (keyboard_.anyKeyHeld()) {
            restoreOrDissolveHud();
        } else {
            restoreLatchedFields();
        }
    }
    if (dissolvedToIdle) {
        dismissedByHome_ = false;
        hud_.mode = HudMode::None;
        hud_.knobBank = baseKnobBank();
        hud_.focusedKnob = kNoFocusedKnob;
    }

    updateLeds(dt, telemetry);
}

void RuidoController::updateLeds(float dtSec, const EngineTelemetry& telemetry) noexcept {
    // 1. Compute steady functional panel button LEDs + synchronized breath/LFO phases
    const float punchEnergy01 = keyboard_.maxHeldPressure();
    const LedInputContext ctx{state_, hud_, telemetry, punchEnergy01};
    stepAndComputePanelLeds(dtSec, ctx, ledSmooth_, leds_);
    hud_.fxLedSync01 = ledSmooth_.fxSync01;
    hud_.lfoLedSync01 = ledSmooth_.lfoSync01;

    // 2. Compute 27-key zones + momentary full-device 42-LED punch-in choreography
    keyboard_.stepAndComputeKeyLeds(dtSec, ledSmooth_.animPhase, ledSmooth_.subBreathWave01, state_,
                                    hud_, telemetry, leds_);
}

} // namespace ruido
