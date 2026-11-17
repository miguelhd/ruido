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
    applyPatchFrame(decodePatchSnapshot(snapshots_.slotBinary(0u)), state_);
    snapshots_.syncHud(hud_);
    midiClock_.reset();
    (void)syncClockState();
    keys_.reset();
    env_.reset();
    ledSmooth_.reset(state_);
    knobTurnedDuringRateHold_ = false;
    dismissedByHome_ = false;
    editRememberPan_ = false;
    settingsDirty_ = false;
    livePatchDirty_ = false;
    saveSwallowedWhiteMask_ = 0u;
    saveSwallowedBlackMask_ = 0u;
    savedSlotMask_ = 0u;
    latchedHudMode_ = HudMode::None;
    latchedButton_ = ButtonId::Count;
    selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
    shiftBlinkPhaseQ8_ = 64u;

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

constexpr float kShiftModelPulseHz = 0.50f;

void RuidoController::snapShiftBlinkToPeak() noexcept {
    const float basePh = wrapLfoPhase01(ledSmooth_.animPhase * kShiftModelPulseHz);
    shiftBlinkPhaseQ8_ = static_cast<uint8_t>(
        static_cast<uint32_t>(wrapLfoPhase01(1.25f - basePh) * 256.0f) & 0xFFu);
}

void RuidoController::triggerUnassignedFeedback(uint8_t channel) noexcept {
    hud_.phantomPinIndex = channel;
    ++hud_.phantomPinSeq;
}

void RuidoController::applyKeyboardModel(KeyboardModel nextModel) noexcept {
    const KeyboardModel prevModel = keyboard_.model();
    keyboard_.setModel(nextModel);
    if (keyboard_.model() != prevModel) {
        settingsDirty_ = true;
    }
    snapshots_.cancelMomentaryIfActive(state_);
    snapshots_.syncHud(hud_);
    hud_.selPageOpen = true;
    ++hud_.selWakeSeq;
    keyboard_.evaluate(state_, hud_);
    hud_.knobBank = KnobBankMode::None;
    latchSwitchHud(HudMode::KeyboardModel, ButtonId::Sel);
    publishState();
}

void RuidoController::applyKeyboardPerformanceChange() noexcept {
    keys_.anyPerfKeyHeld = keyboard_.anyKeyHeld() || keyboard_.isShiftHeld();
    if (keyboard_.isShiftHeld() && hud_.saveArmed) {
        hud_.saveArmed = false;
        hud_.saveDeleteMode = false;
        if (latchedHudMode_ == HudMode::SaveArmed) {
            setLatchedPage(HudMode::None, ButtonId::Count);
        }
    }

    const int16_t unassCh = keyboard_.consumeUnassignedChannelEvent();
    if (unassCh >= 0) {
        triggerUnassignedFeedback(static_cast<uint8_t>(unassCh));
    }

    const int8_t shiftModel = keyboard_.consumeShiftModelEvent();
    if (shiftModel >= 0) {
        const KeyboardModel prevModel = keyboard_.model();
        keyboard_.setModel(static_cast<KeyboardModel>(shiftModel));
        if (keyboard_.model() != prevModel) {
            settingsDirty_ = true;
        }
        snapShiftBlinkToPeak();
        hud_.shiftFocus = ShiftFocus::Model;
        snapshots_.cancelMomentaryIfActive(state_);
        snapshots_.syncHud(hud_);
        if (hud_.selPageOpen) {
            ++hud_.selWakeSeq;
        }
    }

    const uint8_t prevSlot = snapshots_.activeSlot();
    const bool prevDirty = snapshots_.isDirty();
    const auto sHeld = keyboard_.snapKeysHeld();
    snapshots_.onSnapKeysChanged(sHeld, keyboard_.snapKeysPressure(), hud_.config.snapMode, state_);
    if (hud_.config.snapMode == KeySwitchMode::Latching) {
        if (snapshots_.activeSlot() != prevSlot) {
            settingsDirty_ = true;
        }
        if (snapshots_.isDirty() != prevDirty || snapshots_.isDirty()) {
            livePatchDirty_ = true;
        }
    }
    if (sHeld[0] || sHeld[1] || sHeld[2] || sHeld[3]) {
        hud_.snapshotSavedToast = false;
        hud_.snapshotDeletedToast = false;
        hud_.shiftFocus = ShiftFocus::Scene;
    }
    snapshots_.syncHud(hud_);

    if (keyboard_.consumeTapTempoEvent()) {
        if (midiClock_.onTapTempo(hud_.config.clockSync)) {
            settingsDirty_ = true;
        }
        (void)syncClockState();
    }
    (void)keyboard_.consumeHoldToggledEvent();

    keyboard_.evaluate(state_, hud_);
    if (!hud_.shiftHeld) {
        hud_.shiftFocus = ShiftFocus::Idle;
    } else if (hud_.activePunch == KeyPunchKind::SpotlightSolo) {
        hud_.shiftFocus = ShiftFocus::Idle;
    }
    syncModifierHud();
    publishState();
}

bool RuidoController::tryInterceptSaveBlackKey(BlackKeyId key, bool active) noexcept {
    const size_t idx = static_cast<size_t>(key);
    if (idx >= kNumBlackKeys || idx == kShiftBlackKeyIdx) {
        return false;
    }
    const uint16_t bBit = static_cast<uint16_t>(1u << idx);
    if ((saveSwallowedBlackMask_ & bBit) != 0u) {
        if (!active) {
            saveSwallowedBlackMask_ &= static_cast<uint16_t>(~bBit);
        }
        return true;
    }
    if (!active || !hud_.saveArmed || (hud_.heldBlackMask & bBit) != 0u) {
        return false;
    }
    saveSwallowedBlackMask_ |= bBit;
    triggerUnassignedFeedback(phantomBlackKeyChannel(idx));
    return true;
}

void RuidoController::onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys || tryInterceptSaveBlackKey(key, pressed)) {
        return;
    }
    if (key == BlackKeyId::ScopeFreezeKey && pressed && !keyboard_.isShiftHeld()) {
        snapShiftBlinkToPeak();
        hud_.shiftFocus = ShiftFocus::Idle;
    }
    keyboard_.setBlackGate(key, pressed);
    hud_.lastBlackKey = key;
    applyKeyboardPerformanceChange();
}

void RuidoController::onBlackKeyPressure(BlackKeyId key, float pressure) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys ||
        tryInterceptSaveBlackKey(key, pressure > 0.001f)) {
        return;
    }
    if (key == BlackKeyId::ScopeFreezeKey && pressure > 0.001f && !keyboard_.isShiftHeld()) {
        snapShiftBlinkToPeak();
        hud_.shiftFocus = ShiftFocus::Idle;
    }
    keyboard_.setBlackPressure(key, pressure);
    hud_.lastBlackKey = key;
    applyKeyboardPerformanceChange();
}

bool RuidoController::tryInterceptSnapSaveKey(uint8_t whiteKeyIndex, bool active) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys) {
        return false;
    }
    const uint16_t wBit = static_cast<uint16_t>(1u << whiteKeyIndex);
    if ((saveSwallowedWhiteMask_ & wBit) != 0u) {
        if (!active) {
            saveSwallowedWhiteMask_ &= static_cast<uint16_t>(~wBit);
        }
        return true;
    }
    if (!active || !hud_.saveArmed) {
        return false;
    }
    if (whiteKeyIndex < kSceneWhiteKeyBase || whiteKeyIndex >= kSceneWhiteKeyBase + kNumSnapshots) {
        if ((hud_.heldWhiteMask & wBit) != 0u) {
            return false;
        }
        saveSwallowedWhiteMask_ |= wBit;
        triggerUnassignedFeedback(phantomWhiteKeyChannel(whiteKeyIndex));
        return true;
    }
    const uint8_t slot = static_cast<uint8_t>(whiteKeyIndex - kSceneWhiteKeyBase);
    const uint8_t sBit = static_cast<uint8_t>(1u << slot);
    saveSwallowedWhiteMask_ |= wBit;
    if (hud_.saveDeleteMode && !snapshots_.isSlotOccupied(slot)) {
        // Unassigned feedback (CRT ripple + 120 ms 15% glint) on 0%-dark EMPTY slot in DELETE SCENE
        triggerUnassignedFeedback(phantomWhiteKeyChannel(whiteKeyIndex));
        return true;
    }
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
            livePatchDirty_ = true;
        }
    } else {
        snapshots_.saveSlot(slot, state_);
        hud_.snapshotSavedToast = true;
        hud_.snapshotDeletedToast = false;
        savedSlotMask_ |= sBit;
        settingsDirty_ = true;
        livePatchDirty_ = true;
    }
    snapshots_.syncHud(hud_);
    keyboard_.evaluate(state_, hud_);
    hud_.knobBank = KnobBankMode::None;
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

bool RuidoController::loadLivePatch(const PatchSnapshot& snap) noexcept {
    if (!snapshots_.restoreLiveSnapshot(snap, state_)) {
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
    const std::array<bool, kNumSnapshots> snapHeld = keyboard_.snapKeysHeld();
    const std::array<float, kNumSnapshots> snapPress = keyboard_.snapKeysPressure();
    const bool wasTransitioning = snapshots_.isTransitioning();
    if (snapshots_.step(dt, snapHeld, snapPress, hud_.config.snapMode, state_)) {
        if (hud_.config.snapMode == KeySwitchMode::Latching &&
            (snapshots_.isDirty() || !snapshots_.isTransitioning())) {
            livePatchDirty_ = true;
        }
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
    hud_.savePulseQ8 =
        static_cast<uint8_t>(clampf(ledSmooth_.savePulse01, 0.0f, 1.0f) * 255.0f + 0.5f);
    // 0.50 Hz slow SAVE-style pulse (exact 500x multiple of 1000 s animPhase wrap, 5x slower than
    // SAVE's 2.50 Hz flash) for SHIFT OP1..OP3
    const float blinkPhase =
        wrapLfoPhase01(ledSmooth_.animPhase * kShiftModelPulseHz +
                       static_cast<float>(shiftBlinkPhaseQ8_) * (1.0f / 256.0f));
    const float shiftBlink01 = 0.5f * (1.0f + evalAnalogSine(blinkPhase));
    hud_.shiftBlinkQ8 = static_cast<uint8_t>(clampf(shiftBlink01, 0.0f, 1.0f) * 255.0f + 0.5f);

    // 2. Compute 27-key zones + momentary full-device 42-LED punch-in choreography + unassigned
    // glints
    keyboard_.stepAndComputeKeyLeds(dtSec, ledSmooth_.animPhase, state_, hud_, telemetry, leds_);
    hud_.modeWipeQ8 = keyboard_.modeWipeQ8();
    applyUnassignedLedGlints(ledSmooth_, leds_);
}

} // namespace ruido
