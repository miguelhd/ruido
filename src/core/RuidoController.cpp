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
    capsuleTimerTenthsMs_ = 0u;
    selectTickAccum_ = targetTickAccum_ = configTickAccum_ = 0;
    shiftBlinkPhaseQ8_ = 64u;
    knobFocusMask_ = 0u;
    knobTurnHoldQ8_.fill(0u);

    keyboard_.evaluate(state_, hud_);
    syncFocusedColumnMask();
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
constexpr uint16_t kCapsuleFadeTenthsMs = 1500u;  // 150.0 ms fade-in / fade-out
constexpr uint16_t kCapsuleHoldTenthsMs = 10000u; // 1000.0 ms (1.0 s) hold
constexpr uint16_t kCapsuleTotalTenthsMs =
    static_cast<uint16_t>(kCapsuleFadeTenthsMs + kCapsuleHoldTenthsMs + kCapsuleFadeTenthsMs);

uint8_t computeCapsuleAlphaQ8(uint16_t timerTenthsMs) noexcept {
    if (timerTenthsMs == 0u) {
        return 0u;
    }
    constexpr uint16_t kHoldStartTenthsMs =
        static_cast<uint16_t>(kCapsuleFadeTenthsMs + kCapsuleHoldTenthsMs);
    if (timerTenthsMs > kHoldStartTenthsMs) {
        const uint16_t elapsed = static_cast<uint16_t>(kCapsuleTotalTenthsMs - timerTenthsMs);
        const float t =
            static_cast<float>(elapsed) * (1.0f / static_cast<float>(kCapsuleFadeTenthsMs));
        const uint8_t a8 = static_cast<uint8_t>(smoothstep01(t) * 255.0f + 0.5f);
        return (a8 > 0u) ? a8 : 1u;
    }
    if (timerTenthsMs >= kCapsuleFadeTenthsMs) {
        return 255u;
    }
    const float t =
        static_cast<float>(timerTenthsMs) * (1.0f / static_cast<float>(kCapsuleFadeTenthsMs));
    return static_cast<uint8_t>(smoothstep01(t) * 255.0f + 0.5f);
}

void RuidoController::triggerCapsuleToast(CapsuleToastKind kind) noexcept {
    if (kind == CapsuleToastKind::None) {
        capsuleTimerTenthsMs_ = 0u;
        hud_.capsuleToast = CapsuleToastKind::None;
        hud_.capsuleAlphaQ8 = 0u;
        return;
    }
    constexpr uint16_t kHoldStartTenthsMs =
        static_cast<uint16_t>(kCapsuleFadeTenthsMs + kCapsuleHoldTenthsMs);
    hud_.capsuleToast = kind;
    if (capsuleTimerTenthsMs_ == 0u) {
        capsuleTimerTenthsMs_ = kCapsuleTotalTenthsMs;
    } else if (capsuleTimerTenthsMs_ < kCapsuleFadeTenthsMs) {
        capsuleTimerTenthsMs_ =
            static_cast<uint16_t>(kCapsuleTotalTenthsMs - capsuleTimerTenthsMs_);
    } else if (capsuleTimerTenthsMs_ <= kHoldStartTenthsMs) {
        capsuleTimerTenthsMs_ = kHoldStartTenthsMs;
    }
    hud_.capsuleAlphaQ8 = computeCapsuleAlphaQ8(capsuleTimerTenthsMs_);
}

void RuidoController::stepCapsuleToast(float dtSec) noexcept {
    if (capsuleTimerTenthsMs_ == 0u) {
        hud_.capsuleToast = CapsuleToastKind::None;
        hud_.capsuleAlphaQ8 = 0u;
        return;
    }
    const bool isRateCapsule = (hud_.capsuleToast == CapsuleToastKind::Fine ||
                                hud_.capsuleToast == CapsuleToastKind::Coarse);
    if (isRateCapsule && !hud_.rateHoldActive) {
        capsuleTimerTenthsMs_ = 0u;
        hud_.capsuleToast = CapsuleToastKind::None;
        hud_.capsuleAlphaQ8 = 0u;
        return;
    }
    constexpr uint16_t kHoldStartTenthsMs =
        static_cast<uint16_t>(kCapsuleFadeTenthsMs + kCapsuleHoldTenthsMs);
    const uint32_t stepTenths = static_cast<uint32_t>(dtSec * 10000.0f + 0.5f);
    if (isRateCapsule && hud_.rateHoldActive) {
        if (capsuleTimerTenthsMs_ > kHoldStartTenthsMs) {
            const uint32_t remFade = capsuleTimerTenthsMs_ - kHoldStartTenthsMs;
            capsuleTimerTenthsMs_ = (stepTenths >= remFade)
                                        ? kHoldStartTenthsMs
                                        : static_cast<uint16_t>(capsuleTimerTenthsMs_ - stepTenths);
        } else {
            capsuleTimerTenthsMs_ = kHoldStartTenthsMs;
        }
        hud_.capsuleAlphaQ8 = computeCapsuleAlphaQ8(capsuleTimerTenthsMs_);
        return;
    }
    if (stepTenths >= capsuleTimerTenthsMs_) {
        capsuleTimerTenthsMs_ = 0u;
        hud_.capsuleToast = CapsuleToastKind::None;
        hud_.capsuleAlphaQ8 = 0u;
        return;
    }
    capsuleTimerTenthsMs_ = static_cast<uint16_t>(capsuleTimerTenthsMs_ - stepTenths);
    hud_.capsuleAlphaQ8 = computeCapsuleAlphaQ8(capsuleTimerTenthsMs_);
    if (hud_.capsuleAlphaQ8 == 0u && capsuleTimerTenthsMs_ < kCapsuleFadeTenthsMs) {
        capsuleTimerTenthsMs_ = 0u;
        hud_.capsuleToast = CapsuleToastKind::None;
    }
}

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
        hud_.savePageMode = SavePageMode::Store;
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
        hud_.snapshotToast = SnapshotToastKind::None;
        hud_.shiftFocus = ShiftFocus::Scene;
    }
    snapshots_.syncHud(hud_);

    if (keyboard_.consumeTapTempoEvent()) {
        if (midiClock_.onTapTempo(hud_.config.clockSync)) {
            settingsDirty_ = true;
        }
        (void)syncClockState();
    }
    if (keyboard_.consumeHoldToggledEvent()) {
        triggerCapsuleToast(keyboard_.isKeyHoldLatched() ? CapsuleToastKind::LatchOn
                                                         : CapsuleToastKind::LatchOff);
    }

    keyboard_.evaluate(state_, hud_);
    const bool keepHoldInPerfMask =
        hud_.selPageOpen ||
        (hud_.configPageOpen && hud_.configSubView == ConfigSubView::List &&
         hud_.configCursor == static_cast<uint8_t>(ConfigItemId::PressureTouch));
    const uint16_t hudBlackMask =
        keepHoldInPerfMask ? hud_.heldBlackMask
                           : static_cast<uint16_t>(hud_.heldBlackMask &
                                                   ~static_cast<uint16_t>(1u << kHoldBlackKeyIdx));
    keys_.anyPerfKeyHeld =
        (hudBlackMask != 0u) || (hud_.heldWhiteMask != 0u) || keyboard_.isShiftHeld();
    if (!hud_.shiftHeld) {
        hud_.shiftFocus = ShiftFocus::Idle;
    } else if (hud_.activePunch == KeyPunchKind::SpotlightSolo) {
        hud_.shiftFocus = ShiftFocus::Idle;
    }
    if (hud_.lastBlackKey != BlackKeyId::Mono || keepHoldInPerfMask || keys_.anyPerfKeyHeld) {
        syncModifierHud();
    }
    syncFocusedColumnMask();
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

void RuidoController::commitOrDeferKeyboardChange() noexcept {
    if (keyBatchActive_) {
        keyBatchDirty_ = true;
        return;
    }
    applyKeyboardPerformanceChange();
}

void RuidoController::endKeyBatch() noexcept {
    keyBatchActive_ = false;
    if (keyBatchDirty_) {
        keyBatchDirty_ = false;
        applyKeyboardPerformanceChange();
    }
}

void RuidoController::onBlackKeyEvent(BlackKeyId key, bool pressed) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys || tryInterceptSaveBlackKey(key, pressed)) {
        return;
    }
    if (key == BlackKeyId::ScopeFreezeKey && pressed && !keyboard_.isShiftHeld()) {
        snapShiftBlinkToPeak();
        hud_.shiftFocus = ShiftFocus::Idle;
        triggerCapsuleToast(CapsuleToastKind::None);
    }
    keyboard_.setBlackGate(key, pressed);
    hud_.lastBlackKey = key;
    commitOrDeferKeyboardChange();
}

void RuidoController::onBlackKeyPressure(BlackKeyId key, float pressure) noexcept {
    if (static_cast<size_t>(key) >= kNumBlackKeys ||
        tryInterceptSaveBlackKey(key, pressure > 0.001f)) {
        return;
    }
    if (key == BlackKeyId::ScopeFreezeKey && pressure > 0.001f && !keyboard_.isShiftHeld()) {
        snapShiftBlinkToPeak();
        hud_.shiftFocus = ShiftFocus::Idle;
        triggerCapsuleToast(CapsuleToastKind::None);
    }
    keyboard_.setBlackPressure(key, pressure);
    hud_.lastBlackKey = key;
    commitOrDeferKeyboardChange();
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
    const SavePageMode pageMode = hud_.savePageMode;
    if (pageMode == SavePageMode::Delete && !snapshots_.isSlotOccupied(slot)) {
        // Unassigned feedback (CRT ripple + 120 ms 15% glint) on 0%-dark EMPTY slot in DELETE SCENE
        triggerUnassignedFeedback(phantomWhiteKeyChannel(whiteKeyIndex));
        return true;
    }
    hud_.saveArmed = false;
    hud_.savePageMode = SavePageMode::Store;
    setLatchedPage(HudMode::None, ButtonId::Count);
    hud_.snapshotToastSlot = slot;
    if (pageMode == SavePageMode::Delete) {
        const bool deleted = snapshots_.deleteSlot(slot, state_);
        hud_.snapshotToast = deleted ? SnapshotToastKind::Deleted : SnapshotToastKind::None;
        if (deleted) {
            savedSlotMask_ |= sBit;
            settingsDirty_ = true;
            livePatchDirty_ = true;
        }
    } else if (pageMode == SavePageMode::Init) {
        snapshots_.initSlot(slot, state_);
        hud_.snapshotToast = SnapshotToastKind::Initialized;
        savedSlotMask_ |= sBit;
        settingsDirty_ = true;
        livePatchDirty_ = true;
    } else {
        snapshots_.saveSlot(slot, state_);
        hud_.snapshotToast = SnapshotToastKind::Saved;
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
    commitOrDeferKeyboardChange();
}

void RuidoController::onWhiteKeyPressure(uint8_t whiteKeyIndex, float pressure) noexcept {
    if (whiteKeyIndex >= kNumWhiteKeys ||
        tryInterceptSnapSaveKey(whiteKeyIndex, pressure > 0.001f)) {
        return;
    }
    keyboard_.setWhitePressure(whiteKeyIndex, pressure);
    commitOrDeferKeyboardChange();
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

void RuidoController::tick(float dtSec, const EngineTelemetry& telemetry,
                           void (*coProcYield)() noexcept) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    if (dt > 0.0f) {
        constexpr float kMultiKnobChordWindowSec = 0.350f;
        const uint32_t decQ8 = static_cast<uint32_t>(
            clampf(dt * (1.0f / kMultiKnobChordWindowSec), 0.0f, 1.0f) * 255.0f + 0.99f);
        for (uint8_t& holdQ8 : knobTurnHoldQ8_) {
            holdQ8 = (holdQ8 > decQ8) ? static_cast<uint8_t>(holdQ8 - decQ8) : 0u;
        }
    }
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
        if (coProcYield != nullptr) {
            coProcYield();
        }
        keyboard_.evaluate(state_, hud_);
        publishState();
    } else if (wasTransitioning != snapshots_.isTransitioning()) {
        snapshots_.syncHud(hud_);
    }
    if (coProcYield != nullptr) {
        coProcYield();
    }

    const bool modPinned = hud_.rateHoldActive;
    const bool momentaryPinned = keys_.isAnyMomentaryHudHoldActive(hud_.scopeFrozen) ||
                                 (hud_.rateHoldActive && knobTurnedDuringRateHold_);
    const bool switchLatched = (latchedHudMode_ != HudMode::None);
    hud_.modifierHeld = modPinned;

    bool toastExpired = false;
    bool dissolvedToIdle = false;
    env_.step(dt, momentaryPinned, switchLatched, toastExpired, dissolvedToIdle, hud_.opacity);
    if (toastExpired) {
        clearKnobFocus();
        if (keys_.isAnyPerformanceKeyHeld()) {
            restoreOrDissolveHud();
        } else {
            restoreLatchedFields();
        }
    }
    if (dissolvedToIdle) {
        dismissedByHome_ = false;
        hud_.mode = HudMode::None;
        hud_.knobBank = baseKnobBank();
        clearKnobFocus();
    }
    syncFocusedColumnMask();
    stepCapsuleToast(dt);
    if (coProcYield != nullptr) {
        coProcYield();
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
    applyPresetKnobSceneLeds(dtSec, hud_, env_.holdRemainingSec > 0.0f, ledSmooth_, leds_);
    applyUnassignedLedGlints(ledSmooth_, leds_);
}

} // namespace ruido
