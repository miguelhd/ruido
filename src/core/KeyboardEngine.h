#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/GlobalConfig.h"
#include "core/KeyboardModel.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// 27-key performance engine (`20` analog pressure pads + `7` permanent switches, `HOLD` latch,
// universal `SHIFT` layer, `TAP TEMPO`, and 42-channel varibright LED choreographer).
class KeyboardEngine {
  public:
    static constexpr float kDefaultGatePressure = 0.78f;

    void reset() noexcept;

    KeyboardModel model() const noexcept { return model_; }
    void setModel(KeyboardModel nextModel) noexcept;
    void syncConfig(const GlobalConfigState& config) noexcept;

    void setBlackGate(BlackKeyId key, bool pressed) noexcept;
    void setBlackPressure(BlackKeyId key, float pressure) noexcept;
    void setWhiteGate(uint8_t whiteIdx, bool pressed) noexcept;
    void setWhitePressure(uint8_t whiteIdx, float pressure) noexcept;

    void clearShiftNotices() noexcept { shiftModelSeq_ = 0u; }
    void clearKeyHold() noexcept;

    bool anyKeyHeld() const noexcept { return (heldBlackMask_ != 0u) || (heldWhiteMask_ != 0u); }
    bool isShiftHeld() const noexcept {
        return (heldBlackMask_ & static_cast<uint16_t>(1u << kShiftBlackKeyIdx)) != 0u;
    }
    bool isKeyHoldLatched() const noexcept { return holdActive_; }
    uint8_t modeWipeQ8() const noexcept {
        return static_cast<uint8_t>(clampf(wipePhase_, 0.0f, 1.0f) * 255.0f + 0.5f);
    }
    float maxHeldPressure() const noexcept;

    bool consumeHoldToggledEvent() noexcept {
        const bool ev = holdToggledEvent_;
        holdToggledEvent_ = false;
        return ev;
    }
    bool consumeTapTempoEvent() noexcept {
        const bool ev = tapTempoEvent_;
        tapTempoEvent_ = false;
        return ev;
    }
    int8_t consumeShiftModelEvent() noexcept {
        const int8_t ev = shiftModelEvent_;
        shiftModelEvent_ = -1;
        return ev;
    }
    int16_t consumeUnassignedChannelEvent() noexcept {
        const int16_t ev = unassignedChannelEvent_;
        unassignedChannelEvent_ = -1;
        return ev;
    }

    // Universal SHIFT + Keys 9-12 Scene recall / pressure morph across ALL 3 keyboard models!
    std::array<bool, kNumSnapshots> snapKeysHeld() const noexcept {
        return {
            (shiftSceneMask_ & 0x01u) != 0u,
            (shiftSceneMask_ & 0x02u) != 0u,
            (shiftSceneMask_ & 0x04u) != 0u,
            (shiftSceneMask_ & 0x08u) != 0u,
        };
    }
    std::array<float, kNumSnapshots> snapKeysPressure() const noexcept {
        constexpr float kInvQ16 = 1.0f / 65535.0f;
        return {
            ((shiftSceneMask_ & 0x01u) != 0u)
                ? static_cast<float>(whitePressureQ16_[kSceneWhiteKeyBase + 0u]) * kInvQ16
                : 0.0f,
            ((shiftSceneMask_ & 0x02u) != 0u)
                ? static_cast<float>(whitePressureQ16_[kSceneWhiteKeyBase + 1u]) * kInvQ16
                : 0.0f,
            ((shiftSceneMask_ & 0x04u) != 0u)
                ? static_cast<float>(whitePressureQ16_[kSceneWhiteKeyBase + 2u]) * kInvQ16
                : 0.0f,
            ((shiftSceneMask_ & 0x08u) != 0u)
                ? static_cast<float>(whitePressureQ16_[kSceneWhiteKeyBase + 3u]) * kInvQ16
                : 0.0f,
        };
    }

    // Evaluate non-destructive DSP overlay & HUD telemetry from live + HOLD-latched pressures.
    void evaluate(EngineControlState& state, HudState& hud) const noexcept;

    // Advance wipe animation & choreograph all 42 device LEDs (27 keys + 14 top panel switches).
    void stepAndComputeKeyLeds(float dtSec, float animPhase, const EngineControlState& state,
                               const HudState& hud, const EngineTelemetry& telemetry,
                               LedState& outLeds) noexcept;

  private:
    bool isBlackHeld(size_t idx) const noexcept {
        return (heldBlackMask_ & static_cast<uint16_t>(1u << idx)) != 0u;
    }
    bool isWhiteHeld(size_t idx) const noexcept {
        return (heldWhiteMask_ & static_cast<uint16_t>(1u << idx)) != 0u;
    }
    uint16_t nextPressSeq() noexcept;
    void onBlackRisingEdge(size_t idx) noexcept;
    void onWhiteRisingEdge(uint8_t whiteIdx) noexcept;
    void buildEffectiveInput(std::array<bool, kNumBlackKeys>& effBlackHeld,
                             std::array<bool, kNumWhiteKeys>& effWhiteHeld,
                             std::array<float, kNumBlackKeys>& effBlackPress,
                             std::array<float, kNumWhiteKeys>& effWhitePress,
                             std::array<uint16_t, kNumBlackKeys>& effBlackSeq,
                             std::array<uint16_t, kNumWhiteKeys>& effWhiteSeq,
                             uint16_t& outEffBlackMask, uint16_t& outEffWhiteMask,
                             uint8_t& outMuteHeld, uint8_t& outSoloHeld) const noexcept;

    KeyboardModel model_ = KeyboardModel::LiveMixer;
    std::array<uint16_t, kNumBlackKeys> blackPressureQ16_{};
    std::array<uint16_t, kNumWhiteKeys> whitePressureQ16_{};
    std::array<uint16_t, kNumBlackKeys> blackPressSeq_{};
    std::array<uint16_t, kNumWhiteKeys> whitePressSeq_{};

    std::array<uint16_t, kNumTopMacroKeys> latchedBlackPressureQ16_{};
    std::array<uint16_t, kMuteWhiteKeyBase> latchedWhitePressureQ16_{};
    uint16_t latchedBlackMask_ = 0u;
    uint16_t latchedWhiteMask_ = 0u;
    uint16_t repressBlackMask_ = 0u;
    uint16_t repressWhiteMask_ = 0u;

    uint16_t heldBlackMask_ = 0u;
    uint16_t heldWhiteMask_ = 0u;
    uint16_t shiftSwallowedBlackMask_ = 0u;
    uint16_t shiftSwallowedWhiteMask_ = 0u;
    uint16_t pressCounter_ = 0u;
    uint16_t shiftModelSeq_ = 0u;
    int16_t unassignedChannelEvent_ = -1;
    uint8_t shiftSceneMask_ = 0u;
    uint8_t shiftSoloHeldMask_ = 0u;
    int8_t shiftModelEvent_ = -1;
    bool holdActive_ = false;
    bool holdToggledEvent_ = false;
    bool tapTempoEvent_ = false;

    KeyboardLatchState latch_{};
    GlobalConfigState config_{};
    float wipePhase_ = 0.0f; // [1.0 -> 0.0] 220 ms signature LED sweep on model change
};

} // namespace ruido
