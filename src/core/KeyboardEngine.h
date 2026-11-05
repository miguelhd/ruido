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

// 27-key performance engine & EP-133-inspired varibright LED choreographer.
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

    bool anyKeyHeld() const noexcept { return (heldBlackMask_ != 0u) || (heldWhiteMask_ != 0u); }
    float maxHeldPressure() const noexcept;

    const std::array<bool, kNumBlackKeys>& blackHeld() const noexcept { return blackHeld_; }
    const std::array<bool, kNumWhiteKeys>& whiteHeld() const noexcept { return whiteHeld_; }

    std::array<bool, kNumSnapshots> snapKeysHeld() const noexcept {
        return {whiteHeld_[12], whiteHeld_[13], whiteHeld_[14], whiteHeld_[15]};
    }
    std::array<float, kNumSnapshots> snapKeysPressure() const noexcept {
        return {whitePressure_[12], whitePressure_[13], whitePressure_[14], whitePressure_[15]};
    }

    // Evaluate non-destructive DSP overlay & HUD telemetry from current 27-key pressures.
    void evaluate(EngineControlState& state, HudState& hud) const noexcept;

    // Advance wipe animation & choreograph all 42 device LEDs (27 keys + 14 top panel switches).
    void stepAndComputeKeyLeds(float dtSec, float animPhase, float subBreathWave01,
                               const EngineControlState& state, const HudState& hud,
                               const EngineTelemetry& telemetry, LedState& outLeds) noexcept;

  private:
    uint16_t nextPressSeq() noexcept;

    KeyboardModel model_ = KeyboardModel::LiveMixer;
    std::array<bool, kNumBlackKeys> blackHeld_{};
    std::array<bool, kNumWhiteKeys> whiteHeld_{};
    std::array<float, kNumBlackKeys> blackPressure_{};
    std::array<float, kNumWhiteKeys> whitePressure_{};
    std::array<uint16_t, kNumBlackKeys> blackPressSeq_{};
    std::array<uint16_t, kNumWhiteKeys> whitePressSeq_{};
    uint16_t pressCounter_ = 0u;
    uint16_t heldBlackMask_ = 0u;
    uint16_t heldWhiteMask_ = 0u;
    KeyboardLatchState latch_{};
    GlobalConfigState config_{};
    float wipePhase_ = 0.0f; // [1.0 -> 0.0] 220 ms signature LED sweep on model change
};

} // namespace ruido
