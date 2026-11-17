#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/GlobalConfig.h"
#include "core/Parameters.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Per-voice quantized binary snapshot payload (30 bytes exact, alignof == 2).
struct VoiceSnapshotBin {
    uint16_t freqLogQ16 = 0u;
    uint16_t pulseWidthQ16 = 0u;
    uint16_t driftQ16 = 0u;
    uint16_t hpCutLogQ16 = 0u;
    uint16_t hpResQ16 = 0u;
    uint16_t lpCutLogQ16 = 0u;
    uint16_t lpResQ16 = 0u;
    uint16_t amplitudeQ16 = 0u;
    uint16_t panBiQ16 = 0u;
    uint16_t lfoRateLogQ16 = 0u;
    uint16_t lfoDepthQ16 = 0u;
    uint16_t lfoBRateLogQ16 = 0u;
    uint16_t lfoBDepthQ16 = 0u;
    uint8_t lfoShapeQ8 = 0u;
    uint8_t lfoBShapeQ8 = 0u;
    uint8_t lfoTargetsPacked = 0u;
    uint8_t flags = 0u;
};
static_assert(sizeof(VoiceSnapshotBin) == 30u, "VoiceSnapshotBin must be 30 bytes");

inline constexpr uint8_t kSnapshotFlagOccupied = 0x01u;

// CRC-verified 128-byte binary patch snapshot ready for FM-1 NVS flash (`slot_*`, ADR-025).
// `flags & kSnapshotFlagOccupied`: 1 = valid stored scene, 0 = user-deleted (`EMPTY`) scene.
struct PatchSnapshot {
    uint32_t crc32 = 0u;
    uint16_t circuitCouplingQ16 = 0u;
    uint8_t version = 1u;
    uint8_t flags = kSnapshotFlagOccupied;
    std::array<VoiceSnapshotBin, kNumOscillators> voices{};
};
static_assert(sizeof(PatchSnapshot) == 128u, "PatchSnapshot must be exactly 128 bytes");
static_assert(__is_trivially_copyable(PatchSnapshot),
              "PatchSnapshot must remain trivially copyable");

// Decoded floating-point synthesis patch frame (excludes hardware Master pot & Global Config).
struct PatchFrame {
    std::array<VoiceParams, kNumOscillators> voices{};
    float circuitCoupling = 0.25f;
};

PatchSnapshot encodePatchSnapshot(const EngineControlState& state) noexcept;
PatchSnapshot makeEmptyPatchSnapshot() noexcept;
constexpr bool isPatchSnapshotOccupied(const PatchSnapshot& snap) noexcept {
    return (snap.flags & kSnapshotFlagOccupied) != 0u;
}
PatchFrame decodePatchSnapshot(const PatchSnapshot& snap) noexcept;
bool verifyPatchSnapshotCrc(const PatchSnapshot& snap) noexcept;

PatchFrame extractPatchFrame(const EngineControlState& state) noexcept;
void applyPatchFrame(const PatchFrame& patch, EngineControlState& state) noexcept;
PatchFrame morphPatchFrames(const PatchFrame& a, const PatchFrame& b, float t01) noexcept;
PatchFrame blendPatchSnapshots(const std::array<PatchSnapshot, kNumSnapshots>& slots,
                               const std::array<float, kNumSnapshots>& normWeights,
                               const PatchFrame& fallbackFrame) noexcept;
EngineControlState makeFactorySnapshotPreset(size_t slot) noexcept;

// 4-slot Snapshot Bank + endless 360-deg circular morph, 1-tap Snap-Back & pressure-morph engine.
class SnapshotBank {
  public:
    SnapshotBank() noexcept { reset(); }

    void reset() noexcept;

    // Clear all 4 Scene slots to CRC-valid `EMPTY` markers and apply
    // `EngineControlState::makeDefault()`.
    void resetEmpty(EngineControlState& state) noexcept;

    uint8_t activeSlot() const noexcept { return activeSlot_; }
    bool isSlotOccupied(size_t idx) const noexcept {
        return isPatchSnapshotOccupied(slots_[idx & 3u]);
    }
    uint8_t occupiedMask() const noexcept;
    bool isDirty() const noexcept { return dirty_; }
    bool isTransitioning() const noexcept { return latchGliding_ || momentaryActive_; }
    float morphPosition() const noexcept { return morphPos_; }
    float visualLoopPhase() const noexcept { return visualLoopPhase_; }
    const std::array<float, kNumSnapshots>& slotWeights() const noexcept { return slotWeights_; }
    const PatchSnapshot& slotBinary(size_t idx) const noexcept { return slots_[idx & 3u]; }

    // Export/restore persistent 128-byte LIVE scratchpad patch across power cycles (`occupied` when
    // `isDirty() == true`, `empty` marker when sitting cleanly on a pure stored Scene).
    PatchSnapshot exportLiveSnapshot(const EngineControlState& state) const noexcept;
    bool restoreLiveSnapshot(const PatchSnapshot& snap, EngineControlState& state) noexcept;

    // Store current synthesis patch into slot 0..3 (`SAVE` + Key 9-12).
    void saveSlot(uint8_t slot, EngineControlState& state) noexcept;

    // Clear slot 0..3 into a CRC-valid `EMPTY` state (`SAVE` -> `DELETE SCENE` + Key 9-12);
    // returns true if the slot held a scene prior to deletion.
    bool deleteSlot(uint8_t slot, EngineControlState& state) noexcept;

    // Load a CRC-verified binary snapshot from FM-1 NVS flash (`dinghy` `slot_*`) into slot 0..3;
    // returns false and preserves the existing/factory slot if `snap` fails version or CRC checks.
    bool loadSlotBinary(uint8_t slot, const PatchSnapshot& snap,
                        EngineControlState& state) noexcept;

    // Immediately activate slot 0..3 without starting a glide (used on NVS boot restore).
    void selectActiveSlotImmediate(uint8_t slot, EngineControlState& state) noexcept;

    // Stepped recall via left PRESETS encoder (`OCT+` Coarse circular scene step).
    void recallSlotStepped(uint8_t slot, KnobRate rate, EngineControlState& state) noexcept;

    // Endless circular 1 -> 2 -> 3 -> 4 -> 1 morph via left PRESETS encoder across all models.
    bool morphByEncoder(int32_t ticks, KnobRate rate, EngineControlState& state) noexcept;

    // Update LIVE MIXER Keys 9-12 gate/pressure state (Latching glide/blend or Momentary morph).
    bool onSnapKeysChanged(const std::array<bool, kNumSnapshots>& held,
                           const std::array<float, kNumSnapshots>& pressure, KeySwitchMode snapMode,
                           EngineControlState& state) noexcept;

    // Advance active latching glide, multi-finger pressure sculpt, or momentary morph; returns true
    // if `state` changed and should be published to Core 1.
    bool step(float dtSec, const std::array<bool, kNumSnapshots>& held,
              const std::array<float, kNumSnapshots>& pressure, KeySwitchMode snapMode,
              EngineControlState& state) noexcept;

    // Cancel any active glide at current `state` when the user manually turns a synthesis knob.
    void notifyManualEdit(EngineControlState& state) noexcept;

    // Cleanly restore underlying home patch if Momentary morph was active during mode/model change.
    bool cancelMomentaryIfActive(EngineControlState& state) noexcept;

    void syncHud(HudState& hud) const noexcept;

  private:
    static std::array<uint16_t, kNumSnapshots>
    packWeightsQ16(const std::array<float, kNumSnapshots>& w) noexcept {
        return {
            static_cast<uint16_t>((w[0] < 0.0f ? 0.0f : (w[0] > 1.0f ? 1.0f : w[0])) * 65535.0f +
                                  0.5f),
            static_cast<uint16_t>((w[1] < 0.0f ? 0.0f : (w[1] > 1.0f ? 1.0f : w[1])) * 65535.0f +
                                  0.5f),
            static_cast<uint16_t>((w[2] < 0.0f ? 0.0f : (w[2] > 1.0f ? 1.0f : w[2])) * 65535.0f +
                                  0.5f),
            static_cast<uint16_t>((w[3] < 0.0f ? 0.0f : (w[3] > 1.0f ? 1.0f : w[3])) * 65535.0f +
                                  0.5f),
        };
    }
    static std::array<float, kNumSnapshots>
    unpackWeightsQ16(const std::array<uint16_t, kNumSnapshots>& q) noexcept {
        constexpr float kInv = 1.0f / 65535.0f;
        return {
            static_cast<float>(q[0]) * kInv,
            static_cast<float>(q[1]) * kInv,
            static_cast<float>(q[2]) * kInv,
            static_cast<float>(q[3]) * kInv,
        };
    }
    PatchFrame blendSlots(const std::array<float, kNumSnapshots>& normWeights) const noexcept {
        return blendPatchSnapshots(slots_, normWeights, originFrame_);
    }
    void syncKnobMorphAnchor() noexcept;
    void applyMomentaryWeightsAndPhase(const std::array<float, kNumSnapshots>& tgtW) noexcept;

    std::array<PatchSnapshot, kNumSnapshots> slots_{};
    PatchFrame originFrame_{};
    std::array<float, kNumSnapshots> slotWeights_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<uint16_t, kNumSnapshots> originWeightsQ16_{65535u, 0u, 0u, 0u};
    std::array<uint16_t, kNumSnapshots> targetWeightsQ16_{65535u, 0u, 0u, 0u};
    std::array<bool, kNumSnapshots> prevHeld_{false, false, false, false};
    float glideProgress_ = 1.0f;
    float glideDurationSec_ = 0.50f;
    float momentaryBlend_ = 0.0f;
    float morphPos_ = 1.0f;
    float loopPhase_ = 0.0f;
    float visualLoopPhase_ = 0.0f;
    float liveTakeoverBlend_ = 0.0f;
    uint16_t glideOriginPhaseQ14_ = 0u;
    int8_t presetsCoarseAccum_ = 0;
    int8_t glideArcDir_ = 0;
    uint8_t activeSlot_ = 0u;
    uint8_t momentaryHomeSlot_ = 0u;
    bool dirty_ = false;
    bool latchGliding_ = false;
    bool latchHybridHeld_ = false;
    bool momentaryActive_ = false;
    bool knobMorphActive_ = false;
};

} // namespace ruido
