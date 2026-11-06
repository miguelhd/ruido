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
EngineControlState makeFactorySnapshotPreset(size_t slot) noexcept;

// 4-slot Snapshot Bank + smooth log-octave latching glide & momentary pressure-morph engine.
class SnapshotBank {
  public:
    SnapshotBank() noexcept { reset(); }

    void reset() noexcept;

    uint8_t activeSlot() const noexcept { return activeSlot_; }
    bool isSlotOccupied(size_t idx) const noexcept {
        return isPatchSnapshotOccupied(slots_[idx & 3u]);
    }
    uint8_t occupiedMask() const noexcept;
    bool isDirty() const noexcept { return dirty_; }
    bool isTransitioning() const noexcept { return latchGliding_ || momentaryActive_; }
    float morphPosition() const noexcept { return morphPos_; }
    const std::array<float, kNumSnapshots>& slotWeights() const noexcept { return slotWeights_; }
    const PatchSnapshot& slotBinary(size_t idx) const noexcept { return slots_[idx & 3u]; }

    // Store current synthesis patch into slot 0..3 (`SAVE` + Key 13-16).
    void saveSlot(uint8_t slot, EngineControlState& state) noexcept;

    // Clear slot 0..3 into a CRC-valid `EMPTY` state (`SAVE` -> `DELETE SCENE` + Key 13-16);
    // returns true if the slot held a scene prior to deletion.
    bool deleteSlot(uint8_t slot, EngineControlState& state) noexcept;

    // Load a CRC-verified binary snapshot from FM-1 NVS flash (`dinghy` `slot_*`) into slot 0..3;
    // returns false and preserves the existing/factory slot if `snap` fails version or CRC checks.
    bool loadSlotBinary(uint8_t slot, const PatchSnapshot& snap,
                        EngineControlState& state) noexcept;

    // Immediately activate slot 0..3 without starting a glide (used on NVS boot restore).
    void selectActiveSlotImmediate(uint8_t slot, EngineControlState& state) noexcept;

    // Stepped recall via left PRESETS encoder (works across all 3 keyboard models).
    void recallSlotStepped(uint8_t slot, KnobRate rate, EngineControlState& state) noexcept;

    // Update LIVE MIXER Keys 13-16 gate/pressure state (Latching glide/blend or Momentary morph).
    bool onSnapKeysChanged(const std::array<bool, kNumSnapshots>& held,
                           const std::array<float, kNumSnapshots>& pressure, KeySwitchMode snapMode,
                           EngineControlState& state) noexcept;

    // Advance active latching glide or momentary pressure morph/release; returns true if `state`
    // changed and should be published to Core 1.
    bool step(float dtSec, const std::array<bool, kNumSnapshots>& held,
              const std::array<float, kNumSnapshots>& pressure, KeySwitchMode snapMode,
              EngineControlState& state) noexcept;

    // Cancel any active glide at current `state` when the user manually turns a synthesis knob.
    void notifyManualEdit(EngineControlState& state) noexcept;

    // Cleanly restore underlying home patch if Momentary morph was active during mode/model change.
    bool cancelMomentaryIfActive(EngineControlState& state) noexcept;

    void syncHud(HudState& hud) const noexcept;

  private:
    PatchFrame blendSlots(const std::array<float, kNumSnapshots>& normWeights) const noexcept;

    std::array<PatchSnapshot, kNumSnapshots> slots_{};
    PatchFrame originFrame_{};
    std::array<float, kNumSnapshots> slotWeights_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumSnapshots> originWeights_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumSnapshots> targetWeights_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<bool, kNumSnapshots> prevHeld_{false, false, false, false};
    float glideProgress_ = 1.0f;
    float glideDurationSec_ = 0.50f;
    float momentaryBlend_ = 0.0f;
    float morphPos_ = 1.0f;
    uint8_t activeSlot_ = 0u;
    bool dirty_ = false;
    bool latchGliding_ = false;
    bool latchHybridHeld_ = false;
    bool momentaryActive_ = false;
};

} // namespace ruido
