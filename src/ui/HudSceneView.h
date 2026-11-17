#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

inline constexpr std::array<float, kNumSnapshots> kSceneNodeX = {48.0f, 96.0f, 144.0f, 192.0f};

// Container-free scanline-composited Spatial Scene UX overlays (`0 B` framebuffer RAM):
//  1. `SAVE` 2-step screen (`SCENE 1/2` `STORE SCENE >` <-> `SCENE 2/2` `< DELETE SCENE`):
//     SEL-matched mode carousel, open 4-slot status columns (`ACTIVE` / `STORED` / `EMPTY`),
//     shared `topY = 92` 27-key FM-1 diagram + `"SCENE"` bracket (pure blinking on `Keys 9-12`),
//     and calm 2-line action guidance.
//  2. Clean-Scope Momentary `PRESETS` 4-Key `SCENE` Morph Screen: renders only the 4 Scene key
//     capsules (`1-4`) with uniform full-height LED diffuser slits (zero pressure bars, zero
//     other keyboard keys) plus 2-line `SHIFT` location guidance below.
class HudSceneView {
  public:
    void reset() noexcept;
    void clearImmediate() noexcept;

    bool isSaveActive(const HudState& hud) const noexcept {
        return hud.opacity > 0.004f && !hud.shiftHeld &&
               (hud.saveArmed || (hud.mode == HudMode::SaveArmed && hud.opacity >= 0.99f));
    }

    bool isPresetSceneActive(const HudState& hud) const noexcept {
        return hud.opacity > 0.004f && !hud.shiftHeld && !hud.saveArmed &&
               !hasPinnedPageOpen(hud) && !hud.snapshotSavedToast && !hud.snapshotDeletedToast &&
               hud.mode == HudMode::SnapshotSlot;
    }

    bool hasResidualPhase() const noexcept { return savePhase_ > 0.002f || morphPhase_ > 0.002f; }

    float savePhase() const noexcept { return savePhase_; }
    float morphPhase() const noexcept { return morphPhase_; }

    void update(float dtClamped, const HudState& effectiveHud) noexcept;

    void compositeSaveStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                            const HudState& frameHud) const noexcept;

    void compositePresetSceneStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                   const HudState& frameHud) const noexcept;

  private:
    std::array<float, kNumSnapshots> sceneKeyGlow_{};
    float savePhase_ = 0.0f;
    float morphPhase_ = 0.0f;
    uint8_t latchedOccupiedMask_ = 0x0Fu;
    uint8_t latchedSlot_ = 0u;
    bool latchedDeleteMode_ = false;
    bool primed_ = false;
};

} // namespace ruido
