#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/MonoCanvas.h"

#include <cstdint>

namespace ruido {

void compositeIdlePeakMeters(const MonoStripCanvas& idleCv, float smoothedPeakL,
                             float smoothedPeakR) noexcept;

void compositeTopHeaderStrip(const MonoStripCanvas& cv, const EngineControlState& control,
                             const HudState& hud, const LfoSignalArray& lfoOutput,
                             uint8_t stutterGateMask, float headerBarSmooth, float finePhase,
                             float coarsePhase) noexcept;

} // namespace ruido
