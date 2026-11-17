#pragma once

#include "core/ControlSurface.h"
#include "core/Parameters.h"
#include "ui/MonoCanvas.h"

#include <array>
#include <cstdint>

namespace ruido {

struct PreparedHudHeader {
    char leftMain[24]{};
    char leftSuffix[16]{};
    char rightText[32]{};
    char fineRateVal[24]{};
    char coarseRateVal[24]{};
    std::array<float, kNumSnapshots> sceneDotWeights{0.0f, 0.0f, 0.0f, 0.0f};
    float sceneDialPhase = 0.0f;
    int16_t mainX = 10;
    int16_t suffixX = 10;
    int16_t leftEndX = 73;
    int16_t rightTxtW = 0;
    int16_t minBarSlotW = 0;
    int16_t fineValW = 0;
    int16_t coarseValW = 0;
    int16_t fineBadgeW = 0;
    int16_t coarseBadgeW = 0;
    int8_t fineBadgeTrk = 1;
    int8_t coarseBadgeTrk = 1;
    bool showHeaderBar = false;
    bool showSceneDots = false;
    bool isRuidoAbout = false;
    bool isStepBadgeText = false;
};

void compositeIdlePeakMeters(const MonoStripCanvas& idleCv, float smoothedPeakL,
                             float smoothedPeakR) noexcept;

void prepareHudHeader(PreparedHudHeader& out, const EngineControlState& control,
                      const HudState& hud, const LfoSignalArray& lfoOutput, uint8_t stutterGateMask,
                      float finePhase, float coarsePhase) noexcept;

void compositeTopHeaderStrip(const MonoStripCanvas& cv, const PreparedHudHeader& prep,
                             const HudState& hud, float headerBarSmooth, float finePhase,
                             float coarsePhase) noexcept;

} // namespace ruido
