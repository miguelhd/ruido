#include "core/DspMath.h"
#include "ui/ScopeTraceEngine.h"
#include "ui/ScopeTraceMath.h"

#include <cstring>

namespace ruido {

void ScopeTraceEngine::rebuildTilesAndBloom() noexcept {
    bloomGrid_.fill(0u);
    constexpr float kFixedToPx = 1.0f / 64.0f;

    for (size_t s = 0u; s < kNumStrands; ++s) {
        if (strandWeightQ8_[s] == 0u) {
            strandTileMinY_[s].fill(255u);
            strandTileMaxY_[s].fill(0u);
            continue;
        }
        const float wgt = static_cast<float>(strandWeightQ8_[s]) * (1.0f / 256.0f);
        const auto& yFixed = strandYFixed_[s];

        for (size_t tile = 0u; tile < kNumTiles; ++tile) {
            int tileMin = static_cast<int>(kDisplayHeight);
            int tileMax = -1;
            const size_t colBase = tile * kTileCols;

            for (size_t c = 0u; c < kTileCols; ++c) {
                const size_t col = colBase + c;
                const float yA = static_cast<float>(yFixed[col]) * kFixedToPx;
                const float yB = static_cast<float>(yFixed[col + 1u]) * kFixedToPx;
                const float yLo = (yA < yB) ? yA : yB;
                const float yHi = (yA > yB) ? yA : yB;

                const int rawTop = static_cast<int>(yLo + 512.0f) - 512 - 3;
                const int rawBot = static_cast<int>(yHi + 512.0f) - 512 + 4;
                if (rawBot >= 0 && rawTop < static_cast<int>(kDisplayHeight)) {
                    const int segTop = (rawTop < 0) ? 0 : rawTop;
                    const int segBot = (rawBot >= static_cast<int>(kDisplayHeight))
                                           ? (static_cast<int>(kDisplayHeight) - 1)
                                           : rawBot;
                    if (segTop < tileMin) {
                        tileMin = segTop;
                    }
                    if (segBot > tileMax) {
                        tileMax = segBot;
                    }
                }

                const int byStart = static_cast<int>((yLo + 512.0f) * 0.25f) - 128;
                const int byEnd = static_cast<int>((yHi + 512.0f) * 0.25f) - 128;
                if (byEnd >= 0 && byStart < static_cast<int>(kBloomHeight)) {
                    const float dy = yB - yA;
                    const float len2 = 1.0f + dy * dy;
                    const float segLen = clampf(len2 * fastInvSqrt(len2), 1.0f, 4.2f);
                    const float dwell = (0.58f + 0.42f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * wgt;
                    const uint32_t energyQ4 = static_cast<uint32_t>(1174.4f * dwell * segLen);

                    const size_t bx = col >> 2u;
                    const uint32_t cellCount = static_cast<uint32_t>(byEnd - byStart + 1);
                    const uint32_t addPerCell = energyQ4 / cellCount;
                    const size_t by0 = (byStart < 0) ? 0u : static_cast<size_t>(byStart);
                    const size_t by1 = (byEnd >= static_cast<int>(kBloomHeight))
                                           ? (kBloomHeight - 1u)
                                           : static_cast<size_t>(byEnd);
                    for (size_t by = by0; by <= by1; ++by) {
                        uint16_t& cell = bloomGrid_[by * kBloomWidth + bx];
                        const uint32_t next = static_cast<uint32_t>(cell) + addPerCell;
                        cell = static_cast<uint16_t>((next > 65535u) ? 65535u : next);
                    }
                }
            }

            if (tileMin <= tileMax) {
                strandTileMinY_[s][tile] = static_cast<uint8_t>(tileMin);
                strandTileMaxY_[s][tile] = static_cast<uint8_t>(tileMax);
            } else {
                strandTileMinY_[s][tile] = 255u;
                strandTileMaxY_[s][tile] = 0u;
            }
        }
    }

    blurSeparableBloomGrid<kBloomWidth, kBloomHeight>(bloomGrid_);
}

void ScopeTraceEngine::renderBackgroundChunk(int y0, uint16_t chunkH, uint32_t bgKeepQ8,
                                             uint16_t* linBuf, uint8_t* lumBuf) const noexcept {
    const int y1 = y0 + static_cast<int>(chunkH);
    std::memset(linBuf, 0, static_cast<size_t>(chunkH) * kDisplayWidth * sizeof(uint16_t));

    if (scopeValid_) {
        constexpr float kFixedToPx = 1.0f / 64.0f;
        for (size_t s = 0u; s < kNumStrands; ++s) {
            const uint32_t wgtQ8 = strandWeightQ8_[s];
            if (wgtQ8 == 0u) {
                continue;
            }
            const float wgt = static_cast<float>(wgtQ8) * (1.0f / 256.0f);
            const auto& yFixed = strandYFixed_[s];
            const auto& tMin = strandTileMinY_[s];
            const auto& tMax = strandTileMaxY_[s];

            for (size_t tile = 0u; tile < kNumTiles; ++tile) {
                if (static_cast<int>(tMax[tile]) < y0 || static_cast<int>(tMin[tile]) >= y1) {
                    continue;
                }
                const size_t colBase = tile * kTileCols;
                for (size_t c = 0u; c < kTileCols; ++c) {
                    const size_t colX = colBase + c;
                    const float yA = static_cast<float>(yFixed[colX]) * kFixedToPx;
                    const float yB = static_cast<float>(yFixed[colX + 1u]) * kFixedToPx;
                    const int segTop = static_cast<int>(((yA < yB) ? yA : yB) + 512.0f) - 515;
                    const int segBot = static_cast<int>(((yA > yB) ? yA : yB) + 512.0f) - 508;
                    if (segBot <= y0 || segTop >= y1) {
                        continue;
                    }

                    const float yMid = 0.5f * (yA + yB);
                    const float dy = yB - yA;
                    const float invL2 = 1.0f / (1.0f + dy * dy);
                    const float dwell = (0.58f + 0.42f * fastInvSqrt(1.0f + 0.14f * dy * dy)) * wgt;
                    const int pyS = (segTop < y0) ? y0 : segTop;
                    const int pyE = (segBot > y1) ? y1 : segBot;

                    for (int py = pyS; py < pyE; ++py) {
                        const float vy = static_cast<float>(py) - yMid;
                        const float segT = clampf(vy * dy * invL2, -0.5f, 0.5f);
                        const float ey = vy - dy * segT;
                        const float d2 = segT * segT + ey * ey;
                        if (d2 < 3.95f) {
                            const size_t lutIdx = static_cast<size_t>(d2 * 16.0f);
                            const uint32_t add = static_cast<uint32_t>(
                                static_cast<float>(kGaussLut[lutIdx]) * dwell);
                            uint16_t& cell =
                                linBuf[static_cast<size_t>(py - y0) * kDisplayWidth + colX];
                            const uint32_t next = static_cast<uint32_t>(cell) + add;
                            cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                        }
                    }
                }
            }
        }
    }

    for (uint16_t rowIdx = 0u; rowIdx < chunkH; ++rowIdx) {
        const int py = y0 + static_cast<int>(rowIdx);
        const uint16_t* linRow = linBuf + static_cast<size_t>(rowIdx) * kDisplayWidth;
        uint8_t* lumRow = lumBuf + static_cast<size_t>(rowIdx) * kDisplayWidth;
        tonemapBloomScanline<kDisplayWidth, kBloomWidth, kBloomHeight>(
            py, bgKeepQ8, bloomGrid_.data(), linRow, lumRow);
    }
}

} // namespace ruido
