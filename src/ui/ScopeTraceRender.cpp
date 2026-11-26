#include "core/DspMath.h"
#include "ui/ScopeTraceEngine.h"
#include "ui/ScopeTraceMath.h"

#include <cstring>

namespace ruido {

namespace {

constexpr std::array<uint16_t, 65u> buildRecipCellQ16() noexcept {
    std::array<uint16_t, 65u> tbl{};
    tbl[0] = 65535u;
    tbl[1] = 65535u;
    for (size_t i = 2u; i < tbl.size(); ++i) {
        tbl[i] = static_cast<uint16_t>((65536u + (i >> 1u)) / i);
    }
    return tbl;
}

constexpr auto kRecipCellQ16 = buildRecipCellQ16();

RUIDO_NOINLINE void rasterScopeCapSpan(uint16_t* linBuf, int y0, size_t colX, int rStart, int rEnd,
                                       int32_t sumAB_Q6, int32_t dyQ6, int32_t dyInvL2Q15,
                                       uint32_t dwellQ8) noexcept {
    int32_t vyQ7 = (rStart << 7) - sumAB_Q6;
    uint16_t* cellPtr = linBuf + static_cast<size_t>(rStart - y0) * kDisplayWidth + colX;
    for (int py = rStart; py < rEnd; ++py, vyQ7 += 128, cellPtr += kDisplayWidth) {
        int32_t segTQ15 = (vyQ7 * dyInvL2Q15) >> 7;
        if (segTQ15 < -16384) {
            segTQ15 = -16384;
        } else if (segTQ15 > 16384) {
            segTQ15 = 16384;
        }
        const int32_t eyQ7 = vyQ7 - ((dyQ6 * segTQ15 + 8192) >> 14);
        if (eyQ7 <= -256 || eyQ7 >= 256) {
            continue;
        }
        const int32_t segTQ7 = (segTQ15 + 128) >> 8;
        const int32_t d2Q14 = segTQ7 * segTQ7 + eyQ7 * eyQ7;
        if (d2Q14 < 64717) {
            const auto lutIdx = static_cast<size_t>(d2Q14 >> 10);
            const uint32_t add = (static_cast<uint32_t>(kGaussLut[lutIdx]) * dwellQ8) >> 8u;
            const uint32_t next = static_cast<uint32_t>(*cellPtr) + add;
            *cellPtr = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
        }
    }
}

} // namespace

void ScopeTraceEngine::rebuildTilesAndBloom(void (*coProcYield)() noexcept) noexcept {
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
        auto& colCache = strandColCache_[s];

        for (size_t tile = 0u; tile < kNumTiles; ++tile) {
            if (coProcYield != nullptr && (tile & 3u) == 3u) {
                coProcYield();
            }
            int tileMin = static_cast<int>(kDisplayHeight);
            int tileMax = -1;
            const size_t colBase = tile * kTileCols;

            for (size_t c = 0u; c < kTileCols; ++c) {
                const size_t col = colBase + c;
                const float yA = static_cast<float>(yFixed[col]) * kFixedToPx;
                const float yB = static_cast<float>(yFixed[col + 1u]) * kFixedToPx;
                const float yLo = (yA < yB) ? yA : yB;
                const float yHi = (yA > yB) ? yA : yB;
                const float dy = yB - yA;
                const float dy2 = dy * dy;
                const float len2 = 1.0f + dy2;
                float yInv = 1.0f;
                {
                    const float safeD = 1.0f + 0.14f * dy2;
                    uint32_t bits = 0u;
                    std::memcpy(&bits, &safeD, sizeof(bits));
                    bits = 0x5F375A86u - (bits >> 1u);
                    std::memcpy(&yInv, &bits, sizeof(yInv));
                    yInv = yInv * (1.5f - (0.5f * safeD) * yInv * yInv);
                }
                const float dwell = (0.58f + 0.42f * yInv) * wgt;

                const int rawTop = static_cast<int>(yLo + 512.0f) - 515;
                const int rawBot = static_cast<int>(yHi + 512.0f) - 508;
                StrandColCache& cc = colCache[col];
                if (rawBot > 0 && rawTop < static_cast<int>(kDisplayHeight)) {
                    const int segTop = (rawTop < 0) ? 0 : rawTop;
                    const int segBotExcl = (rawBot > static_cast<int>(kDisplayHeight))
                                               ? static_cast<int>(kDisplayHeight)
                                               : rawBot;
                    const int segBotIncl = segBotExcl - 1;
                    if (segTop < tileMin) {
                        tileMin = segTop;
                    }
                    if (segBotIncl > tileMax) {
                        tileMax = segBotIncl;
                    }
                    const float dyInvL2 = clampf(dy * fastRecipPos(len2), -0.5f, 0.5f);
                    const float scaledSlope = dyInvL2 * 32768.0f;
                    cc.segTop = static_cast<uint8_t>(segTop);
                    cc.segBot = static_cast<uint8_t>(segBotExcl);
                    cc.dwellQ8 = static_cast<uint16_t>(dwell * 256.0f + 0.5f);
                    cc.dyInvL2Q15 =
                        static_cast<int16_t>(scaledSlope + ((scaledSlope >= 0.0f) ? 0.5f : -0.5f));
                } else {
                    cc.segTop = 255u;
                    cc.segBot = 0u;
                    cc.dwellQ8 = 0u;
                    cc.dyInvL2Q15 = 0;
                }

                const int byStart = static_cast<int>((yLo + 512.0f) * 0.25f) - 128;
                const int byEnd = static_cast<int>((yHi + 512.0f) * 0.25f) - 128;
                if (byEnd >= 0 && byStart < static_cast<int>(kBloomHeight)) {
                    const float segLen =
                        (len2 >= 17.64f) ? 4.2f : clampf(len2 * fastInvSqrt(len2), 1.0f, 4.2f);
                    const uint32_t energyQ4 = static_cast<uint32_t>(1174.4f * dwell * segLen);

                    const size_t bx = col >> 2u;
                    const uint32_t cellCount = static_cast<uint32_t>(byEnd - byStart + 1);
                    uint32_t addPerCell = energyQ4;
                    if (cellCount == 2u) {
                        addPerCell = energyQ4 >> 1u;
                    } else if (cellCount == 4u) {
                        addPerCell = energyQ4 >> 2u;
                    } else if (cellCount > 1u) {
                        const uint32_t recipQ16 = (cellCount < kRecipCellQ16.size())
                                                      ? kRecipCellQ16[cellCount]
                                                      : (65536u / cellCount);
                        addPerCell = (energyQ4 * recipQ16) >> 16u;
                    }
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
        if (coProcYield != nullptr) {
            coProcYield();
        }
    }

    blurSeparableBloomGrid<kBloomWidth, kBloomHeight>(bloomGrid_, coProcYield);
}

void ScopeTraceEngine::renderBackgroundChunk(int y0, uint16_t chunkH, uint32_t bgKeepQ8,
                                             uint16_t* linBuf, uint8_t* lumBuf,
                                             void (*coProcYield)() noexcept) const noexcept {
    if (bgKeepQ8 == 0u) {
        std::memset(lumBuf, 0, static_cast<size_t>(chunkH) * kDisplayWidth);
        return;
    }
    const int y1 = y0 + static_cast<int>(chunkH);
    std::memset(linBuf, 0, static_cast<size_t>(chunkH) * kDisplayWidth * sizeof(uint16_t));

    if (scopeValid_) {
        uint32_t rasterCols = 0u;
        for (size_t s = 0u; s < kNumStrands; ++s) {
            if (strandWeightQ8_[s] == 0u) {
                continue;
            }
            if (coProcYield != nullptr) {
                coProcYield();
            }
            const auto& yFixed = strandYFixed_[s];
            const auto& tMin = strandTileMinY_[s];
            const auto& tMax = strandTileMaxY_[s];
            const auto& colCache = strandColCache_[s];

            for (size_t tile = 0u; tile < kNumTiles; ++tile) {
                if (static_cast<int>(tMax[tile]) < y0 || static_cast<int>(tMin[tile]) >= y1) {
                    continue;
                }
                const size_t colBase = tile * kTileCols;
                for (size_t c = 0u; c < kTileCols; ++c) {
                    const size_t colX = colBase + c;
                    const StrandColCache& cc = colCache[colX];
                    const int segTop = static_cast<int>(cc.segTop);
                    const int segBot = static_cast<int>(cc.segBot);
                    if (segBot <= y0 || segTop >= y1) {
                        continue;
                    }
                    if (coProcYield != nullptr && ((++rasterCols & 31u) == 0u)) {
                        coProcYield();
                    }

                    const int pyS = (segTop < y0) ? y0 : segTop;
                    const int pyE = (segBot > y1) ? y1 : segBot;
                    const int32_t yA_Q6 = static_cast<int32_t>(yFixed[colX]);
                    const int32_t yB_Q6 = static_cast<int32_t>(yFixed[colX + 1u]);
                    const int32_t dyQ6 = yB_Q6 - yA_Q6;
                    const int32_t dyInvL2Q15 = static_cast<int32_t>(cc.dyInvL2Q15);
                    const uint32_t dwellQ8 = static_cast<uint32_t>(cc.dwellQ8);
                    const int32_t sumAB_Q6 = yA_Q6 + yB_Q6;

                    const int32_t yLoQ6 = (yA_Q6 < yB_Q6) ? yA_Q6 : yB_Q6;
                    const int32_t yHiQ6 = (yA_Q6 > yB_Q6) ? yA_Q6 : yB_Q6;
                    if ((yHiQ6 - yLoQ6) >= 384) {
                        const int innerTop = ((yLoQ6 + 32768 + 63) >> 6) - 511;
                        const int innerBot = ((yHiQ6 + 32768) >> 6) - 512;
                        const int cS = (innerTop > pyS) ? innerTop : pyS;
                        const int cE = (innerBot < pyE) ? innerBot : pyE;
                        if (cS < cE) {
                            if (pyS < cS) {
                                rasterScopeCapSpan(linBuf, y0, colX, pyS, cS, sumAB_Q6, dyQ6,
                                                   dyInvL2Q15, dwellQ8);
                            }
                            int32_t segTQ15 = (((cS << 7) - sumAB_Q6) * dyInvL2Q15) >> 7;
                            uint16_t* cellPtr =
                                linBuf + static_cast<size_t>(cS - y0) * kDisplayWidth + colX;
                            for (int py = cS; py < cE;
                                 ++py, segTQ15 += dyInvL2Q15, cellPtr += kDisplayWidth) {
                                const int32_t segTQ7 = (segTQ15 + 128) >> 8;
                                const auto lutIdx =
                                    static_cast<size_t>((segTQ7 * segTQ7) >> 10) & 63u;
                                const uint32_t add =
                                    (static_cast<uint32_t>(kGaussLut[lutIdx]) * dwellQ8) >> 8u;
                                const uint32_t next = static_cast<uint32_t>(*cellPtr) + add;
                                *cellPtr = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                            }
                            if (cE < pyE) {
                                rasterScopeCapSpan(linBuf, y0, colX, cE, pyE, sumAB_Q6, dyQ6,
                                                   dyInvL2Q15, dwellQ8);
                            }
                            continue;
                        }
                    }
                    rasterScopeCapSpan(linBuf, y0, colX, pyS, pyE, sumAB_Q6, dyQ6, dyInvL2Q15,
                                       dwellQ8);
                }
            }
        }
    }

    if (coProcYield != nullptr) {
        coProcYield();
    }
    for (uint16_t rowIdx = 0u; rowIdx < chunkH; ++rowIdx) {
        const int py = y0 + static_cast<int>(rowIdx);
        const uint16_t* linRow = linBuf + static_cast<size_t>(rowIdx) * kDisplayWidth;
        uint8_t* lumRow = lumBuf + static_cast<size_t>(rowIdx) * kDisplayWidth;
        tonemapBloomScanline<kDisplayWidth, kBloomWidth, kBloomHeight>(
            py, bgKeepQ8, bloomGrid_.data(), linRow, lumRow);
        if (coProcYield != nullptr && (rowIdx & 3u) == 3u) {
            coProcYield();
        }
    }
}

} // namespace ruido
