#include "ui/ScopeTraceEngine.h"

#include "core/DspMath.h"
#include "ui/ScopeTraceMath.h"

#include <cstring>

namespace ruido {

void ScopeTraceEngine::reset() noexcept {
    history_.fill({});
    historyWrite_ = 0u;
    historyCount_ = 0u;
    bassZoom2x_ = true;
    scopeValid_ = false;
    trigSigValid_ = false;
    phosphorPrimed_ = false;
    smoothStride_ = 2.0f;
    timeSinceScopePushSec_ = 0.0f;
    thermalPhase_ = 0.0f;
    busPresenceSmooth_ = 0.0f;
    soloFactorSmooth_ = 0.0f;
    monoCollapseSmooth_ = 0.0f;
    voicePresenceSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prevTrigSig_.fill(0.0f);

    for (size_t s = 0u; s < kNumStrands; ++s) {
        strandSmoothY_[s].fill(120.0f);
        strandYFixed_[s].fill(static_cast<int16_t>(120 * 64));
        strandWeightQ8_[s] = 0u;
        strandTileMinY_[s].fill(255u);
        strandTileMaxY_[s].fill(0u);
    }
    for (auto& vt : voiceTrailQ12_) {
        vt.fill(0);
    }
    busTrailLQ12_.fill(0);
    busTrailRQ12_.fill(0);

    strandWeightQ8_[12] = 112u;
    rebuildTilesAndBloom();
    scopeValid_ = true;
}

void ScopeTraceEngine::ingestScopeRing(ScopeRingBuffer& ring, bool scopeFrozen) noexcept {
    std::array<ScopeFrame, 256> scratch{};
    while (ring.available() > 0u) {
        const size_t n = ring.popBatch(scratch.data(), scratch.size());
        if (n == 0u) {
            break;
        }
        pushScopeSamples(scratch.data(), n, scopeFrozen);
    }
}

void ScopeTraceEngine::pushScopeSamples(const ScopeFrame* frames, size_t count,
                                        bool scopeFrozen) noexcept {
    if (frames == nullptr || count == 0u) {
        return;
    }
    timeSinceScopePushSec_ = 0.0f;
    for (size_t i = 0u; i < count; ++i) {
        if (!scopeFrozen) {
            CompactScopeSample packed{};
            packed.leftQ12 = packQ3_12(frames[i].left);
            packed.rightQ12 = packQ3_12(frames[i].right);
            const uint8_t mask = frames[i].voiceRisingMask;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const bool edge = ((mask >> v) & 1u) != 0u;
                packed.voiceEdgeQ12[v] = packVoiceEdgeQ12(frames[i].voiceQ13[v], edge);
            }
            history_[historyWrite_] = packed;
            historyWrite_ = (historyWrite_ + 1u) & kHistoryMask;
            if (historyCount_ < kHistorySize) {
                ++historyCount_;
            }
        }
    }
}

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

void ScopeTraceEngine::buildWaveformTrace(float dtSec, const EngineControlState& control,
                                          bool scopeFrozen) noexcept {
    if (scopeFrozen && scopeValid_) {
        return;
    }

    const float dtClamped = clampf(dtSec, 0.002f, 0.080f);

    size_t anchorVoice = 0u;
    bool hasAudioAnchor = false;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (!control.voices[v].muted && control.voices[v].frequencyHz >= kAudioFreqMinHz) {
            anchorVoice = v;
            hasAudioAnchor = true;
            break;
        }
    }

    if (hasAudioAnchor) {
        const float anchorHz = control.voices[anchorVoice].frequencyHz;
        if (bassZoom2x_ && anchorHz > 92.0f) {
            bassZoom2x_ = false;
        } else if (!bassZoom2x_ && anchorHz < 78.0f) {
            bassZoom2x_ = true;
        }
    }

    const float targetStride = bassZoom2x_ ? 2.0f : 1.0f;
    if (!phosphorPrimed_) {
        smoothStride_ = targetStride;
    } else {
        smoothStride_ = slewScalar(smoothStride_, targetStride,
                                   1.0f - fastExpNeg(dtClamped * (1.0f / 0.160f)), 0.001f);
    }

    constexpr size_t kTrigPt = 120u;
    constexpr size_t kLeftSpan = kTrigPt * 2u + 8u;
    constexpr size_t kRightSpan = (kTracePoints - kTrigPt) * 2u + 44u;

    if (historyCount_ < kLeftSpan + kRightSpan + 8u) {
        phosphorPrimed_ = true;
        scopeValid_ = true;
        return;
    }

    const bool streamStalled = (timeSinceScopePushSec_ > 0.065f);

    std::array<float, kTracePoints> liveL{};
    std::array<float, kTracePoints> liveR{};
    std::array<std::array<float, kTracePoints>, kNumOscillators> voiceCur{};
    std::array<float, kNumOscillators> voicePeak{0.0f, 0.0f, 0.0f, 0.0f};
    float busPeak = 0.0f;
    size_t chosenTrigInt = (historyWrite_ - kRightSpan) & kHistoryMask;
    float totalShift = 0.0f;

    if (!streamStalled) {
        const size_t latestEdgeLimit = (historyWrite_ - kRightSpan) & kHistoryMask;
        const size_t maxBack = historyCount_ - kLeftSpan - kRightSpan;
        const size_t searchLen = (maxBack > 3480u) ? 3480u : maxBack;

        struct TrigCandidate {
            size_t trigInt = 0u;
            float trigFrac = 0.0f;
        };
        constexpr size_t kMaxCandidates = 8u;
        std::array<TrigCandidate, kMaxCandidates> candidates{};
        size_t numCand = 0u;
        size_t minNextBack = 0u;

        if (hasAudioAnchor && searchLen > 0u) {
            for (size_t back = 0u; back < searchLen && numCand < kMaxCandidates; ++back) {
                if (back < minNextBack) {
                    continue;
                }
                const size_t edgePos = (latestEdgeLimit - back) & kHistoryMask;
                if (!unpackVoiceEdgeFlag(history_[edgePos].voiceEdgeQ12[anchorVoice])) {
                    continue;
                }

                size_t candInt = edgePos;
                float candFrac = 0.0f;
                for (int off = -3; off <= 32; ++off) {
                    const size_t p1 =
                        (edgePos + static_cast<size_t>(off + static_cast<int>(kHistorySize))) &
                        kHistoryMask;
                    const size_t p0 = (p1 - 1u) & kHistoryMask;
                    const float v0 = unpackVoiceEdgeSample(history_[p0].voiceEdgeQ12[anchorVoice]);
                    const float v1 = unpackVoiceEdgeSample(history_[p1].voiceEdgeQ12[anchorVoice]);
                    const float slope = v1 - v0;
                    if (v0 <= 0.0f && v1 > 0.0f && slope > 0.002f) {
                        candInt = p0;
                        candFrac = clampf(-v0 / slope, 0.0f, 1.0f);
                        break;
                    }
                }
                candidates[numCand++] = {candInt, candFrac};
                minNextBack = back + 16u;
            }
        }

        chosenTrigInt = latestEdgeLimit;
        float chosenTrigFrac = 0.0f;
        if (numCand == 1u || (numCand > 1u && !trigSigValid_)) {
            chosenTrigInt = candidates[0].trigInt;
            chosenTrigFrac = candidates[0].trigFrac;
        } else if (numCand > 1u) {
            float bestScore = 1.0e9f;
            size_t bestC = 0u;
            for (size_t c = 0u; c < numCand; ++c) {
                float err = 0.0f;
                const size_t cTrig = candidates[c].trigInt;
                const float cFrac = candidates[c].trigFrac;
                for (size_t k = 0u; k < kTrigSigTaps; ++k) {
                    const float relOff =
                        cFrac + static_cast<float>(static_cast<int>(10u + k * 20u) -
                                                   static_cast<int>(kTrigPt)) *
                                    smoothStride_;
                    const int relFloor = static_cast<int>(relOff + 512.0f) - 512;
                    const float relFrac = clampf(relOff - static_cast<float>(relFloor), 0.0f, 1.0f);
                    const size_t p0 =
                        (cTrig + static_cast<size_t>(relFloor + static_cast<int>(kHistorySize))) &
                        kHistoryMask;
                    const size_t p1 = (p0 + 1u) & kHistoryMask;
                    for (size_t v = 0u; v < kNumOscillators; ++v) {
                        const float v0 = unpackVoiceEdgeSample(history_[p0].voiceEdgeQ12[v]);
                        const float v1 = unpackVoiceEdgeSample(history_[p1].voiceEdgeQ12[v]);
                        const float sv = v0 + relFrac * (v1 - v0);
                        err += absf(sv - prevTrigSig_[k * kNumOscillators + v]);
                    }
                }
                const float score = (err * (1.0f / static_cast<float>(kTrigSigPoints))) +
                                    0.008f * static_cast<float>(c);
                if (score < bestScore) {
                    bestScore = score;
                    bestC = c;
                }
            }
            chosenTrigInt = candidates[bestC].trigInt;
            chosenTrigFrac = candidates[bestC].trigFrac;
        }

        thermalPhase_ += clampf(dtSec, 0.0f, 0.10f) * 0.065f;
        while (thermalPhase_ >= 1.0f) {
            thermalPhase_ -= 1.0f;
        }
        const float slowBreathSamples =
            0.52f *
            (fastSinPhase01(thermalPhase_) + 0.45f * fastSinPhase01(thermalPhase_ * 2.0f + 0.19f));
        totalShift = clampf(chosenTrigFrac + slowBreathSamples, -2.0f, 2.0f);

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float relOff =
                totalShift + (static_cast<float>(pt) - static_cast<float>(kTrigPt)) * smoothStride_;
            const int offFloor = static_cast<int>(relOff + 512.0f) - 512;
            const float frac = clampf(relOff - static_cast<float>(offFloor), 0.0f, 1.0f);
            const size_t base =
                (chosenTrigInt + static_cast<size_t>(offFloor + static_cast<int>(kHistorySize))) &
                kHistoryMask;
            const size_t i0 = (base - 1u) & kHistoryMask;
            const size_t i1 = base;
            const size_t i2 = (base + 1u) & kHistoryMask;
            const size_t i3 = (base + 2u) & kHistoryMask;

            const float sL =
                catmullRom(unpackQ3_12(history_[i0].leftQ12), unpackQ3_12(history_[i1].leftQ12),
                           unpackQ3_12(history_[i2].leftQ12), unpackQ3_12(history_[i3].leftQ12),
                           frac) *
                1.50f;
            const float sR =
                catmullRom(unpackQ3_12(history_[i0].rightQ12), unpackQ3_12(history_[i1].rightQ12),
                           unpackQ3_12(history_[i2].rightQ12), unpackQ3_12(history_[i3].rightQ12),
                           frac) *
                1.50f;
            liveL[pt] = sL;
            liveR[pt] = sR;
            const float pkBus = (absf(sL) > absf(sR)) ? absf(sL) : absf(sR);
            if (pkBus > busPeak) {
                busPeak = pkBus;
            }

            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float sv =
                    catmullRom(unpackVoiceEdgeSample(history_[i0].voiceEdgeQ12[v]),
                               unpackVoiceEdgeSample(history_[i1].voiceEdgeQ12[v]),
                               unpackVoiceEdgeSample(history_[i2].voiceEdgeQ12[v]),
                               unpackVoiceEdgeSample(history_[i3].voiceEdgeQ12[v]), frac);
                voiceCur[v][pt] = sv;
                const float av = absf(sv);
                if (av > voicePeak[v]) {
                    voicePeak[v] = av;
                }
            }
        }
    }

    std::array<float, kNumOscillators> rawVoicePres{};
    float rawTotalVoicePres = 0.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        rawVoicePres[v] = smoothstep01(voicePeak[v] * (1.0f / 0.028f));
        rawTotalVoicePres += rawVoicePres[v];
    }
    const float rawBusPres = smoothstep01(busPeak * (1.0f / 0.022f));
    const float rawVoiceMixPres = smoothstep01(rawTotalVoicePres * 0.5f);
    const float rawAnyPres = (rawBusPres > rawVoiceMixPres) ? rawBusPres : rawVoiceMixPres;

    if (!phosphorPrimed_) {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            voicePresenceSmooth_[v] = rawVoicePres[v];
        }
        busPresenceSmooth_ = rawAnyPres;
        soloFactorSmooth_ = clampf(2.0f - rawTotalVoicePres, 0.0f, 1.0f);
        monoCollapseSmooth_ = control.global.monoCollapse ? 1.0f : 0.0f;
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float tauV = (rawVoicePres[v] >= voicePresenceSmooth_[v]) ? 0.065f : 0.210f;
            voicePresenceSmooth_[v] =
                slewScalar(voicePresenceSmooth_[v], rawVoicePres[v],
                           1.0f - fastExpNeg(dtClamped * (1.0f / tauV)), 0.002f);
        }
        const float tauBus = (rawAnyPres >= busPresenceSmooth_) ? 0.065f : 0.220f;
        busPresenceSmooth_ = slewScalar(busPresenceSmooth_, rawAnyPres,
                                        1.0f - fastExpNeg(dtClamped * (1.0f / tauBus)), 0.002f);
        if (rawTotalVoicePres > 0.35f) {
            const float targetSolo = clampf(2.0f - rawTotalVoicePres, 0.0f, 1.0f);
            soloFactorSmooth_ = slewScalar(soloFactorSmooth_, targetSolo,
                                           1.0f - fastExpNeg(dtClamped * (1.0f / 0.110f)), 0.002f);
        }
        monoCollapseSmooth_ =
            slewScalar(monoCollapseSmooth_, control.global.monoCollapse ? 1.0f : 0.0f,
                       1.0f - fastExpNeg(dtClamped * (1.0f / 0.060f)), 0.002f);
    }

    if (rawAnyPres > 0.15f) {
        for (size_t k = 0u; k < kTrigSigTaps; ++k) {
            const size_t pt = 10u + k * 20u;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float sv = voiceCur[v][pt];
                float& ref = prevTrigSig_[k * kNumOscillators + v];
                ref = (!trigSigValid_) ? sv : (ref + 0.25f * (sv - ref));
            }
        }
        trigSigValid_ = true;
    } else if (busPresenceSmooth_ <= 0.005f) {
        trigSigValid_ = false;
    }

    const float stoppingBlend = clampf((busPresenceSmooth_ - rawAnyPres) * 1.5f, 0.0f, 1.0f);
    const float alphaMid =
        1.0f - fastExpNeg(dtClamped * (1.0f / (0.085f + 0.055f * stoppingBlend)));
    const float alphaSlow =
        1.0f - fastExpNeg(dtClamped * (1.0f / (0.135f + 0.055f * stoppingBlend)));

    std::array<float, kTracePoints> trailBusL{};
    std::array<float, kTracePoints> trailBusR{};
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        const float prevL = unpackQ3_12(busTrailLQ12_[pt]);
        const float prevR = unpackQ3_12(busTrailRQ12_[pt]);
        trailBusL[pt] = (!phosphorPrimed_) ? liveL[pt] : (prevL + alphaMid * (liveL[pt] - prevL));
        trailBusR[pt] = (!phosphorPrimed_) ? liveR[pt] : (prevR + alphaMid * (liveR[pt] - prevR));
        busTrailLQ12_[pt] = packQ3_12(trailBusL[pt]);
        busTrailRQ12_[pt] = packQ3_12(trailBusR[pt]);
    }

    const float soloFactor = soloFactorSmooth_;
    const float voiceVisGain = 1.12f + 0.24f * soloFactor;
    const float busSympathy = 0.22f * (1.0f - soloFactor);

    const float startRelOff = totalShift - static_cast<float>(kTrigPt) * smoothStride_;
    const int startOffFloor = static_cast<int>(startRelOff + 512.0f) - 512;
    const size_t startIdx =
        (chosenTrigInt + static_cast<size_t>(startOffFloor + static_cast<int>(kHistorySize))) &
        kHistoryMask;
    const size_t oldestHistIdx = (historyWrite_ - historyCount_) & kHistoryMask;
    const size_t preAvailSamples = streamStalled ? 0u : ((startIdx - oldestHistIdx) & kHistoryMask);
    size_t preWarmSteps =
        (preAvailSamples > 6u)
            ? static_cast<size_t>(static_cast<float>(preAvailSamples - 4u) / smoothStride_)
            : 0u;
    if (preWarmSteps > 40u) {
        preWarmSteps = 40u;
    }
    const float gCap = 0.017095f * smoothStride_;
    const float kCapG = gCap / (1.0f + gCap);

    std::array<float, kTracePoints> voiceCap{};
    std::array<float, kTracePoints> scratchA{};
    std::array<float, kTracePoints> scratchB{};

    auto applyStrandPhosphor = [this](size_t s, const float* targetY, float alpha,
                                      bool snapImmediate) noexcept {
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            float y = targetY[pt];
            if (!snapImmediate) {
                y = strandSmoothY_[s][pt] + alpha * (y - strandSmoothY_[s][pt]);
                if (absf(y - 120.0f) < 0.008f && absf(targetY[pt] - 120.0f) < 0.001f) {
                    y = 120.0f;
                }
            }
            strandSmoothY_[s][pt] = y;
            strandYFixed_[s][pt] = screenYToFixed(y);
        }
    };

    constexpr float kBaseDispersiveShifts[kNumOscillators] = {-5.0f, -2.0f, 2.0f, 5.0f};
    constexpr float kSoloDispersiveShifts[kNumOscillators] = {-5.0f, -5.0f, 5.0f, 5.0f};
    const bool snapColdBoot = !phosphorPrimed_;

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const size_t baseStrand = v * 3u;
        const float pres = voicePresenceSmooth_[v];
        if (pres <= 0.001f && rawVoicePres[v] <= 0.001f) {
            voiceTrailQ12_[v].fill(0);
            for (size_t k = 0u; k < 3u; ++k) {
                strandSmoothY_[baseStrand + k].fill(120.0f);
                strandYFixed_[baseStrand + k].fill(static_cast<int16_t>(120 * 64));
                strandWeightQ8_[baseStrand + k] = 0u;
                strandTileMinY_[baseStrand + k].fill(255u);
                strandTileMaxY_[baseStrand + k].fill(0u);
            }
            continue;
        }

        const float vStop = clampf((pres - rawVoicePres[v]) * 1.5f, 0.0f, 1.0f);
        const float vAlphaFast = 1.0f - fastExpNeg(dtClamped * (1.0f / (0.038f + 0.057f * vStop)));
        const float vAlphaMid = 1.0f - fastExpNeg(dtClamped * (1.0f / (0.085f + 0.055f * vStop)));
        const float vAlphaSlow = 1.0f - fastExpNeg(dtClamped * (1.0f / (0.135f + 0.055f * vStop)));

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float curScaled = voiceCur[v][pt] * voiceVisGain;
            voiceCur[v][pt] = curScaled;
            float tv = curScaled;
            if (!snapColdBoot) {
                const float prevTv = unpackQ3_12(voiceTrailQ12_[v][pt]);
                tv = prevTv + vAlphaMid * (curScaled - prevTv);
            }
            voiceTrailQ12_[v][pt] = packQ3_12(tv);
            scratchA[pt] = tv;
        }

        float capState = voiceCur[v][0] * 0.35f;
        if (preWarmSteps > 0u) {
            const int firstBack =
                static_cast<int>(static_cast<float>(preWarmSteps) * smoothStride_ + 0.5f);
            const size_t firstWarm =
                (startIdx + static_cast<size_t>(static_cast<int>(kHistorySize) - firstBack)) &
                kHistoryMask;
            capState =
                unpackVoiceEdgeSample(history_[firstWarm].voiceEdgeQ12[v]) * voiceVisGain * 0.35f;
            for (size_t step = preWarmSteps; step >= 1u; --step) {
                const int backSamples =
                    static_cast<int>(static_cast<float>(step) * smoothStride_ + 0.5f);
                const size_t wIdx =
                    (startIdx + static_cast<size_t>(static_cast<int>(kHistorySize) - backSamples)) &
                    kHistoryMask;
                const float xWarm =
                    unpackVoiceEdgeSample(history_[wIdx].voiceEdgeQ12[v]) * voiceVisGain;
                const float vCap = (xWarm - capState) * kCapG;
                const float yCap = vCap + capState;
                capState = yCap + vCap;
            }
        }
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float vCap = (voiceCur[v][pt] - capState) * kCapG;
            const float yCap = vCap + capState;
            capState = yCap + vCap;
            voiceCap[pt] = yCap * 1.32f;
        }

        const auto& busRef = ((v & 1u) != 0u) ? liveR : liveL;
        const auto& prevBusRef = ((v & 1u) != 0u) ? trailBusR : trailBusL;
        const float vSympathy = busSympathy * rawVoicePres[v];

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            const float coupledCap = (1.0f - vSympathy) * voiceCap[pt] + vSympathy * prevBusRef[pt];
            scratchB[pt] = sampleToScreenY(coupledCap * 0.76f);
        }
        applyStrandPhosphor(baseStrand + 0u, scratchB.data(), vAlphaSlow, snapColdBoot);
        strandWeightQ8_[baseStrand + 0u] =
            static_cast<uint16_t>((0.34f + 0.16f * soloFactor) * pres * 256.0f + 0.5f);

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchA[pt] =
                (1.0f - vSympathy * 1.15f) * scratchA[pt] + (vSympathy * 1.15f) * prevBusRef[pt];
        }
        const float dispShift = kBaseDispersiveShifts[v] +
                                (kSoloDispersiveShifts[v] - kBaseDispersiveShifts[v]) * soloFactor;
        dispersiveShell(scratchA.data(), scratchB.data(), kTracePoints, 0.30f, dispShift);
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchB[pt] = sampleToScreenY(scratchB[pt] * 0.92f);
        }
        applyStrandPhosphor(baseStrand + 1u, scratchB.data(), vAlphaMid, snapColdBoot);
        strandWeightQ8_[baseStrand + 1u] =
            static_cast<uint16_t>((0.37f + 0.19f * soloFactor) * pres * 256.0f + 0.5f);

        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchA[pt] =
                (1.0f - vSympathy * 0.85f) * voiceCur[v][pt] + (vSympathy * 0.85f) * busRef[pt];
        }
        smoothZeroPhase(scratchA.data(), scratchB.data(), kTracePoints, 0.36f, 2u);
        for (size_t pt = 0u; pt < kTracePoints; ++pt) {
            scratchB[pt] = sampleToScreenY(scratchB[pt] * 1.04f);
        }
        applyStrandPhosphor(baseStrand + 2u, scratchB.data(), vAlphaFast, snapColdBoot);
        strandWeightQ8_[baseStrand + 2u] =
            static_cast<uint16_t>((0.45f + 0.27f * soloFactor) * pres * 256.0f + 0.5f);
    }

    smoothZeroPhase(liveL.data(), scratchB.data(), kTracePoints, 0.24f, 2u);
    const float s12Gain = 0.56f - 0.12f * soloFactor;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchB[pt] = sampleToScreenY(scratchB[pt] * s12Gain);
    }
    applyStrandPhosphor(12u, scratchB.data(), alphaSlow, snapColdBoot);
    const float w12 = 0.4375f + (0.40f - 0.4375f) * busPresenceSmooth_;
    strandWeightQ8_[12] = static_cast<uint16_t>(w12 * 256.0f + 0.5f);

    const float soloRibBlend = smoothstep01((soloFactor - 0.45f) * (1.0f / 0.45f));
    const float trailRibMix =
        (monoCollapseSmooth_ > soloRibBlend) ? monoCollapseSmooth_ : soloRibBlend;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchA[pt] = liveR[pt] + trailRibMix * (trailBusL[pt] - liveR[pt]);
    }
    smoothZeroPhase(scratchA.data(), scratchB.data(), kTracePoints, 0.28f, 2u);
    const float s13Gain = 0.82f - 0.20f * soloFactor;
    for (size_t pt = 0u; pt < kTracePoints; ++pt) {
        scratchB[pt] = sampleToScreenY(scratchB[pt] * s13Gain);
    }
    applyStrandPhosphor(13u, scratchB.data(), alphaMid, snapColdBoot);
    strandWeightQ8_[13] = static_cast<uint16_t>(0.46f * busPresenceSmooth_ * 256.0f + 0.5f);
    phosphorPrimed_ = true;

    rebuildTilesAndBloom();
    scopeValid_ = true;
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
