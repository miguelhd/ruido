#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"
#include "ui/FontSansAA.h"
#include "ui/GraphStyle.h"
#include "ui/MonoCanvas.h"
#include "ui/ScopeTraceEngine.h"
#include "ui/ScopeTraceMath.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ruido {

// Concept D layout constants: Top PWR Rail + 4 Live Oscillator Charge-and-Fire Ramp Lanes
inline constexpr int kCplTrackX0 = 48;
inline constexpr int kCplTrackX1 = 228;
inline constexpr size_t kCplCols = static_cast<size_t>(kCplTrackX1 - kCplTrackX0); // 180 intervals
inline constexpr size_t kCplPts = kCplCols + 1u;                                   // 181 endpoints

inline constexpr float kCplPwrTopY = 41.0f;
inline constexpr float kCplPwrBotY = 66.0f;
inline constexpr float kCplLaneTop0 = 78.0f;
inline constexpr float kCplLaneStride = 27.0f;
inline constexpr float kCplLaneHeight = 18.0f;

inline uint8_t packCouplingWrap(float uWrap01, float peak01) noexcept {
    const int uNib = static_cast<int>(clampf(uWrap01 * 14.0f + 1.5f, 1.0f, 15.0f));
    const int pNib =
        static_cast<int>(clampf((peak01 - 0.15f) * (15.0f / 0.85f) + 0.5f, 0.0f, 15.0f));
    return static_cast<uint8_t>((uNib << 4) | pNib);
}

inline void unpackCouplingWrap(uint8_t packed, float& outUWrap01, float& outPeak01) noexcept {
    const int uNib = (packed >> 4) & 0x0F;
    const int pNib = packed & 0x0F;
    outUWrap01 = static_cast<float>(uNib - 1) * (1.0f / 14.0f);
    outPeak01 = 0.15f + static_cast<float>(pNib) * (0.85f / 15.0f);
}

// Reconstruct Concept D directly from Core 1's live 44.1 kHz ScopeTraceEngine history
// (sample-accurate Schmitt-trigger rising pulses + post-filter/VCA voice audio envelopes).
inline void buildCouplingFromLiveHistory(
    float dtClamped, bool snapImmediate, float coupling, float chaos,
    const std::array<float, kNumOscillators>& liveHz,
    const std::array<bool, kNumOscillators>& voiceMuted, const ScopeTraceEngine& traceEngine,
    float& ioWinSamplesSmooth, std::array<float, kNumOscillators>& ioFullSpanSmooth,
    std::array<float, kNumOscillators>& ioCapAct, std::array<uint8_t, kCplPts>& outSagQ8,
    std::array<std::array<uint8_t, kCplPts>, kNumOscillators>& outRampQ8,
    std::array<std::array<uint8_t, kCplCols>, kNumOscillators>& outWrapPacked,
    std::array<uint8_t, kCplCols>& outSyncMask, CoProcYieldFn coProcYield = nullptr) noexcept {
    const size_t histCount = traceEngine.historySampleCount();
    const size_t histWrite = traceEngine.historyWritePos();
    const auto& hist = traceEngine.rawHistory();
    constexpr size_t kMask = ScopeTraceEngine::kHistoryMask;

    // 1. Choose trigger reference voice (OSC 1 when active, else lowest active unmuted voice)
    size_t trigVoice = 0u;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (!voiceMuted[v] && liveHz[v] >= 16.0f) {
            trigVoice = v;
            break;
        }
    }

    const float trigHz = clampf(liveHz[trigVoice], 18.0f, 1200.0f);
    const float trigNomPeriod = clampf(kBaseSampleRate / trigHz, 36.0f, 920.0f);
    const float tgtWinSamples = clampf(3.05f * trigNomPeriod, 180.0f, 2400.0f);
    const float alphaWin = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.085f)));
    ioWinSamplesSmooth = slewScalar(ioWinSamplesSmooth, tgtWinSamples, alphaWin, 0.25f);

    const float maxAvailWin = static_cast<float>((histCount > 640u) ? (histCount - 640u) : 180u);
    const float winSamples = clampf(ioWinSamplesSmooth, 180.0f, maxAvailWin);
    const size_t winInt = static_cast<size_t>(winSamples) + 2u;

    // 2. Search backward for the most recent real rising pulse of `trigVoice`
    const size_t latestStart = (histWrite + ScopeTraceEngine::kHistorySize - winInt - 8u) & kMask;
    const size_t maxBackRoom = (histCount > winInt + 320u) ? (histCount - winInt - 320u) : 0u;
    const size_t searchLim = static_cast<size_t>(1.55f * trigNomPeriod) + 48u;
    const size_t maxSearch = (searchLim < maxBackRoom) ? searchLim : maxBackRoom;

    size_t chosenBack = 0u;
    float anchorFrac = 0.0f;
    for (size_t back = 0u; back < maxSearch; ++back) {
        yieldCoProcEvery(coProcYield, back, 63u);
        const size_t pos = (latestStart + ScopeTraceEngine::kHistorySize - back) & kMask;
        if (unpackVoiceEdgeFlag(hist[pos].voiceEdgeQ12[trigVoice])) {
            chosenBack = back;
            // Sub-sample refinement for crisp high-frequency lock
            if (winSamples < 720.0f) {
                for (int off = -1; off <= 3; ++off) {
                    const size_t p1 =
                        (pos + static_cast<size_t>(
                                   off + static_cast<int>(ScopeTraceEngine::kHistorySize))) &
                        kMask;
                    const size_t p0 = (p1 + ScopeTraceEngine::kHistorySize - 1u) & kMask;
                    const float v0 = unpackVoiceEdgeSample(hist[p0].voiceEdgeQ12[trigVoice]);
                    const float v1 = unpackVoiceEdgeSample(hist[p1].voiceEdgeQ12[trigVoice]);
                    const float slope = v1 - v0;
                    if (v0 <= 0.0f && v1 > 0.0f && slope > 1.0e-4f) {
                        anchorFrac = clampf(-v0 / slope, 0.0f, 1.0f);
                        break;
                    }
                }
            }
            break;
        }
    }

    const size_t anchorPos = (latestStart + ScopeTraceEngine::kHistorySize - chosenBack) & kMask;
    const size_t remHistoryBeforeAnchor =
        (histCount > winInt + 12u + chosenBack) ? (histCount - winInt - 12u - chosenBack) : 0u;
    const size_t preRoll = (remHistoryBeforeAnchor > 960u) ? 960u : remHistoryBeforeAnchor;
    const size_t originPos = (anchorPos + ScopeTraceEngine::kHistorySize - preRoll) & kMask;
    const size_t totalScan = preRoll + winInt + 4u;

    // Choose clean power-of-2 prescaler for ultra-high pitches (>9 cycles across 180 px) so ramps
    // always remain crisp and legible even up to 20 kHz
    std::array<uint8_t, kNumOscillators> edgeDiv{1u, 1u, 1u, 1u};
    std::array<float, kNumOscillators> nomSpanSamples{};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        float p = clampf(kBaseSampleRate / clampf(liveHz[v], 0.5f, 20000.0f), 2.2f, 2400.0f);
        uint8_t div = 1u;
        while ((winSamples / p) > 9.5f && div < 64u) {
            p *= 2.0f;
            div = static_cast<uint8_t>(div << 1u);
        }
        edgeDiv[v] = div;
        nomSpanSamples[v] = p;
    }

    // 3. Pass 1: Measure real audio peak envelope & observed full-cycle span per voice
    std::array<int32_t, kNumOscillators> prevEdgeRel{-1, -1, -1, -1};
    std::array<int32_t, kNumOscillators> maxObsSpan{0, 0, 0, 0};
    std::array<uint8_t, kNumOscillators> divCount{0u, 0u, 0u, 0u};
    std::array<float, kNumOscillators> audioPeak{0.0f, 0.0f, 0.0f, 0.0f};

    for (size_t rel = 0u; rel < totalScan; ++rel) {
        yieldCoProcEvery(coProcYield, rel, 63u);
        const CompactScopeSample& smp = hist[(originPos + rel) & kMask];
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            if (rel >= preRoll) {
                const float a = absf(unpackVoiceEdgeSample(smp.voiceEdgeQ12[v]));
                if (a > audioPeak[v]) {
                    audioPeak[v] = a;
                }
            }
            if (unpackVoiceEdgeFlag(smp.voiceEdgeQ12[v])) {
                if (++divCount[v] >= edgeDiv[v]) {
                    divCount[v] = 0u;
                    const int32_t rI = static_cast<int32_t>(rel);
                    if (prevEdgeRel[v] >= 0) {
                        const int32_t sp = rI - prevEdgeRel[v];
                        if (sp > maxObsSpan[v]) {
                            maxObsSpan[v] = sp;
                        }
                    }
                    prevEdgeRel[v] = rI;
                }
            }
        }
    }

    const float alphaAct = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.022f)));
    const float alphaSpan = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.045f)));
    std::array<float, kNumOscillators> invFullSpan{};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float tgtAct =
            voiceMuted[v] ? 0.0f : clampf(0.22f + 0.85f * audioPeak[v], 0.0f, 1.0f);
        ioCapAct[v] = slewScalar(ioCapAct[v], tgtAct, alphaAct, 0.004f);

        const float obsF = static_cast<float>(maxObsSpan[v]);
        const float minNom = 0.86f * nomSpanSamples[v];
        const float tgtSpan =
            (obsF > minNom) ? obsF : ((maxObsSpan[v] > 0) ? minNom : nomSpanSamples[v]);
        ioFullSpanSmooth[v] =
            (snapImmediate || ioFullSpanSmooth[v] < 4.0f)
                ? tgtSpan
                : slewScalar(ioFullSpanSmooth[v], clampf(tgtSpan, 12.0f, 2600.0f), alphaSpan, 0.2f);
        invFullSpan[v] = fastRecipPos(ioFullSpanSmooth[v]);
    }

    // 4. Pass 2: Reconstruct exact per-column ramps, sub-pixel wraps, coincident syncs & PWR sag
    for (auto& wRow : outWrapPacked) {
        wRow.fill(0u);
    }
    outSyncMask.fill(0u);

    const float samplesPerPx = winSamples * (1.0f / static_cast<float>(kCplCols));
    const float invSamplesPerPx = fastRecipPos(samplesPerPx);
    const float invTauSag =
        fastRecipPos(clampf(0.15f * (winSamples * (1.0f / 3.05f)), 8.0f, 180.0f));
    const float syncCoincWin = (samplesPerPx > 2.2f) ? (samplesPerPx * 0.95f) : 2.2f;

    std::array<float, kNumOscillators> lastEdgeF{};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        lastEdgeF[v] = static_cast<float>(preRoll) - 0.35f * ioFullSpanSmooth[v];
        divCount[v] = 0u;
    }

    size_t scanRel = 0u;
    // Warm up `lastEdgeF` across `0 .. preRoll`
    while (scanRel < preRoll) {
        yieldCoProcEvery(coProcYield, scanRel, 63u);
        const CompactScopeSample& smp = hist[(originPos + scanRel) & kMask];
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            if (unpackVoiceEdgeFlag(smp.voiceEdgeQ12[v])) {
                if (++divCount[v] >= edgeDiv[v]) {
                    divCount[v] = 0u;
                    lastEdgeF[v] = static_cast<float>(scanRel);
                }
            }
        }
        ++scanRel;
    }

    const float c = clampf(coupling, 0.0f, 1.0f);
    const float ch = clampf(chaos, 0.0f, 1.0f);
    const float shareAmt = clampf(c * (1.0f / 0.45f), 0.0f, 1.0f);

    for (size_t col = 0u; col <= kCplCols; ++col) {
        yieldCoProcEvery(coProcYield, col, 15u);
        const float sPt =
            static_cast<float>(preRoll) + anchorFrac + static_cast<float>(col) * samplesPerPx;
        const float sPrevCol = sPt - samplesPerPx;

        uint8_t intervalFired = 0u;
        while (scanRel < totalScan && static_cast<float>(scanRel) <= sPt) {
            const CompactScopeSample& smp = hist[(originPos + scanRel) & kMask];
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (unpackVoiceEdgeFlag(smp.voiceEdgeQ12[v])) {
                    if (++divCount[v] >= edgeDiv[v]) {
                        divCount[v] = 0u;
                        const float edgeF = static_cast<float>(scanRel);
                        if (col > 0u && ioCapAct[v] > 0.03f) {
                            const float uWrap =
                                clampf((edgeF - sPrevCol) * invSamplesPerPx, 0.0f, 1.0f);
                            const float peakPre =
                                clampf((edgeF - lastEdgeF[v]) * invFullSpan[v], 0.15f, 1.0f);
                            outWrapPacked[v][col - 1u] = packCouplingWrap(uWrap, peakPre);
                            intervalFired |= static_cast<uint8_t>(1u << v);
                        }
                        lastEdgeF[v] = edgeF;
                    }
                }
            }
            ++scanRel;
        }

        // Detect real coincident sync snaps (2+ active oscillators firing together, or trigVoice
        // pulse when coupled)
        if (col > 0u && c > 0.02f && intervalFired != 0u) {
            uint8_t syncedVoices = 0u;
            for (size_t a = 0u; a < kNumOscillators; ++a) {
                if (ioCapAct[a] <= 0.04f) {
                    continue;
                }
                for (size_t b = a + 1u; b < kNumOscillators; ++b) {
                    if (ioCapAct[b] <= 0.04f) {
                        continue;
                    }
                    if (absf(lastEdgeF[a] - lastEdgeF[b]) <= syncCoincWin &&
                        (lastEdgeF[a] >= sPrevCol - 1.0f || lastEdgeF[b] >= sPrevCol - 1.0f)) {
                        syncedVoices |= static_cast<uint8_t>((1u << a) | (1u << b));
                    }
                }
            }
            if ((intervalFired & (1u << trigVoice)) != 0u && shareAmt > 0.12f) {
                syncedVoices |= static_cast<uint8_t>(1u << trigVoice);
            }
            outSyncMask[col - 1u] = syncedVoices;
        }

        // Evaluate exact continuous ramp height & shared PWR voltage sag at column endpoint `sPt`
        float sagSum = 0.0f;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float dtSmp = (sPt > lastEdgeF[v]) ? (sPt - lastEdgeF[v]) : 0.0f;
            const float ramp01 = clampf(dtSmp * invFullSpan[v], 0.0f, 1.0f) * ioCapAct[v];
            outRampQ8[v][col] = static_cast<uint8_t>(ramp01 * 255.0f + 0.5f);

            if (ioCapAct[v] > 0.03f && c > 0.002f) {
                const float wV = (v == trigVoice) ? 0.85f : 0.45f;
                sagSum += wV * ioCapAct[v] * fastExpNeg(dtSmp * invTauSag);
            }
        }
        const float sagNorm = clampf(sagSum * (0.62f * shareAmt + 0.52f * ch), 0.0f, 1.0f);
        outSagQ8[col] = static_cast<uint8_t>(sagNorm * 255.0f + 0.5f);
    }
}

// Analytical fallback when ScopeTraceEngine has no audio samples yet (e.g. silent unit-test setup)
inline void
buildCouplingFallback(float coupling, float chaos, const std::array<float, kNumOscillators>& liveHz,
                      const std::array<float, kNumOscillators>& voicePhase01,
                      const std::array<float, kNumOscillators>& capAct,
                      std::array<uint8_t, kCplPts>& outSagQ8,
                      std::array<std::array<uint8_t, kCplPts>, kNumOscillators>& outRampQ8,
                      std::array<std::array<uint8_t, kCplCols>, kNumOscillators>& outWrapPacked,
                      std::array<uint8_t, kCplCols>& outSyncMask) noexcept {
    for (auto& wRow : outWrapPacked) {
        wRow.fill(0u);
    }
    outSyncMask.fill(0u);
    const float c = clampf(coupling, 0.0f, 1.0f);
    const float ch = clampf(chaos, 0.0f, 1.0f);
    const float shareAmt = clampf(c * (1.0f / 0.45f), 0.0f, 1.0f);
    const float refHz = (liveHz[0] > 0.5f) ? liveHz[0] : 55.0f;

    std::array<float, kNumOscillators> ph{};
    std::array<float, kNumOscillators> inc{};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        ph[v] = wrapLfoPhase01(voicePhase01[v]);
        inc[v] = (1.0f / 56.0f) * clampf(liveHz[v] / refHz, 0.55f, 2.60f);
    }

    float sag = 0.0f;
    for (size_t col = 0u; col <= kCplCols; ++col) {
        sag *= 0.88f;
        if (col > 0u) {
            uint8_t fired = 0u;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                ph[v] += inc[v];
                if (ph[v] >= 1.0f) {
                    const float uWrap = clampf(1.0f - (ph[v] - 1.0f) / inc[v], 0.0f, 1.0f);
                    outWrapPacked[v][col - 1u] = packCouplingWrap(uWrap, 1.0f);
                    ph[v] -= 1.0f;
                    fired |= static_cast<uint8_t>(1u << v);
                    sag = clampf(sag + 0.45f * shareAmt + 0.45f * ch, 0.0f, 1.0f);
                }
            }
            if ((fired & 1u) != 0u && ch > 0.05f) {
                uint8_t syncM = 1u;
                for (size_t j = 1u; j < kNumOscillators; ++j) {
                    if (ph[j] > 0.45f) {
                        outWrapPacked[j][col - 1u] = packCouplingWrap(0.5f, ph[j]);
                        ph[j] = 0.0f;
                        syncM |= static_cast<uint8_t>(1u << j);
                    }
                }
                outSyncMask[col - 1u] = syncM;
            }
        }
        outSagQ8[col] = static_cast<uint8_t>(sag * 255.0f + 0.5f);
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            outRampQ8[v][col] =
                static_cast<uint8_t>(clampf(ph[v] * capAct[v], 0.0f, 1.0f) * 255.0f + 0.5f);
        }
    }
}

// Render Concept D (Top PWR Voltage Rail + 4 Live Oscillator Charge-and-Fire Ramp Lanes)
inline void
renderCouplingStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf, uint16_t* linScratch,
                    float effCouplingAlpha01, uint8_t alpha8, float couplingSmooth,
                    float couplingChaos, const std::array<uint8_t, kCplPts>& sagQ8,
                    const std::array<std::array<uint8_t, kCplPts>, kNumOscillators>& rampQ8,
                    const std::array<std::array<uint8_t, kCplCols>, kNumOscillators>& wrapPacked,
                    const std::array<uint8_t, kCplCols>& syncMask,
                    const std::array<float, kNumOscillators>& capAct,
                    const std::array<float, kNumOscillators>& muteDim,
                    CoProcYieldFn coProcYield = nullptr) noexcept {
    if (alpha8 == 0u || y1 <= kGraphStripMinY1 || y0 >= kGraphStripMaxY0) {
        return;
    }

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    const float c = clampf(couplingSmooth, 0.0f, 1.0f);
    const float ch = clampf(couplingChaos, 0.0f, 1.0f);
    const float shareAmt = clampf(c * (1.0f / 0.45f), 0.0f, 1.0f);
    const float masterE = clampf(effCouplingAlpha01, 0.0f, 1.0f);

    static constexpr const char* kOscLabels[kNumOscillators] = {"OSC 1", "OSC 2", "OSC 3", "OSC 4"};
    std::array<float, kNumOscillators> actW{};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        actW[v] = clampf(capAct[v] * (1.0f - 0.85f * muteDim[v]), 0.0f, 1.0f);
    }

    // 1. Subtle industrial reference horizons & left-column labels (PWR, OSC 1..4)
    cv.drawHorizontalFilament(static_cast<float>(kCplTrackX0), static_cast<float>(kCplTrackX1),
                              kCplPwrBotY, 20.0f, 0.48f, false);
    if (ch > 0.01f) {
        cv.drawHorizontalFilament(static_cast<float>(kCplTrackX0), static_cast<float>(kCplTrackX1),
                                  kCplPwrTopY, 18.0f * ch, 0.45f, false);
    }
    cv.drawSans(kFontSansSmall, 10, 41, "PWR", 122u, 0);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float laneBot =
            kCplLaneTop0 + static_cast<float>(v) * kCplLaneStride + kCplLaneHeight;
        cv.drawHorizontalFilament(static_cast<float>(kCplTrackX0), static_cast<float>(kCplTrackX1),
                                  laneBot, 22.0f, 0.48f, false);
        const auto lblLum = static_cast<uint8_t>(68.0f + 56.0f * actW[v] + 0.5f);
        cv.drawSans(kFontSansSmall, 10, static_cast<int>(laneBot) - 11, kOscLabels[v], lblLum, 0);
    }

    const auto evalPwrY = [&](size_t colIdx) noexcept -> float {
        const float sag01 = static_cast<float>(sagQ8[colIdx]) * (1.0f / 255.0f);
        const float baseRailY = kCplPwrTopY + ch * 7.5f;
        const float dipPx = sag01 * (14.0f + 6.0f * ch);
        return clampf(baseRailY + dipPx, kCplPwrTopY, kCplPwrBotY);
    };

    // At 0% coupling, 3 clean gaps split the PWR rail into 4 isolated power segments
    const float isoGapOpen = clampf(1.0f - c * (1.0f / 0.06f), 0.0f, 1.0f);
    const auto pwrGapWeight = [isoGapOpen](int x) noexcept -> float {
        if (isoGapOpen <= 0.01f) {
            return 1.0f;
        }
        constexpr int kGapCenters[3] = {93, 138, 183};
        for (int g : kGapCenters) {
            const int d = (x > g) ? (x - g) : (g - x);
            if (d <= 2) {
                return 1.0f - isoGapOpen;
            }
            if (d == 3) {
                return 1.0f - 0.5f * isoGapOpen;
            }
        }
        return 1.0f;
    };

    // 2. Live vertical coupling/sync filaments at real coincident oscillator pulses
    if (c > 0.02f) {
        for (size_t col = 1u; col + 1u < kCplCols; ++col) {
            yieldCoProcEvery(coProcYield, col, 31u);
            const uint8_t sMask = syncMask[col];
            if (sMask == 0u) {
                continue;
            }
            // Find the lowest-downward synced voice so the sync line spans from PWR to that voice!
            size_t maxV = 0u;
            uint8_t nSynced = 0u;
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if ((sMask & (1u << v)) != 0u) {
                    maxV = v;
                    ++nSynced;
                }
            }
            const float xF = static_cast<float>(kCplTrackX0 + static_cast<int>(col)) + 0.5f;
            const float yRail = evalPwrY(col + 1u);
            const float yBotSync =
                kCplLaneTop0 + static_cast<float>((nSynced >= 2u) ? maxV : 3u) * kCplLaneStride +
                kCplLaneHeight;
            const float syncLum = (nSynced >= 2u) ? (22.0f + 16.0f * shareAmt + 26.0f * ch)
                                                  : (14.0f + 12.0f * shareAmt);
            cv.drawVerticalSlit(xF, yRail + 1.0f, yBotSync, syncLum);
        }
    }

    // 3. Linear-photon CRT phosphor glow + sub-pixel anti-aliased spines
    if (masterE > 0.004f && y1 > 36 && y0 <= 184) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));

        // (a) Top PWR rail soft downward glow + spine
        if (y1 > static_cast<int>(kCplPwrTopY) - 3 && y0 <= static_cast<int>(kCplPwrBotY) + 3) {
            for (size_t col = 0u; col < kCplCols; ++col) {
                yieldCoProcEvery(coProcYield, col, 15u);
                const int x = kCplTrackX0 + static_cast<int>(col);
                const float wGap = pwrGapWeight(x);
                if (wGap <= 0.02f) {
                    continue;
                }
                const float yA = evalPwrY(col);
                const float yB = evalPwrY(col + 1u);
                const float yMid = 0.5f * (yA + yB);

                int pyStart = static_cast<int>(yMid);
                if (pyStart < y0) {
                    pyStart = y0;
                }
                int pyEnd = static_cast<int>(kCplPwrBotY);
                if (pyEnd > y1 - 1) {
                    pyEnd = y1 - 1;
                }
                for (int py = pyStart; py <= pyEnd; ++py) {
                    const float dy = static_cast<float>(py) - yMid;
                    if (dy >= 0.0f) {
                        const float glowE = 82.0f * fastExpNeg(dy * 0.16f) * wGap * masterE;
                        if (glowE >= 1.0f) {
                            uint16_t& cell =
                                linScratch[static_cast<size_t>(py - y0) * kDisplayWidth +
                                           static_cast<size_t>(x)];
                            const uint32_t next =
                                static_cast<uint32_t>(cell) + static_cast<uint32_t>(glowE + 0.5f);
                            cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                        }
                    }
                }
                splatPhotonSegment(x, yA, yB, 0.72f * wGap * masterE, 0.60f, y0, y1, linScratch);
            }
        }

        // (b) 4 Oscillator Charge-and-Fire Ramp Lanes with sub-pixel wrap distribution
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float act = actW[v];
            if (act <= 0.02f) {
                continue;
            }
            const float laneTop = kCplLaneTop0 + static_cast<float>(v) * kCplLaneStride;
            const float laneBot = laneTop + kCplLaneHeight;
            if (y1 <= static_cast<int>(laneTop) - 2 || y0 > static_cast<int>(laneBot) + 2) {
                continue;
            }

            const float baseGlowE = ((v == 0u) ? 68.0f : 52.0f) * (0.45f + 0.55f * act) * masterE;
            const float spineWgt = ((v == 0u) ? 0.64f : 0.48f) * (0.45f + 0.55f * act) * masterE;
            const float dropWgt = spineWgt * 0.78f;

            for (size_t col = 0u; col < kCplCols; ++col) {
                yieldCoProcEvery(coProcYield, col, 31u);
                const int x = kCplTrackX0 + static_cast<int>(col);
                const float rA = static_cast<float>(rampQ8[v][col]) * (1.0f / 255.0f);
                const float rB = static_cast<float>(rampQ8[v][col + 1u]) * (1.0f / 255.0f);
                const float yA = laneBot - rA * kCplLaneHeight;
                const float yB = laneBot - rB * kCplLaneHeight;
                const uint8_t wp = wrapPacked[v][col];

                const float rGlow = (wp == 0u) ? (0.5f * (rA + rB)) : rA;
                const float yGlowTop = laneBot - rGlow * kCplLaneHeight;
                int pyStart = static_cast<int>(yGlowTop);
                if (pyStart < y0) {
                    pyStart = y0;
                }
                int pyEnd = static_cast<int>(laneBot);
                if (pyEnd > y1 - 1) {
                    pyEnd = y1 - 1;
                }
                for (int py = pyStart; py <= pyEnd; ++py) {
                    const float dy = static_cast<float>(py) - yGlowTop;
                    if (dy >= 0.0f) {
                        const float glowE = baseGlowE * fastExpNeg(dy * 0.20f);
                        if (glowE >= 1.0f) {
                            uint16_t& cell =
                                linScratch[static_cast<size_t>(py - y0) * kDisplayWidth +
                                           static_cast<size_t>(x)];
                            const uint32_t next =
                                static_cast<uint32_t>(cell) + static_cast<uint32_t>(glowE + 0.5f);
                            cell = static_cast<uint16_t>((next > 4095u) ? 4095u : next);
                        }
                    }
                }

                if (wp == 0u) {
                    splatPhotonSegment(x, yA, yB, spineWgt, 0.58f, y0, y1, linScratch);
                } else {
                    float uWrap = 0.5f;
                    float peak01 = 1.0f;
                    unpackCouplingWrap(wp, uWrap, peak01);
                    const float yPeak = laneBot - (peak01 * act) * kCplLaneHeight;
                    if (1.0f - uWrap > 0.02f) {
                        splatPhotonSegment(x, yPeak, yB, dropWgt * (1.0f - uWrap), 0.58f, y0, y1,
                                           linScratch);
                    }
                    if (uWrap > 0.02f && x + 1 < kCplTrackX1) {
                        splatPhotonSegment(x + 1, yPeak, yB, dropWgt * uWrap, 0.58f, y0, y1,
                                           linScratch);
                    }
                }
            }
        }

        screenBlendLinearStrip(stripHeight, static_cast<size_t>(kCplTrackX0 - 2),
                               static_cast<size_t>(kCplTrackX1 + 2), linScratch, lumBuf);
    }

    // 4. Subtle sub-pixel optical beads at real synced resets / mid-cycle chops (>45% regime)
    if (ch > 0.04f) {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            if (actW[v] <= 0.05f) {
                continue;
            }
            const float laneBot =
                kCplLaneTop0 + static_cast<float>(v) * kCplLaneStride + kCplLaneHeight;
            if (y1 < static_cast<int>(laneBot) - 3 || y0 > static_cast<int>(laneBot) + 3) {
                continue;
            }
            for (size_t col = 1u; col + 1u < kCplCols; ++col) {
                yieldCoProcEvery(coProcYield, col, 31u);
                const uint8_t wp = wrapPacked[v][col];
                if (wp == 0u) {
                    continue;
                }
                const uint8_t sMask = syncMask[col];
                float uWrap = 0.5f;
                float peak01 = 1.0f;
                unpackCouplingWrap(wp, uWrap, peak01);
                const bool syncedHere = (sMask & (1u << v)) != 0u && ((sMask & (sMask - 1u)) != 0u);
                const bool midCycleChopped = (peak01 < 0.90f);
                if (syncedHere || midCycleChopped) {
                    const float xBead =
                        static_cast<float>(kCplTrackX0 + static_cast<int>(col)) + uWrap;
                    cv.drawOpticalBead(xBead, laneBot, 0.54f * ch * actW[v], 1.38f);
                }
            }
        }
    }
}

} // namespace ruido
