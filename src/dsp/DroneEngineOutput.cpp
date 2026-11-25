#include "core/DspMath.h"
#include "dsp/DroneEngine.h"

namespace ruido {

RUIDO_NOINLINE void DroneEngineCore::finishInFlightFilterStage(InFlightFilterSpan& span) noexcept {
    if (!span.valid) {
        return;
    }
    auto& spanSub = spanSub_[span.bufIdx & 1u];
    const auto& coeffs = spanCoeffs_[span.bufIdx & 1u];
    if (span.coProcHandled && coProc_ != nullptr) {
        coProc_->finishBalanced4x(spanSub, filters_, coeffs);
    }
    if (span.anyCrush) {
        applySpanCrushPostFilter4x(spanSub, span.chunk, span.crushDepths, hzToOversampledPhaseU32_,
                                   crushStates_);
    } else {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            crushStates_[v].snhPhase = 0u;
            const float rest = sanitizeState(spanSub[v][span.chunk - 1u][kOversampleFactor - 1u]);
            crushStates_[v].heldSample = rest;
            crushStates_[v].capSample = rest;
        }
    }
}

RUIDO_NOINLINE void DroneEngineCore::launchSpanFilters(
    size_t bufIdx, size_t chunk, const std::array<float, kNumOscillators>& startGainL,
    const std::array<float, kNumOscillators>& endGainL,
    const std::array<float, kNumOscillators>& startGainR,
    const std::array<float, kNumOscillators>& endGainR,
    const std::array<float, kNumOscillators>& startScope,
    const std::array<float, kNumOscillators>& endScope, bool& outCoProcHandled) noexcept {
    auto& spanSub = spanSub_[bufIdx & 1u];
    const auto& coeffs = spanCoeffs_[bufIdx & 1u];

    uint8_t activeMask = 0u;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const uint8_t vBit = static_cast<uint8_t>(1u << v);
        const bool voiceActive = (endGainL[v] != 0.0f) || (endGainR[v] != 0.0f) ||
                                 (startGainL[v] != 0.0f) || (startGainR[v] != 0.0f) ||
                                 (endScope[v] != 0.0f) || (startScope[v] != 0.0f);
        if (voiceActive) {
            activeMask |= vBit;
            silentFilterMask_ = static_cast<uint8_t>(silentFilterMask_ & ~vBit);
        } else {
            if ((silentFilterMask_ & vBit) == 0u) {
                filters_[v].reset();
                silentFilterMask_ |= vBit;
            }
            for (size_t k = 0u; k < chunk; ++k) {
                spanSub[v][k] = {0.0f, 0.0f, 0.0f, 0.0f};
            }
        }
    }

    const bool coProcHandled =
        (activeMask != 0u) && (coProc_ != nullptr) && (chunk >= 4u) &&
        coProc_->beginBalanced4x(spanSub, chunk, filters_, coeffs, activeMask);
    outCoProcHandled = coProcHandled;
    if (activeMask != 0u && !coProcHandled) {
#if !(defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__))
        if (activeMask == 0x0Fu) {
            std::array<std::array<float, kOversampleFactor>, kNumOscillators> quadSub{};
            for (size_t k = 0u; k < chunk; ++k) {
                for (size_t v = 0u; v < kNumOscillators; ++v) {
                    quadSub[v] = spanSub[v][k];
                }
                Ms20DualFilter::processQuadBank4x(filters_, quadSub, coeffs);
                for (size_t v = 0u; v < kNumOscillators; ++v) {
                    spanSub[v][k] = quadSub[v];
                }
            }
        } else
#endif
        {
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if ((activeMask & static_cast<uint8_t>(1u << v)) != 0u) {
                    filters_[v].processSpan4x(spanSub[v].data(), chunk, coeffs[v]);
                }
            }
        }
    }
}

RUIDO_NOINLINE void
DroneEngineCore::mixAndFlushCompletedSpan(const InFlightFilterSpan& span, EngineSharedBuses& buses,
                                          float* outLeft, float* outRight,
                                          std::array<float, kNumOscillators>& blockVoicePeak,
                                          float& blockPeakL, float& blockPeakR) noexcept {
    if (!span.valid) {
        return;
    }
    const size_t chunk = span.chunk;
    const float invChunk = span.invChunk;
    const bool chunkExactMono = span.chunkExactMono;
    const auto& spanSub = spanSub_[span.bufIdx & 1u];
    auto& osStereoSpan = osStereoSpan_;

    std::array<float, kNumOscillators> curGainL = span.startGainL, curGainR = span.startGainR;
    std::array<float, kNumOscillators> stepGainL, stepGainR;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        stepGainL[i] = (span.endGainL[i] - span.startGainL[i]) * invChunk;
        stepGainR[i] = (span.endGainR[i] - span.startGainR[i]) * invChunk;
    }
    float curRun = span.startRun;
    const float stepRun = (span.endRun - span.startRun) * invChunk;

    for (size_t k = 0u; k < chunk; ++k) {
        if (k + 1u == chunk) {
            curGainL = span.endGainL;
            curGainR = span.endGainR;
            curRun = span.endRun;
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                curGainL[i] += stepGainL[i];
                curGainR[i] += stepGainR[i];
            }
            curRun += stepRun;
        }
        const float gL0 = curGainL[0], gL1 = curGainL[1], gL2 = curGainL[2], gL3 = curGainL[3];
        const auto s0 = spanSub[0][k], s1 = spanSub[1][k], s2 = spanSub[2][k], s3 = spanSub[3][k];
        auto& osStereo = osStereoSpan[k];

        const float gR0 = chunkExactMono ? gL0 : curGainR[0];
        const float gR1 = chunkExactMono ? gL1 : curGainR[1];
        const float gR2 = chunkExactMono ? gL2 : curGainR[2];
        const float gR3 = chunkExactMono ? gL3 : curGainR[3];
        for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
            const float sumL = s0[sub] * gL0 + s1[sub] * gL1 + s2[sub] * gL2 + s3[sub] * gL3;
            const float busL = masterBusSoftClip(sumL) * curRun;
            const float busR = chunkExactMono ? busL
                                              : (masterBusSoftClip(s0[sub] * gR0 + s1[sub] * gR1 +
                                                                   s2[sub] * gR2 + s3[sub] * gR3) *
                                                 curRun);
            osStereo[sub] = {busL, busR};
        }
    }

    std::array<std::array<int16_t, kNumOscillators>, kMaxFilterSpanFrames> voiceQ13{};
    std::array<float, kNumOscillators> chunkRawPeak{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> curScope = span.startScope;
    std::array<float, kNumOscillators> stepScope;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        stepScope[i] = (span.endScope[i] - span.startScope[i]) * invChunk;
    }
    for (size_t k = 0u; k < chunk; ++k) {
        if (k + 1u == chunk) {
            curScope = span.endScope;
        } else {
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                curScope[i] += stepScope[i];
            }
        }
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            const auto& s = spanSub[i][k];
            const float vAccum = s[0] + s[1] + s[2] + s[3];
            const float vAbs = absf(vAccum);
            chunkRawPeak[i] = (vAbs > chunkRawPeak[i]) ? vAbs : chunkRawPeak[i];
            voiceQ13[k][i] = quantizeVoiceScopeQ13(vAccum * curScope[i]);
        }
    }
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const float scaledPeak = chunkRawPeak[i] * 0.25f * span.vcaGates[i] * span.endRun;
        blockVoicePeak[i] = (scaledPeak > blockVoicePeak[i]) ? scaledPeak : blockVoicePeak[i];
    }

    StereoFrame* decimatedSpan = osStereoSpan[0].data();
    decimator_.processSpan4x(osStereoSpan.data(), decimatedSpan, chunk, chunkExactMono);

    size_t scopeWritePos = 0u;
    const size_t scopeAvail = buses.scopeBuffer.beginPushSpan(chunk, scopeWritePos);
    float curTrim = span.startTrim;
    const float stepTrim = (span.endTrim - span.startTrim) * invChunk;
    float peakL = blockPeakL, peakR = blockPeakR;
    uint32_t delaySlot = voiceScopeDelayWrite_;
    for (size_t k = 0u; k < chunk; ++k) {
        const size_t frame = span.baseFrame + k;
        curTrim = (k + 1u == chunk) ? span.endTrim : (curTrim + stepTrim);

        const float rawL = sanitizeState(decimatedSpan[k].left);
        const float rawR = sanitizeState(decimatedSpan[k].right);
        const float outL = dacSafetyClip(rawL * curTrim);
        const float outR = dacSafetyClip(rawR * curTrim);
        outLeft[frame] = outL;
        outRight[frame] = outR;

        const float absL = absf(outL), absR = absf(outR);
        peakL = (absL > peakL) ? absL : peakL;
        peakR = (absR > peakR) ? absR : peakR;

        voiceScopeDelay_[delaySlot] = voiceQ13[k];
        voiceRisingMaskDelay_[delaySlot] = span.rising[k];
        delaySlot = (delaySlot + 1u < kScopeAlignDelaySlots) ? (delaySlot + 1u) : 0u;
        if (k < scopeAvail) {
            ScopeFrame& dst = buses.scopeBuffer.slotAt(scopeWritePos + k);
            dst.leftQ13 = quantizeVoiceScopeQ13(dacSafetyClip(rawL));
            dst.rightQ13 = quantizeVoiceScopeQ13(dacSafetyClip(rawR));
            dst.voiceQ13 = voiceScopeDelay_[delaySlot];
            dst.voiceRisingMask = voiceRisingMaskDelay_[delaySlot];
        }
    }
    voiceScopeDelayWrite_ = delaySlot;
    blockPeakL = peakL;
    blockPeakR = peakR;
    if (scopeAvail > 0u) {
        buses.scopeBuffer.commitPushSpan(scopeWritePos + scopeAvail);
    }
}

} // namespace ruido
