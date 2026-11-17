#include "core/DspMath.h"
#include "core/SnapshotBank.h"
#include "core/SnapshotPresets.h"

namespace ruido {

namespace {

constexpr float kFreqLogMin = cxLog2Pos(kAudioFreqMinHz);
constexpr float kFreqLogSpan = cxLog2Pos(kAudioFreqMaxHz) - kFreqLogMin;
constexpr float kFltLogMin = cxLog2Pos(kFilterCutoffMinHz);
constexpr float kFltLogSpan = cxLog2Pos(kFilterCutoffMaxHz) - kFltLogMin;
constexpr float kLfoLogMinSnap = cxLog2Pos(kLfoFreqMinHz);
constexpr float kLfoLogSpanSnap = cxLog2Pos(kLfoFreqMaxHz) - kLfoLogMinSnap;

inline uint16_t quantizeNormQ16(float norm01) noexcept {
    const float c = clampf(norm01, 0.0f, 1.0f);
    return static_cast<uint16_t>(c * 65535.0f + 0.5f);
}

inline constexpr float unquantizeNormQ16(uint16_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 65535.0f);
}

inline uint8_t quantizeNormQ8(float norm01) noexcept {
    const float c = clampf(norm01, 0.0f, 1.0f);
    return static_cast<uint8_t>(c * 255.0f + 0.5f);
}

inline constexpr float unquantizeNormQ8(uint8_t q) noexcept {
    return static_cast<float>(q) * (1.0f / 255.0f);
}

inline float interpLogRange(float a, float b, float t, float lo, float hi) noexcept {
    const float la = fastLog2(clampf(a, lo, hi));
    const float lb = fastLog2(clampf(b, lo, hi));
    return clampf(fastExp2(la + t * (lb - la)), lo, hi);
}

inline uint32_t computeSnapshotPayloadCrc32(const PatchSnapshot& snap) noexcept {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&snap) + sizeof(uint32_t);
    return computeCrc32Bytes(bytes, sizeof(PatchSnapshot) - sizeof(uint32_t));
}

} // namespace

EngineControlState makeFactorySnapshotPreset(size_t slot) noexcept {
    EngineControlState s = EngineControlState::makeDefault();
    const FactorySceneSpec& spec = kFactoryScenes[(slot < kNumSnapshots) ? slot : 3u];
    s.global.circuitCoupling = static_cast<float>(spec.couplingPct) * 0.01f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoicePresetSpec& vs = spec.voices[v];
        VoiceParams& vp = s.voices[v];
        vp.frequencyHz = vs.freqHz;
        vp.pulseWidth = static_cast<float>(vs.dutyPct) * 0.01f;
        vp.drift = static_cast<float>(vs.driftPct) * 0.01f;
        vp.hpCutoffHz = static_cast<float>(vs.hpHz);
        vp.hpResonance = static_cast<float>(vs.hpResPct) * 0.01f;
        vp.lpCutoffHz = static_cast<float>(vs.lpHz);
        vp.lpResonance = static_cast<float>(vs.lpResPct) * 0.01f;
        vp.amplitude = static_cast<float>(vs.ampPct) * 0.01f;
        vp.pan = static_cast<float>(vs.panPct) * 0.01f;
        vp.lfoRateHz = static_cast<float>(vs.lfoAMilliHz) * 0.001f;
        vp.lfoShape = static_cast<float>(vs.lfoAShapePct) * 0.01f;
        vp.lfoDepth = static_cast<float>(vs.lfoADepthPct) * 0.01f;
        vp.lfoBRateHz = static_cast<float>(vs.lfoBMilliHz) * 0.001f;
        vp.lfoBShape = static_cast<float>(vs.lfoBShapePct) * 0.01f;
        vp.lfoBDepth = static_cast<float>(vs.lfoBDepthPct) * 0.01f;
        vp.lfoTarget = static_cast<LfoTarget>(vs.targetsPacked & 0x0Fu);
        vp.lfoBTarget = static_cast<LfoTarget>((vs.targetsPacked >> 4u) & 0x0Fu);
        vp.muted = false;
    }
    return s;
}

PatchSnapshot encodePatchSnapshot(const EngineControlState& state) noexcept {
    PatchSnapshot snap{};
    snap.version = 1u;
    snap.flags = kSnapshotFlagOccupied;
    snap.circuitCouplingQ16 = quantizeNormQ16(state.global.circuitCoupling);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = state.voices[v];
        VoiceSnapshotBin& dst = snap.voices[v];
        const float hz = clampf(vp.frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
        const float hp = clampf(vp.hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float lp = clampf(vp.lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float rA = clampf(vp.lfoRateHz, kLfoFreqMinHz, kLfoFreqMaxHz);
        const float rB = clampf(vp.lfoBRateHz, kLfoFreqMinHz, kLfoFreqMaxHz);

        dst.freqLogQ16 = quantizeNormQ16((cxLog2Pos(hz) - kFreqLogMin) * (1.0f / kFreqLogSpan));
        dst.pulseWidthQ16 = quantizeNormQ16(
            (clampf(vp.pulseWidth, kPulseWidthMin, kPulseWidthMax) - kPulseWidthMin) *
            (1.0f / (kPulseWidthMax - kPulseWidthMin)));
        dst.driftQ16 = quantizeNormQ16(vp.drift);
        dst.hpCutLogQ16 = quantizeNormQ16((cxLog2Pos(hp) - kFltLogMin) * (1.0f / kFltLogSpan));
        dst.hpResQ16 = quantizeNormQ16(vp.hpResonance);
        dst.lpCutLogQ16 = quantizeNormQ16((cxLog2Pos(lp) - kFltLogMin) * (1.0f / kFltLogSpan));
        dst.lpResQ16 = quantizeNormQ16(vp.lpResonance);
        dst.amplitudeQ16 = quantizeNormQ16(vp.amplitude);
        dst.panBiQ16 = quantizeNormQ16(0.5f * (clampf(vp.pan, -1.0f, 1.0f) + 1.0f));
        dst.lfoRateLogQ16 =
            quantizeNormQ16((cxLog2Pos(rA) - kLfoLogMinSnap) * (1.0f / kLfoLogSpanSnap));
        dst.lfoDepthQ16 = quantizeNormQ16(vp.lfoDepth);
        dst.lfoBRateLogQ16 =
            quantizeNormQ16((cxLog2Pos(rB) - kLfoLogMinSnap) * (1.0f / kLfoLogSpanSnap));
        dst.lfoBDepthQ16 = quantizeNormQ16(vp.lfoBDepth);
        dst.lfoShapeQ8 = quantizeNormQ8(vp.lfoShape);
        dst.lfoBShapeQ8 = quantizeNormQ8(vp.lfoBShape);

        const auto tA = static_cast<uint8_t>(vp.lfoTarget) & 0x0Fu;
        const auto tB = static_cast<uint8_t>(vp.lfoBTarget) & 0x0Fu;
        dst.lfoTargetsPacked = static_cast<uint8_t>(tA | (tB << 4u));
        dst.flags = vp.muted ? 1u : 0u;
    }

    snap.crc32 = computeSnapshotPayloadCrc32(snap);
    return snap;
}

PatchSnapshot makeEmptyPatchSnapshot() noexcept {
    PatchSnapshot snap{};
    snap.version = 1u;
    snap.flags = 0u;
    snap.crc32 = computeSnapshotPayloadCrc32(snap);
    return snap;
}

PatchFrame decodePatchSnapshot(const PatchSnapshot& snap) noexcept {
    PatchFrame out{};
    out.circuitCoupling = unquantizeNormQ16(snap.circuitCouplingQ16);
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceSnapshotBin& src = snap.voices[v];
        VoiceParams& vp = out.voices[v];
        vp.frequencyHz =
            clampf(fastExp2(kFreqLogMin + unquantizeNormQ16(src.freqLogQ16) * kFreqLogSpan),
                   kAudioFreqMinHz, kAudioFreqMaxHz);
        vp.pulseWidth = clampf(kPulseWidthMin + unquantizeNormQ16(src.pulseWidthQ16) *
                                                    (kPulseWidthMax - kPulseWidthMin),
                               kPulseWidthMin, kPulseWidthMax);
        vp.drift = unquantizeNormQ16(src.driftQ16);
        vp.hpCutoffHz =
            clampf(fastExp2(kFltLogMin + unquantizeNormQ16(src.hpCutLogQ16) * kFltLogSpan),
                   kFilterCutoffMinHz, kFilterCutoffMaxHz);
        vp.hpResonance = unquantizeNormQ16(src.hpResQ16);
        vp.lpCutoffHz =
            clampf(fastExp2(kFltLogMin + unquantizeNormQ16(src.lpCutLogQ16) * kFltLogSpan),
                   kFilterCutoffMinHz, kFilterCutoffMaxHz);
        vp.lpResonance = unquantizeNormQ16(src.lpResQ16);
        vp.amplitude = unquantizeNormQ16(src.amplitudeQ16);
        vp.pan = clampf(2.0f * unquantizeNormQ16(src.panBiQ16) - 1.0f, -1.0f, 1.0f);
        vp.lfoRateHz = clampf(
            fastExp2(kLfoLogMinSnap + unquantizeNormQ16(src.lfoRateLogQ16) * kLfoLogSpanSnap),
            kLfoFreqMinHz, kLfoFreqMaxHz);
        vp.lfoDepth = unquantizeNormQ16(src.lfoDepthQ16);
        vp.lfoBRateHz = clampf(
            fastExp2(kLfoLogMinSnap + unquantizeNormQ16(src.lfoBRateLogQ16) * kLfoLogSpanSnap),
            kLfoFreqMinHz, kLfoFreqMaxHz);
        vp.lfoBDepth = unquantizeNormQ16(src.lfoBDepthQ16);
        vp.lfoShape = unquantizeNormQ8(src.lfoShapeQ8);
        vp.lfoBShape = unquantizeNormQ8(src.lfoBShapeQ8);

        const uint8_t rawTA = src.lfoTargetsPacked & 0x0Fu;
        const uint8_t rawTB = (src.lfoTargetsPacked >> 4u) & 0x0Fu;
        vp.lfoTarget =
            (rawTA < kNumLfoATargets) ? static_cast<LfoTarget>(rawTA) : LfoTarget::LpCutoff;
        vp.lfoBTarget =
            (rawTB < kNumLfoBTargets) ? static_cast<LfoTarget>(rawTB) : LfoTarget::LpCutoff;
        vp.muted = (src.flags & 1u) != 0u;
    }
    return out;
}

bool verifyPatchSnapshotCrc(const PatchSnapshot& snap) noexcept {
    return (snap.version == 1u) && (snap.crc32 == computeSnapshotPayloadCrc32(snap));
}

PatchFrame extractPatchFrame(const EngineControlState& state) noexcept {
    return {state.voices, state.global.circuitCoupling};
}

void applyPatchFrame(const PatchFrame& patch, EngineControlState& state) noexcept {
    state.voices = patch.voices;
    state.global.circuitCoupling = clampf(patch.circuitCoupling, 0.0f, 1.0f);
}

PatchFrame morphPatchFrames(const PatchFrame& a, const PatchFrame& b, float t01) noexcept {
    const float t = clampf(t01, 0.0f, 1.0f);
    if (t <= 0.0f) {
        return a;
    }
    if (t >= 1.0f) {
        return b;
    }

    PatchFrame out{};
    out.circuitCoupling = a.circuitCoupling + t * (b.circuitCoupling - a.circuitCoupling);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& va = a.voices[v];
        const VoiceParams& vb = b.voices[v];
        VoiceParams& dst = out.voices[v];

        dst.frequencyHz =
            interpLogRange(va.frequencyHz, vb.frequencyHz, t, kAudioFreqMinHz, kAudioFreqMaxHz);
        dst.hpCutoffHz =
            interpLogRange(va.hpCutoffHz, vb.hpCutoffHz, t, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        dst.lpCutoffHz =
            interpLogRange(va.lpCutoffHz, vb.lpCutoffHz, t, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        dst.lfoRateHz = interpLogRange(va.lfoRateHz, vb.lfoRateHz, t, kLfoFreqMinHz, kLfoFreqMaxHz);
        dst.lfoBRateHz =
            interpLogRange(va.lfoBRateHz, vb.lfoBRateHz, t, kLfoFreqMinHz, kLfoFreqMaxHz);

        dst.pulseWidth = va.pulseWidth + t * (vb.pulseWidth - va.pulseWidth);
        dst.drift = va.drift + t * (vb.drift - va.drift);
        dst.hpResonance = va.hpResonance + t * (vb.hpResonance - va.hpResonance);
        dst.lpResonance = va.lpResonance + t * (vb.lpResonance - va.lpResonance);
        dst.pan = va.pan + t * (vb.pan - va.pan);
        dst.lfoShape = va.lfoShape + t * (vb.lfoShape - va.lfoShape);
        dst.lfoDepth = va.lfoDepth + t * (vb.lfoDepth - va.lfoDepth);
        dst.lfoBShape = va.lfoBShape + t * (vb.lfoBShape - va.lfoBShape);
        dst.lfoBDepth = va.lfoBDepth + t * (vb.lfoBDepth - va.lfoBDepth);
        dst.lfoTarget = (t < 0.5f) ? va.lfoTarget : vb.lfoTarget;
        dst.lfoBTarget = (t < 0.5f) ? va.lfoBTarget : vb.lfoBTarget;

        // Smooth amplitude swell across mute boundaries so muted <-> active voices never pop
        if (va.muted == vb.muted) {
            dst.muted = va.muted;
            dst.amplitude = va.amplitude + t * (vb.amplitude - va.amplitude);
        } else if (va.muted) {
            dst.muted = (t <= 0.001f);
            dst.amplitude = dst.muted ? va.amplitude : (t * vb.amplitude);
        } else {
            dst.muted = (t >= 0.999f);
            dst.amplitude = dst.muted ? vb.amplitude : ((1.0f - t) * va.amplitude);
        }
    }
    return out;
}

PatchFrame blendPatchSnapshots(const std::array<PatchSnapshot, kNumSnapshots>& slots,
                               const std::array<float, kNumSnapshots>& normWeights,
                               const PatchFrame& fallbackFrame) noexcept {
    size_t domSlot = 0u;
    float domWeight = -1.0f;
    float keptSum = 0.0f;
    size_t nonZeroCount = 0u;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (isPatchSnapshotOccupied(slots[s]) && normWeights[s] > 1.0e-4f) {
            ++nonZeroCount;
            keptSum += normWeights[s];
            if (normWeights[s] > domWeight) {
                domWeight = normWeights[s];
                domSlot = s;
            }
        }
    }
    if (nonZeroCount == 0u) {
        return fallbackFrame;
    }
    if (nonZeroCount == 1u) {
        return decodePatchSnapshot(slots[domSlot]);
    }

    const float invKept = (keptSum > 1.0e-5f) ? (1.0f / keptSum) : 1.0f;
    std::array<float, kNumSnapshots> effWeights{};
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (isPatchSnapshotOccupied(slots[s]) && normWeights[s] > 1.0e-4f) {
            effWeights[s] = normWeights[s] * invKept;
        }
    }

    PatchFrame out{};
    out.circuitCoupling = 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        out.circuitCoupling += effWeights[s] * unquantizeNormQ16(slots[s].circuitCouplingQ16);
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        float normF = 0.0f, normHp = 0.0f, normLp = 0.0f, normRa = 0.0f, normRb = 0.0f;
        float normPw = 0.0f, drift = 0.0f, hpRes = 0.0f, lpRes = 0.0f, amp = 0.0f, normPan = 0.0f;
        float shA = 0.0f, dpA = 0.0f, shB = 0.0f, dpB = 0.0f;
        bool allMuted = true;

        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const float w = effWeights[s];
            if (w <= 0.0f) {
                continue;
            }
            const VoiceSnapshotBin& src = slots[s].voices[v];
            const bool vMuted = (src.flags & 1u) != 0u;
            normF += w * unquantizeNormQ16(src.freqLogQ16);
            normHp += w * unquantizeNormQ16(src.hpCutLogQ16);
            normLp += w * unquantizeNormQ16(src.lpCutLogQ16);
            normRa += w * unquantizeNormQ16(src.lfoRateLogQ16);
            normRb += w * unquantizeNormQ16(src.lfoBRateLogQ16);
            normPw += w * unquantizeNormQ16(src.pulseWidthQ16);
            drift += w * unquantizeNormQ16(src.driftQ16);
            hpRes += w * unquantizeNormQ16(src.hpResQ16);
            lpRes += w * unquantizeNormQ16(src.lpResQ16);
            amp += w * (vMuted ? 0.0f : unquantizeNormQ16(src.amplitudeQ16));
            normPan += w * unquantizeNormQ16(src.panBiQ16);
            shA += w * unquantizeNormQ8(src.lfoShapeQ8);
            dpA += w * unquantizeNormQ16(src.lfoDepthQ16);
            shB += w * unquantizeNormQ8(src.lfoBShapeQ8);
            dpB += w * unquantizeNormQ16(src.lfoBDepthQ16);
            if (!vMuted) {
                allMuted = false;
            }
        }

        const VoiceSnapshotBin& domSrc = slots[domSlot].voices[v];
        const uint8_t rawTA = domSrc.lfoTargetsPacked & 0x0Fu;
        const uint8_t rawTB = (domSrc.lfoTargetsPacked >> 4u) & 0x0Fu;

        VoiceParams& dst = out.voices[v];
        dst.frequencyHz =
            clampf(fastExp2(kFreqLogMin + normF * kFreqLogSpan), kAudioFreqMinHz, kAudioFreqMaxHz);
        dst.hpCutoffHz = clampf(fastExp2(kFltLogMin + normHp * kFltLogSpan), kFilterCutoffMinHz,
                                kFilterCutoffMaxHz);
        dst.lpCutoffHz = clampf(fastExp2(kFltLogMin + normLp * kFltLogSpan), kFilterCutoffMinHz,
                                kFilterCutoffMaxHz);
        dst.lfoRateHz = clampf(fastExp2(kLfoLogMinSnap + normRa * kLfoLogSpanSnap), kLfoFreqMinHz,
                               kLfoFreqMaxHz);
        dst.lfoBRateHz = clampf(fastExp2(kLfoLogMinSnap + normRb * kLfoLogSpanSnap), kLfoFreqMinHz,
                                kLfoFreqMaxHz);
        dst.pulseWidth = clampf(kPulseWidthMin + normPw * (kPulseWidthMax - kPulseWidthMin),
                                kPulseWidthMin, kPulseWidthMax);
        dst.drift = clampf(drift, 0.0f, 1.0f);
        dst.hpResonance = clampf(hpRes, 0.0f, 1.0f);
        dst.lpResonance = clampf(lpRes, 0.0f, 1.0f);
        dst.muted = allMuted;
        dst.amplitude = allMuted ? unquantizeNormQ16(domSrc.amplitudeQ16) : clampf(amp, 0.0f, 1.0f);
        dst.pan = clampf(2.0f * normPan - 1.0f, -1.0f, 1.0f);
        dst.lfoShape = clampf(shA, 0.0f, 1.0f);
        dst.lfoDepth = clampf(dpA, 0.0f, 1.0f);
        dst.lfoBShape = clampf(shB, 0.0f, 1.0f);
        dst.lfoBDepth = clampf(dpB, 0.0f, 1.0f);
        dst.lfoTarget =
            (rawTA < kNumLfoATargets) ? static_cast<LfoTarget>(rawTA) : LfoTarget::LpCutoff;
        dst.lfoBTarget =
            (rawTB < kNumLfoBTargets) ? static_cast<LfoTarget>(rawTB) : LfoTarget::LpCutoff;
    }
    return out;
}

} // namespace ruido
