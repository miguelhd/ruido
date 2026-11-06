#include "core/DspMath.h"
#include "core/SnapshotBank.h"

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
    if (slot == 0u) {
        // SNAP 1: Open Fifth Foundation (exact boot default)
        return s;
    }
    if (slot == 1u) {
        // SNAP 2: Formant Pulse Choir (narrow Schmitt PWM + vocal MS-20 peaks + slow LFO breath)
        constexpr std::array<float, 4> kFreq = {55.0f, 110.0f, 137.5f, 220.0f};
        constexpr std::array<float, 4> kDuty = {0.22f, 0.76f, 0.30f, 0.68f};
        constexpr std::array<float, 4> kHp = {65.0f, 180.0f, 320.0f, 440.0f};
        constexpr std::array<float, 4> kLp = {1400.0f, 2200.0f, 1850.0f, 3400.0f};
        s.global.circuitCoupling = 0.32f;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            s.voices[v].frequencyHz = kFreq[v];
            s.voices[v].pulseWidth = kDuty[v];
            s.voices[v].drift = 0.16f;
            s.voices[v].hpCutoffHz = kHp[v];
            s.voices[v].hpResonance = 0.42f;
            s.voices[v].lpCutoffHz = kLp[v];
            s.voices[v].lpResonance = 0.56f;
            s.voices[v].amplitude = 0.68f;
            s.voices[v].lfoTarget = LfoTarget::LpCutoff;
            s.voices[v].lfoRateHz = 0.14f + 0.05f * static_cast<float>(v);
            s.voices[v].lfoShape = 0.0f;
            s.voices[v].lfoDepth = 0.24f;
        }
        return s;
    }
    if (slot == 2u) {
        // SNAP 3: Sub-Harmonic Tectonic (deep sub-octave foundation + warm Korg35 low-pass drive)
        constexpr std::array<float, 4> kFreq = {27.5f, 41.20f, 55.0f, 82.41f};
        constexpr std::array<float, 4> kLp = {340.0f, 520.0f, 780.0f, 1250.0f};
        s.global.circuitCoupling = 0.38f;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            s.voices[v].frequencyHz = kFreq[v];
            s.voices[v].pulseWidth = (v & 1u) != 0u ? 0.42f : 0.50f;
            s.voices[v].drift = 0.22f;
            s.voices[v].hpCutoffHz = 24.0f;
            s.voices[v].hpResonance = 0.28f;
            s.voices[v].lpCutoffHz = kLp[v];
            s.voices[v].lpResonance = 0.48f;
            s.voices[v].amplitude = 0.80f;
            s.voices[v].lfoTarget = LfoTarget::PulseWidth;
            s.voices[v].lfoRateHz = 0.08f + 0.03f * static_cast<float>(v);
            s.voices[v].lfoShape = 1.0f;
            s.voices[v].lfoDepth = 0.30f;
        }
        return s;
    }
    // SNAP 4: Starved-Rail Swarm (CMOS sync tension + organic B->A modulation + MS-20 bite)
    constexpr std::array<float, 4> kFreq = {55.0f, 82.65f, 123.75f, 192.5f};
    constexpr std::array<float, 4> kDuty = {0.16f, 0.62f, 0.28f, 0.82f};
    s.global.circuitCoupling = 0.68f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        s.voices[v].frequencyHz = kFreq[v];
        s.voices[v].pulseWidth = kDuty[v];
        s.voices[v].drift = 0.35f;
        s.voices[v].hpCutoffHz = 48.0f + 45.0f * static_cast<float>(v);
        s.voices[v].hpResonance = 0.36f;
        s.voices[v].lpCutoffHz = 2600.0f + 1100.0f * static_cast<float>(v);
        s.voices[v].lpResonance = 0.64f;
        s.voices[v].amplitude = 0.84f;
        s.voices[v].lfoTarget = LfoTarget::LpCutoff;
        s.voices[v].lfoRateHz = 0.32f + 0.11f * static_cast<float>(v);
        s.voices[v].lfoShape = 0.25f;
        s.voices[v].lfoDepth = 0.32f;
        s.voices[v].lfoBTarget = LfoTarget::LfoARate;
        s.voices[v].lfoBRateHz = 0.09f + 0.04f * static_cast<float>(v);
        s.voices[v].lfoBShape = 1.0f;
        s.voices[v].lfoBDepth = 0.35f;
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

} // namespace ruido
