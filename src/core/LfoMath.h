#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/Parameters.h"

#include <array>
#include <cstdint>

namespace ruido {

// Pack 5 signed 6-bit drift knots in [-31, +31] into a single 32-bit atomic word (0 -> {0,0,0,0,0})
inline uint32_t packDriftKnots(const std::array<int8_t, 5>& knots) noexcept {
    uint32_t packed = 0u;
    for (size_t i = 0u; i < 5u; ++i) {
        int32_t q = static_cast<int32_t>(knots[i]);
        if (q < -31) {
            q = -31;
        } else if (q > 31) {
            q = 31;
        }
        const uint32_t u6 = static_cast<uint32_t>(q) & 0x3Fu;
        packed |= (u6 << static_cast<uint32_t>(i * 6u));
    }
    return packed;
}

inline std::array<int8_t, 5> unpackDriftKnots(uint32_t packed) noexcept {
    std::array<int8_t, 5> knots{0, 0, 0, 0, 0};
    for (size_t i = 0u; i < 5u; ++i) {
        const uint32_t u6 = (packed >> static_cast<uint32_t>(i * 6u)) & 0x3Fu;
        int32_t s = (u6 >= 32u) ? (static_cast<int32_t>(u6) - 64) : static_cast<int32_t>(u6);
        if (s < -31) {
            s = -31;
        }
        knots[i] = static_cast<int8_t>(s);
    }
    return knots;
}

inline float wrapLfoPhase01(float phase) noexcept {
    float w = sanitizeState(phase);
    w -= static_cast<float>(static_cast<int32_t>(w));
    if (w < 0.0f) {
        w += 1.0f;
    }
    if (w >= 1.0f) {
        w = 0.0f;
    }
    return w;
}

// 0.00 Landmark: C^2 folded polynomial sine + JFET differential-pair crest warmth (0 DC, +-1 peak).
inline float evalAnalogSine(float phase01) noexcept {
    float q = phase01 + 0.25f;
    if (q >= 1.0f) {
        q -= 1.0f;
    }
    const float u = 1.0f - 4.0f * absf(q - 0.5f);
    const float u2 = u * u;
    // 7th-order odd polynomial matching sin(pi/2 * u) with exact f(+-1) = +-1 and f'(+-1) = 0
    const float s = u * (1.57079633f + u2 * (-0.64596410f + u2 * (0.07953921f - 0.00437144f * u2)));
    constexpr float kDrive = 0.55f;
    constexpr float kNorm = (27.0f + 9.0f * kDrive * kDrive) / (kDrive * (27.0f + kDrive * kDrive));
    return clampf(padeClip(kDrive * s) * kNorm, -1.0f, 1.0f);
}

// 0.25 -> 0.50 Landmarks: MS-20 variable-symmetry Triangle -> Sawtooth with C^1 quadratic
// vertex fillets, rate-adaptive capacitor flyback across 0.01 Hz .. 2 kHz, +-1.0 peak, 0.0 DC.
inline float evalSkewedTriSaw(float phase01, float skew01, float rateHz) noexcept {
    const float s = clampf(skew01, 0.0f, 1.0f);
    const float rClamped = clampf(rateHz, 0.0f, kLfoFreqMaxHz);
    const float lowSlew = clampf(rClamped * (1.0f / 40.0f), 0.0f, 1.0f);
    const float audioSlew = smoothstep01((rClamped - 40.0f) * (1.0f / 1960.0f));
    const float minFall = clampf(0.012f + 0.128f * lowSlew + 0.280f * audioSlew, 0.012f, 0.42f);
    const float r = 0.5f + s * ((1.0f - minFall) - 0.5f);
    const float halfR = 0.5f * r;
    const float halfF = 0.5f - halfR;

    const float sign = (phase01 <= 0.5f) ? 1.0f : -1.0f;
    const float q = (phase01 <= 0.5f) ? phase01 : (1.0f - phase01);

    const float a = q / halfR;
    const float b = (0.5f - q) / halfF;
    constexpr float kCorner = 0.08f;
    const float h = clampf(0.5f + 0.5f * (b - a) * (1.0f / kCorner), 0.0f, 1.0f);
    const float arch = b * (1.0f - h) + a * h - kCorner * h * (1.0f - h);
    const float invPeak = 1.0f / (1.0f - kCorner * r * (1.0f - r));
    return clampf(sign * arch * invPeak, -1.0f, 1.0f);
}

// 0.75 Landmark: Vactrol-slewed Schmitt square wave (C^1 cubic edges across 0.01 Hz .. 2 kHz, 0 DC)
inline float evalAnalogSquare(float phase01, float rateHz) noexcept {
    const float rClamped = clampf(rateHz, 0.0f, kLfoFreqMaxHz);
    const float lowSlew = clampf(rClamped * (1.0f / 40.0f), 0.0f, 1.0f);
    const float audioSlew = smoothstep01((rClamped - 40.0f) * (1.0f / 1960.0f));
    const float halfEdge = clampf(0.008f + 0.088f * lowSlew + 0.144f * audioSlew, 0.008f, 0.24f);
    const float sign = (phase01 <= 0.5f) ? 1.0f : -1.0f;
    const float q = (phase01 <= 0.5f) ? phase01 : (1.0f - phase01);

    float mag = 1.0f;
    if (q < halfEdge) {
        const float u = q / halfEdge;
        mag = u * (1.5f - 0.5f * u * u);
    } else if ((0.5f - q) < halfEdge) {
        const float u = (0.5f - q) / halfEdge;
        mag = u * (1.5f - 0.5f * u * u);
    }
    return sign * mag;
}

// 1.00 Landmark: 2-segment Uniform Cubic B-Spline non-repeating organic drift (C^2 continuous).
inline float evalCubicBSplineDrift(float phase01, const std::array<int8_t, 5>& knots) noexcept {
    constexpr float kInv31 = 1.0f / 31.0f;
    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float c3 = 0.0f;
    float t = 0.0f;

    if (phase01 < 0.5f) {
        t = phase01 * 2.0f;
        c0 = static_cast<float>(knots[0]) * kInv31;
        c1 = static_cast<float>(knots[1]) * kInv31;
        c2 = static_cast<float>(knots[2]) * kInv31;
        c3 = static_cast<float>(knots[3]) * kInv31;
    } else {
        t = phase01 * 2.0f - 1.0f;
        c0 = static_cast<float>(knots[1]) * kInv31;
        c1 = static_cast<float>(knots[2]) * kInv31;
        c2 = static_cast<float>(knots[3]) * kInv31;
        c3 = static_cast<float>(knots[4]) * kInv31;
    }

    const float omt = 1.0f - t;
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float w0 = omt * omt * omt;
    const float w1 = 3.0f * t3 - 6.0f * t2 + 4.0f;
    const float w2 = -3.0f * t3 + 3.0f * t2 + 3.0f * t + 1.0f;
    const float w3 = t3;
    const float raw = (w0 * c0 + w1 * c1 + w2 * c2 + w3 * c3) * (1.0f / 6.0f);

    constexpr float kGain = 1.65f;
    constexpr float kNorm = (27.0f + 9.0f * kGain * kGain) / (kGain * (27.0f + kGain * kGain));
    return clampf(padeClip(kGain * raw) * kNorm, -1.0f, 1.0f);
}

// Continuous stepless waveform morph across [0.0, 1.0]:
//   0.00 Sine -> 0.25 Triangle -> 0.50 Sawtooth -> 0.75 Slewed Square -> 1.00 Smooth Drift
inline float evalMorphedLfoWave(float phase01, float shape01, float rateHz,
                                const std::array<int8_t, 5>& knots) noexcept {
    const float w = wrapLfoPhase01(phase01);
    const float s = clampf(shape01, 0.0f, 1.0f);

    if (s <= 0.25f) {
        const float t = s * 4.0f;
        if (t <= 1.0e-5f) {
            return evalAnalogSine(w);
        }
        return (1.0f - t) * evalAnalogSine(w) + t * evalSkewedTriSaw(w, 0.0f, rateHz);
    }
    if (s <= 0.50f) {
        return evalSkewedTriSaw(w, (s - 0.25f) * 4.0f, rateHz);
    }
    if (s <= 0.75f) {
        const float t = (s - 0.50f) * 4.0f;
        return (1.0f - t) * evalSkewedTriSaw(w, 1.0f, rateHz) + t * evalAnalogSquare(w, rateHz);
    }
    const float t = (s - 0.75f) * 4.0f;
    if (t >= 0.99999f) {
        return evalCubicBSplineDrift(w, knots);
    }
    return (1.0f - t) * evalAnalogSquare(w, rateHz) + t * evalCubicBSplineDrift(w, knots);
}

// Headroom-aware MS-20 filter cutoff modulation (shared by LPF and HPF).
// Shifts center smoothly inward when parked near 2 Hz or 28 kHz, and slams full rail-to-rail
// (+-7.5 octaves / 15-octave span) at extreme depth.
inline float applyLfoCutoffHz(float baseHz, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseHz;
    }
    constexpr float kOctMin = cxLog2Pos(kFilterCutoffMinHz);
    constexpr float kOctMax = cxLog2Pos(kFilterCutoffMaxHz);
    constexpr float kHalfRange = 0.5f * (kOctMax - kOctMin);
    const float halfSpan = 7.5f * d * (0.25f + 0.75f * d);
    const float guard = (halfSpan < kHalfRange) ? halfSpan : kHalfRange;
    const float baseOct = fastLog2(clampf(baseHz, kFilterCutoffMinHz, kFilterCutoffMaxHz));
    const float centerOct = clampf(baseOct, kOctMin + guard, kOctMax - guard);
    const float modOct = centerOct + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f);
    return clampf(fastExp2(modOct), kFilterCutoffMinHz, kFilterCutoffMaxHz);
}

// Uncapped MS-20 Rev 2 / Korg35 resonance modulation across full [0.0, 1.0] self-oscillation span.
inline float applyLfoResonance(float baseRes, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseRes;
    }
    const float cleanBase = clampf(baseRes, 0.0f, 1.0f);
    const float halfSpan = 0.54f * d;
    const float guard = (halfSpan < 0.5f) ? halfSpan : 0.5f;
    const float center = clampf(cleanBase, guard, 1.0f - guard);
    return clampf(center + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f), 0.0f, 1.0f);
}

// Concave audio-taper VCA / overdrive modulation (0.0 silence trough -> 1.0 full 4.5x drive crest).
inline float applyLfoAmplitude(float baseAmp, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseAmp;
    }
    const float cleanBase = clampf(baseAmp, 0.0f, 1.0f);
    const float ampMin = cleanBase * (1.0f - d);
    const float ampMax = clampf(cleanBase + (1.0f - cleanBase) * d, 0.0f, 1.0f);
    const float u = 0.5f * (clampf(lfoBipolar, -1.0f, 1.0f) + 1.0f);
    const float shapedU = u * (0.65f + 0.35f * u);
    return clampf(ampMin + (ampMax - ampMin) * shapedU, 0.0f, 1.0f);
}

// Concave log-octave pitch vibrato -> +-5-octave audio-rate FM / dive-bomb sweep.
inline float applyLfoPitchHz(float baseHz, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseHz;
    }
    const float octSpan = 5.0f * d * (0.10f + 0.90f * d);
    return clampf(baseHz * fastExp2(octSpan * clampf(lfoBipolar, -1.0f, 1.0f)),
                  kAudioFreqMinHz * 0.5f, kAudioFreqMaxHz);
}

// Headroom-aware constant-power stereo panner orbit across [-1.0, +1.0].
inline float applyLfoPan(float basePan, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return basePan;
    }
    const float cleanBase = clampf(basePan, -1.0f, 1.0f);
    const float halfSpan = d;
    const float center = clampf(cleanBase, -1.0f + halfSpan, 1.0f - halfSpan);
    return clampf(center + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f), -1.0f, 1.0f);
}

// Headroom-aware band-limited Schmitt PWM sweep across [kPulseWidthMin, kPulseWidthMax] (5%-95%).
inline float applyLfoPulseWidth(float basePw, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return basePw;
    }
    constexpr float kHalfMax = 0.5f * (kPulseWidthMax - kPulseWidthMin);
    const float cleanBase = clampf(basePw, kPulseWidthMin, kPulseWidthMax);
    const float halfSpan = kHalfMax * d;
    const float center = clampf(cleanBase, kPulseWidthMin + halfSpan, kPulseWidthMax - halfSpan);
    return clampf(center + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f), kPulseWidthMin,
                  kPulseWidthMax);
}

// LFO B -> LFO A Rate cross-modulation (+-6 octaves speed warp across [0.01 Hz, 2,000 Hz]).
inline float applyLfoModRateHz(float baseRateHz, float modDepth01, float lfoBipolar) noexcept {
    const float d = clampf(modDepth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseRateHz;
    }
    const float octSpan = 6.0f * d * (0.20f + 0.80f * d);
    return clampf(baseRateHz * fastExp2(octSpan * clampf(lfoBipolar, -1.0f, 1.0f)), kLfoFreqMinHz,
                  kLfoFreqMaxHz);
}

// LFO B -> LFO A Depth cross-modulation (swells LFO A strength from stillness up to crest).
inline float applyLfoModDepth(float baseDepth, float modDepth01, float lfoBipolar) noexcept {
    const float base = clampf(baseDepth, 0.0f, 1.0f);
    const float d = clampf(modDepth01, 0.0f, 1.0f);
    if (d <= 0.0f || base <= 0.0f) {
        return base;
    }
    const float dMin = base * (1.0f - d);
    const float dMax = clampf(base + (1.0f - base) * 0.35f * d, 0.0f, 1.0f);
    const float u = 0.5f * (clampf(lfoBipolar, -1.0f, 1.0f) + 1.0f);
    return clampf(dMin + (dMax - dMin) * u, 0.0f, 1.0f);
}

inline float computeLiveLfoADepth(const VoiceParams& vp, float lfoBRaw) noexcept {
    return (vp.lfoBTarget == LfoTarget::LfoADepth)
               ? applyLfoModDepth(vp.lfoDepth, vp.lfoBDepth, lfoBRaw)
               : vp.lfoDepth;
}

inline float computeLiveLfoARateHz(const VoiceParams& vp, float lfoBRaw) noexcept {
    return (vp.lfoBTarget == LfoTarget::LfoARate)
               ? applyLfoModRateHz(vp.lfoRateHz, vp.lfoBDepth, lfoBRaw)
               : vp.lfoRateHz;
}

inline float computeLiveVoicePitchHz(const VoiceParams& vp, float lfoARaw, float lfoBRaw) noexcept {
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float hz = (vp.lfoTarget == LfoTarget::Pitch) ? applyLfoPitchHz(vp.frequencyHz, dA, lfoARaw)
                                                  : vp.frequencyHz;
    if (vp.lfoBTarget == LfoTarget::Pitch) {
        hz = applyLfoPitchHz(hz, vp.lfoBDepth, lfoBRaw);
    }
    return hz;
}

inline float computeLiveVoicePulseWidth(const VoiceParams& vp, float lfoARaw, float lfoBRaw,
                                        float keyPwDelta = 0.0f) noexcept {
    const float basePw = clampf(vp.pulseWidth + keyPwDelta, kPulseWidthMin, kPulseWidthMax);
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float pw =
        (vp.lfoTarget == LfoTarget::PulseWidth) ? applyLfoPulseWidth(basePw, dA, lfoARaw) : basePw;
    if (vp.lfoBTarget == LfoTarget::PulseWidth) {
        pw = applyLfoPulseWidth(pw, vp.lfoBDepth, lfoBRaw);
    }
    return pw;
}

inline constexpr std::array<float, kNumOscillators> kKeyWideFanPans = {-1.0f, -0.52f, 0.52f, 1.0f};

inline float computeLiveVoiceAmplitude(const VoiceParams& vp, float lfoARaw, float lfoBRaw,
                                       float overloadSurge = 0.0f) noexcept {
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float amp = (vp.lfoTarget == LfoTarget::Amplitude)
                    ? applyLfoAmplitude(vp.amplitude, dA, lfoARaw)
                    : vp.amplitude;
    if (vp.lfoBTarget == LfoTarget::Amplitude) {
        amp = applyLfoAmplitude(amp, vp.lfoBDepth, lfoBRaw);
    }
    if (overloadSurge > 1.0e-4f) {
        const float s = clampf(overloadSurge, 0.0f, 1.0f);
        const float sCurve = s * (0.62f + 0.38f * s);
        amp = clampf(amp + (1.0f - amp) * sCurve, 0.0f, 1.0f);
    }
    return amp;
}

inline float computeLiveVoicePan(const VoiceParams& vp, float lfoARaw, float lfoBRaw) noexcept {
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float pan = (vp.lfoTarget == LfoTarget::Pan) ? applyLfoPan(vp.pan, dA, lfoARaw) : vp.pan;
    if (vp.lfoBTarget == LfoTarget::Pan) {
        pan = applyLfoPan(pan, vp.lfoBDepth, lfoBRaw);
    }
    return pan;
}

inline float computeEffectivePanScale(bool monoCollapse, float spatialFocus) noexcept {
    const float baseScale = monoCollapse ? 0.0f : 1.0f;
    const float sf = clampf(spatialFocus, -1.0f, 1.0f);
    return (sf < 0.0f) ? (baseScale * (1.0f + sf)) : (baseScale + (1.0f - baseScale) * sf);
}

inline float applySpatialFocusPan(float pan, size_t voiceIdx, bool monoCollapse,
                                  float spatialFocus) noexcept {
    const float sf = clampf(spatialFocus, -1.0f, 1.0f);
    float p = pan;
    if (sf > 0.0f) {
        p += (kKeyWideFanPans[voiceIdx & 3u] - p) * sf;
    }
    return clampf(p * computeEffectivePanScale(monoCollapse, sf), -1.0f, 1.0f);
}

inline ModulatedFilterParams applyKeyDubLowPass(ModulatedFilterParams mod, float dubLpfDrop,
                                                float overloadSurge = 0.0f,
                                                float filterScream = 0.0f) noexcept {
    const float surge = clampf(overloadSurge, 0.0f, 1.0f);
    if (surge > 0.22f) {
        const float u = (surge - 0.22f) * (1.0f / 0.78f);
        const float driveCeilHz = 28000.0f * fastExp2(-3.05f * u); // 28 kHz -> 3.38 kHz OTA scream
        if (driveCeilHz < mod.lpCutoffHz) {
            mod.lpCutoffHz = clampf(driveCeilHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        mod.lpResonance = clampf(mod.lpResonance + 0.54f * u * u, 0.0f, 0.90f);
    }
    const float dubDrop = clampf(dubLpfDrop, 0.0f, 1.0f);
    if (dubDrop > 1.0e-4f) {
        const float dubCeilHz = 28000.0f * fastExp2(-9.8f * dubDrop); // 28 kHz -> 31.4 Hz
        if (dubCeilHz < mod.lpCutoffHz) {
            mod.lpCutoffHz = clampf(dubCeilHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        const float dubResBump = 0.76f * dubDrop * (0.55f + 0.45f * dubDrop);
        mod.lpResonance = clampf(mod.lpResonance + dubResBump, 0.0f, 0.92f);
    }
    const float scream = clampf(filterScream, 0.0f, 1.0f);
    if (scream > 1.0e-4f) {
        const float engage = smoothstep01(scream * 5.0f);
        const float screamHpHz = 55.0f * fastExp2(2.90f * scream);     // 55 Hz -> 410 Hz
        const float screamLpHz = 14000.0f * fastExp2(-3.95f * scream); // 14 kHz -> 905 Hz
        if (screamHpHz > mod.hpCutoffHz) {
            mod.hpCutoffHz =
                clampf(mod.hpCutoffHz * fastExp2(engage * fastLog2(screamHpHz / mod.hpCutoffHz)),
                       kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        if (screamLpHz < mod.lpCutoffHz) {
            mod.lpCutoffHz =
                clampf(mod.lpCutoffHz * fastExp2(engage * fastLog2(screamLpHz / mod.lpCutoffHz)),
                       kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        const float screamRes = 0.96f * scream * (0.68f + 0.32f * scream);
        if (screamRes > mod.hpResonance) {
            mod.hpResonance = screamRes;
        }
        if (screamRes > mod.lpResonance) {
            mod.lpResonance = screamRes;
        }
    }
    return mod;
}

inline ModulatedFilterParams computeLiveVoiceFilter(const VoiceParams& vp, float pressure,
                                                    float pitchMode, float lfoARaw,
                                                    float lfoBRaw = 0.0f, float dubLpfDrop = 0.0f,
                                                    float overloadSurge = 0.0f,
                                                    float filterScream = 0.0f) noexcept {
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float hpBase = (vp.lfoTarget == LfoTarget::HpCutoff)
                       ? applyLfoCutoffHz(vp.hpCutoffHz, dA, lfoARaw)
                       : vp.hpCutoffHz;
    if (vp.lfoBTarget == LfoTarget::HpCutoff) {
        hpBase = applyLfoCutoffHz(hpBase, vp.lfoBDepth, lfoBRaw);
    }
    float hpResBase = (vp.lfoTarget == LfoTarget::HpResonance)
                          ? applyLfoResonance(vp.hpResonance, dA, lfoARaw)
                          : vp.hpResonance;
    if (vp.lfoBTarget == LfoTarget::HpResonance) {
        hpResBase = applyLfoResonance(hpResBase, vp.lfoBDepth, lfoBRaw);
    }
    float lpBase = (vp.lfoTarget == LfoTarget::LpCutoff)
                       ? applyLfoCutoffHz(vp.lpCutoffHz, dA, lfoARaw)
                       : vp.lpCutoffHz;
    if (vp.lfoBTarget == LfoTarget::LpCutoff) {
        lpBase = applyLfoCutoffHz(lpBase, vp.lfoBDepth, lfoBRaw);
    }
    float lpResBase = (vp.lfoTarget == LfoTarget::LpResonance)
                          ? applyLfoResonance(vp.lpResonance, dA, lfoARaw)
                          : vp.lpResonance;
    if (vp.lfoBTarget == LfoTarget::LpResonance) {
        lpResBase = applyLfoResonance(lpResBase, vp.lfoBDepth, lfoBRaw);
    }
    return applyKeyDubLowPass(computeVoicePressureMacro(hpBase, hpResBase, lpBase, lpResBase,
                                                        pressure, vp.muted ? 0.0f : 1.0f,
                                                        pitchMode),
                              dubLpfDrop, overloadSurge, filterScream);
}

inline constexpr bool isFilterLfoTarget(LfoTarget t) noexcept {
    return t == LfoTarget::LpCutoff || t == LfoTarget::LpResonance || t == LfoTarget::HpCutoff ||
           t == LfoTarget::HpResonance;
}

inline bool isVoiceFilterActive(const VoiceParams& vp, float pressure, float pitchMode) noexcept {
    if (vp.hpCutoffHz > 2.05f || vp.lpCutoffHz < 27900.0f || vp.hpResonance > 0.002f ||
        vp.lpResonance > 0.002f) {
        return true;
    }
    if ((vp.lfoDepth > 0.002f && isFilterLfoTarget(vp.lfoTarget)) ||
        (vp.lfoBDepth > 0.002f && isFilterLfoTarget(vp.lfoBTarget))) {
        return true;
    }
    const float fp = clampf(pressure, 0.0f, 1.0f) * (1.0f - clampf(pitchMode, 0.0f, 1.0f));
    return fp > 0.002f;
}

inline float computeVoiceFilterEngagement(const VoiceParams& vp, float pressure, float pitchMode,
                                          float lfoARaw, float lfoBRaw = 0.0f) noexcept {
    const ModulatedFilterParams flt =
        computeLiveVoiceFilter(vp, pressure, pitchMode, lfoARaw, lfoBRaw);
    constexpr float kHpMinOct = cxLog2Pos(kFilterCutoffMinHz);
    constexpr float kLpMaxOct = cxLog2Pos(kFilterCutoffMaxHz);
    const float hpEng =
        (flt.hpCutoffHz > 2.05f)
            ? clampf((fastLog2(flt.hpCutoffHz) - kHpMinOct) * (1.0f / 11.0f), 0.0f, 1.0f)
            : 0.0f;
    const float lpEng =
        (flt.lpCutoffHz < 27900.0f)
            ? clampf((kLpMaxOct - fastLog2(flt.lpCutoffHz)) * (1.0f / 11.0f), 0.0f, 1.0f)
            : 0.0f;
    const float cutEng = (hpEng > lpEng) ? hpEng : lpEng;
    const float resEng =
        clampf((flt.hpResonance > flt.lpResonance) ? flt.hpResonance : flt.lpResonance, 0.0f, 1.0f);
    const float fltEng = clampf(
        (cutEng > resEng) ? (cutEng + 0.35f * resEng) : (resEng + 0.35f * cutEng), 0.0f, 1.0f);
    if (fltEng > 0.02f || !isVoiceFilterActive(vp, pressure, pitchMode)) {
        return fltEng;
    }
    return 0.02f;
}

inline constexpr float kSubAudioLoOct = cxLog2Pos(15.0f);
inline constexpr float kInvSubAudioSpanOct = 1.0f / (cxLog2Pos(25.0f) - cxLog2Pos(15.0f));
inline constexpr float kSilentAmpThreshold = 1.0e-4f;

inline constexpr const char* lfoTargetName(LfoTarget target) noexcept {
    switch (target) {
    case LfoTarget::Pitch:
        return "FREQ";
    case LfoTarget::PulseWidth:
        return "DUTY";
    case LfoTarget::LpCutoff:
        return "LPF";
    case LfoTarget::LpResonance:
        return "LP RES";
    case LfoTarget::HpCutoff:
        return "HPF";
    case LfoTarget::HpResonance:
        return "HP RES";
    case LfoTarget::Amplitude:
        return "LEVEL";
    case LfoTarget::Pan:
        return "PAN";
    case LfoTarget::LfoARate:
        return "A RATE";
    case LfoTarget::LfoADepth:
        return "A DEPTH";
    case LfoTarget::Count:
        break;
    }
    return "LPF";
}

inline constexpr const char* lfoShapeLabel(float shape01) noexcept {
    if (shape01 < 0.04f) {
        return "SINE";
    }
    if (shape01 < 0.21f) {
        return "SIN-TRI";
    }
    if (shape01 <= 0.29f) {
        return "TRI";
    }
    if (shape01 < 0.46f) {
        return "TRI-SAW";
    }
    if (shape01 <= 0.54f) {
        return "SAW";
    }
    if (shape01 < 0.71f) {
        return "SAW-SQR";
    }
    if (shape01 <= 0.79f) {
        return "SQUARE";
    }
    if (shape01 < 0.96f) {
        return "SQR-RND";
    }
    return "RANDOM";
}

// CIRCUIT LAB shared acoustic curves (consumed 1-to-1 by Core 1 DSP and Core 0 HUD readouts):
//  - FOLD (Keys 9-12): 1.0x -> 6.2x West-Coast Serge wavefolder stages on CD40106 capacitor ramp
//  - CRUSH (Keys 13-16): 16.0 kHz -> 165 Hz CD4013 sample-and-hold decimation clock
inline float computeWavefoldStages(float foldDepth01) noexcept {
    const float d = clampf(foldDepth01, 0.0f, 1.0f);
    return 1.0f + 5.2f * d * (0.30f + 0.70f * d);
}

inline float computeCrushClockHz(float crushDepth01) noexcept {
    return 16000.0f * fastExp2(-6.6f * clampf(crushDepth01, 0.0f, 1.0f));
}

} // namespace ruido
