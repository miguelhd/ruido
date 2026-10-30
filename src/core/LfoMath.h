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
    const float s =
        u * (1.57079633f + u2 * (-0.64596410f + u2 * (0.07953921f - 0.00437144f * u2)));
    constexpr float kDrive = 0.55f;
    constexpr float kNorm =
        (27.0f + 9.0f * kDrive * kDrive) / (kDrive * (27.0f + kDrive * kDrive));
    return clampf(padeClip(kDrive * s) * kNorm, -1.0f, 1.0f);
}

// 0.25 -> 0.50 Landmarks: MS-20 variable-symmetry Triangle -> Sawtooth with C^1 quadratic
// vertex fillets, rate-adaptive capacitor flyback, exact +-1.0 peak, and exact 0.0 DC integral.
inline float evalSkewedTriSaw(float phase01, float skew01, float rateHz) noexcept {
    const float s = clampf(skew01, 0.0f, 1.0f);
    const float minFall = clampf(0.012f + clampf(rateHz, 0.0f, 40.0f) * 0.0032f, 0.012f, 0.18f);
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

// 0.75 Landmark: Vactrol-slewed Schmitt square wave (C^1 cubic edges, crisp flat plateaus, 0 DC).
inline float evalAnalogSquare(float phase01, float rateHz) noexcept {
    const float halfEdge = clampf(0.008f + clampf(rateHz, 0.0f, 40.0f) * 0.0022f, 0.008f, 0.12f);
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
// Shifts center smoothly inward when parked near 20 Hz or 20 kHz so sweeps never flat-top.
inline float applyLfoCutoffHz(float baseHz, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseHz;
    }
    constexpr float kOctMin = 4.32192809f;  // log2(20 Hz)
    constexpr float kOctMax = 14.28771238f; // log2(20000 Hz)
    const float halfSpan = 4.5f * d * (0.30f + 0.70f * d);
    const float baseOct = fastLog2(clampf(baseHz, kFilterCutoffMinHz, kFilterCutoffMaxHz));
    const float centerOct = clampf(baseOct, kOctMin + halfSpan, kOctMax - halfSpan);
    const float modOct = centerOct + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f);
    return clampf(fastExp2(modOct), kFilterCutoffMinHz, kFilterCutoffMaxHz);
}

// Headroom-aware MS-20 Rev 2 OTA resonance modulation (shared by LP RES and HP RES).
inline float applyLfoResonance(float baseRes, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseRes;
    }
    const float cleanBase = clampf(baseRes, 0.0f, 1.0f);
    const float resCeil = (cleanBase > 0.96f) ? cleanBase : 0.96f;
    const float halfSpan = 0.47f * d;
    const float center = clampf(cleanBase, halfSpan, resCeil - halfSpan);
    return clampf(center + halfSpan * clampf(lfoBipolar, -1.0f, 1.0f), 0.0f, 1.0f);
}

// Concave audio-taper VCA tremolo modulation (drops to 0.0 at 100% depth trough).
inline float applyLfoAmplitude(float baseAmp, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseAmp;
    }
    const float cleanBase = clampf(baseAmp, 0.0f, 1.0f);
    const float ampMin = cleanBase * (1.0f - d);
    const float ampMax = clampf(cleanBase * (1.0f + 0.45f * d), 0.0f, 1.0f);
    const float u = 0.5f * (clampf(lfoBipolar, -1.0f, 1.0f) + 1.0f);
    const float shapedU = u * (0.65f + 0.35f * u);
    return clampf(ampMin + (ampMax - ampMin) * shapedU, 0.0f, 1.0f);
}

// Concave log-octave pitch vibrato -> +-2-octave FM sweep.
inline float applyLfoPitchHz(float baseHz, float depth01, float lfoBipolar) noexcept {
    const float d = clampf(depth01, 0.0f, 1.0f);
    if (d <= 0.0f) {
        return baseHz;
    }
    const float octSpan = 2.0f * d * (0.12f + 0.88f * d);
    return clampf(baseHz * fastExp2(octSpan * clampf(lfoBipolar, -1.0f, 1.0f)), kLfoFreqMinHz,
                  kAudioFreqMaxHz * 4.0f);
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

inline constexpr const char* lfoTargetName(LfoTarget target) noexcept {
    switch (target) {
    case LfoTarget::LpCutoff:
        return "LPF";
    case LfoTarget::LpResonance:
        return "LP RES";
    case LfoTarget::HpCutoff:
        return "HPF";
    case LfoTarget::HpResonance:
        return "HP RES";
    case LfoTarget::Amplitude:
        return "AMP";
    case LfoTarget::Pitch:
        return "PITCH";
    case LfoTarget::Pan:
        return "PAN";
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
        return "SQR-DRF";
    }
    return "DRIFT";
}

} // namespace ruido
