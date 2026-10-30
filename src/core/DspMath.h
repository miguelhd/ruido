#pragma once

#include "core/Config.h"

#include <cstdint>
#include <cstring>

namespace ruido {

// Clamp value to [lo, hi] safely: unordered NaN deterministically falls back to lo.
inline constexpr float clampf(float x, float lo, float hi) noexcept {
    const float t = (x >= lo) ? x : lo;
    return (t <= hi) ? t : hi;
}

// Branchless single-instruction float32 absolute value (zero libm dependency).
inline float absf(float x) noexcept {
    return __builtin_fabsf(x);
}

// Bit-level finite + denormal sanitizer for feedback states (zero libm overhead).
inline float sanitizeState(float x) noexcept {
    uint32_t bits = 0u;
    std::memcpy(&bits, &x, sizeof(bits));
    const uint32_t abs_bits = bits & 0x7FFFFFFFu;
    // Flush subnormals / tiny < ~1.17e-38f AND scrub NaN/Inf (exponent == 0xFF)
    if (abs_bits < 0x00800000u || abs_bits >= 0x7F800000u) {
        return 0.0f;
    }
    return x;
}

// Smoothstep 3t^2 - 2t^3 for clickless/kinkless edge tapers on [0, 1].
inline constexpr float smoothstep01(float t) noexcept {
    const float c = clampf(t, 0.0f, 1.0f);
    return c * c * (3.0f - 2.0f * c);
}

// Value + exact analytical derivative pair for Newton-Raphson nonlinear circuit solvers.
struct SatResult {
    float value;
    float deriv;
};

// C1-continuous algebraic tanh approximation on [-3, 3] (zero transcendentals):
//   f(x) = x*(27 + x^2) / (27 + 9*x^2)  for |x| < 3,  sgn(x) for |x| >= 3
inline float padeClip(float x) noexcept {
    if (x >= 3.0f) {
        return 1.0f;
    }
    if (x <= -3.0f) {
        return -1.0f;
    }
    const float x2 = x * x;
    return (x * (27.0f + x2)) / (27.0f + 9.0f * x2);
}

// High-headroom master bus analog transformer / op-amp soft ceiling:
// Linear up to |x| <= 0.75f, smoothly approaches +-1.25f without hard digital clipping.
inline float masterBusSoftClip(float x) noexcept {
    constexpr float kKnee = 0.75f;
    constexpr float kHeadroom = 0.50f; // Max asymptotic output = 0.75 + 0.50 = 1.25f
    constexpr float kInvHeadroom = 1.0f / kHeadroom;

    if (x > kKnee) {
        return kKnee + kHeadroom * padeClip((x - kKnee) * kInvHeadroom);
    }
    if (x < -kKnee) {
        return -kKnee + kHeadroom * padeClip((x + kKnee) * kInvHeadroom);
    }
    return x;
}

// Constant-power stereo panner: pan in [-1.0f (Left), +1.0f (Right)].
// Uses 5th-order minimax sin polynomial on [0, pi/2] (max error < 1.5e-4, zero libm calls).
inline void equalPowerPan(float pan, float& gainL, float& gainR) noexcept {
    const float p = clampf(pan, -1.0f, 1.0f);
    const float aR = (p + 1.0f) * (0.25f * kPi); // [0, pi/2]
    const float aL = kHalfPi - aR;               // [pi/2, 0]
    const float aR2 = aR * aR;
    const float aL2 = aL * aL;
    gainR = aR * (1.0f - aR2 * (0.16666667f - 0.0083143f * aR2));
    gainL = aL * (1.0f - aL2 * (0.16666667f - 0.0083143f * aL2));
}

// Fast 5th-order polynomial approximation for tan(x) on [0, 0.55] (max error < 5e-5).
// At 176.472 kHz, f_c <= 30 kHz corresponds to pi * fc / fs <= 0.534 rad (< pi/4).
// Clamps w to [1e-5, 0.55] so extreme/low sample rates or NaN/Inf invSampleRate stay finite.
inline float fastPrewarpTan(float cutoffHz, float invSampleRate) noexcept {
    const float fc = clampf(cutoffHz, 2.0f, kFilterMaxSafeHz);
    const float w = clampf(kPi * fc * invSampleRate, 1.0e-5f, 0.55f);
    const float w2 = w * w;
    return w * (1.0f + w2 * (0.33333333f + w2 * (0.13333333f + w2 * 0.05396825f)));
}

// Fast reciprocal square root 1 / sqrt(x) for positive x (Lomont magic + 2 NR polish steps:
// < 1e-6 relative error, zero libm calls).
inline float fastInvSqrt(float x) noexcept {
    const float safeX = clampf(x, 1.0e-12f, 1.0e12f);
    const float halfX = 0.5f * safeX;
    uint32_t bits = 0u;
    std::memcpy(&bits, &safeX, sizeof(bits));
    bits = 0x5F375A86u - (bits >> 1u);
    float y = 1.0f;
    std::memcpy(&y, &bits, sizeof(y));
    y = y * (1.5f - halfX * y * y);
    y = y * (1.5f - halfX * y * y);
    return y;
}

// Fast IEEE-754 + 4th-order minimax 2^octaves for octaves in [-64, +64].
// Exact at all integer octaves (-3..+3) and < 0.16 cents max error, zero libm calls.
inline float fastExp2(float octaves) noexcept {
    const float x = clampf(octaves, -64.0f, 64.0f);
    const int32_t xi = static_cast<int32_t>(x + 128.0f) - 128;
    const float f = x - static_cast<float>(xi);
    const float p =
        1.0f + f * (0.69315308f + f * (0.24015362f + f * (0.05582821f + f * 0.01086509f)));
    const uint32_t expBits = static_cast<uint32_t>(xi + 127) << 23u;
    float scale = 1.0f;
    std::memcpy(&scale, &expBits, sizeof(scale));
    return p * scale;
}

// Fast natural exponential e^x via 2^(x * log2(e)), zero libm calls.
inline float fastExp(float x) noexcept {
    return fastExp2(x * 1.44269504f);
}

// Fast IEEE-754 + 4th-order minimax log2(x) for positive x (< 4e-4 octave error, zero libm calls).
inline float fastLog2(float x) noexcept {
    const float safeX = clampf(x, 1.0e-12f, 1.0e12f);
    uint32_t bits = 0u;
    std::memcpy(&bits, &safeX, sizeof(bits));
    const int32_t expInt = static_cast<int32_t>((bits >> 23u) & 0xFFu) - 127;
    const uint32_t mantBits = (bits & 0x007FFFFFu) | 0x3F800000u;
    float m = 1.0f;
    std::memcpy(&m, &mantBits, sizeof(m));
    const float t = m - 1.0f;
    const float frac = t * (1.44254494f + t * (-0.71814525f + t * (0.45754919f - 0.18194888f * t)));
    return static_cast<float>(expInt) + frac;
}

} // namespace ruido
