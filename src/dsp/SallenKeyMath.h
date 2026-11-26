#pragma once

#include "core/Config.h"
#include "core/DspMath.h"

namespace ruido {

// Exact Butterworth maximally-flat Sallen-Key gain: k = 2 - sqrt(2) => Q = 0.70710678
inline constexpr float kMs20ButterworthK = 2.0f - kSqrt2; // 0.58578644f
// Two-regime MS-20 resonance ceiling:
//  - [0.00, 0.60]: creamy KLM-307 Rev 2 LM13600 OTA vocal Q & singing sine limit cycle (k <= 3.60)
//  - (0.60, 1.00]: feral Korg35 Rev 1 silicon-clamped screaming instability (k up to 6.80)
inline constexpr float kMs20Rev2MaxK = 3.60f;
inline constexpr float kMs20Korg35MaxK = 6.80f;

// C^2 quintic LM13600 diode-linearized OTA rail spans:
// P(u) = u*(1 - (2/3)*u^2 + (1/5)*u^4) has asymptote (8/15)*span, so span = (15/8)*I_max.
inline constexpr float kHpOta1Pos = (15.0f / 8.0f) * 1.064f;
inline constexpr float kInvHpOta1Pos = 1.0f / kHpOta1Pos;
inline constexpr float kHpOta1Neg = (15.0f / 8.0f) * 1.096f;
inline constexpr float kInvHpOta1Neg = 1.0f / kHpOta1Neg;
inline constexpr float kHpOta2Rail = (15.0f / 8.0f) * 1.080f;
inline constexpr float kInvHpOta2Rail = 1.0f / kHpOta2Rail;

inline constexpr float kLpOta1Rail = (15.0f / 8.0f) * 1.080f;
inline constexpr float kInvLpOta1Rail = 1.0f / kLpOta1Rail;
inline constexpr float kLpOta2Pos = (15.0f / 8.0f) * 1.108f;
inline constexpr float kLpOta2Neg = (15.0f / 8.0f) * 1.052f;

// C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback knees:
// Q(u) = u*(1 - (3/5)*u^4 + (2/7)*u^6) has asymptote (24/35)*span, so span = (35/24)*V_diode.
inline constexpr float kHpDiodeSpan = (35.0f / 24.0f) * 1.07f;
inline constexpr float kLpDiodeSpan = (35.0f / 24.0f) * 0.91f;

inline constexpr float kDiodeOhmicSlope = 0.024f;
inline constexpr float kDiodeJunctionWeight = 1.0f - kDiodeOhmicSlope;

// Pure-FPU branchless NaN/Inf/subnormal scrubber (under -ffp-contract=off, +1e-18f - 1e-18f
// flushes |x| < ~3e-26f to bit-exact +0.0f).
RUIDO_ALWAYS_INLINE float scrubFilterState(float x) noexcept {
    const float bounded = clampf(x, -64.0f, 64.0f);
    return (bounded + 1.0e-18f) - 1.0e-18f;
}

inline float computeKorg35Chaos(float resonance) noexcept {
    return smoothstep01((clampf(resonance, 0.0f, 1.0f) - 0.60f) * (1.0f / 0.40f));
}

// Three-regime MS-20 resonance curve:
//  - [0.00, 0.50]: warm non-oscillating LM13600 OTA resonant Q (k = 0.5858 -> 2.00)
//  - (0.50, 0.60]: creamy KLM-307 Rev 2 singing sine self-oscillation (k = 2.00 -> 3.60)
//  - (0.60, 1.00]: feral Korg35 Rev 1 silicon-clamped screaming instability (k = 3.60 -> 6.80)
inline float mapResonanceToK(float resonance) noexcept {
    const float r = clampf(resonance, 0.0f, 1.0f);
    if (r <= 0.50f) {
        const float u = r * 2.0f;
        const float shaped = u * (0.82f + 0.18f * u);
        return kMs20ButterworthK + shaped * (2.00f - kMs20ButterworthK);
    }
    if (r <= 0.60f) {
        const float w = (r - 0.50f) * 10.0f;
        const float shaped = w * (1.28f - 0.28f * w);
        return 2.00f + shaped * (kMs20Rev2MaxK - 2.00f);
    }
    const float v = (r - 0.60f) * (1.0f / 0.40f);
    return kMs20Rev2MaxK + (kMs20Korg35MaxK - kMs20Rev2MaxK) * v * (0.62f + 0.38f * v);
}

// Analog LM13600 / Korg35 OTA gain-bandwidth (GBW) high-frequency loop-gain roll-off:
// Untouched up to 8.2 kHz (preserving < 3% self-oscillation tracking through 8 kHz); smoothly
// tapers excess feedback across 8.2..12.2 kHz below k < 1.78 so opening or LFO-sweeping the
// filter into 12..28 kHz preserves bright resonant sheen without sustaining an ultrasonic
// limit cycle or square-edge intermodulation fizz.
inline float applyOtaHfLoopTaper(float k, float cutoffHz) noexcept {
    if (cutoffHz <= 8200.0f || k <= 1.55f) {
        return k;
    }
    const float roll = smoothstep01((cutoffHz - 8200.0f) * (1.0f / 4000.0f));
    const float hfCeil = 1.55f + 0.22f * clampf((k - 1.55f) * 0.20f, 0.0f, 1.0f);
    const float target = (k < hfCeil) ? k : hfCeil;
    return k + roll * (target - k);
}

// Groszkowski reactive-power + dual-OTA compression pitch compensation:
// Accurate < 2.5% tracking through the 0-60% Rev 2 singing zone; intentionally tapers above
// k = 3.60 so the 60-100% Korg35 regime exhibits authentic diode-squaring pitch growl.
inline float compensateHpSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float rev2Ex = (k < kMs20Rev2MaxK) ? (k - 2.0f) : (kMs20Rev2MaxK - 2.0f);
    const float korgEx = (k > kMs20Rev2MaxK) ? (k - kMs20Rev2MaxK) : 0.0f;
    const float shift = rev2Ex * (0.197f - 0.038f * rev2Ex) - korgEx * 0.008f;
    return clampf(hz * (1.0f + shift), kFilterCutoffMinHz, kFilterMaxSafeHz);
}

inline float compensateLpSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float rev2Ex = (k < kMs20Rev2MaxK) ? (k - 2.0f) : (kMs20Rev2MaxK - 2.0f);
    const float korgEx = (k > kMs20Rev2MaxK) ? (k - kMs20Rev2MaxK) : 0.0f;
    const float shift = rev2Ex * (0.216f - 0.040f * rev2Ex) - korgEx * 0.008f;
    return clampf(hz * (1.0f + shift), kFilterCutoffMinHz, kFilterMaxSafeHz);
}

// Pre-filter MS-20 ESP-style asymmetric transistor input overdrive (1.0x = bit-exact unity pass;
// > 1.0x = asymmetric even/odd soft-clipping before HPF C1 capacitor strips input DC offset).
// Clamping `driven` to [-3*1.12, 3*1.48] = [-3.36, 4.44] keeps |u| <= 3.0f so `rail * padeClip(u)`
// simplifies identically to `vc * (27 + u^2) / (27 + 9*u^2)` with zero saturation branches.
RUIDO_ALWAYS_INLINE float applyMs20InputOverdrive(float input, float preDrive,
                                                  float driveBlend) noexcept {
    const float clean = clampf(input, -16.0f, 16.0f);
    if (driveBlend <= 0.0f) {
        return clean;
    }
    const float vc = clampf(clean * preDrive, -3.36f, 4.44f);
    const float u = vc * ((vc >= 0.0f) ? (1.0f / 1.48f) : (1.0f / 1.12f));
    const float u2 = u * u;
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
    const float clipped = (vc * (27.0f + u2)) / (27.0f + 9.0f * u2);
#else
    const float clipped = (vc * (27.0f + u2)) * fastRecipPos(27.0f + 9.0f * u2);
#endif
    return clean + driveBlend * (clipped - clean);
}

// Pure-FPU C^2-continuous LM13600 diode-linearized OTA transconductor
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
RUIDO_ALWAYS_INLINE SatResult evalOta(float v, float posSpan, float negSpan, float invPosSpan,
                                      float invNegSpan) noexcept {
    const bool pos = (v >= 0.0f);
    const float u = clampf(v * (pos ? invPosSpan : invNegSpan), -1.0f, 1.0f);
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {(u * (pos ? posSpan : negSpan)) * (1.0f - u2 * (0.66666667f - 0.20f * u2)),
            oneMinusU2 * oneMinusU2};
}

RUIDO_ALWAYS_INLINE SatResult evalOtaSymmetric(float v, float span, float invSpan) noexcept {
    const float u = clampf(v * invSpan, -1.0f, 1.0f);
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {(u * span) * (1.0f - u2 * (0.66666667f - 0.20f * u2)), oneMinusU2 * oneMinusU2};
}

RUIDO_ALWAYS_INLINE SatResult evalSiliconDiode(float w, float span, float invSpan) noexcept {
    constexpr float kFoldedC4 = kDiodeJunctionWeight * 0.60f;
    constexpr float kFoldedC6 = kDiodeJunctionWeight * 0.28571429f;
    constexpr float kDerivC4 = kDiodeJunctionWeight * 3.0f;
    constexpr float kDerivC6 = kDiodeJunctionWeight * 2.0f;
    const float u = clampf(w * invSpan, -1.0f, 1.0f);
    const float u2 = u * u;
    const float u4 = u2 * u2;
    return {(u * span) * (kDiodeJunctionWeight - u4 * (kFoldedC4 - kFoldedC6 * u2)) +
                kDiodeOhmicSlope * w,
            1.0f - u4 * (kDerivC4 - kDerivC6 * u2)};
}
#else
RUIDO_ALWAYS_INLINE SatResult evalOta(float v, float posSpan, float negSpan, float invPosSpan,
                                      float invNegSpan) noexcept {
    const float vc = clampf(v, -negSpan, posSpan);
    const float u = vc * ((vc >= 0.0f) ? invPosSpan : invNegSpan);
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {vc * (1.0f - u2 * (0.66666667f - 0.20f * u2)), oneMinusU2 * oneMinusU2};
}

RUIDO_ALWAYS_INLINE SatResult evalOtaSymmetric(float v, float span, float invSpan) noexcept {
    const float vc = clampf(v, -span, span);
    const float u = vc * invSpan;
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {vc * (1.0f - u2 * (0.66666667f - 0.20f * u2)), oneMinusU2 * oneMinusU2};
}

// Pure-FPU C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback limiter.
// Uses exact polynomial identity (1 - u^2)^2 * (1 + 2*u^2) == 1 - u^4 * (3 - 2*u^2) with pre-folded
// junction/ohmic weights to save 4 FPU ops and 1 live register per 176.47 kHz stage evaluation.
RUIDO_ALWAYS_INLINE SatResult evalSiliconDiode(float w, float span, float invSpan) noexcept {
    constexpr float kFoldedC4 = kDiodeJunctionWeight * 0.60f;
    constexpr float kFoldedC6 = kDiodeJunctionWeight * 0.28571429f;
    constexpr float kDerivC4 = kDiodeJunctionWeight * 3.0f;
    constexpr float kDerivC6 = kDiodeJunctionWeight * 2.0f;
    const float wc = clampf(w, -span, span);
    const float u = wc * invSpan;
    const float u2 = u * u;
    const float u4 = u2 * u2;
    return {wc * (kDiodeJunctionWeight - u4 * (kFoldedC4 - kFoldedC6 * u2)) + kDiodeOhmicSlope * w,
            1.0f - u4 * (kDerivC4 - kDerivC6 * u2)};
}
#endif

// Exact linear ZDF 2-pole Butterworth capacitive Sallen-Key HPF for parked 2 Hz / 0% RES AC
// coupling under pre-filter overdrive (keeps s1/s2/hist1/hist2 at exact capacitor nodal voltages).
RUIDO_ALWAYS_INLINE float stepHpLinearCapacitive(float x, float& s1, float& s2, float& hist1,
                                                 float& hist2, float g, float invLinDet) noexcept {
    const float y2 = ((x + s1) + (1.0f + g) * s2) * invLinDet;
    const float ic2 = -g * y2;
    const float y1 = y2 - s2 - ic2;
    s1 = 2.0f * (y1 - x) - s1;
    s2 = s2 + 2.0f * ic2;
    hist2 = hist1;
    hist1 = y2 - x;
    return y2;
}

RUIDO_ALWAYS_INLINE float stepHpNonlinearPrecomputed(float x, float& s1, float& s2, float& hist1,
                                                     float& hist2, float g, float k, float pos1,
                                                     float neg1, float invPos1, float invNeg1,
                                                     float rail2, float invRail2, float diodeSpan,
                                                     float invDiodeSpan) noexcept {
    const float xPlusS1 = x + s1;
    float y2 = x + clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
    // At 176.47 kHz (4x OS) from the 2nd-order (2*h1 - h2) predictor, 1 exact analytical NR step
    // + tangent update reaches O(dt^6) accuracy; early Jacobian folding kills 6 live values before
    // `evalOta` so states stay register-resident in 16-GPR q32DSP.
    const SatResult ota2_0 = evalOtaSymmetric(-y2, rail2, invRail2);
    const float y1_0 = y2 - s2 - g * ota2_0.value;
    const float onePlusD2 = 1.0f + g * ota2_0.deriv;
    const SatResult fb_0 = evalSiliconDiode(k * y2, diodeSpan, invDiodeSpan);
    const float kFbDeriv = k * fb_0.deriv;
    const SatResult ota1_0 = evalOta(fb_0.value - y1_0, pos1, neg1, invPos1, invNeg1);
    const float fHp0 = (y1_0 - xPlusS1) - g * ota1_0.value;
    const float d1_0 = g * ota1_0.deriv;
    const float dfHp = (1.0f + d1_0) * onePlusD2 - d1_0 * kFbDeriv;
    const float dy2_0 = clampf(fHp0 / ((dfHp > 1.0e-6f) ? dfHp : 1.0e-6f), -4.0f, 4.0f);
    y2 -= dy2_0;
    const float finalOta2 = ota2_0.value + ota2_0.deriv * dy2_0;
#else
    const SatResult ota2_0 = evalOtaSymmetric(-y2, rail2, invRail2);
    const float y1_0 = y2 - s2 - g * ota2_0.value;
    const SatResult fb_0 = evalSiliconDiode(k * y2, diodeSpan, invDiodeSpan);
    const SatResult ota1_0 = evalOta(fb_0.value - y1_0, pos1, neg1, invPos1, invNeg1);

    const float fHp0 = y1_0 - xPlusS1 - g * ota1_0.value;
    const float d1_0 = g * ota1_0.deriv;
    const float d2_0 = g * ota2_0.deriv;
    const float dy2_0 =
        fHp0 * fastRecipPos((1.0f + d1_0) * (1.0f + d2_0) - d1_0 * (k * fb_0.deriv));
    y2 -= dy2_0;
    float finalOta2 = ota2_0.value + ota2_0.deriv * dy2_0;

    if (!(absf(dy2_0) < 1.0e-2f)) {
        const SatResult ota2_1 = evalOtaSymmetric(-y2, rail2, invRail2);
        const float y1_1 = y2 - s2 - g * ota2_1.value;
        const SatResult fb_1 = evalSiliconDiode(k * y2, diodeSpan, invDiodeSpan);
        const SatResult ota1_1 = evalOta(fb_1.value - y1_1, pos1, neg1, invPos1, invNeg1);

        const float fHp1 = y1_1 - xPlusS1 - g * ota1_1.value;
        const float d1_1 = g * ota1_1.deriv;
        const float d2_1 = g * ota2_1.deriv;
        const float dy2_1 =
            fHp1 * fastRecipPos((1.0f + d1_1) * (1.0f + d2_1) - d1_1 * (k * fb_1.deriv));
        y2 -= dy2_1;
        finalOta2 = ota2_1.value + ota2_1.deriv * dy2_1;
    }
#endif

    const float ic2 = g * finalOta2;
    const float y1 = y2 - s2 - ic2;
    s1 = 2.0f * (y1 - x) - s1;
    s2 = s2 + 2.0f * ic2;
    hist2 = hist1;
    hist1 = y2 - x;
    return y2;
}

RUIDO_ALWAYS_INLINE float stepLpNonlinear(float x, float& s1, float& s2, float& hist1, float& hist2,
                                          float g, float k, float rail1, float invRail1, float pos2,
                                          float neg2, float invPos2, float invNeg2, float diodeSpan,
                                          float invDiodeSpan) noexcept {
    float w = clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
    const SatResult ota2_0 = evalOta(w, pos2, neg2, invPos2, invNeg2);
    const float y2_0 = s2 + g * ota2_0.value;
    const float y1_0 = w + y2_0;
    const SatResult ota1_0 = evalOtaSymmetric(x - y1_0, rail1, invRail1);
    const float jac12 = (1.0f + g * ota1_0.deriv) * (1.0f + g * ota2_0.deriv);
    const float resPre = (y1_0 - s1) - g * ota1_0.value;
    const SatResult fb_0 = evalSiliconDiode(k * w, diodeSpan, invDiodeSpan);
    const float fLp0 = resPre - g * fb_0.value;
    const float dfLp = jac12 - g * (k * fb_0.deriv);
    const float dw0 = clampf(fLp0 / ((dfLp > 1.0e-6f) ? dfLp : 1.0e-6f), -4.0f, 4.0f);
    w -= dw0;
    const float finalOta2 = ota2_0.value - ota2_0.deriv * dw0;
#else
    const SatResult ota2_0 = evalOta(w, pos2, neg2, invPos2, invNeg2);
    const float y2_0 = s2 + g * ota2_0.value;
    const float y1_0 = w + y2_0;
    const SatResult ota1_0 = evalOtaSymmetric(x - y1_0, rail1, invRail1);
    const SatResult fb_0 = evalSiliconDiode(k * w, diodeSpan, invDiodeSpan);

    const float fLp0 = y1_0 - s1 - g * (ota1_0.value + fb_0.value);
    const float d1_0 = g * ota1_0.deriv;
    const float d2_0 = g * ota2_0.deriv;
    const float dw0 = fLp0 * fastRecipPos((1.0f + d1_0) * (1.0f + d2_0) - g * (k * fb_0.deriv));
    w -= dw0;
    float finalOta2 = ota2_0.value - ota2_0.deriv * dw0;

    if (!(absf(dw0) < 1.0e-2f)) {
        const SatResult ota2_1 = evalOta(w, pos2, neg2, invPos2, invNeg2);
        const float y2_1 = s2 + g * ota2_1.value;
        const float y1_1 = w + y2_1;
        const SatResult ota1_1 = evalOtaSymmetric(x - y1_1, rail1, invRail1);
        const SatResult fb_1 = evalSiliconDiode(k * w, diodeSpan, invDiodeSpan);

        const float fLp1 = y1_1 - s1 - g * (ota1_1.value + fb_1.value);
        const float d1_1 = g * ota1_1.deriv;
        const float d2_1 = g * ota2_1.deriv;
        const float dw1 = fLp1 * fastRecipPos((1.0f + d1_1) * (1.0f + d2_1) - g * (k * fb_1.deriv));
        w -= dw1;
        finalOta2 = ota2_1.value - ota2_1.deriv * dw1;
    }
#endif

    const float ic2 = g * finalOta2;
    const float y2 = s2 + ic2;
    const float y1 = w + y2;
    s1 = 2.0f * y1 - s1;
    s2 = s2 + 2.0f * ic2;
    hist2 = hist1;
    hist1 = w;
    return y2;
}

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
// Paired 2-subsample (176.47 kHz -> 88.24 kHz secant/Jacobian refresh) ZDF solvers for AC791N:
// Even subsamples evaluate exact C^2/C^4 OTA + silicon-diode polynomials and 1/df; odd subsamples
// step full 176.47 kHz trapezoidal capacitor states and [-1, 1] rail clamps using the even tick's
// smooth secant conductances and inverse Jacobian (eliminating 50% of fdivs and u^5/u^6 polys while
// keeping any conductance-rate images > 68 kHz inside the 4x half-band decimator's -108 dB
// stopband).
RUIDO_ALWAYS_INLINE void stepHpPair4xPi32(float x0, float x1, float& out0, float& out1, float& s1,
                                          float& s2, float& hist1, float& hist2, float g, float k,
                                          float pos1, float neg1, float invPos1, float invNeg1,
                                          float rail2, float invRail2, float diodeSpan,
                                          float invDiodeSpan) noexcept {
    constexpr float kFoldedC4 = kDiodeJunctionWeight * 0.60f;
    constexpr float kFoldedC6 = kDiodeJunctionWeight * 0.28571429f;
    constexpr float kDerivC4 = kDiodeJunctionWeight * 3.0f;
    constexpr float kDerivC6 = kDiodeJunctionWeight * 2.0f;

    const float x0PlusS1 = x0 + s1;
    float y2_0 = x0 + clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    const float uOta2_0 = clampf(-y2_0 * invRail2, -1.0f, 1.0f);
    const float u2Ota2 = uOta2_0 * uOta2_0;
    const float effRail2 = rail2 * (1.0f - u2Ota2 * (0.66666667f - 0.20f * u2Ota2));
    const float omOta2 = 1.0f - u2Ota2;
    const float dOta2 = omOta2 * omOta2;
    const float ota2Val0 = uOta2_0 * effRail2;
    const float y1_0 = y2_0 - s2 - g * ota2Val0;
    const float onePlusD2 = 1.0f + g * dOta2;

    const float ky2_0 = k * y2_0;
    const float uDiode0 = clampf(ky2_0 * invDiodeSpan, -1.0f, 1.0f);
    const float u2Diode = uDiode0 * uDiode0;
    const float u4Diode = u2Diode * u2Diode;
    const float effDiodeSpan =
        diodeSpan * (kDiodeJunctionWeight - u4Diode * (kFoldedC4 - kFoldedC6 * u2Diode));
    const float fbVal0 = uDiode0 * effDiodeSpan + kDiodeOhmicSlope * ky2_0;
    const float kFbDeriv = k * (1.0f - u4Diode * (kDerivC4 - kDerivC6 * u2Diode));

    const float v1_0 = fbVal0 - y1_0;
    const bool p1_0 = (v1_0 >= 0.0f);
    const float uOta1_0 = clampf(v1_0 * (p1_0 ? invPos1 : invNeg1), -1.0f, 1.0f);
    const float u2Ota1 = uOta1_0 * uOta1_0;
    const float secOta1 = 1.0f - u2Ota1 * (0.66666667f - 0.20f * u2Ota1);
    const float omOta1 = 1.0f - u2Ota1;
    const float d1_0 = g * (omOta1 * omOta1);
    const float ota1Val0 = (uOta1_0 * (p1_0 ? pos1 : neg1)) * secOta1;

    const float fHp0 = (y1_0 - x0PlusS1) - g * ota1Val0;
    const float dfHp = (1.0f + d1_0) * onePlusD2 - d1_0 * kFbDeriv;
    const float invDfHp = 1.0f / ((dfHp > 1.0e-6f) ? dfHp : 1.0e-6f);
    const float dy2_0 = clampf(fHp0 * invDfHp, -4.0f, 4.0f);
    y2_0 -= dy2_0;
    const float ic2_0 = g * (ota2Val0 + dOta2 * dy2_0);
    const float y1_0f = y2_0 - s2 - ic2_0;
    s1 = 2.0f * (y1_0f - x0) - s1;
    s2 = s2 + 2.0f * ic2_0;
    hist2 = hist1;
    hist1 = y2_0 - x0;
    out0 = y2_0;

    const float x1PlusS1 = x1 + s1;
    float y2_1 = x1 + clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    const float ota2Val1 = clampf(-y2_1 * invRail2, -1.0f, 1.0f) * effRail2;
    const float y1_1 = y2_1 - s2 - g * ota2Val1;
    const float ky2_1 = k * y2_1;
    const float fbVal1 =
        clampf(ky2_1 * invDiodeSpan, -1.0f, 1.0f) * effDiodeSpan + kDiodeOhmicSlope * ky2_1;
    const float v1_1 = fbVal1 - y1_1;
    const bool p1_1 = (v1_1 >= 0.0f);
    const float uOta1_1 = clampf(v1_1 * (p1_1 ? invPos1 : invNeg1), -1.0f, 1.0f);
    const float ota1Val1 = (uOta1_1 * (p1_1 ? pos1 : neg1)) * secOta1;
    const float dy2_1 = clampf(((y1_1 - x1PlusS1) - g * ota1Val1) * invDfHp, -4.0f, 4.0f);
    y2_1 -= dy2_1;
    const float ic2_1 = g * (ota2Val1 + dOta2 * dy2_1);
    const float y1_1f = y2_1 - s2 - ic2_1;
    s1 = 2.0f * (y1_1f - x1) - s1;
    s2 = s2 + 2.0f * ic2_1;
    hist2 = hist1;
    hist1 = y2_1 - x1;
    out1 = y2_1;
}

RUIDO_ALWAYS_INLINE void stepLpPair4xPi32(float x0, float x1, float& out0, float& out1, float& s1,
                                          float& s2, float& hist1, float& hist2, float g, float k,
                                          float rail1, float invRail1, float pos2, float neg2,
                                          float invPos2, float invNeg2, float diodeSpan,
                                          float invDiodeSpan) noexcept {
    constexpr float kFoldedC4 = kDiodeJunctionWeight * 0.60f;
    constexpr float kFoldedC6 = kDiodeJunctionWeight * 0.28571429f;
    constexpr float kDerivC4 = kDiodeJunctionWeight * 3.0f;
    constexpr float kDerivC6 = kDiodeJunctionWeight * 2.0f;

    float w0 = clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    const bool p2_0 = (w0 >= 0.0f);
    const float uOta2_0 = clampf(w0 * (p2_0 ? invPos2 : invNeg2), -1.0f, 1.0f);
    const float u2Ota2 = uOta2_0 * uOta2_0;
    const float secOta2 = 1.0f - u2Ota2 * (0.66666667f - 0.20f * u2Ota2);
    const float omOta2 = 1.0f - u2Ota2;
    const float dOta2 = omOta2 * omOta2;
    const float ota2Val0 = (uOta2_0 * (p2_0 ? pos2 : neg2)) * secOta2;
    const float y2_0 = s2 + g * ota2Val0;
    const float y1_0 = w0 + y2_0;

    const float uOta1_0 = clampf((x0 - y1_0) * invRail1, -1.0f, 1.0f);
    const float u2Ota1 = uOta1_0 * uOta1_0;
    const float effRail1 = rail1 * (1.0f - u2Ota1 * (0.66666667f - 0.20f * u2Ota1));
    const float omOta1 = 1.0f - u2Ota1;
    const float jac12 = (1.0f + g * (omOta1 * omOta1)) * (1.0f + g * dOta2);
    const float resPre0 = (y1_0 - s1) - g * (uOta1_0 * effRail1);

    const float kw0 = k * w0;
    const float uDiode0 = clampf(kw0 * invDiodeSpan, -1.0f, 1.0f);
    const float u2Diode = uDiode0 * uDiode0;
    const float u4Diode = u2Diode * u2Diode;
    const float effDiodeSpan =
        diodeSpan * (kDiodeJunctionWeight - u4Diode * (kFoldedC4 - kFoldedC6 * u2Diode));
    const float fbVal0 = uDiode0 * effDiodeSpan + kDiodeOhmicSlope * kw0;
    const float fbDeriv0 = 1.0f - u4Diode * (kDerivC4 - kDerivC6 * u2Diode);

    const float dfLp = jac12 - g * (k * fbDeriv0);
    const float invDfLp = 1.0f / ((dfLp > 1.0e-6f) ? dfLp : 1.0e-6f);
    const float dw0 = clampf((resPre0 - g * fbVal0) * invDfLp, -4.0f, 4.0f);
    w0 -= dw0;
    const float ic2_0 = g * (ota2Val0 - dOta2 * dw0);
    const float y2_0f = s2 + ic2_0;
    const float y1_0f = w0 + y2_0f;
    s1 = 2.0f * y1_0f - s1;
    s2 = s2 + 2.0f * ic2_0;
    hist2 = hist1;
    hist1 = w0;
    out0 = y2_0f;

    float w1 = clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    const bool p2_1 = (w1 >= 0.0f);
    const float uOta2_1 = clampf(w1 * (p2_1 ? invPos2 : invNeg2), -1.0f, 1.0f);
    const float ota2Val1 = (uOta2_1 * (p2_1 ? pos2 : neg2)) * secOta2;
    const float y2_1 = s2 + g * ota2Val1;
    const float y1_1 = w1 + y2_1;
    const float ota1Val1 = clampf((x1 - y1_1) * invRail1, -1.0f, 1.0f) * effRail1;
    const float kw1 = k * w1;
    const float fbVal1 =
        clampf(kw1 * invDiodeSpan, -1.0f, 1.0f) * effDiodeSpan + kDiodeOhmicSlope * kw1;
    const float dw1 = clampf(((y1_1 - s1) - g * (ota1Val1 + fbVal1)) * invDfLp, -4.0f, 4.0f);
    w1 -= dw1;
    const float ic2_1 = g * (ota2Val1 - dOta2 * dw1);
    const float y2_1f = s2 + ic2_1;
    const float y1_1f = w1 + y2_1f;
    s1 = 2.0f * y1_1f - s1;
    s2 = s2 + 2.0f * ic2_1;
    hist2 = hist1;
    hist1 = w1;
    out1 = y2_1f;
}
#endif

} // namespace ruido
