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

inline float mapResonanceToK(float resonance) noexcept {
    const float r = clampf(resonance, 0.0f, 1.0f);
    if (r <= 0.60f) {
        const float u = r * (1.0f / 0.60f);
        const float shaped = u * (1.38f - 0.38f * u);
        return kMs20ButterworthK + shaped * (kMs20Rev2MaxK - kMs20ButterworthK);
    }
    const float v = (r - 0.60f) * (1.0f / 0.40f);
    return kMs20Rev2MaxK + (kMs20Korg35MaxK - kMs20Rev2MaxK) * v * (0.62f + 0.38f * v);
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
    const float shift = rev2Ex * (0.062f + 0.046f * rev2Ex) - korgEx * 0.008f;
    return clampf(hz * (1.0f + shift), kFilterCutoffMinHz, kFilterMaxSafeHz);
}

inline float compensateLpSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float rev2Ex = (k < kMs20Rev2MaxK) ? (k - 2.0f) : (kMs20Rev2MaxK - 2.0f);
    const float korgEx = (k > kMs20Rev2MaxK) ? (k - kMs20Rev2MaxK) : 0.0f;
    const float shift = rev2Ex * (0.068f + 0.053f * rev2Ex) - korgEx * 0.008f;
    return clampf(hz * (1.0f + shift), kFilterCutoffMinHz, kFilterMaxSafeHz);
}

// Pre-filter MS-20 ESP-style asymmetric transistor input overdrive (1.0x = bit-exact unity pass;
// > 1.0x = asymmetric even/odd soft-clipping before HPF C1 capacitor strips input DC offset).
RUIDO_ALWAYS_INLINE float applyMs20InputOverdrive(float input, float preDrive,
                                                  float driveBlend) noexcept {
    const float clean = clampf(input, -16.0f, 16.0f);
    if (driveBlend <= 0.0f) {
        return clean;
    }
    const float driven = clean * preDrive;
    const float rail = (driven >= 0.0f) ? 1.48f : 1.12f;
    const float invRail = (driven >= 0.0f) ? (1.0f / 1.48f) : (1.0f / 1.12f);
    const float clipped = rail * padeClip(driven * invRail);
    return clean + driveBlend * (clipped - clean);
}

// Pure-FPU C^2-continuous LM13600 diode-linearized OTA transconductor
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

// Pure-FPU C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback limiter
RUIDO_ALWAYS_INLINE SatResult evalSiliconDiode(float w, float span, float invSpan) noexcept {
    const float wc = clampf(w, -span, span);
    const float u = wc * invSpan;
    const float u2 = u * u;
    const float u4 = u2 * u2;
    const float oneMinusU2 = 1.0f - u2;
    const float dVal = wc * (1.0f - u4 * (0.60f - 0.28571429f * u2));
    const float dDeriv = (oneMinusU2 * oneMinusU2) * (1.0f + 2.0f * u2);
    return {kDiodeJunctionWeight * dVal + kDiodeOhmicSlope * w,
            kDiodeJunctionWeight * dDeriv + kDiodeOhmicSlope};
}

RUIDO_ALWAYS_INLINE float stepHpNonlinearPrecomputed(float x, float& s1, float& s2, float& hist1,
                                                     float& hist2, float g, float k, float pos1,
                                                     float neg1, float invPos1, float invNeg1,
                                                     float rail2, float invRail2, float diodeSpan,
                                                     float invDiodeSpan) noexcept {
    const float xPlusS1 = x + s1;
    float y2 = x + clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);

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

    const float ic2 = g * finalOta2;
    const float y2 = s2 + ic2;
    const float y1 = w + y2;
    s1 = 2.0f * y1 - s1;
    s2 = s2 + 2.0f * ic2;
    hist2 = hist1;
    hist1 = w;
    return y2;
}

} // namespace ruido
