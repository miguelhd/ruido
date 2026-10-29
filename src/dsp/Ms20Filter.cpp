#include "dsp/Ms20Filter.h"

#include "core/Config.h"
#include "core/DspMath.h"

#include <cmath>

namespace ruido {

namespace {

// Exact Butterworth maximally-flat Sallen-Key gain: k = 2 - sqrt(2) => Q = 0.70710678
constexpr float kButterworthK = 2.0f - kSqrt2; // 0.58578644f
// Self-oscillation threshold occurs at k = 2.0f; kMax = 3.20f sustains rich diode howl
// while keeping alpha * k < 0.25 * 3.20 = 0.80 < 1.0 (strictly monotonic root!).
constexpr float kMaxResonanceK = 3.20f;

inline float mapResonanceToK(float resonance) noexcept {
    const float r = clampf(resonance, 0.0f, 1.0f);
    const float shaped = r * (0.75f + 0.25f * r);
    return kButterworthK + shaped * (kMaxResonanceK - kButterworthK);
}

// Compensates Van der Pol nonlinear period elongation when k > 2.0 drives into diode clipping,
// keeping self-oscillation pitch accurate across the upper resonance range.
inline float compensateSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float excess = k - 2.0f;
    return hz * (1.0f + 0.0365f * excess);
}

// Exact fused single-division Newton-Raphson step for F(w) = w - b - alpha * phi(k * w) = 0.
// By placing phi(y) = y*(27 + z^2)/D and phi'(y) = (Nd/D)^2 over common denominator D^2 - ak*Nd^2,
// evaluating phi, phi', solving the Jacobian f/f', and updating phi_lin = phi - k*phi'*dw
// collapses identically into ONE division:
//   phi_next = (y * (27 + z^2) * D - ak * Nd^2 * phi_prev) / (D^2 - ak * Nd^2)
inline float newtonStepKorg35(float b, float alpha, float k, float ak, float phiPrev) noexcept {
    constexpr float kPosLim = 0.92f;
    constexpr float kInvPosLim = 1.0f / kPosLim;
    constexpr float kNegLim = 1.08f;
    constexpr float kInvNegLim = 1.0f / kNegLim;

    const float y = k * (b + alpha * phiPrev);
    const float invLim = (y >= 0.0f) ? kInvPosLim : kInvNegLim;
    const float z = y * invLim;
    if (z >= 3.0f) {
        return kPosLim;
    }
    if (z <= -3.0f) {
        return -kNegLim;
    }
    const float z2 = z * z;
    const float dPoly = 27.0f + 9.0f * z2;
    const float ndPoly = 27.0f - 3.0f * z2;
    const float akNd2 = ak * (ndPoly * ndPoly);
    const float phiNext =
        (y * (27.0f + z2) * dPoly - akNd2 * phiPrev) / (dPoly * dPoly - akNd2);
    return clampf(phiNext, -kNegLim, kPosLim);
}

inline float solveDiodeFeedback(float b, float alpha, float k, float ak, float lastPhi) noexcept {
    const float phi1 = newtonStepKorg35(b, alpha, k, ak, lastPhi);
    if (::fabsf(phi1 - lastPhi) > 1.0e-3f) {
        return newtonStepKorg35(b, alpha, k, ak, phi1);
    }
    return phi1;
}

} // namespace

void Ms20DualFilter::reset() noexcept {
    hpf_.reset();
    lpf_.reset();
}

Ms20DualFilter::Coeffs Ms20DualFilter::prepareCoeffs(float hpCutoffHz, float lpCutoffHz,
                                                     float resonance, float drive,
                                                     float invSampleRate) noexcept {
    Coeffs c{};
    const float hpHz = clampf(hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float lpHz = clampf(lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float drv = clampf(drive, 0.0f, 1.0f);

    if (drv > 1.0e-5f) {
        const float preGain = 1.0f + drv * drv * 5.5f;
        c.preDriveGain = preGain * (1.0f / 1.8f);
        c.postDriveScale = (1.8f * fastInvSqrt(preGain)) * (1.0f + 0.35f * drv);
        c.hasDrive = true;
    }

    // Smart-Park Shared Resonance & Phase-Transparent Parking:
    //  - When HP is parked near 20 Hz (20..36 Hz), Q tapers to Butterworth 0.7071 AND internal
    //    pole glides down to 2.5 Hz so 55-100 Hz bass square waves keep flat phase-true roofs!
    //  - When LP is parked near 20 kHz (16.5k..20k Hz), Q tapers to Butterworth 0.7071 AND
    //    internal pole opens to 28 kHz for zero top-octave phase droop.
    const float targetK = mapResonanceToK(resonance);
    const float hpActive = smoothstep01((hpHz - 20.0f) * (1.0f / 16.0f));
    const float lpActive = smoothstep01((20000.0f - lpHz) * (1.0f / 3500.0f));

    c.kHp = kButterworthK + hpActive * (targetK - kButterworthK);
    c.kLp = kButterworthK + lpActive * (targetK - kButterworthK);

    const float effectiveHpHz =
        compensateSelfOscPitch(2.5f + hpActive * (hpHz - 2.5f), c.kHp);
    const float effectiveLpHz =
        compensateSelfOscPitch(28000.0f + lpActive * (lpHz - 28000.0f), c.kLp);

    const float tanHp = fastPrewarpTan(effectiveHpHz, invSampleRate);
    const float tanLp = fastPrewarpTan(effectiveLpHz, invSampleRate);

    c.gHp = tanHp / (1.0f + tanHp);
    c.oneMinusGHp = 1.0f - c.gHp;
    c.alphaHp = c.gHp * c.oneMinusGHp;
    c.akHp = c.alphaHp * c.kHp;

    c.gLp = tanLp / (1.0f + tanLp);
    c.oneMinusGLp = 1.0f - c.gLp;
    c.alphaLp = c.gLp * c.oneMinusGLp;
    c.akLp = c.alphaLp * c.kLp;
    return c;
}

float Ms20DualFilter::processHighPass(Korg35Stage& st, float x, const Coeffs& c) noexcept {
    const float b = c.alphaHp * x + c.oneMinusGHp * (st.s2 - c.gHp * st.s1);

    const float phi = solveDiodeFeedback(b, c.alphaHp, c.kHp, c.akHp, st.lastPhi);
    st.lastPhi = sanitizeState(phi);
    const float u = x + phi;

    const float v1 = c.gHp * (u - st.s1);
    const float lp1 = v1 + st.s1;
    const float hp1 = u - lp1;
    st.s1 = sanitizeState(lp1 + v1);

    const float v2 = c.gHp * (hp1 - st.s2);
    const float lp2 = v2 + st.s2;
    const float hp2 = hp1 - lp2;
    st.s2 = sanitizeState(lp2 + v2);

    return hp2;
}

float Ms20DualFilter::processLowPass(Korg35Stage& st, float x, const Coeffs& c) noexcept {
    const float b = c.alphaLp * x + c.oneMinusGLp * (c.oneMinusGLp * st.s1 - st.s2);

    const float phi = solveDiodeFeedback(b, c.alphaLp, c.kLp, c.akLp, st.lastPhi);
    st.lastPhi = sanitizeState(phi);
    const float u = x + phi;

    const float v1 = c.gLp * (u - st.s1);
    const float lp1 = v1 + st.s1;
    st.s1 = sanitizeState(lp1 + v1);

    const float v2 = c.gLp * (lp1 - st.s2);
    const float lp2 = v2 + st.s2;
    st.s2 = sanitizeState(lp2 + v2);

    return lp2;
}

float Ms20DualFilter::processPrepared(float input, const Coeffs& c) noexcept {
    float driven = clampf(sanitizeState(input), -16.0f, 16.0f);
    if (c.hasDrive) {
        driven = padeClip(driven * c.preDriveGain) * c.postDriveScale;
    }
    const float afterHp = processHighPass(hpf_, driven, c);
    const float afterLp = processLowPass(lpf_, afterHp, c);
    return sanitizeState(afterLp);
}

} // namespace ruido
