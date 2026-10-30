#include "dsp/Ms20Filter.h"

#include "core/Config.h"
#include "core/DspMath.h"

namespace ruido {

namespace {

// Exact Butterworth maximally-flat Sallen-Key gain: k = 2 - sqrt(2) => Q = 0.70710678
constexpr float kButterworthK = 2.0f - kSqrt2; // 0.58578644f
// Self-oscillation begins at k = 2.0f; kMax = 3.72f drives aggressive screaming Korg35 limit
// cycles while keeping alpha * k <= 0.25 * 3.72 = 0.93 < 1.0 (strictly monotonic unique root).
constexpr float kMaxResonanceK = 3.72f;

inline float mapResonanceToK(float resonance) noexcept {
    const float r = clampf(resonance, 0.0f, 1.0f);
    // Concave taper: thick vocal Q by 25%, self-oscillation by ~35%, screaming howl above 65%
    const float shaped = r * (1.42f - 0.42f * r);
    return kButterworthK + shaped * (kMaxResonanceK - kButterworthK);
}

// Compensates Van der Pol nonlinear period elongation when k > 2.0 drives into diode clipping,
// keeping self-oscillation pitch accurate across the upper resonance range.
inline float compensateSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float excess = k - 2.0f;
    return hz * (1.0f + excess * (0.0255f + 0.0130f * excess));
}

// Exact fused single-division Newton-Raphson step for F(w) = w - b - alpha * phi(k * w) = 0.
inline float newtonStepKorg35(float b, float alpha, float k, float ak, float phiPrev) noexcept {
    constexpr float kPosLim = 0.90f;
    constexpr float kInvPosLim = 1.0f / kPosLim;
    constexpr float kNegLim = 1.12f;
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
    if (absf(phi1 - lastPhi) > 1.2e-2f) {
        const float phi2 = newtonStepKorg35(b, alpha, k, ak, phi1);
        if (absf(phi2 - phi1) > 2.5e-2f) {
            return newtonStepKorg35(b, alpha, k, ak, phi2);
        }
        return phi2;
    }
    return phi1;
}

} // namespace

void Ms20DualFilter::reset() noexcept {
    hpf_.reset();
    lpf_.reset();
    dcMean_ = 0.0f;
}

Ms20DualFilter::Coeffs Ms20DualFilter::prepareCoeffs(float hpCutoffHz, float hpResonance,
                                                     float lpCutoffHz, float lpResonance,
                                                     float invSampleRate) noexcept {
    Coeffs c{};
    const float hpHz = clampf(hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float lpHz = clampf(lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float hpRes = clampf(hpResonance, 0.0f, 1.0f);
    const float lpRes = clampf(lpResonance, 0.0f, 1.0f);

    // Tight Smart-Park Independent HP & LP Resonance:
    // Flat phase-true bypass when parked right at 20 Hz / 20 kHz, waking up immediately within
    // a few encoder ticks (HP > 24 Hz or LP < 18.8 kHz) so each filter's peak grabs right away.
    const float targetKHp = mapResonanceToK(hpRes);
    const float targetKLp = mapResonanceToK(lpRes);
    const float hpActive = smoothstep01((hpHz - 20.0f) * (1.0f / 4.0f));
    const float lpActive = smoothstep01((20000.0f - lpHz) * (1.0f / 1200.0f));

    c.kHp = kButterworthK + hpActive * (targetKHp - kButterworthK);
    c.kLp = kButterworthK + lpActive * (targetKLp - kButterworthK);

    // Independent Korg35 per-stage passband-vs-resonance dividers: lets each stage's diode peak
    // tower prominently above the passband and prevents a self-oscillating HPF from swamping LPF.
    const float hpFocus = hpRes * hpActive;
    const float lpFocus = lpRes * lpActive;
    c.hpInScale = 1.0f / (1.0f + 0.70f * hpFocus);
    c.interStageScale = (1.0f + 0.52f * hpFocus) / (1.0f + 0.70f * lpFocus);
    c.lpOutScale = 1.0f + 0.52f * lpFocus;
    c.dcTrackG = clampf(kTwoPi * 8.0f * invSampleRate, 1.0e-5f, 0.05f);
    c.dcCancelMix = smoothstep01(lpFocus * 8.0f);

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
    const float cleanIn = clampf(input, -16.0f, 16.0f);
    const float afterHp = processHighPass(hpf_, cleanIn * c.hpInScale, c);
    const float afterLp = processLowPass(lpf_, afterHp * c.interStageScale, c);
    dcMean_ = sanitizeState(dcMean_ + c.dcTrackG * (afterLp - dcMean_));
    return (afterLp - c.dcCancelMix * dcMean_) * c.lpOutScale;
}

} // namespace ruido
