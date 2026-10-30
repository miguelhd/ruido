#include "dsp/Ms20Filter.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

// Exact Butterworth maximally-flat Sallen-Key gain: k = 2 - sqrt(2) => Q = 0.70710678
constexpr float kButterworthK = 2.0f - kSqrt2; // 0.58578644f
// Self-oscillation Hopf bifurcation occurs at k = 2.0f; kMax = 3.68f drives creamy MS-20 Rev 2
// LM13600 + 1S1588 silicon limit cycles while ultrasonic loop-gain clamp guarantees df > 0.12.
constexpr float kMaxResonanceK = 3.68f;

// C^2 quintic LM13600 diode-linearized OTA rail spans:
// P(u) = u*(1 - (2/3)*u^2 + (1/5)*u^4) has asymptote (8/15)*span, so span = (15/8)*I_max.
// Realistic KLM-307 ~1.08 nominal OTA saturation knee delivers creamy 0%-RES 2nd/3rd harmonic
// saturation, while monolithic LM13600 Wilson mirror tolerances (~1.5% - 2.4%) self-cancel
// limit-cycle rectification DC across HPF (C2-buffered output) and LPF (|y1| = sqrt(2)*|w|).
constexpr float kHpOta1Pos = (15.0f / 8.0f) * 1.064f;
constexpr float kInvHpOta1Pos = 1.0f / kHpOta1Pos;
constexpr float kHpOta1Neg = (15.0f / 8.0f) * 1.096f;
constexpr float kInvHpOta1Neg = 1.0f / kHpOta1Neg;
constexpr float kHpOta2Rail = (15.0f / 8.0f) * 1.080f;
constexpr float kInvHpOta2Rail = 1.0f / kHpOta2Rail;

constexpr float kLpOta1Rail = (15.0f / 8.0f) * 1.080f;
constexpr float kInvLpOta1Rail = 1.0f / kLpOta1Rail;
constexpr float kLpOta2Pos = (15.0f / 8.0f) * 1.108f;
constexpr float kInvLpOta2Pos = 1.0f / kLpOta2Pos;
constexpr float kLpOta2Neg = (15.0f / 8.0f) * 1.052f;
constexpr float kInvLpOta2Neg = 1.0f / kLpOta2Neg;

// C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback knees:
// Q(u) = u*(1 - (3/5)*u^4 + (2/7)*u^6) has asymptote (24/35)*span, so span = (35/24)*V_diode.
// Matched KLM-307 IC5 (HPF) & IC6 (LPF) anti-parallel silicon pairs prevent limit-cycle DC shift.
constexpr float kHpDiodeSpan = (35.0f / 24.0f) * 1.07f;
constexpr float kInvHpDiodeSpan = 1.0f / kHpDiodeSpan;

constexpr float kLpDiodeSpan = (35.0f / 24.0f) * 0.91f;
constexpr float kInvLpDiodeSpan = 1.0f / kLpDiodeSpan;

constexpr float kDiodeOhmicSlope = 0.024f;
constexpr float kDiodeJunctionWeight = 1.0f - kDiodeOhmicSlope;

// Pure-FPU branchless NaN/Inf/subnormal scrubber (zero GPR<->FPU register stalls; under
// -ffp-contract=off, +1e-18f - 1e-18f flushes |x| < ~3e-26f to bit-exact +0.0f).
inline float scrubFilterState(float x) noexcept {
    const float bounded = clampf(x, -64.0f, 64.0f);
    return (bounded + 1.0e-18f) - 1.0e-18f;
}

inline float mapResonanceToK(float resonance) noexcept {
    const float r = clampf(resonance, 0.0f, 1.0f);
    // Concave taper: liquid vocal Q by 25%, self-oscillation by ~35%, fat silicon howl above 65%
    const float shaped = r * (1.42f - 0.42f * r);
    return kButterworthK + shaped * (kMaxResonanceK - kButterworthK);
}

// Groszkowski reactive-power + dual-OTA compression pitch compensation for HPF & LPF limit cycles
inline float compensateHpSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float excess = k - 2.0f;
    return hz * (1.0f + excess * (0.062f + 0.046f * excess));
}

inline float compensateLpSelfOscPitch(float hz, float k) noexcept {
    if (k <= 2.0f) {
        return hz;
    }
    const float excess = k - 2.0f;
    return hz * (1.0f + excess * (0.068f + 0.053f * excess));
}

// Pure-FPU C^2-continuous LM13600 diode-linearized OTA transconductor:
//   sigma(v)  = vc * (1 - (2/3)*u^2 + (1/5)*u^4),   vc = clamp(v, -negSpan, posSpan)
//   sigma'(v) = (1 - u^2)^2 in [0, 1]
inline SatResult evalOta(float v, float posSpan, float negSpan, float invPosSpan,
                         float invNegSpan) noexcept {
    const float vc = clampf(v, -negSpan, posSpan);
    const float u = vc * ((vc >= 0.0f) ? invPosSpan : invNegSpan);
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {vc * (1.0f - u2 * (0.66666667f - 0.20f * u2)), oneMinusU2 * oneMinusU2};
}

inline SatResult evalOtaSymmetric(float v, float span, float invSpan) noexcept {
    const float vc = clampf(v, -span, span);
    const float u = vc * invSpan;
    const float u2 = u * u;
    const float oneMinusU2 = 1.0f - u2;
    return {vc * (1.0f - u2 * (0.66666667f - 0.20f * u2)), oneMinusU2 * oneMinusU2};
}

// Pure-FPU C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback limiter:
//   Q(u)  = u * (1 - (3/5)*u^4 + (2/7)*u^6)   (zero u^3 term -> flat small-signal deadband!)
//   Q'(u) = (1 - u^2)^2 * (1 + 2*u^2) in [0, 1]
inline SatResult evalSiliconDiode(float w, float span, float invSpan) noexcept {
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

inline float stepHpNonlinear(float x, float& s1, float& s2, float& hist1, float& hist2, float g,
                             float k, float otaScale, float invOtaScale) noexcept {
    const float pos1 = kHpOta1Pos * otaScale;
    const float neg1 = kHpOta1Neg * otaScale;
    const float invPos1 = kInvHpOta1Pos * invOtaScale;
    const float invNeg1 = kInvHpOta1Neg * invOtaScale;
    const float rail2 = kHpOta2Rail * otaScale;
    const float invRail2 = kInvHpOta2Rail * invOtaScale;
    const float dSpan = kHpDiodeSpan * otaScale;
    const float invDSpan = kInvHpDiodeSpan * invOtaScale;

    // Smooth capacitor-sum q = y2 - x = v_C1 + v_C2 never jumps on input steps dx!
    const float xPlusS1 = x + s1;
    float y2 = x + clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    float finalOta2 = -y2;

    for (int iter = 0; iter < 3; ++iter) {
        const SatResult ota2 = evalOtaSymmetric(-y2, rail2, invRail2);
        const float y1 = y2 - s2 - g * ota2.value;

        const SatResult fb = evalSiliconDiode(k * y2, dSpan, invDSpan);
        const SatResult ota1 = evalOta(fb.value - y1, pos1, neg1, invPos1, invNeg1);

        const float fHp = y1 - xPlusS1 - g * ota1.value;
        const float d1 = g * ota1.deriv;
        const float d2 = g * ota2.deriv;
        const float dFb = k * fb.deriv;
        const float dfDy2 = (1.0f + d1) * (1.0f + d2) - d1 * dFb;

        const float dy2 = fHp / dfDy2;
        y2 -= dy2;
        finalOta2 = ota2.value + ota2.deriv * dy2;

        if (absf(dy2) < 1.0e-2f) {
            break;
        }
    }

    const float ic2 = g * finalOta2;
    const float y1 = y2 - s2 - ic2;
    s1 = 2.0f * (y1 - x) - s1;
    s2 = s2 + 2.0f * ic2;
    hist2 = hist1;
    hist1 = y2 - x;
    return y2;
}

inline float stepLpNonlinear(float x, float& s1, float& s2, float& hist1, float& hist2, float g,
                             float k, float otaScale, float invOtaScale) noexcept {
    const float rail1 = kLpOta1Rail * otaScale;
    const float invRail1 = kInvLpOta1Rail * invOtaScale;
    const float pos2 = kLpOta2Pos * otaScale;
    const float neg2 = kLpOta2Neg * otaScale;
    const float invPos2 = kInvLpOta2Pos * invOtaScale;
    const float invNeg2 = kInvLpOta2Neg * invOtaScale;
    const float dSpan = kLpDiodeSpan * otaScale;
    const float invDSpan = kInvLpDiodeSpan * invOtaScale;

    // 2-point extrapolation of continuous Stage-2 differential voltage w = y1 - y2:
    float w = clampf(2.0f * hist1 - hist2, -8.0f, 8.0f);
    float finalOta2 = w;

    for (int iter = 0; iter < 3; ++iter) {
        const SatResult ota2 = evalOta(w, pos2, neg2, invPos2, invNeg2);
        const float y2 = s2 + g * ota2.value;
        const float y1 = w + y2;

        const SatResult ota1 = evalOtaSymmetric(x - y1, rail1, invRail1);
        const SatResult fb = evalSiliconDiode(k * w, dSpan, invDSpan);

        const float fLp = y1 - s1 - g * (ota1.value + fb.value);
        const float d1 = g * ota1.deriv;
        const float d2 = g * ota2.deriv;
        const float dFb = k * fb.deriv;
        const float dfDw = (1.0f + d1) * (1.0f + d2) - g * dFb;

        const float dw = fLp / dfDw;
        w -= dw;
        finalOta2 = ota2.value - ota2.deriv * dw;

        if (absf(dw) < 1.0e-2f) {
            break;
        }
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

} // namespace

void Ms20DualFilter::reset() noexcept {
    hpf_.reset();
    lpf_.reset();
}

Ms20DualFilter::Coeffs Ms20DualFilter::prepareCoeffs(float hpCutoffHz, float hpResonance,
                                                     float lpCutoffHz, float lpResonance,
                                                     float invSampleRate) noexcept {
    Coeffs c{};
    const float hpHz = clampf(hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float lpHz = clampf(lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float hpRes = clampf(hpResonance, 0.0f, 1.0f);
    const float lpRes = clampf(lpResonance, 0.0f, 1.0f);

    // Tight Smart-Park Independent HP & LP Engagement:
    // Transparent linear bypass when parked right at 20 Hz / 20 kHz, waking smoothly into full
    // LM13600 OTA saturation & 1S1588 resonance within a few encoder ticks (HP > 24, LP < 18.8k).
    const float targetKHp = mapResonanceToK(hpRes);
    const float targetKLp = mapResonanceToK(lpRes);
    const float hpActive = smoothstep01((hpHz - 20.0f) * (1.0f / 4.0f));
    const float lpActive = smoothstep01((20000.0f - lpHz) * (1.0f / 1200.0f));

    c.kHp = kButterworthK + hpActive * (targetKHp - kButterworthK);
    c.kLp = kButterworthK + lpActive * (targetKLp - kButterworthK);

    c.hpLinearBypass = (hpActive <= 1.0e-4f);
    if (!c.hpLinearBypass) {
        c.invOtaScaleHp = (hpActive > 0.05f) ? hpActive : 0.05f;
        c.otaScaleHp = 1.0f / c.invOtaScaleHp;
    }

    c.lpLinearBypass = (lpActive <= 1.0e-4f);
    if (!c.lpLinearBypass) {
        c.invOtaScaleLp = (lpActive > 0.05f) ? lpActive : 0.05f;
        c.otaScaleLp = 1.0f / c.invOtaScaleLp;
    }

    // Per-stage KLM-307 passband-vs-resonance dividers: lets each stage's silicon peak tower
    // prominently above the passband and prevents a self-oscillating HPF from swamping LPF.
    const float hpFocus = hpRes * hpActive;
    const float lpFocus = lpRes * lpActive;
    c.hpInScale = 1.0f / (1.0f + 0.70f * hpFocus);
    c.interStageScale = (1.0f + 0.64f * hpFocus) / (1.0f + 0.70f * lpFocus);
    c.lpOutScale = 1.0f + 0.58f * lpFocus;

    const float effectiveHpHz = compensateHpSelfOscPitch(2.5f + hpActive * (hpHz - 2.5f), c.kHp);
    const float effectiveLpHz =
        compensateLpSelfOscPitch(28000.0f + lpActive * (lpHz - 28000.0f), c.kLp);

    c.gHp = fastPrewarpTan(effectiveHpHz, invSampleRate);
    c.gLp = fastPrewarpTan(effectiveLpHz, invSampleRate);

    // Ultrasonic zero-delay loop-gain cap (only active above ~17.5 kHz): guarantees df > 0.12
    // globally across all R^2 even under deeply saturated OTAs.
    if (c.gHp > 0.15f) {
        const float maxSafeKHp = 0.88f * (1.0f + c.gHp) / c.gHp;
        if (c.kHp > maxSafeKHp) {
            c.kHp = maxSafeKHp;
        }
    }
    if (c.gLp > 0.15f) {
        const float maxSafeKLp = 0.88f * (1.0f + 0.90f * c.gLp) / c.gLp;
        if (c.kLp > maxSafeKLp) {
            c.kLp = maxSafeKLp;
        }
    }

    const float onePlusGHp = 1.0f + c.gHp;
    c.invLinDetHp = 1.0f / (onePlusGHp * onePlusGHp - c.gHp * c.kHp);

    const float onePlusGLp = 1.0f + c.gLp;
    c.invLinDetLp = 1.0f / (onePlusGLp * onePlusGLp - c.gLp * c.kLp);
    return c;
}

float Ms20DualFilter::processHighPass(OtaSallenKeyStage& st, float x, const Coeffs& c) noexcept {
    if (c.hpLinearBypass) {
        const float onePlusG = 1.0f + c.gHp;
        const float y2 = (x + st.s1 + onePlusG * st.s2) * c.invLinDetHp;
        const float y1 = onePlusG * y2 - st.s2;
        const float ic1 = c.gHp * (c.kHp * y2 - y1);
        const float ic2 = -c.gHp * y2;
        st.s1 = scrubFilterState(st.s1 + 2.0f * ic1);
        st.s2 = scrubFilterState(st.s2 + 2.0f * ic2);
        st.hist2 = st.hist1;
        st.hist1 = scrubFilterState(y2 - x);
        return y2;
    }

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;
    const float y2 =
        stepHpNonlinear(x, s1, s2, h1, h2, c.gHp, c.kHp, c.otaScaleHp, c.invOtaScaleHp);
    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
    return y2;
}

float Ms20DualFilter::processLowPass(OtaSallenKeyStage& st, float x, const Coeffs& c) noexcept {
    if (c.lpLinearBypass) {
        const float onePlusG = 1.0f + c.gLp;
        const float w = (st.s1 + c.gLp * x - onePlusG * st.s2) * c.invLinDetLp;
        const float ic2 = c.gLp * w;
        const float y2 = st.s2 + ic2;
        const float y1 = w + y2;
        st.s1 = scrubFilterState(2.0f * y1 - st.s1);
        st.s2 = scrubFilterState(st.s2 + 2.0f * ic2);
        st.hist2 = st.hist1;
        st.hist1 = scrubFilterState(w);
        return y2;
    }

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;
    const float y2 =
        stepLpNonlinear(x, s1, s2, h1, h2, c.gLp, c.kLp, c.otaScaleLp, c.invOtaScaleLp);
    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
    return y2;
}

float Ms20DualFilter::processPrepared(float input, const Coeffs& c) noexcept {
    const float cleanIn = clampf(input, -16.0f, 16.0f);
    const float afterHp = processHighPass(hpf_, cleanIn * c.hpInScale, c);
    const float afterLp = processLowPass(lpf_, afterHp * c.interStageScale, c);
    return afterLp * c.lpOutScale;
}

void Ms20DualFilter::processQuadBank4x(
    std::array<Ms20DualFilter, kNumOscillators>& filters,
    std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSamples,
    const std::array<Coeffs, kNumOscillators>& coeffs) noexcept {
    static_assert(kNumOscillators == 4u && kOversampleFactor == 4u, "Expects 4 voices x 4x OS");

    // --- Stage 1: 4-voice High-Pass ---
    const bool allHpBypass = coeffs[0].hpLinearBypass && coeffs[1].hpLinearBypass &&
                             coeffs[2].hpLinearBypass && coeffs[3].hpLinearBypass;
    const bool noneHpBypass = !coeffs[0].hpLinearBypass && !coeffs[1].hpLinearBypass &&
                              !coeffs[2].hpLinearBypass && !coeffs[3].hpLinearBypass;

    if (allHpBypass) {
        alignas(16) std::array<float, 4> s1{filters[0].hpf_.s1, filters[1].hpf_.s1,
                                            filters[2].hpf_.s1, filters[3].hpf_.s1};
        alignas(16) std::array<float, 4> s2{filters[0].hpf_.s2, filters[1].hpf_.s2,
                                            filters[2].hpf_.s2, filters[3].hpf_.s2};
        alignas(16) std::array<float, 4> h1{filters[0].hpf_.hist1, filters[1].hpf_.hist1,
                                            filters[2].hpf_.hist1, filters[3].hpf_.hist1};
        alignas(16) std::array<float, 4> h2{filters[0].hpf_.hist2, filters[1].hpf_.hist2,
                                            filters[2].hpf_.hist2, filters[3].hpf_.hist2};
        alignas(16) const std::array<float, 4> g{coeffs[0].gHp, coeffs[1].gHp, coeffs[2].gHp,
                                                 coeffs[3].gHp};
        alignas(16) const std::array<float, 4> k{coeffs[0].kHp, coeffs[1].kHp, coeffs[2].kHp,
                                                 coeffs[3].kHp};
        alignas(16) const std::array<float, 4> invDet{coeffs[0].invLinDetHp, coeffs[1].invLinDetHp,
                                                      coeffs[2].invLinDetHp, coeffs[3].invLinDetHp};
        alignas(16) const std::array<float, 4> inScale{coeffs[0].hpInScale, coeffs[1].hpInScale,
                                                       coeffs[2].hpInScale, coeffs[3].hpInScale};

        for (size_t sub = 0u; sub < 4u; ++sub) {
            for (size_t v = 0u; v < 4u; ++v) {
                const float x = clampf(voiceSamples[v][sub], -16.0f, 16.0f) * inScale[v];
                const float onePlusG = 1.0f + g[v];
                const float y2 = (x + s1[v] + onePlusG * s2[v]) * invDet[v];
                const float y1 = onePlusG * y2 - s2[v];
                const float ic1 = g[v] * (k[v] * y2 - y1);
                const float ic2 = -g[v] * y2;
                s1[v] = scrubFilterState(s1[v] + 2.0f * ic1);
                s2[v] = scrubFilterState(s2[v] + 2.0f * ic2);
                h2[v] = h1[v];
                h1[v] = scrubFilterState(y2 - x);
                voiceSamples[v][sub] = y2;
            }
        }
        for (size_t v = 0u; v < 4u; ++v) {
            filters[v].hpf_.s1 = s1[v];
            filters[v].hpf_.s2 = s2[v];
            filters[v].hpf_.hist1 = h1[v];
            filters[v].hpf_.hist2 = h2[v];
        }
    } else if (noneHpBypass) {
        alignas(16) std::array<float, 4> s1{filters[0].hpf_.s1, filters[1].hpf_.s1,
                                            filters[2].hpf_.s1, filters[3].hpf_.s1};
        alignas(16) std::array<float, 4> s2{filters[0].hpf_.s2, filters[1].hpf_.s2,
                                            filters[2].hpf_.s2, filters[3].hpf_.s2};
        alignas(16) std::array<float, 4> h1{filters[0].hpf_.hist1, filters[1].hpf_.hist1,
                                            filters[2].hpf_.hist1, filters[3].hpf_.hist1};
        alignas(16) std::array<float, 4> h2{filters[0].hpf_.hist2, filters[1].hpf_.hist2,
                                            filters[2].hpf_.hist2, filters[3].hpf_.hist2};
        alignas(16) const std::array<float, 4> g{coeffs[0].gHp, coeffs[1].gHp, coeffs[2].gHp,
                                                 coeffs[3].gHp};
        alignas(16) const std::array<float, 4> k{coeffs[0].kHp, coeffs[1].kHp, coeffs[2].kHp,
                                                 coeffs[3].kHp};
        alignas(16) const std::array<float, 4> inScale{coeffs[0].hpInScale, coeffs[1].hpInScale,
                                                       coeffs[2].hpInScale, coeffs[3].hpInScale};
        alignas(16) std::array<float, 4> pos1{};
        alignas(16) std::array<float, 4> neg1{};
        alignas(16) std::array<float, 4> invPos1{};
        alignas(16) std::array<float, 4> invNeg1{};
        alignas(16) std::array<float, 4> rail2{};
        alignas(16) std::array<float, 4> invRail2{};
        alignas(16) std::array<float, 4> dSpan{};
        alignas(16) std::array<float, 4> invDSpan{};
        for (size_t v = 0u; v < 4u; ++v) {
            pos1[v] = kHpOta1Pos * coeffs[v].otaScaleHp;
            neg1[v] = kHpOta1Neg * coeffs[v].otaScaleHp;
            invPos1[v] = kInvHpOta1Pos * coeffs[v].invOtaScaleHp;
            invNeg1[v] = kInvHpOta1Neg * coeffs[v].invOtaScaleHp;
            rail2[v] = kHpOta2Rail * coeffs[v].otaScaleHp;
            invRail2[v] = kInvHpOta2Rail * coeffs[v].invOtaScaleHp;
            dSpan[v] = kHpDiodeSpan * coeffs[v].otaScaleHp;
            invDSpan[v] = kInvHpDiodeSpan * coeffs[v].invOtaScaleHp;
        }

        for (size_t sub = 0u; sub < 4u; ++sub) {
            alignas(16) std::array<float, 4> x{};
            alignas(16) std::array<float, 4> y2{};
            alignas(16) std::array<float, 4> dy2{};
            alignas(16) std::array<float, 4> finalOta2{};

            // Auto-vectorized 4-voice SIMD Iteration 0 (hides fdiv latency 4x across voices!)
            for (size_t v = 0u; v < 4u; ++v) {
                x[v] = clampf(voiceSamples[v][sub], -16.0f, 16.0f) * inScale[v];
                y2[v] = x[v] + clampf(2.0f * h1[v] - h2[v], -8.0f, 8.0f);
                const SatResult ota2 = evalOtaSymmetric(-y2[v], rail2[v], invRail2[v]);
                const float y1 = y2[v] - s2[v] - g[v] * ota2.value;
                const SatResult fb = evalSiliconDiode(k[v] * y2[v], dSpan[v], invDSpan[v]);
                const SatResult ota1 =
                    evalOta(fb.value - y1, pos1[v], neg1[v], invPos1[v], invNeg1[v]);

                const float fHp = y1 - (x[v] + s1[v]) - g[v] * ota1.value;
                const float d1 = g[v] * ota1.deriv;
                const float d2 = g[v] * ota2.deriv;
                const float df = (1.0f + d1) * (1.0f + d2) - d1 * (k[v] * fb.deriv);
                dy2[v] = fHp / df;
                y2[v] -= dy2[v];
                finalOta2[v] = ota2.value + ota2.deriv * dy2[v];
            }

            if (!(absf(dy2[0]) < 1.0e-2f && absf(dy2[1]) < 1.0e-2f && absf(dy2[2]) < 1.0e-2f &&
                  absf(dy2[3]) < 1.0e-2f)) {
                for (size_t v = 0u; v < 4u; ++v) {
                    if (!(absf(dy2[v]) < 1.0e-2f)) {
                        for (int iter = 1; iter < 3; ++iter) {
                            const SatResult ota2 = evalOtaSymmetric(-y2[v], rail2[v], invRail2[v]);
                            const float y1 = y2[v] - s2[v] - g[v] * ota2.value;
                            const SatResult fb =
                                evalSiliconDiode(k[v] * y2[v], dSpan[v], invDSpan[v]);
                            const SatResult ota1 =
                                evalOta(fb.value - y1, pos1[v], neg1[v], invPos1[v], invNeg1[v]);
                            const float fHp = y1 - (x[v] + s1[v]) - g[v] * ota1.value;
                            const float d1 = g[v] * ota1.deriv;
                            const float d2 = g[v] * ota2.deriv;
                            const float step =
                                fHp / ((1.0f + d1) * (1.0f + d2) - d1 * (k[v] * fb.deriv));
                            y2[v] -= step;
                            finalOta2[v] = ota2.value + ota2.deriv * step;
                            if (absf(step) < 1.0e-2f) {
                                break;
                            }
                        }
                    }
                }
            }

            for (size_t v = 0u; v < 4u; ++v) {
                const float ic2 = g[v] * finalOta2[v];
                const float y1 = y2[v] - s2[v] - ic2;
                s1[v] = scrubFilterState(2.0f * (y1 - x[v]) - s1[v]);
                s2[v] = scrubFilterState(s2[v] + 2.0f * ic2);
                h2[v] = scrubFilterState(h1[v]);
                h1[v] = scrubFilterState(y2[v] - x[v]);
                voiceSamples[v][sub] = y2[v];
            }
        }
        for (size_t v = 0u; v < 4u; ++v) {
            filters[v].hpf_.s1 = s1[v];
            filters[v].hpf_.s2 = s2[v];
            filters[v].hpf_.hist1 = h1[v];
            filters[v].hpf_.hist2 = h2[v];
        }
    } else {
        for (size_t v = 0u; v < 4u; ++v) {
            const float inScale = coeffs[v].hpInScale;
            for (size_t sub = 0u; sub < 4u; ++sub) {
                const float cleanIn = clampf(voiceSamples[v][sub], -16.0f, 16.0f) * inScale;
                voiceSamples[v][sub] = processHighPass(filters[v].hpf_, cleanIn, coeffs[v]);
            }
        }
    }

    // --- Stage 2: 4-voice Low-Pass ---
    const bool allLpBypass = coeffs[0].lpLinearBypass && coeffs[1].lpLinearBypass &&
                             coeffs[2].lpLinearBypass && coeffs[3].lpLinearBypass;
    const bool noneLpBypass = !coeffs[0].lpLinearBypass && !coeffs[1].lpLinearBypass &&
                              !coeffs[2].lpLinearBypass && !coeffs[3].lpLinearBypass;

    if (allLpBypass) {
        alignas(16) std::array<float, 4> s1{filters[0].lpf_.s1, filters[1].lpf_.s1,
                                            filters[2].lpf_.s1, filters[3].lpf_.s1};
        alignas(16) std::array<float, 4> s2{filters[0].lpf_.s2, filters[1].lpf_.s2,
                                            filters[2].lpf_.s2, filters[3].lpf_.s2};
        alignas(16) std::array<float, 4> h1{filters[0].lpf_.hist1, filters[1].lpf_.hist1,
                                            filters[2].lpf_.hist1, filters[3].lpf_.hist1};
        alignas(16) std::array<float, 4> h2{filters[0].lpf_.hist2, filters[1].lpf_.hist2,
                                            filters[2].lpf_.hist2, filters[3].lpf_.hist2};
        alignas(16) const std::array<float, 4> g{coeffs[0].gLp, coeffs[1].gLp, coeffs[2].gLp,
                                                 coeffs[3].gLp};
        alignas(16) const std::array<float, 4> invDet{coeffs[0].invLinDetLp, coeffs[1].invLinDetLp,
                                                      coeffs[2].invLinDetLp, coeffs[3].invLinDetLp};
        alignas(16) const std::array<float, 4> interScale{
            coeffs[0].interStageScale, coeffs[1].interStageScale, coeffs[2].interStageScale,
            coeffs[3].interStageScale};
        alignas(16) const std::array<float, 4> outScale{coeffs[0].lpOutScale, coeffs[1].lpOutScale,
                                                        coeffs[2].lpOutScale, coeffs[3].lpOutScale};

        for (size_t sub = 0u; sub < 4u; ++sub) {
            for (size_t v = 0u; v < 4u; ++v) {
                const float x = voiceSamples[v][sub] * interScale[v];
                const float w = (s1[v] + g[v] * x - (1.0f + g[v]) * s2[v]) * invDet[v];
                const float ic2 = g[v] * w;
                const float y2 = s2[v] + ic2;
                s1[v] = scrubFilterState(2.0f * (w + y2) - s1[v]);
                s2[v] = scrubFilterState(s2[v] + 2.0f * ic2);
                h2[v] = h1[v];
                h1[v] = scrubFilterState(w);
                voiceSamples[v][sub] = y2 * outScale[v];
            }
        }
        for (size_t v = 0u; v < 4u; ++v) {
            filters[v].lpf_.s1 = s1[v];
            filters[v].lpf_.s2 = s2[v];
            filters[v].lpf_.hist1 = h1[v];
            filters[v].lpf_.hist2 = h2[v];
        }
    } else if (noneLpBypass) {
        alignas(16) std::array<float, 4> s1{filters[0].lpf_.s1, filters[1].lpf_.s1,
                                            filters[2].lpf_.s1, filters[3].lpf_.s1};
        alignas(16) std::array<float, 4> s2{filters[0].lpf_.s2, filters[1].lpf_.s2,
                                            filters[2].lpf_.s2, filters[3].lpf_.s2};
        alignas(16) std::array<float, 4> h1{filters[0].lpf_.hist1, filters[1].lpf_.hist1,
                                            filters[2].lpf_.hist1, filters[3].lpf_.hist1};
        alignas(16) std::array<float, 4> h2{filters[0].lpf_.hist2, filters[1].lpf_.hist2,
                                            filters[2].lpf_.hist2, filters[3].lpf_.hist2};
        alignas(16) const std::array<float, 4> g{coeffs[0].gLp, coeffs[1].gLp, coeffs[2].gLp,
                                                 coeffs[3].gLp};
        alignas(16) const std::array<float, 4> k{coeffs[0].kLp, coeffs[1].kLp, coeffs[2].kLp,
                                                 coeffs[3].kLp};
        alignas(16) const std::array<float, 4> interScale{
            coeffs[0].interStageScale, coeffs[1].interStageScale, coeffs[2].interStageScale,
            coeffs[3].interStageScale};
        alignas(16) const std::array<float, 4> outScale{coeffs[0].lpOutScale, coeffs[1].lpOutScale,
                                                        coeffs[2].lpOutScale, coeffs[3].lpOutScale};
        alignas(16) std::array<float, 4> rail1{};
        alignas(16) std::array<float, 4> invRail1{};
        alignas(16) std::array<float, 4> pos2{};
        alignas(16) std::array<float, 4> neg2{};
        alignas(16) std::array<float, 4> invPos2{};
        alignas(16) std::array<float, 4> invNeg2{};
        alignas(16) std::array<float, 4> dSpan{};
        alignas(16) std::array<float, 4> invDSpan{};
        for (size_t v = 0u; v < 4u; ++v) {
            rail1[v] = kLpOta1Rail * coeffs[v].otaScaleLp;
            invRail1[v] = kInvLpOta1Rail * coeffs[v].invOtaScaleLp;
            pos2[v] = kLpOta2Pos * coeffs[v].otaScaleLp;
            neg2[v] = kLpOta2Neg * coeffs[v].otaScaleLp;
            invPos2[v] = kInvLpOta2Pos * coeffs[v].invOtaScaleLp;
            invNeg2[v] = kInvLpOta2Neg * coeffs[v].invOtaScaleLp;
            dSpan[v] = kLpDiodeSpan * coeffs[v].otaScaleLp;
            invDSpan[v] = kInvLpDiodeSpan * coeffs[v].invOtaScaleLp;
        }

        for (size_t sub = 0u; sub < 4u; ++sub) {
            alignas(16) std::array<float, 4> x{};
            alignas(16) std::array<float, 4> w{};
            alignas(16) std::array<float, 4> dw{};
            alignas(16) std::array<float, 4> finalOta2{};

            // Auto-vectorized 4-voice SIMD Iteration 0
            for (size_t v = 0u; v < 4u; ++v) {
                x[v] = voiceSamples[v][sub] * interScale[v];
                w[v] = clampf(2.0f * h1[v] - h2[v], -8.0f, 8.0f);
                const SatResult ota2 = evalOta(w[v], pos2[v], neg2[v], invPos2[v], invNeg2[v]);
                const float y2 = s2[v] + g[v] * ota2.value;
                const float y1 = w[v] + y2;
                const SatResult ota1 = evalOtaSymmetric(x[v] - y1, rail1[v], invRail1[v]);
                const SatResult fb = evalSiliconDiode(k[v] * w[v], dSpan[v], invDSpan[v]);

                const float fLp = y1 - s1[v] - g[v] * (ota1.value + fb.value);
                const float d1 = g[v] * ota1.deriv;
                const float d2 = g[v] * ota2.deriv;
                const float df = (1.0f + d1) * (1.0f + d2) - g[v] * (k[v] * fb.deriv);
                dw[v] = fLp / df;
                w[v] -= dw[v];
                finalOta2[v] = ota2.value - ota2.deriv * dw[v];
            }

            if (!(absf(dw[0]) < 1.0e-2f && absf(dw[1]) < 1.0e-2f && absf(dw[2]) < 1.0e-2f &&
                  absf(dw[3]) < 1.0e-2f)) {
                for (size_t v = 0u; v < 4u; ++v) {
                    if (!(absf(dw[v]) < 1.0e-2f)) {
                        for (int iter = 1; iter < 3; ++iter) {
                            const SatResult ota2 =
                                evalOta(w[v], pos2[v], neg2[v], invPos2[v], invNeg2[v]);
                            const float y2 = s2[v] + g[v] * ota2.value;
                            const float y1 = w[v] + y2;
                            const SatResult ota1 =
                                evalOtaSymmetric(x[v] - y1, rail1[v], invRail1[v]);
                            const SatResult fb =
                                evalSiliconDiode(k[v] * w[v], dSpan[v], invDSpan[v]);
                            const float fLp = y1 - s1[v] - g[v] * (ota1.value + fb.value);
                            const float d1 = g[v] * ota1.deriv;
                            const float d2 = g[v] * ota2.deriv;
                            const float step =
                                fLp / ((1.0f + d1) * (1.0f + d2) - g[v] * (k[v] * fb.deriv));
                            w[v] -= step;
                            finalOta2[v] = ota2.value - ota2.deriv * step;
                            if (absf(step) < 1.0e-2f) {
                                break;
                            }
                        }
                    }
                }
            }

            for (size_t v = 0u; v < 4u; ++v) {
                const float ic2 = g[v] * finalOta2[v];
                const float y2 = s2[v] + ic2;
                s1[v] = scrubFilterState(2.0f * (w[v] + y2) - s1[v]);
                s2[v] = scrubFilterState(s2[v] + 2.0f * ic2);
                h2[v] = scrubFilterState(h1[v]);
                h1[v] = scrubFilterState(w[v]);
                voiceSamples[v][sub] = y2 * outScale[v];
            }
        }
        for (size_t v = 0u; v < 4u; ++v) {
            filters[v].lpf_.s1 = s1[v];
            filters[v].lpf_.s2 = s2[v];
            filters[v].lpf_.hist1 = h1[v];
            filters[v].lpf_.hist2 = h2[v];
        }
    } else {
        for (size_t v = 0u; v < 4u; ++v) {
            const float interScale = coeffs[v].interStageScale;
            const float outScale = coeffs[v].lpOutScale;
            for (size_t sub = 0u; sub < 4u; ++sub) {
                const float y2 =
                    processLowPass(filters[v].lpf_, voiceSamples[v][sub] * interScale, coeffs[v]);
                voiceSamples[v][sub] = y2 * outScale;
            }
        }
    }
}

} // namespace ruido
