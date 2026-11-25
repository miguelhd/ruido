#include "dsp/SallenKeyFilter.h"

#include "dsp/SallenKeyMath.h"

namespace ruido {

void Ms20DualFilter::reset() noexcept {
    hpf_.reset();
    lpf_.reset();
}

Ms20DualFilter::Coeffs Ms20DualFilter::prepareCoeffs(float hpCutoffHz, float hpResonance,
                                                     float lpCutoffHz, float lpResonance,
                                                     float invSampleRate, float preDrive) noexcept {
    Coeffs c{};
    const float hpHz = clampf(hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float lpHz = clampf(lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float hpRes = clampf(hpResonance, 0.0f, 1.0f);
    const float lpRes = clampf(lpResonance, 0.0f, 1.0f);

    c.preDrive = clampf(preDrive, 1.0f, 6.0f);
    c.driveBlend = smoothstep01((c.preDrive - 1.0f) * (1.0f / 3.5f));

    // Zero Smart-Park Resonance Muting:
    // Linear bypass engages ONLY when parked right at 2 Hz / 28 kHz with 0% RES and 1.0x drive.
    // Any resonance (> 0%) or pre-filter overdrive (> 1.0x) wakes 100% nonlinear silicon across
    // the entire 2 Hz - 28 kHz continuum.
    const float hpResWake = smoothstep01(hpRes * (1.0f / 0.015f));
    const float lpResWake = smoothstep01(lpRes * (1.0f / 0.015f));
    const float driveWake = smoothstep01((c.preDrive - 1.0f) * (1.0f / 0.06f));
    const float hpFreqWake = smoothstep01((hpHz - kFilterCutoffMinHz) * (1.0f / 1.5f));
    const float lpFreqWake = smoothstep01((kFilterCutoffMaxHz - lpHz) * (1.0f / 900.0f));

    const float hpWakeMax = (hpResWake > driveWake) ? hpResWake : driveWake;
    const float lpWakeMax = lpResWake;
    const float hpActive = clampf(hpFreqWake + (1.0f - hpFreqWake) * hpWakeMax, 0.0f, 1.0f);
    const float lpActive = clampf(lpFreqWake + (1.0f - lpFreqWake) * lpWakeMax, 0.0f, 1.0f);

    const float hpChaos = computeKorg35Chaos(hpRes);
    const float lpChaos = computeKorg35Chaos(lpRes);

    c.hpLinearBypass = (hpActive <= 1.0e-4f);
    if (!c.hpLinearBypass) {
        c.kHp = applyOtaHfLoopTaper(mapResonanceToK(hpRes), hpHz);
        c.invOtaScaleHp = (hpActive > 0.05f) ? hpActive : 0.05f;
        c.otaScaleHp = 1.0f / c.invOtaScaleHp;
        c.hpPos1 = kHpOta1Pos * c.otaScaleHp;
        c.hpNeg1 = kHpOta1Neg * c.otaScaleHp;
        c.invHpPos1 = kInvHpOta1Pos * c.invOtaScaleHp;
        c.invHpNeg1 = kInvHpOta1Neg * c.invOtaScaleHp;
        c.hpRail2 = kHpOta2Rail * c.otaScaleHp;
        c.invHpRail2 = kInvHpOta2Rail * c.invOtaScaleHp;
        c.hpDiodeSpan = kHpDiodeSpan * c.otaScaleHp * (1.0f - 0.12f * hpChaos);
        c.invHpDiodeSpan = 1.0f / c.hpDiodeSpan;
        const float effectiveHpHz = compensateHpSelfOscPitch(hpHz, c.kHp);
        c.gHp = fastPrewarpTan(effectiveHpHz, invSampleRate);
        if (c.gHp * (c.kHp - 1.0f) > 0.88f) {
            c.kHp = 1.0f + 0.88f / c.gHp;
        }
        const float onePlusGHp = 1.0f + c.gHp;
        c.invLinDetHp = 1.0f / (onePlusGHp * onePlusGHp - c.gHp * c.kHp);
    }

    c.lpLinearBypass = (lpActive <= 1.0e-4f);
    if (!c.lpLinearBypass) {
        c.kLp = applyOtaHfLoopTaper(mapResonanceToK(lpRes), lpHz);
        const float invOtaLp = (lpActive > 0.05f) ? lpActive : 0.05f;
        const float otaScaleLp = 1.0f / invOtaLp;
        c.lpRail1 = kLpOta1Rail * otaScaleLp;
        c.invLpRail1 = 1.0f / c.lpRail1;
        c.lpPos2 = kLpOta2Pos * otaScaleLp * (1.0f + 0.14f * lpChaos);
        c.lpNeg2 = kLpOta2Neg * otaScaleLp * (1.0f - 0.16f * lpChaos);
        c.invLpPos2 = 1.0f / c.lpPos2;
        c.invLpNeg2 = 1.0f / c.lpNeg2;
        c.lpDiodeSpan = kLpDiodeSpan * otaScaleLp * (1.0f - 0.12f * lpChaos);
        c.invLpDiodeSpan = 1.0f / c.lpDiodeSpan;
        const float effectiveLpHz = compensateLpSelfOscPitch(lpHz, c.kLp);
        c.gLp = fastPrewarpTan(effectiveLpHz, invSampleRate);
        if (c.gLp * (c.kLp - 0.72f) > 0.88f) {
            c.kLp = 0.72f + 0.88f / c.gLp;
        }
        const float onePlusGLp = 1.0f + c.gLp;
        c.invLinDetLp = 1.0f / (onePlusGLp * onePlusGLp - c.gLp * c.kLp);
    }

    // Rev 2 (0-60%) balances passband vs resonance; Korg35 (60-100%) unpads HPF -> LPF coupling
    // so dual-stage self-oscillation slams LPF OTAs into raw intermodulation screaming.
    const float hpRev2 = clampf(hpRes, 0.0f, 0.60f) * hpActive;
    const float lpRev2 = clampf(lpRes, 0.0f, 0.60f) * lpActive;
    c.hpInScale = (hpRev2 > 0.0f) ? (1.0f / (1.0f + 0.68f * hpRev2)) : 1.0f;
    const float baseInter = (hpRev2 == 0.0f && lpRev2 == 0.0f)
                                ? 1.0f
                                : ((1.0f + 0.64f * hpRev2) / (1.0f + 0.68f * lpRev2));
    c.interStageScale = baseInter * (1.0f + 0.52f * hpChaos + 0.18f * lpChaos);
    c.lpOutScale = 1.0f + 0.56f * lpRev2 + 0.42f * lpChaos;
    return c;
}

// Parked stages (HPF 2 Hz / LPF 28 kHz at 0% RES & 1.0x drive) are fully open, so they pass
// straight through at zero cost. States are kept at the exact steady state of the open stage (HPF
// at rest, LPF integrators tracking the input) so un-parking resumes clicklessly.
RUIDO_ALWAYS_INLINE float Ms20DualFilter::processHighPass(OtaSallenKeyStage& st, float x,
                                                          const Coeffs& c) noexcept {
    if (c.hpLinearBypass) {
        st.reset();
        return x;
    }

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;
    const float y2 = stepHpNonlinearPrecomputed(x, s1, s2, h1, h2, c.gHp, c.kHp, c.hpPos1, c.hpNeg1,
                                                c.invHpPos1, c.invHpNeg1, c.hpRail2, c.invHpRail2,
                                                c.hpDiodeSpan, c.invHpDiodeSpan);
    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
    return y2;
}

RUIDO_ALWAYS_INLINE float Ms20DualFilter::processLowPass(OtaSallenKeyStage& st, float x,
                                                         const Coeffs& c) noexcept {
    if (c.lpLinearBypass) {
        const float tracked = scrubFilterState(x);
        st.s1 = tracked;
        st.s2 = tracked;
        st.hist1 = 0.0f;
        st.hist2 = 0.0f;
        return x;
    }

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;
    const float y2 =
        stepLpNonlinear(x, s1, s2, h1, h2, c.gLp, c.kLp, c.lpRail1, c.invLpRail1, c.lpPos2,
                        c.lpNeg2, c.invLpPos2, c.invLpNeg2, c.lpDiodeSpan, c.invLpDiodeSpan);
    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
    return y2;
}

float Ms20DualFilter::processPrepared(float input, const Coeffs& c) noexcept {
    const float drivenIn = applyMs20InputOverdrive(input, c.preDrive, c.driveBlend);
    const float afterHp = processHighPass(hpf_, drivenIn * c.hpInScale, c);
    const float afterLp = processLowPass(lpf_, afterHp * c.interStageScale, c);
    return afterLp * c.lpOutScale;
}

RUIDO_NOINLINE void
Ms20DualFilter::processVoiceHpSpan4x(OtaSallenKeyStage& st,
                                     std::array<float, kOversampleFactor>* frames4x,
                                     size_t numFrames, const Coeffs& c) noexcept {
    const float inScale = c.hpInScale;
    const float preDrive = c.preDrive;
    const float driveBlend = c.driveBlend;
    const bool hasDrive = (driveBlend > 0.0f);

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;

    float* smp = frames4x[0].data();
    const size_t total = numFrames * kOversampleFactor;
    if (c.hpLinearBypass) {
        if (hasDrive) {
            for (size_t idx = 0u; idx < total; ++idx) {
                smp[idx] = applyMs20InputOverdrive(smp[idx], preDrive, driveBlend) * inScale;
            }
        } else if (inScale != 1.0f) {
            for (size_t idx = 0u; idx < total; ++idx) {
                smp[idx] = clampf(smp[idx], -16.0f, 16.0f) * inScale;
            }
        } else {
            for (size_t idx = 0u; idx < total; ++idx) {
                smp[idx] = clampf(smp[idx], -16.0f, 16.0f);
            }
        }
        st.reset();
        return;
    }
    if (hasDrive) {
        for (size_t idx = 0u; idx < total; ++idx) {
            const float rawIn = applyMs20InputOverdrive(smp[idx], preDrive, driveBlend);
            smp[idx] = stepHpNonlinearPrecomputed(
                rawIn * inScale, s1, s2, h1, h2, c.gHp, c.kHp, c.hpPos1, c.hpNeg1, c.invHpPos1,
                c.invHpNeg1, c.hpRail2, c.invHpRail2, c.hpDiodeSpan, c.invHpDiodeSpan);
        }
    } else {
        for (size_t idx = 0u; idx < total; ++idx) {
            const float rawIn = clampf(smp[idx], -16.0f, 16.0f);
            smp[idx] = stepHpNonlinearPrecomputed(
                rawIn * inScale, s1, s2, h1, h2, c.gHp, c.kHp, c.hpPos1, c.hpNeg1, c.invHpPos1,
                c.invHpNeg1, c.hpRail2, c.invHpRail2, c.hpDiodeSpan, c.invHpDiodeSpan);
        }
    }

    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
}

RUIDO_NOINLINE void
Ms20DualFilter::processVoiceLpSpan4x(OtaSallenKeyStage& st,
                                     std::array<float, kOversampleFactor>* frames4x,
                                     size_t numFrames, const Coeffs& c) noexcept {
    const float interScale = c.interStageScale;
    const float outScale = c.lpOutScale;

    float* smp = frames4x[0].data();
    const size_t total = numFrames * kOversampleFactor;

    if (c.lpLinearBypass) {
        const float lastX = smp[total - 1u] * interScale;
        const float totalScale = interScale * outScale;
        if (totalScale != 1.0f) {
            for (size_t idx = 0u; idx < total; ++idx) {
                smp[idx] = (smp[idx] * interScale) * outScale;
            }
        }
        const float tracked = scrubFilterState(lastX);
        st.s1 = tracked;
        st.s2 = tracked;
        st.hist1 = 0.0f;
        st.hist2 = 0.0f;
        return;
    }

    float s1 = st.s1;
    float s2 = st.s2;
    float h1 = st.hist1;
    float h2 = st.hist2;
    for (size_t idx = 0u; idx < total; ++idx) {
        const float y2 = stepLpNonlinear(smp[idx] * interScale, s1, s2, h1, h2, c.gLp, c.kLp,
                                         c.lpRail1, c.invLpRail1, c.lpPos2, c.lpNeg2, c.invLpPos2,
                                         c.invLpNeg2, c.lpDiodeSpan, c.invLpDiodeSpan);
        smp[idx] = y2 * outScale;
    }

    st.s1 = scrubFilterState(s1);
    st.s2 = scrubFilterState(s2);
    st.hist1 = scrubFilterState(h1);
    st.hist2 = scrubFilterState(h2);
}

void Ms20DualFilter::processSpan4x(std::array<float, kOversampleFactor>* frames4x, size_t numFrames,
                                   const Coeffs& c) noexcept {
    if (frames4x == nullptr || numFrames == 0u) {
        return;
    }
    processVoiceHpSpan4x(hpf_, frames4x, numFrames, c);
    processVoiceLpSpan4x(lpf_, frames4x, numFrames, c);
}

void Ms20DualFilter::processQuadBank4x(
    std::array<Ms20DualFilter, kNumOscillators>& filters,
    std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSamples,
    const std::array<Coeffs, kNumOscillators>& coeffs) noexcept {
    static_assert(kNumOscillators == 4u && kOversampleFactor == 4u, "Expects 4 voices x 4x OS");
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__) ||                    \
    !(defined(__SSE__) || defined(__ARM_NEON) || defined(__wasm__))
    for (size_t v = 0u; v < 4u; ++v) {
        filters[v].processSpan4x(&voiceSamples[v], 1u, coeffs[v]);
    }
    return;
#else
    // --- Stage 1: 4-voice High-Pass ---
    const bool allHpBypass = coeffs[0].hpLinearBypass && coeffs[1].hpLinearBypass &&
                             coeffs[2].hpLinearBypass && coeffs[3].hpLinearBypass;
    const bool noneHpBypass = !coeffs[0].hpLinearBypass && !coeffs[1].hpLinearBypass &&
                              !coeffs[2].hpLinearBypass && !coeffs[3].hpLinearBypass;

    if (allHpBypass) {
        for (size_t v = 0u; v < 4u; ++v) {
            const float inScale = coeffs[v].hpInScale;
            for (size_t sub = 0u; sub < 4u; ++sub) {
                voiceSamples[v][sub] =
                    applyMs20InputOverdrive(voiceSamples[v][sub], coeffs[v].preDrive,
                                            coeffs[v].driveBlend) *
                    inScale;
            }
            filters[v].hpf_.reset();
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
            pos1[v] = coeffs[v].hpPos1;
            neg1[v] = coeffs[v].hpNeg1;
            invPos1[v] = coeffs[v].invHpPos1;
            invNeg1[v] = coeffs[v].invHpNeg1;
            rail2[v] = coeffs[v].hpRail2;
            invRail2[v] = coeffs[v].invHpRail2;
            dSpan[v] = coeffs[v].hpDiodeSpan;
            invDSpan[v] = coeffs[v].invHpDiodeSpan;
        }

        for (size_t sub = 0u; sub < 4u; ++sub) {
            alignas(16) std::array<float, 4> x{};
            alignas(16) std::array<float, 4> y2{};
            alignas(16) std::array<float, 4> dy2{};
            alignas(16) std::array<float, 4> finalOta2{};

            for (size_t v = 0u; v < 4u; ++v) {
                const float driven = applyMs20InputOverdrive(
                    voiceSamples[v][sub], coeffs[v].preDrive, coeffs[v].driveBlend);
                x[v] = driven * inScale[v];
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
                dy2[v] = fHp * fastRecipPos(df);
                y2[v] -= dy2[v];
                finalOta2[v] = ota2.value + ota2.deriv * dy2[v];
            }

            if (!(absf(dy2[0]) < 1.0e-2f && absf(dy2[1]) < 1.0e-2f && absf(dy2[2]) < 1.0e-2f &&
                  absf(dy2[3]) < 1.0e-2f)) {
                for (size_t v = 0u; v < 4u; ++v) {
                    if (!(absf(dy2[v]) < 1.0e-2f)) {
                        const SatResult ota2 = evalOtaSymmetric(-y2[v], rail2[v], invRail2[v]);
                        const float y1 = y2[v] - s2[v] - g[v] * ota2.value;
                        const SatResult fb = evalSiliconDiode(k[v] * y2[v], dSpan[v], invDSpan[v]);
                        const SatResult ota1 =
                            evalOta(fb.value - y1, pos1[v], neg1[v], invPos1[v], invNeg1[v]);
                        const float fHp = y1 - (x[v] + s1[v]) - g[v] * ota1.value;
                        const float d1 = g[v] * ota1.deriv;
                        const float d2 = g[v] * ota2.deriv;
                        const float step =
                            fHp * fastRecipPos((1.0f + d1) * (1.0f + d2) - d1 * (k[v] * fb.deriv));
                        y2[v] -= step;
                        finalOta2[v] = ota2.value + ota2.deriv * step;
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
                const float driven = applyMs20InputOverdrive(
                    voiceSamples[v][sub], coeffs[v].preDrive, coeffs[v].driveBlend);
                voiceSamples[v][sub] =
                    processHighPass(filters[v].hpf_, driven * inScale, coeffs[v]);
            }
        }
    }

    // --- Stage 2: 4-voice Low-Pass ---
    const bool allLpBypass = coeffs[0].lpLinearBypass && coeffs[1].lpLinearBypass &&
                             coeffs[2].lpLinearBypass && coeffs[3].lpLinearBypass;
    const bool noneLpBypass = !coeffs[0].lpLinearBypass && !coeffs[1].lpLinearBypass &&
                              !coeffs[2].lpLinearBypass && !coeffs[3].lpLinearBypass;

    if (allLpBypass) {
        for (size_t v = 0u; v < 4u; ++v) {
            const float interScale = coeffs[v].interStageScale;
            const float outScale = coeffs[v].lpOutScale;
            float lastX = filters[v].lpf_.s1;
            for (size_t sub = 0u; sub < 4u; ++sub) {
                lastX = voiceSamples[v][sub] * interScale;
                voiceSamples[v][sub] = lastX * outScale;
            }
            const float tracked = scrubFilterState(lastX);
            filters[v].lpf_ = {tracked, tracked, 0.0f, 0.0f};
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
        alignas(16) const std::array<float, 4> rail1{coeffs[0].lpRail1, coeffs[1].lpRail1,
                                                     coeffs[2].lpRail1, coeffs[3].lpRail1};
        alignas(16) const std::array<float, 4> invRail1{coeffs[0].invLpRail1, coeffs[1].invLpRail1,
                                                        coeffs[2].invLpRail1, coeffs[3].invLpRail1};
        alignas(16) const std::array<float, 4> pos2{coeffs[0].lpPos2, coeffs[1].lpPos2,
                                                    coeffs[2].lpPos2, coeffs[3].lpPos2};
        alignas(16) const std::array<float, 4> neg2{coeffs[0].lpNeg2, coeffs[1].lpNeg2,
                                                    coeffs[2].lpNeg2, coeffs[3].lpNeg2};
        alignas(16) const std::array<float, 4> invPos2{coeffs[0].invLpPos2, coeffs[1].invLpPos2,
                                                       coeffs[2].invLpPos2, coeffs[3].invLpPos2};
        alignas(16) const std::array<float, 4> invNeg2{coeffs[0].invLpNeg2, coeffs[1].invLpNeg2,
                                                       coeffs[2].invLpNeg2, coeffs[3].invLpNeg2};
        alignas(16) const std::array<float, 4> dSpan{coeffs[0].lpDiodeSpan, coeffs[1].lpDiodeSpan,
                                                     coeffs[2].lpDiodeSpan, coeffs[3].lpDiodeSpan};
        alignas(16) const std::array<float, 4> invDSpan{
            coeffs[0].invLpDiodeSpan, coeffs[1].invLpDiodeSpan, coeffs[2].invLpDiodeSpan,
            coeffs[3].invLpDiodeSpan};

        for (size_t sub = 0u; sub < 4u; ++sub) {
            alignas(16) std::array<float, 4> x{};
            alignas(16) std::array<float, 4> w{};
            alignas(16) std::array<float, 4> dw{};
            alignas(16) std::array<float, 4> finalOta2{};

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
                dw[v] = fLp * fastRecipPos(df);
                w[v] -= dw[v];
                finalOta2[v] = ota2.value - ota2.deriv * dw[v];
            }

            if (!(absf(dw[0]) < 1.0e-2f && absf(dw[1]) < 1.0e-2f && absf(dw[2]) < 1.0e-2f &&
                  absf(dw[3]) < 1.0e-2f)) {
                for (size_t v = 0u; v < 4u; ++v) {
                    if (!(absf(dw[v]) < 1.0e-2f)) {
                        const SatResult ota2 =
                            evalOta(w[v], pos2[v], neg2[v], invPos2[v], invNeg2[v]);
                        const float y2 = s2[v] + g[v] * ota2.value;
                        const float y1 = w[v] + y2;
                        const SatResult ota1 = evalOtaSymmetric(x[v] - y1, rail1[v], invRail1[v]);
                        const SatResult fb = evalSiliconDiode(k[v] * w[v], dSpan[v], invDSpan[v]);
                        const float fLp = y1 - s1[v] - g[v] * (ota1.value + fb.value);
                        const float d1 = g[v] * ota1.deriv;
                        const float d2 = g[v] * ota2.deriv;
                        const float step = fLp * fastRecipPos((1.0f + d1) * (1.0f + d2) -
                                                              g[v] * (k[v] * fb.deriv));
                        w[v] -= step;
                        finalOta2[v] = ota2.value - ota2.deriv * step;
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
#endif
}

} // namespace ruido
