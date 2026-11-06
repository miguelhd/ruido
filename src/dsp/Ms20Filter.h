#pragma once

#include "core/Config.h"

#include <array>
#include <cstddef>

namespace ruido {

// Single-voice series MS-20 Rev 2 (KLM-307 LM13600 OTA Sallen-Key) 2-pole High-Pass ->
// 2-pole Low-Pass ZDF filter with independent High-Pass Peak (HP RES) and Low-Pass Peak (LP RES):
//  - True capacitive Sallen-Key nodal topology (C1/C2 series capacitive injection on HPF;
//    C2 differential state-sensed feedback on LPF -> organic zero DC without post-filter blockers,
//    and zero bass-induced resonance quenching during simultaneous dual self-oscillation)
//  - C^2 pure-FPU LM13600 diode-linearized OTA transconductors on all 4 poles with complementary
//    Wilson current-mirror rail asymmetry (warm 2nd/3rd harmonics & slew rounding even at 0% RES)
//  - C^4-deadband / C^2-rail 1S1588 anti-parallel silicon PN-junction feedback limiter
//  - Exact coupled analytical Newton-Raphson ZDF solver with proven global uniqueness (df > 0.12)
class Ms20DualFilter {
  public:
    struct Coeffs {
        float gHp = 0.0f;
        float kHp = 0.58578644f;
        float otaScaleHp = 20.0f;
        float invOtaScaleHp = 0.05f;
        float hpPos1 = 39.9f;
        float hpNeg1 = 41.1f;
        float invHpPos1 = 0.02506f;
        float invHpNeg1 = 0.02433f;
        float hpRail2 = 40.5f;
        float invHpRail2 = 0.02469f;
        float hpDiodeSpan = 31.0f;
        float invHpDiodeSpan = 0.032f;
        float invLinDetHp = 1.0f;
        bool hpLinearBypass = true;

        float gLp = 0.25f;
        float kLp = 0.58578644f;
        float lpRail1 = 40.5f;
        float invLpRail1 = 0.0247f;
        float lpPos2 = 41.55f;
        float lpNeg2 = 39.45f;
        float invLpPos2 = 0.0241f;
        float invLpNeg2 = 0.0253f;
        float lpDiodeSpan = 26.5f;
        float invLpDiodeSpan = 0.0377f;
        float invLinDetLp = 0.70618806f;
        bool lpLinearBypass = true;

        float preDrive = 1.0f;
        float driveBlend = 0.0f;
        float hpInScale = 1.0f;
        float interStageScale = 1.0f;
        float lpOutScale = 1.0f;
    };

    Ms20DualFilter() noexcept {}

    void reset() noexcept;

    static Coeffs prepareCoeffs(float hpCutoffHz, float hpResonance, float lpCutoffHz,
                                float lpResonance, float invSampleRate,
                                float preDrive = 1.0f) noexcept;

    float processPrepared(float input, const Coeffs& c) noexcept;

    // Process `numFrames` consecutive 4x-oversampled frames for one voice with hoisted bypass
    // and overdrive checks and register-resident integrator states across all `4 * numFrames`
    // ticks.
    void processSpan4x(std::array<float, kOversampleFactor>* frames4x, size_t numFrames,
                       const Coeffs& c) noexcept;

    // Lockstep 4-voice x 4x-oversampled bank processor: evaluates all 4 independent voice filters
    // in parallel FPU/SIMD lanes per sub-step while preserving exact per-voice circuit isolation.
    static void processQuadBank4x(
        std::array<Ms20DualFilter, kNumOscillators>& filters,
        std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSamples,
        const std::array<Coeffs, kNumOscillators>& coeffs) noexcept;

    float processSample(float input, float hpCutoffHz, float hpResonance, float lpCutoffHz,
                        float lpResonance, float invSampleRate, float preDrive = 1.0f) noexcept {
        return processPrepared(input, prepareCoeffs(hpCutoffHz, hpResonance, lpCutoffHz,
                                                    lpResonance, invSampleRate, preDrive));
    }

  private:
    struct OtaSallenKeyStage {
        float s1 = 0.0f;
        float s2 = 0.0f;
        float hist1 = 0.0f;
        float hist2 = 0.0f;

        void reset() noexcept {
            s1 = 0.0f;
            s2 = 0.0f;
            hist1 = 0.0f;
            hist2 = 0.0f;
        }
    };

    static float processHighPass(OtaSallenKeyStage& st, float x, const Coeffs& c) noexcept;
    static float processLowPass(OtaSallenKeyStage& st, float x, const Coeffs& c) noexcept;
    static void processVoiceHpSpan4x(OtaSallenKeyStage& st,
                                     std::array<float, kOversampleFactor>* frames4x,
                                     size_t numFrames, const Coeffs& c) noexcept;
    static void processVoiceLpSpan4x(OtaSallenKeyStage& st,
                                     std::array<float, kOversampleFactor>* frames4x,
                                     size_t numFrames, const Coeffs& c) noexcept;

    OtaSallenKeyStage hpf_{};
    OtaSallenKeyStage lpf_{};
};

} // namespace ruido
