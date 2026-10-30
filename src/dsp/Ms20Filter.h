#pragma once

#include "core/Config.h"

#include <array>

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
        float invLinDetHp = 1.0f;
        bool hpLinearBypass = true;

        float gLp = 0.25f;
        float kLp = 0.58578644f;
        float otaScaleLp = 20.0f;
        float invOtaScaleLp = 0.05f;
        float invLinDetLp = 0.70618806f;
        bool lpLinearBypass = true;

        float hpInScale = 1.0f;
        float interStageScale = 1.0f;
        float lpOutScale = 1.0f;
    };

    Ms20DualFilter() noexcept = default;

    void reset() noexcept;

    static Coeffs prepareCoeffs(float hpCutoffHz, float hpResonance, float lpCutoffHz,
                                float lpResonance, float invSampleRate) noexcept;

    float processPrepared(float input, const Coeffs& c) noexcept;

    // Lockstep 4-voice x 4x-oversampled bank processor: evaluates all 4 independent voice filters
    // in parallel FPU/SIMD lanes per sub-step while preserving exact per-voice circuit isolation.
    static void processQuadBank4x(
        std::array<Ms20DualFilter, kNumOscillators>& filters,
        std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSamples,
        const std::array<Coeffs, kNumOscillators>& coeffs) noexcept;

    float processSample(float input, float hpCutoffHz, float hpResonance, float lpCutoffHz,
                        float lpResonance, float invSampleRate) noexcept {
        return processPrepared(
            input,
            prepareCoeffs(hpCutoffHz, hpResonance, lpCutoffHz, lpResonance, invSampleRate));
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

    OtaSallenKeyStage hpf_{};
    OtaSallenKeyStage lpf_{};
};

} // namespace ruido
