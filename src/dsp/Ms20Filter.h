#pragma once

namespace ruido {

// Single-voice series Korg35 (MS-20) 2-pole High-Pass -> 2-pole Low-Pass ZDF filter
// with independent High-Pass Peak (HP RES) and Low-Pass Peak (LP RES):
//  - Topology-Preserving Transform (TPT) trapezoidal integrators
//  - Exact scalar Newton-Raphson delay-free nonlinear feedback loop solver (k up to 3.72)
//  - Asymmetric silicon diode clipper inside both feedback paths + Korg35 passband dividers
//  - Van der Pol nonlinear period compensation for pitch-true self-oscillation on both filters
//  - Tight Smart-Park Resonance & phase-transparent parking at 20 Hz / 20 kHz
class Ms20DualFilter {
public:
    struct Coeffs {
        float gHp = 0.0f;
        float oneMinusGHp = 1.0f;
        float alphaHp = 0.0f;
        float kHp = 0.58578644f;
        float akHp = 0.0f;

        float gLp = 0.25f;
        float oneMinusGLp = 0.75f;
        float alphaLp = 0.1875f;
        float kLp = 0.58578644f;
        float akLp = 0.10983496f;

        float hpInScale = 1.0f;
        float interStageScale = 1.0f;
        float lpOutScale = 1.0f;
        float dcTrackG = 0.000285f;
        float dcCancelMix = 0.0f;
    };

    Ms20DualFilter() noexcept = default;

    void reset() noexcept;

    static Coeffs prepareCoeffs(float hpCutoffHz, float hpResonance, float lpCutoffHz,
                                float lpResonance, float invSampleRate) noexcept;

    float processPrepared(float input, const Coeffs& c) noexcept;

    float processSample(float input, float hpCutoffHz, float hpResonance, float lpCutoffHz,
                        float lpResonance, float invSampleRate) noexcept {
        return processPrepared(
            input,
            prepareCoeffs(hpCutoffHz, hpResonance, lpCutoffHz, lpResonance, invSampleRate));
    }

private:
    struct Korg35Stage {
        float s1 = 0.0f;
        float s2 = 0.0f;
        float lastPhi = 0.0f;

        void reset() noexcept {
            s1 = 0.0f;
            s2 = 0.0f;
            lastPhi = 0.0f;
        }
    };

    static float processHighPass(Korg35Stage& st, float x, const Coeffs& c) noexcept;
    static float processLowPass(Korg35Stage& st, float x, const Coeffs& c) noexcept;

    Korg35Stage hpf_{};
    Korg35Stage lpf_{};
    float dcMean_ = 0.0f;
};

} // namespace ruido
