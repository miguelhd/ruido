#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// 4x-oversampled (176.472 kHz) CMOS breadboard / West-Coast circuit-bend stages invoked around
// SchmittOscillatorBank and Ms20DualFilter on Core 1. Zero keyboard or UI coupling.

struct VoiceCrushState {
    uint32_t snhPhase = 0u;
    float heldSample = 0.0f;
    float capSample = 0.0f;
    uint8_t divCounter = 0u;
};

// 1. CROSS-PIN pre-oscillator FM: neighbor `(v + 1) & 3` raw pre-mute CD40106 capacitor ramp +
//    chaotic self-feedback drives continuous +-3.2 Oct exponential FM on Voice `v` itself.
inline void applyCrossPinFm4x(std::array<float, kNumOscillators>& subFreqs,
                              const std::array<float, kNumOscillators>& prevCapRamp,
                              const std::array<float, kNumOscillators>& prevOscSamples,
                              const std::array<float, kNumOscillators>& xmodDepth) noexcept {
    const std::array<float, kNumOscillators> baseFreqs = subFreqs;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float d = clampf(xmodDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            continue;
        }
        const size_t nbr = (v + 1u) & 3u;
        const float modWave =
            0.72f * prevCapRamp[nbr] + 0.28f * clampf(prevOscSamples[v], -1.25f, 1.25f);
        const float octSpan = 3.2f * d * (0.25f + 0.75f * d);
        subFreqs[v] = clampf(baseFreqs[v] * fastExp2(octSpan * modWave), 0.005f, 21200.0f);
    }
}

// 2. CROSS-PIN post-oscillator comparator tear: injects neighbor's raw capacitor ramp into OSC v.
inline void applyCrossPinXor4x(std::array<float, kNumOscillators>& oscSamples,
                               const std::array<float, kNumOscillators>& capRamp,
                               const std::array<float, kNumOscillators>& xmodDepth) noexcept {
    const std::array<float, kNumOscillators> s = oscSamples;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float d = clampf(xmodDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            continue;
        }
        const size_t nbr = (v + 1u) & 3u;
        const float xorTear = clampf(s[v] * capRamp[nbr] * 1.35f, -1.15f, 1.15f);
        oscSamples[v] = s[v] + (xorTear - s[v]) * (0.58f * d);
    }
}

// 3. Paired 4x-oversampled balanced ring / bipolar CMOS XOR sidebands (1x2, 2x3, 3x4, 4x1).
inline void applyRingXorMatrix4x(std::array<float, kNumOscillators>& oscSamples,
                                 float ringModMix) noexcept {
    const float mix = clampf(ringModMix, 0.0f, 1.0f);
    if (mix <= 1.0e-4f) {
        return;
    }
    const std::array<float, kNumOscillators> s = oscSamples;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const float ring = clampf(s[i] * s[(i + 1u) & 3u], -1.25f, 1.25f);
        oscSamples[i] = s[i] + (ring - s[i]) * mix;
    }
}

// 4. FOLD: 1.0x -> 6.2x West-Coast Serge sinusoidal wavefolder tapped off the CD40106 cap ramp.
inline void applyCapacitorWavefold4x(std::array<float, kNumOscillators>& oscSamples,
                                     const std::array<float, kNumOscillators>& capRamp,
                                     const std::array<float, kNumOscillators>& foldDepth) noexcept {
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float d = clampf(foldDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            continue;
        }
        const float foldGain = computeWavefoldStages(d);
        const float asym = 0.20f * d * clampf(oscSamples[v], -1.0f, 1.0f);
        const float folded =
            1.12f * evalAnalogSine(wrapLfoPhase01(0.25f * (capRamp[v] * foldGain + asym)));
        const float mix = smoothstep01(d * 2.4f);
        oscSamples[v] += (folded - oscSamples[v]) * mix;
    }
}

// 5a. CRUSH pre-filter stage: CD4040 3-bit ripple counter (/2, /4, /8) sub-octave logic XOR.
inline void
applyRippleSubXorPreFilter4x(std::array<float, kNumOscillators>& oscSamples, uint8_t risingMask,
                             const std::array<float, kNumOscillators>& crushDepth,
                             std::array<VoiceCrushState, kNumOscillators>& st) noexcept {
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (((risingMask >> v) & 1u) != 0u) {
            st[v].divCounter = static_cast<uint8_t>((st[v].divCounter + 1u) & 7u);
        }
        const float d = clampf(crushDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            continue;
        }
        const uint8_t cnt = st[v].divCounter;
        const float sub1 = ((cnt & 1u) != 0u) ? 1.0f : -1.0f;
        const float sub2 = ((cnt & 2u) != 0u) ? 1.0f : -1.0f;
        const float sub3 = ((cnt & 4u) != 0u) ? 1.0f : -1.0f;
        const float s = clampf(oscSamples[v], -1.1f, 1.1f);
        const float xor1 = -s * sub1;
        const float cascade = clampf(
            0.58f * xor1 + 0.32f * d * sub2 + 0.28f * (d * d) * (xor1 * sub3), -1.15f, 1.15f);
        const float mix = 0.76f * smoothstep01(d * 1.7f);
        oscSamples[v] += (cascade - oscSamples[v]) * mix;
    }
}

// 5b. CRUSH post-filter stage: CD4013 zero-order S&H clock decimator (16 kHz -> 165 Hz) + R-2R
//     stepped DAC rung quantizer capturing resonant Korg35 filter ringing before 4x->1x FIR.
inline void applySampleHoldCrushPostFilter4x(
    std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSub,
    const std::array<float, kNumOscillators>& crushDepth, float hzToOversampledPhaseU32,
    std::array<VoiceCrushState, kNumOscillators>& st) noexcept {
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float d = clampf(crushDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            st[v].snhPhase = 0u;
            const float rest = sanitizeState(voiceSub[v][kOversampleFactor - 1u]);
            st[v].heldSample = rest;
            st[v].capSample = rest;
            continue;
        }
        const float snhHz = computeCrushClockHz(d);
        const float incF = snhHz * hzToOversampledPhaseU32;
        const uint32_t inc = (incF >= 1.0f) ? static_cast<uint32_t>(incF) : 1u;
        const float steps = 10.0f - 7.5f * d;
        const float invSteps = 1.0f / steps;
        const float wet = smoothstep01(d * 2.5f);

        for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
            const float raw = voiceSub[v][sub];
            const uint32_t prev = st[v].snhPhase;
            st[v].snhPhase = prev + inc;
            if (st[v].snhPhase < prev) {
                const float railClamped = clampf(raw, -1.35f, 1.35f) * steps;
                const auto qInt =
                    static_cast<int32_t>(railClamped + ((railClamped >= 0.0f) ? 0.5f : -0.5f));
                st[v].heldSample = sanitizeState(static_cast<float>(qInt) * invSteps);
            }
            // CD4013/CD4066 transmission-gate RON * CHOLD RC slew (~90 us settle at 176.472 kHz):
            // preserves sharp 16 kHz -> 165 Hz staircases while rounding 1-tick Dirac rail clicks.
            st[v].capSample =
                sanitizeState(st[v].capSample + 0.24f * (st[v].heldSample - st[v].capSample));
            voiceSub[v][sub] = raw + (st[v].capSample - raw) * wet;
        }
    }
}

} // namespace ruido
