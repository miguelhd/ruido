#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "dsp/SallenKeyFilter.h"
#include "dsp/SchmittOscillatorBank.h"
#include "dsp/VoiceFilterCoProcessor.h"

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
RUIDO_ALWAYS_INLINE float evalWavefoldSineFast(float cycles) noexcept {
    // Bounded positive phase shift (+4.25f) fuses `wrapLfoPhase01` + `evalAnalogSine`'s +0.25f
    // quarter-cycle offset into a single branchless `itof(ftoi(qRaw))` wrap across [-3.5, +3.5].
    const float qRaw = clampf(cycles, -3.5f, 3.5f) + 4.25f;
    const float q = qRaw - static_cast<float>(static_cast<int32_t>(qRaw));
    const float u = 1.0f - 4.0f * absf(q - 0.5f);
    const float u2 = u * u;
    const float s = u * (1.57079633f + u2 * (-0.64596410f + u2 * (0.07953921f - 0.00437144f * u2)));
    const float s2 = s * s;
    return clampf(s * (1.0886366f - 0.0886366f * s2), -1.0f, 1.0f);
}

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
        const float folded = 1.12f * evalWavefoldSineFast(0.25f * (capRamp[v] * foldGain + asym));
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
//     Hoists per-voice S&H clock increment and R-2R step reciprocals across the full `chunk` span.
RUIDO_NOINLINE inline void
applySpanCrushPostFilter4x(VoiceFilterCoProcessor::VoiceSpanBuffer& spanSub, size_t chunk,
                           const std::array<float, kNumOscillators>& crushDepth,
                           float hzToOversampledPhaseU32,
                           std::array<VoiceCrushState, kNumOscillators>& st) noexcept {
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float d = clampf(crushDepth[v], 0.0f, 1.0f);
        if (d <= 1.0e-4f) {
            st[v].snhPhase = 0u;
            const float rest = sanitizeState(spanSub[v][chunk - 1u][kOversampleFactor - 1u]);
            st[v].heldSample = rest;
            st[v].capSample = rest;
            continue;
        }
        const float snhHz = computeCrushClockHz(d);
        const float incF = snhHz * hzToOversampledPhaseU32;
        const uint32_t inc = (incF >= 1.0f) ? static_cast<uint32_t>(incF) : 1u;
        const float steps = 10.0f - 7.5f * d;
        const float invSteps = fastRecipPos(steps);
        const float wet = smoothstep01(d * 2.5f);

        float* smp = spanSub[v][0].data();
        const size_t total = chunk * kOversampleFactor;
        uint32_t phase = st[v].snhPhase;
        float held = st[v].heldSample;
        float cap = st[v].capSample;
        for (size_t idx = 0u; idx < total; ++idx) {
            const float raw = smp[idx];
            const uint32_t prev = phase;
            phase = prev + inc;
            if (phase < prev) {
                const float railClamped = clampf(raw, -1.35f, 1.35f) * steps;
                const auto qInt =
                    static_cast<int32_t>(railClamped + ((railClamped >= 0.0f) ? 0.5f : -0.5f));
                held = sanitizeState(static_cast<float>(qInt) * invSteps);
            }
            // CD4013/CD4066 transmission-gate RON * CHOLD RC slew (~90 us settle at 176.472 kHz):
            // preserves sharp 16 kHz -> 165 Hz staircases while rounding 1-tick Dirac rail clicks.
            cap += 0.24f * (held - cap);
            smp[idx] = raw + (cap - raw) * wet;
        }
        st[v].snhPhase = phase;
        st[v].heldSample = held;
        st[v].capSample = sanitizeState(cap);
    }
}

// 6. Full 4x-oversampled per-substep patched oscillator frame (XMod / Wavefold / Sub-XOR / Ring).
inline uint8_t renderPatchedOscFrame4x(
    SchmittOscillatorBank& oscBank, OscillatorBankOutput& oscOut,
    std::array<std::array<float, kOversampleFactor>, kNumOscillators>& voiceSub,
    const std::array<float, kNumOscillators>& startFreq,
    const std::array<float, kNumOscillators>& endFreq, const std::array<float, kNumOscillators>& pw,
    const std::array<float, kNumOscillators>& drifts, float coupling, bool stasisFreeze,
    float hardSync, bool anyXmod, bool anyFold, bool anyCrush, bool anyRing, float ringModMix,
    const std::array<float, kNumOscillators>& xmodDepths,
    const std::array<float, kNumOscillators>& foldDepths,
    const std::array<float, kNumOscillators>& crushDepths,
    std::array<float, kNumOscillators>& prevOscSamples,
    std::array<float, kNumOscillators>& prevCapRamp,
    std::array<VoiceCrushState, kNumOscillators>& crushStates) noexcept {
    uint8_t voiceRisingInFrame = 0u;
    for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
        std::array<float, kNumOscillators> subFreqs;
        const float subAlpha = 0.25f * static_cast<float>(sub + 1u);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            subFreqs[i] = startFreq[i] + (endFreq[i] - startFreq[i]) * subAlpha;
        }
        if (anyXmod) {
            applyCrossPinFm4x(subFreqs, prevCapRamp, prevOscSamples, xmodDepths);
        }
        oscBank.processSample(oscOut, subFreqs, pw, drifts, coupling, stasisFreeze, hardSync);
        prevOscSamples = oscOut.samples;
        prevCapRamp = oscOut.capRamp;
        std::array<float, kNumOscillators> patched = oscOut.samples;
        if (anyXmod) {
            applyCrossPinXor4x(patched, oscOut.capRamp, xmodDepths);
        }
        if (anyFold) {
            applyCapacitorWavefold4x(patched, oscOut.capRamp, foldDepths);
        }
        if (anyCrush) {
            applyRippleSubXorPreFilter4x(patched, oscOut.risingMask, crushDepths, crushStates);
        } else if (oscOut.risingMask != 0u) {
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (((oscOut.risingMask >> v) & 1u) != 0u) {
                    crushStates[v].divCounter =
                        static_cast<uint8_t>((crushStates[v].divCounter + 1u) & 7u);
                }
            }
        }
        if (anyRing) {
            applyRingXorMatrix4x(patched, ringModMix);
        }
        voiceRisingInFrame |= oscOut.risingMask;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            voiceSub[i][sub] = patched[i];
        }
    }
    return voiceRisingInFrame;
}

// 7. Out-of-line pre-filter sub-block driver for Circuit Lab patched oscillator spans; filtering
//    then runs through the shared dual-core `VoiceFilterCoProcessor` pipeline.
struct PatchedSpanContext {
    size_t chunk = 0u;
    float invChunk = 1.0f;
    bool isLastChunk = false;
    bool anyXmod = false;
    bool anyFold = false;
    bool anyCrush = false;
    bool anyRing = false;
    bool stasisFreeze = false;
    float coupling = 0.0f;
    float hardSync = 0.0f;
    float ringModMix = 0.0f;
};

RUIDO_NOINLINE inline void renderPatchedOscSpan4x(
    const PatchedSpanContext& ctx, SchmittOscillatorBank& oscBank,
    std::array<std::array<float, kOversampleFactor>, kMaxFilterSpanFrames>* spanSub,
    uint8_t* spanRising, const std::array<float, kNumOscillators>& prevOscFreqHz,
    const std::array<float, kNumOscillators>& oscFreqs,
    const std::array<float, kNumOscillators>& prevPulseWidth,
    const std::array<float, kNumOscillators>& oscPulseWidths,
    const std::array<float, kNumOscillators>& oscDrifts,
    const std::array<float, kNumOscillators>& xmodDepths,
    const std::array<float, kNumOscillators>& foldDepths,
    const std::array<float, kNumOscillators>& crushDepths,
    std::array<float, kNumOscillators>& prevOscSamples,
    std::array<float, kNumOscillators>& prevCapRamp,
    std::array<VoiceCrushState, kNumOscillators>& crushStates,
    std::array<float, kNumOscillators>& lastGateHigh,
    std::array<float, kNumOscillators>& lastDriftNorm) noexcept {
    // Hoist per-voice XMod / Wavefold / Sub-XOR depths & cap-ramp mask across the entire span!
    std::array<float, kNumOscillators> xmodClamped{};
    std::array<float, kNumOscillators> xmodOctSpan{};
    std::array<float, kNumOscillators> foldClamped{};
    std::array<float, kNumOscillators> foldGain{};
    std::array<float, kNumOscillators> foldMix{};
    std::array<float, kNumOscillators> crushClamped{};
    std::array<float, kNumOscillators> crushSubMix{};
    std::array<uint8_t, kNumOscillators> xmodIdx{};
    std::array<uint8_t, kNumOscillators> foldIdx{};
    std::array<uint8_t, kNumOscillators> crushIdx{};
    uint8_t numXmod = 0u;
    uint8_t numFold = 0u;
    uint8_t numCrush = 0u;
    uint8_t needCapMask = 0u;
    uint8_t xmodVoiceMask = 0u;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float xd = clampf(xmodDepths[v], 0.0f, 1.0f);
        xmodClamped[v] = xd;
        if (xd > 1.0e-4f) {
            xmodOctSpan[v] = 3.2f * xd * (0.25f + 0.75f * xd);
            needCapMask |= static_cast<uint8_t>(1u << ((v + 1u) & 3u));
            xmodVoiceMask |= static_cast<uint8_t>(1u << v);
            xmodIdx[numXmod++] = static_cast<uint8_t>(v);
        }
        const float fd = clampf(foldDepths[v], 0.0f, 1.0f);
        foldClamped[v] = fd;
        if (fd > 1.0e-4f) {
            foldGain[v] = computeWavefoldStages(fd);
            foldMix[v] = smoothstep01(fd * 2.4f);
            needCapMask |= static_cast<uint8_t>(1u << v);
            foldIdx[numFold++] = static_cast<uint8_t>(v);
        }
        const float cd = clampf(crushDepths[v], 0.0f, 1.0f);
        crushClamped[v] = cd;
        if (cd > 1.0e-4f) {
            crushSubMix[v] = 0.76f * smoothstep01(cd * 1.7f);
            crushIdx[numCrush++] = static_cast<uint8_t>(v);
        }
    }
    const float ringMix = clampf(ctx.ringModMix, 0.0f, 1.0f);
    const float invSpan4x = 0.25f * ctx.invChunk;

    SchmittOscillatorBank::PatchedSpanState oscSt{};
    oscBank.preparePatchedSpan(oscSt, ctx.invChunk, prevOscFreqHz, oscFreqs, prevPulseWidth,
                               oscPulseWidths, oscDrifts, ctx.coupling, ctx.stasisFreeze,
                               ctx.hardSync, needCapMask, xmodVoiceMask);

    std::array<float, kNumOscillators> curBaseFreq = prevOscFreqHz;
    std::array<float, kNumOscillators> freqStepFrame{};
    std::array<float, kNumOscillators> frameMidFreq{};
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        freqStepFrame[i] = 4.0f * (oscFreqs[i] - prevOscFreqHz[i]) * invSpan4x;
    }

    std::array<float, kNumOscillators> oscSamples0 = prevOscSamples;
    std::array<float, kNumOscillators> oscSamples1 = prevOscSamples;
    std::array<float, kNumOscillators> oscCapRamp0 = prevCapRamp;
    std::array<float, kNumOscillators> oscCapRamp1 = prevCapRamp;

    const auto postPatchSub = [&](std::array<float, kNumOscillators>& patched,
                                  const std::array<float, kNumOscillators>& capRamp,
                                  uint8_t risingMask) noexcept {
        if (numXmod > 0u) {
            const std::array<float, kNumOscillators> s = patched;
            for (uint8_t j = 0u; j < numXmod; ++j) {
                const size_t v = xmodIdx[j];
                const size_t nbr = (v + 1u) & 3u;
                const float xorTear = clampf(s[v] * capRamp[nbr] * 1.35f, -1.15f, 1.15f);
                patched[v] = s[v] + (xorTear - s[v]) * (0.58f * xmodClamped[v]);
            }
        }
        if (numFold > 0u) {
            for (uint8_t j = 0u; j < numFold; ++j) {
                const size_t v = foldIdx[j];
                const float asym = 0.20f * foldClamped[v] * clampf(patched[v], -1.0f, 1.0f);
                const float folded =
                    1.12f * evalWavefoldSineFast(0.25f * (capRamp[v] * foldGain[v] + asym));
                patched[v] += (folded - patched[v]) * foldMix[v];
            }
        }
        if (risingMask != 0u) {
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (((risingMask >> v) & 1u) != 0u) {
                    crushStates[v].divCounter =
                        static_cast<uint8_t>((crushStates[v].divCounter + 1u) & 7u);
                }
            }
        }
        if (numCrush > 0u) {
            for (uint8_t j = 0u; j < numCrush; ++j) {
                const size_t v = crushIdx[j];
                const float cd = crushClamped[v];
                const uint8_t cnt = crushStates[v].divCounter;
                const float sub1 = ((cnt & 1u) != 0u) ? 1.0f : -1.0f;
                const float sub2 = ((cnt & 2u) != 0u) ? 1.0f : -1.0f;
                const float sub3 = ((cnt & 4u) != 0u) ? 1.0f : -1.0f;
                const float s = clampf(patched[v], -1.1f, 1.1f);
                const float xor1 = -s * sub1;
                const float cascade =
                    clampf(0.58f * xor1 + 0.32f * cd * sub2 + 0.28f * (cd * cd) * (xor1 * sub3),
                           -1.15f, 1.15f);
                patched[v] += (cascade - patched[v]) * crushSubMix[v];
            }
        }
        if (ctx.anyRing && ringMix > 1.0e-4f) {
            const std::array<float, kNumOscillators> s = patched;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                const float ring = clampf(s[i] * s[(i + 1u) & 3u], -1.25f, 1.25f);
                patched[i] = s[i] + (ring - s[i]) * ringMix;
            }
        }
    };

    std::array<float, kNumOscillators> pairFreqs{};
    for (size_t k = 0u; k < ctx.chunk; ++k) {
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            frameMidFreq[i] = curBaseFreq[i] + 0.625f * freqStepFrame[i];
        }
        oscBank.beginPatchedFrame(oscSt, frameMidFreq);
        uint8_t voiceRisingInFrame = 0u;

        for (size_t pair = 0u; pair < 2u; ++pair) {
            if (numXmod > 0u) {
                const float baseOff = (pair == 0u) ? 0.25f : 0.75f;
                for (uint8_t j = 0u; j < numXmod; ++j) {
                    const size_t v = xmodIdx[j];
                    const size_t nbr = (v + 1u) & 3u;
                    const float bHz = curBaseFreq[v] + baseOff * freqStepFrame[v];
                    const float modWave =
                        0.72f * prevCapRamp[nbr] + 0.28f * clampf(prevOscSamples[v], -1.25f, 1.25f);
                    pairFreqs[v] =
                        clampf(bHz * fastExp2(xmodOctSpan[v] * modWave), 0.005f, 21200.0f);
                }
            }

            uint8_t rising0 = 0u;
            uint8_t rising1 = 0u;
            oscBank.stepPatchedSubPair(oscSt, pairFreqs, oscSamples0, oscSamples1, oscCapRamp0,
                                       oscCapRamp1, rising0, rising1);
            prevOscSamples = oscSamples1;
            prevCapRamp = oscCapRamp1;

            const size_t subBase = pair * 2u;
            for (size_t sIdx = 0u; sIdx < 2u; ++sIdx) {
                auto& smpArr = (sIdx == 0u) ? oscSamples0 : oscSamples1;
                const auto& capArr = (sIdx == 0u) ? oscCapRamp0 : oscCapRamp1;
                const uint8_t rMask = (sIdx == 0u) ? rising0 : rising1;
                postPatchSub(smpArr, capArr, rMask);
                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    spanSub[i][k][subBase + sIdx] = smpArr[i];
                }
            }
            voiceRisingInFrame |= static_cast<uint8_t>(rising0 | rising1);
        }
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            curBaseFreq[i] += freqStepFrame[i];
        }
        spanRising[k] = voiceRisingInFrame;
    }

    oscBank.finishPatchedSpan(ctx.isLastChunk ? &lastGateHigh : nullptr,
                              ctx.isLastChunk ? &lastDriftNorm : nullptr);
}

} // namespace ruido
