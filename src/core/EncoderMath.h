#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"

#include <cstdint>

namespace ruido {

// Strictly linear 3-gear X4 quadrature encoder scaling (96 steps = 360 deg; zero velocity accel):
//   Fine (Hold OCT-)   : ~63 cents/turn audio (~0.66 cents/step)
//   Standard           : ~2.7 oct/turn audio (musical pitch sweeping)
//   Coarse (Hold OCT+) : ~1.35 turns full 18.6-octave sweep (0.05 Hz <-> 20 kHz)
inline void adjustVoiceFrequency(VoiceParams& vp, int32_t ticks, KnobRate rate) noexcept {
    const float steps = static_cast<float>(ticks);
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00055f : ((rate == KnobRate::Coarse) ? 0.14500f : 0.02850f);
    const float nextHz = vp.frequencyHz * fastExp2(steps * octPerStep);
    vp.frequencyHz = clampf(nextHz, kAudioFreqMinHz, kAudioFreqMaxHz);
}

inline float adjustFilterCutoff(float currentHz, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00250f : ((rate == KnobRate::Coarse) ? 0.10800f : 0.03650f);
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);
}

inline float adjustLfoRate(float currentHz, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00250f : ((rate == KnobRate::Coarse) ? 0.13600f : 0.04200f);
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kLfoFreqMinHz, kLfoFreqMaxHz);
}

inline float adjustLinear01(float current, int32_t ticks, KnobRate rate, float lo = 0.0f,
                            float hi = 1.0f) noexcept {
    const float stepFrac =
        (rate == KnobRate::Fine) ? 0.00090f : ((rate == KnobRate::Coarse) ? 0.00800f : 0.00420f);
    const float span = hi - lo;
    const float delta = static_cast<float>(ticks) * stepFrac * span;
    return clampf(current + delta, lo, hi);
}

// 3-gear logarithmic Master Tempo scaling on ALGORITHM in RHYTHM LAB (20 -> 3,000 BPM = 7.23 oct):
//   Fine (Hold OCT-)   : ~0.058 oct/turn (~0.2 BPM/detent @ 120 BPM; ~0.72 cents/tick @ audio rate)
//   Standard           : 2.00 oct/turn (1 semitone/detent; ~3.6 turns across 20 -> 3,000 BPM)
//   Coarse (Hold OCT+) : 6.00 oct/turn (3 semitones/detent; ~1.2 turns across 20 -> 3,000 BPM)
inline float adjustClockBpm(float currentBpm, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00060f : ((rate == KnobRate::Coarse) ? 0.06250f : 0.02083333f);
    const float nextBpm = currentBpm * fastExp2(static_cast<float>(ticks) * octPerStep);
    return clampf(nextBpm, kMinInternalClockBpm, kMaxInternalClockBpm);
}

// Normalized [0, 1] log-octave position across [kMinInternalClockBpm, kMaxInternalClockBpm]
inline float normalizeClockBpmLog01(float bpm) noexcept {
    constexpr float kLogMin = 4.32192809f;                            // log2(20.0f)
    constexpr float kInvLogSpan = 1.0f / (11.5507468f - 4.32192809f); // 1 / log2(3000 / 20)
    const float clamped = clampf(bpm, kMinInternalClockBpm, kMaxInternalClockBpm);
    return clampf((fastLog2(clamped) - kLogMin) * kInvLogSpan, 0.0f, 1.0f);
}

// Discrete notched encoder stepper: always 1 item per physical detent (4 X4 quadrature ticks)
// across Fine, Standard, and Coarse gears, with zero-windup clamping at [0, maxVal].
// Performs arithmetic in `int32_t` before storing the [-3, +3] sub-detent remainder so `int8_t`
// accumulators never overflow regardless of `ticks` magnitude.
template <typename AccumInt>
inline int32_t stepClampedDetent(int32_t currentVal, int32_t maxVal, int32_t ticks, KnobRate rate,
                                 AccumInt& accum) noexcept {
    (void)rate;
    constexpr int32_t kDetentThreshold = 4;
    int32_t acc = static_cast<int32_t>(accum);
    if ((ticks > 0 && acc < 0) || (ticks < 0 && acc > 0)) {
        acc = 0;
    }
    acc += ticks;
    const int32_t steps = acc / kDetentThreshold;
    if (steps == 0) {
        if ((currentVal <= 0 && acc < 0) || (currentVal >= maxVal && acc > 0)) {
            acc = 0;
        }
        accum = static_cast<AccumInt>(acc);
        return currentVal;
    }
    acc -= steps * kDetentThreshold;
    int32_t next = currentVal + steps;
    if (next <= 0) {
        next = 0;
        if (acc < 0) {
            acc = 0;
        }
    } else if (next >= maxVal) {
        next = maxVal;
        if (acc > 0) {
            acc = 0;
        }
    }
    accum = static_cast<AccumInt>(acc);
    return next;
}

inline void applyEditKnobDelta(EditTarget target, VoiceParams& vp, int32_t ticks,
                               KnobRate rate) noexcept {
    switch (target) {
    case EditTarget::Width:
        vp.pulseWidth = adjustLinear01(vp.pulseWidth, ticks, rate, kPulseWidthMin, kPulseWidthMax);
        break;
    case EditTarget::Drift:
        vp.drift = adjustLinear01(vp.drift, ticks, rate, 0.0f, 1.0f);
        break;
    case EditTarget::Level:
    case EditTarget::Count:
        if (vp.muted) {
            if (ticks > 0) {
                vp.muted = false;
                vp.amplitude = adjustLinear01(0.0f, ticks, rate, 0.0f, 1.0f);
            }
        } else {
            const float nextAmp = adjustLinear01(vp.amplitude, ticks, rate, 0.0f, 1.0f);
            const bool lfoAmpActive =
                (vp.lfoTarget == LfoTarget::Amplitude && vp.lfoDepth > 1.0e-4f) ||
                (vp.lfoBTarget == LfoTarget::Amplitude && vp.lfoBDepth > 1.0e-4f);
            if (nextAmp <= 1.0e-4f && !lfoAmpActive) {
                vp.amplitude = 0.0f;
                vp.muted = true;
            } else {
                vp.amplitude = nextAmp;
            }
        }
        break;
    }
}

inline void applyVoiceFilterKnobDelta(VoiceParams& vp, uint8_t knobIdx, int32_t ticks,
                                      KnobRate rate) noexcept {
    if (knobIdx == 0u) {
        vp.hpCutoffHz = adjustFilterCutoff(vp.hpCutoffHz, ticks, rate);
    } else if (knobIdx == 1u) {
        vp.hpResonance = adjustLinear01(vp.hpResonance, ticks, rate, 0.0f, 1.0f);
    } else if (knobIdx == 2u) {
        vp.lpCutoffHz = adjustFilterCutoff(vp.lpCutoffHz, ticks, rate);
    } else {
        vp.lpResonance = adjustLinear01(vp.lpResonance, ticks, rate, 0.0f, 1.0f);
    }
}

template <typename AccumInt>
inline void applyVoiceLfoKnobDelta(VoiceParams& vp, size_t subIdx, uint8_t knobIdx, int32_t ticks,
                                   KnobRate rate, AccumInt& targetTickAccum) noexcept {
    if (knobIdx == 0u) {
        LfoTarget& tgtRef = lfoSlotTargetRef(vp, subIdx);
        const int32_t maxTgt =
            static_cast<int32_t>((subIdx == 0u) ? kNumLfoATargets : kNumLfoBTargets) - 1;
        tgtRef = static_cast<LfoTarget>(
            stepClampedDetent(static_cast<int32_t>(tgtRef), maxTgt, ticks, rate, targetTickAccum));
    } else if (knobIdx == 1u) {
        lfoSlotShapeRef(vp, subIdx) =
            adjustLinear01(lfoSlotShape(vp, subIdx), ticks, rate, 0.0f, 1.0f);
    } else if (knobIdx == 2u) {
        lfoSlotRateHzRef(vp, subIdx) = adjustLfoRate(lfoSlotRateHz(vp, subIdx), ticks, rate);
    } else {
        lfoSlotDepthRef(vp, subIdx) =
            adjustLinear01(lfoSlotDepth(vp, subIdx), ticks, rate, 0.0f, 1.0f);
    }
}

inline void toggleVoiceMuteGate(VoiceParams& vp) noexcept {
    const bool lfoAmpActive =
        ((vp.lfoTarget == LfoTarget::Amplitude) && (vp.lfoDepth > kSilentAmpThreshold)) ||
        ((vp.lfoBTarget == LfoTarget::Amplitude) && (vp.lfoBDepth > kSilentAmpThreshold));
    if (!vp.muted && vp.amplitude <= kSilentAmpThreshold && !lfoAmpActive) {
        vp.amplitude = kDefaultVoiceAmp;
    } else {
        vp.muted = !vp.muted;
        if (!vp.muted && vp.amplitude <= 0.01f && !lfoAmpActive) {
            vp.amplitude = kDefaultVoiceAmp;
        }
    }
}

} // namespace ruido
