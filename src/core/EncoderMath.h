#pragma once

#include "core/Config.h"
#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/Parameters.h"

#include <cstdint>

namespace ruido {

// Strictly linear 3-gear X4 quadrature encoder scaling (96 steps = 360 deg; zero velocity accel):
//   Fine (Hold OCT-)   : ~63 cents/turn audio (~0.66 cents/step)
//   Standard           : ~2.8 turns full sweep
//   Fast/Coarse (OCT+) : ~1.3 turns full sweep
inline void adjustVoiceFrequency(VoiceParams& vp, int32_t ticks, KnobRate rate) noexcept {
    const float steps = static_cast<float>(ticks);
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00055f : ((rate == KnobRate::Fast) ? 0.06000f : 0.02850f);
    const float nextHz = vp.frequencyHz * fastExp2(steps * octPerStep);
    vp.frequencyHz = clampf(nextHz, kAudioFreqMinHz, kAudioFreqMaxHz);
}

inline float adjustFilterCutoff(float currentHz, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00250f : ((rate == KnobRate::Fast) ? 0.07800f : 0.03650f);
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);
}

inline float adjustLfoRate(float currentHz, int32_t ticks, KnobRate rate) noexcept {
    const float octPerStep =
        (rate == KnobRate::Fine) ? 0.00250f : ((rate == KnobRate::Fast) ? 0.08200f : 0.03800f);
    const float octDelta = static_cast<float>(ticks) * octPerStep;
    return clampf(currentHz * fastExp2(octDelta), kLfoFreqMinHz, kLfoFreqMaxHz);
}

inline float adjustLinear01(float current, int32_t ticks, KnobRate rate, float lo = 0.0f,
                            float hi = 1.0f) noexcept {
    const float stepFrac =
        (rate == KnobRate::Fine) ? 0.00090f : ((rate == KnobRate::Fast) ? 0.00800f : 0.00420f);
    const float span = hi - lo;
    const float delta = static_cast<float>(ticks) * stepFrac * span;
    return clampf(current + delta, lo, hi);
}

inline int32_t stepClampedDetent(int32_t currentVal, int32_t maxVal, int32_t ticks, KnobRate rate,
                                 int32_t& accum) noexcept {
    const int32_t threshold = (rate == KnobRate::Fine) ? 8 : ((rate == KnobRate::Fast) ? 2 : 4);
    if ((ticks > 0 && accum < 0) || (ticks < 0 && accum > 0)) {
        accum = 0;
    }
    accum += ticks;
    const int32_t steps = accum / threshold;
    if (steps == 0) {
        if ((currentVal <= 0 && accum < 0) || (currentVal >= maxVal && accum > 0)) {
            accum = 0;
        }
        return currentVal;
    }
    accum -= steps * threshold;
    int32_t next = currentVal + steps;
    if (next <= 0) {
        next = 0;
        if (accum < 0) {
            accum = 0;
        }
    } else if (next >= maxVal) {
        next = maxVal;
        if (accum > 0) {
            accum = 0;
        }
    }
    return next;
}

} // namespace ruido
