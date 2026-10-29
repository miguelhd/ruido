#pragma once

#include "core/DspMath.h"

#include <cmath>

namespace ruido {

// Clickless one-pole exponential smoother with automatic snap-to-target to prevent
// denormal creep and save FPU work when controls are stationary.
class OnePoleSmoother {
public:
    constexpr OnePoleSmoother() noexcept = default;

    void init(float initialValue, float timeConstantSec, float sampleRate) noexcept {
        const float clean = sanitizeState(initialValue);
        state_ = clean;
        target_ = clean;
        setTimeConstant(timeConstantSec, sampleRate);
    }

    void setTimeConstant(float timeConstantSec, float sampleRate) noexcept {
        if (!(timeConstantSec > 0.0f) || !(sampleRate > 0.0f)) {
            coeff_ = 1.0f;
            return;
        }
        coeff_ = 1.0f - ::expf(-1.0f / (timeConstantSec * sampleRate));
        coeff_ = clampf(coeff_, 1.0e-6f, 1.0f);
    }

    void snapTo(float value) noexcept {
        const float clean = sanitizeState(value);
        state_ = clean;
        target_ = clean;
    }

    void setTarget(float target) noexcept {
        target_ = sanitizeState(target);
    }

    float target() const noexcept {
        return target_;
    }

    float current() const noexcept {
        return state_;
    }

    float next() noexcept {
        if (state_ == target_) {
            return state_;
        }
        const float diff = target_ - state_;
        const float nextState = state_ + coeff_ * diff;
        const float rem = ::fabsf(target_ - nextState);
        const float snapTol = 1.0e-5f * (1.0f + ::fabsf(target_));
        if (nextState == state_ || rem <= snapTol) {
            state_ = target_;
            return state_;
        }
        state_ = nextState;
        return state_;
    }

private:
    float state_ = 0.0f;
    float target_ = 0.0f;
    float coeff_ = 1.0f;
};

} // namespace ruido
