#pragma once

#include "core/DspMath.h"

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
        coeff_ = 1.0f - fastExp(-1.0f / (timeConstantSec * sampleRate));
        coeff_ = clampf(coeff_, 1.0e-6f, 1.0f);
    }

    void snapTo(float value) noexcept {
        const float clean = sanitizeState(value);
        state_ = clean;
        target_ = clean;
    }

    void setTarget(float target) noexcept { target_ = sanitizeState(target); }

    float target() const noexcept { return target_; }

    float current() const noexcept { return state_; }

    float next() noexcept {
        if (state_ == target_) {
            return state_;
        }
        const float diff = target_ - state_;
        const float nextState = state_ + coeff_ * diff;
        const float rem = absf(target_ - nextState);
        const float snapTol = 1.0e-5f * (1.0f + absf(target_));
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

// Asymmetric optocoupler (CdS vactrol) smoother: fast LED attack on finger press,
// organic photoconductive release tail on finger lift, with denormal-safe target snap.
class VactrolSmoother {
  public:
    constexpr VactrolSmoother() noexcept = default;

    void init(float initialValue, float attackSec, float releaseSec, float sampleRate) noexcept {
        const float clean = sanitizeState(initialValue);
        state_ = clean;
        target_ = clean;
        riseCoeff_ = computeCoeff(attackSec, sampleRate);
        fallCoeff_ = computeCoeff(releaseSec, sampleRate);
    }

    void snapTo(float value) noexcept {
        const float clean = sanitizeState(value);
        state_ = clean;
        target_ = clean;
    }

    void setTarget(float target) noexcept { target_ = sanitizeState(target); }

    float current() const noexcept { return state_; }

    float next() noexcept {
        if (state_ == target_) {
            return state_;
        }
        const float diff = target_ - state_;
        const float coeff = (diff > 0.0f) ? riseCoeff_ : fallCoeff_;
        const float nextState = state_ + coeff * diff;
        const float rem = absf(target_ - nextState);
        const float snapTol = 1.0e-5f * (1.0f + absf(target_));
        if (nextState == state_ || rem <= snapTol) {
            state_ = target_;
            return state_;
        }
        state_ = nextState;
        return state_;
    }

  private:
    static float computeCoeff(float tauSec, float sampleRate) noexcept {
        if (!(tauSec > 0.0f) || !(sampleRate > 0.0f)) {
            return 1.0f;
        }
        return clampf(1.0f - fastExp(-1.0f / (tauSec * sampleRate)), 1.0e-6f, 1.0f);
    }

    float state_ = 0.0f;
    float target_ = 0.0f;
    float riseCoeff_ = 1.0f;
    float fallCoeff_ = 1.0f;
};

// Second-order critically-damped (2-pole cascade, C^1 continuous velocity, 0% overshoot)
// analog potentiometer smoother. Eliminates t=0 slope kinks and zipper plateaus between
// discrete rotary encoder steps while snapping cleanly at rest to prevent denormals.
class CriticallyDampedSmoother {
  public:
    constexpr CriticallyDampedSmoother() noexcept = default;

    void init(float initialValue, float poleTimeConstantSec, float sampleRate) noexcept {
        const float clean = sanitizeState(initialValue);
        s1_ = clean;
        s2_ = clean;
        target_ = clean;
        setTimeConstant(poleTimeConstantSec, sampleRate);
    }

    void setTimeConstant(float poleTimeConstantSec, float sampleRate) noexcept {
        if (!(poleTimeConstantSec > 0.0f) || !(sampleRate > 0.0f)) {
            coeff_ = 1.0f;
            return;
        }
        coeff_ = clampf(1.0f - fastExp(-1.0f / (poleTimeConstantSec * sampleRate)), 1.0e-6f, 1.0f);
    }

    void snapTo(float value) noexcept {
        const float clean = sanitizeState(value);
        s1_ = clean;
        s2_ = clean;
        target_ = clean;
    }

    void setTarget(float target) noexcept { target_ = sanitizeState(target); }

    float target() const noexcept { return target_; }

    float current() const noexcept { return s2_; }

    bool isSettled() const noexcept { return s2_ == target_ && s1_ == target_; }

    float next() noexcept {
        if (s2_ == target_ && s1_ == target_) {
            return s2_;
        }
        const float next1 = s1_ + coeff_ * (target_ - s1_);
        const float next2 = s2_ + coeff_ * (next1 - s2_);
        const float snapTol = 1.0e-5f * (1.0f + absf(target_));
        if ((next1 == s1_ && next2 == s2_) ||
            (absf(target_ - next2) <= snapTol && absf(target_ - next1) <= snapTol)) {
            s1_ = target_;
            s2_ = target_;
            return s2_;
        }
        s1_ = next1;
        s2_ = next2;
        return s2_;
    }

  private:
    float s1_ = 0.0f;
    float s2_ = 0.0f;
    float target_ = 0.0f;
    float coeff_ = 1.0f;
};

// Logarithmic-pitch (log2 Hz) critically-damped smoother for oscillator & filter frequencies.
// Slews at uniform musical cents/ms across 0.02 Hz - 20 kHz (never rushing bass octaves) and
// preserves exact bit-identical targetHz when settled at zero stationary CPU cost.
class LogFreqSmoother {
  public:
    constexpr LogFreqSmoother() noexcept = default;

    void init(float initialHz, float poleTimeConstantSec, float sampleRate) noexcept {
        const float cleanHz = clampf(sanitizeState(initialHz), 0.005f, 100000.0f);
        targetHz_ = cleanHz;
        currentHz_ = cleanHz;
        octSmoother_.init(fastLog2(cleanHz), poleTimeConstantSec, sampleRate);
    }

    void snapTo(float hz) noexcept {
        const float cleanHz = clampf(sanitizeState(hz), 0.005f, 100000.0f);
        targetHz_ = cleanHz;
        currentHz_ = cleanHz;
        octSmoother_.snapTo(fastLog2(cleanHz));
    }

    void setTarget(float hz) noexcept {
        const float cleanHz = clampf(sanitizeState(hz), 0.005f, 100000.0f);
        if (cleanHz == targetHz_) {
            return;
        }
        targetHz_ = cleanHz;
        octSmoother_.setTarget(fastLog2(cleanHz));
    }

    float current() const noexcept { return currentHz_; }

    float next() noexcept {
        if (octSmoother_.isSettled()) {
            return currentHz_;
        }
        const float oct = octSmoother_.next();
        currentHz_ = octSmoother_.isSettled() ? targetHz_ : fastExp2(oct);
        return currentHz_;
    }

  private:
    CriticallyDampedSmoother octSmoother_{};
    float targetHz_ = 440.0f;
    float currentHz_ = 440.0f;
};

} // namespace ruido
