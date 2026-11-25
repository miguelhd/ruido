#pragma once

#include "core/DspMath.h"

#include <array>
#include <cstddef>

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
#define RUIDO_SMOOTHER_COLD RUIDO_NOINLINE
#else
#define RUIDO_SMOOTHER_COLD inline
#endif

namespace ruido {

// Exact closed-form (1 - coeff)^steps decay factor via straight-line squaring for common sub-block
// sizes (16/8/4) and binary exponentiation otherwise — eliminates serial per-sample smoother loops.
RUIDO_ALWAYS_INLINE float powPoleDecay(float coeff, uint32_t steps) noexcept {
    const float r = 1.0f - coeff;
    if (steps == 16u) {
        const float r2 = r * r;
        const float r4 = r2 * r2;
        const float r8 = r4 * r4;
        return r8 * r8;
    }
    if (steps == 8u) {
        const float r2 = r * r;
        const float r4 = r2 * r2;
        return r4 * r4;
    }
    if (steps == 4u) {
        const float r2 = r * r;
        return r2 * r2;
    }
    float acc = 1.0f;
    float base = r;
    uint32_t n = steps;
    while (n != 0u) {
        if ((n & 1u) != 0u) {
            acc *= base;
        }
        n >>= 1u;
        if (n != 0u) {
            base *= base;
        }
    }
    return acc;
}

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

    RUIDO_ALWAYS_INLINE float next() noexcept {
        if (state_ == target_) {
            return state_;
        }
        return stepActive();
    }

    RUIDO_ALWAYS_INLINE float nextN(uint32_t steps) noexcept {
        if (state_ == target_ || steps == 0u) {
            return state_;
        }
        return advanceActive(steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceActive(uint32_t steps) noexcept {
        if (steps == 1u) {
            return stepActive();
        }
        const float nextState = target_ + powPoleDecay(coeff_, steps) * (state_ - target_);
        const float snapTol = 1.0e-5f * (1.0f + absf(target_));
        if (nextState == state_ || absf(target_ - nextState) <= snapTol) {
            state_ = target_;
            return target_;
        }
        state_ = nextState;
        return nextState;
    }

    float stepActive() noexcept {
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

    float state_ = 0.0f;
    float target_ = 0.0f;
    float coeff_ = 1.0f;
};

// Multi-channel one-pole smoother sharing a single time-constant coefficient across N channels
// to eliminate duplicate `coeff_` storage inside Core 1's 48 KB SRAM budget.
template <size_t N> class OnePoleSmootherBank {
  public:
    constexpr OnePoleSmootherBank() noexcept = default;

    void setTimeConstant(float timeConstantSec, float sampleRate) noexcept {
        if (!(timeConstantSec > 0.0f) || !(sampleRate > 0.0f)) {
            coeff_ = 1.0f;
            return;
        }
        coeff_ = clampf(1.0f - fastExp(-1.0f / (timeConstantSec * sampleRate)), 1.0e-6f, 1.0f);
    }

    void snapChannel(size_t idx, float value) noexcept {
        const float clean = sanitizeState(value);
        state_[idx] = clean;
        target_[idx] = clean;
    }

    void setTarget(size_t idx, float target) noexcept { target_[idx] = sanitizeState(target); }

    float target(size_t idx) const noexcept { return target_[idx]; }

    float current(size_t idx) const noexcept { return state_[idx]; }

    RUIDO_ALWAYS_INLINE float next(size_t idx) noexcept {
        const float cur = state_[idx];
        if (cur == target_[idx]) {
            return cur;
        }
        return stepActive(idx);
    }

    RUIDO_ALWAYS_INLINE float nextN(size_t idx, uint32_t steps) noexcept {
        if (state_[idx] == target_[idx] || steps == 0u) {
            return state_[idx];
        }
        return advanceChannel(idx, steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceChannel(size_t idx, uint32_t steps) noexcept {
        if (steps == 1u) {
            return stepActive(idx);
        }
        const float cur = state_[idx];
        const float tgt = target_[idx];
        const float nextState = tgt + powPoleDecay(coeff_, steps) * (cur - tgt);
        const float snapTol = 1.0e-5f * (1.0f + absf(tgt));
        if (nextState == cur || absf(tgt - nextState) <= snapTol) {
            state_[idx] = tgt;
            return tgt;
        }
        state_[idx] = nextState;
        return nextState;
    }

    float stepActive(size_t idx) noexcept {
        const float cur = state_[idx];
        const float tgt = target_[idx];
        const float nextState = cur + coeff_ * (tgt - cur);
        const float snapTol = 1.0e-5f * (1.0f + absf(tgt));
        if (nextState == cur || absf(tgt - nextState) <= snapTol) {
            state_[idx] = tgt;
            return tgt;
        }
        state_[idx] = nextState;
        return nextState;
    }

    std::array<float, N> state_{};
    std::array<float, N> target_{};
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

    float target() const noexcept { return target_; }

    float current() const noexcept { return state_; }

    RUIDO_ALWAYS_INLINE float next() noexcept {
        if (state_ == target_) {
            return state_;
        }
        return stepActive();
    }

    RUIDO_ALWAYS_INLINE float nextN(uint32_t steps) noexcept {
        if (state_ == target_ || steps == 0u) {
            return state_;
        }
        return advanceActive(steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceActive(uint32_t steps) noexcept {
        if (steps == 1u) {
            return stepActive();
        }
        const float coeff = (target_ > state_) ? riseCoeff_ : fallCoeff_;
        const float nextState = target_ + powPoleDecay(coeff, steps) * (state_ - target_);
        const float snapTol = 1.0e-5f * (1.0f + absf(target_));
        if (nextState == state_ || absf(target_ - nextState) <= snapTol) {
            state_ = target_;
            return target_;
        }
        state_ = nextState;
        return nextState;
    }

    float stepActive() noexcept {
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

    RUIDO_ALWAYS_INLINE bool isSettled() const noexcept { return s2_ == target_ && s1_ == target_; }

    RUIDO_ALWAYS_INLINE float next() noexcept {
        if (s2_ == target_ && s1_ == target_) {
            return s2_;
        }
        return stepActive();
    }

    RUIDO_ALWAYS_INLINE float nextN(uint32_t steps) noexcept {
        if ((s2_ == target_ && s1_ == target_) || steps == 0u) {
            return s2_;
        }
        return advanceActive(steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceActive(uint32_t steps) noexcept {
        if (steps == 1u) {
            return stepActive();
        }
        const float tgt = target_;
        const float e1 = s1_ - tgt;
        const float e2 = s2_ - tgt;
        const float rN = powPoleDecay(coeff_, steps);
        const float next1 = tgt + rN * e1;
        const float next2 = tgt + rN * (e2 + (static_cast<float>(steps) * coeff_) * e1);
        const float snapTol = 1.0e-5f * (1.0f + absf(tgt));
        if ((next1 == s1_ && next2 == s2_) ||
            (absf(tgt - next2) <= snapTol && absf(tgt - next1) <= snapTol)) {
            s1_ = tgt;
            s2_ = tgt;
            return tgt;
        }
        s1_ = next1;
        s2_ = next2;
        return next2;
    }

    float stepActive() noexcept {
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

    float s1_ = 0.0f;
    float s2_ = 0.0f;
    float target_ = 0.0f;
    float coeff_ = 1.0f;
};

// Multi-channel 2-pole critically-damped smoother sharing a single `coeff_` across N channels.
template <size_t N> class CriticallyDampedSmootherBank {
  public:
    constexpr CriticallyDampedSmootherBank() noexcept = default;

    void setTimeConstant(float poleTimeConstantSec, float sampleRate) noexcept {
        if (!(poleTimeConstantSec > 0.0f) || !(sampleRate > 0.0f)) {
            coeff_ = 1.0f;
            return;
        }
        coeff_ = clampf(1.0f - fastExp(-1.0f / (poleTimeConstantSec * sampleRate)), 1.0e-6f, 1.0f);
    }

    void snapChannel(size_t idx, float value) noexcept {
        const float clean = sanitizeState(value);
        s1_[idx] = clean;
        s2_[idx] = clean;
        target_[idx] = clean;
    }

    void setTarget(size_t idx, float target) noexcept { target_[idx] = sanitizeState(target); }

    bool isSettled(size_t idx) const noexcept {
        return s2_[idx] == target_[idx] && s1_[idx] == target_[idx];
    }

    RUIDO_ALWAYS_INLINE float next(size_t idx) noexcept {
        const float cur2 = s2_[idx];
        const float tgt = target_[idx];
        if (cur2 == tgt && s1_[idx] == tgt) {
            return cur2;
        }
        return stepActive(idx);
    }

    RUIDO_ALWAYS_INLINE float nextN(size_t idx, uint32_t steps) noexcept {
        const float tgt = target_[idx];
        if ((s2_[idx] == tgt && s1_[idx] == tgt) || steps == 0u) {
            return s2_[idx];
        }
        return advanceChannel(idx, steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceChannel(size_t idx, uint32_t steps) noexcept {
        if (steps == 1u) {
            return stepActive(idx);
        }
        const float cur1 = s1_[idx];
        const float cur2 = s2_[idx];
        const float tgt = target_[idx];
        const float e1 = cur1 - tgt;
        const float e2 = cur2 - tgt;
        const float rN = powPoleDecay(coeff_, steps);
        const float next1 = tgt + rN * e1;
        const float next2 = tgt + rN * (e2 + (static_cast<float>(steps) * coeff_) * e1);
        const float snapTol = 1.0e-5f * (1.0f + absf(tgt));
        if ((next1 == cur1 && next2 == cur2) ||
            (absf(tgt - next2) <= snapTol && absf(tgt - next1) <= snapTol)) {
            s1_[idx] = tgt;
            s2_[idx] = tgt;
            return tgt;
        }
        s1_[idx] = next1;
        s2_[idx] = next2;
        return next2;
    }

    float stepActive(size_t idx) noexcept {
        const float cur2 = s2_[idx];
        const float cur1 = s1_[idx];
        const float tgt = target_[idx];
        const float next1 = cur1 + coeff_ * (tgt - cur1);
        const float next2 = cur2 + coeff_ * (next1 - cur2);
        const float snapTol = 1.0e-5f * (1.0f + absf(tgt));
        if ((next1 == cur1 && next2 == cur2) ||
            (absf(tgt - next2) <= snapTol && absf(tgt - next1) <= snapTol)) {
            s1_[idx] = tgt;
            s2_[idx] = tgt;
            return tgt;
        }
        s1_[idx] = next1;
        s2_[idx] = next2;
        return next2;
    }

    std::array<float, N> s1_{};
    std::array<float, N> s2_{};
    std::array<float, N> target_{};
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

    float target() const noexcept { return targetHz_; }

    float current() const noexcept { return currentHz_; }

    RUIDO_ALWAYS_INLINE float next() noexcept {
        if (octSmoother_.isSettled()) {
            return currentHz_;
        }
        return stepActive();
    }

    RUIDO_ALWAYS_INLINE float nextN(uint32_t steps) noexcept {
        if (octSmoother_.isSettled() || steps == 0u) {
            return currentHz_;
        }
        return advanceActive(steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceActive(uint32_t steps) noexcept {
        const float oct = octSmoother_.nextN(steps);
        currentHz_ = octSmoother_.isSettled() ? targetHz_ : fastExp2(oct);
        return currentHz_;
    }

    float stepActive() noexcept {
        const float oct = octSmoother_.next();
        currentHz_ = octSmoother_.isSettled() ? targetHz_ : fastExp2(oct);
        return currentHz_;
    }
    CriticallyDampedSmoother octSmoother_{};
    float targetHz_ = 440.0f;
    float currentHz_ = 440.0f;
};

// Multi-channel log2-frequency critically-damped smoother sharing a single `coeff_` across N
// channels to eliminate duplicate coefficient storage inside Core 1 L1 SRAM.
template <size_t N> class LogFreqSmootherBank {
  public:
    constexpr LogFreqSmootherBank() noexcept = default;

    void setTimeConstant(float poleTimeConstantSec, float sampleRate) noexcept {
        octSmoothers_.setTimeConstant(poleTimeConstantSec, sampleRate);
    }

    void snapChannel(size_t idx, float hz) noexcept {
        const float cleanHz = clampf(sanitizeState(hz), 0.005f, 100000.0f);
        targetHz_[idx] = cleanHz;
        currentHz_[idx] = cleanHz;
        octSmoothers_.snapChannel(idx, fastLog2(cleanHz));
    }

    void setTarget(size_t idx, float hz) noexcept {
        const float cleanHz = clampf(sanitizeState(hz), 0.005f, 100000.0f);
        if (cleanHz == targetHz_[idx]) {
            return;
        }
        targetHz_[idx] = cleanHz;
        octSmoothers_.setTarget(idx, fastLog2(cleanHz));
    }

    RUIDO_ALWAYS_INLINE float nextN(size_t idx, uint32_t steps) noexcept {
        if (octSmoothers_.isSettled(idx) || steps == 0u) {
            return currentHz_[idx];
        }
        return advanceChannel(idx, steps);
    }

  private:
    RUIDO_SMOOTHER_COLD float advanceChannel(size_t idx, uint32_t steps) noexcept {
        const float oct = octSmoothers_.nextN(idx, steps);
        currentHz_[idx] = octSmoothers_.isSettled(idx) ? targetHz_[idx] : fastExp2(oct);
        return currentHz_[idx];
    }

    CriticallyDampedSmootherBank<N> octSmoothers_{};
    std::array<float, N> targetHz_{};
    std::array<float, N> currentHz_{};
};

} // namespace ruido
