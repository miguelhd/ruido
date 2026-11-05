#include "dsp/SchmittOscillatorBank.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

// Subtle per-inverter CMOS hysteresis threshold duty offsets around 50% modeled after real
// CD40106 manufacturing spread (~49.2% to ~50.9%), yielding organic ~-35 dB 2nd/4th harmonics.
constexpr std::array<float, kNumOscillators> kInverterDutyTrim = {
    -0.0080f, // ~0.4920 at 50%
    +0.0070f, // ~0.5070 at 50%
    -0.0040f, // ~0.4960 at 50%
    +0.0090f, // ~0.5090 at 50%
};

constexpr std::array<uint32_t, kNumOscillators> kInitialRngSeeds = {
    0x13579BDFu,
    0x2468ACE1u,
    0x9E3779B9u,
    0x7F4A7C15u,
};

constexpr std::array<uint32_t, kNumOscillators> kInitialPhases = {
    0x00000000u,
    0x3504F333u,
    0x6A09E667u,
    0xBB67AE85u,
};

constexpr float kUint32ToNormFloat = 2.3283064365386963e-10f; // 1.0 / 2^32
constexpr float kHalfPhaseToBipolar = 9.31322575e-10f;        // 2.0 / 2^31
constexpr uint32_t kHalfPhase = 0x80000000u;
constexpr uint32_t kMaxPhaseInc = 0x1F000000u; // >= 4.13 ticks per half-cycle (< fs/8 safe BLEP)

} // namespace

SchmittOscillatorBank::SchmittOscillatorBank() noexcept {
    reset(kOversampledRate);
}

void SchmittOscillatorBank::reset(float sampleRate) noexcept {
    sampleRate_ = (sampleRate > 1000.0f) ? sampleRate : kOversampledRate;
    hzToPhaseU32_ = 4294967296.0f / sampleRate_;

    // 1.5 Hz AC coupling / capacitor roof-tilt corner:
    // preserves 99.5% weight on 15 Hz sub-bass while sculpting < 2 Hz into sharp relaxation pulses.
    rcHighpassPole_ = fastExp(-kTwoPi * 1.5f / sampleRate_);
    // ~0.45 ms shared Vdd rail decoupling recovery time constant
    railDecayCoeff_ = fastExp(-1.0f / (0.00045f * sampleRate_));
    prevRisingMask_ = 0u;

    for (size_t i = 0; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];
        v.phase = kInitialPhases[i];
        v.currentLevel = (v.phase < kHalfPhase) ? 1.0f : -1.0f;
        v.naiveD1 = v.currentLevel;
        v.naiveD2 = v.currentLevel;
        v.blepResidual = {0.0f, 0.0f, 0.0f, 0.0f};
        v.rcCapState = 0.0f;
        v.rcPrevInput = v.currentLevel;
        v.rngState = kInitialRngSeeds[i];
        v.driftFast = 0.0f;
        v.driftSlow = 0.0f;
        v.driftCounter = static_cast<uint32_t>(i * 16u);
        v.cachedDriftRatio = 1.0f;
        v.cachedDriftNorm = 0.0f;
        v.lastEdgePulse = 0.0f;
        voiceRailSag_[i] = 0.0f;
    }
}

void SchmittOscillatorBank::addCubicBsplineBlep(std::array<float, 4>& res, float d,
                                                float delta) noexcept {
    const float dc = clampf(d, 0.0f, 1.0f);
    const float d2 = dc * dc;
    const float u1 = 1.0f - dc;
    const float u1_2 = u1 * u1;

    // 4-point integrated 3rd-order B-spline step residual at [n-2, n-1, n, n+1]
    const float rM2 = (d2 * d2) * (1.0f / 24.0f);
    const float x = dc - 1.0f;
    const float x2 = x * x;
    const float rM1 = 0.5f + x * (0.66666667f - x2 * 0.33333333f - (x2 * x) * 0.125f);
    const float r0 = (0.5f + dc * (0.66666667f - d2 * 0.33333333f + (d2 * dc) * 0.125f)) - 1.0f;
    const float rP1 = -(u1_2 * u1_2) * (1.0f / 24.0f);

    res[0] += delta * rM2;
    res[1] += delta * rM1;
    res[2] += delta * r0;
    res[3] += delta * rP1;
}

float SchmittOscillatorBank::nextWhiteNoise(uint32_t& state) noexcept {
    // xorshift32 PRNG mapped to [-1.0f, +1.0f]
    uint32_t x = (state != 0u) ? state : 0x6D2B79F5u;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    state = x;
    return static_cast<float>(static_cast<int32_t>(x)) * 4.65661287e-10f;
}

OscillatorBankOutput
SchmittOscillatorBank::processSample(const std::array<float, kNumOscillators>& freqHz,
                                     const std::array<float, kNumOscillators>& pulseWidth,
                                     const std::array<float, kNumOscillators>& drift,
                                     float circuitCoupling, bool freezeDrift,
                                     float hardSync) noexcept {
    OscillatorBankOutput out{};
    const float coupling = clampf(circuitCoupling, 0.0f, 1.0f);
    const float syncAmt = clampf(hardSync, 0.0f, 1.0f);
    const float syncShaped = smoothstep01(syncAmt * 1.45f);
    const uint32_t hardSyncThreshold =
        (syncShaped <= 1.0e-4f)
            ? 0xFFFFFFFFu
            : ((syncShaped >= 0.9999f)
                   ? 0u
                   : static_cast<uint32_t>((1.0f - syncShaped) * 4294967040.0f));
    bool osc0Wrapped = false;
    float osc0WrapD = 0.0f;
    // Two-regime ALGORITHM coupling:
    //  - [0.00, 0.45]: smooth CD40106 Vdd rail sag & subtle organic chorus shimmer (25% default)
    //  - (0.45, 1.00]: starved-rail voltage collapse -> PWM duty warp, subharmonic sync locks,
    //                  and hard phase-lock tearing across the 4 inverters
    const float chaos = smoothstep01((coupling - 0.45f) * (1.0f / 0.55f));
    const float couplingGain = coupling * (0.10f + 0.65f * coupling) + 4.80f * chaos * chaos;
    const float minPull = 0.55f - 0.43f * chaos;
    const float maxPull = 1.75f + 2.75f * chaos;
    const uint8_t lastRising = prevRisingMask_;
    const uint32_t syncCapturePhase =
        0xFFFFFFFFu - static_cast<uint32_t>(chaos * 601295424.0f); // upper 14% Vt+ threshold window

    // Update per-voice rail sag states & total static output load from previous sample
    float totalRailSag = 0.0f;
    float totalGateLoad = 0.0f;
    for (size_t i = 0; i < kNumOscillators; ++i) {
        voiceRailSag_[i] =
            sanitizeState(voiceRailSag_[i] * railDecayCoeff_ + voices_[i].lastEdgePulse);
        voices_[i].lastEdgePulse = 0.0f;
        totalRailSag += voiceRailSag_[i];
        totalGateLoad += voices_[i].currentLevel;
    }

    for (size_t i = 0; i < kNumOscillators; ++i) {
        VoiceState& v = voices_[i];

        // Subsampled thermal pitch drift update every 64 oversampled ticks (~2.75 kHz rate)
        if (!freezeDrift && ((v.driftCounter++ & 63u) == 0u)) {
            const float dAmt = clampf(drift[i], 0.0f, 1.0f);
            if (enableDrift_) {
                const float wn = nextWhiteNoise(v.rngState);
                v.driftFast = sanitizeState(0.996f * v.driftFast + 0.004f * wn);
                v.driftSlow = sanitizeState(0.9997f * v.driftSlow + 0.0003f * v.driftFast);
                if (dAmt > 1.0e-5f) {
                    const float wanderNorm = clampf(
                        0.35f * (v.driftFast * 12.5f) + 0.65f * (v.driftSlow * 45.0f), -1.0f, 1.0f);
                    // 0% = 0c locked, 10% default = ~+-2.2c analog warmth, 100% = ~+-120c drift
                    const float maxDriftOct = dAmt * (0.018f + 0.082f * dAmt * dAmt);
                    v.cachedDriftRatio = fastExp2(wanderNorm * maxDriftOct);
                    v.cachedDriftNorm = wanderNorm * dAmt;
                } else {
                    v.cachedDriftRatio = 1.0f;
                    v.cachedDriftNorm = 0.0f;
                }
            } else {
                v.cachedDriftRatio = 1.0f;
                v.cachedDriftNorm = 0.0f;
            }
        }

        // Compute neighbor-only Vdd rail sag + starved-CMOS gate-load duty asymmetry
        float railPullRatio = 1.0f;
        if (couplingGain > 0.0f) {
            const float neighborSag = totalRailSag - voiceRailSag_[i];
            const float phaseNorm = static_cast<float>(v.phase) * kUint32ToNormFloat;
            const float dHalf = absf(2.0f * phaseNorm - 1.0f);
            const float signedPrc = dHalf * dHalf * (3.0f - 2.0f * dHalf) * 2.0f - 1.0f;
            float pull = 1.0f - couplingGain * neighborSag * signedPrc;
            if (chaos > 0.0f) {
                const float neighborLoad = (totalGateLoad - v.currentLevel) * (1.0f / 3.0f);
                const float asymSign = (v.currentLevel > 0.0f) ? 0.36f : -0.36f;
                pull += chaos * asymSign * neighborLoad;
            }
            railPullRatio = clampf(pull, minPull, maxPull);
        }

        const float targetHz =
            clampf(freqHz[i] * v.cachedDriftRatio * railPullRatio, 0.005f, 21200.0f);
        const float baseIncFloat = targetHz * hzToPhaseU32_;

        // Piecewise-slope CD40106 diode-shunted PWM around fixed kHalfPhase (0x80000000):
        // preserves exact 1/f fundamental pitch invariance and mid-cycle response down to 0.05 Hz
        // while guarding 4-tap Cubic B-Spline PolyBLEP non-overlap near Nyquist.
        const float pwClamped = clampf(pulseWidth[i], kPulseWidthMin, kPulseWidthMax);
        const float trimWeight = clampf(1.0f - absf(2.0f * pwClamped - 1.0f), 0.0f, 1.0f);
        const float minSafeDuty =
            clampf(baseIncFloat * (0.5f / static_cast<float>(kMaxPhaseInc)), kPulseWidthMin, 0.49f);
        const float effDuty =
            clampf(pwClamped + kInverterDutyTrim[i] * trimWeight, minSafeDuty, 1.0f - minSafeDuty);

        const float halfBaseInc = 0.5f * baseIncFloat;
        const float incHighFloat = halfBaseInc / effDuty;
        const float incLowFloat = halfBaseInc / (1.0f - effDuty);
        uint32_t incHighU32 = (incHighFloat >= 1.0f) ? static_cast<uint32_t>(incHighFloat) : 1u;
        uint32_t incLowU32 = (incLowFloat >= 1.0f) ? static_cast<uint32_t>(incLowFloat) : 1u;
        if (incHighU32 > kMaxPhaseInc) {
            incHighU32 = kMaxPhaseInc;
        }
        if (incLowU32 > kMaxPhaseInc) {
            incLowU32 = kMaxPhaseInc;
        }

        const uint8_t neighborRose =
            static_cast<uint8_t>((lastRising | out.risingMask) & static_cast<uint8_t>(~(1u << i)));
        if (i > 0u && osc0Wrapped && syncAmt > 1.0e-4f && v.phase >= hardSyncThreshold) {
            // Band-limited master->slave hard/threshold sync (OSC 1 -> OSC 2..4) at osc0WrapD
            const float preFrac = 1.0f - osc0WrapD;
            if (v.currentLevel < 0.0f) {
                const uint32_t remToWrap = 0u - v.phase;
                const float preLow = preFrac * static_cast<float>(incLowU32);
                if (static_cast<float>(remToWrap) < preLow && incLowU32 > 0u) {
                    const float dNat =
                        1.0f - static_cast<float>(remToWrap) / static_cast<float>(incLowU32);
                    addCubicBsplineBlep(v.blepResidual, dNat, 2.0f);
                } else {
                    addCubicBsplineBlep(v.blepResidual, osc0WrapD, 2.0f);
                }
            } else {
                const uint32_t remToHalf = kHalfPhase - v.phase;
                const float preHigh = preFrac * static_cast<float>(incHighU32);
                if (static_cast<float>(remToHalf) < preHigh && incHighU32 > 0u) {
                    const float dFall =
                        1.0f - static_cast<float>(remToHalf) / static_cast<float>(incHighU32);
                    addCubicBsplineBlep(v.blepResidual, dFall, -2.0f);
                    addCubicBsplineBlep(v.blepResidual, osc0WrapD, 2.0f);
                }
            }
            v.lastEdgePulse += 0.22f;
            out.risingMask |= static_cast<uint8_t>(1u << i);
            uint32_t postHigh =
                static_cast<uint32_t>(osc0WrapD * static_cast<float>(incHighU32) + 0.5f);
            if (postHigh >= kHalfPhase) {
                postHigh = kHalfPhase - 1u;
            }
            v.phase = postHigh;
            v.currentLevel = 1.0f;
        } else if (chaos > 0.02f && neighborRose != 0u && v.currentLevel < 0.0f &&
                   v.phase >= syncCapturePhase) {
            // Starved-rail Vt+ threshold collapse: band-limited hard-sync snap at d = 0.5
            v.phase = (incHighU32 >> 1u);
            addCubicBsplineBlep(v.blepResidual, 0.5f, 2.0f);
            v.lastEdgePulse += 0.22f;
            v.currentLevel = 1.0f;
            out.risingMask |= static_cast<uint8_t>(1u << i);
            if (i == 0u) {
                osc0Wrapped = true;
                osc0WrapD = 0.5f;
            }
        } else if (v.phase < kHalfPhase) {
            const uint32_t remToHalf = kHalfPhase - v.phase;
            if (incHighU32 < remToHalf) {
                v.phase += incHighU32;
            } else {
                const uint32_t overHigh = incHighU32 - remToHalf;
                const float d = static_cast<float>(overHigh) / static_cast<float>(incHighU32);
                addCubicBsplineBlep(v.blepResidual, d, -2.0f);
                v.lastEdgePulse -= 0.16f;
                uint32_t postLow = static_cast<uint32_t>(d * static_cast<float>(incLowU32) + 0.5f);
                if (postLow >= kHalfPhase) {
                    postLow = kHalfPhase - 1u;
                }
                v.phase = kHalfPhase + postLow;
                v.currentLevel = -1.0f;
            }
        } else {
            const uint32_t remToWrap = 0u - v.phase;
            if (incLowU32 < remToWrap) {
                v.phase += incLowU32;
            } else {
                const uint32_t overLow = incLowU32 - remToWrap;
                const float d = static_cast<float>(overLow) / static_cast<float>(incLowU32);
                addCubicBsplineBlep(v.blepResidual, d, 2.0f);
                v.lastEdgePulse += 0.22f;
                out.risingMask |= static_cast<uint8_t>(1u << i);
                if (i == 0u) {
                    osc0Wrapped = true;
                    osc0WrapD = d;
                }
                uint32_t postHigh =
                    static_cast<uint32_t>(d * static_cast<float>(incHighU32) + 0.5f);
                if (postHigh >= kHalfPhase) {
                    postHigh = kHalfPhase - 1u;
                }
                v.phase = postHigh;
                v.currentLevel = 1.0f;
            }
        }

        // Read completed band-limited sample (n - 2) and shift 4-tap BLEP delay line
        const float bandLimitedSquare = v.naiveD2 + v.blepResidual[0];
        v.naiveD2 = v.naiveD1;
        v.naiveD1 = v.currentLevel;
        v.blepResidual[0] = v.blepResidual[1];
        v.blepResidual[1] = v.blepResidual[2];
        v.blepResidual[2] = v.blepResidual[3];
        v.blepResidual[3] = 0.0f;

        // Apply subtle RC coupling capacitor roof tilt + DC block:
        // y[n] = pole * (y[n-1] + x[n] - x[n-1])
        float voiceSample = bandLimitedSquare;
        if (enableRcTilt_) {
            const float rcOut =
                rcHighpassPole_ * (v.rcCapState + bandLimitedSquare - v.rcPrevInput);
            v.rcPrevInput = bandLimitedSquare;
            v.rcCapState = sanitizeState(rcOut);
            voiceSample = v.rcCapState;
        }

        out.samples[i] = voiceSample;
        out.capRamp[i] =
            (v.phase < kHalfPhase)
                ? (static_cast<float>(v.phase) * kHalfPhaseToBipolar - 1.0f)
                : (1.0f - static_cast<float>(v.phase - kHalfPhase) * kHalfPhaseToBipolar);
        out.gateHigh[i] = (v.currentLevel > 0.0f) ? 1.0f : 0.0f;
        out.driftNorm[i] = v.cachedDriftNorm;
    }

    prevRisingMask_ = out.risingMask;
    return out;
}

} // namespace ruido
