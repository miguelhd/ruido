#include "dsp/SchmittOscillatorBank.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

// Subtle per-inverter CMOS hysteresis threshold duty ratios modeled after real CD40106
// manufacturing spread (~49.2% to ~50.9%), yielding organic ~-35 dB 2nd/4th harmonics.
constexpr std::array<uint32_t, kNumOscillators> kInverterDutyThresholds = {
    2113123901u, // ~0.4920
    2177548411u, // ~0.5070
    2130303770u, // ~0.4960
    2186138346u, // ~0.5090
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
constexpr uint32_t kMaxPhaseInc = 0x1F000000u;                // ~21.37 kHz (< fs/8 safe BLEP)

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
        v.dutyThreshold = kInverterDutyThresholds[i];
        v.currentLevel = (v.phase < v.dutyThreshold) ? 1.0f : -1.0f;
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
                                     float circuitCoupling) noexcept {
    OscillatorBankOutput out{};
    const float coupling = clampf(circuitCoupling, 0.0f, 1.0f);
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
        if ((v.driftCounter++ & 63u) == 0u) {
            if (enableDrift_) {
                const float wn = nextWhiteNoise(v.rngState);
                v.driftFast = sanitizeState(0.996f * v.driftFast + 0.004f * wn);
                v.driftSlow = sanitizeState(0.9997f * v.driftSlow + 0.0003f * v.driftFast);
                // ~+-2.2 cents organic analog thermal wander
                const float driftOct = (0.35f * v.driftFast + 0.65f * v.driftSlow) * 0.0035f;
                v.cachedDriftRatio = 1.0f + driftOct * 0.69314718f;
            } else {
                v.cachedDriftRatio = 1.0f;
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
        const float incFloat = targetHz * hzToPhaseU32_;
        uint32_t incU32 = (incFloat >= 1.0f) ? static_cast<uint32_t>(incFloat) : 1u;
        if (incU32 > kMaxPhaseInc) {
            incU32 = kMaxPhaseInc;
        }

        const uint8_t neighborRose =
            static_cast<uint8_t>((lastRising | out.risingMask) & static_cast<uint8_t>(~(1u << i)));
        if (chaos > 0.02f && neighborRose != 0u && v.currentLevel < 0.0f &&
            v.phase >= syncCapturePhase) {
            // Starved-rail Vt+ threshold collapse: band-limited hard-sync snap at d = 0.5
            v.phase = (incU32 >> 1u);
            addCubicBsplineBlep(v.blepResidual, 0.5f, 2.0f);
            v.lastEdgePulse += 0.22f;
            v.currentLevel = 1.0f;
            out.risingMask |= static_cast<uint8_t>(1u << i);
        } else {
            // Advance 32-bit unsigned phase accumulator
            v.phase += incU32;

            // Only compute reciprocal 1 / incU32 on the rare samples where an edge occurs
            const uint32_t distPastDuty = v.phase - v.dutyThreshold;
            const bool fell = (distPastDuty < incU32);
            const bool rose = (v.phase < incU32);
            if (fell || rose) {
                const float invIncFloat = 1.0f / static_cast<float>(incU32);
                if (fell) {
                    const float d = static_cast<float>(distPastDuty) * invIncFloat;
                    addCubicBsplineBlep(v.blepResidual, d, -2.0f);
                    v.lastEdgePulse -= 0.16f;
                }
                if (rose) {
                    const float d = static_cast<float>(v.phase) * invIncFloat;
                    addCubicBsplineBlep(v.blepResidual, d, 2.0f);
                    v.lastEdgePulse += 0.22f;
                    out.risingMask |= static_cast<uint8_t>(1u << i);
                }
                v.currentLevel = (v.phase < v.dutyThreshold) ? 1.0f : -1.0f;
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
        out.gateHigh[i] = (v.currentLevel > 0.0f) ? 1.0f : 0.0f;
    }

    prevRisingMask_ = out.risingMask;
    return out;
}

} // namespace ruido
