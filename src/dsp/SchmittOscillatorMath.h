#pragma once

#include "core/Config.h"
#include "core/DspMath.h"

#include <array>
#include <cstdint>

namespace ruido {

// Subtle per-inverter CMOS hysteresis threshold duty offsets around 50% modeled after real
// CD40106 manufacturing spread (~49.2% to ~50.9%), yielding organic ~-35 dB 2nd/4th harmonics.
inline constexpr std::array<float, kNumOscillators> kInverterDutyTrim = {
    -0.0080f, // ~0.4920 at 50%
    +0.0070f, // ~0.5070 at 50%
    -0.0040f, // ~0.4960 at 50%
    +0.0090f, // ~0.5090 at 50%
};

inline constexpr std::array<uint32_t, kNumOscillators> kInitialRngSeeds = {
    0x13579BDFu, 0x2468ACE1u, 0x9E3779B9u, 0x7F4A7C15u};
inline constexpr std::array<uint32_t, kNumOscillators> kInitialPhases = {0x00000000u, 0x3504F333u,
                                                                         0x6A09E667u, 0xBB67AE85u};

inline constexpr float kSignedPhaseToBipolar = 4.6566128730773926e-10f; // 1.0 / 2^31
inline constexpr float kHalfPhaseToBipolar = 9.31322575e-10f;           // 2.0 / 2^31
inline constexpr uint32_t kHalfPhase = 0x80000000u;
inline constexpr uint32_t kMaxPhaseInc = 0x1F000000u; // >= 4.13 ticks per half-cycle (< fs/8 BLEP)
inline constexpr float kMaxPhaseIncFloat = static_cast<float>(kMaxPhaseInc);

struct CouplingParams {
    float chaos;
    float couplingGain;
    float minPull;
    float maxPull;
    uint32_t syncCapturePhase;
};

inline CouplingParams computeCouplingParams(float circuitCoupling) noexcept {
    const float coupling = clampf(circuitCoupling, 0.0f, 1.0f);
    // Two-regime ALGORITHM coupling:
    //  - [0.00, 0.45]: smooth CD40106 Vdd rail sag & subtle organic chorus shimmer (25% default)
    //  - (0.45, 1.00]: starved-rail voltage collapse -> PWM duty warp, subharmonic sync locks,
    //                  and hard phase-lock tearing across the 4 inverters
    const float chaos = smoothstep01((coupling - 0.45f) * (1.0f / 0.55f));
    const float couplingGain = coupling * (0.10f + 0.65f * coupling) + 4.80f * chaos * chaos;
    const float minPull = 0.55f - 0.43f * chaos;
    const float maxPull = 1.75f + 2.75f * chaos;
    const uint32_t syncCapturePhase =
        0xFFFFFFFFu - static_cast<uint32_t>(chaos * 601295424.0f); // upper 14% Vt+ threshold window
    return {chaos, couplingGain, minPull, maxPull, syncCapturePhase};
}

inline uint32_t computeHardSyncThreshold(float hardSync) noexcept {
    const float syncShaped = smoothstep01(clampf(hardSync, 0.0f, 1.0f) * 1.45f);
    if (syncShaped <= 1.0e-4f) {
        return 0xFFFFFFFFu;
    }
    return (syncShaped >= 0.9999f) ? 0u
                                   : static_cast<uint32_t>((1.0f - syncShaped) * 4294967040.0f);
}

RUIDO_ALWAYS_INLINE float phaseToCapRamp(uint32_t phase) noexcept {
    return (phase < kHalfPhase)
               ? (static_cast<float>(phase) * kHalfPhaseToBipolar - 1.0f)
               : (1.0f - static_cast<float>(phase - kHalfPhase) * kHalfPhaseToBipolar);
}

template <bool IncludeChaosLoad>
RUIDO_ALWAYS_INLINE float computeVoiceRailPull(uint32_t phase, float currentLevel,
                                               float neighborSag, float totalGateLoad,
                                               const CouplingParams& cp) noexcept {
    if (cp.couplingGain <= 0.0f) {
        return 1.0f;
    }
    const float dHalf =
        absf(static_cast<float>(static_cast<int32_t>(phase ^ kHalfPhase)) * kSignedPhaseToBipolar);
    const float signedPrc = dHalf * dHalf * (3.0f - 2.0f * dHalf) * 2.0f - 1.0f;
    float pull = 1.0f - cp.couplingGain * neighborSag * signedPrc;
    if constexpr (IncludeChaosLoad) {
        if (cp.chaos > 0.0f) {
            const float neighborLoad = (totalGateLoad - currentLevel) * (1.0f / 3.0f);
            const float asymSign = (currentLevel > 0.0f) ? 0.36f : -0.36f;
            pull += cp.chaos * asymSign * neighborLoad;
        }
    }
    return clampf(pull, cp.minPull, cp.maxPull);
}

} // namespace ruido
