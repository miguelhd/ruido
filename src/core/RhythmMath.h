#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Per-oscillator bottom-row rhythm gate modes on 3/3 RHYTHM LAB
enum class VoiceRhythmMode : uint8_t {
    Off = 0,
    Stutter = 1, // Keys 5-8:  Free pressure-speed chopper (1.5 -> 24 Hz, ignores master clock)
    Divide = 2,  // Keys 9-12: Master-clock-locked musical clock dividers (/2 .. x8)
    Euclid = 3,  // Keys 13-16: 16-step Euclidean groove patterns (2/16 .. 16/16)
};

inline constexpr size_t kNumClockDivisions = 6u;

// Cycles per 4-beat bar for each clock division (/2 = 2/bar, x1 = 4/bar .. x8 = 32/bar)
inline constexpr std::array<float, kNumClockDivisions> kClockDivCyclesPerBar = {
    2.0f, 4.0f, 8.0f, 12.0f, 16.0f, 32.0f,
};

inline constexpr std::array<const char*, kNumClockDivisions> kClockDivLabels = {
    "/2", "X1", "X2", "X3", "X4", "X8",
};

// Base clock division offset per oscillator (OSC 1..4 = /2, x1, x2, x3) so holding multiple
// DIVIDE keys at light pressure immediately locks into a multi-rate clock-divider ensemble,
// while pressing harder steps any oscillator up toward x4 / x8 rolls.
inline constexpr uint8_t selectVoiceClockDivIndex(uint8_t voiceIdx, float pressure01) noexcept {
    const uint8_t baseIdx = static_cast<uint8_t>(voiceIdx & 3u); // 0..3 (/2, x1, x2, x3)
    const float p = clampf(pressure01, 0.0f, 1.0f);
    const uint8_t boost = (p >= 0.80f) ? 2u : (p >= 0.48f) ? 1u : 0u;
    const uint8_t idx = static_cast<uint8_t>(baseIdx + boost);
    return (idx < kNumClockDivisions) ? idx : static_cast<uint8_t>(kNumClockDivisions - 1u);
}

inline constexpr const char* clockDivisionLabel(uint8_t divIdx) noexcept {
    return (divIdx < kNumClockDivisions) ? kClockDivLabels[divIdx] : kClockDivLabels[1];
}

inline constexpr size_t kNumEuclidTiers = 6u;

// Precomputed 16-step Bjorklund Euclidean bitmasks (bit s = 1 when step s in 0..15 fires)
inline constexpr std::array<uint16_t, kNumEuclidTiers> kEuclidMasks16 = {
    0x0101u, // 2/16:  X.......X.......
    0x1041u, // 3/16:  X.....X.....X... (Tresillo)
    0x2449u, // 5/16:  X..X..X...X..X.. (Clave / Bossa)
    0x54A9u, // 7/16:  X..X.X.X..X.X.X. (7/16 Bell)
    0xDB6Du, // 11/16: X.XX.XX.XX.XX.XX (Driving syncopation)
    0xFFFFu, // 16/16: XXXXXXXXXXXXXXXX (Full 16th roll)
};

inline constexpr std::array<uint8_t, kNumEuclidTiers> kEuclidHitCounts = {
    2u, 3u, 5u, 7u, 11u, 16u,
};

// Step rotation offsets across OSC 1..4 so simultaneous EUCLID keys interlock like a drum kit
inline constexpr std::array<uint8_t, kNumOscillators> kEuclidVoiceRotation = {0u, 2u, 1u, 3u};

inline constexpr uint8_t selectEuclidTierIndex(float pressure01) noexcept {
    const float p = clampf(pressure01, 0.0f, 1.0f);
    if (p < 0.18f) {
        return 0u;
    }
    if (p < 0.36f) {
        return 1u;
    }
    if (p < 0.54f) {
        return 2u;
    }
    if (p < 0.72f) {
        return 3u;
    }
    if (p < 0.88f) {
        return 4u;
    }
    return 5u;
}

inline constexpr uint16_t rotateEuclidMask16(uint16_t mask, uint8_t stepOffset) noexcept {
    const uint8_t sh = static_cast<uint8_t>(stepOffset & 15u);
    return (sh == 0u) ? mask : static_cast<uint16_t>((mask << sh) | (mask >> (16u - sh)));
}

inline constexpr uint16_t voiceEuclidMask16(uint8_t voiceIdx, uint8_t tierIdx) noexcept {
    const uint16_t base = (tierIdx < kNumEuclidTiers) ? kEuclidMasks16[tierIdx] : kEuclidMasks16[2];
    return rotateEuclidMask16(base, kEuclidVoiceRotation[voiceIdx & 3u]);
}

// Free-speed STUTTER rate: 1.5 Hz slow pulse -> 24.0 Hz fast buzz
RUIDO_ALWAYS_INLINE float computeStutterRateHz(float pressure01) noexcept {
    return 1.5f * fastExp2(4.0f * clampf(pressure01, 0.0f, 1.0f));
}

// PIT TEMPO multiplier: 1.0x -> 4.0x master clock speed
RUIDO_ALWAYS_INLINE float computeTempoMultiplier(float pressure01) noexcept {
    return fastExp2(2.0f * clampf(pressure01, 0.0f, 1.0f));
}

// POLY SWING percentage: 50% straight -> 72% deep shuffle
RUIDO_ALWAYS_INLINE constexpr float computeSwingRatio(float swing01) noexcept {
    return 0.50f + 0.22f * clampf(swing01, 0.0f, 1.0f);
}

// Continuous MPC-style 16th-note shuffle warp across the 8 eighth-note pairs in a 4-beat bar.
// Maps raw bar phase [0, 1) -> swung bar phase [0, 1) so odd 16th off-beats delay up to 72%.
RUIDO_ALWAYS_INLINE float warpSwingBarPhase01(float rawBarPhase01, float swing01) noexcept {
    const float raw = wrapLfoPhase01(rawBarPhase01);
    const float s = clampf(swing01, 0.0f, 1.0f);
    if (s <= 1.0e-4f) {
        return raw;
    }
    const float pairScaled = raw * 8.0f;
    const auto pairIdx = static_cast<uint32_t>(pairScaled);
    const float pairFrac = pairScaled - static_cast<float>(pairIdx);
    const float split = computeSwingRatio(s); // [0.50, 0.72]
    float warpedPair = 0.0f;
    if (pairFrac < split) {
        warpedPair = 0.5f * (pairFrac / split);
    } else {
        warpedPair = 0.5f + 0.5f * ((pairFrac - split) / (1.0f - split));
    }
    return clampf((static_cast<float>(pairIdx & 7u) + warpedPair) * 0.125f, 0.0f, 0.999999f);
}

// Evaluates per-oscillator percussive pluck envelope [0, 1] for Core 0 MS-20 filter graph & HUD
inline float evalVoicePluckPreviewEnv01(float rhythmPluck, uint8_t rhythmModeRaw,
                                        uint8_t rhythmParamRaw, uint8_t voiceIdx, float barPhase01,
                                        bool gateBitOpen) noexcept {
    const float p = clampf(rhythmPluck, 0.0f, 1.0f);
    if (p <= 1.0e-4f || !gateBitOpen) {
        return 0.0f;
    }
    const float bar01 = wrapLfoPhase01(barPhase01);
    float subFrac = 0.0f;
    const auto mode = static_cast<VoiceRhythmMode>(rhythmModeRaw);
    if (mode == VoiceRhythmMode::Divide) {
        const float cyc = kClockDivCyclesPerBar[rhythmParamRaw % kNumClockDivisions];
        const float sc = bar01 * cyc;
        subFrac = sc - static_cast<float>(static_cast<uint32_t>(sc));
    } else {
        (void)voiceIdx;
        const float sc = bar01 * 16.0f;
        subFrac = sc - static_cast<float>(static_cast<uint32_t>(sc));
    }
    return fastExpNeg(subFrac * (3.0f + 6.5f * p));
}

} // namespace ruido
