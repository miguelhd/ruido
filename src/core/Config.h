#pragma once

#include <cstddef>
#include <cstdint>

namespace ruido {

// Hardware & DSP timing constants (M-VAVE FM-1 / JieLi AC791N target)
inline constexpr float kBaseSampleRate = 44118.0f;
inline constexpr uint32_t kOversampleFactor = 4u;
inline constexpr float kOversampledRate = kBaseSampleRate * 4.0f; // 176472.0f
inline constexpr float kInvOversampledRate = 1.0f / kOversampledRate;
inline constexpr size_t kBlockFrames = 64u;
inline constexpr size_t kOversampledBlockFrames = kBlockFrames * kOversampleFactor;

// Voice & snapshot topology
inline constexpr size_t kNumOscillators = 4u;
inline constexpr size_t kNumSnapshots = 8u;

// Oscillator & per-voice LFO frequency bounds
inline constexpr float kAudioFreqMinHz = 20.0f;
inline constexpr float kAudioFreqMaxHz = 4000.0f;
inline constexpr float kLfoFreqMinHz = 0.02f;
inline constexpr float kLfoFreqMaxHz = 30.0f;

// MS-20 dual filter bounds
inline constexpr float kFilterCutoffMinHz = 20.0f;
inline constexpr float kFilterCutoffMaxHz = 20000.0f;
inline constexpr float kFilterMaxSafeHz = 30000.0f; // Safe ceiling @ 176.47 kHz

// Lock-free SPSC oscilloscope ring buffer capacity (power of two)
inline constexpr size_t kScopeRingCapacity = 2048u;
inline constexpr size_t kScopeRingMask = kScopeRingCapacity - 1u;

// Math constants (strictly float32)
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;
inline constexpr float kHalfPi = 1.57079632679489661923f;
inline constexpr float kSqrt2 = 1.41421356237309504880f;

} // namespace ruido
