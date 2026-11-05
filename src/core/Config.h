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

// Voice, dual-LFO & snapshot topology
inline constexpr size_t kNumOscillators = 4u;
inline constexpr size_t kLfosPerVoice = 2u;
inline constexpr size_t kNumVoiceLfos = kNumOscillators * kLfosPerVoice; // 8 LFOs (0..3=A, 4..7=B)
inline constexpr size_t kNumSnapshots = 4u;

// Oscillator & per-voice LFO frequency bounds (unbroken sub-cycle -> ultrasonic continuum)
inline constexpr float kAudioFreqMinHz = 0.05f;
inline constexpr float kAudioFreqMaxHz = 20000.0f;
inline constexpr float kPulseWidthMin = 0.05f;
inline constexpr float kPulseWidthMax = 0.95f;
inline constexpr float kDefaultVoiceDrift = 0.10f;
inline constexpr float kDefaultVoiceAmp = 0.65f;
inline constexpr float kLfoFreqMinHz = 0.01f;
inline constexpr float kLfoFreqMaxHz = 2000.0f;

// MS-20 dual filter bounds (infrasonic 2 Hz rumble -> 28 kHz ultrasonic openness @ 176.47 kHz)
inline constexpr float kFilterCutoffMinHz = 2.0f;
inline constexpr float kFilterCutoffMaxHz = 28000.0f;
inline constexpr float kFilterMaxSafeHz = 30000.0f; // Safe ceiling @ 176.47 kHz

// Master rhythm internal tempo bounds vs external 24-PPQN MIDI clock PLL sanity window (BPM)
inline constexpr float kDefaultClockBpm = 120.0f;
inline constexpr float kMinInternalClockBpm = 20.0f;
inline constexpr float kMaxInternalClockBpm = 3000.0f;
inline constexpr float kMaxEffectiveClockBpm = 6000.0f;
inline constexpr float kMinMidiClockBpm = 30.0f;
inline constexpr float kMaxMidiClockBpm = 300.0f;

// Lock-free SPSC oscilloscope ring buffer capacity (power of two)
inline constexpr size_t kScopeRingCapacity = 2048u;
inline constexpr size_t kScopeRingMask = kScopeRingCapacity - 1u;

// Math constants (strictly float32)
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;
inline constexpr float kHalfPi = 1.57079632679489661923f;
inline constexpr float kSqrt2 = 1.41421356237309504880f;

} // namespace ruido
