#pragma once

#include "core/Config.h"

#include <array>
#include <cstdint>

namespace ruido {

struct OscillatorBankOutput {
    std::array<float, kNumOscillators> samples{};
    std::array<float, kNumOscillators> gateHigh{}; // 1.0f high, 0.0f low (for LED pulse)
    bool osc1RisingEdge = false;                   // True on sample where Osc 1 wrapped 0
};

// 4-voice CMOS Schmitt-Trigger relaxation oscillator bank (CD40106 model) running at
// 4x oversampling (176.472 kHz):
//  - 32-bit unsigned integer phase accumulators (41 uHz resolution, immune to float32 stall)
//  - 4-point Integrated Cubic B-Spline PolyBLEP (~-110 dB alias floor < 20 kHz)
//  - Per-inverter threshold duty calibration (~49.2% - 50.9%) for subtle analog even harmonics
//  - Independent slow 1/f-like thermal pitch drift per voice
//  - Shared Vdd power-rail threshold coupling (`ALGORITHM` knob) for weak injection locking
//  - Subtle RC coupling capacitor roof tilt + DC blocking
class SchmittOscillatorBank {
public:
    SchmittOscillatorBank() noexcept;

    void reset(float sampleRate = kOversampledRate) noexcept;

    // Enable/disable analog thermal drift & RC capacitor tilt (useful for isolated DFT tests).
    void setAnalogCharacterEnabled(bool enableDrift, bool enableRcTilt) noexcept {
        enableDrift_ = enableDrift;
        enableRcTilt_ = enableRcTilt;
    }

    OscillatorBankOutput processSample(const std::array<float, kNumOscillators>& freqHz,
                                       float circuitCoupling) noexcept;

private:
    struct VoiceState {
        uint32_t phase = 0u;
        uint32_t dutyThreshold = 0x80000000u;
        float currentLevel = 1.0f;
        float naiveD1 = 1.0f;
        float naiveD2 = 1.0f;
        std::array<float, 4> blepResidual{0.0f, 0.0f, 0.0f, 0.0f};

        // RC coupling capacitor roof sag / high-pass tilt state
        float rcCapState = 0.0f;
        float rcPrevInput = 0.0f;

        // Per-voice slow thermal pitch drift state
        uint32_t rngState = 1u;
        float driftFast = 0.0f;
        float driftSlow = 0.0f;
        uint32_t driftCounter = 0u;
        float cachedDriftRatio = 1.0f;

        // Rail sag transient emitted on this voice's previous sample
        float lastEdgePulse = 0.0f;
    };

    static void addCubicBsplineBlep(std::array<float, 4>& res, float d, float delta) noexcept;
    static float nextWhiteNoise(uint32_t& state) noexcept;

    std::array<VoiceState, kNumOscillators> voices_{};
    float sampleRate_ = kOversampledRate;
    float hzToPhaseU32_ = 4294967296.0f / kOversampledRate;
    float rcHighpassPole_ = 0.9996f;

    // Shared Vdd supply rail sag states (fast crowbar spike + decoupling cap recovery)
    std::array<float, kNumOscillators> voiceRailSag_{0.0f, 0.0f, 0.0f, 0.0f};
    float railDecayCoeff_ = 0.985f;
    bool enableDrift_ = true;
    bool enableRcTilt_ = true;
};

} // namespace ruido
