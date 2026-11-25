#pragma once

#include "core/Config.h"
#include "dsp/SchmittOscillatorMath.h"

#include <array>
#include <cstdint>

namespace ruido {

struct OscillatorBankOutput {
    std::array<float, kNumOscillators> samples;
    std::array<float, kNumOscillators> capRamp;   // [-1.0f, +1.0f] CD40106 timing-capacitor ramp
    std::array<float, kNumOscillators> gateHigh;  // 1.0f high, 0.0f low (for LED pulse)
    std::array<float, kNumOscillators> driftNorm; // [-1.0f, +1.0f] live 1/f thermal drift
    uint8_t risingMask;                           // Bit v set on sample where Osc v wrapped 0
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

    void processSample(OscillatorBankOutput& __restrict out,
                       const std::array<float, kNumOscillators>& __restrict freqHz,
                       const std::array<float, kNumOscillators>& __restrict pulseWidth,
                       const std::array<float, kNumOscillators>& __restrict drift,
                       float circuitCoupling, bool freezeDrift = false,
                       float hardSync = 0.0f) noexcept;

    // Fast 4x-oversampled multi-frame span synthesis path for unpatched voices (hoists coupling,
    // hard-sync thresholds, PWM duty reciprocals, and denormal scrubbing across `numFrames`).
    void processSpan4x(std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
                       uint8_t* __restrict spanRising,
                       std::array<uint8_t, kNumOscillators>& totalRisingCounts, size_t numFrames,
                       const std::array<float, kNumOscillators>& __restrict startFreqHz,
                       const std::array<float, kNumOscillators>& __restrict endFreqHz,
                       const std::array<float, kNumOscillators>& __restrict startPw,
                       const std::array<float, kNumOscillators>& __restrict endPw,
                       const std::array<float, kNumOscillators>& __restrict drift,
                       float circuitCoupling, bool freezeDrift, float hardSync,
                       std::array<float, kNumOscillators>& outLastSamples,
                       std::array<float, kNumOscillators>& outLastCapRamp,
                       std::array<float, kNumOscillators>* optGateHigh = nullptr,
                       std::array<float, kNumOscillators>* optDriftNorm = nullptr) noexcept;

    // Precomputed span context for Circuit Lab patched sub-stepping (XMod / Fold / Crush / Ring):
    // hoists coupling thresholds, hard-sync thresholds, endpoint PWM duty reciprocals, and
    // base-rate thermal drift updates out of the 176.47 kHz 4x sub-sample inner loop.
    struct PatchedSpanState {
        CouplingParams cp{};
        uint32_t hardSyncThreshold = 0xFFFFFFFFu;
        float maxBaseInc = 0.0f;
        std::array<float, kNumOscillators> curBaseScale{};
        std::array<float, kNumOscillators> halfInvHigh{};
        std::array<float, kNumOscillators> halfInvLow{};
        std::array<float, kNumOscillators> stepInvHigh{};
        std::array<float, kNumOscillators> stepInvLow{};
        std::array<float, kNumOscillators> driftClamped{};
        uint8_t needCapRampMask = 0x0Fu;
        bool syncActive = false;
        bool anyPwSweep = false;
        bool freezeDrift = false;
    };

    void preparePatchedSpan(PatchedSpanState& st, float invSpan,
                            const std::array<float, kNumOscillators>& startFreqHz,
                            const std::array<float, kNumOscillators>& endFreqHz,
                            const std::array<float, kNumOscillators>& startPw,
                            const std::array<float, kNumOscillators>& endPw,
                            const std::array<float, kNumOscillators>& drift, float circuitCoupling,
                            bool freezeDrift, float hardSync, uint8_t needCapRampMask) noexcept;
    void beginPatchedFrame(PatchedSpanState& st) noexcept;
    uint8_t stepPatchedSubSample(const PatchedSpanState& st,
                                 const std::array<float, kNumOscillators>& subFreqHz,
                                 std::array<float, kNumOscillators>& outSamples,
                                 std::array<float, kNumOscillators>& outCapRamp) noexcept;
    void finishPatchedSpan(std::array<float, kNumOscillators>* optGateHigh,
                           std::array<float, kNumOscillators>* optDriftNorm) noexcept;

    OscillatorBankOutput processSample(const std::array<float, kNumOscillators>& freqHz,
                                       const std::array<float, kNumOscillators>& pulseWidth,
                                       const std::array<float, kNumOscillators>& drift,
                                       float circuitCoupling, bool freezeDrift = false,
                                       float hardSync = 0.0f) noexcept {
        OscillatorBankOutput out;
        processSample(out, freqHz, pulseWidth, drift, circuitCoupling, freezeDrift, hardSync);
        return out;
    }

    OscillatorBankOutput processSample(const std::array<float, kNumOscillators>& freqHz,
                                       float circuitCoupling) noexcept {
        constexpr std::array<float, kNumOscillators> kDefaultPw = {0.50f, 0.50f, 0.50f, 0.50f};
        constexpr std::array<float, kNumOscillators> kDefaultDr = {
            kDefaultVoiceDrift, kDefaultVoiceDrift, kDefaultVoiceDrift, kDefaultVoiceDrift};
        return processSample(freqHz, kDefaultPw, kDefaultDr, circuitCoupling);
    }

  private:
    struct VoiceState {
        uint32_t phase = 0u;
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
        float cachedDriftNorm = 0.0f;

        // Rail sag transient emitted on this voice's previous sample
        float lastEdgePulse = 0.0f;

        // Cached PWM duty reciprocals (eliminates 2 hardware fdivs/frame when duty is unchanged)
        float cachedPw = -1.0f;
        float cachedMinSafeDuty = -1.0f;
        float cachedHalfInvHigh = 1.0f;
        float cachedHalfInvLow = 1.0f;
        uint8_t blepPending = 0u;
    };

    static void addCubicBsplineBlep(VoiceState& v, float d, float delta) noexcept;
    static void applyMasterHardSync(VoiceState& v, uint32_t incHighU32, uint32_t incLowU32,
                                    float osc0WrapD) noexcept;
    static void stepVoiceThermalDrift(VoiceState& v, float dAmt, bool enableDrift) noexcept;
    static float nextWhiteNoise(uint32_t& state) noexcept;
    static uint32_t fastFallingEdge(VoiceState& v, uint32_t overHigh, uint32_t incHigh,
                                    uint32_t incLow) noexcept;
    static uint32_t fastRisingEdge(VoiceState& v, uint32_t overLow, uint32_t incLow,
                                   uint32_t incHigh) noexcept;
    void processFastDroneSpan4x(
        std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
        uint8_t* __restrict spanRising, std::array<uint8_t, kNumOscillators>& totalRisingCounts,
        size_t numFrames, float invSpan,
        const std::array<float, kNumOscillators>& __restrict startFreqHz,
        const std::array<float, kNumOscillators>& __restrict endFreqHz,
        const std::array<float, kNumOscillators>& __restrict startPw,
        const std::array<float, kNumOscillators>& __restrict endPw,
        const std::array<float, kNumOscillators>& __restrict drift, float couplingGain,
        float minPull, float maxPull, bool freezeDrift) noexcept;
    void processCoupledSyncSpan4x(
        std::array<std::array<float, kOversampleFactor>, 16>* __restrict spanSub,
        uint8_t* __restrict spanRising, std::array<uint8_t, kNumOscillators>& totalRisingCounts,
        size_t numFrames, float invSpan,
        const std::array<float, kNumOscillators>& __restrict startFreqHz,
        const std::array<float, kNumOscillators>& __restrict endFreqHz,
        const std::array<float, kNumOscillators>& __restrict startPw,
        const std::array<float, kNumOscillators>& __restrict endPw,
        const std::array<float, kNumOscillators>& __restrict drift, const CouplingParams& cp,
        bool freezeDrift, bool syncActive, uint32_t hardSyncThreshold) noexcept;
    void initSpanVoiceTables(size_t i, float sHz, float eHz, float sPw, float ePw, float driftAmt,
                             float maxPullScale, float invSpan, float maxSafePeakHz,
                             float& outCurBaseScale, float& outHalfInvHigh, float& outHalfInvLow,
                             float& outStepInvHigh, float& outStepInvLow,
                             bool& ioAnyPwSweep) noexcept;

    std::array<VoiceState, kNumOscillators> voices_{};
    float sampleRate_ = kOversampledRate;
    float hzToPhaseU32_ = 4294967296.0f / kOversampledRate;
    float rcHighpassPole_ = 0.9996f;

    // Shared Vdd supply rail sag states (fast crowbar spike + starved-rail sync state)
    std::array<float, kNumOscillators> voiceRailSag_{0.0f, 0.0f, 0.0f, 0.0f};
    float railDecayCoeff_ = 0.985f;
    float railDecayCoeff4x_ = 0.941f;
    uint8_t prevRisingMask_ = 0u;
    bool enableDrift_ = true;
    bool enableRcTilt_ = true;
};

} // namespace ruido
