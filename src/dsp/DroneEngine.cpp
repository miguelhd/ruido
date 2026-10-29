#include "dsp/DroneEngine.h"

#include "core/DspMath.h"

#include <cmath>

namespace ruido {

namespace {

constexpr float kControlSlewSec = 0.0050f; // 5.0 ms smooth parameter glide
constexpr float kVactrolSlewSec = 0.0038f; // 3.8 ms clickless mute/gate vactrol feel
constexpr float kMasterKillSec = 0.0080f;  // 8.0 ms ultra-clean master PLAY/STOP fade
constexpr float kVoiceSumScale = 0.35f;    // Headroom scaling for 4 hot square voices

} // namespace

DroneEngine::DroneEngine() noexcept {
    init(kBaseSampleRate);
}

void DroneEngine::init(float baseSampleRate) noexcept {
    baseSampleRate_ = (baseSampleRate > 8000.0f) ? baseSampleRate : kBaseSampleRate;
    oversampledRate_ = baseSampleRate_ * static_cast<float>(kOversampleFactor);
    invOversampledRate_ = 1.0f / oversampledRate_;

    oscBank_.reset(oversampledRate_);
    for (auto& f : filters_) {
        f.reset();
    }
    decimator_.reset();
    coeffCache_ = {};
    osc1EdgeDelayLine_ = 0u;

    // Smoothers step once per 44.118 kHz output audio frame (100% sample-accurate audio rate
    // smoothing while saving 4x redundant coefficient / pan updates inside the 4x inner loop).
    const EngineControlState initState = controlBus_.consumeLatest();
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceSmoothers& sm = voiceSmoothers_[i];
        sm.freqHz.init(initState.voices[i].frequencyHz, kControlSlewSec, baseSampleRate_);
        sm.amplitude.init(initState.voices[i].amplitude, kControlSlewSec, baseSampleRate_);
        sm.pan.init(initState.voices[i].pan, kControlSlewSec, baseSampleRate_);
        sm.hpCutoffHz.init(initState.voices[i].hpCutoffHz, kControlSlewSec, baseSampleRate_);
        sm.lpCutoffHz.init(initState.voices[i].lpCutoffHz, kControlSlewSec, baseSampleRate_);
        sm.resonance.init(initState.voices[i].resonance, kControlSlewSec, baseSampleRate_);
        sm.drive.init(initState.voices[i].drive, kControlSlewSec, baseSampleRate_);
        sm.vactrolGate.init(initState.voices[i].muted ? 0.0f : 1.0f, kVactrolSlewSec,
                            baseSampleRate_);
        sm.pressure.init(initState.voicePressure[i], kVactrolSlewSec, baseSampleRate_);
    }

    couplingSmoother_.init(initState.global.circuitCoupling, kControlSlewSec, baseSampleRate_);
    spreadSmoother_.init(initState.global.stereoSpread, kControlSlewSec, baseSampleRate_);
    masterTrimSmoother_.init(initState.global.masterTrim, kControlSlewSec, baseSampleRate_);
    monoMixSmoother_.init(initState.global.monoCollapse ? 1.0f : 0.0f, kVactrolSlewSec,
                          baseSampleRate_);
    masterRunSmoother_.init(initState.global.masterRun ? 1.0f : 0.0f, kMasterKillSec,
                            baseSampleRate_);
    pitchModeSmoother_.init(initState.global.pitchPressureMode ? 1.0f : 0.0f, kControlSlewSec,
                            baseSampleRate_);

    updateTargetsFromState(initState, true);
}

void DroneEngine::reset() noexcept {
    oscBank_.reset(oversampledRate_);
    for (auto& f : filters_) {
        f.reset();
    }
    decimator_.reset();
    coeffCache_ = {};
    osc1EdgeDelayLine_ = 0u;
    snapParametersToLatest();
}

void DroneEngine::snapParametersToLatest() noexcept {
    const EngineControlState& state = controlBus_.consumeLatest();
    updateTargetsFromState(state, true);
}

void DroneEngine::updateTargetsFromState(const EngineControlState& state,
                                         bool snapImmediate) noexcept {
    const int8_t octShift = static_cast<int8_t>(
        clampf(static_cast<float>(state.global.globalOctaveShift), -3.0f, 3.0f));
    const float globalOctaveMul = fastExp2(static_cast<float>(octShift));

    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = state.voices[i];
        VoiceSmoothers& sm = voiceSmoothers_[i];

        const float minF = vp.subAudioLfo ? kLfoFreqMinHz : kAudioFreqMinHz;
        const float maxF = vp.subAudioLfo ? kLfoFreqMaxHz : kAudioFreqMaxHz;
        const float clampedBaseHz = clampf(vp.frequencyHz, minF, maxF);
        // Allow OCT- / OCT+ to span [0.005 Hz, 16 kHz] without compressing harmonic intervals
        const float shiftedHz = clampf(clampedBaseHz * globalOctaveMul, 0.005f,
                                       kAudioFreqMaxHz * 4.0f);

        const float amp = clampf(vp.amplitude, 0.0f, 1.0f);
        const float pan = clampf(vp.pan, -1.0f, 1.0f);
        const float hpHz = clampf(vp.hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float lpHz = clampf(vp.lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float reso = clampf(vp.resonance, 0.0f, 1.0f);
        const float drv = clampf(vp.drive, 0.0f, 1.0f);
        const float press = clampf(state.voicePressure[i], 0.0f, 1.0f);
        const float gateTarget = vp.muted ? 0.0f : 1.0f;

        if (snapImmediate) {
            sm.freqHz.snapTo(shiftedHz);
            sm.amplitude.snapTo(amp);
            sm.pan.snapTo(pan);
            sm.hpCutoffHz.snapTo(hpHz);
            sm.lpCutoffHz.snapTo(lpHz);
            sm.resonance.snapTo(reso);
            sm.drive.snapTo(drv);
            sm.vactrolGate.snapTo(gateTarget);
            sm.pressure.snapTo(press);
        } else {
            sm.freqHz.setTarget(shiftedHz);
            sm.amplitude.setTarget(amp);
            sm.pan.setTarget(pan);
            sm.hpCutoffHz.setTarget(hpHz);
            sm.lpCutoffHz.setTarget(lpHz);
            sm.resonance.setTarget(reso);
            sm.drive.setTarget(drv);
            sm.vactrolGate.setTarget(gateTarget);
            sm.pressure.setTarget(press);
        }
    }

    const float coupling = clampf(state.global.circuitCoupling, 0.0f, 1.0f);
    const float spread = clampf(state.global.stereoSpread, 0.0f, 1.0f);
    const float trim = clampf(state.global.masterTrim, 0.0f, 1.5f);
    const float monoTarget = state.global.monoCollapse ? 1.0f : 0.0f;
    const float runTarget = state.global.masterRun ? 1.0f : 0.0f;
    const float pitchModeTarget = state.global.pitchPressureMode ? 1.0f : 0.0f;

    if (snapImmediate) {
        couplingSmoother_.snapTo(coupling);
        spreadSmoother_.snapTo(spread);
        masterTrimSmoother_.snapTo(trim);
        monoMixSmoother_.snapTo(monoTarget);
        masterRunSmoother_.snapTo(runTarget);
        pitchModeSmoother_.snapTo(pitchModeTarget);
    } else {
        couplingSmoother_.setTarget(coupling);
        spreadSmoother_.setTarget(spread);
        masterTrimSmoother_.setTarget(trim);
        monoMixSmoother_.setTarget(monoTarget);
        masterRunSmoother_.setTarget(runTarget);
        pitchModeSmoother_.setTarget(pitchModeTarget);
    }
}

void DroneEngine::processBlock(float* outLeft, float* outRight, size_t numFrames) noexcept {
    if (numFrames == 0u || outLeft == nullptr || outRight == nullptr) {
        return;
    }

    const EngineControlState& latestState = controlBus_.consumeLatest();
    updateTargetsFromState(latestState, false);

    float blockPeakL = 0.0f;
    float blockPeakR = 0.0f;
    std::array<float, kNumOscillators> lastGateHigh{0.0f, 0.0f, 0.0f, 0.0f};

    for (size_t frame = 0u; frame < numFrames; ++frame) {
        // Step audio-rate (44.118 kHz) smoothers & prepare voice filter/pan coefficients
        const float coupling = couplingSmoother_.next();
        const float spread = spreadSmoother_.next();
        const float masterTrim = masterTrimSmoother_.next();
        const float monoMix = monoMixSmoother_.next();
        const float effectiveSpread = spread * (1.0f - monoMix);
        const bool exactMono = (effectiveSpread <= 0.0f);
        const float pitchMode = pitchModeSmoother_.next();
        const float finalScale = masterTrim * masterRunSmoother_.next();

        std::array<float, kNumOscillators> oscFreqs{};
        std::array<float, kNumOscillators> gainL{};
        std::array<float, kNumOscillators> gainR{};

        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceSmoothers& sm = voiceSmoothers_[i];
            VoiceCoeffCache& cache = coeffCache_[i];
            const float p = sm.pressure.next();
            const float baseGate = sm.vactrolGate.next();
            // Scale pitch/filter pressure expression by smoothed baseGate so muted swells stay
            // rock-solid in tune and toggling Mute or PIT while pressing never clicks!
            const float droneExpr = baseGate * p;
            const float pitchExpr = droneExpr * pitchMode;
            const float filterExpr = droneExpr * (1.0f - pitchMode);

            float fHz = sm.freqHz.next();
            if (pitchExpr > 0.0f) {
                fHz *= fastExp2(0.50f * pitchExpr * pitchExpr);
            }
            oscFreqs[i] = fHz;

            const float effectiveGate = clampf(baseGate + (1.0f - baseGate) * p, 0.0f, 1.0f);
            const float hpHz = sm.hpCutoffHz.next();
            float lpHz = sm.lpCutoffHz.next();
            float drv = sm.drive.next();
            if (filterExpr > 0.0f) {
                lpHz = clampf(lpHz * fastExp2(2.5f * filterExpr),
                              kFilterCutoffMinHz, kFilterCutoffMaxHz);
                drv = clampf(drv + 0.55f * filterExpr, 0.0f, 1.0f);
            }

            const float reso = sm.resonance.next();
            if (hpHz != cache.hpHz || lpHz != cache.lpHz || reso != cache.reso ||
                drv != cache.drv) {
                cache.hpHz = hpHz;
                cache.lpHz = lpHz;
                cache.reso = reso;
                cache.drv = drv;
                cache.filter = Ms20DualFilter::prepareCoeffs(hpHz, lpHz, reso, drv,
                                                             invOversampledRate_);
            }

            const float voiceGain = sm.amplitude.next() * effectiveGate * kVoiceSumScale;
            const float panEff = sm.pan.next() * effectiveSpread;
            if (panEff != cache.panEff) {
                cache.panEff = panEff;
                equalPowerPan(panEff, cache.panL, cache.panR);
            }
            gainL[i] = voiceGain * cache.panL;
            gainR[i] = voiceGain * cache.panR;
        }

        // Run unified 4x oversampled (176.472 kHz) synthesis, rail coupling, ZDF filters & bus sag
        std::array<StereoFrame, 4> osStereo{};
        bool osc1RisingInFrame = false;

        for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
            const OscillatorBankOutput oscOut = oscBank_.processSample(oscFreqs, coupling);
            if (oscOut.osc1RisingEdge) {
                osc1RisingInFrame = true;
            }
            lastGateHigh = oscOut.gateHigh;

            if (exactMono) {
                float sumMono = 0.0f;
                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    const float filtered = filters_[i].processPrepared(oscOut.samples[i],
                                                                       coeffCache_[i].filter);
                    sumMono += filtered * gainL[i];
                }
                const float busMono = masterBusSoftClip(sumMono) * finalScale;
                osStereo[sub] = {busMono, busMono};
            } else {
                float sumL = 0.0f;
                float sumR = 0.0f;
                for (size_t i = 0u; i < kNumOscillators; ++i) {
                    const float filtered = filters_[i].processPrepared(oscOut.samples[i],
                                                                       coeffCache_[i].filter);
                    sumL += filtered * gainL[i];
                    sumR += filtered * gainR[i];
                }
                const float busL = masterBusSoftClip(sumL) * finalScale;
                const float busR = masterBusSoftClip(sumR) * finalScale;
                osStereo[sub] = {busL, busR};
            }
        }

        // Linear-phase 4x -> 1x decimation to 44.118 kHz
        const StereoFrame out = decimator_.process4x(osStereo);
        const float cleanL = sanitizeState(out.left);
        const float cleanR = sanitizeState(out.right);

        outLeft[frame] = cleanL;
        outRight[frame] = cleanR;

        const float absL = ::fabsf(cleanL);
        const float absR = ::fabsf(cleanR);
        if (absL > blockPeakL) {
            blockPeakL = absL;
        }
        if (absR > blockPeakR) {
            blockPeakR = absR;
        }

        // Delay Osc-1 rising-edge trigger by 36 base frames to match PolyBLEP + FIR group delay
        osc1EdgeDelayLine_ = (osc1EdgeDelayLine_ << 1u) | (osc1RisingInFrame ? 1ULL : 0ULL);
        const uint8_t delayedEdge = static_cast<uint8_t>((osc1EdgeDelayLine_ >> 36u) & 1ULL);
        scopeBuffer_.push({cleanL, cleanR, delayedEdge});
    }

    telemetry_.peakLeft.store(blockPeakL, std::memory_order_relaxed);
    telemetry_.peakRight.store(blockPeakR, std::memory_order_relaxed);
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        telemetry_.oscGateHigh[i].store(lastGateHigh[i], std::memory_order_relaxed);
    }
}

} // namespace ruido
