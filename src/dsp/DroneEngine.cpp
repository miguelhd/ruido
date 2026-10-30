#include "dsp/DroneEngine.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"

namespace ruido {

namespace {

constexpr float kPotPoleTauSec = 0.0140f;    // 2-pole C^1 pot glide (~28 ms mean, 0% overshoot)
constexpr float kControlSlewSec = 0.0050f;   // 5.0 ms mode/switch crossfade
constexpr float kVactrolSlewSec = 0.0038f;   // 3.8 ms clickless mute/gate vactrol feel
constexpr float kPressureAttackSec = 0.0045f;  // 4.5 ms snappy optocoupler LED turn-on
constexpr float kPressureReleaseSec = 0.0260f; // 26.0 ms liquid CdS photoconductive decay
constexpr float kMasterKillSec = 0.0080f;    // 8.0 ms ultra-clean master PLAY/STOP fade
constexpr float kVoiceSumScale = 0.35f;      // Headroom scaling for 4 hot square voices

} // namespace

DroneEngine::DroneEngine() noexcept {
    init(kBaseSampleRate);
}

void DroneEngine::init(float baseSampleRate) noexcept {
    baseSampleRate_ = (baseSampleRate > 8000.0f) ? baseSampleRate : kBaseSampleRate;
    oversampledRate_ = baseSampleRate_ * static_cast<float>(kOversampleFactor);
    invOversampledRate_ = 1.0f / oversampledRate_;

    oscBank_.reset(oversampledRate_);
    lfoBank_.reset(baseSampleRate_);
    for (auto& f : filters_) {
        f.reset();
    }
    decimator_.reset();
    coeffCache_ = {};
    filterCoeffs_ = {};
    prevOscGateHigh_ = {1.0f, 1.0f, 1.0f, 1.0f};
    voiceEdgeDelayLines_ = {0u, 0u, 0u, 0u};
    voiceScopeDelay_ = {};
    voiceScopeDelayWrite_ = 0u;

    // Smoothers step once per 44.118 kHz output audio frame (100% sample-accurate audio rate
    // smoothing while saving 4x redundant coefficient / pan updates inside the 4x inner loop).
    const EngineControlState initState = controlBus_.consumeLatest();
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceSmoothers& sm = voiceSmoothers_[i];
        sm.freqHz.init(initState.voices[i].frequencyHz, kPotPoleTauSec, baseSampleRate_);
        sm.amplitude.init(initState.voices[i].amplitude, kPotPoleTauSec, baseSampleRate_);
        sm.pan.init(initState.voices[i].pan, kPotPoleTauSec, baseSampleRate_);
        sm.hpCutoffHz.init(initState.voices[i].hpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.hpResonance.init(initState.voices[i].hpResonance, kPotPoleTauSec, baseSampleRate_);
        sm.lpCutoffHz.init(initState.voices[i].lpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.lpResonance.init(initState.voices[i].lpResonance, kPotPoleTauSec, baseSampleRate_);
        sm.vactrolGate.init(initState.voices[i].muted ? 0.0f : 1.0f, kVactrolSlewSec,
                            baseSampleRate_);
        sm.pressure.init(initState.voicePressure[i], kPressureAttackSec, kPressureReleaseSec,
                         baseSampleRate_);
    }

    couplingSmoother_.init(initState.global.circuitCoupling, kPotPoleTauSec, baseSampleRate_);
    masterTrimSmoother_.init(initState.global.masterTrim, kPotPoleTauSec, baseSampleRate_);
    monoMixSmoother_.init(initState.global.monoCollapse ? 1.0f : 0.0f, kVactrolSlewSec,
                          baseSampleRate_);
    masterRunSmoother_.init(initState.global.masterRun ? 1.0f : 0.0f, kMasterKillSec,
                            baseSampleRate_);
    pitchModeSmoother_.init(initState.global.pitchPressureMode ? 1.0f : 0.0f, kControlSlewSec,
                            baseSampleRate_);

    updateTargetsFromState(initState, true);
    lfoBank_.writeInitialTelemetry(telemetry_);
}

void DroneEngine::reset() noexcept {
    oscBank_.reset(oversampledRate_);
    lfoBank_.reset(baseSampleRate_);
    for (auto& f : filters_) {
        f.reset();
    }
    decimator_.reset();
    coeffCache_ = {};
    filterCoeffs_ = {};
    prevOscGateHigh_ = {1.0f, 1.0f, 1.0f, 1.0f};
    voiceEdgeDelayLines_ = {0u, 0u, 0u, 0u};
    voiceScopeDelay_ = {};
    voiceScopeDelayWrite_ = 0u;
    snapParametersToLatest();
    lfoBank_.writeInitialTelemetry(telemetry_);
}

void DroneEngine::snapParametersToLatest() noexcept {
    const EngineControlState& state = controlBus_.consumeLatest();
    updateTargetsFromState(state, true);
}

void DroneEngine::updateTargetsFromState(const EngineControlState& state,
                                         bool snapImmediate) noexcept {
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        const VoiceParams& vp = state.voices[i];
        VoiceSmoothers& sm = voiceSmoothers_[i];

        const float clampedBaseHz = clampf(vp.frequencyHz, kLfoFreqMinHz, kAudioFreqMaxHz);

        const float amp = clampf(vp.amplitude, 0.0f, 1.0f);
        const float pan = clampf(vp.pan, -1.0f, 1.0f);
        const float hpHz = clampf(vp.hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float hpRes = clampf(vp.hpResonance, 0.0f, 1.0f);
        const float lpHz = clampf(vp.lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float lpRes = clampf(vp.lpResonance, 0.0f, 1.0f);
        const float press = clampf(state.voicePressure[i], 0.0f, 1.0f);
        const float gateTarget = vp.muted ? 0.0f : 1.0f;

        if (snapImmediate) {
            sm.freqHz.snapTo(clampedBaseHz);
            sm.amplitude.snapTo(amp);
            sm.pan.snapTo(pan);
            sm.hpCutoffHz.snapTo(hpHz);
            sm.hpResonance.snapTo(hpRes);
            sm.lpCutoffHz.snapTo(lpHz);
            sm.lpResonance.snapTo(lpRes);
            sm.vactrolGate.snapTo(gateTarget);
            sm.pressure.snapTo(press);
        } else {
            sm.freqHz.setTarget(clampedBaseHz);
            if (sm.vactrolGate.current() <= 1.0e-4f && sm.pressure.current() <= 1.0e-4f &&
                gateTarget > 0.5f) {
                // Voice is currently completely silent: snap remembered amplitude to new target
                // before opening vactrol gate so fading in from 0% via GLO never blips upward.
                sm.amplitude.snapTo(amp);
            } else {
                sm.amplitude.setTarget(amp);
            }
            sm.pan.setTarget(pan);
            sm.hpCutoffHz.setTarget(hpHz);
            sm.hpResonance.setTarget(hpRes);
            sm.lpCutoffHz.setTarget(lpHz);
            sm.lpResonance.setTarget(lpRes);
            sm.vactrolGate.setTarget(gateTarget);
            sm.pressure.setTarget(press);
        }
    }

    const float coupling = clampf(state.global.circuitCoupling, 0.0f, 1.0f);
    const float trim = clampf(state.global.masterTrim, 0.0f, 1.5f);
    const float monoTarget = state.global.monoCollapse ? 1.0f : 0.0f;
    const float runTarget = state.global.masterRun ? 1.0f : 0.0f;
    const float pitchTarget = state.global.pitchPressureMode ? 1.0f : 0.0f;

    if (snapImmediate) {
        couplingSmoother_.snapTo(coupling);
        masterTrimSmoother_.snapTo(trim);
        monoMixSmoother_.snapTo(monoTarget);
        masterRunSmoother_.snapTo(runTarget);
        pitchModeSmoother_.snapTo(pitchTarget);
    } else {
        couplingSmoother_.setTarget(coupling);
        masterTrimSmoother_.setTarget(trim);
        monoMixSmoother_.setTarget(monoTarget);
        masterRunSmoother_.setTarget(runTarget);
        pitchModeSmoother_.setTarget(pitchTarget);
    }

    lfoBank_.updateTargets(state, snapImmediate);
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
    std::array<VoiceLfoFrame, kNumOscillators> lastLfo{};

    for (size_t frame = 0u; frame < numFrames; ++frame) {
        // Step audio-rate (44.118 kHz) smoothers, per-voice LFOs & prepare filter/pan coefficients
        const std::array<VoiceLfoFrame, kNumOscillators> lfoFrames = lfoBank_.stepFrame();
        lastLfo = lfoFrames;

        const float coupling = couplingSmoother_.next();
        const float masterTrim = masterTrimSmoother_.next();
        const float monoMix = monoMixSmoother_.next();
        const float panScale = 1.0f - monoMix;
        const bool exactMono = (panScale <= 0.0f);
        const float finalScale = masterTrim * masterRunSmoother_.next();
        const float pitchMode = pitchModeSmoother_.next();

        std::array<float, kNumOscillators> oscFreqs{};
        std::array<float, kNumOscillators> gainL{};
        std::array<float, kNumOscillators> gainR{};
        std::array<float, kNumOscillators> voiceScopeScale{};

        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceSmoothers& sm = voiceSmoothers_[i];
            VoiceCoeffCache& cache = coeffCache_[i];
            const VoiceLfoFrame& lfo = lfoFrames[i];
            const auto& td = lfo.targetDepths;

            const float p = sm.pressure.next();
            const float baseGate = sm.vactrolGate.next();
            const float baseHz = applyLfoPitchHz(
                sm.freqHz.next(), td[static_cast<size_t>(LfoTarget::Pitch)], lfo.raw);
            const float pitchMul = computeVoicePitchRatio(p, pitchMode);
            oscFreqs[i] = (pitchMul != 1.0f) ?
                              clampf(baseHz * pitchMul, 0.005f, kAudioFreqMaxHz * 4.0f) :
                              baseHz;

            const float hpBase = applyLfoCutoffHz(
                sm.hpCutoffHz.next(), td[static_cast<size_t>(LfoTarget::HpCutoff)], lfo.raw);
            const float hpResBase = applyLfoResonance(
                sm.hpResonance.next(), td[static_cast<size_t>(LfoTarget::HpResonance)], lfo.raw);
            const float lpBase = applyLfoCutoffHz(
                sm.lpCutoffHz.next(), td[static_cast<size_t>(LfoTarget::LpCutoff)], lfo.raw);
            const float lpResBase = applyLfoResonance(
                sm.lpResonance.next(), td[static_cast<size_t>(LfoTarget::LpResonance)], lfo.raw);

            const ModulatedFilterParams mod = computeVoicePressureMacro(
                hpBase, hpResBase, lpBase, lpResBase, p, baseGate, pitchMode);

            if (mod.hpCutoffHz != cache.hpHz || mod.hpResonance != cache.hpRes ||
                mod.lpCutoffHz != cache.lpHz || mod.lpResonance != cache.lpRes) {
                cache.hpHz = mod.hpCutoffHz;
                cache.hpRes = mod.hpResonance;
                cache.lpHz = mod.lpCutoffHz;
                cache.lpRes = mod.lpResonance;
                filterCoeffs_[i] = Ms20DualFilter::prepareCoeffs(
                    mod.hpCutoffHz, mod.hpResonance, mod.lpCutoffHz, mod.lpResonance,
                    invOversampledRate_);
            }

            const float modAmp = applyLfoAmplitude(
                sm.amplitude.next(), td[static_cast<size_t>(LfoTarget::Amplitude)], lfo.raw);
            const float ampGate = modAmp * mod.effectiveGate;
            const float voiceGain = ampGate * kVoiceSumScale;
            voiceScopeScale[i] = ampGate * finalScale * 0.25f;
            const float modPan =
                applyLfoPan(sm.pan.next(), td[static_cast<size_t>(LfoTarget::Pan)], lfo.raw);
            const float panEff = modPan * panScale;
            if (panEff != cache.panEff) {
                cache.panEff = panEff;
                equalPowerPan(panEff, cache.panL, cache.panR);
            }
            gainL[i] = voiceGain * cache.panL;
            gainR[i] = voiceGain * cache.panR;
        }

        // Run unified 4x oversampled (176.472 kHz) synthesis, rail coupling, ZDF filters & bus sag
        std::array<std::array<float, kOversampleFactor>, kNumOscillators> voiceSub{};
        std::array<StereoFrame, kOversampleFactor> osStereo{};
        std::array<float, kNumOscillators> voiceScopeAccum{0.0f, 0.0f, 0.0f, 0.0f};
        uint8_t voiceRisingInFrame = 0u;

        for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
            const OscillatorBankOutput oscOut = oscBank_.processSample(oscFreqs, coupling);
            if (oscOut.osc1RisingEdge) {
                voiceRisingInFrame |= 1u;
            }
            for (size_t v = 1u; v < kNumOscillators; ++v) {
                if (prevOscGateHigh_[v] < 0.5f && oscOut.gateHigh[v] > 0.5f) {
                    voiceRisingInFrame |= static_cast<uint8_t>(1u << v);
                }
            }
            prevOscGateHigh_ = oscOut.gateHigh;
            lastGateHigh = oscOut.gateHigh;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                voiceSub[i][sub] = oscOut.samples[i];
            }
        }

        Ms20DualFilter::processQuadBank4x(filters_, voiceSub, filterCoeffs_);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            voiceScopeAccum[i] =
                voiceSub[i][0] + voiceSub[i][1] + voiceSub[i][2] + voiceSub[i][3];
        }

        if (exactMono) {
            for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
                const float sumMono = voiceSub[0][sub] * gainL[0] + voiceSub[1][sub] * gainL[1] +
                                      voiceSub[2][sub] * gainL[2] + voiceSub[3][sub] * gainL[3];
                const float busMono = masterBusSoftClip(sumMono) * finalScale;
                osStereo[sub] = {busMono, busMono};
            }
        } else {
            for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
                const float sumL = voiceSub[0][sub] * gainL[0] + voiceSub[1][sub] * gainL[1] +
                                   voiceSub[2][sub] * gainL[2] + voiceSub[3][sub] * gainL[3];
                const float sumR = voiceSub[0][sub] * gainR[0] + voiceSub[1][sub] * gainR[1] +
                                   voiceSub[2][sub] * gainR[2] + voiceSub[3][sub] * gainR[3];
                osStereo[sub] = {masterBusSoftClip(sumL) * finalScale,
                                 masterBusSoftClip(sumR) * finalScale};
            }
        }

        // Linear-phase 4x -> 1x decimation to 44.118 kHz
        const StereoFrame out = decimator_.process4x(osStereo);
        const float cleanL = sanitizeState(out.left);
        const float cleanR = sanitizeState(out.right);

        outLeft[frame] = cleanL;
        outRight[frame] = cleanR;

        const float absL = absf(cleanL);
        const float absR = absf(cleanR);
        if (absL > blockPeakL) {
            blockPeakL = absL;
        }
        if (absR > blockPeakR) {
            blockPeakR = absR;
        }

        // Delay 4-voice rising-edge triggers & scope taps by 36 base frames to match FIR delay
        std::array<int16_t, kNumOscillators> curVoiceQ13{};
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            curVoiceQ13[i] = quantizeVoiceScopeQ13(voiceScopeAccum[i] * voiceScopeScale[i]);
        }
        const uint32_t delayWriteSlot = voiceScopeDelayWrite_ & 63u;
        const uint32_t delayReadSlot = (voiceScopeDelayWrite_ - 36u) & 63u;
        voiceScopeDelay_[delayWriteSlot] = curVoiceQ13;
        const std::array<int16_t, kNumOscillators> delayedVoices = voiceScopeDelay_[delayReadSlot];
        ++voiceScopeDelayWrite_;

        uint8_t delayedMask = 0u;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const uint64_t bit = ((voiceRisingInFrame >> v) & 1u) != 0u ? 1ULL : 0ULL;
            voiceEdgeDelayLines_[v] = (voiceEdgeDelayLines_[v] << 1u) | bit;
            if (((voiceEdgeDelayLines_[v] >> 36u) & 1ULL) != 0ULL) {
                delayedMask |= static_cast<uint8_t>(1u << v);
            }
        }
        const uint8_t delayedEdge = static_cast<uint8_t>(delayedMask & 1u);
        scopeBuffer_.push({cleanL, cleanR, delayedEdge, delayedMask, delayedVoices});
    }

    telemetry_.peakLeft.store(blockPeakL, std::memory_order_relaxed);
    telemetry_.peakRight.store(blockPeakR, std::memory_order_relaxed);
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        telemetry_.oscGateHigh[i].store(lastGateHigh[i], std::memory_order_relaxed);
        telemetry_.lfoOutput[i].store(lastLfo[i].raw, std::memory_order_relaxed);
        telemetry_.lfoPhase[i].store(lastLfo[i].phase01, std::memory_order_relaxed);
        telemetry_.lfoDriftPacked[i].store(lastLfo[i].driftPacked, std::memory_order_relaxed);
    }
}

} // namespace ruido
