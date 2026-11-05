#include "dsp/DroneEngine.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "dsp/CircuitPatchMatrix.h"

namespace ruido {

namespace {

constexpr float kPotPoleTauSec = 0.0140f;      // 2-pole C^1 pot glide (~28 ms mean, 0% overshoot)
constexpr float kControlSlewSec = 0.0050f;     // 5.0 ms mode/switch crossfade
constexpr float kVactrolSlewSec = 0.0038f;     // 3.8 ms clickless mute/gate vactrol feel
constexpr float kPressureAttackSec = 0.0045f;  // 4.5 ms snappy optocoupler LED turn-on
constexpr float kPressureReleaseSec = 0.0260f; // 26.0 ms liquid CdS photoconductive decay
constexpr float kMasterKillSec = 0.0080f;      // 8.0 ms ultra-clean master PLAY/STOP fade
constexpr float kStutterEdgeSec = 0.0022f;     // 2.2 ms snappy click-free stutter gate edge
constexpr float kPunchBusSlewSec = 0.0085f;    // 8.5 ms punch-in bus surge / dub LPF / spatial slew
constexpr float kVoiceSumScale = 0.36f;        // Headroom scaling into masterBusSoftClip
constexpr std::array<float, kNumOscillators> kWideFanPans = {-1.0f, -0.52f, 0.52f, 1.0f};

} // namespace

DroneEngine::DroneEngine() noexcept {
    init(kBaseSampleRate);
}

void DroneEngine::init(float baseSampleRate) noexcept {
    baseSampleRate_ = (baseSampleRate >= 16000.0f && baseSampleRate <= 384000.0f) ? baseSampleRate
                                                                                  : kBaseSampleRate;
    oversampledRate_ = baseSampleRate_ * static_cast<float>(kOversampleFactor);
    invOversampledRate_ = 1.0f / oversampledRate_;
    hzToBasePhaseU32_ = 4294967296.0f / baseSampleRate_;
    hzToOversampledPhaseU32_ = 4294967296.0f / oversampledRate_;

    oscBank_.reset(oversampledRate_);
    lfoBank_.reset(baseSampleRate_);
    for (auto& f : filters_) {
        f.reset();
    }
    decimator_.reset();
    coeffCache_ = {};
    filterCoeffs_ = {};
    crushStates_ = {};
    prevOscSamples_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prevCapRamp_ = {0.0f, 0.0f, 0.0f, 0.0f};
    voiceEdgeDelayLines_ = {0u, 0u, 0u, 0u};
    voiceScopeDelay_ = {};
    voiceScopeDelayWrite_ = 0u;

    // Smoothers step once per 44.118 kHz output audio frame (100% sample-accurate audio rate
    // smoothing while saving 4x redundant coefficient / pan updates inside the 4x inner loop).
    const EngineControlState initState = controlBus_.consumeLatest();
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceSmoothers& sm = voiceSmoothers_[i];
        sm.freqHz.init(initState.voices[i].frequencyHz, kPotPoleTauSec, baseSampleRate_);
        sm.pulseWidth.init(initState.voices[i].pulseWidth, kPotPoleTauSec, baseSampleRate_);
        sm.drift.init(initState.voices[i].drift, kPotPoleTauSec, baseSampleRate_);
        sm.hpCutoffHz.init(initState.voices[i].hpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.hpResonance.init(initState.voices[i].hpResonance, kPotPoleTauSec, baseSampleRate_);
        sm.lpCutoffHz.init(initState.voices[i].lpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.lpResonance.init(initState.voices[i].lpResonance, kPotPoleTauSec, baseSampleRate_);
        sm.amplitude.init(initState.voices[i].amplitude, kPotPoleTauSec, baseSampleRate_);
        sm.pan.init(initState.voices[i].pan, kPotPoleTauSec, baseSampleRate_);
        sm.vactrolGate.init(initState.voices[i].muted ? 0.0f : 1.0f, kVactrolSlewSec,
                            baseSampleRate_);
        sm.pressure.init(initState.voicePressure[i], kPressureAttackSec, kPressureReleaseSec,
                         baseSampleRate_);
        sm.keyGainMul.init(initState.keyMod.voiceGainMul[i], kVactrolSlewSec, baseSampleRate_);
        sm.keyPitchMul.init(initState.keyMod.voicePitchMul[i], kPotPoleTauSec, baseSampleRate_);
        sm.stutterDepth.init(initState.keyMod.stutterDepth[i], kVactrolSlewSec, baseSampleRate_);
        sm.stutterGate.init(1.0f, kStutterEdgeSec, baseSampleRate_);
        sm.xmodDepth.init(initState.keyMod.xmodDepth[i], kPunchBusSlewSec, baseSampleRate_);
        sm.foldDepth.init(initState.keyMod.foldDepth[i], kPunchBusSlewSec, baseSampleRate_);
        sm.crushDepth.init(initState.keyMod.crushDepth[i], kPunchBusSlewSec, baseSampleRate_);
        sm.stutterPhase = 0x80000000u;
        sm.stutterRateHz = initState.keyMod.stutterRateHz[i];
    }

    couplingSmoother_.init(initState.global.circuitCoupling, kPotPoleTauSec, baseSampleRate_);
    masterTrimSmoother_.init(initState.global.masterTrim, kPotPoleTauSec, baseSampleRate_);
    monoMixSmoother_.init(initState.global.monoCollapse ? 1.0f : 0.0f, kVactrolSlewSec,
                          baseSampleRate_);
    masterRunSmoother_.init(initState.global.masterRun ? 1.0f : 0.0f, kMasterKillSec,
                            baseSampleRate_);
    pitchModeSmoother_.init(initState.global.pitchPressureMode ? 1.0f : 0.0f, kControlSlewSec,
                            baseSampleRate_);
    overloadSurgeSmoother_.init(initState.keyMod.overloadSurge, kPunchBusSlewSec, baseSampleRate_);
    dubLpfDropSmoother_.init(initState.keyMod.dubLpfDrop, kPunchBusSlewSec, baseSampleRate_);
    spatialFocusSmoother_.init(initState.keyMod.spatialFocus, kPunchBusSlewSec, baseSampleRate_);
    ringModSmoother_.init(initState.keyMod.ringModMix, kPunchBusSlewSec, baseSampleRate_);
    hardSyncSmoother_.init(initState.keyMod.hardSyncAmount, kPunchBusSlewSec, baseSampleRate_);
    filterScreamSmoother_.init(initState.keyMod.filterScream, kPunchBusSlewSec, baseSampleRate_);
    stasisFreeze_ = initState.keyMod.stasisFreeze;
    telemetry_.stutterGateMask.store(0x0Fu, std::memory_order_relaxed);

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
    crushStates_ = {};
    prevOscSamples_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prevCapRamp_ = {0.0f, 0.0f, 0.0f, 0.0f};
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

        const float clampedBaseHz = clampf(vp.frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
        const float pw =
            clampf(vp.pulseWidth + state.keyMod.voicePwDelta[i], kPulseWidthMin, kPulseWidthMax);
        const float dr = clampf(
            vp.drift * (1.0f - clampf(state.keyMod.ratioLockAmount, 0.0f, 1.0f)), 0.0f, 1.0f);
        const float hpHz = clampf(vp.hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float hpRes = clampf(vp.hpResonance, 0.0f, 1.0f);
        const float lpHz = clampf(vp.lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float lpRes = clampf(vp.lpResonance, 0.0f, 1.0f);
        const float amp = clampf(vp.amplitude, 0.0f, 1.0f);
        const float pan = clampf(vp.pan, -1.0f, 1.0f);
        const float press = clampf(state.voicePressure[i], 0.0f, 1.0f);
        const float gateTarget = vp.muted ? 0.0f : 1.0f;
        const float kGain = clampf(state.keyMod.voiceGainMul[i], 0.0f, 1.5f);
        const float kPitch = clampf(state.keyMod.voicePitchMul[i], 0.125f, 8.0f);
        const float kStutDepth = clampf(state.keyMod.stutterDepth[i], 0.0f, 1.0f);
        const float kXMod = clampf(state.keyMod.xmodDepth[i], 0.0f, 1.0f);
        const float kFold = clampf(state.keyMod.foldDepth[i], 0.0f, 1.0f);
        const float kCrush = clampf(state.keyMod.crushDepth[i], 0.0f, 1.0f);
        sm.stutterRateHz = clampf(state.keyMod.stutterRateHz[i], 1.0f, 30.0f);

        if (snapImmediate) {
            sm.freqHz.snapTo(clampedBaseHz);
            sm.pulseWidth.snapTo(pw);
            sm.drift.snapTo(dr);
            sm.hpCutoffHz.snapTo(hpHz);
            sm.hpResonance.snapTo(hpRes);
            sm.lpCutoffHz.snapTo(lpHz);
            sm.lpResonance.snapTo(lpRes);
            sm.amplitude.snapTo(amp);
            sm.pan.snapTo(pan);
            sm.vactrolGate.snapTo(gateTarget);
            sm.pressure.snapTo(press);
            sm.keyGainMul.snapTo(kGain);
            sm.keyPitchMul.snapTo(kPitch);
            sm.stutterDepth.snapTo(kStutDepth);
            sm.stutterGate.snapTo(1.0f);
            sm.xmodDepth.snapTo(kXMod);
            sm.foldDepth.snapTo(kFold);
            sm.crushDepth.snapTo(kCrush);
            sm.stutterPhase = 0x80000000u;
            const float pm = state.global.pitchPressureMode ? 1.0f : 0.0f;
            prevOscFreqHz_[i] = clampf(clampedBaseHz * kPitch * computeVoicePitchRatio(press, pm),
                                       0.005f, kAudioFreqMaxHz * 4.0f);
        } else {
            sm.freqHz.setTarget(clampedBaseHz);
            sm.pulseWidth.setTarget(pw);
            sm.drift.setTarget(dr);
            sm.hpCutoffHz.setTarget(hpHz);
            sm.hpResonance.setTarget(hpRes);
            sm.lpCutoffHz.setTarget(lpHz);
            sm.lpResonance.setTarget(lpRes);
            if (sm.vactrolGate.current() <= 1.0e-4f && sm.pressure.current() <= 1.0e-4f &&
                gateTarget > 0.5f) {
                // Voice is currently completely silent: snap remembered amplitude to new target
                // before opening vactrol gate so fading in from 0% never blips upward.
                sm.amplitude.snapTo(amp);
            } else {
                sm.amplitude.setTarget(amp);
            }
            sm.pan.setTarget(pan);
            sm.vactrolGate.setTarget(gateTarget);
            sm.pressure.setTarget(press);
            sm.keyGainMul.setTarget(kGain);
            sm.keyPitchMul.setTarget(kPitch);
            sm.stutterDepth.setTarget(kStutDepth);
            sm.xmodDepth.setTarget(kXMod);
            sm.foldDepth.setTarget(kFold);
            sm.crushDepth.setTarget(kCrush);
        }
    }

    const float coupling = clampf(state.global.circuitCoupling, 0.0f, 1.0f);
    const float trim = clampf(state.global.masterTrim, 0.0f, 1.5f);
    const float monoTarget = state.global.monoCollapse ? 1.0f : 0.0f;
    const float runTarget = state.global.masterRun ? 1.0f : 0.0f;
    const float pitchTarget = state.global.pitchPressureMode ? 1.0f : 0.0f;
    const float surgeTarget = clampf(state.keyMod.overloadSurge, 0.0f, 1.0f);
    const float dubLpfTarget = clampf(state.keyMod.dubLpfDrop, 0.0f, 1.0f);
    const float spatialTarget = clampf(state.keyMod.spatialFocus, -1.0f, 1.0f);
    const float ringTarget = clampf(state.keyMod.ringModMix, 0.0f, 1.0f);
    const float syncTarget = clampf(state.keyMod.hardSyncAmount, 0.0f, 1.0f);
    const float screamTarget = clampf(state.keyMod.filterScream, 0.0f, 1.0f);
    stasisFreeze_ = state.keyMod.stasisFreeze;

    if (snapImmediate) {
        couplingSmoother_.snapTo(coupling);
        masterTrimSmoother_.snapTo(trim);
        monoMixSmoother_.snapTo(monoTarget);
        masterRunSmoother_.snapTo(runTarget);
        pitchModeSmoother_.snapTo(pitchTarget);
        overloadSurgeSmoother_.snapTo(surgeTarget);
        dubLpfDropSmoother_.snapTo(dubLpfTarget);
        spatialFocusSmoother_.snapTo(spatialTarget);
        ringModSmoother_.snapTo(ringTarget);
        hardSyncSmoother_.snapTo(syncTarget);
        filterScreamSmoother_.snapTo(screamTarget);
    } else {
        couplingSmoother_.setTarget(coupling);
        masterTrimSmoother_.setTarget(trim);
        monoMixSmoother_.setTarget(monoTarget);
        masterRunSmoother_.setTarget(runTarget);
        pitchModeSmoother_.setTarget(pitchTarget);
        overloadSurgeSmoother_.setTarget(surgeTarget);
        dubLpfDropSmoother_.setTarget(dubLpfTarget);
        spatialFocusSmoother_.setTarget(spatialTarget);
        ringModSmoother_.setTarget(ringTarget);
        hardSyncSmoother_.setTarget(syncTarget);
        filterScreamSmoother_.setTarget(screamTarget);
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
    std::array<float, kNumOscillators> blockVoicePeak{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> lastGateHigh{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> lastDriftNorm{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<VoiceLfoFrame, kNumVoiceLfos> lastLfo{};
    uint8_t lastStutterMask = 0x0Fu;

    for (size_t frame = 0u; frame < numFrames; ++frame) {
        // Step audio-rate (44.118 kHz) smoothers, 8x dual voice LFOs & prepare filter/pan coeffs
        const std::array<VoiceLfoFrame, kNumVoiceLfos> lfoFrames =
            lfoBank_.stepFrame(stasisFreeze_);
        lastLfo = lfoFrames;

        const float surge = overloadSurgeSmoother_.next();
        const float surgeDrive = surge * (0.62f + 0.38f * surge);
        const float dubDrop = dubLpfDropSmoother_.next();
        const float spatialFocus = spatialFocusSmoother_.next();
        const float ringModMix = ringModSmoother_.next();
        const float hardSync = hardSyncSmoother_.next();
        const float filterScream = filterScreamSmoother_.next();
        const float baseCoupling = couplingSmoother_.next();
        const float coupling = baseCoupling + (1.0f - baseCoupling) * (surge * surge);
        const float masterTrim = masterTrimSmoother_.next();
        const float monoMix = monoMixSmoother_.next();
        const float basePanScale = 1.0f - monoMix;
        const float panScale = (spatialFocus < 0.0f)
                                   ? (basePanScale * (1.0f + spatialFocus))
                                   : (basePanScale + (1.0f - basePanScale) * spatialFocus);
        const bool exactMono = (panScale <= 0.0f);
        const float finalScale = masterTrim * masterRunSmoother_.next();
        const float pitchMode = pitchModeSmoother_.next();
        uint8_t frameStutterMask = 0x0Fu;

        std::array<float, kNumOscillators> oscFreqs{};
        std::array<float, kNumOscillators> oscPulseWidths{};
        std::array<float, kNumOscillators> oscDrifts{};
        std::array<float, kNumOscillators> xmodDepths{};
        std::array<float, kNumOscillators> foldDepths{};
        std::array<float, kNumOscillators> crushDepths{};
        std::array<float, kNumOscillators> vcaGates{};
        std::array<float, kNumOscillators> gainL{};
        std::array<float, kNumOscillators> gainR{};
        std::array<float, kNumOscillators> voiceScopeScale{};

        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceSmoothers& sm = voiceSmoothers_[i];
            VoiceCoeffCache& cache = coeffCache_[i];
            const VoiceLfoFrame& lfoA = lfoFrames[i];
            const VoiceLfoFrame& lfoB = lfoFrames[4u + i];
            const auto& tdA = lfoA.targetDepths;
            const auto& tdB = lfoB.targetDepths;

            const float p = sm.pressure.next();
            const float baseGate = sm.vactrolGate.next();
            const float kGainMul = sm.keyGainMul.next();
            const float kPitchMul = sm.keyPitchMul.next();
            const float stutDepth = sm.stutterDepth.next();
            xmodDepths[i] = sm.xmodDepth.next();
            foldDepths[i] = sm.foldDepth.next();
            crushDepths[i] = sm.crushDepth.next();
            float stutMul = 1.0f;
            if (stutDepth > 1.0e-4f) {
                if (!stasisFreeze_) {
                    const uint32_t stutInc =
                        static_cast<uint32_t>(sm.stutterRateHz * hzToBasePhaseU32_ + 0.5f);
                    sm.stutterPhase += stutInc;
                }
                const bool chopOpen = (sm.stutterPhase < 0x80000000u);
                if (!chopOpen) {
                    frameStutterMask &= static_cast<uint8_t>(~(1u << i));
                }
                sm.stutterGate.setTarget(chopOpen ? 1.0f : 0.0f);
                const float gateEnv = sm.stutterGate.next();
                stutMul = 1.0f - stutDepth * (1.0f - gateEnv);
            } else {
                sm.stutterPhase = 0x80000000u;
                sm.stutterGate.setTarget(1.0f);
                (void)sm.stutterGate.next();
            }

            float baseHz = applyLfoPitchHz(sm.freqHz.next(),
                                           tdA[static_cast<size_t>(LfoTarget::Pitch)], lfoA.raw);
            baseHz = applyLfoPitchHz(baseHz, tdB[static_cast<size_t>(LfoTarget::Pitch)], lfoB.raw);
            const float pitchMul = computeVoicePitchRatio(p, pitchMode) * kPitchMul;
            oscFreqs[i] = (pitchMul != 1.0f)
                              ? clampf(baseHz * pitchMul, 0.005f, kAudioFreqMaxHz * 4.0f)
                              : baseHz;

            float modPw = applyLfoPulseWidth(
                sm.pulseWidth.next(), tdA[static_cast<size_t>(LfoTarget::PulseWidth)], lfoA.raw);
            oscPulseWidths[i] = applyLfoPulseWidth(
                modPw, tdB[static_cast<size_t>(LfoTarget::PulseWidth)], lfoB.raw);
            oscDrifts[i] = sm.drift.next();

            float hpBase = applyLfoCutoffHz(
                sm.hpCutoffHz.next(), tdA[static_cast<size_t>(LfoTarget::HpCutoff)], lfoA.raw);
            hpBase =
                applyLfoCutoffHz(hpBase, tdB[static_cast<size_t>(LfoTarget::HpCutoff)], lfoB.raw);
            float hpResBase = applyLfoResonance(
                sm.hpResonance.next(), tdA[static_cast<size_t>(LfoTarget::HpResonance)], lfoA.raw);
            hpResBase = applyLfoResonance(
                hpResBase, tdB[static_cast<size_t>(LfoTarget::HpResonance)], lfoB.raw);
            float lpBase = applyLfoCutoffHz(
                sm.lpCutoffHz.next(), tdA[static_cast<size_t>(LfoTarget::LpCutoff)], lfoA.raw);
            lpBase =
                applyLfoCutoffHz(lpBase, tdB[static_cast<size_t>(LfoTarget::LpCutoff)], lfoB.raw);
            float lpResBase = applyLfoResonance(
                sm.lpResonance.next(), tdA[static_cast<size_t>(LfoTarget::LpResonance)], lfoA.raw);
            lpResBase = applyLfoResonance(
                lpResBase, tdB[static_cast<size_t>(LfoTarget::LpResonance)], lfoB.raw);

            ModulatedFilterParams mod =
                applyKeyDubLowPass(computeVoicePressureMacro(hpBase, hpResBase, lpBase, lpResBase,
                                                             p, baseGate, pitchMode),
                                   dubDrop, surge, filterScream);

            float modAmp = applyLfoAmplitude(
                sm.amplitude.next(), tdA[static_cast<size_t>(LfoTarget::Amplitude)], lfoA.raw);
            modAmp =
                applyLfoAmplitude(modAmp, tdB[static_cast<size_t>(LfoTarget::Amplitude)], lfoB.raw);
            modAmp = clampf(modAmp + (1.0f - modAmp) * surgeDrive, 0.0f, 1.0f);
            const VoiceChannelGain chGain = computeVoiceChannelGain(modAmp);

            if (mod.hpCutoffHz != cache.hpHz || mod.hpResonance != cache.hpRes ||
                mod.lpCutoffHz != cache.lpHz || mod.lpResonance != cache.lpRes ||
                chGain.preDrive != cache.preDrive) {
                cache.hpHz = mod.hpCutoffHz;
                cache.hpRes = mod.hpResonance;
                cache.lpHz = mod.lpCutoffHz;
                cache.lpRes = mod.lpResonance;
                cache.preDrive = chGain.preDrive;
                filterCoeffs_[i] = Ms20DualFilter::prepareCoeffs(
                    mod.hpCutoffHz, mod.hpResonance, mod.lpCutoffHz, mod.lpResonance,
                    invOversampledRate_, chGain.preDrive);
            }

            const float ampGate = chGain.postVca * mod.effectiveGate * kGainMul * stutMul;
            vcaGates[i] = ampGate;
            const float voiceGain = ampGate * kVoiceSumScale;
            voiceScopeScale[i] = ampGate * finalScale * 0.25f;
            float modPan =
                applyLfoPan(sm.pan.next(), tdA[static_cast<size_t>(LfoTarget::Pan)], lfoA.raw);
            modPan = applyLfoPan(modPan, tdB[static_cast<size_t>(LfoTarget::Pan)], lfoB.raw);
            if (spatialFocus > 0.0f) {
                modPan += (kWideFanPans[i] - modPan) * spatialFocus;
            }
            const float panEff = clampf(modPan * panScale, -1.0f, 1.0f);
            if (panEff != cache.panEff) {
                cache.panEff = panEff;
                equalPowerPan(panEff, cache.panL, cache.panR);
            }
            gainL[i] = voiceGain * cache.panL;
            gainR[i] = voiceGain * cache.panR;
        }
        lastStutterMask = frameStutterMask;

        // Run unified 4x oversampled (176.472 kHz) synthesis, rail coupling, ZDF filters & bus sag
        std::array<std::array<float, kOversampleFactor>, kNumOscillators> voiceSub{};
        std::array<StereoFrame, kOversampleFactor> osStereo{};
        std::array<float, kNumOscillators> voiceScopeAccum{0.0f, 0.0f, 0.0f, 0.0f};
        uint8_t voiceRisingInFrame = 0u;

        for (size_t sub = 0u; sub < kOversampleFactor; ++sub) {
            std::array<float, kNumOscillators> subFreqs{};
            const float subAlpha = 0.25f * static_cast<float>(sub + 1u);
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                subFreqs[i] = prevOscFreqHz_[i] + (oscFreqs[i] - prevOscFreqHz_[i]) * subAlpha;
            }
            applyCrossPinFm4x(subFreqs, prevCapRamp_, prevOscSamples_, xmodDepths);
            const OscillatorBankOutput oscOut = oscBank_.processSample(
                subFreqs, oscPulseWidths, oscDrifts, coupling, stasisFreeze_, hardSync);
            prevOscSamples_ = oscOut.samples;
            prevCapRamp_ = oscOut.capRamp;
            std::array<float, kNumOscillators> patchedSamples = oscOut.samples;
            applyCrossPinXor4x(patchedSamples, oscOut.capRamp, xmodDepths);
            applyCapacitorWavefold4x(patchedSamples, oscOut.capRamp, foldDepths);
            applyRippleSubXorPreFilter4x(patchedSamples, oscOut.risingMask, crushDepths,
                                         crushStates_);
            applyRingXorMatrix4x(patchedSamples, ringModMix);
            voiceRisingInFrame |= oscOut.risingMask;
            lastGateHigh = oscOut.gateHigh;
            lastDriftNorm = oscOut.driftNorm;
            for (size_t i = 0u; i < kNumOscillators; ++i) {
                voiceSub[i][sub] = patchedSamples[i];
            }
        }
        prevOscFreqHz_ = oscFreqs;

        Ms20DualFilter::processQuadBank4x(filters_, voiceSub, filterCoeffs_);
        applySampleHoldCrushPostFilter4x(voiceSub, crushDepths, hzToOversampledPhaseU32_,
                                         crushStates_);
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            voiceScopeAccum[i] = voiceSub[i][0] + voiceSub[i][1] + voiceSub[i][2] + voiceSub[i][3];
            const float vAbs = absf(voiceScopeAccum[i]) * 0.25f * vcaGates[i];
            if (vAbs > blockVoicePeak[i]) {
                blockVoicePeak[i] = vAbs;
            }
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

        // Linear-phase 4x -> 1x decimation to 44.118 kHz + ultrasonic Gibbs DAC seatbelt
        const StereoFrame out = decimator_.process4x(osStereo);
        const float cleanL = dacSafetyClip(sanitizeState(out.left));
        const float cleanR = dacSafetyClip(sanitizeState(out.right));

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
        voiceScopeDelay_[voiceScopeDelayWrite_] = curVoiceQ13;
        const uint32_t nextDelaySlot = (voiceScopeDelayWrite_ + 1u < kScopeAlignDelaySlots)
                                           ? (voiceScopeDelayWrite_ + 1u)
                                           : 0u;
        const std::array<int16_t, kNumOscillators> delayedVoices = voiceScopeDelay_[nextDelaySlot];
        voiceScopeDelayWrite_ = nextDelaySlot;

        uint8_t delayedMask = 0u;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const uint64_t bit = ((voiceRisingInFrame >> v) & 1u) != 0u ? 1ULL : 0ULL;
            voiceEdgeDelayLines_[v] = (voiceEdgeDelayLines_[v] << 1u) | bit;
            if (((voiceEdgeDelayLines_[v] >> 36u) & 1ULL) != 0ULL) {
                delayedMask |= static_cast<uint8_t>(1u << v);
            }
        }
        scopeBuffer_.push({cleanL, cleanR, delayedMask, delayedVoices});
    }

    telemetry_.peakLeft.store(blockPeakL, std::memory_order_relaxed);
    telemetry_.peakRight.store(blockPeakR, std::memory_order_relaxed);
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        telemetry_.oscGateHigh[i].store(lastGateHigh[i], std::memory_order_relaxed);
        telemetry_.voicePeak[i].store(blockVoicePeak[i], std::memory_order_relaxed);
        telemetry_.voiceDriftNorm[i].store(lastDriftNorm[i], std::memory_order_relaxed);
    }
    telemetry_.stutterGateMask.store(lastStutterMask, std::memory_order_relaxed);
    for (size_t s = 0u; s < kNumVoiceLfos; ++s) {
        telemetry_.lfoOutput[s].store(lastLfo[s].raw, std::memory_order_relaxed);
        telemetry_.lfoPhase[s].store(lastLfo[s].phase01, std::memory_order_relaxed);
        telemetry_.lfoDriftPacked[s].store(lastLfo[s].driftPacked, std::memory_order_relaxed);
    }
}

} // namespace ruido
