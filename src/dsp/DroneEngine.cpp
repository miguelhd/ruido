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
    rhythmState_.init(baseSampleRate_, kPunchBusSlewSec);
    stutterPhases_.fill(0x80000000u);
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
        sm.hpCutoffHz.init(initState.voices[i].hpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.lpCutoffHz.init(initState.voices[i].lpCutoffHz, kPotPoleTauSec, baseSampleRate_);
        sm.keyPitchMul.init(initState.keyMod.voicePitchMul[i], kPotPoleTauSec, baseSampleRate_);
        sm.potSmoothers.setTimeConstant(kPotPoleTauSec, baseSampleRate_);
        sm.potSmoothers.snapChannel(0u, initState.voices[i].pulseWidth);
        sm.potSmoothers.snapChannel(1u, initState.voices[i].drift);
        sm.potSmoothers.snapChannel(2u, initState.voices[i].hpResonance);
        sm.potSmoothers.snapChannel(3u, initState.voices[i].lpResonance);
        sm.potSmoothers.snapChannel(4u, initState.voices[i].amplitude);
        sm.potSmoothers.snapChannel(5u, initState.voices[i].pan);
        sm.pressure.init(initState.voicePressure[i], kPressureAttackSec, kPressureReleaseSec,
                         baseSampleRate_);
        sm.stutterGate.init(1.0f, kStutterEdgeSec, baseSampleRate_);
        sm.vactrolMods.setTimeConstant(kVactrolSlewSec, baseSampleRate_);
        sm.vactrolMods.snapChannel(0u, initState.voices[i].muted ? 0.0f : 1.0f);
        sm.vactrolMods.snapChannel(1u, initState.keyMod.voiceGainMul[i]);
        sm.vactrolMods.snapChannel(2u, initState.keyMod.stutterDepth[i]);
        sm.circuitDepths.setTimeConstant(kPunchBusSlewSec, baseSampleRate_);
        sm.circuitDepths.snapChannel(0u, initState.keyMod.xmodDepth[i]);
        sm.circuitDepths.snapChannel(1u, initState.keyMod.foldDepth[i]);
        sm.circuitDepths.snapChannel(2u, initState.keyMod.crushDepth[i]);
        stutterPhases_[i] = 0x80000000u;
        stutterRatesHz_[i] = initState.keyMod.stutterRateHz[i];
    }

    couplingSmoother_.init(initState.global.circuitCoupling, kPotPoleTauSec, baseSampleRate_);
    masterTrimSmoother_.init(initState.global.masterTrim, kPotPoleTauSec, baseSampleRate_);
    monoMixSmoother_.init(initState.global.monoCollapse ? 1.0f : 0.0f, kVactrolSlewSec,
                          baseSampleRate_);
    masterRunSmoother_.init(initState.global.masterRun ? 1.0f : 0.0f, kMasterKillSec,
                            baseSampleRate_);
    pitchModeSmoother_.init(initState.global.pitchPressureMode ? 1.0f : 0.0f, kControlSlewSec,
                            baseSampleRate_);
    punchBusSmoothers_.setTimeConstant(kPunchBusSlewSec, baseSampleRate_);
    punchBusSmoothers_.snapChannel(0u, initState.keyMod.overloadSurge);
    punchBusSmoothers_.snapChannel(1u, initState.keyMod.dubLpfDrop);
    punchBusSmoothers_.snapChannel(2u, initState.keyMod.spatialFocus);
    punchBusSmoothers_.snapChannel(3u, initState.keyMod.ringModMix);
    punchBusSmoothers_.snapChannel(4u, initState.keyMod.hardSyncAmount);
    punchBusSmoothers_.snapChannel(5u, initState.keyMod.filterScream);
    stasisFreeze_ = initState.keyMod.stasisFreeze;
    telemetry_.stutterGateMask.store(0x0Fu, std::memory_order_relaxed);
    telemetry_.rhythmPhase.store(0.0f, std::memory_order_relaxed);

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
    rhythmState_.init(baseSampleRate_, kPunchBusSlewSec);
    stutterPhases_.fill(0x80000000u);
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
        stutterRatesHz_[i] = clampf(state.keyMod.stutterRateHz[i], 1.0f, 30.0f);

        if (snapImmediate) {
            sm.freqHz.snapTo(clampedBaseHz);
            sm.hpCutoffHz.snapTo(hpHz);
            sm.lpCutoffHz.snapTo(lpHz);
            sm.potSmoothers.snapChannel(0u, pw);
            sm.potSmoothers.snapChannel(1u, dr);
            sm.potSmoothers.snapChannel(2u, hpRes);
            sm.potSmoothers.snapChannel(3u, lpRes);
            sm.potSmoothers.snapChannel(4u, amp);
            sm.potSmoothers.snapChannel(5u, pan);
            sm.pressure.snapTo(press);
            sm.keyPitchMul.snapTo(kPitch);
            sm.stutterGate.snapTo(1.0f);
            sm.vactrolMods.snapChannel(0u, gateTarget);
            sm.vactrolMods.snapChannel(1u, kGain);
            sm.vactrolMods.snapChannel(2u, kStutDepth);
            sm.circuitDepths.snapChannel(0u, kXMod);
            sm.circuitDepths.snapChannel(1u, kFold);
            sm.circuitDepths.snapChannel(2u, kCrush);
            stutterPhases_[i] = 0x80000000u;
            const float pm = state.global.pitchPressureMode ? 1.0f : 0.0f;
            prevOscFreqHz_[i] = clampf(clampedBaseHz * kPitch * computeVoicePitchRatio(press, pm),
                                       0.005f, kAudioFreqMaxHz * 4.0f);
        } else {
            sm.freqHz.setTarget(clampedBaseHz);
            sm.hpCutoffHz.setTarget(hpHz);
            sm.lpCutoffHz.setTarget(lpHz);
            sm.potSmoothers.setTarget(0u, pw);
            sm.potSmoothers.setTarget(1u, dr);
            sm.potSmoothers.setTarget(2u, hpRes);
            sm.potSmoothers.setTarget(3u, lpRes);
            if (sm.vactrolMods.current(0u) <= 1.0e-4f && sm.pressure.current() <= 1.0e-4f &&
                gateTarget > 0.5f) {
                // Voice is currently completely silent: snap remembered amplitude to new target
                // before opening vactrol gate so fading in from 0% never blips upward.
                sm.potSmoothers.snapChannel(4u, amp);
            } else {
                sm.potSmoothers.setTarget(4u, amp);
            }
            sm.potSmoothers.setTarget(5u, pan);
            sm.pressure.setTarget(press);
            sm.keyPitchMul.setTarget(kPitch);
            sm.vactrolMods.setTarget(0u, gateTarget);
            sm.vactrolMods.setTarget(1u, kGain);
            sm.vactrolMods.setTarget(2u, kStutDepth);
            sm.circuitDepths.setTarget(0u, kXMod);
            sm.circuitDepths.setTarget(1u, kFold);
            sm.circuitDepths.setTarget(2u, kCrush);
        }
    }

    const float coupling = clampf(state.global.circuitCoupling, 0.0f, 1.0f);
    const float trim = clampf(state.global.masterTrim, 0.0f, 1.5f);
    const float monoTarget = state.global.monoCollapse ? 1.0f : 0.0f;
    const float runTarget = state.global.masterRun ? 1.0f : 0.0f;
    const float pitchTarget = state.global.pitchPressureMode ? 1.0f : 0.0f;
    const std::array<float, 6> punchTargets = {
        clampf(state.keyMod.overloadSurge, 0.0f, 1.0f),
        clampf(state.keyMod.dubLpfDrop, 0.0f, 1.0f),
        clampf(state.keyMod.spatialFocus, -1.0f, 1.0f),
        clampf(state.keyMod.ringModMix, 0.0f, 1.0f),
        clampf(state.keyMod.hardSyncAmount, 0.0f, 1.0f),
        clampf(state.keyMod.filterScream, 0.0f, 1.0f),
    };
    stasisFreeze_ = state.keyMod.stasisFreeze;

    if (snapImmediate) {
        couplingSmoother_.snapTo(coupling);
        masterTrimSmoother_.snapTo(trim);
        monoMixSmoother_.snapTo(monoTarget);
        masterRunSmoother_.snapTo(runTarget);
        pitchModeSmoother_.snapTo(pitchTarget);
        for (size_t k = 0u; k < 6u; ++k) {
            punchBusSmoothers_.snapChannel(k, punchTargets[k]);
        }
    } else {
        couplingSmoother_.setTarget(coupling);
        masterTrimSmoother_.setTarget(trim);
        monoMixSmoother_.setTarget(monoTarget);
        masterRunSmoother_.setTarget(runTarget);
        pitchModeSmoother_.setTarget(pitchTarget);
        for (size_t k = 0u; k < 6u; ++k) {
            punchBusSmoothers_.setTarget(k, punchTargets[k]);
        }
    }

    rhythmState_.updateTargets(state, snapImmediate);
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

        const float surge = punchBusSmoothers_.next(0u);
        const float surgeDrive = surge * (0.62f + 0.38f * surge);
        const float dubDrop = punchBusSmoothers_.next(1u);
        const float spatialFocus = punchBusSmoothers_.next(2u);
        const float ringModMix = punchBusSmoothers_.next(3u);
        const float hardSync = punchBusSmoothers_.next(4u);
        const float filterScream = punchBusSmoothers_.next(5u);
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
        std::array<float, kNumOscillators> stutDepths{};
        std::array<float, kNumOscillators> xmodDepths{};
        std::array<float, kNumOscillators> foldDepths{};
        std::array<float, kNumOscillators> crushDepths{};
        std::array<float, kNumOscillators> vcaGates{};
        std::array<float, kNumOscillators> gainL{};
        std::array<float, kNumOscillators> gainR{};
        std::array<float, kNumOscillators> voiceScopeScale{};
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            stutDepths[i] = voiceSmoothers_[i].vactrolMods.next(2u);
        }
        const RhythmFrameResult rhythmFrame =
            stepRhythmFrame(rhythmState_, stutterPhases_, stutterRatesHz_, stutDepths,
                            hzToBasePhaseU32_, stasisFreeze_);
        frameStutterMask = rhythmFrame.gateMask;

        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceSmoothers& sm = voiceSmoothers_[i];
            VoiceCoeffCache& cache = coeffCache_[i];
            const VoiceLfoFrame& lfoA = lfoFrames[i];
            const VoiceLfoFrame& lfoB = lfoFrames[4u + i];
            const auto& tdA = lfoA.targetDepths;
            const auto& tdB = lfoB.targetDepths;

            const float p = sm.pressure.next();
            const float baseGate = sm.vactrolMods.next(0u);
            const float kGainMul = sm.vactrolMods.next(1u);
            const float kPitchMul = sm.keyPitchMul.next();
            xmodDepths[i] = sm.circuitDepths.next(0u);
            foldDepths[i] = sm.circuitDepths.next(1u);
            crushDepths[i] = sm.circuitDepths.next(2u);
            sm.stutterGate.setTarget(rhythmFrame.vcaGateTarget[i]);
            const float stutMul = sm.stutterGate.next();

            float baseHz = applyLfoPitchHz(sm.freqHz.next(),
                                           tdA[static_cast<size_t>(LfoTarget::Pitch)], lfoA.raw);
            baseHz = applyLfoPitchHz(baseHz, tdB[static_cast<size_t>(LfoTarget::Pitch)], lfoB.raw);
            const float pitchMul = computeVoicePitchRatio(p, pitchMode) * kPitchMul;
            oscFreqs[i] = (pitchMul != 1.0f)
                              ? clampf(baseHz * pitchMul, 0.005f, kAudioFreqMaxHz * 4.0f)
                              : baseHz;

            float modPw =
                applyLfoPulseWidth(sm.potSmoothers.next(0u),
                                   tdA[static_cast<size_t>(LfoTarget::PulseWidth)], lfoA.raw);
            oscPulseWidths[i] = applyLfoPulseWidth(
                modPw, tdB[static_cast<size_t>(LfoTarget::PulseWidth)], lfoB.raw);
            oscDrifts[i] = sm.potSmoothers.next(1u);

            float hpBase = applyLfoCutoffHz(
                sm.hpCutoffHz.next(), tdA[static_cast<size_t>(LfoTarget::HpCutoff)], lfoA.raw);
            hpBase =
                applyLfoCutoffHz(hpBase, tdB[static_cast<size_t>(LfoTarget::HpCutoff)], lfoB.raw);
            float hpResBase =
                applyLfoResonance(sm.potSmoothers.next(2u),
                                  tdA[static_cast<size_t>(LfoTarget::HpResonance)], lfoA.raw);
            hpResBase = applyLfoResonance(
                hpResBase, tdB[static_cast<size_t>(LfoTarget::HpResonance)], lfoB.raw);
            float lpBase = applyLfoCutoffHz(
                sm.lpCutoffHz.next(), tdA[static_cast<size_t>(LfoTarget::LpCutoff)], lfoA.raw);
            lpBase =
                applyLfoCutoffHz(lpBase, tdB[static_cast<size_t>(LfoTarget::LpCutoff)], lfoB.raw);
            float lpResBase =
                applyLfoResonance(sm.potSmoothers.next(3u),
                                  tdA[static_cast<size_t>(LfoTarget::LpResonance)], lfoA.raw);
            lpResBase = applyLfoResonance(
                lpResBase, tdB[static_cast<size_t>(LfoTarget::LpResonance)], lfoB.raw);

            ModulatedFilterParams mod = applyKeyDubLowPass(
                computeVoicePressureMacro(hpBase, hpResBase, lpBase, lpResBase, p, baseGate,
                                          pitchMode),
                dubDrop, surge, filterScream, rhythmFrame.pluckDepth, rhythmFrame.pluckEnv01[i]);

            float modAmp = applyLfoAmplitude(
                sm.potSmoothers.next(4u), tdA[static_cast<size_t>(LfoTarget::Amplitude)], lfoA.raw);
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
            float modPan = applyLfoPan(sm.potSmoothers.next(5u),
                                       tdA[static_cast<size_t>(LfoTarget::Pan)], lfoA.raw);
            modPan = applyLfoPan(modPan, tdB[static_cast<size_t>(LfoTarget::Pan)], lfoB.raw);
            if (spatialFocus > 0.0f) {
                modPan += (kWideFanPans[i] - modPan) * spatialFocus;
            }
            modPan = clampf(modPan + rhythmFrame.swingPanOffset, -1.0f, 1.0f);
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
    telemetry_.rhythmPhase.store(rhythmState_.warpedBar01, std::memory_order_relaxed);
    for (size_t s = 0u; s < kNumVoiceLfos; ++s) {
        telemetry_.lfoOutput[s].store(lastLfo[s].raw, std::memory_order_relaxed);
        telemetry_.lfoPhase[s].store(lastLfo[s].phase01, std::memory_order_relaxed);
        telemetry_.lfoDriftPacked[s].store(lastLfo[s].driftPacked, std::memory_order_relaxed);
    }
}

} // namespace ruido
