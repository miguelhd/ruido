#include "dsp/DroneEngine.h"

#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/LfoMath.h"
#include "dsp/CircuitPatchMatrix.h"

namespace ruido {

namespace {

constexpr float kPotPoleTauSec = 0.0140f;      // 2-pole C^1 pot glide (~28 ms mean, 0% overshoot)
constexpr float kMasterTrimTauSec = 0.0040f;   // 4.0 ms real-time analog MASTER volume pot glide
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

RUIDO_NOINLINE void DroneEngineCore::init(EngineSharedBuses& buses, float baseSampleRate) noexcept {
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
    spanCoeffs_ = {};
    crushStates_ = {};
    rhythmState_.init(baseSampleRate_, kPunchBusSlewSec);
    stutterPhases_.fill(0x80000000u);
    prevOscSamples_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prevCapRamp_ = {0.0f, 0.0f, 0.0f, 0.0f};
    voiceRisingMaskDelay_.fill(0u);
    voiceScopeDelay_ = {};
    voiceScopeDelayWrite_ = 0u;
    silentFilterMask_ = 0u;
    spanMeta_ = {};
    inFlightMetaIdx_ = 0u;
    hasInFlightSpan_ = false;

    // Smoothers step once per 44.118 kHz output audio frame (100% sample-accurate audio rate
    // smoothing while saving 4x redundant coefficient / pan updates inside the 4x inner loop).
    const EngineControlState& initState = buses.controlBus.consumeLatest();
    lastControlPtr_ = &initState;
    lastControlSeq_ = buses.controlBus.readSeq();
    prevStutterEdgeSec_ = kStutterEdgeSec;
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        VoiceSmoothers& sm = voiceSmoothers_[i];
        sm.logSmoothers.setTimeConstant(kPotPoleTauSec, baseSampleRate_);
        sm.logSmoothers.snapChannel(0u, initState.voices[i].frequencyHz);
        sm.logSmoothers.snapChannel(1u, initState.voices[i].hpCutoffHz);
        sm.logSmoothers.snapChannel(2u, initState.voices[i].lpCutoffHz);
        sm.logSmoothers.snapChannel(3u, initState.keyMod.voicePitchMul[i]);
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
    masterTrimSmoother_.init(initState.global.masterTrim, kMasterTrimTauSec, baseSampleRate_);
    monoMixSmoother_.init(initState.global.monoCollapse ? 1.0f : 0.0f, kVactrolSlewSec,
                          baseSampleRate_);
    masterRunSmoother_.init(initState.global.masterRun ? 1.0f : 0.0f, kMasterKillSec,
                            baseSampleRate_);
    pitchModeSmoother_.init(initState.global.pitchPressureMode ? 1.0f : 0.0f, kControlSlewSec,
                            baseSampleRate_);
    punchBusSmoothers_.setTimeConstant(kPunchBusSlewSec, baseSampleRate_);
    buses.telemetry.stutterGateMask.store(0x0Fu, std::memory_order_relaxed);
    buses.telemetry.rhythmPhase.store(0.0f, std::memory_order_relaxed);

    updateTargetsFromState(initState, true);
    lfoBank_.writeInitialTelemetry(buses.telemetry);
}

RUIDO_NOINLINE void DroneEngineCore::reset(EngineSharedBuses& buses) noexcept {
    init(buses, baseSampleRate_);
}

RUIDO_NOINLINE void DroneEngineCore::snapParametersToLatest(EngineSharedBuses& buses) noexcept {
    const EngineControlState& state = buses.controlBus.consumeLatest();
    lastControlPtr_ = &state;
    lastControlSeq_ = buses.controlBus.readSeq();
    updateTargetsFromState(state, true);
}

RUIDO_NOINLINE void DroneEngineCore::updateTargetsFromState(const EngineControlState& state,
                                                            bool snapImmediate) noexcept {
    const float effClockBpm =
        clampf(state.global.clockBpm, kMinInternalClockBpm, kMaxInternalClockBpm) *
        computeTempoMultiplier(clampf(state.keyMod.rhythmTempo, 0.0f, 1.0f));
    const float adaptiveGateEdgeSec =
        (effClockBpm > 300.0f)
            ? clampf(kStutterEdgeSec * (300.0f / effClockBpm), 0.00035f, kStutterEdgeSec)
            : kStutterEdgeSec;
    if (adaptiveGateEdgeSec != prevStutterEdgeSec_ || snapImmediate) {
        prevStutterEdgeSec_ = adaptiveGateEdgeSec;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            voiceSmoothers_[i].stutterGate.setTimeConstant(adaptiveGateEdgeSec, baseSampleRate_);
        }
    }

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
            sm.logSmoothers.snapChannel(0u, clampedBaseHz);
            sm.logSmoothers.snapChannel(1u, hpHz);
            sm.logSmoothers.snapChannel(2u, lpHz);
            sm.potSmoothers.snapChannel(0u, pw);
            sm.potSmoothers.snapChannel(1u, dr);
            sm.potSmoothers.snapChannel(2u, hpRes);
            sm.potSmoothers.snapChannel(3u, lpRes);
            sm.potSmoothers.snapChannel(4u, amp);
            sm.potSmoothers.snapChannel(5u, pan);
            sm.pressure.snapTo(press);
            sm.logSmoothers.snapChannel(3u, kPitch);
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
            sm.logSmoothers.setTarget(0u, clampedBaseHz);
            sm.logSmoothers.setTarget(1u, hpHz);
            sm.logSmoothers.setTarget(2u, lpHz);
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
            sm.logSmoothers.setTarget(3u, kPitch);
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
    const std::array<float, 7> punchTargets = {
        clampf(state.keyMod.overloadSurge, 0.0f, 1.0f),
        clampf(state.keyMod.dubLpfDrop, 0.0f, 1.0f),
        clampf(state.keyMod.spatialFocus, -1.0f, 1.0f),
        clampf(state.keyMod.ringModMix, 0.0f, 1.0f),
        clampf(state.keyMod.hardSyncAmount, 0.0f, 1.0f),
        clampf(state.keyMod.filterScream, 0.0f, 1.0f),
        clampf(state.keyMod.masterHpfSweep, 0.0f, 1.0f),
    };
    stasisFreeze_ = state.keyMod.stasisFreeze;

    if (snapImmediate) {
        couplingSmoother_.snapTo(coupling);
        masterTrimSmoother_.snapTo(trim);
        monoMixSmoother_.snapTo(monoTarget);
        masterRunSmoother_.snapTo(runTarget);
        pitchModeSmoother_.snapTo(pitchTarget);
        for (size_t k = 0u; k < 7u; ++k) {
            punchBusSmoothers_.snapChannel(k, punchTargets[k]);
        }
        prevStatesValid_ = false;
    } else {
        couplingSmoother_.setTarget(coupling);
        masterTrimSmoother_.setTarget(trim);
        monoMixSmoother_.setTarget(monoTarget);
        masterRunSmoother_.setTarget(runTarget);
        pitchModeSmoother_.setTarget(pitchTarget);
        for (size_t k = 0u; k < 7u; ++k) {
            punchBusSmoothers_.setTarget(k, punchTargets[k]);
        }
    }

    rhythmState_.updateTargets(state, snapImmediate);
    lfoBank_.updateTargets(state, snapImmediate);
}

void DroneEngineCore::processBlock(EngineSharedBuses& buses, float* outLeft, float* outRight,
                                   size_t numFrames) noexcept {
    if (numFrames == 0u || outLeft == nullptr || outRight == nullptr) {
        return;
    }

    const EngineControlState& latestState = buses.controlBus.consumeLatest();
    const uint32_t curSeq = buses.controlBus.readSeq();
    if (curSeq != lastControlSeq_ || &latestState != lastControlPtr_) {
        lastControlPtr_ = &latestState;
        lastControlSeq_ = curSeq;
        updateTargetsFromState(latestState, false);
    }

    float blockPeakL = 0.0f;
    float blockPeakR = 0.0f;
    std::array<float, kNumOscillators> blockVoicePeak{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> lastGateHigh{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, kNumOscillators> lastDriftNorm{0.0f, 0.0f, 0.0f, 0.0f};
    auto& lfoFrames = lfoFrames_;
    uint8_t lastStutterMask = 0x0Fu;

    const bool fastMod = lfoBank_.hasFastModulation() || rhythmState_.hasFastRhythm();
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
    (void)fastMod;
    constexpr size_t maxSubBlock = kMaxFilterSpanFrames;
#else
    const size_t maxSubBlock = fastMod ? 1u : 16u;
#endif

    size_t curSpanSlot = 0u;
    hasInFlightSpan_ = false;
    for (size_t baseFrame = 0u; baseFrame < numFrames;) {
        const size_t remFrames = numFrames - baseFrame;
        const size_t chunk = (remFrames < maxSubBlock) ? remFrames : maxSubBlock;
        const auto stride = static_cast<uint32_t>(chunk);
        const bool isLastChunk = ((baseFrame + chunk) == numFrames);
        const size_t curBuf = curSpanSlot & 1u;
        ++curSpanSlot;

        // Step smoothers, 8x dual voice LFOs & prepare filter/pan coeffs per sub-block
        const float runScale = masterRunSmoother_.nextN(stride);
        const bool stoppedSilent =
            !latestState.global.masterRun && (runScale <= 1.0e-4f) && (prevRunScale_ <= 1.0e-4f);
        const bool effFreeze = stasisFreeze_ || stoppedSilent;
        lfoBank_.stepSubBlock(lfoFrames, stride, effFreeze, isLastChunk);

        const float surge = punchBusSmoothers_.nextN(0u, stride);
        const float surgeDrive = surge * (0.62f + 0.38f * surge);
        const float dubDrop = punchBusSmoothers_.nextN(1u, stride);
        const float spatialFocus = punchBusSmoothers_.nextN(2u, stride);
        const float ringModMix = punchBusSmoothers_.nextN(3u, stride);
        const float hardSync = punchBusSmoothers_.nextN(4u, stride);
        const float filterScream = punchBusSmoothers_.nextN(5u, stride);
        const float hpfSweep = punchBusSmoothers_.nextN(6u, stride);
        const float baseCoupling = couplingSmoother_.nextN(stride);
        const float coupling = baseCoupling + (1.0f - baseCoupling) * (surge * surge);
        const float masterTrim = masterTrimSmoother_.nextN(stride);
        const float monoMix = monoMixSmoother_.nextN(stride);
        const float basePanScale = 1.0f - monoMix;
        const float panScale = (spatialFocus < 0.0f)
                                   ? (basePanScale * (1.0f + spatialFocus))
                                   : (basePanScale + (1.0f - basePanScale) * spatialFocus);
        const bool exactMono = (panScale <= 0.0f);
        const float pitchMode = pitchModeSmoother_.nextN(stride);

        std::array<float, kNumOscillators> oscFreqs, oscPulseWidths, oscDrifts, stutDepths;
        std::array<float, kNumOscillators> xmodDepths, foldDepths, crushDepths;
        std::array<float, kNumOscillators> vcaGates, gainL, gainR, voiceScopeScale;
        for (size_t i = 0u; i < kNumOscillators; ++i) {
            stutDepths[i] = voiceSmoothers_[i].vactrolMods.nextN(2u, stride);
        }
        const RhythmFrameResult rhythmFrame =
            stepRhythmSubBlock(rhythmState_, stutterPhases_, stutterRatesHz_, stutDepths,
                               hzToBasePhaseU32_, effFreeze, stride);
        lastStutterMask = rhythmFrame.gateMask;
        const bool anyFilterMacro = (surge > 0.22f) || (dubDrop > 1.0e-4f) ||
                                    (hpfSweep > 1.0e-4f) || (filterScream > 1.0e-4f) ||
                                    (rhythmFrame.pluckDepth > 1.0e-4f);
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
        const bool refreshCoeffs = ((baseFrame & 31u) == 0u) || isLastChunk;
#else
        const bool refreshCoeffs = (chunk >= 4u) || ((baseFrame & 3u) == 0u) || isLastChunk;
#endif

        for (size_t i = 0u; i < kNumOscillators; ++i) {
            VoiceSmoothers& sm = voiceSmoothers_[i];
            VoiceCoeffCache& cache = coeffCache_[i];
            const VoiceLfoFrame& lfoA = lfoFrames[i];
            const VoiceLfoFrame& lfoB = lfoFrames[4u + i];

            const float p = sm.pressure.nextN(stride);
            const float baseGate = sm.vactrolMods.nextN(0u, stride);
            const float kGainMul = sm.vactrolMods.nextN(1u, stride);
            const float kPitchMul = sm.logSmoothers.nextN(3u, stride);
            xmodDepths[i] = sm.circuitDepths.nextN(0u, stride);
            foldDepths[i] = sm.circuitDepths.nextN(1u, stride);
            crushDepths[i] = sm.circuitDepths.nextN(2u, stride);
            sm.stutterGate.setTarget(rhythmFrame.vcaGateTarget[i]);
            const float stutMul = sm.stutterGate.nextN(stride);

            float baseHz = sm.logSmoothers.nextN(0u, stride);
            float modPw = sm.potSmoothers.nextN(0u, stride);
            oscDrifts[i] = sm.potSmoothers.nextN(1u, stride);
            float hpBase = sm.logSmoothers.nextN(1u, stride);
            float hpResBase = sm.potSmoothers.nextN(2u, stride);
            float lpBase = sm.logSmoothers.nextN(2u, stride);
            float lpResBase = sm.potSmoothers.nextN(3u, stride);
            float modAmp = sm.potSmoothers.nextN(4u, stride);
            float modPan = sm.potSmoothers.nextN(5u, stride);

            if (lfoA.activeDepth > 0.0f) {
                VoiceLfoBank::applySlot(lfoA, baseHz, modPw, hpBase, hpResBase, lpBase, lpResBase,
                                        modAmp, modPan);
            }
            if (lfoB.activeDepth > 0.0f) {
                VoiceLfoBank::applySlot(lfoB, baseHz, modPw, hpBase, hpResBase, lpBase, lpResBase,
                                        modAmp, modPan);
            }

            const float pitchMul =
                (p > 0.0f) ? (computeVoicePitchRatio(p, pitchMode) * kPitchMul) : kPitchMul;
            oscFreqs[i] = (pitchMul != 1.0f)
                              ? clampf(baseHz * pitchMul, 0.005f, kAudioFreqMaxHz * 4.0f)
                              : baseHz;
            oscPulseWidths[i] = modPw;

            ModulatedFilterParams mod = computeVoicePressureMacro(
                hpBase, hpResBase, lpBase, lpResBase, p, baseGate, pitchMode);
            if (anyFilterMacro) {
                const FilterMacroContext fltMacros{
                    dubDrop,
                    surge,
                    filterScream,
                    rhythmFrame.pluckDepth,
                    rhythmFrame.pluckEnv01[i],
                    hpfSweep,
                };
                mod = applyFilterPerformanceMacros(mod, fltMacros);
            }

            if (surgeDrive > 0.0f) {
                modAmp = clampf(modAmp + (1.0f - modAmp) * surgeDrive, 0.0f, 1.0f);
            }
            const VoiceChannelGain chGain = computeVoiceChannelGain(modAmp);

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
            const bool coeffsDirty =
                refreshCoeffs &&
                (cache.hpHz < 0.0f || absf(mod.hpCutoffHz - cache.hpHz) > cache.hpHz * 3.0e-4f ||
                 absf(mod.hpResonance - cache.hpRes) > 1.5e-4f ||
                 absf(mod.lpCutoffHz - cache.lpHz) > cache.lpHz * 3.0e-4f ||
                 absf(mod.lpResonance - cache.lpRes) > 1.5e-4f ||
                 absf(chGain.preDrive - cache.preDrive) > 5.0e-4f);
#else
            const bool coeffsDirty =
                refreshCoeffs && (mod.hpCutoffHz != cache.hpHz || mod.hpResonance != cache.hpRes ||
                                  mod.lpCutoffHz != cache.lpHz || mod.lpResonance != cache.lpRes ||
                                  chGain.preDrive != cache.preDrive);
#endif
            if (coeffsDirty) {
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
            voiceScopeScale[i] = ampGate * runScale * 0.25f;
            if (spatialFocus > 0.0f) {
                modPan += (kWideFanPans[i] - modPan) * spatialFocus;
            }
            if (rhythmFrame.swingPanOffset != 0.0f) {
                modPan = clampf(modPan + rhythmFrame.swingPanOffset, -1.0f, 1.0f);
            }
            const float panEff =
                (panScale == 1.0f) ? modPan : clampf(modPan * panScale, -1.0f, 1.0f);
            if (panEff != cache.panEff) {
                cache.panEff = panEff;
                equalPowerPan(panEff, cache.panL, cache.panR);
            }
            gainL[i] = voiceGain * cache.panL;
            gainR[i] = voiceGain * cache.panR;
            spanCoeffs_[curBuf][i] = filterCoeffs_[i];
        }

        if (!prevStatesValid_) {
            prevOscFreqHz_ = oscFreqs;
            prevPulseWidth_ = oscPulseWidths;
            prevGainL_ = gainL;
            prevGainR_ = gainR;
            prevScopeScale_ = voiceScopeScale;
            prevRunScale_ = runScale;
            prevMasterTrim_ = masterTrim;
            prevExactMono_ = exactMono;
            prevStatesValid_ = true;
        }

        const auto max4 = [](const std::array<float, kNumOscillators>& a) noexcept -> float {
            const float m01 = (a[0] > a[1]) ? a[0] : a[1];
            const float m23 = (a[2] > a[3]) ? a[2] : a[3];
            return (m01 > m23) ? m01 : m23;
        };
        const bool anyXmod = max4(xmodDepths) > 1.0e-4f;
        const bool anyFold = max4(foldDepths) > 1.0e-4f;
        const bool anyCrush = max4(crushDepths) > 1.0e-4f;
        const bool anyRing = (ringModMix > 1.0e-4f);
        const bool useFastOsc4x = !anyXmod && !anyFold && !anyCrush && !anyRing;
        const bool chunkExactMono = prevExactMono_ && exactMono;
        const float invChunk =
            (chunk == 32u)
                ? 0.03125f
                : ((chunk == 16u) ? 0.0625f
                                  : ((chunk == 8u) ? 0.125f : (1.0f / static_cast<float>(chunk))));

        auto& curSpanSub = spanSub_[curBuf];
        std::array<uint8_t, kMaxFilterSpanFrames> spanRising{};

        // 1. Synthesize `curBuf` oscillators while Core 0 is still filtering `inFlightSpan_`!
        if (useFastOsc4x) {
            std::array<uint8_t, kNumOscillators> risingCounts;
            oscBank_.processSpan4x(curSpanSub.data(), spanRising.data(), risingCounts, chunk,
                                   prevOscFreqHz_, oscFreqs, prevPulseWidth_, oscPulseWidths,
                                   oscDrifts, coupling, effFreeze, hardSync, prevOscSamples_,
                                   prevCapRamp_, isLastChunk ? &lastGateHigh : nullptr,
                                   isLastChunk ? &lastDriftNorm : nullptr);
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                crushStates_[v].divCounter =
                    static_cast<uint8_t>((crushStates_[v].divCounter + risingCounts[v]) & 7u);
            }
        } else {
            const PatchedSpanContext patchCtx{
                chunk,   invChunk,  isLastChunk, anyXmod,  anyFold,    anyCrush,
                anyRing, effFreeze, coupling,    hardSync, ringModMix,
            };
            renderPatchedOscSpan4x(patchCtx, oscBank_, curSpanSub.data(), spanRising.data(),
                                   prevOscFreqHz_, oscFreqs, prevPulseWidth_, oscPulseWidths,
                                   oscDrifts, xmodDepths, foldDepths, crushDepths, prevOscSamples_,
                                   prevCapRamp_, crushStates_, lastGateHigh, lastDriftNorm);
        }
        if (stoppedSilent) {
            spanRising.fill(0u);
        }

        // 2. Join previous in-flight filters, immediately launch `curBuf` onto Core 0, then mix &
        //    decimate previous span in parallel while Core 0 filters `curBuf`!
        const bool hadPrevSpan = hasInFlightSpan_;
        const size_t prevMetaIdx = inFlightMetaIdx_ & 1u;
        if (hadPrevSpan) {
            finishInFlightFilterStage(spanMeta_[prevMetaIdx]);
        }

        bool curCoProcHandled = false;
        launchSpanFilters(curBuf, chunk, prevGainL_, gainL, prevGainR_, gainR, prevScopeScale_,
                          voiceScopeScale, !useFastOsc4x || anyCrush, curCoProcHandled);

        if (hadPrevSpan) {
            mixAndFlushCompletedSpan(spanMeta_[prevMetaIdx], buses, outLeft, outRight,
                                     blockVoicePeak, blockPeakL, blockPeakR);
            spanMeta_[prevMetaIdx].valid = false;
        }

        InFlightFilterSpan& dstSpan = spanMeta_[curBuf];
        dstSpan.startGainL = prevGainL_;
        dstSpan.endGainL = gainL;
        dstSpan.startGainR = prevGainR_;
        dstSpan.endGainR = gainR;
        dstSpan.startScope = prevScopeScale_;
        dstSpan.endScope = voiceScopeScale;
        dstSpan.vcaGates = vcaGates;
        dstSpan.crushDepths = crushDepths;
        dstSpan.rising = spanRising;
        dstSpan.bufIdx = curBuf;
        dstSpan.baseFrame = baseFrame;
        dstSpan.chunk = chunk;
        dstSpan.invChunk = invChunk;
        dstSpan.startRun = prevRunScale_;
        dstSpan.endRun = runScale;
        dstSpan.startTrim = prevMasterTrim_;
        dstSpan.endTrim = masterTrim;
        dstSpan.chunkExactMono = chunkExactMono;
        dstSpan.anyCrush = anyCrush;
        dstSpan.coProcHandled = curCoProcHandled;
        dstSpan.valid = true;
        inFlightMetaIdx_ = curBuf;
        hasInFlightSpan_ = true;

        prevOscFreqHz_ = oscFreqs;
        prevPulseWidth_ = oscPulseWidths;
        prevGainL_ = gainL;
        prevGainR_ = gainR;
        prevScopeScale_ = voiceScopeScale;
        prevRunScale_ = runScale;
        prevMasterTrim_ = masterTrim;
        prevExactMono_ = exactMono;
        baseFrame += chunk;
    }
    if (hasInFlightSpan_) {
        InFlightFilterSpan& tailSpan = spanMeta_[inFlightMetaIdx_ & 1u];
        finishInFlightFilterStage(tailSpan);
        mixAndFlushCompletedSpan(tailSpan, buses, outLeft, outRight, blockVoicePeak, blockPeakL,
                                 blockPeakR);
        tailSpan.valid = false;
        hasInFlightSpan_ = false;
    }

    const bool blockStopped = !latestState.global.masterRun && (prevRunScale_ <= 1.0e-4f);
    buses.telemetry.peakLeft.store(blockPeakL, std::memory_order_relaxed);
    buses.telemetry.peakRight.store(blockPeakR, std::memory_order_relaxed);
    for (size_t i = 0u; i < kNumOscillators; ++i) {
        buses.telemetry.oscGateHigh[i].store(blockStopped ? 0.0f : lastGateHigh[i],
                                             std::memory_order_relaxed);
        buses.telemetry.voicePeak[i].store(blockStopped ? 0.0f : blockVoicePeak[i],
                                           std::memory_order_relaxed);
        buses.telemetry.voiceDriftNorm[i].store(blockStopped ? 0.0f : lastDriftNorm[i],
                                                std::memory_order_relaxed);
    }
    buses.telemetry.stutterGateMask.store(blockStopped ? 0x0Fu : lastStutterMask,
                                          std::memory_order_relaxed);
    buses.telemetry.rhythmPhase.store(rhythmState_.warpedBar01, std::memory_order_relaxed);
    for (size_t s = 0u; s < kNumVoiceLfos; ++s) {
        buses.telemetry.lfoOutput[s].store(blockStopped ? 0.0f : lfoFrames[s].raw,
                                           std::memory_order_relaxed);
        buses.telemetry.lfoPhase[s].store(lfoFrames[s].phase01, std::memory_order_relaxed);
        buses.telemetry.lfoDriftPacked[s].store(lfoFrames[s].driftPacked,
                                                std::memory_order_relaxed);
    }
    hwCacheFlushInvalidateRange(&buses.telemetry, sizeof(buses.telemetry));
}

} // namespace ruido
