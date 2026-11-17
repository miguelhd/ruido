#include "ui/ScopeHudRenderer.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "ui/GraphStyle.h"
#include "ui/HudChrome.h"
#include "ui/HudHeader.h"
#include "ui/MonoCanvas.h"

namespace ruido {

ScopeHudRenderer::ScopeHudRenderer() noexcept {
    reset();
}

void ScopeHudRenderer::reset() noexcept {
    bootIntro_.reset();
    traceEngine_.reset();
    graphs_.reset();
    ampGraph_.reset();
    globalViews_.reset();
    keybedView_.reset();
    sceneView_.reset();

    frameControl_ = EngineControlState::makeDefault();
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        prevVoiceFreqHz_[v] = frameControl_.voices[v].frequencyHz;
    }
    frameHud_ = {};
    prevRawHud_ = {};
    releaseLatchHud_ = {};
    preparedHeader_ = {};
    preparedChrome_ = {};
    frameLfoOutput_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    frameLfoPhase_ = {0.0f, 0.25f, 0.50f, 0.75f, 0.50f, 0.75f, 0.125f, 0.375f};
    frameLfoDriftPacked_ = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    frameVoicePeak_ = {0.0f, 0.0f, 0.0f, 0.0f};
    frameVoiceDrift_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobPosSignedSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobDimSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobFocusSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    sceneDotsSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    headerBarSmooth_ = 0.0f;
    fineOverlayPhase_ = 0.0f;
    coarseOverlayPhase_ = 0.0f;
    sceneDialPhaseSmooth_ = 0.0f;
    renderedOpacityLevel_ = 0.0f;
    smoothedPeakL_ = 0.0f;
    smoothedPeakR_ = 0.0f;
    syncRecoilEnv_ = 0.0f;
    syncRecoilPhase_ = 0.0f;
    prevHeaderBarMode_ = HudMode::None;
    prevKnobBank_ = KnobBankMode::Frequency;
    frameRhythmPhase_ = 0.0f;
    frameBgKeepQ8_ = 256u;
    frameStutterGateMask_ = 0x0Fu;
    lastPhantomSeq_ = 0u;
    phantomPin_ = 0u;
    wasShowingSceneDots_ = false;
    releaseLatchActive_ = false;
    hudGaugesPrimed_ = false;
}

void ScopeHudRenderer::ingestScopeRing(ScopeRingBuffer& ring) noexcept {
    traceEngine_.ingestScopeRing(ring, frameHud_.scopeFrozen);
}

void ScopeHudRenderer::pushScopeSamples(const ScopeFrame* frames, size_t count) noexcept {
    traceEngine_.pushScopeSamples(frames, count, frameHud_.scopeFrozen);
}

uint32_t ScopeHudRenderer::computeFrameBgKeepQ8() const noexcept {
    const float scopeVis = bootIntro_.scopeVisibility01();
    const float bgDim01 = smoothstep01(clampf(graphs_.backdropDim(), 0.0f, 1.0f));
    const float hudAlpha01 = clampf(frameHud_.opacity, 0.0f, 1.0f);
    float cfgBackdrop01 = 0.0f;
    float shiftBackdrop01 = 0.0f;
    float homeDim01 = 0.0f;
    (void)resolveSubViewAlpha(globalViews_.configPhase(), hudAlpha01, cfgBackdrop01);
    (void)resolveSubViewAlpha(keybedView_.shiftPhase(), hudAlpha01, shiftBackdrop01);
    (void)resolveSubViewAlpha(ampGraph_.translucentPhase(), hudAlpha01, homeDim01);
    const float keep01 = (1.0f - kFilterBgDimDepth * bgDim01) * (1.0f - cfgBackdrop01) *
                         (1.0f - shiftBackdrop01) * (1.0f - kHomeSubBgDimDepth * homeDim01) *
                         scopeVis;
    return static_cast<uint32_t>(keep01 * 256.0f + 0.5f);
}

void ScopeHudRenderer::updateTraceForFrame(float dtSec, const EngineControlState& control,
                                           const HudState& hud,
                                           void (*coProcYield)() noexcept) noexcept {
    frameBgKeepQ8_ = computeFrameBgKeepQ8();
    if (frameBgKeepQ8_ > 0u) {
        traceEngine_.buildWaveformTrace(dtSec, control, hud.scopeFrozen, frameStutterGateMask_,
                                        hud.scopeTimebase, false, coProcYield);
    } else {
        traceEngine_.notifyScopeHidden(hud.scopeFrozen);
        if (HudAmpGraph::isLevelViewActive(hud) || ampGraph_.levelPhase() > 0.002f) {
            traceEngine_.refreshOverdriveHair(control, hud.scopeFrozen);
        }
    }
}

void ScopeHudRenderer::updateHudGauges(float dtClamped, bool snapImmediate) noexcept {
    const float alphaGauge =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.022f)));
    const float alphaChop = snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.004f)));
    const HudGaugeTargets tgt =
        computeHudGaugeTargets(frameControl_, frameHud_, frameLfoOutput_, frameStutterGateMask_);

    if (tgt.hasHeaderBar) {
        const bool snapHeader = snapImmediate || (prevHeaderBarMode_ != frameHud_.mode);
        headerBarSmooth_ = snapHeader ? tgt.headerBarNorm
                                      : slewScalar(headerBarSmooth_, tgt.headerBarNorm, alphaGauge);
        prevHeaderBarMode_ = frameHud_.mode;
    } else {
        prevHeaderBarMode_ = HudMode::None;
    }

    const bool snapBank = snapImmediate || (prevKnobBank_ != frameHud_.knobBank);
    prevKnobBank_ = frameHud_.knobBank;
    const bool perOscBank = isPerOscKnobBank(frameHud_.knobBank);
    for (uint8_t k = 0u; k < 4u; ++k) {
        if (snapBank) {
            knobPosSignedSmooth_[k] = tgt.posSigned[k];
            knobDimSmooth_[k] = tgt.dimAmount[k];
            knobFocusSmooth_[k] = tgt.focused[k] ? 1.0f : 0.0f;
        } else {
            const bool oscChop = perOscBank && (frameControl_.keyMod.stutterDepth[k] > 1.0e-4f);
            const bool posChop = oscChop && frameHud_.knobBank == KnobBankMode::Amplitude;
            knobPosSignedSmooth_[k] = slewScalar(knobPosSignedSmooth_[k], tgt.posSigned[k],
                                                 posChop ? alphaChop : alphaGauge);
            knobDimSmooth_[k] =
                slewScalar(knobDimSmooth_[k], tgt.dimAmount[k], oscChop ? alphaChop : alphaGauge);
            knobFocusSmooth_[k] =
                slewScalar(knobFocusSmooth_[k], tgt.focused[k] ? 1.0f : 0.0f, alphaGauge);
        }
    }
}

void ScopeHudRenderer::prepareFrame(float dtSec, const EngineControlState& control,
                                    const HudState& hud, const EngineTelemetry& telemetry,
                                    void (*coProcYield)() noexcept) noexcept {
    frameControl_ = control;
    frameHud_.scopeFrozen = hud.scopeFrozen;

    const float dtClamped = clampf(dtSec, 0.001f, 0.250f);
    if (bootIntro_.isActive()) {
        const bool hudActive = (hud.mode != HudMode::None) || (hud.opacity > 0.001f);
        bootIntro_.step(dtClamped, hudActive);
    }
    traceEngine_.advanceWatchdog(dtClamped, hud.scopeFrozen);
    const bool streamStalled =
        traceEngine_.isStreamStalled(hud.scopeFrozen) || !control.global.masterRun;
    if (!control.global.masterRun) {
        frameStutterGateMask_ = 0x0Fu;
        frameLfoOutput_.fill(0.0f);
        frameVoicePeak_.fill(0.0f);
        frameVoiceDrift_.fill(0.0f);
    } else if (!hud.scopeFrozen) {
        frameStutterGateMask_ = telemetry.stutterGateMask.load(std::memory_order_relaxed);
        frameRhythmPhase_ = telemetry.rhythmPhase.load(std::memory_order_relaxed);
        for (size_t i = 0u; i < kNumVoiceLfos; ++i) {
            frameLfoOutput_[i] = telemetry.lfoOutput[i].load(std::memory_order_relaxed);
            frameLfoPhase_[i] =
                wrapLfoPhase01(telemetry.lfoPhase[i].load(std::memory_order_relaxed));
            frameLfoDriftPacked_[i] = telemetry.lfoDriftPacked[i].load(std::memory_order_relaxed);
        }
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            frameVoicePeak_[v] =
                streamStalled ? 0.0f : telemetry.voicePeak[v].load(std::memory_order_relaxed);
            frameVoiceDrift_[v] =
                streamStalled ? 0.0f : telemetry.voiceDriftNorm[v].load(std::memory_order_relaxed);
        }
    }
    const float pL = streamStalled ? 0.0f : telemetry.peakLeft.load(std::memory_order_relaxed);
    const float pR = streamStalled ? 0.0f : telemetry.peakRight.load(std::memory_order_relaxed);
    const float alphaPeakAtk = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.028f));
    const float alphaPeakRel = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.155f));
    smoothedPeakL_ =
        slewScalar(smoothedPeakL_, pL, (pL >= smoothedPeakL_) ? alphaPeakAtk : alphaPeakRel);
    smoothedPeakR_ =
        slewScalar(smoothedPeakR_, pR, (pR >= smoothedPeakR_) ? alphaPeakAtk : alphaPeakRel);

    if (hud.phantomPinSeq != lastPhantomSeq_) {
        lastPhantomSeq_ = hud.phantomPinSeq;
        phantomPin_ = static_cast<uint8_t>(hud.phantomPinIndex & 3u);
        syncRecoilEnv_ = 1.0f;
        syncRecoilPhase_ = 0.0f;
    } else if (syncRecoilEnv_ > 0.0f) {
        syncRecoilEnv_ *= fastExpNeg(dtClamped * (1.0f / 0.056f));
        if (syncRecoilEnv_ < 0.004f) {
            syncRecoilEnv_ = 0.0f;
            syncRecoilPhase_ = 0.0f;
        } else {
            syncRecoilPhase_ = wrapLfoPhase01(syncRecoilPhase_ + dtClamped * 24.0f);
        }
    }

    bool freqChanged = false;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (absf(control.voices[v].frequencyHz - prevVoiceFreqHz_[v]) > 1.0e-4f) {
            freqChanged = true;
        }
        prevVoiceFreqHz_[v] = control.voices[v].frequencyHz;
    }

    if (hud.mode == HudMode::None && hud.opacity <= 0.001f) {
        const bool syntheticInstantHide = (prevRawHud_.opacity > 0.25f);
        releaseLatchActive_ = false;
        prevRawHud_ = hud;
        if (dtSec <= 0.0f || syntheticInstantHide ||
            (renderedOpacityLevel_ <= 0.002f && !graphs_.hasResidualPhase() &&
             !ampGraph_.hasResidualPhase() && !globalViews_.hasResidualPhase() &&
             !keybedView_.hasResidualPhase() && !sceneView_.hasResidualPhase())) {
            frameHud_ = hud;
            renderedOpacityLevel_ = 0.0f;
            fineOverlayPhase_ = 0.0f;
            coarseOverlayPhase_ = 0.0f;
            wasShowingSceneDots_ = false;
            graphs_.clearImmediate();
            ampGraph_.clearImmediate();
            globalViews_.clearImmediate();
            keybedView_.clearImmediate();
            sceneView_.clearImmediate();
            hudGaugesPrimed_ = false;
            updateTraceForFrame(dtSec, control, hud, coProcYield);
            return;
        }

        renderedOpacityLevel_ = stepHudFadeLinear(renderedOpacityLevel_, false, dtClamped);
        graphs_.stepFadeOut(dtClamped);
        ampGraph_.stepFadeOut(dtClamped);
        globalViews_.update(dtClamped, control, hud, frameLfoOutput_, frameVoicePeak_,
                            frameVoiceDrift_, frameStutterGateMask_, traceEngine_);
        keybedView_.update(dtClamped, control, hud, frameVoicePeak_, frameStutterGateMask_);
        sceneView_.update(dtClamped, hud);
        frameHud_.opacity = smoothstep01(renderedOpacityLevel_);
        if (renderedOpacityLevel_ <= 0.001f && !graphs_.hasResidualPhase() &&
            !ampGraph_.hasResidualPhase() && !globalViews_.hasResidualPhase() &&
            !keybedView_.hasResidualPhase() && !sceneView_.hasResidualPhase()) {
            renderedOpacityLevel_ = 0.0f;
            fineOverlayPhase_ = 0.0f;
            coarseOverlayPhase_ = 0.0f;
            wasShowingSceneDots_ = false;
            frameHud_.mode = HudMode::None;
            frameHud_.opacity = 0.0f;
            hudGaugesPrimed_ = false;
        } else if (frameHud_.opacity > 0.001f && frameHud_.mode != HudMode::None) {
            prepareHudHeader(preparedHeader_, frameControl_, frameHud_, frameLfoOutput_,
                             frameStutterGateMask_, fineOverlayPhase_, coarseOverlayPhase_);
            preparedHeader_.sceneDotWeights = sceneDotsSmooth_;
            preparedHeader_.sceneDialPhase = sceneDialPhaseSmooth_;
            prepareHudChrome(preparedChrome_, frameControl_, frameHud_, frameLfoOutput_,
                             frameStutterGateMask_);
        }
        updateTraceForFrame(dtSec, control, hud, coProcYield);
        return;
    }

    const bool nowKeyHeld = (hud.heldBlackMask != 0u) || (hud.heldWhiteMask != 0u);
    const bool prevKeyHeld = (prevRawHud_.heldBlackMask != 0u) || (prevRawHud_.heldWhiteMask != 0u);
    const bool nowPinnedOrOpen =
        hud.shiftHeld || hud.modifierHeld || nowKeyHeld || hud.freqPageOpen || hud.fxPageOpen ||
        hud.panPageOpen || hud.lfoPageOpen || hud.editPageOpen || hud.configPageOpen ||
        hud.selPageOpen || hud.saveArmed || (hud.scopeFrozen && hud.mode == HudMode::ScopeFreeze);
    const bool prevWasPinnedOrOpen =
        prevRawHud_.shiftHeld || prevRawHud_.modifierHeld || prevKeyHeld ||
        prevRawHud_.freqPageOpen || prevRawHud_.fxPageOpen || prevRawHud_.panPageOpen ||
        prevRawHud_.lfoPageOpen || prevRawHud_.editPageOpen || prevRawHud_.configPageOpen ||
        prevRawHud_.selPageOpen || prevRawHud_.saveArmed ||
        (prevRawHud_.scopeFrozen && prevRawHud_.mode == HudMode::ScopeFreeze);
    const bool fadingSameMode = !nowPinnedOrOpen && !freqChanged && (hud.opacity > 0.001f) &&
                                ((prevWasPinnedOrOpen && hud.mode == prevRawHud_.mode &&
                                  hud.opacity <= prevRawHud_.opacity + 0.002f) ||
                                 (releaseLatchActive_ && hud.mode == releaseLatchHud_.mode &&
                                  hud.opacity <= prevRawHud_.opacity + 0.002f));

    if (fadingSameMode) {
        if (!releaseLatchActive_) {
            releaseLatchActive_ = true;
            releaseLatchHud_ = prevRawHud_;
        }
    } else {
        releaseLatchActive_ = false;
    }

    HudState effectiveHud = hud;
    if (releaseLatchActive_) {
        effectiveHud = releaseLatchHud_;
        effectiveHud.opacity = hud.opacity;
        effectiveHud.scopeFrozen = hud.scopeFrozen;
        effectiveHud.savePulseQ8 = hud.savePulseQ8;
        effectiveHud.shiftBlinkQ8 = hud.shiftBlinkQ8;
        effectiveHud.modeWipeQ8 = hud.modeWipeQ8;
        effectiveHud.phantomPinSeq = hud.phantomPinSeq;
        effectiveHud.phantomPinIndex = hud.phantomPinIndex;
        if (releaseLatchHud_.mode != HudMode::KnobRate) {
            effectiveHud.rateHoldActive = hud.rateHoldActive;
            effectiveHud.knobRate = hud.knobRate;
            effectiveHud.modifierHeld = hud.modifierHeld;
        }
    }

    frameHud_ = effectiveHud;
    renderedOpacityLevel_ = clampf(effectiveHud.opacity, 0.0f, 1.0f);
    prevRawHud_ = hud;

    const bool snapGauges = !hudGaugesPrimed_;
    const bool wantFine = effectiveHud.rateHoldActive && (effectiveHud.knobRate == KnobRate::Fine);
    const bool wantCoarse =
        effectiveHud.rateHoldActive && (effectiveHud.knobRate == KnobRate::Coarse);
    if (snapGauges) {
        fineOverlayPhase_ = wantFine ? 1.0f : 0.0f;
        coarseOverlayPhase_ = wantCoarse ? 1.0f : 0.0f;
    } else {
        fineOverlayPhase_ = stepHudFadeLinear(fineOverlayPhase_, wantFine, dtClamped);
        coarseOverlayPhase_ = stepHudFadeLinear(coarseOverlayPhase_, wantCoarse, dtClamped);
    }

    hudGaugesPrimed_ = (effectiveHud.opacity > 0.004f);
    if (hudGaugesPrimed_) {
        updateHudGauges(dtClamped, snapGauges);
    }
    if (frameHud_.opacity > 0.001f && frameHud_.mode != HudMode::None) {
        prepareHudHeader(preparedHeader_, frameControl_, frameHud_, frameLfoOutput_,
                         frameStutterGateMask_, fineOverlayPhase_, coarseOverlayPhase_);
        if (preparedHeader_.showSceneDots) {
            const bool snapScene = snapGauges || !wasShowingSceneDots_;
            wasShowingSceneDots_ = true;
            const bool liveMorph = isSceneLiveMorphActive(frameHud_);
            const float dotAlpha =
                snapScene ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.036f)));
            const float tgtDial =
                liveMorph
                    ? wrapCircularPhase(frameHud_.snapshotLoopPhase, 4.0f)
                    : static_cast<float>((frameHud_.activePunch == KeyPunchKind::SnapshotRecall)
                                             ? (frameHud_.activePunchVoice & 3u)
                                             : (frameHud_.snapshotSlot & 3u));
            if (snapScene) {
                sceneDialPhaseSmooth_ = tgtDial;
            } else {
                float diff = tgtDial - sceneDialPhaseSmooth_;
                if (diff > 2.0f) {
                    diff -= 4.0f;
                } else if (diff < -2.0f) {
                    diff += 4.0f;
                }
                sceneDialPhaseSmooth_ =
                    (absf(diff) <= 0.001f)
                        ? tgtDial
                        : wrapCircularPhase(sceneDialPhaseSmooth_ + diff * dotAlpha, 4.0f);
            }
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                const bool occ = (frameHud_.snapshotOccupiedMask & (1u << s)) != 0u;
                const float tgtW =
                    !occ ? 0.0f
                         : (liveMorph ? clampf(frameHud_.snapshotWeights[s], 0.0f, 1.0f)
                                      : (((frameHud_.snapshotSlot & 3u) == s) ? 1.0f : 0.0f));
                sceneDotsSmooth_[s] = slewScalar(sceneDotsSmooth_[s], tgtW, dotAlpha, 0.001f);
            }
            preparedHeader_.sceneDotWeights = sceneDotsSmooth_;
            preparedHeader_.sceneDialPhase = sceneDialPhaseSmooth_;
        } else {
            wasShowingSceneDots_ = false;
        }
        prepareHudChrome(preparedChrome_, frameControl_, frameHud_, frameLfoOutput_,
                         frameStutterGateMask_);
    }

    globalViews_.update(dtClamped, control, hud, frameLfoOutput_, frameVoicePeak_, frameVoiceDrift_,
                        frameStutterGateMask_, traceEngine_);
    keybedView_.update(dtClamped, control, hud, frameVoicePeak_, frameStutterGateMask_,
                       frameRhythmPhase_);
    sceneView_.update(dtClamped, hud);
    ampGraph_.update(dtClamped, control, hud, frameLfoOutput_, frameVoiceDrift_,
                     frameStutterGateMask_);
    const bool extraCenterView = globalViews_.anyActiveTarget(hud) ||
                                 HudAmpGraph::isLevelViewActive(hud) || keybedView_.isActive(hud) ||
                                 keybedView_.isShiftActive(hud) || sceneView_.isSaveActive(hud) ||
                                 sceneView_.isPresetSceneActive(hud);
    HudState graphsRawHud = hud;
    if (HudGlobalViews::isRailCouplingActive(hud)) {
        graphsRawHud.knobBank = KnobBankMode::None;
        graphsRawHud.lfoPageOpen = false;
    }
    graphs_.update(dtClamped, control, graphsRawHud, effectiveHud, extraCenterView, frameLfoOutput_,
                   frameLfoDriftPacked_, frameRhythmPhase_, frameStutterGateMask_);
    if (coProcYield != nullptr) {
        coProcYield();
    }
    updateTraceForFrame(dtSec, control, hud, coProcYield);
}

int ScopeHudRenderer::computeScanlineSyncShift(int py) const noexcept {
    if (syncRecoilEnv_ <= 0.0f) {
        return 0;
    }
    const float y01 = static_cast<float>(py) * (1.0f / 239.0f);
    const float pinF = static_cast<float>(phantomPin_ & 3u);
    const float spring = fastSinPhase01(wrapLfoPhase01(syncRecoilPhase_ + 0.25f));
    const float shear = fastSinPhase01(
        wrapLfoPhase01(y01 * (1.15f + 0.35f * pinF) + syncRecoilPhase_ * 1.4f + 0.14f * pinF));
    const float tear =
        fastSinPhase01(wrapLfoPhase01(y01 * 5.2f - syncRecoilPhase_ * 2.3f + 0.29f * pinF));
    const float dir = ((phantomPin_ & 1u) != 0u) ? -1.0f : 1.0f;
    const float shiftF = dir * syncRecoilEnv_ * (9.5f * spring + 2.8f * shear + 1.2f * tear);
    return static_cast<int>(shiftF + ((shiftF >= 0.0f) ? 0.5f : -0.5f));
}

void ScopeHudRenderer::renderStrip(uint16_t yStart, uint16_t stripHeight, uint16_t* rgb565Out,
                                   bool bigEndianRgb565,
                                   void (*coProcYield)() noexcept) const noexcept {
    if (rgb565Out == nullptr || yStart >= kDisplayHeight || stripHeight == 0u) {
        return;
    }

    const uint16_t totalEnd = (yStart + stripHeight > kDisplayHeight)
                                  ? kDisplayHeight
                                  : static_cast<uint16_t>(yStart + stripHeight);

    const float scopeVis = bootIntro_.scopeVisibility01();
    const float hudAlpha01 = clampf(frameHud_.opacity, 0.0f, 1.0f);
    const uint32_t bgKeepQ8 = (scopeVis <= 0.001f) ? 0u : frameBgKeepQ8_;
    const uint8_t hudAlpha8 = static_cast<uint8_t>(hudAlpha01 * 255.0f + 0.5f);

    auto& linBuf = stripLinScratch_;
    auto& lumBuf = stripLumScratch_;
    uint16_t curY = yStart;

    while (curY < totalEnd) {
        const uint16_t chunkH = (totalEnd - curY > kSubStripRows)
                                    ? kSubStripRows
                                    : static_cast<uint16_t>(totalEnd - curY);
        const int y0 = static_cast<int>(curY);
        const int y1 = y0 + static_cast<int>(chunkH);

        if (scopeVis <= 0.001f) {
            bootIntro_.compositeStripChunk(y0, chunkH, lumBuf.data(), false);
        } else {
            traceEngine_.renderBackgroundChunk(y0, chunkH, bgKeepQ8, linBuf.data(), lumBuf.data());

            if (hudAlpha8 < 255u && y0 < 18 && y1 > 4) {
                const uint8_t idleAlpha8 =
                    static_cast<uint8_t>(static_cast<float>(255u - hudAlpha8) * scopeVis + 0.5f);
                if (idleAlpha8 > 0u) {
                    const MonoStripCanvas idleCv{lumBuf.data(), y0, y1, idleAlpha8};
                    compositeIdlePeakMeters(idleCv, smoothedPeakL_, smoothedPeakR_);
                }
            }
            if (bootIntro_.isActive()) {
                bootIntro_.compositeStripChunk(y0, chunkH, lumBuf.data(), true);
            }
        }

        if (coProcYield != nullptr) {
            coProcYield();
        }
        if (hudAlpha8 > 0u && frameHud_.mode != HudMode::None) {
            compositeHudStrip(curY, chunkH, lumBuf.data(), linBuf.data(), hudAlpha8);
            if (coProcYield != nullptr) {
                coProcYield();
            }
        }

        uint16_t* dstBase = rgb565Out + static_cast<size_t>(curY - yStart) * kDisplayWidth;
        const bool recoiling = (syncRecoilEnv_ > 0.0f);
        const uint32_t surgeQ8 =
            recoiling ? static_cast<uint32_t>(256.0f + 48.0f * syncRecoilEnv_ + 0.5f) : 256u;
        for (uint16_t r = 0u; r < chunkH; ++r) {
            const int py = y0 + static_cast<int>(r);
            const uint8_t* srcRow = lumBuf.data() + static_cast<size_t>(r) * kDisplayWidth;
            uint16_t* dstRow = dstBase + static_cast<size_t>(r) * kDisplayWidth;
            if (!recoiling) {
                for (size_t x = 0u; x < kDisplayWidth; ++x) {
                    const uint16_t px =
                        packNeutralGrayRgb565<false>(srcRow[x], static_cast<int>(x), py);
                    dstRow[x] =
                        bigEndianRgb565 ? static_cast<uint16_t>((px << 8u) | (px >> 8u)) : px;
                }
            } else {
                const int shiftPx = computeScanlineSyncShift(py);
                for (size_t x = 0u; x < kDisplayWidth; ++x) {
                    const int sx = static_cast<int>(x) - shiftPx;
                    if (sx < 0 || sx >= static_cast<int>(kDisplayWidth)) {
                        dstRow[x] = 0x0000u;
                    } else {
                        const uint32_t boosted =
                            (static_cast<uint32_t>(srcRow[static_cast<size_t>(sx)]) * surgeQ8) >>
                            8u;
                        const uint8_t lum = static_cast<uint8_t>((boosted > 255u) ? 255u : boosted);
                        const uint16_t px =
                            packNeutralGrayRgb565<false>(lum, static_cast<int>(x), py);
                        dstRow[x] =
                            bigEndianRgb565 ? static_cast<uint16_t>((px << 8u) | (px >> 8u)) : px;
                    }
                }
            }
        }

        curY = static_cast<uint16_t>(curY + chunkH);
    }
}

void ScopeHudRenderer::compositeHudStrip(uint16_t yStart, uint16_t stripHeight, uint8_t* lumBuf,
                                         uint16_t* linScratch, uint8_t hudAlpha8) const noexcept {
    const int y0 = static_cast<int>(yStart);
    const int y1 = y0 + static_cast<int>(stripHeight);
    const MonoStripCanvas cv{lumBuf, y0, y1, hudAlpha8};

    const float hudAlpha01 = static_cast<float>(hudAlpha8) * (1.0f / 255.0f);

    float effFilterAlpha01 = 0.0f;
    float effLfoAlpha01 = 0.0f;
    float effEditGraphAlpha01 = 0.0f;
    float effPanAlpha01 = 0.0f;
    float effCouplingAlpha01 = 0.0f;
    float dummy01 = 0.0f;
    const uint8_t filterAlpha8 =
        resolveSubViewAlpha(graphs_.filterGraphPhase(), hudAlpha01, effFilterAlpha01);
    const uint8_t lfoAlpha8 =
        resolveSubViewAlpha(graphs_.lfoGraphPhase(), hudAlpha01, effLfoAlpha01);
    (void)resolveSubViewAlpha(ampGraph_.maxPhase(), hudAlpha01, effEditGraphAlpha01);
    const uint8_t panAlpha8 =
        resolveSubViewAlpha(globalViews_.panPhase(), hudAlpha01, effPanAlpha01);
    const uint8_t couplingAlpha8 =
        resolveSubViewAlpha(globalViews_.couplingPhase(), hudAlpha01, effCouplingAlpha01);
    const uint8_t cfgAlpha8 = resolveSubViewAlpha(globalViews_.configPhase(), hudAlpha01, dummy01);
    const uint8_t keybedAlpha8 = resolveSubViewAlpha(keybedView_.phase(), hudAlpha01, dummy01);
    const uint8_t shiftAlpha8 = resolveSubViewAlpha(keybedView_.shiftPhase(), hudAlpha01, dummy01);
    const uint8_t saveAlpha8 = resolveSubViewAlpha(sceneView_.savePhase(), hudAlpha01, dummy01);
    const uint8_t presetSceneAlpha8 =
        resolveSubViewAlpha(sceneView_.morphPhase(), hudAlpha01, dummy01);
    const bool showBotStrip = (frameHud_.knobBank != KnobBankMode::None) ||
                              HudGlobalViews::isRailCouplingActive(frameHud_);

    if (frameBgKeepQ8_ > 0u || bootIntro_.isActive()) {
        cv.applyVerticalScrim(0, 34, 0.80f, true);
        if (showBotStrip) {
            float graphEnvMax =
                (effFilterAlpha01 > effLfoAlpha01) ? effFilterAlpha01 : effLfoAlpha01;
            if (effEditGraphAlpha01 > graphEnvMax) {
                graphEnvMax = effEditGraphAlpha01;
            }
            if (effPanAlpha01 > graphEnvMax) {
                graphEnvMax = effPanAlpha01;
            }
            if (effCouplingAlpha01 > graphEnvMax) {
                graphEnvMax = effCouplingAlpha01;
            }
            const int botScrimTop = 178 - static_cast<int>(8.0f * graphEnvMax + 0.5f);
            cv.applyVerticalScrim(botScrimTop, 240, 0.90f, false);
        }
    }

    if (lfoAlpha8 > 0u) {
        graphs_.compositeLfoStrip(y0, y1, stripHeight, lumBuf, linScratch, effLfoAlpha01, lfoAlpha8,
                                  frameControl_, frameHud_, frameLfoPhase_);
    }
    if (filterAlpha8 > 0u) {
        graphs_.compositeFilterStrip(y0, y1, stripHeight, lumBuf, linScratch, effFilterAlpha01,
                                     filterAlpha8);
    }
    if (effEditGraphAlpha01 > 0.0f) {
        ampGraph_.compositeStrip(y0, y1, stripHeight, lumBuf, linScratch, hudAlpha01, frameHud_,
                                 traceEngine_.overdriveHair());
    }
    if (panAlpha8 > 0u) {
        globalViews_.compositePanStageStrip(y0, y1, stripHeight, lumBuf, linScratch, effPanAlpha01,
                                            panAlpha8, frameHud_);
    }
    if (couplingAlpha8 > 0u) {
        globalViews_.compositeRailCouplingStrip(y0, y1, stripHeight, lumBuf, linScratch,
                                                effCouplingAlpha01, couplingAlpha8);
    }
    if (cfgAlpha8 > 0u) {
        globalViews_.compositeGlobalConfigStrip(y0, y1, lumBuf, cfgAlpha8, frameControl_,
                                                frameHud_);
    }
    if (keybedAlpha8 > 0u) {
        keybedView_.compositeStrip(y0, y1, lumBuf, keybedAlpha8, frameHud_);
    }
    if (shiftAlpha8 > 0u) {
        keybedView_.compositeShiftStrip(y0, y1, lumBuf, shiftAlpha8, frameControl_, frameHud_);
    }
    if (sceneView_.isSaveActive(frameHud_)) {
        if (presetSceneAlpha8 > 0u) {
            sceneView_.compositePresetSceneStrip(y0, y1, lumBuf, presetSceneAlpha8, frameHud_);
        }
        if (saveAlpha8 > 0u) {
            sceneView_.compositeSaveStrip(y0, y1, lumBuf, saveAlpha8, frameHud_);
        }
    } else {
        if (saveAlpha8 > 0u) {
            sceneView_.compositeSaveStrip(y0, y1, lumBuf, saveAlpha8, frameHud_);
        }
        if (presetSceneAlpha8 > 0u) {
            sceneView_.compositePresetSceneStrip(y0, y1, lumBuf, presetSceneAlpha8, frameHud_);
        }
    }
    if (y0 < 26) {
        compositeTopHeaderStrip(cv, preparedHeader_, frameHud_, headerBarSmooth_, fineOverlayPhase_,
                                coarseOverlayPhase_);
    }
    if (y1 > 188 && showBotStrip) {
        compositeBottomKnobStrip(cv, preparedChrome_, frameControl_, frameHud_,
                                 knobPosSignedSmooth_, knobDimSmooth_, knobFocusSmooth_);
    }
}

} // namespace ruido
