#include "ui/ScopeHudRenderer.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "ui/GraphStyle.h"
#include "ui/HudChrome.h"
#include "ui/MonoCanvas.h"

namespace ruido {

ScopeHudRenderer::ScopeHudRenderer() noexcept {
    reset();
}

void ScopeHudRenderer::reset() noexcept {
    traceEngine_.reset();
    graphs_.reset();
    ampGraph_.reset();
    globalViews_.reset();

    frameControl_ = EngineControlState::makeDefault();
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        prevVoiceFreqHz_[v] = frameControl_.voices[v].frequencyHz;
    }
    frameHud_ = {};
    prevRawHud_ = {};
    releaseLatchHud_ = {};
    frameLfoOutput_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    frameLfoPhase_ = {0.0f, 0.25f, 0.50f, 0.75f, 0.50f, 0.75f, 0.125f, 0.375f};
    frameLfoDriftPacked_ = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    frameVoicePeak_ = {0.0f, 0.0f, 0.0f, 0.0f};
    frameVoiceDrift_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobPosSignedSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobDimSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobFocusSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    headerBarSmooth_ = 0.0f;
    fineOverlayPhase_ = 0.0f;
    coarseOverlayPhase_ = 0.0f;
    renderedOpacityLevel_ = 0.0f;
    smoothedPeakL_ = 0.0f;
    smoothedPeakR_ = 0.0f;
    prevHeaderBarMode_ = HudMode::None;
    prevKnobBank_ = KnobBankMode::Frequency;
    releaseLatchActive_ = false;
    hudGaugesPrimed_ = false;
}

void ScopeHudRenderer::ingestScopeRing(ScopeRingBuffer& ring) noexcept {
    traceEngine_.ingestScopeRing(ring, frameHud_.scopeFrozen);
}

void ScopeHudRenderer::pushScopeSamples(const ScopeFrame* frames, size_t count) noexcept {
    traceEngine_.pushScopeSamples(frames, count, frameHud_.scopeFrozen);
}

void ScopeHudRenderer::updateHudGauges(float dtClamped, bool snapImmediate) noexcept {
    const float alphaGauge =
        snapImmediate ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.045f)));
    const HudGaugeTargets tgt = computeHudGaugeTargets(frameControl_, frameHud_, frameLfoOutput_);

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
    for (uint8_t k = 0u; k < 4u; ++k) {
        if (snapBank) {
            knobPosSignedSmooth_[k] = tgt.posSigned[k];
            knobDimSmooth_[k] = tgt.dimAmount[k];
            knobFocusSmooth_[k] = tgt.focused[k] ? 1.0f : 0.0f;
        } else {
            knobPosSignedSmooth_[k] =
                slewScalar(knobPosSignedSmooth_[k], tgt.posSigned[k], alphaGauge);
            knobDimSmooth_[k] = slewScalar(knobDimSmooth_[k], tgt.dimAmount[k], alphaGauge);
            knobFocusSmooth_[k] =
                slewScalar(knobFocusSmooth_[k], tgt.focused[k] ? 1.0f : 0.0f, alphaGauge);
        }
    }
}

void ScopeHudRenderer::prepareFrame(float dtSec, const EngineControlState& control,
                                    const HudState& hud,
                                    const EngineTelemetry& telemetry) noexcept {
    frameControl_ = control;
    frameHud_.scopeFrozen = hud.scopeFrozen;

    const float dtClamped = clampf(dtSec, 0.001f, 0.250f);
    traceEngine_.advanceWatchdog(dtClamped, hud.scopeFrozen);
    const bool streamStalled = traceEngine_.isStreamStalled(hud.scopeFrozen);
    if (!hud.scopeFrozen) {
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

    traceEngine_.buildWaveformTrace(dtSec, control, hud.scopeFrozen);

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
             !ampGraph_.hasResidualPhase() && !globalViews_.hasResidualPhase())) {
            frameHud_ = hud;
            renderedOpacityLevel_ = 0.0f;
            fineOverlayPhase_ = 0.0f;
            coarseOverlayPhase_ = 0.0f;
            graphs_.clearImmediate();
            ampGraph_.clearImmediate();
            globalViews_.clearImmediate();
            hudGaugesPrimed_ = false;
            return;
        }

        renderedOpacityLevel_ = stepHudFadeLinear(renderedOpacityLevel_, false, dtClamped);
        graphs_.stepFadeOut(dtClamped);
        ampGraph_.stepFadeOut(dtClamped);
        globalViews_.update(dtClamped, control, hud, frameLfoOutput_, frameVoicePeak_);
        frameHud_.opacity = smoothstep01(renderedOpacityLevel_);
        if (renderedOpacityLevel_ <= 0.001f && !graphs_.hasResidualPhase() &&
            !ampGraph_.hasResidualPhase() && !globalViews_.hasResidualPhase()) {
            renderedOpacityLevel_ = 0.0f;
            fineOverlayPhase_ = 0.0f;
            coarseOverlayPhase_ = 0.0f;
            frameHud_.mode = HudMode::None;
            frameHud_.opacity = 0.0f;
            hudGaugesPrimed_ = false;
        }
        return;
    }

    const bool nowPinnedOrOpen = hud.modifierHeld || hud.freqPageOpen || hud.fxPageOpen ||
                                 hud.panPageOpen || hud.lfoPageOpen || hud.editPageOpen ||
                                 hud.configPageOpen || hud.saveArmed ||
                                 (hud.scopeFrozen && hud.mode == HudMode::ScopeFreeze);
    const bool prevWasPinnedOrOpen =
        prevRawHud_.modifierHeld || prevRawHud_.freqPageOpen || prevRawHud_.fxPageOpen ||
        prevRawHud_.panPageOpen || prevRawHud_.lfoPageOpen || prevRawHud_.editPageOpen ||
        prevRawHud_.configPageOpen || prevRawHud_.saveArmed ||
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
        effectiveHud.mode = releaseLatchHud_.mode;
        effectiveHud.knobBank = releaseLatchHud_.knobBank;
        effectiveHud.focusedVoice = releaseLatchHud_.focusedVoice;
        effectiveHud.focusedKnob = releaseLatchHud_.focusedKnob;
        effectiveHud.fxVoiceFocus = releaseLatchHud_.fxVoiceFocus;
        effectiveHud.lfoVoiceFocus = releaseLatchHud_.lfoVoiceFocus;
        effectiveHud.freqPageOpen = releaseLatchHud_.freqPageOpen;
        effectiveHud.fxPageOpen = releaseLatchHud_.fxPageOpen;
        effectiveHud.panPageOpen = releaseLatchHud_.panPageOpen;
        effectiveHud.editPageOpen = releaseLatchHud_.editPageOpen;
        effectiveHud.editTarget = releaseLatchHud_.editTarget;
        effectiveHud.globalTarget = releaseLatchHud_.globalTarget;
        effectiveHud.configCursor = releaseLatchHud_.configCursor;
        effectiveHud.config = releaseLatchHud_.config;
        if (releaseLatchHud_.mode == HudMode::KnobRate) {
            effectiveHud.rateHoldActive = releaseLatchHud_.rateHoldActive;
            effectiveHud.knobRate = releaseLatchHud_.knobRate;
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

    globalViews_.update(dtClamped, control, hud, frameLfoOutput_, frameVoicePeak_);
    ampGraph_.update(dtClamped, control, hud, frameLfoOutput_, frameVoiceDrift_,
                     traceEngine_.overdriveHair());
    const bool extraCenterView = globalViews_.anyActiveTarget(hud) || ampGraph_.isActive(hud);
    graphs_.update(dtClamped, control, hud, effectiveHud, extraCenterView, frameLfoOutput_,
                   frameLfoDriftPacked_);
}

void ScopeHudRenderer::renderStrip(uint16_t yStart, uint16_t stripHeight,
                                   uint16_t* rgb565Out) const noexcept {
    if (rgb565Out == nullptr || yStart >= kDisplayHeight || stripHeight == 0u) {
        return;
    }

    const uint16_t totalEnd = (yStart + stripHeight > kDisplayHeight)
                                  ? kDisplayHeight
                                  : static_cast<uint16_t>(yStart + stripHeight);

    const float bgDim01 = smoothstep01(clampf(graphs_.backdropDim(), 0.0f, 1.0f));
    const uint32_t bgKeepQ8 =
        static_cast<uint32_t>((1.0f - kFilterBgDimDepth * bgDim01) * 256.0f + 0.5f);

    constexpr uint16_t kSubStripRows = 8u;
    std::array<uint16_t, kDisplayWidth * kSubStripRows> linBuf{};
    std::array<uint8_t, kDisplayWidth * kSubStripRows> lumBuf{};
    uint16_t curY = yStart;

    while (curY < totalEnd) {
        const uint16_t chunkH = (totalEnd - curY > kSubStripRows)
                                    ? kSubStripRows
                                    : static_cast<uint16_t>(totalEnd - curY);
        const int y0 = static_cast<int>(curY);
        const int y1 = y0 + static_cast<int>(chunkH);

        traceEngine_.renderBackgroundChunk(y0, chunkH, bgKeepQ8, linBuf.data(), lumBuf.data());

        const uint8_t hudAlpha8 =
            static_cast<uint8_t>(clampf(frameHud_.opacity, 0.0f, 1.0f) * 255.0f + 0.5f);
        if (hudAlpha8 < 255u && y0 < 18 && y1 > 4) {
            const MonoStripCanvas idleCv{lumBuf.data(), y0, y1,
                                         static_cast<uint8_t>(255u - hudAlpha8)};
            compositeIdlePeakMeters(idleCv, smoothedPeakL_, smoothedPeakR_);
        }

        if (hudAlpha8 > 0u && frameHud_.mode != HudMode::None) {
            compositeHudStrip(curY, chunkH, lumBuf.data(), linBuf.data(), hudAlpha8);
        }

        uint16_t* dstBase = rgb565Out + static_cast<size_t>(curY - yStart) * kDisplayWidth;
        for (uint16_t r = 0u; r < chunkH; ++r) {
            const int py = y0 + static_cast<int>(r);
            const uint8_t* srcRow = lumBuf.data() + static_cast<size_t>(r) * kDisplayWidth;
            uint16_t* dstRow = dstBase + static_cast<size_t>(r) * kDisplayWidth;
            for (size_t x = 0u; x < kDisplayWidth; ++x) {
                dstRow[x] = packNeutralGrayRgb565(srcRow[x], static_cast<int>(x), py);
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
    float dummy01 = 0.0f;
    const uint8_t filterAlpha8 =
        resolveSubViewAlpha(graphs_.filterGraphPhase(), hudAlpha01, effFilterAlpha01);
    const uint8_t lfoAlpha8 =
        resolveSubViewAlpha(graphs_.lfoGraphPhase(), hudAlpha01, effLfoAlpha01);
    (void)resolveSubViewAlpha(ampGraph_.maxPhase(), hudAlpha01, effEditGraphAlpha01);
    const uint8_t panAlpha8 =
        resolveSubViewAlpha(globalViews_.panPhase(), hudAlpha01, effPanAlpha01);
    const uint8_t cfgAlpha8 = resolveSubViewAlpha(globalViews_.configPhase(), hudAlpha01, dummy01);

    cv.applyVerticalScrim(0, 34, 0.80f, true);
    if (frameHud_.knobBank != KnobBankMode::None) {
        float graphEnvMax = (effFilterAlpha01 > effLfoAlpha01) ? effFilterAlpha01 : effLfoAlpha01;
        if (effEditGraphAlpha01 > graphEnvMax) {
            graphEnvMax = effEditGraphAlpha01;
        }
        if (effPanAlpha01 > graphEnvMax) {
            graphEnvMax = effPanAlpha01;
        }
        const int botScrimTop = 178 - static_cast<int>(8.0f * graphEnvMax + 0.5f);
        cv.applyVerticalScrim(botScrimTop, 240, 0.90f, false);
    }

    graphs_.compositeLfoStrip(y0, y1, stripHeight, lumBuf, linScratch, effLfoAlpha01, lfoAlpha8,
                              frameControl_, frameHud_, frameLfoPhase_);
    graphs_.compositeFilterStrip(y0, y1, stripHeight, lumBuf, linScratch, effFilterAlpha01,
                                 filterAlpha8);
    ampGraph_.compositeStrip(y0, y1, stripHeight, lumBuf, linScratch, hudAlpha01, frameHud_);
    globalViews_.compositePanStageStrip(y0, y1, stripHeight, lumBuf, linScratch, effPanAlpha01,
                                        panAlpha8, frameHud_);
    globalViews_.compositeGlobalConfigStrip(y0, y1, lumBuf, cfgAlpha8, frameControl_, frameHud_);
    compositeTopHeaderStrip(cv, frameControl_, frameHud_, frameLfoOutput_, headerBarSmooth_,
                            fineOverlayPhase_, coarseOverlayPhase_);
    compositeBottomKnobStrip(cv, frameControl_, frameHud_, frameLfoOutput_, knobPosSignedSmooth_,
                             knobDimSmooth_, knobFocusSmooth_);
}

} // namespace ruido
