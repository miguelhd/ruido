#include "ui/ScopeHudRenderer.h"

#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "ui/HudChrome.h"
#include "ui/MonoCanvas.h"

namespace ruido {

ScopeHudRenderer::ScopeHudRenderer() noexcept {
    reset();
}

void ScopeHudRenderer::reset() noexcept {
    traceEngine_.reset();
    graphs_.reset();

    frameControl_ = EngineControlState::makeDefault();
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        prevVoiceFreqHz_[v] = frameControl_.voices[v].frequencyHz;
    }
    frameHud_ = {};
    prevRawHud_ = {};
    releaseLatchHud_ = {};
    frameLfoOutput_ = {0.0f, 0.0f, 0.0f, 0.0f};
    frameLfoPhase_ = {0.0f, 0.25f, 0.50f, 0.75f};
    frameLfoDriftPacked_ = {0u, 0u, 0u, 0u};
    knobPosSignedSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobDimSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    knobFocusSmooth_ = {0.0f, 0.0f, 0.0f, 0.0f};
    headerBarSmooth_ = 0.0f;
    renderedHudOpacity_ = 0.0f;
    smoothedPeakL_ = 0.0f;
    smoothedPeakR_ = 0.0f;
    prevHeaderBarMode_ = HudMode::None;
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

    for (uint8_t k = 0u; k < 4u; ++k) {
        knobPosSignedSmooth_[k] = slewScalar(knobPosSignedSmooth_[k], tgt.posSigned[k], alphaGauge);
        knobDimSmooth_[k] = slewScalar(knobDimSmooth_[k], tgt.dimmed[k] ? 1.0f : 0.0f, alphaGauge);
        knobFocusSmooth_[k] =
            slewScalar(knobFocusSmooth_[k], tgt.focused[k] ? 1.0f : 0.0f, alphaGauge);
    }
}

void ScopeHudRenderer::prepareFrame(float dtSec, const EngineControlState& control,
                                    const HudState& hud,
                                    const EngineTelemetry& telemetry) noexcept {
    frameControl_ = control;
    frameHud_.scopeFrozen = hud.scopeFrozen;

    const float dtClamped = clampf(dtSec, 0.001f, 0.250f);
    traceEngine_.advanceWatchdog(dtClamped, hud.scopeFrozen);
    if (!hud.scopeFrozen) {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            frameLfoOutput_[v] = telemetry.lfoOutput[v].load(std::memory_order_relaxed);
            frameLfoPhase_[v] =
                wrapLfoPhase01(telemetry.lfoPhase[v].load(std::memory_order_relaxed));
            frameLfoDriftPacked_[v] = telemetry.lfoDriftPacked[v].load(std::memory_order_relaxed);
        }
    }

    const bool streamStalled = traceEngine_.isStreamStalled(hud.scopeFrozen);
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
        releaseLatchActive_ = false;
        prevRawHud_ = hud;
        if (hud.scopeFrozen ||
            (renderedHudOpacity_ <= 0.004f && graphs_.filterGraphPhase() <= 0.004f &&
             graphs_.backdropDim() <= 0.004f)) {
            frameHud_ = hud;
            renderedHudOpacity_ = 0.0f;
            graphs_.clearImmediate();
            hudGaugesPrimed_ = false;
            return;
        }

        const float homeDecay = fastExpNeg(dtClamped * (1.0f / 0.065f));
        renderedHudOpacity_ *= homeDecay;
        graphs_.decayOnHome(homeDecay);
        if (renderedHudOpacity_ < 0.004f) {
            renderedHudOpacity_ = 0.0f;
            frameHud_.mode = HudMode::None;
            hudGaugesPrimed_ = false;
        }
        frameHud_.opacity = renderedHudOpacity_;
        return;
    }

    const bool prevWasMomentary =
        prevRawHud_.modifierHeld || prevRawHud_.mode == HudMode::ScopeFreeze ||
        prevRawHud_.mode == HudMode::VoiceFilter || prevRawHud_.mode == HudMode::GlobalFilter ||
        prevRawHud_.mode == HudMode::Amplitude || prevRawHud_.mode == HudMode::Panning ||
        prevRawHud_.mode == HudMode::KnobRate || prevRawHud_.mode == HudMode::VoicePressure ||
        prevRawHud_.mode == HudMode::VoiceLfo || prevRawHud_.mode == HudMode::GlobalLfo;
    const bool isPassiveFallback = !hud.modifierHeld && (hud.mode == HudMode::VoiceFrequency) &&
                                   (hud.knobBank == KnobBankMode::Frequency) && !freqChanged &&
                                   (hud.opacity > 0.004f) &&
                                   (hud.opacity <= prevRawHud_.opacity + 0.002f);

    if (!releaseLatchActive_ && prevWasMomentary && prevRawHud_.opacity > 0.02f &&
        isPassiveFallback) {
        releaseLatchActive_ = true;
        releaseLatchHud_ = prevRawHud_;
    } else if (releaseLatchActive_ && !isPassiveFallback) {
        releaseLatchActive_ = false;
    }

    HudState effectiveHud = hud;
    if (releaseLatchActive_) {
        effectiveHud.mode = releaseLatchHud_.mode;
        effectiveHud.knobBank = releaseLatchHud_.knobBank;
        effectiveHud.focusedVoice = releaseLatchHud_.focusedVoice;
        effectiveHud.focusedKnob = releaseLatchHud_.focusedKnob;
        effectiveHud.lfoVoiceFocus = releaseLatchHud_.lfoVoiceFocus;
        effectiveHud.lfoPageOpen = releaseLatchHud_.lfoPageOpen;
        effectiveHud.knobRate = releaseLatchHud_.knobRate;
    }

    frameHud_ = effectiveHud;
    renderedHudOpacity_ = effectiveHud.opacity;
    prevRawHud_ = hud;

    const bool snapGauges = !hudGaugesPrimed_;
    hudGaugesPrimed_ = (renderedHudOpacity_ > 0.004f);
    if (hudGaugesPrimed_) {
        updateHudGauges(dtClamped, snapGauges);
    }

    graphs_.update(dtClamped, control, hud, effectiveHud, frameLfoOutput_, frameLfoDriftPacked_);
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
    const float filterEnv01 = smoothstep01(clampf(graphs_.filterGraphPhase(), 0.0f, 1.0f));
    const float effFilterAlpha01 = filterEnv01 * hudAlpha01;
    const auto filterAlpha8 = static_cast<uint8_t>(effFilterAlpha01 * 255.0f + 0.5f);
    const float lfoEnv01 = smoothstep01(clampf(graphs_.lfoGraphPhase(), 0.0f, 1.0f));
    const float effLfoAlpha01 = lfoEnv01 * hudAlpha01;
    const auto lfoAlpha8 = static_cast<uint8_t>(effLfoAlpha01 * 255.0f + 0.5f);

    cv.applyVerticalScrim(0, 34, 0.80f, true);
    const float graphEnvMax = (filterEnv01 > lfoEnv01) ? filterEnv01 : lfoEnv01;
    const int botScrimTop = 178 - static_cast<int>(8.0f * graphEnvMax + 0.5f);
    cv.applyVerticalScrim(botScrimTop, 240, 0.90f, false);

    graphs_.compositeLfoStrip(y0, y1, stripHeight, lumBuf, linScratch, effLfoAlpha01, lfoAlpha8,
                              frameControl_, frameHud_, frameLfoPhase_);
    compositeTopHeaderStrip(cv, frameControl_, frameHud_, frameLfoOutput_, headerBarSmooth_);
    graphs_.compositeFilterStrip(y0, y1, stripHeight, lumBuf, linScratch, effFilterAlpha01,
                                 filterAlpha8);
    compositeBottomKnobStrip(cv, frameControl_, frameHud_, frameLfoOutput_, knobPosSignedSmooth_,
                             knobDimSmooth_, knobFocusSmooth_);
}

} // namespace ruido
