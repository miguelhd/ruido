#include "ui/HudGlobalViews.h"

#include "core/DspMath.h"
#include "core/GlobalConfig.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "ui/FontSansAA.h"
#include "ui/HudGraphs.h"
#include "ui/MonoCanvas.h"

namespace ruido {

namespace {

constexpr float kGvAudioLogMin = cxLog2Pos(kAudioFreqMinHz);
constexpr float kGvAudioLogSpan = cxLog2Pos(kAudioFreqMaxHz) - kGvAudioLogMin;
constexpr float kStageYBot = 180.0f;
constexpr float kFreqXLeft = 14.0f;
constexpr float kFreqXSpan = 212.0f; // x = 14..226

} // namespace

void HudGlobalViews::reset() noexcept {
    freqStemX_ = {40.0f, 90.0f, 140.0f, 190.0f};
    freqStemTopY_.fill(kStageYBot);
    freqStemInt_.fill(0.0f);

    ampStemX_ = {60.0f, 100.0f, 140.0f, 180.0f};
    ampStemTopY_.fill(kStageYBot);
    ampStemInt_.fill(0.0f);

    allStemX_ = {40.0f, 70.0f, 100.0f, 130.0f};
    allStemTopY_.fill(kFilterYBot);
    allStemInt_.fill(0.0f);

    clearImmediate();
}

void HudGlobalViews::clearImmediate() noexcept {
    freqPhase_ = 0.0f;
    ampPhase_ = 0.0f;
    allOverlayPhase_ = 0.0f;
    configPhase_ = 0.0f;
}

bool HudGlobalViews::anyActiveTarget(const HudState& rawHud) const noexcept {
    if (rawHud.opacity <= 0.004f || rawHud.mode == HudMode::None) {
        return false;
    }
    return rawHud.knobBank == KnobBankMode::GlobalFreq ||
           rawHud.knobBank == KnobBankMode::GlobalAmp ||
           rawHud.knobBank == KnobBankMode::GlobalAll ||
           (rawHud.editPageOpen && !rawHud.modifierHeld &&
            rawHud.knobBank == KnobBankMode::Panning) ||
           rawHud.configPageOpen;
}

bool HudGlobalViews::hasResidualPhase() const noexcept {
    return freqPhase_ > 0.002f || ampPhase_ > 0.002f || allOverlayPhase_ > 0.002f ||
           configPhase_ > 0.002f;
}

void HudGlobalViews::update(float dtClamped, const EngineControlState& control,
                            const HudState& rawHud,
                            const std::array<float, kNumOscillators>& lfoOutput) noexcept {
    const bool hudVisible = (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None);
    const bool freqActive = hudVisible && (rawHud.knobBank == KnobBankMode::GlobalFreq);
    const bool ampActive =
        hudVisible &&
        (rawHud.knobBank == KnobBankMode::GlobalAmp ||
         (rawHud.editPageOpen && !rawHud.modifierHeld && rawHud.knobBank == KnobBankMode::Panning));
    const bool allActive = hudVisible && (rawHud.knobBank == KnobBankMode::GlobalAll);
    const bool cfgActive =
        hudVisible && (rawHud.configPageOpen ||
                       (rawHud.mode == HudMode::GlobalConfig && rawHud.opacity >= 0.99f));

    const bool snapFreq = (freqPhase_ <= 0.015f);
    const bool snapAmp = (ampPhase_ <= 0.015f);
    const bool snapAll = (allOverlayPhase_ <= 0.015f);

    freqPhase_ = stepHudFadeLinear(freqPhase_, freqActive, dtClamped);
    ampPhase_ = stepHudFadeLinear(ampPhase_, ampActive, dtClamped);
    allOverlayPhase_ = stepHudFadeLinear(allOverlayPhase_, allActive, dtClamped);
    configPhase_ = stepHudFadeLinear(configPhase_, cfgActive, dtClamped);

    if (!freqActive && !ampActive && !allActive) {
        return;
    }

    const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
    const float panScale = control.global.monoCollapse ? 0.0f : 1.0f;
    const float alphaSlew = 1.0f - fastExpNeg(dtClamped * (1.0f / 0.040f));

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const VoiceAudibility aud = computeVoiceAudibility(vp, control.voicePressure[v], pitchMode);
        const bool silentMute = aud.muteBlend >= 0.999f;
        const float dim = aud.inaudibleDim;
        const float pMul = computeVoicePitchRatio(control.voicePressure[v], pitchMode);
        const float lfoHz = (vp.lfoTarget == LfoTarget::Pitch)
                                ? applyLfoPitchHz(vp.frequencyHz, vp.lfoDepth, lfoOutput[v])
                                : vp.frequencyHz;
        const float liveHz = clampf(lfoHz * pMul, kAudioFreqMinHz, kAudioFreqMaxHz);
        const float liveAmpRaw = (vp.lfoTarget == LfoTarget::Amplitude)
                                     ? applyLfoAmplitude(vp.amplitude, vp.lfoDepth, lfoOutput[v])
                                     : vp.amplitude;
        const float liveAmp = silentMute ? 0.0f : clampf(liveAmpRaw, 0.0f, 1.0f);
        const float livePanRaw = (vp.lfoTarget == LfoTarget::Pan)
                                     ? applyLfoPan(vp.pan, vp.lfoDepth, lfoOutput[v])
                                     : vp.pan;
        const float livePan = clampf(livePanRaw * panScale, -1.0f, 1.0f);

        if (freqActive) {
            const float uFreq =
                clampf((fastLog2(liveHz) - kGvAudioLogMin) * (1.0f / kGvAudioLogSpan), 0.0f, 1.0f);
            const float tx = kFreqXLeft + uFreq * kFreqXSpan;
            const float ty =
                silentMute ? (kStageYBot - 5.0f) : (kStageYBot - 14.0f - liveAmp * 94.0f);
            const float baseTi = 0.68f + 0.32f * liveAmp;
            const float ti = baseTi + (0.28f - baseTi) * dim;
            freqStemX_[v] = snapFreq ? tx : slewScalar(freqStemX_[v], tx, alphaSlew, 0.01f);
            freqStemTopY_[v] = snapFreq ? ty : slewScalar(freqStemTopY_[v], ty, alphaSlew, 0.01f);
            freqStemInt_[v] = snapFreq ? ti : slewScalar(freqStemInt_[v], ti, alphaSlew, 0.002f);
        }

        if (ampActive) {
            const float tx = 120.0f + livePan * 96.0f;
            const float ty =
                silentMute ? (kStageYBot - 5.0f) : (kStageYBot - 12.0f - liveAmp * 96.0f);
            const float baseTi = 0.66f + 0.34f * liveAmp;
            const float ti = baseTi + (0.26f - baseTi) * dim;
            ampStemX_[v] = snapAmp ? tx : slewScalar(ampStemX_[v], tx, alphaSlew, 0.01f);
            ampStemTopY_[v] = snapAmp ? ty : slewScalar(ampStemTopY_[v], ty, alphaSlew, 0.01f);
            ampStemInt_[v] = snapAmp ? ti : slewScalar(ampStemInt_[v], ti, alphaSlew, 0.002f);
        }

        if (allActive) {
            const float uFlt =
                clampf((fastLog2(liveHz) - kFilterLogMin) * (1.0f / kFilterLogSpan), 0.0f, 1.0f);
            const float tx = filterNormToScreenX(uFlt);
            const ModulatedFilterParams mod =
                computeLiveVoiceFilter(vp, control.voicePressure[v], pitchMode, lfoOutput[v]);
            const DualMs20Coeffs c = makeDualMs20Coeffs(mod.hpCutoffHz, mod.hpResonance,
                                                        mod.lpCutoffHz, mod.lpResonance);
            const float db = evalDualMs20Db(liveHz, c);
            const float trans = clampf(fastExp2(db * (1.0f / 6.0206f)), 0.08f, 1.35f);
            const float effH = clampf(liveAmp * trans, 0.0f, 1.25f);
            const float ty = silentMute ? (kFilterYBot - 6.0f)
                                        : clampf(kFilterYBot - 12.0f - effH * 112.0f,
                                                 kFilterYTop + 2.0f, kFilterYBot - 4.0f);
            const float baseTi = clampf(0.56f + 0.44f * effH, 0.30f, 1.0f);
            const float ti = baseTi + (0.24f - baseTi) * dim;
            allStemX_[v] = snapAll ? tx : slewScalar(allStemX_[v], tx, alphaSlew, 0.01f);
            allStemTopY_[v] = snapAll ? ty : slewScalar(allStemTopY_[v], ty, alphaSlew, 0.01f);
            allStemInt_[v] = snapAll ? ti : slewScalar(allStemInt_[v], ti, alphaSlew, 0.002f);
        }
    }
}

void HudGlobalViews::compositeGlobalFreqStrip(int y0, int y1, uint8_t* lumBuf,
                                              uint8_t alpha8) const noexcept {
    if (alpha8 == 0u || y1 <= 62 || y0 >= 186) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    cv.drawHorizontalFilament(kFreqXLeft, kFreqXLeft + kFreqXSpan, kStageYBot, 36.0f, 0.50f, true);

    // 6 landmark ticks across 0.05 Hz .. 20 kHz: 0.5, 4, 32, 256, 2048, 16384 Hz
    static constexpr std::array<float, 6> kOctTickX = {
        kFreqXLeft + ((cxLog2Pos(0.5f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
        kFreqXLeft + ((cxLog2Pos(4.0f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
        kFreqXLeft + ((cxLog2Pos(32.0f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
        kFreqXLeft + ((cxLog2Pos(256.0f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
        kFreqXLeft + ((cxLog2Pos(2048.0f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
        kFreqXLeft + ((cxLog2Pos(16384.0f) - kGvAudioLogMin) / kGvAudioLogSpan) * kFreqXSpan,
    };
    for (float tx : kOctTickX) {
        cv.drawVerticalSlit(tx, kStageYBot - 2.5f, kStageYBot + 2.5f, 58.0f);
    }

    float minX = freqStemX_[0];
    float maxX = freqStemX_[0];
    float meanY = 0.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (freqStemX_[v] < minX) {
            minX = freqStemX_[v];
        }
        if (freqStemX_[v] > maxX) {
            maxX = freqStemX_[v];
        }
        meanY += freqStemTopY_[v];
    }
    meanY *= 0.25f;
    if (maxX - minX > 2.0f) {
        cv.drawHorizontalFilament(minX, maxX, meanY, 34.0f, 0.58f, false);
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float inten = freqStemInt_[v];
        if (inten <= 0.01f) {
            continue;
        }
        cv.drawVerticalSlit(freqStemX_[v], freqStemTopY_[v], kStageYBot, 64.0f + 150.0f * inten);
        cv.drawOpticalBead(freqStemX_[v], freqStemTopY_[v], inten, 1.85f + 0.65f * inten);
    }
}

void HudGlobalViews::compositeGlobalAmpStrip(int y0, int y1, uint8_t* lumBuf,
                                             uint8_t alpha8) const noexcept {
    if (alpha8 == 0u || y1 <= 62 || y0 >= 186) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    cv.drawHorizontalFilament(20.0f, 220.0f, kStageYBot, 38.0f, 0.50f, true);
    cv.drawVerticalSlit(120.0f, 68.0f, kStageYBot + 3.0f, 38.0f);
    cv.drawVerticalSlit(20.0f, kStageYBot - 3.0f, kStageYBot + 3.0f, 68.0f);
    cv.drawVerticalSlit(220.0f, kStageYBot - 3.0f, kStageYBot + 3.0f, 68.0f);

    float minX = ampStemX_[0];
    float maxX = ampStemX_[0];
    for (size_t v = 1u; v < kNumOscillators; ++v) {
        if (ampStemX_[v] < minX) {
            minX = ampStemX_[v];
        }
        if (ampStemX_[v] > maxX) {
            maxX = ampStemX_[v];
        }
    }
    if (maxX - minX > 1.5f) {
        cv.drawHorizontalFilament(minX, maxX, kStageYBot, 108.0f, 0.65f, false);
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float inten = ampStemInt_[v];
        if (inten <= 0.01f) {
            continue;
        }
        cv.drawVerticalSlit(ampStemX_[v], ampStemTopY_[v], kStageYBot, 66.0f + 152.0f * inten);
        cv.drawOpticalBead(ampStemX_[v], ampStemTopY_[v], inten, 1.90f + 0.65f * inten);
    }
}

void HudGlobalViews::compositeGlobalAllOverlayStrip(int y0, int y1, uint8_t* lumBuf,
                                                    uint8_t alpha8) const noexcept {
    if (alpha8 == 0u || y1 <= 18 || y0 >= 196) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const float inten = allStemInt_[v];
        if (inten <= 0.01f) {
            continue;
        }
        cv.drawVerticalSlit(allStemX_[v], allStemTopY_[v], kFilterYBot, 54.0f + 136.0f * inten);
        cv.drawOpticalBead(allStemX_[v], allStemTopY_[v], inten, 1.75f + 0.55f * inten);
    }
}

void HudGlobalViews::compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                                const EngineControlState& control,
                                                const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 38 || y0 >= 232) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    constexpr size_t kMaxVisibleRows = 6u;
    constexpr int kRowStartY = 44;
    constexpr int kRowPitch = 28;

    const size_t total = kNumGlobalConfigItems;
    const size_t cursor = (hud.configCursor < total) ? static_cast<size_t>(hud.configCursor) : 0u;
    size_t firstRow = 0u;
    if (total > kMaxVisibleRows && cursor >= kMaxVisibleRows - 1u) {
        firstRow = cursor - (kMaxVisibleRows - 2u);
        if (firstRow + kMaxVisibleRows > total) {
            firstRow = total - kMaxVisibleRows;
        }
    }
    const size_t visibleCount =
        (total - firstRow > kMaxVisibleRows) ? kMaxVisibleRows : (total - firstRow);

    for (size_t r = 0u; r < visibleCount; ++r) {
        const size_t itemIdx = firstRow + r;
        const auto itemId = static_cast<ConfigItemId>(itemIdx);
        const int ry = kRowStartY + static_cast<int>(r) * kRowPitch;
        if (ry + kRowPitch < y0 - 2 || ry > y1 + 2) {
            continue;
        }

        const bool selected = (itemIdx == cursor);
        if (selected) {
            const int fillTop = (ry < y0) ? y0 : ry;
            const int fillBot = (ry + 24 > y1) ? y1 : (ry + 24);
            const auto fillAlpha =
                static_cast<uint8_t>((30u * static_cast<uint32_t>(alpha8) + 127u) / 255u);
            if (fillAlpha > 0u && fillTop < fillBot) {
                for (int py = fillTop; py < fillBot; ++py) {
                    uint8_t* row = lumBuf + static_cast<size_t>(py - y0) * kDisplayWidth;
                    for (int px = 10; px <= 230; ++px) {
                        row[px] = blendLum8(row[px], 255u, fillAlpha);
                    }
                }
            }
        }

        const char* itemLbl = globalConfigItemLabel(itemId);
        const char* valLbl = globalConfigValueLabel(itemId, hud.config, control.global);
        if (selected) {
            cv.drawSans(kFontSansMedium, 16, ry + 5, itemLbl, 255u, 0);
            cv.drawSansRight(kFontSansMedium, 224, ry + 5, valLbl, 255u, 0);
        } else {
            cv.drawSans(kFontSansSmall, 16, ry + 6, itemLbl, 138u, 1);
            cv.drawSansRight(kFontSansMedium, 224, ry + 5, valLbl, 174u, 0);
        }
    }
}

} // namespace ruido
