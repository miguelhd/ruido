#include "ui/HudGlobalViews.h"

#include "core/DspMath.h"
#include "core/GlobalConfig.h"
#include "core/HudEnvelope.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"
#include "ui/FontSansAA.h"
#include "ui/GraphStyle.h"
#include "ui/HudCouplingView.h"
#include "ui/HudFormat.h"
#include "ui/MonoCanvas.h"
#include "ui/ScopeTraceMath.h"

#include <cstring>

namespace ruido {

namespace {

constexpr float kPanYTop = kGraphTopY;
constexpr float kPanYBot = kGraphBaselineY;
constexpr float kPanZeroRef = 110.0f;
constexpr float kPanCenterX = 120.0f;
constexpr float kPanHalfSpanX = 106.0f; // L100 = 14.0f, C = 120.0f, R100 = 226.0f

// Steep-shouldered Cauchy-Lorentzian acoustic localization bell: (1 + (dx/w)^2)^-1.5
// with smooth zero-toe so distant tails settle cleanly onto kPanYBot without baseline haze.
inline float evalPanBellY(float x, float cx, float apexH, float invW2) noexcept {
    const float dx = x - cx;
    const float d = 1.0f + dx * dx * invW2;
    const float invSqrtD = fastInvSqrt(d);
    const float rawBell = invSqrtD * invSqrtD * invSqrtD;
    const float bell = (rawBell > 0.012f) ? ((rawBell - 0.012f) * (1.0f / 0.988f)) : 0.0f;
    return clampf(kPanYBot - apexH * bell, kPanYTop, kPanYBot);
}

inline void shadePanoramaCurveSkirt(int y0, int y1, uint8_t* lumBuf, float cx, float apexH,
                                    float invW2, float colScale) noexcept {
    if (colScale <= 0.004f) {
        return;
    }
    float yA = evalPanBellY(0.0f, cx, apexH, invW2);
    for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
        const float yB = evalPanBellY(static_cast<float>(colX + 1u), cx, apexH, invW2);
        shadeUnipolarGraphColumn(colX, yA, yB, kPanZeroRef, 74.0f, colScale, y0, y1, lumBuf);
        yA = yB;
    }
}

// Analog carbon-pill keys only (`OP1..GLO`, `Keys 1-12`); binary right-bay switches excluded.
constexpr uint16_t kAnalogBlackMask = 0x00FFu;
constexpr uint16_t kAnalogWhiteMask = 0x0FFFu;
constexpr float kNoPressureNoticeSec = 0.9f;
constexpr float kPressMeterX0 = 16.0f;
constexpr float kPressMeterX1 = 224.0f;
constexpr float kPressPeakHoldSec = 1.2f;

} // namespace

void HudGlobalViews::reset() noexcept {
    panX_ = {83.0f, 157.0f, 51.0f, 189.0f};
    panLfoDepth_.fill(0.0f);
    apexH_.fill(88.0f);
    invWidth2_ = {0.0029f, 0.0035f, 0.0041f, 0.0049f};
    focusWeight_.fill(0.0f);
    muteDim_.fill(0.0f);
    voicePhase01_ = {0.0f, 0.21f, 0.42f, 0.73f};
    capAct_.fill(0.72f);
    couplingFullSpanSmooth_ = {800.0f, 534.0f, 400.0f, 267.0f};
    couplingSagQ8_.fill(0u);
    for (auto& r : couplingRampQ8_) {
        r.fill(0u);
    }
    for (auto& w : couplingWrapPacked_) {
        w.fill(0u);
    }
    couplingSyncMask_.fill(0u);
    hasFocusSmooth_ = 0.0f;
    couplingSmooth_ = 0.0f;
    couplingChaos_ = 0.0f;
    couplingWinSamplesSmooth_ = 1800.0f;
    monoCollapse_ = false;
    primed_ = false;
    couplingPrimed_ = false;
    clearImmediate();
}

void HudGlobalViews::clearImmediate() noexcept {
    panPhase_ = 0.0f;
    couplingPhase_ = 0.0f;
    configPhase_ = 0.0f;
    pressLive01_ = pressPeak01_ = pressPeakHoldSec_ = pressMeterPhase_ = 0.0f;
    noPressureHoldSec_ = 0.0f;
    primed_ = false;
    couplingPrimed_ = false;
}

void HudGlobalViews::updatePressureMeter(float dtClamped, const HudState& rawHud,
                                         bool cfgActive) noexcept {
    const bool onRow = cfgActive && rawHud.configSubView == ConfigSubView::List &&
                       rawHud.configCursor == static_cast<uint8_t>(ConfigItemId::PressureTouch);
    uint8_t liveQ8 = 0u;
    bool analogHeld = false;
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        if (((kAnalogBlackMask & rawHud.heldBlackMask) >> i) & 1u) {
            analogHeld = true;
            liveQ8 = (rawHud.blackPressureQ8[i] > liveQ8) ? rawHud.blackPressureQ8[i] : liveQ8;
        }
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        if (((kAnalogWhiteMask & rawHud.heldWhiteMask) >> i) & 1u) {
            analogHeld = true;
            liveQ8 = (rawHud.whitePressureQ8[i] > liveQ8) ? rawHud.whitePressureQ8[i] : liveQ8;
        }
    }
    const bool nonAnalogHeld = ((rawHud.heldBlackMask & ~kAnalogBlackMask) != 0u) ||
                               ((rawHud.heldWhiteMask & ~kAnalogWhiteMask) != 0u);
    if (onRow && nonAnalogHeld) {
        noPressureHoldSec_ = kNoPressureNoticeSec;
    } else if (noPressureHoldSec_ > 0.0f) {
        noPressureHoldSec_ =
            (onRow && noPressureHoldSec_ > dtClamped) ? (noPressureHoldSec_ - dtClamped) : 0.0f;
    }
    analogHeld = analogHeld && onRow;
    pressLive01_ = analogHeld ? static_cast<float>(liveQ8) * (1.0f / 255.0f) : 0.0f;
    if (analogHeld) {
        pressPeak01_ = (pressLive01_ > pressPeak01_) ? pressLive01_ : pressPeak01_;
        pressPeakHoldSec_ = kPressPeakHoldSec;
    } else if (pressPeakHoldSec_ > 0.0f) {
        pressPeakHoldSec_ -= dtClamped;
    }
    pressMeterPhase_ = stepHudFadeLinear(
        pressMeterPhase_, onRow && (analogHeld || pressPeakHoldSec_ > 0.0f), dtClamped);
    if (pressMeterPhase_ <= 0.002f) {
        pressPeak01_ = 0.0f;
        pressPeakHoldSec_ = 0.0f;
    }
}

bool HudGlobalViews::anyActiveTarget(const HudState& rawHud) const noexcept {
    if (rawHud.opacity <= 0.004f || rawHud.mode == HudMode::None) {
        return false;
    }
    return isPanStageActive(rawHud) || isRailCouplingActive(rawHud) || rawHud.configPageOpen;
}

bool HudGlobalViews::hasResidualPhase() const noexcept {
    return panPhase_ > 0.002f || couplingPhase_ > 0.002f || configPhase_ > 0.002f ||
           pressMeterPhase_ > 0.002f || noPressureHoldSec_ > 0.0f;
}

void HudGlobalViews::update(float dtClamped, const EngineControlState& control,
                            const HudState& rawHud, const LfoSignalArray& lfoOutput,
                            const std::array<float, kNumOscillators>& voicePeak,
                            const std::array<float, kNumOscillators>& voiceDriftNorm,
                            uint8_t stutterGateMask, const ScopeTraceEngine& traceEngine) noexcept {
    const bool hudVisible =
        (rawHud.opacity > 0.004f) && (rawHud.mode != HudMode::None) && !rawHud.shiftHeld;
    const bool panActive = isPanStageActive(rawHud);
    const bool couplingActive = isRailCouplingActive(rawHud);
    const bool cfgActive =
        hudVisible && (rawHud.configPageOpen ||
                       (rawHud.mode == HudMode::GlobalConfig && rawHud.opacity >= 0.99f));
    const bool wasPanInvisible = (panPhase_ <= 0.002f);
    const bool wasCouplingInvisible = (couplingPhase_ <= 0.002f);

    panPhase_ = stepHudFadeLinear(panPhase_, panActive, dtClamped);
    couplingPhase_ = stepHudFadeLinear(couplingPhase_, couplingActive, dtClamped);
    configPhase_ = stepHudFadeLinear(configPhase_, cfgActive, dtClamped);
    updatePressureMeter(dtClamped, rawHud, cfgActive);

    if (couplingActive || couplingPhase_ > 0.002f) {
        const bool snapCoupling = !couplingPrimed_ || (couplingActive && wasCouplingInvisible);
        couplingPrimed_ = true;
        const float alphaCpl =
            snapCoupling ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.028f)));
        const float alphaGate =
            snapCoupling ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.004f)));
        const float effCoupling =
            clampf(control.global.circuitCoupling + control.keyMod.overloadSurge, 0.0f, 1.0f);
        const float effChaos =
            (effCoupling > 0.45f) ? smoothstep01((effCoupling - 0.45f) * (1.0f / 0.55f)) : 0.0f;
        couplingSmooth_ = slewScalar(couplingSmooth_, effCoupling, alphaCpl, 0.002f);
        couplingChaos_ = slewScalar(couplingChaos_, effChaos, alphaCpl, 0.002f);

        const float pitchMode = control.global.pitchPressureMode ? 1.0f : 0.0f;
        std::array<float, kNumOscillators> liveHz{};
        std::array<bool, kNumOscillators> voiceMuted{};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const VoiceParams& vp = control.voices[v];
            const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
            const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
            const float pMul = computeVoicePitchRatio(control.voicePressure[v], pitchMode) *
                               control.keyMod.voicePitchMul[v];
            const float dAmt = clampf(vp.drift, 0.0f, 1.0f);
            const float maxDriftOct = dAmt * (0.018f + 0.082f * dAmt * dAmt);
            const float driftMul = fastExp2(clampf(voiceDriftNorm[v], -1.0f, 1.0f) * maxDriftOct);
            liveHz[v] =
                clampf(computeLiveVoicePitchHz(vp, lfoA, lfoB) * pMul * driftMul, 0.5f, 20000.0f);

            const VoiceCircuitState vc = evaluateVoiceCircuit(control, v, stutterGateMask);
            voiceMuted[v] = vc.isMuted || !control.global.masterRun;
            const float aGate = (control.keyMod.stutterDepth[v] > 1.0e-4f) ? alphaGate : alphaCpl;
            if (!panActive) {
                muteDim_[v] = slewScalar(muteDim_[v], vc.inaudibleDim, aGate, 0.005f);
            }
        }

        if (traceEngine.hasLiveAudioHistory() && !traceEngine.isStreamStalled(rawHud.scopeFrozen)) {
            if (!rawHud.scopeFrozen || snapCoupling) {
                buildCouplingFromLiveHistory(
                    dtClamped, snapCoupling, couplingSmooth_, couplingChaos_, liveHz, voiceMuted,
                    traceEngine, couplingWinSamplesSmooth_, couplingFullSpanSmooth_, capAct_,
                    couplingSagQ8_, couplingRampQ8_, couplingWrapPacked_, couplingSyncMask_);
            }
        } else {
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
                const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
                const float effAmp =
                    evaluateLiveOscAmplitude(control, v, lfoA, lfoB, stutterGateMask);
                const float tgtAct =
                    voiceMuted[v] ? 0.0f : clampf(0.35f + 0.65f * effAmp, 0.0f, 1.0f);
                capAct_[v] = slewScalar(capAct_[v], tgtAct, alphaCpl, 0.004f);
                if (v > 0u && !rawHud.scopeFrozen && control.global.masterRun && dtClamped > 0.0f) {
                    voicePhase01_[v] = wrapLfoPhase01(
                        voicePhase01_[v] + (0.22f + 0.14f * static_cast<float>(v)) * dtClamped);
                }
            }
            buildCouplingFallback(couplingSmooth_, couplingChaos_, liveHz, voicePhase01_, capAct_,
                                  couplingSagQ8_, couplingRampQ8_, couplingWrapPacked_,
                                  couplingSyncMask_);
        }
    } else {
        couplingPrimed_ = false;
    }

    if (!panActive) {
        if (panPhase_ <= 0.002f) {
            primed_ = false;
        }
        return;
    }

    const bool snapPan = !primed_ || wasPanInvisible;
    primed_ = true;
    const float sf = control.keyMod.spatialFocus;
    const float panScale = computeEffectivePanScale(control.global.monoCollapse, sf);
    monoCollapse_ = (panScale <= 0.001f);

    const float alphaPos = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.034f)));
    const float alphaChop = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.004f)));
    const float alphaFocus = snapPan ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.042f)));

    const bool spotlightActive = (rawHud.knobBank == KnobBankMode::Panning) &&
                                 (rawHud.focusedKnob < kNumOscillators) && !monoCollapse_;
    hasFocusSmooth_ =
        slewScalar(hasFocusSmooth_, spotlightActive ? 1.0f : 0.0f, alphaFocus, 0.002f);

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        const VoiceParams& vp = control.voices[v];
        const float lfoA = lfoOutput[lfoBankIndex(v, 0u)];
        const float lfoB = lfoOutput[lfoBankIndex(v, 1u)];
        const VoiceCircuitState vc = evaluateVoiceCircuit(control, v, stutterGateMask);
        const bool silentMute = vc.isMuted;
        const float effAmp = evaluateLiveOscAmplitude(control, v, lfoA, lfoB, stutterGateMask);
        const float livePan = applySpatialFocusPan(computeLiveVoicePan(vp, lfoA, lfoB), v,
                                                   control.global.monoCollapse, sf);

        const float dA = computeLiveLfoADepth(vp, lfoB);
        float modDepth = (vp.lfoTarget == LfoTarget::Pan) ? dA : 0.0f;
        if (vp.lfoBTarget == LfoTarget::Pan) {
            modDepth = clampf(modDepth + vp.lfoBDepth, 0.0f, 1.0f);
        }
        modDepth *= panScale;

        const float peakNorm =
            silentMute ? 0.0f : clampf(voicePeak[v] * (1.0f / 1.15f), 0.0f, 1.0f);
        const float tgtApexH =
            silentMute ? 22.0f : clampf(46.0f + 78.0f * effAmp + 10.0f * peakNorm, 42.0f, 134.0f);

        // Psychoacoustic localization width: bass (~55 Hz) warmer/wider, highs (~440 Hz+) tighter
        const float octAboveBass = clampf(
            fastLog2(clampf(vp.frequencyHz, 20.0f, 20000.0f) * (1.0f / 55.0f)) * (1.0f / 3.0f),
            0.0f, 1.0f);
        const float halfW = 18.5f - 6.0f * octAboveBass;
        const float tgtInvW2 = 1.0f / (halfW * halfW);
        const float tgtFocus = (spotlightActive && rawHud.focusedKnob == v) ? 1.0f : 0.0f;
        const float aGate = (control.keyMod.stutterDepth[v] > 1.0e-4f) ? alphaChop : alphaPos;

        panX_[v] = slewScalar(panX_[v], kPanCenterX + livePan * kPanHalfSpanX, alphaPos, 0.02f);
        panLfoDepth_[v] = slewScalar(panLfoDepth_[v], modDepth, alphaPos, 0.002f);
        apexH_[v] = slewScalar(apexH_[v], tgtApexH, aGate, 0.02f);
        invWidth2_[v] = slewScalar(invWidth2_[v], tgtInvW2, alphaPos, 0.00002f);
        focusWeight_[v] = slewScalar(focusWeight_[v], tgtFocus, alphaFocus, 0.002f);
        muteDim_[v] = slewScalar(muteDim_[v], vc.inaudibleDim, aGate, 0.005f);
    }
}

void HudGlobalViews::compositePanStageStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                                            uint16_t* linScratch, float effPanAlpha01,
                                            uint8_t alpha8, const HudState& hud) const noexcept {
    (void)hud;
    if (alpha8 == 0u || y1 <= kGraphStripMinY1 || y0 >= kGraphStripMaxY0) {
        return;
    }

    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};

    // 1. Canonical edge-to-edge baseline filament (y = 186, shared with FILTER & LEVEL)
    drawGraphBaseline(cv);
    if (!monoCollapse_) {
        drawGraphGuideSlit(cv, kPanCenterX, 36.0f);
    }

    const float hF = clampf(hasFocusSmooth_, 0.0f, 1.0f);

    // 2. Layer 1: 4x-subsampled vertical-decay skirt shading (companions first, spotlight on top)
    if (y1 > static_cast<int>(kPanYTop) && y0 <= static_cast<int>(kPanYBot)) {
        for (int pass = 0; pass < 2; ++pass) {
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                const float fW = clampf(focusWeight_[v], 0.0f, 1.0f);
                const bool isPrimary = (fW >= 0.5f);
                if ((pass == 0 && isPrimary) || (pass == 1 && !isPrimary)) {
                    continue;
                }
                const float activeW = 1.0f - 0.78f * clampf(muteDim_[v], 0.0f, 1.0f);
                const float spotScale = 0.16f + 0.76f * fW;
                const float baseFill = 0.34f + hF * (spotScale - 0.34f);
                shadePanoramaCurveSkirt(y0, y1, lumBuf, panX_[v], apexH_[v], invWidth2_[v],
                                        baseFill * activeW * effPanAlpha01);
            }
        }
    }

    // 3. Layer 2: Linear-photon curve spines (additive caustic crossings where voices overlap)
    if (effPanAlpha01 > 0.004f && y1 > static_cast<int>(kPanYTop) - 4 &&
        y0 <= static_cast<int>(kPanYBot) + 5) {
        std::memset(linScratch, 0,
                    static_cast<size_t>(stripHeight) * kDisplayWidth * sizeof(uint16_t));
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float activeW = 1.0f - 0.76f * clampf(muteDim_[v], 0.0f, 1.0f);
            const float fW = clampf(focusWeight_[v], 0.0f, 1.0f);
            const float spotWgt = 0.28f + 0.76f * fW;
            const float weight = (0.62f + hF * (spotWgt - 0.62f)) * activeW * effPanAlpha01;
            if (weight <= 0.004f) {
                continue;
            }
            const float cx = panX_[v];
            const float apexH = apexH_[v];
            const float invW2 = invWidth2_[v];
            float yA = evalPanBellY(0.0f, cx, apexH, invW2);
            for (size_t colX = 0u; colX < kDisplayWidth; ++colX) {
                const float yB = evalPanBellY(static_cast<float>(colX + 1u), cx, apexH, invW2);
                splatUnipolarGraphSegment(colX, yA, yB, weight, y0, y1, linScratch);
                yA = yB;
            }
        }
        screenBlendLinearStrip(stripHeight, 0u, kDisplayWidth, linScratch, lumBuf);
    }

    // 4. Layer 3: Phase Beacons / Optical Crest Beads on live curve apexes
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const float fW = clampf(focusWeight_[v], 0.0f, 1.0f);
            const bool isPrimary = (fW >= 0.5f);
            if ((pass == 0 && isPrimary) || (pass == 1 && !isPrimary)) {
                continue;
            }
            const float dim = clampf(muteDim_[v], 0.0f, 1.0f);
            const float cx = panX_[v];
            const float apexY = evalPanBellY(cx, cx, apexH_[v], invWidth2_[v]);
            const float modD = panLfoDepth_[v];

            if (modD > 0.01f && dim < 0.90f) {
                const float spotBeacon = 0.52f + 0.48f * fW;
                const float beaconInt = (0.84f + hF * (spotBeacon - 0.84f)) * (1.0f - 0.58f * dim);
                cv.drawLfoPhaseBeacon(cx, apexY, beaconInt, 0.35f + 0.65f * modD);
            } else {
                const float spotBeadInt = 0.38f + 0.60f * fW;
                const float beadInt = clampf(
                    (0.78f + hF * (spotBeadInt - 0.78f)) * (1.0f - 0.60f * dim), 0.22f, 1.0f);
                const float spotBeadRad = 1.50f + 0.75f * fW;
                const float beadRad = 1.85f + hF * (spotBeadRad - 1.85f);
                cv.drawOpticalBead(cx, apexY, beadInt, beadRad);
            }
        }
    }
}

void HudGlobalViews::compositeRailCouplingStrip(int y0, int y1, uint16_t stripHeight,
                                                uint8_t* lumBuf, uint16_t* linScratch,
                                                float effCouplingAlpha01,
                                                uint8_t alpha8) const noexcept {
    renderCouplingStrip(y0, y1, stripHeight, lumBuf, linScratch, effCouplingAlpha01, alpha8,
                        couplingSmooth_, couplingChaos_, couplingSagQ8_, couplingRampQ8_,
                        couplingWrapPacked_, couplingSyncMask_, capAct_, muteDim_);
}

namespace {

void compositeConfigSubViewStrip(const MonoStripCanvas& cv, const HudState& hud) noexcept {
    constexpr uint8_t kSubtitleLum = 220u;
    constexpr uint8_t kBodyLum = 174u;

    if (isResetConfirmSubView(hud.configSubView) || isResetDoneSubView(hud.configSubView)) {
        const size_t selIdx = (hud.configSubView == ConfigSubView::CleanInitConfirm ||
                               hud.configSubView == ConfigSubView::CleanInitDone)
                                  ? 1u
                                  : 0u;
        const bool isDone = isResetDoneSubView(hud.configSubView);
        const uint32_t clampedTicks =
            isDone ? 8u : ((hud.configResetTicks > 8u) ? 8u : hud.configResetTicks);
        const float norm = static_cast<float>(clampedTicks) * (1.0f / 8.0f);

        // 1. Native 2-Row GLOBAL CONFIG List (`y = 44, 72`, `h = 24`, inline bottom hairline)
        for (size_t r = 0u; r < 2u; ++r) {
            const int ry = 44 + static_cast<int>(r) * 28;
            const bool sel = (r == selIdx);
            if (ry + 28 >= cv.y0 - 2 && ry <= cv.y1 + 2) {
                if (sel) {
                    const int fillTop = (ry < cv.y0) ? cv.y0 : ry;
                    const int fillBot = (ry + 24 > cv.y1) ? cv.y1 : (ry + 24);
                    const auto fillAlpha = static_cast<uint8_t>(
                        (28u * static_cast<uint32_t>(cv.masterAlpha8) + 127u) / 255u);
                    if (fillAlpha > 0u && fillTop < fillBot) {
                        for (int py = fillTop; py < fillBot; ++py) {
                            uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
                            for (int px = 10; px <= 230; ++px) {
                                row[px] = blendLum8(row[px], 255u, fillAlpha);
                            }
                        }
                    }
                }
                cv.drawSansFlushLeft(kFontSansMedium, 16, ry + 5,
                                     (r == 0u) ? "Factory Presets" : "Clean Init",
                                     sel ? 255u : 174u, 0);

                char pctBuf[12]{};
                const char* rightStr = (r == 0u) ? "RESET >" : "INIT >";
                uint8_t rightLum = sel ? 196u : 120u;
                if (sel && isDone) {
                    rightStr = "DONE";
                    rightLum = 255u;
                } else if (sel && clampedTicks > 0u) {
                    formatPercent(pctBuf, sizeof(pctBuf), norm);
                    rightStr = pctBuf;
                    rightLum = 255u;
                }
                cv.drawSansRight(kFontSansMedium, 224, ry + 5, rightStr, rightLum, 0);

                if (sel) {
                    const float yLine = static_cast<float>(ry) + 22.0f;
                    const float span = kPressMeterX1 - kPressMeterX0;
                    cv.drawHorizontalFilament(kPressMeterX0, kPressMeterX1, yLine, 40.0f, 0.5f,
                                              false);
                    if (norm > 0.001f) {
                        const float fillEnd = kPressMeterX0 + span * norm;
                        cv.drawHorizontalFilament(kPressMeterX0, fillEnd, yLine,
                                                  isDone ? 255.0f : 225.0f, 0.65f, false);
                        if (!isDone) {
                            cv.drawOpticalBead(fillEnd, yLine, 0.95f, 1.85f);
                        }
                    }
                }
            }
        }

        // 2. Quiet 3-Line Scope Summary (`SOUND`, `SCENES 1-4`, `GLOBAL CONFIG`)
        cv.drawHorizontalFilament(16.0f, 224.0f, 112.5f, 28.0f, 0.5f, false);
        if (cv.y1 > 120 && cv.y0 < 182) {
            constexpr std::array<const char*, 3> kKeys = {"SOUND", "SCENES 1-4", "GLOBAL CONFIG"};
            const std::array<const char*, 3> vals =
                (selIdx == 0u) ? std::array<const char*, 3>{"FACTORY", "FACTORY", "RESET"}
                               : std::array<const char*, 3>{"INIT NO FX", "EMPTY", "KEEP"};
            for (size_t i = 0u; i < 3u; ++i) {
                const int sy = 124 + static_cast<int>(i) * 20;
                cv.drawSansFlushLeft(kFontSansSmall, 16, sy, kKeys[i], 116u, 0);
                cv.drawSansRight(kFontSansSmall, 224, sy, vals[i], 196u, 0);
            }
        }

        // 3. Calm Single-Line Footer (`y = 196`, strictly above y = 210 bottom bezel band)
        if (cv.y1 > 192 && cv.y0 < 210) {
            cv.drawSansCentered(kFontSansSmall, 120, 196,
                                isDone ? "APPLIED  /  CCW BACK" : "TURN ALGORITHM CW  /  CCW BACK",
                                122u, 0);
        }
        return;
    }

    // ConfigSubView::Credits ("ABOUT" — aligned to kConfigMenuLeftX = 16 matching menu options)
    constexpr int kAboutLeftX = 16;
    cv.drawSansFlushLeft(kFontSansMedium, kAboutLeftX, 46, "4-Oscillator Drone Synthesizer",
                         kSubtitleLum);
    cv.drawSansFlushLeft(kFontSansMedium, kAboutLeftX, 78, "By Miguel Hern\xC3\xA1ndez Domenech",
                         kBodyLum);
    cv.drawSansFlushLeft(kFontSansMedium, kAboutLeftX, 102, "GPL-3.0  /  2026  /  Dinghy HAL",
                         kBodyLum);
    cv.drawSansFlushLeft(kFontSansMedium, kAboutLeftX, 126, "github.com/miguelhd/ruido", kBodyLum);
}

} // namespace

void HudGlobalViews::compositeGlobalConfigStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                                const EngineControlState& control,
                                                const HudState& hud) const noexcept {
    if (alpha8 == 0u || y1 <= 24 || y0 >= 234) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    if (hud.configSubView != ConfigSubView::List) {
        compositeConfigSubViewStrip(cv, hud);
        return;
    }
    if (y1 <= 38 || y0 >= 232) {
        return;
    }
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
                static_cast<uint8_t>((28u * static_cast<uint32_t>(alpha8) + 127u) / 255u);
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
        const uint8_t rowLum = selected ? 255u : 174u;
        const bool transportGatedOff = (itemId == ConfigItemId::MidiTransport) &&
                                       (hud.config.clockSync == ClockSyncMode::Internal);
        const uint8_t valLum = transportGatedOff ? (selected ? 148u : 96u) : rowLum;
        if (selected && itemId == ConfigItemId::PressureTouch && noPressureHoldSec_ > 0.0f) {
            // An on/off key (SOLO/MUTE/FREEZE) was touched: say so plainly across the whole row.
            cv.drawSansFlushLeft(kFontSansMedium, 16, ry + 5, "Not pressure sensitive", rowLum, 0);
        } else {
            cv.drawSansFlushLeft(kFontSansMedium, 16, ry + 5, itemLbl, rowLum, 0);
            cv.drawSansRight(kFontSansMedium, 224, ry + 5, valLbl, valLum, 0);
        }

        if (selected && itemId == ConfigItemId::PressureTouch && pressMeterPhase_ > 0.002f) {
            const auto meterA8 =
                static_cast<uint8_t>(pressMeterPhase_ * static_cast<float>(alpha8) + 0.5f);
            const MonoStripCanvas mcv{lumBuf, y0, y1, meterA8};
            const float yLine = static_cast<float>(ry) + 22.0f;
            const float span = kPressMeterX1 - kPressMeterX0;
            mcv.drawHorizontalFilament(kPressMeterX0, kPressMeterX1, yLine, 34.0f, 0.5f, false);
            if (pressLive01_ > 0.004f) {
                const float lum = (pressLive01_ >= 0.995f) ? 255.0f : 176.0f;
                mcv.drawHorizontalFilament(kPressMeterX0, kPressMeterX0 + span * pressLive01_,
                                           yLine, lum, 0.5f, false);
            }
            if (pressPeak01_ > 0.01f) {
                mcv.drawVerticalSlit(kPressMeterX0 + span * pressPeak01_, yLine - 3.0f,
                                     yLine + 1.0f, 230.0f);
            }
        }
    }
}

} // namespace ruido
