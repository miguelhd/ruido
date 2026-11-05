#include "ui/HudKeybedView.h"

#include "core/DspMath.h"
#include "core/HudEnvelope.h"
#include "core/KeyboardModelSpec.h"
#include "core/LfoMath.h"
#include "core/RhythmMath.h"
#include "ui/FontSansAA.h"
#include "ui/GraphStyle.h"
#include "ui/HudColumnText.h"
#include "ui/HudFormat.h"
#include "ui/MonoCanvas.h"

namespace ruido {

namespace {

// Exact physical M-VAVE FM-1 keybed proportions centered on the 240x240 display:
//  - 16 bottom keys on a 13.2 px pitch (x = 21.0 .. 219.0, center = 120.0)
//  - 11 top raised keys centered strictly over piano half-step gaps (3 | 2 | 3 | 2 | 1)
//  - 4 registration dots at natural E-F / B-C gaps (4 . 3 . 4 . 3 . 2)
constexpr float kWhiteFirstCenterX = 21.0f;
constexpr float kWhitePitchX = 13.2f;
constexpr float kKeyHalfW = 4.5f;

constexpr std::array<float, kNumBlackKeys> kBlackCenterX = {
    27.6f,  40.8f,  54.0f,  // OP1, OP2, OP3 (gaps 0-1, 1-2, 2-3)
    80.4f,  93.6f,          // OP4, OP5      (gaps 4-5, 5-6)
    120.0f, 133.2f, 146.4f, // OP6, PIT, GLO (gaps 7-8, 8-9, 9-10)
    172.8f, 186.0f,         // MONO, POLY    (gaps 11-12, 12-13)
    212.4f,                 // 11th tall key (gap 14-15)
};

constexpr std::array<float, 4> kRegDotCenterX = {67.2f, 106.8f, 159.6f, 199.2f};

inline float sdfRoundedBox(float px, float py, float cx, float cy, float hx, float hy,
                           float radius) noexcept {
    const float dx = absf(px - cx) - hx;
    const float dy = absf(py - cy) - hy;
    if (dx <= 0.0f && dy <= 0.0f) {
        const float inside = (dx > dy) ? dx : dy;
        return inside - radius;
    }
    const float qx = (dx > 0.0f) ? dx : 0.0f;
    const float qy = (dy > 0.0f) ? dy : 0.0f;
    const float q2 = qx * qx + qy * qy;
    const float qLen = (q2 > 1.0e-6f) ? (q2 * fastInvSqrt(q2)) : 0.0f;
    return qLen - radius;
}

// Anti-aliased SDF rounded stadium enclosure / silicone key capsule (zero 90-degree corners).
inline void fillStadiumSDF(const MonoStripCanvas& cv, float x0, float y0, float x1, float y1,
                           float radius, uint8_t fillGray, uint8_t borderGray,
                           uint8_t maxAlpha8 = 242u) noexcept {
    int py0 = static_cast<int>(y0) - 2;
    int py1 = static_cast<int>(y1) + 2;
    if (py0 < cv.y0) {
        py0 = cv.y0;
    }
    if (py1 > cv.y1 - 1) {
        py1 = cv.y1 - 1;
    }
    if (py0 > py1) {
        return;
    }
    int px0 = static_cast<int>(x0) - 2;
    int px1 = static_cast<int>(x1) + 2;
    if (px0 < 0) {
        px0 = 0;
    }
    if (px1 > static_cast<int>(kDisplayWidth) - 1) {
        px1 = static_cast<int>(kDisplayWidth) - 1;
    }

    const float cx = 0.5f * (x0 + x1);
    const float cy = 0.5f * (y0 + y1);
    const float halfW = 0.5f * (x1 - x0);
    const float halfH = 0.5f * (y1 - y0);
    const float r = (radius < halfW) ? ((radius < halfH) ? radius : halfH) : halfW;
    const float hx = halfW - r;
    const float hy = halfH - r;
    const float alphaScale =
        static_cast<float>(maxAlpha8) * (static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f));

    for (int py = py0; py <= py1; ++py) {
        const float fy = static_cast<float>(py);
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        for (int px = px0; px <= px1; ++px) {
            const float d = sdfRoundedBox(static_cast<float>(px), fy, cx, cy, hx, hy, r);
            if (d >= 0.65f) {
                continue;
            }
            const float outerCov = clampf(0.55f - d, 0.0f, 1.0f);
            const float rimMix = clampf(1.15f + d, 0.0f, 1.0f);
            const float grayF =
                static_cast<float>(fillGray) +
                (static_cast<float>(borderGray) - static_cast<float>(fillGray)) * rimMix;
            const auto a8 = static_cast<uint8_t>(outerCov * alphaScale + 0.5f);
            if (a8 > 0u) {
                row[px] = blendLum8(row[px], static_cast<uint8_t>(grayF + 0.5f), a8);
            }
        }
    }
}

// Anti-aliased SDF translucent silicone LED diffuser slit with varibright phosphor bloom.
inline void fillLedSlitSDF(const MonoStripCanvas& cv, float cx, float yTop, float yBot,
                           float slitHalfW, float glow01) noexcept {
    const float g = clampf(glow01, 0.0f, 1.0f);
    // Perceptual varibright mapping so 0%, 6%, 15%, 30%, 50%, 75%, 80%, 100% are all distinct
    const float lifted = g * (1.58f - 0.58f * g);
    const auto slitGray = static_cast<uint8_t>(38.0f + 217.0f * clampf(lifted, 0.0f, 1.0f) + 0.5f);

    const float reach = (g > 0.20f) ? 3.2f : 1.5f;
    int py0 = static_cast<int>(yTop - reach);
    int py1 = static_cast<int>(yBot + reach) + 1;
    if (py0 < cv.y0) {
        py0 = cv.y0;
    }
    if (py1 > cv.y1 - 1) {
        py1 = cv.y1 - 1;
    }
    if (py0 > py1) {
        return;
    }
    int px0 = static_cast<int>(cx - slitHalfW - reach);
    int px1 = static_cast<int>(cx + slitHalfW + reach) + 1;
    if (px0 < 0) {
        px0 = 0;
    }
    if (px1 > static_cast<int>(kDisplayWidth) - 1) {
        px1 = static_cast<int>(kDisplayWidth) - 1;
    }

    const float cy = 0.5f * (yTop + yBot);
    const float halfH = 0.5f * (yBot - yTop);
    const float r = (slitHalfW < halfH) ? slitHalfW : halfH;
    const float hy = halfH - r;
    const float masterF = static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f);

    for (int py = py0; py <= py1; ++py) {
        const float fy = static_cast<float>(py);
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        for (int px = px0; px <= px1; ++px) {
            const float d = sdfRoundedBox(static_cast<float>(px), fy, cx, cy, 0.0f, hy, r);
            if (d < 0.65f) {
                const float cov = clampf(0.58f - d, 0.0f, 1.0f);
                const auto a8 = static_cast<uint8_t>(cov * 250.0f * masterF + 0.5f);
                if (a8 > 0u) {
                    row[px] = blendLum8(row[px], slitGray, a8);
                }
            } else if (g > 0.22f && d < 2.8f) {
                const float halo = fastExpNeg(d * d * 0.62f) * (g - 0.18f) * 0.52f;
                const auto h8 =
                    static_cast<uint8_t>(clampf(halo, 0.0f, 1.0f) * 255.0f * masterF + 0.5f);
                if (h8 >= 2u) {
                    row[px] = blendLum8(row[px], 255u, h8);
                }
            }
        }
    }
}

struct TopZoneSpan {
    float xL;
    float xR;
    int labelX;
    uint8_t firstKey;
    uint8_t keyCount;
};

// 4 model-specific top-row zones across OP1..POLY (leaving the universal 11th Freeze key open).
constexpr std::array<TopZoneSpan, 4> kTopZoneSpans = {{
    {kBlackCenterX[0] - 3.2f, kBlackCenterX[3] + 3.2f, 53, 0u, 4u},  // OP1-OP4
    {kBlackCenterX[4] - 3.8f, kBlackCenterX[4] + 3.8f, 93, 4u, 1u},  // OP5
    {kBlackCenterX[5] - 3.2f, kBlackCenterX[7] + 3.2f, 134, 5u, 3u}, // OP6, PIT, GLO
    {kBlackCenterX[8] - 3.2f, kBlackCenterX[9] + 3.2f, 180, 8u, 2u}, // MONO, POLY
}};

} // namespace

void formatKeybedTopHeader(const HudState& hud, char* leftMain, size_t leftMainCap,
                           char* leftSuffix, size_t leftSuffixCap, char* rightText, size_t rightCap,
                           bool& outShowBar, float& outBarNorm) noexcept {
    outShowBar = false;
    outBarNorm = 0.0f;

    if (hud.mode == HudMode::KeyboardModel) {
        appendStr(leftMain, 0u, leftMainCap, "KEYBOARD");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " ");
        sp = appendUInt(leftSuffix, sp, leftSuffixCap,
                        static_cast<uint32_t>(hud.keyboardModel) + 1u);
        sp = appendStr(leftSuffix, sp, leftSuffixCap, "/");
        appendUInt(leftSuffix, sp, leftSuffixCap, static_cast<uint32_t>(KeyboardModel::Count));
        appendStr(rightText, 0u, rightCap, "SELECT MODEL");
        return;
    }

    const uint32_t oscNum = static_cast<uint32_t>((hud.activePunchVoice & 3u) + 1u);
    switch (hud.activePunch) {
    case KeyPunchKind::SpotlightSolo: {
        appendStr(leftMain, 0u, leftMainCap, "SOLO");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        appendStr(rightText, 0u, rightCap, (hud.activePunchValue > 0.5f) ? "SOLOED" : "UNSOLOED");
        break;
    }
    case KeyPunchKind::OverloadSurge:
        appendStr(leftMain, 0u, leftMainCap, "DRIVE");
        outBarNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        formatPercent(rightText, rightCap, outBarNorm);
        outShowBar = true;
        break;
    case KeyPunchKind::SubDrop: {
        appendStr(leftMain, 0u, leftMainCap, "SUB OCTAVE");
        const float mag = clampf(-hud.activePunchValue, 0.0f, 2.0f);
        const auto tenths = static_cast<uint32_t>(mag * 10.0f + 0.5f);
        size_t rp = appendStr(rightText, 0u, rightCap, "-");
        rp = appendUInt(rightText, rp, rightCap, tenths / 10u);
        rp = appendStr(rightText, rp, rightCap, ".");
        rp = appendUInt(rightText, rp, rightCap, tenths % 10u);
        appendStr(rightText, rp, rightCap, " OCT");
        outBarNorm = clampf(mag * 0.5f, 0.0f, 1.0f);
        outShowBar = true;
        break;
    }
    case KeyPunchKind::RatioMagnet: {
        appendStr(leftMain, 0u, leftMainCap, "RATIO LOCK");
        const size_t rIdx = (hud.activePunchVoice < kNumJustRatios) ? hud.activePunchVoice : 12u;
        size_t rp = appendUInt(rightText, 0u, rightCap, kJustRatioTable[rIdx].num);
        rp = appendStr(rightText, rp, rightCap, ":");
        appendUInt(rightText, rp, rightCap, kJustRatioTable[rIdx].den);
        outBarNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        outShowBar = true;
        break;
    }
    case KeyPunchKind::DubLpfDrop:
        appendStr(leftMain, 0u, leftMainCap, "LOW PASS");
        formatCompactHz(rightText, rightCap, clampf(hud.activePunchValue, 20.0f, 28000.0f));
        break;
    case KeyPunchKind::MonoFocus:
        appendStr(leftMain, 0u, leftMainCap, "UNISON");
        outBarNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        formatPercent(rightText, rightCap, outBarNorm);
        outShowBar = true;
        break;
    case KeyPunchKind::PolySpread:
        appendStr(leftMain, 0u, leftMainCap, "DETUNE SPREAD");
        outBarNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        formatPercent(rightText, rightCap, outBarNorm);
        outShowBar = true;
        break;
    case KeyPunchKind::StasisFreeze:
        appendStr(leftMain, 0u, leftMainCap, "FREEZE");
        appendStr(rightText, 0u, rightCap, "HOLDING");
        break;
    case KeyPunchKind::VoiceCut: {
        appendStr(leftMain, 0u, leftMainCap, "MUTE");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        appendStr(rightText, 0u, rightCap, (hud.activePunchValue > 0.5f) ? "MUTED" : "UNMUTED");
        break;
    }
    case KeyPunchKind::VoiceFormant: {
        appendStr(leftMain, 0u, leftMainCap, "FILTER");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outBarNorm = clampf(hud.activePunchValue, 0.0f, 1.0f);
        formatPercent(rightText, rightCap, outBarNorm);
        outShowBar = true;
        break;
    }
    case KeyPunchKind::VoiceStutter: {
        appendStr(leftMain, 0u, leftMainCap, "STUTTER");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatCompactHz(rightText, rightCap, computeStutterRateHz(outBarNorm));
        break;
    }
    case KeyPunchKind::VoiceSiren: {
        appendStr(leftMain, 0u, leftMainCap, "OCTAVE");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        const float mag = clampf(hud.activePunchValue, 0.0f, 2.0f);
        const auto tenths = static_cast<uint32_t>(mag * 10.0f + 0.5f);
        size_t rp = appendStr(rightText, 0u, rightCap, "+");
        rp = appendUInt(rightText, rp, rightCap, tenths / 10u);
        rp = appendStr(rightText, rp, rightCap, ".");
        rp = appendUInt(rightText, rp, rightCap, tenths % 10u);
        appendStr(rightText, rp, rightCap, " OCT");
        outBarNorm = clampf(mag * 0.5f, 0.0f, 1.0f);
        outShowBar = true;
        break;
    }
    case KeyPunchKind::SnapshotRecall:
        formatSnapshotHeader(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                             rightCap, outShowBar, outBarNorm);
        break;
    case KeyPunchKind::VoiceXMod: {
        appendStr(leftMain, 0u, leftMainCap, "CROSS-PIN");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    }
    case KeyPunchKind::VoiceFold: {
        appendStr(leftMain, 0u, leftMainCap, "FOLD");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatMultiplierX(rightText, rightCap, computeWavefoldStages(outBarNorm));
        break;
    }
    case KeyPunchKind::VoiceCrush: {
        appendStr(leftMain, 0u, leftMainCap, "CRUSH");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatCompactHz(rightText, rightCap, computeCrushClockHz(outBarNorm));
        break;
    }
    case KeyPunchKind::RailStarve:
        appendStr(leftMain, 0u, leftMainCap, "STARVE");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::RingXor:
        appendStr(leftMain, 0u, leftMainCap, "RING MOD");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::LfoTurbo:
        appendStr(leftMain, 0u, leftMainCap, "TURBO LFO");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatMultiplierX(rightText, rightCap, fastExp2(5.0f * outBarNorm));
        break;
    case KeyPunchKind::FilterScream:
        appendStr(leftMain, 0u, leftMainCap, "SCREAM");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::HardSync:
        appendStr(leftMain, 0u, leftMainCap, "HARD SYNC");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::PhaseScatter:
        appendStr(leftMain, 0u, leftMainCap, "SCATTER");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::VoiceDivide: {
        appendStr(leftMain, 0u, leftMainCap, "DIVIDE");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        appendStr(rightText, 0u, rightCap,
                  clockDivisionLabel(selectVoiceClockDivIndex(hud.activePunchVoice, outBarNorm)));
        break;
    }
    case KeyPunchKind::VoiceEuclid: {
        appendStr(leftMain, 0u, leftMainCap, "EUCLID");
        size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        const uint8_t hits = kEuclidHitCounts[selectEuclidTierIndex(outBarNorm)];
        size_t rp = appendUInt(rightText, 0u, rightCap, hits);
        appendStr(rightText, rp, rightCap, "/16");
        break;
    }
    case KeyPunchKind::RhythmPump:
        appendStr(leftMain, 0u, leftMainCap, "PUMP");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::RhythmChase:
        appendStr(leftMain, 0u, leftMainCap, "CHASE");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::RhythmTempo: {
        appendStr(leftMain, 0u, leftMainCap, "TEMPO");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        const auto bpmInt = static_cast<uint32_t>(
            clampf(hud.effectiveClockBpm * computeTempoMultiplier(outBarNorm), 30.0f, 1200.0f) +
            0.5f);
        size_t rp = appendUInt(rightText, 0u, rightCap, bpmInt);
        appendStr(rightText, rp, rightCap, hud.midiClockSynced ? " BPM EXT" : " BPM");
        break;
    }
    case KeyPunchKind::RhythmPluck:
        appendStr(leftMain, 0u, leftMainCap, "PLUCK");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::RhythmShort:
        appendStr(leftMain, 0u, leftMainCap, "SHORT");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case KeyPunchKind::RhythmSwing:
        appendStr(leftMain, 0u, leftMainCap, "SWING");
        outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);
        formatPercent(rightText, rightCap, computeSwingRatio(outBarNorm));
        break;
    case KeyPunchKind::None:
        appendStr(leftMain, 0u, leftMainCap, keyboardModelTitle(hud.keyboardModel));
        appendStr(rightText, 0u, rightCap, "PREVIEW");
        break;
    }
}

void HudKeybedView::reset() noexcept {
    blackGlow_.fill(0.0f);
    whiteGlow_.fill(0.0f);
    phase_ = 0.0f;
    breathPhase_ = 0.0f;
    primed_ = false;
}

void HudKeybedView::clearImmediate() noexcept {
    phase_ = 0.0f;
    primed_ = false;
}

void HudKeybedView::update(float dtClamped, const EngineControlState& control,
                           const HudState& rawHud,
                           const std::array<float, kNumOscillators>& voicePeak,
                           uint8_t stutterGateMask) noexcept {
    (void)voicePeak;

    const bool active = isActive(rawHud);
    const bool wasInvisible = !hasResidualPhase();
    phase_ = stepHudFadeLinear(phase_, active, dtClamped);
    if (!active && !hasResidualPhase()) {
        primed_ = false;
        return;
    }

    breathPhase_ += dtClamped * 0.35f;
    if (breathPhase_ >= 1.0f) {
        breathPhase_ -= 1.0f;
    }
    const float subBreath01 = 0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(breathPhase_)));
    const float savePulse01 = 0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(breathPhase_ * 7.0f)));

    std::array<float, kNumBlackKeys> tgtBlack{};
    std::array<float, kNumWhiteKeys> tgtWhite{};
    computeKeyboardModelRestLeds(rawHud.keyboardModel, control, rawHud, stutterGateMask,
                                 subBreath01, savePulse01, tgtBlack, tgtWhite);

    for (size_t i = kNumOscillators; i < kNumBlackKeys; ++i) {
        if (((rawHud.heldBlackMask >> i) & 1u) != 0u) {
            tgtBlack[i] = 1.0f;
        }
    }
    for (size_t i = kNumOscillators; i < kNumWhiteKeys; ++i) {
        if (((rawHud.heldWhiteMask >> i) & 1u) != 0u) {
            tgtWhite[i] = 1.0f;
        }
    }

    const bool snap = !primed_ || (active && wasInvisible);
    primed_ = true;
    const float alpha = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.040f)));
    const float alphaChop = snap ? 1.0f : (1.0f - fastExpNeg(dtClamped * (1.0f / 0.004f)));
    for (size_t i = 0u; i < kNumBlackKeys; ++i) {
        const float a =
            (i < kNumOscillators && control.keyMod.stutterDepth[i] > 1.0e-4f) ? alphaChop : alpha;
        blackGlow_[i] = slewScalar(blackGlow_[i], tgtBlack[i], a, 0.002f);
    }
    for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
        const bool isChopKey = (control.keyMod.stutterDepth[i & 3u] > 1.0e-4f);
        whiteGlow_[i] =
            slewScalar(whiteGlow_[i], tgtWhite[i], isChopKey ? alphaChop : alpha, 0.002f);
    }
}

void HudKeybedView::compositeStrip(int y0, int y1, uint8_t* lumBuf, uint8_t alpha8,
                                   const HudState& frameHud) const noexcept {
    if (alpha8 == 0u || y1 <= 28 || y0 >= 236) {
        return;
    }
    const MonoStripCanvas cv{lumBuf, y0, y1, alpha8};
    const KeyboardModelSpec& spec = keyboardModelSpec(frameHud.keyboardModel);
    const auto modelIdx = static_cast<uint8_t>(frameHud.keyboardModel);
    constexpr uint8_t kModelCount = static_cast<uint8_t>(KeyboardModel::Count);

    // 1. Swappable Model Carousel & Role Banner (y = 33..69)
    if (y1 > 30 && y0 < 72) {
        if (modelIdx > 0u) {
            cv.drawSans(kFontSansMedium, 18, 34, "<", 132u, 0);
        }
        if (modelIdx + 1u < kModelCount) {
            cv.drawSansRight(kFontSansMedium, 222, 34, ">", 132u, 0);
        }
        cv.drawSansCentered(kFontSansBold, 120, 33, spec.title, 255u, 0);
        cv.drawSansCentered(kFontSansSmall, 120, 50, spec.subtitle, 168u, 1);

        // Centered N-slot optical model indicator pips around x = 120
        constexpr float kBeadSpacing = 14.0f;
        const float firstX = 120.0f - 0.5f * static_cast<float>(kModelCount - 1u) * kBeadSpacing;
        for (uint8_t m = 0u; m + 1u < kModelCount; ++m) {
            const float fx0 = firstX + static_cast<float>(m) * kBeadSpacing + 3.0f;
            const float fx1 = firstX + static_cast<float>(m + 1u) * kBeadSpacing - 3.0f;
            cv.drawHorizontalFilament(fx0, fx1, 66.0f, 40.0f, 0.55f, true);
        }
        for (uint8_t m = 0u; m < kModelCount; ++m) {
            const float sx = firstX + static_cast<float>(m) * kBeadSpacing;
            const bool sel = (m == modelIdx);
            cv.drawOpticalBead(sx, 66.0f, sel ? 0.98f : 0.28f, sel ? 2.05f : 1.10f);
        }
    }

    // 2. Top-Row 4-Zone Brackets & Labels over OP1..POLY (y = 72..90; 11th Freeze key left open)
    if (y1 > 70 && y0 < 92) {
        for (size_t z = 0u; z < kTopZoneSpans.size(); ++z) {
            const TopZoneSpan& span = kTopZoneSpans[z];
            float glowSum = 0.0f;
            for (uint8_t k = 0u; k < span.keyCount; ++k) {
                glowSum += blackGlow_[span.firstKey + k];
            }
            const float zoneGlow = glowSum * (1.0f / static_cast<float>(span.keyCount));
            const float brkLum = 56.0f + 92.0f * clampf(zoneGlow, 0.0f, 1.0f);
            cv.drawSansCentered(kFontSansSmall, span.labelX, 72, spec.topZones[z], 210u, 0);
            cv.drawHorizontalFilament(span.xL, span.xR, 87.0f, brkLum, 0.58f, true);
            cv.drawVerticalSlit(span.xL, 87.0f, 89.6f, brkLum);
            cv.drawVerticalSlit(span.xR, 87.0f, 89.6f, brkLum);
        }
    }

    // 3. Anatomically Accurate Rounded-Capsule FM-1 Keybed Map (y = 92..164)
    if (y1 > 90 && y0 < 167) {
        fillStadiumSDF(cv, 8.0f, 92.0f, 232.0f, 164.0f, 13.0f, 13u, 74u, 242u);

        // Top 11 raised keys: OP1-POLY have short upper slits + lower silkscreen bar;
        // 11th key has a tall full-height slit (matching physical FM-1 faceplate 1-to-1).
        for (size_t i = 0u; i < kNumBlackKeys; ++i) {
            const float cx = kBlackCenterX[i];
            const float g = clampf(blackGlow_[i], 0.0f, 1.0f);
            const auto rimGray = static_cast<uint8_t>(62.0f + 68.0f * g + 0.5f);
            fillStadiumSDF(cv, cx - kKeyHalfW, 98.0f, cx + kKeyHalfW, 124.0f, kKeyHalfW, 20u,
                           rimGray, 248u);
            if (i < 10u) {
                fillLedSlitSDF(cv, cx, 101.8f, 109.8f, 1.05f, g);
                cv.drawHorizontalFilament(cx - 2.0f, cx + 2.0f, 117.5f, 48.0f, 0.55f, true);
            } else {
                fillLedSlitSDF(cv, cx, 102.2f, 119.8f, 1.10f, g);
            }
        }

        // 4 piano E-F / B-C registration dots (4 . 3 . 4 . 3 . 2)
        for (float dotX : kRegDotCenterX) {
            cv.drawOpticalBead(dotX, 128.0f, 0.38f, 0.95f);
        }

        // Bottom 16 keys with tall centered LED slits driven by live varibright zone levels
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float cx = kWhiteFirstCenterX + static_cast<float>(i) * kWhitePitchX;
            const float g = clampf(whiteGlow_[i], 0.0f, 1.0f);
            const auto rimGray = static_cast<uint8_t>(62.0f + 68.0f * g + 0.5f);
            fillStadiumSDF(cv, cx - kKeyHalfW, 132.0f, cx + kKeyHalfW, 158.0f, kKeyHalfW, 20u,
                           rimGray, 248u);
            fillLedSlitSDF(cv, cx, 136.2f, 153.8f, 1.15f, g);
        }
    }

    // 4. Bottom 4x4 Zone Brackets & Labels (y = 167..192)
    if (y1 > 166 && y0 < 194) {
        for (size_t z = 0u; z < 4u; ++z) {
            const float xL =
                (kWhiteFirstCenterX + static_cast<float>(z * 4u) * kWhitePitchX) - 3.2f;
            const float xR =
                (kWhiteFirstCenterX + static_cast<float>(z * 4u + 3u) * kWhitePitchX) + 3.2f;
            const float cxF = 0.5f * (xL + xR);
            const float zoneGlow = 0.25f * (whiteGlow_[z * 4u] + whiteGlow_[z * 4u + 1u] +
                                            whiteGlow_[z * 4u + 2u] + whiteGlow_[z * 4u + 3u]);
            const float brkLum = 56.0f + 92.0f * clampf(zoneGlow, 0.0f, 1.0f);
            cv.drawHorizontalFilament(xL, xR, 170.0f, brkLum, 0.58f, true);
            cv.drawVerticalSlit(xL, 167.4f, 170.0f, brkLum);
            cv.drawVerticalSlit(xR, 167.4f, 170.0f, brkLum);
            cv.drawSansCentered(kFontSansSmall, static_cast<int>(cxF + 0.5f), 175,
                                spec.bottomZones[z], 222u, 0);
        }
    }

    // 5. Swappable-Model & Performance Guidance Footer (y = 202..230)
    if (y1 > 200 && y0 < 234) {
        cv.drawSansCentered(kFontSansSmall, 120, 202, "TURN SELECT TO SWAP MODEL", 148u, 0);
        cv.drawSansCentered(kFontSansSmall, 120, 217, spec.actionHint, 118u, 0);
    }
}

} // namespace ruido
