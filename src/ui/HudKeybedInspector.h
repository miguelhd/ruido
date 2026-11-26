#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/KeyPunchSpec.h"
#include "core/RhythmMath.h"
#include "ui/FontSansAA.h"
#include "ui/HudColumnText.h"
#include "ui/HudFormat.h"
#include "ui/MonoCanvas.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

// Anti-aliased SDF rounded stadium enclosure / silicone key capsule (zero 90-degree corners).
inline void fillKeybedStadiumSDF(const MonoStripCanvas& cv, float x0, float y0, float x1, float y1,
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

    const auto interiorA8 = static_cast<uint8_t>(alphaScale + 0.5f);
    const float fillBase = static_cast<float>(fillGray);
    const float borderDelta = static_cast<float>(borderGray) - fillBase;
    const float innerInset = r - 1.15f;

    auto evalEdgePixel = [&](uint8_t* row, int px, float dy, float qy2,
                             bool straightSideRow) noexcept {
        const float dx = absf(static_cast<float>(px) - cx) - hx;
        float d;
        if (straightSideRow) {
            d = ((dx > dy) ? dx : dy) - r;
        } else if (dx <= 0.0f) {
            d = dy - r;
        } else {
            const float q2 = dx * dx + qy2;
            d = ((q2 > 1.0e-6f) ? (q2 * fastInvSqrt(q2)) : 0.0f) - r;
        }
        if (d >= 0.65f) {
            return;
        }
        const float outerCov = clampf(0.55f - d, 0.0f, 1.0f);
        const float rimMix = clampf(1.15f + d, 0.0f, 1.0f);
        const float grayF = fillBase + borderDelta * rimMix;
        const auto a8 = static_cast<uint8_t>(outerCov * alphaScale + 0.5f);
        if (a8 > 0u) {
            row[px] = blendLum8(row[px], static_cast<uint8_t>(grayF + 0.5f), a8);
        }
    };

    for (int py = py0; py <= py1; ++py) {
        const float dy = absf(static_cast<float>(py) - cy) - hy;
        const float qy = (dy > 0.0f) ? dy : 0.0f;
        const float qy2 = qy * qy;
        const bool straightSideRow = (dy <= 0.0f);
        const float flatHalfSpan =
            straightSideRow ? (hx + innerInset) : ((dy <= innerInset) ? hx : -1.0f);
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;

        if (flatHalfSpan > 0.5f) {
            int in0 = static_cast<int>(cx - flatHalfSpan + 0.999f);
            int in1 = static_cast<int>(cx + flatHalfSpan);
            if (in0 < px0) {
                in0 = px0;
            }
            if (in1 > px1) {
                in1 = px1;
            }
            if (in0 <= in1) {
                for (int px = px0; px < in0; ++px) {
                    evalEdgePixel(row, px, dy, qy2, straightSideRow);
                }
                if (interiorA8 > 0u) {
                    for (int px = in0; px <= in1; ++px) {
                        row[px] = blendLum8(row[px], fillGray, interiorA8);
                    }
                }
                for (int px = in1 + 1; px <= px1; ++px) {
                    evalEdgePixel(row, px, dy, qy2, straightSideRow);
                }
                continue;
            }
        }

        for (int px = px0; px <= px1; ++px) {
            evalEdgePixel(row, px, dy, qy2, straightSideRow);
        }
    }
}

// Anti-aliased SDF vertical diffuser line for analog pressure pads (with upward squeeze fill).
inline void fillKeybedSlitSDF(const MonoStripCanvas& cv, float cx, float yTop, float yBot,
                              float slitHalfW, float glow01, float pressFill01 = -1.0f,
                              bool activeKey = true) noexcept {
    if (cv.y1 <= static_cast<int>(yTop) - 4 || cv.y0 > static_cast<int>(yBot) + 4) {
        return;
    }
    const float g = activeKey ? clampf(glow01, 0.0f, 1.0f) : 0.0f;
    const float lifted = g * (1.58f - 0.58f * g);
    const auto baseSlitGray =
        activeKey ? static_cast<uint8_t>(42.0f + 213.0f * clampf(lifted, 0.0f, 1.0f) + 0.5f) : 24u;

    const bool hasPress = activeKey && (pressFill01 >= 0.0f);
    const float pClamped = hasPress ? clampf(pressFill01, 0.0f, 1.0f) : 0.0f;
    const float yFillTop = yBot - (yBot - yTop) * pClamped;
    const float effBloom = hasPress ? clampf(0.25f + 0.75f * pClamped, 0.0f, 1.0f) : g;

    const float reach = (effBloom > 0.20f) ? 3.2f : 1.5f;
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
        const float dy = absf(fy - cy) - hy;
        const bool shaftRow = (dy <= 0.0f);
        const float qy2 = shaftRow ? 0.0f : (dy * dy);
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        const uint8_t rowGray =
            hasPress ? static_cast<uint8_t>(
                           54.0f + 201.0f * clampf(fy - yFillTop + 0.65f, 0.0f, 1.0f) + 0.5f)
                     : baseSlitGray;
        const float rowGlow = hasPress ? ((fy >= yFillTop - 0.5f) ? effBloom : 0.0f) : effBloom;
        for (int px = px0; px <= px1; ++px) {
            const float dx = absf(static_cast<float>(px) - cx);
            float d;
            if (shaftRow) {
                d = dx - r;
            } else {
                const float q2 = dx * dx + qy2;
                d = ((q2 > 1.0e-6f) ? (q2 * fastInvSqrt(q2)) : 0.0f) - r;
            }
            if (d < 0.65f) {
                const float cov = clampf(0.58f - d, 0.0f, 1.0f);
                const auto a8 = static_cast<uint8_t>(cov * 250.0f * masterF + 0.5f);
                if (a8 > 0u) {
                    row[px] = blendLum8(row[px], rowGray, a8);
                }
            } else if (rowGlow > 0.22f && d < 2.8f) {
                const float halo = fastExpNeg(d * d * 0.62f) * (rowGlow - 0.18f) * 0.52f;
                const auto h8 =
                    static_cast<uint8_t>(clampf(halo, 0.0f, 1.0f) * 255.0f * masterF + 0.5f);
                if (h8 >= 2u) {
                    row[px] = blendLum8(row[px], 255u, h8);
                }
            }
        }
    }
}

// Anti-aliased SDF centered circular indicator dot for digital switches (distinct from pad lines).
inline void fillKeybedSwitchDotSDF(const MonoStripCanvas& cv, float cx, float cy, float glow01,
                                   bool activeKey = true) noexcept {
    constexpr float kDotRadius = 1.85f;
    const float g = activeKey ? clampf(glow01, 0.0f, 1.0f) : 0.0f;
    const float reach = (g > 0.35f) ? 4.4f : 2.6f;
    int py0 = static_cast<int>(cy - reach);
    int py1 = static_cast<int>(cy + reach) + 1;
    if (py0 < cv.y0) {
        py0 = cv.y0;
    }
    if (py1 > cv.y1 - 1) {
        py1 = cv.y1 - 1;
    }
    if (py0 > py1) {
        return;
    }
    int px0 = static_cast<int>(cx - reach);
    int px1 = static_cast<int>(cx + reach) + 1;
    if (px0 < 0) {
        px0 = 0;
    }
    if (px1 > static_cast<int>(kDisplayWidth) - 1) {
        px1 = static_cast<int>(kDisplayWidth) - 1;
    }

    uint8_t dotGray = 26u;
    if (activeKey) {
        if (g <= 0.03f) {
            dotGray = 52u;
        } else {
            const float lifted = g * (1.45f - 0.45f * g);
            dotGray = static_cast<uint8_t>(80.0f + 175.0f * clampf(lifted, 0.0f, 1.0f) + 0.5f);
        }
    }
    const float masterF = static_cast<float>(cv.masterAlpha8) * (1.0f / 255.0f);

    for (int py = py0; py <= py1; ++py) {
        const float dy = static_cast<float>(py) - cy;
        const float dy2 = dy * dy;
        uint8_t* row = cv.lum + static_cast<size_t>(py - cv.y0) * kDisplayWidth;
        for (int px = px0; px <= px1; ++px) {
            const float dx = static_cast<float>(px) - cx;
            const float q2 = dx * dx + dy2;
            const float dist = (q2 > 1.0e-6f) ? (q2 * fastInvSqrt(q2)) : 0.0f;
            const float d = dist - kDotRadius;
            if (d < 0.65f) {
                const float cov = clampf(0.58f - d, 0.0f, 1.0f);
                const auto a8 = static_cast<uint8_t>(cov * 252.0f * masterF + 0.5f);
                if (a8 > 0u) {
                    row[px] = blendLum8(row[px], dotGray, a8);
                }
            } else if (g > 0.35f && d < 2.6f) {
                const float halo = fastExpNeg(d * d * 0.68f) * (g - 0.30f) * 0.58f;
                const auto h8 =
                    static_cast<uint8_t>(clampf(halo, 0.0f, 1.0f) * 255.0f * masterF + 0.5f);
                if (h8 >= 2u) {
                    row[px] = blendLum8(row[px], 255u, h8);
                }
            }
        }
    }
}

inline void formatSignedOctaveReadout(char* dst, size_t cap, const char* signPrefix,
                                      float mag) noexcept {
    const auto tenths = static_cast<uint32_t>(clampf(mag, 0.0f, 2.0f) * 10.0f + 0.5f);
    size_t rp = appendStr(dst, 0u, cap, signPrefix);
    rp = appendUInt(dst, rp, cap, tenths / 10u);
    rp = appendStr(dst, rp, cap, ".");
    rp = appendUInt(dst, rp, cap, tenths % 10u);
    appendStr(dst, rp, cap, " OCT");
}

inline void formatKeyPunchReadout(const HudState& hud, char* leftMain, size_t leftMainCap,
                                  char* leftSuffix, size_t leftSuffixCap, char* rightText,
                                  size_t rightCap, bool& outShowBar, float& outBarNorm,
                                  bool* outShowSceneDots = nullptr) noexcept {
    outShowBar = false;
    outBarNorm = 0.0f;
    if (outShowSceneDots != nullptr) {
        *outShowSceneDots = false;
    }

    const KeyPunchSpec& spec = keyPunchSpec(hud.activePunch);
    if (spec.readoutKind == PunchReadoutKind::Snapshot) {
        formatSnapshotHeader(hud, leftMain, leftMainCap, leftSuffix, leftSuffixCap, rightText,
                             rightCap, outShowBar, outBarNorm, outShowSceneDots);
        return;
    }
    if (spec.readoutKind == PunchReadoutKind::ModelPreview) {
        appendStr(leftMain, 0u, leftMainCap, keyboardModelTitle(hud.keyboardModel));
        appendStr(rightText, 0u, rightCap, "PREVIEW");
        return;
    }

    appendStr(leftMain, 0u, leftMainCap, spec.title);
    if (spec.showOscSuffix) {
        const uint32_t oscNum = static_cast<uint32_t>((hud.activePunchVoice & 3u) + 1u);
        const size_t sp = appendStr(leftSuffix, 0u, leftSuffixCap, " OSC ");
        appendUInt(leftSuffix, sp, leftSuffixCap, oscNum);
    }
    outShowBar = evalKeyPunchHeaderBar(hud, outBarNorm);

    switch (spec.readoutKind) {
    case PunchReadoutKind::Percent:
        formatPercent(rightText, rightCap, outBarNorm);
        break;
    case PunchReadoutKind::SoloState:
        appendStr(rightText, 0u, rightCap, (hud.activePunchValue > 0.5f) ? "SOLOED" : "UNSOLOED");
        break;
    case PunchReadoutKind::MuteState:
        appendStr(rightText, 0u, rightCap, (hud.activePunchValue > 0.5f) ? "MUTED" : "UNMUTED");
        break;
    case PunchReadoutKind::HoldState:
        appendStr(rightText, 0u, rightCap,
                  hud.keyHoldLatched ? ((hud.keyHoldCount > 0u) ? "LATCHED" : "ARMED")
                                     : "RELEASED");
        break;
    case PunchReadoutKind::Holding:
        appendStr(rightText, 0u, rightCap, "HOLDING");
        break;
    case PunchReadoutKind::SubOctave:
        formatSignedOctaveReadout(rightText, rightCap, "-", -hud.activePunchValue);
        break;
    case PunchReadoutKind::PosOctave:
        formatSignedOctaveReadout(rightText, rightCap, "+", hud.activePunchValue);
        break;
    case PunchReadoutKind::JustRatio: {
        const size_t rIdx = (hud.activePunchVoice < kNumJustRatios) ? hud.activePunchVoice : 12u;
        size_t rp = appendUInt(rightText, 0u, rightCap, kJustRatioTable[rIdx].num);
        rp = appendStr(rightText, rp, rightCap, ":");
        appendUInt(rightText, rp, rightCap, kJustRatioTable[rIdx].den);
        break;
    }
    case PunchReadoutKind::RawCutoffHz:
        formatCompactHz(rightText, rightCap, clampf(hud.activePunchValue, 20.0f, 28000.0f));
        break;
    case PunchReadoutKind::StutterHz:
        formatCompactHz(rightText, rightCap, computeStutterRateHz(outBarNorm));
        break;
    case PunchReadoutKind::FoldStages:
        formatMultiplierX(rightText, rightCap, computeWavefoldStages(outBarNorm));
        break;
    case PunchReadoutKind::CrushHz:
        formatCompactHz(rightText, rightCap, computeCrushClockHz(outBarNorm));
        break;
    case PunchReadoutKind::TurboMult:
        formatMultiplierX(rightText, rightCap, fastExp2(5.0f * outBarNorm));
        break;
    case PunchReadoutKind::ClockDiv:
        appendStr(rightText, 0u, rightCap,
                  clockDivisionLabel(selectVoiceClockDivIndex(hud.activePunchVoice, outBarNorm)));
        break;
    case PunchReadoutKind::EuclidHits: {
        const uint8_t hits = kEuclidHitCounts[selectEuclidTierIndex(outBarNorm)];
        const size_t rp = appendUInt(rightText, 0u, rightCap, hits);
        appendStr(rightText, rp, rightCap, "/16");
        break;
    }
    case PunchReadoutKind::TempoBpm: {
        const auto bpmInt =
            static_cast<uint32_t>(clampf(hud.effectiveClockBpm * computeTempoMultiplier(outBarNorm),
                                         kMinInternalClockBpm, kMaxEffectiveClockBpm) +
                                  0.5f);
        const size_t rp = appendUInt(rightText, 0u, rightCap, bpmInt);
        appendStr(rightText, rp, rightCap, hud.midiClockSynced ? " BPM EXT" : " BPM");
        break;
    }
    case PunchReadoutKind::TapBpm:
        formatClockBpm(rightText, rightCap, hud.effectiveClockBpm, hud.midiClockSynced, false);
        break;
    case PunchReadoutKind::ModelPreview:
    case PunchReadoutKind::Snapshot:
        break;
    }
}

// Concise plain-English 1-line effect description (`<= 35` chars) replacing static bottom guidance
// on `SEL` and `SHIFT` while any active key or `SHIFT` control is held/focused.
inline constexpr const char* keyPunchGuidanceDesc(KeyPunchKind kind) noexcept {
    switch (kind) {
    case KeyPunchKind::SpotlightSolo:
        return "ISOLATE SINGLE OSCILLATOR";
    case KeyPunchKind::OverloadSurge:
        return "PUSH VCA OVERDRIVE & RAIL COUPLING";
    case KeyPunchKind::SubDrop:
        return "DROP ALL OSCILLATORS UP TO 2 OCTAVES";
    case KeyPunchKind::RatioMagnet:
        return "LOCK OSCS TO PURE HARMONIC RATIOS";
    case KeyPunchKind::DubLpfDrop:
        return "SWEEP RESONANT LOW-PASS DOWN";
    case KeyPunchKind::HighPass:
        return "SWEEP RESONANT HIGH-PASS BASS CUT";
    case KeyPunchKind::MonoFocus:
        return "COLLAPSE ALL OSCS TO UNISON CENTER";
    case KeyPunchKind::DetuneChorus:
        return "THICKEN CHORUS DETUNE ACROSS OSCS";
    case KeyPunchKind::PolySpread:
        return "WIDEN STEREO PAN & DUTY OUTWARD";
    case KeyPunchKind::StasisFreeze:
        return "FREEZE OSCILLOSCOPE TRACE";
    case KeyPunchKind::VoiceCut:
        return "SILENCE OSCILLATOR OUTPUT";
    case KeyPunchKind::VoiceFormant:
        return "SWEEP DUAL RESONANT VOCAL FILTER";
    case KeyPunchKind::VoiceSiren:
        return "BEND PITCH UP TO +2 OCTAVES";
    case KeyPunchKind::VoiceDuty:
        return "PINCH PULSE DUTY TOWARD NARROW";
    case KeyPunchKind::SnapshotRecall:
        return "RECALL SCENE (1-4)";
    case KeyPunchKind::RailStarve:
        return "STARVE SHARED POWER RAIL & DRIVE";
    case KeyPunchKind::RingXor:
        return "CMOS XOR RING-MOD ADJACENT PAIRS";
    case KeyPunchKind::LfoTurbo:
        return "ACCELERATE ALL LFOS UP TO 32X";
    case KeyPunchKind::FilterScream:
        return "PUSH FILTER RESONANCE TO SCREAM";
    case KeyPunchKind::HardSync:
        return "HARD-SYNC OSCS 2-4 TO OSC 1 MASTER";
    case KeyPunchKind::PhaseScatter:
        return "CHAOTIC PITCH, DUTY & PAN SCATTER";
    case KeyPunchKind::SubXor:
        return "FLIP-FLOP SUB-OCTAVE XOR CRUNCH";
    case KeyPunchKind::FeedbackRoar:
        return "SELF-OSCILLATING FILTER FEEDBACK";
    case KeyPunchKind::VoiceXMod:
        return "NEIGHBOR PIN FM & LOGIC XOR";
    case KeyPunchKind::VoiceFold:
        return "MULTI-STAGE TRIANGLE WAVEFOLDER";
    case KeyPunchKind::VoiceCrush:
        return "LO-FI SAMPLE & HOLD CLOCK CRUSH";
    case KeyPunchKind::VoiceStutter:
        return "RHYTHMIC SQUARE-WAVE AUDIO CHOP";
    case KeyPunchKind::VoiceDivide:
        return "CLOCK-DIVIDED RHYTHM MULTIPLIER";
    case KeyPunchKind::VoiceEuclid:
        return "16-STEP EUCLIDEAN POLYRHYTHM GATE";
    case KeyPunchKind::RhythmPump:
        return "4-ON-THE-FLOOR SIDECHAIN DUCKING";
    case KeyPunchKind::RhythmChase:
        return "ROUND-ROBIN 1-2-3-4 VOICE CHASE";
    case KeyPunchKind::RhythmTempo:
        return "MULTIPLY CLOCK SPEED UP TO 4X";
    case KeyPunchKind::RhythmPluck:
        return "PERCUSSIVE LOW-PASS FILTER PLUCK";
    case KeyPunchKind::RhythmShort:
        return "TIGHTEN GATE LENGTH TO STACCATO";
    case KeyPunchKind::RhythmSwing:
        return "DELAY EVEN 16THS FOR SHUFFLE GROOVE";
    case KeyPunchKind::RhythmPingPong:
        return "ALTERNATE HITS LEFT & RIGHT STEREO";
    case KeyPunchKind::RhythmRoll:
        return "FAST X2-X8 RATCHET FILL BURST";
    case KeyPunchKind::TapTempo:
        return "TAP REPEATEDLY TO SET MASTER BPM";
    case KeyPunchKind::KeyHoldSwitch:
        return "LATCH HELD PRESSURE KEYS HANDS-FREE";
    default:
        return "";
    }
}

// Evaluates [0, 1] live finger squeeze for the full-width bottom pressure bar on `SEL` and `SHIFT`
// (strictly true for the 20 analog pressure keys; false for digital switches).
inline bool evalKeybedBottomPressureBar(const HudState& hud, float& outPress01) noexcept {
    outPress01 = 0.0f;
    switch (hud.activePunch) {
    case KeyPunchKind::None:
    case KeyPunchKind::SpotlightSolo:
    case KeyPunchKind::VoiceCut:
    case KeyPunchKind::KeyHoldSwitch:
    case KeyPunchKind::TapTempo:
    case KeyPunchKind::StasisFreeze:
        return false;
    case KeyPunchKind::SnapshotRecall: {
        bool anyOccHeld = false;
        uint8_t maxQ8 = 0u;
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const size_t wIdx = kSceneWhiteKeyBase + s;
            if (((hud.heldWhiteMask >> wIdx) & 1u) != 0u &&
                ((hud.snapshotOccupiedMask >> s) & 1u) != 0u) {
                anyOccHeld = true;
                if (hud.whitePressureQ8[wIdx] > maxQ8) {
                    maxQ8 = hud.whitePressureQ8[wIdx];
                }
            }
        }
        if (!anyOccHeld) {
            return false;
        }
        outPress01 = static_cast<float>(maxQ8) * (1.0f / 255.0f);
        return true;
    }
    case KeyPunchKind::DubLpfDrop:
        outPress01 = static_cast<float>(hud.blackPressureQ8[static_cast<size_t>(BlackKeyId::Op4)]) *
                     (1.0f / 255.0f);
        return true;
    case KeyPunchKind::HighPass:
        outPress01 = static_cast<float>(hud.blackPressureQ8[static_cast<size_t>(BlackKeyId::Op5)]) *
                     (1.0f / 255.0f);
        return true;
    default:
        return evalKeyPunchHeaderBar(hud, outPress01);
    }
}

inline void drawKeybedBottomPressureBar(const MonoStripCanvas& cv, float press01) noexcept {
    constexpr float kBarLeftX = 12.0f;
    constexpr float kBarSpanX = 216.0f;
    constexpr float kBarY = 198.5f;
    const float p = clampf(press01, 0.0f, 1.0f);
    const float fillEnd = kBarLeftX + p * kBarSpanX;
    cv.drawHorizontalFilament(kBarLeftX, kBarLeftX + kBarSpanX, kBarY, 42.0f, 0.58f, true);
    if (p > 0.004f) {
        cv.drawHorizontalFilament(kBarLeftX, fillEnd, kBarY, 225.0f, 0.68f, false);
    }
    cv.drawOpticalBead(fillEnd, kBarY, 0.95f, 2.05f);
}

inline void measureCapsuleInkBounds(const char* mainTxt, const char* subTxt, int& outMinX,
                                    int& outMaxX, int& outMainAdvance) noexcept {
    int penX = 0;
    int minX = 1000;
    int maxX = -1000;
    auto scan = [&](const SansFont& font, const char* str) noexcept {
        if (str == nullptr) {
            return;
        }
        for (size_t i = 0u; str[i] != '\0'; ++i) {
            bool acute = false;
            const unsigned char baseUc = decodeSansGlyphChar(str, i, acute);
            const GlyphMetrics& gm = font.metrics[static_cast<size_t>(baseUc - 32u)];
            if (gm.width > 0u) {
                const int gx0 = penX + static_cast<int>(gm.xOffset);
                const int gx1 = gx0 + static_cast<int>(gm.width) - 1;
                if (gx0 < minX) {
                    minX = gx0;
                }
                if (gx1 > maxX) {
                    maxX = gx1;
                }
            }
            penX += static_cast<int>(gm.advance);
        }
    };
    scan(kFontSansBold, mainTxt);
    outMainAdvance = penX;
    scan(kFontSansMedium, subTxt);
    if (minX > maxX) {
        minX = 0;
        maxX = 0;
    }
    outMinX = minX;
    outMaxX = maxX;
}

// Momentary center stadium capsule toast (`y = 107.5..131.5`, centered at `(119.5, 119.5)`, no
// indicator dot) for LATCH ON / LATCH OFF and FINE / COARSE modifier engagement.
inline void compositeCapsuleToastStrip(const MonoStripCanvas& cv, CapsuleToastKind kind) noexcept {
    if (kind == CapsuleToastKind::None || cv.y0 >= 135 || cv.y1 <= 105) {
        return;
    }
    const char* mainTxt = "";
    const char* subTxt = "";
    uint8_t mainGray = 255u;
    uint8_t subGray = 196u;
    uint8_t borderGray = 184u;
    switch (kind) {
    case CapsuleToastKind::LatchOn:
        mainTxt = "LATCH";
        subTxt = " ON";
        break;
    case CapsuleToastKind::LatchOff:
        mainTxt = "LATCH";
        subTxt = " OFF";
        mainGray = 168u;
        subGray = 122u;
        borderGray = 82u;
        break;
    case CapsuleToastKind::Fine:
        mainTxt = "FINE";
        break;
    case CapsuleToastKind::Coarse:
        mainTxt = "COARSE";
        break;
    case CapsuleToastKind::None:
    default:
        return;
    }
    int minX = 0;
    int maxX = 0;
    int mainAdv = 0;
    measureCapsuleInkBounds(mainTxt, subTxt, minX, maxX, mainAdv);
    const int inkW = maxX - minX + 1;
    const int inkLeft = (static_cast<int>(kDisplayWidth) - inkW) / 2;
    const int tx = inkLeft - minX;
    const float cx = static_cast<float>(inkLeft) + 0.5f * static_cast<float>(inkW - 1);
    const float halfW = 0.5f * static_cast<float>(inkW - 1) + 12.5f;
    fillKeybedStadiumSDF(cv, cx - halfW, 107.5f, cx + halfW, 131.5f, 12.0f, 13u, borderGray, 236u);
    cv.drawSans(kFontSansBold, tx, 112, mainTxt, mainGray);
    if (subTxt[0] != '\0') {
        cv.drawSans(kFontSansMedium, tx + mainAdv, 112, subTxt, subGray);
    }
}

} // namespace ruido
