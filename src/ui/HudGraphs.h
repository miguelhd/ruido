#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/FilterModMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"
#include "ui/GraphStyle.h"
#include "ui/MonoCanvas.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Full-screen (x = 0..239 edge-to-edge, y = 26..186) MS-20 dual-filter graph & backdrop dim
inline constexpr int kFilterXStart = 0;
inline constexpr size_t kFilterSweepPts = kDisplayWidth + 1u;
inline constexpr float kFilterLogMin = cxLog2Pos(kFilterCutoffMinHz);
inline constexpr float kFilterLogSpan = cxLog2Pos(kFilterCutoffMaxHz) - kFilterLogMin;
inline constexpr float kFilterYTop = kGraphTopY;
inline constexpr float kFilterYZeroDb = 86.0f;
inline constexpr float kFilterYBot = kGraphBaselineY;
inline constexpr float kFilterDbToPx = 3.36f;
inline constexpr float kFilterBgDimDepth = 1.0f; // 100% BG waveform hide behind active menus

inline constexpr float filterNormToScreenX(float u01) noexcept {
    return clampf(u01 * static_cast<float>(kDisplayWidth) - 0.5f, 0.0f,
                  static_cast<float>(kDisplayWidth - 1u));
}

// Full-screen (x = 0..239 edge-to-edge, y = 26..188) bipolar per-voice LFO visualization
inline constexpr float kLfoYTop = kGraphTopY;
inline constexpr float kLfoYZero = 107.0f; // Midpoint between top header and bottom strip
inline constexpr float kLfoYBot = 188.0f;
inline constexpr float kLfoMaxAmpPx = 75.0f; // +-75 px (150 px peak-to-peak) excursion
inline constexpr float kLfoLogMin = cxLog2Pos(kLfoFreqMinHz);
inline constexpr float kLfoLogSpan = cxLog2Pos(kLfoFreqMaxHz) - kLfoLogMin;

constexpr std::array<float, kFilterSweepPts> makeFilterSweepFreqs() noexcept {
    std::array<float, kFilterSweepPts> freqs{};
    for (size_t i = 0u; i < kFilterSweepPts; ++i) {
        const float u = static_cast<float>(i) * (1.0f / static_cast<float>(kFilterSweepPts - 1u));
        const float log2Freq = kFilterLogMin + u * kFilterLogSpan;
        freqs[i] = cxExpPos(log2Freq * 0.69314718f);
    }
    return freqs;
}

inline constexpr std::array<float, kFilterSweepPts> kFilterSweepFreqs = makeFilterSweepFreqs();

struct DualMs20Coeffs {
    float invEffHp;
    float dampHp2;
    float invEffLp;
    float dampLp2;
    float hpAct;
    float lpAct;
    float clampedHp;
    float clampedLp;
    float rHp;
    float rLp;
};

inline DualMs20Coeffs makeDualMs20Coeffs(float hpHz, float hpRes, float lpHz,
                                         float lpRes) noexcept {
    const float clampedHp = clampf(hpHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float clampedLp = clampf(lpHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
    const float rHp = clampf(hpRes, 0.0f, 1.0f);
    const float rLp = clampf(lpRes, 0.0f, 1.0f);

    constexpr float kButterK = 2.0f - kSqrt2;
    const float korgHp = smoothstep01((rHp - 0.60f) * (1.0f / 0.40f));
    const float korgLp = smoothstep01((rLp - 0.60f) * (1.0f / 0.40f));
    const float rawKHp =
        kButterK + rHp * (1.18f - 0.18f * rHp) * (1.84f - kButterK) + 0.07f * korgHp;
    const float rawKLp =
        kButterK + rLp * (1.18f - 0.18f * rLp) * (1.84f - kButterK) + 0.07f * korgLp;
    const float hpHfRoll =
        (clampedHp > 8200.0f) ? smoothstep01((clampedHp - 8200.0f) * (1.0f / 4000.0f)) : 0.0f;
    const float lpHfRoll =
        (clampedLp > 8200.0f) ? smoothstep01((clampedLp - 8200.0f) * (1.0f / 4000.0f)) : 0.0f;
    const float hpHfCeil = 1.55f + 0.18f * korgHp;
    const float lpHfCeil = 1.55f + 0.18f * korgLp;
    const float targetKHp =
        (rawKHp > hpHfCeil) ? (rawKHp + hpHfRoll * (hpHfCeil - rawKHp)) : rawKHp;
    const float targetKLp =
        (rawKLp > lpHfCeil) ? (rawKLp + lpHfRoll * (lpHfCeil - rawKLp)) : rawKLp;
    const float hpResWake = smoothstep01(rHp * (1.0f / 0.02f));
    const float lpResWake = smoothstep01(rLp * (1.0f / 0.02f));
    const float hpFreqWake = smoothstep01((clampedHp - kFilterCutoffMinHz) * (1.0f / 1.5f));
    const float lpFreqWake = smoothstep01((kFilterCutoffMaxHz - clampedLp) * (1.0f / 900.0f));
    const float hpAct = clampf(hpFreqWake + (1.0f - hpFreqWake) * hpResWake, 0.0f, 1.0f);
    const float lpAct = clampf(lpFreqWake + (1.0f - lpFreqWake) * lpResWake, 0.0f, 1.0f);

    const float dampHp = clampf(2.0f - (kButterK + hpAct * (targetKHp - kButterK)), 0.09f, 1.42f);
    const float dampLp = clampf(2.0f - (kButterK + lpAct * (targetKLp - kButterK)), 0.09f, 1.42f);
    const float effHp = clampedHp;
    const float effLp = clampedLp;

    return {
        1.0f / effHp, dampHp * dampHp,
        1.0f / effLp, dampLp * dampLp,
        hpAct,        lpAct,
        clampedHp,    clampedLp,
        rHp,          rLp,
    };
}

inline float evalDualMs20Db(float freq, const DualMs20Coeffs& c) noexcept {
    const float rHp = freq * c.invEffHp;
    const float rHp2 = rHp * rHp;
    const float oneMinusHp2 = 1.0f - rHp2;
    const float denHp = oneMinusHp2 * oneMinusHp2 + c.dampHp2 * rHp2;
    const float mag2Hp = (rHp2 * rHp2) / (denHp + 1.0e-9f);

    const float rLp = freq * c.invEffLp;
    const float rLp2 = rLp * rLp;
    const float oneMinusLp2 = 1.0f - rLp2;
    const float denLp = oneMinusLp2 * oneMinusLp2 + c.dampLp2 * rLp2;
    const float mag2Lp = 1.0f / (denLp + 1.0e-9f);

    return 3.0103f * fastLog2(clampf(mag2Hp * mag2Lp, 1.0e-6f, 64.0f));
}

inline int16_t dbToFilterYFixed(float db) noexcept {
    const float yPx = clampf(kFilterYZeroDb - db * kFilterDbToPx, kFilterYTop, kFilterYBot);
    return static_cast<int16_t>(yPx * 64.0f + 0.5f);
}

// Analytical MS-20 dual-filter passband graph + full-screen morphed LFO graph state & compositors.
class HudGraphs {
  public:
    static constexpr size_t kTracePoints = kDisplayWidth + 1u;

    void reset() noexcept;
    void clearImmediate() noexcept;
    void stepFadeOut(float dtClamped) noexcept;
    bool hasResidualPhase() const noexcept {
        return filterGraphPhase_ > 0.002f || lfoGraphPhase_ > 0.002f || filterBackdropDim_ > 0.002f;
    }

    void update(float dtClamped, const EngineControlState& control, const HudState& rawHud,
                const HudState& effectiveHud, bool extraCenterViewActive,
                const LfoSignalArray& lfoOutput, const LfoDriftArray& lfoDriftPacked,
                float rhythmPhase01 = 0.0f, uint8_t stutterGateMask = 0x0Fu) noexcept;

    float filterGraphPhase() const noexcept { return filterGraphPhase_; }

    float lfoGraphPhase() const noexcept { return lfoGraphPhase_; }

    float backdropDim() const noexcept { return filterBackdropDim_; }

    void compositeLfoStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                           uint16_t* linScratch, float effLfoAlpha01, uint8_t lfoAlpha8,
                           const EngineControlState& control, const HudState& hud,
                           const LfoSignalArray& lfoPhase) const noexcept;

    void compositeFilterStrip(int y0, int y1, uint16_t stripHeight, uint8_t* lumBuf,
                              uint16_t* linScratch, float effFilterAlpha01,
                              uint8_t filterAlpha8) const noexcept;

  private:
    void updateFilterGraph(float dtClamped, bool snapImmediate,
                           const std::array<ModulatedFilterParams, kNumOscillators>& mods,
                           bool unified) noexcept;
    void updateLfoGraph(float dtClamped, bool snapImmediate, const EngineControlState& control,
                        const HudState& effectiveHud, const LfoSignalArray& lfoOutput,
                        const LfoDriftArray& lfoDriftPacked) noexcept;

    std::array<std::array<int16_t, kFilterSweepPts>, kNumOscillators> filterCurvesFixed_{};
    float filterDivergenceSmooth_ = 0.0f;
    float filterHpBeadX_ = 0.0f;
    float filterHpBeadY_ = kFilterYZeroDb;
    float filterHpBeadInt_ = 0.0f;
    float filterHpBeadRad_ = 2.0f;
    float filterLpBeadX_ = static_cast<float>(kDisplayWidth - 1u);
    float filterLpBeadY_ = kFilterYZeroDb;
    float filterLpBeadInt_ = 0.0f;
    float filterLpBeadRad_ = 2.0f;
    float filterGraphPhase_ = 0.0f;
    float filterBackdropDim_ = 0.0f;

    // [0] = focused LFO slot curve, [1] = same-oscillator sibling companion curve
    std::array<std::array<int16_t, kTracePoints>, 2> lfoCurvesFixed_{};
    float lfoBeadIntensity_ = 0.0f;
    std::array<uint8_t, 2> lfoBeadBankIdx_{0u, 4u};
    float lfoSiblingWeight_ = 0.0f;
    float lfoGraphPhase_ = 0.0f;
};

} // namespace ruido
