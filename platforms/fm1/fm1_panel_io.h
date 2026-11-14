#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/RuidoController.h"
#include "fm1_led_modulator.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido::fm1 {

// High-side `DRIVE 2` (`DIR=0, OUT=1, HD0=1, HD=0`, `R_s ~ 132 ohm`) across the 1N4148 matrix
// diode + PCB series/74HC595 trace floor + silicone carbon pill:
//   - Initial dome touch-down (`ADC ~ 815 -> 650`) happens at very light finger weight.
//   - Elastomer bulk compression (`ADC ~ 650 -> 528`) carries the active musical finger squeeze,
//     bottoming out smoothly at `<= 528` (`100%`) before the PCB/74HC595 series-resistance wall.
// Any closed-key reading in `[815..1022]` clamps at `4%`; only `< 120` or open `1023` returns `-1`.
inline constexpr int32_t kAdcCarbonValidMin = 120;
inline constexpr int32_t kAdcCarbonValidMax = 1022;
inline constexpr int32_t kAdcCarbonHardPress = 528;  // Comfortable firm press -> 1.0 (100%)
inline constexpr int32_t kAdcCarbonFeatherMax = 815; // Feather contact -> 0.04 (4%)

// `GLOBAL CONFIG > Key Sensitivity` presets. `Medium` is the hardware-calibrated default
// (`[528, 815]`, `t = u^2 * (0.5 + 0.5 * u)`). Lighter presets move full pressure to a much higher
// ADC reading (little finger force) with a near-linear taper; firmer presets keep the full-scale
// point near the physical series-resistance floor (`~490..520`) and steepen the taper (`u^3`,
// `u^4`) so mid squeezes stay low. `t = a*u + b*u^2 + c*u^3 + d*u^4`, `u` = 0..1 normalized
// squeeze.
struct CarbonCurveSpec {
    int32_t hardPressAdc;
    float invSpan;
    float a, b, c, d;
};

constexpr CarbonCurveSpec makeCarbonCurve(int32_t hardPressAdc, float a, float b, float c,
                                          float d) noexcept {
    return {
        hardPressAdc, 1.0f / static_cast<float>(kAdcCarbonFeatherMax - hardPressAdc), a, b, c, d};
}

inline constexpr std::array<CarbonCurveSpec, kNumPressureTouchLevels> kCarbonCurves = {{
    makeCarbonCurve(690, 1.0f, 0.0f, 0.0f, 0.0f), // VERY HIGH: linear, maxes at a feather squeeze
    makeCarbonCurve(620, 0.5f, 0.5f, 0.0f, 0.0f), // HIGH
    makeCarbonCurve(kAdcCarbonHardPress, 0.0f, 0.5f, 0.5f, 0.0f), // MEDIUM (default, calibrated)
    makeCarbonCurve(522, 0.0f, 0.0f, 1.0f, 0.0f),                 // LOW: cubic
    makeCarbonCurve(515, 0.0f, 0.0f, 0.0f, 1.0f),                 // VERY LOW: quartic
}};

// Convert a settled 10-bit SARADC reading on PA7 (ch 0) or PA8 (ch 1) under `DRIVE 2` (`R_s = 132
// ohm`) into normalized `[0.04, 1.0]` finger pressure using the selected `Key Pressure` curve.
inline float adcToCarbonPressure01(int32_t adcRaw,
                                   PressureTouch touch = PressureTouch::Medium) noexcept {
    if (adcRaw < kAdcCarbonValidMin || adcRaw > kAdcCarbonValidMax) {
        return -1.0f;
    }
    const auto rawTouch = static_cast<uint8_t>(touch);
    const CarbonCurveSpec& c = kCarbonCurves[(rawTouch < kNumPressureTouchLevels)
                                                 ? rawTouch
                                                 : static_cast<uint8_t>(PressureTouch::Medium)];
    if (adcRaw <= c.hardPressAdc) {
        return 1.0f;
    }
    if (adcRaw >= kAdcCarbonFeatherMax) {
        return 0.04f;
    }
    const float u =
        clampf(static_cast<float>(kAdcCarbonFeatherMax - adcRaw) * c.invSpan, 0.0f, 1.0f);
    const float t = u * (c.a + u * (c.b + u * (c.c + u * c.d)));
    return clampf(0.04f + 0.96f * t, 0.04f, 1.0f);
}

// Exact hardware `ENC_*` (`platform.c` line 43) -> portable `ruido::EncoderId` remap:
//   0: ENC_SELECT    -> Select (0)
//   1: ENC_ALGORITHM -> Algorithm (2)
//   2: ENC_KNOB1     -> Knob1 (3)
//   3: ENC_KNOB2     -> Knob2 (4)
//   4: ENC_KNOB3     -> Knob3 (5)
//   5: ENC_KNOB4     -> Knob4 (6)
//   6: ENC_PRESETS   -> Presets (1)
inline constexpr std::array<EncoderId, kFm1NumEncoders> kHwEncoderMap = {{
    EncoderId::Select,
    EncoderId::Algorithm,
    EncoderId::Knob1,
    EncoderId::Knob2,
    EncoderId::Knob3,
    EncoderId::Knob4,
    EncoderId::Presets,
}};

struct NoteKeyMapping {
    bool isBlack;
    uint8_t subIndex; // 0..15 whiteKeyIndex, or 0..10 BlackKeyId
    uint8_t col;      // 74HC595 column 0..10
    int8_t saradcCh;  // 0 = PA7 (row 3), 1 = PA8 (row 4), -1 = PA6 (row 2, digital only)
    uint8_t paPinBit; // 7 for PA7, 8 for PA8, 6 for PA6
};

constexpr std::array<NoteKeyMapping, kFm1NumNoteKeys> buildNoteKeyTable() noexcept {
    std::array<NoteKeyMapping, kFm1NumNoteKeys> tbl{};
    for (uint8_t n = 0u; n < kFm1NumNoteKeys; ++n) {
        const MatrixCoord mc = kFm1KeyIdCoord[14u + n];
        const int8_t ch = (mc.row == 3u) ? 0 : ((mc.row == 4u) ? 1 : -1);
        const uint8_t pin = (mc.row == 3u) ? 7u : ((mc.row == 4u) ? 8u : 6u);
        tbl[n] = {false, 0u, mc.col, ch, pin};
    }
    for (uint8_t w = 0u; w < kNumWhiteKeys; ++w) {
        const uint8_t n = kWhiteKeyToNoteBit[w];
        tbl[n].isBlack = false;
        tbl[n].subIndex = w;
    }
    for (uint8_t b = 0u; b < kNumBlackKeys; ++b) {
        const uint8_t n = kBlackKeyToNoteBit[b];
        tbl[n].isBlack = true;
        tbl[n].subIndex = b;
    }
    return tbl;
}

inline constexpr std::array<NoteKeyMapping, kFm1NumNoteKeys> kNoteKeyTable = buildNoteKeyTable();

// `fm1_enc_take()` returns whole physical detent clicks; `EncoderMath.h` uses X4 quadrature ticks
// (`4` ticks = 1 detent, `96` ticks = 24-detent 360-degree turn).
constexpr int32_t encoderStepsToTicks(int32_t detentSteps) noexcept {
    return detentSteps * 4;
}

// Round-robin carbon-pill SARADC pressure scanner & full FM-1 panel input dispatcher.
class Fm1PanelState {
  public:
    struct AdcSampleRequest {
        uint8_t active;   // 1 = sample requested when `fm1__tick_col == col`
        uint8_t noteBit;  // 0..26
        uint8_t col;      // 0..10
        uint8_t saradcCh; // 0 (PA7) or 1 (PA8)
        uint8_t paPinBit; // 7 or 8
    };

    void reset(float initialMasterGain01) noexcept {
        prevButtonsMask_ = 0u;
        prevNotesMask_ = 0u;
        scanCursor_ = 0u;
        inputEventCount_ = 0u;
        masterInitialized_ = false;
        lastMasterNorm_ = clampf(initialMasterGain01, 0.0f, 1.0f);
        analogRowProven_[0] = false;
        analogRowProven_[1] = false;
        for (size_t i = 0u; i < kFm1NumNoteKeys; ++i) {
            rawAdc_[i].store(-1, std::memory_order_relaxed);
            adcSeeded_[i] = false;
            smoothedPressure_[i] = 0.0f;
            reportedPressure_[i] = 0.0f;
        }
        reqNote_.store(0xFFu, std::memory_order_relaxed);
    }

    // Sync `MASTER` pot (`0..4088` quadratic Q12) into `RuidoController::onMasterPot` with a small
    // ADC noise deadband so thermal LSB jitter never pops up `HudMode::MasterLevel` at idle.
    void syncMasterPot(RuidoController& ctrl, float gain01) noexcept {
        const float clamped = clampf(gain01, 0.0f, 1.0f);
        if (!masterInitialized_) {
            masterInitialized_ = true;
            lastMasterNorm_ = clamped;
            // Sync initial hardware pot level quietly and dissolve boot MasterLevel toast to idle
            ctrl.onMasterPot(clamped);
            const EngineTelemetry zeroTel{};
            for (uint32_t i = 0u; i < 8u; ++i) {
                ctrl.tick(0.25f, zeroTel);
            }
            return;
        }
        if (absf(clamped - lastMasterNorm_) >= 0.0025f) {
            lastMasterNorm_ = clamped;
            ctrl.onMasterPot(clamped);
            ++inputEventCount_;
        }
    }

    // Dispatch all 14 tactile buttons and 7 endless encoders (`steps * 4` X4 ticks).
    // Returns `true` if any button or encoder moved.
    bool pollButtonsAndEncoders(RuidoController& ctrl, uint32_t pressedEdges,
                                uint32_t releasedEdges, uint32_t curButtonsMask,
                                const int32_t encDetentSteps[kFm1NumEncoders]) noexcept {
        bool touched = false;
        const uint32_t mask14 = (1u << kFm1NumButtons) - 1u;
        const uint32_t curB = curButtonsMask & mask14;
        const uint32_t pEdges = pressedEdges & mask14;
        const uint32_t rEdges = releasedEdges & mask14;

        for (uint8_t b = 0u; b < kFm1NumButtons; ++b) {
            const uint32_t bit = 1u << b;
            const auto btnId = static_cast<ButtonId>(b);
            const bool hadP = (pEdges & bit) != 0u;
            const bool hadR = (rEdges & bit) != 0u;
            const bool isDown = (curB & bit) != 0u;
            const bool wasDown = (prevButtonsMask_ & bit) != 0u;

            if (hadP && hadR) {
                if (isDown) {
                    if (wasDown) {
                        ctrl.onButtonEvent(btnId, false);
                    }
                    ctrl.onButtonEvent(btnId, true);
                } else {
                    ctrl.onButtonEvent(btnId, true);
                    ctrl.onButtonEvent(btnId, false);
                }
                touched = true;
                ++inputEventCount_;
            } else if (!wasDown && (hadP || isDown)) {
                ctrl.onButtonEvent(btnId, true);
                touched = true;
                ++inputEventCount_;
            } else if (wasDown && (hadR || !isDown)) {
                ctrl.onButtonEvent(btnId, false);
                touched = true;
                ++inputEventCount_;
            }
        }
        prevButtonsMask_ = curB;

        for (uint8_t hwEnc = 0u; hwEnc < kFm1NumEncoders; ++hwEnc) {
            const int32_t steps = encDetentSteps[hwEnc];
            if (steps != 0) {
                ctrl.onEncoderDelta(kHwEncoderMap[hwEnc], encoderStepsToTicks(steps));
                touched = true;
                ++inputEventCount_;
            }
        }
        return touched;
    }

    // Dispatch all 27 silicone keys + continuous PA7/PA8 carbon-pill pressure updates, and arm the
    // next round-robin SARADC column sample for `TIMER5`.
    bool pollKeysAndPressure(RuidoController& ctrl, uint32_t curNotesMask,
                             uint32_t notePressEdges) noexcept {
        bool touched = false;
        touch_ = ctrl.hud().config.pressureTouch;
        const uint32_t mask27 = (1u << kFm1NumNoteKeys) - 1u;
        const uint32_t notes = curNotesMask & mask27;
        const uint32_t pEdges = notePressEdges & mask27;

        for (uint8_t n = 0u; n < kFm1NumNoteKeys; ++n) {
            const uint32_t bit = 1u << n;
            const bool isDown = (notes & bit) != 0u;
            const bool wasDown = (prevNotesMask_ & bit) != 0u;
            const bool hadPressEdge = (pEdges & bit) != 0u;
            const NoteKeyMapping& km = kNoteKeyTable[n];

            // Fast intra-poll tap (`0 -> 1 -> 0` completed between two main-loop polls)
            if (!wasDown && !isDown && hadPressEdge) {
                const float tapP = resolveTargetPressure(n, km, true);
                dispatchKeyPress(ctrl, km, tapP);
                dispatchKeyRelease(ctrl, km);
                rawAdc_[n].store(-1, std::memory_order_relaxed);
                adcSeeded_[n] = false;
                smoothedPressure_[n] = 0.0f;
                reportedPressure_[n] = 0.0f;
                touched = true;
                ++inputEventCount_;
                continue;
            }

            if (!wasDown && isDown) {
                adcSeeded_[n] = false;
                const float initP = resolveTargetPressure(n, km, true);
                smoothedPressure_[n] = initP;
                reportedPressure_[n] = initP;
                dispatchKeyPress(ctrl, km, initP);
                touched = true;
                ++inputEventCount_;
            } else if (wasDown && !isDown) {
                rawAdc_[n].store(-1, std::memory_order_relaxed);
                adcSeeded_[n] = false;
                smoothedPressure_[n] = 0.0f;
                reportedPressure_[n] = 0.0f;
                dispatchKeyRelease(ctrl, km);
                touched = true;
                ++inputEventCount_;
            } else if (isDown && km.saradcCh >= 0) {
                const bool wasSeeded = adcSeeded_[n];
                const float targetP = resolveTargetPressure(n, km, false);
                if (!wasSeeded && adcSeeded_[n]) {
                    smoothedPressure_[n] = targetP;
                } else {
                    smoothedPressure_[n] += 0.65f * (targetP - smoothedPressure_[n]);
                    if (targetP >= 0.999f && smoothedPressure_[n] >= 0.994f) {
                        smoothedPressure_[n] = 1.0f;
                    }
                }
                if (absf(smoothedPressure_[n] - reportedPressure_[n]) >= 0.002f) {
                    reportedPressure_[n] = smoothedPressure_[n];
                    dispatchKeyPressure(ctrl, km, reportedPressure_[n]);
                }
            }
        }

        prevNotesMask_ = notes;
        scheduleNextAdcNote(notes);
        return touched;
    }

    // Queried from `TIMER5` ISR once per 1.1 ms frame: returns the single held PA7/PA8 key to
    // sample when its `74HC595` column is latched low.
    AdcSampleRequest currentAdcRequest() const noexcept {
        const uint8_t n = reqNote_.load(std::memory_order_acquire);
        if (n >= kFm1NumNoteKeys) {
            return {0u, 0u, 0u, 0u, 0u};
        }
        const NoteKeyMapping& km = kNoteKeyTable[n];
        if (km.saradcCh < 0) {
            return {0u, 0u, 0u, 0u, 0u};
        }
        return {1u, n, km.col, static_cast<uint8_t>(km.saradcCh), km.paPinBit};
    }

    // Called from `TIMER5` ISR (or unit tests) to record the settled 10-bit SARADC reading.
    void recordAdcSample(uint8_t noteBit, int32_t adcVal) noexcept {
        if (noteBit < kFm1NumNoteKeys) {
            rawAdc_[noteBit].store(adcVal, std::memory_order_release);
        }
    }

    uint16_t inputEventCount() const noexcept {
        return static_cast<uint16_t>(inputEventCount_ & 0xFFFFu);
    }

  private:
    float resolveTargetPressure(uint8_t noteBit, const NoteKeyMapping& km,
                                bool isRisingEdge) noexcept {
        if (km.saradcCh < 0) {
            return 1.0f;
        }
        const int32_t adc = rawAdc_[noteBit].load(std::memory_order_acquire);
        const float mapped = adcToCarbonPressure01(adc, touch_);
        if (mapped > 0.0f) {
            analogRowProven_[static_cast<size_t>(km.saradcCh)] = true;
            adcSeeded_[noteBit] = true;
            return mapped;
        }
        // Before the first 1.1 ms scan sample arrives on proven physical silicone, start gently so
        // light touches never spike to 100% on frame 0; in digital `fm1-emu` (`analogRowProven_ ==
        // false`), immediately deliver full `1.0f` gate pressure.
        if (analogRowProven_[static_cast<size_t>(km.saradcCh)]) {
            return isRisingEdge ? 0.06f : smoothedPressure_[noteBit];
        }
        return 1.0f;
    }

    void scheduleNextAdcNote(uint32_t notes) noexcept {
        for (uint32_t step = 0u; step < kFm1NumNoteKeys; ++step) {
            const uint8_t idx = static_cast<uint8_t>((scanCursor_ + step) % kFm1NumNoteKeys);
            if (((notes >> idx) & 1u) != 0u && kNoteKeyTable[idx].saradcCh >= 0) {
                scanCursor_ = static_cast<uint8_t>((idx + 1u) % kFm1NumNoteKeys);
                reqNote_.store(idx, std::memory_order_release);
                return;
            }
        }
        reqNote_.store(0xFFu, std::memory_order_release);
    }

    static void dispatchKeyPress(RuidoController& ctrl, const NoteKeyMapping& km,
                                 float initPressure) noexcept {
        if (km.isBlack) {
            ctrl.onBlackKeyPressure(static_cast<BlackKeyId>(km.subIndex), initPressure);
        } else {
            ctrl.onWhiteKeyPressure(km.subIndex, initPressure);
        }
    }

    static void dispatchKeyPressure(RuidoController& ctrl, const NoteKeyMapping& km,
                                    float pressure) noexcept {
        if (km.isBlack) {
            ctrl.onBlackKeyPressure(static_cast<BlackKeyId>(km.subIndex), pressure);
        } else {
            ctrl.onWhiteKeyPressure(km.subIndex, pressure);
        }
    }

    static void dispatchKeyRelease(RuidoController& ctrl, const NoteKeyMapping& km) noexcept {
        if (km.isBlack) {
            ctrl.onBlackKeyEvent(static_cast<BlackKeyId>(km.subIndex), false);
        } else {
            ctrl.onWhiteKeyGate(km.subIndex, false);
        }
    }

    uint32_t prevButtonsMask_ = 0u;
    uint32_t prevNotesMask_ = 0u;
    uint8_t scanCursor_ = 0u;
    uint32_t inputEventCount_ = 0u;
    bool masterInitialized_ = false;
    float lastMasterNorm_ = 0.5f;
    bool analogRowProven_[2]{false, false};
    std::array<std::atomic<int32_t>, kFm1NumNoteKeys> rawAdc_{};
    std::array<bool, kFm1NumNoteKeys> adcSeeded_{};
    PressureTouch touch_ = PressureTouch::Medium;
    std::array<float, kFm1NumNoteKeys> smoothedPressure_{};
    std::array<float, kFm1NumNoteKeys> reportedPressure_{};
    std::atomic<uint8_t> reqNote_{0xFFu};
};

} // namespace ruido::fm1
