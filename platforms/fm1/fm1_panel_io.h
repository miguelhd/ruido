#pragma once

#include "core/ControlSurface.h"
#include "core/DspMath.h"
#include "core/RuidoController.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido::fm1 {

inline constexpr uint32_t kFm1NumCols = 11u;
inline constexpr uint32_t kFm1NumNoteKeys = 27u;
inline constexpr uint32_t kFm1NumButtons = 14u;
inline constexpr uint32_t kFm1NumEncoders = 7u;

// Physical M-VAVE FM-1 split PLAY/STOP LED coordinates on the 2x74HC595 matrix:
// - BTN_PLAY (id 12) sits at (col 10, row 2) -> drives the red/amber STOP die (`LedState::stopLed`)
// - PLAY's dedicated green LED sits at (col 8, row 1), where `FM1_KEYMAP[1][8] == -1`
inline constexpr uint8_t kPlayGreenCol = 8u;
inline constexpr uint8_t kPlayGreenRow = 1u;
inline constexpr uint8_t kStopRedCol = 10u;
inline constexpr uint8_t kStopRedRow = 2u;

// Sub-slot pulse fractions matching `fm1_input.h`:
// - Tier 0 (dark)       :  0.0 us / 95 us = 0.0000 physical duty
// - Tier 1 (fm1_led_bg) : ~2.0 us / 95 us = 0.0211 physical duty (~15% perceptual rest backlight)
// - Tier 2 (fm1_led_dim):  4.0 us / 95 us = 0.0421 physical duty (~24% perceptual pad landmark)
// - Tier 3 (fm1_led)    : 95.0 us / 95 us = 1.0000 physical duty (100% lit)
inline constexpr uint16_t kFm1LedBgPulseNs = 2000u;
inline constexpr float kLedTier1Duty = 2.0f / 95.0f;
inline constexpr float kLedTier2Duty = 4.0f / 95.0f;

// 1N4148 diode (~0.60V) + silicone carbon-pill (~150..2500 ohm) under weak high-side pull-up:
// Valid analog contact readings sit strictly inside [120, 920] on 10-bit SARADC (0.39V..2.97V).
// Readings outside [120, 920] (e.g. `0` or `1023` in digital-only `fm1-emu`) fall back to `1.0f`.
inline constexpr int32_t kAdcCarbonValidMin = 120;
inline constexpr int32_t kAdcCarbonValidMax = 920;
inline constexpr int32_t kAdcCarbonHardPress = 250; // ~0.81V: firm bottom-out -> 1.00f
inline constexpr int32_t kAdcCarbonSoftTouch = 780; // ~2.51V: initial feather contact -> ~0.18f

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

// Exact `FM1_KEYMAP` matrix coordinates `(col 0..10, row 1..4)` for `id = 0..40`
// (`0..13` = tactile buttons, `14..40` = 27 silicone keys `F3..G5`).
struct MatrixCoord {
    uint8_t col;
    uint8_t row;
};

inline constexpr std::array<MatrixCoord, 41u> kFm1KeyIdCoord = {{
    // 0..13: Panel buttons (BTN_OCT_DOWN .. BTN_REC)
    {0u, 4u},  //  0: OCT-
    {1u, 4u},  //  1: OCT+
    {6u, 1u},  //  2: FX
    {4u, 1u},  //  3: SEL
    {2u, 1u},  //  4: ENV
    {0u, 1u},  //  5: LFO
    {9u, 2u},  //  6: EDIT
    {8u, 2u},  //  7: GLO
    {7u, 1u},  //  8: HOME
    {5u, 1u},  //  9: SAVE
    {3u, 1u},  // 10: ARP
    {1u, 1u},  // 11: SEQ
    {10u, 2u}, // 12: PLAY/STOP (STOP red/amber die; PLAY green is at {8u, 1u})
    {7u, 2u},  // 13: REC
    // 14..40: 27 silicone note keys (`noteBit = id - 14`, `0 = F3 .. 26 = G5`)
    {3u, 4u},  // 14 (n0  F3  - White 0,  PA8 ch1)
    {2u, 4u},  // 15 (n1  F#3 - Black 0,  PA8 ch1)
    {5u, 4u},  // 16 (n2  G3  - White 1,  PA8 ch1)
    {4u, 4u},  // 17 (n3  G#3 - Black 1,  PA8 ch1)
    {7u, 4u},  // 18 (n4  A3  - White 2,  PA8 ch1)
    {6u, 4u},  // 19 (n5  A#3 - Black 2,  PA8 ch1)
    {8u, 4u},  // 20 (n6  B3  - White 3,  PA8 ch1)
    {9u, 4u},  // 21 (n7  C4  - White 4,  PA8 ch1)
    {10u, 4u}, // 22 (n8  C#4 - Black 3,  PA8 ch1)
    {0u, 3u},  // 23 (n9  D4  - White 5,  PA7 ch0)
    {1u, 3u},  // 24 (n10 D#4 - Black 4,  PA7 ch0)
    {2u, 3u},  // 25 (n11 E4  - White 6,  PA7 ch0)
    {3u, 3u},  // 26 (n12 F4  - White 7,  PA7 ch0)
    {4u, 3u},  // 27 (n13 F#4 - Black 5,  PA7 ch0)
    {5u, 3u},  // 28 (n14 G4  - White 8,  PA7 ch0)
    {6u, 3u},  // 29 (n15 G#4 - Black 6,  PA7 ch0)
    {7u, 3u},  // 30 (n16 A4  - White 9,  PA7 ch0)
    {8u, 3u},  // 31 (n17 A#4 - Black 7,  PA7 ch0)
    {9u, 3u},  // 32 (n18 B4  - White 10, PA7 ch0)
    {10u, 3u}, // 33 (n19 C5  - White 11, PA7 ch0)
    {0u, 2u},  // 34 (n20 C#5 - Black 8,  PA6 digital)
    {1u, 2u},  // 35 (n21 D5  - White 12, PA6 digital)
    {2u, 2u},  // 36 (n22 D#5 - Black 9,  PA6 digital)
    {3u, 2u},  // 37 (n23 E5  - White 13, PA6 digital)
    {4u, 2u},  // 38 (n24 F5  - White 14, PA6 digital)
    {6u, 2u},  // 39 (n25 F#5 - Black 10, PA6 digital)
    {5u, 2u},  // 40 (n26 G5  - White 15, PA6 digital)
}};

struct NoteKeyMapping {
    bool isBlack;
    uint8_t subIndex; // 0..15 whiteKeyIndex, or 0..10 BlackKeyId
    uint8_t col;      // 74HC595 column 0..10
    int8_t saradcCh;  // 0 = PA7 (row 3), 1 = PA8 (row 4), -1 = PA6 (row 2, digital only)
    uint8_t paPinBit; // 7 for PA7, 8 for PA8, 6 for PA6
};

inline constexpr std::array<uint8_t, kNumWhiteKeys> kWhiteKeyToNoteBit = {{
    0u,
    2u,
    4u,
    6u,
    7u,
    9u,
    11u,
    12u,
    14u,
    16u,
    18u,
    19u,
    21u,
    23u,
    24u,
    26u,
}};

inline constexpr std::array<uint8_t, kNumBlackKeys> kBlackKeyToNoteBit = {{
    1u,
    3u,
    5u,
    8u,
    10u,
    13u,
    15u,
    17u,
    20u,
    22u,
    25u,
}};

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

// Convert a settled 10-bit SARADC reading on PA7 (ch 0) or PA8 (ch 1) into normalized [0.18, 1.0]
// finger pressure. Returns `-1.0f` when `adcRaw` lies outside the physical 1N4148 diode window
// (`< 120` or `> 920`, e.g. digital-only `fm1-emu`), signaling full `1.0f` digital gate fallback.
inline float adcToCarbonPressure01(int32_t adcRaw) noexcept {
    if (adcRaw < kAdcCarbonValidMin || adcRaw > kAdcCarbonValidMax) {
        return -1.0f;
    }
    if (adcRaw <= kAdcCarbonHardPress) {
        return 1.0f;
    }
    if (adcRaw >= kAdcCarbonSoftTouch) {
        return 0.18f;
    }
    constexpr float kInvSpan = 1.0f / static_cast<float>(kAdcCarbonSoftTouch - kAdcCarbonHardPress);
    const float lin01 =
        clampf(static_cast<float>(kAdcCarbonSoftTouch - adcRaw) * kInvSpan, 0.0f, 1.0f);
    // Gentle concave response so light-to-medium finger rolls bloom smoothly before bottom-out
    const float shaped = lin01 * (0.65f + 0.35f * lin01);
    return clampf(0.18f + 0.82f * shaped, 0.18f, 1.0f);
}

// Perceptual brightness [0..1] -> physical LED duty [0..1]:
// Maps Ruido's 12%-15% rest button backlights onto `fm1_led_bg` (~2.1% duty) and 24% filter pad
// landmarks onto `fm1_led_dim` (~4.2% duty), then applies a smooth gamma ~2.2 curve up to 100%.
inline float perceptualToPhysicalLedDuty(float brightness01) noexcept {
    const float x = clampf(brightness01, 0.0f, 1.0f);
    if (x <= 1.0e-4f) {
        return 0.0f;
    }
    if (x <= 0.15f) {
        return x * (kLedTier1Duty / 0.15f);
    }
    if (x <= 0.24f) {
        const float t = (x - 0.15f) * (1.0f / 0.09f);
        return kLedTier1Duty + t * (kLedTier2Duty - kLedTier1Duty);
    }
    const float t = (x - 0.24f) * (1.0f / 0.76f);
    const float curved = t * t * (0.78f + 0.22f * t);
    return clampf(kLedTier2Duty + curved * (1.0f - kLedTier2Duty), 0.0f, 1.0f);
}

// Lock-free 42-channel multi-tier 909.1 Hz 1st-order Delta-Sigma LED modulator.
// Core 0 publishes double-buffered `(baseTier, fracQ8)` targets at ~60 Hz; `TIMER5` ISR steps pure
// integer error-diffusion accumulators once per 11-column scan frame (909.1 Hz) with golden-ratio
// spatial phase offsets across all 44 matrix nodes (zero BAM mid-scale glitch, zero coherent ground
// whine into +40 dB pedal chains, and zero low-duty flicker via `bg`/`glow` sub-slot tiers).
class Fm1LedModulator {
  public:
    void reset() noexcept {
        for (uint32_t b = 0u; b < 2u; ++b) {
            for (uint32_t c = 0u; c < kFm1NumCols; ++c) {
                for (uint32_t r = 0u; r < 4u; ++r) {
                    buffers_[b].baseTier[c][r] = 0u;
                    buffers_[b].fracQ8[c][r] = 0u;
                }
            }
        }
        for (uint32_t c = 0u; c < kFm1NumCols; ++c) {
            for (uint32_t r = 0u; r < 4u; ++r) {
                // Stagger initial phases across columns & rows so 50%-duty banks never switch in
                // unison on VDDIO
                errAcc_[c][r] = static_cast<uint8_t>((c * 73u + r * 151u + 29u) & 0xFFu);
            }
        }
        activeBuf_.store(0u, std::memory_order_relaxed);
        playGreenActive_ = 0u;
        stopRedActive_ = 0u;
    }

    void updateTargets(const LedState& leds) noexcept {
        const uint8_t curIdx = activeBuf_.load(std::memory_order_relaxed) & 1u;
        const uint8_t nextIdx = static_cast<uint8_t>(curIdx ^ 1u);
        TargetBank& dst = buffers_[nextIdx];
        for (uint32_t c = 0u; c < kFm1NumCols; ++c) {
            for (uint32_t r = 0u; r < 4u; ++r) {
                dst.baseTier[c][r] = 0u;
                dst.fracQ8[c][r] = 0u;
            }
        }

        // 1. 13 panel buttons at their keymap coordinates (`PlayStop` = 12 routes to PLAY green!)
        for (uint8_t b = 0u; b < kNumPanelButtons; ++b) {
            const float val = leds.buttonLeds[b];
            if (b == static_cast<uint8_t>(ButtonId::PlayStop)) {
                encodeNode(dst, kPlayGreenCol, kPlayGreenRow, val);
                playGreenActive_ = (val > 0.05f) ? 1u : 0u;
            } else {
                const MatrixCoord mc = kFm1KeyIdCoord[b];
                encodeNode(dst, mc.col, mc.row, val);
            }
        }

        // 2. Red/amber STOP die at BTN_PLAY's matrix slot `(col 10, row 2)`
        encodeNode(dst, kStopRedCol, kStopRedRow, leds.stopLed);
        stopRedActive_ = (leds.stopLed > 0.05f) ? 1u : 0u;

        // 3. 16 bottom white keys + 11 raised black keys (`id = 14 + noteBit`)
        for (uint8_t w = 0u; w < kNumWhiteKeys; ++w) {
            const MatrixCoord mc = kFm1KeyIdCoord[14u + kWhiteKeyToNoteBit[w]];
            encodeNode(dst, mc.col, mc.row, leds.whiteKeyLeds[w]);
        }
        for (uint8_t k = 0u; k < kNumBlackKeys; ++k) {
            const MatrixCoord mc = kFm1KeyIdCoord[14u + kBlackKeyToNoteBit[k]];
            encodeNode(dst, mc.col, mc.row, leds.blackKeyLeds[k]);
        }

        activeBuf_.store(nextIdx, std::memory_order_release);
    }

    // Pure-integer 909.1 Hz frame step (safe inside `TIMER5` ISR; ~55 integer ops per 1.1 ms).
    // Outputs packed row bitmasks (`bit 1 = row 1 .. bit 4 = row 4`) for `fm1_led`, `fm1_led_dim`,
    // and `fm1_led_bg`.
    void stepFrame909Hz(uint8_t outLit[kFm1NumCols], uint8_t outDim[kFm1NumCols],
                        uint8_t outBg[kFm1NumCols]) noexcept {
        const TargetBank& src = buffers_[activeBuf_.load(std::memory_order_acquire) & 1u];
        uint8_t litOr = 0u;
        uint8_t dimOr = 0u;
        uint8_t bgOr = 0u;

        for (uint32_t c = 0u; c < kFm1NumCols; ++c) {
            uint8_t litMask = 0u;
            uint8_t dimMask = 0u;
            uint8_t bgMask = 0u;
            for (uint32_t r = 0u; r < 4u; ++r) {
                uint8_t tier = src.baseTier[c][r];
                const uint8_t frac = src.fracQ8[c][r];
                if (frac != 0u) {
                    const uint16_t sum = static_cast<uint16_t>(errAcc_[c][r]) + frac;
                    errAcc_[c][r] = static_cast<uint8_t>(sum & 0xFFu);
                    if (sum >= 256u) {
                        ++tier;
                    }
                }
                const uint8_t bit = static_cast<uint8_t>(1u << (r + 1u));
                if (tier >= 3u) {
                    litMask |= bit;
                } else if (tier == 2u) {
                    dimMask |= bit;
                } else if (tier == 1u) {
                    bgMask |= bit;
                }
            }
            outLit[c] = litMask;
            outDim[c] = dimMask;
            outBg[c] = bgMask;
            litOr |= litMask;
            dimOr |= dimMask;
            bgOr |= bgMask;
        }
        lastSummary_ =
            static_cast<uint32_t>(litOr) | (static_cast<uint32_t>(dimOr) << 8u) |
            (static_cast<uint32_t>(bgOr) << 16u) |
            (static_cast<uint32_t>((playGreenActive_ & 1u) | ((stopRedActive_ & 1u) << 1u)) << 24u);
    }

    uint32_t lastSummary() const noexcept { return lastSummary_; }

  private:
    struct TargetBank {
        uint8_t baseTier[kFm1NumCols][4]; // 0 = dark, 1 = bg, 2 = dim, 3 = full
        uint8_t fracQ8[kFm1NumCols][4];   // [0..255] Delta-Sigma blend into `baseTier + 1`
    };

    static void encodeNode(TargetBank& dst, uint8_t col, uint8_t row1Based,
                           float brightness01) noexcept {
        if (col >= kFm1NumCols || row1Based < 1u || row1Based > 4u) {
            return;
        }
        const uint8_t r = static_cast<uint8_t>(row1Based - 1u);
        const float duty = perceptualToPhysicalLedDuty(brightness01);
        if (duty <= 1.0e-5f) {
            dst.baseTier[col][r] = 0u;
            dst.fracQ8[col][r] = 0u;
            return;
        }
        if (duty >= 0.998f) {
            dst.baseTier[col][r] = 3u;
            dst.fracQ8[col][r] = 0u;
            return;
        }

        uint8_t base = 0u;
        float t = 0.0f;
        if (duty < kLedTier1Duty) {
            base = 0u;
            t = duty * (1.0f / kLedTier1Duty);
        } else if (duty < kLedTier2Duty) {
            base = 1u;
            t = (duty - kLedTier1Duty) * (1.0f / (kLedTier2Duty - kLedTier1Duty));
        } else {
            base = 2u;
            t = (duty - kLedTier2Duty) * (1.0f / (1.0f - kLedTier2Duty));
        }
        const int32_t q = static_cast<int32_t>(clampf(t, 0.0f, 1.0f) * 256.0f + 0.5f);
        if (q >= 256) {
            dst.baseTier[col][r] = static_cast<uint8_t>(base + 1u);
            dst.fracQ8[col][r] = 0u;
        } else {
            dst.baseTier[col][r] = base;
            dst.fracQ8[col][r] = static_cast<uint8_t>((q > 0) ? q : 0);
        }
    }

    TargetBank buffers_[2]{};
    uint8_t errAcc_[kFm1NumCols][4]{};
    std::atomic<uint8_t> activeBuf_{0u};
    uint8_t playGreenActive_ = 0u;
    uint8_t stopRedActive_ = 0u;
    volatile uint32_t lastSummary_ = 0u;
};

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
                dispatchKeyPress(ctrl, km, 1.0f);
                dispatchKeyRelease(ctrl, km);
                rawAdc_[n].store(-1, std::memory_order_relaxed);
                smoothedPressure_[n] = 0.0f;
                reportedPressure_[n] = 0.0f;
                touched = true;
                ++inputEventCount_;
                continue;
            }

            if (!wasDown && isDown) {
                const float initP = resolveTargetPressure(n, km, true);
                smoothedPressure_[n] = initP;
                reportedPressure_[n] = initP;
                dispatchKeyPress(ctrl, km, initP);
                touched = true;
                ++inputEventCount_;
            } else if (wasDown && !isDown) {
                rawAdc_[n].store(-1, std::memory_order_relaxed);
                smoothedPressure_[n] = 0.0f;
                reportedPressure_[n] = 0.0f;
                dispatchKeyRelease(ctrl, km);
                touched = true;
                ++inputEventCount_;
            } else if (isDown && km.saradcCh >= 0) {
                const float targetP = resolveTargetPressure(n, km, false);
                smoothedPressure_[n] += 0.35f * (targetP - smoothedPressure_[n]);
                if (absf(smoothedPressure_[n] - reportedPressure_[n]) >= 0.008f) {
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
        const float mapped = adcToCarbonPressure01(adc);
        if (mapped > 0.0f) {
            analogRowProven_[static_cast<size_t>(km.saradcCh)] = true;
            return mapped;
        }
        // Before the first 1.1 ms scan sample arrives on proven physical silicone, start gently so
        // light touches never spike to 100% on frame 0; in digital `fm1-emu` (`analogRowProven_ ==
        // false`), immediately deliver full `1.0f` gate pressure.
        if (analogRowProven_[static_cast<size_t>(km.saradcCh)]) {
            return isRisingEdge ? 0.32f : smoothedPressure_[noteBit];
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
    std::array<float, kFm1NumNoteKeys> smoothedPressure_{};
    std::array<float, kFm1NumNoteKeys> reportedPressure_{};
    std::atomic<uint8_t> reqNote_{0xFFu};
};

} // namespace ruido::fm1
