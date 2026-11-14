#pragma once

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

/* First-party monochrome operational screen states for M-VAVE FM-1 hardware lifecycle events:
 *   - Stage 1 live USB-MIDI package check & RAM loader staging (`ota_show`)
 *   - Stage 2 standalone RAM flash writer @ 0x01C0A800 (`loader.bin`)
 *   - Power-on `Hold OCT-` / `bootguard` USB Rescue (`recovery_main`)
 *   - Unhandled CPU0 hardware exception (`fm1_fault`) & ROM DFU jump (`UBOOT`)
 *
 * Uses zero global C++ constructors, zero heap, zero floating-point, and only 240 B of stack,
 * making it 100% safe inside pre-boot `Hold OCT-` rescue, standalone RAM `loader.bin`, and CPU
 * fault handlers with hardware stack guards enabled. */
typedef enum {
    RUIDO_OPS_OTA_CHECK = 0,       /* Stage 1 steps 1..2: Checking package... */
    RUIDO_OPS_OTA_PREPARE = 1,     /* Stage 1 steps 3..4: Preparing update... */
    RUIDO_OPS_OTA_ENTER = 2,       /* Stage 1 step 5: Entering update mode... */
    RUIDO_OPS_OTA_VERIFIED = 3,    /* Stage 1 step 9 (code == 1): Package verified */
    RUIDO_OPS_OTA_CANCELLED = 4,   /* Stage 1 step 9 (code != 1): Update cancelled */
    RUIDO_OPS_LDR_WAITING = 5,     /* Stage 2 idle / power-loss resume: Waiting for USB installer */
    RUIDO_OPS_LDR_WRITING = 6,     /* Stage 2 active flash write: Writing firmware... (0%..100%) */
    RUIDO_OPS_LDR_FAILED = 7,      /* Stage 2 interrupted: Update interrupted */
    RUIDO_OPS_RESCUE_READY = 8,    /* Hold OCT- / bootguard rescue (flash_ok != 0) */
    RUIDO_OPS_RESCUE_BADFLASH = 9, /* Hold OCT- / bootguard rescue (flash_ok == 0) */
    RUIDO_OPS_SYSTEM_FAULT = 10,   /* Unhandled CPU0 exception (vec, pc, rets) */
    RUIDO_OPS_UBOOT = 11           /* Mask-ROM USB DFU jump */
} RuidoOpsKind;

typedef struct {
    uint8_t kind;        /* RuidoOpsKind */
    uint8_t percent;     /* 0..100 for RUIDO_OPS_LDR_WRITING */
    uint16_t bar_px;     /* 0..220 fill (WRITING) or 0..184 glider offset (WAITING) */
    uint32_t fault_vec;  /* Exception vector (FAULT) or bootguard diag_code (RESCUE) */
    uint32_t fault_pc;   /* Faulting PC (fm1_crash.pc) */
    uint32_t fault_rets; /* Return address RETS (fm1_crash.rets) */
    uint32_t fault_emu;  /* CPU0 EMU->MSG (fm1_crash.emu) */
    uint32_t fault_dbg;  /* Shared DBG->MSG (fm1_crash.dbg) */
    uint32_t fault_sp;   /* Faulting SP | (vec << 24) */
} RuidoOpsState;

/* Render `strip_h` scanlines starting at `y0` into `out_strip_be` (big-endian RGB565 for ST7789
 * SPI1 DMA) or `out_strip_le` (native little-endian RGB565 for host unit tests / previews). */
void ruido_ops_render_strip_be(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_be);
void ruido_ops_render_strip_le(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_le);

#ifdef __cplusplus
} // extern "C"

#include "fm1_ops_font.h"

namespace ruido::fm1 {

__attribute__((always_inline)) inline void drawOpsBarRow(const RuidoOpsState& st, int py,
                                                         uint8_t* rowLum) noexcept {
    if (py < kOpsBarY || py >= kOpsBarY + kOpsBarH) {
        return;
    }
    const auto kind = static_cast<RuidoOpsKind>(st.kind);
    const bool hasBar = (kind == RUIDO_OPS_OTA_CHECK || kind == RUIDO_OPS_OTA_PREPARE ||
                         kind == RUIDO_OPS_OTA_ENTER || kind == RUIDO_OPS_LDR_WAITING ||
                         kind == RUIDO_OPS_LDR_WRITING || kind == RUIDO_OPS_LDR_FAILED);
    if (!hasBar) {
        return;
    }
    for (int x = 0; x < kOpsBarW; ++x) {
        rowLum[kOpsLeftX + x] = kOpsLumTrack;
    }
    if (kind == RUIDO_OPS_LDR_WRITING) {
        const int fill =
            (st.bar_px > static_cast<uint16_t>(kOpsBarW)) ? kOpsBarW : static_cast<int>(st.bar_px);
        for (int x = 0; x < fill; ++x) {
            rowLum[kOpsLeftX + x] = kOpsLumWhite;
        }
    } else if (kind != RUIDO_OPS_LDR_FAILED) {
        constexpr uint8_t kShoulder[6] = {78u, 108u, 138u, 168u, 198u, 228u};
        const int rawOff = (kind == RUIDO_OPS_OTA_CHECK)     ? 24
                           : (kind == RUIDO_OPS_OTA_PREPARE) ? 92
                           : (kind == RUIDO_OPS_OTA_ENTER)   ? 160
                                                             : static_cast<int>(st.bar_px);
        const int off = (rawOff > kOpsGliderMaxOffset) ? kOpsGliderMaxOffset : rawOff;
        for (int i = 0; i < kOpsGliderW; ++i) {
            uint8_t lum = kOpsLumWhite;
            if (i < 6) {
                lum = kShoulder[i];
            } else if (i >= kOpsGliderW - 6) {
                lum = kShoulder[(kOpsGliderW - 1) - i];
            }
            rowLum[kOpsLeftX + off + i] = lum;
        }
    }
}

__attribute__((always_inline)) inline void renderOpsScreenStripLdrImpl(const RuidoOpsState& st,
                                                                       uint16_t y0, uint16_t stripH,
                                                                       uint16_t* outStrip,
                                                                       bool bigEndian) noexcept {
    if (outStrip == nullptr || stripH == 0u) {
        return;
    }
    const OpsCompactFontView boldView = makeOpsCompactFontView(kOpsLdrBoldStorage);
    const OpsCompactFontView medView = makeOpsCompactFontView(kOpsLdrMediumStorage);

    const char* hdrRight = "READY";
    uint8_t hdrRightLum = kOpsLumGray;
    const char* line1 = "Waiting for USB installer";
    const char* line2 = "Connect USB-C to continue";
    const char* foot2 = "Restarts when complete";
    char pctBuf[8]{};

    if (static_cast<RuidoOpsKind>(st.kind) == RUIDO_OPS_LDR_WRITING) {
        formatOpsPercent(st.percent, pctBuf);
        hdrRight = pctBuf;
        hdrRightLum = kOpsLumWhite;
        line1 = "Writing firmware...";
        line2 = "Do not turn off power";
    } else if (static_cast<RuidoOpsKind>(st.kind) == RUIDO_OPS_LDR_FAILED) {
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Update interrupted";
        line2 = "Reconnect USB and retry";
        foot2 = "Stays in update mode";
    }

    const int hdrRightX = kOpsRightX - measureOpsCompactText(medView, hdrRight);
    struct OpsLdrLine {
        int16_t x;
        int16_t y;
        uint8_t lum;
        const char* str;
    };
    const OpsLdrLine medLines[4] = {
        {static_cast<int16_t>(hdrRightX), kOpsHeaderY, hdrRightLum, hdrRight},
        {kOpsLeftX, 46, kOpsLumWhite, line1},
        {kOpsLeftX, 70, kOpsLumGray, line2},
        {kOpsLeftX, 158, kOpsLumGray, foot2},
    };

#if defined(RUIDO_LDR_SUBSET)
    (void)bigEndian;
    constexpr bool kPackBe = true;
#else
    const bool kPackBe = bigEndian;
#endif

    for (uint16_t r = 0u; r < stripH; ++r) {
        const int py = static_cast<int>(y0 + r);
        uint16_t* dstRow = outStrip + static_cast<size_t>(r) * static_cast<size_t>(kOpsScreenW);
        if (py < 0 || py >= kOpsScreenH) {
            for (int x = 0; x < kOpsScreenW; ++x) {
                dstRow[x] = 0u;
            }
            continue;
        }
        uint8_t rowLum[kOpsScreenW]{};
        drawOpsCompactRow(boldView, kOpsLeftX, kOpsHeaderY, "FIRMWARE UPDATE", kOpsLumWhite, py,
                          rowLum);
        for (size_t i = 0u; i < 4u; ++i) {
            drawOpsCompactRow(medView, medLines[i].x, medLines[i].y, medLines[i].str,
                              medLines[i].lum, py, rowLum);
        }
        drawOpsBarRow(st, py, rowLum);
        for (int x = 0; x < kOpsScreenW; ++x) {
            dstRow[x] = packOpsMonoRgb565(rowLum[x], kPackBe);
        }
    }
}

inline void renderOpsScreenStripFullImpl(const RuidoOpsState& st, uint16_t y0, uint16_t stripH,
                                         uint16_t* outStrip, bool bigEndian) noexcept {
    if (outStrip == nullptr || stripH == 0u) {
        return;
    }

    // Compile-time margin & header clearance proof across all operational screen strings
    static_assert(kOpsLeftX + measureSansText(kFontSansBold, "FIRMWARE UPDATE", 0) + 10 <=
                      kOpsRightX - measureSansText(kFontSansMedium, "STAGE", 0),
                  "FIRMWARE UPDATE + STAGE header collision");
    static_assert(kOpsLeftX + measureSansText(kFontSansBold, "FIRMWARE UPDATE", 0) + 10 <=
                      kOpsRightX - measureSansText(kFontSansMedium, "ERROR", 0),
                  "FIRMWARE UPDATE + ERROR header collision");
    static_assert(kOpsLeftX + measureSansText(kFontSansBold, "FIRMWARE UPDATE", 0) + 10 <=
                      kOpsRightX - measureSansText(kFontSansMedium, "100%", 0),
                  "FIRMWARE UPDATE + 100% header collision");
    static_assert(kOpsLeftX + measureSansText(kFontSansBold, "USB BOOTLOADER", 0) + 10 <=
                      kOpsRightX - measureSansText(kFontSansMedium, "DFU", 0),
                  "USB BOOTLOADER + DFU header collision");
    static_assert(kOpsLeftX + measureSansText(kFontSansMedium, "Waiting for USB installer", 0) <=
                      kOpsRightX,
                  "Waiting line exceeds right margin");
    static_assert(kOpsLeftX + measureSansText(kFontSansMedium, "Connect USB-C to continue", 0) <=
                      kOpsRightX,
                  "Connect USB-C line exceeds right margin");
    static_assert(kOpsLeftX + measureSansText(kFontSansMedium, "Entering ROM bootloader...", 0) <=
                      kOpsRightX,
                  "ROM bootloader line exceeds right margin");
    static_assert(kOpsLeftX + measureSansText(kFontSansMedium, "Restarting...", 0) <= kOpsRightX,
                  "Fault primary line exceeds right margin");
    static_assert(kOpsLeftX +
                          measureSansText(kFontSansMedium, "Rescue: Hold OCT- at power-on", 0) <=
                      kOpsRightX,
                  "Fault rescue line exceeds right margin");
    static_assert(56 + measureSansText(kFontSansMedium, "0xFFFFFFFF", 0) <= kOpsRightX,
                  "Fault hex readout exceeds right margin");

    const char* hdrLeft = "FIRMWARE UPDATE";
    const char* hdrRight = "READY";
    uint8_t hdrRightLum = kOpsLumGray;
    const char* line1 = "";
    const char* line2 = "";
    const char* line3 = "";
    const char* foot2 = "";
    char pctBuf[8]{};

    switch (static_cast<RuidoOpsKind>(st.kind)) {
    case RUIDO_OPS_OTA_CHECK:
        hdrRight = "STAGE";
        line1 = "Checking package...";
        line2 = "Do not disconnect USB";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_OTA_PREPARE:
        hdrRight = "STAGE";
        line1 = "Preparing update...";
        line2 = "Do not disconnect USB";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_OTA_ENTER:
        hdrRight = "STAGE";
        line1 = "Entering update mode...";
        line2 = "Do not disconnect USB";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_OTA_VERIFIED:
        hdrRight = "OK";
        line1 = "Package verified";
        line2 = "Firmware unchanged";
        break;
    case RUIDO_OPS_OTA_CANCELLED:
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Update cancelled";
        line2 = "Firmware unchanged";
        break;
    case RUIDO_OPS_LDR_WAITING:
    default:
        hdrRight = "READY";
        line1 = "Waiting for USB installer";
        line2 = "Connect USB-C to continue";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_LDR_WRITING:
        formatOpsPercent(st.percent, pctBuf);
        hdrRight = pctBuf;
        hdrRightLum = kOpsLumWhite;
        line1 = "Writing firmware...";
        line2 = "Do not turn off power";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_LDR_FAILED:
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Update interrupted";
        line2 = "Reconnect USB and retry";
        foot2 = "Stays in update mode";
        break;
    case RUIDO_OPS_RESCUE_READY:
        hdrLeft = "USB RESCUE";
        hdrRight = "READY";
        line1 = "Ready for USB recovery";
        line2 = "Connect USB-C to computer";
        line3 = "Run firmware installer";
        foot2 = "Power off to exit";
        break;
    case RUIDO_OPS_RESCUE_BADFLASH:
        hdrLeft = "USB RESCUE";
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Unsupported flash chip";
        line2 = "Flash JEDEC check failed";
        foot2 = "Power off to exit";
        break;
    case RUIDO_OPS_SYSTEM_FAULT:
        hdrLeft = "SYSTEM FAULT";
        hdrRight = "v0.1.0";
        line1 = "Restarting...";
        foot2 = "Rescue: Hold OCT- at power-on";
        break;
    case RUIDO_OPS_UBOOT:
        hdrLeft = "USB BOOTLOADER";
        hdrRight = "DFU";
        line1 = "Entering ROM bootloader...";
        break;
    }

    const bool isFault = (static_cast<RuidoOpsKind>(st.kind) == RUIDO_OPS_SYSTEM_FAULT);
    const bool isRescueDiag =
        (static_cast<RuidoOpsKind>(st.kind) == RUIDO_OPS_RESCUE_READY && st.fault_vec != 0u);
    const bool showDiagTable = isFault || isRescueDiag;
    if (isRescueDiag) {
        line2 = "";
        line3 = "";
    }
    const char* const diagLabels[6] = {
        isFault ? "VEC" : "BG", "PC", "RET", "EMU", "DBG", "SP",
    };
    const uint32_t diagVals[6] = {
        st.fault_vec, st.fault_pc, st.fault_rets, st.fault_emu, st.fault_dbg, st.fault_sp,
    };
    char diagHex[6][12]{};
    if (showDiagTable) {
        for (size_t i = 0u; i < 6u; ++i) {
            formatOpsHex32(diagVals[i], diagHex[i]);
        }
    }
    const int primaryY = showDiagTable ? 36 : 46;
    const int foot2Y = showDiagTable ? 196 : 158;

    for (uint16_t r = 0u; r < stripH; ++r) {
        const int py = static_cast<int>(y0 + r);
        uint16_t* dstRow = outStrip + static_cast<size_t>(r) * static_cast<size_t>(kOpsScreenW);
        if (py < 0 || py >= kOpsScreenH) {
            for (int x = 0; x < kOpsScreenW; ++x) {
                dstRow[x] = 0u;
            }
            continue;
        }

        uint8_t rowLum[kOpsScreenW]{};
        drawOpsSansRow(kFontSansBold, kOpsLeftX, kOpsHeaderY, hdrLeft, kOpsLumWhite, py, rowLum);
        drawOpsSansRightRow(kFontSansMedium, kOpsRightX, kOpsHeaderY, hdrRight, hdrRightLum, py,
                            rowLum);

        drawOpsSansRow(kFontSansMedium, kOpsLeftX, primaryY, line1, kOpsLumWhite, py, rowLum);
        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 70, line2, kOpsLumGray, py, rowLum);
        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 94, line3, kOpsLumGray, py, rowLum);

        if (showDiagTable) {
            for (size_t i = 0u; i < 6u; ++i) {
                const int ry = 64 + static_cast<int>(i) * 20;
                drawOpsSansRow(kFontSansMedium, kOpsLeftX, ry, diagLabels[i], kOpsLumGray, py,
                               rowLum);
                drawOpsSansRow(kFontSansMedium, 56, ry, diagHex[i], kOpsLumWhite, py, rowLum);
            }
        } else {
            drawOpsBarRow(st, py, rowLum);
        }

        drawOpsSansRow(kFontSansMedium, kOpsLeftX, foot2Y, foot2, kOpsLumGray, py, rowLum);

        for (int x = 0; x < kOpsScreenW; ++x) {
            dstRow[x] = packOpsMonoRgb565(rowLum[x], bigEndian);
        }
    }
}

__attribute__((always_inline)) inline void renderOpsScreenStripImpl(const RuidoOpsState& st,
                                                                    uint16_t y0, uint16_t stripH,
                                                                    uint16_t* outStrip,
                                                                    bool bigEndian) noexcept {
#if defined(RUIDO_LDR_SUBSET)
    renderOpsScreenStripLdrImpl(st, y0, stripH, outStrip, bigEndian);
#else
    renderOpsScreenStripFullImpl(st, y0, stripH, outStrip, bigEndian);
#endif
}

} // namespace ruido::fm1
#endif
