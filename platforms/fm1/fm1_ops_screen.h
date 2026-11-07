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
    uint32_t fault_vec;  /* Exception vector for RUIDO_OPS_SYSTEM_FAULT */
    uint32_t fault_pc;   /* Faulting PC for RUIDO_OPS_SYSTEM_FAULT */
    uint32_t fault_rets; /* Return address RETS for RUIDO_OPS_SYSTEM_FAULT */
} RuidoOpsState;

/* Render `strip_h` scanlines starting at `y0` into `out_strip_be` (big-endian RGB565 for ST7789
 * SPI1 DMA) or `out_strip_le` (native little-endian RGB565 for host unit tests / previews). */
void ruido_ops_render_strip_be(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_be);
void ruido_ops_render_strip_le(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_le);

#ifdef __cplusplus
} // extern "C"

#include "ui/FontSansAA.h"

namespace ruido::fm1 {

inline constexpr int kOpsScreenW = 240;
inline constexpr int kOpsScreenH = 240;
inline constexpr int kOpsLeftX = 10;
inline constexpr int kOpsRightX = 230;
inline constexpr int kOpsHeaderY = 6;
inline constexpr int kOpsBarY = 112;
inline constexpr int kOpsBarH = 4;
inline constexpr int kOpsBarW = 220;
inline constexpr int kOpsGliderW = 36;
inline constexpr int kOpsGliderMaxOffset = kOpsBarW - kOpsGliderW; // 184

inline constexpr uint8_t kOpsLumWhite = 255u;
inline constexpr uint8_t kOpsLumGray = 174u;
inline constexpr uint8_t kOpsLumTrack = 48u;

constexpr size_t opsStrLen(const char* s) noexcept {
    size_t n = 0u;
    while (s[n] != '\0') {
        ++n;
    }
    return n;
}

constexpr bool opsCharsetStrictlySorted(const char* charset) noexcept {
    if (charset[0] == '\0') {
        return false;
    }
    for (size_t i = 0u; charset[i] != '\0'; ++i) {
        const auto uc = static_cast<unsigned char>(charset[i]);
        if (uc < 32u || uc > 126u) {
            return false;
        }
        if (i > 0u && static_cast<unsigned char>(charset[i - 1u]) >= uc) {
            return false;
        }
    }
    return true;
}

constexpr size_t computeOpsSubsetBitmapBytes(const SansFont& src, const char* charset) noexcept {
    size_t total = 0u;
    for (size_t i = 0u; charset[i] != '\0'; ++i) {
        const auto uc = static_cast<unsigned char>(charset[i]);
        if (uc >= 32u && uc <= 126u) {
            const GlyphMetrics gm = src.metrics[uc - 32u];
            total += (static_cast<size_t>(gm.width) * static_cast<size_t>(gm.height) + 3u) >> 2u;
        }
    }
    return (total > 0u) ? total : 1u;
}

// Sparse Structure-of-Arrays (SoA) 2-bit anti-aliased (`{0, 5, 10, 15}`) font subset for
// standalone RAM `loader.bin` (`0x01C0A800`): halves bitmap storage and groups metric runs so
// LZ4 `ota.bin` keeps `ruido.fwsc` well below the 600 KiB ceiling.
template <size_t GlyphCount, size_t BitmapBytes> struct OpsCompactFontStorage {
    uint8_t lineHeight;
    uint8_t glyphCount;
    uint16_t dataOffsets[GlyphCount];
    char chars[GlyphCount];
    uint8_t widths[GlyphCount];
    uint8_t heights[GlyphCount];
    int8_t xOffsets[GlyphCount];
    int8_t yOffsets[GlyphCount];
    uint8_t advances[GlyphCount];
    uint8_t bitmaps[BitmapBytes];
};

struct OpsCompactFontView {
    uint8_t lineHeight;
    uint8_t glyphCount;
    const uint16_t* dataOffsets;
    const char* chars;
    const uint8_t* widths;
    const uint8_t* heights;
    const int8_t* xOffsets;
    const int8_t* yOffsets;
    const uint8_t* advances;
    const uint8_t* bitmaps;
};

template <size_t GlyphCount, size_t BitmapBytes>
constexpr OpsCompactFontStorage<GlyphCount, BitmapBytes>
makeOpsCompactFont(const SansFont& src, const char* charset) noexcept {
    OpsCompactFontStorage<GlyphCount, BitmapBytes> out = {
        src.lineHeight, static_cast<uint8_t>(GlyphCount), {}, {}, {}, {}, {}, {}, {}, {}};
    size_t writeOff = 0u;
    for (size_t idx = 0u; idx < GlyphCount; ++idx) {
        const auto uc = static_cast<unsigned char>(charset[idx]);
        const GlyphMetrics gm = src.metrics[static_cast<size_t>(uc - 32u)];
        const size_t nPix = static_cast<size_t>(gm.width) * static_cast<size_t>(gm.height);
        const size_t nBytes2 = (nPix + 3u) >> 2u;
        out.dataOffsets[idx] = static_cast<uint16_t>(writeOff);
        out.chars[idx] = charset[idx];
        out.widths[idx] = gm.width;
        out.heights[idx] = gm.height;
        out.xOffsets[idx] = gm.xOffset;
        out.yOffsets[idx] = gm.yOffset;
        out.advances[idx] = gm.advance;
        for (size_t p = 0u; p < nPix; ++p) {
            const uint8_t srcByte = src.bitmaps[gm.dataOffset + (p >> 1u)];
            const uint8_t nib = ((p & 1u) == 0u) ? (srcByte >> 4u) : (srcByte & 0x0Fu);
            const uint8_t q2 = (nib >= 13u) ? 3u : (nib >= 8u) ? 2u : (nib >= 3u) ? 1u : 0u;
            out.bitmaps[writeOff + (p >> 2u)] |= static_cast<uint8_t>(q2 << ((p & 3u) << 1u));
        }
        writeOff += nBytes2;
    }
    return out;
}

template <size_t GlyphCount, size_t BitmapBytes>
constexpr OpsCompactFontView
makeOpsCompactFontView(const OpsCompactFontStorage<GlyphCount, BitmapBytes>& s) noexcept {
    return OpsCompactFontView{
        s.lineHeight, s.glyphCount, s.dataOffsets, s.chars,    s.widths,
        s.heights,    s.xOffsets,   s.yOffsets,    s.advances, s.bitmaps,
    };
}

constexpr int findOpsCompactGlyphIdx(const OpsCompactFontView& font, unsigned char uc) noexcept {
    int lo = 0;
    int hi = static_cast<int>(font.glyphCount) - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) >> 1;
        const auto mc = static_cast<unsigned char>(font.chars[mid]);
        if (mc == uc) {
            return mid;
        }
        if (mc < uc) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return -1;
}

constexpr bool opsCharsetCovers(const char* charset, const char* text) noexcept {
    for (size_t i = 0u; text[i] != '\0'; ++i) {
        bool found = false;
        for (size_t j = 0u; charset[j] != '\0'; ++j) {
            if (charset[j] == text[i]) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

inline constexpr char kOpsLdrBoldCharset[] = " ADEFIMPRTUW";
inline constexpr char kOpsLdrMediumCharset[] = " %-.0123456789ABCDEORSUWYacdefghilmnoprstuvwy";

static_assert(opsCharsetStrictlySorted(kOpsLdrBoldCharset),
              "Loader Bold charset must be strictly sorted ASCII");
static_assert(opsCharsetStrictlySorted(kOpsLdrMediumCharset),
              "Loader Medium charset must be strictly sorted ASCII");
static_assert(opsCharsetCovers(kOpsLdrBoldCharset, "FIRMWARE UPDATE"), "Missing loader Bold glyph");
static_assert(opsCharsetCovers(kOpsLdrMediumCharset,
                               "READY ERROR 0123456789% Waiting for USB installer Connect USB-C to "
                               "continue Writing firmware... Do not turn off power Update "
                               "interrupted Reconnect USB and retry Scenes 1-4 preserved Restarts "
                               "when complete Stays in update mode"),
              "Missing loader Medium glyph");

inline constexpr auto kOpsLdrBoldStorage =
    makeOpsCompactFont<opsStrLen(kOpsLdrBoldCharset),
                       computeOpsSubsetBitmapBytes(kFontSansBold, kOpsLdrBoldCharset)>(
        kFontSansBold, kOpsLdrBoldCharset);
inline constexpr auto kOpsLdrMediumStorage =
    makeOpsCompactFont<opsStrLen(kOpsLdrMediumCharset),
                       computeOpsSubsetBitmapBytes(kFontSansMedium, kOpsLdrMediumCharset)>(
        kFontSansMedium, kOpsLdrMediumCharset);

inline uint8_t blendOpsNib(uint8_t bg, uint8_t fg, uint8_t nib) noexcept {
    if (nib == 0u) {
        return bg;
    }
    if (nib >= 15u) {
        return fg;
    }
    const uint32_t w256 = static_cast<uint32_t>(nib) * 17u + (static_cast<uint32_t>(nib) >> 3u);
    return static_cast<uint8_t>(((256u - w256) * bg + w256 * fg + 128u) >> 8u);
}

inline uint16_t packOpsMonoRgb565(uint8_t lum, bool bigEndian) noexcept {
    const uint32_t t = static_cast<uint32_t>(lum) * 31u + 128u;
    const uint32_t g5 = (t + (t >> 8u)) >> 8u;
    const uint32_t g6 = (g5 << 1u) | (g5 >> 4u);
    const auto rgb = static_cast<uint16_t>((g5 << 11u) | (g6 << 5u) | g5);
    return bigEndian ? static_cast<uint16_t>((rgb >> 8u) | (rgb << 8u)) : rgb;
}

inline int measureOpsCompactText(const OpsCompactFontView& font, const char* str) noexcept {
    int w = 0;
    for (size_t i = 0u; str[i] != '\0'; ++i) {
        const int gi = findOpsCompactGlyphIdx(font, static_cast<unsigned char>(str[i]));
        if (gi >= 0) {
            w += static_cast<int>(font.advances[gi]);
        }
    }
    return w;
}

inline void drawOpsCompactRow(const OpsCompactFontView& font, int tx, int ty, const char* str,
                              uint8_t gray, int py, uint8_t* rowLum) noexcept {
    if (str == nullptr || str[0] == '\0' || py < ty ||
        py >= ty + static_cast<int>(font.lineHeight)) {
        return;
    }
    int penX = tx;
    for (size_t i = 0u; str[i] != '\0'; ++i) {
        const int gi = findOpsCompactGlyphIdx(font, static_cast<unsigned char>(str[i]));
        if (gi < 0) {
            continue;
        }
        const int gx0 = penX + static_cast<int>(font.xOffsets[gi]);
        const int gy0 = ty + static_cast<int>(font.yOffsets[gi]);
        const int gw = static_cast<int>(font.widths[gi]);
        const int gh = static_cast<int>(font.heights[gi]);
        if (gw > 0 && gh > 0 && py >= gy0 && py < gy0 + gh) {
            const int ry = py - gy0;
            const uint16_t baseOff = font.dataOffsets[gi];
            for (int rx = 0; rx < gw; ++rx) {
                const int px = gx0 + rx;
                if (px >= 0 && px < kOpsScreenW) {
                    const size_t pixIdx = static_cast<size_t>(ry * gw + rx);
                    const uint8_t packed = font.bitmaps[baseOff + (pixIdx >> 2u)];
                    const uint8_t q2 = (packed >> ((pixIdx & 3u) << 1u)) & 0x03u;
                    if (q2 != 0u) {
                        rowLum[px] = blendOpsNib(rowLum[px], gray, static_cast<uint8_t>(q2 * 5u));
                    }
                }
            }
        }
        penX += static_cast<int>(font.advances[gi]);
    }
}

inline void drawOpsSansRow(const SansFont& font, int tx, int ty, const char* str, uint8_t gray,
                           int py, uint8_t* rowLum) noexcept {
    if (str == nullptr || str[0] == '\0' || py < ty ||
        py >= ty + static_cast<int>(font.lineHeight)) {
        return;
    }
    int penX = tx;
    for (size_t i = 0u; str[i] != '\0'; ++i) {
        bool hasAcute = false;
        const unsigned char baseUc = decodeSansGlyphChar(str, i, hasAcute);
        const GlyphMetrics& gm = font.metrics[static_cast<size_t>(baseUc - 32u)];
        const int gx0 = penX + static_cast<int>(gm.xOffset);
        const int gy0 = ty + static_cast<int>(gm.yOffset);
        const int gw = static_cast<int>(gm.width);
        const int gh = static_cast<int>(gm.height);
        if (gw > 0 && gh > 0 && py >= gy0 && py < gy0 + gh) {
            const int ry = py - gy0;
            for (int rx = 0; rx < gw; ++rx) {
                const int px = gx0 + rx;
                if (px >= 0 && px < kOpsScreenW) {
                    const size_t nibIdx = static_cast<size_t>(ry * gw + rx);
                    const uint8_t packed = font.bitmaps[gm.dataOffset + (nibIdx >> 1u)];
                    const uint8_t nib = ((nibIdx & 1u) == 0u) ? (packed >> 4u) : (packed & 0x0Fu);
                    if (nib != 0u) {
                        rowLum[px] = blendOpsNib(rowLum[px], gray, nib);
                    }
                }
            }
        }
        if (hasAcute) {
            constexpr uint8_t kAcute3x3[3][3] = {{0u, 10u, 15u}, {4u, 15u, 6u}, {13u, 5u, 0u}};
            const int ax0 = penX + (static_cast<int>(gm.advance) >> 1) - 1;
            const int ayRaw = gy0 - 4;
            const int ay0 = (ayRaw < ty) ? ty : ayRaw;
            if (py >= ay0 && py < ay0 + 3) {
                const int ary = py - ay0;
                for (int arx = 0; arx < 3; ++arx) {
                    const int px = ax0 + arx;
                    const uint8_t nib = kAcute3x3[ary][arx];
                    if (nib != 0u && px >= 0 && px < kOpsScreenW) {
                        rowLum[px] = blendOpsNib(rowLum[px], gray, nib);
                    }
                }
            }
        }
        penX += static_cast<int>(gm.advance);
    }
}

inline void drawOpsSansRightRow(const SansFont& font, int rightX, int ty, const char* str,
                                uint8_t gray, int py, uint8_t* rowLum) noexcept {
    if (str == nullptr || str[0] == '\0' || py < ty ||
        py >= ty + static_cast<int>(font.lineHeight)) {
        return;
    }
    const int w = measureSansText(font, str, 0);
    drawOpsSansRow(font, rightX - w, ty, str, gray, py, rowLum);
}

inline void formatOpsPercent(uint8_t pct, char out[8]) noexcept {
    const uint32_t p = (pct > 100u) ? 100u : static_cast<uint32_t>(pct);
    size_t n = 0u;
    if (p >= 100u) {
        out[n++] = '1';
        out[n++] = '0';
        out[n++] = '0';
    } else {
        const uint32_t tens = (p * 205u) >> 11u;
        const uint32_t ones = p - tens * 10u;
        if (tens > 0u) {
            out[n++] = static_cast<char>('0' + tens);
        }
        out[n++] = static_cast<char>('0' + ones);
    }
    out[n++] = '%';
    out[n] = '\0';
}

inline void formatOpsHex32(uint32_t val, char out[12]) noexcept {
    constexpr char kHex[17] = "0123456789ABCDEF";
    out[0] = '0';
    out[1] = 'x';
    for (uint32_t i = 0u; i < 8u; ++i) {
        out[2u + i] = kHex[(val >> (28u - 4u * i)) & 0x0Fu];
    }
    out[10] = '\0';
}

inline void drawOpsBarRow(const RuidoOpsState& st, int py, uint8_t* rowLum) noexcept {
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
    } else if (kind == RUIDO_OPS_LDR_WAITING) {
        constexpr uint8_t kShoulder[6] = {78u, 108u, 138u, 168u, 198u, 228u};
        const int off = (st.bar_px > static_cast<uint16_t>(kOpsGliderMaxOffset))
                            ? kOpsGliderMaxOffset
                            : static_cast<int>(st.bar_px);
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

inline void renderOpsScreenStripLdrImpl(const RuidoOpsState& st, uint16_t y0, uint16_t stripH,
                                        uint16_t* outStrip, bool bigEndian) noexcept {
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
    const OpsLdrLine medLines[5] = {
        {static_cast<int16_t>(hdrRightX), kOpsHeaderY, hdrRightLum, hdrRight},
        {kOpsLeftX, 46, kOpsLumWhite, line1},
        {kOpsLeftX, 69, kOpsLumGray, line2},
        {kOpsLeftX, 152, kOpsLumGray, "Scenes 1-4 preserved"},
        {kOpsLeftX, 175, kOpsLumGray, foot2},
    };

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
        for (size_t i = 0u; i < 5u; ++i) {
            drawOpsCompactRow(medView, medLines[i].x, medLines[i].y, medLines[i].str,
                              medLines[i].lum, py, rowLum);
        }
        drawOpsBarRow(st, py, rowLum);
        for (int x = 0; x < kOpsScreenW; ++x) {
            dstRow[x] = packOpsMonoRgb565(rowLum[x], bigEndian);
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
    static_assert(kOpsLeftX + measureSansText(kFontSansMedium, "Audio muted, restarting...", 0) <=
                      kOpsRightX,
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
    const char* foot1 = "";
    const char* foot2 = "";
    char pctBuf[8]{};

    switch (static_cast<RuidoOpsKind>(st.kind)) {
    case RUIDO_OPS_OTA_CHECK:
        hdrRight = "STAGE";
        line1 = "Checking package...";
        line2 = "Do not disconnect USB";
        foot1 = "Scenes 1-4 preserved";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_OTA_PREPARE:
        hdrRight = "STAGE";
        line1 = "Preparing update...";
        line2 = "Do not disconnect USB";
        foot1 = "Scenes 1-4 preserved";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_OTA_ENTER:
        hdrRight = "STAGE";
        line1 = "Entering update mode...";
        line2 = "Do not disconnect USB";
        foot1 = "Scenes 1-4 preserved";
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
        foot1 = "Scenes 1-4 preserved";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_LDR_WRITING:
        formatOpsPercent(st.percent, pctBuf);
        hdrRight = pctBuf;
        hdrRightLum = kOpsLumWhite;
        line1 = "Writing firmware...";
        line2 = "Do not turn off power";
        foot1 = "Scenes 1-4 preserved";
        foot2 = "Restarts when complete";
        break;
    case RUIDO_OPS_LDR_FAILED:
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Update interrupted";
        line2 = "Reconnect USB and retry";
        foot1 = "Scenes 1-4 preserved";
        foot2 = "Stays in update mode";
        break;
    case RUIDO_OPS_RESCUE_READY:
        hdrLeft = "USB RESCUE";
        hdrRight = "READY";
        line1 = "Ready for USB recovery";
        line2 = "Connect USB-C to computer";
        line3 = "Run firmware installer";
        foot1 = "Audio output muted";
        foot2 = "Power off to exit";
        break;
    case RUIDO_OPS_RESCUE_BADFLASH:
        hdrLeft = "USB RESCUE";
        hdrRight = "ERROR";
        hdrRightLum = kOpsLumWhite;
        line1 = "Unsupported flash chip";
        line2 = "Flash JEDEC check failed";
        foot1 = "Audio output muted";
        foot2 = "Power off to exit";
        break;
    case RUIDO_OPS_SYSTEM_FAULT:
        hdrLeft = "SYSTEM FAULT";
        hdrRight = "v1.0";
        line1 = "Audio muted, restarting...";
        foot2 = "Rescue: Hold OCT- at power-on";
        break;
    case RUIDO_OPS_UBOOT:
        hdrLeft = "USB BOOTLOADER";
        hdrRight = "DFU";
        line1 = "Entering ROM bootloader...";
        line2 = "Audio output muted";
        break;
    }

    char vecHex[12]{};
    char pcHex[12]{};
    char retHex[12]{};
    const bool isFault = (static_cast<RuidoOpsKind>(st.kind) == RUIDO_OPS_SYSTEM_FAULT);
    if (isFault) {
        formatOpsHex32(st.fault_vec, vecHex);
        formatOpsHex32(st.fault_pc, pcHex);
        formatOpsHex32(st.fault_rets, retHex);
    }

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

        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 46, line1, kOpsLumWhite, py, rowLum);
        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 69, line2, kOpsLumGray, py, rowLum);
        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 92, line3, kOpsLumGray, py, rowLum);

        if (isFault) {
            drawOpsSansRow(kFontSansMedium, kOpsLeftX, 80, "VEC", kOpsLumGray, py, rowLum);
            drawOpsSansRow(kFontSansMedium, 56, 80, vecHex, kOpsLumGray, py, rowLum);
            drawOpsSansRow(kFontSansMedium, kOpsLeftX, 103, "PC", kOpsLumGray, py, rowLum);
            drawOpsSansRow(kFontSansMedium, 56, 103, pcHex, kOpsLumGray, py, rowLum);
            drawOpsSansRow(kFontSansMedium, kOpsLeftX, 126, "RET", kOpsLumGray, py, rowLum);
            drawOpsSansRow(kFontSansMedium, 56, 126, retHex, kOpsLumGray, py, rowLum);
        } else {
            drawOpsBarRow(st, py, rowLum);
        }

        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 152, foot1, kOpsLumGray, py, rowLum);
        drawOpsSansRow(kFontSansMedium, kOpsLeftX, 175, foot2, kOpsLumGray, py, rowLum);

        for (int x = 0; x < kOpsScreenW; ++x) {
            dstRow[x] = packOpsMonoRgb565(rowLum[x], bigEndian);
        }
    }
}

inline void renderOpsScreenStripImpl(const RuidoOpsState& st, uint16_t y0, uint16_t stripH,
                                     uint16_t* outStrip, bool bigEndian) noexcept {
#if defined(RUIDO_LDR_SUBSET)
    renderOpsScreenStripLdrImpl(st, y0, stripH, outStrip, bigEndian);
#else
    renderOpsScreenStripFullImpl(st, y0, stripH, outStrip, bigEndian);
#endif
}

} // namespace ruido::fm1
#endif
