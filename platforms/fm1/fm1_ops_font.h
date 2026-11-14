#pragma once

#include "ui/FontSansAA.h"

#include <cstddef>
#include <cstdint>

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
        src.lineHeight, static_cast<uint8_t>(GlyphCount), {}, {}, {}, {}, {}, {}, {}};
    size_t writeOff = 0u;
    for (size_t idx = 0u; idx < GlyphCount; ++idx) {
        const auto uc = static_cast<unsigned char>(charset[idx]);
        const GlyphMetrics gm = src.metrics[static_cast<size_t>(uc - 32u)];
        const size_t nPix = static_cast<size_t>(gm.width) * static_cast<size_t>(gm.height);
        const size_t nBytes2 = (nPix + 3u) >> 2u;
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
        s.lineHeight, s.glyphCount, s.chars,    s.widths,  s.heights,
        s.xOffsets,   s.yOffsets,   s.advances, s.bitmaps,
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

__attribute__((always_inline)) inline constexpr uint16_t
computeOpsCompactGlyphOffset(const OpsCompactFontView& font, int gi) noexcept {
    uint16_t off = 0u;
    for (int j = 0; j < gi; ++j) {
        const uint16_t nPix =
            static_cast<uint16_t>(font.widths[j]) * static_cast<uint16_t>(font.heights[j]);
        off = static_cast<uint16_t>(off + ((nPix + 3u) >> 2u));
    }
    return off;
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
inline constexpr char kOpsLdrMediumCharset[] = " %-.0123456789ABCDEORSUWYacdefghilmnoprstuwy";

static_assert(opsCharsetStrictlySorted(kOpsLdrBoldCharset),
              "Loader Bold charset must be strictly sorted ASCII");
static_assert(opsCharsetStrictlySorted(kOpsLdrMediumCharset),
              "Loader Medium charset must be strictly sorted ASCII");
static_assert(opsCharsetCovers(kOpsLdrBoldCharset, "FIRMWARE UPDATE"), "Missing loader Bold glyph");
static_assert(opsCharsetCovers(kOpsLdrMediumCharset,
                               "READY ERROR 0123456789% Waiting for USB installer Connect USB-C to "
                               "continue Writing firmware... Do not turn off power Update "
                               "interrupted Reconnect USB and retry Restarts "
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

__attribute__((always_inline)) inline uint8_t blendOpsNib(uint8_t bg, uint8_t fg,
                                                          uint8_t nib) noexcept {
    if (nib == 0u) {
        return bg;
    }
    if (nib >= 15u) {
        return fg;
    }
    const uint32_t w256 = static_cast<uint32_t>(nib) * 17u + (static_cast<uint32_t>(nib) >> 3u);
    return static_cast<uint8_t>(((256u - w256) * bg + w256 * fg + 128u) >> 8u);
}

__attribute__((always_inline)) inline uint16_t packOpsMonoRgb565(uint8_t lum,
                                                                 bool bigEndian) noexcept {
    const uint32_t t = static_cast<uint32_t>(lum) * 31u + 128u;
    const uint32_t g5 = (t + (t >> 8u)) >> 8u;
    const uint32_t g6 = (g5 << 1u) | (g5 >> 4u);
    const auto rgb = static_cast<uint16_t>((g5 << 11u) | (g6 << 5u) | g5);
    return bigEndian ? static_cast<uint16_t>((rgb >> 8u) | (rgb << 8u)) : rgb;
}

__attribute__((always_inline)) inline int measureOpsCompactText(const OpsCompactFontView& font,
                                                                const char* str) noexcept {
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
    const int gi0 =
        (tx == kOpsLeftX) ? findOpsCompactGlyphIdx(font, static_cast<unsigned char>(str[0])) : -1;
    int penX = (gi0 >= 0) ? (tx - static_cast<int>(font.xOffsets[gi0])) : tx;
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
            const uint16_t baseOff = computeOpsCompactGlyphOffset(font, gi);
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
    size_t idx0 = 0u;
    bool ac0 = false;
    const unsigned char uc0 = decodeSansGlyphChar(str, idx0, ac0);
    int penX = (tx == kOpsLeftX)
                   ? (tx - static_cast<int>(font.metrics[static_cast<size_t>(uc0 - 32u)].xOffset))
                   : tx;
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

} // namespace ruido::fm1
