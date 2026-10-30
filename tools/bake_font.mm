// Offline CoreText -> 4-bit grayscale anti-aliased sans-serif ROM font baker for Ruido.
// Generates `src/ui/FontSansAA.h` (100% pure portable C++17 constexpr tables, 0 bytes SRAM).
// Usage: clang++ -std=c++17 -O2 tools/bake_font.mm -framework CoreText -framework CoreGraphics \
//        -framework Foundation -o build/bake_font && ./build/bake_font > src/ui/FontSansAA.h

#import <CoreGraphics/CoreGraphics.h>
#import <CoreText/CoreText.h>
#import <Foundation/Foundation.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

struct BakedGlyph {
    uint8_t width = 0;
    uint8_t height = 0;
    int8_t xOffset = 0;
    int8_t yOffset = 0; // Offset down from top of line box
    uint8_t advance = 0;
    uint32_t dataOffset = 0;
};

struct BakedFont {
    const char* cppName;
    int lineHeight;
    std::vector<BakedGlyph> glyphs;
    std::vector<uint8_t> packedNibbles;
};

static BakedFont bakeFont(NSString* fontName, CGFloat ptSize, int canvasH, int baselineFromBottom,
                          const char* cppName) {
    BakedFont out{};
    out.cppName = cppName;
    out.lineHeight = canvasH;

    CTFontRef font = CTFontCreateWithName((__bridge CFStringRef)fontName, ptSize, nullptr);

    // Measure max digit advance so '0'..'9' use crisp tabular numeral spacing
    int tabularDigitAdv = 0;
    for (char c = '0'; c <= '9'; ++c) {
        UniChar uc = static_cast<UniChar>(c);
        CGGlyph cg = 0;
        CTFontGetGlyphsForCharacters(font, &uc, &cg, 1);
        CGSize adv{};
        CTFontGetAdvancesForGlyphs(font, kCTFontOrientationHorizontal, &cg, &adv, 1);
        const int a = static_cast<int>(std::round(adv.width));
        if (a > tabularDigitAdv) {
            tabularDigitAdv = a;
        }
    }

    const int canvasW = 32;
    std::vector<uint8_t> pixels(static_cast<size_t>(canvasW * canvasH), 0);
    CGColorSpaceRef graySpace = CGColorSpaceCreateDeviceGray();

    for (int ch = 32; ch <= 126; ++ch) {
        std::fill(pixels.begin(), pixels.end(), 0);
        CGContextRef ctx = CGBitmapContextCreate(pixels.data(), canvasW, canvasH, 8, canvasW,
                                                 graySpace, kCGImageAlphaNone);
        CGContextSetAllowsAntialiasing(ctx, true);
        CGContextSetShouldAntialias(ctx, true);
        CGContextSetAllowsFontSmoothing(ctx, false);
        CGContextSetShouldSmoothFonts(ctx, false);
        CGContextSetGrayFillColor(ctx, 1.0, 1.0);

        UniChar uc = static_cast<UniChar>(ch);
        CGGlyph cg = 0;
        const bool hasGlyph = CTFontGetGlyphsForCharacters(font, &uc, &cg, 1);
        CGSize adv{};
        if (hasGlyph) {
            CTFontGetAdvancesForGlyphs(font, kCTFontOrientationHorizontal, &cg, &adv, 1);
        }

        const bool isDigit = (ch >= '0' && ch <= '9');
        const int intAdv = isDigit ? tabularDigitAdv :
                                     std::max(3, static_cast<int>(std::round(adv.width)));
        const CGFloat drawX =
            isDigit ? (2.0 + 0.5 * (tabularDigitAdv - adv.width)) : 2.0;

        if (hasGlyph && ch != ' ') {
            CGPoint pos = CGPointMake(drawX, static_cast<CGFloat>(baselineFromBottom));
            CTFontDrawGlyphs(font, &cg, &pos, 1, ctx);
        }
        CGContextRelease(ctx);

        // Find tight non-zero bounding box in top-down coordinates
        int minX = canvasW, maxX = -1, minY = canvasH, maxY = -1;
        for (int y = 0; y < canvasH; ++y) {
            for (int x = 0; x < canvasW; ++x) {
                const uint8_t v = pixels[static_cast<size_t>(y * canvasW + x)];
                if (v >= 16u) {
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }

        BakedGlyph g{};
        g.advance = static_cast<uint8_t>(intAdv);
        g.dataOffset = static_cast<uint32_t>(out.packedNibbles.size());

        if (maxX >= minX && maxY >= minY) {
            g.width = static_cast<uint8_t>(maxX - minX + 1);
            g.height = static_cast<uint8_t>(maxY - minY + 1);
            g.xOffset = static_cast<int8_t>(minX - 2);
            g.yOffset = static_cast<int8_t>(minY);

            std::vector<uint8_t> nibs;
            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    // Slight gamma boost so thin sans-serif stems read crisply on 240x240 LCD
                    const float norm =
                        static_cast<float>(pixels[static_cast<size_t>(y * canvasW + x)]) / 255.0f;
                    const float boosted = std::pow(norm, 0.82f);
                    const int q4 =
                        std::min(15, std::max(0, static_cast<int>(boosted * 15.0f + 0.5f)));
                    nibs.push_back(static_cast<uint8_t>(q4));
                }
            }
            for (size_t i = 0; i < nibs.size(); i += 2) {
                const uint8_t hi = nibs[i];
                const uint8_t lo = (i + 1 < nibs.size()) ? nibs[i + 1] : 0u;
                out.packedNibbles.push_back(static_cast<uint8_t>((hi << 4u) | lo));
            }
        }
        out.glyphs.push_back(g);
    }

    CGColorSpaceRelease(graySpace);
    CFRelease(font);
    return out;
}

static void emitFontStruct(const BakedFont& f) {
    std::printf("inline constexpr uint8_t %sBitmaps[%zu] = {\n", f.cppName,
                f.packedNibbles.size());
    for (size_t i = 0; i < f.packedNibbles.size(); ++i) {
        if (i % 12 == 0) std::printf("    ");
        std::printf("0x%02Xu,", f.packedNibbles[i]);
        if (i % 12 == 11 || i + 1 == f.packedNibbles.size()) std::printf("\n");
        else std::printf(" ");
    }
    std::printf("};\n\n");

    std::printf("inline constexpr GlyphMetrics %sMetrics[95] = {\n", f.cppName);
    for (size_t i = 0; i < f.glyphs.size(); ++i) {
        const auto& g = f.glyphs[i];
        std::printf("    {%2uu, %2uu, %3d, %3d, %2uu, %5uu}, // '%c'\n", g.width, g.height,
                    static_cast<int>(g.xOffset), static_cast<int>(g.yOffset), g.advance,
                    g.dataOffset, (i + 32 == '\\') ? '/' : static_cast<char>(i + 32));
    }
    std::printf("};\n\n");

    std::printf("inline constexpr SansFont %s = {%uu, %sMetrics, %sBitmaps};\n\n", f.cppName,
                static_cast<unsigned>(f.lineHeight), f.cppName, f.cppName);
}

int main() {
    @autoreleasepool {
        const BakedFont small =
            bakeFont(@"HelveticaNeue-Medium", 10.0, 13, 3, "kFontSansSmall");
        const BakedFont medium =
            bakeFont(@"HelveticaNeue-Medium", 13.0, 16, 3, "kFontSansMedium");
        const BakedFont bold =
            bakeFont(@"HelveticaNeue-Bold", 13.0, 16, 3, "kFontSansBold");

        std::printf("#pragma once\n\n");
        std::printf("#include <cstddef>\n#include <cstdint>\n\n");
        std::printf("namespace ruido {\n\n");
        std::printf("// 4-bit grayscale anti-aliased Helvetica Neue Medium ROM font tables.\n");
        std::printf("// Pre-rasterized offline (`tools/bake_font.mm`); 0 bytes SRAM.\n\n");
        std::printf("struct GlyphMetrics {\n");
        std::printf("    uint8_t width;\n");
        std::printf("    uint8_t height;\n");
        std::printf("    int8_t xOffset;\n");
        std::printf("    int8_t yOffset;\n");
        std::printf("    uint8_t advance;\n");
        std::printf("    uint16_t dataOffset;\n");
        std::printf("};\n\n");
        std::printf("struct SansFont {\n");
        std::printf("    uint8_t lineHeight;\n");
        std::printf("    const GlyphMetrics* metrics;\n");
        std::printf("    const uint8_t* bitmaps;\n");
        std::printf("};\n\n");

        emitFontStruct(small);
        emitFontStruct(medium);
        emitFontStruct(bold);

        std::printf("inline constexpr int measureSansText(const SansFont& font,\n");
        std::printf("                                     const char* str,\n");
        std::printf("                                     int tracking = 0) noexcept {\n");
        std::printf("    if (str == nullptr || str[0] == '\\0') {\n");
        std::printf("        return 0;\n");
        std::printf("    }\n");
        std::printf("    int w = 0;\n");
        std::printf("    for (size_t i = 0u; str[i] != '\\0'; ++i) {\n");
        std::printf("        const unsigned char uc = static_cast<unsigned char>(str[i]);\n");
        std::printf("        const size_t idx = (uc >= 32u && uc <= 126u) ? (uc - 32u) : 0u;\n");
        std::printf("        w += static_cast<int>(font.metrics[idx].advance);\n");
        std::printf("        if (str[i + 1u] != '\\0') {\n");
        std::printf("            w += tracking;\n");
        std::printf("        }\n");
        std::printf("    }\n");
        std::printf("    return w;\n");
        std::printf("}\n\n");
        std::printf("} // namespace ruido\n");

        std::fprintf(stderr,
                     "Baked kFontSansSmall (%zu B) + kFontSansMedium (%zu B) + "
                     "kFontSansBold (%zu B)\n",
                     small.packedNibbles.size(), medium.packedNibbles.size(),
                     bold.packedNibbles.size());
    }
    return 0;
}
