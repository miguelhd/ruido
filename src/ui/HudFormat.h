#pragma once

#include "core/DspMath.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

// Zero-allocation fixed-buffer technical string formatters for Core 0 HUD rendering
inline size_t appendUInt(char* dst, size_t pos, size_t cap, uint32_t val) noexcept {
    char rev[12]{};
    size_t n = 0u;
    do {
        rev[n++] = static_cast<char>('0' + (val % 10u));
        val /= 10u;
    } while (val > 0u && n < sizeof(rev));
    while (n > 0u && pos + 1u < cap) {
        dst[pos++] = rev[--n];
    }
    dst[pos] = '\0';
    return pos;
}

inline size_t appendStr(char* dst, size_t pos, size_t cap, const char* s) noexcept {
    for (size_t i = 0u; s[i] != '\0' && pos + 1u < cap; ++i) {
        dst[pos++] = s[i];
    }
    dst[pos] = '\0';
    return pos;
}

// Compact technical frequency formatter for 60px 4-knob columns
inline void formatCompactHz(char* buf, size_t cap, float hz) noexcept {
    size_t p = 0u;
    const float f = clampf(hz, 0.001f, 99999.0f);
    if (f < 1.0f) {
        const uint32_t centi = static_cast<uint32_t>(f * 100.0f + 0.5f);
        p = appendUInt(buf, p, cap, centi / 100u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, (centi / 10u) % 10u);
        p = appendUInt(buf, p, cap, centi % 10u);
        appendStr(buf, p, cap, " Hz");
    } else if (f < 100.0f) {
        const uint32_t deci = static_cast<uint32_t>(f * 10.0f + 0.5f);
        p = appendUInt(buf, p, cap, deci / 10u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, deci % 10u);
        appendStr(buf, p, cap, " Hz");
    } else if (f < 1000.0f) {
        p = appendUInt(buf, p, cap, static_cast<uint32_t>(f + 0.5f));
        appendStr(buf, p, cap, " Hz");
    } else if (f < 10000.0f) {
        const uint32_t centiK = static_cast<uint32_t>(f * 0.01f + 0.5f);
        p = appendUInt(buf, p, cap, centiK / 10u);
        p = appendStr(buf, p, cap, ".");
        p = appendUInt(buf, p, cap, centiK % 10u);
        appendStr(buf, p, cap, " kHz");
    } else {
        const uint32_t intK = static_cast<uint32_t>(f * 0.001f + 0.5f);
        p = appendUInt(buf, p, cap, intK);
        appendStr(buf, p, cap, " kHz");
    }
}

// High-precision frequency counter formatter for top-right header readouts:
//  - < 1000 Hz:    0.01 Hz resolution ("55.00 Hz", "999.99 Hz")
//  - < 10000 Hz:   1 Hz / 0.001 kHz resolution ("1.234 kHz")
//  - >= 10000 Hz:  10 Hz / 0.01 kHz resolution ("20.00 kHz")
inline size_t appendPrecisionHz(char* buf, size_t pos, size_t cap, float hz) noexcept {
    const float f = clampf(hz, 0.01f, 99999.99f);
    if (f < 1000.0f) {
        const uint32_t centi = static_cast<uint32_t>(f * 100.0f + 0.5f);
        pos = appendUInt(buf, pos, cap, centi / 100u);
        pos = appendStr(buf, pos, cap, ".");
        pos = appendUInt(buf, pos, cap, (centi / 10u) % 10u);
        pos = appendUInt(buf, pos, cap, centi % 10u);
        return appendStr(buf, pos, cap, " Hz");
    }
    if (f < 10000.0f) {
        const uint32_t milliK = static_cast<uint32_t>(f + 0.5f);
        pos = appendUInt(buf, pos, cap, milliK / 1000u);
        pos = appendStr(buf, pos, cap, ".");
        pos = appendUInt(buf, pos, cap, (milliK / 100u) % 10u);
        pos = appendUInt(buf, pos, cap, (milliK / 10u) % 10u);
        pos = appendUInt(buf, pos, cap, milliK % 10u);
        return appendStr(buf, pos, cap, " kHz");
    }
    const uint32_t centiK = static_cast<uint32_t>(f * 0.1f + 0.5f);
    pos = appendUInt(buf, pos, cap, centiK / 100u);
    pos = appendStr(buf, pos, cap, ".");
    pos = appendUInt(buf, pos, cap, (centiK / 10u) % 10u);
    pos = appendUInt(buf, pos, cap, centiK % 10u);
    return appendStr(buf, pos, cap, " kHz");
}

inline void formatPercent(char* buf, size_t cap, float norm01) noexcept {
    const uint32_t pct = static_cast<uint32_t>(clampf(norm01, 0.0f, 1.0f) * 100.0f + 0.5f);
    size_t p = appendUInt(buf, 0u, cap, pct);
    appendStr(buf, p, cap, "%");
}

inline void formatPanValue(char* buf, size_t cap, float pan) noexcept {
    const int pInt = static_cast<int>(pan * 100.0f + ((pan >= 0.0f) ? 0.5f : -0.5f));
    if (pInt > -2 && pInt < 2) {
        appendStr(buf, 0u, cap, "C");
        return;
    }
    size_t p = 0u;
    if (pInt < 0) {
        p = appendStr(buf, p, cap, "L");
        appendUInt(buf, p, cap, static_cast<uint32_t>(-pInt));
    } else {
        p = appendStr(buf, p, cap, "R");
        appendUInt(buf, p, cap, static_cast<uint32_t>(pInt));
    }
}

inline void formatSemitones(char* buf, size_t cap, float semitones, bool signedPrefix) noexcept {
    const float absSt = absf(semitones);
    const uint32_t deci = static_cast<uint32_t>(clampf(absSt, 0.0f, 99.9f) * 10.0f + 0.5f);
    size_t p = 0u;
    if (signedPrefix && deci > 0u) {
        p = appendStr(buf, p, cap, (semitones >= 0.0f) ? "+" : "-");
    }
    p = appendUInt(buf, p, cap, deci / 10u);
    p = appendStr(buf, p, cap, ".");
    p = appendUInt(buf, p, cap, deci % 10u);
    appendStr(buf, p, cap, " st");
}

inline void formatSignedPercent(char* buf, size_t cap, float bipolar) noexcept {
    const int pct = static_cast<int>(clampf(bipolar, -1.0f, 1.0f) * 100.0f +
                                     ((bipolar >= 0.0f) ? 0.5f : -0.5f));
    if (pct == 0) {
        appendStr(buf, 0u, cap, "0%");
        return;
    }
    size_t p = appendStr(buf, 0u, cap, (pct > 0) ? "+" : "-");
    p = appendUInt(buf, p, cap, static_cast<uint32_t>((pct > 0) ? pct : -pct));
    appendStr(buf, p, cap, "%");
}

inline void formatSignedCents(char* buf, size_t cap, float cents) noexcept {
    const int cInt =
        static_cast<int>(clampf(cents, -999.0f, 999.0f) + ((cents >= 0.0f) ? 0.5f : -0.5f));
    if (cInt == 0) {
        appendStr(buf, 0u, cap, "0c");
        return;
    }
    size_t p = appendStr(buf, 0u, cap, (cInt > 0) ? "+" : "-");
    p = appendUInt(buf, p, cap, static_cast<uint32_t>((cInt > 0) ? cInt : -cInt));
    appendStr(buf, p, cap, "c");
}

inline void formatMultiplierX(char* buf, size_t cap, float mult) noexcept {
    const uint32_t deci = static_cast<uint32_t>(clampf(mult, 1.0f, 99.9f) * 10.0f + 0.5f);
    size_t p = appendUInt(buf, 0u, cap, deci / 10u);
    p = appendStr(buf, p, cap, ".");
    p = appendUInt(buf, p, cap, deci % 10u);
    appendStr(buf, p, cap, "x");
}

} // namespace ruido
