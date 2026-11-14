#pragma once

#include <stdint.h>

/* Fast 4-word unrolled `memset`/`memcpy` replacing dinghy's 1-byte-at-a-time `libc.c` loops. */
void* memset(void* dst, int c, unsigned n) {
    uint8_t* p = (uint8_t*)dst;
    const uint8_t b = (uint8_t)c;
    while (n > 0u && (((uintptr_t)p & 3u) != 0u)) {
        *p++ = b;
        --n;
    }
    if (n >= 16u) {
        const uint32_t w =
            (uint32_t)b | ((uint32_t)b << 8u) | ((uint32_t)b << 16u) | ((uint32_t)b << 24u);
        uint32_t* wp = (uint32_t*)p;
        while (n >= 16u) {
            wp[0] = w;
            wp[1] = w;
            wp[2] = w;
            wp[3] = w;
            wp += 4;
            n -= 16u;
        }
        while (n >= 4u) {
            *wp++ = w;
            n -= 4u;
        }
        p = (uint8_t*)wp;
    }
    while (n-- > 0u) {
        *p++ = b;
    }
    return dst;
}

void* memcpy(void* dst, const void* src, unsigned n) {
    uint8_t* d = (uint8_t*)dst;
    const uint8_t* s = (const uint8_t*)src;
    if (n >= 16u && (((uintptr_t)d & 3u) == ((uintptr_t)s & 3u))) {
        while (((uintptr_t)d & 3u) != 0u) {
            *d++ = *s++;
            --n;
        }
        uint32_t* wd = (uint32_t*)d;
        const uint32_t* ws = (const uint32_t*)s;
        while (n >= 16u) {
            wd[0] = ws[0];
            wd[1] = ws[1];
            wd[2] = ws[2];
            wd[3] = ws[3];
            wd += 4;
            ws += 4;
            n -= 16u;
        }
        while (n >= 4u) {
            *wd++ = *ws++;
            n -= 4u;
        }
        d = (uint8_t*)wd;
        s = (const uint8_t*)ws;
    }
    while (n-- > 0u) {
        *d++ = *s++;
    }
    return dst;
}
