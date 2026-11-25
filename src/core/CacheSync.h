#pragma once

#include <cstddef>
#include <cstdint>

namespace ruido {

// JieLi AC791N (WL82 / dual q32DSP) L1 Data-Cache line geometry (32 bytes).
// Core 0 (`q32DSP(0)`), Core 1 (`q32DSP(1)`), and hardware DMA (`ALNK0`, `SPI1`) do not snoop
// each other's L1 data caches over `0x01C00000..0x01C80000` SRAM. All shared SPSC structures must
// isolate single-writer fields onto disjoint cache lines and explicitly write-back / invalidate
// (`flushinv`) producer/consumer ranges across handoff boundaries.
inline constexpr size_t kHwCacheLineBytes = 32u;

#if (defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)) &&                  \
    !defined(__aarch64__) && !defined(__x86_64__) && !defined(__wasm__)
#define RUIDO_HAS_PI32_CACHE_ASM 1
#else
#define RUIDO_HAS_PI32_CACHE_ASM 0
#endif

inline void hwCacheFlushInvalidateLine(const volatile void* ptr) noexcept {
#if RUIDO_HAS_PI32_CACHE_ASM
    __asm__ volatile("csync\n\tflushinv [%0]\n\tssync\n\tcsync" ::"r"(ptr) : "memory");
#else
    (void)ptr;
#endif
}

inline void hwCacheFlushInvalidateRange(const volatile void* ptr, size_t numBytes) noexcept {
#if RUIDO_HAS_PI32_CACHE_ASM
    if (ptr == nullptr || numBytes == 0u) {
        return;
    }
    const uintptr_t raw = reinterpret_cast<uintptr_t>(ptr);
    const uintptr_t start = raw & ~static_cast<uintptr_t>(kHwCacheLineBytes - 1u);
    const uintptr_t end = raw + numBytes;
    __asm__ volatile("csync" ::: "memory");
    for (uintptr_t a = start; a < end; a += kHwCacheLineBytes) {
        __asm__ volatile("flushinv [%0]" ::"r"(a) : "memory");
    }
    __asm__ volatile("ssync\n\tcsync" ::: "memory");
#else
    (void)ptr;
    (void)numBytes;
#endif
}

} // namespace ruido
