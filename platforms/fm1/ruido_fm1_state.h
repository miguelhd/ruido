#pragma once

/* Shared cache-line-isolated state layout between `ruido_fm1_bridge.cpp` (Core 0, XIP)
 * and `ruido_fm1_c1.cpp` (Core 1, `.c1_text` internal SRAM).
 *
 * Every cross-core or per-core arena starts on its own 64-byte boundary in `.pool` SRAM so
 * Core 0 UI / DMA traffic never false-shares cache lines with Core 1 ZDF filter state. */

#include "core/ParameterSmoother.h"
#include "dsp/DroneEngine.h"
#include "dsp/VoiceFilterCoProcessor.h"
#include "fm1_audio_io.h"
#include "ruido_fm1_bridge.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido::fm1 {

inline void hwMemoryBarrier() noexcept {
    __asm__ volatile("ssync\n\tcsync" ::: "memory");
}

inline void flushCacheLine(const volatile void* ptr) noexcept {
    __asm__ volatile("flushinv [%0]\n\tssync\n\tcsync" ::"r"(ptr) : "memory");
}

/* Full 32-bit indirect call wrapper (`call rX`) across XIP (0x02000120) -> .c1_text (0x01C0E000),
 * avoiding pi32v2's +-4 MiB PC-relative `R_PI32V2_LONG_JUMP_23M2` direct-call limit. */
template <typename Fn, typename... Args>
inline auto callC1Far(Fn* fn, Args... args) noexcept -> decltype(fn(args...)) {
    Fn* volatile p = fn;
    return p(args...);
}

struct alignas(64) Fm1InterCoreControl {
    // Two-word SRAM integrity canary (`[0]=0x00000000`, `[1]=0xFFFFFFFF`) probed by Core 1 at boot
    // and on every 64-frame block to catch AC791N bit-21 (`0x00200000`) RAM misreads immediately.
    volatile uint32_t sramCanaries[2]{0u, 0xFFFFFFFFu};
    std::atomic<uint32_t> c1SelfFault{0u};
    volatile uint32_t c1Booted{0u};
    // Diagnostics: what tripped Core 1 (`fm1_cpu1.S` writes DBG_MSG / `0xC1xxxxxx` / `0xCA...`)
    volatile uint32_t c1FaultCode{0u};
    volatile uint32_t c1FaultAux{0u};
    alignas(64) std::atomic<uint32_t> engineReady{0u};
    std::atomic<uint32_t> c1RunEnable{0u};
    std::atomic<uint32_t> c1FallbackActive{0u};
    std::atomic<uint32_t> c1BenchReq{0u};
    std::atomic<uint32_t> c1BenchTicks{0u};
    std::atomic<uint32_t> c1FlashHold{0u};
    std::atomic<uint32_t> c1FlashAck{0u};
};

struct alignas(64) Fm1C1DspRuntime {
    Fm1TpdfQuantizer quantizer{};
    volatile uint8_t soloHalfRateActive = 0u;
    volatile uint8_t inSoloDsp = 0u;
    float soloHistL[3]{0.0f, 0.0f, 0.0f};
    float soloHistR[3]{0.0f, 0.0f, 0.0f};
};

} // namespace ruido::fm1

extern uint8_t g_ruido_shared_buses_arena[sizeof(ruido::EngineSharedBuses)];
extern uint8_t g_ruido_audio_ring_arena[sizeof(ruido::Fm1AudioBlockRing)];
extern uint8_t g_ruido_coproc_arena[sizeof(ruido::VoiceFilterCoProcessor)];
extern uint8_t g_ruido_dsp_core_arena[sizeof(ruido::DroneEngineCore)];
extern uint8_t g_ruido_c1_runtime_arena[sizeof(ruido::fm1::Fm1C1DspRuntime)];
extern ruido::fm1::Fm1InterCoreControl g_ruido_ic_ctrl;

inline ruido::EngineSharedBuses* ruidoSharedBuses() noexcept {
    return reinterpret_cast<ruido::EngineSharedBuses*>(g_ruido_shared_buses_arena);
}

inline ruido::Fm1AudioBlockRing* ruidoAudioRing() noexcept {
    return reinterpret_cast<ruido::Fm1AudioBlockRing*>(g_ruido_audio_ring_arena);
}

inline ruido::VoiceFilterCoProcessor* ruidoCoProc() noexcept {
    return reinterpret_cast<ruido::VoiceFilterCoProcessor*>(g_ruido_coproc_arena);
}

inline ruido::DroneEngineCore* ruidoDspCore() noexcept {
    return reinterpret_cast<ruido::DroneEngineCore*>(g_ruido_dsp_core_arena);
}

inline ruido::fm1::Fm1C1DspRuntime* ruidoC1Runtime() noexcept {
    return reinterpret_cast<ruido::fm1::Fm1C1DspRuntime*>(g_ruido_c1_runtime_arena);
}

extern "C" {

/* Placement-new construct Core 1 DSP state (`EngineSharedBuses`, `Fm1AudioBlockRing`,
 * `VoiceFilterCoProcessor`, `DroneEngineCore`, `Fm1C1DspRuntime`) inside `.c1_text`. */
void ruido_fm1_c1_construct(float initial_master_gain);

/* Snap all Core 1 DSP parameter smoothers immediately to the latest published control bus state. */
void ruido_fm1_c1_snap_latest(void);

/* Pre-fill `num_blocks` 64-frame TPDF-dithered stereo blocks into `Fm1AudioBlockRing` prior to
 * arming ALNK0 DMA so the very first callback drains click-free audio. */
void ruido_fm1_c1_prime_blocks(uint32_t num_blocks, uint32_t reattach_coproc);

/* Pump up to `max_blocks` into `Fm1AudioBlockRing` on Core 0 when `c1FallbackActive != 0`. */
void ruido_fm1_c1_pump_solo_ring(uint32_t max_blocks);

/* Switch `DroneEngineCore` into 22.05 kHz half-rate (88.2 kHz 4x ZDF) solo-core fallback mode. */
void ruido_fm1_c1_enter_solo_fallback(void);

/* Render one 64-frame float32 DSP block directly in `.c1_text` SRAM (fallback boot benchmark). */
void ruido_fm1_c1_step_solo_bench64(void);

/* Claim and run one pending `Voices 2-3` 4x ZDF filter span inside `.c1_text` SRAM (`-O3`). */
void ruido_fm1_c1_assist_dsp(void);

/* Arm/disarm Core 0 filter assistance via Dekker SPSC fence inside `.c1_text` SRAM. */
void ruido_fm1_c1_set_helper_armed(uint32_t armed);

/* Stackless leaf SRAM read-bus POST executed on Core 1 before entering `ruido_fm1_cpu1_loop`. */
void ruido_fm1_c1_boot_post(void);

/* Core 1 infinite DSP worker loop invoked by `fm1_cpu1_entry` in `.c1_text` SRAM. */
void ruido_fm1_cpu1_loop(void);

} // extern "C"
