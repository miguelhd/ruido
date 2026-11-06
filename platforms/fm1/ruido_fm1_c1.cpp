#include "ruido_fm1_state.h"

/* Core 1 hot DSP translation unit.
 * `build_fm1.py` renames all `.text*` and `.rodata*` sections of `[ruido_fm1_c1.o, *dsp_objs]`
 * into `.c1_text*` (`0x01C0E000..0x01C20000` internal SRAM) and redirects `memcpy`/`memset` to
 * `ruido_c1_memcpy`/`ruido_c1_memset` below so Core 1 executes 0 instructions and loads 0 bytes
 * from XIP SPI NOR Flash (`0x02000000+`). */

inline void* operator new(size_t, void* p) noexcept {
    return p;
}

namespace {

constexpr float kSampleRateHz = 44100.0f;
constexpr float kMasterPotSmoothSec = 0.015f; // 15 ms clickless analog MASTER pot slew
volatile uint32_t* const kTimer4CntReg = reinterpret_cast<volatile uint32_t*>(0x00010804u);

inline float bitsToFloat(uint32_t bits) noexcept {
    union {
        uint32_t u;
        float f;
    } conv{bits};
    return conv.f;
}

RUIDO_ALWAYS_INLINE bool pollSramCanaryFault(uint32_t pairs) noexcept {
    volatile const uint32_t* canaries = g_ruido_ic_ctrl.sramCanaries;
    uint32_t badBits = 0u;
    do {
        badBits |= canaries[0] | ~canaries[1];
    } while (--pairs != 0u);
    return badBits != 0u;
}

[[noreturn]] RUIDO_ALWAYS_INLINE void parkOnCore1SelfFault() noexcept {
    g_ruido_ic_ctrl.c1SelfFault.store(1u, std::memory_order_release);
    for (;;) {
        __asm__ volatile("nop" ::: "memory");
    }
}

inline void renderQuantizedBlock64(ruido::Fm1AudioBlock& outBlk) noexcept {
    auto* buses = ruidoSharedBuses();
    auto* core = ruidoDspCore();
    auto* rt = ruidoC1Runtime();

    const float targetGain = ruido::clampf(
        bitsToFloat(g_ruido_ic_ctrl.masterGainBits.load(std::memory_order_relaxed)), 0.0f, 1.0f);
    rt->masterGainSmoother.setTarget(targetGain);

    float left[ruido::kBlockFrames];
    float right[ruido::kBlockFrames];
    core->processBlock(*buses, left, right, ruido::kBlockFrames);

    int32_t blockPeak = 0;
    const bool gainSettled = (rt->masterGainSmoother.current() == rt->masterGainSmoother.target());
    const float staticGain = rt->masterGainSmoother.current();
    for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
        const float g = gainSettled ? staticGain : rt->masterGainSmoother.next();
        const int16_t sL = rt->quantizer.quantizeLeft(left[i], g);
        const int16_t sR = rt->quantizer.quantizeRight(right[i], g);
        outBlk.lr[2u * i] = sL;
        outBlk.lr[2u * i + 1u] = sR;
        const int32_t aL = (sL < 0) ? -static_cast<int32_t>(sL) : static_cast<int32_t>(sL);
        const int32_t aR = (sR < 0) ? -static_cast<int32_t>(sR) : static_cast<int32_t>(sR);
        if (aL > blockPeak) {
            blockPeak = aL;
        }
        if (aR > blockPeak) {
            blockPeak = aR;
        }
    }
    outBlk.peakAbsQ15 = blockPeak;
}

} // namespace

extern "C" {

/* SRAM-resident `memset` / `memcpy` for Core 1 TUs (inline asm barrier blocks loop-idiom recursion)
 */
void* ruido_c1_memset(void* dst, int c, unsigned n) {
    auto* p = static_cast<uint8_t*>(dst);
    const auto b = static_cast<uint8_t>(c);
    while (n > 0u && ((reinterpret_cast<uintptr_t>(p) & 3u) != 0u)) {
        *p++ = b;
        --n;
        __asm__ volatile("" : "+r"(p));
    }
    if (n >= 4u) {
        const uint32_t w = static_cast<uint32_t>(b) | (static_cast<uint32_t>(b) << 8u) |
                           (static_cast<uint32_t>(b) << 16u) | (static_cast<uint32_t>(b) << 24u);
        auto* wp = reinterpret_cast<uint32_t*>(p);
        while (n >= 4u) {
            *wp++ = w;
            n -= 4u;
            __asm__ volatile("" : "+r"(wp));
        }
        p = reinterpret_cast<uint8_t*>(wp);
    }
    while (n-- > 0u) {
        *p++ = b;
        __asm__ volatile("" : "+r"(p));
    }
    return dst;
}

void* ruido_c1_memcpy(void* dst, const void* src, unsigned n) {
    auto* d = static_cast<uint8_t*>(dst);
    const auto* s = static_cast<const uint8_t*>(src);
    if (n >= 4u &&
        ((reinterpret_cast<uintptr_t>(d) & 3u) == (reinterpret_cast<uintptr_t>(s) & 3u))) {
        while ((reinterpret_cast<uintptr_t>(d) & 3u) != 0u) {
            *d++ = *s++;
            --n;
            __asm__ volatile("" : "+r"(d), "+r"(s));
        }
        auto* wd = reinterpret_cast<uint32_t*>(d);
        const auto* ws = reinterpret_cast<const uint32_t*>(s);
        while (n >= 4u) {
            *wd++ = *ws++;
            n -= 4u;
            __asm__ volatile("" : "+r"(wd), "+r"(ws));
        }
        d = reinterpret_cast<uint8_t*>(wd);
        s = reinterpret_cast<const uint8_t*>(ws);
    }
    while (n-- > 0u) {
        *d++ = *s++;
        __asm__ volatile("" : "+r"(d), "+r"(s));
    }
    return dst;
}

void ruido_fm1_c1_construct(float initial_master_gain) {
    auto* buses = new (g_ruido_shared_buses_arena) ruido::EngineSharedBuses();
    auto* ring = new (g_ruido_audio_ring_arena) ruido::Fm1AudioBlockRing();
    auto* coproc = new (g_ruido_coproc_arena) ruido::VoiceFilterCoProcessor();
    auto* core = new (g_ruido_dsp_core_arena) ruido::DroneEngineCore();
    auto* rt = new (g_ruido_c1_runtime_arena) ruido::fm1::Fm1C1DspRuntime();

    ring->reset();
    coproc->reset();
    core->init(*buses, kSampleRateHz);
    rt->masterGainSmoother.init(kSampleRateHz, kMasterPotSmoothSec,
                                ruido::clampf(initial_master_gain, 0.0f, 1.0f));
    rt->quantizer.reset();
}

void ruido_fm1_c1_snap_latest(void) {
    ruidoDspCore()->snapParametersToLatest(*ruidoSharedBuses());
}

void ruido_fm1_c1_prime_blocks(uint32_t num_blocks, uint32_t reattach_coproc) {
    auto* ring = ruidoAudioRing();
    auto* core = ruidoDspCore();
    auto* coproc = ruidoCoProc();
    core->attachFilterCoProcessor(nullptr);
    coproc->reset();
    ring->reset();
    for (uint32_t b = 0u; b < num_blocks; ++b) {
        ruido::Fm1AudioBlock* slot = ring->beginPush();
        if (slot == nullptr) {
            break;
        }
        renderQuantizedBlock64(*slot);
        ruido::fm1::hwMemoryBarrier();
        ring->commitPush();
    }
    if (reattach_coproc != 0u) {
        coproc->setHelperArmed(true);
        core->attachFilterCoProcessor(coproc);
    }
    ruido::fm1::hwMemoryBarrier();
}

void ruido_fm1_c1_render_one_block(ruido::Fm1AudioBlock* dst_block) {
    if (dst_block == nullptr) {
        return;
    }
    renderQuantizedBlock64(*dst_block);
}

void ruido_fm1_set_bench_stress(uint32_t stress_mode) {
    auto* buses = ruidoSharedBuses();
    auto* core = ruidoDspCore();
    if (stress_mode != 0u) {
        ruido::EngineControlState st{};
        for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
            auto& v = st.voices[i];
            v.frequencyHz = 110.0f * static_cast<float>(i + 1u);
            v.pulseWidth = 0.32f;
            v.drift = 0.35f;
            v.hpCutoffHz = 220.0f;
            v.hpResonance = 0.55f;
            v.lpCutoffHz = 1100.0f;
            v.lpResonance = 0.72f;
            v.amplitude = 0.88f; // Active MS-20 input overdrive (> 0.70 knee)
            v.pan = ((i & 1u) != 0u) ? 0.45f : -0.45f;
            v.lfoTarget = ruido::LfoTarget::LpCutoff;
            v.lfoShape = 0.65f;
            v.lfoRateHz = 3.5f;
            v.lfoDepth = 0.55f;
            v.lfoBTarget = ruido::LfoTarget::Pitch;
            v.lfoBShape = 0.90f;
            v.lfoBRateHz = 7.0f;
            v.lfoBDepth = 0.35f;
        }
        buses->controlBus.publish(st);
        core->snapParametersToLatest(*buses);
    } else {
        core->init(*buses, kSampleRateHz);
        core->snapParametersToLatest(*buses);
    }
}

void ruido_fm1_c1_step_solo_bench64(void) {
    float left[ruido::kBlockFrames];
    float right[ruido::kBlockFrames];
    ruidoDspCore()->processBlock(*ruidoSharedBuses(), left, right, ruido::kBlockFrames);
}

void ruido_fm1_c1_assist_dsp(void) {
    (void)ruidoCoProc()->tryAssist();
}

void ruido_fm1_c1_set_helper_armed(uint32_t armed) {
    ruidoCoProc()->setHelperArmed(armed != 0u);
}

/* Pure register-only leaf POST (`r0..r3` caller-saved only -> `0` stack loads / `[sp++]` pops):
 * executes 512 SRAM canary reads (`0x00000000` & `0xFFFFFFFF`) before Core 1 builds any C++ call
 * stack, catching AC791N bit-21 (`0x00200000`) misreads cleanly on affected silicon. */
void ruido_fm1_c1_boot_post(void) {
    if (pollSramCanaryFault(256u)) {
        parkOnCore1SelfFault();
    }
}

void ruido_fm1_cpu1_loop(void) {
    auto* ring = ruidoAudioRing();
    auto* core = ruidoDspCore();
    auto* buses = ruidoSharedBuses();
    for (;;) {
        if (pollSramCanaryFault(16u)) {
            parkOnCore1SelfFault();
        }

        if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) != 0u) {
            __asm__ volatile("nop\n\tnop" ::: "memory");
            continue;
        }

        if (g_ruido_ic_ctrl.c1BenchReq.load(std::memory_order_acquire) != 0u) {
            float left[ruido::kBlockFrames];
            float right[ruido::kBlockFrames];
            const uint32_t t0 = *kTimer4CntReg;
            core->processBlock(*buses, left, right, ruido::kBlockFrames);
            const uint32_t dt = *kTimer4CntReg - t0;
            if (pollSramCanaryFault(16u)) {
                parkOnCore1SelfFault();
            }
            g_ruido_ic_ctrl.c1BenchTicks.store(dt, std::memory_order_relaxed);
            g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_release);
            continue;
        }

        if (g_ruido_ic_ctrl.c1RunEnable.load(std::memory_order_acquire) == 0u) {
            __asm__ volatile("nop" ::: "memory");
            continue;
        }

        ruido::Fm1AudioBlock* slot = ring->beginPush();
        if (slot == nullptr) {
            /* Paced strictly by ALNK0 24 MHz crystal DMA draining slots on Core 0 (never `idle`!)
             */
            __asm__ volatile("nop\n\tnop\n\tnop\n\tnop" ::: "memory");
            continue;
        }

        const uint32_t t0 = *kTimer4CntReg;
        renderQuantizedBlock64(*slot);
        const uint32_t dt = *kTimer4CntReg - t0;

        if (pollSramCanaryFault(16u)) {
            parkOnCore1SelfFault();
        }

        ruido::fm1::hwMemoryBarrier();
        ring->commitPush();
        ruido::fm1::hwMemoryBarrier();

        g_ruido_fm1_status.c1_last64_ticks = dt;
        g_ruido_fm1_status.c1_blocks = g_ruido_fm1_status.c1_blocks + 1u;
    }
}

} // extern "C"
