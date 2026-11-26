#include "ruido_fm1_state.h"

/* Core 1 hot DSP translation unit.
 * `build_fm1.py` renames all `.text*` and `.rodata*` sections of `[ruido_fm1_c1.o, *dsp_objs]`
 * into `.c1_text*` (`0x01C0E000..0x01C20000` internal SRAM) and redirects `memcpy`/`memset` to
 * `ruido_c1_memcpy`/`ruido_c1_memset` below so Core 1 executes 0 instructions and loads 0 bytes
 * from XIP SPI NOR Flash (`0x02000000+`). */

static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, sramCanaries) == 0u,
              "fm1_cpu1.S expects sramCanaries at offset 0");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, c1SelfFault) == 8u,
              "fm1_cpu1.S expects c1SelfFault at offset 8");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, c1Booted) == 12u,
              "fm1_cpu1.S expects c1Booted at offset 12");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, c1FaultCode) == 16u,
              "fm1_cpu1.S expects c1FaultCode at offset 16");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, c1FaultAux) == 20u,
              "fm1_cpu1.S expects c1FaultAux at offset 20");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, engineReady) == 64u,
              "Core 0 control atomics isolated on cache line 1");
static_assert(__builtin_offsetof(ruido::fm1::Fm1InterCoreControl, c1FlashAck) + sizeof(uint32_t) <=
                  96u,
              "Control atomics must fit within single 32-byte q32DSP cache line [64..95]");

inline void* operator new(size_t, void* p) noexcept {
    return p;
}

namespace {

constexpr float kSampleRateHz = 44100.0f;
volatile uint32_t* const kTimer4CntReg = reinterpret_cast<volatile uint32_t*>(0x00010804u);

RUIDO_ALWAYS_INLINE bool pollSramCanaryFault(uint32_t pairs) noexcept {
    volatile const uint32_t* canaries = g_ruido_ic_ctrl.sramCanaries;
    uint32_t badBits = 0u;
    do {
        badBits |= canaries[0] | ~canaries[1];
    } while (--pairs != 0u);
    return badBits != 0u;
}

[[noreturn]] RUIDO_ALWAYS_INLINE void parkOnCore1SelfFault() noexcept {
    g_ruido_ic_ctrl.c1FaultCode = 0xCA000000u;
    g_ruido_boot_diag.c1_code = 0xCA000000u;
    __asm__ volatile("flushinv [%0]\n\tssync" ::"r"(&g_ruido_boot_diag) : "memory");
    g_ruido_ic_ctrl.c1SelfFault.store(1u, std::memory_order_release);
    __asm__ volatile("flushinv [%0]\n\tssync\n\tcsync" ::"r"(&g_ruido_ic_ctrl) : "memory");
    for (;;) {
        __asm__ volatile("nop" ::: "memory");
    }
}

inline float activeBaseSampleRate(const ruido::fm1::Fm1C1DspRuntime* rt) noexcept {
    return (rt->soloHalfRateActive != 0u) ? (kSampleRateHz * 0.5f) : kSampleRateHz;
}

inline void upsampleCatmullRom32To64(const float halfIn[ruido::kBlockFrames / 2u], float hist[3],
                                     float out64[ruido::kBlockFrames]) noexcept {
    constexpr size_t kHalfFrames = ruido::kBlockFrames / 2u;
    float ext[kHalfFrames + 3u];
    ext[0] = hist[0];
    ext[1] = hist[1];
    ext[2] = hist[2];
    for (size_t i = 0u; i < kHalfFrames; ++i) {
        ext[i + 3u] = halfIn[i];
    }
    for (size_t i = 0u; i < kHalfFrames; ++i) {
        const float a = ext[i];
        const float b = ext[i + 1u];
        const float c = ext[i + 2u];
        const float d = ext[i + 3u];
        out64[2u * i] = b;
        out64[2u * i + 1u] = ruido::dacSafetyClip(0.5625f * (b + c) - 0.0625f * (a + d));
    }
    hist[0] = halfIn[kHalfFrames - 3u];
    hist[1] = halfIn[kHalfFrames - 2u];
    hist[2] = halfIn[kHalfFrames - 1u];
}

inline void renderQuantizedBlock64(ruido::Fm1AudioBlock& outBlk) noexcept {
    auto* buses = ruidoSharedBuses();
    auto* core = ruidoDspCore();
    auto* rt = ruidoC1Runtime();

    float left[ruido::kBlockFrames];
    float right[ruido::kBlockFrames];
    if (rt->soloHalfRateActive != 0u) {
        constexpr size_t kHalfFrames = ruido::kBlockFrames / 2u;
        float halfL[kHalfFrames];
        float halfR[kHalfFrames];
        core->processBlock(*buses, halfL, halfR, kHalfFrames);
        upsampleCatmullRom32To64(halfL, rt->soloHistL, left);
        upsampleCatmullRom32To64(halfR, rt->soloHistR, right);
    } else {
        core->processBlock(*buses, left, right, ruido::kBlockFrames);
    }

    bool blockSilent = true;
    for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
        if (left[i] != 0.0f || right[i] != 0.0f) {
            blockSilent = false;
            break;
        }
    }
    if (blockSilent) {
        constexpr uint32_t kZeroPacked = 0u;
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            __builtin_memcpy(&outBlk.lr[2u * i], &kZeroPacked, sizeof(kZeroPacked));
        }
        outBlk.peakAbsQ15 = 0;
        return;
    }

    int32_t blockPeak = 0;
    for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
        const int16_t sL = rt->quantizer.quantizeLeftUnity(left[i]);
        const int16_t sR = rt->quantizer.quantizeRightUnity(right[i]);
        const uint32_t packed = static_cast<uint32_t>(static_cast<uint16_t>(sL)) |
                                (static_cast<uint32_t>(static_cast<uint16_t>(sR)) << 16u);
        __builtin_memcpy(&outBlk.lr[2u * i], &packed, sizeof(packed));
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

[[gnu::noinline]] uint32_t runCore1Bench64(ruido::DroneEngineCore* core,
                                           ruido::EngineSharedBuses* buses) noexcept {
    float left[ruido::kBlockFrames];
    float right[ruido::kBlockFrames];
    const uint32_t t0 = *kTimer4CntReg;
    core->processBlock(*buses, left, right, ruido::kBlockFrames);
    return *kTimer4CntReg - t0;
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
        while (n >= 16u) {
            wd[0] = ws[0];
            wd[1] = ws[1];
            wd[2] = ws[2];
            wd[3] = ws[3];
            wd += 4;
            ws += 4;
            n -= 16u;
            __asm__ volatile("" : "+r"(wd), "+r"(ws));
        }
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
    (void)initial_master_gain;
    auto* buses = new (g_ruido_shared_buses_arena) ruido::EngineSharedBuses();
    auto* ring = new (g_ruido_audio_ring_arena) ruido::Fm1AudioBlockRing();
    auto* coproc = new (g_ruido_coproc_arena) ruido::VoiceFilterCoProcessor();
    auto* core = new (g_ruido_dsp_core_arena) ruido::DroneEngineCore();
    auto* rt = new (g_ruido_c1_runtime_arena) ruido::fm1::Fm1C1DspRuntime();

    ring->reset();
    coproc->reset();
    core->init(*buses, kSampleRateHz);
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

void ruido_fm1_c1_enter_solo_fallback(void) {
    auto* rt = ruidoC1Runtime();
    auto* core = ruidoDspCore();
    auto* buses = ruidoSharedBuses();
    core->attachFilterCoProcessor(nullptr);
    if (rt->soloHalfRateActive == 0u) {
        rt->soloHalfRateActive = 1u;
        for (size_t k = 0u; k < 3u; ++k) {
            rt->soloHistL[k] = 0.0f;
            rt->soloHistR[k] = 0.0f;
        }
        if (rt->inSoloDsp == 0u) {
            core->init(*buses, kSampleRateHz * 0.5f);
            core->snapParametersToLatest(*buses);
        }
    }
}

void ruido_fm1_c1_pump_solo_ring(uint32_t max_blocks) {
    auto* rt = ruidoC1Runtime();
    if (rt->inSoloDsp != 0u) {
        return;
    }
    rt->inSoloDsp = 1u;
    auto* ring = ruidoAudioRing();
    for (uint32_t b = 0u; b < max_blocks; ++b) {
        ruido::Fm1AudioBlock* slot = ring->beginPush();
        if (slot == nullptr) {
            break;
        }
        renderQuantizedBlock64(*slot);
        ruido::fm1::hwMemoryBarrier();
        ring->commitPush();
    }
    rt->inSoloDsp = 0u;
}

void ruido_fm1_set_bench_stress(uint32_t stress_mode) {
    auto* buses = ruidoSharedBuses();
    auto* core = ruidoDspCore();
    const float sr = activeBaseSampleRate(ruidoC1Runtime());
    if (stress_mode == 2u) {
        auto st = ruido::EngineControlState::makeDefault();
        st.global.masterRun = true;
        buses->controlBus.publish(st);
        core->init(*buses, sr);
        core->snapParametersToLatest(*buses);
    } else if (stress_mode != 0u) {
        ruido::EngineControlState st{};
        st.global.masterRun = true;
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
        core->init(*buses, sr);
        core->snapParametersToLatest(*buses);
    }
}

void ruido_fm1_c1_step_solo_bench64(void) {
    auto* rt = ruidoC1Runtime();
    float left[ruido::kBlockFrames];
    float right[ruido::kBlockFrames];
    if (rt->soloHalfRateActive != 0u) {
        constexpr size_t kHalfFrames = ruido::kBlockFrames / 2u;
        float halfL[kHalfFrames];
        float halfR[kHalfFrames];
        ruidoDspCore()->processBlock(*ruidoSharedBuses(), halfL, halfR, kHalfFrames);
        upsampleCatmullRom32To64(halfL, rt->soloHistL, left);
        upsampleCatmullRom32To64(halfR, rt->soloHistR, right);
    } else {
        ruidoDspCore()->processBlock(*ruidoSharedBuses(), left, right, ruido::kBlockFrames);
    }
}

void ruido_fm1_c1_assist_dsp(void) {
    auto* coproc = ruidoCoProc();
    if (!coproc->hasPendingWork()) {
        return;
    }
    (void)coproc->tryAssist();
}

uint32_t ruido_fm1_c1_usb_feed(ruido::Fm1UsbDriftResampler* resampler,
                               const ruido::fm1::RuidoUsbFeedParams* p) {
    if (resampler == nullptr || p == nullptr) {
        return 0u;
    }
    return resampler->processBlockToRing(p->inLrQ15, p->numInFrames, p->rate48k != 0u,
                                         p->fillFrames, p->targetFill, p->uaRingBuf, p->uaRingMask,
                                         p->writePos, p->freeSlots, p->streamSeq);
}

void ruido_fm1_c1_set_helper_armed(uint32_t armed) {
    ruidoCoProc()->setHelperArmed(armed != 0u);
}

void ruido_fm1_cpu1_loop(void) {
    auto* ring = ruidoAudioRing();
    auto* core = ruidoDspCore();
    auto* buses = ruidoSharedBuses();
    auto* rt = ruidoC1Runtime();
    uint32_t idleSpins = 0u;
    for (;;) {
        if (pollSramCanaryFault(16u)) {
            parkOnCore1SelfFault();
        }

        if ((idleSpins & 63u) == 0u) {
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
        }

        if (g_ruido_ic_ctrl.c1FallbackActive.load(std::memory_order_acquire) != 0u) {
            ++idleSpins;
            __asm__ volatile("nop\n\tnop" ::: "memory");
            continue;
        }

        if (g_ruido_ic_ctrl.c1FlashHold.load(std::memory_order_acquire) != 0u) {
            idleSpins = 0u;
            g_ruido_ic_ctrl.c1FlashAck.store(1u, std::memory_order_release);
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
            uint32_t holdSpins = 0u;
            while (g_ruido_ic_ctrl.c1FlashHold.load(std::memory_order_acquire) != 0u) {
                if ((++holdSpins & 31u) == 0u) {
                    ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
                }
                __asm__ volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop" ::
                                     : "memory");
            }
            g_ruido_ic_ctrl.c1FlashAck.store(0u, std::memory_order_release);
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
            continue;
        }

        if (g_ruido_ic_ctrl.c1BenchReq.load(std::memory_order_acquire) != 0u) {
            idleSpins = 0u;
            const uint32_t dt = runCore1Bench64(core, buses);
            if (pollSramCanaryFault(16u)) {
                parkOnCore1SelfFault();
            }
            g_ruido_ic_ctrl.c1BenchTicks.store(dt, std::memory_order_relaxed);
            g_ruido_ic_ctrl.c1BenchReq.store(0u, std::memory_order_release);
            ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.engineReady);
            continue;
        }

        if (g_ruido_ic_ctrl.c1RunEnable.load(std::memory_order_acquire) == 0u) {
            ++idleSpins;
            __asm__ volatile("nop\n\tnop\n\tnop\n\tnop" ::: "memory");
            continue;
        }

        ruido::Fm1AudioBlock* slot = ring->beginPush();
        if (slot == nullptr) {
            /* Paced strictly by ALNK0 24 MHz crystal DMA draining slots on Core 0 (never `idle`!)
             */
            ++idleSpins;
            __asm__ volatile("nop\n\tnop\n\tnop\n\tnop" ::: "memory");
            continue;
        }
        idleSpins = 0u;

        const uint32_t t0 = *kTimer4CntReg;
        renderQuantizedBlock64(*slot);
        const uint32_t dt = *kTimer4CntReg - t0;

        if (pollSramCanaryFault(16u)) {
            parkOnCore1SelfFault();
        }

        ruido::fm1::hwMemoryBarrier();
        ring->commitPush();
        ruido::fm1::hwMemoryBarrier();

        g_ruido_ic_ctrl.c1Last64Ticks.store(dt, std::memory_order_relaxed);
        ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c0ResetMaxReq);
        const uint32_t resetSeq = g_ruido_ic_ctrl.c0ResetMaxReq.load(std::memory_order_acquire);
        uint32_t prevMax = g_ruido_ic_ctrl.c1Max64Ticks.load(std::memory_order_relaxed);
        if (resetSeq != rt->seenResetSeq) {
            rt->seenResetSeq = resetSeq;
            prevMax = 0u;
        }
        if (dt > prevMax) {
            g_ruido_ic_ctrl.c1Max64Ticks.store(dt, std::memory_order_relaxed);
        }
        const uint32_t nextBlocks = g_ruido_ic_ctrl.c1Blocks.load(std::memory_order_relaxed) + 1u;
        g_ruido_ic_ctrl.c1Blocks.store(nextBlocks, std::memory_order_release);
        ruido::fm1::flushCacheLine(&g_ruido_ic_ctrl.c1Blocks);
    }
}

} // extern "C"
