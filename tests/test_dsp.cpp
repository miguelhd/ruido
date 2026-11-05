#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"
#include "dsp/DroneEngine.h"
#include "dsp/HalfBandDecimator.h"
#include "dsp/Ms20Filter.h"
#include "dsp/SchmittOscillatorBank.h"
#include "dsp/VoiceLfoBank.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>

namespace {

int gFailures = 0;

#define EXPECT_TRUE(cond, msg, ...)                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("  [FAIL] %s:%d: " msg "\n", __FILE__, __LINE__, ##__VA_ARGS__);           \
            ++gFailures;                                                                           \
        }                                                                                          \
    } while (false)

bool isSubnormalFloat(float x) {
    uint32_t bits = 0u;
    std::memcpy(&bits, &x, sizeof(bits));
    const uint32_t expBits = bits & 0x7F800000u;
    const uint32_t mantBits = bits & 0x007FFFFFu;
    return (expBits == 0u) && (mantBits != 0u);
}

bool isFiniteNonSubnormal(float x) {
    return std::isfinite(x) && !isSubnormalFloat(x);
}

// In-place Radix-2 Cooley-Tukey FFT (double precision allowed strictly in host test bench
// for 120+ dB spectral measurement accuracy).
void radix2Fft(double* re, double* im, size_t n) {
    size_t j = 0u;
    for (size_t i = 1u; i < n; ++i) {
        size_t bit = n >> 1u;
        while ((j & bit) != 0u) {
            j ^= bit;
            bit >>= 1u;
        }
        j ^= bit;
        if (i < j) {
            const double tr = re[i];
            re[i] = re[j];
            re[j] = tr;
            const double ti = im[i];
            im[i] = im[j];
            im[j] = ti;
        }
    }

    for (size_t len = 2u; len <= n; len <<= 1u) {
        const double ang = -2.0 * 3.14159265358979323846 / static_cast<double>(len);
        const double wlenRe = std::cos(ang);
        const double wlenIm = std::sin(ang);
        for (size_t i = 0u; i < n; i += len) {
            double wRe = 1.0;
            double wIm = 0.0;
            const size_t half = len >> 1u;
            for (size_t k = 0u; k < half; ++k) {
                const double uRe = re[i + k];
                const double uIm = im[i + k];
                const double vRe = re[i + k + half] * wRe - im[i + k + half] * wIm;
                const double vIm = re[i + k + half] * wIm + im[i + k + half] * wRe;
                re[i + k] = uRe + vRe;
                im[i + k] = uIm + vIm;
                re[i + k + half] = uRe - vRe;
                im[i + k + half] = uIm - vIm;
                const double nextWRe = wRe * wlenRe - wIm * wlenIm;
                wIm = wRe * wlenIm + wIm * wlenRe;
                wRe = nextWRe;
            }
        }
    }
}

// 1. Static memory footprint & lock-free IPC verification
void testMemoryFootprintAndLockFreeBus() {
    std::printf("[Test 1] Static SRAM footprint & lock-free IPC buses...\n");
    std::printf("  sizeof(SchmittOscillatorBank)    = %zu bytes\n",
                sizeof(ruido::SchmittOscillatorBank));
    std::printf("  sizeof(Ms20DualFilter)           = %zu bytes\n", sizeof(ruido::Ms20DualFilter));
    std::printf("  sizeof(StereoHalfBandDecimator4x)= %zu bytes\n",
                sizeof(ruido::StereoHalfBandDecimator4x));
    std::printf("  sizeof(DroneEngine) total        = %zu bytes (%.2f KB)\n",
                sizeof(ruido::DroneEngine),
                static_cast<double>(sizeof(ruido::DroneEngine)) / 1024.0);

    // Total engine (including 2048-frame 16-bit 4-voice + stereo scope ring) must stay < 48 KB
    EXPECT_TRUE(sizeof(ruido::DroneEngine) < 49152u, "DroneEngine exceeded 48 KB SRAM budget");

    // Verify LockFreeControlBus never drops latest published state
    ruido::LockFreeControlBus bus;
    for (int i = 0; i < 5000; ++i) {
        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.voices[0].frequencyHz = 100.0f + static_cast<float>(i);
        st.voices[3].lpResonance = 0.0001f * static_cast<float>(i % 1000);
        bus.publish(st);
        if ((i % 7) == 0) {
            const auto& readSt = bus.consumeLatest();
            EXPECT_TRUE(std::fabs(readSt.voices[0].frequencyHz - (100.0f + static_cast<float>(i))) <
                            1e-4f,
                        "ControlBus lost latest state at iter %d", i);
        }
    }

    // Verify ScopeRingBuffer FIFO order & overflow resilience
    ruido::ScopeRingBuffer ring;
    for (size_t i = 0; i < ruido::kScopeRingCapacity + 64u; ++i) {
        const bool ok = ring.push(
            {static_cast<float>(i), -static_cast<float>(i), static_cast<uint8_t>(i & 1u)});
        if (i < ruido::kScopeRingCapacity) {
            EXPECT_TRUE(ok, "ScopeRingBuffer failed push before full at %zu", i);
        } else {
            EXPECT_TRUE(!ok, "ScopeRingBuffer accepted push while full at %zu", i);
        }
    }
    std::array<ruido::ScopeFrame, 128> popped{};
    const size_t nPop = ring.popBatch(popped.data(), popped.size());
    EXPECT_TRUE(nPop == 128u && popped[0].left == 0.0f && popped[127].left == 127.0f,
                "ScopeRingBuffer FIFO ordering mismatch");

    // Multi-threaded Core 0 (UI writer / scope reader) <-> Core 1 (Audio DSP) stress test
    static ruido::DroneEngine mtEngine;
    mtEngine.init(ruido::kBaseSampleRate);
    {
        ruido::EngineControlState initMt = ruido::EngineControlState::makeDefault();
        initMt.voices[0].frequencyHz = 110.0f;
        initMt.voices[1].frequencyHz = 220.0f;
        mtEngine.controlBus().publish(initMt);
        mtEngine.snapParametersToLatest();
    }
    std::atomic<bool> audioDone{false};
    bool tearDetected = false;

    std::thread core0Thread([&]() {
        uint32_t step = 0u;
        std::array<ruido::ScopeFrame, 64> scopeChunk{};
        while (!audioDone.load(std::memory_order_acquire)) {
            ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
            const float tag = 110.0f + static_cast<float>(step % 250u);
            st.voices[0].frequencyHz = tag;
            st.voices[1].frequencyHz = tag * 2.0f;
            st.voicePressure[0] = 0.01f * static_cast<float>(step % 100u);
            mtEngine.controlBus().publish(st);
            (void)mtEngine.scopeBuffer().available();
            (void)mtEngine.scopeBuffer().popBatch(scopeChunk.data(), scopeChunk.size());
            (void)mtEngine.telemetry().peakLeft.load(std::memory_order_relaxed);
            ++step;
        }
    });

    std::thread core1Thread([&]() {
        std::array<float, ruido::kBlockFrames> outL{};
        std::array<float, ruido::kBlockFrames> outR{};
        for (int b = 0; b < 600; ++b) {
            const auto& snap = mtEngine.controlBus().consumeLatest();
            if (std::fabs(snap.voices[1].frequencyHz - 2.0f * snap.voices[0].frequencyHz) > 1e-3f) {
                tearDetected = true;
            }
            mtEngine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        }
        audioDone.store(true, std::memory_order_release);
    });

    core1Thread.join();
    core0Thread.join();
    EXPECT_TRUE(!tearDetected, "Torn multi-parameter read detected across threads");
}

// 2. Sub-audio 0.05 Hz .. 0.50 Hz oscillator & 0.01 Hz .. 0.50 Hz LFO precision
void testUltraSlowLfoPrecision() {
    std::printf("[Test 2] Sub-audio LFO & oscillator precision (0.02 Hz, 0.05 Hz & 0.50 Hz)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, true);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.global.circuitCoupling = 0.0f;
    st.voices[0].frequencyHz = 0.50f; // Unbroken sub-audio oscillator: 2.000 s period!
    st.voices[0].lfoRateHz = 0.50f;   // 2.000 s period = 88,236 base frames
    st.voices[0].lfoDepth = 1.0f;
    st.voices[0].lfoShape = 0.75f; // Slewed Square
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> scopeOut{};

    int firstWrapFrame = -1;
    int secondWrapFrame = -1;
    bool sawLfoHigh = false;
    bool sawLfoLow = false;
    float prevLfoPhase = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    int firstVoiceEdge = -1;
    int secondVoiceEdge = -1;

    const int totalFrames05Hz = static_cast<int>(ruido::kBaseSampleRate * 4.5f);
    for (int baseFrame = 0; baseFrame < totalFrames05Hz;
         baseFrame += static_cast<int>(ruido::kBlockFrames)) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        const float lfoOut = engine.telemetry().lfoOutput[0].load(std::memory_order_relaxed);
        const float curPhase = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
        if (lfoOut > 0.5f) {
            sawLfoHigh = true;
        } else if (lfoOut < -0.5f) {
            sawLfoLow = true;
        }
        if (prevLfoPhase > 0.80f && curPhase < 0.20f) {
            const int endFrame = baseFrame + static_cast<int>(ruido::kBlockFrames);
            if (firstWrapFrame < 0) {
                firstWrapFrame = endFrame;
            } else if (secondWrapFrame < 0) {
                secondWrapFrame = endFrame;
            }
        }
        prevLfoPhase = curPhase;

        const size_t n = engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size());
        for (size_t i = 0u; i < n; ++i) {
            if ((scopeOut[i].voiceRisingMask & 1u) != 0u &&
                (baseFrame + static_cast<int>(i)) > 200) {
                if (firstVoiceEdge < 0) {
                    firstVoiceEdge = baseFrame + static_cast<int>(i);
                } else if (secondVoiceEdge < 0) {
                    secondVoiceEdge = baseFrame + static_cast<int>(i);
                }
            }
        }
    }

    EXPECT_TRUE(sawLfoHigh && sawLfoLow, "0.50 Hz VoiceLfoBank failed bipolar swing");
    EXPECT_TRUE(firstWrapFrame > 0 && secondWrapFrame > firstWrapFrame,
                "0.50 Hz VoiceLfoBank did not complete two phase wraps");
    if (firstWrapFrame > 0 && secondWrapFrame > firstWrapFrame) {
        const double measuredPeriodSec = static_cast<double>(secondWrapFrame - firstWrapFrame) /
                                         static_cast<double>(ruido::kBaseSampleRate);
        std::printf("  0.50 Hz LFO measured period = %.5f s (expected 2.00000 s)\n",
                    measuredPeriodSec);
        EXPECT_TRUE(std::fabs(measuredPeriodSec - 2.0) < 0.005, "0.50 Hz period inaccurate: %.5f s",
                    measuredPeriodSec);
    }
    EXPECT_TRUE(firstVoiceEdge > 0 && secondVoiceEdge > firstVoiceEdge,
                "0.50 Hz sub-audio Osc 1 did not emit rising edges");
    if (firstVoiceEdge > 0 && secondVoiceEdge > firstVoiceEdge) {
        const double voiceHz = static_cast<double>(ruido::kBaseSampleRate) /
                               static_cast<double>(secondVoiceEdge - firstVoiceEdge);
        EXPECT_TRUE(std::fabs(voiceHz - 0.50) < 0.005,
                    "0.50 Hz sub-audio oscillator inaccurate (got %.4f Hz)", voiceHz);
    }

    // Verify out-of-bounds (< 0.05 Hz) voice frequency clamps cleanly to 0.05 Hz (20.000 s period),
    // while 0.02 Hz LFO wraps cleanly at 50.0 s with zero float32 stall!
    engine.reset();
    engine.setAnalogCharacterEnabled(false, true);
    st.voices[0].frequencyHz = 0.002f; // Below 0.05 Hz floor -> clamps to 0.05 Hz (20.0 s)
    st.voices[0].lfoDepth = 0.0f;
    st.voices[0].lfoRateHz = 0.02f;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();
    while (engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size()) > 0u) {
    }

    int wrapCount002Hz = 0;
    int lastWrap002 = -1;
    int clampEdge1 = -1;
    int clampEdge2 = -1;
    prevLfoPhase = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    const int frames55Sec = static_cast<int>(ruido::kBaseSampleRate * 55.0f);
    for (int baseFrame = 0; baseFrame < frames55Sec;
         baseFrame += static_cast<int>(ruido::kBlockFrames)) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        const size_t n = engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size());
        for (size_t i = 0u; i < n; ++i) {
            if ((scopeOut[i].voiceRisingMask & 1u) != 0u &&
                (baseFrame + static_cast<int>(i)) > 200) {
                if (clampEdge1 < 0) {
                    clampEdge1 = baseFrame + static_cast<int>(i);
                } else if (clampEdge2 < 0) {
                    clampEdge2 = baseFrame + static_cast<int>(i);
                }
            }
        }
        const float curPhase = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
        if (prevLfoPhase > 0.80f && curPhase < 0.20f && baseFrame > 1000) {
            ++wrapCount002Hz;
            lastWrap002 = baseFrame + static_cast<int>(ruido::kBlockFrames);
        }
        prevLfoPhase = curPhase;
    }
    EXPECT_TRUE(clampEdge1 > 0 && clampEdge2 > clampEdge1,
                "Clamped 0.05 Hz Osc 1 did not emit two rising edges across 55 s");
    if (clampEdge1 > 0 && clampEdge2 > clampEdge1) {
        const double clampedPeriod = static_cast<double>(clampEdge2 - clampEdge1) /
                                     static_cast<double>(ruido::kBaseSampleRate);
        EXPECT_TRUE(std::fabs(clampedPeriod - 20.0) < 0.05,
                    "0.05 Hz floor clamp period inaccurate (got %.4f s)", clampedPeriod);
    }
    const double firstWrapSec =
        static_cast<double>(lastWrap002) / static_cast<double>(ruido::kBaseSampleRate);
    std::printf("  0.02 Hz LFO wrapped at t = %.3f s (expected ~50.0 s, wraps=%d)\n", firstWrapSec,
                wrapCount002Hz);
    EXPECT_TRUE(wrapCount002Hz == 1 && std::fabs(firstWrapSec - 50.0) < 0.25,
                "0.02 Hz VoiceLfoBank phase accumulator stalled or drifted (wraps=%d, t=%.3f s)",
                wrapCount002Hz, firstWrapSec);

    ruido::SchmittOscillatorBank slowOsc;
    slowOsc.reset(ruido::kBaseSampleRate);
    slowOsc.setAnalogCharacterEnabled(false, false);
    int oscWraps = 0;
    int oscLastWrap = -1;
    const std::array<float, ruido::kNumOscillators> slowFreqs{0.02f, 20.0f, 20.0f, 20.0f};
    for (int f = 0; f < frames55Sec; ++f) {
        const ruido::OscillatorBankOutput o = slowOsc.processSample(slowFreqs, 0.0f);
        if ((o.risingMask & 1u) != 0u && f > 1000) {
            ++oscWraps;
            oscLastWrap = f;
        }
    }
    const double oscWrapSec =
        static_cast<double>(oscLastWrap) / static_cast<double>(ruido::kBaseSampleRate);
    EXPECT_TRUE(oscWraps == 1 && std::fabs(oscWrapSec - 50.0) < 0.05,
                "0.02 Hz SchmittOscillatorBank phase accumulator stalled (wraps=%d, t=%.3f s)",
                oscWraps, oscWrapSec);
}

// 3. Exact Blackman-Harris 32768-pt FFT Alias Suppression Test (> 70 dB Gate)
void testSpectralAliasSuppression() {
    std::printf("[Test 3] 32768-pt FFT Alias Suppression (< 20 kHz, > 70 dB gate)...\n");
    constexpr size_t kFftSize = 32768u;
    static std::array<double, kFftSize> fftRe{};
    static std::array<double, kFftSize> fftIm{};

    constexpr std::array<float, 4> kTestFreqs = {233.71f, 997.31f, 2345.67f, 3713.91f};

    static ruido::DroneEngine engine;
    for (float f0 : kTestFreqs) {
        engine.init(ruido::kBaseSampleRate);
        // Disable random thermal drift during stationary FFT so harmonic bins stay razor-sharp
        engine.setAnalogCharacterEnabled(false, true);

        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.global.circuitCoupling = 0.0f;
        st.global.monoCollapse = true;
        st.voices[0].frequencyHz = f0;
        st.voices[0].amplitude = ruido::kCleanAmpKnee;
        st.voices[0].pan = 0.0f;
        st.voices[0].hpCutoffHz = 2.0f;
        st.voices[0].hpResonance = 0.0f;
        st.voices[0].lpCutoffHz = 28000.0f;
        st.voices[0].lpResonance = 0.0f;
        st.voices[0].muted = false;
        st.voices[1].muted = true;
        st.voices[2].muted = true;
        st.voices[3].muted = true;

        engine.controlBus().publish(st);
        engine.snapParametersToLatest();

        std::array<float, ruido::kBlockFrames> outL{};
        std::array<float, ruido::kBlockFrames> outR{};
        std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

        // Warm up 64 blocks to settle RC capacitor & FIR rings
        for (int w = 0; w < 64; ++w) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
        }

        // Capture kFftSize decimated samples
        for (size_t offset = 0u; offset < kFftSize; offset += ruido::kBlockFrames) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                fftRe[offset + i] = static_cast<double>(outL[i]);
                fftIm[offset + i] = 0.0;
            }
        }

        // Apply 4-term Nuttall / Blackman-Harris window (-98 dB sidelobe floor)
        for (size_t n = 0u; n < kFftSize; ++n) {
            const double ph = (2.0 * 3.14159265358979323846 * static_cast<double>(n)) /
                              static_cast<double>(kFftSize);
            const double w = 0.35875 - 0.48829 * std::cos(ph) + 0.14128 * std::cos(2.0 * ph) -
                             0.01168 * std::cos(3.0 * ph);
            fftRe[n] *= w;
        }

        radix2Fft(fftRe.data(), fftIm.data(), kFftSize);

        const double binHz =
            static_cast<double>(ruido::kBaseSampleRate) / static_cast<double>(kFftSize);

        // Locate fundamental peak magnitude near f0
        const size_t f0Bin = static_cast<size_t>(std::round(static_cast<double>(f0) / binHz));
        double fundMag = 0.0;
        for (size_t b = (f0Bin > 4u ? f0Bin - 4u : 1u); b <= f0Bin + 4u; ++b) {
            const double mag = std::hypot(fftRe[b], fftIm[b]);
            if (mag > fundMag) {
                fundMag = mag;
            }
        }

        // Scan 60 Hz .. 20,000 Hz, masking ONLY true un-aliased harmonics (k * f0 < Nyquist)
        double worstAliasDb = -200.0;
        double worstAliasFreq = 0.0;
        const size_t minBin = static_cast<size_t>(std::ceil(60.0 / binHz));
        const size_t maxBin = static_cast<size_t>(std::floor(20000.0 / binHz));
        const double nyquist = 0.5 * static_cast<double>(ruido::kBaseSampleRate);

        for (size_t b = minBin; b <= maxBin; ++b) {
            const double freq = static_cast<double>(b) * binHz;
            bool isTrueHarmonic = false;
            for (int k = 1; k <= 1200; ++k) {
                const double harmHz = static_cast<double>(k) * static_cast<double>(f0);
                if (harmHz >= nyquist) {
                    break; // Above Nyquist: any folded image IS an alias and MUST NOT be masked!
                }
                if (std::fabs(freq - harmHz) <= 14.0 * binHz) {
                    isTrueHarmonic = true;
                    break;
                }
            }
            if (isTrueHarmonic) {
                continue;
            }
            const double mag = std::hypot(fftRe[b], fftIm[b]);
            const double dbRel = 20.0 * std::log10(std::max(mag / fundMag, 1.0e-15));
            if (dbRel > worstAliasDb) {
                worstAliasDb = dbRel;
                worstAliasFreq = freq;
            }
        }

        std::printf("  f0 = %7.2f Hz -> worst alias < 20 kHz = %6.2f dBc (at %7.1f Hz)\n",
                    static_cast<double>(f0), worstAliasDb, worstAliasFreq);
        EXPECT_TRUE(worstAliasDb < -75.0,
                    "Alias suppression failed >70 dB gate at %.2f Hz (got %.2f dBc)",
                    static_cast<double>(f0), worstAliasDb);
    }
}

// 4. Parked MS-20 filter passband flatness, unmuted rail resonance & Rev 2 -> Korg35 tracking
void testMs20FilterPassbandAndSmartPark() {
    std::printf(
        "[Test 4] MS-20 Rev 2 -> Korg35 parked flatness, unmuted rails & self-oscillation...\n");
    ruido::Ms20DualFilter filter;

    constexpr std::array<float, 4> kProbeFreqs = {15.0f, 120.0f, 1000.0f, 8000.0f};
    for (float fProbe : kProbeFreqs) {
        filter.reset();
        const size_t totalSamples = 17647u * 3u;
        const size_t measureStart = 17647u;
        double sumInSq = 0.0;
        double sumOutSq = 0.0;

        for (size_t n = 0u; n < totalSamples; ++n) {
            const float x = 0.20f * std::sin(ruido::kTwoPi * fProbe * static_cast<float>(n) *
                                             ruido::kInvOversampledRate);
            const float y =
                filter.processSample(x, 2.0f, 0.0f, 28000.0f, 0.0f, ruido::kInvOversampledRate);
            if (n >= measureStart) {
                sumInSq += static_cast<double>(x) * static_cast<double>(x);
                sumOutSq += static_cast<double>(y) * static_cast<double>(y);
            }
        }
        const double gainDb = 10.0 * std::log10(sumOutSq / sumInSq);
        EXPECT_TRUE(std::fabs(gainDb) < 0.15, "Parked filter not flat at %.0f Hz: %.3f dB",
                    static_cast<double>(fProbe), gainDb);
    }

    // Verify Smart-Park resonance muting is GONE: HP=12 Hz, HP RES=1.0 actively self-oscillates
    // in deep sub-bass, and LP=24 kHz, LP RES=1.0 self-oscillates cleanly in 4x oversampled space!
    filter.reset();
    (void)filter.processSample(0.5f, 12.0f, 1.0f, 28000.0f, 0.0f, ruido::kInvOversampledRate);
    double subTailEnergy = 0.0;
    for (size_t n = 0u; n < 176472u; ++n) {
        const float y =
            filter.processSample(0.0f, 12.0f, 1.0f, 28000.0f, 0.0f, ruido::kInvOversampledRate);
        if (n >= 88236u) {
            subTailEnergy += static_cast<double>(y) * static_cast<double>(y);
        }
    }
    const double subOscRms = std::sqrt(subTailEnergy / 88236.0);
    std::printf("  Unmuted sub-bass HP=12 Hz, Reso=1.0 self-oscillation RMS = %.3f\n", subOscRms);
    EXPECT_TRUE(subOscRms > 0.25 && subOscRms < 2.2,
                "Expected active 12 Hz sub-bass self-oscillation without Smart-Park (RMS=%.3f)",
                subOscRms);

    filter.reset();
    (void)filter.processSample(0.5f, 2.0f, 0.0f, 24000.0f, 1.0f, ruido::kInvOversampledRate);
    double ultraSq = 0.0;
    float ultraPeak = 0.0f;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 2.0f, 0.0f, 24000.0f, 1.0f, ruido::kInvOversampledRate);
        if (!isFiniteNonSubnormal(y)) {
            ultraPeak = 999.0f;
        }
        if (n >= 44118u) {
            ultraSq += static_cast<double>(y) * static_cast<double>(y);
            ultraPeak = std::max(ultraPeak, std::fabs(y));
        }
    }
    const double ultraRms = std::sqrt(ultraSq / 44118.0);
    EXPECT_TRUE(ultraRms > 0.10 && ultraPeak < 3.5f,
                "24 kHz Korg35 ultrasonic self-oscillation unstable (RMS=%.3f, Peak=%.3f)",
                ultraRms, static_cast<double>(ultraPeak));

    // Verify musical Rev 2 zone (reso = 0.55) < 3% pitch tracking AND Korg35 (reso = 0.95) scream
    // across 200 Hz, 1 kHz, and 8 kHz on both independent LPF and independent HPF!
    for (float targetFc : {200.0f, 1000.0f, 8000.0f}) {
        filter.reset();
        (void)filter.processSample(0.02f, 2.0f, 0.0f, targetFc, 0.55f, ruido::kInvOversampledRate);

        constexpr size_t kSettle = 88236u;
        constexpr size_t kMeasure = 88236u; // 0.500 s @ 176.472 kHz
        float prevY = 0.0f;
        int risingZeroCrossings = 0;
        double oscSq = 0.0;

        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y =
                filter.processSample(0.0f, 2.0f, 0.0f, targetFc, 0.55f, ruido::kInvOversampledRate);
            if (n >= kSettle) {
                if (prevY <= 0.0f && y > 0.0f) {
                    ++risingZeroCrossings;
                }
                oscSq += static_cast<double>(y) * static_cast<double>(y);
            }
            prevY = y;
        }
        const double measuredHz = static_cast<double>(risingZeroCrossings) / 0.5;
        const double oscRms = std::sqrt(oscSq / static_cast<double>(kMeasure));
        std::printf("  LPF Rev2 (0.55) Self-Osc @ fc=%6.1f Hz -> measured=%7.1f Hz (RMS=%.3f)\n",
                    static_cast<double>(targetFc), measuredHz, oscRms);
        EXPECT_TRUE(oscRms > 0.15 && oscRms < 1.8,
                    "LPF self-oscillation amplitude out of bounds at %.1f Hz (RMS=%.3f)",
                    static_cast<double>(targetFc), oscRms);
        EXPECT_TRUE(std::fabs(measuredHz - static_cast<double>(targetFc)) <
                        0.03 * static_cast<double>(targetFc),
                    "LPF Rev2 pitch tracking error > 3%% at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), measuredHz);

        // Korg35 feral zone (reso = 0.95): hotter harmonic limit cycle + bounded reactive pull
        filter.reset();
        (void)filter.processSample(0.02f, 2.0f, 0.0f, targetFc, 0.95f, ruido::kInvOversampledRate);
        prevY = 0.0f;
        int k35Cross = 0;
        double k35Sq = 0.0;
        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y =
                filter.processSample(0.0f, 2.0f, 0.0f, targetFc, 0.95f, ruido::kInvOversampledRate);
            if (n >= kSettle) {
                if (prevY <= 0.0f && y > 0.0f) {
                    ++k35Cross;
                }
                k35Sq += static_cast<double>(y) * static_cast<double>(y);
            }
            prevY = y;
        }
        const double k35Hz = static_cast<double>(k35Cross) / 0.5;
        const double k35Rms = std::sqrt(k35Sq / static_cast<double>(kMeasure));
        EXPECT_TRUE(k35Rms > oscRms && k35Rms < 2.2,
                    "LPF Korg35 (0.95) failed hotter scream than Rev2 (%.3f vs %.3f)", k35Rms,
                    oscRms);
        EXPECT_TRUE(k35Hz > 0.80 * static_cast<double>(targetFc) &&
                        k35Hz < 1.05 * static_cast<double>(targetFc),
                    "LPF Korg35 pitch drifted excessively at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), k35Hz);
    }

    // Verify independent HPF self-oscillation pitch tracking across 200 Hz, 1 kHz, and 8 kHz!
    for (float targetFc : {200.0f, 1000.0f, 8000.0f}) {
        filter.reset();
        (void)filter.processSample(0.02f, targetFc, 0.55f, 28000.0f, 0.0f,
                                   ruido::kInvOversampledRate);

        constexpr size_t kSettle = 88236u;
        constexpr size_t kMeasure = 88236u;
        float prevY = 0.0f;
        int risingZeroCrossings = 0;
        double oscSq = 0.0;

        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y = filter.processSample(0.0f, targetFc, 0.55f, 28000.0f, 0.0f,
                                                 ruido::kInvOversampledRate);
            if (n >= kSettle) {
                if (prevY <= 0.0f && y > 0.0f) {
                    ++risingZeroCrossings;
                }
                oscSq += static_cast<double>(y) * static_cast<double>(y);
            }
            prevY = y;
        }
        const double measuredHz = static_cast<double>(risingZeroCrossings) / 0.5;
        const double oscRms = std::sqrt(oscSq / static_cast<double>(kMeasure));
        std::printf("  HPF Rev2 (0.55) Self-Osc @ fc=%6.1f Hz -> measured=%7.1f Hz (RMS=%.3f)\n",
                    static_cast<double>(targetFc), measuredHz, oscRms);
        EXPECT_TRUE(oscRms > 0.15 && oscRms < 1.8,
                    "HPF self-oscillation amplitude out of bounds at %.1f Hz (RMS=%.3f)",
                    static_cast<double>(targetFc), oscRms);
        EXPECT_TRUE(std::fabs(measuredHz - static_cast<double>(targetFc)) <
                        0.03 * static_cast<double>(targetFc),
                    "HPF Rev2 pitch tracking error > 3%% at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), measuredHz);
    }

    // Verify early MS-20 self-oscillation onset (LP Reso=0.45 already sustains limit cycle)
    filter.reset();
    (void)filter.processSample(0.02f, 2.0f, 0.0f, 800.0f, 0.45f, ruido::kInvOversampledRate);
    double earlyOscSq = 0.0;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 2.0f, 0.0f, 800.0f, 0.45f, ruido::kInvOversampledRate);
        if (n >= 44118u) {
            earlyOscSq += static_cast<double>(y) * static_cast<double>(y);
        }
    }
    const double earlyOscRms = std::sqrt(earlyOscSq / 44118.0);
    EXPECT_TRUE(earlyOscRms > 0.25, "Expected sustained self-oscillation by Reso=0.45 (RMS=%.3f)",
                earlyOscRms);

    // Verify simultaneous dual HPF (220 Hz) + LPF (880 Hz) Korg35 self-oscillation screams cleanly!
    filter.reset();
    (void)filter.processSample(0.02f, 220.0f, 0.95f, 880.0f, 0.95f, ruido::kInvOversampledRate);
    double dualSq = 0.0;
    float dualPeak = 0.0f;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 220.0f, 0.95f, 880.0f, 0.95f, ruido::kInvOversampledRate);
        if (n >= 44118u) {
            dualSq += static_cast<double>(y) * static_cast<double>(y);
            dualPeak = std::max(dualPeak, std::fabs(y));
        }
    }
    const double dualRms = std::sqrt(dualSq / 44118.0);
    std::printf("  Dual Korg35 HPF(220)+LPF(880) Self-Oscillation: RMS=%.3f, Peak=%.3f\n", dualRms,
                static_cast<double>(dualPeak));
    EXPECT_TRUE(dualRms > 0.72 && dualPeak < 3.5f,
                "Simultaneous dual self-oscillation out of expected range (RMS=%.3f, Peak=%.3f)",
                dualRms, static_cast<double>(dualPeak));
}

// 5. Extreme torture test: max Reso, max Drive, HP/LP crossover sweeps -> zero NaN/denormals
void testExtremeStabilityAndZeroDenormals() {
    std::printf("[Test 5] Extreme self-oscillation, formant crossover & denormal torture...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    bool allValid = true;
    float maxAbsSample = 0.0f;

    // 3 seconds of chaotic extreme modulation across all 4 voices
    for (int block = 0; block < 2000; ++block) {
        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.global.circuitCoupling = 1.0f;
        st.global.masterTrim = 1.0f;
        const float t = static_cast<float>(block) * 0.01f;
        for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
            const float ph = t + static_cast<float>(i) * 1.3f;
            st.voices[i].frequencyHz = 0.05f + 19999.0f * (0.5f + 0.5f * std::sin(ph));
            st.voices[i].hpCutoffHz = 2.0f + 27990.0f * (0.5f + 0.5f * std::cos(ph * 1.7f));
            st.voices[i].hpResonance = 1.0f;
            st.voices[i].lpCutoffHz = 2.0f + 27990.0f * (0.5f + 0.5f * std::sin(ph * 2.3f));
            st.voices[i].lpResonance = 1.0f;
            st.voices[i].amplitude = 1.0f; // Full 4.5x pre-filter MS-20 overdrive!
            st.voices[i].lfoRateHz = 2000.0f;
            st.voices[i].lfoDepth = 1.0f;
            st.voices[i].lfoTarget = static_cast<ruido::LfoTarget>(i % ruido::kNumLfoATargets);
            st.voices[i].lfoBRateHz = 1750.0f;
            st.voices[i].lfoBDepth = 1.0f;
            st.voices[i].lfoBTarget =
                static_cast<ruido::LfoTarget>((block + i) % ruido::kNumLfoBTargets);
            st.voices[i].pan = std::sin(ph * 0.5f);
            st.voicePressure[i] = 0.5f + 0.5f * std::sin(ph * 3.1f);
        }
        engine.controlBus().publish(st);
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());

        for (size_t n = 0u; n < ruido::kBlockFrames; ++n) {
            if (!isFiniteNonSubnormal(outL[n]) || !isFiniteNonSubnormal(outR[n])) {
                allValid = false;
            }
            maxAbsSample = std::max(maxAbsSample, std::max(std::fabs(outL[n]), std::fabs(outR[n])));
        }
    }

    // Cut resonance to 0 and mute all voices + master kill for 4 seconds to check denormal decay
    ruido::EngineControlState silent = ruido::EngineControlState::makeDefault();
    silent.global.masterRun = false;
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        silent.voices[i].muted = true;
        silent.voices[i].hpResonance = 0.0f;
        silent.voices[i].lpResonance = 0.0f;
    }
    engine.controlBus().publish(silent);

    for (int block = 0; block < 2800; ++block) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t n = 0u; n < ruido::kBlockFrames; ++n) {
            if (!isFiniteNonSubnormal(outL[n]) || !isFiniteNonSubnormal(outR[n])) {
                allValid = false;
            }
        }
    }

    std::printf("  Torture max |sample| = %.4f, finite & subnormal-free = %s\n",
                static_cast<double>(maxAbsSample), allValid ? "YES" : "NO");
    EXPECT_TRUE(allValid, "NaN, Inf, or subnormal float detected during torture/decay test");
    EXPECT_TRUE(maxAbsSample < 1.6f, "Master bus exceeded safe ceiling (max=%.4f)",
                static_cast<double>(maxAbsSample));

    // Adversarial NaN / +-Inf poison test on EngineControlState, fastExp2, and Ms20DualFilter
    const float qNan = std::numeric_limits<float>::quiet_NaN();
    const float pInf = std::numeric_limits<float>::infinity();
    EXPECT_TRUE(std::isfinite(ruido::fastExp2(qNan)) && std::isfinite(ruido::fastExp2(pInf)),
                "fastExp2 failed to sanitize NaN/Inf");

    ruido::EngineControlState nanSt = ruido::EngineControlState::makeDefault();
    nanSt.voices[0].frequencyHz = qNan;
    nanSt.voices[0].hpCutoffHz = qNan;
    nanSt.voices[0].hpResonance = qNan;
    nanSt.voices[0].lpCutoffHz = pInf;
    nanSt.voices[0].lpResonance = -pInf;
    nanSt.voices[0].lfoRateHz = qNan;
    nanSt.voices[0].lfoShape = pInf;
    nanSt.voices[0].lfoDepth = qNan;
    nanSt.voicePressure[0] = qNan;
    nanSt.global.circuitCoupling = qNan;
    engine.controlBus().publish(nanSt);
    for (int b = 0; b < 16; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            if (!isFiniteNonSubnormal(outL[i]) || !isFiniteNonSubnormal(outR[i])) {
                allValid = false;
            }
        }
    }
    ruido::Ms20DualFilter rawFilter;
    const float filtNanOut =
        rawFilter.processSample(qNan, 500.0f, 1.0f, 1500.0f, 1.0f, ruido::kInvOversampledRate);
    const float filtBadRateOut = rawFilter.processSample(4.5f, qNan, -pInf, pInf, 0.0f, qNan);
    const float filtLowRateOut =
        rawFilter.processSample(4.5f, 20.0f, 0.0f, 19500.0f, 0.0f, 1.0f / 32000.0f);
    EXPECT_TRUE(allValid && isFiniteNonSubnormal(filtNanOut) &&
                    isFiniteNonSubnormal(filtBadRateOut) && isFiniteNonSubnormal(filtLowRateOut),
                "NaN/Inf or low sample rate poisoned engine or standalone Ms20DualFilter");
}

// 6. Zero DC offset runaway over 10 seconds of asymmetric CMOS + dual resonant filters
void testZeroDcOffsetRunaway() {
    std::printf("[Test 6] Zero DC offset runaway (10 s continuous drone)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.voices[0].frequencyHz = 3.7f;
    st.voices[1].hpCutoffHz = 120.0f;
    st.voices[1].hpResonance = 0.75f;
    st.voices[1].lpCutoffHz = 900.0f;
    st.voices[1].lpResonance = 0.6f;
    st.voices[2].hpCutoffHz = 250.0f;
    st.voices[2].hpResonance = 0.5f;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    const int totalBlocks = static_cast<int>((ruido::kBaseSampleRate * 10.0f) /
                                             static_cast<float>(ruido::kBlockFrames));
    const int measureStartBlock = totalBlocks / 5;
    double sumL = 0.0;
    double sumR = 0.0;
    size_t count = 0u;

    for (int b = 0; b < totalBlocks; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= measureStartBlock) {
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                sumL += static_cast<double>(outL[i]);
                sumR += static_cast<double>(outR[i]);
            }
            count += ruido::kBlockFrames;
        }
    }

    const double meanL = sumL / static_cast<double>(count);
    const double meanR = sumR / static_cast<double>(count);
    std::printf("  10 s steady-state DC offset: L = %+.6f, R = %+.6f\n", meanL, meanR);
    EXPECT_TRUE(std::fabs(meanL) < 0.004 && std::fabs(meanR) < 0.004,
                "DC offset runaway detected (L=%+.6f, R=%+.6f)", meanL, meanR);
}

// 7. Clickless mute/gate, MS-20 Filter Accent / Vactrol LPG pressure & zero pitch deviation
void testClicklessTransitionsAndMonoPoly() {
    std::printf("[Test 7] Clickless vactrol mutes, LPG/MS-20 pressure & zero pitch shift...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, false);

    // Use filtered low-frequency tone (80 Hz, LP=250 Hz) so any raw step click stands out sharply
    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.voices[0].frequencyHz = 80.0f;
    st.voices[0].lpCutoffHz = 250.0f;
    st.voices[0].amplitude = 0.65f;
    st.voices[0].pan = -1.0f;
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
    st.global.circuitCoupling = 0.0f;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    for (int w = 0; w < 40; ++w) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    float prevL = outL[ruido::kBlockFrames - 1u];
    float maxStepDelta = 0.0f;

    // Toggle mute abruptly on -> swell via Vactrol LPG (p=0.55) -> release -> MONO collapse
    for (int step = 0; step < 60; ++step) {
        if (step == 5) {
            st.voices[0].muted = true;
            engine.controlBus().publish(st);
        } else if (step == 25) {
            // While muted, swell voice 0 in via Vactrol Low-Pass Gate!
            st.voicePressure[0] = 0.75f;
            engine.controlBus().publish(st);
        } else if (step == 45) {
            st.global.monoCollapse = true;
            engine.controlBus().publish(st);
        }

        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());

        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            const float d = std::fabs(outL[i] - prevL);
            if (d > maxStepDelta) {
                maxStepDelta = d;
            }
            prevL = outL[i];
        }
    }

    // Settle MONO mode for 14 blocks, then verify L == R bit-for-bit + multi-cycle 80 Hz LPG RMS
    bool monoBitIdentical = true;
    double pressureRms = 0.0;
    size_t rmsSamples = 0u;
    for (int w = 0; w < 30; ++w) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (w >= 14) {
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                if (w == 29 && outL[i] != outR[i]) {
                    monoBitIdentical = false;
                }
                pressureRms += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
                ++rmsSamples;
            }
        }
    }
    pressureRms = std::sqrt(pressureRms / static_cast<double>(rmsSamples));

    // Verify Keys 5-8 pressure NEVER changes pitch in default Filter/LPG mode (0.000 Hz shift),
    // and smoothly bends pitch (+24 semitones -> 4.0x frequency) when SEL selects Pitch Bend mode
    auto measureOsc1PeriodSamples = [&](float pressureVal, bool pitchMode) -> double {
        st.global.pitchPressureMode = pitchMode;
        st.voices[0].muted = false;
        st.voices[0].lpCutoffHz = 28000.0f;
        st.voicePressure[0] = pressureVal;
        engine.controlBus().publish(st);
        engine.reset();
        engine.setAnalogCharacterEnabled(false, false);

        size_t totalFrames = 0u;
        size_t firstEdge = 0u;
        size_t lastEdge = 0u;
        size_t edgeCount = 0u;
        for (int b = 0; b < 700; ++b) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            const size_t nPop = engine.scopeBuffer().popBatch(drain.data(), drain.size());
            for (size_t i = 0u; i < nPop; ++i) {
                if (b >= 40 && (drain[i].voiceRisingMask & 1u) != 0u) {
                    if (edgeCount == 0u) {
                        firstEdge = totalFrames;
                    }
                    lastEdge = totalFrames;
                    ++edgeCount;
                }
                ++totalFrames;
            }
        }
        return static_cast<double>(lastEdge - firstEdge) / static_cast<double>(edgeCount - 1u);
    };

    const double periodNoPress = measureOsc1PeriodSamples(0.0f, false);
    const double periodFilterPress = measureOsc1PeriodSamples(1.0f, false);
    const double periodPitchPress = measureOsc1PeriodSamples(1.0f, true);
    const double pitchRatio = periodNoPress / periodPitchPress;

    std::printf("  Max step delta = %.5f, MONO L==R = %s, LPG RMS = %.3f, "
                "dPeriod = %.6f smp, bend = %.3fx\n",
                static_cast<double>(maxStepDelta), monoBitIdentical ? "YES" : "NO", pressureRms,
                std::fabs(periodFilterPress - periodNoPress), pitchRatio);
    EXPECT_TRUE(maxStepDelta < 0.05f, "Audible click discontinuity detected (delta=%.5f)",
                static_cast<double>(maxStepDelta));
    EXPECT_TRUE(monoBitIdentical, "MONO output collapse did not yield L == R");
    EXPECT_TRUE(pressureRms > 0.05, "Vactrol LPG pressure pad failed to swell muted oscillator");
    EXPECT_TRUE(std::fabs(periodFilterPress - periodNoPress) < 1.0e-9,
                "Filter-mode pressure altered oscillator frequency (period %.6f vs %.6f)",
                periodNoPress, periodFilterPress);
    EXPECT_TRUE(std::fabs(pitchRatio - 4.0) < 0.04,
                "SEL Pitch-Bend mode failed expected +24 st (4.0x) bend ratio (got %.4f)",
                pitchRatio);

    // Verify fading a muted high-amplitude (0.90) voice up from 0% via GLO never blips upward
    st = ruido::EngineControlState::makeDefault();
    st.voices[0].amplitude = 0.90f;
    st.voices[0].muted = true;
    for (size_t i = 1u; i < ruido::kNumOscillators; ++i) {
        st.voices[i].muted = true;
    }
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();
    for (int b = 0; b < 8; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    // Simulate +1 clockwise GLO mixer tick: unmute and start fading in at 1%
    st.voices[0].muted = false;
    st.voices[0].amplitude = 0.01f;
    engine.controlBus().publish(st);
    float maxFadePeak = 0.0f;
    for (int b = 0; b < 24; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            const float a = std::fabs(outL[i]);
            if (a > maxFadePeak) {
                maxFadePeak = a;
            }
        }
    }
    EXPECT_TRUE(maxFadePeak < 0.015f,
                "Transient volume blip when fading muted voice up from 0%% (peak=%.4f)",
                static_cast<double>(maxFadePeak));

    // Verify stepless C^1 log-octave frequency glide across discrete encoder steps (zero plateaus,
    // zero overshoot, zero t=0 velocity kink)
    ruido::LogFreqSmoother logSm;
    logSm.init(55.0f, 0.014f, ruido::kBaseSampleRate);
    float prevSmHz = 55.0f;
    bool stalledBetweenSteps = false;
    bool overshotTarget = false;
    float targetSweepHz = 55.0f;
    for (int step = 0; step < 8; ++step) {
        targetSweepHz *= std::pow(2.0f, 0.090f);
        logSm.setTarget(targetSweepHz);
        // 16 x 64-sample blocks (~23.2 ms between discrete encoder detents)
        for (int b = 0; b < 16; ++b) {
            float blkHz = prevSmHz;
            for (size_t s = 0u; s < ruido::kBlockFrames; ++s) {
                blkHz = logSm.next();
                if (blkHz > targetSweepHz + 1.0e-3f) {
                    overshotTarget = true;
                }
            }
            if ((blkHz - prevSmHz) < 1.0e-3f) {
                stalledBetweenSteps = true;
            }
            prevSmHz = blkHz;
        }
    }
    EXPECT_TRUE(!stalledBetweenSteps, "LogFreqSmoother stalled into stair-step plateau mid-sweep");
    EXPECT_TRUE(!overshotTarget, "LogFreqSmoother overshot target frequency");
}

// 8. CD40106 Shared Power-Rail Coupling (Weak Injection Locking via ALGORITHM parameter)
void testPowerRailInjectionLocking() {
    std::printf("[Test 8] CD40106 shared Vdd rail coupling (injection locking)...\n");
    ruido::SchmittOscillatorBank bank;
    bank.setAnalogCharacterEnabled(false, false);

    const std::array<float, ruido::kNumOscillators> freqs = {200.0f, 200.9f, 55.0f, 55.0f};
    const size_t simSamples = static_cast<size_t>(ruido::kOversampledRate * 2.0f); // 2.0 s

    auto measureFreqDiff = [&](float coupling) -> double {
        bank.reset(ruido::kOversampledRate);
        int edges0 = 0;
        int edges1 = 0;
        size_t firstEdge0 = 0u;
        size_t lastEdge0 = 0u;
        size_t firstEdge1 = 0u;
        size_t lastEdge1 = 0u;
        float prevG0 = 1.0f;
        float prevG1 = 1.0f;
        const size_t settle = static_cast<size_t>(ruido::kOversampledRate * 0.25f);
        for (size_t n = 0u; n < settle + simSamples; ++n) {
            const auto out = bank.processSample(freqs, coupling);
            if (n >= settle) {
                if (prevG0 < 0.5f && out.gateHigh[0] > 0.5f) {
                    if (edges0 == 0) {
                        firstEdge0 = n;
                    }
                    lastEdge0 = n;
                    ++edges0;
                }
                if (prevG1 < 0.5f && out.gateHigh[1] > 0.5f) {
                    if (edges1 == 0) {
                        firstEdge1 = n;
                    }
                    lastEdge1 = n;
                    ++edges1;
                }
            }
            prevG0 = out.gateHigh[0];
            prevG1 = out.gateHigh[1];
        }
        const double f0 = static_cast<double>(edges0 - 1) *
                          static_cast<double>(ruido::kOversampledRate) /
                          static_cast<double>(lastEdge0 - firstEdge0);
        const double f1 = static_cast<double>(edges1 - 1) *
                          static_cast<double>(ruido::kOversampledRate) /
                          static_cast<double>(lastEdge1 - firstEdge1);
        return std::fabs(f1 - f0);
    };

    const double beatUncoupledHz = measureFreqDiff(0.0f);
    const double beatCoupledHz = measureFreqDiff(1.0f);
    std::printf(
        "  200.0 Hz vs 200.9 Hz beat rate: ALGORITHM=0%% -> %.3f Hz, ALGORITHM=100%% -> %.4f Hz\n",
        beatUncoupledHz, beatCoupledHz);
    EXPECT_TRUE(beatUncoupledHz >= 0.85, "Uncoupled oscillators failed to beat freely");
    EXPECT_TRUE(beatCoupledHz < 0.02,
                "Rail coupling failed to phase-lock nearly-tuned oscillators");
}

// 9. Linear-Phase Decimator Square-Wave Crest Factor Verification
void testDecimatorLinearPhaseCrest() {
    std::printf("[Test 9] Linear-phase decimator square flat-top preservation...\n");

    // Part A: Isolated 4x PolyBLEP square -> StereoHalfBandDecimator4x (0 deg dispersion check)
    ruido::SchmittOscillatorBank bank;
    bank.reset(ruido::kOversampledRate);
    bank.setAnalogCharacterEnabled(false, false);
    ruido::StereoHalfBandDecimator4x decim;
    decim.reset();

    const std::array<float, ruido::kNumOscillators> freqs = {100.0f, 100.0f, 100.0f, 100.0f};
    std::array<ruido::StereoFrame, 4> in4x{};
    for (int w = 0; w < 1024; ++w) {
        for (size_t s = 0u; s < 4u; ++s) {
            const auto o = bank.processSample(freqs, 0.0f);
            in4x[s] = {o.samples[0], o.samples[0]};
        }
        (void)decim.process4x(in4x);
    }

    float decimPeak = 0.0f;
    double decimSqSum = 0.0;
    constexpr int kDecimFrames = 4096;
    for (int n = 0; n < kDecimFrames; ++n) {
        for (size_t s = 0u; s < 4u; ++s) {
            const auto o = bank.processSample(freqs, 0.0f);
            in4x[s] = {o.samples[0], o.samples[0]};
        }
        const ruido::StereoFrame y = decim.process4x(in4x);
        decimPeak = std::max(decimPeak, std::fabs(y.left));
        decimSqSum += static_cast<double>(y.left) * static_cast<double>(y.left);
    }
    const double decimRms = std::sqrt(decimSqSum / static_cast<double>(kDecimFrames));
    const double decimCrest = static_cast<double>(decimPeak) / decimRms;

    // Part B: Full DroneEngine crest (includes 2.5 Hz AC-coupling 2-pole Butterworth HPF)
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, false);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.global.circuitCoupling = 0.0f;
    st.global.masterTrim = 1.0f;
    st.global.monoCollapse = true;
    st.voices[0].frequencyHz = 100.0f;
    st.voices[0].amplitude = ruido::kCleanAmpKnee;
    st.voices[0].pan = 0.0f;
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    for (int w = 0; w < 30; ++w) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    float peak = 0.0f;
    double sqSum = 0.0;
    constexpr int kBlocks = 40;
    for (int b = 0; b < kBlocks; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            peak = std::max(peak, std::fabs(outL[i]));
            sqSum += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
        }
    }
    const double rms = std::sqrt(sqSum / static_cast<double>(kBlocks * ruido::kBlockFrames));
    const double crestRatio = static_cast<double>(peak) / rms;

    // Bipolar [-1, +1] square has step jump = 2.0, so exact 0-deg linear-phase Wilbraham-Gibbs
    // crest is 1 + 2 * 0.08949 = 1.179 (Kaiser beta=8.8 yields ~1.159), whereas nonlinear-phase
    // IIR decimators spike to 1.35 - 1.50 even before AC coupling.
    std::printf("  100 Hz square crest: isolated FIR decimator = %.3f, full engine = %.3f\n",
                decimCrest, crestRatio);
    EXPECT_TRUE(decimCrest < 1.19,
                "Excessive phase-dispersion overshoot in FIR decimator (crest=%.3f)", decimCrest);
    EXPECT_TRUE(crestRatio < 1.26, "Excessive full-engine crest factor (crest=%.3f)", crestRatio);
}

// 10. Real-Time Throughput Benchmark
void testThroughputBenchmark() {
    std::printf("[Test 10] Real-time throughput benchmark (10,000 x 64-frame blocks)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        st.voices[i].hpCutoffHz = 65.0f;
        st.voices[i].hpResonance = 0.55f;
        st.voices[i].lpCutoffHz = 1800.0f;
        st.voices[i].lpResonance = 0.75f;
    }
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

#if defined(__has_feature) && __has_feature(thread_sanitizer)
    constexpr int kBenchBlocks = 1000;
    constexpr double kMinRealtimeMultiple = 0.2;
#elif defined(__has_feature) && __has_feature(undefined_behavior_sanitizer)
    constexpr int kBenchBlocks = 10000;
    constexpr double kMinRealtimeMultiple = 2.0;
#else
    constexpr int kBenchBlocks = 10000;
    constexpr double kMinRealtimeMultiple = 15.0;
#endif
    const auto t0 = std::chrono::steady_clock::now();
    float checksum = 0.0f;
    for (int b = 0; b < kBenchBlocks; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        checksum += outL[0];
    }
    const auto t1 = std::chrono::steady_clock::now();

    const double elapsedSec = std::chrono::duration<double>(t1 - t0).count();
    const double audioSec = static_cast<double>(kBenchBlocks * ruido::kBlockFrames) /
                            static_cast<double>(ruido::kBaseSampleRate);
    const double nsPerFrame =
        (elapsedSec * 1.0e9) / static_cast<double>(kBenchBlocks * ruido::kBlockFrames);

    std::printf(
        "  Rendered %.2f s of 4x stereo audio in %.4f s (%.1fx RT, %.1f ns/frame, chk=%.2f)\n",
        audioSec, elapsedSec, audioSec / elapsedSec, nsPerFrame, static_cast<double>(checksum));
    EXPECT_TRUE((audioSec / elapsedSec) > kMinRealtimeMultiple, "Throughput unexpectedly slow");
}

// 11. Per-Voice Analog LFO Physics, C^2 B-Spline Drift & 7-Target Modulation
void testPerVoiceAnalogLfoModels() {
    std::printf("[Test 11] Per-voice analog LFO waveforms, C^2 B-spline drift & 7 targets...\n");

    // 1. 30-bit signed 6-bit drift knot packing/unpacking roundtrip
    const std::array<int8_t, 5> zeroKnots{0, 0, 0, 0, 0};
    EXPECT_TRUE(ruido::packDriftKnots(zeroKnots) == 0u, "Zero drift knots must pack to 0u");
    EXPECT_TRUE(ruido::unpackDriftKnots(0u) == zeroKnots, "0u must unpack to all-zero knots");

    const std::array<int8_t, 5> testKnots{-31, -14, 0, 19, 31};
    const uint32_t packed = ruido::packDriftKnots(testKnots);
    EXPECT_TRUE((packed >> 30u) == 0u, "Packed drift knots must fit in 30 bits");
    EXPECT_TRUE(ruido::unpackDriftKnots(packed) == testKnots, "Drift knot pack/unpack mismatch");

    // 2. Periodic waveform landmarks (Sine, Triangle, Sawtooth, Slewed Square):
    //    verify exact +-1 peak normalization, strict [-1, +1] bounds, and zero DC integral.
    constexpr int kPhaseSamples = 4096;
    for (float shape : {0.00f, 0.25f, 0.50f, 0.75f}) {
        float minV = 10.0f;
        float maxV = -10.0f;
        double sum = 0.0;
        float peakPhase = 0.0f;
        for (int i = 0; i < kPhaseSamples; ++i) {
            const float ph = static_cast<float>(i) / static_cast<float>(kPhaseSamples);
            const float v = ruido::evalMorphedLfoWave(ph, shape, 4.0f, zeroKnots);
            if (v > maxV) {
                maxV = v;
                peakPhase = ph;
            }
            minV = std::min(minV, v);
            sum += static_cast<double>(v);
        }
        const double dcMean = sum / static_cast<double>(kPhaseSamples);
        EXPECT_TRUE(maxV > 0.995f && maxV <= 1.00001f,
                    "LFO shape %.2f positive peak out of spec (%.5f)", static_cast<double>(shape),
                    static_cast<double>(maxV));
        EXPECT_TRUE(minV < -0.995f && minV >= -1.00001f,
                    "LFO shape %.2f negative peak out of spec (%.5f)", static_cast<double>(shape),
                    static_cast<double>(minV));
        EXPECT_TRUE(std::fabs(dcMean) < 1.0e-5, "LFO shape %.2f non-zero DC integral (%+.7f)",
                    static_cast<double>(shape), dcMean);
        if (shape == 0.25f) {
            EXPECT_TRUE(std::fabs(peakPhase - 0.25f) < 0.01f,
                        "Triangle crest expected at phase ~0.25, got %.4f",
                        static_cast<double>(peakPhase));
        } else if (shape == 0.50f) {
            EXPECT_TRUE(peakPhase > 0.44f && peakPhase < 0.50f,
                        "Sawtooth crest expected near capacitor flyback (>0.44), got %.4f",
                        static_cast<double>(peakPhase));
        }
    }

    // 3. VoiceLfoBank 32-bit phase accumulator precision (10.0 Hz & 0.02 Hz) + quadrature init
    {
        ruido::VoiceLfoBank bank;
        bank.setAnalogCharacterEnabled(false);
        bank.reset(ruido::kBaseSampleRate);

        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.voices[0].lfoRateHz = 10.0f;
        st.voices[0].lfoShape = 0.0f;
        st.voices[0].lfoDepth = 1.0f;
        st.voices[1].lfoRateHz = 0.02f;
        bank.updateTargets(st, true);

        const auto initFrame = bank.stepFrame();
        EXPECT_TRUE(std::fabs(initFrame[0].phase01 - 0.00f) < 1.0e-5f &&
                        std::fabs(initFrame[1].phase01 - 0.25f) < 1.0e-5f &&
                        std::fabs(initFrame[2].phase01 - 0.50f) < 1.0e-5f &&
                        std::fabs(initFrame[3].phase01 - 0.75f) < 1.0e-5f,
                    "Expected quadrature {0.0, 0.25, 0.50, 0.75} LFO phase initialization");

        float prevRaw0 = initFrame[0].raw;
        size_t firstCross = 0u;
        size_t lastCross = 0u;
        size_t crossCount = 0u;
        const size_t twoSecSamples = static_cast<size_t>(ruido::kBaseSampleRate * 2.0f);
        for (size_t n = 1u; n <= twoSecSamples; ++n) {
            const auto fr = bank.stepFrame();
            if (prevRaw0 <= 0.0f && fr[0].raw > 0.0f) {
                if (crossCount == 0u) {
                    firstCross = n;
                }
                lastCross = n;
                ++crossCount;
            }
            prevRaw0 = fr[0].raw;
        }
        const double measuredHz = static_cast<double>(crossCount - 1u) *
                                  static_cast<double>(ruido::kBaseSampleRate) /
                                  static_cast<double>(lastCross - firstCross);
        std::printf("  10.00 Hz LFO measured = %.5f Hz (%zu cycles)\n", measuredHz, crossCount);
        EXPECT_TRUE(std::fabs(measuredHz - 10.0) < 0.005,
                    "10 Hz LFO period error too large (got %.5f Hz)", measuredHz);
    }

    // 4. Non-repeating C^2 Uniform Cubic B-Spline Smooth Drift across 32 wraps (64 spline segments)
    {
        ruido::VoiceLfoBank bank;
        bank.setAnalogCharacterEnabled(true);
        bank.reset(ruido::kBaseSampleRate);

        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            st.voices[v].lfoRateHz = 8.0f;
            st.voices[v].lfoShape = 1.0f; // Pure C^2 Smooth Drift
            st.voices[v].lfoDepth = 1.0f;
        }
        bank.updateTargets(st, true);

        const size_t totalSamples = static_cast<size_t>(ruido::kBaseSampleRate * 4.0f);
        std::array<float, ruido::kNumOscillators> prev1{};
        std::array<float, ruido::kNumOscillators> prev2{};
        std::array<float, ruido::kNumOscillators> minVal{10.0f, 10.0f, 10.0f, 10.0f};
        std::array<float, ruido::kNumOscillators> maxVal{-10.0f, -10.0f, -10.0f, -10.0f};
        std::array<double, ruido::kNumOscillators> sumVal{};
        float maxFirstDiff = 0.0f;
        float maxSecondDiff = 0.0f;
        uint32_t firstPacked0 = 0u;
        uint32_t lastPacked0 = 0u;
        double dot01 = 0.0;
        double sq0 = 0.0;
        double sq1 = 0.0;

        for (size_t n = 0u; n < totalSamples; ++n) {
            const auto fr = bank.stepFrame();
            if (n == 0u) {
                firstPacked0 = fr[0].driftPacked;
            }
            lastPacked0 = fr[0].driftPacked;

            for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
                const float x = fr[v].raw;
                minVal[v] = std::min(minVal[v], x);
                maxVal[v] = std::max(maxVal[v], x);
                sumVal[v] += static_cast<double>(x);
                if (n >= 1u) {
                    maxFirstDiff = std::max(maxFirstDiff, std::fabs(x - prev1[v]));
                }
                if (n >= 2u) {
                    const float d2 = std::fabs(x - 2.0f * prev1[v] + prev2[v]);
                    maxSecondDiff = std::max(maxSecondDiff, d2);
                }
                prev2[v] = prev1[v];
                prev1[v] = x;
            }
            dot01 += static_cast<double>(fr[0].raw) * static_cast<double>(fr[1].raw);
            sq0 += static_cast<double>(fr[0].raw) * static_cast<double>(fr[0].raw);
            sq1 += static_cast<double>(fr[1].raw) * static_cast<double>(fr[1].raw);
        }

        const double corr01 = dot01 / std::sqrt(sq0 * sq1);
        std::printf(
            "  C^2 Drift @ 8 Hz (32 wraps): max|dx|=%.5f, max|d2x|=%.6f, corr(V0,V1)=%+.3f\n",
            static_cast<double>(maxFirstDiff), static_cast<double>(maxSecondDiff), corr01);
        EXPECT_TRUE(firstPacked0 != lastPacked0, "Smooth Drift knots failed to evolve on wraps");
        EXPECT_TRUE(maxFirstDiff < 0.0025f, "C^1 velocity jump on Smooth Drift wrap (%.5f)",
                    static_cast<double>(maxFirstDiff));
        EXPECT_TRUE(maxSecondDiff < 5.0e-5f, "C^2 curvature jump on Smooth Drift wrap (%.6f)",
                    static_cast<double>(maxSecondDiff));
        EXPECT_TRUE(std::fabs(corr01) < 0.35, "Per-voice Drift LFOs unexpectedly correlated");
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            EXPECT_TRUE((maxVal[v] - minVal[v]) > 1.15f,
                        "Voice %zu Smooth Drift dynamic range too narrow (%.3f)", v,
                        static_cast<double>(maxVal[v] - minVal[v]));
            const double mean = sumVal[v] / static_cast<double>(totalSamples);
            EXPECT_TRUE(std::fabs(mean) < 0.25, "Voice %zu Smooth Drift drifted off zero (%+.3f)",
                        v, mean);
        }
    }

    // 5. Clickless SHAPE morphing & live 7-destination TARGET switching at 100% DEPTH
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, false);

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    {
        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.voices[0].frequencyHz = 110.0f;
        st.voices[0].lpCutoffHz = 700.0f;
        st.voices[0].lpResonance = 0.45f;
        st.voices[0].hpCutoffHz = 90.0f;
        st.voices[0].amplitude = ruido::kCleanAmpKnee;
        st.voices[0].pan = 0.0f;
        st.voices[0].lfoRateHz = 11.0f;
        for (size_t i = 1u; i < ruido::kNumOscillators; ++i) {
            st.voices[i].muted = true;
        }
        auto runLiveSwitchSweep = [&](float depth, float& maxDeltaOut, bool& finiteOut) {
            st.voices[0].lfoShape = 0.0f;
            st.voices[0].lfoDepth = depth;
            st.voices[0].lfoTarget = ruido::LfoTarget::LpCutoff;
            engine.controlBus().publish(st);
            engine.snapParametersToLatest();

            for (int w = 0; w < 30; ++w) {
                engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                engine.scopeBuffer().popBatch(drain.data(), drain.size());
            }

            float prevL = outL[ruido::kBlockFrames - 1u];
            float prevR = outR[ruido::kBlockFrames - 1u];
            maxDeltaOut = 0.0f;
            finiteOut = true;

            // Sweep SHAPE continuously AND cycle through all 8 LFO A + 10 LFO B targets live
            st.voices[0].lfoBRateHz = 7.3f;
            st.voices[0].lfoBDepth = depth * 0.65f;
            for (int b = 0; b < 200; ++b) {
                st.voices[0].lfoShape = static_cast<float>(b) / 199.0f;
                st.voices[0].lfoBShape = 1.0f - st.voices[0].lfoShape;
                if ((b % 20) == 0) {
                    const uint8_t stepIdx = static_cast<uint8_t>(b / 20);
                    st.voices[0].lfoTarget =
                        static_cast<ruido::LfoTarget>(stepIdx % ruido::kNumLfoATargets);
                    st.voices[0].lfoBTarget =
                        static_cast<ruido::LfoTarget>(stepIdx % ruido::kNumLfoBTargets);
                }
                engine.controlBus().publish(st);
                engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                engine.scopeBuffer().popBatch(drain.data(), drain.size());

                for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                    if (!isFiniteNonSubnormal(outL[i]) || !isFiniteNonSubnormal(outR[i])) {
                        finiteOut = false;
                    }
                    maxDeltaOut = std::max(maxDeltaOut, std::fabs(outL[i] - prevL));
                    maxDeltaOut = std::max(maxDeltaOut, std::fabs(outR[i] - prevR));
                    prevL = outL[i];
                    prevR = outR[i];
                }
            }
        };

        float maxSwitchDeltaMod = 0.0f;
        bool finiteModOk = true;
        runLiveSwitchSweep(0.45f, maxSwitchDeltaMod, finiteModOk);

        float maxSwitchDeltaFull = 0.0f;
        bool finiteFullOk = true;
        runLiveSwitchSweep(1.0f, maxSwitchDeltaFull, finiteFullOk);

        std::printf("  Live 7-target + SHAPE morph max |dSample|: 45%%=%.5f, 100%%=%.5f\n",
                    static_cast<double>(maxSwitchDeltaMod),
                    static_cast<double>(maxSwitchDeltaFull));
        EXPECT_TRUE(finiteModOk && finiteFullOk,
                    "Non-finite or subnormal sample during live LFO morph/target switch");
        EXPECT_TRUE(maxSwitchDeltaMod < 0.14f,
                    "Discontinuity click during 45%% LFO target/shape switch (delta=%.5f)",
                    static_cast<double>(maxSwitchDeltaMod));
        EXPECT_TRUE(maxSwitchDeltaFull < 0.45f,
                    "Discontinuity click during 100%% +/-5-oct LFO switch (delta=%.5f)",
                    static_cast<double>(maxSwitchDeltaFull));
    }

    // 6. Acoustic verification of all 7 per-voice LFO targets:
    auto measureMinMaxBlockRms = [&](const ruido::EngineControlState& cfg, double& minRmsOut,
                                     double& maxRmsOut, double& maxLeftDom, double& maxRightDom,
                                     double* maxDiffRatioOut = nullptr) {
        engine.controlBus().publish(cfg);
        engine.reset();
        engine.setAnalogCharacterEnabled(false, false);

        for (int w = 0; w < 24; ++w) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
        }

        minRmsOut = 100.0;
        maxRmsOut = 0.0;
        maxLeftDom = 0.0;
        maxRightDom = 0.0;
        double minDiffRms = 100.0;
        double maxDiffRms = 0.0;
        float prevSmp = outL[ruido::kBlockFrames - 1u];
        // 300 blocks (~0.435 s) covers > 1.7 full cycles at 4.0 Hz
        for (int b = 0; b < 300; ++b) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
            double sqL = 0.0;
            double sqR = 0.0;
            double diffSq = 0.0;
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                sqL += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
                sqR += static_cast<double>(outR[i]) * static_cast<double>(outR[i]);
                const double ds = static_cast<double>(outL[i] - prevSmp);
                diffSq += ds * ds;
                prevSmp = outL[i];
            }
            const double rms =
                std::sqrt((sqL + sqR) / static_cast<double>(2u * ruido::kBlockFrames));
            const double dRms = std::sqrt(diffSq / static_cast<double>(ruido::kBlockFrames));
            minRmsOut = std::min(minRmsOut, rms);
            maxRmsOut = std::max(maxRmsOut, rms);
            minDiffRms = std::min(minDiffRms, dRms);
            maxDiffRms = std::max(maxDiffRms, dRms);
            if (sqL + sqR > 1.0e-6) {
                maxLeftDom = std::max(maxLeftDom, sqL / (sqL + sqR));
                maxRightDom = std::max(maxRightDom, sqR / (sqL + sqR));
            }
        }
        if (maxDiffRatioOut != nullptr) {
            *maxDiffRatioOut = (minDiffRms > 1.0e-6) ? (maxDiffRms / minDiffRms) : 0.0;
        }
    };

    ruido::EngineControlState base = ruido::EngineControlState::makeDefault();
    base.voices[0].frequencyHz = 65.0f;
    base.voices[0].amplitude = 0.85f;
    base.voices[0].pan = 0.0f;
    base.voices[0].lfoRateHz = 4.0f;
    base.voices[0].lfoShape = 0.0f;
    base.voices[0].lfoDepth = 0.0f;
    for (size_t i = 1u; i < ruido::kNumOscillators; ++i) {
        base.voices[i].muted = true;
    }

    double unmodMin = 0.0;
    double unmodMax = 0.0;
    double domL = 0.0;
    double domR = 0.0;
    measureMinMaxBlockRms(base, unmodMin, unmodMax, domL, domR);

    // (a) LpCutoff: parked at 20 kHz, headroom-aware LFO sweeps LPF downward into sub-bass
    {
        auto cfg = base;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::LpCutoff;
        cfg.voices[0].lfoDepth = 0.90f;
        double mn = 0.0;
        double mx = 0.0;
        measureMinMaxBlockRms(cfg, mn, mx, domL, domR);
        EXPECT_TRUE(mn < mx * 0.45,
                    "LpCutoff LFO failed to sweep parked 20 kHz LPF downward (min=%.3f, max=%.3f)",
                    mn, mx);
    }

    // (b) HpCutoff: parked at 20 Hz, headroom-aware LFO sweeps HPF upward past 65 Hz fundamental
    {
        auto cfg = base;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::HpCutoff;
        cfg.voices[0].lfoDepth = 0.90f;
        double mn = 0.0;
        double mx = 0.0;
        measureMinMaxBlockRms(cfg, mn, mx, domL, domR);
        EXPECT_TRUE(mn < mx * 0.50,
                    "HpCutoff LFO failed to sweep parked 20 Hz HPF upward (min=%.3f, max=%.3f)", mn,
                    mx);
    }

    // (c) LpResonance & (d) HpResonance: sweep 0% resonance up into singing MS-20 self-oscillation
    {
        auto cfgLp = base;
        cfgLp.voices[0].lpCutoffHz = 900.0f;
        cfgLp.voices[0].lpResonance = 0.0f;
        double baseLpMin = 0.0;
        double baseLpMax = 0.0;
        measureMinMaxBlockRms(cfgLp, baseLpMin, baseLpMax, domL, domR);

        cfgLp.voices[0].lfoTarget = ruido::LfoTarget::LpResonance;
        cfgLp.voices[0].lfoDepth = 1.0f;
        double modLpMin = 0.0;
        double modLpMax = 0.0;
        double lpDiffRatio = 0.0;
        measureMinMaxBlockRms(cfgLp, modLpMin, modLpMax, domL, domR, &lpDiffRatio);
        EXPECT_TRUE(modLpMax > baseLpMax * 1.24 && lpDiffRatio > 1.80,
                    "LpResonance LFO failed MS-20 LPF peak (base=%.3f, mod=%.3f, hf=%.2fx)",
                    baseLpMax, modLpMax, lpDiffRatio);

        auto cfgHp = base;
        cfgHp.voices[0].hpCutoffHz = 220.0f;
        cfgHp.voices[0].hpResonance = 0.0f;
        double baseHpMin = 0.0;
        double baseHpMax = 0.0;
        measureMinMaxBlockRms(cfgHp, baseHpMin, baseHpMax, domL, domR);

        cfgHp.voices[0].lfoTarget = ruido::LfoTarget::HpResonance;
        cfgHp.voices[0].lfoDepth = 1.0f;
        double modHpMin = 0.0;
        double modHpMax = 0.0;
        measureMinMaxBlockRms(cfgHp, modHpMin, modHpMax, domL, domR);
        EXPECT_TRUE(modHpMax > baseHpMax * 1.30,
                    "HpResonance LFO failed to excite MS-20 HPF peak (base=%.3f, mod=%.3f)",
                    baseHpMax, modHpMax);
    }

    // (e) Amplitude: 100% depth drops trough to near-zero while crest swells cleanly
    {
        auto cfg = base;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::Amplitude;
        cfg.voices[0].lfoDepth = 1.0f;
        double mn = 0.0;
        double mx = 0.0;
        measureMinMaxBlockRms(cfg, mn, mx, domL, domR);
        EXPECT_TRUE(mn < 0.015 && mx > 0.22,
                    "Amplitude LFO failed full tremolo range (min=%.4f, max=%.4f)", mn, mx);
    }

    // (f) Pan: 100% depth orbits stereo image hard-left <-> hard-right with steady total power
    {
        auto cfg = base;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::Pan;
        cfg.voices[0].lfoDepth = 1.0f;
        double mn = 0.0;
        double mx = 0.0;
        measureMinMaxBlockRms(cfg, mn, mx, domL, domR);
        EXPECT_TRUE(domL > 0.98 && domR > 0.98,
                    "Pan LFO failed full hard-L/hard-R orbit (domL=%.3f, domR=%.3f)", domL, domR);
        EXPECT_TRUE(mn > mx * 0.80,
                    "Constant-power Pan LFO dropped stereo energy mid-orbit (min=%.3f, max=%.3f)",
                    mn, mx);
    }

    // (g) Pitch: log-octave vibrato/FM sweeps rising-edge period across +-2 octaves
    {
        auto cfg = base;
        cfg.voices[0].frequencyHz = 220.0f;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::Pitch;
        cfg.voices[0].lfoDepth = 0.65f;
        engine.controlBus().publish(cfg);
        engine.reset();
        engine.setAnalogCharacterEnabled(false, false);

        size_t sampleIdx = 0u;
        size_t prevEdge = 0u;
        bool hasPrev = false;
        size_t minPeriod = 100000u;
        size_t maxPeriod = 0u;
        for (int b = 0; b < 320; ++b) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            const size_t nPop = engine.scopeBuffer().popBatch(drain.data(), drain.size());
            for (size_t i = 0u; i < nPop; ++i) {
                if (b >= 20 && (drain[i].voiceRisingMask & 1u) != 0u) {
                    if (hasPrev) {
                        const size_t per = sampleIdx - prevEdge;
                        minPeriod = std::min(minPeriod, per);
                        maxPeriod = std::max(maxPeriod, per);
                    }
                    prevEdge = sampleIdx;
                    hasPrev = true;
                }
                ++sampleIdx;
            }
        }
        const double octRatio = static_cast<double>(maxPeriod) / static_cast<double>(minPeriod);
        EXPECT_TRUE(octRatio > 3.2, "Pitch LFO failed wide log-octave period sweep (max/min=%.3f)",
                    octRatio);
    }

    // (h) PulseWidth (WIDTH): LFO sweeps Schmitt PWM across 5%-95% with zero DC offset runaway
    {
        auto cfg = base;
        cfg.voices[0].frequencyHz = 110.0f;
        cfg.voices[0].pulseWidth = 0.50f;
        cfg.voices[0].lfoTarget = ruido::LfoTarget::PulseWidth;
        cfg.voices[0].lfoDepth = 1.0f;
        double mn = 0.0;
        double mx = 0.0;
        double pwmDiffRatio = 0.0;
        measureMinMaxBlockRms(cfg, mn, mx, domL, domR, &pwmDiffRatio);
        EXPECT_TRUE(mn < mx * 0.72 && mx > 0.18,
                    "PulseWidth LFO failed 5%%-95%% PWM sweep (min=%.3f, max=%.3f, hf=%.2fx)", mn,
                    mx, pwmDiffRatio);
    }

    // (i) Dual-LFO Cross-Modulation: LFO B -> A RATE warps LFO A period, and LFO B -> A DEPTH
    //     swells LFO A tremolo from near-stillness up to full crest!
    {
        ruido::VoiceLfoBank dualBank;
        dualBank.reset(ruido::kBaseSampleRate);
        dualBank.setAnalogCharacterEnabled(false);

        auto cfgRate = base;
        cfgRate.voices[0].lfoRateHz = 8.0f;
        cfgRate.voices[0].lfoDepth = 1.0f;
        cfgRate.voices[0].lfoBRateHz = 2.0f;
        cfgRate.voices[0].lfoBDepth = 0.75f;
        cfgRate.voices[0].lfoBTarget = ruido::LfoTarget::LfoARate;
        dualBank.updateTargets(cfgRate, true);

        float prevPhaseA = dualBank.stepFrame()[0].phase01;
        size_t smpSinceWrap = 0u;
        size_t minLfoAPeriod = 1000000u;
        size_t maxLfoAPeriod = 0u;
        bool seenFirstWrap = false;
        for (size_t n = 0u; n < 44118u; ++n) {
            const float phA = dualBank.stepFrame()[0].phase01;
            ++smpSinceWrap;
            if (phA + 0.5f < prevPhaseA) {
                if (seenFirstWrap) {
                    minLfoAPeriod = std::min(minLfoAPeriod, smpSinceWrap);
                    maxLfoAPeriod = std::max(maxLfoAPeriod, smpSinceWrap);
                }
                seenFirstWrap = true;
                smpSinceWrap = 0u;
            }
            prevPhaseA = phA;
        }
        const double lfoARateSpan = static_cast<double>(maxLfoAPeriod) /
                                    static_cast<double>(std::max<size_t>(minLfoAPeriod, 1u));
        EXPECT_TRUE(lfoARateSpan > 4.0,
                    "LFO B -> A RATE failed to warp LFO A cycle speed (max/min=%.3f)",
                    lfoARateSpan);

        auto cfgDepth = base;
        cfgDepth.voices[0].lfoTarget = ruido::LfoTarget::Amplitude;
        cfgDepth.voices[0].lfoRateHz = 24.0f;
        cfgDepth.voices[0].lfoDepth = 1.0f;
        cfgDepth.voices[0].lfoBTarget = ruido::LfoTarget::LfoADepth;
        cfgDepth.voices[0].lfoBRateHz = 2.0f;
        cfgDepth.voices[0].lfoBDepth = 1.0f;
        engine.controlBus().publish(cfgDepth);
        engine.reset();
        engine.setAnalogCharacterEnabled(false, false);

        double calmWindowSwing = 100.0;
        double intenseWindowSwing = 0.0;
        for (int win = 0; win < 16; ++win) {
            double winMin = 100.0;
            double winMax = 0.0;
            for (int b = 0; b < 24; ++b) {
                engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                engine.scopeBuffer().popBatch(drain.data(), drain.size());
                double sq = 0.0;
                for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                    sq += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
                }
                const double r = std::sqrt(sq / static_cast<double>(ruido::kBlockFrames));
                winMin = std::min(winMin, r);
                winMax = std::max(winMax, r);
            }
            if (win >= 2) {
                const double swing = winMax - winMin;
                calmWindowSwing = std::min(calmWindowSwing, swing);
                intenseWindowSwing = std::max(intenseWindowSwing, swing);
            }
        }
        EXPECT_TRUE(intenseWindowSwing > calmWindowSwing * 2.5 && intenseWindowSwing > 0.14,
                    "LFO B -> A DEPTH failed to swell/fade LFO A tremolo (calm=%.4f, peak=%.4f)",
                    calmWindowSwing, intenseWindowSwing);
    }
}

// 12. MS-20 Rev 2 (KLM-307 LM13600 OTA Sallen-Key) 0%-RES warmth, DC rejection & 4-lane parity
void testRev2OtaSallenKeyPhysics() {
    std::printf(
        "[Test 12] MS-20 Rev 2 LM13600 OTA 0%%-RES warmth, capacitive DC & SIMD parity...\n");

    // (a) 0%-Resonance LM13600 OTA Harmonic Warmth & Alias Floor (32768-pt Blackman-Harris FFT):
    // Drive a pure 300 Hz full-scale sine through engaged LPF (450 Hz, RES = 0.0) at 4x OS and
    // verify genuine 2nd + 3rd harmonic OTA saturation with < -85 dBc ultrasonic alias floor.
    {
        constexpr size_t kFftSize = 32768u;
        static std::array<double, kFftSize> fftRe{};
        static std::array<double, kFftSize> fftIm{};

        ruido::Ms20DualFilter filter;
        ruido::StereoHalfBandDecimator4x decim;
        filter.reset();
        decim.reset();
        const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(2.0f, 0.0f, 450.0f, 0.0f,
                                                                 ruido::kInvOversampledRate);

        constexpr double kTestHz = 300.0;
        size_t osStep = 0u;
        std::array<ruido::StereoFrame, 4> in4x{};

        for (size_t w = 0u; w < 4096u; ++w) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(std::sin(
                    2.0 * 3.14159265358979323846 * kTestHz * static_cast<double>(osStep++) *
                    static_cast<double>(ruido::kInvOversampledRate)));
                const float y = filter.processPrepared(x, coeffs);
                in4x[s] = {y, y};
            }
            (void)decim.process4x(in4x);
        }

        for (size_t n = 0u; n < kFftSize; ++n) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(std::sin(
                    2.0 * 3.14159265358979323846 * kTestHz * static_cast<double>(osStep++) *
                    static_cast<double>(ruido::kInvOversampledRate)));
                const float y = filter.processPrepared(x, coeffs);
                in4x[s] = {y, y};
            }
            const ruido::StereoFrame dec = decim.process4x(in4x);
            const double ph = (2.0 * 3.14159265358979323846 * static_cast<double>(n)) /
                              static_cast<double>(kFftSize);
            const double win = 0.35875 - 0.48829 * std::cos(ph) + 0.14128 * std::cos(2.0 * ph) -
                               0.01168 * std::cos(3.0 * ph);
            fftRe[n] = static_cast<double>(dec.left) * win;
            fftIm[n] = 0.0;
        }

        radix2Fft(fftRe.data(), fftIm.data(), kFftSize);
        const double binHz =
            static_cast<double>(ruido::kBaseSampleRate) / static_cast<double>(kFftSize);

        auto peakNearHz = [&](double targetHz) -> double {
            const size_t center = static_cast<size_t>(std::round(targetHz / binHz));
            double pk = 0.0;
            for (size_t b = (center > 4u ? center - 4u : 1u); b <= center + 4u; ++b) {
                pk = std::max(pk, std::hypot(fftRe[b], fftIm[b]));
            }
            return pk;
        };

        const double h1 = peakNearHz(kTestHz);
        const double h2Db = 20.0 * std::log10(std::max(peakNearHz(2.0 * kTestHz) / h1, 1.0e-15));
        const double h3Db = 20.0 * std::log10(std::max(peakNearHz(3.0 * kTestHz) / h1, 1.0e-15));

        double worstAliasDb = -200.0;
        const size_t minBin = static_cast<size_t>(std::ceil(60.0 / binHz));
        const size_t maxBin = static_cast<size_t>(std::floor(20000.0 / binHz));
        const double nyquist = 0.5 * static_cast<double>(ruido::kBaseSampleRate);
        for (size_t b = minBin; b <= maxBin; ++b) {
            const double freq = static_cast<double>(b) * binHz;
            bool isHarmonic = false;
            for (int m = 1; m <= 100; ++m) {
                const double hm = static_cast<double>(m) * kTestHz;
                if (hm >= nyquist) {
                    break;
                }
                if (std::fabs(freq - hm) <= 12.0 * binHz) {
                    isHarmonic = true;
                    break;
                }
            }
            if (!isHarmonic) {
                const double db =
                    20.0 * std::log10(std::max(std::hypot(fftRe[b], fftIm[b]) / h1, 1.0e-15));
                worstAliasDb = std::max(worstAliasDb, db);
            }
        }

        std::printf("  RES=0%% LM13600 OTA @ 300 Hz -> H2=%.2f dBc, H3=%.2f dBc, alias=%.2f dBc\n",
                    h2Db, h3Db, worstAliasDb);
        EXPECT_TRUE(h3Db > -48.0 && h3Db < -20.0,
                    "Expected warm 3rd-harmonic LM13600 OTA saturation at RES=0%% (got %.2f dBc)",
                    h3Db);
        EXPECT_TRUE(h2Db > -70.0 && h2Db < -24.0,
                    "Expected LM13600 Wilson-mirror 2nd harmonic at RES=0%% (got %.2f dBc)", h2Db);
        EXPECT_TRUE(worstAliasDb < -85.0,
                    "C^2 LM13600 OTA exceeded -85 dBc alias floor (got %.2f dBc)", worstAliasDb);
    }

    // (b) Capacitive C1/C2 Sallen-Key Zero-DC Rejection under +0.85 DC bias + self-oscillation:
    {
        auto measureSteadyMean = [](float dcBias) -> double {
            ruido::Ms20DualFilter filter;
            filter.reset();
            const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(90.0f, 0.65f, 1200.0f, 0.85f,
                                                                     ruido::kInvOversampledRate);
            const size_t totalSamples = static_cast<size_t>(ruido::kOversampledRate * 2.0f);
            const size_t halfSamples = totalSamples / 2u;
            const size_t winLen = totalSamples - halfSamples;
            double sumOut = 0.0;
            double sumWin = 0.0;
            for (size_t n = 0u; n < totalSamples; ++n) {
                const float x =
                    dcBias + 0.65f * std::sin(ruido::kTwoPi * 240.0f * static_cast<float>(n) *
                                              ruido::kInvOversampledRate);
                const float y = filter.processPrepared(x, coeffs);
                if (n >= halfSamples) {
                    const double w = 1.0 - std::cos((2.0 * 3.14159265358979323846 *
                                                     static_cast<double>(n - halfSamples)) /
                                                    static_cast<double>(winLen));
                    sumOut += static_cast<double>(y) * w;
                    sumWin += w;
                }
            }
            return sumOut / sumWin;
        };
        const double meanNoBias = measureSteadyMean(0.0f);
        const double meanWithBias = measureSteadyMean(0.85f);
        const double dcLeak = std::fabs(meanWithBias - meanNoBias);
        std::printf("  +0.85 DC input -> steady DC = %+.6f (leak = %.2e)\n", meanWithBias, dcLeak);
        EXPECT_TRUE(std::fabs(meanWithBias) < 1.5e-3,
                    "Capacitive Sallen-Key limit-cycle DC exceeded 1.5 mV (mean=%+.6f)",
                    meanWithBias);
        EXPECT_TRUE(dcLeak < 1.0e-5,
                    "Capacitive Sallen-Key leaked input DC bias without post-blocker (leak=%.2e)",
                    dcLeak);
    }

    // (c) Exact numerical parity between processQuadBank4x (allBypass / noneBypass / mixed)
    //     and standalone per-sample Ms20DualFilter::processPrepared:
    {
        float maxParityDiff = 0.0f;
        for (int mode = 0; mode < 3; ++mode) {
            std::array<ruido::Ms20DualFilter, 4> bankFilters{};
            std::array<ruido::Ms20DualFilter, 4> refFilters{};
            std::array<ruido::Ms20DualFilter::Coeffs, 4> coeffs{};
            for (size_t v = 0u; v < 4u; ++v) {
                bankFilters[v].reset();
                refFilters[v].reset();
                if (mode == 0) {
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(2.0f, 0.0f, 28000.0f, 0.0f,
                                                                     ruido::kInvOversampledRate);
                } else if (mode == 1) {
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(
                        75.0f + 55.0f * static_cast<float>(v), 0.72f,
                        600.0f + 450.0f * static_cast<float>(v), 0.88f, ruido::kInvOversampledRate,
                        2.8f);
                } else {
                    const float hp = (v & 1u) != 0u ? 180.0f : 2.0f;
                    const float hpRes = (v & 1u) != 0u ? 0.85f : 0.0f;
                    const float lp = (v & 2u) != 0u ? 950.0f : 28000.0f;
                    const float lpRes = (v & 2u) != 0u ? 0.85f : 0.0f;
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(hp, hpRes, lp, lpRes,
                                                                     ruido::kInvOversampledRate);
                }
            }

            for (size_t frame = 0u; frame < 1024u; ++frame) {
                std::array<std::array<float, 4>, 4> voiceSub{};
                std::array<std::array<float, 4>, 4> refOut{};
                for (size_t sub = 0u; sub < 4u; ++sub) {
                    const float t = static_cast<float>(frame * 4u + sub) * 0.037f;
                    for (size_t v = 0u; v < 4u; ++v) {
                        float sig = 0.0f;
                        if (frame < 512u) {
                            sig = (std::sin(t * (1.0f + 0.3f * static_cast<float>(v))) >= 0.0f)
                                      ? 0.95f
                                      : -0.95f;
                        }
                        voiceSub[v][sub] = sig;
                        refOut[v][sub] = refFilters[v].processPrepared(sig, coeffs[v]);
                    }
                }
                ruido::Ms20DualFilter::processQuadBank4x(bankFilters, voiceSub, coeffs);
                for (size_t v = 0u; v < 4u; ++v) {
                    for (size_t sub = 0u; sub < 4u; ++sub) {
                        maxParityDiff =
                            std::max(maxParityDiff, std::fabs(voiceSub[v][sub] - refOut[v][sub]));
                    }
                }
            }
        }
        std::printf("  processQuadBank4x vs processPrepared max |diff| = %.3e\n",
                    static_cast<double>(maxParityDiff));
        EXPECT_TRUE(maxParityDiff == 0.0f,
                    "SIMD processQuadBank4x diverged from scalar processPrepared (diff=%.3e)",
                    static_cast<double>(maxParityDiff));
    }

    // (d) Hot standalone input (±10.0f) bypass handoff continuity across 27.85..28.00 kHz:
    {
        ruido::Ms20DualFilter filter;
        filter.reset();
        float prevY = 0.0f;
        float maxHandoffStep = 0.0f;
        for (size_t n = 0u; n < 4000u; ++n) {
            const float lpHz = 27850.0f + 0.05f * static_cast<float>(n); // crosses ~27972 Hz
            const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(2.0f, 0.0f, lpHz, 0.0f,
                                                                     ruido::kInvOversampledRate);
            const float x = 10.0f * std::sin(ruido::kTwoPi * 3000.0f * static_cast<float>(n) *
                                             ruido::kInvOversampledRate);
            const float y = filter.processPrepared(x, coeffs);
            if (n > 64u) {
                maxHandoffStep = std::max(maxHandoffStep, std::fabs(y - prevY));
            }
            prevY = y;
        }
        EXPECT_TRUE(maxHandoffStep < 1.25f, "Hot ±10V bypass handoff clicked (max |dy| = %.4f)",
                    static_cast<double>(maxHandoffStep));
    }
}

// 13. Unbroken Extremes: 15 Hz sub-bass, EDIT 4.5x overdrive, 100% ALGORITHM sync & 2 kHz FM LFO
void testUnbrokenExtremesSubBassOverdriveSyncAndAudioLfo() {
    std::printf(
        "[Test 13] Unbroken 15 Hz sub-bass, 4.5x MS-20 drive, 100%% sync & audio-rate LFO FM...\n");

    // (a) 15 Hz deep sub-bass flatness (< 0.15 dB loss with 1.5 Hz RC coupling capacitor):
    {
        ruido::SchmittOscillatorBank flatBank;
        ruido::SchmittOscillatorBank rcBank;
        flatBank.reset(ruido::kOversampledRate);
        rcBank.reset(ruido::kOversampledRate);
        flatBank.setAnalogCharacterEnabled(false, false);
        rcBank.setAnalogCharacterEnabled(false, true);

        const std::array<float, ruido::kNumOscillators> subFreqs{15.0f, 55.0f, 55.0f, 55.0f};
        const size_t settle = static_cast<size_t>(ruido::kOversampledRate * 1.5f);
        const size_t measure = static_cast<size_t>(ruido::kOversampledRate * 1.0f);
        double sqFlat = 0.0;
        double sqRc = 0.0;
        for (size_t n = 0u; n < settle + measure; ++n) {
            const auto oFlat = flatBank.processSample(subFreqs, 0.0f);
            const auto oRc = rcBank.processSample(subFreqs, 0.0f);
            if (n >= settle) {
                sqFlat +=
                    static_cast<double>(oFlat.samples[0]) * static_cast<double>(oFlat.samples[0]);
                sqRc += static_cast<double>(oRc.samples[0]) * static_cast<double>(oRc.samples[0]);
            }
        }
        const double subLossDb = 10.0 * std::log10(sqRc / sqFlat);
        std::printf("  15.0 Hz sub-bass RC coupling gain = %+.3f dB\n", subLossDb);
        EXPECT_TRUE(std::fabs(subLossDb) < 0.15,
                    "1.5 Hz RC coupling pole attenuated 15 Hz sub-bass (%.3f dB)", subLossDb);
    }

    // (b) EDIT 70-100% (4.5x) Pre-Filter Overdrive engages on wide-open filter (HP=2, LP=28k)
    //     generating rich asymmetric 2nd + 3rd harmonics while capacitive HPF strips 100% of DC:
    {
        constexpr size_t kFftSize = 16384u;
        static std::array<double, kFftSize> fftRe{};
        static std::array<double, kFftSize> fftIm{};

        ruido::Ms20DualFilter filter;
        ruido::StereoHalfBandDecimator4x decim;
        filter.reset();
        decim.reset();
        const auto cleanCoeffs = ruido::Ms20DualFilter::prepareCoeffs(
            2.0f, 0.0f, 28000.0f, 0.0f, ruido::kInvOversampledRate, 1.0f);
        const auto driveCoeffs = ruido::Ms20DualFilter::prepareCoeffs(
            2.0f, 0.0f, 28000.0f, 0.0f, ruido::kInvOversampledRate, 4.5f);
        EXPECT_TRUE(cleanCoeffs.hpLinearBypass && cleanCoeffs.lpLinearBypass,
                    "Expected wide-open preDrive=1.0x filter to bypass cleanly");
        EXPECT_TRUE(!driveCoeffs.hpLinearBypass,
                    "Expected preDrive=4.5x to engage capacitive Stage-1 HPF + overdrive");

        constexpr double kTestHz = 220.0;
        size_t osStep = 0u;
        std::array<ruido::StereoFrame, 4> in4x{};
        for (size_t w = 0u; w < 32768u; ++w) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(
                    0.85 * std::sin(2.0 * 3.14159265358979323846 * kTestHz *
                                    static_cast<double>(osStep++) * ruido::kInvOversampledRate));
                const float y = filter.processPrepared(x, driveCoeffs);
                in4x[s] = {y, y};
            }
            (void)decim.process4x(in4x);
        }

        double dcSum = 0.0;
        for (size_t n = 0u; n < kFftSize; ++n) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(
                    0.85 * std::sin(2.0 * 3.14159265358979323846 * kTestHz *
                                    static_cast<double>(osStep++) * ruido::kInvOversampledRate));
                const float y = filter.processPrepared(x, driveCoeffs);
                in4x[s] = {y, y};
            }
            const ruido::StereoFrame dec = decim.process4x(in4x);
            dcSum += static_cast<double>(dec.left);
            const double ph = (2.0 * 3.14159265358979323846 * static_cast<double>(n)) /
                              static_cast<double>(kFftSize);
            const double win = 0.35875 - 0.48829 * std::cos(ph) + 0.14128 * std::cos(2.0 * ph) -
                               0.01168 * std::cos(3.0 * ph);
            fftRe[n] = static_cast<double>(dec.left) * win;
            fftIm[n] = 0.0;
        }
        radix2Fft(fftRe.data(), fftIm.data(), kFftSize);
        const double binHz =
            static_cast<double>(ruido::kBaseSampleRate) / static_cast<double>(kFftSize);
        auto peakAt = [&](double hz) -> double {
            const size_t c = static_cast<size_t>(std::round(hz / binHz));
            double pk = 0.0;
            for (size_t b = (c > 4u ? c - 4u : 1u); b <= c + 4u; ++b) {
                pk = std::max(pk, std::hypot(fftRe[b], fftIm[b]));
            }
            return pk;
        };
        const double h1 = peakAt(kTestHz);
        const double h2Db = 20.0 * std::log10(std::max(peakAt(2.0 * kTestHz) / h1, 1.0e-15));
        const double h3Db = 20.0 * std::log10(std::max(peakAt(3.0 * kTestHz) / h1, 1.0e-15));
        const double dcMean = dcSum / static_cast<double>(kFftSize);
        std::printf("  4.5x Overdrive @ 220 Hz -> H2=%.2f dBc, H3=%.2f dBc, DC=%+.5f\n", h2Db, h3Db,
                    dcMean);
        EXPECT_TRUE(h2Db > -28.0 && h3Db > -20.0,
                    "Expected rich asymmetric 2nd+3rd harmonics at 4.5x drive (H2=%.2f, H3=%.2f)",
                    h2Db, h3Db);
        EXPECT_TRUE(std::fabs(dcMean) < 0.025,
                    "Asymmetric 4.5x overdrive leaked excessive standalone DC (%+.5f)", dcMean);
    }

    // (c) ALGORITHM = 100% starved-rail subharmonic hard-sync (300 Hz master -> 143 Hz locks 2:1):
    {
        ruido::SchmittOscillatorBank bank;
        bank.setAnalogCharacterEnabled(false, false);
        const std::array<float, ruido::kNumOscillators> syncFreqs{300.0f, 143.0f, 40.0f, 40.0f};
        auto measureRatio01 = [&](float coupling) -> double {
            bank.reset(ruido::kOversampledRate);
            const size_t settle = static_cast<size_t>(ruido::kOversampledRate * 0.25f);
            const size_t run = static_cast<size_t>(ruido::kOversampledRate * 1.5f);
            int c0 = 0;
            int c1 = 0;
            for (size_t n = 0u; n < settle + run; ++n) {
                const auto o = bank.processSample(syncFreqs, coupling);
                if (n >= settle) {
                    if ((o.risingMask & 1u) != 0u) {
                        ++c0;
                    }
                    if ((o.risingMask & 2u) != 0u) {
                        ++c1;
                    }
                }
            }
            return static_cast<double>(c0) / static_cast<double>(std::max(c1, 1));
        };
        const double freeRatio = measureRatio01(0.0f);
        const double gentleRatio = measureRatio01(0.35f);
        const double syncRatio = measureRatio01(1.0f);
        std::printf("  300/143 Hz ratio: ALG=0%% -> %.4f, 35%% -> %.4f, 100%% -> %.4f\n", freeRatio,
                    gentleRatio, syncRatio);
        EXPECT_TRUE(std::fabs(freeRatio - (300.0 / 143.0)) < 0.02,
                    "Uncoupled 300/143 ratio inaccurate (%.4f)", freeRatio);
        EXPECT_TRUE(gentleRatio > 2.04,
                    "Gentle ALGORITHM=35%% unexpectedly hard-synced 83-cent detune (%.4f)",
                    gentleRatio);
        EXPECT_TRUE(std::fabs(syncRatio - 2.0) < 0.02,
                    "ALGORITHM=100%% failed 2:1 subharmonic hard-sync lock (got %.4f)", syncRatio);
    }

    // (d) Audio-rate LFO FM (600 Hz LFO -> 440 Hz Voice Pitch, 4x sub-sample interpolated):
    {
        constexpr size_t kFftSize = 16384u;
        static std::array<double, kFftSize> fftRe{};
        static std::array<double, kFftSize> fftIm{};

        static ruido::DroneEngine engine;
        engine.init(ruido::kBaseSampleRate);
        engine.setAnalogCharacterEnabled(false, false);

        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.global.circuitCoupling = 0.0f;
        st.global.monoCollapse = true;
        st.voices[0].frequencyHz = 440.0f;
        st.voices[0].amplitude = ruido::kCleanAmpKnee;
        st.voices[0].lfoTarget = ruido::LfoTarget::Pitch;
        st.voices[0].lfoShape = 0.0f;
        st.voices[0].lfoRateHz = 600.0f;
        st.voices[0].lfoDepth = 0.35f;
        st.voices[1].muted = true;
        st.voices[2].muted = true;
        st.voices[3].muted = true;
        engine.controlBus().publish(st);
        engine.snapParametersToLatest();

        std::array<float, ruido::kBlockFrames> outL{};
        std::array<float, ruido::kBlockFrames> outR{};
        std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};
        for (int w = 0; w < 32; ++w) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
        }
        bool finiteOk = true;
        for (size_t offset = 0u; offset < kFftSize; offset += ruido::kBlockFrames) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                if (!isFiniteNonSubnormal(outL[i])) {
                    finiteOk = false;
                }
                const double ph = (2.0 * 3.14159265358979323846 * static_cast<double>(offset + i)) /
                                  static_cast<double>(kFftSize);
                const double win = 0.35875 - 0.48829 * std::cos(ph) + 0.14128 * std::cos(2.0 * ph) -
                                   0.01168 * std::cos(3.0 * ph);
                fftRe[offset + i] = static_cast<double>(outL[i]) * win;
                fftIm[offset + i] = 0.0;
            }
        }
        radix2Fft(fftRe.data(), fftIm.data(), kFftSize);
        const double binHz =
            static_cast<double>(ruido::kBaseSampleRate) / static_cast<double>(kFftSize);
        auto peakAt = [&](double hz) -> double {
            const size_t c = static_cast<size_t>(std::round(hz / binHz));
            double pk = 0.0;
            for (size_t b = (c > 4u ? c - 4u : 1u); b <= c + 4u; ++b) {
                pk = std::max(pk, std::hypot(fftRe[b], fftIm[b]));
            }
            return pk;
        };
        const double carrier = peakAt(440.0);
        const double upperSidebandDb =
            20.0 * std::log10(std::max(peakAt(1040.0) / carrier, 1.0e-15));
        std::printf("  600 Hz audio-rate LFO -> 440 Hz FM upper sideband (1040 Hz) = %.2f dBc\n",
                    upperSidebandDb);
        EXPECT_TRUE(finiteOk && upperSidebandDb > -18.0,
                    "Audio-rate 600 Hz LFO FM failed sideband generation (%.2f dBc)",
                    upperSidebandDb);
    }
}

// 14. Piecewise-slope PolyBLEP Pulse Width (5%..95%) & 0%..100% 1/f Analog Drift verification
void testPulseWidthAndVoiceDriftPhysics() {
    std::printf(
        "[Test 14] Piecewise-slope PolyBLEP PWM (5%%-95%%) & 0%%-100%% 1/f thermal drift...\n");

    // (a) Exact fundamental pitch invariance across 5%, 25%, 50%, and 95% duty cycles
    ruido::SchmittOscillatorBank bank;
    bank.reset(ruido::kOversampledRate);
    bank.setAnalogCharacterEnabled(false, false);

    const std::array<float, ruido::kNumOscillators> freqs = {220.0f, 220.0f, 220.0f, 220.0f};
    const std::array<float, ruido::kNumOscillators> duties = {0.05f, 0.25f, 0.50f, 0.95f};
    const std::array<float, ruido::kNumOscillators> zeroDrifts = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<uint32_t, ruido::kNumOscillators> risingCounts{0u, 0u, 0u, 0u};
    std::array<double, ruido::kNumOscillators> highCounts{0.0, 0.0, 0.0, 0.0};

    constexpr size_t kOneSecSamples = 176472u;
    for (size_t n = 0u; n < kOneSecSamples; ++n) {
        const auto out = bank.processSample(freqs, duties, zeroDrifts, 0.0f);
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            if (((out.risingMask >> v) & 1u) != 0u) {
                ++risingCounts[v];
            }
            highCounts[v] += static_cast<double>(out.gateHigh[v]);
        }
    }
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        const double measDuty = highCounts[v] / static_cast<double>(kOneSecSamples);
        EXPECT_TRUE(risingCounts[v] >= 219u && risingCounts[v] <= 221u,
                    "PWM voice %zu lost fundamental pitch lock at duty=%.2f (rises=%u)", v,
                    static_cast<double>(duties[v]), risingCounts[v]);
        EXPECT_TRUE(std::fabs(measDuty - static_cast<double>(duties[v])) < 0.015,
                    "PWM voice %zu gate duty mismatch (expected %.2f, got %.4f)", v,
                    static_cast<double>(duties[v]), measDuty);
    }

    // (b) Full-engine 32768-pt Blackman-Harris FFT on narrow 15% pulse at irrational 997.31 Hz:
    // verifies BOTH strong even-harmonic generation (H2 > -5 dBc) AND alias suppression (< -75 dBc)
    constexpr size_t kFftSize = 32768u;
    static std::array<double, kFftSize> fftRe{};
    static std::array<double, kFftSize> fftIm{};
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, true);

    const float f0 = 997.31f;
    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.global.circuitCoupling = 0.0f;
    st.global.monoCollapse = true;
    st.voices[0].frequencyHz = f0;
    st.voices[0].pulseWidth = 0.15f;
    st.voices[0].drift = 0.0f;
    st.voices[0].amplitude = ruido::kCleanAmpKnee;
    st.voices[0].pan = 0.0f;
    st.voices[0].hpCutoffHz = 2.0f;
    st.voices[0].hpResonance = 0.0f;
    st.voices[0].lpCutoffHz = 28000.0f;
    st.voices[0].lpResonance = 0.0f;
    st.voices[0].muted = false;
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};
    for (int w = 0; w < 64; ++w) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }
    for (size_t offset = 0u; offset < kFftSize; offset += ruido::kBlockFrames) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            const double ph = (2.0 * 3.14159265358979323846 * static_cast<double>(offset + i)) /
                              static_cast<double>(kFftSize);
            const double w = 0.35875 - 0.48829 * std::cos(ph) + 0.14128 * std::cos(2.0 * ph) -
                             0.01168 * std::cos(3.0 * ph);
            fftRe[offset + i] = static_cast<double>(outL[i]) * w;
            fftIm[offset + i] = 0.0;
        }
    }
    radix2Fft(fftRe.data(), fftIm.data(), kFftSize);

    const double binHz =
        static_cast<double>(ruido::kBaseSampleRate) / static_cast<double>(kFftSize);
    auto peakNear = [&](double hz) -> double {
        const size_t c = static_cast<size_t>(std::round(hz / binHz));
        double pk = 0.0;
        for (size_t b = (c > 4u ? c - 4u : 1u); b <= c + 4u; ++b) {
            pk = std::max(pk, std::hypot(fftRe[b], fftIm[b]));
        }
        return pk;
    };
    const double magFund = peakNear(static_cast<double>(f0));
    const double mag2nd = peakNear(2.0 * static_cast<double>(f0));
    const double h2Db = 20.0 * std::log10(std::max(mag2nd / magFund, 1.0e-15));

    double worstAliasDb = -200.0;
    const size_t minBin = static_cast<size_t>(std::ceil(60.0 / binHz));
    const size_t maxBin = static_cast<size_t>(std::floor(20000.0 / binHz));
    const double nyquist = 0.5 * static_cast<double>(ruido::kBaseSampleRate);
    for (size_t b = minBin; b <= maxBin; ++b) {
        const double freq = static_cast<double>(b) * binHz;
        bool isTrueHarmonic = false;
        for (int k = 1; k <= 64; ++k) {
            const double harmHz = static_cast<double>(k) * static_cast<double>(f0);
            if (harmHz >= nyquist) {
                break;
            }
            if (std::fabs(freq - harmHz) <= 14.0 * binHz) {
                isTrueHarmonic = true;
                break;
            }
        }
        if (isTrueHarmonic) {
            continue;
        }
        const double db =
            20.0 * std::log10(std::max(std::hypot(fftRe[b], fftIm[b]) / magFund, 1.0e-15));
        if (db > worstAliasDb) {
            worstAliasDb = db;
        }
    }
    std::printf("  15%% Pulse @ %.2f Hz: 2nd harmonic = %.2f dBc, worst alias <20 kHz = %.2f dBc\n",
                static_cast<double>(f0), h2Db, worstAliasDb);
    EXPECT_TRUE(h2Db > -5.0, "15%% pulse failed to generate strong 2nd harmonic (%.2f dBc)", h2Db);
    EXPECT_TRUE(worstAliasDb < -75.0, "15%% pulse alias floor exceeded -75 dBc (%.2f dBc)",
                worstAliasDb);

    // (c) 0% LOCKED vs 100% ANALOG 1/F thermal drift telemetry & crystal pitch lock
    bank.reset(ruido::kOversampledRate);
    bank.setAnalogCharacterEnabled(true, true);
    const std::array<float, ruido::kNumOscillators> mixedDrifts = {0.0f, 1.0f, 0.10f, 1.0f};
    float maxLockedNorm = 0.0f;
    float maxWideNorm = 0.0f;
    for (size_t n = 0u; n < kOneSecSamples; ++n) {
        const auto out = bank.processSample(freqs, duties, mixedDrifts, 0.0f);
        maxLockedNorm = std::max(maxLockedNorm, std::fabs(out.driftNorm[0]));
        maxWideNorm = std::max(maxWideNorm, std::fabs(out.driftNorm[1]));
    }
    std::printf("  Thermal drift |driftNorm|: 0%% LOCKED = %.6f, 100%% ANALOG 1/F = %.4f\n",
                static_cast<double>(maxLockedNorm), static_cast<double>(maxWideNorm));
    EXPECT_TRUE(maxLockedNorm == 0.0f, "0%% LOCKED drift leaked non-zero wander (%.6f)",
                static_cast<double>(maxLockedNorm));
    EXPECT_TRUE(maxWideNorm > 0.05f, "100%% ANALOG 1/F drift failed to wander (%.5f)",
                static_cast<double>(maxWideNorm));
}

// 15. Live Mixer, Circuit Lab Stutter, PWM Swarm & Clickless Snapshot Morph DSP
void testLiveMixerAndSnapshotMorphDsp() {
    std::printf(
        "[Test 15] Live Mixer / Circuit Lab punch-ins, PWM swarm & clickless snapshot morph...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    ruido::EngineControlState baseSt = ruido::EngineControlState::makeDefault();
    baseSt.voices[0].frequencyHz = 220.0f;
    baseSt.voices[0].lfoRateHz = 6.0f;
    baseSt.voices[0].lfoDepth = 0.50f;
    engine.controlBus().publish(baseSt);
    engine.snapParametersToLatest();
    for (int b = 0; b < 32; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    // (a) Sample-accurate 2.2ms-slewed Gate Stutter + stutterGateMask telemetry
    ruido::EngineControlState stutSt = baseSt;
    stutSt.keyMod.stutterDepth[0] = 1.0f;
    stutSt.keyMod.stutterRateHz[0] = 14.0f;
    engine.controlBus().publish(stutSt);

    bool sawOpen = false;
    bool sawClosed = false;
    float maxDelta = 0.0f;
    float prevSample = outL[ruido::kBlockFrames - 1u];
    for (int b = 0; b < 60; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        const uint8_t mask = engine.telemetry().stutterGateMask.load(std::memory_order_relaxed);
        if ((mask & 1u) != 0u) {
            sawOpen = true;
        } else {
            sawClosed = true;
        }
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            EXPECT_TRUE(std::isfinite(outL[i]) && std::isfinite(outR[i]),
                        "Non-finite output during gate stutter");
            maxDelta = std::max(maxDelta, std::fabs(outL[i] - prevSample));
            prevSample = outL[i];
        }
    }
    EXPECT_TRUE(sawOpen && sawClosed, "Stutter chopper failed to alternate gate open/closed");
    EXPECT_TRUE(maxDelta < 0.35f, "Stutter edge slew produced harsh discontinuity (%.4f)",
                static_cast<double>(maxDelta));

    // (b) Master Dub LPF Drop (28 kHz -> 55 Hz) + Overload Surge + Sub Drop + Spatial Focus
    ruido::EngineControlState dubSt = baseSt;
    dubSt.keyMod.overloadSurge = 1.0f;
    dubSt.keyMod.dubLpfDrop = 0.92f;
    dubSt.keyMod.spatialFocus = -1.0f; // Momentary mono collapse
    for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
        dubSt.keyMod.voicePitchMul[v] = 0.5f; // -1 Oct Sub Drop
    }
    engine.controlBus().publish(dubSt);
    for (int b = 0; b < 80; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }
    float monoDiff = 0.0f;
    double dubRms = 0.0;
    for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
        monoDiff = std::max(monoDiff, std::fabs(outL[i] - outR[i]));
        dubRms += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
    }
    dubRms = std::sqrt(dubRms / static_cast<double>(ruido::kBlockFrames));
    EXPECT_TRUE(monoDiff < 1.0e-4f, "spatialFocus=-1 failed to collapse stereo field (diff=%.6f)",
                static_cast<double>(monoDiff));
    EXPECT_TRUE(dubRms > 0.02 && dubRms < 1.5, "Dub LPF + Sub Drop RMS out of range (%.4f)",
                dubRms);

    // (c) Stasis Freeze: halts LFO phase advance completely while audio drones cleanly
    ruido::EngineControlState freezeSt = baseSt;
    freezeSt.keyMod.stasisFreeze = true;
    engine.controlBus().publish(freezeSt);
    engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
    const float phaseFrozenA = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    for (int b = 0; b < 20; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }
    const float phaseFrozenB = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    EXPECT_TRUE(std::fabs(phaseFrozenB - phaseFrozenA) < 1.0e-7f,
                "Stasis Freeze failed to hold LFO phase stationary (%.6f -> %.6f)",
                static_cast<double>(phaseFrozenA), static_cast<double>(phaseFrozenB));

    // Release -> LFO resumes advancing immediately
    engine.controlBus().publish(baseSt);
    for (int b = 0; b < 20; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }
    const float phaseResumed = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    EXPECT_TRUE(std::fabs(phaseResumed - phaseFrozenB) > 0.05f,
                "LFO failed to resume after releasing Stasis Freeze");

    // (d) POLY Swarm voicePwDelta shifts harmonic timbre even in MONO output + clickless morph
    ruido::EngineControlState monoBase = baseSt;
    monoBase.global.monoCollapse = true;
    engine.controlBus().publish(monoBase);
    engine.snapParametersToLatest();
    for (int b = 0; b < 24; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }
    ruido::EngineControlState swarmSt = monoBase;
    swarmSt.keyMod.voicePwDelta = {-0.18f, 0.16f, -0.26f, 0.24f};
    swarmSt.keyMod.voicePitchMul = {0.969f, 1.034f, 0.936f, 1.075f};
    engine.controlBus().publish(swarmSt);
    float maxMorphStep = 0.0f;
    float prevM = outL[ruido::kBlockFrames - 1u];
    for (int b = 0; b < 40; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            EXPECT_TRUE(std::isfinite(outL[i]) && std::abs(outL[i] - outR[i]) < 1.0e-5f,
                        "Non-finite or asymmetric MONO output during POLY swarm");
            maxMorphStep = std::max(maxMorphStep, std::fabs(outL[i] - prevM));
            prevM = outL[i];
        }
    }
    EXPECT_TRUE(maxMorphStep < 0.45f, "POLY voicePwDelta produced harsh step (%.4f)",
                static_cast<double>(maxMorphStep));

    std::printf("  Stutter maxDelta=%.4f, Dub RMS=%.3f, Swarm maxStep=%.4f\n",
                static_cast<double>(maxDelta), dubRms, static_cast<double>(maxMorphStep));
}

// 16. Circuit Lab DSP: Sub-Sample Hard Sync, 4x Patch Matrix (X-Mod + Ring/XOR), Turbo LFO & Scream
void testCircuitLabPatchMatrixHardSyncTurboAndScreamDsp() {
    std::printf("[Test 16] Circuit Lab DSP: Hard Sync, 4x X-Mod/Ring Matrix, Turbo LFO & Korg35 "
                "Scream...\n");

    // (a) SchmittOscillatorBank band-limited Hard Sync (OSC 0 master -> OSC 1..3 slaves)
    ruido::SchmittOscillatorBank syncBank;
    syncBank.reset(ruido::kOversampledRate);
    syncBank.setAnalogCharacterEnabled(false, false);
    const std::array<float, ruido::kNumOscillators> syncFreqs = {110.0f, 267.3f, 389.1f, 523.7f};
    const std::array<float, ruido::kNumOscillators> syncDuties = {0.5f, 0.5f, 0.5f, 0.5f};
    const std::array<float, ruido::kNumOscillators> zeroDrifts = {0.0f, 0.0f, 0.0f, 0.0f};

    constexpr size_t kSyncSamples = 88236u; // 0.5 s @ 176.472 kHz
    double dcSlave1 = 0.0;
    float maxSyncSample = 0.0f;
    size_t masterRiseCount = 0u;
    int sinceMasterWrap = -100;
    bool lockConsistent = true;
    for (size_t n = 0u; n < kSyncSamples; ++n) {
        const auto out =
            syncBank.processSample(syncFreqs, syncDuties, zeroDrifts, 0.0f, false, 1.0f);
        for (size_t v = 0u; v < ruido::kNumOscillators; ++v) {
            EXPECT_TRUE(isFiniteNonSubnormal(out.samples[v]),
                        "Non-finite/subnormal sample in Hard Sync");
            maxSyncSample = std::max(maxSyncSample, std::fabs(out.samples[v]));
        }
        if (n > 8000u) {
            dcSlave1 += static_cast<double>(out.samples[1]);
        }
        if (sinceMasterWrap >= 0) {
            ++sinceMasterWrap;
            if (sinceMasterWrap == 4) {
                // 4 samples after master wrap (past 2-sample B-Spline BLEP delay), every synced
                // slave oscillator has been reset to HIGH (+1.0f) regardless of inharmonic ratio!
                for (size_t v = 1u; v < ruido::kNumOscillators; ++v) {
                    if (out.samples[v] < 0.85f) {
                        lockConsistent = false;
                    }
                }
                sinceMasterWrap = -100;
            }
        }
        if ((out.risingMask & 1u) != 0u && n > 16000u) {
            sinceMasterWrap = 0;
            ++masterRiseCount;
        }
    }
    dcSlave1 /= static_cast<double>(kSyncSamples - 8000u);
    EXPECT_TRUE(masterRiseCount >= 40u, "Master OSC 0 failed to emit periodic sync edges");
    EXPECT_TRUE(lockConsistent, "Slave oscillators failed to phase-lock to OSC 0 Hard Sync wrap");
    EXPECT_TRUE(maxSyncSample <= 1.6f, "PolyBLEP Hard Sync overshot safe rail bound (%.4f)",
                static_cast<double>(maxSyncSample));
    EXPECT_TRUE(std::fabs(dcSlave1) < 0.35, "Excessive DC offset under Hard Sync (%.4f)", dcSlave1);

    // (b) Full 4x DroneEngine with X-MOD, RING/XOR, HARD SYNC, TURBO LFO (on dry patch!) & SCREAM
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> drain{};

    ruido::EngineControlState drySt = ruido::EngineControlState::makeDefault();
    drySt.voices[0].lfoRateHz = 1.5f;
    drySt.voices[0].lfoDepth = 0.0f; // Dry default patch!
    engine.controlBus().publish(drySt);
    engine.snapParametersToLatest();
    for (int b = 0; b < 24; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    // Engage TURBO LFO = 1.0 on dry patch: LFO phase must overclock 32x (~48 Hz) and LFO output
    // must wake up via momentary depth floor without modifying stored patch depth!
    ruido::EngineControlState turboSt = drySt;
    turboSt.keyMod.lfoTurbo = 1.0f;
    engine.controlBus().publish(turboSt);
    int turboWraps = 0;
    float maxTurboLfoOut = 0.0f;
    float prevPh = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
    for (int b = 0; b < 160; ++b) { // ~0.23 s -> ~10 wraps @ 48 Hz (vs 0 wraps @ 1.5 Hz!)
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        const float ph = engine.telemetry().lfoPhase[0].load(std::memory_order_relaxed);
        const float lo = engine.telemetry().lfoOutput[0].load(std::memory_order_relaxed);
        maxTurboLfoOut = std::max(maxTurboLfoOut, std::fabs(lo));
        if (prevPh > 0.70f && ph < 0.30f) {
            ++turboWraps;
        }
        prevPh = ph;
    }
    EXPECT_TRUE(turboWraps >= 7, "TURBO LFO failed to overclock 1.5 Hz -> 48 Hz (wraps=%d)",
                turboWraps);
    EXPECT_TRUE(maxTurboLfoOut > 0.25f,
                "TURBO LFO failed to wake dry-patch modulation depth (max=%.4f)",
                static_cast<double>(maxTurboLfoOut));

    // Verify CROSS-PIN 1 (xmodDepth[0]=1) and CRUSH 1 (crushDepth[0]=1) each bend OSC 1 even when
    // OSC 2..4 are SOLO-muted! Compare across 16 blocks (1024 frames > 1 full 55 Hz cycle).
    ruido::EngineControlState soloPlain = drySt;
    soloPlain.keyMod.voiceGainMul = {1.0f, 0.0f, 0.0f, 0.0f};
    soloPlain.global.monoCollapse = true;
    engine.controlBus().publish(soloPlain);
    engine.snapParametersToLatest();
    std::array<float, 16u * ruido::kBlockFrames> plainSoloTrace{};
    float plainSoloPeak = 0.0f;
    for (size_t b = 0u; b < 24u; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= 8u) {
            const size_t base = (b - 8u) * ruido::kBlockFrames;
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                plainSoloTrace[base + i] = outL[i];
                plainSoloPeak = std::max(plainSoloPeak, std::fabs(outL[i]));
            }
        }
    }

    engine.init(ruido::kBaseSampleRate);
    ruido::EngineControlState soloCross = soloPlain;
    soloCross.keyMod.xmodDepth[0] = 1.0f;
    engine.controlBus().publish(soloCross);
    engine.snapParametersToLatest();
    float maxSoloCrossDiff = 0.0f;
    for (size_t b = 0u; b < 24u; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= 8u) {
            const size_t base = (b - 8u) * ruido::kBlockFrames;
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                maxSoloCrossDiff =
                    std::max(maxSoloCrossDiff, std::fabs(outL[i] - plainSoloTrace[base + i]));
            }
        }
    }
    EXPECT_TRUE(plainSoloPeak > 0.05f && maxSoloCrossDiff > 0.80f * plainSoloPeak,
                "CROSS-PIN 1 failed to modulate soloed OSC 1 from pre-mute OSC 2 (diff=%.4f, "
                "peak=%.4f)",
                static_cast<double>(maxSoloCrossDiff), static_cast<double>(plainSoloPeak));

    engine.init(ruido::kBaseSampleRate);
    ruido::EngineControlState soloCrush = soloPlain;
    soloCrush.keyMod.crushDepth[0] = 1.0f;
    engine.controlBus().publish(soloCrush);
    engine.snapParametersToLatest();
    float maxSoloCrushDiff = 0.0f;
    for (size_t b = 0u; b < 24u; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= 8u) {
            const size_t base = (b - 8u) * ruido::kBlockFrames;
            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                maxSoloCrushDiff =
                    std::max(maxSoloCrushDiff, std::fabs(outL[i] - plainSoloTrace[base + i]));
            }
        }
    }
    EXPECT_TRUE(maxSoloCrushDiff > 0.70f * plainSoloPeak,
                "CRUSH 1 failed to decimate/sub-divide soloed OSC 1 (diff=%.4f, peak=%.4f)",
                static_cast<double>(maxSoloCrushDiff), static_cast<double>(plainSoloPeak));

    // Verify FOLD 1 (foldDepth[0]=1) multiplies zero-crossings / crests vs plain square at 110 Hz
    engine.init(ruido::kBaseSampleRate);
    soloPlain.voices[0].frequencyHz = 110.0f;
    engine.controlBus().publish(soloPlain);
    engine.snapParametersToLatest();
    int plainZc = 0;
    float prevSmp = 0.0f;
    for (int b = 0; b < 32; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= 8) {
            for (float s : outL) {
                if ((prevSmp < 0.0f && s >= 0.0f) || (prevSmp >= 0.0f && s < 0.0f)) {
                    ++plainZc;
                }
                prevSmp = s;
            }
        }
    }
    ruido::EngineControlState soloFold = soloPlain;
    soloFold.keyMod.foldDepth[0] = 1.0f;
    engine.controlBus().publish(soloFold);
    engine.snapParametersToLatest();
    int foldZc = 0;
    prevSmp = 0.0f;
    for (int b = 0; b < 32; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        if (b >= 8) {
            for (float s : outL) {
                if ((prevSmp < 0.0f && s >= 0.0f) || (prevSmp >= 0.0f && s < 0.0f)) {
                    ++foldZc;
                }
                prevSmp = s;
            }
        }
    }
    EXPECT_TRUE(foldZc > plainZc * 3,
                "FOLD 1 failed to multiply West-Coast wavefold crests (plain=%d, fold=%d)", plainZc,
                foldZc);

    // Engage simultaneous CROSS-PIN + FOLD + CRUSH + RING/XOR + HARD SYNC + Korg35 SCREAM stress
    ruido::EngineControlState labSt = drySt;
    labSt.keyMod.xmodDepth = {1.0f, 0.85f, 0.70f, 1.0f};
    labSt.keyMod.foldDepth = {1.0f, 0.65f, 0.90f, 0.50f};
    labSt.keyMod.crushDepth = {1.0f, 0.80f, 0.60f, 1.0f};
    labSt.voicePressure = {0.72f, 0.65f, 0.50f, 0.78f};
    labSt.keyMod.ringModMix = 1.0f;
    labSt.keyMod.hardSyncAmount = 1.0f;
    labSt.keyMod.filterScream = 1.0f;
    engine.controlBus().publish(labSt);

    float peakLab = 0.0f;
    double labEnergy = 0.0;
    for (int b = 0; b < 80; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            EXPECT_TRUE(isFiniteNonSubnormal(outL[i]) && isFiniteNonSubnormal(outR[i]),
                        "Non-finite or subnormal sample during full Circuit Lab stress");
            peakLab = std::max(peakLab, std::max(std::fabs(outL[i]), std::fabs(outR[i])));
            if (b >= 40) {
                labEnergy += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
            }
        }
    }
    const double labRms = std::sqrt(labEnergy / static_cast<double>(40u * ruido::kBlockFrames));
    EXPECT_TRUE(peakLab <= 1.25f, "Circuit Lab output exceeded stereo limiter ceiling (%.4f)",
                static_cast<double>(peakLab));
    EXPECT_TRUE(labRms > 0.04, "Circuit Lab output unexpectedly silent (RMS=%.5f)", labRms);

    // Release back to drySt -> smooth, finite restoration without clicks
    engine.controlBus().publish(drySt);
    float maxReleaseStep = 0.0f;
    float prevRel = outL[ruido::kBlockFrames - 1u];
    for (int b = 0; b < 40; ++b) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
        for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
            EXPECT_TRUE(isFiniteNonSubnormal(outL[i]) && isFiniteNonSubnormal(outR[i]),
                        "Non-finite sample releasing Circuit Lab bends");
            maxReleaseStep = std::max(maxReleaseStep, std::fabs(outL[i] - prevRel));
            prevRel = outL[i];
        }
    }
    EXPECT_TRUE(maxReleaseStep < 0.55f, "Harsh click on Circuit Lab bend release (%.4f)",
                static_cast<double>(maxReleaseStep));
    std::printf("  HardSync max=%.3f, Turbo wraps=%d, Lab peak=%.3f (RMS=%.3f), releaseStep=%.4f\n",
                static_cast<double>(maxSyncSample), turboWraps, static_cast<double>(peakLab),
                labRms, static_cast<double>(maxReleaseStep));
}

} // namespace

int main() {
    std::printf("=== Ruido Milestone 1 DSP Verification Suite ===\n\n");

    testMemoryFootprintAndLockFreeBus();
    testUltraSlowLfoPrecision();
    testSpectralAliasSuppression();
    testMs20FilterPassbandAndSmartPark();
    testExtremeStabilityAndZeroDenormals();
    testZeroDcOffsetRunaway();
    testClicklessTransitionsAndMonoPoly();
    testPowerRailInjectionLocking();
    testDecimatorLinearPhaseCrest();
    testThroughputBenchmark();
    testPerVoiceAnalogLfoModels();
    testRev2OtaSallenKeyPhysics();
    testUnbrokenExtremesSubBassOverdriveSyncAndAudioLfo();
    testPulseWidthAndVoiceDriftPhysics();
    testLiveMixerAndSnapshotMorphDsp();
    testCircuitLabPatchMatrixHardSyncTurboAndScreamDsp();

    if (gFailures == 0) {
        std::printf("\n[PASS] All Milestone 1 DSP verification gates passed!\n");
        return 0;
    }
    std::printf("\n[FAIL] %d verification check(s) failed.\n", gFailures);
    return 1;
}
