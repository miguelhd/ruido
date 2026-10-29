#include "core/Config.h"
#include "core/DspMath.h"
#include "core/Parameters.h"
#include "dsp/DroneEngine.h"
#include "dsp/HalfBandDecimator.h"
#include "dsp/Ms20Filter.h"
#include "dsp/SchmittOscillatorBank.h"

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

    // Total engine (including 2048-frame scope ring buffer) must be < 36 KB out of 578 KB SRAM
    EXPECT_TRUE(sizeof(ruido::DroneEngine) < 36864u, "DroneEngine exceeded 36 KB SRAM budget");

    // Verify LockFreeControlBus never drops latest published state
    ruido::LockFreeControlBus bus;
    for (int i = 0; i < 5000; ++i) {
        ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
        st.voices[0].frequencyHz = 100.0f + static_cast<float>(i);
        st.voices[3].resonance = 0.0001f * static_cast<float>(i % 1000);
        bus.publish(st);
        if ((i % 7) == 0) {
            const auto& readSt = bus.consumeLatest();
            EXPECT_TRUE(std::fabs(readSt.voices[0].frequencyHz -
                                  (100.0f + static_cast<float>(i))) < 1e-4f,
                        "ControlBus lost latest state at iter %d", i);
        }
    }

    // Verify ScopeRingBuffer FIFO order & overflow resilience
    ruido::ScopeRingBuffer ring;
    for (size_t i = 0; i < ruido::kScopeRingCapacity + 64u; ++i) {
        const bool ok = ring.push({static_cast<float>(i), -static_cast<float>(i),
                                   static_cast<uint8_t>(i & 1u)});
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

// 2. Ultra-slow 0.02 Hz LFO precision (proving 32-bit phase accumulator never stalls)
void testUltraSlowLfoPrecision() {
    std::printf("[Test 2] Sub-audio LFO precision (0.02 Hz & 0.50 Hz)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);
    engine.setAnalogCharacterEnabled(false, true);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.global.circuitCoupling = 0.0f;
    st.voices[0].subAudioLfo = true;
    st.voices[0].frequencyHz = 0.50f; // 2.000 s period = 88,236 base frames
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<ruido::ScopeFrame, ruido::kBlockFrames> scopeOut{};

    int firstEdgeFrame = -1;
    int secondEdgeFrame = -1;
    bool sawGateHigh = false;
    bool sawGateLow = false;

    const int totalFrames05Hz = static_cast<int>(ruido::kBaseSampleRate * 4.5f);
    for (int baseFrame = 0; baseFrame < totalFrames05Hz;
         baseFrame += static_cast<int>(ruido::kBlockFrames)) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        const float gate = engine.telemetry().oscGateHigh[0].load(std::memory_order_relaxed);
        if (gate > 0.5f) {
            sawGateHigh = true;
        } else {
            sawGateLow = true;
        }
        const size_t n = engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size());
        for (size_t i = 0u; i < n; ++i) {
            if (scopeOut[i].osc1RisingEdge != 0u && (baseFrame + static_cast<int>(i)) > 100) {
                if (firstEdgeFrame < 0) {
                    firstEdgeFrame = baseFrame + static_cast<int>(i);
                } else if (secondEdgeFrame < 0) {
                    secondEdgeFrame = baseFrame + static_cast<int>(i);
                }
            }
        }
    }

    EXPECT_TRUE(sawGateHigh && sawGateLow, "0.50 Hz LFO failed to toggle LED gate state");
    EXPECT_TRUE(firstEdgeFrame > 0 && secondEdgeFrame > firstEdgeFrame,
                "0.50 Hz LFO did not emit two rising edges");
    if (firstEdgeFrame > 0 && secondEdgeFrame > firstEdgeFrame) {
        const double measuredPeriodSec =
            static_cast<double>(secondEdgeFrame - firstEdgeFrame) /
            static_cast<double>(ruido::kBaseSampleRate);
        std::printf("  0.50 Hz LFO measured period = %.5f s (expected 2.00000 s)\n",
                    measuredPeriodSec);
        EXPECT_TRUE(std::fabs(measuredPeriodSec - 2.0) < 0.005,
                    "0.50 Hz period inaccurate: %.5f s", measuredPeriodSec);
    }

    // Now test extreme minimum 0.02 Hz (50.0 s period) over 55 seconds to prove zero float32 stall!
    engine.reset();
    st.voices[0].frequencyHz = 0.02f;
    engine.controlBus().publish(st);
    engine.snapParametersToLatest();

    // Drain scope buffer
    while (engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size()) > 0u) {
    }

    int edgeCount002Hz = 0;
    int lastEdge002 = -1;
    const int frames55Sec = static_cast<int>(ruido::kBaseSampleRate * 55.0f);
    for (int baseFrame = 0; baseFrame < frames55Sec;
         baseFrame += static_cast<int>(ruido::kBlockFrames)) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        const size_t n = engine.scopeBuffer().popBatch(scopeOut.data(), scopeOut.size());
        for (size_t i = 0u; i < n; ++i) {
            if (scopeOut[i].osc1RisingEdge != 0u && (baseFrame + static_cast<int>(i)) > 1000) {
                ++edgeCount002Hz;
                lastEdge002 = baseFrame + static_cast<int>(i);
            }
        }
    }
    const double firstWrapSec =
        static_cast<double>(lastEdge002) / static_cast<double>(ruido::kBaseSampleRate);
    std::printf("  0.02 Hz LFO wrapped at t = %.3f s (expected ~50.0 s, edges=%d)\n",
                firstWrapSec, edgeCount002Hz);
    EXPECT_TRUE(edgeCount002Hz == 1 && std::fabs(firstWrapSec - 50.0) < 0.25,
                "0.02 Hz LFO phase accumulator stalled or drifted (edges=%d, t=%.3f s)",
                edgeCount002Hz, firstWrapSec);
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
        st.voices[0].amplitude = 0.80f;
        st.voices[0].pan = 0.0f;
        st.voices[0].hpCutoffHz = 20.0f;
        st.voices[0].lpCutoffHz = 20000.0f;
        st.voices[0].resonance = 0.0f;
        st.voices[0].drive = 0.0f;
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
            const double ph =
                (2.0 * 3.14159265358979323846 * static_cast<double>(n)) /
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

// 4. Parked MS-20 filter passband flatness, Smart-Park resonance & self-oscillation tracking
void testMs20FilterPassbandAndSmartPark() {
    std::printf("[Test 4] MS-20 Korg35 ZDF parked flatness, Smart-Park & self-oscillation...\n");
    ruido::Ms20DualFilter filter;

    constexpr std::array<float, 4> kProbeFreqs = {120.0f, 440.0f, 1000.0f, 4000.0f};
    for (float reso : {0.0f, 1.0f}) {
        for (float fProbe : kProbeFreqs) {
            filter.reset();
            const size_t totalSamples = 17647u * 2u; // 200 ms @ 176.472 kHz
            const size_t measureStart = 17647u;
            double sumInSq = 0.0;
            double sumOutSq = 0.0;

            for (size_t n = 0u; n < totalSamples; ++n) {
                const float x = 0.20f * std::sin(ruido::kTwoPi * fProbe * static_cast<float>(n) *
                                                 ruido::kInvOversampledRate);
                const float y = filter.processSample(x, 20.0f, 20000.0f, reso, 0.0f,
                                                     ruido::kInvOversampledRate);
                if (n >= measureStart) {
                    sumInSq += static_cast<double>(x) * static_cast<double>(x);
                    sumOutSq += static_cast<double>(y) * static_cast<double>(y);
                }
            }
            const double gainDb = 10.0 * std::log10(sumOutSq / sumInSq);
            EXPECT_TRUE(std::fabs(gainDb) < 0.20,
                        "Parked filter not flat at %.0f Hz (reso=%.1f): %.3f dB",
                        static_cast<double>(fProbe), static_cast<double>(reso), gainDb);
        }
    }

    // Verify Smart-Park: kicking parked filter (HP=20, LP=20k) with Reso=1.0 decays to silence!
    filter.reset();
    (void)filter.processSample(1.0f, 20.0f, 20000.0f, 1.0f, 0.0f, ruido::kInvOversampledRate);
    double tailEnergy = 0.0;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 20.0f, 20000.0f, 1.0f, 0.0f, ruido::kInvOversampledRate);
        if (n > 70000u) {
            tailEnergy += static_cast<double>(y) * static_cast<double>(y);
        }
    }
    const double parkedRms = std::sqrt(tailEnergy / 18236.0);
    std::printf("  Smart-Park Reso=1.0 tail RMS (HP=20, LP=20k) = %.3e\n", parkedRms);
    EXPECT_TRUE(parkedRms < 1.0e-5, "Parked filter whistled/oscillated at Reso=1.0 (RMS=%.3e)",
                parkedRms);

    // Verify un-parked LPF self-oscillation pitch tracking across 200 Hz, 1 kHz, and 8 kHz!
    for (float targetFc : {200.0f, 1000.0f, 8000.0f}) {
        filter.reset();
        // Tiny 1-sample impulse to seed self-oscillation
        (void)filter.processSample(0.01f, 20.0f, targetFc, 0.95f, 0.0f, ruido::kInvOversampledRate);

        constexpr size_t kSettle = 88236u;
        constexpr size_t kMeasure = 88236u; // 0.500 s @ 176.472 kHz
        float prevY = 0.0f;
        int risingZeroCrossings = 0;
        double oscSq = 0.0;

        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y = filter.processSample(0.0f, 20.0f, targetFc, 0.95f, 0.0f,
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
        std::printf("  LPF Self-Oscillation @ fc=%6.1f Hz -> measured=%7.1f Hz (RMS=%.3f)\n",
                    static_cast<double>(targetFc), measuredHz, oscRms);
        EXPECT_TRUE(oscRms > 0.15 && oscRms < 1.5,
                    "Self-oscillation amplitude out of bounds at %.1f Hz (RMS=%.3f)",
                    static_cast<double>(targetFc), oscRms);
        EXPECT_TRUE(std::fabs(measuredHz - static_cast<double>(targetFc)) <
                        0.03 * static_cast<double>(targetFc),
                    "ZDF self-oscillation pitch tracking error > 3%% at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), measuredHz);
    }
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
            st.voices[i].frequencyHz = 20.0f + 3900.0f * (0.5f + 0.5f * std::sin(ph));
            st.voices[i].hpCutoffHz = 20.0f + 18000.0f * (0.5f + 0.5f * std::cos(ph * 1.7f));
            st.voices[i].lpCutoffHz = 40.0f + 19500.0f * (0.5f + 0.5f * std::sin(ph * 2.3f));
            st.voices[i].resonance = 1.0f;
            st.voices[i].drive = 1.0f;
            st.voices[i].amplitude = 1.0f;
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
        silent.voices[i].resonance = 0.0f;
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
    nanSt.voices[0].lpCutoffHz = pInf;
    nanSt.voices[0].resonance = qNan;
    nanSt.voices[0].drive = -pInf;
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
    const float filtNanOut = rawFilter.processSample(qNan, 500.0f, 1500.0f, 1.0f, 1.0f,
                                                     ruido::kInvOversampledRate);
    EXPECT_TRUE(allValid && isFiniteNonSubnormal(filtNanOut),
                "NaN/Inf poisoned engine or standalone Ms20DualFilter");
}

// 6. Zero DC offset runaway over 10 seconds of asymmetric CMOS + driven filters
void testZeroDcOffsetRunaway() {
    std::printf("[Test 6] Zero DC offset runaway (10 s continuous drone)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.voices[0].subAudioLfo = true;
    st.voices[0].frequencyHz = 3.7f;
    st.voices[1].drive = 0.8f;
    st.voices[1].resonance = 0.6f;
    st.voices[1].lpCutoffHz = 900.0f;
    st.voices[2].hpCutoffHz = 250.0f;
    st.voices[2].resonance = 0.5f;
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

// 7. Clickless mute/gate, Lyra-8 pressure swell & MONO/POLY routing
void testClicklessTransitionsAndMonoPoly() {
    std::printf("[Test 7] Clickless vactrol mutes, Lyra pressure swell & MONO/POLY routing...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    // Use filtered low-frequency tone (80 Hz, LP=250 Hz) so any raw step click stands out sharply
    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    st.voices[0].frequencyHz = 80.0f;
    st.voices[0].lpCutoffHz = 250.0f;
    st.voices[0].amplitude = 0.9f;
    st.voices[0].pan = -1.0f;
    st.voices[1].muted = true;
    st.voices[2].muted = true;
    st.voices[3].muted = true;
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

    // Toggle mute abruptly on & off across blocks and ensure zero step discontinuity
    for (int step = 0; step < 60; ++step) {
        if (step == 5) {
            st.voices[0].muted = true;
            engine.controlBus().publish(st);
        } else if (step == 25) {
            // While muted, swell voice 0 in via Lyra pressure pad = 1.0f!
            st.voicePressure[0] = 1.0f;
            engine.controlBus().publish(st);
        } else if (step == 33) {
            // Toggle PIT and unmute while finger pressure = 1.0f is actively held (must not click!)
            st.global.pitchPressureMode = true;
            st.voices[0].muted = false;
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

    // Settle MONO mode for 30 blocks and verify L == R bit-for-bit
    for (int w = 0; w < 30; ++w) {
        engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
        engine.scopeBuffer().popBatch(drain.data(), drain.size());
    }

    bool monoBitIdentical = true;
    double pressureRms = 0.0;
    for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
        if (outL[i] != outR[i]) {
            monoBitIdentical = false;
        }
        pressureRms += static_cast<double>(outL[i]) * static_cast<double>(outL[i]);
    }
    pressureRms = std::sqrt(pressureRms / static_cast<double>(ruido::kBlockFrames));

    std::printf("  Max sample-to-sample step delta = %.5f, MONO L==R = %s, Pressure RMS = %.3f\n",
                static_cast<double>(maxStepDelta), monoBitIdentical ? "YES" : "NO", pressureRms);
    EXPECT_TRUE(maxStepDelta < 0.05f, "Audible click discontinuity detected (delta=%.5f)",
                static_cast<double>(maxStepDelta));
    EXPECT_TRUE(monoBitIdentical, "MONO output collapse did not yield L == R");
    EXPECT_TRUE(pressureRms > 0.05, "Lyra pressure pad failed to swell muted oscillator");
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
    st.voices[0].amplitude = 1.0f;
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
    EXPECT_TRUE(crestRatio < 1.26,
                "Excessive full-engine crest factor (crest=%.3f)", crestRatio);
}

// 10. Real-Time Throughput Benchmark
void testThroughputBenchmark() {
    std::printf("[Test 10] Real-time throughput benchmark (10,000 x 64-frame blocks)...\n");
    static ruido::DroneEngine engine;
    engine.init(ruido::kBaseSampleRate);

    ruido::EngineControlState st = ruido::EngineControlState::makeDefault();
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        st.voices[i].resonance = 0.75f;
        st.voices[i].drive = 0.45f;
        st.voices[i].hpCutoffHz = 65.0f;
        st.voices[i].lpCutoffHz = 1800.0f;
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
    constexpr double kMinRealtimeMultiple = 20.0;
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
    const double audioSec =
        static_cast<double>(kBenchBlocks * ruido::kBlockFrames) /
        static_cast<double>(ruido::kBaseSampleRate);
    const double nsPerFrame =
        (elapsedSec * 1.0e9) / static_cast<double>(kBenchBlocks * ruido::kBlockFrames);

    std::printf(
        "  Rendered %.2f s of 4x stereo audio in %.4f s (%.1fx RT, %.1f ns/frame, chk=%.2f)\n",
        audioSec, elapsedSec, audioSec / elapsedSec, nsPerFrame, static_cast<double>(checksum));
    EXPECT_TRUE((audioSec / elapsedSec) > kMinRealtimeMultiple, "Throughput unexpectedly slow");
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

    if (gFailures == 0) {
        std::printf("\n[PASS] All Milestone 1 DSP verification gates passed!\n");
        return 0;
    }
    std::printf("\n[FAIL] %d verification check(s) failed.\n", gFailures);
    return 1;
}
