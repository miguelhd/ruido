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
        st.voices[0].hpResonance = 0.0f;
        st.voices[0].lpCutoffHz = 20000.0f;
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
    std::printf("[Test 4] MS-20 Rev 2 OTA ZDF parked flatness, Smart-Park & self-oscillation...\n");
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
                const float y = filter.processSample(x, 20.0f, reso, 20000.0f, reso,
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

    // Verify Smart-Park: kicking parked filter (HP=20, LP=20k) with HP/LP Res=1.0 decays silent!
    filter.reset();
    (void)filter.processSample(1.0f, 20.0f, 1.0f, 20000.0f, 1.0f, ruido::kInvOversampledRate);
    double tailEnergy = 0.0;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 20.0f, 1.0f, 20000.0f, 1.0f, ruido::kInvOversampledRate);
        if (n > 70000u) {
            tailEnergy += static_cast<double>(y) * static_cast<double>(y);
        }
    }
    const double parkedRms = std::sqrt(tailEnergy / 18236.0);
    std::printf("  Smart-Park HP/LP Reso=1.0 tail RMS (HP=20, LP=20k) = %.3e\n", parkedRms);
    EXPECT_TRUE(parkedRms < 1.0e-5, "Parked filter whistled/oscillated at Reso=1.0 (RMS=%.3e)",
                parkedRms);

    // Verify independent LPF self-oscillation pitch tracking across 200 Hz, 1 kHz, and 8 kHz!
    for (float targetFc : {200.0f, 1000.0f, 8000.0f}) {
        filter.reset();
        (void)filter.processSample(0.01f, 20.0f, 0.0f, targetFc, 0.95f, ruido::kInvOversampledRate);

        constexpr size_t kSettle = 88236u;
        constexpr size_t kMeasure = 88236u; // 0.500 s @ 176.472 kHz
        float prevY = 0.0f;
        int risingZeroCrossings = 0;
        double oscSq = 0.0;

        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y = filter.processSample(0.0f, 20.0f, 0.0f, targetFc, 0.95f,
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
                    "LPF self-oscillation amplitude out of bounds at %.1f Hz (RMS=%.3f)",
                    static_cast<double>(targetFc), oscRms);
        EXPECT_TRUE(std::fabs(measuredHz - static_cast<double>(targetFc)) <
                        0.03 * static_cast<double>(targetFc),
                    "LPF self-oscillation pitch tracking error > 3%% at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), measuredHz);
    }

    // Verify independent HPF self-oscillation pitch tracking across 200 Hz, 1 kHz, and 8 kHz!
    for (float targetFc : {200.0f, 1000.0f, 8000.0f}) {
        filter.reset();
        (void)filter.processSample(0.01f, targetFc, 0.95f, 20000.0f, 0.0f,
                                   ruido::kInvOversampledRate);

        constexpr size_t kSettle = 88236u;
        constexpr size_t kMeasure = 88236u;
        float prevY = 0.0f;
        int risingZeroCrossings = 0;
        double oscSq = 0.0;

        for (size_t n = 0u; n < kSettle + kMeasure; ++n) {
            const float y = filter.processSample(0.0f, targetFc, 0.95f, 20000.0f, 0.0f,
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
        std::printf("  HPF Self-Oscillation @ fc=%6.1f Hz -> measured=%7.1f Hz (RMS=%.3f)\n",
                    static_cast<double>(targetFc), measuredHz, oscRms);
        EXPECT_TRUE(oscRms > 0.15 && oscRms < 1.5,
                    "HPF self-oscillation amplitude out of bounds at %.1f Hz (RMS=%.3f)",
                    static_cast<double>(targetFc), oscRms);
        EXPECT_TRUE(std::fabs(measuredHz - static_cast<double>(targetFc)) <
                        0.03 * static_cast<double>(targetFc),
                    "HPF self-oscillation pitch tracking error > 3%% at %.1f Hz (got %.1f Hz)",
                    static_cast<double>(targetFc), measuredHz);
    }

    // Verify early MS-20 self-oscillation onset (LP Reso=0.45 already sustains limit cycle)
    filter.reset();
    (void)filter.processSample(0.02f, 20.0f, 0.0f, 800.0f, 0.45f, ruido::kInvOversampledRate);
    double earlyOscSq = 0.0;
    for (size_t n = 0u; n < 88236u; ++n) {
        const float y =
            filter.processSample(0.0f, 20.0f, 0.0f, 800.0f, 0.45f, ruido::kInvOversampledRate);
        if (n >= 44118u) {
            earlyOscSq += static_cast<double>(y) * static_cast<double>(y);
        }
    }
    const double earlyOscRms = std::sqrt(earlyOscSq / 44118.0);
    EXPECT_TRUE(earlyOscRms > 0.25, "Expected sustained self-oscillation by Reso=0.45 (RMS=%.3f)",
                earlyOscRms);

    // Verify simultaneous dual HPF (220 Hz) + LPF (880 Hz) self-oscillation maintains greater
    // combined energy than single-stage alone while staying cleanly bounded!
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
    std::printf("  Dual HPF(220)+LPF(880) Self-Oscillation: RMS=%.3f, Peak=%.3f\n", dualRms,
                static_cast<double>(dualPeak));
    EXPECT_TRUE(dualRms > 0.72 && dualPeak < 2.5f,
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
            st.voices[i].frequencyHz = 20.0f + 3900.0f * (0.5f + 0.5f * std::sin(ph));
            st.voices[i].hpCutoffHz = 20.0f + 18000.0f * (0.5f + 0.5f * std::cos(ph * 1.7f));
            st.voices[i].hpResonance = 1.0f;
            st.voices[i].lpCutoffHz = 40.0f + 19500.0f * (0.5f + 0.5f * std::sin(ph * 2.3f));
            st.voices[i].lpResonance = 1.0f;
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
    const float filtNanOut = rawFilter.processSample(qNan, 500.0f, 1.0f, 1500.0f, 1.0f,
                                                     ruido::kInvOversampledRate);
    const float filtBadRateOut =
        rawFilter.processSample(4.5f, qNan, -pInf, pInf, 0.0f, qNan);
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
    st.voices[0].amplitude = 0.9f;
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

    // Settle MONO mode for 30 blocks and verify L == R bit-for-bit + active LPG swell RMS
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

    // Verify Keys 5-8 pressure NEVER changes pitch in default Filter/LPG mode (0.000 Hz shift),
    // and smoothly bends pitch (+6 semitones -> period * 2^-0.5) when SEL selects Pitch Bend mode
    auto measureOsc1PeriodSamples = [&](float pressureVal, bool pitchMode) -> double {
        st.global.pitchPressureMode = pitchMode;
        st.voices[0].muted = false;
        st.voices[0].lpCutoffHz = 20000.0f;
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
                if (b >= 40 && drain[i].osc1RisingEdge != 0u) {
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
    EXPECT_TRUE(std::fabs(pitchRatio - 1.41421356) < 0.015,
                "SEL Pitch-Bend mode failed expected +6 st bend ratio (got %.4f)", pitchRatio);

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
                    "LFO shape %.2f positive peak out of spec (%.5f)",
                    static_cast<double>(shape), static_cast<double>(maxV));
        EXPECT_TRUE(minV < -0.995f && minV >= -1.00001f,
                    "LFO shape %.2f negative peak out of spec (%.5f)",
                    static_cast<double>(shape), static_cast<double>(minV));
        EXPECT_TRUE(std::fabs(dcMean) < 1.0e-5,
                    "LFO shape %.2f non-zero DC integral (%+.7f)",
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
        const double measuredHz =
            static_cast<double>(crossCount - 1u) * static_cast<double>(ruido::kBaseSampleRate) /
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
        st.voices[0].amplitude = 0.85f;
        st.voices[0].pan = 0.0f;
        st.voices[0].lfoRateHz = 11.0f;
        st.voices[0].lfoShape = 0.0f;
        st.voices[0].lfoDepth = 1.0f;
        st.voices[0].lfoTarget = ruido::LfoTarget::LpCutoff;
        for (size_t i = 1u; i < ruido::kNumOscillators; ++i) {
            st.voices[i].muted = true;
        }
        engine.controlBus().publish(st);
        engine.snapParametersToLatest();

        for (int w = 0; w < 30; ++w) {
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());
        }

        float prevL = outL[ruido::kBlockFrames - 1u];
        float prevR = outR[ruido::kBlockFrames - 1u];
        float maxSwitchDelta = 0.0f;
        bool finiteOk = true;

        // Sweep SHAPE continuously AND cycle through all 7 LFO targets live every 20 blocks
        for (int b = 0; b < 160; ++b) {
            st.voices[0].lfoShape = static_cast<float>(b) / 159.0f;
            if ((b % 20) == 0) {
                const uint8_t nextT = static_cast<uint8_t>((b / 20) % 7);
                st.voices[0].lfoTarget = static_cast<ruido::LfoTarget>(nextT);
            }
            engine.controlBus().publish(st);
            engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
            engine.scopeBuffer().popBatch(drain.data(), drain.size());

            for (size_t i = 0u; i < ruido::kBlockFrames; ++i) {
                if (!isFiniteNonSubnormal(outL[i]) || !isFiniteNonSubnormal(outR[i])) {
                    finiteOk = false;
                }
                maxSwitchDelta = std::max(maxSwitchDelta, std::fabs(outL[i] - prevL));
                maxSwitchDelta = std::max(maxSwitchDelta, std::fabs(outR[i] - prevR));
                prevL = outL[i];
                prevR = outR[i];
            }
        }
        std::printf("  Live 7-target + full SHAPE morph max |dSample| = %.5f\n",
                    static_cast<double>(maxSwitchDelta));
        EXPECT_TRUE(finiteOk, "Non-finite or subnormal sample during live LFO morph/target switch");
        EXPECT_TRUE(maxSwitchDelta < 0.14f,
                    "Discontinuity click during live LFO target/shape switch (delta=%.5f)",
                    static_cast<double>(maxSwitchDelta));
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
                    "HpCutoff LFO failed to sweep parked 20 Hz HPF upward (min=%.3f, max=%.3f)",
                    mn, mx);
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
                if (b >= 20 && drain[i].osc1RisingEdge != 0u) {
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
        EXPECT_TRUE(octRatio > 3.2,
                    "Pitch LFO failed wide log-octave period sweep (max/min=%.3f)", octRatio);
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
        const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(
            20.0f, 0.0f, 450.0f, 0.0f, ruido::kInvOversampledRate);

        constexpr double kTestHz = 300.0;
        size_t osStep = 0u;
        std::array<ruido::StereoFrame, 4> in4x{};

        for (size_t w = 0u; w < 4096u; ++w) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(
                    std::sin(2.0 * 3.14159265358979323846 * kTestHz *
                             static_cast<double>(osStep++) *
                             static_cast<double>(ruido::kInvOversampledRate)));
                const float y = filter.processPrepared(x, coeffs);
                in4x[s] = {y, y};
            }
            (void)decim.process4x(in4x);
        }

        for (size_t n = 0u; n < kFftSize; ++n) {
            for (size_t s = 0u; s < 4u; ++s) {
                const float x = static_cast<float>(
                    std::sin(2.0 * 3.14159265358979323846 * kTestHz *
                             static_cast<double>(osStep++) *
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
                    "Expected LM13600 Wilson-mirror 2nd harmonic at RES=0%% (got %.2f dBc)",
                    h2Db);
        EXPECT_TRUE(worstAliasDb < -85.0,
                    "C^2 LM13600 OTA exceeded -85 dBc alias floor (got %.2f dBc)", worstAliasDb);
    }

    // (b) Capacitive C1/C2 Sallen-Key Zero-DC Rejection under +0.85 DC bias + self-oscillation:
    {
        auto measureSteadyMean = [](float dcBias) -> double {
            ruido::Ms20DualFilter filter;
            filter.reset();
            const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(
                90.0f, 0.65f, 1200.0f, 0.85f, ruido::kInvOversampledRate);
            const size_t totalSamples = static_cast<size_t>(ruido::kOversampledRate * 2.0f);
            const size_t halfSamples = totalSamples / 2u;
            const size_t winLen = totalSamples - halfSamples;
            double sumOut = 0.0;
            double sumWin = 0.0;
            for (size_t n = 0u; n < totalSamples; ++n) {
                const float x = dcBias + 0.65f * std::sin(ruido::kTwoPi * 240.0f *
                                                          static_cast<float>(n) *
                                                          ruido::kInvOversampledRate);
                const float y = filter.processPrepared(x, coeffs);
                if (n >= halfSamples) {
                    const double w =
                        1.0 - std::cos((2.0 * 3.14159265358979323846 *
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
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(
                        20.0f, 0.0f, 20000.0f, 0.0f, ruido::kInvOversampledRate);
                } else if (mode == 1) {
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(
                        75.0f + 55.0f * static_cast<float>(v), 0.72f,
                        600.0f + 450.0f * static_cast<float>(v), 0.88f, ruido::kInvOversampledRate);
                } else {
                    const float hp = (v & 1u) != 0u ? 180.0f : 20.0f;
                    const float lp = (v & 2u) != 0u ? 950.0f : 20000.0f;
                    coeffs[v] = ruido::Ms20DualFilter::prepareCoeffs(
                        hp, 0.85f, lp, 0.85f, ruido::kInvOversampledRate);
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
                            sig = (std::sin(t * (1.0f + 0.3f * static_cast<float>(v))) >= 0.0f) ?
                                      0.95f :
                                      -0.95f;
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

    // (d) Hot standalone input (±10.0f) Smart-Park handoff continuity across 18.8..20.0 kHz:
    {
        ruido::Ms20DualFilter filter;
        filter.reset();
        float prevY = 0.0f;
        float maxHandoffStep = 0.0f;
        for (size_t n = 0u; n < 4000u; ++n) {
            const float lpHz = 19850.0f + 0.05f * static_cast<float>(n); // crosses ~19991 Hz
            const auto coeffs = ruido::Ms20DualFilter::prepareCoeffs(
                20.0f, 0.0f, lpHz, 0.0f, ruido::kInvOversampledRate);
            const float x = 10.0f * std::sin(ruido::kTwoPi * 3000.0f * static_cast<float>(n) *
                                             ruido::kInvOversampledRate);
            const float y = filter.processPrepared(x, coeffs);
            if (n > 64u) {
                maxHandoffStep = std::max(maxHandoffStep, std::fabs(y - prevY));
            }
            prevY = y;
        }
        EXPECT_TRUE(maxHandoffStep < 1.25f,
                    "Hot ±10V Smart-Park handoff clicked (max |dy| = %.4f)",
                    static_cast<double>(maxHandoffStep));
    }
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

    if (gFailures == 0) {
        std::printf("\n[PASS] All Milestone 1 DSP verification gates passed!\n");
        return 0;
    }
    std::printf("\n[FAIL] %d verification check(s) failed.\n", gFailures);
    return 1;
}
