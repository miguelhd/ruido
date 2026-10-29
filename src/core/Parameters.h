#pragma once

#include "core/Config.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

// User-facing per-oscillator state (serializable into <= 128B snapshots in Milestone 3).
struct VoiceParams {
    float frequencyHz = 110.0f;  // Audio: [20, 4000] Hz; LFO: [0.02, 20] Hz
    float amplitude = 0.70f;     // [0.0, 1.0] post-filter mix level
    float pan = 0.0f;            // [-1.0 (Left), +1.0 (Right)]
    float hpCutoffHz = 20.0f;    // [20.0, 20000.0] Hz (20 Hz = parked flat)
    float lpCutoffHz = 20000.0f; // [20.0, 20000.0] Hz (20 kHz = parked flat)
    float resonance = 0.0f;      // [0.0, 1.0] Smart-Park shared MS-20 Peak
    float drive = 0.0f;          // [0.0, 1.0] pre-filter diode push
    bool muted = false;          // White Keys 1-4 latching mute
    bool subAudioLfo = false;    // Right-panel LFO range flag (< 20 Hz)
};

// Global circuit & master routing state (Left encoders + MONO/POLY + PLAY/STOP).
struct GlobalParams {
    float circuitCoupling = 0.25f; // ALGORITHM knob: [0.0, 1.0] CD40106 Vdd rail sag
    float stereoSpread = 1.0f;     // SELECT knob: [0.0 (Mono), 1.0 (Full Pan Width)]
    float masterTrim = 0.85f;      // Software headroom trim before DAC / pedals
    int8_t globalOctaveShift = 0;  // OCT- / OCT+ global octave offset [-3, +3]
    bool monoCollapse = false;      // MONO vs POLY black keys (true = Dual-Mono L==R)
    bool masterRun = true;          // PLAY/STOP button (clickless master kill/run)
    bool pitchPressureMode = false; // PIT black key (true = Lyra pressure bends pitch)
};

// Complete patch state + momentary SARADC finger-pressure modulation from Core 0 -> Core 1.
struct EngineControlState {
    std::array<VoiceParams, kNumOscillators> voices{};
    GlobalParams global{};
    // White Keys 5-8 per-voice continuous SARADC carbon-pill pressure in [0.0, 1.0]
    std::array<float, kNumOscillators> voicePressure{0.0f, 0.0f, 0.0f, 0.0f};

    static EngineControlState makeDefault() noexcept {
        EngineControlState s{};
        // Musical open-fifth / octave starting chord with gentle stereo spread
        constexpr std::array<float, kNumOscillators> kInitFreqs = {
            55.0f, 82.41f, 110.15f, 164.81f};
        constexpr std::array<float, kNumOscillators> kInitPans = {
            -0.35f, 0.35f, -0.65f, 0.65f};
        for (size_t i = 0; i < kNumOscillators; ++i) {
            s.voices[i].frequencyHz = kInitFreqs[i];
            s.voices[i].amplitude = 0.65f;
            s.voices[i].pan = kInitPans[i];
            s.voices[i].hpCutoffHz = kFilterCutoffMinHz;
            s.voices[i].lpCutoffHz = kFilterCutoffMaxHz;
            s.voices[i].resonance = 0.0f;
            s.voices[i].drive = 0.0f;
            s.voices[i].muted = false;
            s.voices[i].subAudioLfo = false;
        }
        return s;
    }
};

// Verify lock-free atomic invariants mandated by src/dsp/AGENTS.md Rule #3
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "uint32_t atomics must be lock-free on target architecture");
static_assert(std::atomic<size_t>::is_always_lock_free,
              "size_t atomics must be lock-free on target architecture");
static_assert(std::atomic<float>::is_always_lock_free,
              "float atomics must be lock-free on target architecture");

// Lock-free triple-buffered control bus (Core 0 writer -> Core 1 audio reader).
// Never blocks either core and guarantees tear-free multi-parameter reads.
class LockFreeControlBus {
public:
    LockFreeControlBus() noexcept {
        const EngineControlState init = EngineControlState::makeDefault();
        buffers_[0] = init;
        buffers_[1] = init;
        buffers_[2] = init;
    }

    // Called from Core 0 (UI / GPIO / SARADC thread). Single-producer only.
    void publish(const EngineControlState& state) noexcept {
        buffers_[writeIdx_] = state;
        // Atomically swap writeIdx_ with the clean middle slot holding latest data.
        // Uses acq_rel so reclaiming readyIdx synchronizes-with Core 1's prior release.
        uint32_t prevReady = sharedState_.load(std::memory_order_relaxed);
        uint32_t nextState = 0u;
        do {
            const uint32_t readyIdx = prevReady & 0x3u;
            nextState = (writeIdx_ & 0x3u) | kNewDataBit;
            if (sharedState_.compare_exchange_weak(
                    prevReady, nextState, std::memory_order_acq_rel, std::memory_order_relaxed)) {
                writeIdx_ = readyIdx;
                break;
            }
        } while (true);
    }

    // Called from Core 1 at block start (Audio thread). Single-consumer only.
    const EngineControlState& consumeLatest() noexcept {
        uint32_t current = sharedState_.load(std::memory_order_acquire);
        if ((current & kNewDataBit) != 0u) {
            do {
                const uint32_t newReadIdx = current & 0x3u;
                // Return old readIdx_ to pool and clear kNewDataBit
                const uint32_t replacement = readIdx_ & 0x3u;
                if (sharedState_.compare_exchange_weak(current, replacement,
                                                       std::memory_order_acq_rel,
                                                       std::memory_order_acquire)) {
                    readIdx_ = newReadIdx;
                    break;
                }
            } while ((current & kNewDataBit) != 0u);
        }
        return buffers_[readIdx_];
    }

private:
    static constexpr uint32_t kNewDataBit = 0x4u;

    std::array<EngineControlState, 3> buffers_{};
    alignas(64) uint32_t writeIdx_ = 0u;
    alignas(64) uint32_t readIdx_ = 1u;
    alignas(64) std::atomic<uint32_t> sharedState_{2u}; // Slot 2 initially parked in middle
};

// Decimated stereo scope sample + Osc-1 rising-edge trigger flag (Core 1 -> Core 0).
struct ScopeFrame {
    float left = 0.0f;
    float right = 0.0f;
    uint8_t osc1RisingEdge = 0u;
};

// Lock-free SPSC ring buffer feeding the 240x240 CRT scope on Core 0.
class ScopeRingBuffer {
public:
    bool push(const ScopeFrame& frame) noexcept {
        const size_t w = writePos_.load(std::memory_order_relaxed);
        const size_t r = readPos_.load(std::memory_order_acquire);
        if ((w - r) >= kScopeRingCapacity) {
            return false; // Full: drop gracefully without blocking audio
        }
        buffer_[w & kScopeRingMask] = frame;
        writePos_.store(w + 1u, std::memory_order_release);
        return true;
    }

    size_t popBatch(ScopeFrame* dst, size_t maxCount) noexcept {
        const size_t r = readPos_.load(std::memory_order_relaxed);
        const size_t w = writePos_.load(std::memory_order_acquire);
        const size_t avail = w - r;
        const size_t count = (avail < maxCount) ? avail : maxCount;
        if (count > 0u) {
            for (size_t i = 0u; i < count; ++i) {
                dst[i] = buffer_[(r + i) & kScopeRingMask];
            }
            readPos_.store(r + count, std::memory_order_release);
        }
        return count;
    }

    size_t available() const noexcept {
        const size_t r = readPos_.load(std::memory_order_acquire);
        const size_t w = writePos_.load(std::memory_order_acquire);
        const size_t diff = w - r;
        return (diff <= kScopeRingCapacity) ? diff : 0u;
    }

private:
    std::array<ScopeFrame, kScopeRingCapacity> buffer_{};
    alignas(64) std::atomic<size_t> writePos_{0u};
    alignas(64) std::atomic<size_t> readPos_{0u};
};

// Real-time acoustic telemetry for Core 0 varibright LED pulsing & headroom indicator.
struct EngineTelemetry {
    std::atomic<float> peakLeft{0.0f};
    std::atomic<float> peakRight{0.0f};
    // Instantaneous [0.0, 1.0] gate state per Schmitt trigger (for < 20 Hz key LED pulse)
    std::array<std::atomic<float>, kNumOscillators> oscGateHigh{
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
        std::atomic<float>{0.0f},
    };
};

} // namespace ruido
