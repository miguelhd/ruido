#pragma once

#include "core/DspMath.h"
#include "core/GlobalConfig.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr float kDefaultClockBpm = 120.0f;
inline constexpr float kMinMidiClockBpm = 30.0f;
inline constexpr float kMaxMidiClockBpm = 300.0f;
inline constexpr float kMidiClockTimeoutSec = 1.50f;
inline constexpr float kMidiTimestampWrapSec = 64.0f;

// Transport-agnostic 24-PPQN MIDI System Real-Time clock follower (0xF8 / 0xFA / 0xFB / 0xFC).
// Designed to ingest TRS UART (31.25 kbaud), USB-MIDI (1 ms frames), and Bluetooth BLE-MIDI
// (7.5-15 ms connection bursts, with or without 13-bit packet timestamps) via a 1-beat (24-tick)
// sliding window + 1-pole PLL filter so wireless packet clumping never jitters rhythmic gates.
class MidiClockTracker {
  public:
    void reset() noexcept {
        tickDeltas100Us_.fill(0u);
        sinceLastTickSec_ = kMaxTickGapSec;
        lastTickTimeSec_ = -1.0f;
        residual100Us_ = 0.0f;
        silenceSec_ = kMidiClockTimeoutSec + 1.0f;
        smoothedBpm_ = kDefaultClockBpm;
        tickWrite_ = 0u;
        tickStored_ = 0u;
        resetSeq_ = 0u;
        running_ = true;
        hasPrevTick_ = false;
        hasEstimate_ = false;
        externalActive_ = false;
        lastWasExternalTs_ = false;
    }

    // Advance bounded relative tick timer & check 1.5 s MIDI clock unplug/stop timeout (Core 0).
    // Keeping `sinceLastTickSec_ <= 10.0f` guarantees sub-microsecond IEEE-754 float32 precision
    // even after days of continuous drone uptime.
    bool step(float dtSec, ClockSyncMode mode) noexcept {
        const float dt = clampf(dtSec, 0.0f, 0.25f);
        sinceLastTickSec_ = clampf(sinceLastTickSec_ + dt, 0.0f, kMaxTickGapSec);
        const bool wasActive = isExternalSynced(mode);
        if (externalActive_) {
            silenceSec_ += dt;
            if (silenceSec_ > kMidiClockTimeoutSec) {
                externalActive_ = false;
                hasPrevTick_ = false;
                hasEstimate_ = false;
                tickStored_ = 0u;
            }
        }
        return wasActive != isExternalSynced(mode);
    }

    // Handle 1-byte MIDI System Real-Time message (0xF8 Clock, 0xFA Start, 0xFB Cont, 0xFC Stop).
    // Pass `msgTimestampSec >= 0.0f` when available (monotonic or wrapped modulo `64.0f` seconds
    // for infinite-uptime sub-4us float32 precision), or `-1.0f` to use bounded internal delta.
    bool onMidiRealtime(uint8_t statusByte, ClockSyncMode mode,
                        float msgTimestampSec = -1.0f) noexcept {
        if (statusByte == 0xFAu) { // MIDI Start: reset bar phase to Beat 1 Step 1
            tickWrite_ = 0u;
            tickStored_ = 0u;
            hasPrevTick_ = false;
            hasEstimate_ = false;
            residual100Us_ = 0.0f;
            running_ = true;
            silenceSec_ = 0.0f;
            if (mode == ClockSyncMode::Auto) {
                ++resetSeq_;
                return true;
            }
            return false;
        }
        if (statusByte == 0xFBu) { // MIDI Continue
            running_ = true;
            silenceSec_ = 0.0f;
            return false;
        }
        if (statusByte == 0xFCu) { // MIDI Stop: release external lock cleanly back to internal
            running_ = false;
            const bool wasSync = isExternalSynced(mode);
            externalActive_ = false;
            hasPrevTick_ = false;
            hasEstimate_ = false;
            tickStored_ = 0u;
            return wasSync;
        }
        if (statusByte != 0xF8u) {
            return false;
        }

        // 0xF8 Timing Clock (24 pulses per quarter note)
        const bool useExtTs = (msgTimestampSec >= 0.0f);
        silenceSec_ = 0.0f;
        running_ = true;

        const float prevBpm = bpm(mode);
        const bool prevSync = isExternalSynced(mode);

        float dtSec = 0.0f;
        bool gapReset = !hasPrevTick_ || (useExtTs != lastWasExternalTs_);
        if (useExtTs) {
            if (!gapReset) {
                dtSec = msgTimestampSec - lastTickTimeSec_;
                if (dtSec < 0.0f && lastTickTimeSec_ <= kMidiTimestampWrapSec &&
                    (lastTickTimeSec_ - msgTimestampSec) > 0.5f * kMidiTimestampWrapSec) {
                    dtSec += kMidiTimestampWrapSec;
                }
                if (dtSec < 0.0f || dtSec > 2.0f) {
                    gapReset = true;
                }
            }
            lastTickTimeSec_ = msgTimestampSec;
        } else {
            dtSec = sinceLastTickSec_;
            sinceLastTickSec_ = 0.0f;
            if (dtSec > 2.0f) {
                gapReset = true;
            }
        }
        lastWasExternalTs_ = useExtTs;

        if (gapReset) {
            residual100Us_ = 0.0f;
            tickWrite_ = 0u;
            tickStored_ = 0u;
            hasPrevTick_ = true;
            hasEstimate_ = false;
            return (prevSync != isExternalSynced(mode));
        }
        const float scaled100Us = dtSec * 10000.0f + residual100Us_;
        const auto q100Us = static_cast<uint16_t>(clampf(scaled100Us + 0.5f, 0.0f, 65535.0f));
        residual100Us_ = clampf(scaled100Us - static_cast<float>(q100Us), -1.0f, 1.0f);

        tickDeltas100Us_[tickWrite_] = q100Us;
        tickWrite_ = static_cast<uint8_t>((tickWrite_ + 1u) % kMaxIntervals);
        if (tickStored_ < kMaxIntervals) {
            ++tickStored_;
        }

        uint32_t sum100Us = 0u;
        for (uint8_t k = 0u; k < tickStored_; ++k) {
            const size_t idx =
                (static_cast<size_t>(tickWrite_) + kMaxIntervals - 1u - k) % kMaxIntervals;
            sum100Us += tickDeltas100Us_[idx];
        }
        const float dtSpan = static_cast<float>(sum100Us) * 1.0e-4f;
        // Reject sub-4ms/tick BLE burst clumps until the next connection interval arrives
        const float minValidSpan = 0.004f * static_cast<float>(tickStored_);
        const float maxValidSpan = 0.100f * static_cast<float>(tickStored_);
        if (dtSpan >= minValidSpan && dtSpan <= maxValidSpan) {
            const float instBpm = clampf((2.5f * static_cast<float>(tickStored_)) / dtSpan,
                                         kMinMidiClockBpm, kMaxMidiClockBpm);
            if (!hasEstimate_) {
                smoothedBpm_ = instBpm;
                hasEstimate_ = true;
            } else {
                smoothedBpm_ += 0.24f * (instBpm - smoothedBpm_);
            }
            externalActive_ = true;
        }

        return (prevSync != isExternalSynced(mode)) || (absf(bpm(mode) - prevBpm) > 0.02f);
    }

    bool isExternalSynced(ClockSyncMode mode) const noexcept {
        return (mode == ClockSyncMode::Auto) && externalActive_ && running_;
    }

    float bpm(ClockSyncMode mode) const noexcept {
        return isExternalSynced(mode) ? smoothedBpm_ : kDefaultClockBpm;
    }

    uint8_t resetSeq() const noexcept { return resetSeq_; }

  private:
    static constexpr size_t kMaxIntervals = 24u; // 24 intervals = 1 full quarter-note beat
    static constexpr float kMaxTickGapSec = 10.0f;

    std::array<uint16_t, kMaxIntervals> tickDeltas100Us_{};
    float sinceLastTickSec_ = kMaxTickGapSec;
    float lastTickTimeSec_ = -1.0f;
    float residual100Us_ = 0.0f;
    float silenceSec_ = kMidiClockTimeoutSec + 1.0f;
    float smoothedBpm_ = kDefaultClockBpm;
    uint8_t tickWrite_ = 0u;
    uint8_t tickStored_ = 0u;
    uint8_t resetSeq_ = 0u;
    bool running_ = true;
    bool hasPrevTick_ = false;
    bool hasEstimate_ = false;
    bool externalActive_ = false;
    bool lastWasExternalTs_ = false;
};

} // namespace ruido
