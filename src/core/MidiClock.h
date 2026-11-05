#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/GlobalConfig.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

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
        tapIntervalsSec_.fill(0.0f);
        sinceLastTickSec_ = kMaxTickGapSec;
        sinceLastTapSec_ = kMaxTickGapSec;
        lastTickTimeSec_ = -1.0f;
        residual100Us_ = 0.0f;
        silenceSec_ = kMidiClockTimeoutSec + 1.0f;
        internalBpm_ = kDefaultClockBpm;
        smoothedBpm_ = kDefaultClockBpm;
        tickWrite_ = 0u;
        tickStored_ = 0u;
        tapWrite_ = 0u;
        tapCount_ = 0u;
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
        sinceLastTapSec_ = clampf(sinceLastTapSec_ + dt, 0.0f, kMaxTickGapSec);
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

    void setInternalBpm(float bpm) noexcept {
        internalBpm_ = clampf(bpm, kMinInternalClockBpm, kMaxInternalClockBpm);
    }

    float internalBpm() const noexcept { return internalBpm_; }

    // Rising-edge Tap Tempo on the 11th tall key in 3/3 RHYTHM LAB:
    //  - Aligns Beat 1 (`++resetSeq_`) on the first tap of a new sequence (or anytime externally
    //    MIDI-synced so players can phase-align bars to incoming raw 0xF8 streams).
    //  - Averages up to 3 consecutive quarter-note intervals (4 taps, resetting immediately on
    //    >1.75x tempo jumps) with a 40 ms silicone contact-bounce guard.
    //  - Returns `true` when `internalBpm_` was updated.
    bool onTapTempo(ClockSyncMode mode) noexcept {
        if (isExternalSynced(mode)) {
            ++resetSeq_;
            sinceLastTapSec_ = 0.0f;
            tapCount_ = 0u;
            tapWrite_ = 0u;
            return false;
        }
        if (sinceLastTapSec_ < kMinTapIntervalSec) {
            return false; // Reject <40 ms mechanical switch bounce
        }
        if (sinceLastTapSec_ > kMaxTapIntervalSec) {
            tapCount_ = 0u;
            tapWrite_ = 0u;
            sinceLastTapSec_ = 0.0f;
            ++resetSeq_;
            return false;
        }

        const float dt = sinceLastTapSec_;
        sinceLastTapSec_ = 0.0f;
        if (tapCount_ > 0u) {
            float prevSum = 0.0f;
            for (uint8_t i = 0u; i < tapCount_; ++i) {
                prevSum += tapIntervalsSec_[i];
            }
            const float prevAvg = prevSum / static_cast<float>(tapCount_);
            if (dt > prevAvg * 1.75f || dt < prevAvg * 0.57f) {
                tapCount_ = 0u;
                tapWrite_ = 0u;
            }
        }
        tapIntervalsSec_[tapWrite_] = dt;
        tapWrite_ = static_cast<uint8_t>((tapWrite_ + 1u) % kMaxTapIntervals);
        if (tapCount_ < kMaxTapIntervals) {
            ++tapCount_;
        }
        float sumSec = 0.0f;
        for (uint8_t i = 0u; i < tapCount_; ++i) {
            sumSec += tapIntervalsSec_[i];
        }
        const float avgSec = sumSec / static_cast<float>(tapCount_);
        internalBpm_ = clampf(60.0f / avgSec, kMinInternalClockBpm, kMaxInternalClockBpm);
        return true;
    }

    bool isExternalSynced(ClockSyncMode mode) const noexcept {
        return (mode == ClockSyncMode::Auto) && externalActive_ && running_;
    }

    float bpm(ClockSyncMode mode) const noexcept {
        return isExternalSynced(mode) ? smoothedBpm_ : internalBpm_;
    }

    uint8_t resetSeq() const noexcept { return resetSeq_; }

  private:
    static constexpr size_t kMaxIntervals = 24u; // 24 intervals = 1 full quarter-note beat
    static constexpr size_t kMaxTapIntervals = 3u;
    static constexpr float kMaxTickGapSec = 10.0f;
    static constexpr float kMinTapIntervalSec = 0.040f; // 40 ms bounce guard (up to 1,500 BPM tap)
    static constexpr float kMaxTapIntervalSec = 3.200f; // 3.2 s fresh-sequence timeout (~18.8 BPM)

    std::array<uint16_t, kMaxIntervals> tickDeltas100Us_{};
    std::array<float, kMaxTapIntervals> tapIntervalsSec_{};
    float sinceLastTickSec_ = kMaxTickGapSec;
    float sinceLastTapSec_ = kMaxTickGapSec;
    float lastTickTimeSec_ = -1.0f;
    float residual100Us_ = 0.0f;
    float silenceSec_ = kMidiClockTimeoutSec + 1.0f;
    float internalBpm_ = kDefaultClockBpm;
    float smoothedBpm_ = kDefaultClockBpm;
    uint8_t tickWrite_ = 0u;
    uint8_t tickStored_ = 0u;
    uint8_t tapWrite_ = 0u;
    uint8_t tapCount_ = 0u;
    uint8_t resetSeq_ = 0u;
    bool running_ = true;
    bool hasPrevTick_ = false;
    bool hasEstimate_ = false;
    bool externalActive_ = false;
    bool lastWasExternalTs_ = false;
};

} // namespace ruido
