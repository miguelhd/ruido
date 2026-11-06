#pragma once

#include "core/Config.h"
#include "dsp/Ms20Filter.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

inline constexpr size_t kMaxFilterSpanFrames = 16u;

// Cache-line-isolated, 100% CAS-free / spinlock-free Dekker work-stealing SPSC co-processor
// splitting the 4 independent 4x oversampled MS-20 ZDF voice filter chains across two cores
// (`Voices 0-1` on Core 1 in parallel with `Voices 2-3` stolen opportunistically by Core 0).
//  - Zero `compare_exchange` / `lockset` / `cli` instructions: immune to hardware spinlock
//    deadlocks even if Core 1 suffers a bit-21 SRAM fault mid-instruction.
//  - Zero stall when Core 0 is busy (UI scanline or panel poll): if Core 0 has not claimed the
//    published span by the time Core 1 finishes `Voices 0-1`, Core 1 reclaims `Voices 2-3`
//    immediately via Dekker's store-load fence with zero wait cycles.
//  - Bit-identical IEEE-754 float32 output whether Core 0 steals the span or Core 1 reclaims it.
class alignas(64) VoiceFilterCoProcessor {
  public:
    using Frame4x = std::array<float, kOversampleFactor>;

    VoiceFilterCoProcessor() noexcept {}

    void reset() noexcept {
        helperArmed_.store(0u, std::memory_order_relaxed);
        c0ClaimingSeq_.store(0u, std::memory_order_relaxed);
        claimedSeq_.store(0u, std::memory_order_relaxed);
        doneSeq_.store(0u, std::memory_order_relaxed);
        assistedSpans_.store(0u, std::memory_order_relaxed);
        c1ReclaimedSeq_.store(0u, std::memory_order_relaxed);
        reqSeq_.store(0u, std::memory_order_relaxed);
        lastSubmittedSeq_ = 0u;
        lastHandledSeq_ = 0u;
        numFrames_ = 0u;
    }

    // Called exclusively by Core 0 (helper thread).
    void setHelperArmed(bool armed) noexcept {
        if (armed) {
            helperArmed_.store(1u, std::memory_order_seq_cst);
            return;
        }
        helperArmed_.store(0u, std::memory_order_seq_cst);
        (void)tryAssist();
    }

    // Non-blocking emergency disarm invoked ONLY when Core 1 silicon has faulted or stalled.
    void abortFromFault() noexcept { helperArmed_.store(0u, std::memory_order_release); }

    bool hasPendingWork() const noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        const uint32_t req = reqSeq_.load(std::memory_order_relaxed);
        if (req == 0u) {
            return false;
        }
        return !seqReached(doneSeq_.load(std::memory_order_relaxed), req) &&
               !seqReached(c1ReclaimedSeq_.load(std::memory_order_relaxed), req);
    }

    // Called exclusively by Core 1. Publishes `Voices 2-3` for opportunistic work-stealing by
    // Core 0 while Core 1 processes `Voices 0-1`.
    bool
    submitVoices23(const Frame4x* span2, const Frame4x* span3, size_t numFrames,
                   const std::array<Ms20DualFilter, kNumOscillators>& filters,
                   const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs) noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u ||
            numFrames > kMaxFilterSpanFrames) {
            return false;
        }
        numFrames_ = numFrames;
        filters23_[0] = filters[2];
        filters23_[1] = filters[3];
        coeffs23_[0] = coeffs[2];
        coeffs23_[1] = coeffs[3];
        for (size_t k = 0u; k < numFrames; ++k) {
            samples23_[0][k] = span2[k];
            samples23_[1][k] = span3[k];
        }
        uint32_t seq = lastSubmittedSeq_ + 1u;
        if (seq == 0u) {
            seq = 1u;
        }
        lastSubmittedSeq_ = seq;
        reqSeq_.store(seq, std::memory_order_seq_cst);
        return true;
    }

    // Called exclusively by Core 0 (`app_idle` / inter-strip / inter-frame loop).
    bool tryAssist() noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        const uint32_t req = reqSeq_.load(std::memory_order_seq_cst);
        if (req == 0u || req == lastHandledSeq_) {
            return false;
        }
        if (seqReached(c1ReclaimedSeq_.load(std::memory_order_acquire), req)) {
            lastHandledSeq_ = req;
            return false;
        }

        // Dekker Side 0: announce intent to claim `req`, then verify Core 1 has not reclaimed or
        // superseded `req` (safe across arbitrary Core 0 ISR preemptions).
        c0ClaimingSeq_.store(req, std::memory_order_seq_cst);
        if (seqReached(c1ReclaimedSeq_.load(std::memory_order_seq_cst), req) ||
            reqSeq_.load(std::memory_order_seq_cst) != req) {
            c0ClaimingSeq_.store(0u, std::memory_order_seq_cst);
            lastHandledSeq_ = req;
            return false;
        }
        claimedSeq_.store(req, std::memory_order_seq_cst);
        c0ClaimingSeq_.store(0u, std::memory_order_seq_cst);
        lastHandledSeq_ = req;

        const size_t n = (numFrames_ <= kMaxFilterSpanFrames) ? numFrames_ : kMaxFilterSpanFrames;
        filters23_[0].processSpan4x(samples23_[0].data(), n, coeffs23_[0]);
        filters23_[1].processSpan4x(samples23_[1].data(), n, coeffs23_[1]);
        assistedSpans_.store(assistedSpans_.load(std::memory_order_relaxed) + 1u,
                             std::memory_order_relaxed);
        doneSeq_.store(req, std::memory_order_release);
        return true;
    }

    // Called exclusively by Core 1 after finishing `Voices 0-1` when `submitVoices23` returned
    // `true`. Reclaims `Voices 2-3` immediately if Core 0 has not claimed `seq` yet.
    void joinVoices23(Frame4x* span2, Frame4x* span3, size_t numFrames,
                      std::array<Ms20DualFilter, kNumOscillators>& filters,
                      const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs) noexcept {
        const uint32_t seq = lastSubmittedSeq_;
        if (doneSeq_.load(std::memory_order_acquire) != seq &&
            claimedSeq_.load(std::memory_order_acquire) != seq) {
            // Dekker Side 1: reclaim `seq` immediately so Core 1 never stalls behind busy Core 0.
            c1ReclaimedSeq_.store(seq, std::memory_order_seq_cst);
            uint32_t claimSpins = 0u;
            while (c0ClaimingSeq_.load(std::memory_order_seq_cst) == seq && ++claimSpins < 65536u) {
            }
            if (claimedSeq_.load(std::memory_order_seq_cst) != seq) {
                filters[2].processSpan4x(span2, numFrames, coeffs[2]);
                filters[3].processSpan4x(span3, numFrames, coeffs[3]);
                return;
            }
        }

        uint32_t spins = 0u;
        while (doneSeq_.load(std::memory_order_acquire) != seq) {
            if (++spins > 500000u && helperArmed_.load(std::memory_order_acquire) == 0u &&
                doneSeq_.load(std::memory_order_acquire) != seq) {
                filters[2].processSpan4x(span2, numFrames, coeffs[2]);
                filters[3].processSpan4x(span3, numFrames, coeffs[3]);
                return;
            }
        }
        filters[2] = filters23_[0];
        filters[3] = filters23_[1];
        for (size_t k = 0u; k < numFrames; ++k) {
            span2[k] = samples23_[0][k];
            span3[k] = samples23_[1][k];
        }
    }

    uint32_t assistedSpans() const noexcept {
        return assistedSpans_.load(std::memory_order_relaxed);
    }

  private:
    static constexpr bool seqReached(uint32_t observed, uint32_t target) noexcept {
        return observed != 0u && static_cast<int32_t>(observed - target) >= 0;
    }

    alignas(64) std::atomic<uint32_t> helperArmed_{0u};
    std::atomic<uint32_t> c0ClaimingSeq_{0u};
    std::atomic<uint32_t> claimedSeq_{0u};
    std::atomic<uint32_t> doneSeq_{0u};
    std::atomic<uint32_t> assistedSpans_{0u};
    uint32_t lastHandledSeq_ = 0u;

    alignas(64) std::atomic<uint32_t> c1ReclaimedSeq_{0u};
    std::atomic<uint32_t> reqSeq_{0u};
    uint32_t lastSubmittedSeq_ = 0u;

    alignas(64) size_t numFrames_ = 0u;
    std::array<Ms20DualFilter, 2> filters23_{};
    std::array<Ms20DualFilter::Coeffs, 2> coeffs23_{};
    std::array<std::array<Frame4x, kMaxFilterSpanFrames>, 2> samples23_{};
};

} // namespace ruido
