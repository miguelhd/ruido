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
// dynamically partitioning the 4 independent 4x oversampled MS-20 ZDF voice filter chains across
// two cores (`1+3` when a single voice is nonlinear, cost-balanced `2+2` otherwise).
//  - Zero `compare_exchange` / `lockset` / `cli` instructions: immune to hardware spinlock
//    deadlocks even if Core 1 suffers a bit-21 SRAM fault mid-instruction.
//  - Zero stall when Core 0 is busy (UI scanline or panel poll): if Core 0 has not claimed the
//    published span by the time Core 1 finishes its local voice(s), Core 1 reclaims the delegated
//    voices immediately via Dekker's store-load fence with zero wait cycles.
//  - Bit-identical IEEE-754 float32 output whether Core 0 steals the span or Core 1 reclaims it.
class alignas(64) VoiceFilterCoProcessor {
  public:
    using Frame4x = std::array<float, kOversampleFactor>;
    using VoiceSpanBuffer = std::array<std::array<Frame4x, kMaxFilterSpanFrames>, kNumOscillators>;
    static constexpr size_t kMaxDelegatedVoices = 3u;

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
        numDelegated_ = 2u;
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

    // Called exclusively by Core 1. Cost-balances the 4 voice filter chains (`1+3` when 1 voice
    // is nonlinear and 3 are linear; heaviest+lightest `2+2` pairing otherwise), publishes the
    // delegated subset for Core 0, runs Core 1's subset, and joins/reclaims with zero stall.
    bool
    processBalanced4x(VoiceSpanBuffer& spanSub, size_t numFrames,
                      std::array<Ms20DualFilter, kNumOscillators>& filters,
                      const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs) noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u ||
            numFrames > kMaxFilterSpanFrames) {
            return false;
        }

        std::array<uint8_t, kNumOscillators> cost{};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            cost[v] = static_cast<uint8_t>((coeffs[v].hpLinearBypass ? 1u : 4u) +
                                           (coeffs[v].lpLinearBypass ? 1u : 4u));
        }

        std::array<uint8_t, kNumOscillators> ord{0u, 1u, 2u, 3u};
        const auto cmpSwap = [&](size_t a, size_t b) noexcept {
            if (cost[ord[a]] < cost[ord[b]]) {
                const uint8_t tmp = ord[a];
                ord[a] = ord[b];
                ord[b] = tmp;
            }
        };
        cmpSwap(0u, 1u);
        cmpSwap(2u, 3u);
        cmpSwap(0u, 2u);
        cmpSwap(1u, 3u);
        cmpSwap(1u, 2u);

        std::array<uint8_t, 2> c1Voices{0u, 1u};
        std::array<uint8_t, kMaxDelegatedVoices> c0Voices{2u, 3u, 0u};
        uint8_t c1Count = 2u;
        uint8_t c0Count = 2u;

        if (cost[ord[0]] > 2u && cost[ord[1]] == 2u) {
            c1Voices[0] = ord[0];
            c1Count = 1u;
            c0Voices[0] = ord[1];
            c0Voices[1] = ord[2];
            c0Voices[2] = ord[3];
            c0Count = 3u;
        } else if (cost[ord[0]] != cost[ord[3]]) {
            c1Voices[0] = ord[0];
            c1Voices[1] = ord[3];
            c0Voices[0] = ord[1];
            c0Voices[1] = ord[2];
        }

        numFrames_ = numFrames;
        numDelegated_ = c0Count;
        for (uint8_t d = 0u; d < c0Count; ++d) {
            const uint8_t v = c0Voices[d];
            delegatedFilters_[d] = filters[v];
            delegatedCoeffs_[d] = coeffs[v];
            for (size_t k = 0u; k < numFrames; ++k) {
                delegatedSamples_[d][k] = spanSub[v][k];
            }
        }

        uint32_t seq = lastSubmittedSeq_ + 1u;
        if (seq == 0u) {
            seq = 1u;
        }
        lastSubmittedSeq_ = seq;
        reqSeq_.store(seq, std::memory_order_seq_cst);

        for (uint8_t i = 0u; i < c1Count; ++i) {
            const uint8_t v = c1Voices[i];
            filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
        }

        joinDelegated(spanSub, numFrames, c0Voices, c0Count, filters, coeffs);
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
        const size_t count =
            (numDelegated_ <= kMaxDelegatedVoices) ? numDelegated_ : kMaxDelegatedVoices;
        for (size_t d = 0u; d < count; ++d) {
            delegatedFilters_[d].processSpan4x(delegatedSamples_[d].data(), n, delegatedCoeffs_[d]);
        }
        assistedSpans_.store(assistedSpans_.load(std::memory_order_relaxed) + 1u,
                             std::memory_order_relaxed);
        doneSeq_.store(req, std::memory_order_release);
        return true;
    }

    uint32_t assistedSpans() const noexcept {
        return assistedSpans_.load(std::memory_order_relaxed);
    }

  private:
    static constexpr bool seqReached(uint32_t observed, uint32_t target) noexcept {
        return observed != 0u && static_cast<int32_t>(observed - target) >= 0;
    }

    void joinDelegated(VoiceSpanBuffer& spanSub, size_t numFrames,
                       const std::array<uint8_t, kMaxDelegatedVoices>& c0Voices, uint8_t c0Count,
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
                for (uint8_t d = 0u; d < c0Count; ++d) {
                    const uint8_t v = c0Voices[d];
                    filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
                }
                return;
            }
        }

        uint32_t spins = 0u;
        while (doneSeq_.load(std::memory_order_acquire) != seq) {
            if (++spins > 500000u && helperArmed_.load(std::memory_order_acquire) == 0u &&
                doneSeq_.load(std::memory_order_acquire) != seq) {
                for (uint8_t d = 0u; d < c0Count; ++d) {
                    const uint8_t v = c0Voices[d];
                    filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
                }
                return;
            }
        }
        for (uint8_t d = 0u; d < c0Count; ++d) {
            const uint8_t v = c0Voices[d];
            filters[v] = delegatedFilters_[d];
            for (size_t k = 0u; k < numFrames; ++k) {
                spanSub[v][k] = delegatedSamples_[d][k];
            }
        }
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
    uint8_t numDelegated_ = 2u;
    std::array<Ms20DualFilter, kMaxDelegatedVoices> delegatedFilters_{};
    std::array<Ms20DualFilter::Coeffs, kMaxDelegatedVoices> delegatedCoeffs_{};
    std::array<std::array<Frame4x, kMaxFilterSpanFrames>, kMaxDelegatedVoices> delegatedSamples_{};
};

} // namespace ruido
