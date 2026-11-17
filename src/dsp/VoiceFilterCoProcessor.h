#pragma once

#include "core/Config.h"
#include "dsp/SallenKeyFilter.h"

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
        spanSubPtr_ = nullptr;
        filtersPtr_ = nullptr;
        coeffsPtr_ = nullptr;
        numFrames_ = 0u;
        numDelegated_ = 2u;
        delegatedVoices_ = {2u, 3u, 0u};
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

        // Keep the 4-element sorting network packed in a single 32-bit register (2 bits per voice)
        // rather than a 4-byte stack array so an AC791N Core 1 bit-21 (`0x00200000`, byte-lane 2)
        // SRAM misread can never corrupt `ord[2]` to `34` and index `spanSub` out of bounds.
        uint32_t ord = 0u | (1u << 2u) | (2u << 4u) | (3u << 6u);
        const auto ordAt = [&](uint32_t slot) noexcept -> uint8_t {
            return static_cast<uint8_t>((ord >> (2u * slot)) & 3u);
        };
        const auto cmpSwap = [&](uint32_t a, uint32_t b) noexcept {
            const uint8_t va = ordAt(a);
            const uint8_t vb = ordAt(b);
            if (cost[va] < cost[vb]) {
                const uint32_t sa = 2u * a;
                const uint32_t sb = 2u * b;
                ord = (ord & ~((3u << sa) | (3u << sb))) | (static_cast<uint32_t>(vb) << sa) |
                      (static_cast<uint32_t>(va) << sb);
            }
        };
        cmpSwap(0u, 1u);
        cmpSwap(2u, 3u);
        cmpSwap(0u, 2u);
        cmpSwap(1u, 3u);
        cmpSwap(1u, 2u);

        const uint8_t o0 = ordAt(0u);
        const uint8_t o1 = ordAt(1u);
        const uint8_t o2 = ordAt(2u);
        const uint8_t o3 = ordAt(3u);

        std::array<uint8_t, 2> c1Voices{0u, 1u};
        std::array<uint8_t, kMaxDelegatedVoices> c0Voices{2u, 3u, 0u};
        uint8_t c1Count = 2u;
        uint8_t c0Count = 2u;

        if (cost[o0] > 2u && cost[o1] == 2u) {
            c1Voices[0] = o0;
            c1Count = 1u;
            c0Voices[0] = o1;
            c0Voices[1] = o2;
            c0Voices[2] = o3;
            c0Count = 3u;
        } else if (cost[o0] != cost[o3]) {
            c1Voices[0] = o0;
            c1Voices[1] = o3;
            c0Voices[0] = o1;
            c0Voices[1] = o2;
        }

        spanSubPtr_ = &spanSub;
        filtersPtr_ = filters.data();
        coeffsPtr_ = coeffs.data();
        numFrames_ = numFrames;
        numDelegated_ = c0Count;
        delegatedVoices_ = c0Voices;

        uint32_t seq = lastSubmittedSeq_ + 1u;
        if (seq == 0u) {
            seq = 1u;
        }
        lastSubmittedSeq_ = seq;
        reqSeq_.store(seq, std::memory_order_seq_cst);

        for (uint8_t i = 0u; i < c1Count; ++i) {
            const size_t v = static_cast<size_t>(c1Voices[i] & 3u);
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

        VoiceSpanBuffer* spanPtr = spanSubPtr_;
        Ms20DualFilter* fltPtr = filtersPtr_;
        const Ms20DualFilter::Coeffs* cfPtr = coeffsPtr_;
        const size_t n = (numFrames_ <= kMaxFilterSpanFrames) ? numFrames_ : kMaxFilterSpanFrames;
        const size_t count =
            (numDelegated_ <= kMaxDelegatedVoices) ? numDelegated_ : kMaxDelegatedVoices;
        if (spanPtr != nullptr && fltPtr != nullptr && cfPtr != nullptr) {
            for (size_t d = 0u; d < count; ++d) {
                const size_t v = static_cast<size_t>(delegatedVoices_[d] & 3u);
                fltPtr[v].processSpan4x((*spanPtr)[v].data(), n, cfPtr[v]);
            }
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
        const size_t safeFrames =
            (numFrames <= kMaxFilterSpanFrames) ? numFrames : kMaxFilterSpanFrames;
        const uint8_t safeCount =
            (c0Count <= kMaxDelegatedVoices) ? c0Count : static_cast<uint8_t>(kMaxDelegatedVoices);
        const uint32_t seq = lastSubmittedSeq_;
        if (doneSeq_.load(std::memory_order_acquire) != seq &&
            claimedSeq_.load(std::memory_order_acquire) != seq) {
            // Dekker Side 1: reclaim `seq` immediately so Core 1 never stalls behind busy Core 0.
            c1ReclaimedSeq_.store(seq, std::memory_order_seq_cst);
            uint32_t claimSpins = 0u;
            while (c0ClaimingSeq_.load(std::memory_order_seq_cst) == seq && ++claimSpins < 4096u) {
            }
            if (claimedSeq_.load(std::memory_order_seq_cst) != seq) {
                for (uint8_t d = 0u; d < safeCount; ++d) {
                    const size_t v = static_cast<size_t>(c0Voices[d] & 3u);
                    filters[v].processSpan4x(spanSub[v].data(), safeFrames, coeffs[v]);
                }
                return;
            }
        }

#if defined(__pi32v2__)
        uint32_t spins = 0u;
#endif
        while (doneSeq_.load(std::memory_order_acquire) != seq) {
#if defined(__pi32v2__)
            // Core 0 has already claimed `seq` (`claimedSeq_ == seq`): never mutate `filters[v]`
            // concurrently on Core 1; bound only against a wedged Core 0 (~25 ms ceiling).
            if (++spins > 2097152u) {
                helperArmed_.store(0u, std::memory_order_release);
                return;
            }
#endif
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

    alignas(64) VoiceSpanBuffer* spanSubPtr_ = nullptr;
    Ms20DualFilter* filtersPtr_ = nullptr;
    const Ms20DualFilter::Coeffs* coeffsPtr_ = nullptr;
    size_t numFrames_ = 0u;
    uint8_t numDelegated_ = 2u;
    std::array<uint8_t, kMaxDelegatedVoices> delegatedVoices_{2u, 3u, 0u};
};

} // namespace ruido
