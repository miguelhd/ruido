#pragma once

#include "core/CacheSync.h"
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
//  - Strict single-writer 64-byte cache-line isolation + explicit `flushinv` handoffs on AC791N so
//    Core 0 and Core 1 never false-share or read stale 32-byte L1 D-Cache lines.
//  - Bypasses cross-core handoff completely when all 4 voices are in linear filter bypass.
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
        lastHandledSeq_ = 0u;
        c1ReclaimedSeq_.store(0u, std::memory_order_relaxed);
        reqSeq_.store(0u, std::memory_order_relaxed);
        lastSubmittedSeq_ = 0u;
        c1HelperTimedOut_ = 0u;
        joinPending_ = false;
        pendingCount_ = 0u;
        spanSubPtr_ = nullptr;
        filtersPtr_ = nullptr;
        coeffsPtr_ = nullptr;
        numFrames_ = 0u;
        numDelegated_ = 2u;
        delegatedVoices_ = {2u, 3u, 0u};
        hwCacheFlushInvalidateRange(this, sizeof(*this));
    }

    // Called exclusively by Core 0 (helper thread).
    void setHelperArmed(bool armed) noexcept {
        if (armed) {
            helperArmed_.store(1u, std::memory_order_seq_cst);
            hwCacheFlushInvalidateLine(&helperArmed_);
            return;
        }
        helperArmed_.store(0u, std::memory_order_seq_cst);
        hwCacheFlushInvalidateLine(&helperArmed_);
        (void)tryAssist();
    }

    // Non-blocking emergency disarm invoked ONLY on Core 0 when Core 1 silicon has faulted/stalled.
    void abortFromFault() noexcept {
        helperArmed_.store(0u, std::memory_order_release);
        hwCacheFlushInvalidateLine(&helperArmed_);
    }

    bool hasPendingWork() const noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        hwCacheFlushInvalidateLine(&c1ReclaimedSeq_);
        const uint32_t req = reqSeq_.load(std::memory_order_relaxed);
        if (req == 0u) {
            return false;
        }
        return !seqReached(doneSeq_.load(std::memory_order_relaxed), req) &&
               !seqReached(c1ReclaimedSeq_.load(std::memory_order_relaxed), req);
    }

    // Called exclusively by Core 1. Skips silent voices (`activeVoiceMask`), runs linear-bypass
    // voices locally in Core 1 L1 cache, and delegates nonlinear active voices to Core 0 only when
    // >= 2 active voices are nonlinear (`1+1`, `2+1`, or `2+2` split).
    bool processBalanced4x(VoiceSpanBuffer& spanSub, size_t numFrames,
                           std::array<Ms20DualFilter, kNumOscillators>& filters,
                           const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs,
                           uint8_t activeVoiceMask = 0x0Fu) noexcept {
        if (!beginBalanced4x(spanSub, numFrames, filters, coeffs, activeVoiceMask)) {
            return false;
        }
        finishBalanced4x(spanSub, filters, coeffs);
        return true;
    }

    // Split-phase variant: submits Core 0's share and filters Core 1's own voices, leaving the
    // delegated join pending so Core 1 can run independent work (e.g. decimating the previous
    // span) while Core 0 claims. Callers MUST invoke `finishBalanced4x` before reading outputs.
    bool beginBalanced4x(VoiceSpanBuffer& spanSub, size_t numFrames,
                         std::array<Ms20DualFilter, kNumOscillators>& filters,
                         const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs,
                         uint8_t activeVoiceMask = 0x0Fu) noexcept {
        joinPending_ = false;
        if (numFrames == 0u || numFrames > kMaxFilterSpanFrames) {
            return false;
        }
        if (activeVoiceMask == 0u) {
            return true;
        }
        if (c1HelperTimedOut_ != 0u) {
            return false;
        }
        hwCacheFlushInvalidateLine(&helperArmed_);
        if (helperArmed_.load(std::memory_order_acquire) == 0u) {
            return false;
        }

        std::array<uint8_t, kNumOscillators> dualNlVoices{};
        std::array<uint8_t, kNumOscillators> singleNlVoices{};
        std::array<uint8_t, kNumOscillators> linVoices{};
        uint8_t dualCount = 0u;
        uint8_t singleCount = 0u;
        uint8_t linCount = 0u;
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            if ((activeVoiceMask & static_cast<uint8_t>(1u << v)) == 0u) {
                continue;
            }
            const bool hpNl = !coeffs[v].hpLinearBypass;
            const bool lpNl = !coeffs[v].lpLinearBypass;
            if (hpNl && lpNl) {
                dualNlVoices[dualCount++] = static_cast<uint8_t>(v);
            } else if (hpNl || lpNl) {
                singleNlVoices[singleCount++] = static_cast<uint8_t>(v);
            } else {
                linVoices[linCount++] = static_cast<uint8_t>(v);
            }
        }
        const uint8_t nlCount = static_cast<uint8_t>(dualCount + singleCount);
        std::array<uint8_t, kNumOscillators> nlVoices{};
        for (uint8_t i = 0u; i < dualCount; ++i) {
            nlVoices[i] = dualNlVoices[i];
        }
        for (uint8_t i = 0u; i < singleCount; ++i) {
            nlVoices[dualCount + i] = singleNlVoices[i];
        }

        if (nlCount < 2u) {
            // 0 or 1 nonlinear voice: keep 100% inside Core 1 L1 cache (zero cross-core flushinv)!
            for (uint8_t i = 0u; i < nlCount; ++i) {
                const size_t v = nlVoices[i];
                filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
            }
            for (uint8_t i = 0u; i < linCount; ++i) {
                const size_t v = linVoices[i];
                filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
            }
            return true;
        }

        // Greedy stage-cost load balancer across Core 0 (`kMaxDelegatedVoices = 3`, up to `6`
        // nonlinear stages) and Core 1 (`c1Load` seeded at `2` for Osc + LFO + Mix + Decimator +
        // Scope), dropping 4-voice HP+LP ZDF stress from 93.8% (`2+2`) down to 81.9% (`3+1`):
        //  - [1,1]     -> C0=2v (2 stg), C1=0v (+base)
        //  - [1,1,1,1] -> C0=3v (3 stg), C1=1v (1 stg + base)
        //  - [2,2,1,1] -> C0=2v (4 stg), C1=2v (2 stg + base)  [Scene 3<->4 morph sweet spot]
        //  - [2,2,2,2] -> C0=3v (6 stg), C1=1v (2 stg + base)  [Worst-case 4v HP+LP stress]
        static constexpr uint8_t kMaxC0StageLoad = 6u;
        std::array<uint8_t, kMaxDelegatedVoices> c0Voices{0u, 0u, 0u};
        std::array<uint8_t, kNumOscillators> c1NlVoices{};
        uint8_t c0Count = 0u;
        uint8_t c1NlCount = 0u;
        uint8_t c0Load = 0u;
        uint8_t c1Load = 2u;
        for (uint8_t i = 0u; i < nlCount; ++i) {
            const uint8_t v = nlVoices[i];
            const uint8_t stageCost = (i < dualCount) ? 2u : 1u;
            if (c0Load <= c1Load && c0Count < kMaxDelegatedVoices &&
                static_cast<uint8_t>(c0Load + stageCost) <= kMaxC0StageLoad) {
                c0Voices[c0Count++] = v;
                c0Load = static_cast<uint8_t>(c0Load + stageCost);
            } else {
                c1NlVoices[c1NlCount++] = v;
                c1Load = static_cast<uint8_t>(c1Load + stageCost);
            }
        }

        spanSubPtr_ = &spanSub;
        filtersPtr_ = filters.data();
        coeffsPtr_ = coeffs.data();
        numFrames_ = numFrames;
        numDelegated_ = c0Count;
        delegatedVoices_ = c0Voices;
        hwCacheFlushInvalidateLine(&spanSubPtr_);

        for (uint8_t d = 0u; d < c0Count; ++d) {
            const size_t v = static_cast<size_t>(c0Voices[d] & 3u);
            hwCacheFlushInvalidateRange(spanSub[v].data(), numFrames * sizeof(Frame4x));
            hwCacheFlushInvalidateRange(&filters[v], sizeof(Ms20DualFilter));
            hwCacheFlushInvalidateRange(&coeffs[v], sizeof(Ms20DualFilter::Coeffs));
        }

        uint32_t seq = lastSubmittedSeq_ + 1u;
        if (seq == 0u) {
            seq = 1u;
        }
        lastSubmittedSeq_ = seq;
        reqSeq_.store(seq, std::memory_order_seq_cst);
        hwCacheFlushInvalidateLine(&c1ReclaimedSeq_);

        for (uint8_t i = 0u; i < c1NlCount; ++i) {
            const size_t v = static_cast<size_t>(c1NlVoices[i] & 3u);
            filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
        }
        for (uint8_t i = 0u; i < linCount; ++i) {
            const size_t v = static_cast<size_t>(linVoices[i] & 3u);
            filters[v].processSpan4x(spanSub[v].data(), numFrames, coeffs[v]);
        }

        joinPending_ = true;
        pendingFrames_ = numFrames;
        pendingVoices_ = c0Voices;
        pendingCount_ = c0Count;
        return true;
    }

    // Completes a split-phase `beginBalanced4x` (no-op when nothing was delegated).
    void
    finishBalanced4x(VoiceSpanBuffer& spanSub, std::array<Ms20DualFilter, kNumOscillators>& filters,
                     const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs) noexcept {
        if (!joinPending_) {
            return;
        }
        joinPending_ = false;
        joinDelegated(spanSub, pendingFrames_, pendingVoices_, pendingCount_, filters, coeffs);
    }

    // Called exclusively by Core 0 (`app_idle` / inter-strip / inter-frame loop).
    bool tryAssist() noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        hwCacheFlushInvalidateLine(&c1ReclaimedSeq_);
        const uint32_t req = reqSeq_.load(std::memory_order_seq_cst);
        if (req == 0u || req == lastHandledSeq_) {
            return false;
        }
        if (seqReached(c1ReclaimedSeq_.load(std::memory_order_acquire), req)) {
            lastHandledSeq_ = req;
            return false;
        }

        // Dekker Side 0: announce intent to claim `req`, flush Group 0, then re-read Group 1.
        c0ClaimingSeq_.store(req, std::memory_order_seq_cst);
        hwCacheFlushInvalidateLine(&helperArmed_);
        hwCacheFlushInvalidateLine(&c1ReclaimedSeq_);
        if (seqReached(c1ReclaimedSeq_.load(std::memory_order_seq_cst), req) ||
            reqSeq_.load(std::memory_order_seq_cst) != req) {
            c0ClaimingSeq_.store(0u, std::memory_order_seq_cst);
            hwCacheFlushInvalidateLine(&helperArmed_);
            lastHandledSeq_ = req;
            return false;
        }
        claimedSeq_.store(req, std::memory_order_seq_cst);
        c0ClaimingSeq_.store(0u, std::memory_order_seq_cst);
        hwCacheFlushInvalidateLine(&helperArmed_);
        lastHandledSeq_ = req;

        hwCacheFlushInvalidateLine(&spanSubPtr_);
        VoiceSpanBuffer* spanPtr = spanSubPtr_;
        Ms20DualFilter* fltPtr = filtersPtr_;
        const Ms20DualFilter::Coeffs* cfPtr = coeffsPtr_;
        const size_t n = (numFrames_ <= kMaxFilterSpanFrames) ? numFrames_ : kMaxFilterSpanFrames;
        const size_t count =
            (numDelegated_ <= kMaxDelegatedVoices) ? numDelegated_ : kMaxDelegatedVoices;
        if (spanPtr != nullptr && fltPtr != nullptr && cfPtr != nullptr) {
            for (size_t d = 0u; d < count; ++d) {
                const size_t v = static_cast<size_t>(delegatedVoices_[d] & 3u);
                hwCacheFlushInvalidateRange((*spanPtr)[v].data(), n * sizeof(Frame4x));
                hwCacheFlushInvalidateRange(&fltPtr[v], sizeof(Ms20DualFilter));
                hwCacheFlushInvalidateRange(&cfPtr[v], sizeof(Ms20DualFilter::Coeffs));
                fltPtr[v].processSpan4x((*spanPtr)[v].data(), n, cfPtr[v]);
                hwCacheFlushInvalidateRange((*spanPtr)[v].data(), n * sizeof(Frame4x));
                hwCacheFlushInvalidateRange(&fltPtr[v], sizeof(Ms20DualFilter));
            }
        }
        assistedSpans_.store(assistedSpans_.load(std::memory_order_relaxed) + 1u,
                             std::memory_order_relaxed);
        doneSeq_.store(req, std::memory_order_release);
        hwCacheFlushInvalidateLine(&helperArmed_);
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
        hwCacheFlushInvalidateLine(&helperArmed_);
        if (doneSeq_.load(std::memory_order_acquire) != seq &&
            claimedSeq_.load(std::memory_order_acquire) != seq) {
            // Dekker Side 1: reclaim `seq` immediately so Core 1 never stalls behind busy Core 0.
            c1ReclaimedSeq_.store(seq, std::memory_order_seq_cst);
            hwCacheFlushInvalidateLine(&c1ReclaimedSeq_);
            hwCacheFlushInvalidateLine(&helperArmed_);
            uint32_t claimSpins = 0u;
            while (c0ClaimingSeq_.load(std::memory_order_seq_cst) == seq && ++claimSpins < 4096u) {
                hwCacheFlushInvalidateLine(&helperArmed_);
            }
            hwCacheFlushInvalidateLine(&helperArmed_);
            if (claimedSeq_.load(std::memory_order_seq_cst) != seq) {
                for (uint8_t d = 0u; d < safeCount; ++d) {
                    const size_t v = static_cast<size_t>(c0Voices[d] & 3u);
                    filters[v].processSpan4x(spanSub[v].data(), safeFrames, coeffs[v]);
                }
                return;
            }
        }

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
        uint32_t spins = 0u;
#endif
        while (doneSeq_.load(std::memory_order_acquire) != seq) {
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
            if ((spins & 15u) == 0u) {
                hwCacheFlushInvalidateLine(&helperArmed_);
            }
            if (++spins > 2097152u) {
                c1HelperTimedOut_ = 1u;
                return;
            }
#endif
        }
        for (uint8_t d = 0u; d < safeCount; ++d) {
            const size_t v = static_cast<size_t>(c0Voices[d] & 3u);
            hwCacheFlushInvalidateRange(spanSub[v].data(), safeFrames * sizeof(Frame4x));
            hwCacheFlushInvalidateRange(&filters[v], sizeof(Ms20DualFilter));
        }
    }

    // Group 0: Written EXCLUSIVELY by Core 0 (never dirtied by Core 1)
    alignas(64) std::atomic<uint32_t> helperArmed_{0u};
    std::atomic<uint32_t> c0ClaimingSeq_{0u};
    std::atomic<uint32_t> claimedSeq_{0u};
    std::atomic<uint32_t> doneSeq_{0u};
    std::atomic<uint32_t> assistedSpans_{0u};
    uint32_t lastHandledSeq_ = 0u;

    // Group 1: Written EXCLUSIVELY by Core 1 (never dirtied by Core 0)
    alignas(64) std::atomic<uint32_t> c1ReclaimedSeq_{0u};
    std::atomic<uint32_t> reqSeq_{0u};
    uint32_t lastSubmittedSeq_ = 0u;
    uint32_t c1HelperTimedOut_ = 0u;
    size_t pendingFrames_ = 0u;
    std::array<uint8_t, kMaxDelegatedVoices> pendingVoices_{0u, 0u, 0u};
    uint8_t pendingCount_ = 0u;
    bool joinPending_ = false;

    // Group 2: Written EXCLUSIVELY by Core 1 (never dirtied by Core 0)
    alignas(64) VoiceSpanBuffer* spanSubPtr_ = nullptr;
    Ms20DualFilter* filtersPtr_ = nullptr;
    const Ms20DualFilter::Coeffs* coeffsPtr_ = nullptr;
    size_t numFrames_ = 0u;
    uint8_t numDelegated_ = 2u;
    std::array<uint8_t, kMaxDelegatedVoices> delegatedVoices_{2u, 3u, 0u};
};

} // namespace ruido
