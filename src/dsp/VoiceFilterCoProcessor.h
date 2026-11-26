#pragma once

#include "core/CacheSync.h"
#include "core/Config.h"
#include "dsp/SallenKeyFilter.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

// Cache-line-isolated, 100% CAS-free / spinlock-free 3-slot per-voice Dekker work-stealing SPSC
// co-processor dynamically partitioning the 4 independent 4x oversampled MS-20 ZDF voice filter
// chains across Core 0 and Core 1:
//  - Core 1 keeps 1 nonlinear voice locally and exposes the remaining 1..3 nonlinear voices as
//    independent 1-voice Dekker slots (`0 -> 1 -> 2` claimed forward by Core 0, `2 -> 1 -> 0`
//    reclaimed backward by Core 1).
//  - Zero idle spin while unclaimed work remains, zero 4-voice solo collapse on Core 0 ISR jitter,
//    and zero `compare_exchange` / `lockset` / `cli` instructions.
//  - Strict single-writer 64-byte cache-line isolation + explicit `flushinv` handoffs on AC791N.
class alignas(64) VoiceFilterCoProcessor {
  public:
    using Frame4x = std::array<float, kOversampleFactor>;
    using VoiceSpanBuffer = std::array<std::array<Frame4x, kMaxFilterSpanFrames>, kNumOscillators>;
    static constexpr size_t kMaxDelegatedVoices = 3u;
    static constexpr uint8_t kSplitHpOnlyFlag = 0x04u;

    VoiceFilterCoProcessor() noexcept {}

    void reset() noexcept {
        helperArmed_.store(0u, std::memory_order_relaxed);
        for (size_t d = 0u; d < kMaxDelegatedVoices; ++d) {
            c0Claiming_[d].store(0u, std::memory_order_relaxed);
            claimed_[d].store(0u, std::memory_order_relaxed);
            done_[d].store(0u, std::memory_order_relaxed);
            lastHandled_[d] = 0u;
            c1Reclaimed_[d].store(0u, std::memory_order_relaxed);
            slotSeq_[d].store(0u, std::memory_order_relaxed);
        }
        assistedSpans_.store(0u, std::memory_order_relaxed);
        lastSubmittedSeq_ = 0u;
        c1HelperTimedOut_ = 0u;
        joinPending_ = false;
        pendingCount_ = 0u;
        pendingFrames_ = 0u;
        pendingVoices_ = {0u, 0u, 0u};
        spanSubPtr_ = nullptr;
        filtersPtr_ = nullptr;
        coeffsPtr_ = nullptr;
        numFrames_ = 0u;
        numDelegated_ = 0u;
        delegatedVoices_ = {0u, 0u, 0u};
        hwCacheFlushInvalidateRange(this, sizeof(*this));
    }

    // Called exclusively by Core 0 (helper thread).
    void setHelperArmed(bool armed) noexcept {
        if (armed) {
            helperArmed_.store(1u, std::memory_order_seq_cst);
            flushGroup0();
            return;
        }
        helperArmed_.store(0u, std::memory_order_seq_cst);
        flushGroup0();
        (void)tryAssist();
    }

    // Non-blocking emergency disarm invoked ONLY on Core 0 when Core 1 silicon has faulted/stalled.
    void abortFromFault() noexcept {
        helperArmed_.store(0u, std::memory_order_release);
        flushGroup0();
    }

    bool hasPendingWork() const noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        flushGroup1();
        for (size_t d = 0u; d < kMaxDelegatedVoices; ++d) {
            const uint32_t req = slotSeq_[d].load(std::memory_order_relaxed);
            if (req != 0u && !seqReached(done_[d].load(std::memory_order_relaxed), req) &&
                !seqReached(c1Reclaimed_[d].load(std::memory_order_relaxed), req)) {
                return true;
            }
        }
        return false;
    }

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

    // Split-phase variant: publishes 1..3 single-voice Dekker slots for Core 0 (`0 -> 1 -> 2`) and
    // filters 1 nonlinear voice + any linear voices locally on Core 1 while Core 0 claims.
    bool beginBalanced4x(VoiceSpanBuffer& spanSub, size_t numFrames,
                         std::array<Ms20DualFilter, kNumOscillators>& filters,
                         const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs,
                         uint8_t activeVoiceMask = 0x0Fu,
                         bool heavyCore1PreFilter = false) noexcept {
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
        flushGroup0();
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
            const bool hpNl = !coeffs[v].hpLinearBypass && !coeffs[v].hpLinearAcCouple;
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

        // Stage-balanced `5 + 3` / `4 + 2` / `3 + 1` dual-core partition:
        //  - 8 stages (`4` dual-NL): Core 0 gets `v0(HP+LP) + v1(HP+LP) + v2(HP-only)` = 5 stages
        //    (~84%), Core 1 gets `v3(HP+LP) + v2(LP-only)` + non-filter DSP = 3 stages (~85%).
        //  - 7 stages (`3` dual + `1` single): Core 0 gets `v0(HP+LP) + v1(HP+LP) + v3(single)` =
        //    5 stages (~84%), Core 1 gets `v2(HP+LP)` + non-filter DSP = 2 stages (~68%).
        //  - 6 stages (`3` dual or `2` dual + `2` single): Core 0 gets `v0 + v1` = 4 stages (~67%),
        //    Core 1 gets remaining 2 stages + non-filter DSP (~68%).
        //  - <= 5 stages: Core 0 gets `nlCount - 1` voices (<= 4 stages), Core 1 gets 1 voice.
        const uint8_t totalNlStages = static_cast<uint8_t>(2u * dualCount + singleCount);
        uint8_t c0Count = static_cast<uint8_t>(nlCount - 1u);
        std::array<uint8_t, kMaxDelegatedVoices> c0Voices{0u, 0u, 0u};
        std::array<uint8_t, kNumOscillators> c1FullVoices{};
        uint8_t c1FullCount = 0u;

        (void)heavyCore1PreFilter;
        if (totalNlStages >= 8u && nlCount == 4u) {
            c0Count = 3u;
            // Put `v2(HP-only)` in slot 0 FIRST so Core 0 finishes `v2.HPF` at ~17% of the span,
            // allowing Core 1 to execute `v2.LPF` in parallel with Core 0's `v1(HP+LP)` tail!
            c0Voices[0] = static_cast<uint8_t>(nlVoices[2] | kSplitHpOnlyFlag);
            c0Voices[1] = nlVoices[0];
            c0Voices[2] = nlVoices[1];
            c1FullVoices[c1FullCount++] = nlVoices[3];
        } else if (totalNlStages == 7u && nlCount == 4u) {
            c0Count = 3u;
            c0Voices[0] = nlVoices[0];
            c0Voices[1] = nlVoices[1];
            c0Voices[2] = nlVoices[3]; // single-NL voice (1 stage -> 5 stages on C0)
            c1FullVoices[c1FullCount++] = nlVoices[2]; // dual-NL voice (2 stages on C1)
        } else if (totalNlStages == 6u && nlCount >= 3u) {
            c0Count = 2u;
            c0Voices[0] = nlVoices[0];
            c0Voices[1] = nlVoices[1];
            for (uint8_t i = 2u; i < nlCount; ++i) {
                c1FullVoices[c1FullCount++] = nlVoices[i];
            }
        } else {
            for (uint8_t d = 0u; d < c0Count; ++d) {
                c0Voices[d] = nlVoices[d];
            }
            c1FullVoices[c1FullCount++] = nlVoices[nlCount - 1u];
        }

        spanSubPtr_ = &spanSub;
        filtersPtr_ = filters.data();
        coeffsPtr_ = coeffs.data();
        numFrames_ = numFrames;
        numDelegated_ = c0Count;
        delegatedVoices_ = c0Voices;
        flushGroup2();

        for (uint8_t d = 0u; d < c0Count; ++d) {
            const uint8_t entry = c0Voices[d];
            const size_t v = static_cast<size_t>(entry & 3u);
            hwCacheFlushInvalidateRange(spanSub[v].data(), numFrames * sizeof(Frame4x));
            if ((entry & kSplitHpOnlyFlag) != 0u) {
                filters[v].flushHpStateCache();
            } else {
                hwCacheFlushInvalidateRange(&filters[v], sizeof(Ms20DualFilter));
            }
            hwCacheFlushInvalidateRange(&coeffs[v], sizeof(Ms20DualFilter::Coeffs));
        }

        uint32_t seq = lastSubmittedSeq_ + 1u;
        if (seq == 0u) {
            seq = 1u;
        }
        lastSubmittedSeq_ = seq;
        for (size_t d = 0u; d < kMaxDelegatedVoices; ++d) {
            const uint32_t slotVal = (d < c0Count) ? seq : 0u;
            slotSeq_[d].store(slotVal, (d + 1u == kMaxDelegatedVoices) ? std::memory_order_seq_cst
                                                                       : std::memory_order_release);
        }
        flushGroup1();

        for (uint8_t i = 0u; i < c1FullCount; ++i) {
            const size_t c1OwnVoice = static_cast<size_t>(c1FullVoices[i] & 3u);
            filters[c1OwnVoice].processSpan4x(spanSub[c1OwnVoice].data(), numFrames,
                                              coeffs[c1OwnVoice]);
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

    // Called exclusively by Core 0 (`app_idle` / inter-strip / intra-strip `coProcYield`).
    // Claims 1 voice at a time in forward order (`0 -> 1 -> 2`), publishing `done_[d]` immediately
    // after each single voice (~45 us) so Core 1 never stalls behind a multi-voice bundle.
    bool tryAssist() noexcept {
        if (helperArmed_.load(std::memory_order_relaxed) == 0u) {
            return false;
        }
        bool didWork = false;
        flushGroup1();
        for (size_t d = 0u; d < kMaxDelegatedVoices; ++d) {
            if (didWork) {
                flushGroup1();
            }
            const uint32_t req = slotSeq_[d].load(std::memory_order_seq_cst);
            if (req == 0u || req == lastHandled_[d]) {
                continue;
            }
            if (seqReached(c1Reclaimed_[d].load(std::memory_order_acquire), req)) {
                lastHandled_[d] = req;
                continue;
            }
            c0Claiming_[d].store(req, std::memory_order_seq_cst);
            flushGroup0();
            flushGroup1();
            if (seqReached(c1Reclaimed_[d].load(std::memory_order_seq_cst), req) ||
                slotSeq_[d].load(std::memory_order_seq_cst) != req) {
                c0Claiming_[d].store(0u, std::memory_order_seq_cst);
                flushGroup0();
                lastHandled_[d] = req;
                continue;
            }
            claimed_[d].store(req, std::memory_order_seq_cst);
            c0Claiming_[d].store(0u, std::memory_order_seq_cst);
            flushGroup0();
            lastHandled_[d] = req;

            flushGroup2();
            VoiceSpanBuffer* spanPtr = spanSubPtr_;
            Ms20DualFilter* fltPtr = filtersPtr_;
            const Ms20DualFilter::Coeffs* cfPtr = coeffsPtr_;
            const size_t n =
                (numFrames_ <= kMaxFilterSpanFrames) ? numFrames_ : kMaxFilterSpanFrames;
            const size_t count =
                (numDelegated_ <= kMaxDelegatedVoices) ? numDelegated_ : kMaxDelegatedVoices;
            if (d < count && spanPtr != nullptr && fltPtr != nullptr && cfPtr != nullptr) {
                filterSingleSlotOnC0(*spanPtr, fltPtr, cfPtr, delegatedVoices_[d], n);
            }
            assistedSpans_.store(assistedSpans_.load(std::memory_order_relaxed) + 1u,
                                 std::memory_order_relaxed);
            done_[d].store(req, std::memory_order_release);
            flushGroup0();
            didWork = true;
        }
        return didWork;
    }

    uint32_t assistedSpans() const noexcept {
        return assistedSpans_.load(std::memory_order_relaxed);
    }

  private:
    static constexpr bool seqReached(uint32_t observed, uint32_t target) noexcept {
        return observed != 0u && static_cast<int32_t>(observed - target) >= 0;
    }

    void flushGroup0() const noexcept { hwCacheFlushInvalidateRange(&helperArmed_, 64u); }
    void flushGroup1() const noexcept { hwCacheFlushInvalidateLine(&c1Reclaimed_[0]); }
    void flushGroup2() const noexcept { hwCacheFlushInvalidateRange(&spanSubPtr_, 64u); }

    static void filterSingleSlotOnC0(VoiceSpanBuffer& spanSub, Ms20DualFilter* fltPtr,
                                     const Ms20DualFilter::Coeffs* cfPtr, uint8_t entry,
                                     size_t n) noexcept {
        const size_t v = static_cast<size_t>(entry & 3u);
        const bool hpOnly = (entry & kSplitHpOnlyFlag) != 0u;
        hwCacheFlushInvalidateRange(spanSub[v].data(), n * sizeof(Frame4x));
        hwCacheFlushInvalidateRange(&cfPtr[v], sizeof(Ms20DualFilter::Coeffs));
        if (hpOnly) {
            fltPtr[v].flushHpStateCache();
            fltPtr[v].processHpOnlySpan4x(spanSub[v].data(), n, cfPtr[v]);
            fltPtr[v].flushHpStateCache();
        } else {
            hwCacheFlushInvalidateRange(&fltPtr[v], sizeof(Ms20DualFilter));
            fltPtr[v].processSpan4x(spanSub[v].data(), n, cfPtr[v]);
            hwCacheFlushInvalidateRange(&fltPtr[v], sizeof(Ms20DualFilter));
        }
        hwCacheFlushInvalidateRange(spanSub[v].data(), n * sizeof(Frame4x));
    }

    void joinDelegated(VoiceSpanBuffer& spanSub, size_t numFrames,
                       const std::array<uint8_t, kMaxDelegatedVoices>& c0Voices, uint8_t c0Count,
                       std::array<Ms20DualFilter, kNumOscillators>& filters,
                       const std::array<Ms20DualFilter::Coeffs, kNumOscillators>& coeffs) noexcept {
        const size_t safeFrames =
            (numFrames <= kMaxFilterSpanFrames) ? numFrames : kMaxFilterSpanFrames;
        const size_t safeCount =
            (c0Count <= kMaxDelegatedVoices) ? static_cast<size_t>(c0Count) : kMaxDelegatedVoices;
        const uint32_t seq = lastSubmittedSeq_;
        std::array<bool, kMaxDelegatedVoices> c0Owned{false, false, false};

        // Pass 1 (reverse order `safeCount - 1 -> 0`): reclaim unclaimed voices ONE AT A TIME and
        // execute them immediately on Core 1 so Core 0 can keep stealing lower slots in parallel
        // and Core 1 never spins idle while unclaimed voices remain.
        for (size_t rev = 0u; rev < safeCount; ++rev) {
            const size_t d = (safeCount - 1u) - rev;
            flushGroup0();
            if (done_[d].load(std::memory_order_acquire) == seq ||
                claimed_[d].load(std::memory_order_acquire) == seq) {
                c0Owned[d] = true;
                continue;
            }
            c1Reclaimed_[d].store(seq, std::memory_order_seq_cst);
            flushGroup1();
            flushGroup0();
            uint32_t claimSpins = 0u;
            while (c0Claiming_[d].load(std::memory_order_seq_cst) == seq && ++claimSpins < 4096u) {
                flushGroup0();
            }
            flushGroup0();
            if (claimed_[d].load(std::memory_order_seq_cst) == seq) {
                c0Owned[d] = true;
            } else {
                const size_t v = static_cast<size_t>(c0Voices[d] & 3u);
                filters[v].processSpan4x(spanSub[v].data(), safeFrames, coeffs[v]);
            }
        }

        // Pass 2: join slots claimed by Core 0 and invalidate Core 1's D-Cache for their outputs.
        for (size_t d = 0u; d < safeCount; ++d) {
            if (!c0Owned[d]) {
                continue;
            }
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
            uint32_t spins = 0u;
#endif
            while (done_[d].load(std::memory_order_acquire) != seq) {
#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
                if ((spins & 63u) == 0u) {
                    flushGroup0();
                }
                if (++spins > 2097152u) {
                    c1HelperTimedOut_ = 1u;
                    return;
                }
#endif
            }
            const uint8_t entry = c0Voices[d];
            const size_t v = static_cast<size_t>(entry & 3u);
            hwCacheFlushInvalidateRange(spanSub[v].data(), safeFrames * sizeof(Frame4x));
            if ((entry & kSplitHpOnlyFlag) != 0u) {
                filters[v].flushHpStateCache();
                filters[v].processLpOnlySpan4x(spanSub[v].data(), safeFrames, coeffs[v]);
            } else {
                hwCacheFlushInvalidateRange(&filters[v], sizeof(Ms20DualFilter));
            }
        }
    }

    // Group 0: Written EXCLUSIVELY by Core 0 (<= 64 B = 2 x 32 B q32DSP L1 D-Cache lines)
    alignas(64) std::atomic<uint32_t> helperArmed_{0u};
    std::array<std::atomic<uint32_t>, kMaxDelegatedVoices> c0Claiming_{};
    std::array<std::atomic<uint32_t>, kMaxDelegatedVoices> claimed_{};
    std::array<std::atomic<uint32_t>, kMaxDelegatedVoices> done_{};
    std::atomic<uint32_t> assistedSpans_{0u};
    std::array<uint32_t, kMaxDelegatedVoices> lastHandled_{0u, 0u, 0u};

    // Group 1: Written EXCLUSIVELY by Core 1 (shared atomics in [0..31] = single 32 B cache line)
    alignas(64) std::array<std::atomic<uint32_t>, kMaxDelegatedVoices> c1Reclaimed_{};
    std::array<std::atomic<uint32_t>, kMaxDelegatedVoices> slotSeq_{};
    uint32_t lastSubmittedSeq_ = 0u;
    uint32_t c1HelperTimedOut_ = 0u;
    size_t pendingFrames_ = 0u;
    std::array<uint8_t, kMaxDelegatedVoices> pendingVoices_{0u, 0u, 0u};
    uint8_t pendingCount_ = 0u;
    bool joinPending_ = false;

    // Group 2: Written EXCLUSIVELY by Core 1 before publishing `slotSeq_`
    alignas(64) VoiceSpanBuffer* spanSubPtr_ = nullptr;
    Ms20DualFilter* filtersPtr_ = nullptr;
    const Ms20DualFilter::Coeffs* coeffsPtr_ = nullptr;
    size_t numFrames_ = 0u;
    uint8_t numDelegated_ = 0u;
    std::array<uint8_t, kMaxDelegatedVoices> delegatedVoices_{0u, 0u, 0u};
};

} // namespace ruido
