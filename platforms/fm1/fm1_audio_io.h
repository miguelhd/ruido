#pragma once

// First-party standalone FM-1 audio output & clock-domain bridge primitives:
//  1. Fm1TpdfQuantizer: +-1.45f dacSafetyClip ceiling -> [-32768, +32767] with uncorrelated
//     stereo 1-LSB TPDF triangular dither and pre-dither float32 master pot scaling.
//  2. Fm1AudioBlockRing: Cache-line-isolated lock-free SPSC Core 1 -> Core 0 audio block ring
//     paced strictly by ALNK0 24 MHz crystal DMA half-buffer consumption.
//  3. Fm1UsbDriftResampler: PI-servoed 4-point Hermite fractional stereo interpolator bridging
//     +-50 ppm 24 MHz crystal vs USB-C host SOF clock drift (44.1 kHz & 48 kHz UAC1).
//
// Pure embedded C++17 (zero SDK/GPL headers) so identical math is verified in `tests/test_dsp`.

#include "core/CacheSync.h"
#include "core/Config.h"
#include "core/DspMath.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ruido {

class Fm1TpdfQuantizer {
  public:
    // Maps +-1.45f dacSafetyClip ceiling to +-32762.75f, leaving > 3 LSB headroom for +-1.0 LSB
    // TPDF triangular dither + 0.5 LSB rounding so peaks never hard-clip [-32768, +32767].
    static constexpr float kFloatToQ15Scale = 22595.0f;
    static constexpr float kQ15ToFloatScale = 1.0f / kFloatToQ15Scale;

    constexpr Fm1TpdfQuantizer() noexcept = default;

    void reset(uint32_t seedL = 0x9E3779B9u, uint32_t seedR = 0x85EBCA6Bu) noexcept {
        rngL_ = (seedL != 0u) ? seedL : 0x9E3779B9u;
        rngR_ = (seedR != 0u) ? seedR : 0x85EBCA6Bu;
    }

    RUIDO_ALWAYS_INLINE int16_t quantizeLeft(float sample, float masterGain = 1.0f) noexcept {
        return quantizeChannel(sample * masterGain, rngL_);
    }

    RUIDO_ALWAYS_INLINE int16_t quantizeRight(float sample, float masterGain = 1.0f) noexcept {
        return quantizeChannel(sample * masterGain, rngR_);
    }

    RUIDO_ALWAYS_INLINE int16_t quantizeLeftUnity(float sample) noexcept {
        return quantizeChannel(sample, rngL_);
    }

    RUIDO_ALWAYS_INLINE int16_t quantizeRightUnity(float sample) noexcept {
        return quantizeChannel(sample, rngR_);
    }

  private:
    // Single-step xorshift32 -> exact 1-LSB TPDF triangular dither in (-1.0f, +1.0f) with zero
    // mean and variance = 1/6 LSB^2: subtracts two independent 16-bit uniform halves in integer
    // registers (`[-65535, +65535]`) with a single hardware `itof`.
    static RUIDO_ALWAYS_INLINE float nextTpdf1Lsb(uint32_t& state) noexcept {
        uint32_t x = (state != 0u) ? state : 0x6D2B79F5u;
        x ^= x << 13u;
        x ^= x >> 17u;
        x ^= x << 5u;
        state = x;
        const int32_t diff = static_cast<int32_t>(x >> 16u) - static_cast<int32_t>(x & 0xFFFFu);
        return static_cast<float>(diff) * (1.0f / 65536.0f);
    }

    static RUIDO_ALWAYS_INLINE int16_t quantizeChannel(float scaledInput, uint32_t& rng) noexcept {
        // Catch NaN via a single hardware `iff` without stack union bit-punning (`+-Inf` clamps
        // cleanly at +-32767.0f below).
        const float clean = (scaledInput == scaledInput) ? scaledInput : 0.0f;
        const float qFloat = clean * kFloatToQ15Scale + nextTpdf1Lsb(rng);
        const float rounded = qFloat + ((qFloat >= 0.0f) ? 0.5f : -0.5f);
        if (rounded >= 32767.0f) {
            return 32767;
        }
        if (rounded <= -32767.0f) {
            return -32767;
        }
        return static_cast<int16_t>(static_cast<int32_t>(rounded));
    }

    uint32_t rngL_ = 0x9E3779B9u;
    uint32_t rngR_ = 0x85EBCA6Bu;
};

struct alignas(kHwCacheLineBytes) Fm1AudioBlock {
    std::array<int16_t, 2u * kBlockFrames> lr{};
    int32_t peakAbsQ15 = 0;
};
static_assert(sizeof(Fm1AudioBlock) % kHwCacheLineBytes == 0u,
              "Fm1AudioBlock must be an exact multiple of 32-byte L1 D-Cache lines");

class Fm1AudioBlockRing {
  public:
    static constexpr uint32_t kNumBlocks = 16u; // Power of 2: 16 x 64 = 1024 frames (4 DMA halves)
    static constexpr uint32_t kMask = kNumBlocks - 1u;

    void reset() noexcept {
        writeIdx_.store(0u, std::memory_order_relaxed);
        readIdx_.store(0u, std::memory_order_relaxed);
        for (auto& blk : blocks_) {
            blk.lr.fill(0);
            blk.peakAbsQ15 = 0;
        }
        hwCacheFlushInvalidateRange(this, sizeof(*this));
    }

    // Core 1 producer: inspect next slot if ring is not full (`(w - r) < kNumBlocks`).
    Fm1AudioBlock* beginPush() noexcept {
        hwCacheFlushInvalidateLine(&readIdx_);
        const uint32_t w = writeIdx_.load(std::memory_order_relaxed);
        const uint32_t r = readIdx_.load(std::memory_order_acquire);
        if ((w - r) >= kNumBlocks) {
            return nullptr;
        }
        return &blocks_[w & kMask];
    }

    void commitPush() noexcept {
        const uint32_t w = writeIdx_.load(std::memory_order_relaxed);
        hwCacheFlushInvalidateRange(&blocks_[w & kMask], sizeof(Fm1AudioBlock));
        writeIdx_.store(w + 1u, std::memory_order_release);
        hwCacheFlushInvalidateLine(&writeIdx_);
    }

    // Core 0 ALNK0 DMA consumer: inspect oldest ready block if available.
    const Fm1AudioBlock* beginPop() const noexcept {
        hwCacheFlushInvalidateLine(&writeIdx_);
        const uint32_t r = readIdx_.load(std::memory_order_relaxed);
        const uint32_t w = writeIdx_.load(std::memory_order_acquire);
        if (r == w) {
            return nullptr;
        }
        const Fm1AudioBlock* blk = &blocks_[r & kMask];
        hwCacheFlushInvalidateRange(blk, sizeof(Fm1AudioBlock));
        return blk;
    }

    void commitPop() noexcept {
        const uint32_t r = readIdx_.load(std::memory_order_relaxed);
        readIdx_.store(r + 1u, std::memory_order_release);
        hwCacheFlushInvalidateLine(&readIdx_);
    }

    uint32_t availableBlocks() const noexcept {
        hwCacheFlushInvalidateLine(&writeIdx_);
        hwCacheFlushInvalidateLine(&readIdx_);
        const uint32_t w = writeIdx_.load(std::memory_order_acquire);
        const uint32_t r = readIdx_.load(std::memory_order_acquire);
        return w - r;
    }

  private:
    alignas(64) std::atomic<uint32_t> writeIdx_{0u};
    alignas(64) std::atomic<uint32_t> readIdx_{0u};
    alignas(64) std::array<Fm1AudioBlock, kNumBlocks> blocks_{};
};

class Fm1UsbDriftResampler {
  public:
    static constexpr float kMaxDriftTrim = 0.0005f; // +-500 ppm (10x margin over +-50 ppm spec)

    constexpr Fm1UsbDriftResampler() noexcept = default;

    void reset(bool rate48k = false, int32_t targetFill = 104) noexcept {
        histL_ = {0.0f, 0.0f, 0.0f, 0.0f};
        histR_ = {0.0f, 0.0f, 0.0f, 0.0f};
        phase_ = 0.0f;
        fillLp1_ = static_cast<float>(targetFill);
        avgFill_ = static_cast<float>(targetFill);
        integralTrim_ = 0.0f;
        currentStep_ = rate48k ? (44100.0f / 48000.0f) : 1.0f;
        rate48k_ = rate48k;
        primed_ = false;
        dither_.reset(0xC2B2AE35u, 0x27D4EB2Fu);
    }

    // Resample `numInFrames` of 44.1 kHz crystal-locked stereo Q15 (`inLrQ15` stride 2) into packed
    // 16-bit USB ring `ring` (`ringSize` power of 2) starting at `writePos`, servoing fractional
    // step rate so `fillFrames` stays centered on `targetFill` under +-50 ppm host SOF drift.
    // Returns number of output USB frames written (or 0 on ring overrun guard).
    uint32_t processBlockToRing(const int32_t* inLrQ15, uint32_t numInFrames, bool rate48k,
                                int32_t fillFrames, int32_t targetFill, uint32_t* ring,
                                uint32_t ringMask, uint32_t writePos, uint32_t freeSlots,
                                uint32_t streamSeq = 0u) noexcept {
        if (inLrQ15 == nullptr || ring == nullptr || numInFrames == 0u) {
            return 0u;
        }
        if (!primed_ || rate48k != rate48k_ || streamSeq != lastStreamSeq_) {
            reset(rate48k, targetFill);
            lastStreamSeq_ = streamSeq;
            primed_ = true;
        }

        // 2-pole LPF (-47 dB at 5-block packet beat) rejects the +-22-frame 1 ms USB SOF packet
        // cadence sawtooth down to < 0.1 frame, driving a critically damped (zeta ~ 1.08) PI servo.
        fillLp1_ += 0.08f * (static_cast<float>(fillFrames) - fillLp1_);
        avgFill_ += 0.08f * (fillLp1_ - avgFill_);
        const float err = clampf(avgFill_ - static_cast<float>(targetFill), -64.0f, 64.0f);
        integralTrim_ = clampf(integralTrim_ + err * 2.0e-7f, -kMaxDriftTrim, kMaxDriftTrim);
        const float propTrim = clampf(err * 6.0e-5f, -kMaxDriftTrim, kMaxDriftTrim);
        const float totalTrim = clampf(propTrim + integralTrim_, -kMaxDriftTrim, kMaxDriftTrim);

        const float baseStep = rate48k ? (44100.0f / 48000.0f) : 1.0f;
        const float targetStep = baseStep * (1.0f + totalTrim);
        currentStep_ += 0.15f * (targetStep - currentStep_);
        const float step = currentStep_;

        // Worst-case output count for `numInFrames` is ceil(numInFrames / step) + 2
        const uint32_t maxNeeded =
            rate48k ? (numInFrames + (numInFrames >> 3u) + 4u) : (numInFrames + 4u);
        if (freeSlots < maxNeeded) {
            return 0u;
        }

        constexpr float kNorm = 1.0f / 32768.0f;
        constexpr float kOutScale = 32768.0f / Fm1TpdfQuantizer::kFloatToQ15Scale;
        uint32_t outCount = 0u;

        for (uint32_t i = 0u; i < numInFrames; ++i) {
            histL_[0] = histL_[1];
            histL_[1] = histL_[2];
            histL_[2] = histL_[3];
            histL_[3] = static_cast<float>(inLrQ15[2u * i]) * kNorm;

            histR_[0] = histR_[1];
            histR_[1] = histR_[2];
            histR_[2] = histR_[3];
            histR_[3] = static_cast<float>(inLrQ15[2u * i + 1u]) * kNorm;

            while (phase_ < 1.0f) {
                const float yL = evalHermite4pt(histL_[0], histL_[1], histL_[2], histL_[3], phase_);
                const float yR = evalHermite4pt(histR_[0], histR_[1], histR_[2], histR_[3], phase_);
                const int16_t sL = dither_.quantizeLeft(yL, kOutScale);
                const int16_t sR = dither_.quantizeRight(yR, kOutScale);
                const uint32_t packed = static_cast<uint32_t>(static_cast<uint16_t>(sL)) |
                                        (static_cast<uint32_t>(static_cast<uint16_t>(sR)) << 16u);
                ring[(writePos + outCount) & ringMask] = packed;
                ++outCount;
                phase_ += step;
            }
            phase_ -= 1.0f;
        }

        return outCount;
    }

    float currentStep() const noexcept { return currentStep_; }

  private:
    static RUIDO_ALWAYS_INLINE float evalHermite4pt(float y0, float y1, float y2, float y3,
                                                    float t) noexcept {
        const float c1 = 0.5f * (y2 - y0);
        const float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
        const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
        return ((c3 * t + c2) * t + c1) * t + y1;
    }

    std::array<float, 4> histL_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> histR_{0.0f, 0.0f, 0.0f, 0.0f};
    float phase_ = 0.0f;
    float fillLp1_ = 104.0f;
    float avgFill_ = 104.0f;
    float integralTrim_ = 0.0f;
    float currentStep_ = 1.0f;
    uint32_t lastStreamSeq_ = 0u;
    bool rate48k_ = false;
    bool primed_ = false;
    Fm1TpdfQuantizer dither_{};
};

namespace fm1 {

// ALNK0 DMA helpers: widen one 64-frame Q15 block, or slew the last output to silence on a miss.
inline void copyStereoBlockToDma(const Fm1AudioBlock& src, int32_t* dst, uint32_t frames,
                                 int32_t& lastL, int32_t& lastR) noexcept {
    for (uint32_t i = 0u; i < frames; ++i) {
        dst[2u * i] = static_cast<int32_t>(src.lr[2u * i]);
        dst[2u * i + 1u] = static_cast<int32_t>(src.lr[2u * i + 1u]);
    }
    if (frames > 0u) {
        lastL = dst[2u * (frames - 1u)];
        lastR = dst[2u * (frames - 1u) + 1u];
    }
}

inline void slewStereoBlockToZero(int32_t* dst, uint32_t frames, int32_t& lastL,
                                  int32_t& lastR) noexcept {
    int32_t l = lastL;
    int32_t r = lastR;
    for (uint32_t i = 0u; i < frames; ++i) {
        l = ((l * 31) + ((l >> 31) & 31)) >> 5;
        r = ((r * 31) + ((r >> 31) & 31)) >> 5;
        dst[2u * i] = l;
        dst[2u * i + 1u] = r;
    }
    lastL = l;
    lastR = r;
}

} // namespace fm1

} // namespace ruido
