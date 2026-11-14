#pragma once

#include <array>
#include <cstddef>

namespace ruido {

struct StereoFrame {
    float left = 0.0f;
    float right = 0.0f;
};

// Two-stage cascaded 4x -> 2x -> 1x Linear-Phase Symmetric Half-Band FIR Stereo Decimator:
//  - Stage 1 (176.472 -> 88.236 kHz): 31-tap half-band (P=8 folded pairs, >90 dB stopband)
//  - Stage 2 (88.236  -> 44.118 kHz): 127-tap half-band (P=32 folded pairs, >87 dB stopband
//    all the way to 20.0 kHz, <0.0005 dB passband ripple)
//  - Exact linear phase (0 deg dispersion): preserves flat square-wave tops & peak crest factor.
class StereoHalfBandDecimator4x {
  public:
    StereoHalfBandDecimator4x() noexcept {}

    void reset() noexcept;

    // Consumes 4 consecutive oversampled stereo frames and returns 1 decimated 44.118 kHz frame.
    StereoFrame process4x(const std::array<StereoFrame, 4>& in4x) noexcept;

    // Batch-decimates `count` consecutive 4x stereo frames with ring indices hoisted in registers.
    void processSpan4x(const std::array<StereoFrame, 4>* in4x, StereoFrame* out,
                       size_t count) noexcept;

  private:
    static constexpr size_t kStage1Pairs = 8u;
    static constexpr size_t kStage1OddPeriod = 2u * kStage1Pairs; // 16

    static constexpr size_t kStage2Pairs = 32u;
    static constexpr size_t kStage2OddPeriod = 2u * kStage2Pairs; // 64

    // Polyphase half-band state:
    //  - Even branch (s0) is a pure (P - 1)-sample delay line hitting center tap h[0] = 0.5
    //  - Odd branch (s1) uses a double-mirrored ring buffer so all P symmetric tap pairs
    //    execute with contiguous unit-stride pointer reads and zero modulo/mask operations.
    struct ChannelState {
        std::array<float, kStage1Pairs> even1{};
        std::array<float, 2u * kStage1OddPeriod> odd1{};
        std::array<float, kStage2Pairs> even2{};
        std::array<float, 2u * kStage2OddPeriod> odd2{};

        void reset() noexcept {
            even1.fill(0.0f);
            odd1.fill(0.0f);
            even2.fill(0.0f);
            odd2.fill(0.0f);
        }
    };

    StereoFrame decimate2xStage1Stereo(StereoFrame s0, StereoFrame s1) noexcept;
    StereoFrame decimate2xStage2Stereo(StereoFrame s0, StereoFrame s1) noexcept;

    ChannelState left_{};
    ChannelState right_{};
    size_t ew1_ = 0u;
    size_t ow1_ = 0u;
    size_t ew2_ = 0u;
    size_t ow2_ = 0u;
};

} // namespace ruido
