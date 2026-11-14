#include "dsp/HalfBandDecimator.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

// Kaiser-windowed (beta = 8.6, P = 8) half-band taps normalized for exact 0.0 dB DC gain
constexpr std::array<float, 8> kStage1Taps = {
    3.133159608e-01f,  // h[+-1]
    -9.193905366e-02f, // h[+-3]
    4.249729529e-02f,  // h[+-5]
    -2.019388921e-02f, // h[+-7]
    8.801246793e-03f,  // h[+-9]
    -3.243871523e-03f, // h[+-11]
    8.995069499e-04f,  // h[+-13]
    -1.371954671e-04f, // h[+-15]
};

// Kaiser-windowed (beta = 8.8, P = 32) half-band taps normalized for exact 0.0 dB DC gain
constexpr std::array<float, 32> kStage2Taps = {
    3.179913124e-01f,  // h[+-1]
    -1.051425983e-01f, // h[+-3]
    6.207113656e-02f,  // h[+-5]
    -4.326901119e-02f, // h[+-7]
    3.257420503e-02f,  // h[+-9]
    -2.558296834e-02f, // h[+-11]
    2.060451308e-02f,  // h[+-13]
    -1.685187847e-02f, // h[+-15]
    1.390978498e-02f,  // h[+-17]
    -1.153855420e-02f, // h[+-19]
    9.590066286e-03f,  // h[+-21]
    -7.967572330e-03f, // h[+-23]
    6.604815212e-03f,  // h[+-25]
    -5.454457649e-03f, // h[+-27]
    4.481332619e-03f,  // h[+-29]
    -3.658344575e-03f, // h[+-31]
    2.963901076e-03f,  // h[+-33]
    -2.380264921e-03f, // h[+-35]
    1.892479002e-03f,  // h[+-37]
    -1.487657267e-03f, // h[+-39]
    1.154514500e-03f,  // h[+-41]
    -8.830538703e-04f, // h[+-43]
    6.643591864e-04f,  // h[+-45]
    -4.904562346e-04f, // h[+-47]
    3.542188561e-04f,  // h[+-49]
    -2.493029391e-04f, // h[+-51]
    1.700966640e-04f,  // h[+-53]
    -1.116790024e-04f, // h[+-55]
    6.978112958e-05f,  // h[+-57]
    -4.074737628e-05f, // h[+-59]
    2.149381846e-05f,  // h[+-61]
    -9.463699574e-06f, // h[+-63]
};

} // namespace

void StereoHalfBandDecimator4x::reset() noexcept {
    left_.reset();
    right_.reset();
    ew1_ = 0u;
    ow1_ = 0u;
    ew2_ = 0u;
    ow2_ = 0u;
}

namespace {

RUIDO_ALWAYS_INLINE void decimateStage1Inline(float s0L, float s0R, float s1L, float s1R,
                                              float* evenL, float* evenR, float* oddL, float* oddR,
                                              size_t& ew1, size_t& ow1, float& outL,
                                              float& outR) noexcept {
    constexpr size_t kPairs = 8u;
    constexpr size_t kOddPeriod = 16u;
    constexpr size_t kEvenMask = kPairs - 1u;
    constexpr size_t kOddMask = kOddPeriod - 1u;

    const size_t ew = ew1;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    evenL[ew] = s0L;
    evenR[ew] = s0R;
    const float s0DelL = evenL[nextEw];
    const float s0DelR = evenR[nextEw];
    ew1 = nextEw;

    const size_t ow = ow1;
    oddL[ow] = s1L;
    oddL[ow + kOddPeriod] = s1L;
    oddR[ow] = s1R;
    oddR[ow + kOddPeriod] = s1R;

    size_t base = (ow - (kPairs - 1u)) & kOddMask;
    if (base < kPairs) {
        base += kOddPeriod;
    }
    ow1 = (ow + 1u) & kOddMask;

    const float* rL = oddL + base;
    const float* lL = rL - 1;
    const float* rR = oddR + base;
    const float* lR = rR - 1;

    // Fully unrolled 8-pair symmetric FIR: zero loop/branch overhead, exact even/odd FP sum order
    float accL0 = 0.5f * s0DelL;
    float accL1 = 0.0f;
    float accR0 = 0.5f * s0DelR;
    float accR1 = 0.0f;

    accL0 += kStage1Taps[0] * (rL[0] + lL[0]);
    accL1 += kStage1Taps[1] * (rL[1] + lL[-1]);
    accR0 += kStage1Taps[0] * (rR[0] + lR[0]);
    accR1 += kStage1Taps[1] * (rR[1] + lR[-1]);

    accL0 += kStage1Taps[2] * (rL[2] + lL[-2]);
    accL1 += kStage1Taps[3] * (rL[3] + lL[-3]);
    accR0 += kStage1Taps[2] * (rR[2] + lR[-2]);
    accR1 += kStage1Taps[3] * (rR[3] + lR[-3]);

    accL0 += kStage1Taps[4] * (rL[4] + lL[-4]);
    accL1 += kStage1Taps[5] * (rL[5] + lL[-5]);
    accR0 += kStage1Taps[4] * (rR[4] + lR[-4]);
    accR1 += kStage1Taps[5] * (rR[5] + lR[-5]);

    accL0 += kStage1Taps[6] * (rL[6] + lL[-6]);
    accL1 += kStage1Taps[7] * (rL[7] + lL[-7]);
    accR0 += kStage1Taps[6] * (rR[6] + lR[-6]);
    accR1 += kStage1Taps[7] * (rR[7] + lR[-7]);

    outL = accL0 + accL1;
    outR = accR0 + accR1;
}

RUIDO_ALWAYS_INLINE StereoFrame decimateStage2Inline(float s0L, float s0R, float s1L, float s1R,
                                                     float* evenL, float* evenR, float* oddL,
                                                     float* oddR, size_t& ew2,
                                                     size_t& ow2) noexcept {
    constexpr size_t kPairs = 32u;
    constexpr size_t kOddPeriod = 64u;
    constexpr size_t kEvenMask = kPairs - 1u;
    constexpr size_t kOddMask = kOddPeriod - 1u;

    const size_t ew = ew2;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    evenL[ew] = s0L;
    evenR[ew] = s0R;
    const float s0DelL = evenL[nextEw];
    const float s0DelR = evenR[nextEw];
    ew2 = nextEw;

    const size_t ow = ow2;
    oddL[ow] = s1L;
    oddL[ow + kOddPeriod] = s1L;
    oddR[ow] = s1R;
    oddR[ow + kOddPeriod] = s1R;

    size_t base = (ow - (kPairs - 1u)) & kOddMask;
    if (base < kPairs) {
        base += kOddPeriod;
    }
    ow2 = (ow + 1u) & kOddMask;

    const float* rL = oddL + base;
    const float* lL = rL - 1;
    const float* rR = oddR + base;
    const float* lR = rR - 1;

#if !(defined(__SSE__) || defined(__ARM_NEON) || defined(__wasm__))
    const float* hp = kStage2Taps.data();
    float accL0 = 0.5f * s0DelL;
    float accL1 = 0.0f;
    float accR0 = 0.5f * s0DelR;
    float accR1 = 0.0f;

    // 4x unrolled (8 iterations instead of 16) with immediate offsets to cut pointer/branch ops 4x
    for (size_t i = 0u; i < kPairs; i += 4u) {
        const float h0 = hp[0];
        const float h1 = hp[1];
        accL0 += h0 * (rL[0] + lL[0]);
        accL1 += h1 * (rL[1] + lL[-1]);
        accR0 += h0 * (rR[0] + lR[0]);
        accR1 += h1 * (rR[1] + lR[-1]);

        const float h2 = hp[2];
        const float h3 = hp[3];
        accL0 += h2 * (rL[2] + lL[-2]);
        accL1 += h3 * (rL[3] + lL[-3]);
        accR0 += h2 * (rR[2] + lR[-2]);
        accR1 += h3 * (rR[3] + lR[-3]);

        hp += 4;
        rL += 4;
        lL -= 4;
        rR += 4;
        lR -= 4;
    }
    return {accL0 + accL1, accR0 + accR1};
#else
    float accL0 = 0.5f * s0DelL;
    float accL1 = 0.0f;
    float accL2 = 0.0f;
    float accL3 = 0.0f;
    float accR0 = 0.5f * s0DelR;
    float accR1 = 0.0f;
    float accR2 = 0.0f;
    float accR3 = 0.0f;

    for (size_t i = 0u; i < kPairs; i += 4u) {
        const ptrdiff_t i0 = static_cast<ptrdiff_t>(i);
        const ptrdiff_t i1 = i0 + 1;
        const ptrdiff_t i2 = i0 + 2;
        const ptrdiff_t i3 = i0 + 3;
        const float h0 = kStage2Taps[i];
        const float h1 = kStage2Taps[i + 1u];
        const float h2 = kStage2Taps[i + 2u];
        const float h3 = kStage2Taps[i + 3u];

        accL0 += h0 * (rL[i0] + lL[-i0]);
        accL1 += h1 * (rL[i1] + lL[-i1]);
        accL2 += h2 * (rL[i2] + lL[-i2]);
        accL3 += h3 * (rL[i3] + lL[-i3]);

        accR0 += h0 * (rR[i0] + lR[-i0]);
        accR1 += h1 * (rR[i1] + lR[-i1]);
        accR2 += h2 * (rR[i2] + lR[-i2]);
        accR3 += h3 * (rR[i3] + lR[-i3]);
    }
    return {(accL0 + accL1) + (accL2 + accL3), (accR0 + accR1) + (accR2 + accR3)};
#endif
}

} // namespace

StereoFrame StereoHalfBandDecimator4x::decimate2xStage1Stereo(StereoFrame s0,
                                                              StereoFrame s1) noexcept {
    float outL = 0.0f;
    float outR = 0.0f;
    decimateStage1Inline(s0.left, s0.right, s1.left, s1.right, left_.even1.data(),
                         right_.even1.data(), left_.odd1.data(), right_.odd1.data(), ew1_, ow1_,
                         outL, outR);
    return {outL, outR};
}

StereoFrame StereoHalfBandDecimator4x::decimate2xStage2Stereo(StereoFrame s0,
                                                              StereoFrame s1) noexcept {
    return decimateStage2Inline(s0.left, s0.right, s1.left, s1.right, left_.even2.data(),
                                right_.even2.data(), left_.odd2.data(), right_.odd2.data(), ew2_,
                                ow2_);
}

RUIDO_NOINLINE void StereoHalfBandDecimator4x::processSpan4x(const std::array<StereoFrame, 4>* in4x,
                                                             StereoFrame* out,
                                                             size_t count) noexcept {
    size_t ew1 = ew1_;
    size_t ow1 = ow1_;
    size_t ew2 = ew2_;
    size_t ow2 = ow2_;
    float* ev1L = left_.even1.data();
    float* ev1R = right_.even1.data();
    float* od1L = left_.odd1.data();
    float* od1R = right_.odd1.data();
    float* ev2L = left_.even2.data();
    float* ev2R = right_.even2.data();
    float* od2L = left_.odd2.data();
    float* od2R = right_.odd2.data();

    for (size_t k = 0u; k < count; ++k) {
        const auto& f = in4x[k];
        float m0L = 0.0f;
        float m0R = 0.0f;
        float m1L = 0.0f;
        float m1R = 0.0f;
        decimateStage1Inline(f[0].left, f[0].right, f[1].left, f[1].right, ev1L, ev1R, od1L, od1R,
                             ew1, ow1, m0L, m0R);
        decimateStage1Inline(f[2].left, f[2].right, f[3].left, f[3].right, ev1L, ev1R, od1L, od1R,
                             ew1, ow1, m1L, m1R);
        out[k] = decimateStage2Inline(m0L, m0R, m1L, m1R, ev2L, ev2R, od2L, od2R, ew2, ow2);
    }

    ew1_ = ew1;
    ow1_ = ow1;
    ew2_ = ew2;
    ow2_ = ow2;
}

StereoFrame StereoHalfBandDecimator4x::process4x(const std::array<StereoFrame, 4>& in4x) noexcept {
    StereoFrame out{};
    processSpan4x(&in4x, &out, 1u);
    return out;
}

} // namespace ruido
