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
    prevMono_ = false;
}

namespace {

RUIDO_ALWAYS_INLINE float decimateStage1Mono(float s0, float s1, float* __restrict even,
                                             float* __restrict odd, size_t& ew1,
                                             size_t& ow1) noexcept {
    constexpr size_t kPairs = 8u;
    constexpr size_t kOddPeriod = 16u;
    constexpr size_t kEvenMask = kPairs - 1u;
    constexpr size_t kOddMask = kOddPeriod - 1u;

    const size_t ew = ew1;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    even[ew] = s0;
    const float s0Del = even[nextEw];
    ew1 = nextEw;

    const size_t ow = ow1;
    odd[ow] = s1;
    odd[ow + kOddPeriod] = s1;

    const size_t nextOw = (ow + 1u) & kOddMask;
    ow1 = nextOw;

    const float* r = odd + (nextOw + kPairs);
    const float* l = r - 1;

    float acc0 = 0.5f * s0Del;
    float acc1 = 0.0f;
    acc0 += kStage1Taps[0] * (r[0] + l[0]);
    acc1 += kStage1Taps[1] * (r[1] + l[-1]);
    acc0 += kStage1Taps[2] * (r[2] + l[-2]);
    acc1 += kStage1Taps[3] * (r[3] + l[-3]);
    acc0 += kStage1Taps[4] * (r[4] + l[-4]);
    acc1 += kStage1Taps[5] * (r[5] + l[-5]);
    acc0 += kStage1Taps[6] * (r[6] + l[-6]);
    acc1 += kStage1Taps[7] * (r[7] + l[-7]);
    return acc0 + acc1;
}

RUIDO_ALWAYS_INLINE float decimateStage2Mono(float s0, float s1, float* __restrict even,
                                             float* __restrict odd, size_t& ew2,
                                             size_t& ow2) noexcept {
    constexpr size_t kPairs = 32u;
    constexpr size_t kOddPeriod = 64u;
    constexpr size_t kEvenMask = kPairs - 1u;
    constexpr size_t kOddMask = kOddPeriod - 1u;

    const size_t ew = ew2;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    even[ew] = s0;
    const float s0Del = even[nextEw];
    ew2 = nextEw;

    const size_t ow = ow2;
    odd[ow] = s1;
    odd[ow + kOddPeriod] = s1;

    const size_t nextOw = (ow + 1u) & kOddMask;
    ow2 = nextOw;

    const float* r = odd + (nextOw + kPairs);
    const float* l = r - 1;

#if defined(RUIDO_TARGET_PI32V2) || defined(__Pi32__) || defined(__pi32v2__)
    // 95-tap (P = 24, Kaiser beta = 8.2, > 81.6 dB FIR stopband, 0.004 dB ripple, exact 0.0 dB DC
    // gain & 31-sample Stage-2 group delay) matched to FM-1's 16-bit TPDF-dithered DAC ceiling.
    static constexpr std::array<float, 24> kStage2Pi32Taps = {
        3.177867234e-01f, -1.045240462e-01f, 6.105801463e-02f, -4.188959301e-02f,
        3.086609952e-02f, -2.359173633e-02f, 1.838186011e-02f, -1.445362531e-02f,
        1.139373332e-02f, -8.962356485e-03f, 7.009196561e-03f, -5.433501676e-03f,
        4.163460806e-03f, -3.144969232e-03f, 2.335251775e-03f, -1.699154498e-03f,
        1.206960646e-03f, -8.330997080e-04f, 5.553860101e-04f, -3.545652144e-04f,
        2.140324650e-04f, -1.196367084e-04f, 5.951775165e-05f, -2.394500189e-05f,
    };
    (void)kStage2Taps;
    const float* hp = kStage2Pi32Taps.data();
    float acc0 = 0.5f * s0Del;
    float acc1 = 0.0f;
    for (size_t i = 0u; i < 24u; i += 4u) {
        acc0 += hp[0] * (r[0] + l[0]);
        acc1 += hp[1] * (r[1] + l[-1]);
        acc0 += hp[2] * (r[2] + l[-2]);
        acc1 += hp[3] * (r[3] + l[-3]);
        hp += 4;
        r += 4;
        l -= 4;
    }
    return acc0 + acc1;
#elif !(defined(__SSE__) || defined(__ARM_NEON) || defined(__wasm__))
    const float* hp = kStage2Taps.data();
    float acc0 = 0.5f * s0Del;
    float acc1 = 0.0f;
    for (size_t i = 0u; i < kPairs; i += 4u) {
        acc0 += hp[0] * (r[0] + l[0]);
        acc1 += hp[1] * (r[1] + l[-1]);
        acc0 += hp[2] * (r[2] + l[-2]);
        acc1 += hp[3] * (r[3] + l[-3]);
        hp += 4;
        r += 4;
        l -= 4;
    }
    return acc0 + acc1;
#else
    float acc0 = 0.5f * s0Del;
    float acc1 = 0.0f;
    float acc2 = 0.0f;
    float acc3 = 0.0f;
    for (size_t i = 0u; i < kPairs; i += 4u) {
        const ptrdiff_t i0 = static_cast<ptrdiff_t>(i);
        acc0 += kStage2Taps[i] * (r[i0] + l[-i0]);
        acc1 += kStage2Taps[i + 1u] * (r[i0 + 1] + l[-i0 - 1]);
        acc2 += kStage2Taps[i + 2u] * (r[i0 + 2] + l[-i0 - 2]);
        acc3 += kStage2Taps[i + 3u] * (r[i0 + 3] + l[-i0 - 3]);
    }
    return (acc0 + acc1) + (acc2 + acc3);
#endif
}

RUIDO_NOINLINE void decimateChannelSpan(float* __restrict ev1, float* __restrict od1,
                                        float* __restrict ev2, float* __restrict od2, size_t& ew1Io,
                                        size_t& ow1Io, size_t& ew2Io, size_t& ow2Io,
                                        const float* inInterleaved, float* outInterleaved,
                                        size_t count) noexcept {
    size_t ew1 = ew1Io;
    size_t ow1 = ow1Io;
    size_t ew2 = ew2Io;
    size_t ow2 = ow2Io;
    const float* inP = inInterleaved;
    float* outP = outInterleaved;
    for (size_t k = 0u; k < count; ++k) {
        const float x0 = inP[0];
        const float x1 = inP[2];
        const float x2 = inP[4];
        const float x3 = inP[6];
        inP += 8;
        const float m0 = decimateStage1Mono(x0, x1, ev1, od1, ew1, ow1);
        const float m1 = decimateStage1Mono(x2, x3, ev1, od1, ew1, ow1);
        *outP = decimateStage2Mono(m0, m1, ev2, od2, ew2, ow2);
        outP += 2;
    }
    ew1Io = ew1;
    ow1Io = ow1;
    ew2Io = ew2;
    ow2Io = ow2;
}

} // namespace

RUIDO_NOINLINE void StereoHalfBandDecimator4x::processSpan4x(const std::array<StereoFrame, 4>* in4x,
                                                             StereoFrame* out, size_t count,
                                                             bool exactMono) noexcept {
    if (count == 0u) {
        return;
    }
    if (exactMono) {
        decimateChannelSpan(left_.even1.data(), left_.odd1.data(), left_.even2.data(),
                            left_.odd2.data(), ew1_, ow1_, ew2_, ow2_, &in4x[0][0].left,
                            &out[0].left, count);
        for (size_t k = 0u; k < count; ++k) {
            out[k].right = out[k].left;
        }
        prevMono_ = true;
        return;
    }

    if (prevMono_) {
        right_ = left_;
        prevMono_ = false;
    }
    size_t ew1R = ew1_;
    size_t ow1R = ow1_;
    size_t ew2R = ew2_;
    size_t ow2R = ow2_;
    decimateChannelSpan(left_.even1.data(), left_.odd1.data(), left_.even2.data(),
                        left_.odd2.data(), ew1_, ow1_, ew2_, ow2_, &in4x[0][0].left, &out[0].left,
                        count);
    decimateChannelSpan(right_.even1.data(), right_.odd1.data(), right_.even2.data(),
                        right_.odd2.data(), ew1R, ow1R, ew2R, ow2R, &in4x[0][0].right,
                        &out[0].right, count);
}

StereoFrame StereoHalfBandDecimator4x::process4x(const std::array<StereoFrame, 4>& in4x) noexcept {
    StereoFrame out{};
    processSpan4x(&in4x, &out, 1u);
    return out;
}

} // namespace ruido
