#include "dsp/HalfBandDecimator.h"

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
}

StereoFrame StereoHalfBandDecimator4x::decimate2xStage1Stereo(ChannelState& chL, ChannelState& chR,
                                                              StereoFrame s0,
                                                              StereoFrame s1) noexcept {
    constexpr size_t kEvenMask = kStage1Pairs - 1u;    // 7
    constexpr size_t kOddMask = kStage1OddPeriod - 1u; // 15

    const size_t ew = chL.ew1;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    chL.even1[ew] = s0.left;
    chR.even1[ew] = s0.right;
    const float s0DelL = chL.even1[nextEw];
    const float s0DelR = chR.even1[nextEw];
    chL.ew1 = nextEw;
    chR.ew1 = nextEw;

    const size_t ow = chL.ow1;
    chL.odd1[ow] = s1.left;
    chL.odd1[ow + kStage1OddPeriod] = s1.left;
    chR.odd1[ow] = s1.right;
    chR.odd1[ow + kStage1OddPeriod] = s1.right;

    size_t base = (ow - (kStage1Pairs - 1u)) & kOddMask;
    if (base < kStage1Pairs) {
        base += kStage1OddPeriod;
    }
    const size_t nextOw = (ow + 1u) & kOddMask;
    chL.ow1 = nextOw;
    chR.ow1 = nextOw;

    const float* rL = &chL.odd1[base];
    const float* lL = rL - 1;
    const float* rR = &chR.odd1[base];
    const float* lR = rR - 1;

    // 2 unrolled quads (8 tap pairs) break serial FPU fadd latency across L & R
    float accL0 = 0.5f * s0DelL;
    float accL1 = 0.0f;
    float accR0 = 0.5f * s0DelR;
    float accR1 = 0.0f;
    for (size_t i = 0u; i < kStage1Pairs; i += 2u) {
        const ptrdiff_t i0 = static_cast<ptrdiff_t>(i);
        const ptrdiff_t i1 = i0 + 1;
        const float h0 = kStage1Taps[i];
        const float h1 = kStage1Taps[i + 1u];
        accL0 += h0 * (rL[i0] + lL[-i0]);
        accL1 += h1 * (rL[i1] + lL[-i1]);
        accR0 += h0 * (rR[i0] + lR[-i0]);
        accR1 += h1 * (rR[i1] + lR[-i1]);
    }
    return {accL0 + accL1, accR0 + accR1};
}

StereoFrame StereoHalfBandDecimator4x::decimate2xStage2Stereo(ChannelState& chL, ChannelState& chR,
                                                              StereoFrame s0,
                                                              StereoFrame s1) noexcept {
    constexpr size_t kEvenMask = kStage2Pairs - 1u;    // 31
    constexpr size_t kOddMask = kStage2OddPeriod - 1u; // 63

    const size_t ew = chL.ew2;
    const size_t nextEw = (ew + 1u) & kEvenMask;
    chL.even2[ew] = s0.left;
    chR.even2[ew] = s0.right;
    const float s0DelL = chL.even2[nextEw];
    const float s0DelR = chR.even2[nextEw];
    chL.ew2 = nextEw;
    chR.ew2 = nextEw;

    const size_t ow = chL.ow2;
    chL.odd2[ow] = s1.left;
    chL.odd2[ow + kStage2OddPeriod] = s1.left;
    chR.odd2[ow] = s1.right;
    chR.odd2[ow + kStage2OddPeriod] = s1.right;

    size_t base = (ow - (kStage2Pairs - 1u)) & kOddMask;
    if (base < kStage2Pairs) {
        base += kStage2OddPeriod;
    }
    const size_t nextOw = (ow + 1u) & kOddMask;
    chL.ow2 = nextOw;
    chR.ow2 = nextOw;

    const float* rL = &chL.odd2[base];
    const float* lL = rL - 1;
    const float* rR = &chR.odd2[base];
    const float* lR = rR - 1;

    // 4 independent accumulators per channel break FPU fadd latency without -ffast-math
    float accL0 = 0.5f * s0DelL;
    float accL1 = 0.0f;
    float accL2 = 0.0f;
    float accL3 = 0.0f;
    float accR0 = 0.5f * s0DelR;
    float accR1 = 0.0f;
    float accR2 = 0.0f;
    float accR3 = 0.0f;

    for (size_t i = 0u; i < kStage2Pairs; i += 4u) {
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
}

StereoFrame StereoHalfBandDecimator4x::process4x(const std::array<StereoFrame, 4>& in4x) noexcept {
    const StereoFrame mid0 = decimate2xStage1Stereo(left_, right_, in4x[0], in4x[1]);
    const StereoFrame mid1 = decimate2xStage1Stereo(left_, right_, in4x[2], in4x[3]);
    return decimate2xStage2Stereo(left_, right_, mid0, mid1);
}

} // namespace ruido
