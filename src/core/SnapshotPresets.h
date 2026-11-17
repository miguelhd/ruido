#pragma once

#include "core/SnapshotBank.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ruido {

struct VoicePresetSpec {
    float freqHz;
    uint16_t hpHz;
    uint16_t lpHz;
    uint16_t lfoAMilliHz;
    uint16_t lfoBMilliHz;
    uint8_t dutyPct;
    uint8_t driftPct;
    uint8_t hpResPct;
    uint8_t lpResPct;
    uint8_t ampPct;
    int8_t panPct;
    uint8_t lfoAShapePct;
    uint8_t lfoADepthPct;
    uint8_t lfoBShapePct;
    uint8_t lfoBDepthPct;
    uint8_t targetsPacked;
    uint8_t reserved;
};

struct FactorySceneSpec {
    uint8_t couplingPct;
    std::array<VoicePresetSpec, kNumOscillators> voices;
};

constexpr uint8_t packLfoTargets(LfoTarget a, LfoTarget b) noexcept {
    return static_cast<uint8_t>((static_cast<uint8_t>(a) & 0x0Fu) |
                                ((static_cast<uint8_t>(b) & 0x0Fu) << 4u));
}

inline constexpr std::array<FactorySceneSpec, kNumSnapshots> kFactoryScenes = {{
    // SCENE 1 (First Load): "Luminous Horizon" — Fast, liquid 1:2:3:5 Cmaj overtone cinema
    // (C2 65.41 · C3 131.06 · G3 196.52 · E4 327.36 Hz). Smooth 0.52..1.92 Hz Sine/Triangle LFOs
    // sweep warm MS-20 Rev2 resonant LPFs, CS-80 PWM, stereo orbit, and shimmer swells.
    {20u,
     {{
         {65.41f, 2u, 1060u, 780u, 380u, 40u, 10u, 0u, 46u, 68u, -20, 0u, 32u, 0u, 20u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {131.06f, 2u, 1860u, 1260u, 740u, 60u, 10u, 0u, 48u, 64u, 44, 25u, 34u, 0u, 36u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {196.52f, 2u, 28000u, 1580u, 920u, 34u, 11u, 0u, 0u, 44u, -66, 0u, 46u, 0u, 38u,
          packLfoTargets(LfoTarget::PulseWidth, LfoTarget::Pan), 0u},
         {327.36f, 2u, 28000u, 1880u, 1140u, 66u, 12u, 0u, 0u, 30u, 70, 0u, 46u, 0u, 30u,
          packLfoTargets(LfoTarget::PulseWidth, LfoTarget::Amplitude), 0u},
     }}},

    // SCENE 2 (1/3 LIVE MIXER Showcase): "Velvet Poly-Brass" — Warm Fmaj9 CS-80 pad
    // (F2 87.31 · C3 131.35 · G3 196.60 · A3 220.65 Hz). Tucked LPFs bloom under FILTER 1-4,
    // mid-register roots roar under SUB OCTAVE + DRIVE, and 44-56% duties pinch sharply on DUTY.
    {24u,
     {{
         {87.31f, 2u, 680u, 440u, 710u, 44u, 18u, 0u, 44u, 70u, -18, 0u, 30u, 25u, 26u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {131.35f, 2u, 1060u, 580u, 410u, 56u, 22u, 0u, 48u, 64u, 42, 0u, 32u, 0u, 28u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {196.60f, 2u, 1580u, 740u, 490u, 46u, 24u, 0u, 42u, 56u, -68, 25u, 30u, 0u, 32u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::Pan), 0u},
         {220.65f, 2u, 28000u, 910u, 620u, 54u, 26u, 0u, 0u, 36u, 72, 0u, 36u, 25u, 26u,
          packLfoTargets(LfoTarget::PulseWidth, LfoTarget::Amplitude), 0u},
     }}},

    // SCENE 3 (2/3 CIRCUIT LAB Showcase): "Harmonic Sync & Wavefold Rig" — Tight 1:2:3:4 series
    // (A1 55.00 · A2 110.32 · E3 165.38 · A3 220.48 Hz). Low drift + 55 Hz anchor make HARD SYNC,
    // FOLD 5-8, and RING MOD razor-articulate, while 0.75..3 Hz LFOs rocket into FM under TURBO.
    {38u,
     {{
         {55.00f, 2u, 1080u, 1500u, 750u, 40u, 5u, 0u, 42u, 72u, -16, 25u, 34u, 0u, 26u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {110.32f, 2u, 2200u, 2250u, 1125u, 60u, 6u, 0u, 46u, 62u, 44, 0u, 32u, 25u, 28u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
         {165.38f, 2u, 28000u, 3000u, 1500u, 34u, 7u, 0u, 0u, 48u, -64, 0u, 36u, 0u, 34u,
          packLfoTargets(LfoTarget::PulseWidth, LfoTarget::Pan), 0u},
         {220.48f, 2u, 28000u, 1750u, 875u, 66u, 8u, 0u, 0u, 42u, 68, 25u, 18u, 0u, 36u,
          packLfoTargets(LfoTarget::Pitch, LfoTarget::PulseWidth), 0u},
     }}},

    // SCENE 4 (3/3 RHYTHM LAB Showcase): "Analog Drum + 303 Acid Kit" (Steady levels)
    // OSC 1 = 49 Hz G1 Sub Kick (115 Hz LPF) | OSC 2 = 196 Hz Electro Snare (185 Hz HPF + FM
    // rattle)
    // OSC 3 = 2350 Hz Metallic Hi-Hat (4.6 kHz HPF + cross-FM) | OSC 4 = 98 Hz G2 Squelchy Acid
    // Bass
    {16u,
     {{
         {49.00f, 2u, 115u, 2000u, 1000u, 46u, 4u, 0u, 52u, 74u, 0, 50u, 8u, 50u, 18u,
          packLfoTargets(LfoTarget::Pitch, LfoTarget::LpCutoff), 0u},
         {196.00f, 185u, 28000u, 43500u, 37100u, 24u, 28u, 50u, 0u, 48u, -38, 75u, 56u, 50u, 88u,
          packLfoTargets(LfoTarget::Pitch, LfoTarget::LfoARate), 0u},
         {2350.00f, 4600u, 28000u, 44200u, 31300u, 8u, 34u, 46u, 0u, 38u, 64, 75u, 72u, 25u, 92u,
          packLfoTargets(LfoTarget::Pitch, LfoTarget::LfoARate), 0u},
         {98.00f, 2u, 460u, 1000u, 1500u, 18u, 10u, 0u, 58u, 60u, 28, 50u, 46u, 0u, 34u,
          packLfoTargets(LfoTarget::LpCutoff, LfoTarget::PulseWidth), 0u},
     }}},
}};

} // namespace ruido
