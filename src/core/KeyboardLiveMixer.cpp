#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"

namespace ruido {

namespace {

inline size_t findNearestJustRatioIdx(float rawRatio) noexcept {
    const float clamped = clampf(rawRatio, 0.20f, 4.50f);
    const float logTarget = fastLog2(clamped);
    size_t bestIdx = 8u; // 1/1 default
    float bestDist = 99.0f;
    for (size_t i = 0u; i < kNumJustRatios; ++i) {
        const float d = absf(logTarget - fastLog2(kJustRatioTable[i].ratio));
        if (d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }
    return bestIdx;
}

// Continuous 0.0 -> -2.0 Oct sub-drop curve across [0, 1] (-1.0 Oct inflection at 50%).
inline float evalSubDropOctaves(float p) noexcept {
    const float x = clampf(p, 0.0f, 1.0f);
    if (x <= 0.001f) {
        return 0.0f;
    }
    const float t = 2.0f * x - 1.0f;
    return -1.0f - 0.65f * t - 0.35f * t * t * t;
}

// Continuous 0.0 -> +2.0 Oct harmonic rise across [0, 1] (+1.0 Oct at 50%, +2.0 Oct at 100%).
inline float evalOctaveRise(float p) noexcept {
    return -evalSubDropOctaves(p);
}

} // namespace

void onLiveMixerKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                         KeyboardLatchState& latch) noexcept {
    onConsoleSoloMuteKeyPress(isBlack, keyIdx, config, latch);
}

void evaluateLiveMixerModel(const KeyboardInputView& in, EngineControlState& state,
                            HudState& hud) noexcept {
    KeyboardModState mod{};
    PunchTracker punch{};

    // 1. OP1..OP4 (SOLO 1-4) & White Keys 0..3 (MUTE 1-4): Momentary or Latching Console Switches
    evaluateConsoleSoloMute(in, mod, punch);

    // 2. OP5: Drive -> progressive 4.5x MS-20 overdrive + starved-rail sync + Korg35 scream
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op5)]) {
        const size_t idx = static_cast<size_t>(BlackKeyId::Op5);
        mod.overloadSurge = in.blackPressure[idx];
        punch.note(in.blackPressSeq[idx], KeyPunchKind::OverloadSurge, 0u, mod.overloadSurge);
    }

    // 3. OP6: Sub Octave (continuous 0.0 -> -2.0 Oct dive with -1.0 Oct inflection at 50%)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op6)]) {
        const size_t idx = static_cast<size_t>(BlackKeyId::Op6);
        const float oct = evalSubDropOctaves(in.blackPressure[idx]);
        const float mul = fastExp2(oct);
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePitchMul[v] *= mul;
        }
        punch.note(in.blackPressSeq[idx], KeyPunchKind::SubDrop, 0u, oct);
    }

    // 4. PIT: Just-Intonation Ratio Lock + progressive 1/f thermal drift damping
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Pit)]) {
        const size_t idx = static_cast<size_t>(BlackKeyId::Pit);
        const float p = clampf(in.blackPressure[idx], 0.0f, 1.0f);
        const float lockAmt = p * (1.25f - 0.25f * p);
        mod.ratioLockAmount = lockAmt;
        const float anchorHz =
            clampf(state.voices[0].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
        size_t repRatioIdx = 12u; // 3/2 fallback
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            const float vHz = clampf(state.voices[v].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
            const size_t rIdx = findNearestJustRatioIdx(vHz / anchorHz);
            if (v == 1u) {
                repRatioIdx = rIdx;
            }
            const float targetHz = anchorHz * kJustRatioTable[rIdx].ratio;
            const float logDelta = fastLog2(targetHz / vHz);
            mod.voicePitchMul[v] *= fastExp2(lockAmt * logDelta);
        }
        punch.note(in.blackPressSeq[idx], KeyPunchKind::RatioMagnet,
                   static_cast<uint8_t>(repRatioIdx), p);
    }

    // 5. GLO key: Low Pass sweep (28 kHz -> 31 Hz with monotonically rising Korg35 resonance)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Glo)]) {
        const size_t idx = static_cast<size_t>(BlackKeyId::Glo);
        const float p = in.blackPressure[idx];
        mod.dubLpfDrop = p;
        const float cutoffHz = 28000.0f * fastExp2(-9.8f * p);
        punch.note(in.blackPressSeq[idx], KeyPunchKind::DubLpfDrop, 0u, cutoffHz);
    }

    // 6. MONO / POLY keys: 4-Oscillator Analog Unison Stack vs. Detuned + Schmitt PWM Swarm
    const size_t monoIdx = static_cast<size_t>(BlackKeyId::Mono);
    const size_t polyIdx = static_cast<size_t>(BlackKeyId::Poly);
    const bool monoOn = in.blackHeld[monoIdx];
    const bool polyOn = in.blackHeld[polyIdx];
    if (monoOn || polyOn) {
        const bool usePoly =
            polyOn && (!monoOn || in.blackPressSeq[polyIdx] >= in.blackPressSeq[monoIdx]);
        if (usePoly) {
            const float p = clampf(in.blackPressure[polyIdx], 0.0f, 1.0f);
            const float s = p * (0.75f + 0.25f * p);
            constexpr std::array<float, 4> kSwarmOct = {-0.045f, 0.048f, -0.095f, 0.105f};
            constexpr std::array<float, 4> kSwarmPw = {-0.18f, 0.16f, -0.26f, 0.24f};
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                mod.voicePitchMul[v] *= fastExp2(kSwarmOct[v] * s);
                mod.voicePwDelta[v] += kSwarmPw[v] * s;
            }
            mod.spatialFocus = p;
            punch.note(in.blackPressSeq[polyIdx], KeyPunchKind::PolySpread, 0u, p);
        } else {
            const float p = clampf(in.blackPressure[monoIdx], 0.0f, 1.0f);
            const float u = p * (1.25f - 0.25f * p);
            const float anchorHz =
                clampf(state.voices[0].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
            // Tight ~+-2 cent analog VCO unison offsets prevent sterile digital square cancellation
            constexpr std::array<float, 4> kUnisonOct = {0.0f, -0.0015f, 0.0016f, -0.0026f};
            for (size_t v = 1u; v < kNumOscillators; ++v) {
                const float vHz =
                    clampf(state.voices[v].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
                const float logDelta = fastLog2(anchorHz / vHz) + kUnisonOct[v];
                mod.voicePitchMul[v] *= fastExp2(u * logDelta);
            }
            if (0.75f * u > mod.ratioLockAmount) {
                mod.ratioLockAmount = 0.75f * u;
            }
            mod.spatialFocus = -p;
            punch.note(in.blackPressSeq[monoIdx], KeyPunchKind::MonoFocus, 0u, p);
        }
    }

    // 7. 11th Tall Key: Freeze (8 LFOs + Drift + Scope)
    evaluateUniversalFreeze(in, mod, hud, punch);

    // 8. Bottom Effect & Scene Groups (FILTER 5-8 | OCTAVE 9-12 | SCENE 13-16)
    std::array<float, kNumOscillators> voiceFormant{0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        if (in.whiteHeld[4u + v]) {
            const float p = in.whitePressure[4u + v];
            voiceFormant[v] = p;
            punch.note(in.whitePressSeq[4u + v], KeyPunchKind::VoiceFormant, v, p);
        }
        if (in.whiteHeld[8u + v]) {
            const float p = in.whitePressure[8u + v];
            const float oct = evalOctaveRise(p);
            mod.voicePitchMul[v] *= fastExp2(oct);
            const float sirenBite = 0.72f * p * p;
            if (sirenBite > voiceFormant[v]) {
                voiceFormant[v] = sirenBite;
            }
            punch.note(in.whitePressSeq[8u + v], KeyPunchKind::VoiceSiren, v, oct);
        }
        if (in.whiteHeld[12u + v] && !hud.saveArmed) {
            punch.note(in.whitePressSeq[12u + v], KeyPunchKind::SnapshotRecall, v,
                       in.whitePressure[12u + v]);
        }
        state.voicePressure[v] = voiceFormant[v];
    }

    punch.commitTo(hud);
    state.keyMod = mod;
}

void choreographLiveMixerLeds(const KeyboardInputView& in, float animPhase, float subBreathWave01,
                              const EngineControlState& state, const HudState& hud,
                              uint8_t stutMask, float rhythmPhase01, LedState& outLeds) noexcept {
    (void)rhythmPhase01;
    // 1. OP5 Drive: Dual-edge plasma flood + top VU bar + physical RED REC clip beacon!
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op5)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op5)];
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op5)] = 1.0f;
        const float reach = p * 8.4f;
        const float shimmer =
            (p > 0.55f) ? (0.80f + 0.20f * evalAnalogSine(wrapLfoPhase01(animPhase * 13.0f)))
                        : 1.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float edgeDist = (i < 8u) ? static_cast<float>(i) : static_cast<float>(15u - i);
            const float fill = clampf(reach - edgeDist, 0.0f, 1.0f);
            const float flood = fill * (0.45f + 0.55f * p) * shimmer;
            if (flood > outLeds.whiteKeyLeds[i]) {
                outLeds.whiteKeyLeds[i] = flood;
            }
        }
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, p * shimmer);
        maxPanelBtnLed(outLeds, ButtonId::OctPlus,
                       clampf((p - 0.15f) * 1.2f, 0.0f, 1.0f) * shimmer);
        for (size_t c = 0u; c < 6u; ++c) {
            const float bar = clampf(p * 6.4f - static_cast<float>(c), 0.0f, 1.0f) * shimmer;
            maxPanelBtnLed(outLeds, kPanelGridTop[c], bar);
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], bar * 0.88f);
            }
        }
        if (p > 0.45f) {
            maxPanelBtnLed(outLeds, ButtonId::Rec,
                           clampf((p - 0.45f) * (1.0f / 0.55f), 0.0f, 1.0f) * shimmer);
        }
    }

    // 2. OP6 Sub Octave: OCT- blazes 100%, top row dims, bottom switches & keys throb at 2 Hz
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op6)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op6)];
        const float subThrob =
            0.55f + 0.45f * (0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(animPhase * 2.0f))));
        for (size_t i = 4u; i < kNumBlackKeys; ++i) {
            outLeds.blackKeyLeds[i] *= 0.25f;
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op6)] = 1.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            outLeds.whiteKeyLeds[i] = clampf(0.32f + 0.68f * p * subThrob, 0.0f, 1.0f);
        }
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.55f + 0.45f * p);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.0f);
        for (size_t c = 0u; c < 6u; ++c) {
            outLeds.buttonLeds[static_cast<size_t>(kPanelGridTop[c])] *= 0.20f;
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], (0.30f + 0.70f * p) * subThrob);
            }
        }
    }

    // 3. PIT Ratio Lock: Beating switch columns converge into steady harmonic lock as p -> 1
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Pit)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Pit)];
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Pit)] = 1.0f;
        const float waveA = 0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(animPhase * 4.0f)));
        const float waveB = 1.0f - waveA;
        for (size_t b = 1u; b < 4u; ++b) {
            outLeds.whiteKeyLeds[b * 4u] = clampf(0.55f + 0.45f * p, 0.0f, 1.0f);
            for (size_t v = 1u; v < 4u; ++v) {
                const float lockGlow = p * (0.45f + 0.45f * (1.0f - p) * waveA + 0.45f * p);
                if (lockGlow > outLeds.whiteKeyLeds[b * 4u + v]) {
                    outLeds.whiteKeyLeds[b * 4u + v] = lockGlow;
                }
            }
        }
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, 0.35f + 0.55f * p);
        maxPanelBtnLed(outLeds, ButtonId::OctPlus, 0.35f + 0.55f * p);
        for (size_t c = 0u; c < 6u; ++c) {
            const float beat = ((c & 1u) == 0u) ? waveA : waveB;
            const float colGlow = (0.25f + 0.65f * beat) * (1.0f - p) + 0.82f * p;
            maxPanelBtnLed(outLeds, kPanelGridTop[c], colGlow * p);
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], colGlow * p * 0.85f);
            }
        }
    }

    // 4. GLO Low Pass: Right-to-left closing passband curtain across BOTH top switches & keys
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Glo)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Glo)];
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Glo)] = 1.0f;
        const float crestIdx = (1.0f - p) * 15.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float fi = static_cast<float>(i);
            const float crest = clampf(1.0f - absf(fi - crestIdx) * 0.55f, 0.0f, 1.0f);
            const float pass = (fi <= crestIdx) ? 0.34f : 0.02f;
            outLeds.whiteKeyLeds[i] = clampf(pass + 0.66f * crest, 0.0f, 1.0f);
        }
        const float pCrest = (1.0f - p) * 7.0f;
        for (size_t slot = 0u; slot < 8u; ++slot) {
            const float fs = static_cast<float>(slot);
            const float crest = clampf(1.0f - absf(fs - pCrest) * 0.75f, 0.0f, 1.0f);
            const float lvl = clampf(((fs <= pCrest) ? 0.30f : 0.0f) + 0.70f * crest, 0.0f, 1.0f);
            if (slot == 0u) {
                setPanelBtnLed(outLeds, ButtonId::OctMinus, lvl);
            } else if (slot == 1u) {
                setPanelBtnLed(outLeds, ButtonId::OctPlus, lvl);
            } else {
                const size_t c = slot - 2u;
                setPanelBtnLed(outLeds, kPanelGridTop[c], lvl);
                if (kPanelGridBot[c] != ButtonId::PlayStop) {
                    setPanelBtnLed(outLeds, kPanelGridBot[c], lvl * ((c == 5u) ? 0.60f : 0.90f));
                }
            }
        }
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.45f + 0.55f * p);
    }

    // 5. MONO Unison Stack vs. POLY Detune/PWM Swarm across BOTH rows!
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Mono)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Mono)];
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Mono)] = 1.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float tgt = (i >= 6u && i <= 9u) ? 0.95f : 0.03f;
            outLeds.whiteKeyLeds[i] += (tgt - outLeds.whiteKeyLeds[i]) * p;
        }
        scalePanelGrid(outLeds, 1.0f - 0.92f * p);
        maxPanelBtnLed(outLeds, ButtonId::Env, 0.95f * p);
        maxPanelBtnLed(outLeds, ButtonId::Lfo, 0.95f * p);
        maxPanelBtnLed(outLeds, ButtonId::Arp, 0.95f * p);
        maxPanelBtnLed(outLeds, ButtonId::Seq, 0.95f * p);
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Poly)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Poly)];
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Poly)] = 1.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float tgt = (i <= 2u || i >= 13u) ? 0.95f : 0.04f;
            outLeds.whiteKeyLeds[i] += (tgt - outLeds.whiteKeyLeds[i]) * p;
        }
        scalePanelGrid(outLeds, 1.0f - 0.88f * p);
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, 0.98f * p);
        maxPanelBtnLed(outLeds, ButtonId::OctPlus, 0.90f * p);
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.85f * p);
        maxPanelBtnLed(outLeds, ButtonId::Home, 0.85f * p);
        maxPanelBtnLed(outLeds, ButtonId::Edit, 0.90f * p);
        maxPanelBtnLed(outLeds, ButtonId::GloRight, 0.98f * p);
    }

    // 6. Bottom Effect & Snapshot Groups (FILTER 5-8 | OCTAVE 9-12 | SNAP 13-16)
    float maxOctP = 0.0f;
    float maxSnapP = 0.0f;
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (in.whiteHeld[4u + v]) {
            const float p = in.whitePressure[4u + v];
            outLeds.whiteKeyLeds[4u + v] = 0.56f + 0.44f * p;
            maxPanelBtnLed(outLeds, ButtonId::Fx, 0.48f + 0.52f * p);
        }
        if (in.whiteHeld[8u + v]) {
            const float p = in.whitePressure[8u + v];
            outLeds.whiteKeyLeds[8u + v] = 0.48f + 0.52f * p;
            if (p > maxOctP) {
                maxOctP = p;
            }
        }
        if (in.whiteHeld[12u + v]) {
            const bool occ = (hud.snapshotOccupiedMask & (1u << v)) != 0u;
            if (occ) {
                const float p = in.whitePressure[12u + v];
                outLeds.whiteKeyLeds[12u + v] = 1.0f;
                if (p > maxSnapP) {
                    maxSnapP = p;
                }
            } else {
                outLeds.whiteKeyLeds[12u + v] = 0.15f;
            }
        }
    }
    if (maxOctP > 0.001f) {
        const float sirenPos = wrapLfoPhase01(animPhase * (3.5f + 6.5f * maxOctP)) * 6.0f;
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.55f + 0.45f * maxOctP);
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.0f);
        for (size_t c = 0u; c < 6u; ++c) {
            float dist = absf(static_cast<float>(c) - sirenPos);
            if (dist > 3.0f) {
                dist = 6.0f - dist;
            }
            const float beam = clampf(1.0f - dist * 0.65f, 0.0f, 1.0f);
            maxPanelBtnLed(outLeds, kPanelGridTop[c], beam * (0.45f + 0.55f * maxOctP));
        }
    }
    if (maxSnapP > 0.001f) {
        maxPanelBtnLed(outLeds, ButtonId::Save, 0.45f + 0.55f * hud.morphPosition);
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const float w = clampf(hud.snapshotWeights[s], 0.0f, 1.0f);
            if (w > 0.01f) {
                maxPanelBtnLed(outLeds, kPanelGridTop[1u + s], 0.30f + 0.65f * w);
            }
        }
    }

    // 7. 11th Key Stasis Freeze: Crystalline still-frame across ALL 42 device LEDs
    applyFreezeStillFrameLeds(in, outLeds);

    // 8. Authoritative Console SOLO (OP 1-4) & MUTE (Keys 1-4) circuit state via VoiceCircuit.h
    applyAuthoritativeConsoleLeds(state, stutMask, subBreathWave01, outLeds);
}

} // namespace ruido
