#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"

namespace ruido {

namespace {

inline size_t findNearestJustRatioIdx(float logTarget) noexcept {
    const float clampedLog = clampf(logTarget, -2.3219281f, 2.1699250f);
    size_t bestIdx = 8u; // 1/1 default
    float bestDist = 99.0f;
    for (size_t i = 0u; i < kNumJustRatios; ++i) {
        const float d = absf(clampedLog - kJustRatioTable[i].log2Ratio);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }
    return bestIdx;
}

inline float evalSubDropOctaves(float p) noexcept {
    const float x = clampf(p, 0.0f, 1.0f);
    if (x <= 0.001f) {
        return 0.0f;
    }
    const float t = 2.0f * x - 1.0f;
    return -1.0f - 0.65f * t - 0.35f * t * t * t;
}

inline float evalOctaveRise(float p) noexcept {
    return -evalSubDropOctaves(p);
}

} // namespace

void evaluateLiveMixerModel(const KeyboardInputView& in, EngineControlState& state,
                            HudState& hud) noexcept {
    KeyboardModState mod{};
    PunchTracker punch{};

    // 1. Right-hand Console MUTE 1-4 (Keys 13-16) & SHIFT + SOLO 1-4
    evaluateConsoleSoloMute(in, mod, punch);

    // 2. OP1: DRIVE -> progressive 4.5x MS-20 overdrive + starved-rail sync
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op1)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op1);
        mod.overloadSurge = in.blackPressure[kIdx];
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::OverloadSurge, 0u, mod.overloadSurge);
    }

    // 3. OP2: SUB OCTAVE (continuous 0.0 -> -2.0 Oct dive)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op2)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op2);
        const float oct = evalSubDropOctaves(in.blackPressure[kIdx]);
        const float mul = fastExp2(oct);
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePitchMul[v] *= mul;
        }
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::SubDrop, 0u, oct);
    }

    // 4. OP3: RATIO LOCK (Just-Intonation interval snap + drift damping)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op3)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op3);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        const float lockAmt = p * (1.25f - 0.25f * p);
        mod.ratioLockAmount = lockAmt;
        const float anchorHz =
            clampf(state.voices[0].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
        size_t repRatioIdx = 12u;
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            const float vHz = clampf(state.voices[v].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz);
            const size_t rIdx = findNearestJustRatioIdx(fastLog2(vHz / anchorHz));
            if (v == 1u) {
                repRatioIdx = rIdx;
            }
            const float targetHz = anchorHz * kJustRatioTable[rIdx].ratio;
            const float logDelta = fastLog2(targetHz / vHz);
            mod.voicePitchMul[v] *= fastExp2(lockAmt * logDelta);
        }
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::RatioMagnet,
                   static_cast<uint8_t>(repRatioIdx), p);
    }

    // 5. OP4: LOW PASS sweep (28 kHz -> 31 Hz) & OP5: HIGH PASS sweep (20 Hz -> 3.2 kHz)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op4)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op4);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.dubLpfDrop = p;
        const float cutoffHz = 28000.0f * fastExp2(-9.8f * p);
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::DubLpfDrop, 0u, cutoffHz);
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op5)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op5);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.masterHpfSweep = p;
        const float cutoffHz = 20.0f * fastExp2(7.32f * p);
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::HighPass, 0u, cutoffHz);
    }

    // 6. SPACE Cluster (`OP6 = UNISON`, `PIT = DETUNE`, `GLO = SPREAD`):
    constexpr size_t kUniIdx = static_cast<size_t>(BlackKeyId::Op6);
    constexpr size_t kDetIdx = static_cast<size_t>(BlackKeyId::Pit);
    constexpr size_t kSprIdx = static_cast<size_t>(BlackKeyId::Glo);
    if (in.blackHeld[kUniIdx]) {
        const float p = clampf(in.blackPressure[kUniIdx], 0.0f, 1.0f);
        const float u = p * (1.25f - 0.25f * p);
        const float logAnchor =
            fastLog2(clampf(state.voices[0].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz));
        constexpr std::array<float, 4> kUnisonOct = {0.0f, -0.0015f, 0.0016f, -0.0026f};
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            const float logV =
                fastLog2(clampf(state.voices[v].frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz));
            const float logDelta = (logAnchor - logV) + kUnisonOct[v];
            mod.voicePitchMul[v] *= fastExp2(u * logDelta);
        }
        if (0.75f * u > mod.ratioLockAmount) {
            mod.ratioLockAmount = 0.75f * u;
        }
        mod.spatialFocus = -p;
        punch.note(in.blackPressSeq[kUniIdx], KeyPunchKind::MonoFocus, 0u, p);
    }
    if (in.blackHeld[kDetIdx]) {
        const float p = clampf(in.blackPressure[kDetIdx], 0.0f, 1.0f);
        const float d = p * (0.70f + 0.30f * p);
        constexpr std::array<float, 4> kChorusOct = {-0.052f, 0.056f, -0.102f, 0.112f};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePitchMul[v] *= fastExp2(kChorusOct[v] * d);
        }
        punch.note(in.blackPressSeq[kDetIdx], KeyPunchKind::DetuneChorus, 0u, p);
    }
    if (in.blackHeld[kSprIdx]) {
        const float p = clampf(in.blackPressure[kSprIdx], 0.0f, 1.0f);
        const float s = p * (0.75f + 0.25f * p);
        constexpr std::array<float, 4> kSwarmPw = {-0.22f, 0.20f, -0.32f, 0.30f};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePwDelta[v] += kSwarmPw[v] * s;
        }
        mod.spatialFocus = clampf(mod.spatialFocus + p, -1.0f, 1.0f);
        punch.note(in.blackPressSeq[kSprIdx], KeyPunchKind::PolySpread, 0u, p);
    }

    // 7. Bottom 3x4 Expressive Oscillator Banks (`FILTER 1-4` | `OCTAVE 5-8` | `DUTY 9-12`)
    std::array<float, kNumOscillators> voiceFormant{0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        if (in.whiteHeld[kBankAWhiteKeyBase + v]) {
            const float p = in.whitePressure[kBankAWhiteKeyBase + v];
            voiceFormant[v] = p;
            punch.note(in.whitePressSeq[kBankAWhiteKeyBase + v], KeyPunchKind::VoiceFormant, v, p);
        }
        if (in.whiteHeld[kBankBWhiteKeyBase + v]) {
            const float p = in.whitePressure[kBankBWhiteKeyBase + v];
            const float oct = evalOctaveRise(p);
            mod.voicePitchMul[v] *= fastExp2(oct);
            const float sirenBite = 0.72f * p * p;
            if (sirenBite > voiceFormant[v]) {
                voiceFormant[v] = sirenBite;
            }
            punch.note(in.whitePressSeq[kBankBWhiteKeyBase + v], KeyPunchKind::VoiceSiren, v, oct);
        }
        if (in.whiteHeld[kSceneWhiteKeyBase + v] && !hud.saveArmed) {
            const float p = clampf(in.whitePressure[kSceneWhiteKeyBase + v], 0.0f, 1.0f);
            const float dir = (state.voices[v].pulseWidth <= 0.50f) ? 0.42f : -0.42f;
            mod.voicePwDelta[v] += dir * p;
            punch.note(in.whitePressSeq[kSceneWhiteKeyBase + v], KeyPunchKind::VoiceDuty, v, p);
        }
        state.voicePressure[v] = voiceFormant[v];
    }

    punch.commitTo(hud);
    state.keyMod = mod;
}

void choreographLiveMixerLeds(const KeyboardInputView& in, float animPhase,
                              const EngineControlState& state, const HudState& hud,
                              uint8_t stutMask, float rhythmPhase01, LedState& outLeds) noexcept {
    (void)stutMask;
    (void)rhythmPhase01;

    // 1. OP1 Drive: Dual-edge plasma flood + top VU bar + physical RED REC clip beacon!
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op1)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op1)];
        const float reach = p * 8.4f;
        const float shimmer =
            (p > 0.55f) ? (0.80f + 0.20f * evalAnalogSine(wrapLfoPhase01(animPhase * 13.0f)))
                        : 1.0f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float edgeDist = (i < 6u) ? static_cast<float>(i) : static_cast<float>(11u - i);
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

    // 2. OP2 Sub Octave: OCT- blazes 100%, bottom keys throb at 2 Hz
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op2)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op2)];
        const float subThrob =
            0.55f + 0.45f * (0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(animPhase * 2.0f))));
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            outLeds.whiteKeyLeds[i] = clampf(0.32f + 0.68f * p * subThrob, 0.0f, 1.0f);
        }
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.55f + 0.45f * p);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.0f);
    }

    // 3. OP3 Ratio Lock: Beating switch columns converge into steady harmonic lock as p -> 1
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op3)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op3)];
        const float waveA = 0.5f * (1.0f + evalAnalogSine(wrapLfoPhase01(animPhase * 4.0f)));
        for (size_t b = 0u; b < 3u; ++b) {
            outLeds.whiteKeyLeds[b * 4u] = clampf(0.55f + 0.45f * p, 0.0f, 1.0f);
            for (size_t v = 1u; v < 4u; ++v) {
                const float lockGlow = p * (0.45f + 0.45f * (1.0f - p) * waveA + 0.45f * p);
                if (lockGlow > outLeds.whiteKeyLeds[b * 4u + v]) {
                    outLeds.whiteKeyLeds[b * 4u + v] = lockGlow;
                }
            }
        }
    }

    // 4. OP4 Low Pass (R->L closing curtain) & OP5 High Pass (L->R rising bass-cut curtain)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op4)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op4)];
        const float crestIdx = (1.0f - p) * 11.0f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float fi = static_cast<float>(i);
            const float crest = clampf(1.0f - absf(fi - crestIdx) * 0.55f, 0.0f, 1.0f);
            const float pass = (fi <= crestIdx) ? 0.34f : 0.02f;
            outLeds.whiteKeyLeds[i] = clampf(pass + 0.66f * crest, 0.0f, 1.0f);
        }
        scalePanelGrid(outLeds, 0.14f);
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, 0.55f + 0.45f * p);
        maxPanelBtnLed(outLeds, ButtonId::OctPlus, 0.45f + 0.55f * p);
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.45f + 0.55f * p);
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op5)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op5)];
        const float crestIdx = p * 11.0f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float fi = static_cast<float>(i);
            const float crest = clampf(1.0f - absf(fi - crestIdx) * 0.55f, 0.0f, 1.0f);
            const float pass = (fi >= crestIdx) ? 0.34f : 0.02f;
            outLeds.whiteKeyLeds[i] = clampf(pass + 0.66f * crest, 0.0f, 1.0f);
        }
        scalePanelGrid(outLeds, 0.14f);
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, 0.55f + 0.45f * p);
        maxPanelBtnLed(outLeds, ButtonId::OctPlus, 0.45f + 0.55f * p);
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.45f + 0.55f * p);
    }

    // 5. SPACE Cluster (`OP6 = UNISON`, `PIT = DETUNE`, `GLO = SPREAD`)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op6)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Op6)];
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float tgt = (i >= 4u && i <= 7u) ? 0.95f : 0.03f;
            outLeds.whiteKeyLeds[i] += (tgt - outLeds.whiteKeyLeds[i]) * p;
        }
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Pit)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Pit)];
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float w =
                0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(animPhase * 2.4f + i * 0.14f));
            outLeds.whiteKeyLeds[i] = clampf(0.15f + 0.78f * p * w, 0.0f, 1.0f);
        }
    }
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Glo)]) {
        const float p = in.blackPressure[static_cast<size_t>(BlackKeyId::Glo)];
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float tgt = (i <= 2u || i >= 9u) ? 0.95f : 0.04f;
            outLeds.whiteKeyLeds[i] += (tgt - outLeds.whiteKeyLeds[i]) * p;
        }
    }

    // 6. Bottom Oscillator Banks (`FILTER 1-4` | `OCTAVE 5-8` | `DUTY 9-12`) + Top Held Pins
    for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
        if (in.blackHeld[k]) {
            outLeds.blackKeyLeds[k] = 1.0f;
        }
    }
    for (size_t v = 0u; v < kNumOscillators; ++v) {
        if (in.whiteHeld[kBankAWhiteKeyBase + v]) {
            const float p = in.whitePressure[kBankAWhiteKeyBase + v];
            outLeds.whiteKeyLeds[kBankAWhiteKeyBase + v] = 0.84f + 0.16f * p;
            maxPanelBtnLed(outLeds, ButtonId::Fx, 0.48f + 0.52f * p);
        }
        if (in.whiteHeld[kBankBWhiteKeyBase + v]) {
            const float p = in.whitePressure[kBankBWhiteKeyBase + v];
            outLeds.whiteKeyLeds[kBankBWhiteKeyBase + v] = 0.52f + 0.48f * p;
            maxPanelBtnLed(outLeds, ButtonId::OctPlus, 0.55f + 0.45f * p);
        }
        if (in.whiteHeld[kSceneWhiteKeyBase + v] && !hud.saveArmed) {
            const float p = in.whitePressure[kSceneWhiteKeyBase + v];
            outLeds.whiteKeyLeds[kSceneWhiteKeyBase + v] = 0.45f + 0.55f * p;
            maxPanelBtnLed(outLeds, ButtonId::Home, 0.45f + 0.55f * p);
        }
    }

    applyAuthoritativeConsoleLeds(state, hud, outLeds);
}

} // namespace ruido
