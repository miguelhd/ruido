#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/LfoMath.h"
#include "core/RhythmMath.h"

namespace ruido {

void onRhythmLabKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                         KeyboardLatchState& latch) noexcept {
    onConsoleSoloMuteKeyPress(isBlack, keyIdx, config, latch);
}

void evaluateRhythmLabModel(const KeyboardInputView& in, EngineControlState& state,
                            HudState& hud) noexcept {
    KeyboardModState mod{};
    PunchTracker punch{};

    // 1. Shared Console SOLO 1-4 (OP1-OP4) & MUTE 1-4 (Keys 1-4) + 11th Tall Key TAP TEMPO
    evaluateConsoleSoloMute(in, mod, punch);
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)]) {
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)],
                   KeyPunchKind::TapTempo, 0u, 1.0f);
    }

    // 2. Top Raised Black Keys: Global Rhythm & Clock Modifiers
    // OP5: PUMP -> 4-on-the-floor sidechain ducking swell (double-time above 72% pressure)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op5)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op5)], 0.0f, 1.0f);
        mod.rhythmPump = p;
        mod.overloadSurge = 0.28f * p * p;
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Op5)], KeyPunchKind::RhythmPump,
                   0u, p);
    }

    // OP6: CHASE -> 1 -> 2 -> 3 -> 4 round-robin oscillator step loop
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op6)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op6)], 0.0f, 1.0f);
        mod.rhythmChase = p;
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Op6)],
                   KeyPunchKind::RhythmChase, 0u, p);
    }

    // PIT: TEMPO -> 1.0x -> 4.0x master clock multiplier
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Pit)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Pit)], 0.0f, 1.0f);
        mod.rhythmTempo = p;
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Pit)],
                   KeyPunchKind::RhythmTempo, 0u, p);
    }

    // GLO: PLUCK -> Short percussive MS-20 low-pass filter plucks + live 4-voice filter graph
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Glo)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Glo)], 0.0f, 1.0f);
        mod.rhythmPluck = p;
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Glo)],
                   KeyPunchKind::RhythmPluck, 0u, p);
    }

    // MONO: SHORT -> Pinpoint 50% -> 6% gate length + tight center stereo focus
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Mono)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Mono)], 0.0f, 1.0f);
        mod.rhythmShort = p;
        mod.spatialFocus = clampf(mod.spatialFocus - 0.72f * p, -1.0f, 1.0f);
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Mono)],
                   KeyPunchKind::RhythmShort, 0u, p);
    }

    // POLY: SWING -> 50% -> 72% MPC shuffle swing + wide stereo bounce
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Poly)]) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Poly)], 0.0f, 1.0f);
        mod.rhythmSwing = p;
        mod.spatialFocus = clampf(mod.spatialFocus + 0.65f * p, -1.0f, 1.0f);
        punch.note(in.blackPressSeq[static_cast<size_t>(BlackKeyId::Poly)],
                   KeyPunchKind::RhythmSwing, 0u, p);
    }

    // 3. Bottom White Keys: Per-Oscillator Rhythm Engines (STUTTER | DIVIDE | EUCLID)
    bool anyVoiceRhythm = false;
    std::array<float, kNumOscillators> voiceFormant{0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        uint32_t winSeq = 0u;
        VoiceRhythmMode winMode = VoiceRhythmMode::Off;
        float winPressure = 0.0f;

        // Bank B (Keys 5-8): STUTTER -> Free-speed chopper (1.5 -> 24 Hz, ignores clock)
        if (in.whiteHeld[4u + v]) {
            const float p = clampf(in.whitePressure[4u + v], 0.0f, 1.0f);
            const uint32_t seq = in.whitePressSeq[4u + v];
            punch.note(seq, KeyPunchKind::VoiceStutter, v, p);
            if (seq >= winSeq) {
                winSeq = seq;
                winMode = VoiceRhythmMode::Stutter;
                winPressure = p;
            }
        }

        // Bank C (Keys 9-12): DIVIDE -> Master-clock-locked musical dividers (/2 .. x8)
        if (in.whiteHeld[8u + v]) {
            const float p = clampf(in.whitePressure[8u + v], 0.0f, 1.0f);
            const uint32_t seq = in.whitePressSeq[8u + v];
            punch.note(seq, KeyPunchKind::VoiceDivide, v, p);
            if (seq >= winSeq) {
                winSeq = seq;
                winMode = VoiceRhythmMode::Divide;
                winPressure = p;
            }
        }

        // Bank D (Keys 13-16): EUCLID -> 16-step Euclidean groove patterns (unless SAVE is armed)
        if (in.whiteHeld[12u + v] && !hud.saveArmed) {
            const float p = clampf(in.whitePressure[12u + v], 0.0f, 1.0f);
            const uint32_t seq = in.whitePressSeq[12u + v];
            punch.note(seq, KeyPunchKind::VoiceEuclid, v, p);
            if (seq >= winSeq) {
                winSeq = seq;
                winMode = VoiceRhythmMode::Euclid;
                winPressure = p;
            }
        }

        if (winMode != VoiceRhythmMode::Off) {
            anyVoiceRhythm = true;
            mod.rhythmMode[v] = static_cast<uint8_t>(winMode);
            mod.stutterDepth[v] = 1.0f;
            voiceFormant[v] = 0.34f + 0.56f * winPressure;
            if (winMode == VoiceRhythmMode::Stutter) {
                mod.stutterRateHz[v] = computeStutterRateHz(winPressure);
            } else if (winMode == VoiceRhythmMode::Divide) {
                mod.rhythmParam[v] = selectVoiceClockDivIndex(v, winPressure);
            } else if (winMode == VoiceRhythmMode::Euclid) {
                mod.rhythmParam[v] = selectEuclidTierIndex(winPressure);
            }
        }
        state.voicePressure[v] = voiceFormant[v];
    }

    // Foreground mix focus: when playing individual bottom rhythm keys without SOLO or global
    // CHASE/PUMP, gently tuck un-gated drone voices to 34% so rhythmic patterns punch cleanly.
    const bool hasChase = (mod.rhythmChase > 0.001f);
    const bool hasPump = (mod.rhythmPump > 0.001f);
    const bool hasSoloTopPulse = !anyVoiceRhythm && !hasChase && !hasPump &&
                                 ((mod.rhythmTempo > 0.001f) || (mod.rhythmPluck > 0.001f) ||
                                  (mod.rhythmShort > 0.001f) || (mod.rhythmSwing > 0.001f));

    if (anyVoiceRhythm && mod.soloMask == 0u && !hasChase && !hasPump) {
        for (uint8_t v = 0u; v < kNumOscillators; ++v) {
            if (mod.rhythmMode[v] != 0u) {
                mod.voiceGainMul[v] = clampf(mod.voiceGainMul[v] * 1.14f, 0.0f, 1.35f);
            } else {
                mod.voiceGainMul[v] *= 0.34f;
            }
        }
    }

    // Mirror global gate activity into stutterDepth so VoiceCircuit columns & Console LEDs pulse
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        if (hasChase || hasSoloTopPulse) {
            mod.stutterDepth[v] = 1.0f;
        } else if (hasPump && mod.stutterDepth[v] < 0.90f * mod.rhythmPump) {
            mod.stutterDepth[v] = 0.90f * mod.rhythmPump;
        }
    }

    punch.commitTo(hud);
    state.keyMod = mod;
}

void choreographRhythmLabLeds(const KeyboardInputView& in, float animPhase, float subBreathWave01,
                              const EngineControlState& state, const HudState& hud,
                              uint8_t stutMask, float rhythmPhase01, LedState& outLeds) noexcept {
    (void)animPhase;
    const float barPhase = wrapLfoPhase01(rhythmPhase01);
    const auto step16 = static_cast<size_t>(barPhase * 16.0f) & 15u;
    const float step16Frac = barPhase * 16.0f - static_cast<float>(step16);
    // Smooth 60 Hz anti-strobe damping when effective clock outruns visual step persistence
    const float effBpm = hud.effectiveClockBpm * computeTempoMultiplier(state.keyMod.rhythmTempo);
    const float stepSmooth = smoothstep01(clampf((effBpm - 320.0f) * (1.0f / 280.0f), 0.0f, 1.0f));
    const float beatSmooth = smoothstep01(clampf((effBpm - 480.0f) * (1.0f / 420.0f), 0.0f, 1.0f));

    // LIFO top-row show selection across OP5..POLY (indices 4..9)
    int topBlackShow = -1;
    uint32_t topBlackSeq = 0u;
    for (size_t k = 4u; k <= 9u; ++k) {
        if (in.blackHeld[k] && (topBlackShow < 0 || in.blackPressSeq[k] >= topBlackSeq)) {
            topBlackShow = static_cast<int>(k);
            topBlackSeq = in.blackPressSeq[k];
        }
    }

    // 1. OP5 PUMP: 4-on-the-floor sidechain duck on the beat + red REC kick transient + swell
    if (topBlackShow == static_cast<int>(BlackKeyId::Op5)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op5)], 0.0f, 1.0f);
        const float cycles = (p >= 0.72f) ? 8.0f : 4.0f;
        const float beatScaled = barPhase * cycles;
        const float beatFrac = beatScaled - static_cast<float>(static_cast<uint32_t>(beatScaled));
        const float rawSwell = smoothstep01(beatFrac * 1.18f);
        const float swell = rawSwell + (0.56f - rawSwell) * beatSmooth;
        const float rawKick = fastExpNeg(beatFrac * 8.5f);
        const float kickFlash = rawKick + (0.38f - rawKick) * beatSmooth;
        const float whiteBreathe = (1.0f - 0.94f * p) + 0.94f * p * swell;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            outLeds.whiteKeyLeds[i] = clampf(0.03f + 0.92f * whiteBreathe, 0.02f, 1.0f);
        }
        for (size_t i = 4u; i < 10u; ++i) {
            outLeds.blackKeyLeds[i] = clampf(0.05f + 0.75f * whiteBreathe, 0.02f, 1.0f);
        }
        scalePanelGrid(outLeds, 0.12f + 0.88f * whiteBreathe);
        for (size_t c = 0u; c < 6u; ++c) {
            maxPanelBtnLed(outLeds, kPanelGridTop[c], 0.15f + 0.80f * p * swell);
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], 0.12f + 0.78f * p * swell);
            }
        }
        maxPanelBtnLed(outLeds, ButtonId::Rec, clampf(kickFlash * (0.45f + 0.55f * p), 0.0f, 1.0f));
    }

    // 2. OP6 CHASE: 1 -> 2 -> 3 -> 4 round-robin column spotlight chaser synced to stutMask
    if (topBlackShow == static_cast<int>(BlackKeyId::Op6)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op6)], 0.0f, 1.0f);
        scalePanelGrid(outLeds, 0.14f);
        for (size_t i = 4u; i < 10u; ++i) {
            outLeds.blackKeyLeds[i] = 0.08f;
        }
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            const bool activeCol = (((stutMask >> v) & 1u) != 0u);
            const float rawLvl = activeCol ? (0.55f + 0.45f * p) : 0.03f;
            const float lvl = rawLvl + ((0.28f + 0.22f * p) - rawLvl) * stepSmooth;
            for (size_t b = 0u; b < 4u; ++b) {
                outLeds.whiteKeyLeds[b * 4u + v] = lvl;
            }
            if (activeCol || stepSmooth > 0.05f) {
                const float gLvl =
                    activeCol ? (0.65f + 0.35f * p) : ((0.30f + 0.20f * p) * stepSmooth);
                maxPanelBtnLed(outLeds, kPanelGridTop[1u + v], gLvl);
                maxPanelBtnLed(outLeds, kPanelGridBot[1u + v], gLvl * 0.88f);
            }
        }
    }

    // 3. PIT TEMPO: Clock-locked 16-step speedometer comet + OCT+ speedup beacon
    if (topBlackShow == static_cast<int>(BlackKeyId::Pit)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Pit)], 0.0f, 1.0f);
        const float head = barPhase * 16.0f;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            float dist = absf(static_cast<float>(i) - head);
            if (dist > 8.0f) {
                dist = 16.0f - dist;
            }
            const float comet = clampf(1.0f - dist * 0.48f, 0.0f, 1.0f);
            const float shaped = comet * comet + (0.52f - comet * comet) * stepSmooth;
            outLeds.whiteKeyLeds[i] = 0.03f + (0.45f + 0.52f * p) * shaped;
        }
        scalePanelGrid(outLeds, 0.18f);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.45f + 0.55f * p);
        setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.05f);
        const size_t beatCol = (step16 >> 2u) & 3u;
        for (size_t c = 0u; c < 6u; ++c) {
            const float rawFlash = (c == (1u + beatCol)) ? (0.65f + 0.35f * p) : 0.08f;
            const float flash = rawFlash + ((0.32f + 0.20f * p) - rawFlash) * beatSmooth;
            maxPanelBtnLed(outLeds, kPanelGridTop[c], flash);
        }
        if (((step16 & 3u) == 0u && step16Frac < 0.35f) || beatSmooth > 0.25f) {
            const float recHit = ((step16 & 3u) == 0u && step16Frac < 0.35f) ? 1.0f : 0.0f;
            const float recBlend = recHit + (0.42f - recHit) * beatSmooth;
            maxPanelBtnLed(outLeds, ButtonId::Rec, (0.35f + 0.65f * p) * recBlend);
        }
    }

    // 4. GLO PLUCK: Percussive MS-20 filter pluck curtain snapping & decaying on every trigger
    if (topBlackShow == static_cast<int>(BlackKeyId::Glo)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Glo)], 0.0f, 1.0f);
        const float rawEnv =
            (stutMask != 0u) ? fastExpNeg(step16Frac * (2.6f + 4.8f * p)) : (0.12f * (1.0f - p));
        const float env = rawEnv + (0.48f - rawEnv) * stepSmooth;
        const float reach = clampf(env * (4.0f + 12.0f * p), 0.0f, 15.5f);
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const float fi = static_cast<float>(i);
            const float bar = clampf(reach - fi, 0.0f, 1.0f);
            outLeds.whiteKeyLeds[i] = 0.02f + 0.94f * bar;
        }
        scalePanelGrid(outLeds, 0.16f);
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.60f + 0.40f * env);
        for (size_t c = 0u; c < 6u; ++c) {
            const float gBar = clampf(env * 6.2f - static_cast<float>(c), 0.0f, 1.0f);
            maxPanelBtnLed(outLeds, kPanelGridTop[c], gBar * (0.45f + 0.55f * p));
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], gBar * (0.38f + 0.50f * p));
            }
        }
    }

    // 5. MONO SHORT: Pinpoint staccato needle flashes in center chassis
    if (topBlackShow == static_cast<int>(BlackKeyId::Mono)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Mono)], 0.0f, 1.0f);
        const bool open = (stutMask != 0u);
        const float gateVis = (open ? 1.0f : 0.0f) + (0.45f - (open ? 1.0f : 0.0f)) * stepSmooth;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const bool center = (i >= 6u && i <= 9u);
            outLeds.whiteKeyLeds[i] = center ? (0.04f + (0.56f + 0.40f * p) * gateVis) : 0.02f;
        }
        scalePanelGrid(outLeds, 0.10f);
        const float pin = 0.04f + (0.61f + 0.35f * p) * gateVis;
        maxPanelBtnLed(outLeds, ButtonId::Env, pin);
        maxPanelBtnLed(outLeds, ButtonId::Lfo, pin);
        maxPanelBtnLed(outLeds, ButtonId::Arp, pin);
        maxPanelBtnLed(outLeds, ButtonId::Seq, pin);
    }

    // 6. POLY SWING: Left-to-right shuffle pendulum showing delayed off-beat swing groove
    if (topBlackShow == static_cast<int>(BlackKeyId::Poly)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Poly)], 0.0f, 1.0f);
        const bool rightSwing = ((step16 & 1u) != 0u);
        const float pulse = fastExpNeg(step16Frac * 2.4f);
        const float activeLvl = clampf(0.38f + 0.62f * pulse * (0.45f + 0.55f * p), 0.0f, 1.0f);
        const float steadyLvl = 0.32f + 0.36f * p;
        for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
            const bool isRight = (i >= 8u);
            const float raw = (isRight == rightSwing) ? activeLvl : 0.03f;
            outLeds.whiteKeyLeds[i] = raw + (steadyLvl - raw) * stepSmooth;
        }
        scalePanelGrid(outLeds, 0.14f);
        const float octL = (rightSwing ? 0.05f : activeLvl);
        const float octR = (rightSwing ? activeLvl : 0.05f);
        setPanelBtnLed(outLeds, ButtonId::OctMinus, octL + (steadyLvl - octL) * stepSmooth);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, octR + (steadyLvl - octR) * stepSmooth);
        for (size_t c = 0u; c < 6u; ++c) {
            const bool colRight = (c >= 3u);
            const float rawC = (colRight == rightSwing) ? activeLvl : 0.04f;
            const float cLvl = rawC + (steadyLvl - rawC) * stepSmooth;
            maxPanelBtnLed(outLeds, kPanelGridTop[c], cLvl);
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], cLvl * 0.85f);
            }
        }
    }

    // 7. Bottom Rhythm Shows across all 42 LEDs (when no top-row show leads):
    //    STUTTER (Keys 5-8) | DIVIDE (Keys 9-12) | EUCLID (Keys 13-16)
    int botKey = -1;
    uint32_t botSeq = 0u;
    for (size_t k = 4u; k < kNumWhiteKeys; ++k) {
        if (k >= 12u && hud.saveArmed) {
            break;
        }
        if (in.whiteHeld[k] && (botKey < 0 || in.whitePressSeq[k] >= botSeq)) {
            botKey = static_cast<int>(k);
            botSeq = in.whitePressSeq[k];
        }
    }

    if (topBlackShow < 0 && botKey >= 4) {
        const auto wIdx = static_cast<size_t>(botKey);
        scalePanelGrid(outLeds, 0.16f);

        if (wIdx < 8u) {
            // 7a. STUTTER (5-8): Polyrhythmic radial strobe shockwaves synced 1:1 to stutMask
            for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
                outLeds.whiteKeyLeds[i] = 0.03f;
            }
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (!in.whiteHeld[4u + v]) {
                    continue;
                }
                const float p = clampf(in.whitePressure[4u + v], 0.0f, 1.0f);
                const bool open = (((stutMask >> v) & 1u) != 0u);
                if (open) {
                    for (size_t b = 0u; b < 4u; ++b) {
                        outLeds.whiteKeyLeds[b * 4u + v] = 0.58f + 0.42f * p;
                    }
                    maxPanelBtnLed(outLeds, kPanelGridTop[1u + v], 0.65f + 0.35f * p);
                    maxPanelBtnLed(outLeds, kPanelGridBot[1u + v], 0.55f + 0.40f * p);
                } else {
                    outLeds.whiteKeyLeds[4u + v] = 0.24f;
                }
            }
        } else if (wIdx < 12u) {
            // 7b. DIVIDE (9-12): 16-step master clock playhead + 4-key zone bursts per divider
            for (size_t i = 0u; i < kNumWhiteKeys; ++i) {
                const float rawPlayhead = (i == step16) ? 0.44f : 0.03f;
                outLeds.whiteKeyLeds[i] = rawPlayhead + (0.10f - rawPlayhead) * stepSmooth;
            }
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (!in.whiteHeld[8u + v]) {
                    continue;
                }
                const float p = clampf(in.whitePressure[8u + v], 0.0f, 1.0f);
                const bool open = (((stutMask >> v) & 1u) != 0u);
                const float rawBurst = open ? (0.52f + 0.48f * p) : 0.0f;
                const float burst = rawBurst + ((0.34f + 0.32f * p) - rawBurst) * stepSmooth;
                if (burst > 0.05f) {
                    for (size_t k = 0u; k < 4u; ++k) {
                        const size_t idx = v * 4u + k;
                        if (burst > outLeds.whiteKeyLeds[idx]) {
                            outLeds.whiteKeyLeds[idx] = burst;
                        }
                    }
                    maxPanelBtnLed(outLeds, kPanelGridTop[1u + v],
                                   open ? (0.70f + 0.30f * p) : ((0.40f + 0.25f * p) * stepSmooth));
                }
            }
            if ((step16 & 3u) == 0u || beatSmooth > 0.25f) {
                const float seqRaw = ((step16 & 3u) == 0u) ? 0.75f : 0.16f;
                maxPanelBtnLed(outLeds, ButtonId::Seq, seqRaw + (0.45f - seqRaw) * beatSmooth);
            }
        } else {
            // 7c. EUCLID (13-16): Live 16-step Euclidean pattern across Keys 1-16 + step cursor!
            uint16_t unionMask = 0u;
            float maxP = 0.0f;
            for (uint8_t v = 0u; v < kNumOscillators; ++v) {
                if (in.whiteHeld[12u + v]) {
                    const float p = clampf(in.whitePressure[12u + v], 0.0f, 1.0f);
                    unionMask |= voiceEuclidMask16(v, selectEuclidTierIndex(p));
                    if (p > maxP) {
                        maxP = p;
                    }
                }
            }
            const bool hitNow = (((unionMask >> step16) & 1u) != 0u);
            for (size_t s = 0u; s < kNumWhiteKeys; ++s) {
                const bool isStepHit = (((unionMask >> s) & 1u) != 0u);
                const float baseHit =
                    isStepHit ? (0.28f + 0.32f * stepSmooth + 0.18f * maxP) : 0.02f;
                const float cursorBoost =
                    (s == step16) ? ((isStepHit ? 0.72f : 0.60f) * (1.0f - stepSmooth)) : 0.0f;
                outLeds.whiteKeyLeds[s] = clampf(baseHit + cursorBoost, 0.02f, 1.0f);
            }
            const float seqRaw = hitNow ? 1.0f : 0.25f;
            maxPanelBtnLed(outLeds, ButtonId::Seq, seqRaw + (0.55f - seqRaw) * stepSmooth);
            const float arpRaw = ((step16 & 3u) == 0u) ? 0.80f : 0.15f;
            maxPanelBtnLed(outLeds, ButtonId::Arp, arpRaw + (0.42f - arpRaw) * beatSmooth);
            for (size_t v = 0u; v < kNumOscillators; ++v) {
                if (in.whiteHeld[12u + v]) {
                    const bool open = (((stutMask >> v) & 1u) != 0u);
                    if (open || stepSmooth > 0.10f) {
                        const float colRaw = open ? (0.72f + 0.28f * maxP) : 0.16f;
                        maxPanelBtnLed(outLeds, kPanelGridTop[1u + v],
                                       colRaw + ((0.44f + 0.22f * maxP) - colRaw) * stepSmooth);
                    }
                }
            }
            if ((hitNow || stepSmooth > 0.25f) && maxP > 0.65f) {
                const float recRaw = hitNow ? (0.65f * maxP) : 0.0f;
                maxPanelBtnLed(outLeds, ButtonId::Rec,
                               recRaw + (0.35f * maxP - recRaw) * stepSmooth);
            }
        }
    }

    // Keep every physically held top black key (OP5..POLY) unmistakably at 100%
    for (size_t k = 4u; k <= 9u; ++k) {
        if (in.blackHeld[k]) {
            outLeds.blackKeyLeds[k] = 1.0f;
        }
    }
    // Keep SAVE + SNAP 13-16 clear when SAVE is armed
    if (hud.saveArmed) {
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            if (in.whiteHeld[12u + s]) {
                outLeds.whiteKeyLeds[12u + s] = 1.0f;
            }
        }
    }

    // 8. 11th Key Tap Tempo flash + Authoritative Console SOLO (OP1-4) & MUTE (Keys 1-4)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)]) {
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::ScopeFreezeKey)] = 1.0f;
    }
    applyAuthoritativeConsoleLeds(state, stutMask, subBreathWave01, outLeds);
}

} // namespace ruido
