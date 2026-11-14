#include "core/DspMath.h"
#include "core/KeyboardModel.h"
#include "core/LfoMath.h"
#include "core/VoiceCircuit.h"

namespace ruido {

namespace {

// Deterministic integer hash -> [0.0f, 1.0f] for organic brownout sputter & Geiger sparks.
inline float clHashFloat01(uint32_t seed) noexcept {
    uint32_t x = seed * 0x9E3779B9u + 0x7F4A7C15u;
    x ^= x >> 16u;
    x *= 0x85EBCA6Bu;
    x ^= x >> 13u;
    x *= 0xC2B2AE35u;
    x ^= x >> 16u;
    return static_cast<float>(x >> 8u) * (1.0f / 16777216.0f);
}

} // namespace

void onCircuitLabKeyPress(bool isBlack, uint8_t keyIdx, const GlobalConfigState& config,
                          KeyboardLatchState& latch) noexcept {
    onConsoleSoloMuteKeyPress(isBlack, keyIdx, config, latch);
}

void evaluateCircuitLabModel(const KeyboardInputView& in, EngineControlState& state,
                             HudState& hud) noexcept {
    KeyboardModState mod{};
    std::array<float, kNumOscillators> voiceFormant{0.0f, 0.0f, 0.0f, 0.0f};
    PunchTracker punch{};

    // 1. PIT..POLY (SOLO 1-4) & White Keys 12..15 (MUTE 1-4): Shared Momentary/Latching Console
    evaluateConsoleSoloMute(in, mod, punch);

    // 2. OP1: STARVE -> Dying-9V CMOS rail collapse, asymmetric duty thinning & voltage pitch sag
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op1)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op1);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.overloadSurge = p;
        const float sagMul = fastExp2(-1.15f * p * p);
        constexpr std::array<float, 4> kStarvePw = {-0.34f, 0.31f, -0.38f, 0.36f};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePwDelta[v] += kStarvePw[v] * p;
            mod.voicePitchMul[v] *= sagMul;
        }
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::RailStarve, 0u, p);
    }

    // 3. OP2: RING MOD -> 4x oversampled paired Ring / XOR sideband matrix (1x2, 2x3, 3x4, 4x1)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op2)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op2);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.ringModMix = p;
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::RingXor, 0u, p);
    }

    // 4. OP3: TURBO LFO -> 1x..32x LFO overclock + momentary dry-patch depth floor
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op3)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op3);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.lfoTurbo = p;
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::LfoTurbo, 0u, p);
    }

    // 5. OP4: SCREAM -> Twin Korg35 HPF + LPF self-oscillation sweep (opens GlobalFilter graph)
    if (in.blackHeld[static_cast<size_t>(BlackKeyId::Op4)]) {
        constexpr size_t kIdx = static_cast<size_t>(BlackKeyId::Op4);
        const float p = clampf(in.blackPressure[kIdx], 0.0f, 1.0f);
        mod.filterScream = p;
        punch.note(in.blackPressSeq[kIdx], KeyPunchKind::FilterScream, 0u, p);
    }

    // 6. OP5 (HARD SYNC: OSC 1 -> OSC 2..4 PolyBLEP sync + slave harmonic sweep) &
    //    OP6 (SCATTER: opposite Schmitt PWM, pitch, filter & wide pan scramble)
    constexpr size_t kMonoIdx = static_cast<size_t>(BlackKeyId::Op5);
    constexpr size_t kPolyIdx = static_cast<size_t>(BlackKeyId::Op6);
    const bool monoOn = in.blackHeld[kMonoIdx];
    const bool polyOn = in.blackHeld[kPolyIdx];
    if (monoOn) {
        const float p = clampf(in.blackPressure[kMonoIdx], 0.0f, 1.0f);
        mod.hardSyncAmount = p;
        if (0.65f * p > mod.ratioLockAmount) {
            mod.ratioLockAmount = 0.65f * p;
        }
        constexpr std::array<float, 4> kSyncSweepOct = {0.0f, 1.15f, 1.45f, 1.75f};
        for (size_t v = 1u; v < kNumOscillators; ++v) {
            mod.voicePitchMul[v] *= fastExp2(kSyncSweepOct[v] * p);
        }
        if (!polyOn || in.blackPressSeq[kMonoIdx] >= in.blackPressSeq[kPolyIdx]) {
            mod.spatialFocus = -0.35f * p;
        }
        punch.note(in.blackPressSeq[kMonoIdx], KeyPunchKind::HardSync, 0u, p);
    }
    if (polyOn) {
        const float p = clampf(in.blackPressure[kPolyIdx], 0.0f, 1.0f);
        const float s = p * (0.70f + 0.30f * p);
        constexpr std::array<float, 4> kScatOct = {-0.16f, 0.19f, -0.31f, 0.27f};
        constexpr std::array<float, 4> kScatPw = {-0.36f, 0.34f, -0.40f, 0.38f};
        constexpr std::array<float, 4> kScatFlt = {0.28f, 0.72f, 0.46f, 0.86f};
        for (size_t v = 0u; v < kNumOscillators; ++v) {
            mod.voicePitchMul[v] *= fastExp2(kScatOct[v] * s);
            mod.voicePwDelta[v] += kScatPw[v] * s;
            const float fb = kScatFlt[v] * p;
            if (fb > voiceFormant[v]) {
                voiceFormant[v] = fb;
            }
        }
        if (!monoOn || in.blackPressSeq[kPolyIdx] >= in.blackPressSeq[kMonoIdx]) {
            mod.spatialFocus = p;
        }
        punch.note(in.blackPressSeq[kPolyIdx], KeyPunchKind::PhaseScatter, 0u, p);
    }

    // 7. 11th Tall Key: Universal Freeze (8 LFOs + Drift + Scope)
    evaluateUniversalFreeze(in, mod, hud, punch);

    // 8. Bottom 4x4 Strip (CROSS-PIN 1-4 | FOLD 5-8 | CRUSH 9-12 | MUTE 13-16)
    for (uint8_t v = 0u; v < kNumOscillators; ++v) {
        // Bank A (White 0..3): CROSS-PIN -> shorts neighbor pre-mute capacitor ramp & XOR into OSC
        // v
        if (in.whiteHeld[kBankAWhiteKeyBase + v]) {
            const float p = clampf(in.whitePressure[kBankAWhiteKeyBase + v], 0.0f, 1.0f);
            mod.xmodDepth[v] = p;
            mod.voicePwDelta[v] += (((v & 1u) == 0u) ? -0.28f : 0.28f) * p;
            const float bite = 0.78f * p;
            if (bite > voiceFormant[v]) {
                voiceFormant[v] = bite;
            }
            punch.note(in.whitePressSeq[kBankAWhiteKeyBase + v], KeyPunchKind::VoiceXMod, v, p);
        }
        // Bank B (White 4..7): FOLD -> 1x..6.2x West-Coast Serge wavefolder on CD40106 cap ramp
        if (in.whiteHeld[kBankBWhiteKeyBase + v]) {
            const float p = clampf(in.whitePressure[kBankBWhiteKeyBase + v], 0.0f, 1.0f);
            mod.foldDepth[v] = p;
            const float sheen = 0.44f * p;
            if (sheen > voiceFormant[v]) {
                voiceFormant[v] = sheen;
            }
            punch.note(in.whitePressSeq[kBankBWhiteKeyBase + v], KeyPunchKind::VoiceFold, v, p);
        }
        // Bank C (White 8..11): CRUSH -> CD4040 binary sub-divider XOR + CD4013 S&H clock crush
        if (in.whiteHeld[kSceneWhiteKeyBase + v] && !hud.saveArmed) {
            const float p = clampf(in.whitePressure[kSceneWhiteKeyBase + v], 0.0f, 1.0f);
            mod.crushDepth[v] = p;
            const float ring = 0.72f * p;
            if (ring > voiceFormant[v]) {
                voiceFormant[v] = ring;
            }
            punch.note(in.whitePressSeq[kSceneWhiteKeyBase + v], KeyPunchKind::VoiceCrush, v, p);
        }
        state.voicePressure[v] = voiceFormant[v];
    }

    punch.commitTo(hud);
    state.keyMod = mod;
}

void choreographCircuitLabLeds(const KeyboardInputView& in, float animPhase, float subBreathWave01,
                               const EngineControlState& state, const HudState& hud,
                               uint8_t stutMask, float rhythmPhase01, LedState& outLeds) noexcept {
    (void)subBreathWave01;
    (void)stutMask;
    (void)rhythmPhase01;
    // LIFO black-key show selection across OP1..OP6 (indices 0..5): matches HUD header 1-to-1
    // when chording multiple global bends, while keeping every physically held black key at 100%.
    int topBlackShow = -1;
    uint32_t topBlackSeq = 0u;
    for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
        if (in.blackHeld[k] && (topBlackShow < 0 || in.blackPressSeq[k] >= topBlackSeq)) {
            topBlackShow = static_cast<int>(k);
            topBlackSeq = in.blackPressSeq[k];
        }
    }

    // 1. OP1 STARVE: Dying-9V brownout dim & organic voltage sputter across all 42 LEDs
    if (topBlackShow == static_cast<int>(BlackKeyId::Op1)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op1)], 0.0f, 1.0f);
        const auto tick = static_cast<uint32_t>(animPhase * 18.0f);
        const float flutter =
            0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(animPhase * (5.5f + 6.0f * p)));
        const float keep = 1.0f - 0.80f * p;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float rnd = clHashFloat01(tick * 31u + static_cast<uint32_t>(i));
            const float sputter = (rnd > 0.76f) ? ((rnd - 0.76f) * 3.6f * p * flutter) : 0.0f;
            outLeds.whiteKeyLeds[i] = clampf(outLeds.whiteKeyLeds[i] * keep + sputter, 0.02f, 1.0f);
        }
        for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
            const float rnd = clHashFloat01(tick * 17u + static_cast<uint32_t>(i) + 99u);
            const float sputter = (rnd > 0.78f) ? ((rnd - 0.78f) * 3.2f * p) : 0.0f;
            outLeds.blackKeyLeds[i] = clampf(outLeds.blackKeyLeds[i] * keep + sputter, 0.02f, 1.0f);
        }
        scalePanelGrid(outLeds, keep);
        for (size_t c = 0u; c < 6u; ++c) {
            const float rnd = clHashFloat01(tick * 23u + static_cast<uint32_t>(c) + 211u);
            if (rnd > 0.75f) {
                maxPanelBtnLed(outLeds, kPanelGridTop[c], (rnd - 0.75f) * 3.0f * p * flutter);
            }
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op1)] = 1.0f;
    }

    // 2. OP2 RING MOD: Alternating binary XOR logic checkerboard that flips parity with clock/press
    if (topBlackShow == static_cast<int>(BlackKeyId::Op2)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op2)], 0.0f, 1.0f);
        const bool clkFlip = evalAnalogSine(wrapLfoPhase01(animPhase * (2.5f + 9.5f * p))) < 0.0f;
        const bool pressParity = (p >= 0.55f);
        const bool flip = (clkFlip != pressParity);
        const float hi = 0.30f + 0.70f * p;
        const float lo = 0.03f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const bool lit = (((i & 1u) == 0u) != flip);
            outLeds.whiteKeyLeds[i] = lit ? hi : lo;
        }
        for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
            const bool lit = (((i & 1u) != 0u) != flip);
            outLeds.blackKeyLeds[i] = lit ? hi : lo;
        }
        setPanelBtnLed(outLeds, ButtonId::OctMinus, flip ? lo : hi);
        setPanelBtnLed(outLeds, ButtonId::OctPlus, flip ? hi : lo);
        for (size_t c = 0u; c < 6u; ++c) {
            const bool topLit = (((c & 1u) == 0u) != flip);
            setPanelBtnLed(outLeds, kPanelGridTop[c], topLit ? hi : lo);
            if (c < 4u) {
                setPanelBtnLed(outLeds, kPanelGridBot[c], topLit ? lo : hi);
            }
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op2)] = 1.0f;
    }

    // 3. OP3 TURBO LFO: Accelerating cyclotron wave racing across keys & 2x6 button perimeter
    if (topBlackShow == static_cast<int>(BlackKeyId::Op3)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op3)], 0.0f, 1.0f);
        const float cycPhase = wrapLfoPhase01(animPhase * (2.5f + 19.5f * p * p));
        const float keyHead = cycPhase * 12.0f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            float dist = absf(static_cast<float>(i) - keyHead);
            if (dist > 6.0f) {
                dist = 12.0f - dist;
            }
            const float comet = clampf(1.0f - dist * 0.42f, 0.0f, 1.0f);
            const float glow = 0.05f + (0.45f + 0.50f * p) * comet * comet;
            if (glow > outLeds.whiteKeyLeds[i]) {
                outLeds.whiteKeyLeds[i] = glow;
            }
        }
        constexpr std::array<ButtonId, 10> kButtonLoop = {
            ButtonId::Fx,       ButtonId::Sel, ButtonId::Env, ButtonId::Lfo,  ButtonId::Edit,
            ButtonId::GloRight, ButtonId::Seq, ButtonId::Arp, ButtonId::Save, ButtonId::Home,
        };
        const float loopHead = cycPhase * 10.0f;
        scalePanelGrid(outLeds, 0.25f);
        for (size_t b = 0u; b < kButtonLoop.size(); ++b) {
            float dist = absf(static_cast<float>(b) - loopHead);
            if (dist > 5.0f) {
                dist = 10.0f - dist;
            }
            const float beam = clampf(1.0f - dist * 0.55f, 0.0f, 1.0f);
            maxPanelBtnLed(outLeds, kButtonLoop[b], beam * (0.50f + 0.50f * p));
        }
        maxPanelBtnLed(outLeds, ButtonId::Lfo, 0.65f + 0.35f * p);
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op3)] = 1.0f;
    }

    // 4. OP4 SCREAM: Twin Korg35 resonance peaks sweeping inward + FX & red REC clip flare
    if (topBlackShow == static_cast<int>(BlackKeyId::Op4)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op4)], 0.0f, 1.0f);
        const float shimmer =
            (p > 0.45f) ? (0.92f + 0.08f * evalAnalogSine(wrapLfoPhase01(animPhase * 16.0f)))
                        : 1.0f;
        const float hpPeak = p * 4.5f;
        const float lpPeak = 11.0f - p * 4.5f;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float fi = static_cast<float>(i);
            const float dHp = clampf(1.0f - absf(fi - hpPeak) * 0.62f, 0.0f, 1.0f);
            const float dLp = clampf(1.0f - absf(fi - lpPeak) * 0.62f, 0.0f, 1.0f);
            const float peak = (dHp > dLp) ? dHp : dLp;
            const float pass = (fi >= hpPeak && fi <= lpPeak) ? 0.16f : 0.02f;
            outLeds.whiteKeyLeds[i] = clampf((pass + 0.84f * peak) * shimmer, 0.0f, 1.0f);
        }
        scalePanelGrid(outLeds, 0.18f);
        const float pHp = p * 2.5f;
        const float pLp = 5.0f - p * 2.5f;
        for (size_t c = 0u; c < 6u; ++c) {
            const float fc = static_cast<float>(c);
            const float bHp = clampf(1.0f - absf(fc - pHp) * 0.80f, 0.0f, 1.0f);
            const float bLp = clampf(1.0f - absf(fc - pLp) * 0.80f, 0.0f, 1.0f);
            const float pk = ((bHp > bLp) ? bHp : bLp) * shimmer;
            maxPanelBtnLed(outLeds, kPanelGridTop[c], pk);
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], pk * 0.82f);
            }
        }
        maxPanelBtnLed(outLeds, ButtonId::Fx, 0.65f + 0.35f * p);
        if (p > 0.40f) {
            maxPanelBtnLed(outLeds, ButtonId::Rec,
                           clampf((p - 0.40f) * (1.0f / 0.60f), 0.0f, 1.0f) * shimmer);
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op4)] = 1.0f;
    }

    // 5. OP5 HARD SYNC: Master clock pulse anchored on OSC 1 column radiating L->R across slaves
    if (topBlackShow == static_cast<int>(BlackKeyId::Op5)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op5)], 0.0f, 1.0f);
        const float pulsePos = wrapLfoPhase01(animPhase * (3.5f + 8.5f * p)) * 4.0f;
        for (size_t b = 0u; b < 3u; ++b) {
            outLeds.whiteKeyLeds[b * 4u] = clampf(0.75f + 0.25f * p, 0.0f, 1.0f);
            for (size_t v = 1u; v < 4u; ++v) {
                const float wave =
                    clampf(1.0f - absf(static_cast<float>(v) - pulsePos) * 0.85f, 0.0f, 1.0f);
                outLeds.whiteKeyLeds[b * 4u + v] = 0.06f + 0.90f * p * wave;
            }
        }
        scalePanelGrid(outLeds, 0.20f);
        maxPanelBtnLed(outLeds, ButtonId::OctMinus, 0.70f + 0.30f * p);
        for (size_t c = 0u; c < 6u; ++c) {
            const float wave =
                clampf(1.0f - absf(static_cast<float>(c) * 0.65f - pulsePos) * 0.85f, 0.0f, 1.0f);
            maxPanelBtnLed(outLeds, kPanelGridTop[c], (c == 0u) ? 0.95f : (0.90f * p * wave));
            if (c < 4u) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], (c == 0u) ? 0.85f : (0.80f * p * wave));
            }
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op5)] = 1.0f;
    }

    // 6. OP6 SCATTER: Geiger-counter static sparks crackling randomly across all 42 LEDs
    if (topBlackShow == static_cast<int>(BlackKeyId::Op6)) {
        const float p = clampf(in.blackPressure[static_cast<size_t>(BlackKeyId::Op6)], 0.0f, 1.0f);
        const auto tick = static_cast<uint32_t>(animPhase * 28.0f);
        const float thresh = 0.86f - 0.42f * p;
        for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
            const float r = clHashFloat01(tick * 47u + static_cast<uint32_t>(i) * 13u);
            outLeds.whiteKeyLeds[i] = (r > thresh) ? clampf(0.45f + 0.55f * p, 0.0f, 1.0f)
                                                   : (outLeds.whiteKeyLeds[i] * 0.15f);
        }
        for (size_t i = 0u; i < kNumTopMacroKeys; ++i) {
            const float r = clHashFloat01(tick * 53u + static_cast<uint32_t>(i) * 19u + 7u);
            if (r > thresh) {
                outLeds.blackKeyLeds[i] = 0.50f + 0.50f * p;
            } else {
                outLeds.blackKeyLeds[i] *= 0.20f;
            }
        }
        scalePanelGrid(outLeds, 0.16f);
        for (size_t c = 0u; c < 6u; ++c) {
            const float rT = clHashFloat01(tick * 61u + static_cast<uint32_t>(c) + 31u);
            const float rB = clHashFloat01(tick * 67u + static_cast<uint32_t>(c) + 97u);
            if (rT > thresh) {
                maxPanelBtnLed(outLeds, kPanelGridTop[c], 0.55f + 0.45f * p);
            }
            if (c < 4u && rB > thresh) {
                maxPanelBtnLed(outLeds, kPanelGridBot[c], 0.50f + 0.50f * p);
            }
        }
        outLeds.blackKeyLeds[static_cast<size_t>(BlackKeyId::Op6)] = 1.0f;
    }

    // 7. Bottom Circuit-Bend Shows across all 42 LEDs (LIFO-selected when no top-row show leads):
    //    CROSS-PIN (White 0..3) | FOLD (White 4..7) | CRUSH (White 8..11)
    int botKey = -1;
    uint32_t botSeq = 0u;
    for (size_t k = 0u; k < kMuteWhiteKeyBase; ++k) {
        if (k >= kSceneWhiteKeyBase && k < kSceneWhiteKeyBase + kNumSnapshots && hud.saveArmed) {
            continue;
        }
        if (in.whiteHeld[k] && (botKey < 0 || in.whitePressSeq[k] >= botSeq)) {
            botKey = static_cast<int>(k);
            botSeq = in.whitePressSeq[k];
        }
    }

    if (topBlackShow < 0 && botKey >= 0) {
        const auto wIdx = static_cast<size_t>(botKey);
        const size_t v = wIdx & 3u;
        const float p = clampf(in.whitePressure[wIdx], 0.0f, 1.0f);

        if (wIdx < kBankBWhiteKeyBase) {
            // 7a. CROSS-PIN (1-4): High-voltage arc lightning jumping between shorted columns
            const size_t nbr = (v + 1u) & 3u;
            const float boltPing =
                0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(animPhase * (4.5f + 13.5f * p)));
            const float arcPos = boltPing * 11.0f;
            const auto sparkTick = static_cast<uint32_t>(animPhase * (18.0f + 22.0f * p));
            for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
                const size_t col = i & 3u;
                const bool isPole = (col == v || col == nbr);
                const float poleGlow =
                    isPole ? (0.42f + 0.50f * p * ((col == v) ? boltPing : (1.0f - boltPing)))
                           : 0.03f;
                const float bolt =
                    clampf(1.0f - absf(static_cast<float>(i) - arcPos) * 0.52f, 0.0f, 1.0f) *
                    (0.35f + 0.65f * p);
                const float crackle = (clHashFloat01(sparkTick * 37u + static_cast<uint32_t>(i)) >
                                       (0.88f - 0.28f * p))
                                          ? (0.65f * p)
                                          : 0.0f;
                const float hi = (poleGlow > bolt) ? poleGlow : bolt;
                outLeds.whiteKeyLeds[i] = clampf((hi > crackle) ? hi : crackle, 0.02f, 1.0f);
            }
            for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
                const bool even = ((k & 1u) == 0u);
                outLeds.blackKeyLeds[k] =
                    0.05f + (0.40f + 0.55f * p) * (even ? boltPing : (1.0f - boltPing));
            }
            scalePanelGrid(outLeds, 0.15f);
            setPanelBtnLed(outLeds, ButtonId::OctMinus, 0.20f + 0.80f * p * boltPing);
            setPanelBtnLed(outLeds, ButtonId::OctPlus, 0.20f + 0.80f * p * (1.0f - boltPing));
            const float gridArc = boltPing * 5.0f;
            for (size_t c = 0u; c < 6u; ++c) {
                const float dTop =
                    clampf(1.0f - absf(static_cast<float>(c) - gridArc) * 0.72f, 0.0f, 1.0f);
                const float dBot =
                    clampf(1.0f - absf(static_cast<float>(5u - c) - gridArc) * 0.72f, 0.0f, 1.0f);
                maxPanelBtnLed(outLeds, kPanelGridTop[c], dTop * (0.45f + 0.55f * p));
                if (c < 4u) {
                    maxPanelBtnLed(outLeds, kPanelGridBot[c], dBot * (0.45f + 0.55f * p));
                }
            }
            if (p > 0.55f) {
                maxPanelBtnLed(outLeds, ButtonId::Rec, (p - 0.55f) * (1.0f / 0.45f) * boltPing);
            }
        } else if (wIdx < kSceneWhiteKeyBase) {
            // 7b. FOLD (5-8): Symmetrical standing-wave crests (1 -> 6 folds) mirroring across
            // chassis
            const float foldFreq = 0.08f + 0.34f * p;
            const float breath = animPhase * (1.6f + 4.4f * p);
            for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
                const float dist = absf(static_cast<float>(i) - static_cast<float>(wIdx));
                const float wave =
                    0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(0.25f + dist * foldFreq - breath));
                const float crest = wave * wave;
                outLeds.whiteKeyLeds[i] = 0.03f + (0.38f + 0.59f * p) * crest;
            }
            for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
                const float symDist = absf(static_cast<float>(k) - 2.5f);
                const float wave = 0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(
                                                     0.25f + symDist * (foldFreq * 1.5f) - breath));
                outLeds.blackKeyLeds[k] = 0.04f + (0.35f + 0.60f * p) * wave * wave;
            }
            scalePanelGrid(outLeds, 0.14f);
            const float edgeWave =
                0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(0.25f + 3.0f * foldFreq - breath));
            setPanelBtnLed(outLeds, ButtonId::OctMinus, (0.35f + 0.65f * p) * edgeWave);
            setPanelBtnLed(outLeds, ButtonId::OctPlus, (0.35f + 0.65f * p) * (1.0f - edgeWave));
            for (size_t c = 0u; c < 6u; ++c) {
                const float mirrorDist = absf(static_cast<float>(c) - 2.5f);
                const float wTop =
                    0.5f + 0.5f * evalAnalogSine(wrapLfoPhase01(
                                      0.25f + mirrorDist * (foldFreq * 1.8f) - breath));
                const float wBot = 1.0f - wTop;
                maxPanelBtnLed(outLeds, kPanelGridTop[c], (0.35f + 0.65f * p) * wTop * wTop);
                if (c < 4u) {
                    maxPanelBtnLed(outLeds, kPanelGridBot[c], (0.35f + 0.65f * p) * wBot * wBot);
                }
            }
        } else {
            // 7c. CRUSH (9-12): Live CD4040 binary ripple counter & stepped S&H data bus
            const auto clk = static_cast<uint32_t>(animPhase * (4.0f + 24.0f * p)) + 1u;
            const uint32_t lfsrWord =
                (clk * 0x2C9277B5u) ^ (clk >> 1u) ^ (static_cast<uint32_t>(v) * 0x5555u);
            const float hi = 0.38f + 0.62f * p;
            const float lo = 0.02f;
            for (size_t i = 0u; i < kMuteWhiteKeyBase; ++i) {
                const uint32_t divBit = (clk >> (i & 3u)) & 1u;
                const uint32_t busBit = (lfsrWord >> i) & 1u;
                const bool on = ((p < 0.45f) ? (divBit != 0u) : ((divBit ^ busBit) != 0u));
                outLeds.whiteKeyLeds[i] = on ? hi : lo;
            }
            // Top OP1..OP6 display true 6-stage binary ripple division (/2, /4, /8, /16, /32, /64)
            for (size_t b = 0u; b < kNumTopMacroKeys; ++b) {
                const bool bitOn = ((clk >> b) & 1u) != 0u;
                outLeds.blackKeyLeds[b] = bitOn ? hi : lo;
            }
            scalePanelGrid(outLeds, 0.12f);
            setPanelBtnLed(outLeds, ButtonId::OctMinus, ((clk & 1u) != 0u) ? hi : lo);
            setPanelBtnLed(outLeds, ButtonId::OctPlus, ((clk & 2u) != 0u) ? hi : lo);
            for (size_t c = 0u; c < 6u; ++c) {
                const bool tBit = (((clk + static_cast<uint32_t>(v)) >> (c % 4u)) & 1u) != 0u;
                const bool bBit = (((lfsrWord >> (c + 2u)) & 1u) != 0u);
                setPanelBtnLed(outLeds, kPanelGridTop[c], tBit ? hi : lo);
                if (c < 4u) {
                    setPanelBtnLed(outLeds, kPanelGridBot[c], bBit ? hi : lo);
                }
            }
            if (p > 0.60f && ((clk & 1u) != 0u)) {
                maxPanelBtnLed(outLeds, ButtonId::Rec, (p - 0.60f) * (1.0f / 0.40f));
            }
        }
    }

    // Pin every physically held circuit-bend key (top & bottom) to unmistakable 100% brightness
    for (size_t k = 0u; k < kNumTopMacroKeys; ++k) {
        if (in.blackHeld[k]) {
            outLeds.blackKeyLeds[k] = 1.0f;
        }
    }
    for (size_t k = 0u; k < kMuteWhiteKeyBase; ++k) {
        if (k >= kSceneWhiteKeyBase && k < kSceneWhiteKeyBase + kNumSnapshots && hud.saveArmed) {
            continue;
        }
        if (in.whiteHeld[k]) {
            outLeds.whiteKeyLeds[k] = 1.0f;
        }
    }

    // 8. Universal 11th Key Stasis Freeze + Authoritative binary Console SOLO/MUTE state
    applyFreezeStillFrameLeds(in, outLeds);
    applyAuthoritativeConsoleLeds(state, outLeds);
}

} // namespace ruido
