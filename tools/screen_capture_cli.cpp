#include "core/KeyboardModelSpec.h"
#include "core/RuidoController.h"
#include "core/SnapshotBank.h"
#include "dsp/DroneEngine.h"
#include "ui/FontSansAA.h"
#include "ui/HudKeybedView.h"
#include "ui/MonoCanvas.h"
#include "ui/ScopeHudRenderer.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

namespace {

constexpr uint16_t kW = ruido::kDisplayWidth;
constexpr uint16_t kH = ruido::kDisplayHeight;

bool parseButton(const std::string& s, ruido::ButtonId& out) {
    if (s == "OctMinus") {
        out = ruido::ButtonId::OctMinus;
        return true;
    }
    if (s == "OctPlus") {
        out = ruido::ButtonId::OctPlus;
        return true;
    }
    if (s == "Fx") {
        out = ruido::ButtonId::Fx;
        return true;
    }
    if (s == "Sel") {
        out = ruido::ButtonId::Sel;
        return true;
    }
    if (s == "Env") {
        out = ruido::ButtonId::Env;
        return true;
    }
    if (s == "Lfo") {
        out = ruido::ButtonId::Lfo;
        return true;
    }
    if (s == "Edit") {
        out = ruido::ButtonId::Edit;
        return true;
    }
    if (s == "GloRight") {
        out = ruido::ButtonId::GloRight;
        return true;
    }
    if (s == "Home") {
        out = ruido::ButtonId::Home;
        return true;
    }
    if (s == "Save") {
        out = ruido::ButtonId::Save;
        return true;
    }
    if (s == "Arp") {
        out = ruido::ButtonId::Arp;
        return true;
    }
    if (s == "Seq") {
        out = ruido::ButtonId::Seq;
        return true;
    }
    if (s == "PlayStop") {
        out = ruido::ButtonId::PlayStop;
        return true;
    }
    if (s == "Rec") {
        out = ruido::ButtonId::Rec;
        return true;
    }
    return false;
}

bool parseEncoder(const std::string& s, ruido::EncoderId& out) {
    if (s == "Select") {
        out = ruido::EncoderId::Select;
        return true;
    }
    if (s == "Presets") {
        out = ruido::EncoderId::Presets;
        return true;
    }
    if (s == "Algorithm") {
        out = ruido::EncoderId::Algorithm;
        return true;
    }
    if (s == "Knob1") {
        out = ruido::EncoderId::Knob1;
        return true;
    }
    if (s == "Knob2") {
        out = ruido::EncoderId::Knob2;
        return true;
    }
    if (s == "Knob3") {
        out = ruido::EncoderId::Knob3;
        return true;
    }
    if (s == "Knob4") {
        out = ruido::EncoderId::Knob4;
        return true;
    }
    return false;
}

bool parseBlackKey(const std::string& s, ruido::BlackKeyId& out) {
    if (s == "Op1") {
        out = ruido::BlackKeyId::Op1;
        return true;
    }
    if (s == "Op2") {
        out = ruido::BlackKeyId::Op2;
        return true;
    }
    if (s == "Op3") {
        out = ruido::BlackKeyId::Op3;
        return true;
    }
    if (s == "Op4") {
        out = ruido::BlackKeyId::Op4;
        return true;
    }
    if (s == "Op5") {
        out = ruido::BlackKeyId::Op5;
        return true;
    }
    if (s == "Op6") {
        out = ruido::BlackKeyId::Op6;
        return true;
    }
    if (s == "Pit") {
        out = ruido::BlackKeyId::Pit;
        return true;
    }
    if (s == "Glo") {
        out = ruido::BlackKeyId::Glo;
        return true;
    }
    if (s == "Mono") {
        out = ruido::BlackKeyId::Mono;
        return true;
    }
    if (s == "Poly") {
        out = ruido::BlackKeyId::Poly;
        return true;
    }
    if (s == "ScopeFreezeKey") {
        out = ruido::BlackKeyId::ScopeFreezeKey;
        return true;
    }
    return false;
}

bool parseLfoTarget(const std::string& s, ruido::LfoTarget& out) {
    if (s == "Pitch") {
        out = ruido::LfoTarget::Pitch;
        return true;
    }
    if (s == "PulseWidth") {
        out = ruido::LfoTarget::PulseWidth;
        return true;
    }
    if (s == "LpCutoff") {
        out = ruido::LfoTarget::LpCutoff;
        return true;
    }
    if (s == "LpResonance") {
        out = ruido::LfoTarget::LpResonance;
        return true;
    }
    if (s == "HpCutoff") {
        out = ruido::LfoTarget::HpCutoff;
        return true;
    }
    if (s == "HpResonance") {
        out = ruido::LfoTarget::HpResonance;
        return true;
    }
    if (s == "Amplitude") {
        out = ruido::LfoTarget::Amplitude;
        return true;
    }
    if (s == "Pan") {
        out = ruido::LfoTarget::Pan;
        return true;
    }
    if (s == "LfoARate") {
        out = ruido::LfoTarget::LfoARate;
        return true;
    }
    if (s == "LfoADepth") {
        out = ruido::LfoTarget::LfoADepth;
        return true;
    }
    return false;
}

struct Harness {
    ruido::DroneEngine engine{};
    ruido::RuidoController ctrl{};
    ruido::ScopeHudRenderer renderer{};
    std::array<float, ruido::kBlockFrames> outL{};
    std::array<float, ruido::kBlockFrames> outR{};
    std::array<uint16_t, kW * kH> fb{};

    void initColdBoot() {
        engine.init(ruido::kBaseSampleRate);
        ctrl.reset();
        ctrl.attachControlBus(&engine.controlBus());
        engine.snapParametersToLatest();
        renderer.reset();
    }

    void recallSceneClean(int scene1Based) {
        const auto keyIdx = static_cast<uint8_t>(ruido::kSceneWhiteKeyBase +
                                                 static_cast<size_t>((scene1Based - 1) & 3));
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, true);
        ctrl.onWhiteKeyPressure(keyIdx, 1.0f);
        step(16, 1.0f / 60.0f, 12);
        ctrl.onWhiteKeyGate(keyIdx, false);
        ctrl.onBlackKeyEvent(ruido::BlackKeyId::ScopeFreezeKey, false);
        ctrl.onButtonEvent(ruido::ButtonId::Home, true);
        ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        engine.snapParametersToLatest();
    }

    void initPlaying(int settleFrames, int scene1Based) {
        initColdBoot();
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, true);
        ctrl.onButtonEvent(ruido::ButtonId::PlayStop, false);
        if (scene1Based >= 2 && scene1Based <= 4) {
            recallSceneClean(scene1Based);
        } else {
            ctrl.onButtonEvent(ruido::ButtonId::Home, true);
            ctrl.onButtonEvent(ruido::ButtonId::Home, false);
        }
        engine.snapParametersToLatest();
        step(settleFrames, 1.0f / 60.0f, 12);
    }

    void step(int frames, float dt, int dspBlocks) {
        for (int f = 0; f < frames; ++f) {
            for (int b = 0; b < dspBlocks; ++b) {
                engine.processBlock(outL.data(), outR.data(), ruido::kBlockFrames);
                renderer.ingestScopeRing(engine.scopeBuffer());
            }
            ctrl.tick(dt, engine.telemetry());
            renderer.prepareFrame(dt, ctrl.state(), ctrl.hud(), engine.telemetry());
        }
    }

    void focusKnobExact(ruido::EncoderId enc) {
        const ruido::PatchSnapshot saved = ctrl.exportLivePatch();
        ctrl.onEncoderDelta(enc, 1);
        ctrl.onEncoderDelta(enc, -1);
        ctrl.loadLivePatch(saved);
        engine.snapParametersToLatest();
    }

    void dumpGenericKeyboardRgb(const std::string& path) {
        std::array<uint8_t, kW * ruido::kScanlineStripHeight> lumStrip{};
        std::array<float, ruido::kNumBlackKeys> blackGlow{};
        std::array<float, ruido::kNumWhiteKeys> whiteGlow{};
        blackGlow[ruido::kHoldBlackKeyIdx] = ruido::kLedAssignedRest;
        blackGlow[ruido::kShiftBlackKeyIdx] = 1.0f;
        for (size_t i = ruido::kMuteWhiteKeyBase; i < ruido::kNumWhiteKeys; ++i) {
            whiteGlow[i] = 0.80f;
        }
        constexpr uint16_t kActiveBlack = static_cast<uint16_t>((1u << ruido::kHoldBlackKeyIdx) |
                                                                (1u << ruido::kShiftBlackKeyIdx));
        constexpr uint16_t kActiveWhite = 0xF000u;

        for (uint16_t y0 = 0u; y0 < kH;
             y0 = static_cast<uint16_t>(y0 + ruido::kScanlineStripHeight)) {
            const int iy0 = static_cast<int>(y0);
            const int iy1 = iy0 + static_cast<int>(ruido::kScanlineStripHeight);
            lumStrip.fill(0u);
            const ruido::MonoStripCanvas cv{lumStrip.data(), iy0, iy1, 255u};

            // Top header matching KEYBOARD screens (`KEYBOARD` + `STEREO`)
            if (iy0 < 24) {
                cv.drawSans(ruido::kFontSansBold, 10, 6, "KEYBOARD", 255u, 0);
                cv.drawSansRight(ruido::kFontSansSmall, 230, 6, "STEREO", 168u, 1);
            }

            // Right-bay top callouts: LATCH & SHIFT only
            if (iy1 > 70 && iy0 < 92) {
                const float latchKeyX = ruido::blackKeyDiagramCenterX(ruido::kHoldBlackKeyIdx);
                const float latchLum = 56.0f + 92.0f * blackGlow[ruido::kHoldBlackKeyIdx];
                const float shiftKeyX = ruido::blackKeyDiagramCenterX(ruido::kShiftBlackKeyIdx);
                const float shiftLum = 56.0f + 92.0f * blackGlow[ruido::kShiftBlackKeyIdx];
                cv.drawSansCentered(ruido::kFontSansSmall, static_cast<int>(latchKeyX + 0.5f), 72,
                                    "LATCH", 214u, 0);
                cv.drawVerticalSlit(latchKeyX, 84.5f, 89.6f, latchLum);
                cv.drawSansCentered(ruido::kFontSansSmall, static_cast<int>(shiftKeyX + 0.5f), 72,
                                    "SHIFT", 214u, 0);
                cv.drawVerticalSlit(shiftKeyX, 84.5f, 89.6f, shiftLum);
            }

            // Shared Two-Bay 20+7 hardware diagram with only LATCH, SHIFT & MUTE 1-4 active
            ruido::compositeFm1KeybedDiagram(cv, ruido::kKeybedDiagramTopY, blackGlow, whiteGlow,
                                             nullptr, kActiveBlack, kActiveWhite);

            // Right-bay bottom bracket & label: MUTE only
            if (iy1 > 166 && iy0 < 192) {
                const float xL = ruido::whiteKeyDiagramCenterX(ruido::kMuteWhiteKeyBase) -
                                 ruido::kKeybedBracketOverhangX;
                const float xR = ruido::whiteKeyDiagramCenterX(ruido::kNumWhiteKeys - 1u) +
                                 ruido::kKeybedBracketOverhangX;
                const float cxF = 0.5f * (xL + xR);
                const float brkLum = 56.0f + 92.0f * 0.80f;
                cv.drawHorizontalFilament(xL, xR, ruido::kKeybedBracketY, brkLum, 0.58f, true);
                cv.drawVerticalSlit(xL, ruido::kKeybedBracketTickTopY, ruido::kKeybedBracketY,
                                    brkLum);
                cv.drawVerticalSlit(xR, ruido::kKeybedBracketTickTopY, ruido::kKeybedBracketY,
                                    brkLum);
                cv.drawSansCentered(ruido::kFontSansSmall, static_cast<int>(cxF + 0.5f),
                                    ruido::kKeybedBracketLabelY, "MUTE", 222u, 0);
            }

            for (int sy = 0; sy < static_cast<int>(ruido::kScanlineStripHeight); ++sy) {
                for (int sx = 0; sx < static_cast<int>(kW); ++sx) {
                    const uint8_t lum =
                        lumStrip[static_cast<size_t>(sy) * kW + static_cast<size_t>(sx)];
                    fb[static_cast<size_t>(iy0 + sy) * kW + static_cast<size_t>(sx)] =
                        ruido::packNeutralGrayRgb565(lum, sx, iy0 + sy);
                }
            }
        }
        writeFbAsRawRgb(path);
    }

    void dumpRawRgb(const std::string& path) {
        renderer.renderFullScreenStrips(fb.data());
        writeFbAsRawRgb(path);
    }

    void writeFbAsRawRgb(const std::string& path) {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) {
            std::perror(path.c_str());
            std::exit(2);
        }
        for (uint16_t px : fb) {
            const auto r5 = static_cast<uint8_t>((px >> 11u) & 0x1Fu);
            const auto g6 = static_cast<uint8_t>((px >> 5u) & 0x3Fu);
            const auto b5 = static_cast<uint8_t>(px & 0x1Fu);
            // Match ST7789 TFT RGB565 -> RGB666 -> RGB888 LUT (bit-identical to wasm_bridge.cpp)
            const auto r6 = static_cast<uint8_t>((r5 << 1u) | (r5 >> 4u));
            const auto b6 = static_cast<uint8_t>((b5 << 1u) | (b5 >> 4u));
            const uint8_t rgb[3] = {
                static_cast<uint8_t>((r6 << 2u) | (r6 >> 4u)),
                static_cast<uint8_t>((g6 << 2u) | (g6 >> 4u)),
                static_cast<uint8_t>((b6 << 2u) | (b6 >> 4u)),
            };
            std::fwrite(rgb, 1, 3, f);
        }
        std::fclose(f);
    }
};

} // namespace

int main() {
    static Harness h{};
    char rawBuf[2048];
    int lineNo = 0;
    while (std::fgets(rawBuf, sizeof(rawBuf), stdin) != nullptr) {
        std::string line(rawBuf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
            line.pop_back();
        }
        ++lineNo;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        if (cmd == "INIT_BOOT") {
            h.initColdBoot();
        } else if (cmd == "INIT_PLAYING") {
            int frames = 60, scene = 1;
            iss >> frames >> scene;
            h.initPlaying(frames, scene);
        } else if (cmd == "ARM_BOOT_INTRO") {
            h.renderer.armBootIntro();
        } else if (cmd == "RECALL_SCENE") {
            int scene = 1;
            iss >> scene;
            h.recallSceneClean(scene);
        } else if (cmd == "STEP") {
            int frames = 1, blocks = 12;
            float dt = 1.0f / 60.0f;
            iss >> frames >> dt >> blocks;
            h.step(frames, dt, blocks);
        } else if (cmd == "TAP") {
            std::string btnName;
            iss >> btnName;
            ruido::ButtonId id{};
            if (!parseButton(btnName, id)) {
                std::fprintf(stderr, "Line %d: unknown button '%s'\n", lineNo, btnName.c_str());
                return 1;
            }
            h.ctrl.onButtonEvent(id, true);
            h.ctrl.onButtonEvent(id, false);
        } else if (cmd == "BUTTON") {
            std::string btnName;
            int pressed = 0;
            iss >> btnName >> pressed;
            ruido::ButtonId id{};
            if (!parseButton(btnName, id)) {
                std::fprintf(stderr, "Line %d: unknown button '%s'\n", lineNo, btnName.c_str());
                return 1;
            }
            h.ctrl.onButtonEvent(id, pressed != 0);
        } else if (cmd == "TURN") {
            std::string encName;
            int32_t ticks = 0;
            iss >> encName >> ticks;
            ruido::EncoderId id{};
            if (!parseEncoder(encName, id)) {
                std::fprintf(stderr, "Line %d: unknown encoder '%s'\n", lineNo, encName.c_str());
                return 1;
            }
            h.ctrl.onEncoderDelta(id, ticks);
        } else if (cmd == "FOCUS_KNOB") {
            std::string encName;
            iss >> encName;
            ruido::EncoderId id{};
            if (!parseEncoder(encName, id)) {
                std::fprintf(stderr, "Line %d: unknown knob '%s'\n", lineNo, encName.c_str());
                return 1;
            }
            h.focusKnobExact(id);
        } else if (cmd == "BLACK_KEY") {
            std::string keyName;
            int pressed = 1;
            float pressure = 1.0f;
            iss >> keyName >> pressed >> pressure;
            ruido::BlackKeyId id{};
            if (!parseBlackKey(keyName, id)) {
                std::fprintf(stderr, "Line %d: unknown black key '%s'\n", lineNo, keyName.c_str());
                return 1;
            }
            if (pressed) {
                h.ctrl.onBlackKeyEvent(id, true);
                h.ctrl.onBlackKeyPressure(id, pressure);
            } else {
                h.ctrl.onBlackKeyPressure(id, 0.0f);
                h.ctrl.onBlackKeyEvent(id, false);
            }
        } else if (cmd == "WHITE_KEY") {
            int idx = 0, pressed = 1;
            float pressure = 1.0f;
            iss >> idx >> pressed >> pressure;
            const auto wIdx = static_cast<uint8_t>(idx & 15);
            if (pressed) {
                h.ctrl.onWhiteKeyGate(wIdx, true);
                h.ctrl.onWhiteKeyPressure(wIdx, pressure);
            } else {
                h.ctrl.onWhiteKeyPressure(wIdx, 0.0f);
                h.ctrl.onWhiteKeyGate(wIdx, false);
            }
        } else if (cmd == "MASTER_POT") {
            float val = 0.85f;
            iss >> val;
            h.ctrl.onMasterPot(val);
        } else if (cmd == "SET_COUPLING") {
            float val = 0.25f;
            iss >> val;
            ruido::EngineControlState st = h.ctrl.state();
            st.global.circuitCoupling = val;
            h.ctrl.loadLivePatch(ruido::encodePatchSnapshot(st));
            h.engine.snapParametersToLatest();
        } else if (cmd == "SET_VOICE") {
            int v1 = 1;
            iss >> v1;
            const auto vi = static_cast<size_t>((v1 - 1) & 3);
            ruido::EngineControlState st = h.ctrl.state();
            ruido::VoiceParams& vp = st.voices[vi];
            std::string kv;
            while (iss >> kv) {
                const size_t eq = kv.find('=');
                if (eq == std::string::npos) {
                    continue;
                }
                const std::string k = kv.substr(0, eq);
                const std::string v = kv.substr(eq + 1);
                if (k == "freq_hz")
                    vp.frequencyHz = std::strtof(v.c_str(), nullptr);
                else if (k == "duty")
                    vp.pulseWidth = std::strtof(v.c_str(), nullptr);
                else if (k == "drift")
                    vp.drift = std::strtof(v.c_str(), nullptr);
                else if (k == "hp_hz")
                    vp.hpCutoffHz = std::strtof(v.c_str(), nullptr);
                else if (k == "hp_res")
                    vp.hpResonance = std::strtof(v.c_str(), nullptr);
                else if (k == "lp_hz")
                    vp.lpCutoffHz = std::strtof(v.c_str(), nullptr);
                else if (k == "lp_res")
                    vp.lpResonance = std::strtof(v.c_str(), nullptr);
                else if (k == "amp")
                    vp.amplitude = std::strtof(v.c_str(), nullptr);
                else if (k == "pan")
                    vp.pan = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_rate_hz")
                    vp.lfoRateHz = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_shape")
                    vp.lfoShape = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_depth")
                    vp.lfoDepth = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_b_rate_hz")
                    vp.lfoBRateHz = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_b_shape")
                    vp.lfoBShape = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_b_depth")
                    vp.lfoBDepth = std::strtof(v.c_str(), nullptr);
                else if (k == "lfo_target") {
                    ruido::LfoTarget t{};
                    if (parseLfoTarget(v, t))
                        vp.lfoTarget = t;
                } else if (k == "lfo_b_target") {
                    ruido::LfoTarget t{};
                    if (parseLfoTarget(v, t))
                        vp.lfoBTarget = t;
                } else if (k == "muted") {
                    vp.muted = (v == "1" || v == "true");
                }
            }
            h.ctrl.loadLivePatch(ruido::encodePatchSnapshot(st));
            h.engine.snapParametersToLatest();
        } else if (cmd == "DUMP_GENERIC_KEYBOARD") {
            std::string path;
            iss >> path;
            h.dumpGenericKeyboardRgb(path);
        } else if (cmd == "DUMP") {
            std::string path;
            iss >> path;
            h.dumpRawRgb(path);
        } else {
            std::fprintf(stderr, "Line %d: unknown command '%s'\n", lineNo, cmd.c_str());
            return 1;
        }
    }
    return 0;
}
