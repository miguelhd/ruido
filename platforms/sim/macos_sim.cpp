// Native macOS CoreAudio (AudioQueue) real-time hardware auditioning host for Ruido.
// Streams 44,118 Hz 4x-oversampled stereo float32 audio directly to the default macOS
// audio interface for patching into external guitar pedals and Eurorack rigs.

#include "core/RuidoController.h"
#include "dsp/DroneEngine.h"
#include "ui/ScopeHudRenderer.h"

#include <AudioToolbox/AudioQueue.h>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr uint32_t kNumQueueBuffers = 4u;
constexpr uint32_t kQueueBufferFrames = 128u; // 2 x 64-frame FM-1 blocks (~2.9 ms)

struct CoreAudioContext {
    ruido::DroneEngine engine{};
    std::array<float, kQueueBufferFrames> scratchL{};
    std::array<float, kQueueBufferFrames> scratchR{};
};

std::atomic<bool> gKeepRunning{true};

void handleSignal(int) {
    gKeepRunning.store(false, std::memory_order_relaxed);
}

void audioQueueCallback(void* userData, AudioQueueRef queue, AudioQueueBufferRef buffer) {
    auto* ctx = static_cast<CoreAudioContext*>(userData);
    auto* interleaved = static_cast<float*>(buffer->mAudioData);
    const uint32_t frames = kQueueBufferFrames;

    size_t offset = 0u;
    while (offset < frames) {
        const size_t chunk =
            (frames - offset > ruido::kBlockFrames) ? ruido::kBlockFrames : (frames - offset);
        ctx->engine.processBlock(ctx->scratchL.data() + offset, ctx->scratchR.data() + offset,
                                 chunk);
        offset += chunk;
    }

    for (uint32_t i = 0u; i < frames; ++i) {
        interleaved[i * 2u + 0u] = ctx->scratchL[i];
        interleaved[i * 2u + 1u] = ctx->scratchR[i];
    }
    buffer->mAudioDataByteSize = frames * 2u * sizeof(float);

    if (gKeepRunning.load(std::memory_order_relaxed)) {
        AudioQueueEnqueueBuffer(queue, buffer, 0, nullptr);
    }
}

struct RawTerminalGuard {
    termios orig{};
    int origFlags = 0;
    bool active = false;

    RawTerminalGuard() {
        if (isatty(STDIN_FILENO) == 0) {
            return;
        }
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            termios raw = orig;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            origFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, origFlags | O_NONBLOCK);
            active = true;
        }
    }

    ~RawTerminalGuard() {
        if (active) {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
            fcntl(STDIN_FILENO, F_SETFL, origFlags);
        }
    }
};

} // namespace

int main(int argc, char** argv) {
    static CoreAudioContext audioCtx{};
    ruido::RuidoController controller{};
    static ruido::ScopeHudRenderer renderer{};

    audioCtx.engine.init(ruido::kBaseSampleRate);
    controller.attachControlBus(&audioCtx.engine.controlBus());
    audioCtx.engine.snapParametersToLatest();

    // Headless smoke-test mode for automated build verification
    if (argc > 1 && std::strcmp(argv[1], "--headless-check") == 0) {
        std::array<uint16_t, ruido::kDisplayWidth * ruido::kScanlineStripHeight> strip{};
        audioCtx.engine.processBlock(audioCtx.scratchL.data(), audioCtx.scratchR.data(),
                                     ruido::kBlockFrames);
        renderer.ingestScopeRing(audioCtx.engine.scopeBuffer());
        controller.tick(0.016f, audioCtx.engine.telemetry());
        renderer.prepareFrame(0.016f, controller.state(), controller.hud(),
                              audioCtx.engine.telemetry());
        renderer.renderStrip(0u, ruido::kScanlineStripHeight, strip.data());
        std::printf("ruido_sim_macos headless check OK (strip[0]=0x%04X)\n", strip[0]);
        return 0;
    }

    renderer.armBootIntro();
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    AudioStreamBasicDescription fmt{};
    fmt.mSampleRate = static_cast<double>(ruido::kBaseSampleRate);
    fmt.mFormatID = kAudioFormatLinearPCM;
    fmt.mFormatFlags = kLinearPCMFormatFlagIsFloat | kLinearPCMFormatFlagIsPacked;
    fmt.mBytesPerPacket = 2u * sizeof(float);
    fmt.mFramesPerPacket = 1u;
    fmt.mBytesPerFrame = 2u * sizeof(float);
    fmt.mChannelsPerFrame = 2u;
    fmt.mBitsPerChannel = 32u;

    AudioQueueRef queue = nullptr;
    OSStatus status =
        AudioQueueNewOutput(&fmt, audioQueueCallback, &audioCtx, nullptr, nullptr, 0, &queue);
    if (status != noErr || queue == nullptr) {
        std::fprintf(stderr, "Failed to open macOS CoreAudio output (OSStatus=%d)\n",
                     static_cast<int>(status));
        return 1;
    }

    const uint32_t bufBytes = kQueueBufferFrames * 2u * sizeof(float);
    for (uint32_t i = 0u; i < kNumQueueBuffers; ++i) {
        AudioQueueBufferRef buf = nullptr;
        AudioQueueAllocateBuffer(queue, bufBytes, &buf);
        audioQueueCallback(&audioCtx, queue, buf);
    }

    status = AudioQueueStart(queue, nullptr);
    if (status != noErr) {
        std::fprintf(stderr, "Failed to start macOS CoreAudio queue (OSStatus=%d)\n",
                     static_cast<int>(status));
        AudioQueueDispose(queue, true);
        return 1;
    }

    std::printf("=== Ruido Native macOS CoreAudio Auditioner (44,118 Hz / 4x OS) ===\n");
    std::printf("  [j] FX Filter  | [b] SEL Model  | [k] EDIT/PAN   | [l] LFO | [;] CONFIG\n");
    std::printf("  [1-6,g,m,0] Top Key Punch-Ins   | [,/.] SELECT   | [h] HOME\n");
    std::printf("  [q/w/e/r] KNOB 1-4 +            | [u/i/o/p] KNOB 1-4 -\n");
    std::printf("  [z/x/c/v] Kill Cut 1-4          | [a/s/d/f] Formant Bite 1-4\n");
    std::printf("  [ [ / ] ] ALGORITHM Coupling    | [ - / = ] Knob Rate (Fine/Coarse)\n");
    std::printf("  [Space] PLAY/STOP               | [ESC] or [Q] Quit\n\n");

    RawTerminalGuard termGuard{};
    int latchedMod = -1;
    int latchedRateBtn = -1;
    std::array<uint16_t, ruido::kDisplayWidth * ruido::kScanlineStripHeight> stripBuf{};

    while (gKeepRunning.load(std::memory_order_relaxed)) {
        char ch = 0;
        while (read(STDIN_FILENO, &ch, 1) == 1) {
            if (ch == 27 || ch == 'Q') {
                gKeepRunning.store(false, std::memory_order_relaxed);
                break;
            }
            const auto toggleMod = [&](ruido::BlackKeyId key) {
                const int k = static_cast<int>(key);
                if (latchedMod == k) {
                    controller.onBlackKeyEvent(key, false);
                    latchedMod = -1;
                } else {
                    if (latchedMod >= 0) {
                        controller.onBlackKeyEvent(static_cast<ruido::BlackKeyId>(latchedMod),
                                                   false);
                    }
                    controller.onBlackKeyEvent(key, true);
                    latchedMod = k;
                }
            };
            const auto toggleRateHold = [&](ruido::ButtonId btn) {
                const int b = static_cast<int>(btn);
                if (latchedRateBtn == b) {
                    controller.onButtonEvent(btn, false);
                    latchedRateBtn = -1;
                } else {
                    if (latchedRateBtn >= 0) {
                        controller.onButtonEvent(static_cast<ruido::ButtonId>(latchedRateBtn),
                                                 false);
                    }
                    controller.onButtonEvent(btn, true);
                    latchedRateBtn = b;
                }
            };

            switch (ch) {
            case '1':
                toggleMod(ruido::BlackKeyId::Op1);
                break;
            case '2':
                toggleMod(ruido::BlackKeyId::Op2);
                break;
            case '3':
                toggleMod(ruido::BlackKeyId::Op3);
                break;
            case '4':
                toggleMod(ruido::BlackKeyId::Op4);
                break;
            case '5':
                toggleMod(ruido::BlackKeyId::Op5);
                break;
            case '6':
                toggleMod(ruido::BlackKeyId::Op6);
                break;
            case 'g':
                toggleMod(ruido::BlackKeyId::Glo);
                break;
            case 'q':
                controller.onEncoderDelta(ruido::EncoderId::Knob1, 8);
                break;
            case 'w':
                controller.onEncoderDelta(ruido::EncoderId::Knob2, 8);
                break;
            case 'e':
                controller.onEncoderDelta(ruido::EncoderId::Knob3, 8);
                break;
            case 'r':
                controller.onEncoderDelta(ruido::EncoderId::Knob4, 8);
                break;
            case 'u':
                controller.onEncoderDelta(ruido::EncoderId::Knob1, -8);
                break;
            case 'i':
                controller.onEncoderDelta(ruido::EncoderId::Knob2, -8);
                break;
            case 'o':
                controller.onEncoderDelta(ruido::EncoderId::Knob3, -8);
                break;
            case 'p':
                controller.onEncoderDelta(ruido::EncoderId::Knob4, -8);
                break;
            case '[':
                controller.onEncoderDelta(ruido::EncoderId::Algorithm, -8);
                break;
            case ']':
                controller.onEncoderDelta(ruido::EncoderId::Algorithm, 8);
                break;
            case '-':
                toggleRateHold(ruido::ButtonId::OctMinus);
                break;
            case '=':
                toggleRateHold(ruido::ButtonId::OctPlus);
                break;
            case 'j':
                controller.onButtonEvent(ruido::ButtonId::Fx, true);
                controller.onButtonEvent(ruido::ButtonId::Fx, false);
                break;
            case 'b':
                controller.onButtonEvent(ruido::ButtonId::Sel, true);
                controller.onButtonEvent(ruido::ButtonId::Sel, false);
                break;
            case 'k':
                controller.onButtonEvent(ruido::ButtonId::Edit, true);
                controller.onButtonEvent(ruido::ButtonId::Edit, false);
                break;
            case 'l':
                controller.onButtonEvent(ruido::ButtonId::Lfo, true);
                controller.onButtonEvent(ruido::ButtonId::Lfo, false);
                break;
            case ';':
                controller.onButtonEvent(ruido::ButtonId::GloRight, true);
                controller.onButtonEvent(ruido::ButtonId::GloRight, false);
                break;
            case 'h':
                controller.onButtonEvent(ruido::ButtonId::Home, true);
                controller.onButtonEvent(ruido::ButtonId::Home, false);
                break;
            case ',':
                controller.onEncoderDelta(ruido::EncoderId::Select, -4);
                break;
            case '.':
                controller.onEncoderDelta(ruido::EncoderId::Select, 4);
                break;
            case 'm':
                toggleMod(ruido::BlackKeyId::Mono);
                break;
            case '0':
                toggleMod(ruido::BlackKeyId::ScopeFreezeKey);
                break;
            case ' ':
                controller.onButtonEvent(ruido::ButtonId::PlayStop, true);
                controller.onButtonEvent(ruido::ButtonId::PlayStop, false);
                break;
            case 'z':
            case 'x':
            case 'c':
            case 'v': {
                const uint8_t idx = (ch == 'z') ? 0u : (ch == 'x') ? 1u : (ch == 'c') ? 2u : 3u;
                const bool cur = (controller.hud().heldWhiteMask & (1u << idx)) != 0u;
                controller.onWhiteKeyPressure(idx, cur ? 0.0f : 0.85f);
                break;
            }
            case 'a':
            case 's':
            case 'd':
            case 'f': {
                const uint8_t idx = (ch == 'a') ? 4u : (ch == 's') ? 5u : (ch == 'd') ? 6u : 7u;
                const bool cur = (controller.hud().heldWhiteMask & (1u << idx)) != 0u;
                controller.onWhiteKeyPressure(idx, cur ? 0.0f : 0.78f);
                break;
            }
            default:
                break;
            }
        }

        renderer.ingestScopeRing(audioCtx.engine.scopeBuffer());
        controller.tick(0.016f, audioCtx.engine.telemetry());
        renderer.prepareFrame(0.016f, controller.state(), controller.hud(),
                              audioCtx.engine.telemetry());
        for (uint16_t y = 0u; y < ruido::kDisplayHeight;
             y = static_cast<uint16_t>(y + ruido::kScanlineStripHeight)) {
            renderer.renderStrip(y, ruido::kScanlineStripHeight, stripBuf.data());
        }

        const auto& st = controller.state();
        const float pL = audioCtx.engine.telemetry().peakLeft.load(std::memory_order_relaxed);
        const float pR = audioCtx.engine.telemetry().peakRight.load(std::memory_order_relaxed);
        std::printf("\r %s | O1:%5.1f O2:%5.1f O3:%5.1f O4:%5.1f Hz | SAG:%3.0f%% | L:%.2f R:%.2f ",
                    st.global.masterRun ? "RUN " : "MUTE",
                    static_cast<double>(st.voices[0].frequencyHz),
                    static_cast<double>(st.voices[1].frequencyHz),
                    static_cast<double>(st.voices[2].frequencyHz),
                    static_cast<double>(st.voices[3].frequencyHz),
                    static_cast<double>(st.global.circuitCoupling * 100.0f),
                    static_cast<double>(pL), static_cast<double>(pR));
        std::fflush(stdout);
        usleep(16000);
    }

    std::printf("\nStopping CoreAudio engine...\n");
    AudioQueueStop(queue, true);
    AudioQueueDispose(queue, true);
    return 0;
}
