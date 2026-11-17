// WebAssembly simulator bridge for Ruido.
// Compiles the exact portable C++17 firmware engine (`src/core/`, `src/dsp/`, `src/ui/`)
// as a zero-dependency freestanding Wasm module instantiable in both AudioWorklet (Core 1)
// and Main UI Thread (Core 0) without Emscripten glue or external runtime imports.

#include "WasmAbiLayout.h"

#include <cstddef>
#include <cstdint>

// Placement new for zero-static-ctor deterministic initialization in freestanding Wasm
inline void* operator new(size_t, void* p) noexcept {
    return p;
}

extern "C" {

void* memcpy(void* dst, const void* src, size_t n) {
    auto* d = static_cast<uint8_t*>(dst);
    const auto* s = static_cast<const uint8_t*>(src);
    for (size_t i = 0u; i < n; ++i) {
        d[i] = s[i];
        __asm__ volatile("" ::: "memory");
    }
    return dst;
}

void* memset(void* dst, int c, size_t n) {
    auto* d = static_cast<uint8_t*>(dst);
    const auto v = static_cast<uint8_t>(c);
    for (size_t i = 0u; i < n; ++i) {
        d[i] = v;
        __asm__ volatile("" ::: "memory");
    }
    return dst;
}

size_t strlen(const char* s) {
    size_t n = 0u;
    while (s[n] != '\0') {
        ++n;
        __asm__ volatile("" ::: "memory");
    }
    return n;
}

} // extern "C"

// Unity-include the 100% platform-independent Ruido C++17 sources
#include "core/ControllerEncoder.cpp"
#include "core/ControllerPageNav.cpp"
#include "core/KeyboardCircuitLab.cpp"
#include "core/KeyboardEngine.cpp"
#include "core/KeyboardLiveMixer.cpp"
#include "core/KeyboardRhythmLab.cpp"
#include "core/LedEngine.cpp"
#include "core/RuidoController.cpp"
#include "core/SnapshotBank.cpp"
#include "core/SnapshotCodec.cpp"
#include "core/SnapshotMorph.cpp"
#include "dsp/DroneEngine.cpp"
#include "dsp/HalfBandDecimator.cpp"
#include "dsp/SallenKeyFilter.cpp"
#include "dsp/SchmittOscillatorBank.cpp"
#include "dsp/VoiceLfoBank.cpp"
#include "ui/BootIntroRender.cpp"
#include "ui/HudAmpGraph.cpp"
#include "ui/HudChrome.cpp"
#include "ui/HudGlobalViews.cpp"
#include "ui/HudGraphs.cpp"
#include "ui/HudHeader.cpp"
#include "ui/HudKeybedView.cpp"
#include "ui/HudSceneView.cpp"
#include "ui/ScopeHudRenderer.cpp"
#include "ui/ScopeTraceEngine.cpp"
#include "ui/ScopeTraceRender.cpp"

namespace {

constexpr size_t kMaxWorkletFrames = 256u;
constexpr size_t kMaxScopeBatch = 512u;

// Linear-memory arena starting at 128 KB (0x20000), well above static .rodata/.bss (< 24 KB)
// and well below downward stack pointer (initialized at 1 MB = 0x100000).
// Keeps compiled `ruido.wasm` binary ~25 KB instead of embedding 275 KB of zero-filled arrays!
constexpr uintptr_t kArenaBase = 0x20000u;
constexpr uintptr_t kDspArenaAddr = kArenaBase;
constexpr uintptr_t kCtrlArenaAddr = (kDspArenaAddr + sizeof(ruido::DroneEngine) + 63u) & ~63u;
constexpr uintptr_t kRendererArenaAddr =
    (kCtrlArenaAddr + sizeof(ruido::RuidoController) + 63u) & ~63u;
constexpr uintptr_t kRgbaArenaAddr =
    (kRendererArenaAddr + sizeof(ruido::ScopeHudRenderer) + 63u) & ~63u;
static_assert(kRgbaArenaAddr + (ruido::kDisplayWidth * ruido::kDisplayHeight * 4u) < 0xC0000u,
              "Wasm linear memory arena must leave >= 256 KB stack headroom below 1 MB");

// --- Core 1 (AudioWorklet) Static Storage ---
ruido::DroneEngine* gDsp = nullptr;

using namespace ruido::wasm_abi;

alignas(16) ruido::EngineControlState gDspStagedControl{};
alignas(16) float gAudioLeft[kMaxWorkletFrames]{};
alignas(16) float gAudioRight[kMaxWorkletFrames]{};
alignas(16) ruido::ScopeFrame gScopeBatchOut[kMaxScopeBatch]{};
alignas(16) float gTelemetryOut[WasmTelemetryIdx::kCount]{};

// --- Core 0 (Main UI Thread) Static Storage ---
alignas(64) uint8_t gUiTelemetryStorage[sizeof(ruido::EngineTelemetry)]{};

ruido::RuidoController* gCtrl = nullptr;
ruido::ScopeHudRenderer* gRenderer = nullptr;
ruido::EngineTelemetry* gUiTelemetry = nullptr;

alignas(16) ruido::ScopeFrame gScopeBatchIn[kMaxScopeBatch]{};
alignas(16) float gTelemetryIn[WasmTelemetryIdx::kCount]{};
alignas(16) uint16_t gScanlineStripRgb565[ruido::kDisplayWidth * ruido::kScanlineStripHeight]{};
uint8_t* const gScreenRgba = reinterpret_cast<uint8_t*>(kRgbaArenaAddr);
alignas(16) float gSummaryOut[WasmSummaryIdx::kCount]{};
bool gControlDirty = true;

void refreshSummary() noexcept {
    if (gCtrl == nullptr) {
        return;
    }
    const ruido::EngineControlState& st = gCtrl->state();
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        const size_t base = WasmSummaryIdx::kVoiceBase + i * WasmSummaryIdx::kVoiceStride;
        const ruido::VoiceParams& vp = st.voices[i];
        gSummaryOut[base + WasmSummaryIdx::kVoiceFreqHz] = vp.frequencyHz;
        gSummaryOut[base + WasmSummaryIdx::kVoiceAmplitude] = vp.amplitude;
        gSummaryOut[base + WasmSummaryIdx::kVoicePan] = vp.pan;
        gSummaryOut[base + WasmSummaryIdx::kVoiceHpCutoffHz] = vp.hpCutoffHz;
        gSummaryOut[base + WasmSummaryIdx::kVoiceHpResonance] = vp.hpResonance;
        gSummaryOut[base + WasmSummaryIdx::kVoiceLpCutoffHz] = vp.lpCutoffHz;
        gSummaryOut[base + WasmSummaryIdx::kVoiceLpResonance] = vp.lpResonance;
        gSummaryOut[base + WasmSummaryIdx::kVoiceMuted] = vp.muted ? 1.0f : 0.0f;
    }
    gSummaryOut[WasmSummaryIdx::kCircuitCoupling] = st.global.circuitCoupling;
    gSummaryOut[WasmSummaryIdx::kMasterTrim] = st.global.masterTrim;
    gSummaryOut[WasmSummaryIdx::kKnobRate] =
        static_cast<float>(static_cast<int>(gCtrl->hud().knobRate));
    gSummaryOut[WasmSummaryIdx::kMonoCollapse] = st.global.monoCollapse ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kMasterRun] = st.global.masterRun ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kPitchPressureMode] = st.global.pitchPressureMode ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kSaveArmed] = gCtrl->hud().saveArmed ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kLfoPageOpen] = gCtrl->hud().lfoPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kLfoVoiceFocus] = static_cast<float>(gCtrl->hud().lfoVoiceFocus);

    const uint8_t lfoSlot = gCtrl->hud().lfoVoiceFocus & 7u;
    const ruido::VoiceParams& lfoVp = st.voices[ruido::lfoFocusVoice(lfoSlot)];
    const uint8_t lfoSub = ruido::lfoFocusSub(lfoSlot);
    gSummaryOut[WasmSummaryIdx::kLfoTarget] =
        static_cast<float>(static_cast<uint8_t>(ruido::lfoSlotTarget(lfoVp, lfoSub)));
    gSummaryOut[WasmSummaryIdx::kLfoShape] = ruido::lfoSlotShape(lfoVp, lfoSub);
    gSummaryOut[WasmSummaryIdx::kLfoRateHz] = ruido::lfoSlotRateHz(lfoVp, lfoSub);
    gSummaryOut[WasmSummaryIdx::kLfoDepth] = ruido::lfoSlotDepth(lfoVp, lfoSub);
    gSummaryOut[WasmSummaryIdx::kConfigPageOpen] = gCtrl->hud().configPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kKnobBank] =
        static_cast<float>(static_cast<uint8_t>(gCtrl->hud().knobBank));
    gSummaryOut[WasmSummaryIdx::kEditPageOpen] = gCtrl->hud().editPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kFxPageOpen] = gCtrl->hud().fxPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kFxVoiceFocus] = static_cast<float>(gCtrl->hud().fxVoiceFocus);
    gSummaryOut[WasmSummaryIdx::kHomeTarget] =
        static_cast<float>(static_cast<uint8_t>(gCtrl->hud().homeTarget));
    gSummaryOut[WasmSummaryIdx::kPanPageOpen] = gCtrl->hud().panPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kKeyboardModel] =
        static_cast<float>(static_cast<uint8_t>(gCtrl->hud().keyboardModel));
    gSummaryOut[WasmSummaryIdx::kSelPageOpen] = gCtrl->hud().selPageOpen ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kEffectiveClockBpm] = gCtrl->hud().effectiveClockBpm;
    gSummaryOut[WasmSummaryIdx::kMidiClockSynced] = gCtrl->hud().midiClockSynced ? 1.0f : 0.0f;
    gSummaryOut[WasmSummaryIdx::kMidiChannelIn] =
        static_cast<float>(gCtrl->hud().config.midiChannel);
}

} // namespace

#define RUIDO_EXPORT(name) __attribute__((export_name(name), visibility("default")))

extern "C" {

// ============================================================================
// Core 1: Real-Time DSP Exports (AudioWorklet Thread)
// ============================================================================

RUIDO_EXPORT("wasm_dsp_init")
void wasm_dsp_init(float sampleRate) {
    auto* dspMem = reinterpret_cast<void*>(kDspArenaAddr);
    memset(dspMem, 0, sizeof(ruido::DroneEngine));
    gDsp = new (dspMem) ruido::DroneEngine();
    gDspStagedControl = ruido::EngineControlState::makeDefault();
    ruido::applyPatchFrame(ruido::decodePatchSnapshot(
                               ruido::encodePatchSnapshot(ruido::makeFactorySnapshotPreset(0u))),
                           gDspStagedControl);
    gDsp->controlBus().publish(gDspStagedControl);
    gDsp->init(sampleRate);
}

RUIDO_EXPORT("wasm_dsp_set_analog_character")
void wasm_dsp_set_analog_character(int32_t enableDrift, int32_t enableRcTilt) {
    if (gDsp != nullptr) {
        gDsp->setAnalogCharacterEnabled(enableDrift != 0, enableRcTilt != 0);
    }
}

RUIDO_EXPORT("wasm_dsp_get_control_ptr")
void* wasm_dsp_get_control_ptr() {
    return &gDspStagedControl;
}

RUIDO_EXPORT("wasm_dsp_get_control_size")
uint32_t wasm_dsp_get_control_size() {
    return static_cast<uint32_t>(sizeof(ruido::EngineControlState));
}

RUIDO_EXPORT("wasm_dsp_publish_control")
void wasm_dsp_publish_control(int32_t snapImmediate) {
    if (gDsp == nullptr) {
        return;
    }
    gDsp->controlBus().publish(gDspStagedControl);
    if (snapImmediate != 0) {
        gDsp->snapParametersToLatest();
    }
}

RUIDO_EXPORT("wasm_dsp_process_block")
void wasm_dsp_process_block(uint32_t numFrames) {
    if (gDsp == nullptr) {
        return;
    }
    const size_t total = (numFrames > kMaxWorkletFrames) ? kMaxWorkletFrames : numFrames;
    size_t offset = 0u;
    // Process in 64-frame hardware-matched sub-blocks (e.g. 2 x 64 = 128 AudioWorklet quantum)
    while (offset < total) {
        const size_t chunk =
            (total - offset > ruido::kBlockFrames) ? ruido::kBlockFrames : (total - offset);
        gDsp->processBlock(gAudioLeft + offset, gAudioRight + offset, chunk);
        offset += chunk;
    }
}

RUIDO_EXPORT("wasm_dsp_get_left_ptr")
float* wasm_dsp_get_left_ptr() {
    return gAudioLeft;
}

RUIDO_EXPORT("wasm_dsp_get_right_ptr")
float* wasm_dsp_get_right_ptr() {
    return gAudioRight;
}

RUIDO_EXPORT("wasm_dsp_get_scope_frame_size")
uint32_t wasm_dsp_get_scope_frame_size() {
    return static_cast<uint32_t>(sizeof(ruido::ScopeFrame));
}

RUIDO_EXPORT("wasm_dsp_pop_scope")
uint32_t wasm_dsp_pop_scope(uint32_t maxFrames) {
    if (gDsp == nullptr) {
        return 0u;
    }
    const size_t cap = (maxFrames > kMaxScopeBatch) ? kMaxScopeBatch : maxFrames;
    return static_cast<uint32_t>(gDsp->scopeBuffer().popBatch(gScopeBatchOut, cap));
}

RUIDO_EXPORT("wasm_dsp_get_scope_ptr")
const void* wasm_dsp_get_scope_ptr() {
    return gScopeBatchOut;
}

RUIDO_EXPORT("wasm_dsp_get_telemetry_floats")
uint32_t wasm_dsp_get_telemetry_floats() {
    return static_cast<uint32_t>(WasmTelemetryIdx::kCount);
}

RUIDO_EXPORT("wasm_dsp_read_telemetry")
const float* wasm_dsp_read_telemetry() {
    if (gDsp != nullptr) {
        const ruido::EngineTelemetry& t = gDsp->telemetry();
        gTelemetryOut[WasmTelemetryIdx::kPeakLeft] = t.peakLeft.load(std::memory_order_relaxed);
        gTelemetryOut[WasmTelemetryIdx::kPeakRight] = t.peakRight.load(std::memory_order_relaxed);
        for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
            gTelemetryOut[WasmTelemetryIdx::kOscGate + i] =
                t.oscGateHigh[i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoAOut + i] =
                t.lfoOutput[i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoAPhase + i] =
                t.lfoPhase[i].load(std::memory_order_relaxed);
            const uint32_t packedA = t.lfoDriftPacked[i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoADrift + i * 2u] =
                static_cast<float>(packedA & 0x7FFFu);
            gTelemetryOut[WasmTelemetryIdx::kLfoADrift + i * 2u + 1u] =
                static_cast<float>((packedA >> 15u) & 0x7FFFu);
            gTelemetryOut[WasmTelemetryIdx::kVoicePeak + i] =
                t.voicePeak[i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kVoiceDrift + i] =
                t.voiceDriftNorm[i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoBOut + i] =
                t.lfoOutput[4u + i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoBPhase + i] =
                t.lfoPhase[4u + i].load(std::memory_order_relaxed);
            const uint32_t packedB = t.lfoDriftPacked[4u + i].load(std::memory_order_relaxed);
            gTelemetryOut[WasmTelemetryIdx::kLfoBDrift + i * 2u] =
                static_cast<float>(packedB & 0x7FFFu);
            gTelemetryOut[WasmTelemetryIdx::kLfoBDrift + i * 2u + 1u] =
                static_cast<float>((packedB >> 15u) & 0x7FFFu);
        }
        gTelemetryOut[WasmTelemetryIdx::kStutterGateMask] =
            static_cast<float>(t.stutterGateMask.load(std::memory_order_relaxed));
        gTelemetryOut[WasmTelemetryIdx::kRhythmPhase] =
            t.rhythmPhase.load(std::memory_order_relaxed);
    }
    return gTelemetryOut;
}

// ============================================================================
// Core 0: Stateless Controller & 240x16 Scanline UI Exports (Main Thread)
// ============================================================================

RUIDO_EXPORT("wasm_ui_init")
void wasm_ui_init() {
    auto* ctrlMem = reinterpret_cast<void*>(kCtrlArenaAddr);
    auto* rendMem = reinterpret_cast<void*>(kRendererArenaAddr);
    memset(ctrlMem, 0, sizeof(ruido::RuidoController));
    memset(rendMem, 0, sizeof(ruido::ScopeHudRenderer));
    gCtrl = new (ctrlMem) ruido::RuidoController();
    gRenderer = new (rendMem) ruido::ScopeHudRenderer();
    gRenderer->armBootIntro();
    gUiTelemetry = new (gUiTelemetryStorage) ruido::EngineTelemetry();
    gTelemetryIn[WasmTelemetryIdx::kStutterGateMask] = 15.0f;
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_encoder")
void wasm_ui_encoder(uint32_t encoderId, int32_t ticks) {
    if (gCtrl == nullptr || encoderId >= static_cast<uint32_t>(ruido::EncoderId::Count)) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onEncoderDelta(static_cast<ruido::EncoderId>(encoderId), ticks);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_master_pot")
void wasm_ui_master_pot(float level) {
    if (gCtrl == nullptr) {
        return;
    }
    gCtrl->onMasterPot(level);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_button")
void wasm_ui_button(uint32_t buttonId, int32_t pressed) {
    if (gCtrl == nullptr || buttonId >= static_cast<uint32_t>(ruido::ButtonId::Count)) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onButtonEvent(static_cast<ruido::ButtonId>(buttonId), pressed != 0);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_black_key")
void wasm_ui_black_key(uint32_t keyId, int32_t pressed) {
    if (gCtrl == nullptr || keyId >= static_cast<uint32_t>(ruido::BlackKeyId::Count)) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onBlackKeyEvent(static_cast<ruido::BlackKeyId>(keyId), pressed != 0);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_black_pressure")
void wasm_ui_black_pressure(uint32_t keyId, float pressure) {
    if (gCtrl == nullptr || keyId >= static_cast<uint32_t>(ruido::BlackKeyId::Count)) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onBlackKeyPressure(static_cast<ruido::BlackKeyId>(keyId), pressure);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_white_gate")
void wasm_ui_white_gate(uint32_t whiteIdx, int32_t pressed) {
    if (gCtrl == nullptr || whiteIdx >= ruido::kNumWhiteKeys) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onWhiteKeyGate(static_cast<uint8_t>(whiteIdx), pressed != 0);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_white_pressure")
void wasm_ui_white_pressure(uint32_t whiteIdx, float pressure) {
    if (gCtrl == nullptr || whiteIdx >= ruido::kNumWhiteKeys) {
        return;
    }
    if (gRenderer != nullptr) {
        gRenderer->notifyUserInput();
    }
    gCtrl->onWhiteKeyPressure(static_cast<uint8_t>(whiteIdx), pressure);
    gControlDirty = true;
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_midi_realtime")
void wasm_ui_midi_realtime(uint32_t statusByte, float timestampSec) {
    if (gCtrl == nullptr) {
        return;
    }
    gCtrl->onMidiRealtime(static_cast<uint8_t>(statusByte & 0xFFu), timestampSec);
    if (gCtrl->consumeStateDirty()) {
        gControlDirty = true;
    }
    refreshSummary();
}

RUIDO_EXPORT("wasm_ui_get_scope_in_ptr")
void* wasm_ui_get_scope_in_ptr() {
    return gScopeBatchIn;
}

RUIDO_EXPORT("wasm_ui_push_scope")
void wasm_ui_push_scope(uint32_t count) {
    if (gRenderer == nullptr) {
        return;
    }
    const size_t n = (count > kMaxScopeBatch) ? kMaxScopeBatch : count;
    gRenderer->pushScopeSamples(gScopeBatchIn, n);
}

RUIDO_EXPORT("wasm_ui_get_telemetry_in_ptr")
float* wasm_ui_get_telemetry_in_ptr() {
    return gTelemetryIn;
}

RUIDO_EXPORT("wasm_ui_apply_telemetry")
void wasm_ui_apply_telemetry() {
    if (gUiTelemetry == nullptr) {
        return;
    }
    gUiTelemetry->peakLeft.store(gTelemetryIn[WasmTelemetryIdx::kPeakLeft],
                                 std::memory_order_relaxed);
    gUiTelemetry->peakRight.store(gTelemetryIn[WasmTelemetryIdx::kPeakRight],
                                  std::memory_order_relaxed);
    for (size_t i = 0u; i < ruido::kNumOscillators; ++i) {
        gUiTelemetry->oscGateHigh[i].store(gTelemetryIn[WasmTelemetryIdx::kOscGate + i],
                                           std::memory_order_relaxed);
        gUiTelemetry->lfoOutput[i].store(gTelemetryIn[WasmTelemetryIdx::kLfoAOut + i],
                                         std::memory_order_relaxed);
        gUiTelemetry->lfoPhase[i].store(gTelemetryIn[WasmTelemetryIdx::kLfoAPhase + i],
                                        std::memory_order_relaxed);
        const uint32_t lo15A =
            static_cast<uint32_t>(gTelemetryIn[WasmTelemetryIdx::kLfoADrift + i * 2u]) & 0x7FFFu;
        const uint32_t hi15A =
            static_cast<uint32_t>(gTelemetryIn[WasmTelemetryIdx::kLfoADrift + i * 2u + 1u]) &
            0x7FFFu;
        gUiTelemetry->lfoDriftPacked[i].store((hi15A << 15u) | lo15A, std::memory_order_relaxed);
        gUiTelemetry->voicePeak[i].store(gTelemetryIn[WasmTelemetryIdx::kVoicePeak + i],
                                         std::memory_order_relaxed);
        gUiTelemetry->voiceDriftNorm[i].store(gTelemetryIn[WasmTelemetryIdx::kVoiceDrift + i],
                                              std::memory_order_relaxed);
        gUiTelemetry->lfoOutput[4u + i].store(gTelemetryIn[WasmTelemetryIdx::kLfoBOut + i],
                                              std::memory_order_relaxed);
        gUiTelemetry->lfoPhase[4u + i].store(gTelemetryIn[WasmTelemetryIdx::kLfoBPhase + i],
                                             std::memory_order_relaxed);
        const uint32_t lo15B =
            static_cast<uint32_t>(gTelemetryIn[WasmTelemetryIdx::kLfoBDrift + i * 2u]) & 0x7FFFu;
        const uint32_t hi15B =
            static_cast<uint32_t>(gTelemetryIn[WasmTelemetryIdx::kLfoBDrift + i * 2u + 1u]) &
            0x7FFFu;
        gUiTelemetry->lfoDriftPacked[4u + i].store((hi15B << 15u) | lo15B,
                                                   std::memory_order_relaxed);
    }
    gUiTelemetry->stutterGateMask.store(
        static_cast<uint8_t>(
            static_cast<uint32_t>(gTelemetryIn[WasmTelemetryIdx::kStutterGateMask]) & 0x0Fu),
        std::memory_order_relaxed);
    gUiTelemetry->rhythmPhase.store(gTelemetryIn[WasmTelemetryIdx::kRhythmPhase],
                                    std::memory_order_relaxed);
}

RUIDO_EXPORT("wasm_ui_tick_and_render")
void wasm_ui_tick_and_render(float dtSec) {
    if (gCtrl == nullptr || gRenderer == nullptr || gUiTelemetry == nullptr) {
        return;
    }
    gCtrl->tick(dtSec, *gUiTelemetry);
    if (gCtrl->consumeStateDirty()) {
        gControlDirty = true;
        refreshSummary();
    }
    gRenderer->prepareFrame(dtSec, gCtrl->state(), gCtrl->hud(), *gUiTelemetry);

    // Render authentic 240x16 RGB565 hardware strips and expand to RGBA8888 for HTML5 Canvas
    for (uint16_t y0 = 0u; y0 < ruido::kDisplayHeight;
         y0 = static_cast<uint16_t>(y0 + ruido::kScanlineStripHeight)) {
        gRenderer->renderStrip(y0, ruido::kScanlineStripHeight, gScanlineStripRgb565);

        const size_t stripPixels =
            static_cast<size_t>(ruido::kDisplayWidth) * ruido::kScanlineStripHeight;
        uint8_t* rgbaDst = gScreenRgba + (static_cast<size_t>(y0) * ruido::kDisplayWidth) * 4u;

        for (size_t i = 0u; i < stripPixels; ++i) {
            const uint16_t px = gScanlineStripRgb565[i];
            const uint8_t r5 = static_cast<uint8_t>((px >> 11u) & 0x1Fu);
            const uint8_t g6 = static_cast<uint8_t>((px >> 5u) & 0x3Fu);
            const uint8_t b5 = static_cast<uint8_t>(px & 0x1Fu);
            // Match ST7789 controller RGB565 -> RGB666 internal LUT expansion
            const uint8_t r6 = static_cast<uint8_t>((r5 << 1u) | (r5 >> 4u));
            const uint8_t b6 = static_cast<uint8_t>((b5 << 1u) | (b5 >> 4u));
            rgbaDst[i * 4u + 0u] = static_cast<uint8_t>((r6 << 2u) | (r6 >> 4u));
            rgbaDst[i * 4u + 1u] = static_cast<uint8_t>((g6 << 2u) | (g6 >> 4u));
            rgbaDst[i * 4u + 2u] = static_cast<uint8_t>((b6 << 2u) | (b6 >> 4u));
            rgbaDst[i * 4u + 3u] = 255u;
        }
    }
}

RUIDO_EXPORT("wasm_ui_get_rgba_ptr")
const uint8_t* wasm_ui_get_rgba_ptr() {
    return gScreenRgba;
}

RUIDO_EXPORT("wasm_ui_get_leds_ptr")
const float* wasm_ui_get_leds_ptr() {
    if (gCtrl == nullptr) {
        return nullptr;
    }
    return reinterpret_cast<const float*>(&gCtrl->leds());
}

RUIDO_EXPORT("wasm_ui_get_control_ptr")
const void* wasm_ui_get_control_ptr() {
    if (gCtrl == nullptr) {
        return nullptr;
    }
    return &gCtrl->state();
}

RUIDO_EXPORT("wasm_ui_get_summary_floats")
uint32_t wasm_ui_get_summary_floats() {
    return static_cast<uint32_t>(WasmSummaryIdx::kCount);
}

RUIDO_EXPORT("wasm_ui_get_summary_ptr")
const float* wasm_ui_get_summary_ptr() {
    return gSummaryOut;
}

RUIDO_EXPORT("wasm_ui_consume_dirty")
int32_t wasm_ui_consume_dirty() {
    const int32_t wasDirty = gControlDirty ? 1 : 0;
    gControlDirty = false;
    return wasDirty;
}

} // extern "C"
