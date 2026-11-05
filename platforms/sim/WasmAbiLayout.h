#pragma once

#include <cstddef>

namespace ruido::wasm_abi {

// Core 1 -> Core 0 flat float telemetry array indices (`gTelemetryOut` / `gTelemetryIn`).
// Must stay 1:1 synchronized with `WASM_TELEMETRY` in `platforms/sim/web/wasm-abi.js`
// (enforced by `tools/check_hygiene.py` and `tests/verify_wasm.mjs`).
namespace WasmTelemetryIdx {
inline constexpr size_t kPeakLeft = 0u;
inline constexpr size_t kPeakRight = 1u;
inline constexpr size_t kOscGate = 2u;    // [2..5] Osc 1..4 gate
inline constexpr size_t kLfoAOut = 6u;    // [6..9] Osc 1..4 LFO A output
inline constexpr size_t kLfoAPhase = 10u; // [10..13] Osc 1..4 LFO A phase
inline constexpr size_t kLfoADrift = 14u; // [14..21] Osc 1..4 LFO A packed drift pairs (lo15, hi15)
inline constexpr size_t kVoicePeak = 22u; // [22..25] Osc 1..4 post-VCA peak
inline constexpr size_t kVoiceDrift = 26u; // [26..29] Osc 1..4 drift norm
inline constexpr size_t kLfoBOut = 30u;    // [30..33] Osc 1..4 LFO B output
inline constexpr size_t kLfoBPhase = 34u;  // [34..37] Osc 1..4 LFO B phase
inline constexpr size_t kLfoBDrift = 38u; // [38..45] Osc 1..4 LFO B packed drift pairs (lo15, hi15)
inline constexpr size_t kStutterGateMask = 46u;
inline constexpr size_t kRhythmPhase = 47u;
inline constexpr size_t kCount = 48u;
} // namespace WasmTelemetryIdx

// Core 0 UI summary array indices (`gSummaryOut`).
// Must stay 1:1 synchronized with `WASM_SUMMARY` in `platforms/sim/web/wasm-abi.js`
// (enforced by `tools/check_hygiene.py` and `tests/verify_wasm.mjs`).
namespace WasmSummaryIdx {
inline constexpr size_t kVoiceBase = 0u;
inline constexpr size_t kVoiceStride = 8u;
inline constexpr size_t kVoiceFreqHz = 0u;
inline constexpr size_t kVoiceAmplitude = 1u;
inline constexpr size_t kVoicePan = 2u;
inline constexpr size_t kVoiceHpCutoffHz = 3u;
inline constexpr size_t kVoiceHpResonance = 4u;
inline constexpr size_t kVoiceLpCutoffHz = 5u;
inline constexpr size_t kVoiceLpResonance = 6u;
inline constexpr size_t kVoiceMuted = 7u;
inline constexpr size_t kCircuitCoupling = 32u;
inline constexpr size_t kMasterTrim = 33u;
inline constexpr size_t kKnobRate = 34u;
inline constexpr size_t kMonoCollapse = 35u;
inline constexpr size_t kMasterRun = 36u;
inline constexpr size_t kPitchPressureMode = 37u;
inline constexpr size_t kSaveArmed = 38u;
inline constexpr size_t kLfoPageOpen = 39u;
inline constexpr size_t kLfoVoiceFocus = 40u;
inline constexpr size_t kLfoTarget = 41u;
inline constexpr size_t kLfoShape = 42u;
inline constexpr size_t kLfoRateHz = 43u;
inline constexpr size_t kLfoDepth = 44u;
inline constexpr size_t kConfigPageOpen = 45u;
inline constexpr size_t kGlobalTarget = 46u;
inline constexpr size_t kKnobBank = 47u;
inline constexpr size_t kEditPageOpen = 48u;
inline constexpr size_t kFxPageOpen = 49u;
inline constexpr size_t kFxVoiceFocus = 50u;
inline constexpr size_t kEditTarget = 51u;
inline constexpr size_t kPanPageOpen = 52u;
inline constexpr size_t kKeyboardModel = 53u;
inline constexpr size_t kSelPageOpen = 54u;
inline constexpr size_t kEffectiveClockBpm = 55u;
inline constexpr size_t kMidiClockSynced = 56u;
inline constexpr size_t kMidiChannelIn = 57u;
inline constexpr size_t kCount = 58u;
} // namespace WasmSummaryIdx

} // namespace ruido::wasm_abi
