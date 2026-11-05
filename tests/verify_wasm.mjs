// Automated Node V8 verification for `platforms/sim/web/ruido.wasm`:
//  1. Zero unexpected imports (freestanding linear memory + stack + indirect table only).
//  2. Finite, non-silent stereo DSP synthesis & scope ring streaming at 44,118 Hz.
//  3. Two-hand controller + 42-channel hardware LED + 240x16 RGB565->RGBA8888 rendering.

import fs from 'node:fs';
import assert from 'node:assert/strict';
import { WASM_SUMMARY, WASM_TELEMETRY } from '../platforms/sim/web/wasm-abi.js';

const wasmPath = new URL('../platforms/sim/web/ruido.wasm', import.meta.url);
const bytes = fs.readFileSync(wasmPath);
assert.ok(bytes.length > 4096 && bytes.length < 550000, `Unexpected wasm size: ${bytes.length}`);

const mod = await WebAssembly.compile(bytes);
const imports = WebAssembly.Module.imports(mod);
for (const imp of imports) {
  assert.equal(imp.module, 'env');
  assert.ok(
    ['__linear_memory', '__stack_pointer', '__indirect_function_table'].includes(imp.name),
    `Unexpected Wasm import: ${imp.module}.${imp.name}`
  );
}

const memory = new WebAssembly.Memory({ initial: 16, maximum: 16 });
const stackPointer = new WebAssembly.Global({ value: 'i32', mutable: true }, 16 * 65536);
const table = new WebAssembly.Table({ initial: 32, element: 'anyfunc' });

const { exports: ex } = await WebAssembly.instantiate(mod, {
  env: {
    __linear_memory: memory,
    __stack_pointer: stackPointer,
    __indirect_function_table: table,
  },
});

ex.wasm_dsp_init(44118.0);
ex.wasm_ui_init();
assert.equal(ex.wasm_dsp_get_telemetry_floats(), WASM_TELEMETRY.COUNT);
assert.equal(ex.wasm_ui_get_summary_floats(), WASM_SUMMARY.COUNT);

const u8 = new Uint8Array(memory.buffer);
const f32 = new Float32Array(memory.buffer);

const leftIdx = ex.wasm_dsp_get_left_ptr() >> 2;
const rightIdx = ex.wasm_dsp_get_right_ptr() >> 2;
const scopeOut = ex.wasm_dsp_get_scope_ptr();
const scopeIn = ex.wasm_ui_get_scope_in_ptr();
const scopeFrameBytes = ex.wasm_dsp_get_scope_frame_size();

// Exercise latching FX Filter Page (button 2) + SELECT voice focus + controller -> DSP bus transfer
ex.wasm_ui_button(2, 1);
ex.wasm_ui_button(2, 0);
ex.wasm_ui_encoder(3, 30);  // HPF
ex.wasm_ui_encoder(4, 45);  // HP RES
ex.wasm_ui_encoder(5, -30); // LPF
ex.wasm_ui_encoder(6, 55);  // LP RES
assert.equal(ex.wasm_ui_consume_dirty(), 1);

const ctrlSize = ex.wasm_dsp_get_control_size();
u8.copyWithin(
  ex.wasm_dsp_get_control_ptr(),
  ex.wasm_ui_get_control_ptr(),
  ex.wasm_ui_get_control_ptr() + ctrlSize
);
ex.wasm_dsp_publish_control(1);

let energyL = 0;
let energyR = 0;
for (let b = 0; b < 24; b++) {
  ex.wasm_dsp_process_block(128);
  for (let i = 0; i < 128; i++) {
    const l = f32[leftIdx + i];
    const r = f32[rightIdx + i];
    assert.ok(Number.isFinite(l) && Math.abs(l) < 1.5);
    assert.ok(Number.isFinite(r) && Math.abs(r) < 1.5);
    energyL += l * l;
    energyR += r * r;
  }
  const nScope = ex.wasm_dsp_pop_scope(256);
  if (nScope > 0) {
    u8.copyWithin(scopeIn, scopeOut, scopeOut + nScope * scopeFrameBytes);
    ex.wasm_ui_push_scope(nScope);
  }
}
assert.ok(energyL > 1.0 && energyR > 1.0);

const telIdx = ex.wasm_dsp_read_telemetry() >> 2;
const telInIdx = ex.wasm_ui_get_telemetry_in_ptr() >> 2;
for (let i = 0; i < WASM_TELEMETRY.COUNT; i++) {
  f32[telInIdx + i] = f32[telIdx + i];
}
assert.equal(f32[telIdx + WASM_TELEMETRY.STUTTER_GATE_MASK], 15.0); // stutterGateMask === 0x0F when idle
assert.ok(
  f32[telIdx + WASM_TELEMETRY.RHYTHM_PHASE] >= 0.0 &&
    f32[telIdx + WASM_TELEMETRY.RHYTHM_PHASE] < 1.0
); // rhythmPhase in [0, 1)
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.2);

// Verify 42 hardware LED channels (Mode 1 LIVE MIXER rest landmarks + panel buttons)
const ledsIdx = ex.wasm_ui_get_leds_ptr() >> 2;
const summaryIdx = ex.wasm_ui_get_summary_ptr() >> 2;
assert.equal(f32[summaryIdx + 49], 1.0);               // fxPageOpen === true
assert.equal(f32[summaryIdx + 50], 0.0);               // fxVoiceFocus === 0 (OSC 1)
assert.equal(f32[summaryIdx + 53], 0.0);               // keyboardModel === 0 (LiveMixer)
ex.wasm_ui_encoder(0, 4);                              // SELECT -> OSC 2 on FX page
assert.equal(f32[summaryIdx + 50], 1.0);
for (let op = 0; op < 4; op++) {
  assert.ok(Math.abs(f32[ledsIdx + op] - 0.64) < 1e-4); // OP1..OP4 Solo rest -> 64%
}
assert.ok(Math.abs(f32[ledsIdx + 4] - 0.46) < 1e-4);    // OP5 Drive rest -> 46%
assert.ok(Math.abs(f32[ledsIdx + 5] - 0.30) < 1e-4);    // OP6 Sub Octave rest -> 30%
assert.ok(Math.abs(f32[ledsIdx + 6] - 0.30) < 1e-4);    // PIT Ratio Lock rest -> 30%
assert.ok(Math.abs(f32[ledsIdx + 7] - 0.30) < 1e-4);    // GLO Low Pass rest -> 30%
assert.ok(Math.abs(f32[ledsIdx + 8] - 0.18) < 1e-4);    // MONO rest -> 18%
assert.ok(Math.abs(f32[ledsIdx + 9] - 0.18) < 1e-4);    // POLY rest -> 18%
assert.ok(Math.abs(f32[ledsIdx + 10] - 0.08) < 1e-4);   // 11th Freeze rest -> 8%
assert.ok(Math.abs(f32[ledsIdx + 27 + 2] - 1.0) < 1e-4);  // FX button -> 100% while open
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.15) < 1e-4); // SEL button -> 15% rest when closed
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 0.15) < 1e-4); // LFO button -> 15% rest when closed & depth==0
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 0.15) < 1e-4); // EDIT button -> 15% rest when closed & clean
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 0.15) < 1e-4); // GLO right button -> 15% rest when closed
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 0.0) < 1e-6); // PLAY Green LED -> 0% while sound ON
assert.ok(Math.abs(f32[ledsIdx + 27 + 13] - 0.0) < 1e-6); // REC Red LED -> 0%
assert.ok(Math.abs(f32[ledsIdx + 41] - 1.0) < 1e-4);      // STOP Red LED -> 100% while sound ON
for (let i = 0; i < 4; i++) {
  assert.ok(Math.abs(f32[ledsIdx + 11 + i] - 0.64) < 1e-4);      // Keys 1-4 Mute -> 64% unmuted rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 4 + i] - 0.42) < 1e-4);  // Keys 5-8 Filter -> 42% rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 8 + i] - 0.22) < 1e-4);  // Keys 9-12 Octave -> 22% rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 12 + i] - 0.08) < 1e-4); // Keys 13-16 Snap (edited) -> 8% rest
}

// Tapping SEL (button 3) opens persistent Keyboard Model Status & Guidance screen (closing FX page),
// lets SELECT encoder swap LiveMixer(0) <-> CircuitLab(1) <-> RhythmLab(2), and dismisses on 2nd SEL tap!
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 49], 0.0);
assert.equal(f32[summaryIdx + 52], 0.0); // panPageOpen === false
assert.equal(f32[summaryIdx + 54], 1.0); // selPageOpen === true
assert.equal(f32[summaryIdx + 47], 13.0); // KnobBankMode::None (13) while full-screen SEL map is open
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 1.0) < 1e-4);
assert.ok(f32[ledsIdx + 27 + 2] > 0.30 && f32[ledsIdx + 27 + 2] <= 0.85);
const selRgbaStart = ex.wasm_ui_get_rgba_ptr();
const selRgbaBytes = u8.subarray(selRgbaStart, selRgbaStart + 240 * 240 * 4);
for (let i = 0; i < selRgbaBytes.length; i += 4) {
  assert.equal(selRgbaBytes[i], selRgbaBytes[i + 1]);
  assert.equal(selRgbaBytes[i + 1], selRgbaBytes[i + 2]);
}
ex.wasm_ui_encoder(0, 4); // SELECT -> CircuitLab (1)
assert.equal(f32[summaryIdx + 53], 1.0);
ex.wasm_ui_encoder(0, 4); // SELECT -> RhythmLab (2)
assert.equal(f32[summaryIdx + 53], 2.0);
ex.wasm_ui_encoder(0, -8); // SELECT -> back to LiveMixer (0)
assert.equal(f32[summaryIdx + 53], 0.0);
// Persists across 3.0s until dismissed by 2nd SEL tap
for (let i = 0; i < 15; i++) {
  ex.wasm_ui_tick_and_render(0.2);
}
assert.equal(f32[summaryIdx + 54], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 1.0) < 1e-4);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 54], 0.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.15) < 1e-4);

// Open Dual Per-Voice LFO Page (button 5), step SELECT (encoder 0) across
// LFO A OSC 1 -> LFO B OSC 1 -> LFO A OSC 2, edit KNOB1-4 (including LFO B -> A DEPTH),
// and verify 46-float Dual-LFO telemetry roundtrip!
ex.wasm_ui_button(5, 1);
ex.wasm_ui_button(5, 0);
assert.equal(f32[summaryIdx + 39], 1.0); // lfoPageOpen === true
assert.equal(f32[summaryIdx + 40], 0.0); // lfoVoiceFocus === 0 (LFO A OSC 1)

ex.wasm_ui_encoder(0, 4); // SELECT -> 1 (LFO B OSC 1)
assert.equal(f32[summaryIdx + 40], 1.0);
assert.equal(f32[summaryIdx + 41], 2.0); // Default TARGET === LPF (2)
ex.wasm_ui_encoder(3, 28);  // KNOB1 TARGET -> A DEPTH (9, exclusive to LFO B!)
assert.equal(f32[summaryIdx + 41], 9.0);
ex.wasm_ui_encoder(6, 120); // KNOB4 DEPTH on LFO B OSC 1

ex.wasm_ui_encoder(0, 4); // SELECT -> 2 (LFO A OSC 2)
assert.equal(f32[summaryIdx + 40], 2.0);
assert.equal(f32[summaryIdx + 41], 2.0); // Default TARGET === LPF (2)
ex.wasm_ui_encoder(3, -8);  // KNOB1 TARGET -> PITCH (0, past WIDTH=1)
assert.equal(f32[summaryIdx + 41], 0.0);
ex.wasm_ui_encoder(3, 16);  // KNOB1 TARGET -> HPF (4)
ex.wasm_ui_encoder(4, 240); // KNOB2 SHAPE -> 1.0 (Smooth Drift)
ex.wasm_ui_encoder(5, 60);  // KNOB3 RATE
ex.wasm_ui_encoder(6, 200); // KNOB4 DEPTH > 0.8
assert.equal(f32[summaryIdx + 41], 4.0);
assert.ok(f32[summaryIdx + 42] > 0.95);
assert.ok(f32[summaryIdx + 44] > 0.80);

u8.copyWithin(
  ex.wasm_dsp_get_control_ptr(),
  ex.wasm_ui_get_control_ptr(),
  ex.wasm_ui_get_control_ptr() + ctrlSize
);
ex.wasm_dsp_publish_control(1);
for (let b = 0; b < 8; b++) {
  ex.wasm_dsp_process_block(128);
}

const telOutIdx = ex.wasm_dsp_read_telemetry() >> 2;
for (let i = 0; i < WASM_TELEMETRY.COUNT; i++) {
  f32[telInIdx + i] = f32[telOutIdx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 1.0) < 1e-4); // LFO LED === 100% while open

// Verify EDIT Page 3-Step Cycle (button 6):
//  1st tap = Page 1 (WIDTH=0, DRIFT=1, LEVEL=2)
//  2nd tap = Page 2 (PAN)
//  3rd tap = Close
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
assert.equal(f32[summaryIdx + 48], 1.0); // editPageOpen === true
assert.equal(f32[summaryIdx + 52], 0.0); // panPageOpen === false
assert.equal(f32[summaryIdx + 51], 2.0); // editTarget === Level (2) default
assert.equal(f32[summaryIdx + 39], 0.0); // lfoPageOpen closed automatically
ex.wasm_ui_encoder(0, -8);               // SELECT -> Width (0)
assert.equal(f32[summaryIdx + 51], 0.0);
ex.wasm_ui_encoder(3, -40);              // Narrow Osc 1 pulse width
ex.wasm_ui_encoder(0, 4);                // SELECT -> Drift (1)
assert.equal(f32[summaryIdx + 51], 1.0);
ex.wasm_ui_encoder(4, 60);               // Increase Osc 2 analog drift
ex.wasm_ui_encoder(0, 4);                // SELECT -> Level (2)
assert.equal(f32[summaryIdx + 51], 2.0);
ex.wasm_ui_encoder(3, 180);              // Crank Osc 1 into 100% (4.5x pre-filter overdrive)
u8.copyWithin(
  ex.wasm_dsp_get_control_ptr(),
  ex.wasm_ui_get_control_ptr(),
  ex.wasm_ui_get_control_ptr() + ctrlSize
);
ex.wasm_dsp_publish_control(1);
for (let b = 0; b < 6; b++) {
  ex.wasm_dsp_process_block(128);
}
for (let i = 0; i < WASM_TELEMETRY.COUNT; i++) {
  f32[telInIdx + i] = f32[telOutIdx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 1.0) < 1e-4); // EDIT LED === 100% on Page 1

// 2nd EDIT tap -> Page 2 (Spatial PAN), EDIT LED remains 100%
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
ex.wasm_ui_tick_and_render(0.05);
assert.equal(f32[summaryIdx + 48], 0.0); // editPageOpen === false
assert.equal(f32[summaryIdx + 52], 1.0); // panPageOpen === true
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 1.0) < 1e-4); // EDIT LED === 100% on Page 2
ex.wasm_ui_encoder(6, -30);              // Pan Osc 4 inward
assert.ok(f32[summaryIdx + 26] < 0.65);

// 3rd EDIT tap -> closes EDIT/PAN cleanly, then re-open Page 1 for RGBA neutrality check
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
assert.equal(f32[summaryIdx + 48], 0.0);
assert.equal(f32[summaryIdx + 52], 0.0);
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
ex.wasm_ui_tick_and_render(0.2);

// Re-open LFO page briefly so the subsequent LFO RGBA & HOME tests exercise LFO graph too,
// after verifying HudAmpGraph RGBA neutrality right here!
const editRgbaStart = ex.wasm_ui_get_rgba_ptr();
const editRgbaBytes = u8.subarray(editRgbaStart, editRgbaStart + 240 * 240 * 4);
for (let i = 0; i < editRgbaBytes.length; i += 4) {
  assert.equal(editRgbaBytes[i], editRgbaBytes[i + 1]);
  assert.equal(editRgbaBytes[i + 1], editRgbaBytes[i + 2]);
}
ex.wasm_ui_button(5, 1);
ex.wasm_ui_button(5, 0);
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 0.15) < 1e-4); // Closed EDIT LED returns to 15% dim!

// Verify 240x240 RGBA frame is strictly 100% neutral monochrome (R === G === B) with rich CRT shades
const rgbaStart = ex.wasm_ui_get_rgba_ptr();
const rgbaBytes = u8.subarray(rgbaStart, rgbaStart + 240 * 240 * 4);
const shades = new Set();
for (let i = 0; i < rgbaBytes.length; i += 4) {
  const r = rgbaBytes[i];
  const g = rgbaBytes[i + 1];
  const b = rgbaBytes[i + 2];
  const a = rgbaBytes[i + 3];
  assert.equal(r, g, `Expected neutral gray R===G at byte ${i}, got ${r} vs ${g}`);
  assert.equal(g, b, `Expected neutral gray G===B at byte ${i}, got ${g} vs ${b}`);
  assert.equal(a, 255);
  shades.add(r);
}
assert.ok(shades.size >= 24, `Expected smooth anti-aliased monochrome palette, got ${shades.size}`);

// Exit LFO Page via HOME: active LFO keeps pulsing LFO LED between 15% and 60%
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 39], 0.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 0.12) < 1e-4);
assert.ok(f32[ledsIdx + 27 + 5] >= 0.149 && f32[ledsIdx + 27 + 5] <= 0.61);

// Pressing HOME when no HUD is active latches FREQUENCY HUD open (100% HOME LED); pressing again hides it
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(2.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 1.0) < 1e-4);
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 0.12) < 1e-4);

// Verify right-panel GLO opens GLOBAL CONFIG (button 7, OUTPUT ROUTING at row 0),
// 4-slot SAVE + SNAP 1-4 + PRESETS smooth glide, and 3/3 CIRCUIT LAB Stutter chopper!
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 1.0) < 1e-4);
assert.equal(f32[summaryIdx + 35], 0.0); // STEREO initially
ex.wasm_ui_encoder(2, 4);                // ALGORITHM -> MONO!
assert.equal(f32[summaryIdx + 35], 1.0);
ex.wasm_ui_encoder(2, -4);               // ALGORITHM -> back to STEREO!
assert.equal(f32[summaryIdx + 35], 0.0);
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 0.0);

// Arm SAVE (button 9) + tap Key 14 (white 13 = SNAP 2) to store current patch, then PRESETS -> SNAP 3
const osc1HzBeforeSave = f32[summaryIdx + 0];
ex.wasm_ui_button(9, 1);
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + 38], 1.0); // saveArmed === true
ex.wasm_ui_white_pressure(13, 0.90);     // Store into SNAP 2!
ex.wasm_ui_white_gate(13, 0);
assert.equal(f32[summaryIdx + 38], 0.0); // saveArmed cleared!
for (let i = 0; i < 12; i++) {
  ex.wasm_ui_tick_and_render(0.2);
}
// Clean active SNAP 2 rests at 16% (< 22% Octave), inactive SNAP slots rest at 8%
assert.ok(Math.abs(f32[ledsIdx + 11 + 13] - 0.16) < 1e-4);
assert.ok(Math.abs(f32[ledsIdx + 11 + 12] - 0.08) < 1e-4);

// Turn PRESETS encoder (1) CW -> SNAP 3 (27.5 Hz sub-tectonic) and verify tick_and_render sets dirty!
ex.wasm_ui_consume_dirty();
ex.wasm_ui_encoder(1, 4);
assert.equal(ex.wasm_ui_consume_dirty(), 1);
for (let i = 0; i < 6; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
assert.equal(ex.wasm_ui_consume_dirty(), 1); // Smooth glide inside tick_and_render dirties control!
assert.ok(Math.abs(f32[summaryIdx + 0] - 27.5) < 0.1);

// Tap Key 14 (white 13 = SNAP 2) -> glides smoothly back to saved patch!
ex.wasm_ui_white_pressure(13, 1.0);
ex.wasm_ui_white_gate(13, 0);
for (let i = 0; i < 6; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
assert.ok(Math.abs(f32[summaryIdx + 0] - osc1HzBeforeSave) < 0.1);

// Switch across 3-model SEL carousel (0..2 -> RhythmLab at end) then select 2/3 CIRCUIT LAB (1)
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_encoder(0, 12); // -> RhythmLab (2)
assert.equal(f32[summaryIdx + 53], 2.0);
ex.wasm_ui_encoder(0, -4); // -> CircuitLab (1)
assert.equal(f32[summaryIdx + 53], 1.0);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);

// Exercise CROSS-PIN (white 4), FOLD (white 8), CRUSH (white 12), RING MOD (OP6), HARD SYNC (MONO), SCREAM (GLO)
ex.wasm_ui_white_pressure(4, 0.85);
ex.wasm_ui_white_pressure(8, 0.90);
ex.wasm_ui_white_pressure(12, 0.95);
ex.wasm_ui_black_pressure(5, 0.90);
ex.wasm_ui_black_pressure(8, 1.00);
ex.wasm_ui_black_pressure(7, 0.95);
assert.equal(f32[summaryIdx + 47], 8.0); // GLO SCREAM routes to KnobBankMode::GlobalFilter (8)
u8.copyWithin(
  ex.wasm_dsp_get_control_ptr(),
  ex.wasm_ui_get_control_ptr(),
  ex.wasm_ui_get_control_ptr() + ctrlSize
);
ex.wasm_dsp_publish_control(0);
for (let b = 0; b < 16; b++) {
  ex.wasm_dsp_process_block(128);
}
const labAudioL = ex.wasm_dsp_get_left_ptr() >> 2;
for (let i = 0; i < 128; i++) {
  assert.ok(Number.isFinite(f32[labAudioL + i]) && Math.abs(f32[labAudioL + i]) <= 1.25);
}
ex.wasm_ui_tick_and_render(0.2);
assert.ok(f32[ledsIdx + 40] > 0.70); // Red REC warning LED flares during SCREAM
ex.wasm_ui_black_key(5, 0);
ex.wasm_ui_black_key(7, 0);
ex.wasm_ui_black_key(8, 0);
ex.wasm_ui_white_gate(4, 0);
ex.wasm_ui_white_gate(8, 0);

// Isolate CRUSH 1 (white 12) + render HUD strips -> verify 42-LED binary bus & strict B&W RGBA neutrality
ex.wasm_ui_tick_and_render(0.2);
assert.ok(Math.abs(f32[ledsIdx + 11 + 12] - 1.0) < 1e-4);
const playRgba = u8.subarray(rgbaStart, rgbaStart + 240 * 240 * 4);
for (let i = 0; i < playRgba.length; i += 4) {
  assert.equal(playRgba[i], playRgba[i + 1]);
  assert.equal(playRgba[i + 1], playRgba[i + 2]);
}
ex.wasm_ui_white_gate(12, 0);
ex.wasm_ui_tick_and_render(0.2);

// Switch to 3/3 RHYTHM LAB (2), lock external 24-PPQN MIDI Clock @ 135 BPM via wasm_ui_midi_realtime,
// engage EUCLID + PLUCK + SWING, verify DSP rhythmPhase + 42-LED bounds + B&W RGBA strips, then MIDI Stop!
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_encoder(0, 8); // -> RhythmLab (2)
assert.equal(f32[summaryIdx + 53], 2.0);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);

assert.equal(f32[summaryIdx + 55], 120.0); // Default internal 120 BPM
assert.equal(f32[summaryIdx + 56], 0.0);   // midiClockSynced === false
ex.wasm_ui_midi_realtime(0xFA, 5.0);       // MIDI Start (seconds)
const tickSec135 = 60.0 / (135.0 * 24.0);
for (let k = 0; k < 32; k++) {
  ex.wasm_ui_midi_realtime(0xF8, 5.0 + k * tickSec135);
}
assert.equal(f32[summaryIdx + 56], 1.0);   // midiClockSynced === true
assert.ok(Math.abs(f32[summaryIdx + 55] - 135.0) < 0.5);

ex.wasm_ui_white_pressure(12, 0.70); // EUCLID 1
ex.wasm_ui_black_pressure(7, 0.85);  // GLO PLUCK
ex.wasm_ui_black_pressure(9, 0.75);  // POLY SWING
u8.copyWithin(
  ex.wasm_dsp_get_control_ptr(),
  ex.wasm_ui_get_control_ptr(),
  ex.wasm_ui_get_control_ptr() + ctrlSize
);
ex.wasm_dsp_publish_control(0);
for (let b = 0; b < 20; b++) {
  ex.wasm_dsp_process_block(128);
}
const rhyTelIdx = ex.wasm_dsp_read_telemetry() >> 2;
for (let i = 0; i < WASM_TELEMETRY.COUNT; i++) {
  f32[telInIdx + i] = f32[rhyTelIdx + i];
}
assert.ok(
  f32[rhyTelIdx + WASM_TELEMETRY.RHYTHM_PHASE] > 0.01 &&
    f32[rhyTelIdx + WASM_TELEMETRY.RHYTHM_PHASE] < 1.0
);
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
for (let i = 0; i < 42; i++) {
  assert.ok(f32[ledsIdx + i] >= 0.0 && f32[ledsIdx + i] <= 1.0);
}
const rhyRgba = u8.subarray(rgbaStart, rgbaStart + 240 * 240 * 4);
for (let i = 0; i < rhyRgba.length; i += 4) {
  assert.equal(rhyRgba[i], rhyRgba[i + 1]);
  assert.equal(rhyRgba[i + 1], rhyRgba[i + 2]);
}
ex.wasm_ui_white_gate(12, 0);
ex.wasm_ui_black_key(7, 0);
ex.wasm_ui_black_key(9, 0);
ex.wasm_ui_midi_realtime(0xFC, 6.0); // MIDI Stop -> back to 120 BPM internal
assert.equal(f32[summaryIdx + 56], 0.0);
assert.equal(f32[summaryIdx + 55], 120.0);

// Toggle PLAY/STOP -> AUDIO OUTPUT OFF (quiet): PLAY (Green) -> 100%, STOP (Red) -> 0%
ex.wasm_ui_button(12, 1);
ex.wasm_ui_button(12, 0);
for (let i = 0; i < 12; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 1.0) < 1e-4);
assert.ok(Math.abs(f32[ledsIdx + 41] - 0.0) < 1e-4);

console.log(
  `[PASS] Wasm V8 verification (${(bytes.length / 1024).toFixed(1)} KB wasm, ${shades.size} neutral B&W shades).`
);
