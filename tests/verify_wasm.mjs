// Automated Node V8 verification for `platforms/sim/web/ruido.wasm`:
//  1. Zero unexpected imports (freestanding linear memory + stack + indirect table only).
//  2. Finite, non-silent stereo DSP synthesis & scope ring streaming at 44,118 Hz.
//  3. Two-hand controller + 42-channel hardware LED + 240x16 RGB565->RGBA8888 rendering.

import fs from 'node:fs';
import assert from 'node:assert/strict';

const wasmPath = new URL('../platforms/sim/web/ruido.wasm', import.meta.url);
const bytes = fs.readFileSync(wasmPath);
assert.ok(bytes.length > 4096 && bytes.length < 350000, `Unexpected wasm size: ${bytes.length}`);

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
const table = new WebAssembly.Table({ initial: 8, element: 'anyfunc' });

const { exports: ex } = await WebAssembly.instantiate(mod, {
  env: {
    __linear_memory: memory,
    __stack_pointer: stackPointer,
    __indirect_function_table: table,
  },
});

ex.wasm_dsp_init(44118.0);
ex.wasm_ui_init();

const u8 = new Uint8Array(memory.buffer);
const f32 = new Float32Array(memory.buffer);

const leftIdx = ex.wasm_dsp_get_left_ptr() >> 2;
const rightIdx = ex.wasm_dsp_get_right_ptr() >> 2;
const scopeOut = ex.wasm_dsp_get_scope_ptr();
const scopeIn = ex.wasm_ui_get_scope_in_ptr();
const scopeFrameBytes = ex.wasm_dsp_get_scope_frame_size();

// Exercise OP1 dual-resonant MS-20 filter hold + controller -> DSP control bus transfer
ex.wasm_ui_black_key(0, 1);
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
for (let i = 0; i < 22; i++) {
  f32[telInIdx + i] = f32[telIdx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.2);

// Verify 42 hardware LED channels (unassigned buttons 0.0, SEL === White Keys 5-8, Green PLAY / Red STOP)
const ledsIdx = ex.wasm_ui_get_leds_ptr() >> 2;
assert.ok(Math.abs(f32[ledsIdx + 0] - 1.0) < 1e-4);   // OP1 held -> 100%
assert.ok(Math.abs(f32[ledsIdx + 4] - 0.25) < 1e-4);  // OP5 Amplitude landmark -> 25%
assert.ok(Math.abs(f32[ledsIdx + 5] - 0.25) < 1e-4);  // OP6 Panning landmark -> 25%
assert.ok(Math.abs(f32[ledsIdx + 7] - 0.25) < 1e-4);  // GLO Global landmark -> 25%
assert.ok(Math.abs(f32[ledsIdx + 6] - 0.0) < 1e-6);   // PIT black key -> 0%
assert.ok(Math.abs(f32[ledsIdx + 10] - 0.0) < 1e-6);  // 11th black key -> 0%
assert.ok(Math.abs(f32[ledsIdx + 27 + 2] - 0.0) < 1e-6); // FX button -> 0%
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 0.15) < 1e-4); // LFO button -> 15% rest when closed & depth==0
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 0.15) < 1e-4); // GLO right button -> 15% rest when closed
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 0.0) < 1e-6); // PLAY Green LED -> 0% while sound ON
assert.ok(Math.abs(f32[ledsIdx + 27 + 13] - 0.0) < 1e-6); // REC Red LED -> 0%
assert.ok(Math.abs(f32[ledsIdx + 41] - 1.0) < 1e-4);      // STOP Red LED -> 100% while sound ON
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.24) < 1e-4); // SEL Filter/LPG mode -> 24%
for (let i = 0; i < 4; i++) {
  assert.equal(f32[ledsIdx + 11 + 4 + i], f32[ledsIdx + 27 + 3]); // Keys 5-8 === SEL
}
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.72) < 1e-4); // SEL Pitch mode -> 72%
for (let i = 0; i < 4; i++) {
  assert.equal(f32[ledsIdx + 11 + 4 + i], f32[ledsIdx + 27 + 3]); // Keys 5-8 === SEL
}

// Release OP1, open Per-Voice LFO Page (button 5), step SELECT (encoder 0), edit KNOB1-4,
// and verify 22-float LFO + 30-bit drift knot telemetry roundtrip & monochrome LFO graph!
ex.wasm_ui_black_key(0, 0);
ex.wasm_ui_button(5, 1);
ex.wasm_ui_button(5, 0);
const summaryIdx = ex.wasm_ui_get_summary_ptr() >> 2;
assert.equal(f32[summaryIdx + 39], 1.0); // lfoPageOpen === true
assert.equal(f32[summaryIdx + 40], 0.0); // lfoVoiceFocus === 0 (OSC 1)

ex.wasm_ui_encoder(0, 4); // SELECT -> OSC 2
assert.equal(f32[summaryIdx + 40], 1.0);
ex.wasm_ui_encoder(3, 8);   // KNOB1 TARGET -> HPF (2)
ex.wasm_ui_encoder(4, 240); // KNOB2 SHAPE -> 1.0 (Smooth Drift)
ex.wasm_ui_encoder(5, 60);  // KNOB3 RATE
ex.wasm_ui_encoder(6, 200); // KNOB4 DEPTH > 0.8
assert.equal(f32[summaryIdx + 41], 2.0);
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

const tel22Idx = ex.wasm_dsp_read_telemetry() >> 2;
for (let i = 0; i < 22; i++) {
  f32[telInIdx + i] = f32[tel22Idx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 1.0) < 1e-4); // LFO LED === 100% while open

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
assert.ok(f32[ledsIdx + 27 + 5] >= 0.149 && f32[ledsIdx + 27 + 5] <= 0.61);

// Verify right-panel GLO opens GLOBAL CONFIG (button 7) & keybed GLO + SELECT steps GlobalTarget
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 1.0) < 1e-4);
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 0.0);

ex.wasm_ui_black_key(7, 1);
assert.equal(f32[summaryIdx + 46], 4.0); // GlobalTarget::All default
ex.wasm_ui_encoder(0, -16);
assert.equal(f32[summaryIdx + 46], 0.0); // GlobalTarget::Freq
ex.wasm_ui_tick_and_render(0.2);
ex.wasm_ui_black_key(7, 0);
ex.wasm_ui_tick_and_render(0.2);

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
