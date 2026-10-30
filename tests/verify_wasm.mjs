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
for (let i = 0; i < 46; i++) {
  f32[telInIdx + i] = f32[telIdx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.2);

// Verify 42 hardware LED channels (OP1..OP5 & SEL strictly 0.0, FX 100% open -> engaged glow when closed)
const ledsIdx = ex.wasm_ui_get_leds_ptr() >> 2;
const summaryIdx = ex.wasm_ui_get_summary_ptr() >> 2;
assert.equal(f32[summaryIdx + 49], 1.0);               // fxPageOpen === true
assert.equal(f32[summaryIdx + 50], 0.0);               // fxVoiceFocus === 0 (OSC 1)
ex.wasm_ui_encoder(0, 4);                              // SELECT -> OSC 2 on FX page
assert.equal(f32[summaryIdx + 50], 1.0);
for (let op = 0; op <= 4; op++) {
  assert.ok(Math.abs(f32[ledsIdx + op] - 0.0) < 1e-6); // OP1..OP5 black keys -> 0% (unassigned)
}
assert.ok(Math.abs(f32[ledsIdx + 5] - 0.25) < 1e-4);   // OP6 Panning landmark -> 25%
assert.ok(Math.abs(f32[ledsIdx + 7] - 0.25) < 1e-4);   // GLO Global landmark -> 25%
assert.ok(Math.abs(f32[ledsIdx + 6] - 0.0) < 1e-6);    // PIT black key -> 0%
assert.ok(Math.abs(f32[ledsIdx + 10] - 0.0) < 1e-6);   // 11th black key -> 0%
assert.ok(Math.abs(f32[ledsIdx + 27 + 2] - 1.0) < 1e-4);  // FX button -> 100% while open
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.0) < 1e-6);  // SEL button -> 0% (unassigned)
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 0.15) < 1e-4); // LFO button -> 15% rest when closed & depth==0
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 0.15) < 1e-4); // EDIT button -> 15% rest when closed & clean
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 0.15) < 1e-4); // GLO right button -> 15% rest when closed
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 0.0) < 1e-6); // PLAY Green LED -> 0% while sound ON
assert.ok(Math.abs(f32[ledsIdx + 27 + 13] - 0.0) < 1e-6); // REC Red LED -> 0%
assert.ok(Math.abs(f32[ledsIdx + 41] - 1.0) < 1e-4);      // STOP Red LED -> 100% while sound ON
for (let i = 0; i < 4; i++) {
  assert.ok(Math.abs(f32[ledsIdx + 11 + 4 + i] - 0.24) < 1e-4); // Keys 5-8 -> 24% rest
}

// Tapping unassigned SEL (button 3) closes FX page, keeps pitchPressureMode === false & SEL LED === 0%,
// and FX LED shows engaged filter glow (> 15%) because Osc 1 filter resonance/cutoffs are engaged!
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_tick_and_render(0.016);
assert.equal(f32[summaryIdx + 49], 0.0);
assert.equal(f32[summaryIdx + 37], 0.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.0) < 1e-6);
assert.ok(f32[ledsIdx + 27 + 2] > 0.30 && f32[ledsIdx + 27 + 2] <= 0.85);

// Open Dual Per-Voice LFO Page (button 5), step SELECT (encoder 0) across 1A -> 1B -> 2A,
// edit KNOB1-4 (including 1B -> A RATE), and verify 46-float Dual-LFO telemetry roundtrip!
ex.wasm_ui_button(5, 1);
ex.wasm_ui_button(5, 0);
assert.equal(f32[summaryIdx + 39], 1.0); // lfoPageOpen === true
assert.equal(f32[summaryIdx + 40], 0.0); // lfoVoiceFocus === 0 (OSC 1A)

ex.wasm_ui_encoder(0, 4); // SELECT -> 1 (OSC 1B)
assert.equal(f32[summaryIdx + 40], 1.0);
assert.equal(f32[summaryIdx + 41], 2.0); // Default TARGET === LPF (2)
ex.wasm_ui_encoder(3, 28);  // KNOB1 TARGET -> A DEPTH (9, exclusive to LFO B!)
assert.equal(f32[summaryIdx + 41], 9.0);
ex.wasm_ui_encoder(6, 120); // KNOB4 DEPTH on 1B

ex.wasm_ui_encoder(0, 4); // SELECT -> 2 (OSC 2A)
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

const tel46Idx = ex.wasm_dsp_read_telemetry() >> 2;
for (let i = 0; i < 46; i++) {
  f32[telInIdx + i] = f32[tel46Idx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 5] - 1.0) < 1e-4); // LFO LED === 100% while open

// Verify EDIT Page (button 6): 4-mode SELECT (WIDTH=0, DRIFT=1, LEVEL=2, PAN=3) & 4.5x overdrive
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
assert.equal(f32[summaryIdx + 48], 1.0); // editPageOpen === true
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
for (let i = 0; i < 46; i++) {
  f32[telInIdx + i] = f32[tel46Idx + i];
}
ex.wasm_ui_apply_telemetry();
ex.wasm_ui_tick_and_render(0.1);
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 1.0) < 1e-4); // EDIT LED === 100% while open

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
assert.ok(f32[ledsIdx + 27 + 6] > 0.25); // Closed EDIT LED flares live with 4.5x overdrive!

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

// Verify right-panel GLO opens GLOBAL CONFIG (button 7), ALGORITHM scrolls options, & keybed GLO + SELECT steps GlobalTarget
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 1.0) < 1e-4);
ex.wasm_ui_encoder(0, 4);                // SELECT -> Row 1 (OUTPUT ROUTING)
assert.equal(f32[summaryIdx + 35], 0.0); // STEREO initially
ex.wasm_ui_encoder(2, 4);                // ALGORITHM -> DUAL MONO!
assert.equal(f32[summaryIdx + 35], 1.0);
ex.wasm_ui_encoder(2, -4);               // ALGORITHM -> back to STEREO!
assert.equal(f32[summaryIdx + 35], 0.0);
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
