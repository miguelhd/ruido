// Automated Node V8 verification for `platforms/sim/web/ruido.wasm`:
//  1. Zero unexpected imports (freestanding linear memory + stack + indirect table only).
//  2. Finite, non-silent stereo DSP synthesis & scope ring streaming at 44,118 Hz.
//  3. Two-hand controller + 42-channel hardware LED + 240x16 RGB565->RGBA8888 rendering.

import fs from 'node:fs';
import assert from 'node:assert/strict';
import { WASM_SUMMARY, WASM_TELEMETRY } from '../platforms/sim/web/wasm-abi.js';

const wasmPath = new URL('../platforms/sim/web/ruido.wasm', import.meta.url);
const bytes = fs.readFileSync(wasmPath);
// Unlinked `-O3 -c` Wasm object includes `.linking` + `.reloc.*` sections (FM-1 `.fwsc` < 600 KiB
// XIP ceiling is enforced separately in `platforms/fm1/build_fm1.py`).
assert.ok(bytes.length > 4096 && bytes.length < 720000, `Unexpected wasm size: ${bytes.length}`);

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

// Verify cold-boot CRT `RUIDO` intro renders neutral B&W scanlines on initial frames, then reset UI
{
  for (let i = 0; i < 5; i++) {
    ex.wasm_ui_tick_and_render(0.08); // Advance 5 x 80 ms = 400 ms into settled RUIDO hold
  }
  const introPtr = ex.wasm_ui_get_rgba_ptr();
  const introRgba = u8.subarray(introPtr, introPtr + 240 * 240 * 4);
  let litLogoPx = 0;
  for (let i = 0; i < introRgba.length; i += 4) {
    assert.equal(introRgba[i], introRgba[i + 1]);
    assert.equal(introRgba[i + 1], introRgba[i + 2]);
    if (introRgba[i] > 180) litLogoPx++;
  }
  assert.ok(litLogoPx > 1200, `Expected lit RUIDO boot wordmark pixels, got ${litLogoPx}`);
  assert.equal(introRgba[0], 0); // Pure #000000 top-left corner
  ex.wasm_ui_init();
}

// Verify cold-boot defaults to stopped (`masterRun === false`, PLAY Green=100%, STOP Red=0%, silent)
assert.equal(f32[(ex.wasm_ui_get_summary_ptr() >> 2) + WASM_SUMMARY.MASTER_RUN], 0.0);
assert.ok(Math.abs(f32[(ex.wasm_ui_get_leds_ptr() >> 2) + 27 + 12] - 1.0) < 1e-6);
assert.ok(Math.abs(f32[(ex.wasm_ui_get_leds_ptr() >> 2) + 41] - 0.0) < 1e-6);
ex.wasm_dsp_process_block(128);
for (let i = 0; i < 128; i++) {
  assert.equal(f32[leftIdx + i], 0.0);
  assert.equal(f32[rightIdx + i], 0.0);
}
ex.wasm_ui_button(12, 1); // PLAY/STOP -> start audio output (`masterRun = true`)
ex.wasm_ui_button(12, 0);
for (let i = 0; i < 28; i++) {
  ex.wasm_ui_tick_and_render(1.0 / (0.35 * 28.0));
}
assert.equal(f32[(ex.wasm_ui_get_summary_ptr() >> 2) + WASM_SUMMARY.MASTER_RUN], 1.0);

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
assert.equal(f32[summaryIdx + WASM_SUMMARY.FX_PAGE_OPEN], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.FX_VOICE_FOCUS], 0.0); // OSC 1
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 0.0); // LiveMixer
ex.wasm_ui_button(2, 1);                                          // 2nd FX tap -> OSC 2
ex.wasm_ui_button(2, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.FX_VOICE_FOCUS], 1.0);
ex.wasm_ui_encoder(0, -4);                                        // SELECT CCW -> back to OSC 1
assert.equal(f32[summaryIdx + WASM_SUMMARY.FX_VOICE_FOCUS], 0.0);
assert.ok(Math.abs(f32[ledsIdx + 0] - 0.65) < 1e-4);      // OP1 Drive rest -> 65%
assert.ok(Math.abs(f32[ledsIdx + 1] - 0.65) < 1e-4);      // OP2 Sub Octave rest -> 65%
assert.ok(Math.abs(f32[ledsIdx + 2] - 0.65) < 1e-4);      // OP3 Ratio Lock rest -> 65%
assert.ok(Math.abs(f32[ledsIdx + 3] - 0.40) < 1e-4);      // OP4 Low Pass rest -> 40%
assert.ok(Math.abs(f32[ledsIdx + 4] - 0.40) < 1e-4);      // OP5 High Pass rest -> 40%
assert.ok(Math.abs(f32[ledsIdx + 5] - 0.15) < 1e-4);      // OP6 Unison rest -> 15%
assert.ok(Math.abs(f32[ledsIdx + 6] - 0.15) < 1e-4);      // PIT Detune rest -> 15%
assert.ok(Math.abs(f32[ledsIdx + 7] - 0.15) < 1e-4);      // GLO Spread rest -> 15%
assert.ok(Math.abs(f32[ledsIdx + 8] - 0.15) < 1e-4);      // MONO Hold rest -> 15%
assert.ok(Math.abs(f32[ledsIdx + 9] - 0.0) < 1e-4);       // POLY Tap Tempo rest -> 0% in LiveMixer
assert.ok(Math.abs(f32[ledsIdx + 10] - 1.0) < 1e-4);      // 11th SHIFT beacon -> 100%
assert.ok(Math.abs(f32[ledsIdx + 27 + 2] - 1.0) < 1e-4);  // FX button -> 100% while open
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 0.15) < 1e-4); // SEL button -> 15% rest when closed
assert.ok(f32[ledsIdx + 27 + 5] > 0.15 && f32[ledsIdx + 27 + 5] <= 0.375); // LFO button breathes w/ active boot LFOs
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 0.15) < 1e-4); // EDIT button -> 15% rest when closed & clean
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 0.15) < 1e-4); // GLO right button -> 15% rest when closed
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 0.15) < 1e-4); // HOME button -> 15% while FX page is open
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 0.0) < 1e-6); // PLAY Green LED -> 0% while sound ON
assert.ok(Math.abs(f32[ledsIdx + 27 + 13] - 0.0) < 1e-6); // REC Red LED -> 0%
assert.ok(Math.abs(f32[ledsIdx + 41] - 1.0) < 1e-4);      // STOP Red LED -> 100% while sound ON
for (let i = 0; i < 4; i++) {
  assert.ok(Math.abs(f32[ledsIdx + 11 + i] - 0.65) < 1e-4);       // Keys 1-4 Filter -> 65% rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 4 + i] - 0.40) < 1e-4);   // Keys 5-8 Octave -> 40% rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 8 + i] - 0.15) < 1e-4);   // Keys 9-12 Duty -> 15% rest
  assert.ok(Math.abs(f32[ledsIdx + 11 + 12 + i] - 0.80) < 1e-4);   // Keys 13-16 Mute -> 80% audible (>=20 Hz)
}

// Tapping SEL (button 3) opens persistent Keyboard Model Status & Guidance screen (closing FX page),
// lets SELECT encoder swap LiveMixer(0) <-> CircuitLab(1) <-> RhythmLab(2), and dismisses on 2nd SEL tap!
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + WASM_SUMMARY.FX_PAGE_OPEN], 0.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.PAN_PAGE_OPEN], 0.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.SEL_PAGE_OPEN], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 8.0); // KnobBankMode::None (8) while SEL map is open
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 1.0) < 1e-4);
assert.ok(f32[ledsIdx + 27 + 2] > 0.30 && f32[ledsIdx + 27 + 2] <= 0.85);
const selRgbaStart = ex.wasm_ui_get_rgba_ptr();
const selRgbaBytes = u8.subarray(selRgbaStart, selRgbaStart + 240 * 240 * 4);
for (let i = 0; i < selRgbaBytes.length; i += 4) {
  assert.equal(selRgbaBytes[i], selRgbaBytes[i + 1]);
  assert.equal(selRgbaBytes[i + 1], selRgbaBytes[i + 2]);
}
ex.wasm_ui_encoder(0, 4); // SELECT -> CircuitLab (1)
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 1.0);
ex.wasm_ui_encoder(0, 4); // SELECT -> RhythmLab (2)
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 2.0);
ex.wasm_ui_encoder(0, -8); // SELECT -> back to LiveMixer (0)
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 0.0);
// Persists across 3.0s; subsequent SEL taps cycle 0 -> 1 -> 2 -> 0; HOME dismisses!
for (let i = 0; i < 15; i++) {
  ex.wasm_ui_tick_and_render(0.2);
}
assert.equal(f32[summaryIdx + WASM_SUMMARY.SEL_PAGE_OPEN], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 3] - 1.0) < 1e-4);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 1.0);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 2.0);
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 0.0);
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + WASM_SUMMARY.SEL_PAGE_OPEN], 0.0);
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
assert.equal(f32[summaryIdx + 41], 1.0); // Scene 1 LFO B1 TARGET === WIDTH (1)
ex.wasm_ui_encoder(3, 32);  // KNOB1 TARGET -> A DEPTH (9, exclusive to LFO B!)
assert.equal(f32[summaryIdx + 41], 9.0);
ex.wasm_ui_encoder(6, 120); // KNOB4 DEPTH on LFO B OSC 1

ex.wasm_ui_button(5, 1);  // 2nd LFO tap -> 2 (LFO A OSC 2)
ex.wasm_ui_button(5, 0);
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

// Verify HOME 4-Step Oscillator Core Cycle (button 8: Clean -> FREQUENCY(0) -> DUTY(1) -> DRIFT(2)
// -> Clean) and EDIT 2-Step Console Mixer Cycle (button 6: LEVEL <-> PAN):
ex.wasm_ui_button(8, 1);                  // Escape LFO -> Clean Scope
ex.wasm_ui_button(8, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.LFO_PAGE_OPEN], 0.0);

ex.wasm_ui_button(8, 1);                  // 1st HOME tap -> FREQUENCY (0)
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(0.05);
assert.equal(f32[summaryIdx + WASM_SUMMARY.HOME_TARGET], 0.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 0.0); // KnobBankMode::Frequency
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 1.0) < 1e-4);     // HOME LED === 100%

ex.wasm_ui_button(8, 1);                  // 2nd HOME tap -> DUTY (1)
ex.wasm_ui_button(8, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.HOME_TARGET], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 1.0); // KnobBankMode::PulseWidth
ex.wasm_ui_encoder(3, -40);               // Narrow Osc 1 pulse width

ex.wasm_ui_button(8, 1);                  // 3rd HOME tap -> DRIFT (2)
ex.wasm_ui_button(8, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.HOME_TARGET], 2.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 2.0); // KnobBankMode::VoiceDrift
ex.wasm_ui_encoder(4, 60);                // Increase Osc 2 analog drift

ex.wasm_ui_button(8, 1);                  // 4th HOME tap -> Clean Scope (Off)
ex.wasm_ui_button(8, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.HOME_TARGET], 0.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 0.0);

// Open EDIT (button 6): 1st tap = LEVEL, 2nd tap = PAN, 3rd tap = wraps to LEVEL
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.EDIT_PAGE_OPEN], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.PAN_PAGE_OPEN], 0.0);
ex.wasm_ui_encoder(3, 180);               // Crank Osc 1 into 100% (4.5x pre-filter overdrive)
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
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 1.0) < 1e-4); // EDIT LED === 100% on LEVEL

// 2nd EDIT tap -> Spatial PAN, EDIT LED remains 100%
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
ex.wasm_ui_tick_and_render(0.05);
assert.equal(f32[summaryIdx + WASM_SUMMARY.EDIT_PAGE_OPEN], 0.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.PAN_PAGE_OPEN], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 6] - 1.0) < 1e-4); // EDIT LED === 100% on PAN
ex.wasm_ui_encoder(6, -30);               // Pan Osc 4 inward
assert.ok(f32[summaryIdx + 26] < 0.65);

// 3rd EDIT tap -> wraps to LEVEL for HudAmpGraph RGBA neutrality check!
ex.wasm_ui_button(6, 1);
ex.wasm_ui_button(6, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.EDIT_PAGE_OPEN], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.PAN_PAGE_OPEN], 0.0);
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

// Exit LFO Page via HOME: HOME LED lights to 100% on Clean Scope while LFO LED pulses!
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 39], 0.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 1.0) < 1e-4);
assert.ok(f32[ledsIdx + 27 + 5] >= 0.149 && f32[ledsIdx + 27 + 5] <= 0.61);

// Pressing HOME when no HUD is active latches FREQUENCY HUD open (100% HOME LED);
// 3 more taps step DUTY -> DRIFT -> Clean Scope (still 100% HOME LED across all Home states!)
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);
ex.wasm_ui_tick_and_render(2.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 1.0) < 1e-4);
for (let step = 0; step < 3; step++) {
  ex.wasm_ui_button(8, 1);
  ex.wasm_ui_button(8, 0);
}
ex.wasm_ui_tick_and_render(0.2);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 1.0) < 1e-4);

// Verify right-panel GLO opens GLOBAL CONFIG (button 7, dimming HOME LED to 15%),
// 4-slot SAVE + SNAP 1-4 + PRESETS smooth glide, and 3/3 CIRCUIT LAB Stutter chopper!
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 1.0);
assert.ok(Math.abs(f32[ledsIdx + 27 + 7] - 1.0) < 1e-4);
assert.ok(Math.abs(f32[ledsIdx + 27 + 8] - 0.15) < 1e-4);
assert.equal(f32[summaryIdx + 35], 0.0); // STEREO initially
ex.wasm_ui_encoder(2, 4);                // ALGORITHM -> MONO!
assert.equal(f32[summaryIdx + 35], 1.0);
ex.wasm_ui_encoder(2, -4);               // ALGORITHM -> back to STEREO!
assert.equal(f32[summaryIdx + 35], 0.0);
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.2);
assert.equal(f32[summaryIdx + 45], 0.0);

// Verify 3-step SAVE cycle (1st = STORE SCENE, 2nd = DELETE SCENE, 3rd = clean 150ms exit),
// then store current patch into SCENE 2 (Key 10 = white 9)
const osc1HzBeforeSave = f32[summaryIdx + 0];
ex.wasm_ui_button(9, 1);
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + 38], 1.0); // 1/2 STORE SCENE armed
ex.wasm_ui_button(9, 1);
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + 38], 1.0); // 2/2 DELETE SCENE armed
ex.wasm_ui_button(9, 1);
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + 38], 0.0); // 3rd press exits cleanly!
ex.wasm_ui_tick_and_render(0.2);

ex.wasm_ui_button(9, 1);
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + 38], 1.0);
ex.wasm_ui_white_pressure(9, 0.90);      // Store into SCENE 2!
ex.wasm_ui_white_gate(9, 0);
assert.equal(f32[summaryIdx + 38], 0.0); // saveArmed cleared!
for (let i = 0; i < 12; i++) {
  ex.wasm_ui_tick_and_render(0.2);
}
// While holding SHIFT (black key 10), ACTIVE SCENE 2 lights at 80%, other STORED slots at 40%
ex.wasm_ui_black_key(10, 1);
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[ledsIdx + 11 + 9] - 0.80) < 1e-4);
assert.ok(Math.abs(f32[ledsIdx + 11 + 8] - 0.40) < 1e-4);
ex.wasm_ui_black_key(10, 0);
ex.wasm_ui_tick_and_render(0.016);

// Endless 360-deg circular PRESETS morph (2 -> 3 -> 4 -> 1 -> 2):
ex.wasm_ui_consume_dirty();
ex.wasm_ui_encoder(1, 24); // +6 detents: smooth midpoint blend between SCENE 2 and SCENE 3
assert.equal(ex.wasm_ui_consume_dirty(), 1);
ex.wasm_ui_tick_and_render(0.016);
const partialMorphHz = f32[summaryIdx + 0];
assert.ok(partialMorphHz < osc1HzBeforeSave && partialMorphHz > 56.0);
ex.wasm_ui_encoder(1, 24); // +6 more detents (48 ticks) -> 100% pure SCENE 3 (55.0 Hz)!
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[summaryIdx + 0] - 55.0) < 0.15);
ex.wasm_ui_encoder(1, 96); // +2 spans (3 -> 4 -> wraps seamlessly to 100% pure SCENE 1 = 65.41 Hz!)
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[summaryIdx + 0] - 65.41) < 0.15);
ex.wasm_ui_encoder(1, 48); // +1 span (1 -> 100% pure SCENE 2)
for (let i = 0; i < 12; i++) {
  ex.wasm_ui_tick_and_render(0.2);
}
assert.ok(Math.abs(f32[summaryIdx + 0] - osc1HzBeforeSave) < 0.1);

// Tweak KNOB 1 -> SHIFT + SCENE 2 stays at 80% ACTIVE (zero EDITED dimming); 1-Tap Snap-Back glides back!
ex.wasm_ui_encoder(3, -40);
ex.wasm_ui_black_key(10, 1);
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[ledsIdx + 11 + 9] - 0.80) < 1e-3);
ex.wasm_ui_white_pressure(9, 1.0);
ex.wasm_ui_white_gate(9, 0);
ex.wasm_ui_black_key(10, 0);
ex.wasm_ui_consume_dirty();
for (let i = 0; i < 6; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
assert.equal(ex.wasm_ui_consume_dirty(), 1);
assert.ok(Math.abs(f32[summaryIdx + 0] - osc1HzBeforeSave) < 0.1);
ex.wasm_ui_black_key(10, 1);
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[ledsIdx + 11 + 9] - 0.80) < 1e-4);
ex.wasm_ui_black_key(10, 0);
ex.wasm_ui_tick_and_render(0.016);

// Switch across 3-model SEL carousel (0..2 -> RhythmLab at end) then select 2/3 CIRCUIT LAB (1)
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_encoder(0, 12); // -> RhythmLab (2)
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 2.0);
ex.wasm_ui_encoder(0, -4); // -> CircuitLab (1)
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 1.0);
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);

// Exercise CROSS-PIN (white 0), FOLD (white 4), CRUSH (white 8), RING MOD (OP2), HARD SYNC (OP5), SCREAM (OP4)
ex.wasm_ui_white_pressure(0, 0.85);
ex.wasm_ui_white_pressure(4, 0.90);
ex.wasm_ui_white_pressure(8, 0.95);
ex.wasm_ui_black_pressure(1, 0.90);
ex.wasm_ui_black_pressure(4, 1.00);
ex.wasm_ui_black_pressure(3, 0.95);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 7.0); // OP4 SCREAM routes to KnobBankMode::GlobalFilter (7)
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
ex.wasm_ui_black_key(1, 0);
ex.wasm_ui_black_key(3, 0);
ex.wasm_ui_black_key(4, 0);
ex.wasm_ui_white_gate(0, 0);
ex.wasm_ui_white_gate(4, 0);

// Isolate CRUSH 1 (white 8) + render HUD strips -> verify 42-LED binary bus & strict B&W RGBA neutrality
ex.wasm_ui_tick_and_render(0.2);
assert.ok(Math.abs(f32[ledsIdx + 11 + 8] - 1.0) < 1e-4);
const playRgba = u8.subarray(rgbaStart, rgbaStart + 240 * 240 * 4);
for (let i = 0; i < playRgba.length; i += 4) {
  assert.equal(playRgba[i], playRgba[i + 1]);
  assert.equal(playRgba[i + 1], playRgba[i + 2]);
}
ex.wasm_ui_white_gate(8, 0);
ex.wasm_ui_tick_and_render(0.2);

// Switch to 3/3 RHYTHM LAB (2), lock external 24-PPQN MIDI Clock @ 135 BPM via wasm_ui_midi_realtime,
// engage EUCLID + PLUCK + SWING, verify DSP rhythmPhase + 42-LED bounds + B&W RGBA strips, then MIDI Stop!
ex.wasm_ui_button(3, 1);
ex.wasm_ui_button(3, 0);
ex.wasm_ui_button(3, 1); // 2nd SEL tap -> RhythmLab (2) from CircuitLab (1)
ex.wasm_ui_button(3, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KEYBOARD_MODEL], 2.0);
ex.wasm_ui_button(8, 1);
ex.wasm_ui_button(8, 0);

assert.equal(f32[summaryIdx + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM], 120.0); // Default internal 120 BPM
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CLOCK_SYNCED], 0.0);

// Across all 3 models, SHIFT + ALGORITHM (black key 10 + encoder 2) adjusts Master Tempo
// logarithmically across [20, 3000] BPM without touching circuitCoupling, and POLY (black key 9)
// performs TAP TEMPO!
const couplingBeforeAlg = f32[summaryIdx + WASM_SUMMARY.CIRCUIT_COUPLING];
ex.wasm_ui_black_key(10, 1);
ex.wasm_ui_encoder(2, 48); // +1.0 octave -> ~240.0 BPM
ex.wasm_ui_black_key(10, 0);
assert.ok(Math.abs(f32[summaryIdx + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM] - 240.0) < 0.5);
assert.equal(f32[summaryIdx + WASM_SUMMARY.CIRCUIT_COUPLING], couplingBeforeAlg);
ex.wasm_ui_tick_and_render(0.05);
const tempoRgba = u8.subarray(rgbaStart, rgbaStart + 240 * 240 * 4);
for (let i = 0; i < tempoRgba.length; i += 4) {
  assert.equal(tempoRgba[i], tempoRgba[i + 1]);
  assert.equal(tempoRgba[i + 1], tempoRgba[i + 2]);
}
// Tap Tempo on POLY key (black key 9) spaced by 0.40 s (4 x 0.10 s) -> 150.0 BPM!
ex.wasm_ui_black_key(9, 1);
ex.wasm_ui_black_key(9, 0);
for (let i = 0; i < 4; i++) {
  ex.wasm_ui_tick_and_render(0.10);
}
ex.wasm_ui_black_key(9, 1);
ex.wasm_ui_black_key(9, 0);
assert.ok(Math.abs(f32[summaryIdx + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM] - 150.0) < 0.5);

ex.wasm_ui_midi_realtime(0xFA, 5.0);       // MIDI Start (seconds)
const tickSec135 = 60.0 / (135.0 * 24.0);
for (let k = 0; k < 32; k++) {
  ex.wasm_ui_midi_realtime(0xF8, 5.0 + k * tickSec135);
}
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CLOCK_SYNCED], 1.0);
assert.ok(Math.abs(f32[summaryIdx + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM] - 135.0) < 0.5);

ex.wasm_ui_white_pressure(8, 0.70); // EUCLID 1
ex.wasm_ui_black_pressure(3, 0.85); // OP4 PLUCK
ex.wasm_ui_black_pressure(5, 0.75); // OP6 SWING
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
ex.wasm_ui_white_gate(8, 0);
ex.wasm_ui_black_key(3, 0);
ex.wasm_ui_black_key(5, 0);
// Default MIDI TRANSPORT IN == OFF ignores 0xFC Stop (external 135 BPM clock & audio stay untouched)
ex.wasm_ui_midi_realtime(0xFC, 6.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CLOCK_SYNCED], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MASTER_RUN], 1.0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CHANNEL_IN], 0.0); // Default OMNI (0)

// GLOBAL CONFIG: verify MIDI TRANSPORT IN (OFF -> PHASE -> RUN/STOP) & MIDI CHANNEL IN (CH 3)
ex.wasm_ui_button(7, 1); // ButtonId::GloRight -> open GLOBAL CONFIG
ex.wasm_ui_button(7, 0);
ex.wasm_ui_encoder(0, 20); // EncoderId::Select +5 detents -> Row 5: MIDI TRANSPORT IN
ex.wasm_ui_encoder(2, 4);  // EncoderId::Algorithm +1 detent -> PHASE
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_midi_realtime(0xFC, 6.5); // Stop in PHASE mode releases clock lock to 150 BPM, keeps audio running
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CLOCK_SYNCED], 0.0);
assert.ok(Math.abs(f32[summaryIdx + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM] - 150.0) < 0.5);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MASTER_RUN], 1.0);

ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_encoder(2, 4);  // EncoderId::Algorithm +1 detent -> RUN/STOP (cursor stays on Row 5)
ex.wasm_ui_encoder(0, 4);  // EncoderId::Select +1 detent -> Row 6: MIDI CHANNEL IN
ex.wasm_ui_encoder(2, 12); // EncoderId::Algorithm +3 detents -> CH 3
ex.wasm_ui_button(7, 1);   // Close GLOBAL CONFIG
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.05);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CHANNEL_IN], 3.0);
ex.wasm_ui_midi_realtime(0xFC, 7.0); // Stop in RUN/STOP mode mutes masterRun
ex.wasm_ui_tick_and_render(0.02);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MASTER_RUN], 0.0);
ex.wasm_ui_midi_realtime(0xFA, 7.1); // Start in RUN/STOP mode resumes masterRun
ex.wasm_ui_tick_and_render(0.02);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MASTER_RUN], 1.0);
// Restore OMNI & OFF and close GLOBAL CONFIG
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_encoder(2, -12); // Row 6: MIDI CHANNEL IN -> OMNI
ex.wasm_ui_encoder(0, -4);  // Row 5: MIDI TRANSPORT IN
ex.wasm_ui_encoder(2, -8);  // RUN/STOP -> OFF
ex.wasm_ui_button(7, 1);
ex.wasm_ui_button(7, 0);
ex.wasm_ui_tick_and_render(0.02);
assert.equal(f32[summaryIdx + WASM_SUMMARY.MIDI_CHANNEL_IN], 0.0);

// Toggle PLAY/STOP -> AUDIO OUTPUT OFF (quiet): PLAY (Green) -> 100%, STOP (Red) -> 0%
ex.wasm_ui_button(12, 1);
ex.wasm_ui_button(12, 0);
for (let i = 0; i < 12; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
assert.ok(Math.abs(f32[ledsIdx + 27 + 12] - 1.0) < 1e-4);
assert.ok(Math.abs(f32[ledsIdx + 41] - 0.0) < 1e-4);

// Idle SELECT (encoder 0) steps OSCILLOSCOPE timebase (keeping bottom strip hidden), while
// unconnected switches ENV/ARP/SEQ/REC (buttons 4, 10, 11, 13) come up to the exact 15% steady
// glow of assigned buttons and fade back to 0% dark in 120 ms without waking HUD text:
ex.wasm_ui_button(8, 1); // HOME -> clear latched AudioOutput HUD
ex.wasm_ui_button(8, 0);
for (let i = 0; i < 6; i++) {
  ex.wasm_ui_tick_and_render(0.05);
}
ex.wasm_ui_encoder(0, 4); // SELECT -> OSCILLOSCOPE Wide2x
ex.wasm_ui_tick_and_render(0.05);
assert.equal(f32[summaryIdx + WASM_SUMMARY.KNOB_BANK], 8.0); // KnobBankMode::None (8)
for (let i = 0; i < 20; i++) {
  ex.wasm_ui_tick_and_render(0.1);
}
ex.wasm_ui_button(4, 1); // ENV phantom pin touch
ex.wasm_ui_button(4, 0);
ex.wasm_ui_tick_and_render(0.016);
assert.ok(Math.abs(f32[ledsIdx + 27 + 4] - 0.15) < 1e-4);
ex.wasm_ui_tick_and_render(0.060);
assert.ok(Math.abs(f32[ledsIdx + 27 + 4] - 0.075) < 0.005);
ex.wasm_ui_tick_and_render(0.065);
assert.ok(Math.abs(f32[ledsIdx + 27 + 4]) < 1e-4);

// Arm SAVE (button 9), turn SELECT (encoder 0) to DELETE SCENE, delete SCENE 2 (white key 9),
// and verify PRESETS (encoder 1) skips deleted SCENE 2 straight from SCENE 1 -> SCENE 3 (55.0 Hz):
ex.wasm_ui_button(9, 1); // SAVE -> arm STORE SCENE
ex.wasm_ui_button(9, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.SAVE_ARMED], 1.0);
ex.wasm_ui_encoder(0, 4); // SELECT -> DELETE SCENE
ex.wasm_ui_white_gate(9, 1); // Delete SCENE 2
ex.wasm_ui_white_gate(9, 0);
assert.equal(f32[summaryIdx + WASM_SUMMARY.SAVE_ARMED], 0.0);
ex.wasm_ui_encoder(1, 48); // PRESETS +12 detents -> skips deleted SCENE 2 & centers on SCENE 3
ex.wasm_ui_tick_and_render(0.06);
assert.ok(f32[summaryIdx + WASM_SUMMARY.VOICE_BASE] < 58.0 && f32[summaryIdx + WASM_SUMMARY.VOICE_BASE] > 52.0);

console.log(
  `[PASS] Wasm V8 verification (${(bytes.length / 1024).toFixed(1)} KB wasm, ${shades.size} neutral B&W shades).`
);
