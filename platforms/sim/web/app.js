// Main-Thread (Core 0) Hardware Surface, 240x16 Scanline Canvas & WebMIDI Host for Ruido.

import { LFO_TARGET_NAMES, WASM_SUMMARY, WASM_TELEMETRY } from './wasm-abi.js';

const DISPLAY_W = 240;
const DISPLAY_H = 240;

// Exact F3 (MIDI 53) .. G5 (MIDI 79) 27-key mapping of the physical M-VAVE FM-1 keybed
const MIDI_KEYBED_MAP = [
  { kind: 'white', idx: 0 },  // 53 F3
  { kind: 'black', idx: 0 },  // 54 F#3 (OP1)
  { kind: 'white', idx: 1 },  // 55 G3
  { kind: 'black', idx: 1 },  // 56 G#3 (OP2)
  { kind: 'white', idx: 2 },  // 57 A3
  { kind: 'black', idx: 2 },  // 58 A#3 (OP3)
  { kind: 'white', idx: 3 },  // 59 B3
  { kind: 'white', idx: 4 },  // 60 C4  (Pressure Pad 1)
  { kind: 'black', idx: 3 },  // 61 C#4 (OP4)
  { kind: 'white', idx: 5 },  // 62 D4  (Pressure Pad 2)
  { kind: 'black', idx: 4 },  // 63 D#4 (OP5 Unassigned)
  { kind: 'white', idx: 6 },  // 64 E4  (Pressure Pad 3)
  { kind: 'white', idx: 7 },  // 65 F4  (Pressure Pad 4)
  { kind: 'black', idx: 5 },  // 66 F#4 (OP6 Pan)
  { kind: 'white', idx: 8 },  // 67 G4  (Snap 1)
  { kind: 'black', idx: 6 },  // 68 G#4 (PIT)
  { kind: 'white', idx: 9 },  // 69 A4  (Snap 2)
  { kind: 'black', idx: 7 },  // 70 A#4 (GLO Global)
  { kind: 'white', idx: 10 }, // 71 B4  (Snap 3)
  { kind: 'white', idx: 11 }, // 72 C5  (Snap 4)
  { kind: 'black', idx: 8 },  // 73 C#5 (MONO)
  { kind: 'white', idx: 12 }, // 74 D5  (Snap 5)
  { kind: 'black', idx: 9 },  // 75 D#5 (POLY)
  { kind: 'white', idx: 13 }, // 76 E5  (Snap 6)
  { kind: 'white', idx: 14 }, // 77 F5  (Snap 7)
  { kind: 'black', idx: 10 }, // 78 F#5 (Scope Freeze)
  { kind: 'white', idx: 15 }, // 79 G5  (Snap 8)
];

let wasmBytes = null;
let uiWasm = null;
let uiMem = null;
let uiU8 = null;
let uiF32 = null;
let uiPtrs = null;

let audioCtx = null;
let workletNode = null;
let audioStarting = false;

let masterPotValue = 0.85;
const encoderAngles = new Float32Array(7);
const activeBlackPressures = new Float32Array(11);
const activeWhitePressures = new Float32Array(16);
const heldKeyboardCodes = new Set();
const heldKeyboardBlackPads = new Set();
const heldKeyboardPressurePads = new Set();

let announceTimer = null;
function announcePolite(message) {
  const el = document.getElementById('sr-announcer');
  if (!el) return;
  if (announceTimer) clearTimeout(announceTimer);
  announceTimer = setTimeout(() => {
    el.textContent = message;
  }, 180);
}

function clamp01(v) {
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

async function bootUiWasm() {
  const resp = await fetch(`ruido.wasm?v=${Date.now()}`, { cache: 'no-store' });
  wasmBytes = await resp.arrayBuffer();

  uiMem = new WebAssembly.Memory({ initial: 16, maximum: 16 });
  const stackPointer = new WebAssembly.Global({ value: 'i32', mutable: true }, 16 * 65536);
  const table = new WebAssembly.Table({ initial: 32, element: 'anyfunc' });

  const inst = await WebAssembly.instantiate(wasmBytes, {
    env: {
      __linear_memory: uiMem,
      __stack_pointer: stackPointer,
      __indirect_function_table: table,
    },
  });

  uiWasm = inst.instance.exports;
  uiWasm.wasm_ui_init();
  uiU8 = new Uint8Array(uiMem.buffer);
  uiF32 = new Float32Array(uiMem.buffer);

  uiPtrs = {
    rgba: uiWasm.wasm_ui_get_rgba_ptr(),
    leds: uiWasm.wasm_ui_get_leds_ptr() >> 2,
    control: uiWasm.wasm_ui_get_control_ptr(),
    controlSize: uiWasm.wasm_dsp_get_control_size(),
    scopeIn: uiWasm.wasm_ui_get_scope_in_ptr(),
    telemetryIn: uiWasm.wasm_ui_get_telemetry_in_ptr() >> 2,
    summary: uiWasm.wasm_ui_get_summary_ptr() >> 2,
  };

  updateMasterVisual();
  syncAriaFromSummary();
}

function flushControlToWorklet(snap = false) {
  if (!uiWasm) return;
  if (!snap && uiWasm.wasm_ui_consume_dirty() === 0) return;
  syncAriaFromSummary();
  if (!workletNode) return;

  const copy = uiU8.slice(uiPtrs.control, uiPtrs.control + uiPtrs.controlSize).buffer;
  workletNode.port.postMessage({ type: 'control', bytes: copy, snap }, [copy]);
}

async function ensureAudioRunning() {
  if (audioCtx && audioCtx.state === 'running') return;
  if (audioStarting) return;

  const powerBtn = document.getElementById('audio-toggle-btn');
  const rateBadge = document.getElementById('audio-rate');

  if (audioCtx && audioCtx.state === 'suspended') {
    audioStarting = true;
    try {
      await audioCtx.resume();
      powerBtn.setAttribute('aria-pressed', 'true');
      powerBtn.textContent = 'Audio Active (Pause)';
    } finally {
      audioStarting = false;
    }
    return;
  }

  audioStarting = true;
  try {
    audioCtx = new AudioContext({ latencyHint: 'interactive' });
    await audioCtx.audioWorklet.addModule(`ruido-worklet.js?v=${Date.now()}`);

    workletNode = new AudioWorkletNode(audioCtx, 'ruido-audio-processor', {
      numberOfInputs: 0,
      numberOfOutputs: 1,
      outputChannelCount: [2],
    });

    workletNode.port.onmessage = (event) => {
      const msg = event.data;
      if (!msg || !uiWasm) return;

      if (msg.type === 'ready') {
        const osKhz = ((msg.sampleRate * 4) / 1000).toFixed(1);
        rateBadge.textContent = `DSP: ${(msg.sampleRate / 1000).toFixed(1)} kHz (${osKhz} kHz 4x)`;
      } else if (msg.type === 'telemetry') {
        const t = msg.telemetry;
        if (t && t.length >= WASM_TELEMETRY.VOICE_DRIFT) {
          uiF32.set(t.subarray(0, Math.min(t.length, WASM_TELEMETRY.COUNT)), uiPtrs.telemetryIn);
          uiWasm.wasm_ui_apply_telemetry();
        }
        if (msg.scopeBuffer && msg.scopeCount > 0) {
          uiU8.set(new Uint8Array(msg.scopeBuffer), uiPtrs.scopeIn);
          uiWasm.wasm_ui_push_scope(msg.scopeCount);
        }
      } else if (msg.type === 'error') {
        console.error('Ruido AudioWorklet error:', msg.message);
      }
    };

    workletNode.connect(audioCtx.destination);

    const initialControl = uiU8.slice(uiPtrs.control, uiPtrs.control + uiPtrs.controlSize).buffer;
    uiWasm.wasm_ui_consume_dirty();
    workletNode.port.postMessage({ type: 'init', wasmBytes, initialControl }, [initialControl]);

    if (audioCtx.state !== 'running') {
      await audioCtx.resume();
    }

    powerBtn.setAttribute('aria-pressed', 'true');
    powerBtn.textContent = 'Audio Active (Pause)';
    announcePolite('Ruido stereo audio engine active');
  } finally {
    audioStarting = false;
  }
}

async function toggleAudioPower() {
  const powerBtn = document.getElementById('audio-toggle-btn');
  if (!audioCtx || audioCtx.state !== 'running') {
    if (uiWasm && uiPtrs && uiF32[uiPtrs.summary + WASM_SUMMARY.MASTER_RUN] < 0.5) {
      uiWasm.wasm_ui_button(12, 1);
      uiWasm.wasm_ui_button(12, 0);
    }
    await ensureAudioRunning();
    flushControlToWorklet();
  } else {
    await audioCtx.suspend();
    powerBtn.setAttribute('aria-pressed', 'false');
    powerBtn.textContent = 'Resume Audio Engine';
    announcePolite('Audio engine paused');
  }
}

function updateMasterVisual() {
  const knob = document.getElementById('knob-master');
  if (!knob) return;
  const deg = -135 + masterPotValue * 270;
  knob.style.setProperty('--angle', `${deg.toFixed(1)}deg`);
  const pct = Math.round(masterPotValue * 100);
  knob.setAttribute('aria-valuenow', String(pct));
  knob.setAttribute('aria-valuetext', `Master Volume ${pct}%`);
}

function sendEncoderSteps(encId, ticks) {
  if (!uiWasm || ticks === 0) return;
  ensureAudioRunning();

  uiWasm.wasm_ui_encoder(encId, ticks);

  // 96 X4 quadrature transitions = 360 deg (3.75 deg/step), strictly 1:1 linear with zero acceleration
  encoderAngles[encId] = (encoderAngles[encId] + ticks * 3.75) % 360;
  const el = document.querySelector(`.rotary-encoder[data-encoder="${encId}"]`);
  if (el) {
    el.style.setProperty('--angle', `${encoderAngles[encId]}deg`);
  }

  flushControlToWorklet();
}

function syncAriaFromSummary() {
  if (!uiWasm || !uiPtrs) return;
  const s = uiF32.subarray(uiPtrs.summary, uiPtrs.summary + WASM_SUMMARY.COUNT);

  const couplingPct = Math.round(s[WASM_SUMMARY.CIRCUIT_COUPLING] * 100);
  const mono = s[WASM_SUMMARY.MONO_COLLAPSE] > 0.5;
  const run = s[WASM_SUMMARY.MASTER_RUN] > 0.5;
  const pitchMode = s[WASM_SUMMARY.PITCH_PRESSURE_MODE] > 0.5;
  const saveArmed = s[WASM_SUMMARY.SAVE_ARMED] > 0.5;
  const lfoOpen = s[WASM_SUMMARY.LFO_PAGE_OPEN] > 0.5;
  const lfoVoiceFocus = Math.round(s[WASM_SUMMARY.LFO_VOICE_FOCUS]);
  const lfoTarget = Math.round(s[WASM_SUMMARY.LFO_TARGET]);
  const lfoShape = s[WASM_SUMMARY.LFO_SHAPE];
  const lfoRateHz = s[WASM_SUMMARY.LFO_RATE_HZ];
  const lfoDepth = s[WASM_SUMMARY.LFO_DEPTH];

  const selectEl = document.querySelector('[data-encoder="0"]');
  if (selectEl) {
    if (lfoOpen) {
      const voiceNum = ((lfoVoiceFocus >> 1) & 3) + 1;
      const subName = (lfoVoiceFocus & 1) === 0 ? 'A' : 'B';
      selectEl.setAttribute('aria-valuenow', String(Math.min(100, Math.max(0, lfoVoiceFocus * 14))));
      selectEl.setAttribute('aria-valuetext', `LFO ${subName} Osc ${voiceNum}`);
    } else {
      selectEl.setAttribute('aria-valuenow', '0');
      selectEl.setAttribute('aria-valuetext', 'Unassigned');
    }
  }

  const algEl = document.querySelector('[data-encoder="2"]');
  if (algEl) {
    algEl.setAttribute('aria-valuenow', String(couplingPct));
    algEl.setAttribute('aria-valuetext', `Rail Coupling ${couplingPct}%`);
  }

  for (let i = 0; i < 4; i++) {
    const base = WASM_SUMMARY.VOICE_BASE + i * WASM_SUMMARY.VOICE_STRIDE;
    const hz = s[base + WASM_SUMMARY.VOICE_FREQ_HZ];
    const muted = s[base + WASM_SUMMARY.VOICE_MUTED] > 0.5;
    const knobEl = document.querySelector(`[data-encoder="${3 + i}"]`);
    if (knobEl) {
      if (lfoOpen) {
        if (i === 0) {
          knobEl.setAttribute(
            'aria-valuetext',
            `LFO Target ${LFO_TARGET_NAMES[lfoTarget] || 'LPF'}`
          );
        } else if (i === 1) {
          knobEl.setAttribute('aria-valuetext', `LFO Shape ${Math.round(lfoShape * 100)}%`);
        } else if (i === 2) {
          const rateText = lfoRateHz < 10 ? lfoRateHz.toFixed(2) : lfoRateHz.toFixed(1);
          knobEl.setAttribute('aria-valuetext', `LFO Rate ${rateText} Hz`);
        } else {
          knobEl.setAttribute('aria-valuetext', `LFO Depth ${Math.round(lfoDepth * 100)}%`);
        }
      } else {
        const hzText = hz < 100 ? hz.toFixed(2) : hz.toFixed(1);
        knobEl.setAttribute('aria-valuetext', `Osc ${i + 1} Frequency ${hzText} Hz`);
      }
    }
    const muteKey = document.querySelector(`[data-white="${i}"]`);
    if (muteKey) {
      muteKey.setAttribute('aria-pressed', muted ? 'false' : 'true');
    }
  }

  const setPressed = (sel, val) => {
    const el = document.querySelector(sel);
    if (el) el.setAttribute('aria-pressed', val ? 'true' : 'false');
  };
  setPressed('[data-black="8"]', mono);
  setPressed('[data-black="9"]', !mono);
  setPressed('[data-button="3"]', pitchMode);
  setPressed('[data-button="5"]', lfoOpen);
  setPressed(
    '[data-button="12"]',
    run && Boolean((audioCtx && audioCtx.state === 'running') || audioStarting)
  );
  setPressed('[data-button="9"]', saveArmed);
}

function bindKnobs() {
  const masterEl = document.getElementById('knob-master');
  let potStartY = 0;
  let potStartVal = 0;

  masterEl.addEventListener('pointerdown', (e) => {
    ensureAudioRunning();
    masterEl.setPointerCapture(e.pointerId);
    potStartY = e.clientY;
    potStartVal = masterPotValue;

    const onMove = (ev) => {
      const dy = potStartY - ev.clientY;
      masterPotValue = clamp01(potStartVal + dy / 150);
      uiWasm.wasm_ui_master_pot(masterPotValue);
      updateMasterVisual();
      flushControlToWorklet();
    };
    const onUp = () => {
      masterEl.removeEventListener('pointermove', onMove);
      masterEl.removeEventListener('pointerup', onUp);
      masterEl.removeEventListener('pointercancel', onUp);
    };
    masterEl.addEventListener('pointermove', onMove);
    masterEl.addEventListener('pointerup', onUp);
    masterEl.addEventListener('pointercancel', onUp);
  });

  masterEl.addEventListener(
    'wheel',
    (e) => {
      e.preventDefault();
      ensureAudioRunning();
      const step = e.shiftKey ? 0.005 : 0.025;
      masterPotValue = clamp01(masterPotValue + (e.deltaY < 0 ? step : -step));
      uiWasm.wasm_ui_master_pot(masterPotValue);
      updateMasterVisual();
      flushControlToWorklet();
    },
    { passive: false }
  );

  masterEl.addEventListener('keydown', (e) => {
    let delta = 0;
    if (e.key === 'ArrowUp' || e.key === 'ArrowRight') delta = 0.02;
    else if (e.key === 'ArrowDown' || e.key === 'ArrowLeft') delta = -0.02;
    else if (e.key === 'PageUp') delta = 0.1;
    else if (e.key === 'PageDown') delta = -0.1;
    if (delta !== 0) {
      e.preventDefault();
      ensureAudioRunning();
      masterPotValue = clamp01(masterPotValue + delta);
      uiWasm.wasm_ui_master_pot(masterPotValue);
      updateMasterVisual();
      flushControlToWorklet();
    }
  });

  document.querySelectorAll('.rotary-encoder').forEach((encEl) => {
    const encId = Number(encEl.dataset.encoder);
    let accumPx = 0;
    let wheelAccum = 0;
    let lastY = 0;

    encEl.addEventListener('pointerdown', (e) => {
      ensureAudioRunning();
      encEl.setPointerCapture(e.pointerId);
      lastY = e.clientY;
      accumPx = 0;

      const onMove = (ev) => {
        const dy = lastY - ev.clientY;
        lastY = ev.clientY;
        accumPx += dy;
        const ticks = Math.trunc(accumPx);
        if (ticks !== 0) {
          accumPx -= ticks;
          sendEncoderSteps(encId, ticks);
        }
      };
      const onUp = () => {
        encEl.removeEventListener('pointermove', onMove);
        encEl.removeEventListener('pointerup', onUp);
        encEl.removeEventListener('pointercancel', onUp);
      };
      encEl.addEventListener('pointermove', onMove);
      encEl.addEventListener('pointerup', onUp);
      encEl.addEventListener('pointercancel', onUp);
    });

    encEl.addEventListener(
      'wheel',
      (e) => {
        e.preventDefault();
        const knobBank =
          uiWasm && uiPtrs ? Math.round(uiF32[uiPtrs.summary + WASM_SUMMARY.KNOB_BANK]) : 0;
        // KnobBankMode: Lfo = 6, None (GlobalConfig / SEL) = 9
        const isDiscrete =
          encId === 0 ||
          encId === 1 ||
          (encId === 2 && knobBank === 9) ||
          (encId === 3 && knobBank === 6);

        if (isDiscrete) {
          if (e.deltaMode === 1 || Math.abs(e.deltaY) >= 24) {
            wheelAccum = 0;
            sendEncoderSteps(encId, e.deltaY < 0 ? 4 : -4);
          } else {
            wheelAccum += -e.deltaY / 16;
            const detents = Math.trunc(wheelAccum);
            if (detents !== 0) {
              wheelAccum -= detents;
              sendEncoderSteps(encId, detents > 0 ? 4 : -4);
            }
          }
          return;
        }

        wheelAccum += -e.deltaY * 0.35;
        let ticks = Math.trunc(wheelAccum);
        if (ticks === 0 && Math.abs(e.deltaY) >= 4) {
          ticks = e.deltaY < 0 ? 1 : -1;
          wheelAccum = 0;
        } else {
          wheelAccum -= ticks;
        }
        if (ticks !== 0) {
          sendEncoderSteps(encId, ticks);
        }
      },
      { passive: false }
    );

    encEl.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowUp' || e.key === 'ArrowRight') {
        e.preventDefault();
        sendEncoderSteps(encId, 4);
      } else if (e.key === 'ArrowDown' || e.key === 'ArrowLeft') {
        e.preventDefault();
        sendEncoderSteps(encId, -4);
      } else if (e.key === 'PageUp') {
        e.preventDefault();
        sendEncoderSteps(encId, 16);
      } else if (e.key === 'PageDown') {
        e.preventDefault();
        sendEncoderSteps(encId, -16);
      }
    });
  });
}

function pressPanelButton(btnId) {
  const wasAudioRunning = Boolean(audioCtx && audioCtx.state === 'running');
  const wasMasterRun = uiF32[uiPtrs.summary + WASM_SUMMARY.MASTER_RUN] > 0.5;
  ensureAudioRunning();
  if (btnId === 12 && !wasAudioRunning && wasMasterRun) {
    // Browser was quiet because WebAudio had not started yet:
    // cycle off -> on synchronously so first PLAY/STOP press starts sound (AUDIO OUTPUT ON).
    uiWasm.wasm_ui_button(12, 1);
    uiWasm.wasm_ui_button(12, 0);
  }
  uiWasm.wasm_ui_button(btnId, 1);
  flushControlToWorklet();
}

function bindButtonsAndKeybed() {
  // 14 Panel Buttons
  document.querySelectorAll('.panel-btn').forEach((btn) => {
    const btnId = Number(btn.dataset.button);

    btn.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      btn.setPointerCapture(e.pointerId);
      btn.classList.add('is-held');
      pressPanelButton(btnId);

      let released = false;
      const onRelease = () => {
        if (released) return;
        released = true;
        btn.classList.remove('is-held');
        btn.blur();
        uiWasm.wasm_ui_button(btnId, 0);
        flushControlToWorklet();
        btn.removeEventListener('pointerup', onRelease);
        btn.removeEventListener('pointercancel', onRelease);
        btn.removeEventListener('lostpointercapture', onRelease);
      };
      btn.addEventListener('pointerup', onRelease);
      btn.addEventListener('pointercancel', onRelease);
      btn.addEventListener('lostpointercapture', onRelease);
    });

    // Keyboard activation on native <button> fires click with detail === 0
    btn.addEventListener('click', (e) => {
      if (e.detail === 0) {
        pressPanelButton(btnId);
        uiWasm.wasm_ui_button(btnId, 0);
        flushControlToWorklet();
      }
    });
  });

  // 11 Raised Black Silicone Keys (OP1..6, PIT, GLO, MONO, POLY, 11th — momentary + pressure)
  document.querySelectorAll('.black-key').forEach((keyEl) => {
    const keyId = Number(keyEl.dataset.black);

    const setBlackPressure = (p) => {
      const clamped = clamp01(p);
      activeBlackPressures[keyId] = clamped;
      keyEl.style.setProperty('--pressure', clamped.toFixed(3));
      uiWasm.wasm_ui_black_pressure(keyId, clamped);
      flushControlToWorklet();
    };

    keyEl.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      ensureAudioRunning();

      keyEl.setPointerCapture(e.pointerId);
      keyEl.classList.add('is-held');

      const rect = keyEl.getBoundingClientRect();
      const hitNorm = 1 - clamp01((e.clientY - rect.top) / Math.max(1, rect.height));
      const basePressure = 0.16 + 0.22 * hitNorm;
      const startY = e.clientY;

      uiWasm.wasm_ui_black_key(keyId, 1);
      setBlackPressure(basePressure);

      const onMove = (ev) => {
        const dy = startY - ev.clientY;
        setBlackPressure(basePressure + dy / 115);
      };

      let released = false;
      const onRelease = () => {
        if (released) return;
        released = true;
        keyEl.classList.remove('is-held');
        keyEl.blur();
        activeBlackPressures[keyId] = 0;
        keyEl.style.setProperty('--pressure', '0');
        uiWasm.wasm_ui_black_key(keyId, 0);
        flushControlToWorklet();
        keyEl.removeEventListener('pointermove', onMove);
        keyEl.removeEventListener('pointerup', onRelease);
        keyEl.removeEventListener('pointercancel', onRelease);
        keyEl.removeEventListener('lostpointercapture', onRelease);
      };
      keyEl.addEventListener('pointermove', onMove);
      keyEl.addEventListener('pointerup', onRelease);
      keyEl.addEventListener('pointercancel', onRelease);
      keyEl.addEventListener('lostpointercapture', onRelease);
    });

    keyEl.addEventListener(
      'wheel',
      (e) => {
        if (activeBlackPressures[keyId] <= 0 && !heldKeyboardBlackPads.has(keyId)) return;
        e.preventDefault();
        const nextP = clamp01(activeBlackPressures[keyId] + (e.deltaY < 0 ? 0.06 : -0.06));
        setBlackPressure(nextP);
      },
      { passive: false }
    );

    keyEl.addEventListener('click', (e) => {
      if (e.detail === 0) {
        ensureAudioRunning();
        uiWasm.wasm_ui_black_key(keyId, 1);
        uiWasm.wasm_ui_black_key(keyId, 0);
        flushControlToWorklet();
      }
    });
  });

  // 16 Bottom White Silicone Keys (all 16 momentary + continuous pressure)
  document.querySelectorAll('.white-key').forEach((keyEl) => {
    const wIdx = Number(keyEl.dataset.white);

    const setPadPressure = (p) => {
      const clamped = clamp01(p);
      activeWhitePressures[wIdx] = clamped;
      keyEl.style.setProperty('--pressure', clamped.toFixed(3));
      uiWasm.wasm_ui_white_pressure(wIdx, clamped);
      flushControlToWorklet();
    };

    keyEl.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      ensureAudioRunning();
      keyEl.setPointerCapture(e.pointerId);
      keyEl.classList.add('is-held');

      const rect = keyEl.getBoundingClientRect();
      const hitNorm = 1 - clamp01((e.clientY - rect.top) / Math.max(1, rect.height));
      const basePressure = 0.16 + 0.22 * hitNorm;
      const startY = e.clientY;

      uiWasm.wasm_ui_white_gate(wIdx, 1);
      setPadPressure(basePressure);

      const onMove = (ev) => {
        const dy = startY - ev.clientY;
        setPadPressure(basePressure + dy / 115);
      };

      let released = false;
      const onRelease = () => {
        if (released) return;
        released = true;
        keyEl.classList.remove('is-held');
        keyEl.blur();
        activeWhitePressures[wIdx] = 0;
        keyEl.style.setProperty('--pressure', '0');
        uiWasm.wasm_ui_white_gate(wIdx, 0);
        flushControlToWorklet();
        keyEl.removeEventListener('pointermove', onMove);
        keyEl.removeEventListener('pointerup', onRelease);
        keyEl.removeEventListener('pointercancel', onRelease);
        keyEl.removeEventListener('lostpointercapture', onRelease);
      };

      keyEl.addEventListener('pointermove', onMove);
      keyEl.addEventListener('pointerup', onRelease);
      keyEl.addEventListener('pointercancel', onRelease);
      keyEl.addEventListener('lostpointercapture', onRelease);
    });

    keyEl.addEventListener(
      'wheel',
      (e) => {
        if (activeWhitePressures[wIdx] <= 0 && !heldKeyboardPressurePads.has(wIdx)) return;
        e.preventDefault();
        const nextP = clamp01(activeWhitePressures[wIdx] + (e.deltaY < 0 ? 0.06 : -0.06));
        setPadPressure(nextP);
      },
      { passive: false }
    );

    keyEl.addEventListener('click', (e) => {
      if (e.detail === 0) {
        ensureAudioRunning();
        uiWasm.wasm_ui_white_gate(wIdx, 1);
        uiWasm.wasm_ui_white_gate(wIdx, 0);
        flushControlToWorklet();
      }
    });
  });
}

function bindComputerKeyboard() {
  const keyToBlack = {
    Digit1: 0,
    Digit2: 1,
    Digit3: 2,
    Digit4: 3,
    Digit5: 4,
    Digit6: 5,
    KeyP: 6,
    KeyG: 7,
    KeyM: 8,
    KeyN: 9,
    Digit0: 10,
  };
  const keyToButton = {
    BracketLeft: 0,  // OCT- (momentary Fine knob rate)
    Minus: 0,        // OCT- (momentary Fine knob rate)
    BracketRight: 1, // OCT+ (momentary Coarse knob rate)
    Equal: 1,        // OCT+ (momentary Coarse knob rate)
    KeyQ: 2,         // FX (Per-Voice MS-20 Dual Filter Page toggle)
    KeyW: 3,         // SEL (Stateless Keyboard Model Toast)
    KeyL: 5,         // LFO (Per-Voice LFO Page toggle)
    KeyE: 6,         // EDIT (Page 1 Mix -> Page 2 Pan -> Close)
    Semicolon: 7,    // GLO right grid (GLOBAL CONFIG menu toggle)
    KeyH: 8,         // HOME
    Escape: 8,       // HOME
  };
  const keyToWhite = {
    KeyZ: 0,
    KeyX: 1,
    KeyC: 2,
    KeyV: 3,
    KeyA: 4,
    KeyS: 5,
    KeyD: 6,
    KeyF: 7,
    KeyR: 8,
    KeyT: 9,
    KeyY: 10,
    KeyU: 11,
    KeyJ: 12,
    KeyK: 13,
    KeyI: 14,
    KeyO: 15,
  };

  window.addEventListener('keydown', (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;

    // Allow ArrowUp / ArrowDown to lean harder/softer while holding any keybed punch-in pad
    if (
      (e.key === 'ArrowUp' || e.key === 'ArrowDown') &&
      (heldKeyboardPressurePads.size > 0 || heldKeyboardBlackPads.size > 0)
    ) {
      e.preventDefault();
      const delta = e.key === 'ArrowUp' ? 0.08 : -0.08;
      for (const bId of heldKeyboardBlackPads) {
        const nextP = clamp01(activeBlackPressures[bId] + delta);
        activeBlackPressures[bId] = nextP;
        const el = document.querySelector(`[data-black="${bId}"]`);
        if (el) el.style.setProperty('--pressure', nextP.toFixed(3));
        uiWasm.wasm_ui_black_pressure(bId, nextP);
      }
      for (const wIdx of heldKeyboardPressurePads) {
        const nextP = clamp01(activeWhitePressures[wIdx] + delta);
        activeWhitePressures[wIdx] = nextP;
        const el = document.querySelector(`[data-white="${wIdx}"]`);
        if (el) el.style.setProperty('--pressure', nextP.toFixed(3));
        uiWasm.wasm_ui_white_pressure(wIdx, nextP);
      }
      flushControlToWorklet();
      return;
    }

    if (e.repeat || heldKeyboardCodes.has(e.code)) return;

    if (e.code === 'Comma' || e.code === 'Period') {
      e.preventDefault();
      sendEncoderSteps(0, e.code === 'Period' ? 4 : -4);
      return;
    }

    if (e.code === 'Space' && document.activeElement?.id !== 'audio-toggle-btn') {
      e.preventDefault();
      if (
        document.activeElement &&
        (document.activeElement.classList.contains('panel-btn') ||
          document.activeElement.classList.contains('silicone-key'))
      ) {
        document.activeElement.blur();
      }
      heldKeyboardCodes.add(e.code);
      const playEl = document.querySelector('[data-button="12"]');
      if (playEl) playEl.classList.add('is-held');
      pressPanelButton(12);
      return;
    }

    if (e.code in keyToBlack) {
      ensureAudioRunning();
      heldKeyboardCodes.add(e.code);
      const bId = keyToBlack[e.code];
      heldKeyboardBlackPads.add(bId);
      activeBlackPressures[bId] = 0.45;
      const el = document.querySelector(`[data-black="${bId}"]`);
      if (el) {
        el.classList.add('is-held');
        el.style.setProperty('--pressure', '0.450');
      }
      uiWasm.wasm_ui_black_key(bId, 1);
      uiWasm.wasm_ui_black_pressure(bId, 0.45);
      flushControlToWorklet();
    } else if (e.code in keyToButton) {
      ensureAudioRunning();
      heldKeyboardCodes.add(e.code);
      const btnId = keyToButton[e.code];
      const el = document.querySelector(`[data-button="${btnId}"]`);
      if (el) el.classList.add('is-held');
      uiWasm.wasm_ui_button(btnId, 1);
      flushControlToWorklet();
    } else if (e.code in keyToWhite) {
      ensureAudioRunning();
      heldKeyboardCodes.add(e.code);
      const wIdx = keyToWhite[e.code];
      heldKeyboardPressurePads.add(wIdx);
      activeWhitePressures[wIdx] = 0.45;
      const el = document.querySelector(`[data-white="${wIdx}"]`);
      if (el) {
        el.classList.add('is-held');
        el.style.setProperty('--pressure', '0.450');
      }
      uiWasm.wasm_ui_white_gate(wIdx, 1);
      uiWasm.wasm_ui_white_pressure(wIdx, 0.45);
      flushControlToWorklet();
    }
  });

  window.addEventListener('keyup', (e) => {
    if (!heldKeyboardCodes.has(e.code)) return;
    heldKeyboardCodes.delete(e.code);

    if (e.code === 'Space') {
      const playEl = document.querySelector('[data-button="12"]');
      if (playEl) playEl.classList.remove('is-held');
      uiWasm.wasm_ui_button(12, 0);
      flushControlToWorklet();
    } else if (e.code in keyToBlack) {
      const bId = keyToBlack[e.code];
      heldKeyboardBlackPads.delete(bId);
      activeBlackPressures[bId] = 0;
      const el = document.querySelector(`[data-black="${bId}"]`);
      if (el) {
        el.classList.remove('is-held');
        el.style.setProperty('--pressure', '0');
      }
      uiWasm.wasm_ui_black_key(bId, 0);
      flushControlToWorklet();
    } else if (e.code in keyToButton) {
      const btnId = keyToButton[e.code];
      const el = document.querySelector(`[data-button="${btnId}"]`);
      if (el) el.classList.remove('is-held');
      uiWasm.wasm_ui_button(btnId, 0);
      flushControlToWorklet();
    } else if (e.code in keyToWhite) {
      const wIdx = keyToWhite[e.code];
      heldKeyboardPressurePads.delete(wIdx);
      activeWhitePressures[wIdx] = 0;
      const el = document.querySelector(`[data-white="${wIdx}"]`);
      if (el) {
        el.classList.remove('is-held');
        el.style.setProperty('--pressure', '0');
      }
      uiWasm.wasm_ui_white_gate(wIdx, 0);
      flushControlToWorklet();
    }
  });

  // Release any held keyboard modifiers if the browser window loses focus (e.g. Cmd+Tab)
  window.addEventListener('blur', () => {
    if (!uiWasm || heldKeyboardCodes.size === 0) return;
    for (const code of heldKeyboardCodes) {
      if (code === 'Space') {
        const playEl = document.querySelector('[data-button="12"]');
        if (playEl) playEl.classList.remove('is-held');
        uiWasm.wasm_ui_button(12, 0);
      } else if (code in keyToBlack) {
        const bId = keyToBlack[code];
        activeBlackPressures[bId] = 0;
        const el = document.querySelector(`[data-black="${bId}"]`);
        if (el) {
          el.classList.remove('is-held');
          el.style.setProperty('--pressure', '0');
        }
        uiWasm.wasm_ui_black_key(bId, 0);
      } else if (code in keyToButton) {
        const btnId = keyToButton[code];
        const el = document.querySelector(`[data-button="${btnId}"]`);
        if (el) el.classList.remove('is-held');
        uiWasm.wasm_ui_button(btnId, 0);
      } else if (code in keyToWhite) {
        const wIdx = keyToWhite[code];
        activeWhitePressures[wIdx] = 0;
        const el = document.querySelector(`[data-white="${wIdx}"]`);
        if (el) {
          el.classList.remove('is-held');
          el.style.setProperty('--pressure', '0');
        }
        uiWasm.wasm_ui_white_gate(wIdx, 0);
      }
    }
    heldKeyboardCodes.clear();
    heldKeyboardBlackPads.clear();
    heldKeyboardPressurePads.clear();
    flushControlToWorklet();
  });
}

async function initWebMidi() {
  const badge = document.getElementById('midi-status');
  if (!navigator.requestMIDIAccess) {
    badge.textContent = 'MIDI: Unavailable in Browser';
    return;
  }

  try {
    const midi = await navigator.requestMIDIAccess({ sysex: false });
    const heldMidiBlackPads = new Set();
    const heldMidiWhitePads = new Set();
    const lastCcValues = new Int16Array(128).fill(-1);

    let midiDeviceCount = 0;
    let midiTimeOriginMs = -1;
    const renderMidiBadge = () => {
      const synced =
        uiWasm && uiPtrs && uiF32[uiPtrs.summary + WASM_SUMMARY.MIDI_CLOCK_SYNCED] > 0.5;
      const bpm = synced ? Math.round(uiF32[uiPtrs.summary + WASM_SUMMARY.EFFECTIVE_CLOCK_BPM]) : 0;
      const base =
        midiDeviceCount > 0
          ? `MIDI: ${midiDeviceCount} Device${midiDeviceCount > 1 ? 's' : ''} Active`
          : 'MIDI: Ready (0 Devices)';
      badge.textContent = synced ? `${base} · ${bpm} BPM` : base;
    };

    const handleMidiMessage = (ev) => {
      if (!uiWasm || !ev.data || ev.data.length < 1) return;
      const raw0 = ev.data[0];

      // 1-byte MIDI System Real-Time Clock & Transport (USB, TRS interface, or BLE-MIDI)
      if (raw0 >= 0xf8) {
        if (raw0 === 0xf8 || raw0 === 0xfa || raw0 === 0xfb || raw0 === 0xfc) {
          let tsSec = -1;
          if (typeof ev.timeStamp === 'number' && ev.timeStamp >= 0) {
            if (midiTimeOriginMs < 0 || ev.timeStamp < midiTimeOriginMs) {
              midiTimeOriginMs = ev.timeStamp;
            }
            tsSec = ((ev.timeStamp - midiTimeOriginMs) * 0.001) % 64;
          }
          uiWasm.wasm_ui_midi_realtime(raw0, tsSec);
          flushControlToWorklet();
          renderMidiBadge();
        }
        return;
      }

      if (ev.data.length < 2) return;

      const status = ev.data[0] & 0xf0;
      const msgCh = (ev.data[0] & 0x0f) + 1;
      const cfgCh = uiPtrs ? Math.round(uiF32[uiPtrs.summary + WASM_SUMMARY.MIDI_CHANNEL_IN]) : 0;
      const d1 = ev.data[1];
      const d2 = ev.data.length > 2 ? ev.data[2] : 0;
      const isNoteOff = status === 0x80 || (status === 0x90 && d2 === 0);
      if (cfgCh > 0 && msgCh !== cfgCh && !isNoteOff) return;

      ensureAudioRunning();

      // Note On / Note Off across F3..G5 (MIDI 53..79)
      if (status === 0x90 || status === 0x80) {
        const pressed = status === 0x90 && d2 > 0;
        const entry = MIDI_KEYBED_MAP[d1 - 53];
        if (!entry) return;

        if (entry.kind === 'black') {
          uiWasm.wasm_ui_black_key(entry.idx, pressed ? 1 : 0);
          const el = document.querySelector(`[data-black="${entry.idx}"]`);
          if (el) el.classList.toggle('is-held', pressed);
          if (pressed) {
            heldMidiBlackPads.add(entry.idx);
            const velP = clamp01(d2 / 127);
            activeBlackPressures[entry.idx] = velP;
            if (el) el.style.setProperty('--pressure', velP.toFixed(3));
            uiWasm.wasm_ui_black_pressure(entry.idx, velP);
          } else {
            heldMidiBlackPads.delete(entry.idx);
            activeBlackPressures[entry.idx] = 0;
            if (el) el.style.setProperty('--pressure', '0');
          }
        } else {
          uiWasm.wasm_ui_white_gate(entry.idx, pressed ? 1 : 0);
          const el = document.querySelector(`[data-white="${entry.idx}"]`);
          if (el) el.classList.toggle('is-held', pressed);
          if (pressed) {
            heldMidiWhitePads.add(entry.idx);
            const velP = clamp01(d2 / 127);
            activeWhitePressures[entry.idx] = velP;
            if (el) el.style.setProperty('--pressure', velP.toFixed(3));
            uiWasm.wasm_ui_white_pressure(entry.idx, velP);
          } else {
            heldMidiWhitePads.delete(entry.idx);
            activeWhitePressures[entry.idx] = 0;
            if (el) el.style.setProperty('--pressure', '0');
          }
        }
        flushControlToWorklet();
      } else if (status === 0xa0) {
        // Polyphonic Key Pressure (Aftertouch per note) across all 27 keys
        const entry = MIDI_KEYBED_MAP[d1 - 53];
        if (!entry) return;
        const p = clamp01(d2 / 127);
        if (entry.kind === 'black') {
          activeBlackPressures[entry.idx] = p;
          const el = document.querySelector(`[data-black="${entry.idx}"]`);
          if (el) el.style.setProperty('--pressure', p.toFixed(3));
          uiWasm.wasm_ui_black_pressure(entry.idx, p);
        } else {
          activeWhitePressures[entry.idx] = p;
          const el = document.querySelector(`[data-white="${entry.idx}"]`);
          if (el) el.style.setProperty('--pressure', p.toFixed(3));
          uiWasm.wasm_ui_white_pressure(entry.idx, p);
        }
        flushControlToWorklet();
      } else if (status === 0xd0) {
        // Channel Aftertouch -> applies to any held keybed pads
        const p = clamp01(d1 / 127);
        for (const bIdx of heldMidiBlackPads) {
          activeBlackPressures[bIdx] = p;
          const el = document.querySelector(`[data-black="${bIdx}"]`);
          if (el) el.style.setProperty('--pressure', p.toFixed(3));
          uiWasm.wasm_ui_black_pressure(bIdx, p);
        }
        for (const wIdx of heldMidiWhitePads) {
          activeWhitePressures[wIdx] = p;
          const el = document.querySelector(`[data-white="${wIdx}"]`);
          if (el) el.style.setProperty('--pressure', p.toFixed(3));
          uiWasm.wasm_ui_white_pressure(wIdx, p);
        }
        flushControlToWorklet();
      } else if (status === 0xb0) {
        // MIDI CC: CC 1 / 74 = Pressure, CC 7 = Master, CC 16..19 / 70..73 = KNOB1..4
        if (d1 === 1 || d1 === 74) {
          const p = clamp01(d2 / 127);
          for (const bIdx of heldMidiBlackPads) {
            activeBlackPressures[bIdx] = p;
            const el = document.querySelector(`[data-black="${bIdx}"]`);
            if (el) el.style.setProperty('--pressure', p.toFixed(3));
            uiWasm.wasm_ui_black_pressure(bIdx, p);
          }
          for (const wIdx of heldMidiWhitePads) {
            activeWhitePressures[wIdx] = p;
            const el = document.querySelector(`[data-white="${wIdx}"]`);
            if (el) el.style.setProperty('--pressure', p.toFixed(3));
            uiWasm.wasm_ui_white_pressure(wIdx, p);
          }
          flushControlToWorklet();
        } else if (d1 === 7) {
          masterPotValue = clamp01(d2 / 127);
          uiWasm.wasm_ui_master_pot(masterPotValue);
          updateMasterVisual();
          flushControlToWorklet();
        } else if ((d1 >= 16 && d1 <= 19) || (d1 >= 70 && d1 <= 73) || d1 === 20 || d1 === 21) {
          const encId =
            d1 === 20 ? 0 : d1 === 21 ? 2 : 3 + (d1 >= 70 ? d1 - 70 : d1 - 16);
          const prev = lastCcValues[d1];
          lastCcValues[d1] = d2;
          if (prev >= 0) {
            let ticks = Math.round((d2 - prev) * 2.0);
            if (ticks === 0 && d2 === 127 && prev === 127) ticks = 2;
            else if (ticks === 0 && d2 === 0 && prev === 0) ticks = -2;
            if (ticks !== 0) {
              sendEncoderSteps(encId, ticks);
            }
          }
        }
      }
    };

    const bindInputs = () => {
      midiDeviceCount = 0;
      for (const input of midi.inputs.values()) {
        input.onmidimessage = handleMidiMessage;
        midiDeviceCount++;
      }
      renderMidiBadge();
    };

    midi.onstatechange = bindInputs;
    bindInputs();
  } catch {
    badge.textContent = 'MIDI: Permission Denied';
  }
}

function startDisplayAndLedLoop() {
  const canvas = document.getElementById('tft-canvas');
  const ctx = canvas.getContext('2d', { alpha: false });
  const rgbaView = new Uint8ClampedArray(uiMem.buffer, uiPtrs.rgba, DISPLAY_W * DISPLAY_H * 4);
  const imageData = new ImageData(rgbaView, DISPLAY_W, DISPLAY_H);

  const blackKeyEls = Array.from({ length: 11 }, (_, i) =>
    document.querySelector(`[data-black="${i}"]`)
  );
  const whiteKeyEls = Array.from({ length: 16 }, (_, i) =>
    document.querySelector(`[data-white="${i}"]`)
  );
  const buttonEls = Array.from({ length: 14 }, (_, i) =>
    document.querySelector(`[data-button="${i}"]`)
  );

  let webAudioSmooth = 0;
  let lastFrameMs = performance.now();

  const frame = (nowMs) => {
    const dtSec = Math.min(0.1, Math.max(0.001, (nowMs - lastFrameMs) / 1000));
    lastFrameMs = nowMs;

    uiWasm.wasm_ui_tick_and_render(dtSec);
    flushControlToWorklet();
    ctx.putImageData(imageData, 0, 0);

    const webAudioActive = Boolean((audioCtx && audioCtx.state === 'running') || audioStarting);
    const webAudioTarget = webAudioActive ? 1 : 0;
    webAudioSmooth += (webAudioTarget - webAudioSmooth) * (1 - Math.exp(-dtSec * 12.5));
    if (Math.abs(webAudioTarget - webAudioSmooth) < 1e-4) {
      webAudioSmooth = webAudioTarget;
    }

    // Refresh all 42 hardware software-PWM LED channels:
    // [0..10] 11 white black keys, [11..26] 16 white keys, [27..38] 12 white buttons,
    // [39] Green PLAY LED (lit when stopped), [40] Red REC LED, [41] Red STOP LED (lit when playing)
    const ledsBase = uiPtrs.leds;
    for (let i = 0; i < 11; i++) {
      if (blackKeyEls[i]) {
        blackKeyEls[i].style.setProperty('--led', uiF32[ledsBase + i].toFixed(3));
      }
    }
    for (let i = 0; i < 16; i++) {
      if (whiteKeyEls[i]) {
        whiteKeyEls[i].style.setProperty('--led', uiF32[ledsBase + 11 + i].toFixed(3));
      }
    }
    for (let i = 0; i < 14; i++) {
      if (buttonEls[i] && i !== 12) {
        buttonEls[i].style.setProperty('--led', uiF32[ledsBase + 27 + i].toFixed(3));
      }
    }
    const playStopEl = buttonEls[12];
    if (playStopEl) {
      const wasmPlayG = clamp01(uiF32[ledsBase + 39]);
      const wasmStopR = clamp01(uiF32[ledsBase + 41]);
      const stopR = clamp01(webAudioSmooth * wasmStopR);
      const playG = clamp01((1 - webAudioSmooth) + webAudioSmooth * wasmPlayG);
      const rawR = 46 * playG + 255 * stopR;
      const rawG = 232 * playG + 56 * stopR;
      const rawB = 110 * playG + 44 * stopR;
      const peak = Math.max(rawR, rawG, rawB, 1e-4);
      const nr = Math.round((rawR / peak) * 255);
      const ng = Math.round((rawG / peak) * 255);
      const nb = Math.round((rawB / peak) * 255);
      const level = clamp01(Math.max(playG, stopR, (playG + stopR) * 0.75));
      playStopEl.style.setProperty('--led', level.toFixed(3));
      playStopEl.style.setProperty('--ps-r', String(nr));
      playStopEl.style.setProperty('--ps-g', String(ng));
      playStopEl.style.setProperty('--ps-b', String(nb));
      playStopEl.style.setProperty('--ps-color', `rgb(${nr}, ${ng}, ${nb})`);
    }

    requestAnimationFrame(frame);
  };

  requestAnimationFrame(frame);
}

async function main() {
  await bootUiWasm();
  document.getElementById('audio-toggle-btn').addEventListener('click', toggleAudioPower);
  bindKnobs();
  bindButtonsAndKeybed();
  bindComputerKeyboard();
  initWebMidi();
  startDisplayAndLedLoop();
}

main().catch((err) => {
  console.error('Failed to boot Ruido Simulator:', err);
});
