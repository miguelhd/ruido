// Core 1 Real-Time DSP AudioWorkletProcessor for Ruido.
// Instantiates the freestanding `ruido.wasm` C++17 engine on the dedicated WebAudio
// rendering thread with zero main-thread blocking and zero SharedArrayBuffer/COOP headers.

class RuidoAudioProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.ex = null;
    this.memory = null;
    this.u8 = null;
    this.f32 = null;
    this.ptrs = null;
    this.blockCounter = 0;

    this.port.onmessage = (event) => {
      const msg = event.data;
      if (!msg) return;

      if (msg.type === 'init') {
        this.initWasm(msg.wasmBytes, msg.initialControl);
      } else if (msg.type === 'control' && this.ex && this.u8) {
        const src = msg.bytes;
        if (src && src.byteLength === this.ptrs.controlSize) {
          this.u8.set(new Uint8Array(src), this.ptrs.control);
          this.ex.wasm_dsp_publish_control(msg.snap ? 1 : 0);
        }
      }
    };
  }

  async initWasm(wasmBytes, initialControl) {
    try {
      const memory = new WebAssembly.Memory({ initial: 16, maximum: 16 });
      const stackPointer = new WebAssembly.Global({ value: 'i32', mutable: true }, 16 * 65536);
      const table = new WebAssembly.Table({ initial: 8, element: 'anyfunc' });

      const result = await WebAssembly.instantiate(wasmBytes, {
        env: {
          __linear_memory: memory,
          __stack_pointer: stackPointer,
          __indirect_function_table: table,
        },
      });

      const ex = result.instance.exports;
      ex.wasm_dsp_init(sampleRate);

      this.memory = memory;
      this.u8 = new Uint8Array(memory.buffer);
      this.f32 = new Float32Array(memory.buffer);
      this.ptrs = {
        control: ex.wasm_dsp_get_control_ptr(),
        controlSize: ex.wasm_dsp_get_control_size(),
        left: ex.wasm_dsp_get_left_ptr() >> 2,
        right: ex.wasm_dsp_get_right_ptr() >> 2,
        scopeOut: ex.wasm_dsp_get_scope_ptr(),
        scopeFrameSize: ex.wasm_dsp_get_scope_frame_size(),
      };
      this.ex = ex;

      if (initialControl && initialControl.byteLength === this.ptrs.controlSize) {
        this.u8.set(new Uint8Array(initialControl), this.ptrs.control);
        this.ex.wasm_dsp_publish_control(1);
      }

      this.port.postMessage({ type: 'ready', sampleRate });
    } catch (err) {
      this.port.postMessage({ type: 'error', message: String(err) });
    }
  }

  process(_inputs, outputs) {
    const output = outputs[0];
    if (!output || output.length === 0) {
      return true;
    }
    const outL = output[0];
    const outR = output.length > 1 ? output[1] : null;
    const frames = outL.length;

    if (!this.ex) {
      outL.fill(0);
      if (outR) outR.fill(0);
      return true;
    }

    this.ex.wasm_dsp_process_block(frames);

    const srcL = this.ptrs.left;
    const srcR = this.ptrs.right;
    outL.set(this.f32.subarray(srcL, srcL + frames));
    if (outR) {
      outR.set(this.f32.subarray(srcR, srcR + frames));
    }

    // Stream decimated scope frames + LED gate telemetry to Core 0 UI at ~70-85 Hz
    this.blockCounter = (this.blockCounter + 1) & 3;
    if (this.blockCounter === 0) {
      const nScope = this.ex.wasm_dsp_pop_scope(256);
      const telPtr = this.ex.wasm_dsp_read_telemetry() >> 2;
      const telemetry = this.f32.slice(telPtr, telPtr + 26);

      let scopeBuffer = null;
      if (nScope > 0) {
        const byteLen = nScope * this.ptrs.scopeFrameSize;
        scopeBuffer = this.u8.slice(this.ptrs.scopeOut, this.ptrs.scopeOut + byteLen).buffer;
      }

      if (scopeBuffer) {
        this.port.postMessage({ type: 'telemetry', scopeBuffer, scopeCount: nScope, telemetry }, [
          scopeBuffer,
        ]);
      } else {
        this.port.postMessage({ type: 'telemetry', scopeBuffer: null, scopeCount: 0, telemetry });
      }
    }

    return true;
  }
}

registerProcessor('ruido-audio-processor', RuidoAudioProcessor);
