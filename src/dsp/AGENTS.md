# Real-Time Audio / DSP Rules (`src/dsp/`)

All code in `src/dsp/` executes on the hard real-time audio path (`Core 1` on hardware: 64 stereo frames @ `44,118 Hz` = `~1.45 ms` deadline; `176,472 Hz` internal `4x` oversampled rate).

## Mandatory Invariants

1. **Zero Dynamic Allocation:** Never call `new`, `delete`, `malloc`, `free`, or resize any buffer inside `src/dsp/`. All state lives in pre-allocated, cache-aligned structs initialized at boot.
2. **Single-Precision `float` Only (No `double`):** The `pi32v2` CPU has a hardware single-precision FPU (`float`), NOT double-precision. Always suffix floating-point literals with `f` (e.g., `0.5f`, `3.14159265f`) and use `sinf`/`tanf`/fast rational approximations—never `double`, which triggers slow software float emulation and causes audio dropouts.
3. **No Locks or Blocking Calls:** Never use mutexes, semaphores, logging, or I/O in DSP routines. Read control updates via lock-free atomic snapshots and write oscilloscope wave frames via a lock-free single-producer single-consumer (SPSC) ring buffer.
4. **Mandatory Per-Sample Parameter Smoothing:** Encoder ticks and mute toggles arrive at control rate. Every audible target (`frequency`, `amplitude`, `pan`, `hp_cutoff`, `lp_cutoff`, `resonance`, `drive`, `mute_gate`) must pass through clickless slewing before touching the signal path.
5. **Denormal & NaN Safety:** Flush denormals / subnormals to zero in feedback paths (ZDF filter integrators, RC tilt states, one-pole smoothers) and bound nonlinear feedback states so resonant self-oscillation can never diverge to `NaN`/`Inf`.
6. **Unified 4x Oversampling:** Keep oscillators, shared-rail coupling, per-voice `HPF -> LPF` ZDF filters, and master bus saturation in the `4x` oversampled domain; decimate once at the final stereo output stage.
