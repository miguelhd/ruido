# Real-Time Audio / DSP Rules (`src/dsp/`)

All code in `src/dsp/` executes on the hard real-time audio path (`Core 1` on hardware: 64 stereo frames @ `44,118 Hz` = `~1.45 ms` deadline; `176,472 Hz` internal `4x` oversampled rate).

## Mandatory Invariants

1. **Zero Dynamic Allocation:** Never call `new`, `delete`, `malloc`, `free`, or resize any buffer inside `src/dsp/`. All state lives in pre-allocated, cache-aligned structs initialized at boot.
2. **Single-Precision `float` Only, Strict IEEE-754 Flags & Division Discipline:** The `AC791N` `pi32v2` CPU emits hardware single-precision FPU instructions (`fadds`, `fsubs`, `fmuls`, `fdivs`) under `-target pi32v2 -mcpu=r3 -mfprev1`, NOT double-precision (`double` triggers banned `soft-double` helpers).
   - Always suffix literals with `f` (`-Wdouble-promotion -Werror`) and avoid 64-bit integer divisions (`__aeabi_ldivmod`).
   - **Never pass `-ffast-math`:** Compile strictly with `-fno-math-errno -ffp-contract=off` so `(x + 1e-18f) - 1e-18f` subnormal scrubbing (`scrubFilterState`) and `NaN`/`Inf` safety checks (`clampf`, `sanitizeState`) are never optimized away and simulator-to-hardware bit parity holds.
   - **Minimize `float / float` in `4x` loops:** Although `-mcpu=r3 -mfprev1` emits hardware `fdivs`, it is still a multi-cycle operation compared to `fmuls`; hoist loop-invariant reciprocals (`invHpRail`, `invLpRail`) to frame/block rate and use **2-step** Newton-Raphson IEEE-754 reciprocals (`< 1.5e-6` / `~-116 dB` error; never 1-step) when optimizing inner-loop divisions.
3. **No Locks or Blocking Calls:** Never use mutexes, semaphores, logging, or I/O in DSP routines. Read control updates via lock-free atomic snapshots and write oscilloscope wave frames via a lock-free single-producer single-consumer (SPSC) ring buffer.
4. **Mandatory Per-Sample Parameter Smoothing:** Encoder ticks and mute toggles arrive at control rate. Every audible target (`frequency`, `amplitude`, `pan`, `hp_cutoff`, `lp_cutoff`, `resonance`, `drive`, `mute_gate`) must pass through clickless slewing before touching the signal path.
5. **Denormal & NaN Safety:** Flush denormals / subnormals to zero in feedback paths (ZDF filter integrators, RC tilt states, one-pole smoothers) and bound nonlinear feedback states so resonant self-oscillation can never diverge to `NaN`/`Inf`.
6. **Unified 4x Oversampling:** Keep oscillators, shared-rail coupling, per-voice `HPF -> LPF` ZDF filters, and master bus saturation in the `4x` oversampled domain; decimate once at the final stereo output stage.
