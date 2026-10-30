# Ruido — Technical & Hardware Architecture

## 1. Target Hardware Profile (`M-VAVE FM-1`)

| Subsystem | Hardware Component | Key Constraints & Notes |
| :--- | :--- | :--- |
| **SoC** | Zhuhai JieLi **AC791N** (`WL82`) | AIoT multimedia + BLE SoC |
| **CPU** | **Dual-core 32-bit `pi32v2`** @ **240-320 MHz** | Proprietary Blackfin-derived RISC/DSP ISA; hardware **single-precision FPU** (`float`), 32 KB I-Cache, 24 KB D-Cache, MMU |
| **SRAM** | **~578 KB internal SRAM** | Shared across FreeRTOS SMP kernel, `jlstream` audio, USB/BLE stacks, and our static/heap allocations |
| **Flash** | **1 MB XIP NOR Flash** (`0x02000000`) | Code executes in-place; **~600 KB strict ceiling** for our compiled `.fwsc` application + assets |
| **Audio I/O** | **44,118 Hz** stereo, **64-frame blocks** | `~1.45 ms` block callback budget. Stereo 3.5mm TRS headphone/line out (AC-coupled, ~1-2 Vpp), auto-muting internal speaker, plus USB-C digital stereo audio |
| **Display** | **1.54" `240x240` RGB565 TFT LCD** | Driven over high-speed SPI with DMA |
| **Rotary Controls** | **1x `MASTER` analog pot + 7x endless encoders** | Left: `SELECT`, `PRESETS`, `ALGORITHM` (no push switches); Right: `KNOB1`-`KNOB4` |
| **Buttons & Keys** | **14 tactile buttons + 27 silicone keys (`F3-G5`)** | Left `OCT-`/`OCT+`, right `2x6` button matrix, 11 labeled black keys + 16 white keys |
| **Keybed Pressure** | **Carbon-pill `SARADC` resistance sensing** | Matrix rows tied to `AC791N` SARADC pins (`PA7`/`PA8` technique) yield continuous per-key analog pressure without hardware mods |
| **LEDs** | **42 physical LED channels (27 keys + 14 buttons)** | `2 × 74HC595` software-PWM matrix: 27 white key LEDs, 12 white button LEDs (`OCT-/+`, `FX`..`SEQ`), dual-channel **Green `PLAY` + Red/Orange `STOP`** under `PLAY/STOP` (mixes to Yellow), and **Red** under `REC` |
| **Connectivity** | **USB-C** (MIDI + Audio + SysEx OTA), **BLE MIDI**, **3.5mm TRS MIDI IN** | Protected USB-MIDI SysEx bootloader allows safe `.fwsc` flashing & stock recovery |

> **Electrical Routing Note for Pedals & Eurorack:** Because the 3.5mm output is a **stereo TRS** headphone/line driver, users should use a **3.5mm TRS stereo breakout** (never a mono TS patch cable directly, which shorts the Right ring to ground). Output is AC-coupled (~1-2 Vpp), ideal for pedals and suited for Eurorack via an input boost/VCA module.

---

## 2. Dual-Core Execution Model

Ruido pins real-time DSP and UI/control scanning to separate `pi32v2` cores communicating strictly via **lock-free atomic snapshots and SPSC ring buffers**:

```text
┌─────────────────────────────────────────────┐       Lock-Free        ┌─────────────────────────────────────────────┐
│             CORE 0 (UI & I/O)               │    Control Snapshot    │          CORE 1 (Hard Real-Time DSP)        │
│                                             │ ─────────────────────> │                                             │
│ • GPIO + SARADC Key Matrix (Gate + Pressure)│                        │ • 44.118 kHz Modulation: 4x Analog Voice LFO│
│ • 7x Encoder Quadrature + 3-Gear Rate Scale │    Lock-Free SPSC +    │ • 4x Oversampled Domain (176,472 Hz):       │
│ • 42-Channel Hardware LED PWM Refresh       │  LFO/Gate Telemetry    │   - 4x Schmitt PolyBLEP Oscillators         │
│ • 240x16 Ping-Pong Scanline Strip Renderer  │ <───────────────────── │   - Shared Power-Rail Coupling (`ALGORITHM`)│
│ • SPI DMA Display Transfer (~45-55 FPS)     │                        │   - 4x Series ZDF MS-20 (HPF -> LPF)        │
│ • Snapshot Rubber-Band / Slew & Flash NVS   │                        │   - Clickless Mute/Pressure + Sin/Cos Pan   │
└─────────────────────────────────────────────┘                        │ • Nonlinear Sum + 4x -> 1x FIR Decimator    │
                                                                       └─────────────────────────────────────────────┘
```

---

## 3. DSP Pipeline (`src/dsp/`)

All synthesis and filtering run inside a unified **`4x` oversampled domain (`176,472 Hz`)** using single-precision `float` before decimating once to `44,118 Hz`.

### 3.1 Schmitt-Trigger Relaxation Oscillator Model (`4x` Voices)
1. **Band-Limited Step Generation:** 32-bit unsigned integer (`uint32_t`) phase accumulators (`41 uHz` resolution, immune to `float32` sub-audio ULP stall down to `0.02 Hz`) with **4-point Integrated Cubic B-Spline PolyBLEP** anti-aliasing on both rising and falling transitions (`< -108 dBc` alias floor `< 20 kHz`) plus subtle per-inverter CMOS hysteresis duty spread (`~49.2% - 50.9%`).
2. **RC Capacitor Tilt & DC Blocking:** Subtle leaky-integrator / high-pass roof sag (`~11.5 Hz` corner) modeling physical timing/coupling capacitor discharge across flat square tops.
3. **1/f Thermal Drift:** Independent slow filtered random walk (`±2.2 cents`) per oscillator so unison/octaves breathe organically.
4. **Shared Power-Rail Threshold Coupling (`ALGORITHM` Knob):** Each oscillator's transition feeds a transient crowbar/shoot-through pulse scaled by the `ALGORITHM` parameter (`0..100%`) into a virtual `Vdd` sag bus, nudging neighbor hysteresis thresholds (`Vt+`/`Vt-`) via a signed phase-response curve to reproduce analog `CD40106` Adler injection locking (`~1.6%` max capture range).

### 3.2 Per-Oscillator MS-20 `Rev 2` (`KLM-307` `LM13600` OTA Sallen-Key) Dual Filter (`8x` Nodal ZDF Stages)
Each oscillator feeds its dedicated **2-pole OTA Sallen-Key High-Pass Filter (`IC5`) -> 2-pole OTA Sallen-Key Low-Pass Filter (`IC6`)** controlled in classic MS-20 pairing by `HPF` (`KNOB1`), `HP RES` (`KNOB2`), `LPF` (`KNOB3`), and `LP RES` (`KNOB4`) (`ADR-019`, `ADR-024`):
- **Capacitive Sallen-Key Nodal Topology (Zero Post-Blocker DC & Quench-Free Dual Resonance):** `HPF` injects input $x$ through series capacitors `C1` ($v_{C1} = y_1 - x$) and buffered `C2` ($v_{C2} = y_2 - y_1$) with `OTA 1` discharging toward $\phi_{\text{diode}}(k_{\text{hp}} y_2)$ and `OTA 2` discharging toward ground—delivering exact `0 Hz` capacitive DC rejection (`-156 dB` measured leak) without any synthetic post-filter DC tracker. `LPF` senses `Stage 2` capacitor differential $w = y_1 - y_2 = \tilde{s} y_2$ into its `1S1588` limiter $\phi_{\text{diode}}(k_{\text{lp}} w)$, preserving the exact Sallen-Key denominator $\tilde{s}^2 + (2 - k)\tilde{s} + 1$ while keeping sub-cutoff bass fundamentals inside $w$'s band-pass null so simultaneous `HPF + LPF` self-oscillation never suffers Van der Pol bass quenching.
- **Pure-FPU $C^2$ `LM13600` Dual-OTA Poles & $C^4$-Deadband `1S1588` Silicon Limiters:** All 4 filter poles per voice model `LM13600` diode-linearized OTA transconductance via odd quintic $C^2$ saturation $\sigma_{\text{ota}}(v) = v_c(1 - \frac{2}{3}u^2 + \frac{1}{5}u^4)$ (`~1.08` nominal knee, monolithic `1.5%–2.6%` Wilson current-mirror tolerances balanced across nodes so steady-state limit-cycle DC stays `< 0.5 mV`), generating warm 2nd + 3rd harmonics (`H3 ≈ -45.8 dBc`, `H2 ≈ -61.2 dBc` @ `RES = 0%`, `< -108.6 dBc` alias floor) and organic square-edge slew rounding even at zero resonance. Both resonance paths use matched anti-parallel `1S1588` silicon PN-junction $C^4$-deadband / $C^2$-rail limiters $Q(u) = u(1 - \frac{3}{5}u^4 + \frac{2}{7}u^6) + R_s$ (`kMax = 3.68`) plus `KLM-307` passband-vs-peak dividers (`hpInScale`, `interStageScale`, `lpOutScale`).
- **Coupled 1D Analytical Newton-Raphson Solver, Smart-Park & 4-Voice Lockstep SIMD:** Reducing all 3 nonlinear devices (`OTA 1`, `OTA 2`, `1S1588` limiter) per stage to a single scalar unknown ($y_2$ for `HPF`, $w$ for `LPF`) with ultrasonic loop-gain bounding (`df > 0.12` globally across $\mathbb{R}^2$), continuous 2-point capacitor-state predictors (`q = y2 - x`, `w = y1 - y2`), and Groszkowski reactive-power + dual-OTA compression pitch curves achieves `< 0.5%` self-oscillation pitch accuracy (`200 Hz` & `1 kHz`), transparent Smart-Park linear bypass (`20..24 Hz` `HPF`, `18.8..20 kHz` `LPF`), and bit-exact 4-voice SIMD execution via `Ms20DualFilter::processQuadBank4x` (`sizeof(Ms20DualFilter) = 32 B`).

### 3.3 Per-Voice Analog-Modeled LFO Bank (`4x` Independent Voice LFOs)
Each voice (`Osc 1..4`) owns a dedicated analog-modeled modulation LFO (`VoiceLfoBank` + shared zero-`libm` `LfoMath.h`, `ADR-022`) stepped once per `44,118 Hz` output frame:
- **32-Bit Phase Accumulator & Capacitor Drift:** `uint32_t` phase accumulators (`~10.3 uHz` resolution, exact `50.0 s` cycles at `0.02 Hz` up to `30.0 Hz` audio-edge flutter/FM) initialized in `{0°, 90°, 180°, 270°}` quadrature with per-voice `1%` polystyrene timing-capacitor tolerances and slow thermal random walk.
- **Stepless Analog Waveform Morph (`SHAPE` `0..100%`):** Continuous morph across `C^2` folded polynomial `Sine` + JFET Padé crest warmth (`0.00`), MS-20 variable-symmetry `Triangle -> Sawtooth` (`0.25 -> 0.50`) with `C^1` quadratic vertex fillets, exact analytical `1 - k*r*(1-r)` peak normalization, rate-adaptive capacitor flyback, and exact `0.0` DC integral, vactrol-rounded `C^1` cubic `Slewed Square` (`0.75`), and non-repeating 2-segment **`C^2` Uniform Cubic B-Spline `Smooth Drift`** (`1.00`) driven by a phase-clocked 5-knot Sloth anti-clump zero-mean feedback generator (`packDriftKnots` packs 5 signed 6-bit knots into a single `30-bit` atomic `uint32_t` in `EngineTelemetry` so Core 0 renders the exact live spline terrain).
- **Clickless 7-Target Crossfade & Headroom-Aware Modulation Laws:** Switching `KNOB 1` (`TARGET`) live crossfades per-destination weights (`~12.5 ms` tau) across **`LPF`**, **`LP RES`**, **`HPF`**, **`HP RES`**, **`AMP`**, **`PITCH`**, and **`PAN`**, while headroom-aware octave/resonance/pan windowing shifts parked centers inward (`20 Hz` HPF, `20 kHz` LPF, `0%` RES, hard `L/R` PAN) so sweeps never flat-top against rails.

### 3.4 Mixing, Equal-Power Pan, Master Bus Sag & Polyphase Decimation
- **Stepless `C^1` Log-Potentiometer Smoothing, Vactrol Mute, `SEL`-Routed Pressure & Pan:** Continuous rotary controls (`freqHz`, `hpCutoffHz`, `lpCutoffHz`, `lfoRateHz` in `log2(Hz)` octave space via `LogFreqSmoother`; `amplitude`, `pan`, `hpResonance`, `lpResonance`, `lfoShape`, `lfoDepth`, `circuitCoupling`, `masterTrim` via `CriticallyDampedSmoother`) use 2-pole critically-damped cascades (`14 ms/pole`, `~28 ms` mean S-curve glide, `0%` overshoot, zero stationary FPU cost) to fuse discrete encoder transitions into stepless analog sweeps; gates/modes use `3.8-5.0 ms` one-pole slews, paired with an asymmetric `VactrolSmoother` (`4.5 ms` optical LED attack / `26.0 ms` CdS photoconductive release) driving White Keys `5-8`.
- **Master Bus Sag:** High-headroom (`0.75` linear knee, `±1.25` soft ceiling) transformer/op-amp summing bus restoring subtle intermodulation glue when multiple hot voices collide.
- **Stereo Decimation (`176.472 kHz -> 44.118 kHz`):** Two cascaded 2x **Linear-Phase Symmetric Polyphase Half-Band FIR** stages (`Stage 1`: 31-tap `P=8`, `Stage 2`: 127-tap `P=32`, Kaiser-windowed) using double-mirrored unit-stride odd rings and 4-way unrolled stereo accumulators (`> 87 dB` stopband rejection to `20 kHz`, `< 0.0005 dB` passband ripple, `0°` phase dispersion).

### 3.5 Cycle Budget Estimate (Core 1 @ 240-320 MHz)
Available cycles per `44.118 kHz` stereo frame: **`~5,440 - 7,250` cycles**.
- `4x` Schmitt PolyBLEP + drift + rail coupling (@ `4x` OS): `~650` cycles
- `8x` Nonlinear `Rev 2` OTA Sallen-Key ZDF stages (`4 voices x [HPF + LPF]` @ `4x` OS, 4-lane lockstep): `~1,550` cycles
- `4x` Per-voice analog LFOs + `C^2` B-spline drift + 7-target modulation laws (@ `1x` base rate): `~180` cycles
- Slews, pressure routing, stereo pan, master bus saturation & cascaded half-band stereo decimator: `~450` cycles
- **Estimated Total:** **`~2,830` cycles/frame (`~39-52%` of Core 1)**, leaving ~50% safety margin (`sizeof(DroneEngine) = 45.50 KB`; `29.0x` real-time single-core host throughput).

---

## 4. Keybed `SARADC` Pressure & Scanline Strip UI (`src/ui/`)

### 4.1 Carbon-Pill Polyphonic Pressure Scanner (`Core 0`)
- Between digital matrix scans, Core 0 pulses weak GPIO pull-ups and reads the `AC791N` `SARADC` channels tied to the silicone key matrix rows, mapping nonlinear carbon-pill resistance drop through a calibrated curve + low-pass slew into continuous `[0.0f .. 1.0f]` per-key pressure values (mirrored in `platforms/sim/` via mouse drag/wheel pressure + MIDI Poly/Channel Aftertouch).

### 4.2 `240x16` Coupled 4-Voice Caustic Ribbon Scope & Modular Optical-Filament HUD Compositor
- **Ping-Pong Strip Buffers & Compact Zero-Framebuffer State:** Two `240 x 16` pixel RGB565 buffers (`7,680 bytes` each = **`15.36 KB`**) plus `85.85 KB` static `ScopeHudRenderer` state (`4096`-sample compact `12 B` `Q3.12` stereo + 4-voice/rising-edge scope history covering down to `20 Hz` and multi-cycle super-periods, continuous phosphor persistence states, `14` `Q8.6` strands with `8`-column `[minY, maxY]` tile-culling bounds, a `60x60` `uint16_t` quarter-resolution bloom grid, slewed `4 x 221` + `2 x 221` `Q8.6` MS-20 filter curves, dedicated full-screen `4 x 241` `Q8.6` per-voice LFO curves, and `5` synchronization-unified `150 ms` center-view fade envelopes), avoiding any `115.2 KB` `240x240` framebuffer while streaming at ~50-60 FPS over SPI DMA (`~19-22%` of `Core 0`).
- **Coupled 14-Strand Circuit Mesh, Super-Period Lock & Zero-Hard-Cut Galvanometer Relaxation (`ADR-018`, `ADR-023`):** Core 1 delays 4x-averaged `int16` (`Q2.13`, `0.0095 px` precision) per-voice post-MS-20 taps and 4-voice rising-edge flags by `36` base frames to align sample-accurately with the linear-phase FIR decimated stereo bus. On Core 0, `ScopeTraceEngine::prepare` scans up to `8` rising-edge candidates across `4096` samples (`~93 ms`), refines each to its continuous sub-sample post-MS-20 zero-crossing (immune to filter group delay), locks onto the multi-cycle harmonic super-period via a `48`-tap per-voice similarity signature across a continuous Catmull-Rom `smoothStride_` (`[1.0f, 2.0f]`, eliminating `1x <-> 2x` bass-zoom snaps), applies continuous `38–190 ms` phosphor/galvanometer slewing (smoothly relaxing all `14` strands into the `y = 120` silent horizon when `PLAY/STOP` stops, `MASTER = 0%`, all voices mute, or the audio stream pauses, and smoothly expanding back on resume), splats analytical column photon energy into the `60x60` grid, and runs a 2-pass in-place separable `[1, 3, 4, 3, 1] / 12` blur calibrated at `25%` bloom intensity.
- **Tile-Culled Linear Photon Rasterizer, Multi-View Center Graphs & `4x4` Bayer Neutral RGB565 (`ADR-020`, `ADR-022`, `ADR-023`, `ADR-027`):** `ScopeHudRenderer::renderChunk` processes stateless `<= 8`-row sub-chunks (`5.76 KB` stack), accumulates `64`-entry LUT Gaussian strand capsules in linear photon space (`uint16_t`), adds exact eighth-step integer bilinearly upsampled `60x60` bloom, maps through a `1024`-entry `constexpr` soft-knee Reinhard + `0.82` gamma LUT (`1.12 KB` XIP Flash), applies a centralized `150 ms` S-curve `72%` background waveform dimming envelope (`HudEnvelope.h`) strictly behind active center views (`MS-20 Filter`, `LFO`, `GLOBAL FREQ`, `GLOBAL AMP`, `GLOBAL ALL`, and `GLOBAL CONFIG`), composites `HudGraphs` + `HudGlobalViews` + `HudChrome` (`10 / 13 pt` Medium + `13 pt` Bold `HelveticaNeue` ROM tables, `7.70 KB` XIP Flash, `0 B` SRAM, plus Option A `FINE`/`COARSE` split top-bar override and 4-voice spread micro-beads), and packs strictly neutral monochrome RGB565 (`R5 == B5`, `G6 == (R5 << 1) | (R5 >> 4)`) with `4x4` ordered Bayer dithering (`~496` effective monochrome shades, zero contour banding).

---

## 5. Modular Codebase Hygiene & Cross-Platform Portability (`src/` <-> `platforms/`)

All handwritten files in `src/` are strictly capped at **`<= 700` lines**, organized as a one-way subsystem DAG (`src/core` <- `src/dsp` and `src/core` <- `src/ui`, zero cross-includes between `dsp` and `ui`), and verified automatically on every `make test` via `tools/check_hygiene.py`:
- **`src/core/` (`sizeof(RuidoController) = 720 B`):** `RuidoController` (orchestration), `ModifierTracker` (LIFO black-key & `OCT-/+` stacks), `HudEnvelope` (`150 ms` centralized S-curve timer), `EncoderMath` (3-gear linear X4 scaling + gear-invariant `4`-tick detents), `GlobalControl` (orthogonal `GLOBAL FREQ/FILTER/LFO/AMP/ALL` transforms + `SNAP BACK` snapshot), `GlobalConfig` (data-driven `GLOBAL CONFIG` table), `LedEngine` (`42`-channel hardware PWM mapper).
- **`src/ui/` (`sizeof(ScopeHudRenderer) = 85,848 B`):** `ScopeHudRenderer` (chunk pipeline & tonemap), `ScopeTraceEngine` (trigger lock & `60x60` bloom), `HudGraphs` (MS-20 & LFO plots), `HudGlobalViews` (`GLOBAL FREQ/AMP/ALL` & `GLOBAL CONFIG` list), `HudChrome` + `HudColumnText` + `HudFormat` (top/bottom chrome, Option A `FINE`/`COARSE` header, 4-voice micro-beads, zero-`snprintf` formatters).

```text
          ┌──────────────────────────────────────────┐
          │     Portable Modular Engine (src/)       │
          │  • src/dsp/  DroneEngine (45.50 KB)      │
          │  • src/ui/   ScopeHudRenderer (85.85 KB) │
          │  • src/core/ RuidoController (720 B)     │
          └────────────────────┬─────────────────────┘
                               │  Pure C++17 Structs (Zero OS/SDK Headers)
             ┌─────────────────┴─────────────────┐
             ▼                                   ▼
┌──────────────────────────┐       ┌──────────────────────────┐
│     platforms/sim/       │       │     platforms/fm1/       │
│ • WebAudio / CoreAudio   │       │ • JieLi AC791N / FreeRTOS│
│ • Exact FM-1 Panel UI    │       │ • jlstream 64-smp Audio  │
│   (7 Encoders + Master + │       │ • SPI LCD DMA + Key/LEDs │
│    27 Pressure Keys +    │       │ • SARADC Carbon Pressure │
│    14 Buttons + 240x240) │       │ • .fwsc SysEx Packaging  │
└──────────────────────────┘       └──────────────────────────┘
```

Every DSP sample and every `240x240` RGB565 pixel produced in the simulator is **bit-for-bit / pixel-for-pixel identical** to the hardware code path.
