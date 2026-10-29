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
| **Rotary Controls** | **1x `MASTER` analog pot + 7x endless encoders** | Left: `SELECT` (push-click), `PRESETS`, `ALGORITHM`; Right: `KNOB1`-`KNOB4` |
| **Buttons & Keys** | **14 tactile buttons + 27 silicone keys (`F3-G5`)** | Left `OCT-`/`OCT+`, right `2x6` button matrix, 11 labeled black keys + 16 white keys |
| **Keybed Pressure** | **Carbon-pill `SARADC` resistance sensing** | Matrix rows tied to `AC791N` SARADC pins (`PA7`/`PA8` technique) yield continuous per-key analog pressure without hardware mods |
| **LEDs** | **27 key LEDs + 14 button LEDs** | Monochrome white; per-LED software PWM brightness |
| **Connectivity** | **USB-C** (MIDI + Audio + SysEx OTA), **BLE MIDI**, **3.5mm TRS MIDI IN** | Protected USB-MIDI SysEx bootloader allows safe `.fwsc` flashing & stock recovery |

> **Electrical Routing Note for Pedals & Eurorack:** Because the 3.5mm output is a **stereo TRS** headphone/line driver, users should use a **3.5mm TRS stereo breakout** (never a mono TS patch cable directly, which shorts the Right ring to ground). Output is AC-coupled (~1-2 Vpp), ideal for pedals and suited for Eurorack via an input boost/VCA module.

---

## 2. Dual-Core Execution Model

Ruido pins real-time DSP and UI/control scanning to separate `pi32v2` cores communicating strictly via **lock-free atomic snapshots and SPSC ring buffers**:

```text
┌─────────────────────────────────────────────┐       Lock-Free        ┌─────────────────────────────────────────────┐
│             CORE 0 (UI & I/O)               │    Control Snapshot    │          CORE 1 (Hard Real-Time DSP)        │
│                                             │ ─────────────────────> │                                             │
│ • GPIO + SARADC Key Matrix (Gate + Pressure)│                        │ • 4x Oversampled Domain (176,472 Hz):       │
│ • 7x Encoder Quadrature + Velocity Ballistic│    Lock-Free SPSC      │   - 4x Schmitt PolyBLEP Oscillators         │
│ • Varibright White LED PWM Refresh          │   Scope Trace + Peak   │   - Shared Power-Rail Coupling (`ALGORITHM`)│
│ • 240x16 Ping-Pong Scanline Strip Renderer  │ <───────────────────── │   - 4x Series ZDF MS-20 (Drive + HPF -> LPF)│
│ • SPI DMA Display Transfer (~45-55 FPS)     │                        │   - Clickless Mute/Pressure + Sin/Cos Pan   │
│ • Snapshot Rubber-Band / Slew & Flash NVS   │                        │   - Nonlinear Master Summing Stage          │
└─────────────────────────────────────────────┘                        │ • Stereo Polyphase 4x -> 1x Decimator       │
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

### 3.2 Per-Oscillator MS-20 (`Korg35`) Dual Filter (`8x` ZDF Stages Total)
Each oscillator feeds **Pre-Filter Drive (`KNOB4`)** into its dedicated **2-pole Sallen-Key High-Pass Filter -> 2-pole Sallen-Key Low-Pass Filter**:
- **Topology-Preserving Transform (TPT / ZDF):** Trapezoidal integrators with Picard warm-started, **fused single-division Newton-Raphson** resolution of the delay-free nonlinear feedback loop (`ADR-010`) and Van der Pol period elongation compensation above `k = 2.0`—zero `z^-1` unit-delay cramping, `< 0.6%` self-oscillation pitch accuracy to `8 kHz`, and unconditional stability (`F'(w) >= 0.20 > 0`).
- **Asymmetric Diode Clipper in Feedback:** `C^2`-at-origin asymmetric Padé diode clipper (`+0.92 / -1.08`) inside both `HPF` and `LPF` Sallen-Key feedback paths so small-signal Butterworth passband stays `100%` linear while hot resonance generates rich even + odd harmonics decoupled from post-filter mix `Amplitude`.
- **Smart-Park Shared Resonance:** Smooth `smoothstep` edge taper (`Q -> 0.7071`, internal poles gliding to `2.5 Hz` / `28 kHz`) as `HP Cutoff -> 20 Hz` or `LP Cutoff -> 20 kHz`, preventing sub-sonic rumble, bass square-wave roof tilt, or ultrasonic self-oscillation on parked stages.

### 3.3 Mixing, Stereo Spread, Master Bus Sag & Polyphase Decimation
- **Clickless Vactrol Mute, Pressure Swell & Pan:** Per-sample one-pole slews (`3.8-5.0 ms` time constants with ULP-safe target snap) on mutes, continuous `SARADC` key-pressure modulation, post-filter amplitudes, and minimax constant-power (`sin`/`cos`) stereo pan weights (scaled by `SELECT` stereo spread or collapsed bit-identically in `MONO` mode).
- **Master Bus Sag:** High-headroom (`0.75` linear knee, `±1.25` soft ceiling) transformer/op-amp summing bus restoring subtle intermodulation glue when multiple hot voices collide.
- **Stereo Decimation (`176.472 kHz -> 44.118 kHz`):** Two cascaded 2x **Linear-Phase Symmetric Polyphase Half-Band FIR** stages (`Stage 1`: 31-tap `P=8`, `Stage 2`: 127-tap `P=32`, Kaiser-windowed) using double-mirrored unit-stride odd rings and 4-way unrolled stereo accumulators (`> 87 dB` stopband rejection to `20 kHz`, `< 0.0005 dB` passband ripple, `0°` phase dispersion).

### 3.4 Cycle Budget Estimate (Core 1 @ 240-320 MHz)
Available cycles per `44.118 kHz` stereo frame: **`~5,440 - 7,250` cycles**.
- `4x` Schmitt PolyBLEP + drift + rail coupling (@ `4x` OS): `~650` cycles
- `8x` Nonlinear ZDF Sallen-Key stages (`4 voices x [HPF + LPF]` @ `4x` OS): `~1,450` cycles
- Slews, pressure routing, stereo pan, master bus saturation & cascaded half-band stereo decimator: `~450` cycles
- **Estimated Total:** **`~2,550` cycles/frame (`~35-47%` of Core 1)**, leaving >50% safety margin.

---

## 4. Keybed `SARADC` Pressure & Scanline Strip UI (`src/ui/`)

### 4.1 Carbon-Pill Polyphonic Pressure Scanner (`Core 0`)
- Between digital matrix scans, Core 0 pulses weak GPIO pull-ups and reads the `AC791N` `SARADC` channels tied to the silicone key matrix rows, mapping nonlinear carbon-pill resistance drop through a calibrated curve + low-pass slew into continuous `[0.0f .. 1.0f]` per-key pressure values (mirrored in `platforms/sim/` via mouse drag/wheel pressure + MIDI Poly/Channel Aftertouch).

### 4.2 `240x16` Scanline Strip Scope & HUD Compositor
- **Ping-Pong Strip Buffers:** Two `240 x 16` pixel RGB565 buffers (`7,680 bytes` each = **`15.36 KB` total SRAM**), fitting inside Core 0's `24 KB` L1 D-Cache while SPI DMA streams at ~45-55 FPS.
- **Osc-1 Triggered CRT Scope:** Rising-edge synced on Oscillator 1 with vertical min/max phosphor intensity columns and `L`/`R` stereo trace layering.
- **Alpha-Blended Quasimode HUD:** Fixed-point 5-bit alpha blending (`0..32`) composites active typography, level/pan bars, or `[ HP |══════| LP ]` curves over a ~30%-dimmed waveform backdrop (`~120 ms` bloom, `1.5 s` hold, `~800 ms` dissolve).

---

## 5. Cross-Platform Portability (`platforms/sim/` <-> `platforms/fm1/`)

```text
          ┌──────────────────────────────────────────┐
          │        Portable Engine (src/)            │
          │  • RuidoEngine::processAudioBlock(...)   │
          │  • RuidoUI::renderStrip(y0, h, rgb565*)  │
          │  • RuidoController::onInputEvent(...)    │
          └────────────────────┬─────────────────────┘
                               │  Pure C++17 Structs (PlatformHAL)
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
