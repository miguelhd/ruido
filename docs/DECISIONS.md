# Ruido — Architecture & Product Decision Log (ADRs)

This file tracks foundational engineering and UX decisions for **Ruido**. Append new records chronologically when making significant changes; mark superseded decisions rather than deleting history.

---

## ADR-001: Target Hardware & Cross-Platform Simulator-First Architecture
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Target hardware is the **M-VAVE FM-1** (`JieLi AC791N / pi32v2`, ~578 KB SRAM, ~600 KB XIP Flash app slot), which requires a Linux-distributed custom Clang (`pi32v2`) toolchain, while primary development happens on macOS.
- **Decision:** Isolate 100% of DSP, control logic, and `240x240` RGB565 scanline rendering inside platform-agnostic C++17 (`src/`), wrapped by two thin targets: `platforms/sim/` (native macOS / WebAssembly simulator with real-time audio & faithful front-panel UI) and `platforms/fm1/` (Dockerized `pi32v2` firmware build).
- **Consequences:** Sub-second iteration for tuning sound quality and UX on macOS using external pedals/Eurorack before flashing `.fwsc` images, with zero divergence between simulator and hardware code.

---

## ADR-002: Unified `4x` Oversampled (`176.47 kHz`) End-to-End Voice & Filter Path
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Naive digital square waves and 44.1 kHz nonlinear filters generate non-harmonic aliasing that external fuzz, distortion, wavefolder, and resonant Eurorack modules severely amplify. Because Ruido runs only 4 continuous drone voices (no polyphonic voice-stealing or sample playback), Core 1 has ~5,440-7,250 FPU cycles per stereo frame available.
- **Decision:** Run the entire synthesis and nonlinear filtering chain—oscillators, shared-rail coupling, per-voice ZDF filters, and master bus saturation—at **`4x` oversampling (`176,472 Hz`)** in single-precision `float`, decimating once at the final stereo output via cascaded polyphase half-band filters.
- **Consequences:** Uses ~35-47% of Core 1 while eliminating audible harmonic foldback and intermodulation aliasing into external drive pedals.

---

## ADR-003: CMOS Schmitt-Trigger Circuit Modeling (`CD40106` Reference)
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Standard digital square oscillators sound sterile and phase-cancel mechanically compared to the creator's reference DIY 3-oscillator analog Schmitt-trigger synth.
- **Decision:** Model three physical CMOS traits atop PolyBLEP square generation: (1) subtle RC capacitor roof tilt, (2) independent slow `1/f` thermal pitch drift per voice, and (3) weak **shared power-rail threshold coupling** controlled hands-on by the left-panel **`ALGORITHM` encoder** (`Vdd` sag nudging neighbor hysteresis thresholds so nearly-tuned oscillators pull and lock organically).
- **Consequences:** Captures authentic DIY CMOS drone interaction with direct front-panel control over circuit coupling.

---

## ADR-004: Per-Oscillator MS-20 (`Korg35`) Dual `HPF -> LPF` ZDF Filters with Smart-Park Shared Resonance & Pre-Filter Drive
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Evaluated a single shared stereo master filter vs. 4 independent per-oscillator filters, `LP/BP/HP` mode switches vs. series `HPF -> LPF`, and `Knob 4 = Pan` vs. `Pre-Filter Drive`.
- **Decision:**
  1. Give **each of the 4 oscillators its own mono series `HPF -> LPF` Korg35 ZDF filter** prior to constant-power stereo panning (plus a nonlinear master summing bus afterward to preserve multi-voice intermodulation glue when pushed).
  2. Keep the 4-knob filter strip 100% dedicated to filter physics (`KNOB1: HP Cutoff`, `KNOB2: LP Cutoff`, `KNOB3: Shared Resonance`, `KNOB4: Pre-Filter Drive`), decoupling diode saturation grit from post-filter voice `Amplitude`, and keeping `Pan` cleanly separated as a voice-mixer property.
  3. Use **Smart-Park Taper** on Shared Resonance: resonance tapers to flat (`Q = 0.707`) when `HP` is parked at `20 Hz` or `LP` is parked at `20 kHz`, preventing unwanted sub-rumble or ultrasonic whistle while waking up twin MS-20 formant peaks when both cutoffs move inward.
- **Consequences:** Eliminates filter mode switches, keeps filter vs. mixer mental models clean, and fits comfortably under 50% of Core 1 CPU.

---

## ADR-005: Stateless Two-Hand Ergonomics & Silkscreen-Matched Front Panel (`Approach A`)
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** On the physical FM-1 faceplate (`docs/assets/fm1-front-panel.png`), `KNOB1-4` and the `2x6` button matrix both sit to the **right** of the center `240x240` screen, while `MASTER`, `SELECT`, `PRESETS`, `ALGORITHM`, `OCT-`/`OCT+`, and black keys `OP1-OP6`/`PIT`/`GLO` sit to the **left / bottom-center**. Holding a right-matrix button with the left hand would cross the user's arm straight over the screen.
- **Decision:**
  1. **Approach A on `KNOB1-4`:** Idle `KNOB1-4` *always* control `Osc 1-4 Speed`.
  2. **All `KNOB1-4` Momentary Holds Live on Left/Bottom-Center Black Keys:** Hold `OP1-OP4` = Osc 1-4 Filter; Hold `GLO` (bottom key) = All 4 Filters; Hold `OP5` = Osc 1-4 Amplitude; Hold `OP6` = Osc 1-4 Pan. Hands never cross the display.
  3. **Right `2x6` Button Grid = Single-Tap Only:** Only literal single-tap labels (`PLAY/STOP`, `HOME`, `SAVE`, `LFO`) are active; unused FM/sequencer buttons stay `0%` unlit.
  4. **Left Encoders = Zero-Modifier Globals:** `ALGORITHM` = Circuit Coupling/Sag, `SELECT` = Global Stereo Spread, `PRESETS` = Continuous Snapshot Morph.
- **Consequences:** 100% unobstructed display, zero modal traps, and maximum alignment with factory silkscreen labels.

---

## ADR-006: Always-On Triggered Scope with `240x16` Scanline Alpha-Blended HUD
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Full-screen `240x240` RGB565 double-buffering would consume `112.5-225 KB` of the SoC's `578 KB` SRAM, yet switching between a scope page and parameter pages breaks visual continuity.
- **Decision:** Render the `240x240` screen continuously on Core 0 via `240x16` ping-pong scanline strips (`15.36 KB` total SRAM) over SPI DMA (~45-55 FPS). Keep the Osc-1 rising-edge-triggered stereo waveform visible 100% of the time, compositing translucent HUD overlays on touch (`~120 ms` fade-in) that dissolve back to `0%` opacity after ~1.5 s idle (`~800 ms` fade-out).
- **Consequences:** Boutique CRT aesthetic and real-time phase/interval tuning with tiny RAM footprint.

---

## ADR-007: Varibright Monochrome Keybed Zoning & LFO-Coupled Key Pulse
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Hardware verification confirmed the FM-1's 27 silicone keys and 14 buttons have individually addressable **monochrome white** LEDs with software PWM dimming (not RGB), with 4 silkscreen dots dividing the 16 white keys into four groups of 4 (`[1-4]`, `[5-8]`, `[9-12]`, `[13-16]`).
- **Decision:** Map the 4 dotted white-key groups to `Osc 1-4 Latching Mutes` (`1-4`), `Osc 1-4 Pressure Expression Pads` (`5-8`), and `8 Drone Snapshots` (`9-16`), using Monome-style varibright tiers (`0%` unused/dark, `10%` muted ember, `25%` modifier landmark, `60-100%` active) and LFO-synced pulsing below `20 Hz`.
- **Consequences:** Immediate dark-room legibility that honors the hardware's silkscreen dots and monochrome LEDs.

---

## ADR-008: Stereo TRS Breakout & `MONO`/`POLY` Output Routing
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** The FM-1's 3.5mm jack is an AC-coupled stereo headphone/line output; inserting a mono TS Eurorack patch cable directly shorts the Right ring to ground, while pedal rigs alternate between mono fuzz chains and stereo delay/reverb setups.
- **Decision:** Design stereo panning (`L`/`R`) assuming a **3.5mm TRS stereo breakout** cable, and bind the silkscreened `MONO` / `POLY` black keys to instant clickless **Dual-Mono** vs. **Full Stereo** output routing.
- **Consequences:** Protects the hardware output stage and adapts cleanly to both mono pedalboards and stereo/Eurorack setups.

---

## ADR-009: `SARADC` Carbon-Pill Polyphonic Keybed Pressure Expression
- **Date:** 2026-10-29 · **Status:** Accepted (updated by ADR-013: pitch bend removed; strictly Filter Accent + Vactrol LPG)
- **Context:** Community hardware research demonstrated that sampling `AC791N` `SARADC` channels across the silicone key matrix rows reads the analog resistance drop of the keys' conductive carbon pills under finger force without hardware modification.
- **Decision:** Integrate continuous per-key pressure sensing on Core 0 to drive **timbral & dynamic expression without ever altering oscillator frequency**: (1) sweeping MS-20 filter formant/cutoff, resonance, and diode drive on active voices or swelling muted voices through a Buchla-style Vactrol Low-Pass Gate (`LPG`) on White Keys `5-8`, and (2) momentary **rubber-band snapshot morphing** when leaning into Snapshot White Keys `9-16`.
- **Consequences:** Turns inexpensive rubber dome keys into a deeply expressive tactile filter/LPG surface while keeping oscillator pitch 100% locked to the knobs.

---

## ADR-010: `uint32` Phase Accumulators, Fused Single-Division Korg35 ZDF Solver & Linear-Phase Polyphase Half-Band FIR Decimation
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Implementing Milestone 1 exposed three subtle real-world numerical & acoustic trade-offs at `176.472 kHz` (`4x` oversampling) on a single-precision (`float32`) embedded FPU:
  1. At `0.02 Hz`, `dt = 1.13e-7`, whereas `ULP(1.0f)` in `float32` is `5.96e-8`—causing `float32` `[0, 1)` phase accumulators to suffer massive quantization error near `1.0f`.
  2. Standard 2-step Newton-Raphson resolution of two Korg35 diode-clipped Sallen-Key stages (`HPF -> LPF`) across 4 voices at `4x` OS requires up to 256 serial `fdiv` instructions per stereo frame unless algebraically reduced.
  3. Minimum-phase / allpass IIR half-band decimators introduce phase dispersion near `20 kHz` that tilts flat square-wave roofs and inflates peak crest factor (`~1.35-1.50`), eating headroom into external pedals.
- **Decision:**
  1. Drive all 4 Schmitt oscillators with **32-bit unsigned integer (`uint32_t`) phase accumulators** (`41 uHz` resolution, immune to `float32` ULP stall) paired with **4-point Integrated Cubic B-Spline PolyBLEP** (`~-108.5 to -112.6 dBc` alias floor `< 20 kHz`).
  2. Place the Padé diode clipper `phi(y) = y*(27 + z^2)/(27 + 9*z^2)` and its exact derivative `((27 - 3*z^2)/(27 + 9*z^2))^2` over a single common denominator `D^2 - alpha*k*Nd^2`, collapsing each Korg35 ZDF Newton-Raphson iteration into **one `fdiv`** (with Van der Pol period elongation compensation above `k = 2.0`).
  3. Use a **two-stage cascaded Linear-Phase Symmetric Polyphase Half-Band FIR Decimator** (`P=8` 31-tap Stage 1 + `P=32` 127-tap Stage 2, Kaiser-windowed) with double-mirrored unit-stride odd rings and 4-way unrolled accumulators—preserving `0°` phase dispersion (`1.159` isolated square crest factor) and `0.0005 dB` passband flatness.
- **Consequences:** Exceeds the `-70 dB` alias gate by `~38 dB` (`-108.5 dBc` worst-case), tracks self-oscillation within `< 0.6%` up to `8 kHz`, keeps total `DroneEngine` SRAM at `27.36 KB` (including the 24 KB scope buffer), and cuts filter division pressure by `4x-8x` on `pi32v2`.

---

## ADR-011: Freestanding Zero-`libm` WebAssembly Dual-Instance Simulator & Native `CoreAudio` Host
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Running the exact C++17 firmware engine in the browser without installing heavy external SDKs (`emsdk`) or requiring `SharedArrayBuffer` HTTP isolation headers (`COOP`/`COEP`) requires avoiding host `libc`/`libm` imports and decoupling hard real-time DSP from main-thread DOM/Canvas rendering.
- **Decision:**
  1. Replace remaining `<cmath>` calls across `src/` with deterministic IEEE-754 polynomial helpers (`fastExp`, `fastLog2`, `__builtin_fabsf`) compiled under `-ffp-contract=off` and tiny freestanding standard C++17 headers (`platforms/sim/wasm_libc/`), allowing stock Clang `--target=wasm32 -nostdinc -nostdinc++ -nostdlib` to emit a self-contained `~97 KB` `ruido.wasm` binary in `< 1 s`.
  2. Mirror the `AC791N` dual-core split in the browser by instantiating `ruido.wasm` inside an `AudioWorkletProcessor` (`Core 1` `DroneEngine`) and on the UI thread (`Core 0` `RuidoController` + `ScopeHudRenderer` rendering `15 x [240x16]` RGB565 scanline strips), exchanging `160`-byte `EngineControlState` packets and `ScopeFrame` batches via `MessagePort` transferables, alongside a native macOS `AudioToolbox` (`AudioQueue`) `44,118 Hz` live runner (`platforms/sim/macos_sim.cpp`).
- **Consequences:** Zero external SDK dependencies for Milestone 2, zero main-thread audio dropouts, `1:1` USB-MIDI `F3-G5` (`53-79`) + Aftertouch hardware controller mapping, and `43.6 KB` total combined Core 0 + Core 1 portable engine SRAM (`28,416 B` DSP + `14,808 B` UI + `448 B` Controller).

---

## ADR-012: Pure Monochrome Analog Tektronix CRT Scope, 4-Bit Sans-Serif ROM Typography & Unified Divider-Free HUD
- **Date:** 2026-10-29 · **Status:** Accepted (refines visual design of ADR-006)
- **Context:** Auditioning the initial Milestone 2 simulator revealed four UX shortcomings: (1) colored cyan/amber chrome and 1-bit `5x7` bitmap pixel fonts looked rough and toy-like compared to boutique hardware; (2) integer-sample vertical min/max column spans produced jagged stair-stepped waveforms; (3) boxed cards, divider lines, and colloquial labels (`"SPEED"`, `"DRONE"`) cluttered the `240x240` screen; and (4) unassigned keys gave zero visual confirmation when pressed. Since `Core 0` is 100% dedicated to UI/scanning on the dual-core `AC791N` (`~960` cycles/pixel available at 50 FPS), ample CPU headroom exists on `Core 0` without touching `Core 1` DSP.
- **Decision:**
  1. **100% Monochromatic B&W Aesthetic:** Pitch-black (`#000000`) background and strictly neutral grayscale across both the `240x240` TFT (`R5 == B5`, `G6 == (R5 << 1) | (R5 >> 4)` with `2x2` ordered Bayer dithering) and the web simulator chassis.
  2. **Analog Tektronix Dual-Beam CRT (`Option A`):** Sub-sample zero-crossing trigger lock + 4-point Catmull-Rom interpolation, 2D capsule distance Gaussian beam + `1 / sqrt(1 + slope^2)` dwell-time velocity shading + optical phosphor halo, and 6-frame `Q8.6` decaying phosphor persistence trails (`22,560 B` total `ScopeHudRenderer` SRAM, zero framebuffer).
  3. **4-Bit Anti-Aliased Sans-Serif ROM Font (`src/ui/FontSansAA.h`):** Offline-baked `HelveticaNeue-Medium` (`10pt` & `13pt` with tabular numerals `'0'..'9'`, `4.75 KB` `constexpr` XIP Flash, `0 B` SRAM).
  4. **Unified Divider-Free HUD & Full-Surface Technical Feedback:** Constant `y = 120` wave scale (zero vertical wave compression when HUD opens), soft feathered vertical vignettes with zero divider lines, `200 ms` `smoothstep01` S-curve fade-in, technical terminology throughout (`FREQUENCY`, `AUDIO RANGE` / `SUB-AUDIO LFO`, `RAIL COUPLING`, `STEREO WIDTH`), 11th black key bound to momentary **Scope Freeze**, and HUD feedback on every key, switch, encoder, and pot.
- **Consequences:** Crisp boutique monochrome instrument aesthetic, rock-solid sub-pixel analog CRT trace, `51.4 KB` total combined Core 0 + Core 1 engine SRAM (`28,416 B` DSP + `22,560 B` UI + `456 B` Controller), and immediate visual confirmation for all 27 keys and 14 buttons.

---

## ADR-013: `SEL` Brightness-Coupled Pressure Routing & Dual-State MS-20 Filter Accent / Vactrol LPG (`Keys 5–8`)
- **Date:** 2026-10-29 · **Status:** Accepted (supersedes `PIT` pitch-pressure toggle in ADR-009)
- **Context:** Auditioning `White Keys 5–8` exposed three UX and DSP issues with the original `PIT` black-key toggle: (1) placing a latching mode toggle on `PIT` violated the bottom black-key row's universal momentary-hold grammar (`OP1–OP6`, `GLO`, `Scope Freeze`); (2) having `PIT` dark (`0%`) in Filter mode made a live control look dead/unassigned and failed to communicate any link to `Keys 5–8`; and (3) in Filter mode, default `LPF = 20 kHz` clamped upward cutoff multiplication so pressure was barely audible on boot.
- **Decision:**
  1. Move `Keys 5–8` mode selection off the momentary black-key row (`PIT` becomes `0%` dark / `UNASSIGNED`) onto **`SEL`** in the right-hand `2x6` single-tap utility grid (`PRESSURE TARGET`: `MS-20 FILTER` <-> `PITCH BEND`), and **visually couple `SEL` to `Keys 5–8` via identical non-zero PWM brightness** (`24%` shared rest glow in Filter/LPG mode, `72%` shared rest glow in Pitch Bend mode, stepping in unison on tap, plus `SEL` flaring to `100%` alongside any pressed pad).
  2. Drive `White Keys 5–8` through an asymmetric `VactrolSmoother` (`4.5 ms` attack / `26 ms` photoconductive release): in default Filter/LPG mode (`0.000 Hz` pitch shift), active voices open `LPF` (`+4 oct`) when darkened or sweep a resonant `HPF -> LPF` Korg35 formant peak + diode drive when `LPF` is parked wide open, while muted voices swell via a Buchla-style Vactrol Low-Pass Gate (`95 Hz` sub murmur -> full harmonics); in Pitch Bend mode (`SEL` at `72%`), pressure bends pitch upward up to `+6` semitones with live `Hz` HUD tracking.
- **Consequences:** Every row on the faceplate obeys one consistent interaction rule (black-key modifiers strictly momentary, right grid strictly single-tap), monochrome white LEDs create an unmistakable Gestalt link between `SEL` and `Keys 5–8` without ever looking "off", and default Filter/LPG mode guarantees `0.000 Hz` pitch shift with dramatic out-of-the-box MS-20 expression.

---

## ADR-014: Contiguous `OP1–OP5` Filter Modifier Group, `GLO` Amplitude Swap & Strictly Momentary Hold Release
- **Date:** 2026-10-29 · **Status:** Accepted (updates modifier assignments in ADR-006)
- **Context:** Auditioning the left-hand black modifier keys revealed two ergonomic friction points: (1) `Global Filter` was separated onto `GLO` across `[OP6][PIT]` instead of sitting directly beside `OP1–OP4`; and (2) `OP1–OP4` felt like latching toggles because `syncModifierHud()` left `HudMode::VoiceFilter` frozen at `100%` opacity for `1.5 s` after finger release (and the web simulator included accidental `Shift+Click`, right-click `contextmenu`, and keyboard `click` latching hooks).
- **Decision:**
  1. Group **all 5 filter holds** contiguously on **`OP1–OP5`** (`OP1–OP4` = `Osc 1–4 Filter`, `OP5` = `Global Filter`, all sharing a `15%` rest LED glow) and move **`Osc 1–4 Amplitude`** to **`GLO`** (paired with `OP6` = `Panning` at `25%` landmark LED glow).
  2. Make all modifier black keys (`OP1–OP6`, `GLO`, `Scope Freeze`) strictly momentary: on release, `RuidoController::syncModifierHud` immediately exits the modifier HUD mode and begins smooth exponential dissolve (`holdRemainingSec_ = 0.0f` rather than a `1.5 s` freeze), and all simulator latching hooks are removed in favor of clean `pointerup` / `pointercancel` / `lostpointercapture` / `blur` release handling.
- **Consequences:** Cohesive 5-key filter cluster (`1–5`) + 2-key mix cluster (`6`, `G`), matched LED brightness tiers per functional group, and crisp, unmistakable momentary-hold release feedback.

---

## ADR-015: Extreme Self-Oscillating `Korg35` Taper & Multi-Stage Buchla/Serge `C^1` Wavefolder `DRIVE` Stage
- **Date:** 2026-10-29 · **Status:** Accepted (upgrades filter saturation & resonance staging in ADR-002 & ADR-010)
- **Context:** Auditioning `OP1–OP5` filter controls revealed two sonic limitations in the initial `Ms20DualFilter` staging: (1) `DRIVE` (`KNOB4`) sounded nearly inaudible because applying a symmetric pre-clipper (`padeClip`) to an already flat-topped `±1.0` square wave merely rescaled a constant `±1.0` into `±const` and then attenuated `postDriveScale` down to `0.953` *before* the filter—starving the `Korg35` feedback diodes; and (2) the filter felt tame because wide Smart-Park deadbands (`20–36 Hz` and `16.5–20 kHz`), a late self-oscillation threshold (`58%`, `kMax = 3.20`), unity passband gain drowning out `±1` diode limit cycles, and slow `0.010 oct/tick` cutoff stepping muted resonant drama.
- **Decision:** Push both `RES` and `DRIVE` all the way into extreme MS-20 self-oscillation and West-Coast wavefolding territory within the `176.472 kHz` `4x` oversampled loop:
  1. **Aggressive `Korg35` Self-Oscillation & Tight Smart-Park:** Raise `kMax` to `3.72` (`alpha * k <= 0.93 < 1.0`) with a concave taper (`self-oscillation` by `~35%`, screaming howl above `65%`), tighten Smart-Park to `20..24 Hz` (`HPF`) and `18.8..20 kHz` (`LPF`), add physical `Korg35` input/output resonance gain staging (`resInScale` / `resOutScale`) so resonant peaks tower above the dry square wave, recalibrate `compensateSelfOscPitch` (`0.0%` error at `200 Hz` & `1 kHz`, `< 0.5%` at `8 kHz`), and double cutoff encoder speed (`0.022 oct/tick`).
  2. **4-Stage `DRIVE` Overdrive + Division-Free `C^1` Sine Wavefolder:** When `DRIVE > 0`, (a) shape flat square roofs through a two-stage RC sharkfin contour (`95 Hz` roof integrator -> `2.4 kHz..1.1 kHz` edge slew) so every half-cycle sweeps smoothly across folds even when `HP = 20 Hz, LP = 20 kHz` are wide open while bandlimiting step edges before folding; (b) slam `1.0x -> 6.0x` unattenuated gain into `Korg35 HPF -> ±10V asymmetric buffer -> Korg35 LPF`; (c) crossfade into a division-free `C^1` polynomial Buchla/Serge multi-stage wavefolder (`wavefoldBuchlaC1`, folding across `±1, ±3, ±5...` with bounded even-harmonic bias); and (d) block asymmetric DC drift via a `9.5 Hz` output coupling capacitor (`+16 B` per voice, `Ms20DualFilter = 40 B`).
- **Consequences:** Immediate `6x` West-Coast wavefolding on `DRIVE` even with parked filters (`22 -> 132` zero-crossings on a `110 Hz` square at `0.717` RMS), ferocious pitch-accurate `Korg35` self-oscillation and chaotic diode ducking on `RES`, `< -108 dBc` clean alias floor preserved at `DRIVE = 0`, `< 0.0001` steady-state DC offset, and `18.8x` real-time single-core throughput (`1,207 ns/frame`).

---

## ADR-016: Mute Amplitude Memory + `GLO` Mixer `0%` Display & Clickless Fade-In Unmute
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Toggling `White Keys 1–4` remembers each oscillator's pre-mute `amplitude` (which also sets the ceiling for `White Keys 5–8` Vactrol LPG swells while muted), but previously holding `GLO` (`Amplitude` mixer) still showed that stored percentage (e.g. `65%`) on muted silent channels and kept them muted if turned.
- **Decision:**
  1. Keep `VoiceParams::amplitude` intact when muting/unmuting via `White Keys 1–4`, while `ScopeHudRenderer` (`KnobBankMode::Amplitude`) displays **`0%`** (`norm = 0.0f`) for any muted oscillator when viewing the `GLO` mixer.
  2. In `RuidoController::onEncoderDelta` (`HeldModifier::Amplitude`), turning a muted oscillator's knob clockwise (`ticks > 0`) clears `muted = false` and fades `amplitude` up cleanly from `0.0f` (while accidental counter-clockwise ticks at `0%` preserve both mute and remembered amplitude).
  3. In `DroneEngine::updateTargetsFromState`, when a silent closed-gate voice (`vactrolGate <= 1e-4` and `pressure <= 1e-4`) unmutes, snap `amplitude` to its new target at zero gain before opening `vactrolGate` so fading in from `0%` never produces a transient volume hump.
- **Consequences:** Gate toggles preserve exact mix levels, the `GLO` mixer truthfully reflects `0%` silence on muted channels, and turning any muted channel's knob upward smoothly fades it into the drone from `0%` and lights its gate key active.

---

## ADR-017: Momentary Linear 3-Speed X4 Quadrature Encoders & `C^1` Log-Octave Potentiometer Smoothing
- **Date:** 2026-10-29 · **Status:** Accepted (replaces octave-shift & velocity-ballistics encoder model from ADR-006)
- **Context:** Auditioning rotary sweeps exposed three issues: (1) discrete `24-detent` jumps smoothed with a `5.0 ms` linear-`Hz` one-pole filter settled within `15 ms` and sat on flat chromatic plateaus between detents (while rushing bass octaves `20x` faster than treble), producing audible stair-stepping; (2) velocity-based acceleration made `STANDARD` speed feel rubbery and inconsistent; and (3) `OCT-` / `OCT+` needed to follow Ruido's strictly momentary modifier rule.
- **Decision:**
  1. **Strictly Momentary `OCT-` / `OCT+` with Zero Velocity Acceleration (`96` X4 quadrature transitions = `360°`):** Remove velocity acceleration completely so angle-to-parameter response is `100%` linear and deterministic in every gear (`Released = Standard`: `~2.5–2.8` turns full sweep; `Hold OCT- = Fine`: `0.66 cents/step` audio freq, `0.09%/step` linear; `Hold OCT+ = Fast`: **`~1.30–1.33` turns** full end-to-end sweep across every parameter, chordable with `OP1–OP6` / `GLO`).
  2. **2-Pole Critically-Damped (`C^1`) Log-Octave & Linear Potentiometer Smoothers (`ParameterSmoother.h`):** Smooth `freqHz`, `hpCutoffHz`, and `lpCutoffHz` in `log2(Hz)` octave space (`LogFreqSmoother`) and continuous linear controls via `CriticallyDampedSmoother` (`14 ms` per pole, `~28 ms` mean S-curve glide, `0%` overshoot, exact settled `targetHz` caching at `0` stationary FLOPs), eliminating `t = 0` slope kinks, bass-octave rushing, and inter-step plateaus.
- **Consequences:** Liquid, stepless analog-potentiometer sweeps on every knob, predictable 1:1 linear rotation speed in `Standard`, 1.3-turn full-range throws while holding `OCT+`, surgical sub-Hz beating while holding `OCT-`, and zero latched state.

---

## ADR-018: Open Edge-to-Edge Coupled 4-Voice Caustic Ribbon Scope & Quarter-Res Separable Bloom
- **Date:** 2026-10-29 · **Status:** Accepted (upgrades waveform renderer in ADR-012)
- **Context:** While the 2-beam Tektronix CRT trace in `ADR-012` rendered clean stereo contours, Ruido's sonic heart is `4` coupled Schmitt oscillators driving `4` independent MS-20 (`Korg35`) filters + wavefolders. We wanted a silky, multi-strand caustic ribbon waveform where overlapping circuit trajectories flare organically into incandescent white—while staying strictly faithful to real acoustic/circuit state (`AGENTS.md` Priority #3), keeping `Core 1` DSP untouched, avoiding any `115.2 KB` `240x240` framebuffer, and fitting well within `Core 0`'s actual `~83–111` cycles/pixel budget at `50 FPS` (`240–320 MHz / 50 / 57,600`; correcting the `960` cycles/pixel typo in `ADR-012`). Interactive visual prototyping (`waveform_comparison_lab.html`) also revealed two critical UX insights: (1) plotting `Osc 1` in 100% isolation while triggering on `Osc 1` caused `Osc 1`'s strands to freeze like a static stencil; and (2) pinching wave endpoints to `y = 120` compressed outer gain-staging readability compared to Ruido's continuous open edge-to-edge drone grammar.
- **Decision:**
  1. **Sample-Aligned 16-Bit 4-Voice `ScopeFrame` Taps & Rising-Edge Mask (`20 B`):** Extend `ScopeFrame` with `int16_t voiceQ13[4]` (`[-3.5, +3.5]` @ `1/8192` = `0.0095 px` sub-pixel precision, eliminating `0.93 px` `int8` roof chatter) and `uint8_t voiceRisingMask` delayed by `36` base frames inside `DroneEngine` so all 4 voice taps and rising edges align sample-accurately with the linear-phase FIR decimated stereo output.
  2. **4096-Sample Multi-Cycle Super-Period Trigger & Continuous Phosphor Persistence (`Option A`):** Store `4096` compact `12 B` `Q3.12` scope samples (`~93 ms` @ `44.1 kHz`, guaranteeing `100%` trigger lock down to `20 Hz` and `4+` full cycles at `55 Hz`), refine each oscillator period marker to its exact continuous post-MS-20 upward zero-crossing (compensating filter group delay), match up to `8` candidate rising edges against a `48`-tap per-voice reference signature so `3:2` fifths (`55 + 82.4 Hz`) and `4:3` fourths lock cleanly onto their multi-cycle super-period with zero `30 Hz` alternating-cycle flip (`0.107 / 255` consecutive-frame delta), and apply continuous `38–135 ms` exponential phosphor slewing across `14` open edge-to-edge (`W(x) = 1.0`) strands (`3` per active voice + `2` stereo ribs).
  3. **Tile-Culled Linear Photon Accumulator, `25%` `60x60` Separable Bloom & `4x4` Bayer RGB565:** Cull empty column regions in `2` integer comparisons per `8`-px horizontal tile, accumulate `64`-entry LUT Gaussian capsules in linear photon space (`<= 8`-row sub-strips = `5.76 KB` stack), splat analytical photon energy into a `60x60` (`7.2 KB`) quarter-res grid blurred in-place via two separable `[1, 3, 4, 3, 1] / 12` passes (`25%` bloom calibration) and exact eighth-step integer bilinear upsampling, tonemap via a `1024`-entry `constexpr` soft-knee Reinhard + `0.82` gamma LUT (`1.12 KB` XIP Flash), and dither to neutral `RGB565` via a `4x4` Bayer matrix.
- **Consequences:** Calm, silky 3D caustic ribbon acoustics reflecting every individual voice mute, filter sweep, resonance peak, wavefold, natural beating, and LPG swell without scope-trigger jitter; zero `240x240` framebuffer (`ScopeHudRenderer = 79.28 KB`, `DroneEngine = 44.75 KB`, combined `< 125 KB` of `578 KB` SRAM); `100%` bit-identical `8`-, `16`-, and `24`-row strip equivalence; and `< 21%` estimated `Core 0` load on `AC791N`.

---

## ADR-019: Independent MS-20 High-Pass & Low-Pass Resonance (`HPF | HP RES | LPF | LP RES`) & Unassigned `LFO` Key Reservation
- **Date:** 2026-10-29 · **Status:** Accepted (supersedes shared resonance + `DRIVE` wavefolder in ADR-006 / ADR-015 and per-voice sub-audio range toggle on `LFO` in ADR-004)
- **Context:** In hardware playtesting, two features fell short musically: (1) tapping `LFO` to flip an audio drone oscillator down into a sub-audio `[0.02, 20 Hz]` clicking square wave was unmusical compared to reserving `LFO` for a proper modulation LFO later; and (2) dedicating `KNOB4` to filter `DRIVE` while forcing `HPF` and `LPF` to share a single `RES` control prevented classic Korg MS-20 dual-peak patching (such as cranking `HP RES` at the fundamental for massive sub-bass boost while keeping `LPF` smooth, or pairing asymmetrical `HP` and `LP` peaks / independent self-oscillation).
- **Decision:**
  1. **Unassign `LFO` Switch for Future Modulation Engine:** Remove `VoiceParams::subAudioLfo` and the `ButtonId::Lfo` oscillator-range toggle. `LFO` now rests at `0%` dark LED and displays `LFO SWITCH · UNASSIGNED` on press alongside the other unassigned right-grid switches (`FX`, `ENV`, `EDIT`, `ARP`, `SEQ`, `REC`), while keeping `SchmittOscillatorBank`'s 32-bit `0.02 Hz` phase-accumulator precision intact under the hood.
  2. **Classic MS-20 4-Knob Filter Pairing (`HPF | HP RES | LPF | LP RES`):** Replace `VoiceParams::{resonance, drive}` with independent `hpResonance` and `lpResonance` (`[0.0, 1.0]`), mapped on `OP1–OP4` (Voice Filter) and `OP5` (Global Filter) to `KNOB1 = HPF`, `KNOB2 = HP RES`, `KNOB3 = LPF`, `KNOB4 = LP RES`—matching the physical Korg MS-20 layout and the left/right knees of the HUD passband curve.
  3. **Per-Stage `Korg35` Dividers, Smart-Park & LPF Diode AC Coupling:** Remove the `DRIVE` contour/wavefolder stages from `Ms20DualFilter` (`40 B -> 28 B`), giving `HPF` and `LPF` independent Smart-Park resonance tapers, per-stage gain dividers (`hpInScale`, `interStageScale`, `lpOutScale`), and a 1-pole `8 Hz` AC-coupling DC tracker on `LPF` resonant feedback so either or both filters can resonate and self-oscillate cleanly with zero DC drift and without `HPF` swamping `LPF`'s diode loop, while rendering independent `HP` and `LP` peaks on `ScopeHudRenderer::buildFilterCurve`.
- **Consequences:** Authentic MS-20 sub-bass peak boosting, asymmetric vowel formants, and dual-sine self-oscillation per voice; lower Core 1 filter CPU & state footprint; cleaner HUD curve correspondence; and a clean slate on the `LFO` button for a dedicated modulation LFO later.

---

## ADR-020: Concept A3 `1/3`-Screen Dual-Resonance Filter Graph, Unified Optical-Bead Gauges & `70%` Background Dim Transition
- **Date:** 2026-10-29 · **Status:** Accepted (upgrades HUD filter curve & level-bar rendering in ADR-012 / ADR-018)
- **Context:** After upgrading the live scope to the 14-strand caustic ribbon (`ADR-018`) and adopting independent `HP RES` + `LP RES` (`ADR-019`), the legacy `39 px`-tall single-stroke filter curve (`y = 145..184`) and rectangular `fillRect` level bars felt cramped and visually disconnected from the optical-caustic scope. In addition, superimposing a large dual-resonant filter graph directly over an undimmed `100%`-bright waveform caused foreground/background luminance competition.
- **Decision:**
  1. **Concept A3 `1/3`-Screen (`80 px`: `y = 102..182`, `x = 10..230`) Dual-Resonance MS-20 Graph:** Expand the filter graph to a full third of the `240x240` vertical screen (`0 dB` at `y = 132` with `30 px` headroom up to `y = 102` for `+17 dB` dual Korg35 resonant horns and `50 px` passband skirt down to `y = 182`), rendered with **Option A3** hybrid under-curve shading (`~10%` solid floor silhouette anchored by a subtle `y = 182` horizon filament + `~30%` exponential caustic crest glow), linear-photon `kGaussLut` curve + Korg35 companion contours screen-blended via `kTonemapLut`, and dual `HP`/`LP` cutoff focal beads whose halo radii and intensities scale with `HP RES` and `LP RES` (faded cleanly at `20 Hz` / `20 kHz` via Smart-Park).
  2. **Zero Artificial Voice Offset (`0%`) on `OP5` (`GLOBAL FILTER`):** Never synthesize visual voice offsets—when all 4 voices share matching filter parameters, `OP5` renders the exact unified single-curve `Option A3` graph + focal beads; only when the musician explicitly diverges individual voices via `OP1–OP4` does `OP5` overlay all 4 real per-voice curves.
  3. **Smooth `70%` Background Waveform Dimming Transition:** Slew `filterBackdropDim_` (`~110 ms` attack / `~240 ms` release through `smoothstep01`) strictly when a filter graph mode (`OP1–OP5` or `White Keys 5–8` in Filter/LPG mode) is active, attenuating background scope + bloom luminance down to `30%` (`70%` dimming) behind the `80 px` filter graph while keeping all non-filter HUD overlays (`Frequency`, `Amplitude`, `Panning`, `ScopeFreeze`, global encoders) at `100%` waveform brightness.
  4. **Unified Optical-Filament & Sub-Pixel Focal-Bead Gauges Across the System:** Replace every `fillRect` bar (` KNOB 1–4` bottom strip, bipolar `L..C..R` pan center-slit gauges, top header gauges, and idle top-right `L/R` peak meters) with sub-pixel Gaussian horizontal filaments and incandescent optical focal beads.
- **Consequences:** Cohesive optical-caustic visual grammar across every graph and meter on the instrument, effortless dual-peak MS-20 filter readability over a smoothly dimmed live waveform, `100%` bit-identical `8`-, `16`-, and `24`-row scanline strip output, zero extra stack RAM (reusing the `<= 8`-row `linBuf`), and `ScopeHudRenderer = 82,584 B` (`80.65 KB` SRAM, zero `240x240` framebuffer).

---

## ADR-021: Removal of `Stereo Width`, Release of `SELECT` Encoder & Judicious Right-Grid Page UX
- **Date:** 2026-10-29 · **Status:** Accepted (supersedes `SELECT = Stereo Width` in ADR-006, removes erroneous `SelectPush` switch assumption, and refines strict zero-menu constraint in ADR-005)
- **Context:** Per-oscillator equal-power panning (`OP6`) and `Stereo / Dual Mono` output collapse already cover Ruido's spatial staging needs; dedicating the `SELECT` endless encoder (`EncoderId::Select`) to a global `Stereo Width` scaler wasted the FM-1's dedicated selection encoder (note: physical FM-1 rotary encoders do not have push-switches—`ButtonId::SelectPush` was an erroneous early simulator assumption). Simultaneously, none of the `2x6` switches below `KNOB 1–4` are momentary holds—they naturally support latching toggles and judicious, focused pages (such as a rich `LFO` modulation view on `LFO` and a future Configuration Menu on the right-grid `GLO` button where `Stereo / Dual Mono` routing will live).
- **Decision:**
  1. **Remove `Stereo Width` (`GlobalParams::stereoSpread` & `spreadSmoother_`) & Phantom `SelectPush`:** Strip `stereoSpread` and `ButtonId::SelectPush` (`ButtonId::Count` = `14` real switches) across `src/`, `platforms/sim/`, `tests/`, and `docs/`. Per-voice `pan` (`-1.0..+1.0`) maps directly to equal-power `L/R` gains when Stereo (`monoMix == 0`) or collapses clicklessly to Dual Mono (`monoMix == 1`).
  2. **Release `SELECT` Encoder for Future Page / Modulation Navigation:** Turning `EncoderId::Select` (`HudMode::UnassignedEncoder` -> `SELECT ENCODER · UNASSIGNED`) confirms hardware contact without altering audio state, freeing `SELECT` for upcoming `LFO` and `GLO` Config pages.
  3. **Adopt Judicious Right-Grid Page UX:** Keep momentary holds strictly on the left/bottom (`OP1–OP6`, `GLO` black key, `OCT-` / `OCT+`, `Scope Freeze`), while allowing non-momentary `2x6` right-grid switches (`LFO`, right-grid `GLO`, etc.) to open focused, high-utility pages navigated with `SELECT` + `KNOB 1–4` and exited cleanly via `HOME`.
- **Consequences:** Hardware-accurate 7-encoder / 14-button model, cleaner DSP mix stage (`DroneEngine` saves a 2-pole smoother), freed `SELECT` rotary encoder, and far greater ergonomic flexibility for a capable `LFO` engine and future `GLO` configuration menu.

---

## ADR-022: Dedicated Per-Voice LFO Specification (`SELECT` = Voice, `TARGET | SHAPE | RATE | DEPTH`)
- **Date:** 2026-10-30 · **Status:** Implemented (supersedes unassigned `LFO` placeholder in ADR-019)
- **Context:** Ruido consists of 4 independent mono synth voices (`4` oscillators, `4` MS-20 dual filters, `4` VCAs, `4` panners). A single shared global LFO violates that 4-voice independence, while multi-slot mod matrices with cross-mod menus violate Ruido's immediate analog hardware spirit. With `ButtonId::Lfo` operating as a latching page switch and `EncoderId::Select` freed (`ADR-021`), each voice can own a dedicated, hands-on LFO with zero sub-menus.
- **Decision (Locked Specification):**
  1. **1 Dedicated LFO Per Voice (`4` Independent Voice LFOs):** Each voice (`Osc 1..4`) owns one dedicated LFO (`uint32_t` phase accumulator + smooth drift generator running at audio/frame rate on Core 1).
  2. **Page Entry, Exit & Voice Selection:** Tapping **`LFO`** (`ButtonId::Lfo`) enters the **LFO Page**; tapping `LFO` again or pressing **`HOME`** returns to the pure scope while active LFOs continue running. While on the LFO Page, turning **`SELECT`** (`EncoderId::Select`) steps across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4` · `ALL 4`**.
  3. **Ergonomic Knob Order (`TARGET | SHAPE | RATE | DEPTH` — Setup Left, Perform Right):**
     - **`KNOB 1` — `TARGET` (7 Stepped Per-Voice Circuit Parameters):** **`LPF`** *(default)* · **`LP RES`** · **`HPF`** · **`HP RES`** · **`AMP`** · **`PITCH`** · **`PAN`** (strictly 1-to-1 with real `VoiceParams` circuit parameters).
     - **`KNOB 2` — `SHAPE` (`0–100%` Continuous Seamless Morph):** Stepless continuous morph across **`Sine` (`0%`) -> `Triangle` (`25%`) -> `Sawtooth` (`50%`) -> `Slewed Square` (`75%`, vactrol-rounded corners) -> `Smooth Drift` (`100%`, non-repeating organic wander)**.
     - **`KNOB 3` — `RATE` (`0.02 Hz – 30 Hz` Continuous Log-Octave Sweep):** `50 s` glacial drift up to `30 Hz` flutter/FM (`LogFreqSmoother`).
     - **`KNOB 4` — `DEPTH` (`0–100%` Unipolar Continuous Amount):** Hard stop at `0%` CCW so any LFO can be zeroed blindly by feel without overshooting into inverted modulation (`CriticallyDampedSmoother`).
  4. **Acoustic Visuals (`ScopeHudRenderer`):** On the LFO Page, the `80 px` (`y = 102..182`) Concept A3 graph region renders the focused voice's live morphed LFO wave + moving phase focal bead over the `70%`-dimmed background waveform (or overlays all 4 voices' LFO curves when `SELECT = ALL 4`, matching `OP5`), and active filter LFO modulation animates the MS-20 filter curves/beads in real time whenever `OP1–OP5` is held.
- **Consequences:** Preserves 100% per-voice musical independence (e.g., rock-steady sub bass on `Osc 1` while `Osc 2–4` run independent filter swells, resonance self-oscillation tides, tremolo, or pitch/pan drift), keeps discrete choices on the left (`SELECT`, `KNOB 1`) and continuous performance sweeps (`SHAPE`, `RATE`, `DEPTH`) on the right, and requires zero nested menus.

---

## ADR-023: Zero-Hard-Cut Visual Transitions & Persistent Right-Grid (`2x6`) Switch HUDs
- **Date:** 2026-10-30 · **Status:** Accepted (refines scope, HUD persistence, and LED transition behavior in ADR-005, ADR-018, ADR-020, and ADR-021)
- **Context:** Auditing visual transitions and button persistence across Ruido exposed two UX gaps: (1) right-side `2x6` switches (`SEL`, `PLAY/STOP`, `SAVE`, `LFO`, `FX`, `GLO` right, etc.) auto-faded out after `1.5 s` just like momentary functions, even though they are non-momentary switches/pages paired with a dedicated `HOME` button; and (2) remaining 1-frame hard cuts occurred when stopping audio (`setSilentHorizon()` snap), crossing the `78/92 Hz` bass-zoom stride boundary, releasing momentary black keys (`FREQUENCY` flash on frame 1 of fade-out), tapping `HOME`, or switching filter views/snapshots.
- **Decision:**
  1. **Persistent Right-Grid `2x6` Switch HUDs (`latchedHudMode_` / `latchedButton_`):** Pressing any right-side `2x6` switch (`SEL`, `PLAY/STOP`, `SAVE` when arming, `FX`, `ENV`, `LFO`, `EDIT`, `GloRight`, `ARP`, `SEQ`, `REC`) latches its HUD at `opacity = 1.0f` indefinitely until **`HOME`** is pressed (or until `SAVE` completes into `Key 9-16` / is cancelled by a second tap on `SAVE`). Holding and releasing any momentary modifier (`OP1–OP6`, `GLO`, `OCT-/+`, `ScopeFreezeKey`, `Keys 5–8`) or performing a standalone knob/gate tweak (`1.5 s` preview) while a switch HUD is latched smoothly returns to the latched switch HUD instead of fading out to pure scope.
  2. **Continuous Galvanometer Relaxation (`Active Wave <-> y = 120 Flatline`) & Fractional Timebase Zoom (`smoothStride_`):** Remove mid-run `setSilentHorizon()` and per-voice instant snap so all 14 strands + `60x60` bloom relax (`95–190 ms`) onto `y = 120` when audio stops or pauses (`timeSinceScopePushSec_ > 65 ms`) and bloom smoothly back out on resume; slew `smoothStride_` (`[1.0f, 2.0f]`, `160 ms` tau) continuously via Catmull-Rom interpolation across `1x <-> 2x` bass zoom.
  3. **Slewed Filter Curves, Optical Gauges, Outgoing HUD Latch & `HOME` Quick-Dissolve:** Slew `Q8.6` filter curves (`slewQ8_6` with 1-LSB convergence), `OP5` `1 <-> 4` curve divergence, cutoff focal beads, optical-filament gauges, `HOME` LED, and `PLAY/STOP` LED; latch outgoing momentary HUD labels during unlatched fade-out (`releaseLatchHud_`); and quick-dissolve (`~65 ms` tau) on `HOME`.
- **Consequences:** Right-grid switches behave like true persistent hardware switches/pages exited cleanly via `HOME` while momentary holds/tweaks auto-fade naturally, with zero 1-frame visual discontinuities anywhere (`ScopeHudRenderer = 83,616 B`, `RuidoController = 464 B`, `100%` bit-identical strips).
