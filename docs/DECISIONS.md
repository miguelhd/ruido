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
- **Date:** 2026-10-29 · **Status:** Accepted
- **Context:** Community hardware research demonstrated that sampling `AC791N` `SARADC` channels across the silicone key matrix rows reads the analog resistance drop of the keys' conductive carbon pills under finger force without hardware modification.
- **Decision:** Integrate continuous per-key pressure sensing on Core 0 to drive **Soma Lyra-8-style drone expression**: (1) swelling muted oscillators or opening MS-20 filter cutoff/drive (or pitch bend when `PIT` is active) on White Keys `5-8`, and (2) momentary **rubber-band snapshot morphing** when leaning into Snapshot White Keys `9-16`.
- **Consequences:** Turns inexpensive rubber dome keys into a deeply expressive tactile drone surface.
