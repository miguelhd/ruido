# Ruido — Development Roadmap

Each milestone has a strict verification gate before advancing to the next phase so sound quality and code hygiene never regress.

---

## Milestone 0: Workspace Hygiene & Documentation Baseline *(Complete)*
- [x] Initialize standalone Git repository and clean `src/` / `platforms/` / `tests/` / `docs/` separation.
- [x] Establish C++17 formatting (`.clang-format`, `.editorconfig`) and agent guardrails (`AGENTS.md`, `src/dsp/AGENTS.md`).
- [x] Record baseline product spec (`docs/PRODUCT.md` + `docs/assets/fm1-front-panel.png`), hardware/DSP architecture (`docs/ARCHITECTURE.md`), and ADRs (`ADR-001` through `ADR-009` in `docs/DECISIONS.md`).

---

## Milestone 1: Core DSP Engine & Headless Audio Verification (`src/dsp/`, `tests/`) *(Complete)*
- [x] Implement lock-free parameter state & per-sample clickless slew smoothers (`src/core/`).
- [x] Implement `4x` oversampled **Schmitt-Trigger Oscillator** (PolyBLEP, RC capacitor tilt, `1/f` drift, `ALGORITHM`-scaled shared power-rail threshold coupling, Audio `20-4000 Hz` + LFO `0.02-20 Hz` ranges).
- [x] Implement per-oscillator **MS-20 `Rev 2` (`KLM-307` `LM13600` OTA Sallen-Key) ZDF `HPF -> LPF`** filter pair with nonlinear dual-OTA poles, `1S1588` silicon feedback limiters, and Smart-Park Independent `HP` & `LP` Resonance (`ADR-004`, `ADR-019`, `ADR-024`).
- [x] Implement constant-power stereo panner + `SELECT` stereo spread + `MONO`/`POLY` collapse, clickless mute/pressure modulation, nonlinear master bus sag, and cascaded polyphase half-band `4x -> 1x` stereo decimator.
- [x] **Verification Gate:** Headless C++ unit tests verifying zero `NaN`/denormals under extreme self-oscillation, alias suppression (`> 70 dB` rejection — achieved `< -108.5 dBc`), zero DC offset runaway, and flat passband when filters are parked.

---

## Milestone 2: Interactive macOS / WebAssembly Simulator (`platforms/sim/`, `src/ui/`) *(Complete)*
- [x] Implement the portable `240x16` RGB565 scanline strip compositor (`src/ui/`):
  - Osc-1 rising-edge-triggered anti-aliased CRT stereo waveform renderer.
  - Alpha-blended translucent HUD (`120 ms` bloom, `1.5 s` hold, `800 ms` dissolve) including `[ HP |══════| LP ]` filter passband curve and headroom indicator.
- [x] Implement stateless two-hand controller (`OP1-4` filter holds, `OP5` Amp, `OP6` Pan, `GLO` global filter, `PIT` pitch pressure, left encoders, right single-taps) + encoder velocity ballistics + 27-key / 14-button varibright white LED engine (`src/core/`, `src/ui/`).
- [x] Wrap portable engine in `platforms/sim/` (WebAssembly + WebAudio + faithful FM-1 front-panel UI matching `docs/assets/fm1-front-panel.png`, including simulated finger-pressure expression on keys `5-16` & MIDI Aftertouch, plus native macOS `CoreAudio` live runner).
- [x] **Verification Gate:** Automated unit + UBSan + Node V8 Wasm tests and real-time auditioning (`make serve` WebAudio & `./build/ruido_sim_macos` CoreAudio) from macOS audio interface directly into external pedals / Eurorack.

---

## Milestone 3: Per-Voice LFO, Config Menu & Drone Snapshots
- [x] Implement dedicated 4-voice LFO engine & `LFO` HUD page (`ADR-022`: `SELECT` = `Osc 1..4 / All`, `KNOB 1–4` = `TARGET | SHAPE | RATE | DEPTH` with continuous `Sine -> Tri -> Saw -> Slewed Square -> Smooth Drift` morphing and `7` voice circuit targets).
- [ ] Implement focused Configuration Menu behind right-grid `GLO` button (housing `Stereo / Dual Mono` output routing and global hardware settings).
- [ ] Implement compact binary snapshot struct (`<= 128 bytes` per snapshot) for the 8 snapshot keys (`White Keys 9-16`).
- [ ] Implement smooth glide on tap, continuous macro scrub/crossfade via the `PRESETS` encoder, and **rubber-band pressure morphing** when leaning into a snapshot key (`White Keys 9-16`).
- [ ] Implement **real-time keybed LED visualization** during `PRESETS` Continuous Macro Scrub (`ADR-025`): smoothly pan/crossfade varibright PWM brightness (`20% <-> 100%` across populated slots, `0%` on empty slots) across `White Keys 9-16` in real time to reflect exact hybrid morph weights (staying parked on intermediate blends even after the HUD dissolves), while `White Keys 1-4` simultaneously track live morphed voice amplitudes and gates.

---

## Milestone 4: FM-1 Firmware Target & Hardware Bring-Up (`platforms/fm1/`, `tools/`)
- [ ] Create reproducible `linux/amd64` Docker build environment in `tools/` with JieLi `pi32v2` Clang toolchain + `fm1_open` / `fw-AC79_AIoT_SDK` BSP.
- [ ] Bind `platforms/fm1/` to Core 1 `jlstream` 64-sample audio callback, Core 0 SPI LCD DMA strip push, 7-encoder + button/key GPIO scanner, `SARADC` carbon-pill pressure reader, monochrome LED PWM driver, and NVS snapshot persistence.
- [ ] Validate compiled `.fwsc` binary inside `fm1-emu` (verifying Flash `< 600 KB`, SRAM headroom, and Core 1 cycle margin).
- [ ] **Verification Gate:** Flash `.fwsc` to physical M-VAVE FM-1 via USB-MIDI SysEx and verify thermal/CPU stability, tear-free scope framerate, calibrated carbon-pill pressure response, and stereo TRS output into external pedals/Eurorack.
