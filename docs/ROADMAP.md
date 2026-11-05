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

## Milestone 3: Per-Voice LFO, Config Menu, 3-Model Performance Keyboard & Snapshots *(Complete)*
- [x] Implement dedicated dual per-voice LFO engine (`8` LFOs: `LFO A/B OSC 1..4`) & `LFO` HUD page (`ADR-022`, `ADR-031`: `SELECT` = `LFO A OSC 1 · LFO B OSC 1 .. LFO B OSC 4`, `KNOB 1–4` = `TARGET | SHAPE | RATE | DEPTH` with continuous `Sine -> Tri -> Saw -> Slewed Square -> Smooth Drift` morphing, `WIDTH` PWM target, `LFO B -> LFO A` `A RATE` / `A DEPTH` cross-modulation, companion sibling curve HUD, and smart `GLOBAL LFO` scaling).
- [x] Implement focused Configuration Menu behind right-grid `GLO` button (`ADR-027`, `ADR-030`, `ADR-036`, `ADR-037`, `ADR-040`, `ADR-042`: housing `OUTPUT ROUTING` (`STEREO` / `MONO`), `MUTE MODE` (`MOMENTARY` / `LATCHING`), `SOLO MODE` (`MOMENTARY` / `LATCHING`), `SCENE MODE` (`LATCHING` / `MOMENTARY`), `MIDI CLOCK IN` (`AUTO` / `OFF`), `MIDI TRANSPORT IN` (`PHASE` / `RUN/STOP`), and `MIDI CHANNEL IN` (`OMNI` / `CH 1–16`)).
- [x] Implement 2-page `EDIT` cycle (`1st tap` = `DUTY · DRIFT · LEVEL`, `2nd tap` = `PAN`, `3rd tap` = Close), persistent `SEL` + `SELECT` knob **3-Model Performance Keyboard** selector, 7-tier monotonic left-to-right varibright LED zoning, unified per-oscillator circuit mute truth, and **Mode 1 (`1/3`): `LIVE MIXER`** (`MUTE · FILTER · OCTAVE · SCENE` + `UNISON`/`SPREAD`) across all 27 pressure-sensitive keys (`ADR-036`, `ADR-037`, `ADR-039`).
- [x] Complete **Mode 2 (`2/3`): `CIRCUIT LAB` (`CMOS PATCH BAY`)** across all 27 pressure-sensitive keys and 42 LEDs (`SOLO 1–4`, `MUTE 1–4`, `CROSS-PIN 5–8`, `FOLD 9–12` `1x->6.2x` capacitor wavefolder, `CRUSH 13–16` `CD4040` sub-XOR + `CD4013` S&H clock crush, `OP5 STARVE`, `OP6 RING MOD`, `PIT TURBO LFO`, `GLO SCREAM`, `MONO HARD SYNC`, `POLY SCATTER`, universal `11th FREEZE` + modular `CircuitPatchMatrix` / `KeyboardModel` architecture, `ADR-038`).
- [x] Complete **Mode 3 (`3/3`): `RHYTHM LAB` (`CLOCK, DIVIDE & EUCLID`)** + **Port-Agnostic `24 PPQN` MIDI Clock In** (`SOLO 1–4`, `MUTE 1–4`, `STUTTER 5–8`, `DIVIDE 9–12` `/2..X8`, `EUCLID 13–16` 16-step rotated Euclidean grooves, `OP5 PUMP`, `OP6 CHASE`, `PIT TEMPO`, `GLO PLUCK` live MS-20 filter plucks, `MONO SHORT`, `POLY SWING`, logarithmic `20–3,000 BPM` `ALGORITHM` tempo + `11th TAP TEMPO`, WebMIDI USB/TRS/BLE clock sync in simulator, and BLE-jitter-immune 24-tick PLL, `ADR-040`, `ADR-042`).
- [x] Implement compact `128-byte` CRC-32 binary snapshot engine (`4` slots on `LIVE MIXER` `Keys 13–16`), `SAVE` + `Keys 13–16` direct slot store, `LATCHING` pressure-accelerated glide / multi-key hybrid blend + `MOMENTARY` non-destructive pressure morph, and stepped `PRESETS` encoder recall across all 3 keyboard models (`ADR-037`, `ADR-039`).
- [x] Permanently retire the legacy 5-mode 4-voice `GLOBAL` macro (`ADR-043`), stripping dead multi-voice encoder/HUD paths ahead of Milestone 4 hardware bring-up while retaining 4-voice MS-20 filter readout/graph support for keybed `GLO` punch-ins (`LOW PASS`, `SCREAM`, `PLUCK`).

---

## Milestone 4: FM-1 Firmware Target & Hardware Bring-Up (`platforms/fm1/`, `tools/`)
- [ ] **Gate 4.1 — Toolchain, `-mcpu=r3 -mfprev1` ISA, Anti-Brick Flags & `__divsf3` Benchmark (`tools/`, `src/dsp/`):** Create a reproducible `linux/amd64` Docker environment (`/opt/jieli/pi32v2`) targeting `zvenson/dinghy` (`ADR-034`, strictly isolated in `platforms/fm1/`). Compile with `-target pi32v2 -mcpu=r3 -mfprev1 -Wglobal-constructors -Wexit-time-destructors -fno-math-errno -ffp-contract=off` (**never `-ffast-math`**, guaranteeing zero pre-boot `.init_array` constructors before `dinghy`'s `Hold OCT-` rescue check and preserving subnormal/`NaN` guards), verify `dinghy`'s `app.ld` flash bounds (`>= 0x4000` / `0x02000120` XIP, `< 600 KB` `.fwsc` well below `0xEA000` `USR` flash, zero `0x0000–0x3FFF` boot-head writes), audit `llvm-objdump` for native `fadds`/`fmuls` and zero `soft-double` / `__aeabi_ldivmod` calls, and apply 2-step Newton-Raphson IEEE-754 reciprocals (`< 1.5e-6` / `~-116 dB` error; verified against `make test-dsp`'s `< -108.5 dBc` alias gate) where `/` lowers to `__divsf3`.
- [ ] **Gate 4.2 — Dual-Core SRAM DSP, 16-Bit TPDF Audio & Clock Domain (`platforms/fm1/`):** Construct engine state via placement `new` strictly *after* `dinghy`'s power-on **`Hold OCT-` USB rescue mode** check clears; boot **`Core 1` (`cpu1`)** via `fm1_cpu1.{h,S}` with hot DSP code + `.rodata` in internal SRAM (`.c1_text`, immune to `30–100 ms` NOR Flash sector-erase stalls) and `Core 0 <-> Core 1` shared buses in uncached SRAM. Pace DSP blocks strictly from the hardware **`ALNK0` I2S `24 MHz` crystal DMA**, calibrate `±1.45f` peak headroom cleanly inside signed 16-bit `[-32768, +32767]` with **1-LSB TPDF dither**, and bridge `±50 ppm` USB-C stereo audio host clock drift via fractional interpolation.
- [ ] **Gate 4.3 — `Core 0` Scanline DMA, Settled `PA7`/`PA8` `SARADC`, EMI-Safe `74HC595` LEDs, MIDI Ports & Safe `USR` NVS:** Stream `240x16` ping-pong RGB565 strips via `dinghy`'s non-blocking `LCD_IDLE` `SPI1` DMA; wire 7 encoders + 14 buttons + 27 keys + `Velo`-style `PA7`/`PA8` `SARADC` carbon-pill pressure (`White Keys 5–16`) with mux settling delay; route `3.5mm TRS` UART MIDI + `USB-C` MIDI System Real-Time clock (`0xF8/0xFA/0xFB/0xFC`) directly into the transport-agnostic MIDI clock follower and integrate JieLi's trimmed `AC791N` BLE peripheral GATT service (`03B80E5A-...`) unpacking `13-bit` ms packet timestamps for wireless **BLE-MIDI** clock sync; drive the multiplexed `2 × 74HC595` 42-LED matrix via fixed-rate `5/6-bit` gamma PWM or 1st-order `ΔΣ` modulation; and persist `GLOBAL CONFIG` + the 4 `128-byte` Scene slots exclusively through `dinghy`'s CRC-verified `settings_*` / `slot_*` `USR` flash APIs (`0xEA000–0xFC000`, never raw SPI flash writes).
- [ ] **Verification Gate (`joakimeriksson/fm1-emu` -> Physical M-VAVE FM-1):** Validate every `.fwsc` image inside `fm1-emu` **before** ever touching physical hardware (verifying `>= 0x4000` offset, `< 600 KB` size, zero `.init_array` global constructors, working power-on `Hold OCT-` USB rescue entry, SRAM headroom, `Core 1` cycle margin, and glitch-free `USR` saves), then flash physical hardware with uninterrupted power and verify thermal/CPU stability, tear-free scope FPS, calibrated pressure curves, stereo 3.5mm TRS + USB-C output, and silent-mode (`PLAY/STOP = STOP`) analog noise floor into `+40 dB` gain across `0–100%` LED brightness.

---

## Post-Launch (v1.x+): Full MIDI Expansion
- [ ] **External MIDI Note In & Voice / Envelope Architecture:** Design dedicated MIDI note articulation, envelope/LPG behavior, velocity/aftertouch expression, and tonal/paraphonic vs 27-key performance routing across TRS MIDI IN, USB-C, and BLE-MIDI (`ADR-039`) without compromising continuous drone sound quality or Core 1 CPU headroom.
- [ ] **MIDI CC & Program Change In (`ON` / `OFF`):** Hands-free `SCENE 1–4` morph recall via Program Change (`0–3`) and a curated fixed CC map across oscillator pitches, MS-20 filters, dual LFOs, overdrive, and rail coupling.
- [ ] **USB-C / BLE-MIDI Clock Out & BLE Radio Control:** Optional `24 PPQN` MIDI Clock Out over bidirectional USB-C and BLE-MIDI (with rate clamping/division above `300 BPM`; physical 3.5mm TRS jack is IN-only on the FM-1 PCB) plus a `BLE MIDI` (`ON` / `OFF`) radio toggle in `GLOBAL CONFIG` for battery and RF hygiene.
