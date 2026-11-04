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

## Milestone 3: Per-Voice LFO, Config Menu, 3-Model Performance Keyboard & Snapshots
- [x] Implement dedicated dual per-voice LFO engine (`8` LFOs: `LFO A/B OSC 1..4`) & `LFO` HUD page (`ADR-022`, `ADR-031`: `SELECT` = `LFO A OSC 1 · LFO B OSC 1 .. LFO B OSC 4`, `KNOB 1–4` = `TARGET | SHAPE | RATE | DEPTH` with continuous `Sine -> Tri -> Saw -> Slewed Square -> Smooth Drift` morphing, `WIDTH` PWM target, `LFO B -> LFO A` `A RATE` / `A DEPTH` cross-modulation, companion sibling curve HUD, and smart `GLOBAL LFO` scaling).
- [x] Implement focused Configuration Menu behind right-grid `GLO` button (`ADR-027`, `ADR-030`, `ADR-036`, `ADR-037`: housing `OUTPUT ROUTING` (`STEREO` / `MONO`), `MUTE MODE` (`MOMENTARY` / `LATCHING`), `SOLO MODE` (`MOMENTARY` / `LATCHING`), and `SNAP MODE` (`LATCHING` / `MOMENTARY`)).
- [x] Implement 2-page `EDIT` cycle (`1st tap` = `DUTY · DRIFT · LEVEL`, `2nd tap` = `PAN`, `3rd tap` = Close), persistent `SEL` + `SELECT` knob **3-Model Performance Keyboard** selector, 7-tier monotonic left-to-right varibright LED zoning, unified per-oscillator circuit mute truth, and **Mode 1: `LIVE MIXER`** (`MUTE · FILTER · OCTAVE · SNAP` + `UNISON`/`SPREAD`) across all 27 pressure-sensitive keys (`ADR-036`, `ADR-037`).
- [ ] Implement **Mode 2: `HARMONIUM`** across all 27 pressure-sensitive keys (interval / reed-organ harmonic performance model + dedicated acoustic visualization & keybed LED choreography).
- [ ] Complete **Mode 3: `CIRCUIT LAB`** across all 27 pressure-sensitive keys (expanding beyond its active `STUTTER` chopper + `OSC`/`GATE` strips to full CMOS cross-modulation & bend routing).
- [x] Implement compact `128-byte` CRC-32 binary snapshot engine (`4` slots on `LIVE MIXER` `Keys 13–16`), `SAVE` + `Keys 13–16` direct slot store, `LATCHING` pressure-accelerated glide / multi-key hybrid blend + `MOMENTARY` non-destructive pressure morph, and stepped `PRESETS` encoder recall across all 3 keyboard models (`ADR-037`).

### Open Product Discussions (Tabled for Post-Playtest Evaluation)
- [ ] **4-Voice `GLOBAL` (`GLO`) Macro — Bring Back vs Retire Permanently:** Keybed `GLO` was repurposed during the 27-key overhaul (the underlying 5-mode 4-voice macro math remains intact in `src/core/` so reviving it is trivial). After playtesting all 3 keyboard models (`LIVE MIXER`, `HARMONIUM`, `CIRCUIT LAB`), decide whether multi-voice `GLOBAL` is essential enough to bring back onto the surface or whether Ruido is a stronger, simpler instrument without it.

---

## Milestone 4: FM-1 Firmware Target & Hardware Bring-Up (`platforms/fm1/`, `tools/`)
- [ ] **Gate 4.1 — Toolchain, `-mcpu=r3 -mfprev1` ISA & `__divsf3` Benchmark (`tools/`, `src/dsp/`):** Create a reproducible `linux/amd64` Docker environment (`/opt/jieli/pi32v2`) targeting the composite community HAL stack (`ADR-034`: `zvenson/dinghy` + `hugelton/felucca-bsp` `MPL-2.0` + `fm1_cpu1.{h,S}`, strictly isolated in `platforms/fm1/`). Compile with `-target pi32v2 -mcpu=r3 -mfprev1 -fno-math-errno -ffp-contract=off` (**never `-ffast-math`**, preserving subnormal scrubbing, `NaN` guards, and simulator bit parity), audit `llvm-objdump` for native `fadds`/`fmuls`, zero `soft-double` / 64-bit integer divisions (`__aeabi_ldivmod`), and `__divsf3` calls, and hoist frame-rate PWM duty reciprocals + apply 2-step Newton-Raphson IEEE-754 reciprocals (`< 1.5e-6` / `~-116 dB` error; verified against `make test-dsp`'s `< -108.5 dBc` alias gate) where `/` lowers to `__divsf3`.
- [ ] **Gate 4.2 — Dual-Core SRAM DSP, 16-Bit TPDF Audio & Clock Domain (`platforms/fm1/`):** Boot **`Core 1` (`cpu1`)** via `fm1_cpu1.{h,S}` with hot DSP code + `.rodata` linked into internal SRAM (`.c1_text`, immune to `cpu0`/`cpu1` XIP I-cache contention and `30–100 ms` NOR Flash sector-erase stalls) and `Core 0 <-> Core 1` shared buses placed in uncached SRAM / explicit D-Cache maintenance. Pace DSP blocks strictly from the hardware **`ALNK0` I2S `24 MHz` crystal DMA** (matching exact hardware rate/block size), calibrate `±1.45f` peak headroom cleanly inside signed 16-bit `[-32768, +32767]` with **1-LSB TPDF dither** (zero hard clipping or truncation fuzz into pedals), and bridge `±50 ppm` USB-C stereo audio host clock drift via fractional interpolation, while preserving power-on **`Hold OCT-` USB rescue mode**.
- [ ] **Gate 4.3 — `Core 0` Scanline DMA, Settled `PA7`/`PA8` `SARADC`, EMI-Safe `74HC595` LEDs & NVS:** Stream `240x16` ping-pong RGB565 strips via `dinghy`'s non-blocking `LCD_IDLE` `SPI1` DMA; wire 7 encoders + 14 buttons + 27 keys + `Velo`-style `PA7`/`PA8` `SARADC` carbon-pill pressure (`White Keys 5–16`) with mux settling delay to prevent crosstalk into the `MASTER` pot; drive the multiplexed `2 × 74HC595` 42-LED matrix via fixed-rate `5/6-bit` gamma PWM or 1st-order `ΔΣ` modulation (zero BAM `127 <-> 128` flash, avoiding `1–4 kHz` ground-plane whine); and bind CRC-verified `settings_*` / `slot_*` flash partitions for `GLOBAL CONFIG` and the 4 `128-byte` Snapshot slots (`White Keys 13–16`).
- [ ] **Verification Gate (`joakimeriksson/fm1-emu` -> Physical M-VAVE FM-1):** Validate the `.fwsc` image inside `fm1-emu` first (Flash `< 600 KB`, SRAM headroom, zero forbidden symbols, `Core 1` cycle margin, glitch-free Flash saves), then flash physical hardware (`Hold OCT-` rescue / USB-MIDI SysEx) and verify thermal/CPU stability, tear-free scope FPS, calibrated pressure curves, stereo 3.5mm TRS + USB-C output into external pedals/Eurorack/DAW, and silent-mode (`PLAY/STOP = STOP`) analog noise floor into `+40 dB` gain across `0–100%` LED brightness.
