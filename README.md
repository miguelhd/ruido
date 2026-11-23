# Ruido

**Ruido** is a custom, **4-oscillator Schmitt-trigger drone synthesizer firmware** for the **M-VAVE FM-1** (`JieLi AC791N / pi32v2`), engineered specifically as a raw, harmonically rich stereo sound source for external guitar pedals and Eurorack modular systems.

**User Guide:** [noisexrecords.com/ruido](https://noisexrecords.com/ruido)

## Core Pillars

1. **Uncompromising Sound Quality:** Unified `4x` oversampled (`176.47 kHz` -> `~44.1 kHz` base rate) signal path modeling CMOS Schmitt-trigger physics (PolyBLEP edges, RC capacitor tilt, thermal drift, and shared power-rail threshold coupling) feeding four dedicated per-oscillator **Dual Series Sallen-Key (`12 dB/oct HPF -> 12 dB/oct LPF`, `OTA -> Silicon-Clamped`) Zero-Delay Feedback filters** and a nonlinear master summing stage across 3.5 mm TRS analog and **`44.1 / 48 kHz` USB-C digital stereo audio**.
2. **Stateless UX + 3-Model Pressure Keybed:** 1:1 encoder-to-oscillator mapping paired with 3 swappable pressure sensitive keyboards (**`1/3 LIVE MIXER`**, **`2/3 CIRCUIT LAB`**, and **`3/3 RHYTHM LAB`**) and 42-channel varibright LED modulation.
3. **Always-On Acoustic Visuals:** `240x240` CRT-style rising-edge-triggered oscilloscope rendered continuously via scanline strip compositing, with translucent parameter overlays.

## Repository Layout

```text
Ruido/
├── AGENTS.md              # Root operational rules & architecture map for AI coding agents
├── docs/
│   ├── assets/            # Hardware faceplate reference (fm1-front-panel.png)
│   ├── PRODUCT.md         # Product vision, UX philosophy, voice controls & v1 scope boundaries
│   ├── ARCHITECTURE.md    # Hardware specs, dual-core split, DSP pipeline & memory budgets
│   ├── DECISIONS.md       # Chronological Architecture & Product Decision Records (ADRs)
│   └── ROADMAP.md         # Phased milestones & verification plan
├── src/                   # 100% platform-agnostic C++17 engine (zero OS/SDK headers)
│   ├── core/              # Shared lock-free state, parameter slews, HAL interface, fixed types
│   ├── dsp/               # 4x oversampled Schmitt oscillators, ZDF filters, decimator
│   └── ui/                # 240x240 RGB565 scanline strip compositor, scope & varibright LED logic
├── platforms/             # Thin target-specific hardware wrappers
│   ├── sim/               # Native macOS / WebAssembly simulator (real-time audio + 240x240 UI)
│   └── fm1/               # M-VAVE FM-1 target (Felucca / Dinghy HAL, fm1_cpu1 SRAM DSP, SARADC, .fwsc)
├── tests/                 # Headless DSP, UI, controller & Wasm verification suites
└── tools/                 # Build scripts, Docker pi32v2 cross-compiler & SysEx flashing utilities
```

## Quick Start

```bash
# Run hygiene, DSP, UI, macOS CoreAudio headless, and Wasm V8 verification suites
make test

# Launch the bit-identical browser simulator at http://127.0.0.1:8080
make serve
```

## Building & Flashing M-VAVE FM-1 Hardware (`FM-1_312`)

```bash
# 1. One-time reproducible linux/amd64 toolchain + fm1-emu Docker image (~450 MB)
make fm1-image

# 2. Compile build/fm1/ruido.fwsc and run all 5 offline fm1-emu anti-brick & NVS gates
make fm1-check

# 3. Verify USB-MIDI connection (read-only) and flash over app-only FELUCCA-LOADER-1
make fm1-info
make fm1-flash
```

- **USB Rescue Mode (`Hold OCT-`):** Holding **`OCT-`** while powering on the FM-1 enters standalone USB-MIDI rescue mode before any synthesis or `Core 1` code runs, so an interrupted update can always be resumed with `make fm1-flash`.

## Documentation Index

- **[Product Specification](docs/PRODUCT.md)** — Sound references, UX, LED zoning, and non-goals.
- **[Technical Architecture](docs/ARCHITECTURE.md)** — Hardware specs, `176.47 kHz` DSP graph, scanline UI renderer, and SRAM/Flash budgets.
- **[Decision Log (ADRs)](docs/DECISIONS.md)** — Why key engineering and UX choices were made.
- **[Development Roadmap](docs/ROADMAP.md)** — Step-by-step build and hardware bring-up plan.
- **[Agent Guidelines](AGENTS.md)** — Mandatory coding, hygiene, and real-time rules.

## License

Ruido is licensed under the **GNU General Public License v3.0 only (`GPL-3.0-only`)** — see **[LICENSE](LICENSE)** and **[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)** for full license terms, third-party attributions, and trademark disclaimers.
