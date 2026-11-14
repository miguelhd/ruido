# Ruido

**Ruido** is a custom, audiophile-grade **4-oscillator Schmitt-trigger drone synthesizer firmware** for the **M-VAVE FM-1** (`JieLi AC791N / pi32v2`), engineered specifically as a raw, harmonically rich stereo sound source for external guitar pedals and Eurorack modular systems.

## Core Pillars

1. **Uncompromising Sound Quality:** Unified `4x` oversampled (`176.47 kHz`) signal path modeling CMOS Schmitt-trigger physics (PolyBLEP edges, RC capacitor tilt, thermal drift, and shared power-rail threshold coupling) feeding four dedicated per-oscillator **Dual Series Sallen-Key (`12 dB/oct HPF -> 12 dB/oct LPF`, `OTA -> Silicon-Clamped`) Zero-Delay Feedback filters** and a nonlinear master summing stage.
2. **Stateless "Zero-Menu" UX + 3-Model Pressure Keybed:** 1:1 encoder-to-oscillator mapping (`Knobs 1-4 = Oscillators 1-4`) paired with 3 swappable 27-key carbon-pill pressure performance keyboards (**`1/3 LIVE MIXER`**, **`2/3 CIRCUIT LAB`**, and **`3/3 RHYTHM LAB`** via `SEL` + `SELECT`) and 42-channel varibright LED modulation.
3. **Always-On Acoustic Visuals:** Tear-free `240x240` CRT-style rising-edge-triggered oscilloscope rendered continuously via scanline strip compositing, with translucent parameter overlays that gently bloom on touch and dissolve away when idle.

## Repository Layout

```text
Ruido/
├── AGENTS.md              # Root operational rules & architecture map for AI coding agents
├── docs/
│   ├── assets/            # Hardware faceplate reference (fm1-front-panel.png)
│   ├── PRODUCT.md         # Product vision, UX philosophy, voice controls & v1 scope boundaries
│   ├── ARCHITECTURE.md    # Hardware specs, dual-core split, DSP pipeline & memory budgets
│   ├── DECISIONS.md       # Chronological Architecture & Product Decision Records (ADRs)
│   ├── ROADMAP.md         # Phased milestones & verification plan
│   └── WEBSITE.md         # Website copy, sonic philosophy & technical specifications
├── src/                   # 100% platform-agnostic C++17 engine (zero OS/SDK headers)
│   ├── core/              # Shared lock-free state, parameter slews, HAL interface, fixed types
│   ├── dsp/               # 4x oversampled Schmitt oscillators, ZDF filters, decimator
│   └── ui/                # 240x240 RGB565 scanline strip compositor, scope & varibright LED logic
├── platforms/             # Thin target-specific hardware wrappers
│   ├── sim/               # Native macOS / WebAssembly simulator (real-time audio + 240x240 UI)
│   └── fm1/               # M-VAVE FM-1 composite HAL (dinghy/felucca-bsp, fm1_cpu1 SRAM DSP, SARADC, .fwsc)
├── tests/                 # Headless DSP unit tests (aliasing, DC offset, filter stability, CPU bounds)
└── tools/                 # Build scripts, Docker pi32v2 cross-compiler & SysEx flashing utilities
```

## Documentation Index

- **[Product Specification](docs/PRODUCT.md)** — Sound references, stateless UX, LED zoning, and non-goals.
- **[Technical Architecture](docs/ARCHITECTURE.md)** — Hardware specs, `176.47 kHz` DSP graph, scanline UI renderer, and SRAM/Flash budgets.
- **[Decision Log (ADRs)](docs/DECISIONS.md)** — Why key engineering and UX choices were made.
- **[Development Roadmap](docs/ROADMAP.md)** — Step-by-step build and hardware bring-up plan.
- **[Website & Marketing Copy](docs/WEBSITE.md)** — Positioning, feature cards, outboard gear guide, and FAQ.
- **[Agent Guidelines](AGENTS.md)** — Mandatory coding, hygiene, and real-time rules.
