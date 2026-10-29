# Ruido

**Ruido** is a custom, audiophile-grade **4-oscillator Schmitt-trigger drone synthesizer firmware** for the **M-VAVE FM-1** (`JieLi AC791N / pi32v2`), engineered specifically as a raw, harmonically rich stereo sound source for external guitar pedals and Eurorack modular systems.

## Core Pillars

1. **Uncompromising Sound Quality:** Unified `4x` oversampled (`176.47 kHz`) signal path modeling CMOS Schmitt-trigger physics (PolyBLEP edges, RC capacitor tilt, thermal drift, and shared power-rail threshold coupling) feeding four dedicated per-oscillator **MS-20 (`Korg35`) dual `HPF -> LPF` Zero-Delay Feedback filters** and a nonlinear master summing stage.
2. **Stateless "Zero-Menu" UX:** 1:1 encoder-to-oscillator mapping (`Knobs 1-4 = Oscillators 1-4`) using momentary hold gestures ("quasimodes") instead of modal pages, paired with Monome-style varibright PWM zoning across the 27-key + 14-button monochrome LED surface.
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
│   └── ROADMAP.md         # Phased milestones & verification plan
├── src/                   # 100% platform-agnostic C++17 engine (zero OS/SDK headers)
│   ├── core/              # Shared lock-free state, parameter slews, HAL interface, fixed types
│   ├── dsp/               # 4x oversampled Schmitt oscillators, ZDF filters, decimator
│   └── ui/                # 240x240 RGB565 scanline strip compositor, scope & varibright LED logic
├── platforms/             # Thin target-specific hardware wrappers
│   ├── sim/               # Native macOS / WebAssembly simulator (real-time audio + 240x240 UI)
│   └── fm1/               # M-VAVE FM-1 BSP (JieLi AC791N / pi32v2, SPI DMA, USB/MIDI, .fwsc)
├── tests/                 # Headless DSP unit tests (aliasing, DC offset, filter stability, CPU bounds)
└── tools/                 # Build scripts, Docker pi32v2 cross-compiler & SysEx flashing utilities
```

## Documentation Index

- **[Product Specification](docs/PRODUCT.md)** — Sound references, stateless UX, LED zoning, and non-goals.
- **[Technical Architecture](docs/ARCHITECTURE.md)** — Hardware specs, `176.47 kHz` DSP graph, scanline UI renderer, and SRAM/Flash budgets.
- **[Decision Log (ADRs)](docs/DECISIONS.md)** — Why key engineering and UX choices were made.
- **[Development Roadmap](docs/ROADMAP.md)** — Step-by-step build and hardware bring-up plan.
- **[Agent Guidelines](AGENTS.md)** — Mandatory coding, hygiene, and real-time rules.
