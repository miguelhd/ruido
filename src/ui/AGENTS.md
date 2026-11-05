# `src/ui/` — Scanline Oscilloscope & HUD Compositor Guidelines

`src/ui/` renders the `240x240` monochrome CRT oscilloscope and anti-aliased sans-serif HUD on `Core 0` via `240x16` `RGB565` scanline strips (`ST7789` SPI DMA ping-pong compatible).

## Layer & Hardware Rules

1. **Read-Only Consumer of `src/core/`:** `src/ui/` MUST NOT `#include` headers from `src/dsp/` or controller state-machine headers (`core/RuidoController.h`, `core/LedEngine.h`). All rendering is a pure function of `EngineControlState`, `HudState`, `EngineTelemetry`, and `ScopeRingBuffer`.
2. **Zero Full-Screen Framebuffer:** Never allocate a `240x240` pixel buffer (`115 KB`). `renderStrip(yStart, stripHeight, rgb565Out)` must produce **bit-for-bit identical** pixels across `8`-row, `16`-row, and `24`-row strip heights, using `<= 6 KB` of stack scratch (`linBuf` + `lumBuf`) passed down to sub-compositors strictly by pointer/reference.
3. **Strictly Neutral Monochrome `RGB565`:** Pack all output pixels through `packNeutralGrayRgb565` (`R5 == B5`, `G6 == (R5 << 1) | (R5 >> 4)`) with 4x4 ordered Bayer dithering—zero color tint.
4. **XIP Flash & LUT Discipline:**
   - Keep per-pixel canvas primitives (`MonoStripCanvas`) and string formatters (`HudFormat.h`) `inline` in headers so `pi32v2` Clang inlines across module boundaries without XIP call penalties.
   - Declare all shared read-only tables (`kGaussLut`, `kTonemapLut`, `kFilterSweepFreqs`) as `inline constexpr` in headers so the linker emits a single `.rodata` copy in Flash.
5. **Modularity Cap:** Keep every handwritten `.h` and `.cpp` file in `src/ui/` `<= 700` lines (exempting generated glyph tables in `FontSansAA.h`), separating trace/bloom (`ScopeTraceEngine`), filter/LFO graphs (`HudGraphs`), global/menu views (`HudGlobalViews`), and strip orchestration (`ScopeHudRenderer`).
6. **Single Source of Truth for Graph Styling (`GraphStyle.h`):** Never copy-paste viewport coordinates, baseline/horizon optical weights, column centers, or unipolar skirt/spine shaders across `HudGraphs`, `HudGlobalViews`, `HudAmpGraph`, or `HudChrome`. Import shared constants and `inline` helpers from `src/ui/GraphStyle.h`, and derive UI test sample regions from those layout constants rather than hardcoded pixel literals.
7. **HUD Header & Nomenclature Grammar (`HudHeader.cpp`, `HudKeybedView.cpp`, `HudFormat.h`):**
   - **Top-Left Primary Title (`leftMain`, `#FFFFFF` Bold):** Name the **musical or circuit parameter/domain** in full, plain English (`FREQUENCY`, `DUTY`, `DRIFT`, `FILTER`, `LEVEL`, `PAN`, `TEMPO`, `TAP TEMPO`, `RAIL COUPLING`, `MASTER LEVEL`, `KEYBOARD`, `GLOBAL`, `SCENE`)—**never** the physical knob/encoder label (`ALGORITHM`, `KNOB 1`, `VOL`) and **never** cryptic abbreviations (`ALG`, `SNAP`, `PW`, `CFG`).
   - **Top-Left Secondary Qualifier (`leftSuffix`, `118/255` Silver Bold):** Reserved **strictly** for target or index disambiguation when multiple instances exist (` OSC 1..4`, ` 1/3..3/3`, ` 1..4`, ` CONFIG`, ` FREQ`, ` FILTER`, ` LFO`, ` LEVEL`, ` ALL`). Single-instance global parameters (`MASTER LEVEL`, `RAIL COUPLING`, `TEMPO`, `TAP TEMPO`, `DRIVE`, `LOW PASS`, etc.) MUST leave `leftSuffix` empty (`""`).
   - **Top-Right Slot (`rightText`) & Copy Hygiene:** Show live acoustic values/units (`110.04 Hz`, `120 BPM`, `25%`, `STEREO` / `MONO`, `12 dB HP -> LP`, `1 / 7`) or `FINE` / `COARSE` rate badges without duplicating body instructions, and stick strictly to printable ASCII (`0x20..0x7E`) matching `FontSansAA.h`.
