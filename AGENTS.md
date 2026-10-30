# Ruido — Agent Guidelines & Engineering Rules

You are working on **Ruido**, a custom embedded drone synthesizer firmware for the **M-VAVE FM-1** (`JieLi AC791N` dual-core `pi32v2` @ 240-320 MHz, ~578 KB SRAM, ~600 KB XIP Flash app ceiling) paired with a bit-identical macOS / WebAssembly simulator.

## 1. Product & Engineering Priorities (Strict Order)

1. **Sound Quality Above All:** Spend CPU headroom on anti-aliasing, oversampling, Zero-Delay Feedback (ZDF) filter accuracy, nonlinear analog circuit behavior, and clickless parameter smoothing—never on feature bloat.
2. **Stateless, Immediate UX:** Never introduce scrolling menus, modal state traps, or multi-page config screens. Default idle state is always edge-to-edge live waveform + `KNOB1-4 = Osc 1-4 Speed`, with momentary holds strictly on the bottom-left/center black keys (`OP1-OP6`, `GLO`) so hands never cross over the `240x240` screen, and single-tap utilities only on the right button grid.
3. **Useful Acoustic Visuals:** Every pixel on the `240x240` display and every PWM level on the 27-key + 14-button monochrome LEDs must reflect real acoustic/circuit state.
4. **Code Hygiene & Portability:** Keep `src/` 100% platform-independent so identical C++17 code runs in `platforms/sim/` (macOS/Wasm), `tests/`, and `platforms/fm1/` (`pi32v2` Clang).

## 2. Canonical Documentation Map (Read Before Modifying Architecture)

- `docs/PRODUCT.md` & `docs/assets/fm1-front-panel.png` — Instrument spec, physical faceplate reference, two-hand control & `SARADC` pressure map, stateless HUD behavior, keybed LED zones, and explicit non-goals.
- `docs/ARCHITECTURE.md` — FM-1 hardware constraints, dual-core (`Core 0` UI / `Core 1` DSP) split, `176.47 kHz` 4x oversampled DSP chain, and `240x16` scanline strip compositor.
- `docs/DECISIONS.md` — Chronological ADR log. **Check this before proposing alternative designs**, and append a new ADR whenever a major architectural or UX decision changes.
- `docs/ROADMAP.md` — Current milestone tracking and verification gates.

## 3. Repository Boundaries & Hygiene Rules

- **Strict Platform Isolation:** Files inside `src/core/`, `src/dsp/`, and `src/ui/` MUST NOT include JieLi SDK headers, FreeRTOS headers, WebAssembly/Emscripten headers, or macOS headers. All hardware I/O crosses the thin C/C++ interface in `src/core/`.
- **Embedded C++17 Subset:**
  - Compile cleanly with `-std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti`.
  - No `std::function`, `std::shared_ptr`, `<iostream>`, `<sstream>`, or dynamic STL containers (`std::vector`, `std::map`, `std::string`) in runtime firmware code; use fixed-capacity `std::array`, `constexpr` tables, and POD structs.
  - Format all C/C++ code according to `.clang-format` (4-space indent, 100-col limit, LLVM-based).
- **Binary & Memory Discipline:**
  - Target firmware + read-only lookup tables must stay well below the **600 KB XIP Flash** ceiling (`-Oz` / `-Os` + LTO).
  - Total runtime static + stack + heap RAM usage by our app layer must leave generous headroom within the **578 KB total SoC SRAM** (shared with FreeRTOS + USB/MIDI + `jlstream`). Never allocate a full `240x240` framebuffer (use `240x16` ping-pong scanline strips).
- **Decision & Spec Sync:** When adding parameters, changing DSP topology, or altering button/key mappings, update `docs/PRODUCT.md`, `docs/ARCHITECTURE.md`, and `docs/DECISIONS.md` in the same change.
- **Periodic Logical Commits:** Commit verified changes periodically in clean, logical commits (e.g., `dsp`, `ui`, `sim`, `docs`) once `make test` passes—do not let large multi-feature diffs accumulate uncommitted in the working tree.
- **Lean Dev Simulator (`platforms/sim/web/`):** The browser UI is strictly a local hardware test harness for FM-1 firmware development, not a web product. Do not spend time or tokens on ARIA attributes, screen-reader state plumbing, web accessibility audits, or `modern-web-guidance` skill lookups when modifying `platforms/sim/web/`.
