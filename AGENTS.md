# Ruido — Agent Guidelines & Engineering Rules

You are working on **Ruido**, a custom embedded drone synthesizer firmware for the **M-VAVE FM-1** (`JieLi AC791N` dual-core `pi32v2` @ 240-320 MHz, ~578 KB SRAM, ~600 KB XIP Flash app ceiling) paired with a bit-identical macOS / WebAssembly simulator.

## 1. Product & Engineering Priorities (Strict Order)

1. **Sound Quality Above All:** Spend CPU headroom on anti-aliasing, oversampling, Zero-Delay Feedback (ZDF) filter accuracy, nonlinear analog circuit behavior, and clickless parameter smoothing—never on feature bloat.
2. **Stateless, Immediate UX:** Avoid multi-page modal state traps. Default idle state is always edge-to-edge live waveform + `KNOB1-4 = Osc 1-4 Speed`, with momentary holds strictly on the bottom-left/center black keys (`OP1-OP6`, `GLO`) so hands never cross over the `240x240` screen, single-tap utilities on the right button grid, and a single flat `GLOBAL CONFIG` list menu on right `GLO`.
3. **Useful Acoustic Visuals:** Every pixel on the `240x240` display and every PWM level on the 27-key + 14-button monochrome LEDs must reflect real acoustic/circuit state.
4. **Code Modularity, Hygiene & Portability:** Keep `src/` modular, strictly layered, and 100% platform-independent so identical C++17 code runs in `platforms/sim/` (macOS/Wasm), `tests/`, and `platforms/fm1/` (`pi32v2` Clang).

## 2. Canonical Documentation Map (Read Before Modifying Architecture)

- `docs/PRODUCT.md` & `docs/assets/fm1-front-panel.png` — Instrument spec, physical faceplate reference, two-hand control & `SARADC` pressure map, stateless HUD behavior, keybed LED zones, and explicit non-goals.
- `docs/ARCHITECTURE.md` — FM-1 hardware constraints, dual-core (`Core 0` UI / `Core 1` DSP) split, `176.47 kHz` 4x oversampled DSP chain, and `240x16` scanline strip compositor.
- `docs/DECISIONS.md` — Chronological ADR log. **Check this before proposing alternative designs**, and append a new ADR whenever a major architectural or UX decision changes.
- `docs/ROADMAP.md` — Current milestone tracking and verification gates.
- `src/core/AGENTS.md`, `src/dsp/AGENTS.md`, `src/ui/AGENTS.md` — Layer-specific modularity, real-time DSP, and scanline compositor rules.

## 3. Repository Boundaries & Automated Hygiene Rules (`make check`)

- **Strict Platform & Layer Isolation (Acyclic DAG):**
  - Files inside `src/core/`, `src/dsp/`, and `src/ui/` MUST NOT include JieLi SDK headers, FreeRTOS headers, WebAssembly/Emscripten headers, or macOS headers. All hardware I/O crosses the thin C/C++ interface in `src/core/`.
  - `src/core/` MUST NOT include `dsp/` or `ui/` headers.
  - `src/dsp/` MUST NOT include `ui/` headers or UI controller headers (`RuidoController.h`, `LedEngine.h`).
  - `src/ui/` MUST NOT include `dsp/` headers or controller implementation headers (`RuidoController.h`, `LedEngine.h`); `src/ui/` consumes only `const EngineControlState&`, `const HudState&`, and `const EngineTelemetry&`.
- **Single-Responsibility Modularity & File Size Cap:**
  - Every handwritten `.h` and `.cpp` file in `src/` MUST stay **`<= 700` lines** (exempting baked ROM data headers such as `src/ui/FontSansAA.h`). Split growing subsystems into cohesive modules rather than accumulating multi-thousand-line monoliths.
  - Keep hot inner-loop math/pixel primitives `inline` in headers (`DspMath.h`, `EncoderMath.h`, `MonoCanvas.h`, `HudFormat.h`) and shared ROM lookup tables `inline constexpr` so multi-TU splits incur zero XIP Flash cache-miss overhead or duplicate `.rodata` tables on `AC791N`.
  - Keep `Makefile` (`DSP_SRCS` + `UI_SRCS`) and `platforms/sim/wasm_bridge.cpp` unity-includes 1-to-1 synchronized (enforced by `tools/check_hygiene.py`).
- **Embedded C++17 Subset:**
  - Compile cleanly with `-std=c++17 -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -fno-exceptions -fno-rtti`.
  - No `<cmath>` / `libm`, `std::function`, `std::shared_ptr`, `<iostream>`, `<sstream>`, or dynamic STL containers (`std::vector`, `std::map`, `std::string`) in `src/`; use fixed-capacity `std::array`, `constexpr` tables, and POD structs.
  - Format all C/C++ code according to `.clang-format` (`make format`; verified by `make check`).
- **Binary & Memory Discipline:**
  - Target firmware + read-only lookup tables must stay well below the **600 KB XIP Flash** ceiling (`-Oz` / `-Os` + LTO).
  - Total runtime static + stack + heap RAM usage by our app layer must leave generous headroom within the **578 KB total SoC SRAM** (shared with `Core 1` `.c1_text` SRAM DSP code, uncached inter-core buses, and `dinghy`/`felucca-bsp` USB/MIDI/DMA buffers). Never allocate a full `240x240` framebuffer (use `240x16` ping-pong scanline strips, passing strip buffers by pointer/reference with `0 B` extra stack copies).
- **FM-1 Flash Memory Map & Anti-Brick Safety (Mandatory):**
  - **Never touch `0x0000 – 0x3FFF` (Boot Head / U-Boot) or `dinghy`'s boot layout:** Writing below `0x4000` hard-bricks the FM-1 (requiring external RP2040 hardware recovery). Keep `src/` 100% free of custom `__attribute__((section(...)))` directives and leave `dinghy`'s proven `app.ld` (`0x4000` / `0x02000120` XIP start, `< 0xEA000` app boundary) and early `platform.c` startup intact.
  - **Protect Power-On `Hold OCT-` USB Rescue (`-Wglobal-constructors -Wexit-time-destructors`):** Because the FM-1's USB-MIDI firmware updater lives inside the firmware image itself, any C++ `.init_array` static constructor crashing before `main()` defeats power-on **`Hold OCT-` USB Rescue Mode** and soft-bricks USB recovery. `src/` MUST emit zero global C++ constructors or exit destructors; instantiate engine objects strictly via explicit placement `new` *after* `dinghy`'s `Hold OCT-` check clears.
  - **Safe `USR` Flash (`0xEA000 – 0xFC000`) & Mandatory `fm1-emu` Pre-Flash Gate:** Never access raw SPI NOR flash registers in `src/`; pass only CRC-verified in-RAM POD buffers to `dinghy`'s `slot_*` and `settings_*` APIs, enforce `< 600 KB` `.fwsc` size at build time, and verify boot + `Hold OCT-` rescue inside `joakimeriksson/fm1-emu` before ever flashing physical hardware.
- **Fast Domain-Scoped Verification (`make test-ui` vs `make test`):**
  - For UI, HUD, LED, or controller-only changes, verify with `make test-ui` (~1.5s: runs `make check`, `./build/test_ui`, `ruido.wasm`, and `verify_wasm.mjs`). Run `make ubsan-ui` only when modifying pointer/buffer indexing or fixed-point math.
  - For DSP or audio-engine changes, run `make test-dsp` (and `make tsan` / `make ubsan` when touching concurrency or DSP math). Run the full `make test` suite at milestone boundaries or cross-layer changes.
- **Low-Frequency, High-Level Doc Sync:** Update `docs/PRODUCT.md`, `docs/ARCHITECTURE.md`, and `docs/DECISIONS.md` **only** when user-facing features, hardware button/encoder mappings, parameter ranges, or DSP/system architecture change—never for visual/aesthetic tuning (line thickness, brightness, opacity, layout polish). Keep docs focused on product behavior and architecture, free of C++ function names or pixel coordinates.
- **Batched Logical Commits:** Batch iterative visual/UX polish and commit in clean, logical milestones once a feature or cohesive round of refinements has settled—do not run a commit cycle on every micro-tweak during interactive visual iteration.
- **Lean Dev Simulator (`platforms/sim/web/`):** The browser UI is strictly a local hardware test harness for FM-1 firmware development, not a web product. Do not spend time or tokens on ARIA attributes, screen-reader state plumbing, web accessibility audits, or `modern-web-guidance` skill lookups when modifying `platforms/sim/web/`.
