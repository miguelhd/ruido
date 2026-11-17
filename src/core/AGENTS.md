# `src/core/` — Portable Control, Math & Inter-Core Bus Guidelines

`src/core/` defines the pure data contracts, single-precision math primitives, stateless control surface state machine, and 42-channel varibright LED engine shared between `Core 0` (UI/control) and `Core 1` (DSP).

## Layer Rules

1. **Zero Upward Dependencies:** `src/core/` is the foundation layer and MUST NOT `#include` anything from `src/dsp/` or `src/ui/`.
2. **POD Lock-Free Bus Contracts:** `EngineControlState`, `VoiceParams`, `GlobalParams`, `HudState`, and `LedState` must remain trivially copyable, deterministic, fixed-size structs with zero heap pointers.
3. **Pure Header Helpers:** Keep deterministic math (`DspMath.h`, `LfoMath.h`, `FilterModMath.h`), encoder gear/detent curves (`EncoderMath.h`), HUD envelope constants (`HudEnvelope.h`), and global configuration contracts (`GlobalConfig.h`) pure, `noexcept`, and `inline` / `constexpr`.
4. **Modularity Cap:** Keep every `.h` and `.cpp` file in `src/core/` `<= 700` lines. Separate input routing (`RuidoController`, `ControllerPageNav`, `ControllerEncoder`), keyboard models (`KeyboardEngine`), LED PWM calculation (`LedEngine`), encoder scaling (`EncoderMath`), and scene storage (`SnapshotBank`, `SnapshotCodec`) into distinct modules.
5. **Centralized `SHIFT`, `SCENE` & Unassigned-Control Invariants:**
   - **Centralized `SHIFT` Layer:** While `SHIFT` (`11th` key) is held, `HudMode` MUST remain **`HudMode::ShiftMap` `100%` of the time** (`SHIFT + ALGORITHM` = `TEMPO`, `SHIFT + SELECT` & `SHIFT + OP1..OP3` = `KEYBOARD`, `SHIFT + PRESETS` & `SHIFT + Keys 9-12` = `SCENE`, `SHIFT + Keys 13-16` = `SOLO`), and every keyboard model change (`SEL`, `SELECT`, `SHIFT + SELECT`, `SHIFT + OP1..OP3`) MUST trigger the shared `220 ms` LED wave (`wipePhase_`).
   - **LED Brightness & `SAVE` Purity (`KeyboardModelSpec.h`, `LedEngine.cpp`):** `0%` is reserved strictly for unassigned controls and silent/muted voices; assigned `SCENE` keys (`Keys 9-12`) use the calibrated **`15%` (`EMPTY`) / `40%` (`STORED`) / `80%` (`ACTIVE`)** ladder so `SHIFT` (`100%`) remains the brightest beacon; and while `SAVE` (`STORE` / `DELETE`) is armed, **only** actionable `Keys 9-12` blink while all other 23 keyboard keys rest at `0%`.
   - **Unified Unassigned Feedback:** Never emit `"UNASSIGNED"` text or dedicated punch kinds for `0%` unassigned buttons/keys; route every unassigned press through `triggerUnassignedFeedback` (`phantomPinSeq` CRT ripple + `120 ms` `0% -> 15% -> 0%` local LED glint).
