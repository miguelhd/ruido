# `src/core/` — Portable Control, Math & Inter-Core Bus Guidelines

`src/core/` defines the pure data contracts, single-precision math primitives, stateless control surface state machine, and 42-channel varibright LED engine shared between `Core 0` (UI/control) and `Core 1` (DSP).

## Layer Rules

1. **Zero Upward Dependencies:** `src/core/` is the foundation layer and MUST NOT `#include` anything from `src/dsp/` or `src/ui/`.
2. **POD Lock-Free Bus Contracts:** `EngineControlState`, `VoiceParams`, `GlobalParams`, `HudState`, and `LedState` must remain trivially copyable, deterministic, fixed-size structs with zero heap pointers.
3. **Pure Header Helpers:** Keep deterministic math (`DspMath.h`, `LfoMath.h`, `FilterModMath.h`), encoder gear/detent curves (`EncoderMath.h`), HUD envelope constants (`HudEnvelope.h`), and global configuration contracts (`GlobalConfig.h`) pure, `noexcept`, and `inline` / `constexpr`.
4. **Modularity Cap:** Keep every `.h` and `.cpp` file in `src/core/` `<= 700` lines. Separate input routing (`RuidoController`, `ControllerPageNav`), keyboard models (`KeyboardEngine`), LED PWM calculation (`LedEngine`), encoder scaling (`EncoderMath`), and scene storage (`SnapshotBank`, `SnapshotCodec`) into distinct modules.
