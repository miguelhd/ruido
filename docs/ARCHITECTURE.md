# Ruido — Technical & Hardware Architecture

## 1. Target Hardware Profile (`M-VAVE FM-1`)

| Subsystem | Hardware Component | Key Constraints & Notes |
| :--- | :--- | :--- |
| **SoC** | Zhuhai JieLi **AC791N** (`WL82`) | AIoT multimedia + BLE SoC |
| **CPU** | **Dual-core 32-bit `pi32v2`** @ **240-320 MHz** | Proprietary Blackfin-derived RISC/DSP ISA; hardware **single-precision FPU** (`-mcpu=r3 -mfprev1`: single-cycle `fadds`/`fsubs`/`fmuls` + `2.66`-cycle native `fdiv` with `0` `__divsf3`, strict IEEE-754 `-fno-math-errno -ffp-contract=off`, never `-ffast-math`), shared cache, MMU |
| **SRAM** | **~578 KB internal SRAM** | Hosts `Core 1` `.c1_text` DSP code + `.rodata` (`56.4 KiB` @ `0x01C0E000`, `15.6 KiB` free, `0` XIP refs, zero Flash stall/cache contention) + hot `Core 1` DSP state (`6.00 KiB`), cache-line-isolated `.pool` shared buses (`41.88 KiB` + `16`-block audio ring + cost-balanced filter co-processor + UI/panel/LED arenas = `191.4 KiB`), and `.data+.bss` (`16.2 KiB`), leaving `> 300 KiB` free SRAM headroom |
| **Flash** | **1 MB XIP NOR Flash** (`0x02000000`; `P25Q80` `0x856014` & `UC25HQ80` `0xB36014`) | Code executes in-place on `Core 0` (`318.5 KiB` app XIP = `56.1%` of `567.9 KiB` `APP_SLOT`; `599.5 KiB < 600 KiB` `.fwsc`), plus dual-CRC `0xF4000–0xF8FFF` `USR` flash storage accepting both FM-1 NOR flash chips |
| **Audio I/O** | **~44.1 kHz (`44,118 Hz`)** stereo, **64-frame blocks** | `~1.45 ms` block budget paced strictly by hardware **`ALNK0` I2S `24 MHz` crystal DMA** (`±1.45f`-calibrated 16-bit `int16_t` headroom + **1-LSB TPDF dither** -> zero hard clipping or truncation fuzz into pedals). Stereo 3.5mm TRS out (~1-2 Vpp), auto-muting internal speaker, plus fractional-interpolated USB-C digital stereo audio (`44.1 / 48 kHz`) |
| **Display** | **1.54" `240x240` RGB565 TFT LCD** | Driven over `30 MHz` (`LCD_BAUD=1`) `SPI1` (`~30.5 FPS` full-frame `240x240x16b` wire ceiling; `~25–29 FPS` live with persistent cross-frame ping-pong non-blocking `LCD_IDLE` strip DMA + 2-row inter-slice `Core 0` DSP assist) |
| **Rotary Controls** | **1x `MASTER` analog pot + 7x endless encoders** | Left: `SELECT`, `PRESETS`, `ALGORITHM` (no push switches); Right: `KNOB1`-`KNOB4`; `PB6` `MASTER` pot smoothed pre-dither in float32 |
| **Buttons & Keys** | **14 tactile buttons + 27 silicone keys (`F3-G5`)** | Left `OCT-`/`OCT+`, right `2x6` button matrix, 11 labeled black keys + 16 white keys |
| **Keybed Pressure** | **Carbon-pill `SARADC` resistance sensing** | Matrix rows `PA7`/`PA8` (`SARADC` ch 0 & 1, settled single-column drive across `F3..C5` `20` keys with digital fallback) yield continuous `[0..1]` pressure without hardware mods |
| **LEDs** | **42 physical LED channels (27 keys + 14 buttons)** | Multiplexed `2 × 74HC595` (16-pin) scan driven at `909.1 Hz` via a staggered-phase multi-tier (`2 us / 4 us / 95 us`) 1st-order `ΔΣ` modulator (zero BAM glitch or `1–4 kHz` ground whine): 27 white key LEDs, 12 white button LEDs, split **Green `PLAY` (`col 8, row 1`) + Red `STOP` (`col 10, row 2`)**, and **Red `REC`** |
| **Connectivity & Shell** | **`zvenson/dinghy` (`GPL-3.0-only`)**, **USB-C** (MIDI + Stereo Audio + SysEx), **3.5mm TRS MIDI IN** | Bare-metal HAL (`0` JieLi BLE/RTOS binary blobs, `ADR-044`, `ADR-046`) strictly isolated in `platforms/fm1/`: power-on **`Hold OCT-` USB rescue mode**, `Core 1` SRAM offload, `LCD_IDLE` `SPI1` DMA, CRC `blob_*` `USR` flash NVS, `.fwsc` packaging, and `24 MHz` `TIMER4`-timestamped `24 PPQN` MIDI Clock & Transport In across `3.5mm TRS` (`PH8` UART1 DMA) + `USB-C` (WebMIDI BLE-MIDI active in simulator; onboard `AC791N` BLE-MIDI GATT scheduled for `v1.x+`) |

> **Electrical Routing Note for Pedals & Eurorack:** Because the 3.5mm output is a **stereo TRS** headphone/line driver, users should use a **3.5mm TRS stereo breakout** (never a mono TS patch cable directly, which shorts the Right ring to ground). Output is AC-coupled (~1-2 Vpp), ideal for pedals and suited for Eurorack via an input boost/VCA module.

### 1.1 FM-1 1 MB SPI NOR Flash Map & Anti-Brick Safety Rules

On the M-VAVE FM-1, **there is no separate factory USB-MIDI DFU partition outside the application**—USB firmware updates are served by the running firmware itself. Respecting physical flash boundaries and early boot order is mandatory to prevent soft or hard bricks:

| Physical SPI Flash Offset | Size / XIP Mapping | Role & Hardware Safety Rule |
| :--- | :--- | :--- |
| **`0x00000 – 0x03FFF`** | `16 KB` (Boot Head / U-Boot) | **Strictly off-limits.** Stores mask-ROM / U-Boot parameters; overwriting `< 0x4000` causes a hard brick (`WL82 UBOOT1.00`) requiring external RP2040 `D+`/`D-` hardware recovery. |
| **`0x04000 – 0xE9FFF`** | `~920 KB` window (`0x02000120` XIP; **`<= 600 KB` `.fwsc` ceiling**) | **Application XIP region.** Linked exclusively via `dinghy`'s proven `app.ld`. Portable `src/` emits standard `.text`/`.rodata`/`.bss` with zero custom linker section overrides and fails the build if `.fwsc` approaches the ceiling. |
| **`0xEA000 – 0xFBFFF`** | `72 KB` (`USR` NVS Area) | **Persistent `settings_*` & `slot_*` storage.** Accessed exclusively through `dinghy`'s CRC-verified storage APIs on `Core 0`'s main loop—never via raw SPI flash register writes or inside ISRs. |
| **`0xFC000 – 0xFFFFF`** | `16 KB` (High System Tail) | **Reserved SoC / calibration tail.** Never written by application code. |

1. **Unconditional Power-On `Hold OCT-` USB Rescue Priority:** At power-on, `dinghy` checks whether **`OCT-` is physically held** *before* initializing the synth engine, entering a minimal USB-MIDI rescue updater if held. To guarantee synth bugs can never run before that check, portable C++ code enforces **zero global static constructors or exit destructors** (`-Wglobal-constructors -Wexit-time-destructors` under `-Werror`, zero `.init_array` entries); engine instances are constructed in `.bss` RAM via placement `new` strictly *after* `Hold OCT-` rescue has cleared.
2. **Dual Flash JEDEC ID Support (`P25Q80` `0x856014` + `UC25HQ80` `0xB36014`):** Units shipped since Autumn 2026 populate a `UC25HQ80` (`0xB36014`) 1 MiB SPI NOR instead of Puya `P25Q80` (`0x856014`). Both the embedded USB update loader (`usb_hid_ota.bin` / `ldr_flash_known`) and the app NVS driver (`fl_jedec_ram`) MUST accept both JEDEC IDs (enforced automatically by `build_fm1.py`) so newer hardware never refuses USB firmware updates or drops `USR` saves to RAM.
3. **Mandatory `fm1-emu` Gate Before Physical Flashing:** Every `.fwsc` image must verify partition bounds (`>= 0x4000`, `< 600 KB`), clean boot, NVS round-trip, and `Hold OCT-` USB rescue entry inside `joakimeriksson/fm1-emu` before ever being transmitted to a physical FM-1.

---

## 2. Dual-Core Execution Model

Ruido pins real-time DSP and UI/control scanning to separate `pi32v2` cores (`Core 1` executing out of internal `.c1_text` SRAM to prevent XIP Flash sector-erase stalls and cache contention), communicating strictly via **uncached / D-Cache-coherent lock-free triple-buffered snapshots and SPSC ring buffers**:

```text
┌─────────────────────────────────────────────┐       Uncached         ┌─────────────────────────────────────────────┐
│        CORE 0 (UI & I/O — XIP Flash)        │    Lock-Free Bus       │      CORE 1 (Hard Real-Time DSP — SRAM)     │
│                                             │ ─────────────────────> │                                             │
│ • GPIO + Settled PA7/PA8 SARADC Pressure    │                        │ • ALNK0 24 MHz Crystal DMA Master Clock     │
│ • 7x Encoder Quadrature + 3-Gear Rate Scale │    Uncached SPSC +     │ • 44.118 kHz Modulation: 8x Dual Voice LFOs │
│ • 2x 74HC595 42-Ch Gamma/ΔΣ LED Multiplexer │  LFO/Gate Telemetry    │ • 4x Oversampled Domain (176,472 Hz):       │
│ • 240x16 Ping-Pong Scanline Strip Renderer  │ <───────────────────── │   - 4x Schmitt PolyBLEP + Rail Coupling     │
│ • LCD_IDLE SPI1 DMA Transfer (~50-60 FPS)   │                        │   - 8x Nodal ZDF MS-20 (HPF -> LPF)         │
│ • Snapshot Morph & CRC Flash NVS (slot_*)   │                        │ • Bus Sag + FIR + 1-LSB TPDF 16-Bit DAC Out │
└─────────────────────────────────────────────┘                        └─────────────────────────────────────────────┘
```

---

## 3. DSP Pipeline (`src/dsp/`)

All synthesis and filtering run inside a unified **`4x` oversampled domain (`176,472 Hz`)** using single-precision `float` before decimating once to `44,118 Hz`.

### 3.1 Schmitt-Trigger Relaxation Oscillator Model (`4x` Voices, `0.05 Hz - 20,000 Hz`, `5–95%` PWM & `0–100%` `1/f` Drift)
1. **Piecewise-Slope Band-Limited Step Generation (`5–95%` Pulse Width):** 32-bit unsigned integer (`uint32_t`) phase accumulators (`41 uHz` resolution, immune to `float32` sub-audio ULP stall down to `0.05 Hz`, bounded by `kMaxPhaseInc = 0x1F000000u` / `~21.37 kHz` with a Nyquist `minSafeDuty` guard keeping consecutive transitions `> 8` oversampled samples apart) split at `0x80000000u` with asymmetric high/low phase slopes (`incHigh = 0.5*inc / duty`, `incLow = 0.5*inc / (1 - duty)`) for exact fundamental pitch invariance across `5–95%` duty, **4-point Integrated Cubic B-Spline PolyBLEP** anti-aliasing on both rising and falling transitions (`< -109.9 dBc` alias floor `< 20 kHz` on `15%` narrow pulse), and subtle per-inverter CMOS hysteresis duty spread (`~49.2% - 50.9%`) near `50%`.
2. **RC Coupling Cap (`1.5 Hz` Corner):** Leaky-integrator / high-pass roof sag (`~1.5 Hz` corner) preserving `-0.037 dB` seismic `10–30 Hz` sub-bass weight while stripping raw DC.
3. **Variable `1/f` Thermal Drift (`0–100%`):** Per-voice dual-pole (`0.996` fast + `0.9997` slow) filtered random walk normalized to `[-1, +1]` (`EngineTelemetry::voiceDriftNorm[4]`) and scaled via `fastExp2` from **`0%` `LOCKED`** (`0.00 cents` crystal phase-lock) through **`10%` default** (`~±2.2 cents` organic shimmer) up to **`100%`** (`~±120 cents` tape-warble / unstable transistor swarm).
4. **Two-Regime Shared Power-Rail Coupling & Starved-Rail Hard Sync (`ALGORITHM` Knob):** `0–45%` injects smooth `CD40106` crowbar pulses into a virtual `Vdd` sag bus (`25%` default chorus shimmer); `45–100%` enters **starved-CMOS rail collapse** (`[0.12x..4.5x]` phase pull + neighbor gate-load PWM duty warp + band-limited PolyBLEP hard-sync snap in the upper `14%` `Vt+` capture window).

### 3.2 Per-Oscillator MS-20 `Rev 2 -> Korg35` Dual Filter (`8x` Nodal ZDF Stages, `2 Hz - 28 kHz`) & Pre-Filter Overdrive
Each oscillator feeds an asymmetric **ESP-style pre-filter transistor overdrive (`1.0x..4.5x`, `EDIT` page `70–100%`) -> 2-pole Sallen-Key High-Pass Filter (`IC5`) -> 2-pole Sallen-Key Low-Pass Filter (`IC6`)** controlled on the latching **`FX`** page (`SELECT = OSC 1..4`) by `HPF` (`KNOB1`), `HP RES` (`KNOB2`), `LPF` (`KNOB3`), and `LP RES` (`KNOB4`) (`ADR-019`, `ADR-024`, `ADR-028`, `ADR-029`):
- **Capacitive Sallen-Key Nodal Topology & Zero Smart-Park Muting:** `HPF` injects pre-saturated input $x$ through series capacitors `C1` ($v_{C1} = y_1 - x$) and buffered `C2` ($v_{C2} = y_2 - y_1$), delivering exact `0 Hz` capacitive DC rejection (`-156 dB` input DC leak; stripping asymmetric rectifier DC from `4.5x` overdrive). Linear bypass engages strictly when parked at `2.0 Hz` / `28.0 kHz` with `0%` resonance and `1.0x` drive—any resonance (`> 0%`) or overdrive (`> 1.0x`) engages 100% nonlinear silicon across the entire `2 Hz – 28 kHz` continuum. `LPF` senses `Stage 2` differential $w = y_1 - y_2$ into its `1S1588` limiter so simultaneous `HPF + LPF` self-oscillation never suffers Van der Pol bass quenching.
- **Continuous `Rev 2` (`0–60%`) -> `Korg35` (`60–100%`) Morph (`SallenKeyMath.h`):** `0–60%` models creamy `LM13600` OTA + `1S1588` `Rev 2` (`k <= 3.60`); `60–100%` morphs continuously into feral early `Korg35` (`k` up to `6.80`, tighter diode knees, asymmetric Stage-2 BJT/OTA rails, and unpadded/boosted `HPF -> LPF` `interStageScale` for screaming dual-peak intermodulation).
- **Coupled 1D Analytical Newton-Raphson Solver, Exact Nyquist Guard & 4-Voice Lockstep SIMD:** Reducing all 3 nonlinearities per stage to a 1D scalar root with exact Nyquist Jacobian loop-gain bounding (`gHp * (kHp - 1.0) <= 0.88`, `gLp * (kLp - 0.72) <= 0.88`, guaranteeing `df >= 0.12` globally while keeping `k > 2.0` self-oscillation alive past `24 kHz`), calibrated pitch-compensation curves, and bit-exact 4-voice SIMD execution via `Ms20DualFilter::processQuadBank4x` (`sizeof(Ms20DualFilter) = 32 B`).

### 3.3 Dual Per-Voice Analog-Modeled LFO Bank (`8x` Independent Voice LFOs: `LFO A/B OSC 1..4`, `0.01 Hz - 2,000 Hz`)
Each voice (`Osc 1..4`) owns **2 dedicated analog-modeled modulation oscillators (`LFO A` + `LFO B` = `8` total)** inside `VoiceLfoBank` (`[0..3] = LFO A`, `[4..7] = LFO B`, shared zero-`libm` `LfoMath.h`, `ADR-022`, `ADR-028`, `ADR-031`), stepped at `44,118 Hz` with `4x` linear sub-sample pitch & PWM duty interpolation (`prevOscFreqHz_`, `prevOscPw_`) into the `176,472 Hz` oscillator bank:
- **Zero-Delay `LFO B -> LFO A` Evaluation Order, 32-Bit Phase Accumulator & Capacitor Drift:** `VoiceLfoBank::stepFrame` evaluates `LFO B` (`[4..7]`) first so its live instantaneous output modulates `LFO A` (`[0..3]`) with zero 1-sample delay when `LFO B` targets `A RATE` (`±6 oct` across `[0.01 Hz, 2,000 Hz]`) or `A DEPTH` (swelling/fading `LFO A` from stillness up to crest). `uint32_t` phase accumulators (`~10.3 uHz` resolution, `100.0 s` cycles at `0.01 Hz` up to `2,000 Hz` audio-rate FM / PWM / Filter-FM / AM) initialize across all `8` phases in `45°` increments with per-LFO `1%` polystyrene timing-capacitor tolerances and slow thermal random walk.
- **Stepless Analog Waveform Morph (`SHAPE` `0..100%`):** Continuous morph across `C^2` folded polynomial `Sine` + JFET Padé crest warmth (`0.00`), MS-20 variable-symmetry `Triangle -> Sawtooth` (`0.25 -> 0.50`) with `C^1` quadratic vertex fillets and rate-adaptive flyback, vactrol-rounded `C^1` cubic `Slewed Square` (`0.75`), and non-repeating 2-segment **`C^2` Uniform Cubic B-Spline `Random` Drift** (`1.00`) packed into `8 x 30-bit` atomic `lfoDriftPacked` telemetry.
- **Clickless 10-Target Crossfade, Band-Limited PWM & Additive Dual Stacking:** Switching `KNOB 1` (`TARGET`) live crossfades per-destination weights (`~12.5 ms` tau) across **`FREQ`** (`±5 oct` audio-rate FM), **`DUTY`** (`5–95%` band-limited Schmitt PWM), **`LPF`** / **`HPF`** (`±7.5 oct` across `2 Hz – 28 kHz`), **`LP RES`** / **`HP RES`** (`0–100%` into `Korg35`), **`LEVEL`** (`0–100%` into `4.5x` MS-20 overdrive), and **`PAN`** (`L100..R100`), plus **`A RATE`** and **`A DEPTH`** exclusively on `LFO B` (`kNumLfoATargets = 8`, `kNumLfoBTargets = 10`). Pointing both `LFO A` and `LFO B` at the same destination stacks both modulations sequentially/additively.

### 3.4 Mixing, Equal-Power Pan, Master Bus Sag, DAC Safety & Polyphase Decimation
- **Stepless `C^1` Log-Potentiometer Smoothing, Vactrol Mute, Pad Pressure & Pan:** Continuous rotary controls (`freqHz`, `hpCutoffHz`, `lpCutoffHz`, `lfoRateHz`, `lfoBRateHz` in `log2(Hz)` octave space via `LogFreqSmoother`; `pulseWidth`, `drift`, `amplitude`, `pan`, `hpResonance`, `lpResonance`, `lfoShape`, `lfoDepth`, `lfoBShape`, `lfoBDepth`, `circuitCoupling`, `masterTrim` via `CriticallyDampedSmoother`) use 2-pole critically-damped cascades (`14 ms/pole`, `~28 ms` mean S-curve glide, `0%` overshoot); White Keys `5-8` drive uncapped MS-20 filter crossover/resonance or muted Buchla-style Vactrol LPG swells via `VactrolSmoother` (`4.5 ms` attack / `26.0 ms` release, `0.000 Hz` pitch shift).
- **Master Bus Sag & Post-Decimator DAC Seatbelt:** High-headroom (`0.75` linear knee, `±1.25` soft ceiling) summing bus followed by two cascaded 2x **Linear-Phase Symmetric Polyphase Half-Band FIR** stages (`> 87 dB` stopband rejection to `20 kHz`, `0°` phase dispersion) and a transparent `dacSafetyClip` guard (`100%` bit-exact linear for `|x| <= 1.15f`, smooth `±1.45f` ceiling catching FIR Gibbs overshoot on `21–28 kHz` ultrasonic edges).

### 3.5 Verified Target Cycle Budget (`fm1-emu` Dual-Core @ 240-320 MHz, `ADR-045`, `ADR-046`)
Available cycles per `44.118 kHz` stereo frame: **`5,442` cycles (`240 MHz`) – `7,256` cycles (`320 MHz`)**.
- **`Core 0` `ALNK0` `256`-Frame DMA Drain ISR:** **`~8.8` insns/frame** (`0.16%` @ `240 MHz`) draining the 16-block (`1,024`-frame) audio ring (`0` underruns across timed `SPI1` DMA + panel/MIDI stress)
- **Dual-Core Idle (`4x` PolyBLEP + `1/f` drift + rail coupling + parked ZDF + FIR + TPDF):** **`~4,255` insns/frame** (`78.2%` @ `240 MHz`, **`58.6%` @ `320 MHz`**)
- **Dual-Core Full Stress (`4` voices simultaneous `HPF+LPF` Korg35 ZDF self-oscillation + overdrive + `8x` LFOs, cost-balanced `1+3` / `2+2` Dekker work-stealing filter co-processor):** **`~6,666` insns/frame** (**`91.9%` @ `320 MHz`**; `~30x` real-time single-core host throughput, `0.0e+00` float32 dual-core parity)

---

## 4. Keybed `SARADC` Pressure & Scanline Strip UI (`src/ui/`)

### 4.1 Carbon-Pill Polyphonic Pressure Scanner (`Core 0`)
- Between digital matrix scans, Core 0 pulses weak GPIO pull-ups and reads the `AC791N` `SARADC` channels tied to the silicone key matrix rows, mapping nonlinear carbon-pill resistance drop through a calibrated curve + low-pass slew into continuous `[0.0f .. 1.0f]` per-key pressure values (mirrored in `platforms/sim/` via mouse drag/wheel pressure + MIDI Poly/Channel Aftertouch).

### 4.2 `240x16` Coupled 4-Voice Caustic Ribbon Scope & Modular Optical-Filament HUD Compositor
- **Ping-Pong Strip Buffers & Compact Zero-Framebuffer State:** Two `240 x 16` pixel RGB565 buffers (`7,680 bytes` each = **`15.36 KB`**, shared with hardware lifecycle screens) plus `121.27 KB` (`124,184 B`) static `ScopeHudRenderer` state (`4096`-sample compact `12 B` `Q3.12` stereo + 4-voice/rising-edge scope history covering down to `20 Hz` and multi-cycle super-periods, continuous phosphor persistence states, `14` `Q8.6` strands with `8`-column `[minY, maxY]` tile-culling bounds, `14 x 240` `6 B` per-column fixed-point `StrandColCache` segment geometry eliminating `30x` per-slice float math, `4 x 3 x 48` live-audio `OverdriveHairTable` persistence traces, a `60x60` `uint16_t` quarter-resolution bloom grid, full-screen slewed `4 x 241` + `2 x 241` `Q8.6` MS-20 filter curves (`x = 0..239`, `y = 26..186`), dedicated full-screen `4 x 241` `Q8.6` LFO curves + `241`-column sibling companion curve, and synchronization-unified `150 ms` center-view fade envelopes across `DUTY`, `DRIFT`, `LEVEL`, `PAN`, `FX`, `LFO`, `GLOBAL`, and `CONFIG`), avoiding any `115.2 KB` `240x240` framebuffer while streaming at ~25–29 FPS (`~30.5 FPS` `30 MHz` `SPI1` wire ceiling) over non-blocking SPI DMA.
- **Coupled 14-Strand Circuit Mesh, Super-Period Lock & Zero-Hard-Cut Galvanometer Relaxation (`ADR-018`, `ADR-023`):** Core 1 delays 4x-averaged `int16` (`Q2.13`, `0.0095 px` precision) per-voice post-MS-20 taps and 4-voice rising-edge flags by `36` base frames to align sample-accurately with the linear-phase FIR decimated stereo bus. On Core 0, `ScopeTraceEngine::prepare` scans up to `8` rising-edge candidates across `4096` samples (`~93 ms`), refines each to its continuous sub-sample post-MS-20 zero-crossing (immune to filter group delay), locks onto the multi-cycle harmonic super-period via a `48`-tap per-voice similarity signature across a continuous Catmull-Rom `smoothStride_` (`[1.0f, 2.0f]`, eliminating `1x <-> 2x` bass-zoom snaps), extracts 3-layer live-audio overdrive hair traces (`updateOverdriveHair`), applies continuous `38–190 ms` phosphor/galvanometer slewing (smoothly relaxing all `14` strands into the `y = 120` silent horizon when `PLAY/STOP` stops, `MASTER = 0%`, all voices mute, or the audio stream pauses, and smoothly expanding back on resume), caches per-column `StrandColCache` fixed-point geometry once per frame, splats analytical column photon energy into the `60x60` grid, and runs a 2-pass in-place separable `[1, 3, 4, 3, 1] / 12` blur calibrated at `25%` bloom intensity.
- **Tile-Culled Linear Photon Rasterizer, Unified 1-to-1 Center Graphs & Single-Pass Target-Endian `4x4` Bayer Neutral RGB565 (`ADR-020`, `ADR-022`, `ADR-023`, `ADR-028`, `ADR-031`, `ADR-032`, `ADR-033`, `ADR-036`, `ADR-043`):** `ScopeHudRenderer::renderStrip` processes stateless `<= 8`-row sub-chunks (`5.76 KB` stack), accumulates `64`-entry LUT Gaussian strand capsules in linear photon space (`uint16_t`) using integer fixed-point distance math (`0` float reciprocals/invSqrts per slice), adds exact eighth-step integer bilinearly upsampled `60x60` bloom, maps through a `1024`-entry `constexpr` soft-knee Reinhard + `0.82` gamma LUT (`1.12 KB` XIP Flash), applies a centralized `150 ms` S-curve background waveform crossfade envelope (`HudEnvelope.h`) behind active center views (`HudAmpGraph` `DUTY`/`DRIFT`/`LEVEL`, `MS-20 Filter`, `LFO`, `PAN` Stereo Panorama Curve, `SEL` Keybed Map, and `GLOBAL CONFIG` — keeping `FREQUENCY` and pitch/unison/rhythm macros `100%` undimmed), composites `HudAmpGraph` (including Option `3A` narrow triangular live-audio overdrive cap `> 70%`) + `HudGraphs` + `HudGlobalViews` + `HudKeybedView` + `HudChrome` (`10 / 13 pt` Medium + `13 pt` Bold `HelveticaNeue` ROM tables, `7.70 KB` XIP Flash, `0 B` SRAM, plus `150 ms` `FINE`/`COARSE` status badge transitions and 4-voice filter spread micro-beads), and packs strictly neutral monochrome RGB565 (`R5 == B5`, `G6 == (R5 << 1) | (R5 >> 4)`) with `4x4` ordered Bayer dithering (`~496` effective monochrome shades, zero contour banding) directly in target byte order (big-endian for `ST7789` `SPI1` DMA on FM-1, little-endian on host/Wasm).

---

## 5. Modular Codebase Hygiene & Cross-Platform Portability (`src/` <-> `platforms/`)

All handwritten files in `src/` and `platforms/` are strictly capped at **`<= 700` lines**, organized as a one-way subsystem DAG (`src/core` <- `src/dsp` and `src/core` <- `src/ui`, zero cross-includes between `dsp` and `ui`), and verified automatically on every `make test` via `tools/check_hygiene.py`:
- **`src/core/` (`sizeof(RuidoController) <= 2 KB`):** Controller orchestration (`FX`, 2-page `EDIT`, 8-slot `LFO A/B OSC 1..4`, `SEL` 3-model keyboard selector, and `GLOBAL CONFIG` pages), 3-model `KeyboardEngine` (`LIVE MIXER`, `CIRCUIT LAB`, `RHYTHM LAB`) + unified per-oscillator circuit audibility/mute truth, transport-agnostic `24 PPQN` `MidiClockTracker` (24-interval sliding window + 1-pole PLL immune to `7.5–15 ms` Bluetooth BLE packet clumping + logarithmic `20–3,000 BPM` internal/tap tempo), 4-slot `128-byte` CRC-32 `SnapshotBank` (log-octave glide, multi-slot pressure blend, and bit-exact momentary morph), 3-gear linear quadrature encoder math, zero-`libm` LFO/filter/rhythm helpers, and 42-channel varibright LED engine.
- **`src/ui/` (`sizeof(ScopeHudRenderer) <= 124 KB`):** `ScopeHudRenderer` (chunk pipeline & single-pass target-endian tonemap), `ScopeTraceEngine` + `ScopeTraceRender` (trigger lock, per-column `StrandColCache`, `60x60` bloom & `OverdriveHairTable`), `HudAmpGraph` (`EDIT` 4-column `DUTY` / `DRIFT` / `LEVEL`, `DRIVE` punch-in & Option `3A` narrow hairy CRT overdrive cap), `HudGraphs` (full-screen `FX` / keybed `GLO` `GlobalFilter` MS-20 passbands & dual-slot `LFO` + companion sibling plots), `HudGlobalViews` (`EDIT` Page 2 Edge-to-Edge Stereo Panorama Curve + divider-free `GLOBAL CONFIG` list), `HudKeybedView` (`SEL` SDF keybed diagram with `5 s` `50%` slow dim & `SELECT` wake), `HudHeader` + `HudChrome` + `HudColumnText` + `HudFormat` (divider-free top/bottom chrome, steady 1-to-1 `OSC 1–4` audibility dimming, `FINE`/`COARSE` status badge, 4-voice filter micro-beads, zero-`snprintf` formatters).

```text
          ┌───────────────────────────────────────────┐
          │     Portable Modular Engine (src/)        │
          │  • src/dsp/  DroneEngine (47.88 KB)       │
          │  • src/ui/   ScopeHudRenderer (121.27 KB) │
          │  • src/core/ RuidoController (1.98 KB)    │
          └────────────────────┬──────────────────────┘
                               │  Pure C++17 Structs (Zero OS/SDK Headers)
             ┌─────────────────┴─────────────────┐
             ▼                                   ▼
┌──────────────────────────┐       ┌──────────────────────────┐
│     platforms/sim/       │       │     platforms/fm1/       │
│ • WebAudio / CoreAudio   │       │ • dinghy + felucca-bsp   │
│ • Exact FM-1 Panel UI    │       │ • fm1_cpu1 Core 1 SRAM   │
│   (7 Encoders + Master + │       │ • ALNK0 + TPDF + USB Aud │
│    27 Pressure Keys +    │       │ • LCD_IDLE DMA + ΔΣ LEDs │
│    14 Buttons + 240x240) │       │ • Velo SARADC + .fwsc    │
└──────────────────────────┘       └──────────────────────────┘
```

Every DSP sample and every `240x240` RGB565 pixel produced in the simulator is **bit-for-bit / pixel-for-pixel identical** to the hardware code path.
