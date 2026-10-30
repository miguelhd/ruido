# Ruido — Product & UX Specification

## 1. Vision & Musical Identity

**Ruido** transforms the pocket **M-VAVE FM-1** hardware into a dedicated, boutique **4-oscillator CMOS Schmitt-trigger stereo drone instrument**.

- **Primary Acoustic Reference:** Classic DIY analog Schmitt-trigger relaxation oscillator synths (`CD40106` / `74HC14` style)—organic capacitor tilt, micro-thermal drift, shared power-rail phase pulling, and starved-rail hard-sync tearing—paired with feral **Korg MS-20 `Rev 2 -> Korg35` dual `HPF -> LPF` silicon-limited resonant filters** and **ESP-style asymmetric pre-filter transistor overdrive**.
- **Primary Studio Role:** A pristine-to-feral, harmonically rich, zero-aliasing stereo sound source with **zero artificial musical clamps**, designed specifically to feed **external guitar/synth effects pedals** (fuzz, wavefolders, BBD delays, reverbs, granular) and **Eurorack modular rigs**.

### Core Design Values (In Priority Order)
1. **Great Sound & Unclamped Extremes:** Spend every spare CPU cycle on analog-faithful oscillator physics, `4x` oversampling, nonlinear Zero-Delay Feedback filtering, and clickless slewing—imposing safety limits strictly for numerical/hardware protection (anti-aliasing, zero `NaN`/DC runaway, DAC rail safety), never to domesticate musical range.
2. **Immediate, Ergonomic UX with Judicious Pages:** Default idle state is always live waveform + `KNOB1-4 = Osc 1-4 Frequency`, with momentary holds strictly on the left/bottom (`OP6`, `GLO`, `OCT-/OCT+`) so forearms never cross over the display, and latching toggles / judicious single-level pages on the right-side `2x6` button grid (`FX` MS-20 Filters, `EDIT` Channel Level/Drive, `LFO` modulation, and `GLO` `GLOBAL CONFIG`, always one tap from `HOME`).
3. **Useful Waveform Visuals:** An always-on oscilloscope that serves as an actual acoustic tuning and gain-staging instrument, with minimal controls gently fading in over the wave only while touched or viewing a focused page.

---

## 2. Voice & Signal Architecture (User-Facing)

Ruido consists of **4 identical mono voices** (`Osc 1..4`) mixed in stereo through a subtle nonlinear master bus:

```text
[Schmitt Square Osc 1..4] ──> [Pre-Filter Drive 1.0x..4.5x] ──> [MS-20 Dual HPF -> LPF 1..4] ──> (VCA + Gate + Pan) ──> [Master Sag] ──> L/R Out
          ▲                                                                                                                  │
          └────────────────────── Shared Power-Rail Coupling & Starved-Rail Sync (`ALGORITHM`: 0-100%) ──────────────────────┘
```

### Per-Oscillator Controls (`Osc 1..4`)
- **Frequency:** Unbroken continuous stepless sweep from **Sub-Cycle Pulses to Ultrasonic Edge (`0.05 Hz - 20,000 Hz`)** (`1.5 Hz` RC coupling corner preserving `-0.04 dB` seismic `10–30 Hz` sub-bass weight), using **strictly linear 1:1 X4 quadrature scaling (`96 steps/turn`, zero velocity acceleration)** + **`C^1` critically-damped log-octave smoothing**:
  - *`FINE` (`Hold OCT-`):* Surgical `~0.66 cents/step` (`~63 cents/turn`) precision for locking slow phase beating and harmonic intervals.
  - *`STANDARD` (Released Default):* Predictable linear `0.0387 oct/step` (`~3.7 oct/turn`) sweep regardless of hand speed.
  - *`COARSE` (`Hold OCT+`):* Rapid end-to-end `0.05 Hz - 20,000 Hz` (`18.6`-octave) sweep in **`~2.0` turns** (`0.096 oct/step`).
- **Dual MS-20 `Rev 2 -> Korg35` Filter (`HPF -> LPF` in series across `2.0 Hz - 28,000 Hz`, Zero Smart-Park Muting):**
  - **HP Cutoff (`KNOB1`):** `2.0 Hz` (`0%` RES parked flat/transparent) up to `28,000 Hz`, engaging warm `LM13600` OTA saturation across all 4 filter poles even at `0%` resonance.
  - **HP Resonance (`KNOB2`):** Unmuted full-range High-Pass Peak (`0-100%`)—`0-60%` models creamy **`Rev 2`** (`LM13600` OTA + `1S1588`, `k <= 3.60`), while `60-100%` morphs continuously into feral early **`Korg35`** (`k` up to `6.80`, tighter diode knees, asymmetric Stage-2 BJT/OTA rails), self-oscillating all the way down to `2 Hz` infrasonic throb with exact `0 Hz` capacitive DC rejection.
  - **LP Cutoff (`KNOB3`):** `2.0 Hz` up to `28,000 Hz` (`0%` RES parked flat/wide-open), rounding square edges with creamy `LM13600` OTA 2nd/3rd harmonic warmth and extending self-oscillation past `20 kHz`.
  - **LP Resonance (`KNOB4`):** Unmuted full-range Low-Pass Peak (`0-100%`, `Rev 2` `0-60%` -> `Korg35` `60-100%` up to `k = 6.80`), unpadding `HPF -> LPF` inter-stage gain above `60%` so dual-resonant formants slam `IC6` into raw intermodulation screaming without bass-quenching.
- **Channel Level & MS-20 Pre-Filter Overdrive (`EDIT` Page, `0-100%`):** Two-zone analog console/ESP gain law (`computeVoiceChannelGain`): `0-70%` (`kCleanAmpKnee`) = clean post-filter VCA attenuation (`1.0x` pre-drive; `65%` default); **`70-100%` = `1.0x -> 4.5x` asymmetric pre-filter MS-20 transistor overdrive** slamming `HPF -> LPF` into heavy 2nd/3rd harmonic saturation + `+4.1 dB` post-filter trim.
- **Panning:** Equal-power (`sin`/`cos`) stereo placement from hard Left (`L100`) through Center (`C`) to hard Right (`R100`), or collapsed to Dual Mono (`L == R`) when `MONO` output routing is active.

---

## 3. Physical Front-Panel Map & Two-Hand Ergonomics

Reference faceplate photo: [`docs/assets/fm1-front-panel.png`](assets/fm1-front-panel.png).

```text
       LEFT HAND ZONE                  CENTER                 RIGHT HAND ZONE
 (Holds & Global Circuit)         (100% Unobstructed)      (Turns Knobs & Single-Taps)
┌───────────────────────────┐    ┌──────────────────┐    ┌────────────────────────┐
│ MASTER          SELECT    │    │                  │    │ KNOB1 KNOB2 KNOB3 KNOB4│
│ PRESETS         ALGORITHM │    │   240x240 TFT    │    │ [FX] [SEL][ENV][LFO]...│
│ [OCT-] [OCT+]             │    │      SCREEN      │    │ [HOME][SAVE]...[PLAY]  │
│                           │    │                  │    │  (Single-tap only—     │
│ [OP1][OP2][OP3] [OP4][OP5]│    │   [OP6][PIT][GLO]│    │   NEVER left-hand hold)│
│ [White Keys 1-8]          │    └──────────────────┘    │ [MONO][POLY] [Keys 9-16│
└───────────────────────────┘                            └────────────────────────┘
```

### Ergonomic Rules
1. **Strict Left-Hand Momentary Holds:** Every momentary `"Hold + Turn KNOB1-4"` gesture lives in the **bottom-center black keys** (`OP6`, `GLO`) and left-hand **`OCT-` / `OCT+`**. The left hand stays down-left, the right hand stays top-right on `KNOB1-4`, and the center `240x240` screen is never occluded by a crossed forearm.
2. **Right-Side `2x6` Button Grid = Non-Momentary Toggles & Judicious Pages:** None of the `2x6` buttons directly below `KNOB1-4` are momentary holds; they act as latching toggles, immediate utilities, or standardized single-level page switches (`FX` MS-20 Filters, `EDIT` Channel Level/Drive, `LFO` modulation, and `GLO` `GLOBAL CONFIG`).

### 3.1 Top-Right Encoders: `KNOB1` – `KNOB4` (Idle, Momentary Holds & 5-Mode `GLOBAL`)

*(Focused right-grid pages `FX`, `EDIT`, and `LFO` also remap `KNOB1-4` while latched open; see §3.4.)*

| Left-Hand Hold (Black Key) | `KNOB1` | `KNOB2` | `KNOB3` | `KNOB4` | Monochrome HUD Header + Center Graph + Bottom 4-Knob Strip |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **None (Idle Default)** | **Osc 1 Freq** | **Osc 2 Freq** | **Osc 3 Freq** | **Osc 4 Freq** | `FREQUENCY OSC 1..4` (`XX.XX Hz` exact counter) + 4 `Hz` filaments |
| **Hold `OP6`** *(Lit `25%`)* | **Osc 1 Pan** | **Osc 2 Pan** | **Osc 3 Pan** | **Osc 4 Pan** | `PANNING` (`STEREO` / `DUAL MONO`) + 4 bipolar `L..C..R` filaments |
| **Hold `GLO` (`SELECT = FREQ`)** *(Lit `25%`)* | **Pitch** | **Spread** | **Detune** | **Tilt** | `GLOBAL FREQ` + `0.05 Hz–20 kHz` log-frequency 4-voice spectral stems & beads + 4-voice spread micro-beads |
| **Hold `GLO` (`SELECT = FILTER`)** *(Lit `25%`)* | **All HPF** | **All HP Res** | **All LPF** | **All LP Res** | `GLOBAL FILTER` + 4-voice overlaid `2 Hz–28 kHz` MS-20 passbands & beads + relative delta filaments |
| **Hold `GLO` (`SELECT = LFO`)** *(Lit `25%`)* | **All Target** | **All Shape** | **All Rate** | **All Depth** | `GLOBAL LFO` + full-screen 4-voice morphed LFO waves & 4 phase beacons (`MIXED` when diverged) |
| **Hold `GLO` (`SELECT = AMP`)** *(Lit `25%`)* | **Level** | **Tilt** | **Width** | **Balance** | `GLOBAL AMP` + `L..C..R` stereo soundstage horizon & 4 voice pillars |
| **Hold `GLO` (`SELECT = ALL`)** *(Lit `25%`, Default)* | **Freq** | **Filter** | **LFO** | **Amp** | `GLOBAL ALL` + unified `2 Hz–28 kHz` MS-20 passband overlaid with the 4 live oscillator pitch/level stems |

### 3.2 Top-Left Knobs & `OCT-` / `OCT+` (Zero-Modifier Global Controls)
- **`MASTER` (Analog Pot):** Hardware master output level (`MASTER LEVEL` + `%` bar on HUD, anchored with `> 0.2%` cumulative deadband so slow analog sweeps wake the HUD reliably without ADC chatter).
- **`ALGORITHM` (Encoder):** **Rail Coupling & Starved-Rail Chaos** (`0-45%` smooth `CD40106` shared-rail threshold pulling around the `25%` default <-> `45-100%` starved-CMOS voltage collapse: `[0.12x..4.5x]` phase pull, neighbor gate-load PWM duty warp, and band-limited PolyBLEP subharmonic hard-sync locking). *Mnemonic: defines how the 4 oscillators electrically couple, analogous to FM operator algorithms.*
- **`SELECT` (Encoder):** Context-sensitive left-hand encoder:
  - While holding keybed **`GLO`**: steps **`GLOBAL`** (`1` item/detent) across **`FREQ` · `FILTER` · `LFO` · `AMP` · `ALL`** (defaults to `ALL` on boot and remembers your last selected destination).
  - When the **`EDIT` Channel Level & Drive Page** is open: acts as a continuous **4-voice linked gang fader** (respecting `FINE` / `STANDARD` / `COARSE` gears), scaling all unmuted voices' Level/Overdrive together while preserving relative voice offsets and mute states.
  - When the **Per-Voice MS-20 Dual Filter Page** (`FX`) or **Per-Voice LFO Page** (`LFO`) is open: steps (`1` item/detent) strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**.
  - When **`GLOBAL CONFIG`** (right-grid `GLO`) is open: moves (`1` row/detent) the row cursor up/down the configuration list.
  - Outside those contexts: displays `SELECT ENCODER · UNASSIGNED` (`1.5 s` toast).
- **`PRESETS` (Encoder):** Reserved for **Snapshot Morph (Continuous Macro Scrub)** in Milestone 3 (currently displays `PRESETS ENCODER · UNASSIGNED` `1.5 s` toast until the 8-slot Snapshot Engine lands; once active, smoothly scrubs/crossfades across saved snapshots while panning varibright LED brightness across **`White Keys 9–16`** per `ADR-025`).
- **`OCT-` / `OCT+` (Momentary Rate Modifiers — Top-Right Status Badge & `150 ms` Transition):** **Strictly momentary left-hand encoder rate modifiers** governing continuous endless rotary sweeps with **zero velocity acceleration** (100% deterministic 1:1 angle-to-parameter response at `96` X4 quadrature transitions per `360°` turn), freely chordable alongside `OP6`, `GLO`, and any open right-grid page (discrete notched selectors like `SELECT` on `FX`/`LFO`/`GLO`, `LFO TARGET`, and `GLOBAL CONFIG` rows remain `1` item per detent across all 3 gears):
  - *Released (Idle Default = `STANDARD`):* Both `OCT-` and `OCT+` rest at `15%` dim glow; full continuous parameter sweeps take an exact linear **`2.5-2.8` turns**.
  - *Hold `OCT-` -> **`FINE`** (`OCT-` LED `100%`):* Surgical fine-tuning while held (`0.66 cents/step` audio frequency, `3.0 cents/step` filter cutoff, `0.09%/step` linear controls).
  - *Hold `OCT+` -> **`COARSE`** (`OCT+` LED `100%`):* Rapid performance sweep while held—covering the **entire `0-100%` / `0.05 Hz-20 kHz` / `2 Hz-28 kHz` sweep in `~1.3-2.0` turns**.
  - *Anchored Left Title + Top-Right `FINE` / `COARSE` Transition:* Holding `OCT-` or `OCT+` **never moves or replaces the active page/tool title on the top-left** (`FREQUENCY` when held from idle). Instead, the **top-right status slot** smoothly crossfades over **`150 ms`** into **`FINE`** or **`COARSE`** (with zero underline or divider rules)—preserving live precision values alongside the badge (`FINE · 110.04 Hz` on `FREQUENCY`, `FINE · 25%` on `RAIL COUPLING`)—and smoothly dissolves back on release.
  - *Release:* Immediately returns to `STANDARD` rate (`15%` rest glow); if a knob was turned during the hold, its remaining `1.5 s` readout toast completes before dissolving in `150 ms` (or fades out immediately in `150 ms` if `OCT-/+` was tapped alone).

### 3.3 Right-Panel `2x6` Function Buttons (Latching Toggles & Focused Pages)
- **`PLAY/STOP`:** **Audio Output** toggle (`ON` / `OFF`, clickless fade to silence and back for safe cable patching). Drives the hardware's dual-channel `PLAY/STOP` LEDs by action affordance: steady **Green** (`PLAY` LED) when quiet (`OFF`), steady **Red** (`STOP` LED) when sound is `ON`, crossfading smoothly between them.
- **`HOME`:** **Exit Page / Clear HUD to Pure Scope** (tap dismisses any active page or toast with the unified **`150 ms`** S-curve dissolve; LED mirrors live HUD opacity `0%..100%`).
- **`SAVE`:** Tap `SAVE` (`STORE SNAPSHOT` / `TAP KEY 9-16`, `100%` LED) then tap a Snapshot white key (`9-16`) to store the current patch; tapping `SAVE` a second time disarms and displays `STORE SNAPSHOT · CANCELLED` (`1.5 s` toast -> `150 ms` fade-out).
- **`FX`:** Opens/closes the **Per-Voice MS-20 Dual Filter Page** (see §3.4 below); LED glows at **`15%` rest** when all active voices' filters are parked flat (`HPF = 2.0 Hz`, `LPF = 28 kHz`, `0%` resonance), scales dynamically (**`15%–85%`**) whenever any unmuted or pressure-active voice has filter cutoff or resonance engaged (including live Filter LFO or Pad Pressure sweeps), and lights to **`100%`** while the page is open.
- **`EDIT`:** Opens/closes the **Channel Level & MS-20 Pre-Filter Overdrive Page** (see §3.4 below); LED glows at **`15%` rest** when all active voices are in the `0–70%` clean VCA region, flares dynamically (**`25%–85%`**) in proportion to live voice signal peaks whenever any active voice is driven into `70–100%` (`1.0x..4.5x`) MS-20 overdrive, and lights to **`100%`** while the page is open.
- **`LFO`:** Opens/closes the **Per-Voice LFO Page** (see §3.4 below); LED glows at **`15%` rest** when all 4 LFOs are at `0%` depth (confirming an assigned page switch), breathes continuously (`15%–60%`) with active LFO modulation when closed, and lights to **`100%`** while the page is open.
- **`GLO` (Right Grid under `KNOB 1–4`):** Opens/closes the scalable **`GLOBAL CONFIG`** menu (`15%` steady rest LED, `100%` while open; see §3.4 below).
- **Unassigned / Reserved Switches (`SEL`, `ENV`, `ARP`, `SEQ`, `REC`):** LEDs stay **`0%` completely unlit** at rest while unassigned (`REC` is physically a single-color **Red** LED on the FM-1 PCB); pressing any unassigned switch closes open pages and displays its silkscreen name + `UNASSIGNED` in the HUD header so every control confirms hardware contact.

### 3.4 Focused Right-Grid Pages (`FX`, `EDIT`, `LFO`, `GLOBAL CONFIG`)

#### Per-Voice MS-20 Dual Filter Page (`FX` — `ADR-029`)
- **Enter / Exit:** Tap **`FX`** (top-left switch of the `2x6` grid below `KNOB 1`) to open/close the latching MS-20 Dual Filter page (`FILTER OSC 1..4` with `12 dB HP -> LP` right header); mutually exclusive with `EDIT`, `LFO`, `GLOBAL CONFIG`, and `SAVE`, and dismissed in `150 ms` by a second `FX` tap or **`HOME`** (momentary holds `OP6`, `GLO`, `ScopeFreezeKey`, and `White Keys 5–16` temporarily override and return seamlessly on release).
- **`SELECT` Encoder (Left Hand):** Steps (`1` item/detent, clamped `0..3` with zero windup) strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**, remembering the last focused voice across page re-opens.
- **`KNOB 1–4` (`HPF | HP RES | LPF | LP RES`):** Adjust High-Pass Cutoff (`2 Hz–28 kHz`), High-Pass Resonance (`0–100%`), Low-Pass Cutoff (`2 Hz–28 kHz`), and Low-Pass Resonance (`0–100%`) for the selected voice (while multi-voice relative filter sweeps remain immediately available on **`Hold GLO` -> `GLOBAL FILTER`**).
- **Live MS-20 Passband Visualization (`HudGraphs`):** Renders the `80 px` (`y = 102..182`, `x = 10..230`, `2 Hz–28 kHz`) dual-peak MS-20 `Rev 2 -> Korg35` transfer curve for the focused voice over a `72%`-dimmed live waveform backdrop, reflecting live Filter LFO and Pad Pressure modulation in real time.

#### Channel Level & MS-20 Pre-Filter Overdrive Page (`EDIT` — `ADR-028`)
- **Enter / Exit:** Tap **`EDIT`** (right-grid switch below `KNOB 1–4`) to open/close the latching `LEVEL` & Drive page (`LEVEL ALL` or `LEVEL OSC 1..4`, with right status `CLEAN VCA`, `DRIVE 1.1x..4.5x`, or `MUTED`); mutually exclusive with `FX`, `LFO`, `GLOBAL CONFIG`, and `SAVE`, and dismissed in `150 ms` by a second `EDIT` tap or **`HOME`** (momentary holds `OP6`, `GLO` temporarily override and return seamlessly on release).
- **`KNOB 1–4` (`OSC 1..4` Level / Drive) & `SELECT` (4-Voice Gang Fader):** `KNOB 1–4` sweep `Osc 1–4` from `0–70%` clean post-filter VCA (`0%` if muted; turning CW fades in smoothly from `0%` and clears mute) into **`70–100%` (`1.0x -> 4.5x`) MS-20 pre-filter input overdrive**; **`SELECT`** scales all unmuted voices' Level/Drive together as a linked gang fader.
- **Live Full-Screen Channel & Saturation Visualization (`HudAmpGraph`):** Spans `x = 0..239, y = 26..188` over a `72%`-dimmed background scope, aligning 4 vertical optical columns (`cx = 30, 90, 150, 210`) directly above `KNOB 1–4` with a `0%` floor (`y = 180`), a continuous **`70%` (`1.0x`) Overdrive Knee** horizon (`y = 79.2`), clean VCA stems + optical beads, live slewed `voicePeak` plasma, symmetrical linear-photon saturation wings + `"1.1x..4.5x"` micro-callouts above the knee, and live AM LFO coronas.

#### Per-Voice LFO Page (`LFO OSC 1..4` — `ADR-022`, `ADR-027`, `ADR-028`)
Each of the 4 voices (`Osc 1..4`) owns **1 dedicated LFO** matching its dedicated MS-20 filter, VCA/overdrive, and panner (multi-voice LFO macro control lives on Keybed `GLO` -> `GLOBAL LFO`):
- **Enter / Exit:** Tap **`LFO`** (right-grid switch) to open the LFO Page (`LFO OSC 1..4` showing `ACTIVE` / `BYPASSED`); tap **`LFO`** again or **`HOME`** to return to the pure scope in `150 ms` while active LFOs continue running.
- **`SELECT` Encoder (Left Hand):** Steps strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**.
- **`KNOB 1–4` (`TARGET | SHAPE | RATE | DEPTH` — Setup Left, Perform Right):**
  - **`KNOB 1` (`TARGET`):** Stepped per-voice parameter selector (`1` item/detent): **`PITCH`** · **`LPF`** *(default)* · **`LP RES`** · **`HPF`** · **`HP RES`** · **`AMP`** · **`PAN`**.
  - **`KNOB 2` (`SHAPE`):** Continuous `0–100%` seamless wave morph across **`Sine` (`0%`) -> `Triangle` (`25%`) -> `Sawtooth` (`50%`) -> `Slewed Square` (`75%`) -> `Smooth Drift` (`100%`, non-repeating organic wander)**.
  - **`KNOB 3` (`RATE`):** Unclamped continuous **`0.01 Hz` (`100 s` glacial cycle) to `2,000 Hz` audio-rate FM / Filter-FM / AM** log-octave sweep.
  - **`KNOB 4` (`DEPTH`):** Unipolar `0–100%` modulation amount (`±5 oct` pitch FM, `±7.5 oct` filter cutoff, `0–100%` resonance, `0–100%` AM into pre-filter overdrive) with a hard stop at `0%` CCW so any LFO zeros cleanly by feel.
- **Live Full-Screen LFO Visualization:** Spans edge-to-edge across the entire `240 px` display (`x = 0..239`) and fills the full vertical viewport (`y = 26..188`, `±75 px` / `150 px` peak-to-peak centered on a `y = 107` zero-horizon filament) over a `72%`-dimmed background scope, with clean gradient shading and a high-radiance `#FFFFFF` optical Phase Beacon.

#### Scalable `GLOBAL CONFIG` Menu (Right-Grid `GLO` — `ADR-027`, `ADR-029`)
- **Enter / Exit:** Tap right-grid **`GLO`** (`ButtonId::GloRight`) to open/close **`GLOBAL CONFIG`** (`1/N` row counter in top-right; divider-free row list with subtle selected-row tint, zero row-separator lines or left edge bars, and bottom 4-knob strip cleanly hidden over a `72%`-dimmed live scope; `HOME` or 2nd `GLO` tap dissolves in `150 ms`).
- **Navigation & Editing:** Turn **`SELECT`** (`1` row/detent) to move the highlighted row cursor across the vertical `5`-row viewport; turn **any `KNOB 1–4`** (`1` option/detent) to edit the highlighted row's value:
  1. **`GLO RELEASE`:** **`LATCH`** *(default — `GLOBAL` multi-voice edits persist when releasing keybed `GLO`)* / **`SNAP BACK`** *(Elektron `CTRL-ALL`-style temporary performance warp: captures `4 x VoiceParams` on keybed `GLO` press and smoothly restores them via `C^1` parameter slews on release)*.
  2. **`OUTPUT ROUTING`:** **`STEREO`** / **`DUAL MONO`** (synced bidirectionally with `GlobalParams::monoCollapse` and the physical `MONO` / `POLY` black keys).

### 3.5 Bottom 27-Key Surface & Carbon-Pill Pressure Expression (`SARADC`)

Using analog `SARADC` sampling of the silicone keybed's carbon-pill contact resistance, Ruido supports continuous **Soma Lyra-8-style finger-lean pressure expression** alongside tactile switching, and **every single key displays live technical feedback on the HUD**:

#### Top Row — 11 Raised Black Keys (Silkscreen-Matched Modifiers & Output Routing)
- **`OP6` (`PANNING`):** **Momentary Per-Voice Spatial Modifier** (`25%` landmark rest glow, `100%` while held) mapping `KNOB 1–4` to `Osc 1–4 Panning`.
- **`GLO` (`GLOBAL` + `SELECT`):** **5-Mode Multi-Voice Performance Modifier** (`25%` landmark rest glow, `100%` while held; releasing either latches or snaps back per `GLO RELEASE` in `GLOBAL CONFIG`). While holding `GLO`, turning `SELECT` chooses **`FREQ`** (`PITCH | SPREAD | DETUNE | TILT` mutually orthogonal 4-voice pitch basis), **`FILTER`** (`HPF | HP RES | LPF | LP RES` relative octave/linear deltas), **`LFO`** (`TARGET | SHAPE | RATE | DEPTH` across all 4 LFOs), **`AMP`** (`LEVEL | TILT | WIDTH | BALANCE` across VCA/Drive + stereo field), or **`ALL`** (`FREQ | FILTER | LFO | AMP` 4-stage macro), each paired with a bespoke acoustic center visualization and 4-voice spread micro-beads on diverged bottom-strip filaments.
- **`MONO` · `POLY`:** Instant `OUTPUT ROUTING` toggle between **Dual Mono (`MONO`)** (for mono pedalboards) and **Stereo (`POLY`)** (for stereo pedals / dual Eurorack channels).
- **11th Tall Black Key (`F#5`):** **Momentary Scope Freeze** (`SCOPE FREEZE` / `HOLDING`; LED `0%` at rest, `100%` while freezing the coupled 4-voice caustic ribbon mesh and optical bloom).
- **Unassigned / Reserved Black Keys (`OP1`–`OP5`, `PIT`):** LEDs stay **`0%` dark** at rest; pressing any unassigned black key displays `OP1 KEY`..`OP5 KEY` or `PIT KEY` + `UNASSIGNED` without overriding `KNOB 1–4` (per-voice Filters and Channel Level/Overdrive live on the right-grid **`FX`** and **`EDIT`** pages).

#### Bottom Row — 16 White Keys (4 Silkscreen-Dotted Groups of 4)
- **Dot 1 (`White Keys 1-4`): Osc 1-4 Latching Mute / Gate Switches**
  - Clean tap toggles `Osc 1-4` gate (`GATE OSC 1..4`: `ACTIVE` / `MUTED` with `2-5 ms` clickless vactrol fade), remembering the oscillator's pre-mute amplitude when toggled back on. On the **`EDIT`** (`Level & Drive`) page, muted oscillators display **`0%`** (without erasing their remembered level unless turned) and turning their `KNOB 1-4` clockwise fades them in smoothly from `0%` while clearing their mute state.
  - **Acoustic & Diagnostic Key LED Feedback (`LedEngine.cpp` — Smooth `C^2` Transitions, Zero Harsh Pulses):**
    - *User-Activated Muted (`muted == true`):* Pulses smoothly at **`1.50 Hz`** (`4% <-> 54%` `C^2` analog sine, gliding in/out smoothly via an S-curve slew on tap) to clearly catch the eye whenever an oscillator is silent because it was muted.
    - *Active Below Hearing Range (`!muted`, `< 20 Hz`):* Breathes slowly and calmly at **`0.35 Hz`** (`~2.86 s` cycle — `4.3x` slower than Muted, `14% <-> ~70%` `C^2` analog sine, smoothly crossfading across `15–25 Hz`) to indicate an unmuted oscillator running below audible pitch.
    - *Active at Audible Rate (`!muted`, `>= 25 Hz`):* Steady **`60–100%`** scaled by voice level (tracking slow `< 2 Hz` `AMP` LFO tremolo smoothly while tapering out `>= 7 Hz` fast/audio-rate modulation so keys never strobe).
- **Dot 2 (`White Keys 5-8`): Osc 1-4 MS-20 Filter / Vactrol LPG Pressure Expression Pads**
  - Rest at **`24%` soft glow** and flare continuously up to **`100%`** proportionally to finger pressure (`0.000 Hz` pitch shift):
  - Displays `FILTER EXP OSC 1..4` (or `LPG SWELL OSC 1..4` when muted) + live `0-100%` bar and the live pressure-modulated MS-20 passband curve on the HUD.
  - *When Osc is Active:* **Lean harder** to sweep its **MS-20 Filter + Full `Korg35` Dual HP/LP Resonance** (uncapped up to `100%` resonance and free to cross `HPF > LPF` into screaming band-reject/self-oscillating territories, decaying via a `26 ms` CdS vactrol tail on release).
  - *When Osc is Muted:* **Press & lean harder** to swell that oscillator in from silence through a **Buchla-style Vactrol Low-Pass Gate (`LPG`)** (`85 Hz` sub murmur -> full harmonic roar).
- **Dots 3 & 4 (`White Keys 9-16`): 8 Snapshot Slots (`SNAPSHOT 1..8`)**
  - *Quick Tap:* Glides smoothly to Snapshot `1-8` (`0%` empty, `20%` saved, `100%` active, slow breathing during glide).
  - *Press, Hold & Lean:* **Rubber-band morphs** all 4 oscillators, filters, and LFOs *toward* that snapshot proportionally to finger pressure (`20% -> 100%` proportional LED flare)—release your finger, and the sound melts smoothly back to your active state.
  - *`PRESETS` Macro Scrub Real-Time Keybed Display (`ADR-025`):* Turning `PRESETS` pans varibright LED intensity smoothly across populated snapshot keys (`led[s] = 20% + 80% * morphWeight[s]`, skipping `0%` empty slots) in real time—leaving both participating keys proportionally lit whenever parked on an intermediate hybrid state, even after the screen HUD fades out.

---

## 4. Coupled 4-Voice Monochrome Caustic Ribbon Scope + Divider-Free Sans-Serif HUD

- **Pure Monochromatic B&W Aesthetic:** Pitch-black (`#000000`) background, strictly neutral achromatic white/silver shades (`R == G == B`), anti-aliased sans-serif typography (`Helvetica Neue`), and **zero divider lines, boxes, or grid clutter**.
- **Open Edge-to-Edge Coupled 4-Voice Caustic Ribbon (`100%` of the time, constant vertical scale):**
  - Sample-accurately aligned post-MS-20 per-voice taps (`Osc 1..4`) + stereo master bus (`L/R`) locked with sub-sample Catmull-Rom zero-crossing interpolation, HF-rejected composite fundamental trigger sway, and subtle `1/f` analog thermal breath so no strand ever freezes into a static stencil.
  - **14 Coupled Circuit-State Strands (`4 voices × 3 + 2 stereo ribs`):** Each active oscillator renders its `240 Hz` TPT leaky-capacitor trajectory, a 3-frame historical dispersive shell, and its live post-MS-20 filtered trace (with ~`24%` sympathetic summing-node deflection when multiple voices play, and clean vactrol fade-out to `0` weight when muted), paired with 2 master stereo harmonic ribs.
  - **Additive Caustic Envelope + `25%` Quarter-Res Separable Bloom:** Overlapping strands accumulate in linear photon space where envelope crossings flare naturally to incandescent `#FFFFFF` white, backed by a `60x60` 2-pass separable `[1, 3, 4, 3, 1]` optical bloom (`25%` calibrated intensity), soft-knee Reinhard + `0.82` CRT gamma tonemapping, and `4x4` ordered Bayer neutral `RGB565` dithering.
  - **Open Full-Span Geometry (`x = 0..239`, constant `y = 120` center & `112 px` linear CRT deflection) & Zero Hard-Cut Transitions:** Zero artificial edge pinching to `0 V` at the left/right bezels, **zero vertical compression or shift** when the HUD fades in (resonant peaks deflect linearly past `y = 0` and `y = 239` without soft-clip squashing), and **zero 1-frame visual snaps** anywhere on the instrument—stopping audio (`PLAY/STOP`, `MASTER = 0%`, muting all voices, or pausing audio) smoothly relaxes all 14 strands + bloom into a poised `y = 120` horizon flatline and blooms back out on resume, while per-voice mutes, `1x <-> 2x` bass timebase zoom, filter curves, optical beads, and HUD overlays all transition continuously.
- **Unified Soft-Transitioning Optical-Filament HUD (`150 ms` Centralized S-Curve Envelope):**
  - **Top Function Header (`13 pt` Bold Two-Tone Hierarchy & Top-Right `FINE`/`COARSE` Transition):** Reserved top strip (`y = 6..22`) leading with the stationary active function title (`FREQUENCY`, `FILTER`, `LEVEL`, `PANNING`, `GLOBAL`, `LFO`, `GLOBAL CONFIG`) in `100%`-white `13 pt` Bold `Helvetica Neue` (`kFontSansBold`), followed by dimmed trailing target qualifiers (`OSC 1..4`, `FREQ`, `FILTER`, `LFO`, `AMP`, `ALL` at `~46%` silver) alongside pixel-exact top-aligned (`y = 9`) right-hand `10 pt` metadata, `150 ms`-transitioning `FINE` / `COARSE` rate badges (paired with live `0.01 Hz` / `%` values where needed), or `13 pt` `%` + tapered optical-filament gauges (paired with idle top-right `L/R` optical-filament peak meters when the HUD is closed).
  - **Bottom 4-Knob Strip, 4-Voice Spread Micro-Beads & Bespoke Acoustic Center Visualizations:** Reserved bottom strip (`y = 190..235`) **always** showing what `KNOB 1..4` currently control whenever the HUD is visible (hidden only inside `GLOBAL CONFIG` where `KNOB 1..4` edit the highlighted menu row), with 4 tapered sub-pixel optical filaments, focal beads, 4 subtle per-voice spread micro-beads when `GLOBAL` voices diverge, and tabular sans-serif readouts (`55.0 Hz`, `2.4 kHz`, `65%`, `L35`, `MIXED`). Center visualizations all share a `72%` background scope dim synchronized to the same `150 ms` S-curve:
    - **`EDIT` Channel Level & MS-20 Drive (`HudAmpGraph`):** Full-screen `4`-column vertical console/saturation view (`x = 0..239, y = 26..188`) with `0%` floor (`y = 180`), `70%` (`1.0x`) Drive Knee horizon (`y = 79.2`), clean VCA stems + optical beads, live slewed `voicePeak` plasma, symmetrical linear-photon saturation wings + `"1.1x..4.5x"` micro-callouts above `70%`, and AM LFO coronas.
    - **MS-20 Passband (`FX` Page, `Keys 5–8` Filter/LPG, `GLOBAL FILTER`):** `80 px` (`y = 102..182`, `x = 10..230`, `2 Hz–28 kHz`) Option A3 dual-peak MS-20 transfer curve(s) with `10%` passband silhouette, `30%` crest glow, and `HP`/`LP` focal beads (`4` overlaid per-voice curves in `GLOBAL FILTER`).
    - **`GLOBAL FREQ` Spectral Stems:** `0.05 Hz–20 kHz` log-frequency horizon (`y = 104..182`) with 4 voice stems, amplitude-scaled radiant tips, and centroid guide.
    - **`GLOBAL LFO` & `LFO` Full-Screen Wave:** Edge-to-edge (`x = 0..239`, `y = 26..188`, `±75 px` on `y = 107`) morphed LFO wave(s) and high-radiance Phase Beacon(s) (`1` voice on `LFO` page, `4` voices overlaid in `GLOBAL LFO`).
    - **`GLOBAL AMP` Stereo Soundstage:** `L..C..R` equal-power stereo horizon (`y = 104..182`) with center axis tick, stereo width bracket, and 4 amplitude-height voice pillars (`0%` if muted).
    - **`GLOBAL ALL` Unified Synth Patch View:** `2 Hz–28 kHz` 4-voice average MS-20 passband overlaid with the 4 live oscillator pitch/amplitude stems.
  - **Centralized `150 ms` S-Curve Envelope (`HudEnvelope.h`), Latched Pages & Outgoing-View Latch:** Every HUD fade-in and fade-out across the entire instrument—keybed holds, standalone knob `1.5 s` toasts, center graphs, `72%` backdrop dim, `HOME` LED, and page opens/closes—runs on a single centralized **`150 ms` linear ramp mapped through smoothstep** (`kHudFadeSec = 0.150f`, `kHudToastHoldSec = 1.500f`). Right `2x6` switches (`PLAY/STOP`, `SAVE` while armed, `FX`, `EDIT`, `LFO`, `GLOBAL CONFIG`, unassigned switches) latch until `HOME` (or a second toggle tap on `FX` / `EDIT` / `LFO` / `GLO` / `SAVE`), smoothly returning to any latched page after momentary holds or `1.5 s` knob toasts, while outgoing labels latch during fade-out so `FREQUENCY` never flashes on release.

---

## 5. Explicit Non-Goals (v1 Scope Guardrails)

To keep Ruido focused, reliable, and sonically uncompromising, **v1 explicitly excludes**:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI polyphony / voice-stealing envelopes.
- Onboard time-based FX (reverb, delay, chorus)—Ruido is the **dry analog-style source** feeding your external pedals and Eurorack modules.
- Deep nested sub-menus or file browsers (focused single-level pages on right-grid buttons—`FX`, `EDIT`, `LFO`, and `GLO` Config—are used judiciously, always instant-exit via `HOME`).
