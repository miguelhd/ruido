# Ruido — Product & UX Specification

## 1. Vision & Musical Identity

**Ruido** transforms the pocket **M-VAVE FM-1** hardware into a dedicated, boutique **4-oscillator CMOS Schmitt-trigger stereo drone instrument**.

- **Primary Acoustic Reference:** Classic DIY analog Schmitt-trigger relaxation oscillator synths (`CD40106` / `74HC14` style)—organic capacitor tilt, micro-thermal drift, shared power-rail phase pulling, and starved-rail hard-sync tearing—paired with feral **Korg MS-20 `Rev 2 -> Korg35` dual `HPF -> LPF` silicon-limited resonant filters** and **ESP-style asymmetric pre-filter transistor overdrive**.
- **Primary Studio Role:** A pristine-to-feral, harmonically rich, zero-aliasing stereo sound source with **zero artificial musical clamps**, designed specifically to feed **external guitar/synth effects pedals** (fuzz, wavefolders, BBD delays, reverbs, granular) and **Eurorack modular rigs**.

### Core Design Values (In Priority Order)
1. **Great Sound & Unclamped Extremes:** Spend every spare CPU cycle on analog-faithful oscillator physics, `4x` oversampling, nonlinear Zero-Delay Feedback filtering, and clickless slewing—imposing safety limits strictly for numerical/hardware protection (anti-aliasing, zero `NaN`/DC runaway, DAC rail safety), never to domesticate musical range.
2. **Immediate, Ergonomic UX with Judicious Pages:** Default idle state is always live waveform + `KNOB1-4 = Osc 1-4 Frequency`, with momentary holds strictly on the left/bottom (`GLO`, `OCT-/OCT+`) so forearms never cross over the display, and latching toggles / judicious single-level pages on the right-side `2x6` button grid (`FX` MS-20 Filters, `SEL` Stereo Panning, `EDIT` `DUTY · DRIFT · LEVEL`, `LFO` modulation, and `GLO` `GLOBAL CONFIG`, always one tap from `HOME`).
3. **Useful Waveform Visuals:** An always-on oscilloscope that serves as an actual acoustic tuning and gain-staging instrument, with minimal controls gently fading in over the wave only while touched or viewing a focused page.

---

## 2. Voice & Signal Architecture (User-Facing)

Ruido consists of **4 identical mono voices** (`Osc 1..4`) mixed in stereo through a subtle nonlinear master bus:

```text
[Schmitt Pulse/Square Osc 1..4 (DUTY + 1/f DRIFT)] ──> [Pre-Filter Drive 1.0x..4.5x] ──> [MS-20 Dual HPF -> LPF 1..4] ──> (VCA + Gate + Pan) ──> [Master Sag] ──> L/R Out
                       ▲                                                                                                                             │
                       └────────────────────────── Shared Power-Rail Coupling & Starved-Rail Sync (`ALGORITHM`: 0-100%) ─────────────────────────────┘
```

### Per-Oscillator Controls (`Osc 1..4`)
- **Frequency:** Unbroken continuous stepless sweep from **Sub-Cycle Pulses to Ultrasonic Edge (`0.05 Hz - 20,000 Hz`)** (`1.5 Hz` RC coupling corner preserving `-0.04 dB` seismic `10–30 Hz` sub-bass weight), using **strictly linear 1:1 X4 quadrature scaling (`96 steps/turn`, zero velocity acceleration)** + **`C^1` critically-damped log-octave smoothing**:
  - *`FINE` (`Hold OCT-`):* Surgical `~0.66 cents/step` (`~63 cents/turn`) precision for locking slow phase beating and harmonic intervals.
  - *`STANDARD` (Released Default):* Predictable linear `0.0387 oct/step` (`~3.7 oct/turn`) sweep regardless of hand speed.
  - *`COARSE` (`Hold OCT+`):* Rapid end-to-end `0.05 Hz - 20,000 Hz` (`18.6`-octave) sweep in **`~2.0` turns** (`0.096 oct/step`).
- **Pulse Duty (`EDIT -> DUTY`, `5-95%`):** Continuous piecewise-slope PolyBLEP CMOS Schmitt comparator duty-cycle control (`50%` square default, sweeping from `5%` razor-thin nasal pulse through `50%` hollow square to `95%` wide pulse with exact fundamental pitch invariance and `< -109 dBc` alias suppression).
- **Thermal Pitch Drift (`EDIT -> DRIFT`, `0-100%`):** Per-oscillator `1/f` analog RC thermal pitch instability (`10%` / `~±2.2 cents` organic default; `0%` = **`LOCKED`** `0.00 cents` crystal phase-lock up to `100%` = `~±120 cents` tape-warble / unstable transistor swarm).
- **Dual MS-20 `Rev 2 -> Korg35` Filter (`HPF -> LPF` in series across `2.0 Hz - 28,000 Hz`, Zero Smart-Park Muting):**
  - **HP Cutoff (`KNOB1`):** `2.0 Hz` (`0%` RES parked flat/transparent) up to `28,000 Hz`, engaging warm `LM13600` OTA saturation across all 4 filter poles even at `0%` resonance.
  - **HP Resonance (`KNOB2`):** Unmuted full-range High-Pass Peak (`0-100%`)—`0-60%` models creamy **`Rev 2`** (`LM13600` OTA + `1S1588`, `k <= 3.60`), while `60-100%` morphs continuously into feral early **`Korg35`** (`k` up to `6.80`, tighter diode knees, asymmetric Stage-2 BJT/OTA rails), self-oscillating all the way down to `2 Hz` infrasonic throb with exact `0 Hz` capacitive DC rejection.
  - **LP Cutoff (`KNOB3`):** `2.0 Hz` up to `28,000 Hz` (`0%` RES parked flat/wide-open), rounding square edges with creamy `LM13600` OTA 2nd/3rd harmonic warmth and extending self-oscillation past `20 kHz`.
  - **LP Resonance (`KNOB4`):** Unmuted full-range Low-Pass Peak (`0-100%`, `Rev 2` `0-60%` -> `Korg35` `60-100%` up to `k = 6.80`), unpadding `HPF -> LPF` inter-stage gain above `60%` so dual-resonant formants slam `IC6` into raw intermodulation screaming without bass-quenching.
- **Channel Level & MS-20 Pre-Filter Overdrive (`EDIT -> LEVEL`, `0-100%`, Boot Default on `EDIT`):** Two-zone analog console/ESP gain law (`computeVoiceChannelGain`): `0-70%` (`kCleanAmpKnee`) = clean post-filter VCA attenuation (`1.0x` pre-drive; `65%` default); **`70-100%` = `1.0x -> 4.5x` asymmetric pre-filter MS-20 transistor overdrive** slamming `HPF -> LPF` into heavy 2nd/3rd harmonic saturation + `+4.1 dB` post-filter trim.
- **Stereo Pan (`SEL` Page):** Equal-power (`sin`/`cos`) stereo placement from hard Left (`L100`) through Center (`C`) to hard Right (`R100`), or collapsed to Dual Mono (`L == R`) when `MONO` output routing is active.

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
1. **Strict Left-Hand Momentary Holds:** Every momentary `"Hold + Turn KNOB1-4"` gesture lives in the **bottom-center black key** (`GLO`) and left-hand **`OCT-` / `OCT+`**. The left hand stays down-left, the right hand stays top-right on `KNOB1-4`, and the center `240x240` screen is never occluded by a crossed forearm.
2. **Right-Side `2x6` Button Grid = Non-Momentary Toggles & Judicious Pages:** None of the `2x6` buttons directly below `KNOB1-4` are momentary holds; they act as latching toggles, immediate utilities, or standardized single-level page switches (`FX` MS-20 Filters, `SEL` Stereo Panning, `EDIT` `DUTY · DRIFT · LEVEL`, `LFO` modulation, and `GLO` `GLOBAL CONFIG`).

### 3.1 Top-Right Encoders: `KNOB1` – `KNOB4` (Idle & 5-Mode `GLOBAL`)

*(Focused right-grid pages `FX`, `SEL`, `EDIT`, and `LFO` also remap `KNOB1-4` while latched open; see §3.4.)*

| Left-Hand Hold (Black Key) | `KNOB1` | `KNOB2` | `KNOB3` | `KNOB4` | Monochrome HUD Header + Unified Center View + Bottom 4-Knob Strip |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **None (Idle Default)** | **Osc 1 Freq** | **Osc 2 Freq** | **Osc 3 Freq** | **Osc 4 Freq** | `FREQUENCY OSC 1..4` (`XX.XX Hz` exact counter) + pure undimmed live 4-voice waveform scope + 4 `Hz` filaments |
| **Hold `GLO` (`SELECT = FREQ`)** *(Lit `25%`)* | **Freq** | **Spread** | **Detune** | **Tilt** | `GLOBAL FREQ` + pure undimmed live 4-voice waveform scope (identical to `FREQUENCY`) + 4-voice spread micro-beads |
| **Hold `GLO` (`SELECT = FILTER`)** *(Lit `25%`)* | **All HPF** | **All HP Res** | **All LPF** | **All LP Res** | `GLOBAL FILTER` + 4-voice overlaid `2 Hz–28 kHz` MS-20 passbands & beads + relative delta filaments |
| **Hold `GLO` (`SELECT = LFO`)** *(Lit `25%`)* | **All Target** | **All Shape** | **All Rate** | **All Depth** | `GLOBAL LFO` (`X/8 ACTIVE` / `BYPASSED`) + 4-voice morphed LFO waves & 4 phase beacons (`RATE` scales all 8 proportionally; `DEPTH` scales active LFOs only or wakes `LFO A` across `OSC 1..4`) |
| **Hold `GLO` (`SELECT = LEVEL`)** *(Lit `25%`)* | **Level** | **Tilt** | **Spread** | **Pan** | `GLOBAL LEVEL` (`CLEAN VCA` / `DRIVE x.xx` / `STEREO` / `MONO`) + unified `LEVEL` + `3A` overdrive graph on `K1–K3`, switching cleanly to `L..C..R` `PAN` soundstage on `K4` (`PAN`, strictly locked as `LOCKED` in `MONO`) |
| **Hold `GLO` (`SELECT = ALL`)** *(Lit `25%`, Default)* | **Freq** | **Filter** | **LFO** | **Level** | `GLOBAL ALL` + focused-knob 1-to-1 center view (`K1` = pure live scope, `K2` & initial overview = MS-20 `FILTER` graph, `K3` = `LFO` graph, `K4` = `LEVEL` + `3A` overdrive graph) |

### 3.2 Top-Left Knobs & `OCT-` / `OCT+` (Zero-Modifier Global Controls)
- **`MASTER` (Analog Pot):** Hardware master output level (`MASTER LEVEL` + `%` bar on HUD, anchored with `> 0.2%` cumulative deadband so slow analog sweeps wake the HUD reliably without ADC chatter).
- **`ALGORITHM` (Encoder):** **Rail Coupling & Starved-Rail Chaos** (`0-45%` smooth `CD40106` shared-rail threshold pulling around the `25%` default <-> `45-100%` starved-CMOS voltage collapse: `[0.12x..4.5x]` phase pull, neighbor gate-load PWM duty warp, and band-limited PolyBLEP subharmonic hard-sync locking). When **`GLOBAL CONFIG`** (right-grid `GLO`) is open, **`ALGORITHM`** steps (`1` option/detent) the highlighted menu row's option value so both menu navigation (`SELECT`) and option editing (`ALGORITHM`) stay paired on the left-hand encoder column while `KNOB 1–4` remain `Osc 1–4 Frequency`.
- **`SELECT` (Encoder):** Context-sensitive discrete left-hand mode/voice/row selector (`1` item per `4`-tick detent across all 3 gears):
  - While holding keybed **`GLO`**: steps **`GLOBAL`** across **`FREQ` · `FILTER` · `LFO` · `LEVEL` · `ALL`** (defaults to `ALL` on boot and remembers your last selected destination).
  - When the **`EDIT` 3-Mode Oscillator Channel Strip** is open: steps strictly in signal-flow order across **`DUTY` · `DRIFT` · `LEVEL` *(boot default)***, remembering your last selected mode across page re-opens.
  - When the **Per-Voice MS-20 Dual Filter Page** (`FX`) is open: steps strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**.
  - When the **Stereo Pan Page** (`SEL`) is open: switches **`OUTPUT ROUTING`** between **`STEREO`** and **`MONO`** (displaying `OUTPUT ROUTING · STEREO / MONO` in the top header for `1.5 s` before settling back to `PAN`).
  - When the **Dual Per-Oscillator LFO Page** (`LFO`) is open: steps strictly across all 8 modulation slots **`LFO A OSC 1` · `LFO B OSC 1` · `LFO A OSC 2` · `LFO B OSC 2` · `LFO A OSC 3` · `LFO B OSC 3` · `LFO A OSC 4` · `LFO B OSC 4`**.
  - When **`GLOBAL CONFIG`** (right-grid `GLO`) is open: moves (`1` row/detent) the row cursor up/down the configuration list.
  - Outside those contexts: displays `SELECT ENCODER · UNASSIGNED` (`1.5 s` toast).
- **`PRESETS` (Encoder):** Reserved for **Snapshot Morph (Continuous Macro Scrub)** in Milestone 3 (currently displays `PRESETS ENCODER · UNASSIGNED` `1.5 s` toast until the 8-slot Snapshot Engine lands; once active, smoothly scrubs/crossfades across saved snapshots while panning varibright LED brightness across **`White Keys 9–16`** per `ADR-025`).
- **`OCT-` / `OCT+` (Momentary Rate Modifiers — Top-Right Status Badge & `150 ms` Transition):** **Strictly momentary left-hand encoder rate modifiers** governing continuous endless rotary sweeps with **zero velocity acceleration** (100% deterministic 1:1 angle-to-parameter response at `96` X4 quadrature transitions per `360°` turn), freely chordable alongside `GLO` and any open right-grid page (discrete notched selectors like `SELECT` on `FX`/`EDIT`/`LFO`/`GLO`, `LFO TARGET`, and `GLOBAL CONFIG` rows remain `1` item per detent across all 3 gears):
  - *Released (Idle Default = `STANDARD`):* Both `OCT-` and `OCT+` rest at `15%` dim glow; full continuous parameter sweeps take an exact linear **`2.5-2.8` turns**.
  - *Hold `OCT-` -> **`FINE`** (`OCT-` LED `100%`):* Surgical fine-tuning while held (`0.66 cents/step` audio frequency, `3.0 cents/step` filter cutoff, `0.09%/step` linear controls).
  - *Hold `OCT+` -> **`COARSE`** (`OCT+` LED `100%`):* Rapid performance sweep while held—covering the **entire `0-100%` / `0.05 Hz-20 kHz` / `2 Hz-28 kHz` sweep in `~1.3-2.0` turns**.
  - *Anchored Left Title + Top-Right `FINE` / `COARSE` Transition:* Holding `OCT-` or `OCT+` **never moves or replaces the active page/tool title on the top-left** (`FREQUENCY` when held from idle). Instead, the **top-right status slot** smoothly crossfades over **`150 ms`** into **`FINE`** or **`COARSE`** (with zero underline or divider rules)—preserving live precision values alongside the badge (`FINE · 110.04 Hz` on `FREQUENCY`, `FINE · 25%` on `RAIL COUPLING`)—and smoothly dissolves back on release.
  - *Release:* Immediately returns to `STANDARD` rate (`15%` rest glow); if a knob was turned during the hold, its remaining `1.5 s` readout toast completes before dissolving in `150 ms` (or fades out immediately in `150 ms` if `OCT-/+` was tapped alone).

### 3.3 Right-Panel `2x6` Function Buttons (Latching Toggles & Focused Pages)
- **`PLAY/STOP`:** **Audio Output** toggle (`ON` / `OFF`, clickless fade to silence and back for safe cable patching). Drives the hardware's dual-channel `PLAY/STOP` LEDs by action affordance: steady **Green** (`PLAY` LED) when quiet (`OFF`), steady **Red** (`STOP` LED) when sound is `ON`, crossfading smoothly between them.
- **`HOME`:** **Toggle Active HUD Off <-> Latch `FREQUENCY` HUD** — when any page, modifier, or transient toast is active, pressing `HOME` hides the active HUD and dissolves smoothly over **`150 ms`** to the pure edge-to-edge waveform (`12%` rest LED, `12%–35%` ambient fade while other HUDs are visible); when no HUD is active, pressing `HOME` latches the **`FREQUENCY`** HUD open (`FREQUENCY` / `FREQUENCY OSC 1..4` + bottom `OSC 1..4` `Hz` gauges over the `100%` undimmed live scope, `100%` `HOME` LED) until tapped again or switched to another page.
- **`SAVE`:** Tap `SAVE` (`STORE SNAPSHOT` / `TAP KEY 9-16`, `100%` LED) then tap a Snapshot white key (`9-16`) to store the current patch; tapping `SAVE` a second time disarms and displays `STORE SNAPSHOT · CANCELLED` (`1.5 s` toast -> `150 ms` fade-out).
- **`FX`:** Opens/closes the **Per-Voice MS-20 Dual Filter Page** (see §3.4 below); LED glows at **`15%` rest** when all active voices' filters are parked flat (`HPF = 2.0 Hz`, `LPF = 28 kHz`, `0%` resonance), scales dynamically (**`15%–85%`**) whenever any unmuted or pressure-active voice has filter cutoff or resonance engaged (including live Filter LFO or Pad Pressure sweeps), and lights to **`100%`** while the page is open.
- **`SEL`:** Opens/closes the **Stereo Pan Page (`PAN`)** (see §3.4 below); LED glows at **`15%` rest** within the default `±65%` stereo spread, scales dynamically (**`15%–75%`**) whenever any active stereo voice widens past `±65%` or engages `PAN` LFO modulation, and lights to **`100%`** while the page is open.
- **`EDIT`:** Opens/closes the **3-Mode Oscillator Channel Strip (`DUTY · DRIFT · LEVEL`)** (see §3.4 below); LED glows at **`15%` dim rest** whenever closed or viewing another function, and lights to **`100%`** while `EDIT` is open.
- **`LFO`:** Opens/closes the **Dual Per-Oscillator LFO Page** (see §3.4 below); LED glows at **`15%` rest** when all 8 LFOs (`LFO A/B` across `OSC 1..4`) are at `0%` depth (confirming an assigned page switch), breathes continuously (`15%–60%`) with active LFO modulation when closed, and lights to **`100%`** while the page is open.
- **`GLO` (Right Grid under `KNOB 1–4`):** Opens/closes the scalable **`GLOBAL CONFIG`** menu (`15%` steady rest LED, `100%` while open; see §3.4 below).
- **Unassigned / Reserved Switches (`ENV`, `ARP`, `SEQ`, `REC`):** LEDs stay **`0%` completely unlit** at rest while unassigned (`REC` is physically a single-color **Red** LED on the FM-1 PCB); pressing any unassigned switch closes open pages and displays its silkscreen name + `UNASSIGNED` in the HUD header so every control confirms hardware contact.

### 3.4 Focused Right-Grid Pages (`FX`, `SEL`, `EDIT`, `LFO`, `GLOBAL CONFIG`)

#### Per-Voice MS-20 Dual Filter Page (`FX` — `ADR-029`)
- **Enter / Exit:** Tap **`FX`** (top-left switch of the `2x6` grid below `KNOB 1`) to open/close the latching MS-20 Dual Filter page (`FILTER OSC 1..4` with `12 dB HP -> LP` right header); mutually exclusive with `SEL`, `EDIT`, `LFO`, `GLOBAL CONFIG`, and `SAVE`, and dismissed in `150 ms` by a second `FX` tap or **`HOME`** (momentary holds `GLO`, `ScopeFreezeKey`, and `White Keys 5–16` temporarily override and return seamlessly on release).
- **`SELECT` Encoder (Left Hand):** Steps (`1` item/detent, clamped `0..3` with zero windup) strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**, remembering the last focused voice across page re-opens.
- **`KNOB 1–4` (`HPF | HP RES | LPF | LP RES`):** Adjust High-Pass Cutoff (`2 Hz–28 kHz`), High-Pass Resonance (`0–100%`), Low-Pass Cutoff (`2 Hz–28 kHz`), and Low-Pass Resonance (`0–100%`) for the selected voice (while multi-voice relative filter sweeps remain immediately available on **`Hold GLO` -> `GLOBAL FILTER`**).
- **Live Full-Screen MS-20 Passband Visualization:** Spans edge-to-edge across the display (`2 Hz–28 kHz`, anchored to the shared unipolar graph baseline) over a dimmed live waveform backdrop, rendering the `0 dB` passband reference, dual `Korg35` resonant peaks, and real-time Filter LFO + Pad Pressure modulation for the focused voice.

#### Dedicated Stereo Panning Page (`SEL` — `ADR-032`, `ADR-033`)
- **Enter / Exit:** Tap **`SEL`** (right-grid switch next to `FX`) to open/close the latching Stereo Pan page (`PAN` / `PAN OSC 1..4` with `STEREO`, `MONO`, or `MUTED` right header); mutually exclusive with `FX`, `EDIT`, `LFO`, `GLOBAL CONFIG`, and `SAVE`, and dismissed in `150 ms` by a second `SEL` tap or **`HOME`** (momentary holds `GLO`, `ScopeFreezeKey`, and `White Keys 5–16` temporarily override and return seamlessly on release).
- **`SELECT` (`STEREO` / `MONO`), `KNOB 1–4` (`OSC 1..4`) & `MONO` Pan Lock:** Left-hand **`SELECT`** switches **`OUTPUT ROUTING`** directly between **`STEREO`** (`CCW`) and **`MONO`** (`CW`) right on the `PAN` page while keeping the top-left title steady as **`PAN`** (**`PAN` · `STEREO` / `MONO`**) and keeping the live soundstage curve on screen. In `STEREO`, **`KNOB 1–4`** adjust equal-power stereo placement (`L100..C..R100`) per oscillator across all 3 encoder gears (`Standard`, `OCT- Fine`, `OCT+ Coarse`) and spotlight that oscillator's curve. While **`MONO`** is active (`GlobalParams::monoCollapse`), `KNOB 1–4` on `SEL` (and `KNOB 4 = PAN` on `GLOBAL LEVEL`) act strictly **locked** (no stored stereo pan mutation, no column focus, and never changing the `PAN` title), and the 4 `OSC 1–4` bottom columns on `SEL` are replaced by a single centered **`OUTPUT ROUTING SET TO MONO`** caption.
- **Live Edge-to-Edge Stereo Panorama Curve:** Spatial sister to `FX` and `LFO` over a dimmed live scope (shared with `GLOBAL LEVEL` `KNOB 4 = PAN`), anchored to the shared unipolar graph baseline with a subtle stereo center guide in `STEREO`. Renders 4 acoustic localization bells (psychoacoustic bass-to-treble width, height tracking voice `LEVEL` and live signal peak) with bright spotlight contrast when turning a voice knob, `LFO -> PAN` excursion brackets + Phase Beacons, and automatic centered convergence in `MONO`.

#### 3-Mode Oscillator Channel Strip (`EDIT` — `ADR-028`, `ADR-030`, `ADR-032`)
- **Enter / Exit:** Tap **`EDIT`** (right-grid switch below `KNOB 1–4`) to open/close the latching 3-mode per-oscillator channel strip (`DUTY`, `DRIFT`, **`LEVEL`** *(boot default)*); mutually exclusive with `FX`, `SEL`, `LFO`, `GLOBAL CONFIG`, and `SAVE`, and dismissed in `150 ms` by a second `EDIT` tap or **`HOME`** (momentary hold `GLO` temporarily overrides and returns seamlessly on release).
- **`SELECT` Encoder (Left Hand) & `KNOB 1–4` (`OSC 1..4`):** **`SELECT`** steps (`1` mode/detent, clamped `0..2` in signal-flow order and remembered across page re-opens) across **`DUTY`** (`5–95%` piecewise-slope PolyBLEP CMOS Schmitt pulse duty, `50%` `SQUARE` default), **`DRIFT`** (`0–100%` `1/f` analog thermal pitch wander, `10%` `~±2.2c` default, `0%` = `LOCKED` `0.00c` -> `100%` = `~±120c`), and **`LEVEL`** (`0–70%` clean post-filter VCA -> **`70–100%` `1.0x..4.5x` MS-20 pre-filter input overdrive** kept 100% fused; turning all the way CCW to the `0%` lower rail without active `LEVEL` LFO engages `MUTED`, while turning CW fades in smoothly from `0%` and clears mute).
- **Live Full-Screen Channel Visualizations:** Aligns 4 vertical optical columns directly above `KNOB 1–4` over a dimmed live scope: `DUTY` renders single-cycle CMOS pulse duty cells, `DRIFT` renders thermal excursion brackets + live wander beacons, and `LEVEL` renders clean VCA phosphor stems + crest beads (`0–70%`, anchored to the shared unipolar graph baseline and brightening monotonically with level) topped above the **`70%` / `1.0x` Overdrive Knee** by a live-audio triangular overdrive flare (`70–100%`).

#### Dual Per-Oscillator LFO Page (`LFO A/B OSC 1..4` — `ADR-022`, `ADR-027`, `ADR-028`, `ADR-031`, `ADR-032`)
Each of the 4 voices (`Osc 1..4`) owns **2 dedicated LFOs (`LFO A` + `LFO B` = `8` total)** matching its dedicated MS-20 filter, VCA/overdrive, and panner (multi-voice LFO macro control lives on Keybed `GLO` -> `GLOBAL LFO`):
- **Enter / Exit:** Tap **`LFO`** (right-grid switch) to open/close the LFO Page (`LFO A OSC 1` .. `LFO B OSC 4` showing `ACTIVE` / `BYPASSED`); tap **`LFO`** again or **`HOME`** to return to the pure scope in `150 ms` while active LFOs continue running.
- **`SELECT` Encoder (Left Hand):** Steps (`1` slot/detent, clamped `0..7` with zero windup) across all 8 modulation slots **`LFO A OSC 1` · `LFO B OSC 1` · `LFO A OSC 2` · `LFO B OSC 2` · `LFO A OSC 3` · `LFO B OSC 3` · `LFO A OSC 4` · `LFO B OSC 4`**, remembering the last focused slot across page re-opens.
- **`KNOB 1–4` (`TARGET | SHAPE | RATE | DEPTH` — Setup Left, Perform Right):**
  - **`KNOB 1` (`TARGET`):** Stepped per-slot parameter selector (`1` item/detent):
    - **Shared on `LFO A` & `LFO B` (`0..7`):** **`FREQ`** · **`DUTY`** (`5–95%` band-limited Schmitt PWM) · **`LPF`** *(default)* · **`LP RES`** · **`HPF`** · **`HP RES`** · **`LEVEL`** · **`PAN`**. Pointing both `A` and `B` at the same destination stacks both modulations sequentially/additively.
    - **Exclusive Cross-Modulation on `LFO B` (`8..9`):** **`A RATE`** (`LFO B` warps `LFO A`'s speed `±6 oct` across `0.01 Hz–2,000 Hz` with zero 1-sample delay) · **`A DEPTH`** (`LFO B` swells and fades `LFO A`'s intensity from stillness up to crest).
  - **`KNOB 2` (`SHAPE`):** Continuous `0–100%` seamless wave morph across **`Sine` (`0%`) -> `Triangle` (`25%`) -> `Sawtooth` (`50%`) -> `Slewed Square` (`75%`) -> `Random` (`100%`, non-repeating organic `C^2` B-spline drift)**.
  - **`KNOB 3` (`RATE`):** Unclamped continuous **`0.01 Hz` (`100 s` glacial cycle) to `2,000 Hz` audio-rate FM / PWM / Filter-FM / AM** log-octave sweep.
  - **`KNOB 4` (`DEPTH`):** Unipolar `0–100%` modulation amount (`±5 oct` pitch FM, `±45%` pulse duty, `±7.5 oct` filter cutoff, `0–100%` resonance, `0–100%` AM into pre-filter overdrive, `±6 oct` `A RATE`) with a hard stop at `0%` CCW so any LFO zeros cleanly by feel.
- **Live Full-Screen LFO + Companion Sibling Visualization:** Spans edge-to-edge across the display over a dimmed background scope, rendering the focused slot (`A` or `B`) in high-contrast foreground with gradient shading and a live Phase Beacon while showing the same oscillator's active sibling (`B` or `A`, when `depth > 0%`) as a silver companion trace (reflecting live `LFO B -> LFO A` rate/depth warp directly on `LFO A`'s curve and bottom-strip readouts).

#### Scalable `GLOBAL CONFIG` Menu (Right-Grid `GLO` — `ADR-027`, `ADR-029`, `ADR-030`, `ADR-032`)
- **Enter / Exit:** Tap right-grid **`GLO`** (`ButtonId::GloRight`) to open/close **`GLOBAL CONFIG`** (`1/N` row counter in top-right; divider-free row list with subtle selected-row tint, zero row-separator lines or left edge bars, and bottom 4-knob strip cleanly hidden over a `72%`-dimmed live scope; `HOME` or 2nd `GLO` tap dissolves in `150 ms`).
- **Navigation & Editing (Paired Left-Hand Encoders):** Turn **`SELECT`** (`1` row/detent) to move the highlighted row cursor across the vertical `5`-row viewport; turn **`ALGORITHM`** (`1` option/detent, directly below `SELECT`) to scroll through the highlighted row's options (leaving `KNOB 1–4` dedicated to `Osc 1–4 Frequency`):
  1. **`GLOBAL RELEASE`:** **`LATCH`** *(default — `GLOBAL` multi-voice edits persist when releasing keybed `GLO`)* / **`SNAP BACK`** *(Elektron `CTRL-ALL`-style temporary performance warp: captures `4 x VoiceParams` on keybed `GLO` press and smoothly restores them via `C^1` parameter slews on release)*.
  2. **`OUTPUT ROUTING`:** **`STEREO`** / **`MONO`** (synced bidirectionally with `GlobalParams::monoCollapse` and the physical `MONO` / `POLY` black keys).

### 3.5 Bottom 27-Key Surface & Carbon-Pill Pressure Expression (`SARADC`)

Using analog `SARADC` sampling of the silicone keybed's carbon-pill contact resistance, Ruido supports continuous **Soma Lyra-8-style finger-lean pressure expression** alongside tactile switching, and **every single key displays live technical feedback on the HUD**:

#### Top Row — 11 Raised Black Keys (Silkscreen-Matched Modifiers & Output Routing)
- **`GLO` (`GLOBAL` + `SELECT`):** **5-Mode Multi-Voice Performance Modifier** (`25%` landmark rest glow, `100%` while held; releasing either latches or snaps back per `GLOBAL RELEASE` in `GLOBAL CONFIG`). While holding `GLO`, turning `SELECT` chooses **`FREQ`** (`FREQ | SPREAD | DETUNE | TILT` mutually orthogonal 4-voice pitch basis over the pure undimmed live scope), **`FILTER`** (`HPF | HP RES | LPF | LP RES` relative octave/linear deltas), **`LFO`** (`TARGET | SHAPE | RATE | DEPTH` across all 8 LFOs: `RATE` scales all 8 proportionally while `DEPTH` scales only active LFOs or wakes `LFO A` across `OSC 1..4` from `0%`), **`LEVEL`** (`LEVEL | TILT | SPREAD | PAN`: ensemble gain, bass<->treble voice-level tilt, RMS dynamic amplitude spread `0–100%`, and ensemble stereo pan shift `L100..C..R100`), or **`ALL`** (`FREQ | FILTER | LFO | LEVEL` 4-stage macro routing 1-to-1 to the focused knob's visualization), paired with 4-voice spread micro-beads on diverged bottom-strip filaments.
- **`MONO` · `POLY`:** Instant `OUTPUT ROUTING` toggle between **Mono (`MONO`, `L == R`)** (for mono pedalboards, locking stereo `PAN` encoders while preserving stereo positions for `POLY`) and **Stereo (`POLY`)** (for stereo pedals / dual Eurorack channels).
- **11th Tall Black Key (`F#5`):** **Momentary Scope Freeze** (`SCOPE FREEZE` / `HOLDING`; LED `0%` at rest, `100%` while freezing the coupled 4-voice caustic ribbon mesh and optical bloom).
- **Unassigned / Reserved Black Keys (`OP1`–`OP6`, `PIT`):** LEDs stay **`0%` dark** at rest; pressing any unassigned black key displays `OP1 KEY`..`OP6 KEY` or `PIT KEY` + `UNASSIGNED` without overriding `KNOB 1–4` (per-voice Filters, Stereo Panning, and Channel `DUTY · DRIFT · LEVEL` live on the right-grid **`FX`**, **`SEL`**, and **`EDIT`** pages).

#### Bottom Row — 16 White Keys (4 Silkscreen-Dotted Groups of 4)
- **Dot 1 (`White Keys 1-4`): Osc 1-4 Latching Mute / Gate Switches & 1-to-1 HUD Audibility Sync**
  - Clean tap toggles `Osc 1-4` gate (`GATE OSC 1..4`: `ACTIVE` / `SUB-AUDIO` / `MUTED` with `2-5 ms` clickless vactrol fade), remembering the oscillator's pre-mute amplitude when toggled back on (or restoring the **`65%` default audible level** if the oscillator had been dialed down to the `0%` `LEVEL` lower rail, so unmuting never leaves a voice silent). On **`EDIT` (`LEVEL`)**, muted oscillators display **`0%`** (`MUTED`) and turning `KNOB 1-4` clockwise fades them in smoothly from `0%` while clearing mute; turning `KNOB 1-4` down to the **`0%` lower rail** (without active `AMP` LFO) automatically engages `MUTED`.
  - **Acoustic & Diagnostic Key LED Feedback + Steady Non-Pulsing Screen Dimming (`LfoMath.h`, `LedEngine.cpp`, `HudChrome.cpp` — `ADR-030`):** Both `White Keys 1-4` and the HUD's `OSC 1-4` lower-rail columns/graphs derive from a single shared `computeVoiceAudibility` model—while **only the physical key LEDs pulse/breathe and the `OSC 1-4` screen text, filaments, beads, and values stay steadily dimmed (`0%` screen pulsing, even when `focusedKnob` is on that column)**:
    - *User-Muted or `0%` Silent Rail (`muted == true` or `amplitude == 0%` without `AMP` LFO):* Key LED pulses smoothly at **`1.50 Hz`** (`4% <-> 54%` `C^2` analog sine, lifting smoothly if swelled via `Pad 5-8` LPG); HUD `OSC 1-4` column stays **steadily dimmed**.
    - *Active Below Hearing Range (`!muted`, `< 20 Hz` down to `0.05 Hz`):* Key LED breathes slowly and calmly at **`0.35 Hz`** (`~2.86 s` cycle — `4.3x` slower than Muted, `14% <-> ~70%` `C^2` analog sine, smoothly crossfading across `15–25 Hz`); HUD `OSC 1-4` column crossfades smoothly into the **exact same steady dimmed state** with zero pulsing.
    - *Active at Audible Rate (`!muted`, `>= 25 Hz`):* Key LED glows steadily at **`60–100%`** scaled by voice level (tracking slow `< 2 Hz` `AMP` LFO tremolo while tapering out `>= 7 Hz` fast/audio-rate modulation); HUD `OSC 1-4` column renders at full crisp contrast (`100%` `#FFFFFF` when focused).
- **Dot 2 (`White Keys 5-8`): Osc 1-4 MS-20 Filter / Vactrol LPG Pressure Expression Pads**
  - Rest at **`24%` soft glow** and flare continuously up to **`100%`** proportionally to finger pressure (`0.000 Hz` pitch shift):
  - Displays `PRESSURE OSC 1..4` + live `0-100%` bar and the live pressure-modulated MS-20 passband curve on the HUD.
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
- **Unified Soft-Transitioning Optical-Filament HUD (`150 ms` Centralized S-Curve Envelope & 1-to-1 Visual Unification):**
  - **Top Function Header (`13 pt` Bold Two-Tone Hierarchy & Top-Right `FINE`/`COARSE` Transition):** Reserved top strip (`y = 6..22`) leading with the stationary active function title (`FREQUENCY`, `DUTY`, `DRIFT`, `FILTER`, `LEVEL`, `PAN`, `PRESSURE`, `GLOBAL`, `LFO A`, `LFO B`, `GLOBAL CONFIG`) in `100%`-white `13 pt` Bold `Helvetica Neue` (`kFontSansBold`), followed by dimmed trailing target qualifiers (`OSC 1..4`, `FREQ`, `FILTER`, `LFO`, `LEVEL`, `ALL` at `~46%` silver) alongside pixel-exact top-aligned (`y = 9`) right-hand `10 pt` metadata, `150 ms`-transitioning `FINE` / `COARSE` rate badges (paired with live `0.01 Hz` / `%` values where needed), or `13 pt` `%` + tapered optical-filament gauges (paired with idle top-right `L/R` optical-filament peak meters when the HUD is closed).
  - **Bottom 4-Knob Strip, 4-Voice Spread Micro-Beads & Unified 1-to-1 Center Visualizations (`ADR-032`):** Reserved bottom strip (`y = 190..235`) **always** showing what `KNOB 1..4` currently control whenever the HUD is visible (hidden only inside `GLOBAL CONFIG` where `SELECT` + `ALGORITHM` navigate/edit the menu list), with 4 tapered sub-pixel optical filaments, focal beads, 4 subtle per-voice spread micro-beads when `GLOBAL` voices diverge, and tabular sans-serif readouts (`55.0 Hz`, `2.4 kHz`, `65%`, `L35`, `MIXED`). Every acoustic domain uses **one single unified visual language** across per-voice and global modes (center graphs share a `72%` background scope dim synchronized to the `150 ms` S-curve, while pure waveform views stay `100%` undimmed):
    - **Pure Live Waveform Scope (`FREQUENCY`, `GLOBAL FREQ`, & `GLOBAL ALL` `K1 = FREQ`):** `100%` undimmed 14-strand coupled caustic ribbon mesh with zero artificial overlays.
    - **`LEVEL` + Option `3A` Live-Audio Overdrive (`EDIT -> LEVEL`, `GLOBAL LEVEL` `K1–K3`, & `GLOBAL ALL` `K4 = LEVEL`):** Full-screen `4`-column vertical circuit view (`x = 0..239, y = 26..188`, `0%` floor at `y = 180`, `70%` / `1.0x` Drive Knee at `y = 79.2`), razor-clean Gaussian phosphor stems + optical beads strictly when `<= 70%`, and **Option `3A` Triangular Hairy Overdrive Cap (`> 70%`)** (3 live-audio persistence traces tapering linearly in a V-triangle from `0 px` at `70%` up to `±3.8 px` max peak width at `100%` `4.5x` overdrive with subtle CRT phosphor bloom). `EDIT` also renders `DUTY` (`5–95%` CMOS pulse duty cells) and `DRIFT` (`1/f` thermal excursion brackets + live `voiceDriftNorm` beacons).
    - **MS-20 Passband (`FX` Page, `Keys 5–8` `PRESSURE`, `GLOBAL FILTER`, & `GLOBAL ALL` `K2 = FILTER` / Initial Overview):** Full-screen (`x = 0..239` edge-to-edge, `y = 26..186`, `160 px` tall, `0 dB` at `y = 86`, `2 Hz–28 kHz`) dual-peak MS-20 transfer curve(s) with `4x`-subsampled steep horn silhouette shading, crest glow, `0 dB` / `-30 dB` reference filaments, and `HP`/`LP` focal beads (`4` overlaid per-voice curves in `GLOBAL FILTER` and `GLOBAL ALL`).
    - **Full-Screen LFO Wave (`LFO` Page, `GLOBAL LFO`, & `GLOBAL ALL` `K3 = LFO`):** Edge-to-edge (`x = 0..239`, `y = 26..188`, `±75 px` on `y = 107`) morphed LFO wave(s) and high-radiance Phase Beacon(s) (focused `A`/`B` slot + `~32%` silver active sibling companion trace on `LFO` page, `4` voices overlaid in `GLOBAL LFO` and `GLOBAL ALL` `K3`).
    - **`PAN` Edge-to-Edge Stereo Panorama Curve (`SEL` `PAN` Page & `GLOBAL LEVEL` `K4 = PAN`):** `HudGraphs`-matched spatial-domain sister to `FX` (`x = 0..239`, `y = 28..184`, `y = 184` baseline + `x = 120` center hairline in `STEREO`) rendering 4 Cauchy-Lorentzian acoustic localization bells with `4x`-subsampled exponential vertical-decay skirts, additive linear-photon spines (`#FFFFFF` spotlight vs `~30%` silver companions), `LFO -> PAN` excursion brackets + Phase Beacons, and centered convergence + centered **`OUTPUT ROUTING SET TO MONO`** bottom caption (omitting `OSC 1–4` columns) in `MONO`.
  - **Centralized `150 ms` S-Curve Envelope (`HudEnvelope.h`), Latched Pages & Outgoing-View Latch:** Every HUD fade-in and fade-out across the entire instrument—keybed holds, standalone knob `1.5 s` toasts, center graphs, `72%` backdrop dim, `HOME` LED, and page opens/closes—runs on a single centralized **`150 ms` linear ramp mapped through smoothstep** (`kHudFadeSec = 0.150f`, `kHudToastHoldSec = 1.500f`). Right `2x6` switches (`PLAY/STOP`, `SAVE` while armed, `FX`, `SEL`, `EDIT`, `LFO`, `GLOBAL CONFIG`, unassigned switches) latch until `HOME` (or a second toggle tap on `FX` / `SEL` / `EDIT` / `LFO` / `GLO` / `SAVE`), smoothly returning to any latched page after momentary holds or `1.5 s` knob toasts, while outgoing labels latch during fade-out so `FREQUENCY` never flashes on release.

---

## 5. Explicit Non-Goals (v1 Scope Guardrails)

To keep Ruido focused, reliable, and sonically uncompromising, **v1 explicitly excludes**:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI polyphony / voice-stealing envelopes.
- Onboard time-based FX (reverb, delay, chorus)—Ruido is the **dry analog-style source** feeding your external pedals and Eurorack modules.
- Deep nested sub-menus or file browsers (focused single-level pages on right-grid buttons—`FX`, `SEL`, `EDIT`, `LFO`, and `GLO` Config—are used judiciously, always instant-exit via `HOME`).
