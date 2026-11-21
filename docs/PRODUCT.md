# Ruido: Product & UX Specification

## 1. Overview

**Ruido** is a 4-oscillator drone and noise synthesizer firmware for the **M-VAVE FM-1** with coupled voices, dual resonant filters, scene morphing, and pressure-sensitive performance effects. It is engineered as a dry stereo sound source for external pedals and modular setups.

- **Signal Path:** Four free-running Schmitt-trigger oscillators share power-rail coupling and thermal pitch drift. Each voice feeds a pre-filter overdrive stage and its own dual Sallen-Key resonant filter (`12 dB/oct HPF -> 12 dB/oct LPF`) at `4x` oversampling (`176.47 kHz` internal rate) into a stereo summing bus.
- **Audio Output:** Dry stereo output for external effects pedals and modular systems over the 3.5 mm TRS analog jack, alongside class-compliant `44.1 / 48 kHz` USB-C digital stereo audio.

### Core Design Values
1. **Sound Quality First:** CPU headroom goes to `4x` oversampling, anti-aliasing, nonlinear zero-delay feedback filters, and smooth parameter transitions. Safety limits protect against numerical errors and hardware clipping and leave the musical range open.
2. **Immediate, Ergonomic UX:** Flat, single-level pages with a direct return to the main scope. Performance modifiers are momentary and reset on release, while active modes stay visible on the LEDs and header. Controls respond linearly and sit around the display for clear two-handed play. Off-page changes appear in the top bar while the current view stays open.
3. **Acoustic Visuals:** The `240x240` monochrome display and all 42 hardware LEDs show live audio and circuit state. Parameter overlays stay minimal and fade back to the waveform at rest.

---

## 2. Voice & Signal Architecture

Ruido runs **4 identical mono voices** (`Osc 1-4`) mixed in stereo through a nonlinear master bus:

```text
[Schmitt Pulse Osc 1..4 (Duty + Drift)] ──> [Pre-Filter Drive] ──> [Dual Sallen-Key HPF -> LPF 1..4] ──> [VCA + Gate + Pan] ──> [Master Bus] ──> L/R Out
                   ▲                                                                                                                   │
                   └──────────────────────────── Shared Power-Rail Coupling & Sync (ALGORITHM: 0-100%) ────────────────────────────────┘
```

### Per-Oscillator Parameters (`Osc 1-4`)
- **Frequency (`0.05 Hz - 20,000 Hz`):** Continuous range from sub-audio pulses (`20 s` cycles) to `20 kHz` with linear, non-accelerated encoder scaling:
  - *`FINE` (`Hold OCT-`):* `~0.66 cents/step` (`~63 cents/turn`) for fine pitch and interval tuning.
  - *`STANDARD` (Default):* `~3.7 octaves/turn`.
  - *`COARSE` (`Hold OCT+`):* Full `0.05 Hz - 20 kHz` range in `~2` turns.
- **Pulse Duty (`HOME -> DUTY`, `5-95%`):** Continuous pulse-width control (`50%` square default) with constant fundamental pitch across the range.
- **Thermal Pitch Drift (`HOME -> DRIFT`, `0-100%`):** Per-oscillator `1/f` pitch drift (`0%` = `LOCKED`, `10%` default, `100%` = `~±120 cents`).
- **Dual Sallen-Key Filter (`FX`, `2.0 Hz - 28,000 Hz`):** Series `12 dB/oct` High-Pass into `12 dB/oct` Low-Pass per voice:
  - **HP Cutoff (`KNOB 1`) & LP Cutoff (`KNOB 3`):** `2.0 Hz` to `28,000 Hz` (`2.0 Hz` HP and `28 kHz` LP sit flat at `0%` resonance).
  - **HP Resonance (`KNOB 2`) & LP Resonance (`KNOB 4`):** `0-100%`. `0-60%` applies diode-limited resonance, and `60-100%` drives feedback clipping and self-oscillation across the frequency range.
- **Level & Pre-Filter Overdrive (`EDIT -> LEVEL`, `0-100%`):** Two-range gain curve (`65%` default). `0-70%` sets clean post-filter level (`0%` mutes the voice unless a `LEVEL` LFO is active). `70-100%` applies `1.0x` to `4.5x` asymmetric pre-filter drive into the filter input.
- **Stereo Pan (`EDIT -> PAN`):** Equal-power stereo pan from `L100` through `C` to `R100` (summed to `L == R` when `Output Routing` is `MONO`).

---

## 3. Front-Panel Controls & Layout

Reference faceplate photo: [`docs/assets/fm1-front-panel.png`](assets/fm1-front-panel.png).

```text
          LEFT ZONE                       CENTER                      RIGHT ZONE
┌───────────────────────────┐      ┌──────────────────┐      ┌──────────────────────────┐
│ MASTER          SELECT    │      │                  │      │ KNOB1  KNOB2  KNOB3 KNOB4│
│ PRESETS         ALGORITHM │      │   240x240 IPS    │      │ [FX] [SEL][ENV][LFO][EDIT│
│ [OCT-] [OCT+]             │      │     DISPLAY      │      │ [HOME][SAVE][ARP]...[PLAY│
├───────────────────────────┴──────┴──────────────────┴──────┼──────────────────────────┤
│ Left 20 Analog Pressure Keys (OP1-GLO + White Keys 1-12)   │ Right 7 Utility Switches │
│ Swappable: 1/3 LIVE MIXER · 2/3 CIRCUIT LAB · 3/3 RHYTHM   │ LATCH · TAP · SHIFT      │
│                                                            │ MUTE / SOLO 1-4          │
└────────────────────────────────────────────────────────────┴──────────────────────────┘
```

### 3.1 Top-Right Encoders (`KNOB 1-4`)
At idle, on `HOME (FREQUENCY)`, on `SEL`, on `GLOBAL CONFIG`, and during keybed play, `KNOB 1-4` control **`Osc 1-4 Frequency`**. Opening `HOME (DUTY / DRIFT)`, `FX`, `EDIT`, or `LFO` maps `KNOB 1-4` to that page's parameters while open.

### 3.2 Top-Left Controls & Rate Modifiers
- **`MASTER` (Pot):** Hardware master output volume.
- **`ALGORITHM` (Encoder):**
  - **Unshifted (Power-Rail Coupling, `0-100%`, `25%` default):** `0-45%` applies shared-rail phase pulling between voices, and `45-100%` adds rail sag, duty skew, and subharmonic hard sync. Turning it from `HOME` opens the `RAIL COUPLING` view for `2.2 s`; on other open pages it updates the top bar.
  - **`SHIFT + ALGORITHM` (Master Tempo, `20-3,000 BPM`, `120 BPM` default):** Adjusts the logarithmic internal clock (`OCT-` for `0.1 BPM` steps, `OCT+` for fast turns; displays `BPM EXT` when locked to external MIDI Clock).
  - **Inside `GLOBAL CONFIG`:** Steps the selected row's value (`1` step/detent) or enters sub-screens clockwise.
- **`SELECT` (Encoder):** Stepped selector (`1` item/detent):
  - **Idle Scope:** Adjusts horizontal timebase zoom (`4X`, `2X`, `AUTO`, `1/2X`, `1/4X`, `1/8X`).
  - **`HOME` (latched):** Steps across `FREQUENCY`, `DUTY`, and `DRIFT`.
  - **`FX`:** Selects `OSC 1-4`.
  - **`LFO`:** Selects `LFO A1-B4` (`8` slots).
  - **`EDIT (PAN)`:** Toggles `STEREO` and `MONO` output routing.
  - **`SEL`:** Steps across `1/3 LIVE MIXER`, `2/3 CIRCUIT LAB`, and `3/3 RHYTHM LAB`.
  - **`SAVE`:** Switches between `STORE SCENE` and `DELETE SCENE`.
  - **`GLOBAL CONFIG`:** Moves the row highlight up and down.
- **`PRESETS` (Encoder):** Continuous `360°` circular morph across `Scene 1-4` (`1 -> 2 -> 3 -> 4 -> 1`). Empty slots are skipped, and manual knob edits crossfade onto the loop without jumps (`OCT-` for fine morphs, `OCT+` for whole-scene steps).
- **`OCT-` / `OCT+` (Momentary Rate Modifiers):** Hold `OCT-` for `FINE` resolution or `OCT+` for `COARSE` range on any continuous encoder. Releasing the button returns to `STANDARD` rate.

### 3.3 Right `2x6` Function Buttons
- **`PLAY/STOP`:** Boots off; toggles audio output on and off with a short fade and pauses live visual and LED modulation while stopped (`Green` LED when off, `Red` LED when on).
- **`HOME`:** Returns to the clean scope in one tap from any open page, modifier, or toast. From the clean scope, repeated taps cycle `FREQUENCY -> DUTY -> DRIFT -> Clean Scope` (`LED` is `100%` on all Home states and `15%` when another page is open).
- **`SAVE`:** Two-step scene manager (`LED` is off on an unmodified scene, pulses slowly when live edits exist, and blinks while open):
  - `1st tap` (`STORE SCENE`): Blinks all 4 scene keys (`Keys 9-12`); tap one to store.
  - `2nd tap` or `SELECT` (`DELETE SCENE`): Blinks occupied scene keys at double speed; tap one to clear it to `EMPTY`.
  - `3rd tap` or `HOME`: Exits without changes.
- **`FX`:** Opens the **Dual Filter** page; repeated taps cycle `OSC 1 -> 2 -> 3 -> 4`.
- **`SEL`:** Opens the **Keyboard Explorer**; repeated taps cycle `1/3 LIVE MIXER -> 2/3 CIRCUIT LAB -> 3/3 RHYTHM LAB`. Holding an active key while `SEL` is open shows its parameter name, live pressure bar, and a 1-line description.
- **`EDIT`:** Opens the **Mixer** page; repeated taps toggle `LEVEL <-> PAN`.
- **`LFO`:** Opens the **LFO** page (`KNOB 1 = TARGET`, `KNOB 2 = SHAPE`, `KNOB 3 = RATE`, `KNOB 4 = DEPTH`); repeated taps cycle `LFO A1 -> B1 -> A2 -> B2 -> A3 -> B3 -> A4 -> B4`:
  - **Shared Destinations (`A` & `B`):** `FREQ`, `DUTY`, `LPF`, `LP RES`, `HPF`, `HP RES`, `LEVEL`, `PAN` (summed when `A` and `B` share a target).
  - **Cross-Modulation (`B` only):** `A1-A4 RATE` (`±6 oct`) and `A1-A4 DEPTH`.
  - **Shape (`0-100%`):** Continuous morph across `Sine -> Triangle -> Sawtooth -> Square -> Smooth Random`.
  - **Rate:** `0.01 Hz` to `2,000 Hz`.
- **`GLO`:** Opens and closes **`GLOBAL CONFIG`**:
  1. **`Output Routing`:** `STEREO` / `MONO`.
  2. **`Key Sensitivity`:** `VERY HIGH` / `HIGH` / `MEDIUM` *(default)* / `LOW` / `VERY LOW`. Pressing any of the 20 pressure keys while this row is selected shows a live pressure meter on the row.
  3. **`Mute Mode`:** `MOMENTARY` *(default)* / `LATCHING` (applies to `MUTE 1-4` and `SOLO 1-4`).
  4. **`Scene Mode`:** `LATCHING` *(default)* / `MOMENTARY` (`LATCHING` stays on the recalled scene; `MOMENTARY` morphs back on key release).
  5. **`MIDI Clock In`:** `AUTO` *(default, syncs `30-300 BPM` `24 PPQN` clock over 3.5 mm TRS and USB-C MIDI)* / `OFF`.
  6. **`MIDI Transport In`:** `OFF` *(default)* / `PHASE` *(resets bar phase on Start)* / `RUN/STOP` *(starts and mutes audio with MIDI Start/Continue/Stop)*.
  7. **`MIDI Channel In`:** `OMNI` *(default)* / `CH 1-16`.
  8. **`Factory Reset`:** Two-detent confirmation on `ALGORITHM` (`CCW` cancels) for **`Factory Presets`** (restores default sound, all 4 scenes, and config) or **`Clean Init`** (loads an unmodulated startup sound, clears scenes to `EMPTY`, and keeps config).
  9. **`About`:** Displays version, instrument summary, author, license, and repository URL.
- **`ENV`, `ARP`, `SEQ`, `REC`:** Unassigned (`0%` LED at rest). A press nudges the scope horizontally and flashes the button LED with no change to audio or open pages.

---

## 4. `20 + 7` Keybed & `SHIFT` Layer

### 4.1 Hardware Split
1. **Left `20` Analog Pressure Keys (`OP1-GLO` + `Keys 1-12`):** Respond continuously to finger force across the 3 keyboard layouts (`LIVE MIXER`, `CIRCUIT LAB`, `RHYTHM LAB`) and return to the resting patch on release.
2. **Right `7` Utility Switches:**
   - **`MONO` (`LATCH`):** Holds or releases pressed left-bay pressure pads hands-free (blinks while active; clears on keyboard change).
   - **`POLY` (`TAP`):** Active in `RHYTHM LAB` for beat pulse indication, tap tempo (`20-3,000 BPM`), and Beat-1 phase alignment.
   - **`Keys 13-16` (`MUTE 1-4`):** Momentary or latching voice mutes (`LED` indicates steady voice state: `80%` audible, `50%` sub-audio, `0%` silent or muted).
   - **Top-Right Corner Key (`SHIFT`):** Lit at `100%` at all times. Holding `SHIFT` displays the full-chassis `SHIFT` map and enables:
     - `SHIFT + ALGORITHM`: Master Tempo (`20-3,000 BPM`)
     - `SHIFT + SELECT` or `SHIFT + OP1-OP3`: Switch keyboard layout (`1-3`)
     - `SHIFT + Keys 9-12`: Recall or pressure-morph `Scene 1-4` (holding multiple scene keys blends them by finger pressure)
     - `SHIFT + Keys 13-16`: Momentary or latching `SOLO 1-4`

### 4.2 Mode 1 (`1/3`): `LIVE MIXER` *(Default)*
- **Top `8` Global Macros:**
  - `OP1` (`DRIVE`): Increases pre-filter overdrive, power-rail coupling, and filter resonance across all 4 voices.
  - `OP2` (`SUB OCTAVE`): Lowers all 4 voices continuously down through `-1` to `-2` octaves.
  - `OP3` (`RATIO LOCK`): Pulls `OSC 2-4` into harmonic frequency ratios relative to `OSC 1` and damps pitch drift.
  - `OP4` (`LOW PASS`) / `OP5` (`HIGH PASS`): Global resonant low-pass (`28 kHz -> 31 Hz`) and high-pass (`2 Hz -> 4.2 kHz`) filter sweeps.
  - `OP6` (`UNISON`) / `PIT` (`DETUNE`) / `GLO` (`SPREAD`): Pulls voices to `OSC 1` unison (`OP6`), adds chorus detune (`PIT`), or spreads detune, pulse width, and stereo pan (`GLO`).
- **Bottom `12` Per-Voice Pads (`Osc 1-4` per zone):**
  - `Keys 1-4` (`FILTER 1-4`): Sweeps dual resonant formant peaks on active voices, or brings in silent/muted voices through a low-pass gate.
  - `Keys 5-8` (`OCTAVE 1-4`): Raises individual voices continuously through `+1` to `+2` octaves.
  - `Keys 9-12` (`DUTY 1-4`): Narrows pulse width toward `6%`.

### 4.3 Mode 2 (`2/3`): `CIRCUIT LAB`
- **Top `8` Global Macros:**
  - `OP1` (`STARVE`): Increases shared-rail coupling, sags pitch, and skews pulse widths.
  - `OP2` (`RING MOD`): Multiplies adjacent voice pairs (`1x2`, `2x3`, `3x4`, `4x1`) for ring and XOR sidebands.
  - `OP3` (`TURBO LFO`): Multiplies all 8 LFO rates up to `32x` into the audio range.
  - `OP4` (`SCREAM`) / `OP5` (`HARD SYNC`): Drives dual filters into self-oscillation (`OP4`) or phase-locks `OSC 2-4` to `OSC 1` (`OP5`).
  - `OP6` (`SCATTER`) / `PIT` (`SUB-XOR`) / `GLO` (`FEEDBACK`): Offsets pitch, duty, and pan (`OP6`), applies `/2, /4, /8` sub-octave XOR (`PIT`), or routes the master bus back into overdrive and rail coupling (`GLO`).
- **Bottom `12` Per-Voice Pads (`Osc 1-4` per zone):**
  - `Keys 1-4` (`CROSS-PIN 1-4`): Routes the neighboring voice's capacitor ramp into pitch FM, comparator XOR, and filter resonance.
  - `Keys 5-8` (`FOLD 1-4`): Routes the voice's internal triangle ramp through a `1.0x-6.2x` sinusoidal wavefolder.
  - `Keys 9-12` (`CRUSH 1-4`): Applies sub-octave XOR and sample-and-hold clock reduction (`16 kHz -> 165 Hz`).

### 4.4 Mode 3 (`3/3`): `RHYTHM LAB`
Uses the internal `20-3,000 BPM` clock (`SHIFT + ALGORITHM`), live `TAP` (`POLY`), or external `24 PPQN` MIDI Clock:
- **Top `8` Global Macros:**
  - `OP1` (`PUMP`): Quarter-note ducking envelope (`8/bar` at high pressure).
  - `OP2` (`CHASE`): Round-robin `1 -> 2 -> 3 -> 4` voice gate (`16th` or `32nd` notes).
  - `OP3` (`SPEED`): Multiplies master clock rate `1.0x -> 4.0x`.
  - `OP4` (`PLUCK`) / `OP5` (`SHORT`): Shapes active gates into resonant low-pass filter plucks (`OP4`) or short clicks (`OP5`).
  - `OP6` (`SHUFFLE`) / `PIT` (`PING-PONG`) / `GLO` (`ROLL`): Adds off-beat timing shuffle (`OP6`), L/R stereo gate alternation (`PIT`), or `x2/x4/x8` gate subdivisions (`GLO`).
- **Bottom `12` Per-Voice Pads (`Osc 1-4` per zone):**
  - `Keys 1-4` (`STUTTER 1-4`): Unsynced per-voice gate (`1.5-24 Hz`).
  - `Keys 5-8` (`DIVIDE 1-4`): Synced clock divisions and multiples (`/2, X1, X2, X3, X4, X8`), offset across voices for polyrhythms.
  - `Keys 9-12` (`EUCLID 1-4`): 16-step Euclidean gate patterns (`2/16` to `16/16`), phase-offset across voices.

---

## 5. Display, HUD & LED Feedback

- **Monochrome Oscilloscope:** Black-and-white waveform display of all four voices and the stereo mix at a constant vertical scale. When audio stops, the trace settles to a flat center line.
- **Three-Zone Screen Layout:**
  - **Top Bar:** Displays active parameter titles, voice/slot qualifiers, `FINE` / `COARSE` badges, 4-dot scene morph indicators, and live values. Off-page adjustments on a pinned page appear as `1.5 s` top-bar toasts and leave the open view in place.
  - **Center View:** Displays the live waveform, the active parameter graph (`FX` filter response, `LFO` waveform, `LEVEL` bars and overdrive cap, `PAN` stereo field, `DUTY` / `DRIFT` overlays, `RAIL COUPLING`), the `GLOBAL CONFIG` list, or the `SEL` / `SHIFT` / `SAVE` keybed diagrams.
  - **Bottom Strip:** Displays `KNOB 1-4` gauges, numerical values, and per-voice `FX` / `LFO A` / `LFO B` status dots on parameter pages, or 1-line control descriptions on keybed views.
- **Transitions & System Screens:** All overlays fade in and out over `150 ms`. System screens handle USB firmware updates (`Scenes 1-4` preserved), `Hold OCT-` USB rescue mode, and fault diagnostics.
- **42-LED Brightness Levels:** LEDs turn off (`0%`) on unassigned controls and muted voices, sit at `15%` on available controls at rest, pulse slowly when `FX`, `LFO`, or unsaved `SAVE` state is active in the background, use three brightness groups across the 20 pressure keys, and light to `100%` on the active page or held control.

---

## 6. Explicit Non-Goals

Ruido excludes:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI note polyphony or voice-stealing envelopes.
- Built-in time-based effects (reverb, delay, chorus): Ruido is a dry sound source for external pedals and modular gear.
- Nested menus or file browsers.
