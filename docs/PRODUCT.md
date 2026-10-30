# Ruido — Product & UX Specification

## 1. Vision & Musical Identity

**Ruido** transforms the pocket **M-VAVE FM-1** hardware into a dedicated, boutique **4-oscillator CMOS Schmitt-trigger stereo drone instrument**.

- **Primary Acoustic Reference:** Classic DIY analog Schmitt-trigger relaxation oscillator synths (`CD40106` / `74HC14` style)—organic capacitor tilt, micro-thermal drift, and shared power-rail phase pulling—paired with creamy, aggressive **Korg MS-20 `Rev 2` (`KLM-307` `LM13600` OTA Sallen-Key) dual `HPF -> LPF` silicon-limited resonant filters**.
- **Primary Studio Role:** A pristine, harmonically rich, zero-aliasing stereo sound source designed specifically to feed **external guitar/synth effects pedals** (fuzz, wavefolders, BBD delays, reverbs, granular) and **Eurorack modular rigs**.

### Core Design Values (In Priority Order)
1. **Great Sound:** Spend every spare CPU cycle on analog-faithful oscillator physics, `4x` oversampling, nonlinear Zero-Delay Feedback filtering, and clickless slewing.
2. **Immediate, Ergonomic UX with Judicious Pages:** Default idle state is always live waveform + `KNOB1-4 = Osc 1-4 Frequency`, with momentary holds strictly on the left/bottom (`OP1-OP6`, `GLO`, `OCT-/OCT+`) so forearms never cross over the display, and latching toggles / judicious focused pages on the right-side `2x6` button grid (e.g. `LFO` modulation and a future `GLO` configuration menu, always one tap from `HOME`).
3. **Useful Waveform Visuals:** An always-on oscilloscope that serves as an actual acoustic tuning and gain-staging instrument, with minimal controls gently fading in over the wave only while touched or viewing a focused page.

---

## 2. Voice & Signal Architecture (User-Facing)

Ruido consists of **4 identical mono voices** (`Osc 1..4`) mixed in stereo through a subtle nonlinear master bus:

```text
[Schmitt Square Osc 1..4] ──> [MS-20 Dual HPF -> LPF Filter 1..4] ──> (Amp + Gate + Pan) ──> [Master Bus Sag] ──> Stereo L/R Out
          ▲                                                                                         │
          └───────────── Shared Power-Rail Coupling (`ALGORITHM` Knob: 0-100% Circuit Sag) ─────────┘
```

### Per-Oscillator Controls (`Osc 1..4`)
- **Frequency:** Continuous stepless sweep across **Audio Range** (`20 Hz - 4,000 Hz`), using **strictly linear 1:1 X4 quadrature scaling (`96 steps/turn`, zero velocity acceleration)** + **`C^1` critically-damped log-octave smoothing**:
  - *`FINE` (`Hold OCT-`):* Surgical `~0.66 cents/step` (`~63 cents/turn`) precision for locking slow phase beating and harmonic intervals.
  - *`STANDARD` (Released Default):* Predictable linear `~2.8`-turn full sweep regardless of hand speed.
  - *`FAST` (`Hold OCT+`):* Full end-to-end `20 Hz - 4,000 Hz` sweep in **`~1.33` turns**.
- **Amplitude:** Continuous post-filter VCA level (`0-100%`).
- **Panning:** Equal-power (`sin`/`cos`) stereo placement from hard Left (`L100`) through Center (`C`) to hard Right (`R100`), or collapsed to Dual Mono (`L == R`) when `MONO` output routing is active.
- **Dual MS-20 `Rev 2` OTA Filter (`HPF -> LPF` in series with Independent Peak Resonance):**
  - **HP Cutoff (`KNOB1`):** `20 Hz` (parked flat/transparent) up to `20 kHz`, engaging warm `LM13600` OTA saturation across all 4 filter poles even at `0%` resonance.
  - **HP Resonance (`KNOB2`):** Independent `Rev 2` OTA + `1S1588` silicon High-Pass Peak (`0-100%`, `k` up to `3.68`) with **Tight Smart-Park Taper**—rests flat (`Q = 0.7071`) when `HPF` is parked at `20 Hz`, wakes up immediately past `24 Hz`, boosts sub-bass fundamentals cleanly with organic capacitive `0 Hz` DC rejection, and enters pitch-tracked self-oscillation by `~35%`.
  - **LP Cutoff (`KNOB3`):** `20 Hz` up to `20 kHz` (parked flat/wide-open), rounding square edges with creamy `LM13600` OTA 2nd/3rd harmonic warmth as soon as engaged.
  - **LP Resonance (`KNOB4`):** Independent `Rev 2` OTA + `1S1588` silicon Low-Pass Peak (`0-100%`, `k` up to `3.68`) with **Tight Smart-Park Taper**—rests flat at `20 kHz`, wakes up below `18.8 kHz`, enters sustained self-oscillation by `~35%`, and pairs with `HP RES` for asymmetrical vocal formants or quench-free simultaneous dual-sine self-oscillation on a single voice.

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
1. **Strict Left-Hand Momentary Holds:** Every momentary `"Hold + Turn KNOB1-4"` gesture lives in the **bottom-left / bottom-center black keys** (`OP1-OP6`, `GLO`) and left-hand **`OCT-` / `OCT+`**. The left hand stays down-left, the right hand stays top-right on `KNOB1-4`, and the center `240x240` screen is never occluded by a crossed forearm.
2. **Right-Side `2x6` Button Grid = Non-Momentary Toggles & Judicious Pages:** None of the `2x6` buttons directly below `KNOB1-4` are momentary holds; they act as latching toggles, immediate utilities, or focused page switches (such as `LFO` modulation and a future `GLO` configuration menu).

### 3.1 Top-Right Encoders: `KNOB1` – `KNOB4` (Approach A: Always Oscillators 1–4)

| Left-Hand Hold (Black Key) | `KNOB1` | `KNOB2` | `KNOB3` | `KNOB4` | Monochrome HUD Header + Bottom 4-Knob Strip |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **None (Idle Default)** | **Osc 1 Freq** | **Osc 2 Freq** | **Osc 3 Freq** | **Osc 4 Freq** | `FREQUENCY` + 4 `Hz` bars |
| **Hold `OP1`..`OP4`** *(Lit `15%`)* | **HPF Cutoff** | **HP Resonance** | **LPF Cutoff** | **LP Resonance** | `FILTER OSC 1..4` + frameless MS-20 curve + `HPF/HP RES/LPF/LP RES` |
| **Hold `OP5`** *(Lit `15%`)* | **All HPF** | **All HP Res** | **All LPF** | **All LP Res** | `FILTER GLOBAL` + frameless MS-20 curve + `HPF/HP RES/LPF/LP RES` |
| **Hold `OP6`** *(Lit `25%`)* | **Osc 1 Pan** | **Osc 2 Pan** | **Osc 3 Pan** | **Osc 4 Pan** | `PANNING` (`EQUAL POWER`) + 4 bipolar `L..C..R` bars |
| **Hold `GLO`** *(Lit `25%`)* | **Osc 1 Amp** | **Osc 2 Amp** | **Osc 3 Amp** | **Osc 4 Amp** | `AMPLITUDE` (`POST-FILTER VCA`) + 4 `%` level bars (`0%` if muted; turning up fades in from `0%` & unmutes) |

### 3.2 Top-Left Knobs & `OCT-` / `OCT+` (Zero-Modifier Global Controls)
- **`MASTER` (Analog Pot):** Hardware master output level (`MASTER LEVEL` + `%` bar on HUD).
- **`ALGORITHM` (Encoder):** **Rail Coupling** (`0%` isolated clean circuits <-> `100%` heavy `CD40106` shared-rail threshold pulling & cross-modulation). *Mnemonic: defines how the 4 oscillators electrically couple, analogous to FM operator algorithms.*
- **`SELECT` (Encoder):** Unassigned / reserved for focused page & menu navigation (`SELECT ENCODER · UNASSIGNED` on turn).
- **`PRESETS` (Encoder):** **Snapshot Morph (Continuous Macro Scrub)** — smoothly scrubs/crossfades across saved snapshots (`SNAPSHOT MORPH` + `%` bar on HUD) while panning varibright LED brightness across **`White Keys 9–16`** (and morphed voice levels on **`White Keys 1–4`**) in real time so the physical keybed continuously shows the exact hybrid blend between slots.
- **`OCT-` / `OCT+` (Momentary Rate Modifiers):** **Strictly momentary left-hand encoder rate modifiers (`KNOB RATE`)** governing all 7 endless rotary encoders (`KNOB 1-4`, `ALGORITHM`, `SELECT`, `PRESETS`) with **zero velocity acceleration** (100% deterministic 1:1 angle-to-parameter response at `96` X4 quadrature transitions per `360°` turn), freely chordable alongside `OP1-OP6` and `GLO`:
  - *Released (Idle Default = `STANDARD`):* Both `OCT-` and `OCT+` rest at `15%` dim glow; full parameter sweeps take an exact linear **`2.5-2.8` turns**.
  - *Hold `OCT-` -> `FINE` (`OCT-` LED `100%`):* Surgical fine-tuning while held (`0.66 cents/step` audio frequency, `3.0 cents/step` filter cutoff, `0.09%/step` linear controls).
  - *Hold `OCT+` -> `FAST` (`OCT+` LED `100%`):* Rapid performance sweep while held—covering the **entire `0-100%` / `20 Hz-4 kHz` / `20 Hz-20 kHz` sweep in `~1.3` turns** (`125-128` X4 steps).
  - *Release:* Immediately returns to `STANDARD` rate (`15%` rest glow) and begins smooth HUD dissolve with zero latched state.

### 3.3 Right-Panel `2x6` Function Buttons (Latching Toggles & Focused Pages)
- **`SEL`:** **Pressure Pad Mode Select (`PRESSURE TARGET`)** — toggles **White Keys 5–8** between **MS-20 Filter Accent / Vactrol LPG** (default) and **Pitch Bend**. To connect `SEL` visually to `Keys 5-8` on monochrome white LEDs without ever looking "off", **`SEL` and all four `Keys 5-8` share identical PWM brightness in both modes** (`24%` soft glow in Filter/LPG mode <-> `72%` bright glow in Pitch Bend mode, stepping in unison on tap, plus `SEL` flaring toward `100%` alongside any pressed pad).
- **`PLAY/STOP`:** **Audio Output** toggle (`ACTIVE` / `MUTED`, clickless fade to silence and back for safe cable patching).
- **`HOME`:** **Exit Page / Clear HUD to Pure Scope** (tap dismisses any active overlay immediately).
- **`SAVE`:** Tap `SAVE` (`STORE SNAPSHOT` / `TAP KEY 9-16`) then tap a Snapshot white key (`9-16`) to store the current patch.
- **Unassigned / Reserved Switches (`FX`, `ENV`, `LFO`, `EDIT`, `GLO` under knobs, `ARP`, `SEQ`, `REC`):** LEDs stay **`0%` completely unlit** at rest while unassigned (`LFO` is specified in `ADR-022` below for upcoming implementation, and `GLO` under the knobs is reserved for a future Configuration Menu where `Stereo / Dual Mono` routing will live); pressing any unassigned switch briefly displays its silkscreen name + `UNASSIGNED` in the HUD header so every control confirms hardware contact.

### 3.4 Per-Voice LFO Page (`LFO OSC 1..4` / `LFO GLOBAL` — `ADR-022`)
Each of the 4 voices (`Osc 1..4`) owns **1 dedicated LFO** matching its dedicated MS-20 filter, VCA, and panner:
- **Enter / Exit:** Tap **`LFO`** (right-grid switch) to open the LFO Page (`LFO OSC 1..4` / `LFO GLOBAL`); tap **`LFO`** again or **`HOME`** to return to the pure scope while active LFOs continue running.
- **`SELECT` Encoder (Left Hand):** Steps across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4` · `ALL 4`**.
- **`KNOB 1–4` (`TARGET | SHAPE | RATE | DEPTH` — Setup Left, Perform Right):**
  - **`KNOB 1` (`TARGET`):** Stepped per-voice parameter selector: **`LPF`** *(default)* · **`LP RES`** · **`HPF`** · **`HP RES`** · **`AMP`** · **`PITCH`** · **`PAN`**.
  - **`KNOB 2` (`SHAPE`):** Continuous `0–100%` seamless wave morph across **`Sine` (`0%`) -> `Triangle` (`25%`) -> `Sawtooth` (`50%`) -> `Slewed Square` (`75%`) -> `Smooth Drift` (`100%`, non-repeating organic wander)**.
  - **`KNOB 3` (`RATE`):** Continuous `0.02 Hz` (`50 s` cycle) to `30 Hz` log-octave sweep.
  - **`KNOB 4` (`DEPTH`):** Unipolar `0–100%` modulation amount with a hard stop at `0%` CCW so any LFO zeros cleanly by feel.
- **Live Full-Screen LFO Visualization:** Spans edge-to-edge across the entire `240 px` display (`x = 0..239`) and fills the full vertical viewport (`y = 26..188`, `±75 px` / `150 px` peak-to-peak centered on a `y = 107` zero-horizon filament) over a `72%`-dimmed background scope, with clean gradient shading (zero decorative inner contour lines) and a crisp `#FFFFFF` optical phase bead proportioned to match the MS-20 filter cutoff beads (or overlaying all 4 voices' LFO curves & beads when `SELECT` is on `ALL 4`).

### 3.5 Bottom 27-Key Surface & Carbon-Pill Pressure Expression (`SARADC`)

Using analog `SARADC` sampling of the silicone keybed's carbon-pill contact resistance, Ruido supports continuous **Soma Lyra-8-style finger-lean pressure expression** alongside tactile switching, and **every single key displays live technical feedback on the HUD**:

#### Top Row — 11 Raised Black Keys (Silkscreen-Matched, Strictly Momentary Modifiers)
- **`OP1` · `OP2` · `OP3` · `OP4` · `OP5`:** Contiguous 5-key **Filter Modifier Family** (`15%` dim rest glow, `100%` while held): `OP1–OP4` focus `Osc 1-4 Filter`, and `OP5` sweeps **Global Filter** across all 4 voices simultaneously. Releasing any momentary modifier immediately exits the filter overlay and smoothly dissolves back to pure scope.
- **`OP6` (`PAN`) · `GLO` (`AMP`):** Mix / spatial momentary holds for `Osc 1-4 Panning` (`OP6`) and `Osc 1-4 Amplitude` (`GLO`) (`25%` steady landmark glow, `100%` while held).
- **`PIT`:** Unassigned in v1 (`0%` dark LED at rest; displays `PIT KEY · UNASSIGNED` if tapped—keeping the bottom black-key modifier row `100%` momentary).
- **`MONO` · `POLY`:** Instant `OUTPUT ROUTING` toggle between **Dual Mono (`MONO`)** (for mono pedalboards) and **Stereo (`POLY`)** (for stereo pedals / dual Eurorack channels).
- **11th Tall Black Key (`F#5`):** **Momentary Scope Freeze** (`SCOPE FREEZE` / `HOLDING`; LED `0%` at rest, `100%` while freezing the coupled 4-voice caustic ribbon mesh and optical bloom).

#### Bottom Row — 16 White Keys (4 Silkscreen-Dotted Groups of 4)
- **Dot 1 (`White Keys 1-4`): Osc 1-4 Latching Gates**
  - Clean tap toggles `Osc 1-4` gate (`GATE OSC 1..4`: `ACTIVE` / `MUTED` with `2-5 ms` clickless vactrol fade), remembering the oscillator's pre-mute amplitude when toggled back on. Holding **`GLO`** (`Mixer`) displays **`0%`** for muted oscillators (without erasing their remembered level unless turned) and turning their `KNOB 1-4` clockwise fades them in smoothly from `0%` while clearing their mute state.
  - LEDs glow at `10%` ember when muted and `60-100%` scaled by amplitude when active.
- **Dot 2 (`White Keys 5-8`): Osc 1-4 Pressure Expression Pads (Brightness-Coupled to `SEL`)**
  - Visually linked to **`SEL`** via matched PWM rest brightness (`24%` in Filter/LPG mode, `72%` in Pitch Bend mode) and synchronized `-> 100%` touch flare.
  - **Filter / LPG Mode (`SEL` at `24%`, Default — `0.000 Hz` Pitch Shift):**
    - Displays `FILTER EXP OSC 1..4` (or `LPG SWELL OSC 1..4` when muted) + live `0-100%` bar and the live pressure-modulated MS-20 passband curve on the HUD.
    - *When Osc is Active:* **Lean harder** to sweep its **MS-20 Filter + Dual HP/LP Resonance** (opens `LPF` upward when darkened, or sweeps a resonant `HPF -> LPF` formant peak across harmonics when `LPF` is parked wide open, decaying via a `26 ms` CdS vactrol tail on release).
    - *When Osc is Muted:* **Press & lean harder** to swell that oscillator in from silence through a **Buchla-style Vactrol Low-Pass Gate (`LPG`)** (`95 Hz` sub murmur -> full harmonic roar).
  - **Pitch Bend Mode (`SEL` at `72%`):**
    - Displays `PITCH BEND OSC 1..4` + live `0-100%` bar while updating the live bent `Hz` readout in the bottom strip; leans bend pitch upward up to `+6` semitones (`4.5 ms` attack / `26 ms` CdS vactrol glide back to the knob frequency on release) and gently open the VCA gate if muted.
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
  - **Open Full-Span Geometry (`x = 0..239`, constant `y = 120` center & `78 px` scale) & Zero Hard-Cut Transitions:** Zero artificial edge pinching to `0 V` at the left/right bezels, **zero vertical compression or shift** when the HUD fades in, and **zero 1-frame visual snaps** anywhere on the instrument—stopping audio (`PLAY/STOP`, `MASTER = 0%`, muting all voices, or pausing audio) smoothly relaxes all 14 strands + bloom into a poised `y = 120` horizon flatline and blooms back out on resume, while per-voice mutes, `1x <-> 2x` bass timebase zoom, filter curves, optical beads, and HUD overlays all transition continuously.
- **Unified Soft-Transitioning Optical-Filament HUD:**
  - **Top Function Header (`13 pt` Bold Two-Tone Hierarchy & Top-Aligned Metadata):** Reserved top strip (`y = 6..22`) leading with the active function title (`FILTER`, `LFO`, `GATE`, `FILTER EXP`, `LPG SWELL`, `PITCH BEND`) in `100%`-white `13 pt` Bold `Helvetica Neue` (`kFontSansBold`, matching the `13 pt` numeric readout scale), followed by dimmed trailing target qualifiers (`OSC 1..4`, `GLOBAL` at `~46%` silver) alongside pixel-exact top-aligned (`y = 9`) right-hand `10 pt` metadata or `13 pt` `%` + tapered optical-filament gauges (paired with idle top-right `L/R` optical-filament peak meters when the HUD is closed).
  - **Bottom 4-Knob Strip & `1/3`-Screen (`80 px`) Dual-Resonance Filter Graph:** Reserved bottom strip (`y = 190..235`) **always** showing what `KNOB 1..4` currently control (`OSC 1..4` or `HPF / HP RES / LPF / LP RES`), 4 tapered sub-pixel optical filaments with glowing focal beads (including center-slit bipolar `L..C..R` pan gauges), and tabular sans-serif readouts (`55.0 Hz`, `2.4 kHz`, `65%`, `L35`). Holding `OP1..5` (or pressing `Keys 5..8` in Filter/LPG mode) blooms a `1/3`-screen (`y = 102..182`, `80 px` tall, `x = 10..230`) **Option A3** dual-peak MS-20 transfer graph—combining a `10%` solid passband silhouette down to a `y = 182` horizon filament, a `30%` caustic crest glow, incandescent linear-photon curve + MS-20 companion contours, and independent `HP`/`LP` cutoff focal beads (`0%` artificial voice offset on `OP5`)—while smoothly dimming the background waveform by `70%` (`~110 ms` attack / `~240 ms` release S-curve transition).
  - **Soft S-Curve Envelope, Latched Right-Grid Switches & Outgoing-View Latch:** Right `2x6` switches (`SEL`, `PLAY/STOP`, `SAVE` while armed, `LFO`, `FX`, `GLO` right, etc.) keep their HUD **latched on screen** until `HOME` is pressed (smoothly returning to the latched switch HUD after any momentary hold or standalone knob tweak). Momentary holds (`OP1–OP6`, `GLO` black key, `OCT-/+`, `ScopeFreezeKey`, `Keys 5–8`) stay pinned while held and dissolve immediately on release (latching outgoing labels so `FREQUENCY` never flashes during fade-out), standalone knob turns and keybed taps hold `1.5 s` before a `~850 ms` dissolve, and `HOME` performs a `~150 ms` quick-dissolve back to pure scope.

---

## 5. Explicit Non-Goals (v1 Scope Guardrails)

To keep Ruido focused, reliable, and sonically uncompromising, **v1 explicitly excludes**:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI polyphony / voice-stealing envelopes.
- Onboard time-based FX (reverb, delay, chorus)—Ruido is the **dry analog-style source** feeding your external pedals and Eurorack modules.
- Deep nested sub-menus or file browsers (focused single-level pages on right-grid buttons such as `LFO` and `GLO` Config are used judiciously, always instant-exit via `HOME`).
