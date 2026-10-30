# Ruido — Product & UX Specification

## 1. Vision & Musical Identity

**Ruido** transforms the pocket **M-VAVE FM-1** hardware into a dedicated, boutique **4-oscillator CMOS Schmitt-trigger stereo drone instrument**.

- **Primary Acoustic Reference:** Classic DIY analog Schmitt-trigger relaxation oscillator synths (`CD40106` / `74HC14` style)—organic capacitor tilt, micro-thermal drift, and shared power-rail phase pulling—paired with creamy, aggressive **Korg MS-20 `Rev 2` (`KLM-307` `LM13600` OTA Sallen-Key) dual `HPF -> LPF` silicon-limited resonant filters**.
- **Primary Studio Role:** A pristine, harmonically rich, zero-aliasing stereo sound source designed specifically to feed **external guitar/synth effects pedals** (fuzz, wavefolders, BBD delays, reverbs, granular) and **Eurorack modular rigs**.

### Core Design Values (In Priority Order)
1. **Great Sound:** Spend every spare CPU cycle on analog-faithful oscillator physics, `4x` oversampling, nonlinear Zero-Delay Feedback filtering, and clickless slewing.
2. **Immediate, Ergonomic UX with Judicious Pages:** Default idle state is always live waveform + `KNOB1-4 = Osc 1-4 Frequency`, with momentary holds strictly on the left/bottom (`OP1-OP6`, `GLO`, `OCT-/OCT+`) so forearms never cross over the display, and latching toggles / judicious single-level pages on the right-side `2x6` button grid (`LFO` modulation and `GLO` `GLOBAL CONFIG`, always one tap from `HOME`).
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
  - *`COARSE` (`Hold OCT+`):* Full end-to-end `20 Hz - 4,000 Hz` sweep in **`~1.33` turns**.
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

### 3.1 Top-Right Encoders: `KNOB1` – `KNOB4` (Per-Oscillator Banks & 5-Mode `GLOBAL`)

| Left-Hand Hold (Black Key) | `KNOB1` | `KNOB2` | `KNOB3` | `KNOB4` | Monochrome HUD Header + Center Graph + Bottom 4-Knob Strip |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **None (Idle Default)** | **Osc 1 Freq** | **Osc 2 Freq** | **Osc 3 Freq** | **Osc 4 Freq** | `FREQUENCY OSC 1..4` (`XX.XX Hz` exact counter) + 4 `Hz` filaments |
| **Hold `OP1`..`OP4`** *(Lit `15%`)* | **HPF Cutoff** | **HP Resonance** | **LPF Cutoff** | **LP Resonance** | `FILTER OSC 1..4` (`12 dB HP -> LP`) + single-voice MS-20 passband + `HPF/HP RES/LPF/LP RES` |
| **Hold `OP5`** *(Lit `25%`)* | **Osc 1 Amp** | **Osc 2 Amp** | **Osc 3 Amp** | **Osc 4 Amp** | `AMPLITUDE` (`POST-FILTER VCA`) + 4 `%` level filaments (`0%` if muted; turning CW fades in from `0%` & unmutes) |
| **Hold `OP6`** *(Lit `25%`)* | **Osc 1 Pan** | **Osc 2 Pan** | **Osc 3 Pan** | **Osc 4 Pan** | `PANNING` (`STEREO` / `DUAL MONO`) + 4 bipolar `L..C..R` filaments |
| **Hold `GLO` (`SELECT = FREQ`)** *(Lit `25%`)* | **Pitch** | **Spread** | **Detune** | **Tilt** | `GLOBAL FREQ` + `20 Hz–4 kHz` log-frequency 4-voice spectral stems & beads + 4-voice spread micro-beads |
| **Hold `GLO` (`SELECT = FILTER`)** *(Lit `25%`)* | **All HPF** | **All HP Res** | **All LPF** | **All LP Res** | `GLOBAL FILTER` + 4-voice overlaid MS-20 passbands & beads + relative delta filaments |
| **Hold `GLO` (`SELECT = LFO`)** *(Lit `25%`)* | **All Target** | **All Shape** | **All Rate** | **All Depth** | `GLOBAL LFO` + full-screen 4-voice morphed LFO waves & 4 phase beacons (`MIXED` when diverged) |
| **Hold `GLO` (`SELECT = AMP`)** *(Lit `25%`)* | **Level** | **Tilt** | **Width** | **Balance** | `GLOBAL AMP` + `L..C..R` stereo soundstage horizon & 4 voice pillars |
| **Hold `GLO` (`SELECT = ALL`)** *(Lit `25%`, Default)* | **Freq** | **Filter** | **LFO** | **Amp** | `GLOBAL ALL` + unified `20 Hz–20 kHz` MS-20 passband overlaid with the 4 live oscillator pitch/level stems |

### 3.2 Top-Left Knobs & `OCT-` / `OCT+` (Zero-Modifier Global Controls)
- **`MASTER` (Analog Pot):** Hardware master output level (`MASTER LEVEL` + `%` bar on HUD, anchored with `> 0.2%` cumulative deadband so slow analog sweeps wake the HUD reliably without ADC chatter).
- **`ALGORITHM` (Encoder):** **Rail Coupling** (`0%` isolated clean circuits <-> `100%` heavy `CD40106` shared-rail threshold pulling & cross-modulation). *Mnemonic: defines how the 4 oscillators electrically couple, analogous to FM operator algorithms.*
- **`SELECT` (Encoder):** Context-sensitive discrete notched selector (`1` item per physical detent / `4` X4 ticks across all 3 encoder gears):
  - While holding keybed **`GLO`**: steps **`GLOBAL`** across **`FREQ` · `FILTER` · `LFO` · `AMP` · `ALL`** (defaults to `ALL` on boot and remembers your last selected destination).
  - When the **Per-Voice LFO Page** (`LFO`) is open: steps strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**.
  - When **`GLOBAL CONFIG`** (right-grid `GLO`) is open: moves the row cursor up/down the configuration list.
  - Outside those contexts: displays `SELECT ENCODER · UNASSIGNED` (`1.5 s` toast).
- **`PRESETS` (Encoder):** Reserved for **Snapshot Morph (Continuous Macro Scrub)** in Milestone 3 (currently displays `PRESETS ENCODER · UNASSIGNED` `1.5 s` toast until the 8-slot Snapshot Engine lands; once active, smoothly scrubs/crossfades across saved snapshots while panning varibright LED brightness across **`White Keys 9–16`** per `ADR-025`).
- **`OCT-` / `OCT+` (Momentary Rate Modifiers — Split Top-Bar Override):** **Strictly momentary left-hand encoder rate modifiers** governing continuous endless rotary sweeps with **zero velocity acceleration** (100% deterministic 1:1 angle-to-parameter response at `96` X4 quadrature transitions per `360°` turn), freely chordable alongside `OP1-OP6` and `GLO` (discrete notched selectors like `SELECT`, `LFO TARGET`, and `GLOBAL CONFIG` rows remain `1` item per detent across all 3 gears):
  - *Released (Idle Default = `STANDARD`):* Both `OCT-` and `OCT+` rest at `15%` dim glow; full continuous parameter sweeps take an exact linear **`2.5-2.8` turns**.
  - *Hold `OCT-` -> **`FINE`** (`OCT-` LED `100%`):* Surgical fine-tuning while held (`0.66 cents/step` audio frequency, `3.0 cents/step` filter cutoff, `0.09%/step` linear controls).
  - *Hold `OCT+` -> **`COARSE`** (`OCT+` LED `100%`):* Rapid performance sweep while held—covering the **entire `0-100%` / `20 Hz-4 kHz` / `20 Hz-20 kHz` sweep in `~1.3` turns** (`125-128` X4 steps).
  - *Split Top-Bar Override While Held:* For the entire duration `OCT-` or `OCT+` is physically held, the top-left HUD header stays locked to **`FINE`** or **`COARSE`**, while the top-right slot updates live with a high-precision readout of whichever knob is turned (`OSC 1  110.04 Hz`, `LPF  1.42 kHz`, `MASTER  85%`, etc.) and the bottom 4-knob strip stays active.
  - *Release:* Immediately returns to `STANDARD` rate (`15%` rest glow); if a knob was turned during the hold, its remaining `1.5 s` readout toast completes before dissolving in `150 ms` (or fades out immediately in `150 ms` if `OCT-/+` was tapped alone).

### 3.3 Right-Panel `2x6` Function Buttons (Latching Toggles & Focused Pages)
- **`SEL`:** **Pressure Pad Mode Select (`PRESSURE TARGET`)** — toggles **White Keys 5–8** between **MS-20 Filter Accent / Vactrol LPG** (default) and **Pitch Bend**. To connect `SEL` visually to `Keys 5-8` on monochrome white LEDs without ever looking "off", **`SEL` and all four `Keys 5-8` share identical PWM brightness in both modes** (`24%` soft glow in Filter/LPG mode <-> `72%` bright glow in Pitch Bend mode, stepping in unison on tap, plus `SEL` flaring toward `100%` alongside any pressed pad).
- **`PLAY/STOP`:** **Audio Output** toggle (`ON` / `OFF`, clickless fade to silence and back for safe cable patching). Drives the hardware's dual-channel `PLAY/STOP` LEDs by action affordance: steady **Green** (`PLAY` LED) when quiet (`OFF`), steady **Red** (`STOP` LED) when sound is `ON`, crossfading smoothly between them.
- **`HOME`:** **Exit Page / Clear HUD to Pure Scope** (tap dismisses any active page or toast with the unified **`150 ms`** S-curve dissolve; LED mirrors live HUD opacity `0%..100%`).
- **`SAVE`:** Tap `SAVE` (`STORE SNAPSHOT` / `TAP KEY 9-16`, `100%` LED) then tap a Snapshot white key (`9-16`) to store the current patch; tapping `SAVE` a second time disarms and displays `STORE SNAPSHOT · CANCELLED` (`1.5 s` toast -> `150 ms` fade-out).
- **`LFO`:** Opens/closes the **Per-Voice LFO Page** (see §3.4 below); LED glows at **`15%` rest** when all 4 LFOs are at `0%` depth (confirming an assigned page switch), breathes continuously (`15%–60%`) with active LFO modulation when closed, and lights to **`100%`** while the page is open.
- **`GLO` (Right Grid under `KNOB 1–4`):** Opens/closes the scalable **`GLOBAL CONFIG`** menu (`15%` steady rest LED, `100%` while open; see §3.4 below).
- **Unassigned / Reserved Switches (`FX`, `ENV`, `EDIT`, `ARP`, `SEQ`, `REC`):** LEDs stay **`0%` completely unlit** at rest while unassigned (`REC` is physically a single-color **Red** LED on the FM-1 PCB); pressing any unassigned switch displays its silkscreen name + `UNASSIGNED` in the HUD header so every control confirms hardware contact.

### 3.4 Per-Voice LFO Page (`LFO`) & `GLOBAL CONFIG` Menu (Right-Grid `GLO`)

#### Per-Voice LFO Page (`LFO OSC 1..4` — `ADR-022`, `ADR-027`)
Each of the 4 voices (`Osc 1..4`) owns **1 dedicated LFO** matching its dedicated MS-20 filter, VCA, and panner (multi-voice LFO macro control lives on Keybed `GLO` -> `GLOBAL LFO`):
- **Enter / Exit:** Tap **`LFO`** (right-grid switch) to open the LFO Page (`LFO OSC 1..4` showing `ACTIVE` / `BYPASSED`); tap **`LFO`** again or **`HOME`** to return to the pure scope in `150 ms` while active LFOs continue running.
- **`SELECT` Encoder (Left Hand):** Steps strictly across **`OSC 1` · `OSC 2` · `OSC 3` · `OSC 4`**.
- **`KNOB 1–4` (`TARGET | SHAPE | RATE | DEPTH` — Setup Left, Perform Right):**
  - **`KNOB 1` (`TARGET`):** Stepped per-voice parameter selector (`1` item/detent): **`LPF`** *(default)* · **`LP RES`** · **`HPF`** · **`HP RES`** · **`AMP`** · **`PITCH`** · **`PAN`**.
  - **`KNOB 2` (`SHAPE`):** Continuous `0–100%` seamless wave morph across **`Sine` (`0%`) -> `Triangle` (`25%`) -> `Sawtooth` (`50%`) -> `Slewed Square` (`75%`) -> `Smooth Drift` (`100%`, non-repeating organic wander)**.
  - **`KNOB 3` (`RATE`):** Continuous `0.02 Hz` (`50 s` cycle) to `30 Hz` log-octave sweep.
  - **`KNOB 4` (`DEPTH`):** Unipolar `0–100%` modulation amount with a hard stop at `0%` CCW so any LFO zeros cleanly by feel.
- **Live Full-Screen LFO Visualization:** Spans edge-to-edge across the entire `240 px` display (`x = 0..239`) and fills the full vertical viewport (`y = 26..188`, `±75 px` / `150 px` peak-to-peak centered on a `y = 107` zero-horizon filament) over a `72%`-dimmed background scope, with clean gradient shading and a high-radiance `#FFFFFF` optical Phase Beacon.

#### Scalable `GLOBAL CONFIG` Menu (Right-Grid `GLO` — `ADR-027`)
- **Enter / Exit:** Tap right-grid **`GLO`** (`ButtonId::GloRight`) to open/close **`GLOBAL CONFIG`** (`1/N` row counter in top-right; bottom 4-knob strip cleanly hidden over a `72%`-dimmed live scope; `HOME` or 2nd `GLO` tap dissolves in `150 ms`).
- **Navigation & Editing:** Turn **`SELECT`** (`1` row/detent) to move the highlighted row cursor across the vertical `5`-row viewport; turn **any `KNOB 1–4`** (`1` option/detent) to edit the highlighted row's value:
  1. **`GLO RELEASE`:** **`LATCH`** *(default — `GLOBAL` multi-voice edits persist when releasing keybed `GLO`)* / **`SNAP BACK`** *(Elektron `CTRL-ALL`-style temporary performance warp: captures `4 x VoiceParams` on keybed `GLO` press and smoothly restores them via `C^1` parameter slews on release)*.
  2. **`OUTPUT ROUTING`:** **`STEREO`** / **`DUAL MONO`** (synced bidirectionally with `GlobalParams::monoCollapse` and the physical `MONO` / `POLY` black keys).

### 3.5 Bottom 27-Key Surface & Carbon-Pill Pressure Expression (`SARADC`)

Using analog `SARADC` sampling of the silicone keybed's carbon-pill contact resistance, Ruido supports continuous **Soma Lyra-8-style finger-lean pressure expression** alongside tactile switching, and **every single key displays live technical feedback on the HUD**:

#### Top Row — 11 Raised Black Keys (Silkscreen-Matched, Strictly Momentary Modifiers)
- **`OP1` · `OP2` · `OP3` · `OP4`:** Contiguous 4-key **Per-Voice Filter Modifiers** (`15%` dim rest glow, `100%` while held) focusing `Osc 1–4 Filter`.
- **`OP5` (`AMPLITUDE`) · `OP6` (`PANNING`):** Adjacent 2-key **Per-Voice Mix & Spatial Pair** (`25%` landmark rest glow, `100%` while held) mapping `KNOB 1–4` to `Osc 1–4 Amplitude` (`OP5`) and `Osc 1–4 Panning` (`OP6`).
- **`GLO` (`GLOBAL` + `SELECT`):** **5-Mode Multi-Voice Performance Modifier** (`25%` landmark rest glow, `100%` while held; releasing either latches or snaps back per `GLO RELEASE` in `GLOBAL CONFIG`). While holding `GLO`, turning `SELECT` chooses **`FREQ`** (`PITCH | SPREAD | DETUNE | TILT` mutually orthogonal 4-voice pitch basis), **`FILTER`** (`HPF | HP RES | LPF | LP RES` relative octave/linear deltas), **`LFO`** (`TARGET | SHAPE | RATE | DEPTH` across all 4 LFOs), **`AMP`** (`LEVEL | TILT | WIDTH | BALANCE` across VCA + stereo field), or **`ALL`** (`FREQ | FILTER | LFO | AMP` 4-stage macro), each paired with a bespoke acoustic center visualization and 4-voice spread micro-beads on diverged bottom-strip filaments.
- **`PIT`:** Unassigned in v1 (`0%` dark LED at rest; displays `PIT KEY · UNASSIGNED` if tapped—keeping the bottom black-key modifier row `100%` momentary).
- **`MONO` · `POLY`:** Instant `OUTPUT ROUTING` toggle between **Dual Mono (`MONO`)** (for mono pedalboards) and **Stereo (`POLY`)** (for stereo pedals / dual Eurorack channels).
- **11th Tall Black Key (`F#5`):** **Momentary Scope Freeze** (`SCOPE FREEZE` / `HOLDING`; LED `0%` at rest, `100%` while freezing the coupled 4-voice caustic ribbon mesh and optical bloom).

#### Bottom Row — 16 White Keys (4 Silkscreen-Dotted Groups of 4)
- **Dot 1 (`White Keys 1-4`): Osc 1-4 Latching Gates**
  - Clean tap toggles `Osc 1-4` gate (`GATE OSC 1..4`: `ACTIVE` / `MUTED` with `2-5 ms` clickless vactrol fade), remembering the oscillator's pre-mute amplitude when toggled back on. Holding **`OP5`** (`Amplitude`) displays **`0%`** for muted oscillators (without erasing their remembered level unless turned) and turning their `KNOB 1-4` clockwise fades them in smoothly from `0%` while clearing their mute state.
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
  - **Open Full-Span Geometry (`x = 0..239`, constant `y = 120` center & `112 px` linear CRT deflection) & Zero Hard-Cut Transitions:** Zero artificial edge pinching to `0 V` at the left/right bezels, **zero vertical compression or shift** when the HUD fades in (resonant peaks deflect linearly past `y = 0` and `y = 239` without soft-clip squashing), and **zero 1-frame visual snaps** anywhere on the instrument—stopping audio (`PLAY/STOP`, `MASTER = 0%`, muting all voices, or pausing audio) smoothly relaxes all 14 strands + bloom into a poised `y = 120` horizon flatline and blooms back out on resume, while per-voice mutes, `1x <-> 2x` bass timebase zoom, filter curves, optical beads, and HUD overlays all transition continuously.
- **Unified Soft-Transitioning Optical-Filament HUD (`150 ms` Centralized S-Curve Envelope):**
  - **Top Function Header (`13 pt` Bold Two-Tone Hierarchy, Split `FINE`/`COARSE` Override & Top-Aligned Metadata):** Reserved top strip (`y = 6..22`) leading with the active function title (`FREQUENCY`, `FILTER`, `AMPLITUDE`, `PANNING`, `GLOBAL`, `LFO`, `GLOBAL CONFIG`, or locked **`FINE`** / **`COARSE`** while `OCT-` / `OCT+` is physically held) in `100%`-white `13 pt` Bold `Helvetica Neue` (`kFontSansBold`), followed by dimmed trailing target qualifiers (`OSC 1..4`, `FREQ`, `FILTER`, `LFO`, `AMP`, `ALL` at `~46%` silver) alongside pixel-exact top-aligned (`y = 9`) right-hand `10 pt` metadata, live `OCT-/+` high-precision readouts, or `13 pt` `%` + tapered optical-filament gauges (paired with idle top-right `L/R` optical-filament peak meters when the HUD is closed).
  - **Bottom 4-Knob Strip, 4-Voice Spread Micro-Beads & Bespoke Acoustic Center Visualizations:** Reserved bottom strip (`y = 190..235`) **always** showing what `KNOB 1..4` currently control whenever the HUD is visible (hidden only inside `GLOBAL CONFIG` where `KNOB 1..4` edit the highlighted menu row), with 4 tapered sub-pixel optical filaments, focal beads, 4 subtle per-voice spread micro-beads when `GLOBAL` voices diverge, and tabular sans-serif readouts (`55.0 Hz`, `2.4 kHz`, `65%`, `L35`, `MIXED`). Center visualizations all share a `72%` background scope dim synchronized to the same `150 ms` S-curve:
    - **MS-20 Passband (`OP1–OP4`, `Keys 5–8` Filter/LPG, `GLOBAL FILTER`):** `80 px` (`y = 102..182`, `x = 10..230`) Option A3 dual-peak MS-20 transfer curve(s) with `10%` passband silhouette, `30%` crest glow, and `HP`/`LP` focal beads (`4` overlaid per-voice curves in `GLOBAL FILTER`).
    - **`GLOBAL FREQ` Spectral Stems:** `20 Hz–4 kHz` log-frequency horizon (`y = 104..182`) with 4 voice stems, amplitude-scaled radiant tips, and centroid guide.
    - **`GLOBAL LFO` & `LFO` Full-Screen Wave:** Edge-to-edge (`x = 0..239`, `y = 26..188`, `±75 px` on `y = 107`) morphed LFO wave(s) and high-radiance Phase Beacon(s) (`1` voice on `LFO` page, `4` voices overlaid in `GLOBAL LFO`).
    - **`GLOBAL AMP` Stereo Soundstage:** `L..C..R` equal-power stereo horizon (`y = 104..182`) with center axis tick, stereo width bracket, and 4 amplitude-height voice pillars (`0%` if muted).
    - **`GLOBAL ALL` Unified Synth Patch View:** `20 Hz–20 kHz` 4-voice average MS-20 passband overlaid with the 4 live oscillator pitch/amplitude stems.
  - **Centralized `150 ms` S-Curve Envelope (`HudEnvelope.h`), Latched Pages & Outgoing-View Latch:** Every HUD fade-in and fade-out across the entire instrument—keybed holds, standalone knob `1.5 s` toasts, center graphs, `72%` backdrop dim, `HOME` LED, and page opens/closes—runs on a single centralized **`150 ms` linear ramp mapped through smoothstep** (`kHudFadeSec = 0.150f`, `kHudToastHoldSec = 1.500f`). Right `2x6` switches (`SEL`, `PLAY/STOP`, `SAVE` while armed, `LFO`, `GLOBAL CONFIG`, unassigned switches) latch until `HOME` (or a second toggle tap on `LFO` / `GLO` / `SAVE`), smoothly returning to any latched page after momentary holds or `1.5 s` knob toasts, while outgoing labels latch during fade-out so `FREQUENCY` never flashes on release.

---

## 5. Explicit Non-Goals (v1 Scope Guardrails)

To keep Ruido focused, reliable, and sonically uncompromising, **v1 explicitly excludes**:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI polyphony / voice-stealing envelopes.
- Onboard time-based FX (reverb, delay, chorus)—Ruido is the **dry analog-style source** feeding your external pedals and Eurorack modules.
- Deep nested sub-menus or file browsers (focused single-level pages on right-grid buttons such as `LFO` and `GLO` Config are used judiciously, always instant-exit via `HOME`).
