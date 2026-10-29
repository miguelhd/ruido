# Ruido — Product & UX Specification

## 1. Vision & Musical Identity

**Ruido** transforms the pocket **M-VAVE FM-1** hardware into a dedicated, boutique **4-oscillator CMOS Schmitt-trigger stereo drone instrument**.

- **Primary Acoustic Reference:** Classic DIY analog Schmitt-trigger relaxation oscillator synths (`CD40106` / `74HC14` style)—organic capacitor tilt, micro-thermal drift, and shared power-rail phase pulling—paired with aggressive **Korg MS-20 (`Korg35`) dual `HPF -> LPF` diode-clipped resonant filters**.
- **Primary Studio Role:** A pristine, harmonically rich, zero-aliasing stereo sound source designed specifically to feed **external guitar/synth effects pedals** (fuzz, wavefolders, BBD delays, reverbs, granular) and **Eurorack modular rigs**.

### Core Design Values (In Priority Order)
1. **Great Sound:** Spend every spare CPU cycle on analog-faithful oscillator physics, `4x` oversampling, nonlinear Zero-Delay Feedback filtering, and clickless slewing.
2. **Great, Stateless UX:** Zero menus, zero scrolling lists, zero persistent modal pages to get lost in, and strict two-hand ergonomics that never cross arms over the display.
3. **Useful Waveform Visuals:** An always-on oscilloscope that serves as an actual acoustic tuning and gain-staging instrument, with minimal controls gently fading in over the wave only while touched.

---

## 2. Voice & Signal Architecture (User-Facing)

Ruido consists of **4 identical mono voices** (`Osc 1..4`) mixed in stereo through a subtle nonlinear master bus:

```text
[Schmitt Square Osc 1..4] ──> (Drive) ──> [MS-20 HPF -> LPF 1..4] ──> (Amp + Mute/Gate + Pan) ──> [Master Bus Sag] ──> Stereo L/R Out
          ▲                                                                                                │
          └────────────── Shared Power-Rail Coupling (`ALGORITHM` Knob: 0-100% Circuit Sag) ───────────────┘
```

### Per-Oscillator Controls (`Osc 1..4`)
- **Speed (Frequency):** Continuous wide sweep across Audio Drone range (`20 Hz - 4,000 Hz`) or Sub-Audio LFO clicking range (`0.02 Hz - 20 Hz`, toggled per voice via `LFO`), using **velocity-sensitive encoder ballistics**:
  - *Slow turn:* Sub-cent / hundredth-of-a-Hz resolution for dialing slow phase beating and harmonic locks.
  - *Fast turn:* Wide multi-octave sweeps.
- **Amplitude:** Continuous post-filter voice mix level.
- **Pan:** Constant-power (`sin`/`cos`) stereo placement from hard Left (`100% L`) through Center to hard Right (`100% R`), scaled globally by `SELECT` (Stereo Spread) or collapsed via `MONO`.
- **Dual MS-20 Filter (`HPF -> LPF` in series):**
  - **HP Cutoff (`KNOB1`):** `20 Hz` (parked flat/transparent) up to `20 kHz`.
  - **LP Cutoff (`KNOB2`):** `20 Hz` up to `20 kHz` (parked flat/wide-open).
  - **Shared Resonance (`KNOB3`):** Controls diode-clipped resonant emphasis for both `HPF` and `LPF` simultaneously using **Smart-Park Taper**—a stage parked at `20 Hz` (`HP`) or `20 kHz` (`LP`) tapers its resonance bump to flat (`Q = 0.707`) so parked filters never cause sub-rumble or ultrasonic whistle. Moving *both* cutoffs inward awakens twin MS-20 formant/throat peaks.
  - **Pre-Filter Drive (`KNOB4`):** Pushes the square wave harder into the internal Sallen-Key diode clippers independently of post-filter mix `Amplitude`.

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
1. **Strict Left-Hand Holds:** Every momentary `"Hold + Turn KNOB1-4"` gesture lives in the **bottom-left / bottom-center black keys** (`OP1-OP6`, `GLO`). The left hand stays down-left, the right hand stays top-right on `KNOB1-4`, and the center `240x240` screen is never occluded by a crossed forearm.
2. **Right-Side Button Grid = Single-Tap Only:** The `2x6` button cluster sits directly under `KNOB1-4` on the right and is strictly used for single-tap actions—never for momentary holds alongside `KNOB1-4`.

### 3.1 Top-Right Encoders: `KNOB1` – `KNOB4` (Approach A: Always Oscillators 1–4)

| Left-Hand Hold (Black Key) | `KNOB1` | `KNOB2` | `KNOB3` | `KNOB4` | Translucent Scope Overlay |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **None (Idle Default)** | **Osc 1 Speed** | **Osc 2 Speed** | **Osc 3 Speed** | **Osc 4 Speed** | Minimal `Hz` readout on touched voice |
| **Hold `OP5`** *(Lit `25%`)* | **Osc 1 Amp** | **Osc 2 Amp** | **Osc 3 Amp** | **Osc 4 Amp** | `AMPLITUDE` + 4 vertical level bars |
| **Hold `OP6`** *(Lit `25%`)* | **Osc 1 Pan** | **Osc 2 Pan** | **Osc 3 Pan** | **Osc 4 Pan** | `PAN` + 4 `L ──●── R` ticks |
| **Hold `OP1`..`OP4`** | **HP Cutoff** | **LP Cutoff** | **Resonance** | **Filter Drive** | That Osc's `[ HP \|════\| LP ]` MS-20 curve |
| **Hold `GLO`** *(Bottom Key)* | **All HP Cut** | **All LP Cut** | **All Reso** | **All Drive** | Global MS-20 curve sweeping all 4 filters |

### 3.2 Top-Left Knobs & `OCT-` / `OCT+` (Zero-Modifier Global Controls)
- **`MASTER` (Analog Pot):** Hardware master output volume.
- **`ALGORITHM` (Encoder):** **CMOS Circuit Coupling / Rail Sag** (`0%` isolated clean circuits <-> `100%` heavy `CD40106` shared-rail threshold pulling & cross-modulation). *Mnemonic: defines how the 4 oscillators electrically couple, analogous to FM operator algorithms.*
- **`SELECT` (Push-Encoder):** **Global Stereo Spread** (`0%` Mono <-> `100%` Full Pan Width; push-click resets to `100%`).
- **`PRESETS` (Encoder):** **Continuous Snapshot Morph** — smoothly scrubs/crossfades across saved Drone Snapshots.
- **`OCT-` / `OCT+` (Buttons):**
  - *Standalone tap:* Halves (`÷2`) or doubles (`×2`) all 4 oscillator frequencies by an exact octave (`OCT-` + `OCT+` together resets global octave shift to `0`).
  - *While holding `OP1..4`:* Exact `÷2` / `×2` octave jump on that single oscillator.

### 3.3 Right-Panel `2x6` Function Buttons (Single-Tap Utilities Only)
- **`PLAY/STOP`:** **Master Drone Run / Mute** (clickless fade to silence and back for safe cable patching).
- **`HOME`:** **Clear HUD to Pure Scope** (tap dismisses any active overlay immediately).
- **`SAVE`:** Tap `SAVE` (arms save LED briefly) then tap a Snapshot white key (`9-16`) to store the current drone.
- **`LFO`:** Hold `OP1..4` (left hand) + tap `LFO` (right hand) to toggle that oscillator between **Audio Drone (`20-4000 Hz`)** and **Sub-Audio LFO (`0.02-20 Hz`)** speed ranges.
- **Unassigned Buttons (`FX`, `SEL`, `ENV`, `EDIT`, `GLO` top-right, `ARP`, `SEQ`, `REC`):** Kept **`0%` completely unlit** in v1 so only active, literally-labeled controls draw the eye.

### 3.4 Bottom 27-Key Surface & Carbon-Pill Pressure Expression (`SARADC`)

Using analog `SARADC` sampling of the silicone keybed's carbon-pill contact resistance, Ruido supports continuous **Soma Lyra-8-style finger-lean pressure expression** alongside tactile switching:

#### Top Row — 11 Raised Black Keys (Silkscreen-Matched)
- **`OP1` · `OP2` · `OP3` · `OP4`:** Left-hand hold to focus `Osc 1-4` Filter on `KNOB1-4` (`15%` dim rest glow, `100%` while held).
- **`OP5` (`AMP`) · `OP6` (`PAN`):** Left-hand holds for `Osc 1-4 Amplitude` and `Osc 1-4 Pan` (`25%` steady landmark glow, `100%` while held).
- **`PIT` ("Pitch"):** Toggles whether finger pressure on the expression pads bends **Oscillator Pitch** (organic tape/capacitor bend) instead of **Filter Cutoff & Drive**.
- **`GLO` ("Global"):** Hold to sweep **All 4 MS-20 Filters** together on `KNOB1-4`.
- **`MONO` · `POLY`:** Instant output mode toggle between **Dual-Mono (`MONO`)** (for mono pedalboards) and **Full Stereo (`POLY`)** (for stereo pedals / dual Eurorack channels).
- **11th Unlabeled Black Key (`F#5`):** `0%` off (reserved).

#### Bottom Row — 16 White Keys (4 Silkscreen-Dotted Groups of 4)
- **Dot 1 (`White Keys 1-4`): Osc 1-4 Latching Mutes**
  - Clean tap toggles `Osc 1-4` on/off (`2-5 ms` clickless vactrol fade).
  - LEDs glow at `10%` ember when muted, `60-100%` scaled by amplitude when active, and **physically pulse with every click** when running `< 20 Hz`.
- **Dot 2 (`White Keys 5-8`): Osc 1-4 Lyra-8 Pressure Expression Pads**
  - *When Osc is Muted:* **Press & lean harder** to swell that oscillator in from silence proportionally to finger pressure; release to fade out.
  - *When Osc is Droning:* **Lean harder** to push its MS-20 Filter Cutoff + Drive open (or bend its pitch organically when `PIT` is lit).
- **Dots 3 & 4 (`White Keys 9-16`): 8 Drone Snapshots (with Rubber-Band Pressure Pull)**
  - *Quick Tap:* Glides smoothly to Snapshot `1-8` (`0%` empty, `20%` saved, `100%` active, slow breathing during glide).
  - *Press, Hold & Lean:* **Rubber-band morphs** all 4 oscillators and filters *toward* that snapshot proportionally to finger pressure—release your finger, and the drone melts smoothly back to your active state.

---

## 4. Always-On `240x240` Scope + Translucent Fade-In HUD

- **Base Canvas (100% of the time):** Edge-to-edge CRT-style anti-aliased oscilloscope **rising-edge triggered on Oscillator 1**. Osc 1 stands still on screen while Osc 2-4 visibly drift, step, and lock into harmonic intervals across it, with subtle `Left`/`Right` stereo dual-trace separation and a subtle true-peak clip indicator.
- **Transient HUD Overlays:** Touching an encoder or holding a black-key modifier gently fades in (`~120 ms`) a minimal translucent overlay right over the live moving wave (dimming the waveform ~30% behind active typography or the `[ HP |══════| LP ]` filter passband curve). ~1.5 seconds after release, the HUD dissolves (`~800 ms` exponential fade) back to `0%` opacity.

---

## 5. Explicit Non-Goals (v1 Scope Guardrails)

To keep Ruido focused, reliable, and sonically uncompromising, **v1 explicitly excludes**:
- Multi-track step sequencers, piano rolls, or arpeggiators.
- Chromatic MIDI polyphony / voice-stealing envelopes.
- Onboard time-based FX (reverb, delay, chorus)—Ruido is the **dry analog-style source** feeding your external pedals and Eurorack modules.
- Multi-page settings menus, file browsers, or modal button traps.
