# Changelog

## v0.8.0 (`FM-1_312`) — 2026-11-25

Major sound engine upgrade with warmer analog synthesis and smoother performance across the instrument.

- **Upgraded synthesis & filters:** Reworked the oscillators, overdrive, and resonant filters to behave closer to analog hardware. Multi-voice drones sound thicker without harsh digital edges, filter resonance sweeps cleanly from warm shaping into self-oscillation without high-end fizz, and drive sounds richer.
- **Click-free performance:** Eliminated clicks and audio dropouts when sweeping filters, pushing levels into heavy drive, morphing scenes, or layering keyboard effects.
- **Initialize single scenes (`INIT SCENE`):** Reset any scene slot to a clean starting drone from the `SAVE` menu without wiping your other saved scenes.
- **Better visual feedback:** Turning multiple knobs or holding multiple keys now highlights every active voice on screen at once, with clear indicators for `FINE`, `COARSE`, and `LATCH`, and safer factory reset confirmation.

User Guide: https://noisexrecords.com/ruido

Send feedback or comments to ruido@noisexrecords.com

## v0.7.3 (`FM-1_312`) — 2026-11-23

User Guide: https://noisexrecords.com/ruido

- **UX refinements:** Improved keybed LED feedback when morphing scenes and added the firmware version to the boot screen.
- **User Guide:** Added `noisexrecords.com/ruido` to the `About` screen and documentation.

## v0.7.2 (`FM-1_312`) — 2026-11-20

User Guide: https://noisexrecords.com/ruido

- **Silent power-on:** Boots with audio output muted (`PLAY` green LED lit) until `PLAY/STOP` is pressed.
- **UI clarity:** Added the `SHIFT` label to all `SEL` keyboard maps and clarified `SAVE` / `DELETE` scene labels.
- **User Guide & screenshots:** Published the web User Guide and updated documentation screenshots.

## v0.7.1 (`FM-1_312`) — 2026-11-17

- Updated Firmware Hub card metadata (`fmhub.json`), hero screenshot selection, and product documentation.

## v0.7.0 (`FM-1_312`) — 2026-11-16

- Initial Firmware Hub public release (`FM-1_312`) with 4 factory scenes, 3 pressure-sensitive performance keyboards (`LIVE MIXER`, `CIRCUIT LAB`, `RHYTHM LAB`), universal `SHIFT` layer, circular scene morphing, `SAVE` / `DELETE` scene manager, `Clean Init` reset mode, and paused visual/LED freeze when stopped.
