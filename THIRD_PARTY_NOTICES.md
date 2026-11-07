# Licensing & Third-Party Notices

**Ruido** (`v1.0`, FM-1 package identity `FM-1_960`)
Copyright (C) 2026 Miguel Hernández Domenech (<https://github.com/miguelhd/Ruido>)

Ruido firmware images built for the M-VAVE FM-1 (`build/fm1/ruido.fwsc`) link against the open-source FM-1 platform layer (`GPL-3.0-only`) and are distributed under the **GNU General Public License, version 3 only (`GPL-3.0-only`)**. See [`LICENSE`](LICENSE) for the full license text.

If you distribute a compiled `.fwsc` or `.bin` firmware image built from this project, you must provide or offer the complete corresponding source code under `GPL-3.0-only` as required by Section 6 of the license.

---

## 1. FM-1 Hardware Platform, Bootguard & USB Updater (`GPL-3.0-only`)

The FM-1 hardware abstraction layer, `Hold OCT-` USB rescue mode, `FELUCCA-LOADER-1` RAM OTA updater, USB Audio/MIDI stack, and NVS sector wrapper are fetched at container build time from [`zvenson/dinghy`](https://github.com/zvenson/dinghy) (`GPL-3.0-only`, no third-party files vendored in this repository) and build upon:

- **Felucca** (<https://github.com/hugelton/Felucca>) — Copyright (C) 2026 Leo Kuroshita (`@kurogedelic`), Hügelton Instruments (`firmware/hal/`, `firmware/loader/`, `crt0.S`, `app.ld`, `usb.c`, `ota.c`, `recovery.c`, `lcd.c`, `gfx.c`, `libc.c`, `bootguard.h`, `uac_fir.h`, `tools/`).
- **SLOOP** (<https://github.com/isod89/sloop-fm1>) — Platform and USB audio/MIDI refinements by `isod89`, plus TRS UART MIDI ring handling from **Felucca [Salt]** by `ChanceTheMaker` (identified by `keremimo`).
- **Dinghy / OberVave** (<https://github.com/zvenson/dinghy>) — Copyright (C) 2026 `zvenson` (`platform.c`, `store.h`, `LCD_IDLE`, `ldr_screen.c`).
- **Skiff** (<https://github.com/illuminus5/skiff>) — Update loader screen concept and `5x7` glyph layout by `illuminus5` (`GPL-3.0`).
- **sloop-fm1 `smk37` / packaging tools** (<https://github.com/genkigenki/sloop-fm1>) — Copyright (C) 2026 M. Geisler (`genkigenki` / `makkuro`) (`fm1_flash_map.h`, `fm1pkg_make.py`, `fm1_install.py`).

---

## 2. Rescue & Boot-Stage Font (`SIL Open Font License 1.1`)

- **Terminus Font (`ter-u16n`, 8x16)** — Copyright (C) Dimitar Toshkov Zhekov, licensed under the **SIL Open Font License, Version 1.1 (`OFL-1.1`)** (<https://openfontlicense.org>). Compiled into the pre-`main` `USB RESCUE` and `UPDATE` diagnostic screens via `dinghy`'s `felucca_font.h`.

---

## 3. JieLi `AC79NN` SoC Bootloader & Audio EQ Blobs (`Apache-2.0`)

At `.fwsc` container packaging time (`platforms/fm1/build_fm1.py`), three unmodified SDK binary blobs (`uboot.boot`, `cfg_tool.bin`, and `cfg/eq_cfg_hw.bin`, verified by SHA-256) are read from the JieLi `fw-AC79_AIoT_SDK` (`AC79NN_SDK_V1.2.1_2023-12-13`, <https://gitee.com/Jieli-Tech/fw-AC79_AIoT_SDK>):

- Copyright (C) JieLi Tech, licensed under the **Apache License, Version 2.0 (`Apache-2.0`)** (<http://www.apache.org/licenses/LICENSE-2.0>). No JieLi SDK files are stored in this Git repository.

---

## 4. Build-Time Verification Emulator (`MIT`, Not Linked Into Firmware)

- **fm1-emu** (<https://github.com/joakimeriksson/fm1-emu>) — Copyright (C) Joakim Eriksson, licensed under the **MIT License**. Used exclusively during offline build/CI verification (`make fm1-check`); no `fm1-emu` code is linked into `ruido.bin` or `ruido.fwsc`.

---

## 5. Trademarks

"M-VAVE" and "FM-1" are trademarks of their respective owners. "Felucca" and "Hügelton Instruments" are trademarks of Hügelton Instruments. Ruido is an independent open-source firmware project and is not affiliated with, sponsored by, or endorsed by M-VAVE or any third-party trademark holder.
