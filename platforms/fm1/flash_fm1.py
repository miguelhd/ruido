#!/usr/bin/env python3
"""Safe macOS / Linux host wrapper for querying (`--info`) and flashing `build/fm1/ruido.fwsc`.

Uses Dinghy/Felucca's battle-tested pull-based USB-MIDI SysEx installer (`fm1_install.py`, staged
into gitignored `build/fm1/` from `ruido-fm1-toolchain:latest` so zero GPL-3.0 third-party source
is vendored in Git per ADR-044) inside an isolated `build/.venv` (`mido` + `python-rtmidi`), gated
by offline CRC-16, `FELUCCA-LOADER-1`, size (`< 600 KiB`), and dual-JEDEC (`P25Q80` + `UC25HQ80`)
pre-flight verification before opening any MIDI port.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import pathlib
import resource
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "build" / "fm1"
VENV_DIR = ROOT / "build" / ".venv-fm1-flash"
INSTALL_SCRIPT = OUT_DIR / "fm1_install.py"
FWSC_PATH = OUT_DIR / "ruido.fwsc"
APP_BIN_PATH = OUT_DIR / "ruido.bin"
LDR_BIN_PATH = OUT_DIR / "loader" / "loader.bin"
DOCKER_IMAGE = os.environ.get("RUIDO_FM1_DOCKER_IMAGE", "ruido-fm1-toolchain:latest")
EXPECTED_PRODUCT = "FM-1_960"
MAX_FWSC_BYTES = 600 * 1024

REQUIRED_JEDEC_IDS = (
    ("P25Q80", 0x00856014, b"\x14\x60\x85\x00"),
    ("UC25HQ80", 0x00B36014, b"\x14\x60\xb3\x00"),
)


def ensure_install_script() -> None:
    if INSTALL_SCRIPT.is_file():
        return
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    res = subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--platform",
            "linux/amd64",
            DOCKER_IMAGE,
            "cat",
            "/opt/dinghy/tools/fm1_install.py",
        ],
        capture_output=True,
        check=False,
    )
    if res.returncode != 0 or not res.stdout:
        err = res.stderr.decode("utf-8", "replace").strip()
        raise SystemExit(
            f"[FM-1 FLASH] Could not stage fm1_install.py from {DOCKER_IMAGE}: {err}\n"
            "  Run `make fm1-check` first (with Docker running)."
        )
    INSTALL_SCRIPT.write_bytes(res.stdout)


def ensure_midi_python() -> pathlib.Path:
    probe = (
        "import importlib.util, sys; "
        "sys.exit(0 if importlib.util.find_spec('mido') and importlib.util.find_spec('rtmidi') else 1)"
    )
    for candidate in (pathlib.Path(sys.executable), VENV_DIR / "bin" / "python3"):
        if not candidate.is_file():
            continue
        chk = subprocess.run(
            [str(candidate), "-c", probe],
            capture_output=True,
            check=False,
        )
        if chk.returncode == 0:
            return candidate

    print("[FM-1 FLASH] Bootstrapping isolated CoreMIDI Python venv in build/.venv-fm1-flash ...")
    subprocess.run([sys.executable, "-m", "venv", str(VENV_DIR)], check=True)
    venv_py = VENV_DIR / "bin" / "python3"
    subprocess.run(
        [str(venv_py), "-m", "pip", "install", "--quiet", "mido", "python-rtmidi"],
        check=True,
    )
    return venv_py


def check_macos_coremidi_access() -> int:
    """Probe CoreMIDI in pure C so Seatbelt blocks exit cleanly without RtMidiError SIGABRT."""
    if sys.platform != "darwin":
        return 0
    try:
        cf = ctypes.CDLL("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation")
        cm = ctypes.CDLL("/System/Library/Frameworks/CoreMIDI.framework/CoreMIDI")
        cf.CFStringCreateWithCString.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint32,
        ]
        cf.CFStringCreateWithCString.restype = ctypes.c_void_p
        cf.CFRelease.argtypes = [ctypes.c_void_p]
        cf.CFRelease.restype = None
        cm.MIDIClientCreate.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint32),
        ]
        cm.MIDIClientCreate.restype = ctypes.c_int32
        cm.MIDIClientDispose.argtypes = [ctypes.c_uint32]
        cm.MIDIClientDispose.restype = ctypes.c_int32

        name = cf.CFStringCreateWithCString(None, b"ruido-preflight", 0x08000100)
        if not name:
            return 0
        try:
            client = ctypes.c_uint32(0)
            status = int(cm.MIDIClientCreate(name, None, None, ctypes.byref(client)))
            if status == 0 and client.value:
                cm.MIDIClientDispose(client.value)
            return status
        finally:
            cf.CFRelease(name)
    except OSError:
        return 0


def audit_local_artifacts(fwsc_file: pathlib.Path) -> None:
    if not fwsc_file.is_file():
        raise SystemExit(
            f"[FM-1 FLASH] Missing {fwsc_file}. Run `make fm1-check` first to build and emu-verify."
        )
    raw = fwsc_file.read_bytes()
    if len(raw) >= MAX_FWSC_BYTES:
        raise SystemExit(
            f"[FM-1 FLASH] Refusing to flash: {fwsc_file} ({len(raw):,} B) >= 600 KiB limit."
        )
    if b"FELUCCA-LOADER-1" not in raw:
        raise SystemExit(
            f"[FM-1 FLASH] Refusing to flash: {fwsc_file} lacks `FELUCCA-LOADER-1` safe app-only loader."
        )

    sys.path.insert(0, str(OUT_DIR))
    import fm1_install  # type: ignore

    prod, img = fm1_install.load_package(str(fwsc_file), force=False)
    dmg = fm1_install.package_damage(img)
    if dmg is not None:
        raise SystemExit(f"[FM-1 FLASH] Refusing to flash damaged package: {dmg}")
    if fwsc_file.resolve() == FWSC_PATH.resolve():
        if prod != EXPECTED_PRODUCT:
            raise SystemExit(
                f"[FM-1 FLASH] Stale package identity {prod!r} (expected {EXPECTED_PRODUCT!r}). "
                "Run `make fm1-check` to rebuild."
            )
        for bin_label, bin_path in (("ruido.bin", APP_BIN_PATH), ("loader.bin", LDR_BIN_PATH)):
            if not bin_path.is_file():
                raise SystemExit(
                    f"[FM-1 FLASH] Missing {bin_path} build artifact. Run `make fm1-check` first."
                )
            blob = bin_path.read_bytes()
            for chip_name, jedec_id, le_bytes in REQUIRED_JEDEC_IDS:
                if le_bytes not in blob:
                    raise SystemExit(
                        f"[FM-1 FLASH] {bin_label} missing {chip_name} JEDEC ID {jedec_id:#08x}!"
                    )
    print(
        f"[Pre-Flash Audit OK] {fwsc_file.name}: {len(raw):,} B ({len(raw) / 1024.0:.1f} KiB), "
        f"identity={prod}, CRC16=OK, loader=FELUCCA-LOADER-1 (app-only >= 0x4000, P25Q80+UC25HQ80)"
    )


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--info", action="store_true", help="Read-only USB-MIDI handshake check")
    ap.add_argument("--port", metavar="NAME", help="Optional CoreMIDI port substring filter")
    ap.add_argument("--yes", action="store_true", help="Skip interactive [y/N] confirmation")
    ap.add_argument("--fwsc", type=pathlib.Path, default=FWSC_PATH, help="Path to .fwsc package")
    args = ap.parse_args(argv)

    ensure_install_script()
    if not args.info:
        audit_local_artifacts(args.fwsc)

    midi_status = check_macos_coremidi_access()
    if midi_status != 0:
        print(
            f"[FM-1 FLASH] macOS CoreMIDI server unavailable in sandbox (OSStatus {midi_status}); "
            "re-run with host MIDI access enabled.",
            file=sys.stderr,
        )
        return 2

    try:
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    except (OSError, ValueError):
        pass

    py = ensure_midi_python()
    runner = (
        "import os, resource, sys\n"
        "try:\n"
        "    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n"
        "except (OSError, ValueError):\n"
        "    pass\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "import fm1_install\n"
        "rc = fm1_install.main(sys.argv[2:])\n"
        "sys.stdout.flush()\n"
        "sys.stderr.flush()\n"
        "os._exit(rc if isinstance(rc, int) else 0)\n"
    )
    cmd = [str(py), "-c", runner, str(OUT_DIR)]
    if args.info:
        cmd.append("--info")
    else:
        cmd.append(str(args.fwsc))
        if args.yes:
            cmd.append("--yes")
    if args.port:
        cmd.extend(["--port", args.port])

    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
