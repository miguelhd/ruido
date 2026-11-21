#!/usr/bin/env python3
"""Scalable, declarative screenshot generator & auditor for Ruido (`240x240` + `960x960`).

Reads screen recipes from `tools/screens.json` (or `--spec <file.json>`), compiles and caches a
reusable headless C++ capture interpreter (`build/screen_capture_cli` from `tools/screen_capture_cli.cpp`),
renders bit-identical `240x240` RGB565 scanline frames, verifies pixel invariants, and writes:
  - `<out-dir>/240x240/<id>.png` (1:1 `240x240` RGB888)
  - `<out-dir>/<id>.png`         (4x nearest-neighbor `960x960` RGB888)

Usage examples:
  python3 tools/generate_screens.py                   # Regenerate all screens in tools/screens.json
  python3 tools/generate_screens.py --only 02 08      # Regenerate matching screen IDs or categories
  python3 tools/generate_screens.py --list            # List registered screens and summaries
  python3 tools/generate_screens.py --check           # Verify PNGs are up-to-date, valid & linked
  python3 tools/generate_screens.py --prune           # Remove orphan PNGs not in tools/screens.json
  python3 tools/generate_screens.py --spec s.json --out-dir /tmp/preview
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import struct
import subprocess
import sys
import tempfile
import zlib
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_SPEC = ROOT / "tools" / "screens.json"
DEFAULT_OUT_DIR = ROOT / "docs" / "assets" / "screens"
CPP_DRIVER_PATH = ROOT / "tools" / "screen_capture_cli.cpp"
BUILD_DIR = ROOT / "build"
CLI_BIN = BUILD_DIR / "screen_capture_cli"
CLI_STAMP = BUILD_DIR / "screen_capture_cli.sha256"

BUTTONS = {
    "OctMinus", "OctPlus", "Fx", "Sel", "Env", "Lfo", "Edit",
    "GloRight", "Home", "Save", "Arp", "Seq", "PlayStop", "Rec",
}
ENCODERS = {"Select", "Presets", "Algorithm", "Knob1", "Knob2", "Knob3", "Knob4"}
BLACK_KEYS = {
    "Op1", "Op2", "Op3", "Op4", "Op5", "Op6",
    "Pit", "Glo", "Mono", "Poly", "ScopeFreezeKey",
}
LFO_TARGETS = {
    "Pitch", "PulseWidth", "LpCutoff", "LpResonance", "HpCutoff",
    "HpResonance", "Amplitude", "Pan", "LfoARate", "LfoADepth",
}
VOICE_FLOAT_FIELDS = {
    "freq_hz", "duty", "drift", "hp_hz", "hp_res", "lp_hz", "lp_res",
    "amp", "pan", "lfo_rate_hz", "lfo_shape", "lfo_depth",
    "lfo_b_rate_hz", "lfo_b_shape", "lfo_b_depth",
}


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag)
    crc = zlib.crc32(data, crc) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


def encode_rgb_png(rgb: bytes, width: int, height: int) -> bytes:
    row_bytes = width * 3
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # Filter type 0 (None)
        raw.extend(rgb[y * row_bytes : (y + 1) * row_bytes])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    compressed = zlib.compress(bytes(raw), level=9)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", compressed)
        + _png_chunk(b"IEND", b"")
    )


def upscale_4x_nearest(rgb240: bytes, w: int = 240, h: int = 240) -> bytes:
    out = bytearray(w * 4 * h * 4 * 3)
    dst_row_bytes = w * 4 * 3
    for y in range(h):
        row = bytearray(dst_row_bytes)
        src_base = y * w * 3
        for x in range(w):
            px = rgb240[src_base + x * 3 : src_base + x * 3 + 3]
            d = x * 12
            row[d : d + 12] = px * 4
        for dy in range(4):
            off = (y * 4 + dy) * dst_row_bytes
            out[off : off + dst_row_bytes] = row
    return bytes(out)


def ensure_capture_binary(force_rebuild: bool = False) -> pathlib.Path:
    subprocess.run(
        ["make", "build/test_ui"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    objs = sorted(
        [
            *(BUILD_DIR / "dsp").glob("*.o"),
            *(BUILD_DIR / "core").glob("*.o"),
            *(BUILD_DIR / "ui").glob("*.o"),
        ]
    )
    driver_hash = hashlib.sha256(CPP_DRIVER_PATH.read_bytes()).hexdigest()
    newest_obj_mtime = max((o.stat().st_mtime for o in objs), default=0.0)
    need_link = (
        force_rebuild
        or not CLI_BIN.exists()
        or not CLI_STAMP.exists()
        or CLI_STAMP.read_text(encoding="utf-8").strip() != driver_hash
        or CLI_BIN.stat().st_mtime < newest_obj_mtime
    )
    if need_link:
        cxx = os.environ.get("CXX", "clang++")
        subprocess.run(
            [
                cxx,
                "-std=c++17",
                "-O2",
                "-fno-exceptions",
                "-fno-rtti",
                "-fno-math-errno",
                "-ffp-contract=off",
                f"-I{ROOT / 'src'}",
                str(CPP_DRIVER_PATH),
                *(str(o) for o in objs),
                "-o",
                str(CLI_BIN),
            ],
            cwd=ROOT,
            check=True,
        )
        CLI_STAMP.write_text(driver_hash + "\n", encoding="utf-8")
    return CLI_BIN


def compile_screen_script(screen: dict[str, Any], rgb_out_path: pathlib.Path) -> list[str]:
    sid = screen["id"]
    lines = [f"# Screen: {sid}"]
    dumped_custom = False
    for idx, step in enumerate(screen.get("steps", [])):
        op = step.get("op")
        loc = f"{sid} step[{idx}] ({op})"
        if op == "render_generic_keyboard":
            lines.append(f"DUMP_GENERIC_KEYBOARD {rgb_out_path}")
            dumped_custom = True
        elif op == "init_boot":
            lines.append("INIT_BOOT")
        elif op == "init_playing":
            frames = int(step.get("frames", 60))
            scene = int(step.get("scene", 1))
            if scene not in (1, 2, 3, 4):
                raise ValueError(f"{loc}: scene must be 1..4, got {scene}")
            lines.append(f"INIT_PLAYING {frames} {scene}")
        elif op == "arm_boot_intro":
            lines.append("ARM_BOOT_INTRO")
        elif op == "recall_scene":
            scene = int(step["scene"])
            if scene not in (1, 2, 3, 4):
                raise ValueError(f"{loc}: scene must be 1..4, got {scene}")
            lines.append(f"RECALL_SCENE {scene}")
        elif op == "step":
            frames = int(step.get("frames", 1))
            dt = float(step.get("dt", 1.0 / 60.0))
            blocks = int(step.get("dsp_blocks", 12))
            lines.append(f"STEP {frames} {dt:.8f} {blocks}")
        elif op == "tap":
            btn = step["button"]
            if btn not in BUTTONS:
                raise ValueError(f"{loc}: unknown button '{btn}'")
            lines.append(f"TAP {btn}")
        elif op == "button":
            btn = step["button"]
            if btn not in BUTTONS:
                raise ValueError(f"{loc}: unknown button '{btn}'")
            pressed = 1 if step.get("pressed", True) else 0
            lines.append(f"BUTTON {btn} {pressed}")
        elif op == "turn":
            enc = step["encoder"]
            if enc not in ENCODERS:
                raise ValueError(f"{loc}: unknown encoder '{enc}'")
            ticks = int(step["ticks"])
            lines.append(f"TURN {enc} {ticks}")
        elif op == "focus_knob":
            knob = step["knob"]
            if knob not in {"Knob1", "Knob2", "Knob3", "Knob4"}:
                raise ValueError(f"{loc}: focus_knob expects Knob1..Knob4, got '{knob}'")
            lines.append(f"FOCUS_KNOB {knob}")
        elif op == "black_key":
            key = step["key"]
            if key not in BLACK_KEYS:
                raise ValueError(f"{loc}: unknown black key '{key}'")
            pressed = 1 if step.get("pressed", True) else 0
            pressure = float(step.get("pressure", 1.0 if pressed else 0.0))
            lines.append(f"BLACK_KEY {key} {pressed} {pressure:.6f}")
        elif op == "white_key":
            idx = int(step["index"])
            if not (0 <= idx <= 15):
                raise ValueError(f"{loc}: white_key index must be 0..15, got {idx}")
            pressed = 1 if step.get("pressed", True) else 0
            pressure = float(step.get("pressure", 1.0 if pressed else 0.0))
            lines.append(f"WHITE_KEY {idx} {pressed} {pressure:.6f}")
        elif op == "master_pot":
            val = float(step["value"])
            lines.append(f"MASTER_POT {val:.6f}")
        elif op == "set_coupling":
            val = float(step["value"])
            lines.append(f"SET_COUPLING {val:.6f}")
        elif op == "set_voice":
            voice = int(step["voice"])
            if voice not in (1, 2, 3, 4):
                raise ValueError(f"{loc}: voice must be 1..4, got {voice}")
            tokens: list[str] = []
            for k, v in step.items():
                if k in ("op", "voice"):
                    continue
                if k in VOICE_FLOAT_FIELDS:
                    tokens.append(f"{k}={float(v):.6f}")
                elif k in ("lfo_target", "lfo_b_target"):
                    if v not in LFO_TARGETS:
                        raise ValueError(f"{loc}: unknown LFO target '{v}'")
                    tokens.append(f"{k}={v}")
                elif k == "muted":
                    tokens.append(f"muted={1 if v else 0}")
                else:
                    raise ValueError(f"{loc}: unknown set_voice field '{k}'")
            lines.append(f"SET_VOICE {voice} " + " ".join(tokens))
        else:
            raise ValueError(f"{loc}: unsupported step op '{op}'")
    if not dumped_custom:
        lines.append(f"DUMP {rgb_out_path}")
    return lines


def validate_rgb_frame(sid: str, rgb: bytes, checks: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if len(rgb) != 240 * 240 * 3:
        return [f"{sid}: expected {240 * 240 * 3} RGB bytes, got {len(rgb)}"]
    lit_px = 0
    non_bw = 0
    for i in range(0, len(rgb), 3):
        r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
        if r > 0 or g > 0 or b > 0:
            lit_px += 1
        if not (r == g == b):
            non_bw += 1
    if checks.get("pure_bw", True) and non_bw > 0:
        errors.append(f"{sid}: expected pure neutral B&W, found {non_bw} colored pixels")
    min_lit = checks.get("min_lit_px")
    if min_lit is not None and lit_px < int(min_lit):
        errors.append(f"{sid}: lit pixels {lit_px} < min_lit_px {min_lit} (blank/faded frame?)")
    max_lit = checks.get("max_lit_px")
    if max_lit is not None and lit_px > int(max_lit):
        errors.append(f"{sid}: lit pixels {lit_px} > max_lit_px {max_lit} (blown-out frame?)")
    return errors


def load_screens(spec_path: pathlib.Path, filters: list[str]) -> list[dict[str, Any]]:
    data = json.loads(spec_path.read_text(encoding="utf-8"))
    screens: list[dict[str, Any]] = data["screens"]
    seen: set[str] = set()
    for s in screens:
        sid = s["id"]
        if sid in seen:
            raise ValueError(f"Duplicate screen id '{sid}' in {spec_path}")
        seen.add(sid)
    if not filters:
        return screens
    lowered = [f.lower() for f in filters]
    matched = [
        s
        for s in screens
        if any(
            q in s["id"].lower() or q == s.get("category", "").lower()
            for q in lowered
        )
    ]
    if not matched:
        raise SystemExit(f"No screens matched --only {filters}")
    return matched


def audit_doc_references(out_dir: pathlib.Path) -> list[str]:
    errors: list[str] = []
    ref_re = re.compile(r"(?:docs/)?assets/screens/(?:240x240/)?[A-Za-z0-9._-]+\.png")
    candidates = [
        ROOT / "fmhub.json",
        ROOT / "README.md",
        *sorted((ROOT / "docs").glob("*.md")),
    ]
    for doc in candidates:
        if not doc.exists():
            continue
        txt = doc.read_text(encoding="utf-8")
        for m in ref_re.finditer(txt):
            rel = m.group(0)
            tail = rel.split("assets/screens/", 1)[1]
            target = out_dir / tail
            if not target.exists():
                errors.append(f"{doc.relative_to(ROOT)} references missing '{rel}'")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--spec", type=pathlib.Path, default=DEFAULT_SPEC, help="Screen manifest JSON")
    ap.add_argument("--out-dir", type=pathlib.Path, default=DEFAULT_OUT_DIR, help="Output root dir")
    ap.add_argument("--only", nargs="+", default=[], help="Filter by screen id substring or category")
    ap.add_argument("--list", action="store_true", help="List screens in manifest and exit")
    ap.add_argument("--check", action="store_true", help="Verify PNGs are fresh, valid & linked")
    ap.add_argument("--prune", action="store_true", help="Delete orphan PNGs not in manifest")
    ap.add_argument("--force-rebuild", action="store_true", help="Force recompile capture binary")
    args = ap.parse_args()

    all_screens = load_screens(args.spec, [])
    target_screens = load_screens(args.spec, args.only)

    if args.list:
        print(f"Manifest: {args.spec} ({len(target_screens)}/{len(all_screens)} screens)")
        for s in target_screens:
            cat = s.get("category", "-")
            print(f"  {s['id']:<34s} [{cat:<11s}] {s.get('summary', '')}")
        return 0

    out_960 = args.out_dir
    out_240 = out_960 / "240x240"

    if args.prune:
        if args.only:
            raise SystemExit("--prune cannot be combined with --only")
        expected = {f"{s['id']}.png" for s in all_screens}
        removed = 0
        for d in (out_240, out_960):
            if not d.exists():
                continue
            for p in sorted(d.glob("*.png")):
                if p.name not in expected:
                    p.unlink()
                    print(f"  [PRUNED] {p}")
                    removed += 1
        print(f"Pruned {removed} orphan PNG(s).")

    cli_bin = ensure_capture_binary(force_rebuild=args.force_rebuild)

    with tempfile.TemporaryDirectory(prefix="ruido_screens_") as tmp:
        tmp_path = pathlib.Path(tmp)
        script_lines: list[str] = []
        rgb_paths: dict[str, pathlib.Path] = {}
        for s in target_screens:
            rgb_path = tmp_path / f"{s['id']}.rgb"
            rgb_paths[s["id"]] = rgb_path
            script_lines.extend(compile_screen_script(s, rgb_path))

        subprocess.run(
            [str(cli_bin)],
            input="\n".join(script_lines) + "\n",
            text=True,
            cwd=ROOT,
            check=True,
        )

        errors: list[str] = []
        if not args.check:
            out_240.mkdir(parents=True, exist_ok=True)
            out_960.mkdir(parents=True, exist_ok=True)

        for s in target_screens:
            sid = s["id"]
            rgb240 = rgb_paths[sid].read_bytes()
            errors.extend(validate_rgb_frame(sid, rgb240, s.get("checks", {})))

            png240 = encode_rgb_png(rgb240, 240, 240)
            png960 = encode_rgb_png(upscale_4x_nearest(rgb240, 240, 240), 960, 960)
            p240 = out_240 / f"{sid}.png"
            p960 = out_960 / f"{sid}.png"

            if args.check:
                if not p240.exists() or p240.read_bytes() != png240:
                    errors.append(f"{sid}: stale or missing {p240}")
                if not p960.exists() or p960.read_bytes() != png960:
                    errors.append(f"{sid}: stale or missing {p960}")
            else:
                p240.write_bytes(png240)
                p960.write_bytes(png960)
                print(
                    f"  [OK] {sid + '.png':<34s} 240x240={len(png240):6,d} B | 960x960={len(png960):6,d} B"
                )

        if args.check and not args.only:
            expected_names = {f"{s['id']}.png" for s in all_screens}
            for d in (out_240, out_960):
                if d.exists():
                    for p in sorted(d.glob("*.png")):
                        if p.name not in expected_names:
                            errors.append(f"Orphan PNG not in manifest: {p}")
            if out_960.resolve() == DEFAULT_OUT_DIR.resolve():
                errors.extend(audit_doc_references(out_960))

        if errors:
            for err in errors:
                print(f"  [FAIL] {err}", file=sys.stderr)
            return 1

        if args.check:
            print(
                f"[Screens OK] {len(target_screens)} screen pair(s) fresh, pure B&W, and all doc links valid."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
