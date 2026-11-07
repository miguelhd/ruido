#!/usr/bin/env python3
"""Automated modularity, layer-isolation, Wasm-sync, and formatting gate for Ruido."""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "src"
TESTS_DIR = ROOT / "tests"
MAKEFILE = ROOT / "Makefile"
WASM_BRIDGE = ROOT / "platforms" / "sim" / "wasm_bridge.cpp"
WASM_ABI_CPP = ROOT / "platforms" / "sim" / "WasmAbiLayout.h"
WASM_ABI_JS = ROOT / "platforms" / "sim" / "web" / "wasm-abi.js"

MAX_HANDWRITTEN_LINES = 700
LINE_CAP_EXEMPT = {
    pathlib.Path("src/ui/BootIntroData.h"),
    pathlib.Path("src/ui/FontSansAA.h"),
}

FORBIDDEN_SYSTEM_HEADERS = {
    "cmath",
    "math.h",
    "vector",
    "map",
    "unordered_map",
    "set",
    "unordered_set",
    "string",
    "sstream",
    "iostream",
    "fstream",
    "functional",
    "memory",
    "thread",
    "mutex",
}

FORBIDDEN_PLATFORM_PREFIXES = (
    "emscripten",
    "AudioToolbox",
    "CoreFoundation",
    "FreeRTOS",
    "system/",
    "asm/",
)

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.MULTILINE)


def find_clang_format() -> str | None:
    direct = shutil.which("clang-format")
    if direct:
        return direct
    if shutil.which("xcrun"):
        res = subprocess.run(
            ["xcrun", "--find", "clang-format"],
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
    return None


def parse_makefile_var(text: str, name: str) -> list[str]:
    pattern = re.compile(
        rf"^{re.escape(name)}\s*:=\s*((?:.*\\\n)*.*)$", re.MULTILINE
    )
    match = pattern.search(text)
    if not match:
        return []
    raw = match.group(1).replace("\\\n", " ")
    return [tok for tok in raw.split() if tok]


def main() -> int:
    errors: list[str] = []

    src_files = sorted(
        p for p in SRC_DIR.rglob("*") if p.suffix in {".h", ".cpp"} and p.is_file()
    )
    test_files = sorted(
        p for p in TESTS_DIR.rglob("*") if p.suffix in {".h", ".cpp"} and p.is_file()
    )
    fm1_dir = ROOT / "platforms" / "fm1"
    fm1_c_files = sorted(
        p for p in fm1_dir.glob("*") if p.suffix in {".h", ".c", ".cpp"} and p.is_file()
    )
    fm1_py_files = sorted(
        p for p in fm1_dir.glob("*.py") if p.is_file()
    )
    tool_py_files = sorted(
        p for p in (ROOT / "tools").glob("*.py") if p.is_file()
    )

    for path in [*fm1_c_files, *fm1_py_files, *tool_py_files]:
        rel = path.relative_to(ROOT)
        text = path.read_text(encoding="utf-8")
        line_count = text.count("\n") + (
            1 if text and not text.endswith("\n") else 0
        )
        if rel not in LINE_CAP_EXEMPT and line_count > MAX_HANDWRITTEN_LINES:
            errors.append(
                f"[line-cap] {rel} has {line_count} lines (max {MAX_HANDWRITTEN_LINES})"
            )

    # 1. File line-count cap & include/layer isolation checks
    for path in src_files:
        rel = path.relative_to(ROOT)
        text = path.read_text(encoding="utf-8")
        line_count = text.count("\n") + (
            1 if text and not text.endswith("\n") else 0
        )
        if rel not in LINE_CAP_EXEMPT and line_count > MAX_HANDWRITTEN_LINES:
            errors.append(
                f"[line-cap] {rel} has {line_count} lines (max {MAX_HANDWRITTEN_LINES})"
            )

        layer = rel.parts[1] if len(rel.parts) >= 3 else ""
        for bracket, inc in INCLUDE_RE.findall(text):
            if bracket == "<" and inc in FORBIDDEN_SYSTEM_HEADERS:
                errors.append(
                    f"[forbidden-header] {rel} includes <{inc}> (violates embedded C++17 subset)"
                )
            if any(inc.startswith(pfx) for pfx in FORBIDDEN_PLATFORM_PREFIXES):
                errors.append(
                    f"[platform-leak] {rel} includes {inc} inside portable src/"
                )
            if bracket == '"':
                if layer == "core" and (
                    inc.startswith("dsp/") or inc.startswith("ui/")
                ):
                    errors.append(
                        f"[layer-dag] {rel} (core) must not include '{inc}'"
                    )
                elif layer == "dsp" and (
                    inc.startswith("ui/")
                    or inc
                    in {
                        "core/ControlSurface.h",
                        "core/EncoderMath.h",
                        "core/GlobalConfig.h",
                        "core/HudEnvelope.h",
                        "core/KeyboardEngine.h",
                        "core/KeyboardModel.h",
                        "core/KeyboardModelSpec.h",
                        "core/KeyPunchSpec.h",
                        "core/LedEngine.h",
                        "core/MidiClock.h",
                        "core/ModifierTracker.h",
                        "core/RuidoController.h",
                        "core/SnapshotBank.h",
                        "core/VoiceCircuit.h",
                    }
                ):
                    errors.append(
                        f"[layer-dag] {rel} (dsp) must not include '{inc}'"
                    )
                elif layer == "ui" and (
                    inc.startswith("dsp/")
                    or inc
                    in {
                        "core/KeyboardEngine.h",
                        "core/KeyboardModel.h",
                        "core/LedEngine.h",
                        "core/MidiClock.h",
                        "core/ModifierTracker.h",
                        "core/RuidoController.h",
                        "core/SnapshotBank.h",
                    }
                ):
                    errors.append(
                        f"[layer-dag] {rel} (ui) must not include '{inc}'"
                    )

    # 2. Makefile <-> wasm_bridge.cpp unity-build source synchronization
    mk_text = MAKEFILE.read_text(encoding="utf-8")
    mk_srcs = set(
        parse_makefile_var(mk_text, "DSP_SRCS")
        + parse_makefile_var(mk_text, "UI_SRCS")
    )
    wasm_text = WASM_BRIDGE.read_text(encoding="utf-8")
    wasm_srcs = {
        f"src/{inc}"
        for bracket, inc in INCLUDE_RE.findall(wasm_text)
        if bracket == '"' and inc.endswith(".cpp")
    }
    if mk_srcs != wasm_srcs:
        missing_in_wasm = sorted(mk_srcs - wasm_srcs)
        extra_in_wasm = sorted(wasm_srcs - mk_srcs)
        errors.append(
            f"[wasm-sync] Makefile vs wasm_bridge.cpp mismatch: "
            f"missing_in_wasm={missing_in_wasm}, extra_in_wasm={extra_in_wasm}"
        )

    # 3. Wasm C++ <-> JS ABI schema synchronization
    if not WASM_ABI_CPP.is_file() or not WASM_ABI_JS.is_file():
        errors.append("[wasm-abi] WasmAbiLayout.h or wasm-abi.js missing")
    else:
        abi_cpp_text = WASM_ABI_CPP.read_text(encoding="utf-8")
        abi_js_text = WASM_ABI_JS.read_text(encoding="utf-8")
        for cpp_ns, js_obj in (
            ("WasmTelemetryIdx", "WASM_TELEMETRY"),
            ("WasmSummaryIdx", "WASM_SUMMARY"),
        ):
            cpp_block = re.search(
                rf"namespace\s+{cpp_ns}\s*\{{([^}}]+)\}}", abi_cpp_text
            )
            js_block = re.search(
                rf"export\s+const\s+{js_obj}\s*=\s*Object\.freeze\(\{{([^}}]+)\}}\)",
                abi_js_text,
            )
            if not cpp_block or not js_block:
                errors.append(
                    f"[wasm-abi] Missing {cpp_ns} in WasmAbiLayout.h or {js_obj} in wasm-abi.js"
                )
                continue
            cpp_map = {
                re.sub(r"(?<!^)(?=[A-Z])", "_", k[1:]).upper(): int(v)
                for k, v in re.findall(
                    r"constexpr\s+size_t\s+(k[A-Za-z0-9]+)\s*=\s*(\d+)u?\s*;",
                    cpp_block.group(1),
                )
            }
            js_map = {
                k: int(v)
                for k, v in re.findall(
                    r"([A-Z0-9_]+)\s*:\s*(\d+)", js_block.group(1)
                )
            }
            if cpp_map != js_map:
                errors.append(
                    f"[wasm-abi] {cpp_ns} != {js_obj}: cpp={cpp_map} vs js={js_map}"
                )

    # 4. .clang-format check
    cf_bin = find_clang_format()
    if not cf_bin:
        errors.append("[clang-format] clang-format binary not found")
    else:
        cf_targets = [str(p) for p in (src_files + test_files + fm1_c_files)]
        res = subprocess.run(
            [cf_bin, "--dry-run", "--Werror", *cf_targets],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        if res.returncode != 0:
            snippet = "\n".join(res.stderr.splitlines()[:20])
            errors.append(
                f"[clang-format] formatting drift detected (run 'make format'):\n{snippet}"
            )

    if errors:
        print("=== Ruido Code Hygiene Check FAILED ===", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1

    max_src = max(
        (
            (p.read_text(encoding="utf-8").count("\n"), p.relative_to(ROOT))
            for p in src_files
            if p.relative_to(ROOT) not in LINE_CAP_EXEMPT
        ),
        default=(0, pathlib.Path(".")),
    )
    print(
        f"[Hygiene OK] {len(src_files)} src files (largest handwritten: {max_src[1]} @ "
        f"{max_src[0]}/{MAX_HANDWRITTEN_LINES} lines), DAG clean, Wasm synced, clang-format clean."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
