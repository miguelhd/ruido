#!/usr/bin/env python3
"""Offline vector + Gaussian phosphor halo baker for the Ruido FM-1 CRT boot intro.

Rasterizes the exact `#ruido-logo` SVG paths at 8x8 (64x) subpixel supersampling
at settled `logoScale = 1.00` into 240x240 screen space, computes exact separable
float64 Gaussian blurs (`tight = 1.6x0.992`, `med = 1.5x0.870`, `wide = 1.0x0.800`),
and emits pure C++17 `inline constexpr` ROM tables in `src/ui/BootIntroData.h`
(~13.4 KiB XIP `.rodata`, 0 B SRAM).

Usage:
    python3 tools/bake_boot_intro.py
"""

from __future__ import annotations

import math
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_HEADER = ROOT / "src" / "ui" / "BootIntroData.h"

DISPLAY_W = 240
DISPLAY_H = 240
SUPERSAMPLE = 8

LOGO_BASE_TX = 26.4
LOGO_BASE_TY = 95.15
LOGO_BASE_SCALE = 1.08837
LOGO_USER_SCALE = 1.00
LOGO_USER_OFFSET_Y = 0.0

# Exact RUIDO vector paths (`#ruido-logo`)
RUIDO_SVG_PATHS = (
    # O
    "M171.315 22.0312C171.315 29.1211 169.557 34.5703 166.042 38.3789"
    "C162.526 42.1875 157.487 44.0918 150.925 44.0918C144.362 44.0918 "
    "139.323 42.1875 135.807 38.3789C132.292 34.5703 130.534 29.1016 "
    "130.534 21.9727C130.534 14.8438 132.292 9.4043 135.807 5.6543"
    "C139.343 1.88477 144.401 0 150.983 0C157.565 0 162.595 1.89453 "
    "166.071 5.68359C169.567 9.47266 171.315 14.9219 171.315 22.0312Z"
    "M140.055 22.0312C140.055 26.8164 140.964 30.4199 142.78 32.8418"
    "C144.596 35.2637 147.311 36.4746 150.925 36.4746C158.171 36.4746 "
    "161.794 31.6602 161.794 22.0312C161.794 12.3828 158.19 7.55859 "
    "150.983 7.55859C147.37 7.55859 144.645 8.7793 142.809 11.2207"
    "C140.973 13.6426 140.055 17.2461 140.055 22.0312Z",
    # D
    "M127.761 21.6797C127.761 28.7305 125.75 34.1309 121.726 37.8809"
    "C117.722 41.6309 111.931 43.5059 104.353 43.5059H92.2242V0.673828"
    "H105.671C112.664 0.673828 118.093 2.51953 121.961 6.21094C125.828 "
    "9.90234 127.761 15.0586 127.761 21.6797ZM118.328 21.9141C118.328 "
    "12.7148 114.265 8.11523 106.14 8.11523H101.306V36.0059H105.203"
    "C113.953 36.0059 118.328 31.3086 118.328 21.9141Z",
    # I
    "M76.5609 43.5059V0.673828H85.643V43.5059H76.5609Z",
    # U
    "M70.2726 0.673828V28.3887C70.2726 31.5527 69.5598 34.3262 68.134 "
    "36.709C66.7277 39.0918 64.6867 40.918 62.0109 42.1875C59.3351 "
    "43.457 56.1711 44.0918 52.5187 44.0918C47.0109 44.0918 42.7336 "
    "42.6855 39.6867 39.873C36.6398 37.041 35.1164 33.1738 35.1164 "
    "28.2715V0.673828H44.1691V26.8945C44.1691 30.1953 44.8332 32.6172 "
    "46.1613 34.1602C47.4894 35.7031 49.6867 36.4746 52.7531 36.4746"
    "C55.7219 36.4746 57.8703 35.7031 59.1984 34.1602C60.5461 32.5977 "
    "61.2199 30.1562 61.2199 26.8359V0.673828H70.2726Z",
    # R
    "M9.08203 19.6875H12.0117C14.8828 19.6875 17.002 19.209 18.3691 "
    "18.252C19.7363 17.2949 20.4199 15.791 20.4199 13.7402C20.4199 "
    "11.709 19.7168 10.2637 18.3105 9.4043C16.9238 8.54492 14.7656 "
    "8.11523 11.8359 8.11523H9.08203V19.6875ZM9.08203 27.0703V43.5059"
    "H0V0.673828H12.4805C18.3008 0.673828 22.6074 1.73828 25.4004 "
    "3.86719C28.1934 5.97656 29.5898 9.18945 29.5898 13.5059C29.5898 "
    "16.0254 28.8965 18.2715 27.5098 20.2441C26.123 22.1973 24.1602 "
    "23.7305 21.6211 24.8438C28.0664 34.4727 32.2656 40.6934 34.2188 "
    "43.5059H24.1406L13.916 27.0703H9.08203Z",
)

TOKEN_RE = re.compile(r"[MCHVLZ]|[-+]?(?:\d+\.?\d*|\.\d+)")


def map_x(x: float) -> float:
    bx = LOGO_BASE_TX + x * LOGO_BASE_SCALE
    return 120.0 + (bx - 120.0) * LOGO_USER_SCALE


def map_y(y: float) -> float:
    by = LOGO_BASE_TY + y * LOGO_BASE_SCALE
    return (120.0 + LOGO_USER_OFFSET_Y) + (by - 120.0) * LOGO_USER_SCALE


def flatten_cubic(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    depth: int = 0,
) -> list[tuple[float, float]]:
    if depth >= 6:
        return [p3]
    mx = 0.125 * (p0[0] + 3.0 * p1[0] + 3.0 * p2[0] + p3[0])
    my = 0.125 * (p0[1] + 3.0 * p1[1] + 3.0 * p2[1] + p3[1])
    lx = 0.5 * (p0[0] + p3[0])
    ly = 0.5 * (p0[1] + p3[1])
    if (mx - lx) ** 2 + (my - ly) ** 2 < 0.0001:
        return [p3]
    q0 = (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]))
    q1 = (0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1]))
    q2 = (0.5 * (p2[0] + p3[0]), 0.5 * (p2[1] + p3[1]))
    r0 = (0.5 * (q0[0] + q1[0]), 0.5 * (q0[1] + q1[1]))
    r1 = (0.5 * (q1[0] + q2[0]), 0.5 * (q1[1] + q2[1]))
    s = (0.5 * (r0[0] + r1[0]), 0.5 * (r0[1] + r1[1]))
    return flatten_cubic(p0, q0, r0, s, depth + 1) + flatten_cubic(
        s, r1, q2, p3, depth + 1
    )


def parse_svg_edges() -> list[tuple[float, float, float, float]]:
    polys: list[list[tuple[float, float]]] = []
    for d in RUIDO_SVG_PATHS:
        toks = TOKEN_RE.findall(d)
        i = 0
        cmd: str | None = None
        cx = cy = sx = sy = 0.0
        cur: list[tuple[float, float]] = []
        while i < len(toks):
            if toks[i].isalpha():
                cmd = toks[i]
                i += 1
            if cmd == "M":
                if cur:
                    polys.append(cur)
                    cur = []
                cx, cy = map_x(float(toks[i])), map_y(float(toks[i + 1]))
                i += 2
                sx, sy = cx, cy
                cur.append((cx, cy))
                cmd = "L"
            elif cmd == "L":
                cx, cy = map_x(float(toks[i])), map_y(float(toks[i + 1]))
                i += 2
                cur.append((cx, cy))
            elif cmd == "H":
                cx = map_x(float(toks[i]))
                i += 1
                cur.append((cx, cy))
            elif cmd == "V":
                cy = map_y(float(toks[i]))
                i += 1
                cur.append((cx, cy))
            elif cmd == "C":
                x1, y1 = map_x(float(toks[i])), map_y(float(toks[i + 1]))
                x2, y2 = map_x(float(toks[i + 2])), map_y(float(toks[i + 3]))
                x3, y3 = map_x(float(toks[i + 4])), map_y(float(toks[i + 5]))
                i += 6
                cur.extend(flatten_cubic((cx, cy), (x1, y1), (x2, y2), (x3, y3)))
                cx, cy = x3, y3
            elif cmd == "Z":
                if cur:
                    if cur[-1] != (sx, sy):
                        cur.append((sx, sy))
                    polys.append(cur)
                    cur = []
                cmd = None
        if cur:
            polys.append(cur)

    edges: list[tuple[float, float, float, float]] = []
    for poly in polys:
        for a, b in zip(poly, poly[1:]):
            if a[1] != b[1]:
                edges.append((a[0], a[1], b[0], b[1]))
    return edges


def rasterize_supersampled(edges: list[tuple[float, float, float, float]]) -> list[list[float]]:
    img = [[0.0] * DISPLAY_W for _ in range(DISPLAY_H)]
    inv_ss2 = 1.0 / (SUPERSAMPLE * SUPERSAMPLE)

    for py in range(DISPLAY_H):
        row_acc = [0] * DISPLAY_W
        for sy in range(SUPERSAMPLE):
            sample_y = py + (sy + 0.5) / SUPERSAMPLE
            crossings: list[tuple[float, int]] = []
            for x0, y0, x1, y1 in edges:
                if (y0 <= sample_y < y1) or (y1 <= sample_y < y0):
                    dw = 1 if y1 > y0 else -1
                    ix = x0 + (sample_y - y0) * (x1 - x0) / (y1 - y0)
                    crossings.append((ix, dw))
            if not crossings:
                continue
            crossings.sort(key=lambda c: c[0])
            nc = len(crossings)
            c_idx = 0
            winding = 0
            for px in range(DISPLAY_W):
                base_x = float(px)
                for sx in range(SUPERSAMPLE):
                    sample_x = base_x + (sx + 0.5) / SUPERSAMPLE
                    while c_idx < nc and crossings[c_idx][0] < sample_x:
                        winding += crossings[c_idx][1]
                        c_idx += 1
                    if winding != 0:
                        row_acc[px] += 1
        for px in range(DISPLAY_W):
            if row_acc[px]:
                img[py][px] = row_acc[px] * inv_ss2
    return img


def gaussian_blur_2d(
    src: list[list[float]], sigma_x: float, sigma_y: float
) -> list[list[float]]:
    def make_kernel(sigma: float) -> tuple[int, list[float]]:
        if sigma <= 0.05:
            return 0, [1.0]
        rad = max(1, int(math.ceil(sigma * 3.5)))
        weights = [math.exp(-0.5 * (k / sigma) ** 2) for k in range(-rad, rad + 1)]
        inv_sum = 1.0 / sum(weights)
        return rad, [w * inv_sum for w in weights]

    rx, kx = make_kernel(sigma_x)
    ry, ky = make_kernel(sigma_y)

    tmp = [[0.0] * DISPLAY_W for _ in range(DISPLAY_H)]
    for y in range(DISPLAY_H):
        srow = src[y]
        trow = tmp[y]
        for x in range(DISPLAY_W):
            acc = 0.0
            for k_i, w in enumerate(kx):
                sx = x + (k_i - rx)
                if 0 <= sx < DISPLAY_W:
                    acc += srow[sx] * w
            trow[x] = acc

    out = [[0.0] * DISPLAY_W for _ in range(DISPLAY_H)]
    for y in range(DISPLAY_H):
        orow = out[y]
        for k_i, w in enumerate(ky):
            sy = y + (k_i - ry)
            if 0 <= sy < DISPLAY_H:
                trow = tmp[sy]
                for x in range(DISPLAY_W):
                    orow[x] += trow[x] * w
    return out


def format_u8_array(name: str, data: list[int]) -> str:
    lines = [f"inline constexpr std::array<uint8_t, {len(data)}> {name} = {{"]
    per_line = 16
    for i in range(0, len(data), per_line):
        chunk = ", ".join(f"{v}u" for v in data[i : i + per_line])
        lines.append(f"    {chunk},")
    lines.append("};")
    return "\n".join(lines)


def main() -> None:
    edges = parse_svg_edges()
    core = rasterize_supersampled(edges)

    # Exact SVG filter sigmas matching `ruido_intro_editor.html`:
    #   blurTight = (1.6, 1.6 * 0.62 = 0.992)
    #   blurMed   = (1.5, 1.5 * 0.58 = 0.870)
    #   blurWide  = (1.0, max(0.8, 1.0 * 0.09) = 0.800)
    b_tight = gaussian_blur_2d(core, 1.6, 1.6 * 0.62)
    b_med = gaussian_blur_2d(core, 1.5, 1.5 * 0.58)
    b_wide = gaussian_blur_2d(core, 1.0, max(0.8, 1.0 * 0.09))

    # 1. Core 4-bit anti-aliased glyph bitmap (`x in [10..229]`, `y in [90..147]` -> 220 x 58)
    core_x0, core_w = 10, 220
    core_y0, core_h = 90, 58
    assert core_w % 2 == 0
    core_packed: list[int] = []
    for y in range(core_y0, core_y0 + core_h):
        for x in range(core_x0, core_x0 + core_w, 2):
            n0 = min(15, max(0, int(round(core[y][x] * 15.0))))
            n1 = min(15, max(0, int(round(core[y][x + 1] * 15.0))))
            core_packed.append((n0 << 4) | n1)

    # 2. Near-field phosphor bloom `1 - (1 - b_tight)(1 - b_med)` at 2x2 step
    #    Covers y in [82..157] (76 px -> 39 grid rows @ 2px stride), x in [0..240] (121 grid cols @ 2px stride)
    near_y0, near_rows = 82, 39
    near_cols = 121  # x = 0, 2, ..., 240 (clamped to 239)
    bloom_near: list[int] = []
    for gr in range(near_rows):
        py = min(DISPLAY_H - 1, near_y0 + gr * 2)
        for gc in range(near_cols):
            px = min(DISPLAY_W - 1, gc * 2)
            bt = b_tight[py][px]
            bm = b_med[py][px]
            merged = 1.0 - (1.0 - bt) * (1.0 - bm)
            bloom_near.append(min(255, max(0, int(round(merged * 255.0)))))

    # 3. Wide horizontal electron-beam bleed `b_wide` (sigma_x = 20.0, sigma_y = 2.5) at 4x2 step
    #    Covers y in [78..161] (84 px -> 43 grid rows @ 2px stride), x in [0..240] (61 grid cols @ 4px stride)
    wide_y0, wide_rows = 78, 43
    wide_cols = 61  # x = 0, 4, ..., 240 (clamped to 239)
    bloom_wide: list[int] = []
    for gr in range(wide_rows):
        py = min(DISPLAY_H - 1, wide_y0 + gr * 2)
        for gc in range(wide_cols):
            px = min(DISPLAY_W - 1, gc * 2 * 2)
            bw = b_wide[py][px]
            bloom_wide.append(min(255, max(0, int(round(bw * 255.0)))))

    total_bytes = len(core_packed) + len(bloom_near) + len(bloom_wide)

    header = f"""#pragma once

// Auto-generated by `tools/bake_boot_intro.py` — DO NOT EDIT MANUALLY.
// Baked RUIDO CRT boot wordmark (`logoScale = 1.00`) + separable Gaussian phosphor bloom fields
// (`{total_bytes}` bytes XIP `.rodata`, `0` bytes SRAM).

#include <array>
#include <cstdint>

namespace ruido::intro_data {{

inline constexpr int kCoreX0 = {core_x0};
inline constexpr int kCoreY0 = {core_y0};
inline constexpr int kCoreW = {core_w};
inline constexpr int kCoreH = {core_h};
inline constexpr int kCoreRowBytes = {core_w // 2};

inline constexpr int kNearY0 = {near_y0};
inline constexpr int kNearStepX = 2;
inline constexpr int kNearStepY = 2;
inline constexpr int kNearCols = {near_cols};
inline constexpr int kNearRows = {near_rows};

inline constexpr int kWideY0 = {wide_y0};
inline constexpr int kWideStepX = 4;
inline constexpr int kWideStepY = 2;
inline constexpr int kWideCols = {wide_cols};
inline constexpr int kWideRows = {wide_rows};

{format_u8_array("kLogoCore4Bit", core_packed)}

{format_u8_array("kBloomNear8Bit", bloom_near)}

{format_u8_array("kBloomWide8Bit", bloom_wide)}

}} // namespace ruido::intro_data
"""
    OUT_HEADER.write_text(header, encoding="utf-8")
    print(
        f"[bake_boot_intro] Wrote {OUT_HEADER.relative_to(ROOT)}: "
        f"{total_bytes:,} B ({total_bytes / 1024.0:.2f} KiB) .rodata "
        f"[core={len(core_packed)} B, near={len(bloom_near)} B, wide={len(bloom_wide)} B]"
    )


if __name__ == "__main__":
    main()
