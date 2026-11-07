"""Stage Dinghy platform sources and incrementally build standalone RAM `loader.bin` (`ota.bin`)."""

from __future__ import annotations

import hashlib
import pathlib
import struct
import subprocess

from fm1_common import (
    FM1_DIR,
    GEN_DIR,
    LDR_DIR,
    LOADER_LOAD,
    LOADER_NAME,
    OTA_MAX_BYTES,
    REQUIRED_JEDEC_IDS,
    CompileSpec,
    compile_cached_jobs,
    crc16_ccitt,
    ldr_head,
    run_cmd,
    write_text_if_changed,
)


def replace_once(src: str, old: str, new: str, label: str) -> str:
    cnt = src.count(old)
    if cnt != 1:
        raise SystemExit(f"[FM-1 FAIL] expected 1 occurrence of {label}, found {cnt}")
    return src.replace(old, new, 1)


def stage_platform_sources(fw: pathlib.Path) -> None:
    """Stage `platform_ruido.c` and `recovery_ruido.c` into gitignored `build/fm1/gen/`."""
    rec_src = (fw / "src" / "recovery.c").read_text(encoding="utf-8")
    rec_src = replace_once(
        rec_src,
        "    lcd_init();\n"
        "    lcd_fill(0, 0, 240, 240, C_BLACK);\n"
        '    draw_text_box(0, 64, 240, &FONT_S, APP_NAME " USB RESCUE", C_WHITE, 1);\n'
        '    draw_text_box(0, 96, 240, &FONT_S, "CONNECT USB", C_WHITE, 1);\n'
        "    draw_text_box(0, 124, 240, &FONT_S,\n"
        '                  flash_ok ? "OPEN THE INSTALLER" : "UNKNOWN FLASH", C_WHITE, 1);\n'
        '    draw_text_box(0, 164, 240, &FONT_S, "AUDIO OFF", C_WHITE, 1);\n'
        "    lcd_sync();",
        "    lcd_init();\n    ruido_fm1_plat_rescue_show(flash_ok);",
        "recovery_main screen block",
    )
    if "draw_text_box" in rec_src:
        raise SystemExit("[FM-1 FAIL] recovery_ruido.c still references draw_text_box")
    write_text_if_changed(GEN_DIR / "recovery_ruido.c", rec_src)

    plat_src = (fw / "src" / "platform.c").read_text(encoding="utf-8")
    plat_src = replace_once(
        plat_src,
        "static void ota_show(uint32_t step, int32_t code)\n"
        "{\n"
        "#if BOARD_SMK37   /* step 5 stays on screen while the loader writes the flash: say so */\n"
        '    static const char *const STEP[] = {"", "PACKAGE", "CHECK HEAD", "LOADER", "CONFIRM", "UPDATING..."};\n'
        "#else\n"
        '    static const char *const STEP[] = {"", "PACKAGE", "CHECK HEAD", "LOADER", "CONFIRM", "RESTART"};\n'
        "#endif\n"
        "    if (recovery_active) return;\n"
        "    lcd_fill(0, 0, 240, 240, C_BLACK);\n"
        '    draw_text_box(0, 92, 240, &FONT_S, "UPDATE", C_WHITE, 1);\n'
        "    if (step < 9u)\n"
        "        draw_text_box(0, 124, 240, &FONT_S, STEP[step < 6u ? step : 0], C_WHITE, 1);\n"
        "    else\n"
        '        draw_text_box(0, 124, 240, &FONT_S, code == 1 ? "DRY RUN OK" : "FAILED", C_WHITE, 1);\n'
        "}",
        "static void ota_show(uint32_t step, int32_t code)\n"
        "{\n"
        "    ruido_fm1_plat_ota_show(step, code);\n"
        "}",
        "ota_show screen block",
    )
    plat_src = replace_once(
        plat_src,
        '#include "recovery.c"',
        '#include "recovery_ruido.c"',
        "recovery.c include",
    )
    plat_src = replace_once(
        plat_src,
        "        lcd_fill(0, 0, 240, 240, C_BLACK);\n"
        '        draw_text_box(0, 110, 240, &FONT_S, "UBOOT (USB)", C_WHITE, 1);',
        "        ruido_fm1_plat_uboot_show();",
        "uboot_req screen block",
    )
    plat_src = replace_once(
        plat_src,
        "static void fm1_fault(const fm1_crash_t *c)\n"
        "{\n"
        "    char b[12];\n"
        "    uint32_t i, t0, v[3] = {c->vec, c->pc, c->rets};\n"
        "    fm1_audio_stop();\n"
        "    lcd_fill(0, 0, 240, 240, RGB(160, 0, 0));\n"
        '    draw_text_box(0, 20, 240, &FONT_S, APP_NAME " CRASH", C_WHITE, 1);\n'
        "    for (i = 0; i < 3u; i++) {\n"
        "        uint32_t k;\n"
        '        for (k = 0; k < 8u; k++) b[k] = "0123456789ABCDEF"[(v[i] >> (28u - 4u * k)) & 15u];\n'
        "        b[8] = 0;\n"
        "        draw_text_box(10, 60 + i * 20, 220, &FONT_S, b, C_WHITE, 0);\n"
        "    }\n"
        "    t0 = fm1_ticks();\n"
        "    while ((uint32_t)(fm1_ticks() - t0) < 8000u * 1000u * FM1_TICKS_PER_US)\n"
        "        fm1_wdt_feed();\n"
        "    fm1_reboot();\n"
        "}",
        "static void fm1_fault(const fm1_crash_t *c)\n"
        "{\n"
        "    uint32_t t0;\n"
        "    fm1_audio_stop();\n"
        "    ruido_fm1_plat_fault_show(c->vec, c->pc, c->rets);\n"
        "    t0 = fm1_ticks();\n"
        "    while ((uint32_t)(fm1_ticks() - t0) < 8000u * 1000u * FM1_TICKS_PER_US)\n"
        "        fm1_wdt_feed();\n"
        "    fm1_reboot();\n"
        "}",
        "fm1_fault screen block",
    )
    if "draw_text_box(" in plat_src:
        raise SystemExit("[FM-1 FAIL] platform_ruido.c still calls draw_text_box")
    write_text_if_changed(GEN_DIR / "platform_ruido.c", plat_src)


def detect_ldr_size_opt(clang_bin: str, cxx_std: str) -> str:
    res = subprocess.run(
        [clang_bin, "-target", "pi32v2", "-x", "c++", cxx_std, "-Oz", "-fsyntax-only", "-"],
        input="int kOptTest = 0;\n",
        capture_output=True,
        text=True,
        check=False,
    )
    return "-Oz" if res.returncode == 0 else "-Os"


def build_loader(
    tc: pathlib.Path,
    dinghy_dir: pathlib.Path,
    lz4blk_mod,
    cxx_std: str,
    file_sha_cache: dict[pathlib.Path, str],
    rebuild: bool = False,
) -> tuple[bytes, int, int]:
    LDR_DIR.mkdir(parents=True, exist_ok=True)
    clang = str(tc / "pi32v2" / "bin" / "clang")
    ld = str(tc / "pi32v2" / "bin" / "ld")
    objcopy = str(tc / "common" / "bin" / "objcopy")
    objdump = str(tc / "common" / "bin" / "objdump")
    fw = dinghy_dir / "firmware"
    ldr_src = fw / "loader"

    ldr_c_text = (ldr_src / "loader.c").read_text(encoding="utf-8")
    if ldr_c_text.count('#include "../src/') < 3:
        raise SystemExit("[FM-1 FAIL] unexpected ../src/ includes in loader.c")
    ldr_c_text = ldr_c_text.replace('#include "../src/', '#include "')
    ldr_c_text = replace_once(
        ldr_c_text,
        '#include "ldr_screen.c"',
        '#include "fm1_ldr_screen.h"',
        "loader.c ldr_screen.c include",
    )
    staged_ldr_c = LDR_DIR / "loader_ruido.c"
    write_text_if_changed(staged_ldr_c, ldr_c_text)

    cflags = [
        "-target",
        "pi32v2",
        "-Os",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-builtin",
        "-Wall",
        "-Wno-unused-function",
        "-DLCD_BAUD=1u",
        f"-I{FM1_DIR}",
        f"-I{ldr_src}",
        f"-I{fw / 'hal'}",
        f"-I{fw / 'src'}",
    ]
    ldr_opt_flag = detect_ldr_size_opt(clang, cxx_std)
    ldr_cxxflags = [
        "-target",
        "pi32v2",
        cxx_std,
        ldr_opt_flag,
        "-DRUIDO_LDR_SUBSET=1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wno-missing-braces",
        "-Wglobal-constructors",
        "-Wexit-time-destructors",
        "-fno-exceptions",
        "-fno-rtti",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-threadsafe-statics",
        "-fno-builtin",
        "-nostdinc",
        "-nostdinc++",
        "-isystem",
        "platforms/fm1/libc",
        "-Isrc",
        f"-I{FM1_DIR}",
    ]

    crt0_o = LDR_DIR / "crt0_ldr.o"
    ldr_o = LDR_DIR / "loader.o"
    ops_o = LDR_DIR / "fm1_ops_screen.o"
    ldr_specs = [
        CompileSpec(
            ldr_src / "crt0_ldr.S",
            crt0_o,
            [clang, "-target", "pi32v2", "-c", str(ldr_src / "crt0_ldr.S"), "-o", str(crt0_o)],
            use_depfile=False,
        ),
        CompileSpec(
            staged_ldr_c,
            ldr_o,
            [clang, *cflags, "-c", str(staged_ldr_c), "-o", str(ldr_o)],
        ),
        CompileSpec(
            FM1_DIR / "fm1_ops_screen.cpp",
            ops_o,
            [clang, *ldr_cxxflags, "-c", str(FM1_DIR / "fm1_ops_screen.cpp"), "-o", str(ops_o)],
        ),
    ]
    hits, built = compile_cached_jobs(ldr_specs, file_sha_cache, rebuild=rebuild)

    ldr_ld = ldr_src / "loader.ld"
    ldr_bin = LDR_DIR / "loader.bin"
    ota_path = LDR_DIR / "ota.bin"
    link_stamp = LDR_DIR / "ota.sha256"
    link_h = hashlib.sha256()
    for p in (crt0_o, ldr_o, ops_o, ldr_ld):
        link_h.update(hashlib.sha256(p.read_bytes()).digest())
    link_digest = link_h.hexdigest()

    if not rebuild and ldr_bin.is_file() and ota_path.is_file() and link_stamp.is_file():
        if link_stamp.read_text(encoding="ascii").strip() == link_digest:
            return ota_path.read_bytes(), hits, built

    elf = LDR_DIR / "loader.elf"
    run_cmd([
        ld,
        "--gc-sections",
        "-e",
        "_start",
        "-T",
        str(ldr_ld),
        str(crt0_o),
        str(ldr_o),
        str(ops_o),
        "-o",
        str(elf),
    ])
    hdr = run_cmd([objdump, "-h", str(elf)])
    for ln in hdr.splitlines():
        parts = ln.split()
        if len(parts) > 4 and parts[1].startswith(".") and int(parts[3], 16) >= 0x02000000 and int(parts[2], 16):
            raise SystemExit(f"[FM-1 FAIL] loader section {parts[1]} placed in XIP ({parts[3]})")
        if len(parts) > 3 and parts[0].isdigit() and int(parts[2], 16) > 0:
            if any(parts[1].startswith(bad) for bad in (".init_array", ".ctors", ".dtors", ".fini_array")):
                raise SystemExit(f"[FM-1 FAIL] loader contains global constructor section {parts[1]}")
    run_cmd([objcopy, "-O", "binary", "-j", ".text", str(elf), str(ldr_bin)])
    image = ldr_bin.read_bytes()
    if not image or len(image) > 0x14000:
        raise SystemExit(f"[FM-1 FAIL] loader size invalid ({len(image)} B)")
    if b"FELUCCA-LOADER-1" not in image[:64]:
        raise SystemExit("[FM-1 FAIL] loader.bin missing FELUCCA-LOADER-1 entry signature at 0x01C0A800")
    if (b"firmware" + b"hub") in image.lower():
        raise SystemExit("[FM-1 FAIL] loader.bin contains forbidden third-party URL string")
    for chip_name, jedec_id, le_bytes in REQUIRED_JEDEC_IDS:
        if le_bytes not in image:
            raise SystemExit(
                f"[FM-1 FAIL] update loader (`loader.bin`) missing {chip_name} JEDEC ID {jedec_id:#08x}"
            )

    blocks = bytearray()
    for i in range(0, len(image), 4096):
        raw = image[i : i + 4096]
        comp = lz4blk_mod.compress(raw)
        if len(comp) > 4096 or lz4blk_mod.decompress(comp) != raw:
            raise SystemExit(f"[FM-1 FAIL] loader LZ4 block {i // 4096} failed round-trip")
        blocks += struct.pack("<II", len(comp), len(raw)) + comp
    inner = ldr_head(crc16_ccitt(image), len(image), LOADER_LOAD, 0, LOADER_NAME)
    body = inner + bytes(blocks)
    ota = ldr_head(crc16_ccitt(body), 0x20, len(body), 0x41, LOADER_NAME) + body
    if len(ota) > OTA_MAX_BYTES:
        raise SystemExit(f"[FM-1 FAIL] ota.bin ({len(ota)} B) exceeds OTA staging limit ({OTA_MAX_BYTES} B)")
    ota_path.write_bytes(ota)
    link_stamp.write_text(link_digest + "\n", encoding="ascii")
    return ota, hits, built
