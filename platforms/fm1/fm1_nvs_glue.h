/* NVS (`USR` Flash `0xEA000..0xFC000`) helper glue for `fm1_main.c`.
 * Wraps `blob_save`, `fl_write`, and `st_read` with `ruido_fm1_flash_hold_begin`/`end` and
 * readback verification while leaving `dinghy`'s boot layout and OTA session intact. */

#pragma once

static void prepare_for_nvs_erase(void) {
    /* Finish any in-flight SPI1 LCD DMA strip and silence ALNK0 DMA halves so the ~35 ms IRQ-masked
     * 4 KiB sector erase outputs clean silence instead of looping an 11.6 ms (86 Hz) grain. */
    lcd_sync();
    memset(abuf, 0, sizeof(abuf));
}

static int nvs_sector_base(uint32_t sec, uint32_t* out_base) {
    if (sec < 6u && sec < NBLOB) {
        *out_base = BLOBS_AT + sec * 0x1000u;
        return 0;
    }
    return -1;
}

static int nvs_verify_readback_locked(uint32_t sec, uint32_t flash_off, const void* expected,
                                      uint32_t n) {
    uint32_t buf[32];
    if (n == 0u || n > sizeof(buf) || st_read(flash_off, buf, n) != 0) {
        return -5;
    }
    const uint32_t idx = (sec == 1u && n == 128u)   ? 0u
                         : (sec == 4u && n == 16u)  ? 32u
                         : (sec == 5u && n == 128u) ? 36u
                                                    : 0xFFu;
    if (idx != 0xFFu) {
        for (uint32_t w = 0u; w < (n >> 2u); ++w) {
            g_ruido_fm1_status.nvs_flash_readback[idx + w] = buf[w];
        }
    }
    return (memcmp(buf, expected, n) == 0) ? 0 : -5;
}

static int nvs_save_c1_safe(uint32_t sec, const void* p, uint32_t n) {
    uint32_t base = 0u;
    ruido_fm1_flash_hold_begin();
    int rc = blob_save(sec, p, n);
    if (rc == 0 && nvs_sector_base(sec, &base) == 0) {
        rc = nvs_verify_readback_locked(sec, base + 12u, p, n);
    }
    ruido_fm1_flash_hold_end();
    return rc;
}

static int nvs_tail_read_boot(uint32_t sec, uint32_t off, void* dst, uint32_t n) {
    uint32_t base = 0u;
    const uint32_t min_off = (sec == 4u) ? 0x20u : 0x100u;
    if (!flash_ok || n == 0u || off < min_off || off + n > 0x1000u ||
        nvs_sector_base(sec, &base) != 0) {
        return -1;
    }
    return st_read(base + off, dst, n);
}

static int nvs_tail_write_c1_safe(uint32_t sec, uint32_t off, const void* src, uint32_t n) {
    uint32_t base = 0u;
    const uint32_t min_off = (sec == 4u) ? 0x20u : 0x100u;
    const uint32_t req_len = (sec == 4u) ? 16u : 128u;
    if (!flash_ok || off < min_off || n != req_len || (off & (req_len - 1u)) != 0u ||
        off + n > 0x1000u || nvs_sector_base(sec, &base) != 0) {
        return -1;
    }
    ruido_fm1_flash_hold_begin();
    int rc = fl_write(base + off, src, n);
    if (rc == 0) {
        rc = nvs_verify_readback_locked(sec, base + off, src, n);
    }
    ruido_fm1_flash_hold_end();
    return rc;
}
