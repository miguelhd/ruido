/* First-party dual-core linker layout for Ruido on M-VAVE FM-1 (JieLi AC791N / pi32v2).
 *
 * Flash / XIP anti-brick invariants:
 *   - App entry stays at 0x02000120 (flash offset 0x4000, strictly clear of 0x0000..0x3FFF U-Boot).
 *   - XIP window stays within 0x8DFBC (567.9 KiB APP_SLOT, < 0xEA000 USR flash boundary).
 *
 * Internal SRAM (512 KiB contiguous 0x01C00000..0x01C80000) partition:
 *   - 0x01C00000..0x01C06000 ( 24 KiB): .ram_text (SPI0 NOR flash driver while XIP is suspended)
 *   - 0x01C06000..0x01C08000 (  8 KiB): reserved boot/debug area (0x01C07F00 STATUS)
 *   - 0x01C08000..0x01C0E000 ( 24 KiB): .data + .bss
 *   - 0x01C0E000..0x01C20000 ( 72 KiB): .c1_text (Core 1 DSP .text + .rodata in zero-wait SRAM)
 *   - 0x01C20000..0x01C70000 (320 KiB): .pool (cache-line-isolated shared buses, Core 1/0 arenas)
 *   - 0x01C70000..0x01C74000 ( 16 KiB): Core 1 guarded user + supervisor stacks
 *   - 0x01C74000..0x01C7C000 ( 32 KiB): Core 0 guarded user + supervisor stacks
 *   - 0x01C7C000..0x01C7FD50 (15.3KiB): .noinit (bootguard + crash telemetry across resets)
 *   - 0x01C7FD50..0x01C80000 (688 B  ): reserved boot_info, vector table, 0x01C7FFF8 Core 1 entry
 */

MEMORY
{
    XIP     (rx)  : ORIGIN = 0x02000120, LENGTH = 0x8DFBC
    RAMTEXT (rwx) : ORIGIN = 0x01C00000, LENGTH = 0x6000
    RAM     (rwx) : ORIGIN = 0x01C08000, LENGTH = 24K
    C1TEXT  (rwx) : ORIGIN = 0x01C0E000, LENGTH = 72K
    POOL    (rw)  : ORIGIN = 0x01C20000, LENGTH = 0x50000
    NOINIT  (rw)  : ORIGIN = 0x01C7C000, LENGTH = 0x3D50
}

SECTIONS
{
    .text : {
        KEEP(*(.text.crt0))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > XIP

    .ram_text : {
        . = ALIGN(4);
        _rt_start = .;
        KEEP(*(.ram_text*))
        . = ALIGN(4);
        _rt_end = .;
    } > RAMTEXT AT > XIP
    _rt_load = LOADADDR(.ram_text);

    .data : {
        . = ALIGN(4);
        _data_start = .;
        *(.data*)
        *(.sdata*)
        . = ALIGN(4);
        _data_end = .;
    } > RAM AT > XIP
    _data_load = LOADADDR(.data);

    .c1_text : {
        . = ALIGN(4);
        _c1_start = .;
        KEEP(*(.c1_text.entry))
        *(.c1_text*)
        . = ALIGN(4);
        _c1_end = .;
    } > C1TEXT AT > XIP
    _c1_load = LOADADDR(.c1_text);

    .bss (NOLOAD) : {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss*)
        *(.sbss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > RAM

    .pool (NOLOAD) : {
        . = ALIGN(4);
        _pool_start = .;
        *(.pool*)
        . = ALIGN(4);
        _pool_end = .;
    } > POOL

    .noinit (NOLOAD) : {
        *(.noinit*)
    } > NOINIT

    /* Core 1 dedicated stacks (6 KiB user + 6 KiB supervisor + 4 KiB unmapped gap below Core 0) */
    _c1_guard0 = 0x01C70000;
    _c1_ustack_lo = 0x01C70100;
    _c1_ustack_top = 0x01C71800;
    _c1_guard1 = 0x01C71800;
    _c1_sstack_lo = 0x01C71900;
    _c1_sstack_top = 0x01C73000;
    _c1_sp_fence = 0x01C73800;

    /* Core 0 stacks (exact addresses expected by crt0.S and fm1_guard_lock_top) */
    _guard0 = 0x01C74000;
    _ustack_lo = 0x01C74100;
    _ustack_top = 0x01C7A000;
    _guard1 = 0x01C7A000;
    _sstack_lo = 0x01C7A100;
    _sstack_top = 0x01C7C000;

    /DISCARD/ : {
        *(.comment)
        *(.note*)
    }
}
