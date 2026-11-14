#include "fm1_cpu1.h"

/* AC791N (WL82) clock / core-control & DBG memory-protection MMIO registers */
#define FM1_REG_SYS_CON (*(volatile uint32_t*)0x00010008u)
#define FM1_REG_SYS_DIV (*(volatile uint32_t*)0x0001000Cu)
#define FM1_REG_CLK_CON0 (*(volatile uint32_t*)0x00010010u)
#define FM1_REG_CLK_CON2 (*(volatile uint32_t*)0x00010014u)
#define FM1_REG_T4_CNT (*(volatile uint32_t*)0x00010804u)

#define FM1_REG_C1_CON (*(volatile uint32_t*)0x01EEE004u)
#define FM1_REG_DBG_WR_EN (*(volatile uint32_t*)0x01EEE240u)
#define FM1_REG_DBG_MSG_CLR (*(volatile uint32_t*)0x01EEE248u)
#define FM1_REG_DBG_CON (*(volatile uint32_t*)0x01EEE340u)
#define FM1_REG_DBG_WIN_EN (*(volatile uint32_t*)0x01EEE348u)
#define FM1_REG_DBG_PC_LIM0_H (*(volatile uint32_t*)0x01EEE380u)
#define FM1_REG_DBG_PC_LIM0_L (*(volatile uint32_t*)0x01EEE384u)
#define FM1_REG_DBG_PC_LIM1_H (*(volatile uint32_t*)0x01EEE388u)
#define FM1_REG_DBG_PC_LIM1_L (*(volatile uint32_t*)0x01EEE38Cu)

/* Core 0 (`q32DSP(0)`) & Core 1 (`q32DSP(1)`) private interrupt / exception banks */
#define FM1_REG_C0_ILAT_CLR (*(volatile uint32_t*)0x01EEF1A4u)
#define FM1_REG_C1_EMU_CON (*(volatile uint32_t*)0x01EEF2D0u)
#define FM1_REG_C1_EMU_MSG (*(volatile uint32_t*)0x01EEF2D4u)
#define FM1_REG_C1_EMU_SSP_H (*(volatile uint32_t*)0x01EEF2D8u)
#define FM1_REG_C1_EMU_SSP_L (*(volatile uint32_t*)0x01EEF2DCu)
#define FM1_REG_C1_EMU_USP_H (*(volatile uint32_t*)0x01EEF2E0u)
#define FM1_REG_C1_EMU_USP_L (*(volatile uint32_t*)0x01EEF2E4u)
#define FM1_C1_ICFG_WORD(i) (*(volatile uint32_t*)(0x01EEF300u + 4u * (i)))
#define FM1_REG_C1_ILAT_CLR (*(volatile uint32_t*)0x01EEF3A4u)

#define FM1_VEC_TABLE ((volatile uint32_t*)0x01C7FE00u)
#define FM1_CPU1_BOOT_VEC (*(volatile uint32_t*)0x01C7FFF8u)

#define FM1_C1_CON_HOLD_BIT (1u << 1)   /* AC79 SDK: hold = bit 1 set, then bit 3 cleared */
#define FM1_C1_CON_ENABLE_BIT (1u << 3) /* AC79 SDK EnableOtherCpu: bit 3 set, bit 1 cleared */
#define FM1_DBG_KEY_TOGGLE 0xE7u
#define FM1_DBG_TOP_WIN_BIT (1u << 2)
#define FM1_DBG_BUS_INVALID_BITS ((0x3Fu << 16) | (0x3u << 4))
#define FM1_SYS_CPU1_START_BIT (1u << 3)
#define FM1_C1_BOOT_TIMEOUT_T4_TICKS (20000u * 24u) /* 20 ms, as X0X / Melodee */

/* Hold Core 1 exactly as the AC79 SDK (`cpu/wl82/debug.c`) and X0X / Melodee do on real units */
static inline void fm1_cpu1_hold_regs(void) {
    FM1_REG_C1_CON |= FM1_C1_CON_HOLD_BIT;
    FM1_REG_C1_CON &= ~FM1_C1_CON_ENABLE_BIT;
    __asm__ volatile("csync" ::: "memory");
}

extern uint32_t _c1_start[];
extern uint32_t _c1_end[];
extern uint32_t _c1_load[];
extern char _etext[];
extern char _c1_ustack_lo[];
extern char _c1_sstack_top[];
extern void fm1_fatal_stubs(void);

static void fm1_flush_code_range(uintptr_t start, uintptr_t end) {
    uintptr_t a = start & ~(uintptr_t)31u;
    __asm__ volatile("csync" ::: "memory");
    for (; a < end; a += 32u) {
        __asm__ volatile("flushinv [%0]\n\tiflush [%0]" ::"r"(a) : "memory");
    }
    __asm__ volatile("ssync\n\tcsync" ::: "memory");
}

static void fm1_flush_data_range(uintptr_t start, uintptr_t end) {
    uintptr_t a = start & ~(uintptr_t)31u;
    __asm__ volatile("csync" ::: "memory");
    for (; a < end; a += 32u) {
        __asm__ volatile("flushinv [%0]" ::"r"(a) : "memory");
    }
    __asm__ volatile("ssync\n\tcsync" ::: "memory");
}

void fm1_cpu1_load_sram(void) {
    const uint32_t* src = _c1_load;
    uint32_t* dst = _c1_start;
    uint32_t bus_mask = FM1_DBG_BUS_INVALID_BITS;
    uint32_t was_unlocked = 0u;
    while (dst < _c1_end) {
        *dst++ = *src++;
    }
    fm1_flush_code_range((uintptr_t)_c1_start, (uintptr_t)_c1_end);

    /* Extend `PC_LIMIT1` (`0x01C00000.._c1_end-1`) immediately so Core 0 construction helpers
     * (`callC1Far`) execute legally inside `.c1_text` SRAM prior to `fm1_cpu1_start()`. */
    was_unlocked = FM1_REG_DBG_WR_EN & 1u;
    if (was_unlocked == 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
    if ((uint32_t)(uintptr_t)_c1_end > 0x01C0E000u) {
        FM1_REG_DBG_PC_LIM1_L = 0x01C00000u;
        FM1_REG_DBG_PC_LIM1_H = (uint32_t)(uintptr_t)_c1_end - 1u;
    }
    __asm__ volatile("" : "+r"(bus_mask));
    FM1_REG_DBG_CON &= ~bus_mask;
    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    __asm__ volatile("csync" ::: "memory");
    if (was_unlocked == 0u && (FM1_REG_DBG_WR_EN & 1u) != 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
}

int fm1_cpu1_start(void (*entry)(void), volatile uint32_t* booted_flag) {
    uint32_t was_unlocked = 0u;
    uint32_t saved_win = 0u;
    uint32_t saved_dbg_con = 0u;
    uint32_t bus_mask = FM1_DBG_BUS_INVALID_BITS;
    uint32_t old_base = (uint32_t)(uintptr_t)fm1_fatal_stubs;
    uint32_t new_base = (uint32_t)(uintptr_t)fm1_dc_fatal_stubs;
    uint32_t t0 = 0u;
    uint32_t saved_sys_con = 0u;
    uint32_t i = 0u;
    int booted_ok = 0;

    if (entry == (void (*)(void))0 || booted_flag == (volatile uint32_t*)0) {
        return 0;
    }

    __asm__ volatile("cli" ::: "memory");

    /* 1. Hold Core 1, then mask all its peripheral IRQs + stack-limit comparators before release
     *    (`fm1_cpu1_entry` arms `C1_EMU_CON` and Vector 1 AFTER loading `usp` and `ssp`). */
    fm1_cpu1_hold_regs();
    for (i = 0u; i < 32u; ++i) {
        FM1_C1_ICFG_WORD(i) = 0u;
    }
    FM1_REG_C1_ILAT_CLR = 0xFFu;
    FM1_REG_C1_EMU_CON &= ~((1u << 2) | (1u << 3));
    FM1_REG_C1_EMU_SSP_L = (uint32_t)(uintptr_t)_c1_ustack_lo;
    FM1_REG_C1_EMU_SSP_H = (uint32_t)(uintptr_t)_c1_sstack_top;
    FM1_REG_C1_EMU_USP_L = (uint32_t)(uintptr_t)_c1_ustack_lo;
    FM1_REG_C1_EMU_USP_H = (uint32_t)(uintptr_t)_c1_sstack_top;
    FM1_REG_C1_EMU_MSG = 0xFFFFFFFFu;

    /* 2. Install `fm1_dc_fatal_stubs`, write `0x01C7FFF8`, and temporarily open `PC_LIMIT0/1` +
     *    `DBG` bus-invalid guards while Core 1 traverses Mask ROM (`0x00FFxxxx`) -> `uboot.boot`
     *    (`0x01E00000`) -> `fm1_cpu1_boot_xip` -> `fm1_cpu1_entry` (`0x01C0E000`). */
    was_unlocked = FM1_REG_DBG_WR_EN & 1u;
    if (was_unlocked == 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");

    saved_dbg_con = FM1_REG_DBG_CON;
    saved_win = FM1_REG_DBG_WIN_EN;
    __asm__ volatile("" : "+r"(bus_mask));
    FM1_REG_DBG_CON = saved_dbg_con & ~bus_mask;
    FM1_REG_DBG_WIN_EN = saved_win & ~FM1_DBG_TOP_WIN_BIT;
    FM1_REG_DBG_PC_LIM0_L = 0x00000000u;
    FM1_REG_DBG_PC_LIM0_H = 0xFFFFFFFFu;
    FM1_REG_DBG_PC_LIM1_L = 0x00000000u;
    FM1_REG_DBG_PC_LIM1_H = 0xFFFFFFFFu;
    __asm__ volatile("csync" ::: "memory");

    for (i = 0u; i < 128u; ++i) {
        const uint32_t expected = old_base + 6u * i;
        if (FM1_VEC_TABLE[i] == expected) {
            FM1_VEC_TABLE[i] = new_base + 6u * i;
        }
    }
    FM1_CPU1_BOOT_VEC = (uint32_t)(uintptr_t)entry;
    *booted_flag = 0u;
    fm1_flush_data_range((uintptr_t)booted_flag & ~(uintptr_t)63u,
                         ((uintptr_t)booted_flag & ~(uintptr_t)63u) + 64u);
    fm1_flush_data_range(0x01C7FE00u, 0x01C80000u);

    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    __asm__ volatile("csync" ::: "memory");

    /* 3. Release Core 1 (AC79 SDK `EnableOtherCpu`: clock bit 3 held while it starts, C1_CON
     *    bit 3 set + bit 1 cleared) and wait (<= 20 ms) until `fm1_cpu1_entry` reports in. */
    saved_sys_con = FM1_REG_SYS_CON;
    t0 = FM1_REG_T4_CNT;
    FM1_REG_SYS_CON = saved_sys_con | FM1_SYS_CPU1_START_BIT;
    __asm__ volatile("csync" ::: "memory");
    FM1_REG_C1_CON |= FM1_C1_CON_ENABLE_BIT;
    FM1_REG_C1_CON &= ~FM1_C1_CON_HOLD_BIT;
    __asm__ volatile("csync" ::: "memory");

    for (;;) {
        __asm__ volatile("flushinv [%0]\n\tcsync" ::"r"(booted_flag) : "memory");
        if (*booted_flag != 0u) {
            booted_ok = 1;
            break;
        }
        if ((uint32_t)(FM1_REG_T4_CNT - t0) >= FM1_C1_BOOT_TIMEOUT_T4_TICKS) {
            fm1_cpu1_hold_regs();
            break;
        }
    }
    FM1_REG_SYS_CON = saved_sys_con;
    __asm__ volatile("csync" ::: "memory");

    /* 4. Re-arm strict `PC_LIMIT0` (XIP), `PC_LIMIT1` (`.c1_text` SRAM), and `DBG` guards */
    FM1_REG_DBG_PC_LIM0_L = 0x02000120u;
    FM1_REG_DBG_PC_LIM0_H = (uint32_t)(uintptr_t)_etext;
    if ((uint32_t)(uintptr_t)_c1_end > 0x01C0E000u) {
        FM1_REG_DBG_PC_LIM1_L = 0x01C00000u;
        FM1_REG_DBG_PC_LIM1_H = (uint32_t)(uintptr_t)_c1_end - 1u;
    } else {
        FM1_REG_DBG_PC_LIM1_L = 0x02000120u;
        FM1_REG_DBG_PC_LIM1_H = (uint32_t)(uintptr_t)_etext;
    }
    FM1_REG_DBG_WIN_EN = saved_win;
    FM1_REG_DBG_CON = saved_dbg_con & ~bus_mask;
    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    FM1_REG_C0_ILAT_CLR = (1u << 1);
    __asm__ volatile("csync" ::: "memory");

    if (was_unlocked == 0u && (FM1_REG_DBG_WR_EN & 1u) != 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync\n\tsti" ::: "memory");
    return booted_ok;
}

void fm1_cpu1_pause(void) {
    uint32_t was_unlocked = FM1_REG_DBG_WR_EN & 1u;
    fm1_cpu1_hold_regs();
    if (was_unlocked == 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
    FM1_REG_DBG_PC_LIM0_L = 0x00000000u;
    FM1_REG_DBG_PC_LIM0_H = 0xFFFFFFFFu;
    FM1_REG_DBG_PC_LIM1_L = 0x00000000u;
    FM1_REG_DBG_PC_LIM1_H = 0xFFFFFFFFu;
    FM1_REG_DBG_CON = 0u;
    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    __asm__ volatile("csync" ::: "memory");
    if (was_unlocked == 0u && (FM1_REG_DBG_WR_EN & 1u) != 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
}

void fm1_clock_snapshot(Fm1ClockSnapshot* out) {
    if (out == (Fm1ClockSnapshot*)0) {
        return;
    }
    out->sys_con = FM1_REG_SYS_CON;
    out->sys_div = FM1_REG_SYS_DIV;
    out->clk_con0 = FM1_REG_CLK_CON0;
    out->clk_con2 = FM1_REG_CLK_CON2;
}
