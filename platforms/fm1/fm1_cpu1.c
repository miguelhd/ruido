#include "fm1_cpu1.h"

/* AC791N (WL82) clock / core-control & DBG memory-protection MMIO registers */
#define FM1_REG_SYS_CON (*(volatile uint32_t*)0x00010008u)
#define FM1_REG_SYS_DIV (*(volatile uint32_t*)0x0001000Cu)
#define FM1_REG_CLK_CON0 (*(volatile uint32_t*)0x00010010u)
#define FM1_REG_CLK_CON2 (*(volatile uint32_t*)0x00010014u)

#define FM1_REG_C1_CON (*(volatile uint32_t*)0x01EEE004u)
#define FM1_REG_DBG_WR_EN (*(volatile uint32_t*)0x01EEE240u)
#define FM1_REG_DBG_MSG_CLR (*(volatile uint32_t*)0x01EEE248u)
#define FM1_REG_DBG_CON (*(volatile uint32_t*)0x01EEE340u)
#define FM1_REG_DBG_WIN_EN (*(volatile uint32_t*)0x01EEE348u)
#define FM1_REG_DBG_PC_LIM1_H (*(volatile uint32_t*)0x01EEE388u)
#define FM1_REG_DBG_PC_LIM1_L (*(volatile uint32_t*)0x01EEE38Cu)

/* Core 1 (`q32DSP(1)`) private exception / interrupt bank (`0x01EEF200..0x01EEF3FF`) */
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

#define FM1_C1_CON_PAUSE_BIT (1u << 2)
#define FM1_DBG_KEY_TOGGLE 0xE7u
#define FM1_DBG_TOP_WIN_BIT (1u << 2)
#define FM1_DBG_BUS_INVALID_BITS ((0x3Fu << 16) | (0x3u << 4))
#define FM1_SYS_CPU1_START_BIT (1u << 3)

extern uint32_t _c1_start[];
extern uint32_t _c1_end[];
extern uint32_t _c1_load[];
extern char _c1_ustack_lo[];
extern char _c1_sstack_top[];
extern void fm1_fatal_stubs(void);

void fm1_cpu1_load_sram(void) {
    const uint32_t* src = _c1_load;
    uint32_t* dst = _c1_start;
    uint32_t was_unlocked = 0u;
    while (dst < _c1_end) {
        *dst++ = *src++;
    }
    __asm__ volatile("csync" ::: "memory");

    /* Extend `PC_LIMIT1` (`0x01C00000.._c1_end-1`) immediately so both Core 0 construction
     * helpers (`callC1Far`) and Core 1 SRAM execution stay inside the hardware PC guard window,
     * while keeping `FM1_DBG_BUS_INVALID_BITS` armed so any bit-21 SRAM glitch (`0x01E00000+`)
     * raises CPU Exception Vector 1 immediately instead of fetching unmapped space. */
    was_unlocked = FM1_REG_DBG_WR_EN & 1u;
    if (was_unlocked == 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
    if ((uint32_t)(uintptr_t)_c1_end > 0x01C0E000u) {
        FM1_REG_DBG_PC_LIM1_L = 0x01C00000u;
        FM1_REG_DBG_PC_LIM1_H = (uint32_t)(uintptr_t)_c1_end - 1u;
    }
    FM1_REG_DBG_CON |= FM1_DBG_BUS_INVALID_BITS;
    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    __asm__ volatile("csync" ::: "memory");
    if (was_unlocked == 0u && (FM1_REG_DBG_WR_EN & 1u) != 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");
}

void fm1_cpu1_start(void (*entry)(void)) {
    uint32_t was_unlocked = 0u;
    uint32_t saved_win = 0u;
    uint32_t old_base = (uint32_t)(uintptr_t)fm1_fatal_stubs;
    uint32_t new_base = (uint32_t)(uintptr_t)fm1_dc_fatal_stubs;
    uint32_t i = 0u;

    __asm__ volatile("cli" ::: "memory");

    /* 1. Mask all peripheral IRQs (`2..127`) in Core 1's private `q32DSP(1)` interrupt bank so
     *    ALNK0, TIMER5, SPI1, UART1, and SARADC remain exclusively on Core 0, while enabling
     *    CPU Exception Vector 1 + stack-limit guards on Core 1 so silicon faults trap cleanly. */
    for (i = 0u; i < 32u; ++i) {
        FM1_C1_ICFG_WORD(i) = 0u;
    }
    FM1_REG_C1_ILAT_CLR = 0xFFu;
    FM1_REG_C1_EMU_MSG = 0xFFFFFFFFu;
    FM1_REG_C1_EMU_CON &= ~((1u << 2) | (1u << 3));
    FM1_REG_C1_EMU_SSP_L = (uint32_t)(uintptr_t)_c1_ustack_lo;
    FM1_REG_C1_EMU_SSP_H = (uint32_t)(uintptr_t)_c1_sstack_top;
    FM1_REG_C1_EMU_USP_L = (uint32_t)(uintptr_t)_c1_ustack_lo;
    FM1_REG_C1_EMU_USP_H = (uint32_t)(uintptr_t)_c1_sstack_top;
    FM1_REG_C1_EMU_CON |= (1u << 3);
    FM1_C1_ICFG_WORD(0) = 0xF0u; /* Vector 1 (CPU Exception): enabled at priority 7 */

    /* 2. Install `fm1_dc_fatal_stubs` in shared `0x01C7FE00` vector table and write `0x01C7FFF8` */
    was_unlocked = FM1_REG_DBG_WR_EN & 1u;
    if (was_unlocked == 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");

    if ((uint32_t)(uintptr_t)_c1_end > 0x01C0E000u) {
        FM1_REG_DBG_PC_LIM1_L = 0x01C00000u;
        FM1_REG_DBG_PC_LIM1_H = (uint32_t)(uintptr_t)_c1_end - 1u;
    }
    FM1_REG_DBG_CON |= FM1_DBG_BUS_INVALID_BITS;

    saved_win = FM1_REG_DBG_WIN_EN;
    FM1_REG_DBG_WIN_EN = saved_win & ~FM1_DBG_TOP_WIN_BIT;
    __asm__ volatile("csync" ::: "memory");

    for (i = 0u; i < 128u; ++i) {
        const uint32_t expected = old_base + 6u * i;
        if (FM1_VEC_TABLE[i] == expected) {
            FM1_VEC_TABLE[i] = new_base + 6u * i;
        }
    }
    FM1_CPU1_BOOT_VEC = (uint32_t)(uintptr_t)entry;
    __asm__ volatile("csync" ::: "memory");

    FM1_REG_DBG_MSG_CLR = 0xFFFFFFFFu;
    FM1_REG_DBG_WIN_EN = saved_win;
    __asm__ volatile("csync" ::: "memory");

    if (was_unlocked == 0u && (FM1_REG_DBG_WR_EN & 1u) != 0u) {
        FM1_REG_DBG_WR_EN = FM1_DBG_KEY_TOGGLE;
    }
    __asm__ volatile("csync" ::: "memory");

    /* 3. Release Core 1 (`q32DSP(1)`) to fetch `entry` in `.c1_text` internal SRAM */
    FM1_REG_SYS_CON |= FM1_SYS_CPU1_START_BIT;
    __asm__ volatile("csync\n\tsti" ::: "memory");
}

void fm1_cpu1_pause(void) {
    FM1_REG_C1_CON = FM1_C1_CON_PAUSE_BIT;
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
