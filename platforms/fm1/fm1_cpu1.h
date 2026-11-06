#pragma once

/* Standalone first-party AC791N (WL82 / q32DSP) Core 1 SRAM loader, launcher, and clock probe.
 * Zero dependency on dinghy or vendor SDK headers so it remains 100% reusable across HALs. */

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

typedef struct {
    uint32_t sys_con;  /* 0x10008: Core 1 reset/enable latch on bit 3 */
    uint32_t sys_div;  /* 0x1000C: HSB / LSB / CPU clock divider register */
    uint32_t clk_con0; /* 0x10010: primary clock mux & UART clock select */
    uint32_t clk_con2; /* 0x10014: PLL / ALNK0 clock source register */
} Fm1ClockSnapshot;

/* Assembly reset trampoline placed at the start of `.c1_text` (0x01C0E000 internal SRAM). */
void fm1_cpu1_entry(void);

/* Dual-core fault-isolating exception stub table in XIP `.text` (128 x 6-byte entries).
 * Routes Core 1 exceptions cleanly to `ruido_fm1_on_c1_fault()` while forwarding genuine Core 0
 * faults to `fm1_fatal_common` with registers and vector index preserved. */
void fm1_dc_fatal_stubs(void);

/* Copy `.c1_text` (Core 1 entry + hot DSP `.text` + `.rodata`) from XIP into 0x01C0E000 SRAM.
 * MUST be invoked strictly AFTER power-on Hold OCT- USB rescue clears and BEFORE benchmarks
 * or `fm1_cpu1_start()` execute any `.c1_text` symbol. */
void fm1_cpu1_load_sram(void);

/* Unlock top-of-RAM DBG guard window 2, write `entry` to 0x01C7FFF8, relock the guard window,
 * and release Core 1 (`q32DSP(1)`) via bit 3 of 0x10008. */
void fm1_cpu1_start(void (*entry)(void));

/* Physically pause Core 1 (`q32DSP(1)`) via `C1_CON` bit 2 (`0x01EEE004`) so a faulted Core 1
 * stops fetching instructions or touching SRAM buses after single-core fallback engages. */
void fm1_cpu1_pause(void);

/* Capture power-on AC791N clock / divider registers without disturbing TIMER4, USB, UART1, or
 * ALNK0. */
void fm1_clock_snapshot(Fm1ClockSnapshot* out);

#ifdef __cplusplus
}
#endif
