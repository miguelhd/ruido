/* Standalone AC791N (q32DSP) Core 1 entry trampoline (`.c1_text` internal SRAM @ 0x01C0E000)
 * and dual-core exception-isolation vector stubs (`.text.c1_guard` in XIP Flash).
 *
 * Core 1 runs permanently with interrupts masked (`cli`) so all hardware IRQs (ALNK0, TIMER5,
 * SPI1, UART1, SARADC) remain strictly on Core 0. Both banked stack pointers (`usp` and `ssp`)
 * are initialized to dedicated Core 1 SRAM stacks (`0x01C70100..0x01C73000`).
 * Never execute `idle` on Core 1. */

	.section .c1_text.entry,"ax",@progbits
	.globl fm1_cpu1_entry
	.globl ruido_fm1_c1_boot_post
	.globl ruido_fm1_c1_park_sram
	.globl fm1_cpu1_run
	.balign 4
fm1_cpu1_entry:
	cli
	sp = _c1_ustack_top
	ssp = _c1_sstack_top
	csync
	r0 = 0x01EEF2D0
	r2 = [r0+0]
	r1 = 0xFFE0FFF3 /* EMU_CON: bits 2..3 and FPU traps 16..20 off (X0X / Melodee) */
	r2 &= r1
	[r0+0] = r2
	r1 = _c1_ustack_lo
	[r0+12] = r1
	[r0+20] = r1
	r1 = _c1_sstack_top
	[r0+8] = r1
	[r0+16] = r1
	r1 = -1
	[r0+4] = r1
	r2 |= 8
	[r0+0] = r2
	r1 = 0xF0
	[r0+48] = r1
	r0 = icfg
	r0 |= 0x100
	icfg = r0
	csync
	sti
	r0 = g_ruido_ic_ctrl
	r1 = 1
	[r0+12] = r1
	flushinv [r0]
	ssync
	csync

/* Stackless register-only (`r0..r3`) 512-read SRAM canary POST (`0x00000000` & `0xFFFFFFFF`)
 * executed before Core 1 pushes a single word to RAM, catching bit-21 (`0x00200000`) misreads. */
ruido_fm1_c1_boot_post:
	r0 = g_ruido_ic_ctrl
	r1 = 256
	r2 = 0
1:
	r3 = [r0+0]
	r2 |= r3
	r3 = [r0+4]
	r3 = ~r3
	r2 |= r3
	r1 += -1
	if (r1 != 0) goto 1b
	if (r2 == 0) goto fm1_cpu1_run
	r1 = 1
	[r0+8] = r1
	flushinv [r0]
	ssync
	csync
ruido_fm1_c1_park_sram:
	nop
	goto ruido_fm1_c1_park_sram

fm1_cpu1_run:
	call ruido_fm1_cpu1_loop
3:
	nop
	goto 3b

/* Shared RAM vector table (`0x01C7FE00`) serves both Core 0 and Core 1 on AC791N.
 * Installing `fm1_dc_fatal_stubs` in place of `fm1_fatal_stubs` isolates Core 1 silicon faults
 * (`cnum != 0`, or shared `DBG_MSG` bit-12 `PC_LIMIT` trips caused by Core 1 while Core 0's `reti`
 * is inside XIP `.text` `[0x02000120, _etext)` or SRAM `.ram_text`/`.c1_text` `[0x01C00000, _c1_end)`)
 * so Core 1 parks harmlessly and notifies Core 0 via `c1SelfFault`, while genuine Core 0 faults
 * translate `rets` back to `fm1_fatal_stubs` and jump to `fm1_fatal_common`.
 * Keep `fm1_dc_fatal_common` in a separate section so MC never relaxes `call` to 2 bytes. */
	.section .text.c1_guard,"ax",@progbits
	.globl fm1_dc_fatal_stubs
	.balign 4
fm1_dc_fatal_stubs:
	.rept 128
	[--sp] = {rets}
	call fm1_dc_fatal_common
	.endr

	.section .text.c1_guard_common,"ax",@progbits
	.globl fm1_cpu1_boot_xip
	.globl fm1_dc_fatal_common
	.balign 4
fm1_cpu1_boot_xip:
	/* Exactly the AC79 SDK `startup.S` / X0X / Melodee Core 1 entry: the chip's ROM enters here
	 * inside a start-up interrupt, so mask interrupts, clear icfg, load Core 1's own stacks
	 * BEFORE anything can fault, then `rti` out of the start-up interrupt into `.c1_text`. */
	cli
	icfg = 0
	usp = _c1_ustack_top
	sp = _c1_sstack_top
	ssp = _c1_sstack_top
	reti = fm1_cpu1_entry
	rti

	.balign 4
fm1_dc_fatal_common:
	[--sp] = {r3-r0}
	r3 = psr
	r0 = cnum
	if (r0 != 0) goto .Ldc_c1_fault

	/* Core 0 (`cnum == 0`): check whether shared SoC `DBG` Vector 1 fired solely because Core 1
	 * tripped `JL_DBG` (`DBG_MSG != 0`, `C0_EMU_MSG == 0`, `DBG_CON != 0`, and Core 0 `reti` in
	 * `[0x02000120, _etext)` or `[0x01C00000, _c1_end)`). */
	r0 = 0x01EEF0D4
	r0 = [r0+0]
	if (r0 != 0) goto .Ldc_c0_fault
	r0 = 0x01EEE244
	r2 = [r0+0]
	if (r2 == 0) goto .Ldc_c0_fault
	r0 = 0x01EEE340
	r0 = [r0+0]
	if (r0 == 0) goto .Ldc_c0_fault
	r0 = reti
	r1 = 0x02000120
	if (r0 < r1) goto .Ldc_chk_c0_sram
	r1 = _etext
	if (r0 < r1) goto .Ldc_c1_pclim_trap
	goto .Ldc_c0_fault
.Ldc_chk_c0_sram:
	r1 = 0x01C00000
	if (r0 < r1) goto .Ldc_c0_fault
	r1 = _c1_end
	if (r0 >= r1) goto .Ldc_c0_fault
.Ldc_c1_pclim_trap:
	/* Diagnostics: record shared DBG_MSG (r2) and Core 0's interrupted PC (r0 = reti) */
	r1 = g_ruido_ic_ctrl
	[r1+16] = r2
	[r1+20] = r0
	r1 = g_ruido_boot_diag
	[r1+8] = r2
	[r1+12] = r0
	flushinv [r1]
	/* Hold Core 1 (AC79 SDK: C1_CON bit 1 set, then bit 3 cleared) */
	r0 = 0x01EEE004
	r1 = [r0+0]
	r1 |= 2
	[r0+0] = r1
	r2 = -9
	r1 &= r2
	[r0+0] = r1
	r0 = g_ruido_ic_ctrl
	r1 = 1
	[r0+8] = r1
	flushinv [r0]
	r0 = 0x01EEE240
	r2 = [r0+0]
	r1 = r2 & 1
	if (r1 == 0) goto .Ldc_unlock_dbg
	goto .Ldc_clr_dbg
.Ldc_unlock_dbg:
	r1 = 0xE7
	[r0+0] = r1
	csync
.Ldc_clr_dbg:
	r0 = 0x01EEE380
	r1 = -1
	[r0+0] = r1
	[r0+8] = r1
	r1 = 0
	[r0+4] = r1
	[r0+12] = r1
	r0 = 0x01EEE340
	[r0+0] = r1
	r0 = 0x01EEE240
	r1 = -1
	[r0+8] = r1
	r1 = r2 & 1
	if (r1 != 0) goto .Ldc_done_dbg
	r1 = 0xE7
	[r0+0] = r1
.Ldc_done_dbg:
	r0 = 0x01EEF1A4
	r1 = 2
	[r0+0] = r1
	ssync
	csync
	psr = r3
	{r3-r0} = [sp++]
	{rets} = [sp++]
	csync
	rti

.Ldc_c0_fault:
	r0 = 0x01EEE004
	r1 = [r0+0]
	r1 |= 2
	[r0+0] = r1
	r2 = -9
	r1 &= r2
	[r0+0] = r1
	r2 = 0x01EEF0D0
	r0 = 0
	[r2+0] = r0
	csync
	r0 = sp
	r1 = _ustack_lo
	r1 += 256
	if (r0 < r1) goto .Ldc_c0_sp_fix
	r1 = _sstack_top
	if (r0 <= r1) goto .Ldc_c0_sp_ok
.Ldc_c0_sp_fix:
	r1 = _sstack_top
	r1 += -20
	r2 = [r0+0]
	[r1+0] = r2
	r2 = [r0+4]
	[r1+4] = r2
	r2 = [r0+8]
	[r1+8] = r2
	r2 = [r0+12]
	[r1+12] = r2
	r2 = [r0+16]
	[r1+16] = r2
	ssp = _sstack_top
	sp = _sstack_top
	sp += -20
.Ldc_c0_sp_ok:
	r0 = rets
	r1 = fm1_dc_fatal_stubs
	r0 = r0 - r1
	r1 = fm1_fatal_stubs
	r0 += r1
	rets = r0
	psr = r3
	{r3-r0} = [sp++]
	goto fm1_fatal_common

.Ldc_c1_fault:
	cli
	r0 = 0x01EEF2D0
	r2 = [r0+0]
	r1 = 0xFFE0FFF3 /* EMU_CON: bits 2..3 and FPU traps 16..20 off (X0X / Melodee) */
	r2 &= r1
	[r0+0] = r2
	r1 = -1
	[r0+4] = r1
	r1 = 0
	[r0+48] = r1
	r0 = 0x01EEF3A4
	r1 = 255
	[r0+0] = r1
	r0 = g_ruido_ic_ctrl
	/* Diagnostics: 0xC1000000 | stub offset (vector * 6) and Core 1 RETI */
	r1 = rets
	r2 = fm1_dc_fatal_stubs
	r1 = r1 - r2
	r2 = 0xC1000000
	r1 |= r2
	[r0+16] = r1
	r2 = g_ruido_boot_diag
	[r2+8] = r1
	r1 = reti
	[r0+20] = r1
	[r2+12] = r1
	flushinv [r2]
	r1 = 1
	[r0+8] = r1
	flushinv [r0]
	ssync
	csync
	r0 = ruido_fm1_c1_park_sram
	goto r0
