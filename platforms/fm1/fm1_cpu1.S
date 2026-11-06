/* Standalone AC791N (q32DSP) Core 1 entry trampoline (`.c1_text` internal SRAM @ 0x01C0E000)
 * and dual-core exception-isolation vector stubs (`.text.c1_guard` in XIP Flash).
 *
 * Core 1 runs permanently with interrupts masked (`cli`) so all hardware IRQs (ALNK0, TIMER5,
 * SPI1, UART1, SARADC) remain strictly on Core 0. Both banked stack pointers (`usp` and `ssp`)
 * are initialized to dedicated Core 1 SRAM stacks (`0x01C70100..0x01C73000`).
 * Never execute `idle` on Core 1. */

	.section .c1_text.entry,"ax",@progbits
	.globl fm1_cpu1_entry
	.balign 4
fm1_cpu1_entry:
	cli
	sp = _c1_ustack_top
	ssp = _c1_sstack_top
	r0 = icfg
	r0 |= 0x100
	icfg = r0
	csync
	sti
	call ruido_fm1_c1_boot_post
	call ruido_fm1_cpu1_loop
1:
	nop
	goto 1b

/* Shared RAM vector table (`0x01C7FE00`) serves both Core 0 and Core 1 on AC791N.
 * Installing `fm1_dc_fatal_stubs` in place of `fm1_fatal_stubs` isolates Core 1 silicon faults
 * (`sp <= _c1_sp_fence` or corrupted `sp > _sstack_top`) so Core 1 parks harmlessly in XIP and
 * notifies Core 0 to engage fallback audio, while genuine Core 0 faults (`_c1_sp_fence < sp <=
 * _sstack_top`) translate `rets` back to `fm1_fatal_stubs` and jump to `fm1_fatal_common`. */
	.section .text.c1_guard,"ax",@progbits
	.globl fm1_dc_fatal_stubs
	.balign 4
fm1_dc_fatal_stubs:
	.rept 128
	[--sp] = {rets}
	call .Ldc_fatal_common
	.endr

.Ldc_fatal_common:
	[--sp] = {r3-r0}
	r0 = sp
	r1 = _c1_sp_fence
	if (r1 >= r0) goto .Ldc_c1_fault
	r1 = _sstack_top
	if (r0 > r1) goto .Ldc_c1_fault
	r0 = rets
	r1 = fm1_dc_fatal_stubs
	r0 = r0 - r1
	r1 = fm1_fatal_stubs
	r0 += r1
	rets = r0
	{r3-r0} = [sp++]
	goto fm1_fatal_common

.Ldc_c1_fault:
	cli
	sp = _c1_sstack_top
	ssp = _c1_sstack_top
	call ruido_fm1_on_c1_fault
2:
	nop
	goto 2b
