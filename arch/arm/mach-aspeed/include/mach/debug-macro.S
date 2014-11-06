/* linux/include/asm-arm/arch-gucmvp2000/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include <mach/platform.h>
#include <mach/hardware.h>
	
	.macro	addruart, rx, tmp
	mrc 	p15, 0, \rx, c1, c0
	tst 	\rx, #1 								@ MMU enabled?
	ldreq   \rx, =AST_UART3_BASE
	ldrne	\rx, =IO_ADDRESS(AST_UART3_BASE)
	.endm
	
#define UART_SHIFT      2
#include <asm/hardware/debug-8250.S>
