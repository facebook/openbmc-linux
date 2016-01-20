/* debug-macro.S
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
	ldreq	\rx, =CONFIG_AST_CONSOLE_UART_BASE
	ldrne	\rx, =IO_ADDRESS(CONFIG_AST_CONSOLE_UART_BASE)
	orr 	\rx, \rx, #0x00012000
	.endm
	
#define UART_SHIFT      2
#include <asm/hardware/debug-8250.S>
