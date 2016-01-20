/*
 *  uncompress.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <mach/platform.h>
#include <mach/aspeed_serial.h>

#define UART_PUT_CHAR   (*(volatile unsigned char *)(CONFIG_AST_CONSOLE_UART_BASE + UART_THR))
#define UART_GET_LSR   (*(volatile unsigned char *)(CONFIG_AST_CONSOLE_UART_BASE + UART_LSR))

static void putc(int c)
{

	/* wait for space in the UART's transmitter */
	while (!(UART_GET_LSR & UART_LSR_THRE))
		barrier();
	
	/* send the character out. */
	UART_PUT_CHAR = c;
}

static inline void flush(void)
{
	while (UART_GET_LSR & (1 << 3))
			 barrier();
}

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif /* __ASM_ARCH_UNCOMPRESS_H */
