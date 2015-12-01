/* arch/arm/mach-aspeed/include/mach/regs-uart-sdma.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2013/12/15 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST_UART_SDMA_H
#define __AST_UART_SDMA_H                     1

#define	UART_TX_SDMA_EN					0x00
#define	UART_RX_SDMA_EN					0x04
#define	UART_SDMA_CONF					0x08
#define	UART_SDMA_TIMER					0x0C
//
#define	UART_TX_SDMA_REST				0x20
#define	UART_RX_SDMA_REST				0x24
//
#define	UART_TX_SDMA_IER				0x30
#define	UART_TX_SDMA_ISR				0x34
#define	UART_RX_SDMA_IER				0x38
#define	UART_RX_SDMA_ISR				0x3C
#define	UART_TX_R_POINT(x)				(0x40 + (x*0x20))
#define	UART_TX_W_POINT(x)				(0x44 + (x*0x20))
#define	UART_TX_SDMA_ADDR(x)			(0x48 + (x*0x20))
#define	UART_RX_R_POINT(x)				(0x50 + (x*0x20))
#define	UART_RX_W_POINT(x)				(0x54 + (x*0x20))
#define	UART_RX_SDMA_ADDR(x)			(0x58 + (x*0x20))

/* UART_TX_SDMA_EN	-0x00 : UART TX DMA Enable */
/* UART_RX_SDMA_EN	-0x04 : UART RX DMA Enable */
#define SDMA_CH_EN(x)					(0x1 << (x))

/* UART_SDMA_CONF - 0x08 : Misc, Buffer size  */
#define SDMA_TX_BUFF_SIZE_MASK			(0x3)
#define SDMA_SET_TX_BUFF_SIZE(x)		(x)
#define SDMA_BUFF_SIZE_1KB				(0x0)
#define SDMA_BUFF_SIZE_4KB				(0x1)
#define SDMA_BUFF_SIZE_16KB				(0x2)
#define SDMA_BUFF_SIZE_64KB				(0x3)
#define SDMA_RX_BUFF_SIZE_MASK			(0x3 << 2)
#define SDMA_SET_RX_BUFF_SIZE(x)		(x << 2)
#define SDMA_TIMEOUT_DIS				(0x1 << 4)

/* UART_SDMA_TIMER	-0x0C :  UART DMA time out timer */


/* UART_TX_SDMA_IER				0x30	*/
/* UART_TX_SDMA_ISR				0x34	*/

#define UART_SDMA11_INT	 				(1 << 11)
#define UART_SDMA10_INT	 				(1 << 10)
#define UART_SDMA9_INT	 				(1 << 9)
#define UART_SDMA8_INT	 				(1 << 8)
#define UART_SDMA7_INT	 				(1 << 7)
#define UART_SDMA6_INT	 				(1 << 6)
#define UART_SDMA5_INT	 				(1 << 5)
#define UART_SDMA4_INT	 				(1 << 4)
#define UART_SDMA3_INT	 				(1 << 3)
#define UART_SDMA2_INT	 				(1 << 2)
#define UART_SDMA1_INT	 				(1 << 1)
#define UART_SDMA0_INT	 				(1 << 0)



#endif
