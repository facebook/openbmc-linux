/* arch/arm/mach-aspeed/include/mach/regs-uart-dma.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2013/05/15 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST1070_UART_DMA_H
#define __AST1070_UART_DMA_H                     1

#define	UART_DMA0_TX_CTRL				0x00
#define	UART_DMA0_TX_DESCPT				0x04
#define	UART_DMA1_TX_CTRL				0x08
#define	UART_DMA1_TX_DESCPT				0x0C
#define	UART_DMA2_TX_CTRL				0x10
#define	UART_DMA2_TX_DESCPT				0x14
#define	UART_DMA3_TX_CTRL				0x18
#define	UART_DMA3_TX_DESCPT				0x1C
#define	UART_DMA0_RX_CTRL				0x20
#define	UART_DMA0_RX_DESCPT				0x24
#define	UART_DMA1_RX_CTRL				0x28
#define	UART_DMA1_RX_DESCPT				0x2C
#define	UART_DMA2_RX_CTRL				0x30
#define	UART_DMA2_RX_DESCPT				0x34
#define	UART_DMA3_RX_CTRL				0x38
#define	UART_DMA3_RX_DESCPT				0x3C
#define	UART_DMA_CTRL					0x40
#define	UART_DMA_IER					0x44
#define	UART_DMA_ISR					0x48

/* */
#define DMA_TRIGGER						(1 << 2)
#define DMA_ENABLE						(1 << 0)

/* UART_DMA_CTRL					0x40 */
#define SPI_CLK_MASK					(0x1f << 16)
#define SPI_CLK_SET(x)					((x) << 16)
#define DMA_RX_TIMEOUT(x)				((x) << 4)
#define DMA_BURST_LEN(x)	 			((x) << 2)
#define DMA_BURST_MASK	 				(0x3 << 2)
#define BURST_1							0
#define BURST_2							1
#define BURST_4							2
#define BURST_8							3
#define RXDESC_AUTO_POLLING 			(1 << 1)
#define TXDESC_AUTO_POLLING 			(1 << 0)

/* UART_DMA_IER / UART_DMA_ISR					0x44 0x48 */

#define UART_DMA3_RX_INT 				(1 << 7)
#define UART_DMA2_RX_INT 				(1 << 6)
#define UART_DMA1_RX_INT 				(1 << 5)
#define UART_DMA0_RX_INT 				(1 << 4)
#define UART_DMA3_TX_INT 				(1 << 3)
#define UART_DMA2_TX_INT 				(1 << 2)
#define UART_DMA1_TX_INT 				(1 << 1)
#define UART_DMA0_TX_INT 				(1 << 0)


/* UART DESC #0 Command Register */
#define DESC0_INT_EN					(1 << 9)
#define DESC0_END						(1 << 8)
#define DESC0_HW_OWN					(1 << 0)

/* UART DESC #1 Base Address of Data */
#define DESC1_LEN(x)					((x) << 16)
#define DESC1_NEXT(x)					(x)

/* UART DESC #2 Base Address of Data */

/* UART DESC #3 Descriptor Status Register */
#define DESC3_TIMEOUT_STS				(1 << 16)
#define DESC3_GET_LEN(x)				((x) & 0xffff)
#endif
