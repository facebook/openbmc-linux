/* arch/arm/plat-aspeed/include/mach/regs-udc11.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED UDC11 Controller
*/

#ifndef __ASM_ARCH_REGS_UDC11_H
#define __ASM_ARCH_REGS_UDC11_H __FILE__

#define	AST_UDC11_CTRL			0x00	/* Function Control and Status Register */
#define	AST_UDC11_CONF			0x04	/* Function Configuration Setting Register */
#define	AST_UDC11_REST			0x08	/* Endpoint Toggle Bit Reset Register */
#define	AST_UDC11_STS			0x0C	/* USB Status Register */
#define	AST_UDC11_IER			0x10	/* Interrupt Control Register */
#define	AST_UDC11_ISR			0x14	/* Interrupt Status Register */
#define AST_UDC11_EP0_CTRL		0x18	/* Endpoint 0 Control and Status Register */
#define AST_UDC11_EP1_CTRL		0x1C	/* Endpoint 1 Control and Status Register */
#define AST_UDC11_EP2_CTRL		0x20	/* Endpoint 2 Control and Status Register */
#define AST_UDC11_EP0_SETUP0	0x24	/* Endpoint 0 Setup/OUT Data Buffer LOW Register */
#define AST_UDC11_EP0_SETUP1	0x28	/* Endpoint 0 Setup/OUT Data Buffer HIGH Register */
#define AST_UDC11_EP0_DATA0		0x2C	/* Endpoint 0 IN DATA Buffer LOW Register */
#define AST_UDC11_EP0_DATA1		0x30	/* Endpoint 0 IN DATA Buffer HIGH Register */
#define AST_UDC11_EP1_DATA0		0x34	/* Endpoint 1 IN DATA Buffer LOW Register */
#define AST_UDC11_EP1_DATA1		0x38	/* Endpoint 1 IN DATA Buffer HIGH Register */
#define AST_UDC11_EP2_DATA0		0x3C	/* Endpoint 2 IN DATA Buffer LOW Register */
#define AST_UDC11_EP2_DATA1		0x40	/* Endpoint 2 IN DATA Buffer HIGH Register */

/* AST_UDC11_CTRL			0x00		Function Control and Status Register */
#define UDC11_CTRL_TEST_RESULT			(1 << 10)
#define UDC11_CTRL_TEST_STS				(1 << 9)
#define UDC11_CTRL_TEST_MODE(x)			((x) << 6)
#define UDC11_CTRL_WKP(x)				((x) << 4)
#define UDC11_CTRL_WKP_EN				(1 << 3)
#define UDC11_CTRL_CLK_STOP				(1 << 2)
#define UDC11_CTRL_LS_EN				(1 << 1)
#define UDC11_CTRL_CONNECT_EN			(1)

/* AST_UDC11_CONF			0x04		Function Configuration Setting Register */
#define UDC11_CONF_ADDR_MASK			(0x3f << 1)
#define UDC11_CONF_SET_ADDR(x)			(x << 1)
#define UDC11_CONF_SET_CONF				(1)

/* AST_UDC11_REST			0x08		Endpoint Toggle Bit Reset Register */
#define UDC11_REST_EP2					(1 << 1)
#define UDC11_REST_EP1					(1)


/* AST_UDC11_STS			0x0C	USB Status Register */
#define UDC11_STS_SUSPEND				(1 << 31)
#define UDC11_STS_BUS_RST				(1 << 30)
#define UDC11_STS_LINE_DP				(1 << 29)
#define UDC11_STS_LINE_DN				(1 << 28)
#define UDC11_STS_FRAM_NUM_MASK			(0x7ff << 16)
#define UDC11_STS_GET_FRAM_NUM(x)		((x >> 16) & 0x7ff)
#define UDC11_STS_LAST_ADDR				(0x7f << 4)
#define UDC11_STS_LAST_EP				(0xf)

/* AST_UDC11_IER			0x10		Interrupt Control Register */
/* AST_UDC11_ISR			0x14		Interrupt Status Register */
#define UDC11_EP0_OUT				(1 << 9)
#define UDC11_EP0_NAK				(1 << 8)
#define UDC11_EP2_IN_ACK			(1 << 7)
#define UDC11_EP1_IN_ACK			(1 << 6)
#define UDC11_EP0_IN_ACK			(1 << 5)
#define UDC11_EP0_OUT_ACK			(1 << 4)
#define UDC11_EP0_SETUP				(1 << 3)
#define UDC11_SUSPEND_RESUME		(1 << 2)
#define UDC11_SUSPEND_ENTRY			(1 << 1)
#define UDC11_BUS_REST				(1)

/* AST_UDC11_EP0_CTRL		0x18		Endpoint 0 Control and Status Register */
/* AST_UDC11_EP1_CTRL		0x1C	Endpoint 1 Control and Status Register */
/* AST_UDC11_EP2_CTRL		0x20		Endpoint 2 Control and Status Register */
#define GET_EP_OUT_RX_LEN(x)		((x & 0xf) >> 8)	//only for EP0
#define GET_EP_IN_TX_LEN(x)			((x & 0xf) >> 4)	
#define SET_EP_IN_TX_LEN(x)			((x & 0xf) << 4)
#define EP_OUT_BUFF_RX_RDY			(1 << 2)		//only for EP0
#define EP_IN_BUFF_TX_RDY			(1 << 1)
#define EP_CTRL_STALL












#endif /* __ASM_ARCH_REGS_UDC11_H */
