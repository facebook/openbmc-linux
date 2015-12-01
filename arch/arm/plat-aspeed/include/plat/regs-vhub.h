/* arch/arm/plat-aspeed/include/mach/regs-vhub.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED VHUB Controller
*/

#ifndef __AST_REGS_VHUB_H
#define __AST_REGS_VHUB_H __FILE__

#define	AST_VHUB_CTRL			0x00	/* Root Function Control & Status Register */
#define	AST_VHUB_CONF			0x04	/* Root Configuration Setting Register */
#define	AST_VHUB_IER			0x08	/* Interrupt Ctrl Register */
#define	AST_VHUB_ISR			0x0C	/* Interrupt Status Register */
#define	AST_VHUB_EP_ACK_IER		0x10	/* Programmable Endpoint Pool ACK Interrupt Enable Register */
#define	AST_VHUB_EP_NACK_IER	0x14	/* Programmable Endpoint Pool NACK Interrupt Enable Register  */
#define AST_VHUB_EP_ACK_STS		0x18	/* Programmable Endpoint Pool ACK Interrupt Status Register  */
#define AST_VHUB_EP_NACK_STS	0x1C	/* Programmable Endpoint Pool NACK Interrupt Status Register  */
#define AST_VHUB_SW_REST		0x20	/* Device Controller Soft Reset Enable Register */
#define AST_VHUB_STS			0x24	/* USB Status Register */
#define AST_VHUB_EP_TOGGLE		0x28	/* Programmable Endpoint Pool Data Toggle Value Set */
#define AST_VHUB_ISO_FAIL_ACC	0x2C	/* Isochronous Transaction Fail Accumulator */
#define AST_VHUB_EP0_CTRL		0x30	/* Endpoint 0 Contrl/Status Register */
#define AST_VHUB_EP0_DATA		0x34	/* Base Address of Endpoint 0 In/OUT Data Buffer Register */
#define AST_VHUB_EP1_CTRL		0x38	/* Endpoint 1 Contrl/Status Register */
#define AST_VHUB_EP1_STS_CHG	0x3C	/* Endpoint 1 Status Change Bitmap Data */

#define AST_VHUB_SETUP0			0x80	/* Root Device Setup Data Buffer0 */
#define AST_VHUB_SETUP1			0x84	/* Root Device Setup Data Buffer1 */


/* AST_VHUB_CTRL			0x00	  - 	Root Function Control & Status Register */
#define VHUB_CTRL_PHY_CLK				(1 << 31)
/*....*/
#define VHUB_CTRL_PHY_LOOP_TEST			(1 << 25)
#define VHUB_CTRL_DN_PWN				(1 << 24)
#define VHUB_CTRL_DP_PWN				(1 << 23)
/*....*/
#define VHUB_CTRL_LONG_DESC				(1 << 18)
#define VHUB_CTRL_ISO_RSP_CTRL			(1 << 17)
#define VHUB_CTRL_SPLIT_IN				(1 << 16)
#define VHUB_CTRL_LOOP_T_RESULT			(1 << 15)
#define VHUB_CTRL_LOOP_T_STS			(1 << 14)
#define VHUB_CTRL_PHY_BIST_RESULT		(1 << 13)
#define VHUB_CTRL_PHY_BIST_CTRL			(1 << 12)
#define VHUB_CTRL_PHY_REST_DIS			(1 << 11)
#define VHUB_CTRL_SET_TEST_MODE(x)		(x << 8)

#define VHUB_CTRL_UPSTREAM_CONNECT		(1)












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












#endif 
