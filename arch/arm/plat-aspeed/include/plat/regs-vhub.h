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















#endif 
