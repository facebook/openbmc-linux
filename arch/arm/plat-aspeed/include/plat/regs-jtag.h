/* arch/arm/plat-aspeed/include/mach/regs-jtag.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED JTAG Controller
*/

#define AST_JTAG_DATA			0x00
#define AST_JTAG_INST			0x04
#define AST_JTAG_CTRL			0x08
#define AST_JTAG_ISR			0x0C
#define AST_JTAG_SW				0x10
#define AST_JTAG_TCK			0x14
#define AST_JTAG_IDLE			0x18

/* AST_JTAG_CTRL - 0x08 : Engine Control */
#define JTAG_ENG_EN				(0x1 << 31)
#define JTAG_ENG_OUT_EN			(0x1 << 30)
#define JTAG_FORCE_TMS			(0x1 << 29)

#define JTAG_IR_UPDATE			(0x1 << 26)	//AST2500 only
#define JTAG_INST_LEN_MASK		(0x3f << 20)
#define JTAG_SET_INST_LEN(x)	(x << 20)
#define JTAG_SET_INST_MSB		(0x1 << 19)
#define JTAG_TERMINATE_INST		(0x1 << 18)
#define JTAG_LAST_INST			(0x1 << 17)
#define JTAG_INST_EN			(0x1 << 16)
#define JTAG_DATA_LEN_MASK		(0x3f << 4)

#define JTAG_DR_UPDATE			(0x1 << 10)	//AST2500 only
#define JTAG_DATA_LEN(x)		(x << 4)
#define JTAG_SET_DATA_MSB		(0x1 << 3)
#define JTAG_TERMINATE_DATA		(0x1 << 2)
#define JTAG_LAST_DATA			(0x1 << 1)
#define JTAG_DATA_EN			(0x1)

/* AST_JTAG_ISR	- 0x0C : INterrupt status and enable */
#define JTAG_INST_PAUSE			(0x1 << 19)
#define JTAG_INST_COMPLETE		(0x1 << 18)
#define JTAG_DATA_PAUSE			(0x1 << 17)
#define JTAG_DATA_COMPLETE		(0x1 << 16)

#define JTAG_INST_PAUSE_EN		(0x1 << 3)
#define JTAG_INST_COMPLETE_EN	(0x1 << 2)
#define JTAG_DATA_PAUSE_EN		(0x1 << 1)
#define JTAG_DATA_COMPLETE_EN	(0x1)


/* AST_JTAG_SW	- 0x10 : Software Mode and Status */
#define JTAG_SW_MODE_EN			(0x1 << 19)
#define JTAG_SW_MODE_TCK		(0x1 << 18)
#define JTAG_SW_MODE_TMS		(0x1 << 17)
#define JTAG_SW_MODE_TDIO		(0x1 << 16)
//
#define JTAG_STS_INST_PAUSE		(0x1 << 2)
#define JTAG_STS_DATA_PAUSE		(0x1 << 1)
#define JTAG_STS_ENG_IDLE		(0x1)

/*  AST_JTAG_IDLE - 0x18 : Ctroller set for go to IDLE */
#define JTAG_GO_IDLE			(0x1)
