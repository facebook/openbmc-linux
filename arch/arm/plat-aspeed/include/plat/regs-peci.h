/* arch/arm/plat-aspeed/include/mach/regs-peci.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED PECI Controller
*/

#ifndef __ASM_ARCH_REGS_PECI_H
#define __ASM_ARCH_REGS_PECI_H __FILE__

/*AST PECI Register Definition */
#define AST_PECI_CTRL		0x00
#define AST_PECI_TIMING 	0x04
#define AST_PECI_CMD		0x08
#define AST_PECI_CMD_CTRL	0x0C
#define AST_PECI_EXP_FCS	0x10
#define AST_PECI_CAP_FCS	0x14
#define AST_PECI_INT_CTRL	0x18
#define AST_PECI_INT_STS	0x1C
#define AST_PECI_W_DATA0	0x20
#define AST_PECI_W_DATA1	0x24
#define AST_PECI_W_DATA2	0x28
#define AST_PECI_W_DATA3	0x2c
#define AST_PECI_R_DATA0	0x30
#define AST_PECI_R_DATA1	0x34
#define AST_PECI_R_DATA2	0x38
#define AST_PECI_R_DATA3	0x3c
#define AST_PECI_W_DATA4	0x40
#define AST_PECI_W_DATA5	0x44
#define AST_PECI_W_DATA6	0x48
#define AST_PECI_W_DATA7	0x4c
#define AST_PECI_R_DATA4	0x50
#define AST_PECI_R_DATA5	0x54
#define AST_PECI_R_DATA6	0x58
#define AST_PECI_R_DATA7	0x5c


/* AST_PECI_CTRL - 0x00 : Control Register */
#define PECI_CTRL_SAMPLING_MASK			(0xf << 16)
#define PECI_CTRL_SAMPLING(x)			(x << 16)
#define PECI_CTRL_READ_MODE_MASK		(0xf << 12)
#define PECI_CTRL_CONT_MODE				(1 << 16)
#define PECI_CTRL_DBG_MODE				(2 << 16)
#define PECI_CTRL_CLK_SOURCE			(0x1 << 11)	//0: 24Mhz, 1: MCLK
#define PECI_CTRL_CLK_DIV_MASK			(0x3 << 8)	
#define PECI_CTRL_CLK_DIV(x)			(x << 8)	
#define PECI_CTRL_INVERT_OUT			(0x1 << 7)	
#define PECI_CTRL_INVERT_IN				(0x1 << 6)	
#define PECI_CTRL_BUS_CONTENT_EN		(0x1 << 5)	
#define PECI_CTRL_PECI_EN				(0x1 << 4)	
#define PECI_CTRL_PECI_CLK_EN			(0x1)	

/* AST_PECI_TIMING - 0x04 : Timing Negotiation */
#define PECI_TIMING_MESSAGE_GET(x)		((x & 0xff00) >> 8)
#define PECI_TIMING_MESSAGE(x)			(x << 8)
#define PECI_TIMING_ADDRESS_GET(x)		(x & 0xff)
#define PECI_TIMING_ADDRESS(x)			(x)

/* AST_PECI_CMD	- 0x08 : Command Register */
#define PECI_CMD_PIN_MON				(0x1 << 31)
#define PECI_CMD_STS					(0xf << 24)
#define PECI_CMD_FIRE					(0x1)

/* AST_PECI_LEN	- 0x0C : Read/Write Length Register */
#define PECI_AW_FCS_EN					(0x1 << 31)
#define PECI_READ_LEN_MASK				(0xff << 16)
#define PECI_READ_LEN(x)				(x << 16)
#define PECI_WRITE_LEN_MASK				(0xff << 8)
#define PECI_WRITE_LEN(x)				(x << 8)
#define PECI_TAGET_ADDR_MASK			(0xff)
#define PECI_TAGET_ADDR(x)				(x)


/* AST_PECI_EXP_FCS	- 0x10 : Expected FCS Data Register  */
#define PECI_PROGRAM_AW_FCS				(0xf << 24) 
#define PECI_EXPECT_READ_FCS			(0xf << 16) 
#define PECI_EXPECT_AW_FCS_AUTO			(0xf << 8) 
#define PECI_EXPECT_WRITE_FCS			(0xf) 

/* AST_PECI_CAP_FCS	- 0x14 : Captured FCS Data Register */
#define PECI_CAPTURE_READ_FCS(x)		((x & 0xff) >> 16) 
#define PECI_CAPTURE_WRITE_FCS			(0xff) 

/* AST_PECI_INT_CTRL/ STS  - 0x18/0x1c  : Interrupt Register */
#define PECI_INT_TIMING_RESULT_MASK		(0x3 << 30)
#define PECI_INT_TIMEOUT				(0x1 << 4)
#define PECI_INT_CONNECT				(0x1 << 3)
#define PECI_INT_W_FCS_BAD				(0x1 << 2)
#define PECI_INT_W_FCS_ABORT			(0x1 << 1)
#define PECI_INT_CMD_DONE				(0x1)

#define    AUTO_GEN_AWFCS                         1
//#define    ENABLE_BUS_CONTENTION                  0x20

#define    DISABLE_ENGINE                         0
#define    ENABLE_RX_ENGINE                       (1 << 0)
#define    ENABLE_TX_ENGINE                       (1 << 1)
#define    LEFT_CHANNEL_HIGH                      (1 << 16)
#define    DELAY_CLOCK_CYCLE                      (1 << 17)

#endif /* __ASM_ARCH_REGS_PECI_H */
