/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-mctp.c 
* Author         : Ryan Chen
* Description   : AST MCTP Ctrl
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 


*   History      : 
*    1. 2013/07/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
	
#include <mach/platform.h>
#include <asm/io.h>
	
#include <mach/hardware.h>
#include <plat/regs-mctp.h>
#include <plat/ast_mctp.h>

//#define AST_MCTP_DEBUG 1

#ifdef AST_MCTP_DEBUG
#define MCTPDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define MCTPDBUG(fmt, args...)
#endif

static u32 ast_mctp_base = 0;
static u8 txTag = 0;
static inline u32 
ast_mctp_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_mctp_base + reg);
	
	MCTPDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_mctp_write(u32 val, u32 reg) 
{
	MCTPDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	writel(val, ast_mctp_base + reg);
}

//***********************************Information ***********************************

extern void ast_pcie_cfg_read(u8 type, u32 bdf_offset, u32 *value)
{
	u32 timeout =0;
	u32 desc3,desc2;
	txTag %= 0xf;
//	printf("type = %d, busfunc = %x \n",type, bdf);
	if((ast_mctp_read(AST_MCTP_INT) & MCTP_RX_COMPLETE) != 0)
		printk("EEEEEEEE  \n");
	
	ast_mctp_write(0x4000001 | (type << 24), AST_MCTP_TX_DESC3);	
	ast_mctp_write(0x200f | (txTag << 8), AST_MCTP_TX_DESC2);
	ast_mctp_write(bdf_offset, AST_MCTP_TX_DESC1);
	ast_mctp_write(0, AST_MCTP_TX_DESC0);
//	ast_mctp_write(0, AST_MCTP_TX_DATA);

	//trigger
	ast_mctp_write(7, AST_MCTP_CTRL);	
	//wait 
//	printf("trigger \n");
	while(!(ast_mctp_read(AST_MCTP_INT) & MCTP_RX_COMPLETE)) {
		timeout++;
		if(timeout > 10000) {
			printk("time out \n");
			*value = 0xffffffff;
			goto out;
		}
	};

	//read 
	desc3 = ast_mctp_read(AST_MCTP_RX_DESC3);	
	desc2 = ast_mctp_read(AST_MCTP_RX_DESC2);
	ast_mctp_read(AST_MCTP_RX_DESC1);	

	if( ((desc3 >> 24) == 0x4A) && 
		((desc3 & 0xfff) == 0x1) && 
		((desc2 & 0xe000) == 0)) {
		*value = ast_mctp_read(AST_MCTP_RX_DATA);

	} else {
		*value = 0xffffffff;		
		
	}

out:
	txTag++;
	ast_mctp_write(0x15, AST_MCTP_CTRL);
	ast_mctp_write(0x3, AST_MCTP_INT);	
	//wait 
	while(ast_mctp_read(AST_MCTP_INT) & MCTP_RX_COMPLETE);
	
}

extern void ast_pcie_cfg_write(u8 type, u32 bdf_offset, u32 data)
{
	txTag %= 0xf;

	ast_mctp_write(0x44000001 | (type << 24), AST_MCTP_TX_DESC3);	
	ast_mctp_write(0x200f | (txTag << 8), AST_MCTP_TX_DESC2);
	ast_mctp_write(bdf_offset, AST_MCTP_TX_DESC1);
	ast_mctp_write(0, AST_MCTP_TX_DESC0);
	ast_mctp_write(data, AST_MCTP_TX_DATA);	

	//trigger
	ast_mctp_write(7, AST_MCTP_CTRL);	
//	printf("trigger \n");	
	//wait 
	while(!(ast_mctp_read(AST_MCTP_INT) & MCTP_RX_COMPLETE));

	//read 
	ast_mctp_read(AST_MCTP_RX_DESC3);	
	ast_mctp_read(AST_MCTP_RX_DESC2);
	ast_mctp_read(AST_MCTP_RX_DESC1);	
	txTag++;	
	ast_mctp_write(0x15, AST_MCTP_CTRL);
	ast_mctp_write(0x3, AST_MCTP_INT);	
	//wait 
	while(ast_mctp_read(AST_MCTP_INT) & MCTP_RX_COMPLETE);

}

static int __init ast_mctp_init(void)
{
	MCTPDBUG("\n");
	ast_mctp_base = (u32)ioremap(AST_MCTP_BASE , SZ_256);
	return 0;
}

subsys_initcall(ast_mctp_init);

