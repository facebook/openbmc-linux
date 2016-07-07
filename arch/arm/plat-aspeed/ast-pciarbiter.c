/********************************************************************************
* File Name     : arch/arm/plat-aspeed/ast-pciarbiter.c
* Author         : Ryan Chen
* Description   : AST PCI arbiter
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
*    1. 2015/11/25 Ryan Chen Create
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <plat/ast-ahbc.h>

//#define AST_PCIARBITER_DEBUG

#ifdef AST_PCIARBITER_DEBUG
#define PCIARBITERDBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define PCIARBITERDBUG(fmt, args...)
#endif

void __iomem	*ast_pciarbiter_base = 0;

static inline u32
ast_pciarbiter_read(u32 reg)
{
	u32 val;
	val = readl(ast_pciarbiter_base + reg);
	PCIARBITERDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	return val;
}

static inline void
ast_pciarbiter_write(u32 val, u32 reg)
{
	PCIARBITERDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_pciarbiter_base + reg);
}

//***********************************Information ***********************************
extern void ast_pciarbiter_pcie_init(void)
{
	ast_pciarbiter_write(0xacedbd1f, 0x00); //unlock
	ast_pciarbiter_write(0x00010004, 0x04); //addr_04
	ast_pciarbiter_write(0x00000002, 0x08); //data_vgamm_enable
	ast_pciarbiter_write(0x00010010, 0x04); //addr_10
	ast_pciarbiter_write(0x80000000, 0x08); //data_vgamm_bar
	ast_pciarbiter_write(0x00000000, 0x00); //lock
}

static int __init ast_pciarbiter_init(void)
{
	PCIARBITERDBUG("\n");
	ast_pciarbiter_base = ioremap(AST_PCIARBITER_BASE , SZ_32);
	return 0;
}

arch_initcall(ast_pciarbiter_init);
