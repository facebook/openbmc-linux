/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-bt.c
* Author        : Ryan chen
* Description   : ASPEED BT
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2016/03/29 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  BT
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_IPMI_BT) || defined(CONFIG_AST_IPMI_BT_MODULE)
static u64 ast_bt_dma_mask = 0xffffffffUL;

static struct platform_device ast_bt0_device = {
	.name	= "ast-bt",
	.id = 0,
	.dev = {
		.dma_mask = &ast_bt_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
};

static struct platform_device ast_bt1_device = {
	.name	= "ast-bt",
	.id = 1,
	.dev = {
		.dma_mask = &ast_bt_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
};

void __init ast_add_device_bt(void)
{
	platform_device_register(&ast_bt0_device);
	platform_device_register(&ast_bt1_device);
}
#else
void __init ast_add_device_bt(void) {}
#endif
