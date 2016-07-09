/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-snoop.c
* Author        : Ryan chen
* Description   : ASPEED SNOOP
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
 *  SNOOP
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_SNOOP) || defined(CONFIG_AST_SNOOP_MODULE)
static u64 ast_snoop_dma_mask = 0xffffffffUL;

static struct platform_device ast_snoop0_device = {
	.name	= "ast-snoop",
	.id = 0,
	.dev = {
		.dma_mask = &ast_snoop_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
};

static struct platform_device ast_snoop1_device = {
	.name	= "ast-snoop",
	.id = 1,
	.dev = {
		.dma_mask = &ast_snoop_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
};

static struct platform_device ast_snoop_dma_device = {
	.name	= "ast-snoop-dma",
	.id = 0,
	.dev = {
		.dma_mask = &ast_snoop_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
};

void __init ast_add_device_snoop(void)
{
	platform_device_register(&ast_snoop0_device);
	platform_device_register(&ast_snoop1_device);
	platform_device_register(&ast_snoop_dma_device);
}
#else
void __init ast_add_device_snoop(void) {}
#endif
