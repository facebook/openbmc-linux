/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-snoop.c
* Author        : Ryan chen
* Description   : ASPEED SNOOP Device
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
*    1. 2012/11/29 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-snoop.h>

/* --------------------------------------------------------------------
 *  AST SNOOP -- DMA or 80Port SNOOP 
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SNOOP_AST) || defined(CONFIG_SNOOP_AST_MODULE)
static u64 ast_snoop_dma_mask = 0xffffffffUL;
	
static struct ast_snoop_channel snoop_ch0 = {
	.snoop_ch = 0,
	.snoop_port = 0x80,	
};

static struct ast_snoop_channel snoop_ch1 = {
	.snoop_ch = 1,
	.snoop_port = 0x81,	
};

static struct ast_snoop snoop = {
	.snoop_ch0 = &snoop_ch0,
	.snoop_ch1 = &snoop_ch1,
};

static struct platform_device ast_snoop_device = {
	.name	= "ast_snoop",
    .id = 0,
    .dev = {
        .dma_mask = &ast_snoop_dma_mask,
        .coherent_dma_mask = 0xffffffff,
        .platform_data = &snoop,
    },
};

struct ast_snoop_dma_channel snoop_dma_ch0 = {
	.snoop_ch = 0,
	.snoop_port = 0x3f8,	
	.snoop_mask = 7,		
};

static struct platform_device ast_snoop_dma_device = {
	.name	= "ast_snoop_dma",
    .id = 0,
    .dev = {
        .dma_mask = &ast_snoop_dma_mask,
        .coherent_dma_mask = 0xffffffff,
        .platform_data = &snoop_dma_ch0,
    },
};

void __init ast_add_device_snoop(void)
{
	platform_device_register(&ast_snoop_device);
	platform_device_register(&ast_snoop_dma_device);	
}
#else
void __init ast_add_device_snoop(void) {}
#endif

