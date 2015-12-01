/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-jtag.c
* Author        : Ryan chen
* Description   : ASPEED JTAG Device
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
*    1. 2013/12/06 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/irqs.h>
#include <asm/arch/platform.h>
#include <asm/arch/devs.h>
#include <asm/sizes.h>

#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#endif

/* --------------------------------------------------------------------
 *  PECI
 * -------------------------------------------------------------------- */

#if defined(CONFIG_AST_JTAG) || defined(CONFIG_AST_JTAG_MODULE)
static struct resource ast_jtag_resources[] = {
	[0] = {
		.start = AST_JTAG_BASE,
		.end = AST_JTAG_BASE + (SZ_16*4) - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_JTAG,
		.end = IRQ_JTAG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_jtag_device = {
	.name = "ast-jtag",
	.id = 0,
	.resource = ast_jtag_resources,
	.num_resources = ARRAY_SIZE(ast_jtag_resources),
};

void __init ast_add_device_jtag(void)
{
	platform_device_register(&ast_jtag_device);
}
#else
void __init ast_add_device_jtag(void) {}
#endif
