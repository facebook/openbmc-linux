/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-rtc.c
* Author        : Ryan chen
* Description   : Socle Real Time Clock Device (RTC)
*
* Copyright (C) Socle Tech. Corp.
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
*    1. 2010/09/15 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>

#include <plat/devs.h>

#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  ASPEED SGPIO
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SGPIO_AST) || defined(CONFIG_SGPIO_AST_MODULE)
static struct resource ast_sgpio_resource[] = {
	[0] = {
		.start = AST_SGPIO_BASE,
		.end = AST_SGPIO_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SGPIO,
		.end = IRQ_SGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_device_sgpio = {
	.name		= "ast-sgpio",
	.id			= 0,
	.num_resources	= ARRAY_SIZE(ast_sgpio_resource),
	.resource	= ast_sgpio_resource,
};

extern void __init
ast_add_device_sgpio(void)
{
	ast_scu_multi_func_sgpio();

	platform_device_register(&ast_device_sgpio);
}

#else
extern void __init ast_add_device_sgpio(void) {}
#endif

