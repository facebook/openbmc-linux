/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-wdt.c
* Author        : Ryan Chen
* Description   : AST WDT Device
*
* Copyright (C)  ASPEED Technology Inc.
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

*   History      :
*    1. 2012/09/15 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>

#include <plat/devs.h>


/* --------------------------------------------------------------------
 *  Watchdog
 * -------------------------------------------------------------------- */

#if defined(CONFIG_RTC_DRV_ASPEED) || defined(CONFIG_RTC_DRV_ASPEED_MODULE)

static struct resource ast_rtc_resource[] = {
	[0] = {
		.start	= AST_RTC_BASE,
		.end	= AST_RTC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
			.start = IRQ_RTC,
			.end = IRQ_RTC,
			.flags = IORESOURCE_IRQ,
	},
	
};

static struct platform_device ast_device_rtc = {
	.name		= "ast_rtc",
	.id		= -1,
	.resource       = ast_rtc_resource,
	.num_resources  = ARRAY_SIZE(ast_rtc_resource),
};

void __init ast_add_device_rtc(void)
{
	platform_device_register(&ast_device_rtc);
}
#else
void __init ast_add_device_rtc(void) {}
#endif
