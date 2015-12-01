/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-adc.c
* Author        : Ryan chen
* Description   : ASPEED ADC Device
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
*    1. 2012/08/06 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#endif

/* --------------------------------------------------------------------
 *  ADC
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SENSORS_AST_ADC) || defined(CONFIG_SENSORS_AST_ADC_MODULE) || defined(CONFIG_SENSORS_AST1010_ADC) || defined(CONFIG_SENSORS_AST1010_ADC_MODULE)
static struct resource ast_adc_resources[] = {
	[0] = {
		.start = AST_ADC_BASE,
		.end = AST_ADC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_ADC,
		.end = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_adc_device = {
	.name = "ast_adc",
	.id = 0,
	.resource = ast_adc_resources,
	.num_resources = ARRAY_SIZE(ast_adc_resources),
};

void __init ast_add_device_adc(void)
{

	platform_device_register(&ast_adc_device);
}
#else
void __init ast_add_device_adc(void) {}
#endif
