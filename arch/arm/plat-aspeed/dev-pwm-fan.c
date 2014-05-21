/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-pwm-fan.c
* Author        : Ryan chen
* Description   : ASPEED PWM-FAN Device
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

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>
#include <mach/ast_pwm_techo.h>

/* --------------------------------------------------------------------
 *  PWM-FAN
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SENSORS_AST_PWM_FAN) || defined(CONFIG_SENSORS_AST_PWM_FAN_MODULE)
static struct resource ast_pwm_fan_resources[] = {
	[0] = {
		.start = AST_PWM_BASE,
		.end = AST_PWM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TACHO,
		.end = IRQ_TACHO,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ast_pwm_driver_data ast_pwm_data = {
	.get_pwm_clock = ast_get_h_pll_clk,
};

struct platform_device ast_pwm_fan_device = {
	.name = "ast_pwm_tacho",
	.id = 0,
	.dev = {
		.platform_data = &ast_pwm_data,
	},	
	.resource = ast_pwm_fan_resources,
	.num_resources = ARRAY_SIZE(ast_pwm_fan_resources),
};

void __init ast_add_device_pwm_fan(void)
{
	//SCU Initial 

	//SCU Pin-MUX 	//PWM & TACHO 
	ast_scu_multi_func_pwm_tacho();

	//SCU PWM CTRL Reset
	ast_scu_init_pwm_tacho();	

	platform_device_register(&ast_pwm_fan_device);
}
#else
void __init ast_add_device_pwm_fan(void) {}
#endif
