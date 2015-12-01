/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-rng.c
* Author        : Ryan Chen
* Description   : AST RNG Device
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

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <plat/ast-scu.h>
#include <plat/devs.h>
#endif

/* --------------------------------------------------------------------
 *  Watchdog
 * -------------------------------------------------------------------- */

#if defined(CONFIG_HW_RANDOM_AST) || defined(CONFIG_HW_RANDOM_AST_MODULE)
static struct platform_device ast_device_rng = {
	.name		= "ast-rng",
	.id		= 0,
};

void __init ast_add_device_rng(void)
{
	platform_device_register(&ast_device_rng);
}
#else
void __init ast_add_device_rng(void) {}
#endif
