/*
 *  arch/arm/plat-aspeed/include/plat/irqs.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifdef CONFIG_COLDFIRE
#else 
#include <plat/aspeed.h>
#include <mach/ast_i2c_irqs.h>
#include <mach/ast_sdhci_irqs.h>
#include <mach/ast_gpio_irqs.h>
//#include <mach/ast_lpc_irqs.h>
#include <mach/ast_egfx_irqs.h>
#endif

#if defined(CONFIG_ARCH_AST1010)
#include <asm/ast1010_irqs.h>
#elif defined(CONFIG_ARCH_AST1510)
#include <mach/ast1510_irqs.h>
#elif defined(CONFIG_ARCH_AST1520)
#include <mach/ast1520_irqs.h>
#elif defined(CONFIG_ARCH_AST2000)
#include <mach/ast2000_irqs.h>
#elif defined(CONFIG_ARCH_AST2100) 
#include <mach/ast2100_irqs.h>
#elif defined(CONFIG_ARCH_AST2200) 
#include <mach/ast2200_irqs.h>
#elif defined(CONFIG_ARCH_AST2300)
#include <mach/ast2300_irqs.h>
#elif defined(CONFIG_ARCH_AST2400)
#include <mach/ast2400_irqs.h>
#elif defined(CONFIG_ARCH_AST3100)
#include <mach/ast3100_irqs.h>
#elif defined(AST_SOC_G5)
#include <mach/ast_g5_irqs.h>
#else
#err "no define for irqs.h"
#endif

/*********************************************************************************/
//CVIC
#if defined(CONFIG_ARCH_AST1070)
//Companion chip irq
#include <mach/ast1070_irqs.h>
#include <mach/ast1070_i2c_irqs.h>
#endif

#if defined(CONFIG_AST2400_BMC)
#include <mach/ext_ast2400_irqs.h>
#endif
