/*
 *  arch/arm/plat-aspeed/include/plat/aspeed.h
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

#if defined(CONFIG_ARCH_AST3200) || defined(CONFIG_ARCH_AST2500) || defined(CONFIG_ARCH_AST1520)
#define AST_MACH_NAME 		"AST-G5"
#define AST_SOC_G5
#define NEW_VIC
#elif defined(CONFIG_ARCH_AST1400) || defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST3100)
#define AST_MACH_NAME 		"AST-G4"
#define AST_SOC_G4
#define NEW_VIC
#elif defined(CONFIG_ARCH_AST1300) || defined(CONFIG_ARCH_AST2300) || defined(CONFIG_ARCH_AST1510)
#define AST_MACH_NAME 		"AST-G3"
#define AST_SOC_G3
#define NEW_VIC
#elif defined(CONFIG_ARCH_AST2150) || defined(CONFIG_ARCH_AST2200)
#define AST_SOC_G2_5
#elif defined(CONFIG_ARCH_AST1100) || defined(CONFIG_ARCH_AST2050) || defined(CONFIG_ARCH_AST2100)
#define AST_SOC_G2
#elif defined(CONFIG_ARCH_AST2000) || defined(CONFIG_ARCH_AST1000)
#define AST_SOC_G1
#elif defined(CONFIG_ARCH_AST1010)
#define AST_MACH_NAME 		"AST-Coldfire"
#define AST_SOC_COLDFIRE
#else
#error "Not define SoC generation"
#endif



