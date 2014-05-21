/*
 *  arch/arm/plat-aspeed/include/plat/gpio_irqs.h
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

#ifndef _LPC_IRQS_H_
#define _LPC_IRQS_H_                 1

#define AST_LPC_IRQ_NUM 		7

#define IRQ_KCS0						(IRQ_LPC_CHAIN_START + 0)
#define IRQ_KCS1						(IRQ_LPC_CHAIN_START + 1)
#define IRQ_KCS2						(IRQ_LPC_CHAIN_START + 2)
#define IRQ_KCS3						(IRQ_LPC_CHAIN_START + 3)
#define IRQ_KCS4						(IRQ_LPC_CHAIN_START + 4)
#define IRQ_SNOOP0						(IRQ_LPC_CHAIN_START + 5)
#define IRQ_SNOOP1						(IRQ_LPC_CHAIN_START + 6)

#endif
