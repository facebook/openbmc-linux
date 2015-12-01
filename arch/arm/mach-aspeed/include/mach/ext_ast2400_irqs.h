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

#ifndef _EXT_AST2400_IRQS_H_
#define _EXT_AST2400_IRQS_H_                 1

/*
#define ARCH_NR_GPIOS 					(GPIO_PORT_NUM*8)
#define IRQ_GPIO_CHAIN_START			(AST_VIC_NUM)

#define NR_IRQS                         (AST_VIC_NUM + ARCH_NR_GPIOS)
*/
#define AST2400_VIC_NUM			51

#define IRQ_AST2400_SDRAM_ECC					(IRQ_EXT_CHAIN_START + 0)
#define IRQ_AST2400_MIC							(IRQ_EXT_CHAIN_START + 1)
#define IRQ_AST2400_MAC0						(IRQ_EXT_CHAIN_START + 2)			/* MAC 1 interrupt */
#define IRQ_AST2400_MAC1						(IRQ_EXT_CHAIN_START + 3)			/* MAC 2 interrupt */

#define IRQ_AST2400_UART1						(IRQ_EXT_CHAIN_START + 9)			/* UART 1 interrupt */
#define IRQ_AST2400_UART0						(IRQ_EXT_CHAIN_START + 10)			/* UART 5 interrupt */
//11 Reserved
#define IRQ_AST2400_I2C							(IRQ_EXT_CHAIN_START + 12)
#define IRQ_AST2400_GPIO						(IRQ_EXT_CHAIN_START + 20)
#define IRQ_AST2400_SCU							(IRQ_EXT_CHAIN_START + 21)

#define IRQ_AST2400_UART2						(IRQ_EXT_CHAIN_START + 32)			/* UART 2 interrupt */
#define IRQ_AST2400_UART3						(IRQ_EXT_CHAIN_START + 33)			/* UART 3 interrupt */
#define IRQ_AST2400_UART4						(IRQ_EXT_CHAIN_START + 34)			/* UART 4 interrupt */

#endif
