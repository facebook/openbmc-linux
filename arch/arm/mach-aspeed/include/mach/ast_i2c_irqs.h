/*
 *  arch/arm/plat-aspeed/include/plat/ast_i2c_irqs.h
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

#ifndef _I2C_IRQS_H_
#define _I2C_IRQS_H_                 			1

#define ARCH_NR_I2C 					(14)

#define IRQ_I2C_DEV0					(IRQ_I2C_CHAIN_START + 0)
#define IRQ_I2C_DEV1					(IRQ_I2C_CHAIN_START + 1)
#define IRQ_I2C_DEV2					(IRQ_I2C_CHAIN_START + 2)
#define IRQ_I2C_DEV3					(IRQ_I2C_CHAIN_START + 3)
#define IRQ_I2C_DEV4					(IRQ_I2C_CHAIN_START + 4)
#define IRQ_I2C_DEV5					(IRQ_I2C_CHAIN_START + 5)
#define IRQ_I2C_DEV6					(IRQ_I2C_CHAIN_START + 6)
#define IRQ_I2C_DEV7					(IRQ_I2C_CHAIN_START + 7)
#define IRQ_I2C_DEV8					(IRQ_I2C_CHAIN_START + 8)
#define IRQ_I2C_DEV9					(IRQ_I2C_CHAIN_START + 9)
#define IRQ_I2C_DEV10					(IRQ_I2C_CHAIN_START + 10)
#define IRQ_I2C_DEV11					(IRQ_I2C_CHAIN_START + 11)
#define IRQ_I2C_DEV12					(IRQ_I2C_CHAIN_START + 12)
#define IRQ_I2C_DEV13					(IRQ_I2C_CHAIN_START + 13)

#endif
