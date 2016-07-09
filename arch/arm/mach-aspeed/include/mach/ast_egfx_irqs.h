/*
 *  arch/arm/plat-aspeed/include/plat/ast_egfx_irqs.h
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

#ifndef _EGFX_IRQS_H_
#define _EGFX_IRQS_H_                 1

#define ARCH_NR_EGFX 					(32)

#define IRQ_BB_INT0						(IRQ_EGFX_CHAIN_START + 0)
#define IRQ_BB_DONE						(IRQ_EGFX_CHAIN_START + 1)
#define IRQ_BB_INT2						(IRQ_EGFX_CHAIN_START + 2)
#define IRQ_BB_ERROR					(IRQ_EGFX_CHAIN_START + 3)
#define IRQ_CMDQ_DONE					(IRQ_EGFX_CHAIN_START + 4)
#define IRQ_CMDQ_INT1					(IRQ_EGFX_CHAIN_START + 5)
#define IRQ_CMDQ_INT2					(IRQ_EGFX_CHAIN_START + 6)
#define IRQ_CMDQ_ENG_DONE				(IRQ_EGFX_CHAIN_START + 7)
#define IRQ_CMDQ_SW_CMD				(IRQ_EGFX_CHAIN_START + 8)
#define IRQ_CA_INT0						(IRQ_EGFX_CHAIN_START + 9)
#define IRQ_CA_INT1						(IRQ_EGFX_CHAIN_START + 10)
#define IRQ_CA_INT2						(IRQ_EGFX_CHAIN_START + 11)
#define IRQ_CA_INT3						(IRQ_EGFX_CHAIN_START + 12)
#define IRQ_CA_INT4						(IRQ_EGFX_CHAIN_START + 13)
#define IRQ_CA_INT5						(IRQ_EGFX_CHAIN_START + 14)
#define IRQ_BULK_INT0					(IRQ_EGFX_CHAIN_START + 15)
#define IRQ_BULK_INT1					(IRQ_EGFX_CHAIN_START + 16)
#define IRQ_RLE_INT0						(IRQ_EGFX_CHAIN_START + 17)
#define IRQ_RLE_INT1						(IRQ_EGFX_CHAIN_START + 18)
//#define IRQ_GPIOC3						(IRQ_EGFX_CHAIN_START + 19)
#define IRQ_EGFX_DEC_DONE				(IRQ_EGFX_CHAIN_START + 20)
#define IRQ_EGFX_DEC_EXCPT				(IRQ_EGFX_CHAIN_START + 21)
#define IRQ_EGFX_DEC_HANG				(IRQ_EGFX_CHAIN_START + 22)
#define IRQ_VMASK_DONE					(IRQ_EGFX_CHAIN_START + 23)
#define IRQ_VMASK_ERROR				(IRQ_EGFX_CHAIN_START + 24)
#define IRQ_VMASK_TIMEOUT				(IRQ_EGFX_CHAIN_START + 25)
#define IRQ_GMASK_DONE					(IRQ_EGFX_CHAIN_START + 26)
#define IRQ_GMASK_ERROR				(IRQ_EGFX_CHAIN_START + 27)
#define IRQ_GMASK_TIMEOUT				(IRQ_EGFX_CHAIN_START + 28)
//29
#define IRQ_ARBIT_ERROR0				(IRQ_EGFX_CHAIN_START + 30)
#define IRQ_ARBIT_ERROR1				(IRQ_EGFX_CHAIN_START + 31)

#endif
