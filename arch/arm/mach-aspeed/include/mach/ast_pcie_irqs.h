/*
 *  arch/arm/plat-aspeed/include/plat/ast_pcie_irqs.h
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

#ifndef _PCIE_IRQS_H_
#define _PCIE_IRQS_H_                 			1

#define ARCH_NR_PCIE					4
#ifdef CONFIG_PCI_MSI
#define AST_NUM_MSI_IRQS				32
#else
#define AST_NUM_MSI_IRQS				0
#endif

#define IRQ_PCIE_INTA					(IRQ_PCIE_CHAIN_START)
#define IRQ_PCIE_INTB					(IRQ_PCIE_CHAIN_START + 1)
#define IRQ_PCIE_INTC					(IRQ_PCIE_CHAIN_START + 2)
#define IRQ_PCIE_INTD					(IRQ_PCIE_CHAIN_START + 3)
#define IRQ_PCIE_MSI0					(IRQ_PCIE_INTD + 1)		// support max 32 MSI

#endif
