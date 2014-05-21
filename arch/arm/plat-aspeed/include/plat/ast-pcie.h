/*
 * Platform data for AST PCIe Root Complex module.
 *
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __AST_PCIE_H_
#define __AST_PCIE_H_

struct ast_pcie_data {
	int msi_irq_base;
	int msi_irq_num;
	int force_x1;
	int msi_inv;		/* 1 = MSI ack requires "write 0 to clear" */
	unsigned short int device_id;
};

#endif

