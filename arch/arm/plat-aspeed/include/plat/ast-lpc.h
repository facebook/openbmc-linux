/*
 * Platform data for AST LPC .
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

struct ast_lpc_bus_info
{
	u8 lpc_dev_mode; /* 0: host mode , 1: dev mode*/	
	u8 bus_scan;
	u8 scan_node;	
	u8 lpc_mode; /* 0: lpc , 1: lpc+ */
	u32 bridge_phy_addr;
};

struct ast_lpc_driver_data
{
	struct platform_device *pdev;
	void __iomem		*reg_base;			/* virtual */	
	int 				irq;				//I2C IRQ number 
	u32					bus_id;				//for i2c dev# IRQ number check 
	struct ast_lpc_bus_info *bus_info;
};

extern struct ast_lpc_info *ast_get_lpc_info(void);
