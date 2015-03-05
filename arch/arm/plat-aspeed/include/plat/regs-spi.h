/********************************************************************************
* File Name     : regs-spi.h
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
********************************************************************************/

/* Register offsets */
#define AST_SPI_CONFIG			0x00
#define AST_SPI_CTRL			0x04
#define AST_SPI_MISC			0x10
#define AST_SPI_TIMING			0x14

/* AST_SPI_CONFIG 0x00 : SPI Flash Configuration Register */
#define SPI_CONF_CLKX2			(0x1 << 1)
#define SPI_CONF_WRITE_EN		(0x1)

/* FMC_CE0_CTRL	for SPI 0x10, 0x14, 0x18, 0x1c, 0x20 */
#define SPI_IO_MODE(x)			(x << 28)
#define SPI_SINGLE_BIT			0
#define SPI_DUAL_BIT_D			2
#define SPI_DUAL_BIT_DA			3
#define SPI_CE_WIDTH(x)			(x << 24)
#define SPI_CMD_DATA(x)			(x << 16)
#define SPI_DUMMY_CMD			(1 << 15)
#define SPI_DUMMY_HIGH			(1 << 14)
//#define SPI_CLK_DIV				(1 << 13)		?? TODO ask....
//#define SPI_ADDR_CYCLE			(1 << 13)		?? TODO ask....
#define SPI_CMD_MERGE_DIS		(1 << 12)
#define SPI_CLK_DIV(x)			(x << 8)
#define SPI_CLK_DIV_MASK		(0xf << 8)

#define SPI_DUMMY_LOW			(x << 6)
#define SPI_LSB_FIRST_CTRL		(1 << 5)
#define SPI_CPOL_1				(1 << 4)
#define SPI_DUAL_DATA			(1 << 3)
#define SPI_CE_INACTIVE			(1 << 2)
#define SPI_CMD_MODE			(x)
#define SPI_CMD_NOR_R_MODE		0
#define SPI_CMD_FAST_R_MODE		1
#define SPI_CMD_NOR_W_MODE		2
#define SPI_CMD_USER_MODE		3

