/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-i2c.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/ext-devs.h>
#include <plat/regs-iic.h>
#include <plat/ast2400-scu.h>

/* --------------------------------------------------------------------
 *  I2C
 * -------------------------------------------------------------------- */
#ifdef AST_I2C_DEBUG
#define I2CDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define I2CDBUG(fmt, args...)
#endif

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)

static struct ast_i2c_driver_data ast2400_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz	
	.master_dma = BYTE_MODE,
	.slave_dma = BYTE_MODE,
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif	
	.get_i2c_clock = ast2400_get_pclk,
};

static u64 ast2400_i2c_dma_mask = 0xffffffffUL;
static struct resource ast2400_i2c_dev1_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE1,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE1 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = 0,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,
	},
	.resource = ast2400_i2c_dev1_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev1_resources),
};

static struct resource ast2400_i2c_dev2_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE2,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE2 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = 1,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev2_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev2_resources),
};

static struct resource ast2400_i2c_dev3_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE3,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE3 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = 2,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev3_resources),
};

static struct resource ast2400_i2c_dev4_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE4,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE4 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = 3,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev4_resources),
};

static struct resource ast2400_i2c_dev5_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE5,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE5 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = 4,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev5_resources),
};

static struct resource ast2400_i2c_dev6_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE6,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE6 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = 5,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev6_resources),
};

static struct resource ast2400_i2c_dev7_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE7,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE7 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = 6,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev7_resources),
};

static struct resource ast2400_i2c_dev8_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE8,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE8 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev8_device = {
	.name = "ast-i2c",
	.id = 7,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev8_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev8_resources),
};

static struct resource ast2400_i2c_dev9_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE9,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE9 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev9_device = {
	.name = "ast-i2c",
	.id = 8,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev9_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev9_resources),
};

static struct resource ast2400_i2c_dev10_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE10,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE10 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev10_device = {
	.name = "ast-i2c",
	.id = 9,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev10_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev10_resources),
};

static struct resource ast2400_i2c_dev11_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE11,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE11 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev11_device = {
	.name = "ast-i2c",
	.id = 10,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev11_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev11_resources),
};

static struct resource ast2400_i2c_dev12_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE12,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE12 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev12_device = {
	.name = "ast-i2c",
	.id = 11,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev12_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev12_resources),
};

static struct resource ast2400_i2c_dev13_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE13,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE13 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev13_device = {
	.name = "ast-i2c",
	.id = 12,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev13_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev13_resources),
};

static struct resource ast2400_i2c_dev14_resources[] = {
	[0] = {
		.start = AST_PCI_EXT_I2C + AST_I2C_DEVICE14,
		.end = AST_PCI_EXT_I2C + AST_I2C_DEVICE14 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST2400_I2C,
		.end = IRQ_AST2400_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast2400_i2c_dev14_device = {
	.name = "ast-i2c",
	.id = 13,
	.dev = {
		.dma_mask = &ast2400_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast2400_i2c_data,		
	},
	.resource = ast2400_i2c_dev14_resources,
	.num_resources = ARRAY_SIZE(ast2400_i2c_dev14_resources),
};

/*--------- I2C Board devices ------------*/
//ASPEED AST2300 EVB I2C Device 
#if defined(CONFIG_ARCH_AST2300) || defined(CONFIG_ARCH_AST2400) || defined(CONFIG_AST2400_BMC)
//Under I2C Dev 4
static struct i2c_board_info __initdata ast2400_i2c_board_info_4[] = {
	{
		I2C_BOARD_INFO("24c128", 0x50),						

		
	}
};
//Under I2C Dev 8
static struct i2c_board_info __initdata ast2400_i2c_board_info_8[] = {
	{
		I2C_BOARD_INFO("lm75b", 0x4a),						
	}
};

#endif

/*-------------------------------------*/
void __init ast2400_add_device_i2c(void)
{
	//I2C Multi-Pin
	ast2400_scu_multi_func_i2c();

	//SCU I2C Reset 
	ast2400_scu_init_i2c();

	ast2400_i2c_data.reg_gr = ioremap(AST_PCI_EXT_I2C, 4*SZ_16);
	if (!ast2400_i2c_data.reg_gr) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}

	ast2400_i2c_data.buf_pool= ioremap(AST_PCI_EXT_I2C+0x800, 2048);
	if (!ast2400_i2c_data.buf_pool) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}

	platform_device_register(&ast2400_i2c_dev1_device);
	platform_device_register(&ast2400_i2c_dev2_device);
	platform_device_register(&ast2400_i2c_dev3_device);
	platform_device_register(&ast2400_i2c_dev4_device);
	i2c_register_board_info(3, ast2400_i2c_board_info_4, ARRAY_SIZE(ast2400_i2c_board_info_4));
	platform_device_register(&ast2400_i2c_dev5_device);
	platform_device_register(&ast2400_i2c_dev6_device);
	platform_device_register(&ast2400_i2c_dev7_device);
	platform_device_register(&ast2400_i2c_dev8_device);
	i2c_register_board_info(7, ast2400_i2c_board_info_8, ARRAY_SIZE(ast2400_i2c_board_info_8));
	platform_device_register(&ast2400_i2c_dev9_device);	
	platform_device_register(&ast2400_i2c_dev10_device);
	platform_device_register(&ast2400_i2c_dev11_device);
	platform_device_register(&ast2400_i2c_dev12_device);
	platform_device_register(&ast2400_i2c_dev13_device);
	platform_device_register(&ast2400_i2c_dev14_device);
}
#else
void __init ast2400_add_device_i2c(void) {}
#endif
