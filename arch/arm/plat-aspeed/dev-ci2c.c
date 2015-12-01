/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-ci2c.c
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

#include <plat/devs.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>
#include <plat/ast1070-scu.h>
#include <mach/gpio.h>

#include <plat/ast-lpc.h>

/* --------------------------------------------------------------------
 *  CI2C
 * -------------------------------------------------------------------- */
#if defined(CONFIG_I2C_AST1070) || defined(CONFIG_I2C_AST1070_MODULE)

static struct ast_i2c_driver_data ast_ci2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz	
	.master_dma = DEC_DMA_MODE,
	.slave_dma = BYTE_MODE,
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif	
};

static u64 ast_i2c_dma_mask = 0xffffffffUL;
static struct resource ast_c0_i2c_dev0_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV0_BASE,
		.end = AST1070_C0_I2C_DEV0_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV0,
		.end = IRQ_AST1070_C0_I2C_DEV0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev0_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 0,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,
	},
	.resource = ast_c0_i2c_dev0_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev0_resources),
};

static struct resource ast_c0_i2c_dev1_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV1_BASE,
		.end = AST1070_C0_I2C_DEV1_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV1,
		.end = IRQ_AST1070_C0_I2C_DEV1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 1,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev1_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev1_resources),
};

static struct resource ast_c0_i2c_dev2_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV2_BASE,
		.end = AST1070_C0_I2C_DEV2_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV2,
		.end = IRQ_AST1070_C0_I2C_DEV2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 2,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev2_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev2_resources),
};

static struct resource ast_c0_i2c_dev3_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV3_BASE,
		.end = AST1070_C0_I2C_DEV3_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV3,
		.end = IRQ_AST1070_C0_I2C_DEV3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev3_resources),
};

static struct resource ast_c0_i2c_dev4_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV4_BASE,
		.end = AST1070_C0_I2C_DEV4_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV4,
		.end = IRQ_AST1070_C0_I2C_DEV4,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 4,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev4_resources),
};

static struct resource ast_c0_i2c_dev5_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV5_BASE,
		.end = AST1070_C0_I2C_DEV5_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV5,
		.end = IRQ_AST1070_C0_I2C_DEV5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 5,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev5_resources),
};

static struct resource ast_c0_i2c_dev6_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV6_BASE,
		.end = AST1070_C0_I2C_DEV6_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV6,
		.end = IRQ_AST1070_C0_I2C_DEV6,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev6_resources),
};

static struct resource ast_c0_i2c_dev7_resources[] = {
	[0] = {
		.start = AST1070_C0_I2C_DEV7_BASE,
		.end = AST1070_C0_I2C_DEV7_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C0_I2C_DEV7,
		.end = IRQ_AST1070_C0_I2C_DEV7,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c0_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_c0_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_c0_i2c_dev7_resources),
};

//
static struct ast_i2c_driver_data ast_c1_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz	
	.master_dma = DMA_MODE,
	.slave_dma = BYTE_MODE,
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif	
};

static struct resource ast_c1_i2c_dev0_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV0_BASE,
		.end = AST1070_C1_I2C_DEV0_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV0,
		.end = IRQ_AST1070_C1_I2C_DEV0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev0_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 0,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,
	},
	.resource = ast_c1_i2c_dev0_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev0_resources),
};

static struct resource ast_c1_i2c_dev1_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV1_BASE,
		.end = AST1070_C1_I2C_DEV1_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV1,
		.end = IRQ_AST1070_C1_I2C_DEV1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 1,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,
	},
	.resource = ast_c1_i2c_dev1_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev1_resources),
};

static struct resource ast_c1_i2c_dev2_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV2_BASE,
		.end = AST1070_C1_I2C_DEV2_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV2,
		.end = IRQ_AST1070_C1_I2C_DEV2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 2,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev2_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev2_resources),
};

static struct resource ast_c1_i2c_dev3_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV3_BASE,
		.end = AST1070_C1_I2C_DEV3_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV3,
		.end = IRQ_AST1070_C1_I2C_DEV3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev3_resources),
};

static struct resource ast_c1_i2c_dev4_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV4_BASE,
		.end = AST1070_C1_I2C_DEV4_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV4,
		.end = IRQ_AST1070_C1_I2C_DEV4,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 4,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev4_resources),
};

static struct resource ast_c1_i2c_dev5_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV5_BASE,
		.end = AST1070_C1_I2C_DEV5_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV5,
		.end = IRQ_AST1070_C1_I2C_DEV5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 5,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev5_resources),
};

static struct resource ast_c1_i2c_dev6_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV6_BASE,
		.end = AST1070_C1_I2C_DEV6_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV6,
		.end = IRQ_AST1070_C1_I2C_DEV6,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev6_resources),
};

static struct resource ast_c1_i2c_dev7_resources[] = {
	[0] = {
		.start = AST1070_C1_I2C_DEV7_BASE,
		.end = AST1070_C1_I2C_DEV7_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AST1070_C1_I2C_DEV7,
		.end = IRQ_AST1070_C1_I2C_DEV7,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev7_resources),
};

//
/*-------------------------------------*/
void __init ast_add_device_ci2c(u8 chip, u32 lpc_base)
{
	if(chip == 0) {
		if(ast_scu_get_lpc_plus_enable())	//use lpc+ clock	
			ast_ci2c_data.get_i2c_clock = ast_get_lhclk;
		else
			ast_ci2c_data.get_i2c_clock = ast_get_d2_pll_clk;

		platform_device_register(&ast_c0_i2c_dev0_device);		
		platform_device_register(&ast_c0_i2c_dev1_device);	
		platform_device_register(&ast_c0_i2c_dev2_device);	
		platform_device_register(&ast_c0_i2c_dev3_device);
		platform_device_register(&ast_c0_i2c_dev4_device);	
		platform_device_register(&ast_c0_i2c_dev5_device); 
		platform_device_register(&ast_c0_i2c_dev6_device); 
		platform_device_register(&ast_c0_i2c_dev7_device); 
	}else if(chip == 1) {
		if(ast_scu_get_lpc_plus_enable())	//use lpc+ clock	
			ast_c1_i2c_data.get_i2c_clock = ast_get_lhclk;
		else
			ast_c1_i2c_data.get_i2c_clock = ast_get_d2_pll_clk;

		platform_device_register(&ast_c1_i2c_dev0_device); 
		platform_device_register(&ast_c1_i2c_dev1_device); 
		platform_device_register(&ast_c1_i2c_dev2_device); 
		platform_device_register(&ast_c1_i2c_dev3_device); 
		platform_device_register(&ast_c1_i2c_dev4_device); 
		platform_device_register(&ast_c1_i2c_dev5_device); 
		platform_device_register(&ast_c1_i2c_dev6_device); 
		platform_device_register(&ast_c1_i2c_dev7_device); 
		platform_device_register(&ast_c1_i2c_dev8_device); 
	}else {
	}

}
#else
void __init ast_add_device_ci2c(void) {}
#endif
