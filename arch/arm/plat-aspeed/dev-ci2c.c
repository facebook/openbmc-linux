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
/* --------------------------------------------------------------------
 *  CI2C
 * -------------------------------------------------------------------- */
#if defined(CONFIG_I2C_AST1070) || defined(CONFIG_I2C_AST1070_MODULE)

static struct ast_i2c_driver_data ast_ci2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz	
	.master_dma = DMA_MODE,
	.slave_dma = BYTE_MODE,
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif	
#ifdef CONFIG_AST_LPC_PLUS
	//use lpc+ clock	
	.get_i2c_clock = ast_get_lhclk,
#else 
	.get_i2c_clock = ast_get_d2_pll_clk,
#endif
};

static u64 ast_i2c_dma_mask = 0xffffffffUL;
static struct resource ast_ci2c_dev1_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE1,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE1 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev1_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 0,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,
	},
	.resource = ast_ci2c_dev1_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev1_resources),
};

static struct resource ast_ci2c_dev2_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE2,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE2 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev2_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 1,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev2_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev2_resources),
};

static struct resource ast_ci2c_dev3_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE3,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE3 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev3_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 2,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev3_resources),
};

static struct resource ast_ci2c_dev4_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE4,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE4 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev4_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev4_resources),
};

static struct resource ast_ci2c_dev5_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE5,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE5 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev5_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 4,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev5_resources),
};

static struct resource ast_ci2c_dev6_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE6,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE6 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev6_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 5,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev6_resources),
};

static struct resource ast_ci2c_dev7_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE7,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE7 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev7_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev7_resources),
};

static struct resource ast_ci2c_dev8_resources[] = {
	[0] = {
		.start = AST_C0_I2C_BASE + AST_CI2C_DEVICE8,
		.end = AST_C0_I2C_BASE + AST_CI2C_DEVICE8 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C0_I2C,
		.end = IRQ_C0_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_ci2c_dev8_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_ci2c_data,		
	},
	.resource = ast_ci2c_dev8_resources,
	.num_resources = ARRAY_SIZE(ast_ci2c_dev8_resources),
};

//
#if (CONFIG_AST1070_NR >= 2)

static struct ast_i2c_driver_data ast_c1_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz	
	.master_dma = DMA_MODE,
	.slave_dma = BYTE_MODE,
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif	
#ifdef CONFIG_ARCH_AST2300
		.get_i2c_clock = ast_get_d2_pll_clk,
#else //AST2400 use lpc+ clock	
		.get_i2c_clock = ast_get_lhclk,
#endif
};

static struct resource ast_c1_i2c_dev1_resources[] = {
	[0] = {
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE1,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE1 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 0,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE2,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE2 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 1,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE3,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE3 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 2,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE4,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE4 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 3,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE5,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE5 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 4,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE6,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE6 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 5,
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
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE7,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE7 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev7_resources),
};

static struct resource ast_c1_i2c_dev8_resources[] = {
	[0] = {
		.start = AST_C1_I2C_BASE + AST_CI2C_DEVICE8,
		.end = AST_C1_I2C_BASE + AST_CI2C_DEVICE8 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_C1_I2C,
		.end = IRQ_C1_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_c1_i2c_dev8_device = {
	.name = "ast-i2c",
	.id = NUM_BUS + 8 + 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_c1_i2c_data,		
	},
	.resource = ast_c1_i2c_dev8_resources,
	.num_resources = ARRAY_SIZE(ast_c1_i2c_dev8_resources),
};
#endif
//
/*-------------------------------------*/
void __init ast_add_device_ci2c(void)
{
	ast1070_scu_init_i2c(0);

	ast_ci2c_data.reg_gr = IO_ADDRESS2(AST_C0_I2C_BASE);
	if (!ast_ci2c_data.reg_gr) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}
	platform_device_register(&ast_ci2c_dev1_device);	
	platform_device_register(&ast_ci2c_dev2_device);	
	platform_device_register(&ast_ci2c_dev3_device);	
	platform_device_register(&ast_ci2c_dev4_device);	
	platform_device_register(&ast_ci2c_dev5_device); 
	platform_device_register(&ast_ci2c_dev6_device); 
	platform_device_register(&ast_ci2c_dev7_device); 
	platform_device_register(&ast_ci2c_dev8_device); 


#if (CONFIG_AST1070_NR >= 2)

	ast1070_scu_init_i2c(1);

	ast_c1_i2c_data.reg_gr = IO_ADDRESS2(AST_C1_I2C_BASE);
	if (!ast_c1_i2c_data.reg_gr) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}
	platform_device_register(&ast_c1_i2c_dev1_device); 
	platform_device_register(&ast_c1_i2c_dev2_device); 
	platform_device_register(&ast_c1_i2c_dev3_device); 
	platform_device_register(&ast_c1_i2c_dev4_device); 
	platform_device_register(&ast_c1_i2c_dev5_device); 
	platform_device_register(&ast_c1_i2c_dev6_device); 
	platform_device_register(&ast_c1_i2c_dev7_device); 
	platform_device_register(&ast_c1_i2c_dev8_device); 
#endif

}
#else
void __init ast_add_device_ci2c(void) {}
#endif
