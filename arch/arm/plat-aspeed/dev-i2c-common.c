/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-i2c-common.c
*
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
*
* Modified based on AST SDK
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/gpio.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/devs.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)

static struct ast_i2c_driver_data ast_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
	.get_i2c_clock = ast_get_pclk,
};

static u64 ast_i2c_dma_mask = 0xffffffffUL;
static struct resource ast_i2c_dev0_resources[] = {
	[0] = {
		.start = AST_I2C_DEV0_BASE,
		.end = AST_I2C_DEV0_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV0,
		.end = IRQ_I2C_DEV0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev0_device = {
	.name = "ast-i2c",
	.id = 0,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev0_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev0_resources),
};

static struct resource ast_i2c_dev1_resources[] = {
	[0] = {
		.start = AST_I2C_DEV1_BASE,
		.end = AST_I2C_DEV1_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV1,
		.end = IRQ_I2C_DEV1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = 1,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev1_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev1_resources),
};

static struct resource ast_i2c_dev2_resources[] = {
	[0] = {
		.start = AST_I2C_DEV2_BASE,
		.end = AST_I2C_DEV2_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV2,
		.end = IRQ_I2C_DEV2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = 2,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev2_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev2_resources),
};

static struct resource ast_i2c_dev3_resources[] = {
	[0] = {
		.start = AST_I2C_DEV3_BASE,
		.end = AST_I2C_DEV3_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV3,
		.end = IRQ_I2C_DEV3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev3_resources),
};

#if defined(AST_I2C_DEV4_BASE)
static struct resource ast_i2c_dev4_resources[] = {
	[0] = {
		.start = AST_I2C_DEV4_BASE,
		.end = AST_I2C_DEV4_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV4,
		.end = IRQ_I2C_DEV4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = 4,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev4_resources),
};
#endif

#if defined(AST_I2C_DEV5_BASE)
static struct resource ast_i2c_dev5_resources[] = {
	[0] = {
		.start = AST_I2C_DEV5_BASE,
		.end = AST_I2C_DEV5_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV5,
		.end = IRQ_I2C_DEV5,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = 5,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev5_resources),
};
#endif

#if defined(AST_I2C_DEV6_BASE)
static struct resource ast_i2c_dev6_resources[] = {
	[0] = {
		.start = AST_I2C_DEV6_BASE,
		.end = AST_I2C_DEV6_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV6,
		.end = IRQ_I2C_DEV6,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev6_resources),
};
#endif

#if defined(AST_I2C_DEV7_BASE)
static struct resource ast_i2c_dev7_resources[] = {
	[0] = {
		.start = AST_I2C_DEV7_BASE,
		.end = AST_I2C_DEV7_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV7,
		.end = IRQ_I2C_DEV7,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev7_resources),
};
#endif

#if defined(AST_I2C_DEV8_BASE)
static struct resource ast_i2c_dev8_resources[] = {
	[0] = {
		.start = AST_I2C_DEV8_BASE,
		.end = AST_I2C_DEV8_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV8,
		.end = IRQ_I2C_DEV8,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev8_device = {
	.name = "ast-i2c",
	.id = 8,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev8_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev8_resources),
};
#endif

#if defined(AST_I2C_DEV9_BASE)
static struct resource ast_i2c_dev9_resources[] = {
	[0] = {
		.start = AST_I2C_DEV9_BASE,
		.end = AST_I2C_DEV9_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV9,
		.end = IRQ_I2C_DEV9,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev9_device = {
	.name = "ast-i2c",
	.id = 9,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev9_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev9_resources),
};
#endif

#if defined(AST_I2C_DEV10_BASE)
static struct resource ast_i2c_dev10_resources[] = {
	[0] = {
		.start = AST_I2C_DEV10_BASE,
		.end = AST_I2C_DEV10_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV10,
		.end = IRQ_I2C_DEV10,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev10_device = {
	.name = "ast-i2c",
	.id = 10,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev10_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev10_resources),
};
#endif

#if defined(AST_I2C_DEV11_BASE)
static struct resource ast_i2c_dev11_resources[] = {
	[0] = {
		.start = AST_I2C_DEV11_BASE,
		.end = AST_I2C_DEV11_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV11,
		.end = IRQ_I2C_DEV11,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev11_device = {
	.name = "ast-i2c",
	.id = 11,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev11_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev11_resources),
};
#endif

#if defined(AST_I2C_DEV12_BASE)
static struct resource ast_i2c_dev12_resources[] = {
	[0] = {
		.start = AST_I2C_DEV12_BASE,
		.end = AST_I2C_DEV12_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV12,
		.end = IRQ_I2C_DEV12,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev12_device = {
	.name = "ast-i2c",
	.id = 12,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev12_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev12_resources),
};
#endif

#if defined(AST_I2C_DEV13_BASE)
static struct resource ast_i2c_dev13_resources[] = {
	[0] = {
		.start = AST_I2C_DEV13_BASE,
		.end = AST_I2C_DEV13_BASE + SZ_64 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C_DEV13,
		.end = IRQ_I2C_DEV13,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_i2c_dev13_device = {
	.name = "ast-i2c",
	.id = 13,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev13_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev13_resources),
};
#endif

void __init ast_add_device_i2c_common(void)
{
	//I2C Multi-Pin
	ast_scu_multi_func_i2c();

	ast_i2c_data.reg_gr = ioremap(AST_I2C_BASE, 4*SZ_16);
	if (!ast_i2c_data.reg_gr) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}

	platform_device_register(&ast_i2c_dev0_device);
	platform_device_register(&ast_i2c_dev1_device);
	platform_device_register(&ast_i2c_dev2_device);
	platform_device_register(&ast_i2c_dev3_device);
	platform_device_register(&ast_i2c_dev4_device);
	platform_device_register(&ast_i2c_dev5_device);
	platform_device_register(&ast_i2c_dev6_device);
	platform_device_register(&ast_i2c_dev7_device);
	platform_device_register(&ast_i2c_dev8_device);

#if !defined(CONFIG_MMC_AST)
	/*
	 * i2c buses 9, 10, 11, 12 share pins with eMMC.
	 * not used in this board.
	 */
#if defined(AST_I2C_DEV9_BASE)
	platform_device_register(&ast_i2c_dev9_device);
#endif

#if defined(AST_I2C_DEV10_BASE)
	platform_device_register(&ast_i2c_dev10_device);
#endif

#if defined(AST_I2C_DEV11_BASE)
	platform_device_register(&ast_i2c_dev11_device);
#endif
#if defined(AST_I2C_DEV12_BASE)
	platform_device_register(&ast_i2c_dev12_device);
#endif
#endif

#if defined(AST_I2C_DEV13_BASE)
	platform_device_register(&ast_i2c_dev13_device);
#endif
}

#else

void __init ast_add_device_i2c_common(void) {}

#endif

void __init __attribute__((weak)) ast_add_device_i2c(void)
{
	ast_add_device_i2c_common();
}
