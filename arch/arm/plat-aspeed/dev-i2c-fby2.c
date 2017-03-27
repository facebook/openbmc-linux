/*
 * dev-i2c-fby2.c - i2c device definition for Yosemite V2.
 *
 * Copyright 2016-present Facebook. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/gpio.h>

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/devs.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>
#endif

static struct ast_i2c_driver_data ast_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
#ifdef CONFIG_AST_I2C_SLAVE_EEPROM
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif
	.get_i2c_clock = ast_get_pclk,
};

static struct ast_i2c_driver_data ast_i2c_data_400K = {
	.bus_clk = 400000,		//bus clock 400KHz
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
#ifdef CONFIG_AST_I2C_SLAVE_EEPROM
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif
	.get_i2c_clock = ast_get_pclk,
};

static struct ast_i2c_driver_data ast_i2c_data_1M = {
	.bus_clk = 1000000,		//bus clock 1M
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
#ifdef CONFIG_AST_I2C_SLAVE_EEPROM
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif
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

struct platform_device ast_i2c_dev0_device = {
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

struct platform_device ast_i2c_dev1_device = {
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

struct platform_device ast_i2c_dev1_device_1M = {
        .name = "ast-i2c",
        .id = 1,
        .dev = {
	        .dma_mask = &ast_i2c_dma_mask,
	        .coherent_dma_mask = 0xffffffff,
                .platform_data = &ast_i2c_data_1M,
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

struct platform_device ast_i2c_dev2_device = {
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

struct platform_device ast_i2c_dev3_device = {
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

struct platform_device ast_i2c_dev3_device_1M = {
	.name = "ast-i2c",
	.id = 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data_1M,
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

struct platform_device ast_i2c_dev4_device = {
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

struct platform_device ast_i2c_dev5_device = {
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

struct platform_device ast_i2c_dev5_device_1M = {
        .name = "ast-i2c",
        .id = 5,
        .dev = {
                .dma_mask = &ast_i2c_dma_mask,
                .coherent_dma_mask = 0xffffffff,
                .platform_data = &ast_i2c_data_1M,
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

struct platform_device ast_i2c_dev6_device = {
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

struct platform_device ast_i2c_dev7_device = {
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

struct platform_device ast_i2c_dev7_device_1M = {
        .name = "ast-i2c",
        .id = 7,
        .dev = {
                .dma_mask = &ast_i2c_dma_mask,
                .coherent_dma_mask = 0xffffffff,
                .platform_data = &ast_i2c_data_1M,
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

struct platform_device ast_i2c_dev8_device = {
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

struct platform_device ast_i2c_dev9_device = {
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

struct platform_device ast_i2c_dev10_device = {
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

struct platform_device ast_i2c_dev11_device = {
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

struct platform_device ast_i2c_dev11_device_400K = {
	.name = "ast-i2c",
	.id = 11,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data_400K,
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

struct platform_device ast_i2c_dev12_device = {
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

struct platform_device ast_i2c_dev13_device = {
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

static struct i2c_board_info ast_i2c_board_info_0[] __initdata = {

};

static struct i2c_board_info ast_i2c_board_info_1[] __initdata = {
        // To BIC server board 0x40
        // To Glacier Point
        // EEPROM, 0xA2
        {
            I2C_BOARD_INFO("24c128" , 0x51),
        },
        // Inlet Temp Sensor (0x9A)
        {
            I2C_BOARD_INFO("tmp75", 0x4d),
        },
        // Outlet Temp Sensor (0x9B)
        {
            I2C_BOARD_INFO("tmp75", 0x4e),
        },
        //
        {
            I2C_BOARD_INFO("pca9551", 0x71),
        },
        {
            I2C_BOARD_INFO("pca9551", 0x20),
        },
        {
            I2C_BOARD_INFO("ina230" , 0x40),
        },
};

static struct i2c_board_info ast_i2c_board_info_2[] __initdata = {

};

static struct i2c_board_info ast_i2c_board_info_3[] __initdata = {
        // To BIC server board 0x40
};

static struct i2c_board_info ast_i2c_board_info_4[] __initdata = {

};

static struct i2c_board_info ast_i2c_board_info_5[] __initdata = {
        // To BIC server board 0x40
				// To Glacier Point
        // EEPROM, 0xA2
        {
            I2C_BOARD_INFO("24c128" , 0x51),
        },
        // Inlet Temp Sensor (0x9A)
        {
            I2C_BOARD_INFO("tmp75", 0x4d),
        },
        // Outlet Temp Sensor (0x9B)
        {
            I2C_BOARD_INFO("tmp75", 0x4e),
        },
        //
        {
            I2C_BOARD_INFO("pca9551", 0x71),
        },
        {
            I2C_BOARD_INFO("pca9551", 0x20),
        },
        {
            I2C_BOARD_INFO("ina230" , 0x40),
        },
};

static struct i2c_board_info ast_i2c_board_info_6[] __initdata = {

};

static struct i2c_board_info ast_i2c_board_info_7[] __initdata = {
        // To BIC server board 0x40
};

static struct i2c_board_info ast_i2c_board_info_8[] __initdata = {
        // EEPROM, 0xA2
        {
            I2C_BOARD_INFO("24c128", 0x51),
        },
};

static struct i2c_board_info ast_i2c_board_info_9[] __initdata = {
        // INLET TEMP Sensor (0x9C)
        {
            I2C_BOARD_INFO("tmp421", 0x4e),
        },
        // OUTLET TEMP Sensor (0x9E)
        {
            I2C_BOARD_INFO("tmp421", 0x4f),
        },
};

static struct i2c_board_info ast_i2c_board_info_10[] __initdata = {
        // ADM1278, 0x20 (8BIT) -> ML HSC
        {
            I2C_BOARD_INFO("adm1278", 0x40),
        },
};

static struct i2c_board_info ast_i2c_board_info_11[] __initdata = {
        // Mezz C connector -> OCP MEZZ
        {
            I2C_BOARD_INFO("tmp421", 0x1f),
        },
};

static struct i2c_board_info ast_i2c_board_info_12[] __initdata = {
        // Mezz C connector -> OCP MEZZ EEPROM
        {
            I2C_BOARD_INFO("24c64", 0x51),
        },
};

static struct i2c_board_info ast_i2c_board_info_13[] __initdata = {
        //USB Connector
};

void __init ast_add_device_i2c(void)
{
	/* I2C Multi-Pin */
	ast_scu_multi_func_i2c();

	ast_i2c_data.reg_gr = ioremap(AST_I2C_BASE, 4*SZ_16);
	if (!ast_i2c_data.reg_gr) {
		printk(KERN_ERR "ast_add_device_i2c ERROR \n");
		return;
	}

        ast_i2c_data_1M.reg_gr = ast_i2c_data.reg_gr;// 1MHz reg_gr setting

	platform_device_register(&ast_i2c_dev0_device);
	platform_device_register(&ast_i2c_dev1_device_1M);
	platform_device_register(&ast_i2c_dev2_device);
	platform_device_register(&ast_i2c_dev3_device_1M);
	platform_device_register(&ast_i2c_dev4_device);
	platform_device_register(&ast_i2c_dev5_device_1M);
	platform_device_register(&ast_i2c_dev6_device);
	platform_device_register(&ast_i2c_dev7_device_1M);
	platform_device_register(&ast_i2c_dev8_device);
	platform_device_register(&ast_i2c_dev9_device);
	platform_device_register(&ast_i2c_dev10_device);
	ast_i2c_data_400K.reg_gr = ast_i2c_data.reg_gr;// 400KHz reg_gr setting
	platform_device_register(&ast_i2c_dev11_device_400K);
	platform_device_register(&ast_i2c_dev12_device);
	platform_device_register(&ast_i2c_dev13_device);

	i2c_register_board_info(0, ast_i2c_board_info_0,
			ARRAY_SIZE(ast_i2c_board_info_0));
	i2c_register_board_info(1, ast_i2c_board_info_1,
			ARRAY_SIZE(ast_i2c_board_info_1));
	i2c_register_board_info(2, ast_i2c_board_info_2,
			ARRAY_SIZE(ast_i2c_board_info_2));
	i2c_register_board_info(3, ast_i2c_board_info_3,
			ARRAY_SIZE(ast_i2c_board_info_3));
	i2c_register_board_info(4, ast_i2c_board_info_4,
			ARRAY_SIZE(ast_i2c_board_info_4));
	i2c_register_board_info(5, ast_i2c_board_info_5,
			ARRAY_SIZE(ast_i2c_board_info_5));
	i2c_register_board_info(6, ast_i2c_board_info_6,
			ARRAY_SIZE(ast_i2c_board_info_6));
	i2c_register_board_info(7, ast_i2c_board_info_7,
			ARRAY_SIZE(ast_i2c_board_info_7));
	i2c_register_board_info(8, ast_i2c_board_info_8,
			ARRAY_SIZE(ast_i2c_board_info_8));
	i2c_register_board_info(9, ast_i2c_board_info_9,
			ARRAY_SIZE(ast_i2c_board_info_9));
	i2c_register_board_info(10, ast_i2c_board_info_10,
			ARRAY_SIZE(ast_i2c_board_info_10));
	i2c_register_board_info(11, ast_i2c_board_info_11,
			ARRAY_SIZE(ast_i2c_board_info_11));
	i2c_register_board_info(12, ast_i2c_board_info_12,
			ARRAY_SIZE(ast_i2c_board_info_12));
	i2c_register_board_info(13, ast_i2c_board_info_13,
			ARRAY_SIZE(ast_i2c_board_info_13));
}
