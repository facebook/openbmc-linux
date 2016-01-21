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
/* --------------------------------------------------------------------
 *  I2C
 * -------------------------------------------------------------------- */
#ifdef AST_I2C_DEBUG
#define I2CDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define I2CDBUG(fmt, args...)
#endif

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)

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

/*--------- I2C Board devices ------------*/
//AST EVB I2C Device 
static struct i2c_board_info __initdata ast_i2c_board_info_0[] = {
	{	
		I2C_BOARD_INFO("ddc", 0x50),
	},
#if defined(CONFIG_ARCH_AST3200)	
	{
		.type	= "cat66121_hdmi",
		.addr	= 0x4C,
		.flags	= 0,
		.irq		= IRQ_GPIOB0,
//		.platform_data	= &it66121_hdmi0_pdata,
	},
	{
		.type	= "cat66121_hdmi",
		.addr	= 0x4D,
		.flags	= 1,
		.irq		= IRQ_GPIOB1,
//		.platform_data	= &it66121_hdmi1_pdata,
	},	
#endif	
};

static struct i2c_board_info __initdata ast_i2c_board_info_1[] = {
#if defined(CONFIG_ARCH_AST3200)	
	{
		.type	= "cat66121_hdmi",
		.addr	= 0x4C,
		.flags	= 0,
		.irq		= IRQ_GPIOB2,
//		.platform_data 	= &it66121_hdmi2_pdata,
	},
	{
		.type	= "cat66121_hdmi",
		.addr	= 0x4D,
		.flags	= 1,
		.irq		= IRQ_GPIOB3,
//		.platform_data 	= &it66121_hdmi3_pdata,
	},	
#endif	
};

static struct i2c_board_info __initdata ast_i2c_board_info_2[] = {
};

//Under I2C Dev 3
static struct i2c_board_info __initdata ast_i2c_board_info_3[] = {
	{
		I2C_BOARD_INFO("24c08", 0x50),						
	},
};
//Under I2C Dev 8
static struct i2c_board_info __initdata ast_i2c_board_info_7[] = {
	{
		I2C_BOARD_INFO("lm75", 0x4d),						
	}
};

/*-------------------------------------*/
#ifdef CONFIG_CMM
void __init ast_add_device_i2c(void)
{
  //I2C Multi-Pin
  ast_scu_multi_func_i2c();

  platform_device_register(&ast_i2c_dev0_device);
  platform_device_register(&ast_i2c_dev1_device);
  platform_device_register(&ast_i2c_dev2_device);
  platform_device_register(&ast_i2c_dev3_device);
  platform_device_register(&ast_i2c_dev4_device);
  platform_device_register(&ast_i2c_dev5_device);
  platform_device_register(&ast_i2c_dev6_device);
  platform_device_register(&ast_i2c_dev7_device);
  platform_device_register(&ast_i2c_dev8_device);

  /*
   * i2c buses 9, 10, 11, 12 share pins with eMMC.
   * not used in this board.
   */

  platform_device_register(&ast_i2c_dev13_device);
}

#else

void __init ast_add_device_i2c(void)
{
	//I2C Multi-Pin
	ast_scu_multi_func_i2c();

	platform_device_register(&ast_i2c_dev0_device);
	platform_device_register(&ast_i2c_dev1_device);
	platform_device_register(&ast_i2c_dev2_device);
	platform_device_register(&ast_i2c_dev3_device);
	platform_device_register(&ast_i2c_dev4_device);
	platform_device_register(&ast_i2c_dev5_device);
	platform_device_register(&ast_i2c_dev6_device);
	platform_device_register(&ast_i2c_dev7_device);
	platform_device_register(&ast_i2c_dev8_device);

#if defined(AST_I2C_DEV9_BASE)
	platform_device_register(&ast_i2c_dev9_device);	
#endif
	
#if defined(CONFIG_MMC_AST)
	//Due to share pin with SD 
#else
#if defined(AST_I2C_DEV10_BASE)
	platform_device_register(&ast_i2c_dev10_device);
#endif

#if defined(AST_I2C_DEV11_BASE)
	platform_device_register(&ast_i2c_dev11_device);
#endif	
#if defined(AST_I2C_DEV12_BASE)
	platform_device_register(&ast_i2c_dev12_device);
#endif	
#if defined(AST_I2C_DEV13_BASE)
	platform_device_register(&ast_i2c_dev13_device);
#endif
#endif	
	
	i2c_register_board_info(0, ast_i2c_board_info_0, ARRAY_SIZE(ast_i2c_board_info_0));	
	i2c_register_board_info(1, ast_i2c_board_info_1, ARRAY_SIZE(ast_i2c_board_info_1));	
	i2c_register_board_info(2, ast_i2c_board_info_2, ARRAY_SIZE(ast_i2c_board_info_2));		
	i2c_register_board_info(3, ast_i2c_board_info_3, ARRAY_SIZE(ast_i2c_board_info_3));
	i2c_register_board_info(7, ast_i2c_board_info_7, ARRAY_SIZE(ast_i2c_board_info_7));
}

#endif

#else

void __init ast_add_device_i2c(void) {}

#endif
