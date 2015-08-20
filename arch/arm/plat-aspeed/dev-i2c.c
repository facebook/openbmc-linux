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
#include <plat/devs.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  I2C
 * -------------------------------------------------------------------- */
#ifdef AST_I2C_DEBUG
#define I2CDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define I2CDBUG(fmt, args...)
#endif

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)

#if defined (CONFIG_ARCH_AST2400)
#define I2C_PAGE_SIZE 8
static spinlock_t page_info_lock = SPIN_LOCK_UNLOCKED;
static struct buf_page page_info[I2C_PAGE_SIZE] =
{
	[0] = {
		.flag = 0,
		.page_no = 0,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[1] = {
		.flag = 0,
		.page_no = 1,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[2] = {
		.flag = 0,
		.page_no = 2,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[3] = {
		.flag = 0,
		.page_no = 3,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[4] = {
		.flag = 0,
		.page_no = 4,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[5] = {
		.flag = 0,
		.page_no = 5,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[6] = {
		.flag = 0,
		.page_no = 6,
		.page_size = 256,
		.page_addr_point = 0,
	},
	[7] = {
		.flag = 0,
		.page_no = 7,
		.page_size = 256,
		.page_addr_point = 0,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr)
{
	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;

		page_info[i].page_addr = buf_pool_addr + offset;
//		I2CDBUG( "page[%d],addr :%x \n", i, page_info[i].page_addr);
	}

}

static u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
        unsigned long flags;
	//TODO

        spin_lock_irqsave(&page_info_lock, flags);
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
//			I2CDBUG( "request page addr %x \n", page_info[i].page_addr);
			break;
		}
	}
	spin_unlock_irqrestore(&page_info_lock, flags);
	return (i >= I2C_PAGE_SIZE);
}

static void free_pool_buff_page(struct buf_page *req_page)
{
        unsigned long flags;
        spin_lock_irqsave(&page_info_lock, flags);

	req_page->flag = 0;
//	I2CDBUG( "free page addr %x \n", req_page->page_addr);
	req_page = NULL;
	spin_unlock_irqrestore(&page_info_lock, flags);
}

#elif defined (CONFIG_ARCH_AST2300)
#define I2C_PAGE_SIZE 5

static spinlock_t page_info_lock = SPIN_LOCK_UNLOCKED;
static struct buf_page page_info[I2C_PAGE_SIZE] =
{
	[0] = {
		.flag = 0,
		.page_size = 128,
	},
	[1] = {
		.flag = 0,
		.page_size = 32,
	},
	[2] = {
		.flag = 0,
		.page_size = 32,
	},
	[3] = {
		.flag = 0,
		.page_size = 32,
	},
	[4] = {
		.flag = 0,
		.page_size = 32,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr)
{

	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;

		page_info[i].page_addr = buf_pool_addr + offset;
		page_info[i].page_addr_point = page_info[i].page_addr/4;
//		printk("page[%d],addr :%x , point : %d\n", i, page_info[i].page_addr, page_info[i].page_addr_point);
	}
}

static u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
        unsigned long flags;
	//TODO

        spin_lock_irqsave(&page_info_lock, flags);
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
			spin_unlock_irqrestore(&page_info_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&page_info_lock, flags);
	return 1;

}

//TODO check free ?
static void free_pool_buff_page(struct buf_page *req_page)
{
        unsigned long flags;
        spin_lock_irqsave(&page_info_lock, flags);
	req_page->flag = 0;
	req_page = NULL;
	spin_unlock_irqrestore(&page_info_lock, flags);
}

#else
//DO nothing
static void pool_buff_page_init(void) {}
static u8 request_pool_buff_page(struct buf_page **req_page) {return 0;}
static void free_pool_buff_page(struct buf_page *req_page) {}
#endif

static struct ast_i2c_driver_data ast_i2c_data = {
	.bus_clk = 100000,		//bus clock 100KHz
	.master_dma = BUFF_MODE,
	.slave_dma = BYTE_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	.slave_xfer = i2c_slave_xfer,
	.slave_init = i2c_slave_init,
#endif
	.get_i2c_clock = ast_get_pclk,
};

static u64 ast_i2c_dma_mask = 0xffffffffUL;
static struct resource ast_i2c_dev1_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE1,
		.end = AST_I2C_BASE + AST_I2C_DEVICE1 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev1_device = {
	.name = "ast-i2c",
	.id = 0,
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
		.start = AST_I2C_BASE + AST_I2C_DEVICE2,
		.end = AST_I2C_BASE + AST_I2C_DEVICE2 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev2_device = {
	.name = "ast-i2c",
	.id = 1,
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
		.start = AST_I2C_BASE + AST_I2C_DEVICE3,
		.end = AST_I2C_BASE + AST_I2C_DEVICE3 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev3_device = {
	.name = "ast-i2c",
	.id = 2,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev3_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev3_resources),
};

static struct resource ast_i2c_dev4_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE4,
		.end = AST_I2C_BASE + AST_I2C_DEVICE4 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev4_device = {
	.name = "ast-i2c",
	.id = 3,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev4_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev4_resources),
};

static struct resource ast_i2c_dev5_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE5,
		.end = AST_I2C_BASE + AST_I2C_DEVICE5 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev5_device = {
	.name = "ast-i2c",
	.id = 4,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev5_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev5_resources),
};

static struct resource ast_i2c_dev6_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE6,
		.end = AST_I2C_BASE + AST_I2C_DEVICE6 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev6_device = {
	.name = "ast-i2c",
	.id = 5,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev6_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev6_resources),
};

static struct resource ast_i2c_dev7_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE7,
		.end = AST_I2C_BASE + AST_I2C_DEVICE7 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev7_device = {
	.name = "ast-i2c",
	.id = 6,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev7_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev7_resources),
};

static struct resource ast_i2c_dev8_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE8,
		.end = AST_I2C_BASE + AST_I2C_DEVICE8 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev8_device = {
	.name = "ast-i2c",
	.id = 7,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev8_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev8_resources),
};

static struct resource ast_i2c_dev9_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE9,
		.end = AST_I2C_BASE + AST_I2C_DEVICE9 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev9_device = {
	.name = "ast-i2c",
	.id = 8,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev9_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev9_resources),
};

#if defined(CONFIG_ARCH_AST2400)
static struct resource ast_i2c_dev10_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE10,
		.end = AST_I2C_BASE + AST_I2C_DEVICE10 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev10_device = {
	.name = "ast-i2c",
	.id = 9,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev10_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev10_resources),
};

static struct resource ast_i2c_dev11_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE11,
		.end = AST_I2C_BASE + AST_I2C_DEVICE11 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev11_device = {
	.name = "ast-i2c",
	.id = 10,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev11_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev11_resources),
};

static struct resource ast_i2c_dev12_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE12,
		.end = AST_I2C_BASE + AST_I2C_DEVICE12 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev12_device = {
	.name = "ast-i2c",
	.id = 11,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev12_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev12_resources),
};

static struct resource ast_i2c_dev13_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE13,
		.end = AST_I2C_BASE + AST_I2C_DEVICE13 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev13_device = {
	.name = "ast-i2c",
	.id = 12,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev13_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev13_resources),
};

static struct resource ast_i2c_dev14_resources[] = {
	[0] = {
		.start = AST_I2C_BASE + AST_I2C_DEVICE14,
		.end = AST_I2C_BASE + AST_I2C_DEVICE14 + 4*SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_i2c_dev14_device = {
	.name = "ast-i2c",
	.id = 13,
	.dev = {
		.dma_mask = &ast_i2c_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &ast_i2c_data,
	},
	.resource = ast_i2c_dev14_resources,
	.num_resources = ARRAY_SIZE(ast_i2c_dev14_resources),
};
#endif

/*--------- I2C Board devices ------------*/
//ASPEED AST2300 EVB I2C Device
#if defined(CONFIG_ARCH_AST2300) || defined(CONFIG_ARCH_AST2400)

#if defined(CONFIG_WEDGE100)

static struct i2c_board_info __initdata wedge100_i2c_bus1[] = {
  /* shared NIC, no kernel driver */
};

static struct i2c_board_info __initdata wedge100_i2c_bus2[] = {
  /* TODO: one IR3581 and two IR3584 */
};

static struct i2c_board_info __initdata wedge100_i2c_bus3[] = {
  /* TODO: one PWR1014A */
};

static struct i2c_board_info __initdata wedge100_i2c_bus4[] = {
  {
    I2C_BOARD_INFO("tmp75", 0x48),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x49),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x4a),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x4b),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x4c),
  },
};

static struct i2c_board_info __initdata wedge100_i2c_bus5[] = {
  /* Panther+ microserver */
  {
    I2C_BOARD_INFO("fb_panther_plus", 0x40),
  },
};

static struct i2c_board_info __initdata wedge100_i2c_bus6[] = {
  /* TODO: USB hub */
};

static struct i2c_board_info __initdata wedge100_i2c_bus7[] = {
  {
    I2C_BOARD_INFO("pcf8574", 0x2f),
  },
  {
    I2C_BOARD_INFO("24c64", 0x51),
  }
};

static struct i2c_board_info __initdata wedge100_i2c_bus8[] = {
  /* BMC PHY EEPROM */
  {
    I2C_BOARD_INFO("24c02", 0x50),
  },
  // EEPROM on the pfe1100 power supplies
  {
    I2C_BOARD_INFO("24c64", 0x51),
  },
  {
    I2C_BOARD_INFO("24c64", 0x52),
  },
  /* PSU driver */
  {
    I2C_BOARD_INFO("pfe1100", 0x59),
  },
  {
    I2C_BOARD_INFO("pfe1100", 0x5a),
  },
};

static struct i2c_board_info __initdata wedge100_i2c_bus9[] = {
  {
    I2C_BOARD_INFO("fancpld", 0x33),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x48),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x49),
  },
};

/* i2c bus 10-12 on wedge100 are not connected as i2c bus */

static struct i2c_board_info __initdata wedge100_i2c_bus13[] = {
  {
    I2C_BOARD_INFO("syscpld", 0x31),
  },
};

/* i2c bus 14 on wedge100 are not connected as i2c bus */

/* end of defined(CONFIG_WEDGE100) */

#elif defined(CONFIG_YOSEMITE)

//Under I2C Dev 1
static struct i2c_board_info __initdata ast_i2c_board_info_1[] = {
  // Slot#0 NIC sideband
};

//Under I2C Dev 2
static struct i2c_board_info __initdata ast_i2c_board_info_2[] = {
  // Slot#0 IPMB interface
};

//Under I2C Dev 3
static struct i2c_board_info __initdata ast_i2c_board_info_3[] = {
  // Slot#1 NIC sideband
};


//Under I2C Dev 4
static struct i2c_board_info __initdata ast_i2c_board_info_4[] = {
  // Slot#1 IPMB interface
};

//Under I2C Dev 5
static struct i2c_board_info __initdata ast_i2c_board_info_5[] = {
  // Slot#2 NIC sideband
};

//Under I2C Dev 6
static struct i2c_board_info __initdata ast_i2c_board_info_6[] = {
  // Slot#2 IPMB interface
};

//Under I2C Dev 7
static struct i2c_board_info __initdata ast_i2c_board_info_7[] = {
  // Slot#3 NIC sideband
};

//Under I2C Dev 8
static struct i2c_board_info __initdata ast_i2c_board_info_8[] = {
  // Slot#3 IPMB interface
};

//Under I2C Dev 9
static struct i2c_board_info __initdata ast_i2c_board_info_9[] = {
  // FRUID
  {
    I2C_BOARD_INFO("24c64", 0x51),
  },
};

//Under I2C Dev 10
static struct i2c_board_info __initdata ast_i2c_board_info_10[] = {
  // Inlet and Outlet temp. sensors
  {
    I2C_BOARD_INFO("tmp75", 0x4e),
  },
  {
    I2C_BOARD_INFO("tmp75", 0x4f),
  },
};

//Under I2C Dev 11
static struct i2c_board_info __initdata ast_i2c_board_info_11[] = {
  // Hotswap Sensor
  {
    I2C_BOARD_INFO("adm1278", 0x40),
  },
};

//Under I2C Dev 12
static struct i2c_board_info __initdata ast_i2c_board_info_12[] = {
  // Mezz Card LAN_SMB bus (PHY, Temp. Sensor)
};

//Under I2C Dev 13
static struct i2c_board_info __initdata ast_i2c_board_info_13[] = {
  // Mezz Card Mezz_SMB bus (FRUID, GPIO expander, QSFP+)
};

/* end of CONFIG_YOSEMITE */

#else

/* wedge */

//Under I2C Dev 1
static struct i2c_board_info __initdata ast_i2c_board_info_1[] = {
	{
		I2C_BOARD_INFO("cat9883", 0x4d),
	}
};


//Under I2C Dev 2
static struct i2c_board_info __initdata ast_i2c_board_info_2[] = {
	// Looks like ncp4200 i2c address could be floating depending
	// on the system. List all possibilities here (0x60 - 0x63).
	// Hope the address will not change after probing.
	{
		I2C_BOARD_INFO("ncp4200", 0x60),
	},
};


//Under I2C Dev 3
static struct i2c_board_info __initdata ast_i2c_board_info_3[] = {
	// Looks like ncp4200 i2c address could be floating depending
	// on the system. List all possibilities here (0x60 - 0x63)
	// Hope the address will not change after probing.
	{
		I2C_BOARD_INFO("ncp4200", 0x60),
	},
};


//Under I2C Dev 4
static struct i2c_board_info __initdata ast_i2c_board_info_4[] = {
	// Temperature sensors on Wedge:
	{
		I2C_BOARD_INFO("tmp75", 0x48),
	},
	{
		I2C_BOARD_INFO("tmp75", 0x49),
	},
	{
		I2C_BOARD_INFO("tmp75", 0x4a),
	},
};

//Under I2C Dev 5
static struct i2c_board_info __initdata ast_i2c_board_info_5[] = {
	/* Panther+ microserver */
	{
		I2C_BOARD_INFO("fb_panther_plus", 0x40),
	},
	// Temperature sensor on uServer:
	{
		I2C_BOARD_INFO("tmp75", 0x4c),
	},
	{
		I2C_BOARD_INFO("ads7828", 0x4b),
	},
	{
		I2C_BOARD_INFO("24c128", 0x51),
	},
};

//Under I2C Dev 7
static struct i2c_board_info __initdata ast_i2c_board_info_7[] = {
	// Wedge devices
	{
		I2C_BOARD_INFO("max127", 0x28),
	},
	{
		// Differs from the schematic, but appears to be correct
		I2C_BOARD_INFO("pcf8574", 0x3f),
	},
	{
		I2C_BOARD_INFO("24c64", 0x50),
	},
};


//Under I2C Dev 8
static struct i2c_board_info __initdata ast_i2c_board_info_8[] = {
	{
		// 6pack power supply
		I2C_BOARD_INFO("pfe3000", 0x10),
	},
	{
		// Eval board:
		I2C_BOARD_INFO("lm75b", 0x4a),
	},
	{
		// 6pack power supply EEPROM
		I2C_BOARD_INFO("24c64", 0x50),
	},
	// EEPROMS on the pfe1100 power supplies
	{
		I2C_BOARD_INFO("24c64", 0x51),
	},
	{
		I2C_BOARD_INFO("24c64", 0x52),
	},
	{
		I2C_BOARD_INFO("pfe1100", 0x59),
	},
	{
		I2C_BOARD_INFO("pfe1100", 0x5a),
	},
};


//Under I2C Dev 9
static struct i2c_board_info __initdata ast_i2c_board_info_9[] = {
	// Looks like ncp4200 i2c address could be floating depending
	// on the system. List all possibilities here (0x60 - 0x63)
	// Hope the address will not change after probing.
	{
		I2C_BOARD_INFO("ncp4200", 0x60),
	},
};

//Under I2C Dev 12
static struct i2c_board_info __initdata ast_i2c_board_info_12[] = {
	{
		// Early version of 6pack
		I2C_BOARD_INFO("pfe3000", 0x10),
	},
};

//Under I2C Dev 13
static struct i2c_board_info __initdata ast_i2c_board_info_13[] = {
	{
		I2C_BOARD_INFO("adm1278", 0x10),
	},
};

/* end of WEDGE case */
#endif

/* end of defined(CONFIG_ARCH_AST2300) || defined(CONFIG_ARCH_AST2400) */
#endif

/*-------------------------------------*/
void __init ast_add_device_i2c(void)
{
	//I2C Multi-Pin
	ast_scu_multi_func_i2c();

	//SCU I2C Reset
	ast_scu_init_i2c();

	ast_i2c_data.reg_gr = ioremap(AST_I2C_BASE, 4*SZ_16);
	if (!ast_i2c_data.reg_gr) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}

#if defined (CONFIG_ARCH_AST2400)
	ast_i2c_data.buf_pool= ioremap(AST_I2C_BASE+0x800, 2048);
	if (!ast_i2c_data.buf_pool) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}
#else
	ast_i2c_data.buf_pool = ioremap(AST_I2C_BASE+0x200, 256);
	if (!ast_i2c_data.buf_pool) {
		printk("ast_add_device_i2c ERROR \n");
		return;
	}
#endif

	pool_buff_page_init(ast_i2c_data.buf_pool);

#if defined(CONFIG_WEDGE100)

  platform_device_register(&ast_i2c_dev1_device);
  i2c_register_board_info(0, wedge100_i2c_bus1, ARRAY_SIZE(wedge100_i2c_bus1));

  platform_device_register(&ast_i2c_dev2_device);
  i2c_register_board_info(1, wedge100_i2c_bus2, ARRAY_SIZE(wedge100_i2c_bus2));

  platform_device_register(&ast_i2c_dev3_device);
  i2c_register_board_info(2, wedge100_i2c_bus3, ARRAY_SIZE(wedge100_i2c_bus3));

  platform_device_register(&ast_i2c_dev4_device);
  i2c_register_board_info(3, wedge100_i2c_bus4, ARRAY_SIZE(wedge100_i2c_bus4));

  platform_device_register(&ast_i2c_dev5_device);
  i2c_register_board_info(4, wedge100_i2c_bus5, ARRAY_SIZE(wedge100_i2c_bus5));

  platform_device_register(&ast_i2c_dev6_device);
  i2c_register_board_info(5, wedge100_i2c_bus6, ARRAY_SIZE(wedge100_i2c_bus6));

  platform_device_register(&ast_i2c_dev7_device);
  i2c_register_board_info(6, wedge100_i2c_bus7, ARRAY_SIZE(wedge100_i2c_bus7));

  platform_device_register(&ast_i2c_dev8_device);
  i2c_register_board_info(7, wedge100_i2c_bus8, ARRAY_SIZE(wedge100_i2c_bus8));

  platform_device_register(&ast_i2c_dev9_device);
  i2c_register_board_info(8, wedge100_i2c_bus9, ARRAY_SIZE(wedge100_i2c_bus9));

  /* i2c bus 10 - 12 are not used as i2c on wedge100 */

  platform_device_register(&ast_i2c_dev13_device);
  i2c_register_board_info(12, wedge100_i2c_bus13, ARRAY_SIZE(wedge100_i2c_bus13));

  /* i2c bug 14 is not used as i2c on wedge100 */

  /* end of defined(CONFIG_WEDGE100) */
#else

	platform_device_register(&ast_i2c_dev1_device);
	i2c_register_board_info(0, ast_i2c_board_info_1, ARRAY_SIZE(ast_i2c_board_info_1));
	platform_device_register(&ast_i2c_dev2_device);
	i2c_register_board_info(1, ast_i2c_board_info_2, ARRAY_SIZE(ast_i2c_board_info_2));
	platform_device_register(&ast_i2c_dev3_device);
	i2c_register_board_info(2, ast_i2c_board_info_3, ARRAY_SIZE(ast_i2c_board_info_3));
	platform_device_register(&ast_i2c_dev4_device);
	i2c_register_board_info(3, ast_i2c_board_info_4, ARRAY_SIZE(ast_i2c_board_info_4));
	platform_device_register(&ast_i2c_dev5_device);
	i2c_register_board_info(4, ast_i2c_board_info_5, ARRAY_SIZE(ast_i2c_board_info_5));
	platform_device_register(&ast_i2c_dev6_device);
#if defined(CONFIG_YOSEMITE)
	i2c_register_board_info(5, ast_i2c_board_info_6, ARRAY_SIZE(ast_i2c_board_info_6));
#endif
	platform_device_register(&ast_i2c_dev7_device);
	i2c_register_board_info(6, ast_i2c_board_info_7, ARRAY_SIZE(ast_i2c_board_info_7));
	platform_device_register(&ast_i2c_dev8_device);
	i2c_register_board_info(7, ast_i2c_board_info_8, ARRAY_SIZE(ast_i2c_board_info_8));
	platform_device_register(&ast_i2c_dev9_device);
	i2c_register_board_info(8, ast_i2c_board_info_9, ARRAY_SIZE(ast_i2c_board_info_9));

#if defined(CONFIG_ARCH_AST2400)
	platform_device_register(&ast_i2c_dev10_device);

#if defined(CONFIG_YOSEMITE)
	i2c_register_board_info(9, ast_i2c_board_info_10, ARRAY_SIZE(ast_i2c_board_info_10));
	platform_device_register(&ast_i2c_dev11_device);
	i2c_register_board_info(10, ast_i2c_board_info_11, ARRAY_SIZE(ast_i2c_board_info_11));
#endif

#if defined(CONFIG_MMC_AST)
	//Due to share pin with SD
#else
	/*
	 * On Wedge, bus 13 is used as i2c bus.  Bus 12 is used on other
	 * hardware.  Pins for bus 11, 12, and 14 are used as GPIOs, on
	 * various hardware, but enabling the i2c bus does not seem to
	 * interfere with the GPIOs.
	 */
	platform_device_register(&ast_i2c_dev12_device);
	i2c_register_board_info(11, ast_i2c_board_info_12, ARRAY_SIZE(ast_i2c_board_info_12));
	platform_device_register(&ast_i2c_dev13_device);
	i2c_register_board_info(12, ast_i2c_board_info_13, ARRAY_SIZE(ast_i2c_board_info_13));
#endif

  /* end of defined(CONFIG_ARCH_AST2400) */
#endif

  /* end of else of defined(CONFIG_WEDGE100) */
#endif
}
#else
void __init ast_add_device_i2c(void) {}
#endif
