/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-spi.c
* Author        : Ryan chen
* Description   : ASPEED SPI device
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
*    1. 2012/08/01 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>

#include <linux/spi/spi.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <asm/io.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>
#include <mach/ast_spi.h>

/* --------------------------------------------------------------------
 *   SPI Ctrl, (AST SPI + FMC SPI)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE) || defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
	// [0] ->15 : HCLK , HCLK/16
	u8 SPI_DIV[16] = {16, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	u32 i, hclk, spi_cdvr;

	hclk = ast_get_h_pll_clk();
	for(i=1;i<17;i++) {
		if(max_speed_hz >= (hclk/i)) {
			spi_cdvr = SPI_DIV[i-1];
			break;
		}
	}
		
//	printk("hclk is %d, divisor is %d, target :%d , cal speed %d\n", hclk, spi_cdvr, spi->max_speed_hz, hclk/i);
	return spi_cdvr;
}
#endif

#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE)
static struct ast_spi_driver_data fmc_spi_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = 1,
};

#if defined(CONFIG_AST_CS0_SPI)
static struct resource ast_spi_resource0[] = {
	{
		.start			= AST_FMC_BASE + 0x10,
		.end			= AST_FMC_BASE + 0x10 + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_CS0_DEF_BASE,
		.end			= AST_CS0_DEF_BASE + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};
static struct platform_device ast_spi_device0 = {
        .name           = "fmc-spi",
        .id             = 0,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource0),
        .resource       = ast_spi_resource0,        
};
#endif

#if defined(CONFIG_AST_CS1_SPI)
static struct resource ast_spi_resource1[] = {
	{
		.start			= AST_FMC_BASE + 0x14,
		.end			= AST_FMC_BASE + 0x14 + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_CS1_DEF_BASE,
		.end			= AST_CS1_DEF_BASE + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};
static struct platform_device ast_spi_device1 = {
        .name           = "fmc-spi",
        .id             = 1,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource1),
        .resource       = ast_spi_resource1,  		
};
#endif

#if defined(CONFIG_AST_CS2_SPI)
static struct resource ast_spi_resource2[] = {
	{
		.start			= AST_FMC_BASE + 0x18,
		.end			= AST_FMC_BASE + 0x18 + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_CS2_DEF_BASE,
		.end			= AST_CS2_DEF_BASE + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device ast_spi_device2 = {
        .name           = "fmc-spi",
        .id             = 2,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource2),
        .resource       = ast_spi_resource2,  		
};
#endif

#if defined(CONFIG_AST_CS3_SPI)
static struct resource ast_spi_resource3[] = {
	{
		.start			= AST_FMC_BASE + 0x1c,
		.end			= AST_FMC_BASE + 0x1c + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_CS3_DEF_BASE,
		.end			= AST_CS3_DEF_BASE + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device ast_spi_device3 = {
        .name           = "fmc-spi",
        .id             = 3,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource3),
        .resource       = ast_spi_resource3,  			
};
#endif

#if defined(CONFIG_AST_CS4_SPI)
static struct resource ast_spi_resource4[] = {
	{
		.start			= AST_FMC_BASE + 0x20,
		.end			= AST_FMC_BASE + 0x20 + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_CS4_DEF_BASE,
		.end			= AST_CS4_DEF_BASE + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device ast_spi_device4 = {
        .name           = "fmc-spi",
        .id             = 4,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource4),
        .resource       = ast_spi_resource4,  			
};
#endif

#endif	//CONFIG_SPI_FMC

#if defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
static struct ast_spi_driver_data ast_spi_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = 1,
};

static struct resource ast_spi1_resource[] = {
	{
		.start			= AST_SPI_BASE,
		.end			= AST_SPI_BASE + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_SPI_MEM,
		.end			= AST_SPI_MEM + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device ast_spi_device5 = {
        .name           = "ast-spi",
        .id             = 5,
		.dev = {
			.platform_data = &ast_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi1_resource),
        .resource       = ast_spi1_resource,  			
};
#endif	//CONFIG_SPI_AST

/* All partitions must be within 16M */
static struct mtd_partition ast_spi_flash_partitions[] = {
		{
			.name	= "u-boot",
			.offset = 0, /* From 0 */
			.size	= 0x60000, /* Size 384K */
			.mask_flags	= MTD_WRITEABLE,
                }, {
                        .name   = "env",
                        .offset = 0x60000, /* From 384K */
                        .size   = 0x20000, /* Size 128K, two sectors */
		}, {
		        .name   = "kernel",
			.offset = 0x80000,  /* From 512K */
            		.size   = 0x200000, /* Size 2M */
        }, {
            		.name   = "rootfs",
			.offset = 0x300000, /* From 3M */
            		.size   = 0xC00000, /* Size 12M */
		}, {
			.name	= "data0",
			.offset = MTDPART_OFS_APPEND,
			.size	= MTDPART_SIZ_FULL,
        },
};

static struct flash_platform_data ast_spi_flash_data = {
#ifdef CONFIG_ARCH_AST2400
		.type 		  = "mx25l25635e",	//AST2400 A1
#else		
		.type 		  = "mx25l12805d",	//old AST2300 
#endif		
        .nr_parts       = ARRAY_SIZE(ast_spi_flash_partitions),
        .parts          = ast_spi_flash_partitions,
};

#ifdef CONFIG_ARCH_AST2400
static struct flash_platform_data wedge_spi_flash_data = {
		.type 		  = "n25q128a13",
    .nr_parts       = ARRAY_SIZE(ast_spi_flash_partitions),
    .parts          = ast_spi_flash_partitions,
};
#endif

static struct spi_board_info ast_spi_devices[] = {
#if 0
    {
        .modalias    = "m25p80",
		.platform_data  = &ast_spi_flash_data,
        .chip_select    = 0, //.chip_select This tells your device driver which chipselect to use.
        .max_speed_hz    = 50 * 1000 * 1000, 
        .bus_num    = 0, //  This chooses if SPI0 or SPI1 of the SoC is used.
     	.mode = SPI_MODE_0,
    }, 
#endif
#ifdef CONFIG_ARCH_AST2400
    {
        .modalias    = "m25p80",
        .platform_data  = &wedge_spi_flash_data,
        .chip_select    = 0, //.chip_select This tells your device driver which chipselect to use.
        .max_speed_hz    = 50 * 1000 * 1000,
        .bus_num    = 0, //  This chooses if SPI0 or SPI1 of the SoC is used.
        .mode = SPI_MODE_0,
    },
#endif
    {
        .modalias    = "spidev",
        .chip_select    = 0,
        .max_speed_hz    = 30 * 1000 * 1000,
        .bus_num    = 1,
		.mode = SPI_MODE_0,
    },
};

#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE) || defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)

/*-------------------------------------*/
void __init ast_add_device_spi(void)
{
#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE)
	void __iomem *fmc_regs = ioremap(AST_FMC_BASE, 4*SZ_16);
	u32 tmp = 0;

#if defined(CONFIG_AST_CS0_SPI)
	platform_device_register(&ast_spi_device0);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(0)) & FMC_MASK_TYPE_CS(0);
	writel( tmp | FMC_SET_TYPE_SPI_CS(0), fmc_regs);
#endif

#if defined(CONFIG_AST_CS1_SPI)
	ast_scu_multi_func_romcs(1);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(1)) & FMC_MASK_TYPE_CS(1);
	writel( tmp | FMC_SET_TYPE_SPI_CS(1), fmc_regs);
	platform_device_register(&ast_spi_device1);
#endif

#if defined(CONFIG_AST_CS2_SPI)
	ast_scu_multi_func_romcs(2);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(2)) & FMC_MASK_TYPE_CS(2);
	writel( tmp | FMC_SET_TYPE_SPI_CS(2), fmc_regs);
	platform_device_register(&ast_spi_device2);
#endif
#if defined(CONFIG_AST_CS3_SPI)
	ast_scu_multi_func_romcs(3);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(3)) & FMC_MASK_TYPE_CS(3);
	writel( tmp | FMC_SET_TYPE_SPI_CS(3), fmc_regs);
	platform_device_register(&ast_spi_device3);
#endif
#if defined(CONFIG_AST_CS4_SPI)
	ast_scu_multi_func_romcs(4);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(4)) & FMC_MASK_TYPE_CS(4);
	writel( tmp | FMC_SET_TYPE_SPI_CS(4), fmc_regs);
	platform_device_register(&ast_spi_device4);
#endif

	iounmap(fmc_regs);

#endif
#if defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
	//pin switch by trap[13:12]
	platform_device_register(&ast_spi_device5);
#endif

	spi_register_board_info(ast_spi_devices, ARRAY_SIZE(ast_spi_devices));

}
#else
void __init ast_add_device_spi(void) {}
#endif
