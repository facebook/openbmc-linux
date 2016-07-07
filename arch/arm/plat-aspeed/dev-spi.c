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
#include <asm/io.h>

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>
#include <mach/ast_spi.h>
#endif

/* --------------------------------------------------------------------
 *   SPI Ctrl, (AST SPI + FMC SPI)
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE) || defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
	// [0] ->15 : HCLK , HCLK/16
	u8 SPI_DIV[16] = {16, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	u32 i, hclk, spi_cdvr=0;

	hclk = ast_get_ahbclk();
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
	.num_chipselect = AST_FMC_CS_NUM,
};

static struct resource ast_fmc_spi_resource0[] = {
	{
		.start	= AST_FMC_BASE,
		.end		= AST_FMC_BASE + SZ_16,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AST_FMC_CS0_BASE,
		.end		= AST_FMC_CS0_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},	
	{
		.start	= AST_FMC_CS1_BASE,
		.end		= AST_FMC_CS1_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},		
	{
		.start	= AST_FMC_CS2_BASE,
		.end		= AST_FMC_CS2_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},
#ifdef AST_FMC_CS3_BASE
	{
		.start	= AST_FMC_CS3_BASE,
		.end		= AST_FMC_CS3_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},
#endif
#ifdef AST_FMC_CS4_BASE
	{
		.start	= AST_FMC_CS4_BASE,
		.end		= AST_FMC_CS4_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},
#endif
};

static struct platform_device ast_fmc0_spi_device = {
        .name           = "fmc-spi",
        .id             = 0,
		.dev = {
			.platform_data = &fmc_spi_data,
		},
        .num_resources  = ARRAY_SIZE(ast_fmc_spi_resource0),
        .resource       = ast_fmc_spi_resource0,        
};

static struct ast_spi_driver_data fmc_spix_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = AST_FMC_SPIx_CS_NUM,
};

#ifdef AST_FMC_SPI0_BASE
static struct resource ast_fmc_spi0_resource[] = {
	{
		.start	= AST_FMC_SPI0_BASE,
		.end		= AST_FMC_SPI0_BASE + SZ_16,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AST_SPI0_CS0_BASE,
		.end		= AST_SPI0_CS0_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},	
	{
		.start	= AST_SPI0_CS1_BASE,
		.end		= AST_SPI0_CS1_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},		
};

static struct platform_device ast_fmc_spi0_device = {
        .name           = "fmc-spi",
        .id             = 1,
		.dev = {
			.platform_data = &fmc_spix_data,
		},
        .num_resources  = ARRAY_SIZE(ast_fmc_spi0_resource),
        .resource       = ast_fmc_spi0_resource,        
};
#endif 

#ifdef AST_FMC_SPI1_BASE
static struct resource ast_fmc_spi1_resource[] = {
	{
		.start	= AST_FMC_SPI1_BASE,
		.end		= AST_FMC_SPI1_BASE + SZ_16,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AST_SPI1_CS0_BASE,
		.end		= AST_SPI1_CS0_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},	
	{
		.start	= AST_SPI1_CS1_BASE,
		.end		= AST_SPI1_CS1_BASE + SZ_16,
		.flags	= IORESOURCE_BUS,
	},		
};

static struct platform_device ast_fmc_spi1_device = {
        .name           = "fmc-spi",
        .id             = 2,
		.dev = {
			.platform_data = &fmc_spix_data,
		},
        .num_resources  = ARRAY_SIZE(ast_fmc_spi1_resource),
        .resource       = ast_fmc_spi1_resource,        
};
#endif 

#endif	//CONFIG_SPI_FMC

#if defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
static struct ast_spi_driver_data ast_spi0_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = 1,
};

static struct resource ast_spi_resource0[] = {
	{
		.start			= AST_SPI0_BASE,
		.end				= AST_SPI0_BASE + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_SPI0_MEM + 0x04,
		.end				= AST_SPI0_MEM + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device ast_spi_device0 = {
        .name           = "ast-spi",
#if defined(CONFIG_ARCH_AST1010)		
		.id 		= 0,
#else
		.id		= 1,
#endif
		.dev = {
			.platform_data = &ast_spi0_data,
		},
        .num_resources  = ARRAY_SIZE(ast_spi_resource0),
        .resource       = ast_spi_resource0,  			
};

#if defined(AST_SPI1_BASE)
static struct ast_spi_driver_data ast_spi1_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = 1,
};

static struct resource aspeed_spi1_resource[] = {
	{
		.start			= AST_SPI1_BASE,
		.end			= AST_SPI1_BASE + SZ_16,
		.flags			= IORESOURCE_MEM,
	},
	{
		.start			= AST_SPI1_MEM,
		.end			= AST_SPI1_MEM + SZ_16,
		.flags			= IORESOURCE_IO,
	},	
};

static struct platform_device aspeed_spi_device1 = {
        .name           = "ast-spi",
        .id             = 1,
		.dev = {
			.platform_data = &ast_spi1_data,
		},
        .num_resources  = ARRAY_SIZE(aspeed_spi1_resource),
        .resource       = aspeed_spi1_resource,  			
};

#endif

#endif	//CONFIG_SPI_AST

static struct mtd_partition ast_spi_flash0_partitions[] = {
	{
		.name	= "u-boot",
		.offset 	= 0,
		.size		= 0x80000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name	= "env",
		.offset 	= 0x60000,
		.size		= 0x20000,            
	}, {
		.name   	= "kernel",
		.offset 	= 0x80000,
		.size   	= 0x280000,     
	}, {
		.name   	= "rootfs",
		.offset 	= 0x400000,
		.size		= 0x400000,
	}, {
		.name	= "pd_rootfs",
		.offset 	= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

#ifdef CONFIG_ARCH_AST1010
static struct mtd_partition ast_spi_flash1_partitions[] = {
	{
		.name	= "uboot",
		.offset = 0,
		.size	= 0x00030000,
		.mask_flags = MTD_WRITEABLE,				
	},
	{
		.name	= "uboot-env",
		.offset = MTDPART_OFS_APPEND,
		.size	= 0x00010000,
//		.mask_flags = MTD_WRITEABLE,
	},
	{
		.name   = "uCLinux",
		.offset = MTDPART_OFS_APPEND,
		.size   = 0x003c0000,
//		.mask_flags = MTD_CAP_NORFLASH,
	},
	{
		.name	= "data",
		.offset = MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
//		.mask_flags = MTD_CAP_NORFLASH,
	}
};
#else
static struct mtd_partition ast_spi_flash1_partitions[] = {
	{
		.name	= "bios",
		.offset 	= 0,
		.size		= MTDPART_SIZ_FULL,
	},
};
#endif

static struct flash_platform_data ast_spi_flash0_data = {
#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
	.type 		  = "mx25l25635e",	//AST2400 A1
#elif defined(CONFIG_ARCH_AST3200)
	.type 		  = "w25q256",
#else		
	.type 		  = "mx25l12805d",	//old AST2300 
#endif		
	.nr_parts       = ARRAY_SIZE(ast_spi_flash0_partitions),
	.parts          = ast_spi_flash0_partitions,
};

static struct flash_platform_data ast_spi_flash1_data = {
//	.type         	= "mx25l2005a",
	.type         	= "mx25l6405d",
	.nr_parts       	= ARRAY_SIZE(ast_spi_flash1_partitions),
	.parts          	= ast_spi_flash1_partitions,
};

#ifdef CONFIG_ARCH_AST1010
static struct spi_board_info ast_spi_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  	= &ast_spi_flash1_data,
		.chip_select    		= 0, //.chip_select This tells your device driver which chipselect to use.
		.max_speed_hz    	= 50 * 1000 * 1000, 
		.bus_num    		= 0, //  This chooses if SPI0 or SPI1 of the SoC is used.
		.mode 			= SPI_MODE_0,
	}, {
		.modalias    		= "spidev",
		.chip_select    		= 0,
		.max_speed_hz    	= 30 * 1000 * 1000,
		.bus_num    		= 1,
		.mode 			= SPI_MODE_0,
	},
};
#else
static struct spi_board_info ast_spi_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  	= &ast_spi_flash0_data,
		.chip_select    		= 0, //.chip_select This tells your device driver which chipselect to use.
		.max_speed_hz    	= 100 * 1000 * 1000, 
		.bus_num    		= 0, //  This chooses if SPI0 or SPI1 of the SoC is used.
		.mode 			= SPI_MODE_0,
#if 0		
	}, {
		.modalias    		= "spidev",
		.chip_select    		= 0,
		.max_speed_hz    	= 30 * 1000 * 1000,
		.bus_num    		= 0,
		.mode 			= SPI_MODE_0,
#endif		
	},
};

static struct spi_board_info ast_spi1_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  	= &ast_spi_flash1_data,
		.chip_select    		= 0, //.chip_select This tells your device driver which chipselect to use.
		.max_speed_hz    	= 50 * 1000 * 1000, 
		.bus_num    		= 1, //  This chooses if SPI0 or SPI1 of the SoC is used.
		.mode 			= SPI_MODE_0,
	},
};

#endif

#if defined(AST_SPI1_BASE)
static struct mtd_partition ast_spi_flash1_partitions[] = {
		{
			.name	= "bios",
			.offset = 0,
			.size	= MTDPART_SIZ_FULL,
        }
};

static struct flash_platform_data ast_spi_flash1_data = {
//        .type           = "w25q64",	
	.type           = "mx25l6405d",
        .nr_parts       = ARRAY_SIZE(ast_spi_flash1_partitions),
        .parts          = ast_spi_flash1_partitions,
};


static struct spi_board_info ast_spi1_devices[] = {
    {
        .modalias    = "m25p80",
	.platform_data  = &ast_spi_flash1_data,
        .chip_select    = 0, //.chip_select This tells your device driver which chipselect to use.
        .max_speed_hz    = 100 * 1000 * 1000, 
        .bus_num    = 1, //  This chooses if SPI0 or SPI1 of the SoC is used.
     	.mode = SPI_MODE_0,
    }, 
};
#endif

#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE) || defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)

/*-------------------------------------*/
void __init ast_add_device_spi(void)
{
#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE)
	platform_device_register(&ast_fmc0_spi_device);
	spi_register_board_info(ast_spi_devices, ARRAY_SIZE(ast_spi_devices));
#endif

//	platform_device_register(&ast_fmc_spi0_device);
//	platform_device_register(&ast_fmc_spi1_device);
//	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));

#if defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
	//pin switch by trap[13:12]	-- [0:1] Enable SPI Master 
	platform_device_register(&ast_spi_device0);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#endif

#if defined(AST_SPI1_BASE)
	//AST1010 SCU CONFIG TODO .......
	writel(readl(AST_SCU_BASE + 0x70) | 0x10,AST_SCU_BASE + 0x70);
	platform_device_register(&aspeed_spi_device1);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#endif	

}
#else
void __init ast_add_device_spi(void) {}
#endif


