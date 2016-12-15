/*
 * SPI device definitions for CMM
 *
 * Copyright 2015-present Facebook. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>
#include <mach/ast_spi.h>

static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
	/* [0] ->15 : HCLK , HCLK/16 */
	u32 SPI_DIV[16] = {16, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	u32 i, hclk, spi_cdvr=0;

	hclk = ast_get_ahbclk();
	for(i=1;i<17;i++) {
		if(max_speed_hz >= (hclk/i)) {
			spi_cdvr = SPI_DIV[i-1];
			break;
		}
	}

	return spi_cdvr;
}

/*
 * FMC: Firmware SPI Memory Controller
 * SPI0: SPI Flash Controller
 */

static struct ast_spi_driver_data ast_fmc_driver_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = AST_FMC_CS_NUM,
};

static struct ast_spi_driver_data ast_spi0_driver_data = {
    .get_div = ast_spi_calculate_divisor,
    .num_chipselect = 1,
};

static struct resource ast_fmc_resource[] = {
	{
		.start  = AST_FMC_BASE,
		.end    = AST_FMC_BASE + SZ_16,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = AST_FMC_CS0_BASE,
		.end    = AST_FMC_CS0_BASE + SZ_16,
		.flags  = IORESOURCE_BUS,
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

static struct resource ast_spi0_resource[] = {
  {
     .start  = AST_SPI0_BASE,
     .end    = AST_SPI0_BASE + SZ_16,
     .flags  = IORESOURCE_MEM,
  },
  {
     .start  = AST_SPI0_CS0_BASE,
     .end    = AST_SPI0_CS0_BASE + SZ_16,
     .flags  = IORESOURCE_BUS,
  },
};

static struct platform_device ast_fmc_device = {
	.name           = "fmc-spi",
	.id             = 0,
	.dev = {
		.platform_data = &ast_fmc_driver_data,
	},
	.num_resources  = ARRAY_SIZE(ast_fmc_resource),
	.resource       = ast_fmc_resource,
};

static struct platform_device ast_spi0_device = {
        .name           = "fmc-spi",
        .id             = 1,
        .dev = {
          .platform_data = &ast_spi0_driver_data,
       },
      .num_resources  = ARRAY_SIZE(ast_spi0_resource),
      .resource       = ast_spi0_resource,
};

static struct mtd_partition ast_legacy_partitions[] = {
	{
		.name       = "u-boot",
		.offset     = 0,        /* From 0 */
		.size       = 0x60000,  /* Size 384K */
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name       = "env",
		.offset     = MTDPART_OFS_APPEND,  /* From 384K */
		.size       = 0x20000,  /* Size 128K, two sectors */
	}, {
		.name       = "kernel",
		.offset     = MTDPART_OFS_APPEND,  /* From 512K */
		.size       = 0x400000, /* Size 4M */
	}, {
		.name       = "rootfs",
		.offset     = MTDPART_OFS_APPEND,  /* From 4.5M */
		.size       = 0x1780000, /* Size 23.5M */
	}, {
		.name       = "data0",
		.offset     = MTDPART_OFS_APPEND,
		.size       = MTDPART_SIZ_FULL,
	},
	{
		.name       = "flash0",
		.offset     = 0,        /* From 0 */
		.size       = MTDPART_SIZ_FULL,  /* full size */
	},
};

static struct mtd_partition ast_rom_partitions[] = {
	{
		.name       = "rom",               /* ROM (SPL), recovery U-boot */
		.offset     = 0,                   /* From 0 */
		.size       = MTDPART_SIZ_FULL,    /* Full size */
		.mask_flags = MTD_WRITEABLE,
	},
};

static struct mtd_partition ast_data_partitions[] = {
	{
		.name       = "romx",              /* (unused) */
		.offset     = 0,                   /* From 0 */
		.size       = 0x60000,             /* Size 384K */
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name       = "env",               /* U-Boot NVRAM */
		.offset     = MTDPART_OFS_APPEND,  /* From 384K */
		.size       = 0x20000,             /* Size 128K, two sectors */
	}, {
		.name       = "u-boot",            /* Signed: U-boot, intermediate keys */
		.offset     = MTDPART_OFS_APPEND,  /* From 512K */
		.size       = 0x60000,             /* Size 384K */
	}, {
		.name       = "fit",               /* Signed: kernel, rootfs */
		.offset     = MTDPART_OFS_APPEND,  /* From 896K */
		.size       = 0x1B20000,           /* Size 27.125M */
	}, {
		.name       = "data0",
		.offset     = MTDPART_OFS_APPEND,
		.size       = MTDPART_SIZ_FULL,
	},
	{
		.name       = "flash0",
		.offset     = 0,                   /* From 0 */
		.size       = MTDPART_SIZ_FULL,    /* full size */
	},
};

static struct mtd_partition ast_spi_flash1_partitions[] = {
    {
        .name       = "bios0",
        .offset     = 0,                  /* From 0 */
        .size       = MTDPART_SIZ_FULL,   /* full size */
    },
};

/* The legacy platform data is the non-FIT, non-ROM layout */
static struct flash_platform_data ast_legacy_platform_data = {
	.type       = "mx25l25635e",
	.nr_parts   = ARRAY_SIZE(ast_legacy_partitions),
	.parts      = ast_legacy_partitions,
};

/* The ROM platform data is FMC.0 (CS0) */
static struct flash_platform_data ast_rom_platform_data = {
	.type       = "mx25l25635e",
	.nr_parts   = ARRAY_SIZE(ast_rom_partitions),
	.parts      = ast_rom_partitions,
};

/* The data is FMC.1 (CS1) */
static struct flash_platform_data ast_data_platform_data = {
	.type       = "mx25l25635e",
	.nr_parts   = ARRAY_SIZE(ast_data_partitions),
	.parts      = ast_data_partitions,
};

/* The optional SPI0.0 (Flash1) SPI controller */
static struct flash_platform_data ast_spi_flash1_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_flash1_partitions),
    .parts      = ast_spi_flash1_partitions,
};

static struct spi_board_info ast_fmc_devices[] = {
#ifdef CONFIG_AST_DUAL_FLASH
	{
		.modalias    		= "m25p80",
		.platform_data  = &ast_data_platform_data,
		.chip_select    = 1,
		.max_speed_hz   = 50 * 1000 * 1000,
		.bus_num    		= 0,
		.mode 			    = SPI_MODE_0,
	},
#endif
	{
		.modalias    		= "m25p80",
		/* There are two potential layouts depending ROM availability. */
#ifdef CONFIG_AST_DUAL_FLASH
		.platform_data  = &ast_rom_platform_data,
#else
		.platform_data  = &ast_legacy_platform_data,
#endif
		.chip_select    = 0,
		.max_speed_hz   = 50 * 1000 * 1000,
		.bus_num    		= 0,
		.mode 			    = SPI_MODE_0,
	},

};

#if defined(CONFIG_FBTP)
static struct spi_board_info ast_spi0_devices[] = {
    {
        .modalias           = "m25p80",
        .platform_data      = &ast_spi_flash1_data,
        .chip_select        = 0,
        .max_speed_hz       = 50 * 1000 * 1000,
        .bus_num            = 1,
        .mode               = SPI_MODE_0,
    },
};
#endif

void __init ast_add_device_spi(void)
{
	platform_device_register(&ast_fmc_device);
	spi_register_board_info(ast_fmc_devices, ARRAY_SIZE(ast_fmc_devices));
#if defined(CONFIG_FBTP)
	platform_device_register(&ast_spi0_device);
	spi_register_board_info(ast_spi0_devices, ARRAY_SIZE(ast_spi0_devices));
#endif
}
