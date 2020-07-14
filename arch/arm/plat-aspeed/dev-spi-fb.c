/*
 * SPI device definitions for Facebook devices
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
#include <linux/spi/eeprom.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>
#include <mach/ast_spi.h>

static int dual_flash_enabled = 0;

static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
	/* [0] ->15 : HCLK , HCLK/16 */
	u32 SPI_DIV[16] = {15, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
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
 * SPI0, SPI1: SPI Flash Controller
 */

static struct ast_spi_driver_data ast_fmc_driver_data = {
	.get_div = ast_spi_calculate_divisor,
	.num_chipselect = AST_FMC_CS_NUM,
};

static struct ast_spi_driver_data ast_spi0_driver_data = {
    .get_div = ast_spi_calculate_divisor,
#if defined CONFIG_AST_SPI0_CS1
    .num_chipselect = 2,
#else
    .num_chipselect = 1,
#endif
};

#if defined AST_SOC_G5
static struct ast_spi_driver_data ast_spi1_driver_data = {
    .get_div = ast_spi_calculate_divisor,
#if defined CONFIG_AST_SPI1_CS1
    .num_chipselect = 2,
#else
    .num_chipselect = 1,
#endif
};
#endif

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
#if defined CONFIG_AST_SPI0_CS1
  {
     .start  = AST_SPI0_CS1_BASE,
     .end    = AST_SPI0_CS1_BASE + SZ_16,
     .flags  = IORESOURCE_BUS,
	},
#endif
};

#if defined AST_SOC_G5
static struct resource ast_spi1_resource[] = {
  {
     .start  = AST_SPI1_BASE,
     .end    = AST_SPI1_BASE + SZ_16,
     .flags  = IORESOURCE_MEM,
  },
  {
     .start  = AST_SPI1_CS0_BASE,
     .end    = AST_SPI1_CS0_BASE + SZ_16,
     .flags  = IORESOURCE_BUS,
  },
#if defined CONFIG_AST_SPI1_CS1
  {
     .start  = AST_SPI1_CS1_BASE,
     .end    = AST_SPI1_CS1_BASE + SZ_16,
     .flags  = IORESOURCE_BUS,
  },
#endif
};
#endif

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
#if defined CONFIG_WEDGE100 || defined CONFIG_MINIPACK || defined CONFIG_WEDGE400
          .name           = "ast-spi",
          .id             = 1,
#else
           .name           = "fmc-spi",
           .id             = 1,
#endif
        .dev = {
          .platform_data = &ast_spi0_driver_data,
       },
      .num_resources  = ARRAY_SIZE(ast_spi0_resource),
      .resource       = ast_spi0_resource,
};

#if defined AST_SOC_G5
static struct platform_device ast_spi1_device = {
#if defined(CONFIG_MINILAKETB)
  .name           = "fmc-spi",
#else
  .name           = "ast-spi",
#endif
  .id             = 2,

  .dev = {
    .platform_data = &ast_spi1_driver_data,
  },
  .num_resources  = ARRAY_SIZE(ast_spi1_resource),
  .resource       = ast_spi1_resource,
};
#endif

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

static struct mtd_partition ast_secondary_partitions[] = {
	{
		.name       = "u-bootx",           /* Alt u-boot */
		.offset     = 0,                   /* From 0 */
		.size       = 0x60000,             /* Size 384K */
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name       = "envx",              /* Alt U-Boot NVRAM */
		.offset     = MTDPART_OFS_APPEND,  /* From 384K */
		.size       = 0x20000,             /* Size 128K, two sectors */
	}, {
		.name       = "fitx",              /* Alt kernel, rootfs */
		.offset     = MTDPART_OFS_APPEND,  /* From 896K */
		.size       = 0x1B80000,           /* Size 27.25M */
	}, {
		.name       = "datax",             /* Alt Data partition */
		.offset     = 0x1C00000,	   /* From 0x1C00000 */
		.size       = MTDPART_SIZ_FULL,    /* Full size */
	}, {
		.name       = "flash1",
		.offset     = 0,                   /* Writable alt-flash */
		.size       = MTDPART_SIZ_FULL,    /* full size */
	},
};

static struct mtd_partition ast_primary_partitions[] = {
	{
		.name       = "u-boot",            /* Primary u-boot */
		.offset     = 0,                   /* From 0 */
		.size       = 0x60000,             /* Size 384K */
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name       = "env",               /* U-Boot NVRAM */
		.offset     = MTDPART_OFS_APPEND,  /* From 384K */
		.size       = 0x20000,             /* Size 128K, two sectors */
	}, {
		.name       = "fit",               /* kernel, rootfs */
		.offset     = MTDPART_OFS_APPEND,  /* From 896K */
		.size       = 0x1B80000,           /* Size 27.25M */
	}, {
		.name       = "data0",
		.offset     = 0x1C00000,	   /* From 0x1C00000 */
		.size       = MTDPART_SIZ_FULL,
	}, {
		.name       = "flash0",
		.offset     = 0,                   /* From 0 */
		.size       = MTDPART_SIZ_FULL,    /* full size */
	},
};

static struct mtd_partition ast_rom_partitions[] = {
	{
		.name       = "rom",               /* ROM (SPL), recovery U-boot */
		.offset     = 0,                   /* From 0 */
		.size       = 0x60000,             /* Size 384K */
		.mask_flags = MTD_WRITEABLE,
	}, {
		.name       = "envro",             /* RO U-Boot NVRAM */
		.offset     = MTDPART_OFS_APPEND,  /* From 384K */
		.size       = 0x20000,             /* Size 128K, two sectors */
	}, {
		.name       = "u-bootro",          /* RO U-boot */
		.offset     = MTDPART_OFS_APPEND,  /* From 512K */
		.size       = 0x60000,             /* Size 384K */
	}, {
		.name       = "fitro",             /* RO kernel, rootfs */
		.offset     = MTDPART_OFS_APPEND,  /* From 896K */
		.size       = 0x1B20000,           /* Size 27.125M */
	}, {
		.name       = "dataro",            /* RO Data partition */
		.offset     = 0x1C00000,	   /* From 0x1C00000 */
		.size       = MTDPART_SIZ_FULL,    /* Full size */
	}, {
		.name       = "flash0",
		.offset     = 0,                   /* RO From 0 */
		.size       = MTDPART_SIZ_FULL,    /* full size */
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
		.offset     = 0x1C00000,	   /* From 0x1C00000 */
		.size       = MTDPART_SIZ_FULL,
	}, {
		.name       = "flash1",
		.offset     = 0,                   /* From 0 */
		.size       = MTDPART_SIZ_FULL,    /* full size */
	}, {
		.name       = "flash1rw",          /* Writable flash1 region */
		.offset     = 0x10000,
		.size       = MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition ast_spi_flash1_partitions[] = {
    {
#if defined(CONFIG_FBTP) || defined(CONFIG_PWNEPTUNE)
        .name       = "bios0",
#endif
        .offset     = 0,                  /* From 0 */
        .size       = MTDPART_SIZ_FULL,   /* full size */
    },
};

static struct mtd_partition ast_spi_flash2_partitions[] = {
    {
#if defined(CONFIG_MINILAKETB)
        .name       = "bios0",
        .offset     = 0,                  /* From 0 */
        .size       = MTDPART_SIZ_FULL,   /* full size */
#endif
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

/* The secondary boot platform data */
static struct flash_platform_data ast_secondary_platform_data = {
	.type       = "mx25l25635e",
	.nr_parts   = ARRAY_SIZE(ast_secondary_partitions),
	.parts      = ast_secondary_partitions,
};

/* The primary boot platform data */
static struct flash_platform_data ast_primary_platform_data = {
	.type       = "mx25l25635e",
	.nr_parts   = ARRAY_SIZE(ast_primary_partitions),
	.parts      = ast_primary_partitions,
};

/* The optional SPI0.0 (Flash1) SPI controller */
static struct flash_platform_data ast_spi_flash1_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_flash1_partitions),
    .parts      = ast_spi_flash1_partitions,
};

static struct flash_platform_data ast_spi_flash2_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_flash2_partitions),
    .parts      = ast_spi_flash2_partitions,
};

static struct spi_board_info ast_vboot_fmc_devices[] = {
	{
		.modalias 	= "m25p80",
		.platform_data 	= &ast_data_platform_data,
		.chip_select 	= 1,
		.max_speed_hz 	= 50 * 1000 * 1000,
		.bus_num 	= 0,
		.mode 		= SPI_MODE_0,
	},
	{
		.modalias 	= "m25p80",
		/* There are two potential layouts depending ROM availability. */
		.platform_data 	= &ast_rom_platform_data,
		.chip_select 	= 0,
		.max_speed_hz 	= 50 * 1000 * 1000,
		.bus_num 	= 0,
		.mode 		= SPI_MODE_0,
	},
};

static struct spi_board_info ast_dual_flash_fmc_devices[] = {
	{
		.modalias 	= "m25p80",
		.platform_data 	= &ast_primary_platform_data,
		.chip_select 	= 0,
		.max_speed_hz 	= 50 * 1000 * 1000,
		.bus_num 	= 0,
		.mode 		= SPI_MODE_0,
	},
	{
		.modalias 	= "m25p80",
		.platform_data 	= &ast_secondary_platform_data,
		.chip_select 	= 1,
		.max_speed_hz 	= 50 * 1000 * 1000,
		.bus_num 	= 0,
		.mode 		= SPI_MODE_0,
	},
};

static struct spi_board_info ast_single_flash_fmc_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  = &ast_legacy_platform_data,
		.chip_select    = 0,
		.max_speed_hz   = 50 * 1000 * 1000,
		.bus_num    		= 0,
		.mode 			    = SPI_MODE_0,
	},
};

#ifdef CONFIG_MINIPACK || CONFIG_WEDGE400
static struct spi_eeprom m95m02 = {
  .byte_len    = SZ_2M / 8,
  .name        = "m95m02",
  .page_size   = 256,
  .flags       = EE_ADDR3,
};
#endif

#if defined(CONFIG_FBTP)  || defined (CONFIG_WEDGE100) || defined (CONFIG_PWNEPTUNE) || \
    defined(CONFIG_MINIPACK) || defined (CONFIG_WEDGE400)
static struct spi_board_info ast_spi0_devices[] = {
    {
#if defined(CONFIG_WEDGE100) || defined(CONFIG_MINIPACK) || defined(CONFIG_WEDGE400)
        .modalias           = "spidev",
        .chip_select        = 0,
        .max_speed_hz       = 33 * 1000 * 1000,
        .bus_num            = 1,
#else
        .modalias           = "m25p80",
        .platform_data      = &ast_spi_flash1_data,
#if defined(CONFIG_FBTP) || defined(CONFIG_PWNEPTUNE)
        .chip_select        = 0,
        .max_speed_hz       = 50 * 1000 * 1000,
        .bus_num            = 1,
#endif
#endif
        .mode               = SPI_MODE_0,
    },
};
#endif

#if defined AST_SOC_G5
static struct spi_board_info ast_spi1_devices[] = {
#if defined CONFIG_MINIPACK || defined CONFIG_YAMP || defined CONFIG_WEDGE400
  {
    .modalias           = "spidev",
    .chip_select        = 0,
    .max_speed_hz       = 33 * 1000 * 1000,
    .bus_num            = 2,
    .mode               = SPI_MODE_0,
  },
#endif
#if defined CONFIG_MINIPACK || defined CONFIG_WEDGE400
  {
    .modalias           = "at25",
    .platform_data      = &m95m02,
    .chip_select        = 1,
    .max_speed_hz       = 5 * 1000 * 1000,
    .bus_num            = 2,
    .mode               = SPI_MODE_0,
  },
#endif
#if defined CONFIG_MINILAKETB
  {
    .modalias           = "m25p80",
    .platform_data      = &ast_spi_flash2_data,
    .chip_select        = 0,
    .max_speed_hz       = 50 * 1000 * 1000,
    .bus_num            = 2,
    .mode               = SPI_MODE_0,
  },
#endif
};
#endif


static int __init dual_flash_enabled_handler(char *str)
{
#if defined(CONFIG_FBTP) || defined(CONFIG_FBY2) || defined(CONFIG_FBTTN) || defined(CONFIG_FBY3_POC)
	/* HACK: Considering u-boot stores the boot-parameters in the
	* environment, there is a possibility that u-boot might
	* incorrectly store the flag as 1. This will happen if we upgrade
	* from an image before this commit to an image built with this
	* commit and just before rebooting, the env is modified. This causes
	* the fw_setenv tool to rebuild the env to the default (old boot
	* parameters before this commit). This is a workaround which avoids
	* that by assuming that for these platforms u-boot means 2 when it
	* says 1.
	* It is safe to assume that these platforms have verified boot enabled
	* and hence follow ROM based FIT layout */
	dual_flash_enabled = 2;
#else
	if (kstrtol(str, 10, &dual_flash_enabled)) {
		dual_flash_enabled = 0;
	}
#endif
  return 0;
}
early_param("dual_flash", dual_flash_enabled_handler);

void __init ast_add_device_spi(void)
{
	platform_device_register(&ast_fmc_device);

  /* Use kernel configuration from u-boot to enable dual
   * flash FMC configuration */
  if (dual_flash_enabled == 2) {
    spi_register_board_info(ast_vboot_fmc_devices, ARRAY_SIZE(ast_vboot_fmc_devices));
  } else if (dual_flash_enabled == 1) {
    spi_register_board_info(ast_dual_flash_fmc_devices, ARRAY_SIZE(ast_dual_flash_fmc_devices));
  } else {
    spi_register_board_info(ast_single_flash_fmc_devices, ARRAY_SIZE(ast_single_flash_fmc_devices));
  }
#if defined(CONFIG_FBTP) || defined(CONFIG_WEDGE100) || defined(CONFIG_PWNEPTUNE)
	platform_device_register(&ast_spi0_device);
	spi_register_board_info(ast_spi0_devices, ARRAY_SIZE(ast_spi0_devices));
#elif defined(CONFIG_MINIPACK) || defined(CONFIG_WEDGE400)
	platform_device_register(&ast_spi0_device);
	spi_register_board_info(ast_spi0_devices, ARRAY_SIZE(ast_spi0_devices));
	platform_device_register(&ast_spi1_device);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#elif defined(CONFIG_YAMP) || defined(CONFIG_MINILAKETB)
	platform_device_register(&ast_spi1_device);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#endif
}
