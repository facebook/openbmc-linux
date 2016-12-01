/*
 * SPI device definitions for FBTP
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

#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE)

static u32 ast_spi_calculate_divisor(u32 max_speed_hz)
{
    // [0] ->15 : HCLK , HCLK/16
    u32 SPI_DIV[16] = {16, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
    u32 i, hclk, spi_cdvr=0;

    hclk = ast_get_ahbclk();
    for(i=1;i<17;i++) {
        if(max_speed_hz >= (hclk/i)) {
            spi_cdvr = SPI_DIV[i-1];
            break;
        }
    }

//  printk("hclk is %d, divisor is %d, target :%d , cal speed %d\n",
//         hclk, spi_cdvr, spi->max_speed_hz, hclk/i);
    return spi_cdvr;
}

static struct ast_spi_driver_data fmc_spi_data = {
    .get_div = ast_spi_calculate_divisor,
    .num_chipselect = AST_FMC_CS_NUM,
};

static struct resource ast_fmc_spi_resource0[] = {
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
        .start  = AST_FMC_CS1_BASE,
        .end    = AST_FMC_CS1_BASE + SZ_16,
        .flags  = IORESOURCE_BUS,
    },
    {
        .start  = AST_FMC_CS2_BASE,
        .end    = AST_FMC_CS2_BASE + SZ_16,
        .flags  = IORESOURCE_BUS,
    },
};

static struct ast_spi_driver_data ast_spi0_data = {
    .get_div = ast_spi_calculate_divisor,
    .num_chipselect = 1,
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
static struct platform_device ast_spi0_device = {
        .name           = "fmc-spi",
        .id             = 1,
        .dev = {
          .platform_data = &ast_spi0_data,
       },
      .num_resources  = ARRAY_SIZE(ast_spi0_resource),
      .resource       = ast_spi0_resource,
};

static struct mtd_partition ast_spi_rom_partitions[] = {
    {
        .name       = "rom",
        .offset     = 0,        /* From 0 */
        .size       = MTDPART_SIZ_FULL,  /* full size */
    },
};

static struct flash_platform_data ast_spi_rom_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_rom_partitions),
    .parts      = ast_spi_rom_partitions,
};

static struct mtd_partition ast_spi_flash0_partitions[] = {
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

static struct flash_platform_data ast_spi_flash0_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_flash0_partitions),
    .parts      = ast_spi_flash0_partitions,
};

static struct mtd_partition ast_spi_flash1_partitions[] = {
    {
        .name       = "BIOS0",
        .offset     = 0,        /* From 0 */
        .size       = MTDPART_SIZ_FULL,  /* full size */
    },
};

static struct flash_platform_data ast_spi_flash1_data = {
    .type       = "mx25l25635e",
    .nr_parts   = ARRAY_SIZE(ast_spi_flash1_partitions),
    .parts      = ast_spi_flash1_partitions,
};

static struct spi_board_info ast_spi_devices[] = {
#if defined(CONFIG_SPI_FMC_ROM)
    /* Here the 'reverse' order allows for tooling backward-compatibility. */
    {
        .modalias           = "m25p80",
        .platform_data      = &ast_spi_flash0_data,
        .chip_select        = 1,
        .max_speed_hz       = 50 * 1000 * 1000,
        .bus_num            = 0,
        .mode               = SPI_MODE_0,
    },
#endif
    {
        .modalias           = "m25p80",
        /* There are two potential layouts depending ROM availability. */
#if defined(CONFIG_SPI_FMC_ROM)
        .platform_data      = &ast_spi_rom_data,
#else
        .platform_data      = &ast_spi_flash0_data,
#endif
        .chip_select        = 0,
        .max_speed_hz       = 50 * 1000 * 1000,
        .bus_num            = 0,
        .mode               = SPI_MODE_0,
    },
};

static struct spi_board_info ast_spi1_devices[] = {
    {
        .modalias           = "m25p80",
        .platform_data      = &ast_spi_flash1_data,
        .chip_select        = 0,
        .max_speed_hz       = 50 * 1000 * 1000,
        .bus_num            = 1,
        .mode               = SPI_MODE_0,
    },
};
void __init ast_add_device_spi(void)
{
    platform_device_register(&ast_fmc0_spi_device);
    spi_register_board_info(ast_spi_devices, ARRAY_SIZE(ast_spi_devices));
    platform_device_register(&ast_spi0_device);
    spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
}

#else

void __init ast_add_device_spi(void) {}

#endif
