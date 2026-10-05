// SPDX-License-Identifier: GPL-2.0

#include <linux/mtd/spi-nor.h>

#include "core.h"

static void py25q01ghb_default_init(struct spi_nor *nor)
{
	/* PY25Q01GHB stores QE in status register 2 bit 1. */
	nor->params->quad_enable = spi_nor_sr2_bit1_quad_enable;
}

static struct spi_nor_fixups py25q01ghb_fixups = {
	.default_init = py25q01ghb_default_init,
};

static const struct flash_info puya_parts[] = {
	{ "py25q01ghb", INFO(0x85201b, 0, 64 * 1024, 2048,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_4B_OPCODES)
		.fixups = &py25q01ghb_fixups },
};

const struct spi_nor_manufacturer spi_nor_puya = {
	.name = "puya",
	.parts = puya_parts,
	.nparts = ARRAY_SIZE(puya_parts),
};
