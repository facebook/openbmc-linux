/********************************************************************************
* File Name     :  drivers/mtd/nand/aspeed_nand.c
* Author        : Ryan Chen
* Description   : ASPEED NAND driver
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
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

*  Overview:
*	This is a device driver for the NAND flash device found on the
*	ASPEED board which utilizes the Samsung K9F2808 part. This is
*	a 128Mibit (16MiB x 8 bits) NAND flash device.

*   History      :
*	1. 2012/10/20 Ryan Chen create this file
*
********************************************************************************/
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

struct ast_nand_data {
	struct nand_chip	chip;
	struct mtd_info		mtd;
	void __iomem		*io_base;
#ifdef CONFIG_MTD_PARTITIONS
	int			nr_parts;
	struct mtd_partition	*parts;
#endif
};

static struct nand_ecclayout aspeed_nand_hw_eccoob = {
	.eccbytes = 24,
	.eccpos = {40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63},
};

/* hardware specific access to control-lines  */
static void 
ast_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;

	if (cmd != NAND_CMD_NONE) {
		//writeb(cmd, chip->IO_ADDR_W + ((ctrl & 0x6) << 11));
		//user mode cmd addr[13:12]
		if(ctrl & NAND_CLE)
			writeb(cmd, chip->IO_ADDR_W + (0x1 << 12));
		else if (ctrl & NAND_ALE)
			writeb(cmd, chip->IO_ADDR_W + (0x2 << 12));
		else 
			writeb(cmd, chip->IO_ADDR_W + (1 << 12));
	}
}

/*
 * Main initialization routine
 */
static int __init 
ast_nand_probe(struct platform_device *pdev)
{
	struct ast_nand_data *data;
	struct platform_nand_data *pdata = pdev->dev.platform_data;
	int res = 0;

	/* Allocate memory for the device structure (and zero it) */
	data = kzalloc(sizeof(struct ast_nand_data), GFP_KERNEL);
	if (!data) {
			dev_err(&pdev->dev, "failed to allocate device structure.\n");
			return -ENOMEM;
	}

	data->io_base = ioremap(pdev->resource[0].start,
							pdev->resource[0].end - pdev->resource[0].start );
	if (data->io_base == NULL) {
			dev_err(&pdev->dev, "ioremap failed\n");
			kfree(data);
			return -EIO;
	}

	data->chip.priv = &data;
	data->mtd.priv = &data->chip;
	data->mtd.owner = THIS_MODULE;
	data->mtd.name = pdev->dev.bus_id;
	
	data->chip.IO_ADDR_R = data->io_base;
	data->chip.IO_ADDR_W = data->io_base;
	data->chip.cmd_ctrl = ast_hwcontrol;
	data->chip.dev_ready = pdata->ctrl.dev_ready;
	data->chip.select_chip = pdata->ctrl.select_chip;
	data->chip.chip_delay = pdata->chip.chip_delay;
	data->chip.options |= pdata->chip.options;
	
	data->chip.ecc.hwctl = pdata->ctrl.hwcontrol;
	data->chip.ecc.calculate = pdata->ctrl.calculate;
	data->chip.ecc.correct = pdata->ctrl.correct;
//	data->chip.ecc.layout = pdata->chip.ecclayout;
	data->chip.ecc.layout = &aspeed_nand_hw_eccoob;
	
	data->chip.ecc.bytes = 6;
	data->chip.ecc.size = 512;	
	data->chip.ecc.mode = NAND_ECC_HW; //NAND_ECC_SOFT;

	platform_set_drvdata(pdev, data);

	 /* Scan to find existance of the device */
	 if (nand_scan(&data->mtd, 1)) {
			 res = -ENXIO;
			 goto out;
	 }
#ifdef CONFIG_MTD_PARTITIONS
	if (pdata->chip.part_probe_types) {
			res = parse_mtd_partitions(&data->mtd,
									pdata->chip.part_probe_types,
									&data->parts, 0);
			if (res > 0) {
					add_mtd_partitions(&data->mtd, data->parts, res);
					return 0;
			}
	}
	if (pdata->chip.partitions) {
			data->parts = pdata->chip.partitions;
			res = add_mtd_partitions(&data->mtd, data->parts,
					pdata->chip.nr_partitions);
	} else
#endif
	res = add_mtd_device(&data->mtd);

	if (!res)
			return res;

	nand_release(&data->mtd);
out:
	platform_set_drvdata(pdev, NULL);
	iounmap(data->io_base);
	kfree(data);

	return res;

}

static int __devexit ast_nand_remove(struct platform_device *pdev)
{
	struct ast_nand_data *data = platform_get_drvdata(pdev);
#ifdef CONFIG_MTD_PARTITIONS
	struct platform_nand_data *pdata = pdev->dev.platform_data;
#endif

	nand_release(&data->mtd);
#ifdef CONFIG_MTD_PARTITIONS
	if (data->parts && data->parts != pdata->chip.partitions)
		kfree(data->parts);
#endif
	iounmap(data->io_base);
	kfree(data);

	return 0;
}

static struct platform_driver ast_nand_driver = {
        .probe          = ast_nand_probe,
        .remove         = ast_nand_remove,
        .driver         = {
                .name   = "ast-nand",
                .owner  = THIS_MODULE,
        },
};

static int __init ast_nand_init(void)
{
        return platform_driver_register(&ast_nand_driver);
}

static void __exit ast_nand_exit(void)
{
        platform_driver_unregister(&ast_nand_driver);
}

module_init(ast_nand_init);
module_exit(ast_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("NAND flash driver for ASPEED");
