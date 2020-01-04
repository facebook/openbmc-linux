// SPDX-License-Identifier: GPL-2.0
/*
 * ASPEED SPI Controller Driver
 *
 * Copyright 2019-present Facebook. All Rights Reserved.
 *
 * Borrowed some code from drivers/mtd/spi-nor/aspeed-smc.c.
 *
 * Based on Ryan Chen's work in 2012:
 * Copyright (C) 2012-2020 ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#define ASPEED_SPI_DRIVER		"aspeed-spi"
#define ASPEED_SPI_CS_NUM		2
#define ASPEED_SUPP_MODES		(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

/*
 * Register definitions.
 */
#define ASPEED_CFG_REG			0x00
#define ASPEED_CE_CTRL_REG		0x04
#define ASPEED_CTRL_REG_CE0		0x10
#define ASPEED_CTRL_REG_CE1		0x14
#define ASPEED_ADDR_RANGE_REG_CE0	0x30
#define ASPEED_ADDR_RANGE_REG_CE1	0x34

/*
 * Fields in SPI Flash Configuration Register SPIR00.
 */
#define ASPEED_CFG_ENABLE_WR_CE0	BIT(16)
#define ASPEED_CFG_ENABLE_WR_CE1	BIT(17)

/*
 * Fields in SPI CE Control Register SPIR04.
 */
#define ASPEED_CTRL_DIV2_TIMING_CE0	BIT(8)
#define ASPEED_CTRL_DIV2_TIMING_CE1	BIT(9)

/*
 * Fields in CE# Control register SPIR10/SPIR14.
 */
#define ASPEED_CTRL_NORMAL_RD_MODE	0
#define ASPEED_CTRL_RD_CMD_MODE		1
#define ASPEED_CTRL_WR_CMD_MODE		2
#define ASPEED_CTRL_USER_MODE		3
#define ASPEED_CTRL_STOP_ACTIVE		BIT(2)
#define ASPEED_CTRL_CLK_MODE_3		BIT(4)
#define ASPEED_CTRL_CLK_DIV4_MODE	BIT(13)
#define ASPEED_CTRL_CLK_DIV_MAX		16
#define ASPEED_CTRL_CLK_DIV_MASK	(0xF << 8)
#define ASPEED_CTRL_CLK_DIV(d)		(((d) & 0xF) << 8)

/*
 * The Segment Register uses a 8MB unit to encode the start address
 * and the end address of the mapping window of a flash SPI slave :
 *
 *        | byte 1 | byte 2 | byte 3 | byte 4 |
 *        +--------+--------+--------+--------+
 *        |  end   |  start |   0    |   0    |
 */
#define SEGMENT_ADDR_START(_v)		((((_v) >> 16) & 0xFF) << 23)
#define SEGMENT_ADDR_END(_v)		((((_v) >> 24) & 0xFF) << 23)

struct aspeed_spi_priv {
	void __iomem *reg_base;
	struct spi_master *master;
	struct device *dev;
	unsigned long ahb_clk_freq;
	spinlock_t lock;

	/*
	 * Slave device addresses.
	 */
	void __iomem *slave_base;
	u32 slave_mem_size;
	struct {
		void __iomem *start;
		u32 size;
	} slave_buf[ASPEED_SPI_CS_NUM];
};

static struct {
	u32 ctrl_reg;
	u32 addr_range_reg;
	u32 enable_write;
	u32 div2_mode;
} cs_reg_map[ASPEED_SPI_CS_NUM] = {
	[0] = {	/* chip select #0 */
		.ctrl_reg = ASPEED_CTRL_REG_CE0,
		.addr_range_reg = ASPEED_ADDR_RANGE_REG_CE0,
		.enable_write = ASPEED_CFG_ENABLE_WR_CE0,
		.div2_mode = ASPEED_CTRL_DIV2_TIMING_CE0,
	},
	[1] = {	/* chip select #1 */
		.ctrl_reg = ASPEED_CTRL_REG_CE1,
		.addr_range_reg = ASPEED_ADDR_RANGE_REG_CE1,
		.enable_write = ASPEED_CFG_ENABLE_WR_CE1,
		.div2_mode = ASPEED_CTRL_DIV2_TIMING_CE1,
	},
};

static void aspeed_reg_write(struct aspeed_spi_priv *priv, u32 val, u32 reg)
{
	writel(val, priv->reg_base + reg);
}

static u32 aspeed_reg_read(struct aspeed_spi_priv *priv, u32 reg)
{
	return readl(priv->reg_base + reg);
}

static void aspeed_activate_cs(struct aspeed_spi_priv *priv, u8 cs)
{
	u32 ctrl_reg = cs_reg_map[cs].ctrl_reg;
	u32 val = aspeed_reg_read(priv, ctrl_reg);

	val &= ~ASPEED_CTRL_STOP_ACTIVE;
	aspeed_reg_write(priv, val, ctrl_reg);
}

static void aspeed_deactivate_cs(struct aspeed_spi_priv *priv, u8 cs)
{
	u32 ctrl_reg = cs_reg_map[cs].ctrl_reg;
	u32 val = aspeed_reg_read(priv, ctrl_reg);

	val |= ASPEED_CTRL_STOP_ACTIVE;
	aspeed_reg_write(priv, val, ctrl_reg);
}

static bool aspeed_check_set_div2(struct aspeed_spi_priv *priv,
				  u8 cs, u32 spi_max_freq)
{
	u32 div, val;

	div = priv->ahb_clk_freq / spi_max_freq;
	if (div <= ASPEED_CTRL_CLK_DIV_MAX)
		return false;

	val = aspeed_reg_read(priv, ASPEED_CE_CTRL_REG);
	val |= cs_reg_map[cs].div2_mode;
	aspeed_reg_write(priv, val, ASPEED_CE_CTRL_REG);
	return true;
}

/*
 * Calculate spi clock frequency divisor. Refer to AST2500 datasheet,
 * Chapter 14, CE# Control Register, bit 11:8 for details.
 */
static u32 aspeed_spi_clk_div(unsigned long ahb_clk_freq,
			      u32 spi_max_freq)
{
	unsigned long i;
	u32 div_val = 0;
	static const u32 div_map[ASPEED_CTRL_CLK_DIV_MAX] = {
		15, 7, 14, 6, 13, 5, 12, 4,
		11, 3, 10, 2, 9, 1, 8, 0,
	};

	for (i = 1; i <= ASPEED_CTRL_CLK_DIV_MAX; i++) {
		if ((spi_max_freq * i) >= ahb_clk_freq) {
			div_val = div_map[i - 1];
			break;
		}
	}

	return div_val;
}

static int
aspeed_spi_setup(struct spi_device *slave)
{
	u8 cs = slave->chip_select;
	u32 div, val, ctrl_reg, freq;
	struct aspeed_spi_priv *priv = spi_master_get_devdata(slave->master);

	if (cs == 0 && slave->mode & SPI_CS_HIGH) {
		dev_err(&slave->dev,
			"chip_select %u cannot be active-high\n", cs);
		return -EINVAL;
	}

	/*
	 * Update SPIR00 (Configuration Register).
	 */
	val = aspeed_reg_read(priv, ASPEED_CFG_REG);
	val |= cs_reg_map[cs].enable_write;
	aspeed_reg_write(priv, val, ASPEED_CFG_REG);

	/*
	 * Update CE# Control Register.
	 */
	ctrl_reg = cs_reg_map[cs].ctrl_reg;
	val = aspeed_reg_read(priv, ctrl_reg);
	val &= ~ASPEED_CTRL_CLK_MODE_3; /* clock mode 3 is not supported */
	if (slave->max_speed_hz != 0) {
		freq = slave->max_speed_hz;
		val &= ~ASPEED_CTRL_CLK_DIV_MASK;

		if (aspeed_check_set_div2(priv, cs, freq)) {
			/*
			 * SPI clock divided by 4.
			 */
			val |= ASPEED_CTRL_CLK_DIV4_MODE;
			freq = freq >> 2;
		}
		div = aspeed_spi_clk_div(priv->ahb_clk_freq, freq);
		val |= ASPEED_CTRL_CLK_DIV(div);
	}
	aspeed_reg_write(priv, val, ctrl_reg);

	return 0;
}

static void aspeed_spi_do_xfer(struct aspeed_spi_priv *priv,
			       struct spi_transfer *xfer, u32 cs)
{
	unsigned int i;
	u8 *rx_buf = xfer->rx_buf;
	const u8 *tx_buf = xfer->tx_buf;
	void *slave_buf = priv->slave_buf[cs].start;

	if (tx_buf != NULL) {
		for (i = 0; i < xfer->len; i++)
			writeb(tx_buf[i], slave_buf);
	}

	if (rx_buf != NULL) {
		for (i = 0; i < xfer->len; i++)
			rx_buf[i] = readb(slave_buf);
	}
}

static int aspeed_spi_xfer_one_msg(struct spi_master *master,
				   struct spi_message *msg)
{
	unsigned long flags;
	struct spi_transfer *xfer;
	struct spi_device *slave = msg->spi;
	struct aspeed_spi_priv *priv = spi_master_get_devdata(master);
	u8 cs = slave->chip_select;

	spin_lock_irqsave(&priv->lock, flags);

	msg->actual_length = 0;
	aspeed_activate_cs(priv, cs);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		aspeed_spi_do_xfer(priv, xfer, cs);
		msg->actual_length += xfer->len;
	}
	aspeed_deactivate_cs(priv, cs);

	msg->status = 0;
	msg->complete(msg->context);
	spin_unlock_irqrestore(&priv->lock, flags);

	spi_finalize_current_message(master);
	return 0;
}

static void aspeed_spi_init_hw(struct aspeed_spi_priv *priv, u16 num_cs)
{
	u16 cs;

	for (cs = 0; cs < num_cs; cs++) {
		u32 ctrl_reg = cs_reg_map[cs].ctrl_reg;
		u32 val = aspeed_reg_read(priv, ctrl_reg);

		val |= (ASPEED_CTRL_STOP_ACTIVE | ASPEED_CTRL_USER_MODE);
		aspeed_reg_write(priv, val, ctrl_reg);
	}
}

static int aspeed_spi_init_slave_buf(struct aspeed_spi_priv *priv,
				     struct resource *res, u16 num_cs)
{
	u16 cs;

	for (cs = 0; cs < num_cs; cs++) {
		u32 val, start, end, size, offset;

		val = aspeed_reg_read(priv, cs_reg_map[cs].addr_range_reg);
		start = SEGMENT_ADDR_START(val);
		end = SEGMENT_ADDR_END(val);
		size = end - start;

		if (start < res->start) {
			dev_err(priv->dev,
				"cs %u: invalid start address %#lx\n",
				cs, (unsigned long)start);
			return -EINVAL;
		}

		offset = start - res->start;
		if (offset + size > priv->slave_mem_size) {
			dev_err(priv->dev,
				"cs %u: invalid address range (%#lx - %#lx)\n",
				cs, (unsigned long)start,
				(unsigned long)(start + size));
				return -EINVAL;
		}

		priv->slave_buf[cs].start = priv->slave_base + offset;
		priv->slave_buf[cs].size = size;
	}

	return 0;
}

static int aspeed_spi_probe(struct platform_device *pdev)
{
	u32 slave_mem_size;
	struct resource *res;
	void __iomem *reg_base, *slave_base;
	struct clk *clk;
	struct aspeed_spi_priv *priv;
	struct spi_master *master;
	int error;
	unsigned long ahb_clk_freq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(reg_base))
		return PTR_ERR(reg_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	slave_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(slave_base))
		return PTR_ERR(slave_base);
	slave_mem_size = resource_size(res);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	ahb_clk_freq = clk_get_rate(clk);
	devm_clk_put(&pdev->dev, clk);

	master = spi_alloc_master(&pdev->dev, sizeof(struct aspeed_spi_priv));
	if (master == NULL) {
		dev_err(&pdev->dev, "failed to allocate spi_master\n");
		return -ENOMEM;
	}
	master->mode_bits = ASPEED_SUPP_MODES;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	master->dev.of_node = pdev->dev.of_node;
	master->num_chipselect = ASPEED_SPI_CS_NUM;
	master->setup = aspeed_spi_setup;
	master->transfer_one_message = aspeed_spi_xfer_one_msg;

	priv = spi_master_get_devdata(master);
	priv->dev = &pdev->dev;
	priv->master = master;
	priv->reg_base = reg_base;
	priv->slave_base = slave_base;
	priv->slave_mem_size = slave_mem_size;
	priv->ahb_clk_freq = ahb_clk_freq;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);
	error = aspeed_spi_init_slave_buf(priv, res, master->num_chipselect);
	if (error != 0)
		goto err_exit;

	aspeed_spi_init_hw(priv, master->num_chipselect);

	/* Register our spi controller */
	error = devm_spi_register_master(&pdev->dev, master);
	if (error) {
		dev_err(&pdev->dev, "failed to register SPI master\n");
		goto err_exit;
	}

	return 0;

err_exit:
	spi_master_put(master);
	dev_err(&pdev->dev, "%s returned error %d\n", __func__, error);
	return error;
}

static int
aspeed_spi_remove(struct platform_device *pdev)
{
	struct aspeed_spi_priv *priv = platform_get_drvdata(pdev);

	if (priv == NULL)
		return -1;

	platform_set_drvdata(pdev, NULL);
	spi_master_put(priv->master);
	return 0;
}

static const struct of_device_id aspeed_spi_of_match[] = {
	{ .compatible = "aspeed,ast2500-spi-master", },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_spi_of_match);

static struct platform_driver aspeed_spi_driver = {
	.probe = aspeed_spi_probe,
	.remove = aspeed_spi_remove,
	.driver = {
		.name = ASPEED_SPI_DRIVER,
		.of_match_table = aspeed_spi_of_match,
	},
};
module_platform_driver(aspeed_spi_driver);

MODULE_AUTHOR("Tao Ren <taoren@fb.com>");
MODULE_DESCRIPTION("ASPEED SPI Host Driver");
MODULE_LICENSE("GPL");
