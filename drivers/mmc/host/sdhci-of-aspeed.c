// SPDX-License-Identifier: GPL-2.0
/*
 * ASPEED Secure Digital Host Controller Interface.
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mmc/sdhci-aspeed-data.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "sdhci-pltfm.h"

#define AST_SCU_FUN_PIN_CTRL5	0x90 /* Multi-function Pin Control#5*/
#define SCU_FUC_PIN_SD1_8BIT	(0x1 << 3)

static void sdhci_aspeed_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

	for (div = 1; div < 256; div *= 2) {
		if ((host->max_clk / div) <= clock)
			break;
	}
	div >>= 1;

	clk = div << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		 & SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never stabilised.\n",
			       mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;
}

static void sdhci_aspeed_set_bus_width(struct sdhci_host *host, int width)
{
	struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);
	struct aspeed_sdhci_irq *sdhci_irq = sdhci_pltfm_priv(pltfm_priv);

	u8 ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if (sdhci_irq->regs) {
		if (width == MMC_BUS_WIDTH_8)
			aspeed_sdhci_set_8bit_mode(sdhci_irq, 1);
		else
			aspeed_sdhci_set_8bit_mode(sdhci_irq, 0);
	}
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

}

static struct sdhci_ops  sdhci_aspeed_ops = {
	.set_clock = sdhci_aspeed_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_aspeed_set_bus_width,
	.get_timeout_clock = sdhci_pltfm_clk_get_max_clock,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_aspeed_pdata = {
	.ops = &sdhci_aspeed_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN,
};

static int sdhci_aspeed_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct device_node *pnode;
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host;
	struct aspeed_sdhci_irq *sdhci_irq;
	struct regmap *scu_map;
	unsigned int val;

	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_aspeed_pdata, sizeof(struct aspeed_sdhci_irq));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	sdhci_irq = sdhci_pltfm_priv(pltfm_host);

	sdhci_get_of_property(pdev);

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);

	pnode = of_parse_phandle(np, "interrupt-parent", 0);
	if (pnode)
		memcpy(sdhci_irq, pnode->data, sizeof(struct aspeed_sdhci_irq));

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	if (host->mmc->caps & MMC_CAP_8_BIT_DATA) {
		/* *
		 * just return if failed to enable 8 bits mode
		 * MMC will work under 4 bits mode eventually.
		 */
		scu_map = syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
		if (IS_ERR(scu_map)) {
			printk("%s no syscon regmap\n", (mmc_hostname(host->mmc)));
			return 0;
		}
		ret = regmap_read(scu_map, AST_SCU_FUN_PIN_CTRL5, &val);
		if (ret) {
			printk("%s failed to read SCU90\n", (mmc_hostname(host->mmc)));
			return 0;
		}
		val |= SCU_FUC_PIN_SD1_8BIT; // enable SD1 port 8 bits mode
		ret = regmap_write(scu_map, AST_SCU_FUN_PIN_CTRL5, val);
		if (ret) {
			printk("%s failed to enable SD1 8bit mode\n", (mmc_hostname(host->mmc)));
			return 0;
		}
	}

	return 0;

err_sdhci_add:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_aspeed_remove(struct platform_device *pdev)
{
	struct sdhci_host	*host = platform_get_drvdata(pdev);
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	sdhci_remove_host(host, dead);
	sdhci_pltfm_free(pdev);
	return 0;
}

static const struct of_device_id sdhci_aspeed_of_match[] = {
	{ .compatible = "aspeed,sdhci-ast2400", .data = &sdhci_aspeed_pdata },
	{ .compatible = "aspeed,sdhci-ast2500", .data = &sdhci_aspeed_pdata },
	{}
};

MODULE_DEVICE_TABLE(of, sdhci_aspeed_of_match);

static struct platform_driver sdhci_aspeed_driver = {
	.driver		= {
		.name	= "sdhci-aspeed",
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = sdhci_aspeed_of_match,
	},
	.probe		= sdhci_aspeed_probe,
	.remove		= sdhci_aspeed_remove,
};

module_platform_driver(sdhci_aspeed_driver);

MODULE_DESCRIPTION("Driver for the ASPEED SDHCI Controller");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_LICENSE("GPL v2");
