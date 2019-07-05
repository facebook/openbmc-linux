// SPDX-License-Identifier: GPL-2.0
/*
 * SDHCI IRQCHIP driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/mmc/sdhci-aspeed-data.h>

#define ASPEED_SDHCI_SLOT_NUM			2

static void aspeed_sdhci_irq_handler(struct irq_desc *desc)
{
	struct aspeed_sdhci_irq *sdhci_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int slot_irq;

	chained_irq_enter(chip, desc);
	status = readl(sdhci_irq->regs + ASPEED_SDHCI_ISR);
	for_each_set_bit(bit, &status, ASPEED_SDHCI_SLOT_NUM) {
		slot_irq = irq_find_mapping(sdhci_irq->irq_domain, bit);
		generic_handle_irq(slot_irq);
	}
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

struct irq_chip sdhci_irq_chip = {
	.name		= "sdhci-ic",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable	= noop,
	.irq_disable	= noop,
	.irq_ack	= noop,
	.irq_mask	= noop,
	.irq_unmask	= noop,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int ast_sdhci_map_irq_domain(struct irq_domain *domain,
				    unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &sdhci_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_sdhci_irq_domain_ops = {
	.map = ast_sdhci_map_irq_domain,
};

static int irq_aspeed_sdhci_probe(struct platform_device *pdev)
{
	struct aspeed_sdhci_irq *sdhci_irq;
	struct clk *sdclk;

	sdhci_irq = kzalloc(sizeof(*sdhci_irq), GFP_KERNEL);
	if (!sdhci_irq)
		return -ENOMEM;

	platform_set_drvdata(pdev, sdhci_irq);
	//node->data = sdhci_irq;
	pdev->dev.of_node->data = sdhci_irq;

	sdhci_irq->regs = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(sdhci_irq->regs))
		return PTR_ERR(sdhci_irq->regs);

	sdclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdclk)) {
		dev_err(&pdev->dev, "no sdclk clock defined\n");
		return PTR_ERR(sdclk);
	}

	clk_prepare_enable(sdclk);

	sdhci_irq->parent_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (sdhci_irq->parent_irq < 0)
		return sdhci_irq->parent_irq;

	sdhci_irq->irq_domain = irq_domain_add_linear(
					pdev->dev.of_node, ASPEED_SDHCI_SLOT_NUM,
					&aspeed_sdhci_irq_domain_ops, NULL);
	if (!sdhci_irq->irq_domain)
		return -ENOMEM;

	sdhci_irq->irq_domain->name = "aspeed-sdhci-irq";

	irq_set_chained_handler_and_data(sdhci_irq->parent_irq,
					 aspeed_sdhci_irq_handler, sdhci_irq);

	pr_info("sdhci irq controller registered, irq %d\n", sdhci_irq->parent_irq);

	return 0;
}

static const struct of_device_id irq_aspeed_sdhci_dt_ids[] = {
	{ .compatible = "aspeed,aspeed-sdhci-irq", },
	{},
};
MODULE_DEVICE_TABLE(of, irq_aspeed_sdhci_dt_ids);

static struct platform_driver irq_aspeed_sdhci_device_driver = {
	.probe		= irq_aspeed_sdhci_probe,
	.driver		= {
		.name   = KBUILD_MODNAME,
		.of_match_table	= irq_aspeed_sdhci_dt_ids,
	}
};

static int __init irq_aspeed_sdhci_init(void)
{
	return platform_driver_register(&irq_aspeed_sdhci_device_driver);
}
core_initcall(irq_aspeed_sdhci_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("ASPEED SOC SDHCI IRQ Driver");
MODULE_LICENSE("GPL v2");
