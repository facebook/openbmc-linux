/* SPDX-License-Identifier: GPL-2.0 */
#ifndef LINUX_MMC_SDHCI_ASPEED_DATA_H
#define LINUX_MMC_SDHCI_ASPEED_DATA_H

#include <linux/io.h>

#define ASPEED_SDHCI_INFO			0x00
#define  ASPEED_SDHCI_S1MMC8			BIT(25)
#define  ASPEED_SDHCI_S0MMC8			BIT(24)
#define ASPEED_SDHCI_BLOCK			0x04
#define ASPEED_SDHCI_CTRL			0xF0
#define ASPEED_SDHCI_ISR			0xFC

struct aspeed_sdhci_irq {
	void __iomem *regs;
	int parent_irq;
	struct irq_domain *irq_domain;
};

static inline void aspeed_sdhci_set_8bit_mode(struct aspeed_sdhci_irq *sdhci_irq, int mode)
{
	if (mode)
		writel(ASPEED_SDHCI_S0MMC8 | readl(sdhci_irq->regs), sdhci_irq->regs);
	else
		writel(~ASPEED_SDHCI_S0MMC8 & readl(sdhci_irq->regs), sdhci_irq->regs);
}

#endif
