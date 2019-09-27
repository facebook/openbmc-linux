// SPDX-License-Identifier: GPL-2.0+
// Copyright (C) IBM Corporation 2018
// FSI master driver for AST2600

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fsi.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/iopoll.h>

#include "fsi-master.h"

/* TODO: these are copied from the hub master. Put them in a common header */

/* Control Registers */
#define FSI_MMODE		0x0		/* R/W: mode */
#define FSI_MDLYR		0x4		/* R/W: delay */
#define FSI_MCRSP		0x8		/* R/W: clock rate */
#define FSI_MENP0		0x10		/* R/W: enable */
#define FSI_MLEVP0		0x18		/* R: plug detect */
#define FSI_MSENP0		0x18		/* S: Set enable */
#define FSI_MCENP0		0x20		/* C: Clear enable */
#define FSI_MAEB		0x70		/* R: Error address */
#define FSI_MVER		0x74		/* R: master version/type */
#define FSI_MRESP0		0xd0		/* W: Port reset */
#define FSI_MESRB0		0x1d0		/* R: Master error status */
#define FSI_MRESB0		0x1d0		/* W: Reset bridge */
#define FSI_MECTRL		0x2e0		/* W: Error control */

/* MMODE: Mode control */
#define FSI_MMODE_EIP		0x80000000	/* Enable interrupt polling */
#define FSI_MMODE_ECRC		0x40000000	/* Enable error recovery */
#define FSI_MMODE_EPC		0x10000000	/* Enable parity checking */
#define FSI_MMODE_P8_TO_LSB	0x00000010	/* Timeout value LSB */
						/*   MSB=1, LSB=0 is 0.8 ms */
						/*   MSB=0, LSB=1 is 0.9 ms */
#define FSI_MMODE_CRS0SHFT	18		/* Clk rate selection 0 shift */
#define FSI_MMODE_CRS0MASK	0x3ff		/* Clk rate selection 0 mask */
#define FSI_MMODE_CRS1SHFT	8		/* Clk rate selection 1 shift */
#define FSI_MMODE_CRS1MASK	0x3ff		/* Clk rate selection 1 mask */

/* MRESB: Reset brindge */
#define FSI_MRESB_RST_GEN	0x80000000	/* General reset */
#define FSI_MRESB_RST_ERR	0x40000000	/* Error Reset */

/* MRESB: Reset port */
#define FSI_MRESP_RST_ALL_MASTER 0x20000000	/* Reset all FSI masters */
#define FSI_MRESP_RST_ALL_LINK	0x10000000	/* Reset all FSI port contr. */
#define FSI_MRESP_RST_MCR	0x08000000	/* Reset FSI master reg. */
#define FSI_MRESP_RST_PYE	0x04000000	/* Reset FSI parity error */
#define FSI_MRESP_RST_ALL	0xfc000000	/* Reset any error */

/* MECTRL: Error control */
#define FSI_MECTRL_EOAE		0x8000		/* Enable machine check when */
						/* master 0 in error */
#define FSI_MECTRL_P8_AUTO_TERM	0x4000		/* Auto terminate */

#define FSI_ENGID_HUB_MASTER		0x1c
#define FSI_HUB_LINK_OFFSET		0x80000
#define FSI_HUB_LINK_SIZE		0x80000
#define FSI_HUB_MASTER_MAX_LINKS	8

#define FSI_LINK_ENABLE_SETUP_TIME	10	/* in mS */

struct fsi_master_aspeed {
	struct fsi_master	master;
	struct device		*dev;
	void __iomem		*base;
	struct clk		*clk;
};

#define to_fsi_master_aspeed(m) \
	container_of(m, struct fsi_master_aspeed, master)

/* Control register (size 0x400) */
static const u32 ctrl_base = 0x80000000;

static const u32 fsi_base = 0xa0000000;

#define OPB_FSI_VER	0x00
#define OPB_TRIGGER	0x04
#define OPB_CTRL_BASE	0x08
#define OPB_FSI_BASE	0x0c
#define OPB_CLK_SYNC	0x3c
#define OPB_IRQ_CLEAR	0x40
#define OPB_IRQ_MASK	0x44
#define OPB_IRQ_STATUS	0x48

#define OPB0_SELECT	0x10
#define OPB0_RW		0x14
#define OPB0_XFER_SIZE	0x18
#define OPB0_FSI_ADDR	0x1c
#define OPB0_FSI_DATA_W	0x20
#define OPB0_STATUS	0x80
/* half world */
#define  STATUS_HW_ACK	BIT(0)
/* full word */
#define  STATUS_FW_ACK	BIT(1)
#define  STATUS_ERR_ACK	BIT(2)
#define OPB0_FSI_DATA_R	0x84

#define OPB0_W_ENDIAN	0x4c
#define OPB0_R_ENDIAN	0x5c

/* OPB_IRQ_MASK */
#define OPB1_XFER_ACK_EN BIT(17)
#define OPB0_XFER_ACK_EN BIT(16)

#define OPB_IRQ_CH0_DMA_EOT		BIT(0)
#define OPB_IRQ_CH1_DMA_EOT		BIT(1)
#define OPB_IRQ_CH2_DMA_EOT		BIT(2)
#define OPB_IRQ_CH3_DMA_EOT		BIT(3)
#define OPB_IRQ_CH0_DMA_FIFO_FULL	BIT(4)
#define OPB_IRQ_CH1_DMA_FIFO_FULL	BIT(5)
#define OPB_IRQ_CH2_DMA_FIFO_FULL	BIT(6)
#define OPB_IRQ_CH3_DMA_FIFO_FULL	BIT(7)
#define OPB_IRQ_CH0_DMA_FIFO_EMPTY	BIT(8)
#define OPB_IRQ_CH1_DMA_FIFO_EMPTY	BIT(9)
#define OPB_IRQ_CH2_DMA_FIFO_EMPTY	BIT(10)
#define OPB_IRQ_CH3_DMA_FIFO_EMPTY	BIT(11)
#define OPB_IRQ_CH0_DMA_TCONT_DONE	BIT(12)
#define OPB_IRQ_CH1_DMA_TCONT_DONE	BIT(13)
#define OPB_IRQ_CH2_DMA_TCONT_DONE	BIT(14)
#define OPB_IRQ_CH3_DMA_TCONT_DONE	BIT(15)
#define OPB_IRQ_OPB1_XFER_ACK		BIT(16)
#define OPB_IRQ_OPB0_XFER_ACK		BIT(17)
#define OPB_IRQ_SLAVE0			BIT(18)
#define OPB_IRQ_SLAVE1			BIT(19)
#define OPB_IRQ_SLAVE2			BIT(20)
#define OPB_IRQ_SLAVE3			BIT(21)
#define OPB_IRQ_SLAVE4			BIT(22)
#define OPB_IRQ_SLAVE5			BIT(23)
#define OPB_IRQ_SLAVE6			BIT(24)
#define OPB_IRQ_SLAVE7			BIT(25)
#define OPB_IRQ_ANY_HOTPLUG		BIT(26)
#define OPB_IRQ_ANY_PORT_ERROR		BIT(27)
#define OPB_IRQ_ANY_MST_ERROR		BIT(28)

/* OPB_RW */
#define CMD_READ	BIT(0)
#define CMD_WRITE	0

/* OPBx_XFER_SIZE */
#define XFER_WORD	(BIT(1) | BIT(0))
#define XFER_NIBBLE	(BIT(0))
#define XFER_BYTE	(0)

#define CREATE_TRACE_POINTS
#include <trace/events/fsi_master_aspeed.h>

static u32 opb_write(void __iomem *base, uint32_t addr, uint32_t val,
		     size_t size)
{
	u32 reg, ret, status;

	/* TODO: implement other sizes, see 0x18 */
	WARN_ON(size != 4);

	writel(0x1, base + OPB0_SELECT);
	writel(CMD_WRITE, base + OPB0_RW);
	writel(XFER_WORD, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(val, base + OPB0_FSI_DATA_W);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);

	ret = readl_poll_timeout(base + OPB_IRQ_STATUS, reg,
				(reg & OPB0_XFER_ACK_EN) != 0,
				1, 10000);

	status = readl(base + OPB0_STATUS);

	trace_fsi_master_aspeed_opb_write(base, addr, val, size,
			status, reg);

	/* Return error when poll timed out */
	if (ret)
		return ret;

	/* Command failed, master will reset */
	if (status & STATUS_ERR_ACK)
		return -EIO;

	return 0;
}

static int opb_read(void __iomem *base, uint32_t addr, size_t size, u32 *out)
{
	u32 result, reg;
	int status, ret;

	/* TODO: implement other sizes, see 0x18 */
	WARN_ON(size != 4);

	writel(0x1, base + OPB0_SELECT);
	writel(CMD_READ, base + OPB0_RW);
	writel(XFER_WORD, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);

	ret = readl_poll_timeout(base + OPB_IRQ_STATUS, reg,
			   (reg & OPB0_XFER_ACK_EN) != 0,
			   0, 100);

	status = readl(base + OPB0_STATUS);

	result = readl(base + OPB0_FSI_DATA_R);

	trace_fsi_master_aspeed_opb_read(base, addr, size, result,
			readl(base + OPB0_STATUS),
			reg);

	/* Return error when poll timed out */
	if (ret)
		return ret;

	/* Command failed, master will reset */
	if (status & STATUS_ERR_ACK)
		return -EIO;

	if (out)
		*out = result;

	return 0;
}

static int check_errors(struct fsi_master_aspeed *aspeed, int err)
{
	int ret;

	if (err == -EIO) {
		/* Check MAEB (0x70) ? */

		/* Then clear errors in master */
		ret = opb_write(aspeed->base, ctrl_base + 0xd0,
				cpu_to_be32(0x20000000), 4);
		if (ret) {
			/* TODO: log? return different code? */
			return ret;
		}
		/* TODO: confirm that 0x70 was okay */
	}

	/* This will pass through timeout errors */
	return err;
}

static int aspeed_master_read(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int ret;
	u32 data;

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;
	ret = opb_read(aspeed->base, fsi_base + addr, size, &data);

	ret = check_errors(aspeed, ret);
	if (ret)
		return ret;

	memcpy(val, &data, size);

	return 0;
}

static int aspeed_master_write(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, const void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int ret;

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;
	ret = opb_write(aspeed->base, fsi_base + addr, *(uint32_t *)val, size);

	ret = check_errors(aspeed, ret);
	if (ret)
		return ret;

	return 0;
}

static int aspeed_master_link_enable(struct fsi_master *master, int link)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int idx, bit, ret;
	__be32 reg, result;

	idx = link / 32;
	bit = link % 32;

	reg = cpu_to_be32(0x80000000 >> bit);

	result = opb_write(aspeed->base, ctrl_base + FSI_MSENP0 + (4 * idx),
			reg, 4);

	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	ret = opb_read(aspeed->base, ctrl_base + FSI_MENP0 + (4 * idx),
			4, &result);
	if (ret)
		return ret;

	if (result != reg) {
		dev_err(aspeed->dev, "%s failed: %08x\n", __func__, result);
		return -EIO;
	}

	return 0;
}

static int aspeed_master_term(struct fsi_master *master, int link, uint8_t id)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	uint32_t addr;
	__be32 cmd;
	int rc;

	addr = 0x4;
	cmd = cpu_to_be32(0xecc00000);

	dev_dbg(aspeed->dev, "sending term to link %d slave %d\n", link, id);

	rc = aspeed_master_write(master, link, id, addr, &cmd, 4);

	dev_dbg(aspeed->dev, "term done (%d)\n", rc);

	return rc;
}

static int aspeed_master_break(struct fsi_master *master, int link)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	uint32_t addr;
	__be32 cmd;
	int rc;

	addr = 0x0;
	cmd = cpu_to_be32(0xc0de0000);

	dev_dbg(aspeed->dev, "sending break to link %d\n", link);

	rc = aspeed_master_write(master, link, 0, addr, &cmd, 4);

	dev_dbg(aspeed->dev, "break done (%d)\n", rc);

	return rc;
}

static void aspeed_master_release(struct device *dev)
{
	struct fsi_master_aspeed *aspeed =
		to_fsi_master_aspeed(dev_to_fsi_master(dev));

	kfree(aspeed);
}

/* mmode encoders */
static inline u32 fsi_mmode_crs0(u32 x)
{
	return (x & FSI_MMODE_CRS0MASK) << FSI_MMODE_CRS0SHFT;
}

static inline u32 fsi_mmode_crs1(u32 x)
{
	return (x & FSI_MMODE_CRS1MASK) << FSI_MMODE_CRS1SHFT;
}

static int aspeed_master_init(struct fsi_master_aspeed *aspeed)
{
	__be32 reg;

	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, 4);

	/* Initialize the MFSI (hub master) engine */
	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, 4);

	reg = cpu_to_be32(FSI_MECTRL_EOAE | FSI_MECTRL_P8_AUTO_TERM);
	opb_write(aspeed->base, ctrl_base + FSI_MECTRL, reg, 4);

	reg = cpu_to_be32(FSI_MMODE_EIP | FSI_MMODE_ECRC | FSI_MMODE_EPC
			| fsi_mmode_crs0(0x7f) | fsi_mmode_crs1(0x7f)
			| FSI_MMODE_P8_TO_LSB);
	opb_write(aspeed->base, ctrl_base + FSI_MMODE, reg, 4);

	reg = cpu_to_be32(0xffff0000);
	opb_write(aspeed->base, ctrl_base + FSI_MDLYR, reg, 4);

	reg = cpu_to_be32(~0);
	opb_write(aspeed->base, ctrl_base + FSI_MSENP0, reg, 4);

	/* Leave enabled long enough for master logic to set up */
	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	opb_write(aspeed->base, ctrl_base + FSI_MCENP0, reg, 4);

	opb_read(aspeed->base, ctrl_base + FSI_MAEB, 4, NULL);

	reg = cpu_to_be32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, 4);

	opb_read(aspeed->base, ctrl_base + FSI_MLEVP0, 4, NULL);

	/* Reset the master bridge */
	reg = cpu_to_be32(FSI_MRESB_RST_GEN);
	opb_write(aspeed->base, ctrl_base + FSI_MRESB0, reg, 4);

	reg = cpu_to_be32(FSI_MRESB_RST_ERR);
	opb_write(aspeed->base, ctrl_base + FSI_MRESB0, reg, 4);

	return 0;
}

static int fsi_master_aspeed_probe(struct platform_device *pdev)
{
	struct fsi_master_aspeed *aspeed;
	struct resource *res;
	int rc, links, reg;
	__be32 raw;

	aspeed = devm_kzalloc(&pdev->dev, sizeof(*aspeed), GFP_KERNEL);
	if (!aspeed)
		return -ENOMEM;

	aspeed->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	aspeed->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(aspeed->base))
		return PTR_ERR(aspeed->base);

	aspeed->clk = devm_clk_get(aspeed->dev, NULL);
	if (IS_ERR(aspeed->clk)) {
		dev_err(aspeed->dev, "couldn't get clock\n");
		return PTR_ERR(aspeed->clk);
	}
	rc = clk_prepare_enable(aspeed->clk);
	if (rc) {
		dev_err(aspeed->dev, "couldn't enable clock\n");
		return rc;
	}

	writel(0x1, aspeed->base + OPB_CLK_SYNC);
	writel(OPB1_XFER_ACK_EN | OPB0_XFER_ACK_EN,
			aspeed->base + OPB_IRQ_MASK);
	/* TODO: Try without this */
	writel(0x10, aspeed->base + 0x64); // Retry counter number ???
	writel(0x0f, aspeed->base + 0xe4); // DMA Enable

	writel(ctrl_base, aspeed->base + OPB_CTRL_BASE);
	writel(fsi_base, aspeed->base + OPB_FSI_BASE);

	/* Set read data order */
	writel(0x0011bb1b, aspeed->base + OPB0_R_ENDIAN);

	/* Set write data order */
	writel(0x0011bb1b, aspeed->base + OPB0_W_ENDIAN);
	writel(0xffaa5500, aspeed->base + 0x50);

	rc = opb_read(aspeed->base, ctrl_base + FSI_MVER, 4, &raw);
	if (rc) {
		dev_err(&pdev->dev, "failed to read hub version\n");
		return rc;
	}

	reg = be32_to_cpu(raw);
	links = (reg >> 8) & 0xff;
	dev_info(&pdev->dev, "hub version %08x (%d links)\n", reg, links);

	aspeed->master.dev.parent = &pdev->dev;
	aspeed->master.dev.release = aspeed_master_release;
	aspeed->master.dev.of_node = of_node_get(dev_of_node(&pdev->dev));

	aspeed->master.n_links = links;
	aspeed->master.read = aspeed_master_read;
	aspeed->master.write = aspeed_master_write;
	aspeed->master.send_break = aspeed_master_break;
	aspeed->master.term = aspeed_master_term;
	aspeed->master.link_enable = aspeed_master_link_enable;

	dev_set_drvdata(&pdev->dev, aspeed);

	aspeed_master_init(aspeed);

	rc = fsi_master_register(&aspeed->master);
	if (rc)
		goto err_release;

	/* At this point, fsi_master_register performs the device_initialize(),
	 * and holds the sole reference on master.dev. This means the device
	 * will be freed (via ->release) during any subsequent call to
	 * fsi_master_unregister.  We add our own reference to it here, so we
	 * can perform cleanup (in _remove()) without it being freed before
	 * we're ready.
	 */
	get_device(&aspeed->master.dev);
	return 0;

err_release:
	clk_disable_unprepare(aspeed->clk);
	return rc;
}

static int fsi_master_aspeed_remove(struct platform_device *pdev)
{
	struct fsi_master_aspeed *aspeed = platform_get_drvdata(pdev);

	fsi_master_unregister(&aspeed->master);
	clk_disable_unprepare(aspeed->clk);

	return 0;
}

static const struct of_device_id fsi_master_aspeed_match[] = {
	{ .compatible = "aspeed,ast2600-fsi-master" },
	{ },
};

static struct platform_driver fsi_master_aspeed_driver = {
	.driver = {
		.name		= "fsi-master-aspeed",
		.of_match_table	= fsi_master_aspeed_match,
	},
	.probe	= fsi_master_aspeed_probe,
	.remove = fsi_master_aspeed_remove,
};

module_platform_driver(fsi_master_aspeed_driver);
MODULE_LICENSE("GPL");
