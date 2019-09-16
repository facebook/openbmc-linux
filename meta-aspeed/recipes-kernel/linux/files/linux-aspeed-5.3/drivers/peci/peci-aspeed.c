// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2012-2017 ASPEED Technology Inc.
// Copyright (c) 2018 Intel Corporation

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/peci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

/* ASPEED PECI Registers */
#define ASPEED_PECI_CTRL     0x00
#define ASPEED_PECI_TIMING   0x04
#define ASPEED_PECI_CMD      0x08
#define ASPEED_PECI_CMD_CTRL 0x0c
#define ASPEED_PECI_EXP_FCS  0x10
#define ASPEED_PECI_CAP_FCS  0x14
#define ASPEED_PECI_INT_CTRL 0x18
#define ASPEED_PECI_INT_STS  0x1c
#define ASPEED_PECI_W_DATA0  0x20
#define ASPEED_PECI_W_DATA1  0x24
#define ASPEED_PECI_W_DATA2  0x28
#define ASPEED_PECI_W_DATA3  0x2c
#define ASPEED_PECI_R_DATA0  0x30
#define ASPEED_PECI_R_DATA1  0x34
#define ASPEED_PECI_R_DATA2  0x38
#define ASPEED_PECI_R_DATA3  0x3c
#define ASPEED_PECI_W_DATA4  0x40
#define ASPEED_PECI_W_DATA5  0x44
#define ASPEED_PECI_W_DATA6  0x48
#define ASPEED_PECI_W_DATA7  0x4c
#define ASPEED_PECI_R_DATA4  0x50
#define ASPEED_PECI_R_DATA5  0x54
#define ASPEED_PECI_R_DATA6  0x58
#define ASPEED_PECI_R_DATA7  0x5c

/* ASPEED_PECI_CTRL - 0x00 : Control Register */
#define PECI_CTRL_SAMPLING_MASK      GENMASK(19, 16)
#define PECI_CTRL_READ_MODE_MASK     GENMASK(13, 12)
#define PECI_CTRL_READ_MODE_COUNT    BIT(12)
#define PECI_CTRL_READ_MODE_DBG      BIT(13)
#define PECI_CTRL_CLK_SOURCE_MASK    BIT(11)
#define PECI_CTRL_CLK_DIV_MASK       GENMASK(10, 8)
#define PECI_CTRL_INVERT_OUT         BIT(7)
#define PECI_CTRL_INVERT_IN          BIT(6)
#define PECI_CTRL_BUS_CONTENT_EN     BIT(5)
#define PECI_CTRL_PECI_EN            BIT(4)
#define PECI_CTRL_PECI_CLK_EN        BIT(0)

/* ASPEED_PECI_TIMING - 0x04 : Timing Negotiation Register */
#define PECI_TIMING_MESSAGE_MASK     GENMASK(15, 8)
#define PECI_TIMING_ADDRESS_MASK     GENMASK(7, 0)

/* ASPEED_PECI_CMD - 0x08 : Command Register */
#define PECI_CMD_PIN_MON             BIT(31)
#define PECI_CMD_STS_MASK            GENMASK(27, 24)
#define PECI_CMD_IDLE_MASK           (PECI_CMD_STS_MASK | PECI_CMD_PIN_MON)
#define PECI_CMD_FIRE                BIT(0)

/* ASPEED_PECI_LEN - 0x0C : Read/Write Length Register */
#define PECI_AW_FCS_EN               BIT(31)
#define PECI_READ_LEN_MASK           GENMASK(23, 16)
#define PECI_WRITE_LEN_MASK          GENMASK(15, 8)
#define PECI_TAGET_ADDR_MASK         GENMASK(7, 0)

/* ASPEED_PECI_EXP_FCS - 0x10 : Expected FCS Data Register */
#define PECI_EXPECT_READ_FCS_MASK    GENMASK(23, 16)
#define PECI_EXPECT_AW_FCS_AUTO_MASK GENMASK(15, 8)
#define PECI_EXPECT_WRITE_FCS_MASK   GENMASK(7, 0)

/* ASPEED_PECI_CAP_FCS - 0x14 : Captured FCS Data Register */
#define PECI_CAPTURE_READ_FCS_MASK   GENMASK(23, 16)
#define PECI_CAPTURE_WRITE_FCS_MASK  GENMASK(7, 0)

/* ASPEED_PECI_INT_CTRL/STS - 0x18/0x1c : Interrupt Register */
#define PECI_INT_TIMING_RESULT_MASK  GENMASK(31, 30)
#define PECI_INT_TIMEOUT             BIT(4)
#define PECI_INT_CONNECT             BIT(3)
#define PECI_INT_W_FCS_BAD           BIT(2)
#define PECI_INT_W_FCS_ABORT         BIT(1)
#define PECI_INT_CMD_DONE            BIT(0)

#define PECI_INT_MASK  (PECI_INT_TIMEOUT | PECI_INT_CONNECT | \
			PECI_INT_W_FCS_BAD | PECI_INT_W_FCS_ABORT | \
			PECI_INT_CMD_DONE)

#define PECI_IDLE_CHECK_TIMEOUT_USEC    50000
#define PECI_IDLE_CHECK_INTERVAL_USEC   10000

#define PECI_RD_SAMPLING_POINT_DEFAULT  8
#define PECI_RD_SAMPLING_POINT_MAX      15
#define PECI_CLK_DIV_DEFAULT            0
#define PECI_CLK_DIV_MAX                7
#define PECI_MSG_TIMING_DEFAULT         1
#define PECI_MSG_TIMING_MAX             255
#define PECI_ADDR_TIMING_DEFAULT        1
#define PECI_ADDR_TIMING_MAX            255
#define PECI_CMD_TIMEOUT_MS_DEFAULT     1000
#define PECI_CMD_TIMEOUT_MS_MAX         60000

struct aspeed_peci {
	struct peci_adapter	*adapter;
	struct device		*dev;
	struct regmap		*regmap;
	struct clk		*clk;
	struct reset_control	*rst;
	int			irq;
	spinlock_t		lock; /* to sync completion status handling */
	struct completion	xfer_complete;
	u32			status;
	u32			cmd_timeout_ms;
};

static int aspeed_peci_xfer_native(struct aspeed_peci *priv,
				   struct peci_xfer_msg *msg)
{
	long err, timeout = msecs_to_jiffies(priv->cmd_timeout_ms);
	u32 peci_head, peci_state, rx_data, cmd_sts;
	unsigned long flags;
	int i, rc;
	uint reg;

	/* Check command sts and bus idle state */
	rc = regmap_read_poll_timeout(priv->regmap, ASPEED_PECI_CMD, cmd_sts,
				      !(cmd_sts & PECI_CMD_IDLE_MASK),
				      PECI_IDLE_CHECK_INTERVAL_USEC,
				      PECI_IDLE_CHECK_TIMEOUT_USEC);
	if (rc)
		return rc; /* -ETIMEDOUT */

	spin_lock_irqsave(&priv->lock, flags);
	reinit_completion(&priv->xfer_complete);

	peci_head = FIELD_PREP(PECI_TAGET_ADDR_MASK, msg->addr) |
		    FIELD_PREP(PECI_WRITE_LEN_MASK, msg->tx_len) |
		    FIELD_PREP(PECI_READ_LEN_MASK, msg->rx_len);

	regmap_write(priv->regmap, ASPEED_PECI_CMD_CTRL, peci_head);

	for (i = 0; i < msg->tx_len; i += 4) {
		reg = i < 16 ? ASPEED_PECI_W_DATA0 + i % 16 :
			       ASPEED_PECI_W_DATA4 + i % 16;
		regmap_write(priv->regmap, reg,
			     le32_to_cpup((__le32 *)&msg->tx_buf[i]));
	}

	dev_dbg(priv->dev, "HEAD : 0x%08x\n", peci_head);
	print_hex_dump_debug("TX : ", DUMP_PREFIX_NONE, 16, 1,
			     msg->tx_buf, msg->tx_len, true);

	priv->status = 0;
	regmap_write(priv->regmap, ASPEED_PECI_CMD, PECI_CMD_FIRE);
	spin_unlock_irqrestore(&priv->lock, flags);

	err = wait_for_completion_interruptible_timeout(&priv->xfer_complete,
							timeout);

	spin_lock_irqsave(&priv->lock, flags);
	dev_dbg(priv->dev, "INT_STS : 0x%08x\n", priv->status);
	regmap_read(priv->regmap, ASPEED_PECI_CMD, &peci_state);
	dev_dbg(priv->dev, "PECI_STATE : 0x%lx\n",
		FIELD_GET(PECI_CMD_STS_MASK, peci_state));

	regmap_write(priv->regmap, ASPEED_PECI_CMD, 0);

	if (err <= 0 || priv->status != PECI_INT_CMD_DONE) {
		if (err < 0) { /* -ERESTARTSYS */
			rc = (int)err;
			goto err_irqrestore;
		} else if (err == 0) {
			dev_dbg(priv->dev, "Timeout waiting for a response!\n");
			rc = -ETIMEDOUT;
			goto err_irqrestore;
		}

		dev_dbg(priv->dev, "No valid response!\n");
		rc = -EIO;
		goto err_irqrestore;
	}

	/**
	 * Note that rx_len and rx_buf size can be an odd number.
	 * Byte handling is more efficient.
	 */
	for (i = 0; i < msg->rx_len; i++) {
		u8 byte_offset = i % 4;

		if (byte_offset == 0) {
			reg = i < 16 ? ASPEED_PECI_R_DATA0 + i % 16 :
				       ASPEED_PECI_R_DATA4 + i % 16;
			regmap_read(priv->regmap, reg, &rx_data);
		}

		msg->rx_buf[i] = (u8)(rx_data >> (byte_offset << 3));
	}

	print_hex_dump_debug("RX : ", DUMP_PREFIX_NONE, 16, 1,
			     msg->rx_buf, msg->rx_len, true);

	regmap_read(priv->regmap, ASPEED_PECI_CMD, &peci_state);
	dev_dbg(priv->dev, "PECI_STATE : 0x%lx\n",
		FIELD_GET(PECI_CMD_STS_MASK, peci_state));
	dev_dbg(priv->dev, "------------------------\n");

err_irqrestore:
	spin_unlock_irqrestore(&priv->lock, flags);
	return rc;
}

static irqreturn_t aspeed_peci_irq_handler(int irq, void *arg)
{
	struct aspeed_peci *priv = arg;
	u32 status_ack = 0;
	u32 status;

	spin_lock(&priv->lock);
	regmap_read(priv->regmap, ASPEED_PECI_INT_STS, &status);
	priv->status |= (status & PECI_INT_MASK);

	/**
	 * In most cases, interrupt bits will be set one by one but also note
	 * that multiple interrupt bits could be set at the same time.
	 */
	if (status & PECI_INT_TIMEOUT) {
		dev_dbg(priv->dev, "PECI_INT_TIMEOUT\n");
		status_ack |= PECI_INT_TIMEOUT;
	}

	if (status & PECI_INT_CONNECT) {
		dev_dbg(priv->dev, "PECI_INT_CONNECT\n");
		status_ack |= PECI_INT_CONNECT;
	}

	if (status & PECI_INT_W_FCS_BAD) {
		dev_dbg(priv->dev, "PECI_INT_W_FCS_BAD\n");
		status_ack |= PECI_INT_W_FCS_BAD;
	}

	if (status & PECI_INT_W_FCS_ABORT) {
		dev_dbg(priv->dev, "PECI_INT_W_FCS_ABORT\n");
		status_ack |= PECI_INT_W_FCS_ABORT;
	}

	/**
	 * All commands should be ended up with a PECI_INT_CMD_DONE bit set
	 * even in an error case.
	 */
	if (status & PECI_INT_CMD_DONE) {
		dev_dbg(priv->dev, "PECI_INT_CMD_DONE\n");
		status_ack |= PECI_INT_CMD_DONE;
		complete(&priv->xfer_complete);
	}

	regmap_write(priv->regmap, ASPEED_PECI_INT_STS, status_ack);
	spin_unlock(&priv->lock);
	return IRQ_HANDLED;
}

static int aspeed_peci_init_ctrl(struct aspeed_peci *priv)
{
	u32 msg_timing, addr_timing, rd_sampling_point;
	u32 clk_freq, clk_divisor, clk_div_val = 0;
	int ret;

	priv->clk = devm_clk_get(priv->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(priv->dev, "Failed to get clk source.\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable clock.\n");
		return ret;
	}

	ret = of_property_read_u32(priv->dev->of_node, "clock-frequency",
				   &clk_freq);
	if (ret) {
		dev_err(priv->dev,
			"Could not read clock-frequency property.\n");
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	clk_divisor = clk_get_rate(priv->clk) / clk_freq;

	while ((clk_divisor >> 1) && (clk_div_val < PECI_CLK_DIV_MAX))
		clk_div_val++;

	ret = of_property_read_u32(priv->dev->of_node, "msg-timing",
				   &msg_timing);
	if (ret || msg_timing > PECI_MSG_TIMING_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid msg-timing : %u, Use default : %u\n",
				 msg_timing, PECI_MSG_TIMING_DEFAULT);
		msg_timing = PECI_MSG_TIMING_DEFAULT;
	}

	ret = of_property_read_u32(priv->dev->of_node, "addr-timing",
				   &addr_timing);
	if (ret || addr_timing > PECI_ADDR_TIMING_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid addr-timing : %u, Use default : %u\n",
				 addr_timing, PECI_ADDR_TIMING_DEFAULT);
		addr_timing = PECI_ADDR_TIMING_DEFAULT;
	}

	ret = of_property_read_u32(priv->dev->of_node, "rd-sampling-point",
				   &rd_sampling_point);
	if (ret || rd_sampling_point > PECI_RD_SAMPLING_POINT_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid rd-sampling-point : %u. Use default : %u\n",
				 rd_sampling_point,
				 PECI_RD_SAMPLING_POINT_DEFAULT);
		rd_sampling_point = PECI_RD_SAMPLING_POINT_DEFAULT;
	}

	ret = of_property_read_u32(priv->dev->of_node, "cmd-timeout-ms",
				   &priv->cmd_timeout_ms);
	if (ret || priv->cmd_timeout_ms > PECI_CMD_TIMEOUT_MS_MAX ||
	    priv->cmd_timeout_ms == 0) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid cmd-timeout-ms : %u. Use default : %u\n",
				 priv->cmd_timeout_ms,
				 PECI_CMD_TIMEOUT_MS_DEFAULT);
		priv->cmd_timeout_ms = PECI_CMD_TIMEOUT_MS_DEFAULT;
	}

	regmap_write(priv->regmap, ASPEED_PECI_CTRL,
		     FIELD_PREP(PECI_CTRL_CLK_DIV_MASK, PECI_CLK_DIV_DEFAULT) |
		     PECI_CTRL_PECI_CLK_EN);

	/**
	 * Timing negotiation period setting.
	 * The unit of the programmed value is 4 times of PECI clock period.
	 */
	regmap_write(priv->regmap, ASPEED_PECI_TIMING,
		     FIELD_PREP(PECI_TIMING_MESSAGE_MASK, msg_timing) |
		     FIELD_PREP(PECI_TIMING_ADDRESS_MASK, addr_timing));

	/* Clear interrupts */
	regmap_write(priv->regmap, ASPEED_PECI_INT_STS, PECI_INT_MASK);

	/* Enable interrupts */
	regmap_write(priv->regmap, ASPEED_PECI_INT_CTRL, PECI_INT_MASK);

	/* Read sampling point and clock speed setting */
	regmap_write(priv->regmap, ASPEED_PECI_CTRL,
		     FIELD_PREP(PECI_CTRL_SAMPLING_MASK, rd_sampling_point) |
		     FIELD_PREP(PECI_CTRL_CLK_DIV_MASK, clk_div_val) |
		     PECI_CTRL_PECI_EN | PECI_CTRL_PECI_CLK_EN);

	return 0;
}

static const struct regmap_config aspeed_peci_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = ASPEED_PECI_R_DATA7,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.fast_io = true,
};

static int aspeed_peci_xfer(struct peci_adapter *adapter,
			    struct peci_xfer_msg *msg)
{
	struct aspeed_peci *priv = peci_get_adapdata(adapter);

	return aspeed_peci_xfer_native(priv, msg);
}

static int aspeed_peci_probe(struct platform_device *pdev)
{
	struct peci_adapter *adapter;
	struct aspeed_peci *priv;
	struct resource *res;
	void __iomem *base;
	u32 cmd_sts;
	int ret;

	adapter = peci_alloc_adapter(&pdev->dev, sizeof(*priv));
	if (!adapter)
		return -ENOMEM;

	priv = peci_get_adapdata(adapter);
	priv->adapter = adapter;
	priv->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto err_put_adapter_dev;
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &aspeed_peci_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		goto err_put_adapter_dev;
	}

	/**
	 * We check that the regmap works on this very first access,
	 * but as this is an MMIO-backed regmap, subsequent regmap
	 * access is not going to fail and we skip error checks from
	 * this point.
	 */
	ret = regmap_read(priv->regmap, ASPEED_PECI_CMD, &cmd_sts);
	if (ret) {
		ret = -EIO;
		goto err_put_adapter_dev;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (!priv->irq) {
		ret = -ENODEV;
		goto err_put_adapter_dev;
	}

	ret = devm_request_irq(&pdev->dev, priv->irq, aspeed_peci_irq_handler,
			       0, "peci-aspeed-irq", priv);
	if (ret)
		goto err_put_adapter_dev;

	init_completion(&priv->xfer_complete);
	spin_lock_init(&priv->lock);

	priv->adapter->owner = THIS_MODULE;
	priv->adapter->dev.of_node = of_node_get(dev_of_node(priv->dev));
	strlcpy(priv->adapter->name, pdev->name, sizeof(priv->adapter->name));
	priv->adapter->xfer = aspeed_peci_xfer;

	priv->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller entry");
		ret = PTR_ERR(priv->rst);
		goto err_put_adapter_dev;
	}
	reset_control_deassert(priv->rst);

	ret = aspeed_peci_init_ctrl(priv);
	if (ret)
		goto err_put_adapter_dev;

	ret = peci_add_adapter(priv->adapter);
	if (ret)
		goto err_put_adapter_dev;

	dev_info(&pdev->dev, "peci bus %d registered, irq %d\n",
		 priv->adapter->nr, priv->irq);

	return 0;

err_put_adapter_dev:
	put_device(&adapter->dev);
	return ret;
}

static int aspeed_peci_remove(struct platform_device *pdev)
{
	struct aspeed_peci *priv = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(priv->clk);
	reset_control_assert(priv->rst);
	peci_del_adapter(priv->adapter);
	of_node_put(priv->adapter->dev.of_node);

	return 0;
}

static const struct of_device_id aspeed_peci_of_table[] = {
	{ .compatible = "aspeed,ast2400-peci", },
	{ .compatible = "aspeed,ast2500-peci", },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_peci_of_table);

static struct platform_driver aspeed_peci_driver = {
	.probe  = aspeed_peci_probe,
	.remove = aspeed_peci_remove,
	.driver = {
		.name           = "peci-aspeed",
		.of_match_table = of_match_ptr(aspeed_peci_of_table),
	},
};
module_platform_driver(aspeed_peci_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_AUTHOR("Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>");
MODULE_DESCRIPTION("ASPEED PECI driver");
MODULE_LICENSE("GPL v2");
