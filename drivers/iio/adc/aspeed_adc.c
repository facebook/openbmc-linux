// SPDX-License-Identifier: GPL-2.0-only
/*
 * Aspeed AST2400/2500/2600 ADC
 *
 * Copyright (C) 2017 Google, Inc.
 * Copyright (C) ASPEED Technology Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iopoll.h>

#define ASPEED_RESOLUTION_BITS		10
#define ASPEED_CLOCKS_PER_SAMPLE	12

#define ASPEED_REG_ENGINE_CONTROL	0x00
#define ASPEED_REG_INTERRUPT_CONTROL	0x04
#define ASPEED_REG_VGA_DETECT_CONTROL	0x08
#define ASPEED_REG_CLOCK_CONTROL	0x0C
#define ASPEED_REG_COMPENSATION_TRIM	0xC4
#define ASPEED_REG_MAX			0xC0

//ast2600
#define REF_VLOTAGE_2500mV		0
#define REF_VLOTAGE_1200mV		(1 << 6)
#define REF_VLOTAGE_1550mV		(2 << 6)
#define REF_VLOTAGE_900mV		(3 << 6)

#define ASPEED_AUTOPENSATING		BIT(5)

#define ASPEED_OPERATION_MODE_POWER_DOWN	(0x0 << 1)
#define ASPEED_OPERATION_MODE_STANDBY		(0x1 << 1)
#define ASPEED_OPERATION_MODE_NORMAL		(0x7 << 1)

#define ASPEED_ENGINE_ENABLE		BIT(0)

#define ASPEED_ADC_CTRL_INIT_RDY	BIT(8)

#define ASPEED_CTRL_COMPENSATION	BIT(4)
#define ASPEED_ADC_CTRL_CH_EN(n)	(1 << (16 + n))
#define ASPEED_ADC_CTRL_CH_EN_ALL	GENMASK(31, 16)

#define ASPEED_ADC_INIT_POLLING_TIME	500
#define ASPEED_ADC_INIT_TIMEOUT		500000

struct aspeed_adc_trim_locate {
	unsigned int scu_offset;
	unsigned int bit_offset;
	unsigned int bit_mask;
};

struct aspeed_adc_model_data {
	const char *model_name;
	unsigned int min_sampling_rate;	// Hz
	unsigned int max_sampling_rate;	// Hz
	unsigned int vref_voltage;	// mV
	bool wait_init_sequence;
	struct iio_chan_spec const	*channels;
	int num_channels;
};

struct aspeed_adc_data {
	struct device		*dev;
	void __iomem		*base;
	spinlock_t		clk_lock;
	struct regmap		*scu;
	struct clk_hw		*clk_prescaler;
	struct clk_hw		*clk_scaler;
	struct reset_control	*rst;
	int					cv;
};

#define ASPEED_CHAN(_idx, _data_reg_addr) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec aspeed_adc_iio_channels[] = {
	ASPEED_CHAN(0, 0x10),
	ASPEED_CHAN(1, 0x12),
	ASPEED_CHAN(2, 0x14),
	ASPEED_CHAN(3, 0x16),
	ASPEED_CHAN(4, 0x18),
	ASPEED_CHAN(5, 0x1A),
	ASPEED_CHAN(6, 0x1C),
	ASPEED_CHAN(7, 0x1E),
	ASPEED_CHAN(8, 0x20),
	ASPEED_CHAN(9, 0x22),
	ASPEED_CHAN(10, 0x24),
	ASPEED_CHAN(11, 0x26),
	ASPEED_CHAN(12, 0x28),
	ASPEED_CHAN(13, 0x2A),
	ASPEED_CHAN(14, 0x2C),
	ASPEED_CHAN(15, 0x2E),
};

static const struct iio_chan_spec ast2600_adc_iio_channels[] = {
	ASPEED_CHAN(0, 0x10),
	ASPEED_CHAN(1, 0x12),
	ASPEED_CHAN(2, 0x14),
	ASPEED_CHAN(3, 0x16),
	ASPEED_CHAN(4, 0x18),
	ASPEED_CHAN(5, 0x1A),
	ASPEED_CHAN(6, 0x1C),
	ASPEED_CHAN(7, 0x1E),
};

static int aspeed_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
			of_device_get_match_data(data->dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!strcmp(model_data->model_name, "ast2600-adc")) {
			*val = readw(data->base + chan->address) + data->cv;
		}
		else {
			*val = readw(data->base + chan->address);
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = model_data->vref_voltage;
		*val2 = ASPEED_RESOLUTION_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(data->clk_scaler->clk) /
				ASPEED_CLOCKS_PER_SAMPLE;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
			of_device_get_match_data(data->dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val < model_data->min_sampling_rate ||
			val > model_data->max_sampling_rate)
			return -EINVAL;

		clk_set_rate(data->clk_scaler->clk,
				val * ASPEED_CLOCKS_PER_SAMPLE);
		return 0;

	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_RAW:
		/*
		 * Technically, these could be written but the only reasons
		 * for doing so seem better handled in userspace.  EPERM is
		 * returned to signal this is a policy choice rather than a
		 * hardware limitation.
		 */
		return -EPERM;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_reg_access(struct iio_dev *indio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);

	if (!readval || reg % 4 || reg > ASPEED_REG_MAX)
		return -EINVAL;

	*readval = readl(data->base + reg);

	return 0;
}

static const struct iio_info aspeed_adc_iio_info = {
	.read_raw = aspeed_adc_read_raw,
	.write_raw = aspeed_adc_write_raw,
	.debugfs_reg_access = aspeed_adc_reg_access,
};

static int aspeed_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct aspeed_adc_data *data;
	struct aspeed_adc_trim_locate trim_locate;
	const struct aspeed_adc_model_data *model_data;
	const char *clk_parent_name;
	char prescaler_clk_name[32];
	char scaler_clk_name[32];
	int ret;
	u32 eng_ctrl = 0;
	u32 adc_engine_control_reg_val;
	u32 scu_otp;
	u32 trim;
	u32 compensating_trim;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->dev = &pdev->dev;

	data->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	/* Register ADC clock prescaler with source specified by device tree. */
	spin_lock_init(&data->clk_lock);
	clk_parent_name = of_clk_get_parent_name(pdev->dev.of_node, 0);

	snprintf(prescaler_clk_name, sizeof(prescaler_clk_name), "prescaler-%s",
		pdev->name);
	data->clk_prescaler = clk_hw_register_divider(
				&pdev->dev, prescaler_clk_name, clk_parent_name, 0,
				data->base + ASPEED_REG_CLOCK_CONTROL,
				17, 15, 0, &data->clk_lock);
	if (IS_ERR(data->clk_prescaler))
		return PTR_ERR(data->clk_prescaler);

	snprintf(scaler_clk_name, sizeof(scaler_clk_name), "scaler-%s", pdev->name);
	/*
	 * Register ADC clock scaler downstream from the prescaler. Allow rate
	 * setting to adjust the prescaler as well.
	 */
	data->clk_scaler = clk_hw_register_divider(
				&pdev->dev, scaler_clk_name, prescaler_clk_name,
				CLK_SET_RATE_PARENT,
				data->base + ASPEED_REG_CLOCK_CONTROL,
				0, 10, 0, &data->clk_lock);
	if (IS_ERR(data->clk_scaler)) {
		ret = PTR_ERR(data->clk_scaler);
		goto scaler_error;
	}

	data->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(data->rst)) {
		dev_err(&pdev->dev,
			"invalid or missing reset controller device tree entry");
		ret = PTR_ERR(data->rst);
		goto reset_error;
	}
	reset_control_deassert(data->rst);

	model_data = of_device_get_match_data(&pdev->dev);
	if (!of_property_read_u32(pdev->dev.of_node, "ref_voltage",
	(u32 *)&model_data->vref_voltage)) {
		if (model_data->vref_voltage == 2500)
			eng_ctrl = REF_VLOTAGE_2500mV;
		else if (model_data->vref_voltage == 1200)
			eng_ctrl = REF_VLOTAGE_1200mV;
		else if ((model_data->vref_voltage >= 1550) &&
			(model_data->vref_voltage <= 2700))
			eng_ctrl = REF_VLOTAGE_1550mV;
		else if ((model_data->vref_voltage >= 900) &&
			(model_data->vref_voltage <= 1650))
			eng_ctrl = REF_VLOTAGE_900mV;
		else {
			printk("error ref voltage %d \n", model_data->vref_voltage);
			eng_ctrl = 0;
		}
	} else
		eng_ctrl = 0;

	if (model_data->wait_init_sequence) {
		/* Enable engine in normal mode. */
		eng_ctrl |= ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE;
		writel(eng_ctrl, data->base + ASPEED_REG_ENGINE_CONTROL);

		/* Wait for initial sequence complete. */
		ret = readl_poll_timeout(data->base + ASPEED_REG_ENGINE_CONTROL,
					 adc_engine_control_reg_val,
					 adc_engine_control_reg_val &
					 ASPEED_ADC_CTRL_INIT_RDY,
					 ASPEED_ADC_INIT_POLLING_TIME,
					 ASPEED_ADC_INIT_TIMEOUT);
		if (ret)
			goto poll_timeout_error;
	}

	/*
	 * The auto compensating sensing mode is not ready for AST2600.
	 * Need to set the trimming data for Compensating sensing mode.
	 */
	if (!strcmp(model_data->model_name, "ast2600-adc")) {
		data->scu = syscon_regmap_lookup_by_compatible("aspeed,ast2600-scu");
		if (IS_ERR(data->scu)) {
			dev_err(&pdev->dev, "failed to find SCU regmap\n");
			ret = PTR_ERR(data->scu);
			goto syscon_regmap_error;
		}

		eng_ctrl = readl(data->base + ASPEED_REG_ENGINE_CONTROL);
		eng_ctrl |= (ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE);
		/* Trimming data setting */
		ret = of_property_read_u32_array(data->dev->of_node, "trim_locate",
						(u32 *)&trim_locate,
						sizeof(trim_locate) / 4);
		if (ret < 0) {
			printk(KERN_WARNING "Get trim_locate fail, ret %d\n", ret);
			trim = 0x0;
		} else {
			if (regmap_read(data->scu, trim_locate.scu_offset, &scu_otp)) {
				printk(KERN_WARNING "read scu trim value fail \n");
				trim = 0x0;
			} else {
				trim = (scu_otp >> trim_locate.bit_offset) &
					trim_locate.bit_mask;
			}
		}
		if (trim == 0x0) {
			trim = 0x8;
		}
		dev_info(data->dev, "trim %d \n", trim);
		compensating_trim = readl(data->base + ASPEED_REG_COMPENSATION_TRIM);
		compensating_trim = (compensating_trim & (~(GENMASK(3, 0)))) | trim;
		writel(compensating_trim, data->base + ASPEED_REG_COMPENSATION_TRIM);

		/* Compensating Sensing Mode */
		writel(eng_ctrl | ASPEED_CTRL_COMPENSATION,
				data->base + ASPEED_REG_ENGINE_CONTROL);
		writel(eng_ctrl | ASPEED_CTRL_COMPENSATION | ASPEED_ADC_CTRL_CH_EN(0),
				data->base + ASPEED_REG_ENGINE_CONTROL);
		mdelay(1);

		data->cv = 0x200 - (readl(data->base + 0x10) & GENMASK(9, 0));

		/* Disable Compensating Sensing mode */
		writel(eng_ctrl & (~ASPEED_CTRL_COMPENSATION),
			data->base + ASPEED_REG_ENGINE_CONTROL);
	} else {
		// do compensating calculation use ch 0
		writel(eng_ctrl | ASPEED_OPERATION_MODE_NORMAL |
				ASPEED_ENGINE_ENABLE | ASPEED_AUTOPENSATING,
				data->base + ASPEED_REG_ENGINE_CONTROL);

		writel(eng_ctrl | ASPEED_OPERATION_MODE_NORMAL | BIT(16) |
				ASPEED_ENGINE_ENABLE | ASPEED_AUTOPENSATING,
				data->base + ASPEED_REG_ENGINE_CONTROL);
		mdelay(1);

		data->cv = 0x200 - (readl(data->base + 0x10) & GENMASK(9, 0));

		writel(eng_ctrl | ASPEED_OPERATION_MODE_NORMAL |
				ASPEED_ENGINE_ENABLE | ASPEED_AUTOPENSATING,
				data->base + ASPEED_REG_ENGINE_CONTROL);
	}
	dev_info(data->dev, "cv %d \n", data->cv);

	/* Start all channels in normal mode. */
	ret = clk_prepare_enable(data->clk_scaler->clk);
	if (ret)
		goto clk_enable_error;

	adc_engine_control_reg_val = eng_ctrl | ASPEED_ADC_CTRL_CH_EN_ALL |
		ASPEED_OPERATION_MODE_NORMAL | ASPEED_ENGINE_ENABLE;
	writel(adc_engine_control_reg_val,
		data->base + ASPEED_REG_ENGINE_CONTROL);

	model_data = of_device_get_match_data(&pdev->dev);
	indio_dev->name = model_data->model_name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &aspeed_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = model_data->channels;
	indio_dev->num_channels = model_data->num_channels;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto iio_register_error;

	return 0;

iio_register_error:
	writel(ASPEED_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
clk_enable_error:
syscon_regmap_error:
poll_timeout_error:
	reset_control_assert(data->rst);
reset_error:
	clk_hw_unregister_divider(data->clk_scaler);
scaler_error:
	clk_hw_unregister_divider(data->clk_prescaler);
	return ret;
}

static int aspeed_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	writel(ASPEED_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
	reset_control_assert(data->rst);
	clk_hw_unregister_divider(data->clk_scaler);
	clk_hw_unregister_divider(data->clk_prescaler);

	return 0;
}

static const struct aspeed_adc_model_data ast2400_model_data = {
	.model_name = "ast2400-adc",
	.vref_voltage = 2500, // mV
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
	.channels = aspeed_adc_iio_channels,
	.num_channels = 16,
};

static const struct aspeed_adc_model_data ast2500_model_data = {
	.model_name = "ast2500-adc",
	.vref_voltage = 1800, // mV
	.min_sampling_rate = 1,
	.max_sampling_rate = 1000000,
	.wait_init_sequence = true,
	.channels = aspeed_adc_iio_channels,
	.num_channels = 16,
};

static const struct aspeed_adc_model_data ast2600_model_data = {
	.model_name = "ast2600-adc",
	.vref_voltage = 2500, // mV
	.min_sampling_rate = 1,
	.max_sampling_rate = 1000000,
	.wait_init_sequence = true,
	.channels = ast2600_adc_iio_channels,
	.num_channels = 8,
};

static const struct of_device_id aspeed_adc_matches[] = {
	{ .compatible = "aspeed,ast2400-adc", .data = &ast2400_model_data },
	{ .compatible = "aspeed,ast2500-adc", .data = &ast2500_model_data },
	{ .compatible = "aspeed,ast2600-adc", .data = &ast2600_model_data },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_adc_matches);

static struct platform_driver aspeed_adc_driver = {
	.probe = aspeed_adc_probe,
	.remove = aspeed_adc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_adc_matches,
	}
};

module_platform_driver(aspeed_adc_driver);

MODULE_AUTHOR("Rick Altherr <raltherr@google.com>");
MODULE_DESCRIPTION("Aspeed AST2400/2500/2600 ADC Driver");
MODULE_LICENSE("GPL");
