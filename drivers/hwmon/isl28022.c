// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Renesas ISL28022 power monitor chip
 *
 * Copyright (C) 2023 PeterYin <peter.yin@quantatw.com>
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* ISL28022 register definitions */
#define ISL28022_CONFIG				0x0
#define ISL28022_SHUNT_VOLTAGE			0x1
#define ISL28022_BUS_VOLTAGE			0x2
#define ISL28022_POWER				0x3
#define ISL28022_CURRENT			0x4
#define ISL28022_SHUNT_CALIBRATION		0x5
#define ISL28022_SHUNT_THRESHOLD_VOLTAGE	0x6
#define ISL28022_BUS_THRESHOLD_VOLTAGE		0x7

#define ISL28022_CONFIG_ADCRANGE(x)		(x << 11)
#define ISL28022_CONFIG_ADCRANG_MASK		(3 << 11)


#define ISL28022_REGISTERS			0x9

#define ISL28022_RSHUNT_DEFAULT			1 /* 1mOhm  */

#define ISL28022_ADC_CONFIG_DEFAULT		0x799F
#define ISL28022_CALIBRATION_VALUE		0x1062

#define ISL28022_POWER_SCALE			5000
#define ISL28022_SHUNT_VOLTAGE_LSB		10 /* 10 uV/lsb */
#define ISL28022_BUS_VOLTAGE_LSB		4 /* 4 mV/lsb */

#define ISL28022_VOLTAGE_THRESHOLD_LSB		2560 /* shunt:2.56 mV/lsb, bus:256mv/lsb */
#define ISL28022_BASE_SHUNT_FULL_SCALE		40 /* 40mv */
#define ISL28022_DEFAULT_SADC_SAMPLE		32768


struct isl28022_platform_data {
	long shunt_uohms;
};

static struct regmap_config isl28022_regmap_config = {
	.max_register = ISL28022_REGISTERS,
	.reg_bits = 8,
	.val_bits = 16,
};

struct isl28022_data {
	struct i2c_client *client;
	struct mutex config_lock;
	struct regmap *regmap;
	u32 rshunt;
	int pg;
	int div_gain;
	int current_lsb;
	int power_lsb;
	int shunt_fs;
	int calibration;
};

static int isl28022_read_in(struct device *dev, u32 attr, int channel,
			  long *val)
{
	struct isl28022_data *data = dev_get_drvdata(dev);
	int reg;
	int regval;
	int err;

	switch (channel) {
	case 0:
		switch (attr) {
		case hwmon_in_input:
			reg = ISL28022_SHUNT_VOLTAGE;
			break;
		case hwmon_in_max:
			reg = ISL28022_SHUNT_THRESHOLD_VOLTAGE;
			break;
		case hwmon_in_min:
			reg = ISL28022_SHUNT_THRESHOLD_VOLTAGE;
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	case 1:
		switch (attr) {
		case hwmon_in_input:
			reg = ISL28022_BUS_VOLTAGE;
			break;
		case hwmon_in_max:
			reg = ISL28022_BUS_THRESHOLD_VOLTAGE;
			break;
		case hwmon_in_min:
			reg = ISL28022_BUS_THRESHOLD_VOLTAGE;
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	err = regmap_read(data->regmap, reg, &regval);
	if (err < 0)
		return err;

	switch (attr) {
	case hwmon_in_input:
		if (channel == 0) {
			*val = ((s16)regval * ISL28022_SHUNT_VOLTAGE_LSB) /
			 (1000*data->div_gain);
		}
		if (channel == 1) {
			*val = ((u16)regval >> 2) * ISL28022_BUS_VOLTAGE_LSB;
		}
		break;
	case hwmon_in_max:
		/* signed register, value in mV */
		regval = (s8)(regval >> 8);
		*val = (regval * ISL28022_VOLTAGE_THRESHOLD_LSB) /
		       (1000 * data->div_gain);
		break;
	case hwmon_in_min:
		/* signed register, value in mV */
		regval = (s8)(regval);
		*val = (regval * ISL28022_VOLTAGE_THRESHOLD_LSB) /
		       (1000 * data->div_gain);
		break;
	}
	return 0;
}

static int isl28022_write_in(struct device *dev, u32 attr, int channel,
			   long val)
{
	struct isl28022_data *data = dev_get_drvdata(dev);
	int regval;
	int rd_val;

	if (attr != hwmon_in_max && attr != hwmon_in_min)
		return -EOPNOTSUPP;

	/* convert decimal to register value */
	switch (channel) {
	case 0:
		/* signed value, clamp to max range +/-320 mV */
		regval = clamp_val(val, -320, 320);
		regval = (regval * 1000 ) / ISL28022_VOLTAGE_THRESHOLD_LSB / data->div_gain;
		regval = clamp_val(regval, S16_MIN, S16_MAX);

		regmap_read(data->regmap, ISL28022_SHUNT_THRESHOLD_VOLTAGE, &rd_val);

		switch (attr) {
		case hwmon_in_max:
			rd_val &= ~0xFF00;
			rd_val |= (regval << 8);
			return regmap_write(data->regmap,
				ISL28022_SHUNT_THRESHOLD_VOLTAGE, rd_val);
		case hwmon_in_min:
			rd_val &= ~0x00FF;
			rd_val |= regval;
			return regmap_write(data->regmap,
					    ISL28022_SHUNT_THRESHOLD_VOLTAGE, rd_val);
		default:
			return -EOPNOTSUPP;
		}
	case 1:
		/* signed value, positive values only. Clamp to max 60 V */
		regval = clamp_val(val, 0, 60000);
		regval = regval / ISL28022_VOLTAGE_THRESHOLD_LSB;

		regmap_read(data->regmap, ISL28022_SHUNT_THRESHOLD_VOLTAGE, &rd_val);
		switch (attr) {
		case hwmon_in_max:
			rd_val &= ~0xFF00;
			rd_val |= (regval << 8);
			return regmap_write(data->regmap,
					    ISL28022_BUS_THRESHOLD_VOLTAGE, rd_val);
		case hwmon_in_min:
			rd_val &= ~0x00FF;
			rd_val |= regval;
			return regmap_write(data->regmap,
					    ISL28022_BUS_THRESHOLD_VOLTAGE, rd_val);
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}

}

static int isl28022_read_current(struct device *dev, u32 attr, long *val)
{
	struct isl28022_data *data = dev_get_drvdata(dev);
	int regval;
	int err;

	switch (attr) {
	case hwmon_curr_input:
		err = regmap_read(data->regmap, ISL28022_CURRENT, &regval);
		if (err < 0)
			return err;

		*val = (s16)regval * data->current_lsb / 1000; /*mA*/
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int isl28022_read_power(struct device *dev, u32 attr, long *val)
{
	struct isl28022_data *data = dev_get_drvdata(dev);
	int regval;
	int err;

	switch (attr) {
	case hwmon_power_input:
		err = regmap_read(data->regmap, ISL28022_POWER, &regval);
		if (err < 0)
			return err;

		*val = regval * data->power_lsb; /* uw */
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int isl28022_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_in:
		return isl28022_read_in(dev, attr, channel, val);
	case hwmon_curr:
		return isl28022_read_current(dev, attr, val);
	case hwmon_power:
		return isl28022_read_power(dev, attr, val);
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int isl28022_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct isl28022_data *data = dev_get_drvdata(dev);
	int err;

	mutex_lock(&data->config_lock);

	switch (type) {
	case hwmon_in:
		err = isl28022_write_in(dev, attr, channel, val);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&data->config_lock);
	return err;
}

static umode_t isl28022_is_visible(const void *drvdata,
				 enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		case hwmon_in_max:
		case hwmon_in_min:
			return 0644;
		default:
			return 0;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			return 0444;
		default:
			return 0;
		}
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

#define ISL28022_HWMON_IN_CONFIG (HWMON_I_INPUT | \
				HWMON_I_MAX | HWMON_I_MIN)

static const struct hwmon_channel_info *isl28022_info[] = {
	HWMON_CHANNEL_INFO(in,
			   /* 0: shunt voltage */
			   ISL28022_HWMON_IN_CONFIG,
			   /* 1: bus voltage */
			   ISL28022_HWMON_IN_CONFIG),
	HWMON_CHANNEL_INFO(curr,
			   /* 0: current through shunt */
			   HWMON_C_INPUT),
	HWMON_CHANNEL_INFO(power,
			   /* 0: power */
			   HWMON_P_INPUT),
	NULL
};

static const struct hwmon_ops isl28022_hwmon_ops = {
	.is_visible = isl28022_is_visible,
	.read = isl28022_read,
	.write = isl28022_write,
};

static const struct hwmon_chip_info isl28022_chip_info = {
	.ops = &isl28022_hwmon_ops,
	.info = isl28022_info,
};

static int isl28022_probe(struct i2c_client *client)
{
	struct isl28022_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct isl28022_data *data;
	int config;
	int ret;
	int regval;
	int err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->config_lock);

	data->regmap = devm_regmap_init_i2c(client, &isl28022_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	/* load shunt value */
	data->rshunt = ISL28022_RSHUNT_DEFAULT;
	if (device_property_read_u32(dev, "shunt-resistor", &data->rshunt) < 0 && pdata)
		data->rshunt = pdata->shunt_uohms;
	if (data->rshunt == 0) {
		dev_err(dev, "invalid shunt resister value %u\n", data->rshunt);
		return -EINVAL;
	}

	/* load shunt gain value 0 - 3 */
	if (device_property_read_u32(dev, "isl,shunt-pg", &data->pg) < 0) {
		data->pg = 3; /* Default of ADCRANGE = 11 */
	}
	if (data->pg > 4 || data->pg < 0) {
		dev_err(dev, "invalid shunt gain value %u\n", data->pg);
		return -EINVAL;
	}
	/* Setup CONFIG register */
	err = regmap_read(data->regmap, ISL28022_CONFIG, &regval);
	if (err < 0)
		return err;

	data->div_gain = 8 / (2 << data->pg);
	/* Setup ADC_CONFIG register */
	config = ISL28022_ADC_CONFIG_DEFAULT;
	config &= ~ISL28022_CONFIG_ADCRANG_MASK;
	config |= ISL28022_CONFIG_ADCRANGE(data->pg); /* ADCRANGE: 00=/1, 01=/2, 10=/4, 11=/8*/
	ret = regmap_write(data->regmap, ISL28022_CONFIG, config);
	if (ret < 0) {
		dev_err(dev, "error configuring the device: %d\n", ret);
		return -ENODEV;
	}

	data->shunt_fs = ISL28022_BASE_SHUNT_FULL_SCALE * (2 << (data->pg-1)); /*mv */
	data->current_lsb = (data->shunt_fs * 1000 * 1000) / data->rshunt / ISL28022_DEFAULT_SADC_SAMPLE; /*nA*/
	data->power_lsb = data->current_lsb * (ISL28022_BUS_VOLTAGE_LSB * ISL28022_POWER_SCALE * 2 / 1000); /* mW */
	data->calibration = (32768 / data->shunt_fs) * (4096 * ISL28022_SHUNT_VOLTAGE_LSB)/1000;

	/* Setup SHUNT_CALIBRATION register with fixed value */
	ret = regmap_write(data->regmap, ISL28022_SHUNT_CALIBRATION, ISL28022_CALIBRATION_VALUE );
	if (ret < 0) {
		dev_err(dev, "error configuring the device: %d\n", ret);
		return -ENODEV;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, data,
							 &isl28022_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "power monitor %s (Rshunt = %u uOhm, pg = %u)\n",
		 client->name, data->rshunt, data->pg);

	return 0;
}

static const struct i2c_device_id isl28022_id[] = {
	{ "isl28022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl28022_id);

static const struct of_device_id __maybe_unused isl28022_of_match[] = {
	{ .compatible = "isl,isl28022" },
	{ },
};
MODULE_DEVICE_TABLE(of, isl28022_of_match);

static struct i2c_driver isl28022_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "isl28022",
		.of_match_table = of_match_ptr(isl28022_of_match),
	},
	.probe_new	= isl28022_probe,
	.id_table	= isl28022_id,
};
module_i2c_driver(isl28022_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("isl28022 driver");
MODULE_LICENSE("GPL");
