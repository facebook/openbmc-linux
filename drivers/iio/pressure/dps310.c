/*
 * Copyright 2017 IBM Corporation
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * The DPS310 is a barometric pressure and temperature sensor.
 * Currently only reading a single temperature is supported by
 * this driver.
 *
 * https://www.infineon.com/dgdl/?fileId=5546d462576f34750157750826c42242
 *
 * Temperature calculation:
 *   c0 * 0.5 + c1 * T_raw / kT °C
 *
 * TODO:
 *  - Pressure sensor readings
 *  - Optionally support the FIFO
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DPS310_PRS_B0		0x00
#define DPS310_PRS_B1		0x01
#define DPS310_PRS_B2		0x02
#define DPS310_TMP_B0		0x03
#define DPS310_TMP_B1		0x04
#define DPS310_TMP_B2		0x05
#define DPS310_PRS_CFG		0x06
#define DPS310_TMP_CFG		0x07
#define  DPS310_TMP_RATE_BITS	GENMASK(6, 4)
#define  DPS310_TMP_PRC_BITS	GENMASK(3, 0)
#define  DPS310_TMP_EXT		BIT(7)
#define DPS310_MEAS_CFG		0x08
#define  DPS310_MEAS_CTRL_BITS	GENMASK(2, 0)
#define   DPS310_PRESSURE_EN	BIT(0)
#define   DPS310_TEMP_EN	BIT(1)
#define   DPS310_BACKGROUND	BIT(2)
#define  DPS310_PRS_RDY		BIT(4)
#define  DPS310_TMP_RDY		BIT(5)
#define  DPS310_SENSOR_RDY	BIT(6)
#define  DPS310_COEF_RDY	BIT(7)
#define DPS310_CFG_REG		0x09
#define  DPS310_INT_HL		BIT(7)
#define  DPS310_TMP_SHIFT_EN	BIT(3)
#define  DPS310_PRS_SHIFT_EN	BIT(4)
#define  DPS310_FIFO_EN		BIT(5)
#define  DPS310_SPI_EN		BIT(6)
#define DPS310_RESET		0x0c
#define  DPS310_RESET_MAGIC	(BIT(0) | BIT(3))
#define DPS310_COEF_BASE	0x10

#define DPS310_PRS_BASE		DPS310_PRS_B0
#define DPS310_TMP_BASE		DPS310_TMP_B0

#define DPS310_TMP_RATE(_n)	ilog2(_n)
#define DPS310_TMP_PRC(_n)	ilog2(_n)

#define MCELSIUS_PER_CELSIUS	1000

const int scale_factor[] = {
	 524288,
	1572864,
	3670016,
	7864320,
	 253952,
	 516096,
	1040384,
	2088960,
};

struct dps310_data {
	struct i2c_client *client;
	struct regmap *regmap;

	s32 c0, c1;
	s32 temp_raw;
};

static const struct iio_chan_spec dps310_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_OFFSET) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_RAW),
	},
};

/* To be called after checking the TMP_RDY bit in MEAS_CFG */
static int dps310_get_temp_coef(struct dps310_data *data)
{
	struct regmap *regmap = data->regmap;
	uint8_t coef[3] = {0};
	int r;
	u32 c0, c1;

	/*
	 * Read temperature calibration coefficients c0 and c1 from the
	 * COEF register. The numbers are 12-bit 2's compliment numbers
	 */
	r = regmap_bulk_read(regmap, DPS310_COEF_BASE, coef, 3);
	if (r < 0)
		return r;

	c0 = (coef[0] << 4) | (coef[1] >> 4);
	data->c0 = sign_extend32(c0, 11);

	c1 = ((coef[1] & GENMASK(3, 0)) << 8) | coef[2];
	data->c1 = sign_extend32(c1, 11);

	return 0;
}

static int dps310_get_temp_precision(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_TMP_CFG, &val);
	if (r < 0)
		return r;

	/*
	 * Scale factor is bottom 4 bits of the register, but 1111 is
	 * reserved so just grab bottom three
	 */
	return BIT(val & GENMASK(2, 0));
}

static int dps310_set_temp_precision(struct dps310_data *data, int val)
{
	int ret;
	u8 shift_en;

	if (val < 0 || val > 128)
		return -EINVAL;

	shift_en = val >= 16 ? DPS310_TMP_SHIFT_EN : 0;
	ret = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			DPS310_TMP_SHIFT_EN,
			shift_en);
	if (ret)
		return ret;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
			DPS310_TMP_PRC_BITS, DPS310_TMP_PRC(val));
}

static int dps310_set_temp_samp_freq(struct dps310_data *data, int freq)
{
	uint8_t val;

	if (freq < 0 || freq > 128)
		return -EINVAL;

	val = DPS310_TMP_RATE(freq) << 4;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
			DPS310_TMP_RATE_BITS, val);
}

static int dps310_get_temp_samp_freq(struct dps310_data *data)
{
	int val, r;

	r = regmap_read(data->regmap, DPS310_TMP_CFG, &val);
	if (r < 0)
		return r;

	return BIT((val & DPS310_TMP_RATE_BITS) >> 4);
}

static int dps310_get_temp_k(struct dps310_data *data)
{
	return scale_factor[DPS310_TMP_PRC(dps310_get_temp_precision(data))];
}

static int dps310_read_temp(struct dps310_data *data)
{
	struct device *dev = &data->client->dev;
	struct regmap *regmap = data->regmap;
	uint8_t val[3] = {0};
	int r, ready;
	int T_raw;

	r = regmap_read(regmap, DPS310_MEAS_CFG, &ready);
	if (r < 0)
		return r;
	if (!(ready & DPS310_TMP_RDY)) {
		dev_dbg(dev, "temperature not ready\n");
		return -EAGAIN;
	}

	r = regmap_bulk_read(regmap, DPS310_TMP_BASE, val, 3);
	if (r < 0)
		return r;

	T_raw = (val[0] << 16) | (val[1] << 8) | val[2];
	data->temp_raw = sign_extend32(T_raw, 23);

	return 0;
}

static bool dps310_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_CFG:
	case DPS310_TMP_CFG:
	case DPS310_MEAS_CFG:
	case DPS310_CFG_REG:
	case DPS310_RESET:
	case 0x0e:
	case 0x0f:
	case 0x62:
		return true;
	default:
		return false;
	}
}

static bool dps310_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_B0:
	case DPS310_PRS_B1:
	case DPS310_PRS_B2:
	case DPS310_TMP_B0:
	case DPS310_TMP_B1:
	case DPS310_TMP_B2:
	case DPS310_MEAS_CFG:
	case 0x32:
		return true;
	default:
		return false;
	}
}

static int dps310_write_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);

	if (chan->type != IIO_TEMP)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return dps310_set_temp_samp_freq(data, val);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return dps310_set_temp_precision(data, val);
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int dps310_read_raw(struct iio_dev *iio,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = dps310_get_temp_samp_freq(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		ret = dps310_read_temp(data);
		if (ret)
			return ret;

		*val = data->temp_raw * data->c1;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OFFSET:
		*val = (data->c0 >> 1) * dps310_get_temp_k(data);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1000; /* milliCelsius per Celsius */
		*val2 = dps310_get_temp_k(data);
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = dps310_get_temp_precision(data);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static const struct regmap_config dps310_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = dps310_is_writeable_reg,
	.volatile_reg = dps310_is_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x62,
};

static const struct iio_info dps310_info = {
	.read_raw = dps310_read_raw,
	.write_raw = dps310_write_raw,
};

/*
 * Some verions of chip will read temperatures in the ~60C range when
 * its acutally ~20C. This is the manufacturer recommended workaround
 * to correct the issue.
 */
static int dps310_temp_workaround(struct dps310_data *data)
{
	int r, reg;

	r = regmap_read(data->regmap, 0x32, &reg);
	if (r < 0)
		return r;

	/* If bit 1 is set then the device is okay, and the workaround does not
	 * need to be applied */
	if (reg & BIT(1))
		return 0;

	r = regmap_write(data->regmap, 0x0e, 0xA5);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0f, 0x96);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x62, 0x02);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0e, 0x00);
	if (r < 0)
		return r;

	r = regmap_write(data->regmap, 0x0f, 0x00);

	return r;
}

static int dps310_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct dps310_data *data;
	struct iio_dev *iio;
	int r, ready;

	iio = devm_iio_device_alloc(&client->dev,  sizeof(*data));
	if (!iio)
		return -ENOMEM;

	data = iio_priv(iio);
	data->client = client;

	iio->dev.parent = &client->dev;
	iio->name = id->name;
	iio->channels = dps310_channels;
	iio->num_channels = ARRAY_SIZE(dps310_channels);
	iio->info = &dps310_info;
	iio->modes = INDIO_DIRECT_MODE;

	data->regmap = devm_regmap_init_i2c(client, &dps310_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/*
	 * Set up external (MEMS) temperature sensor in single sample, one
	 * measurement per second mode
	 */
	r = regmap_write(data->regmap, DPS310_TMP_CFG,
			DPS310_TMP_EXT | DPS310_TMP_RATE(1) | DPS310_TMP_PRC(1));
	if (r < 0)
		return r;

	/* Temp shift is disabled when PRC <= 8 */
	r = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			DPS310_TMP_SHIFT_EN, 0);
	if (r < 0)
		return r;

	/* Turn on temperature measurement in the background */
	r = regmap_write_bits(data->regmap, DPS310_MEAS_CFG,
			DPS310_MEAS_CTRL_BITS,
			DPS310_TEMP_EN | DPS310_BACKGROUND);
	if (r < 0)
		return r;

	/*
	 * Calibration coefficients required for reporting temperature.
	 * They are available 40ms after the device has started
	 */
	r = regmap_read_poll_timeout(data->regmap, DPS310_MEAS_CFG, ready,
			ready & DPS310_COEF_RDY,
			10 * 1000,
			40 * 1000);
	if (r < 0)
		return r;

	r = dps310_get_temp_coef(data);
	if (r < 0)
		return r;

	r = dps310_temp_workaround(data);
	if (r < 0)
		return r;

	r = devm_iio_device_register(&client->dev, iio);
	if (r)
		return r;

	i2c_set_clientdata(client, iio);

	dev_info(&client->dev, "%s: sensor '%s'\n", dev_name(&iio->dev),
			client->name);

	return 0;
}

static int dps310_remove(struct i2c_client *client)
{
	struct dps310_data *data = i2c_get_clientdata(client);

	return regmap_write(data->regmap, DPS310_RESET, DPS310_RESET_MAGIC);
}

static const struct i2c_device_id dps310_id[] = {
	{ "dps310", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, dps310_id);

static const unsigned short normal_i2c[] = {
	0x77, 0x76, I2C_CLIENT_END
};

static struct i2c_driver dps310_driver = {
	.driver = {
		.name = "dps310",
	},
	.probe = dps310_probe,
	.remove = dps310_remove,
	.address_list = normal_i2c,
	.id_table = dps310_id,
};
module_i2c_driver(dps310_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("Infineon DPS310 pressure and temperature sensor");
MODULE_LICENSE("GPL");
