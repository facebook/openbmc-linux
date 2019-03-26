/* SPDX-License-Identifier: GPL-2.0
 *
 * Texas Instruments TLA2021/TLA2022/TLA2024 12-bit ADC driver
 *
 * Copyright (C) 2019 Koninklijke Philips N.V.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>

#define TLA2024_DATA 0x00
#define TLA2024_DATA_RES_MASK GENMASK(15, 4)
#define TLA2024_DATA_RES_SHIFT 4

#define TLA2024_CONF 0x01
#define TLA2024_CONF_OS_MASK BIT(15)
#define TLA2024_CONF_OS_SHIFT 15
#define TLA2024_CONF_MUX_MASK GENMASK(14, 12)
#define TLA2024_CONF_MUX_SHIFT 12
#define TLA2024_CONF_PGA_MASK GENMASK(11, 9)
#define TLA2024_CONF_PGA_SHIFT 9
#define TLA2024_CONF_MODE_MASK BIT(8)
#define TLA2024_CONF_MODE_SHIFT 8
#define TLA2024_CONF_DR_MASK GENMASK(7, 5)
#define TLA2024_CONF_DR_SHIFT 5

#define TLA2024_CONV_RETRY 10

struct tla202x_model {
	unsigned int mux_available;
	unsigned int pga_available;
};

struct tla2024 {
	struct i2c_client *i2c;
	struct tla202x_model *model;
	struct mutex lock;
	u8 used_mux_channels;
};

struct tla2024_channel {
	int ainp;
	int ainn;
	const char *datasheet_name;
	bool differential;
};

static const struct tla2024_channel tla2024_all_channels[] = {
	{0, 1, "AIN0-AIN1", 1},
	{0, 3, "AIN0-AIN3", 1},
	{1, 3, "AIN1-AIN3", 1},
	{2, 3, "AIN2-AIN3", 1},
	{0, -1, "AIN0", 0},
	{1, -1, "AIN1", 0},
	{2, -1, "AIN2", 0},
	{3, -1, "AIN3", 0},
};

static int tla2024_find_chan_idx(u32 ainp_in, u32 ainn_in, u16 *idx)
{
	u16 i;

	for (i = 0; i < ARRAY_SIZE(tla2024_all_channels); i++) {
		if ((tla2024_all_channels[i].ainp == ainp_in) &&
		    (tla2024_all_channels[i].ainn == ainn_in)) {
			*idx = i;
			return 0;
		}
	}

	return -EINVAL;
}

#define TLA202x_MODEL(_mux, _pga)		\
	{					\
		.mux_available = (_mux),	\
		.pga_available = (_pga),	\
	}

enum tla2024_model_id {
	TLA2021 = 0,
	TLA2022 = 1,
	TLA2024 = 2,
};

static struct tla202x_model tla202x_models[] = {
	[TLA2021] = TLA202x_MODEL(0, 0),
	[TLA2022] = TLA202x_MODEL(0, 1),
	[TLA2024] = TLA202x_MODEL(1, 1),
};

static const int tla2024_scale[] = { /* scale, int plus micro */
	3, 0, 2, 0, 1, 0, 0, 500000, 0, 250000, 0, 125000 };

static int tla2024_get(struct tla2024 *priv, u8 addr, u16 mask,
		       u16 shift, u16 *val)
{
	int ret;
	u16 data;

	ret = i2c_smbus_read_word_swapped(priv->i2c, addr);
	if (ret < 0)
		return ret;

	data = ret;
	*val = (mask & data) >> shift;

	return 0;
}

static int tla2024_set(struct tla2024 *priv, u8 addr, u16 mask,
		       u16 shift, u16 val)
{
	int ret;
	u16 data;

	ret = i2c_smbus_read_word_swapped(priv->i2c, addr);
	if (ret < 0)
		return ret;

	data = ret;
	data &= ~mask;
	data |= mask & (val << shift);

	ret = i2c_smbus_write_word_swapped(priv->i2c, addr, data);

	return ret;
}

static int tla2024_read_avail(struct iio_dev *idev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:

		*length = ARRAY_SIZE(tla2024_scale);
		*vals = tla2024_scale;

		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}

static int tla2024_of_find_chan(struct tla2024 *priv, struct device_node *ch)
{
	u16 chan_idx = 0;
	u32 ainp, ainn;
	int ret;

	ret = of_property_read_u32_index(ch, "single-channel", 0, &ainp);
	if (ret) {
		ret = of_property_read_u32_index(ch,
						 "diff-channels", 0, &ainp);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(ch,
						 "diff-channels", 1, &ainn);
		if (ret)
			return ret;

	} else {
		ainn = UINT_MAX;
	}

	ret = tla2024_find_chan_idx(ainp, ainn, &chan_idx);
	if (ret < 0)
		return ret;

	/* if model doesn"t have mux then only channel 0 is allowed */
	if (!priv->model->mux_available && chan_idx)
		return -EINVAL;

	/* if already used */
	if ((priv->used_mux_channels) & (1 << chan_idx))
		return -EINVAL;

	return chan_idx;
}

static int tla2024_init_chan(struct iio_dev *idev, struct device_node *node,
			     struct iio_chan_spec *chan)
{
	struct tla2024 *priv = iio_priv(idev);
	u16 chan_idx;
	int ret;

	ret = tla2024_of_find_chan(priv, node);
	if (ret < 0)
		return ret;

	chan_idx = ret;
	priv->used_mux_channels |= BIT(chan_idx);
	chan->type = IIO_VOLTAGE;
	chan->channel = tla2024_all_channels[chan_idx].ainp;
	chan->channel2 = tla2024_all_channels[chan_idx].ainn;
	chan->differential = tla2024_all_channels[chan_idx].differential;
	chan->datasheet_name = tla2024_all_channels[chan_idx].datasheet_name;
	chan->indexed = 1;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE);
	chan->info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SCALE);

	return 0;
}

static int tla2024_wait(struct tla2024 *priv)
{
	int ret;
	unsigned int retry = TLA2024_CONV_RETRY;
	u16 status;

	do {
		if (!--retry)
			return -EIO;
		ret = tla2024_get(priv, TLA2024_CONF, TLA2024_CONF_OS_MASK,
				  TLA2024_CONF_OS_SHIFT, &status);
		if (ret < 0)
			return ret;
		if (!status)
			usleep_range(25, 1000);
	} while (!status);

	return ret;
}

static int tla2024_singleshot_conv(struct tla2024 *priv,
				   struct iio_chan_spec const *chan, int *val)
{
	int ret;
	u32 ainp = chan->channel;
	u32 ainn = chan->channel2;
	u16 chan_id, data;
	s16 tmp;

	ret = tla2024_set(priv, TLA2024_CONF, TLA2024_CONF_MODE_MASK,
			  TLA2024_CONF_MODE_SHIFT, 1);
	if (ret < 0)
		return ret;

	ret = tla2024_find_chan_idx(ainp, ainn, &chan_id);
	if (ret < 0)
		return ret;

	ret = tla2024_set(priv, TLA2024_CONF, TLA2024_CONF_MUX_MASK,
			  TLA2024_CONF_MUX_SHIFT, chan_id);
	if (ret < 0)
		return ret;

	ret = tla2024_set(priv, TLA2024_CONF, TLA2024_CONF_OS_MASK,
			  TLA2024_CONF_OS_SHIFT, 1);
	if (ret < 0)
		return ret;

	ret = tla2024_wait(priv);
	if (ret < 0)
		return ret;

	ret = tla2024_get(priv, TLA2024_DATA, TLA2024_DATA_RES_MASK,
			  TLA2024_DATA_RES_SHIFT, &data);
	if (ret < 0)
		return ret;

	tmp = (s16)(data << TLA2024_DATA_RES_SHIFT);
	*val = tmp >> TLA2024_DATA_RES_SHIFT;
	return IIO_VAL_INT;
}

static int tla2024_set_scale(struct tla2024 *priv, int val, int val2)
{
	int i;

	if (!(priv->model->pga_available))
		return -EINVAL; /* scale can't be changed if no pga */

	for (i = 0; i < ARRAY_SIZE(tla2024_scale); i = i + 2) {
		if ((tla2024_scale[i] == val) &&
		    (tla2024_scale[i + 1] == val2))
			break;
	}

	if (i == ARRAY_SIZE(tla2024_scale))
		return -EINVAL;

	return tla2024_set(priv, TLA2024_CONF, TLA2024_CONF_PGA_MASK,
			   TLA2024_CONF_PGA_SHIFT, i >> 1);
}

static int tla2024_get_scale(struct tla2024 *priv, int *val, int *val2)
{
	u16 data;
	int ret;

	if (!(priv->model->pga_available)) {
		*val = 1; /* Scale always 1 mV when no PGA */
		return IIO_VAL_INT;
	}
	ret = tla2024_get(priv, TLA2024_CONF, TLA2024_CONF_PGA_MASK,
			  TLA2024_CONF_PGA_SHIFT, &data);
	if (ret < 0)
		return ret;

	/* gain for the 3bit pga values 6 and 7 is same as at 5 */
	if (data >= (ARRAY_SIZE(tla2024_scale) >> 1))
		data = (ARRAY_SIZE(tla2024_scale) >> 1) - 1;

	*val = tla2024_scale[data << 1];
	*val2 = tla2024_scale[(data << 1) + 1];
	return IIO_VAL_INT_PLUS_MICRO;
}

static int tla2024_read_raw(struct iio_dev *idev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct tla2024 *priv = iio_priv(idev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&priv->lock);
		ret = tla2024_singleshot_conv(priv, channel, val);
		mutex_unlock(&priv->lock);
		return ret;

	case IIO_CHAN_INFO_SCALE:
		return tla2024_get_scale(priv, val, val2);

	default:
		return -EINVAL;
	}
}

static int tla2024_write_raw(struct iio_dev *idev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tla2024 *priv = iio_priv(idev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&priv->lock);
		ret = tla2024_set_scale(priv, val, val2);
		mutex_unlock(&priv->lock);
		return ret;
	}

	return -EINVAL;
}

static int tla2024_of_chan_init(struct iio_dev *idev)
{
	struct device_node *node = idev->dev.of_node;
	struct device_node *child;
	struct iio_chan_spec *channels;
	int ret, i, num_channels;

	num_channels = of_get_available_child_count(node);
	if (!num_channels) {
		dev_err(&idev->dev, "no channels configured\n");
		return -ENODEV;
	}

	channels = devm_kcalloc(&idev->dev, num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	i = 0;
	for_each_available_child_of_node(node, child) {
		ret = tla2024_init_chan(idev, child, &channels[i]);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		i++;
	}

	idev->channels = channels;
	idev->num_channels = num_channels;

	return 0;
}

static const struct iio_info tla2024_info = {
	.read_raw = tla2024_read_raw,
	.write_raw = tla2024_write_raw,
	.read_avail = tla2024_read_avail,
};

static int tla2024_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct tla2024 *priv;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	iio = devm_iio_device_alloc(&client->dev, sizeof(*priv));
	if (!iio)
		return -ENOMEM;

	priv = iio_priv(iio);
	priv->i2c = client;
	priv->model = &tla202x_models[id->driver_data];
	mutex_init(&priv->lock);

	iio->dev.parent = &client->dev;
	iio->dev.of_node = client->dev.of_node;
	iio->name = client->name;
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &tla2024_info;

	ret = tla2024_of_chan_init(iio);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&client->dev, iio);
}

static const struct i2c_device_id tla2024_id[] = {
	{ "tla2024", TLA2024 },
	{ "tla2022", TLA2022 },
	{ "tla2021", TLA2021 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tla2024_id);

static const struct of_device_id tla2024_of_match[] = {
	{ .compatible = "ti,tla2024" },
	{ .compatible = "ti,tla2022" },
	{ .compatible = "ti,tla2021" },
	{ }
};
MODULE_DEVICE_TABLE(of, tla2024_of_match);

static struct i2c_driver tla2024_driver = {
	.driver = {
		.name = "tla2024",
		.of_match_table = of_match_ptr(tla2024_of_match),
	},
	.probe = tla2024_probe,
	.id_table = tla2024_id,
};
module_i2c_driver(tla2024_driver);

MODULE_AUTHOR("Ibtsam Haq <ibtsam.haq@philips.com>");
MODULE_DESCRIPTION("Texas Instruments TLA2021/TLA2022/TLA2024 ADC driver");
MODULE_LICENSE("GPL v2");
