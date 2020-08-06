// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for MAX127.
 *
 * Copyright (c) 2020 Facebook Inc.
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysfs.h>

/* MAX127 Control Byte. */
#define MAX127_CTRL_START	BIT(7)
#define MAX127_CTRL_SEL_SHIFT	4
#define MAX127_CTRL_RNG		BIT(3)
#define MAX127_CTRL_BIP		BIT(2)
#define MAX127_CTRL_PD1		BIT(1)
#define MAX127_CTRL_PD0		BIT(0)

#define MAX127_SET_CHANNEL(ch)	(((ch) & 7) << (MAX127_CTRL_SEL_SHIFT))

/*
 * MAX127 returns 2 bytes at read:
 *   - the first byte contains data[11:4].
 *   - the second byte contains data[3:0] (MSB) and 4 dummy 0s (LSB).
 */
#define MAX127_DATA1_SHIFT	4

struct max127_data {
	struct i2c_client *client;
	u8 ctrl_mask;
};

static int max127_select_channel(struct max127_data *data, int channel)
{
	int status;
	u8 ctrl_byte;
	struct i2c_msg msg;
	struct i2c_client *client = data->client;

	ctrl_byte = data->ctrl_mask | MAX127_SET_CHANNEL(channel);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = &ctrl_byte;
	msg.len = 1;

	status = i2c_transfer(client->adapter, &msg, 1);
	if (status != 1)
		return status;

	return 0;
}

static int max127_read_channel(struct max127_data *data, int channel, u16 *vin)
{
	int status;
	u8 i2c_data[2];
	struct i2c_msg msg;
	struct i2c_client *client = data->client;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.buf = i2c_data;
	msg.len = 2;

	status = i2c_transfer(client->adapter, &msg, 1);
	if (status != 1)
		return status;

	*vin = ((i2c_data[0] << 8) | i2c_data[1]) >> MAX127_DATA1_SHIFT;
	return 0;
}

static ssize_t max127_voltage_show(struct device *dev,
				   struct device_attribute *dev_attr,
				   char *buf)
{
	u16 vin;
	int status;
	struct max127_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	status = max127_select_channel(data, attr->index);
	if (status < 0)
		return status;

	status = max127_read_channel(data, attr->index, &vin);
	if (status < 0)
		return status;

	return sprintf(buf, "%u", vin);
}

static SENSOR_DEVICE_ATTR_RO(in0_input, max127_voltage, 0);
static SENSOR_DEVICE_ATTR_RO(in1_input, max127_voltage, 1);
static SENSOR_DEVICE_ATTR_RO(in2_input, max127_voltage, 2);
static SENSOR_DEVICE_ATTR_RO(in3_input, max127_voltage, 3);
static SENSOR_DEVICE_ATTR_RO(in4_input, max127_voltage, 4);
static SENSOR_DEVICE_ATTR_RO(in5_input, max127_voltage, 5);
static SENSOR_DEVICE_ATTR_RO(in6_input, max127_voltage, 6);
static SENSOR_DEVICE_ATTR_RO(in7_input, max127_voltage, 7);

static struct attribute *max127_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(max127);

static const struct attribute_group max127_attr_groups = {
	.attrs = max127_attrs,
};

static int max127_probe(struct i2c_client *client,
		        const struct i2c_device_id *id)
{
	struct device *hwmon_dev;
	struct max127_data *data;
	struct device *dev = &client->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->ctrl_mask = MAX127_CTRL_START | MAX127_CTRL_RNG;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
				client->name, data, max127_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id max127_id[] = {
	{ "max127", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max127_id);

static struct i2c_driver max127_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "max127",
	},
	.probe		= max127_probe,
	.id_table	= max127_id,
};

module_i2c_driver(max127_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Choi <mikechoi@fb.com>");
MODULE_AUTHOR("Tao Ren <taoren@fb.com>");
MODULE_DESCRIPTION("MAX127 Hardware Monitoring driver");
