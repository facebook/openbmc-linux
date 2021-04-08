// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for Infineon Multi-phase Digital VR Controllers
 *
 * Copyright (c) 2020 Mellanox Technologies. All rights reserved.
 */

#include <linux/err.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/pmbus.h>
#include "pmbus.h"

#define XDPE132_PAGE_NUM		2

/*
 * Convert xdpe132 output voltage from LINEAR16 format to
 * show a real world value in milli-unit scale.
 */
static ssize_t xdpe132_vout_value_show_page(struct i2c_client *client,
					     int page,
					     char *buf)
{
	s8 exponent;
	long val;
	int ret;

	ret = pmbus_read_word_data(client, page, 0xff, PMBUS_READ_VOUT);
	if (ret < 0)
		return ret;

	/* scale output voltage to milli-units( mU ), which is milli-volts( mV )
	 * in this case.
	 */
	val = ((u16) ret)  * 1000L;

	ret = pmbus_read_byte_data(client, page, PMBUS_VOUT_MODE);
	if (ret < 0)
		return ret;

	exponent = ((s8)(ret << 3)) >> 3;

	if (exponent >= 0)
		val <<= exponent;
	else
		val >>= -exponent;

	return sprintf(buf, "%d\n", val);
}

static ssize_t xdpe132_vout_value_store_page(struct i2c_client *client,
					      int page,
					      const char *buf, size_t count)
{

	u8 vout_mode;
	s8 exponent;
	u16 mantissa;
	int ret, vout_value;
	long val;

	ret = kstrtoint(buf, 10, &vout_value);
	if (ret)
		return ret;

	val = vout_value;

	ret = pmbus_read_byte_data(client, page, PMBUS_VOUT_MODE);
	if (ret < 0)
		return ret;

	vout_mode  = ret & GENMASK(7, 5);
	/* Support only LINEAR mode */
	if (vout_mode > 0)
		return -ENODEV;

	/* Convert 2's complement exponent */
	exponent = ((s8)(ret << 3)) >> 3;

	/* Convert a real world value to LINEAR16 Format */
	if (exponent >= 0)
		val >>= exponent;
	else
		val <<= -exponent;

	/* Convert mantissa from milli-units to units */
	mantissa = (u16) DIV_ROUND_CLOSEST(val, 1000L);

	ret = pmbus_write_word_data(client, page, PMBUS_VOUT_COMMAND,
					   mantissa);

	return (ret < 0) ? ret : count;
}

static ssize_t xdpe132_vout_value_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);

	return xdpe132_vout_value_show_page(client, attr->index, buf);
}

static ssize_t xdpe132_vout_value_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);

	return xdpe132_vout_value_store_page(client, attr->index, buf, count);
}

static SENSOR_DEVICE_ATTR_RW(vout0_value, xdpe132_vout_value, 0);  //PAGE 0
static SENSOR_DEVICE_ATTR_RW(vout1_value, xdpe132_vout_value, 1);  //PAGE 1

static struct attribute *vout_value_attrs[] = {
	&sensor_dev_attr_vout0_value.dev_attr.attr,
	&sensor_dev_attr_vout1_value.dev_attr.attr,
	NULL,
};

static const struct attribute_group vout_value_group = {
	.attrs = vout_value_attrs,
};

static const struct attribute_group *xdpe132_attribute_groups[] = {
	&vout_value_group,
	NULL,
};

static struct pmbus_driver_info xdpe132_info = {
	.pages = XDPE132_PAGE_NUM,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
		PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
		PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.groups = xdpe132_attribute_groups,
};

static int xdpe132_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;

	info = devm_kmemdup(&client->dev, &xdpe132_info, sizeof(*info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id xdpe132_id[] = {
	{"xdpe132g5c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, xdpe132_id);

static const struct of_device_id __maybe_unused xdpe132_of_match[] = {
	{.compatible = "infineon,xdpe132g5c"},
	{}
};
MODULE_DEVICE_TABLE(of, xdpe132_of_match);

static struct i2c_driver xdpe132_driver = {
	.driver = {
		.name = "xdpe132g5c",
		.of_match_table = of_match_ptr(xdpe132_of_match),
	},
	.probe_new = xdpe132_probe,
	.remove = pmbus_do_remove,
	.id_table = xdpe132_id,
};

module_i2c_driver(xdpe132_driver);

MODULE_AUTHOR("Facebook/Celestica");
MODULE_DESCRIPTION("PMBus driver for Infineon XDPE132 family");
MODULE_LICENSE("GPL");
