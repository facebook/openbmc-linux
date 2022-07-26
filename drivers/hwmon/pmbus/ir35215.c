// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon IR35215
 *
 * Copyright (c) 2020 Facebook Inc.
 */

#include <linux/err.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

/*
 * Convert ir35215 output voltage from LINEAR16 format to
 * show a real world value in milli-unit scale.
 */
static ssize_t ir35215_vout_value_show_page(struct i2c_client *client, int page,
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
	val = ((u16)ret) * 1000L;

	ret = pmbus_read_byte_data(client, page, PMBUS_VOUT_MODE);
	if (ret < 0)
		return ret;

	exponent = ((s8)(ret << 3)) >> 3;

	if (exponent >= 0)
		val <<= exponent;
	else
		val >>= -exponent;

	return sprintf(buf, "%d\n", (u16)val);
}

static ssize_t ir35215_vout_value_store_page(struct i2c_client *client,
					     int page, const char *buf,
					     size_t count)
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

	vout_mode = ret & GENMASK(7, 5);
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
	mantissa = (u16)DIV_ROUND_CLOSEST(val, 1000L);

	ret = pmbus_write_word_data(client, page, PMBUS_VOUT_COMMAND, mantissa);

	return (ret < 0) ? ret : count;
}

static ssize_t ir35215_vout_value_show(struct device *dev,
				       struct device_attribute *devattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);

	return ir35215_vout_value_show_page(client, attr->index, buf);
}

static ssize_t ir35215_vout_value_store(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);

	return ir35215_vout_value_store_page(client, attr->index, buf, count);
}

static SENSOR_DEVICE_ATTR_RW(vout0_value, ir35215_vout_value, 0); //PAGE 0
static SENSOR_DEVICE_ATTR_RW(vout1_value, ir35215_vout_value, 1); //PAGE 1

static struct attribute *vout_value_attrs[] = {
	&sensor_dev_attr_vout0_value.dev_attr.attr,
	&sensor_dev_attr_vout1_value.dev_attr.attr,
	NULL,
};

static const struct attribute_group vout_value_group = {
	.attrs = vout_value_attrs,
};

static const struct attribute_group *ir35215_attribute_groups[] = {
	&vout_value_group,
	NULL,
};

static struct pmbus_driver_info ir35215_info = {
	.pages = 2,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_TEMP | PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN |
		   PMBUS_HAVE_POUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
		   PMBUS_HAVE_STATUS_TEMP,
	.func[1] = PMBUS_HAVE_TEMP | PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN |
		   PMBUS_HAVE_POUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
		   PMBUS_HAVE_STATUS_TEMP,
	.groups = ir35215_attribute_groups,
};

static int ir35215_probe(struct i2c_client *client)
{
	/*
	 * IR35215 devices may not stay in page 0 during device
	 * probe which leads to probe failure (read status word failed).
	 * So let's set the device to page 0 at the beginning.
	 */
	i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);
	return pmbus_do_probe(client, &ir35215_info);
}

static const struct i2c_device_id ir35215_id[] = { { "ir35215", 0 }, {} };

MODULE_DEVICE_TABLE(i2c, ir35215_id);

static struct i2c_driver ir35215_driver = {
	.driver = {
		   .name = "ir35215",
		   },
	.probe_new = ir35215_probe,
	.id_table = ir35215_id,
};

module_i2c_driver(ir35215_driver);

MODULE_AUTHOR("Tao Ren <rentao.bupt@gmail.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon IR35215");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
