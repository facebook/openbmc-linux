// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for MPS MP2978
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

static ssize_t mp2978_vout_show(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int val = pmbus_read_word_data(client, attr->index, PMBUS_READ_VOUT);
	return sprintf(buf, "%d\n", val);
}

static ssize_t mp2978_curr_out_show(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int val = pmbus_read_word_data(client, attr->index, PMBUS_READ_IOUT);
	return sprintf(buf, "%d\n", val * 1000);
}

static ssize_t mp2978_vout_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	return -1;
}

static ssize_t mp2978_curr_out_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	return -1;
}

/*
 * There are two rail voltage need monitored by MP2975/MP2975, rail 1 can read
 * from Page0 and rail 2 Rail 2 need read from Page1.
 * But feedback from MPS FAE the MP2978/2975 chip have two issues:
 * 1. page0 vout mode is VID, but the vout_mode of page1 default is linear.
 * 2. page0 vout mode is VID, but vout vout report mode is direct format.
 */
static SENSOR_DEVICE_ATTR_RW(in2_input, mp2978_vout, 0);
static SENSOR_DEVICE_ATTR_RW(in3_input, mp2978_vout, 1);
static SENSOR_DEVICE_ATTR_RW(curr3_input, mp2978_curr_out, 1);

static struct attribute *enable_attrs[] = {
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_curr3_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group enable_group = {
	.attrs = enable_attrs,
};

static const struct attribute_group *attribute_groups[] = {
	&enable_group,
	NULL,
};

/*
 * MP2978/MP2975 support input voltage, output voltage, output current, temperature
 * monitor.
 */
static struct pmbus_driver_info mp2978_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = vid,
	.vrm_version[0] = vr12,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_TEMP
		| PMBUS_HAVE_VIN
		| PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT
		| PMBUS_HAVE_STATUS_IOUT
		| PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_STATUS_TEMP,
	.groups = attribute_groups,
};

static int mp2978_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rv = 0;
	const u8 databuf[2] = {0x0, 0x0};
	/*
	 * fix sometime after MCU warm reboot mp2978/2975 will failed to identify chip
	 * capabilities.
	 * root cause: after MCU reboot, the MP2978/2975 may still stag in page1
	 */
	rv = i2c_master_send(client, databuf, sizeof(databuf));
	if (rv < 0)
		return rv;

	return pmbus_do_probe(client, id, &mp2978_info);
}

static const struct i2c_device_id mp2978_id[] = {
	{"mp2978", 0},
	{"mp2975", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mp2978_id);

static struct i2c_driver mp2978_driver = {
	.driver = {
		   .name = "mp2978",
		   },
	.probe = mp2978_probe,
	.remove = pmbus_do_remove,
	.id_table = mp2978_id,
};

module_i2c_driver(mp2978_driver);

MODULE_AUTHOR("Facebook/Celestica");
MODULE_DESCRIPTION("PMBus driver for MPS MP2978/MP2975");
MODULE_LICENSE("GPL");
