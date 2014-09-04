/*
 * max127.c - Part of lm_sensors, Linux kernel modules for hardware
 *             monitoring
 * Copyright 2004-present Facebook. All Rights Reserved.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * Based on the max1619 driver, which was based on the lm90 driver.
 * The MAX127 is a voltage sensor chip made by Maxim.  It reports
 * up to eight voltages, with a choice of maximums * of 5V or 10V.
 * In addition, it can read either only positive voltages,
 * or negative voltages as well, for a maximum range of -10V to +10V.
 *
 * Complete datasheet can be obtained from Maxim's website at:
 *   http://datasheets.maximintegrated.com/en/ds/MAX127-MAX128B.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

static const unsigned short normal_i2c[] = {
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, I2C_CLIENT_END };

/*
 * Insmod parameters
 */

I2C_CLIENT_INSMOD_1(max127);

static int scaling;
module_param(scaling, int, 0);
MODULE_PARM_DESC(scaling, "Fixed-point scaling factor (* 10000), ie 24414");


/*
 * The MAX127 I2C messages
 */

/* We send a single query byte to the device, setting the following bits: */

#define MAX127_REG_R_START		0x80	/* Top bit must be set */
#define MAX127_REG_R_SEL_MASK		0x70	/* Which of 8 inputs to get */
#define MAX127_REG_R_SEL_SHIFT		4
#define MAX127_REG_R_RNG		0x08	/* 10v (otherwise 5v)  */
#define MAX127_REG_R_BIP		0x04	/* show negative voltage */
#define MAX127_REG_R_PD1		0x02	/* power saving controls */
#define MAX127_REG_R_PD0		0x01

/* Must shift return value to get a 12-bit value */
#define MAX127_RESULT_SHIFT		4

#define MAX127_CHANNELS			8

/*
 * Functions declaration
 */

static int max127_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int max127_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info);
static int max127_remove(struct i2c_client *client);
static void max127_update_device(struct device *dev, int which);

/*
 * Driver data (common to all clients)
 */

static const struct i2c_device_id max127_id[] = {
	{ "max127", max127 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max127_id);

static struct i2c_driver max127_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "max127",
	},
	.probe		= max127_probe,
	.remove		= max127_remove,
	.id_table	= max127_id,
	.detect		= max127_detect,
	.address_data	= &addr_data,
};

/*
 * Client data (each client gets its own)
 */

struct max127_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	u16 valid; /* zero until following fields are valid */

	u16 voltage;
};

/*
 * Sysfs stuff
 */
 
static ssize_t show_in(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max127_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int which = sensor_attr->index;
	int valid;
	unsigned voltage;

	mutex_lock(&data->update_lock);
	max127_update_device(dev, which);
	valid = data->valid;
	voltage = data->voltage;
	mutex_unlock(&data->update_lock);

	if (scaling)
		voltage = voltage * scaling / 10000;

	if (!valid)
		return -EIO;
	return sprintf(buf, "%u\n", voltage);
}


static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, show_in, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, show_in, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, show_in, NULL, 7);

static struct attribute *max127_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL
};

static const struct attribute_group max127_group = {
	.attrs = max127_attributes,
};

/*
 * Real code
 */

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max127_detect(struct i2c_client *new_client, int kind,
		  	 struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	/*
	 * We don't currently do any detection of the MAX127, although
	 * presumably we could try setting and unsetting the top
	 * bit in a query to see whether it does conversions or fails.
	 */

	strlcpy(info->type, "max127", I2C_NAME_SIZE);

	return 0;
}

static int max127_probe(struct i2c_client *new_client,
		        const struct i2c_device_id *id)
{
	struct max127_data *data;
	int err;

	data = kzalloc(sizeof(struct max127_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(new_client, data);
	data->valid = 0;
	mutex_init(&data->update_lock);

	/* Register sysfs hooks */
	if ((err = sysfs_create_group(&new_client->dev.kobj, &max127_group)))
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&new_client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	return 0;

exit_remove_files:
	sysfs_remove_group(&new_client->dev.kobj, &max127_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int max127_remove(struct i2c_client *client)
{
	struct max127_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &max127_group);

	kfree(data);
	return 0;
}

static void max127_update_device(struct device *dev, int which)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max127_data *data = i2c_get_clientdata(client);
	struct i2c_msg msg;
	int status;
	u8 buf[2];

	/*
	 * The MAX127 doesn't use standard SMBus queries; it needs a
	 * write to specify what conversion to make, followed by an i2c
	 * STOP.  It can then be read for the two-byte voltage value.
	 * Perhaps the idea is that a query can be started, then
	 * checked at an arbitrarily later time.  We don't support
	 * that -- we just get a result immediately.
	 *
	 * We have to use i2c_transfer to do the second read without
	 * writing to any registers, rather than using the i2c_smbus_xxxxxx
	 * queries that most of the other hwmon drivers do.
	 */

	dev_dbg(&client->dev, "Updating max127 data for probe %d.\n", which);
	data->valid = 0;

	buf[0] = MAX127_REG_R_START | (which << MAX127_REG_R_SEL_SHIFT) |
		MAX127_REG_R_RNG;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = 1;
	status = i2c_transfer(client->adapter, &msg, 1);

	if (status != 1) {
		return;
	}

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.buf = buf;
	msg.len = 2;
	status = i2c_transfer(client->adapter, &msg, 1);

	data->voltage = (buf[0] << 8) | buf[1];
	data->voltage >>= MAX127_RESULT_SHIFT;

	if (status == 1)
		data->valid = 1;
}

static int __init sensors_max127_init(void)
{
	return i2c_add_driver(&max127_driver);
}

static void __exit sensors_max127_exit(void)
{
	i2c_del_driver(&max127_driver);
}

MODULE_AUTHOR("Kevin Lahey <klahey@fb.com>");
MODULE_DESCRIPTION("MAX127 sensor driver");
MODULE_LICENSE("GPL");

module_init(sensors_max127_init);
module_exit(sensors_max127_exit);
