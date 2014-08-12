/*
 * Hardware monitoring driver for PFE1100 and compatibles
 * Based on the zl6100 driver with the following copyright:
 *
 * Copyright (c) 2011 Ericsson AB.
 * Copyright (c) 2012 Guenter Roeck
 * Copyright 2004-present Facebook. All Rights Reserved.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include "pmbus.h"

enum chips { pfe1100_12_054xA };

struct pfe1100_data {
	int id;
	ktime_t access;		/* chip access time */
	int delay;		/* Delay between chip accesses in uS */
	struct pmbus_driver_info info;
};

#define to_pfe1100_data(x)  container_of(x, struct pfe1100_data, info)

/*
 * I'm not sure whether this is a delay required by the PFE1100 or by
 * the I2C repeater that we're using to talk to it, but it's definitely
 * necessary.  Doh.
 */

#define PFE1100_WAIT_TIME		3500	/* uS	*/

static ushort delay = PFE1100_WAIT_TIME;
module_param(delay, ushort, 0644);
MODULE_PARM_DESC(delay, "Delay between chip accesses in uS");

/* Some chips need a delay between accesses */

static inline void pfe1100_wait(const struct pfe1100_data *data)
{
	if (data->delay) {
		s64 delta = ktime_us_delta(ktime_get(), data->access);
		if (delta < data->delay)
			/*
			 * Note that udelay is busy waiting.  With
			 * continuous queries to the device, I saw about
			 * 24% system CPU time.  msleep is quite a bit
			 * slower (it actually takes a minimum of 20ms),
			 * but doesn't busy wait.  Hmmm.
			 */
			udelay(data->delay - delta);
	}
}

static int pfe1100_read_word_data(struct i2c_client *client, int page, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe1100_data *data = to_pfe1100_data(info);
	int ret;

	if (page > 0)
		return -ENXIO;

	if (reg >= PMBUS_VIRT_BASE)
		return -ENXIO;

	pfe1100_wait(data);
	ret = pmbus_read_word_data(client, page, reg);
	data->access = ktime_get();

	return ret;
}

static int pfe1100_read_byte_data(struct i2c_client *client, int page, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe1100_data *data = to_pfe1100_data(info);
	int ret;

	if (page > 0)
		return -ENXIO;

	pfe1100_wait(data);

	ret = pmbus_read_byte_data(client, page, reg);
	data->access = ktime_get();

	return ret;
}

static int pfe1100_write_word_data(struct i2c_client *client, int page, int reg,
				  u16 word)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe1100_data *data = to_pfe1100_data(info);
	int ret;

	if (page > 0)
		return -ENXIO;

	if (reg >= PMBUS_VIRT_BASE)
		return -ENXIO;

	pfe1100_wait(data);
	ret = pmbus_write_word_data(client, page, reg, word);
	data->access = ktime_get();

	return ret;
}

static int pfe1100_write_byte(struct i2c_client *client, int page, u8 value)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe1100_data *data = to_pfe1100_data(info);
	int ret;

	if (page > 0)
		return -ENXIO;

	pfe1100_wait(data);
	ret = pmbus_write_byte(client, page, value);
	data->access = ktime_get();

	return ret;
}

static const struct i2c_device_id pfe1100_id[] = {
	{"pfe1100", pfe1100_12_054xA},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pfe1100_id);

static int pfe1100_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pfe1100_data *data;
	struct pmbus_driver_info *info;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA
				     | I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(struct pfe1100_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->id = id->driver_data;

	/*
	 * The datasheets don't say anything about it, but it appears
	 * that we need a pause between each query.
	 */
	data->delay = delay;
	info = &data->info;
	info->pages = 1;

	/*
	 * It seems reasonable to just scan the device for supported
	 * values, but most drivers just seem to jam these values in
	 * there.
	 */

	info->func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
	  PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
	  PMBUS_HAVE_FAN12 | PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2 |
	  PMBUS_HAVE_TEMP3 | PMBUS_HAVE_STATUS_VOUT |
	  PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
	  PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_STATUS_FAN12;

	data->access = ktime_get();

	info->read_word_data = pfe1100_read_word_data;
	info->read_byte_data = pfe1100_read_byte_data;
	info->write_word_data = pfe1100_write_word_data;
	info->write_byte = pfe1100_write_byte;

	return pmbus_do_probe(client, id, info);
}

static struct i2c_driver pfe1100_driver = {
	.driver = {
		   .name = "pfe1100",
		   },
	.probe = pfe1100_probe,
	.remove = pmbus_do_remove,
	.id_table = pfe1100_id,
};

module_i2c_driver(pfe1100_driver);

MODULE_AUTHOR("Kevin Lahey, based on work by Guenter Roeck");
MODULE_DESCRIPTION("PMBus driver for PFE1100");
MODULE_LICENSE("GPL");
