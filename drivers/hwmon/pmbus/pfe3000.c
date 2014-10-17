/*
 * Hardware monitoring driver for PFE3000 and compatibles
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
#include <linux/i2c/pmbus.h>
#include "pmbus.h"

enum chips { PFE3000 };

struct pfe3000_data {
	struct pmbus_driver_info info;
};

#define to_pfe3000_data(x)  container_of(x, struct pfe3000_data, info)

/*
 * Other PowerOne device require a wait time;  this is included in case
 * it is necessary for future debugging.
 */

#define PFE3000_WAIT_TIME		0	/* uS	*/

static ushort delay = PFE3000_WAIT_TIME;
module_param(delay, ushort, 0644);
MODULE_PARM_DESC(delay, "Delay between chip accesses in uS");


static const struct i2c_device_id pfe3000_id[] = {
	{"pfe3000", PFE3000 },
	{"pfe3000", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pfe3000_id);

static struct pmbus_platform_data platform_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK
};

static int pfe3000_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct pfe3000_data *data;
	struct pmbus_driver_info *info;

	data = devm_kzalloc(&client->dev, sizeof(struct pfe3000_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	info = &data->info;
	info->pages = 7;
	info->delay = delay;

	/*
	 * Skip extra status checks;  this is required to read the
	 * VOUT_MODE register to determine the exponent to apply
	 * to the VOUT values.
	 */
	dev->platform_data = &platform_data;

	/* Make sure that the page isn't pointing elsewhere! */

	i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);

	/*
	 * It seems reasonable to just scan the device for supported
	 * values, but most drivers seem to jam these values in
	 * there, so that's what we'll do.
	 */

	info->func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
	  PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
	  PMBUS_HAVE_FAN12 | PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2 |
	  PMBUS_HAVE_TEMP3 | PMBUS_HAVE_STATUS_VOUT |
	  PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
	  PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_STATUS_FAN12 |
	  PMBUS_HAVE_MFRDATA;
	info->func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
	  PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
	  PMBUS_HAVE_STATUS_VOUT | PMBUS_HAVE_STATUS_IOUT |
	  PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_STATUS_FAN12 |
	  PMBUS_HAVE_STATUS_INPUT;
	info->func[2] = PMBUS_HAVE_VOUT;
	info->func[4] = PMBUS_HAVE_VOUT;
	info->func[5] = PMBUS_HAVE_VOUT;
	info->func[6] = PMBUS_HAVE_VOUT;

	return pmbus_do_probe(client, id, info);
}

static struct i2c_driver pfe3000_driver = {
	.driver = {
		   .name = "pfe3000",
		   },
	.probe = pfe3000_probe,
	.remove = pmbus_do_remove,
	.id_table = pfe3000_id,
};

module_i2c_driver(pfe3000_driver);

MODULE_AUTHOR("Kevin Lahey, based on work by Guenter Roeck");
MODULE_DESCRIPTION("PMBus driver for PFE1100");
MODULE_LICENSE("GPL");
