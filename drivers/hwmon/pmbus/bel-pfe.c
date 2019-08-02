/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include "pmbus.h"

enum chips {pfe1100};

static struct pmbus_driver_info pfe_driver_info[] = {
	[pfe1100] = {
		.pages = 1,
		.format[PSC_VOLTAGE_IN] = linear,
		.format[PSC_VOLTAGE_OUT] = linear,
		.format[PSC_CURRENT_IN] = linear,
		.format[PSC_CURRENT_OUT] = linear,
		.format[PSC_POWER] = linear,
		.format[PSC_TEMPERATURE] = linear,
		.format[PSC_FAN] = linear,

		.func[0] = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
			   PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
			   PMBUS_HAVE_POUT |
			   PMBUS_HAVE_VIN | PMBUS_HAVE_IIN |
			   PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT |
			   PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2 |
			   PMBUS_HAVE_STATUS_TEMP |
			   PMBUS_HAVE_FAN12,
	},
};

static int pfe_pmbus_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int model;

	model = (int)id->driver_data;
	return pmbus_do_probe(client, id, &pfe_driver_info[model]);
}

static const struct i2c_device_id pfe_device_id[] = {
	{"pfe1100", pfe1100},
	{}
};

MODULE_DEVICE_TABLE(i2c, pfe_device_id);

/* This is the driver that will be inserted */
static struct i2c_driver pfe_pmbus_driver = {
	.driver = {
		   .name = "bel-pfe",
	},
	.probe = pfe_pmbus_probe,
	.remove = pmbus_do_remove,
	.id_table = pfe_device_id,
};

module_i2c_driver(pfe_pmbus_driver);

MODULE_AUTHOR("Tao Ren <taoren@fb.com>");
MODULE_DESCRIPTION("PMBus driver for Bel PFE Power Supplies");
MODULE_LICENSE("GPL");
