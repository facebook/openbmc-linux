// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for MP5920 and compatible chips.
 *
 * Copyright (c) 2019 Facebook Inc.
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
 */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

static struct pmbus_driver_info mp5920_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_VOLTAGE_IN] = 2266,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = -1,
	.m[PSC_VOLTAGE_OUT] = 2266,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = -1,
	.m[PSC_CURRENT_OUT] = 546,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = -2,
	.m[PSC_POWER] = 4833,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = -3,
	.m[PSC_TEMPERATURE] = 1067,
	.b[PSC_TEMPERATURE] = 20500,
	.R[PSC_TEMPERATURE] = -2,
	.func[0] = PMBUS_HAVE_VIN  | PMBUS_HAVE_VOUT |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_POUT |
                   PMBUS_HAVE_TEMP,
};

static int mp5920_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev =  &client->dev;
	int chip_id;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENODEV;

	chip_id = i2c_smbus_read_word_data(client, PMBUS_MFR_ID);
	if (chip_id < 0) {
		dev_err(dev, "Failed to read MFR ID\n");
		return chip_id;
	}

	return pmbus_do_probe(client, id, &mp5920_info);
}

static const struct of_device_id mp5920_of_match[] = {
        { .compatible = "mps,mp5920" },
        {}
};

static const struct i2c_device_id mp5920_id[] = {
        {"mp5920", 0},
        { }
};
MODULE_DEVICE_TABLE(i2c, mp5920_id);

static struct i2c_driver mp5920_driver = {
        .driver = {
		.name = "mp5920",
		.of_match_table = mp5920_of_match,
        },
        .probe = mp5920_probe,
        .remove = pmbus_do_remove,
        .id_table = mp5920_id,
};
module_i2c_driver(mp5920_driver);

MODULE_AUTHOR("Tony Ao <tony_ao@wiwynn.com>");
MODULE_DESCRIPTION("PMBus driver for MP5920");
MODULE_LICENSE("GPL");
