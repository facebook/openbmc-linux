// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for ltc4286/ltc4287 and compatible chips.
 *
 * Copyright (c) 2022 Meta Computer Inc.
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
#include <linux/pmbus.h>
#include "pmbus.h"

//LTC4286 register
#define LTC4286_MFR_CONFIG1    (0xF2)

//LTC4286 config data
#define LTC4286_RS_DEFAULT 1  //milli ohm
#define LTC4286_VRANGE_SELECT  (1 << 1)

static struct pmbus_driver_info ltc4286_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_VOLTAGE_IN] = 32,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = 1,
	.m[PSC_VOLTAGE_OUT] = 32,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 1,
	.m[PSC_CURRENT_OUT] = 1024,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = 0,
	.m[PSC_POWER] = 1,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = 1,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 273.15,
	.R[PSC_TEMPERATURE] = 0,
	.func[0] = PMBUS_HAVE_VIN  | PMBUS_HAVE_VOUT |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN |
                   PMBUS_HAVE_TEMP,
};

static int ltc4286_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int val;
	struct pmbus_driver_info *info;
	bool vrange = false;
	int rs;


	rs = LTC4286_RS_DEFAULT;
	/* load vrange_select */
	if (device_property_read_bool(dev, "vrange_select_25p6"))
		vrange = true; /* Default of VRANGE SELECT = 0, 102.4V */

	/* Setup MFR1 CONFIG register */
	if (vrange) {
		val = i2c_smbus_read_word_data(client, LTC4286_MFR_CONFIG1);
		val |= LTC4286_VRANGE_SELECT; /* VRANGE = 1, 25.6V */
		i2c_smbus_write_word_data(client, LTC4286_MFR_CONFIG1, val);
	}

	info = &ltc4286_info;
	if (vrange) {
		info->m[PSC_VOLTAGE_IN] = 128;
		info->m[PSC_POWER] = 4*rs;
	} else {
		info->m[PSC_POWER] = rs;
	}
	info->m[PSC_CURRENT_OUT] = 1024*rs;

	return pmbus_do_probe(client, info);
}

static const struct of_device_id ltc4286_of_match[] = {
	{ .compatible = "lltc,ltc4286" },
	{ .compatible = "lltc,ltc4287" },
	{}
};

static const struct i2c_device_id ltc4286_id[] = {
	{"ltc4286", 0},
	{"ltc4287", 1},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4286_id);

static struct i2c_driver ltc4286_driver = {
	.driver = {
		.name = "ltc4286",
		.of_match_table = ltc4286_of_match,
	},
	.probe = ltc4286_probe,
	.id_table = ltc4286_id,
};
module_i2c_driver(ltc4286_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for LTC4286");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
