// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for MP5990 and compatible chips.
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

static int mp5990_read_byte_data(struct i2c_client *client, int page, int reg)
{
	switch (reg) {
	case PMBUS_VOUT_MODE:
		/*
		 * Enforce VOUT direct format, C4h reg BIT9
		   default val is not match vout format
		 */
		return PB_VOUT_MODE_DIRECT;
	default:
		return -ENODATA;
	}
}


static struct pmbus_driver_info mp5990_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_VOLTAGE_IN] = 32,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = 0,
	.m[PSC_VOLTAGE_OUT] = 32,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 0,
	.m[PSC_CURRENT_OUT] = 16,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = 0,
	.m[PSC_POWER] = 1,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = 0,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 0,
	.R[PSC_TEMPERATURE] = 0,
	.func[0] = PMBUS_HAVE_VIN  | PMBUS_HAVE_VOUT |
				PMBUS_HAVE_IOUT | PMBUS_HAVE_POUT |
				PMBUS_HAVE_TEMP | PMBUS_PAGE_VIRTUAL,
	.read_byte_data = mp5990_read_byte_data,

};

static struct pmbus_platform_data mp5990_plat_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK,
};


static int mp5990_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev =  &client->dev;
	int ret;

	client->dev.platform_data = &mp5990_plat_data;

	ret = i2c_smbus_write_byte_data(client, PMBUS_VOUT_MODE, 0x40);
	return pmbus_do_probe(client, &mp5990_info);
}

static const struct of_device_id mp5990_of_match[] = {
        { .compatible = "mps,mp5990" },
        {}
};

static const struct i2c_device_id mp5990_id[] = {
        {"mp5990", 0},
        { }
};
MODULE_DEVICE_TABLE(i2c, mp5990_id);

static struct i2c_driver mp5990_driver = {
        .driver = {
		.name = "mp5990",
		.of_match_table = mp5990_of_match,
        },
        .probe = mp5990_probe,
        .remove = pmbus_do_remove,
        .id_table = mp5990_id,
};
module_i2c_driver(mp5990_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for MP5990");
MODULE_LICENSE("GPL");
