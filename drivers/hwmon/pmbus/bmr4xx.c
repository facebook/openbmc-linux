// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for BMR4XX and compatible chips.
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

static struct pmbus_driver_info bmr491_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN  | PMBUS_HAVE_VOUT |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP |
		   PMBUS_PAGE_VIRTUAL,
};

static struct pmbus_platform_data bmrxxx_plat_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK,
};

static int bmr4xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev =  &client->dev;

	client->dev.platform_data = &bmrxxx_plat_data;

	return pmbus_do_probe(client, &bmr491_info);
}

static const struct of_device_id bmr4xx_of_match[] = {
        { .compatible = "mps,bmr491" },
        {}
};

static const struct i2c_device_id bmr491_id[] = {
        {"bmr491", 0},
        { }
};
MODULE_DEVICE_TABLE(i2c, bmr491_id);

static struct i2c_driver bmr491_driver = {
        .driver = {
		.name = "bmr491",
		.of_match_table = bmr4xx_of_match,
        },
        .probe = bmr4xx_probe,
        .remove = pmbus_do_remove,
        .id_table = bmr491_id,
};
module_i2c_driver(bmr491_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for bmr4xx");
MODULE_LICENSE("GPL");
