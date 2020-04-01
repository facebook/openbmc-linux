/*
 * pxe1211c.c - The i2c driver for pxe1211c
 *
 * Copyright 2019-present Facebook. All Rights Reserved.
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

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"


#define PXE1211C_NUM_PAGES	3


/* Identify chip parameters. */
static int pxe1211c_identify(struct i2c_client *client,
							 struct pmbus_driver_info *info)
{
	if (pmbus_check_byte_register(client, 0, PMBUS_VOUT_MODE)) {
		u8 vout_mode;
		int ret;

		/* Read the register with VOUT scaling value.*/
		ret = pmbus_read_byte_data(client, 0, PMBUS_VOUT_MODE);
		if (ret < 0)
			return ret;

		vout_mode = ret & GENMASK(4, 0);

		switch (vout_mode) {
		case 1:
			info->vrm_version = vr12;
			break;
		case 2:
			info->vrm_version = vr13;
			break;
		default:
			return -ENODEV;
		}
	}

	return 0;
}

static int pxe1211c_probe(struct i2c_client *client,
						  const struct i2c_device_id *id)
{
	struct pmbus_driver_info *info;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	if (!i2c_check_functionality(client->adapter,
							  I2C_FUNC_SMBUS_READ_BYTE_DATA
							| I2C_FUNC_SMBUS_READ_WORD_DATA
							| I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0x00);
	if (ret < 0)
	{
		dev_err(&client->dev, "Failed to set PMBUS_PAGE 0x0\n");
		return ret;
	}

	ret = i2c_smbus_read_block_data(client, PMBUS_MFR_ID, buf);
	if (ret < 0)
	{
		dev_err(&client->dev, "Failed to read PMBUS_MFR_ID\n");
		return ret;
	}
	if (ret != 2 || strncmp(buf, "XP", strlen("XP")))
	{
		dev_err(&client->dev, "MFR_ID unrecognised\n");
		return -ENODEV;
	}

	info = devm_kzalloc(&client->dev, sizeof(struct pmbus_driver_info),
						GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pages = PXE1211C_NUM_PAGES;
	info->format[PSC_VOLTAGE_IN] = linear;
	info->format[PSC_VOLTAGE_OUT] = vid;
	info->format[PSC_CURRENT_IN] = linear;
	info->format[PSC_CURRENT_OUT] = linear;
	info->format[PSC_POWER] = linear;
	info->format[PSC_TEMPERATURE] = linear;

	info->func[0] = PMBUS_HAVE_VIN
		| PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN
		| PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN
		| PMBUS_HAVE_POUT | PMBUS_HAVE_TEMP
		| PMBUS_HAVE_STATUS_VOUT | PMBUS_HAVE_STATUS_IOUT
		| PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_STATUS_TEMP;
	info->func[1] = info->func[0];
	info->func[2] = info->func[0];
	info->identify = pxe1211c_identify;

	return pmbus_do_probe(client, id, info);
}

static const struct i2c_device_id pxe1211c_id[] = {
	{"pxe1211c", 0},
	{"pxe1110c", 1},
	{}};

MODULE_DEVICE_TABLE(i2c, pxe1211c_id);

static const struct of_device_id pxe1211_of_match[] = {
	{.compatible = "infineon,pxe1211c"},
	{.compatible = "infineon,pxe1110c"},
	{}
};
MODULE_DEVICE_TABLE(of, pxe1211_of_match);


static struct i2c_driver pxe1211c_driver = {
	.driver = {
		.name = "pxe1211c",
		.of_match_table = of_match_ptr(pxe1211_of_match),
	},
	.probe = pxe1211c_probe,
	.remove = pmbus_do_remove,
	.id_table = pxe1211c_id,
};

module_i2c_driver(pxe1211c_driver);

MODULE_AUTHOR("Kantaphon Kovathana <kkovath@celestica.com");
MODULE_DESCRIPTION("PMBus driver for PXE1211C");
MODULE_LICENSE("GPL");
