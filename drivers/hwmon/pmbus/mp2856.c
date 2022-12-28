// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for MPS2971/2973
 * Multi-phase Digital VR Controllers
 *
 * Copyright (C) 2022 Meta Computer lnc.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pmbus.h>
#include "pmbus.h"

/* Vendor specific registers. */
#define MP2856_MFR_SLOPE_TRIM3		0x1d
#define MP2856_MFR_VR_MULTI_CONFIG_R1	0x0d
#define MP2856_MFR_VR_MULTI_CONFIG_R2	0x1d
#define MP2856_MUL1_CUR_SCALE_R1	0x0b
#define MP2856_MUL1_CUR_SCALE_R2	0x1b
#define MP2856_MFR_DC_LOOP_CTRL		0x59
#define MP2856_MFR_VR_CONFIG1		0x68
#define MP2856_MFR_READ_IOUT_PK		0x90
#define MP2856_MFR_READ_POUT_PK		0x91
#define MP2856_MFR_PROTECT_SET		0xc5
#define MP2856_MFR_RESO_SET		0x5e
#define MP2856_MFR_OVP_TH_SET		0xe5
#define MP2856_MFR_UVP_SET		0xe6

#define MP2856_VID_EN                   BIT(11)
#define MP2856_DRMOS_KCS		GENMASK(15, 12)
#define MP2856_VID_SCALE		5
#define MP2856_VIN_UV_LIMIT_UNIT	8
#define MP2856_PWR_EXPONENT_BIT		GENMASK(10, 6)

#define MP2856_MAX_PHASE_RAIL1		12
#define MP2856_MAX_PHASE_RAIL2		6
#define MP2856_PAGE_NUM			2

#define MP2856_RAIL2_FUNC	(PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT | \
				 PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | \
				 PMBUS_HAVE_POUT | PMBUS_HAVE_TEMP | \
				 PMBUS_PHASE_VIRTUAL)

#define MP2856_RAIL2_MAX_PHASE		4

struct mp2856_data {
	struct pmbus_driver_info info;
	int vout_scale;
	int vid_step[MP2856_PAGE_NUM];
	int vout_format[MP2856_PAGE_NUM];
	int curr_sense_gain[MP2856_PAGE_NUM];
};

static struct pmbus_platform_data mp28xx_plat_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK,
};

#define to_mp2856_data(x)  container_of(x, struct mp2856_data, info)


static int
mp2856_read_word_helper(struct i2c_client *client, int page, int phase, u8 reg,
			u16 mask)
{
	int ret = pmbus_read_word_data(client, page, phase, reg);

	return (ret > 0) ? ret & mask : ret;
}

static int
mp2856_vid2linear(int val)
{
	if (val >= 0x01)
		return 500 + (val - 1) * 10;
	return 0;
}

static int mp2856_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct mp2856_data *data = to_mp2856_data(info);
	int ret;

	switch (reg) {
	case PMBUS_READ_VOUT:
		ret = mp2856_read_word_helper(client, page, phase, reg,
					      GENMASK(11, 0));
		if (ret < 0)
			return ret;

		/*
		 * READ_VOUT can be provided in VID or linear format. This
		 * provided in a linear format. In case format is VID - convert
		 * to linear.
		 */
		if (data->vout_format[page] == vid)
			ret = mp2856_vid2linear(ret);
		break;
	default:
		return -ENODATA;
	}

	return ret;
}

static int mp2856_identify_multiphase_rail2(struct i2c_client *client)
{
	int ret;

	/*
	 * Identify multiphase for rail 2 - could be from 0 to 1.
	 * In case phase number is zero – only page zero is supported
	 */
	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify multiphase for rail 2 - could be from 0 to 1. */
	ret = i2c_smbus_read_word_data(client, MP2856_MFR_VR_MULTI_CONFIG_R2);
	if (ret < 0)
		return ret;

	ret &= GENMASK(2, 0);
	return (ret >= MP2856_RAIL2_MAX_PHASE) ? MP2856_RAIL2_MAX_PHASE : ret;
}

static int
mp2856_identify_vid(struct i2c_client *client, struct mp2856_data *data,
		    struct pmbus_driver_info *info, u32 reg, int page,
		    u32 imvp_bit)
{
	int ret;

	/* Identify VID mode and step selection. */
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		return ret;

	if (ret & imvp_bit) {
		info->vrm_version[page] = vr13;
		data->vid_step[page] = MP2856_VID_SCALE;
	} else {
		//workaround for chip power scale issue
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
		if (ret < 0)
			return ret;
		
		ret = i2c_smbus_read_word_data(client, MP2856_MUL1_CUR_SCALE_R1);
		if (ret < 0)
			return ret;

		ret &= ~MP2856_PWR_EXPONENT_BIT;
                ret = i2c_smbus_write_word_data(client, MP2856_MUL1_CUR_SCALE_R1, ret);
		if (ret < 0)
			return ret;

		ret = i2c_smbus_read_word_data(client, MP2856_MUL1_CUR_SCALE_R2);
		if (ret < 0)
			return ret;

		ret &= ~MP2856_PWR_EXPONENT_BIT;
                ret = i2c_smbus_write_word_data(client, MP2856_MUL1_CUR_SCALE_R2, ret);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int
mp2856_identify_rails_vid(struct i2c_client *client, struct mp2856_data *data,
			  struct pmbus_driver_info *info)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify VID mode for rail 1. */
	ret = mp2856_identify_vid(client, data, info,
				  MP2856_MFR_RESO_SET, 0,
				  MP2856_VID_EN);
	if (ret < 0)
		return ret;

	/* Identify VID mode for rail 2, if connected. */
	if (info->phases[1])
		ret = mp2856_identify_vid(client, data, info,
					  MP2856_MFR_RESO_SET, 1,
					  MP2856_VID_EN);
	return ret;
}

static int
mp2856_current_sense_gain_get(struct i2c_client *client,
			      struct mp2856_data *data)
{
	int i, ret;

	/*
	 * Obtain DrMOS current sense gain of power stage from the register
	 * MP2856_MFR_VR_CONFIG1, bits 15-12. The value is selected as below:
	 * 00b - 5µA/A, 01b - 8.5µA/A, 10b - 9.7µA/A, 11b - 10µA/A. Other
	 * values are invalid.
	 */
	for (i = 0 ; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;
		ret = i2c_smbus_read_word_data(client,
					       MP2856_MFR_VR_CONFIG1);
		if (ret < 0)
			return ret;

		switch ((ret & MP2856_DRMOS_KCS) >> 12) {
		case 0:
			data->curr_sense_gain[i] = 50;
			break;
		case 1:
			data->curr_sense_gain[i] = 85;
			break;
		case 2:
			data->curr_sense_gain[i] = 97;
			break;
		default:
			data->curr_sense_gain[i] = 100;
			break;
		}
	}

	return 0;
}

static int
mp2856_identify_vout_format(struct i2c_client *client,
			    struct mp2856_data *data, int page)
{
	int ret;
  int val;

	ret = i2c_smbus_read_word_data(client, MP2856_MFR_RESO_SET);
	if (ret < 0)
		return ret;

	val = (ret & GENMASK(11, 11)) >> 11;
	switch (val) {
	case 0:
		data->vout_format[page] = vid;
		break;
	default:
		data->vout_format[page] = linear;
		break;
	}
	return 0;
}

static int
mp2856_vout_per_rail_config_get(struct i2c_client *client,
				struct mp2856_data *data,
				struct pmbus_driver_info *info)
{
	int i, ret;

	for (i = 0; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;
		
		/*
		 * Get VOUT format for READ_VOUT command : VID or direct.
		 * Pages on same device can be configured with different
		 * formats.
		 */
		ret = mp2856_identify_vout_format(client, data, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct pmbus_driver_info mp2856_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_POUT |
		PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT | PMBUS_PHASE_VIRTUAL,
	.read_word_data = mp2856_read_word_data,
};

static int mp2856_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	struct mp2856_data *data;
	int ret;

	/*
	 * MP2856/MP2857 devices may not stay in page 0 during device
	 * probe which leads to probe failure (read status word failed).
	 * So let's set the device to page 0 at the beginning.
	 */
	client->dev.platform_data = &mp28xx_plat_data;
	i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);

	data = devm_kzalloc(&client->dev, sizeof(struct mp2856_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memcpy(&data->info, &mp2856_info, sizeof(*info));
	info = &data->info;
	/* Identify multiphase configuration for rail 2. */
	ret = mp2856_identify_multiphase_rail2(client);
	if (ret < 0)
		return ret;

	if (ret) {
		/* Two rails are connected. */
		data->info.pages = MP2856_PAGE_NUM;
		data->info.phases[1] = ret;
		data->info.func[1] = MP2856_RAIL2_FUNC;
	}

	ret = mp2856_identify_rails_vid(client, data, info);
	if (ret < 0)
		return ret;


	/* Obtain current sense gain of power stage. */
	ret = mp2856_current_sense_gain_get(client, data);
	if (ret)
		return ret;


	/* Obtain offsets, maximum and format for vout. */
	ret = mp2856_vout_per_rail_config_get(client, data, info);
	if (ret)
		return ret;


	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id mp2856_id[] = {
	{"mp2856", 0},
	{"mp2857", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mp2856_id);

static const struct of_device_id __maybe_unused mp2856_of_match[] = {
	{.compatible = "mps,mp2856"},
	{.compatible = "mps,mp2857"},
	{}
};
MODULE_DEVICE_TABLE(of, mp2856_of_match);

static struct i2c_driver mp2856_driver = {
	.driver = {
		.name = "mp2856",
		.of_match_table = of_match_ptr(mp2856_of_match),
	},
	.probe_new = mp2856_probe,
	.remove = pmbus_do_remove,
	.id_table = mp2856_id,
};

module_i2c_driver(mp2856_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MP2856/MP2857 device");
MODULE_LICENSE("GPL");
