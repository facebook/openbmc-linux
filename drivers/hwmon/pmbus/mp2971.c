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
#define MP2971_MFR_APS_HYS_R2		0x0d
#define MP2971_MFR_SLOPE_TRIM3		0x1d
#define MP2971_MFR_VR_MULTI_CONFIG_R1	0x0d
#define MP2971_MFR_VR_MULTI_CONFIG_R2	0x1d
#define MP2971_MFR_VR_CONF_IMON_OFFSET	0x0e
#define MP2971_MFR_APS_DECAY_ADV	0x56
#define MP2971_MFR_DC_LOOP_CTRL		0x59
#define MP2971_MFR_OCP_UCP_PHASE_SET	0x65
#define MP2971_MFR_VR_CONFIG1		0x68
#define MP2971_MFR_READ_CS1_2		0x82
#define MP2971_MFR_READ_CS3_4		0x83
#define MP2971_MFR_READ_CS5_6		0x84
#define MP2971_MFR_READ_CS7_8		0x85
#define MP2971_MFR_READ_CS9_10		0x86
#define MP2971_MFR_READ_CS11_12		0x87
#define MP2971_MFR_READ_IOUT_PK		0x90
#define MP2971_MFR_READ_POUT_PK		0x91
#define MP2971_MFR_PROTECT_SET		0xc5
#define MP2971_MFR_RESO_SET		0xc7
#define MP2971_MFR_OVP_TH_SET		0xe5
#define MP2971_MFR_UVP_SET		0xe6

#define MP2971_VOUT_FORMAT		GENMASK(7, 6)
#define MP2971_VID_STEP_SEL_R1		BIT(4)
#define MP2971_IMVP9_EN_R1		BIT(13)
#define MP2971_VID_STEP_SEL_R2		BIT(3)
#define MP2971_IMVP9_EN_R2		BIT(12)
#define MP2971_PRT_THRES_DIV_OV_EN	BIT(14)
#define MP2971_DRMOS_KCS		GENMASK(13, 11)
#define MP2971_PROT_DEV_OV_OFF		10
#define MP2971_PROT_DEV_OV_ON		5
#define MP2971_SENSE_AMPL		BIT(11)
#define MP2971_SENSE_AMPL_UNIT		1
#define MP2971_SENSE_AMPL_HALF		2
#define MP2971_VIN_UV_LIMIT_UNIT	8

#define MP2971_MAX_PHASE_RAIL1	12
#define MP2971_MAX_PHASE_RAIL2	6
#define MP2971_PAGE_NUM		2

#define MP2971_RAIL2_FUNC	(PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT | \
				 PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT | \
				 PMBUS_HAVE_POUT | PMBUS_HAVE_TEMP | \
				 PMBUS_PHASE_VIRTUAL)

struct mp2971_data {
	struct pmbus_driver_info info;
	int vout_scale;
	int vid_step[MP2971_PAGE_NUM];
	int vref[MP2971_PAGE_NUM];
	int vref_off[MP2971_PAGE_NUM];
	int vout_max[MP2971_PAGE_NUM];
	int vout_ov_fixed[MP2971_PAGE_NUM];
	int vout_format[MP2971_PAGE_NUM];
	int curr_sense_gain[MP2971_PAGE_NUM];
};

/*
 * Disable status check for mp2971/mp2973 devices, because some devices report
 * communication error (invalid command) for VOUT_MODE command (0x20)
 * although correct VOUT_MODE (0x16) is returned: it leads to incorrect
 * exponent in linear mode.
 */
static struct pmbus_platform_data mp29xx_plat_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK,
};

#define to_mp2971_data(x)  container_of(x, struct mp2971_data, info)

static int mp2971_read_byte_data(struct i2c_client *client, int page, int reg)
{
	switch (reg) {
	case PMBUS_VOUT_MODE:
		/*
		 * Enforce VOUT direct format, since device allows to set the
		 * different formats for the different rails. Conversion from
		 * VID to direct provided by driver internally, in case it is
		 * necessary.
		 */
		return PB_VOUT_MODE_DIRECT;
	default:
		return -ENODATA;
	}
}

static int
mp2971_read_word_helper(struct i2c_client *client, int page, int phase, u8 reg,
			u16 mask)
{
	int ret = pmbus_read_word_data(client, page, phase, reg);

	return (ret > 0) ? ret & mask : ret;
}

static int
mp2971_vid2direct(int vrf, int val)
{
	switch (vrf) {
	case vr12:
		if (val >= 0x01)
			return 250 + (val - 1) * 5;
		break;
	case vr13:
		if (val >= 0x01)
			return 500 + (val - 1) * 10;
		break;
	case imvp9:
		if (val >= 0x01)
			return 200 + (val - 1) * 10;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mp2971_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct mp2971_data *data = to_mp2971_data(info);
	int ret;

	switch (reg) {
	case PMBUS_OT_FAULT_LIMIT:
		ret = mp2971_read_word_helper(client, page, phase, reg,
					      GENMASK(7, 0));
		break;
	case PMBUS_VIN_OV_FAULT_LIMIT:
		ret = mp2971_read_word_helper(client, page, phase, reg,
					      GENMASK(7, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(ret, MP2971_VIN_UV_LIMIT_UNIT);
		break;
	case PMBUS_VOUT_OV_FAULT_LIMIT:
		/*
		 * Register provides two values for over-voltage protection
		 * threshold for fixed (ovp2) and tracking (ovp1) modes. The
		 * minimum of these two values is provided as over-voltage
		 * fault alarm.
		 */
		ret = mp2971_read_word_helper(client, page, phase,
					      MP2971_MFR_OVP_TH_SET,
					      GENMASK(2, 0));
		if (ret < 0)
			return ret;

		ret = min_t(int, data->vout_max[page] + 50 * (ret + 1),
			    data->vout_ov_fixed[page]);
		break;
	case PMBUS_VOUT_UV_FAULT_LIMIT:
		ret = mp2971_read_word_helper(client, page, phase,
					      MP2971_MFR_UVP_SET,
					      GENMASK(2, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(data->vref[page] * 10 - 50 *
					(ret + 1) * data->vout_scale, 10);
		break;
	case PMBUS_READ_VOUT:
		ret = mp2971_read_word_helper(client, page, phase, reg,
					      GENMASK(11, 0));
		if (ret < 0)
			return ret;

		/*
		 * READ_VOUT can be provided in VID or direct format. The
		 * format type is specified by bit 15 of the register
		 * MP2971_MFR_DC_LOOP_CTRL. The driver enforces VOUT direct
		 * format, since device allows to set the different formats for
		 * the different rails and also all VOUT limits registers are
		 * provided in a direct format. In case format is VID - convert
		 * to direct.
		 */
		if (data->vout_format[page] == vid)
			ret = mp2971_vid2direct(info->vrm_version[page], ret);
		break;

	case PMBUS_VIRT_READ_POUT_MAX:
		ret = mp2971_read_word_helper(client, page, phase,
					      MP2971_MFR_READ_POUT_PK,
					      GENMASK(12, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(ret, 4);
		break;
	case PMBUS_VIRT_READ_IOUT_MAX:
		ret = mp2971_read_word_helper(client, page, phase,
					      MP2971_MFR_READ_IOUT_PK,
					      GENMASK(12, 0));
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST(ret, 4);
		break;
	case PMBUS_UT_WARN_LIMIT:
	case PMBUS_UT_FAULT_LIMIT:
	case PMBUS_VIN_UV_WARN_LIMIT:
	case PMBUS_VIN_UV_FAULT_LIMIT:
	case PMBUS_VOUT_UV_WARN_LIMIT:
	case PMBUS_VOUT_OV_WARN_LIMIT:
	case PMBUS_VIN_OV_WARN_LIMIT:
	case PMBUS_IIN_OC_FAULT_LIMIT:
	case PMBUS_IOUT_OC_LV_FAULT_LIMIT:
	case PMBUS_IIN_OC_WARN_LIMIT:
	case PMBUS_IOUT_OC_WARN_LIMIT:
	case PMBUS_IOUT_OC_FAULT_LIMIT:
	case PMBUS_IOUT_UC_FAULT_LIMIT:
	case PMBUS_POUT_OP_FAULT_LIMIT:
	case PMBUS_POUT_OP_WARN_LIMIT:
	case PMBUS_PIN_OP_WARN_LIMIT:
		return -ENXIO;
	default:
		return -ENODATA;
	}

	return ret;
}

static int mp2971_identify_multiphase_rail2(struct i2c_client *client)
{
	int ret;

	/*
	 * Identify multiphase for rail 2 - could be from 0 to 6.
	 * In case phase number is zero – only page zero is supported
	 */
	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify multiphase for rail 2 - could be from 0 to 6. */
	ret = i2c_smbus_read_word_data(client, MP2971_MFR_VR_MULTI_CONFIG_R2);
	if (ret < 0)
		return ret;

	ret &= GENMASK(2, 0);
	return (ret >= 6) ? 6 : ret;
}

static void mp2971_set_phase_rail1(struct pmbus_driver_info *info)
{
	int i;

	for (i = 0 ; i < info->phases[0]; i++)
		info->pfunc[i] = PMBUS_HAVE_IOUT;
}

static void
mp2971_set_phase_rail2(struct pmbus_driver_info *info, int num_phases)
{
	int i;

	/* Set phases for rail 2 from upper to lower. */
	for (i = 1; i <= num_phases; i++)
		info->pfunc[MP2971_MAX_PHASE_RAIL1 - i] = PMBUS_HAVE_IOUT;
}

static int
mp2971_identify_multiphase(struct i2c_client *client, struct mp2971_data *data,
			   struct pmbus_driver_info *info)
{
	int num_phases2, ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify multiphase for rail 1 - could be from 1 to 12. */
	ret = i2c_smbus_read_word_data(client, MP2971_MFR_VR_MULTI_CONFIG_R1);
	if (ret <= 0)
		return ret;

	info->phases[0] = ret & GENMASK(3, 0);

	/*
	 * The device provides a total of 8 PWM pins, and can be configured
	 * to different phase count applications for rail 1 and rail 2.
	 * Rail 1 can be set to 12 phases, while rail 2 can only be set to 6
	 * phases at most. When rail 1’s phase count is configured as 0, rail
	 * 1 operates with 1-phase DCM. When rail 2 phase count is configured
	 * as 0, rail 2 is disabled.
	 */
	if (info->phases[0] > MP2971_MAX_PHASE_RAIL1)
		return -EINVAL;

	mp2971_set_phase_rail1(info);

	num_phases2 = min(MP2971_MAX_PHASE_RAIL1 - info->phases[0], MP2971_MAX_PHASE_RAIL2);
	if (info->phases[1] && info->phases[1] <= num_phases2)
		mp2971_set_phase_rail2(info, num_phases2);

	return 0;
}

static int
mp2971_identify_vid(struct i2c_client *client, struct mp2971_data *data,
		    struct pmbus_driver_info *info, u32 reg, int page,
		    u32 imvp_bit, u32 vr_bit)
{
	int ret;

	/* Identify VID mode and step selection. */
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		return ret;

	if (ret & imvp_bit) {
		info->vrm_version[page] = imvp9;
		data->vid_step[page] = MP2971_PROT_DEV_OV_OFF;
	} else if (ret & vr_bit) {
		info->vrm_version[page] = vr12;
		data->vid_step[page] = MP2971_PROT_DEV_OV_ON;
	} else {
		info->vrm_version[page] = vr13;
		data->vid_step[page] = MP2971_PROT_DEV_OV_OFF;
	}

	return 0;
}

static int
mp2971_identify_rails_vid(struct i2c_client *client, struct mp2971_data *data,
			  struct pmbus_driver_info *info)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 2);
	if (ret < 0)
		return ret;

	/* Identify VID mode for rail 1. */
	ret = mp2971_identify_vid(client, data, info,
				  MP2971_MFR_VR_MULTI_CONFIG_R1, 0,
				  MP2971_IMVP9_EN_R1, 
				  MP2971_VID_STEP_SEL_R1);
	if (ret < 0)
		return ret;

	/* Identify VID mode for rail 2, if connected. */
	if (info->phases[1])
		ret = mp2971_identify_vid(client, data, info,
					  MP2971_MFR_VR_MULTI_CONFIG_R2, 1,
					  MP2971_IMVP9_EN_R2,
					  MP2971_VID_STEP_SEL_R2);
	return ret;
}

static int
mp2971_current_sense_gain_get(struct i2c_client *client,
			      struct mp2971_data *data)
{
	int i, ret;

	/*
	 * Obtain DrMOS current sense gain of power stage from the register
	 * MP2971_MFR_VR_CONFIG1, bits 13-12. The value is selected as below:
	 * 00b - 5µA/A, 01b - 8.5µA/A, 10b - 9.7µA/A, 11b - 10µA/A. Other
	 * values are invalid.
	 */
	for (i = 0 ; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;
		ret = i2c_smbus_read_word_data(client,
					       MP2971_MFR_VR_CONFIG1);
		if (ret < 0)
			return ret;

		switch ((ret & MP2971_DRMOS_KCS) >> 11) {
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
mp2971_vref_offset_get(struct i2c_client *client, struct mp2971_data *data,
		       int page)
{
	int ret;
        int val;

	ret = i2c_smbus_read_word_data(client, MP2971_MFR_PROTECT_SET);
	if (ret < 0)
		return ret;

        if (page == 0)
          val = (ret & GENMASK(14, 12)) >> 12;
        else if (page == 1)
          val = (ret & GENMASK(12, 10)) >> 10;
        else
          return -EIO; 

	switch (val) {
	case 1:
		data->vref_off[page] = 140;
		break;
	case 2:
		data->vref_off[page] = 220;
		break;
	case 4:
		data->vref_off[page] = 400;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
mp2971_vout_max_get(struct i2c_client *client, struct mp2971_data *data,
		    struct pmbus_driver_info *info, int page)
{
	int ret;

	/* Get maximum reference voltage of VID-DAC in VID format. */
	ret = i2c_smbus_read_word_data(client, PMBUS_VOUT_MAX);
	if (ret < 0)
		return ret;

	data->vout_max[page] = mp2971_vid2direct(info->vrm_version[page], ret &
						 GENMASK(8, 0));
	return 0;
}

static int
mp2971_identify_vout_format(struct i2c_client *client,
			    struct mp2971_data *data, int page)
{
	int ret;
        int val;

	ret = i2c_smbus_read_word_data(client, MP2971_MFR_RESO_SET);
	if (ret < 0)
		return ret;

        if (page == 0)
          val = (ret & GENMASK(7, 6)) >> 6;
        else if (page == 1)
          val = (ret & GENMASK(4, 3)) >> 3;
        else
          return -EIO; 


	switch (val) {
	case 0:
		data->vout_format[page] = vid;
		break;
	case 1:
                data->vout_format[page] = linear;
		break;
	default:
                data->vout_format[page] = direct;
		break;
	}
	return 0;
}

static int
mp2971_vout_ov_scale_get(struct i2c_client *client, struct mp2971_data *data,
			 struct pmbus_driver_info *info)
{
	int thres_dev, sense_ampl, ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);
	if (ret < 0)
		return ret;

	/*
	 * Get divider for over- and under-voltage protection thresholds
	 * configuration from the Advanced Options of Auto Phase Shedding and
	 * decay register.
	 */
	ret = i2c_smbus_read_word_data(client, MP2971_MFR_APS_DECAY_ADV);
	if (ret < 0)
		return ret;
	thres_dev = ret & MP2971_PRT_THRES_DIV_OV_EN ? MP2971_PROT_DEV_OV_ON :
	                                               MP2971_PROT_DEV_OV_OFF;

	/* Select the gain of remote sense amplifier. */
	ret = i2c_smbus_read_word_data(client, PMBUS_VOUT_SCALE_LOOP);
	if (ret < 0)
		return ret;
	sense_ampl = ret & MP2971_SENSE_AMPL ? MP2971_SENSE_AMPL_HALF :
					       MP2971_SENSE_AMPL_UNIT;

	data->vout_scale = sense_ampl * thres_dev;

	return 0;
}

static int
mp2971_vout_per_rail_config_get(struct i2c_client *client,
				struct mp2971_data *data,
				struct pmbus_driver_info *info)
{
	int i, ret;

	for (i = 0; i < data->info.pages; i++) {
		ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, i);
		if (ret < 0)
			return ret;

		/* Obtain voltage reference offsets. */
		ret = mp2971_vref_offset_get(client, data, i);
		if (ret < 0)
			return ret;

		/* Obtain maximum voltage values. */
		ret = mp2971_vout_max_get(client, data, info, i);
		if (ret < 0)
			return ret;
		
		/*
		 * Get VOUT format for READ_VOUT command : VID or direct.
		 * Pages on same device can be configured with different
		 * formats.
		 */
		ret = mp2971_identify_vout_format(client, data, i);
		if (ret < 0)
			return ret;

		/*
		 * Set over-voltage fixed value. Thresholds are provided as
		 * fixed value, and tracking value. The minimum of them are
		 * exposed as over-voltage critical threshold.
		 */
		data->vout_ov_fixed[i] = data->vref[i] +
					 DIV_ROUND_CLOSEST(data->vref_off[i] *
							   data->vout_scale,
							   10);
	}

	return 0;
}

static struct pmbus_driver_info mp2971_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.m[PSC_TEMPERATURE] = 1,
	.m[PSC_VOLTAGE_OUT] = 1,
	.R[PSC_VOLTAGE_OUT] = 3,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_POUT |
		PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT | PMBUS_PHASE_VIRTUAL,
	.read_byte_data = mp2971_read_byte_data,
	.read_word_data = mp2971_read_word_data,
};

static int mp2971_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	struct mp2971_data *data;
	int ret;

	/*
	 * MP2971/MP2973 devices may not stay in page 0 during device
	 * probe which leads to probe failure (read status word failed).
	 * So let's set the device to page 0 at the beginning.
	 */
	client->dev.platform_data = &mp29xx_plat_data;
	i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);

	data = devm_kzalloc(&client->dev, sizeof(struct mp2971_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memcpy(&data->info, &mp2971_info, sizeof(*info));
	info = &data->info;
	/* Identify multiphase configuration for rail 2. */
	ret = mp2971_identify_multiphase_rail2(client);
	if (ret < 0)
		return ret;

	if (ret) {
		/* Two rails are connected. */
		data->info.pages = MP2971_PAGE_NUM;
		data->info.phases[1] = ret;
		data->info.func[1] = MP2971_RAIL2_FUNC;
	}

	ret = mp2971_identify_rails_vid(client, data, info);
	if (ret < 0)
		return ret;

	/* Obtain current sense gain of power stage. */
	ret = mp2971_current_sense_gain_get(client, data);
	if (ret)
		return ret;

	/* Obtain offsets, maximum and format for vout. */
	ret = mp2971_vout_per_rail_config_get(client, data, info);
	if (ret)
		return ret;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id mp2971_id[] = {
	{"mp2971", 0},
	{"mp2973", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mp2971_id);

static const struct of_device_id __maybe_unused mp2971_of_match[] = {
	{.compatible = "mps,mp2971"},
	{.compatible = "mps,mp2973"},
	{}
};
MODULE_DEVICE_TABLE(of, mp2971_of_match);

static struct i2c_driver mp2971_driver = {
	.driver = {
		.name = "mp2971",
		.of_match_table = of_match_ptr(mp2971_of_match),
	},
	.probe_new = mp2971_probe,
	.remove = pmbus_do_remove,
	.id_table = mp2971_id,
};

module_i2c_driver(mp2971_driver);

MODULE_AUTHOR("Peter Yin <peter.yin@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MP2971/MP2973 device");
MODULE_LICENSE("GPL");
