
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Linear Technology LTC2991 power monitor
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

/* Command codes */
#define LTC2991_STATUS_LOW	0x00
#define LTC2991_STATUS_HIGH	0x01
#define LTC2991_ENABLE		0x01
#define LTC2991_CONTROL_A	0x06
#define LTC2991_CONTROL_B	0x07
#define LTC2991_CONTROL_C	0x08
#define LTC2991_PWM_THRESH	0x09
#define LTC2991_V1_MSB		0x0A
#define LTC2991_V2_MSB		0x0C
#define LTC2991_V3_MSB		0x0E
#define LTC2991_V4_MSB		0x10
#define LTC2991_V5_MSB		0x12
#define LTC2991_V6_MSB		0x14
#define LTC2991_V7_MSB		0x16
#define LTC2991_V8_MSB		0x18
#define LTC2991_TINT_MSB	0x1A
#define LTC2991_VCC_MSB		0x1C

/* Telemetry index */
#define LTC2991_TEMP1	BIT(0)
#define LTC2991_TEMP2	BIT(1)
#define LTC2991_TEMP3	BIT(2)
#define LTC2991_TEMP4	BIT(3)
#define LTC2991_TEMP5	BIT(4)
#define LTC2991_CURR1	BIT(5)
#define LTC2991_CURR2	BIT(6)
#define LTC2991_CURR3	BIT(7)
#define LTC2991_CURR4	BIT(8)
#define LTC2991_IN0 	BIT(9)
#define LTC2991_IN1 	BIT(10)
#define LTC2991_IN2 	BIT(11)
#define LTC2991_IN3 	BIT(12)
#define LTC2991_IN4 	BIT(13)
#define LTC2991_IN5 	BIT(14)
#define LTC2991_IN6 	BIT(15)
#define LTC2991_IN7 	BIT(16)
#define LTC2991_IN8 	BIT(17)
#define LTC2991_ALL	GENMASK(17, 0)

/* Command 0x01 bits */
#define LTC2991_V7_V8_ENABLE 	BIT(4)
#define LTC2991_V5_V6_ENABLE 	BIT(3)
#define LTC2991_V3_V4_ENABLE 	BIT(2)
#define LTC2991_V1_V2_ENABLE 	BIT(1)
#define LTC2991_TINT_VCC_ENABLE BIT(0)

/* Command 0x06 bits */
#define LTC2991_V3_V4_FILT BIT(7)
#define LTC2991_TR2_KELVIN BIT(6)
#define LTC2991_V3_V4_TEMP BIT(5)
#define LTC2991_V3_V4_DIFF BIT(4)
#define LTC2991_V1_V2_FILT BIT(3)
#define LTC2991_TR1_KELVIN BIT(2)
#define LTC2991_V1_V2_TEMP BIT(1)
#define LTC2991_V1_V2_DIFF BIT(0)

/* Command 0x07 bits */
#define LTC2991_V7_V8_FILT BIT(7)
#define LTC2991_TR4_KELVIN BIT(6)
#define LTC2991_V7_V8_TEMP BIT(5)
#define LTC2991_V7_V8_DIFF BIT(4)
#define LTC2991_V5_V6_FILT BIT(3)
#define LTC2991_TR3_KELVIN BIT(2)
#define LTC2991_V5_V6_TEMP BIT(1)
#define LTC2991_V5_V6_DIFF BIT(0)

/* Command 0x08 bits */
#define LTC2991_PWM_TH0    BIT(5)
#define LTC2991_PWM_INV    BIT(4)
#define LTC2991_PWM_ENABLE BIT(3)
#define LTC2991_REPEAT_ACQ BIT(2)
#define LTC2991_TINT_FILT  BIT(1)
#define LTC2991_TINT_KELVIN BIT(0)

/* Mode masks for commands 0x01, 0x06, 0x07, 0x08, 0x09 */
#define LTC2991_MODE0_SHIFT	3
#define LTC2991_MODE0_MASK	GENMASK(4, 0)
#define LTC2991_MODE1_SHIFT	0
#define LTC2991_MODE1_MASK	GENMASK(7, 0)
#define LTC2991_MODE2_SHIFT	0
#define LTC2991_MODE2_MASK	GENMASK(7, 0)
#define LTC2991_MODE3_SHIFT	2
#define LTC2991_MODE3_MASK	GENMASK(4, 0)
#define LTC2991_MODE4_SHIFT	0
#define LTC2991_MODE4_MASK	GENMASK(7, 0)

struct ltc2991_data {
	struct i2c_client *i2c;
	u32 mode[5];
};

/* Return the converted value from the given register in uV or mC */
static int ltc2991_get_value(struct ltc2991_data *data, int index, int *result)
{
	int val;
	u8 reg;

	switch (index) {
	case LTC2991_TEMP1:
		reg = LTC2991_TINT_MSB;
		break;
	case LTC2991_TEMP2:
	case LTC2991_CURR1:
		reg = LTC2991_V1_MSB;
		break;
	case LTC2991_TEMP3:
	case LTC2991_CURR2:
		reg = LTC2991_V3_MSB;
		break;
	case LTC2991_TEMP4:
	case LTC2991_CURR3:
		reg = LTC2991_V5_MSB;
		break;
	case LTC2991_TEMP5:
	case LTC2991_CURR4:
		reg = LTC2991_V7_MSB;
		break;
	case LTC2991_IN0:
		reg = LTC2991_VCC_MSB;
		break;
	case LTC2991_IN1:
		reg = LTC2991_V1_MSB;
		break;
	case LTC2991_IN2:
		reg = LTC2991_V2_MSB;
		break;
	case LTC2991_IN3:
		reg = LTC2991_V3_MSB;
		break;
	case LTC2991_IN4:
		reg = LTC2991_V4_MSB;
		break;
	case LTC2991_IN5:
		reg = LTC2991_V5_MSB;
		break;
	case LTC2991_IN6:
		reg = LTC2991_V6_MSB;
		break;
	case LTC2991_IN7:
		reg = LTC2991_V7_MSB;
		break;
	case LTC2991_IN8:
		reg = LTC2991_V8_MSB;
		break;
	default:
		return -EINVAL;
	}

	val = i2c_smbus_read_word_swapped(data->i2c, reg) & 0x7FFF; /* Remove DV bit */
	if (unlikely(val < 0))
		return val;

	switch (index) {
	case LTC2991_TEMP1:
	case LTC2991_TEMP2:
	case LTC2991_TEMP3:
	case LTC2991_TEMP4:
	case LTC2991_TEMP5:
		/* temp, 0.0625 degrees/LSB */
		*result = sign_extend32(val, 12) * 1000 / 16;
		break;

	case LTC2991_CURR1:
	case LTC2991_CURR2:
	case LTC2991_CURR3:
	case LTC2991_CURR4:
		 /* Vx-Vy, 19.075uV/LSB */
		*result = sign_extend32(val, 13) * 19075 / 1000;
		break;
	case LTC2991_IN0:
		/* Vcc, 305.18uV/LSB, 2.5V offset */
		*result = sign_extend32(val, 13) * 30518 / (100 * 1000) + 2500;
		break;
	case LTC2991_IN1:
		if ((data->mode[1] & LTC2991_V1_V2_DIFF) > 0)
		{
			/* Vx, 19.075uV/LSB */
			*result = sign_extend32(val, 13) * 19075 / (1000 * 1000);
		}
		else
		{
			/* Vx, 305.18uV/LSB */
			*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		}
		break;
	case LTC2991_IN2:
		/* Vx, 305.18uV/LSB */
		*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		break;
	case LTC2991_IN3:
		if ((data->mode[1] & LTC2991_V3_V4_DIFF) > 0)
		{
			/* Vx, 19.075uV/LSB */
			*result = sign_extend32(val, 13) * 19075 / (1000 * 1000);
		}
		else
		{
			/* Vx, 305.18uV/LSB */
			*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		}
		break;
	case LTC2991_IN4:
		/* Vx, 305.18uV/LSB */
		*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		break;	
	case LTC2991_IN5:
		if ((data->mode[2] & LTC2991_V5_V6_DIFF) > 0)
		{
			/* Vx, 19.075uV/LSB */
			*result = sign_extend32(val, 13) * 19075 / (1000 * 1000);
		}
		else
		{
			/* Vx, 305.18uV/LSB */
			*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		}
		break;
	case LTC2991_IN6:
		/* Vx, 305.18uV/LSB */
		*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		break;	
	case LTC2991_IN7:
		if ((data->mode[2] & LTC2991_V7_V8_DIFF) > 0)
		{
			/* Vx, 19.075uV/LSB */
			*result = sign_extend32(val, 13) * 19075 / (1000 * 1000);
		}
		else
		{
			/* Vx, 305.18uV/LSB */
			*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		}
		break;
	case LTC2991_IN8:
		/* Vx, 305.18uV/LSB */
		*result = sign_extend32(val, 13) * 30518 / (100 * 1000);
		break;	

	default:
		return -EINVAL; /* won't happen, keep compiler happy */
	}

	return 0;
}

static ssize_t ltc2991_show_value(struct device *dev,
				  struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ltc2991_data *data = dev_get_drvdata(dev);
	int value;
	int ret;

	ret = ltc2991_get_value(data, attr->index, &value);
	if (unlikely(ret < 0))
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static umode_t ltc2991_attrs_visible(struct kobject *kobj,
				     struct attribute *a, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ltc2991_data *data = dev_get_drvdata(dev);
	struct device_attribute *da =
			container_of(a, struct device_attribute, attr);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);

	if  ((attr->index == LTC2991_TEMP1) ||
		((attr->index == LTC2991_TEMP2) && ((data->mode[0] & LTC2991_V1_V2_ENABLE) > 0) && ((data->mode[1] & LTC2991_V1_V2_TEMP) > 0)) ||
		((attr->index == LTC2991_TEMP3) && ((data->mode[0] & LTC2991_V3_V4_ENABLE) > 0) && ((data->mode[1] & LTC2991_V3_V4_TEMP) > 0)) ||
		((attr->index == LTC2991_TEMP4) && ((data->mode[0] & LTC2991_V5_V6_ENABLE) > 0) && ((data->mode[2] & LTC2991_V5_V6_TEMP) > 0)) ||
		((attr->index == LTC2991_TEMP5) && ((data->mode[0] & LTC2991_V7_V8_ENABLE) > 0) && ((data->mode[2] & LTC2991_V7_V8_TEMP) > 0)) ||
		((attr->index == LTC2991_CURR1) && ((data->mode[0] & LTC2991_V1_V2_ENABLE) > 0) && ((data->mode[1] & LTC2991_V1_V2_TEMP) == 0) && ((data->mode[1] & LTC2991_V1_V2_DIFF) > 0)) ||
		((attr->index == LTC2991_CURR2) && ((data->mode[0] & LTC2991_V3_V4_ENABLE) > 0) && ((data->mode[1] & LTC2991_V3_V4_TEMP) == 0) && ((data->mode[1] & LTC2991_V3_V4_DIFF) > 0)) ||
		((attr->index == LTC2991_CURR3) && ((data->mode[0] & LTC2991_V5_V6_ENABLE) > 0) && ((data->mode[2] & LTC2991_V5_V6_TEMP) == 0) && ((data->mode[2] & LTC2991_V5_V6_DIFF) > 0)) ||
		((attr->index == LTC2991_CURR4) && ((data->mode[0] & LTC2991_V7_V8_ENABLE) > 0) && ((data->mode[2] & LTC2991_V7_V8_TEMP) == 0) && ((data->mode[2] & LTC2991_V7_V8_DIFF) > 0)) ||
		((attr->index == LTC2991_IN0)   && ((data->mode[0] & LTC2991_TINT_VCC_ENABLE) > 0)) ||
		((attr->index == LTC2991_IN1)   && ((data->mode[0] & LTC2991_V1_V2_ENABLE) > 0) && ((data->mode[1] & LTC2991_V1_V2_TEMP) == 0)) ||
		((attr->index == LTC2991_IN2)   && ((data->mode[0] & LTC2991_V1_V2_ENABLE) > 0) && ((data->mode[1] & LTC2991_V1_V2_TEMP) == 0) && ((data->mode[1] & LTC2991_V1_V2_DIFF) == 0)) ||
		((attr->index == LTC2991_IN3)   && ((data->mode[0] & LTC2991_V3_V4_ENABLE) > 0) && ((data->mode[1] & LTC2991_V3_V4_TEMP) == 0)) ||
		((attr->index == LTC2991_IN4)   && ((data->mode[0] & LTC2991_V3_V4_ENABLE) > 0) && ((data->mode[1] & LTC2991_V3_V4_TEMP) == 0) && ((data->mode[1] & LTC2991_V3_V4_DIFF) == 0)) ||
		((attr->index == LTC2991_IN5)   && ((data->mode[0] & LTC2991_V5_V6_ENABLE) > 0) && ((data->mode[2] & LTC2991_V5_V6_TEMP) == 0)) ||
		((attr->index == LTC2991_IN6)   && ((data->mode[0] & LTC2991_V5_V6_ENABLE) > 0) && ((data->mode[2] & LTC2991_V5_V6_TEMP) == 0) && ((data->mode[2] & LTC2991_V5_V6_DIFF) == 0)) ||
		((attr->index == LTC2991_IN7)   && ((data->mode[0] & LTC2991_V7_V8_ENABLE) > 0) && ((data->mode[2] & LTC2991_V7_V8_TEMP) == 0)) ||
		((attr->index == LTC2991_IN8)   && ((data->mode[0] & LTC2991_V7_V8_ENABLE) > 0) && ((data->mode[2] & LTC2991_V7_V8_TEMP) == 0) && ((data->mode[2] & LTC2991_V7_V8_DIFF) == 0))
		)
		return a->mode;

	return 0;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_TEMP1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_TEMP2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_TEMP3);
static SENSOR_DEVICE_ATTR(temp4_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_TEMP4);
static SENSOR_DEVICE_ATTR(temp5_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_TEMP5);
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_CURR1);
static SENSOR_DEVICE_ATTR(curr2_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_CURR2);
static SENSOR_DEVICE_ATTR(curr3_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_CURR3);
static SENSOR_DEVICE_ATTR(curr4_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_CURR4);
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN7);
static SENSOR_DEVICE_ATTR(in8_input, S_IRUGO, ltc2991_show_value, NULL, LTC2991_IN8);

static struct attribute *ltc2991_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_curr2_input.dev_attr.attr,
	&sensor_dev_attr_curr3_input.dev_attr.attr,
	&sensor_dev_attr_curr4_input.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	&sensor_dev_attr_in8_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltc2991_group = {
	.attrs = ltc2991_attrs,
	.is_visible = ltc2991_attrs_visible,
};
__ATTRIBUTE_GROUPS(ltc2991);

static int ltc2991_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	int ret;
	struct device *hwmon_dev;
	struct ltc2991_data *data;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	data = devm_kzalloc(&i2c->dev, sizeof(struct ltc2991_data), GFP_KERNEL);
	if (unlikely(!data))
		return -ENOMEM;

	data->i2c = i2c;

	if (dev_fwnode(&i2c->dev)) { i2c_smbus_read_byte_data(i2c, LTC2991_ENABLE); /* Force read to detect call to probe */
		ret = device_property_read_u32_array(&i2c->dev,
						     "lltc,meas-mode",
						     data->mode, 5);
		if (ret < 0)
			return ret;
       i2c_smbus_read_byte_data(i2c, LTC2991_ENABLE); /* Force read to detect call to probe */
		if ((data->mode[0] & ~LTC2991_MODE0_MASK) ||
		    (data->mode[1] & ~LTC2991_MODE1_MASK) ||
		    (data->mode[2] & ~LTC2991_MODE2_MASK) ||
		    (data->mode[3] & ~LTC2991_MODE3_MASK) ||
		    (data->mode[4] & ~LTC2991_MODE4_MASK))
			return -EINVAL;
	} else {
		ret = i2c_smbus_read_byte_data(i2c, LTC2991_ENABLE);
		if (ret < 0)
			return ret;

		data->mode[0] = ret >> LTC2991_MODE0_SHIFT & LTC2991_MODE0_MASK;

		ret = i2c_smbus_read_byte_data(i2c, LTC2991_CONTROL_A);
		if (ret < 0)
			return ret;

		data->mode[1] = ret >> LTC2991_MODE1_SHIFT & LTC2991_MODE1_MASK;

		ret = i2c_smbus_read_byte_data(i2c, LTC2991_CONTROL_B);
		if (ret < 0)
			return ret;

		data->mode[2] = ret >> LTC2991_MODE2_SHIFT & LTC2991_MODE2_MASK;

		ret = i2c_smbus_read_byte_data(i2c, LTC2991_CONTROL_C);
		if (ret < 0)
			return ret;

		data->mode[3] = ret >> LTC2991_MODE3_SHIFT & LTC2991_MODE3_MASK;

		ret = i2c_smbus_read_byte_data(i2c, LTC2991_PWM_THRESH);
		if (ret < 0)
			return ret;

		data->mode[4] = ret >> LTC2991_MODE4_SHIFT & LTC2991_MODE4_MASK;
	}

	/* Set configuration */
	ret = i2c_smbus_write_byte_data(i2c, LTC2991_CONTROL_A,
					data->mode[1] << LTC2991_MODE1_SHIFT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set control a mode.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(i2c, LTC2991_CONTROL_B,
					data->mode[2] << LTC2991_MODE2_SHIFT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set control b mode.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(i2c, LTC2991_CONTROL_C,
					data->mode[3] << LTC2991_MODE3_SHIFT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set control c mode.\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(i2c, LTC2991_PWM_THRESH,
					data->mode[4] << LTC2991_MODE4_SHIFT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set pwm threshold mode.\n");
		return ret;
	}
	/* Enable and trigger (mode must be set for repeated acquisition for constant telemetry updating) */
	ret = i2c_smbus_write_byte_data(i2c, LTC2991_ENABLE,
					data->mode[0] << LTC2991_MODE0_SHIFT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set enable mode.\n");
		return ret;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(&i2c->dev,
							   i2c->name,
							   data,
							   ltc2991_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id ltc2991_i2c_id[] = {
	{ "ltc2991", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2991_i2c_id);

static const struct of_device_id ltc2991_i2c_match[] = {
	{
		.compatible = "lltc,ltc2991",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ltc2991_i2c_match);


static struct i2c_driver ltc2991_i2c_driver = {
	.driver = {
		.name = "ltc2991",
		.of_match_table = of_match_ptr(ltc2991_i2c_match),
	},
	.probe    = ltc2991_i2c_probe,
	.id_table = ltc2991_i2c_id,
};

module_i2c_driver(ltc2991_i2c_driver);

MODULE_DESCRIPTION("LTC2991 Sensor Driver");
MODULE_AUTHOR("Topic Embedded Products");
MODULE_LICENSE("GPL v2");
