/*
 *  ast_adc.c
 *
 *  ASPEED ADC controller driver
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.11.26: Initial version [Ryan Chen]
 */

/* attr ADC sysfs 0~max adc channel
*	 0 - show/store enable
*	 3 - show value
*	 1 - show/store alarm_en set enable
*	 2 - show alarm   get statuse
*	 4 - show/store upper
*	 5 - show/store lower  */


#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <plat/regs-adc.h>
#include <plat/ast-scu.h>


#define REST_DESIGN		5


#ifdef CONFIG_YOSEMITE
enum {
  ADC_P5V = 0,
  ADC_P12V,
  ADC_P3V3_STBY,
  ADC_P12V_SLOT0,
  ADC_P12V_SLOT1,
  ADC_P12V_SLOT2,
  ADC_P12V_SLOT3,
  ADC_P3V3,
};

enum {
  REST_DESIGN_P3V3 = 6,
  REST_DESIGN_P5V = 7,
  REST_DESIGN_P12V = 8,
};

#elif CONFIG_LIGHTNING
enum {
  ADC_P12V = 0,
  ADC_P5V,
  ADC_P3V3_STBY,
  ADC_P1V8_STBY,
  ADC_P1V53,
  ADC_P0V9,
  ADC_P0V9_E,
  ADC_P1V26,
};

enum {
  REST_DESIGN_P3V3 = 6,
  REST_DESIGN_P5V = 7,
  REST_DESIGN_P12V = 8,
  REST_DESIGN_NOPOP = 9,
};
#endif // CONFIG_YOSEMITE & CONFIG_LIGHTNING

struct adc_vcc_ref_data {
	int v2;
	int r1;
	int r2;
};

static struct adc_vcc_ref_data adc_vcc_ref[] = {
	[0] = {
		.v2 = 0,
		.r1 = 5600,
		.r2 = 1000,
	},
	[1] = {
		.v2 = -12,
		.r1 = 1000,
		.r2 = 10000,
	},
	[2] = {
		.v2 = 0,
		.r1 = 1800,
		.r2 = 1000,
	},
	[3] = {
		.v2 = -5,
		.r1 = 2200,
		.r2 = 10000,
	},
	[4] = {
		.v2 = 0,
		.r1 = 56000,
		.r2 = 1000,
	},
	[5] = {
		.v2 = 0,
		.r1 = 1000,
		.r2 = 1000,
	},

#ifdef CONFIG_YOSEMITE
  // P3V3
	[6] = {
		.v2 = 0,
		.r1 = 5110,
		.r2 = 8250,
	},
  // P5V
	[7] = {
		.v2 = 0,
		.r1 = 5110,
		.r2 = 3480,
	},
  // P12V
	[8] = {
		.v2 = 0,
		.r1 = 5110,
		.r2 = 1020,
	},

#elif CONFIG_LIGHTNING
  // P3V3
	[6] = {
		.v2 = 0,
		.r1 = 1000,
		.r2 = 1000,
	},
  // P5V
	[7] = {
		.v2 = 0,
		.r1 = 3300,
		.r2 = 1000,
	},
  // P12V
	[8] = {
		.v2 = 0,
		.r1 = 5600,
		.r2 = 1000,
	},
  // NOPOP
	[9] = {
		.v2 = 0,
		.r1 = 0,
		.r2 = 1000,
	},

#endif // CONFIG_YOSEMITE & CONFIG_LIGHTNING

};

/* Divisors for voltage sense;  right now adc5 & adc6 divide by 2 */

static int adc_divisor[] = { 1, 1, 1, 1,
			     1, 2, 2, 1,
			     1, 1, 1, 1,
			     1, 1, 1, 1};

struct ast_adc_data {
	struct device			*hwmon_dev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;				//ADC IRQ number
	int						compen_value;		//Compensating value
};

struct ast_adc_data *ast_adc;

static u8 ast_get_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch);


static inline void
ast_adc_write(struct ast_adc_data *ast_adc, u32 val, u32 reg)
{
//	printk("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_adc->reg_base+ reg);
}

static inline u32
ast_adc_read(struct ast_adc_data *ast_adc, u32 reg)
{
	u32 val = readl(ast_adc->reg_base + reg);
//	printk("read offset: %x, val: %x \n",reg,val);
	return val;
}

static void ast_adc_ctrl_init(void)
{
	u32 pclk;
	ast_adc_write(ast_adc, AST_ADC_CTRL_COMPEN | AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

	//Set wait a sensing cycle t (s) = 1000 * 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000

	pclk = ast_get_pclk();

#if defined(CONFIG_ARCH_AST2300)
	ast_adc_write(ast_adc, 0x3e7, AST_ADC_CLK);

	ast_adc_write(ast_adc, AST_ADC_CTRL_CH12_EN | AST_ADC_CTRL_COMPEN_CLR| ast_adc_read(ast_adc, AST_ADC_CTRL), AST_ADC_CTRL);

	mdelay(50);

	//compensating value = 0x200 - ADC10[9:0]
	if(ast_adc_read(ast_adc, AST_ADC_CH12_13) & (0x1 << 8))
		ast_adc->compen_value = 0x200 - (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK);
	else
		ast_adc->compen_value = 0 - (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK);

	// printk("compensating value %d \n",ast_adc->compen_value);

#elif defined(CONFIG_ARCH_AST2400)

	//For AST2400 A0 workaround  ... ADC0c = 1 ;
//	ast_adc_write(ast_adc, 1, AST_ADC_CLK);
//	ast_adc_write(ast_adc, (0x3e7<< 17) | 0x40, AST_ADC_CLK);
	ast_adc_write(ast_adc, 0x40, AST_ADC_CLK);

	ast_adc_write(ast_adc, AST_ADC_CTRL_CH0_EN | AST_ADC_CTRL_COMPEN | AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

	ast_adc_read(ast_adc, AST_ADC_CTRL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	ast_adc->compen_value = 0x200 - (ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_L_CH_MASK);
	// printk("compensating value %d \n",ast_adc->compen_value);

#elif defined(CONFIG_ARCH_AST2500)
//	TODO ...
//	scu read trim
//	write trim 0xc4 [3:0]

	ast_adc_write(ast_adc, 0x40, AST_ADC_CLK);

	ast_adc_write(ast_adc, AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

    while(!ast_adc_read(ast_adc, AST_ADC_CTRL) & 0x100);

	ast_adc_write(ast_adc, AST_ADC_CTRL_COMPEN  | AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

	while(ast_adc_read(ast_adc, AST_ADC_CTRL) & AST_ADC_CTRL_COMPEN);

	//compensating value = 0x200 - ADC10[9:0]
	ast_adc->compen_value = 0x200 - ((ast_adc_read(ast_adc, AST_ADC_TRIM) >> 16) & 0x3ff);
	// printk("compensating value %d \n",ast_adc->compen_value);

#else
#err "No define for ADC "
#endif

	ast_adc_write(ast_adc, AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

}

static u16
ast_get_adc_hyster_lower(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_L_BOUND;

//	printk("read val = %d \n",tmp);

	return tmp;

}

static void
ast_set_adc_hyster_lower(struct ast_adc_data *ast_adc, u8 adc_ch, u16 value)
{
	ast_adc_write(ast_adc,
			(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_L_BOUND) |
			value,
			AST_ADC_HYSTER0 + (adc_ch *4));

}

static u16
ast_get_adc_hyster_upper(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_H_BOUND) >> 16);

//	printk("read val = %d \n",tmp);

	return tmp;
}

static void
ast_set_adc_hyster_upper(struct ast_adc_data *ast_adc, u8 adc_ch, u32 value)
{
	ast_adc_write(ast_adc,
			(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_H_BOUND) |
			(value << 16),
			AST_ADC_HYSTER0 + (adc_ch *4));

}

static u8
ast_get_adc_hyster_en(struct ast_adc_data *ast_adc, u8 adc_ch)
{
	//tacho source
	if(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_HYSTER_EN)
		return 1;
	else
		return 0;
}

static void
ast_set_adc_hyster_en(struct ast_adc_data *ast_adc, u8 adc_ch, u8 enable)
{
	//tacho source
	if(enable == 1)
		ast_adc_write(ast_adc,
			ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) | AST_ADC_HYSTER_EN,
			AST_ADC_HYSTER0 + (adc_ch *4));
	else
		ast_adc_write(ast_adc,
			ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_HYSTER_EN,
			AST_ADC_HYSTER0 + (adc_ch *4));
}

static u16
ast_get_adc_lower(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & AST_ADC_L_BOUND;

//	printk("read val = %d \n",tmp);

	return tmp;

}

static void
ast_set_adc_lower(struct ast_adc_data *ast_adc, u8 adc_ch, u16 value)
{
	ast_adc_write(ast_adc,
			(ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & ~AST_ADC_L_BOUND) |
			value,
			AST_ADC_BOUND0 + (adc_ch *4));

}

static u16
ast_get_adc_upper(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & AST_ADC_H_BOUND) >> 16);

	printk("read val = %d \n",tmp);

	return tmp;


}

static void
ast_set_adc_upper(struct ast_adc_data *ast_adc, u8 adc_ch, u32 value)
{
	ast_adc_write(ast_adc,
			(ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & ~AST_ADC_H_BOUND) |
			(value << 16),
			AST_ADC_BOUND0 + (adc_ch *4));

}


static u8
ast_get_adc_alarm(struct ast_adc_data *ast_adc, u8 adc_ch)
{
	//adc ch source
	if(ast_adc_read(ast_adc, AST_ADC_IER) & (0x1 << adc_ch))
		return 1;
	else
		return 0;
}

static u16
ast_get_adc_value(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    int tmp;

	switch(adc_ch) {
		case 0:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_L_CH_MASK;
			break;
		case 1:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 2:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH2_3) & AST_ADC_L_CH_MASK;
			break;
		case 3:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH2_3) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 4:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH4_5) & AST_ADC_L_CH_MASK;
			break;
		case 5:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH4_5) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 6:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH6_7) & AST_ADC_L_CH_MASK;
			break;
		case 7:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH6_7) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 8:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH8_9) & AST_ADC_L_CH_MASK;
			break;
		case 9:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH8_9) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 10:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH10_11) & AST_ADC_L_CH_MASK;
			break;
		case 11:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH10_11) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 12:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK;
			break;
		case 13:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_H_CH_MASK) >> 16;
			break;
		case 14:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH14_15) & AST_ADC_L_CH_MASK;
			break;
		case 15:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH14_15) & AST_ADC_H_CH_MASK) >> 16;
			break;

	}

	tmp += ast_adc->compen_value;

//	printk("voltage = %d \n",tmp);

	return tmp;

}

static u8
ast_get_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u8 tmp=0;

	if(ast_adc_read(ast_adc, AST_ADC_CTRL) & (0x1 << (16+adc_ch)))
		tmp = 1;
	else
		tmp = 0;

	return tmp;

}

static void
ast_set_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch, u8 enable)
{
	if(enable)
		ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) | (0x1 << (16+adc_ch)), AST_ADC_CTRL);
	else
		ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) & ~(0x1 << (16+adc_ch)), AST_ADC_CTRL);
}


/* NAME sysfs */
static ssize_t
show_name(struct device *dev, struct device_attribute *devattr,
                         char *buf)
{
  return sprintf(buf, "ast_adc\n");
}
static SENSOR_DEVICE_ATTR_2(name, S_IRUGO, show_name, NULL, 0, 0);
static struct attribute *name_attributes[] = {
	&sensor_dev_attr_name.dev_attr.attr,
	NULL
};
static const struct attribute_group name_attribute_groups = {
	.attrs = name_attributes,
};

/* attr ADC sysfs 0~max adc channel
*	 0 - show/store channel enable
*	 1 - show value
*	 2 - show alarm   get statuse
*	 3 - show/store upper
*	 4 - show/store lower
*	 5 - show/store hystersis enable
*	 6 - show/store hystersis upper
*	 7 - show/store hystersis low
*/

static u32
ast_get_voltage(int idx) {
  u8 rest_design = REST_DESIGN;
  u16 tmp;
  u32 voltage, tmp1, tmp2, tmp3;
  tmp = ast_get_adc_value(ast_adc, idx);

#ifdef CONFIG_YOSEMITE
  switch (idx) {
  case ADC_P3V3:
  case ADC_P3V3_STBY:
    rest_design = REST_DESIGN_P3V3;
    break;
  case ADC_P5V:
    rest_design = REST_DESIGN_P5V;
    break;
  case ADC_P12V:
  case ADC_P12V_SLOT0:
  case ADC_P12V_SLOT1:
  case ADC_P12V_SLOT2:
  case ADC_P12V_SLOT3:
    rest_design = REST_DESIGN_P12V;
    break;
  }
#elif CONFIG_LIGHTNING
  switch (idx) {
  case ADC_P12V:
    rest_design = REST_DESIGN_P12V;
    break;
  case ADC_P5V:
    rest_design = REST_DESIGN_P5V;
    break;
  case ADC_P3V3_STBY:
    rest_design = REST_DESIGN_P3V3;
    break;
  case ADC_P1V8_STBY:
  case ADC_P1V53:
  case ADC_P0V9:
  case ADC_P0V9_E:
  case ADC_P1V26:
    rest_design = REST_DESIGN_NOPOP;
    break;
  }
#endif // CONFIG_YOSEMITE & CONFIG_LIGHTNING

  // Voltage Sense Method
  tmp1 = (adc_vcc_ref[rest_design].r1 + adc_vcc_ref[rest_design].r2) * tmp * 25 * 10;
  tmp2 = adc_vcc_ref[rest_design].r2 * 1024 ;
  tmp3 = (adc_vcc_ref[rest_design].r1 * adc_vcc_ref[rest_design].v2) / adc_vcc_ref[rest_design].r2;
  voltage = (tmp1/tmp2) - tmp3;

#if !defined(CONFIG_YOSEMITE) && !defined(CONFIG_LIGHTNING)
  // Higher voltage inputs require a divisor

  if (adc_divisor[idx])
	voltage /= adc_divisor[idx];
#endif //CONFIG_YOSEMITE & CONFIG_LIGHTNING

  return voltage;
}

static ssize_t
ast_show_adc(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);
	u32 voltage;

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr)
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_adc_en(ast_adc,sensor_attr->index),ast_get_adc_en(ast_adc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //value
			voltage = ast_get_voltage(sensor_attr->index);
			return sprintf(sysfsbuf, "%d.%02d (V)\n",voltage/100, voltage%100);
			break;
		case 2: //alarm
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_alarm(ast_adc,sensor_attr->index));
			break;
		case 3: //upper
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_upper(ast_adc,sensor_attr->index));
			break;
		case 4: //lower
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_lower(ast_adc,sensor_attr->index));
			break;
		case 5: //hystersis enable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_adc_hyster_en(ast_adc,sensor_attr->index),ast_get_adc_hyster_en(ast_adc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 6: //hystersis upper
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_hyster_upper(ast_adc,sensor_attr->index));
			break;
		case 7: //hystersis lower
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_hyster_lower(ast_adc,sensor_attr->index));
			break;
		case 8:
			voltage = ast_get_voltage(sensor_attr->index);
			return sprintf(sysfsbuf, "%d\n",voltage * 10);

		default:
			return -EINVAL;
			break;
	}
}

static ssize_t
ast_store_adc(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr)
	{
		case 0: //enable, disable
			ast_set_adc_en(ast_adc, sensor_attr->index, input_val);
			break;
		case 1: //value

			break;
		case 2: //alarm
			break;
		case 3:
			ast_set_adc_upper(ast_adc, sensor_attr->index, input_val);
			break;
		case 4:
			ast_set_adc_lower(ast_adc, sensor_attr->index, input_val);
			break;
		case 5: //hystersis
			ast_set_adc_hyster_en(ast_adc, sensor_attr->index, input_val);
			break;
		case 6:
			ast_set_adc_hyster_upper(ast_adc, sensor_attr->index, input_val);
			break;
		case 7:
			ast_set_adc_hyster_lower(ast_adc, sensor_attr->index, input_val);
			break;

		default:
			return -EINVAL;
			break;
	}

	return count;
}

/* attr ADC sysfs 0~max adc channel
*	 0 - show/store channel enable
*	 1 - show value
*	 2 - show alarm   get statuse
*	 3 - show/store upper
*	 4 - show/store lower
*	 5 - show/store hystersis enable
*	 6 - show/store hystersis upper
*	 7 - show/store hystersis low
*	 8 - show value as 1000s, expected by lm-sensors
*/

#define sysfs_adc_ch(index) \
static SENSOR_DEVICE_ATTR_2(adc##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 0, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_value, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 1, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_alarm, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 2, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_upper, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 3, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_lower, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 4, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_en, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 5, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_upper, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 6, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_lower, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 7, index); \
\
static SENSOR_DEVICE_ATTR_2(in##index##_input, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 8, index); \
\
static struct attribute *adc##index##_attributes[] = { \
	&sensor_dev_attr_adc##index##_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_value.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_alarm.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_upper.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_lower.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_upper.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_lower.dev_attr.attr, \
	&sensor_dev_attr_in##index##_input.dev_attr.attr, \
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_adc_ch(0);
sysfs_adc_ch(1);
sysfs_adc_ch(2);
sysfs_adc_ch(3);
sysfs_adc_ch(4);
sysfs_adc_ch(5);
sysfs_adc_ch(6);
sysfs_adc_ch(7);
sysfs_adc_ch(8);
sysfs_adc_ch(9);
sysfs_adc_ch(10);
sysfs_adc_ch(11);
#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
sysfs_adc_ch(12);
sysfs_adc_ch(13);
sysfs_adc_ch(14);
sysfs_adc_ch(15);
#endif

static const struct attribute_group adc_attribute_groups[] = {
	{ .attrs = adc0_attributes },
	{ .attrs = adc1_attributes },
	{ .attrs = adc2_attributes },
	{ .attrs = adc3_attributes },
	{ .attrs = adc4_attributes },
	{ .attrs = adc5_attributes },
	{ .attrs = adc6_attributes },
	{ .attrs = adc7_attributes },
	{ .attrs = adc8_attributes },
	{ .attrs = adc9_attributes },
	{ .attrs = adc10_attributes },
	{ .attrs = adc11_attributes },
#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
	{ .attrs = adc12_attributes },
	{ .attrs = adc13_attributes },
	{ .attrs = adc14_attributes },
	{ .attrs = adc15_attributes },
#endif
};


static int
ast_adc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "ast_adc_probe \n");

	ast_adc = kzalloc(sizeof(struct ast_adc_data), GFP_KERNEL);
	if (!ast_adc) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_mem;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out_mem;
	}

	ast_adc->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_adc->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_adc->irq = platform_get_irq(pdev, 0);
	if (ast_adc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	/* Register sysfs hooks */
	ast_adc->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ast_adc->hwmon_dev)) {
		ret = PTR_ERR(ast_adc->hwmon_dev);
		goto out_region;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &name_attribute_groups);
	if (err)
		goto out_region;

	for(i=0; i<MAX_CH_NO; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &adc_attribute_groups[i]);
		if (err)
			goto out_sysfs00;
	}

	ast_adc_ctrl_init();

	printk(KERN_INFO "ast_adc: driver successfully loaded.\n");

	return 0;


//out_irq:
//	free_irq(ast_adc->irq, NULL);
out_sysfs00:
	sysfs_remove_group(&pdev->dev.kobj, &name_attribute_groups);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out_mem:
	kfree(ast_adc);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int
ast_adc_remove(struct platform_device *pdev)
{
	int i=0;
	struct ast_adc_data *ast_adc = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "ast_adc: driver unloaded.\n");

    hwmon_device_unregister(ast_adc->hwmon_dev);

	for(i=0; i<5; i++)
		sysfs_remove_group(&pdev->dev.kobj, &adc_attribute_groups[i]);

	sysfs_remove_group(&pdev->dev.kobj, &name_attribute_groups);

	platform_set_drvdata(pdev, NULL);
//	free_irq(ast_adc->irq, ast_adc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ast_adc->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(ast_adc);
	return 0;
}

#ifdef CONFIG_PM
static int
ast_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_adc_suspend : TODO \n");
	return 0;
}

static int
ast_adc_resume(struct platform_device *pdev)
{
	ast_adc_ctrl_init();
	return 0;
}

#else
#define ast_adc_suspend        NULL
#define ast_adc_resume         NULL
#endif

static struct platform_driver ast_adc_driver = {
	.probe			= ast_adc_probe,
	.remove 		= __devexit_p(ast_adc_remove),
    .suspend        = ast_adc_suspend,
    .resume         = ast_adc_resume,
    .driver         = {
            .name   = "ast_adc",
            .owner  = THIS_MODULE,
    },
};

static int __init
ast_adc_init(void)
{
	return platform_driver_register(&ast_adc_driver);
}

static void __exit
ast_adc_exit(void)
{
	platform_driver_unregister(&ast_adc_driver);
}

module_init(ast_adc_init);
module_exit(ast_adc_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ADC driver");
MODULE_LICENSE("GPL");
