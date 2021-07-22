/*
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define ASPEED_PWM_CTRL			0x00	//PWM0 General Register
#define ASPEED_PWM_CTRL_CH(x)		((x * 0x10) + 0x00)
#define  PWM_LOAD_AS_WDT			BIT(19)	//load selection as WDT
#define  PWM_DUTY_LOAD_AS_WDT_EN	BIT(18)	//enable PWM duty load as WDT
#define  PWM_DUTY_SYNC_DIS			BIT(17)	//disable PWM duty sync
#define	 PWM_CLK_ENABLE				BIT(16)	//enable PWM clock
#define  PWM_LEVEL_OUTPUT			BIT(15)	//output PWM level
#define  PWM_INVERSE				BIT(14) //inverse PWM pin
#define  PWM_OPEN_DRAIN_EN			BIT(13)	//enable open-drain
#define  PWM_PIN_EN					BIT(12) //enable PWM pin
#define  PWM_CLK_DIV_H_MASK			(0xf << 8)	//PWM clock division H bit [3:0]
#define  PWM_CLK_DIV_L_MASK			(0xff)	//PWM clock division H bit [3:0]

/*
\xregmid {11:8 }{RW}{PWM clock division H bit [3:0]}{
                     0: divide 1 \n
                     1: divide 2 \n
                     2: divide 4 \n
                     3: divide 8 \n
                     ... \n
                     F: divide 32768}
\xregmid {7 :0 }{RW}{PWM clock division L bit [7:0]}{
                     00: divide 1 \n
                     01: divide 2 \n
                     02: divide 3 \n
                     03: divide 4 \n
                     ... \n
                     FF: divide 256}
*/

#define ASPEED_PWM_DUTY_CYCLE		0x04	//PWM0 Duty Cycle Register
#define ASPEED_PWM_DUTY_CYCLE_CH(x)		((x * 0x10) + 0x04)
#define  PWM_LOOP_BIT_MASK				(0xf << 24)	//loop bit [7:0]
#define  PWM_PERIOD_BIT					(24)	//pwm period bit [7:0]
#define  PWM_PERIOD_BIT_MASK			(0xff << 24)	//pwm period bit [7:0]
#define  PWM_RISING_FALLING_AS_WDT_BIT  (16)	
#define  PWM_RISING_FALLING_AS_WDT_MASK (0xff << 16)	//pwm rising/falling point bit [7:0] as WDT
#define  PWM_RISING_FALLING_MASK		(0xffff)	
#define  PWM_RISING_FALLING_BIT			(8)	//pwm falling point bit [7:0]
#define  PWM_RISING_RISING_BIT			(0)	//pwm rising point bit [7:0]

#define  DEF_PWM_PERIOD 0xff


#define ASPEED_TECHO_CTRL		0x08	//TACH0 General Register
#define ASPEED_TECHO_CTRL_CH(x)			((x * 0x10) + 0x08)
#define  TECHO_IER						BIT(31)	//enable tacho interrupt
#define  TECHO_INVERS_LIMIT				BIT(30) //inverse tacho limit comparison
#define  TECHO_LOOPBACK					BIT(29) //tacho loopback
#define  TECHO_ENABLE					BIT(28)	//{enable tacho}
#define  TECHO_DEBOUNCE_BIT				(26) //{tacho de-bounce}
#define  TECHO_DEBOUNCE_MASK			(0x3 << 26) //{tacho de-bounce}
#define  TECHIO_EDGE_MASK				(0x3 << 24) //tacho edge}
#define  TECHIO_EDGE_BIT				(24) //tacho edge}
#define  TECHO_CLK_DIV_T_MASK			(0xf << 20) 
#define  TECHO_CLK_DIV_BIT				(20)
#define  TECHO_THRESHOLD_MASK			(0xfffff)	//tacho threshold bit
/*
\xregmid {23:20}{RW}{tacho clock division T bit [3:0]}{
                     0: divide 1 \n
                     1: divide 4 \n
                     2: divide 16 \n
                     3: divide 64 \n
                     ... \n
                     B: divide 4194304 \n
					 others: reserved}
\xregmidb{19 :0 }{RW}{tacho threshold bit [19:0]}
*/

#define ASPEED_TECHO_STS		0x0C	//TACH0 Status Register
#define ASPEED_TECHO_STS_CH(x)			((x * 0x10) + 0x0C)
#define  TECHO_ISR			BIT(31)	//interrupt status and clear
#define  PWM_OUT			BIT(25)	//{pwm_out}
#define  PWM_OEN			BIT(24)	//{pwm_oeN}
#define  TACHO_DEB_INPUT	BIT(23)	//tacho deB input
#define  TACHO_RAW_INPUT	BIT(22) //tacho raw input}
#define  TACHO_VALUE_UPDATE	BIT(21)	//tacho value updated since the last read
#define  TACHO_FULL_MEASUREMENT	BIT(20) //{tacho full measurement}
#define  TACHO_VALUE_MASK	0xfffff	//tacho value bit [19:0]}

#define MAX_CDEV_NAME_LEN 16

struct aspeed_pwm_channel_params {
	int target_freq;
	int pwm_freq;
	int load_wdt_rising_falling_pt;
	int load_wdt_selection;		//0: rising , 1: falling
	int load_wdt_enable;
	int	duty_sync_enable;
	int invert_pin;
	u8	rising;
	u8	falling;
};

static struct aspeed_pwm_channel_params default_pwm_params[] = {
	[0] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,
		.load_wdt_selection = 0,
		.load_wdt_enable = 1,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
		
	},
	[1] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[2] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[3] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[4] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[5] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[6] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[7] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[8] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[9] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[10] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[11] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[12] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[13] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[14] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
	[15] = {
		.target_freq = 25000,
		.load_wdt_rising_falling_pt = 0x10,	
		.load_wdt_selection = 0,
		.load_wdt_enable = 0,
		.duty_sync_enable = 0,
		.invert_pin = 0,
		.rising = 0x00,
		.falling = 0x0a,
	},
};

/*
 * 5:4 fan tach edge mode selection bit:
 * 00: falling
 * 01: rising
 * 10: both
 * 11: reserved.
 */

#define F2F_EDGES 0x00 /* 10b */
#define R2R_EDGES 0x01 /* 10b */
#define BOTH_EDGES 0x02 /* 10b */

struct aspeed_techo_channel_params {
	u32 min_rpm;
	int limited_inverse;
	u16 threshold;
	u8	tacho_edge;
	u8	tacho_debounce;	
	u32	divide;
};


static struct aspeed_techo_channel_params default_techo_params[] = {
	[0] = {
		.min_rpm = 2900,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,
		.divide = 8,
	},
	[1] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[2] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[3] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[4] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[5] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[6] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[7] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[8] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[9] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[10] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[11] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[12] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[13] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[14] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
	[15] = {
		.min_rpm = 2000,
		.limited_inverse = 0,
		.threshold = 0,
		.tacho_edge = F2F_EDGES,
		.tacho_debounce = 0,		
		.divide = 8,
	},
};

struct aspeed_pwm_tachometer_data {
	struct regmap *regmap;
	unsigned long clk_freq;
	struct reset_control *reset;	
	bool pwm_present[16];
	bool fan_tach_present[16];
	struct aspeed_pwm_channel_params *pwm_channel;
	struct aspeed_techo_channel_params *techo_channel;
	struct aspeed_cooling_device *cdev[8];
	const struct attribute_group *groups[3];
};

struct aspeed_cooling_device {
	char name[16];
	struct aspeed_pwm_tachometer_data *priv;
	struct thermal_cooling_device *tcdev;
	int pwm_channel;
	u8 *cooling_levels;
	u8 max_state;
	u8 cur_state;
};

static int regmap_aspeed_pwm_tachometer_reg_write(void *context, unsigned int reg,
					     unsigned int val)
{
	void __iomem *regs = (void __iomem *)context;

	writel(val, regs + reg);
	return 0;
}

static int regmap_aspeed_pwm_tachometer_reg_read(void *context, unsigned int reg,
					    unsigned int *val)
{
	void __iomem *regs = (void __iomem *)context;

	*val = readl(regs + reg);
	return 0;
}

static const struct regmap_config aspeed_pwm_tachometer_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x100,
	.reg_write = regmap_aspeed_pwm_tachometer_reg_write,
	.reg_read = regmap_aspeed_pwm_tachometer_reg_read,
	.fast_io = true,
};

static void aspeed_set_pwm_channel_enable(struct regmap *regmap, u8 pwm_channel,
				       bool enable)
{
	regmap_update_bits(regmap, ASPEED_PWM_CTRL_CH(pwm_channel), (PWM_CLK_ENABLE | PWM_PIN_EN), enable ? (PWM_CLK_ENABLE | PWM_PIN_EN) : 0);
}

static void aspeed_set_fan_tach_ch_enable(struct aspeed_pwm_tachometer_data *priv, u8 fan_tach_ch,
					  bool enable)
{
	u32 i = 0;
	u32 divide_val = 0;
	u32 target_div = 0;
	u32 reg_value = 0;

	if(enable) {
		//4 ^ n
//		printk("=== %ld \n", (priv->clk_freq * 60 / priv->techo_channel[fan_tach_ch].min_rpm * 2));
		target_div = (priv->clk_freq / priv->techo_channel[fan_tach_ch].min_rpm * 60 * 2) / (0xfffff + 1);
//		printk("min_rpm %d , target_div %d \n", priv->techo_channel[fan_tach_ch].min_rpm, target_div);
		if(target_div) {
			for(i = 0; i < 12; i++) {
				divide_val = BIT(i) * BIT(i);
				if(divide_val > target_div)
					break;

			}
//			printk("xx i %d divide_val %x \n", i, divide_val);
			//add some toleerance for aviod fan/board design glitch
			if(i <= 2)
				i = 3;
			divide_val = BIT(i) * BIT(i);
//			printk("00 i %d divide_val %x \n", i, divide_val);
		} else {
			i = 0;
			divide_val = 1;
		}
		priv->techo_channel[fan_tach_ch].divide = divide_val;
//		printk("i : %d ,target_div %d, divide_val %d, priv->clk_freq/divide_val %ld ", i, target_div, divide_val, priv->clk_freq/divide_val);		

		reg_value = TECHO_ENABLE | 
				(priv->techo_channel[fan_tach_ch].tacho_edge << TECHIO_EDGE_BIT) |
				(i << TECHO_CLK_DIV_BIT) |
				(priv->techo_channel[fan_tach_ch].tacho_debounce << TECHO_DEBOUNCE_BIT);

		if(priv->techo_channel[fan_tach_ch].limited_inverse)
			reg_value |= TECHO_INVERS_LIMIT;

		if(priv->techo_channel[fan_tach_ch].threshold)
			reg_value |= (TECHO_IER | priv->techo_channel[fan_tach_ch].threshold); 

		regmap_write(priv->regmap, ASPEED_TECHO_CTRL_CH(fan_tach_ch), reg_value);
	} else
		regmap_update_bits(priv->regmap, ASPEED_TECHO_CTRL_CH(fan_tach_ch),  TECHO_ENABLE, 0);
}

/*
 * The PWM frequency = HCLK(200Mhz) / (clock division L bit *
 * clock division H bit * (period bit + 1))
 */
static void aspeed_set_pwm_channel_fan_ctrl(struct aspeed_pwm_tachometer_data *priv,
					 u8 index, u8 fan_ctrl)
{
	u32 duty_value,	ctrl_value;
	u32 div_h, div_l, cal_freq;

	if (fan_ctrl == 0) {
		aspeed_set_pwm_channel_enable(priv->regmap, index, false);
	} else {
		cal_freq = priv->clk_freq / (DEF_PWM_PERIOD + 1);
		//calculate for target frequence 
		for(div_l = 0; div_l < 0x100; div_l++) {
			for(div_h = 0; div_h < 0x10; div_h++) {
//				printk("div h %x, l : %x , freq %ld \n", div_h, div_l, (cal_freq / (BIT(div_h) * (div_l + 1))));
				if((cal_freq / (BIT(div_h) * (div_l + 1))) < priv->pwm_channel[index].target_freq)
					break;
			}
			if((cal_freq / (BIT(div_h) * (div_l + 1))) < priv->pwm_channel[index].target_freq)
				break;
		}

		priv->pwm_channel[index].pwm_freq = cal_freq / (BIT(div_h) * (div_l + 1));
//		printk("div h %x, l : %x pwm out clk %d \n", div_h, div_l, priv->pwm_channel[index].pwm_freq);
//		printk("hclk %ld, target pwm freq %d, real pwm freq %d\n", priv->clk_freq, priv->pwm_channel[index].target_freq, priv->pwm_channel[index].pwm_freq);

		ctrl_value = (div_h << 8) | div_l;

		duty_value = (DEF_PWM_PERIOD << PWM_PERIOD_BIT) | 
					(0 << PWM_RISING_RISING_BIT) | (fan_ctrl << PWM_RISING_FALLING_BIT);

		if (priv->pwm_channel[index].load_wdt_enable) {
			ctrl_value |= PWM_DUTY_LOAD_AS_WDT_EN;
			if(priv->pwm_channel[index].load_wdt_selection) {
				ctrl_value |= PWM_LOAD_AS_WDT;
				duty_value |= (priv->pwm_channel[index].load_wdt_rising_falling_pt << PWM_RISING_FALLING_AS_WDT_BIT);
			} else {
				duty_value |= (priv->pwm_channel[index].load_wdt_rising_falling_pt << PWM_RISING_FALLING_AS_WDT_BIT);
			}
		}

		regmap_write(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index), duty_value);
		regmap_write(priv->regmap, ASPEED_PWM_CTRL_CH(index), ctrl_value);

		aspeed_set_pwm_channel_enable(priv->regmap, index, true);
	}
}

static int aspeed_get_fan_tach_ch_rpm(struct aspeed_pwm_tachometer_data *priv,
				      u8 fan_tach_ch)
{
	u32 raw_data, tach_div, clk_source, val;
	u8 two_plus = 2;
	int i, retries = 3;

	for(i = 0; i < retries; i++) {
		regmap_read(priv->regmap, ASPEED_TECHO_STS_CH(fan_tach_ch), &val);
		if (TACHO_FULL_MEASUREMENT & val)
			break;
	}

	raw_data = val & TACHO_VALUE_MASK;
	if(raw_data == 0xfffff)
		return 0;

	raw_data += 1;

	/*
	 * We need the mode to determine if the raw_data is double (from
	 * counting both edges).
	 */
	tach_div = raw_data * (two_plus);

//	printk("clk %ld, raw_data %d , tach_div %d  \n", priv->clk_freq, raw_data, tach_div);
	
	clk_source = priv->clk_freq;

	if (raw_data == 0)
		return 0;

	return ((clk_source / tach_div) * 60 / (priv->techo_channel[fan_tach_ch].divide));

}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int ret;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);
	long fan_ctrl;
	u8 org_falling = priv->pwm_channel[index].falling;

	ret = kstrtol(buf, 10, &fan_ctrl);
	if (ret != 0)
		return ret;

	if (fan_ctrl < 0 || fan_ctrl > DEF_PWM_PERIOD)
		return -EINVAL;

	if (priv->pwm_channel[index].falling == fan_ctrl)
		return count;

	priv->pwm_channel[index].falling = fan_ctrl;

	if(fan_ctrl == 0)
		aspeed_set_pwm_channel_enable(priv->regmap, index, false);
	else
		regmap_update_bits(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index), GENMASK(15, 8), (fan_ctrl << PWM_RISING_FALLING_BIT));

	if(org_falling == 0)
		aspeed_set_pwm_channel_enable(priv->regmap, index, true);

	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", priv->pwm_channel[index].falling);
}

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int rpm;
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	rpm = aspeed_get_fan_tach_ch_rpm(priv, index);
	if (rpm < 0)
		return rpm;

	return sprintf(buf, "%d\n", rpm);
}

static umode_t pwm_is_visible(struct kobject *kobj,
			      struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	if (!priv->pwm_present[index])
		return 0;
	return a->mode;
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_pwm_tachometer_data *priv = dev_get_drvdata(dev);

	if (!priv->fan_tach_present[index])
		return 0;
	return a->mode;
}

static SENSOR_DEVICE_ATTR(pwm0, 0644,
			show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm1, 0644,
			show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm2, 0644,
			show_pwm, set_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm3, 0644,
			show_pwm, set_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm4, 0644,
			show_pwm, set_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm5, 0644,
			show_pwm, set_pwm, 5);
static SENSOR_DEVICE_ATTR(pwm6, 0644,
			show_pwm, set_pwm, 6);
static SENSOR_DEVICE_ATTR(pwm7, 0644,
			show_pwm, set_pwm, 7);
static SENSOR_DEVICE_ATTR(pwm8, 0644,
			show_pwm, set_pwm, 8);
static SENSOR_DEVICE_ATTR(pwm9, 0644,
			show_pwm, set_pwm, 9);
static SENSOR_DEVICE_ATTR(pwm10, 0644,
			show_pwm, set_pwm, 10);
static SENSOR_DEVICE_ATTR(pwm11, 0644,
			show_pwm, set_pwm, 11);
static SENSOR_DEVICE_ATTR(pwm12, 0644,
			show_pwm, set_pwm, 12);
static SENSOR_DEVICE_ATTR(pwm13, 0644,
			show_pwm, set_pwm, 13);
static SENSOR_DEVICE_ATTR(pwm14, 0644,
			show_pwm, set_pwm, 14);
static SENSOR_DEVICE_ATTR(pwm15, 0644,
			show_pwm, set_pwm, 15);
static struct attribute *pwm_dev_attrs[] = {
	&sensor_dev_attr_pwm0.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	&sensor_dev_attr_pwm7.dev_attr.attr,
	&sensor_dev_attr_pwm8.dev_attr.attr,
	&sensor_dev_attr_pwm9.dev_attr.attr,
	&sensor_dev_attr_pwm10.dev_attr.attr,
	&sensor_dev_attr_pwm11.dev_attr.attr,
	&sensor_dev_attr_pwm12.dev_attr.attr,
	&sensor_dev_attr_pwm13.dev_attr.attr,
	&sensor_dev_attr_pwm14.dev_attr.attr,
	&sensor_dev_attr_pwm15.dev_attr.attr,
	NULL,
};

static const struct attribute_group pwm_dev_group = {
	.attrs = pwm_dev_attrs,
	.is_visible = pwm_is_visible,
};

static SENSOR_DEVICE_ATTR(fan0_input, 0444,
		show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan1_input, 0444,
		show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan2_input, 0444,
		show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan3_input, 0444,
		show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan4_input, 0444,
		show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan5_input, 0444,
		show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan6_input, 0444,
		show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan7_input, 0444,
		show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan8_input, 0444,
		show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan9_input, 0444,
		show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan10_input, 0444,
		show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan11_input, 0444,
		show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan12_input, 0444,
		show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan13_input, 0444,
		show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan14_input, 0444,
		show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan15_input, 0444,
		show_rpm, NULL, 15);
static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan0_input.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	&sensor_dev_attr_fan7_input.dev_attr.attr,
	&sensor_dev_attr_fan8_input.dev_attr.attr,
	&sensor_dev_attr_fan9_input.dev_attr.attr,
	&sensor_dev_attr_fan10_input.dev_attr.attr,
	&sensor_dev_attr_fan11_input.dev_attr.attr,
	&sensor_dev_attr_fan12_input.dev_attr.attr,
	&sensor_dev_attr_fan13_input.dev_attr.attr,
	&sensor_dev_attr_fan14_input.dev_attr.attr,
	&sensor_dev_attr_fan15_input.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

static void aspeed_create_pwm_channel(struct aspeed_pwm_tachometer_data *priv,
				   u8 pwm_channel)
{
	priv->pwm_present[pwm_channel] = true;

	//use default 
	aspeed_set_pwm_channel_fan_ctrl(priv, pwm_channel, priv->pwm_channel[pwm_channel].falling);
}

static void aspeed_create_fan_tach_channel(struct aspeed_pwm_tachometer_data *priv,
					   u8 *fan_tach_ch, int count, u32 min_rpm)
{
	u8 val, index;

	for (val = 0; val < count; val++) {
		index = fan_tach_ch[val];
		priv->fan_tach_present[index] = true;
		priv->techo_channel[index].min_rpm = min_rpm;
		aspeed_set_fan_tach_ch_enable(priv, index, true);
	}
}

static int
aspeed_pwm_cz_get_max_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	*state = cdev->max_state;

	return 0;
}

static int
aspeed_pwm_cz_get_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	*state = cdev->cur_state;

	return 0;
}

static int
aspeed_pwm_cz_set_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	if (state > cdev->max_state)
		return -EINVAL;

	cdev->cur_state = state;
	cdev->priv->pwm_channel[cdev->pwm_channel].falling =
					cdev->cooling_levels[cdev->cur_state];
	aspeed_set_pwm_channel_fan_ctrl(cdev->priv, cdev->pwm_channel,
				     cdev->cooling_levels[cdev->cur_state]);

	return 0;
}

static const struct thermal_cooling_device_ops aspeed_pwm_cool_ops = {
	.get_max_state = aspeed_pwm_cz_get_max_state,
	.get_cur_state = aspeed_pwm_cz_get_cur_state,
	.set_cur_state = aspeed_pwm_cz_set_cur_state,
};

static int aspeed_create_pwm_cooling(struct device *dev,
				     struct device_node *child,
				     struct aspeed_pwm_tachometer_data *priv,
				     u32 pwm_channel, u8 num_levels)
{
	int ret;
	struct aspeed_cooling_device *cdev;

	cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	cdev->cooling_levels = devm_kzalloc(dev, num_levels, GFP_KERNEL);
	if (!cdev->cooling_levels)
		return -ENOMEM;

	cdev->max_state = num_levels - 1;
	ret = of_property_read_u8_array(child, "cooling-levels",
					cdev->cooling_levels,
					num_levels);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read.\n");
		return ret;
	}
	snprintf(cdev->name, MAX_CDEV_NAME_LEN, "%s%d", child->name, pwm_channel);

	cdev->tcdev = thermal_of_cooling_device_register(child,
							 cdev->name,
							 cdev,
							 &aspeed_pwm_cool_ops);
	if (IS_ERR(cdev->tcdev))
		return PTR_ERR(cdev->tcdev);

	cdev->priv = priv;
	cdev->pwm_channel = pwm_channel;

	priv->cdev[pwm_channel] = cdev;

	return 0;
}

#define DEFAULT_TARGET_PWM_FREQ		25000
#define DEFAULT_MIN_RPM				2900

static int aspeed_pwm_create_fan(struct device *dev,
			     struct device_node *child,
			     struct aspeed_pwm_tachometer_data *priv)
{
	u8 *fan_tach_ch;
	u32 fan_min_rpm = 0;	
	u32 pwm_channel;
	u32 target_pwm_freq = 0;
	u32 duty_pt;
	int ret, count;

	ret = of_property_read_u32(child, "reg", &pwm_channel);
	if (ret)
		return ret;

	ret = of_property_read_u32(child, "aspeed,target_pwm", &target_pwm_freq);
	if (ret)
		target_pwm_freq = DEFAULT_TARGET_PWM_FREQ;

	ret = of_property_read_u32(child, "aspeed,default-duty-point",
                                   &duty_pt);
	if (!ret)
		priv->pwm_channel[pwm_channel].falling = duty_pt;

	aspeed_create_pwm_channel(priv, (u8)pwm_channel);

	count = of_property_count_u8_elems(child, "aspeed,fan-tach-ch");
	if (count < 1)
		return -EINVAL;

	fan_tach_ch = devm_kzalloc(dev, sizeof(*fan_tach_ch) * count,
				   GFP_KERNEL);
	if (!fan_tach_ch)
		return -ENOMEM;
	ret = of_property_read_u8_array(child, "aspeed,fan-tach-ch",
					fan_tach_ch, count);
	if (ret)
		return ret;
	
	ret = of_property_read_u32(child, "aspeed,min_rpm", &fan_min_rpm);
	if (ret)
		fan_min_rpm = DEFAULT_MIN_RPM;
	// Prevent to devided by zero
	if (fan_min_rpm == 0) {
		fan_min_rpm = 1;
	}
	aspeed_create_fan_tach_channel(priv, fan_tach_ch, count, fan_min_rpm);

	return 0;
}

static int aspeed_pwm_tachometer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *child;
	struct aspeed_pwm_tachometer_data *priv;
	void __iomem *regs;
	struct resource *res;
	struct device *hwmon;
	struct clk *clk;
	int ret;

	np = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pwm_channel = default_pwm_params;	
	priv->techo_channel = default_techo_params;
	priv->regmap = devm_regmap_init(dev, NULL, (__force void *)regs,
			&aspeed_pwm_tachometer_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;
	priv->clk_freq = clk_get_rate(clk);

	priv->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		dev_err(&pdev->dev, "can't get aspeed_pwm_tacho reset\n");
		return PTR_ERR(priv->reset);
	}

	//scu init
	reset_control_assert(priv->reset);
	reset_control_deassert(priv->reset);

	for_each_child_of_node(np, child) {
		ret = aspeed_pwm_create_fan(dev, child, priv);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	priv->groups[0] = &pwm_dev_group;
	priv->groups[1] = &fan_dev_group;
	priv->groups[2] = NULL;
	hwmon = devm_hwmon_device_register_with_groups(dev,
						       "aspeed_pwm_tachometer",
						       priv, priv->groups);

	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id of_pwm_tachometer_match_table[] = {
	{ .compatible = "aspeed,ast2600-pwm-tachometer", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_tachometer_match_table);

static struct platform_driver aspeed_pwm_tachometer_driver = {
	.probe		= aspeed_pwm_tachometer_probe,
	.driver		= {
		.name	= "aspeed_pwm_tachometer",
		.of_match_table = of_pwm_tachometer_match_table,
	},
};

module_platform_driver(aspeed_pwm_tachometer_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM and Fan Tachometer device driver");
MODULE_LICENSE("GPL");
