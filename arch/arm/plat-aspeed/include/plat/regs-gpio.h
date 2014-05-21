/* arch/arm/plat-aspeed/include/mach/regs-gpio.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED I2C Controller
*/

#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H __FILE__

/*AST GPIO Register Definition */
#define AST_GPIO_DATA						0x000
#define AST_GPIO_DIR						0x004
#define AST_GPIO_INT_EN						0x008
#define AST_GPIO_INT_SEN_T0					0x00c
#define AST_GPIO_INT_SEN_T1					0x010
#define AST_GPIO_INT_SEN_T2					0x014
#define AST_GPIO_INT_STS					0x018
#define AST_GPIO_RST_TOR					0x01c
#define AST_EXT_GPIO_DATA					0x020
#define AST_EXT_GPIO_DIR					0x024
#define AST_EXT_GPIO_INT_EN					0x028
#define AST_EXT_GPIO_INT_SEN_T0				0x02c
#define AST_EXT_GPIO_INT_SEN_T1				0x030
#define AST_EXT_GPIO_INT_SEN_T2				0x034
#define AST_EXT_GPIO_INT_STS				0x038
#define AST_EXT_GPIO_RST_TOR				0x03c
#define AST_GPIO_DEBOUNCE_SET1				0x040			//A/B/C/D
#define AST_GPIO_DEBOUNCE_SET2				0x044			//A/B/C/D
#define AST_EXT_GPIO_DEBOUNCE_SET1			0x048			//E/F/G/H
#define AST_EXT_GPIO_DEBOUNCE_SET2			0x04C			//E/F/G/H
#define AST_DEBOUNCE_TIME_SET1				0x050
#define AST_DEBOUNCE_TIME_SET2				0x054
#define AST_DEBOUNCE_TIME_SET3				0x058
#define AST_GPIO_CMD_S0						0x060
#define AST_GPIO_CMD_S1						0x064
#define AST_EXT_GPIO_CMD_S0					0x068
#define AST_EXT_GPIO_CMD_S1					0x06C
#define AST_SIMPLE_GPIO_DATA0				0x070
#define AST_SIMPLE_GPIO_DIR0				0x074
#define AST_SIMPLE_GPIO_DATA1				0x078
#define AST_SIMPLE_GPIO_DIR1				0x07C
#define AST_SIMPLE_GPIO_DATA2				0x080
#define AST_SIMPLE_GPIO_DIR2				0x084
#define AST_SIMPLE_GPIO_DATA3				0x088
#define AST_SIMPLE_GPIO_DIR3				0x08C
#define AST_SIMPLE_GPIO0_CMD_S0				0x090
#define AST_SIMPLE_GPIO0_CMD_S1				0x094
#define AST_SIMPLE_GPIO0_INT_EN				0x098
#define AST_SIMPLE_GPIO0_INT_SEN_T0			0x09c
#define AST_SIMPLE_GPIO0_INT_SEN_T1			0x0a0
#define AST_SIMPLE_GPIO0_INT_SEN_T2			0x0a4
#define AST_SIMPLE_GPIO0_INT_STS			0x0a8
#define AST_SIMPLE_GPIO0_RST_TOR			0x0ac
#define AST_SIMPLE_GPIO0_DEBOUNCE_SET1		0x0b0
#define AST_SIMPLE_GPIO0_DEBOUNCE_SET2		0x0b4
#define AST_SIMPLE_GPIO0_INT_MASK			0x0b8
#define AST_GPIO_DATA_READ					0x0c0
#define AST_EXT_GPIO_DATA_READ				0x0c4
#define AST_SIMPLE_GPIO0_DATA_READ			0x0c8
#define AST_SIMPLE_GPIO1_DATA_READ			0x0cc
#define AST_SIMPLE_GPIO2_DATA_READ			0x0d0
#define AST_SIMPLE_GPIO3_DATA_READ			0x0d4
#define AST_SIMPLE_GPIO4_DATA_READ			0x0d8
#define AST_SIMPLE_GPIO1_CMD_S0				0x0e0
#define AST_SIMPLE_GPIO1_CMD_S1				0x0e4
#define AST_SIMPLE_GPIO1_INT_EN				0x0e8
#define AST_SIMPLE_GPIO1_INT_SEN_T0			0x0ec
#define AST_SIMPLE_GPIO1_INT_SEN_T1			0x0f0
#define AST_SIMPLE_GPIO1_INT_SEN_T2			0x0f4
#define AST_SIMPLE_GPIO1_INT_STS			0x0f8
#define AST_SIMPLE_GPIO1_RST_TOR			0x0fc
#define AST_SIMPLE_GPIO1_DEBOUNCE_SET1		0x100
#define AST_SIMPLE_GPIO1_DEBOUNCE_SET2		0x104
#define AST_SIMPLE_GPIO1_INT_MASK			0x108
#define AST_SIMPLE_GPIO2_CMD_S0				0x110
#define AST_SIMPLE_GPIO2_CMD_S1				0x114
#define AST_SIMPLE_GPIO2_INT_EN				0x118
#define AST_SIMPLE_GPIO2_INT_SEN_T0			0x11c
#define AST_SIMPLE_GPIO2_INT_SEN_T1			0x120
#define AST_SIMPLE_GPIO2_INT_SEN_T2			0x124
#define AST_SIMPLE_GPIO2_INT_STS			0x128
#define AST_SIMPLE_GPIO2_RST_TOR			0x12c
#define AST_SIMPLE_GPIO2_DEBOUNCE_SET1		0x130
#define AST_SIMPLE_GPIO2_DEBOUNCE_SET2		0x134
#define AST_SIMPLE_GPIO2_INT_MASK			0x138
#define AST_SIMPLE_GPIO3_CMD_S0				0x140
#define AST_SIMPLE_GPIO3_CMD_S1				0x144
#define AST_SIMPLE_GPIO3_INT_EN				0x148
#define AST_SIMPLE_GPIO3_INT_SEN_T0			0x14c
#define AST_SIMPLE_GPIO3_INT_SEN_T1			0x150
#define AST_SIMPLE_GPIO3_INT_SEN_T2			0x154
#define AST_SIMPLE_GPIO3_INT_STS			0x158
#define AST_SIMPLE_GPIO3_RST_TOR			0x15c
#define AST_SIMPLE_GPIO3_DEBOUNCE_SET1		0x160
#define AST_SIMPLE_GPIO3_DEBOUNCE_SET2		0x164
#define AST_SIMPLE_GPIO3_INT_MASK			0x168
#define AST_SIMPLE_GPIO4_CMD_S0				0x170
#define AST_SIMPLE_GPIO4_CMD_S1				0x174
#define AST_SIMPLE_GPIO4_INT_EN				0x178
#define AST_SIMPLE_GPIO4_INT_SEN_T0			0x17c
#define AST_SIMPLE_GPIO4_INT_SEN_T1			0x180
#define AST_SIMPLE_GPIO4_INT_SEN_T2			0x184
#define AST_SIMPLE_GPIO4_INT_STS			0x188
#define AST_SIMPLE_GPIO4_RST_TOR			0x18c
#define AST_SIMPLE_GPIO4_DEBOUNCE_SET1		0x190
#define AST_SIMPLE_GPIO4_DEBOUNCE_SET2		0x194
#define AST_SIMPLE_GPIO4_INT_MASK			0x198
#define AST_GPIO_INT_MASK					0x1d0
#define AST_EXT_GPIO_INT_MASK				0x1d4
#ifdef CONFIG_ARCH_AST1010
#else
#define AST_SIMPLE_GPIO_DATA4				0x1e0
#define AST_SIMPLE_GPIO_DIR4				0x1e4
#endif

//Serial GPIO
#define AST_SGPIO_DATA						0x200
#define AST_SGPIO_INT_EN					0x204
#define AST_SGPIO_INT_SEN_T0				0x208
#define AST_SGPIO_INT_SEN_T1				0x20c
#define AST_SGPIO_INT_SEN_T2				0x210
#define AST_SGPIO_INT_STS					0x214
#define AST_SGPIO_RST_TOR					0x218
#define AST_EXT_SGPIO_DATA					0x21c
#define AST_EXT_SGPIO_INT_EN				0x220
#define AST_EXT_SGPIO_INT_SEN_T0			0x224
#define AST_EXT_SGPIO_INT_SEN_T1			0x228
#define AST_EXT_SGPIO_INT_SEN_T2			0x22c
#define AST_EXT_SGPIO_INT_STS				0x230
#define AST_EXT_SGPIO_RST_TOR				0x234
#define AST_SGPIO_CTRL						0x254
#define AST_SGPIO_DATA_READ					0x270
#define AST_EXT_SGPIO_DAT					0x274

//Serial GPIO Slave Monitor
#define AST_SGPIO_SLAVE_DATA_INIT			0x300
#define AST_SGPIO_SLAVE_DATA_TARGET			0x304
#define AST_SGPIO_SLAVE_DATA_LOAD			0x308
#define AST_SGPIO_SLAVE_INT_EN0				0x30c
#define AST_SGPIO_SLAVE_INT_EN1				0x310
#define AST_SGPIO_SLAVE_INT_EN2				0x314
#define AST_SGPIO_SLAVE_INT_STS0			0x318
#define AST_SGPIO_SLAVE_INT_STS1			0x31c
#define AST_SGPIO_SLAVE_INT_STS2			0x320

/**********************************************************************************/
/* AST_GPIO_DATA - 0x000 : A/B/C/D Data Vale  */
#define GET_GPIOD_DATA(x)			((x&0xff000000) >> 24)
#define SET_GPIOD_DATA(x)			(x << 24)
#define GET_GPIOD_PIN_DATA(x,pin)	((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_DATA(pin)		(1<<(pin + 24))
#define GET_GPIOC_DATA(x)			((x&0xff0000) >> 16)
#define SET_GPIOC_DATA(x)			(x << 16)
#define GET_GPIOC_PIN_DATA(x,pin)	((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_DATA(pin)		(1<<(pin + 16))
#define GET_GPIOB_DATA(x)			((x&0xff00) >> 8)
#define SET_GPIOB_DATA(x)			(x << 8)
#define GET_GPIOB_PIN_DATA(x,pin)	((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_DATA(pin)		(1<<(pin + 8))
#define GET_GPIOA_DATA(x)			(x&0xff)
#define SET_GPIOA_DATA(x)			(x)
#define GET_GPIOA_PIN_DATA(x,pin)	((x >> pin) & 1)
#define SET_GPIOA_PIN_DATA(pin)		(1<<pin)

/*  AST_GPIO_DIR - 0x004 : Direction */
#define GET_GPIOD_DIR(x)			((x&0xff000000) >> 24)
#define SET_GPIOD_DIR(x)			(x << 24)
#define GET_GPIOD_PIN_DIR(x,pin)	((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_DIR(pin)		(1<<(pin + 24))
#define GET_GPIOC_DIR(x)			((x&0xff0000) >> 16)
#define SET_GPIOC_DIR(x)			(x << 16)
#define GET_GPIOC_PIN_DIR(x,pin)	((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_DIR(pin)		(1<<(pin + 16))
#define GET_GPIOB_DIR(x)			((x&0xff00) >> 8)
#define SET_GPIOB_DIR(x)			(x << 8)
#define GET_GPIOB_PIN_DIR(x,pin)	((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_DIR(pin)		(1<<(pin + 8))
#define GET_GPIOA_DIR(x)			(x&0xff)
#define SET_GPIOA_DIR(x)			(x)
#define GET_GPIOA_PIN_DIR(x,pin)	((x >> pin) & 1)
#define SET_GPIOA_PIN_DIR(pin)		(1<<pin)

/* AST_GPIO_INT_EN - 0x008 : Interrupt Enable */
#define GET_GPIOD_INT_EN(x)				((x&0xff000000) >> 24)
#define SET_GPIOD_INT_EN(x)				(x << 24)
#define GET_GPIOD_PIN_INT_EN(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_INT_EN(pin)		(1<<(pin + 24))
#define GET_GPIOC_INT_EN(x)				((x&0xff0000) >> 16)
#define SET_GPIOC_INT_EN(x)				(x << 16)
#define GET_GPIOC_PIN_INT_EN(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_INT_EN(pin)		(1<<(pin + 16))
#define GET_GPIOB_INT_EN(x)				((x&0xff00) >> 8)
#define SET_GPIOB_INT_EN(x)				(x << 8)
#define GET_GPIOB_PIN_INT_EN(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_INT_EN(pin)			(1<<(pin + 8))
#define GET_GPIOA_INT_EN(x)				(x&0xff)
#define SET_GPIOA_INT_EN(x)				(x)
#define GET_GPIOA_PIN_INT_EN(x,pin)		((x >> pin) & 1)
#define SET_GPIOA_PIN_INT_EN(pin)		(1<<pin)

/* AST_GPIO_INT_SEN_T0/1/2  - 0x00c/0x010/0x014 : Interrupt Sensitivity Type 0/1/2 */
#define GET_GPIOD_INT_MODE(x)			((x&0xff000000) >> 24)
#define SET_GPIOD_INT_MODE(x)			(x << 24)
#define GET_GPIOD_PIN_INT_MODE(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_INT_MODE(pin)		(1<<(pin + 24))
#define GET_GPIOC_INT_MODE(x)			((x&0xff0000) >> 16)
#define SET_GPIOC_INT_MODE(x)			(x << 16)
#define GET_GPIOC_PIN_INT_MODE(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_INT_MODE(pin)		(1<<(pin + 16))
#define GET_GPIOB_INT_MODE(x)			((x&0xff00) >> 8)
#define SET_GPIOB_INT_MODE(x)			(x << 16)
#define GET_GPIOB_PIN_INT_MODE(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_INT_MODE(pin)		(1<<(pin + 8))
#define GET_GPIOA_INT_MODE(x)			(x&0xff)
#define SET_GPIOA_INT_MODE(x)			(x)
#define GET_GPIOA_PIN_INT_MODE(x,pin)	((x >> pin) & 1)
#define SET_GPIOA_PIN_INT_MODE(pin)		(1 << pin)

/* AST_GPIO_INT_STS	- 0x018 : Interrupt Status */
#define GET_GPIOD_INT_STS(x)			((x&0xff000000) >> 24)
#define SET_GPIOD_INT_STS(x)			(x << 24)
#define GET_GPIOD_PIN_INT_STS(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_INT_STS(pin)		(1<<(pin + 24))
#define GET_GPIOC_INT_STS(x)			((x&0xff0000) >> 16)
#define SET_GPIOC_INT_STS(x)			(x << 16)
#define GET_GPIOC_PIN_INT_STS(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_INT_STS(pin)		(1<<(pin + 16))
#define GET_GPIOB_INT_STS(x)			((x&0xff00) >> 8)
#define SET_GPIOB_INT_STS(x)			(x << 16)
#define GET_GPIOB_PIN_INT_STS(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_INT_STS(pin)		(1<<(pin + 8))
#define GET_GPIOA_INT_STS(x)			(x&0xff)
#define SET_GPIOA_INT_STS(x)			(x)
#define GET_GPIOA_PIN_INT_STS(x,pin)	((x >> pin) & 1)
#define SET_GPIOA_PIN_INT_STS(pin)		(1 << pin)

/* AST_GPIO_RST_TOR - 0x01c : Reset Tolerant */
#define GET_GPIOD_RST_EN(x)				((x&0xff000000) >> 24)
#define SET_GPIOD_RST_EN(x)				(x << 24)
#define GET_GPIOD_PIN_RST_EN(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIOD_PIN_RST_EN(pin)		(1<<(pin + 24))
#define GET_GPIOC_RST_EN(x)				((x&0xff0000) >> 16)
#define SET_GPIOC_RST_EN(x)				(x << 16)
#define GET_GPIOC_PIN_RST_EN(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIOC_PIN_RST_EN(pin)		(1<<(pin + 16))
#define GET_GPIOB_RST_EN(x)				((x&0xff00) >> 8)
#define SET_GPIOB_RST_EN(x)				(x << 16)
#define GET_GPIOB_PIN_RST_EN(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIOB_PIN_RST_EN(pin)		(1<<(pin + 8))
#define GET_GPIOA_RST_EN(x)				(x&0xff)
#define SET_GPIOA_RST_EN(x)				(x)
#define GET_GPIOA_PIN_RST_EN(x,pin)		((x >> pin) & 1)
#define SET_GPIOA_PIN_RST_EN(pin)		(1 << pin)

/* AST_EXT_GPIO_DATA - 0x020 :  E/F/G/H Data Vale  */
#define GET_GPIOH_DATA(x)			((x&0xff000000) >> 24)
#define SET_GPIOH_DATA(x)			(x << 24)
#define GET_GPIOH_PIN_DATA(x,pin)	((x >> (pin + 24)) & 1)
#define SET_GPIOH_PIN_DATA(pin)		(1<<(pin + 24))
#define GET_GPIOG_DATA(x)			((x&0xff0000) >> 16)
#define SET_GPIOG_DATA(x)			(x << 16)
#define GET_GPIOG_PIN_DATA(x,pin)	((x >> (pin + 16)) & 1)
#define SET_GPIOG_PIN_DATA(pin)		(1<<(pin + 16))
#define GET_GPIOF_DATA(x)			((x&0xff00) >> 8)
#define SET_GPIOF_DATA(x)			(x << 8)
#define GET_GPIOF_PIN_DATA(x,pin)	((x >> (pin + 8)) & 1)
#define SET_GPIOF_PIN_DATA(pin)		(1<<(pin + 8))
#define GET_GPIOE_DATA(x)			(x&0xff)
#define SET_GPIOE_DATA(x)			(x)
#define GET_GPIOE_PIN_DATA(x,pin)	((x >> pin) & 1)
#define SET_GPIOE_PIN_DATA(pin)		(1<<pin)

/* AST_EXT_GPIO_DIR		 - 0x024 : */
#define GET_GPIOH_DIR(x)			((x&0xff000000) >> 24)
#define SET_GPIOH_DIR(x)			(x << 24)
#define GET_GPIOH_PIN_DIR(x,pin)	((x >> (pin + 24)) & 1)
#define SET_GPIOH_PIN_DIR(pin)		(1<<(pin + 24))
#define GET_GPIOG_DIR(x)			((x&0xff0000) >> 16)
#define SET_GPIOG_DIR(x)			(x << 16)
#define GET_GPIOG_PIN_DIR(x,pin)	((x >> (pin + 16)) & 1)
#define SET_GPIOG_PIN_DIR(pin)		(1<<(pin + 16))
#define GET_GPIOF_DIR(x)			((x&0xff00) >> 8)
#define SET_GPIOF_DIR(x)			(x << 8)
#define GET_GPIOF_PIN_DIR(x,pin)	((x >> (pin + 8)) & 1)
#define SET_GPIOF_PIN_DIR(pin)		(1<<(pin + 8))
#define GET_GPIOE_DIR(x)			(x&0xff)
#define SET_GPIOE_DIR(x)			(x)
#define GET_GPIOE_PIN_DIR(x,pin)	((x >> pin) & 1)
#define SET_GPIOE_PIN_DIR(pin)		(1<<pin)

/* AST_EXT_GPIO_INT_EN	 - 0x028 */
#define GET_GPIOH_INT_EN(x)				((x&0xff000000) >> 24)
#define SET_GPIOH_INT_EN(x)				(x << 24)
#define GET_GPIOH_PIN_INT_EN(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIOH_PIN_INT_EN(pin)		(1<<(pin + 24))
#define GET_GPIOG_INT_EN(x)				((x&0xff0000) >> 16)
#define SET_GPIOG_INT_EN(x)				(x << 16)
#define GET_GPIOG_PIN_INT_EN(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIOG_PIN_INT_EN(pin)		(1<<(pin + 16))
#define GET_GPIOF_INT_EN(x)				((x&0xff00) >> 8)
#define SET_GPIOF_INT_EN(x)				(x << 8)
#define GET_GPIOF_PIN_INT_EN(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIOF_PIN_INT_EN(pin)			(1<<(pin + 8))
#define GET_GPIOE_INT_EN(x)				(x&0xff)
#define SET_GPIOE_INT_EN(x)				(x)
#define GET_GPIOE_PIN_INT_EN(x,pin)		((x >> pin) & 1)
#define SET_GPIOE_PIN_INT_EN(pin)		(1<<pin)

/* AST_EXT_GPIO_INT_SEN_T0/1/2	- 0x02c/0x30/0x34 :  */
/* AST_EXT_GPIO_INT_STS					0x038 */
/* AST_EXT_GPIO_RST_TOR				0x03c */

/* AST_GPIO_DEBOUNCE_SET1 -	0x040 : Debounce Setting #1 */
#define GET_GPIO3_DEBOUNCE(x)				((x&0xff000000) >> 24)
#define SET_GPIO3_DEBOUNCE(x)				(x << 24)
#define GET_GPIO3_PIN_DEBOUNCE(x,pin)		((x >> (pin + 24)) & 1)
#define SET_GPIO3_PIN_DEBOUNCE(pin)		(1<<(pin + 24))
#define GET_GPIO2_DEBOUNCE(x)				((x&0xff0000) >> 16)
#define SET_GPIO2_DEBOUNCE(x)				(x << 16)
#define GET_GPIO2_PIN_DEBOUNCE(x,pin)		((x >> (pin + 16)) & 1)
#define SET_GPIO2_PIN_DEBOUNCE(pin)		(1<<(pin + 16))
#define GET_GPIO1_DEBOUNCE(x)				((x&0xff00) >> 8)
#define SET_GPIO1_DEBOUNCE(x)				(x << 8)
#define GET_GPIO1_PIN_DEBOUNCE(x,pin)		((x >> (pin + 8)) & 1)
#define SET_GPIO1_PIN_DEBOUNCE(pin)			(1<<(pin + 8))
#define GET_GPIO0_DEBOUNCE(x)				(x&0xff)
#define SET_GPIO0_DEBOUNCE(x)				(x)
#define GET_GPIO0_PIN_DEBOUNCE(x,pin)		((x >> pin) & 1)
#define SET_GPIO0_PIN_DEBOUNCE(pin)		(1<<pin)


#endif /* __ASM_ARCH_REGS_GPIO_H */