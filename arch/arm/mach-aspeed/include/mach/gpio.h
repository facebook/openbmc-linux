/*
 * arch/arm/mach-aspeed/include/mach/gpio.h
 *
 * Support functions for ASPEED GPIO
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 * Written by Ryan Chen <ryan_chen@aspeedtech.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_ASPEED_GPIO_H
#define __ASM_ARCH_ASPEED_GPIO_H

#include <linux/kernel.h>
#ifdef CONFIG_COLDFIRE
#include "irqs.h"
#include "ast_gpio_irqs.h"
#else
#include <mach/irqs.h>
#include <plat/aspeed.h>
#endif
/*************************************************************/
#define GPIO_PORTA							0x0
#define GPIO_PORTB							0x1
#define GPIO_PORTC							0x2
#define GPIO_PORTD							0x3
#define GPIO_PORTE							0x4
#define GPIO_PORTF							0x5
#define GPIO_PORTG							0x6
#define GPIO_PORTH							0x7
#define GPIO_PORTI							0x8
#define GPIO_PORTJ							0x9
#define GPIO_PORTK							0xa
#define GPIO_PORTL							0xb
#define GPIO_PORTM							0xc
#define GPIO_PORTN							0xd
#define GPIO_PORTO							0xe
#define GPIO_PORTP							0xf
#define GPIO_PORTQ							0x10
#define GPIO_PORTR							0x11
#define GPIO_PORTS							0x12
//AST2300 didn't have PORT TT
#define GPIO_PORTT							0x13
#if defined(AST_SOC_G4) || defined(CONFIG_AST2400_BMC)
#define GPIO_PORTU							0x14
#define GPIO_PORTV							0x15
#define GPIO_PORTW							0x16
#define GPIO_PORTX							0x17
#define GPIO_PORTY							0x18
#define GPIO_PORTZ							0x19
#define GPIO_PORTAA							0x1a
#define GPIO_PORTAB							0x1b
#endif
#if defined(AST_SOC_G5) 
#define GPIO_PORTU							0x14
#define GPIO_PORTV							0x15
#define GPIO_PORTW							0x16
#define GPIO_PORTX							0x17
#define GPIO_PORTY							0x18
#define GPIO_PORTZ							0x19
#define GPIO_PORTAA							0x1a
#define GPIO_PORTAB							0x1b
#define GPIO_PORTAC							0x1c
#endif

#define SGPIO_PORTA							0x0
#define SGPIO_PORTB							0x1
#define SGPIO_PORTC							0x2
#define SGPIO_PORTD							0x3
#define SGPIO_PORTE							0x4
#define SGPIO_PORTF							0x5
#define SGPIO_PORTG							0x6
#define SGPIO_PORTH							0x7
#define SGPIO_PORTI							0x8
#define SGPIO_PORTJ							0x9

#define GPIO_PER_PORT_PIN_NUM				8

#define GPIO_INPUT_MODE						0
#define GPIO_OUTPUT_MODE					1

#define GPIO_RISING_EDGE					1
#define GPIO_FALLING_EDGE					0

#define GPIO_LEVEL_HIGH						1
#define GPIO_LEVEL_LOW						1

#define GPIO_EDGE_MODE						0
#define GPIO_LEVEL_MODE						1

#define GPIO_EDGE_LEVEL_MODE				0
#define GPIO_DUAL_EDGE_MODE					1

#define GPIO_NO_DEBOUNCE					0
#define GPIO_DEBOUNCE_TIMER0				2		//GPIO 50 as debounce timer
#define GPIO_DEBOUNCE_TIMER1				1		//GPIO 54 as debounce timer
#define GPIO_DEBOUNCE_TIMER2				3		//GPIO 58 as debounce timer

#define GPIO_CMD_ARM						0
#define GPIO_CMD_LPC						1		
#define GPIO_CMD_COPROCESSOR				2

#define PIN_GPIOA0						(0)
#define PIN_GPIOA1						(1)
#define PIN_GPIOA2						(2)
#define PIN_GPIOA3						(3)
#define PIN_GPIOA4						(4)
#define PIN_GPIOA5						(5)
#define PIN_GPIOA6						(6)
#define PIN_GPIOA7						(7)
#define PIN_GPIOB0						(8)
#define PIN_GPIOB1						(9)
#define PIN_GPIOB2						(10)
#define PIN_GPIOB3						(11)
#define PIN_GPIOB4						(12)
#define PIN_GPIOB5						(13)
#define PIN_GPIOB6						(14)
#define PIN_GPIOB7						(15)
#define PIN_GPIOC0						(16)
#define PIN_GPIOC1						(17)
#define PIN_GPIOC2						(18)
#define PIN_GPIOC3						(19)
#define PIN_GPIOC4						(20)
#define PIN_GPIOC5						(21)
#define PIN_GPIOC6						(22)
#define PIN_GPIOC7						(23)
#define PIN_GPIOD0						(24)
#define PIN_GPIOD1						(25)
#define PIN_GPIOD2						(26)
#define PIN_GPIOD3						(27)
#define PIN_GPIOD4						(28)
#define PIN_GPIOD5						(29)
#define PIN_GPIOD6						(30)
#define PIN_GPIOD7						(31)
#define PIN_GPIOE0						(32)
#define PIN_GPIOE1						(33)
#define PIN_GPIOE2						(34)
#define PIN_GPIOE3						(35)
#define PIN_GPIOE4						(36)
#define PIN_GPIOE5						(37)
#define PIN_GPIOE6						(38)
#define PIN_GPIOE7						(39)
#define PIN_GPIOF0						(40)
#define PIN_GPIOF1						(41)
#define PIN_GPIOF2						(42)
#define PIN_GPIOF3						(43)
#define PIN_GPIOF4						(44)
#define PIN_GPIOF5						(45)
#define PIN_GPIOF6						(46)
#define PIN_GPIOF7						(47)
#define PIN_GPIOG0						(48)
#define PIN_GPIOG1						(49)
#define PIN_GPIOG2						(50)
#define PIN_GPIOG3						(51)
#define PIN_GPIOG4						(52)
#define PIN_GPIOG5						(53)
#define PIN_GPIOG6						(54)
#define PIN_GPIOG7						(55)
#define PIN_GPIOH0						(56)
#define PIN_GPIOH1						(57)
#define PIN_GPIOH2						(58)
#define PIN_GPIOH3						(59)
#define PIN_GPIOH4						(60)
#define PIN_GPIOH5						(61)
#define PIN_GPIOH6						(62)
#define PIN_GPIOH7						(63)
#define PIN_GPIOI0						(64)
#define PIN_GPIOI1						(65)
#define PIN_GPIOI2						(66)
#define PIN_GPIOI3						(67)
#define PIN_GPIOI4						(68)
#define PIN_GPIOI5						(69)
#define PIN_GPIOI6						(70)
#define PIN_GPIOI7						(71)
#define PIN_GPIOJ0						(72)
#define PIN_GPIOJ1						(73)
#define PIN_GPIOJ2						(74)
#define PIN_GPIOJ3						(75)
#define PIN_GPIOJ4						(76)
#define PIN_GPIOJ5						(77)
#define PIN_GPIOJ6						(78)
#define PIN_GPIOJ7						(79)
#define PIN_GPIOK0						(80)
#define PIN_GPIOK1						(81)
#define PIN_GPIOK2						(82)
#define PIN_GPIOK3						(83)
#define PIN_GPIOK4						(84)
#define PIN_GPIOK5						(85)
#define PIN_GPIOK6						(86)
#define PIN_GPIOK7						(87)
#define PIN_GPIOL0						(88)
#define PIN_GPIOL1						(89)
#define PIN_GPIOL2						(90)
#define PIN_GPIOL3						(91)
#define PIN_GPIOL4						(92)
#define PIN_GPIOL5						(93)
#define PIN_GPIOL6						(94)
#define PIN_GPIOL7						(95)
#define PIN_GPIOM0						(96)
#define PIN_GPIOM1						(97)
#define PIN_GPIOM2						(98)
#define PIN_GPIOM3						(99)
#define PIN_GPIOM4						(100)
#define PIN_GPIOM5						(101)
#define PIN_GPIOM6						(102)
#define PIN_GPIOM7						(103)
#define PIN_GPION0						(104)
#define PIN_GPION1						(105)
#define PIN_GPION2						(106)
#define PIN_GPION3						(107)
#define PIN_GPION4						(108)
#define PIN_GPION5						(109)
#define PIN_GPION6						(110)
#define PIN_GPION7						(111)
#define PIN_GPIOO0						(112)
#define PIN_GPIOO1						(113)
#define PIN_GPIOO2						(114)
#define PIN_GPIOO3						(115)
#define PIN_GPIOO4						(116)
#define PIN_GPIOO5						(117)
#define PIN_GPIOO6						(118)
#define PIN_GPIOO7						(119)
#define PIN_GPIOP0						(120)
#define PIN_GPIOP1						(121)
#define PIN_GPIOP2						(122)
#define PIN_GPIOP3						(123)
#define PIN_GPIOP4						(124)
#define PIN_GPIOP5						(125)
#define PIN_GPIOP6						(126)
#define PIN_GPIOP7						(127)
#define PIN_GPIOQ0						(128)
#define PIN_GPIOQ1						(129)
#define PIN_GPIOQ2						(130)
#define PIN_GPIOQ3						(131)
#define PIN_GPIOQ4						(132)
#define PIN_GPIOQ5						(133)
#define PIN_GPIOQ6						(134)
#define PIN_GPIOQ7						(135)
#define PIN_GPIOR0						(136)
#define PIN_GPIOR1						(137)
#define PIN_GPIOR2						(138)
#define PIN_GPIOR3						(139)
#define PIN_GPIOR4						(140)
#define PIN_GPIOR5						(141)
#define PIN_GPIOR6						(142)
#define PIN_GPIOR7						(143)
#define PIN_GPIOS0						(144)
#define PIN_GPIOS1						(145)
#define PIN_GPIOS2						(146)
#define PIN_GPIOS3						(147)
#define PIN_GPIOS4						(148)
#define PIN_GPIOS5						(149)
#define PIN_GPIOS6						(150)
#define PIN_GPIOS7						(151)

#if defined(AST_SOC_G4) || defined(CONFIG_AST2400_BMC)
#define PIN_GPIOT0						(152)
#define PIN_GPIOT1						(153)
#define PIN_GPIOT2						(154)
#define PIN_GPIOT3						(155)
#define PIN_GPIOT4						(156)
#define PIN_GPIOT5						(157)
#define PIN_GPIOT6						(158)
#define PIN_GPIOT7						(159)
#define PIN_GPIOU0						(161)
#define PIN_GPIOU1						(162)
#define PIN_GPIOU2						(163)
#define PIN_GPIOU3						(164)
#define PIN_GPIOU4						(165)
#define PIN_GPIOU5						(166)
#define PIN_GPIOU6						(167)
#define PIN_GPIOU7						(168)
#define PIN_GPIOV0						(169)
#define PIN_GPIOV1						(170)
#define PIN_GPIOV2						(171)
#define PIN_GPIOV3						(172)
#define PIN_GPIOV4						(173)
#define PIN_GPIOV5						(174)
#define PIN_GPIOV6						(175)
#define PIN_GPIOV7						(176)
#define PIN_GPIOW0						(177)
#define PIN_GPIOW1						(178)
#define PIN_GPIOW2						(179)
#define PIN_GPIOW3						(181)
#define PIN_GPIOW4						(182)
#define PIN_GPIOW5						(183)
#define PIN_GPIOW6						(184)
#define PIN_GPIOW7						(185)
#define PIN_GPIOX0						(186)
#define PIN_GPIOX1						(187)
#define PIN_GPIOX2						(188)
#define PIN_GPIOX3						(189)
#define PIN_GPIOX4						(190)
#define PIN_GPIOX5						(191)
#define PIN_GPIOX6						(192)
#define PIN_GPIOX7						(193)
#define PIN_GPIOY0						(194)
#define PIN_GPIOY1						(195)
#define PIN_GPIOY2						(196)
#define PIN_GPIOY3						(197)
#define PIN_GPIOY4						(198)
#define PIN_GPIOY5						(199)
#define PIN_GPIOY6						(200)
#define PIN_GPIOY7						(201)
#define PIN_GPIOZ0						(202)
#define PIN_GPIOZ1						(203)
#define PIN_GPIOZ2						(204)
#define PIN_GPIOZ3						(205)
#define PIN_GPIOZ4						(206)
#define PIN_GPIOZ5						(207)
#define PIN_GPIOZ6						(208)
#define PIN_GPIOZ7						(209)
#define PIN_GPIOAA0						(210)
#define PIN_GPIOAA1						(211)
#define PIN_GPIOAA2						(212)
#define PIN_GPIOAA3						(213)
#define PIN_GPIOAA4						(214)
#define PIN_GPIOAA5						(215)
#define PIN_GPIOAA6						(216)
#define PIN_GPIOAA7						(217)
#define PIN_GPIOAB0						(218)
#define PIN_GPIOAB1						(219)
#define PIN_GPIOAB2						(220)
#define PIN_GPIOAB3						(221)
#define PIN_GPIOAB4						(222)
#define PIN_GPIOAB5						(223)
#define PIN_GPIOAB6						(224)
#define PIN_GPIOAB7						(225)
#define PIN_SGPIO_START					(226)
#elif defined(AST_SOC_G5) 
#define PIN_GPIOT0						(152)
#define PIN_GPIOT1						(153)
#define PIN_GPIOT2						(154)
#define PIN_GPIOT3						(155)
#define PIN_GPIOT4						(156)
#define PIN_GPIOT5						(157)
#define PIN_GPIOT6						(158)
#define PIN_GPIOT7						(159)
#define PIN_GPIOU0						(161)
#define PIN_GPIOU1						(162)
#define PIN_GPIOU2						(163)
#define PIN_GPIOU3						(164)
#define PIN_GPIOU4						(165)
#define PIN_GPIOU5						(166)
#define PIN_GPIOU6						(167)
#define PIN_GPIOU7						(168)
#define PIN_GPIOV0						(169)
#define PIN_GPIOV1						(170)
#define PIN_GPIOV2						(171)
#define PIN_GPIOV3						(172)
#define PIN_GPIOV4						(173)
#define PIN_GPIOV5						(174)
#define PIN_GPIOV6						(175)
#define PIN_GPIOV7						(176)
#define PIN_GPIOW0						(177)
#define PIN_GPIOW1						(178)
#define PIN_GPIOW2						(179)
#define PIN_GPIOW3						(181)
#define PIN_GPIOW4						(182)
#define PIN_GPIOW5						(183)
#define PIN_GPIOW6						(184)
#define PIN_GPIOW7						(185)
#define PIN_GPIOX0						(186)
#define PIN_GPIOX1						(187)
#define PIN_GPIOX2						(188)
#define PIN_GPIOX3						(189)
#define PIN_GPIOX4						(190)
#define PIN_GPIOX5						(191)
#define PIN_GPIOX6						(192)
#define PIN_GPIOX7						(193)
#define PIN_GPIOY0						(194)
#define PIN_GPIOY1						(195)
#define PIN_GPIOY2						(196)
#define PIN_GPIOY3						(197)
#define PIN_GPIOY4						(198)
#define PIN_GPIOY5						(199)
#define PIN_GPIOY6						(200)
#define PIN_GPIOY7						(201)
#define PIN_GPIOZ0						(202)
#define PIN_GPIOZ1						(203)
#define PIN_GPIOZ2						(204)
#define PIN_GPIOZ3						(205)
#define PIN_GPIOZ4						(206)
#define PIN_GPIOZ5						(207)
#define PIN_GPIOZ6						(208)
#define PIN_GPIOZ7						(209)
#define PIN_GPIOAA0						(210)
#define PIN_GPIOAA1						(211)
#define PIN_GPIOAA2						(212)
#define PIN_GPIOAA3						(213)
#define PIN_GPIOAA4						(214)
#define PIN_GPIOAA5						(215)
#define PIN_GPIOAA6						(216)
#define PIN_GPIOAA7						(217)
#define PIN_GPIOAB0						(218)
#define PIN_GPIOAB1						(219)
#define PIN_GPIOAB2						(220)
#define PIN_GPIOAB3						(221)
#define PIN_GPIOAB4						(222)
#define PIN_GPIOAB5						(223)
#define PIN_GPIOAB6						(224)
#define PIN_GPIOAB7						(225)
#define PIN_GPIOAC0						(226)
#define PIN_GPIOAC1						(227)
#define PIN_GPIOAC2						(228)
#define PIN_GPIOAC3						(229)
#define PIN_GPIOAC4						(230)
#define PIN_GPIOAC5						(231)
#define PIN_GPIOAC6						(232)
#define PIN_GPIOAC7						(233)

#define PIN_SGPIO_START					(234)

#else
#define PIN_SGPIO_START					(152)
#endif

#define PIN_SGPIOA0						(PIN_SGPIO_START + 0)
#define PIN_SGPIOA1						(PIN_SGPIO_START + 1)
#define PIN_SGPIOA2						(PIN_SGPIO_START + 2)
#define PIN_SGPIOA3						(PIN_SGPIO_START + 3)
#define PIN_SGPIOA4						(PIN_SGPIO_START + 4)
#define PIN_SGPIOA5						(PIN_SGPIO_START + 5)
#define PIN_SGPIOA6						(PIN_SGPIO_START + 6)
#define PIN_SGPIOA7						(PIN_SGPIO_START + 7)
#define PIN_SGPIOB0						(PIN_SGPIO_START + 8)
#define PIN_SGPIOB1						(PIN_SGPIO_START + 9)
#define PIN_SGPIOB2						(PIN_SGPIO_START + 10)
#define PIN_SGPIOB3						(PIN_SGPIO_START + 11)
#define PIN_SGPIOB4						(PIN_SGPIO_START + 12)
#define PIN_SGPIOB5						(PIN_SGPIO_START + 13)
#define PIN_SGPIOB6						(PIN_SGPIO_START + 14)
#define PIN_SGPIOB7						(PIN_SGPIO_START + 15)
#define PIN_SGPIOC0						(PIN_SGPIO_START + 16)
#define PIN_SGPIOC1						(PIN_SGPIO_START + 17)
#define PIN_SGPIOC2						(PIN_SGPIO_START + 18)
#define PIN_SGPIOC3						(PIN_SGPIO_START + 19)
#define PIN_SGPIOC4						(PIN_SGPIO_START + 20)
#define PIN_SGPIOC5						(PIN_SGPIO_START + 21)
#define PIN_SGPIOC6						(PIN_SGPIO_START + 22)
#define PIN_SGPIOC7						(PIN_SGPIO_START + 23)
#define PIN_SGPIOD0						(PIN_SGPIO_START + 24)
#define PIN_SGPIOD1						(PIN_SGPIO_START + 25)
#define PIN_SGPIOD2						(PIN_SGPIO_START + 26)
#define PIN_SGPIOD3						(PIN_SGPIO_START + 27)
#define PIN_SGPIOD4						(PIN_SGPIO_START + 28)
#define PIN_SGPIOD5						(PIN_SGPIO_START + 29)
#define PIN_SGPIOD6						(PIN_SGPIO_START + 30)
#define PIN_SGPIOD7						(PIN_SGPIO_START + 31)
#define PIN_SGPIOE0						(PIN_SGPIO_START + 32)
#define PIN_SGPIOE1						(PIN_SGPIO_START + 33)
#define PIN_SGPIOE2						(PIN_SGPIO_START + 34)
#define PIN_SGPIOE3						(PIN_SGPIO_START + 35)
#define PIN_SGPIOE4						(PIN_SGPIO_START + 36)
#define PIN_SGPIOE5						(PIN_SGPIO_START + 37)
#define PIN_SGPIOE6						(PIN_SGPIO_START + 38)
#define PIN_SGPIOE7						(PIN_SGPIO_START + 39)
#define PIN_SGPIOF0						(PIN_SGPIO_START + 40)
#define PIN_SGPIOF1						(PIN_SGPIO_START + 41)
#define PIN_SGPIOF2						(PIN_SGPIO_START + 42)
#define PIN_SGPIOF3						(PIN_SGPIO_START + 43)
#define PIN_SGPIOF4						(PIN_SGPIO_START + 44)
#define PIN_SGPIOF5						(PIN_SGPIO_START + 45)
#define PIN_SGPIOF6						(PIN_SGPIO_START + 46)
#define PIN_SGPIOF7						(PIN_SGPIO_START + 47)
#define PIN_SGPIOG0						(PIN_SGPIO_START + 48)
#define PIN_SGPIOG1						(PIN_SGPIO_START + 49)
#define PIN_SGPIOG2						(PIN_SGPIO_START + 50)
#define PIN_SGPIOG3						(PIN_SGPIO_START + 51)
#define PIN_SGPIOG4						(PIN_SGPIO_START + 52)
#define PIN_SGPIOG5						(PIN_SGPIO_START + 53)
#define PIN_SGPIOG6						(PIN_SGPIO_START + 54)
#define PIN_SGPIOG7						(PIN_SGPIO_START + 55)
#define PIN_SGPIOH0						(PIN_SGPIO_START + 56)
#define PIN_SGPIOH1						(PIN_SGPIO_START + 57)
#define PIN_SGPIOH2						(PIN_SGPIO_START + 58)
#define PIN_SGPIOH3						(PIN_SGPIO_START + 59)
#define PIN_SGPIOH4						(PIN_SGPIO_START + 60)
#define PIN_SGPIOH5						(PIN_SGPIO_START + 61)
#define PIN_SGPIOH6						(PIN_SGPIO_START + 62)
#define PIN_SGPIOH7						(PIN_SGPIO_START + 63)
#define PIN_SGPIOI0						(PIN_SGPIO_START + 64)
#define PIN_SGPIOI1						(PIN_SGPIO_START + 65)
#define PIN_SGPIOI2						(PIN_SGPIO_START + 66)
#define PIN_SGPIOI3						(PIN_SGPIO_START + 67)
#define PIN_SGPIOI4						(PIN_SGPIO_START + 68)
#define PIN_SGPIOI5						(PIN_SGPIO_START + 69)
#define PIN_SGPIOI6						(PIN_SGPIO_START + 70)
#define PIN_SGPIOI7						(PIN_SGPIO_START + 71)
#define PIN_SGPIOJ0						(PIN_SGPIO_START + 72)
#define PIN_SGPIOJ1						(PIN_SGPIO_START + 73)
#define PIN_SGPIOJ2						(PIN_SGPIO_START + 74)
#define PIN_SGPIOJ3						(PIN_SGPIO_START + 75)
#define PIN_SGPIOJ4						(PIN_SGPIO_START + 76)
#define PIN_SGPIOJ5						(PIN_SGPIO_START + 77)
#define PIN_SGPIOJ6						(PIN_SGPIO_START + 78)
#define PIN_SGPIOJ7						(PIN_SGPIO_START + 79)

/*************************************************************/
#ifndef __ASSEMBLY__

/* callable at any time */
extern int ast_set_gpio_value(unsigned gpio_pin, int value);
extern int ast_get_gpio_value(unsigned gpio_pin);

/*-------------------------------------------------------------------------*/

/* wrappers for "new style" GPIO calls. the old AT91-specfic ones should
 * eventually be removed (along with this errno.h inclusion), and the
 * gpio request/free calls should probably be implemented.
 */

//extern int gpio_direction_input(unsigned gpio);
//extern int gpio_direction_output(unsigned gpio, int value);

static inline int gpio_get_value(unsigned gpio)
{
	return ast_get_gpio_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	ast_set_gpio_value(gpio, value);
}

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#define gpio_to_irq(gpio)	(IRQ_GPIO_CHAIN_START + (gpio))
#define irq_to_gpio(irq)	((irq) - IRQ_GPIO_CHAIN_START)

#endif	/* __ASSEMBLY__ */

void ast_set_gpio_debounce(int gpio_port, int timer);
void ast_set_gpio_debounce_timer(int timer, int val);

#endif
