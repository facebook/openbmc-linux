/*
 *  arch/arm/plat-aspeed/include/plat/irqs.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AST1520_IRQS_H_
#define _AST1520_IRQS_H_                 1


//--------------GPIO ---------------------------------------------------------------
#define ARCH_NR_GPIOS 					(GPIO_PORT_NUM*8)
#define IRQ_GPIO_CHAIN_START			AST_VIC_NUM
//------------------- ---------------------------------------------------------------
#define NR_IRQS                         (AST_VIC_NUM + ARCH_NR_GPIOS)

#define GPIO_PORT_NUM		28

#define AST_VIC_NUM			51

//#define IRQ_SDRAM_ECC					0
//#define IRQ_MIC							1
#define IRQ_MAC0						2			/* MAC 1 interrupt */
//#define IRQ_MAC1						3			/* MAC 2 interrupt */
#define IRQ_CRYPTO						4
#define IRQ_USB20_HUB					5
#define IRQ_EHCI					5
#define IRQ_XDMA						6
#define IRQ_VIDEO						7
//#define IRQ_LPC							8
#define IRQ_UART1						9			/* UART 1 interrupt */
#define IRQ_UART0						10			/* UART 3 interrupt */
//11 Reserved
#define IRQ_I2C							12
//#define IRQ_UDC11					13
#define IRQ_UHCI					14
//#define IRQ_PECI						15
#define IRQ_TIMER0						16			/* TIMER 1 interrupt */
#define IRQ_TIMER1						17			/* TIMER 2 interrupt */
#define IRQ_TIMER2						18			/* TIMER 3 interrupt */
//#define IRQ_SMC							19
#define IRQ_GPIO						20
#define IRQ_SCU							21
#define IRQ_RTC							22
//23 , 24 reserverd 
#define IRQ_CRT							25
#define IRQ_SDHC						26
#define IRQ_WDT							27
#define IRQ_TACHO						28
#define IRQ_2D							29
#define IRQ_SYS_WAKEUP					30
//#define IRQ_ADC							31
#define IRQ_UART2						32			/* UART 2 interrupt */
//#define IRQ_UART2						33			/* UART 3 interrupt */
//#define IRQ_UART3						34			/* UART 4 interrupt */
//#define IRQ_TIMER3						35			/* TIMER 4 interrupt */
//#define IRQ_TIMER4						36
//#define IRQ_TIMER5						37
//#define IRQ_TIMER6						38
//#define IRQ_TIMER7						39			/* TIMER 8 interrupt */
//#define IRQ_SGPIO_MASTER				40
//#define IRQ_SGPIO_SLAVE					41
#define IRQ_MCTP						42
//#define IRQ_JTAG						43
#define IRQ_PS2							44
#define IRQ_CPU1						45
//#define IRQ_MAILBOX						46
#define IRQ_EXT0_GPIOL1					47
#define IRQ_EXT1_GPIOL3					48
#define IRQ_EXT2_GPIOL3					49
#define IRQ_EXT3_GPIOL3					50

#define IRQ_GPIOA0						(AST_VIC_NUM + 0)
#define IRQ_GPIOA1						(AST_VIC_NUM + 1)
#define IRQ_GPIOA2						(AST_VIC_NUM + 2)
#define IRQ_GPIOA3						(AST_VIC_NUM + 3)
#define IRQ_GPIOA4						(AST_VIC_NUM + 4)
#define IRQ_GPIOA5						(AST_VIC_NUM + 5)
#define IRQ_GPIOA6						(AST_VIC_NUM + 6)
#define IRQ_GPIOA7						(AST_VIC_NUM + 7)
#define IRQ_GPIOB0						(AST_VIC_NUM + 8)
#define IRQ_GPIOB1						(AST_VIC_NUM + 9)
#define IRQ_GPIOB2						(AST_VIC_NUM + 10)
#define IRQ_GPIOB3						(AST_VIC_NUM + 11)
#define IRQ_GPIOB4						(AST_VIC_NUM + 12)
#define IRQ_GPIOB5						(AST_VIC_NUM + 13)
#define IRQ_GPIOB6						(AST_VIC_NUM + 14)
#define IRQ_GPIOB7						(AST_VIC_NUM + 15)
#define IRQ_GPIOC0						(AST_VIC_NUM + 16)
#define IRQ_GPIOC1						(AST_VIC_NUM + 17)
#define IRQ_GPIOC2						(AST_VIC_NUM + 18)
#define IRQ_GPIOC3						(AST_VIC_NUM + 19)
#define IRQ_GPIOC4						(AST_VIC_NUM + 20)
#define IRQ_GPIOC5						(AST_VIC_NUM + 21)
#define IRQ_GPIOC6						(AST_VIC_NUM + 22)
#define IRQ_GPIOC7						(AST_VIC_NUM + 23)
#define IRQ_GPIOD0						(AST_VIC_NUM + 24)
#define IRQ_GPIOD1						(AST_VIC_NUM + 25)
#define IRQ_GPIOD2						(AST_VIC_NUM + 26)
#define IRQ_GPIOD3						(AST_VIC_NUM + 27)
#define IRQ_GPIOD4						(AST_VIC_NUM + 28)
#define IRQ_GPIOD5						(AST_VIC_NUM + 29)
#define IRQ_GPIOD6						(AST_VIC_NUM + 30)
#define IRQ_GPIOD7						(AST_VIC_NUM + 31)
#define IRQ_GPIOE0						(AST_VIC_NUM + 32)
#define IRQ_GPIOE1						(AST_VIC_NUM + 33)
#define IRQ_GPIOE2						(AST_VIC_NUM + 34)
#define IRQ_GPIOE3						(AST_VIC_NUM + 35)
#define IRQ_GPIOE4						(AST_VIC_NUM + 36)
#define IRQ_GPIOE5						(AST_VIC_NUM + 37)
#define IRQ_GPIOE6						(AST_VIC_NUM + 38)
#define IRQ_GPIOE7						(AST_VIC_NUM + 39)
#define IRQ_GPIOF0						(AST_VIC_NUM + 40)
#define IRQ_GPIOF1						(AST_VIC_NUM + 41)
#define IRQ_GPIOF2						(AST_VIC_NUM + 42)
#define IRQ_GPIOF3						(AST_VIC_NUM + 43)
#define IRQ_GPIOF4						(AST_VIC_NUM + 44)
#define IRQ_GPIOF5						(AST_VIC_NUM + 45)
#define IRQ_GPIOF6						(AST_VIC_NUM + 46)
#define IRQ_GPIOF7						(AST_VIC_NUM + 47)
#define IRQ_GPIOG0						(AST_VIC_NUM + 48)
#define IRQ_GPIOG1						(AST_VIC_NUM + 49)
#define IRQ_GPIOG2						(AST_VIC_NUM + 50)
#define IRQ_GPIOG3						(AST_VIC_NUM + 51)
#define IRQ_GPIOG4						(AST_VIC_NUM + 52)
#define IRQ_GPIOG5						(AST_VIC_NUM + 53)
#define IRQ_GPIOG6						(AST_VIC_NUM + 54)
#define IRQ_GPIOG7						(AST_VIC_NUM + 55)
#define IRQ_GPIOH0						(AST_VIC_NUM + 56)
#define IRQ_GPIOH1						(AST_VIC_NUM + 57)
#define IRQ_GPIOH2						(AST_VIC_NUM + 58)
#define IRQ_GPIOH3						(AST_VIC_NUM + 59)
#define IRQ_GPIOH4						(AST_VIC_NUM + 60)
#define IRQ_GPIOH5						(AST_VIC_NUM + 61)
#define IRQ_GPIOH6						(AST_VIC_NUM + 62)
#define IRQ_GPIOH7						(AST_VIC_NUM + 63)
#define IRQ_GPIOI0						(AST_VIC_NUM + 64)
#define IRQ_GPIOI1						(AST_VIC_NUM + 65)
#define IRQ_GPIOI2						(AST_VIC_NUM + 66)
#define IRQ_GPIOI3						(AST_VIC_NUM + 67)
#define IRQ_GPIOI4						(AST_VIC_NUM + 68)
#define IRQ_GPIOI5						(AST_VIC_NUM + 69)
#define IRQ_GPIOI6						(AST_VIC_NUM + 70)
#define IRQ_GPIOI7						(AST_VIC_NUM + 71)
#define IRQ_GPIOJ0						(AST_VIC_NUM + 72)
#define IRQ_GPIOJ1						(AST_VIC_NUM + 73)
#define IRQ_GPIOJ2						(AST_VIC_NUM + 74)
#define IRQ_GPIOJ3						(AST_VIC_NUM + 75)
#define IRQ_GPIOJ4						(AST_VIC_NUM + 76)
#define IRQ_GPIOJ5						(AST_VIC_NUM + 77)
#define IRQ_GPIOJ6						(AST_VIC_NUM + 78)
#define IRQ_GPIOJ7						(AST_VIC_NUM + 79)
#define IRQ_GPIOK0						(AST_VIC_NUM + 80)
#define IRQ_GPIOK1						(AST_VIC_NUM + 81)
#define IRQ_GPIOK2						(AST_VIC_NUM + 82)
#define IRQ_GPIOK3						(AST_VIC_NUM + 83)
#define IRQ_GPIOK4						(AST_VIC_NUM + 84)
#define IRQ_GPIOK5						(AST_VIC_NUM + 85)
#define IRQ_GPIOK6						(AST_VIC_NUM + 86)
#define IRQ_GPIOK7						(AST_VIC_NUM + 87)
#define IRQ_GPIOL0						(AST_VIC_NUM + 88)
#define IRQ_GPIOL1						(AST_VIC_NUM + 89)
#define IRQ_GPIOL2						(AST_VIC_NUM + 90)
#define IRQ_GPIOL3						(AST_VIC_NUM + 91)
#define IRQ_GPIOL4						(AST_VIC_NUM + 92)
#define IRQ_GPIOL5						(AST_VIC_NUM + 93)
#define IRQ_GPIOL6						(AST_VIC_NUM + 94)
#define IRQ_GPIOL7						(AST_VIC_NUM + 95)
#define IRQ_GPIOM0						(AST_VIC_NUM + 96)
#define IRQ_GPIOM1						(AST_VIC_NUM + 97)
#define IRQ_GPIOM2						(AST_VIC_NUM + 98)
#define IRQ_GPIOM3						(AST_VIC_NUM + 99)
#define IRQ_GPIOM4						(AST_VIC_NUM + 100)
#define IRQ_GPIOM5						(AST_VIC_NUM + 101)
#define IRQ_GPIOM6						(AST_VIC_NUM + 102)
#define IRQ_GPIOM7						(AST_VIC_NUM + 103)
#define IRQ_GPION0						(AST_VIC_NUM + 104)
#define IRQ_GPION1						(AST_VIC_NUM + 105)
#define IRQ_GPION2						(AST_VIC_NUM + 106)
#define IRQ_GPION3						(AST_VIC_NUM + 107)
#define IRQ_GPION4						(AST_VIC_NUM + 108)
#define IRQ_GPION5						(AST_VIC_NUM + 109)
#define IRQ_GPION6						(AST_VIC_NUM + 110)
#define IRQ_GPION7						(AST_VIC_NUM + 111)
#define IRQ_GPIOO0						(AST_VIC_NUM + 112)
#define IRQ_GPIOO1						(AST_VIC_NUM + 113)
#define IRQ_GPIOO2						(AST_VIC_NUM + 114)
#define IRQ_GPIOO3						(AST_VIC_NUM + 115)
#define IRQ_GPIOO4						(AST_VIC_NUM + 116)
#define IRQ_GPIOO5						(AST_VIC_NUM + 117)
#define IRQ_GPIOO6						(AST_VIC_NUM + 118)
#define IRQ_GPIOO7						(AST_VIC_NUM + 119)
#define IRQ_GPIOP0						(AST_VIC_NUM + 120)
#define IRQ_GPIOP1						(AST_VIC_NUM + 121)
#define IRQ_GPIOP2						(AST_VIC_NUM + 122)
#define IRQ_GPIOP3						(AST_VIC_NUM + 123)
#define IRQ_GPIOP4						(AST_VIC_NUM + 124)
#define IRQ_GPIOP5						(AST_VIC_NUM + 125)
#define IRQ_GPIOP6						(AST_VIC_NUM + 126)
#define IRQ_GPIOP7						(AST_VIC_NUM + 127)
#define IRQ_GPIOQ0						(AST_VIC_NUM + 128)
#define IRQ_GPIOQ1						(AST_VIC_NUM + 129)
#define IRQ_GPIOQ2						(AST_VIC_NUM + 130)
#define IRQ_GPIOQ3						(AST_VIC_NUM + 131)
#define IRQ_GPIOQ4						(AST_VIC_NUM + 132)
#define IRQ_GPIOQ5						(AST_VIC_NUM + 133)
#define IRQ_GPIOQ6						(AST_VIC_NUM + 134)
#define IRQ_GPIOQ7						(AST_VIC_NUM + 135)
#define IRQ_GPIOR0						(AST_VIC_NUM + 136)
#define IRQ_GPIOR1						(AST_VIC_NUM + 137)
#define IRQ_GPIOR2						(AST_VIC_NUM + 138)
#define IRQ_GPIOR3						(AST_VIC_NUM + 139)
#define IRQ_GPIOR4						(AST_VIC_NUM + 140)
#define IRQ_GPIOR5						(AST_VIC_NUM + 141)
#define IRQ_GPIOR6						(AST_VIC_NUM + 142)
#define IRQ_GPIOR7						(AST_VIC_NUM + 143)
#define IRQ_GPIOS0						(AST_VIC_NUM + 144)
#define IRQ_GPIOS1						(AST_VIC_NUM + 145)
#define IRQ_GPIOS2						(AST_VIC_NUM + 146)
#define IRQ_GPIOS3						(AST_VIC_NUM + 147)
#define IRQ_GPIOS4						(AST_VIC_NUM + 148)
#define IRQ_GPIOS5						(AST_VIC_NUM + 149)
#define IRQ_GPIOS6						(AST_VIC_NUM + 150)
#define IRQ_GPIOS7						(AST_VIC_NUM + 151)
#define IRQ_GPIOT0						(AST_VIC_NUM + 152)
#define IRQ_GPIOT1						(AST_VIC_NUM + 153)
#define IRQ_GPIOT2						(AST_VIC_NUM + 154)
#define IRQ_GPIOT3						(AST_VIC_NUM + 155)
#define IRQ_GPIOT4						(AST_VIC_NUM + 156)
#define IRQ_GPIOT5						(AST_VIC_NUM + 157)
#define IRQ_GPIOT6						(AST_VIC_NUM + 158)
#define IRQ_GPIOT7						(AST_VIC_NUM + 159)
#define IRQ_GPIOU0						(AST_VIC_NUM + 161)
#define IRQ_GPIOU1						(AST_VIC_NUM + 162)
#define IRQ_GPIOU2						(AST_VIC_NUM + 163)
#define IRQ_GPIOU3						(AST_VIC_NUM + 164)
#define IRQ_GPIOU4						(AST_VIC_NUM + 165)
#define IRQ_GPIOU5						(AST_VIC_NUM + 166)
#define IRQ_GPIOU6						(AST_VIC_NUM + 167)
#define IRQ_GPIOU7						(AST_VIC_NUM + 168)
#define IRQ_GPIOV0						(AST_VIC_NUM + 169)
#define IRQ_GPIOV1						(AST_VIC_NUM + 170)
#define IRQ_GPIOV2						(AST_VIC_NUM + 171)
#define IRQ_GPIOV3						(AST_VIC_NUM + 172)
#define IRQ_GPIOV4						(AST_VIC_NUM + 173)
#define IRQ_GPIOV5						(AST_VIC_NUM + 174)
#define IRQ_GPIOV6						(AST_VIC_NUM + 175)
#define IRQ_GPIOV7						(AST_VIC_NUM + 176)
#define IRQ_GPIOW0						(AST_VIC_NUM + 177)
#define IRQ_GPIOW1						(AST_VIC_NUM + 178)
#define IRQ_GPIOW2						(AST_VIC_NUM + 179)
#define IRQ_GPIOW3						(AST_VIC_NUM + 181)
#define IRQ_GPIOW4						(AST_VIC_NUM + 182)
#define IRQ_GPIOW5						(AST_VIC_NUM + 183)
#define IRQ_GPIOW6						(AST_VIC_NUM + 184)
#define IRQ_GPIOW7						(AST_VIC_NUM + 185)
#define IRQ_GPIOX0						(AST_VIC_NUM + 186)
#define IRQ_GPIOX1						(AST_VIC_NUM + 187)
#define IRQ_GPIOX2						(AST_VIC_NUM + 188)
#define IRQ_GPIOX3						(AST_VIC_NUM + 189)
#define IRQ_GPIOX4						(AST_VIC_NUM + 190)
#define IRQ_GPIOX5						(AST_VIC_NUM + 191)
#define IRQ_GPIOX6						(AST_VIC_NUM + 192)
#define IRQ_GPIOX7						(AST_VIC_NUM + 193)
#define IRQ_GPIOY0						(AST_VIC_NUM + 194)
#define IRQ_GPIOY1						(AST_VIC_NUM + 195)
#define IRQ_GPIOY2						(AST_VIC_NUM + 196)
#define IRQ_GPIOY3						(AST_VIC_NUM + 197)
#define IRQ_GPIOY4						(AST_VIC_NUM + 198)
#define IRQ_GPIOY5						(AST_VIC_NUM + 199)
#define IRQ_GPIOY6						(AST_VIC_NUM + 200)
#define IRQ_GPIOY7						(AST_VIC_NUM + 201)
#define IRQ_GPIOZ0						(AST_VIC_NUM + 202)
#define IRQ_GPIOZ1						(AST_VIC_NUM + 203)
#define IRQ_GPIOZ2						(AST_VIC_NUM + 204)
#define IRQ_GPIOZ3						(AST_VIC_NUM + 205)
#define IRQ_GPIOZ4						(AST_VIC_NUM + 206)
#define IRQ_GPIOZ5						(AST_VIC_NUM + 207)
#define IRQ_GPIOZ6						(AST_VIC_NUM + 208)
#define IRQ_GPIOZ7						(AST_VIC_NUM + 209)
#define IRQ_GPIOAA0						(AST_VIC_NUM + 210)
#define IRQ_GPIOAA1						(AST_VIC_NUM + 211)
#define IRQ_GPIOAA2						(AST_VIC_NUM + 212)
#define IRQ_GPIOAA3						(AST_VIC_NUM + 213)
#define IRQ_GPIOAA4						(AST_VIC_NUM + 214)
#define IRQ_GPIOAA5						(AST_VIC_NUM + 215)
#define IRQ_GPIOAA6						(AST_VIC_NUM + 216)
#define IRQ_GPIOAA7						(AST_VIC_NUM + 217)
#define IRQ_GPIOBB0						(AST_VIC_NUM + 218)
#define IRQ_GPIOBB1						(AST_VIC_NUM + 219)
#define IRQ_GPIOBB2						(AST_VIC_NUM + 220)
#define IRQ_GPIOBB3						(AST_VIC_NUM + 221)
#define IRQ_GPIOBB4						(AST_VIC_NUM + 222)
#define IRQ_GPIOBB5						(AST_VIC_NUM + 223)
#define IRQ_GPIOBB6						(AST_VIC_NUM + 224)
#define IRQ_GPIOBB7						(AST_VIC_NUM + 225)
#endif
