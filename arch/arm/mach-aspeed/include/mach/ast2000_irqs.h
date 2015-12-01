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

#ifndef _AST2000_IRQS_H_
#define _AST2000_IRQS_H_                 1

#define NR_IRQS                         (AST_VIC_NUM + ARCH_NR_GPIOS)
//--------------GPIO ---------------------------------------------------------------
#define ARCH_NR_GPIOS 					(GPIO_PORT_NUM*8)
#define IRQ_GPIO_CHAIN_START			AST_VIC_NUM

#define AST_VIC_NUM			32

#define IRQ_SPI							0
#define IRQ_UART0						1
#define IRQ_UART1						2
#define IRQ_TIMER0						3
#define IRQ_TIMER1						4
#define IRQ_TIMER2						5
#define IRQ_RTC							6
#define IRQ_MAC0						7
#define IRQ_GPIO_B0						8
#define IRQ_UDC							9
#define IRQ_PCI							10
#define IRQ_GPIO_B1						11
#define IRQ_GPIO_B2						12
#define IRQ_GPIO_B3						13
#define IRQ_LPC							14
#define IRQ_I2C							15
#define IRQ_USB11						16
#define IRQ_VIDEO						17
#define IRQ_CRYPTO						18
#define IRQ_SCU							19
#define IRQ_GPIO_B4						20
#define IRQ_GPIO_B5						21
#define IRQ_GPIO_B6						22
#define IRQ_GPIO_A0						23
#define IRQ_GPIO_A1						24
#define IRQ_GPIO_A2						25
#define IRQ_GPIO_A3						26
#define IRQ_HDMA						27
#define IRQ_GPIO_A4						28
#define IRQ_GPIO_A5						29
#define IRQ_GPIO_A6						30
#define IRQ_WDT							31

#endif
