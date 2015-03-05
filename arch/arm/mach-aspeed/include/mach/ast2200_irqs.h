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

#ifndef _AST2200_IRQS_H_
#define _AST2200_IRQS_H_                 1

#define NR_IRQS                         (AST_VIC_NUM + ARCH_NR_GPIOS)
//--------------GPIO ---------------------------------------------------------------
#define ARCH_NR_GPIOS 					(GPIO_PORT_NUM*8)
#define IRQ_GPIO_CHAIN_START			AST_VIC_NUM

#define AST_VIC_NUM			32

#define IRQ_SDRAM_ECC					0
#define IRQ_MIC							1
#define IRQ_MAC0						2			/* MAC 1 interrupt */
#define IRQ_MAC1						3			/* MAC 2 interrupt */
#define IRQ_CRYPTO						4
#define IRQ_USB20_HUB					5
#define IRQ_EHCI					5
#define IRQ_XDMA						6
#define IRQ_VIDEO						7
#define IRQ_LPC							8
#define IRQ_UART0						9			/* UART 1 interrupt */
#define IRQ_UART1						10			/* UART 2 interrupt */
//11 reserved 
#define IRQ_I2C							12
#define IRQ_UDC11					13
//14 reserved 
#define IRQ_PECI						15
#define IRQ_TIMER0						16			/* TIMER 1 interrupt */
#define IRQ_TIMER1						17			/* TIMER 2 interrupt */
#define IRQ_TIMER2						18			/* TIMER 3 interrupt */
#define IRQ_SMC							19
#define IRQ_GPIO						20
#define IRQ_SCU							21
#define IRQ_RTC_SEC						22
#define IRQ_RTC_DAY						23
#define IRQ_RTC_HOUR					24
#define IRQ_RTC_MIN						25
#define IRQ_RTC							26
#define IRQ_WDT							27
#define IRQ_TACHO						28
#define IRQ_2D							29
#define IRQ_PCI							30
#define IRQ_AHBC						31

#endif
