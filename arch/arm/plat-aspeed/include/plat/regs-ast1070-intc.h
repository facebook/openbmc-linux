/* arch/arm/mach-aspeed/include/mach/regs-intr.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2012/08/15 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __ASPEED_AST1070_INTR_H
#define __ASPEED_AST1070_INTR_H                     1

#include <asm/io.h>
#include <mach/platform.h>
#include <mach/irqs.h>

/*
 * VIC Register (VA)
 */

#define VIC_BASE_VA(x)             		IO_ADDRESS2(AST_C0_VIC_BASE + (0x10000*x)) 

#define AST_IRQ_STS(x)					(VIC_BASE_VA(x) + 0x00)
#define AST_RAW_STS(x)					(VIC_BASE_VA(x) + 0x08)
#define AST_INTR_EN(x)					(VIC_BASE_VA(x) + 0x10)
#define AST_INTR_DIS(x)					(VIC_BASE_VA(x) + 0x14)
#define AST_INTR_SENSE(x)				(VIC_BASE_VA(x) + 0x24)
#define AST_INTR_BOTH_EDGE(x)			(VIC_BASE_VA(x) + 0x28)
#define AST_INTR_EVENT(x)				(VIC_BASE_VA(x) + 0x2C)

#define IRQ_SET_LEVEL_TRIGGER(x,irq_no)   *((volatile unsigned long*)AST_INTR_SENSE(x)) |= 1 << (irq_no)
#define IRQ_SET_EDGE_TRIGGER(x,irq_no)    *((volatile unsigned long*)AST_INTR_SENSE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_RISING_EDGE(x,irq_no)     *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_FALLING_EDGE(x,irq_no)    *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_SET_HIGH_LEVEL(x,irq_no)      *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_LOW_LEVEL(x,irq_no)       *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))


#endif
