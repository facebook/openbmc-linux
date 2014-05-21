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
#ifndef __ASPEED_AST_INTR_H
#define __ASPEED_AST_INTR_H                     1

#ifndef __ASSEMBLY__
#include <asm/io.h>
#endif
//============== INTERRUPT========================================
#include <mach/platform.h>
#include <mach/irqs.h>
#include <plat/aspeed.h>

/*
 * VIC Register (VA)
 */

#define VIC_BASE_VA              		IO_ADDRESS(AST_VIC_BASE) 

#if defined(NEW_VIC)
//New Mappling

#define AST_IRQ_STS(x)				(VIC_BASE_VA + 0x80 + (x*0x04))
#define AST_FIQ_STS(x)				(VIC_BASE_VA + 0x88 + (x*0x04))
#define AST_RAW_STS(x)				(VIC_BASE_VA + 0x90 + (x*0x04))
#define AST_INTR_SEL(x)				(VIC_BASE_VA + 0x98 + (x*0x04))
#define AST_INTR_EN(x)				(VIC_BASE_VA + 0xA0 + (x*0x04))
#define AST_INTR_DIS(x)				(VIC_BASE_VA + 0xA8 + (x*0x04))
#define AST_INTR_SW_EN(x)			(VIC_BASE_VA + 0xB0 + (x*0x04))
#define AST_INTR_SW_CLR(x)			(VIC_BASE_VA + 0xB8 + (x*0x04))
#define AST_INTR_SENSE(x)			(VIC_BASE_VA + 0xC0 + (x*0x04))
#define AST_INTR_BOTH_EDGE(x)		(VIC_BASE_VA + 0xC8 + (x*0x04))
#define AST_INTR_EVENT(x)			(VIC_BASE_VA + 0xD0 + (x*0x04))
#define AST_INTR_EDGE_CLR(x)		(VIC_BASE_VA + 0xD8 + (x*0x04))
#define AST_INTR_EDGE_STS(x)		(VIC_BASE_VA + 0xE0 + (x*0x04))

#else

//Legacy Maping 

#define AST_IRQ_STS(x)				(VIC_BASE_VA + 0x00)
#define AST_FIQ_STS(x)				(VIC_BASE_VA + 0x04)
#define AST_RAW_STS(x)				(VIC_BASE_VA + 0x08)
#define AST_INTR_SEL(x)				(VIC_BASE_VA + 0x0C)
#define AST_INTR_EN(x)				(VIC_BASE_VA + 0x10)
#define AST_INTR_DIS(x)				(VIC_BASE_VA + 0x14)
#define AST_INTR_SW_EN(x)			(VIC_BASE_VA + 0x18)
#define AST_INTR_SW_CLR(x)			(VIC_BASE_VA + 0x1C)
#define AST_INTR_SENSE(x)			(VIC_BASE_VA + 0x24)
#define AST_INTR_BOTH_EDGE(x)		(VIC_BASE_VA + 0x28)
#define AST_INTR_EVENT(x)			(VIC_BASE_VA + 0x2C)
#define AST_INTR_EDGE_CLR(x)		(VIC_BASE_VA + 0x38)
#endif

#define IRQ_SET_LEVEL_TRIGGER(x, irq_no)   *((volatile unsigned long*)AST_INTR_SENSE(x)) |= 1 << (irq_no)
#define IRQ_SET_EDGE_TRIGGER(x, irq_no)    *((volatile unsigned long*)AST_INTR_SENSE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_RISING_EDGE(x, irq_no)     *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_FALLING_EDGE(x, irq_no)    *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_SET_HIGH_LEVEL(x,irq_no)      *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_LOW_LEVEL(x, irq_no)       *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_EDGE_CLEAR(x, irq_no)          *((volatile unsigned long*)AST_INTR_EDGE_CLR(x)) |= 1 << (irq_no)

#endif

