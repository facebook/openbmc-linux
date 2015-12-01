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
#ifndef __AST1070_INTR_H
#define __AST1070_INTR_H                     1

/*
 * VIC Register (VA)
 */

#define AST1070_IRQ_STS						0x00
#define AST1070_RAW_STS						0x08
#define AST1070_INTR_EN						0x10
#define AST1070_INTR_DIS					0x14
#define AST1070_INTR_SENSE					0x24
#define AST1070_INTR_BOTH_EDGE				0x28
#define AST1070_INTR_EVENT					0x2C

#if 0
#define IRQ_SET_LEVEL_TRIGGER(x,irq_no)   *((volatile unsigned long*)AST1070_INTR_SENSE(x)) |= 1 << (irq_no)
#define IRQ_SET_EDGE_TRIGGER(x,irq_no)    *((volatile unsigned long*)AST1070_INTR_SENSE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_RISING_EDGE(x,irq_no)     *((volatile unsigned long*)AST1070_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_FALLING_EDGE(x,irq_no)    *((volatile unsigned long*)AST1070_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_SET_HIGH_LEVEL(x,irq_no)      *((volatile unsigned long*)AST1070_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_LOW_LEVEL(x,irq_no)       *((volatile unsigned long*)AST1070_INTR_EVENT(x)) &= ~(1 << (irq_no))
#endif

void ast1070_vic_init(u8 vic_nr, u32 base, unsigned int irq_chain, unsigned int irq_chain_start);


#endif
