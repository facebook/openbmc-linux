/* arch/arm/plat-aspeed/include/mach/regs-iic.h
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

#ifndef __ASM_ARCH_REGS_RTC_H
#define __ASM_ARCH_REGS_RTC_H __FILE__

#define RTC_CNTR_STS_1			0x00
#define RTC_CNTR_STS_2			0x04
#define RTC_ALARM				0x08
#define RTC_CONTROL				0x10
#define RTC_ALARM_STS			0x14

/* RTC_CNTR_STS_1			0x00 */
/* RTC_ALARM				0x08 */
#define GET_DAY_VAL(x)	((x >> 24)&0x1f)
#define GET_HOUR_VAL(x)	((x >> 16)&0x1f)
#define GET_MIN_VAL(x)	((x >> 8)&0x3f)
#define GET_SEC_VAL(x)	(x & 0x3f)

#define SET_DAY_VAL(x)	((x&0x1f) << 24)
#define SET_HOUR_VAL(x)	((x&0x1f) << 16)
#define SET_MIN_VAL(x)	((x&0x3f) << 8)
#define SET_SEC_VAL(x)	(x & 0x3f)

/* RTC_CNTR_STS_2			0x04 */
#define GET_CENT_VAL(x)	((x >> 16)&0x1f)
#define GET_YEAR_VAL(x)	((x >> 8)&0x7f)
#define GET_MON_VAL(x)	(x & 0xf)

#define SET_CENT_VAL(x)	((x &0x1f) << 16)
#define SET_YEAR_VAL(x)	((x &0x7f) << 8)
#define SET_MON_VAL(x)	(x & 0xf)

/* RTC_CONTROL				0x10 */
#define ENABLE_SEC_INTERRUPT	(1 << 7)
#define ENABLE_DAY_ALARM		(1 << 6)
#define ENABLE_HOUR_ALARM		(1 << 5)
#define ENABLE_MIN_ALARM		(1 << 4)
#define ENABLE_SEC_ALARM		(1 << 3)
#define ALARM_MODE_SELECT		(1 << 2)
#define RTC_LOCK				(1 << 1)
#define RTC_ENABLE				(1 << 0)
#define ENABLE_ALL_ALARM		0x0000007c


/* RTC_ALARM_STS			0x14 */
#define SEC_INTERRUPT_STATUS	(1 << 4)
#define DAY_ALARM_STATUS		(1 << 3)
#define HOUR_ALARM_STATUS		(1 << 2)
#define MIN_ALARM_STATUS		(1 << 1)
#define SEC_ALARM_STATUS		(1 << 0)



#endif /* __ASM_ARCH_REGS_RTC_H */
