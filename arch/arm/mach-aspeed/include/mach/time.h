/*
 *  time.h
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
#include <asm/system.h>
#include <asm/mach/time.h>
#include <asm/param.h>

/*
 * How long is the timer interval?
 */
#define TIMER_INTERVAL	(ASPEED_TIMER_CLKRATE / HZ)
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)

/*
 * Timer
 */
#define ASPEED_TIMER0_OFFSET		0x0000		/* Timer0 Offset */
#define ASPEED_TIMER1_OFFSET		0x0010		/* Timer1 Offset */
#define ASPEED_TIMER2_OFFSET		0x0020		/* Timer2 Offset */
#define ASPEED_TIMERRC_OFFSET		0x0030		/* Timer RC Offset */

#define ASPEED_TIMER_CLKRATE		(ASPEED_EXTCLK)
#define ASPEED_EXTCLK			(1*1000*1000)	/* 1M */

/*
 * Ticks
 */
//#define TICKS_PER_uSEC                  40 // IP Cam
//#define TICKS_PER_uSEC                  24 // FPGA
#define TICKS_PER_uSEC                  1		/* ASPEED_EXTCLK / 10 ^ 6 */

#define mSEC_1                          1000
#define mSEC_5                          (mSEC_1 * 5)
#define mSEC_10                         (mSEC_1 * 10)
#define mSEC_25                         (mSEC_1 * 25)
#define SEC_1                           (mSEC_1 * 1000)

/*
 * Timer Control
 */
#define TIMER0_ENABLE   0x0001
#define TIMER1_ENABLE   0x0010
#define TIMER2_ENABLE   0x0100

#define TIMER0_RefExt   0x0002
#define TIMER1_RefExt   0x0020
#define TIMER2_RefExt   0x0200

/*
 * What does it look like?
 */
typedef struct TimerStruct {
	unsigned long TimerValue;
	unsigned long TimerLoad;
	unsigned long TimerMatch1;
	unsigned long TimerMatch2;
} TimerStruct_t;

