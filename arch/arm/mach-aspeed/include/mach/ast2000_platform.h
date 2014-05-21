/*
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

#ifndef _AST2000_PLATFORM_H_
#define _AST2000_PLATFORM_H_                 1

#define AST_MAC0_BASE                      0x19c80000	/* MAC1 */ 
#define AST_CRYPTO_BASE                    0x1E6E0040	/* Crypto */
#define AST_USB11_BASE                     0x1E6E0080	/* USB11 */
#define AST_SCU0_BASE                      0x1E6E0100	/* SCU1 */
#define AST_LPC_BASE                       0x1E6E0400	/* LPC */
#define AST_I2C_BASE                       0x1E6E0800	/* I2C */
//---//
#define AST_VIDEO_BASE                     0x1E700000	/* VIDEO ENGINE */
#define AST_AHB_TO_PBUS_BASE               0x1E720000	/* APB -> PBUS */
//...//
#define AST_HDMA_BASE	                  0x1E7c0000	/* HDMA */
#define AST_TIMER_BASE	                  0x1E800000	/* TIMER0/1/2 */
#define AST_RTC_BASE		                  0x1E820000	/* RTC */
#define AST_UART0_BASE	                  0x1E840000	/* UART0 */
#define AST_UART1_BASE	                  0x1E860000	/* UART1 */
#define AST_SPI_BASE		                  0x1E880000	/* SPI */
#define AST_GPIO_BASE	                  0x1E8A0000	/* GPIO */
#define AST_WDT_BASE		                  0x1E8C0000	/* WDT */
#define AST_SCU0_BASE                      0x1E8E000c	/* SCU2 */
 
#endif
