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

#ifndef _AST1520_PLATFORM_H_
#define _AST1520_PLATFORM_H_                 1

#define AST_SRAM_SIZE					(SZ_8K)

#define AST_AHB_CONTROLLER_BASE            0x1E600000	/* AHB CONTROLLER */

#define AST_SPI_BASE        				0x1E620000	/* SPI CONTROLLER */

#define AST_MAC0_BASE                      0x1E660000	/* MAC1 */ 

#define AST_USB20_BASE                     0x1E6A0000	/* USB 2.0 VIRTUAL HUB CONTROLLER */
#define AST_EHCI_BASE                0x1E6A1000	/* USB 2.0 HOST CONTROLLER */
#define AST_UHCI_BASE                0x1E6B0000	/* USB 1.1 HOST CONTROLLER */
#define AST_VIC_BASE                       0x1E6C0000	/* VIC */
#define AST_MMC_BASE                       0x1E6E0000	/* MMC SDRAM*/
#define AST_SCU_BASE                       0x1E6E2000	/* SCU */
#define AST_CRYPTO_BASE                    0x1E6E3000	/* Crypto */

#define AST_I2S_BASE	                   0x1E6E5000	/* I2S */
#define AST_GRAPHIC_BASE                   0x1E6E6000	/* Graphics */
#define AST_XDMA_BASE                      0x1E6E7000	/* XDMA */
#define AST_MCTP_BASE                      0x1E6E8000	/* MCTP */

#define AST_VIDEO_BASE                     0x1E700000	/* VIDEO ENGINE */
#define AST_SRAM_BASE                      0x1E720000	/* SRAM */
#define AST_SDHC_BASE                        0x1E740000	/* SD */
#define AST_2D_BASE                        0x1E760000	/* 2D */
#define AST_GPIO_BASE                      0x1E780000	/* GPIO */
#define AST_RTC_BASE                       0x1E781000	/* RTC */
#define AST_TIMER_BASE                     0x1E782000	/* TIMER #0~2*/
#define AST_UART1_BASE                     0x1E783000	/* UART1 */
#define AST_UART0_BASE                     0x1E784000	/* UART3 */
#define AST_WDT_BASE                       0x1E785000	/* WDT */

#define AST_I2C_BASE                       0x1E78A000	/* I2C */
#define AST_UART2_BASE                     0x1E78D000	/* UART2 */

#define AST_SPI_MEM							0x20000000
#define AST_PCIE_BRIDGE						0x70000000

#endif
