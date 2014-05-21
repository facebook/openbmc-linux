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

#ifndef _AST2400_PLATFORM_H_
#define _AST2400_PLATFORM_H_                 1

#define AST_DRAM_BASE					0x40000000

#define AST_SRAM_SIZE					(SZ_16K*2)

#define AST_OLD_SMC_BASE					0x10000000		/*Legacy BMC Static Memory */
#define AST_OLD_SMC_CTRL_BASE				0x16000000		/*Legacy BMC Static Memory Ctrl*/

#define AST_AHB_CTRL_BASE            0x1E600000	/* AHB CONTROLLER */

#define AST_FMC_BASE        					0x1E620000	/* NEW SMC CONTROLLER */
#define AST_SPI_BASE        					0x1E630000	/* SPI CONTROLLER */
#define AST_MIC_BASE                       0x1E640000	/* MIC CONTROLLER */
#define AST_MAC0_BASE                      0x1E660000	/* MAC1 */ 
#define AST_MAC1_BASE                      0x1E680000	/* MAC2 */

#define AST_USB20_BASE                     0x1E6A0000	/* USB 2.0 VIRTUAL HUB CONTROLLER */
#define AST_EHCI_BASE                0x1E6A1000	/* USB 2.0 HOST CONTROLLER */
#define AST_UHCI_BASE                0x1E6B0000	/* USB 1.1 HOST CONTROLLER */
#define AST_VIC_BASE                       0x1E6C0000	/* VIC */
#define AST_SDMC_BASE                       0x1E6E0000	/* SDRAM CTRL */
#define AST_UDC11_BASE                     0x1E6E1000	/* USB11 */
#define AST_SCU_BASE                       0x1E6E2000	/* SCU */
#define AST_CRYPTO_BASE                    0x1E6E3000	/* Crypto */
#define AST_JTAG_BASE                      0x1E6E4000	/* JTAG */
#define AST_GRAPHIC_BASE                   0x1E6E6000	/* Graphics */
#define AST_XDMA_BASE                      0x1E6E7000	/* XDMA */
#define AST_MCTP_BASE                      0x1E6E8000	/* MCTP */
#define AST_ADC_BASE                       0x1E6E9000	/* ADC */

#define AST_LPC_PLUS_BASE                  0x1E6EC000	/* LPC+ Controller */

#define AST_VIDEO_BASE                     0x1E700000	/* VIDEO ENGINE */
#define AST_SRAM_BASE                      0x1E720000	/* SRAM */
#define AST_SDHC_BASE                      0x1E740000	/* SDHC */
#define AST_2D_BASE                        0x1E760000	/* 2D */
#define AST_GPIO_BASE                      0x1E780000	/* GPIO */
#define AST_RTC_BASE                       0x1E781000	/* RTC */
#define AST_TIMER_BASE                     0x1E782000	/* TIMER #0~7*/
#define AST_UART1_BASE                     0x1E783000	/* UART1 */
#define AST_UART0_BASE                     0x1E784000	/* UART5 */
#define AST_WDT_BASE                       0x1E785000	/* WDT */
#define AST_PWM_BASE                       0x1E786000	/* PWM */
#define AST_VUART0_BASE                    0x1E787000	/* VUART1 */
#define AST_PUART_BASE                     0x1E788000	/* PUART */
#define AST_LPC_BASE                       0x1E789000	/* LPC */
#define AST_MBX_BASE                       0x1E789200	/* MailBox */
#define AST_I2C_BASE                       0x1E78A000	/* I2C */
#define AST_PECI_BASE                      0x1E78B000	/* PECI */
#define AST_PCIARBITER_BASE                0x1E78C000	/* PCI ARBITER */
#define AST_UART2_BASE                     0x1E78D000	/* UART2 */
#define AST_UART3_BASE                     0x1E78E000	/* UART3 */
#define AST_UART4_BASE                     0x1E78F000	/* UART4 */
#define AST_SPI_MEM						  0x30000000

#if defined(CONFIG_AST_LPC_PLUS)
#define AST_LPC_BRIDGE						0x70000000
#else
#define AST_LPC_BRIDGE						0x60000000
#endif


#endif
