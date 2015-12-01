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

#ifndef _AST1010_PLATFORM_H_
#define _AST1010_PLATFORM_H_                 1

/*#define AST_DRAM_BASE				0x40000000*/
#define AST_SRAM_SIZE				(SZ_64K)

#define REMAP_AREA0_BASE			0x00000000	// Map to SPI memory, big endian
#define REMAP_AREA1_BASE			0x00200000	// Remap area 1
#define REMAP_AREA2_BASE			0x00300000	// Remap area 2
#define REMAP_AREA3_BASE			0x00400000	// Remap area 3
#define REMAP_AREA4_BASE			0x00500000	// Remap area 4
#define REMAP_AREA5_BASE			0x00600000	// Remap area 5
#define REMAP_AREA6_BASE			0x00700000	// Remap area 6
#define REMAP_AREA7_BASE			0x00800000	// Remap area 7, default map to DRAM memory, big endian

//#define FLASH_BASE_ADDR			REMAP_AREA0_BASE
#define AST_IO_BASE					REMAP_AREA1_BASE
#define AST_SRAM_BASE				REMAP_AREA2_BASE
#define AST_DRAM_BASE				REMAP_AREA7_BASE
#define AST_SPI0_BUFF				CONFIG_SPI_AST_SPI0_BUFF
#define AST_SPI1_BUFF				(CONFIG_SPI_AST_SPI1_BUFF + 0x00018000)

#define AST_SPI0_BASE				(AST_IO_BASE + 0x00000000)
//#define AST_SPI1_BASE				(AST_IO_BASE + 0x00010000)
#define AST_VIC_BASE				(AST_IO_BASE + 0x00020000)	/* VIC */
#define AST_MAC0_BASE				(AST_IO_BASE + 0x00030000)	/* MAC1 */
#define AST_MMC_BASE				(AST_IO_BASE + 0x00040000)	/* MMC */
#define AST_SCU_BASE				(AST_IO_BASE + 0x00041000)	/* SCU */
#define AST_TIMER_BASE				(AST_IO_BASE + 0x00042000)	/* Timer */
#define AST_WDT_BASE				(AST_IO_BASE + 0x00043000)	/* WDT */
#define AST_I2C_BASE				(AST_IO_BASE + 0x00044000)	/* I2C */
#define AST_UART1_BASE				(AST_IO_BASE + 0x00045000)	/* UART1 */
#define AST_UART2_BASE				(AST_IO_BASE + 0x00046000)	/* UART2 */
#define AST_UART0_BASE				(AST_IO_BASE + 0x00047000)	/* UART3, system debug console */
#define AST_VUART0_BASE			(AST_IO_BASE + 0x00048000)	/* VUART1 */
#define AST_LPC_BASE				(AST_IO_BASE + 0x0004a000)	/* LPC */
#define AST_GPIO_BASE				(AST_IO_BASE + 0x0004b000)	/* GPIO */
#define AST_JTAG_BASE				(AST_IO_BASE + 0x0004c000)	/* JTAG */
#define AST_ADC_BASE				(AST_IO_BASE + 0x0004d000)	/* ADC */
#define AST_PWM_BASE				(AST_IO_BASE + 0x0004e000)	/* PWM */
#define AST_PECI_BASE				(AST_IO_BASE + 0x0004f000)	/* PECI */
#define AST_SGPIO_BASE				(AST_IO_BASE + 0x00050000)	/* SGPIO */
#define AST_LINEFLUSH_BASE			(AST_IO_BASE + 0x000f0000)	/* Line cache flush */


#endif
