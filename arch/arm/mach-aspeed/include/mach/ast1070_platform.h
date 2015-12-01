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

#ifndef _AST1070_PLATFORM_H_
#define _AST1070_PLATFORM_H_                 1

#define MAX_AST1070_NR		2

#if 1	//use lpc+ interface 700n:0000
#define AST1070_C0_UART0_BASE				0x70000000	/* Companion UART1 */
#define AST1070_C0_UART1_BASE				0x70000400	/* Companion UART2 */
#define AST1070_C0_UART2_BASE				0x70000800	/* Companion UART3 */
#define AST1070_C0_UART3_BASE				0x70000C00	/* Companion UART4 */
#define AST1070_C0_LPC0_BASE				0x70001000	/* Companion LPC1 */
#define AST1070_C0_LPC1_BASE				0x70001400	/* Companion LPC2 */
#define AST1070_C0_LPC2_BASE				0x70001800	/* Companion LPC3 */
#define AST1070_C0_LPC3_BASE				0x70001C00	/* Companion LPC4 */
#define AST1070_C0_SCU_BASE				0x70002000	/* Companion SCU */
#define AST1070_C0_VIC_BASE				0x70002400	/* Companion VIC */
#define AST1070_C0_LPC_SLAVE_BASE			0x70002c00	/* Companion LPC SlLAVE */
#define AST1070_C0_I2C_BASE				0x70003000	/* Companion I2C */
#define AST1070_C0_I2C_DEV0_BASE				0x70003040	/* Companion I2C DEV 0*/
#define AST1070_C0_I2C_DEV1_BASE				0x70003080	/* Companion I2C DEV 1*/
#define AST1070_C0_I2C_DEV2_BASE				0x700030C0	/* Companion I2C DEV 2*/
#define AST1070_C0_I2C_DEV3_BASE				0x70003100	/* Companion I2C DEV 3*/
#define AST1070_C0_I2C_DEV4_BASE				0x70003140	/* Companion I2C DEV 4*/
#define AST1070_C0_I2C_DEV5_BASE				0x70003180	/* Companion I2C DEV 5*/
#define AST1070_C0_I2C_DEV6_BASE				0x700031C0	/* Companion I2C DEV 6*/
#define AST1070_C0_I2C_DEV7_BASE				0x70003200	/* Companion I2C DEV 7*/
#define AST1070_C0_SPI_BASE				0x70004000	/* Companion SPI */
#define AST1070_C0_LPC_SPI_BASE			0x70004400	/* Companion LPC SPI */
#define AST1070_C0_UART_DMA_BASE			0x70004800	/* Companion UART DMA */
#define AST1070_C0_SPI_CONTROL_BASE		0x70004c00	/* Companion SPI CONTROL */
#define AST1070_C0_SPI_SHADOW_SRAM_BASE	0x70005000	/* Companion SPI SHADOW SRAM */

#define AST1070_C1_UART0_BASE				0x70010000	/* Companion UART1 */
#define AST1070_C1_UART1_BASE				0x70010400	/* Companion UART2 */
#define AST1070_C1_UART2_BASE				0x70010800	/* Companion UART3 */
#define AST1070_C1_UART3_BASE				0x70010C00	/* Companion UART4 */
#define AST1070_C1_LPC0_BASE					0x70011000	/* Companion LPC1 */
#define AST1070_C1_LPC1_BASE					0x70011400	/* Companion LPC2 */
#define AST1070_C1_LPC2_BASE					0x70011800	/* Companion LPC3 */
#define AST1070_C1_LPC3_BASE					0x70011C00	/* Companion LPC4 */
#define AST1070_C1_SCU_BASE					0x70012000	/* Companion SCU */
#define AST1070_C1_VIC_BASE					0x70012400	/* Companion VIC */
#define AST1070_C1_LPC_SLAVE_BASE			0x70012c00	/* Companion LPC SlLAVE */
#define AST1070_C1_I2C_BASE					0x70013000	/* Companion I2C */
#define AST1070_C1_I2C_DEV0_BASE				0x70013040	/* Companion I2C DEV 0*/
#define AST1070_C1_I2C_DEV1_BASE				0x70013080	/* Companion I2C DEV 1*/
#define AST1070_C1_I2C_DEV2_BASE				0x700130C0	/* Companion I2C DEV 2*/
#define AST1070_C1_I2C_DEV3_BASE				0x70013100	/* Companion I2C DEV 3*/
#define AST1070_C1_I2C_DEV4_BASE				0x70013140	/* Companion I2C DEV 4*/
#define AST1070_C1_I2C_DEV5_BASE				0x70013180	/* Companion I2C DEV 5*/
#define AST1070_C1_I2C_DEV6_BASE				0x700131C0	/* Companion I2C DEV 6*/
#define AST1070_C1_I2C_DEV7_BASE				0x70013200	/* Companion I2C DEV 7*/
#define AST1070_C1_SPI_BASE					0x70014000	/* Companion SPI */
#define AST1070_C1_LPC_SPI_BASE				0x70014400	/* Companion LPC SPI */
#define AST1070_C1_UART_DMA_BASE			0x70014800	/* Companion UART DMA */
#define AST1070_C1_SPI_CONTROL_BASE			0x70014c00	/* Companion SPI CONTROL */
#define AST1070_C1_SPI_SHADOW_SRAM_BASE	0x70015000	/* Companion SPI SHADOW SRAM */
#else

#define AST1070_UART0_BASE(lpc)						(lpc + 0x000)	/* Companion UART1 */
#define AST1070_UART1_BASE(lpc)						(lpc + 0x400)	/* Companion UART2 */
#define AST1070_UART2_BASE(lpc)						(lpc + 0x800)	/* Companion UART3 */
#define AST1070_UART3_BASE(lpc)						(lpc + 0xc00)	/* Companion UART4 */
#define AST1070_LPC0_BASE(lpc)						(lpc + 0x1000)	/* Companion LPC1 */
#define AST1070_LPC1_BASE(lpc)						(lpc + 0x1400)	/* Companion LPC2 */
#define AST1070_LPC2_BASE(lpc)						(lpc + 0x1800)	/* Companion LPC3 */
#define AST1070_LPC3_BASE(lpc)						(lpc + 0x1c00)	/* Companion LPC4 */
#define AST1070_SCU_BASE(lpc)						(lpc + 0x2000)	/* Companion SCU */
#define AST1070_VIC_BASE(lpc)						(lpc + 0x2400)	/* Companion VIC */
#define AST1070_LPC_SLAVE_BASE(lpc)					(lpc + 0x2c00)	/* Companion LPC SlLAVE */
#define AST1070_I2C_BASE(lpc)						(lpc + 0x3000)	/* Companion I2C */
#define AST1070_SPI_BASE(lpc)						(lpc + 0x4000)	/* Companion SPI */
#define AST1070_LPC_SPI_BASE(lpc)					(lpc + 0x4400)	/* Companion LPC SPI */
#define AST1070_UART_DMA_BASE(lpc)					(lpc + 0x4800)	/* Companion UART DMA */
#define AST1070_SPI_CONTROL_BASE(lpc)				(lpc + 0x4c00)	/* Companion SPI CONTROL */
#define AST1070_SPI_SHADOW_SRAM_BASE(lpc)			(lpc + 0x5000)	/* Companion SPI SHADOW SRAM */
#endif

#endif
