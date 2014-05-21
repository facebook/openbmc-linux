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

#define AST_C0_BASE                    		(AST_LPC_BRIDGE)

#define AST_C0_UART0_BASE                    (AST_C0_BASE)	/* Companion UART1 */
#define AST_C0_UART1_BASE                    (AST_C0_BASE + 0x400)	/* Companion UART2 */
#define AST_C0_UART2_BASE                    (AST_C0_BASE + 0x800)	/* Companion UART3 */
#define AST_C0_UART3_BASE                    (AST_C0_BASE + 0xc00)	/* Companion UART4 */
#define AST_C0_LPC0_BASE                     (AST_C0_BASE + 0x1000)	/* Companion LPC1 */
#define AST_C0_LPC1_BASE                     (AST_C0_BASE + 0x1400)	/* Companion LPC2 */
#define AST_C0_LPC2_BASE                     (AST_C0_BASE + 0x1800)	/* Companion LPC3 */
#define AST_C0_LPC3_BASE                     (AST_C0_BASE + 0x1c00)	/* Companion LPC4 */
#define AST_C0_SCU_BASE                      (AST_C0_BASE + 0x2000)	/* Companion SCU */
#define AST_C0_VIC_BASE                      (AST_C0_BASE + 0x2400)	/* Companion VIC */
#define AST_C0_LPC_SLAVE_BASE                (AST_C0_BASE + 0x2c00)	/* Companion LPC SlLAVE */
#define AST_C0_I2C_BASE                      (AST_C0_BASE + 0x3000)	/* Companion I2C */
#define AST_C0_SPI_BASE                      (AST_C0_BASE + 0x4000)	/* Companion SPI */
#define AST_C0_LPC_SPI_BASE                  (AST_C0_BASE + 0x4400)	/* Companion LPC SPI */
#define AST_C0_UART_DMA_BASE                 (AST_C0_BASE + 0x4800)	/* Companion UART DMA */
#define AST_C0_SPI_CONTROL_BASE              (AST_C0_BASE + 0x4c00)	/* Companion SPI CONTROL */
#define AST_C0_SPI_SHADOW_SRAM_BASE          (AST_C0_BASE + 0x5000)	/* Companion SPI SHADOW SRAM */

#define AST_C1_BASE                    		(AST_LPC_BRIDGE + 0x10000)	

#define AST_C1_UART0_BASE                    (AST_C1_BASE)			/* Companion UART1 */
#define AST_C1_UART1_BASE                    (AST_C1_BASE + 0x400)	/* Companion UART2 */
#define AST_C1_UART2_BASE                    (AST_C1_BASE + 0x800)	/* Companion UART3 */
#define AST_C1_UART3_BASE                    (AST_C1_BASE + 0xc00)	/* Companion UART4 */
#define AST_C1_LPC0_BASE                     (AST_C1_BASE + 0x1000)	/* Companion LPC1 */
#define AST_C1_LPC1_BASE                     (AST_C1_BASE + 0x1400)	/* Companion LPC2 */
#define AST_C1_LPC2_BASE                     (AST_C1_BASE + 0x1800)	/* Companion LPC3 */
#define AST_C1_LPC3_BASE                     (AST_C1_BASE + 0x1c00)	/* Companion LPC4 */
#define AST_C1_SCU_BASE                      (AST_C1_BASE + 0x2000)	/* Companion SCU */
#define AST_C1_VIC_BASE                      (AST_C1_BASE + 0x2400)	/* Companion VIC */
#define AST_C1_LPC_SLAVE_BASE                (AST_C1_BASE + 0x2c00)	/* Companion LPC SlLAVE */
#define AST_C1_I2C_BASE                      (AST_C1_BASE + 0x3000)	/* Companion I2C */
#define AST_C1_SPI_BASE                      (AST_C1_BASE + 0x4000)	/* Companion SPI */
#define AST_C1_LPC_SPI_BASE                  (AST_C1_BASE + 0x4400)	/* Companion LPC SPI */
#define AST_C1_UART_DMA_BASE                 (AST_C1_BASE + 0x4800)	/* Companion UART DMA */
#define AST_C1_SPI_CONTROL_BASE              (AST_C1_BASE + 0x4c00)	/* Companion SPI CONTROL */
#define AST_C1_SPI_SHADOW_SRAM_BASE          (AST_C1_BASE + 0x5000)	/* Companion SPI SHADOW SRAM */

#define AST_C2_BASE                    		(AST_LPC_BRIDGE + 0x20000)	

#define AST_C2_UART0_BASE                    (AST_C2_BASE)			/* Companion UART1 */
#define AST_C2_UART1_BASE                    (AST_C2_BASE + 0x400)	/* Companion UART2 */
#define AST_C2_UART2_BASE                    (AST_C2_BASE + 0x800)	/* Companion UART3 */
#define AST_C2_UART3_BASE                    (AST_C2_BASE + 0xc00)	/* Companion UART4 */
#define AST_C2_LPC1_BASE                     (AST_C2_BASE + 0x1000)	/* Companion LPC1 */
#define AST_C2_LPC2_BASE                     (AST_C2_BASE + 0x1400)	/* Companion LPC2 */
#define AST_C2_LPC3_BASE                     (AST_C2_BASE + 0x1800)	/* Companion LPC3 */
#define AST_C2_LPC4_BASE                     (AST_C2_BASE + 0x1c00)	/* Companion LPC4 */
#define AST_C2_SCU_BASE                      (AST_C2_BASE + 0x2000)	/* Companion SCU */
#define AST_C2_VIC_BASE                      (AST_C2_BASE + 0x2400)	/* Companion VIC */
#define AST_C2_LPC_SLAVE_BASE                (AST_C2_BASE + 0x2c00)	/* Companion LPC SlLAVE */
#define AST_C2_I2C_BASE                      (AST_C2_BASE + 0x3000)	/* Companion I2C */
#define AST_C2_SPI_BASE                      (AST_C2_BASE + 0x4000)	/* Companion SPI */
#define AST_C2_LPC_SPI_BASE                  (AST_C2_BASE + 0x4400)	/* Companion LPC SPI */
#define AST_C2_UART_DMA_BASE                 (AST_C2_BASE + 0x4800)	/* Companion UART DMA */
#define AST_C2_SPI_CONTROL_BASE              (AST_C2_BASE + 0x4c00)	/* Companion SPI CONTROL */
#define AST_C2_SPI_SHADOW_SRAM_BASE          (AST_C2_BASE + 0x5000)	/* Companion SPI SHADOW SRAM */

#define AST_C3_BASE                    		(AST_LPC_BRIDGE + 0x30000)	

#define AST_C3_UART0_BASE                    (AST_C3_BASE)			/* Companion UART1 */
#define AST_C3_UART1_BASE                    (AST_C3_BASE + 0x400)	/* Companion UART2 */
#define AST_C3_UART2_BASE                    (AST_C3_BASE + 0x800)	/* Companion UART3 */
#define AST_C3_UART3_BASE                    (AST_C3_BASE + 0xc00)	/* Companion UART4 */
#define AST_C3_LPC0_BASE                     (AST_C3_BASE + 0x1000)	/* Companion LPC1 */
#define AST_C3_LPC1_BASE                     (AST_C3_BASE + 0x1400)	/* Companion LPC2 */
#define AST_C3_LPC2_BASE                     (AST_C3_BASE + 0x1800)	/* Companion LPC3 */
#define AST_C3_LPC3_BASE                     (AST_C3_BASE + 0x1c00)	/* Companion LPC4 */
#define AST_C3_SCU_BASE                      (AST_C3_BASE + 0x2000)	/* Companion SCU */
#define AST_C3_VIC_BASE                      (AST_C3_BASE + 0x2400)	/* Companion VIC */
#define AST_C3_LPC_SLAVE_BASE                (AST_C3_BASE + 0x2c00)	/* Companion LPC SlLAVE */
#define AST_C3_I2C_BASE                      (AST_C3_BASE + 0x3000)	/* Companion I2C */
#define AST_C3_SPI_BASE                      (AST_C3_BASE + 0x4000)	/* Companion SPI */
#define AST_C3_LPC_SPI_BASE                  (AST_C3_BASE + 0x4400)	/* Companion LPC SPI */
#define AST_C3_UART_DMA_BASE                 (AST_C3_BASE + 0x4800)	/* Companion UART DMA */
#define AST_C3_SPI_CONTROL_BASE              (AST_C3_BASE + 0x4c00)	/* Companion SPI CONTROL */
#define AST_C3_SPI_SHADOW_SRAM_BASE          (AST_C3_BASE + 0x5000)	/* Companion SPI SHADOW SRAM */

#endif
