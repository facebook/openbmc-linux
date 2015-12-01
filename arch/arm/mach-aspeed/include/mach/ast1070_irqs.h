/*
 *  arch/arm/plat-aspeed/include/plat/irqs.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
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

#ifndef _AST1070_IRQS_H_
#define _AST1070_IRQS_H_                 1

#define IRQ_C0_VIC_CHAIN					IRQ_EXT0
#define IRQ_C0_VIC_CHAIN_START			(AST_VIC_NUM)

#define IRQ_C1_VIC_CHAIN					IRQ_EXT1
#define IRQ_C1_VIC_CHAIN_START			(IRQ_C0_VIC_CHAIN_START + AST_CVIC_NUM)

#define IRQ_C2_VIC_CHAIN					IRQ_EXT2
#define IRQ_C2_VIC_CHAIN_START			(IRQ_C1_VIC_CHAIN_START + AST_CVIC_NUM)

#define IRQ_C3_VIC_CHAIN					IRQ_EXT3
#define IRQ_C3_VIC_CHAIN_START			(IRQ_C2_VIC_CHAIN_START + AST_CVIC_NUM)

#define AST_CVIC_NUM 					25

#define IRQ_C0_N1_KCS                     (IRQ_C0_VIC_CHAIN_START + 0)
#define IRQ_C0_N1_UART                    (IRQ_C0_VIC_CHAIN_START + 1)
#define IRQ_C0_N1_MAILBOX                 (IRQ_C0_VIC_CHAIN_START + 2)
#define IRQ_C0_N1_PORT80                  (IRQ_C0_VIC_CHAIN_START + 3)
#define IRQ_C0_N1_RESET                   (IRQ_C0_VIC_CHAIN_START + 4)
#define IRQ_C0_N2_KCS                     (IRQ_C0_VIC_CHAIN_START + 5)
#define IRQ_C0_N2_UART                    (IRQ_C0_VIC_CHAIN_START + 6)
#define IRQ_C0_N2_MAILBOX                 (IRQ_C0_VIC_CHAIN_START + 7)
#define IRQ_C0_N2_PORT80                  (IRQ_C0_VIC_CHAIN_START + 8)
#define IRQ_C0_N2_RESET                   (IRQ_C0_VIC_CHAIN_START + 9)
#define IRQ_C0_N3_KCS                     (IRQ_C0_VIC_CHAIN_START + 10)
#define IRQ_C0_N3_UART                    (IRQ_C0_VIC_CHAIN_START + 11)
#define IRQ_C0_N3_MAILBOX                 (IRQ_C0_VIC_CHAIN_START + 12)
#define IRQ_C0_N3_PORT80                  (IRQ_C0_VIC_CHAIN_START + 13)
#define IRQ_C0_N3_RESET                   (IRQ_C0_VIC_CHAIN_START + 14)
#define IRQ_C0_N4_KCS                     (IRQ_C0_VIC_CHAIN_START + 15)
#define IRQ_C0_N4_UART                    (IRQ_C0_VIC_CHAIN_START + 16)
#define IRQ_C0_N4_MAILBOX                 (IRQ_C0_VIC_CHAIN_START + 17)
#define IRQ_C0_N4_PORT80                  (IRQ_C0_VIC_CHAIN_START + 18)
#define IRQ_C0_N4_RESET                   (IRQ_C0_VIC_CHAIN_START + 19)
#define IRQ_C0_N1_UART_DMA                (IRQ_C0_VIC_CHAIN_START + 20)
#define IRQ_C0_N2_UART_DMA                (IRQ_C0_VIC_CHAIN_START + 21)
#define IRQ_C0_N3_UART_DMA                (IRQ_C0_VIC_CHAIN_START + 22)
#define IRQ_C0_N4_UART_DMA                (IRQ_C0_VIC_CHAIN_START + 23)
#define IRQ_C0_I2C						(IRQ_C0_VIC_CHAIN_START + 24)

#define IRQ_C1_N1_KCS                     (IRQ_C1_VIC_CHAIN_START + 0)
#define IRQ_C1_N1_UART                    (IRQ_C1_VIC_CHAIN_START + 1)
#define IRQ_C1_N1_MAILBOX                 (IRQ_C1_VIC_CHAIN_START + 2)
#define IRQ_C1_N1_PORT80                  (IRQ_C1_VIC_CHAIN_START + 3)
#define IRQ_C1_N1_RESET                   (IRQ_C1_VIC_CHAIN_START + 4)
#define IRQ_C1_N2_KCS                     (IRQ_C1_VIC_CHAIN_START + 5)
#define IRQ_C1_N2_UART                    (IRQ_C1_VIC_CHAIN_START + 6)
#define IRQ_C1_N2_MAILBOX                 (IRQ_C1_VIC_CHAIN_START + 7)
#define IRQ_C1_N2_PORT80                  (IRQ_C1_VIC_CHAIN_START + 8)
#define IRQ_C1_N2_RESET                   (IRQ_C1_VIC_CHAIN_START + 9)
#define IRQ_C1_N3_KCS                     (IRQ_C1_VIC_CHAIN_START + 10)
#define IRQ_C1_N3_UART                    (IRQ_C1_VIC_CHAIN_START + 11)
#define IRQ_C1_N3_MAILBOX                 (IRQ_C1_VIC_CHAIN_START + 12)
#define IRQ_C1_N3_PORT80                  (IRQ_C1_VIC_CHAIN_START + 13)
#define IRQ_C1_N3_RESET                   (IRQ_C1_VIC_CHAIN_START + 14)
#define IRQ_C1_N4_KCS                     (IRQ_C1_VIC_CHAIN_START + 15)
#define IRQ_C1_N4_UART                    (IRQ_C1_VIC_CHAIN_START + 16)
#define IRQ_C1_N4_MAILBOX                 (IRQ_C1_VIC_CHAIN_START + 17)
#define IRQ_C1_N4_PORT80                  (IRQ_C1_VIC_CHAIN_START + 18)
#define IRQ_C1_N4_RESET                   (IRQ_C1_VIC_CHAIN_START + 19)
#define IRQ_C1_N1_UART_DMA                (IRQ_C1_VIC_CHAIN_START + 20)
#define IRQ_C1_N2_UART_DMA                (IRQ_C1_VIC_CHAIN_START + 21)
#define IRQ_C1_N3_UART_DMA                (IRQ_C1_VIC_CHAIN_START + 22)
#define IRQ_C1_N4_UART_DMA                (IRQ_C1_VIC_CHAIN_START + 23)
#define IRQ_C1_I2C						(IRQ_C1_VIC_CHAIN_START + 24)

#define IRQ_C2_N1_KCS                     (IRQ_C2_VIC_CHAIN_START + 0)
#define IRQ_C2_N1_UART                    (IRQ_C2_VIC_CHAIN_START + 1)
#define IRQ_C2_N1_MAILBOX                 (IRQ_C2_VIC_CHAIN_START + 2)
#define IRQ_C2_N1_PORT80                  (IRQ_C2_VIC_CHAIN_START + 3)
#define IRQ_C2_N1_RESET                   (IRQ_C2_VIC_CHAIN_START + 4)
#define IRQ_C2_N2_KCS                     (IRQ_C2_VIC_CHAIN_START + 5)
#define IRQ_C2_N2_UART                    (IRQ_C2_VIC_CHAIN_START + 6)
#define IRQ_C2_N2_MAILBOX                 (IRQ_C2_VIC_CHAIN_START + 7)
#define IRQ_C2_N2_PORT80                  (IRQ_C2_VIC_CHAIN_START + 8)
#define IRQ_C2_N2_RESET                   (IRQ_C2_VIC_CHAIN_START + 9)
#define IRQ_C2_N3_KCS                     (IRQ_C2_VIC_CHAIN_START + 10)
#define IRQ_C2_N3_UART                    (IRQ_C2_VIC_CHAIN_START + 11)
#define IRQ_C2_N3_MAILBOX                 (IRQ_C2_VIC_CHAIN_START + 12)
#define IRQ_C2_N3_PORT80                  (IRQ_C2_VIC_CHAIN_START + 13)
#define IRQ_C2_N3_RESET                   (IRQ_C2_VIC_CHAIN_START + 14)
#define IRQ_C2_N4_KCS                     (IRQ_C2_VIC_CHAIN_START + 15)
#define IRQ_C2_N4_UART                    (IRQ_C2_VIC_CHAIN_START + 16)
#define IRQ_C2_N4_MAILBOX                 (IRQ_C2_VIC_CHAIN_START + 17)
#define IRQ_C2_N4_PORT80                  (IRQ_C2_VIC_CHAIN_START + 18)
#define IRQ_C2_N4_RESET                   (IRQ_C2_VIC_CHAIN_START + 19)
#define IRQ_C2_N1_UART_DMA                (IRQ_C2_VIC_CHAIN_START + 20)
#define IRQ_C2_N2_UART_DMA                (IRQ_C2_VIC_CHAIN_START + 21)
#define IRQ_C2_N3_UART_DMA                (IRQ_C2_VIC_CHAIN_START + 22)
#define IRQ_C2_N4_UART_DMA                (IRQ_C2_VIC_CHAIN_START + 23)
#define IRQ_C2_I2C						(IRQ_C2_VIC_CHAIN_START + 24)

#define IRQ_C3_N1_KCS                     (IRQ_C3_VIC_CHAIN_START + 0)
#define IRQ_C3_N1_UART                    (IRQ_C3_VIC_CHAIN_START + 1)
#define IRQ_C3_N1_MAILBOX                 (IRQ_C3_VIC_CHAIN_START + 2)
#define IRQ_C3_N1_PORT80                  (IRQ_C3_VIC_CHAIN_START + 3)
#define IRQ_C3_N1_RESET                   (IRQ_C3_VIC_CHAIN_START + 4)
#define IRQ_C3_N2_KCS                     (IRQ_C3_VIC_CHAIN_START + 5)
#define IRQ_C3_N2_UART                    (IRQ_C3_VIC_CHAIN_START + 6)
#define IRQ_C3_N2_MAILBOX                 (IRQ_C3_VIC_CHAIN_START + 7)
#define IRQ_C3_N2_PORT80                  (IRQ_C3_VIC_CHAIN_START + 8)
#define IRQ_C3_N2_RESET                   (IRQ_C3_VIC_CHAIN_START + 9)
#define IRQ_C3_N3_KCS                     (IRQ_C3_VIC_CHAIN_START + 10)
#define IRQ_C3_N3_UART                    (IRQ_C3_VIC_CHAIN_START + 11)
#define IRQ_C3_N3_MAILBOX                 (IRQ_C3_VIC_CHAIN_START + 12)
#define IRQ_C3_N3_PORT80                  (IRQ_C3_VIC_CHAIN_START + 13)
#define IRQ_C3_N3_RESET                   (IRQ_C3_VIC_CHAIN_START + 14)
#define IRQ_C3_N4_KCS                     (IRQ_C3_VIC_CHAIN_START + 15)
#define IRQ_C3_N4_UART                    (IRQ_C3_VIC_CHAIN_START + 16)
#define IRQ_C3_N4_MAILBOX                 (IRQ_C3_VIC_CHAIN_START + 17)
#define IRQ_C3_N4_PORT80                  (IRQ_C3_VIC_CHAIN_START + 18)
#define IRQ_C3_N4_RESET                   (IRQ_C3_VIC_CHAIN_START + 19)
#define IRQ_C3_N1_UART_DMA                (IRQ_C3_VIC_CHAIN_START + 20)
#define IRQ_C3_N2_UART_DMA                (IRQ_C3_VIC_CHAIN_START + 21)
#define IRQ_C3_N3_UART_DMA                (IRQ_C3_VIC_CHAIN_START + 22)
#define IRQ_C3_N4_UART_DMA                (IRQ_C3_VIC_CHAIN_START + 23)
#define IRQ_C3_I2C						(IRQ_C3_VIC_CHAIN_START + 24)

#endif
