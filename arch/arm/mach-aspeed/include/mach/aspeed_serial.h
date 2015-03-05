/*
 *  file : aspeed_serial.h
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef ASM_ARM_HARDWARE_AST_SERIAL_H
#define ASM_ARM_HARDWARE_AST_SERIAL_H

#define UART_RBR            0x00    /* Receiver Buffer Register */
#define UART_THR            0x00    /* Transmit Holding Register */
#define UART_DLL            0x00    /* Divisor Latch Low Register */
#define UART_DLH            0x04    /* Divisor Latch High Register */
#define UART_IER            0x04    /* Interrupt Enable Register */
#define UART_IIR            0x08    /* Interrupt Identity Register */
#define UART_FCR            0x08    /* FIFO Control Register */
#define UART_LCR            0x0C    /* Line Control Register */
#define UART_MCR            0x10    /* Modem Control Register */
#define UART_LSR            0x14    /* Line Status Register */
#define UART_MSR            0x18    /* Modem Status Register */
#define UART_SCR            0x1C    /* Scratch Register */

/* Interrupt Enable Register */
#define UART_IER_EMSI       0x08    /* Enable Modem Status Interrupt */
#define UART_IER_ELSI       0x04    /* Enable Line Status Interrupt */
#define UART_IER_ETEI       0x02    /* Enable Transmit Holding Empty Interrupt */
#define UART_IER_ERDI       0X01    /* Enable Received Data Interrupt */

/* FIFO Control Register */
#define UART_FCR_XMITR      0x04    /* XMIT FIFO Reset */
#define UART_FCR_RCVRR      0x02    /* RCVR FIFO Reset */
#define UART_FCR_FIFOE      0x01    /* FIEO Enable */

/* Line Control Register */
#define UART_LCR_DLAB       0x80    /* Divisor Latch Address Bit */
#define UART_LCR_BRK        0x40    /* Break Control */
#define UART_LCR_EPS        0x10    /* Even Parity Select */
#define UART_LCR_PEN        0x08    /* Parity Enable */
#define UART_LCR_STOP       0x04    /* Stop Bit */
#define UART_LCR_WLEN_MASK  0x03    /* bits per character mask */
#define UART_LCR_WLEN_8     0x03    /* 8 bits per character */
#define UART_LCR_WLEN_7     0x02    /* 7 bits per character */
#define UART_LCR_WLEN_6     0x01    /* 6 bits per character */
#define UART_LCR_WLEN_5     0x00    /* 5 bits per character */

/* Line Status Register */
#define UART_LSR_TEMT       0x40    /* Transmitter Empty */
#define UART_LSR_THRE       0x20    /* Transmitter Holding Register Empty */
#define UART_LSR_BE         0x10    /* Break Error */
#define UART_LSR_FE         0x08    /* Framing Error */
#define UART_LSR_PE         0x04    /* Parity Error */
#define UART_LSR_OE         0x02    /* Overrun Error */
#define UART_LSR_DR         0x01    /* Data Ready */
#define UART_LSR_ANY        (UART_LSR_BE|UART_LSR_FE|UART_LSR_PE|UART_LSR_OE)

#endif
