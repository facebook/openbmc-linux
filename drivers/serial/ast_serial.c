/*
 *  linux/drivers/char/mvp2_serial.c
 *  modified from linux/drivers/serial/amba.c
 *
 *  Driver for GUC-MVP2000 serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
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
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>

#include <asm/io.h>
#include <asm/irq.h>

#if defined(CONFIG_SERIAL_ASPEED_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/astsim.h>
#include <asm/ast_serial.h>
#define UART_NR		1

#elif defined(CONFIG_ARM)
#include <mach/hardware.h>
#include <mach/aspeed_serial.h>
#define UART_NR		2

#else
#err "NO CONFIG CPU for Serial UART"
#endif




#define SERIAL_MVP2000_CONSLE_NAME	"ttyS"
#define SERIAL_MVP2000_TTY_NAME		"ttyS"
#define SERIAL_MVP2000_DEVFS_NAME	"tts/"
#define SERIAL_MVP2000_MAJOR	4
#define SERIAL_MVP2000_MINOR	64
#define SERIAL_MVP2000_NR	UART_NR

#define CALLOUT_MVP2000_NAME	"cuaam"
#define CALLOUT_MVP2000_MAJOR	4
#define CALLOUT_MVP2000_MINOR	65
#define CALLOUT_MVP2000_NR	UART_NR


#ifdef SUPPORT_SYSRQ
static struct console mvp2000_console;
#endif

#define MVP2000_ISR_PASS_LIMIT	256


/*
 * Access macros for the UARTs
 */
#define UART_GET_CHAR(p)	readl((p)->membase + UART_RBR)
#define UART_PUT_CHAR(p, v)	writel((v), (p)->membase + UART_THR)
#define UART_GET_DLL(p)		readl((p)->membase + UART_DLL)
#define UART_PUT_DLL(p, v)	writel((v), (p)->membase + UART_DLL)
#define UART_GET_DLH(p)		readl((p)->membase + UART_DLH)
#define UART_PUT_DLH(p, v)	writel((v), (p)->membase + UART_DLH)
#define UART_GET_IER(p)		readl((p)->membase + UART_IER)
#define UART_PUT_IER(p, v)	writel((v), (p)->membase + UART_IER)
#define UART_GET_IIR(p)		readl((p)->membase + UART_IIR)
#define UART_GET_FCR(p)		readl((p)->membase + UART_FCR)
#define UART_PUT_FCR(p, v)	writel((v), (p)->membase + UART_FCR)
#define UART_GET_LCR(p)		readl((p)->membase + UART_LCR)
#define UART_PUT_LCR(p, v)	writel((v), (p)->membase + UART_LCR)
#define UART_GET_LSR(p)		readl((p)->membase + UART_LSR)

#define UART_DUMMY_RSR_RX	256
#define UART_PORT_SIZE		64

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_mvp2000_port {
	struct uart_port	port;
};

static void mvp2000_uart_stop_tx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IER(port);
	cr &= ~UART_IER_ETEI;
	UART_PUT_IER(port, cr);
}

static void mvp2000_uart_start_tx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IER(port);
	cr |= UART_IER_ETEI;
	UART_PUT_IER(port, cr);
}

static void mvp2000_uart_stop_rx(struct uart_port *port)
{
	unsigned int cr;

	cr = UART_GET_IER(port);
	cr &= ~UART_IER_ERDI;
	UART_PUT_IER(port, cr);
}

static void mvp2000_uart_enable_ms(struct uart_port *port)
{
	/* printk(KERN_WARNING "ASPEED UART DO NOT Support MODEM operations(emable_ms)\n"); */
}

static void
mvp2000_uart_rx_chars(struct uart_port *port)
{
	struct tty_struct *tty = port->info->port.tty;
	unsigned int status, ch, flag, lsr, max_count = 256;

	status = UART_GET_LSR(port);;
	while ((status & UART_LSR_DR) && max_count--) {
		
		ch = UART_GET_CHAR(port);
		flag = TTY_NORMAL;
		port->icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		lsr = UART_GET_LSR(port); 
		if (unlikely(lsr & UART_LSR_ANY)) {
			if (lsr & UART_LSR_BE) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
				if (uart_handle_break(port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				port->icount.parity++;
			else if (lsr & UART_LSR_FE)
				port->icount.frame++;
			if (lsr & UART_LSR_OE)
				port->icount.overrun++;

			lsr &= port->read_status_mask;

			if (lsr & UART_LSR_BE)
				flag = TTY_BREAK;
			else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch & 255))
			goto ignore_char;

		uart_insert_char(port, lsr, UART_LSR_OE, ch, flag);

	ignore_char:
		status = UART_GET_LSR(port);
	}
	tty_flip_buffer_push(tty);
	return;
}

static void mvp2000_uart_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	int count;

	if (port->x_char) {
		UART_PUT_CHAR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		mvp2000_uart_stop_tx(port);
		return;
	}

	count = port->fifosize >> 1;
	do {
		UART_PUT_CHAR(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		mvp2000_uart_stop_tx(port);
}

static irqreturn_t mvp2000_uart_int(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status, iir, pass_counter = MVP2000_ISR_PASS_LIMIT;

	spin_lock(&port->lock);
		
	status = UART_GET_LSR(port);
	do {
		if (status & UART_LSR_DR)
			mvp2000_uart_rx_chars(port);

		if (status & UART_LSR_THRE) {
			mvp2000_uart_tx_chars(port);
                	iir = UART_GET_IIR(port);
	        }

		if (pass_counter-- == 0)
			break;

		status = UART_GET_LSR(port);
	} while (status & (UART_LSR_THRE|UART_LSR_DR));
	
	spin_unlock(&port->lock);
	
	return IRQ_HANDLED;
}

static unsigned int mvp2000_uart_tx_empty(struct uart_port *port)
{
	return UART_GET_LSR(port) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int mvp2000_uart_get_mctrl(struct uart_port *port)
{
	/* printk(KERN_WARNING "ASPEED UART DO NOT Support MODEM operations(get_mctrl)\n"); */
	return 0;
}

static void mvp2000_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* printk(KERN_WARNING "ASPEED UART DO NOT Support MODEM operations(set_mctrl)\n"); */
}

static void mvp2000_uart_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int lcr;

	spin_lock_irqsave(&port->lock, flags);
	lcr = UART_GET_LCR(port);
	if (break_state == -1)
		lcr |= UART_LCR_BRK;
	else
		lcr &= ~UART_LCR_BRK;
	UART_PUT_LCR(port, lcr);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int mvp2000_uart_startup(struct uart_port *port)
{
	int retval;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, mvp2000_uart_int, IRQF_DISABLED, "mvp2000_serial", port);
	if (retval)
	{
		printk("mvp2000_uart_startup: Can't Get IRQ\n");
		return retval;
	}

	/*
	 * Finally, enable interrupts
	 */
//	IRQ_SET_HIGH_LEVEL(port->irq);
//	IRQ_SET_LEVEL_TRIGGER(port->irq);
	UART_PUT_IER(port, UART_IER_ERDI);

	return 0;
}

static void mvp2000_uart_shutdown(struct uart_port *port)
{
	/*
	 * Free the interrupt
	 */
	free_irq(port->irq, port);

	/*
	 * disable all interrupts, disable the port
	 */
	UART_PUT_IER(port, 0);

	/* disable break condition and fifos */
	UART_PUT_LCR(port, UART_GET_LCR(port)&(~UART_LCR_BRK));
	UART_PUT_FCR(port, UART_GET_FCR(port)&(~UART_FCR_FIFOE));
}

static void
mvp2000_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	unsigned int lcr, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	int ch, i;
 
	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk / (16 * baud);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = UART_LCR_WLEN_5;
		break;
	case CS6:
		lcr = UART_LCR_WLEN_6;
		break;
	case CS7:
		lcr = UART_LCR_WLEN_7;
		break;
	default: // CS8
		lcr = UART_LCR_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB) {
		lcr |= UART_LCR_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr |= UART_LCR_EPS;
	}
	if (port->fifosize > 1)
		fcr |= (UART_FCR_XMITR|UART_FCR_RCVRR|UART_FCR_FIFOE);


	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_RSR_RX;


	/* Set baud rate */
	UART_PUT_LCR(port, UART_LCR_DLAB);	/* enable Divisor Latach Address Bit */
	UART_PUT_DLH(port, ((quot >> 8) & 0xFF));
	UART_PUT_DLL(port, (quot & 0xff));

	UART_PUT_FCR(port, fcr);
	UART_PUT_LCR(port, lcr);
	for (i = 0; i < 16; i++) {
		ch = UART_GET_CHAR (port); /* Clear Timeout Interrupt */
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *mvp2000_uart_type(struct uart_port *port)
{
	return port->type == PORT_GUCMVP2000 ? "GUC-MVP2000 UART" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void mvp2000_uart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int mvp2000_uart_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, UART_PORT_SIZE, "serial_gucmvp2000")
			!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void mvp2000_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_GUCMVP2000;
		mvp2000_uart_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int mvp2000_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_GUCMVP2000)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops mvp2000_pops = {
	.tx_empty	= mvp2000_uart_tx_empty,
	.set_mctrl	= mvp2000_uart_set_mctrl,
	.get_mctrl	= mvp2000_uart_get_mctrl,
	.stop_tx	= mvp2000_uart_stop_tx,
	.start_tx	= mvp2000_uart_start_tx,
	.stop_rx	= mvp2000_uart_stop_rx,
	.enable_ms	= mvp2000_uart_enable_ms,
	.break_ctl	= mvp2000_uart_break_ctl,
	.startup	= mvp2000_uart_startup,
	.shutdown	= mvp2000_uart_shutdown,
	.set_termios	= mvp2000_set_termios,
	.type		= mvp2000_uart_type,
	.release_port	= mvp2000_uart_release_port,
	.request_port	= mvp2000_uart_request_port,
	.config_port	= mvp2000_uart_config_port,
	.verify_port	= mvp2000_uart_verify_port,
};

#if defined(CONFIG_COLDFIRE)
static struct uart_mvp2000_port mvp2000_ports[UART_NR] = {
	{
		.port	= {
			.membase	= (void *) AST_UART0_BASE,
			.mapbase	= AST_UART0_BASE,
			.iotype		= SERIAL_IO_MEM,
			.irq		= IRQ_UART0,
			.uartclk	= (24*1000000L),
			.fifosize	= 16,
			.ops		= &mvp2000_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
	}
};

#elif defined (CONFIG_ARM)
static struct uart_mvp2000_port mvp2000_ports[UART_NR] = {
	{
		.port	= {
			.membase	= (void *) (IO_ADDRESS(AST_UART0_BASE)),
			.mapbase	= AST_UART0_BASE,
			.iotype		= SERIAL_IO_MEM,
			.irq		= IRQ_UART0,
			.uartclk	= (24*1000000L),
			.fifosize	= 16,
			.ops		= &mvp2000_pops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.line		= 0,
		},
	}
};
#else
#err "ERROR~~~"
#endif

#ifdef CONFIG_SERIAL_ASPEED_CONSOLE

static void aspeed_console_putchar(struct uart_port *port, int ch)
{
	while (!(UART_GET_LSR(port) & UART_LSR_THRE))
		barrier();
	UART_PUT_CHAR(port, ch);
}

static void mvp2000_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &mvp2000_ports[co->index].port;
	unsigned int status, old_ier;

	/*
	 *	First save the IER then disable the interrupts
	 */
	old_ier = UART_GET_IER(port);
	UART_PUT_IER(port, 0);

	/*
	 *	Now, do each character
	 */
	uart_console_write(port, s, count, aspeed_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	do {
		status = UART_GET_LSR(port);
	} while (!(status & UART_LSR_TEMT));
	UART_PUT_IER(port, old_ier);
}

static void __init
mvp2000_uart_console_get_options(struct uart_port *port, int *baud, int *parity, int *bits)
{
	if (UART_GET_IER(port) & UART_IER_ERDI) {
		unsigned int lcr, quot;
		lcr = UART_GET_LCR(port);

		*parity = 'n';
		if (lcr & UART_LCR_PEN) {
			if (lcr & UART_LCR_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		switch (lcr & UART_LCR_WLEN_MASK) {
		case UART_LCR_WLEN_8:
		default:
		    *bits = 8;
		    break;
		case UART_LCR_WLEN_7:
		    *bits = 7;
		    break;
		case UART_LCR_WLEN_6:
		    *bits = 6;
		    break;
		case UART_LCR_WLEN_5:
		    *bits = 5;
		    break;
		}

		UART_PUT_LCR(port, UART_LCR_DLAB);	/* enable Divisor Latach Address Bit */
		quot = UART_GET_DLL(port) | (UART_GET_DLH(port) << 8);
		*baud = port->uartclk / (16 * quot);
		UART_PUT_LCR(port, lcr);
	}
}

static int __init mvp2000_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = CONFIG_SERIAL_ASPEED_CONSOLE_BAUD;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &mvp2000_ports[co->index].port;


	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else	
	        mvp2000_uart_console_get_options(port, &baud, &parity, &bits);


	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver mvp2000_reg;

static struct console mvp2000_console = {
	.name		= SERIAL_MVP2000_CONSLE_NAME,
	.write		= mvp2000_uart_console_write,
	.device		= uart_console_device,
	.setup		= mvp2000_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &mvp2000_reg,
};

int __init mvp2000_uart_console_init(void)
{
	register_console(&mvp2000_console);
	return 0;
}

console_initcall(mvp2000_uart_console_init);

#define MVP2000_CONSOLE	&mvp2000_console
#else
#define MVP2000_CONSOLE	NULL
#endif


static struct uart_driver mvp2000_reg = {
	.owner			= THIS_MODULE,	
	.major			= SERIAL_MVP2000_MAJOR,
	.minor			= SERIAL_MVP2000_MINOR,	
	.dev_name		= SERIAL_MVP2000_TTY_NAME,
	.nr			= UART_NR,
	.cons			= MVP2000_CONSOLE,
};

static int __init mvp2000_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&mvp2000_reg);
	if (ret == 0) {
		int i;

		for (i = 0; i < UART_NR; i++)
			uart_add_one_port(&mvp2000_reg, &mvp2000_ports[i].port);
	}
	return ret;
}

static void __exit mvp2000_uart_exit(void)
{
	int i;

	for (i = 0; i < UART_NR; i++)
		uart_remove_one_port(&mvp2000_reg, &mvp2000_ports[i].port);

	uart_unregister_driver(&mvp2000_reg);
}

module_init(mvp2000_uart_init);
module_exit(mvp2000_uart_exit);

MODULE_AUTHOR("ASPEED Technology Inc.");
MODULE_DESCRIPTION("ASPEED serial port driver");
MODULE_LICENSE("GPL");
