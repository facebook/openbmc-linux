/*
 *  linux/drivers/serial/ast1070_dma_uart.c
 *
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 *
 *   History      : 
 *    1. 2012/08/15 Ryan Chen Create
 *  
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/irq.h>

#include "8250.h"
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <plat/regs-uart-dma.h>
#include <mach/ast-uart-dma.h>

//#define CONFIG_UART_DMA_DEBUG

#ifdef CONFIG_UART_DMA_DEBUG
	#define DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

static unsigned int nr_uarts = CONFIG_AST_RUNTIME_DMA_UARTS;

/*
 * Debugging.
 */
#if 0
#define DEBUG_AUTOCONF(fmt...)	printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	256

#include <asm/serial.h>


#define UART_DMA_NR		CONFIG_AST_NR_DMA_UARTS

struct ast_uart_port {
	struct uart_port	port;
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	struct circ_buf		rx_dma_buf;
	struct circ_buf		tx_dma_buf;
	dma_addr_t			dma_rx_addr;	/* Mapped ADMA descr. table */
	dma_addr_t			dma_tx_addr;	/* Mapped ADMA descr. table */
	unsigned int 	dma_buf_size;	//total allocation dma size ..
	struct tasklet_struct	rx_tasklet;
	int rx_tasklet_done;
	struct tasklet_struct	tx_tasklet;
	spinlock_t lock;
	int tx_done;
	int tx_count;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

static struct ast_uart_port ast_uart_ports[UART_DMA_NR];

static inline struct ast_uart_port *
to_ast_dma_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct ast_uart_port, port);
}

struct irq_info {
	spinlock_t		lock;
	struct ast_uart_port *up;
};

static struct irq_info ast_uart_irq[1];
static DEFINE_MUTEX(ast_uart_mutex);

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct serial8250_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_8250] = {
		.name		= "8250",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.name		= "16450",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.name		= "16550",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550A] = {
		.name		= "16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 | UART_FCR_DMA_SELECT,
		.flags		= UART_CAP_FIFO,
	},
};

/* sane hardware needs no mapping */
#define map_8250_in_reg(up, offset) (offset)
#define map_8250_out_reg(up, offset) (offset)

void ast_uart_unregister_port(int line);
int ast_uart_register_port(struct uart_port *port);

static unsigned int serial_in(struct ast_uart_port *up, int offset)
{
	offset = map_8250_in_reg(up, offset) << up->port.regshift;

		return readb(up->port.membase + offset);
}

static void
serial_out(struct ast_uart_port *up, int offset, int value)
{
	/* Save the offset before it's remapped */
	offset = map_8250_out_reg(up, offset) << up->port.regshift;

		writeb(value, up->port.membase + offset);
}


/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)

/* Uart divisor latch read */
static inline int _serial_dl_read(struct ast_uart_port *up)
{
	return serial_inp(up, UART_DLL) | serial_inp(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static inline void _serial_dl_write(struct ast_uart_port *up, int value)
{
	serial_outp(up, UART_DLL, value & 0xff);
	serial_outp(up, UART_DLM, value >> 8 & 0xff);
}

#define serial_dl_read(up) _serial_dl_read(up)
#define serial_dl_write(up, value) _serial_dl_write(up, value)

static void ast_uart_tx_tasklet_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct circ_buf *xmit = &up->port.info->xmit;
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
	
	up->tx_done = 0;
	DBG("line [%d], xmit->head =%d, xmit->tail = %d\n",up->port.line,xmit->head, xmit->tail);

	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		up->tx_count = 0;
		up->tx_done = 1;
		return;
	}

	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	up->tx_count = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (up->tx_count > (UART_XMIT_SIZE - xmit->tail)) {
		up->tx_count = UART_XMIT_SIZE - xmit->tail;
	}

	if (up->tx_count > 4095) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO ....\n");
		up->tx_count = 4095;
	}

	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, 
					uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);

	ast_uart_tx_dma_enqueue(uart_dma_data->chip_no, 
					uart_dma_data->dma_ch, up->dma_tx_addr, up->tx_count);

	dma_sync_single_for_device(up->port.dev,
				   up->dma_tx_addr,
				   up->tx_count,
				   DMA_TO_DEVICE);

	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, 
					uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);


}

static void ast_uart_tx_buffdone(struct ast1070_dma_ch *dma_ch, void *dev_id, u16 len)
{
	struct ast_uart_port *up = (struct ast_uart_port *) dev_id;
	struct circ_buf *xmit = &up->port.info->xmit;

	DBG("line [%d] : tx len = %d \n", up->port.line, len);	
	
	spin_lock(&up->port.lock);
//TODO .....................................len ----> 
	xmit->tail = (xmit->tail + up->tx_count) & (UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->tx_count;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&up->port);

	tasklet_schedule(&up->tx_tasklet);

	spin_unlock(&up->port.lock);
}

static void ast_uart_rx_tasklet_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct tty_struct *tty = up->port.info->port.tty;
	char flag;	
	DBG("line [%d]\n",up->port.line);
	DBG("rx_ring->head = %d, rx_ring->tail = %d , buff addr = %x \n",rx_ring->head, rx_ring->tail, rx_ring->buf);

	spin_lock_irq(&up->lock);
#if 1 	
	DBG("\n rx data :  -- >");

	while (rx_ring->head != rx_ring->tail) {
		DBG("  %x ",rx_ring->buf[rx_ring->tail]);
		flag = TTY_NORMAL;
		uart_insert_char(&up->port, 0, UART_LSR_OE, \
			rx_ring->buf[rx_ring->tail], flag);

//		tty_insert_flip_string

		rx_ring->tail++;
		if (rx_ring->tail == up->dma_buf_size)
			rx_ring->tail = 0;
	}
	DBG("\n");
#else
	
	tty_insert_flip_string(tty, rx_ring->buf + rx_ring->tail, (rx_ring->head - rx_ring->tail));
	rx_ring->tail = rx_ring->head;
#endif
	spin_unlock_irq(&up->lock);

	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);


}

static void ast_uart_rx_buffdone(struct ast1070_dma_ch *dma_ch,
				void *dev_id, u16 len)
{
	struct ast_uart_port *up = (struct ast_uart_port *)dev_id;
//	struct tty_struct *tty = up->port.info->port.tty;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
	u16 remain_size;

	DBG("line [%d]\n",up->port.line);
#if 0
	int i;
	printk("Buff virt addr = %x \n",rx_ring->buf);
	for(i=0;i<len;i++)
		printk("Buff [%x] \n", rx_ring->buf[up->rx_dma_buf.head + i]);
#endif
	DBG("head = %d, len : %d\n",up->rx_dma_buf.head, len);


	//FOR NEXT ......
	rx_ring->head += len;
	
	if (rx_ring->head == up->dma_buf_size) {
		rx_ring->head = 0;
	}

	remain_size = up->dma_buf_size - rx_ring->head;

	//Trigger Next RX dma
	DBG("trigger next size = %d \n",remain_size);	
	
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, 
						uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);

	if(remain_size > DMA_BUFF_SIZE)
		printk("Please check ---> \n");
	
	if(remain_size != 0)  {
		ast_uart_rx_dma_enqueue(uart_dma_data->chip_no, 
							uart_dma_data->dma_ch, up->dma_rx_addr + up->rx_dma_buf.head, remain_size);
	} 
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, 
						uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);

	tasklet_schedule(&up->rx_tasklet);

}

/*
 * FIFO support.
 */
static inline void serial8250_clear_fifos(struct ast_uart_port *p)
{
	serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
		       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(p, UART_FCR, 0);
}


/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.
 */
static void autoconfig(struct ast_uart_port *up, unsigned int probeflags)
{
	unsigned long flags;

	if (!up->port.iobase && !up->port.mapbase && !up->port.membase)
		return;

	DEBUG_AUTOCONF("ttyDMA%d: autoconf (0x%04x, 0x%p): ",
			up->port.line, up->port.iobase, up->port.membase);

	spin_lock_irqsave(&up->port.lock, flags);

	up->capabilities = 0;
	up->bugs = 0;

	up->port.type = PORT_16550A;
	up->capabilities |= UART_CAP_FIFO;

	up->port.fifosize = uart_config[up->port.type].fifo_size;
	up->capabilities = uart_config[up->port.type].flags;
	up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

	if (up->port.type == PORT_UNKNOWN)
		goto out;

	/*
	 * Reset the UART.
	 */
	serial8250_clear_fifos(up);
	serial_in(up, UART_RX);
	serial_outp(up, UART_IER, 0);

 out:
	spin_unlock_irqrestore(&up->port.lock, flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[up->port.type].name);
}


static inline void __stop_tx(struct ast_uart_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void serial8250_stop_tx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

	__stop_tx(up);

}

static void transmit_chars(struct ast_uart_port *up);

static void serial8250_start_tx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

	DBG("line [%d] -->  \n", port->line);
	if (up->tx_done)
		tasklet_schedule(&up->tx_tasklet);
}

static void serial8250_stop_rx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

	DBG("line [%d] -->	\n", port->line);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serial8250_enable_ms(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void transmit_chars(struct ast_uart_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		serial8250_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

//	printk("uart_circ_chars_pending=%d\n",uart_circ_chars_pending(xmit));

	count = up->tx_loadsz;
	do {
//printk("TX : buf = 0x%x\n", xmit->buf[xmit->tail]);
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct ast_uart_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.info != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.info->delta_msr_wait);
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static inline void
serial8250_handle_port(struct ast_uart_port *up)
{
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	DEBUG_INTR("serial8250_handle_port \n");

	status = serial_inp(up, UART_LSR);

	DEBUG_INTR("status = %x...", status);

	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * This is the serial driver's interrupt routine.
 */
static irqreturn_t ast_uart_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	int pass_counter = 0, handled = 0, end = 0;

	DEBUG_INTR("ast_uart_interrupt(%d)...", irq);
	spin_lock(&i->lock);

	do {
		struct ast_uart_port *up;
		unsigned int iir;

		up = (struct ast_uart_port *)(i->up);

		iir = serial_in(up, UART_IIR);
		DEBUG_INTR("iir %x \n", iir);
		if (!(iir & UART_IIR_NO_INT)) {
			printk("handle port \n");
			serial8250_handle_port(up);
			handled = 1;

		}
		else
			end = 1;

		if (pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "ast-uart-dma: too much work for "
				"irq%d\n", irq);
			break;
		}
	} while (end);

	spin_unlock(&i->lock);

	DEBUG_INTR("end.\n");

	return IRQ_RETVAL(handled);
}

static unsigned int serial8250_tx_empty(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int serial8250_get_mctrl(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	unsigned int status;
	unsigned int ret;

	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial8250_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serial8250_break_ctl(struct uart_port *port, int break_state)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial8250_startup(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	//TX DMA 
	struct circ_buf *xmit = &port->info->xmit;
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	int irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

	DBG("line [%d] \n",port->line);
	
	up->capabilities = uart_config[up->port.type].flags;
	up->mcr = 0;
	
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial8250_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	ast_uart_irq[0].up = up;
	retval = request_irq(up->port.irq, ast_uart_interrupt,
				 irq_flags, "ast-uart-dma", ast_uart_irq);
	if (retval)
		return retval;

	/*
	 * Now, initialize the UART
	 */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl |= TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_outp(up, UART_IER, UART_IER_THRI);
	lsr = serial_in(up, UART_LSR);
	iir = serial_in(up, UART_IIR);
	serial_outp(up, UART_IER, 0);

	if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			printk("ttyDMA%d - enabling bad tx status \n",
				 port->line);
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	//RX DMA
	up->rx_dma_buf.head = 0;
	up->rx_dma_buf.tail = 0;
	up->dma_buf_size = 2048;//DMA_BUFF_SIZE -1;	//4096  is dma size  please check
#if 0	
	up->dma_rx_addr = dma_map_single(port->dev,
					   up->rx_dma_buf.buf,
					   up->dma_buf_size,
					   DMA_FROM_DEVICE);
#else	
	up->rx_dma_buf.buf = (unsigned char *)dma_alloc_coherent(NULL, 
								up->dma_buf_size, &up->dma_rx_addr, GFP_KERNEL);
#endif
	DBG("RX buff vir = %x, phy = %x \n", up->rx_dma_buf.buf, up->dma_rx_addr);

	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);

	ast_uart_rx_dma_enqueue(uart_dma_data->chip_no, uart_dma_data->dma_ch, up->dma_rx_addr, up->dma_buf_size);
	
	up->rx_tasklet_done = 1;
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);

	up->tx_dma_buf.head = 0;
	up->tx_dma_buf.buf = xmit->buf;
	up->dma_tx_addr = dma_map_single(port->dev,
				       up->tx_dma_buf.buf,
				       UART_XMIT_SIZE,
				       DMA_TO_DEVICE);
	up->tx_done = 1;
	up->tx_count = 0;

	return 0;
}

static void serial8250_shutdown(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
	unsigned long flags;
	//int i;
	DBG("line [%d]\n",port->line);	
	/*
	 * Disable interrupts from this port
	 */
#if 0
	for(i=0; i<100; i++) {
		printk("tx_count_table[%d] = %d\n", i, tx_count_table[i]);
	}
#endif
	
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	serial8250_clear_fifos(up);

	(void) serial_in(up, UART_RX);
	
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);

	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	//TODO .... Free ---- dma 	
	DBG("free TX , RX buffer \n");
#if 1	
	dma_unmap_single(port->dev, up->dma_rx_addr,
			 up->dma_buf_size,
			 DMA_FROM_DEVICE);
#else
	dma_free_coherent(port->dev, up->dma_buf_size, 
			up->rx_dma_buf.buf, up->dma_rx_addr);
#endif

	dma_unmap_single(port->dev, up->dma_tx_addr,
			 UART_XMIT_SIZE,
			 DMA_TO_DEVICE);


	free_irq(up->port.irq, ast_uart_irq);

}

static unsigned int serial8250_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = uart_get_divisor(port, baud);

	return quot;
}

static void
serial8250_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = serial8250_get_divisor(port, baud);

	if (up->capabilities & UART_CAP_FIFO && up->port.fifosize > 1) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = uart_config[up->port.type].fcr;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);


	serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */

	serial_dl_write(up, quot);

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */

	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

static void
serial8250_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct ast_uart_port *p = (struct ast_uart_port *)port;

	if (p->pm)
		p->pm(port, state, oldstate);
}

/*
 * Resource handling.
 */
static int serial8250_request_std_resource(struct ast_uart_port *up)
{
	unsigned int size = 8 << up->port.regshift;
	int ret = 0;

		if (!up->port.mapbase)
			return ret;

		if (!request_mem_region(up->port.mapbase, size, "ast-uart-dma")) {
			ret = -EBUSY;
			return ret;
		}

		if (up->port.flags & UPF_IOREMAP) {
			up->port.membase = ioremap_nocache(up->port.mapbase,
									size);
			if (!up->port.membase) {
				release_mem_region(up->port.mapbase, size);
				ret = -ENOMEM;
				return ret;
			}
		}
	return ret;
}

static void serial8250_release_std_resource(struct ast_uart_port *up)
{
	unsigned int size = 8 << up->port.regshift;

		if (!up->port.mapbase)
			return;

		if (up->port.flags & UPF_IOREMAP) {
			iounmap(up->port.membase);
			up->port.membase = NULL;
		}

		release_mem_region(up->port.mapbase, size);
}


static void serial8250_release_port(struct uart_port *port)
{
	struct ast_uart_port *up = (struct ast_uart_port *)port;

	serial8250_release_std_resource(up);
}

static int serial8250_request_port(struct uart_port *port)
{
	struct ast_uart_port *up = (struct ast_uart_port *)port;
	int ret = 0;

	ret = serial8250_request_std_resource(up);
	if (ret == 0 )
			serial8250_release_std_resource(up);

	return ret;
}

static void serial8250_config_port(struct uart_port *port, int flags)
{
	struct ast_uart_port *up = (struct ast_uart_port *)port;
	int probeflags = PROBE_ANY;
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = serial8250_request_std_resource(up);
	if (ret < 0)
		return;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);

	if (up->port.type == PORT_UNKNOWN)
		serial8250_release_std_resource(up);
}

static int
serial8250_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

static const char *
serial8250_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static struct uart_ops serial8250_pops = {
	.tx_empty	= serial8250_tx_empty,
	.set_mctrl	= serial8250_set_mctrl,
	.get_mctrl	= serial8250_get_mctrl,
	.stop_tx	= serial8250_stop_tx,
	.start_tx	= serial8250_start_tx,
	.stop_rx	= serial8250_stop_rx,
	.enable_ms	= serial8250_enable_ms,
	.break_ctl	= serial8250_break_ctl,
	.startup	= serial8250_startup,
	.shutdown	= serial8250_shutdown,
	.set_termios	= serial8250_set_termios,
	.pm		= serial8250_pm,
	.type		= serial8250_type,
	.release_port	= serial8250_release_port,
	.request_port	= serial8250_request_port,
	.config_port	= serial8250_config_port,
	.verify_port	= serial8250_verify_port,
};

static void __init serial8250_isa_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < nr_uarts; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		up->mcr_force = ALPHA_KLUDGE_MCR;

		up->port.ops = &serial8250_pops;
	}

}

static void __init
serial8250_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;
	printk("serial8250_register_ports \n");

	serial8250_isa_init_ports();

	for (i = 0; i < nr_uarts; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];
		up->port.dev = dev;
		uart_add_one_port(drv, &up->port);
	}
}

#define SERIAL8250_CONSOLE	NULL

static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ast-uart-dma",
	.dev_name		= "ttyDMA",
#if 0
	.major			= TTY_MAJOR,
	.minor			= 64,
#else
	.major			= 204, // like atmel_serial
	.minor			= 155,
#endif
	.nr			= UART_DMA_NR,
	.cons			= SERIAL8250_CONSOLE,
};


#if 0
/**
 *	serial8250_suspend_port - suspend one serial port
 *	@line:  serial line number
 *
 *	Suspend one serial port.
 */
void serial8250_suspend_port(int line)
{
	uart_suspend_port(&serial8250_reg, &ast_uart_ports[line].port);
}

/**
 *	serial8250_resume_port - resume one serial port
 *	@line:  serial line number
 *
 *	Resume one serial port.
 */
void serial8250_resume_port(int line)
{
	struct ast_uart_port *up = &ast_uart_ports[line];

	uart_resume_port(&serial8250_reg, &up->port);
}
#endif

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int __devinit serial8250_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev->dev.platform_data;
	struct uart_port port;
	struct ast_uart_dma_data *uart_dma_data;
	int ret, i;

	if(UART_XMIT_SIZE > DMA_BUFF_SIZE)
		printk("UART_XMIT_SIZE > DMA_BUFF_SIZE : Please Check \n");
	
	memset(&port, 0, sizeof(struct uart_port));

	for (i = 0; p && p->flags != 0; p++, i++) {
		port.iobase		= p->iobase;
		port.membase		= p->membase;
		port.irq		= p->irq;
		port.uartclk		= p->uartclk;
		port.regshift		= p->regshift;
		port.iotype		= p->iotype;
		port.flags		= p->flags;
		port.mapbase		= p->mapbase;
		port.hub6		= p->hub6;
		port.private_data	= p->private_data;
		port.dev		= &dev->dev;
		uart_dma_data = p->private_data;
		if (share_irqs)
			port.flags |= UPF_SHARE_IRQ;
		ret = ast_uart_register_port(&port);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%llx IRQ%d): %d\n", i,
				p->iobase, (unsigned long long)p->mapbase,
				p->irq, ret);
		}
//		printk("TODO ...... line = %d \n",i);
		ret = ast_uart_rx_dma_request(uart_dma_data->chip_no, uart_dma_data->dma_ch, ast_uart_rx_buffdone, &ast_uart_ports[i]);
		if (ret < 0) {
			printk("Error : failed to get rx dma channel[%d]\n", uart_dma_data->dma_ch);
			goto out_ast_uart_unregister_port;
		}

		ret = ast_uart_tx_dma_request(uart_dma_data->chip_no, uart_dma_data->dma_ch, ast_uart_tx_buffdone, &ast_uart_ports[i]);
		if (ret < 0) {
			printk("Error : failed to get tx dma channel[%d]\n", uart_dma_data->dma_ch);
			return ret;
		}
	}

	return 0;
	
out_ast_uart_unregister_port:
	for (i = 0; i < nr_uarts; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		if (up->port.dev == &dev->dev)
			ast_uart_unregister_port(i);
	};
	return ret;

}

/*
 * Remove serial ports registered against a platform device.
 */
static int __devexit serial8250_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		if (up->port.dev == &dev->dev)
			ast_uart_unregister_port(i);
	}
	//TODO ..
//	pl080_dma_free(uart_dma_rx.channel, (void *)uart_dma_rx.client);
//	pl080_dma_free(uart_dma_tx.channel, (void *)uart_dma_tx.client);
	
	return 0;
}

static int serial8250_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_DMA_NR; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&serial8250_reg, &up->port);
	}

	return 0;
}

static int serial8250_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_DMA_NR; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			serial8250_resume_port(i);
	}

	return 0;
}

static struct platform_driver serial8250_ast_dma_driver = {
	.probe		= serial8250_probe,
	.remove		= __devexit_p(serial8250_remove),
	.suspend	= serial8250_suspend,
	.resume		= serial8250_resume,
	.driver		= {
		.name	= "ast-uart-dma",
		.owner	= THIS_MODULE,
	},
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
static struct platform_device *serial8250_isa_devs;

/*
 * serial8250_register_port and serial8250_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */

static struct ast_uart_port *serial8250_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (uart_match_port(&ast_uart_ports[i].port, port))
			return &ast_uart_ports[i];

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < nr_uarts; i++)
		if (ast_uart_ports[i].port.type == PORT_UNKNOWN &&
		    ast_uart_ports[i].port.iobase == 0)
			return &ast_uart_ports[i];

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (ast_uart_ports[i].port.type == PORT_UNKNOWN)
			return &ast_uart_ports[i];

	return NULL;
}

/**
 *	serial8250_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int ast_uart_register_port(struct uart_port *port)
{
	struct ast_uart_port *uart;
	int ret = -ENOSPC;

	if (port->uartclk == 0)
		return -EINVAL;
printk("register port line %d\n",port->line);
	mutex_lock(&ast_uart_mutex);

	uart = serial8250_find_match_or_unused(port);
	if (uart) {
		uart_remove_one_port(&serial8250_reg, &uart->port);

		uart->port.iobase       = port->iobase;
		uart->port.membase      = port->membase;
		uart->port.irq          = port->irq;
		uart->port.uartclk      = port->uartclk;
		uart->port.fifosize     = port->fifosize;
		uart->port.regshift     = port->regshift;
		uart->port.iotype       = port->iotype;
		uart->port.flags        = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase      = port->mapbase;
		uart->port.private_data = port->private_data;
		if (port->dev)
			uart->port.dev = port->dev;

		ret = uart_add_one_port(&serial8250_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;

		spin_lock_init(&uart->lock);

		tasklet_init(&uart->rx_tasklet, ast_uart_rx_tasklet_func,
				(unsigned long)uart);
	
		tasklet_init(&uart->tx_tasklet, ast_uart_tx_tasklet_func,
				(unsigned long)uart);

	}

	mutex_unlock(&ast_uart_mutex);

	return ret;
}
EXPORT_SYMBOL(ast_uart_register_port);

/**
 *	serial8250_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void ast_uart_unregister_port(int line)
{
	struct ast_uart_port *uart = &ast_uart_ports[line];

	mutex_lock(&ast_uart_mutex);
	uart_remove_one_port(&serial8250_reg, &uart->port);
	if (serial8250_isa_devs) {
		uart->port.flags &= ~UPF_BOOT_AUTOCONF;
		uart->port.type = PORT_UNKNOWN;
		uart->port.dev = &serial8250_isa_devs->dev;
		uart_add_one_port(&serial8250_reg, &uart->port);
	} else {
		uart->port.dev = NULL;
	}
	mutex_unlock(&ast_uart_mutex);
}
EXPORT_SYMBOL(ast_uart_unregister_port);

static int __init ast_uart_init(void)
{
	int ret;

	if (nr_uarts > UART_DMA_NR)
		nr_uarts = UART_DMA_NR;

	printk(KERN_INFO "ast-uart-dma: UART driver with DMA "
		"%d ports, IRQ sharing %sabled\n", nr_uarts,
		share_irqs ? "en" : "dis");

		spin_lock_init(&ast_uart_irq[0].lock);

	ret = uart_register_driver(&serial8250_reg);
	if (ret)
		goto out;

	serial8250_isa_devs = platform_device_alloc("ast-uart-dma",
						    PLAT8250_DEV_LEGACY);
	if (!serial8250_isa_devs) {
		ret = -ENOMEM;
		goto unreg_uart_drv;
	}

	ret = platform_device_add(serial8250_isa_devs);
	if (ret)
		goto put_dev;

	serial8250_register_ports(&serial8250_reg, &serial8250_isa_devs->dev);

	ret = platform_driver_register(&serial8250_ast_dma_driver);
	if (ret == 0)
		goto out;

	platform_device_del(serial8250_isa_devs);
 put_dev:
	platform_device_put(serial8250_isa_devs);
 unreg_uart_drv:
	uart_unregister_driver(&serial8250_reg);
 out:
	return ret;
}

static void __exit ast_uart_exit(void)
{
	struct platform_device *isa_dev = serial8250_isa_devs;

	/*
	 * This tells serial8250_unregister_port() not to re-register
	 * the ports (thereby making serial8250_ast_dma_driver permanently
	 * in use.)
	 */
	serial8250_isa_devs = NULL;

	platform_driver_unregister(&serial8250_ast_dma_driver);
	platform_device_unregister(isa_dev);

	uart_unregister_driver(&serial8250_reg);
}

late_initcall(ast_uart_init);
module_exit(ast_uart_exit);

#if 0
EXPORT_SYMBOL(serial8250_suspend_port);
EXPORT_SYMBOL(serial8250_resume_port);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AST DMA serial driver");
MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
