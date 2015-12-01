/*
 *  linux/drivers/serial/ast_dma_uart.c
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
#include <mach/ast-uart-dma.h>

//#define CONFIG_UART_DMA_DEBUG
//#define CONFIG_UART_TX_DMA_DEBUG

#ifdef CONFIG_UART_DMA_DEBUG
	#define UART_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
	#define UART_DBG(fmt, args...)
#endif

#ifdef CONFIG_UART_TX_DMA_DEBUG
	#define UART_TX_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
	#define UART_TX_DBG(fmt, args...)
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
#if defined(CONFIG_AST1070_UART_DMA)	
	struct tasklet_struct	rx_tasklet;
#elif defined(CONFIG_AST_UART_SDMA)	
#ifdef SDDMA_RX_FIX
	struct tasklet_struct	rx_tasklet;
#else
	struct timer_list		rx_timer;
#endif
#endif
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

static unsigned int ast_serial_in(struct ast_uart_port *up, int offset)
{
	offset = map_8250_in_reg(up, offset) << up->port.regshift;

		return readb(up->port.membase + offset);
}

static void
ast_serial_out(struct ast_uart_port *up, int offset, int value)
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
#define serial_inp(up, offset)		ast_serial_in(up, offset)
#define serial_outp(up, offset, value)	ast_serial_out(up, offset, value)

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

#if defined(CONFIG_AST1070_UART_DMA)
static void ast_uart_tx_1070_tasklet_func(unsigned long data)
{
	int i;
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct circ_buf *xmit = &up->port.state->xmit;	
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
		
	up->tx_done = 0;
	UART_TX_DBG("line [%d], xmit->head =%d, xmit->tail = %d\n",up->port.line,xmit->head, xmit->tail);

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
#if 0	
	printk("\n");	
	for( i=0; i< up->tx_count;i++)
		printk("%x ",readl(xmit->buf + i));

	printk("\n");
#endif	
	up->tx_count = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (up->tx_count > (UART_XMIT_SIZE - xmit->tail)) {
		up->tx_count = UART_XMIT_SIZE - xmit->tail;
	}

	if (up->tx_count > 4095) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO ....\n");
		up->tx_count = 4095;
	}

	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	ast_uart_tx_dma_enqueue(uart_dma_data->chip_no, uart_dma_data->dma_ch, up->dma_tx_addr, up->tx_count);
	dma_sync_single_for_device(up->port.dev,
				   up->dma_tx_addr,
				   up->tx_count,
				   DMA_TO_DEVICE);
	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);

}

#elif defined(CONFIG_AST_UART_SDMA)
static void ast_uart_tx_sdma_tasklet_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct circ_buf *xmit = &up->port.state->xmit;	
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;
	u32 tx_pt;
	int i =0;
//	printk("line [%d], xmit->head =%d, xmit->tail = %d\n",up->port.line,xmit->head, xmit->tail);

	spin_lock(&up->port.lock);

	up->tx_count = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);
//	ast_uart_tx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_PAUSE);
	dma_sync_single_for_device(up->port.dev,
				   up->dma_tx_addr,
				   UART_XMIT_SIZE,
				   DMA_TO_DEVICE);
        // test
	tx_pt = ast_uart_get_tx_sdma_pt(uart_dma_data->dma_ch);


#if 1
	//(tx_pt & 0xffc) == (xmit->head & 0xffc)
	if(tx_pt > xmit->head) {
		//tx_pt = 1/2/3 
		if((tx_pt & 0xfffc) == 0)
			ast_uart_tx_sdma_update(uart_dma_data->dma_ch, 0xffff);
		else
			ast_uart_tx_sdma_update(uart_dma_data->dma_ch, 0);
	} else
		ast_uart_tx_sdma_update(uart_dma_data->dma_ch, xmit->head);
#else
	ast_uart_tx_sdma_update(uart_dma_data->dma_ch, xmit->head);
#endif
	spin_unlock(&up->port.lock);
}
#endif

static void ast_uart_tx_buffdone(void *dev_id, u16 len)
{
	struct ast_uart_port *up = (struct ast_uart_port *) dev_id;
	struct circ_buf *xmit = &up->port.state->xmit;
	int i =0;
	UART_TX_DBG("line [%d] : tx len = %d \n", up->port.line, len);	

	spin_lock(&up->port.lock);
	//-->get tail for update len 
#if defined(CONFIG_AST1070_UART_DMA)	
	xmit->tail = (xmit->tail + up->tx_count) & (UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->tx_count;
#elif defined(CONFIG_AST_UART_SDMA)	
	xmit->tail = len;
	UART_TX_DBG(" line [%d], xmit->head =%d, xmit->tail = %d\n",up->port.line,xmit->head, xmit->tail);		
#endif

#if 0 
	printk("xmit->head =%x, xmit->tail = %x \n", xmit->head, xmit->tail);	
	for(i =0 ; i < 10; i++)
		printk("%x ", xmit->buf[(xmit->tail -4 + i) & 0xffff]);
	printk("\n");	
#endif	

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if(xmit->head != xmit->tail)
		tasklet_schedule(&up->tx_tasklet);

	spin_unlock(&up->port.lock);
}

#if defined(CONFIG_AST1070_UART_DMA)
static void ast_uart_rx_tasklet_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct tty_port *port = &up->port.state->port;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct tty_struct *tty = up->port.state->port.tty;	
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;	

	char flag;	
	int count;
	UART_DBG("line [%d], rx_ring->head = %d, rx_ring->tail = %d\n",up->port.line,rx_ring->head, rx_ring->tail);

	spin_lock(&up->port.lock);

//	printk("\n rx_ring->head %x  rx data : ",rx_ring->head);
	while (rx_ring->head != rx_ring->tail) {
//		printk("[%d] : %c", rx_ring->tail, rx_ring->buf[rx_ring->tail]);
		uart_insert_char(&up->port, 0, UART_LSR_OE, rx_ring->buf[rx_ring->tail], TTY_NORMAL);
		rx_ring->tail++;
		rx_ring->tail &= (DMA_BUFF_SIZE - 1);
		up->port.icount.rx++;
	}

	tty_flip_buffer_push(port);
//	printk("update tail %x  \n", rx_ring->tail);
	ast_uart_rx_sdma_update(uart_dma_data->dma_ch, rx_ring->tail);
	spin_unlock(&up->port.lock);	

}

static void ast_uart_rx_buffdone(void *dev_id, u16 len)
{
	u16 remain_size;
	struct ast_uart_port *up = (struct ast_uart_port *)dev_id;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;

	spin_lock(&up->port.lock);

	UART_DBG("line [%d],head = %d, len : %d\n",up->port.line,up->rx_dma_buf.head, len);

	//FOR NEXT ......
	rx_ring->head += len;
	if (rx_ring->head == up->dma_buf_size) {
		rx_ring->head = 0;
	}
	remain_size = up->dma_buf_size - rx_ring->head;

	//Trigger Next RX dma
	UART_DBG("trigger next size = %d \n",remain_size);	

	if(remain_size > DMA_BUFF_SIZE)
		printk("Please check ---> \n");

	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, 
						uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);

	
	if(remain_size != 0)  {
		ast_uart_rx_dma_enqueue(uart_dma_data->chip_no, 
							uart_dma_data->dma_ch, up->dma_rx_addr + up->rx_dma_buf.head, remain_size);
	} 
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, 
						uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);
	
	tasklet_schedule(&up->rx_tasklet);
	
	spin_unlock(&up->port.lock);

}

#elif defined(CONFIG_AST_UART_SDMA)
#ifdef SDDMA_RX_FIX
static void ast_uart_rx_sdma_tasklet_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct tty_port *port = &up->port.state->port;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct tty_struct *tty = up->port.state->port.tty;	
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;	

	char flag;	
	int count;
	int copy = 0;	
	UART_DBG("line [%d], rx_ring->head = %d, rx_ring->tail = %d\n",up->port.line,rx_ring->head, rx_ring->tail);

	spin_lock(&up->port.lock);
#if 0
       while (rx_ring->head != rx_ring->tail) {
                UART_DBG("  %x ",rx_ring->buf[rx_ring->tail]);
                flag = TTY_NORMAL;
                uart_insert_char(&up->port, 0, UART_LSR_OE, \
                        rx_ring->buf[rx_ring->tail], flag);
		if(ast_rx_pattern(rx_ring->buf[rx_ring->tail])) {
			printk("ERROR !!!!!!!!!!!!!!!!!!!!!!\n");
			serial_outp(up, UART_TX, 0xff);
//			while(1);
		}
                rx_ring->tail++;
		rx_ring->tail &= (SDMA_RX_BUFF_SIZE - 1);
        }
        UART_DBG("\n");
	tty_flip_buffer_push(tty);
	ast_uart_rx_sdma_update(uart_dma_data->dma_ch, rx_ring->tail);
#else
	if(rx_ring->head > rx_ring->tail) {
		count = rx_ring->head - rx_ring->tail;
//		printk("^^^^ count %d rx_ring->head %x ,  rx_ring->tail %x \n", count, rx_ring->head, rx_ring->tail);		
		copy = tty_insert_flip_string(port, rx_ring->buf + rx_ring->tail, count);
	} else if (rx_ring->head < rx_ring->tail) {
		count = SDMA_RX_BUFF_SIZE - rx_ring->tail;
//		printk("^^^^  ~~~~ count %d rx_ring->head %x ,	rx_ring->tail %x \n", count, rx_ring->head, rx_ring->tail);
		copy = tty_insert_flip_string(port, rx_ring->buf + rx_ring->tail, count);	
	} else {
		count = 0;
//		printk("@@--%s--ch = 0x%x\n", __func__, uart_dma_data->dma_ch);
	}

	if(copy != count) printk("!!!!!!!! ERROR 111\n");

	if(count) {
//		printk("\n count = %d \n", count);	
		rx_ring->tail += count;
		rx_ring->tail &= (SDMA_RX_BUFF_SIZE - 1);
		up->port.icount.rx += count;
		tty_flip_buffer_push(port);
		ast_uart_rx_sdma_update(uart_dma_data->dma_ch, rx_ring->tail);
	}
#endif
	spin_unlock(&up->port.lock);	

}

static void ast_uart_rx_buffdone(void *dev_id, u16 len)
{
	u16 remain_size;
	struct ast_uart_port *up = (struct ast_uart_port *)dev_id;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;

	UART_DBG("line [%d],head = %d, len : %d\n",up->port.line,up->rx_dma_buf.head, len);

	spin_lock(&up->port.lock);
	rx_ring->head = len;
	spin_unlock(&up->port.lock);
	tasklet_schedule(&up->rx_tasklet);
}

#else

static void ast_uart_rx_timer_func(unsigned long data)
{
	struct ast_uart_port *up = to_ast_dma_uart_port((struct uart_port *)data);
	struct tty_port *port = &up->port.state->port;
	struct circ_buf *rx_ring = &up->rx_dma_buf;
	struct tty_struct *tty = up->port.state->port.tty;	
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;	

	char flag;	
	int count = 0;
	int copy = 0;
	UART_DBG("line [%d], rx_ring->head = %d, rx_ring->tail = %d\n",up->port.line,rx_ring->head, rx_ring->tail);

	rx_ring->head = ast_uart_get_rx_sdma_pt(uart_dma_data->dma_ch);

	del_timer(&up->rx_timer);
//	printk("\n");
#if 0
	while (rx_ring->head != rx_ring->tail) {
//		if((count % 16) == 0)
//			printk("[%d] : ", rx_ring->tail);
		
//		printk("%x", rx_ring->buf[rx_ring->tail]);	
		uart_insert_char(&up->port, 0, UART_LSR_OE, rx_ring->buf[rx_ring->tail], TTY_NORMAL);
		rx_ring->tail++;
		rx_ring->tail &= (SDMA_RX_BUFF_SIZE - 1);
		count++;	
//		if((count % 16) == 0)
//			printk("\n");
	}
#else

	if(rx_ring->head > rx_ring->tail) {
		ast_uart_set_sdma_time_out(0xffff);		
		count = rx_ring->head - rx_ring->tail;
//		printk("^^^^ count %d rx_ring->head %x ,  rx_ring->tail %x \n", count, rx_ring->head, rx_ring->tail);		
		copy = tty_insert_flip_string(tty, rx_ring->buf + rx_ring->tail, count);
	} else if (rx_ring->head < rx_ring->tail) {
		ast_uart_set_sdma_time_out(0xffff);	
		count = SDMA_RX_BUFF_SIZE - rx_ring->tail;
//		printk("^^^^  ~~~~ count %d rx_ring->head %x ,  rx_ring->tail %x \n", count, rx_ring->head, rx_ring->tail);
		copy = tty_insert_flip_string(tty, rx_ring->buf + rx_ring->tail, count);	
	} else {
		count = 0;
		//printk("@@--%s--ch = 0x%x\n", __func__, ch);
        }
		
	if(copy != count) printk("!!!!!!!! ERROR 111\n");
	rx_ring->tail += count;
	rx_ring->tail &= (SDMA_RX_BUFF_SIZE - 1);
#endif

	if(count) {
//		printk("\n count = %d \n", count);	
		up->port.icount.rx += count;
		spin_lock(&up->port.lock);
		tty_flip_buffer_push(port);
		spin_unlock(&up->port.lock);	
//		printk("update rx_ring->tail %x \n", rx_ring->tail);
		ast_uart_rx_sdma_update(uart_dma_data->dma_ch, rx_ring->tail);
		uart_dma_data->workaround = 1;
	} else {
		if(uart_dma_data->workaround) { 
			uart_dma_data->workaround++;
			if(uart_dma_data->workaround > 1) 
				ast_uart_set_sdma_time_out(0);
			else
				ast_uart_set_sdma_time_out(0xffff);
		}
	}
	add_timer(&up->rx_timer);
}
#endif
#endif

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
	UART_DBG("line [%d] \n",up->port.line);

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
	ast_serial_in(up, UART_RX);
	serial_outp(up, UART_IER, 0);

 out:
	spin_unlock_irqrestore(&up->port.lock, flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[up->port.type].name);
}


static inline void __stop_tx(struct ast_uart_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		ast_serial_out(p, UART_IER, p->ier);
	}
}

static void serial8250_stop_tx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	UART_TX_DBG("line [%d] \n",up->port.line);

	__stop_tx(up);

}

static void transmit_chars(struct ast_uart_port *up);

static void serial8250_start_tx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

//	UART_TX_DBG("line [%d]  \n", port->line);
	tasklet_schedule(&up->tx_tasklet);
}

static void serial8250_stop_rx(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);

	UART_DBG("line [%d]  \n", port->line);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	ast_serial_out(up, UART_IER, up->ier);
}

static void serial8250_enable_ms(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	UART_DBG("line [%d]  \n", port->line);

	up->ier |= UART_IER_MSI;
	ast_serial_out(up, UART_IER, up->ier);
}

static void transmit_chars(struct ast_uart_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;		
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

	count = up->tx_loadsz;
	do {
		ast_serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct ast_uart_port *up)
{
	unsigned int status = ast_serial_in(up, UART_MSR);
	UART_DBG("line [%d] \n",up->port.line);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
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

	DEBUG_INTR("(%d) ", irq);
	spin_lock(&i->lock);

	do {
		struct ast_uart_port *up;
		unsigned int iir;

		up = (struct ast_uart_port *)(i->up);

		iir = ast_serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
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

	UART_TX_DBG("line [%d] \n",up->port.line);

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = ast_serial_in(up, UART_LSR);
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
//printk("serial8250_set_mctrl %x \n",mctrl);
	//TODO .... Issue for fix ......
	mctrl=0;

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

	ast_serial_out(up, UART_MCR, mcr);
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
	ast_serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial8250_startup(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
	//TX DMA 
	struct circ_buf *xmit = &up->port.state->xmit;
#if defined(CONFIG_AST1070_UART_DMA)
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
#elif defined(CONFIG_AST_UART_SDMA)
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;
#else
	
#endif	
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	int irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

	UART_DBG("line [%d] \n",port->line);
	
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
	lsr = ast_serial_in(up, UART_LSR);
	iir = ast_serial_in(up, UART_IIR);
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
	up->port.icount.rx = 0;

	up->tx_done = 1;
	up->tx_count = 0;

#if defined(CONFIG_AST1070_UART_DMA)
	up->rx_dma_buf.buf = (unsigned char *)dma_alloc_coherent(NULL, 
								up->dma_buf_size, &up->dma_rx_addr, GFP_KERNEL);

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
	
#elif defined(CONFIG_AST_UART_SDMA)
	up->rx_dma_buf.head = 0;
	up->rx_dma_buf.tail = 0;	
#ifdef SDDMA_RX_FIX
#else
	uart_dma_data->workaround = 0;
#endif
	ast_uart_rx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	ast_uart_rx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);
#ifdef SDDMA_RX_FIX
#else
	add_timer(&up->rx_timer);
#endif
	up->tx_dma_buf.head = 0;
	up->tx_dma_buf.tail = 0;	
	up->tx_dma_buf.buf = xmit->buf;

	up->dma_tx_addr = dma_map_single(port->dev,
				       up->tx_dma_buf.buf,
				       UART_XMIT_SIZE,
				       DMA_TO_DEVICE);

	ast_uart_tx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	ast_uart_tx_sdma_enqueue(uart_dma_data->dma_ch, up->dma_tx_addr);
	ast_uart_tx_sdma_update(uart_dma_data->dma_ch, 0);
	ast_uart_tx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_TRIGGER);
#endif	

	return 0;
}

static void serial8250_shutdown(struct uart_port *port)
{
	struct ast_uart_port *up = to_ast_dma_uart_port(port);
#if defined(CONFIG_AST1070_UART_DMA)
	struct ast_uart_dma_data *uart_dma_data = up->port.private_data;
#elif defined(CONFIG_AST_UART_SDMA)
	struct ast_uart_sdma_data *uart_dma_data = up->port.private_data;
#else
		
#endif	
		
	unsigned long flags;
	//int i;
	UART_DBG("line [%d]\n",port->line);	

	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	ast_serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	serial8250_clear_fifos(up);

	(void) ast_serial_in(up, UART_RX);

#if defined(CONFIG_AST1070_UART_DMA)
	ast_uart_rx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	ast_uart_tx_dma_ctrl(uart_dma_data->chip_no, uart_dma_data->dma_ch, AST_UART_DMAOP_STOP);
	dma_free_coherent(port->dev, up->dma_buf_size, 
			up->rx_dma_buf.buf, up->dma_rx_addr);	
#elif defined(CONFIG_AST_UART_SDMA)
	ast_uart_rx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_PAUSE);
	ast_uart_tx_sdma_ctrl(uart_dma_data->dma_ch, AST_UART_DMAOP_PAUSE);
#ifdef SDDMA_RX_FIX
#else
	del_timer_sync(&up->rx_timer);
#endif
#endif

	//Tx buffer will free by serial_core.c 
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

	ast_serial_out(up, UART_IER, up->ier);


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

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int __init serial8250_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev->dev.platform_data;
	struct uart_port port;
#if defined(CONFIG_AST1070_UART_DMA)
	struct ast_uart_dma_data *uart_dma_data;
#elif defined(CONFIG_AST_UART_SDMA)
	struct ast_uart_sdma_data *uart_dma_data;
#else
		
#endif	
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

#if defined(CONFIG_AST1070_UART_DMA)
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
#elif defined(CONFIG_AST_UART_SDMA)
#ifdef SDDMA_RX_FIX
		ast_uart_ports[i].rx_dma_buf.buf = ast_uart_rx_sdma_request(uart_dma_data->dma_ch, ast_uart_rx_buffdone, &ast_uart_ports[i]);
		if (ast_uart_ports[i].rx_dma_buf.buf < 0) {
			printk("Error : failed to get rx dma channel[%d]\n", uart_dma_data->dma_ch);
			goto out_ast_uart_unregister_port;
		}

#else
		ast_uart_ports[i].rx_dma_buf.buf = ast_uart_rx_sdma_request(uart_dma_data->dma_ch, &ast_uart_ports[i]);
		if (ast_uart_ports[i].rx_dma_buf.buf < 0) {
			printk("Error : failed to get rx dma channel[%d]\n", uart_dma_data->dma_ch);
			goto out_ast_uart_unregister_port;
		}
#endif
		ret = ast_uart_tx_sdma_request(uart_dma_data->dma_ch, ast_uart_tx_buffdone, &ast_uart_ports[i]);
		if (ret < 0) {
			printk("Error : failed to get tx dma channel[%d]\n", uart_dma_data->dma_ch);
			return ret;
		}
#else 

#endif
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
static int __exit serial8250_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct ast_uart_port *up = &ast_uart_ports[i];

		if (up->port.dev == &dev->dev)
			ast_uart_unregister_port(i);
	}
	
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
	.remove		= serial8250_remove,
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


#if defined(CONFIG_AST1070_UART_DMA)
		tasklet_init(&uart->rx_tasklet, ast_uart_rx_tasklet_func,
				(unsigned long)uart);

		tasklet_init(&uart->tx_tasklet, ast_uart_tx_1070_tasklet_func,
				(unsigned long)uart);
#elif defined(CONFIG_AST_UART_SDMA)
		tasklet_init(&uart->tx_tasklet, ast_uart_tx_sdma_tasklet_func,
				(unsigned long)uart);
#ifdef SDDMA_RX_FIX
		tasklet_init(&uart->rx_tasklet, ast_uart_rx_sdma_tasklet_func,
				(unsigned long)uart);
#else
		uart->rx_timer.data = (unsigned long)uart;
		uart->rx_timer.expires = jiffies + (HZ);
		uart->rx_timer.function = ast_uart_rx_timer_func;
		init_timer(&uart->rx_timer);
#endif

#endif
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
