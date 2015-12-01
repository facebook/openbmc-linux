/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-uart.c
* Author        : Ryan chen
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/09/15 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_8250.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <asm/io.h>

#include <plat/ast-scu.h>
#include <plat/devs.h>

#include <plat/regs-vuart.h>

typedef enum {
	SIRQ0 = 0,
    	SIRQ1,
    	SIRQ2,
    	SIRQ3,
	SIRQ4,
	SIRQ5,
	SIRQ6,
	SIRQ7,
	SIRQ8,
	SIRQ9,
} SIO_serial_irq;

#define AST_COM_PORT		PORT_3F8
#define AST_SIRQ			SIRQ4

#define PORT_2F8		0x2f8
#define PORT_3F8		0x3f8

/* --------------------------------------------------------------------
 * UART
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_SERIAL_AST_VUART)
static struct plat_serial8250_port ast_vuart_data[] = {
	{
		.mapbase		= AST_VUART0_BASE,
		.irq			= IRQ_LPC,
		.uartclk		= (24*1000000L),
		.regshift		= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_SHARE_IRQ,
	},
	{ },
};

struct platform_device ast_vuart_device = {
	.name		= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= ast_vuart_data,
	},
};

void __init ast_add_device_vuart(void)
{
	void __iomem *vuart_base = ioremap(AST_VUART0_BASE, SZ_256);

	writel(0x0, (void *) (vuart_base + AST_VUART_CTRLA));		
	writel(SET_SIRQ_NUM(AST_SIRQ) |0x3, (void *) (vuart_base + AST_VUART_CTRLB));
	writel(AST_COM_PORT & 0xff, (void *) (vuart_base + AST_VUART_ADDRL));
	writel(AST_COM_PORT >> 8, (void *) (vuart_base + AST_VUART_ADDRH));
	writel(0x0, (void *) (vuart_base + AST_VUART_CTRLF));
	writel(VUART_ENABLE | VUART_SIRQ_POLARITY | VUART_DISABLE_H_TX_DISCARD, (void *) (vuart_base + AST_VUART_CTRLA));	

	iounmap(vuart_base);
	platform_device_register(&ast_vuart_device);
}
#else
void __init ast_add_device_vuart(void) {}
#endif
