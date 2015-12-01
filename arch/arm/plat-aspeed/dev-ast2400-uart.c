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

#include <plat/ast2400-scu.h>
#include <plat/ext-devs.h>

/* --------------------------------------------------------------------
 * UART
 * -------------------------------------------------------------------- */
#ifdef CONFIG_SERIAL_8250
static struct plat_serial8250_port ast2400_uart_data[] = {
	{
		.mapbase	= AST_PCI_EXT_UART0,
		.irq		= IRQ_AST2400_UART0,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
	{ },
};

struct platform_device ast2400_uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= ast2400_uart_data,
	},
};

void __init ast2400_add_device_uart(void)
{
//	ast2400_scu_multi_func_uart(1);		
//	ast2400_scu_multi_func_uart(2);	
	platform_device_register(&ast2400_uart_device);
}
#else
void __init ast2400_add_device_uart(void) {}
#endif
