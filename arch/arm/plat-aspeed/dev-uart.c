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

#if defined(CONFIG_COLDFIRE)
#include <asm/sizes.h>
#include <asm/arch/devs.h>
#include <asm/arch/platform.h>
#include <asm/arch/irqs.h>
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <plat/ast-scu.h>
#include <plat/devs.h>
#endif

/* --------------------------------------------------------------------
 * UART
 * -------------------------------------------------------------------- */
#ifdef CONFIG_SERIAL_8250
static struct plat_serial8250_port ast_uart_data[] = {
	{
		.mapbase	= AST_UART0_BASE,
		.irq		= IRQ_UART0,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
#if defined(CONFIG_COLDFIRE)		
		.iotype		= UPIO_MEM32,
#else
		.iotype		= UPIO_MEM,
#endif		
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#if defined(CONFIG_ARCH_AST1010)	
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
	{
		.mapbase	= AST_UART2_BASE,
		.irq		= IRQ_UART2,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM32,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
#else
//BMC UART 1 ,2 default to LPC
#ifdef CONFIG_ARCH_AST1070
#ifdef AST_UART1_BASE
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
#endif
#ifdef AST_UART2_BASE
	{
		.mapbase	= AST_UART2_BASE,
		.irq		= IRQ_UART2,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
#endif
#endif
#ifdef AST_UART1_BASE
	{
		.mapbase	= AST_UART1_BASE,
		.membase	= (char*)(IO_ADDRESS(AST_UART1_BASE)),
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#endif
#ifdef AST_UART3_BASE
	{
		.mapbase	= AST_UART3_BASE,
		.irq		= IRQ_UART3,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
#endif
#ifdef AST_UART4_BASE
	{
		.mapbase	= AST_UART4_BASE,
		.irq		= IRQ_UART4,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
#endif
#endif	
	{ },
};

struct platform_device ast_uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= ast_uart_data,
	},
};

void __init ast_add_device_uart(void)
{
#if defined(CONFIG_ARCH_AST1010)
#else
	ast_scu_multi_func_uart(1);		
	ast_scu_multi_func_uart(3);		
	ast_scu_multi_func_uart(4);	
#endif	
	platform_device_register(&ast_uart_device);
}
#else
void __init ast_add_device_uart(void) {}
#endif
