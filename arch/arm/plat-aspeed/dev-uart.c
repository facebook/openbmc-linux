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

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
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

#if defined(CONFIG_AST_UART_SDMA) && defined(CONFIG_SERIAL_AST_DMA_UART)
#include <mach/ast-uart-dma.h>
static struct ast_uart_sdma_data ast_uart0_dma_data = {
	.dma_ch = 0,	
};

static struct ast_uart_sdma_data ast_uart1_dma_data = {
	.dma_ch = 1,	
};

static struct ast_uart_sdma_data ast_uart2_dma_data = {
	.dma_ch = 2,	
};

static struct ast_uart_sdma_data ast_uart3_dma_data = {
	.dma_ch = 3,	
};

static struct ast_uart_sdma_data ast_uart4_dma_data = {
	.dma_ch = 4,	
};

static struct ast_uart_sdma_data ast_uart5_dma_data = {
	.dma_ch = 5,	
};

static struct ast_uart_sdma_data ast_uart6_dma_data = {
	.dma_ch = 6,	
};

static struct ast_uart_sdma_data ast_uart7_dma_data = {
	.dma_ch = 7,	
};

static struct ast_uart_sdma_data ast_uart8_dma_data = {
	.dma_ch = 8,	
};

static struct ast_uart_sdma_data ast_uart9_dma_data = {
	.dma_ch = 9,	
};

static struct ast_uart_sdma_data ast_uart10_dma_data = {
	.dma_ch = 10,	
};

static struct ast_uart_sdma_data ast_uart11_dma_data = {
	.dma_ch = 11,	
};

#endif	//CONFIG_SERIAL_AST_DMA_UART

#if defined(CONFIG_ARCH_AST1010)
static struct plat_serial8250_port ast_uart_data[] = {
	{
		.mapbase	= AST_UART0_BASE,
		.irq		= IRQ_UART0,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM32,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM32,
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
	{ },
};

#elif defined(CONFIG_ARCH_AST1070)	//ast2400 + ast1070
static struct plat_serial8250_port ast_uart_data[] = {
	{
		.mapbase	= AST_UART0_BASE,
		.irq		= IRQ_UART0,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
	{
		.mapbase	= AST_UART2_BASE,
		.irq		= IRQ_UART2,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
	{
		.mapbase	= AST_UART3_BASE,
		.irq		= IRQ_UART3,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
	{
		.mapbase	= AST_UART4_BASE,
		.irq		= IRQ_UART4,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},	
	{ },

};

#else
static struct plat_serial8250_port ast_uart_data[] = {
	{
		.mapbase	= AST_UART0_BASE,
		.irq		= IRQ_UART0,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
/* Without this, tty offset might change for others */
#if defined(CONFIG_YOSEMITE) || defined(CONFIG_FBTP) || defined(CONFIG_FBTTN)
	{
		.mapbase	= AST_UART2_BASE,
		.irq		= IRQ_UART2,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags	= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#endif
#ifdef CONFIG_SERIAL_AST_DMA_UART
#else
#ifdef AST_UART3_BASE
	{
		.mapbase	= AST_UART3_BASE,
		.irq		= IRQ_UART3,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
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

#endif

#if defined(CONFIG_AST_UART_SDMA) && defined(CONFIG_SERIAL_AST_DMA_UART)
static struct plat_serial8250_port ast_dma_uart_data[] = {
//If use UART1/2 must reset HICR9[5:4]
/*
	{
		.mapbase	= AST_UART1_BASE,
		.irq		= IRQ_UART1,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart0_dma_data,		
	},	
	{
		.mapbase	= AST_UART2_BASE,
		.irq		= IRQ_UART2,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart1_dma_data,				
	},	
*/	
	{
		.mapbase	= AST_UART3_BASE,
		.irq		= IRQ_UART3,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart2_dma_data,						
	},	
	{
		.mapbase	= AST_UART4_BASE,
		.irq		= IRQ_UART4,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart3_dma_data,		
	},	
	{
		.mapbase	= AST_UART5_BASE,
		.irq		= IRQ_UART5,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart4_dma_data,		
	},	
	{
		.mapbase	= AST_UART6_BASE,
		.irq		= IRQ_UART6,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart5_dma_data,		
	},	
	{
		.mapbase	= AST_UART7_BASE,
		.irq		= IRQ_UART7,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart6_dma_data,		
	},	
	{
		.mapbase	= AST_UART8_BASE,
		.irq		= IRQ_UART8,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart7_dma_data,		
	},	
	{
		.mapbase	= AST_UART9_BASE,
		.irq		= IRQ_UART9,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart8_dma_data,		
	},	
	{
		.mapbase	= AST_UART10_BASE,
		.irq		= IRQ_UART10,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart9_dma_data,		
	},	
	{
		.mapbase	= AST_UART11_BASE,
		.irq		= IRQ_UART11,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart10_dma_data,		
	},	
	{
		.mapbase	= AST_UART12_BASE,
		.irq		= IRQ_UART12,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &ast_uart11_dma_data,		
	},	
	{ },
};

struct platform_device ast_dma_uart_device = {
	.name	= "ast-uart-dma",
	.id		= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= ast_dma_uart_data,
	},
};

#endif

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
#elif defined(CONFIG_YOSEMITE)
	ast_scu_multi_func_uart(1);
	ast_scu_multi_func_uart(2);
	ast_scu_multi_func_uart(3);
	ast_scu_multi_func_uart(4);
#elif defined(CONFIG_FBTP)
	ast_scu_multi_func_uart(1);
	ast_scu_multi_func_uart(2);
	ast_scu_multi_func_uart(3);
#else
	ast_scu_multi_func_uart(1);
	ast_scu_multi_func_uart(3);
	ast_scu_multi_func_uart(4);
#endif	
	platform_device_register(&ast_uart_device);

#ifdef CONFIG_SERIAL_AST_DMA_UART
	if(CONFIG_AST_RUNTIME_DMA_UARTS > 2) {
		ast_scu_uartx_init();
		ast_scu_multi_func_uart(6);
		ast_scu_multi_func_uart(7);
		ast_scu_multi_func_uart(8);
		ast_scu_multi_func_uart(9);
		ast_scu_multi_func_uart(10);
		ast_scu_multi_func_uart(11);
		ast_scu_multi_func_uart(12);
		ast_scu_multi_func_uart(13);
	}
#endif	
	
#if defined(CONFIG_AST_UART_SDMA) && defined(CONFIG_SERIAL_AST_DMA_UART)
	platform_device_register(&ast_dma_uart_device);	
#endif
}
#else
void __init ast_add_device_uart(void) {}
#endif
