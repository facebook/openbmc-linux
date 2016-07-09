/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-udc20.c
* Author        : Ryan chen
* Description   : ASPEED USB Device 2.0
*
* Copyright (C) ASPEED Technology Inc.
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
*    1. 2013/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

#include <linux/platform_device.h>
#include <linux/usb/g_hid.h>

/* --------------------------------------------------------------------
 *  UDC 1.1
 * -------------------------------------------------------------------- */
#if defined(CONFIG_USB_AST) || defined(CONFIG_USB_AST_MODULE)
/* hid descriptor for a keyboard */
static struct hidg_func_descriptor my_keyboard_data = {
	.subclass		= 0, /* No subclass */
	.protocol		= 1, /* Keyboard */
	.report_length		= 8,
	.report_desc_length	= 63,
	.report_desc		= {
		0x05, 0x01,	/* USAGE_PAGE (Generic Desktop)	          */
		0x09, 0x06,	/* USAGE (Keyboard)                       */
		0xa1, 0x01,	/* COLLECTION (Application)               */
		0x05, 0x07,	/*   USAGE_PAGE (Keyboard)                */
		0x19, 0xe0,	/*   USAGE_MINIMUM (Keyboard LeftControl) */
		0x29, 0xe7,	/*   USAGE_MAXIMUM (Keyboard Right GUI)   */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x01,	/*   LOGICAL_MAXIMUM (1)                  */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x95, 0x08,	/*   REPORT_COUNT (8)                     */
		0x81, 0x02,	/*   INPUT (Data,Var,Abs)                 */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x81, 0x03,	/*   INPUT (Cnst,Var,Abs)                 */
		0x95, 0x05,	/*   REPORT_COUNT (5)                     */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x05, 0x08,	/*   USAGE_PAGE (LEDs)                    */
		0x19, 0x01,	/*   USAGE_MINIMUM (Num Lock)             */
		0x29, 0x05,	/*   USAGE_MAXIMUM (Kana)                 */
		0x91, 0x02,	/*   OUTPUT (Data,Var,Abs)                */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x03,	/*   REPORT_SIZE (3)                      */
		0x91, 0x03,	/*   OUTPUT (Cnst,Var,Abs)                */
		0x95, 0x06,	/*   REPORT_COUNT (6)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x65,	/*   LOGICAL_MAXIMUM (101)                */
		0x05, 0x07,	/*   USAGE_PAGE (Keyboard)                */
		0x19, 0x00,	/*   USAGE_MINIMUM (Reserved)             */
		0x29, 0x65,	/*   USAGE_MAXIMUM (Keyboard Application) */
		0x81, 0x00,	/*   INPUT (Data,Ary,Abs)                 */
		0xc0		/* END_COLLECTION                         */
	}
};

static struct platform_device my_keyboard_hid = {
	.name			= "hidg",
	.id			= 0,
	.num_resources		= 0,
	.resource		= 0,
	.dev.platform_data	= &my_keyboard_data,
};

static struct hidg_func_descriptor my_mouse_data = {
	.subclass = 0,
	.protocol = 2,
	.report_length = 4,
	.report_desc_length= 52,
	.report_desc={
		0x05,0x01,	/*Usage Page (Generic Desktop Controls)*/
		0x09,0x02,	/*Usage (Mouse)*/
		0xa1,0x01,	/*Collction (Application)*/
		0x09,0x01,	/*Usage (pointer)*/
		0xa1,0x00,	/*Collction (Physical)*/
		0x05,0x09,	/*Usage Page (Button)*/
		0x19,0x01,	/*Usage Minimum(1)*/
		0x29,0x03,	/*Usage Maximum(3) */ 
		0x15,0x00,	/*Logical Minimum(1)*/
		0x25,0x01,	/*Logical Maximum(1)*/
		0x95,0x03,	/*Report Count(5)  */
		0x75,0x01,	/*Report Size(1)*/
		0x81,0x02,	/*Input(Data,Variable,Absolute,BitFiled)*/
		0x95,0x01,	/*Report Count(1)*/
		0x75,0x05,	/*Report Size(5) */
		0x81,0x01,	/*Input(Constant,Array,Absolute,BitFiled) */
		0x05,0x01,	/*Usage Page (Generic Desktop Controls)*/
		0x09,0x30,	/*Usage(x)*/
		0x09,0x31,	/*Usage(y)*/
		0x09,0x38,	/*Usage(Wheel)*/
		0x15,0x81,	/*Logical Minimum(-127)*/
		0x25,0x7f,	/*Logical Maximum(127)*/
		0x75,0x08,	/*Report Size(8)*/
		0x95,0x02,	/*Report Count(2)  */
		0x81,0x06,	/*Input(Data,Variable,Relative,BitFiled)*/
		0xc0,	/*End Collection*/
		0xc0	/*End Collection*/
	}
};

static struct platform_device my_mouse_hid = {
        .name = "hidg",
        .id = 1,
        .num_resources = 0,
        .resource = 0,
        .dev = {
                .platform_data = &my_mouse_data,
        }
};

static struct resource ast_vhub_resource[] = {
	[0] = {
		.start = AST_VHUB_BASE,
		.end = AST_VHUB_BASE + SZ_1K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VHUB,
		.end = IRQ_VHUB,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_udc_dma_mask = 0xffffffffUL;
static struct platform_device ast_udc_device = {
	.name	= "ast_udc",
	.id 		= 0,
	.resource = ast_vhub_resource,
	.num_resources = ARRAY_SIZE(ast_vhub_resource),
	.dev = {
		.dma_mask = &ast_udc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},	
};

void __init ast_add_device_udc20(void)
{
	ast_scu_multi_func_usb_port1_mode(0);
	ast_scu_init_usb_port1();

	platform_device_register(&my_keyboard_hid);
	platform_device_register(&my_mouse_hid);

	platform_device_register(&ast_udc_device);
}
#else
void __init ast_add_device_udc20(void) {}
#endif
