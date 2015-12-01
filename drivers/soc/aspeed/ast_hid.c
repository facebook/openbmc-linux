/********************************************************************************
* File Name     : driver/char/aspeed/ast_hid.c 
* Author         : Ryan Chen
* Description   : AST USB 1.1 HID driver
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
 */

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <plat/regs-udc11.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>


/***************************************************************************/
#define  IOCTL_USB11_MOUSE_PACKET   0x1112

typedef struct _IO_ACCESS_DATA {
    unsigned char Type;
    unsigned long Address;
    unsigned long Data;
    unsigned long Value;
    int      kernel_socket;
//    struct sockaddr_in address_svr;
} IO_ACCESS_DATA, *PIO_ACCESS_DATA;
/***************************************************************************/
#define AST_HID_DEBUG
	
#ifdef AST_HID_DEBUG
#define HID_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define HID_DBUG(fmt, args...)
#endif

struct ast_hid_info {
	void __iomem	*reg_base;	
	int irq;				//JTAG IRQ number 	
	wait_queue_head_t hid_waitqueue;	
	u32 flag;
	bool is_open;
	u32 state;
};

typedef struct _USB_DATA {
    ULONG    setup[2];
    ULONG    datalen;
    ULONG    in[32];
    ULONG    out[32];
    int      incnt;
    ULONG    outcnt, index;
} USB_DATA;

USB_DATA USB11_Data;

//80, 06, 00, 01, 00, 00
BYTE Descriptor1[18] = { 0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,                                 0x2a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                 0x00, 0x01 };

//80, 06, 00, 02, 00, 00
BYTE Descriptor2[34] = { 0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0xa0,
                         0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
                         0x02, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01,
                         0x22, 0x34, 0x00, 0x07, 0x05, 0x81, 0x03, 0x04,
                         0x00, 0x0a };

//81, 06, 00, 22, 00, 00
BYTE Descriptor3[52] = { 0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01,
                         0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
                         0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01,
                         0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
                         0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
                         0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
                         0x81, 0x06, 0xc0, 0xc0 };

/*
//Single Keyboard
//80, 06, 00, 01, 00, 00
BYTE Keyboard_Descriptor1[18] = {
    0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,                                 0x81, 0x0a, 0x01, 0x01, 0x10, 0x01, 0x00, 0x00,                                 0x00, 0x01 };

//80, 06, 00, 02, 00, 00
BYTE Keyboard_Descriptor2[34] = {
    0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0xa0,
    0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
    0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01,
    0x22, 0x41, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08,
    0x00, 0x0a };

//81, 06, 00, 22, 00, 00
BYTE Keyboard_Descriptor3[65] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07,
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x03, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x26, 0xff, 0x00, 0x05,
    0x07, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x81, 0x00,
    0xc0 };
*/

//80, 06, 00, 01, 00, 00
BYTE Keyboard_Descriptor1[18] = {
    0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,                                 0x81, 0x0a, 0x01, 0x01, 0x10, 0x01, 0x00, 0x00,                                 0x00, 0x01 };
/*
//80, 06, 00, 02, 00, 00
BYTE Keyboard_Descriptor2[59] = {
    0x09, 0x02, 0x3B, 0x00, 0x02, 0x01, 0x00, 0xa0,
    0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
    0x02, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01,
    0x22, 0x34, 0x00, 0x07, 0x05, 0x81, 0x03, 0x04,
    0x00, 0x0a, 0x09, 0x04, 0x01, 0x00, 0x01, 0x03,
    0x01, 0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x21,
    0x01, 0x22, 0x41, 0x00, 0x07, 0x05, 0x82, 0x03,
    0x08, 0x00, 0x0a };
*/

//80, 06, 00, 02, 00, 00
BYTE Keyboard_Descriptor2[59] = {
    0x09, 0x02, 0x3B, 0x00, 0x02, 0x01, 0x00, 0xa0,
    0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
    0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x21, 0x01,
    0x22, 0x41, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08,
    0x00, 0x0a, 0x09, 0x04, 0x01, 0x00, 0x01, 0x03,
    0x01, 0x02, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00,
    0x01, 0x22, 0x34, 0x00, 0x07, 0x05, 0x82, 0x03,
    0x04, 0x00, 0x0a };

//81, 06, 00, 22, 00, 00
BYTE Keyboard_Descriptor3[65] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07,
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x03, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x26, 0xff, 0x00, 0x05,
    0x07, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x81, 0x00,
    0xc0 };

DECLARE_WAIT_QUEUE_HEAD (my_queue);

int  flag = 0;

/*************************************************************************************/
static DEFINE_SPINLOCK(hid_state_lock);

/******************************************************************************/
static inline u32 
ast_hid_read(struct ast_hid_info *ast_hid, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_hid->reg_base + reg);
	HID_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_hid->reg_base + reg);;
#endif
}

static inline void
ast_hid_write(struct ast_hid_info *ast_hid, u32 val, u32 reg) 
{
	HID_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_hid->reg_base + reg);	
}

/******************************************************************************/

int USB11_send_mouse_packet(USB_DATA *USB_Info)
{
    unsigned char index = 0;
    unsigned long status;

//    printk ("IN Send MOUSE PACKET\n");
//    printk ("USB_Info->incnt = %d\n", USB_Info->incnt);
//    printk ("USB_Info->in[0] = %x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %x\n", USB_Info->in[1]);

        if (USB_Info->incnt >= 8) {
//            printk ("incnt >= 8\n");
            *(u32 *)(IO_ADDRESS(USB_2TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_2TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = 0x80;
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = 0x82;
            USB_Info->incnt -= 8;
        }
        else if (USB_Info->incnt >= 0) {
//            printk ("incnt < 8\n");
            *(u32 *)(IO_ADDRESS(USB_2TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_2TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = (USB_Info->incnt << 4);
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = (USB_Info->incnt << 4) | 0x02;
            USB_Info->incnt = 0;
        }
//        printk ("Go to Sleep\n");
//Check ACK
        wait_event_interruptible (my_queue, (flag == 1));
        flag = 0;

    return 0;
}


int USB11_send_keyboard_packet(USB_DATA *USB_Info)
{
    unsigned char index = 0;
    unsigned long status;

//    printk ("IN Send Keyboard PACKET\n");
//    printk ("USB_Info->incnt = %d\n", USB_Info->incnt);
//    printk ("USB_Info->in[0] = %x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %x\n", USB_Info->in[1]);

//    do {
        if (USB_Info->incnt >= 8) {
//            printk ("incnt >= 8\n");
            *(u32 *)(IO_ADDRESS(USB_1TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_1TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = 0x80;
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = 0x82;
            USB_Info->incnt -= 8;
        }
        else if (USB_Info->incnt >= 0) {
//            printk ("incnt < 8\n");
            *(u32 *)(IO_ADDRESS(USB_1TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_1TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = (USB_Info->incnt << 4);
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = (USB_Info->incnt << 4) | 0x02;
            USB_Info->incnt = 0;
        }
//        printk ("Go to Sleep\n");
//Check ACK
        wait_event_interruptible (my_queue, (flag == 1));
        flag = 0;
//    } while (USB_Info->incnt != 0);

    return 0;
}

void Setup_Mouse_Descriptor1(USB_DATA *USB_Info)
{
    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor1, 0x12);
//    printk ("USB_Info->in[0] = %8x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %8x\n", USB_Info->in[1]);
    if (USB_Info->datalen > 0x12) {
        USB_Info->incnt = 0x12;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor1(USB_DATA *USB_Info)
{
    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor1, 0x12);
//    printk ("USB_Info->in[0] = %8x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %8x\n", USB_Info->in[1]);
    if (USB_Info->datalen > 0x12) {
        USB_Info->incnt = 0x12;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Mouse_Descriptor2(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor2, 0x22);
    if (USB_Info->datalen > 0x22) {
        USB_Info->incnt = 0x22;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor2(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor2, 0x3B);
    if (USB_Info->datalen > 0x3B) {
        USB_Info->incnt = 0x3B;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Mouse_Descriptor3(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor3, 0x34);
    if (USB_Info->datalen > 0x34) {
        USB_Info->incnt = 0x34;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor3(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor3, 0x41);
    if (USB_Info->datalen > 0x41) {
        USB_Info->incnt = 0x41;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void RX_EP0_Data (USB_DATA *USB_Info)
{
    USB_Info->setup[0] = *(u32 *)(IO_ADDRESS(USB_0RXL));
    USB_Info->setup[1] = *(u32 *)(IO_ADDRESS(USB_0RXH));
    USB_Info->datalen = (USB_Info->setup[1] >> 16);
    USB_Info->setup[1] &= 0xffff;
}


void RX_EP0_OUT_Data (USB_DATA *USB_Info)
{
    unsigned long status;
    unsigned char index = 0;

//Disable EP0 OUT ACK Interrupt
//    *(u32 *)(IO_ADDRESS(USB_INTC)) = 0xcf;

    *(u32 *)(IO_ADDRESS(USB_EP0C)) = 0x04;
    do {
//        status = (*(u32 *)(IO_ADDRESS(USB_INTS)) & 0x10);
        status = (*(u32 *)(IO_ADDRESS(USB_EP0C)) & 0x04);
    } while (status != 0x04);
    USB_Info->outcnt = (*(u32 *)(IO_ADDRESS(USB_EP0C)) >> 8) & 0xf;

    if (USB_Info->outcnt != 0) {
        if (USB_Info->outcnt <= 4) {
            USB_Info->out[index++] = *(u32 *)(IO_ADDRESS(USB_0RXL));
        }
        else {
            USB_Info->out[index++] = *(u32 *)(IO_ADDRESS(USB_0RXH)); 
        }
    }
//    printk ("EP0_OUT Done\n");
}

int TX_EP0_Data (USB_DATA *USB_Info)
{
//    unsigned char index = 0;
    unsigned long status;

//    printk ("TX_EP0_Data USB_Info->incnt = %d\n", USB_Info->incnt);
//    printk ("TX_EP0_Data USB_Info->in[0] = %x\n", USB_Info->in[0]);
//    printk ("TX_EP0_Data USB_Info->in[1] = %x\n", USB_Info->in[1]);

    do {
        if (USB_Info->incnt >= 8) {
//            printk ("incnt >= 8\n");
            *(u32 *)(IO_ADDRESS(USB_0TXL)) = USB_Info->in[USB_Info->index++];
            *(u32 *)(IO_ADDRESS(USB_0TXH)) = USB_Info->in[USB_Info->index++];
//            *(u32 *)(IO_ADDRESS(USB_0TXL)) = USB_Info->in[index++];
//            *(u32 *)(IO_ADDRESS(USB_0TXH)) = USB_Info->in[index++];
            *(u32 *)(IO_ADDRESS(USB_EP0C)) = 0x80;
            *(u32 *)(IO_ADDRESS(USB_EP0C)) = 0x82;
            USB_Info->incnt -= 8;
        }
        else if (USB_Info->incnt >= 0) {
//            printk ("incnt < 8\n");
            *(u32 *)(IO_ADDRESS(USB_0TXL)) = USB_Info->in[USB_Info->index++];
            *(u32 *)(IO_ADDRESS(USB_0TXH)) = USB_Info->in[USB_Info->index++];
//            *(u32 *)(IO_ADDRESS(USB_0TXL)) = USB_Info->in[index++];
//            *(u32 *)(IO_ADDRESS(USB_0TXH)) = USB_Info->in[index++];
            *(u32 *)(IO_ADDRESS(USB_EP0C)) = (USB_Info->incnt << 4);
            *(u32 *)(IO_ADDRESS(USB_EP0C)) = (USB_Info->incnt << 4) | 0x02;
            USB_Info->incnt = 0;
//            USB_Info->incnt = -1;
        }
//Check ACK
        do {
//            status = (*(u32 *)(IO_ADDRESS(USB_INTS)) & 0x20);
            status = (*(u32 *)(IO_ADDRESS(USB_EP0C)) & 0x02);
//            printk ("0x18 reg = %8x\n", *(u32 *)(IO_ADDRESS(USB_EP0C)));
//            printk ("status = %8x\n", *(u32 *)(IO_ADDRESS(USB_INTS)));
        } while (status != 0x00);
//        printk ("ACK\n");
    } while (USB_Info->incnt != 0);


    return 0;
}

static void ast_hid_init(struct ast_udc11 *udc)
{
	//AST UDC initial 
	ast_udc_write(udc, 0x1ff, AST_UDC11_IER);
	ast_udc_write(udc, 0x0, AST_UDC11_EP0_CTRL);
	ast_udc_write(udc, 0x0, AST_UDC11_EP1_CTRL);
	ast_udc_write(udc, 0x0, AST_UDC11_EP2_CTRL);	
	ast_udc_write(udc, UDC11_CTRL_CLK_STOP | UDC11_CTRL_LS_EN | UDC11_CTRL_CONNECT_EN, AST_UDC11_CTRL);
}

void ast_udc11_init(void)
{
    unsigned int temp;
    *(u32 *)(IO_ADDRESS(USB_CTRL)) = 0;
    *(u32 *)(IO_ADDRESS(USB_CONF)) = 0;
    *(u32 *)(IO_ADDRESS(USB_EPTG)) = 3;
    *(u32 *)(IO_ADDRESS(USB_INTC)) = 0x1ff;
    *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x1ff;
    *(u32 *)(IO_ADDRESS(USB_EP0C)) = 0;
    *(u32 *)(IO_ADDRESS(USB_EP1C)) = 0x02;
    *(u32 *)(IO_ADDRESS(USB_EP2C)) = 0x02;
    *(u32 *)(IO_ADDRESS(USB_CTRL)) = 0x03;
}

static irqreturn_t ast_hid_interrupt(int this_irq, void *dev_id)
{
    unsigned int mask, status, IN_Status, Device_Address;

	struct ast_jtag_info *ast_jtag = dev_id;

    mask =  *(u32 *)(IO_ADDRESS(USB_INTC));
    status = *(u32 *)(IO_ADDRESS(USB_INTS));
    udelay (1);
//    printk ("mask = %8x, status = %8x\n", mask, status);

//Write to Clear
//    *(u32 *)(IO_ADDRESS(0x1e6e1014)) = (status & mask);
//      *(u32 *)(IO_ADDRESS(USB_INTS)) = status;
//Check Type


    if (status & 0x40) {
      *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x40;
//        printk ("Wake Up Mouse\n");
        flag = 1;
        wake_up_interruptible (&my_queue);
    }
    else if (status & 0x7) {
        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x07;
//        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x20;
//        TX_EP0_Data (&USB11_Data);
    }
    else if (status & 0x80) {
      *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x80;
//        printk ("Wake Up Keyboard\n");
        flag = 1;
        wake_up_interruptible (&my_queue);
    }
    else if (status & 0x20) {
        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x20;
//        TX_EP0_Data (&USB11_Data);
    }
    else if (status & 0x10) {
      *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x110;
//        printk ("Wake Up Keyboard\n");
//        flag = 1;
//        wake_up_interruptible (&my_queue);
    }
    else if (status & 0x08) {
             *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x08;
             printk ("Setup Packet Arrived\n");
             RX_EP0_Data (&USB11_Data);
             if ((USB11_Data.setup[0] & 0xffff) == 0x0500) {
                 printk ("Set Address\n");
                 USB11_Data.incnt = 0;
                 if (TX_EP0_Data (&USB11_Data) != 0) {
//                     printk ("IN ACK Return Fail\n");
                 }
                 Device_Address = ((USB11_Data.setup[0] >> 16) & 0xff);
                 *(u32 *)(IO_ADDRESS(USB_CONF)) = (Device_Address << 1) | 0x01;
             }
             else if ((USB11_Data.setup[0] == 0x01000680) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Device Descriptor\n");
//                  Setup_Mouse_Descriptor1 (&USB11_Data);
                  Setup_Descriptor1 (&USB11_Data);
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x02000680) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Configuration Descriptor\n");
//                  Setup_Mouse_Descriptor2 (&USB11_Data);
                  Setup_Descriptor2 (&USB11_Data);
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x00000a21) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Class Report 0\n");
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x00000a21) &&
                      (USB11_Data.setup[1] == 0x00000001)) {
                  printk ("Class Report 1\n");
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x22000681) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
//                  printk ("Mouse Descriptor\n");
                  Setup_Descriptor3 (&USB11_Data);
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x22000681) &&
                      (USB11_Data.setup[1] == 0x00000001)) {
//                  printk ("Keyboard Descriptor\n");
                  Setup_Mouse_Descriptor3 (&USB11_Data);
                  TX_EP0_Data (&USB11_Data);
             }
             else if ((USB11_Data.setup[0] & 0xffff) == 0x0900) {
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (&USB11_Data);
                  printk ("Configuration Done\n");
             }
    }
    else if (status & 0x300) {
//        printk ("OUT transaction\n");
        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x100;
        USB11_Data.incnt = 0;
        RX_EP0_OUT_Data (&USB11_Data);
    }
    else if (status & 0x100) {
        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x100;
//        *(u32 *)(IO_ADDRESS(USB_INTS)) = 0x20;
//        TX_EP0_Data (&USB11_Data);
    }

    return IRQ_HANDLED;
}

/*************************************************************************************/
struct ast_hid_info *ast_hid;
                                                                   
int ioaccess_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    IO_ACCESS_DATA  Kernal_IO_Data;
 
    memset (&Kernal_IO_Data, 0, sizeof(IO_ACCESS_DATA));

    Kernal_IO_Data = *(IO_ACCESS_DATA *)arg;

    switch (cmd) {
        case IOCTL_USB11_MOUSE_PACKET:
//            ret = USB11_send_packet(Kernal_IO_Data.Data);
            USB11_Data.in[0] = Kernal_IO_Data.Data;
            USB11_Data.in[1] = Kernal_IO_Data.Value;
            if (Kernal_IO_Data.Address == 0) {
//                printk ("Mouse\n");
                USB11_Data.incnt = 4;
                ret = USB11_send_mouse_packet(&USB11_Data);
            }
            else if (Kernal_IO_Data.Address == 1) {
//                printk ("Keyboard\n");
                USB11_Data.incnt = 8;
                ret = USB11_send_keyboard_packet(&USB11_Data);
            }
        break;

        default:
            ret = 3;
    }
    return ret;
}

static long hid_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
    int ret = 0;
	struct ast_hid_info *ast_hid = file->private_data;	
	void __user *argp = (void __user *)arg;

	switch (cmd) {
		case AST_JTAG_IOCDATA:
			if (copy_from_user(&data, argp, sizeof(struct dr_xfer)))
				ret = -EFAULT;
			else 
				ast_hid_dr_xfer(ast_hid ,&data);

			if (copy_to_user(argp, &data, sizeof(struct dr_xfer)))
				ret = -EFAULT;
			break;
		default:
			return -ENOTTY;
	}

    return ret;
}

static int hid_open(struct inode *inode, struct file *file)
{
//	struct ast_hid_info *drvdata;

	spin_lock(&hid_state_lock);

//	drvdata = container_of(inode->i_cdev, struct ast_hid_info, cdev);
	
	if (ast_hid->is_open) {
		spin_unlock(&hid_state_lock);
		return -EBUSY;
	}

	ast_hid->is_open = true;
	file->private_data = ast_hid;

	spin_unlock(&hid_state_lock);

	return 0;
}

static int hid_release(struct inode *inode, struct file *file)
{
	struct ast_hid_info *drvdata = file->private_data;

	spin_lock(&hid_state_lock);

	drvdata->is_open = false;

	spin_unlock(&hid_state_lock);

	return 0;
}


static const struct file_operations ast_hid_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= hid_ioctl,
	.open		= hid_open,
	.release	= hid_release,
};

struct miscdevice ast_hid_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-hid",
	.fops = &ast_hid_fops,
};

static int ast_hid_probe(struct platform_device *pdev)
{
	struct resource *res;

	int ret=0;


	HID_DBUG("ast_hid_probe\n");	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	if (!(ast_hid = kzalloc(sizeof(struct ast_hid_info), GFP_KERNEL))) {
		return -ENOMEM;
	}
	
	ast_hid->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_hid->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_hid->irq = platform_get_irq(pdev, 0);
	if (ast_hid->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}


    ret = request_irq(ast_hid->irq, ast_hid_interrupt, IRQF_DISABLED, "ast-jtag", ast_hid);
    if (ret) {
        printk("JTAG Unable to get IRQ");
		goto out_region;
    }

    init_waitqueue_head(&ast_hid->hid_waitqueue);

	ret = misc_register(&ast_hid_misc);
	if (ret){		
		printk(KERN_ERR "PECI : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_hid);

	ast_udc11_init();
	
	printk(KERN_INFO "ast_hid: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(ast_hid->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_hid_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_hid_info *ast_hid = platform_get_drvdata(pdev);

	HID_DBUG("ast_hid_remove\n");

	misc_deregister(&ast_hid_misc);

	free_irq(ast_hid->irq, ast_hid);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_hid->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}


#ifdef CONFIG_PM
static int 
ast_hid_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int 
ast_hid_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_hid_suspend        NULL
#define ast_hid_resume         NULL
#endif

static struct platform_driver ast_hid_driver = {
	.probe			= ast_hid_probe,
	.remove 		= __devexit_p(ast_hid_remove),
    .suspend        = ast_hid_suspend,
    .resume         = ast_hid_resume,
    .driver         = {
            .name   = "ast-hid",
            .owner  = THIS_MODULE,
    },
};

static int __init 
ast_hid_init(void)
{
	return platform_driver_register(&ast_hid_driver);
}

static void __exit 
ast_hid_exit(void)
{
	platform_driver_unregister(&ast_hid_driver);
}

module_init(ast_hid_init);
module_exit(ast_hid_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("HID Driver");
MODULE_LICENSE("GPL");
