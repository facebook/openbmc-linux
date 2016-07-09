/********************************************************************************
* arch/arm/plat-aspeed/include/plat/devs.h
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
********************************************************************************/
#ifndef __ASM_PLAT_ASPEED_H
#define __ASM_PLAT_ASPEED_H

extern void __init ast_add_all_devices(void);

//platform
extern void __init ast_add_device_uart(void);
extern void __init ast_add_device_vuart(void);
extern void __init ast_add_device_watchdog(void);
extern void __init ast_add_device_rtc(void);
extern void __init ast_add_device_gpio(void);
extern void __init ast_add_device_sgpio(void);

//ADC 

//Bus
extern void __init ast_add_device_lpc(void);
extern void __init ast_add_device_espi(void);
extern void __init ast_add_device_kcs(void);
extern void __init ast_add_device_bt(void);
extern void __init ast_add_device_snoop(void);

extern void __init ast_add_device_mailbox(void);
extern void __init ast_add_device_i2c(void);
extern void __init ast_add_device_i2c_common(void);
extern void __init ast_add_device_spi(void);
extern void __init ast_add_device_ehci(void);
extern void __init ast_add_device_uhci(void);
extern void __init ast_add_device_gmac(void);
extern void __init ast_add_device_udc11(void);
extern void __init ast_add_device_hid(void);

//USB 
extern void __init ast_add_device_udc20(void);

extern void __init ast_add_device_pcie(void);

extern void __init ast_add_device_peci(void);
extern void __init ast_add_device_jtag(void);

extern void __init ast_add_device_mctp(void);
extern void __init ast_add_device_xdma(void);

extern void __init ast_add_device_crypto(void);
//hwmon
extern void __init ast_add_device_pwm_fan(void);
extern void __init ast_add_device_adc(void);


extern void __init ast_add_device_vhub(void);

//Storage
extern void __init ast_add_device_nand(void);
extern void __init ast_add_device_flash(void);
extern void __init ast_add_device_sdhci(void);
extern void __init ast_add_device_nand(void);

//video
extern void __init ast_add_device_fb(void);
extern void __init ast_add_device_video(void);
extern void __init ast_add_device_h264(void);
extern void __init ast_add_device_formatter(void);
extern void __init ast_add_device_rfx(void);
extern void __init ast_add_device_rng(void);

#endif
