/********************************************************************************
* File Name     : arch/arm/mach-aspeed/include/plat/ast-scu.h
* Author        : Ryan Chen
* Description   : AST SCU Service Header
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

*   History      :
*      1. 2012/08/03 Ryan Chen create this file
*
********************************************************************************/

#ifndef __AST_SCU_H_INCLUDED
#define __AST_SCU_H_INCLUDED

//information
extern void ast_scu_show_system_info (void);
extern void ast_scu_sys_rest_info(void);
extern u32 ast_scu_revision_id(void);
extern u32 ast_scu_get_phy_interface(u8 mac_num);
extern u32 ast_scu_get_phy_config(u8 mac_num);
extern u32 ast_scu_get_vga_memsize(void);
extern void ast_scu_get_who_init_dram(void);


//CLK 
extern void ast_scu_uart_div(void);
extern u32 ast_get_clk_source(void);
extern u32 ast_get_h_pll_clk(void);
extern u32 ast_get_ahb_div(void);

extern u32 ast_get_m_pll_clk(void);
extern u32 ast_get_ahbclk(void);

extern u32 ast_get_pclk(void);
extern u32 ast_get_sd_clock_src(void);
extern u32 ast_get_d2_pll_clk(void);
extern void ast_set_d2_pll_clk(u32 pll_setting);

extern u32 ast_get_lhclk(void);
extern void ast_scu_osc_clk_output(void);

extern void ast_scu_set_vga_display(u8 enable);
extern u8 ast_scu_get_vga_display(void);

extern u32 ast_scu_get_vga_memsize(void);

//Ctrl Initial
extern void ast_scu_init_uart(u8);
extern void ast_scu_init_video(u8 dynamic_en);
extern void ast_scu_reset_video(void);
extern void ast_scu_init_eth(u8 num);
extern void ast_scu_init_lpc(void);
extern void ast_scu_reset_espi(void);
extern u8 ast_scu_get_lpc_plus_enable(void);
extern void ast_scu_init_usb_port1(void);
extern void ast_scu_init_usb_port2(void);
extern void ast_scu_init_uhci(void);
extern void ast_scu_init_sdhci(void);
extern void ast_scu_init_i2c(void);
extern void ast_scu_init_pwm_tacho(void);
extern void ast_scu_init_adc(void);
extern void ast_scu_init_mctp(void);
extern void ast_scu_init_pcie(void);
extern void ast_scu_init_xdma(void);
extern void ast_scu_init_peci(void);
extern void ast_scu_init_jtag(void);
extern void ast_scu_init_rfx(void);
extern void ast_scu_init_h264(void);
extern void ast_scu_set_crt_source(u8 dac_soource);
extern void ast_scu_init_crt(void);
extern void ast_scu_init_hace(void);
extern void ast_scu_uartx_init(void);

//Share pin
extern void ast_scu_multi_func_uart(u8 uart);
extern void ast_scu_multi_func_video(void);
extern void ast_scu_multi_func_eth(u8 num);

extern void ast_scu_multi_func_nand(void);

extern void ast_scu_multi_func_nor(void);

extern void ast_scu_multi_func_romcs(u8 num);

extern void ast_scu_multi_func_i2c(void);
extern void ast_scu_multi_func_pwm_tacho(void);
//0 : usb 1.1 hid controller, 1:usb 2.0 device controller, 2:usb 2.0 host2 controller
extern void ast_scu_multi_func_usb_port1_mode(u8 mode);
//0 : hub mode , 1: usb host mode
extern void ast_scu_multi_func_usb_port2_mode(u8 mode);
//0 : gpioQ6,7 mode , 1: usb1.1 host port 4 mode
extern void ast_scu_multi_func_usb_port34_mode(u8 mode);

extern void ast_scu_multi_func_sdhc_8bit_mode(void);
extern void ast_scu_multi_func_sdhc_slot(u8 slot);
extern void ast_scu_multi_func_crt(void);

extern void ast_scu_multi_nic_switch(u8 enable);
extern void ast_scu_multi_func_sgpio(void);

extern void ast_scu_hw_random_enable(u8 enable);
extern u32 ast_scu_hw_random_read(void);
extern u8 ast_scu_get_hw_random_type(void);
extern void ast_scu_set_hw_random_type(u8 type);
extern u8 ast_scu_adc_trim_read(void);

#endif
