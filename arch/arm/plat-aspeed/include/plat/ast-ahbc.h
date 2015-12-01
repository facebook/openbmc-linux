/********************************************************************************
* File Name     : arch/arm/mach-aspeed/include/plat/ast-ahbc.h
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
*      1. 2014/08/03 Ryan Chen create this file
*
********************************************************************************/

#ifndef __AST_AHBC_H_INCLUDED
#define __AST_AHBC_H_INCLUDED

extern void ast_ahbc_boot_remap(void);

#ifdef AST_SOC_G5
extern void ast_ahbc_lpc_plus_mapping(u8 enable);
extern void ast_ahbc_peie_mapping(u8 enable);
#endif

#endif

