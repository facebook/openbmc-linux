/* arch/arm/plat-aspeed/include/mach/regs-iic.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED VUART Controller
*/

#ifndef __AST_ESPI_H_
#define __AST_ESPI_H_ 

#define AST_ESPI_CTRL			0x00		/* Engine Control */
#define AST_ESPI_STS				0x04		/* Engine Status */
#define AST_ESPI_ISR				0x08		/* Interrupt Status */
#define AST_ESPI_IER				0x0C		/* Interrupt Enable */
#define AST_ESPI_PCP_RX_DMA		0x10		/* DMA Address of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_RX_CTRL	0x14		/* Control of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_RX_DATA	0x18		/* Data Port of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_TX_DMA		0x20		/* DMA Address of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCP_TX_CTRL	0x24		/* Control of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCP_TX_DATA	0x28		/* Data Port of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCNP_TX_DMA	0x30		/* DMA Address of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_PCNP_TX_CTRL	0x34		/* Control of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_PCNP_TX_DATA	0x38		/* Data Port of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_OOB_RX_DMA	0x40		/* DMA Address of OOB Channel Rx Package */
#define AST_ESPI_OOB_RX_CTRL	0x44		/* Control of OOB Channel Rx Package */
#define AST_ESPI_OOB_RX_DATA	0x48		/* Date port of OOB Channel Rx Package */
#define AST_ESPI_OOB_TX_DMA	0x50		/* DMA Address of OOB Channel Tx Package */
#define AST_ESPI_OOB_TX_CTRL	0x54		/* Control of OOB Channel Tx Package */
#define AST_ESPI_OOB_TX_DATA	0x58		/* Date port of OOB Channel Tx Package */
#define AST_ESPI_FLASH_RX_DMA	0x60		/* DMA Address of Flash Channel Rx Package */
#define AST_ESPI_FLASH_RX_CTRL	0x64		/* Control of Flash Channel Rx Package */
#define AST_ESPI_FLASH_RX_DATA	0x68		/* Date port of Flash Channel Rx Package */
#define AST_ESPI_FLASH_TX_DMA	0x70		/* DMA Address of Flash Channel Tx Package */
#define AST_ESPI_FLASH_TX_CTRL	0x74		/* Control of Flash Channel Tx Package */
#define AST_ESPI_FLASH_TX_DATA	0x78		/* Date port of Flash Channel Tx Package */

#define AST_ESPI_PC_RX_SADDR	0x84		/* Mapping Source Address of Peripheral Channel Rx Package */
#define AST_ESPI_PC_RX_TADDR	0x88		/* Mapping Target Address of Peripheral Channel Rx Package */
#define AST_ESPI_PC_RX_TADDRM	0x8c		/* Mapping Target Address Mask of Peripheral Channel Rx Package */
#define AST_ESPI_FLASH_TADDRM	0x90		/* Mapping Target Address Mask of Flash Channel */
#define AST_ESPI_SYS_IER			0x94		/* Interrupt enable of System Event from Master */
#define AST_ESPI_SYS_EVENT		0x98		/* System Event from and to Master */
#define AST_ESPI_GPIO_VIRTCH	0x9C		/* GPIO through Virtual Wire Cahnnel  */
#define AST_ESPI_GCAP_CONFIG	0xA0		/* General Capabilities and Configuration  */
#define AST_ESPI_CH0CAP_CONFIG	0xA4		/* Channel 0 Capabilities and Configuration  */
#define AST_ESPI_CH1CAP_CONFIG	0xA8		/* Channel 1 Capabilities and Configuration  */
#define AST_ESPI_CH2CAP_CONFIG	0xAC		/* Channel 2 Capabilities and Configuration  */
#define AST_ESPI_CH3CAP_CONFIG	0xB0		/* Channel 3 Capabilities and Configuration  */

#define AST_ESPI_GPIO_DIR_VIRTCH	0xB4		/* GPIO Direction of Virtual Wire Channel  */
#define AST_ESPI_GPIO_SEL_VIRTCH	0xB8		/* GPIO Selection of Virtual Wire Channel  */
#define AST_ESPI_GPIO_REST_VIRTCH	0xBC		/* GPIO Reset Selection of Virtual Wire Channel  */

#define AST_ESPI_SYS1_IER			0x100		/* Interrupt enable of System Event from Master */
#define AST_ESPI_SYS1_EVENT			0x104		/* Interrupt enable of System Event from Master */

#define AST_ESPI_SYS_INT_T0		0x110
#define AST_ESPI_SYS_INT_T1		0x114
#define AST_ESPI_SYS_INT_T2		0x118
#define AST_ESPI_SYS_EVENT_ISR		0x11C		


#define AST_ESPI_SYS1_INT_T0		0x120
#define AST_ESPI_SYS1_INT_T1		0x124
#define AST_ESPI_SYS1_INT_T2		0x128
#define AST_ESPI_SYS1_INT_STS		0x12C		



/* AST_ESPI_CTRL	-	0x00	:Engine Control */
#define ESPI_CTRL_FLASH_TX_SW_RESET	(0x1 << 31)
#define ESPI_CTRL_FLASH_RX_SW_RESET	(0x1 << 30)
#define ESPI_CTRL_OOB_TX_SW_RESET		(0x1 << 29)
#define ESPI_CTRL_OOB_RX_SW_RESET		(0x1 << 28)
#define ESPI_CTRL_PCNP_TX_SW_RESET		(0x1 << 27)
#define ESPI_CTRL_PCNP_RX_SW_RESET		(0x1 << 26)
#define ESPI_CTRL_PCP_TX_SW_RESET		(0x1 << 25)
#define ESPI_CTRL_PCP_RX_SW_RESET		(0x1 << 24)
#define ESPI_CTRL_FLASH_TX_DMA			(0x1 << 23)
#define ESPI_CTRL_FLASH_RX_DMA			(0x1 << 22)
#define ESPI_CTRL_OOB_TX_DMA			(0x1 << 21)
#define ESPI_CTRL_OOB_RX_DMA			(0x1 << 20)
#define ESPI_CTRL_PCNP_TX_DMA			(0x1 << 19)
/* */
#define ESPI_CTRL_PCP_TX_DMA			(0x1 << 17)
#define ESPI_CTRL_PCP_RX_DMA			(0x1 << 16)
/* */
#define ESPI_CTRL_DIR_RESET				(0x1 << 13)
#define ESPI_CTRL_VAL_RESET				(0x1 << 12)
#define ESPI_CTRL_SW_FLASH_READ		(0x1 << 10)
#define ESPI_CTRL_SW_GPIO_VIRTCH		(0x1 << 9)


/* AST_ESPI_ISR	- 0x08 : Interrupt Status */
#define ESPI_ISR_HW_RESET				(0x1 << 31)
/* */
#define ESPI_ISR_VIRTW_SYS1				(0x1 << 22)
#define ESPI_ISR_FLASH_TX_ERR			(0x1 << 21)
#define ESPI_ISR_OOB_TX_ERR				(0x1 << 20)
#define ESPI_ISR_FLASH_TX_ABORT			(0x1 << 19)
#define ESPI_ISR_OOB_TX_ABORT			(0x1 << 18)
#define ESPI_ISR_PCNP_TX_ABORT			(0x1 << 17)
#define ESPI_ISR_PCP_TX_ABORT			(0x1 << 16)
#define ESPI_ISR_FLASH_RX_ABORT			(0x1 << 15)
#define ESPI_ISR_OOB_RX_ABORT			(0x1 << 14)
#define ESPI_ISR_PCNP_RX_ABORT			(0x1 << 13)
#define ESPI_ISR_PCP_RX_ABORT			(0x1 << 12)
#define ESPI_ISR_PCNP_TX_ERR			(0x1 << 11)
#define ESPI_ISR_PCP_TX_ERR				(0x1 << 10)
#define ESPI_ISR_VIRTW_GPIO				(0x1 << 9)
#define ESPI_ISR_VIRTW_SYS				(0x1 << 8)
#define ESPI_ISR_FLASH_TX_COMP			(0x1 << 7)
#define ESPI_ISR_FLASH_RX_COMP			(0x1 << 6)
#define ESPI_ISR_OOB_TX_COMP			(0x1 << 5)
#define ESPI_ISR_OOB_RX_COMP			(0x1 << 4)
#define ESPI_ISR_PCNP_TX_COMP			(0x1 << 3)
/* */
#define ESPI_ISR_PCP_TX_COMP			(0x1 << 1)
#define ESPI_ISR_PCP_RX_COMP			(0x1)


/* AST_ESPI_PCP_RX_CTRL	-0x14	:	Control of Peripheral Channel Posted Rx Package */
#define ESPI_TRIGGER_PACKAGE			(0x1 << 31)

#define ESPI_GET_CYCLE_TYPE(x)			(x & 0xff)
#define ESPI_GET_TAG(x)					((x >> 8) & 0xf)
#define ESPI_GET_LEN(x)					((x >> 12) & 0xfff)


/* Cycle Type Define	*/	

/* eSPI Peripheral Channel	*/	
#define MEM_READ32				0x00
#define MEM_WRITE32				0x01
#define MEM_READ64				0x02
#define MEM_WRITE64				0x03

#define MESSAGE_TRCV			0x10
#define MESSAGED_TRCV			0x11

#define SUCCESS_M_COMPLETE		0x09
#define SUCCESS_F_COMPLETE		0x0B
#define SUCCESS_L_COMPLETE		0x0D
#define SUCCESS_O_COMPLETE		0x0F

#define USUCCESS_L_COMPLETE	0x0C
#define USUCCESS_O_COMPLETE	0x0E

#define FLASH_READ				0x00
#define FLASH_WRITE				0x01
#define FLASH_ERASE				0x02



/* OOB Message Channel	*/	


/* AST_ESPI_SYS_EVENT			0x98		System Event from and to Master */
/* AST_ESPI_SYS_EVENT_ISR		0x11C	System Event from and to Master interrupt sts */

#define ESPI_HOST_REST_ACK		(0x1 << 27)

#define ESPI_REST_CPU_INIT		(0x1 << 26)

#define ESPI_BOOT_STS			(0x1 << 23)
#define ESPI_NFATEL_ERR			(0x1 << 22)
#define ESPI_FATEL_ERR			(0x1 << 21)
#define ESPI_BOOT_DWN			(0x1 << 20)
#define ESPI_OOB_REST_ACK		(0x1 << 16)

#define ESPI_HOST_NMI_OUT		(0x1 << 10)
#define ESPI_HOST_SMI_OUT		(0x1 << 9)

#define ESPI_HOST_RST_WARN		(0x1 << 8)

#define ESPI_OOB_RST_WARN		(0x1 << 6)

#define ESPI_SYS_S5_SLEEP		(0x1 << 2)
#define ESPI_SYS_S4_SLEEP		(0x1 << 1)
#define ESPI_SYS_S3_SLEEP		(0x1)



/* AST_ESPI_GCAP_CONFIG	0xA0		General Capabilities and Configuration  */
#define GET_GCAP_IO_MODE(x)		((x >> 26) & 0x3)
#define GET_GCAP_OP_FREQ(x)		((x >> 20) & 0x7)
#define GET_GCAP_CH_SUPPORT(x)	(x & 0xf)




#endif 
