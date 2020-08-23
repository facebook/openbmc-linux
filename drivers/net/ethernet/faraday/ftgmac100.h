/*
 * Faraday FTGMAC100 Gigabit Ethernet
 *
 * (C) Copyright 2009-2011 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __FTGMAC100_H
#define __FTGMAC100_H

#define FTGMAC100_OFFSET_ISR		0x00
#define FTGMAC100_OFFSET_IER		0x04
#define FTGMAC100_OFFSET_MAC_MADR	0x08
#define FTGMAC100_OFFSET_MAC_LADR	0x0c
#define FTGMAC100_OFFSET_MAHT0		0x10
#define FTGMAC100_OFFSET_MAHT1		0x14
#define FTGMAC100_OFFSET_NPTXPD		0x18
#define FTGMAC100_OFFSET_RXPD		0x1c
#define FTGMAC100_OFFSET_NPTXR_BADR	0x20
#define FTGMAC100_OFFSET_RXR_BADR	0x24
#define FTGMAC100_OFFSET_HPTXPD		0x28
#define FTGMAC100_OFFSET_HPTXR_BADR	0x2c
#define FTGMAC100_OFFSET_ITC		0x30
#define FTGMAC100_OFFSET_APTC		0x34
#define FTGMAC100_OFFSET_DBLAC		0x38
#define FTGMAC100_OFFSET_DMAFIFOS	0x3c
#define FTGMAC100_OFFSET_REVR		0x40
#define FTGMAC100_OFFSET_FEAR		0x44
#define FTGMAC100_OFFSET_TPAFCR		0x48
#define FTGMAC100_OFFSET_RBSR		0x4c
#define FTGMAC100_OFFSET_MACCR		0x50
#define FTGMAC100_OFFSET_MACSR		0x54
#define FTGMAC100_OFFSET_TM		0x58
#define FTGMAC100_OFFSET_PHYCR		0x60
#define FTGMAC100_OFFSET_PHYDATA	0x64
#define FTGMAC100_OFFSET_FCR		0x68
#define FTGMAC100_OFFSET_BPR		0x6c
#define FTGMAC100_OFFSET_WOLCR		0x70
#define FTGMAC100_OFFSET_WOLSR		0x74
#define FTGMAC100_OFFSET_WFCRC		0x78
#define FTGMAC100_OFFSET_WFBM1		0x80
#define FTGMAC100_OFFSET_WFBM2		0x84
#define FTGMAC100_OFFSET_WFBM3		0x88
#define FTGMAC100_OFFSET_WFBM4		0x8c
#define FTGMAC100_OFFSET_NPTXR_PTR	0x90
#define FTGMAC100_OFFSET_HPTXR_PTR	0x94
#define FTGMAC100_OFFSET_RXR_PTR	0x98
#define FTGMAC100_OFFSET_TX		0xa0
#define FTGMAC100_OFFSET_TX_MCOL_SCOL	0xa4
#define FTGMAC100_OFFSET_TX_ECOL_FAIL	0xa8
#define FTGMAC100_OFFSET_TX_LCOL_UND	0xac
#define FTGMAC100_OFFSET_RX		0xb0
#define FTGMAC100_OFFSET_RX_BC		0xb4
#define FTGMAC100_OFFSET_RX_MC		0xb8
#define FTGMAC100_OFFSET_RX_PF_AEP	0xbc
#define FTGMAC100_OFFSET_RX_RUNT	0xc0
#define FTGMAC100_OFFSET_RX_CRCER_FTL	0xc4
#define FTGMAC100_OFFSET_RX_COL_LOST	0xc8

/*
 * Interrupt status register & interrupt enable register
 */
#define FTGMAC100_INT_RPKT_BUF		(1 << 0)
#define FTGMAC100_INT_RPKT_FIFO		(1 << 1)
#define FTGMAC100_INT_NO_RXBUF		(1 << 2)
#define FTGMAC100_INT_RPKT_LOST		(1 << 3)
#define FTGMAC100_INT_XPKT_ETH		(1 << 4)
#define FTGMAC100_INT_XPKT_FIFO		(1 << 5)
#define FTGMAC100_INT_NO_NPTXBUF	(1 << 6)
#define FTGMAC100_INT_XPKT_LOST		(1 << 7)
#define FTGMAC100_INT_AHB_ERR		(1 << 8)
#define FTGMAC100_INT_PHYSTS_CHG	(1 << 9)
#define FTGMAC100_INT_NO_HPTXBUF	(1 << 10)

/*
 * Interrupt timer control register
 */
#define FTGMAC100_ITC_RXINT_CNT(x)	(((x) & 0xf) << 0)
#define FTGMAC100_ITC_RXINT_THR(x)	(((x) & 0x7) << 4)
#define FTGMAC100_ITC_RXINT_TIME_SEL	(1 << 7)
#define FTGMAC100_ITC_TXINT_CNT(x)	(((x) & 0xf) << 8)
#define FTGMAC100_ITC_TXINT_THR(x)	(((x) & 0x7) << 12)
#define FTGMAC100_ITC_TXINT_TIME_SEL	(1 << 15)

/*
 * Automatic polling timer control register
 */
#define FTGMAC100_APTC_RXPOLL_CNT(x)	(((x) & 0xf) << 0)
#define FTGMAC100_APTC_RXPOLL_TIME_SEL	(1 << 4)
#define FTGMAC100_APTC_TXPOLL_CNT(x)	(((x) & 0xf) << 8)
#define FTGMAC100_APTC_TXPOLL_TIME_SEL	(1 << 12)

/*
 * DMA burst length and arbitration control register
 */
#define FTGMAC100_DBLAC_RXFIFO_LTHR(x)	(((x) & 0x7) << 0)
#define FTGMAC100_DBLAC_RXFIFO_HTHR(x)	(((x) & 0x7) << 3)
#define FTGMAC100_DBLAC_RX_THR_EN	(1 << 6)
#define FTGMAC100_DBLAC_RXBURST_SIZE(x)	(((x) & 0x3) << 8)
#define FTGMAC100_DBLAC_TXBURST_SIZE(x)	(((x) & 0x3) << 10)
#define FTGMAC100_DBLAC_RXDES_SIZE(x)	(((x) & 0xf) << 12)
#define FTGMAC100_DBLAC_TXDES_SIZE(x)	(((x) & 0xf) << 16)
#define FTGMAC100_DBLAC_IFG_CNT(x)	(((x) & 0x7) << 20)
#define FTGMAC100_DBLAC_IFG_INC		(1 << 23)

/*
 * DMA FIFO status register
 */
#define FTGMAC100_DMAFIFOS_RXDMA1_SM(dmafifos)	((dmafifos) & 0xf)
#define FTGMAC100_DMAFIFOS_RXDMA2_SM(dmafifos)	(((dmafifos) >> 4) & 0xf)
#define FTGMAC100_DMAFIFOS_RXDMA3_SM(dmafifos)	(((dmafifos) >> 8) & 0x7)
#define FTGMAC100_DMAFIFOS_TXDMA1_SM(dmafifos)	(((dmafifos) >> 12) & 0xf)
#define FTGMAC100_DMAFIFOS_TXDMA2_SM(dmafifos)	(((dmafifos) >> 16) & 0x3)
#define FTGMAC100_DMAFIFOS_TXDMA3_SM(dmafifos)	(((dmafifos) >> 18) & 0xf)
#define FTGMAC100_DMAFIFOS_RXFIFO_EMPTY		(1 << 26)
#define FTGMAC100_DMAFIFOS_TXFIFO_EMPTY		(1 << 27)
#define FTGMAC100_DMAFIFOS_RXDMA_GRANT		(1 << 28)
#define FTGMAC100_DMAFIFOS_TXDMA_GRANT		(1 << 29)
#define FTGMAC100_DMAFIFOS_RXDMA_REQ		(1 << 30)
#define FTGMAC100_DMAFIFOS_TXDMA_REQ		(1 << 31)

/*
 * Receive buffer size register
 */
#define FTGMAC100_RBSR_SIZE(x)		((x) & 0x3fff)

/*
 * MAC control register
 */
#define FTGMAC100_MACCR_TXDMA_EN	(1 << 0)
#define FTGMAC100_MACCR_RXDMA_EN	(1 << 1)
#define FTGMAC100_MACCR_TXMAC_EN	(1 << 2)
#define FTGMAC100_MACCR_RXMAC_EN	(1 << 3)
#define FTGMAC100_MACCR_RM_VLAN		(1 << 4)
#define FTGMAC100_MACCR_HPTXR_EN	(1 << 5)
#define FTGMAC100_MACCR_LOOP_EN		(1 << 6)
#define FTGMAC100_MACCR_ENRX_IN_HALFTX	(1 << 7)
#define FTGMAC100_MACCR_FULLDUP		(1 << 8)
#define FTGMAC100_MACCR_GIGA_MODE	(1 << 9)
#define FTGMAC100_MACCR_CRC_APD		(1 << 10)
#define FTGMAC100_MACCR_RX_RUNT		(1 << 12)
#define FTGMAC100_MACCR_JUMBO_LF	(1 << 13)
#define FTGMAC100_MACCR_RX_ALL		(1 << 14)
#define FTGMAC100_MACCR_HT_MULTI_EN	(1 << 15)
#define FTGMAC100_MACCR_RX_MULTIPKT	(1 << 16)
#define FTGMAC100_MACCR_RX_BROADPKT	(1 << 17)
#define FTGMAC100_MACCR_DISCARD_CRCERR	(1 << 18)
#define FTGMAC100_MACCR_FAST_MODE	(1 << 19)
#define FTGMAC100_MACCR_SW_RST		(1 << 31)

/*
 * PHY control register
 */
#define FTGMAC100_PHYCR_MDC_CYCTHR_MASK	0x3f
#define FTGMAC100_PHYCR_MDC_CYCTHR(x)	((x) & 0x3f)
#define FTGMAC100_PHYCR_PHYAD(x)	(((x) & 0x1f) << 16)
#define FTGMAC100_PHYCR_REGAD(x)	(((x) & 0x1f) << 21)
#define FTGMAC100_PHYCR_MIIRD		(1 << 26)
#define FTGMAC100_PHYCR_MIIWR		(1 << 27)

/*
 * PHY data register
 */
#define FTGMAC100_PHYDATA_MIIWDATA(x)		((x) & 0xffff)
#define FTGMAC100_PHYDATA_MIIRDATA(phydata)	(((phydata) >> 16) & 0xffff)

/*
 * Transmit descriptor, aligned to 16 bytes
 */
struct ftgmac100_txdes {
	unsigned int	txdes0;
	unsigned int	txdes1;
	unsigned int	txdes2;	/* not used by HW */
	unsigned int	txdes3;	/* TXBUF_BADR */
} __attribute__ ((aligned(16)));

#define FTGMAC100_TXDES0_TXBUF_SIZE(x)	((x) & 0x3fff)
#define FTGMAC100_TXDES0_CRC_ERR	(1 << 19)
#define FTGMAC100_TXDES0_LTS		(1 << 28)
#define FTGMAC100_TXDES0_FTS		(1 << 29)
#define FTGMAC100_TXDES0_EDOTR		(1 << 30)
#define FTGMAC100_TXDES0_TXDMA_OWN	(1 << 31)

#define FTGMAC100_TXDES1_VLANTAG_CI(x)	((x) & 0xffff)
#define FTGMAC100_TXDES1_INS_VLANTAG	(1 << 16)
#define FTGMAC100_TXDES1_TCP_CHKSUM	(1 << 17)
#define FTGMAC100_TXDES1_UDP_CHKSUM	(1 << 18)
#define FTGMAC100_TXDES1_IP_CHKSUM	(1 << 19)
#define FTGMAC100_TXDES1_TX2FIC		(1 << 30)
#define FTGMAC100_TXDES1_TXIC		(1 << 31)

/*
 * Receive descriptor, aligned to 16 bytes
 */
struct ftgmac100_rxdes {
	unsigned int	rxdes0;
	unsigned int	rxdes1;
	unsigned int	rxdes2;	/* not used by HW */
	unsigned int	rxdes3;	/* RXBUF_BADR */
} __attribute__ ((aligned(16)));

#define FTGMAC100_RXDES0_VDBC		0x3fff
#define FTGMAC100_RXDES0_MULTICAST	(1 << 16)
#define FTGMAC100_RXDES0_BROADCAST	(1 << 17)
#define FTGMAC100_RXDES0_RX_ERR		(1 << 18)
#define FTGMAC100_RXDES0_CRC_ERR	(1 << 19)
#define FTGMAC100_RXDES0_FTL		(1 << 20)
#define FTGMAC100_RXDES0_RUNT		(1 << 21)
#define FTGMAC100_RXDES0_RX_ODD_NB	(1 << 22)
#define FTGMAC100_RXDES0_FIFO_FULL	(1 << 23)
#define FTGMAC100_RXDES0_PAUSE_OPCODE	(1 << 24)
#define FTGMAC100_RXDES0_PAUSE_FRAME	(1 << 25)
#define FTGMAC100_RXDES0_LRS		(1 << 28)
#define FTGMAC100_RXDES0_FRS		(1 << 29)
#define FTGMAC100_RXDES0_EDORR		(1 << 30)
#define FTGMAC100_RXDES0_RXPKT_RDY	(1 << 31)

#define FTGMAC100_RXDES1_VLANTAG_CI	0xffff
#define FTGMAC100_RXDES1_PROT_MASK	(0x3 << 20)
#define FTGMAC100_RXDES1_PROT_NONIP	(0x0 << 20)
#define FTGMAC100_RXDES1_PROT_IP	(0x1 << 20)
#define FTGMAC100_RXDES1_PROT_TCPIP	(0x2 << 20)
#define FTGMAC100_RXDES1_PROT_UDPIP	(0x3 << 20)
#define FTGMAC100_RXDES1_LLC		(1 << 22)
#define FTGMAC100_RXDES1_DF		(1 << 23)
#define FTGMAC100_RXDES1_VLANTAG_AVAIL	(1 << 24)
#define FTGMAC100_RXDES1_TCP_CHKSUM_ERR	(1 << 25)
#define FTGMAC100_RXDES1_UDP_CHKSUM_ERR	(1 << 26)
#define FTGMAC100_RXDES1_IP_CHKSUM_ERR	(1 << 27)

/* NCSI */

/* NCSI define & structure */
/* NC-SI Command Packet */
typedef struct {
/* Ethernet Header */
	unsigned char  DA[6];
	unsigned char  SA[6];
	unsigned short EtherType;	/*DMTF NC-SI */
/* NC-SI Control Packet */
	/* Management Controller should set this field to 0x00 */
	unsigned char  MC_ID;
	/* For NC-SI 1.0 spec, this field has to set 0x01 */
	unsigned char  Header_Revision;
	unsigned char  Reserved_1; /* Reserved has to set to 0x00 */
	unsigned char  IID;	/* Instance ID */
	unsigned char  Command;
	unsigned char  Channel_ID;
	/* Payload Length = 12 bits, 4 bits are reserve */
	unsigned short Payload_Length;
	unsigned long  Reserved_2;
	unsigned long  Reserved_3;
} NCSI_Command_Packet;

/* Command and Response Type */
#define CLEAR_INITIAL_STATE     0x00
#define SELECT_PACKAGE        0x01
#define DESELECT_PACKAGE      0x02
#define ENABLE_CHANNEL        0x03
#define DISABLE_CHANNEL       0x04
#define RESET_CHANNEL       0x05
#define ENABLE_CHANNEL_NETWORK_TX   0x06
#define DISABLE_CHANNEL_NETWORK_TX    0x07
#define AEN_ENABLE        0x08
#define SET_LINK        0x09
#define GET_LINK_STATUS       0x0A
#define SET_VLAN_FILTER       0x0B
#define ENABLE_VLAN       0x0C
#define DISABLE_VLAN        0x0D
#define SET_MAC_ADDRESS       0x0E
#define ENABLE_BROADCAST_FILTERING    0x10
#define DISABLE_BROADCAST_FILTERING   0x11
#define ENABLE_GLOBAL_MULTICAST_FILTERING 0x12
#define DISABLE_GLOBAL_MULTICAST_FILTERING  0x13
#define SET_NCSI_FLOW_CONTROL     0x14
#define GET_VERSION_ID        0x15
#define GET_CAPABILITIES      0x16
#define GET_PARAMETERS        0x17
#define GET_CONTROLLER_PACKET_STATISTICS  0x18
#define GET_NCSI_STATISTICS     0x19
#define GET_NCSI_PASS_THROUGH_STATISTICS  0x1A

#define MAX_NCSI_DATA_PAYLOAD 1480  /* for getting the size of the nc-si control data packet */
 /* max ethernet frame size = 1518 */
 /* ethernet headr (14) + nc-si header (16) + nc-si payload (1480) + nc-si checksum (4) + 4 (FCS) = 1518*/

/* NC-SI Response Packet */
typedef struct {
	unsigned char  DA[6];
	unsigned char  SA[6];
	unsigned short EtherType; /* DMTF NC-SI */
/* NC-SI Control Packet */
	/* Management Controller should set this field to 0x00 */

/* 16 bytes NC-SI header */
	unsigned char  MC_ID;
	/* For NC-SI 1.0 spec, this field has to set 0x01 */
	unsigned char  Header_Revision;
	unsigned char  Reserved_1; /* Reserved has to set to 0x00 */
	unsigned char  IID; /* Instance ID */
	unsigned char  Command;
	unsigned char  Channel_ID;
	/* Payload Length = 12 bits, 4 bits are reserved */
	unsigned short Payload_Length;
	unsigned short  Reserved_2;
	unsigned short  Reserved_3;
	unsigned short  Reserved_4;
	unsigned short  Reserved_5;
/* end of NC-SI header */
	unsigned short  Response_Code;
	unsigned short  Reason_Code;
	unsigned char   Payload_Data[MAX_NCSI_DATA_PAYLOAD-4]; /* Exclude resp/reason code */
} __attribute__((packed)) NCSI_Response_Packet;


/* Standard Response Code */
#define COMMAND_COMPLETED     0x00
#define COMMAND_FAILED        0x01
#define COMMAND_UNAVAILABLE     0x02
#define COMMAND_UNSUPPORTED     0x03

/* Standard Reason Code */
#define NO_ERROR        0x0000
#define INTERFACE_INITIALIZATION_REQUIRED 0x0001
#define PARAMETER_IS_INVALID      0x0002
#define CHANNEL_NOT_READY     0x0003
#define PACKAGE_NOT_READY     0x0004
#define INVALID_PAYLOAD_LENGTH      0x0005
#define UNKNOWN_COMMAND_TYPE      0x7FFF

typedef struct {
/* Ethernet Header */
	unsigned char  DA[6];
	unsigned char  SA[6]; /* Network Controller SA = FF:FF:FF:FF:FF:FF */
	unsigned short EtherType; /* DMTF NC-SI */
/* AEN Packet Format */
	/* AEN HEADER */
	/* Network Controller should set this field to 0x00 */
	unsigned char  MC_ID;
	/* For NC-SI 1.0 spec, this field has to set 0x01 */
	unsigned char  Header_Revision;
	unsigned char  Reserved_1; /* Reserved has to set to 0x00 */
	unsigned char  IID;        /* Instance ID = 0 in AEN */
	unsigned char  Command;    /* AEN = 0xFF */
	unsigned char  Channel_ID;
	unsigned short Payload_Length; /* Payload Length = 4 in Network Controller AEN Packet */
	unsigned long  Reserved_2;
	unsigned long  Reserved_3;
/* end of  AEN HEADER */
	unsigned char  Reserved_4[3];
	unsigned char  AEN_Type;
	unsigned char  Optional_AEN_Data[64];
} __attribute__((packed)) AEN_Packet;

/* AEN Type */
#define AEN_TYPE_LINK_STATUS_CHANGE           0x0
#define AEN_TYPE_CONFIGURATION_REQUIRED       0x1
#define AEN_TYPE_HOST_NC_DRIVER_STATUS_CHANGE 0x2
#define AEN_TYPE_OEM                          0x80


typedef struct {
	unsigned char Package_ID;
	unsigned char Channel_ID;
	unsigned long Capabilities_Flags;
	unsigned long Broadcast_Packet_Filter_Capabilities;
	unsigned long Multicast_Packet_Filter_Capabilities;
	unsigned long Buffering_Capabilities;
	unsigned long AEN_Control_Support;
} NCSI_Capability;
NCSI_Capability NCSI_Cap;

/* SET_MAC_ADDRESS */
#define UNICAST   (0x00 << 5)
#define MULTICAST_ADDRESS (0x01 << 5)
#define DISABLE_MAC_ADDRESS_FILTER  0x00
#define ENABLE_MAC_ADDRESS_FILTER 0x01

/* GET_LINK_STATUS */
#define LINK_DOWN 0
#define LINK_UP   1

#define NCSI_LOOP   100
#define RETRY_COUNT     1

/* Reversed because of 0x88 is low byte, 0xF8 is high byte in memory */
#define NCSI_HEADER 0xF888

#endif /* __FTGMAC100_H */
