/*
 * ASPEED Gigabit Ethernet
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2013.09.26: Initial version [Ryan Chen]
 */

#ifndef __AST_GMAC_H
#define __AST_GMAC_H

#define AST_GMAC_ISR		0x00
#define AST_GMAC_IER		0x04
#define AST_GMAC_MAC_MADR	0x08
#define AST_GMAC_MAC_LADR	0x0c
#define AST_GMAC_MAHT0		0x10
#define AST_GMAC_MAHT1		0x14
#define AST_GMAC_NPTXPD		0x18
#define AST_GMAC_RXPD		0x1c
#define AST_GMAC_NPTXR_BADR	0x20
#define AST_GMAC_RXR_BADR	0x24
#define AST_GMAC_HPTXPD		0x28
#define AST_GMAC_HPTXR_BADR	0x2c
#define AST_GMAC_ITC		0x30
#define AST_GMAC_APTC		0x34
#define AST_GMAC_DBLAC		0x38
#define AST_GMAC_DMAFIFOS	0x3c
#define AST_GMAC_REVR		0x40
#define AST_GMAC_FEAR		0x44
#define AST_GMAC_TPAFCR		0x48
#define AST_GMAC_RBSR		0x4c
#define AST_GMAC_MACCR		0x50
#define AST_GMAC_MACSR		0x54
#define AST_GMAC_TM		0x58
#define AST_GMAC_PHYCR		0x60
#define AST_GMAC_PHYDATA	0x64
#define AST_GMAC_FCR		0x68
#define AST_GMAC_BPR		0x6c
#define AST_GMAC_WOLCR		0x70
#define AST_GMAC_WOLSR		0x74
//#define AST_GMAC_WFCRC		0x78
#define AST_GMAC_WFBM1M		0x78
#define AST_GMAC_WFBM1L		0x7C
#define AST_GMAC_WFBM2M		0x80
#define AST_GMAC_WFBM2L		0x84
#define AST_GMAC_WFBM3M		0x88
#define AST_GMAC_WFBM3L		0x8C
//#define AST_GMAC_WFBM4		0x8c
#define AST_GMAC_NPTXR_PTR	0x90
#define AST_GMAC_HPTXR_PTR	0x94
#define AST_GMAC_RXR_PTR	0x98
#define AST_GMAC_TX		0xa0
#define AST_GMAC_TX_MCOL_SCOL	0xa4
#define AST_GMAC_TX_ECOL_FAIL	0xa8
#define AST_GMAC_TX_LCOL_UND	0xac
#define AST_GMAC_RX		0xb0
#define AST_GMAC_RX_BC		0xb4
#define AST_GMAC_RX_MC		0xb8
#define AST_GMAC_RX_PF_AEP	0xbc
#define AST_GMAC_RX_RUNT	0xc0
#define AST_GMAC_RX_CRCER_FTL	0xc4
#define AST_GMAC_RX_COL_LOST	0xc8

/*
 * Interrupt status register & interrupt enable register
 */
#define GMAC_INT_RPKT_BUF		(1 << 0)
#define GMAC_INT_RPKT_FIFO		(1 << 1)
#define GMAC_INT_NO_RXBUF		(1 << 2)
#define GMAC_INT_RPKT_LOST		(1 << 3)
#define GMAC_INT_XPKT_ETH		(1 << 4)
#define GMAC_INT_XPKT_FIFO		(1 << 5)
#define GMAC_INT_NO_NPTXBUF	(1 << 6)
#define GMAC_INT_XPKT_LOST		(1 << 7)
#define GMAC_INT_AHB_ERR		(1 << 8)
#define GMAC_INT_PHYSTS_CHG	(1 << 9)
#define GMAC_INT_NO_HPTXBUF	(1 << 10)

/*
 * Interrupt timer control register
 */
#define GMAC_ITC_RXINT_CNT(x)	(((x) & 0xf) << 0)
#define GMAC_ITC_RXINT_THR(x)	(((x) & 0x7) << 4)
#define GMAC_ITC_RXINT_TIME_SEL	(1 << 7)
#define GMAC_ITC_TXINT_CNT(x)	(((x) & 0xf) << 8)
#define GMAC_ITC_TXINT_THR(x)	(((x) & 0x7) << 12)
#define GMAC_ITC_TXINT_TIME_SEL	(1 << 15)

/*
 * Automatic polling timer control register
 */
#define GMAC_APTC_RXPOLL_CNT(x)	(((x) & 0xf) << 0)
#define GMAC_APTC_RXPOLL_TIME_SEL	(1 << 4)
#define GMAC_APTC_TXPOLL_CNT(x)	(((x) & 0xf) << 8)
#define GMAC_APTC_TXPOLL_TIME_SEL	(1 << 12)

/*
 * DMA burst length and arbitration control register
 */
#define GMAC_DBLAC_RXFIFO_LTHR(x)	(((x) & 0x7) << 0)
#define GMAC_DBLAC_RXFIFO_HTHR(x)	(((x) & 0x7) << 3)
#define GMAC_DBLAC_RX_THR_EN	(1 << 6)
#define GMAC_DBLAC_RXBURST_SIZE(x)	(((x) & 0x3) << 8)
#define GMAC_DBLAC_TXBURST_SIZE(x)	(((x) & 0x3) << 10)
#define GMAC_DBLAC_RXDES_SIZE(x)	(((x) & 0xf) << 12)
#define GMAC_DBLAC_TXDES_SIZE(x)	(((x) & 0xf) << 16)
#define GMAC_DBLAC_IFG_CNT(x)	(((x) & 0x7) << 20)
#define GMAC_DBLAC_IFG_INC		(1 << 23)

/*
 * DMA FIFO status register
 */
#define GMAC_DMAFIFOS_RXDMA1_SM(dmafifos)	((dmafifos) & 0xf)
#define GMAC_DMAFIFOS_RXDMA2_SM(dmafifos)	(((dmafifos) >> 4) & 0xf)
#define GMAC_DMAFIFOS_RXDMA3_SM(dmafifos)	(((dmafifos) >> 8) & 0x7)
#define GMAC_DMAFIFOS_TXDMA1_SM(dmafifos)	(((dmafifos) >> 12) & 0xf)
#define GMAC_DMAFIFOS_TXDMA2_SM(dmafifos)	(((dmafifos) >> 16) & 0x3)
#define GMAC_DMAFIFOS_TXDMA3_SM(dmafifos)	(((dmafifos) >> 18) & 0xf)
#define GMAC_DMAFIFOS_RXFIFO_EMPTY		(1 << 26)
#define GMAC_DMAFIFOS_TXFIFO_EMPTY		(1 << 27)
#define GMAC_DMAFIFOS_RXDMA_GRANT		(1 << 28)
#define GMAC_DMAFIFOS_TXDMA_GRANT		(1 << 29)
#define GMAC_DMAFIFOS_RXDMA_REQ		(1 << 30)
#define GMAC_DMAFIFOS_TXDMA_REQ		(1 << 31)

/*
 * Receive buffer size register
 */
#define GMAC_RBSR_SIZE(x)		((x) & 0x3fff)

/*
 * MAC control register
 */
#define GMAC_MACCR_TXDMA_EN	(1 << 0)
#define GMAC_MACCR_RXDMA_EN	(1 << 1)
#define GMAC_MACCR_TXMAC_EN	(1 << 2)
#define GMAC_MACCR_RXMAC_EN	(1 << 3)
#define GMAC_MACCR_RM_VLAN		(1 << 4)
#define GMAC_MACCR_HPTXR_EN	(1 << 5)
//#define GMAC_MACCR_LOOP_EN		(1 << 6)
#define GMAC_MACCR_ENRX_IN_HALFTX	(1 << 7)
#define GMAC_MACCR_FULLDUP		(1 << 8)
#define GMAC_MACCR_GIGA_MODE	(1 << 9)
#define GMAC_MACCR_CRC_APD		(1 << 10)
#define GMAC_MACCR_LOW_SEN		(1 << 11)	//new
#define GMAC_MACCR_RX_RUNT		(1 << 12)
#define GMAC_MACCR_JUMBO_LF	(1 << 13)
#define GMAC_MACCR_RX_ALL		(1 << 14)
#define GMAC_MACCR_HT_MULTI_EN	(1 << 15)
#define GMAC_MACCR_RX_MULTIPKT	(1 << 16)
#define GMAC_MACCR_RX_BROADPKT	(1 << 17)
#define GMAC_MACCR_DISCARD_CRCERR	(1 << 18)
#define GMAC_MACCR_FAST_MODE	(1 << 19)
#define GMAC_MACCR_SW_RST		(1 << 31)

/*
 * PHY control register
 */
#define GMAC_PHYCR_MDC_CYCTHR_MASK	0x3f
#define GMAC_PHYCR_MDC_CYCTHR(x)	((x) & 0x3f)
#define GMAC_PHYCR_PHYAD(x)	(((x) & 0x1f) << 16)
#define GMAC_PHYCR_REGAD(x)	(((x) & 0x1f) << 21)
#define GMAC_PHYCR_MIIRD		(1 << 26)
#define GMAC_PHYCR_MIIWR		(1 << 27)

/*
 * PHY data register
 */
#define GMAC_PHYDATA_MIIWDATA(x)		((x) & 0xffff)
#define GMAC_PHYDATA_MIIRDATA(phydata)	(((phydata) >> 16) & 0xffff)

/*
 * Transmit descriptor, aligned to 16 bytes
 */
struct ast_gmac_txdes {
	unsigned int	txdes0;
	unsigned int	txdes1;
	unsigned int	txdes2;	/* not used by HW */
	unsigned int	txdes3;	/* TXBUF_BADR */
} __attribute__ ((aligned(16)));

#define GMAC_TXDES0_TXBUF_SIZE(x)	((x) & 0x3fff)
//#define GMAC_TXDES0_EDOTR		(1 << 15)
#define GMAC_TXDES0_CRC_ERR	(1 << 19)
#define GMAC_TXDES0_LTS		(1 << 28)
#define GMAC_TXDES0_FTS		(1 << 29)
#define GMAC_TXDES0_EDOTR		(1 << 30) //org is 15 ->30
#define GMAC_TXDES0_TXDMA_OWN	(1 << 31)

#define GMAC_TXDES1_VLANTAG_CI(x)	((x) & 0xffff)
#define GMAC_TXDES1_INS_VLANTAG	(1 << 16)
//#define GMAC_TXDES1_TCP_CHKSUM	(1 << 17)
//#define GMAC_TXDES1_UDP_CHKSUM	(1 << 18)
//#define GMAC_TXDES1_IP_CHKSUM	(1 << 19)
#define GMAC_TXDES1_LLC		(1 << 22)
#define GMAC_TXDES1_TX2FIC		(1 << 30)
#define GMAC_TXDES1_TXIC		(1 << 31)

/*
 * Receive descriptor, aligned to 16 bytes
 */
struct ast_gmac_rxdes {
	unsigned int	rxdes0;
	unsigned int	rxdes1;
	unsigned int	rxdes2;	/* not used by HW */
	unsigned int	rxdes3;	/* RXBUF_BADR */
} __attribute__ ((aligned(16)));

#define GMAC_RXDES0_VDBC		0x3fff
//#define GMAC_RXDES0_EDORR		(1 << 15)
#define GMAC_RXDES0_MULTICAST	(1 << 16)
#define GMAC_RXDES0_BROADCAST	(1 << 17)
#define GMAC_RXDES0_RX_ERR		(1 << 18)
#define GMAC_RXDES0_CRC_ERR	(1 << 19)
#define GMAC_RXDES0_FTL		(1 << 20)
#define GMAC_RXDES0_RUNT		(1 << 21)
#define GMAC_RXDES0_RX_ODD_NB	(1 << 22)
#define GMAC_RXDES0_FIFO_FULL	(1 << 23)
#define GMAC_RXDES0_PAUSE_OPCODE	(1 << 24)
#define GMAC_RXDES0_PAUSE_FRAME	(1 << 25)
#define GMAC_RXDES0_LRS		(1 << 28)
#define GMAC_RXDES0_FRS		(1 << 29)
#define GMAC_RXDES0_EDORR		(1 << 30)	//org 15->30
#define GMAC_RXDES0_RXPKT_RDY	(1 << 31)

#define GMAC_RXDES1_VLANTAG_CI	0xffff
#define GMAC_RXDES1_PROT_MASK	(0x3 << 20)
#define GMAC_RXDES1_PROT_NONIP	(0x0 << 20)
#define GMAC_RXDES1_PROT_IP	(0x1 << 20)
#define GMAC_RXDES1_PROT_TCPIP	(0x2 << 20)
#define GMAC_RXDES1_PROT_UDPIP	(0x3 << 20)
#define GMAC_RXDES1_LLC		(1 << 22)
#define GMAC_RXDES1_DF		(1 << 23)
#define GMAC_RXDES1_VLANTAG_AVAIL	(1 << 24)
#define GMAC_RXDES1_TCP_CHKSUM_ERR	(1 << 25)
#define GMAC_RXDES1_UDP_CHKSUM_ERR	(1 << 26)
#define GMAC_RXDES1_IP_CHKSUM_ERR	(1 << 27)

#endif /* __GMAC_H */
