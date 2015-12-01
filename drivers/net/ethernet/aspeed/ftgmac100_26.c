/********************************************************************************
* File Name     : ftgmac100_26.c
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
********************************************************************************/
//-----------------------------------------------------------------------------
//   AST2500 LAN Driver
//1. based on AST2400 LAN driver 06/25/2013
//2. 07/29/2014 - by CC@aspeed
//   Support RTL8211FD (EVB MAC#1 RGMII) and KSZ9031(EVB MAC#2 RGMII)
//   Support RTL8201FD (EVB MAC#1 RMII) and KSZ8081(EVB MAC#2 RMII)
//3. 08/05/2014 - by CC@aspeed
//   Support NC-SI interface, not yet verified
//4. 09/24/2014 - by CC@aspeed
//   Fixed LAN is not workable at 100M half-duplex and RTL8211F
//5. 11/06/2014 - by CC@aspeed
//   Support set mac address function
//6. 02/26/2015 - by CC@aspeed
//   Add NAPI function
//   Add free Rx buffer function when ast_gmac_stop()
//-----------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/ip.h>

#include <linux/skbuff.h>
#include <linux/dma-mapping.h>

#include "ftgmac100_26.h"

#if defined(CONFIG_ARM)
#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <mach/ftgmac100_drv.h>


#elif defined(CONFIG_COLDFIRE)
#include <asm/arch/ftgmac100_drv.h>

#else
#err "Not define include for GMAC"
#endif

/*------------------------------------------------------------------------
 .
 . Configuration options, for the experienced user to change.
 .
 -------------------------------------------------------------------------*/
/* Transmitter timeout */
#define TX_TIMEOUT (1 * HZ)

/*
 . DEBUGGING LEVELS
 .
 . 0 for normal operation
 . 1 for slightly more details
 . 2 for various levels of increasingly useless information
 .    2 for interrupt tracking, status flags
 .    3 for packet info
 .    4 for complete packet dumps
*/

#define DO_PRINT(args...) printk(": " args)

#define FTMAC100_DEBUG  0

#if (FTMAC100_DEBUG > 2 )
    #define PRINTK3(args...) DO_PRINT(args)
#else
    #define PRINTK3(args...)
#endif

#if ( FTMAC100_DEBUG > 1 )
    #define PRINTK2(args...) DO_PRINT(args)
#else
    #define PRINTK2(args...)
#endif

#if ( FTMAC100_DEBUG > 0 )
    #define PRINTK(args...) DO_PRINT(args)
#else
    #define PRINTK(args...)
#endif

/*
 . A rather simple routine to print out a packet for debugging purposes.
*/
#if FTMAC100_DEBUG > 2
static void print_packet( u8 *, int );
#endif

static int ftgmac100_wait_to_send_packet(struct sk_buff * skb, struct net_device * dev);
static volatile int trans_busy[2] = { 0, 0 };

void ftgmac100_phy_rw_waiting(unsigned int ioaddr)
{
    unsigned int tmp;

#ifdef NEW_MDCMDIO_INTERFACE  
    do {
        mdelay(10);
        tmp = ioread32( ioaddr + PHYCR_REG );
    } while ( ( tmp & MAC_PHYBUSY_NEW ) );
#else
    do {
        mdelay(10);
        tmp =ioread32( ioaddr + PHYCR_REG );
    } while ( ( tmp & (PHY_READ_bit | PHY_WRITE_bit) ) > 0 );
#endif       
}


/*------------------------------------------------------------
 . Reads a register from the MII Management serial interface
 .-------------------------------------------------------------*/
static u16 ftgmac100_read_phy_register(unsigned int ioaddr, u8 phyaddr, u8 phyreg)
{
    unsigned int tmp;

    if (phyaddr > 0x1f)	// MII chip IDs are 5 bits long
        return 0xffff;
    
#ifdef NEW_MDCMDIO_INTERFACE    
    tmp  = MAC_PHYRD_NEW;
    tmp |= ( ( phyaddr & 0x1f ) << 5 );
    tmp |= ( phyreg );
    iowrite32( tmp, ioaddr + PHYCR_REG );
    ftgmac100_phy_rw_waiting( ioaddr );

    return ( (u16) (ioread32( ioaddr + PHYDATA_REG ) & 0x0000ffff) );
#else    
    tmp = ioread32( ioaddr + PHYCR_REG );
    tmp &= 0x3000003F;
    tmp |= ( phyaddr << 16 );
    tmp |= ( phyreg << (16 + 5) );
    tmp |= PHY_READ_bit;
    iowrite32( tmp, ioaddr + PHYCR_REG );
    ftgmac100_phy_rw_waiting( ioaddr );

    return ( ioread32(ioaddr + PHYDATA_REG) >> 16 );
#endif        
}


/*------------------------------------------------------------
 . Writes a register to the MII Management serial interface
 .-------------------------------------------------------------*/
static void ftgmac100_write_phy_register(unsigned int ioaddr,
                                              u8 phyaddr, u8 phyreg, u16 phydata)
{
    unsigned int tmp;

    if (phyaddr > 0x1f)	// MII chip IDs are 5 bits long
        return;

#ifdef NEW_MDCMDIO_INTERFACE 
    tmp  = MAC_PHYWR_NEW;
    tmp |= ( phydata << 16 );
    tmp |= ( ( phyaddr & 0x1f ) << 5  );
    tmp |= ( phyreg );
    iowrite32( tmp, ioaddr + PHYCR_REG );
    ftgmac100_phy_rw_waiting( ioaddr );
#else
    tmp = ioread32( ioaddr + PHYCR_REG );
    tmp &= 0x3000003F;
    tmp |= ( phyaddr << 16 );
    tmp |= ( phyreg << (16 + 5) );
    tmp |= PHY_WRITE_bit;

    iowrite32( phydata, ioaddr + PHYDATA_REG );
    iowrite32( tmp, ioaddr + PHYCR_REG );
    ftgmac100_phy_rw_waiting( ioaddr );
#endif     
}

static void ast_gmac_set_mac(struct ftgmac100_priv *priv, const unsigned char *mac)
{
    unsigned int maddr = mac[0] << 8 | mac[1];
    unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];
    
    iowrite32(maddr, priv->netdev->base_addr + MAC_MADR_REG);
    iowrite32(laddr, priv->netdev->base_addr + MAC_LADR_REG);
}

/*
 * MAC1 always has MII MDC+MDIO pins to access PHY registers.  We assume MAC1
 * always has a PHY chip, if MAC1 is enabled.
 * U-Boot can enable MAC2 MDC+MDIO pins for a 2nd PHY, or MAC2 might be
 * disabled (only one port), or it's sideband-RMII which has no PHY chip.
 *
 * Return miiPhyId==0 if the MAC cannot be accessed.
 * Return miiPhyId==1 if the MAC registers are OK but it cannot carry traffic.
 * Return miiPhyId==2 if the MAC can send/receive but it has no PHY chip.
 * Else return the PHY 22-bit vendor ID, 6-bit model and 4-bit revision.
 */
static void getMacHwConfig( struct net_device* dev, struct AstMacHwConfig* out )
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;

    out->miiPhyId = 0;

    // We assume the Clock Stop register does not disable the MAC1 or MAC2 clock
    // unless Reset Control also holds the MAC in reset.
    // For now, we only support a PHY chip on the MAC's own MDC+MDIO bus.
      
    for ( out->phyAddr = 0; out->phyAddr < 32; out->phyAddr++ )
    {
        out->miiPhyId = ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x02);
        if ( out->miiPhyId == 0xFFFF )
            continue;
        else
        {
            out->miiPhyId = ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x02);
            out->miiPhyId = (out->miiPhyId & 0xffff) << 16;
            out->miiPhyId |= ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x03) & 0xffff;
            
            PRINTK("%s:getMacHwConfig, phyAddr=0x%x, miiPhyId=0x%04x_%04x\n",
                        dev->name, out->phyAddr, (out->miiPhyId >> 16), (out->miiPhyId & 0xffff));
            break;
        }
    }
    
    if ( out->phyAddr >= 32 )
    {
        PRINTK("%s:getMacHwConfig, cannot find PHY, Please check MDC and MDIO bus\n",
                        dev->name);
    }

    return;
}


static void ftgmac100_reset( struct net_device* dev )
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    struct AstMacHwConfig* ids  = &priv->ids;
    unsigned int tmp;
    unsigned int speed;
    unsigned int duplex;
    unsigned int cnt;

#ifdef NEW_MDCMDIO_INTERFACE
    PRINTK ("--> Use New MDC and MDIO interface...[%d].\n", ids->macId);
    iowrite32( 0x80000000, dev->base_addr + FEAR_REG0); // SW to new interface
#else
    PRINTK ("--> Use Old MDC and MDIO interface...[%d].\n", ids->macId);
    iowrite32( 0x00000000, dev->base_addr + FEAR_REG0);
#endif        
    
    getMacHwConfig(dev, ids);
    PRINTK("%s:ftgmac100_reset, phyAddr=0x%x, miiPhyId=0x%04x_%04x\n",
                dev->name, ids->phyAddr, (ids->miiPhyId >> 16), (ids->miiPhyId & 0xffff));
    
    if (ids->miiPhyId < 1)
        return;	// Cannot access MAC registers
    
    // Check the link speed and duplex.
    // They are not valid until auto-neg is resolved, which is reg.1 bit[5],
    // or the link is up, which is reg.1 bit[2].
    
    if (ids->phyAddr < 0xff)
        tmp = ftgmac100_read_phy_register(dev->base_addr, ids->phyAddr, 0x1);
    else 
        tmp = 0;
    
    if (ids->phyAddr >= 0xff) {
        // No PHY chip, or link has not negotiated.
        speed  = PHY_SPEED_100M;
        duplex = 1;
        netif_carrier_off(dev);
    }
    else if ((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ9031) {
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1F);
        if ( (tmp & 0x0008) == 0x0008 )
            duplex = 1;
        else
            duplex = 0;
                                   
        switch(tmp & 0x0070) {
            case 0x0040:
                speed = PHY_SPEED_1G; 
                break;
            case 0x0020:    
                speed = PHY_SPEED_100M; 
                break;
            case 0x0010:    
                speed = PHY_SPEED_10M;    
                break;
            default:
                speed = PHY_SPEED_100M;
        }               
    }
    else if (ids->miiPhyId == PHYID_RTL8211FD) {
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1A);
        if ( (tmp & 0x0008) == 0x0008 )
            duplex = 1;
        else
            duplex = 0;
                                   
        switch(tmp & 0x0030) {
            case 0x0020:
                speed = PHY_SPEED_1G; 
                break;
            case 0x0010:    
                speed = PHY_SPEED_100M; 
                break;
            case 0x0000:    
                speed = PHY_SPEED_10M;    
                break;
            default:
                speed = PHY_SPEED_100M;
        }       
    }
    else if ((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ8081) {
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1E);
        switch(tmp & 0x0007) 
        {
            case 0x0000:
                // Still in auto-negotiation
                // Retry again
                break;
            case 0x0001:
                speed  = PHY_SPEED_10M; 
                duplex = 0;
                break;
            case 0x0002:
                speed = PHY_SPEED_100M; 
                duplex = 0;
                break;                    
            case 0x0005:
                speed  = PHY_SPEED_10M; 
                duplex = 1;
                break;
            case 0x0006:    
                speed = PHY_SPEED_100M; 
                duplex = 1;
                break;
            default:
                speed = PHY_SPEED_100M;
        }         
    }
    else if (ids->miiPhyId == PHYID_RTL8201F) {
        // First check autonegotiation complete
        for ( cnt = 0; cnt < 10; cnt++ )
        {
            tmp = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1);                                   
            if ( (tmp & 0x0020) == 0x0020 ) // register 1 bit 5, Autonegotiation Complete
                break;
        }
        if ( cnt < 10 )
        {
            // autonegotiation process completed
            // Get speed and duplex status
            tmp = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x0);
            if ( ( tmp & 0x2000) == 0x2000 )
                speed = PHY_SPEED_100M; 
            else
                speed  = PHY_SPEED_10M; 
                
            if ( ( tmp & 0x0100) == 0x0100 )
                duplex = 1;
            else
                duplex = 0; 
        }
        else
        {
            // autonegotiation process not complete
            PRINTK ("Auto negotiation process not complete (RTL8201F)\n");
        }         
    }    
    else if (ids->miiPhyId == PHYID_RTL8201EL) {
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x00);
        duplex = (tmp & 0x0100) ? 1 : 0;
        speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
    }
    else if (((ids->miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) || 
             (ids->miiPhyId == PHYID_RTL8211E)                               ) {
        // Use reg.17_0.bit[15:13] for {speed[1:0], duplex}.
        tmp    = ftgmac100_read_phy_register(dev->base_addr, ids->phyAddr, 0x11);
        duplex = (tmp & PHY_DUPLEX_mask) >> 13;
        speed  = (tmp & PHY_SPEED_mask)  >> 14;
    }
    else if (priv->ids.miiPhyId == PHYID_BCM54612E) { 
       // Get link status
        // First Switch shadow register selector
        ftgmac100_write_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1C, 0x2000);
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1C);
        if ( (tmp & 0x0080) == 0x0080 )
            duplex = 0;
        else
            duplex = 1;
        
        switch(tmp & 0x0018) {
            case 0x0000:
                speed = PHY_SPEED_1G; break;
            case 0x0008:    
                speed = PHY_SPEED_100M; break;
            case 0x0010:    
                speed = PHY_SPEED_10M; break;
            default:
                speed = PHY_SPEED_100M;
        } 
    }
    else {
        // Assume Broadcom BCM5221.  Use reg.18 bits [1:0] for {100Mb/s, fdx}.
        tmp    = ftgmac100_read_phy_register(dev->base_addr, ids->phyAddr, 0x18);
        duplex = (tmp & 0x0001);
        speed  = (tmp & 0x0002) ? PHY_SPEED_100M : PHY_SPEED_10M;
    }

    if (speed == PHY_SPEED_1G) {
        // Set SPEED_100_bit too, for consistency.
        priv->maccr_val |= GMAC_MODE_bit | SPEED_100_bit;
        
        tmp =  ioread32( dev->base_addr + MACCR_REG );
        tmp |= GMAC_MODE_bit | SPEED_100_bit;
        iowrite32(tmp, dev->base_addr + MACCR_REG );
    } else {
        priv->maccr_val &= ~(GMAC_MODE_bit | SPEED_100_bit);
        
        tmp =  ioread32( dev->base_addr + MACCR_REG );
        tmp &= ~(GMAC_MODE_bit | SPEED_100_bit);
        if (speed == PHY_SPEED_100M) {
            priv->maccr_val |= SPEED_100_bit;
            tmp             |= SPEED_100_bit;
        }
        iowrite32(tmp, dev->base_addr + MACCR_REG );
    }
    if ( duplex )
        priv->maccr_val |= FULLDUP_bit;
    else 
        priv->maccr_val &= ~FULLDUP_bit;
    
    // Do software reset MAC
    tmp = ioread32( dev->base_addr + MACCR_REG ) | SW_RST_bit;
    iowrite32( tmp, dev->base_addr + MACCR_REG );

#ifdef not_complete_yet
    /* Setup for fast accesses if requested */
    /* If the card/system can't handle it then there will */
    /* be no recovery except for a hard reset or power cycle */
    if (dev->dma)
    {
        outw( inw( dev->base_addr + CONFIG_REG ) | CONFIG_NO_WAIT,
        dev->base_addr + CONFIG_REG );
    }
#endif /* end_of_not */

    /* this should pause enough for the chip to be happy */
    for (; (ioread32( dev->base_addr + MACCR_REG ) & SW_RST_bit) != 0; )
    {
        mdelay(10);
        PRINTK3("RESET: reset not complete yet\n" );
    }

    iowrite32( 0, dev->base_addr + IER_REG );			/* Disable all interrupts */
}

static void ftgmac100_enable( struct net_device *dev )
{
    int i;
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    unsigned int           tmp_rsize;
    unsigned int           rfifo_rsize;
    unsigned int           tfifo_rsize;
    unsigned int           rxbuf_size;

    rxbuf_size = RX_BUF_SIZE & 0x3fff;
    iowrite32( rxbuf_size , dev->base_addr + RBSR_REG); //for NC Body

    for (i=0; i<RXDES_NUM; ++i)
        priv->rx_descs[i].RXPKT_RDY = RX_OWNBY_FTGMAC100;   // owned by FTMAC100

    priv->rx_idx = 0;

    for (i=0; i<TXDES_NUM; ++i) {
        priv->tx_descs[i].TXDMA_OWN = TX_OWNBY_SOFTWARE;    // owned by software
        priv->tx_skbuff[i] = 0;
    }

    priv->tx_idx = 0;
    priv->old_tx = 0;

   
    /* Set the MAC address */
    ast_gmac_set_mac(priv, dev->dev_addr);

    iowrite32( priv->rx_descs_dma, dev->base_addr + RXR_BADR_REG);
    iowrite32( priv->tx_descs_dma, dev->base_addr + TXR_BADR_REG);
    iowrite32( 0x00001010, dev->base_addr + ITC_REG);

    iowrite32( (0UL<<TXPOLL_CNT)|(0x1<<RXPOLL_CNT), dev->base_addr + APTC_REG);
    iowrite32( 0x44f97, dev->base_addr + DBLAC_REG );

    /// iowrite32( ioread32(FCR_REG)|0x1, ioaddr + FCR_REG );     // enable flow control
    /// iowrite32( ioread32(BPR_REG)|0x1, ioaddr + BPR_REG );     // enable back pressure register

    tmp_rsize = ioread32( dev->base_addr + FEAR_REG1 );
    rfifo_rsize = tmp_rsize & 0x00000007;
    tfifo_rsize = (tmp_rsize >> 3)& 0x00000007;

    tmp_rsize = ioread32( dev->base_addr + TPAFCR_REG );
    tmp_rsize &= ~0x3f000000;
    tmp_rsize |= (tfifo_rsize << 27);
    tmp_rsize |= (rfifo_rsize << 24);

    iowrite32(tmp_rsize, dev->base_addr + TPAFCR_REG);

//river set MAHT0, MAHT1
    if (priv->maccr_val & GMAC_MODE_bit) {
        iowrite32 (priv->GigaBit_MAHT0, dev->base_addr + MAHT0_REG);
        iowrite32 (priv->GigaBit_MAHT1, dev->base_addr + MAHT1_REG);
    }
    else {
        iowrite32 (priv->Not_GigaBit_MAHT0, dev->base_addr + MAHT0_REG);
        iowrite32 (priv->Not_GigaBit_MAHT1, dev->base_addr + MAHT1_REG);
    }

    /// enable trans/recv,...
    iowrite32(priv->maccr_val, dev->base_addr + MACCR_REG );

	/* now, enable interrupts */
    if ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ9031) {
        priv->EnInt_reg = (
// no link PHY link status pin            PHYSTS_CHG_bit      |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit         |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            );
    }
    else if (priv->ids.miiPhyId == PHYID_RTL8211FD) {
        priv->EnInt_reg = (
// no link PHY link status pin            PHYSTS_CHG_bit      |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit         |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            );
    }
    else if ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ8081) {
        priv->EnInt_reg = (
// no link PHY link status pin            PHYSTS_CHG_bit      |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit         |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            );
    } 
    else if (priv->ids.miiPhyId == PHYID_RTL8201F) {
        priv->EnInt_reg = (
// no link PHY link status pin            PHYSTS_CHG_bit      |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit         |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            );
    } 
    else if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK)       == PHYID_VENDOR_MARVELL) ||
              (priv->ids.miiPhyId                            == PHYID_RTL8211E)         ) {
        priv->EnInt_reg = (
//                PHYSTS_CHG_bit		|
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit			|
            RXBUF_UNAVA_bit		|
            RPKT2B_bit
            );
    }
    else if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK)       == PHYID_VENDOR_BROADCOM) ||
             (priv->ids.miiPhyId == PHYID_RTL8201EL)                                     ) {
        priv->EnInt_reg = (
            TPKT2E_bit          |
            TPKT_LOST_bit       |            
            AHB_ERR_bit			|
            RXBUF_UNAVA_bit		|
            RPKT2B_bit
            );
    }
    else if (priv->ids.miiPhyId == PHYID_BCM54612E) { 
        priv->EnInt_reg = (
// no link PHY link status pin            PHYSTS_CHG_bit      |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit         |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            );
    } 
    else {
        priv->EnInt_reg = (
// no link PHY link status pin			  PHYSTS_CHG_bit	  |
            TPKT2E_bit          |
            TPKT_LOST_bit       |
            AHB_ERR_bit 		|
            RXBUF_UNAVA_bit 	|
            RPKT2B_bit
            );    
    }

    iowrite32( priv->EnInt_reg, dev->base_addr + IER_REG );   
}

static void aspeed_mac_timer(unsigned long data)
{
    struct net_device *dev      = (struct net_device *)data;
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
#ifndef AST_SOC_G5	
	unsigned long ioaddr        = dev->base_addr;    
#endif
    unsigned int status;
    unsigned int tmp;
    unsigned int speed;
    unsigned int duplex;
    unsigned int macSpeed;
    unsigned int cnt;
    unsigned int IgnoreFlag = 0;
#ifndef AST_SOC_G5
// for debug 20150703  (AST2300 issue)   
iowrite32( 0xffffffff, ioaddr + TXPD_REG);
#endif
    
    status = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x01);
    // Some PHY need to read the register twice (RTL28201F...)
    status = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x01);
  
    if (status & LINK_STATUS) { // Bit[2], Link Status, link is up
        priv->timer.expires = jiffies + 10 * HZ;

        if ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ9031)
        {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1F);
            if ( (tmp & 0x0008) == 0x0008 )
                duplex = 1;
            else
                duplex = 0;
                                   
            switch(tmp & 0x0070) {
                case 0x0040:
                    speed = PHY_SPEED_1G; 
                    break;
                case 0x0020:    
                    speed = PHY_SPEED_100M; 
                    break;
                case 0x0010:    
                    speed = PHY_SPEED_10M;    
                    break;
                default:
                    speed = PHY_SPEED_100M;
            }        
        }
        else if (priv->ids.miiPhyId == PHYID_RTL8211FD)
        {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1A);
            if ( (tmp & 0x0008) == 0x0008 )
                duplex = 1;
            else
                duplex = 0;
                                   
            switch(tmp & 0x0030) {
                case 0x0020:
                    speed = PHY_SPEED_1G;            
                    break;
                case 0x0010:    
                    speed = PHY_SPEED_100M; 
                    break;
                case 0x0000:    
                    speed = PHY_SPEED_10M;                                           
                    break;
                default:
                    speed = PHY_SPEED_100M;
            }        
        }
        else if ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ8081)
        {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1E);                                   
            switch(tmp & 0x0007) {
                case 0x0000:
                    // Still in auto-negotiation
                    // Retry again
                    break;
                case 0x0001:
                    speed  = PHY_SPEED_10M; 
                    duplex = 0;
                    break;
                case 0x0002:
                    speed = PHY_SPEED_100M; 
                    duplex = 0;
                    break;                    
                case 0x0005:
                    speed  = PHY_SPEED_10M; 
                    duplex = 1;
                    break;
                case 0x0006:    
                    speed = PHY_SPEED_100M; 
                    duplex = 1;
                    break;
                default:
                    speed = PHY_SPEED_100M;                 
            }        
             
        }
        else if (priv->ids.miiPhyId == PHYID_RTL8201F)
        {
            // First check autonegotiation complete
            for ( cnt = 0; cnt < 10; cnt++ )
            {
                tmp = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1);                                   
                if ( (tmp & 0x0020) == 0x0020 ) // register 1 bit 5, Autonegotiation Complete
                    break;
            }
            if ( cnt < 10 )
            {
                // autonegotiation process completed
                // Get speed and duplex status
                tmp = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x0);
                if ( ( tmp & 0x2000) == 0x2000 )
                    speed = PHY_SPEED_100M; 
                else
                    speed  = PHY_SPEED_10M; 
                
                if ( ( tmp & 0x0100) == 0x0100 )
                    duplex = 1;
                else
                    duplex = 0; 
            }
            else
            {
                // autonegotiation process not complete
                IgnoreFlag = 1;
            }
        }        
        else if ((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x18);
            duplex = (tmp & 0x0001);
            speed  = (tmp & 0x0002) ? PHY_SPEED_100M : PHY_SPEED_10M;
        }
        else if (priv->ids.miiPhyId == PHYID_RTL8201EL) {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x00);
            duplex = (tmp & 0x0100) ? 1 : 0;
            speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
        }
        else if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
                  (priv->ids.miiPhyId                      == PHYID_RTL8211E)         ) {
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x11);
            duplex = (tmp & PHY_DUPLEX_mask)>>13;
            speed  = (tmp & PHY_SPEED_mask)>>14;
        }
        else if (priv->ids.miiPhyId == PHYID_BCM54612E) { 
            // Get link status
            // First Switch shadow register selector
            ftgmac100_write_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1C, 0x2000);
            tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x1C);
            if ( (tmp & 0x0080) == 0x0080 )
                duplex = 0;
            else
                duplex = 1;
                               
            switch(tmp & 0x0018) {
                case 0x0000:
                    speed = PHY_SPEED_1G; 
                    break;
                case 0x0008:    
                    speed = PHY_SPEED_100M; 	                   
                    break;
                case 0x0010:    
                    speed = PHY_SPEED_10M;                         
                    break;
                default:
                    speed = PHY_SPEED_100M;
            } 
        }
        else 
        {
            PRINTK("%s:aspeed_mac_timer, Unknow PHY type\n", dev->name);
            duplex = 1;  
            speed  = PHY_SPEED_100M;
        }

        if ( IgnoreFlag == 0 )
        {
            macSpeed = ( ( (priv->maccr_val & GMAC_MODE_bit) >> 8  ) |    // Move bit[9] to bit[1]
                         ( (priv->maccr_val & SPEED_100_bit) >> 19 )  );  // bit[19] to bit[0]
            // The MAC hardware ignores SPEED_100_bit if GMAC_MODE_bit is set.
            if (macSpeed > PHY_SPEED_1G) 
                macSpeed = PHY_SPEED_1G;	// 0x3 --> 0x2
    
            if ( ( ((priv->maccr_val & FULLDUP_bit) != 0) != duplex ) || 
                 ( macSpeed != speed )                                ||                
                 ( ( priv->LinkStatus == LINK_UNKNOWN ) || ( priv->LinkStatus == LINK_DOWN ) ) )
            {
                PRINTK("%s:aspeed_mac_timer, priv->maccr_val=0x%05x [0x%X], PHY {speed,duplex}=%d,%d\n",
                                                    dev->name, priv->maccr_val, macSpeed, speed, duplex);
                ftgmac100_reset(dev);
                ftgmac100_enable(dev);
                netif_carrier_on(dev);

                priv->LinkStatus = LINK_UP;                
            }           
        }
    }
    else 
    {
        // Link down
        if ( ( priv->LinkStatus == LINK_UNKNOWN ) ||
             ( priv->LinkStatus == LINK_UP )        )
        {
            priv->LinkStatus = LINK_DOWN;

            netif_carrier_off(dev);
        }

        priv->timer.expires = jiffies + 1 * HZ;
    } // End if (status & LINK_STATUS)
    
#ifdef AST_SOC_G3
	//Fix issue 
	//When the Tx-MAC executes its last read request to DMA arbiter and the Rx-MAC send its first burst write request to DMA arbiter at the same time 
	//(i.e., only when Rx first burst write overlaps Tx last read), 
	//then the Rx-MAC write request would not be acknowledged by the DMA arbiter, 
	//this would cause the MAC can't write any more received frame data into the DRAM unless the Tx-MAC send out a read or write request to the arbiter. 
	//We suggest you to write the register MAC18 periodically, the Tx-MAC will issue single read request to the DMA arbiter.
	iowrite32( 0xffffffff, dev->base_addr + TXPD_REG);
#endif 	
	add_timer(&priv->timer);
}

/*
 . Function: ftgmac100_shutdown
 . Purpose:  closes down the SMC91xxx chip.
 . Method:
 .	1. zero the interrupt mask
 .	2. clear the enable receive flag
 .	3. clear the enable xmit flags
 .
 . TODO:
 .   (1) maybe utilize power down mode.
 .	Why not yet?  Because while the chip will go into power down mode,
 .	the manual says that it will wake up in response to any I/O requests
 .	in the register space.   Empirical results do not show this working.
*/
static void ftgmac100_shutdown( unsigned int ioaddr )
{
    //interrupt mask register
    iowrite32( 0, ioaddr + IER_REG );
    /* enable trans/recv,... */
    iowrite32( 0, ioaddr + MACCR_REG );
}

/*
 . Function: ftgmac100_wait_to_send_packet( struct sk_buff * skb, struct device * )
 . Purpose:
 .    Attempt to allocate memory for a packet, if chip-memory is not
 .    available, then tell the card to generate an interrupt when it
 .    is available.
 .
 . Algorithm:
 .
 . o	if the saved_skb is not currently null, then drop this packet
 .		on the floor.  This should never happen, because of TBUSY.
 . o	if the saved_skb is null, then replace it with the current packet,
 . o	See if I can sending it now.
 . o 	(NO): Enable interrupts and let the interrupt handler deal with it.
 . o	(YES):Send it now.
*/
static int ftgmac100_wait_to_send_packet( struct sk_buff * skb, struct net_device * dev )
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    unsigned long ioaddr        = dev->base_addr;
    volatile TX_DESC      *cur_desc;
    int                   length;
    struct iphdr          *ip;   
    unsigned int          tmp;
    
    spin_lock(&priv->tx_lock);  

    if (skb == NULL)
    {
        DO_PRINT("%s(%d): NULL skb???\n", __FILE__,__LINE__);
        priv->stats.tx_dropped ++;
        spin_unlock(&priv->tx_lock);
        return NETDEV_TX_OK;
    }
    
    PRINTK3("%s:ftgmac100_wait_to_send_packet, skb=%x\n", dev->name, skb);
    cur_desc = &priv->tx_descs[priv->tx_idx];
    
    // buffer full
    if ( (priv->old_tx == ((priv->tx_idx + 1) % TXDES_NUM)) ||
         (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)           )
    {
        if (priv->old_tx == ((priv->tx_idx + 1) % TXDES_NUM))
            PRINTK("buffer full [%d][%d]\n", priv->old_tx, priv->tx_idx );
        
        if (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)
            DO_PRINT("no empty TX descriptor or buffer full:0x%x:0x%x\n",
                (unsigned int)cur_desc,((unsigned int *)cur_desc)[0]);        
        
        priv->stats.tx_dropped++;        
        dev_kfree_skb (skb); // free skb
        netif_stop_queue(dev);
        
        spin_unlock(&priv->tx_lock);
        
        return NETDEV_TX_OK;
    }
        
    priv->tx_skbuff[priv->tx_idx] = skb;
    length = (ETH_ZLEN < skb->len) ? skb->len : ETH_ZLEN;
    length = min(length, TX_BUF_SIZE);


    PRINTK3("Transmitting Packet at 0x%x, skb->data = %x, len = %x\n",
            (unsigned int)cur_desc->VIR_TXBUF_BADR, skb->data, length);
#if FTMAC100_DEBUG > 2
    print_packet( skb->data, length );
#endif
    priv->tx_skbuff_len[priv->tx_idx] = length;
    priv->tx_skbdma[priv->tx_idx]     = dma_map_single(&dev->dev, skb->data, length, DMA_TO_DEVICE);
//    cur_desc->VIR_TXBUF_BADR = cpu_to_le32((unsigned long)skb->data);
    cur_desc->TXBUF_BADR = priv->tx_skbdma[priv->tx_idx];

    cur_desc->TXBUF_Size = length;
    cur_desc->LTS = 1;
    cur_desc->FTS = 1;

    cur_desc->TX2FIC = 0;
    cur_desc->TXIC = 0;

    if ( skb->ip_summed == CHECKSUM_PARTIAL )
    {
        ip = ip_hdr(skb);
        if ( ip->protocol == IPPROTO_TCP )
        {
            cur_desc->TCPCS_EN = 1;
            cur_desc->IPCS_EN  = 1;
        }
        else if ( ip->protocol == IPPROTO_UDP )
        {
            cur_desc->UDPCS_EN = 1;
            cur_desc->IPCS_EN  = 1;
        }
        else
        {
            DO_PRINT( "unknow protocol - [%x]\n", ip->protocol );  
        }
    }
  
    cur_desc->TXDMA_OWN = TX_OWNBY_FTGMAC100;
    wmb();
    iowrite32( 0xffffffff, ioaddr + TXPD_REG);

    priv->tx_idx = (priv->tx_idx + 1) % TXDES_NUM;
    priv->stats.tx_packets++;
    
    spin_unlock(&priv->tx_lock);

    return NETDEV_TX_OK;
}

static int ftgmac100_ringbuf_alloc(struct ftgmac100_priv *priv)
{
    int i;
    struct sk_buff *skb;
    
    /* Rx */
    priv->rx_descs = dma_alloc_coherent(priv->dev, 
                                sizeof(RX_DESC)*RXDES_NUM, 
                                &priv->rx_descs_dma, GFP_KERNEL);

    if( !priv->rx_descs )
        return -ENOMEM;
    
    memset(priv->rx_descs, 0, sizeof(RX_DESC) * RXDES_NUM);
    priv->rx_descs[RXDES_NUM-1].EDORR = 1;

    for (i=0; i<RXDES_NUM; i++) {
        skb = dev_alloc_skb(RX_BUF_SIZE + NET_IP_ALIGN);
        if (skb == NULL) {
            PRINTK (KERN_WARNING "alloc_list: allocate Rx buffer error! ");
            break;
        }
        skb_reserve(skb, NET_IP_ALIGN);

        priv->rx_skbuff[i] = skb;
        skb->dev = priv->dev;    /* Mark as being used by this device. */
        priv->rx_skbdma[i] = dma_map_single(&priv->netdev->dev, skb->data, (RX_BUF_SIZE + NET_IP_ALIGN), DMA_FROM_DEVICE);     
        priv->rx_descs[i].RXBUF_BADR     = priv->rx_skbdma[i];
       
        priv->rx_descs[i].RXPKT_RDY = RX_OWNBY_FTGMAC100;
    }

    /* Tx */
    priv->tx_descs = dma_alloc_coherent(priv->dev, 
                        sizeof(TX_DESC)*TXDES_NUM, 
                        &priv->tx_descs_dma ,GFP_KERNEL);

    if(!priv->tx_descs)
        return -ENOMEM;

    memset((void*)priv->tx_descs, 0, sizeof(TX_DESC)*TXDES_NUM);
    priv->tx_descs[TXDES_NUM-1].EDOTR = 1;                  // is last descriptor
    
    return 0;
}

static void ast_gmac_free_rx (struct net_device *dev)
{
    int i;
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    
    // Free Rx 
    for ( i = 0; i < RXDES_NUM; i++ ) 
    {
        dma_unmap_single(&dev->dev, priv->rx_skbdma[i], (RX_BUF_SIZE + NET_IP_ALIGN), DMA_FROM_DEVICE);
        dev_kfree_skb (priv->rx_skbuff[i]);
        priv->rx_skbdma[i] = 0;            
        priv->rx_skbuff[i] = NULL;
    }
}

#if FTMAC100_DEBUG > 2
static void print_packet( u8 * buf, int length )
{
#if 1
#if FTMAC100_DEBUG > 3
    int i;
    int remainder;
    int lines;
#endif


#if FTMAC100_DEBUG > 3
    lines = length / 16;
    remainder = length % 16;

    for ( i = 0; i < lines ; i ++ ) {
        int cur;

        for ( cur = 0; cur < 8; cur ++ ) {
            u8 a, b;

            a = *(buf ++ );
            b = *(buf ++ );
            DO_PRINT("%02x%02x ", a, b );
        }
        DO_PRINT("\n");
    }
    for ( i = 0; i < remainder/2 ; i++ ) {
        u8 a, b;

        a = *(buf ++ );
        b = *(buf ++ );
        DO_PRINT("%02x%02x ", a, b );
    }
    DO_PRINT("\n");
#endif
#endif
}
#endif

/*------------------------------------------------------------
 . Configures the specified PHY using Autonegotiation.
 .-------------------------------------------------------------*/
static void ftgmac100_phy_configure(struct net_device* dev)
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    unsigned long ioaddr = dev->base_addr;
    u32 tmp;

    switch (priv->ids.miiPhyId & PHYID_VENDOR_MASK) {
        case PHYID_VENDOR_MARVELL:
            ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x12, 0x4400);
            tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13 );
            break;
        case PHYID_VENDOR_REALTEK:
            switch (priv->ids.miiPhyId) {
                case PHYID_RTL8211E:
                    // Enable interrupt
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x12, 0x4400);
                    tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13 );
                    break;
                case PHYID_RTL8201EL:
                    break;
                case PHYID_RTL8201F:
                    // Switch to page 7
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0007);
                    // Set RMII Mode Setting Register (RMSR)
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 16, 0x0FFA);
                    // Switch to page 0
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0000);
                    break;
                case PHYID_RTL8211FD:
                    // Switch to page 0x0D08
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0D08);
                    
                    // Set register 21 bit6 = 0, fixed 100 half-duplex issue
                    tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 21);
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 21, (tmp & 0xFFBF));
                    
                    // Switch to page 0x0
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0);
                    
                    // implemented RJ45's LED function
#if defined(RTL8211FD_LED_SETTING)                    
                    // Switch to page 0x0D04
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0D04);
                    // Set RMII Mode Setting Register (RMSR)
                    	// Green LED --> LINK STATUS and ACTIVE
											// Orange LED --> LINK STATUS
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 16, 0x6D60);
                    // EEELCR bit[3:1] = 0 
                    tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 17);
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 17, (tmp & 0xFFF1));
                    // Switch to page 0
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 31, 0x0000);
#endif                    
                    break;                    
            } // End switch (priv->ids.miiPhyId) {
            break;
        case PHYID_VENDOR_BROADCOM:
            switch (priv->ids.miiPhyId) {
                case PHYID_BCM54612E:
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1C, 0x8C00); // Disable GTXCLK Clock Delay Enable
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x18, 0xF0E7); // Disable RGMII RXD to RXC Skew
                    break;			
                case PHYID_BCM5221A4:
                default:
                    tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x1b);
                    tmp |= 0x0004;
                    ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1b, (u16) tmp);
                    break;
                } // End switch (priv->ids.miiPhyId) {
            break;
        case PHYID_VENDOR_MICREL:
            switch (priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) {
                case PHYID_KSZ9031:
                    // implemented RJ45's LED function
                    break;          
                case PHYID_KSZ8081:
                    // implemented RJ45's LED function
                    break;                       
                default:
                    break;
                } // End switch (priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK)
            break;
    } // End switch (priv->ids.miiPhyId & PHYID_VENDOR_MASK) {
}


/*--------------------------------------------------------
 . Called by the kernel to send a packet out into the void
 . of the net.  This routine is largely based on
 . skeleton.c, from Becker.
 .--------------------------------------------------------
*/
static void ftgmac100_timeout (struct net_device *dev)
{
    /* If we get here, some higher level has decided we are broken.
       There should really be a "kick me" function call instead. */
    
    DO_PRINT(KERN_WARNING "%s: transmit timed out? (jiffies=%ld)\n",
                                        dev->name, jiffies);
    /* "kick" the adaptor */
    ftgmac100_reset( dev );
    ftgmac100_enable( dev );

    /* Reconfigure the PHY */
    ftgmac100_phy_configure(dev);

    netif_wake_queue(dev);
    dev->trans_start = jiffies;
}

static void ftgmac100_free_tx (struct net_device *dev) 
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    int                   entry;

    spin_lock(&priv->tx_lock);
 
    // Free used tx skbuffs 
    while ( priv->old_tx != priv->tx_idx )
    {   
        entry = priv->old_tx;
        if (priv->tx_descs[entry].TXDMA_OWN != TX_OWNBY_SOFTWARE)
        {
            break;
        }
        if (priv->tx_skbuff[entry] == NULL)
        {
            break;
        }
        
        dma_unmap_single(&dev->dev, priv->tx_skbdma[entry], priv->tx_skbuff_len[entry], DMA_TO_DEVICE);
        dev_kfree_skb (priv->tx_skbuff[entry]);
        priv->tx_skbuff[entry]     = NULL;
        priv->tx_skbuff_len[entry] = 0;
        priv->tx_skbdma[entry]     = 0;

        priv->old_tx++;
        priv->old_tx = priv->old_tx & ( TXDES_NUM - 1 );

    }
    
    if ( netif_queue_stopped(dev) )
    {        
        netif_wake_queue (dev);
    }
    
    spin_unlock(&priv->tx_lock);            
}

/*-------------------------------------------------------------
 .
 . ftgmac100_rcv -  receive a packet from the card
 .
 --------------------------------------------------------------
*/
static int ftgmac100_rcv(struct net_device *dev)
{
    struct ftgmac100_priv *priv  = (struct ftgmac100_priv *)dev->ml_priv;
    unsigned long         ioaddr = dev->base_addr;
    int 	              	packet_length;
    int 	              	rcv_cnt;
    volatile RX_DESC      *cur_desc;
    int                   cur_idx;
    int                   desc_used = 0;
    int                   start_idx;
    struct sk_buff        *skb;

    start_idx = priv->rx_idx;

    // Check All descriptor of MAC
    for (rcv_cnt = 0; rcv_cnt < RXDES_NUM ; ++rcv_cnt)
    {
        packet_length = 0;
        cur_idx       = priv->rx_idx;

        if( (cur_desc = &priv->rx_descs[priv->rx_idx])->RXPKT_RDY == RX_OWNBY_SOFTWARE )
        {
            desc_used++;
            priv->rx_idx = (priv->rx_idx + 1) % RXDES_NUM;

            if ( (cur_desc->FRS == 1) && (cur_desc->LRS == 1) )
            {
                if ( cur_desc->FIFO_FULL )
                {
                    PRINTK("info: RX_FIFO full\n");                    
                    priv->stats.rx_errors++;    // error frame....
                    continue;                     
                }
                else
                {
                    if ( cur_desc->RX_ERR     || 
                         cur_desc->CRC_ERR    || 
                         cur_desc->FTL        || 
                         cur_desc->RUNT       || 
                         cur_desc->RX_ODD_NB  ||
                         cur_desc->IPCS_FAIL
                       )
                    {
                        if (cur_desc->RX_ERR)
                        {
                            DO_PRINT("err: RX_ERR\n");
                        }
                        if (cur_desc->CRC_ERR)
                        {
                            DO_PRINT("err: CRC_ERR\n");
                        }
                        if (cur_desc->FTL)
                        {
//                        {
                            DO_PRINT("err: FTL\n");
                        }
                        if (cur_desc->RUNT)
                        {
                            DO_PRINT("err: RUNT\n");
                        }
                        if (cur_desc->RX_ODD_NB)
                        {
                            DO_PRINT("err: RX_ODD_NB\n");
                        }
                        if (cur_desc->IPCS_FAIL)
                        {
                            DO_PRINT("err: IPCS_FAIL\n");
                        }

                        priv->stats.rx_errors++;    // error frame....                  
                        continue;                        
                    }

                    if ((cur_desc->DF) && (cur_desc->UDPCS_FAIL))
                        {
                            DO_PRINT("err: UDPCS_FAIL\n");
                        priv->stats.rx_errors++;    // error frame....
                        continue;                                   
                        }
                    if ((cur_desc->DF) && (cur_desc->TCPCS_FAIL) )
                        {
                            DO_PRINT("err: TCPCS_FAIL\n");
                        
                        priv->stats.rx_errors++;    // error frame....
                        continue;                                                
                    }
                }

                if (cur_desc->MULTICAST)
                {
                    priv->stats.multicast++;
                }
            }
            else
            {
                DO_PRINT("INFO : FRS %x or LRS %x\n", cur_desc->FRS, cur_desc->LRS);
                priv->stats.rx_over_errors++;  
                continue; 
            } // End if ( (cur_desc->FRS == 1) && (cur_desc->LRS == 1) )
            
            packet_length += cur_desc->VDBC;
        }
        else
        {
            // No packet
            break;
        } // End if( (cur_desc = &priv->rx_descs[priv->rx_idx])->RXPKT_RDY == RX_OWNBY_SOFTWARE )
        
        if (packet_length > 0)
        {
            packet_length -= 4;
            
            skb = priv->rx_skbuff[cur_idx];
            dma_unmap_single(&dev->dev, priv->rx_skbdma[cur_idx], (RX_BUF_SIZE + NET_IP_ALIGN), DMA_FROM_DEVICE);
            priv->rx_skbdma[cur_idx] = 0;            
            priv->rx_skbuff[cur_idx] = NULL;
            
            skb_put (skb, packet_length);

            // We have enable Rx offload (include TCP and UDP and IP)
            if ( cur_desc->PROTL_TYPE != 0 ) {
                skb->ip_summed = CHECKSUM_UNNECESSARY;
            }

#if FTMAC100_DEBUG > 2                        
            PRINTK3("Receiving Packet at 0x%x, packet len = %x\n",(unsigned int)data, packet_length);
            print_packet( data, packet_length );
#endif            

            skb->protocol = eth_type_trans(skb, dev );

            netif_receive_skb(skb);

            priv->stats.rx_packets++;   
        }
        else
        {
            DO_PRINT("Err: PACKET LEN\n");
            priv->stats.rx_errors++;    // error frame....
        }
    } // End for (rcv_cnt = 0; rcv_cnt < RXDES_NUM ; ++rcv_cnt)

    if ( desc_used != 0 )
    {
        cur_idx = start_idx;
        for (rcv_cnt = 0; rcv_cnt < desc_used ; rcv_cnt++)
        {
            /* Dropped packets don't need to re-allocate */
            if (priv->rx_skbuff[cur_idx] == NULL) 
            {
                skb = dev_alloc_skb(RX_BUF_SIZE + NET_IP_ALIGN);
                if (skb == NULL) {
                    PRINTK (KERN_WARNING
                        "%s: receive_packet: "
                        "Unable to re-allocate Rx skbuff.#%d\n",
                        dev->name, cur_idx);
                }
                skb_reserve(skb, NET_IP_ALIGN);

                priv->rx_skbuff[cur_idx] = skb;
                skb->dev = dev;
                priv->rx_skbdma[cur_idx] = dma_map_single(&dev->dev, skb->data, (RX_BUF_SIZE + NET_IP_ALIGN), DMA_FROM_DEVICE);
                priv->rx_descs[cur_idx].RXBUF_BADR = priv->rx_skbdma[cur_idx];
            }                
            priv->rx_descs[cur_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
            
            cur_idx = (cur_idx + 1) % RXDES_NUM;
        }       
    }
   
    if (trans_busy[priv->ids.macId] == 1)
    {
        iowrite32( priv->maccr_val, ioaddr + MACCR_REG );
        iowrite32( ioread32(ioaddr + IER_REG) | RXBUF_UNAVA_bit, ioaddr + IER_REG);
        
        trans_busy[priv->ids.macId] = 0;
    }
    return rcv_cnt;
}

/*--------------------------------------------------------------------
 .
 . This is the main routine of the driver, to handle the net_device when
 . it needs some attention.
 .
 . So:
 .   first, save state of the chipset
 .   branch off into routines to handle each case, and acknowledge
 .	    each to the interrupt register
 .   and finally restore state.
 .
 ---------------------------------------------------------------------*/
static irqreturn_t ftgmac100_interrupt(int irq, void * dev_id)
{
    struct net_device     *dev   = dev_id;
    struct ftgmac100_priv *priv  = (struct ftgmac100_priv *)dev->ml_priv;
    unsigned long         ioaddr = dev->base_addr;


//  PRINTK3("%s: ftgmac100 interrupt started \n", dev->name);

    if (dev == NULL) {
        DO_PRINT(KERN_WARNING "%s: irq %d for unknown device.\n",	dev->name, irq);
        return IRQ_HANDLED;
    }

    if (likely(netif_running(dev))) {
        /* Disable interrupts for polling */
        iowrite32(0, dev->base_addr + IER_REG);
        napi_schedule(&priv->napi);
    }    

    return IRQ_HANDLED;
}

/*------------------------------------------------------------
 . Get the current statistics.
 . This may be called with the card open or closed.
 .-------------------------------------------------------------*/
static struct net_device_stats* ftgmac100_query_statistics(struct net_device *dev) 
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;
    
    return &priv->stats;
}

#ifdef HAVE_MULTICAST

// --------------------------------------------------------------------
// 	Finds the CRC32 of a set of bytes.
//	Again, from Peter Cammaert's code.
// --------------------------------------------------------------------
static int crc32( char * s, int length ) 
{
    /* indices */
    int perByte;
    int perBit;
    /* crc polynomial for Ethernet */
    const u32 poly = 0xedb88320;
    /* crc value - preinitialized to all 1's */
    u32 crc_value = 0xffffffff;

    for ( perByte = 0; perByte < length; perByte ++ ) {
        unsigned char	c;

        c = *(s++);
        for ( perBit = 0; perBit < 8; perBit++ ) {
            crc_value = (crc_value >> 1) ^ 
                        (((crc_value ^ c) & 0x01)? poly: 0);
            c >>= 1;
        }
    }
    return	crc_value;
}

/*
 . Function: ftgmac100_setmulticast( struct net_device *dev, int count, struct dev_mc_list * addrs )
 . Purpose:
 .    This sets the internal hardware table to filter out unwanted multicast
 .    packets before they take up memory.
*/

static void ftgmac100_setmulticast( struct net_device *dev, int count, struct dev_mc_list * addrs ) 
{
    struct dev_mc_list    *cur_addr;
    int                   crc_val;
    unsigned int          ioaddr = dev->base_addr;
    struct ftgmac100_priv *priv  = (struct ftgmac100_priv *)dev->ml_priv;
    struct AstMacHwConfig *ids   = &priv->ids;
    unsigned long         Combined_Channel_ID, i;
    struct sk_buff        *skb;
    u32                   maht1 = 0;
    u32                   maht0 = 0;
    
    cur_addr = addrs;	

    for (cur_addr = addrs ; cur_addr!=NULL ; cur_addr = cur_addr->next )
    {
        /* make sure this is a multicast address - shouldn't this be a given if we have it here ? */
        if ( !( *cur_addr->dmi_addr & 1 ) )
        {
            continue;
        }
#if 0 
//A0, A1
            crc_val = crc32( cur_addr->dmi_addr, 5 );
            crc_val = (~(crc_val>>2)) & 0x3f;
            if (crc_val >= 32)
            {
                iowrite32(ioread32(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
                priv->GigaBit_MAHT1 = ioread32 (ioaddr + MAHT1_REG);
            }
            else
            {
                iowrite32(ioread32(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
                priv->GigaBit_MAHT0 = ioread32 (ioaddr + MAHT0_REG);
            }
//10/100M
            crc_val = crc32( cur_addr->dmi_addr, 6 );
            crc_val = (~(crc_val>>2)) & 0x3f;
            if (crc_val >= 32)
            {
                iowrite32(ioread32(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
                priv->Not_GigaBit_MAHT1 = ioread32 (ioaddr + MAHT1_REG);
            }
            else
            {
                iowrite32(ioread32(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
                priv->Not_GigaBit_MAHT0 = ioread32 (ioaddr + MAHT0_REG);
            }
#else
//A2
            crc_val = crc32( cur_addr->dmi_addr, 6 );
            crc_val = (~(crc_val>>2)) & 0x3f;
            if (crc_val >= 32)
            {
                maht1 |= (1UL << (crc_val - 32));
            }
            else
            {
                maht0 |= (1UL << crc_val);
            }
#endif
    }
	if (ids->isRevA2) {
		outl(maht1, ioaddr + MAHT1_REG);
		outl(maht0, ioaddr + MAHT0_REG);
		lp->Not_GigaBit_MAHT1 = maht1;
		lp->GigaBit_MAHT1     = maht1;
		lp->Not_GigaBit_MAHT0 = maht0;
		lp->GigaBit_MAHT0     = maht0;
    }
}

/*-----------------------------------------------------------
 . ftgmac100_set_multicast_list
 .
 . This routine will, depending on the values passed to it,
 . either make it accept multicast packets, go into
 . promiscuous mode ( for TCPDUMP and cousins ) or accept
 . a select set of multicast packets
*/
static void ftgmac100_set_multicast_list(struct net_device *dev)
{
    unsigned int ioaddr         = dev->base_addr;
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;

    PRINTK2("%s:ftgmac100_set_multicast_list\n", dev->name);

    if (dev->flags & IFF_PROMISC)
        priv->maccr_val |= RX_ALLADR_bit;
    else
        priv->maccr_val &= ~RX_ALLADR_bit;

    if (dev->flags & IFF_ALLMULTI)
        priv->maccr_val |= RX_MULTIPKT_bit;
    else
        priv->maccr_val &= ~RX_MULTIPKT_bit;

    if (dev->mc_count)
    {
//		PRINTK("set multicast\n");
        priv->maccr_val |= RX_HT_EN_bit;
        ftgmac100_setmulticast( dev, dev->mc_count, dev->mc_list );
    }
    else
    {
        priv->maccr_val &= ~RX_HT_EN_bit;
    }

    iowrite32( priv->maccr_val, ioaddr + MACCR_REG );
}
#endif

static int ast_gmac_stop(struct net_device *dev)
{
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->ml_priv;

    napi_disable(&priv->napi);

    netif_stop_queue(dev);

    /* clear everything */
    ftgmac100_shutdown(dev->base_addr);
    free_irq(dev->irq, dev);

    if (priv->timer.function != NULL) {
        del_timer_sync(&priv->timer);
    }

    ast_gmac_free_rx( dev );

    if (priv->rx_descs)
        dma_free_coherent( NULL, sizeof(RX_DESC)*RXDES_NUM, (void*)priv->rx_descs, (dma_addr_t)priv->rx_descs_dma );
    if (priv->tx_descs)
        dma_free_coherent( NULL, sizeof(TX_DESC)*TXDES_NUM, (void*)priv->tx_descs, (dma_addr_t)priv->tx_descs_dma );

    priv->rx_descs = NULL; priv->rx_descs_dma = 0;
    priv->tx_descs = NULL; priv->tx_descs_dma = 0;
    
    return 0;
}

static int ftgmac100_open(struct net_device *netdev)
{
    struct ftgmac100_priv *priv = netdev_priv(netdev);
    int err;

    DO_PRINT("%s:ftgmac100_open\n", netdev->name);

    priv->maccr_val = (CRC_APD_bit | RXMAC_EN_bit | TXMAC_EN_bit  | RXDMA_EN_bit
                     | TXDMA_EN_bit | CRC_CHK_bit | RX_BROADPKT_bit | SPEED_100_bit | FULLDUP_bit);
    
    // Initail link status
    priv->LinkStatus = LINK_UNKNOWN;
    
    ftgmac100_ringbuf_alloc(priv);

    /* Grab the IRQ next.  Beyond this, we will free the IRQ. */
    err = request_irq(netdev->irq, (void *)&ftgmac100_interrupt,
                        IRQF_DISABLED, netdev->name, netdev);
    if (err)
    {
        DO_PRINT("%s: unable to get IRQ %d (retval=%d).\n",
                                netdev->name, netdev->irq, err);
        kfree(netdev->ml_priv);
        netdev->ml_priv = NULL;
        return err;
    }

    /* reset the hardware */
    ftgmac100_reset(netdev);
    ftgmac100_enable(netdev);

    if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) || 
        ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ9031)   ||
        ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_KSZ8081)   ||
         (priv->ids.miiPhyId == PHYID_RTL8201EL)                            ||
         (priv->ids.miiPhyId == PHYID_RTL8201F)                             ||        
         (priv->ids.miiPhyId == PHYID_RTL8211FD)                            ||        
         (priv->ids.miiPhyId == PHYID_RTL8211E)                             ||   
         (priv->ids.miiPhyId == PHYID_BCM54612E)                               ) 
    { 
        init_timer(&priv->timer);
        priv->timer.data = (unsigned long)netdev;
        priv->timer.function = aspeed_mac_timer;
        priv->timer.expires = jiffies + 1 * HZ;
        add_timer (&priv->timer);
    }
    
    /* Configure the PHY */
    ftgmac100_phy_configure(netdev);


    netif_start_queue(netdev);

    napi_enable(&priv->napi);

    return 0;
}

/* Module Variables */
static const struct net_device_ops ast_netdev_ops = {
        ndo_open:               ftgmac100_open,
        ndo_stop:               ast_gmac_stop,
        ndo_start_xmit:         ftgmac100_wait_to_send_packet,
        ndo_tx_timeout:         ftgmac100_timeout,
        ndo_get_stats:          ftgmac100_query_statistics,
        ndo_set_mac_address:    eth_mac_addr,
#ifdef  HAVE_MULTICAST
        ndo_set_multicast_list: ftgmac100_set_multicast_list,
#endif
};

static int ftgmac100_poll(struct napi_struct *napi, int budget)
{
    struct ftgmac100_priv *priv = container_of(napi, struct ftgmac100_priv, napi);
    struct net_device     *dev  = priv->netdev;
    unsigned long         ioaddr = dev->base_addr;
    unsigned long         tmp;
    int work_done = 0;
    bool completed = true;
    unsigned int          mask;         // interrupt mask
    unsigned int          status;       // interrupt status

    /* read the interrupt status register */
    mask = priv->EnInt_reg;

    /* read the status flag, and mask it */
    status = ioread32( ioaddr + ISR_REG ) & mask;
    iowrite32(status, ioaddr + ISR_REG ); // write to clear

    if ( status & (TPKT2E_bit|TPKT_LOST_bit))
    {
        //free tx skb buf
        ftgmac100_free_tx(dev);
    }

    if ( status & RPKT2B_bit )
    {
        bool retry;
        work_done = ftgmac100_rcv(dev);
        if (work_done >= budget)
            completed = false;           
    }
    
    if (status & RXBUF_UNAVA_bit)
    {
        trans_busy[priv->ids.macId] = 1;
    /*
        rcv_tq.sync = 0;
        rcv_tq.routine = ftgmac100_rcv;
        rcv_tq.data = dev;
        queue_task(&rcv_tq, &tq_timer);
    */

    } else if (status & AHB_ERR_bit)
    {
        DO_PRINT("AHB ERR \n");
    }

    if ( completed )
    {
        napi_complete(napi);

        iowrite32( priv->EnInt_reg, dev->base_addr + IER_REG );
    }
    
    return work_done;
}

static int __init ast_gmac_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct net_device *netdev;
    struct ftgmac100_priv *priv;
    int    err;
    
#if defined( MAC_ADDR_FROM_UBOOT )      
    unsigned int  maddr;
    unsigned int  laddr;
#endif    
    
    if (!pdev)
        return -ENODEV;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    /* setup net_device */
    netdev = alloc_etherdev(sizeof(*priv));
    if (!netdev) {
        err = -ENOMEM;
        goto err_alloc_etherdev;
    }
    netdev->irq = platform_get_irq(pdev, 0);
    if (netdev->irq < 0) {
        err = -ENXIO;
        goto err_netdev;
    }
    SET_NETDEV_DEV(netdev, &pdev->dev);

//  SET_ETHTOOL_OPS(netdev, &ftgmac100_ethtool_ops);
    
    netdev->netdev_ops		 = &ast_netdev_ops;
    netdev->watchdog_timeo = TX_TIMEOUT;

	netdev->features = NETIF_F_GRO;

    platform_set_drvdata(pdev, netdev);

    /* setup private data */
    priv = netdev_priv(netdev);
    priv->netdev = netdev;
    priv->dev = &pdev->dev;
    netdev->ml_priv = priv;
    
    priv->ids.macId = pdev->id; 
    
    spin_lock_init(&priv->tx_lock);

    /* initialize NAPI */
    netif_napi_add( netdev, &priv->napi, ftgmac100_poll, RXDES_NUM );

    /* map io memory */
    res = request_mem_region(res->start, resource_size(res),
                    dev_name(&pdev->dev));
    if (!res) {
        dev_err(&pdev->dev, "Could not reserve memory region\n");
        err = -ENOMEM;
        goto err_req_mem;
    }

    netdev->base_addr = (u32)ioremap(res->start, resource_size(res));

    if (!netdev->base_addr) {
        dev_err(&pdev->dev, "Failed to ioremap ethernet registers\n");
        err = -EIO;
        goto err_ioremap;
    }

//	priv->irq = irq;
#if 0//CONFIG_AST_MDIO
    /* initialize mdio bus */
    priv->mii_bus = mdiobus_alloc();
    if (!priv->mii_bus) {
        err = -EIO;
        goto err_alloc_mdiobus;
    }

    priv->mii_bus->name = "ftgmac100_mdio";
    snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "ftgmac100_mii.%d",pdev->id);

    priv->mii_bus->priv = netdev;
    priv->mii_bus->read = ftgmac100_mdiobus_read;
    priv->mii_bus->write = ftgmac100_mdiobus_write;
    priv->mii_bus->reset = ftgmac100_mdiobus_reset;
    priv->mii_bus->irq = priv->phy_irq;

    for (i = 0; i < PHY_MAX_ADDR; i++)
        priv->mii_bus->irq[i] = PHY_POLL;

    err = mdiobus_register(priv->mii_bus);
    if (err) {
        dev_err(&pdev->dev, "Cannot register MDIO bus!\n");
        goto err_register_mdiobus;
    }

    err = ftgmac100_mii_probe(priv);
    if (err) {
        dev_err(&pdev->dev, "MII Probe failed!\n");
        goto err_mii_probe;
    }
#endif

    /* register network device */
    err = register_netdev(netdev);

    if (err) {
        dev_err(&pdev->dev, "Failed to register netdev\n");
        goto err_alloc_mdiobus;
    }

#if defined( MAC_ADDR_FROM_UBOOT )    
    maddr = ioread32(priv->netdev->base_addr + MAC_MADR_REG);
    laddr = ioread32(priv->netdev->base_addr + MAC_LADR_REG);
    netdev->dev_addr[0] = (unsigned char) ((maddr >> 8 ) & 0x000000FF );
    netdev->dev_addr[1] = (unsigned char) ( maddr        & 0x000000FF );
    netdev->dev_addr[2] = (unsigned char) ((laddr >> 24) & 0x000000FF );
    netdev->dev_addr[3] = (unsigned char) ((laddr >> 16) & 0x000000FF );
    netdev->dev_addr[4] = (unsigned char) ((laddr >>  8) & 0x000000FF );
    netdev->dev_addr[5] = (unsigned char) ( laddr        & 0x000000FF );
#endif    
    
//	PRINTK("irq %d, mapped at %x\n", netdev->irq, (u32)netdev->base_addr);

    if (!is_valid_ether_addr(netdev->dev_addr)) {
        random_ether_addr(netdev->dev_addr);
        PRINTK("generated random MAC address %pM\n",
                netdev->dev_addr);
    }

    return 0;

//err_register_netdev:
//	phy_disconnect(priv->phydev);
//err_mii_probe:
//	mdiobus_unregister(priv->mii_bus);
//err_register_mdiobus:
//	mdiobus_free(priv->mii_bus);
err_alloc_mdiobus:
    iounmap((void __iomem *)netdev->base_addr);
err_ioremap:
    release_resource(res);
err_req_mem:
    netif_napi_del(&priv->napi);
    platform_set_drvdata(pdev, NULL);
err_netdev:	
    free_netdev(netdev);
err_alloc_etherdev:
    return err;

}

static int ast_gmac_remove(struct platform_device *pdev)
{
    struct net_device *dev = platform_get_drvdata(pdev);
    struct ftgmac100_priv *priv = netdev_priv(dev);

//  remove_proc_entry(dev->name, 0);

    unregister_netdev(dev);

#ifdef CONFIG_MII_PHY
    phy_disconnect(priv->phydev);
    mdiobus_unregister(priv->mii_bus);
    mdiobus_free(priv->mii_bus);
#endif

    iounmap((void __iomem *)dev->base_addr);

    netif_napi_del(&priv->napi);

    platform_set_drvdata(pdev, NULL);
    free_netdev(dev);
    return 0;
}

static struct platform_driver ast_gmac_driver = {
    .remove = ast_gmac_remove,
    .driver = {
    .name   = "ast_gmac",
    .owner  = THIS_MODULE,
    },
};

static int __init ast_gmac_init(void)
{
    return platform_driver_probe(&ast_gmac_driver, ast_gmac_probe);
}

static void __exit ast_gmac_exit(void)
{
    platform_driver_unregister(&ast_gmac_driver);
}

module_init(ast_gmac_init)
module_exit(ast_gmac_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ASPEED Technology Inc.");
MODULE_DESCRIPTION("NIC driver for AST Series");
MODULE_LICENSE("GPL");
