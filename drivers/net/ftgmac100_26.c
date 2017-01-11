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
//  "ASPEED MAC Driver, (Linux Kernel 2.6.15.7) 10/02/07 - by ASPEED\n"
// Further improvements:
//
//   -- Assume MAC1 has a PHY chip.  Read the chip type and handle Marvell
//      or Broadcom, else don't touch PHY chip (if present).
//
//   -- If MAC2 is on, check if U-Boot enabled the MII2DC+MII2DIO pins.
//      If yes, handle Marvell or Broadcom PHY.  If no, assume sideband RMII
//      interface with no PHY chip.
// 1.12/27/07 - by river@aspeed
//   Workaround for the gigabit hash function
// 2.12/27/07 - by river@aspeed
//   Synchronize the EDORR bit with document, D[30], D[15] both are EDORR
// 3.12/31/07 - by river@aspeed
//   Add aspeed_i2c_init and aspeed_i2c_read function for DHCP
// 4.04/10/2008 - by river@aspeed
//   Synchronize the EDOTR bit with document, D[30] is EDOTR
// 5.04/10/2008 - by river@aspeed
//   Remove the workaround for multicast hash function in A2 chip
// SDK 0.19
// 6.05/15/2008 - by river@aspeed
//   Fix bug of free sk_buff in wrong routine
// 7.05/16/2008 - by river@aspeed
//   Fix bug of skb_over_panic()
// 8.05/22/2008 - by river@aspeed
//   Support NCSI Feature
// SDK 0.20
// 9.07/02/2008 - by river@aspeed
//   Fix TX will drop packet bug
// SDK 0.21
//10.08/06/2008 - by river@aspeed
//   Add the netif_carrier_on() and netif_carrier_off()
//11.08/06/2008 - by river@aspeed
//   Fix the timer did not work after device closed
// SDK0.22
//12.08/12/2008 - by river@aspeed
//   Support different PHY configuration
// SDK0.23
//13.10/14/2008 - by river@aspeed
//   Support Realtek RTL8211BN Gigabit PHY
//14.11/17/2008 - by river@aspeed
//   Modify the allocate buffer to alignment to IP header
// SDK0.26
//15.07/28/2009 - by river@aspeed
//   Fix memory leakage problem in using multicast
//16.07/28/2009 - by river@aspeed
//   tx_free field in the local structure should be integer
//
//
//
//AST2300 SDK 0.12
//17.03/30/2010 - by river@aspeed
//   Modify for AST2300's hardware CLOCK/RESET/MULTI-PIN configuration
//18.03/30/2010 - by river@aspeed
//   Fix does not report netif_carrier_on() and netif_carrier_off() when use MARVELL PHY
//AST2300 SDK 0.13
//17.06/10/2010 - by river@aspeed
//   Support AST2300 A0
//18.06/10/2010 - by river@aspeed
//   EEPROM is at I2C channel 4 on AST2300 A0 EVB
//AST2300 SDK 0.14
//19.09/13/2010 - by river@aspeed
//   Support Realtek RTL8201EL 10/100M PHY
//AST2400
//20.06/25/2013 - by CC@aspeed
//   Support BCM54612E 10/100/1000M PHY
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
#include <mach/ftgmac100_drv.h>

#include <linux/skbuff.h>

#include "ftgmac100_26.h"

#if defined(CONFIG_ARM)
#include <mach/hardware.h>
#include <asm/cacheflush.h>

#elif defined(CONFIG_COLDFIRE)
#include <asm/astsim.h>

#else
#err "Not define include for GMAC"
#endif

#if defined(CONFIG_WEDGE100) || defined(CONFIG_GALAXY100)
#define PHY_DEFAULT_ADDR 0x18
#else
/* wedge */
#define PHY_DEFAULT_ADDR 0x1F
#endif

/*------------------------------------------------------------------------
 .
 . Configuration options, for the experienced user to change.
 .
 -------------------------------------------------------------------------*/

/*
 . DEBUGGING LEVELS
 .
 . 0 for normal operation
 . 1 for slightly more details
 . >2 for various levels of increasingly useless information
 .    2 for interrupt tracking, status flags
 .    3 for packet info
 .    4 for complete packet dumps
*/

#define DO_PRINT(args...) printk(": " args)

#define FTMAC100_DEBUG  1

#if (FTMAC100_DEBUG > 2 )
#define PRINTK3(args...) DO_PRINT(args)
#else
#define PRINTK3(args...)
#endif

#if FTMAC100_DEBUG > 1
#define PRINTK2(args...) DO_PRINT(args)
#else
#define PRINTK2(args...)
#endif

#ifdef FTMAC100_DEBUG
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

static volatile int trans_busy = 0;

void ftgmac100_phy_rw_waiting(unsigned int ioaddr)
{
	unsigned int tmp;

	do {
		mdelay(10);
		tmp =inl(ioaddr + PHYCR_REG);
	} while ((tmp&(PHY_READ_bit|PHY_WRITE_bit)) > 0);
}


/*------------------------------------------------------------
 . Reads a register from the MII Management serial interface
 .-------------------------------------------------------------*/
static u16 ftgmac100_read_phy_register(unsigned int ioaddr, u8 phyaddr, u8 phyreg)
{
	unsigned int tmp;

	if (phyaddr > 0x1f)	// MII chip IDs are 5 bits long
	    return 0xffff;

	tmp = inl(ioaddr + PHYCR_REG);
	tmp &= 0x3000003F;
	tmp |=(phyaddr<<16);
	tmp |=(phyreg<<(16+5));
	tmp |=PHY_READ_bit;

	outl( tmp, ioaddr + PHYCR_REG );
	ftgmac100_phy_rw_waiting(ioaddr);

	return (inl(ioaddr + PHYDATA_REG)>>16);
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

	tmp = inl(ioaddr + PHYCR_REG);
	tmp &= 0x3000003F;
	tmp |=(phyaddr<<16);
	tmp |=(phyreg<<(16+5));
	tmp |=PHY_WRITE_bit;

	outl( phydata, ioaddr + PHYDATA_REG );
	outl( tmp, ioaddr + PHYCR_REG );
	ftgmac100_phy_rw_waiting(ioaddr);
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
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;

//	out->macId   = dev->dev_id;
//..	getMacAndPhy(dev, out);
	out->phyAddr = PHY_DEFAULT_ADDR;
	out->miiPhyId = 0;

	// We assume the Clock Stop register does not disable the MAC1 or MAC2 clock
	// unless Reset Control also holds the MAC in reset.
	// For now, we only support a PHY chip on the MAC's own MDC+MDIO bus.
	if (out->phyAddr > 0x1f) {
no_phy_access:
	out->phyAddr = 0xff;
	return;
	}

	if (priv->NCSI_support == 0) {
		out->miiPhyId = ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x02);
		if (out->miiPhyId == 0xFFFF) { //Realtek PHY at address 1
			out->phyAddr = 1;
		}
#if 0
		if (out->miiPhyId == 0x0362) {
			out->phyAddr = 1;
		}
#endif
		out->miiPhyId = ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x02);
		out->miiPhyId = (out->miiPhyId & 0xffff) << 16;
		out->miiPhyId |= ftgmac100_read_phy_register(dev->base_addr, out->phyAddr, 0x03) & 0xffff;

		switch (out->miiPhyId >> 16) {
		case 0x0040:	// Broadcom
		case 0x0141:	// Marvell
		case 0x001c:	// Realtek
		case 0x0362:	// BCM54612, BCM54616
		break;

		default:
		// Leave miiPhyId for DO_PRINT(), but reset phyAddr.
		// out->miiPhyId = 2;
		goto no_phy_access;
		break;
		}
	}
	return;
}

// --------------------------------------------------------------------
//  NCSI function
// --------------------------------------------------------------------
void NCSI_Struct_Initialize(struct net_device *dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long i;

  for (i = 0; i < 6; i++) {
    lp->NCSI_Request.DA[i] = 0xFF;
    lp->NCSI_Respond.DA[i] = 0xFF;
    lp->NCSI_Respond.SA[i] = 0xFF;
    lp->NCSI_Request.SA[i] = dev->dev_addr[i];
  }
  lp->NCSI_Request.EtherType = 0xF888;
  lp->NCSI_Request.MC_ID = 0;
  lp->NCSI_Request.Header_Revision = 0x01;
  lp->NCSI_Request.Reserved_1 = 0;
  lp->NCSI_Request.Reserved_2 = 0;
  lp->NCSI_Request.Reserved_3 = 0;
  lp->NCSI_Respond.EtherType = 0xF888;
  lp->NCSI_Respond.MC_ID = 0;
  lp->NCSI_Respond.Header_Revision = 0x01;
  lp->NCSI_Respond.Reserved_1 = 0;
  lp->NCSI_Respond.Reserved_2 = 0;
  lp->NCSI_Respond.Reserved_3 = 0;

  lp->InstanceID = 0;
  lp->Payload_Checksum = 0;
  for (i = 0; i < 4; i++) {
    lp->Payload_Pad[i] = 0;
  }
  for (i = 0; i < 64; i++) {
    lp->Payload_Data[i] = 0;
  }
}

void Calculate_Checksum(struct net_device * dev, unsigned char *buffer_base, int Length)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned int i, CheckSum = 0;
  unsigned int Data, Data1;

  for (i = 0; i < ((Length - 14) / 2); i++) {
    Data = buffer_base[i * 2];
    Data1 = buffer_base[i * 2 + 1];
    CheckSum += ((Data << 8) + Data1);
  }
  lp->Payload_Checksum = (~(CheckSum) + 1); //2's complement
//Inverse for insert into buffer
  Data = (lp->Payload_Checksum & 0xFF000000) >> 24;
  Data1 = (lp->Payload_Checksum & 0x000000FF) << 24;
  lp->Payload_Checksum = (lp->Payload_Checksum & 0x00FFFF00) + Data + Data1;
  Data = (lp->Payload_Checksum & 0x00FF0000) >> 8;
  Data1 = (lp->Payload_Checksum & 0x0000FF00) << 8;
  lp->Payload_Checksum = (lp->Payload_Checksum & 0xFF0000FF) + Data + Data1;
}

void copy_data (struct net_device * dev, struct sk_buff * skb, int Length)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;

  memcpy ((unsigned char *)(skb->data + 30), &lp->Payload_Data, Length);
  Calculate_Checksum(dev, skb->data + 14, 30 + Length);
  memcpy ((unsigned char *)(skb->data + 30 + Length), &lp->Payload_Checksum, 4);
}

void NCSI_Rx (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long status, length, i = 0;
  volatile RX_DESC *cur_desc;
  int count = 0;

ncsi_rx:
  i = 0;
  cur_desc = &lp->rx_descs[lp->rx_idx];
  do {
    status = cur_desc->RXPKT_RDY;
    i++;
  } while (!(((status & RX_OWNBY_SOFTWARE) != 0) || (i >= NCSI_LOOP)));

  if (i < NCSI_LOOP) {
    if (cur_desc->LRS) {
      length = cur_desc->VDBC;
      unsigned char *tbuf = (unsigned char *)phys_to_virt(cur_desc->RXBUF_BADR);
      if (length <= 128 && tbuf[12] == 0x88 && tbuf[13] == 0xF8) {
        memcpy (&lp->NCSI_Respond, (unsigned char *)phys_to_virt(cur_desc->RXBUF_BADR), length);
      } else {
          //printk("NCSI_RX: Skip len: %d, proto: %x:%x\n", length, tbuf[12], tbuf[13]);
          lp->rx_descs[lp->rx_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
          lp->rx_idx = (lp->rx_idx+1)%RXDES_NUM;
          // Check next descriptor for response packet
          if (count++ <= 500000) {
            goto ncsi_rx;
          }
      }

    }
    lp->rx_descs[lp->rx_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
    lp->rx_idx = (lp->rx_idx+1)%RXDES_NUM;
  } else {
    printk("NCSI_Rx: Failed\n");
  }
}

void DeSelect_Package (struct net_device * dev, int Package_ID)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DESELECT_PACKAGE;
  Combined_Channel_ID = (Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F, 0x1F means all channel
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

int Select_Package (struct net_device * dev, int Package_ID)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = SELECT_PACKAGE;
  Combined_Channel_ID = (Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (4 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 4;
  memset ((void *)lp->Payload_Data, 0, 4);
  lp->Payload_Data[3] = 1; //Arbitration Disable
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len = 30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    Found = 0;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
    Found = 1;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;

    return Found;
}


void DeSelect_Active_Package (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DESELECT_PACKAGE;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F, 0x1F means all channel
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}


int Select_Active_Package (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = SELECT_PACKAGE;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (4 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 4;
  memset ((void *)lp->Payload_Data, 0, 4);
  lp->Payload_Data[3] = 1; //Arbitration Disable
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len = 30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
    Found = 0;
  }
  else {
    lp->Retry = 0;
    Found = 1;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;

    return Found;
}

int Clear_Initial_State (struct net_device * dev, int Channel_ID)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = CLEAR_INITIAL_STATE;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + Channel_ID; //Internal Channel ID = 0
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (CLEAR_INITIAL_STATE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
    Found = 0;
  }
  else {
    lp->Retry = 0;
    Found = 1;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;

    return Found;
}

void Get_Version_ID (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = GET_VERSION_ID;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_VERSION_ID | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Get_Capabilities (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = GET_CAPABILITIES;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_CAPABILITIES | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
    lp->NCSI_Cap.Capabilities_Flags = lp->NCSI_Respond.Payload_Data[0];
    lp->NCSI_Cap.Broadcast_Packet_Filter_Capabilities = lp->NCSI_Respond.Payload_Data[1];
    lp->NCSI_Cap.Multicast_Packet_Filter_Capabilities = lp->NCSI_Respond.Payload_Data[2];
    lp->NCSI_Cap.Buffering_Capabilities = lp->NCSI_Respond.Payload_Data[3];
    lp->NCSI_Cap.AEN_Control_Support = lp->NCSI_Respond.Payload_Data[4];
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Enable_AEN (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = AEN_ENABLE;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (8 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 8;
  lp->Payload_Data[3] = 0; //MC ID
  lp->Payload_Data[7] = 1; //Link Status Change AEN
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (AEN_ENABLE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Get_MAC_Address (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = 0x50;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (8 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 8;
  lp->Payload_Data[0] = 0x00;
  lp->Payload_Data[1] = 0x00;
  lp->Payload_Data[2] = 0x81;
  lp->Payload_Data[3] = 0x19;

  lp->Payload_Data[4] = 0x00;
  lp->Payload_Data[5] = 0x00;
  lp->Payload_Data[6] = 0x1B;
  lp->Payload_Data[7] = 0x00;

  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (0x50 | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;

  // Update MAC Address
printk("NCSI: MAC  ");
for (i = 0; i < 6; i++)
  printk("%02X:", lp->NCSI_Respond.Payload_Data[12+i]);
printk("\n");
  memcpy(lp->NCSI_Request.SA, &lp->NCSI_Respond.Payload_Data[12], 6);
  memcpy(dev->dev_addr, &lp->NCSI_Respond.Payload_Data[12], 6);

	/* Update the MAC address */
	ast_gmac_set_mac(lp, dev->dev_addr);
}

void Set_MAC_Affinity (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = 0x50;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (60 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);

  lp->NCSI_Request.Payload_Length = 60;
  memset(lp->Payload_Data, 0x00, 60);
  lp->Payload_Data[0] = 0x00;
  lp->Payload_Data[1] = 0x00;
  lp->Payload_Data[2] = 0x81;
  lp->Payload_Data[3] = 0x19;

  lp->Payload_Data[4] = 0x00;
  lp->Payload_Data[5] = 0x01;
  lp->Payload_Data[6] = 0x07;
  lp->Payload_Data[7] = 0x00;

  for (i = 0; i < 6; i++) {
    lp->Payload_Data[8+i] = lp->NCSI_Request.SA[i];
  }

  lp->Payload_Data[14] = 0x09;

  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (0x50 | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Enable_Set_MAC_Address (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = SET_MAC_ADDRESS;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (8 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 8;
  for (i = 0; i < 6; i++) {
    lp->Payload_Data[i] = lp->NCSI_Request.SA[i];
  }
  lp->Payload_Data[6] = 1; //MAC Address Num = 1 --> address filter 1, fixed in sample code
  lp->Payload_Data[7] = UNICAST + 0 + ENABLE_MAC_ADDRESS_FILTER; //AT + Reserved + E
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SET_MAC_ADDRESS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Enable_Broadcast_Filter (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = ENABLE_BROADCAST_FILTERING;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (4 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 4;
  memset ((void *)lp->Payload_Data, 0, 4);
        lp->Payload_Data[3] = 0x1; //ARP, DHCP, NetBIOS
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_BROADCAST_FILTERING | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Disable_Multicast_Filter (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DISABLE_GLOBAL_MULTICAST_FILTERING;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_GLOBAL_MULTICAST_FILTERING | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Disable_VLAN (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DISABLE_VLAN;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_VLAN | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Get_Parameters (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = GET_PARAMETERS;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_PARAMETERS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
//    printk ("Retry: Command = %x, Response_Code = %x, Resonpd.Command = %x, IID = %x, lp->InstanceID = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code, lp->NCSI_Respond.Command, lp->NCSI_Respond.IID, lp->InstanceID);
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
    lp->NCSI_Cap.Capabilities_Flags = lp->NCSI_Respond.Payload_Data[0];
    lp->NCSI_Cap.Broadcast_Packet_Filter_Capabilities = lp->NCSI_Respond.Payload_Data[1];
    lp->NCSI_Cap.Multicast_Packet_Filter_Capabilities = lp->NCSI_Respond.Payload_Data[2];
    lp->NCSI_Cap.Buffering_Capabilities = lp->NCSI_Respond.Payload_Data[3];
    lp->NCSI_Cap.AEN_Control_Support = lp->NCSI_Respond.Payload_Data[4];
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Enable_Network_TX (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = ENABLE_CHANNEL_NETWORK_TX;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_CHANNEL_NETWORK_TX | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Disable_Network_TX (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DISABLE_CHANNEL_NETWORK_TX;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + (lp->NCSI_Request.Payload_Length % 4) + 8;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_CHANNEL_NETWORK_TX | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Enable_Channel (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = ENABLE_CHANNEL;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_CHANNEL | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

void Disable_Channel (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = DISABLE_CHANNEL;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (4 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 4;
  memset ((void *)lp->Payload_Data, 0, 4);
        lp->Payload_Data[3] = 0x1; //ALD
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_CHANNEL | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

int Get_Link_Status (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = GET_LINK_STATUS;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = 0;
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_LINK_STATUS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
    if (lp->NCSI_Respond.Payload_Data[3] & 0x40) {
      return (lp->NCSI_Respond.Payload_Data[3] & 0x01); //Link Up or Not
    }
    else {
      return 0; //Auto Negotiate did not finish
    }
}

void Set_Link (struct net_device * dev)
{
  struct ftgmac100_priv *lp = (struct ftgmac100_priv *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

    do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = SET_LINK;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (8 << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
  lp->NCSI_Request.Payload_Length = 8;
  memset ((void *)lp->Payload_Data, 0, 8);
  lp->Payload_Data[2] = 0x02; //full duplex
  lp->Payload_Data[3] = 0x04; //100M, auto-disable
  copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
  skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
  ftgmac100_wait_to_send_packet (skb, dev);
//RX
  NCSI_Rx(dev);
  if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SET_LINK | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    lp->Retry = 0;
  }
    } while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
    lp->Retry = 0;
}

static void ftgmac100_reset( struct net_device* dev )
{
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
	struct AstMacHwConfig* ids = &priv->ids;
	unsigned int tmp, speed, duplex;

	getMacHwConfig(dev, ids);
	PRINTK("%s:ftgmac100_reset, phyAddr=0x%x, miiPhyId=0x%04x_%04x\n",
	       dev->name, ids->phyAddr, (ids->miiPhyId >> 16), (ids->miiPhyId & 0xffff));

  if ((priv->NCSI_support == 1) || (priv->INTEL_NCSI_EVA_support == 1)) {
    ids->miiPhyId = 0xFFFF;
    // NCSI mode always is 100M and full duplex
    duplex = 1;
    speed  = PHY_SPEED_100M;
  } else {
    if (ids->miiPhyId < 1)
        return;	// Cannot access MAC registers

    // Check the link speed and duplex.
    // They are not valid until auto-neg is resolved, which is reg.1 bit[5],
    // or the link is up, which is reg.1 bit[2].

    if (ids->phyAddr < 0xff)
        tmp = ftgmac100_read_phy_register(dev->base_addr, ids->phyAddr, 0x1);
    else tmp = 0;

    if (0==(tmp & (1u<<5 | 1u<<2)) || ids->phyAddr >= 0xff) {
        // No PHY chip, or link has not negotiated.
        speed  = PHY_SPEED_100M;
        duplex = 1;
        netif_carrier_off(dev);
    }
    else if (((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL)) {
        tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x00);
        duplex = (tmp & 0x0100) ? 1 : 0;
        speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
    }
    else if (((ids->miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
       ((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
        // Use reg.17_0.bit[15:13] for {speed[1:0], duplex}.
        tmp    = ftgmac100_read_phy_register(dev->base_addr, ids->phyAddr, 0x11);
        duplex = (tmp & PHY_DUPLEX_mask)>>13;
        speed  = (tmp & PHY_SPEED_mask)>>14;
        netif_carrier_on(dev);
    }
    else if (priv->ids.miiPhyId == PHYID_BCM54612E
       || priv->ids.miiPhyId == PHYID_BCM54616S) {
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
  }// NCSI_Check

	if (speed == PHY_SPEED_1G) {
	    // Set SPEED_100_bit too, for consistency.
	    priv->maccr_val |= GMAC_MODE_bit | SPEED_100_bit;
	    tmp = inl( dev->base_addr + MACCR_REG );
	    tmp |= GMAC_MODE_bit | SPEED_100_bit;
	    outl(tmp, dev->base_addr + MACCR_REG );
	} else {
	    priv->maccr_val &= ~(GMAC_MODE_bit | SPEED_100_bit);
	    tmp = inl( dev->base_addr + MACCR_REG );
	    tmp &= ~(GMAC_MODE_bit | SPEED_100_bit);
	    if (speed == PHY_SPEED_100M) {
		priv->maccr_val |= SPEED_100_bit;
		tmp |= SPEED_100_bit;
	    }
	    outl(tmp, dev->base_addr + MACCR_REG );
	}
	if (duplex)
	    priv->maccr_val |= FULLDUP_bit;
    else
		priv->maccr_val &= ~FULLDUP_bit;

	outl( SW_RST_bit, dev->base_addr + MACCR_REG );

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
	for (; (inl( dev->base_addr + MACCR_REG ) & SW_RST_bit) != 0; )
	{
		mdelay(10);
		PRINTK3("RESET: reset not complete yet\n" );
	}

	outl( 0, dev->base_addr + IER_REG );			/* Disable all interrupts */
}

static void ftgmac100_enable( struct net_device *dev )
{
	int i;
	struct ftgmac100_priv *priv 	= (struct ftgmac100_priv *)dev->priv;
	unsigned int tmp_rsize;		//Richard
	unsigned int rfifo_rsize;	//Richard
	unsigned int tfifo_rsize;	//Richard
	unsigned int rxbuf_size;
  unsigned long Package_Found = 0, Channel_Found = 0, Re_Send = 0, Link_Status;

	rxbuf_size = RX_BUF_SIZE & 0x3fff;
	outl( rxbuf_size , dev->base_addr + RBSR_REG); //for NC Body

	for (i=0; i<RXDES_NUM; ++i)
		priv->rx_descs[i].RXPKT_RDY = RX_OWNBY_FTGMAC100;				// owned by FTMAC100

	priv->rx_idx = 0;

	for (i=0; i<TXDES_NUM; ++i) {
		priv->tx_descs[i].TXDMA_OWN = TX_OWNBY_SOFTWARE;			// owned by software
		priv->tx_skbuff[i] = 0;
	}

	priv->tx_idx = 0;
	priv->old_tx = 0;
	priv->tx_free=TXDES_NUM;

	/* Set the MAC address */
	ast_gmac_set_mac(priv, dev->dev_addr);

	outl( priv->rx_descs_dma, dev->base_addr + RXR_BADR_REG);
	outl( priv->tx_descs_dma, dev->base_addr + TXR_BADR_REG);
	outl( 0x00001010, dev->base_addr + ITC_REG);

	outl( (0UL<<TXPOLL_CNT)|(0x1<<RXPOLL_CNT), dev->base_addr + APTC_REG);
	outl( 0x44f97, dev->base_addr + DBLAC_REG );

	///	outl( inl(FCR_REG)|0x1, ioaddr + FCR_REG );				// enable flow control
	///	outl( inl(BPR_REG)|0x1, ioaddr + BPR_REG );				// enable back pressure register

	// +++++ Richard +++++ //
	tmp_rsize = inl( dev->base_addr + FEAR_REG );
	rfifo_rsize = tmp_rsize & 0x00000007;
	tfifo_rsize = (tmp_rsize >> 3)& 0x00000007;

	tmp_rsize = inl( dev->base_addr + TPAFCR_REG );
	tmp_rsize &= ~0x3f000000;
	tmp_rsize |= (tfifo_rsize << 27);
	tmp_rsize |= (rfifo_rsize << 24);

	outl(tmp_rsize, dev->base_addr + TPAFCR_REG);
	// ----- Richard ----- //

//river set MAHT0, MAHT1
	if (priv->maccr_val & GMAC_MODE_bit) {
		outl (priv->GigaBit_MAHT0, dev->base_addr + MAHT0_REG);
		outl (priv->GigaBit_MAHT1, dev->base_addr + MAHT1_REG);
	}
	else {
		outl (priv->Not_GigaBit_MAHT0, dev->base_addr + MAHT0_REG);
		outl (priv->Not_GigaBit_MAHT1, dev->base_addr + MAHT1_REG);
	}

	/// enable trans/recv,...
	outl(priv->maccr_val, dev->base_addr + MACCR_REG );

//NCSI Start
//DeSelect Package/ Select Package
	if ((priv->NCSI_support == 1) || (priv->INTEL_NCSI_EVA_support == 1)) {
		NCSI_Struct_Initialize(dev);
		for (i = 0; i < 4; i++) {
			DeSelect_Package (dev, i);
			Package_Found = Select_Package (dev, i);
			if (Package_Found == 1) {
//AST2100/AST2050/AST1100 supports 1 slave only
				priv->NCSI_Cap.Package_ID = i;
				break;
			}
		}
		if (Package_Found != 0) {
//Initiali State
			for (i = 0; i < 1; i++) { //Suppose 2 channels in current version, You could modify it to 0x1F to support 31 channels
				//Channel_Found = Clear_Initial_State(dev, i);
        Channel_Found = 1;
				if (Channel_Found == 1) {
					priv->NCSI_Cap.Channel_ID = i;
					printk ("Found NCSI Network Controller at (%d, %d)\n", priv->NCSI_Cap.Package_ID, priv->NCSI_Cap.Channel_ID);
          Get_MAC_Address(dev);
          Set_MAC_Affinity(dev);
				  Clear_Initial_State(dev, i);
//Get Version and Capabilities
					Get_Version_ID(dev);
					Get_Capabilities(dev);
//Set MAC Address
					Enable_Set_MAC_Address(dev);
//Enable Broadcast Filter
					Enable_Broadcast_Filter(dev);
//Disable Multicast Filter
          Disable_Multicast_Filter(dev);
//Disable VLAN
					Disable_VLAN(dev);
//Enable AEN
					//Enable_AEN(dev);
//Get Parameters
					//Get_Parameters(dev);
//Enable TX
					Enable_Network_TX(dev);
//Enable Channel
					Enable_Channel(dev);
//Get Link Status
Re_Get_Link_Status:
          //TODO: Workaround for CX4 Link status issue
					//Link_Status = Get_Link_Status(dev);
          Link_Status = LINK_UP;
					if (Link_Status == LINK_UP) {
						printk ("Using NCSI Network Controller (%d, %d)\n", priv->NCSI_Cap.Package_ID, priv->NCSI_Cap.Channel_ID);
						netif_carrier_on(dev);
						break;
					}
					else if ((Link_Status == LINK_DOWN) && (Re_Send < 2)) {
						Re_Send++;
						netif_carrier_off(dev);
						goto Re_Get_Link_Status;
					}
//Disable TX
					Disable_Network_TX(dev);
//Disable Channel
//					Disable_Channel(dev);
					Re_Send = 0;
					Channel_Found = 0;
				}
			}
		}
    // Enable Interrupts
		outl(
			AHB_ERR_bit			|
			TPKT_LOST_bit			|
			TPKT2E_bit			|
			RXBUF_UNAVA_bit		|
			RPKT2B_bit
        	,dev->base_addr + IER_REG
       	 	);

    return;
	}
	/* now, enable interrupts */
	if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
	    ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
		outl(
			PHYSTS_CHG_bit		|
			AHB_ERR_bit			|
			TPKT_LOST_bit		|
			TPKT2E_bit			|
			RXBUF_UNAVA_bit		|
			RPKT2B_bit
        	,dev->base_addr + IER_REG
       	 	);
	}
	else if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) ||
		 ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL)) {
		outl(
			AHB_ERR_bit			|
			TPKT_LOST_bit		|
			TPKT2E_bit			|
			RXBUF_UNAVA_bit		|
			RPKT2B_bit
        	,dev->base_addr + IER_REG
       	 	);
	}
	else if (priv->ids.miiPhyId == PHYID_BCM54612E
		 || priv->ids.miiPhyId == PHYID_BCM54616S) {
        outl(
// no link PHY link status pin            PHYSTS_CHG_bit      |
            AHB_ERR_bit         |
            TPKT_LOST_bit       |
            TPKT2E_bit          |
            RXBUF_UNAVA_bit     |
            RPKT2B_bit
            ,dev->base_addr + IER_REG
            );
    } else {
		outl(
// no link PHY link status pin			  PHYSTS_CHG_bit	  |
			AHB_ERR_bit 		|
			TPKT_LOST_bit		|
			TPKT2E_bit			|
			RXBUF_UNAVA_bit 	|
			RPKT2B_bit
			,dev->base_addr + IER_REG
			);
    }
}

static void aspeed_mac_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
	unsigned int status, tmp, speed, duplex, macSpeed;

#ifdef CONFIG_ARCH_AST2300
	//Fix issue for tx/rx arbiter lock
	outl( 0xffffffff, dev->base_addr + TXPD_REG);
#endif
	status = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x01);

	if (status & LINK_STATUS) { // Bit[2], Link Status, link is up
	    priv->timer.expires = jiffies + 10 * HZ;

	if ((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) {
	    tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x18);
	    duplex = (tmp & 0x0001);
	    speed  = (tmp & 0x0002) ? PHY_SPEED_100M : PHY_SPEED_10M;
	}
	else if ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL) {
	    tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x00);
	    duplex = (tmp & 0x0100) ? 1 : 0;
	    speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
	}
	else if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
		 ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
	    tmp    = ftgmac100_read_phy_register(dev->base_addr, priv->ids.phyAddr, 0x11);
	    duplex = (tmp & PHY_DUPLEX_mask)>>13;
	    speed  = (tmp & PHY_SPEED_mask)>>14;
	}
	else if (priv->ids.miiPhyId == PHYID_BCM54612E
		 || priv->ids.miiPhyId == PHYID_BCM54616S) {
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
	else {
	    duplex = 1;  speed = PHY_SPEED_100M;
	}

	macSpeed = ((priv->maccr_val & GMAC_MODE_bit)>>8   // Move bit[9] to bit[1]
		    | (priv->maccr_val & SPEED_100_bit)>>19);  // bit[19] to bit[0]
	// The MAC hardware ignores SPEED_100_bit if GMAC_MODE_bit is set.
	if (macSpeed > PHY_SPEED_1G) macSpeed = PHY_SPEED_1G;	// 0x3 --> 0x2

	if ( ((priv->maccr_val & FULLDUP_bit)!=0) != duplex
	     || macSpeed != speed )
	{
		PRINTK("%s:aspeed_mac_timer, priv->maccr_val=0x%05x, PHY {speed,duplex}=%d,%d\n",
		   dev->name, priv->maccr_val, speed, duplex);
		ftgmac100_reset(dev);
		ftgmac100_enable(dev);
	}
	    netif_carrier_on(dev);
	}
	else {
	netif_carrier_off(dev);
	    priv->timer.expires = jiffies + 1 * HZ;
	}
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
	///interrupt mask register
	outl( 0, ioaddr + IER_REG );
	/* enable trans/recv,... */
	outl( 0, ioaddr + MACCR_REG );
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
	struct ftgmac100_priv *priv 	= (struct ftgmac100_priv *)dev->priv;
	unsigned long ioaddr 	= dev->base_addr;
	volatile TX_DESC *cur_desc;
	int		length;
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock,flags);

	if (skb==NULL)
	{
		DO_PRINT("%s(%d): NULL skb???\n", __FILE__,__LINE__);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return 0;
	}

	PRINTK3("%s:ftgmac100_wait_to_send_packet, skb=%x\n", dev->name, skb);
	cur_desc = &priv->tx_descs[priv->tx_idx];

#ifdef not_complete_yet
	if (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)		/// no empty transmit descriptor
	{
		DO_PRINT("no empty transmit descriptor\n");
		DO_PRINT("jiffies = %d\n", jiffies);
		priv->stats.tx_dropped++;
		netif_stop_queue(dev);		/// waiting to do:
		spin_unlock_irqrestore(&priv->tx_lock, flags);

		return 1;
   	}
#endif /* end_of_not */

	if (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)		/// no empty transmit descriptor
	{
		DO_PRINT("no empty TX descriptor:0x%x:0x%x\n",
				(unsigned int)cur_desc,((unsigned int *)cur_desc)[0]);
		priv->stats.tx_dropped++;
		netif_stop_queue(dev);		/// waiting to do:
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return 1;
   	}
	priv->tx_skbuff[priv->tx_idx] = skb;
	length = (ETH_ZLEN < skb->len) ? skb->len : ETH_ZLEN;
	length = min(length, TX_BUF_SIZE);

#if FTMAC100_DEBUG > 2
	DO_PRINT("Transmitting Packet at 0x%x, skb->data = %x, len = %x\n",
		 (unsigned int)cur_desc->VIR_TXBUF_BADR, skb->data, length);
	print_packet( skb->data, length );
#endif

	cur_desc->VIR_TXBUF_BADR = (unsigned long)skb->data;
	cur_desc->TXBUF_BADR = virt_to_phys(skb->data);
#ifndef CONFIG_CPU_FA52x_DCE
	dmac_clean_range((void *)skb->data, (void *)(skb->data + length));
#endif

	//clean_dcache_range(skb->data, (char*)(skb->data + length));

	cur_desc->TXBUF_Size = length;
	cur_desc->LTS = 1;
	cur_desc->FTS = 1;

	cur_desc->TX2FIC = 0;
	cur_desc->TXIC = 0;

	cur_desc->TXDMA_OWN = TX_OWNBY_FTGMAC100;

	outl( 0xffffffff, ioaddr + TXPD_REG);

	priv->tx_idx = (priv->tx_idx + 1) % TXDES_NUM;
	priv->stats.tx_packets++;
	priv->tx_free--;

	if (priv->tx_free <= 0) {
		netif_stop_queue(dev);

	}


	dev->trans_start = jiffies;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	return 0;
}

static int ftgmac100_ringbuf_alloc(struct ftgmac100_priv *priv)
{
	int i;
	struct sk_buff *skb;

    priv->rx_descs = dma_alloc_coherent(priv->dev,
								sizeof(RX_DESC)*RXDES_NUM,
								&priv->rx_descs_dma, GFP_KERNEL);

	if(!priv->rx_descs)
		return -ENOMEM;

	memset(priv->rx_descs, 0, sizeof(RX_DESC)*RXDES_NUM);
	priv->rx_descs[RXDES_NUM-1].EDORR = 1;

	for (i=0; i<RXDES_NUM; i++) {
		dma_addr_t mapping;
		skb = dev_alloc_skb(RX_BUF_SIZE + NET_IP_ALIGN);
		skb_reserve(skb, NET_IP_ALIGN);

		priv->rx_skbuff[i] = skb;
		if (skb == NULL) {
			printk ("alloc_list: allocate Rx buffer error! ");
			break;
		}
		mapping = dma_map_single(priv->dev, skb->data, skb->len, DMA_FROM_DEVICE);
		skb->dev = priv->netdev;	/* Mark as being used by this device. */
		priv->rx_descs[i].RXBUF_BADR = mapping;
		priv->rx_descs[i].VIR_RXBUF_BADR = skb->data;
	}

	priv->tx_descs = dma_alloc_coherent(priv->dev,
						sizeof(TX_DESC)*TXDES_NUM,
						&priv->tx_descs_dma ,GFP_KERNEL);

	if(!priv->tx_descs)
		return -ENOMEM;

	memset((void*)priv->tx_descs, 0, sizeof(TX_DESC)*TXDES_NUM);
	priv->tx_descs[TXDES_NUM-1].EDOTR = 1;			// is last descriptor

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
    struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
    unsigned long ioaddr = dev->base_addr;
	u32 tmp;
//	printk("priv->ids.miiPhyId = %x \n",priv->ids.miiPhyId);
	switch (priv->ids.miiPhyId & PHYID_VENDOR_MASK) {
		case PHYID_VENDOR_MARVELL:
			ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x12, 0x4400);
			tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13 );
			break;
		case PHYID_VENDOR_REALTEK:
			switch (priv->ids.miiPhyId) {
				case PHYID_RTL8211:
					ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x12, 0x4400);
					tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13 );
					break;
				case PHYID_RTL8201EL:
					break;
				case PHYID_RTL8201F:
					ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1f, 0x0007);
					tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13 );
					tmp &= ~(0x0030);
					tmp |= 0x0008;
					ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x13, (u16) tmp);
					tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x11);
					tmp &= ~(0x0fff);
					tmp |= 0x0008;
					ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x11, (u16) tmp);
					ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1f, 0x0000);
					break;
			}
			break;
		case PHYID_VENDOR_BROADCOM:
			tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x1b);
			tmp |= 0x0004;
			ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1b, (u16) tmp);
			break;
		case PHYID_VENDOR_BCM5461X:
			ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x1C, 0x8C00); // Disable GTXCLK Clock Delay Enable
			ftgmac100_write_phy_register(ioaddr, priv->ids.phyAddr, 0x18, 0xF0E7); // Disable RGMII RXD to RXC Skew
			break;
		}
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
 struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
 int entry = priv->old_tx % TXDES_NUM;
 unsigned long flags = 0;

 spin_lock_irqsave(&priv->tx_lock,flags);

 /* Free used tx skbuffs */

  while ((priv->tx_descs[entry].TXDMA_OWN == TX_OWNBY_SOFTWARE) && (priv->tx_skbuff[entry] != NULL)) {
  struct sk_buff *skb;

  skb = priv->tx_skbuff[entry];
         dev_kfree_skb_any (skb);
  priv->tx_skbuff[entry] = 0;
  entry = (entry + 1) % TXDES_NUM;
  priv->tx_free++;
 }

 spin_unlock_irqrestore(&priv->tx_lock, flags);
 priv->old_tx = entry;
 if ((netif_queue_stopped(dev)) && (priv->tx_free > 0)) {
 	netif_wake_queue (dev);
 }
}


/*-------------------------------------------------------------
 .
 . ftgmac100_rcv -  receive a packet from the card
 .
 . There is ( at least ) a packet waiting to be read from
 . chip-memory.
 .
 . o Read the status
 . o If an error, record it
 . o otherwise, read in the packet
 --------------------------------------------------------------
*/
// extern dce_dcache_invalidate_range(unsigned int start, unsigned int end);

static void ftgmac100_rcv(struct net_device *dev)
{
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
	unsigned long ioaddr 	= dev->base_addr;
	int 	packet_length;
	int 	rcv_cnt;
	volatile RX_DESC *cur_desc;
	int		cur_idx;
	int		have_package;
	int		have_frs;
	int	 	start_idx;
        int             count = 0;
        int             packet_full = 0;
	int		data_not_fragment = 1;

	start_idx = priv->rx_idx;

	for (rcv_cnt=0; rcv_cnt<RXDES_NUM ; ++rcv_cnt)
	{
		packet_length = 0;
		cur_idx = priv->rx_idx;

		have_package = 0;
		have_frs = 0;

		for (; (cur_desc = &priv->rx_descs[priv->rx_idx])->RXPKT_RDY==RX_OWNBY_SOFTWARE; )
		{
			have_package = 1;
			priv->rx_idx = (priv->rx_idx+1)%RXDES_NUM;
                        count++;
                        if (count == RXDES_NUM) {
                            packet_full = 1;
                        }
//DF_support
                            if (data_not_fragment == 1) {
                                if (!(cur_desc->DF)) {
                                    data_not_fragment = 0;
                                }
                            }

			if (cur_desc->FRS)
			{
				have_frs = 1;
				if (cur_desc->RX_ERR || cur_desc->CRC_ERR || cur_desc->FTL ||
				    cur_desc->RUNT || cur_desc->RX_ODD_NB
				    // cur_desc->IPCS_FAIL || cur_desc->UDPCS_FAIL || cur_desc->TCPCS_FAIL
				    )
				{
				//	#ifdef not_complete_yet
					if (cur_desc->RX_ERR)
					{
						DO_PRINT("err: RX_ERR\n");
					}
					if (cur_desc->CRC_ERR)
					{
				//		DO_PRINT("err: CRC_ERR\n");
					}
					if (cur_desc->FTL)
					{
						DO_PRINT("err: FTL\n");
					}
					if (cur_desc->RX_ODD_NB)
					{
				//		DO_PRINT("err: RX_ODD_NB\n");
				        }
//				        if (cur_desc->IPCS_FAIL || cur_desc->UDPCS_FAIL || cur_desc->TCPCS_FAIL)
//				        {
//				        	DO_PRINT("err: CS FAIL\n");
//				        }
				//	#endif /* end_of_not */
					priv->stats.rx_errors++;			// error frame....
					break;
				}
//DF_support
                                    if (cur_desc->DF) {
				        if (cur_desc->IPCS_FAIL || cur_desc->UDPCS_FAIL || cur_desc->TCPCS_FAIL)
				        {
				        	DO_PRINT("err: CS FAIL\n");
					        priv->stats.rx_errors++;			// error frame....
                                                break;
				        }
                                    }

				if (cur_desc->MULTICAST)
				{
					priv->stats.multicast++;
				}
				if ((priv->NCSI_support == 1) || (priv->INTEL_NCSI_EVA_support == 1)) {
					if (cur_desc->BROADCAST) {
						if (*(unsigned short *)(cur_desc->VIR_RXBUF_BADR + 12) == NCSI_HEADER) {
							printk ("AEN PACKET ARRIVED\n");
							ftgmac100_reset(dev);
							ftgmac100_enable(dev);
							return;
						}
					}
				}
			}

			packet_length += cur_desc->VDBC;

//			if ( cur_desc->LRS )		// packet's last frame
//			{
				break;
//			}
		}
		if (have_package==0)
		{
			goto done;
		}
		if (!have_frs)
		{
			DO_PRINT("error, loss first\n");
			priv->stats.rx_over_errors++;
		}

		if (packet_length > 0)
		{
			struct sk_buff  * skb;
			u8		* data = 0;  if (data) { }

			packet_length -= 4;

			skb_put (skb = priv->rx_skbuff[cur_idx], packet_length);

// Rx Offload DF_support

			    if (data_not_fragment) {
				skb->ip_summed = CHECKSUM_UNNECESSARY;
                                data_not_fragment = 1;
                            }

#if FTMAC100_DEBUG > 2
			DO_PRINT("Receiving Packet at 0x%x, packet len = %x\n",(unsigned int)data, packet_length);
			print_packet( data, packet_length );
#endif

			skb->protocol = eth_type_trans(skb, dev );
			netif_rx(skb);
			priv->stats.rx_packets++;
			priv->rx_skbuff[cur_idx] = NULL;
		}
                if (packet_full) {
//                  DO_PRINT ("RX Buffer full before driver entered ISR\n");
                    goto done;
                }
	}

done:

        if (packet_full) {

		struct sk_buff *skb;

		for (cur_idx = 0; cur_idx < RXDES_NUM; cur_idx++)
		{
			if (priv->rx_skbuff[cur_idx] == NULL) {
				skb = dev_alloc_skb (RX_BUF_SIZE + 16);
				if (skb == NULL) {
					printk (KERN_INFO
						"%s: receive_packet: "
						"Unable to re-allocate Rx skbuff.#%d\n",
						dev->name, cur_idx);
				}
				priv->rx_skbuff[cur_idx] = skb;
				skb->dev = dev;
				// ASPEED: See earlier skb_reserve() cache alignment
				skb_reserve (skb, 2);
				dmac_inv_range ((void *)skb->data, (void *)skb->data + RX_BUF_SIZE);
				priv->rx_descs[cur_idx].RXBUF_BADR = cpu_to_le32(virt_to_phys(skb->tail));
				priv->rx_descs[cur_idx].VIR_RXBUF_BADR = cpu_to_le32((u32)skb->tail);
			}
		        priv->rx_descs[cur_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
                }
                packet_full = 0;

        }
	else {
            if (start_idx != priv->rx_idx) {
		struct sk_buff *skb;

		for (cur_idx = (start_idx+1)%RXDES_NUM; cur_idx != priv->rx_idx; cur_idx = (cur_idx+1)%RXDES_NUM)
		{


			//struct sk_buff *skb;
			/* Dropped packets don't need to re-allocate */
			if (priv->rx_skbuff[cur_idx] == NULL) {
				skb = dev_alloc_skb (RX_BUF_SIZE + 16);
				if (skb == NULL) {
					printk (KERN_INFO
						"%s: receive_packet: "
						"Unable to re-allocate Rx skbuff.#%d\n",
						dev->name, cur_idx);
					break;
				}
				priv->rx_skbuff[cur_idx] = skb;
				skb->dev = dev;
				/* 16 byte align the IP header */
				skb_reserve (skb, 2);
				dmac_inv_range ((void *)skb->data,
						(void *)skb->data + RX_BUF_SIZE);
				priv->rx_descs[cur_idx].RXBUF_BADR = cpu_to_le32(virt_to_phys(skb->tail));
				priv->rx_descs[cur_idx].VIR_RXBUF_BADR = cpu_to_le32((u32)skb->tail);
			}

			priv->rx_descs[cur_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
		}


			//struct sk_buff *skb;
			/* Dropped packets don't need to re-allocate */
			if (priv->rx_skbuff[start_idx] == NULL) {
				skb = dev_alloc_skb (RX_BUF_SIZE + 16);
				if (skb == NULL) {
					printk (KERN_INFO
						"%s: receive_packet: "
						"Unable to re-allocate Rx skbuff.#%d\n",
						dev->name, start_idx);
				}
				priv->rx_skbuff[start_idx] = skb;
				skb->dev = dev;
				/* 16 byte align the IP header */
				skb_reserve (skb, 2);
				dmac_inv_range ((void *)skb->data,
						(void *)skb->data + RX_BUF_SIZE);
				priv->rx_descs[start_idx].RXBUF_BADR = cpu_to_le32(virt_to_phys(skb->tail));
				priv->rx_descs[start_idx].VIR_RXBUF_BADR = cpu_to_le32((u32)skb->tail);
			}


		priv->rx_descs[start_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
	    }
    }
	if (trans_busy == 1)
	{
		/// priv->maccr_val |= RXMAC_EN_bit;
		outl( priv->maccr_val, ioaddr + MACCR_REG );
		outl( inl(ioaddr + IER_REG) | RXBUF_UNAVA_bit, ioaddr + IER_REG);
	}
	return;
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
static irqreturn_t ftgmac100_interrupt(int irq, void * dev_id,  struct pt_regs * regs)
{
	struct net_device *dev 	= dev_id;
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
	unsigned long ioaddr 	= dev->base_addr;
	int		timeout;
	unsigned int tmp;
	unsigned int	mask;			// interrupt mask
	unsigned int	status;			// interrupt status

//	PRINTK3("%s: ftgmac100 interrupt started \n", dev->name);

	if (dev == NULL) {
		DO_PRINT(KERN_WARNING "%s: irq %d for unknown device.\n",	dev->name, irq);
		return IRQ_HANDLED;
	}

	/* read the interrupt status register */
	mask = inl( ioaddr + IER_REG );

	/* set a timeout value, so I don't stay here forever */

	for (timeout=1; timeout>0; --timeout)
	{
		/* read the status flag, and mask it */
		status = inl( ioaddr + ISR_REG ) & mask;

		outl(status, ioaddr + ISR_REG ); //Richard, write to clear

		if (!status )
		{
			break;
		}

		if (status & PHYSTS_CHG_bit) {
		    DO_PRINT("PHYSTS_CHG \n");
		    // Is this interrupt for changes of the PHYLINK pin?
		    // Note: PHYLINK is optional; not all boards connect it.
		    if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
		    	((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211))
		    {
			tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x13);
			PRINTK("%s: PHY interrupt status, read_phy_reg(0x13) = 0x%04x\n",
			       dev->name, tmp);
			tmp &= (PHY_SPEED_CHG_bit | PHY_DUPLEX_CHG_bit | PHY_LINK_CHG_bit);
		    }
		    else if ((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM)
		    {
			tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x1a);
			PRINTK("%s: PHY interrupt status, read_phy_reg(0x1a) = 0x%04x\n",
			       dev->name, tmp);
			// Bits [3:1] are {duplex, speed, link} change interrupts.
			tmp &= 0x000e;
		    }
		    else if (priv->ids.miiPhyId == PHYID_BCM54612E
			     || priv->ids.miiPhyId == PHYID_BCM54616S) {
                tmp = ftgmac100_read_phy_register(ioaddr, priv->ids.phyAddr, 0x1A);
                PRINTK("%s: PHY interrupt status, read_phy_reg(0x1A) = 0x%04x\n",
                    dev->name, tmp);
                tmp &= 0x000E;
            }
		    else tmp = 0;

		    if (tmp) {
			ftgmac100_reset(dev);
			ftgmac100_enable(dev);
		    }
		}

#ifdef not_complete_yet
		if (status & AHB_ERR_bit)
		{
			DO_PRINT("AHB_ERR \n");
		}

		if (status & RPKT_LOST_bit)
		{
			DO_PRINT("RPKT_LOST ");
		}
		if (status & RPKT2F_bit)
		{
			PRINTK2("RPKT_SAV ");
		}

		if (status & TPKT_LOST_bit)
		{
			PRINTK("XPKT_LOST ");
		}
		if (status & TPKT2E_bit)
		{
			PRINTK("XPKT_OK ");
		}
		if (status & NPTXBUF_UNAVA_bit)
		{
			PRINTK("NOTXBUF ");
		}
		if (status & TPKT2F_bit)
		{
			PRINTK("XPKT_FINISH ");
		}

		if (status & RPKT2B_bit)
		{
			DO_PRINT("RPKT_FINISH ");
		}
		PRINTK2("\n");
#endif /* end_of_not */

//		PRINTK3(KERN_WARNING "%s: Handling interrupt status %x \n",	dev->name, status);

		if ( status & (TPKT2E_bit|TPKT_LOST_bit))
		{
			//free tx skb buf
			ftgmac100_free_tx(dev);

		}

		if ( status & RPKT2B_bit )
		{
			ftgmac100_rcv(dev); //Richard
		}
		else if (status & RXBUF_UNAVA_bit)
		{
			outl( mask & ~RXBUF_UNAVA_bit, ioaddr + IER_REG);
			trans_busy = 1;
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
	}

//	PRINTK3("%s: Interrupt done\n", dev->name);
	return IRQ_HANDLED;
}

/*------------------------------------------------------------
 . Get the current statistics.
 . This may be called with the card open or closed.
 .-------------------------------------------------------------*/
static struct net_device_stats* ftgmac100_query_statistics(struct net_device *dev)
{
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;

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
			crc_value = (crc_value>>1)^
				(((crc_value^c)&0x01)?poly:0);
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
	struct dev_mc_list	* cur_addr;
	int crc_val;
	unsigned int	ioaddr = dev->base_addr;
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;
	struct AstMacHwConfig* ids = &priv->ids;
	unsigned long Combined_Channel_ID, i;
	struct sk_buff * skb;
	cur_addr = addrs;

//TX
#if 1
	if (priv->NCSI_support == 1) {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		priv->InstanceID++;
		priv->NCSI_Request.IID = priv->InstanceID;
		priv->NCSI_Request.Command = SET_MAC_ADDRESS;
		Combined_Channel_ID = (priv->NCSI_Cap.Package_ID << 5) + priv->NCSI_Cap.Channel_ID;
		priv->NCSI_Request.Channel_ID = Combined_Channel_ID;
		priv->NCSI_Request.Payload_Length = (8 << 8);
		memcpy ((unsigned char *)skb->data, &priv->NCSI_Request, 30);
		priv->NCSI_Request.Payload_Length = 8;
		for (i = 0; i < 6; i++) {
			priv->Payload_Data[i] = cur_addr->dmi_addr[i];
		}
		priv->Payload_Data[6] = 2; //MAC Address Num = 1 --> address filter 1, fixed in sample code
		priv->Payload_Data[7] = MULTICAST_ADDRESS + 0 + ENABLE_MAC_ADDRESS_FILTER; //AT + Reserved + E
		copy_data (dev, skb, priv->NCSI_Request.Payload_Length);
		skb->len =  30 + priv->NCSI_Request.Payload_Length + 4;
		ftgmac100_wait_to_send_packet(skb, dev);
	}
#endif
	for (cur_addr = addrs ; cur_addr!=NULL ; cur_addr = cur_addr->next )
	{
		/* make sure this is a multicast address - shouldn't this be a given if we have it here ? */
		if ( !( *cur_addr->dmi_addr & 1 ) )
		{
			continue;
		}
#if 1
//A0, A1
			crc_val = crc32( cur_addr->dmi_addr, 5 );
			crc_val = (~(crc_val>>2)) & 0x3f;
			if (crc_val >= 32)
			{
				outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
				priv->GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
			}
			else
			{
				outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
				priv->GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
			}
//10/100M
			crc_val = crc32( cur_addr->dmi_addr, 6 );
			crc_val = (~(crc_val>>2)) & 0x3f;
			if (crc_val >= 32)
			{
				outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
				priv->Not_GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
			}
			else
			{
				outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
				priv->Not_GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
			}
#else
//A2
			crc_val = crc32( cur_addr->dmi_addr, 6 );
			crc_val = (~(crc_val>>2)) & 0x3f;
			if (crc_val >= 32)
			{
				outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
				priv->Not_GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
				priv->GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
			}
			else
			{
				outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
				priv->Not_GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
				priv->GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
			}
#endif
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
	unsigned int ioaddr = dev->base_addr;
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;

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

	outl( priv->maccr_val, ioaddr + MACCR_REG );

}
#endif

static int ast_gmac_stop(struct net_device *dev)
{
	struct ftgmac100_priv *priv = (struct ftgmac100_priv *)dev->priv;

	netif_stop_queue(dev);

	/* clear everything */
	ftgmac100_shutdown(dev->base_addr);
	free_irq(dev->irq, dev);

	if (priv->timer.function != NULL) {
	    del_timer_sync(&priv->timer);
	}

	if (priv->rx_descs)
			dma_free_coherent( NULL, sizeof(RX_DESC)*RXDES_NUM, (void*)priv->rx_descs, (dma_addr_t)priv->rx_descs_dma );
	if (priv->tx_descs)
			dma_free_coherent( NULL, sizeof(TX_DESC)*TXDES_NUM, (void*)priv->tx_descs, (dma_addr_t)priv->tx_descs_dma );
	if (priv->tx_buf)
			dma_free_coherent( NULL, TX_BUF_SIZE*TXDES_NUM, (void*)priv->tx_buf, (dma_addr_t)priv->tx_buf_dma );
	priv->rx_descs = NULL; priv->rx_descs_dma = 0;
	priv->tx_descs = NULL; priv->tx_descs_dma = 0;
	priv->tx_buf   = NULL; priv->tx_buf_dma   = 0;


	return 0;
}

static struct proc_dir_entry *proc_ftgmac100;

static int ftgmac100_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ftgmac100_priv *priv 	= (struct ftgmac100_priv *)dev->priv;
	int num;
	int i;

	num = sprintf(page, "priv->rx_idx = %d\n", priv->rx_idx);
	for (i=0; i<RXDES_NUM; ++i)
	{
		num += sprintf(page + num, "[%d].RXDMA_OWN = %d\n", i, priv->rx_descs[i].RXPKT_RDY);
	}
	return num;
}

static int ftgmac100_open(struct net_device *netdev)
{
	struct ftgmac100_priv *priv = netdev_priv(netdev);
	int err;

	DO_PRINT("%s:ftgmac100_open\n", netdev->name);

	priv->maccr_val = (CRC_APD_bit | RXMAC_EN_bit | TXMAC_EN_bit  | RXDMA_EN_bit
			 | TXDMA_EN_bit | CRC_CHK_bit | RX_BROADPKT_bit | SPEED_100_bit | FULLDUP_bit);

	ftgmac100_ringbuf_alloc(priv);


	/* Grab the IRQ next.  Beyond this, we will free the IRQ. */
	err = request_irq(netdev->irq, (void *)&ftgmac100_interrupt,
			     IRQF_DISABLED, netdev->name, netdev);
	if (err)
	{
		DO_PRINT("%s: unable to get IRQ %d (retval=%d).\n",
			 netdev->name, netdev->irq, err);
		kfree(netdev->priv);
		netdev->priv = NULL;
		return err;
	}


	netif_start_queue(netdev);

	/* reset the hardware */
	ftgmac100_reset(netdev);
	ftgmac100_enable(netdev);

	if (((priv->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) ||
	    ((priv->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL) ||
        (priv->ids.miiPhyId == PHYID_BCM54612E) ||
	(priv->ids.miiPhyId == PHYID_BCM54616S)) {

        init_timer(&priv->timer);
	    priv->timer.data = (unsigned long)netdev;
	    priv->timer.function = aspeed_mac_timer;
	    priv->timer.expires = jiffies + 1 * HZ;
	    add_timer (&priv->timer);
	}

	/* Configure the PHY */
	ftgmac100_phy_configure(netdev);

	netif_start_queue(netdev);
	return 0;
}

static int __init ast_gmac_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct net_device *netdev;
	struct ftgmac100_priv *priv;
	 struct ftgmac100_eth_data *ast_eth_data = pdev->dev.platform_data;;
	int err;

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


//	SET_ETHTOOL_OPS(netdev, &ftgmac100_ethtool_ops);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
	netdev->netdev_ops 			= &ftgmac100_netdev_ops;
#else
	printk("ast_gmac_probe 5\n");

	ether_setup(netdev);

	netdev->open				= ftgmac100_open;
	netdev->stop				= ast_gmac_stop;
	netdev->hard_start_xmit 	= ftgmac100_wait_to_send_packet;
	netdev->tx_timeout 			= ftgmac100_timeout;
	netdev->get_stats 			= ftgmac100_query_statistics;
#ifdef	HAVE_MULTICAST
	netdev->set_multicast_list 	= &ftgmac100_set_multicast_list;
#endif

#endif


#ifdef CONFIG_AST_NPAI
//	netdev->features = NETIF_F_GRO;
//	netdev->features = NETIF_F_IP_CSUM | NETIF_F_GRO;
#endif

	platform_set_drvdata(pdev, netdev);

	/* setup private data */
	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->dev = &pdev->dev;


	priv->ids.macId = pdev->id;

	priv->NCSI_support = ast_eth_data->NCSI_support;
	priv->INTEL_NCSI_EVA_support= ast_eth_data->INTEL_NCSI_EVA_support;
	spin_lock_init(&priv->tx_lock);

#if 0
	/* initialize NAPI */
	netif_napi_add(netdev, &priv->napi, ftgmac100_poll, 64);
#endif
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

//	printk("irq %d, mapped at %x\n", netdev->irq, (u32)netdev->base_addr);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		random_ether_addr(netdev->dev_addr);
		printk("generated random MAC address %pM\n",
				netdev->dev_addr);
	}
#if 0
	if ((proc_ftgmac100 = create_proc_entry( dev->name, 0, 0 )))
	{
		proc_ftgmac100->read_proc = ftgmac100_read_proc;
		proc_ftgmac100->data = dev;
		proc_ftgmac100->owner = THIS_MODULE;
	}
#endif
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
//	netif_napi_del(&priv->napi);
	platform_set_drvdata(pdev, NULL);
err_netdev:
	free_netdev(netdev);
err_alloc_etherdev:
	return err;

}

static int __devexit ast_gmac_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
//	struct ftgmac100_priv *priv = netdev_priv(dev);

//	remove_proc_entry(dev->name, 0);

	unregister_netdev(dev);

#ifdef CONFIG_MII_PHY
	phy_disconnect(priv->phydev);
	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);
#endif

	iounmap((void __iomem *)dev->base_addr);

#ifdef CONFIG_AST_NPAI
	netif_napi_del(&priv->napi);
#endif

	platform_set_drvdata(pdev, NULL);
	free_netdev(dev);
	return 0;
}

static struct platform_driver ast_gmac_driver = {
	.remove		= __devexit_p(ast_gmac_remove),
	.driver		= {
		.name	= "ast_gmac",
		.owner	= THIS_MODULE,
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
