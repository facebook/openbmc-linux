//-----------------------------------------------------------------------------
// Original version by Faraday handled Faraday GMAC and Marvell PHY:
//  "Faraday FTGMAC Driver, (Linux Kernel 2.6.14) 06/16/06 - by Faraday\n"
// Merged improvements from ASPEED to handle Broadcom PHY, and 1 or 2 ports.
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

static const char version[] =
  "ASPEED FTGMAC Driver, (Linux Kernel 2.6.28.9) 09/13/2010 - by ASPEED\n";

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
#include <mach/hardware.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
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

#define DO_PRINT(args...) printk(CARDNAME ": " args)

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



/*------------------------------------------------------------------------
  .
  . The internal workings of the driver.  If you are changing anything
  . here with the SMC stuff, you should have the datasheet and know
  . what you are doing.
  .
  -------------------------------------------------------------------------*/
#define CARDNAME "FTGMAC100"
// static const char    mac_string[] = "Faraday GMAC";
// #define IPMODULE GMAC
// #define IPNAME   FTGMAC1000
//struct tq_struct rcv_tq;

/*-----------------------------------------------------------------
  .
  .  The driver can be entered at any of the following entry points.
  .
  .------------------------------------------------------------------  */

/*
  . This is called by  register_netdev().  It is responsible for
  . checking the portlist for the FTMAC100 series chipset.  If it finds
  . one, then it will initialize the device, find the hardware information,
  . and sets up the appropriate device parameters.
  . NOTE: Interrupts are *OFF* when this procedure is called.
  .
  . NB:This shouldn't be static since it is referred to externally.
*/

/*
  . This is called by  unregister_netdev().  It is responsible for
  . cleaning up before the driver is finally unregistered and discarded.
*/
void ftgmac100_destructor(struct net_device *dev);

/*
  . The kernel calls this function when someone wants to use the net_device,
  . typically 'ifconfig ethX up'.
*/
static int ftgmac100_open(struct net_device *dev);

/*
  . This is called by the kernel to send a packet out into the net.  it's
  . responsible for doing a best-effort send, but if it's simply not possible
  . to send it, the packet gets dropped.
*/
static void ftgmac100_timeout (struct net_device *dev);
/*
  . This is called by the kernel in response to 'ifconfig ethX down'.  It
  . is responsible for cleaning up everything that the open routine
  . does, and maybe putting the card into a powerdown state.
*/
static int ftgmac100_close(struct net_device *dev);

/*
  . This routine allows the proc file system to query the driver's
  . statistics.
*/
static struct net_device_stats * ftgmac100_query_statistics( struct net_device *dev);

/*
  . Finally, a call to set promiscuous mode ( for TCPDUMP and related
  . programs ) and multicast modes.
*/
static void ftgmac100_set_multicast_list(struct net_device *dev);

/*
  . Configures the PHY through the MII Management interface
*/
static void ftgmac100_phy_configure(struct net_device* dev);

/*
  . TX routine
*/
static int ftgmac100_wait_to_send_packet(struct sk_buff *skb, struct net_device *dev);
/*---------------------------------------------------------------
  .
  . Interrupt level calls..
  .
  ----------------------------------------------------------------*/

/*
  . Handles the actual interrupt
*/
static irqreturn_t ftgmac100_interrupt(int irq, void *, struct pt_regs *regs);
/*
  . This is a separate procedure to handle the receipt of a packet, to
  . leave the interrupt code looking slightly cleaner
*/
inline static void ftgmac100_rcv( struct net_device *dev );


/*
  ------------------------------------------------------------
  .
  . Internal routines
  .
  ------------------------------------------------------------
*/

/*
  . Test if a given location contains a chip, trying to cause as
  . little damage as possible if it's not a SMC chip.
*/
static int ftgmac100_probe(struct net_device *dev);

/*
  . A rather simple routine to print out a packet for debugging purposes.
*/
#if FTMAC100_DEBUG > 2
static void print_packet( byte *, int );
#endif


/* this does a soft reset on the device */
static void ftgmac100_reset( struct net_device* dev );

/* Enable Interrupts, Receive, and Transmit */
static void ftgmac100_enable( struct net_device *dev );


/* Routines to Read and Write the PHY Registers across the
   MII Management Interface
*/

static word ftgmac100_read_phy_register(unsigned int ioaddr, byte phyaddr, byte phyreg);
static void ftgmac100_write_phy_register(unsigned int ioaddr, byte phyaddr, byte phyreg, word phydata);
u8  aspeedi2c_read (u8, u8, u16);
void  i2c_init(u8);
static void ftgmac100_free_tx (struct net_device *dev);

static volatile int trans_busy = 0;
// static char mac1[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x1};
// static char mac2[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x2};
static unsigned long IP_va_base[2] = {IO_ADDRESS(0x1e660000), IO_ADDRESS(0x1e680000)};
static unsigned long IP_va_limit[2] =
{IO_ADDRESS(0x1e660000) + 0x20000, IO_ADDRESS(0x1e680000) + 0x20000};
static unsigned long IP_irq[2] = {IRQ_MAC0, IRQ_MAC1};

// Values 0..31 are the PHY chip address on the MDC+MDIO bus driven by this
// MAC.  Values 32..63 mean this MAC's PHY is on the _MAC1_ MDC+MDIO bus.
// Value 255 prevents this MAC from accessing PHY chip; assume 100 Mbit/s FDX.
static unsigned char IP_phy_addr[2] = { 0x0, 0x0 };

static struct net_device *ftgmac100_netdev[IP_COUNT];
//static struct resource ftgmac100_resource[IP_COUNT];
static unsigned int DF_support = 0;


static void auto_get_mac(int id, char *mac_addr)
{

//FIX MAC ADDRESS
  mac_addr[0] = 0;
  mac_addr[1] = 0x84;
  mac_addr[2] = 0x14;
  mac_addr[3] = 0xA0;
  mac_addr[4] = 0xB0;
  mac_addr[5] = 0x22 + id;

  return;
}

void put_mac(int base, char *mac_addr)
{
  int val;
  val = ((u32)mac_addr[0])<<8 | (u32)mac_addr[1];
  outl(val, base);
  val = ((((u32)mac_addr[2])<<24)&0xff000000) |
    ((((u32)mac_addr[3])<<16)&0xff0000) |
    ((((u32)mac_addr[4])<<8)&0xff00)  |
    ((((u32)mac_addr[5])<<0)&0xff);
  outl(val, base+4);
}

void get_mac(int base, char *mac_addr)
{
  int val;
  val = inl(base);
  mac_addr[0] = (val>>8)&0xff;
  mac_addr[1] = val&0xff;
  val = inl(base+4);
  mac_addr[2] = (val>>24)&0xff;
  mac_addr[3] = (val>>16)&0xff;
  mac_addr[4] = (val>>8)&0xff;
  mac_addr[5] = val&0xff;
}

// --------------------------------------------------------------------
//  Print the Ethernet address
// --------------------------------------------------------------------
/*
  void print_mac(char *mac_addr)
  {
  int i;

  DO_PRINT("MAC ADDR: ");
  for (i = 0; i < 5; i++)
  {
  DO_PRINT("%2.2x:", mac_addr[i] );
  }
  DO_PRINT("%2.2x \n", mac_addr[5] );
  }
*/
// --------------------------------------------------------------------
//  NCSI function
// --------------------------------------------------------------------
void NCSI_Struct_Initialize(struct net_device *dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
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
}

void Calculate_Checksum(struct net_device * dev, unsigned char *buffer_base, int Length)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;

  memcpy ((unsigned char *)(skb->data + 30), &lp->Payload_Data, Length);
  Calculate_Checksum(dev, skb->data + 14, 30 + Length);
  memcpy ((unsigned char *)(skb->data + 30 + Length), &lp->Payload_Checksum, 4);
}

void NCSI_Rx (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long status, length, i = 0;
  volatile RX_DESC *cur_desc;


  cur_desc = &lp->rx_descs[lp->rx_idx];
  do {
    status = cur_desc->RXPKT_RDY;
    i++;
  } while (!(((status & RX_OWNBY_SOFTWARE) != 0) || (i >= NCSI_LOOP)));

  if (i < NCSI_LOOP) {
    if (cur_desc->LRS) {
      length = cur_desc->VDBC;
      memcpy (&lp->NCSI_Respond, (unsigned char *)phys_to_virt(cur_desc->RXBUF_BADR), length);
    }
    lp->rx_descs[lp->rx_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
    lp->rx_idx = (lp->rx_idx+1)%RXDES_NUM;
  }
}

void DeSelect_Package (struct net_device * dev, int Package_ID)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

int Select_Package (struct net_device * dev, int Package_ID)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (CLEAR_INITIAL_STATE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_VERSION_ID | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Get_Capabilities (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_CAPABILITIES | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (AEN_ENABLE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Enable_Set_MAC_Address (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SET_MAC_ADDRESS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Enable_Broadcast_Filter (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    lp->Payload_Data[3] = 0xF; //ARP, DHCP, NetBIOS
    copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
    skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
    ftgmac100_wait_to_send_packet (skb, dev);
//RX
    NCSI_Rx(dev);
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_BROADCAST_FILTERING | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Disable_VLAN (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_VLAN | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Get_Parameters (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_PARAMETERS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
//    printk ("Retry: Command = %x, Response_Code = %x, Resonpd.Command = %x, IID = %x, lp->InstanceID = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code, lp->NCSI_Respond.Command, lp->NCSI_Respond.IID, lp->InstanceID);
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_CHANNEL_NETWORK_TX | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Disable_Network_TX (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_CHANNEL_NETWORK_TX | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Enable_Channel (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (ENABLE_CHANNEL | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

void Disable_Channel (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (DISABLE_CHANNEL | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

int Get_Link_Status (struct net_device * dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (GET_LINK_STATUS | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long Combined_Channel_ID;
  struct sk_buff * skb;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
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
    if (((lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (SET_LINK | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry == RETRY_COUNT)) {
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

// --------------------------------------------------------------------
//  Finds the CRC32 of a set of bytes.
//  Again, from Peter Cammaert's code.
// --------------------------------------------------------------------
static int crc32( char * s, int length )
{
  /* indices */
  int perByte;
  int perBit;
  /* crc polynomial for Ethernet */
  const unsigned long poly = 0xedb88320;
  /* crc value - preinitialized to all 1's */
  unsigned long crc_value = 0xffffffff;

  for ( perByte = 0; perByte < length; perByte ++ ) {
    unsigned char c;

    c = *(s++);
    for ( perBit = 0; perBit < 8; perBit++ ) {
      crc_value = (crc_value>>1)^
        (((crc_value^c)&0x01)?poly:0);
      c >>= 1;
    }
  }
  return  crc_value;
}

static void aspeed_mac_timer(unsigned long data)
{
  struct net_device *dev = (struct net_device *)data;
  struct ftgmac100_local *lp  = (struct ftgmac100_local *)dev->priv;
  unsigned long ioaddr = dev->base_addr;
  unsigned int status, tmp, speed, duplex, macSpeed;

  status     = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x01);

  if (status & LINK_STATUS) { // Bit[2], Link Status, link is up
    lp->timer.expires = jiffies + 10 * HZ;

    if ((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) {
      tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x18);
      duplex = (tmp & 0x0001);
      speed  = (tmp & 0x0002) ? PHY_SPEED_100M : PHY_SPEED_10M;
    }
    else if ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL) {
      tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x00);
      duplex = (tmp & 0x0100) ? 1 : 0;
      speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
    }
    else if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
             ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
      tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x11);
      duplex = (tmp & PHY_DUPLEX_mask)>>13;
      speed  = (tmp & PHY_SPEED_mask)>>14;
    }
    else if (lp->ids.miiPhyId == PHYID_BCM54612E) {
      // Get link status
      // First Switch shadow register selector
      ftgmac100_write_phy_register(ioaddr, lp->ids.phyAddr, 0x1C, 0x2000);
      tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x1C);
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

    macSpeed = ((lp->maccr_val & GMAC_MODE_bit)>>8   // Move bit[9] to bit[1]
                | (lp->maccr_val & SPEED_100_bit)>>19);  // bit[19] to bit[0]
    // The MAC hardware ignores SPEED_100_bit if GMAC_MODE_bit is set.
    if (macSpeed > PHY_SPEED_1G) macSpeed = PHY_SPEED_1G; // 0x3 --> 0x2

    if ( ((lp->maccr_val & FULLDUP_bit)!=0) != duplex
         || macSpeed != speed )
    {
      PRINTK("%s:aspeed_mac_timer, lp->maccr_val=0x%05x, PHY {speed,duplex}=%d,%d\n",
             dev->name, lp->maccr_val, speed, duplex);
      ftgmac100_reset(dev);
      ftgmac100_enable(dev);
    }
    netif_carrier_on(dev);
  }
  else {
    netif_carrier_off(dev);
    lp->timer.expires = jiffies + 1 * HZ;
  }
  add_timer(&lp->timer);
}


static void getMacAndPhy( struct net_device* dev, struct AstMacHwConfig* out )
{
  unsigned int macId = 1;

  while (macId > 0 && dev->base_addr != IP_va_base[macId])
    macId --;

  out->macId   = macId;
  out->phyAddr = IP_phy_addr[macId];
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned int regVal;
  unsigned int resetControl, hwStrap_macMode, multiFuncPin;
  unsigned long ioaddr = dev->base_addr;

  getMacAndPhy(dev, out);
  out->miiPhyId = 0;

  regVal = inl(IO_ADDRESS(AST_SCU_BASE + SCU_SILICON_REVISION_REG)) & 0xFF;
  if (regVal >= 0x08 && regVal <= 0x0f) {
    // AST2100 FPGA board: up to 10 means rev.A0, 11 means rev.A1
    out->isRevA0 = (regVal < 11);
  } else {
    // Real silicon: rev.A0 has 0x00 in bits[7:0]. rev A2 = 0x02 in bits[7:0]
    out->isRevA0 = ((regVal & 0x00ff) == 0x00);
    out->isRevA2 = ((regVal & 0x00ff) == 0x02);
  }
  DF_support = !out->isRevA0;
/*
 * D[15:11] in 0x1E6E2040 is NCSI scratch from U-Boot. D[15:14] = MAC1, D[13:12] = MAC2
 * The meanings of the 2 bits are:
 * 00(0): Dedicated PHY
 * 01(1): ASPEED's EVA + INTEL's NC-SI PHY chip EVA
 * 10(2): ASPEED's MAC is connected to NC-SI PHY chip directly
 * 11: Reserved
 */
  if (dev->irq == 2) { //We don't have MAC_ID in private data, we use IRQ number to check MAC1/MAC2 in current version
    regVal = inl(IO_ADDRESS(AST_SCU_BASE + SCU_SCRATCH_REG)) & 0xC000;
    if (regVal == 0x8000) {
      lp->NCSI_support = 1;
      printk ("MAC1 Support NCSI\n");
    }
    else if (regVal == 0x4000) {
      lp->INTEL_NCSI_EVA_support = 1;
      printk ("MAC1 Support INTEL_NCSI_EVA\n");
    }
    else {
      lp->INTEL_NCSI_EVA_support = 0;
      lp->NCSI_support = 0;
    }
  }
  else if (dev->irq == 3) { //MAC2
    regVal = inl(IO_ADDRESS(AST_SCU_BASE + SCU_SCRATCH_REG)) & 0x3000;
    if (regVal == 0x2000) {
      lp->NCSI_support = 1;
      printk ("MAC2 Support NCSI\n");
    }
    else if (regVal == 0x1000) {
      lp->INTEL_NCSI_EVA_support = 1;
      printk ("MAC2 Support INTEL_NCSI_EVA\n");
    }
    else {
      lp->INTEL_NCSI_EVA_support = 0;
      lp->NCSI_support = 0;
    }
  }

  // We assume the Clock Stop register does not disable the MAC1 or MAC2 clock
  // unless Reset Control also holds the MAC in reset.
#if defined(CONFIG_ARCH_AST2300_FPGA_2) || defined(CONFIG_ARCH_AST2300) || defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST3100)
  resetControl    = inl(IO_ADDRESS(AST_SCU_BASE + SCU_RESET_CONTROL_REG));

  switch (out->macId) {
  case 0:
    if (resetControl & SCU_RESET_MAC1)
      goto no_phy_access;
    out->miiPhyId = 1;    // MAC registers are accessible
    multiFuncPin = inl(IO_ADDRESS(AST_SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL3_REG));
    if (lp->NCSI_support == 0) {
      if (0 == (multiFuncPin & (SCU_MFP_MAC1_MDIO | SCU_MFP_MAC1_MDC))
          && out->phyAddr <= 0x1f)
        goto no_phy_access;
      out->miiPhyId = 2;
    }
    else {
      out->miiPhyId = 2;
    }
    break;

  case 1:
    if (resetControl & SCU_RESET_MAC2)
      goto no_phy_access;
    out->miiPhyId = 1;    // The MAC itself is usable
    multiFuncPin = inl(IO_ADDRESS(AST_SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL5_REG));
    // If there are no MII2DC+MII2DIO pins for a MAC2 PHY bus...
    if (lp->NCSI_support == 0) {
      if (0==(multiFuncPin & SCU_MFP_MAC2_MDC_MDIO)
          && out->phyAddr <= 0x1f)
        goto no_phy_access;
      out->miiPhyId = 2;
    }
    else {
      out->miiPhyId = 2;
    }
    break;

  default:
    goto no_phy_access;     // AST2100 has only two MACs
  }
#else
  resetControl    = inl(IO_ADDRESS(AST_SCU_BASE + SCU_RESET_CONTROL_REG));
  hwStrap_macMode = inl(IO_ADDRESS(AST_SCU_BASE + SCU_HARDWARE_TRAPPING_REG));
  hwStrap_macMode =
    (hwStrap_macMode & SCU_HT_MAC_INTERFACE) >> SCU_HT_MAC_INTF_LSBIT;
  multiFuncPin   = inl(IO_ADDRESS(AST_SCU_BASE + SCU_MULTIFUNCTION_PIN_REG));

  switch (out->macId) {
  case 0:
    if (resetControl & SCU_RESET_MAC1)
      goto no_phy_access;
    out->miiPhyId = 1;    // MAC registers are accessible
    if (hwStrap_macMode == 0x7)
      goto no_phy_access;   // No MAC interfaces enabled
    out->miiPhyId = 2;    // The MAC itself is usable
    break;

  case 1:
    if (resetControl & SCU_RESET_MAC2)
      goto no_phy_access;
    out->miiPhyId = 1;    // MAC registers are accessible
    if (hwStrap_macMode == 0x7)
      goto no_phy_access;   // No MAC interfaces enabled
    if (MAC_INTF_SINGLE_PORT_MODES & (1u << hwStrap_macMode)) {
      goto no_phy_access;   // Second MAC interface not enabled
    }
    if (hwStrap_macMode == SCU_HT_MAC_MII_MII) {
      if (0==(multiFuncPin & SCU_MFP_MAC2_MII_INTF))
        goto no_phy_access; // MII pins not enabled for MAC2 in MII mode
    }
    out->miiPhyId = 2;    // The MAC itself is usable
    // If there are no MII2DC+MII2DIO pins for a MAC2 PHY bus...
    if (0==(multiFuncPin & SCU_MFP_MAC2_MDC_MDIO)
        && out->phyAddr <= 0x1f)  // ...and the PHY chip is on the MAC2 bus
      goto no_phy_access;   // then here is no MAC2 PHY chip.
    break;

  default:
    goto no_phy_access;     // AST2100 has only two MACs
  }
#endif
  // For now, we only support a PHY chip on the MAC's own MDC+MDIO bus.
  if (out->phyAddr > 0x1f) {
    no_phy_access:
    out->phyAddr = 0xff;
    return;
  }

  if (lp->NCSI_support == 0) {
    out->miiPhyId = ftgmac100_read_phy_register(ioaddr, out->phyAddr, 0x02);
    if (out->miiPhyId == 0xFFFF) { //Realtek PHY at address 1
      out->phyAddr = 1;
    }
    if (out->miiPhyId == 0x0362) {
      out->phyAddr = 1;
    }
    out->miiPhyId = ftgmac100_read_phy_register(ioaddr, out->phyAddr, 0x02);
    out->miiPhyId = (out->miiPhyId & 0xffff) << 16;
    out->miiPhyId |= ftgmac100_read_phy_register(ioaddr, out->phyAddr, 0x03) & 0xffff;

    switch (out->miiPhyId >> 16) {
    case 0x0040:  // Broadcom
    case 0x0141:  // Marvell
    case 0x001c:  // Realtek
    case 0x0362:    // BCM54612
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

/*
  . Function: ftgmac100_reset( struct device* dev )
  . Purpose:
  .    This sets the SMC91111 chip to its normal state, hopefully from whatever
  .  mess that any other DOS driver has put it in.
  .
  . Maybe I should reset more registers to defaults in here?  SOFTRST  should
  . do that for me.
  .
  . Method:
  .  1.  send a SOFT RESET
  .  2.  wait for it to finish
  .  3.  enable autorelease mode
  .  4.  reset the memory management unit
  .  5.  clear all interrupts
  .
*/
static void ftgmac100_reset( struct net_device* dev )
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  struct AstMacHwConfig* ids = &lp->ids;
  unsigned long ioaddr = dev->base_addr;
  unsigned int tmp, speed, duplex;

  getMacHwConfig(dev, ids);
  PRINTK("%s:ftgmac100_reset, phyAddr=0x%x, miiPhyId=0x%04x_%04x\n",
         dev->name, ids->phyAddr, (ids->miiPhyId >> 16), (ids->miiPhyId & 0xffff));
  if (ids->miiPhyId < 1)
    return; // Cannot access MAC registers

  // Check the link speed and duplex.
  // They are not valid until auto-neg is resolved, which is reg.1 bit[5],
  // or the link is up, which is reg.1 bit[2].

  if (ids->phyAddr < 0xff)
    tmp = ftgmac100_read_phy_register(ioaddr, ids->phyAddr, 0x1);
  else tmp = 0;

  if (0==(tmp & (1u<<5 | 1u<<2)) || ids->phyAddr >= 0xff) {
    // No PHY chip, or link has not negotiated.
    speed  = PHY_SPEED_100M;
    duplex = 1;
    netif_carrier_off(dev);
  }
  else if (((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL)) {
    tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x00);
    duplex = (tmp & 0x0100) ? 1 : 0;
    speed  = (tmp & 0x2000) ? PHY_SPEED_100M : PHY_SPEED_10M;
  }
  else if (((ids->miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
           ((ids->miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
    // Use reg.17_0.bit[15:13] for {speed[1:0], duplex}.
    tmp    = ftgmac100_read_phy_register(ioaddr, ids->phyAddr, 0x11);
    duplex = (tmp & PHY_DUPLEX_mask)>>13;
    speed  = (tmp & PHY_SPEED_mask)>>14;
    netif_carrier_on(dev);
  }
  else if (lp->ids.miiPhyId == PHYID_BCM54612E) {
    // Get link status
    // First Switch shadow register selector
    ftgmac100_write_phy_register(ioaddr, lp->ids.phyAddr, 0x1C, 0x2000);
    tmp    = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x1C);
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
    tmp    = ftgmac100_read_phy_register(ioaddr, ids->phyAddr, 0x18);
    duplex = (tmp & 0x0001);
    speed  = (tmp & 0x0002) ? PHY_SPEED_100M : PHY_SPEED_10M;
  }

  if (speed == PHY_SPEED_1G) {
    // Set SPEED_100_bit too, for consistency.
    lp->maccr_val |= GMAC_MODE_bit | SPEED_100_bit;
    tmp = inl( ioaddr + MACCR_REG );
    tmp |= GMAC_MODE_bit | SPEED_100_bit;
    outl(tmp, ioaddr + MACCR_REG );
  } else {
    lp->maccr_val &= ~(GMAC_MODE_bit | SPEED_100_bit);
    tmp = inl( ioaddr + MACCR_REG );
    tmp &= ~(GMAC_MODE_bit | SPEED_100_bit);
    if (speed == PHY_SPEED_100M) {
      lp->maccr_val |= SPEED_100_bit;
      tmp |= SPEED_100_bit;
    }
    outl(tmp, ioaddr + MACCR_REG );
  }
  if (duplex)
    lp->maccr_val |= FULLDUP_bit;
  else lp->maccr_val &= ~FULLDUP_bit;
  outl( SW_RST_bit, ioaddr + MACCR_REG );

#ifdef not_complete_yet
  /* Setup for fast accesses if requested */
  /* If the card/system can't handle it then there will */
  /* be no recovery except for a hard reset or power cycle */
  if (dev->dma)
  {
    outw( inw( ioaddr + CONFIG_REG ) | CONFIG_NO_WAIT,
          ioaddr + CONFIG_REG );
  }
#endif /* end_of_not */

  /* this should pause enough for the chip to be happy */
  for (; (inl( ioaddr + MACCR_REG ) & SW_RST_bit) != 0; )
  {
    mdelay(10);
    PRINTK3("RESET: reset not complete yet\n" );
  }

  outl( 0, ioaddr + IER_REG );      /* Disable all interrupts */
}

/*
  . Function: ftgmac100_enable
  . Purpose: let the chip talk to the outside work
  . Method:
  .  1.  Enable the transmitter
  .  2.  Enable the receiver
  .  3.  Enable interrupts
*/
static void ftgmac100_enable( struct net_device *dev )
{
  unsigned long ioaddr  = dev->base_addr;
  int i;
  struct ftgmac100_local *lp  = (struct ftgmac100_local *)dev->priv;
  unsigned int tmp_rsize;   //Richard
  unsigned int rfifo_rsize; //Richard
  unsigned int tfifo_rsize; //Richard
  unsigned int rxbuf_size;
  unsigned long Package_Found = 0, Channel_Found = 0, Re_Send = 0, Link_Status;

  PRINTK2("%s:ftgmac100_enable\n", dev->name);

#ifdef CONFIG_FTGMAC_DES_FMT_NC1
  rxbuf_size = RX_BUF_SIZE & 0x3fff;
  outl( rxbuf_size , ioaddr + RBSR_REG); //for NC Body
#endif

  for (i=0; i<RXDES_NUM; ++i)
  {
    lp->rx_descs[i].RXPKT_RDY = RX_OWNBY_FTGMAC100;       // owned by FTMAC100
  }
  lp->rx_idx = 0;

  for (i=0; i<TXDES_NUM; ++i)
  {
    lp->tx_descs[i].TXDMA_OWN = TX_OWNBY_SOFTWARE;      // owned by software
    lp->tx_skbuff[i] = 0;
  }
  lp->tx_idx = 0;lp->old_tx = 0;lp->tx_free=TXDES_NUM;

  /* set the MAC address */
  put_mac(ioaddr + MAC_MADR_REG, dev->dev_addr);

  outl( lp->rx_descs_dma, ioaddr + RXR_BADR_REG);
  outl( lp->tx_descs_dma, ioaddr + TXR_BADR_REG);
  outl( 0x00001010, ioaddr + ITC_REG);

  outl( (0UL<<TXPOLL_CNT)|(0x1<<RXPOLL_CNT), ioaddr + APTC_REG);
  outl( 0x44f97, ioaddr + DBLAC_REG );

  /// outl( inl(FCR_REG)|0x1, ioaddr + FCR_REG );       // enable flow control
  /// outl( inl(BPR_REG)|0x1, ioaddr + BPR_REG );       // enable back pressure register

  // +++++ Richard +++++ //
  tmp_rsize = inl( ioaddr + FEAR_REG );
  rfifo_rsize = tmp_rsize & 0x00000007;
  tfifo_rsize = (tmp_rsize >> 3)& 0x00000007;

  tmp_rsize = inl( ioaddr + TPAFCR_REG );
  tmp_rsize &= ~0x3f000000;
  tmp_rsize |= (tfifo_rsize << 27);
  tmp_rsize |= (rfifo_rsize << 24);

  outl(tmp_rsize, ioaddr + TPAFCR_REG);
  // ----- Richard ----- //

//river set MAHT0, MAHT1
  if (lp->maccr_val & GMAC_MODE_bit) {
    outl (lp->GigaBit_MAHT0, ioaddr + MAHT0_REG);
    outl (lp->GigaBit_MAHT1, ioaddr + MAHT1_REG);
  }
  else {
    outl (lp->Not_GigaBit_MAHT0, ioaddr + MAHT0_REG);
    outl (lp->Not_GigaBit_MAHT1, ioaddr + MAHT1_REG);
  }

  /// enable trans/recv,...
  outl(lp->maccr_val, ioaddr + MACCR_REG );

//NCSI Start
//DeSelect Package/ Select Package
  if ((lp->NCSI_support == 1) || (lp->INTEL_NCSI_EVA_support == 1)) {
    NCSI_Struct_Initialize(dev);
    for (i = 0; i < 4; i++) {
      DeSelect_Package (dev, i);
      Package_Found = Select_Package (dev, i);
      if (Package_Found == 1) {
//AST2100/AST2050/AST1100 supports 1 slave only
        lp->NCSI_Cap.Package_ID = i;
        break;
      }
    }
    if (Package_Found != 0) {
//Initiali State
      for (i = 0; i < 2; i++) { //Suppose 2 channels in current version, You could modify it to 0x1F to support 31 channels
        Channel_Found = Clear_Initial_State(dev, i);
        if (Channel_Found == 1) {
          lp->NCSI_Cap.Channel_ID = i;
          printk ("Found NCSI Network Controller at (%d, %d)\n", lp->NCSI_Cap.Package_ID, lp->NCSI_Cap.Channel_ID);
//Get Version and Capabilities
          Get_Version_ID(dev);
          Get_Capabilities(dev);
//Configuration
          Select_Active_Package(dev);
//Set MAC Address
          Enable_Set_MAC_Address(dev);
//Enable Broadcast Filter
          Enable_Broadcast_Filter(dev);
//Disable VLAN
          Disable_VLAN(dev);
//Enable AEN
          Enable_AEN(dev);
//Get Parameters
          Get_Parameters(dev);
//Enable TX
          Enable_Network_TX(dev);
//Enable Channel
          Enable_Channel(dev);
//Get Link Status
          Re_Get_Link_Status:
          Link_Status = Get_Link_Status(dev);
          if (Link_Status == LINK_UP) {
            printk ("Using NCSI Network Controller (%d, %d)\n", lp->NCSI_Cap.Package_ID, lp->NCSI_Cap.Channel_ID);
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
//          Disable_Channel(dev);
          Re_Send = 0;
          Channel_Found = 0;
        }
      }
    }
  }
  /* now, enable interrupts */

  if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
      ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
    outl(
        PHYSTS_CHG_bit    |
        AHB_ERR_bit     |
        TPKT_LOST_bit   |
        TPKT2E_bit      |
        RXBUF_UNAVA_bit   |
        RPKT2B_bit
        ,ioaddr + IER_REG
    );
  }
  else if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) ||
           ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL)) {
    outl(
        AHB_ERR_bit     |
        TPKT_LOST_bit   |
        TPKT2E_bit      |
        RXBUF_UNAVA_bit   |
        RPKT2B_bit
        ,ioaddr + IER_REG
    );
  }
  else if (lp->ids.miiPhyId == PHYID_BCM54612E) {
    outl(
// no link PHY link status pin            PHYSTS_CHG_bit      |
        AHB_ERR_bit         |
        TPKT_LOST_bit       |
        TPKT2E_bit          |
        RXBUF_UNAVA_bit     |
        RPKT2B_bit
        ,ioaddr + IER_REG
    );
  }
}

/*
  . Function: ftgmac100_shutdown
  . Purpose:  closes down the SMC91xxx chip.
  . Method:
  .  1. zero the interrupt mask
  .  2. clear the enable receive flag
  .  3. clear the enable xmit flags
  .
  . TODO:
  .   (1) maybe utilize power down mode.
  .  Why not yet?  Because while the chip will go into power down mode,
  .  the manual says that it will wake up in response to any I/O requests
  .  in the register space.   Empirical results do not show this working.
*/
static void ftgmac100_shutdown( unsigned int ioaddr )
{
  ///interrupt mask register
  outl( 0, ioaddr + IER_REG );

  /// enable trans/recv,...
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
  . o  if the saved_skb is not currently null, then drop this packet
  .    on the floor.  This should never happen, because of TBUSY.
  . o  if the saved_skb is null, then replace it with the current packet,
  . o  See if I can sending it now.
  . o  (NO): Enable interrupts and let the interrupt handler deal with it.
  . o  (YES):Send it now.
*/
static int ftgmac100_wait_to_send_packet( struct sk_buff * skb, struct net_device * dev )
{
  struct ftgmac100_local *lp  = (struct ftgmac100_local *)dev->priv;
  unsigned long ioaddr  = dev->base_addr;
  volatile TX_DESC *cur_desc;
  int   length;
  unsigned long flags;

  spin_lock_irqsave(&lp->lock,flags);

  if (skb==NULL)
  {
    DO_PRINT("%s(%d): NULL skb???\n", __FILE__,__LINE__);
    spin_unlock_irqrestore(&lp->lock, flags);
    return 0;
  }

  PRINTK3("%s:ftgmac100_wait_to_send_packet, skb=%x\n", dev->name, skb);
  cur_desc = &lp->tx_descs[lp->tx_idx];

#ifdef not_complete_yet
  if (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)   /// no empty transmit descriptor
  {
    DO_PRINT("no empty transmit descriptor\n");
    DO_PRINT("jiffies = %d\n", jiffies);
    lp->stats.tx_dropped++;
    netif_stop_queue(dev);    /// waiting to do:
    spin_unlock_irqrestore(&lp->lock, flags);

    return 1;
  }
#endif /* end_of_not */

  if (cur_desc->TXDMA_OWN != TX_OWNBY_SOFTWARE)   /// no empty transmit descriptor
  {
    DO_PRINT("no empty TX descriptor:0x%x:0x%x\n",
             (unsigned int)cur_desc,((unsigned int *)cur_desc)[0]);
    lp->stats.tx_dropped++;
    netif_stop_queue(dev);    /// waiting to do:
    spin_unlock_irqrestore(&lp->lock, flags);

    return 1;
  }
  lp->tx_skbuff[lp->tx_idx] = skb;
  length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;
  length = min(length, TX_BUF_SIZE);

#if FTMAC100_DEBUG > 2
  DO_PRINT("Transmitting Packet at 0x%x, skb->data = %x, len = %x\n",
           (unsigned int)cur_desc->VIR_TXBUF_BADR, skb->data, length);
  print_packet( skb->data, length );
#endif

#ifndef ZeroCopy
  memcpy((char *)cur_desc->VIR_TXBUF_BADR, skb->data, length);
#else
  cur_desc->VIR_TXBUF_BADR = (unsigned long)skb->data;
  cur_desc->TXBUF_BADR = virt_to_phys(skb->data);
#ifndef CONFIG_CPU_FA52x_DCE
  dmac_clean_range((void *)skb->data, (void *)(skb->data + length));
#endif
#endif

  //clean_dcache_range(skb->data, (char*)(skb->data + length));

  cur_desc->TXBUF_Size = length;
  cur_desc->LTS = 1;
  cur_desc->FTS = 1;

  cur_desc->TX2FIC = 0;
  cur_desc->TXIC = 0;

  cur_desc->TXDMA_OWN = TX_OWNBY_FTGMAC100;

  outl( 0xffffffff, ioaddr + TXPD_REG);

  lp->tx_idx = (lp->tx_idx + 1) % TXDES_NUM;
  lp->stats.tx_packets++;
  lp->tx_free--;

  if (lp->tx_free <= 0) {
    netif_stop_queue(dev);

  }


  dev->trans_start = jiffies;

  spin_unlock_irqrestore(&lp->lock, flags);

  return 0;
}


/*-------------------------------------------------------------------------
  |
  | ftgmac100_destructor( struct device * dev )
  |   Input parameters:
  |  dev, pointer to the device structure
  |
  |   Output:
  |  None.
  |
  ------------------------------------------------------------------ftg---------
*/
void ftgmac100_destructor(struct net_device *dev)
{
  PRINTK3("%s:ftgmac100_destructor\n", dev->name);
}

#define dma_allocate(x,y,z,w) dma_alloc_writecombine((x),(y),(dma_addr_t*)(z),(w))
void ftgmac100_ringbuf_alloc(struct net_device *dev)
{
  int i;
  struct ftgmac100_local *lp;

  lp = (struct ftgmac100_local *)dev->priv;


#ifdef DescpCacheable
  lp->rx_descs = kmalloc( sizeof(RX_DESC)*RXDES_NUM+16, GFP_DMA|GFP_KERNEL );
  lp->rx_descs = (unsigned int) lp->rx_descs & 0xfffffff0;
  lp->rx_descs_dma = virt_to_phys(lp->rx_descs);
#else
  lp->rx_descs = dma_allocate( NULL, sizeof(RX_DESC)*RXDES_NUM,  &(lp->rx_descs_dma),GFP_DMA|GFP_KERNEL );
#endif

  if (lp->rx_descs == NULL || (( (u32)lp->rx_descs & 0xf)!=0))
  {
    DO_PRINT("Receive Ring Buffer(desc)  allocation error, lp->rx_desc = %p\n",
             lp->rx_descs);
    BUG();
  }

  memset((void*)lp->rx_descs, 0, sizeof(RX_DESC)*RXDES_NUM);

#ifndef ZeroCopy
#ifdef CONFIG_CPU_FA52x_DCE
  lp->rx_buf = kmalloc( RX_BUF_SIZE*RXDES_NUM+16, GFP_DMA|GFP_KERNEL );
  lp->rx_buf = (unsigned int) lp->rx_buf & 0xfffffff0;
  lp->rx_buf_dma = virt_to_phys(lp->rx_buf);
#else
  lp->rx_buf = dma_allocate( NULL,  RX_BUF_SIZE*RXDES_NUM, &(lp->rx_buf_dma),GFP_DMA|GFP_KERNEL );
#endif
#endif


#ifndef CONFIG_FTGMAC_DES_FMT_NC1
  if (lp->rx_buf == NULL || (( (u32)lp->rx_buf & 7)!=0) || (((u32)lp->rx_buf_dma & 7)!=0)) //Richard
  {
    DO_PRINT("Receive Ring Buffer (buf) allocation error, lp->rx_buf = %x\n", lp->rx_buf);
    BUG();
  }
#endif

  for (i=0; i<RXDES_NUM; ++i)
  {
#ifndef CONFIG_FTGMAC_DES_FMT_NC1
    lp->rx_descs[i].RXBUF_Size = RX_BUF_SIZE;
#endif
    lp->rx_descs[i].EDORR = 0;    // not last descriptor

#ifndef ZeroCopy
    lp->rx_descs[i].RXBUF_BADR = lp->rx_buf_dma+RX_BUF_SIZE*i;
    lp->rx_descs[i].VIR_RXBUF_BADR = lp->rx_buf+RX_BUF_SIZE*i;
#else
    {
      /* Allocated fixed size of skbuff */
      // Books say dev_alloc_skb() adds 16 bytes to the given
      // length, for an Ethernet MAC header.  RX_BUF_SIZE==1536
      // already covers the header, and is 48*32 so 48 cache lines.
      // alloc_skb() adds sizeof skb_shinfo and rounds up to cache line.
      struct sk_buff *skb = dev_alloc_skb(RX_BUF_SIZE + 16);
      lp->rx_skbuff[i] = skb;
      if (skb == NULL) {
        printk (KERN_ERR
                "%s: alloc_list: allocate Rx buffer error! ",
                dev->name);
        break;
      }
      skb->dev = dev; /* Mark as being used by this device. */
      // skb_reserve(skb, 2); /* 16 byte align the IP header. */
      // ASPEED: Align IP header to 32-byte cache line.
      skb_reserve (skb, 2);
      dmac_inv_range ((void *)skb->data, (void *)skb->data + RX_BUF_SIZE);
      /* Rubicon now supports 40 bits of addressing space. */
      lp->rx_descs[i].RXBUF_BADR = virt_to_phys(skb->tail);
      lp->rx_descs[i].VIR_RXBUF_BADR = (unsigned long)skb->tail;
    }
#endif

  }
  lp->rx_descs[RXDES_NUM-1].EDORR = 1;          // is last descriptor


#ifdef DescpCacheable
  lp->tx_descs = kmalloc( sizeof(TX_DESC)*TXDES_NUM+16, GFP_DMA|GFP_KERNEL );
  lp->tx_descs = (unsigned int) lp->tx_descs & 0xfffffff0;
  lp->tx_descs_dma = virt_to_phys(lp->tx_descs);
#else
  lp->tx_descs = dma_allocate(NULL, sizeof(TX_DESC)*TXDES_NUM, &(lp->tx_descs_dma) ,GFP_DMA|GFP_KERNEL);
#endif

  if (lp->tx_descs == NULL || (( (u32)lp->tx_descs & 0xf)!=0))
  {
    DO_PRINT("Transmit Ring Buffer (desc) allocation error, lp->tx_desc = %p\n",
             lp->tx_descs);
    BUG();
  }

  memset((void*)lp->tx_descs, 0, sizeof(TX_DESC)*TXDES_NUM);

#ifndef ZeroCopy
#ifdef CONFIG_CPU_FA52x_DCE
  lp->tx_buf = kmalloc( TX_BUF_SIZE*TXDES_NUM+16, GFP_DMA|GFP_KERNEL );
  lp->tx_buf = (unsigned int) lp->tx_buf & 0xfffffff0;
  lp->tx_buf_dma = virt_to_phys(lp->tx_buf);
#else
  lp->tx_buf = dma_allocate( NULL,TX_BUF_SIZE*TXDES_NUM, &(lp->tx_buf_dma),GFP_DMA|GFP_KERNEL );
#endif
#endif

#ifndef ZeroCopy
  if (lp->tx_buf == NULL || (( (u32)lp->tx_buf % 4)!=0))
  {
    DO_PRINT("Transmit Ring Buffer (buf) allocation error, lp->tx_buf = %x\n", lp->tx_buf);
    BUG();
  }
#endif

  for (i=0; i<TXDES_NUM; ++i)
  {
    lp->tx_descs[i].EDOTR = 0;      // not last descriptor
#ifndef ZeroCopy
    lp->tx_descs[i].TXBUF_BADR = lp->tx_buf_dma+TX_BUF_SIZE*i;
    lp->tx_descs[i].VIR_TXBUF_BADR = lp->tx_buf+TX_BUF_SIZE*i;
#endif
  }
  lp->tx_descs[TXDES_NUM-1].EDOTR = 1;      // is last descriptor
  PRINTK("lp->rx_descs = %p, lp->rx_rx_descs_dma = %x\n",
         lp->rx_descs, lp->rx_descs_dma);
  PRINTK("lp->rx_buf = %p, lp->rx_buf_dma = %x\n",
         lp->rx_buf, lp->rx_buf_dma);
  PRINTK("lp->tx_descs = %p, lp->tx_rx_descs_dma = %x\n",
         lp->tx_descs, lp->tx_descs_dma);
  PRINTK("lp->tx_buf = %p, lp->tx_buf_dma = %x\n",
         lp->tx_buf, lp->tx_buf_dma);

}


static struct proc_dir_entry *proc_ftgmac100;

static int ftgmac100_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data)
{
  struct net_device *dev = (struct net_device *)data;
  struct ftgmac100_local *lp  = (struct ftgmac100_local *)dev->priv;
  int num;
  int i;

  num = sprintf(page, "lp->rx_idx = %d\n", lp->rx_idx);
  for (i=0; i<RXDES_NUM; ++i)
  {
    num += sprintf(page + num, "[%d].RXDMA_OWN = %d\n", i, lp->rx_descs[i].RXPKT_RDY);
  }
  return num;
}


/*----------------------------------------------------------------------
  . Function: ftgmac100_probe( struct net_device *dev )
  .
  . Purpose:
  .  Tests to see if a given ioaddr points to an ftgmac100 chip.
  .  Returns a 0 on success
  .
  .
  .---------------------------------------------------------------------
*/
/*---------------------------------------------------------------
  . Here I do typical initialization tasks.
  .
  . o  Initialize the structure if needed
  . o  print out my vanity message if not done so already
  . o  print out what type of hardware is detected
  . o  print out the ethernet address
  . o  find the IRQ
  . o  set up my private data
  . o  configure the dev structure with my subroutines
  . o  actually GRAB the irq.
  . o  GRAB the region
  .-----------------------------------------------------------------*/

static int __init ftgmac100_probe(struct net_device *dev )
{
  int retval;
  static unsigned version_printed = 0;
  struct ftgmac100_local *lp = 0;

  PRINTK2("%s:ftgmac100_probe\n", dev->name);

  /// waiting to do: probe
  if (version_printed++ == 0)
  {
    DO_PRINT("%s", version);
  }

  /* now, print out the card info, in a short format.. */
  DO_PRINT("%s: at %#3lx IRQ:%d noWait:%d, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           dev->name, dev->base_addr, dev->irq, dev->dma,
           dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
           dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

  dev->priv = kmalloc(sizeof(struct ftgmac100_local), GFP_KERNEL);
  if (dev->priv == NULL)
  {
    retval = -ENOMEM;
    return retval;
  }

  /* Grab the IRQ next.  Beyond this, we will free the IRQ. */
  retval = request_irq(dev->irq, (void *)&ftgmac100_interrupt,
                       IRQF_DISABLED, dev->name, dev);
  if (retval)
  {
    DO_PRINT("%s: unable to get IRQ %d (retval=%d).\n",
             dev->name, dev->irq, retval);
    kfree(dev->priv);
    dev->priv = NULL;
    return retval;
  }
  // IRQ properties are handled by kernel driver in ASPEED SDK.
  // IRQ_SET_HIGH_LEVEL (dev->irq);
  // IRQ_SET_LEVEL_TRIGGER (dev->irq);

  /// --------------------------------------------------------------------
  ///   initialize ftgmac100_local
  /// --------------------------------------------------------------------
  memset(dev->priv, 0, sizeof(struct ftgmac100_local));
  lp = (struct ftgmac100_local *)dev->priv;
  spin_lock_init(&lp->lock);

  lp->maccr_val = (CRC_APD_bit | RXMAC_EN_bit | TXMAC_EN_bit  | RXDMA_EN_bit
                   | TXDMA_EN_bit | CRC_CHK_bit | RX_BROADPKT_bit | SPEED_100_bit | FULLDUP_bit);

  /* now, reset the chip, and put it into a known state */
  ftgmac100_reset( dev );

  // Quit with an error if this MAC cannot carry traffic.
  if (lp->ids.miiPhyId < 2) {
    retval = -EBADSLT;  // Or -ENODEV...?
    goto err_out;
  }

  ftgmac100_ringbuf_alloc(dev);//(lp); //disable for OTG //fred

  /* Fill in the fields of the device structure with ethernet values. */
  ether_setup(dev);

  dev->open           = ftgmac100_open;
  dev->stop           = ftgmac100_close;
  dev->hard_start_xmit    = ftgmac100_wait_to_send_packet;
  dev->tx_timeout     = ftgmac100_timeout;
  dev->get_stats      = ftgmac100_query_statistics;
#ifdef  HAVE_MULTICAST
  dev->set_multicast_list = &ftgmac100_set_multicast_list;
#endif

  //Fred add for hw checksum offload
  //dev->features|=NETIF_F_HW_CSUM;

  if ((proc_ftgmac100 = create_proc_entry( dev->name, 0, 0 )))
  {
    proc_ftgmac100->read_proc = ftgmac100_read_proc;
    proc_ftgmac100->data = dev;
    proc_ftgmac100->owner = THIS_MODULE;
  }
  return 0;

  err_out:
  if (lp != 0) {
    if (lp->rx_descs)
      dma_free_coherent( NULL, sizeof(RX_DESC)*RXDES_NUM,
                         (void*)lp->rx_descs, (dma_addr_t)lp->rx_descs_dma );
    if (lp->rx_buf)
      dma_free_coherent( NULL, RX_BUF_SIZE*RXDES_NUM,
                         (void*)lp->rx_buf, (dma_addr_t)lp->rx_buf_dma );
    if (lp->tx_descs)
      dma_free_coherent( NULL, sizeof(TX_DESC)*TXDES_NUM,
                         (void*)lp->tx_descs, (dma_addr_t)lp->tx_descs_dma );
    if (lp->tx_buf)
      dma_free_coherent( NULL, TX_BUF_SIZE*TXDES_NUM,
                         (void*)lp->tx_buf, (dma_addr_t)lp->tx_buf_dma );
    lp->rx_descs = NULL; lp->rx_descs_dma = 0;
    lp->rx_buf = NULL; lp->rx_buf_dma = 0;
    lp->tx_descs = NULL; lp->tx_descs_dma = 0;
    lp->tx_buf = NULL; lp->tx_buf_dma = 0;
  }
  free_irq( dev->irq, dev );
  kfree( dev->priv );
  dev->priv = NULL;
  return retval;
}


#if FTMAC100_DEBUG > 2
static void print_packet( byte * buf, int length )
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
      byte a, b;

      a = *(buf ++ );
      b = *(buf ++ );
      DO_PRINT("%02x%02x ", a, b );
    }
    DO_PRINT("\n");
  }
  for ( i = 0; i < remainder/2 ; i++ ) {
    byte a, b;

    a = *(buf ++ );
    b = *(buf ++ );
    DO_PRINT("%02x%02x ", a, b );
  }
  DO_PRINT("\n");
#endif
#endif
}
#endif


/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc ..
 *
 */
static int ftgmac100_open(struct net_device *dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;

  DO_PRINT("%s:ftgmac100_open\n", dev->name);

  netif_start_queue(dev);

  /* reset the hardware */
  ftgmac100_reset( dev );
  ftgmac100_enable( dev );

  if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) ||
      ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL) ||
      (lp->ids.miiPhyId == PHYID_BCM54612E)) {

    init_timer(&lp->timer);
    lp->timer.data = (unsigned long)dev;
    lp->timer.function = aspeed_mac_timer;
    lp->timer.expires = jiffies + 1 * HZ;
    add_timer (&lp->timer);
  }

  /* Configure the PHY */
  ftgmac100_phy_configure(dev);

  netif_start_queue(dev);

  return 0;
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


/*--------------------------------------------------------------------
  .
  . This is the main routine of the driver, to handle the net_device when
  . it needs some attention.
  .
  . So:
  .   first, save state of the chipset
  .   branch off into routines to handle each case, and acknowledge
  .      each to the interrupt register
  .   and finally restore state.
  .
  ---------------------------------------------------------------------*/
static irqreturn_t ftgmac100_interrupt(int irq, void * dev_id,  struct pt_regs * regs)
{
  struct net_device *dev  = dev_id;
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long ioaddr  = dev->base_addr;
  int   timeout;
  unsigned int tmp;
  unsigned int  mask;     // interrupt mask
  unsigned int  status;     // interrupt status

//  PRINTK3("%s: ftgmac100 interrupt started \n", dev->name);

  if (dev == NULL) {
    DO_PRINT(KERN_WARNING "%s: irq %d for unknown device.\n", dev->name, irq);
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
      if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
          ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211))
      {
        tmp = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x13);
        PRINTK("%s: PHY interrupt status, read_phy_reg(0x13) = 0x%04x\n",
               dev->name, tmp);
        tmp &= (PHY_SPEED_CHG_bit | PHY_DUPLEX_CHG_bit | PHY_LINK_CHG_bit);
      }
      else if ((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM)
      {
        tmp = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x1a);
        PRINTK("%s: PHY interrupt status, read_phy_reg(0x1a) = 0x%04x\n",
               dev->name, tmp);
        // Bits [3:1] are {duplex, speed, link} change interrupts.
        tmp &= 0x000e;
      }
      else if (lp->ids.miiPhyId == PHYID_BCM54612E) {
        tmp = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x1A);
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

//    PRINTK3(KERN_WARNING "%s: Handling interrupt status %x \n", dev->name, status);

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

//  PRINTK3("%s: Interrupt done\n", dev->name);
  return IRQ_HANDLED;
}

static void ftgmac100_free_tx (struct net_device *dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  int entry = lp->old_tx % TXDES_NUM;
  unsigned long flags = 0;

  spin_lock_irqsave(&lp->lock,flags);

  /* Free used tx skbuffs */

  while ((lp->tx_descs[entry].TXDMA_OWN == TX_OWNBY_SOFTWARE) && (lp->tx_skbuff[entry] != NULL)) {
    struct sk_buff *skb;

    skb = lp->tx_skbuff[entry];
    dev_kfree_skb_any (skb);
    lp->tx_skbuff[entry] = 0;
    entry = (entry + 1) % TXDES_NUM;
    lp->tx_free++;
  }

  spin_unlock_irqrestore(&lp->lock, flags);
  lp->old_tx = entry;
  if ((netif_queue_stopped(dev)) && (lp->tx_free > 0)) {
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long ioaddr  = dev->base_addr;
  int   packet_length;
  int   rcv_cnt;
  volatile RX_DESC *cur_desc;
  int   cur_idx;
  int   have_package;
  int   have_frs;
  int   start_idx;
  int             count = 0;
  int             packet_full = 0;
  int   data_not_fragment = 1;
#ifndef ZeroCopy
  int             cpy_length = 0;
  int             seg_length = 0;
#endif

//  DO_PRINT("%s:ftgmac100_rcv\n", dev->name);
  start_idx = lp->rx_idx;

  for (rcv_cnt=0; rcv_cnt<RXDES_NUM ; ++rcv_cnt)
  {
    packet_length = 0;
    cur_idx = lp->rx_idx;

    have_package = 0;
    have_frs = 0;

#ifdef CONFIG_CPU_FA52x_DCE
#ifdef DescpCacheable
    //the address range has to be cache line aligned
    dma_inv_range(&lp->rx_descs[lp->rx_idx], &lp->rx_descs[lp->rx_idx+1]);
#endif
#endif

    for (; (cur_desc = &lp->rx_descs[lp->rx_idx])->RXPKT_RDY==RX_OWNBY_SOFTWARE; )
    {
      have_package = 1;
      lp->rx_idx = (lp->rx_idx+1)%RXDES_NUM;
      count++;
      if (count == RXDES_NUM) {
        packet_full = 1;
      }
      if (DF_support) {
        if (data_not_fragment == 1) {
          if (!(cur_desc->DF)) {
            data_not_fragment = 0;
          }
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
          //  #ifdef not_complete_yet
          if (cur_desc->RX_ERR)
          {
            DO_PRINT("err: RX_ERR\n");
          }
          if (cur_desc->CRC_ERR)
          {
            //    DO_PRINT("err: CRC_ERR\n");
          }
          if (cur_desc->FTL)
          {
            DO_PRINT("err: FTL\n");
          }
          if (cur_desc->RX_ODD_NB)
          {
            //    DO_PRINT("err: RX_ODD_NB\n");
          }
//                if (cur_desc->IPCS_FAIL || cur_desc->UDPCS_FAIL || cur_desc->TCPCS_FAIL)
//                {
//                  DO_PRINT("err: CS FAIL\n");
//                }
          //  #endif /* end_of_not */
          lp->stats.rx_errors++;      // error frame....
          break;
        }
        if (DF_support) {
          if (cur_desc->DF) {
            if (cur_desc->IPCS_FAIL || cur_desc->UDPCS_FAIL || cur_desc->TCPCS_FAIL)
            {
              DO_PRINT("err: CS FAIL\n");
              lp->stats.rx_errors++;      // error frame....
              break;
            }
          }
        }
        if (cur_desc->MULTICAST)
        {
          lp->stats.multicast++;
        }
        if ((lp->NCSI_support == 1) || (lp->INTEL_NCSI_EVA_support == 1)) {
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

//      if ( cur_desc->LRS )    // packet's last frame
//      {
      break;
//      }
    }
    if (have_package==0)
    {
      goto done;
    }
    if (have_frs == 0)
    {
      //  DO_PRINT("error, loss first\n");
      lp->stats.rx_over_errors++;
    }

    if (packet_length>0)
    {
      struct sk_buff  * skb;
      byte    * data = 0;  if (data) { }

      packet_length -= 4;

#ifndef ZeroCopy
      // Allocate enough memory for entire receive frame, to be safe
      skb = dev_alloc_skb( packet_length+2 );

      //  DO_PRINT(KERN_NOTICE "dev_alloc_skb (0x%x) sucess in rcv\n", skb);

      if ( skb == NULL )
      {
        DO_PRINT(KERN_NOTICE "%s: Low memory, packet dropped.\n", dev->name);
        lp->stats.rx_dropped++;
        goto done;
      }

      /*
        ! This should work without alignment, but it could be
        ! in the worse case
      */
      /* TODO: Should I use 32bit alignment here ? */

      skb_reserve( skb, 2 );   /* 16 bit alignment */ //Richard

      skb->dev = dev;
      data = skb_put( skb, packet_length);
      cpy_length = 0;
      for (; cur_idx!=lp->rx_idx; cur_idx=(cur_idx+1)%RXDES_NUM)
      {
        seg_length = min(packet_length - cpy_length, RX_BUF_SIZE);
        memcpy(data+cpy_length, (char *)lp->rx_descs[cur_idx].VIR_RXBUF_BADR, seg_length);
        cpy_length += seg_length;
      }

#else
      skb_put (skb = lp->rx_skbuff[cur_idx], packet_length);

#ifndef CONFIG_CPU_FA52x_DCE
//      DO_PRINT("rx: invalidate dcache range (0x%x) (0x%x) len=%x \n", skb->data,(char*)(skb->data + packet_length), packet_length);
//      dmac_inv_range(skb->data, (char*)(skb->data + packet_length));
//      dmac_flush_range(skb->data, (char*)(skb->data + packet_length));
//        outer_flush_range(__pa(skb->data), __pa(skb->data) + packet_length);
#endif
// Rx Offload
      if (DF_support) {
        if (data_not_fragment) {
//        printk ("Offload\n");
          skb->ip_summed = CHECKSUM_UNNECESSARY;
          data_not_fragment = 1;
        }
      }
#endif

#if FTMAC100_DEBUG > 2
      DO_PRINT("Receiving Packet at 0x%x, packet len = %x\n",(unsigned int)data, packet_length);
      print_packet( data, packet_length );
#endif

      skb->protocol = eth_type_trans(skb, dev );
      netif_rx(skb);
      lp->stats.rx_packets++;
      lp->rx_skbuff[cur_idx] = NULL;
    }
    if (packet_full) {
//                  DO_PRINT ("RX Buffer full before driver entered ISR\n");
      goto done;
    }
  }

  done:

  if (packet_full) {
#ifdef ZeroCopy
    struct sk_buff *skb;

    for (cur_idx = 0; cur_idx < RXDES_NUM; cur_idx++)
    {
      if (lp->rx_skbuff[cur_idx] == NULL) {
        skb = dev_alloc_skb (RX_BUF_SIZE + 16);
        if (skb == NULL) {
          printk (KERN_INFO
                  "%s: receive_packet: "
                  "Unable to re-allocate Rx skbuff.#%d\n",
                  dev->name, cur_idx);
        }
        lp->rx_skbuff[cur_idx] = skb;
        skb->dev = dev;
        // ASPEED: See earlier skb_reserve() cache alignment
        skb_reserve (skb, 2);
        dmac_inv_range ((void *)skb->data,
                        (void *)skb->data + RX_BUF_SIZE);
        lp->rx_descs[cur_idx].RXBUF_BADR = virt_to_phys(skb->tail);
        lp->rx_descs[cur_idx].VIR_RXBUF_BADR = (unsigned long)skb->tail;
      }
      lp->rx_descs[cur_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
    }
    packet_full = 0;
#endif
  }
  else {
    if (start_idx != lp->rx_idx) {
      struct sk_buff *skb;

      for (cur_idx = (start_idx+1)%RXDES_NUM; cur_idx != lp->rx_idx; cur_idx = (cur_idx+1)%RXDES_NUM)
      {

#ifdef ZeroCopy
        //struct sk_buff *skb;
        /* Dropped packets don't need to re-allocate */
        if (lp->rx_skbuff[cur_idx] == NULL) {
          skb = dev_alloc_skb (RX_BUF_SIZE + 16);
          if (skb == NULL) {
            printk (KERN_INFO
                    "%s: receive_packet: "
                    "Unable to re-allocate Rx skbuff.#%d\n",
                    dev->name, cur_idx);
            break;
          }
          lp->rx_skbuff[cur_idx] = skb;
          skb->dev = dev;
          /* 16 byte align the IP header */
          skb_reserve (skb, 2);
          dmac_inv_range ((void *)skb->data,
                          (void *)skb->data + RX_BUF_SIZE);
          lp->rx_descs[cur_idx].RXBUF_BADR = virt_to_phys(skb->tail);
          lp->rx_descs[cur_idx].VIR_RXBUF_BADR = (unsigned long)skb->tail;
        }
#endif
        lp->rx_descs[cur_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
      }

#ifdef ZeroCopy
      //struct sk_buff *skb;
      /* Dropped packets don't need to re-allocate */
      if (lp->rx_skbuff[start_idx] == NULL) {
        skb = dev_alloc_skb (RX_BUF_SIZE + 16);
        if (skb == NULL) {
          printk (KERN_INFO
                  "%s: receive_packet: "
                  "Unable to re-allocate Rx skbuff.#%d\n",
                  dev->name, start_idx);
        }
        lp->rx_skbuff[start_idx] = skb;
        skb->dev = dev;
        /* 16 byte align the IP header */
        skb_reserve (skb, 2);
        dmac_inv_range ((void *)skb->data,
                        (void *)skb->data + RX_BUF_SIZE);
        lp->rx_descs[start_idx].RXBUF_BADR = virt_to_phys(skb->tail); //Richard
        lp->rx_descs[start_idx].VIR_RXBUF_BADR = (unsigned long)skb->tail;
      }
#endif

      lp->rx_descs[start_idx].RXPKT_RDY = RX_OWNBY_FTGMAC100;
    }
  }
  if (trans_busy == 1)
  {
    /// lp->maccr_val |= RXMAC_EN_bit;
    outl( lp->maccr_val, ioaddr + MACCR_REG );
    outl( inl(ioaddr + IER_REG) | RXBUF_UNAVA_bit, ioaddr + IER_REG);
  }
  return;
}


/*----------------------------------------------------
  . ftgmac100_close
  .
  . this makes the board clean up everything that it can
  . and not talk to the outside world.   Caused by
  . an 'ifconfig ethX down'
  .
  -----------------------------------------------------*/
static int ftgmac100_close(struct net_device *dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;

  netif_stop_queue(dev);
  //dev->start = 0;

  PRINTK2("%s:ftgmac100_close\n", dev->name);

  /* clear everything */
  ftgmac100_shutdown( dev->base_addr );

  if (lp->timer.function != NULL) {
    del_timer_sync(&lp->timer);
  }

  /* Update the statistics here. */

  return 0;
}

/*------------------------------------------------------------
  . Get the current statistics.
  . This may be called with the card open or closed.
  .-------------------------------------------------------------*/
static struct net_device_stats* ftgmac100_query_statistics(struct net_device *dev)
{
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;

  return &lp->stats;
}



#ifdef HAVE_MULTICAST

/*
  . Function: ftgmac100_setmulticast( struct net_device *dev, int count, struct dev_mc_list * addrs )
  . Purpose:
  .    This sets the internal hardware table to filter out unwanted multicast
  .    packets before they take up memory.
*/

static void ftgmac100_setmulticast( struct net_device *dev, int count, struct dev_mc_list * addrs )
{
  struct dev_mc_list  * cur_addr;
  int crc_val;
  unsigned int  ioaddr = dev->base_addr;
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  struct AstMacHwConfig* ids = &lp->ids;
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;

  cur_addr = addrs;
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
//TX
  if (lp->NCSI_support == 1) {
    lp->InstanceID++;
    lp->NCSI_Request.IID = lp->InstanceID;
    lp->NCSI_Request.Command = SET_MAC_ADDRESS;
    Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
    lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
    lp->NCSI_Request.Payload_Length = (8 << 8);
    memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
    lp->NCSI_Request.Payload_Length = 8;
    for (i = 0; i < 6; i++) {
      lp->Payload_Data[i] = cur_addr->dmi_addr[i];
    }
    lp->Payload_Data[6] = 2; //MAC Address Num = 1 --> address filter 1, fixed in sample code
    lp->Payload_Data[7] = MULTICAST_ADDRESS + 0 + ENABLE_MAC_ADDRESS_FILTER; //AT + Reserved + E
    copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
    skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
    ftgmac100_wait_to_send_packet (skb, dev);
  }

  for (cur_addr = addrs ; cur_addr!=NULL ; cur_addr = cur_addr->next )
  {
    /* make sure this is a multicast address - shouldn't this be a given if we have it here ? */
    if ( !( *cur_addr->dmi_addr & 1 ) )
    {
      continue;
    }
//GigaBit
    if (!(ids->isRevA2)) {//A0, A1
      crc_val = crc32( cur_addr->dmi_addr, 5 );
      crc_val = (~(crc_val>>2)) & 0x3f;
      if (crc_val >= 32)
      {
        outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
        lp->GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
      }
      else
      {
        outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
        lp->GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
      }
//10/100M
      crc_val = crc32( cur_addr->dmi_addr, 6 );
      crc_val = (~(crc_val>>2)) & 0x3f;
      if (crc_val >= 32)
      {
        outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
        lp->Not_GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
      }
      else
      {
        outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
        lp->Not_GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
      }
    }
    else {//A2
      crc_val = crc32( cur_addr->dmi_addr, 6 );
      crc_val = (~(crc_val>>2)) & 0x3f;
      if (crc_val >= 32)
      {
        outl(inl(ioaddr+MAHT1_REG) | (1UL<<(crc_val-32)), ioaddr+MAHT1_REG);
        lp->Not_GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
        lp->GigaBit_MAHT1 = inl (ioaddr + MAHT1_REG);
      }
      else
      {
        outl(inl(ioaddr+MAHT0_REG) | (1UL<<crc_val), ioaddr+MAHT0_REG);
        lp->Not_GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
        lp->GigaBit_MAHT0 = inl (ioaddr + MAHT0_REG);
      }
    }
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
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;

  PRINTK2("%s:ftgmac100_set_multicast_list\n", dev->name);

  if (dev->flags & IFF_PROMISC)
  {
    lp->maccr_val |= RX_ALLADR_bit;
  }
  else
  {
    lp->maccr_val &= ~RX_ALLADR_bit;
  }
  if ( (dev->flags & IFF_ALLMULTI))
  {
    lp->maccr_val |= RX_MULTIPKT_bit;
  }
  else
  {
    lp->maccr_val &= ~RX_MULTIPKT_bit;
  }
  if (dev->mc_count)
  {
//    PRINTK("set multicast\n");
    lp->maccr_val |= RX_HT_EN_bit;
    ftgmac100_setmulticast( dev, dev->mc_count, dev->mc_list );
  }
  else
  {
    lp->maccr_val &= ~RX_HT_EN_bit;
  }
  outl( lp->maccr_val, ioaddr + MACCR_REG );

}
#endif


/*
 * Module initialization function
 */

static char *strdup(const char *s)
{
  char *dups = kmalloc(strlen(s)+1, GFP_KERNEL);
  if (dups)
    strcpy(dups, s);
  return dups;
}

static inline void set_io_resource(int id, struct resource *mac_res)
{
  char buf[32];
  sprintf( buf, "FTGMAC1000 MAC controller %d", id+1 );
  mac_res->name  = strdup( buf );
  mac_res->start = IP_va_base[id];
  mac_res->end   = IP_va_limit[id];
}


static int devnum = IP_COUNT;
module_param(devnum, int, S_IRUGO);


int __init ftgmac100_init(void)
{
  int result, id, thisresult;
  struct net_device *dev;

  result = -ENODEV;

  for (id=0; id < devnum; id++) {

    dev = alloc_etherdev(sizeof(struct ftgmac100_local));
    if (!dev) {
      printk(KERN_ERR "Fail allocating ethernet device");
      return -ENODEV;
    }
    ftgmac100_netdev[id] = NULL;
//                SET_MODULE_OWNER (dev);
    /* Copy the parameters from the platform specification */
    dev->base_addr = IP_va_base[id];
    dev->irq = IP_irq[id];

    /* Setup initial mac address */
    auto_get_mac(id, dev->dev_addr);

    dev->init = ftgmac100_probe;
    if ((thisresult = register_netdev(dev)) != 0) {
      if (thisresult != 0) {
        // register_netdev() filled in dev->name.
        DO_PRINT("%s:register_netdev() returned %d\n",
                 dev->name, thisresult);
      }
      free_netdev(dev);
    } else {
      ftgmac100_netdev[id] = dev;
    }
    if (thisresult == 0) // any of the devices initialized, run
      result = 0;
  }

  return result;
}


/*------------------------------------------------------------
  . Cleanup when module is removed with rmmod
  .-------------------------------------------------------------*/
void ftgmac100_module_exit(void)
{
  int id;
  struct net_device *dev;
  struct ftgmac100_local *priv;
  PRINTK("+cleanup_module\n");

  for (id=0; id < devnum; id++) {
    dev = ftgmac100_netdev[id];

    if (dev==NULL)
      continue;

    priv = (struct ftgmac100_local *)netdev_priv(dev);
    if (priv->rx_descs)
      dma_free_coherent( NULL, sizeof(RX_DESC)*RXDES_NUM, (void*)priv->rx_descs, (dma_addr_t)priv->rx_descs_dma );
    if (priv->rx_buf)
      dma_free_coherent( NULL, RX_BUF_SIZE*RXDES_NUM, (void*)priv->rx_buf, (dma_addr_t)priv->rx_buf_dma );
    if (priv->tx_descs)
      dma_free_coherent( NULL, sizeof(TX_DESC)*TXDES_NUM, (void*)priv->tx_descs, (dma_addr_t)priv->tx_descs_dma );
    if (priv->tx_buf)
      dma_free_coherent( NULL, TX_BUF_SIZE*TXDES_NUM, (void*)priv->tx_buf, (dma_addr_t)priv->tx_buf_dma );
    priv->rx_descs = NULL; priv->rx_descs_dma = 0;
    priv->rx_buf   = NULL; priv->rx_buf_dma   = 0;
    priv->tx_descs = NULL; priv->tx_descs_dma = 0;
    priv->tx_buf   = NULL; priv->tx_buf_dma   = 0;

    /* No need to check MOD_IN_USE, as sys_delete_module() checks. */
    unregister_netdev(dev);

    free_irq(dev->irq, dev);
    free_netdev(dev);
    ftgmac100_netdev[id] = NULL;
    // free resource, free allocated memory
  }
}


module_init(ftgmac100_init);
module_exit(ftgmac100_module_exit);


//---PHY CONTROL AND CONFIGURATION-----------------------------------------
#ifdef not_complete_yet

void ftgmac100_phy_restart_auto(unsigned int ioaddr)
{
  unsigned int tmp;

  if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
      ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
    tmp = ftgmac100_read_phy_register(ioaddr, PHY_MARVELL_ADDR, 0x00 );
    tmp |= PHY_RE_AUTO_bit;
    ftgmac100_write_phy_register(ioaddr, PHY_MARVELL_ADDR, 0x00, tmp);

    // Waiting for complete Auto-N
    do {
      mdelay(2);
      tmp = ftgmac100_read_phy_register(ioaddr, PHY_MARVELL_ADDR, 0x01 );
    } while ((tmp&PHY_AUTO_OK_bit)==0);
  }
  else if ((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) {
  }
}
#endif // not_complete_yet


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
static word ftgmac100_read_phy_register(unsigned int ioaddr, byte phyaddr, byte phyreg)
{
  unsigned int tmp;

  if (phyaddr > 0x1f) // MII chip IDs are 5 bits long
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
                                         byte phyaddr, byte phyreg, word phydata)
{
  unsigned int tmp;

  if (phyaddr > 0x1f) // MII chip IDs are 5 bits long
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


/*------------------------------------------------------------
  . Configures the specified PHY using Autonegotiation.
  .-------------------------------------------------------------*/
static void ftgmac100_phy_configure(struct net_device* dev)
{
#ifdef CONFIG_FTGMAC_PHY_MARVELL
  struct ftgmac100_local *lp = (struct ftgmac100_local *)dev->priv;
  unsigned long ioaddr = dev->base_addr;

  if (((lp->ids.miiPhyId & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
      ((lp->ids.miiPhyId & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
    unsigned int tmp;
    ftgmac100_write_phy_register(ioaddr, lp->ids.phyAddr, 0x12, 0x4400);

    tmp = ftgmac100_read_phy_register(ioaddr, lp->ids.phyAddr, 0x13 );
  }
  else if (lp->ids.miiPhyId == PHYID_BCM54612E) {

    ftgmac100_write_phy_register(ioaddr, lp->ids.phyAddr, 0x1C, 0x8C00); // Disable GTXCLK Clock Delay Enable
    ftgmac100_write_phy_register(ioaddr, lp->ids.phyAddr, 0x18, 0xF0E7); // Disable RGMII RXD to RXC Skew

  }

#endif
}
