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

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/crc32.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <net/ip.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/stat.h>
#include <asm/uaccess.h>
#include "ftgmac100.h"
#include <linux/time.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
// #define TIME_POWERUP_PREP


#define DRV_NAME	"ftgmac100"
#define DRV_VERSION	"0.7"

#define RX_QUEUE_ENTRIES	256	/* must be power of 2 */
#define TX_QUEUE_ENTRIES	512	/* must be power of 2 */

#define MAX_PKT_SIZE		1518
#define RX_BUF_SIZE		PAGE_SIZE	/* must be smaller than 0x3fff */

#define MAX_NCSI_DATA_PAYLOAD 1480  /* for getting the size of the nc-si control data packet */
 /* max ethernet frame size = 1518 */
 /* ethernet headr (14) + nc-si header (16) + nc-si payload (1480) + nc-si checksum (4) + 4 (FCS) = 1518*/

#define noNCSI_DEBUG   /* for debug printf messages */

/*
 * Only for YAMP's BMC address, we need to add 2 to host's MAC Base Address.
 * On other platform we add 1.
 */
#if defined(CONFIG_YAMP)
  #define BMC_MAC_OFFSET 2
#else
  #define BMC_MAC_OFFSET 1
#endif

#define noDEBUG_AEN
// special  channel/cmd  used for register AEN handler with kernel
#define REG_AEN_CH  0x1a
#define REG_AEN_CMD 0xce

#ifdef DEBUG_AEN
  #define AEN_PRINT(fmt, args...) printk(fmt, ##args)
#else
  #define AEN_PRINT(fmt, args...)
#endif

//#define DEBUG_PRINT_NCSI_PACKET

/******************************************************************************
 * private data
 *****************************************************************************/
struct ftgmac100_descs {
	struct ftgmac100_rxdes rxdes[RX_QUEUE_ENTRIES];
	struct ftgmac100_txdes txdes[TX_QUEUE_ENTRIES];
};

#define MAX_AEN_BUFFER 8

struct ftgmac100 {
	struct resource *res;
	void __iomem *base;
	int irq;

	struct ftgmac100_descs *descs;
	dma_addr_t descs_dma_addr;
	struct page *rxdes_pages[RX_QUEUE_ENTRIES];

	unsigned int rx_pointer;
	unsigned int tx_clean_pointer;
	unsigned int tx_pointer;
	unsigned int tx_pending;

	spinlock_t tx_lock;

	struct net_device *netdev;
	struct device *dev;
	struct napi_struct napi;

	struct mii_bus *mii_bus;
	int phy_irq[PHY_MAX_ADDR];
	struct phy_device *phydev;
	int old_speed;
#ifdef CONFIG_FTGMAC100_NCSI
	NCSI_Command_Packet NCSI_Request;
	NCSI_Response_Packet NCSI_Respond;
	NCSI_Capability NCSI_Cap;
	unsigned char InstanceID;
	unsigned int  Retry;
	unsigned char Payload_Data[MAX_NCSI_DATA_PAYLOAD];
	unsigned char Payload_Pad[4];
	unsigned long Payload_Checksum;
	int mezz_type;
		#define MEZZ_UNKNOWN    -1
		#define MEZZ_MLX        0x01
		#define MEZZ_BCM        0x02
                #define MEZZ_INTEL      0x03
	unsigned int  powerup_prep_host_id;
	struct completion ncsi_complete;

	DECLARE_KFIFO(AEN_buffer, AEN_Packet, MAX_AEN_BUFFER);
	struct workqueue_struct *ncsi_wq;  // work queue to NC-SI packets
	struct work_struct work_aen;       // work type for AEN
	int aen_pid;  // pid of AEN handler
#endif
};

#ifdef CONFIG_FTGMAC100_NCSI

#define NETLINK_USER 31

typedef struct ncsi_nl_rsp_hdr_t {
  uint8_t cmd;
  uint16_t payload_length;
} __attribute__((packed)) NCSI_NL_RSP_HDR_T;

#define MAX_PAYLOAD  MAX_NCSI_DATA_PAYLOAD /* maximum payload size*/

typedef struct ncsi_nl_msg_t {
  char dev_name[10];
  unsigned char channel_id;
  unsigned char cmd;
  uint16_t payload_length;
  unsigned char msg_payload[MAX_PAYLOAD];
} NCSI_NL_MSG_T;


#define MAX_RESPONSE_PAYLOAD 1024 /* maximum payload size*/
typedef struct ncsi_nl_response {
  uint8_t cmd;
  uint16_t payload_length;
  unsigned char msg_payload[MAX_RESPONSE_PAYLOAD];
} __attribute__((packed)) NCSI_NL_RSP_T;

NCSI_NL_RSP_T ncsi_nl_rsp;
#endif

static int ftgmac100_alloc_rx_page(struct ftgmac100 *priv,
                                   int rxdes_idx,  gfp_t gfp);

#ifdef CONFIG_FTGMAC100_NCSI

#define NCSI_HDR_LEN 16
#define ETH_P_NCSI 0x88F8

struct mutex ncsi_mutex;

static struct sock *netlink_sk = NULL;
static DEFINE_MUTEX(netlink_mutex);


static int Rx_NCSI(struct sk_buff *skb, struct net_device *dev, struct packet_type *pt, struct net_device *orig_dev);


// worker function for AEN packets
static void
ftgmac_aen_worker(struct work_struct *work)
{
	struct ftgmac100 *lp = container_of(work, struct ftgmac100, work_aen);
	struct nlmsghdr *nlh;
	int pid;
	AEN_Packet packet;
	NCSI_NL_RSP_T *buf;

	/* for outgoing message response */
	struct sk_buff *skb_out;
	int msg_size;
	int res;

	AEN_PRINT("ftgmac workqueue - AEN packet, pid=%d\n", lp->aen_pid);
	// check if there's work to do
	if (kfifo_is_empty(&lp->AEN_buffer)) {
		AEN_PRINT(KERN_ERR "%s: Empty AEN fifo\n", __FUNCTION__);
		return;
	}

	while (!kfifo_is_empty(&lp->AEN_buffer))
	{
		kfifo_out(&lp->AEN_buffer, &packet, 1);

		// check if there is a registered AEN handler for this device
		if (lp->aen_pid == 0) {
			AEN_PRINT("%s: no registered AEN handler found\n", __FUNCTION__);
			return;
		} else {
			// AEN handler registered, check if the process is still running
			struct pid *pid_struct = find_get_pid(lp->aen_pid);
			struct task_struct *task = pid_task(pid_struct,PIDTYPE_PID);
			if (task == NULL) {
				printk(KERN_ERR "%s: AEN handler (pid:%d) does not appear to be running\n",
				       __FUNCTION__, lp->aen_pid);
				return;
			}
		}

		AEN_PRINT("%s: packet->AEN_Type = 0x%x\n", __FUNCTION__, packet.AEN_Type);
		AEN_PRINT("%s: packet->Optional_AEN_Data = 0x%lx\n", __FUNCTION__, (long)packet.Optional_AEN_Data[0]);
		AEN_PRINT("%s: fifo len=%d\n", __FUNCTION__, kfifo_len(&lp->AEN_buffer));

		msg_size =  sizeof(NCSI_NL_RSP_HDR_T) + sizeof(AEN_Packet);
		pid = lp->aen_pid; /*pid of ncsid */
		skb_out = nlmsg_new(msg_size,0);
		if (!skb_out) {
			printk(KERN_ERR "%s: Failed to allocate new skb\n", __FUNCTION__);
			return;
		}

		nlh=nlmsg_put(skb_out,0,0,NLMSG_DONE,msg_size,0);
		NETLINK_CB(skb_out).dst_group = 0; /* not in mcast group */

		buf = (NCSI_NL_RSP_T *)nlmsg_data(nlh);
		buf->payload_length = sizeof(AEN_Packet);
		memcpy(buf->msg_payload, &packet, sizeof(AEN_Packet));

		AEN_PRINT(KERN_INFO, "%s, %d 0x%x 0x%x", __FUNCTION__,
						((NCSI_NL_RSP_T *)(nlmsg_data(nlh)))->payload_length,
						((NCSI_NL_RSP_T *)(nlmsg_data(nlh)))->msg_payload[0],
						((NCSI_NL_RSP_T *)(nlmsg_data(nlh)))->msg_payload[1]
					);

		res = nlmsg_unicast(netlink_sk, skb_out, pid);

		if (res<0) {
			AEN_PRINT(KERN_INFO "%s: Error sending to AEN Handler, res=%d\n",
						__FUNCTION__, res);
		} else {
			AEN_PRINT(KERN_INFO "%s: sending AEN to handler\n", __FUNCTION__);
		}
	}
	return;
}

static int get_netdevice_idx(char *name)
{
    int index = -1;

    if (0 == strcmp("eth0", name))
    {
        index = 0;
    }
    else if (0 == strcmp("eth1", name))
    {
        index = 1;
    }

    return index;
}

static struct packet_type ptype_ncsi[2] __read_mostly = {
    {.type = __constant_htons(ETH_P_NCSI), .dev = NULL, .func = Rx_NCSI},
    {.type = __constant_htons(ETH_P_NCSI), .dev = NULL, .func = Rx_NCSI},
};

static int
Rx_NCSI(struct sk_buff *skb, struct net_device *dev, struct packet_type *pt, struct net_device *orig_dev)
{
  struct ftgmac100 *lp = netdev_priv(dev);
  NCSI_Response_Packet *resp;
  u16 pld_len, resp_len;
  static AEN_Packet localAENbuf = {0};

  if (!(skb = skb_share_check(skb, GFP_ATOMIC)))
    return NET_RX_DROP;

  if (!pskb_may_pull(skb, NCSI_HDR_LEN)) {
    kfree_skb(skb);
    return NET_RX_DROP;
  }

  resp = (NCSI_Response_Packet *)skb_mac_header(skb);
  pld_len = be16_to_cpu(resp->Payload_Length);

  if (!pskb_may_pull(skb, NCSI_HDR_LEN+pld_len)) {
    kfree_skb(skb);
    return NET_RX_DROP;
  }
  resp = (NCSI_Response_Packet *)skb_mac_header(skb);
  resp_len = ETH_HLEN + NCSI_HDR_LEN + pld_len;

  // handle AEN packet
  if ((resp->MC_ID == 0x00) &&
      (resp->Header_Revision == 0x01) &&
      (resp->IID == 0x00) &&
      (resp->Command == 0xFF)) {
    AEN_PRINT("ftgmac: dev:%s AEN received, Type=0x%x, payload=0x%lx\n", dev->name,
              ((AEN_Packet *)resp)->AEN_Type, (long)((AEN_Packet *)resp)->Optional_AEN_Data[0]);

    if ((resp_len > sizeof(AEN_Packet)) ||
        (kfifo_is_full(&lp->AEN_buffer))) {
      printk("ftgmac: AEN packet dropped, len=%d, (max=%d), (skb(%d)), fifofull(%d), \n", resp_len,
			        sizeof(AEN_Packet), (skb->tail - (u8 *)resp), kfifo_is_full(&lp->AEN_buffer));
      kfree_skb(skb);
      return NET_RX_DROP;
    }

    memcpy(&localAENbuf, resp, resp_len);
    kfifo_in(&lp->AEN_buffer, (AEN_Packet *)&localAENbuf, 1);
    queue_work(lp->ncsi_wq, &lp->work_aen);

    kfree_skb(skb);
    return NET_RX_SUCCESS;
  }


  if (resp_len > sizeof(NCSI_Response_Packet)) {
    kfree_skb(skb);
    return NET_RX_DROP;
  }

  memcpy(&lp->NCSI_Respond, resp, resp_len);
  complete(&lp->ncsi_complete);

  kfree_skb(skb);
  return NET_RX_SUCCESS;
}

#define TX_BUF_SIZE 1536
/******************************************************************************
 * NCSI Functions
 *****************************************************************************/
/* Forward delcarations */
static int ftgmac100_hard_start_xmit(struct sk_buff *skb,
		struct net_device *dev);
static struct
ftgmac100_rxdes *ftgmac100_rx_locate_first_segment(struct ftgmac100 *priv);
static bool ftgmac100_rxdes_last_segment(struct ftgmac100_rxdes *rxdes);
static int ftgmac100_current_rxdes_idx(const struct ftgmac100 *priv);
static struct page *ftgmac100_rxdes_get_page(struct ftgmac100 *priv, int idx);
static unsigned int ftgmac100_rxdes_data_length(struct ftgmac100_rxdes *rxdes);
static void ftgmac100_rxdes_set_dma_own(struct ftgmac100_rxdes *rxdes);
static void ftgmac100_rx_pointer_advance(struct ftgmac100 *priv);
static void ftgmac100_set_mac(struct ftgmac100 *priv, const unsigned char *mac);

static int
ftgmac100_wait_to_send_packet(struct sk_buff *skb, struct net_device *dev) {
	int ret;

	/* Disable tx/rx to avoid kernel panic due to race condition */
	local_bh_disable();
	ret = ftgmac100_hard_start_xmit(skb, dev);
	local_bh_enable();

	return ret;
}

void NCSI_Struct_Initialize(struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
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
	for (i = 0; i < 4; i++)
		lp->Payload_Pad[i] = 0;

	for (i = 0; i < MAX_NCSI_DATA_PAYLOAD; i++)
		lp->Payload_Data[i] = 0;
}

void Calculate_Checksum(struct net_device *dev, unsigned char *buffer_base,
		int Length)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned int i, CheckSum = 0;
	unsigned int Data, Data1;

	for (i = 0; i < ((Length - 14) / 2); i++) {
		Data = buffer_base[i * 2];
		Data1 = buffer_base[i * 2 + 1];
		CheckSum += ((Data << 8) + Data1);
	}
	lp->Payload_Checksum = (~(CheckSum) + 1); /* 2's complement */
	/* Inverse for insert into buffer */
	Data = (lp->Payload_Checksum & 0xFF000000) >> 24;
	Data1 = (lp->Payload_Checksum & 0x000000FF) << 24;
	lp->Payload_Checksum =
		(lp->Payload_Checksum & 0x00FFFF00) + Data + Data1;
	Data = (lp->Payload_Checksum & 0x00FF0000) >> 8;
	Data1 = (lp->Payload_Checksum & 0x0000FF00) << 8;
	lp->Payload_Checksum =
		(lp->Payload_Checksum & 0xFF0000FF) + Data + Data1;
}

#ifdef DEBUG_PRINT_NCSI_PACKET
void print_packet(struct sk_buff *skb, int Length)
{
  int i=0;

  printk("NCSI package sent (sizeof(ctrl)=%d)\n",
            sizeof(NCSI_Command_Packet));

  for (i=0; i<(34+Length); ++i) {
    if (i%16 == 0)
      printk("0x%04x:    ", i);
    printk("%02x", *(unsigned char *)(skb->data+i));
    if ((i%4 == 3) && (i%15 != 0))
      printk(" ");

    if (i%16 == 15)
      printk("\n");
  }
  printk("\n\n");
}
#endif


void copy_data(struct net_device *dev, struct sk_buff *skb, int Length)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	memcpy((unsigned char *)(skb->data + 30), &lp->Payload_Data, Length);
	Calculate_Checksum(dev, skb->data + 14, 30 + Length);

	memcpy((unsigned char *)(skb->data + 30 + Length),
         &lp->Payload_Checksum, 4);

#ifdef DEBUG_PRINT_NCSI_PACKET
  print_packet(skb, Length);
#endif
}


#if 0
void NCSI_Rx(struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);

	unsigned long length, i = 0;
	int count = 0;
	struct ftgmac100_rxdes *rxdes;

ncsi_rx:
	i = 0;
	do {
		rxdes = ftgmac100_rx_locate_first_segment(lp);
		i++;
		udelay(1000);
	} while ((!rxdes) && (i < NCSI_LOOP));

	if (i == NCSI_LOOP) {
		/* printk("NCSI_Rx: Failed\n"); */
		return;
	}

	if (!ftgmac100_rxdes_last_segment(rxdes)) {
		/* printk("NCSI_RX: Skip Partial Packet\n"); */
		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(lp);
		/* Check next descriptor for response packet */
		if (count++ <= RX_QUEUE_ENTRIES)
			goto ncsi_rx;
	}

	int rxdes_idx = ftgmac100_current_rxdes_idx(lp);
	struct page *page = ftgmac100_rxdes_get_page(lp, rxdes_idx);
	unsigned char *tbuf = (unsigned char *)page_address(page);

	length = ftgmac100_rxdes_data_length(rxdes);
	if (length <= 128 && tbuf[12] == 0x88 && tbuf[13] == 0xF8) {
		memcpy(&lp->NCSI_Respond, tbuf, length);
		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(lp);
		return;
	} else {
#ifdef NCSI_DEBUG
		printk("NCSI_RX: Skip len: %d, proto: %x:%x\n",
				length, tbuf[12], tbuf[13]);
#endif
		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(lp);
		/* Check next descriptor for response packet */
		if (count++ <= RX_QUEUE_ENTRIES)
			goto ncsi_rx;
	}
}
#endif

void DeSelect_Package(struct net_device *dev, int Package_ID)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff *skb;
	int tmo;

	do {
		skb = dev_alloc_skb(TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		/* TX */
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = DESELECT_PACKAGE;
		/* Internal Channel ID = 0x1F, 0x1F means all channel */
		Combined_Channel_ID = (Package_ID << 5) + 0x1F;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data(dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet(skb, dev);
		/* RX */
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk("Retry: Command = %x, Response_Code = %x\n",
				lp->NCSI_Request.Command,
				lp->NCSI_Respond.Response_Code);
			printk("IID: %x:%x, Command: %x:%x\n", lp->InstanceID,
				lp->NCSI_Respond.IID, lp->NCSI_Request.Command,
				lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		} else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

int Select_Package(struct net_device *dev, int Package_ID)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	int Found = 0;
	struct sk_buff *skb;
	int tmo;

	do {
		skb = dev_alloc_skb(TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);

		/* RX */
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk("Retry: Command = %x, Response_Code = %x\n",
				lp->NCSI_Request.Command,
				lp->NCSI_Respond.Response_Code);
			printk("IID: %x:%x, Command: %x:%x\n", lp->InstanceID,
				lp->NCSI_Respond.IID, lp->NCSI_Request.Command,
				lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			Found = 0;
			lp->InstanceID--;
		} else {
			lp->Retry = 0;
			Found = 1;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;

	return Found;
}

void DeSelect_Active_Package(struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff *skb;
	int tmo;

	do {
		skb = dev_alloc_skb(TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		/* TX */
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = DESELECT_PACKAGE;
		/* Internal Channel ID = 0x1F, 0x1F means all channel */
		Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + 0x1F;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data(dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet(skb, dev);

		/* RX */
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80))
		|| (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED))
		&& (lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk("Retry: Command = %x, Response_Code = %x\n",
				lp->NCSI_Request.Command,
				lp->NCSI_Respond.Response_Code);
#endif
			lp->Retry++;
			lp->InstanceID--;
		} else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

int Select_Active_Package(struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID, Found = 0;
	struct sk_buff *skb;
	int tmo;

	do {
		skb = dev_alloc_skb(TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		/* TX */
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = SELECT_PACKAGE;
		/* Internal Channel ID = 0x1F */
		Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + 0x1F;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (4 << 8);
		memcpy((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		lp->NCSI_Request.Payload_Length = 4;
		memset((void *)lp->Payload_Data, 0, 4);
		lp->Payload_Data[3] = 1; /* Arbitration Disable */
		copy_data(dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len = 30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet(skb, dev);

		/* RX */
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk("Retry: Command = %x, Response_Code = %x\n",
					lp->NCSI_Request.Command,
					lp->NCSI_Respond.Response_Code);
			printk("IID: %x:%x, Command: %x:%x\n", lp->InstanceID,
					lp->NCSI_Respond.IID,
					lp->NCSI_Request.Command,
					lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
			Found = 0;
		} else {
			lp->Retry = 0;
			Found = 1;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;

	return Found;
}

int Clear_Initial_State(struct net_device *dev, int Channel_ID)
{
  struct ftgmac100 *lp = netdev_priv(dev);
  unsigned long Combined_Channel_ID, Found = 0;
  struct sk_buff * skb;
  int tmo;

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
    init_completion(&lp->ncsi_complete);
    ftgmac100_wait_to_send_packet (skb, dev);

//RX
    tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
    if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (CLEAR_INITIAL_STATE | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
      printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);

		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = GET_VERSION_ID;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
			(lp->NCSI_Respond.Command != (GET_VERSION_ID | 0x80)) ||
			(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
			(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
      printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;

  // Set mezz type based on IANA ID
	// MLX IANA = 00 00 81 19
	// Broadcom IANA = 00 00 11 3D
  if (lp->NCSI_Respond.Payload_Data[32] == 0x00 && lp->NCSI_Respond.Payload_Data[33] == 0x00 &&
    lp->NCSI_Respond.Payload_Data[34] == 0x81 && lp->NCSI_Respond.Payload_Data[35] == 0x19) {
    lp->mezz_type = MEZZ_MLX;
    printk("NCSI: Mezz Vendor = Mellanox\n");
  } else if (lp->NCSI_Respond.Payload_Data[32] == 0x00 && lp->NCSI_Respond.Payload_Data[33] == 0x00 &&
    lp->NCSI_Respond.Payload_Data[34] == 0x11 && lp->NCSI_Respond.Payload_Data[35] == 0x3D) {
    lp->mezz_type = MEZZ_BCM;
    printk("NCSI: Mezz Vendor = Broadcom\n");
  } else if ( lp->NCSI_Respond.Payload_Data[35] == 0x57 && lp->NCSI_Respond.Payload_Data[34] == 0x01 &&
    lp->NCSI_Respond.Payload_Data[33] == 0x00 && lp->NCSI_Respond.Payload_Data[32] == 0x00 ) {
    lp->mezz_type = MEZZ_INTEL;
    printk("NCSI: Mezz Vendor = Intel\n");
  } else {
    lp->mezz_type = MEZZ_UNKNOWN;
    printk("NCSI error: Unknown Mezz Vendor!\n");
  }
}

void Get_Capabilities (struct net_device * dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

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
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (GET_CAPABILITIES | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

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
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
			(lp->NCSI_Respond.Command != (AEN_ENABLE | 0x80)) ||
			(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
			(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
		  printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
		  printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;    lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

void Get_MAC_Address_intel(struct net_device *dev)
{
  struct ftgmac100 *lp = netdev_priv(dev);
  struct file *filp = NULL;
  char path[64]={0};
  char mac_addr[6] = {0x00,0x11,0x22,0x33,0x44,0x55};
  char mac_addr_size = sizeof(mac_addr);
  int i;
  mm_segment_t fs;

  fs = get_fs();
  set_fs(KERNEL_DS);

  sprintf(path, "/sys/class/i2c-dev/i2c-6/device/6-0054/eeprom");

  filp = filp_open(path, O_RDONLY, 0);
  if ( (NULL == filp) || IS_ERR(filp) )
  {
    printk("[%s]Cannot use an error file pointer to get the intel NIC MAC\n",__func__);
    printk("Use the default MAC\n");
  }
  else
  {
    filp->f_pos = 0x1907;
    vfs_read(filp, (char *)mac_addr, mac_addr_size, &filp->f_pos);
  }

  set_fs(fs);

  if ( NULL != filp && !IS_ERR(filp) )
  {
    filp_close(filp, NULL);
  }

  printk("NCSI: MAC  ");
  for (i = 0; i < 6; i++)
      printk("%02X:", mac_addr[i]);

  printk("\n");
  memcpy(dev->dev_addr, mac_addr, mac_addr_size);
  memcpy(lp->NCSI_Request.SA, mac_addr, mac_addr_size);

  ftgmac100_set_mac(lp, dev->dev_addr);
}

void Get_MAC_Address_mlx(struct net_device * dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID, i;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = 0x50;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
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
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
			(lp->NCSI_Respond.Command != (0x50 | 0x80)) ||
			(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
			(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	ftgmac100_set_mac(lp, dev->dev_addr);
}


#define BCM_MAC_ADDR_OFFSET   32
void Get_MAC_Address_bcm(struct net_device * dev)
{
  struct ftgmac100 *lp = netdev_priv(dev);
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;
  int tmo;
  uint16_t offset, carry;

  do {
    skb = dev_alloc_skb (TX_BUF_SIZE + 16);
    memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
    lp->InstanceID++;
    lp->NCSI_Request.IID = lp->InstanceID;
    lp->NCSI_Request.Command = 0x50;
    Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
    lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
    lp->NCSI_Request.Payload_Length = (12 << 8);
    memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
    lp->NCSI_Request.Payload_Length = 12;
    lp->Payload_Data[0] = 0x00;
    lp->Payload_Data[1] = 0x00;
    lp->Payload_Data[2] = 0x11;
    lp->Payload_Data[3] = 0x3D;

    lp->Payload_Data[4] = 0x00;
    lp->Payload_Data[5] = 0x01;
    lp->Payload_Data[6] = 0x00;
    lp->Payload_Data[7] = 0x00;

    //copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
    memcpy ((unsigned char *)(skb->data + 30), &lp->Payload_Data, lp->NCSI_Request.Payload_Length);
    //skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
    skb->len =  30 + lp->NCSI_Request.Payload_Length;
    init_completion(&lp->ncsi_complete);
    ftgmac100_wait_to_send_packet (skb, dev);

//RX
    tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
    if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (0x50 | 0x80)) || (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
      printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
    printk("%02X:", lp->NCSI_Respond.Payload_Data[BCM_MAC_ADDR_OFFSET+i]);
  printk("\n");

  carry = BMC_MAC_OFFSET;
  offset = BCM_MAC_ADDR_OFFSET + ETH_ALEN - 1;
  do {
    // Start with the last nimble of the MAC address, then add the carry
    // , if any, to the upper nimble.
    carry += (uint8_t)(lp->NCSI_Respond.Payload_Data[offset]);
    lp->NCSI_Respond.Payload_Data[offset] = (char)carry;
    carry = carry >> 8;
  } while (carry != 0 && --offset >= BCM_MAC_ADDR_OFFSET);

  memcpy(lp->NCSI_Request.SA, &lp->NCSI_Respond.Payload_Data[BCM_MAC_ADDR_OFFSET], 6);
  memcpy(dev->dev_addr, &lp->NCSI_Respond.Payload_Data[BCM_MAC_ADDR_OFFSET], 6);

  /* Update the MAC address */
  ftgmac100_set_mac(lp, dev->dev_addr);
}


#define ETH_HDR_LEN        14
#define NCSI_HDR_LEN       16
#define OEM_HDR_LEN_BCM    12

#define BCM_C16_ADDR_CNT_OFFSET   8
#define BCM_C16_MAC_ADDR_OFFSET   12
int Get_MAC_Address_Bcm_c16 (struct net_device * dev)
{
  struct ftgmac100 *lp = netdev_priv(dev);
  unsigned long Combined_Channel_ID, i;
  struct sk_buff * skb;
  unsigned int oem_payload_len = 4;
  unsigned int oem_cmd_len = OEM_HDR_LEN_BCM + oem_payload_len;

  unsigned char addr_cnt;
  unsigned char mac_addr[6];
  int tmo;

do {
  skb = dev_alloc_skb (TX_BUF_SIZE + 16);
  memset(skb->data, 0, TX_BUF_SIZE + 16);
//TX
  lp->InstanceID++;
  lp->NCSI_Request.IID = lp->InstanceID;
  lp->NCSI_Request.Command = 0x50;
  Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
  lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
  lp->NCSI_Request.Payload_Length = (oem_cmd_len << 8);
  memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, (ETH_HDR_LEN+NCSI_HDR_LEN));
  lp->NCSI_Request.Payload_Length = oem_cmd_len;
  lp->Payload_Data[0] = 0x00;
  lp->Payload_Data[1] = 0x00;
  lp->Payload_Data[2] = 0x11;
  lp->Payload_Data[3] = 0x3D;

  lp->Payload_Data[4] = 0x00;
  lp->Payload_Data[5] = 0x16;
  lp->Payload_Data[6] = 0x00;
  lp->Payload_Data[7] = oem_payload_len;

  //copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
	memcpy ((unsigned char *)(skb->data + 30), &lp->Payload_Data, lp->NCSI_Request.Payload_Length);
  //skb->len = (ETH_HDR_LEN+NCSI_HDR_LEN) + lp->NCSI_Request.Payload_Length + 4;
	skb->len = (ETH_HDR_LEN+NCSI_HDR_LEN) + lp->NCSI_Request.Payload_Length ;
  init_completion(&lp->ncsi_complete);
  ftgmac100_wait_to_send_packet (skb, dev);

//RX
  tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
  if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (0x50 | 0x80)))
       && (lp->Retry < RETRY_COUNT)) {
    printk ("Retry: Command = %x, Response_Code = %x\n",
            lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
    printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID,
            lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
    lp->Retry++;
    lp->InstanceID--;
  }
  else {
    break;
  }
} while ((lp->Retry > 0) && (lp->Retry <= RETRY_COUNT));

  if (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED || lp->Retry >= RETRY_COUNT) {
    printk("Bcm NCSI: GetMCAddr rsp 0x%x, rsn 0x%x, rtry %d\n",
            lp->NCSI_Respond.Response_Code, lp->NCSI_Respond.Reason_Code, lp->Retry);
    lp->Retry = 0;
    return -1;
  }

  addr_cnt = lp->NCSI_Respond.Payload_Data[BCM_C16_ADDR_CNT_OFFSET];
  printk("Bcm NCSI: MCAddrCnt: %d, Addr1: ", addr_cnt);
  // Need to reverse the order as byte 5 comes first
  for (i = 0; i < 6; i++) {
    mac_addr[i] = lp->NCSI_Respond.Payload_Data[BCM_C16_MAC_ADDR_OFFSET+(6-1-i)];
    printk("%02X:", mac_addr[i]);
  }
  printk("\n");

  memcpy(lp->NCSI_Request.SA, mac_addr, 6);
  memcpy(dev->dev_addr, mac_addr, 6);

  /* Update the MAC address */
  ftgmac100_set_mac(lp, dev->dev_addr);
  lp->Retry = 0;
  return 0;
}


#define BCM_RETRY_MAX	2
int Prepare_for_Host_Powerup_Bcm (struct net_device * dev,
                         unsigned int host_id, unsigned int reinit_type)
{
	int ret = 0, tmo;
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	unsigned char oem_cmd_type = 0x1A;
	unsigned int oem_payload_len = 4;
	unsigned int oem_cmd_len = OEM_HDR_LEN_BCM + oem_payload_len;
	int data_len = (ETH_HDR_LEN+NCSI_HDR_LEN) + oem_cmd_len + 4;

#ifdef TIME_POWERUP_PREP
	unsigned long loop_cnt=0;
	struct timeval t_start, t_end;
	long usec_elapsed;
#endif

	/* ensure only 1 instance is accessing global NCSI structure */
	mutex_lock(&ncsi_mutex);

	lp->Retry = 0;
	do {
		lp->InstanceID = (lp->InstanceID+1) & 0xff;

		skb = dev_alloc_skb(data_len + 16);
		memset(skb->data, 0, data_len);
//TX
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = 0x50;
		Combined_Channel_ID = (lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (oem_cmd_len << 8);
		memcpy(skb->data, &lp->NCSI_Request, (ETH_HDR_LEN+NCSI_HDR_LEN));

		// memset(lp->Payload_Data, 0, 64);
		// memset(&lp->NCSI_Respond, 0, sizeof(NCSI_Response_Packet));
		lp->Payload_Data[0] = 0x00;   // 0~3: IANA
		lp->Payload_Data[1] = 0x00;
		lp->Payload_Data[2] = 0x11;
		lp->Payload_Data[3] = 0x3D;

		lp->Payload_Data[4] = 0x00;   // OEM Playload Version
		lp->Payload_Data[5] = oem_cmd_type;   // OEM Command Type
		lp->Payload_Data[6] = 0x00;   // 6~7: OEM Payload Length
		lp->Payload_Data[7] = oem_payload_len;

		lp->Payload_Data[13] = (unsigned char)reinit_type; // 13: ReInit Type
		lp->Payload_Data[14] = 0x00;  // 14~15: HOST ID
		lp->Payload_Data[15] = (unsigned char)host_id;

		copy_data(dev, skb, oem_cmd_len);
		skb_put(skb, data_len);
		init_completion(&lp->ncsi_complete);
		skb_reset_network_header(skb);
		skb->dev = dev;
		dev_queue_xmit(skb);

#ifdef TIME_POWERUP_PREP
		do_gettimeofday(&t_start);   // Start the timer
#endif

//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  // NCSI_Rx(dev);
#ifdef TIME_POWERUP_PREP
		do_gettimeofday(&t_end);     // Stop the timer
		usec_elapsed = (t_end.tv_sec - t_start.tv_sec) * 1000000 +
					(t_end.tv_usec - t_start.tv_usec);
		// printk("Time elapsed: %ld us, loop_cnt: %lu/%u, rsp 0x%x, [IID: 0x%x:0x%x], [tx%d:rx%d]\n",
		// 	 usec_elapsed, loop_cnt, NCSI_LOOP, lp->NCSI_Respond.Command,
		// 	 lp->InstanceID, lp->NCSI_Respond.IID, lp->tx_pointer, lp->rx_pointer);
		printk("Time elapsed: %ld us, rsp 0x%x, [IID: 0x%x:0x%x]\n",
			 usec_elapsed, lp->NCSI_Respond.Command, lp->InstanceID, lp->NCSI_Respond.IID);
#endif
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) || (lp->NCSI_Respond.Command != (0x50 | 0x80))) &&
			(lp->Retry <= BCM_RETRY_MAX)) {
			if (lp->Retry < BCM_RETRY_MAX) {
				printk ("Retry [%d]: Command = 0x%x (0x%x), Response_Code = 0x%x, Reason_Code = 0x%x\n", lp->Retry,
					lp->NCSI_Request.Command, oem_cmd_type, lp->NCSI_Respond.Response_Code, lp->NCSI_Respond.Reason_Code);
				printk ("IID: 0x%x:0x%x, Command: 0x%x:0x%x\n",
					lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
				lp->InstanceID--;
			}
			lp->Retry++;
		} else {
			break;
		}
	} while ((lp->Retry > 0) && (lp->Retry <= BCM_RETRY_MAX));

	mutex_unlock(&ncsi_mutex);

	printk("Bcm NCSI: powerup prep for slot%d ", host_id);
	if (!tmo) {
		printk("timed out (%d)!\n", lp->Retry);
		ret = -lp->Retry;
	} else if (lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED) {
		printk("failed! (Resp code:0x%x, Reason code:0x%x)\n",
		lp->NCSI_Respond.Response_Code, lp->NCSI_Respond.Reason_Code);
		ret = lp->NCSI_Respond.Response_Code;
	} else {
		printk("succeeded.\n");
		ret = 0;
	}

	lp->Retry = 0;

	return ret;
}


/* sysfs hooks */
#define HOST_ID_MIN   1
#define HOST_ID_MAX   4

#define REINIT_TYPE_FULL          0
#define REINIT_TYPE_HOST_RESOURCE 1
/**
 *  Set function to write firmware to device's persistent memory
 */
static ssize_t Perform_Nic_Powerup_Prep(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int combo_id, reinit_type, host_id;
	struct net_device *netdev = to_net_dev(dev);
	struct ftgmac100 *lp = netdev_priv(netdev);

	// Check whether NCSI is ready and whether it's Broadcom NIC
	if (lp->mezz_type != MEZZ_BCM) {
		//printk("\nNIC Powerup Prep: Not a BCM NIC.\n");
		return count;
	}

	// This is a combo ID consists of the following fields:
	// bit 11~8: reinit_type
	// bit 7~0:  host_id
	sscanf(buf, "%u", &combo_id);
	host_id = combo_id & 0xFF;
	reinit_type = (combo_id>>8) & 0xF;
	if (host_id < HOST_ID_MIN || host_id > HOST_ID_MAX) {
		printk("\nNIC Powerup Prep Err: invalid host id %d!\n", host_id);
		return -1;
	}
	if (reinit_type > REINIT_TYPE_HOST_RESOURCE) {
		printk("\nNIC Powerup Prep Err: invalid reinit type %d!\n", reinit_type);
		return -1;
	}
	lp->powerup_prep_host_id = combo_id;
	printk("\nNIC Powerup Prep (type %u) for slot %u.\n", reinit_type, host_id);

	// Send NC-SI cmd
	if (Prepare_for_Host_Powerup_Bcm(netdev, host_id, reinit_type) != 0) {
		return -1;
	}

	// clear the attribute?

	return count;
}

static ssize_t Powerup_Prep_Show_Host_Id(struct device *dev,
										 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_dev(dev);
	struct ftgmac100 *lp = netdev_priv(netdev);
	return sprintf(buf, "%u\n", lp->powerup_prep_host_id);
}

/**
 * powerup_prep_host_id attribute to be exported per ast_gmac.0 interface through sysfs
 * (/sys/devices/platform/ast_gmac.0/powerup_prep_host_id).
 */
static DEVICE_ATTR(powerup_prep_host_id, S_IRUGO | S_IWUSR | S_IWGRP,
					Powerup_Prep_Show_Host_Id, Perform_Nic_Powerup_Prep);



static void send_ncsi_cmd(struct net_device * dev, unsigned char channel_id,
													unsigned char cmd, uint16_t length, unsigned char *buf,
													uint16_t *res_length, unsigned char *res_buf)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;
  int ncsi_pad = (4 - length%4)%4;  // NCSI requries 32 bit alignment
	int data_len = 30 + length + ncsi_pad + 4;
  int i;

	/* ensure only 1 instance is accessing global NCSI structure */
	mutex_lock(&ncsi_mutex);

	do {
		skb = dev_alloc_skb(data_len + 16);
		memset(skb->data, 0, data_len);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = cmd;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + channel_id;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = htons(length);

#ifdef DEBUG_PRINT_NCSI_PACKET
    printk("ch_id(%02x), iid(0x%02x), len(0x%02x), pad(%02x)\n",
           channel_id, lp->InstanceID, length, ncsi_pad);
#endif

		memcpy(skb->data, &lp->NCSI_Request, 30);
		memcpy(lp->Payload_Data, buf, length);
    memcpy((unsigned char *)(skb->data + 30), &lp->Payload_Data, length);
    //Calculate_Checksum(dev, skb->data + 14, 30 + length);
    //memcpy((unsigned char *)(skb->data + 30 + length + ncsi_pad),
          //&lp->Payload_Checksum, 4);
#ifdef DEBUG_PRINT_NCSI_PACKET
  print_packet(skb, length+ncsi_pad);
#endif

		skb_put(skb, data_len);
		init_completion(&lp->ncsi_complete);
		skb_reset_network_header(skb);
		skb->dev = dev;
		dev_queue_xmit(skb);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/10);
		if ((!tmo ||
			(lp->NCSI_Respond.IID != lp->InstanceID) ||
			(lp->NCSI_Respond.Command != (cmd | 0x80)) ||
			(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
			(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;

	if (tmo) {
		*res_length = be16_to_cpu(lp->NCSI_Respond.Payload_Length);
		memcpy(res_buf, (u8 *)&lp->NCSI_Respond.Response_Code, *res_length);
	}
	else {
		*res_length = 4;
		memset(res_buf, 0xff, *res_length);
	}

	mutex_unlock(&ncsi_mutex);

#ifdef DEBUG_PRINT_NCSI_PACKET
	if (!tmo) {
		printk("timed out!");
	} else {
		printk("cmd response detail:\n");
		for (i=0; i<be16_to_cpu(lp->NCSI_Respond.Payload_Length); ++i) {
      if (i%16 == 0)
        printk("0x%04x:    ",i);
			printk("%02x", ((u8 *)&lp->NCSI_Respond.Response_Code)[i]);
      if ((i%4 == 3) && (i%15 != 0))
        printk(" ");

      if (i%16 == 15)
        printk("\n");
		}
	}
	printk("\n\n");
#endif
}


/* find the matching net device */
static struct net_device* getdev(char *dev_name)
{
  struct net_device *dev;

  read_lock(&dev_base_lock);
  dev = first_net_device(&init_net);
  while (dev) {
#ifdef NCSI_NL_DEBUG
    printk(KERN_INFO "found [%s]\n", dev->name);
#endif
    if (strcmp(dev_name, dev->name) == 0)
      break;
    dev = next_net_device(dev);
  }
  read_unlock(&dev_base_lock);

  return dev;
}



static void ncsi_recv_msg_cb(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	int pid;
  struct net_device *dev;
  NCSI_NL_MSG_T *buf;

  /* for outgoing message response */
	struct sk_buff *skb_out;
	int msg_size;
	unsigned char *msg;
	int res;

	nlh = (struct nlmsghdr*)skb->data;
  buf = (NCSI_NL_MSG_T*)nlmsg_data(nlh);

#ifdef NCSI_NL_DEBUG
	int i = 0;
  printk("%s    channel_id=0x%x, cmd 0x%x\n    data=",
	        __FUNCTION__, buf->channel_id, buf->cmd);
  for (i=0; i<buf->payload_length; ++i)
      printk("0x%x ", buf->msg_payload[i]);
  printk("\n");
#endif

  /* find the matching net device for NC-SI cmd */
  dev = getdev(buf->dev_name);
  if (!dev) {
    printk(KERN_ERR "%s: failed to find matching device\n", __FUNCTION__);
    return;
  }

	/* handle AEN daemon registration message */
	if ((buf->channel_id == REG_AEN_CH) && (buf->cmd == REG_AEN_CMD)) {
    struct ftgmac100 *lp = netdev_priv(dev);
    lp->aen_pid = nlh->nlmsg_pid;
		printk(KERN_INFO "ftgmac: %s AEN handler (pid:%d) registered\n",
		       dev->name, lp->aen_pid);
		return;
	}

  if (netif_queue_stopped(dev))
    return;

  if (buf->cmd == 0xde) {
    int j = 0;
    ncsi_nl_rsp.payload_length = buf->payload_length;
    memcpy(ncsi_nl_rsp.msg_payload, buf->msg_payload, ncsi_nl_rsp.payload_length);
    for (j = 0; j < ncsi_nl_rsp.payload_length; ++j) {
      ncsi_nl_rsp.msg_payload[j] -= 1;
    }
  } else {
    send_ncsi_cmd(dev, buf->channel_id, buf->cmd, buf->payload_length,
	             buf->msg_payload,
							 &(ncsi_nl_rsp.payload_length), &(ncsi_nl_rsp.msg_payload[0]));
  }
  ncsi_nl_rsp.cmd = buf->cmd;
	msg_size = sizeof(NCSI_NL_RSP_HDR_T) + ncsi_nl_rsp.payload_length;
	msg = (unsigned char *)&ncsi_nl_rsp;

  /* send the NC-Si response back via Netlink socket */
	pid = nlh->nlmsg_pid; /*pid of sending process */
	skb_out = nlmsg_new(msg_size,0);
	if(!skb_out) {
		printk(KERN_ERR "%s: Failed to allocate new skb\n", __FUNCTION__);
		return;
	}

	nlh=nlmsg_put(skb_out,0,0,NLMSG_DONE,msg_size,0);
	NETLINK_CB(skb_out).dst_group = 0; /* not in mcast group */
	memcpy(nlmsg_data(nlh), msg, msg_size);

	res=nlmsg_unicast(skb->sk, skb_out, pid);
	if(res<0)
	{
		printk(KERN_INFO "%s: Error while sending back to user\n", __FUNCTION__);
	}
}

/* call back function for netlink receive */
static void ncsi_recv_nl_msg(struct sk_buff *skb)
{
	mutex_lock(&netlink_mutex);
	ncsi_recv_msg_cb(skb);
	mutex_unlock(&netlink_mutex);
}


static int __init ncsi_nl_socket_init(void)
{
	struct netlink_kernel_cfg cfg = {
		/*  call back function when netlink msg is received */
		.input = ncsi_recv_nl_msg,
	};

	if (!netlink_sk) {
		netlink_sk = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
		if (!netlink_sk) {
			printk("%s: Error creating netlink socket\n", __FUNCTION__);
			return -10;
		}
		printk("%s: created netlink socket\n", __FUNCTION__);
	}

	return 0;
}


static void __exit ncsi_nl_socket_exit(void)
{
	if (netlink_sk) {
		netlink_kernel_release(netlink_sk);
		netlink_sk = NULL;
	}
}



void Set_MAC_Affinity_mlx(struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID, i;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = 0x50;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
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
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);
		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
			(lp->NCSI_Respond.Command != (0x50 | 0x80)) ||
			(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
			(lp->Retry != RETRY_COUNT)) {

#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID, i;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = SET_MAC_ADDRESS;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (8 << 8);
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		lp->NCSI_Request.Payload_Length = 8;
		for (i = 0; i < 6; i++) {
			lp->Payload_Data[i] = lp->NCSI_Request.SA[i];
		}
		/* MAC Addr Num = 1 --> addrs filter 1, fixed in sample code */
		lp->Payload_Data[6] = 1;
		/* AT + Reserved + E */
		lp->Payload_Data[7] = UNICAST + 0 + ENABLE_MAC_ADDRESS_FILTER;
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (SET_MAC_ADDRESS | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = ENABLE_BROADCAST_FILTERING;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (4 << 8);
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		lp->NCSI_Request.Payload_Length = 4;
		memset ((void *)lp->Payload_Data, 0, 4);
		lp->Payload_Data[3] = 0x1; //ARP, DHCP, NetBIOS
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (ENABLE_BROADCAST_FILTERING | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
		lp->Retry = 0;
}

void Disable_Multicast_Filter (struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff *skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = DISABLE_GLOBAL_MULTICAST_FILTERING;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DISABLE_GLOBAL_MULTICAST_FILTERING | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) && (lp->Retry != RETRY_COUNT)) {

#ifdef NCSI_DEBUG
      printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
      printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;    lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

void Disable_VLAN (struct net_device * dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

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
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DISABLE_VLAN | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;    lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

void Get_Parameters (struct net_device * dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = GET_PARAMETERS;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (GET_PARAMETERS | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x, \
				Resonpd.Command = %x, IID = %x, \
				lp->InstanceID = %x\n",
				lp->NCSI_Request.Command,
				lp->NCSI_Respond.Response_Code,
				lp->NCSI_Respond.Command, lp->NCSI_Respond.IID,
				lp->InstanceID);
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = ENABLE_CHANNEL_NETWORK_TX;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (ENABLE_CHANNEL_NETWORK_TX | 0x80))
		|(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = DISABLE_CHANNEL_NETWORK_TX;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length +
			(lp->NCSI_Request.Payload_Length % 4) + 8;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DISABLE_CHANNEL_NETWORK_TX | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		} else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

void Enable_Channel (struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = ENABLE_CHANNEL;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (ENABLE_CHANNEL | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

void Disable_Channel (struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);

		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = DISABLE_CHANNEL;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (4 << 8);
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		lp->NCSI_Request.Payload_Length = 4;
		memset ((void *)lp->Payload_Data, 0, 4);
		lp->Payload_Data[3] = 0x1; //ALD
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (DISABLE_CHANNEL | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {

#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));
	lp->Retry = 0;
}

int Get_Link_Status (struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = GET_LINK_STATUS;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = 0;
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (GET_LINK_STATUS | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
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

void Set_Link (struct net_device *dev)
{
	struct ftgmac100 *lp = netdev_priv(dev);
	unsigned long Combined_Channel_ID;
	struct sk_buff * skb;
	int tmo;

	do {
		skb = dev_alloc_skb (TX_BUF_SIZE + 16);
		memset(skb->data, 0, TX_BUF_SIZE + 16);
		//TX
		lp->InstanceID++;
		lp->NCSI_Request.IID = lp->InstanceID;
		lp->NCSI_Request.Command = SET_LINK;
		Combined_Channel_ID =
		(lp->NCSI_Cap.Package_ID << 5) + lp->NCSI_Cap.Channel_ID;
		lp->NCSI_Request.Channel_ID = Combined_Channel_ID;
		lp->NCSI_Request.Payload_Length = (8 << 8);
		memcpy ((unsigned char *)skb->data, &lp->NCSI_Request, 30);
		lp->NCSI_Request.Payload_Length = 8;
		memset ((void *)lp->Payload_Data, 0, 8);
		lp->Payload_Data[2] = 0x02; //full duplex
		lp->Payload_Data[3] = 0x04; //100M, auto-disable
		copy_data (dev, skb, lp->NCSI_Request.Payload_Length);
		skb->len =  30 + lp->NCSI_Request.Payload_Length + 4;
		init_completion(&lp->ncsi_complete);
		ftgmac100_wait_to_send_packet (skb, dev);

		//RX
		tmo = wait_for_completion_timeout(&lp->ncsi_complete, HZ/2);  //NCSI_Rx(dev);
		if ((!tmo || (lp->NCSI_Respond.IID != lp->InstanceID) ||
		(lp->NCSI_Respond.Command != (SET_LINK | 0x80)) ||
		(lp->NCSI_Respond.Response_Code != COMMAND_COMPLETED)) &&
		(lp->Retry != RETRY_COUNT)) {
#ifdef NCSI_DEBUG
			printk ("Retry: Command = %x, Response_Code = %x\n", lp->NCSI_Request.Command, lp->NCSI_Respond.Response_Code);
			printk ("IID: %x:%x, Command: %x:%x\n", lp->InstanceID, lp->NCSI_Respond.IID, lp->NCSI_Request.Command, lp->NCSI_Respond.Command);
#endif
			lp->Retry++;
			lp->InstanceID--;
		}
		else {
			lp->Retry = 0;
		}
	} while ((lp->Retry != 0) && (lp->Retry <= RETRY_COUNT));

	lp->Retry = 0;
}

void ncsi_start(struct net_device *dev) {
	struct ftgmac100 *priv = netdev_priv(dev);
	unsigned long Package_Found = 0, Channel_Found = 0, Re_Send = 0, Link_Status;

	int i = 0;
	priv->mezz_type = MEZZ_UNKNOWN;
	//NCSI Start
	//DeSelect Package/ Select Package
	NCSI_Struct_Initialize(dev);
	for (i = 0; i < 4; i++) {
		// DeSelect_Package (dev, i);
		// Package_Found = Select_Package (dev, i);
    Package_Found = 1;
		if (Package_Found == 1) {
			//AST2100/AST2050/AST1100 supports 1 slave only
			priv->NCSI_Cap.Package_ID = i;
			break;
		}
	}
	if (Package_Found != 0) {
		/*
		 * Initiali State
		 * Suppose 2 channels in current version, You could modify it to
		 * 0x1F to support 31 channels
		 */
		for (i = 0; i < 1; i++) {
			//Channel_Found = Clear_Initial_State(dev, i);
			Channel_Found = 1;
			if (Channel_Found == 1) {
				priv->NCSI_Cap.Channel_ID = i;
				printk ("Found NCSI NW Controller at (%d, %d)\n",
					priv->NCSI_Cap.Package_ID,
					priv->NCSI_Cap.Channel_ID);
				Clear_Initial_State(dev, i);
        //TODO: This is an issue in  Get_Version_ID that always returns
        //mezz_type to be -1, so it only calls Get_MAC_Address_bcm.
        //It may need to work with Mlx to find a solution.
#if defined(CONFIG_FBY2) || defined(CONFIG_FBY3_POC) || defined(CONFIG_YOSEMITE)    //For multi-host NIC initialization
        // Try Mlx first
        Get_MAC_Address_mlx(dev);
        Set_MAC_Affinity_mlx(dev);
        Clear_Initial_State(dev, i);
        mdelay(500);
        Get_Version_ID(dev);
        mdelay(500);

        // Then try Bcm
        if (priv->mezz_type == MEZZ_BCM) {
          if (Get_MAC_Address_Bcm_c16(dev) != 0) {
            Get_MAC_Address_bcm(dev);   // legacy cmd
          }
          mdelay(500);
        }
#else                                             //For single host NIC initialization
        mdelay(500);
        Get_Version_ID(dev);
        mdelay(500);
        if (priv->mezz_type == MEZZ_MLX) {
          Get_MAC_Address_mlx(dev);
          Set_MAC_Affinity_mlx(dev);
        } else if (priv->mezz_type == MEZZ_BCM ) {
          Get_MAC_Address_bcm(dev);
          mdelay(500);
        } else {
          Get_MAC_Address_intel(dev);
          mdelay(500);
        }

#endif

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
				Get_Parameters(dev);
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
					printk("Using NCSI Network Controller (%d, %d)\n",
						priv->NCSI_Cap.Package_ID,
						priv->NCSI_Cap.Channel_ID);
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
				//Disable_Channel(dev);
				Re_Send = 0;
				Channel_Found = 0;
			}
		}
	}
}
#endif /* CONFIG_FTGMAC100_NCSI */

/******************************************************************************
 * internal functions (hardware register access)
 *****************************************************************************/
#define INT_MASK_ALL_ENABLED	(FTGMAC100_INT_RPKT_LOST	| \
				 FTGMAC100_INT_XPKT_ETH		| \
				 FTGMAC100_INT_XPKT_LOST	| \
				 FTGMAC100_INT_AHB_ERR		| \
         FTGMAC100_INT_PHYSTS_CHG | \
				 FTGMAC100_INT_RPKT_BUF		| \
				 FTGMAC100_INT_NO_RXBUF)

#define INT_MASK_NCSI_ENABLED	(FTGMAC100_INT_RPKT_LOST	| \
				 FTGMAC100_INT_XPKT_ETH		| \
				 FTGMAC100_INT_XPKT_LOST	| \
				 FTGMAC100_INT_AHB_ERR		| \
				 FTGMAC100_INT_RPKT_BUF		| \
				 FTGMAC100_INT_NO_RXBUF)

static void ftgmac100_set_rx_ring_base(struct ftgmac100 *priv, dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_RXR_BADR);
}

static void ftgmac100_set_rx_buffer_size(struct ftgmac100 *priv,
		unsigned int size)
{
	size = FTGMAC100_RBSR_SIZE(size);
	iowrite32(size, priv->base + FTGMAC100_OFFSET_RBSR);
}

static void ftgmac100_set_normal_prio_tx_ring_base(struct ftgmac100 *priv,
						   dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_NPTXR_BADR);
}

static void ftgmac100_txdma_normal_prio_start_polling(struct ftgmac100 *priv)
{
	iowrite32(1, priv->base + FTGMAC100_OFFSET_NPTXPD);
}

static int ftgmac100_reset_hw(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	int i;

	/* NOTE: reset clears all registers */
	iowrite32(FTGMAC100_MACCR_SW_RST, priv->base + FTGMAC100_OFFSET_MACCR);
	for (i = 0; i < 5; i++) {
		unsigned int maccr;

		maccr = ioread32(priv->base + FTGMAC100_OFFSET_MACCR);
		if (!(maccr & FTGMAC100_MACCR_SW_RST))
			return 0;

		udelay(1000);
	}

	netdev_err(netdev, "software reset failed\n");
	return -EIO;
}

static void ftgmac100_set_mac(struct ftgmac100 *priv, const unsigned char *mac)
{
	unsigned int maddr = mac[0] << 8 | mac[1];
	unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

	iowrite32(maddr, priv->base + FTGMAC100_OFFSET_MAC_MADR);
	iowrite32(laddr, priv->base + FTGMAC100_OFFSET_MAC_LADR);
}

static void ftgmac100_init_hw(struct ftgmac100 *priv)
{
	/* setup ring buffer base registers */
	ftgmac100_set_rx_ring_base(priv,
				   priv->descs_dma_addr +
				   offsetof(struct ftgmac100_descs, rxdes));
	ftgmac100_set_normal_prio_tx_ring_base(priv,
					       priv->descs_dma_addr +
					       offsetof(struct ftgmac100_descs, txdes));

	ftgmac100_set_rx_buffer_size(priv, RX_BUF_SIZE);

	iowrite32(FTGMAC100_APTC_RXPOLL_CNT(1), priv->base + FTGMAC100_OFFSET_APTC);

	ftgmac100_set_mac(priv, priv->netdev->dev_addr);
}

#define MACCR_ENABLE_ALL	(FTGMAC100_MACCR_TXDMA_EN	| \
				 FTGMAC100_MACCR_RXDMA_EN	| \
				 FTGMAC100_MACCR_TXMAC_EN	| \
				 FTGMAC100_MACCR_RXMAC_EN	| \
				 FTGMAC100_MACCR_FULLDUP	| \
				 FTGMAC100_MACCR_CRC_APD	| \
				 FTGMAC100_MACCR_RX_RUNT	| \
				 FTGMAC100_MACCR_RX_BROADPKT)

static void ftgmac100_start_hw(struct ftgmac100 *priv, int speed)
{
	int maccr = MACCR_ENABLE_ALL;

	switch (speed) {
	default:
	case 10:
		break;

	case 100:
		maccr |= FTGMAC100_MACCR_FAST_MODE;
		break;

	case 1000:
		maccr |= FTGMAC100_MACCR_GIGA_MODE;
		break;
	}

	iowrite32(maccr, priv->base + FTGMAC100_OFFSET_MACCR);
}

static void ftgmac100_stop_hw(struct ftgmac100 *priv)
{
	iowrite32(0, priv->base + FTGMAC100_OFFSET_MACCR);
}

/******************************************************************************
 * internal functions (receive descriptor)
 *****************************************************************************/
static bool ftgmac100_rxdes_first_segment(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_FRS);
}

static bool ftgmac100_rxdes_last_segment(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_LRS);
}

static bool ftgmac100_rxdes_packet_ready(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_RXPKT_RDY);
}

static void ftgmac100_rxdes_set_dma_own(struct ftgmac100_rxdes *rxdes)
{
	/* clear status bits */
	rxdes->rxdes0 &= cpu_to_le32(FTGMAC100_RXDES0_EDORR);
}

static bool ftgmac100_rxdes_rx_error(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_RX_ERR);
}

static bool ftgmac100_rxdes_crc_error(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_CRC_ERR);
}

static bool ftgmac100_rxdes_frame_too_long(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_FTL);
}

static bool ftgmac100_rxdes_runt(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_RUNT);
}

static bool ftgmac100_rxdes_odd_nibble(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_RX_ODD_NB);
}

static unsigned int ftgmac100_rxdes_data_length(struct ftgmac100_rxdes *rxdes)
{
	return le32_to_cpu(rxdes->rxdes0) & FTGMAC100_RXDES0_VDBC;
}

static bool ftgmac100_rxdes_multicast(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & cpu_to_le32(FTGMAC100_RXDES0_MULTICAST);
}

static void ftgmac100_rxdes_set_end_of_ring(struct ftgmac100_rxdes *rxdes)
{
	rxdes->rxdes0 |= cpu_to_le32(FTGMAC100_RXDES0_EDORR);
}

static void ftgmac100_rxdes_set_dma_addr(struct ftgmac100_rxdes *rxdes,
					 dma_addr_t addr)
{
	rxdes->rxdes3 = cpu_to_le32(addr);
}

static dma_addr_t ftgmac100_rxdes_get_dma_addr(struct ftgmac100_rxdes *rxdes)
{
	return le32_to_cpu(rxdes->rxdes3);
}

static bool ftgmac100_rxdes_is_tcp(struct ftgmac100_rxdes *rxdes)
{
	return (rxdes->rxdes1 & cpu_to_le32(FTGMAC100_RXDES1_PROT_MASK)) ==
	       cpu_to_le32(FTGMAC100_RXDES1_PROT_TCPIP);
}

static bool ftgmac100_rxdes_is_udp(struct ftgmac100_rxdes *rxdes)
{
	return (rxdes->rxdes1 & cpu_to_le32(FTGMAC100_RXDES1_PROT_MASK)) ==
	       cpu_to_le32(FTGMAC100_RXDES1_PROT_UDPIP);
}

static bool ftgmac100_rxdes_tcpcs_err(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes1 & cpu_to_le32(FTGMAC100_RXDES1_TCP_CHKSUM_ERR);
}

static bool ftgmac100_rxdes_udpcs_err(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes1 & cpu_to_le32(FTGMAC100_RXDES1_UDP_CHKSUM_ERR);
}

static bool ftgmac100_rxdes_ipcs_err(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes1 & cpu_to_le32(FTGMAC100_RXDES1_IP_CHKSUM_ERR);
}

static void ftgmac100_rxdes_set_page(struct ftgmac100 *priv,
                                     int idx,
                                     struct page *page)
{
	priv->rxdes_pages[idx] = page;
}

static struct page *ftgmac100_rxdes_get_page(struct ftgmac100 *priv, int idx)
{
	return priv->rxdes_pages[idx];
}

/******************************************************************************
 * internal functions (receive)
 *****************************************************************************/
static int ftgmac100_next_rx_pointer(int pointer)
{
	return (pointer + 1) & (RX_QUEUE_ENTRIES - 1);
}

static void ftgmac100_rx_pointer_advance(struct ftgmac100 *priv)
{
	priv->rx_pointer = ftgmac100_next_rx_pointer(priv->rx_pointer);
}

static struct ftgmac100_rxdes *ftgmac100_current_rxdes(struct ftgmac100 *priv)
{
	return &priv->descs->rxdes[priv->rx_pointer];
}

static struct ftgmac100_rxdes *
ftgmac100_rx_locate_first_segment(struct ftgmac100 *priv)
{
	struct ftgmac100_rxdes *rxdes = ftgmac100_current_rxdes(priv);

	while (ftgmac100_rxdes_packet_ready(rxdes)) {
		if (ftgmac100_rxdes_first_segment(rxdes))
			return rxdes;

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	}

	return NULL;
}

static int ftgmac100_current_rxdes_idx(const struct ftgmac100 *priv)
{
    return priv->rx_pointer;
}

static bool ftgmac100_rx_packet_error(struct ftgmac100 *priv,
				      struct ftgmac100_rxdes *rxdes)
{
	struct net_device *netdev = priv->netdev;
	bool error = false;

	if (unlikely(ftgmac100_rxdes_rx_error(rxdes))) {
		if (net_ratelimit())
			netdev_info(netdev, "rx err\n");

		netdev->stats.rx_errors++;
		error = true;
	}

	if (unlikely(ftgmac100_rxdes_crc_error(rxdes))) {
		if (net_ratelimit())
			netdev_info(netdev, "rx crc err\n");

		netdev->stats.rx_crc_errors++;
		error = true;
	} else if (unlikely(ftgmac100_rxdes_ipcs_err(rxdes))) {
		if (net_ratelimit())
			;//netdev_info(netdev, "rx IP checksum err\n");

		error = true;
	}

	if (unlikely(ftgmac100_rxdes_frame_too_long(rxdes))) {
		if (net_ratelimit())
			netdev_info(netdev, "rx frame too long\n");

		netdev->stats.rx_length_errors++;
		error = true;
	} else if (unlikely(ftgmac100_rxdes_runt(rxdes))) {
		if (net_ratelimit())
			netdev_info(netdev, "rx runt\n");

		netdev->stats.rx_length_errors++;
		error = true;
	} else if (unlikely(ftgmac100_rxdes_odd_nibble(rxdes))) {
		if (net_ratelimit())
			netdev_info(netdev, "rx odd nibble\n");

		netdev->stats.rx_length_errors++;
		error = true;
	}

	return error;
}

static void ftgmac100_rx_drop_packet(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_rxdes *rxdes = ftgmac100_current_rxdes(priv);
	bool done = false;

	if (net_ratelimit())
		netdev_dbg(netdev, "drop packet %p\n", rxdes);

	do {
		if (ftgmac100_rxdes_last_segment(rxdes))
			done = true;

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	} while (!done && ftgmac100_rxdes_packet_ready(rxdes));

	netdev->stats.rx_dropped++;
}

static bool ftgmac100_rx_packet(struct ftgmac100 *priv, int *processed)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_rxdes *rxdes;
	struct sk_buff *skb;
	int rxdes_idx;
	bool done = false;

	rxdes = ftgmac100_rx_locate_first_segment(priv);
	if (!rxdes)
		return false;

	if (unlikely(ftgmac100_rx_packet_error(priv, rxdes))) {
		ftgmac100_rx_drop_packet(priv);
		return true;
	}

	/* start processing */
	skb = netdev_alloc_skb_ip_align(netdev, 128);
	if (unlikely(!skb)) {
		if (net_ratelimit())
			netdev_err(netdev, "rx skb alloc failed\n");

		ftgmac100_rx_drop_packet(priv);
		return true;
	}

	if (unlikely(ftgmac100_rxdes_multicast(rxdes)))
		netdev->stats.multicast++;

	/*
	 * It seems that HW does checksum incorrectly with fragmented packets,
	 * so we are conservative here - if HW checksum error, let software do
	 * the checksum again.
	 */
	if ((ftgmac100_rxdes_is_tcp(rxdes) && !ftgmac100_rxdes_tcpcs_err(rxdes)) ||
	    (ftgmac100_rxdes_is_udp(rxdes) && !ftgmac100_rxdes_udpcs_err(rxdes)))
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	rxdes_idx = ftgmac100_current_rxdes_idx(priv);
	do {
		dma_addr_t map = ftgmac100_rxdes_get_dma_addr(rxdes);
		struct page *page = ftgmac100_rxdes_get_page(priv, rxdes_idx);
		unsigned int size;

		dma_unmap_page(priv->dev, map, RX_BUF_SIZE, DMA_FROM_DEVICE);

		size = ftgmac100_rxdes_data_length(rxdes);
		skb_fill_page_desc(skb, skb_shinfo(skb)->nr_frags, page, 0, size);

		skb->len += size;
		skb->data_len += size;
		skb->truesize += PAGE_SIZE;

		if (ftgmac100_rxdes_last_segment(rxdes))
			done = true;

		ftgmac100_alloc_rx_page(priv, rxdes_idx, GFP_ATOMIC);

		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
		rxdes_idx = ftgmac100_current_rxdes_idx(priv);
	} while (!done);

	/* Small frames are copied into linear part of skb to free one page */
	if (skb->len <= 128) {
		skb->truesize -= PAGE_SIZE;
		__pskb_pull_tail(skb, skb->len);
	} else {
		/* We pull the minimum amount into linear part */
		__pskb_pull_tail(skb, ETH_HLEN);
	}
	skb->protocol = eth_type_trans(skb, netdev);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += skb->len;

	/* push packet to protocol stack */
	napi_gro_receive(&priv->napi, skb);

	(*processed)++;
	return true;
}

/******************************************************************************
 * internal functions (transmit descriptor)
 *****************************************************************************/
static void ftgmac100_txdes_reset(struct ftgmac100_txdes *txdes)
{
	/* clear all except end of ring bit */
	txdes->txdes0 &= cpu_to_le32(FTGMAC100_TXDES0_EDOTR);
	txdes->txdes1 = 0;
	txdes->txdes2 = 0;
	txdes->txdes3 = 0;
}

static bool ftgmac100_txdes_owned_by_dma(struct ftgmac100_txdes *txdes)
{
	return txdes->txdes0 & cpu_to_le32(FTGMAC100_TXDES0_TXDMA_OWN);
}

static void ftgmac100_txdes_set_dma_own(struct ftgmac100_txdes *txdes)
{
	/*
	 * Make sure dma own bit will not be set before any other
	 * descriptor fields.
	 */
	wmb();
	txdes->txdes0 |= cpu_to_le32(FTGMAC100_TXDES0_TXDMA_OWN);
}

static void ftgmac100_txdes_set_end_of_ring(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= cpu_to_le32(FTGMAC100_TXDES0_EDOTR);
}

static void ftgmac100_txdes_set_first_segment(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= cpu_to_le32(FTGMAC100_TXDES0_FTS);
}

static void ftgmac100_txdes_set_last_segment(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= cpu_to_le32(FTGMAC100_TXDES0_LTS);
}

static void ftgmac100_txdes_set_buffer_size(struct ftgmac100_txdes *txdes,
					    unsigned int len)
{
	txdes->txdes0 |= cpu_to_le32(FTGMAC100_TXDES0_TXBUF_SIZE(len));
}

static void ftgmac100_txdes_set_txint(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= cpu_to_le32(FTGMAC100_TXDES1_TXIC);
}

static void ftgmac100_txdes_set_tcpcs(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= cpu_to_le32(FTGMAC100_TXDES1_TCP_CHKSUM);
}

static void ftgmac100_txdes_set_udpcs(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= cpu_to_le32(FTGMAC100_TXDES1_UDP_CHKSUM);
}

static void ftgmac100_txdes_set_ipcs(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= cpu_to_le32(FTGMAC100_TXDES1_IP_CHKSUM);
}

static void ftgmac100_txdes_set_dma_addr(struct ftgmac100_txdes *txdes,
					 dma_addr_t addr)
{
	txdes->txdes3 = cpu_to_le32(addr);
}

static dma_addr_t ftgmac100_txdes_get_dma_addr(struct ftgmac100_txdes *txdes)
{
	return le32_to_cpu(txdes->txdes3);
}

/*
 * txdes2 is not used by hardware. We use it to keep track of socket buffer.
 * Since hardware does not touch it, we can skip cpu_to_le32()/le32_to_cpu().
 */
static void ftgmac100_txdes_set_skb(struct ftgmac100_txdes *txdes,
				    struct sk_buff *skb)
{
	txdes->txdes2 = (unsigned int)skb;
}

static struct sk_buff *ftgmac100_txdes_get_skb(struct ftgmac100_txdes *txdes)
{
	return (struct sk_buff *)txdes->txdes2;
}

/******************************************************************************
 * internal functions (transmit)
 *****************************************************************************/
static int ftgmac100_next_tx_pointer(int pointer)
{
	return (pointer + 1) & (TX_QUEUE_ENTRIES - 1);
}

static void ftgmac100_tx_pointer_advance(struct ftgmac100 *priv)
{
	priv->tx_pointer = ftgmac100_next_tx_pointer(priv->tx_pointer);
}

static void ftgmac100_tx_clean_pointer_advance(struct ftgmac100 *priv)
{
	priv->tx_clean_pointer = ftgmac100_next_tx_pointer(priv->tx_clean_pointer);
}

static struct ftgmac100_txdes *ftgmac100_current_txdes(struct ftgmac100 *priv)
{
	return &priv->descs->txdes[priv->tx_pointer];
}

static struct ftgmac100_txdes *
ftgmac100_current_clean_txdes(struct ftgmac100 *priv)
{
	return &priv->descs->txdes[priv->tx_clean_pointer];
}

static bool ftgmac100_tx_complete_packet(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_txdes *txdes;
	struct sk_buff *skb;
	dma_addr_t map;

	if (priv->tx_pending == 0)
		return false;

	txdes = ftgmac100_current_clean_txdes(priv);

	if (ftgmac100_txdes_owned_by_dma(txdes))
		return false;

	skb = ftgmac100_txdes_get_skb(txdes);
	map = ftgmac100_txdes_get_dma_addr(txdes);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	dma_unmap_single(priv->dev, map, skb_headlen(skb), DMA_TO_DEVICE);

	dev_kfree_skb(skb);

	ftgmac100_txdes_reset(txdes);

	ftgmac100_tx_clean_pointer_advance(priv);

	spin_lock(&priv->tx_lock);
	priv->tx_pending--;
	spin_unlock(&priv->tx_lock);
	netif_wake_queue(netdev);

	return true;
}

static void ftgmac100_tx_complete(struct ftgmac100 *priv)
{
	while (ftgmac100_tx_complete_packet(priv))
		;
}

static int ftgmac100_xmit(struct ftgmac100 *priv, struct sk_buff *skb,
			  dma_addr_t map)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_txdes *txdes;
	unsigned int len = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;

	txdes = ftgmac100_current_txdes(priv);
	ftgmac100_tx_pointer_advance(priv);

	/* setup TX descriptor */
	ftgmac100_txdes_set_skb(txdes, skb);
	ftgmac100_txdes_set_dma_addr(txdes, map);
	ftgmac100_txdes_set_buffer_size(txdes, len);

	ftgmac100_txdes_set_first_segment(txdes);
	ftgmac100_txdes_set_last_segment(txdes);
	ftgmac100_txdes_set_txint(txdes);
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		__be16 protocol = skb->protocol;

		if (protocol == cpu_to_be16(ETH_P_IP)) {
			u8 ip_proto = ip_hdr(skb)->protocol;

			ftgmac100_txdes_set_ipcs(txdes);
			if (ip_proto == IPPROTO_TCP)
				ftgmac100_txdes_set_tcpcs(txdes);
			else if (ip_proto == IPPROTO_UDP)
				ftgmac100_txdes_set_udpcs(txdes);
		}
	}

	spin_lock(&priv->tx_lock);
	priv->tx_pending++;
	if (priv->tx_pending == TX_QUEUE_ENTRIES)
		netif_stop_queue(netdev);

	/* start transmit */
	ftgmac100_txdes_set_dma_own(txdes);
	spin_unlock(&priv->tx_lock);

	ftgmac100_txdma_normal_prio_start_polling(priv);

	return NETDEV_TX_OK;
}

/******************************************************************************
 * internal functions (buffer)
 *****************************************************************************/
static int ftgmac100_alloc_rx_page(struct ftgmac100 *priv,
                                   int rxdes_idx,
                                   gfp_t gfp)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[rxdes_idx];
	struct page *page;
	dma_addr_t map;

	page = alloc_page(gfp);
	if (!page) {
		if (net_ratelimit())
			netdev_err(netdev, "failed to allocate rx page\n");
		return -ENOMEM;
	}

	map = dma_map_page(priv->dev, page, 0, RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, map))) {
		if (net_ratelimit())
			netdev_err(netdev, "failed to map rx page\n");
		__free_page(page);
		return -ENOMEM;
	}

	ftgmac100_rxdes_set_page(priv, rxdes_idx, page);
	ftgmac100_rxdes_set_dma_addr(rxdes, map);
	ftgmac100_rxdes_set_dma_own(rxdes);
	return 0;
}

static void ftgmac100_free_buffers(struct ftgmac100 *priv)
{
	int i;

	for (i = 0; i < RX_QUEUE_ENTRIES; i++) {
		struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		struct page *page = ftgmac100_rxdes_get_page(priv, i);
		dma_addr_t map = ftgmac100_rxdes_get_dma_addr(rxdes);

		if (!page)
			continue;

		dma_unmap_page(priv->dev, map, RX_BUF_SIZE, DMA_FROM_DEVICE);
		__free_page(page);
	}

	for (i = 0; i < TX_QUEUE_ENTRIES; i++) {
		struct ftgmac100_txdes *txdes = &priv->descs->txdes[i];
		struct sk_buff *skb = ftgmac100_txdes_get_skb(txdes);
		dma_addr_t map = ftgmac100_txdes_get_dma_addr(txdes);

		if (!skb)
			continue;

		dma_unmap_single(priv->dev, map, skb_headlen(skb), DMA_TO_DEVICE);
		kfree_skb(skb);
	}

	dma_free_coherent(priv->dev, sizeof(struct ftgmac100_descs),
			  priv->descs, priv->descs_dma_addr);
}

static int ftgmac100_alloc_buffers(struct ftgmac100 *priv)
{
	int i;

	priv->descs = dma_zalloc_coherent(priv->dev,
					  sizeof(struct ftgmac100_descs),
					  &priv->descs_dma_addr, GFP_KERNEL);
	if (!priv->descs)
		return -ENOMEM;

	/* initialize RX ring */
	ftgmac100_rxdes_set_end_of_ring(&priv->descs->rxdes[RX_QUEUE_ENTRIES - 1]);

	for (i = 0; i < RX_QUEUE_ENTRIES; i++) {
		if (ftgmac100_alloc_rx_page(priv, i, GFP_KERNEL))
			goto err;
	}

	/* initialize TX ring */
	ftgmac100_txdes_set_end_of_ring(&priv->descs->txdes[TX_QUEUE_ENTRIES - 1]);
	return 0;

err:
	ftgmac100_free_buffers(priv);
	return -ENOMEM;
}

#ifndef CONFIG_FTGMAC100_NCSI
/******************************************************************************
 * internal functions (mdio)
 *****************************************************************************/
static void ftgmac100_adjust_link(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	struct phy_device *phydev = priv->phydev;
	int ier;

	if (phydev->speed == priv->old_speed)
		return;

	priv->old_speed = phydev->speed;

	ier = ioread32(priv->base + FTGMAC100_OFFSET_IER);

	/* disable all interrupts */
	iowrite32(0, priv->base + FTGMAC100_OFFSET_IER);

	netif_stop_queue(netdev);
	ftgmac100_stop_hw(priv);

	netif_start_queue(netdev);
	ftgmac100_init_hw(priv);
	ftgmac100_start_hw(priv, phydev->speed);

	/* re-enable interrupts */
	iowrite32(ier, priv->base + FTGMAC100_OFFSET_IER);
}


static int ftgmac100_mii_probe(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	struct phy_device *phydev = NULL;
	int i;

	/* search for connect PHY device */
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		struct phy_device *tmp = priv->mii_bus->phy_map[i];

		if (tmp) {
			phydev = tmp;
			break;
		}
	}

	/* now we are supposed to have a proper phydev, to attach to... */
	if (!phydev) {
		netdev_info(netdev, "%s: no PHY found\n", netdev->name);
		return -ENODEV;
	}

	phydev = phy_connect(netdev, dev_name(&phydev->dev),
			     &ftgmac100_adjust_link, PHY_INTERFACE_MODE_GMII);

	if (IS_ERR(phydev)) {
		netdev_err(netdev, "%s: Could not attach to PHY\n", netdev->name);
		return PTR_ERR(phydev);
	}

	priv->phydev = phydev;
	return 0;
}

/******************************************************************************
 * struct mii_bus functions
 *****************************************************************************/
static int ftgmac100_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct net_device *netdev = bus->priv;
	struct ftgmac100 *priv = netdev_priv(netdev);
	unsigned int phycr;
	int i;

	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr) |
		 FTGMAC100_PHYCR_REGAD(regnum) |
		 FTGMAC100_PHYCR_MIIRD;

	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIRD) == 0) {
			int data;

			data = ioread32(priv->base + FTGMAC100_OFFSET_PHYDATA);
			return FTGMAC100_PHYDATA_MIIRDATA(data);
		}

		udelay(100);
	}

	netdev_err(netdev, "mdio read timed out\n");
	return -EIO;
}

static int ftgmac100_mdiobus_write(struct mii_bus *bus, int phy_addr,
				   int regnum, u16 value)
{
	struct net_device *netdev = bus->priv;
	struct ftgmac100 *priv = netdev_priv(netdev);
	unsigned int phycr;
	int data;
	int i;

	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr) |
		 FTGMAC100_PHYCR_REGAD(regnum) |
		 FTGMAC100_PHYCR_MIIWR;

	data = FTGMAC100_PHYDATA_MIIWDATA(value);

	iowrite32(data, priv->base + FTGMAC100_OFFSET_PHYDATA);
	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIWR) == 0)
			return 0;

		udelay(100);
	}

	netdev_err(netdev, "mdio write timed out\n");
	return -EIO;
}
#endif // #ifndef CONFIG_FTGMAC100_NCSI

/******************************************************************************
 * struct ethtool_ops functions
 *****************************************************************************/
static void ftgmac100_get_drvinfo(struct net_device *netdev,
				  struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(&netdev->dev), sizeof(info->bus_info));
}

static int ftgmac100_get_settings(struct net_device *netdev,
				  struct ethtool_cmd *cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	return phy_ethtool_gset(priv->phydev, cmd);
}

static int ftgmac100_set_settings(struct net_device *netdev,
				  struct ethtool_cmd *cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	return phy_ethtool_sset(priv->phydev, cmd);
}

static const struct ethtool_ops ftgmac100_ethtool_ops = {
	.set_settings		= ftgmac100_set_settings,
	.get_settings		= ftgmac100_get_settings,
	.get_drvinfo		= ftgmac100_get_drvinfo,
	.get_link		= ethtool_op_get_link,
};

/******************************************************************************
 * interrupt handler
 *****************************************************************************/
static irqreturn_t ftgmac100_interrupt(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct ftgmac100 *priv = netdev_priv(netdev);

	if (likely(netif_running(netdev))) {
		/* Disable interrupts for polling */
		iowrite32(0, priv->base + FTGMAC100_OFFSET_IER);
		napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct napi_struct functions
 *****************************************************************************/
static int ftgmac100_poll(struct napi_struct *napi, int budget)
{
	struct ftgmac100 *priv = container_of(napi, struct ftgmac100, napi);
	struct net_device *netdev = priv->netdev;
	unsigned int status;
	bool completed = true;
	int rx = 0;

	status = ioread32(priv->base + FTGMAC100_OFFSET_ISR);
	iowrite32(status, priv->base + FTGMAC100_OFFSET_ISR);

	if (status & (FTGMAC100_INT_RPKT_BUF | FTGMAC100_INT_NO_RXBUF)) {
		/*
		 * FTGMAC100_INT_RPKT_BUF:
		 *	RX DMA has received packets into RX buffer successfully
		 *
		 * FTGMAC100_INT_NO_RXBUF:
		 *	RX buffer unavailable
		 */
		bool retry;

		do {
			retry = ftgmac100_rx_packet(priv, &rx);
		} while (retry && rx < budget);

		if (retry && rx == budget)
			completed = false;
	}

	if (status & (FTGMAC100_INT_XPKT_ETH | FTGMAC100_INT_XPKT_LOST)) {
		/*
		 * FTGMAC100_INT_XPKT_ETH:
		 *	packet transmitted to ethernet successfully
		 *
		 * FTGMAC100_INT_XPKT_LOST:
		 *	packet transmitted to ethernet lost due to late
		 *	collision or excessive collision
		 */
		ftgmac100_tx_complete(priv);
	}

	if (status & (FTGMAC100_INT_NO_RXBUF | FTGMAC100_INT_RPKT_LOST |
		      FTGMAC100_INT_AHB_ERR
#if  !defined(CONFIG_FTGMAC100_NCSI) && !defined(CONFIG_PWNEPTUNE)
					 | FTGMAC100_INT_PHYSTS_CHG
#endif
		 )) {
		if (net_ratelimit())
			netdev_info(netdev, "[ISR] = 0x%x: %s%s%s\n", status,
			status & FTGMAC100_INT_NO_RXBUF ? "NO_RXBUF " : "",
			status & FTGMAC100_INT_RPKT_LOST ? "RPKT_LOST " : "",
			status & FTGMAC100_INT_AHB_ERR ? "AHB_ERR " : "");

		if (status & FTGMAC100_INT_NO_RXBUF) {
			/* RX buffer unavailable */
			netdev->stats.rx_over_errors++;
		}

		if (status & FTGMAC100_INT_RPKT_LOST) {
			/* received packet lost due to RX FIFO full */
			netdev->stats.rx_fifo_errors++;
		}
	}

	if (completed) {
		napi_complete(napi);

		/* enable all interrupts */
#ifdef CONFIG_FTGMAC100_NCSI
		iowrite32(INT_MASK_NCSI_ENABLED,
				priv->base + FTGMAC100_OFFSET_IER);
#else
		iowrite32(INT_MASK_ALL_ENABLED,
				priv->base + FTGMAC100_OFFSET_IER);
#endif

	}

	return rx;
}

/******************************************************************************
 * struct net_device_ops functions
 *****************************************************************************/
static int ftgmac100_open(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	int err;
        int idx;

	err = ftgmac100_alloc_buffers(priv);
	if (err) {
		netdev_err(netdev, "failed to allocate buffers\n");
		goto err_alloc;
	}

	err = request_irq(priv->irq,
							ftgmac100_interrupt, 0, netdev->name, netdev);
	if (err) {
		netdev_err(netdev, "failed to request irq %d\n", priv->irq);
		goto err_irq;
	}

	priv->rx_pointer = 0;
	priv->tx_clean_pointer = 0;
	priv->tx_pointer = 0;
	priv->tx_pending = 0;

	err = ftgmac100_reset_hw(priv);
	if (err)
		goto err_hw;

	ftgmac100_init_hw(priv);

#if defined(CONFIG_WEDGE) || defined(CONFIG_WEDGE100) || \
    defined(CONFIG_CMM) || defined(CONFIG_PWNEPTUNE) || \
    defined(CONFIG_MINIPACK) || defined(CONFIG_GALAXY100) || \
    defined(CONFIG_MINILAKETB) || defined(CONFIG_WEDGE400)
	ftgmac100_start_hw(priv, 1000);
#elif defined(CONFIG_FBTP)
	ftgmac100_start_hw(priv, 100);
#elif defined(CONFIG_YAMP)
	ftgmac100_start_hw(priv, 100);
#elif defined(CONFIG_FBTTN)
	ftgmac100_start_hw(priv, 100);
#elif defined(CONFIG_FBY2)  || defined(CONFIG_FBY3_POC) || defined(CONFIG_YOSEMITE)
	ftgmac100_start_hw(priv, 100);
#else
	ftgmac100_start_hw(priv, 10);
#endif

#ifdef CONFIG_FTGMAC100_NCSI
	init_completion(&priv->ncsi_complete);
        idx = get_netdevice_idx(netdev->name);
        ptype_ncsi[idx].dev = netdev;
	dev_add_pack(&ptype_ncsi[idx]);
#else
	phy_config_led(priv->mii_bus);
	phy_start(priv->phydev);
#endif

	napi_enable(&priv->napi);
	netif_start_queue(netdev);

	/* enable all interrupts */
#ifdef CONFIG_FTGMAC100_NCSI
	iowrite32(INT_MASK_NCSI_ENABLED, priv->base + FTGMAC100_OFFSET_IER);
	ncsi_start(netdev);
#else
	iowrite32(INT_MASK_ALL_ENABLED, priv->base + FTGMAC100_OFFSET_IER);
#endif

	return 0;

err_hw:
	free_irq(priv->irq, netdev);
err_irq:
	ftgmac100_free_buffers(priv);
err_alloc:
	return err;
}

static int ftgmac100_stop(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	/* disable all interrupts */
	iowrite32(0, priv->base + FTGMAC100_OFFSET_IER);

	netif_stop_queue(netdev);
	napi_disable(&priv->napi);
#ifndef CONFIG_FTGMAC100_NCSI
	phy_stop(priv->phydev);
#else
	int idx;
	idx = get_netdevice_idx(netdev->name);
	dev_remove_pack(&ptype_ncsi[idx]);
#endif

	ftgmac100_stop_hw(priv);
	free_irq(priv->irq, netdev);
	ftgmac100_free_buffers(priv);

	return 0;
}

static int ftgmac100_hard_start_xmit(struct sk_buff *skb,
				     struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	dma_addr_t map;

	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (net_ratelimit())
			netdev_dbg(netdev, "tx packet too big\n");

		netdev->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	map = dma_map_single(priv->dev, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, map))) {
		/* drop packet */
		if (net_ratelimit())
			netdev_err(netdev, "map socket buffer failed\n");

		netdev->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	return ftgmac100_xmit(priv, skb, map);
}

/* optional */
static int ftgmac100_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	return phy_mii_ioctl(priv->phydev, ifr, cmd);
}

/*
 * This routine will, depending on the values passed to it,
 * either make it accept multicast packets, go into
 * promiscuous mode ( for TCPDUMP and cousins ) or accept
 * a select set of multicast packets
 */
static void ftgmac100_set_multicast(struct net_device *dev)
{
	struct ftgmac100 *priv = netdev_priv(dev);
	u32 maccr;

	maccr = ioread32(priv->base + FTGMAC100_OFFSET_MACCR);

	if (dev->flags & IFF_PROMISC) {
		maccr |= FTGMAC100_MACCR_RX_ALL;
	} else {
		maccr &= ~FTGMAC100_MACCR_RX_ALL;
	}

	if (dev->flags & IFF_ALLMULTI) {
		maccr |= FTGMAC100_MACCR_RX_MULTIPKT;
	} else {
		maccr &= ~FTGMAC100_MACCR_RX_MULTIPKT;
	}

	if (!netdev_mc_empty(dev)) {
		/* the following algorithm is copied from Aspeed ftgmac100 driver */
		struct netdev_hw_addr *ha;
		u32 maht0 = 0;
		u32 maht1 = 0;
		netdev_for_each_mc_addr(ha, dev) {
			u32 crc_val = ether_crc_le(ETH_ALEN, ha->addr);
			int bit = (~(crc_val >> 2)) & 0x3f;
			if (bit >= 32) {
				maht1 |= 1 << (bit - 32);
			} else {
				maht0 |= 1 << bit;
			}
		}
		iowrite32(maht0, priv->base + FTGMAC100_OFFSET_MAHT0);
		iowrite32(maht1, priv->base + FTGMAC100_OFFSET_MAHT1);
		maccr |= FTGMAC100_MACCR_HT_MULTI_EN;
	} else {
		maccr &= ~FTGMAC100_MACCR_HT_MULTI_EN;
	}

	iowrite32(maccr, priv->base + FTGMAC100_OFFSET_MACCR);
}

static const struct net_device_ops ftgmac100_netdev_ops = {
	.ndo_open		= ftgmac100_open,
	.ndo_stop		= ftgmac100_stop,
	.ndo_start_xmit		= ftgmac100_hard_start_xmit,
	.ndo_set_rx_mode	= ftgmac100_set_multicast,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= ftgmac100_do_ioctl,
};

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int ftgmac100_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq;
	struct net_device *netdev;
	struct ftgmac100 *priv;
	int err;
#ifndef CONFIG_FTGMAC100_NCSI
	int i;
#endif

	if (!pdev)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* setup net_device */
	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	netdev->ethtool_ops = &ftgmac100_ethtool_ops;
	netdev->netdev_ops = &ftgmac100_netdev_ops;

  // AST2400 doesn't support HW checksum
#ifndef CONFIG_ARCH_AST2400
	netdev->features = NETIF_F_IP_CSUM | NETIF_F_GRO;
#endif

	platform_set_drvdata(pdev, netdev);

	/* setup private data */
	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->dev = &pdev->dev;

	spin_lock_init(&priv->tx_lock);

	/* initialize NAPI */
	netif_napi_add(netdev, &priv->napi, ftgmac100_poll, 64);

	/* map io memory */
	priv->res = request_mem_region(res->start, resource_size(res),
				       dev_name(&pdev->dev));
	if (!priv->res) {
		dev_err(&pdev->dev, "Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_req_mem;
	}

	priv->base = ioremap(res->start, resource_size(res));
	if (!priv->base) {
		dev_err(&pdev->dev, "Failed to ioremap ethernet registers\n");
		err = -EIO;
		goto err_ioremap;
	}

	priv->irq = irq;

#ifndef CONFIG_FTGMAC100_NCSI
	/* initialize mdio bus */
	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus) {
		err = -EIO;
		goto err_alloc_mdiobus;
	}

	priv->mii_bus->name = "ftgmac100_mdio";
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "ftgmac100_mii");

	priv->mii_bus->priv = netdev;
	priv->mii_bus->read = ftgmac100_mdiobus_read;
	priv->mii_bus->write = ftgmac100_mdiobus_write;
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
		goto err_register_netdev;
	}

	netdev_info(netdev, "irq %d, mapped at %p\n", priv->irq, priv->base);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		eth_hw_addr_random(netdev);
		netdev_info(netdev, "generated random MAC address %pM\n",
			    netdev->dev_addr);
	}

#ifdef CONFIG_FTGMAC100_NCSI
	if (device_create_file(&netdev->dev, &dev_attr_powerup_prep_host_id))
		printk("error: cannot register powerup_prep_host_id attribute.\n");
	else
		printk("dev_attr_powerup_prep_host_id registered\n");

	ncsi_nl_socket_init();
	mutex_init(&ncsi_mutex);

	priv->ncsi_wq = alloc_ordered_workqueue("ncsi_aen_work", 0);
	if (priv->ncsi_wq == NULL) {
		printk("ftgmac: error creating AEN work queue for %s", netdev->name);
	} else {
		printk("ftgmac: aen work queue created for %s", netdev->name);
	}

	INIT_WORK(&priv->work_aen, ftgmac_aen_worker);
	INIT_KFIFO(priv->AEN_buffer);

	/* show the number of used elements */
	AEN_PRINT(KERN_INFO "ftgmac %d fifo len: %u\n", netdev->name, kfifo_len(&priv->AEN_buffer));

#endif
	return 0;

err_register_netdev:
	phy_disconnect(priv->phydev);
#ifndef CONFIG_FTGMAC100_NCSI
err_mii_probe:
	mdiobus_unregister(priv->mii_bus);
err_register_mdiobus:
	mdiobus_free(priv->mii_bus);
err_alloc_mdiobus:
	iounmap(priv->base);
#endif // #ifndef CONFIG_FTGMAC100_NCSI
err_ioremap:
	release_resource(priv->res);
err_req_mem:
	netif_napi_del(&priv->napi);
	free_netdev(netdev);
err_alloc_etherdev:
	return err;
}

static int __exit ftgmac100_remove(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct ftgmac100 *priv;

	netdev = platform_get_drvdata(pdev);

	priv = netdev_priv(netdev);

#ifdef CONFIG_FTGMAC100_NCSI
	device_remove_file(&netdev->dev, &dev_attr_powerup_prep_host_id);
	ncsi_nl_socket_exit();
	cancel_work_sync(&priv->work_aen);
	if (priv->ncsi_wq) {
		flush_workqueue(priv->ncsi_wq);
		destroy_workqueue(priv->ncsi_wq);
	}
#endif

	unregister_netdev(netdev);

	phy_disconnect(priv->phydev);
	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	iounmap(priv->base);
	release_resource(priv->res);

	netif_napi_del(&priv->napi);

	free_netdev(netdev);
	return 0;
}

static struct platform_driver ftgmac100_driver = {
	.probe		= ftgmac100_probe,
	.remove		= __exit_p(ftgmac100_remove),
	.driver		= {
		.name	= DRV_NAME,
	},
};

module_platform_driver(ftgmac100_driver);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("FTGMAC100 driver");
MODULE_LICENSE("GPL");
