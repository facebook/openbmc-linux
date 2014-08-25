#ifndef __LINUX_USB_GADGET_ASPEED_UDC_H
#define __LINUX_USB_GADGET_ASPEED_UDC_H

#define AST_INT_EP_POOL_NAK (1 << 17)
#define AST_INT_EP_POOL_ACK (1 << 16)
#define AST_INT_DEV5_CONTROLLER (1 << 13)
#define AST_INT_DEV4_CONTROLLER (1 << 12)
#define AST_INT_DEV3_CONTROLLER (1 << 11)
#define AST_INT_DEV2_CONTROLLER (1 << 10)
#define AST_INT_DEV1_CONTROLLER (1 << 9)

/* Interrupt control bits */
#define AST_INT_EP1_IN_ACK (1 << 5)
#define AST_INT_EP0_IN_NAK (1 << 4)
#define AST_INT_EP0_IN_ACK (1 << 3)
#define AST_INT_EP0_OUT_NAK (1 << 2)
#define AST_INT_EP0_OUT_ACK (1 << 1)
#define AST_INT_EP0_SETUP_ACK (1 << 0)

/* Interrupt status bits */
#define AST_IRQ_EP_POOL_NAK (1 << 17)
#define AST_IRQ_EP_POOL_ACK (1 << 16)
#define AST_IRQ_EP0_SETUP (1 << 0)
#define AST_IRQ_EP0_OUT_ACK (1 << 1)
#define AST_IRQ_EP0_OUT_NAK (1 << 2)
#define AST_IRQ_EP0_IN_ACK (1 << 3)
#define AST_IRQ_EP0_IN_NAK (1 << 4)
#define AST_IRQ_EP1_IN_ACK (1 << 5)

#define AST_HUB_RESET_DISABLE (1 << 11)

#define AST_HUB_BASE_ADDR 0x1e6a0000
#define AST_HUB_STATUS 0x00
#define AST_HUB_ISR 0x0c
#define AST_HUB_EP_ACK_ISR 0x18
#define AST_HUB_EP_NAK_ISR 0x1C
#define AST_HUB_SOFTRESET_ENABLE 0x20
#define AST_HUB_INTERRUPT_ENABLE 0x08
#define AST_HUB_EP0_STATUS 0x30
#define AST_HUB_EP1_STATUS 0x38
#define AST_HUB_EP0_DMA_ADDR 0x34
#define AST_HUB_ROOT_CONFIG 0x04
#define AST_HUB_EP_ACK_INT_ENABLE 0x10
#define AST_HUB_EP_NAK_INT_ENABLE 0x14

#define AST_HUB_ROOT_SETUP_BUFFER 0x80

#define AST_EP0_OUT_READY (1 << 2)
#define AST_EP0_IN_READY (1 << 1)
#define AST_EP0_STALL (1 << 0)

#define AST_EP_REG_SIZE 0x10
#define AST_EP_BASE(i) (0x200 + (i) * 0x10)

#define AST_EP_CONFIG 0x00
#define AST_EP_DMA_CONTROL 0x04
#define AST_EP_DESC_BASE 0x08
#define AST_EP_DESC_STATUS 0x0C

#define AST_EP_DL_RESET (1 << 2)
#define AST_EP_SINGLE_STAGE (1 << 1)
#define AST_EP_STALL_ENABLED (1 << 12)

#define AST_EP_TYPE_DISABLED 0
#define AST_EP_TYPE_BULK_IN  2
#define AST_EP_TYPE_BULK_OUT 3
#define AST_EP_TYPE_INT_IN   4
#define AST_EP_TYPE_INT_OUT  5
#define AST_EP_TYPE_ISO_IN   6
#define AST_EP_TYPE_ISO_OUT  7

#define AST_EP_ENABLED 1

#define ep_set_type(type, config) \
  (config | (AST_EP_TYPE_##type << 4))

#define ast_hreadl(udcp, reg) \
  ioread32((udcp)->regs + AST_HUB_##reg)

#define ast_hwritel(udcp, reg, value) \
  iowrite32(value, (udcp)->regs + AST_HUB_##reg)

#define ep_hreadl(ep, reg) \
  ioread32((ep)->ep_regs + AST_EP_##reg)

#define ep_hwritel(ep, reg, value) \
  iowrite32(value, (ep)->ep_regs + AST_EP_##reg)


struct aspeed_udc {
  spinlock_t lock;

  void __iomem *regs;
  struct usb_gadget gadget;
  struct usb_gadget_driver *driver;
  struct platform_device *pdev;

  struct dma_pool *pool;

  void* ep0_dma_virt;
  dma_addr_t ep0_dma_phys;

  u16 hub_address;
  int irq;
  unsigned int pullup_on;

  enum {
    EP0_STAGE_SETUP,
    EP0_STAGE_DATA,
    EP0_STAGE_STATUS,
  } ep0_stage;
  /* either USB_DIR_OUT or USB_DIR_IN, valid if it is in data or status stage */
  u8 ep0_dir;
};

struct ast_usb_request {
  struct usb_request req;
  struct list_head queue;
  size_t lastpacket;

  unsigned int in_transit:1;
};

struct ast_ep {
  struct usb_ep ep;
  u8 addr;
  u8 index;
  char epname[7];
  void __iomem *ep_regs;
  size_t maxpacket;

  spinlock_t lock;
  struct list_head queue;
  void *txbuf;
  dma_addr_t txbuf_phys;

  unsigned int dma_busy:1;
  unsigned int to_host:1;
  unsigned int active:1;
  // const struct usb_endpoint_descriptor	*desc;
};


#endif /* __LINUX_USB_GADGET_ASPEED_UDC_H */
