/*
 * aspeed_udc
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/cacheflush.h>

#include "aspeed_udc.h"

#define NUM_ENDPOINTS 14

struct ast_ep *eps;
static const struct usb_ep_ops ast_ep_ops;
static int ast_ep_queue(struct usb_ep* _ep, struct usb_request *_rq, gfp_t gfp_flags);
static void ep_txrx_check_done(struct ast_ep* ep);
static void enable_upstream_port(void);
static void disable_upstream_port(void);
static int ast_udc_pullup(struct usb_gadget* gadget, int on);

static int ast_udc_get_frame(struct usb_gadget* gadget) {
  return -EOPNOTSUPP;
}

static struct usb_gadget_ops ast_gadget_ops = {
  .get_frame = ast_udc_get_frame,
  .pullup = ast_udc_pullup,
};

static void norelease(struct device *dev) {
}

static struct aspeed_udc udc = {
  .gadget = {
    .ops = &ast_gadget_ops,
    .ep_list = LIST_HEAD_INIT(udc.gadget.ep_list),
    .is_dualspeed = 1,
    .name = "aspeed_udc",
    .dev = {
      .bus_id = "gadget",
      .release = norelease,
    },
  },
  .ep0_out_wait = 0,
  .pullup_on = 1,
};

static void enable_hub_interrupts(u32 interrupts) {
  ast_hwritel(&udc, INTERRUPT_ENABLE, ast_hreadl(&udc, INTERRUPT_ENABLE) | interrupts);
}

static void disable_hub_interrupts(u32 interrupts) {
  ast_hwritel(&udc, INTERRUPT_ENABLE, ast_hreadl(&udc, INTERRUPT_ENABLE) &~ interrupts);
}

static int ast_udc_pullup(struct usb_gadget* gadget, int on) {
  unsigned long flags;
  if(on) {
    enable_upstream_port();
  } else {
    disable_upstream_port();
  }
  spin_lock_irqsave(&udc.lock, flags);
  udc.pullup_on = on;
  spin_unlock_irqrestore(&udc.lock, flags);
  return 0;
}

static struct ast_ep ep0_ep = {
  .ep_regs = NULL,
  .ep = {
    .name = "ep0",
    .ops = &ast_ep_ops,
    .maxpacket = 64,
  },
  .queue = LIST_HEAD_INIT(ep0_ep.queue),
  .lock = SPIN_LOCK_UNLOCKED,
  .dma_busy = 0,
};

static void clear_isr(u32 flag) {
  ast_hwritel(&udc, ISR, flag);
}

static void ep0_stall(void) {
  ast_hwritel(&udc, EP0_STATUS, AST_EP0_STALL);
}

static void ep0_rxwait(void) {
  udc.ep0_out_wait = 1;
  ast_hwritel(&udc, EP0_DMA_ADDR, udc.ep0_dma_phys);
  ast_hwritel(&udc, EP0_STATUS, ast_hreadl(&udc, EP0_STATUS) |
      AST_EP0_OUT_READY);
  ep0_ep.dma_busy = 1;
}

static void enable_upstream_port(void) {
  ast_hwritel(&udc, STATUS, ast_hreadl(&udc, STATUS) | 0x5);
}

static void disable_upstream_port(void) {
  ast_hwritel(&udc, STATUS, ast_hreadl(&udc, STATUS) &~ 0x5);
}

// send up to 64 bytes from ep0
static void ep0_tx(u8* data, size_t size) {
  if(size != 0)
    memcpy(udc.ep0_dma_virt, data, size);
  ast_hwritel(&udc, EP0_DMA_ADDR, udc.ep0_dma_phys);
  dma_sync_single_for_device(udc.gadget.dev.parent, udc.ep0_dma_phys, 64, DMA_TO_DEVICE);
  ast_hwritel(&udc, EP0_STATUS, (size << 8));
  ast_hwritel(&udc, EP0_STATUS, (size << 8) | AST_EP0_IN_READY);
  disable_hub_interrupts(AST_INT_EP0_OUT_NAK);
}

struct ast_usb_request empty_rq = {
  .req = {
    .buf = "",
    .length = 0,
  },
  .queue = LIST_HEAD_INIT(empty_rq.queue),
  .in_transit = 0,
};

struct ast_ep* ep_by_addr(int addr) {
  unsigned long flags;
  int i;
  struct ast_ep* ep = NULL;
  for(i = 0; i < NUM_ENDPOINTS && ep == NULL; i++) {
    struct ast_ep* test = &eps[i];
    spin_lock_irqsave(&test->lock, flags);
    if(test->active && test->addr == addr) {
      ep = test;
    }
    spin_unlock_irqrestore(&test->lock, flags);
  }
  return ep;
}

static int handle_ep0_setup(void) {
  struct usb_ctrlrequest crq;
  int ret = 0;
  u16 val, idx, len;
  memcpy_fromio(&crq, udc.regs + AST_HUB_ROOT_SETUP_BUFFER, sizeof(crq));
  val = le16_to_cpu(crq.wValue);
  idx = le16_to_cpu(crq.wIndex);
  len = le16_to_cpu(crq.wLength);
  switch (crq.bRequest) {
    case USB_REQ_SET_ADDRESS:
      udc.hub_address = val;
      //udc.setting_address = 1;
      ast_hwritel(&udc, ROOT_CONFIG, udc.hub_address);
      //respond with an empty IN pkt
      ast_ep_queue(&ep0_ep.ep, &empty_rq.req, GFP_KERNEL);
      return 0;
      break;

    case USB_REQ_CLEAR_FEATURE:
    case USB_REQ_SET_FEATURE:
      if ((crq.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
        break;
      if ((crq.bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
        int epaddr = idx & USB_ENDPOINT_NUMBER_MASK;
        struct ast_ep *ep;
        if(val != 0 || len != 0 || epaddr > 15)
          break;
        ep = ep_by_addr(epaddr);
        if(ep) {
          if(crq.bRequest == USB_REQ_SET_FEATURE) {
            usb_ep_set_halt(&ep->ep);
          } else {
            usb_ep_clear_halt(&ep->ep);
          }
        }
      }
      ast_ep_queue(&ep0_ep.ep, &empty_rq.req, GFP_KERNEL);
    break;
    default:
      ret = udc.driver->setup(&udc.gadget, &crq);
      if (ret < 0) {
        printk("req %02x, %02x; fail: %d\n", crq.bRequestType, crq.bRequest, ret);
        ep0_stall();
      }
      break;
  }
  return 0;
}

static inline struct ast_usb_request *to_ast_req(struct usb_request* rq) {
  return container_of(rq, struct ast_usb_request, req);
}

static inline struct ast_ep *to_ast_ep(struct usb_ep* ep) {
  return container_of(ep, struct ast_ep, ep);
}

static struct usb_request *ast_alloc_request(struct usb_ep *ep, gfp_t gfp_flags) {
  struct ast_usb_request *req;
  (void) ep;
  req = kzalloc(sizeof(*req), gfp_flags);
  INIT_LIST_HEAD(&req->queue);
  if(!req)
    return NULL;

  return &req->req;
}

static void ast_free_request(struct usb_ep *ep, struct usb_request *_req) {
  struct ast_usb_request *req = to_ast_req(_req);
  (void) ep;
  kfree(req);
}

static int ast_ep_enable(struct usb_ep* _ep, const struct usb_endpoint_descriptor *desc) {
  struct ast_ep *ep = to_ast_ep(_ep);
  size_t maxpacket = le16_to_cpu(desc->wMaxPacketSize);
  int eptype = 0;
  if(ep->active) {
    return -EBUSY;
  }
  ep->maxpacket = maxpacket;
  printk("Enabling endpoint %s (%p), maxpacket %d: ", ep->ep.name, ep->ep_regs, ep->maxpacket);
  if (desc->bEndpointAddress & USB_DIR_IN) {
    ep->to_host = 1;
    switch (desc->bmAttributes) {
      case USB_ENDPOINT_XFER_BULK:
        printk("bulk to host\n");
        eptype = AST_EP_TYPE_BULK_IN;
        break;
      case USB_ENDPOINT_XFER_INT:
        printk("int to host\n");
        eptype = AST_EP_TYPE_BULK_IN;
        break;
      case USB_ENDPOINT_XFER_ISOC:
        printk("isoc to host\n");
        eptype = AST_EP_TYPE_ISO_IN;
        break;
    }
  } else {
    ep->to_host = 0;
    switch (desc->bmAttributes) {
      case USB_ENDPOINT_XFER_BULK:
        printk("bulk from host\n");
        eptype = AST_EP_TYPE_BULK_OUT;
        break;
      case USB_ENDPOINT_XFER_INT:
        printk("int from host\n");
        eptype = AST_EP_TYPE_INT_OUT;
        break;
      case USB_ENDPOINT_XFER_ISOC:
        printk("isoc from host\n");
        eptype = AST_EP_TYPE_ISO_OUT;
        break;
    }
  }

  ep_hwritel(ep, DMA_CONTROL, AST_EP_DL_RESET);
  ep_hwritel(ep, DMA_CONTROL, AST_EP_SINGLE_STAGE);

  ep->txbuf = kmalloc(ep->maxpacket, GFP_DMA | GFP_ATOMIC);
  ep->txbuf_phys = 0;

  if (maxpacket == 1024)
    maxpacket = 0;
  ep_hwritel(ep, CONFIG, (maxpacket << 16) | (ep->addr << 8) | (eptype << 4) | AST_EP_ENABLED);

  ep_hwritel(ep, DESC_BASE, ep->txbuf_phys);
  ep_hwritel(ep, DESC_STATUS, 0);

  ast_hwritel(&udc, EP_ACK_INT_ENABLE, 0x7fff);
  ep->active = 1;
  return 0;
}

static void ep_dequeue_locked(struct ast_ep* ep, struct ast_usb_request *req) {
  // Cancel in-progress transfer
  req->req.status = -ECONNRESET;
  if(req->in_transit) {
    req->in_transit = 0;
    ep->dma_busy = 0;
    if(ep->txbuf_phys) {
      dma_unmap_single(udc.gadget.dev.parent,
          ep->txbuf_phys,
          ep->to_host ? req->lastpacket : ep->maxpacket,
          ep->to_host ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
      ep->txbuf_phys = 0;
    }
    ep_hwritel(ep, DESC_STATUS, 0);
    list_del_init(&req->queue);
  }
}

static int ast_ep_dequeue(struct usb_ep* _ep, struct usb_request *_rq) {
  struct ast_usb_request *req = to_ast_req(_rq);
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  spin_lock_irqsave(&ep->lock, flags);
  ep_dequeue_locked(ep, req);
  if(req->req.complete) {
    req->req.complete(&ep->ep, &req->req);
  }
  spin_unlock_irqrestore(&ep->lock, flags);
  return 0;
}

static int ast_ep_disable(struct usb_ep* _ep) {
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  if(!ep->active)
    return 0;
  spin_lock_irqsave(ep->lock, flags);
  while(!list_empty(&ep->queue)) {
    struct ast_usb_request *req;
    req = list_entry(ep->queue.next, struct ast_usb_request, queue);
    ep_dequeue_locked(ep, req);
    if(req->req.complete) {
      spin_unlock_irqrestore(ep->lock, flags);
      req->req.complete(&ep->ep, &req->req);
      spin_lock_irqsave(ep->lock, flags);
    }
  }
  kfree(ep->txbuf);
  ep->txbuf = NULL;
  ep->active = 0;
  ep_hwritel(ep, CONFIG, 0);
  spin_unlock_irqrestore(ep->lock, flags);
  return 0;
}

static void disable_all_endpoints(void) {
  int i;
  for(i = 0; i < NUM_ENDPOINTS; i++) {
    if(eps[i].active) {
      ast_ep_disable(&eps[i].ep);
    }
  }
}

static int ast_ep0_queue_run(void) {
  struct ast_ep *ep = &ep0_ep;
  struct ast_usb_request *req;
  unsigned long flags, status;
  spin_lock_irqsave(&ep->lock, flags);
  status = ast_hreadl(&udc, EP0_STATUS);
  if(!ep->dma_busy &&
      (status & (AST_EP0_IN_READY  |
                 AST_EP0_OUT_READY |
                 AST_EP0_STALL)) == 0) {
    //ep0 isn't busy..
    if(list_empty(&ep->queue)) {
      // nothing
    } else {
      size_t sendable;
      req = list_entry(ep->queue.next, struct ast_usb_request, queue);
      sendable = req->req.length - req->req.actual;
      if (sendable > 64) {
        sendable = 64;
      }
      ep0_tx(req->req.buf + req->req.actual, sendable);
      req->req.actual += sendable;
      req->in_transit = 1;
      ep->dma_busy = 1;
    }
  }
  spin_unlock_irqrestore(&ep->lock, flags);
  return 0;
}

static void ep_txrx(struct ast_ep* ep, void* buf, size_t len) {
  if(ep->to_host) {
    memcpy(ep->txbuf, buf, len);
    ep->txbuf_phys = dma_map_single(udc.gadget.dev.parent,
                                    ep->txbuf,
                                    len,
                                    DMA_TO_DEVICE);
  } else {
    ep->txbuf_phys = dma_map_single(udc.gadget.dev.parent,
                                    ep->txbuf,
                                    ep->maxpacket,
                                    DMA_FROM_DEVICE);
  }
  ep_hwritel(ep, DESC_BASE, ep->txbuf_phys);
  ep_hwritel(ep, DESC_STATUS, len << 16);
  ep_hwritel(ep, DESC_STATUS, (len << 16) | 1);
}

static void ep_txrx_check_done(struct ast_ep* ep) {
  struct ast_usb_request *req;
  unsigned long flags;
  u32 status;
  spin_lock_irqsave(&ep->lock, flags);
  status = ep_hreadl(ep, DESC_STATUS);
  // if txrx complete;
  if(!(status & 0xff) &&
     !list_empty(&ep->queue)) {
    req = list_entry(ep->queue.next, struct ast_usb_request, queue);
    if(!req->in_transit) {
      spin_unlock_irqrestore(&ep->lock, flags);
      return;
    }
    //head rq completed
    req->in_transit = 0;
    ep->dma_busy = 0;
    if(!ep->to_host) {
      req->lastpacket = (status >> 16) & 0x3ff;
      __cpuc_flush_kern_all();
      dma_unmap_single(udc.gadget.dev.parent, ep->txbuf_phys, ep->maxpacket, DMA_FROM_DEVICE);
      memcpy(req->req.buf + req->req.actual, ep->txbuf, req->lastpacket);
      //print_hex_dump(KERN_DEBUG, "epXrx: ", DUMP_PREFIX_OFFSET, 16, 1, ep->txbuf, req->lastpacket, 0);
      ep->txbuf_phys = 0;
    } else {
      dma_unmap_single(udc.gadget.dev.parent, ep->txbuf_phys, req->lastpacket, DMA_TO_DEVICE);
      ep->txbuf_phys = 0;
    }
    req->req.actual += req->lastpacket;
    if(req->req.actual == req->req.length ||
        req->lastpacket < ep->maxpacket) {
      list_del_init(&req->queue);
      spin_unlock_irqrestore(&ep->lock, flags);
      //printk("rq done ep%d\n", ep->addr);
      req->req.status = 0;
      if(req->req.complete) {
        req->req.complete(&ep->ep, &req->req);
      }
    } else {
      //printk("rq partial ep%d %d/%d\n", ep->addr, req->req.actual, req->req.length);
      spin_unlock_irqrestore(&ep->lock, flags);
    }
  }
}

static int ast_ep_queue_run(struct ast_ep* ep) {
  struct ast_usb_request *req;
  unsigned long flags;
  spin_lock_irqsave(&ep->lock, flags);
  if(ep->active && !ep->dma_busy) {
    //ep isn't busy..
    if(list_empty(&ep->queue)) {
      // nothing
    } else {
      req = list_entry(ep->queue.next, struct ast_usb_request, queue);
      req->lastpacket = req->req.length - req->req.actual;
      if (req->lastpacket > ep->maxpacket) {
        req->lastpacket = ep->maxpacket;
      }
      //printk("ep%d%sx:%d-%d/%d\n",
      //    ep->addr,
      //    ep->to_host ? "t" : "r",
      //    req->req.actual,
      //    req->req.actual + req->lastpacket,
      //    req->req.length);
      ep_txrx(ep, req->req.buf + req->req.actual, req->lastpacket);
      req->in_transit = 1;
      ep->dma_busy = 1;
    }
  }
  spin_unlock_irqrestore(&ep->lock, flags);
  return 0;
}

static void run_all_ep_queues(unsigned long data) {
  int i;
  for(i = 0; i < NUM_ENDPOINTS; i++) {
    if(eps[i].active) {
      ep_txrx_check_done(&eps[i]);
      ast_ep_queue_run(&eps[i]);
    }
  }
}

DECLARE_TASKLET(check_ep_queues, run_all_ep_queues, 0);

static int ast_ep_queue(struct usb_ep* _ep, struct usb_request *_rq, gfp_t gfp_flags) {
  struct ast_usb_request *req = to_ast_req(_rq);
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  _rq->status = -EINPROGRESS;
  spin_lock_irqsave(&ep->lock, flags);
  list_add_tail(&req->queue, &ep->queue);
  req->req.actual = 0;
  spin_unlock_irqrestore(&ep->lock, flags);
  if(ep == &ep0_ep) {
    ast_ep0_queue_run();
  } else {
    ast_ep_queue_run(ep);
  }

  return 0;
}

static int ast_ep_set_halt(struct usb_ep* _ep, int value) {
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  spin_lock_irqsave(&ep->lock, flags);
  if(value) {
    ep_hwritel(ep, CONFIG, ep_hreadl(ep, CONFIG) | AST_EP_STALL_ENABLED);
  } else {
    ep_hwritel(ep, CONFIG, ep_hreadl(ep, CONFIG) &~ AST_EP_STALL_ENABLED);
  }
  spin_unlock_irqrestore(&ep->lock, flags);
  return 0;
}

static const struct usb_ep_ops ast_ep_ops = {
  .enable = ast_ep_enable,
  .disable = ast_ep_disable,
  .queue = ast_ep_queue,
  .dequeue = ast_ep_dequeue,
  .set_halt = ast_ep_set_halt,
  .alloc_request = ast_alloc_request,
  .free_request = ast_free_request,
};

static void ep0_txrx_check_done(void) {
  struct ast_usb_request *req;
  struct ast_ep *ep = &ep0_ep;
  unsigned long flags, status, flagsudc;
  spin_lock_irqsave(&ep->lock, flags);
  spin_lock_irqsave(&udc.lock, flagsudc);
  status = ast_hreadl(&udc, EP0_STATUS);

  if(ep->dma_busy &&
      udc.ep0_out_wait &&
      (status & (AST_EP0_IN_READY  |
                 AST_EP0_OUT_READY |
                 AST_EP0_STALL)) == 0) {
    udc.ep0_out_wait = 0;
    ep->dma_busy = 0;
    enable_hub_interrupts(AST_INT_EP0_OUT_NAK);
  } else if(ep->dma_busy &&
      !list_empty(&ep->queue) &&
      (status & (AST_EP0_IN_READY  |
                 AST_EP0_OUT_READY |
                 AST_EP0_STALL)) == 0) {
    req = container_of(ep->queue.next, struct ast_usb_request, queue);
    //head rq completed
    req->in_transit = 0;
    ep->dma_busy = 0;
    enable_hub_interrupts(AST_INT_EP0_OUT_NAK);
    if(req->req.actual == req->req.length) {
      list_del_init(&req->queue);
      spin_unlock_irqrestore(&udc.lock, flagsudc);
      spin_unlock_irqrestore(&ep->lock, flags);
      req->req.status = 0;
      if(req->req.complete) {
        req->req.complete(&ep->ep, &req->req);
      }
      return;
    }
  }
  spin_unlock_irqrestore(&udc.lock, flagsudc);
  spin_unlock_irqrestore(&ep->lock, flags);
}

static irqreturn_t ast_vhub_udc_irq(int irq, void* devid) {
  irqreturn_t status = IRQ_NONE;
  u32 istatus, enabled;
  unsigned long flagsep0;
  (void) devid;
  istatus = ast_hreadl(&udc, ISR);
  enabled = ast_hreadl(&udc, INTERRUPT_ENABLE);
  if(istatus & AST_IRQ_EP0_OUT_ACK) {
    clear_isr(AST_IRQ_EP0_OUT_ACK);
    ast_ep0_queue_run();
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP0_OUT_NAK &&
     enabled & AST_INT_EP0_OUT_NAK) {
    spin_lock_irqsave(&ep0_ep.lock, flagsep0);
    if(!ep0_ep.dma_busy) {
      ep0_rxwait();
    } else {
      disable_hub_interrupts(AST_INT_EP0_OUT_NAK);
    }
    clear_isr(AST_IRQ_EP0_OUT_NAK);
    spin_unlock_irqrestore(&ep0_ep.lock, flagsep0);
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP0_SETUP) {
    clear_isr(AST_IRQ_EP0_SETUP);
    handle_ep0_setup();
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP0_IN_ACK) {
    ep0_txrx_check_done();
    clear_isr(AST_IRQ_EP0_IN_ACK);
    ast_ep0_queue_run();
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP_POOL_ACK) {
    u32 acked = ast_hreadl(&udc, EP_ACK_ISR);
    ast_hwritel(&udc, EP_ACK_ISR, acked);
    clear_isr(AST_IRQ_EP_POOL_ACK);
    status = IRQ_HANDLED;
    tasklet_schedule(&check_ep_queues);
  }

  if(istatus & AST_IRQ_EP_POOL_NAK) {
    printk("Got NAK on the EP pool");
    clear_isr(AST_IRQ_EP_POOL_NAK);
    status = IRQ_HANDLED;
  }

  ep0_txrx_check_done();
  if(status != IRQ_HANDLED) {
    printk("vhub: unhandled interrupts! ISR: %08x\n", istatus);
  }
  return status;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver) {
  int err = 0;
  unsigned long flags;
  printk("gadget register driver: %s\n", driver->driver.name);
  if(!udc.pdev)
    return -ENODEV;

  spin_lock_irqsave(&udc.lock, flags);
  if (udc.driver) {
    err = -EBUSY;
    goto err;
  }
  udc.driver = driver;
  udc.gadget.dev.driver = &driver->driver;
err:
  spin_unlock_irqrestore(&udc.lock, flags);
  if(err == 0) {
    //ok so far
    err = driver->bind(&udc.gadget);
  }
  if(err == 0) {
    if(udc.pullup_on) {
      enable_upstream_port();
    }
    printk("vhub: driver registered, port on!\n");
  } else {
    printk("vhub: driver failed to register: %d\n", err);
    spin_lock_irqsave(&udc.lock, flags);
    udc.driver = NULL;
    udc.gadget.dev.driver = NULL;
    spin_unlock_irqrestore(&udc.lock, flags);
  }
  return err;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver) {
  int err = 0;
  unsigned long flags;
  if(!udc.pdev)
    return -ENODEV;
  if(driver != udc.driver || !driver->unbind) {
    err = -EINVAL;
    goto cleanup;
  }

  // disappear from the host
  ast_udc_pullup(&udc.gadget, 0);
  if(udc.driver->disconnect) {
    udc.driver->disconnect(&udc.gadget);
  }

  driver->unbind(&udc.gadget);
  // turn off the EPs
  spin_lock_irqsave(&udc.lock, flags);
  disable_all_endpoints();

  udc.gadget.dev.driver = NULL;
  udc.driver = NULL;

cleanup:
  spin_unlock_irqrestore(&udc.lock, flags);
  return err;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static int __init ast_vhub_udc_probe(struct platform_device *pdev) {
  struct resource *res;
  int err = -ENODEV;
  int irq;
  int i;

  udc.regs = NULL;
  irq = platform_get_irq(pdev, 0);
  if(irq < 0)
    return irq;
  udc.irq = irq;

  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

  spin_lock_init(&udc.lock);
  err = -ENOMEM;
  udc.regs = ioremap(res->start, res->end - res->start + 1);
  if(!udc.regs) {
    dev_err(&pdev->dev, "Couldn't map I/O memory!\n");
    goto error;
  }

  dev_info(&pdev->dev, "aspeed_udc at 0x%08lx mapped at %p\n",
      (unsigned long)res->start, udc.regs);
  platform_set_drvdata(pdev, &udc);
  device_initialize(&udc.gadget.dev);

  udc.gadget.dev.parent = &pdev->dev;
  udc.gadget.dev.dma_mask = pdev->dev.dma_mask;
  ast_hwritel(&udc, ISR, 0x1ffff);
  err = request_irq(irq, ast_vhub_udc_irq, 0, "aspeed_udc", &udc);

  if(err) {
    dev_err(&pdev->dev, "failed to get irq %d: %d\n", irq, err);
    goto error;
  }
  udc.ep0_dma_virt = kmalloc(64, GFP_KERNEL | GFP_DMA);
  if (!udc.ep0_dma_virt) {
    printk("vhub fatal: Couldn't get DMA memory!\n");
    goto error;
  } else {
    udc.ep0_dma_phys = dma_map_single(&pdev->dev, udc.ep0_dma_virt, 64, DMA_BIDIRECTIONAL);
  }

  printk("virthub init...\n");
  ast_hwritel(&udc, SOFTRESET_ENABLE, 0x33f);
  ast_hwritel(&udc, SOFTRESET_ENABLE, 0x0);
  ast_hwritel(&udc, ISR, 0x1ffff);
  ast_hwritel(&udc, EP_ACK_ISR, 0xffffffff);
  ast_hwritel(&udc, EP_NAK_ISR, 0xffffffff);
  enable_hub_interrupts(
      AST_INT_EP_POOL_ACK |
      AST_INT_EP0_IN_ACK |
      AST_INT_EP0_OUT_NAK |
      AST_INT_EP0_OUT_ACK |
      AST_INT_EP0_SETUP_ACK);

  ast_hwritel(&udc, EP1_STATUS, 0x1);
  ast_hwritel(&udc, STATUS, AST_HUB_RESET_DISABLE);

  udc.pdev = pdev;

  INIT_LIST_HEAD(&ep0_ep.ep.ep_list);
  udc.gadget.ep0 = &ep0_ep.ep;

  eps = kzalloc(sizeof(struct ast_ep) * 14, GFP_KERNEL);
  if(!eps) {
    goto error;
  }

  for (i = 0; i < NUM_ENDPOINTS; i++) {
    struct ast_ep* ep = &eps[i];
    ep->ep_regs = udc.regs + 0x200 + (i * 0x10);
    INIT_LIST_HEAD(&ep->queue);
    ep->ep.ops = &ast_ep_ops;
    ep->index = i;
    // i+2, Can't use EP1 as the 'virtual hub' has it built in
    // when using the root device.
    ep->addr = i+2;
    snprintf(ep->epname, 7, "ep%d", ep->addr);
    ep->ep.name = ep->epname;
    ep->ep.maxpacket = 1024;
    ep->lock = SPIN_LOCK_UNLOCKED;
    list_add_tail(&ep->ep.ep_list, &udc.gadget.ep_list);
  }
  err = device_add(&udc.gadget.dev);
  if(err) {
    dev_dbg(&pdev->dev, "Could not add gadget: %d\n", err);
    goto error;
  }
  return 0;
error:
  if(udc.regs)
    iounmap(udc.regs);
  platform_set_drvdata(pdev, NULL);
	return err;
}

static int ast_vhub_udc_remove(struct platform_device *pdev) {
  if(udc.regs)
    iounmap(udc.regs);
  if(udc.irq)
    free_irq(udc.irq, &udc);
  platform_set_drvdata(pdev, NULL);
  return 0;
}

static struct platform_driver ast_vhub_udc_driver = {
  .probe = ast_vhub_udc_probe,
  .remove = ast_vhub_udc_remove,
  .driver = {
    .name = "aspeed_udc",
    .owner = THIS_MODULE,
  },
};

static int __init ast_udc_init(void) {
  return platform_driver_probe(&ast_vhub_udc_driver, ast_vhub_udc_probe);
}
module_init(ast_udc_init);

static void __exit ast_udc_exit(void) {
  platform_driver_unregister(&ast_vhub_udc_driver);
}
module_exit(ast_udc_exit);

MODULE_DESCRIPTION("AST2400/1250 USB UDC Driver");
MODULE_AUTHOR("");
MODULE_LICENSE("");
MODULE_ALIAS("platform:aspeed_udc");
