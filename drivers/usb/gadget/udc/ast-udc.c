/*
 * aspeed_udc - Driver for Aspeed virtual hub (usb gadget)
 *
 * Copyright 2014-present Facebook. All Rights Reserved.
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

//#define DEBUG
//#define VERBOSE

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
#include <asm/cacheflush.h>

#include <asm/div64.h>      
#include <asm/bitops.h>    
#include <asm/atomic.h>    

#include "ast-udc.h"

#define NUM_ENDPOINTS 14
#define AST_UDC_EP0_MAXPACKET 64

#define AST_UDC_EP_MAXPACKET 512

#ifdef VERBOSE
#define ep_vdebug pr_debug
#else
#define ep_vdebug
#endif

struct ast_ep *eps;
static const struct usb_ep_ops ast_ep_ops;
static int ast_ep_queue(struct usb_ep* _ep, struct usb_request *_rq, gfp_t gfp_flags);
static void ep_dequeue_all(struct ast_ep* ep, int status);
static void ep_txrx_check_done(struct ast_ep* ep);
static void enable_upstream_port(void);
static void disable_upstream_port(void);
static int ast_udc_pullup(struct usb_gadget* gadget, int on);

static inline const char* ast_udc_dir2str(u8 dir) {
  return (dir & USB_DIR_IN) ? "IN" : "OUT";
}

static int ast_udc_get_frame(struct usb_gadget* gadget) {
  return -EOPNOTSUPP;
}

// Added for kernel 4.1
static int ast_udc_start(struct usb_gadget *gadget,
    struct usb_gadget_driver *driver);

static int ast_udc_stop(struct usb_gadget *gadget);

static struct usb_gadget_ops ast_gadget_ops = {
  .get_frame = ast_udc_get_frame,
  .pullup = ast_udc_pullup,
  .udc_start = ast_udc_start,   
  .udc_stop = ast_udc_stop,
};

static void norelease(struct device *dev) {
}

static struct aspeed_udc udc = {
  .gadget = {
    .ops = &ast_gadget_ops,
    .ep_list = LIST_HEAD_INIT(udc.gadget.ep_list),
    .max_speed = USB_SPEED_HIGH,
    .name = "ast-vhub",
    .dev = {
      .release = norelease,
    },
  },
  .pullup_on = 1,
  .ep0_stage = EP0_STAGE_SETUP,
  .ep0_dir = USB_DIR_IN,
};

static inline struct ast_usb_request *to_ast_req(struct usb_request* rq)
{
  return container_of(rq, struct ast_usb_request, req);
}

static inline struct ast_ep *to_ast_ep(struct usb_ep* ep)
{
  return container_of(ep, struct ast_ep, ep);
}

static void* ast_alloc_dma_memory(int sz, enum dma_data_direction dir,
                                  dma_addr_t *phys)
{
  void* mem;
  mem = kmalloc(sz, GFP_KERNEL | GFP_DMA);
  if (!mem) {
    return NULL;
  }

  *phys = dma_map_single(udc.gadget.dev.parent, mem, sz, dir);
  return mem;
}

static void ast_free_dma_memory(int sz, enum dma_data_direction dir,
                                void *mem, dma_addr_t phys)
{
  if (phys) {
    dma_unmap_single(udc.gadget.dev.parent, phys, sz, dir);
  }
  if (mem) {
    kfree(mem);
  }
}

static int ast_udc_pullup(struct usb_gadget* gadget, int on) {
  if(on) {
    enable_upstream_port();
  } else {
    disable_upstream_port();
  }
  udc.pullup_on = on;
  return 0;
}

static struct ast_ep ep0_ep = {
  .ep_regs = NULL,
  .ep = {
    .name = "ep0",
    .ops = &ast_ep_ops,
    .maxpacket = AST_UDC_EP0_MAXPACKET,
  },
  .queue = LIST_HEAD_INIT(ep0_ep.queue),
  .dma_busy = 0,
};

//static DEFINE_SPINLOCK(ep0_ep.lock);

static void clear_isr(u32 flag) {
  ast_hwritel(&udc, ISR, flag);
}

static void ep0_stall_ready(void) {
  ast_hwritel(&udc, EP0_STATUS, AST_EP0_STALL);
}

static void ep0_out_ready(void) {
  ast_hwritel(&udc, EP0_STATUS, AST_EP0_OUT_READY);
}

static void ep0_in_ready(size_t size) {
  ast_hwritel(&udc, EP0_STATUS, (size << 8));
  ast_hwritel(&udc, EP0_STATUS, (size << 8) | AST_EP0_IN_READY);
}

static void enable_upstream_port(void) {
  ast_hwritel(&udc, STATUS, ast_hreadl(&udc, STATUS) | 0x5);
}

static void disable_upstream_port(void) {
  ast_hwritel(&udc, STATUS, ast_hreadl(&udc, STATUS) &~ 0x5);
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

static inline void ep0_init_stage(void)
{
  /* expect SETUP after */
  udc.ep0_stage = EP0_STAGE_SETUP;
  udc.ep0_dir = USB_DIR_IN;
}

static void ep0_stall(void) {
  pr_debug("Prepare for EP0 stall, Reset EP0 stage to SETUP\n");
  /* reply stall on next pkt */
  ep0_stall_ready();
  ep0_init_stage();
}

static void ep0_next_stage(int has_data_phase, u8 data_dir) {
#ifdef VERBOSE
  int prev_stage = udc.ep0_stage;
  int prev_dir = udc.ep0_dir;
#endif

  switch (udc.ep0_stage) {
  case EP0_STAGE_SETUP:
    if (has_data_phase) {
      udc.ep0_stage = EP0_STAGE_DATA;
      udc.ep0_dir = (data_dir == USB_DIR_IN) ? USB_DIR_IN : USB_DIR_OUT;
    } else {
      udc.ep0_stage = EP0_STAGE_STATUS;
      udc.ep0_dir = USB_DIR_IN;
    }
    break;
  case EP0_STAGE_DATA:
    udc.ep0_stage = EP0_STAGE_STATUS;
    udc.ep0_dir = (udc.ep0_dir == USB_DIR_IN) ? USB_DIR_OUT : USB_DIR_IN;
    break;
  case EP0_STAGE_STATUS:
    udc.ep0_stage = EP0_STAGE_SETUP;
    udc.ep0_dir = USB_DIR_IN;
    break;
  default:
    pr_err("Wrong EP0 stage %d\n", udc.ep0_stage);
    break;
  }

#ifdef VERBOSE
  ep_vdebug("EP0 stage is changed from %d (%s) to %d (%s)\n",
            prev_stage, ast_udc_dir2str(prev_dir),
            udc.ep0_stage, ast_udc_dir2str(udc.ep0_dir));
#endif
}

static void ep0_queue_run(void) {
  struct ast_ep *ep = &ep0_ep;

  /* if ep0 is still doing DMA or no request pending, nothing to do */
  if (ep->dma_busy || list_empty(&ep->queue)) {
    return;
  }

  if (udc.ep0_dir == USB_DIR_OUT) {
    /* just tell HW we are ready for OUT */
    ep0_out_ready();
  } else {
    size_t sendable;
    struct ast_usb_request *req;

    req = list_entry(ep->queue.next, struct ast_usb_request, queue);
    sendable = req->req.length - req->req.actual;
    if (sendable > AST_UDC_EP0_MAXPACKET) {
      sendable = AST_UDC_EP0_MAXPACKET;
    }
    if (sendable) {
      memcpy(udc.ep0_dma_virt, req->req.buf + req->req.actual, sendable);
    }
    dma_sync_single_for_device(udc.gadget.dev.parent, udc.ep0_dma_phys,
                               AST_UDC_EP0_MAXPACKET, DMA_TO_DEVICE);
    ep0_in_ready(sendable);
    req->lastpacket = sendable;
  }
  ep->dma_busy = 1;
}

static int ep0_enqueue(struct usb_ep* _ep, struct usb_request *_rq,
                       gfp_t gfp_flags) {
  struct ast_usb_request *req = to_ast_req(_rq);
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  int rc = 0;

  _rq->status = -EINPROGRESS;
  req->req.actual = 0;

  ep_vdebug("EP0 enqueue %d bytes at stage %d, %s\n",
            req->req.length, udc.ep0_stage, ast_udc_dir2str(udc.ep0_dir));

  spin_lock_irqsave(&ep->lock, flags);

  /* shall only happen when in data or status stage */
  if (udc.ep0_stage != EP0_STAGE_DATA && udc.ep0_stage != EP0_STAGE_STATUS) {
    pr_err("EP0 enqueue happens at wrong stage %d\n", udc.ep0_stage);
    rc = -EINPROGRESS;
    goto out;
  }

  /* ep0 shall only have one pending request a time */
  if (!list_empty(&ep->queue)) {
    pr_err("EP0 enqueue happens with an existing entry on queue\n");
  } else {
    list_add_tail(&req->queue, &ep->queue);
    ep0_queue_run();
  }

 out:
  if (rc < 0) {
    ep0_stall();
  }
  spin_unlock_irqrestore(&ep->lock, flags);
  return rc;
}

static void ep0_cancel_current(void)
{
  struct ast_ep *ep = &ep0_ep;
  unsigned long flags;
  ep_dequeue_all(ep, -ECONNRESET);
  spin_lock_irqsave(&ep->lock, flags);
  ep0_init_stage();
  spin_unlock_irqrestore(&ep->lock, flags);
}

static int ep0_handle_setup(void) {
  struct ast_ep *ep = &ep0_ep;
  struct usb_ctrlrequest crq;
  int rc = 0;
  u16 val, idx, len;
  unsigned long flags;

  /* make sure we are expecting setup packet */
  if (udc.ep0_stage != EP0_STAGE_SETUP) {
    /*
     * with g_cdc, we are seeing this message pretty often. could be an
     * issue on g_cdc. make the log message as debug now
     */
    pr_debug("Received SETUP pkt on wrong stage %d. Cancelling "
             "the current SETUP\n", udc.ep0_stage);
    
    ep0_cancel_current();
  }

  memcpy_fromio(&crq, udc.regs + AST_HUB_ROOT_SETUP_BUFFER, sizeof(crq));
  val = le16_to_cpu(crq.wValue);
  idx = le16_to_cpu(crq.wIndex);
  len = le16_to_cpu(crq.wLength);

  ep_vdebug("request=%d, reqType=0x%x val=0x%x idx=%d len=%d %s\n",
            crq.bRequest, crq.bRequestType, val, idx, len,
            ast_udc_dir2str(crq.bRequestType));
  
  spin_lock_irqsave(&ep->lock, flags);
  ep0_next_stage(
      len, (crq.bRequestType & USB_DIR_IN) ? USB_DIR_IN : USB_DIR_OUT);
  spin_unlock_irqrestore(&ep->lock, flags);

  switch (crq.bRequest) {
    case USB_REQ_SET_ADDRESS:
      udc.hub_address = val;
      ast_hwritel(&udc, ROOT_CONFIG, udc.hub_address);
      // reply with an empty pkt for STATUS
      ep0_enqueue(&ep0_ep.ep, &empty_rq.req, GFP_KERNEL);
      break;

    case USB_REQ_CLEAR_FEATURE:
    case USB_REQ_SET_FEATURE:
      if ((crq.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
        break;
      if ((crq.bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
        int epaddr = idx & USB_ENDPOINT_NUMBER_MASK;
        struct ast_ep *ep2;
        if(val != 0 || len != 0 || epaddr > 15) {
          ep0_stall();
          break;
        }
        ep2 = ep_by_addr(epaddr);
        if (ep2) {
          if(crq.bRequest == USB_REQ_SET_FEATURE) {
            usb_ep_set_halt(&ep2->ep);
          } else {
            usb_ep_clear_halt(&ep2->ep);
          }
        }
      }
      // reply with an empty pkt for STATUS
      ep0_enqueue(&ep0_ep.ep, &empty_rq.req, GFP_KERNEL);
      break;

    default:
      rc = udc.driver->setup(&udc.gadget, &crq);
      if (rc < 0) {
        printk("req %02x, %02x; fail: %d\n", crq.bRequestType,
               crq.bRequest, rc);
        ep0_stall();
      }
      break;
  }

  return rc;
}

static void ep0_handle_ack(void) {
  struct ast_usb_request *req;
  struct ast_ep *ep = &ep0_ep;
  unsigned long status;
  unsigned long flags;
  int done = 0;

  spin_lock_irqsave(&ep->lock, flags);

  if (!ep->dma_busy) {
    goto out;
  }

  status = ast_hreadl(&udc, EP0_STATUS);

  ep_vdebug("Check done with status 0x%lx: stage=%d %s\n", status,
            udc.ep0_stage, ast_udc_dir2str((udc.ep0_dir)));

  if (list_empty(&ep->queue)) {
    /* we requested DMA but no request */
    ep->dma_busy = 0;
    pr_debug("Empty request queue with ep0 status: 0x%lx\n", status);
    goto out;
  }

  if ((udc.ep0_dir == USB_DIR_IN && (!(status & AST_EP0_IN_READY)))
      || (udc.ep0_dir == USB_DIR_OUT && (!(status & AST_EP0_OUT_READY)))) {
    /* something is ready */
    ep->dma_busy = 0;
    req = container_of(ep->queue.next, struct ast_usb_request, queue);
    req->in_transit = 0;
    if (udc.ep0_dir == USB_DIR_OUT) {
      req->lastpacket = (status >> 16) & 0x3F;
      __cpuc_flush_kern_all();
      if (req->lastpacket) {
        memcpy(req->req.buf + req->req.actual, udc.ep0_dma_virt,
               req->lastpacket);
      }
    }
    req->req.actual += req->lastpacket;
    ep_vdebug("EP0 req actual=%d length=%d lastpacket=%d\n",
              req->req.actual, req->req.length, req->lastpacket);
    if (req->req.actual >= req->req.length
        || req->lastpacket < AST_UDC_EP0_MAXPACKET) {
      list_del_init(&req->queue);
      done = 1;
      ep0_next_stage(0, 0);
    } else {
      /* the request is not done yet, restart the queue */
      ep0_queue_run();
    }
  }

 out:

  spin_unlock_irqrestore(&ep->lock, flags);

  if (done) {
    req->req.status = 0;
    if (req->req.complete) {
      req->req.complete(&ep->ep, &req->req);
    }
  }
}

static void ep0_handle_status(void) {
  struct ast_ep *ep = &ep0_ep;
  unsigned long flags;

  spin_lock_irqsave(&ep->lock, flags);

  /*
   * If we expect a STATUS from host (OUT), enqueue a reqeust to get it going.
   * For STATUS to host (IN), the SETUP packet handler needs to enqueue a
   * STATUS message, therefore, no need to handle it here.
   */
  if (udc.ep0_stage == EP0_STAGE_STATUS && udc.ep0_dir == USB_DIR_OUT) {
    // request with an empty pkt for STATUS from host
    ep0_enqueue(&ep0_ep.ep, &empty_rq.req, GFP_KERNEL);
  }

  spin_unlock_irqrestore(&ep->lock, flags);
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
    pr_err("Enable an already enabled ep: %s\n", ep->ep.name);
    return -EBUSY;
  }
  ep->ep.maxpacket = maxpacket;
  pr_debug("Enabling endpoint %s (%p), maxpacket %d: ",
           ep->ep.name, ep->ep_regs, ep->ep.maxpacket);
  if (desc->bEndpointAddress & USB_DIR_IN) {
    ep->to_host = 1;
    switch (desc->bmAttributes) {
      case USB_ENDPOINT_XFER_BULK:
        pr_debug("bulk to host\n");
        eptype = AST_EP_TYPE_BULK_IN;
        break;
      case USB_ENDPOINT_XFER_INT:
        pr_debug("int to host\n");
        eptype = AST_EP_TYPE_BULK_IN;
        break;
      case USB_ENDPOINT_XFER_ISOC:
        pr_debug("isoc to host\n");
        eptype = AST_EP_TYPE_ISO_IN;
        break;
    }
  } else {
    ep->to_host = 0;
    switch (desc->bmAttributes) {
      case USB_ENDPOINT_XFER_BULK:
        pr_debug("bulk from host\n");
        eptype = AST_EP_TYPE_BULK_OUT;
        break;
      case USB_ENDPOINT_XFER_INT:
        pr_debug("int from host\n");
        eptype = AST_EP_TYPE_INT_OUT;
        break;
      case USB_ENDPOINT_XFER_ISOC:
        pr_debug("isoc from host\n");
        eptype = AST_EP_TYPE_ISO_OUT;
        break;
    }
  }

  ep_hwritel(ep, DMA_CONTROL, AST_EP_DL_RESET);
  ep_hwritel(ep, DMA_CONTROL, AST_EP_SINGLE_STAGE);

  ep->txbuf = ast_alloc_dma_memory(
      ep->ep.maxpacket, ep->to_host ? DMA_TO_DEVICE : DMA_FROM_DEVICE,
      &ep->txbuf_phys);

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
    ep_hwritel(ep, DESC_STATUS, 0);
  }
  list_del_init(&req->queue);
}

static void ep_dequeue_all(struct ast_ep* ep, int status) {
  struct ast_usb_request *req;
  unsigned long flags;
  spin_lock_irqsave(&ep->lock, flags);
  while (!list_empty(&ep->queue)) {
    req = list_entry(ep->queue.next, struct ast_usb_request, queue);
    ep_dequeue_locked(ep, req);
    req->req.status = status;
    if(req->req.complete) {
      spin_unlock_irqrestore(&ep->lock, flags);
      req->req.complete(&ep->ep, &req->req);
      spin_lock_irqsave(&ep->lock, flags);
    }
  }
  spin_unlock_irqrestore(&ep->lock, flags);
}

static int ast_ep_dequeue(struct usb_ep* _ep, struct usb_request *_rq) {
  struct ast_usb_request *req = to_ast_req(_rq);
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  pr_debug("Dequeue a request (%d bytes) from %s\n",
           req->req.length, ep->ep.name);
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
  pr_debug("Disable %s\n", ep->ep.name);
  if (!ep->active)
    return 0;
  ep_dequeue_all(ep, -ESHUTDOWN);
  spin_lock_irqsave(&ep->lock, flags);
  ast_free_dma_memory(ep->ep.maxpacket,
      ep->to_host ? DMA_TO_DEVICE : DMA_FROM_DEVICE,
      ep->txbuf, ep->txbuf_phys);
  ep->txbuf_phys = 0;
  ep->txbuf = NULL;
  ep->active = 0;
  ep->ep.maxpacket = 1024;
  ep_hwritel(ep, CONFIG, 0);
  spin_unlock_irqrestore(&ep->lock, flags);
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

static void ep_txrx(struct ast_ep* ep, void* buf, size_t len) {
  if (ep->to_host && len) {
    memcpy(ep->txbuf, buf, len);
  }
  ep_hwritel(ep, DESC_STATUS, len << 16);
  ep_hwritel(ep, DESC_STATUS, (len << 16) | 1);
}

static void ep_txrx_check_done(struct ast_ep* ep) {
  struct ast_usb_request *req;
  unsigned long flags;
  u32 status;
  int req_done;

  spin_lock_irqsave(&ep->lock, flags);
  status = ep_hreadl(ep, DESC_STATUS);
  // if txrx complete;
  if (!(status & 0xff)) {
    /* DMA is done */
    ep->dma_busy = 0;
    /* if the list is empty, nothing to do */
    if (list_empty(&ep->queue)) {
      spin_unlock_irqrestore(&ep->lock, flags);
      return;
    }
    req = list_entry(ep->queue.next, struct ast_usb_request, queue);
    if(!req->in_transit) {
      spin_unlock_irqrestore(&ep->lock, flags);
      return;
    }
    //head rq completed
    req->in_transit = 0;
    ep->dma_busy = 0;
    if (!ep->to_host) {
      req->lastpacket = (status >> 16) & 0x3ff;
      __cpuc_flush_kern_all();
      memcpy(req->req.buf + req->req.actual, ep->txbuf, req->lastpacket);
      //print_hex_dump(KERN_DEBUG, "epXrx: ", DUMP_PREFIX_OFFSET, 16, 1, ep->txbuf, req->lastpacket, 0);
    }
    req->req.actual += req->lastpacket;

    req_done = 1;
    if (req->lastpacket < ep->ep.maxpacket) {
      /* the last packet was smaller than maximum size. we are done */

    } else {
      if (req->req.actual < req->req.length) {
        /* the transferred is smaller than requested, need more data xfer */
        req_done = 0;
      } else if (req->req.actual == req->req.length) {
        /*
         * The exact requested has been xferred. which means the xfer is done.
         * Unless, req->req.zero is set (to_host direction only).
         * If req->req.zero is set, we need to make sure a 'short'
         * pkt is sent (zero pkt if needed)
         */
        if (ep->to_host && req->req.zero) {
          /*
           * to_host direction; and req->req.zero is set;
           * and the last pkt was full pkt (not short).
           * Need to send a zero length pkt (short)
           */
          req_done = 0;
        }
      }
    }

    if (req_done) {
      list_del_init(&req->queue);
      spin_unlock_irqrestore(&ep->lock, flags);
      //printk("rq done ep%d\n", ep->addr);
      req->req.status = 0;
      if (req->req.complete) {
        req->req.complete(&ep->ep, &req->req);
      }
    } else {
      //printk("rq partial ep%d %d/%d\n", ep->addr, req->req.actual, req->req.length);
      spin_unlock_irqrestore(&ep->lock, flags);
    }

    ep_vdebug("%s:%d %s lastpacket=%d maxpacket=%d zero=%d to_host=%d "
              "actual=%d length=%d done=%d\n",
              __FUNCTION__, __LINE__, ep->ep.name ? ep->ep.name : "Unknown",
              req->lastpacket, ep->ep.maxpacket, req->req.zero, ep->to_host,
              req->req.actual, req->req.length, req_done);
  }
}

static int ast_ep_queue_run(struct ast_ep* ep) {
  struct ast_usb_request *req = NULL;
  unsigned long flags;
  spin_lock_irqsave(&ep->lock, flags);
  if(ep->active && !ep->dma_busy) {
    //ep isn't busy..
    if(list_empty(&ep->queue)) {
      // nothing
    } else {
      req = list_entry(ep->queue.next, struct ast_usb_request, queue);
      if (req->req.length < req->req.actual) {
        req->lastpacket = 0;
      } else {
        req->lastpacket = req->req.length - req->req.actual;
      }
      if (req->lastpacket > ep->ep.maxpacket) {
        req->lastpacket = ep->ep.maxpacket;
      }
      ep_txrx(ep, req->req.buf + req->req.actual, req->lastpacket);
      req->in_transit = 1;
      ep->dma_busy = 1;
      ep_vdebug("%s:%d %s lastpacket=%d maxpacket=%d zero=%d to_host=%d "
                "actual=%d length=%d \n",
                __FUNCTION__, __LINE__, ep->ep.name ? ep->ep.name : "Unknown",
                req->lastpacket, ep->ep.maxpacket, req->req.zero, ep->to_host,
                req->req.actual, req->req.length);
    }
  }
  spin_unlock_irqrestore(&ep->lock, flags);

  return 0;
}

static void run_all_ep_queues(unsigned long data) {
  int i;
  unsigned long flags;
  spin_lock_irqsave(&udc.lock, flags);

  for(i = 0; i < NUM_ENDPOINTS; i++) {
    if(eps[i].active) {
      ep_txrx_check_done(&eps[i]);
      ast_ep_queue_run(&eps[i]);
    }
  }

  spin_unlock_irqrestore(&udc.lock, flags);
}

DECLARE_TASKLET(check_ep_queues, run_all_ep_queues, 0);

static int ast_ep_queue(struct usb_ep* _ep, struct usb_request *_rq, gfp_t gfp_flags) {
  struct ast_usb_request *req = to_ast_req(_rq);
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;

  ep_vdebug("%s: req l/%u d/%08x %c%c%c\n",
            ep->ep.name ? ep->ep.name : "Unknown",
            req->req.length, req->req.dma,
            req->req.zero ? 'Z' : 'z',
            req->req.short_not_ok ? 'S' : 's',
            req->req.no_interrupt ? 'I' : 'i');
  
  if (ep == &ep0_ep) {
    return ep0_enqueue(_ep, _rq, gfp_flags);
  }

  _rq->status = -EINPROGRESS;
  spin_lock_irqsave(&ep->lock, flags);
  list_add_tail(&req->queue, &ep->queue);
  req->req.actual = 0;
  spin_unlock_irqrestore(&ep->lock, flags);
  ast_ep_queue_run(ep);

  return 0;
}

static int ast_ep_set_halt(struct usb_ep* _ep, int value) {
  struct ast_ep *ep = to_ast_ep(_ep);
  unsigned long flags;
  if (ep == &ep0_ep) {
    /* cannot halt ep0, just return */
    return 0;
  }
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

static irqreturn_t ast_vhub_udc_irq(int irq, void* devid) {
  irqreturn_t status = IRQ_NONE;
  u32 istatus;
  (void) devid;

  istatus = ast_hreadl(&udc, ISR);

  /*
   * Must handle IN/OUT ACK first.
   * For example, in the case of SETUP request DATA IN, after we send out data,
   * we enqueue a request for STATUS ack from host, and stage will be
   * STATUS(IN). When host receives the data, it sends out the STATUS msg
   * and then a new SETUP. In this case, if we process the SETUP first,
   * that will confuse the stage state machine, as it expects STATUS msg
   * for now.
   */
  if(istatus & AST_IRQ_EP0_OUT_ACK) {
    clear_isr(AST_IRQ_EP0_OUT_ACK);
    ep0_handle_ack();
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP0_IN_ACK) {
    clear_isr(AST_IRQ_EP0_IN_ACK);
    ep0_handle_ack();
    status = IRQ_HANDLED;
  }

  if(istatus & AST_IRQ_EP0_SETUP) {
    clear_isr(AST_IRQ_EP0_SETUP);
    ep0_handle_setup();
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
    clear_isr(AST_IRQ_EP_POOL_NAK);
    printk("Got NAK on the EP pool");
    status = IRQ_HANDLED;
  }

  if (status != IRQ_HANDLED) {
    printk("vhub: unhandled interrupts! ISR: %08x\n", istatus);
  } else {
    ep0_handle_status();
  }

  return status;
}

// Added for kernel 4.1
static int ast_udc_start(struct usb_gadget *gadget,
    struct usb_gadget_driver *driver) 
{

  udc.pullup_on = 1;
  udc.driver = driver;
  udc.gadget.dev.driver = &driver->driver;

  return 0;
}

static int ast_udc_stop(struct usb_gadget *gadget) {

  // disappear from the host
  ast_udc_pullup(&udc.gadget, 0);
  if(udc.driver->disconnect) {
    udc.driver->disconnect(&udc.gadget);
  }

  // turn off the EPs
  disable_all_endpoints();

  //udc.gadget.dev.driver = NULL;
  udc.driver = NULL;

  return 0;
}

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
  
  udc.regs = ioremap(res->start, resource_size(res)); 
  if(!udc.regs) {
    dev_err(&pdev->dev, "Couldn't map I/O memory!\n");
    goto error;
  }

  dev_info(&pdev->dev, "ast-vhub at 0x%08lx mapped at %p\n",
      (unsigned long)res->start, udc.regs);
  platform_set_drvdata(pdev, &udc);

  udc.gadget.dev.parent = &pdev->dev;
  udc.gadget.dev.dma_mask = pdev->dev.dma_mask;

  /* disable all interrupts first */
  ast_hwritel(&udc, INTERRUPT_ENABLE, 0);

  ast_hwritel(&udc, ISR, 0x1ffff);
  err = request_irq(irq, ast_vhub_udc_irq, IRQF_SHARED, "ast-vhub", &udc); //harry modified

  if(err) {
    dev_err(&pdev->dev, "failed to get irq %d: %d\n", irq, err);
    goto error;
  }

  udc.ep0_dma_virt = ast_alloc_dma_memory(
      AST_UDC_EP0_MAXPACKET, DMA_BIDIRECTIONAL, &udc.ep0_dma_phys);
  if (!udc.ep0_dma_virt) {
    printk("vhub fatal: Couldn't get DMA memory!\n");
    goto error;
  }
  ast_hwritel(&udc, EP0_DMA_ADDR, udc.ep0_dma_phys);

  printk("virthub init...\n");
  ast_hwritel(&udc, SOFTRESET_ENABLE, 0x33f);
  ast_hwritel(&udc, SOFTRESET_ENABLE, 0x0);
  ast_hwritel(&udc, ISR, 0x1ffff);
  ast_hwritel(&udc, EP_ACK_ISR, 0xffffffff);
  ast_hwritel(&udc, EP_NAK_ISR, 0xffffffff);

  ast_hwritel(&udc, EP1_STATUS, 0x1);
  ast_hwritel(&udc, STATUS, AST_HUB_RESET_DISABLE);

  udc.pdev = pdev;

  INIT_LIST_HEAD(&ep0_ep.ep.ep_list);
  udc.gadget.ep0 = &ep0_ep.ep;
  udc.gadget.speed = USB_SPEED_HIGH;

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
    usb_ep_set_maxpacket_limit(&ep->ep, AST_UDC_EP_MAXPACKET); // kernel 4.1
    spin_lock_init(&ep->lock);
    list_add_tail(&ep->ep.ep_list, &udc.gadget.ep_list);
    
  }

  /* enable interrupts */
  ast_hwritel(&udc, INTERRUPT_ENABLE,
              AST_INT_EP_POOL_ACK
              | AST_INT_EP0_IN_ACK
              | AST_INT_EP0_OUT_ACK
              | AST_INT_EP0_SETUP_ACK);

  //err = device_add(&udc.gadget.dev);    //harry removed
  err = usb_add_gadget_udc(&pdev->dev, &udc.gadget); // harry added, 4.1 use it.
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
    .name = "ast-vhub", //harry modify
    .owner = THIS_MODULE,
  },
};

static int __init ast_udc_init(void) {
  return platform_driver_probe(&ast_vhub_udc_driver, ast_vhub_udc_probe);
}
module_init(ast_udc_init);

static void __exit ast_udc_exit(void) {
  ast_free_dma_memory(AST_UDC_EP0_MAXPACKET, DMA_BIDIRECTIONAL,
                      udc.ep0_dma_virt, udc.ep0_dma_phys);
  platform_driver_unregister(&ast_vhub_udc_driver);
}
module_exit(ast_udc_exit);

MODULE_DESCRIPTION("AST2400/1250 USB UDC Driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:aspeed_udc");
