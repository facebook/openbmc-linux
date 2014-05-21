/*
 * Universal Host Controller Interface driver for USB.
 *
 * Maintainer: Alan Stern <stern@rowland.harvard.edu>
 *
 * (C) Copyright 1999 Linus Torvalds
 * (C) Copyright 1999-2002 Johannes Erdfelt, johannes@erdfelt.com
 * (C) Copyright 1999 Randy Dunlap
 * (C) Copyright 1999 Georg Acher, acher@in.tum.de
 * (C) Copyright 1999 Deti Fliegl, deti@fliegl.de
 * (C) Copyright 1999 Thomas Sailer, sailer@ife.ee.ethz.ch
 * (C) Copyright 1999 Roman Weissgaerber, weissg@vienna.at
 * (C) Copyright 2000 Yggdrasil Computing, Inc. (port of new PCI interface
 *               support from usb-ohci.c by Adam Richter, adam@yggdrasil.com).
 * (C) Copyright 1999 Gregory P. Smith (from usb-ohci.c)
 * (C) Copyright 2004-2007 Alan Stern, stern@rowland.harvard.edu
 *
 * Intel documents this fairly well, and as far as I know there
 * are no royalties or anything like that, but even so there are
 * people who decided that they want to do the same thing in a
 * completely different way.
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/pm.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/bitops.h>
#include <linux/dmi.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <mach/platform.h>
#include "../core/hcd.h"
#include "uhci-hcd.h"
//#include "pci-quirks.h"

/*
 * Version Information
 */
#define DRIVER_AUTHOR "Linus 'Frodo Rabbit' Torvalds, Johannes Erdfelt, \
Randy Dunlap, Georg Acher, Deti Fliegl, Thomas Sailer, Roman Weissgaerber, \
Alan Stern"
#define DRIVER_DESC "USB Universal Host Controller Interface driver"

/* for flakey hardware, ignore overcurrent indicators */
static int ignore_oc;
module_param(ignore_oc, bool, S_IRUGO);
MODULE_PARM_DESC(ignore_oc, "ignore hardware overcurrent indications");

/*
 * debug = 0, no debugging messages
 * debug = 1, dump failed URBs except for stalls
 * debug = 2, dump all failed URBs (including stalls)
 *            show all queues in /debug/uhci/[pci_addr]
 * debug = 3, show all TDs in URBs when dumping
 */
#ifdef DEBUG
#define DEBUG_CONFIGURED	1
static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug level");

#else
#define DEBUG_CONFIGURED	0
#define debug			0
#endif



static char *errbuf;
#define ERRBUF_LEN    (32 * 1024)

static struct kmem_cache *uhci_up_cachep;	/* urb_priv */

void suspend_rh(struct uhci_hcd *uhci, enum uhci_rh_state new_state);
void wakeup_rh(struct uhci_hcd *uhci);
void uhci_get_current_frame_number(struct uhci_hcd *uhci);

void uhci_reset_hc(struct uhci_hcd *uhci);
int uhci_check_and_reset_hc(struct uhci_hcd *uhci);

/*
 * Calculate the link pointer DMA value for the first Skeleton QH in a frame.
 */
static __le32 uhci_frame_skel_link(struct uhci_hcd *uhci, int frame)
{
	int skelnum;

	/*
	 * The interrupt queues will be interleaved as evenly as possible.
	 * There's not much to be done about period-1 interrupts; they have
	 * to occur in every frame.  But we can schedule period-2 interrupts
	 * in odd-numbered frames, period-4 interrupts in frames congruent
	 * to 2 (mod 4), and so on.  This way each frame only has two
	 * interrupt QHs, which will help spread out bandwidth utilization.
	 *
	 * ffs (Find First bit Set) does exactly what we need:
	 * 1,3,5,...  => ffs = 0 => use period-2 QH = skelqh[8],
	 * 2,6,10,... => ffs = 1 => use period-4 QH = skelqh[7], etc.
	 * ffs >= 7 => not on any high-period queue, so use
	 *	period-1 QH = skelqh[9].
	 * Add in UHCI_NUMFRAMES to insure at least one bit is set.
	 */
	skelnum = 8 - (int) __ffs(frame | UHCI_NUMFRAMES);
	if (skelnum <= 1)
		skelnum = 9;
	return LINK_TO_QH(uhci->skelqh[skelnum]);
}

#include "uhci-debug.c"
#include "uhci-q.c"
#include "uhci-hub.c"

/*
 * Finish up a host controller reset and update the recorded state.
 */
void finish_reset(struct uhci_hcd *uhci)
{
	int port;

	/* HCRESET doesn't affect the Suspend, Reset, and Resume Detect
	 * bits in the port status and control registers.
	 * We have to clear them by hand.
	 */
	for (port = 0; port < uhci->rh_numports; ++port)
		writel(0, uhci->regbase + USBPORTSC1 + (port * 1));

	uhci->port_c_suspend = uhci->resuming_ports = 0;
	uhci->rh_state = UHCI_RH_RESET;
	uhci->is_stopped = UHCI_IS_STOPPED;
	uhci_to_hcd(uhci)->state = HC_STATE_HALT;
	uhci_to_hcd(uhci)->poll_rh = 0;

	uhci->dead = 0;		/* Full reset resurrects the controller */
}

/*
 * Last rites for a defunct/nonfunctional controller
 * or one we don't want to use any more.
 */
void uhci_hc_died(struct uhci_hcd *uhci)
{
	uhci_get_current_frame_number(uhci);
//yriver
//	uhci_reset_hc(to_pci_dev(uhci_dev(uhci)), uhci->io_addr);
	uhci_reset_hc(uhci);
	finish_reset(uhci);
	uhci->dead = 1;

	/* The current frame may already be partway finished */
	++uhci->frame_number;
}

/*
 * Initialize a controller that was newly discovered or has lost power
 * or otherwise been reset while it was suspended.  In none of these cases
 * can we be sure of its previous state.
 */
void check_and_reset_hc(struct uhci_hcd *uhci)
{
//yriver
//	if (uhci_check_and_reset_hc(to_pci_dev(uhci_dev(uhci)), uhci->io_addr))
	if (uhci_check_and_reset_hc(uhci))
		finish_reset(uhci);
}

/*
 * Store the basic register settings needed by the controller.
 */
void configure_hc(struct uhci_hcd *uhci)
{
	/* Set the frame length to the default: 1 ms exactly */
//	outb(USBSOF_DEFAULT, uhci->io_addr + USBSOF);
	writel(USBSOF_DEFAULT, uhci->regbase + USBSOF);

	/* Store the frame list base address */
//	outl(uhci->frame_dma_handle, uhci->io_addr + USBFLBASEADD);
	writel(uhci->frame_dma_handle, uhci->regbase + USBFLBASEADD);

	/* Set the current frame number */
//	outw(uhci->frame_number & UHCI_MAX_SOF_NUMBER,
//			uhci->io_addr + USBFRNUM);
	writel(uhci->frame_number & UHCI_MAX_SOF_NUMBER,
			uhci->regbase + USBFRNUM);

	/* Mark controller as not halted before we enable interrupts */
	uhci_to_hcd(uhci)->state = HC_STATE_SUSPENDED;
	mb();
	/* Enable PIRQ */
//yriver
//	pci_write_config_word(to_pci_dev(uhci_dev(uhci)), USBLEGSUP,
//			USBLEGSUP_DEFAULT);
}


int resume_detect_interrupts_are_broken(struct uhci_hcd *uhci)
{
/*

	int port;


	if (ignore_oc)
		return 1;

	switch (to_pci_dev(uhci_dev(uhci))->vendor) {
	    default:
		break;

	    case PCI_VENDOR_ID_GENESYS:
		return 1;

	    case PCI_VENDOR_ID_INTEL:
		for (port = 0; port < uhci->rh_numports; ++port) {
			if (inw(uhci->io_addr + USBPORTSC1 + port * 2) &
					USBPORTSC_OC)
				return 1;
		}
		break;
	}
*/	
	return 0;
}

int global_suspend_mode_is_broken(struct uhci_hcd *uhci)
{
	int port;
	const char *sys_info;
	static char bad_Asus_board[] = "A7V8X";

	/* One of Asus's motherboards has a bug which causes it to
	 * wake up immediately from suspend-to-RAM if any of the ports
	 * are connected.  In such cases we will not set EGSM.
	 */
	sys_info = dmi_get_system_info(DMI_BOARD_NAME);
	if (sys_info && !strcmp(sys_info, bad_Asus_board)) {
		for (port = 0; port < uhci->rh_numports; ++port) {
//yriver
//			if (inw(uhci->io_addr + USBPORTSC1 + port * 2) &
			if (readl(uhci->regbase + USBPORTSC1 + port * 1) &
					USBPORTSC_CCS)
				return 1;
		}
	}

	return 0;
}

void suspend_rh(struct uhci_hcd *uhci, enum uhci_rh_state new_state)
__releases(uhci->lock)
__acquires(uhci->lock)
{
	int auto_stop;
	int int_enable, egsm_enable, wakeup_enable;
	struct usb_device *rhdev = uhci_to_hcd(uhci)->self.root_hub;

	auto_stop = (new_state == UHCI_RH_AUTO_STOPPED);
	dev_dbg(&rhdev->dev, "%s%s\n", __func__,
			(auto_stop ? " (auto-stop)" : ""));

	/* Start off by assuming Resume-Detect interrupts and EGSM work
	 * and that remote wakeups should be enabled.
	 */
	egsm_enable = USBCMD_EGSM;
	uhci->RD_enable = 1;
	int_enable = USBINTR_RESUME;
	wakeup_enable = 1;
/*
	if (auto_stop) {
		if (!device_may_wakeup(&rhdev->dev))
			int_enable = 0;

	} else {
#ifdef CONFIG_PM
		if (!rhdev->do_remote_wakeup)
			wakeup_enable = 0;
#endif
	}

	if (!wakeup_enable || global_suspend_mode_is_broken(uhci))
		egsm_enable = 0;
*/
	/* If we're ignoring wakeup events then there's no reason to
	 * enable Resume-Detect interrupts.  We also shouldn't enable
	 * them if they are broken or disallowed.
	 *
	 * This logic may lead us to enabling RD but not EGSM.  The UHCI
	 * spec foolishly says that RD works only when EGSM is on, but
	 * there's no harm in enabling it anyway -- perhaps some chips
	 * will implement it!
	 */
//yriver
//	if (!wakeup_enable || resume_detect_interrupts_are_broken(uhci) ||
//			!int_enable)
//		uhci->RD_enable = int_enable = 0;

//	outw(int_enable, uhci->io_addr + USBINTR);
//	outw(egsm_enable | USBCMD_CF, uhci->io_addr + USBCMD);
	writel(int_enable, uhci->regbase + USBINTR);
//	writel(egsm_enable | USBCMD_CF, uhci->regbase + USBCMD);
	writel(USBCMD_EGSM | USBCMD_CF, uhci->regbase + USBCMD);
	mb();
	udelay(5);

	/* If we're auto-stopping then no devices have been attached
	 * for a while, so there shouldn't be any active URBs and the
	 * controller should stop after a few microseconds.  Otherwise
	 * we will give the controller one frame to stop.
	 */
//yriver
//	if (!auto_stop && !(inw(uhci->io_addr + USBSTS) & USBSTS_HCH)) {
	if (!auto_stop && !(readl(uhci->regbase + USBSTS) & USBSTS_HCH)) {
		uhci->rh_state = UHCI_RH_SUSPENDING;
		spin_unlock_irq(&uhci->lock);
		msleep(1);
		spin_lock_irq(&uhci->lock);
		if (uhci->dead)
			return;
	}
//yriver
//	if (!(inw(uhci->io_addr + USBSTS) & USBSTS_HCH))
//	if (!(readl(uhci->regbase + USBSTS) & USBSTS_HCH))
//		dev_warn(uhci_dev(uhci), "Controller not stopped yet!\n");

	uhci_get_current_frame_number(uhci);

	uhci->rh_state = new_state;
	uhci->is_stopped = UHCI_IS_STOPPED;

	/* If interrupts don't work and remote wakeup is enabled then
	 * the suspended root hub needs to be polled.
	 */
//yriver
//	uhci_to_hcd(uhci)->poll_rh = (!int_enable && wakeup_enable);
	uhci_to_hcd(uhci)->poll_rh = !int_enable;

	uhci_scan_schedule(uhci);
	uhci_fsbr_off(uhci);
}

void start_rh(struct uhci_hcd *uhci)
{
	uhci_to_hcd(uhci)->state = HC_STATE_RUNNING;
	uhci->is_stopped = 0;
	/* Mark it configured and running with a 64-byte max packet.
	 * All interrupts are enabled, even though RESUME won't do anything.
	 */
//yriver
//	outw(USBCMD_RS | USBCMD_CF | USBCMD_MAXP, uhci->io_addr + USBCMD);
//	outw(USBINTR_TIMEOUT | USBINTR_RESUME | USBINTR_IOC | USBINTR_SP,
//			uhci->io_addr + USBINTR);

	writel(USBCMD_RS | USBCMD_CF | USBCMD_MAXP, uhci->regbase + USBCMD);
	writel(USBINTR_TIMEOUT | USBINTR_RESUME | USBINTR_IOC | USBINTR_SP,
			uhci->regbase + USBINTR);
	mb();
	uhci->rh_state = UHCI_RH_RUNNING;
	uhci_to_hcd(uhci)->poll_rh = 1;
}

void wakeup_rh(struct uhci_hcd *uhci)
__releases(uhci->lock)
__acquires(uhci->lock)
{
	dev_dbg(&uhci_to_hcd(uhci)->self.root_hub->dev,
			"%s%s\n", __func__,
			uhci->rh_state == UHCI_RH_AUTO_STOPPED ?
				" (auto-start)" : "");

	/* If we are auto-stopped then no devices are attached so there's
	 * no need for wakeup signals.  Otherwise we send Global Resume
	 * for 20 ms.
	 */
	if (uhci->rh_state == UHCI_RH_SUSPENDED) {
		uhci->rh_state = UHCI_RH_RESUMING;
	}

	start_rh(uhci);

	/* Restart root hub polling */
	mod_timer(&uhci_to_hcd(uhci)->rh_timer, jiffies);
}

void wakeup_hc(struct uhci_hcd *uhci) {
//	printk("wake_uhci\n");
	wakeup_rh(uhci);
	

}

irqreturn_t uhci_irq(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	unsigned short status;

	/*
	 * Read the interrupt status, and write it back to clear the
	 * interrupt cause.  Contrary to the UHCI specification, the
	 * "HC Halted" status bit is persistent: it is RO, not R/WC.
	 */
//yriver
//	status = inw(uhci->io_addr + USBSTS);
	status = readl(uhci->regbase + USBSTS);
	if (!(status & ~USBSTS_HCH))	/* shared interrupt, not mine */
		return IRQ_NONE;
//	outw(status, uhci->io_addr + USBSTS);		/* Clear it */
	writel(status, uhci->regbase + USBSTS);		/* Clear it */

	if (status & ~(USBSTS_USBINT | USBSTS_ERROR | USBSTS_RD)) {
		if (status & USBSTS_HSE)
			dev_err(uhci_dev(uhci), "host system error, "
					"PCI problems?\n");
		if (status & USBSTS_HCPE)
			dev_err(uhci_dev(uhci), "host controller process "
					"error, something bad happened!\n");
		if (status & USBSTS_HCH) {
			spin_lock(&uhci->lock);
			if (uhci->rh_state >= UHCI_RH_RUNNING) {
				dev_err(uhci_dev(uhci),
					"host controller halted, "
					"very bad!\n");
				if (debug > 1 && errbuf) {
					/* Print the schedule for debugging */
					uhci_sprint_schedule(uhci,
							errbuf, ERRBUF_LEN);
					lprintk(errbuf);
				}
				uhci_hc_died(uhci);

				/* Force a callback in case there are
				 * pending unlinks */
//yriver
//				mod_timer(&hcd->rh_timer, jiffies);
				mod_timer(&uhci_to_hcd(uhci)->rh_timer, jiffies);
			}
			spin_unlock(&uhci->lock);
		}
	}

	if (status & USBSTS_RD)
		usb_hcd_poll_rh_status(hcd);
	else {
		spin_lock(&uhci->lock);
		uhci_scan_schedule(uhci);
		spin_unlock(&uhci->lock);
	}

	return IRQ_HANDLED;
}

/*
 * Store the current frame number in uhci->frame_number if the controller
 * is runnning.  Expand from 11 bits (of which we use only 10) to a
 * full-sized integer.
 *
 * Like many other parts of the driver, this code relies on being polled
 * more than once per second as long as the controller is running.
 */
void uhci_get_current_frame_number(struct uhci_hcd *uhci)
{
	if (!uhci->is_stopped) {
		unsigned delta;
//yriver
//		delta = (inw(uhci->io_addr + USBFRNUM) - uhci->frame_number) &
//				(UHCI_NUMFRAMES - 1);
		delta = (readl(uhci->regbase + USBFRNUM) - uhci->frame_number) &
				(UHCI_NUMFRAMES - 1);
		uhci->frame_number += delta;
	}
}

/*
 * De-allocate all resources
 */
void release_uhci(struct uhci_hcd *uhci)
{
	int i;

	if (DEBUG_CONFIGURED) {
		spin_lock_irq(&uhci->lock);
		uhci->is_initialized = 0;
		spin_unlock_irq(&uhci->lock);

		debugfs_remove(uhci->dentry);
	}

	for (i = 0; i < UHCI_NUM_SKELQH; i++)
		uhci_free_qh(uhci, uhci->skelqh[i]);

	uhci_free_td(uhci, uhci->term_td);

	dma_pool_destroy(uhci->qh_pool);

	dma_pool_destroy(uhci->td_pool);

	kfree(uhci->frame_cpu);

	dma_free_coherent(uhci_dev(uhci),
			UHCI_NUMFRAMES * sizeof(*uhci->frame),
			uhci->frame, uhci->frame_dma_handle);
}

void uhci_reset_hc(struct uhci_hcd *uhci)
{
	/* Reset the HC - this will force us to get a
	 * new notification of any already connected
	 * ports due to the virtual disconnect that it
	 * implies.
	 */
    writel(0,(uhci->regbase + USBINTR));
    writel(USBCMD_GRESET,(uhci->regbase + USBCMD));
    mdelay(50);
    
    writel(0,(uhci->regbase + USBCMD));
    mdelay(10);
}

int uhci_check_and_reset_hc(struct uhci_hcd *uhci)
{ 
    unsigned int cmd, intr;

    cmd = readl((uhci->regbase + USBCMD));
	if ((cmd & USBCMD_RS) || !(cmd & USBCMD_CF) ||
			!(cmd & USBCMD_EGSM)) {
		printk("%s: cmd = 0x%04x\n", __FUNCTION__, cmd);
		goto reset_needed;
	}

	//intr = inw(base + UHCI_USBINTR);
    intr = readl((uhci->regbase + USBINTR));
	if (intr & (~USBINTR_RESUME)) {
		printk("%s: intr = 0x%04x\n", __FUNCTION__, intr);
		goto reset_needed;
	}
	return 0;

reset_needed:
//	printk("Performing full reset\n");
	uhci_reset_hc(uhci);
	return 1;
}

int uhci_reset(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);

	
#ifdef CONFIG_GUC_USB_UHCI_MULTIPORT_2
	uhci->rh_numports = 2;
#elif defined (CONFIG_GUC_USB_UHCI_MULTIPORT_4)
	uhci->rh_numports = 4;
#else
	uhci->rh_numports = 1;
#endif

	/* Kick BIOS off this hardware and reset if the controller
	 * isn't already safely quiescent.
	 */
	check_and_reset_hc(uhci);
	return 0;
}

int uhci_init(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	//unsigned io_size = (unsigned) hcd->rsrc_len;
	int port;

	uhci->io_addr = (unsigned long) hcd->rsrc_start;

	/* The UHCI spec says devices must have 2 ports, and goes on to say
	 * they may have more but gives no way to determine how many there
	 * are.  However according to the UHCI spec, Bit 7 of the port
	 * status and control register is always set to 1.  So we try to
	 * use this to our advantage.  Another common failure mode when
	 * a nonexistent register is addressed is to return all ones, so
	 * we test for that also.
	 */
	for (port = 0; port < UHCI_RH_MAXCHILD; port++) {
		unsigned int portstatus;
//yriver
//		portstatus = inw(uhci->io_addr + USBPORTSC1 + (port * 2));
		portstatus = readl(uhci->regbase + USBPORTSC1 + (port * 1));
		if (!(portstatus & 0x0080) || portstatus == 0xffff)
			break;
	}
	if (debug) {
		dev_info(uhci_dev(uhci), "detected %d ports\n", port);
	}

	/* Anything greater than 7 is weird so we'll ignore it. */
	if (port > UHCI_RH_MAXCHILD) {
		dev_info(uhci_dev(uhci), "port count misdetected? "
				"forcing to 2 ports\n");
		port = 2;
	}
	uhci->rh_numports = port;

	/* Kick BIOS off this hardware and reset if the controller
	 * isn't already safely quiescent.
	 */
	check_and_reset_hc(uhci);
	return 0;
}

/* Make sure the controller is quiescent and that we're not using it
 * any more.  This is mainly for the benefit of programs which, like kexec,
 * expect the hardware to be idle: not doing DMA or generating IRQs.
 *
 * This routine may be called in a damaged or failing kernel.  Hence we
 * do not acquire the spinlock before shutting down the controller.
 */
void uhci_shutdown(struct pci_dev *pdev)
{
	struct usb_hcd *hcd = (struct usb_hcd *) pci_get_drvdata(pdev);

	uhci_hc_died(hcd_to_uhci(hcd));
}

/*
 * Allocate a frame list, and then setup the skeleton
 *
 * The hardware doesn't really know any difference
 * in the queues, but the order does matter for the
 * protocols higher up.  The order in which the queues
 * are encountered by the hardware is:
 *
 *  - All isochronous events are handled before any
 *    of the queues. We don't do that here, because
 *    we'll create the actual TD entries on demand.
 *  - The first queue is the high-period interrupt queue.
 *  - The second queue is the period-1 interrupt and async
 *    (low-speed control, full-speed control, then bulk) queue.
 *  - The third queue is the terminating bandwidth reclamation queue,
 *    which contains no members, loops back to itself, and is present
 *    only when FSBR is on and there are no full-speed control or bulk QHs.
 */
int uhci_start(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	int retval = -EBUSY;
	int i;
	struct dentry *dentry;

	hcd->uses_new_polling = 1;

	spin_lock_init(&uhci->lock);
	setup_timer(&uhci->fsbr_timer, uhci_fsbr_timeout,
			(unsigned long) uhci);
	INIT_LIST_HEAD(&uhci->idle_qh_list);
	init_waitqueue_head(&uhci->waitqh);

	if (DEBUG_CONFIGURED) {
		dentry = debugfs_create_file(hcd->self.bus_name,
				S_IFREG|S_IRUGO|S_IWUSR, uhci_debugfs_root,
				uhci, &uhci_debug_operations);
		if (!dentry) {
			dev_err(uhci_dev(uhci), "couldn't create uhci "
					"debugfs entry\n");
			retval = -ENOMEM;
			goto err_create_debug_entry;
		}
		uhci->dentry = dentry;
	}

	uhci->frame = dma_alloc_coherent(uhci_dev(uhci),
			UHCI_NUMFRAMES * sizeof(*uhci->frame),
			&uhci->frame_dma_handle, 0);
	if (!uhci->frame) {
		dev_err(uhci_dev(uhci), "unable to allocate "
				"consistent memory for frame list\n");
		goto err_alloc_frame;
	}

	memset(uhci->frame, 0, UHCI_NUMFRAMES * sizeof(*uhci->frame));

	uhci->frame_cpu = kcalloc(UHCI_NUMFRAMES, sizeof(*uhci->frame_cpu),
			GFP_KERNEL);
	if (!uhci->frame_cpu) {
		dev_err(uhci_dev(uhci), "unable to allocate "
				"memory for frame pointers\n");
		goto err_alloc_frame_cpu;
	}

	uhci->td_pool = dma_pool_create("uhci_td", uhci_dev(uhci),
			sizeof(struct uhci_td), 16, 0);
	if (!uhci->td_pool) {
		dev_err(uhci_dev(uhci), "unable to create td dma_pool\n");
		goto err_create_td_pool;
	}

	uhci->qh_pool = dma_pool_create("uhci_qh", uhci_dev(uhci),
			sizeof(struct uhci_qh), 16, 0);
	if (!uhci->qh_pool) {
		dev_err(uhci_dev(uhci), "unable to create qh dma_pool\n");
		goto err_create_qh_pool;
	}

	uhci->term_td = uhci_alloc_td(uhci);
	if (!uhci->term_td) {
		dev_err(uhci_dev(uhci), "unable to allocate terminating TD\n");
		goto err_alloc_term_td;
	}

	for (i = 0; i < UHCI_NUM_SKELQH; i++) {
		uhci->skelqh[i] = uhci_alloc_qh(uhci, NULL, NULL);
		if (!uhci->skelqh[i]) {
			dev_err(uhci_dev(uhci), "unable to allocate QH\n");
			goto err_alloc_skelqh;
		}
	}

	/*
	 * 8 Interrupt queues; link all higher int queues to int1 = async
	 */
	for (i = SKEL_ISO + 1; i < SKEL_ASYNC; ++i)
		uhci->skelqh[i]->link = LINK_TO_QH(uhci->skel_async_qh);
	uhci->skel_async_qh->link = UHCI_PTR_TERM;
	uhci->skel_term_qh->link = LINK_TO_QH(uhci->skel_term_qh);

	/* This dummy TD is to work around a bug in Intel PIIX controllers */
	uhci_fill_td(uhci->term_td, 0, uhci_explen(0) |
			(0x7f << TD_TOKEN_DEVADDR_SHIFT) | USB_PID_IN, 0);
	uhci->term_td->link = UHCI_PTR_TERM;
	uhci->skel_async_qh->element = uhci->skel_term_qh->element =
			LINK_TO_TD(uhci->term_td);

	/*
	 * Fill the frame list: make all entries point to the proper
	 * interrupt queue.
	 */
	for (i = 0; i < UHCI_NUMFRAMES; i++) {

		/* Only place we don't use the frame list routines */
		uhci->frame[i] = uhci_frame_skel_link(uhci, i);
	}

	/*
	 * Some architectures require a full mb() to enforce completion of
	 * the memory writes above before the I/O transfers in configure_hc().
	 */
	mb();

	configure_hc(uhci);

	uhci->is_initialized = 1;
	start_rh(uhci);

	return 0;

/*
 * error exits:
 */
err_alloc_skelqh:
	for (i = 0; i < UHCI_NUM_SKELQH; i++) {
		if (uhci->skelqh[i])
			uhci_free_qh(uhci, uhci->skelqh[i]);
	}

	uhci_free_td(uhci, uhci->term_td);

err_alloc_term_td:
	dma_pool_destroy(uhci->qh_pool);

err_create_qh_pool:
	dma_pool_destroy(uhci->td_pool);

err_create_td_pool:
	kfree(uhci->frame_cpu);

err_alloc_frame_cpu:
	dma_free_coherent(uhci_dev(uhci),
			UHCI_NUMFRAMES * sizeof(*uhci->frame),
			uhci->frame, uhci->frame_dma_handle);

err_alloc_frame:
	debugfs_remove(uhci->dentry);

err_create_debug_entry:
	return retval;
}

void uhci_stop(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);

	spin_lock_irq(&uhci->lock);
	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) && !uhci->dead)
		uhci_hc_died(uhci);
	uhci_scan_schedule(uhci);
	spin_unlock_irq(&uhci->lock);

	del_timer_sync(&uhci->fsbr_timer);
	release_uhci(uhci);
}

void stop_hc(struct uhci_hcd *uhci)
{
    // Disable all interrupts
    writel(0, (uhci->regbase + USBINTR));
    writel(USBCMD_MAXP,(uhci->regbase + USBCMD)); // disable hc
                
}

#ifdef CONFIG_PM
int uhci_rh_suspend(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	int rc = 0;

	spin_lock_irq(&uhci->lock);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))
		rc = -ESHUTDOWN;
	else if (!uhci->dead)
		suspend_rh(uhci, UHCI_RH_SUSPENDED);
	spin_unlock_irq(&uhci->lock);
	return rc;
}

int uhci_rh_resume(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	int rc = 0;

	spin_lock_irq(&uhci->lock);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))
		rc = -ESHUTDOWN;
	else if (!uhci->dead)
		wakeup_rh(uhci);
	spin_unlock_irq(&uhci->lock);
	return rc;
}

int uhci_pci_suspend(struct usb_hcd *hcd, pm_message_t message)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	int rc = 0;

	dev_dbg(uhci_dev(uhci), "%s\n", __func__);

	spin_lock_irq(&uhci->lock);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) || uhci->dead)
		goto done_okay;		/* Already suspended or dead */

	if (uhci->rh_state > UHCI_RH_SUSPENDED) {
		dev_warn(uhci_dev(uhci), "Root hub isn't suspended!\n");
		rc = -EBUSY;
		goto done;
	};

	/* All PCI host controllers are required to disable IRQ generation
	 * at the source, so we must turn off PIRQ.
	 */
	pci_write_config_word(to_pci_dev(uhci_dev(uhci)), USBLEGSUP, 0);
	mb();
	hcd->poll_rh = 0;

	/* FIXME: Enable non-PME# remote wakeup? */

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW)
		uhci_hc_died(uhci);

done_okay:
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
done:
	spin_unlock_irq(&uhci->lock);
	return rc;
}

int uhci_pci_resume(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);

	dev_dbg(uhci_dev(uhci), "%s\n", __func__);

	/* Since we aren't in D3 any more, it's safe to set this flag
	 * even if the controller was dead.
	 */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	mb();

	spin_lock_irq(&uhci->lock);

	/* FIXME: Disable non-PME# remote wakeup? */

	/* The firmware or a boot kernel may have changed the controller
	 * settings during a system wakeup.  Check it and reconfigure
	 * to avoid problems.
	 */
	check_and_reset_hc(uhci);

	/* If the controller was dead before, it's back alive now */
	configure_hc(uhci);

	if (uhci->rh_state == UHCI_RH_RESET) {

		/* The controller had to be reset */
		usb_root_hub_lost_power(hcd->self.root_hub);
		suspend_rh(uhci, UHCI_RH_SUSPENDED);
	}

	spin_unlock_irq(&uhci->lock);

	/* If interrupts don't work and remote wakeup is enabled then
	 * the suspended root hub needs to be polled.
	 */
	if (!uhci->RD_enable && hcd->self.root_hub->do_remote_wakeup) {
		hcd->poll_rh = 1;
		usb_hcd_poll_rh_status(hcd);
	}
	return 0;
}
#endif

/* Wait until a particular device/endpoint's QH is idle, and free it */
void uhci_hcd_endpoint_disable(struct usb_hcd *hcd,
		struct usb_host_endpoint *hep)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	struct uhci_qh *qh;

	spin_lock_irq(&uhci->lock);
	qh = (struct uhci_qh *) hep->hcpriv;
	if (qh == NULL)
		goto done;

	while (qh->state != UHCI_QH_STATE_IDLE) {
		++uhci->num_waiting;
		spin_unlock_irq(&uhci->lock);
		wait_event_interruptible(uhci->waitqh,
				qh->state == UHCI_QH_STATE_IDLE);
		spin_lock_irq(&uhci->lock);
		--uhci->num_waiting;
	}

	uhci_free_qh(uhci, qh);
done:
	spin_unlock_irq(&uhci->lock);
}

int uhci_hcd_get_frame_number(struct usb_hcd *hcd)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);
	unsigned frame_number;
	unsigned delta;

	/* Minimize latency by avoiding the spinlock */
	frame_number = uhci->frame_number;
	barrier();
//yriver
//	delta = (inw(uhci->io_addr + USBFRNUM) - frame_number) &
//			(UHCI_NUMFRAMES - 1);
	delta = (readl(uhci->regbase	 + USBFRNUM) - frame_number) &
			(UHCI_NUMFRAMES - 1);
	return frame_number + delta;
}

//static const char hcd_name[] = "uhci_hcd";

static const struct hc_driver _uhci_driver = {
//yriver
//	.description =		hcd_name,
	.description =		"ast uhci",
	.product_desc =		"UHCI Host Controller",
	.hcd_priv_size =	sizeof(struct uhci_hcd),

	/* Generic hardware linkage */
	.irq =			uhci_irq,
	.flags =		HCD_USB11,

	/* Basic lifecycle operations */
//	.reset =		uhci_init,
	.reset =		uhci_reset,
	.start =		uhci_start,
#ifdef CONFIG_PM
	.pci_suspend =		uhci_pci_suspend,
	.pci_resume =		uhci_pci_resume,
	.bus_suspend =		uhci_rh_suspend,
	.bus_resume =		uhci_rh_resume,
#endif
	.stop =			uhci_stop,

	.urb_enqueue =		uhci_urb_enqueue,
	.urb_dequeue =		uhci_urb_dequeue,

	.endpoint_disable =	uhci_hcd_endpoint_disable,
	.get_frame_number =	uhci_hcd_get_frame_number,

	.hub_status_data =	uhci_hub_status_data,
	.hub_control =		uhci_hub_control,
};

int __init uhci_hcd_init(void)
{
	int retval = -ENOMEM;

	if (usb_disabled())
		return -ENODEV;

	printk(KERN_INFO "uhci_hcd: " DRIVER_DESC "%s\n",
			ignore_oc ? ", overcurrent ignored" : "");
	set_bit(USB_UHCI_LOADED, &usb_hcds_loaded);

	if (DEBUG_CONFIGURED) {
		errbuf = kmalloc(ERRBUF_LEN, GFP_KERNEL);
		if (!errbuf)
			goto errbuf_failed;
		uhci_debugfs_root = debugfs_create_dir("uhci", NULL);
		if (!uhci_debugfs_root)
			goto debug_failed;
	}

	uhci_up_cachep = kmem_cache_create("uhci_urb_priv",
		sizeof(struct urb_priv), 0, 0, NULL);
	if (!uhci_up_cachep)
		goto up_failed;
/*
	retval = pci_register_driver(&uhci_pci_driver);
	if (retval)
		goto init_failed;
*/
	return 0;

//init_failed:
//	kmem_cache_destroy(uhci_up_cachep);

up_failed:
	debugfs_remove(uhci_debugfs_root);

debug_failed:
	kfree(errbuf);

errbuf_failed:

	clear_bit(USB_UHCI_LOADED, &usb_hcds_loaded);
	return retval;
}

void uhci_hcd_cleanup(void) 
{
//	pci_unregister_driver(&uhci_pci_driver);
	kmem_cache_destroy(uhci_up_cachep);
	debugfs_remove(uhci_debugfs_root);
	kfree(errbuf);
	clear_bit(USB_UHCI_LOADED, &usb_hcds_loaded);
}

void __exit cleanup_UHCI(void)
{
    uhci_hcd_cleanup();
}

/*-------------------------------------------------------------------------*/
static int usb_hcd_guc_probe (const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	unsigned int *base, temp;
	int retval;
	struct resource *res;	
	struct usb_hcd *hcd = 0;
	struct uhci_hcd *uhci;
	int irq;
	
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto err1;
	}

	hcd = usb_create_hcd (driver, &pdev->dev, dev_name(&pdev->dev));	
//	printk("alloc_uhci(2): uhci_dev:%x, _uhci_hcd.state:%x\n", &uhci_dev, _uhci_hcd->state);
	if (!hcd) {
		retval = -ENOMEM;
		return retval;
	}


	uhci = hcd_to_uhci(hcd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto err1;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1,
				res->name)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto err1;
	}

	base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (base == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -ENOMEM;
		goto err1;
	}

	uhci->regbase = (unsigned int *)base;
	uhci->io_addr = (unsigned int)base;

//	printk("UHCI Base address is %x, phy %08x\n",(unsigned int)base, UHC_BASE_ADDR);

	retval = usb_add_hcd (hcd, irq, IRQF_SHARED);
	
	//    printk("alloc_uhci(3): uhci_dev:%x, _uhci_hcd.state:%x\n", &uhci_dev, _uhci_hcd->state);

	if (retval == 0)
		return retval;

	//BruceToDo. Stop USB 1.1 Host's clock.
	iounmap((void*)uhci->io_addr);
err1:
	usb_put_hcd(hcd);

	printk("add UHCI to USB host controller list failed!\n");
	return retval;
}


static inline void
usb_hcd_guc_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct uhci_hcd *uhci = hcd_to_uhci(hcd);

	usb_remove_hcd(hcd);
	//BruceToDo. Stop USB 1.1 Host's clock.
	iounmap((void*)uhci->io_addr);
	usb_put_hcd(hcd);
}

/*-------------------------------------------------------------------------*/
#ifdef	CONFIG_PM
static int uhci_guc_suspend(struct platform_device *dev, pm_message_t message)
{
	struct uhci_hcd	*uhci = hcd_to_uhci(platform_get_drvdata(dev));
#if 0
	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;
	omap_ohci_clock_power(0);
#endif
	uhci_to_hcd(uhci)->state = HC_STATE_SUSPENDED;
	return 0;
}
static int uhci_guc_resume(struct platform_device *dev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
#if 0
	struct uhci_hcd	*uhci = hcd_to_uhci(hcd);

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;
	omap_ohci_clock_power(1);
	ohci_finish_controller_resume(hcd);
#endif

	/*Bruce111220. This line is copied from AST1510 uhci-hcd.c and OMAP kernel 2.6.15*/
	usb_hcd_resume_root_hub(hcd);
	return 0;
}
#endif
/*-------------------------------------------------------------------------*/

static int uhci_hcd_guc_drv_probe(struct platform_device *dev)
{
	return usb_hcd_guc_probe(&_uhci_driver, dev);
}
static int uhci_hcd_guc_drv_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);

	usb_hcd_guc_remove(hcd, dev);
	platform_set_drvdata(dev, NULL);
	return 0;
}

/*
 * Driver definition to register
 */
static struct platform_driver uhci_hcd_guc_driver = {
	.probe		= uhci_hcd_guc_drv_probe,
	.remove		= uhci_hcd_guc_drv_remove,
#ifdef	CONFIG_PM
	.suspend	= uhci_guc_suspend,
	.resume		= uhci_guc_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ast_uhci",
	},
};

static int __init uhci_hcd_guc_init (void)
{
	uhci_hcd_init();
	return platform_driver_register(&uhci_hcd_guc_driver);
}

static void __exit uhci_hcd_guc_cleanup (void)
{
	uhci_hcd_cleanup();
	platform_driver_unregister(&uhci_hcd_guc_driver);
}

module_init(uhci_hcd_guc_init);
module_exit(uhci_hcd_guc_cleanup);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
