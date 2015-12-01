/********************************************************************************
* File Name     : drivers/usb/host/ehci-aspeed.c 
* Author         : Ryan Chen
* Description   : EHCI HCD (Host Controller Driver) for USB
* 
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
*
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   History      : 
*    1. 2012/08/17 ryan chen create this file 
* 
********************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>


/* ASPEED EHCI USB Host Controller */

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

static int ehci_ast_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

#ifdef CONFIG_USB_EHCI_ROOT_HUB_TT
	hcd->has_tt = 1;
#else
	hcd->has_tt = 0;
#endif

	ehci->sbrn = 0x20;
		 
//	retval = ehci_halt(ehci);
//	if (retval)
//		return retval;


	/*
	 * data structure init
	 */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci_reset(ehci);
//	ehci_port_power(ehci, 0);

	return retval;
}

static const struct hc_driver ehci_ast_hc_driver = {
	.description = hcd_name,
	.product_desc = "ASPEED On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),
	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,
	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_ast_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,
	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,
	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,
};

static int ehci_ast_drv_probe(struct platform_device *pdev)
{
		struct resource *res;
		struct usb_hcd *hcd;
//		struct ehci_hcd *ehci;
		void __iomem *regs;
		int irq, err;
	
		if (usb_disabled())
			return -ENODEV;

		pr_debug("Initializing ASPEED-SoC USB Host Controller\n");
	
		irq = platform_get_irq(pdev, 0);
		if (irq <= 0) {
			dev_err(&pdev->dev,
				"Found HC with no IRQ. Check %s setup!\n",
				dev_name(&pdev->dev));
			err = -ENODEV;
			goto err1;
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev,
				"Found HC with no register addr. Check %s setup!\n",
				dev_name(&pdev->dev));
			err = -ENODEV;
			goto err1;
		}

		if (!request_mem_region(res->start, res->end - res->start + 1,
					res->name)) {
			dev_dbg(&pdev->dev, "controller already in use\n");
			err = -EBUSY;
			goto err1;
		}

		regs = ioremap_nocache(res->start, res->end - res->start + 1);
		if (regs == NULL) {
			dev_dbg(&pdev->dev, "error mapping memory\n");
			err = -EFAULT;
			goto err2;
		}

		hcd = usb_create_hcd(&ehci_ast_hc_driver,
				&pdev->dev, dev_name(&pdev->dev));
		if (!hcd) {
			err = -ENOMEM;
			goto err3;
		}

		hcd->rsrc_start = res->start;
		hcd->rsrc_len = res->end - res->start + 1;
		hcd->regs = regs;

		err = usb_add_hcd(hcd, irq, IRQF_DISABLED);
		if (err)
			goto err4;
		
		return 0;
	
	err4:
		usb_put_hcd(hcd);
	err3:
		iounmap(regs);
	err2:
		release_mem_region(res->start, res->end - res->start + 1);
	err1:
		dev_err(&pdev->dev, "init %s fail, %d\n",
			dev_name(&pdev->dev), err);
	
		return err;


}

static int ehci_ast_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;	
}

 /*TBD*/
#ifdef CONFIG_PM
static int ehci_hcd_ast_drv_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	unsigned long		flags;
	int			rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave (&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 bail:
	spin_unlock_irqrestore (&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;
}
static int ehci_hcd_ast_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;
	return 0;
}
#endif

MODULE_ALIAS("platform:ehci_ast");

static struct platform_driver ehci_hcd_ast_driver = {
	.probe = ehci_ast_drv_probe,
	.remove = ehci_ast_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM	
	.suspend      	= ehci_hcd_ast_drv_suspend,
	.resume       	= ehci_hcd_ast_drv_resume,
#endif	
	.driver = {
		.name	= "ehci-ast",
	},
};
