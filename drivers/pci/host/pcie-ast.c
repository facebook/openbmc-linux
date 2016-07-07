/********************************************************************************
* File Name     : arch/arm/plat-aspeed/pcie.c
* Author        : Ryan Chen
* Description   : Aspeed PCI Express Driver
*
* Copyright (C) Aspeed Tech. Corp.
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

* History      :
*    1. 2013/07/08 Ryan Chen create this file
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/delay.h>

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/msi.h>

#include <asm/io.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/irqs.h>

#include <linux/module.h>

#include <linux/slab.h>
#include <linux/msi.h>
#include <linux/pci.h>

#include <plat/regs-pcie.h>
#include <plat/ast-scu.h>
#include <plat/ast-ahbc.h>
#include <plat/ast-sdmc.h>
#include <plat/ast-pciarbiter.h>
#include <plat/ast_p2x.h>

//#define CONFIG_PCIE_DEBUG

#ifdef CONFIG_PCIE_DEBUG
#define AST_PCIEDBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AST_PCIEDBG(fmt, args...)
#endif

#ifdef CONFIG_PCI_MSI
static DECLARE_BITMAP(msi_irq_in_use, 32);
#endif

void __iomem	*ast_pcie_base;

/*************************************************************************/
static inline void
ast_pcie_write(u32 val, u32 reg)
{
//	AST_PCIEDBG("reg = 0x%04x, val = 0x%08x, base = 0x%08x\n", reg, val, base);
	iowrite32(val, ast_pcie_base + reg);
}

static inline u32
ast_pcie_read(u32 reg)
{
	u32 val = ioread32(ast_pcie_base + reg);
//	AST_PCIEDBG("reg = 0x%04x, val = 0x%08x, base = 0x%08x\n", reg, val, base);
	return val;
}
/*************************************************************************/
static int
ast_pcie_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	if(!(ast_pcie_read(AST_PCIE_LINK) & PCIE_LINK_STS))
		return PCIBIOS_DEVICE_NOT_FOUND;

	AST_PCIEDBG("%x:%x:%x => size = %d, where = %xh, ",
		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), size, where);

	if((PCI_SLOT(devfn) > 0) && (bus->number < 2)) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	} else if(bus->number == 0) {
		ast_pcie_cfg_read(0, 
						(bus->number << 24) |
						(PCI_SLOT(devfn) << 19) |
						(PCI_FUNC(devfn) << 16) | 
						(where & ~3), val);		
	} else {
		ast_pcie_cfg_read(1, 
						(bus->number << 24) |
						(PCI_SLOT(devfn) << 19) |
						(PCI_FUNC(devfn) << 16) | 
						(where & ~3), val);
	}

	AST_PCIEDBG("val = %xh \n",*val);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	return PCIBIOS_SUCCESSFUL;
}

static int
ast_pcie_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	AST_PCIEDBG("%x:%x:%x => size = %d, where = %xh, val = %xh\n",
		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), size, where, val);

	if(bus->number == 0) {
		ast_pcie_cfg_write(0, (((1 << size) -1) << (where & 0x3)),
					(bus->number << 24) |
					(PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) | 
					(where & ~3), val << ((where & 0x3) * 8));
	} else {
		ast_pcie_cfg_write(1, (((1 << size) -1) << (where & 0x3)),
					(bus->number << 24) |
					(PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) | 
					(where & ~3), val << ((where & 0x3) * 8));	
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops ast_pcie_ops = {
	.read	= ast_pcie_read_config,
	.write	= ast_pcie_write_config,
};

static struct resource non_mem = {
	.name	= "AST PCIe non-prefetchable memory region",
	.start	= AST_PCIE_WIN_BASE,
	.end		= AST_PCIE_WIN_BASE + AST_PCIE_WIN_SIZE - 1,
	.flags	= IORESOURCE_MEM,
};

static int __init
ast_pcie_setup(int nr, struct pci_sys_data *sys)
{
	AST_PCIEDBG("PCI nr : %d >>> \n", nr);

	//px2 decode addr
	ast_p2x_addr_map(0xF0000000, AST_PCIE_WIN_BASE);

	//VGA Init for memory cycle
	ast_pciarbiter_pcie_init();

	ast_sdmc_disable_mem_protection(16);

	//Add for Gen2
	ast_pcie_write(0x20, AST_PCIE_CFG_DIN);
	ast_pcie_write((ast_pcie_read(AST_PCIE_CFG3) & ~(PCIE_CFG_ADDR_MASK| PCIE_CFG_WRITE)) | PCIE_CFG_ADDR(0x090), AST_PCIE_CFG3);
	ast_pcie_write(ast_pcie_read(AST_PCIE_CFG3) | PCIE_CFG_WRITE , AST_PCIE_CFG3);
	ast_pcie_write((ast_pcie_read(AST_PCIE_CFG3) & ~PCIE_CFG_WRITE) , AST_PCIE_CFG3);	

	ast_pcie_cfg_write(0, (((1 << 4) -1) << (PCI_PRIMARY_BUS & 0x3)),
				(PCI_PRIMARY_BUS & ~3), 0 << ((PCI_PRIMARY_BUS & 0x3) * 8));

	ast_pcie_cfg_write(0, (((1 << 4) -1) << (PCI_SECONDARY_BUS & 0x3)),
				(PCI_SECONDARY_BUS & ~3), 1 << ((PCI_SECONDARY_BUS & 0x3) * 8));

	ast_pcie_cfg_write(0, (((1 << 4) -1) << (PCI_SUBORDINATE_BUS & 0x3)),
				(PCI_SUBORDINATE_BUS & ~3), 0xf << ((PCI_SUBORDINATE_BUS & 0x3) * 8));

//	ast_pcie_cfg_write(0, 0xf, PCI_PRIMARY_BUS, 0);
//	ast_pcie_cfg_write(0, 0xf, PCI_SECONDARY_BUS, 1);
//	ast_pcie_cfg_write(0, 0xf, PCI_SUBORDINATE_BUS, 1);

	if (request_resource(&iomem_resource, &non_mem)) {
		printk(KERN_ERR "PCI: unable to allocate non-prefetchable "
		       "memory region\n");
		return -EBUSY;
	}
	pci_add_resource_offset(&sys->resources, &non_mem, sys->mem_offset);

	return 1;
}

static int irq_tab[] __initdata = {
	IRQ_PCIE_INTA,
	IRQ_PCIE_INTB,
	IRQ_PCIE_INTC,
	IRQ_PCIE_INTD
};

static int 
ast_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	AST_PCIEDBG("slot = %d, pin = %d, irq = %d\n", slot, pin, irq_tab[(slot + pin - 1) % 4]);
	return irq_tab[(slot + pin - 1) % 4];
}

static inline void 
init_pcie_rc_bridge(void)
{
	AST_PCIEDBG("\n");

	ast_scu_init_pcie();
	ast_ahbc_peie_mapping(1);

	//plda enable 
	ast_pcie_write(PCIE_UNLOCK, AST_PCIE_LOCK);
	ast_pcie_write(PCIE_CFG_CLASS_CODE(0x60400) | PCIE_CFG_REV_ID(4), AST_PCIE_CFG2);
	ast_pcie_write(ROOT_COMPLEX_ID(0x3), AST_PCIE_GLOBAL);



#if 0
	ast_pcie_cfg_write(0, 0xf, PCI_COMMAND, 0x147);	
#if 1
//	ast_pcie_cfg_write(0, 0xf, PCI_BASE_ADDRESS_2, 0x00020100); 
//	ast_pcie_cfg_write(0, 0xf, PCI_BASE_ADDRESS_4, 0x7fff7000); 
#else
	ast_pcie_cfg_write(0, 0xf, PCI_BASE_ADDRESS_2, 0x00010100); 
	ast_pcie_cfg_write(0, 0xf, PCI_BASE_ADDRESS_4, 0x1e701e70); 

#endif
#endif


}

static struct hw_pci ast_pcie = {
	.nr_controllers = 1,
	.setup          = ast_pcie_setup,
	.ops			= &ast_pcie_ops,	
	.map_irq        = ast_pcie_map_irq,
};

/**
 * ast_pcie_probe() - Invoke PCI BIOS to perrform enumeration.
 * @pdev: Contains platform data as supplied from board level code.
 *
 * Also stores reference to platform device structure for use during PCIe
 * module initialization and configuration.
 */

static int ast_pcie_probe(struct platform_device *pdev)
{
	ast_pcie_base =  ioremap(AST_PCIE_PLDA_BASE , SZ_256);	

	AST_PCIEDBG("\n");
	init_pcie_rc_bridge();

	mdelay(30);

	if(ast_pcie_read(AST_PCIE_LINK) & PCIE_LINK_STS) {
		printk("PCIE Link \n");
		pci_common_init_dev(&pdev->dev, &ast_pcie);
	} else {
		printk("PCIE unLink \n");
	}

	return 0;
}

static struct platform_driver ast_pcie_driver = {
	.driver = {
		.name	= "ast-pcie",
		.owner	= THIS_MODULE,
	},
	.probe = ast_pcie_probe,
};

/**
 * ast_pcie_rc_init() - Register PCIe Root Complex node.
 *
 * Invoked as subsystem initialization.
 *
 * IMPORTANT NOTE: We are relying on SoC/Board level code to check PCIESS
 * mode setting (RC/EP) and register the RC device only in RC mode.
 */
static int __init ast_pcie_rc_init(void)
{
	platform_driver_register(&ast_pcie_driver);
	return 0;
}
module_init(ast_pcie_rc_init);
