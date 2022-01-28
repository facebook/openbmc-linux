// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2020 Aspeed Technology Inc.
 */
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/sysfs.h>
#include <dt-bindings/gpio/aspeed-gpio.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "aspeed-espi.h"
#include "aspeed-espi-ctrl.h"
#include "aspeed-espi-perif.h"
#include "aspeed-espi-mmbi.h"

#define AST_SCU_HW_STRAP2	(0x510)
#define ENABLE_LPC_MODE		BIT(6)
#define AST_SCU_DBG_CTRL2	(0xD8)
#define FORCE_DISABLE_ESPI_TO_AHB_BRIDGE	BIT(0)
#define MAX_APP_INSTANCES	(8)

struct aspeed_espi_mmbi_data {
	struct device			*dev;
	void __iomem			*reg_base;
	int 					irq;
	int						dma_mode;		/* o:disable , 1:enable */
	int          			espi_mmbi_reset;
	int						reset_irq;
	u8						*mmbi_blk_virt;
	dma_addr_t				mmbi_blk_phys;
	bool 					is_open;
	spinlock_t 				espi_mmbi_lock;
	u8						espi_mmbi_intr_sts[MAX_APP_INSTANCES];
	wait_queue_head_t 		espi_mmbi_intr_wq[MAX_APP_INSTANCES];
	struct mmbi_host_rwp   	arr_mmbi_host_rwp[MAX_APP_INSTANCES];
	struct aspeed_espi_ctrl *espi_ctrl;
};


#define ASPEED_ESPI_MMBI_DEBUG

#ifdef ASPEED_ESPI_MMBI_DEBUG
#define ESPI_MMBI_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define ESPI_MMBI_DBUG(fmt, args...)
#endif


static DEFINE_SPINLOCK(espi_mmbi_state_lock);


static inline u32 aspeed_espi_mmbi_read(struct aspeed_espi_mmbi_data *aspeed_espi_mmbi, u32 reg)
{
#if 0
	u32 val;
	val = readl(aspeed_espi_mmbi->reg_base + reg);
	ESPI_MMBI_DBUG("aspeed_espi_mmbi_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(aspeed_espi_mmbi->reg_base + reg);
#endif
}

static inline void aspeed_espi_mmbi_write(struct aspeed_espi_mmbi_data *aspeed_espi_mmbi, u32 val, u32 reg)
{
//	ESPI_MMBI_DBUG("aspeed_espi_mmbi_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_espi_mmbi->reg_base + reg);
}

static u8* aspeed_espi_mmbi_get_mem_loc(struct aspeed_espi_mmbi_data *aspeed_espi_mmbi,
						u32 offset, u32 len) {
	u8	*mmbi_ptr = aspeed_espi_mmbi->mmbi_blk_virt;
	if (offset+len<=MMBI_TOTAL_SIZE) {
		return (u8*)(mmbi_ptr+offset);
	}
	else {
		return NULL;
	}
}

static void aspeed_espi_mmbi_ctrl_init(struct aspeed_espi_mmbi_data *aspeed_espi_mmbi)
{
	u32 val = 0;

	aspeed_espi_mmbi_write(aspeed_espi_mmbi, 0x0, ESPI_MMBI_INT_ENL);
	aspeed_espi_mmbi_write(aspeed_espi_mmbi, 0xffff, ESPI_MMBI_INT_STS);
	//enable MMBI, single instance size 8k, total size 64k
	val = MMBI_PART_SIZE_ENUM<<MMBI_CTRL_INST_SIZE_POS;
	val |= MMBI_TOTAL_SIZE_ENUM<<MMBI_CTRL_TOTAL_SIZE_POS;
	val |= 0x1;
	ESPI_MMBI_DBUG("MMBI Control register : %x\n", val);
	aspeed_espi_mmbi_write(aspeed_espi_mmbi, val, ESPI_MMBI_CTRL);
	aspeed_espi_mmbi_write(aspeed_espi_mmbi, 0xffff, ESPI_MMBI_INT_ENL);

}

static void aspeed_espi_mmbi_h2b_rwp_int(u32 sts_mmbi,void *dev_id)
{
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi = dev_id;
	u8 h2b_rwp_sts = 0;
	u8 app_index = 0;
	unsigned long flags;

	for (app_index=0;app_index<MAX_APP_INSTANCES;app_index++) {
		if ( (h2b_rwp_sts=(sts_mmbi & 0x3)) ) {
			ESPI_MMBI_DBUG("APP #%d interrupt status : %x\n", app_index, h2b_rwp_sts);
			spin_lock_irqsave(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
			if (h2b_rwp_sts &1) {
				aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].b2h_host_rd =
						aspeed_espi_mmbi_read(aspeed_espi_mmbi, ESPI_MMBI_RWP_N(app_index));
			}
			if (h2b_rwp_sts &2) {
				aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].h2b_host_wr =
						aspeed_espi_mmbi_read(aspeed_espi_mmbi, ESPI_MMBI_RWP_N(app_index)+0x4);;
			}
			aspeed_espi_mmbi->espi_mmbi_intr_sts[app_index] = h2b_rwp_sts;
			spin_unlock_irqrestore(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
			wake_up_interruptible(&aspeed_espi_mmbi->espi_mmbi_intr_wq[app_index]);

			ESPI_MMBI_DBUG("PTR0: 0x%x\n", aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].b2h_host_rd);
			ESPI_MMBI_DBUG("PTR1: 0x%x\n", aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].h2b_host_wr );
		}
		sts_mmbi >>= 2;
	}
}

static irqreturn_t aspeed_espi_mmbi_isr(int this_irq, void *dev_id)
{
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi = dev_id;

	u32 sts = aspeed_espi_mmbi_read(aspeed_espi_mmbi, ESPI_INT_STS);
	u32 sts_mmbi = aspeed_espi_mmbi_read(aspeed_espi_mmbi, ESPI_MMBI_INT_STS);

	ESPI_MMBI_DBUG("sts : %x\n", sts);
	ESPI_MMBI_DBUG("sts_mmbi : %x\n", sts_mmbi);

	aspeed_espi_mmbi_h2b_rwp_int(sts_mmbi, dev_id);
	aspeed_espi_mmbi_write(aspeed_espi_mmbi, sts_mmbi, ESPI_MMBI_INT_STS);

	return IRQ_HANDLED;
}

static long espi_mmbi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi = dev_get_drvdata(c->this_device);
	unsigned long flags;
	struct aspeed_espi_mmbi_xfer espi_mmbi_xfer = {0};
	struct aspeed_espi_mmbi_ints espi_mmbi_ints = {0};
	struct aspeed_espi_mmbi_partition_op espi_mmbi_pop = {0};
	struct aspeed_espi_mmbi_reg espi_mmbi_reg = {0};
	int app_index = 0;
	u8	*mmbi_ptr = NULL;

	switch (cmd) {
	case ASPEED_ESPI_MMBI_IOCRX:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBI_IOCRX\n");
		if (copy_from_user(&espi_mmbi_xfer, (void*)arg, sizeof(espi_mmbi_xfer)) ) {
			ESPI_MMBI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" App: %d, Offset: 0x%x, len:0x%x \n",
				espi_mmbi_xfer.app_index,
				espi_mmbi_xfer.offset,
				espi_mmbi_xfer.len);
		mmbi_ptr = aspeed_espi_mmbi_get_mem_loc(aspeed_espi_mmbi,
				espi_mmbi_xfer.offset,
				espi_mmbi_xfer.len);

		if (mmbi_ptr){
			if (copy_to_user(espi_mmbi_xfer.xfer_buf,
					mmbi_ptr,
					espi_mmbi_xfer.len) ) {
				ESPI_MMBI_DBUG("copy_to_user  fail\n");
				ret = -EFAULT;
			}
			spin_lock_irqsave(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
//			aspeed_espi_mmbi->espi_mmbi_intr_sts[app_index] &= (~0x1);
			spin_unlock_irqrestore(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
		} else {
			ret = EINVAL;
		}
		break;
	case ASPEED_ESPI_MMBIP_IOCTX:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBIP_IOCTX\n");
		if (copy_from_user(&espi_mmbi_xfer, (void*)arg, sizeof(espi_mmbi_xfer))){
			ESPI_MMBI_DBUG("copy_to_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" App: %d, Offset: 0x%x, len:0x%x \n",
				espi_mmbi_xfer.app_index,
				espi_mmbi_xfer.offset,
				espi_mmbi_xfer.len);
		mmbi_ptr = aspeed_espi_mmbi_get_mem_loc(aspeed_espi_mmbi,
				espi_mmbi_xfer.offset,
				espi_mmbi_xfer.len);

		if (mmbi_ptr){
			if (copy_from_user(mmbi_ptr,
						espi_mmbi_xfer.xfer_buf,
						espi_mmbi_xfer.len) ) {
				ESPI_MMBI_DBUG("copy_from_user  fail\n");
				ret = -EFAULT;
			}
			spin_lock_irqsave(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
//			aspeed_espi_mmbi->espi_mmbi_intr_sts[app_index] &= (~0x2);
			spin_unlock_irqrestore(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
		} else {
			ret = EINVAL;
		}
		break;
	case ASPEED_ESPI_MMBI_IOWFI:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBI_IOWFI\n");
		if (copy_from_user(&espi_mmbi_ints, (void*)arg, sizeof(espi_mmbi_ints))) {
			ESPI_MMBI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" APP index: %d\n",espi_mmbi_ints.app_index);

		wait_event_interruptible(aspeed_espi_mmbi->espi_mmbi_intr_wq[espi_mmbi_ints.app_index],
						((aspeed_espi_mmbi->espi_mmbi_intr_sts[espi_mmbi_ints.app_index] != 0)
						|| (aspeed_espi_mmbi->espi_mmbi_reset != 0)));

		spin_lock_irqsave(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
		app_index = espi_mmbi_ints.app_index;
		espi_mmbi_ints.int_sts = aspeed_espi_mmbi->espi_mmbi_intr_sts[app_index];
		aspeed_espi_mmbi->espi_mmbi_intr_sts[app_index] &= (~0x3);
		espi_mmbi_ints.host_rwp[0] = aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].b2h_host_rd;
		espi_mmbi_ints.host_rwp[1] = aspeed_espi_mmbi->arr_mmbi_host_rwp[app_index].h2b_host_wr;
		spin_unlock_irqrestore(&aspeed_espi_mmbi->espi_mmbi_lock, flags);
		if (copy_to_user((void*)arg, &espi_mmbi_ints, sizeof(espi_mmbi_ints))) {
			ESPI_MMBI_DBUG("copy_to_user  fail\n");
			ret = -EFAULT;
		}
		break;
	case ASPEED_ESPI_MMBI_IOREAD:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBI_IOREAD\n");
		if (copy_from_user(&espi_mmbi_pop, (void*)arg, sizeof(espi_mmbi_pop))) {
			ESPI_MMBI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" Offset: 0x%x, len:0x%x \n",
						espi_mmbi_pop.offset, espi_mmbi_pop.len);
		mmbi_ptr = aspeed_espi_mmbi_get_mem_loc(aspeed_espi_mmbi,
				espi_mmbi_pop.offset,
				espi_mmbi_pop.len);
		if (mmbi_ptr){
			if (copy_to_user(espi_mmbi_pop.data_buf,
					mmbi_ptr,
					espi_mmbi_pop.len) ) {
				ESPI_MMBI_DBUG("copy_to_user  fail\n");
				ret = -EFAULT;
			}
		} else {
			ret = EINVAL;
		}
		break;
	case ASPEED_ESPI_MMBI_IOWRITE:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBI_IOWRITE\n");
		if (copy_from_user(&espi_mmbi_pop, (void*)arg, sizeof(espi_mmbi_pop))) {
			ESPI_MMBI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" Offset: 0x%x, len:0x%x \n",
				espi_mmbi_pop.offset, espi_mmbi_pop.len);
		mmbi_ptr = aspeed_espi_mmbi_get_mem_loc(aspeed_espi_mmbi,
						espi_mmbi_pop.offset,
						espi_mmbi_pop.len);
		if (mmbi_ptr){
			if (copy_from_user(mmbi_ptr,
				espi_mmbi_pop.data_buf,
				espi_mmbi_pop.len) ) {

				ESPI_MMBI_DBUG("copy_from_user  fail\n");
				ret = -EFAULT;
			}
		} else {
			ret = EINVAL;
		}
		break;
	case ASPEED_ESPI_MMBI_REG:
		ESPI_MMBI_DBUG(" ASPEED_ESPI_MMBI_REG\n");
		if (copy_from_user(&espi_mmbi_reg, (void*)arg, sizeof(espi_mmbi_reg))) {
			ESPI_MMBI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		}
		ESPI_MMBI_DBUG(" read Offset: 0x%x\n",espi_mmbi_reg.offset);
		espi_mmbi_reg.value = aspeed_espi_mmbi_read(aspeed_espi_mmbi, espi_mmbi_reg.offset);
		if (copy_to_user((void*)arg, &espi_mmbi_reg, sizeof(espi_mmbi_reg))) {
			ESPI_MMBI_DBUG("copy_to_user  fail\n");
			ret = -EFAULT;
		}
		break;
	default:
		ESPI_MMBI_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_mmbi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi = dev_get_drvdata(c->this_device);

	ESPI_MMBI_DBUG("\n");
	spin_lock(&espi_mmbi_state_lock);
	if (aspeed_espi_mmbi->is_open) {
		spin_unlock(&espi_mmbi_state_lock);
		return -EBUSY;
	}
	aspeed_espi_mmbi->is_open = true;
	spin_unlock(&espi_mmbi_state_lock);

	return 0;
}

static int espi_mmbi_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi = dev_get_drvdata(c->this_device);

	ESPI_MMBI_DBUG("\n");
	spin_lock(&espi_mmbi_state_lock);
	aspeed_espi_mmbi->is_open = false;
	spin_unlock(&espi_mmbi_state_lock);

	return 0;
}


static const struct file_operations aspeed_espi_mmbi_fops = {
	.owner				= THIS_MODULE,
	.unlocked_ioctl		= espi_mmbi_ioctl,
	.open				= espi_mmbi_open,
	.release			= espi_mmbi_release,
};


struct miscdevice aspeed_espi_mmbi_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-espi-mmbi",
	.fops = &aspeed_espi_mmbi_fops,
};

static const struct of_device_id aspeed_espi_mmbi_of_matches[] = {
	{ .compatible = "aspeed,ast2600-espi-mmbi", .data = (void *) 6, },
	{},
};

MODULE_DEVICE_TABLE(of, aspeed_espi_mmbi_of_matches);

static int aspeed_espi_mmbi_probe(struct platform_device *pdev)
{
	static struct aspeed_espi_mmbi_data *aspeed_espi_mmbi;
	const struct of_device_id *dev_id;
	int ret = 0, i = 0;
	struct regmap *scu;
	struct device_node *espi_dev_node;
	struct device_node *espi_ctrl_dev_node;
	struct platform_device *espi_ctrl_pdev;
	u32 val;

	ESPI_MMBI_DBUG("\n");
	scu = syscon_regmap_lookup_by_compatible("aspeed,ast2600-scu");
	if (IS_ERR(scu)) {
		dev_err(&pdev->dev, "failed to find 2600 SCU regmap\n");
		return PTR_ERR(scu);
	}
	regmap_read(scu, AST_SCU_HW_STRAP2, &val);
	if (val & ENABLE_LPC_MODE)//to do check if strap set SCU510 bit 6 0:espi, 1:lpc
	{
	   printk("eSPI strap not set\n");
	   return -EPFNOSUPPORT;
	}
	aspeed_espi_mmbi = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_mmbi_data), GFP_KERNEL);
	if (aspeed_espi_mmbi == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	ESPI_MMBI_DBUG("get dev_id\n");
	dev_id = of_match_device(aspeed_espi_mmbi_of_matches, &pdev->dev);
	if (!dev_id) {
		return -EINVAL;
	}

	aspeed_espi_mmbi->dev = &pdev->dev;

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		aspeed_espi_mmbi->dma_mode = 1;
	else
		aspeed_espi_mmbi->dma_mode = 0;

	ESPI_MMBI_DBUG("dma_mode=%d\n", aspeed_espi_mmbi->dma_mode );

	espi_dev_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-espi");
	aspeed_espi_mmbi->reg_base = of_iomap(espi_dev_node, 0);
	if (aspeed_espi_mmbi->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		return -ENODEV;
	}

	ESPI_MMBI_DBUG("aspeed_espi_mmbi->reg_base=0x%x\n", (u32)aspeed_espi_mmbi->reg_base );
	//check if peripheral channel is set up
	espi_ctrl_dev_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-espi-ctrl");
	espi_ctrl_pdev = of_find_device_by_node(espi_ctrl_dev_node);
	if ((!espi_ctrl_pdev) || (!espi_ctrl_pdev->dev.driver)) {
		ESPI_MMBI_DBUG("peripheral channel not set up, defer probe\n");
		return -EPROBE_DEFER;
	}

	aspeed_espi_mmbi->espi_ctrl = dev_get_drvdata(&espi_ctrl_pdev->dev);
	if (aspeed_espi_mmbi->espi_ctrl && aspeed_espi_mmbi->espi_ctrl->perif && aspeed_espi_mmbi->espi_ctrl->perif->mcyc_virt) {
		aspeed_espi_mmbi->mmbi_blk_virt = aspeed_espi_mmbi->espi_ctrl->perif->mcyc_virt;
		aspeed_espi_mmbi->mmbi_blk_phys = aspeed_espi_mmbi->espi_ctrl->perif->mcyc_taddr;
	} else {
		//dma_alloc_coherent 64KB MMBI block and program mmbi control register
		aspeed_espi_mmbi->espi_ctrl = NULL;
		aspeed_espi_mmbi->mmbi_blk_virt = dma_alloc_coherent(&pdev->dev, MMBI_TOTAL_SIZE, &aspeed_espi_mmbi->mmbi_blk_phys, GFP_KERNEL);
		ESPI_MMBI_DBUG("aspeed_espi_mmbi->mmbi_blk_virt =0x%x\n", (u32)aspeed_espi_mmbi->mmbi_blk_virt );
		ESPI_MMBI_DBUG("aspeed_espi_mmbi->mmbi_blk_phys =0x%x\n", (u32)aspeed_espi_mmbi->mmbi_blk_phys );
		if (!aspeed_espi_mmbi->mmbi_blk_virt) {
			dev_err(&pdev->dev, "cannot allocate memory cycle region\n");
			return -ENOMEM;
		}
		//aspeed_espi_mmbi_write(aspeed_espi_mmbi, (u32)aspeed_espi_mmbi->mmbi_blk_phys, ESPI_PERIF_PC_RX_SADDR);
		aspeed_espi_mmbi_write(aspeed_espi_mmbi, (u32)aspeed_espi_mmbi->mmbi_blk_phys, ESPI_PERIF_PC_RX_TADDR);
	}

	// peripheral channel memory W/R cycle enable
	regmap_read(scu, AST_SCU_DBG_CTRL2, &val);
	val &= ~FORCE_DISABLE_ESPI_TO_AHB_BRIDGE;
	regmap_write(scu, AST_SCU_DBG_CTRL2, val);
	ESPI_MMBI_DBUG("aspeed_espi_mmbi  AST_SCU_DBG_CTRL2=0x%x\n", val);
	ESPI_MMBI_DBUG("platform get irq");
	aspeed_espi_mmbi->irq = platform_get_irq(pdev, 0);
	if (aspeed_espi_mmbi->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return -ENOENT;
	}

	ESPI_MMBI_DBUG("request irq\n");
	ret = devm_request_irq(&pdev->dev, aspeed_espi_mmbi->irq, aspeed_espi_mmbi_isr,
						   0, dev_name(&pdev->dev), aspeed_espi_mmbi);
	if (ret) {
		printk("AST ESPI MMBI Unable to get IRQ");
		return -ENOENT;
	}

	spin_lock_init(&aspeed_espi_mmbi->espi_mmbi_lock);
	for (i=0;i<MAX_APP_INSTANCES;i++) {
		init_waitqueue_head(&aspeed_espi_mmbi->espi_mmbi_intr_wq[i]);
	}
	ESPI_MMBI_DBUG("mmbi control init\n");
	aspeed_espi_mmbi_ctrl_init(aspeed_espi_mmbi);

	ESPI_MMBI_DBUG("misc register\n");
	ret = misc_register(&aspeed_espi_mmbi_misc);
	if (ret) {
		printk(KERN_ERR "ESPI MMBI: failed misc_register\n");
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, aspeed_espi_mmbi);
	dev_set_drvdata(aspeed_espi_mmbi_misc.this_device, aspeed_espi_mmbi);

	printk(KERN_INFO "aspeed_espi_mmbi: driver successfully loaded.\n");

	return 0;

err_free_irq:
	free_irq(aspeed_espi_mmbi->irq, pdev);

	return ret;
}

static int aspeed_espi_mmbi_remove(struct platform_device *pdev)
{
	struct aspeed_espi_mmbi_data *aspeed_espi_mmbi;

	ESPI_MMBI_DBUG("\n");
	aspeed_espi_mmbi = platform_get_drvdata(pdev);
	if (aspeed_espi_mmbi == NULL)
		return -ENODEV;

	misc_deregister(&aspeed_espi_mmbi_misc);
	if (!aspeed_espi_mmbi->espi_ctrl)
		dma_free_coherent(&pdev->dev, MMBI_TOTAL_SIZE,aspeed_espi_mmbi->mmbi_blk_virt,aspeed_espi_mmbi->mmbi_blk_phys);
	return 0;
}

static struct platform_driver aspeed_espi_mmbi_driver = {
	.probe		= aspeed_espi_mmbi_probe,
	.remove		= aspeed_espi_mmbi_remove,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_espi_mmbi_of_matches,
	},
};

module_platform_driver(aspeed_espi_mmbi_driver);

MODULE_AUTHOR("Yunlin Tang <yunlin.tang@aspeedtech.com>");
MODULE_DESCRIPTION("AST eSPI MMBI driver");
MODULE_LICENSE("GPL");
