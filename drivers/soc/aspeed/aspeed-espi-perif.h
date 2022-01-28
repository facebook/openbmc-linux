// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 * Author: Chia-Wei Wang <chiawei_wang@aspeedtech.com>
 *
 * This header includes the data strcuture and the handler
 * for eSPI peripheral channel. It is part of Aspeed eSPI
 * slave driver implementation
 */
#ifndef _ASPEED_ESPI_PERIF_H_
#define _ASPEED_ESPI_PERIF_H_

#define PERIF_MDEV_NAME		"aspeed-espi-peripheral"
#define PERIF_MEMCYC_UNLOCK_KEY	0xfedc756e
#define PERIF_MEMCYC_SIZE_MIN	0x10000

struct aspeed_espi_perif_dma {
	void *pc_tx_virt;
	dma_addr_t pc_tx_addr;
	void *pc_rx_virt;
	dma_addr_t pc_rx_addr;
	void *np_tx_virt;
	dma_addr_t np_tx_addr;
};

struct aspeed_espi_perif {
	void *mcyc_virt;
	phys_addr_t mcyc_saddr;
	phys_addr_t mcyc_taddr;
	uint32_t mcyc_size;
	uint32_t mcyc_mask;

	uint32_t dma_mode;
	struct aspeed_espi_perif_dma dma;

	uint32_t rx_ready;
	wait_queue_head_t wq;

	spinlock_t rx_lock;
	struct mutex pc_tx_lock;
	struct mutex np_tx_lock;

	struct miscdevice mdev;
	struct aspeed_espi_ctrl *ctrl;
};

static long aspeed_espi_perif_pc_get_rx(struct file *fp,
					struct aspeed_espi_ioc *ioc,
					struct aspeed_espi_perif *espi_perif)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	uint32_t pkt_len;
	struct espi_comm_hdr *hdr;
	unsigned long flags;
	struct aspeed_espi_ctrl *espi_ctrl = espi_perif->ctrl;

	if (!espi_perif->rx_ready) {
		if (fp->f_flags & O_NONBLOCK)
			return -ENODATA;

		rc = wait_event_interruptible(espi_perif->wq, espi_perif->rx_ready);
		if (rc == -ERESTARTSYS)
			return -EINTR;
	}

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_ctrl->map, ESPI_PERIF_PC_RX_CTRL, &reg);
	cyc = (reg & ESPI_PERIF_PC_RX_CTRL_CYC_MASK) >> ESPI_PERIF_PC_RX_CTRL_CYC_SHIFT;
	tag = (reg & ESPI_PERIF_PC_RX_CTRL_TAG_MASK) >> ESPI_PERIF_PC_RX_CTRL_TAG_SHIFT;
	len = (reg & ESPI_PERIF_PC_RX_CTRL_LEN_MASK) >> ESPI_PERIF_PC_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_PERIF_MSG:
		pkt_len = len + sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_MSG_D:
		pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			  sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_SUC_CMPLT_D_MIDDLE:
	case ESPI_PERIF_SUC_CMPLT_D_FIRST:
	case ESPI_PERIF_SUC_CMPLT_D_LAST:
	case ESPI_PERIF_SUC_CMPLT_D_ONLY:
		pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			  sizeof(struct espi_perif_cmplt);
		break;
	case ESPI_PERIF_SUC_CMPLT:
	case ESPI_PERIF_UNSUC_CMPLT:
		pkt_len = len + sizeof(struct espi_perif_cmplt);
		break;
	default:
		return -EFAULT;
	}

	if (ioc->pkt_len < pkt_len)
	    return -EINVAL;

	pkt = vmalloc(pkt_len);
	if (!pkt)
	    return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (espi_perif->dma_mode) {
		memcpy(hdr + 1, espi_perif->dma.pc_rx_virt,
		       pkt_len - sizeof(*hdr));
	}
	else {
		for (i = sizeof(*hdr); i < pkt_len; ++i) {
			regmap_read(espi_ctrl->map,
				    ESPI_PERIF_PC_RX_PORT, &reg);
			pkt[i] = reg & 0xff;
		}
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len))
		return -EFAULT;

	spin_lock_irqsave(&espi_perif->rx_lock, flags);

	regmap_write_bits(espi_ctrl->map, ESPI_PERIF_PC_RX_CTRL,
			  ESPI_PERIF_PC_RX_CTRL_PEND_SERV,
			  ESPI_PERIF_PC_RX_CTRL_PEND_SERV);

	espi_perif->rx_ready = 0;

	spin_unlock_irqrestore(&espi_perif->rx_lock, flags);

	return pkt_len;
}

static long aspeed_espi_perif_pc_put_tx(struct file *fp,
					struct aspeed_espi_ioc *ioc,
					struct aspeed_espi_perif *espi_perif)
{
	int i, rc = 0;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	struct espi_comm_hdr *hdr;
	struct aspeed_espi_ctrl *espi_ctrl = espi_perif->ctrl;

	if (!mutex_trylock(&espi_perif->pc_tx_lock))
		return -EAGAIN;

	regmap_read(espi_ctrl->map, ESPI_PERIF_PC_TX_CTRL, &reg);
	if (reg & ESPI_PERIF_PC_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (espi_perif->dma_mode) {
		memcpy(espi_perif->dma.pc_tx_virt, hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	}
	else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			regmap_write(espi_ctrl->map,
				     ESPI_PERIF_PC_TX_PORT, pkt[i]);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_PERIF_PC_TX_CTRL_CYC_SHIFT) & ESPI_PERIF_PC_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_PERIF_PC_TX_CTRL_TAG_SHIFT) & ESPI_PERIF_PC_TX_CTRL_TAG_MASK)
		| ((len << ESPI_PERIF_PC_TX_CTRL_LEN_SHIFT) & ESPI_PERIF_PC_TX_CTRL_LEN_MASK)
		| ESPI_PERIF_PC_TX_CTRL_TRIGGER;

	regmap_write(espi_ctrl->map, ESPI_PERIF_PC_TX_CTRL, reg);

free_n_out:
	vfree(pkt);

unlock_n_out:
	mutex_unlock(&espi_perif->pc_tx_lock);

	return rc;
}

static long aspeed_espi_perif_np_put_tx(struct file *fp,
					struct aspeed_espi_ioc *ioc,
					struct aspeed_espi_perif *espi_perif)
{
	int i, rc = 0;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	struct espi_comm_hdr *hdr;
	struct aspeed_espi_ctrl *espi_ctrl = espi_perif->ctrl;

	if (!mutex_trylock(&espi_perif->np_tx_lock))
		return -EAGAIN;

	regmap_read(espi_ctrl->map, ESPI_PERIF_NP_TX_CTRL, &reg);
	if (reg & ESPI_PERIF_NP_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (espi_perif->dma_mode) {
		memcpy(espi_perif->dma.np_tx_virt, hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	}
	else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			regmap_write(espi_ctrl->map,
				     ESPI_PERIF_NP_TX_PORT, pkt[i]);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_PERIF_NP_TX_CTRL_CYC_SHIFT) & ESPI_PERIF_NP_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_PERIF_NP_TX_CTRL_TAG_SHIFT) & ESPI_PERIF_NP_TX_CTRL_TAG_MASK)
		| ((len << ESPI_PERIF_NP_TX_CTRL_LEN_SHIFT) & ESPI_PERIF_NP_TX_CTRL_LEN_MASK)
		| ESPI_PERIF_NP_TX_CTRL_TRIGGER;

	regmap_write(espi_ctrl->map, ESPI_PERIF_NP_TX_CTRL, reg);

free_n_out:
	vfree(pkt);

unlock_n_out:
	mutex_unlock(&espi_perif->np_tx_lock);

	return rc;

}

static long aspeed_espi_perif_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	struct aspeed_espi_ioc ioc;
	struct aspeed_espi_perif *espi_perif = container_of(
			fp->private_data,
			struct aspeed_espi_perif,
			mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
		return -EFAULT;

	if (ioc.pkt_len > ESPI_PKT_LEN_MAX)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_PERIF_PC_GET_RX:
		return aspeed_espi_perif_pc_get_rx(fp, &ioc, espi_perif);
	case ASPEED_ESPI_PERIF_PC_PUT_TX:
		return aspeed_espi_perif_pc_put_tx(fp, &ioc, espi_perif);
	case ASPEED_ESPI_PERIF_NP_PUT_TX:
		return aspeed_espi_perif_np_put_tx(fp, &ioc, espi_perif);
	};

	return -EINVAL;
}

static int aspeed_espi_perif_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct aspeed_espi_perif *espi_perif = container_of(
			fp->private_data,
			struct aspeed_espi_perif,
			mdev);
	unsigned long vm_size = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vm_size) > espi_perif->mcyc_size)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (espi_perif->mcyc_taddr >> PAGE_SHIFT) + vma->vm_pgoff,
			    vm_size, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_espi_perif_fops = {
	.owner = THIS_MODULE,
	.mmap = aspeed_espi_perif_mmap,
	.unlocked_ioctl = aspeed_espi_perif_ioctl,
};

static void aspeed_espi_perif_event(uint32_t sts, struct aspeed_espi_perif *espi_perif)
{
	unsigned long flags;

	if (sts & ESPI_INT_STS_PERIF_PC_RX_CMPLT) {
		spin_lock_irqsave(&espi_perif->rx_lock, flags);
		espi_perif->rx_ready = 1;
		spin_unlock_irqrestore(&espi_perif->rx_lock, flags);

		wake_up_interruptible(&espi_perif->wq);
	}
}

static void aspeed_espi_perif_enable(struct aspeed_espi_perif *espi_perif)
{
	struct aspeed_espi_perif_dma *dma = &espi_perif->dma;
	struct aspeed_espi_ctrl *espi_ctrl = espi_perif->ctrl;

	if (espi_ctrl->version == ESPI_AST2500) {
		regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_MASK, PERIF_MEMCYC_UNLOCK_KEY);
		regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_MASK, espi_perif->mcyc_mask);
	}
	else {
		regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_MASK,
			     espi_perif->mcyc_mask | ESPI_PERIF_PC_RX_MASK_CFG_WP);
		regmap_update_bits(espi_ctrl->map, ESPI_CTRL2,
				   ESPI_CTRL2_MEMCYC_RD_DIS | ESPI_CTRL2_MEMCYC_WR_DIS, 0);
	}

	regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_SADDR, espi_perif->mcyc_saddr);
	regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_TADDR, espi_perif->mcyc_taddr);

	if (espi_perif->dma_mode) {
	    regmap_write(espi_ctrl->map, ESPI_PERIF_PC_RX_DMA, dma->pc_rx_addr);
	    regmap_write(espi_ctrl->map, ESPI_PERIF_PC_TX_DMA, dma->pc_tx_addr);
	    regmap_write(espi_ctrl->map, ESPI_PERIF_NP_TX_DMA, dma->np_tx_addr);

	    regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
			       ESPI_CTRL_PERIF_NP_TX_DMA_EN |
			       ESPI_CTRL_PERIF_PC_TX_DMA_EN |
			       ESPI_CTRL_PERIF_PC_RX_DMA_EN,
			       ESPI_CTRL_PERIF_NP_TX_DMA_EN |
			       ESPI_CTRL_PERIF_PC_TX_DMA_EN |
			       ESPI_CTRL_PERIF_PC_RX_DMA_EN);
	}

	regmap_write(espi_ctrl->map, ESPI_INT_STS,
		     ESPI_INT_STS_PERIF_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_INT_EN,
			   ESPI_INT_EN_PERIF_BITS,
			   ESPI_INT_EN_PERIF_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
			   ESPI_CTRL_PERIF_SW_RDY,
			   ESPI_CTRL_PERIF_SW_RDY);
}

static void *aspeed_espi_perif_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl)
{
	int rc;
	struct aspeed_espi_perif *espi_perif;
	struct aspeed_espi_perif_dma *dma;

	espi_perif = devm_kzalloc(dev, sizeof(*espi_perif), GFP_KERNEL);
	if (!espi_perif)
		return ERR_PTR(-ENOMEM);

	espi_perif->ctrl = espi_ctrl;

	init_waitqueue_head(&espi_perif->wq);

	spin_lock_init(&espi_perif->rx_lock);

	mutex_init(&espi_perif->pc_tx_lock);
	mutex_init(&espi_perif->np_tx_lock);

	rc = of_property_read_u32(dev->of_node, "perif,memcyc-src-addr", &espi_perif->mcyc_saddr);
	if (rc) {
		dev_err(dev, "cannot get Host source address for memroy cycle\n");
		return ERR_PTR(-ENODEV);
	}

	rc = of_property_read_u32(dev->of_node, "perif,memcyc-size", &espi_perif->mcyc_size);
	if (rc) {
		dev_err(dev, "cannot get size for memory cycle\n");
		return ERR_PTR(-ENODEV);
	}

	if (espi_perif->mcyc_size < PERIF_MEMCYC_SIZE_MIN)
		espi_perif->mcyc_size = PERIF_MEMCYC_SIZE_MIN;
	else
		espi_perif->mcyc_size = roundup_pow_of_two(espi_perif->mcyc_size);

	espi_perif->mcyc_mask = ~(espi_perif->mcyc_size - 1);

	espi_perif->mcyc_virt = dma_alloc_coherent(dev, espi_perif->mcyc_size,
						  &espi_perif->mcyc_taddr, GFP_KERNEL);
	if (!espi_perif->mcyc_virt) {
	    dev_err(dev, "cannot allocate memory cycle region\n");
	    return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_bool(dev->of_node, "perif,dma-mode")) {
		dma = &espi_perif->dma;

		dma->pc_tx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
						     &dma->pc_tx_addr, GFP_KERNEL);
		if (!dma->pc_tx_virt) {
			dev_err(dev, "cannot allocate posted TX DMA buffer\n");
			return ERR_PTR(-ENOMEM);
		}

		dma->pc_rx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
						     &dma->pc_rx_addr, GFP_KERNEL);
		if (!dma->pc_rx_virt) {
			dev_err(dev, "cannot allocate posted RX DMA buffer\n");
			return ERR_PTR(-ENOMEM);
		}

		dma->np_tx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
				&dma->np_tx_addr, GFP_KERNEL);
		if (!dma->np_tx_virt) {
			dev_err(dev, "cannot allocate non-posted TX DMA buffer\n");
			return ERR_PTR(-ENOMEM);
		}

		espi_perif->dma_mode = 1;
	}

	espi_perif->mdev.parent = dev;
	espi_perif->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_perif->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", PERIF_MDEV_NAME);
	espi_perif->mdev.fops = &aspeed_espi_perif_fops;
	rc = misc_register(&espi_perif->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return ERR_PTR(rc);
	}

	aspeed_espi_perif_enable(espi_perif);

	return espi_perif;
}

static void aspeed_espi_perif_free(struct device *dev, struct aspeed_espi_perif *espi_perif)
{
	struct aspeed_espi_perif_dma *dma = &espi_perif->dma;

	if (espi_perif->mcyc_virt)
		dma_free_coherent(dev, espi_perif->mcyc_size,
				  espi_perif->mcyc_virt,
				  espi_perif->mcyc_taddr);

	if (espi_perif->dma_mode) {
		dma_free_coherent(dev, PAGE_SIZE, dma->pc_tx_virt,
				  dma->pc_tx_addr);
		dma_free_coherent(dev, PAGE_SIZE, dma->pc_rx_virt,
				  dma->pc_rx_addr);
		dma_free_coherent(dev, PAGE_SIZE, dma->np_tx_virt,
				  dma->np_tx_addr);
	}

	mutex_destroy(&espi_perif->pc_tx_lock);
	mutex_destroy(&espi_perif->np_tx_lock);

	devm_kfree(dev, espi_perif);
}

#endif
