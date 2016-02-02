/********************************************************************************
* File Name     : driver/spi/ast-spi.c
* Author        : Ryan Chen
* Description   : ASPEED SPI host driver
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
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
*	1. 2012/10/20 Ryan Chen create this file
*	1. 2013/01/05 Ryan Chen modify
*
********************************************************************************/
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <mach/ast_spi.h>
#include <plat/regs-spi.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>

//IN FMC SPI always control offset 0x00

struct fmc_spi_host {
	void __iomem		*reg;
	u32		buff[5];
	struct ast_spi_driver_data *spi_data;
	struct spi_master *master;
	struct spi_device *spi_dev;
	struct device *dev;
	spinlock_t				lock;
};

static inline void
fmc_spi_write(struct fmc_spi_host *host, u32 val, u32 reg)
{
//	printk("write : val: %x , offset : %x \n",val, reg);
	writel(val, host->reg + reg + (host->spi_dev->chip_select*0x4) + 0x10);

}

static inline u32
fmc_spi_read(struct fmc_spi_host *host, u32 reg)
{
	return readl(host->reg + reg + (host->spi_dev->chip_select*0x4) + 0x10);
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int
fmc_spi_setup(struct spi_device *spi)
{
	struct fmc_spi_host *host = (struct fmc_spi_host *)spi_master_get_devdata(spi->master);
	unsigned int         bits = spi->bits_per_word;
	u32 spi_ctrl;
	u32 divisor;

	int err = 0;
	dev_dbg(host->dev, "fmc_spi_setup() ======================>>\n");

	host->spi_dev = spi;

	spi_ctrl = fmc_spi_read(host, 0x00);
//	printk("trl : %x \n",spi_ctrl);
	spi_ctrl &= ~SPI_DUAL_DATA;
	if (spi->chip_select > (spi->master->num_chipselect - 1)) {
			dev_dbg(&spi->dev,
					"setup: invalid chipselect %u (%u defined)\n",
					spi->chip_select, spi->master->num_chipselect);
			return -EINVAL;
	} else {
		switch(spi->chip_select) {
			case 0:
				writel(((readl(host->reg) | FMC_SET_WRITE_CS(0)) & FMC_MASK_TYPE_CS(0)) | FMC_SET_TYPE_SPI_CS(0), host->reg);
				break;
			case 1:
				ast_scu_multi_func_romcs(1);
				writel(((readl(host->reg) | FMC_SET_WRITE_CS(1)) & FMC_MASK_TYPE_CS(1)) | FMC_SET_TYPE_SPI_CS(1), host->reg);
				break;
			case 2:
				ast_scu_multi_func_romcs(2);
				writel(((readl(host->reg) | FMC_SET_WRITE_CS(2)) & FMC_MASK_TYPE_CS(2)) | FMC_SET_TYPE_SPI_CS(2), host->reg);
				break;
			case 3:
				ast_scu_multi_func_romcs(3);
				writel(((readl(host->reg) | FMC_SET_WRITE_CS(3)) & FMC_MASK_TYPE_CS(3)) | FMC_SET_TYPE_SPI_CS(3), host->reg);
				break;
			case 4:
				ast_scu_multi_func_romcs(4);
				writel(((readl(host->reg) | FMC_SET_WRITE_CS(4)) & FMC_MASK_TYPE_CS(4)) | FMC_SET_TYPE_SPI_CS(4), host->reg);
				break;
			default:
				dev_dbg(&spi->dev,
						"setup: invalid chipselect %u (%u defined)\n",
						spi->chip_select, spi->master->num_chipselect);
				return -EINVAL;
				break;
		}
	}


	if (bits == 0)
			bits = 8;

	if (bits < 8 || bits > 16) {
			dev_dbg(&spi->dev,
					"setup: invalid bits_per_word %u (8 to 16)\n",
					bits);
			return -EINVAL;
	}

	if (spi->mode & ~MODEBITS) {
			dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
					spi->mode & ~MODEBITS);
			return -EINVAL;
	}

	/* see notes above re chipselect */
	if((spi->chip_select == 0) && (spi->mode & SPI_CS_HIGH)) {
			dev_dbg(&spi->dev, "setup: can't be active-high\n");
			return -EINVAL;
	}

	/*
	 * Pre-new_1 chips start out at half the peripheral
	 * bus speed.
	 */

	if (spi->max_speed_hz) {
		/* Set the SPI slaves select and characteristic control register */
		divisor = host->spi_data->get_div(spi->max_speed_hz);
	} else {
		/* speed zero means "as slow as possible" */
		divisor = 15;
	}

	spi_ctrl &= ~SPI_CLK_DIV_MASK;
//	printk("set div %x \n",divisor);
	//TODO MASK first
	spi_ctrl |= SPI_CLK_DIV(divisor);

#if 0
	if (SPI_CPHA & spi->mode)
		cpha = SPI_CPHA_1;
	else
		cpha = SPI_CPHA_0;
#endif

//	if (SPI_CPOL & spi->mode)
//		spi_ctrl |= SPI_CPOL_1;
//	else
//		spi_ctrl &= ~SPI_CPOL_1;

	//ISSUE : ast spi ctrl couldn't use mode 3, so fix mode 0
	spi_ctrl &= ~SPI_CPOL_1;


	if (SPI_LSB_FIRST & spi->mode)
		spi_ctrl |= SPI_LSB_FIRST_CTRL;
	else
		spi_ctrl &= ~SPI_LSB_FIRST_CTRL;


	/* Configure SPI controller */
	fmc_spi_write(host, spi_ctrl, 0x00);

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __FUNCTION__, spi->mode, spi->bits_per_word, spi->max_speed_hz);
	return err;
}

static int fmc_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct fmc_spi_host *host = (struct fmc_spi_host *)spi_master_get_devdata(spi->master);
	struct spi_transfer 	*xfer;
	const u8 *tx_buf;
	u8 *rx_buf;
	unsigned long		flags;

	int i=0,j=0;

	dev_dbg(host->dev, "new message %p submitted for %s \n",
					msg, dev_name(&spi->dev));

	spin_lock_irqsave(&host->lock, flags);
//	writel( (readl(host->spi_data->ctrl_reg) | SPI_CMD_USER_MODE) | SPI_CE_INACTIVE,host->spi_data->ctrl_reg);
	fmc_spi_write(host, fmc_spi_read(host, 0x00) | SPI_CMD_USER_MODE, 0x00);
	msg->actual_length = 0;
	msg->status = 0;

//	writel( ~SPI_CE_INACTIVE & readl(host->spi_data->ctrl_reg),host->spi_data->ctrl_reg);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			dev_dbg(host->dev,
					"xfer[%d] %p: width %d, len %u, tx %p/%08x, rx %p/%08x\n",
					j, xfer,
					xfer->bits_per_word, xfer->len,
					xfer->tx_buf, xfer->tx_dma,
					xfer->rx_buf, xfer->rx_dma);

		tx_buf = xfer->tx_buf;
		rx_buf = xfer->rx_buf;


		if(tx_buf != 0) {
#if 0
			printk("tx : ");
			if(xfer->len > 10) {
				for(i=0;i<10;i++)
					printk("%x ",tx_buf[i]);
			} else {
				for(i=0;i<xfer->len;i++)
					printk("%x ",tx_buf[i]);
			}
			printk("\n");
#endif
			for(i=0;i<xfer->len;i++) {
				writeb(tx_buf[i], host->buff[host->spi_dev->chip_select]);
			}
		}
		//Issue need clarify
		udelay(1);
		if(rx_buf != 0) {
			for(i=0;i<xfer->len;i++) {
				rx_buf[i] = readb(host->buff[host->spi_dev->chip_select]);
			}
#if 0
			printk("rx : ");
			if(xfer->len > 10) {
				for(i=0;i<10;i++)
					printk(" %x",rx_buf[i]);
			} else {
				for(i=0;i<xfer->len;i++)
					printk(" %x",rx_buf[i]);
			}
			printk("\n");
#endif
		}
		dev_dbg(host->dev,"old msg->actual_length %d , +len %d \n",msg->actual_length, xfer->len);
		msg->actual_length += xfer->len;
		dev_dbg(host->dev,"new msg->actual_length %d \n",msg->actual_length);
//		j++;

	}

//	writel( SPI_CE_INACTIVE | readl(host->spi_data->ctrl_reg),host->spi_data->ctrl_reg);
	fmc_spi_write(host, (fmc_spi_read(host, 0x00) & ~SPI_CMD_USER_MODE), 0x00);
	msg->status = 0;

	msg->complete(msg->context);

//	spin_unlock(&host->lock);
	spin_unlock_irqrestore(&host->lock, flags);




	return 0;

}
static void fmc_spi_cleanup(struct spi_device *spi)
{
        struct fmc_spi_host     *host = spi_master_get_devdata(spi->master);
        unsigned long           flags;
		dev_dbg(host->dev, "fmc_spi_cleanup() \n");

        spin_lock_irqsave(&host->lock, flags);
//        if (host->stay == spi) {
//                host->stay = NULL;
//                cs_deactivate(host, spi);
//        }
        spin_unlock_irqrestore(&host->lock, flags);
}

static int fmc_spi_probe(struct platform_device *pdev)
{
	struct resource		*res0, *res1=0;
	struct fmc_spi_host *host;
	struct spi_master *master;
	int cs_num = 0;
	int err;

	dev_dbg(&pdev->dev, "fmc_spi_probe() \n\n\n");

	master = spi_alloc_master(&pdev->dev, sizeof(struct fmc_spi_host));
	if (NULL == master) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	host = spi_master_get_devdata(master);
	memset(host, 0, sizeof(struct fmc_spi_host));

	/* Find and claim our resources */
	host->spi_data = pdev->dev.platform_data;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res0) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM 0\n");
		err = -ENXIO;
		goto err_no_io_res;
	}

	host->reg = ioremap(res0->start, resource_size(res0));
	if (!host->reg) {
		dev_err(&pdev->dev, "cannot remap register\n");
		err = -EIO;
		goto release_mem;
	}

	dev_dbg(&pdev->dev, "remap phy %x, virt %x \n",(u32)res0->start, (u32)host->reg);

	for(cs_num = 0; cs_num < host->spi_data->num_chipselect ; cs_num++) {
		res1 = platform_get_resource(pdev, IORESOURCE_BUS, cs_num);
		if (!res1) {
			dev_err(&pdev->dev, "cannot get IORESOURCE_IO 0\n");
			return -ENXIO;
		}

		host->buff[cs_num] = (u32) ioremap(res1->start, resource_size(res1));
		if (!host->buff[cs_num]) {
			dev_err(&pdev->dev, "cannot remap buffer \n");
			err = -EIO;
			goto release_mem;
		}

		dev_dbg(&pdev->dev, "remap io phy %x, virt %x \n",(u32)res1->start, (u32)host->buff[cs_num]);
	}

	host->master = spi_master_get(master);
	host->master->bus_num = pdev->id;
	host->master->num_chipselect = host->spi_data->num_chipselect;
	host->dev = &pdev->dev;

	/* Setup the state for bitbang driver */
	host->master->setup = fmc_spi_setup;
	host->master->transfer = fmc_spi_transfer;
	host->master->cleanup = fmc_spi_cleanup;

	platform_set_drvdata(pdev, host);

	/* Register our spi controller */
	err = spi_register_master(host->master);
	if (err) {
			dev_err(&pdev->dev, "failed to register SPI master\n");
			goto err_register;
	}

	dev_dbg(&pdev->dev, "fmc_spi_probe() return \n\n\n");

	return 0;

err_register:
	spi_master_put(host->master);
	iounmap(host->reg);
	for(cs_num = 0; cs_num < host->spi_data->num_chipselect ; cs_num++) {
		iounmap((void *)host->buff[cs_num]);
	}

release_mem:
	release_mem_region(res0->start, res0->end - res0->start + 1);
	release_mem_region(res1->start, res1->end - res1->start + 1);

err_no_io_res:
	kfree(master);
	kfree(host);

err_nomem:
	return err;

}

static int
fmc_spi_remove(struct platform_device *pdev)
{
	struct resource 	*res0, *res1;
	struct fmc_spi_host *host = platform_get_drvdata(pdev);

	dev_dbg(host->dev, "fmc_spi_remove()\n");

	if (!host)
		return -1;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	release_mem_region(res0->start, res0->end - res0->start + 1);
	release_mem_region(res1->start, res1->end - res1->start + 1);
	iounmap(host->reg);
	iounmap(host->buff);

	platform_set_drvdata(pdev, NULL);
	spi_unregister_master(host->master);
	spi_master_put(host->master);
	return 0;
}

#ifdef CONFIG_PM
static int
fmc_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int
fmc_spi_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define fmc_spi_suspend NULL
#define fmc_spi_resume NULL
#endif

static struct platform_driver fmc_spi_driver = {
	.probe = fmc_spi_probe,
	.remove = fmc_spi_remove,
	.suspend = fmc_spi_suspend,
	.resume = fmc_spi_resume,
	.driver = {
		.name = "fmc-spi",
		.owner = THIS_MODULE,
	},
};

static int __init
fmc_spi_init(void)
{
	return  platform_driver_register(&fmc_spi_driver);
}

static void __exit
fmc_spi_exit(void)
{
	platform_driver_unregister(&fmc_spi_driver);
}

arch_initcall(fmc_spi_init);
//module_init(fmc_spi_init);
module_exit(fmc_spi_exit);

MODULE_DESCRIPTION("FMC SPI Driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");
