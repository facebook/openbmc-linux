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
#include <linux/device.h>
#include <asm/io.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/ast_spi.h>
#include <asm/arch/regs-spi.h>
#else
#include <plat/regs-spi.h>
#include <mach/ast_spi.h>
#endif

struct ast_spi_host {
	void __iomem		*reg;			
	void __iomem		*buff;			
	struct ast_spi_driver_data *spi_data;
	struct spi_master *master;
	struct spi_device *spi_dev;
	struct device *dev;
	spinlock_t				lock;
};

static inline void
ast_spi_write(struct ast_spi_host *spi, u32 val, u32 reg)
{
//	printk("ast_spi_write : val: %x , reg : %x \n",val,reg);	
	writel(val, spi->reg+ reg);
}

static inline u32
ast_spi_read(struct ast_spi_host *spi, u32 reg)
{
#if 0
	u32 val=0;
	val = readl(spi->reg + reg);
//	printk("read offset %x, value %x",reg,readl(spi->reg + reg));
	return val;
#else
	return readl(spi->reg + reg);
#endif
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int
ast_spi_setup(struct spi_device *spi)
{
	struct ast_spi_host *host = (struct ast_spi_host *)spi_master_get_devdata(spi->master);
	unsigned int         bits = spi->bits_per_word;
	u32 spi_ctrl;
	u32 spi_conf;
	u32 divisor;
	
	int err = 0;
	return err;

//	printk( "ast_spi_setup()!!!!------ , cs %d\n", spi->chip_select);
	
	host->spi_dev = spi;

	spi_ctrl = ast_spi_read(host, AST_SPI_CTRL);
	spi_conf = ast_spi_read(host, AST_SPI_CONFIG);
		
	if (spi->chip_select > spi->master->num_chipselect) {
		dev_dbg(&spi->dev,
				"setup: invalid chipselect %u (%u defined)\n",
				spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	} else {
		switch(spi->chip_select) {
			case 0:
				spi_conf |= SPI_CONF_CS0_WRITE;
				break;
			case 1:
				spi_conf |= SPI_CONF_CS1_WRITE;
				break;
			default:
				dev_dbg(&spi->dev,
						"setup: invalid chipselect %u (%u defined)\n",
						spi->chip_select, spi->master->num_chipselect);
				return -EINVAL;
				break;
		}
		ast_spi_write(host, spi_conf, AST_SPI_CONFIG);
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
		divisor = 0;
	}

//	printk("target %d , div = %d ----------------\n",spi->max_speed_hz, divisor);

	//TODO MASK first
	spi_ctrl |= (divisor << 8);
	
	if (spi->chip_select > (spi->master->num_chipselect - 1)) {
		dev_err(&spi->dev, "chipselect %d exceed the number of chipselect master supoort\n", spi->chip_select);
		return -EINVAL;
	}

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
	ast_spi_write(host, spi_ctrl, AST_SPI_CTRL);
	
	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __FUNCTION__, spi->mode, spi->bits_per_word, spi->max_speed_hz);
	return err;
}

static int ast_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct ast_spi_host *host = (struct ast_spi_host *)spi_master_get_devdata(spi->master);
	struct spi_transfer 	*xfer;
	const u8 *tx_buf;
	u8 *rx_buf;
	int i=0;
	unsigned long		flags;

	spin_lock_irqsave(&host->lock, flags);

//	writel( (readl(host->spi_data->ctrl_reg) | SPI_CMD_USER_MODE) | SPI_CE_INACTIVE,host->spi_data->ctrl_reg);
	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) & ~SPI_CE_INACTIVE, AST_SPI_CTRL);

//	writel( ~SPI_CE_INACTIVE & readl(host->spi_data->ctrl_reg),host->spi_data->ctrl_reg);

	msg->actual_length = 0;
	msg->status = 0;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		//TX ----
		tx_buf = xfer->tx_buf;
		rx_buf = xfer->rx_buf;

		if(xfer->tx_buf != 0) {
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
				writeb(tx_buf[i], host->buff);
			}
		}
		
		if(xfer->rx_buf != 0) {
			for(i=0;i<xfer->len;i++) {
				rx_buf[i] = readb(host->buff);
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
//		dev_dbg(host->dev,"old msg->actual_length %d , +len %d \n",msg->actual_length, xfer->len);
		msg->actual_length += xfer->len;
//		dev_dbg(host->dev,"new msg->actual_length %d \n",msg->actual_length);
	}



//	writel( SPI_CE_INACTIVE | readl(host->spi_data->ctrl_reg),host->spi_data->ctrl_reg);
	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) | SPI_CE_INACTIVE, AST_SPI_CTRL);

	msg->status = 0;
	msg->complete(msg->context);
	spin_unlock_irqrestore(&host->lock, flags);

	return 0;
	
}
static void ast_spi_cleanup(struct spi_device *spi)
{
        struct ast_spi_host     *host = spi_master_get_devdata(spi->master);
        unsigned long           flags;
		dev_dbg(host->dev, "ast_spi_cleanup() \n");

        spin_lock_irqsave(&host->lock, flags);
//        if (host->stay == spi) {
//                host->stay = NULL;
//                cs_deactivate(host, spi);
//        }
        spin_unlock_irqrestore(&host->lock, flags);
}

static int ast_spi_probe(struct platform_device *pdev)
{
	struct resource		*res0, *res1;
	struct ast_spi_host *host;
	struct spi_master *master;
	int err, div;

	dev_dbg(&pdev->dev, "ast_spi_probe() \n\n\n");

	master = spi_alloc_master(&pdev->dev, sizeof(struct ast_spi_host));
	if (NULL == master) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	host = spi_master_get_devdata(master);
	memset(host, 0, sizeof(struct ast_spi_host));

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res0) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM 0\n");		
		err = -ENXIO;
		goto err_no_io_res;
	}

	if (!request_mem_region(res0->start, resource_size(res0), res0->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto err_no_io_res;
	}

	host->reg = ioremap(res0->start, resource_size(res0));
	if (!host->reg) {
		err = -EIO;
		goto release_mem0;
	}

	dev_dbg(&pdev->dev, "remap phy %x, virt %x \n",(u32)res0->start, (u32)host->reg);

	res1 = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (!res1) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_IO 0\n");		
		return -ENXIO;	
	}

	if (!request_mem_region(res1->start, resource_size(res1), res1->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto err_no_io_res;
	}

	host->buff = ioremap(res1->start, resource_size(res1));

	if (!host->buff) {
		err = -EIO;
		dev_err(&pdev->dev, "cannot map buff\n");		
		goto release_mem1;
	}

	dev_dbg(&pdev->dev, "remap io phy %x, virt %x \n",(u32)res1->start, (u32)host->buff);	
	
	host->master = spi_master_get(master);
	host->master->bus_num = pdev->id;

	host->dev = &pdev->dev;

	/* Setup the state for bitbang driver */
	host->master->setup = ast_spi_setup;
	host->master->transfer = ast_spi_transfer;
	host->master->cleanup = ast_spi_cleanup;

	/* Find and claim our resources */
	host->spi_data = pdev->dev.platform_data;		
	host->master->num_chipselect = host->spi_data->num_chipselect;

	platform_set_drvdata(pdev, host);

	div = host->spi_data->get_div(50000000);

	ast_spi_write(host, ast_spi_read(host, AST_SPI_CONFIG) | SPI_CONF_WRITE_EN, AST_SPI_CONFIG);
	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) & ~SPI_CLK_DIV_MASK, AST_SPI_CTRL);	
	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) | SPI_CLK_DIV(div), AST_SPI_CTRL);		
//	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) | SPI_CE_INACTIVE | SPI_CMD_USER_MODE | SPI_LSB_FIRST_CTRL, AST_SPI_CTRL);
	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) | SPI_CE_INACTIVE | SPI_CMD_USER_MODE, AST_SPI_CTRL);
//	ast_spi_write(host, ast_spi_read(host, AST_SPI_CTRL) | SPI_CE_INACTIVE, AST_SPI_CTRL);

	/* Register our spi controller */
	err = spi_register_master(host->master);
	if (err) {
			dev_err(&pdev->dev, "failed to register SPI master\n");
			goto err_register;
	}

	dev_dbg(&pdev->dev, "ast_spi_probe() return \n\n\n");

	return 0;

err_register:
	spi_master_put(host->master);

release_mem1:	
	release_mem_region(res1->start, res1->end - res1->start + 1);	

release_mem0:
	release_mem_region(res0->start, res0->end - res0->start + 1);


err_no_io_res:
	kfree(master);
	kfree(host);
	
err_nomem:
	return err;
	
}

static int 
ast_spi_remove(struct platform_device *pdev)
{
	struct resource 	*res0, *res1;
	struct ast_spi_host *host = platform_get_drvdata(pdev);

	dev_dbg(host->dev, "ast_spi_remove()\n");

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
ast_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int
ast_spi_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define ast_spi_suspend NULL
#define ast_spi_resume NULL
#endif

static struct platform_driver ast_spi_driver = {
	.probe = ast_spi_probe,
	.remove = ast_spi_remove,
	.suspend = ast_spi_suspend,
	.resume = ast_spi_resume,
	.driver = {
		.name = "ast-spi",
		.owner = THIS_MODULE,
	},
};

static int __init
ast_spi_init(void)
{
	return  platform_driver_register(&ast_spi_driver);
}

static void __exit
ast_spi_exit(void)
{
	platform_driver_unregister(&ast_spi_driver);
}

subsys_initcall(ast_spi_init);
//module_init(ast_spi_init);
module_exit(ast_spi_exit);

MODULE_DESCRIPTION("AST SPI Driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");
