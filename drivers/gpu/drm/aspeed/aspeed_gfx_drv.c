// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018 IBM Corporation

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_atomic_helper.h>

#include "aspeed_gfx.h"

static const struct drm_mode_config_funcs aspeed_gfx_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void aspeed_gfx_setup_mode_config(struct drm_device *drm)
{
	drm_mode_config_init(drm);

	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = 800;
	drm->mode_config.max_height = 600;
	drm->mode_config.funcs = &aspeed_gfx_mode_config_funcs;
}

static int aspeed_gfx_load(struct drm_device *drm)
{
	struct platform_device *pdev = to_platform_device(drm->dev);
	struct aspeed_gfx *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	drm->dev_private = priv;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(drm->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->scu = syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
	if (IS_ERR(priv->scu)) {
		dev_err(&pdev->dev, "failed to find SCU regmap\n");
		return PTR_ERR(priv->scu);
	}

	ret = of_reserved_mem_device_init(drm->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to initialize reserved mem: %d\n", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "failed to set DMA mask: %d\n", ret);
		return ret;
	}

	priv->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller device tree entry");
		return PTR_ERR(priv->rst);
	}
	reset_control_deassert(priv->rst);

	priv->clk = devm_clk_get(drm->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev,
			"missing or invalid clk device tree entry");
		return PTR_ERR(priv->clk);
	}
	clk_prepare_enable(priv->clk);

	/* Sanitize control registers */
	writel(0, priv->base + CRT_CTRL1);
	writel(0, priv->base + CRT_CTRL2);

	aspeed_gfx_setup_mode_config(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to initialise vblank\n");
		return ret;
	}

	ret = aspeed_gfx_create_output(drm);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to create outputs\n");
		return ret;
	}

	ret = aspeed_gfx_create_pipe(drm);
	if (ret < 0) {
		dev_err(drm->dev, "Cannot setup simple display pipe\n");
		return ret;
	}

	ret = drm_irq_install(drm, platform_get_irq(pdev, 0));
	if (ret < 0) {
		dev_err(drm->dev, "Failed to install IRQ handler\n");
		return ret;
	}

	drm_mode_config_reset(drm);

	priv->fbdev = drm_fbdev_cma_init(drm, 32, 1);
	if (IS_ERR(priv->fbdev)) {
		ret = PTR_ERR(priv->fbdev);
		dev_err(drm->dev, "Failed to init FB CMA area\n");
		goto err_cma;
	}

	return 0;

err_cma:
	drm_irq_uninstall(drm);
	return ret;
}

static void aspeed_gfx_unload(struct drm_device *drm)
{
	struct aspeed_gfx *priv = drm->dev_private;

	if (priv->fbdev)
		drm_fbdev_cma_fini(priv->fbdev);

	drm_kms_helper_poll_fini(drm);
	drm_mode_config_cleanup(drm);

	drm_irq_uninstall(drm);

	drm->dev_private = NULL;
}

static void aspeed_gfx_lastclose(struct drm_device *drm)
{
	struct aspeed_gfx *priv = drm->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static irqreturn_t aspeed_gfx_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct aspeed_gfx *priv = drm->dev_private;
	u32 reg;

	reg = readl(priv->base + CRT_CTRL1);

	if (reg & CRT_CTRL_VERTICAL_INTR_STS) {
		drm_crtc_handle_vblank(&priv->pipe.crtc);
		writel(reg, priv->base + CRT_CTRL1);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver aspeed_gfx_driver = {
	.driver_features        = DRIVER_GEM | DRIVER_MODESET |
				DRIVER_PRIME | DRIVER_ATOMIC |
				DRIVER_HAVE_IRQ,
	.lastclose              = aspeed_gfx_lastclose,
	.irq_handler            = aspeed_gfx_irq_handler,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops             = &drm_gem_cma_vm_ops,
	.dumb_create            = drm_gem_cma_dumb_create,
	.prime_handle_to_fd     = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle     = drm_gem_prime_fd_to_handle,
	.gem_prime_export       = drm_gem_prime_export,
	.gem_prime_import       = drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap         = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap       = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap         = drm_gem_cma_prime_mmap,
	.fops = &fops,
	.name = "aspeed-gfx-drm",
	.desc = "ASPEED GFX DRM",
	.date = "20180319",
	.major = 1,
	.minor = 0,

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = aspeed_gfx_debugfs_init,
#endif
};

static const struct of_device_id aspeed_gfx_match[] = {
	{ .compatible = "aspeed,ast2400-gfx" },
	{ .compatible = "aspeed,ast2500-gfx" },
	{ }
};

static int aspeed_gfx_probe(struct platform_device *pdev)
{
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(&aspeed_gfx_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	ret = aspeed_gfx_load(drm);
	if (ret)
		goto err_free;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	return 0;

err_unload:
	aspeed_gfx_unload(drm);
err_free:
	drm_dev_put(drm);

	return ret;
}

static int aspeed_gfx_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	aspeed_gfx_unload(drm);
	drm_dev_put(drm);

	return 0;
}

static struct platform_driver aspeed_gfx_platform_driver = {
	.probe		= aspeed_gfx_probe,
	.remove		= aspeed_gfx_remove,
	.driver = {
		.name = "aspeed_gfx",
		.of_match_table = aspeed_gfx_match,
	},
};

module_platform_driver(aspeed_gfx_platform_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("ASPEED BMC DRM/KMS driver");
MODULE_LICENSE("GPL");
