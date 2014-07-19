/* --------------------------------------------------------------------
 *  VIRTHUB
 * -------------------------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

static struct resource ast_virthub_resource[] = {
  [0] = {
    .start = AST_USB20_BASE,
    .end = AST_USB20_BASE + SZ_1K,
    .flags = IORESOURCE_MEM,
  },
  [1] = {
    .start = IRQ_USB20_HUB,
    .end = IRQ_USB20_HUB,
    .flags = IORESOURCE_IRQ,
  },
};

static u64 ast_virthub_dma_mask = 0xfffffff8UL;

static struct platform_device ast_virthub_device = {
  .name	= "aspeed_udc",
  .id = 0,
  .dev = {
    .dma_mask = &ast_virthub_dma_mask,
    .coherent_dma_mask = 0xffffffff,
  },
  .resource = ast_virthub_resource,
  .num_resources = ARRAY_SIZE(ast_virthub_resource),
};

void __init ast_add_device_virthub(void)
{
  ast_scu_multi_func_usb20_host_hub(0);
  ast_scu_init_vhub();
  printk("virtual hub inited?\n");
  platform_device_register(&ast_virthub_device);
}
