// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Intel Corporation

#include <linux/bitfield.h>
#include <linux/mfd/core.h>
#include <linux/mfd/intel-peci-client.h>
#include <linux/module.h>
#include <linux/peci.h>
#include <linux/of_device.h>

#define CPU_ID_MODEL_MASK      GENMASK(7, 4)
#define CPU_ID_FAMILY_MASK     GENMASK(11, 8)
#define CPU_ID_EXT_MODEL_MASK  GENMASK(19, 16)
#define CPU_ID_EXT_FAMILY_MASK GENMASK(27, 20)

#define LOWER_NIBBLE_MASK      GENMASK(3, 0)
#define UPPER_NIBBLE_MASK      GENMASK(7, 4)
#define LOWER_BYTE_MASK        GENMASK(7, 0)
#define UPPER_BYTE_MASK        GENMASK(16, 8)

enum cpu_gens {
	CPU_GEN_HSX = 0, /* Haswell Xeon */
	CPU_GEN_BRX,     /* Broadwell Xeon */
	CPU_GEN_SKX,     /* Skylake Xeon */
};

static struct mfd_cell peci_functions[] = {
	{ .name = "peci-cputemp", },
	{ .name = "peci-dimmtemp", },
	/* TODO: Add additional PECI sideband functions into here */
};

static const struct cpu_gen_info cpu_gen_info_table[] = {
	[CPU_GEN_HSX] = {
		.family        = 6, /* Family code */
		.model         = INTEL_FAM6_HASWELL_X,
		.core_max      = CORE_MAX_ON_HSX,
		.chan_rank_max = CHAN_RANK_MAX_ON_HSX,
		.dimm_idx_max  = DIMM_IDX_MAX_ON_HSX },
	[CPU_GEN_BRX] = {
		.family        = 6, /* Family code */
		.model         = INTEL_FAM6_BROADWELL_X,
		.core_max      = CORE_MAX_ON_BDX,
		.chan_rank_max = CHAN_RANK_MAX_ON_BDX,
		.dimm_idx_max  = DIMM_IDX_MAX_ON_BDX },
	[CPU_GEN_SKX] = {
		.family        = 6, /* Family code */
		.model         = INTEL_FAM6_SKYLAKE_X,
		.core_max      = CORE_MAX_ON_SKX,
		.chan_rank_max = CHAN_RANK_MAX_ON_SKX,
		.dimm_idx_max  = DIMM_IDX_MAX_ON_SKX },
};

static int peci_client_get_cpu_gen_info(struct peci_client_manager *priv)
{
	u32 cpu_id;
	u16 family;
	u8 model;
	int rc;
	int i;

	rc = peci_get_cpu_id(priv->client->adapter, priv->client->addr,
			     &cpu_id);
	if (rc)
		return rc;

	family = FIELD_PREP(LOWER_BYTE_MASK,
			    FIELD_GET(CPU_ID_FAMILY_MASK, cpu_id)) |
		 FIELD_PREP(UPPER_BYTE_MASK,
			    FIELD_GET(CPU_ID_EXT_FAMILY_MASK, cpu_id));
	model = FIELD_PREP(LOWER_NIBBLE_MASK,
			   FIELD_GET(CPU_ID_MODEL_MASK, cpu_id)) |
		FIELD_PREP(UPPER_NIBBLE_MASK,
			   FIELD_GET(CPU_ID_EXT_MODEL_MASK, cpu_id));

	for (i = 0; i < ARRAY_SIZE(cpu_gen_info_table); i++) {
		const struct cpu_gen_info *cpu_info = &cpu_gen_info_table[i];

		if (family == cpu_info->family && model == cpu_info->model) {
			priv->gen_info = cpu_info;
			break;
		}
	}

	if (!priv->gen_info) {
		dev_err(priv->dev, "Can't support this CPU: 0x%x\n", cpu_id);
		rc = -ENODEV;
	}

	return rc;
}

static int peci_client_probe(struct peci_client *client)
{
	struct device *dev = &client->dev;
	struct peci_client_manager *priv;
	uint cpu_no;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->client = client;
	priv->dev = dev;
	cpu_no = client->addr - PECI_BASE_ADDR;

	ret = peci_client_get_cpu_gen_info(priv);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(priv->dev, cpu_no, peci_functions,
				   ARRAY_SIZE(peci_functions), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to register child devices: %d\n",
			ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id peci_client_of_table[] = {
	{ .compatible = "intel,peci-client" },
	{ }
};
MODULE_DEVICE_TABLE(of, peci_client_of_table);
#endif

static const struct peci_device_id peci_client_ids[] = {
	{ .name = "peci-client" },
	{ }
};
MODULE_DEVICE_TABLE(peci, peci_client_ids);

static struct peci_driver peci_client_driver = {
	.probe    = peci_client_probe,
	.id_table = peci_client_ids,
	.driver   = {
		.name           = "peci-client",
		.of_match_table = of_match_ptr(peci_client_of_table),
	},
};
module_peci_driver(peci_client_driver);

MODULE_AUTHOR("Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>");
MODULE_DESCRIPTION("PECI client driver");
MODULE_LICENSE("GPL v2");
