/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sharkl3/glb.h>
#include <linux/regmap.h>

#include "sprd_dsi.h"

static struct dsi_glb_context {
	struct clk *clk_aon_apb_disp_eb;
	struct regmap *aon_apb;
} dsi_glb_ctx;


static int dsi_glb_parse_dt(struct dsi_context *ctx,
				struct device_node *np)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	glb_ctx->clk_aon_apb_disp_eb =
		of_clk_get_by_name(np, "clk_aon_apb_disp_eb");
	if (IS_ERR(glb_ctx->clk_aon_apb_disp_eb)) {
		pr_warn("read clk_aon_apb_disp_eb failed\n");
		glb_ctx->clk_aon_apb_disp_eb = NULL;
	}

	glb_ctx->aon_apb = syscon_regmap_lookup_by_phandle(np,
					    "sprd,syscon-aon-apb");
	if (IS_ERR(glb_ctx->aon_apb)) {
		pr_err("error: parse syscon-aon-apb failed\n");
		glb_ctx->aon_apb = NULL;
	}

	return 0;
}

static void dsi_glb_enable(struct dsi_context *ctx)
{
	int ret;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	ret = clk_prepare_enable(glb_ctx->clk_aon_apb_disp_eb);
	if (ret)
		pr_err("enable clk_aon_apb_disp_eb failed!\n");
}

static void dsi_glb_disable(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	clk_disable_unprepare(glb_ctx->clk_aon_apb_disp_eb);
}

static void dsi_reset(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx;

	regmap_update_bits(glb_ctx->aon_apb,
		REG_AON_APB_JVL_DUMMY, BIT(2), BIT(2));
	udelay(10);
	regmap_update_bits(glb_ctx->aon_apb,
		REG_AON_APB_JVL_DUMMY, BIT(2), (u32)(~BIT(2)));
}

static struct dsi_glb_ops dsi_glb_ops = {
	.parse_dt = dsi_glb_parse_dt,
	.reset = dsi_reset,
	.enable = dsi_glb_enable,
	.disable = dsi_glb_disable,
};

static struct ops_entry entry = {
	.ver = "sharkl3",
	.ops = &dsi_glb_ops,
};

static int __init dsi_glb_register(void)
{
	return dsi_glb_ops_register(&entry);
}

subsys_initcall(dsi_glb_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharkl3 dsi global APB regs low-level config");
