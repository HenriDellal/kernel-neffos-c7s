/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/uaccess.h>
#include <video/sprd_mm.h>
#include <video/sprd_isp_r6p10v2.h>
#include "../isp_drv.h"

static int isp_k_y_delay_block(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_ydelay_info ydelay_info;

	memset(&ydelay_info, 0x00, sizeof(ydelay_info));
	ret = copy_from_user((void *)&ydelay_info,
		param->property_param, sizeof(ydelay_info));
	if (ret != 0) {
		pr_err("isp_k_uvd_block: copy error, ret = 0x%x\n",
			(unsigned int)ret);
		return -1;
	}
	ISP_REG_MWR(idx, ISP_YDELAY_PARAM, BIT_0, ydelay_info.bypass);
	ISP_REG_MWR(idx, ISP_YDELAY_PARAM + ISP_CH1_ADDR_OFFSET, BIT_0,
		ydelay_info.bypass);
	if (ydelay_info.bypass)
		return 0;

	val = ydelay_info.step & 0x1FFF;
	ISP_REG_WR(idx, ISP_YDELAY_STEP, val);
	ISP_REG_WR(idx, ISP_YDELAY_STEP + ISP_CH1_ADDR_OFFSET, val);
	return ret;
}

int isp_k_cfg_ydelay(struct isp_io_param *param, enum isp_id idx)
{
	int ret = 0;

	if (!param) {
		pr_err("isp_k_cfg_rgb2y: param is null error\n");
		return -1;
	}

	if (!param->property_param) {
		pr_err("isp_k_cfg_rgb2y: property_param is null error\n");
		return -1;
	}

	switch (param->property) {
	case ISP_PRO_Y_DELAY_BLOCK:
		ret = isp_k_y_delay_block(param, idx);
		break;
	default:
		pr_err("isp_k_cfg_rgb2y: fail cmd id %d, not supported\n",
			param->property);
		break;
	}

	return ret;
}

