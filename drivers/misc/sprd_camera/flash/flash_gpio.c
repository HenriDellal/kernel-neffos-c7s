/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include "flash_drv.h"

#define FLASH_GPIO_MAX 2

/* Structure Definitions */

struct flash_driver_data {
	int gpio_tab[SPRD_FLASH_NUM_MAX][FLASH_GPIO_MAX];
	spinlock_t slock;
};

/* Static Variables Definitions */

static const char *const flash_gpio_names[FLASH_GPIO_MAX] = {
	"torch-en-gpios",
	"flash-en-gpios",
};

static int torch_en_gpios,flash_en_gpios;

/* API Function Implementation */

static int sprd_flash_gpio_open_torch(void *drvd, uint8_t idx)
{
	int ret = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
	
	pr_info("sprd_flash_gpio_open_torch E");

	if (!drv_data)
		return -EFAULT;

	ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_ON);
	return ret;
}


static int sprd_flash_gpio_close_torch(void *drvd, uint8_t idx)
{
	int ret = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	pr_info("sprd_flash_gpio_close_torch E");

	if (!drv_data)
		return -EFAULT;

	ret = gpio_direction_output(torch_en_gpios, SPRD_FLASH_OFF);
	return ret;
}

static int sprd_flash_gpio_open_preflash(void *drvd, uint8_t idx)
{
	int ret = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	pr_info("sprd_flash_gpio_open_preflash E");

	if (!drv_data)
		return -EFAULT;
	
	ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_ON);
	return ret;
	
}

static int sprd_flash_gpio_close_preflash(void *drvd, uint8_t idx)
{
	int ret = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	pr_info("sprd_flash_gpio_close_preflash E");

	if (!drv_data)
		return -EFAULT;


	ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_OFF);
	return ret;
	
}

static int sprd_flash_gpio_open_highlight(void *drvd, uint8_t idx)
{
	int ret, i= 0;
	unsigned long flag;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

	pr_info("sprd_flash_gpio_open_highlight E");

	spin_lock_irqsave(&drv_data->slock, flag);

	if (!drv_data)
		return -EFAULT;
		ret=gpio_direction_output(flash_en_gpios, SPRD_FLASH_ON);
	if (ret)				
		return ret;
	for(i=0;i<9;i++){
		ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_OFF);
		//udelay(10);
		ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_ON);
		//udelay(10);
	}

	spin_unlock_irqrestore(&drv_data->slock, flag);
	return ret;
	
}

static int sprd_flash_gpio_close_highlight(void *drvd, uint8_t idx)
{
	int ret = 0;
	struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
	
	pr_info("sprd_flash_gpio_close_highlight E");

	if (!drv_data)
		return -EFAULT;

	ret=gpio_direction_output(torch_en_gpios, SPRD_FLASH_OFF);
	if (ret)				
		return ret;		
	ret=gpio_direction_output(flash_en_gpios, SPRD_FLASH_OFF);
	return ret;
	
}

static const struct sprd_flash_driver_ops flash_gpio_ops = {
	.open_torch = sprd_flash_gpio_open_torch,
	.close_torch = sprd_flash_gpio_close_torch,
	.open_preflash = sprd_flash_gpio_open_preflash,
	.close_preflash = sprd_flash_gpio_close_preflash,
	.open_highlight = sprd_flash_gpio_open_highlight,
	.close_highlight = sprd_flash_gpio_close_highlight,
};

static int sprd_flash_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	u32 gpio_node = 0;
	struct flash_driver_data *drv_data = NULL;
	int gpio[FLASH_GPIO_MAX];

	if (IS_ERR(pdev))
		return -EINVAL;

	pr_info("sprd_flash_gpio_probe E\n");

	ret = of_property_read_u32(pdev->dev.of_node, 
				"sprd,gpio", &gpio_node);
	if (ret) {
		pr_err("no gpio flash\n");
		return -EINVAL;
	}

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

		for (i = 0; i < FLASH_GPIO_MAX; i++) {
			gpio[i] = of_get_named_gpio(pdev->dev.of_node,
						       flash_gpio_names[i], 0);
			if (gpio_is_valid(gpio[i])) {
				ret = devm_gpio_request(&pdev->dev,
							gpio[i],
							flash_gpio_names[i]);
				if (ret)
					goto exit;
			}
		}

	torch_en_gpios = gpio[0];
	   
	flash_en_gpios = gpio[1];

	ret = sprd_flash_register(&flash_gpio_ops, drv_data, gpio_node);

	spin_lock_init(&drv_data->slock);

exit:
	
	pr_info("sprd_flash_gpio_probe X\n");
	return ret;
}

static int sprd_flash_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static void sprd_flash_gpio_shutdown(struct platform_device *pdev)
{
	pr_info("sprd_flash_gpio_shutdown E\n");
	gpio_direction_output(torch_en_gpios, SPRD_FLASH_OFF);
	gpio_direction_output(flash_en_gpios, SPRD_FLASH_OFF);
}

static const struct of_device_id sprd_flash_gpio_of_match[] = {
	{ .compatible = "sprd,flash_gpio", },
	{},
};

static struct platform_driver sprd_flash_gpio_driver = {
	.probe = sprd_flash_gpio_probe,
	.remove = sprd_flash_gpio_remove,
	.driver = {
		.name = "flash-gpio",
		.of_match_table = of_match_ptr(sprd_flash_gpio_of_match),
	},
	.shutdown = sprd_flash_gpio_shutdown,
};

module_platform_driver(sprd_flash_gpio_driver);
