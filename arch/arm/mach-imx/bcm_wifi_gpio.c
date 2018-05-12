/* Copyright (c) 2009-2015, FIC Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
/*
 * WiFi AmPak(bcm4330) Power Switch Module
 * controls power to external WiFi device
 * with interface to power management device
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

static int wifi_power_gpio;
static int wifi_host_wakeup_gpio;
DEFINE_MUTEX(wifi_mutex);
static bool wifi_gpio_state = false;

#if defined(CONFIG_OF)
static const struct of_device_id bcm_wifi_gpio_dt_ids[] = {
	{ .compatible = "broadcom,bcm_wifi_gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm_wifi_gpio_dt_ids);
#endif

void wifi_power(int nSwitch)
{
	int ret;

	ret = gpio_request_one(wifi_power_gpio, GPIOF_OUT_INIT_LOW, "wifi-power");
	if (ret) {
		pr_err("failed to get GPIO wifi_power_gpio: %d\n",
			ret);
		return;
	}

	// wifi and bt pins setting
	printk("**%s:%d** set wifi gpio (%d)\n", __func__, __LINE__, nSwitch);
	mutex_lock(&wifi_mutex);
	gpio_set_value_cansleep(wifi_power_gpio, 0);
	wifi_gpio_state = false;
	mdelay(100);
	if (nSwitch) {
		gpio_set_value_cansleep(wifi_power_gpio, 1);
		wifi_gpio_state = true;
		msleep(100);
	}
	mutex_unlock(&wifi_mutex);

	gpio_free(wifi_power_gpio);
}
EXPORT_SYMBOL(wifi_power);

int get_wifi_host_wakeup_gpio(void)
{
	return wifi_host_wakeup_gpio;
}
EXPORT_SYMBOL(get_wifi_host_wakeup_gpio);

int get_wifi_power_gpio(void)
{
	int state;
	mutex_lock(&wifi_mutex);

	state = (int)wifi_gpio_state;
	mutex_unlock(&wifi_mutex);
	return state;
}

EXPORT_SYMBOL(get_wifi_power_gpio);

static int bcm_wifi_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	wifi_host_wakeup_gpio = of_get_named_gpio(np, "wifi-host-wakeup-gpio", 0);
	if (!gpio_is_valid(wifi_host_wakeup_gpio)) {
		dev_err(&pdev->dev, "unable to get bt-power-gpios\n");
		goto error_request_gpio;
	}
	wifi_power_gpio = of_get_named_gpio(np, "wifi-power-gpio", 0);
	if (!gpio_is_valid(wifi_power_gpio)) {
		dev_err(&pdev->dev, "unable to get wifi-power-gpios\n");
		goto error_request_gpio;
	}
	printk(KERN_INFO "bcm_wifi_gpio driver success loaded\n");
	return 0;

error_request_gpio:
	return -1;
}

static int bcm_wifi_gpio_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver bcm_wifi_gpio_driver = {
	.probe	= bcm_wifi_gpio_probe,
	.remove	= bcm_wifi_gpio_remove,
	.driver = {
		.name	= "bcm_wifi_gpio",
		.owner	= THIS_MODULE,
		.of_match_table = bcm_wifi_gpio_dt_ids,
	},
};

module_platform_driver(bcm_wifi_gpio_driver)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BCM WiFi power control driver");
MODULE_VERSION("1.00");
