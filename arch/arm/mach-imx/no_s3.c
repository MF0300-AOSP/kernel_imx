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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>

static struct wake_lock wakelock;

#if defined(CONFIG_OF)
static const struct of_device_id no_s3_dt_ids[] = {
	{ .compatible = "fic,no_s3", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, no_s3_dt_ids);

#endif
static int no_s3_probe(struct platform_device *pdev)
{

	android_wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND,
		       "no_s3");
	android_wake_lock(&wakelock);
	printk(KERN_INFO "no_s3 driver success loaded\n");
	return 0;
}

static int  no_s3_remove(struct platform_device *pdev)
{
	android_wake_unlock(&wakelock);
	android_wake_lock_destroy(&wakelock);
	return 0;
}


static struct platform_driver no_s3_driver = {
	.probe	= no_s3_probe,
	.remove	= no_s3_remove,
	.driver = {
		.name	= "no_s3",
		.owner	= THIS_MODULE,
		.of_match_table = no_s3_dt_ids,
	},
};

module_platform_driver(no_s3_driver)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FIC wake lock to stop suspend  driver");
MODULE_VERSION("1.00");
