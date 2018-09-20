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
 * Power Management for Fixed regulator on suspend and early-suspend time
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#endif

#define MAX_COUNT 32
struct fic_pm_data {
	int regulator_sleep_count;
	struct regulator *pm_regulator[MAX_COUNT];
	int regulator_earlysuspend_count;
	struct regulator *pm_earlysuspend_regulator[MAX_COUNT];
};

struct fic_pm_pdata {
};


static struct fic_pm_data pm_data;

static int pm_resume_early(void)
{
	int i;
	int ret = 0;
	for(i=0; i<pm_data.regulator_sleep_count; i++){

		if (pm_data.pm_regulator[i]!=(struct regulator *)NULL)
			ret = regulator_enable(pm_data.pm_regulator[i]);
	}
	return ret;
}

static int pm_suspend_late(void)
{
	int i;
	int ret = 0;
	for(i=pm_data.regulator_sleep_count-1; i>=0; i--){

		if (pm_data.pm_regulator[i]!=(struct regulator *)NULL)
			ret = regulator_disable(pm_data.pm_regulator[i]);
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pm_earlysuspend_on_power(void)
{
	int i;
	int ret = 0;
	for(i=0; i<pm_data.regulator_earlysuspend_count; i++){

		if (pm_data.pm_earlysuspend_regulator[i]!=(struct regulator *)NULL)
			ret = regulator_enable(pm_data.pm_earlysuspend_regulator[i]);
	}
	return;
}

static void pm_earlysuspend_off_power(void)
{
	int i;
	int ret = 0;
	for(i=pm_data.regulator_earlysuspend_count-1; i>=0; i--){

		if (pm_data.pm_earlysuspend_regulator[i]!=(struct regulator *)NULL)
			ret = regulator_disable(pm_data.pm_earlysuspend_regulator[i]);
	}
	return;
}

static void pm_earlysuspend_resume(struct early_suspend *h)
{
	pm_earlysuspend_on_power();
}

static void pm_earlysuspend_suspend(struct early_suspend *h)
{
	pm_earlysuspend_off_power();
}

struct early_suspend fic_pm_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_POST_DISABLE_FB,
	.suspend = pm_earlysuspend_suspend,
	.resume = pm_earlysuspend_resume,
};
#endif

static int fic_power_event(struct notifier_block *this,
							unsigned long event, void *dummy)
{

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* going to suspend, don't reset chip */
		break;
	case PM_POST_SUSPEND:
		/* System is resume, can reset chip */
		break;
	case PM_RESTORE_PREPARE:
		/* Going to restore a saved image */
		break;
	case PM_POST_RESTORE:
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block fic_power_notifier = {
	.notifier_call = fic_power_event,
};

#if defined(CONFIG_OF)
static const struct of_device_id fic_pm_dt_ids[] = {
	{ .compatible = "fic,fic_pm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fic_pm_dt_ids);

static struct fic_pm_pdata *fic_pm_of_populate_pdata(
		struct device *dev)
{
	struct device_node *of_node = dev->of_node;
	struct fic_pm_pdata *pdata = dev->platform_data;

	if (!of_node || pdata)
		return pdata;

	pdata = devm_kzalloc(dev, sizeof(struct fic_pm_pdata),
				GFP_KERNEL);
	if (!pdata)
		return pdata;

	return pdata;
}
#endif
static int fic_pm_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct fic_pm_pdata *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int i,regulator_count;
	int ret;

	for (i=0; i<MAX_COUNT; i++){
		pm_data.pm_regulator[i] = (struct regulator *)NULL;
		pm_data.pm_earlysuspend_regulator[i] = (struct regulator *)NULL;
	}
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pdata = fic_pm_of_populate_pdata(&pdev->dev);
		if (!pdata)
			return -EINVAL;
	}
	regulator_count = of_property_count_strings(np, "regulator-names");
	pm_data.regulator_sleep_count = regulator_count;

	for (i=0; i<regulator_count; i++){
		const char *regulator_name;

		ret = of_property_read_string_index(np, "regulator-names",
						    i,
						    &regulator_name);
		dev_dbg(dev, "regulator[%d]:%s\n", i, regulator_name);
		pm_data.pm_regulator[i] = devm_regulator_get(dev, regulator_name);
		if (!IS_ERR(pm_data.pm_regulator[i])) {
			ret = regulator_enable(pm_data.pm_regulator[i]);
			if (ret) {
				dev_err(dev, "set io voltage failed\n");
				return ret;
			} else {
				dev_dbg(dev, "set io voltage ok\n");
			}
		} else {
			pm_data.pm_regulator[i] = NULL;
			dev_warn(dev, "cannot get io voltage\n");
		}
	}

	regulator_count = of_property_count_strings(np, "regulator-earlysuspend-names");
	pm_data.regulator_earlysuspend_count = regulator_count;

	for (i=0; i<regulator_count; i++){
		const char *regulator_name;

		ret = of_property_read_string_index(np, "regulator-earlysuspend-names",
						    i,
						    &regulator_name);
		dev_dbg(dev, "regulator[%d]:%s\n", i, regulator_name);
		pm_data.pm_earlysuspend_regulator[i] = devm_regulator_get(dev, regulator_name);
		if (!IS_ERR(pm_data.pm_earlysuspend_regulator[i])) {
			ret = regulator_enable(pm_data.pm_earlysuspend_regulator[i]);
			if (ret) {
				dev_err(dev, "set io voltage failed\n");
				return ret;
			} else {
				dev_dbg(dev, "set io voltage ok\n");
			}
		} else {
			pm_data.pm_earlysuspend_regulator[i] = NULL;
			dev_dbg(dev, "cannot get io voltage\n");
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&fic_pm_earlysuspend);
#endif
	rc = register_pm_notifier(&fic_power_notifier);
	if (rc)
		goto error_check_func;

	printk(KERN_INFO "fic_pm driver success loaded\n");
	return 0;

error_check_func:
	return rc;
}

static int  fic_pm_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void fic_pm_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	pm_earlysuspend_off_power();
#endif
	pm_suspend_late();
	return;
}

#ifdef CONFIG_PM
static int fic_pm_suspend(struct device *dev){
	return 0;
}

static int fic_pm_resume(struct device *dev){
	return 0;
}

static int fic_pm_poweroff(struct device *dev){
	pm_suspend_late();
	return 0;
}

static int fic_pm_suspend_late(struct device *dev){
	pm_suspend_late();
	return 0;
}

static int fic_pm_resume_early(struct device *dev){
	pm_resume_early();
	return 0;
}


static const struct dev_pm_ops fic_pm_ops = {
	.suspend = fic_pm_suspend,
	.resume = fic_pm_resume,
	.poweroff = fic_pm_poweroff,
	.suspend_late = fic_pm_suspend_late,
	.resume_early = fic_pm_resume_early,
};
#endif

static struct platform_driver fic_pm_driver = {
	.probe	= fic_pm_probe,
	.remove	= fic_pm_remove,
	.shutdown = fic_pm_shutdown,
	.driver = {
		.name	= "fic_pm",
		.owner	= THIS_MODULE,
		.of_match_table = fic_pm_dt_ids,
#ifdef CONFIG_PM
		.pm = &fic_pm_ops,
#endif
	},
};

module_platform_driver(fic_pm_driver)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FIC power management driver");
MODULE_VERSION("1.00");
