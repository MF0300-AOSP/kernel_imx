
#include <osl.h>

#ifdef CUSTOMER_HW

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#ifdef CONFIG_MACH_ODROID_4210
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

#include <plat/sdhci.h>
#include <plat/devs.h>	// modifed plat-samsung/dev-hsmmcX.c EXPORT_SYMBOL(s3c_device_hsmmcx) added

#define	sdmmc_channel	s3c_device_hsmmc0
#endif

struct wifi_platform_data {
	int (*set_power)(bool val);
	int (*set_carddetect)(bool val);
	void *(*mem_prealloc)(int section, unsigned long size);
	int (*get_mac_addr)(unsigned char *buf);
	void *(*get_country_code)(char *ccode);
};

struct resource dhd_wlan_resources = {0};
struct wifi_platform_data dhd_wlan_control = {0};

#ifdef CUSTOMER_OOB
#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))
#define TF9300_WIFI_HOST_WAKE		IMX_GPIO_NR(3, 16)
#define TF9300_WIFI_RST			IMX_GPIO_NR(3, 18)

void wifi_power(int nSwitch) 
{
	int ret;

	ret = gpio_request_one(TF9300_WIFI_RST, GPIOF_OUT_INIT_LOW, "wifi-reset");
	if (ret) {
		pr_err("failed to get GPIO TF9300_WIFI_RST: %d\n",
			ret);
		return;
	}

	// wifi and bt pins setting
	printk("**%s:%d** set wifi gpio (%d)\n", __func__, __LINE__, nSwitch);
	gpio_set_value_cansleep(TF9300_WIFI_RST, 0);
	mdelay(100); 
	if (nSwitch) {
		gpio_set_value_cansleep(TF9300_WIFI_RST, 1);
		msleep(100);
	}

	gpio_free(TF9300_WIFI_RST);
}

uint bcm_wlan_get_oob_irq(void)
{
	uint host_oob_irq = 0;

#ifdef CONFIG_MACH_ODROID_4210
	printk("GPIO(WL_HOST_WAKE) = EXYNOS4_GPX0(7) = %d\n", EXYNOS4_GPX0(7));
	host_oob_irq = gpio_to_irq(EXYNOS4_GPX0(7));
	gpio_direction_input(EXYNOS4_GPX0(7));
#endif

	host_oob_irq = gpio_to_irq(TF9300_WIFI_HOST_WAKE);
	printk("host_oob_irq: %d \r\n", host_oob_irq);

	return host_oob_irq;
}

uint bcm_wlan_get_oob_irq_flags(void)
{
	uint host_oob_irq_flags = 0;

//#ifdef CONFIG_MACH_ODROID_4210
	host_oob_irq_flags = (IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE) & IRQF_TRIGGER_MASK;
//#endif
	printk("host_oob_irq_flags=%d\n", host_oob_irq_flags);

	return host_oob_irq_flags;
}
#endif

extern void wifi_power(int nSwitch);
int bcm_wlan_set_power(bool on)
{
	int err = 0;

	if(of_machine_is_compatible("fsl,imx6q-tf9300"))
	{
		if(on)
             		wifi_power(1);
		else
			wifi_power(0);
	}

	return err;
}

int bcm_wlan_set_carddetect(bool present)
{
	int err = 0;

	if (present) {
		printk("======== Card detection to detect SDIO card! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 1);
#endif
	} else {
		printk("======== Card detection to remove SDIO card! ========\n");
#ifdef CONFIG_MACH_ODROID_4210
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 0);
#endif
	}

	return err;
}

#ifdef CONFIG_DHD_USE_STATIC_BUF
extern void *bcmdhd_mem_prealloc(int section, unsigned long size);
void* bcm_wlan_prealloc(int section, unsigned long size)
{
	void *alloc_ptr = NULL;
	alloc_ptr = bcmdhd_mem_prealloc(section, size);
	if (alloc_ptr) {
		printk("success alloc section %d, size %ld\n", section, size);
		if (size != 0L)
			bzero(alloc_ptr, size);
		return alloc_ptr;
	}
	printk("can't alloc section %d\n", section);
	return NULL;
}
#endif

int bcm_wlan_set_plat_data(void) {
	printk("======== %s ========\n", __FUNCTION__);
	dhd_wlan_control.set_power = bcm_wlan_set_power;
	dhd_wlan_control.set_carddetect = bcm_wlan_set_carddetect;
#ifdef CONFIG_DHD_USE_STATIC_BUF
	dhd_wlan_control.mem_prealloc = bcm_wlan_prealloc;
#endif
	return 0;
}

#endif /* CUSTOMER_HW */
