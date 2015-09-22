/*
 * Copyright 2012, 2014 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/switch.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>

#include "../codecs/rt5631.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_rt5631_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
};

#define AMP_MAX 3

struct imx_priv {
	int hp_gpio;
	int hp_active_low;
    struct snd_soc_codec *codec;
	struct platform_device *pdev;
	struct snd_card *snd_card;
	int clk_gpio;
	int clk_active_low;
	int amp_spk_gpio[AMP_MAX];
	int amp_spk_active_low[AMP_MAX];
	int amp_total;
    int hp_irq;
};

static struct imx_priv card_priv;

static int sample_rate = 44100;
static unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static unsigned int spk_swap = 0;
static int hpjack_status_check(void)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	char *envp[3], *buf;
	int hp_status;

	if (!gpio_is_valid(priv->hp_gpio))
		return 0;

	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;

	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdev->dev, "%s kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (hp_status != priv->hp_active_low)
        snprintf(buf, 32, "STATE=%d", 2);
	else
		snprintf(buf, 32, "STATE=%d", 0);

	envp[0] = "NAME=headphone";
	envp[1] = buf;
	envp[2] = NULL;
    kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	enable_irq(priv->hp_irq);

	return;
}
static DECLARE_DELAYED_WORK(hp_event, hpjack_status_check);

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;

	if (gpio_is_valid(priv->clk_gpio)) 
		if (!codec_dai->active)
			gpio_set_value(priv->clk_gpio, !priv->clk_active_low);
	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;

	if (gpio_is_valid(priv->clk_gpio)) 
		if (!codec_dai->active)  /* why ? not active disable clk */
			gpio_set_value(priv->clk_gpio, !!priv->clk_active_low);

	return;
}

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai; 
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct imx_priv *priv = &card_priv;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_rt5631_data *data = snd_soc_card_get_drvdata(card);
	struct device *dev = &priv->pdev->dev;
	u32 dai_format;
    unsigned int pll_out;
	int ret = 0;

    unsigned int channels = params_channels(params);
    sample_rate = params_rate(params);
    sample_format = params_format(params);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

    /* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	if (spk_swap)
		dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
			SND_SOC_DAIFMT_CBM_CFM;
	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret)
		return ret;
    
    if (sample_format == SNDRV_PCM_FORMAT_S24_LE)
        pll_out = 192 * sample_rate;
    else
        pll_out = 256 * sample_rate;

    ret = snd_soc_dai_set_pll(codec_dai, RT5631_PLL1, 
            RT5631_PLL_S_MCLK, data->clk_frequency, pll_out);
    if (ret) {
        pr_err("Failed to start FLL: %d\n", ret);
        return ret;
    }   

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5631_SCLK1, pll_out, SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("failed to set SYSCLK: %d\n", ret);
		return ret;
    }

	return 0;
}

static struct snd_soc_ops imx_hifi_ops = {
	.startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown,
	.hw_params = imx_hifi_hw_params,
};

static int rt5631_jack_func;
static int rt5631_spk_func;
static int rt5631_line_in_func;

static int imx_rt5631_gpio_init(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct imx_priv *priv = &card_priv;

	priv->codec = codec;

    return 0;
}

static irqreturn_t imx_headphone_detect_handler(int irq, void *data)
{
	printk(KERN_WARNING"imx_headphone_detect_handler\n ");
	disable_irq_nosync(irq);
	schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int hp_status;

	if (!gpio_is_valid(priv->hp_gpio)) {
		strcpy(buf, "no detect gpio connected\n");
		return strlen(buf);
	}

	/* Check if headphone is plugged in */
	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;

	if (hp_status != priv->hp_active_low)
		strcpy(buf, "headphone\n");
	else
		strcpy(buf, "speaker\n");

	return strlen(buf);
}
static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum rt5631_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

static int rt5631_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = rt5631_jack_func;
	return 0;
}

static int rt5631_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk(KERN_WARNING "%s:ucontrol->value.enumerated.item[0]:%d\n", __func__, ucontrol->value.enumerated.item[0]);
	if (rt5631_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	rt5631_jack_func = ucontrol->value.enumerated.item[0];
	if (rt5631_jack_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int rt5631_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = rt5631_spk_func;
	return 0;
}

static int rt5631_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk(KERN_WARNING "%s:ucontrol->value.enumerated.item[0]:%d\n", __func__, ucontrol->value.enumerated.item[0]);

	if (rt5631_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	return 0; //FIC
	rt5631_spk_func = ucontrol->value.enumerated.item[0];
	if (rt5631_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int rt5631_get_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = rt5631_line_in_func;
	return 0;
}

static int rt5631_set_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (rt5631_line_in_func == ucontrol->value.enumerated.item[0])
		return 0;

	rt5631_line_in_func = ucontrol->value.enumerated.item[0];
	if (rt5631_line_in_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Line In Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
                         struct snd_kcontrol *kcontrol, int event)
{
        struct imx_priv *priv = &card_priv;
        struct platform_device *pdev = priv->pdev;
	int i;

	for (i=0; i<priv->amp_total; i++){
		if (gpio_is_valid(priv->amp_spk_gpio[i])){
	
		        if (SND_SOC_DAPM_EVENT_ON(event))
				gpio_set_value(priv->amp_spk_gpio[i], !priv->amp_spk_active_low[i]);
 	       	else
				gpio_set_value(priv->amp_spk_gpio[i], !!priv->amp_spk_active_low[i]);
		}
	}
        return 0;
}


static const struct snd_soc_dapm_widget imx_rt5631_dapm_widgets[] = {
    SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", spk_amp_event),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static const struct snd_kcontrol_new rt5631_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", rt5631_enum[0], rt5631_get_jack,
		     rt5631_set_jack),
	SOC_ENUM_EXT("Speaker Function", rt5631_enum[1], rt5631_get_spk,
		     rt5631_set_spk),
	SOC_ENUM_EXT("Line In Function", rt5631_enum[1], rt5631_get_line_in,
		     rt5631_set_line_in),
};

static int imx_rt5631_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
    struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_rt5631_data *data;
    int int_port, ext_port;
    enum of_gpio_flags flags;
    struct snd_card *card;
	int ret;
    int i;

    priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
    if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

    if (!strstr(cpu_np->name, "ssi"))
        goto audmux_bypass;

    ret = of_property_read_u32(np, "mux-int-port", &int_port);
    if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

    ret = of_property_read_u32(np, "spk-channel-swap", &spk_swap);
    if (ret) {
		dev_err(&pdev->dev, "spk_swap missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}
audmux_bypass:
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}
	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EINVAL;
        goto fail;
	}
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = of_property_read_u32(codec_np, "clock-frequency", &data->clk_frequency);
	if (ret) {
        dev_err(&codec_dev->dev, "clock-frequency missing or invalid\n");
		goto fail;
	}

    priv->hp_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "hp-det-gpios", 0,
				(enum of_gpio_flags *)&priv->hp_active_low);
	priv->clk_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "clk-enable", 0, &flags);
	if (gpio_is_valid(priv->clk_gpio)){ 
		priv->clk_active_low = flags & OF_GPIO_ACTIVE_LOW;
		devm_gpio_request_one(&pdev->dev, priv->clk_gpio, priv->clk_active_low?GPIOF_OUT_INIT_HIGH:GPIOF_OUT_INIT_LOW,
					"clk-enable");
	}
	for (i=0;i<AMP_MAX;i++){
		priv->amp_spk_gpio[i] = of_get_named_gpio_flags(pdev->dev.of_node, "amp-spk-enable",i ,&flags);
		if (gpio_is_valid(priv->amp_spk_gpio[i])){ 
			char gpio_name[128];
			priv->amp_spk_active_low[i] = flags & OF_GPIO_ACTIVE_LOW;
			sprintf(gpio_name, "amp-spk-enable%d", i);
			devm_gpio_request_one(&pdev->dev, priv->amp_spk_gpio[i], 
					priv->amp_spk_active_low?GPIOF_OUT_INIT_HIGH:GPIOF_OUT_INIT_LOW,
					gpio_name);
		}
		else{
			priv->amp_total = i;
			break;
		}
	}
        dev_dbg(&codec_dev->dev, "amp_total = %d\n", priv->amp_total);

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "rt5631-hifi";
	data->dai.codec_of_node = codec_np;
    data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
    data->dai.ops = &imx_hifi_ops;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_rt5631_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_rt5631_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
    snd_soc_card_set_drvdata(&data->card, data);

    ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

    imx_rt5631_gpio_init(&data->card);

    if (gpio_is_valid(priv->hp_gpio)) {
		ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
		if (ret) {
			dev_err(&pdev->dev, "create hp attr failed (%d)\n", ret);
			goto fail_hp;
		}
	}

    /* Add imx specific controls */
    card = priv->codec->card->snd_card;
    for (i = 0; i < ARRAY_SIZE(rt5631_machine_controls); i++) {
		ret = snd_ctl_add(card,
				  snd_soc_cnew(&rt5631_machine_controls[i],
					       priv->codec, rt5631_machine_controls[i].name, priv->codec->name_prefix));
		if (ret < 0)
			return ret;
	}

    /* Add imx specific widgets */
    snd_soc_dapm_new_controls(&priv->codec->dapm, imx_rt5631_dapm_widgets,
				  ARRAY_SIZE(imx_rt5631_dapm_widgets));

    snd_soc_dapm_enable_pin(&priv->codec->dapm, "Line In Jack");
    snd_soc_dapm_sync(&priv->codec->dapm);

    if (gpio_is_valid(priv->hp_gpio)) {
		priv->hp_irq = gpio_to_irq(priv->hp_gpio);

		ret = request_irq(priv->hp_irq,
			imx_headphone_detect_handler,
			IRQ_TYPE_EDGE_BOTH, pdev->name, priv);

		if (ret < 0) {
			ret = -EINVAL;
			return ret;
		}
    }
    goto fail;

fail_hp:
	snd_soc_unregister_card(&data->card);
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_rt5631_remove(struct platform_device *pdev)
{
	struct imx_rt5631_data *data = platform_get_drvdata(pdev);
    struct imx_priv *priv = &card_priv;

    driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
	snd_soc_unregister_card(&data->card);

	return 0;
}

static const struct of_device_id imx_rt5631_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-rt5631", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rt5631_dt_ids);

static struct platform_driver imx_rt5631_driver = {
	.driver = {
		.name = "imx-rt5631",
		.owner = THIS_MODULE,
        .pm = &snd_soc_pm_ops,
		.of_match_table = imx_rt5631_dt_ids,
	},
	.probe = imx_rt5631_probe,
	.remove = imx_rt5631_remove,
};
module_platform_driver(imx_rt5631_driver);

MODULE_DESCRIPTION("Freescale i.MX RT5631 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-rt5631");
