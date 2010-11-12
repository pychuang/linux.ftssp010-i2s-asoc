/*
 * A369 SoC machine driver
 *
 * (C) Copyright 2010 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "codecs/wm8731.h"

#include <mach/ftscu010.h>
#include "ftssp010-i2s.h"
#include "a369-pcm.h"

static struct clk *clk;

/******************************************************************************
 * struct snd_soc_ops functions
 *****************************************************************************/

/**
 * a369evb_snd_soc_hw_params() - Setup according to hardware parameters
 *
 * Called by soc_pcm_hw_params().
 */
static int a369evb_snd_soc_hw_params(struct snd_pcm_substream *ss,
	struct snd_pcm_hw_params *params)
{
	struct device *dev = ss->pcm->card->dev;
	struct snd_soc_pcm_runtime *rtd = ss->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int rate = params_rate(params);
	unsigned int sysclk;
	unsigned int fmt;
	int ret = 0;

	sysclk = clk_get_rate(clk);

	if (sysclk % rate) {
		dev_err(dev, "Sample rate %d not supported\n", rate);
		return -EINVAL;
	}

	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		dev_err(dev, "Failed to set codec DAI format\n");
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		dev_err(dev, "Failed to set CPU DAI format\n");
		return ret;
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK, sysclk,
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "Failed to set codec DAI system clock\n");
		return ret;
	}

	/* set the I2S system clock as input */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(dev, "Failed to set CPU DAI system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops a369evb_snd_soc_ops = {
	.hw_params	= a369evb_snd_soc_hw_params,
};

/* a369evb machine dapm widgets */
static const struct snd_soc_dapm_widget wm8731_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_SPK("Ext Spk", NULL),
SND_SOC_DAPM_MIC("Mic Jack", NULL),
SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

/* a369evb machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {
	/* headphone connected to LHPOUT, RHPOUT */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	/* mic is connected to MICIN */
	{"MICIN", NULL, "Mic Jack"},

	/* Same as the above but no mic bias for line signals */
	{"LLINEIN", NULL, "Line In Jack"},
	{"RLINEIN", NULL, "Line In Jack"},
};

/******************************************************************************
 * struct snd_soc_device functions
 *****************************************************************************/
/**
 * a369evb_wm8731_init() - setup DAPM stuff
 *
 * Called by snd_soc_register_card().
 */
static int a369evb_wm8731_init(struct snd_soc_codec *codec)
{
	/* Add a369evb specific widgets */
	snd_soc_dapm_new_controls(codec, wm8731_dapm_widgets,
				  ARRAY_SIZE(wm8731_dapm_widgets));

	/* Set up a369evb specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}

/*
 * a369evb digital audio interface glue - connects codec <--> CPU
 */
static struct snd_soc_dai_link a369evb_snd_soc_dai_link = {
	.name		= "WM8731",
	.stream_name	= "WM8731",
	.cpu_dai	= &ftssp010_i2s_dai,
	.codec_dai	= &wm8731_dai,
	.init		= a369evb_wm8731_init,
	.ops		= &a369evb_snd_soc_ops,
};

/*
 * a369evb audio machine driver
 *
 * Board specific stuffs
 */
static struct snd_soc_card a369evb_snd_soc_card = {
	.name		= "a369evb",
	.platform	= &a369_snd_soc_platform,
	.dai_link	= &a369evb_snd_soc_dai_link,
	.num_links	= 1,
};

/*
 * a369evb audio subsystem
 *
 * This structure collects all the stuffs
 */
static struct snd_soc_device a369evb_snd_soc_device = {
	.card		= &a369evb_snd_soc_card,
	.codec_dev	= &soc_codec_dev_wm8731,
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static struct platform_device *a369evb_snd_device;

static int __init a369evb_snd_init(void)
{
	int ret;

	clk = clk_get(NULL, "ssp0-extclk");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "%s: Failed to get clock\n", __func__);
		return -ENODEV;
	}

	clk_enable(clk);

	a369evb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!a369evb_snd_device) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* drvdata is used by soc-audio driver in sound/soc/soc-core.c */
	platform_set_drvdata(a369evb_snd_device, &a369evb_snd_soc_device);
	a369evb_snd_soc_device.dev = &a369evb_snd_device->dev;

	ret = platform_device_add(a369evb_snd_device);
	if (ret)
		goto err_add;

	return 0;

err_add:
	platform_device_put(a369evb_snd_device);
err_alloc:
	clk_disable(clk);
	clk_put(clk);
	return ret;
}

static void __exit a369evb_snd_exit(void)
{
	platform_device_unregister(a369evb_snd_device);
	clk_disable(clk);
	clk_put(clk);
}

module_init(a369evb_snd_init);
module_exit(a369evb_snd_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("ALSA SoC a369evb");
MODULE_LICENSE("GPL");
