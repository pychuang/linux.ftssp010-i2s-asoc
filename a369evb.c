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

/******************************************************************************
 * struct snd_soc_ops functions
 *****************************************************************************/
/**
 * a369evb_snd_soc_hw_params() - Setup according to hardware parameters
 *
 * Called by soc_pcm_hw_params().
 */
static int a369evb_snd_soc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int rate = params_rate(params);
	unsigned int clk = 0;
	unsigned int fmt;
	int ret = 0;

	switch (rate) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
	case 192000:
		clk = 12288000;
		break;

	default:
		printk(KERN_ERR "Sample rate %d not supported\n", rate);
		return -EINVAL;
	}

	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set codec DAI format\n");
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set CPU DAI format\n");
		return ret;
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set codec DAI system clock\n");
		return ret;
	}

	/* set the I2S system clock as input */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, clk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set CPU DAI system clock\n");
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
static struct snd_soc_machine a369evb_snd_soc_machine = {
	.name		= "a369evb",
	.dai_link	= &a369evb_snd_soc_dai_link,
	.num_links	= 1,
};

/*
 * a369evb audio private data
 *
 * Codec data
 */
static struct wm8731_setup_data a369evb_wm8731_setup_data = {
	.i2c_bus	= 0,
	.i2c_address	= 0x1b,
};

/*
 * a369evb audio subsystem
 *
 * This structure collects all the stuffs
 */
static struct snd_soc_device a369evb_snd_soc_device = {
	.machine	= &a369evb_snd_soc_machine,
	.platform	= &a369_snd_soc_platform,
	.codec_dev	= &soc_codec_dev_wm8731,
	.codec_data	= &a369evb_wm8731_setup_data,
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static struct resource ftssp010_i2s_resources[] = {
	{
		.start	= SSP_FTSSP010_0_PA_BASE,
		.end	= SSP_FTSSP010_0_PA_LIMIT,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= SSP_FTSSP010_0_IRQ,
		.end	= SSP_FTSSP010_0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device *a369evb_snd_device;

static int __init a369evb_snd_init(void)
{
	unsigned int sclk_cfg1;
	int ret;

	/*
	 * Select external clock source (X_SSP_CLK) as SSP0 clock input, or it
	 * will be (AHB clock * 2) by default.
	 */
	sclk_cfg1 = ioread32(SCU_FTSCU010_VA_BASE + FTSCU010_OFFSET_SCLK_CFG1);
	sclk_cfg1 |= 1 << 4;
	iowrite32(sclk_cfg1, SCU_FTSCU010_VA_BASE + FTSCU010_OFFSET_SCLK_CFG1);

	a369evb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!a369evb_snd_device)
		return -ENOMEM;

	/* drvdata is used by soc-audio driver in sound/soc/soc-core.c */
	platform_set_drvdata(a369evb_snd_device, &a369evb_snd_soc_device);
	a369evb_snd_soc_device.dev = &a369evb_snd_device->dev;

	/* add resources for ftssp010_i2s_dai.probe() */
	ret = platform_device_add_resources(a369evb_snd_device,
			ftssp010_i2s_resources, ARRAY_SIZE(ftssp010_i2s_resources));
	if (ret) {
		platform_device_put(a369evb_snd_device);
		return ret;
	}

	ret = platform_device_add(a369evb_snd_device);
	if (ret)
		platform_device_put(a369evb_snd_device);

	return ret;
}

static void __exit a369evb_snd_exit(void)
{
	platform_device_unregister(a369evb_snd_device);
}

module_init(a369evb_snd_init);
module_exit(a369evb_snd_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("ALSA SoC a369evb");
MODULE_LICENSE("GPL");
