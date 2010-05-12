/*
 * Faraday FTSSP010 I2S ALSA SoC Digital Audio Interface
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

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include "ftssp010-i2s.h"
#include "ftssp010.h"

struct ftssp010_i2s {
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	unsigned int sysclk;
	struct ftssp010_i2s_dma_params *dma_params[2];
};

/******************************************************************************
 * interrupt handler
 *****************************************************************************/
static irqreturn_t ftssp010_i2s_interrupt(int irq, void *dev_id)
{
	struct ftssp010_i2s *ftssp010_i2s = dev_id;
	unsigned int status;

	status = ioread32(ftssp010_i2s->base + FTSSP010_OFFSET_ISR);

	if (status & FTSSP010_ISR_RFOR) {
		printk(KERN_INFO "ftssp010-i2s: rx fifo overrun\n");
	}

	if (status & FTSSP010_ISR_TFUR) {
		printk(KERN_INFO "ftssp010-i2s: tx fifo underrun\n");
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct snd_soc_ops functions
 *****************************************************************************/
/**
 * @brief Open CPU DAI
 *
 * Called by soc_pcm_open()
 */
static int ftssp010_i2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;

	cpu_dai->dma_data = ftssp010_i2s->dma_params[substream->stream];
	return 0;
}

/**
 * @brief Setup CPU DAI resources according to hardware parameters
 *
 * Called by soc_pcm_hw_params()
 */
static int ftssp010_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;
	struct ftssp010_i2s_dma_params *dma_params = cpu_dai->dma_data;
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	unsigned int cr0;
	unsigned int cr1;
	int data_len;
	int padding;
	int sclk;
	int sclkdiv;

	cr0 = ioread32(ftssp010_i2s->base + FTSSP010_OFFSET_CR0);
	switch (channels) {
	case 1:
		cr0 &= ~FTSSP010_CR0_STEREO;
		break;

	case 2:
		cr0 |= FTSSP010_CR0_STEREO;
		break;

	default:
		printk(KERN_ERR "ftssp010-i2s: Invalid number of channels: %d\n",
			channels);
		return -EINVAL;
	}

	printk(KERN_DEBUG "ftssp010-i2s: [CR0] = %08x\n", cr0);
	iowrite32(cr0, ftssp010_i2s->base + FTSSP010_OFFSET_CR0);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		data_len = 8;
		padding = 0;
		break;

	case SNDRV_PCM_FORMAT_S16:
	case SNDRV_PCM_FORMAT_U16:
		data_len = 16;
		padding	= 0;
		break;

	case SNDRV_PCM_FORMAT_S24:
	case SNDRV_PCM_FORMAT_U24:
		data_len = 24;
		padding = 8;
		break;

	case SNDRV_PCM_FORMAT_S32:
	case SNDRV_PCM_FORMAT_U32:
		data_len = 32;
		padding = 0;
		break;

	default:
		printk(KERN_ERR "ftssp010-i2s: Format not supported\n");
		return -EINVAL;
	}

	cr1 = FTSSP010_CR1_SDL(data_len - 1) | FTSSP010_CR1_PDL(padding);

	sclk = 2 * (data_len + padding) * rate;
	sclkdiv = ftssp010_i2s->sysclk / 2 / sclk;

	if (sclk * sclkdiv * 2 != ftssp010_i2s->sysclk) {
		printk(KERN_ERR "ftssp010-i2s: Sample rate %d not supported\n", rate);
		return -EINVAL;
	}

	cr1 |= FTSSP010_CR1_SCLKDIV(sclkdiv - 1);
	printk(KERN_DEBUG "ftssp010-i2s: [CR1] = %08x\n", cr1);
	iowrite32(cr1, ftssp010_i2s->base + FTSSP010_OFFSET_CR1);

	dma_params->width = data_len + padding;
	return 0;
}

/**
 * @brief CPU DAI preparation
 *
 * Called by soc_pcm_prepare()
 */
static int ftssp010_i2s_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;
	unsigned int icr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		icr = FTSSP010_ICR_TFDMA | FTSSP010_ICR_TFTHOD(12);
	} else {
		icr = FTSSP010_ICR_RFDMA | FTSSP010_ICR_RFTHOD(4);
	}

	printk(KERN_DEBUG "ftssp010-i2s: [ICR] = %08x\n", icr);
	iowrite32(icr, ftssp010_i2s->base + FTSSP010_OFFSET_ICR);
	return 0;
}

/**
 * @brief CPU DAI action trigger
 *
 * Called by soc_pcm_trigger()
 */
static int ftssp010_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;
	unsigned int cr2;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		cr2 = FTSSP010_CR2_SSPEN | FTSSP010_CR2_TXDOE;
		cr2 |= FTSSP010_CR2_RXFCLR | FTSSP010_CR2_TXFCLR;
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		cr2 = 0;
		break;

	default:
		return -EINVAL;
	}

	printk(KERN_DEBUG "ftssp010-i2s: [CR2] = %08x\n", cr2);
	iowrite32(cr2, ftssp010_i2s->base + FTSSP010_OFFSET_CR2);
	return 0;
}

/******************************************************************************
 * struct snd_soc_dai_ops functions
 *****************************************************************************/
/**
 * @brief Setup hardware according to the DAI format
 *
 * Called by snd_soc_dai_set_fmt()
 */
static int ftssp010_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;
	unsigned int cr0;

	cr0 = FTSSP010_CR0_FFMT_I2S;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		cr0 |= FTSSP010_CR0_FSDIST(1);
		break;

	case SND_SOC_DAIFMT_RIGHT_J:
		cr0 |= FTSSP010_CR0_FSDIST(0);
		cr0 |= FTSSP010_CR0_FSJSTFY;	/* padding in front of data */
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		cr0 |= FTSSP010_CR0_FSDIST(0);
		break;

	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;

	case SND_SOC_DAIFMT_NB_IF:
		cr0 |= FTSSP010_CR0_FSPO;	/* frame sync active low */
		break;

	default:
		return -EINVAL;
	}

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/*
		 * codec is clock & frame master ->
		 * interface is clock & frame slave
		 */
		break;

	case SND_SOC_DAIFMT_CBS_CFS:
		/*
		 * codec is clock & frame slave ->
		 * interface is clock & frame master
		 */
		cr0 |= FTSSP010_CR0_MASTER;
		break;

	default:
		return -EINVAL;
	}

	printk(KERN_DEBUG "ftssp010-i2s: [CR0] = %08x\n", cr0);
	iowrite32(cr0, ftssp010_i2s->base + FTSSP010_OFFSET_CR0);

	return 0;
}

/**
 * @brief Setup system clock
 *
 * @param cpu_dai CPU DAI
 * @param clk_id DAI specific clock ID
 * @param freq new clock frequency in Hz
 * @param dir new clock direction
 *
 * Called by snd_soc_dai_set_sysclk()
 */
static int ftssp010_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct ftssp010_i2s *ftssp010_i2s = cpu_dai->private_data;

	ftssp010_i2s->sysclk = freq;
	return 0;
}

/******************************************************************************
 * struct snd_soc_dai functions
 *****************************************************************************/
static struct ftssp010_i2s_dma_params ftssp010_i2s_playback_dma_params;
static struct ftssp010_i2s_dma_params ftssp010_i2s_capture_dma_params;

/**
 * @brief Get hardware resource of FTSSP010
 *
 * Called by soc_probe()
 */
static int ftssp010_i2s_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	struct ftssp010_i2s *ftssp010_i2s;
	struct resource *res;
	int irq;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ftssp010_i2s = kzalloc(sizeof(*ftssp010_i2s), GFP_KERNEL);
	if (!ftssp010_i2s) {
		printk(KERN_ERR "ftssp010-i2s: Failed to allocate private data\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	ftssp010_i2s->res = request_mem_region(res->start,
			res->end - res->start, dev_name(&pdev->dev));
	if (ftssp010_i2s->res == NULL) {
		printk(KERN_ERR "ftssp010-i2s: Failed to reserve memory region\n");
		ret = -ENOMEM;
		goto err_req_mem;
	}

	ftssp010_i2s->base = ioremap(res->start, res->end - res->start);
	if (ftssp010_i2s->base == NULL) {
		printk(KERN_ERR "ftssp010-i2s: Failed to ioremap\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = request_irq(irq, ftssp010_i2s_interrupt, IRQF_SHARED,
		"ftssp010-i2s", ftssp010_i2s);
	if (ret) {
		printk(KERN_ERR "ftssp010-i2s: Failed to request irq %d\n", irq);
		goto err_irq;
	}

	ftssp010_i2s->irq = irq;

	/*
	 * setup dma parameters
	 */
	ftssp010_i2s->dma_params[SNDRV_PCM_STREAM_PLAYBACK]
		= &ftssp010_i2s_playback_dma_params;
	ftssp010_i2s->dma_params[SNDRV_PCM_STREAM_PLAYBACK]->reg_addr
		= res->start + FTSSP010_OFFSET_DATA;

	ftssp010_i2s->dma_params[SNDRV_PCM_STREAM_CAPTURE]
		= &ftssp010_i2s_capture_dma_params;
	ftssp010_i2s->dma_params[SNDRV_PCM_STREAM_CAPTURE]->reg_addr
		= res->start + FTSSP010_OFFSET_DATA;

	dai->private_data = ftssp010_i2s;

	printk(KERN_INFO "ftssp010-i2s: irq %d, mapped at %p\n", irq,
		ftssp010_i2s->base);

	return 0;

err_irq:
	iounmap(ftssp010_i2s->base);
err_ioremap:
	release_resource(ftssp010_i2s->res);
err_req_mem:
	kfree(ftssp010_i2s);
err_alloc:
	return ret;
}

/**
 * @brief Release hardware resource of FTSSP010
 *
 * Called by soc_remove()
 */
static void ftssp010_i2s_remove(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	struct ftssp010_i2s *ftssp010_i2s = dai->private_data;

	dai->private_data = NULL;
	free_irq(ftssp010_i2s->irq, ftssp010_i2s);
	iounmap(ftssp010_i2s->base);
	release_resource(ftssp010_i2s->res);
	kfree(ftssp010_i2s);
}

#define FTSSP010_I2S_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 \
				| SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 \
				| SNDRV_PCM_RATE_96000)
#define FTSSP010_I2S_FORMATS	(SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 \
				| SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_U16 \
				| SNDRV_PCM_FMTBIT_S24 | SNDRV_PCM_FMTBIT_U24 \
				| SNDRV_PCM_FMTBIT_S32 | SNDRV_PCM_FMTBIT_U32)

/**
 * @brief SoC CPU Digital Audio Interface
 */
struct snd_soc_dai ftssp010_i2s_dai = {
	.name	= "ftssp010-i2s",
	.id	= 0,
	.type	= SND_SOC_DAI_I2S,
	.probe	= ftssp010_i2s_probe,
	.remove	= ftssp010_i2s_remove,
	.playback = {
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= FTSSP010_I2S_RATES,
		.formats	= FTSSP010_I2S_FORMATS,
	},
	.capture = {
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= FTSSP010_I2S_RATES,
		.formats	= FTSSP010_I2S_FORMATS,
	},
	.ops = {
		.startup	= ftssp010_i2s_startup,
		.hw_params	= ftssp010_i2s_hw_params,
		.prepare	= ftssp010_i2s_prepare,
		.trigger	= ftssp010_i2s_trigger,
	},
	.dai_ops = {
		.set_fmt	= ftssp010_i2s_set_dai_fmt,
		.set_sysclk	= ftssp010_i2s_set_dai_sysclk,
	},
};
EXPORT_SYMBOL_GPL(ftssp010_i2s_dai);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("FTSSP010 I2S ASoC Digital Audio Interface");
MODULE_LICENSE("GPL");
