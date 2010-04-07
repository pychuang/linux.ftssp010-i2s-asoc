/*
 * ALSA SoC PCM Interface for A369
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

#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/soc.h>

#include <mach/apb_dma.h>
#include <mach/platform/apb_dma.h>

#include "a369-pcm.h"
#include "ftssp010-i2s.h"

/*
 * The period is a term that corresponds to a fragment in the OSS world.
 * The period defines the size at which a PCM interrupt is generated.
 * This size strongly depends on the hardware. Generally, the smaller period
 * size will give you more interrupts, that is, more controls.
 * In the case of capture, this size defines the input latency.
 * On the other hand, the whole buffer size defines the output latency for the
 * playback direction. 
 */
struct a369_snd_pcm_runtime {
	int channel;
	struct ftssp010_i2s_dma_params *dma_params;
	int period;
	dma_addr_t reg_addr;	/* physical address of ftssp010 data register */
	size_t period_bytes;	/* # of bytes per period */
	int stype, dtype;
	int sincr, dincr;
};

/**
 * @brief hardware information
 *
 * This structure will be copied to runtime->hw by snd_soc_set_runtime_hwparams()
 * runtime->hw is in turn used by snd_pcm_hw_constraints_complete()
 */
static struct snd_pcm_hardware a369_snd_pcm_hardware = {
	.info	= SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_BLOCK_TRANSFER
		| SNDRV_PCM_INFO_MMAP
		| SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_PAUSE,
	.buffer_bytes_max	= 0x40000,	/* max size of buffer in bytes */
	.period_bytes_min	= 1,		/* min size of period in bytes */
	.period_bytes_max	= 0xffffff,	/* max size of period in bytes */
	.periods_min		= 2,		/* min # of periods in the buffer */
	.periods_max		= 256,		/* max # of periods in the buffer */
	.fifo_size		= 0,		/* not used */
};

/******************************************************************************
 * APB DMA internal functions
 *****************************************************************************/
static void a369_snd_pcm_setup_dma(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;
	int width;
	int incr;
	int count;
	int dreqsel;

	a369rtd->reg_addr = a369rtd->dma_params->reg_addr;
	a369rtd->period_bytes = snd_pcm_lib_period_bytes(substream);

	switch (a369rtd->dma_params->width) {
	case 8:
		width = APBDMA_WIDTH_8BIT;
		incr = APBDMA_CTL_INC1;
		count = a369rtd->period_bytes;
		break;

	case 16:
		width = APBDMA_WIDTH_16BIT;
		incr = APBDMA_CTL_INC2;
		count = a369rtd->period_bytes / 2;
		break;

	case 32:
		width = APBDMA_WIDTH_32BIT;
		incr = APBDMA_CTL_INC4;
		count = a369rtd->period_bytes / 4;
		break;

	default:
		printk(KERN_ERR "Invalid data width %d\n", a369rtd->dma_params->width);
		BUG();
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		a369rtd->sincr = incr;
		a369rtd->dincr = 0;
		dreqsel = SSP_APBDMA_REQ_0;
		a369rtd->stype = APBDMA_TYPE_AHB;
		a369rtd->dtype = APBDMA_TYPE_APB;
	} else {
		a369rtd->sincr = 0;
		a369rtd->dincr = incr;
		dreqsel = SSP_APBDMA_ACK_0;
		a369rtd->stype = APBDMA_TYPE_APB;
		a369rtd->dtype = APBDMA_TYPE_AHB;
	}

	fa_set_apb_dma_transfer_params(a369rtd->channel, 0, dreqsel,
		width, 0, count, INT_DMA_ERROR | INT_DMA_TRIGGER);
}

static void a369_snd_pcm_enqueue_dma(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;
	size_t dma_offset;
	dma_addr_t dma_pos;
	dma_addr_t saddr, daddr;

	dma_offset = a369rtd->period * a369rtd->period_bytes;
	dma_pos = runtime->dma_addr + dma_offset;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		saddr = dma_pos;
		daddr = a369rtd->reg_addr;
	} else {
		saddr = a369rtd->reg_addr;
		daddr = dma_pos;
	}

	fa_set_apb_dma_src_params(a369rtd->channel, saddr, a369rtd->stype,
		a369rtd->sincr);
	fa_set_apb_dma_dst_params(a369rtd->channel, daddr, a369rtd->dtype,
		a369rtd->dincr);

	a369rtd->period++;
	if (a369rtd->period == runtime->periods) {
		a369rtd->period = 0;
	}
}

/******************************************************************************
 * APB DMA callback function
 *****************************************************************************/
static void a369_snd_pcm_irq(int ch, u16 int_status, void * data)
{
	struct snd_pcm_substream *substream = data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;

	if (int_status != INT_DMA_TRIGGER)
		return;

	if (snd_pcm_running(substream)) {
		snd_pcm_period_elapsed(substream);
		a369_snd_pcm_enqueue_dma(substream);
		fa_apb_dma_start(a369rtd->channel);
	}
}

/******************************************************************************
 * struct snd_pcm_ops functions
 *****************************************************************************/
/**
 * @brief Open platform - allocate PCM runtime private data
 *
 * Called by soc_pcm_open()
 */
static int a369_snd_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct ftssp010_i2s_dma_params *dma_params = cpu_dai->dma_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd;
	int ret;

	if (!dma_params)
		return -ENODEV;

	snd_soc_set_runtime_hwparams(substream, &a369_snd_pcm_hardware);

	a369rtd = kzalloc(sizeof(*a369rtd), GFP_KERNEL);
	if (!a369rtd)
		return -ENOMEM;

	runtime->private_data = a369rtd;

	a369rtd->dma_params = dma_params;

	ret = fa_request_apb_dma_auto(0, "ftssp010-i2s-pcm",
		a369_snd_pcm_irq, substream, &a369rtd->channel, 0);
	if (ret) {
		printk(KERN_ERR "Failed to allocate APB DMA channel\n");
		goto err_req_dma;
	}

	printk(KERN_DEBUG "Allocated APB DMA channel %d\n", a369rtd->channel);
	return 0;

err_req_dma:
	runtime->private_data = NULL;
	kfree(a369rtd);
	return ret;
}

/**
 * @brief Close platform - release PCM runtime private data
 *
 * Called by soc_codec_close()
 */
static int a369_snd_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;

	fa_free_apb_dma(a369rtd->channel);

	runtime->private_data = NULL;
	kfree(a369rtd);
	return 0;
}

/**
 * @brief Setup DMA resources according to hardware parameters
 *
 * Called by soc_pcm_hw_params()
 */
static int a369_snd_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

/**
 * @brief Free DMA resources
 *
 * Called by soc_pcm_hw_free()
 */
static int a369_snd_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/**
 * @brief DMA preparation
 *
 * Called by soc_pcm_prepare()
 */
static int a369_snd_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;

	a369rtd->period = 0;
	a369_snd_pcm_setup_dma(substream);
	a369_snd_pcm_enqueue_dma(substream);
	return 0;
}

/**
 * @brief DMA action trigger
 *
 * Called by soc_pcm_trigger()
 */
static int a369_snd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		fa_apb_dma_start(a369rtd->channel);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		fa_apb_dma_stop(a369rtd->channel);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

/**
 * @brief Returns the hardware progress of moving data.
 *
 * @return how many frames have been moved
 * @retval SNDRV_PCM_POS_XRUN overrun or underrun
 *
 * This function will be assigned to soc_pcm_ops.pointer in soc_new_pcm()
 * and then be called by snd_pcm_update_hw_ptr_pos()
 */
static snd_pcm_uframes_t
a369_snd_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct a369_snd_pcm_runtime *a369rtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t offset;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ptr = fa_apb_dma_get_src_addr(a369rtd->channel);
	} else {
		ptr = fa_apb_dma_get_dest_addr(a369rtd->channel);
	}

	offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);

	if (offset == runtime->buffer_size)
		return 0;
	else if (offset > runtime->buffer_size)
		return SNDRV_PCM_POS_XRUN;

	return offset;
}

/**
 * @brief mmap DMA buffer
 *
 * This function will be assigned to soc_pcm_ops.mmap in soc_new_pcm()
 * and then be called by snd_pcm_mmap_data()
 */
static int a369_snd_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

/**
 * @brief PCM operations
 *
 * DMA stuff
 */
static struct snd_pcm_ops a369_snd_pcm_ops = {
	.open		= a369_snd_pcm_open,
	.close		= a369_snd_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= a369_snd_pcm_hw_params,
	.hw_free	= a369_snd_pcm_hw_free,
	.prepare	= a369_snd_pcm_prepare,
	.trigger	= a369_snd_pcm_trigger,
	.pointer	= a369_snd_pcm_pointer,
	.mmap		= a369_snd_pcm_mmap,
};

/******************************************************************************
 * struct snd_soc_platform functions
 *****************************************************************************/
static int a369_snd_soc_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = a369_snd_pcm_hardware.buffer_bytes_max;

	buf->dev.type		= SNDRV_DMA_TYPE_DEV;
	buf->dev.dev		= pcm->card->dev;
	buf->private_data	= NULL;
	buf->area		= dma_alloc_writecombine(pcm->card->dev, size,
					&buf->addr, GFP_KERNEL);

	printk(KERN_DEBUG "preallocate_dma_buffer: area=%p, addr=%p, size=%x\n",
		(void *) buf->area, (void *) buf->addr, size);

	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static u64 a369_snd_soc_pcm_dmamask = DMA_32BIT_MASK;

/**
 * @brief SoC platform PCM constructor
 *
 * Called by soc_new_pcm()
 */
static int a369_snd_soc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &a369_snd_soc_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = a369_snd_soc_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = a369_snd_soc_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
out:
	return ret;
}

/**
 * @brief SoC platform PCM destructor
 *
 * This function will be assigned to pcm->private_free in soc_new_pcm()
 * and called by snd_pcm_free() later.
 */
static void a369_snd_soc_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
			buf->area, buf->addr);
		buf->area = NULL;
	}
}

/**
 * @brief SoC Platform Interface
 */
struct snd_soc_platform a369_snd_soc_platform = {
	.name		= "a369-audio",
	.pcm_ops 	= &a369_snd_pcm_ops,
	.pcm_new	= a369_snd_soc_pcm_new,
	.pcm_free	= a369_snd_soc_pcm_free,
};
EXPORT_SYMBOL_GPL(a369_snd_soc_platform);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("A369 PCM DMA module");
MODULE_LICENSE("GPL");
