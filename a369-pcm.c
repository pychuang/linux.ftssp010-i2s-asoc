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
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>

#include <mach/ftapbb020.h>
#include <mach/ftdmac020.h>
#include <mach/dma-a369.h>

#include "a369-pcm.h"
#include "ftssp010-i2s.h"

/*
 * Specify which dma engine to use:
 *	ftapbb020 if not defined
 *	ftdmac020:0 if 0
 *	ftdmac020:1 if 1
 */
#define CONFIG_USE_DMAC	0

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
	struct dma_chan *dma_chan;
#ifdef CONFIG_USE_DMAC
	struct ftdmac020_dma_slave slave;
#else
	struct ftapbb020_dma_slave slave;
#endif
	struct tasklet_struct tasklet;
	struct snd_pcm_substream *substream;
	int period;
	size_t period_bytes;	/* # of bytes per period */
};

/**
 * struct snd_pcm_hardware - Hardware information.
 * @buffer_bytes_max: max size of buffer in bytes
 * @period_bytes_min: min size of period in bytes
 * @period_bytes_max: max size of period in bytes
 * @periods_min: min # of periods in the buffer
 * @periods_max: max # of periods in the buffer
 * @fifo_size: not used
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
	.buffer_bytes_max	= 0x40000,
	.period_bytes_min	= 1,
	.period_bytes_max	= 0xffffff,
	.periods_min		= 2,
	.periods_max		= 256,
	.fifo_size		= 0,
};

/******************************************************************************
 * DMA internal functions
 *****************************************************************************/
static void a369_snd_pcm_dma_complete(void *param)
{
	struct a369_snd_pcm_runtime *a369rtd = param;
	struct snd_pcm_substream *ss = a369rtd->substream;
	struct snd_pcm_runtime *rt = ss->runtime;

	a369rtd->period++;
	if (a369rtd->period == rt->periods)
		a369rtd->period = 0;

	/* dma completion handler cannot submit new operations */
	if (snd_pcm_running(ss)) {
		snd_pcm_period_elapsed(ss);
		tasklet_schedule(&a369rtd->tasklet);
	}
}

static struct dma_async_tx_descriptor *a369_snd_pcm_dma_prepare(
		struct a369_snd_pcm_runtime *a369rtd,
		dma_addr_t dma_addr, unsigned int len)
{
	struct snd_pcm_substream *ss = a369rtd->substream;
	struct dma_chan *chan = a369rtd->dma_chan;
	struct dma_async_tx_descriptor *desc;
	enum dma_data_direction direction;
	struct scatterlist sg;

	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK)
		direction = DMA_TO_DEVICE;
	else
		direction = DMA_FROM_DEVICE;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_addr)), len,
		dma_addr & (PAGE_SIZE - 1));
	sg_dma_address(&sg) = dma_addr;

	desc = chan->device->device_prep_slave_sg(chan, &sg, 1, direction,
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK |
		DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP);

	desc->callback = a369_snd_pcm_dma_complete;
	desc->callback_param = a369rtd;
	return desc;
}

static void a369_snd_pcm_dma_issue_pending(struct a369_snd_pcm_runtime *a369rtd)
{
	struct dma_chan *chan = a369rtd->dma_chan;

	chan->device->device_issue_pending(chan);
}
/******************************************************************************
 * tasklet - transfer a period of data
 *****************************************************************************/
static void a369_snd_pcm_tasklet(unsigned long data)
{
	struct a369_snd_pcm_runtime *a369rtd = (struct a369_snd_pcm_runtime *)data;
	struct snd_pcm_substream *ss = a369rtd->substream;
	struct snd_pcm_runtime *rt = ss->runtime;
	struct dma_async_tx_descriptor *desc;
	size_t dma_offset;
	dma_addr_t dma_pos;

	dma_offset = a369rtd->period * a369rtd->period_bytes;
	dma_pos = rt->dma_addr + dma_offset;

	desc = a369_snd_pcm_dma_prepare(a369rtd, dma_pos, a369rtd->period_bytes);

	/* submit to DMA engine */
	desc->tx_submit(desc);

	a369_snd_pcm_dma_issue_pending(a369rtd);
}

/******************************************************************************
 * struct snd_pcm_ops functions
 *****************************************************************************/
/**
 * a369_snd_pcm_open() - Open platform
 *
 * Allocate PCM runtime private data.
 * Called by soc_pcm_open().
 */
static int a369_snd_pcm_open(struct snd_pcm_substream *ss)
{
	struct device *dev = ss->pcm->card->dev;
	struct snd_soc_pcm_runtime *rtd = ss->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_pcm_runtime *rt = ss->runtime;
	struct ftssp010_i2s_dma_params *dma_params;
	struct a369_snd_pcm_runtime *a369rtd;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	dma_params = snd_soc_dai_get_dma_data(cpu_dai, ss);
	if (!dma_params)
		return -ENODEV;

	snd_soc_set_runtime_hwparams(ss, &a369_snd_pcm_hardware);

	a369rtd = kzalloc(sizeof(*a369rtd), GFP_KERNEL);
	if (!a369rtd)
		return -ENOMEM;

	rt->private_data = a369rtd;

	a369rtd->substream = ss;
	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#ifdef CONFIG_USE_DMAC
		ret = a369_dmac_handshake_alloc("ssp0tx");
		if (ret < 0)
			goto err_hs;

		a369_dmac_handshake_setup(ret, CONFIG_USE_DMAC);
		a369rtd->slave.handshake = ret;
#else
		a369rtd->slave.handshake = A369_APBB_HANDSHAKE_SSP0TX;
#endif
		a369rtd->slave.common.direction = DMA_TO_DEVICE;
		a369rtd->slave.common.dst_addr = dma_params->reg_addr;
	} else {
#ifdef CONFIG_USE_DMAC
		ret = a369_dmac_handshake_alloc("ssp0rx");
		if (ret < 0)
			goto err_hs;

		a369_dmac_handshake_setup(ret, CONFIG_USE_DMAC);
		a369rtd->slave.handshake = ret;
#else
		a369rtd->slave.handshake = A369_APBB_HANDSHAKE_SSP0RX;
#endif
		a369rtd->slave.common.direction = DMA_FROM_DEVICE;
		a369rtd->slave.common.src_addr = dma_params->reg_addr;
	}

#ifdef CONFIG_USE_DMAC
	a369rtd->slave.id = CONFIG_USE_DMAC;
	a369rtd->slave.channels = FTDMAC020_CHANNEL_ALL;
#else
	a369rtd->slave.channels = FTAPBB020_CHANNEL_ALL;
	a369rtd->slave.type = FTAPBB020_BUS_TYPE_APB;
#endif

	tasklet_init(&a369rtd->tasklet, a369_snd_pcm_tasklet,
		(unsigned long)a369rtd);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
#ifdef CONFIG_USE_DMAC
	dma_chan = dma_request_channel(mask, ftdmac020_chan_filter, &a369rtd->slave);
#else
	dma_chan = dma_request_channel(mask, ftapbb020_chan_filter, &a369rtd->slave);
#endif
	if (!dma_chan) {
		dev_err(dev, "Failed to allocate DMA channel\n");
		ret = -EBUSY;
		goto err_request;
	}

	a369rtd->dma_chan = dma_chan;

	dev_info(dev, "Using %s for DMA transfers\n", dma_chan_name(dma_chan));

	return 0;

err_request:
	tasklet_kill(&a369rtd->tasklet);
#ifdef CONFIG_USE_DMAC
err_hs:
#endif
	rt->private_data = NULL;
	kfree(a369rtd);
	return ret;
}

/**
 * a369_snd_pcm_close() - Close platform
 *
 * Release PCM runtime private data.
 * Called by soc_codec_close().
 */
static int a369_snd_pcm_close(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *rt = ss->runtime;
	struct a369_snd_pcm_runtime *a369rtd = rt->private_data;
	struct dma_chan *chan = a369rtd->dma_chan;

	chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
	dma_release_channel(chan);
#ifdef CONFIG_USE_DMAC
	a369_dmac_handshake_free(a369rtd->slave.handshake);
#endif
	tasklet_kill(&a369rtd->tasklet);
	rt->private_data = NULL;
	kfree(a369rtd);
	return 0;
}

/**
 * a369_snd_pcm_hw_params() - Setup DMA resources according to hardware parameters
 *
 * Called by soc_pcm_hw_params().
 */
static int a369_snd_pcm_hw_params(struct snd_pcm_substream *ss,
	struct snd_pcm_hw_params *params)
{
	struct device *dev = ss->pcm->card->dev;
	struct snd_soc_pcm_runtime *rtd = ss->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_pcm_runtime *rt = ss->runtime;
	struct ftssp010_i2s_dma_params *dma_params;
	struct a369_snd_pcm_runtime *a369rtd = rt->private_data;
	struct dma_slave_config *slaveconf = &a369rtd->slave.common;

	dma_params = snd_soc_dai_get_dma_data(cpu_dai, ss);
	if (!dma_params)
		return -ENODEV;

	switch (dma_params->width) {
	case 8:
		slaveconf->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		slaveconf->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 16:
		slaveconf->src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		slaveconf->dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 32:
		slaveconf->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slaveconf->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		dev_err(dev, "Invalid data width %d\n", dma_params->width);
		return -EINVAL;
	}

	return snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(params));
}

/**
 * a369_snd_pcm_hw_free() - Free DMA resources
 *
 * Called by soc_pcm_hw_free().
 */
static int a369_snd_pcm_hw_free(struct snd_pcm_substream *ss)
{
	return snd_pcm_lib_free_pages(ss);
}

/**
 * a369_snd_pcm_prepare() - DMA preparation
 *
 * Called by soc_pcm_prepare().
 */
static int a369_snd_pcm_prepare(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *rt = ss->runtime;
	struct a369_snd_pcm_runtime *a369rtd = rt->private_data;

	a369rtd->period = 0;
	a369rtd->period_bytes = snd_pcm_lib_period_bytes(ss);
	return 0;
}

/**
 * a369_snd_pcm_trigger() - DMA action trigger
 *
 * Called by soc_pcm_trigger().
 */
static int a369_snd_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct snd_pcm_runtime *rt = ss->runtime;
	struct a369_snd_pcm_runtime *a369rtd = rt->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		tasklet_schedule(&a369rtd->tasklet);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

/**
 * a369_snd_pcm_pointer() - Returns the hardware progress of moving data
 *
 * Returns offset to buffer in frames
 * or SNDRV_PCM_POS_XRUN for overrun or underrun.
 *
 * This function will be assigned to soc_pcm_ops.pointer in soc_new_pcm()
 * and then be called by snd_pcm_update_hw_ptr0().
 */
static snd_pcm_uframes_t
a369_snd_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *rt = ss->runtime;
	struct a369_snd_pcm_runtime *a369rtd = rt->private_data;
	size_t offset;
	snd_pcm_uframes_t pointer;

	offset = a369rtd->period * a369rtd->period_bytes;
	pointer = bytes_to_frames(rt, offset);

	if (pointer == rt->buffer_size)
		return 0;
	else if (pointer > rt->buffer_size)
		return SNDRV_PCM_POS_XRUN;

	return pointer;
}

/**
 * a369_snd_pcm_mmap() - mmap DMA buffer
 *
 * This function will be assigned to soc_pcm_ops.mmap in soc_new_pcm()
 * and then be called by snd_pcm_mmap_data().
 */
static int a369_snd_pcm_mmap(struct snd_pcm_substream *ss,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *rt = ss->runtime;

	return dma_mmap_writecombine(ss->pcm->card->dev, vma,
				     rt->dma_area,
				     rt->dma_addr,
				     rt->dma_bytes);
}

/*
 * PCM operations
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
	struct device *dev = pcm->card->dev;
	struct snd_pcm_substream *ss = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &ss->dma_buffer;
	size_t size = a369_snd_pcm_hardware.buffer_bytes_max;

	buf->dev.type		= SNDRV_DMA_TYPE_DEV;
	buf->dev.dev		= dev;
	buf->private_data	= NULL;
	buf->area		= dma_alloc_writecombine(pcm->card->dev, size,
					&buf->addr, GFP_KERNEL);

	if (!buf->area) {
		dev_err(dev, "Failed to preallocate dma buffer\n");
		return -ENOMEM;
	}

	buf->bytes = size;
	return 0;
}

static u64 a369_snd_soc_pcm_dmamask = DMA_BIT_MASK(32);

/**
 * a369_snd_soc_pcm_new() - SoC platform PCM constructor
 *
 * Called by soc_new_pcm().
 */
static int a369_snd_soc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &a369_snd_soc_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

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
 * a369_snd_soc_pcm_free() - SoC platform PCM destructor
 *
 * This function will be assigned to pcm->private_free in soc_new_pcm()
 * and called by snd_pcm_free() later.
 */
static void a369_snd_soc_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *ss;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		ss = pcm->streams[stream].substream;
		if (!ss)
			continue;

		buf = &ss->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
			buf->area, buf->addr);
		buf->area = NULL;
	}
}

/*
 * SoC Platform Interface
 */
struct snd_soc_platform a369_snd_soc_platform = {
	.name		= "a369-audio",
	.pcm_ops 	= &a369_snd_pcm_ops,
	.pcm_new	= a369_snd_soc_pcm_new,
	.pcm_free	= a369_snd_soc_pcm_free,
};
EXPORT_SYMBOL_GPL(a369_snd_soc_platform);

static int __init a369_snd_soc_platform_init(void)
{
	return snd_soc_register_platform(&a369_snd_soc_platform);
}
module_init(a369_snd_soc_platform_init);

static void __exit a369_snd_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&a369_snd_soc_platform);
}
module_exit(a369_snd_soc_platform_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("A369 PCM DMA module");
MODULE_LICENSE("GPL");
