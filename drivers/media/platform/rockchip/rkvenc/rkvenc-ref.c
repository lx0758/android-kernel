// SPDX-License-Identifier: GPL-2.0-only
/*
 * Reconstruction / reference slot pool for the RKVENC v1 encoder.
 *
 * Two slots alternate in the single short-term reference (spec/05 8.4,
 * spec/06 3.10): a failed frame never touches a reference slot, so
 * rollback is free. The buffers are sized by the validated formulas in
 * rkvenc_vepu541_slot_sizes() and allocated as separate DMA regions so an
 * overrun of one cannot silently corrupt another.
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "rkvenc.h"

/* Index into slot_cpu[][4] / the per-slot allocation order. */
enum {
	RKVENC_BUF_HDR = 0,
	RKVENC_BUF_BODY,
	RKVENC_BUF_DSP,
	RKVENC_BUF_CMV,
};

static void rkvenc_ref_free_slot(struct rkvenc_ctx *ctx, unsigned int i)
{
	struct device *dev = ctx->dev->dev;
	size_t sizes[4];
	u32 handles[4];
	unsigned int b;

	sizes[RKVENC_BUF_HDR] = ctx->slot_sz.rfp_hdr;
	sizes[RKVENC_BUF_BODY] = ctx->slot_sz.rfp_body;
	sizes[RKVENC_BUF_DSP] = ctx->slot_sz.dsp;
	sizes[RKVENC_BUF_CMV] = ctx->slot_sz.cmv;
	handles[RKVENC_BUF_HDR] = ctx->slot[i].rfp_hdr;
	handles[RKVENC_BUF_BODY] = ctx->slot[i].rfp_body;
	handles[RKVENC_BUF_DSP] = ctx->slot[i].dsp;
	handles[RKVENC_BUF_CMV] = ctx->slot[i].cmv;

	for (b = 0; b < 4; b++) {
		if (ctx->slot_cpu[i][b])
			dma_free_attrs(dev, sizes[b], ctx->slot_cpu[i][b],
				       handles[b], DMA_ATTR_NO_KERNEL_MAPPING);
		ctx->slot_cpu[i][b] = NULL;
	}
	memset(&ctx->slot[i], 0, sizeof(ctx->slot[i]));
}

int rkvenc_ref_alloc(struct rkvenc_ctx *ctx)
{
	struct device *dev = ctx->dev->dev;
	struct rkvenc_slot_sizes *sz = &ctx->slot_sz;
	dma_addr_t dma;
	void *cpu;
	unsigned int i;
	int ret = 0;

	if (ctx->slots_alloc)
		return 0;

	rkvenc_vepu541_slot_sizes(ctx->codec, ctx->width, ctx->height, sz);

	for (i = 0; i < RKVENC_SLOTS; i++) {
		/* Reconstruction header. Body follows it with a 4 KiB step. */
		cpu = dma_alloc_attrs(dev, sz->rfp_hdr, &dma, GFP_KERNEL,
				      DMA_ATTR_NO_KERNEL_MAPPING);
		if (!cpu)
			goto err;
		ctx->slot_cpu[i][RKVENC_BUF_HDR] = cpu;
		ctx->slot[i].rfp_hdr = lower_32_bits(dma);

		cpu = dma_alloc_attrs(dev, sz->rfp_body, &dma, GFP_KERNEL,
				      DMA_ATTR_NO_KERNEL_MAPPING);
		if (!cpu)
			goto err;
		ctx->slot_cpu[i][RKVENC_BUF_BODY] = cpu;
		ctx->slot[i].rfp_body = lower_32_bits(dma);

		cpu = dma_alloc_attrs(dev, sz->dsp, &dma, GFP_KERNEL,
				      DMA_ATTR_NO_KERNEL_MAPPING);
		if (!cpu)
			goto err;
		ctx->slot_cpu[i][RKVENC_BUF_DSP] = cpu;
		ctx->slot[i].dsp = lower_32_bits(dma);

		if (ctx->codec == RKVENC_CODEC_HEVC && sz->cmv) {
			cpu = dma_alloc_attrs(dev, sz->cmv, &dma, GFP_KERNEL,
					      DMA_ATTR_NO_KERNEL_MAPPING);
			if (!cpu)
				goto err;
			ctx->slot_cpu[i][RKVENC_BUF_CMV] = cpu;
			ctx->slot[i].cmv = lower_32_bits(dma);
		} else {
			ctx->slot[i].cmv = 0;
		}
	}

	ctx->slots_alloc = true;
	return 0;

err:
	for (i = 0; i < RKVENC_SLOTS; i++)
		rkvenc_ref_free_slot(ctx, i);
	ret = -ENOMEM;
	return ret;
}

void rkvenc_ref_free(struct rkvenc_ctx *ctx)
{
	unsigned int i;

	if (!ctx->slots_alloc)
		return;

	for (i = 0; i < RKVENC_SLOTS; i++)
		rkvenc_ref_free_slot(ctx, i);
	ctx->slots_alloc = false;
	ctx->cur = 0;
	ctx->last = 0;
}
