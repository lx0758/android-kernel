// SPDX-License-Identifier: GPL-2.0-only
/*
 * H.264 syntax state and parameter-set generation for the RKVENC v1
 * encoder. The same struct rkvenc_h264_seq feeds both the hardware
 * SYNT_* registers (through the engine) and the software SPS/PPS writer,
 * so the slice header the hardware emits and the SPS/PPS cannot disagree.
 *
 * POC type 0, one short-term reference, max_num_ref_frames 1, every intra
 * picture is an IDR (spec/05 2, 8.2). The parameter sets are cached at
 * stream configuration and copied (not regenerated) per IDR.
 */

#include <linux/string.h>

#include "rkvenc.h"
#include "rkvenc-nal.h"

static const u8 rkvenc_h264_level_idc[] = {
	10, 11, 11, 12, 13, 20, 21, 22, 30, 31, 32, 40,
	41, 42, 50, 51, 52, 60, 61, 62,
};

static u8 rkvenc_h264_level(u32 ctrl)
{
	if (ctrl >= ARRAY_SIZE(rkvenc_h264_level_idc))
		return 62;
	return rkvenc_h264_level_idc[ctrl];
}

static u8 rkvenc_h264_profile_idc(u32 ctrl)
{
	switch (ctrl) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		return 66;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return 77;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
	default:
		return 100;
	}
}

/* Build the per-stream SPS/PPS values from the control snapshot. */
int rkvenc_h264_setup(struct rkvenc_ctx *ctx)
{
	struct rkvenc_h264_seq *seq = &ctx->h264_seq;

	memset(seq, 0, sizeof(*seq));
	seq->width = ctx->width;
	seq->height = ctx->height;
	seq->profile_idc = rkvenc_h264_profile_idc(ctx->c.profile);
	seq->level_idc = rkvenc_h264_level(ctx->c.level);
	/* Smallest frame-num / POC ranges that exceed any reachable GOP. */
	seq->log2_max_frame_num_m4 = 4;	/* 2^8 = 256 */
	seq->log2_max_poc_lsb_m4 = 5;		/* 2^9 = 512 */
	seq->pic_init_qp = ctx->c.p_frame_qp;
	seq->chroma_qp_offset = ctx->c.chroma_qp_offset;
	seq->cabac = ctx->c.cabac && seq->profile_idc != 66;
	if (ctx->c.profile == V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)
		seq->transform_8x8 = ctx->c.transform_8x8;
	seq->dblk_idc = ctx->c.lf_mode;
	seq->alpha_offset = ctx->c.lf_alpha;
	seq->beta_offset = ctx->c.lf_beta;
	seq->timing = true;
	seq->fps = ctx->fps ?: 30;

	return rkvenc_h264_headers_write(seq, ctx->hdr, RKVENC_HDR_MAX);
}

/* Copy the cached parameter sets into a capture buffer. */
int rkvenc_h264_write_headers(struct rkvenc_ctx *ctx, u8 *buf, u32 size)
{
	if (ctx->hdr_len > size)
		return -ENOSPC;
	memcpy(buf, ctx->hdr, ctx->hdr_len);
	return ctx->hdr_len;
}

void rkvenc_h264_build_pic(struct rkvenc_ctx *ctx,
			   struct rkvenc_h264_pic *pic, bool idr)
{
	struct rkvenc_gop *g = &ctx->gop;

	pic->seq = &ctx->h264_seq;
	pic->nal_ref_idc = idr ? 3 : 2;
	pic->frame_num = g->frames_since_idr & 0xff;
	pic->poc_lsb = (2 * g->frames_since_idr) & 0x1ff;
	pic->idr_pic_id = g->idr_pic_id & 0xffff;
}
