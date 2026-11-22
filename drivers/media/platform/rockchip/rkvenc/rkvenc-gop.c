// SPDX-License-Identifier: GPL-2.0-only
/*
 * GOP / frame-type state machine for the RKVENC v1 encoder.
 *
 * One short-term reference, low-delay IDR+P, POC type 0, no B frames.
 * The slot ping-pong (cur writes, last is the reference; after a good
 * frame last = cur; cur ^= 1) mirrors the validated lab harness exactly
 * so the driver produces the same coding structure the hardware was
 * measured against. A failed frame never commits, so rollback is free.
 */

#include "rkvenc.h"

void rkvenc_gop_init(struct rkvenc_ctx *ctx)
{
	ctx->gop.frames_since_idr = 0;
	ctx->gop.idr_pic_id = 0;
	ctx->cur = 0;
	ctx->last = 0;
	ctx->last_good_valid = false;
	ctx->consecutive_errors = 0;
}

/* Pictures too small for the hardware to code P frames correctly. */
static bool rkvenc_gop_intra_only(struct rkvenc_ctx *ctx)
{
	u32 w16 = ALIGN(ctx->width, 16) / 16;
	u32 h16 = ALIGN(ctx->height, 16) / 16;

	if (ctx->codec == RKVENC_CODEC_HEVC)
		return ctx->height <= 64;

	return !((w16 >= 3 && h16 >= 2) || (w16 == 1 && h16 == 1));
}

/* Decide whether the next frame is an IDR. Called from device_run(). */
bool rkvenc_gop_is_idr(struct rkvenc_ctx *ctx)
{
	struct rkvenc_gop *g = &ctx->gop;

	if (g->frames_since_idr == 0 || rkvenc_gop_intra_only(ctx))
		return true;
	if (ctx->c.gop_size && g->frames_since_idr >= ctx->c.gop_size)
		return true;
	if (ctx->c.force_keyframe)
		return true;
	/* After a recovery with no usable reference, resync with an IDR. */
	if (!ctx->last_good_valid)
		return true;
	return false;
}

void rkvenc_gop_commit(struct rkvenc_ctx *ctx)
{
	struct rkvenc_gop *g = &ctx->gop;

	if (ctx->cur_intra) {
		g->idr_pic_id++;
		ctx->last_good = ctx->cur;
		ctx->last_good_valid = true;
		/* A new GOP starts; the next (P) frame is #1 of it. */
		g->frames_since_idr = 1;
	} else {
		g->frames_since_idr++;
	}
	ctx->last = ctx->cur;
	ctx->cur ^= 1;
	ctx->c.force_keyframe = false;
}

void rkvenc_gop_rollback(struct rkvenc_ctx *ctx)
{
	/* The failed frame did not touch the reference slot, so there is
	 * nothing to undo except to drop the consumed force-key-frame flag.
	 */
	ctx->c.force_keyframe = false;
}

void rkvenc_gop_force_keyframe(struct rkvenc_ctx *ctx)
{
	ctx->c.force_keyframe = true;
}
