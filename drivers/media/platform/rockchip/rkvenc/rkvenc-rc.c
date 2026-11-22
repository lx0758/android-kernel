// SPDX-License-Identifier: GPL-2.0-only
/*
 * Frame-level integer rate controller for the RKVENC v1 encoder
 * (design/02 section 12.3).
 *
 * The hardware row-level RC is intentionally left disabled in this series
 * (the engine is given f->rc == NULL), so the controller only varies the
 * per-picture QP; the picture itself is encoded at a constant QP. The QP
 * scale qs[] is shared by H.264 and HEVC (quantiser step doubles every
 * six QP) and is derived exactly from round(256 * 2^((q-4)/6)).
 *
 * Of the policy constants the design marks "needs approval", the C11 VBV
 * size and the D horizon are one GOP of bits, C6 is the usual 30 fps,
 * and the C12 I/P ratio clamp, the C13 QP step and the P complexity
 * weight are measured on RV1126 camera streams at 4K and 720p.
 */

#include <linux/math.h>
#include <linux/math64.h>

#include "rkvenc.h"

/* qs[q] = round(256 * 2^((q-4)/6)) in Q8, q = 0..51. */
#define RKVENC_QS_BASE	{ 256, 287, 323, 362, 406, 456 }

/* Derived safe policy values (design/04 "needs approval" boxes). */
#define RKVENC_RC_DEFAULT_FPS	30u	/* C6: default frame rate (S_PARM) */

/* C12: I/P complexity ratio cap, Q8; static scenes measured up to 37 */
#define RKVENC_RC_W_CLAMP (64u * 256)
/* C13: max P QP change per frame; a larger step makes the P QP flip */
#define RKVENC_RC_QP_STEP 1
/* Weight of a new P frame in the P complexity, Q8; measured */
#define RKVENC_RC_X_WEIGHT 32

/* I-frame QP below the P-frame QP, Rockchip MPP's qp_delta_ip defaults */
#define RKVENC_RC_QP_DELTA_IP_HEVC 4
#define RKVENC_RC_QP_DELTA_IP_H264 2

static u32 rkvenc_qs[RKVENC_QP_MAX + 1];

static void rkvenc_qs_build(void)
{
	static const u32 base[6] = RKVENC_QS_BASE;
	unsigned int q;
	int k, sh, m;
	u64 v;

	for (q = 0; q <= RKVENC_QP_MAX; q++) {
		k = (int)q - 4;
		/* floor(k / 6), correct for negative k. */
		sh = (k - (k < 0 ? 5 : 0)) / 6;
		m = k - 6 * sh;
		if (sh >= 0)
			v = (u64)base[m] << sh;
		else
			v = (u64)base[m] >> (-sh);
		rkvenc_qs[q] = (u32)v;
	}
}

/* C11: the bucket holds Bs = T_avg * GOP length (one GOP of bits). */
static u32 rkvenc_rc_capacity(struct rkvenc_ctx *ctx)
{
	if (ctx->c.vbv_size)
		return ctx->c.vbv_size;
	return ctx->rc_tavg * max_t(u32, ctx->c.gop_size, 1);
}

/*
 * The plan budgets the I frame of a GOP, so the bucket is only on plan
 * at a GOP boundary: its deviation there is spread over the next GOP.
 */
static u64 rkvenc_rc_correct(struct rkvenc_ctx *ctx, u64 t_t, u32 n)
{
	s64 c = div_s64(ctx->rc_buf_gop, n);

	if (c < 0)
		return t_t + (u64)-c;
	if ((u64)c >= t_t)
		return 1;
	return t_t - (u64)c;
}

/* X_t = S * qs[q_avg] / 256. */
static u32 rkvenc_rc_complexity(struct rkvenc_ctx *ctx, unsigned int t, u64 s,
				u32 qp_avg)
{
	u32 x = (u32)div_u64(s * rkvenc_qs[min(qp_avg, RKVENC_QP_MAX)], 256);

	/* a P frame below the skip threshold alone would swing the model */
	if (!t || ctx->rc_first[1])
		return x;
	return (u32)div_u64((u64)ctx->rc_x[1] * (256 - RKVENC_RC_X_WEIGHT) +
				    (u64)x * RKVENC_RC_X_WEIGHT,
			    256);
}

/* Rounded mean QP of the P frames since the last I frame; resets it. */
static int rkvenc_rc_p_qp_mean(struct rkvenc_ctx *ctx)
{
	int q = ctx->rc_qp[1];

	if (ctx->rc_p_qp_cnt)
		q = DIV_ROUND_CLOSEST(ctx->rc_p_qp_sum, ctx->rc_p_qp_cnt);
	ctx->rc_p_qp_sum = 0;
	ctx->rc_p_qp_cnt = 0;
	return q;
}

void rkvenc_rc_init(struct rkvenc_ctx *ctx)
{
	u32 bitrate = ctx->c.bitrate ?: 4000000;
	u32 fps = ctx->fps ?: RKVENC_RC_DEFAULT_FPS;

	rkvenc_qs_build();

	ctx->rc_tavg = div_u64((u64)bitrate, fps);
	if (!ctx->rc_tavg)
		ctx->rc_tavg = 1;
	ctx->rc_buf = 0;
	ctx->rc_buf_gop = 0;
	ctx->rc_p_qp_sum = 0;
	ctx->rc_p_qp_cnt = 0;
	ctx->rc_x[0] = ctx->rc_x[1] = 0;
	ctx->rc_qp[0] = ctx->c.i_frame_qp;
	ctx->rc_qp[1] = ctx->c.p_frame_qp;
	ctx->rc_first[0] = true;
	ctx->rc_first[1] = true;
}

/*
 * Pick the QP for the next frame of type t (0 = I, 1 = P). Called from
 * device_run(); the chosen value is written to *qp.
 */
void rkvenc_rc_pick(struct rkvenc_ctx *ctx, u8 *qp, u8 frame_type)
{
	unsigned int t = frame_type ? 1 : 0;
	u8 lo = ctx->c.min_qp;
	u8 hi = ctx->c.max_qp;
	s32 prev = ctx->rc_qp[t];
	u32 target;
	int q;

	if (!ctx->rc_active) {
		*qp = frame_type ? ctx->c.p_frame_qp : ctx->c.i_frame_qp;
		return;
	}

	/* First frame of a type: no model yet, use the control QP. */
	if (ctx->rc_first[t]) {
		*qp = frame_type ? ctx->c.p_frame_qp : ctx->c.i_frame_qp;
		return;
	}

	if (!t)
		ctx->rc_buf_gop = ctx->rc_buf;

	/* An I frame coarser than the P frames before it shows as a pulse. */
	if (!t && !ctx->rc_first[1]) {
		/* the GOP mean: the last P QP alone makes I sizes swing */
		q = rkvenc_rc_p_qp_mean(ctx);
		if (ctx->codec == RKVENC_CODEC_HEVC)
			q -= RKVENC_RC_QP_DELTA_IP_HEVC;
		else
			q -= RKVENC_RC_QP_DELTA_IP_H264;
		q = clamp_t(int, q, lo, hi);
		ctx->rc_qp[0] = (s8)q;
		*qp = q;
		return;
	}

	/*
	 * T_I/T_P keep the GOP average at T_avg; the buffer correction
	 * over a GOP pulls the rate back toward target. qs_target is
	 * the quantiser step that, at the measured complexity, would
	 * have produced T bits; the smallest QP whose qs[] reaches it
	 * wins.
	 */
	if (ctx->rc_x[0] && ctx->rc_x[1]) {
		u64 w = div_u64((u64)ctx->rc_x[0] * 256,
				ctx->rc_x[1] ?: 1);
		u32 n = max_t(u32, ctx->c.gop_size, 1);
		u64 t_p, t_t;

		w = min_t(u64, w, RKVENC_RC_W_CLAMP);
		t_p = div64_u64((u64)ctx->rc_tavg * n * 256,
				w + 256 * (n - 1));
		t_t = t ? t_p : t_p * w / 256;
		/* Buffer correction: spend down an excess, save a deficit. */
		t_t = rkvenc_rc_correct(ctx, t_t, n);

		target = (u32)div_u64((u64)ctx->rc_x[t] * 256, t_t);

		for (q = 0; q <= RKVENC_QP_MAX; q++)
			if (rkvenc_qs[q] >= target)
				break;
		if (q > RKVENC_QP_MAX)
			q = RKVENC_QP_MAX;
	} else {
		q = prev;
	}

	q = clamp_t(int, q, lo, hi);
	/* C13: limit the per-type QP swing to avoid oscillation. */
	if (q > prev + RKVENC_RC_QP_STEP)
		q = prev + RKVENC_RC_QP_STEP;
	if (q < prev - RKVENC_RC_QP_STEP)
		q = prev - RKVENC_RC_QP_STEP;
	q = clamp_t(int, q, lo, hi);

	ctx->rc_qp[t] = (s8)q;
	if (t) {
		ctx->rc_p_qp_sum += q;
		ctx->rc_p_qp_cnt++;
	}
	*qp = q;
}

/*
 * Update the controller after a completed frame of type t produced
 * bytes = hdr + ST_BSL bits of output at average QP qp_avg.
 */
void rkvenc_rc_update(struct rkvenc_ctx *ctx, u32 bytes, u32 qp_avg,
		     u8 frame_type)
{
	unsigned int t = frame_type ? 1 : 0;
	u64 s = (u64)bytes * 8;		/* frame size in bits */
	s64 cap = max_t(u64, rkvenc_rc_capacity(ctx), 1);

	if (!ctx->rc_active)
		return;

	ctx->rc_x[t] = rkvenc_rc_complexity(ctx, t, s, qp_avg);

	/* Leaky bucket: f += S - T_avg, clamped to [-Bs, Bs]. */
	ctx->rc_buf += (s32)s - (s32)ctx->rc_tavg;
	ctx->rc_buf = clamp_t(s64, ctx->rc_buf, -cap, cap);

	ctx->rc_first[t] = false;
}

/*
 * A frame of type t overflowed its capture buffer of bytes at the picked
 * QP. The retry restarts from the control QP; the step limit alone would
 * need frames that the overflow keeps dropping.
 */
void rkvenc_rc_overflow(struct rkvenc_ctx *ctx, u32 bytes, u8 frame_type)
{
	s8 start = frame_type ? ctx->c.p_frame_qp : ctx->c.i_frame_qp;
	unsigned int t = frame_type ? 1 : 0;

	rkvenc_rc_update(ctx, bytes, ctx->cur_qp, frame_type);
	if (ctx->rc_qp[t] < start)
		ctx->rc_qp[t] = start;
}
