// SPDX-License-Identifier: GPL-2.0-only
/*
 * V4L2 stateful mem-to-mem front end for the Rockchip RKVENC v1 encoder.
 *
 * This file is the thin V4L2 (and only V4L2) layer: it exposes the OUTPUT
 * (raw) and CAPTURE (coded) queues, the standard encoder controls, the
 * format/sizing negotiation and the m2m job hook that hands a fully formed
 * struct rkvenc_frame to the codec-neutral engine. All coding-structure
 * knowledge (GOP, reference picture management, rate control, parameter
 * sets) lives in the engine and is driven here through rkvenc_*.c.
 *
 * Coding structure (see design/02-architecture.md): low-delay IDR+P, one
 * short-term reference, POC type 0, no B frames. The parameter sets for a
 * stream are generated once and copied (never regenerated differently)
 * into every IDR; in SEPARATE_HEADER_MODE they are emitted as the first
 * CAPTURE buffer, in JOINED_WITH_1ST_FRAME they are prepended to the first
 * IDR's coded slice data.
 */

#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "rkvenc.h"

static inline struct rkvenc_ctx *fh_to_rkvenc_ctx(void *priv)
{
	return container_of(priv, struct rkvenc_ctx, fh);
}

/* ----------------------------------------------------------------------- */
/* Format tables                                                            */
/* ----------------------------------------------------------------------- */

struct rkvenc_fmt {
	u32 fourcc;
	u8 num_planes;
};

/* OUTPUT (raw) formats: NV12/NV12M，或交给硬件 CSC 的 RGBA。 */
static const struct rkvenc_fmt rkvenc_output_fmts[] = {
	{ V4L2_PIX_FMT_ABGR32, 1 },
	{ V4L2_PIX_FMT_NV12, 1 },
	{ V4L2_PIX_FMT_NV12M, 2 },
};

/* CAPTURE (coded) formats. */
static const struct rkvenc_fmt rkvenc_capture_fmts[] = {
	{ V4L2_PIX_FMT_H264, 1 },
	{ V4L2_PIX_FMT_HEVC, 1 },
};

static const struct rkvenc_fmt *
rkvenc_find_fmt(const struct rkvenc_fmt *fmts, unsigned int n, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];
	return NULL;
}

static bool rkvenc_is_h264(u32 fourcc)
{
	return fourcc == V4L2_PIX_FMT_H264;
}

/* ----------------------------------------------------------------------- */
/* Control snapshot helpers                                                 */
/* ----------------------------------------------------------------------- */

static u32 rkvenc_ctrl_val(struct rkvenc_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (!ctrl)
		return 0;
	return ctrl->cur.val;
}

/*
 * Read the live control values into the per-stream snapshot ctx->c and
 * derive rc_active. Called from device_run() (every frame) so runtime
 * control changes (QP limits, GOP size, level, ...) take effect on the
 * next frame.
 */
static void rkvenc_apply_ctrls(struct rkvenc_ctx *ctx)
{
	struct rkvenc_ctrls *c = &ctx->c;
	bool h264 = ctx->codec == RKVENC_CODEC_H264;

	c->bitrate_mode = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_BITRATE_MODE);
	c->bitrate = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_BITRATE);
	c->vbv_size = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_VBV_SIZE);
	c->frame_rc_enable =
		rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE);
	c->gop_size = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_GOP_SIZE);
	c->i_period = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD);
	c->header_mode = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEADER_MODE);
	c->prepend_spspps =
		rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR);

	if (h264) {
		c->profile = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_PROFILE);
		c->level = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_LEVEL);
		c->cabac = rkvenc_ctrl_val(ctx,
				V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE) ==
				V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC;
		c->transform_8x8 =
			rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM);
		c->lf_mode =
			rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE);
		c->lf_alpha =
			rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA);
		c->lf_beta =
			rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA);
		c->chroma_qp_offset =
			rkvenc_ctrl_val(ctx,
				V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET);
		c->min_qp = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_MIN_QP);
		c->max_qp = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_MAX_QP);
		c->i_frame_qp =
			rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP);
		c->p_frame_qp =
			rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP);
		/* Either GOP_SIZE or I_PERIOD may drive IDR cadence. */
		if (rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD) >
		    c->gop_size)
			c->gop_size =
				rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_H264_I_PERIOD);
	} else {
		c->level = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL);
		c->tier = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_TIER);
		c->lf_mode =
			rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE);
		c->hevc_beta_offset = rkvenc_ctrl_val(
			ctx, V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2);
		c->hevc_tc_offset = rkvenc_ctrl_val(
			ctx, V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2);
		c->min_qp = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP);
		c->max_qp = rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP);
		c->i_frame_qp =
			rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP);
		c->p_frame_qp =
			rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP);
		c->hevc_max_merge =
			rkvenc_ctrl_val(ctx,
				V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1);
		/* No HEVC_SAO control in this uAPI revision; SAO tracks the
		 * loop-filter mode (a disabled loop filter implies no SAO). */
		c->hevc_sao =
			c->lf_mode != V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED;
		c->hevc_strong_intra =
			rkvenc_ctrl_val(ctx,
				V4L2_CID_MPEG_VIDEO_HEVC_STRONG_SMOOTHING);
		c->hevc_tmv =
			rkvenc_ctrl_val(ctx,
				V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION);
		/* HEVC bitstreams are always CABAC; no cabac_init control. */
		c->hevc_cabac_init = 0;
		/* Either GOP_SIZE or REFRESH_PERIOD may drive IDR cadence. */
		if (rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD) >
		    c->gop_size)
			c->gop_size =
				rkvenc_ctrl_val(ctx,
					V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD);
	}

	c->framerate = ctx->fps ?: 30;
	ctx->rc_active = c->frame_rc_enable;
}

/* Generate (or refresh) the parameter sets cached in ctx->hdr/hdr_len. */
static void rkvenc_refresh_header(struct rkvenc_ctx *ctx)
{
	rkvenc_apply_ctrls(ctx);
	ctx->hdr_len = ctx->codec == RKVENC_CODEC_H264 ?
		rkvenc_h264_setup(ctx) : rkvenc_hevc_setup(ctx);
}

static void rkvenc_ensure_header(struct rkvenc_ctx *ctx)
{
	if (ctx->hdr_len == 0)
		rkvenc_refresh_header(ctx);
}

static int rkvenc_copy_headers(struct rkvenc_ctx *ctx, u8 *buf, u32 size)
{
	return ctx->codec == RKVENC_CODEC_H264 ?
		rkvenc_h264_write_headers(ctx, buf, size) :
		rkvenc_hevc_write_headers(ctx, buf, size);
}

static void rkvenc_update_blocks(struct rkvenc_ctx *ctx)
{
	if (ctx->codec == RKVENC_CODEC_H264) {
		ctx->blocks = (ALIGN(ctx->width, 16) / 16) *
			      (ALIGN(ctx->height, 16) / 16);
		ctx->qp_blocks = ctx->blocks;
	} else {
		ctx->blocks = (ALIGN(ctx->width, 64) / 64) *
			      (ALIGN(ctx->height, 64) / 64);
		ctx->qp_blocks = (ALIGN(ctx->width, 8) / 8) *
				 (ALIGN(ctx->height, 8) / 8);
	}
}

/* ----------------------------------------------------------------------- */
/* Sizing                                                                   */
/* ----------------------------------------------------------------------- */

static u32 rkvenc_output_plane_size(struct rkvenc_ctx *ctx, unsigned int plane)
{
	u32 bpl = ctx->bytesperline;
	u32 h = ctx->height;

	/* RGBA_8888 是单平面 4 字节/像素，色度由硬件 CSC 生成。 */
	if (ctx->src_fourcc == V4L2_PIX_FMT_ABGR32)
		return bpl * h;

	/* NV12: luma + interleaved chroma share the buffer; the chroma
	 * starts at bpl * h. NV12M: two distinct planes. */
	if (ctx->src_num_planes == 1)
		return bpl * h * 3 / 2;
	return plane == 0 ? bpl * h : bpl * h / 2;
}

static u32 rkvenc_capture_size(struct rkvenc_ctx *ctx)
{
	if (ctx->codec == RKVENC_CODEC_H264) {
		u32 mbw = ALIGN(ctx->width, 16) / 16;
		u32 mbh = ALIGN(ctx->height, 16) / 16;

		return min(mbw * mbh * 400, ctx->width * ctx->height / 2) +
		       RKVENC_HDR_MAX;
	}
	return ctx->width * ctx->height / 2 + RKVENC_HDR_MAX;
}

/* ----------------------------------------------------------------------- */
/* vb2 queue callbacks                                                      */
/* ----------------------------------------------------------------------- */

static int rkvenc_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			      unsigned int *num_planes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vq);
	const struct rkvenc_fmt *fmt;
	unsigned int i;
	u32 size;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		fmt = rkvenc_find_fmt(rkvenc_output_fmts,
				      ARRAY_SIZE(rkvenc_output_fmts),
				      ctx->src_fourcc);
		if (!fmt)
			return -EINVAL;
		*num_planes = fmt->num_planes;
		for (i = 0; i < *num_planes; i++)
			sizes[i] = rkvenc_output_plane_size(ctx, i);
		return 0;
	}

	fmt = rkvenc_find_fmt(rkvenc_capture_fmts,
			      ARRAY_SIZE(rkvenc_capture_fmts),
			      ctx->coded_fourcc);
	if (!fmt)
		return -EINVAL;
	size = rkvenc_capture_size(ctx);
	*num_planes = 1;
	if (*num_buffers)
		sizes[0] = max(sizes[0], size);
	else
		sizes[0] = size;
	return 0;
}

static int rkvenc_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		if (vb2_plane_size(vb, 0) < rkvenc_output_plane_size(ctx, 0))
			return -EINVAL;
	} else {
		if (vb2_plane_size(vb, 0) < rkvenc_capture_size(ctx))
			return -EINVAL;
	}
	return 0;
}

static void rkvenc_buf_queue(struct vb2_buffer *vb)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type)) {
		/* Drain: mark the last queued capture buffer with LAST. */
		if (vb2_is_streaming(vb->vb2_queue) &&
		    v4l2_m2m_dst_buf_is_last(ctx->m2m_ctx)) {
			vb2_set_plane_payload(vb, 0, 0);
			vbuf->field = V4L2_FIELD_NONE;
			v4l2_m2m_last_buffer_done(ctx->m2m_ctx, vbuf);
			return;
		}

		/* SEPARATE header: emit the cached parameter sets as the
		 * first CAPTURE buffer, without consuming a source frame. */
		if (ctx->header_pending &&
		    ctx->c.header_mode ==
			V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE) {
			u8 *vaddr = vb2_plane_vaddr(vb, 0);

			if (vaddr) {
				rkvenc_ensure_header(ctx);
				if (rkvenc_copy_headers(ctx, vaddr,
							vb2_plane_size(vb, 0)) < 0) {
					vb2_set_plane_payload(vb, 0, 0);
				} else {
					vb2_set_plane_payload(vb, 0,
							     ctx->hdr_len);
					/* 该缓冲之后还会被硬件复用，先把 CPU
					 * 的写入同步出去，避免脏行延迟回写
					 * 覆盖后续帧的码流。 */
					dma_sync_single_for_device(
						ctx->dev->dev,
						vb2_dma_contig_plane_dma_addr(vb, 0),
						ctx->hdr_len, DMA_TO_DEVICE);
				}
			} else {
				vb2_set_plane_payload(vb, 0, 0);
			}
			vbuf->field = V4L2_FIELD_NONE;
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
			ctx->header_pending = false;
			return;
		}
	}

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vbuf);
}

static int rkvenc_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void rkvenc_return_dst_bufs(struct rkvenc_ctx *ctx)
{
	struct vb2_v4l2_buffer *vbuf;

	while ((vbuf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx)))
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_QUEUED);
}

static int rkvenc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	/* Both formats must be set before the capture queue streams on. */
	if (!ctx->coded_fourcc || !ctx->src_fourcc) {
		rkvenc_return_dst_bufs(ctx);
		return -EINVAL;
	}

	/* H.264 pictures up to two macroblocks wide and two rows high hang. */
	if (ctx->codec == RKVENC_CODEC_H264 && ALIGN(ctx->width, 16) <= 32 &&
	    ALIGN(ctx->height, 16) > 16) {
		rkvenc_return_dst_bufs(ctx);
		return -EINVAL;
	}

	rkvenc_update_blocks(ctx);
	rkvenc_refresh_header(ctx);

	ret = rkvenc_ref_alloc(ctx);
	if (ret) {
		rkvenc_return_dst_bufs(ctx);
		return ret;
	}

	rkvenc_rc_init(ctx);
	rkvenc_gop_init(ctx);
	ctx->joined_header_done = false;
	return 0;
}

static void rkvenc_stop_streaming(struct vb2_queue *q)
{
	struct rkvenc_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;

	v4l2_m2m_update_stop_streaming_state(ctx->m2m_ctx, q);
	rkvenc_dvfs_ctx_idle(ctx->dev, ctx);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		while ((vbuf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	} else {
		while ((vbuf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		rkvenc_ref_free(ctx);
	}
}

static const struct vb2_ops rkvenc_queue_ops = {
	.queue_setup = rkvenc_queue_setup,
	.buf_prepare = rkvenc_buf_prepare,
	.buf_queue = rkvenc_buf_queue,
	.buf_out_validate = rkvenc_buf_out_validate,
	.start_streaming = rkvenc_start_streaming,
	.stop_streaming = rkvenc_stop_streaming,
};

/* ----------------------------------------------------------------------- */
/* m2m job hook                                                             */
/* ----------------------------------------------------------------------- */

static void rkvenc_device_run(void *priv)
{
	struct rkvenc_ctx *ctx = priv;
	struct rkvenc_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;
	struct rkvenc_frame f = {};
	u8 qp;
	bool idr;
	u32 hdr_len = 0;
	int ret;

	dev->job_ctx = ctx;
	dev->job_running = true;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (!src || !dst)
		goto finish_error;

	idr = rkvenc_gop_is_idr(ctx);

	/*
	 * The IDR frame is frame #0 of its GOP, so the per-picture
	 * numbering (frame_num / poc_lsb) must start from zero. Reset the
	 * counter here, before build_pic() reads it (design/02 12.2).
	 */
	if (idr)
		ctx->gop.frames_since_idr = 0;

	rkvenc_apply_ctrls(ctx);
	rkvenc_rc_pick(ctx, &qp, idr ? 0 : 1);
	ctx->cur_qp = qp;
	ctx->cur_intra = idr;

	if (idr)
		rkvenc_refresh_header(ctx);

	/*
	 * JOINED_WITH_1ST_FRAME: 参数集要拼在码流前面，但**不能**在这里写进
	 * capture 缓冲。这块内存同时被硬件 DMA 写：CPU 侧一次 35 字节的拷贝
	 * 会把整条 64 字节 cache line 弄脏，而这行脏数据可能在硬件写完切片
	 * 之后才被回写，从而把硬件刚写好的起始码和 slice header 覆盖成 0
	 * （实测表现为 IDR 包只剩 SPS+PPS，解码器报 Invalid data）。
	 *
	 * 因此这里只记录需要拼接的长度，真正的拷贝放到 job 完成之后
	 * （rkvenc_finish_job -> rkvenc_prepend_headers），让 CPU 成为最后
	 * 一个写这块内存的人。
	 */
	if (ctx->c.header_mode ==
		V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME &&
	    idr && (!ctx->joined_header_done || ctx->c.prepend_spspps)) {
		ctx->joined_header_done = true;
		hdr_len = min_t(u32, ctx->hdr_len,
				vb2_plane_size(&dst->vb2_buf, 0));
	}
	ctx->cur_prepend = hdr_len;

	f.codec = ctx->codec;
	f.intra = idr;
	f.referenced = true;
	f.src.width = ctx->width;
	f.src.height = ctx->height;
	f.src.stride[0] = ctx->bytesperline;
	f.src.stride[1] = ctx->bytesperline;
	f.src.rgb = ctx->src_fourcc == V4L2_PIX_FMT_ABGR32;
	f.src.rbuv_swap = f.src.rgb;
	f.src.plane[0] = lower_32_bits(
		vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0));
	if (f.src.rgb) {
		/* RGB 单平面：色度由硬件 CSC 生成，不使用 ADR_SRC1/2。 */
		f.src.plane[1] = 0;
		f.src.plane[2] = 0;
	} else if (ctx->src_num_planes == 1) {
		f.src.plane[1] = f.src.plane[0] + ctx->bytesperline * ctx->height;
	} else {
		f.src.plane[1] = lower_32_bits(
			vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 1) +
			src->vb2_buf.planes[1].data_offset);
	}

	f.bs.base = lower_32_bits(
		vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));
	/* 拼接参数集需要一个 hdr_len 的头部空间，硬件只能写它之后的部分。 */
	f.bs.size = vb2_plane_size(&dst->vb2_buf, 0) - hdr_len;
	f.bs.offset = 0;

	/* Slot ping-pong (design/02 12.2): write to cur, reference last
	 * (or cur for an IDR); gop_commit flips them after a good frame. */
	f.recon = ctx->slot[ctx->cur];
	f.ref = idr ? ctx->slot[ctx->cur] : ctx->slot[ctx->last];

	f.qp = qp;
	f.qp_min = ctx->c.min_qp;
	f.qp_max = ctx->c.max_qp;
	/* Hardware row-level RC is left disabled in this series. */
	f.rc = NULL;
	f.wdg_vs_load = 0;

	if (f.codec == RKVENC_CODEC_H264)
		rkvenc_h264_build_pic(ctx, &f.h264, idr);
	else
		rkvenc_hevc_build_pic(ctx, &f.hevc, idr);

	rkvenc_dvfs_job_begin(dev);

	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret)
		goto finish_error_idle;

	ret = rkvenc_hw_run(dev, &f);
	if (ret) {
		pm_runtime_put(dev->dev);
		goto finish_error_idle;
	}
	return;

finish_error_idle:
	rkvenc_dvfs_job_end(dev, ctx);

finish_error:
	dev->job_running = false;
	dev->job_ctx = NULL;
	if (src && dst)
		v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->m2m_ctx,
						 VB2_BUF_STATE_ERROR);
	else
		v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);
}

static int rkvenc_job_ready(void *priv)
{
	struct rkvenc_ctx *ctx = priv;

	return rkvenc_dvfs_start_allowed(ctx->dev);
}

const struct v4l2_m2m_ops rkvenc_m2m_ops = {
	.device_run = rkvenc_device_run,
	.job_ready = rkvenc_job_ready,
};

/* ----------------------------------------------------------------------- */
/* Controls                                                                 */
/* ----------------------------------------------------------------------- */

static int rkvenc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rkvenc_ctx *ctx =
		container_of(ctrl->handler, struct rkvenc_ctx, ctrl_hdl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		rkvenc_gop_force_keyframe(ctx);
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		/* Switching to SEPARATE_HEADER_MODE re-emits the parameter sets
		 * on the next stream; JOINED stays valid for the running one. */
		if (ctrl->val == V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE)
			ctx->header_pending = true;
		break;
	default:
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops rkvenc_ctrl_ops = {
	.s_ctrl = rkvenc_s_ctrl,
};

static int rkvenc_init_ctrls(struct rkvenc_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrl_hdl;

	v4l2_ctrl_handler_init(hdl, 48);

	/* Bitrate / rate control. Only CBR is wired to the engine. */
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
			       ~(1 << V4L2_MPEG_VIDEO_BITRATE_MODE_CBR),
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_BITRATE,
			  25600, 40 * 1000 * 1000, 1, 4 * 1000 * 1000);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0, 1, 1, 1);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_VBV_SIZE,
			  0, 1 << 24, 1, 0);

	/* GOP / key frames. */
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			  0, 300, 1, 30);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 0, 300, 1, 30);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_PERIOD, 0, 300, 1, 30);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_REFRESH_TYPE,
			       V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR,
			       ~(1 << V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR),
			       V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);

	/* Header placement. */
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			       0,
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
			  0, 1, 1, 0);

	/* H.264 specific. */
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			       ~((1 << V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
				 (1 << V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
				 (1 << V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)),
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_5_2,
			       0, V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, 0,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std_menu(
		hdl, &rkvenc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
		V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY,
		0, V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET,
			  -12, 12, 1, 0);

	/* HEVC specific. */
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN, 0,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
			       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2,
			       0, V4L2_MPEG_VIDEO_HEVC_LEVEL_4);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_TIER,
			       V4L2_MPEG_VIDEO_HEVC_TIER_HIGH,
			       ~(1 << V4L2_MPEG_VIDEO_HEVC_TIER_MAIN),
			       V4L2_MPEG_VIDEO_HEVC_TIER_MAIN);
	v4l2_ctrl_new_std_menu(hdl, &rkvenc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE,
			       V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED,
			       0,
			       V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_LF_BETA_OFFSET_DIV2,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_LF_TC_OFFSET_DIV2,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_STRONG_SMOOTHING,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_TMV_PREDICTION,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_MAX_NUM_MERGE_MV_MINUS1,
			  0, 4, 1, 2);

	/* QP controls (per codec; the driver applies only the one matching
	 * the active coded format). */
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
			  1, 51, 1, 1);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			  1, 51, 1, 51);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 1, 51, 1, 26);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 1, 51, 1, 28);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
			  1, 51, 1, 1);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
			  1, 51, 1, 51);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 1, 51, 1, 26);
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP, 1, 51, 1, 28);

	/* Read-only average QP (updated by the IRQ thread). */
	v4l2_ctrl_new_std(hdl, &rkvenc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_AVERAGE_QP, 0, 51, 1, 0);

	if (hdl->error)
		return hdl->error;

	v4l2_ctrl_handler_setup(hdl);

	/* With SEPARATE the first CAPTURE buffer carries the parameter
	 * sets. */
	ctx->header_pending =
		rkvenc_ctrl_val(ctx, V4L2_CID_MPEG_VIDEO_HEADER_MODE) ==
		V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE;

	ctx->fh.ctrl_handler = hdl;
	return 0;
}

/* ----------------------------------------------------------------------- */
/* Format negotiation                                                       */
/* ----------------------------------------------------------------------- */

static int rkvenc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);
	struct rkvenc_dev *dev = ctx->dev;

	strscpy(cap->driver, "rkvenc", sizeof(cap->driver));
	strscpy(cap->card, "rockchip-rkvenc", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(dev->dev));

	return 0;
}

static int rkvenc_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f, bool capture)
{
	const struct rkvenc_fmt *fmts = capture ? rkvenc_capture_fmts
						: rkvenc_output_fmts;
	unsigned int n = capture ? ARRAY_SIZE(rkvenc_capture_fmts)
				 : ARRAY_SIZE(rkvenc_output_fmts);

	if (f->index >= n)
		return -EINVAL;
	f->pixelformat = fmts[f->index].fourcc;
	f->flags = 0;
	return 0;
}

static int rkvenc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return rkvenc_enum_fmt(file, priv, f, false);
}

static int rkvenc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return rkvenc_enum_fmt(file, priv, f, true);
}

static void rkvenc_try_fmt_vid_out(struct rkvenc_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct rkvenc_fmt *fmt;
	u32 bpl;

	fmt = rkvenc_find_fmt(rkvenc_output_fmts,
			      ARRAY_SIZE(rkvenc_output_fmts),
			      pix->pixelformat);
	if (!fmt) {
		fmt = &rkvenc_output_fmts[0];
		pix->pixelformat = fmt->fourcc;
	}

	pix->field = V4L2_FIELD_NONE;
	pix->width = clamp(round_up(pix->width, 2), 16, ctx->dev->max_width);
	pix->height = clamp(round_up(pix->height, 2), 16, ctx->dev->max_height);
	pix->num_planes = fmt->num_planes;

	if (fmt->fourcc == V4L2_PIX_FMT_ABGR32) {
		/* Android RGBA_8888：每像素 4 字节，单平面。 */
		bpl = pix->plane_fmt[0].bytesperline;
		if (bpl < pix->width * 4)
			bpl = pix->width * 4;
		pix->plane_fmt[0].bytesperline = bpl;
		pix->plane_fmt[0].sizeimage = bpl * pix->height;
		return;
	}

	bpl = ALIGN(pix->width, 16);
	pix->plane_fmt[0].bytesperline = bpl;
	if (fmt->num_planes == 2) {
		pix->plane_fmt[0].sizeimage = bpl * pix->height;
		pix->plane_fmt[1].bytesperline = bpl;
		pix->plane_fmt[1].sizeimage = bpl * pix->height / 2;
	} else {
		pix->plane_fmt[0].sizeimage = bpl * pix->height * 3 / 2;
	}
}

static void rkvenc_try_fmt_vid_cap(struct rkvenc_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct rkvenc_fmt *fmt;

	fmt = rkvenc_find_fmt(rkvenc_capture_fmts,
			      ARRAY_SIZE(rkvenc_capture_fmts),
			      pix->pixelformat);
	if (!fmt) {
		fmt = &rkvenc_capture_fmts[0];
		pix->pixelformat = fmt->fourcc;
	}

	pix->field = V4L2_FIELD_NONE;
	pix->width = clamp(round_up(pix->width, 2), 16, ctx->dev->max_width);
	pix->height = clamp(round_up(pix->height, 2), 16, ctx->dev->max_height);
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = 0;

	if (fmt->fourcc == V4L2_PIX_FMT_H264) {
		u32 mbw = ALIGN(pix->width, 16) / 16;
		u32 mbh = ALIGN(pix->height, 16) / 16;

		pix->plane_fmt[0].sizeimage =
			min(mbw * mbh * 400, pix->width * pix->height / 2) +
			RKVENC_HDR_MAX;
	} else {
		pix->plane_fmt[0].sizeimage =
			pix->width * pix->height / 2 + RKVENC_HDR_MAX;
	}
}

static int rkvenc_try_fmt_vid_out_ioctl(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);

	rkvenc_try_fmt_vid_out(ctx, f);
	return 0;
}

static int rkvenc_try_fmt_vid_cap_ioctl(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);

	rkvenc_try_fmt_vid_cap(ctx, f);
	return 0;
}

static int rkvenc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;

	rkvenc_try_fmt_vid_out(ctx, f);
	ctx->src_fourcc = f->fmt.pix_mp.pixelformat;
	ctx->src_num_planes = f->fmt.pix_mp.num_planes;
	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;
	ctx->bytesperline = f->fmt.pix_mp.plane_fmt[0].bytesperline;
	return 0;
}

static int rkvenc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;

	rkvenc_try_fmt_vid_cap(ctx, f);
	ctx->coded_fourcc = f->fmt.pix_mp.pixelformat;
	ctx->codec = rkvenc_is_h264(f->fmt.pix_mp.pixelformat) ?
		RKVENC_CODEC_H264 : RKVENC_CODEC_HEVC;
	return 0;
}

static int rkvenc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->pixelformat = ctx->src_fourcc;
		pix->num_planes = ctx->src_num_planes;
	} else {
		pix->pixelformat = ctx->coded_fourcc;
		pix->num_planes = 1;
	}
	pix->width = ctx->width;
	pix->height = ctx->height;
	pix->field = V4L2_FIELD_NONE;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->plane_fmt[0].bytesperline = ctx->bytesperline;
		pix->plane_fmt[0].sizeimage =
			rkvenc_output_plane_size(ctx, 0);
		if (ctx->src_num_planes == 2) {
			pix->plane_fmt[1].bytesperline = ctx->bytesperline;
			pix->plane_fmt[1].sizeimage =
				rkvenc_output_plane_size(ctx, 1);
		}
	} else {
		pix->plane_fmt[0].sizeimage = rkvenc_capture_size(ctx);
	}
	return 0;
}

static int rkvenc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *a)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);
	struct v4l2_fract *tpf = &a->parm.output.timeperframe;
	unsigned int fps;

	if (!V4L2_TYPE_IS_MULTIPLANAR(a->type))
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0) {
		tpf->numerator = 1;
		tpf->denominator = 30;
	}
	fps = tpf->denominator / tpf->numerator;
	if (fps < 1)
		fps = 1;
	ctx->fps = fps;
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	return 0;
}

static int rkvenc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *a)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);

	if (!V4L2_TYPE_IS_MULTIPLANAR(a->type))
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe.numerator = 1;
	a->parm.output.timeperframe.denominator = ctx->fps ?: 30;
	return 0;
}

/*
 * 有状态 V4L2 编码客户端（如 v4l2_codec2）在 S_FMT 之后会用 VIDIOC_S_SELECTION
 * 声明可见矩形，用来表达"协商尺寸里真正要编码的区域"。若驱动不支持该 ioctl，
 * 客户端会退到 VIDIOC_S_CROP，两者都失败时直接放弃创建编码器。
 *
 * 本驱动与硬件都按 S_FMT 协商出的尺寸出流，编码前没有真实的裁剪级。因此这里
 * 只接受与当前编码尺寸完全一致的可见矩形；更小的矩形会被明确拒绝，避免出现
 * 声明尺寸与实际码流尺寸不一致的静默错误。
 */
static int rkvenc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (!ctx->width || !ctx->height)
			return -EINVAL;
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->width;
		s->r.height = ctx->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rkvenc_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(file->private_data);

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;
	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (!ctx->width || !ctx->height)
		return -EINVAL;

	if (s->r.left || s->r.top ||
	    s->r.width != ctx->width || s->r.height != ctx->height) {
		dev_warn_ratelimited(ctx->dev->dev,
				     "cropping not supported: %ux%u+%d+%d, coded size %ux%u\n",
				     s->r.width, s->r.height, s->r.left, s->r.top,
				     ctx->width, ctx->height);
		return -EINVAL;
	}

	return 0;
}

static int rkvenc_vidioc_encoder_cmd(struct file *file, void *priv,
				     struct v4l2_encoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_ENC_CMD_STOP && cmd->cmd != V4L2_ENC_CMD_START)
		return -EINVAL;
	return v4l2_m2m_ioctl_encoder_cmd(file, priv, cmd);
}

static int rkvenc_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct rkvenc_ctx *ctx = priv;
	struct rkvenc_dev *dev = ctx->dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &rkvenc_queue_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->dma_attrs = DMA_ATTR_NO_KERNEL_MAPPING;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &dev->vdev_lock;
	src_vq->dev = dev->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &rkvenc_queue_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &dev->vdev_lock;
	dst_vq->dev = dev->v4l2_dev.dev;
	return vb2_queue_init(dst_vq);
}

static int rkvenc_open(struct file *filp)
{
	struct rkvenc_dev *dev = video_drvdata(filp);
	struct rkvenc_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->codec = RKVENC_CODEC_H264;
	ctx->coded_fourcc = V4L2_PIX_FMT_H264;
	ctx->src_fourcc = V4L2_PIX_FMT_NV12;
	ctx->src_num_planes = 1;
	ctx->width = 1920;
	ctx->height = 1080;
	ctx->bytesperline = ALIGN(1920, 16);
	ctx->fps = 30;
	mutex_init(&ctx->vlock);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, rkvenc_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto err_fh;
	}
	ctx->fh.m2m_ctx = ctx->m2m_ctx;

	ret = rkvenc_init_ctrls(ctx);
	if (ret)
		goto err_m2m;

	rkvenc_dvfs_ctx_add(dev, ctx);
	v4l2_fh_add(&ctx->fh, filp);
	return 0;

err_m2m:
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
err_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int rkvenc_release(struct file *filp)
{
	struct rkvenc_ctx *ctx = fh_to_rkvenc_ctx(filp->private_data);

	rkvenc_dvfs_ctx_del(ctx->dev, ctx);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_del(&ctx->fh, filp);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_ioctl_ops rkvenc_v4l2_ioctl_ops = {
	.vidioc_querycap = rkvenc_querycap,

	.vidioc_enum_fmt_vid_cap = rkvenc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out = rkvenc_enum_fmt_vid_out,
	.vidioc_try_fmt_vid_cap_mplane = rkvenc_try_fmt_vid_cap_ioctl,
	.vidioc_try_fmt_vid_out_mplane = rkvenc_try_fmt_vid_out_ioctl,
	.vidioc_s_fmt_vid_cap_mplane = rkvenc_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_out_mplane = rkvenc_s_fmt_vid_out,
	.vidioc_g_fmt_vid_cap_mplane = rkvenc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = rkvenc_g_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_s_parm = rkvenc_s_parm,
	.vidioc_g_parm = rkvenc_g_parm,

	.vidioc_g_selection = rkvenc_g_selection,
	.vidioc_s_selection = rkvenc_s_selection,

	.vidioc_encoder_cmd = rkvenc_vidioc_encoder_cmd,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations rkvenc_v4l2_fops = {
	.owner = THIS_MODULE,
	.open = rkvenc_open,
	.release = rkvenc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

int rkvenc_v4l2_init(struct rkvenc_dev *dev)
{
	struct video_device *vdev = &dev->vdev;
	int ret;

	dev->mdev.dev = dev->dev;
	strscpy(dev->mdev.model, "rkvenc", sizeof(dev->mdev.model));
	strscpy(dev->mdev.bus_info, "platform:rkvenc",
		sizeof(dev->mdev.bus_info));
	media_device_init(&dev->mdev);
	dev->v4l2_dev.mdev = &dev->mdev;

	vdev->lock = &dev->vdev_lock;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->fops = &rkvenc_v4l2_fops;
	vdev->release = video_device_release_empty;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	vdev->ioctl_ops = &rkvenc_v4l2_ioctl_ops;
	video_set_drvdata(vdev, dev);
	strscpy(vdev->name, "rkvenc", sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto err_cleanup_mc;
	}

	ret = v4l2_m2m_register_media_controller(dev->m2m_dev, vdev,
						MEDIA_ENT_F_PROC_VIDEO_ENCODER);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to init M2M media controller\n");
		goto err_unregister_vdev;
	}

	ret = media_device_register(&dev->mdev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register media device\n");
		goto err_unregister_mc;
	}

	return 0;

err_unregister_mc:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev);
err_unregister_vdev:
	video_unregister_device(vdev);
err_cleanup_mc:
	media_device_cleanup(&dev->mdev);
	return ret;
}

void rkvenc_v4l2_cleanup(struct rkvenc_dev *dev)
{
	media_device_unregister(&dev->mdev);
	v4l2_m2m_unregister_media_controller(dev->m2m_dev);
	video_unregister_device(&dev->vdev);
	media_device_cleanup(&dev->mdev);
}