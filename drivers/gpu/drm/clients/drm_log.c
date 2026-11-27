// SPDX-License-Identifier: GPL-2.0 or MIT
/*
 * Copyright (c) 2024 Red Hat.
 * Author: Jocelyn Falempe <jfalempe@redhat.com>
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/iosys-map.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/types.h>

#include <drm/drm_client.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <drm/drm_rect.h>

#include "drm_client_internal.h"
#include "drm_draw_internal.h"
#include "drm_internal.h"

MODULE_AUTHOR("Jocelyn Falempe");
MODULE_DESCRIPTION("DRM boot logger");
MODULE_LICENSE("GPL");

static unsigned int scale = 1;
module_param(scale, uint, 0444);
MODULE_PARM_DESC(scale, "Integer scaling factor for drm_log, default is 1");

/**
 * DOC: overview
 *
 * This is a simple graphic logger, to print the kernel message on screen, until
 * a userspace application is able to take over.
 * It is only for debugging purpose.
 */

struct drm_log_scanout {
	struct drm_client_buffer *buffer;
	const struct font_desc *font;
	u32 rows;
	u32 columns;
	u32 scaled_font_h;
	u32 scaled_font_w;
	u32 line;
	u32 format;
	u32 px_width;
	u32 front_color;
	u32 prefix_color;
	u32 width;
	u32 height;
	struct drm_rect dirty;
	bool dirty_valid;
	/* 首次 flush 错误码；0 表示没有错误。只记录，在注销时打印。 */
	int flush_err;
};

struct drm_log {
	struct mutex lock;
	struct drm_client_dev client;
	struct console con;
	bool probed;
	/* 下一次允许提交 scanout 的时间点，用于合并高频日志写入 */
	unsigned long next_flush;
	/* init 失败原因（0 表示成功），在 console 注册后统一打印，避免写入路径自陷 */
	int init_err;
	/* write_thread 实际处理过的记录数，用于判断 console 是否真的被调用 */
	u32 n_drawn;
	u32 n_scanout;
	struct drm_log_scanout *scanout;
};

static struct drm_log *client_to_drm_log(struct drm_client_dev *client)
{
	return container_of(client, struct drm_log, client);
}

static struct drm_log *console_to_drm_log(struct console *con)
{
	return container_of(con, struct drm_log, con);
}

static void drm_log_blit(struct iosys_map *dst, unsigned int dst_pitch,
			 const u8 *src, unsigned int src_pitch,
			 u32 height, u32 width, u32 px_width, u32 color)
{
	switch (px_width) {
	case 2:
		drm_draw_blit16(dst, dst_pitch, src, src_pitch, height, width, scale, color);
		break;
	case 3:
		drm_draw_blit24(dst, dst_pitch, src, src_pitch, height, width, scale, color);
		break;
	case 4:
		drm_draw_blit32(dst, dst_pitch, src, src_pitch, height, width, scale, color);
		break;
	default:
		WARN_ONCE(1, "Can't blit with pixel width %d\n", px_width);
	}
}

/*
 * 把本次改动合并进待提交区域，由 drm_log_write_thread() 按固定间隔统一提交。
 *
 * 真实显示驱动（如 rockchipdrm）的 fb dirty 会触发一次阻塞式 atomic commit，
 * 每次 commit 至少等一个 vblank。若每画一行就提交一次，高频日志会让控制台
 * 永远追不上，屏幕上只剩一条缓慢下移的黑色擦除带。这里把一段时间内的所有
 * 行合并成一次提交，使控制台刷新速率与显示刷新率解耦。
 */
static void drm_log_mark_dirty(struct drm_log_scanout *scanout,
			       const struct drm_rect *rect)
{
	if (!scanout->dirty_valid) {
		scanout->dirty = *rect;
		scanout->dirty_valid = true;
		return;
	}

	scanout->dirty.x1 = min(scanout->dirty.x1, rect->x1);
	scanout->dirty.y1 = min(scanout->dirty.y1, rect->y1);
	scanout->dirty.x2 = max(scanout->dirty.x2, rect->x2);
	scanout->dirty.y2 = max(scanout->dirty.y2, rect->y2);
}

static void drm_log_clear_line(struct drm_log_scanout *scanout, u32 line)
{
	struct drm_framebuffer *fb = scanout->buffer->fb;
	unsigned long height = scanout->scaled_font_h;
	struct iosys_map map;
	struct drm_rect r = DRM_RECT_INIT(0, line * height, fb->width, height);

	if (drm_client_buffer_vmap_local(scanout->buffer, &map))
		return;
	iosys_map_memset(&map, r.y1 * fb->pitches[0], 0, height * fb->pitches[0]);
	drm_client_buffer_vunmap_local(scanout->buffer);
	drm_log_mark_dirty(scanout, &r);
}

static void drm_log_draw_line(struct drm_log_scanout *scanout, const char *s,
			      unsigned int len, unsigned int prefix_len)
{
	struct drm_framebuffer *fb = scanout->buffer->fb;
	struct iosys_map map;
	const struct font_desc *font = scanout->font;
	size_t font_pitch = DIV_ROUND_UP(font->width, 8);
	const u8 *src;
	u32 px_width = fb->format->cpp[0];
	struct drm_rect r = DRM_RECT_INIT(0, scanout->line * scanout->scaled_font_h,
					  fb->width, (scanout->line + 1) * scanout->scaled_font_h);
	u32 i;

	if (drm_client_buffer_vmap_local(scanout->buffer, &map))
		return;

	iosys_map_incr(&map, r.y1 * fb->pitches[0]);
	for (i = 0; i < len && i < scanout->columns; i++) {
		u32 color = (i < prefix_len) ? scanout->prefix_color : scanout->front_color;
		src = drm_draw_get_char_bitmap(font, s[i], font_pitch);
		drm_log_blit(&map, fb->pitches[0], src, font_pitch,
			     scanout->scaled_font_h, scanout->scaled_font_w,
			     px_width, color);
		iosys_map_incr(&map, scanout->scaled_font_w * px_width);
	}

	scanout->line++;
	if (scanout->line >= scanout->rows)
		scanout->line = 0;
	drm_client_buffer_vunmap_local(scanout->buffer);
	drm_log_mark_dirty(scanout, &r);
}

static void drm_log_draw_new_line(struct drm_log_scanout *scanout,
				  const char *s, unsigned int len, unsigned int prefix_len)
{
	if (scanout->line == 0) {
		drm_log_clear_line(scanout, 0);
		drm_log_clear_line(scanout, 1);
		drm_log_clear_line(scanout, 2);
	} else if (scanout->line + 2 < scanout->rows)
		drm_log_clear_line(scanout, scanout->line + 2);

	drm_log_draw_line(scanout, s, len, prefix_len);
}

/*
 * Depends on print_time() in printk.c
 * Timestamp is written with "[%5lu.%06lu]"
 */
#define TS_PREFIX_LEN 13

static void drm_log_draw_kmsg_record(struct drm_log_scanout *scanout,
				     const char *s, unsigned int len)
{
	u32 prefix_len = 0;

	if (len > TS_PREFIX_LEN && s[0] == '[' && s[6] == '.' && s[TS_PREFIX_LEN] == ']')
		prefix_len = TS_PREFIX_LEN + 1;

	/* do not print the ending \n character */
	if (s[len - 1] == '\n')
		len--;

	while (len > scanout->columns) {
		drm_log_draw_new_line(scanout, s, scanout->columns, prefix_len);
		s += scanout->columns;
		len -= scanout->columns;
		prefix_len = 0;
	}
	if (len)
		drm_log_draw_new_line(scanout, s, len, prefix_len);
}

static u32 drm_log_find_usable_format(struct drm_plane *plane)
{
	int i;

	for (i = 0; i < plane->format_count; i++)
		if (drm_draw_color_from_xrgb8888(0xffffff, plane->format_types[i]) != 0)
			return plane->format_types[i];
	return DRM_FORMAT_INVALID;
}

static int drm_log_setup_modeset(struct drm_client_dev *client,
				 struct drm_mode_set *mode_set,
				 struct drm_log_scanout *scanout)
{
	struct drm_crtc *crtc = mode_set->crtc;
	u32 width = mode_set->mode->hdisplay;
	u32 height = mode_set->mode->vdisplay;
	u32 format;

	scanout->font = get_default_font(width, height, NULL, NULL);
	if (!scanout->font)
		return -ENOENT;

	format = drm_log_find_usable_format(crtc->primary);
	if (format == DRM_FORMAT_INVALID)
		return -EINVAL;

	scanout->buffer = drm_client_framebuffer_create(client, width, height, format);
	if (IS_ERR(scanout->buffer)) {
		drm_warn(client->dev, "drm_log can't create framebuffer %d %d %p4cc\n",
			 width, height, &format);
		return -ENOMEM;
	}
	mode_set->fb = scanout->buffer->fb;
	scanout->width = width;
	scanout->height = height;
	scanout->scaled_font_h = scanout->font->height * scale;
	scanout->scaled_font_w = scanout->font->width * scale;
	scanout->rows = height / scanout->scaled_font_h;
	scanout->columns = width / scanout->scaled_font_w;
	scanout->front_color = drm_draw_color_from_xrgb8888(0xffffff, format);
	scanout->prefix_color = drm_draw_color_from_xrgb8888(0x4e9a06, format);
	return 0;
}

static int drm_log_count_modeset(struct drm_client_dev *client)
{
	struct drm_mode_set *mode_set;
	int count = 0;

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client)
		count++;
	mutex_unlock(&client->modeset_mutex);
	return count;
}

static void drm_log_init_client(struct drm_log *dlog)
{
	struct drm_client_dev *client = &dlog->client;
	struct drm_mode_set *mode_set;
	int i, max_modeset;
	int n_modeset = 0;

	dlog->probed = true;
	dlog->init_err = 0;

	if (drm_client_modeset_probe(client, 0, 0)) {
		dlog->init_err = -1;
		return;
	}

	max_modeset = drm_log_count_modeset(client);
	if (!max_modeset) {
		dlog->init_err = -2;
		return;
	}

	dlog->scanout = kcalloc(max_modeset, sizeof(*dlog->scanout), GFP_KERNEL);
	if (!dlog->scanout) {
		dlog->init_err = -3;
		return;
	}

	mutex_lock(&client->modeset_mutex);
	drm_client_for_each_modeset(mode_set, client) {
		if (!mode_set->mode)
			continue;
		if (drm_log_setup_modeset(client, mode_set, &dlog->scanout[n_modeset]))
			continue;
		n_modeset++;
	}
	mutex_unlock(&client->modeset_mutex);
	if (n_modeset == 0)
		goto err_nomodeset;

	if (drm_client_modeset_commit(client)) {
		/*
		 * 初始提交失败时 drm_log 会静默地没有任何 scanout，屏幕上也就
		 * 看不到任何内容；记录原因，在 console 注册后统一打印。
		 */
		dlog->init_err = -4;
		goto err_failed_commit;
	}

	dlog->n_scanout = n_modeset;
	return;

err_failed_commit:
	for (i = 0; i < n_modeset; i++)
		drm_client_framebuffer_delete(dlog->scanout[i].buffer);

err_nomodeset:
	dlog->init_err = -5;
	kfree(dlog->scanout);
	dlog->scanout = NULL;
}

static void drm_log_free_scanout(struct drm_client_dev *client)
{
	struct drm_log *dlog = client_to_drm_log(client);
	int i;

	if (dlog->n_scanout) {
		for (i = 0; i < dlog->n_scanout; i++)
			drm_client_framebuffer_delete(dlog->scanout[i].buffer);
		dlog->n_scanout = 0;
		kfree(dlog->scanout);
		dlog->scanout = NULL;
	}
}

static void drm_log_client_unregister(struct drm_client_dev *client)
{
	struct drm_log *dlog = client_to_drm_log(client);
	struct drm_device *dev = client->dev;
	int i;

	unregister_console(&dlog->con);

	mutex_lock(&dlog->lock);
	for (i = 0; i < dlog->n_scanout; i++) {
		if (dlog->scanout[i].flush_err)
			drm_warn(dev, "drm_log: scanout %d flush failed (%d)\n",
				 i, dlog->scanout[i].flush_err);
	}
	drm_info(dev, "drm_log: unregister, %u scanout(s), %u records drawn\n",
		 dlog->n_scanout, dlog->n_drawn);
	drm_log_free_scanout(client);
	drm_client_release(client);
	mutex_unlock(&dlog->lock);
	kfree(dlog);
	drm_dbg(dev, "Unregistered with drm log\n");
}

static int drm_log_client_hotplug(struct drm_client_dev *client)
{
	struct drm_log *dlog = client_to_drm_log(client);

	mutex_lock(&dlog->lock);
	drm_log_free_scanout(client);
	dlog->probed = false;
	mutex_unlock(&dlog->lock);
	return 0;
}

static int drm_log_client_suspend(struct drm_client_dev *client, bool _console_lock)
{
	struct drm_log *dlog = client_to_drm_log(client);

	console_suspend(&dlog->con);

	return 0;
}

static int drm_log_client_resume(struct drm_client_dev *client, bool _console_lock)
{
	struct drm_log *dlog = client_to_drm_log(client);

	console_resume(&dlog->con);

	return 0;
}

static const struct drm_client_funcs drm_log_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= drm_log_client_unregister,
	.hotplug	= drm_log_client_hotplug,
	.suspend	= drm_log_client_suspend,
	.resume		= drm_log_client_resume,
};

/*
 * 周期性提交 scanout 的间隔（毫秒），0 表示不做周期性提交。
 *
 * rockchip 的 VOP 一直扫描同一块 framebuffer，直接写内存下一帧就能看到；
 * drm_atomic_helper_dirtyfb() 只是一次阻塞式 atomic commit（还要等 vblank），
 * 既不会让内容更早可见，又把控制台刷新率压到二十几帧，写入线程还要在 commit
 * 里空等一个 vblank，滚动看起来一卡一卡的。默认不提交，让显示控制器按自己的
 * 刷新率取最新内容；带 PSR/自刷新、必须显式 damage 通知的面板可以设成 >0。
 */
static int drm_log_flush_interval_ms;
module_param_named(flush_interval_ms, drm_log_flush_interval_ms, int, 0644);
MODULE_PARM_DESC(flush_interval_ms,
		 "Periodic scanout flush interval in ms (0 = never flush, default)");

static void drm_log_flush_dirty(struct drm_log *dlog)
{
	int i;

	for (i = 0; i < dlog->n_scanout; i++) {
		struct drm_log_scanout *scanout = &dlog->scanout[i];
		struct drm_rect dirty;
		int ret;

		if (!scanout->dirty_valid)
			continue;

		dirty = scanout->dirty;
		scanout->dirty_valid = false;
		ret = drm_client_framebuffer_flush(scanout->buffer, &dirty);
		/*
		 * 这里位于 console 写入路径，且在 device_lock 内，不能 printk
		 * （会递归进入本 console 并重复加锁）。只记录首个错误，注销时打印。
		 */
		if (ret && !scanout->flush_err)
			scanout->flush_err = ret;
	}
}

static void drm_log_write_thread(struct console *con, struct nbcon_write_context *wctxt)
{
	struct drm_log *dlog = console_to_drm_log(con);
	int i;

	if (!dlog->probed)
		drm_log_init_client(dlog);

	/* Check that we are still the master before drawing */
	if (!drm_master_internal_acquire(dlog->client.dev))
		return;
	drm_master_internal_release(dlog->client.dev);

	/*
	 * 本回调由 printk 的 nbcon 线程上下文调用，进入前已经持有 device_lock
	 * （drm_log_lock()），即 dlog->lock 已在本线程手里。
	 */
	dlog->n_drawn++;
	for (i = 0; i < dlog->n_scanout; i++)
		drm_log_draw_kmsg_record(&dlog->scanout[i], wctxt->outbuf, wctxt->len);

	/*
	 * 按固定间隔统一提交（默认关闭，见 drm_log_flush_interval_ms）。真实
	 * 显示驱动的 dirty 回调会做一次阻塞式 atomic commit（至少等一个 vblank），
	 * 而 scanout buffer 本来就被显示控制器持续扫描，周期性提交既不能更早
	 * 显示内容，又会让控制台刷新跟着 vblank 一卡一卡地走。
	 */
	if (drm_log_flush_interval_ms > 0 &&
	    time_after_eq(jiffies, dlog->next_flush)) {
		drm_log_flush_dirty(dlog);
		dlog->next_flush = jiffies +
			msecs_to_jiffies(drm_log_flush_interval_ms);
	}
}

static void drm_log_lock(struct console *con, unsigned long *flags)
{
	struct drm_log *dlog = console_to_drm_log(con);

	mutex_lock(&dlog->lock);
	migrate_disable();
}

static void drm_log_unlock(struct console *con, unsigned long flags)
{
	struct drm_log *dlog = console_to_drm_log(con);

	migrate_enable();
	mutex_unlock(&dlog->lock);
}

static void drm_log_register_console(struct console *con)
{
	strscpy(con->name, "drm_log");
	con->write_thread = drm_log_write_thread;
	con->device_lock = drm_log_lock;
	con->device_unlock = drm_log_unlock;
	con->flags = CON_PRINTBUFFER | CON_NBCON;
	con->index = -1;

	register_console(con);
}

/**
 * drm_log_register() - Register a drm device to drm_log
 * @dev: the drm device to register.
 */
void drm_log_register(struct drm_device *dev)
{
	struct drm_log *new;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		goto err_warn;

	mutex_init(&new->lock);
	new->next_flush = jiffies;
	if (drm_client_init(dev, &new->client, "drm_log", &drm_log_client_funcs))
		goto err_free;

	drm_client_register(&new->client);

	drm_log_register_console(&new->con);

	/*
	 * 在 console 注册之后打印初始化结果：drm_log_init_client() 运行在
	 * console 写入路径里，不能在那里直接 printk，否则会递归回本 console。
	 */
	drm_info(dev,
		 "drm_log: init_err=%d, n_scanout=%u, scanout_ok=%d, mode=%ux%u\n",
		 new->init_err, new->n_scanout, new->scanout != NULL,
		 new->scanout ? new->scanout[0].width : 0,
		 new->scanout ? new->scanout[0].height : 0);

	drm_dbg(dev, "Registered with drm log as %s\n", new->con.name);
	return;

err_free:
	kfree(new);
err_warn:
	drm_warn(dev, "Failed to register with drm log\n");
}
