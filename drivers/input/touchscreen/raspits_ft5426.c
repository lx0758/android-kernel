// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT5426 touchscreen driver for the Raspberry Pi 7" panel.
 *
 * The board's DSI connector exposes only I2C and no touch interrupt line,
 * so this driver polls the controller at a fixed rate instead of using an
 * IRQ.  Coordinates and orientation are taken from the device tree.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/workqueue.h>

#include "raspits_ft5426.h"

struct ft5426_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct delayed_work work;
	struct mutex lock;	/* protects stopping and work scheduling */
	bool stopping;
	bool touch_down;
	u16 last_x;
	u16 last_y;
};

static int ft5426_read(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == 2 ? 0 : -EIO;
}

static int ft5426_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	return ft5426_read(client, reg, val, 1);
}

static int ft5426_detect(struct i2c_client *client)
{
	u8 version;
	int ret = -ENODEV;
	int i;

	for (i = 0; i < 5; i++) {
		ret = ft5426_read_reg(client, FT5426_REG_FW_VER, &version);
		if (!ret)
			return 0;
		if (ret != -ENXIO && ret != -EAGAIN)
			break;
		msleep(100);
	}

	/*
	 * -ENXIO/-EAGAIN can mean the I2C bus is not usable yet (for example
	 * the panel is still powering up); let the driver core retry later.
	 * Any other error means the controller is not there.
	 */
	if (ret == -ENXIO || ret == -EAGAIN) {
		dev_dbg(&client->dev, "FT5426 not ready yet: %d\n", ret);
		return -EPROBE_DEFER;
	}

	dev_err(&client->dev, "FT5426 not responding: %d\n", ret);
	return -ENODEV;
}

static void ft5426_report(struct ft5426_data *ts, bool down, u16 x, u16 y)
{
	struct input_dev *input = ts->input;

	if (down) {
		input_mt_slot(input, 0);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
		touchscreen_report_pos(input, &ts->prop, x, y, true);
	} else {
		input_mt_slot(input, 0);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
	}

	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
}

static void ft5426_work(struct work_struct *work)
{
	struct ft5426_data *ts = container_of(to_delayed_work(work),
					      struct ft5426_data, work);
	struct i2c_client *client = ts->client;
	u8 status, buf[4];
	u16 x, y;
	int ret;

	mutex_lock(&ts->lock);
	if (ts->stopping)
		goto out_unlock;

	ret = ft5426_read_reg(client, FT5426_REG_TD_STATUS, &status);
	if (ret) {
		dev_dbg_ratelimited(&client->dev, "status read failed: %d\n", ret);
		goto out_resched;
	}
	status &= 0x0f;

	if (status > 0 && status < 0x0f) {
		ret = ft5426_read(client, FT5426_REG_P1_XH, buf, sizeof(buf));
		if (ret) {
			dev_dbg_ratelimited(&client->dev, "point read failed: %d\n", ret);
			goto out_resched;
		}

		x = ((buf[0] & 0x0f) << 8) | buf[1];
		y = ((buf[2] & 0x0f) << 8) | buf[3];

		if (!ts->touch_down || x != ts->last_x || y != ts->last_y)
			ft5426_report(ts, true, x, y);

		ts->touch_down = true;
		ts->last_x = x;
		ts->last_y = y;
	} else if (ts->touch_down) {
		ft5426_report(ts, false, 0, 0);
		ts->touch_down = false;
	}

out_resched:
	if (!ts->stopping)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(FT5426_POLL_INTERVAL_MS));
out_unlock:
	mutex_unlock(&ts->lock);
}

static int ft5426_probe(struct i2c_client *client)
{
	struct ft5426_data *ts;
	struct input_dev *input;
	int ret;

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	mutex_init(&ts->lock);
	i2c_set_clientdata(client, ts);

	ret = ft5426_detect(client);
	if (ret)
		return ret;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "ft5426-touch";
	input->id.bustype = BUS_I2C;

	input_set_capability(input, EV_KEY, BTN_TOUCH);

	/*
	 * The axes must be registered before touchscreen_parse_properties(),
	 * otherwise touchscreen_set_params() refuses to apply the DT values
	 * and prop.max_x/max_y stay zero.
	 */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y) {
		dev_err(&client->dev, "missing touchscreen-size-x/y in DT\n");
		return -EINVAL;
	}

	input_set_abs_params(input, ABS_X, 0, ts->prop.max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ts->prop.max_y, 0, 0);

	ret = input_mt_init_slots(input, FT5426_MAX_TOUCH_POINTS,
				  INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&ts->work, ft5426_work);
	schedule_delayed_work(&ts->work, 0);

	return 0;
}

static void ft5426_stop(struct ft5426_data *ts)
{
	mutex_lock(&ts->lock);
	ts->stopping = true;
	mutex_unlock(&ts->lock);

	cancel_delayed_work_sync(&ts->work);
}

static void ft5426_remove(struct i2c_client *client)
{
	struct ft5426_data *ts = i2c_get_clientdata(client);

	ft5426_stop(ts);
	input_unregister_device(ts->input);
}

static int ft5426_suspend(struct device *dev)
{
	struct ft5426_data *ts = i2c_get_clientdata(to_i2c_client(dev));

	ft5426_stop(ts);

	mutex_lock(&ts->lock);
	if (ts->touch_down) {
		ft5426_report(ts, false, 0, 0);
		ts->touch_down = false;
	}
	mutex_unlock(&ts->lock);

	return 0;
}

static int ft5426_resume(struct device *dev)
{
	struct ft5426_data *ts = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&ts->lock);
	ts->stopping = false;
	ts->touch_down = false;
	mutex_unlock(&ts->lock);

	schedule_delayed_work(&ts->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ft5426_pm_ops, ft5426_suspend, ft5426_resume);

static const struct i2c_device_id ft5426_id[] = {
	{ "raspits_ft5426", 0 },
	{ "ft5426", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5426_id);

static const struct of_device_id ft5426_of_match[] = {
	{ .compatible = "focaltech,ft5426" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft5426_of_match);

static struct i2c_driver ft5426_driver = {
	.driver = {
		.name = "raspits_ft5426",
		.of_match_table = ft5426_of_match,
		.pm = &ft5426_pm_ops,
	},
	.probe = ft5426_probe,
	.remove = ft5426_remove,
	.id_table = ft5426_id,
};
module_i2c_driver(ft5426_driver);

MODULE_DESCRIPTION("FocalTech FT5426 touchscreen for Raspberry Pi 7\" panel");
MODULE_LICENSE("GPL v2");
