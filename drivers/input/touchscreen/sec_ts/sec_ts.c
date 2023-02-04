/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct sec_ts_data *tsp_info;

#include "sec_ts.h"

/* Switch GPIO values */
#define SEC_SWITCH_GPIO_VALUE_SLPI_MASTER 	0
#define SEC_SWITCH_GPIO_VALUE_AP_MASTER 	1

struct sec_ts_data *ts_dup;

#ifndef CONFIG_SEC_SYSFS
/* Declare extern sec_class */
struct class *sec_class;
#endif

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work);
#endif
static void sec_ts_read_info_work(struct work_struct *work);
static void sec_ts_fw_update_work(struct work_struct *work);
static void sec_ts_suspend_work(struct work_struct *work);
static void sec_ts_resume_work(struct work_struct *work);

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev);
static void sec_ts_input_close(struct input_dev *dev);
#endif

int sec_ts_read_information(struct sec_ts_data *ts);


int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 *buf;
	int ret;
	unsigned char retry;
	struct i2c_msg msg;

	if (len + 1 > sizeof(ts->i2c_write_buf)) {
		input_err(true, &ts->client->dev, "%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	mutex_lock(&ts->i2c_mutex);

	buf = ts->i2c_write_buf;
	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if ((ret = i2c_transfer(ts->client->adapter, &msg, 1)) == 1)
			break;

		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
			mutex_unlock(&ts->i2c_mutex);
			goto err;
		}

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts->client->dev, "%s: I2C write over retry limit\n", __func__);
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going)
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif
	}

	if (ret == 1)
		return 0;
err:
	return -EIO;
}

static int sec_ts_i2c_read_internal(struct sec_ts_data *ts, u8 reg,
			     u8 *data, int len, bool dma_safe)
{
	u8 *buf;
	int ret;
	unsigned char retry;
	struct i2c_msg msg[2];
	int remain = len;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	if (len > sizeof(ts->i2c_read_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev, "%s: len %d over pre-allocated size %d\n",
			__func__, len, I2C_PREALLOC_READ_BUF_SZ);
		return -ENOSPC;
	}

	mutex_lock(&ts->i2c_mutex);

	buf = ts->i2c_write_buf;
	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	if (dma_safe == false)
		msg[1].buf = ts->i2c_read_buf;
	else
		msg[1].buf = data;

	if (len <= ts->i2c_burstmax) {

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			if (ret == 2)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
				ts->comm_err_count++;
			}
		}

		if (ret == 2 && dma_safe == false)
			memcpy(data, ts->i2c_read_buf, len);

	} else {
		/*
		 * I2C read buffer is 256 byte. do not support long buffer over than 256.
		 * So, try to seperate reading data about 256 bytes.
		 */

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
				ts->comm_err_count++;
			}
		}

		do {
			if (remain > ts->i2c_burstmax)
				msg[1].len = ts->i2c_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->i2c_burstmax;

			for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
				ret = i2c_transfer(ts->client->adapter, &msg[1], 1);
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
					mutex_unlock(&ts->i2c_mutex);
					goto err;
				}

				if (retry > 1) {
					input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
					ts->comm_err_count++;
				}
			}

			msg[1].buf += msg[1].len;

		} while (remain > 0);

		if (ret == 1 && dma_safe == false)
			memcpy(data, ts->i2c_read_buf, len);
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts->client->dev, "%s: I2C read over retry limit\n", __func__);
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going)
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif

	}

	return ret;

err:
	return -EIO;
}

static int sec_ts_i2c_write_burst_internal(struct sec_ts_data *ts,
					   u8 *data, int len, bool dma_safe)
{
	int ret;
	int retry;

	if (len > sizeof(ts->i2c_write_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev, "%s: len %d over pre-allocated size %d\n",
			__func__, len, sizeof(ts->i2c_write_buf));
		return -ENOSPC;
	}

	mutex_lock(&ts->i2c_mutex);

	if (dma_safe == false) {
		memcpy(ts->i2c_write_buf, data, len);
		data = ts->i2c_write_buf;
	}

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if ((ret = i2c_master_send(ts->client, data, len)) == len)
			break;

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->i2c_mutex);
	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts->client->dev, "%s: I2C write over retry limit\n", __func__);
		ret = -EIO;
	}

	return ret;
}

static int sec_ts_i2c_read_bulk_internal(struct sec_ts_data *ts,
					 u8 *data, int len, bool dma_safe)
{
	int ret;
	unsigned char retry;
	int remain = len;
	struct i2c_msg msg;

	if (len > sizeof(ts->i2c_read_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev,
			  "%s: len %d over pre-allocated size %d\n", __func__,
			  len, sizeof(ts->i2c_read_buf));
		return -ENOSPC;
	}

	mutex_lock(&ts->i2c_mutex);

	msg.addr = ts->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	if (dma_safe == false)
		msg.buf = ts->i2c_read_buf;
	else
		msg.buf = data;

	do {
		if (remain > ts->i2c_burstmax)
			msg.len = ts->i2c_burstmax;
		else
			msg.len = remain;

		remain -= ts->i2c_burstmax;

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, &msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);

			if (retry > 1) {
				input_err(true, &ts->client->dev, "%s: I2C retry %d\n", __func__, retry + 1);
				ts->comm_err_count++;
			}
		}

		if (retry == SEC_TS_I2C_RETRY_CNT) {
			input_err(true, &ts->client->dev,
				  "%s: I2C read over retry limit\n", __func__);
			ret = -EIO;
			break;
		}

		msg.buf += msg.len;

	} while (remain > 0);

	if (ret == 1 && dma_safe == false)
		memcpy(data, ts->i2c_read_buf, len);

	mutex_unlock(&ts->i2c_mutex);

	if (ret == 1)
		return 0;

	return -EIO;
}

/* Wrapper API for i2c read and write */
int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	return sec_ts_i2c_read_internal(ts, reg, data, len, false);
}

int sec_ts_i2c_read_heap(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	return sec_ts_i2c_read_internal(ts, reg, data, len, true);
}

int sec_ts_i2c_write_burst(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_i2c_write_burst_internal(ts, data, len, false);
}

int sec_ts_i2c_write_burst_heap(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_i2c_write_burst_internal(ts, data, len, true);
}

int sec_ts_i2c_read_bulk(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_i2c_read_bulk_internal(ts, data, len, false);
}

int sec_ts_i2c_read_bulk_heap(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_i2c_read_bulk_internal(ts, data, len, true);
}

static int sec_ts_read_from_customlib(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: fail to read custom library command\n", __func__);

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, (u8 *)data, len);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: fail to read custom library command\n", __func__);

	return ret;
}

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
#include <linux/sec_debug.h>
extern struct tsp_dump_callbacks dump_callbacks;
static struct delayed_work *p_ghost_check;

static void sec_ts_check_rawdata(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data, ghost_check.work);

	if (ts->tsp_dump_lock == 1) {
		input_err(true, &ts->client->dev, "%s: ignored ## already checking..\n", __func__);
		return;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: ignored ## IC is power off\n", __func__);
		return;
	}

	ts->tsp_dump_lock = 1;
	input_info(true, &ts->client->dev, "%s: start ##\n", __func__);
	sec_ts_run_rawdata_all((void *)ts, false);
	msleep(100);

	input_info(true, &ts->client->dev, "%s: done ##\n", __func__);
	ts->tsp_dump_lock = 0;

}

static void dump_tsp_log(void)
{
	pr_info("%s: %s %s: start\n", SEC_TS_I2C_NAME, SECLOG, __func__);

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		pr_err("%s: %s %s: ignored ## lpm charging Mode!!\n", SEC_TS_I2C_NAME, SECLOG, __func__);
		return;
	}
#endif

	if (p_ghost_check == NULL) {
		pr_err("%s: %s %s: ignored ## tsp probe fail!!\n", SEC_TS_I2C_NAME, SECLOG, __func__);
		return;
	}
	schedule_delayed_work(p_ghost_check, msecs_to_jiffies(100));
}
#endif


void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack)
{
	int rc = -1;
	int retry = 0;
	u8 tBuff[SEC_TS_EVENT_BUFF_SIZE] = {0,};

	while (sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, tBuff, SEC_TS_EVENT_BUFF_SIZE)) {
		if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		} else if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_VENDOR_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		}

		if (retry++ > SEC_TS_WAIT_RETRY_CNT) {
			input_err(true, &ts->client->dev, "%s: Time Over\n", __func__);
			break;
		}
		sec_ts_delay(20);
	}

	input_info(true, &ts->client->dev,
		"%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X [%d]\n",
		__func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3],
		tBuff[4], tBuff[5], tBuff[6], tBuff[7], retry);

	return rc;
}

int sec_ts_read_calibration_report(struct sec_ts_data *ts)
{
	int ret;
	u8 buf[5] = { 0 };

	buf[0] = SEC_TS_READ_CALIBRATION_REPORT;

	ret = sec_ts_i2c_read(ts, buf[0], &buf[1], 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to read, %d\n", __func__, ret);
		return ret;
	}

	input_info(true, &ts->client->dev, "%s: count:%d, pass count:%d, fail count:%d, status:0x%X\n",
				__func__, buf[1], buf[2], buf[3], buf[4]);

	return buf[4];
}

static void sec_ts_reinit(struct sec_ts_data *ts)
{
	u8 w_data[2] = {0x00, 0x00};
	int ret = 0;

	input_info(true, &ts->client->dev,
				"%s : charger=0x%x, Cover=0x%x, Power mode=0x%x\n",
				__func__, ts->charger_mode, ts->touch_functions, ts->lowpower_status);

	/* charger mode */
	if (ts->charger_mode != SEC_TS_BIT_CHARGER_MODE_NO) {
		w_data[0] = ts->charger_mode;
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to send command(0x%x)",
						__func__, SET_TS_CMD_SET_CHARGER_MODE);
	}

	/* Cover mode */
	if (ts->touch_functions & SEC_TS_BIT_SETFUNC_COVER) {
		w_data[0] = ts->cover_cmd;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to send command(0x%x)",
						__func__, SEC_TS_CMD_SET_COVERTYPE);

		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to send command(0x%x)",
						__func__, SEC_TS_CMD_SET_TOUCHFUNCTION);
	}

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	/* Power mode */
	if (ts->lowpower_status == TO_LOWPOWER_MODE) {
		w_data[0] = (ts->lowpower_mode & SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to send command(0x%x)",
						__func__, SEC_TS_CMD_WAKEUP_GESTURE_MODE);

		w_data[0] = TO_LOWPOWER_MODE;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to send command(0x%x)",
						__func__, SEC_TS_CMD_SET_POWER_MODE);

		sec_ts_delay(50);

		if (ts->lowpower_mode & SEC_TS_MODE_CUSTOMLIB_AOD) {
			int i, ret;
			u8 data[10] = {0x02, 0};
		
			for (i = 0; i < 4; i++) {
				data[i * 2 + 2] = ts->rect_data[i] & 0xFF;
				data[i * 2 + 3] = (ts->rect_data[i] >> 8) & 0xFF;
			}

			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, &data[0], 10);
			if (ret < 0)
				input_err(true, &ts->client->dev, "%s: Failed to write offset\n", __func__);
		
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
			if (ret < 0)
				input_err(true, &ts->client->dev, "%s: Failed to send notify\n", __func__);

		}

	} else {

		sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

		if (ts->dex_mode) {
			input_info(true, &ts->client->dev, "%s: set dex mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DEX_MODE, &ts->dex_mode, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: failed to set dex mode %x\n", __func__, ts->dex_mode);
		}

		if (ts->brush_mode) {
			input_info(true, &ts->client->dev, "%s: set brush mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_BRUSH_MODE, &ts->brush_mode, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
							"%s: failed to set brush mode\n", __func__);
		}

		if (ts->touchable_area) {
			input_info(true, &ts->client->dev, "%s: set 16:9 mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA, &ts->touchable_area, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
							"%s: failed to set 16:9 mode\n", __func__);
		}

	}
	return;
}

#define MAX_EVENT_COUNT 32
static void sec_ts_read_event(struct sec_ts_data *ts)
{
	int ret;
	u8 t_id;
	u8 event_id;
	u8 left_event_count;
	u8 read_event_buff[MAX_EVENT_COUNT][SEC_TS_EVENT_BUFF_SIZE] = { { 0 } };
	u8 *event_buff;
	struct sec_ts_event_coordinate *p_event_coord;
	struct sec_ts_gesture_status *p_gesture_status;
	struct sec_ts_event_status *p_event_status;
	int curr_pos;
	int remain_event_count = 0;

	if (ts->power_status == SEC_TS_STATE_LPM) {

		pm_wakeup_event(&ts->client->dev, 3 * MSEC_PER_SEC);
		/* waiting for blsp block resuming, if not occurs i2c error */
		ret = wait_for_completion_interruptible_timeout(&ts->resume_done, msecs_to_jiffies(3 * MSEC_PER_SEC));
		if (ret == 0) {
			input_err(true, &ts->client->dev, "%s: LPM: pm resume is not handled\n", __func__);
			return;
		}

		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: LPM: -ERESTARTSYS if interrupted, %d\n", __func__, ret);
			return;
		}

		input_info(true, &ts->client->dev, "%s: run LPM interrupt handler, %d\n", __func__, ret);
		/* run lpm interrupt handler */
	}

	ret = t_id = event_id = curr_pos = remain_event_count = 0;
	/* repeat READ_ONE_EVENT until buffer is empty(No event) */
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, (u8 *)read_event_buff[0], SEC_TS_EVENT_BUFF_SIZE);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: i2c read one event failed\n", __func__);
		return;
	}

	if (ts->temp == 0x01)
		input_info(true, &ts->client->dev, "ONE: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			read_event_buff[0][0], read_event_buff[0][1], read_event_buff[0][2], read_event_buff[0][3],
			read_event_buff[0][4], read_event_buff[0][5], read_event_buff[0][6], read_event_buff[0][7]);

	if (read_event_buff[0][0] == 0) {
		input_info(true, &ts->client->dev, "%s: event buffer is empty\n", __func__);
		return;
	}

	left_event_count = read_event_buff[0][7] & 0x3F;
	remain_event_count = left_event_count;

	if (left_event_count > MAX_EVENT_COUNT - 1 || left_event_count == 0xFF) {
		input_err(true, &ts->client->dev, "%s: event buffer overflow\n", __func__);

		/* write clear event stack command when read_event_count > MAX_EVENT_COUNT */
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: i2c write clear event failed\n", __func__);
		return;
	}

	if (left_event_count > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ALL_EVENT, (u8 *)read_event_buff[1],
				sizeof(u8) * (SEC_TS_EVENT_BUFF_SIZE) * (left_event_count));
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: i2c read one event failed\n", __func__);
			return;
		}
	}

	do {
		s16 max_force_p = 0;
		event_buff = read_event_buff[curr_pos];
		event_id = event_buff[0] & 0x3;

		if (ts->temp == 0x01)
			input_info(true, &ts->client->dev, "ALL: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				event_buff[0], event_buff[1], event_buff[2], event_buff[3],
				event_buff[4], event_buff[5], event_buff[6], event_buff[7]);

		switch (event_id) {
		case SEC_TS_STATUS_EVENT:
			p_event_status = (struct sec_ts_event_status *)event_buff;

			/* tchsta == 0 && ttype == 0 && eid == 0 : buffer empty */
			if (p_event_status->stype > 0) {
				/* Demote 'vendor' messages */
				if (p_event_status->stype ==
				    TYPE_STATUS_EVENT_VENDOR_INFO)
					input_dbg(true, &ts->client->dev,
						"%s: STATUS %x %x %x %x %x %x %x %x\n",
						__func__, event_buff[0],
						event_buff[1], event_buff[2],
						event_buff[3], event_buff[4],
						event_buff[5], event_buff[6],
						event_buff[7]);
				else
					input_info(true, &ts->client->dev,
						"%s: STATUS %x %x %x %x %x %x %x %x\n",
						__func__, event_buff[0],
						event_buff[1], event_buff[2],
						event_buff[3], event_buff[4],
						event_buff[5], event_buff[6],
						event_buff[7]);
			}

			/* watchdog reset -> send SENSEON command */ /*=>?????*/
			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_BOOT_COMPLETE) &&
				(p_event_status->status_data_1 == 0x20)) {

				sec_ts_unlocked_release_all_finger(ts);

				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
				if (ret < 0)
					input_err(true, &ts->client->dev, "%s: fail to write Sense_on\n", __func__);

				sec_ts_reinit(ts);
			}

			/* event queue full-> all finger release */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_QUEUE_FULL)) {
				input_err(true, &ts->client->dev, "%s: IC Event Queue is full\n", __func__);
				sec_ts_unlocked_release_all_finger(ts);
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_ESD)) {
				input_err(true, &ts->client->dev, "%s: ESD detected. run reset\n", __func__);
#ifdef USE_RESET_DURING_POWER_ON
				schedule_work(&ts->reset_work.work);
#endif
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_WET_MODE)) {
				ts->wet_mode = p_event_status->status_data_1;
				input_info(true, &ts->client->dev, "%s: water wet mode %d\n",
					__func__, ts->wet_mode);
				if (ts->wet_mode)
					ts->wet_count++;

				}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_CUSTOMLIB_INFO) &&
				(p_event_status->status_id == SEC_TS_EVENT_CUSTOMLIB_FORCE_KEY)) {
				if (ts->power_status == SEC_TS_STATE_POWER_ON) {
					if (p_event_status->status_data_1 & SEC_TS_CUSTOMLIB_EVENT_PRESSURE_TOUCHED) {
						ts->all_force_count++;
						ts->scrub_id = CUSTOMLIB_EVENT_TYPE_PRESSURE_TOUCHED;
					} else {
						if (ts->scrub_id == CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_PRESS) {
							input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
							ts->scrub_id = CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RELEASE;
						} else {
							ts->scrub_id = CUSTOMLIB_EVENT_TYPE_PRESSURE_RELEASED;
						}
					}

					input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 1);
				} else {
					if (p_event_status->status_data_1 & SEC_TS_CUSTOMLIB_EVENT_PRESSURE_RELEASED) {
						input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
						input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 1);
						ts->scrub_id = CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RELEASE_NO_HAPTIC;
						input_sync(ts->input_dev);

						haptic_homekey_release();
					} else {
						input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);
						input_sync(ts->input_dev);

						ts->scrub_id = CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_PRESS;
						haptic_homekey_press();
						ts->all_force_count++;
					}
				}

				ts->scrub_x = ((p_event_status->status_data_4 >> 4) & 0xF) << 8 | (p_event_status->status_data_3 & 0xFF);
				ts->scrub_y = ((p_event_status->status_data_4 >> 0) & 0xF) << 8 | (p_event_status->status_data_2 & 0xFF);

				input_info(true, &ts->client->dev, "%s: PRESSURE[%d]\n", __func__, ts->scrub_id);

				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 0);
			}

			break;

		case SEC_TS_COORDINATE_EVENT:
			if (ts->input_closed) {
				input_err(true, &ts->client->dev, "%s: device is closed\n", __func__);
				break;
			}
			p_event_coord = (struct sec_ts_event_coordinate *)event_buff;

			t_id = (p_event_coord->tid - 1);

			if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
				ts->coord[t_id].id = t_id;
				ts->coord[t_id].action = p_event_coord->tchsta;
				ts->coord[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
				ts->coord[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
				ts->coord[t_id].z = p_event_coord->z &
							SEC_TS_PRESSURE_MAX;
				ts->coord[t_id].ttype = p_event_coord->ttype_3_2 << 2 | p_event_coord->ttype_1_0 << 0;
				ts->coord[t_id].major = p_event_coord->major;
				ts->coord[t_id].minor = p_event_coord->minor;

				if (!ts->coord[t_id].palm && (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM))
					ts->coord[t_id].palm_count++;

				ts->coord[t_id].palm = (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM);
				ts->coord[t_id].left_event = p_event_coord->left_event;

				if (ts->coord[t_id].z <= 0)
					ts->coord[t_id].z = 1;

				if ((ts->coord[t_id].ttype ==
				     SEC_TS_TOUCHTYPE_NORMAL) ||
				    (ts->coord[t_id].ttype ==
				     SEC_TS_TOUCHTYPE_PALM) ||
				    (ts->coord[t_id].ttype ==
				     SEC_TS_TOUCHTYPE_WET) ||
				    (ts->coord[t_id].ttype ==
				     SEC_TS_TOUCHTYPE_GLOVE)) {

					if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
						u8 rbuf[2] = {0, };

						do_gettimeofday(&ts->time_released[t_id]);

						if (ts->time_longest < (ts->time_released[t_id].tv_sec - ts->time_pressed[t_id].tv_sec))
							ts->time_longest = (ts->time_released[t_id].tv_sec - ts->time_pressed[t_id].tv_sec);

						ret = sec_ts_i2c_read(ts, SEC_TS_READ_FORCE_SIG_MAX_VAL, rbuf, 2);
						if (ret < 0)
							input_err(true, &ts->client->dev,
									"%s: fail to read max_pressure data\n",
									__func__);
						else
							max_force_p = (rbuf[0] & 0xFF) << 8 | (rbuf[1] & 0xFF);

						input_mt_slot(ts->input_dev, t_id);
						if (ts->plat_data->support_mt_pressure)
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);

						if (ts->touch_count > 0)
							ts->touch_count--;
						if (ts->touch_count == 0) {
							input_report_key(ts->input_dev, BTN_TOUCH, 0);
							input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
							ts->check_multi = 0;
						}

					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS) {
						do_gettimeofday(&ts->time_pressed[t_id]);

						ts->touch_count++;
						ts->all_finger_count++;

						ts->max_z_value = max((unsigned int)ts->coord[t_id].z, ts->max_z_value);
						ts->min_z_value = min((unsigned int)ts->coord[t_id].z, ts->min_z_value);
						ts->sum_z_value += (unsigned int)ts->coord[t_id].z;

						input_mt_slot(ts->input_dev, t_id);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
						input_report_key(ts->input_dev, BTN_TOUCH, 1);
						input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);

						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->coord[t_id].x);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->coord[t_id].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->coord[t_id].major);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, ts->coord[t_id].minor);
						if (ts->brush_mode)
							input_report_abs(ts->input_dev, ABS_MT_CUSTOM, (ts->coord[t_id].z << 1) | ts->coord[t_id].palm);
						else
							input_report_abs(ts->input_dev, ABS_MT_CUSTOM, (BRUSH_Z_DATA << 1) | ts->coord[t_id].palm);
						if (ts->plat_data->support_mt_pressure)
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE, ts->coord[t_id].z);

						if ((ts->touch_count > 4) && (ts->check_multi == 0)) {
							ts->check_multi = 1;
							ts->multi_count++;
						}

					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_MOVE) {
						if ((ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GLOVE) && !ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = true;
							input_report_switch(ts->input_dev, SW_GLOVE, 1);
						} else if ((ts->coord[t_id].ttype != SEC_TS_TOUCHTYPE_GLOVE) && ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = false;
							input_report_switch(ts->input_dev, SW_GLOVE, 0);
						}

						input_mt_slot(ts->input_dev, t_id);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
						input_report_key(ts->input_dev, BTN_TOUCH, 1);
						input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);

						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->coord[t_id].x);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->coord[t_id].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->coord[t_id].major);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, ts->coord[t_id].minor);
						if (ts->brush_mode)
							input_report_abs(ts->input_dev, ABS_MT_CUSTOM, (ts->coord[t_id].z << 1) | ts->coord[t_id].palm);
						else
							input_report_abs(ts->input_dev, ABS_MT_CUSTOM, (BRUSH_Z_DATA << 1) | ts->coord[t_id].palm);

						if (ts->plat_data->support_mt_pressure)
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE, ts->coord[t_id].z);
						ts->coord[t_id].mcount++;
					} else {
						input_dbg(true, &ts->client->dev,
								"%s: do not support coordinate action(%d)\n", __func__, ts->coord[t_id].action);
					}
				} else {
					input_dbg(true, &ts->client->dev,
							"%s: do not support coordinate type(%d)\n", __func__, ts->coord[t_id].ttype);
				}
			} else {
				input_err(true, &ts->client->dev, "%s: tid(%d) is out of range\n", __func__, t_id);
			}
			break;

		case SEC_TS_GESTURE_EVENT:
			p_gesture_status = (struct sec_ts_gesture_status *)event_buff;
			if ((p_gesture_status->eid == 0x02) && (p_gesture_status->stype == 0x00)) {
				u8 customlib[3] = { 0 };

				ret = sec_ts_read_from_customlib(ts, customlib, 3);
				if (ret < 0)
					input_err(true, &ts->client->dev, "%s: fail to read custom library data\n", __func__);

				input_info(true, &ts->client->dev, "%s: Custom Library, %x, %x, %x\n",
							__func__, customlib[0], customlib[1], customlib[2]);

				if (p_gesture_status->gesture_id == SEC_TS_GESTURE_CODE_SPAY ||
					p_gesture_status->gesture_id == SEC_TS_GESTURE_CODE_DOUBLE_TAP) {
					/* will be fixed to data structure */
					if (customlib[1] & SEC_TS_MODE_CUSTOMLIB_AOD) {
						u8 data[5] = { 0x0A, 0x00, 0x00, 0x00, 0x00 };

						ret = sec_ts_read_from_customlib(ts, data, 5);
						if (ret < 0)
							input_err(true, &ts->client->dev, "%s: fail to read custom library data\n", __func__);

						if (data[4] & SEC_TS_AOD_GESTURE_DOUBLETAB)
							ts->scrub_id = CUSTOMLIB_EVENT_TYPE_AOD_DOUBLETAB;

						ts->scrub_x = (data[1] & 0xFF) << 8 | (data[0] & 0xFF);
						ts->scrub_y = (data[3] & 0xFF) << 8 | (data[2] & 0xFF);
						input_info(true, &ts->client->dev, "%s: aod: %d\n",
								__func__, ts->scrub_id);
						ts->all_aod_tap_count++;
					}
					if (customlib[1] & SEC_TS_MODE_CUSTOMLIB_SPAY) {
						ts->scrub_id = CUSTOMLIB_EVENT_TYPE_SPAY;
						input_info(true, &ts->client->dev, "%s: SPAY: %d\n",
									__func__, ts->scrub_id);
						ts->all_spay_count++;
					}
					input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 0);
				}
			}
			break;

		default:
			input_err(true, &ts->client->dev, "%s: unknown event %x %x %x %x %x %x\n", __func__,
					event_buff[0], event_buff[1], event_buff[2],
					event_buff[3], event_buff[4], event_buff[5]);
			break;
		}

		if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
			if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS) {
				input_dbg(false, &ts->client->dev,
					"%s[P] tID:%d x:%d y:%d z:%d major:%d minor:%d tc:%d type:%X\n",
					ts->dex_name,
					t_id, ts->coord[t_id].x, ts->coord[t_id].y, ts->coord[t_id].z,
					ts->coord[t_id].major, ts->coord[t_id].minor, ts->touch_count,
					ts->coord[t_id].ttype);

			} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
				input_dbg(false, &ts->client->dev,
					"%s[R] tID:%d mc:%d tc:%d lx:%d ly:%d f:%d v:%02X%02X cal:%02X(%02X) id(%d,%d) p:%d P%02XT%04X\n",
					ts->dex_name,
					t_id, ts->coord[t_id].mcount, ts->touch_count,
					ts->coord[t_id].x, ts->coord[t_id].y, max_force_p,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->nv, ts->tspid_val,
					ts->tspicid_val, ts->coord[t_id].palm_count,
					ts->cal_count, ts->tune_fix_ver );

				ts->coord[t_id].action = SEC_TS_COORDINATE_ACTION_NONE;
				ts->coord[t_id].mcount = 0;
				ts->coord[t_id].palm_count = 0;
				max_force_p = 0;
			}
		}

		curr_pos++;
		remain_event_count--;
	} while (remain_event_count >= 0);

	input_sync(ts->input_dev);
}

static irqreturn_t sec_ts_irq_thread(int irq, void *ptr)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)ptr;

	if (sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_IRQ, true) < 0) {
		/* Interrupt during bus suspend */
		input_info(true, &ts->client->dev,
			   "%s: Skipping stray interrupt since bus is suspended.\n",
			   __func__);
		return IRQ_HANDLED;
	}
	mutex_lock(&ts->eventlock);

	sec_ts_read_event(ts);

	mutex_unlock(&ts->eventlock);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_IRQ, false);

	return IRQ_HANDLED;
}

int get_tsp_status(void)
{
	return 0;
}
EXPORT_SYMBOL(get_tsp_status);

void sec_ts_set_charger(bool enable)
{
	return;
/*
	int ret;
	u8 noise_mode_on[] = {0x01};
	u8 noise_mode_off[] = {0x00};

	if (enable) {
		input_info(true, &ts->client->dev, "sec_ts_set_charger : charger CONNECTED!!\n");
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_NOISE_MODE, noise_mode_on, sizeof(noise_mode_on));
		if (ret < 0)
			input_err(true, &ts->client->dev, "sec_ts_set_charger: fail to write NOISE_ON\n");
	} else {
		input_info(true, &ts->client->dev, "sec_ts_set_charger : charger DISCONNECTED!!\n");
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_NOISE_MODE, noise_mode_off, sizeof(noise_mode_off));
		if (ret < 0)
			input_err(true, &ts->client->dev, "sec_ts_set_charger: fail to write NOISE_OFF\n");
	}
 */
}
EXPORT_SYMBOL(sec_ts_set_charger);

int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode)
{
	int ret;

	if (mode)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_GLOVE | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_GLOVE)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: pwr off, glove:%d, status:%x\n", __func__,
					mode, ts->touch_functions);
		goto glove_enable_err;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Failed to send command", __func__);
		goto glove_enable_err;
	}

	input_info(true, &ts->client->dev, "%s: glove:%d, status:%x\n", __func__,
		mode, ts->touch_functions);

	return 0;

glove_enable_err:
	return -EIO;
}
EXPORT_SYMBOL(sec_ts_glove_mode_enables);

int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable)
{
	int ret;

	input_info(true, &ts->client->dev, "%s: %d\n", __func__, ts->cover_type);


	switch (ts->cover_type) {
	case SEC_TS_VIEW_WIRELESS:
	case SEC_TS_VIEW_COVER:
	case SEC_TS_VIEW_WALLET:
	case SEC_TS_FLIP_WALLET:
	case SEC_TS_LED_COVER:
	case SEC_TS_MONTBLANC_COVER:
	case SEC_TS_CLEAR_FLIP_COVER:
	case SEC_TS_QWERTY_KEYBOARD_EUR:
	case SEC_TS_QWERTY_KEYBOARD_KOR:
		ts->cover_cmd = (u8)ts->cover_type;
		break;
	case SEC_TS_CHARGER_COVER:
	case SEC_TS_COVER_NOTHING1:
	case SEC_TS_COVER_NOTHING2:
	default:
		ts->cover_cmd = 0;
		input_err(true, &ts->client->dev, "%s: not chage touch state, %d\n",
				__func__, ts->cover_type);
		break;
	}

	if (enable)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: pwr off, close:%d, status:%x\n", __func__,
					enable, ts->touch_functions);
		goto cover_enable_err;
	}

	if (enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: Failed to send covertype command: %d", __func__, ts->cover_cmd);
			goto cover_enable_err;
		}
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Failed to send command", __func__);
		goto cover_enable_err;
	}

	input_info(true, &ts->client->dev, "%s: close:%d, status:%x\n", __func__,
		enable, ts->touch_functions);

	return 0;

cover_enable_err:
	return -EIO;


}
EXPORT_SYMBOL(sec_ts_set_cover_type);

void sec_ts_set_grip_type(struct sec_ts_data *ts, u8 set_type)
{
	u8 mode = G_NONE;

	input_info(true, &ts->client->dev, "%s: re-init grip(%d), edh:%d, edg:%d, lan:%d\n", __func__,
		set_type, ts->grip_edgehandler_direction, ts->grip_edge_range, ts->grip_landscape_mode);

	/* edge handler */
	if (ts->grip_edgehandler_direction != 0)
		mode |= G_SET_EDGE_HANDLER;

	if (set_type == GRIP_ALL_DATA) {
		/* edge */
		if (ts->grip_edge_range != 60)
			mode |= G_SET_EDGE_ZONE;

		/* dead zone */
		if (ts->grip_landscape_mode == 1)	/* default 0 mode, 32 */
			mode |= G_SET_LANDSCAPE_MODE;
		else
			mode |= G_SET_NORMAL_MODE;
	}

	if (mode)
		set_grip_data_to_ic(ts, mode);

}

/* for debugging--------------------------------------------------------------------------------------*/

static int sec_ts_pinctrl_configure(struct sec_ts_data *ts, bool enable)
{
	struct pinctrl_state *state;

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, enable ? "ACTIVE" : "SUSPEND");

	if (enable) {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl, "on_state");
		if (IS_ERR(ts->plat_data->pinctrl))
			input_err(true, &ts->client->dev, "%s: could not get active pinstate\n", __func__);
	} else {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl, "off_state");
		if (IS_ERR(ts->plat_data->pinctrl))
			input_err(true, &ts->client->dev, "%s: could not get suspend pinstate\n", __func__);
	}

	if (!IS_ERR_OR_NULL(state))
		return pinctrl_select_state(ts->plat_data->pinctrl, state);

	return 0;

}

static int sec_ts_power(void *data, bool on)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)data;
	const struct sec_ts_plat_data *pdata = ts->plat_data;
	struct regulator *regulator_dvdd = NULL;
	struct regulator *regulator_avdd = NULL;
	static bool enabled;
	int ret = 0;

	if (enabled == on)
		return ret;

	regulator_dvdd = regulator_get(NULL, pdata->regulator_dvdd);
	if (IS_ERR_OR_NULL(regulator_dvdd)) {
		input_err(true, &ts->client->dev, "%s: Failed to get %s regulator.\n",
			 __func__, pdata->regulator_dvdd);
		ret = PTR_ERR(regulator_dvdd);
		goto error;
	}

	regulator_avdd = regulator_get(NULL, pdata->regulator_avdd);
	if (IS_ERR_OR_NULL(regulator_avdd)) {
		input_err(true, &ts->client->dev, "%s: Failed to get %s regulator.\n",
			 __func__, pdata->regulator_avdd);
		ret = PTR_ERR(regulator_avdd);
		goto error;
	}

	if (on) {
		ret = regulator_enable(regulator_dvdd);
		if (ret) {
			input_err(true, &ts->client->dev, "%s: Failed to enable avdd: %d\n", __func__, ret);
			goto out;
		}

		sec_ts_delay(1);

		ret = regulator_enable(regulator_avdd);
		if (ret) {
			input_err(true, &ts->client->dev, "%s: Failed to enable vdd: %d\n", __func__, ret);
			goto out;
		}
	} else {
		regulator_disable(regulator_dvdd);
		regulator_disable(regulator_avdd);
	}

	enabled = on;

out:
	input_err(true, &ts->client->dev, "%s: %s: avdd:%s, dvdd:%s\n", __func__, on ? "on" : "off",
		regulator_is_enabled(regulator_avdd) ? "on" : "off",
		regulator_is_enabled(regulator_dvdd) ? "on" : "off");

error:
	regulator_put(regulator_dvdd);
	regulator_put(regulator_avdd);

	return ret;
}

static int sec_ts_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_ts_plat_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	u32 coords[2];
	int ret = 0;
	int count = 0;
	u32 ic_match_value;
	int lcdtype = 0;
#if defined(CONFIG_EXYNOS_DECON_FB)
	int connected;
#endif

	pdata->tsp_icid = of_get_named_gpio(np, "sec,tsp-icid_gpio", 0);
	if (gpio_is_valid(pdata->tsp_icid)) {
		input_info(true, dev, "%s: TSP_ICID : %d\n", __func__, gpio_get_value(pdata->tsp_icid));
		if (of_property_read_u32(np, "sec,icid_match_value", &ic_match_value)) {
			input_err(true, dev, "%s: Failed to get icid match value\n", __func__);
			return -EINVAL;
		}

		if (gpio_get_value(pdata->tsp_icid) != ic_match_value) {
			input_err(true, dev, "%s: Do not match TSP_ICID\n", __func__);
			return -EINVAL;
		}
	} else {
		input_err(true, dev, "%s: Failed to get tsp-icid gpio\n", __func__);
	}

	pdata->tsp_vsync = of_get_named_gpio(np, "sec,tsp_vsync_gpio", 0);
	if (gpio_is_valid(pdata->tsp_vsync))
		input_info(true, &client->dev, "%s: vsync %s\n", __func__,
				gpio_get_value(pdata->tsp_vsync) ? "disable" : "enable");

	pdata->irq_gpio = of_get_named_gpio(np, "sec,irq_gpio", 0);
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "sec,tsp_int");
		if (ret) {
			input_err(true, &client->dev, "%s: Unable to request tsp_int [%d]\n", __func__, pdata->irq_gpio);
			return -EINVAL;
		}
	} else {
		input_err(true, &client->dev, "%s: Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}

	client->irq = gpio_to_irq(pdata->irq_gpio);

	if (of_property_read_u32(np, "sec,irq_type", &pdata->irq_type)) {
		input_err(true, dev, "%s: Failed to get irq_type property\n", __func__);
		pdata->irq_type = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	}

	if (of_property_read_u32(np, "sec,i2c-burstmax", &pdata->i2c_burstmax)) {
		input_dbg(false, &client->dev, "%s: Failed to get i2c_burstmax property\n", __func__);
		pdata->i2c_burstmax = 256;
	}
	if (pdata->i2c_burstmax > I2C_PREALLOC_READ_BUF_SZ ||
	    pdata->i2c_burstmax > I2C_PREALLOC_WRITE_BUF_SZ) {
		input_err(true, &client->dev,
			  "%s: i2c_burstmax is larger than i2c_read_buf and/or i2c_write_buf.\n",
			  __func__);
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "sec,max_coords", coords, 2)) {
		input_err(true, &client->dev, "%s: Failed to get max_coords property\n", __func__);
		return -EINVAL;
	}
	pdata->max_x = coords[0] - 1;
	pdata->max_y = coords[1] - 1;

#ifdef PAT_CONTROL
	if (of_property_read_u32(np, "sec,pat_function", &pdata->pat_function) < 0) {
		pdata->pat_function = 0;
		input_err(true, dev, "%s: Failed to get pat_function property\n", __func__);
	}

	if (of_property_read_u32(np, "sec,afe_base", &pdata->afe_base) < 0) {
		pdata->afe_base = 0;
		input_err(true, dev, "%s: Failed to get afe_base property\n", __func__);
	}
#endif

	pdata->tsp_id = of_get_named_gpio(np, "sec,tsp-id_gpio", 0);
	if (gpio_is_valid(pdata->tsp_id))
		input_info(true, dev, "%s: TSP_ID : %d\n", __func__, gpio_get_value(pdata->tsp_id));
	else
		input_err(true, dev, "%s: Failed to get tsp-id gpio\n", __func__);

	pdata->switch_gpio = of_get_named_gpio(np,
					       "sec,switch_gpio", 0);
	if (gpio_is_valid(pdata->switch_gpio)) {
		ret = gpio_request_one(pdata->switch_gpio,
				       GPIOF_OUT_INIT_HIGH,
				       "sec,touch_i2c_switch");
		if (ret) {
			input_err(true, dev,
				  "%s: Failed to request gpio %d\n",
				  __func__, pdata->switch_gpio);
			return -EINVAL;
		}

		ret = gpio_direction_output(pdata->switch_gpio, 0);
		if (ret) {
			input_err(true, dev,
				  "%s: Failed to set gpio %d direction\n",
				  __func__, pdata->switch_gpio);
			return -EINVAL;
		}
	} else {
		input_err(true, dev, "%s: Failed to get switch_gpio\n",
			  __func__);
	}

	count = of_property_count_strings(np, "sec,firmware_name");
	if (count <= 0) {
		pdata->firmware_name = NULL;
	} else {
		if (gpio_is_valid(pdata->tsp_id))
			of_property_read_string_index(np, "sec,firmware_name", gpio_get_value(pdata->tsp_id), &pdata->firmware_name);
		else
			of_property_read_string_index(np, "sec,firmware_name", 0, &pdata->firmware_name);
	}

	if (of_property_read_string_index(np, "sec,project_name", 0, &pdata->project_name))
		input_err(true, &client->dev, "%s: skipped to get project_name property\n", __func__);
	if (of_property_read_string_index(np, "sec,project_name", 1, &pdata->model_name))
		input_err(true, &client->dev, "%s: skipped to get model_name property\n", __func__);

#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
	lcdtype = get_lcd_attached("GET");
	if (lcdtype < 0) {
		input_err(true, &client->dev, "%s: lcd is not attached\n", __func__);
		return -ENODEV;
	}
#endif

#if defined(CONFIG_EXYNOS_DECON_FB)
	connected = get_lcd_info("connected");
	if (connected < 0) {
		input_err(true, dev, "%s: Failed to get lcd info\n", __func__);
		return -EINVAL;
	}

	if (!connected) {
		input_err(true, &client->dev, "%s: lcd is disconnected\n", __func__);
		return -ENODEV;
	}

	input_info(true, &client->dev, "%s: lcd is connected\n", __func__);

	lcdtype = get_lcd_info("id");
	if (lcdtype < 0) {
		input_err(true, dev, "%s: Failed to get lcd info\n", __func__);
		return -EINVAL;
	}
#endif

	input_info(true, &client->dev, "%s: lcdtype 0x%08X\n", __func__, lcdtype);

	if (strncmp(pdata->model_name, "G950", 4) == 0)
		pdata->panel_revision = 0;
	else
		pdata->panel_revision = ((lcdtype >> 8) & 0xFF) >> 4;

	if (of_property_read_string(np, "sec,regulator_dvdd", &pdata->regulator_dvdd)) {
		input_err(true, dev, "%s: Failed to get regulator_dvdd name property\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_string(np, "sec,regulator_avdd", &pdata->regulator_avdd)) {
		input_err(true, dev, "%s: Failed to get regulator_avdd name property\n", __func__);
		return -EINVAL;
	}

	pdata->power = sec_ts_power;

	if (of_property_read_u32(np, "sec,always_lpmode", &pdata->always_lpmode) < 0)
		pdata->always_lpmode = 0;

	if (of_property_read_u32(np, "sec,bringup", &pdata->bringup) < 0)
		pdata->bringup = 0;

	if (of_property_read_u32(np, "sec,mis_cal_check", &pdata->mis_cal_check) < 0)
		pdata->mis_cal_check = 0;

	pdata->regulator_boot_on = of_property_read_bool(np, "sec,regulator_boot_on");
	pdata->support_sidegesture = of_property_read_bool(np, "sec,support_sidegesture");
	pdata->support_dex = of_property_read_bool(np, "support_dex_mode");

	pdata->support_mt_pressure = true;

#ifdef PAT_CONTROL
	input_err(true, &client->dev, "%s: i2c buffer limit: %d, lcd_id:%06X, bringup:%d, FW:%s(%d), id:%d,%d, pat_function:%d mis_cal:%d dex:%d, gesture:%d\n",
			__func__, pdata->i2c_burstmax, lcdtype, pdata->bringup, pdata->firmware_name,
			count, pdata->tsp_id, pdata->tsp_icid, pdata->pat_function,
			pdata->mis_cal_check, pdata->support_dex, pdata->support_sidegesture);
#else
	input_err(true, &client->dev, "%s: i2c buffer limit: %d, lcd_id:%06X, bringup:%d, FW:%s(%d), id:%d,%d, dex:%d, gesture:%d\n",
		__func__, pdata->i2c_burstmax, lcdtype, pdata->bringup, pdata->firmware_name,
		count, pdata->tsp_id, pdata->tsp_icid, pdata->support_dex, pdata->support_sidegesture);
#endif
	return ret;
}

int sec_ts_read_information(struct sec_ts_data *ts)
{
	unsigned char data[13] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_READ_INFO, true);

	memset(data, 0x0, 3);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ID, data, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read device id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: %X, %X, %X\n",
				__func__, data[0], data[1], data[2]);
	memset(data, 0x0, 11);
	ret = sec_ts_i2c_read(ts,  SEC_TS_READ_PANEL_INFO, data, 11);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: nTX:%X, nRX:%X, rY:%d, rX:%d\n",
				__func__, data[8], data[9],
				(data[2] << 8) | data[3], (data[0] << 8) | data[1]);

	/* Set X,Y Resolution from IC information. */
	if (((data[0] << 8) | data[1]) > 0)
		ts->plat_data->max_x = ((data[0] << 8) | data[1]) - 1;

	if (((data[2] << 8) | data[3]) > 0)
		ts->plat_data->max_y = ((data[2] << 8) | data[3]) - 1;

	ts->tx_count = data[8];
	ts->rx_count = data[9];

	data[0] = 0;
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: STATUS : %X\n",
				__func__, data[0]);

	memset(data, 0x0, 4);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, data, 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: TOUCH STATUS : %02X, %02X, %02X, %02X\n",
				__func__, data[0], data[1], data[2], data[3]);
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,  (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read touch functions(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: Functions : %02X\n",
				__func__, ts->touch_functions);

out:
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_READ_INFO, false);
	return ret;
}

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
int sec_ts_set_custom_library(struct sec_ts_data *ts)
{
	u8 data[3] = { 0 };
	int ret;

	input_err(true, &ts->client->dev, "%s: Custom Library (0x%02x)\n",
				__func__, ts->lowpower_mode);

	data[2] = ts->lowpower_mode;

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, &data[0], 3);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: Failed to Custom Library\n", __func__);

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: Failed to send NOTIFY Custom Library\n", __func__);

	return ret;
}

int sec_ts_check_custom_library(struct sec_ts_data *ts)
{
	u8 data[10] = { 0 };
	int ret = -1;

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_CUSTOMLIB_GET_INFO, &data[0], 10);

	input_info(true, &ts->client->dev,
				"%s: (%d) %c%c%c%c, || %02X, %02X, %02X, %02X, || %02X, %02X\n",
				__func__, ret, data[0], data[1], data[2], data[3], data[4],
				data[5], data[6], data[7], data[8], data[9]);

	/* compare model name with device tree */
	if (ts->plat_data->model_name)
		ret = strncmp(data, ts->plat_data->model_name, 4);

	if (ret == 0)
		ts->use_customlib= true;
	else
		ts->use_customlib= false;

	input_err(true, &ts->client->dev, "%s: use %s\n",
				__func__, ts->use_customlib? "CUSTOMLIB" : "VENDOR");

	return ret;
}
#endif

static void sec_ts_set_input_prop(struct sec_ts_data *ts, struct input_dev *dev, u8 propbit)
{
	static char sec_ts_phys[64] = { 0 };

	snprintf(sec_ts_phys, sizeof(sec_ts_phys), "%s/input1",
			dev->name);
	dev->phys = sec_ts_phys;
	dev->id.bustype = BUS_I2C;
	dev->dev.parent = &ts->client->dev;

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SW, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_TOOL_FINGER, dev->keybit);
	set_bit(KEY_BLACK_UI_GESTURE, dev->keybit);
#ifdef SEC_TS_SUPPORT_TOUCH_KEY
	if (ts->plat_data->support_mskey) {
		int i;

		for (i = 0 ; i < ts->plat_data->num_touchkey ; i++)
			set_bit(ts->plat_data->touchkey[i].keycode, dev->keybit);

		set_bit(EV_LED, dev->evbit);
		set_bit(LED_MISC, dev->ledbit);
	}
#endif
	if (ts->plat_data->support_sidegesture) {
		set_bit(KEY_SIDE_GESTURE, dev->keybit);
		set_bit(KEY_SIDE_GESTURE_RIGHT, dev->keybit);
		set_bit(KEY_SIDE_GESTURE_LEFT, dev->keybit);
	}
	set_bit(propbit, dev->propbit);
	set_bit(KEY_HOMEPAGE, dev->keybit);

	input_set_capability(dev, EV_SW, SW_GLOVE);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->plat_data->max_x, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->plat_data->max_y, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_CUSTOM, 0, 0xFFFF, 0, 0);
	if (ts->plat_data->support_mt_pressure)
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0,
				     SEC_TS_PRESSURE_MAX, 0, 0);

	if (propbit == INPUT_PROP_POINTER)
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_POINTER);
	else
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_DIRECT);

	input_set_drvdata(dev, ts);
}

static int sec_ts_fw_init(struct sec_ts_data *ts)
{
	int ret = SEC_TS_ERR_NA;
	bool force_update = false;
	bool valid_firmware_integrity = false;
	unsigned char data[5] = { 0 };
	unsigned char deviceID[5] = { 0 };
	unsigned char result = 0;

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, deviceID, 5);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: failed to read device ID(%d)\n",
			  __func__, ret);
	else
		input_info(true, &ts->client->dev,
			"%s: TOUCH DEVICE ID : %02X, %02X, %02X, %02X, %02X\n",
			__func__, deviceID[0], deviceID[1], deviceID[2],
			deviceID[3], deviceID[4]);

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_FIRMWARE_INTEGRITY, &result, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to integrity check (%d)\n",
			  __func__, ret);
	} else {
		if (result & 0x80)
			valid_firmware_integrity = true;
		else
			input_err(true, &ts->client->dev, "%s: invalid integrity result (0x%x)\n",
				  __func__, result);
	}

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &data[0], 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: failed to read sub id(%d)\n", __func__, ret);
	} else {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, &data[1], 4);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to touch status(%d)\n",
				  __func__, ret);
	}
	input_info(true, &ts->client->dev,
		"%s: TOUCH STATUS : %02X || %02X, %02X, %02X, %02X\n",
		__func__, data[0], data[1], data[2], data[3], data[4]);

	if (data[0] == SEC_TS_STATUS_BOOT_MODE)
		ts->checksum_result = 1;

	if (((data[0] == SEC_TS_STATUS_APP_MODE &&
	      data[2] == TOUCH_SYSTEM_MODE_FLASH) || ret < 0) &&
	    (valid_firmware_integrity == false))
		force_update = true;

	ret = sec_ts_read_information(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to read information 0x%x\n",
			  __func__, ret);
		return SEC_TS_ERR_INIT;
	}

	ts->touch_functions |= SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			       (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: Failed to send touch func_mode command",
			  __func__);

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to write Sense_on 0x%x\n",
			  __func__, ret);
		return SEC_TS_ERR_INIT;
	}

	ts->pFrame = kzalloc(ts->tx_count * ts->rx_count * 2, GFP_KERNEL);
	if (!ts->pFrame)
		return SEC_TS_ERR_ALLOC_FRAME;

	ts->gainTable = kzalloc(ts->tx_count * ts->rx_count, GFP_KERNEL);
	if (!ts->gainTable) {
		kfree(ts->pFrame);
		ts->pFrame = NULL;
		return SEC_TS_ERR_ALLOC_GAINTABLE;
	}

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad->name = "sec_touchpad";
		sec_ts_set_input_prop(ts, ts->input_dev_pad,
				      INPUT_PROP_POINTER);
	}
	ts->dex_name = "";

	ts->input_dev->name = "sec_touchscreen";
	sec_ts_set_input_prop(ts, ts->input_dev, INPUT_PROP_DIRECT);
#ifdef USE_OPEN_CLOSE
	ts->input_dev->open = sec_ts_input_open;
	ts->input_dev->close = sec_ts_input_close;
#endif
	ts->input_dev_touch = ts->input_dev;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		input_err(true, &ts->client->dev, "%s: Unable to register %s input device 0x%x\n",
			  __func__, ts->input_dev->name, ret);
		return SEC_TS_ERR_REG_INPUT_DEV;
	}

	if (ts->plat_data->support_dex) {
		ret = input_register_device(ts->input_dev_pad);
		if (ret) {
			input_err(true, &ts->client->dev, "%s: Unable to register %s input device 0x%x\n",
				  __func__, ts->input_dev_pad->name, ret);
			return SEC_TS_ERR_REG_INPUT_PAD_DEV;
		}
	}

	return SEC_TS_ERR_NA;
}

static void sec_ts_device_init(struct sec_ts_data *ts)
{
#if (1) //!defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	sec_ts_raw_device_init(ts);
#endif
	sec_ts_fn_init(ts);

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	sec_ts_check_custom_library(ts);
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
#endif
}

static struct notifier_block sec_ts_screen_nb;

static int sec_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sec_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	int ret = 0;

	input_info(true, &client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev, "%s: EIO err!\n", __func__);
		return -EIO;
	}

	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			input_err(true, &client->dev, "%s: Failed to allocate platform data\n", __func__);
			goto error_allocate_pdata;
		}

		client->dev.platform_data = pdata;

		ret = sec_ts_parse_dt(client);
		if (ret) {
			input_err(true, &client->dev, "%s: Failed to parse dt\n", __func__);
			goto error_allocate_mem;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			input_err(true, &client->dev, "%s: No platform data found\n", __func__);
			goto error_allocate_pdata;
		}
	}

	if (!pdata->power) {
		input_err(true, &client->dev, "%s: No power contorl found\n", __func__);
		goto error_allocate_mem;
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pdata->pinctrl))
		input_err(true, &client->dev, "%s: could not get pinctrl\n", __func__);

	ts = kzalloc(sizeof(struct sec_ts_data), GFP_KERNEL);
	if (!ts)
		goto error_allocate_mem;

	ts->client = client;
	ts->plat_data = pdata;
	ts->crc_addr = 0x0001FE00;
	ts->fw_addr = 0x00002000;
	ts->para_addr = 0x18000;
	ts->flash_page_size = SEC_TS_FW_BLK_SIZE_DEFAULT;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;
	ts->sec_ts_i2c_read_heap = sec_ts_i2c_read_heap;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_write_burst = sec_ts_i2c_write_burst;
	ts->sec_ts_i2c_write_burst_heap = sec_ts_i2c_write_burst_heap;
	ts->sec_ts_i2c_read_bulk = sec_ts_i2c_read_bulk;
	ts->sec_ts_i2c_read_bulk_heap = sec_ts_i2c_read_bulk_heap;
	ts->i2c_burstmax = pdata->i2c_burstmax;
#ifdef USE_POWER_RESET_WORK
	INIT_DELAYED_WORK(&ts->reset_work, sec_ts_reset_work);
#endif
	INIT_DELAYED_WORK(&ts->work_read_info, sec_ts_read_info_work);
	INIT_WORK(&ts->suspend_work, sec_ts_suspend_work);
	INIT_WORK(&ts->resume_work, sec_ts_resume_work);

	init_completion(&ts->bus_resumed);
	complete_all(&ts->bus_resumed);

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	INIT_DELAYED_WORK(&ts->work_fw_update, sec_ts_fw_update_work);
#else
	input_info(true, &ts->client->dev, "%s: fw update on probe disabled!\n",
		   __func__);
#endif

	ts->is_fw_corrupted = false;

	/* Assume screen is on throughout probe */
	ts->bus_refmask = SEC_TS_BUS_REF_SCREEN_ON;

	i2c_set_clientdata(client, ts);

#ifdef CONFIG_TOUCHSCREEN_TBN
	ts->tbn = tbn_init(&ts->client->dev);
	if (!ts->tbn) {
		input_err(true, &ts->client->dev,
			  "%s: TBN initialization error\n", __func__);
		ret = -ENODEV;
		goto err_init_tbn;
	}
#endif

	if (gpio_is_valid(ts->plat_data->tsp_id))
		ts->tspid_val = gpio_get_value(ts->plat_data->tsp_id);

	if (gpio_is_valid(ts->plat_data->tsp_icid))
		ts->tspicid_val = gpio_get_value(ts->plat_data->tsp_icid);

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		input_err(true, &ts->client->dev, "%s: allocate device err!\n", __func__);
		ret = -ENOMEM;
		goto err_allocate_input_dev;
	}

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad = input_allocate_device();
		if (!ts->input_dev_pad) {
			input_err(true, &ts->client->dev, "%s: allocate device err!\n", __func__);
			ret = -ENOMEM;
			goto err_allocate_input_dev_pad;
		}
	}

	ts->touch_count = 0;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;
	ts->sec_ts_i2c_read_heap = sec_ts_i2c_read_heap;
	ts->sec_ts_read_customlib = sec_ts_read_from_customlib;

	ts->max_z_value = 0;
	ts->min_z_value = 0xFFFFFFFF;
	ts->sum_z_value = 0;

	mutex_init(&ts->bus_mutex);
	mutex_init(&ts->lock);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->eventlock);

	init_completion(&ts->resume_done);
	complete_all(&ts->resume_done);

	if (pdata->always_lpmode)
		ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

	input_info(true, &client->dev, "%s: init resource\n", __func__);


	/* power enable */
	sec_ts_power(ts, true);
	if (!pdata->regulator_boot_on)
		sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	ts->external_factory = false;

	/*
	 * Ensure a clean start by pulsing reset.
	 * reset off -> reset on -> reset off -> I2C
	 * The first pinctrl_configure ensures reset starts off.
	 */
	sec_ts_pinctrl_configure(ts, true);
	/* wait 10ms, then reset on. */
	sec_ts_delay(10);
	sec_ts_pinctrl_configure(ts, false);
	/* wait 10ms then reset off. */
	sec_ts_delay(10);
	sec_ts_pinctrl_configure(ts, true);
	/* wait 10ms, then I2C. */
	sec_ts_delay(10);
	ret = sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	if (ret < 0) {
		u8 boot_status;
		/* Read the boot status in case device is in bootloader mode */
		ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS,
					  &boot_status, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: could not read boot status. Assuming no device connected.\n",
				  __func__);
			goto err_init;
		}

		input_info(true, &ts->client->dev,
			   "%s: Attempting to reflash the firmware. Boot status = 0x%02X\n",
			   __func__, boot_status);
		if (boot_status != SEC_TS_STATUS_BOOT_MODE)
			input_err(true, &ts->client->dev,
				  "%s: device is not in bootloader mode!\n",
				  __func__);

		ts->is_fw_corrupted = true;
	}

	input_info(true, &client->dev, "%s: power enable\n", __func__);

	if (ts->is_fw_corrupted == false) {
		switch (sec_ts_fw_init(ts)) {
		case SEC_TS_ERR_INIT:
			goto err_init;
		case SEC_TS_ERR_ALLOC_FRAME:
			goto err_allocate_frame;
		case SEC_TS_ERR_ALLOC_GAINTABLE:
			goto err_allocate_gaintable;
		case SEC_TS_ERR_REG_INPUT_DEV:
			goto err_input_register_device;
		case SEC_TS_ERR_REG_INPUT_PAD_DEV:
			goto err_input_pad_register_device;
		}
	}

	input_info(true, &ts->client->dev, "%s: request_irq = %d\n", __func__, client->irq);

	ret = request_threaded_irq(client->irq, NULL, sec_ts_irq_thread,
			ts->plat_data->irq_type, SEC_TS_I2C_NAME, ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Unable to request threaded irq\n", __func__);
		goto err_irq;
	}

	ts->notifier = sec_ts_screen_nb;
	ret = msm_drm_register_client(&ts->notifier);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: msm_drm_register_client failed. ret = 0x%08X\n",
			  __func__, ret);
		goto err_register_drm_client;
	}

#ifndef CONFIG_SEC_SYSFS
	sec_class = class_create(THIS_MODULE, "sec");
#endif

	device_init_wakeup(&client->dev, true);

	if (ts->is_fw_corrupted == false) {
		sec_ts_device_init(ts);
		schedule_delayed_work(&ts->work_read_info,
				      msecs_to_jiffies(5000));
	}

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	schedule_delayed_work(&ts->work_fw_update, msecs_to_jiffies(10000));
#endif

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
	dump_callbacks.inform_dump = dump_tsp_log;
	INIT_DELAYED_WORK(&ts->ghost_check, sec_ts_check_rawdata);
	p_ghost_check = &ts->ghost_check;
#endif

	ts_dup = ts;
	ts->probe_done = true;

	input_err(true, &ts->client->dev, "%s: done\n", __func__);
	input_log_fix();

	return 0;

	/* need to be enabled when new goto statement is added */
/*
	sec_ts_fn_remove(ts);
	free_irq(client->irq, ts);
*/
err_register_drm_client:
	free_irq(client->irq, ts);
err_irq:
	if (ts->plat_data->support_dex) {
		input_unregister_device(ts->input_dev_pad);
		ts->input_dev_pad = NULL;
	}
err_input_pad_register_device:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
	ts->input_dev_touch = NULL;
err_input_register_device:
	kfree(ts->gainTable);
err_allocate_gaintable:
	kfree(ts->pFrame);
err_allocate_frame:
err_init:
	sec_ts_power(ts, false);
	if (ts->plat_data->support_dex) {
		if (ts->input_dev_pad)
			input_free_device(ts->input_dev_pad);
	}
err_allocate_input_dev_pad:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
err_allocate_input_dev:
#ifdef CONFIG_TOUCHSCREEN_TBN
	tbn_cleanup(ts->tbn);
err_init_tbn:
#endif
	kfree(ts);

error_allocate_mem:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (gpio_is_valid(pdata->tsp_id))
		gpio_free(pdata->tsp_id);
	if (gpio_is_valid(pdata->tsp_icid))
		gpio_free(pdata->tsp_icid);

error_allocate_pdata:
	if (ret == -ECONNREFUSED)
		sec_ts_delay(100);
	ret = -ENODEV;
#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	ts_dup = NULL;
	input_err(true, &client->dev, "%s: failed(%d)\n", __func__, ret);
	input_log_fix();
	return ret;
}

void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			input_info(true, &ts->client->dev,
					"%s: [RA] tID:%d mc:%d tc:%d v:%02X%02X cal:%02X(%02X) id(%d,%d) p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->nv, ts->tspid_val,
					ts->tspicid_val, ts->coord[i].palm_count);

			do_gettimeofday(&ts->time_released[i]);
			
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	input_mt_slot(ts->input_dev, 0);

	input_report_key(ts->input_dev, BTN_TOUCH, false);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, false);
	input_report_switch(ts->input_dev, SW_GLOVE, false);
	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;

	if (ts->plat_data->support_sidegesture) {
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_LEFT, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_RIGHT, 0);
	}

	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);

}

void sec_ts_locked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	mutex_lock(&ts->eventlock);

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			input_info(true, &ts->client->dev,
					"%s: [RA] tID:%d mc: %d tc:%d, v:%02X%02X, cal:%X(%X|%X), id(%d,%d), p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->nv, ts->cal_count, ts->tspid_val,
					ts->tspicid_val, ts->coord[i].palm_count);

			do_gettimeofday(&ts->time_released[i]);
			
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	input_mt_slot(ts->input_dev, 0);

	input_report_key(ts->input_dev, BTN_TOUCH, false);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, false);
	input_report_switch(ts->input_dev, SW_GLOVE, false);
	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;

	if (ts->plat_data->support_sidegesture) {
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_LEFT, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_RIGHT, 0);
	}

	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);

	mutex_unlock(&ts->eventlock);

}

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
							reset_work.work);

	ts->reset_is_on_going = true;
	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_RESET, true);

	sec_ts_stop_device(ts);

	sec_ts_delay(30);

	sec_ts_start_device(ts);

	if (ts->input_dev_touch->disabled) {
		input_err(true, &ts->client->dev , "%s: call input_close\n", __func__);

		sec_ts_input_close(ts->input_dev);

		if ((ts->lowpower_mode & SEC_TS_MODE_CUSTOMLIB_AOD) && ts->use_customlib) {
			int i, ret;
			u8 data[10] = {0x02, 0};

			for (i = 0; i < 4; i++) {
				data[i * 2 + 2] = ts->rect_data[i] & 0xFF;
				data[i * 2 + 3] = (ts->rect_data[i] >> 8) & 0xFF;
			}

			disable_irq(ts->client->irq);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, &data[0], 10);
			if (ret < 0)
				input_err(true, &ts->client->dev, "%s: Failed to write offset\n", __func__);

			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
			if (ret < 0)
				input_err(true, &ts->client->dev, "%s: Failed to send notify\n", __func__);
			enable_irq(ts->client->irq);
		}
	}
	ts->reset_is_on_going = false;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_RESET, false);
}
#endif

static void sec_ts_read_info_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
							work_read_info.work);
#ifndef CONFIG_SEC_FACTORY
	struct sec_ts_test_mode mode;
	char para = TO_TOUCH_MODE;
#endif
#ifdef USE_PRESSURE_SENSOR
	int ret;
	unsigned char data[18] = { 0 };
#endif

	ts->nv = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	ts->cal_count = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);
	ts->pressure_cal_base = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_PRESSURE_BASE_CAL_COUNT);
	ts->pressure_cal_delta = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_PRESSURE_DELTA_CAL_COUNT);

	input_info(true, &ts->client->dev, "%s: fac_nv:%02X, cal_count:%02X\n", __func__, ts->nv, ts->cal_count);

#ifdef PAT_CONTROL
	ts->tune_fix_ver = (get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_TUNE_VERSION) << 8) | get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_TUNE_VERSION+1);
	input_info(true, &ts->client->dev, "%s: tune_fix_ver [%04X]\n", __func__, ts->tune_fix_ver);
#endif

#ifdef USE_PRESSURE_SENSOR
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_SET_GET_PRESSURE, data, 18);
	if (ret < 0)
		return;

	ts->pressure_left = ((data[16] << 8) | data[17]);
	ts->pressure_center = ((data[8] << 8) | data[9]);
	ts->pressure_right = ((data[0] << 8) | data[1]);
	input_info(true, &ts->client->dev, "%s: left: %d, center: %d, right: %d\n",
		__func__, ts->pressure_left, ts->pressure_center, ts->pressure_right);
#endif

#ifndef CONFIG_SEC_FACTORY
	/* run self-test */
	disable_irq(ts->client->irq);
	execute_selftest(ts, false);
	enable_irq(ts->client->irq);

	input_info(true, &ts->client->dev, "%s: %02X %02X %02X %02X\n",
		__func__, ts->ito_test[0], ts->ito_test[1]
		, ts->ito_test[2], ts->ito_test[3]);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		 input_err(true, &ts->client->dev, "%s: Failed to set\n", __func__);

	sec_ts_delay(350);

	/* run ambient read */
	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_AMBIENT_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, NULL, &mode);
#endif

	input_log_fix();
}

static void sec_ts_fw_update_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      work_fw_update.work);
	int ret;

	input_info(true, &ts->client->dev,
		   "%s: Beginning firmware update after probe.\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, true);

	ret = sec_ts_firmware_update_on_probe(ts, false);
	if (ret < 0)
		input_info(true, &ts->client->dev,
			   "%s: firmware update was unsuccessful.\n",
			   __func__);

	if (ts->is_fw_corrupted == true && ret == 0) {
		ret = sec_ts_fw_init(ts);
		if (ret == SEC_TS_ERR_NA) {
			ts->is_fw_corrupted = false;
			sec_ts_device_init(ts);
			sec_ts_read_info_work(&ts->work_read_info.work);
		} else
			input_info(true, &ts->client->dev,
				"%s: fail to sec_ts_fw_init 0x%x\n",
				__func__, ret);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, false);
}

int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode)
{
	int ret;
	int retrycnt = 0;
	u8 data;
	char para = 0;

	input_err(true, &ts->client->dev, "%s: %s(%X)\n", __func__,
			mode == TO_LOWPOWER_MODE ? "ENTER" : "EXIT", ts->lowpower_mode);

	if (mode) {
		#ifdef SEC_TS_SUPPORT_CUSTOMLIB
		if (ts->use_customlib)
			sec_ts_set_custom_library(ts);
		#endif

		data = (ts->lowpower_mode & SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE, &data, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to set\n", __func__);
	}

retry_pmode:
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
				"%s: failed\n", __func__);
	sec_ts_delay(50);

	/* read data */

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: read power mode failed!\n", __func__);
	else
		input_info(true, &ts->client->dev, "%s: power mode - write(%d) read(%d)\n", __func__, mode, para);

	if (mode != para) {
		retrycnt++;
		if (retrycnt < 5)
			goto retry_pmode;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: i2c write clear event failed\n", __func__);


	sec_ts_locked_release_all_finger(ts);

	if (device_may_wakeup(&ts->client->dev)) {
		if (mode)
			enable_irq_wake(ts->client->irq);
		else
			disable_irq_wake(ts->client->irq);
	}

	ts->lowpower_status = mode;
	input_info(true, &ts->client->dev, "%s: end\n", __func__);

	return ret;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);
	int ret;

	ts->input_closed = false;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, true);

	if (ts->lowpower_status) {
#ifdef USE_RESET_EXIT_LPM
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#else
		sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
#endif
		ts->power_status = SEC_TS_STATE_POWER_ON;
	} else {
		ret = sec_ts_start_device(ts);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to start device\n", __func__);
	}

	/* because edge and dead zone will recover soon */
	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, false);

	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);

	ts->input_closed = true;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, true);

	cancel_work(&ts->suspend_work);
	cancel_work(&ts->resume_work);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work(&ts->reset_work);
#endif

#ifndef CONFIG_SEC_FACTORY
	ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
#endif
	if (ts->lowpower_mode) {
		sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
		ts->power_status = SEC_TS_STATE_LPM;
	} else {
		sec_ts_stop_device(ts);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, false);
}
#endif

static int sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	input_info(true, &ts->client->dev, "%s\n", __func__);

	/* Force the bus active throughout removal of the client */
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FORCE_ACTIVE, true);

	msm_drm_unregister_client(&ts->notifier);

	cancel_work(&ts->suspend_work);
	flush_work(&ts->suspend_work);
	cancel_work(&ts->resume_work);
	flush_work(&ts->resume_work);

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	cancel_delayed_work_sync(&ts->work_fw_update);
	flush_delayed_work(&ts->work_fw_update);
#endif

	cancel_delayed_work_sync(&ts->work_read_info);
	flush_delayed_work(&ts->work_read_info);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts);
	input_info(true, &ts->client->dev, "%s: irq disabled\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);

	input_info(true, &ts->client->dev, "%s: flush queue\n", __func__);

#endif

	sec_ts_fn_remove(ts);

#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	device_init_wakeup(&client->dev, false);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev = ts->input_dev_touch;
	input_mt_destroy_slots(ts->input_dev);
	input_unregister_device(ts->input_dev);

	ts->input_dev_pad = NULL;
	ts->input_dev = NULL;
	ts->input_dev_touch = NULL;
	ts_dup = NULL;
	ts->plat_data->power(ts, false);

#ifdef CONFIG_TOUCHSCREEN_TBN
	tbn_cleanup(ts->tbn);
#endif

	kfree(ts);
	return 0;
}

static void sec_ts_shutdown(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_remove(client);
}

int sec_ts_stop_device(struct sec_ts_data *ts)
{
	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: already power off\n", __func__);
		goto out;
	}

	ts->power_status = SEC_TS_STATE_POWER_OFF;

	disable_irq(ts->client->irq);
	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts, false);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	sec_ts_pinctrl_configure(ts, false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int sec_ts_start_device(struct sec_ts_data *ts)
{
	int ret;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_pinctrl_configure(ts, true);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		input_err(true, &ts->client->dev, "%s: already power on\n", __func__);
		goto out;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts, true);
	sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

	if (ts->flip_enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);

		ts->touch_functions = ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER;
		input_info(true, &ts->client->dev,
				"%s: cover cmd write type:%d, mode:%x, ret:%d", __func__, ts->touch_functions, ts->cover_cmd, ret);
	} else {
		ts->touch_functions = (ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER));
		input_info(true, &ts->client->dev,
			"%s: cover open, not send cmd", __func__);
	}

	ts->touch_functions = ts->touch_functions | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: Failed to send touch function command", __func__);

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	if (ts->dex_mode) {
		input_info(true, &ts->client->dev, "%s: set dex mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DEX_MODE, &ts->dex_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: failed to set dex mode %x\n", __func__, ts->dex_mode);
	}

	if (ts->brush_mode) {
		input_info(true, &ts->client->dev, "%s: set brush mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_BRUSH_MODE, &ts->brush_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
						"%s: failed to set brush mode\n", __func__);
	}

	if (ts->touchable_area) {
		input_info(true, &ts->client->dev, "%s: set 16:9 mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA, &ts->touchable_area, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
						"%s: failed to set 16:9 mode\n", __func__);
	}

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: fail to write Sense_on\n", __func__);

	enable_irq(ts->client->irq);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

#ifdef CONFIG_PM
static int sec_ts_pm_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_ts_pm_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	return 0;
}
#endif

static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_I2C_NAME, 0 },
	{ },
};

#ifdef CONFIG_PM
static const struct dev_pm_ops sec_ts_dev_pm_ops = {
	.suspend = sec_ts_pm_suspend,
	.resume = sec_ts_pm_resume,
};
#endif

/*
 * Configure the switch GPIO to toggle bus master between AP and SLPI.
 * gpio_value takes one of
 * { SEC_SWITCH_GPIO_VALUE_SLPI_MASTER, SEC_SWITCH_GPIO_VALUE_AP_MASTER }
 */
static void sec_set_switch_gpio(struct sec_ts_data *ts, int gpio_value)
{
	int retval;
	unsigned int gpio = ts->plat_data->switch_gpio;

	if (!gpio_is_valid(gpio))
		return;

	input_info(true, &ts->client->dev, "%s: toggling i2c switch to %s\n",
		   __func__, gpio_value == SEC_SWITCH_GPIO_VALUE_AP_MASTER ?
		   "AP" : "SLPI");

	retval = gpio_direction_output(gpio, gpio_value);
	if (retval < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to toggle switch_gpio, err = %d\n",
			  __func__, retval);
}

static void sec_ts_suspend_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      suspend_work);
	int ret = 0;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->device_mutex);

	reinit_completion(&ts->bus_resumed);

	if (ts->power_status == SEC_TS_STATE_SUSPEND) {
		input_err(true, &ts->client->dev, "%s: already suspended.\n",
			  __func__);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	/* Sense_off */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: failed to write Sense_off.\n", __func__);

	disable_irq_nosync(ts->client->irq);
	sec_ts_locked_release_all_finger(ts);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	ts->power_status = SEC_TS_STATE_SUSPEND;

	sec_set_switch_gpio(ts, SEC_SWITCH_GPIO_VALUE_SLPI_MASTER);

#ifdef CONFIG_TOUCHSCREEN_TBN
	if (ts->tbn)
		tbn_release_bus(ts->tbn);
#endif

	mutex_unlock(&ts->device_mutex);
}

static void sec_ts_resume_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      resume_work);
	int ret = 0;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->device_mutex);

#ifdef CONFIG_TOUCHSCREEN_TBN
	if (ts->tbn)
		tbn_request_bus(ts->tbn);
#endif

	sec_set_switch_gpio(ts, SEC_SWITCH_GPIO_VALUE_AP_MASTER);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		input_err(true, &ts->client->dev, "%s: already resumed.\n",
			  __func__);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->power_status = SEC_TS_STATE_POWER_ON;

	ret = sec_ts_sw_reset(ts);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: software reset failed!\n", __func__);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

	ts->touch_functions =
	    ts->touch_functions | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			       (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to send touch function command.",
			  __func__);

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
#endif

	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	if (ts->dex_mode) {
		input_info(true, &ts->client->dev, "%s: set dex mode.\n",
			   __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DEX_MODE,
					   &ts->dex_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set dex mode %x.\n", __func__,
				  ts->dex_mode);
	}

	if (ts->brush_mode) {
		input_info(true, &ts->client->dev, "%s: set brush mode.\n",
			   __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_BRUSH_MODE,
					   &ts->brush_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set brush mode.\n", __func__);
	}

	if (ts->touchable_area) {
		input_info(true, &ts->client->dev, "%s: set 16:9 mode.\n",
			   __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA,
					   &ts->touchable_area, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set 16:9 mode.\n", __func__);
	}

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: failed to write Sense_on.\n", __func__);

	enable_irq(ts->client->irq);

	complete_all(&ts->bus_resumed);

	mutex_unlock(&ts->device_mutex);
}

static void sec_ts_aggregate_bus_state(struct sec_ts_data *ts)
{
	input_dbg(true, &ts->client->dev, "%s: bus_refmask = 0x%02X.\n",
		  __func__, ts->bus_refmask);

	/* Complete or cancel any outstanding transitions */
	cancel_work_sync(&ts->suspend_work);
	cancel_work_sync(&ts->resume_work);

	if ((ts->bus_refmask == 0 &&
		ts->power_status == SEC_TS_STATE_SUSPEND) ||
	    (ts->bus_refmask != 0 &&
		ts->power_status != SEC_TS_STATE_SUSPEND))
		return;

	if (ts->bus_refmask == 0)
		schedule_work(&ts->suspend_work);
	else
		schedule_work(&ts->resume_work);
}

int sec_ts_set_bus_ref(struct sec_ts_data *ts, u16 ref, bool enable)
{
	int result = 0;

	mutex_lock(&ts->bus_mutex);

	input_dbg(true, &ts->client->dev, "%s: bus_refmask = 0x%02X.\n",
		  __func__, ref);

	if ((enable && (ts->bus_refmask & ref)) ||
	    (!enable && !(ts->bus_refmask & ref))) {
		input_info(true, &ts->client->dev,
			"%s: reference is unexpectedly set: mask=0x%04X, ref=0x%04X, enable=%d.\n",
			__func__, ts->bus_refmask, ref, enable);
		mutex_unlock(&ts->bus_mutex);
		return -EINVAL;
	}

	if (enable) {
		/* IRQs can only keep the bus active. IRQs received while the
		 * bus is transferred to SLPI should be ignored.
		 */
		if (ref == SEC_TS_BUS_REF_IRQ && ts->bus_refmask == 0)
			result = -EAGAIN;
		else
			ts->bus_refmask |= ref;
	} else
		ts->bus_refmask &= ~ref;
	sec_ts_aggregate_bus_state(ts);

	mutex_unlock(&ts->bus_mutex);

	/* When triggering a wake, wait up to one second to resume. SCREEN_ON
	 * and IRQ references do not need to wait.
	 */
	if (enable &&
	    ref != SEC_TS_BUS_REF_SCREEN_ON && ref != SEC_TS_BUS_REF_IRQ) {
		wait_for_completion_timeout(&ts->bus_resumed, HZ);
		if (ts->power_status != SEC_TS_STATE_POWER_ON) {
			input_info(true, &ts->client->dev,
				   "%s: Failed to wake the touch bus.\n",
				   __func__);
			result = -ETIMEDOUT;
		}
	}

	return result;
}

static int sec_ts_screen_state_chg_callback(struct notifier_block *nb,
					    unsigned long val, void *data)
{
	struct sec_ts_data *ts = container_of(nb, struct sec_ts_data,
					      notifier);
	struct msm_drm_notifier *evdata = (struct msm_drm_notifier *)data;
	unsigned int blank;

	input_dbg(true, &ts->client->dev, "%s: enter.\n", __func__);

	if (val != MSM_DRM_EVENT_BLANK && val != MSM_DRM_EARLY_EVENT_BLANK)
		return NOTIFY_DONE;

	if (!ts || !evdata || !evdata->data) {
		input_err(true, &ts->client->dev,
			  "%s: Bad screen state change notifier call.\n",
			  __func__);
		return NOTIFY_DONE;
	}

	blank = *((unsigned int *)evdata->data);
	switch (blank) {
	case MSM_DRM_BLANK_POWERDOWN:
	case MSM_DRM_BLANK_LP:
		if (val == MSM_DRM_EARLY_EVENT_BLANK) {
			input_dbg(true, &ts->client->dev,
				  "%s: MSM_DRM_BLANK_POWERDOWN.\n", __func__);
			sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SCREEN_ON, false);
		}
		break;
	case MSM_DRM_BLANK_UNBLANK:
		if (val == MSM_DRM_EVENT_BLANK) {
			input_dbg(true, &ts->client->dev,
				  "%s: MSM_DRM_BLANK_UNBLANK.\n", __func__);
			sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SCREEN_ON, true);
		}
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block sec_ts_screen_nb = {
	.notifier_call = sec_ts_screen_state_chg_callback,
};

#ifdef CONFIG_OF
static const struct of_device_id sec_ts_match_table[] = {
	{ .compatible = "sec,sec_ts",},
	{ },
};
#else
#define sec_ts_match_table NULL
#endif

static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SEC_TS_I2C_NAME,
#ifdef CONFIG_OF
		.of_match_table = sec_ts_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};

static int __init sec_ts_init(void)
{
#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		pr_err("%s %s: Do not load driver due to : lpm %d\n",
				SECLOG, __func__, lpcharge);
		return -ENODEV;
	}
#endif
	pr_err("%s %s\n", SECLOG, __func__);

	return i2c_add_driver(&sec_ts_driver);
}

static void __exit sec_ts_exit(void)
{
	i2c_del_driver(&sec_ts_driver);
}

MODULE_AUTHOR("Hyobae, Ahn<hyobae.ahn@samsung.com>");
MODULE_DESCRIPTION("Samsung Electronics TouchScreen driver");
MODULE_LICENSE("GPL");

module_init(sec_ts_init);
module_exit(sec_ts_exit);
