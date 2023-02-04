/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** File:
**     drv2624.c
**
** Description:
**     DRV2624 chip driver
**
** =============================================================================
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/syscalls.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "drv2624.h"

static struct drv2624_data *drv2624_plat_data;

static bool drv2624_is_volatile_reg(struct device *dev, unsigned int reg);

static int drv2624_reg_read(struct drv2624_data *drv2624, unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(drv2624->regmap, reg, &val);
	if (ret < 0) {
		dev_err(drv2624->dev, "%s reg=0x%x error %d\n", __func__, reg,
			ret);
		return ret;
	}

	dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__, reg, val);

	return val;
}

static int drv2624_reg_write(
	struct drv2624_data *drv2624, unsigned char reg, unsigned char val)
{
	int ret;

	ret = regmap_write(drv2624->regmap, reg, val);
	if (ret < 0) {
		dev_err(drv2624->dev, "%s reg=0x%x, value=0%x error %d\n",
			__func__, reg, val, ret);
	} else {
		dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__, reg,
			val);
	}

	return ret;
}

static int drv2624_bulk_write(struct drv2624_data *drv2624, unsigned char reg,
	const u8 *buf, unsigned int count)
{
	int ret, i;

	ret = regmap_bulk_write(drv2624->regmap, reg, buf, count);
	if (ret < 0) {
		dev_err(drv2624->dev, "%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, ret);
	} else {
		for (i = 0; i < count; i++)
			dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__,
				reg + i, buf[i]);
	}
	return ret;
}

static int drv2624_set_bits(struct drv2624_data *drv2624, unsigned char reg,
	unsigned char mask, unsigned char val)
{
	int ret;

	if (drv2624_is_volatile_reg(drv2624->dev, reg))
		ret = regmap_write_bits(drv2624->regmap, reg, mask, val);
	else
		ret = regmap_update_bits(drv2624->regmap, reg, mask, val);
	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=%x, mask=0x%x, value=0x%x error %d\n", __func__,
			reg, mask, val, ret);
	} else {
		dev_dbg(drv2624->dev, "%s, Reg[0x%x]:M=0x%x, V=0x%x\n",
			__func__, reg, mask, val);
	}

	return ret;
}

static void drv2624_enable_irq(struct drv2624_data *drv2624, bool rtp)
{
	unsigned char mask;

	if (!drv2624->irq)
		return;

	mask = INT_ENABLE_CRITICAL;

	if (rtp)
		mask = INT_ENABLE_ALL;

	drv2624_reg_read(drv2624, DRV2624_REG_STATUS);
	drv2624_reg_write(drv2624, DRV2624_REG_INT_ENABLE, mask);

	enable_irq(drv2624->irq);
}

static void drv2624_disable_irq(struct drv2624_data *drv2624)
{
	if (!drv2624->irq)
		return;

	disable_irq(drv2624->irq);
	drv2624_reg_write(drv2624, DRV2624_REG_INT_ENABLE, INT_MASK_ALL);
}

static void drv2624_reset(struct drv2624_data *drv2624)
{
	int ret;

	gpio_set_value(drv2624->plat_data.gpio_nrst, 0);
	usleep_range(1000, 2000);
	gpio_set_value(drv2624->plat_data.gpio_nrst, 1);
	usleep_range(1000, 2000);

	regcache_mark_dirty(drv2624->regmap);
	ret = regcache_sync(drv2624->regmap);
	if (ret) {
		dev_err(drv2624->dev, "Failed to sync cache: %d\n", ret);
		gpio_direction_output(drv2624->plat_data.gpio_nrst, 0);
	}
}

static int drv2624_poll_go_bit_stop(struct drv2624_data *drv2624)
{
	int ret, value;
	int poll_ready = POLL_GO_BIT_RETRY; /* to finish auto-brake */

	do {
		ret = drv2624_reg_read(drv2624, DRV2624_REG_GO);
		if (ret >= 0) {
			value = ret & 0x01;
			if (!value)
				return ret;
		}
		usleep_range(8000, 10000);
	} while (poll_ready--);

	dev_err(drv2624->dev, "%s, ERROR: failed to clear GO\n", __func__);

	return -EINVAL;
}

static int drv2624_set_go_bit(struct drv2624_data *drv2624, unsigned char val)
{
	int ret;
	int retry = POLL_GO_BIT_RETRY;

	val &= 0x01;

	do {
		ret = drv2624_set_bits(drv2624, DRV2624_REG_GO, 0x01, val);
		if (ret >= 0) {
			usleep_range(1000, 1100);
			/* Only poll GO bit for STOP */
			if (!val)
				ret = drv2624_poll_go_bit_stop(drv2624);
			return ret;
		}
		usleep_range(8000, 10000);
	} while (retry--);

	drv2624_reset(drv2624);

	return ret;
}

static void drv2624_change_mode(
	struct drv2624_data *drv2624, unsigned char work_mode)
{
	drv2624_set_bits(drv2624, DRV2624_REG_MODE, WORKMODE_MASK, work_mode);
}

static int drv2624_get_mode(struct drv2624_data *drv2624)
{
	return drv2624_reg_read(drv2624, DRV2624_REG_MODE) & WORKMODE_MASK;
}

static int drv2624_stop(struct drv2624_data *drv2624)
{
	int nResult = 0, mode = 0;

	nResult = drv2624_reg_read(drv2624, DRV2624_REG_MODE);
	if (nResult < 0)
		return 0;

	mode =  nResult & WORKMODE_MASK;
	if (mode == MODE_WAVEFORM_SEQUENCER) {
		dev_dbg(drv2624->dev, "In sequence play, ignore stop\n");
		return 0;
	}

	if (drv2624->vibrator_playing) {
		dev_dbg(drv2624->dev, "%s\n", __func__);
		drv2624_disable_irq(drv2624);
		nResult = drv2624_set_go_bit(drv2624, STOP);
		drv2624->work_mode = WORK_IDLE;
		drv2624->vibrator_playing = false;
		pm_relax(drv2624->dev);
	}

	return nResult;
}

static void drv2624_haptics_stopwork(struct work_struct *work)
{
	struct drv2624_data *drv2624 =
		container_of(work, struct drv2624_data, stop_work);

	mutex_lock(&drv2624->lock);

	drv2624_stop(drv2624);
	drv2624->led_dev.brightness = LED_OFF;

	mutex_unlock(&drv2624->lock);
}

static void drv2624_haptics_work(struct work_struct *work)
{
	struct drv2624_data *drv2624 =
		container_of(work, struct drv2624_data, work);
	int ret = 0;

	mutex_lock(&drv2624->lock);

	drv2624_stop(drv2624);

	pm_stay_awake(drv2624->dev);
	drv2624->vibrator_playing = true;
	drv2624_enable_irq(drv2624, true);

	ret = drv2624_set_go_bit(drv2624, GO);
	if (ret < 0) {
		dev_warn(drv2624->dev, "Start playback failed\n");
		drv2624->vibrator_playing = false;
		drv2624_disable_irq(drv2624);
		pm_relax(drv2624->dev);
	} else {
		drv2624->led_dev.brightness = LED_FULL;
		drv2624->work_mode |= WORK_VIBRATOR;
	}

	mutex_unlock(&drv2624->lock);
}

static void vibrator_enable(
	struct led_classdev *led_cdev, enum led_brightness value)
{
	struct drv2624_data *drv2624 =
		container_of(led_cdev, struct drv2624_data, led_dev);

	if (value == LED_OFF)
		queue_work(drv2624->drv2624_wq, &drv2624->stop_work);
	else
		queue_work(drv2624->drv2624_wq, &drv2624->work);
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2624_data *drv2624 =
		container_of(work, struct drv2624_data, vibrator_work);
	unsigned char mode = MODE_RTP;
	unsigned char status;
	int ret = 0;

	mutex_lock(&drv2624->lock);

	dev_dbg(drv2624->dev, "%s, afer mnWorkMode=0x%x\n", __func__,
		drv2624->work_mode);

	if (drv2624->work_mode & WORK_IRQ) {
		ret = drv2624_reg_read(drv2624, DRV2624_REG_STATUS);
		if (ret >= 0)
			drv2624->int_status = ret;

		drv2624_disable_irq(drv2624);

		if (ret < 0) {
			dev_err(drv2624->dev, "%s, reg read error\n", __func__);
			goto err;
		}

		status = drv2624->int_status;
		dev_dbg(drv2624->dev, "%s, status=0x%x\n", __func__,
			drv2624->int_status);

		if (status & OVERCURRENT_MASK) {
			dev_err(drv2624->dev,
				"ERROR, Over Current detected!!\n");
		}

		if (status & OVERTEMPRATURE_MASK) {
			dev_err(drv2624->dev,
				"ERROR, Over Temperature detected!!\n");
		}

		if (status & ULVO_MASK)
			dev_err(drv2624->dev, "ERROR, VDD drop observed!!\n");

		if (status & PRG_ERR_MASK)
			dev_err(drv2624->dev, "ERROR, PRG error!!\n");

		if (status & PROCESS_DONE_MASK) {
			ret = drv2624_reg_read(drv2624, DRV2624_REG_MODE);
			if (ret < 0) {
				dev_err(drv2624->dev, "%s, reg read error\n",
					__func__);
				goto err;
			}

			mode = ret & WORKMODE_MASK;
			if (mode == MODE_CALIBRATION) {
				drv2624->auto_cal_result.result = status;
				if ((status & DIAG_MASK) != DIAG_SUCCESS) {
					dev_err(drv2624->dev,
						"Calibration fail\n");
				} else {
					drv2624->auto_cal_result.cal_comp =
						drv2624_reg_read(drv2624,
							DRV2624_REG_CAL_COMP);
					drv2624->auto_cal_result.cal_bemf =
						drv2624_reg_read(drv2624,
							DRV2624_REG_CAL_BEMF);
					drv2624->auto_cal_result.cal_gain =
						drv2624_reg_read(drv2624,
							DRV2624_REG_CAL_COMP) &
						BEMFGAIN_MASK;
					dev_dbg(drv2624->dev,
						"AutoCal : Comp=0x%x, Bemf=0x%x, Gain=0x%x\n",
						drv2624->auto_cal_result
							.cal_comp,
						drv2624->auto_cal_result
							.cal_bemf,
						drv2624->auto_cal_result
							.cal_gain);
				}
			} else if (mode == MODE_DIAGNOSTIC) {
				drv2624->diag_result.result = status;
				if ((status & DIAG_MASK) != DIAG_SUCCESS) {
					dev_err(drv2624->dev,
						"Diagnostic fail\n");
				} else {
					drv2624->diag_result.diagz =
						drv2624_reg_read(drv2624,
							DRV2624_REG_DIAG_Z);
					drv2624->diag_result.diagk =
						drv2624_reg_read(drv2624,
							DRV2624_REG_DIAG_K);
					dev_dbg(drv2624->dev,
						"Diag : ZResult=0x%x, CurrentK=0x%x\n",
						drv2624->diag_result.diagz,
						drv2624->diag_result.diagk);
				}
			} else if (mode == MODE_WAVEFORM_SEQUENCER) {
				dev_dbg(drv2624->dev,
					"Waveform Sequencer Playback finished\n");
			} else if (mode == MODE_RTP) {
				dev_dbg(drv2624->dev, "RTP IRQ\n");
			}
		}

		if ((mode != MODE_RTP) && drv2624->vibrator_playing) {
			dev_info(drv2624->dev, "release wklock\n");
			drv2624->vibrator_playing = false;
			pm_relax(drv2624->dev);
		}

		drv2624->work_mode &= ~WORK_IRQ;
	}

	if (drv2624->work_mode & WORK_VIBRATOR) {
		drv2624_stop(drv2624);
		drv2624->work_mode &= ~WORK_VIBRATOR;
	}
err:

	mutex_unlock(&drv2624->lock);
}

static int drv2624_set_waveform(struct drv2624_data *drv2624,
	struct drv2624_waveform_sequencer *sequencer)
{
	int ret = 0;
	int i = 0;
	unsigned char loop[2] = { 0 };
	unsigned char effects[DRV2624_SEQUENCER_SIZE] = { 0 };
	unsigned char len = 0;

	for (i = 0; i < DRV2624_SEQUENCER_SIZE; i++) {
		len++;
		if (sequencer->waveform[i].effect != 0) {
			if (i < 4)
				loop[0] |= (sequencer->waveform[i].loop
					    << (2 * i));
			else
				loop[1] |= (sequencer->waveform[i].loop
					    << (2 * (i - 4)));

			effects[i] = sequencer->waveform[i].effect;
		} else {
			break;
		}
	}

	if (len == 1)
		ret = drv2624_reg_write(drv2624, DRV2624_REG_SEQUENCER_1, 0);
	else
		ret = drv2624_bulk_write(
			drv2624, DRV2624_REG_SEQUENCER_1, effects, len);

	if (ret < 0)
		dev_err(drv2624->dev, "sequence error\n");

	if ((ret >= 0) && (len > 1)) {
		if ((len - 1) <= 4)
			drv2624_reg_write(
				drv2624, DRV2624_REG_SEQ_LOOP_1, loop[0]);
		else
			drv2624_bulk_write(
				drv2624, DRV2624_REG_SEQ_LOOP_1, loop, 2);
	}

	return ret;
}

static int fw_chksum(const struct firmware *fw)
{
	int sum = 0;
	int i = 0;
	int size = fw->size;
	const unsigned char *buffer = fw->data;

	for (i = 0; i < size; i++) {
		if (!((i > 11) && (i < 16)))
			sum += buffer[i];
	}

	return sum;
}

/* drv2624_firmware_load:   This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to drv2624 data
 *
 *
 */
static void drv2624_firmware_load(const struct firmware *fw, void *context)
{
	struct drv2624_data *drv2624 = context;
	int size = 0, fwsize = 0, i = 0;
	const unsigned char *buffer = NULL;

	if (!context)
		return;

	mutex_lock(&drv2624->lock);

	if (fw) {
		buffer = fw->data;
		size = fw->size;
		if (size > 1024) {
			dev_err(drv2624->dev,
				"%s, ERROR!! firmware size %d too big\n",
				__func__, size);
		} else {
			memcpy(&drv2624->fw_header, buffer,
				sizeof(struct drv2624_fw_header));
			if ((drv2624->fw_header.fw_magic != DRV2624_MAGIC) ||
				(drv2624->fw_header.fw_size != size) ||
				(drv2624->fw_header.fw_chksum !=
					fw_chksum(fw))) {
				dev_err(drv2624->dev,
					"%s, ERROR!! firmware not right:Magic=0x%x,Size=%d,chksum=0x%x\n",
					__func__, drv2624->fw_header.fw_magic,
					drv2624->fw_header.fw_size,
					drv2624->fw_header.fw_chksum);
			} else {
				dev_info(drv2624->dev, "%s, firmware good\n",
					__func__);

				buffer += sizeof(struct drv2624_fw_header);

				drv2624_reg_write(
					drv2624, DRV2624_REG_RAM_ADDR_UPPER, 0);
				drv2624_reg_write(
					drv2624, DRV2624_REG_RAM_ADDR_LOWER, 0);

				fwsize =
					size - sizeof(struct drv2624_fw_header);

				for (i = 0; i < fwsize; i++) {
					drv2624_reg_write(drv2624,
						DRV2624_REG_RAM_DATA,
						buffer[i]);
				}
			}
		}
	} else {
		dev_err(drv2624->dev, "%s, ERROR!! firmware not found\n",
			__func__);
	}

	release_firmware(fw);

	mutex_unlock(&drv2624->lock);
}

static int haptics_init(struct drv2624_data *drv2624)
{
	int ret;

	drv2624->vibrator_playing = false;
	drv2624->led_dev.name = "vibrator";
	drv2624->led_dev.max_brightness = LED_FULL;
	drv2624->led_dev.brightness_set = vibrator_enable;
	drv2624->led_dev.flags = LED_BRIGHTNESS_FAST;

	ret = led_classdev_register(drv2624->dev, &drv2624->led_dev);
	if (ret) {
		dev_err(drv2624->dev, "drv2624: fail to create led classdev\n");
		return ret;
	}

	device_init_wakeup(drv2624->dev, false);
	mutex_init(&drv2624->lock);

	drv2624->drv2624_wq = alloc_ordered_workqueue("drv2624_wq", WQ_HIGHPRI);
	if (!drv2624->drv2624_wq) {
		dev_err(drv2624->dev,
			"drv2624: fail to alloc_workqueue for drv2624_wq\n");
		return -ENOMEM;
	}

	INIT_WORK(&drv2624->vibrator_work, vibrator_work_routine);
	INIT_WORK(&drv2624->work, drv2624_haptics_work);
	INIT_WORK(&drv2624->stop_work, drv2624_haptics_stopwork);

	return 0;
}

static void dev_init_platform_data(struct drv2624_data *drv2624)
{
	struct drv2624_platform_data *pdata = &drv2624->plat_data;
	struct actuator_data actuator = pdata->actuator;
	unsigned char value_temp = 0;
	unsigned char mask_temp = 0;

	drv2624_set_bits(drv2624, DRV2624_REG_MODE, PINFUNC_MASK,
		(PINFUNC_INT << PINFUNC_SHIFT));

	if ((actuator.actuator_type == ERM) ||
		(actuator.actuator_type == LRA)) {
		mask_temp |= ACTUATOR_MASK;
		value_temp |= (actuator.actuator_type << ACTUATOR_SHIFT);
	}

	if ((pdata->loop == CLOSE_LOOP) || (pdata->loop == OPEN_LOOP)) {
		mask_temp |= LOOP_MASK;
		value_temp |= (pdata->loop << LOOP_SHIFT);
	}

	if (value_temp != 0) {
		drv2624_set_bits(drv2624, DRV2624_REG_CONTROL1,
			mask_temp | AUTOBRK_OK_MASK,
			value_temp | AUTOBRK_OK_ENABLE);
	}

	value_temp = 0;
	if (actuator.actuator_type == ERM)
		value_temp = LIB_ERM;
	else if (actuator.actuator_type == LRA)
		value_temp = LIB_LRA;
	if (value_temp != 0) {
		drv2624_set_bits(drv2624, DRV2624_REG_CONTROL2, LIB_MASK,
			value_temp << LIB_SHIFT);
	}

	if (actuator.rated_voltage != 0) {
		drv2624_reg_write(drv2624, DRV2624_REG_RATED_VOLTAGE,
			actuator.rated_voltage);
	} else {
		dev_err(drv2624->dev, "%s, ERROR Rated ZERO\n", __func__);
	}

	if (actuator.over_drive_clamp_voltage != 0) {
		drv2624_reg_write(drv2624, DRV2624_REG_OVERDRIVE_CLAMP,
			actuator.over_drive_clamp_voltage);
	} else {
		dev_err(drv2624->dev, "%s, ERROR OverDriveVol ZERO\n",
			__func__);
	}

	if (actuator.actuator_type == LRA) {
		unsigned char drive_time =
			5 * (1000 - actuator.lra_freq) / actuator.lra_freq;
		unsigned short open_loop_period =
			(unsigned short)(1000000000 /
					 (24615 * actuator.lra_freq));

		if (actuator.lra_freq < 125)
			drive_time |= (MINFREQ_SEL_45HZ << MINFREQ_SEL_SHIFT);

		drv2624_set_bits(drv2624, DRV2624_REG_DRIVE_TIME,
			DRIVE_TIME_MASK | MINFREQ_SEL_MASK, drive_time);

		if (actuator.ol_lra_freq > -1)
			open_loop_period =
				(unsigned short)(1000000000 /
						 (24615 *
							 actuator.ol_lra_freq));

		drv2624_set_bits(drv2624, DRV2624_REG_OL_PERIOD_H, 0x03,
			(open_loop_period & 0x0300) >> 8);
		drv2624_reg_write(drv2624, DRV2624_REG_OL_PERIOD_L,
			(open_loop_period & 0x00ff));

		dev_info(drv2624->dev, "%s, LRA = %d, drive_time=0x%x\n",
			__func__, actuator.lra_freq, drive_time);

		if (actuator.lra_wave_shape > -1)
			drv2624_set_bits(drv2624, DRV2624_REG_LRA_OL_CTRL,
				LRA_WAVE_SHAPE_MASK, actuator.lra_wave_shape);
	}

	if (actuator.voltage_comp > -1)
		drv2624_reg_write(
			drv2624, DRV2624_REG_CAL_COMP, actuator.voltage_comp);

	if (actuator.bemf_factor > -1)
		drv2624_reg_write(
			drv2624, DRV2624_REG_CAL_BEMF, actuator.bemf_factor);

	if (actuator.bemf_gain > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_LOOP_CONTROL,
			BEMFGAIN_MASK, actuator.bemf_gain);

	if (actuator.fb_brake_factor > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_LOOP_CONTROL,
			FB_BRAKE_FACTOR_MASK, actuator.fb_brake_factor);

	if (actuator.blanking_time > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_BLK_IDISS_TIME,
			BLANKING_TIME_MASK,
			actuator.blanking_time << BLANKING_TIME_SHIFT);

	if (actuator.idiss_time > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_BLK_IDISS_TIME,
			IDISS_TIME_MASK, actuator.idiss_time);

	if (actuator.zc_det_time > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_ZC_OD_TIME,
			ZC_DET_TIME_MASK, actuator.zc_det_time);

	if (actuator.waveform_interval > -1)
		drv2624_set_bits(drv2624, DRV2624_REG_CONTROL2, INTERVAL_MASK,
			actuator.waveform_interval << INTERVAL_SHIFT);
}

static irqreturn_t drv2624_irq_handler(int irq, void *dev_id)
{
	struct drv2624_data *drv2624 = (struct drv2624_data *)dev_id;

	drv2624->work_mode |= WORK_IRQ;

	schedule_work(&drv2624->vibrator_work);

	return IRQ_HANDLED;
}

static int drv2624_parse_dt(struct device *dev, struct drv2624_data *drv2624)
{
	struct device_node *np = dev->of_node;
	struct drv2624_platform_data *pdata = &drv2624->plat_data;
	int ret = 0;
	unsigned int value;

	pdata->gpio_nrst = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_nrst)) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pdata->gpio_nrst);
		ret = -EINVAL;
		goto drv2624_parse_dt_out;
	}

	/* irq-gpio is optional */
	pdata->gpio_int = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_int)) {
		dev_warn(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, pdata->gpio_int);
	}

	ret = of_property_read_u32(np, "ti,smart-loop", &value);
	if (ret) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,smart-loop", np->full_name, ret);
		ret = -EINVAL;
		goto drv2624_parse_dt_out;
	}

	pdata->loop = value & 0x01;
	dev_dbg(drv2624->dev, "ti,smart-loop=%d\n", pdata->loop);

	ret = of_property_read_u32(np, "ti,actuator", &value);
	if (ret) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,actuator", np->full_name, ret);
		ret = -EINVAL;
		goto drv2624_parse_dt_out;
	}

	pdata->actuator.actuator_type = value & 0x01;
	dev_dbg(drv2624->dev, "ti,actuator=%d\n",
		pdata->actuator.actuator_type);

	ret = of_property_read_u32(np, "ti,rated-voltage", &value);
	if (ret) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,rated-voltage", np->full_name, ret);
		ret = -EINVAL;
		goto drv2624_parse_dt_out;
	}

	pdata->actuator.rated_voltage = value;
	dev_dbg(drv2624->dev, "ti,rated-voltage=0x%x\n",
		pdata->actuator.rated_voltage);

	ret = of_property_read_u32(np, "ti,odclamp-voltage", &value);
	if (ret) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,odclamp-voltage", np->full_name, ret);
		ret = -EINVAL;
		goto drv2624_parse_dt_out;
	}

	pdata->actuator.over_drive_clamp_voltage = value;
	dev_dbg(drv2624->dev, "ti,odclamp-voltage=0x%x\n",
		pdata->actuator.over_drive_clamp_voltage);

	if (pdata->actuator.actuator_type == LRA) {
		ret = of_property_read_u32(np, "ti,lra-frequency", &value);
		if (ret) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-frequency", np->full_name, ret);
			ret = -EINVAL;
			goto drv2624_parse_dt_out;
		} else {
			if ((value >= 45) && (value <= 300)) {
				pdata->actuator.lra_freq = value;
				dev_dbg(drv2624->dev, "ti,lra-frequency=%d\n",
					pdata->actuator.lra_freq);
			} else {
				dev_err(drv2624->dev,
					"ERROR, ti,lra-frequency=%d, out of range\n",
					value);
				ret = -EINVAL;
				goto drv2624_parse_dt_out;
			}
		}
	}

	/* actuator properties below are optional to have */

	if (!of_property_read_u32(np, "ti,voltage-comp", &value))
		pdata->actuator.voltage_comp = value;
	else
		pdata->actuator.voltage_comp = -1;

	dev_dbg(drv2624->dev, "ti,voltage-comp=%d\n",
		pdata->actuator.voltage_comp);

	if (!of_property_read_u32(np, "ti,ol-lra-frequency", &value)) {
		if ((value >= 45) && (value <= 300)) {
			pdata->actuator.ol_lra_freq = value;
		} else {
			pdata->actuator.ol_lra_freq = -1;
			dev_err(drv2624->dev,
				"ERROR, ti,ol-lra-frequency=%d, out of range\n",
				value);
		}
	} else {
		pdata->actuator.ol_lra_freq = -1;
	}

	dev_dbg(drv2624->dev, "ti,ol-lra-frequency=%d\n",
		pdata->actuator.ol_lra_freq);

	if (!of_property_read_u32(np, "ti,fb-brake-factor", &value))
		pdata->actuator.fb_brake_factor = value;
	else
		pdata->actuator.fb_brake_factor = -1;

	dev_dbg(drv2624->dev, "ti,fb-brake-factor=%d\n",
		pdata->actuator.fb_brake_factor);

	if (!of_property_read_u32(np, "ti,bemf-factor", &value))
		pdata->actuator.bemf_factor = value;
	else
		pdata->actuator.bemf_factor = -1;

	dev_dbg(drv2624->dev, "ti,bemf-factor=%d\n",
		pdata->actuator.bemf_factor);

	if (!of_property_read_u32(np, "ti,bemf-gain", &value))
		pdata->actuator.bemf_gain = value;
	else
		pdata->actuator.bemf_gain = -1;

	dev_dbg(drv2624->dev, "ti,bemf-gain=%d\n", pdata->actuator.bemf_gain);

	if (!of_property_read_u32(np, "ti,blanking-time", &value))
		pdata->actuator.blanking_time = value;
	else
		pdata->actuator.blanking_time = -1;

	dev_dbg(drv2624->dev, "ti,blanking-time=%d\n",
		pdata->actuator.ol_lra_freq);

	if (!of_property_read_u32(np, "ti,idiss-time", &value))
		pdata->actuator.idiss_time = value;
	else
		pdata->actuator.idiss_time = -1;

	dev_dbg(drv2624->dev, "ti,idiss-time=%d\n", pdata->actuator.idiss_time);

	if (!of_property_read_u32(np, "ti,zc-det-time", &value))
		pdata->actuator.zc_det_time = value;
	else
		pdata->actuator.zc_det_time = -1;

	dev_dbg(drv2624->dev, "ti,zc-det-time=%d\n",
		pdata->actuator.zc_det_time);

	if (!of_property_read_u32(np, "ti,lra-wave-shape", &value))
		pdata->actuator.lra_wave_shape = value;
	else
		pdata->actuator.lra_wave_shape = -1;

	dev_dbg(drv2624->dev, "ti,lra-wave-shape=%d\n",
		pdata->actuator.lra_wave_shape);

	if (!of_property_read_u32(np, "ti,waveform-interval", &value))
		pdata->actuator.waveform_interval = value;
	else
		pdata->actuator.waveform_interval = -1;

	dev_dbg(drv2624->dev, "ti,waveform-interval=%d\n",
		pdata->actuator.waveform_interval);

drv2624_parse_dt_out:
	return ret;
}

static const struct reg_default drv2624_reg_defaults[] = {
	{ DRV2624_REG_ID, 0x03 },
	{ DRV2624_REG_STATUS, 0x00 },
	{ DRV2624_REG_INT_ENABLE, INT_MASK_ALL },
	{ DRV2624_REG_DIAG_Z, 0x00 },
	{ DRV2624_REG_MODE, 0x44 },
	{ DRV2624_REG_LRA_PERIOD_H, 0x00 },
	{ DRV2624_REG_LRA_PERIOD_L, 0x00 },
	{ DRV2624_REG_CONTROL1, 0x88 },
	{ DRV2624_REG_GO, 0x00 },
	{ DRV2624_REG_CONTROL2, 0x00 },
	{ DRV2624_REG_RTP_INPUT, 0x7F },
	{ DRV2624_REG_SEQUENCER_1, 0x01 },
	{ DRV2624_REG_SEQ_LOOP_1, 0x00 },
	{ DRV2624_REG_SEQ_LOOP_2, 0x00 },
	{ DRV2624_REG_MAIN_LOOP, 0x00 },
	{ DRV2624_REG_RATED_VOLTAGE, 0x3F },
	{ DRV2624_REG_OVERDRIVE_CLAMP, 0x89 },
	{ DRV2624_REG_CAL_COMP, 0x0D },
	{ DRV2624_REG_CAL_BEMF, 0x6D },
	{ DRV2624_REG_LOOP_CONTROL, 0x36 },
	{ DRV2624_REG_DRIVE_TIME, 0x10 },
	{ DRV2624_REG_BLK_IDISS_TIME, 0x11 },
	{ DRV2624_REG_ZC_OD_TIME, 0x0C },
	{ DRV2624_REG_LRA_OL_CTRL, 0x00 },
	{ DRV2624_REG_OL_PERIOD_H, 0x00 },
	{ DRV2624_REG_OL_PERIOD_L, 0xC6 },
	{ DRV2624_REG_DIAG_K, 0x55 },
	{ DRV2624_REG_RAM_ADDR_UPPER, 0x00 },
	{ DRV2624_REG_RAM_ADDR_LOWER, 0x00 },
	{ DRV2624_REG_RAM_DATA, 0x32 },
};

static bool drv2624_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DRV2624_REG_GO:
	case DRV2624_REG_LRA_PERIOD_H:
	case DRV2624_REG_LRA_PERIOD_L:
	case DRV2624_REG_LOOP_CONTROL:
	case DRV2624_REG_DIAG_Z:
	case DRV2624_REG_DIAG_K:
	case DRV2624_REG_CAL_COMP:
	case DRV2624_REG_CAL_BEMF:
	case DRV2624_REG_RAM_ADDR_UPPER:
	case DRV2624_REG_RAM_ADDR_LOWER:
	case DRV2624_REG_RAM_DATA:
		return true;
	default:
		return false;
	}
}

static bool drv2624_is_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DRV2624_REG_STATUS:
		return true;
	default:
		return false;
	}
}

static bool drv2624_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DRV2624_REG_ID:
	case DRV2624_REG_STATUS:
	case DRV2624_REG_DIAG_Z:
	case DRV2624_REG_DIAG_K:
	case DRV2624_REG_LRA_PERIOD_H:
	case DRV2624_REG_LRA_PERIOD_L:
		return false;
	default:
		return true;
	}
}

static struct regmap_config drv2624_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_defaults = drv2624_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(drv2624_reg_defaults),
	.volatile_reg = drv2624_is_volatile_reg,
	.precious_reg = drv2624_is_precious_reg,
	.writeable_reg = drv2624_is_writeable_reg,
	.max_register = DRV2624_REG_RAM_DATA,
	.cache_type = REGCACHE_RBTREE,
	.can_multi_write = true,
};

static ssize_t rtp_input_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int rtp_input;

	mutex_lock(&drv2624->lock);
	rtp_input = drv2624_reg_read(drv2624, DRV2624_REG_RTP_INPUT);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", rtp_input);
}

static ssize_t rtp_input_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	char rtp_input;

	ret = kstrtos8(buf, 10, &rtp_input);
	if (ret) {
		pr_err("Invalid input for rtp_input: ret = %d\n", ret);
		return ret;
	}

	mutex_lock(&drv2624->lock);
	drv2624_reg_write(drv2624, DRV2624_REG_RTP_INPUT, rtp_input);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t mode_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int mode;

	mutex_lock(&drv2624->lock);
	mode = drv2624_get_mode(drv2624);
	mutex_unlock(&drv2624->lock);

	if (mode >= ARRAY_SIZE(drv2624_modes) || mode < 0) {
		pr_err("Invalid mode: mode = %d\n", mode);
		return snprintf(buf, PAGE_SIZE, "%d\n", mode);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", drv2624_modes[mode]);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	char mode_name[25];
	size_t len;
	unsigned char new_mode;

	mode_name[sizeof(mode_name) - 1] = '\0';
	strlcpy(mode_name, buf, sizeof(mode_name) - 1);
	len = strlen(mode_name);

	if (len && mode_name[len - 1] == '\n')
		mode_name[len - 1] = '\0';

	for (new_mode = 0; new_mode < ARRAY_SIZE(drv2624_modes); new_mode++) {
		if (!strcmp(mode_name, drv2624_modes[new_mode])) {
			mutex_lock(&drv2624->lock);
			drv2624_change_mode(drv2624, new_mode);
			mutex_unlock(&drv2624->lock);
			break;
		}
	}

	return count;
}

static ssize_t loop_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int loop;

	mutex_lock(&drv2624->lock);
	loop = drv2624_reg_read(drv2624, DRV2624_REG_MAIN_LOOP) &
	       MAIN_LOOP_MASK;
	mutex_unlock(&drv2624->lock);

	if (loop == MAIN_LOOP_MASK)
		loop = -1; /* infinite */

	return snprintf(buf, PAGE_SIZE, "%d\n", loop);
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	int loop;

	ret = kstrtoint(buf, 10, &loop);
	if (ret) {
		pr_err("Invalid input for loop: ret = %d\n", ret);
		return ret;
	}

	loop = max(min(loop, MAIN_LOOP_MASK), -1);

	if (loop == -1)
		loop = MAIN_LOOP_MASK; /*infinite */

	mutex_lock(&drv2624->lock);
	drv2624_reg_write(drv2624, DRV2624_REG_MAIN_LOOP, loop);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t interval_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int interval;

	mutex_lock(&drv2624->lock);
	interval = drv2624_reg_read(drv2624, DRV2624_REG_CONTROL2);
	interval = ((interval & INTERVAL_MASK) >> INTERVAL_SHIFT);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", interval);
}

static ssize_t interval_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	int interval;

	ret = kstrtoint(buf, 10, &interval);
	if (ret) {
		pr_err("Invalid input for loop: ret = %d\n", ret);
		return ret;
	}

	interval = max(min(interval, 1), 0);

	mutex_lock(&drv2624->lock);
	drv2624_set_bits(drv2624, DRV2624_REG_CONTROL2, INTERVAL_MASK,
		interval << INTERVAL_SHIFT);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t scale_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int interval;

	mutex_lock(&drv2624->lock);
	interval = drv2624_reg_read(drv2624, DRV2624_REG_CONTROL2);
	interval = interval & SCALE_MASK;
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", interval);
}

static ssize_t scale_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	int interval;

	ret = kstrtoint(buf, 10, &interval);
	if (ret) {
		pr_err("Invalid input for loop: ret = %d\n", ret);
		return ret;
	}

	interval = max(min(interval, 3), 0);

	mutex_lock(&drv2624->lock);
	drv2624_set_bits(drv2624, DRV2624_REG_CONTROL2, SCALE_MASK, interval);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t ctrl_loop_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ctrl_loop;

	mutex_lock(&drv2624->lock);
	ctrl_loop = drv2624_reg_read(drv2624, DRV2624_REG_CONTROL1);
	ctrl_loop = (ctrl_loop & LOOP_MASK) >> LOOP_SHIFT;
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", ctrl_loop);
}

static ssize_t ctrl_loop_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	int ctrl_loop;

	ret = kstrtoint(buf, 10, &ctrl_loop);
	if (ret) {
		pr_err("Invalid input for loop: ret = %d\n", ret);
		return ret;
	}

	ctrl_loop = max(min(ctrl_loop, 1), 0);

	mutex_lock(&drv2624->lock);
	drv2624_set_bits(drv2624, DRV2624_REG_CONTROL1, LOOP_MASK,
		ctrl_loop << LOOP_SHIFT);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t set_sequencer_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	struct drv2624_waveform_sequencer sequencer;
	int n;

	memset(&sequencer, 0, sizeof(sequencer));

	n = sscanf(buf,
		"%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
		&sequencer.waveform[0].effect, &sequencer.waveform[0].loop,
		&sequencer.waveform[1].effect, &sequencer.waveform[1].loop,
		&sequencer.waveform[2].effect, &sequencer.waveform[2].loop,
		&sequencer.waveform[3].effect, &sequencer.waveform[3].loop,
		&sequencer.waveform[4].effect, &sequencer.waveform[4].loop,
		&sequencer.waveform[5].effect, &sequencer.waveform[5].loop,
		&sequencer.waveform[6].effect, &sequencer.waveform[6].loop,
		&sequencer.waveform[7].effect, &sequencer.waveform[7].loop);
	if (n > DRV2624_SEQUENCER_SIZE * 2)
		return -EINVAL;

	mutex_lock(&drv2624->lock);
	drv2624_set_waveform(drv2624, &sequencer);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t od_clamp_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int od_clamp;

	mutex_lock(&drv2624->lock);
	od_clamp = drv2624_reg_read(drv2624, DRV2624_REG_OVERDRIVE_CLAMP);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", od_clamp);
}

static ssize_t od_clamp_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	unsigned char od_clamp;

	ret = kstrtou8(buf, 10, &od_clamp);
	if (ret) {
		pr_err("Invalid input for rtp_input: ret = %d\n", ret);
		return ret;
	}

	mutex_lock(&drv2624->lock);
	drv2624_reg_write(drv2624, DRV2624_REG_OVERDRIVE_CLAMP, od_clamp);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t diag_result_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int diag_z, diag_k;

	mutex_lock(&drv2624->lock);
	diag_z = drv2624_reg_read(drv2624, DRV2624_REG_DIAG_Z);
	diag_k = drv2624_reg_read(drv2624, DRV2624_REG_DIAG_K);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "z=%d k=%d\n", diag_z, diag_k);
}

static ssize_t autocal_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 autocal_comp, autocal_bemf, autocal_gain;

	mutex_lock(&drv2624->lock);
	autocal_comp = drv2624_reg_read(drv2624, DRV2624_REG_CAL_COMP);
	autocal_bemf = drv2624_reg_read(drv2624, DRV2624_REG_CAL_BEMF);
	autocal_gain = drv2624_reg_read(drv2624, DRV2624_REG_LOOP_CONTROL) &
		       BEMFGAIN_MASK;
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", autocal_comp,
		autocal_bemf, autocal_gain);
}

static ssize_t autocal_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int n;
	unsigned char comp, bemf, bemfgain;

	n = sscanf(buf, "%hhu %hhu %hhu", &comp, &bemf, &bemfgain);
	if (n != 3)
		return -EINVAL;

	mutex_lock(&drv2624->lock);
	drv2624_reg_write(drv2624, DRV2624_REG_CAL_COMP, comp);
	drv2624_reg_write(drv2624, DRV2624_REG_CAL_BEMF, bemf);
	drv2624_set_bits(
		drv2624, DRV2624_REG_LOOP_CONTROL, BEMFGAIN_MASK, bemfgain);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t lra_period_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 lra_period;

	mutex_lock(&drv2624->lock);
	lra_period = drv2624_reg_read(drv2624, DRV2624_REG_LRA_PERIOD_H) & 0x03;
	lra_period = (lra_period << 8) |
		     drv2624_reg_read(drv2624, DRV2624_REG_LRA_PERIOD_L);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", lra_period);
}

static ssize_t status_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 status;

	mutex_lock(&drv2624->lock);
	status = drv2624_reg_read(drv2624, DRV2624_REG_STATUS);
	/* Ignore Bit 6-5 reserved, Bit 4 PRG_ERR and BIT 3 PROC_DONE */
	status &= ~((1 << 6) | (1 << 5) | (1 << 4) | (1 << 3));
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t ol_lra_period_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 ol_lra_period;

	mutex_lock(&drv2624->lock);
	ol_lra_period =
		drv2624_reg_read(drv2624, DRV2624_REG_OL_PERIOD_H) & 0x03;
	ol_lra_period = (ol_lra_period << 8) |
			drv2624_reg_read(drv2624, DRV2624_REG_OL_PERIOD_L);
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", ol_lra_period);
}

static ssize_t ol_lra_period_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	u32 ol_lra_period;

	ret = kstrtou32(buf, 10, &ol_lra_period);
	if (ret) {
		pr_err("Invalid input for ol_lra_period: ret = %d\n", ret);
		return ret;
	}

	mutex_lock(&drv2624->lock);
	drv2624_reg_write(
		drv2624, DRV2624_REG_OL_PERIOD_H, (ol_lra_period >> 8) & 0x03);
	drv2624_reg_write(
		drv2624, DRV2624_REG_OL_PERIOD_L, ol_lra_period & 0xFF);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t lra_wave_shape_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 lra_wave_shape;

	mutex_lock(&drv2624->lock);
	lra_wave_shape = drv2624_reg_read(drv2624, DRV2624_REG_LRA_OL_CTRL) &
			 LRA_WAVE_SHAPE_MASK;
	mutex_unlock(&drv2624->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", lra_wave_shape);
}

static ssize_t lra_wave_shape_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;
	u32 lra_wave_shape;

	ret = kstrtou32(buf, 10, &lra_wave_shape);
	if (ret) {
		dev_err(dev, "Invalid input for ol_lra_period: ret = %d\n",
			ret);
		return ret;
	}

	mutex_lock(&drv2624->lock);
	drv2624_set_bits(drv2624, DRV2624_REG_LRA_OL_CTRL, LRA_WAVE_SHAPE_MASK,
		lra_wave_shape);
	mutex_unlock(&drv2624->lock);

	return count;
}

static ssize_t lp_trigger_effect_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", drv2624->lp_trigger_effect);
}

static ssize_t lp_trigger_effect_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtou8(buf, 10, &drv2624->lp_trigger_effect);
	if (ret) {
		dev_err(dev, "Invalid input for lp_trigger_effect: ret = %d\n",
			ret);
		return ret;
	}

	return count;
}

static ssize_t dump_reg_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	unsigned char i;
	ssize_t ret = 0;

	mutex_lock(&drv2624->lock);
	for (i = 0; i < 0x31; i++) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"reg(0x%x) = 0x%x\n", (unsigned int)i,
			(unsigned int)drv2624_reg_read(drv2624, i));
	}
	mutex_unlock(&drv2624->lock);

	return ret;
}

static ssize_t dump_reg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	u32 reg = 0, value = 0;
	int n;

	n = sscanf(buf, "%x %x", &reg, &value);

	if (n != 2)
		return -EINVAL;

	drv2624_reg_write(drv2624, reg, value);

	return count;
}

static DEVICE_ATTR(rtp_input, 0660, rtp_input_show, rtp_input_store);
static DEVICE_ATTR(mode, 0660, mode_show, mode_store);
static DEVICE_ATTR(loop, 0660, loop_show, loop_store);
static DEVICE_ATTR(interval, 0660, interval_show, interval_store);
static DEVICE_ATTR(scale, 0660, scale_show, scale_store);
static DEVICE_ATTR(ctrl_loop, 0660, ctrl_loop_show, ctrl_loop_store);
static DEVICE_ATTR(set_sequencer, 0660, NULL, set_sequencer_store);
static DEVICE_ATTR(od_clamp, 0660, od_clamp_show, od_clamp_store);
static DEVICE_ATTR(diag_result, 0600, diag_result_show, NULL);
static DEVICE_ATTR(autocal, 0660, autocal_show, autocal_store);
static DEVICE_ATTR(lra_period, 0600, lra_period_show, NULL);
static DEVICE_ATTR(status, 0600, status_show, NULL);
static DEVICE_ATTR(
	ol_lra_period, 0660, ol_lra_period_show, ol_lra_period_store);
static DEVICE_ATTR(
	lra_wave_shape, 0660, lra_wave_shape_show, lra_wave_shape_store);
static DEVICE_ATTR(lp_trigger_effect, 0660, lp_trigger_effect_show,
	lp_trigger_effect_store);
static DEVICE_ATTR(dump_reg, 0660, dump_reg_show, dump_reg_store);

static struct attribute *drv2624_fs_attrs[] = {
	&dev_attr_rtp_input.attr,
	&dev_attr_mode.attr,
	&dev_attr_loop.attr,
	&dev_attr_interval.attr,
	&dev_attr_scale.attr,
	&dev_attr_ctrl_loop.attr,
	&dev_attr_set_sequencer.attr,
	&dev_attr_od_clamp.attr,
	&dev_attr_diag_result.attr,
	&dev_attr_autocal.attr,
	&dev_attr_lra_period.attr,
	&dev_attr_status.attr,
	&dev_attr_ol_lra_period.attr,
	&dev_attr_lra_wave_shape.attr,
	&dev_attr_lp_trigger_effect.attr,
	&dev_attr_dump_reg.attr,
	NULL,
};

static struct attribute_group drv2624_fs_attr_group = {
	.attrs = drv2624_fs_attrs,
};

static int drv2624_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct drv2624_data *drv2624;
	int err = 0;

	dev_info(&client->dev, "%s enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s:I2C check failed\n", __func__);
		return -ENODEV;
	}

	drv2624 = devm_kzalloc(
		&client->dev, sizeof(struct drv2624_data), GFP_KERNEL);
	if (!drv2624) {
		dev_err(&client->dev, "%s:no memory\n", __func__);
		return -ENOMEM;
	}

	drv2624->dev = &client->dev;
	i2c_set_clientdata(client, drv2624);
	dev_set_drvdata(&client->dev, drv2624);

	drv2624->regmap = devm_regmap_init_i2c(client, &drv2624_i2c_regmap);
	if (IS_ERR(drv2624->regmap)) {
		err = PTR_ERR(drv2624->regmap);
		dev_err(drv2624->dev,
			"%s:Failed to allocate register map: %d\n", __func__,
			err);
		return err;
	}

	if (client->dev.of_node) {
		dev_dbg(drv2624->dev, "of node parse\n");
		err = drv2624_parse_dt(&client->dev, drv2624);
		if (err) {
			dev_err(drv2624->dev, "%s: platform data error\n",
				__func__);
			return -ENODEV;
		}
	} else if (client->dev.platform_data) {
		dev_dbg(drv2624->dev, "platform data parse\n");
		memcpy(&drv2624->plat_data, client->dev.platform_data,
			sizeof(drv2624->plat_data));
	} else {
		dev_err(drv2624->dev, "%s: ERROR no platform data\n", __func__);
		return -ENODEV;
	}

	if (gpio_is_valid(drv2624->plat_data.gpio_nrst)) {
		err = devm_gpio_request(&client->dev,
			drv2624->plat_data.gpio_nrst, "DRV2624-NRST");
		if (err < 0) {
			dev_err(drv2624->dev,
				"%s: GPIO %d request NRST error\n", __func__,
				drv2624->plat_data.gpio_nrst);
			return err;
		}

		gpio_direction_output(drv2624->plat_data.gpio_nrst, 0);
		usleep_range(1000, 2000);
		gpio_direction_output(drv2624->plat_data.gpio_nrst, 1);
		usleep_range(1000, 2000); /* t(on) = 1ms */
	}

	err = drv2624_reg_read(drv2624, DRV2624_REG_ID);
	if (err < 0) {
		dev_err(drv2624->dev, "%s, i2c bus fail (%d)\n", __func__, err);
		goto drv2624_i2c_probe_err;
	} else {
		dev_info(drv2624->dev, "%s, ID status (0x%x)\n", __func__, err);
		drv2624->device_id = err;
	}

	if ((drv2624->device_id & 0xf0) != DRV2624_ID) {
		dev_err(drv2624->dev, "%s, device_id(0x%x) fail\n", __func__,
			drv2624->device_id);
		goto drv2624_i2c_probe_err;
	}

	dev_init_platform_data(drv2624);

	if (gpio_is_valid(drv2624->plat_data.gpio_int)) {
		err = devm_gpio_request(&client->dev,
			drv2624->plat_data.gpio_int, "DRV2624-IRQ");
		if (err < 0) {
			dev_err(drv2624->dev, "%s: GPIO %d request INT error\n",
				__func__, drv2624->plat_data.gpio_int);
			goto drv2624_i2c_probe_err;
		}

		gpio_direction_input(drv2624->plat_data.gpio_int);

		drv2624->irq = gpio_to_irq(drv2624->plat_data.gpio_int);
		dev_dbg(drv2624->dev, "irq = %d\n", drv2624->irq);

		err = devm_request_threaded_irq(&client->dev, drv2624->irq,
			drv2624_irq_handler, NULL,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name,
			drv2624);

		if (err < 0) {
			dev_err(drv2624->dev, "request_irq failed, %d\n", err);
			goto drv2624_i2c_probe_err;
		}
		drv2624_disable_irq(drv2624);
	}

	drv2624_plat_data = drv2624;

	err = haptics_init(drv2624);
	if (err)
		goto drv2624_i2c_probe_err;

	err = sysfs_create_group(&drv2624->dev->kobj, &drv2624_fs_attr_group);
	if (err)
		goto drv2624_i2c_probe_err;

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "drv2624.bin",
		&client->dev, GFP_KERNEL, drv2624, drv2624_firmware_load);

	dev_info(drv2624->dev, "drv2624 probe succeeded\n");

	return 0;

drv2624_i2c_probe_err:

	dev_err(drv2624->dev, "%s failed, err=%d\n", __func__, err);
	return err;
}

static int drv2624_i2c_remove(struct i2c_client *client)
{
	struct drv2624_data *drv2624 = i2c_get_clientdata(client);

	cancel_work_sync(&drv2624->vibrator_work);
	cancel_work_sync(&drv2624->work);
	destroy_workqueue(drv2624->drv2624_wq);

	led_classdev_unregister(&drv2624->led_dev);

	mutex_destroy(&drv2624->lock);

	return 0;
}

static const struct i2c_device_id drv2624_i2c_id[] = { { "drv2624", 0 }, {} };

MODULE_DEVICE_TABLE(i2c, drv2624_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id drv2624_of_match[] = {
	{ .compatible = "ti,drv2624" },
	{},
};

MODULE_DEVICE_TABLE(of, drv2624_of_match);
#endif

void drv2624_trigger_mode(struct drv2624_data *drv2624, int mode)
{
	drv2624_set_bits(drv2624, DRV2624_REG_MODE, PINFUNC_MASK,
		(mode << PINFUNC_SHIFT));
}

#ifdef CONFIG_PM
static int drv2624_suspend(struct device *dev)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);
	struct drv2624_waveform_sequencer sequencer;

	memset(&sequencer, 0, sizeof(sequencer));

	cancel_work_sync(&drv2624->vibrator_work);
	cancel_work_sync(&drv2624->work);
	cancel_work_sync(&drv2624->stop_work);
	drv2624_stop(drv2624);

	if (drv2624->lp_trigger_effect) {
		sequencer.waveform[0].effect = drv2624->lp_trigger_effect;
		drv2624_set_waveform(drv2624, &sequencer);

		drv2624_change_mode(drv2624, MODE_WAVEFORM_SEQUENCER);
		drv2624_set_bits(drv2624, DRV2624_REG_CONTROL1, LOOP_MASK,
			0 << LOOP_SHIFT);

		drv2624_trigger_mode(drv2624, PINFUNC_TRIG_PULSE);
	}

	return 0;
}

static int drv2624_resume(struct device *dev)
{
	struct drv2624_data *drv2624 = dev_get_drvdata(dev);

	drv2624_trigger_mode(drv2624, PINFUNC_INT);

	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(drv2624_pm_ops, drv2624_suspend, drv2624_resume);

static struct i2c_driver drv2624_i2c_driver = {
	.driver = {
			.name = "drv2624",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(drv2624_of_match),
#endif
			.pm	= &drv2624_pm_ops,
			},
	.probe = drv2624_i2c_probe,
	.remove = drv2624_i2c_remove,
	.id_table = drv2624_i2c_id,
};

module_i2c_driver(drv2624_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("DRV2624 I2C Smart Haptics driver");
MODULE_LICENSE("GPL v2");
