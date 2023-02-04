/*
 * Copyright (C) 2017-2018 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>


#include "s2mpg01-core.h"

#define DRIVER_NAME "s2mpg01"

/* defines the number of tries to repeat an I2C transaction */
#define S2MPG01_I2C_RETRY_COUNT 2

/* defines the delay in ms for reset completion */
#define S2MPG01_PON_RESET_DELAY 2

/* defines the timeout in jiffies for reset completion */
#define S2MPG01_PON_RESET_TIMEOUT msecs_to_jiffies(15)

/* defines the number of retries for powering on */
#define S2MPG01_PON_RETRY_CNT 1

/* defines the timeout in jiffies for reset completion after shutdown */
#define S2MPG01_SHUTDOWN_RESET_TIMEOUT msecs_to_jiffies(1000)

/* defines the timeout in jiffies for ADC conversion completion */
#define S2MPG01_ADC_CONV_TIMEOUT  msecs_to_jiffies(100)

static void s2mpg01_print_status(struct s2mpg01_core *ddata);
static int s2mpg01_core_fixup(struct s2mpg01_core *ddata);
static void s2mpg01_mask_interrupts_all(struct s2mpg01_core *ddata);
static void s2mpg01_unmask_interrupts_all(struct s2mpg01_core *ddata);
static void s2mpg01_mask_boost_mode_interrupts(struct s2mpg01_core *ddata);
static void s2mpg01_unmask_boost_mode_interrupts(struct s2mpg01_core *ddata);

static const struct mfd_cell s2mpg01_devs[] = {
	{
		.name = "s2mpg01-regulator",
		.of_compatible = "samsung,s2mpg01-regulator",
	},
	{
		.name = "s2mpg01-gpio",
		.of_compatible = "samsung,s2mpg01-gpio",
	},
	{
		.name = "s2mpg01-thermal",
		.of_compatible = "samsung,s2mpg01-thermal",
	},
};

static const struct regmap_config s2mpg01_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

void s2mpg01_toggle_pon_oneway(struct s2mpg01_core *ddata, bool turn_on)
{
	unsigned long timeout;

	dev_info(ddata->dev, "%s: setting PON to %d\n", __func__, turn_on);

	if (!turn_on) {
		gpio_set_value_cansleep(ddata->pdata->pon_gpio, 0);
		return;
	}

	reinit_completion(&ddata->init_complete);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	/* wait for chip to come out of reset */
	timeout = wait_for_completion_timeout(&ddata->init_complete,
					      S2MPG01_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);
}

void s2mpg01_toggle_pon(struct s2mpg01_core *ddata)
{
	s2mpg01_toggle_pon_oneway(ddata, /*turn_on=*/false);
	msleep(20);
	s2mpg01_toggle_pon_oneway(ddata, /*turn_on=*/true);
}
EXPORT_SYMBOL_GPL(s2mpg01_toggle_pon);

int s2mpg01_dump_regs(struct s2mpg01_core *ddata)
{
	u8 reg_data;
	int i;

	for (i = 0; i <= 0x40; i++) {
		s2mpg01_read_byte(ddata, i, &reg_data);
		dev_info(ddata->dev, "[0x%02x] = 0x%02x\n", i, reg_data);
	}

	s2mpg01_read_byte(ddata, S2MPG01_REG_TIME_CTRL2, &reg_data); /* 0x51 */
	dev_info(ddata->dev,
		 "[0x%02x] = 0x%02x\n",
		 S2MPG01_REG_TIME_CTRL2, reg_data);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg01_dump_regs);

static inline bool s2mpg01_is_ready(struct s2mpg01_core *ddata)
{
#if 0
	return (gpio_get_value(ddata->pdata->pmic_ready_gpio) == 1);
#else
	return true;
#endif
}

int s2mpg01_read_byte(struct s2mpg01_core *ddata, u8 addr, u8 *data)
{
	int ret;
	unsigned int val;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_read(ddata->regmap, addr, &val);
		if (!ret) {
			*data = (u8)val;
			return 0;
		}

		dev_err(ddata->dev,
			"failed to read addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_read_byte);

int s2mpg01_read_bytes(struct s2mpg01_core *ddata, u8 addr, u8 *data,
		       size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_read(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev, "%s: failed to read %zd bytes from addr 0x%.2x (%d), retry %d\n",
			__func__, count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_read_bytes);

int s2mpg01_write_byte(struct s2mpg01_core *ddata, u8 addr, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_write(ddata->regmap, addr, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_write_byte);

int s2mpg01_write_bytes(struct s2mpg01_core *ddata, u8 addr, u8 *data,
			size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_write(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write %zd bytes to addr 0x%.2x (%d), retry %d\n",
			count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_write_bytes);

int s2mpg01_update_bits(struct s2mpg01_core *ddata, u8 addr,
			unsigned int mask, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_update_bits(ddata->regmap, addr, mask, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to update addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_update_bits);

int __s2mpg01_read_adc_chan(struct s2mpg01_core *ddata,
			    int chan_num,
			    u8 *chan_data,
			    bool one_shot_mode)
{
	int ret = 0;
	unsigned long timeout;

	/* serialize manual adc conversions */
	if (test_and_set_bit(0, &ddata->adc_conv_busy))
		return -EAGAIN;

	/* clear adc conversion completion before starting conversion */
	reinit_completion(&ddata->adc_conv_complete);

	/* write the channel number */
	ret = s2mpg01_write_byte(ddata, S2MPG01_REG_MUXSEL0, chan_num);
	if (ret)
		goto adc_cleanup;

	/* select one-shot mode _before_ enabling ADC */
	ret = s2mpg01_write_byte(ddata, S2MPG01_REG_ADC_CTRL2,
				 one_shot_mode ? 0x10 : 0x00);
	if (ret)
		goto adc_cleanup;

	/* enable the clock at 125 kHz, 1 channel, and 8 samples */
	ret = s2mpg01_write_byte(ddata, S2MPG01_REG_ADC_CTRL, 0xC3);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: started ADC conversion\n", __func__);

	/* wait for completion signaled by interrupt */
	timeout = wait_for_completion_timeout(&ddata->adc_conv_complete,
					      S2MPG01_ADC_CONV_TIMEOUT);
	if (!timeout) {
		ret = -ETIMEDOUT;
		dev_err(ddata->dev, "%s: ADC conversion timeout\n", __func__);
		goto adc_cleanup;
	}

	/* read and format the conversion result */
	ret = s2mpg01_read_byte(ddata, S2MPG01_REG_ADC0DATA, chan_data);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: chan_data 0x%02x\n", __func__, *chan_data);

adc_cleanup:
	/* disable the ADC clock in continuous mode */
	if (!one_shot_mode)
		s2mpg01_write_byte(ddata, S2MPG01_REG_ADC_CTRL, 0x43);

	/* release adc conversion */
	clear_bit(0, &ddata->adc_conv_busy);

	return ret;
}

int s2mpg01_read_adc_chan(struct s2mpg01_core *ddata,
			  int chan_num, u8 *chan_data)
{
	return __s2mpg01_read_adc_chan(ddata, chan_num, chan_data, true);
}
EXPORT_SYMBOL_GPL(s2mpg01_read_adc_chan);

bool s2mpg01_boost_mode_status(struct s2mpg01_core *ddata)
{
	u8 reg_data;
	int offset = 7; /* bit[7]: SYNC_L3_EN*/

	s2mpg01_read_byte(ddata, S2MPG01_REG_BOOST_CTRL, &reg_data);
	return !!(reg_data & (1 << offset));
}

static int __s2mpg01_set_boost_mode(struct s2mpg01_core *ddata, bool enable)
{
	int offset = 7; /* bit[7]: SYNC_L3_EN*/

	return s2mpg01_update_bits(ddata,
				   S2MPG01_REG_BOOST_CTRL,
				   (1 << offset),
				   ((enable ? 1 : 0) << offset));
}

int s2mpg01_enable_boost(struct s2mpg01_core *ddata)
{
	int ret;

	dev_dbg(ddata->dev, "%s: Enabling boost mode\n", __func__);
	s2mpg01_unmask_boost_mode_interrupts(ddata);
	ret = __s2mpg01_set_boost_mode(ddata, true);

	return ret;
}

int s2mpg01_disable_boost(struct s2mpg01_core *ddata)
{
	int ret;

	dev_dbg(ddata->dev, "%s: Disabling boost mode\n", __func__);
	ret = __s2mpg01_set_boost_mode(ddata, false);
	s2mpg01_mask_boost_mode_interrupts(ddata);

	return ret;
}

#define NOTIFY(id, event) s2mpg01_regulator_notify(id, event)

/* print the device id */
static int s2mpg01_print_id(struct s2mpg01_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 reg_data;
	int ret;

	ret = s2mpg01_read_byte(ddata, S2MPG01_REG_CHIP_ID, &reg_data);
	if (!ret) {
		dev_info(dev, "ID: 0x%02X\n", reg_data);
		ddata->rev_id = reg_data;
	} else {
		dev_err(dev, "%s: Could not read PMIC ID (%d)\n", __func__,
			ret);
		return ret;
	}
	return 0;
}

/* print the status register */
static void s2mpg01_print_status(struct s2mpg01_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 status[3];
	int ret;

	ret = s2mpg01_read_bytes(ddata, S2MPG01_REG_STATUS1, status, 3);
	if (!ret)
		dev_info(dev, "%s: Status: 0x%02x, 0x%02x, 0x%02x\n", __func__,
			 status[0], status[1], status[2]);
}

static void s2mpg01_notify_fail_all(void)
{
	/*
	 * Notify regulator clients of failures.
	 * Use the same sequence as ABH power down.
	 */
	NOTIFY(S2MPG01_ID_LDO2, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_LDO3, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_SMPS1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_LDO5, REGULATOR_EVENT_FAIL);

	NOTIFY(S2MPG01_ID_LDO4, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_SMPS3, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_LDO1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_SMPS2, REGULATOR_EVENT_FAIL);
}

/* handle an interrupt flag */
static int s2mpg01_handle_int(struct s2mpg01_core *ddata,
			      unsigned int flag_num)
{
	struct device *dev = ddata->dev;

	dev_dbg(dev, "%s: flag %d\n", __func__, flag_num);

	switch (flag_num) {
	case S2MPG01_INT_PONR:
		dev_info(dev, "%s: Observed PON rising edge\n", __func__);
		break;
	case S2MPG01_INT_PONF:
		dev_warn(dev, "%s: Observed PON falling edge\n", __func__);
		break;
	case S2MPG01_INT_ADC_CH0:
		dev_dbg(dev, "%s: ADC channel0 over threshold\n", __func__);
		break;
	case S2MPG01_INT_ADC_CH1:
		dev_dbg(dev, "%s: ADC channel1 over threshold1\n", __func__);
		break;
	case S2MPG01_INT_ADC_CH2:
		dev_dbg(dev, "%s: ADC channel2 over threshold\n", __func__);
		break;
	case S2MPG01_INT_ADC_CH3:
		dev_dbg(dev, "%s: ADC channel3 over threshold\n", __func__);
		break;
	case S2MPG01_INT_ADC_DONE:
		dev_dbg(dev, "%s: completing adc conversion\n", __func__);
		complete(&ddata->adc_conv_complete);
		break;
	case S2MPG01_INT_WATCHDOG:
		dev_err(dev, "%s: Watchdog timer expired\n", __func__);
		break;
	case S2MPG01_INT_T_ALARM:
		dev_warn(dev, "%s: Watchdog alarm event\n", __func__);
		break;
	case S2MPG01_INT_NORM:
		dev_err(dev, "%s: Normal mode setting violation\n", __func__);
		break;
	case S2MPG01_INT_BOOST:
		dev_err(dev, "%s: Boost mode setting violation\n", __func__);
		break;
	case S2MPG01_INT_SMPS1_UV:
		dev_err(dev, "Detected SMPS1 under-voltage event\n");
		NOTIFY(S2MPG01_ID_SMPS1,
		       REGULATOR_EVENT_UNDER_VOLTAGE | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_TINT_OUT:
		dev_warn(dev, "%s: Thermal warm event\n", __func__);
		break;
	case S2MPG01_INT_TSD:
		dev_err(dev, "%s: Thermal shutdown\n", __func__);
		break;
	case S2MPG01_INT_TH_TRIPR:
		dev_err(dev, "%s: therm_trip asserted\n", __func__);
		break;
	case S2MPG01_INT_TH_TRIPF:
		dev_info(dev, "%s: therm_trip has returned to normal\n",
			 __func__);
		break;
	case S2MPG01_INT_LDO1_OI:
		dev_err(dev, "Detected LDO1 over-current event\n");
		NOTIFY(S2MPG01_ID_LDO1,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_LDO2_OI:
		dev_err(dev, "Detected LDO2 over-current event\n");
		NOTIFY(S2MPG01_ID_LDO2,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_LDO3_OI:
		dev_err(dev, "Detected LDO3 over-current event\n");
		NOTIFY(S2MPG01_ID_LDO3,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_LDO4_OI:
		dev_err(dev, "Detected LDO4 over-current event\n");
		NOTIFY(S2MPG01_ID_LDO4,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_LDO5_OI:
		dev_err(dev, "Detected LDO5 over-current event\n");
		NOTIFY(S2MPG01_ID_LDO5,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_SMPS3_OI:
		dev_err(dev, "Detected SMPS3 over-current event\n");
		NOTIFY(S2MPG01_ID_SMPS3,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPG01_ID_LDO3, REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPG01_ID_LDO4, REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_SMPS2_OI:
		dev_err(dev, "Detected SMPS2 over-current event\n");
		NOTIFY(S2MPG01_ID_SMPS2,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPG01_ID_LDO2, REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_SMPS1_OI:
		dev_err(dev, "Detected SMPS1 over-current event\n");
		NOTIFY(S2MPG01_ID_SMPS1,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;
	case S2MPG01_INT_LDO3_DVS_END:
		dev_info(dev, "%s: LDO3 boost mode DVS done\n", __func__);
		break;
	case S2MPG01_INT_SMPS1_DVS_END:
		dev_info(dev, "%s: SMPS1 boost mode DVS done\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s: Reserved flag %d\n", __func__, flag_num);
		break;
	}

	return 0;
}

/* find pending interrupt flags */
static int s2mpg01_check_int_flags(struct s2mpg01_core *ddata)
{
	u8 flags[S2MPG01_NUM_IRQ_REGS], flag_mask;
	unsigned int first_bit, flag_num;
	int ret;
	int i;

	/* read interrupt status flags */
	ret = s2mpg01_read_bytes(ddata, S2MPG01_REG_INT1,
				 flags, S2MPG01_NUM_IRQ_REGS);
	if (ret) {
		dev_err(ddata->dev,
			"%s: failed to read interrupt registers: %d\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(ddata->dev,
		"%s: [0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x, [3] = 0x%02x\n",
		__func__, flags[0], flags[1], flags[2], flags[3]);

	/* iterate through each interrupt */
	for (i = 0; i < S2MPG01_NUM_IRQ_REGS; i++) {
		while (flags[i]) {
			/* find first set interrupt flag */
			first_bit = ffs(flags[i]);
			flag_mask = 1 << (first_bit - 1);
			flag_num = (i * 8) + (first_bit - 1);

			/* handle interrupt */
			ret = s2mpg01_handle_int(ddata, flag_num);

			flags[i] &= ~flag_mask;
		}
	}

	return ret;
}

/* delayed work to poll for ready after reset
 */
static void s2mpg01_poll_ready_work(struct work_struct *data)
{
	static int poll_count;
	int poll_interval_ms;
	struct s2mpg01_core *ddata = container_of(data, struct s2mpg01_core,
						   poll_ready_work.work);

	poll_count++;

	if (!gpio_get_value(ddata->pdata->pmic_ready_gpio)) {
		/*
		 * Use shorter interval at the beginning, and increase
		 * the interval as the retry count increases.
		 */
		poll_interval_ms = (poll_count < 10) ? 100 : poll_count * 200;
		schedule_delayed_work(&ddata->poll_ready_work,
				      msecs_to_jiffies(poll_interval_ms));
		return;
	}

	dev_info(ddata->dev,
		 "%s: completing reset after %d attempts\n",
		 __func__, poll_count);
	poll_count = 0;

	/* read chip status */
	s2mpg01_print_status(ddata);
	s2mpg01_core_fixup(ddata);
	s2mpg01_unmask_interrupts_all(ddata);
	/*
	 * Voltage changes at normal mode could also trigger
	 * boost mode interrupts. Mask the 2 boost mode interrupts
	 * in normal mode to avoid confusion.
	 */
	s2mpg01_mask_boost_mode_interrupts(ddata);
	complete(&ddata->init_complete);
}

/* kernel thread to notify regulator fail event and clear any pending
 * interrupt status
 */
static void s2mpg01_reset_work(struct work_struct *data)
{
	struct s2mpg01_core *ddata = container_of(data, struct s2mpg01_core,
						   reset_work);

	/* cancel existing polling work if any */
	cancel_delayed_work_sync(&ddata->poll_ready_work);

	/* notify regulators of shutdown event */
	dev_err(ddata->dev,
		"%s: Notifying regulators of shutdown event\n", __func__);
	s2mpg01_notify_fail_all();

	schedule_delayed_work(&ddata->poll_ready_work, 0);
}

/* irq handler for resetb falling edge only
 */
static irqreturn_t s2mpg01_resetb_irq_handler(int irq, void *cookie)
{
	struct s2mpg01_core *ddata = (struct s2mpg01_core *)cookie;

	dev_dbg(ddata->dev, "%s: observed resetb, irq = %d\n", __func__, irq);

	dev_err(ddata->dev,
		"%s: device reset, scheduling reset work\n",
		__func__);
	reinit_completion(&ddata->init_complete);
	schedule_work(&ddata->reset_work);

	return IRQ_HANDLED;
}

/* threaded irq handler for intb pin */
static irqreturn_t s2mpg01_intb_irq_handler(int irq, void *cookie)
{
	struct s2mpg01_core *ddata = (struct s2mpg01_core *)cookie;

	dev_dbg(ddata->dev, "%s: observed irq %d\n", __func__, irq);

	s2mpg01_check_int_flags(ddata);

	dev_dbg(ddata->dev, "%s: handled irq %d\n", __func__, irq);

	return IRQ_HANDLED;
}

/* some changes to register reset values */
static int s2mpg01_core_fixup(struct s2mpg01_core *ddata)
{
	/* set SMPS1 boost voltage to 0.85V; reset value is 0.75V */
	s2mpg01_write_byte(ddata, S2MPG01_REG_BUCK1_OUT_DVS, 0x58);

	/* set boost mode threshold to 0.5V; reset value is max */
	s2mpg01_write_byte(ddata, S2MPG01_REG_BOOST_THRES, 0x28);

	return 0;
}

/* get platform data from the device tree */
static struct s2mpg01_platform_data *s2mpg01_get_platform_data_from_dt
	(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct s2mpg01_platform_data *pdata;

	if (!np)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->pon_gpio = of_get_named_gpio(np, "samsung,pon-gpio", 0);
	pdata->pmic_ready_gpio = of_get_named_gpio(np,
						   "samsung,pmic-ready-gpio",
						   0);
	pdata->intb_gpio = of_get_named_gpio(np, "samsung,intb-gpio", 0);
	pdata->resetb_irq = gpio_to_irq(pdata->pmic_ready_gpio);
	pdata->intb_irq = gpio_to_irq(pdata->intb_gpio);

	dev_dbg(dev, "%s: pon_gpio: %d\n", __func__, pdata->pon_gpio);
	dev_dbg(dev, "%s: pmic_ready_gpio: %d\n", __func__,
		pdata->pmic_ready_gpio);
	dev_dbg(dev, "%s: intb_gpio: %d\n", __func__, pdata->intb_gpio);
	dev_dbg(dev, "%s: resetb_irq: %u\n", __func__, pdata->resetb_irq);
	dev_dbg(dev, "%s: intb_irq: %u\n", __func__, pdata->intb_irq);

	return pdata;
}

static void s2mpg01_clear_interrupts(struct s2mpg01_core *ddata)
{
	u8 bytes[S2MPG01_NUM_IRQ_REGS];

	/* interrupt registers are read-clear */
	s2mpg01_read_bytes(ddata, S2MPG01_REG_INT1,
			   bytes, S2MPG01_NUM_IRQ_REGS);
}

__maybe_unused
static void s2mpg01_mask_interrupts_all(struct s2mpg01_core *ddata)
{
	u8 bytes[S2MPG01_NUM_IRQ_REGS] = { 0xFF };

	/* mask all interrupts */
	s2mpg01_write_bytes(ddata, S2MPG01_REG_INT1M,
			    bytes, S2MPG01_NUM_IRQ_REGS);
}

static void s2mpg01_unmask_interrupts_all(struct s2mpg01_core *ddata)
{
	/* unmask all (non-reserved) interrupts */
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT1M, 0x00);
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT2M, 0x00);
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT3M, 0x00);
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT4M, S2MPG01_INT_4_RSVD_BITS);
}

static void s2mpg01_mask_boost_mode_interrupts(struct s2mpg01_core *ddata)
{
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT4M, 0xFF);
}

static void s2mpg01_unmask_boost_mode_interrupts(struct s2mpg01_core *ddata)
{
	/* unmask INT4M[7:6] except for the reserved bits */
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT4M, S2MPG01_INT_4_RSVD_BITS);
}

static int s2mpg01_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct s2mpg01_core *ddata;
	struct s2mpg01_platform_data *pdata;
	unsigned long timeout;
	int ret;
	int i;

	/* allocate memory for chip structure */
	ddata = devm_kzalloc(dev, sizeof(struct s2mpg01_core),
			     GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	/* set client data */
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);

	/* get platform data */
	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = s2mpg01_get_platform_data_from_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* initialize chip structure */
	ddata->dev = dev;
	ddata->pdata = pdata;
	dev->platform_data = pdata;

	/* initialize some structures */
	INIT_WORK(&ddata->reset_work, s2mpg01_reset_work);
	INIT_DELAYED_WORK(&ddata->poll_ready_work, s2mpg01_poll_ready_work);

	/* initialize completions */
	init_completion(&ddata->init_complete);
	init_completion(&ddata->adc_conv_complete);

	/* initialize regmap */
	ddata->regmap = devm_regmap_init_i2c(client, &s2mpg01_regmap_config);
	if (IS_ERR(ddata->regmap)) {
		ret = PTR_ERR(ddata->regmap);
		dev_err(dev,
			"%s: could not initialize regmap (%d)\n",
			__func__, ret);
		return -ENOMEM;
	}

	/* request GPIOs and IRQs */
	ret = devm_gpio_request_one(dev, pdata->pon_gpio, GPIOF_OUT_INIT_LOW,
				    "S2MPG01 PON");
	if (ret) {
		dev_err(dev,
			"%s: failed to request pon_gpio %u: %d\n",
			__func__, pdata->pon_gpio, ret);
		goto error_reset;
	}

	ret = devm_gpio_request_one(dev, pdata->pmic_ready_gpio, GPIOF_IN,
				    "S2MPG01 PMIC READY");
	if (ret) {
		dev_err(dev,
			"%s: failed to request pmic_ready_gpio %u: %d\n",
			__func__, pdata->pmic_ready_gpio, ret);
		goto error_reset;
	}

	ret = devm_gpio_request_one(dev, pdata->intb_gpio, GPIOF_IN,
				    "S2MPG01 INTB");
	if (ret) {
		dev_err(dev,
			"%s: failed to request intb_gpio %u: %d\n",
			__func__, pdata->intb_gpio, ret);
		goto error_reset;
	}

	for (i = 0; i < S2MPG01_PON_RETRY_CNT; i++) {
		dev_dbg(dev, "%s: powering on s2mpg01\n", __func__);
		gpio_set_value_cansleep(pdata->pon_gpio, 1);

		/* give the chip some time to power on */
		msleep(S2MPG01_PON_RESET_DELAY);

		/* poll on the gpio until it goes high or until we timeout */
		timeout = jiffies + S2MPG01_PON_RESET_TIMEOUT;
		while (!gpio_get_value(pdata->pmic_ready_gpio) &&
		       time_before(jiffies, timeout)) {
			usleep_range(100, 105);
		}

		if (gpio_get_value(pdata->pmic_ready_gpio))
			break;

		dev_err(dev, "%s: powering on timed out, try (%d/%d)\n",
			__func__, i + 1, S2MPG01_PON_RETRY_CNT);

		gpio_set_value_cansleep(pdata->pon_gpio, 0);
		usleep_range(100, 105);
	}

	/* check for failure */
	if (i == S2MPG01_PON_RETRY_CNT) {
		ret = -ETIMEDOUT;
		dev_err(dev, "%s: powering on failed\n", __func__);
		goto error_reset;
	}

	if (s2mpg01_print_id(ddata)) {
		ret = -ENODEV;
		dev_err(dev, "%s: couldn't communicate over I2C, giving up",
			__func__);
		goto error_reset;
	}

	/* create sysfs attributes */
	s2mpg01_config_sysfs(dev);

	/* print chip status */
	s2mpg01_print_status(ddata);

	/* apply fixup */
	s2mpg01_core_fixup(ddata);

	/* enable all interrupts */
	s2mpg01_clear_interrupts(ddata);
	s2mpg01_unmask_interrupts_all(ddata);

	/*
	 * Voltage changes at normal mode could also trigger
	 * boost mode interrupts. Mask the 2 boost mode interrupts
	 * in normal mode to avoid confusion.
	 */
	s2mpg01_mask_boost_mode_interrupts(ddata);

	ret = devm_request_threaded_irq(dev, pdata->intb_irq, NULL,
					s2mpg01_intb_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"s2mpg01-intb", ddata);
	if (ret) {
		dev_err(dev,
			"%s: failed to request irq %u: %d\n",
			__func__, pdata->intb_irq, ret);
		goto error_reset;
	}

	ret = devm_request_threaded_irq(dev, pdata->resetb_irq, NULL,
					s2mpg01_resetb_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"s2mpg01-resetb", ddata);
	if (ret) {
		dev_err(dev,
			"%s: failed to request irq %u: %d\n",
			__func__, pdata->resetb_irq, ret);
		goto error_reset;
	}

	return mfd_add_devices(dev, -1, s2mpg01_devs, ARRAY_SIZE(s2mpg01_devs),
			       NULL, 0, NULL);

error_reset:
	return ret;
}

#ifdef CONFIG_PM
static int s2mpg01_suspend(struct device *dev)
{
	struct s2mpg01_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		enable_irq_wake(pdata->intb_irq);
	return 0;
}

static int s2mpg01_resume(struct device *dev)
{
	struct s2mpg01_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		disable_irq_wake(pdata->intb_irq);
	return 0;
}

static const struct dev_pm_ops s2mpg01_dev_pm_ops = {
	.suspend = s2mpg01_suspend,
	.resume  = s2mpg01_resume,
};
#endif

static const struct of_device_id s2mpg01_dt_ids[] = {
	{ .compatible = "samsung,s2mpg01", },
	{ },
};
MODULE_DEVICE_TABLE(of, s2mpg01_dt_ids);

static const struct i2c_device_id s2mpg01_id_table[] = {
	{ .name = DRIVER_NAME, .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mpg01_id_table);

static struct i2c_driver s2mpg01_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(s2mpg01_dt_ids),
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &s2mpg01_dev_pm_ops,
#endif
	},
	.probe = s2mpg01_probe,
	.id_table = s2mpg01_id_table,
};

module_i2c_driver(s2mpg01_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPG01 Device Driver");
MODULE_LICENSE("GPL");
