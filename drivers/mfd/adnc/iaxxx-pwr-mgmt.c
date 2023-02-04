/*
 * iaxxx-sysfs.c -- IAxxx Sysfs attributes
 *
 * Copyright 2018 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-plugin-instance-header.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include <linux/mfd/adnc/iaxxx-plugin-common.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include "iaxxx-btp.h"
#include "iaxxx.h"

#define IAXXX_PROC_STATUS_MASK	0x34

#define IAXXX_PWR_DWN_VAL 0x01C00050
#define IAXXX_PWR_ON_VAL 0x845
#define IAXXX_PWR_STATE_RETRY	0x5

#define IAXXX_PM_AUTOSUSPEND_DELAY 3000

#define IAXXX_PWR_MAX_SPI_SPEED   25000000
#define IAXXX_STRM_STATUS_INIT	 0

#define IAXXX_PROC_STATUS_WAIT_MIN		(1000)
#define IAXXX_PROC_STATUS_WAIT_MAX		(1005)

#define IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id) \
	(IAXXX_SRB_PROC_PWR_CTRL_PWR_ON_PROC_0_MASK << proc_id)

#define IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id) \
	(IAXXX_SRB_PROC_PWR_CTRL_STALL_PROC_0_MASK << proc_id)

#define IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_CTRL_MEM_PWR_ON_PROC_0_MASK << \
	proc_id)

#define IAXXX_MEM_RETN_ON_OFF_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_CTRL_MEM_RETN_PROC_0_MASK << \
	proc_id)

#define IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id) \
	(IAXXX_SRB_PROC_ACTIVE_STATUS_PWR_STATUS_PROC_0_MASK << \
	proc_id)

#define IAXXX_GET_PROC_PWR_STALL_STATUS_MASK(proc_id) \
	(IAXXX_SRB_PROC_ACTIVE_STATUS_STALL_STATUS_PROC_0_MASK << \
	proc_id)

#define IAXXX_GET_PER_PROC_MEM_PWR_STATUS_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_STATUS_PROC_0_MEM_PWR_MASK << \
	proc_id)

#define IAXXX_GET_PER_PROC_MEM_RETN_STATUS_MASK(proc_id) \
	(IAXXX_SRB_DED_MEM_PWR_STATUS_PROC_0_MEM_RETN_MASK << \
	proc_id)

void iaxxx_pm_enable(struct iaxxx_priv *priv)
{
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	int ret = 0;

	priv->in_suspend = 0;
	priv->in_resume = 0;
	ret = pm_runtime_set_active(priv->dev);
	if (ret < 0)
		dev_err(priv->dev, "pm_runtime_set_active fail %d\n", ret);

	pm_runtime_set_autosuspend_delay(priv->dev, IAXXX_PM_AUTOSUSPEND_DELAY);
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_use_autosuspend(priv->dev);
	pm_runtime_enable(priv->dev);
#endif
}

int iaxxx_pm_get_sync(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		if (ret == -EACCES) {
			dev_dbg(dev, "%s() runtime PM disabled\n", __func__);
			ret = 0;
		} else if (ret == -EIO || ret == -EINVAL) {
			/* When the chip crashes and we just triggered recovery
			 * in pm_runtime_get_sync() context, it is expected to
			 * get -EIO or -EINVAL here. Use pm_runtime_set_active
			 * to clear the state of runtime_error.
			 */
			if (test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
				pm_runtime_set_active(priv->dev);
		} else
			dev_err(dev, "%s() failed %d\n", __func__, ret);
	}
#endif
	return ret;
}

int iaxxx_pm_put_autosuspend(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	pm_runtime_mark_last_busy(dev);
	ret = pm_runtime_put_sync_autosuspend(dev);
	if (ret < 0) {
		if (ret == -EACCES) {
			dev_dbg(dev, "%s() runtime PM disabled\n", __func__);
			ret = 0;
		} else if (ret == -EBUSY)
			ret = 0;
		else
			dev_err(dev, "%s() failed %d\n", __func__, ret);
	}
#endif
	return ret;
}

int iaxxx_wakeup_chip(struct iaxxx_priv *priv)
{
	int rc, reg_val;
	uint32_t status;
	long wake_timeout = HZ;


	/* Enable external clock */
	if (priv->iaxxx_state->power_state == IAXXX_SLEEP_MODE) {
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 1);
	}

	/* If the chip is already woken up, skip wakeup*/

	if (!test_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
						&priv->flags)) {
		rc = regmap_read(priv->regmap_no_pm,
				IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);

		if (priv->iaxxx_state->power_state == IAXXX_SLEEP_MODE)
			wake_timeout = 100 * HZ / 1000;

		rc = wait_event_timeout(priv->wakeup_wq,
				test_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
					&priv->flags), wake_timeout);
		if (!test_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
					&priv->flags) && rc == 0)
			dev_err(priv->dev,
				"Timeout for wakeup event rc :%d wake flag :%d",
				rc, test_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
				&priv->flags));
		else
			goto chip_woken_up;

		rc = regmap_read(priv->regmap_no_pm,
				IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);

		/* if read failed or SYS mode is not in APP mode,
		 * flag error
		 */
		reg_val &= IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(priv->dev,
			"%s chip wake up reg_val%d\n", __func__,
			reg_val);
		if (!rc && (reg_val == SYSTEM_STATUS_MODE_APPS)) {
			test_and_set_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
						&priv->flags);
			goto chip_woken_up;
		}
		dev_err(priv->dev,
			"%s chip wake up failed %d rc %d\n",
			__func__, reg_val, rc);
		goto chip_recovery;
	}
chip_woken_up:
	if (priv->iaxxx_state->power_state == IAXXX_SLEEP_MODE) {
		s64 ts_diff = ktime_ms_delta(ktime_get_boottime(),
					     priv->iaxxx_state->ktime_sleep);
		if (ts_diff > U32_MAX)
			ts_diff = U32_MAX;
		else if (ts_diff < 0)
			ts_diff = 0;

		/* switch to internal oscillator if dt entry
		 * is selected for internal oscillator mode.
		 */
		if (priv->oscillator_mode) {
			/* Switch to internal oscillator */
			rc = iaxxx_set_mpll_source_no_pm(priv, IAXXX_INT_OSC);
			if (rc) {
				dev_err(priv->dev,
				"%s() Failed to set MPLL Clk Src to internal\n",
								__func__);
				goto chip_recovery;
			}
		}

		/* program SRB */
		rc = regmap_write(priv->regmap_no_pm,
				  IAXXX_SRB_SYSTEM_SLEEP_DURATION_ADDR,
				  ts_diff);
		if (!rc) {
			/* Update block lock is not taken for no_pm calls
			 * because those can trigger PM wakeup which will
			 * try to do some fw-setup which will need the
			 * update block.
			 */
			rc = iaxxx_send_update_block_request_with_options(
					priv->dev, IAXXX_BLOCK_0,
					IAXXX_HOST_0, priv->regmap_no_pm,
					10,
					UPDATE_BLOCK_FIXED_WAIT_OPTION |
					UPDATE_BLOCK_NO_LOCK_OPTION,
					&status);
		} else
			goto chip_recovery;

		if (rc)
			dev_err(priv->dev, "%s failed to program sleep time\n",
				__func__);

		priv->iaxxx_state->power_state = IAXXX_NORMAL_MODE;
		dev_info(priv->dev, "%s set to normal power mode done\n",
			__func__);
	}

	dev_info(priv->dev, "%s() Success\n", __func__);
	return 0;

chip_recovery:
	iaxxx_fw_crash(priv->dev, IAXXX_FW_CRASH_RESUME, IAXXX_NO_PROC);
	dev_err(priv->dev, "%s() fail\n", __func__);
	return -EIO;
}

static int iaxxx_check_and_powerdown_cores(struct iaxxx_priv *priv,
		uint32_t *proc_status)
{
	int rc;
	uint32_t status, proc_active, core_status;

	/* Read core processor status */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR,
			&proc_active);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read proc status %s\n", __func__);
		return rc;
	}
	/* Check if SSP, DMX or HMD processor is active */
	proc_active = proc_active & IAXXX_PROC_STATUS_MASK;
	if (proc_active)
		dev_info(priv->dev, "Proc status 0x%x\n", proc_active);
	else {
		*proc_status = 0;
		return 0;
	}

	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_STR_HDR_STR_ST_ADDR,
			&status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read stream status %s\n", __func__);
		return rc;
	}

	/*Check if SSP need to be powered down*/
	if (!status && (proc_active &
			IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_SSP_ID))) {
		rc = iaxxx_power_down_core_mem(priv, IAXXX_SSP_ID);
		if (rc) {
			dev_err(priv->dev,
				"Failed to power down SSP in %s\n", __func__);
			return rc;
		}
		proc_active &= ~(IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_SSP_ID));
	}

	/* Read Plugin crate status for HMD */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
			IAXXX_BLOCK_1, IAXXX_HOST_0),
			&status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read HMD plugin status %s\n", __func__);
		return rc;
	}

	core_status = status;
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
			IAXXX_BLOCK_1, IAXXX_HOST_1),
			&status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read HMD plugin status %s\n", __func__);
		return rc;
	}

	core_status |= status;

	/*Check if HMD need to be powered down*/
	if (!core_status && (proc_active &
			IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_HMD_ID))) {
		rc = iaxxx_power_down_core_mem(priv, IAXXX_HMD_ID);
		if (rc) {
			dev_err(priv->dev,
				"Failed to power down HMD in %s\n", __func__);
			return rc;
		}
		proc_active &= ~(IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_HMD_ID));
	}

	/* Read Plgin crate status for DMX */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
			IAXXX_BLOCK_2, IAXXX_HOST_0),
			&status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read DMX plugin status %s\n", __func__);
		return rc;
	}

	core_status = status;
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
			IAXXX_BLOCK_2, IAXXX_HOST_1),
			&status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read DMX plugin status %s\n", __func__);
		return rc;
	}

	core_status |= status;
	/*Check if DMX  need to be powered down*/
	if (!core_status && (proc_active &
			IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_DMX_ID))) {
		rc = iaxxx_power_down_core_mem(priv, IAXXX_DMX_ID);
		if (rc) {
			dev_err(priv->dev,
				"Failed to power down DMX in %s\n", __func__);
			return rc;
		}
		proc_active &= ~(IAXXX_GET_PROC_PWR_STATUS_MASK(IAXXX_DMX_ID));
	}

	*proc_status = proc_active;
	return rc;

}

int iaxxx_suspend_chip(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t status;
	uint32_t proc_status = 0;

	/* set up the SPI speed thats expected when the system is wake up
	 * Set the SPI Speed to maximum so system will be awake with max
	 * clock speed.
	 */
	rc = regmap_write(priv->regmap_no_pm,
			IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR,
			IAXXX_PWR_MAX_SPI_SPEED);

	if (rc) {
		dev_err(priv->dev,
			"Failed to set Max SPI speed in %s\n", __func__);
		goto chip_recovery;
	}

	rc = iaxxx_check_and_powerdown_cores(priv, &proc_status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to do power check on cores in %s\n", __func__);
		goto chip_recovery;
	}

	/* Note: Control interface of second host should also be disabled
	 * to achieve optimal or sleep mode.
	 */
	/*
	 * SLEEP MODE: If plugin count is zero and route is inactive and
	 * Processor status for SSP, HMD and DMX is inactive
	 */
	if (iaxxx_core_plg_list_empty(priv) &&
			!iaxxx_core_get_route_status(priv) &&
			!proc_status) {
		/* Enable external clock */
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 1);

		/* switch to external oscillator if dt entry
		 * is selected for internal oscillator mode.
		 * so that on wakeup we switch back to internal
		 * oscillator mode.
		 */
		if (priv->oscillator_mode) {
			/* Switch to external oscillator */
			rc = iaxxx_set_mpll_source_no_pm(priv, IAXXX_EXT_OSC);
			if (rc) {
				dev_err(priv->dev,
				"%s() Failed to set MPLL Clk Src to external\n",
								__func__);
				goto chip_recovery;
			}
		}

		/* Issue sleep power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK,
				(IAXXX_SLEEP_MODE <<
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS));

		if (rc) {
			dev_err(priv->dev, "%s() Fail\n", __func__);
			goto chip_recovery;
		}

		/* Update block lock is not taken for no_pm calls
		 * because those can trigger PM wakeup which will
		 * try to do some fw-setup which will need the
		 * update block.
		 */
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				IAXXX_HOST_0, priv->regmap_no_pm,
				20,
				UPDATE_BLOCK_FIXED_WAIT_OPTION |
				UPDATE_BLOCK_NO_LOCK_OPTION,
				&status);

		priv->iaxxx_state->ktime_sleep = ktime_get_boottime();

		/* Disable external clock */
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 0);

		priv->iaxxx_state->power_state = IAXXX_SLEEP_MODE;
		dev_info(priv->dev, "%s() chip put into sleep power mode\n",
								__func__);
	} else {
	/* OPTIMAL MODE: If plugin count is not zero or route is still active */

		/* Issue optimal power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			(IAXXX_OPTIMAL_MODE <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS));

		if (rc) {
			dev_err(priv->dev, "%s() Fail\n", __func__);
			goto chip_recovery;
		}

		/* Update block lock is not taken for no_pm calls
		 * because those can trigger PM wakeup which will
		 * try to do some fw-setup which will need the
		 * update block.
		 */
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				IAXXX_HOST_0, priv->regmap_no_pm,
				20,
				UPDATE_BLOCK_FIXED_WAIT_OPTION |
				UPDATE_BLOCK_NO_LOCK_OPTION,
				&status);

		priv->iaxxx_state->power_state = IAXXX_OPTIMAL_MODE;
		dev_info(priv->dev, "%s() chip put into optimal power mode\n",
								__func__);
	}
	test_and_clear_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
						&priv->flags);
	dev_info(priv->dev, "%s() Success\n", __func__);

	return 0;

chip_recovery:
	iaxxx_fw_crash(priv->dev, IAXXX_FW_CRASH_SUSPEND, IAXXX_NO_PROC);
	dev_err(priv->dev, "%s() fail\n", __func__);
	return -EIO;
}

/*
 * iaxxx_pm_set_aclk, input value as follows.
 * 0 - 3.072 MHz
 * 1 - 6.144 MHz
 * 2 - 12.288 MHz
 * 3 - 24.576 MHz
 * 4 - 49.152 MHz
 * 5 - 98.304 MHz
 * 6 - 368.640 MHz
 * otherwise - Invalid
 */
int iaxxx_pm_set_aclk(struct device *dev, int clk_freq)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc, status;

	dev_dbg(dev, "%s() freq:%d\n", __func__, clk_freq);

	rc = regmap_update_bits(priv->regmap,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_MASK,
			clk_freq <<
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_POS);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK);

	if (!rc)
		rc = iaxxx_send_update_block_request(dev, &status,
				IAXXX_BLOCK_0);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return rc;
}

/*
 * iaxxx_pm_set_optimal_power_mode (host0)
 * Need to do wake up to come out
 */
int iaxxx_pm_set_optimal_power_mode_host0(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t status;

	dev_dbg(dev, "%s()\n", __func__);

	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(priv->regmap, IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR,
			priv->spi_app_speed);

	rc = regmap_update_bits(priv->regmap, IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS);
	if (rc) {
		dev_err(dev, "%s() Fail\n", __func__);
		return rc;
	}

	rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap,
			20,
			UPDATE_BLOCK_FIXED_WAIT_OPTION,
			&status);
	priv->iaxxx_state->power_state = IAXXX_OPTIMAL_MODE;
	return rc;
}

/*
 * iaxxx_pm_set_optimal_power_mode (host1)
 * Need to do wake up to come out
 */
int iaxxx_pm_set_optimal_power_mode_host1(struct device *dev, bool no_pm)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t status;

	/* Choose the regmap based on which context this function is
	 * being executed. If it is executed from Power management
	 * context, then no_pm regmap should be forced so
	 * no further SPI wakeups are possible.
	 */
	struct regmap *regmap = no_pm ? priv->regmap_no_pm : priv->regmap;

	dev_dbg(dev, "%s()\n", __func__);

	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(regmap,
			IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_1_ADDR,
			priv->spi_app_speed);
	if (!rc)
		rc = regmap_update_bits(regmap,
			IAXXX_SRB_SYS_POWER_CTRL_1_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_POS);
	else {
		dev_err(dev, "%s() Fail\n", __func__);
		return rc;
	}

	if (no_pm) {
		/* Update block lock is not taken for no_pm calls
		 * because those can trigger PM wakeup which will
		 * try to do some fw-setup which will need the
		 * update block.
		 */
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev, IAXXX_BLOCK_0,
				IAXXX_HOST_1, regmap,
				20,
				UPDATE_BLOCK_FIXED_WAIT_OPTION |
				UPDATE_BLOCK_NO_LOCK_OPTION,
				&status);
	} else {
		rc = iaxxx_send_update_block_request_with_options(
				priv->dev,  IAXXX_BLOCK_0,
				IAXXX_HOST_1, regmap,
				20,
				UPDATE_BLOCK_FIXED_WAIT_OPTION,
				&status);
	}

	return rc;
}

int iaxxx_set_mpll_source(struct iaxxx_priv *priv, int source)
{
	int rc;
	uint32_t status;

	dev_dbg(priv->dev, "%s() source:%d\n", __func__, source);

	rc = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_MASK,
			source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_POS);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK);

	if (!rc) {
		rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap,
			20,
			UPDATE_BLOCK_FIXED_WAIT_OPTION |
			UPDATE_BLOCK_STATUS_CHECK_AFTER_FIXED_WAIT_OPTION,
			&status);
	}
	if (rc) {
		dev_err(priv->dev, "%s failed error code = %d\n", __func__, rc);
		return rc;
	}

	return rc;
}

int iaxxx_set_mpll_source_no_pm(struct iaxxx_priv *priv, int source)
{
	int rc;
	uint32_t status;

	dev_dbg(priv->dev, "%s() source:%d\n", __func__, source);

	if (source == IAXXX_EXT_OSC) {
		/* Disable control interface 1 */
		rc = iaxxx_pm_set_optimal_power_mode_host1(priv->dev, true);
		if (rc) {
			dev_err(priv->dev,
				"%s() disabling controle interface 1 Fail\n",
				__func__);
			return rc;
		}
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_MASK,
		(source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_POS));

	if (rc) {
		dev_err(priv->dev, "%s() Failed to set MPLL err = %d\n",
							__func__, rc);
		return rc;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
		IAXXX_SRB_SYS_POWER_CTRL_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK,
		(1 << IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_POS));

	if (rc) {
		dev_err(priv->dev, "%s() Fail err = %d\n", __func__, rc);
		return rc;
	}

	/* Update block lock is not taken for no_pm calls
	 * because those can trigger PM wakeup which will
	 * try to do some fw-setup which will need the
	 * update block.
	 */
	rc = iaxxx_send_update_block_request_with_options(
		priv->dev, IAXXX_BLOCK_0,
		IAXXX_HOST_0, priv->regmap_no_pm,
		20,
		UPDATE_BLOCK_FIXED_WAIT_OPTION |
		UPDATE_BLOCK_NO_LOCK_OPTION |
		UPDATE_BLOCK_STATUS_CHECK_AFTER_FIXED_WAIT_OPTION,
		&status);
	if (rc) {
		dev_err(priv->dev, "%s() Fail err = %d\n", __func__, rc);
		return rc;
	}

	return rc;
}

int iaxxx_set_apll_source(struct iaxxx_priv *priv, int source)
{
	int rc;
	u32 efuse_trim_value, status = 0;

	dev_dbg(priv->dev, "%s() source:%d\n", __func__, source);

	rc = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_MASK,
		source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_POS);

	if (!rc)
		rc = regmap_update_bits(priv->regmap,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK,
				IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK);

	regmap_read(priv->regmap, IAXXX_AO_EFUSE_BOOT_ADDR, &efuse_trim_value);
	efuse_trim_value = (efuse_trim_value >> 25) & 0x7F;

	if (efuse_trim_value) {
		rc = regmap_update_bits(priv->regmap, IAXXX_AO_OSC_CTRL_ADDR,
			IAXXX_AO_OSC_CTRL_ADJ_MASK, efuse_trim_value);
	}

	if (!rc)
		rc = iaxxx_send_update_block_request(priv->dev,
				&status, IAXXX_BLOCK_0);
	if (rc)
		dev_err(priv->dev, "%s failed to set up apll source err = %0x\n",
					__func__, rc);
	return rc;

}


int iaxxx_set_proc_pwr_ctrl(struct iaxxx_priv *priv,
			uint32_t proc_id, uint32_t proc_state,
			struct regmap *regmap)
{
	uint32_t proc_pwr_ctrl_val = 0;
	uint32_t proc_pwr_ctrl_mask = 0;
	int rc;

	/* Make sure update block bit is in cleared state */
	rc = iaxxx_poll_update_block_req_bit_clr(priv, regmap);
	if (rc) {
		dev_err(priv->dev,
			"%s() Don't do Update Block in progress, rc = %d\n",
			__func__, rc);
		goto exit;
	}

	switch (proc_state) {
	case PROC_PWR_DOWN:
		proc_pwr_ctrl_mask = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
		proc_pwr_ctrl_val = 0;
		break;
	case PROC_PWR_UP:
		proc_pwr_ctrl_mask = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
		proc_pwr_ctrl_val = IAXXX_PROC_POWER_UP_DOWN_MASK(proc_id);
		break;
	case PROC_STALL_ENABLE:
		proc_pwr_ctrl_mask =
				IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
		proc_pwr_ctrl_val =
				IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
		break;
	case PROC_STALL_DISABLE:
		proc_pwr_ctrl_mask =
				IAXXX_PROC_STALL_ENABLE_DISABLE_MASK(proc_id);
		proc_pwr_ctrl_val = 0;
		break;
	default:
		dev_err(priv->dev, "%s wrong proc state requested (%d)\n",
			__func__, proc_state);
		rc = -EINVAL;
		goto exit;
	}

	/* set the processor bits */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_PROC_PWR_CTRL_ADDR,
				proc_pwr_ctrl_mask, proc_pwr_ctrl_val);
	if (rc) {
		dev_err(priv->dev,
			"%s SRB_PROC_PWR_CTRL_ADDR write err = %0x\n",
			__func__, rc);
		goto exit;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK);
	if (rc) {
		dev_err(priv->dev,
			"%s SRB_SYS_POWER_CTRL_ADDR write err = %0x\n",
			__func__, rc);
		goto exit;
	}

exit:
	return rc;
}

int iaxxx_set_mem_pwr_ctrl(struct iaxxx_priv *priv,
			uint32_t proc_id, uint32_t mem_state,
			struct regmap *regmap)
{
	uint32_t mem_pwr_ctrl_val = 0;
	uint32_t mem_pwr_ctrl_mask = 0;
	int rc;

	dev_dbg(priv->dev, "%s() proc_id:%u mem_state:%u\n", __func__,
		proc_id, mem_state);

	/* Make sure update block bit is in cleared state */
	rc = iaxxx_poll_update_block_req_bit_clr(priv, regmap);
	if (rc) {
		dev_err(priv->dev,
			"%s() Don't do Update Block in progress, rc = %d\n",
			__func__, rc);
		goto exit;
	}

	switch (mem_state) {
	case MEM_PWR_DOWN:
		mem_pwr_ctrl_mask = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
		mem_pwr_ctrl_val = 0;
		break;
	case MEM_PWR_UP:
		mem_pwr_ctrl_mask = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
		mem_pwr_ctrl_val = IAXXX_MEM_POWER_UP_DOWN_MASK(proc_id);
		break;
	case MEM_RETN_ON:
		mem_pwr_ctrl_mask = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
		mem_pwr_ctrl_val = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
		break;
	case MEM_RETN_OFF:
		mem_pwr_ctrl_mask = IAXXX_MEM_RETN_ON_OFF_MASK(proc_id);
		mem_pwr_ctrl_val = 0;
		break;
	default:
		dev_err(priv->dev, "%s wrong mem_state state requested (%d)\n",
			__func__, mem_state);
		rc = -EINVAL;
		goto exit;
	}

	/* set the processor bits */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_DED_MEM_PWR_CTRL_ADDR,
				mem_pwr_ctrl_mask, mem_pwr_ctrl_val);
	if (rc) {
		dev_err(priv->dev, "%s SRB_DED_MEM_PWR_CTRL write err = %0x\n",
			__func__, rc);
		goto exit;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK,
				IAXXX_SRB_SYS_POWER_CTRL_SET_PROC_PWR_REQ_MASK);
	if (rc)
		dev_err(priv->dev,
			"%s SRB_SYS_POWER_CTRL_ADDR write err = %0x\n",
			__func__, rc);

exit:
	return rc;

}

static int iaxxx_get_proc_pwr_status(struct iaxxx_priv *priv,
	uint32_t proc_id, uint8_t *pwr_status, uint8_t *mem_status)
{
	uint32_t proc_pwr_status;
	uint32_t mem_pwr_status;
	int rc;

	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR,
		&proc_pwr_status);
	if (rc) {
		dev_err(priv->dev,
			"%s SRB_PROC_ACTIVE_STATUS_ADDR read err = %0x\n",
			__func__, rc);
		goto exit;
	}
	*pwr_status = proc_pwr_status &
			IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id);
	dev_info(priv->dev,
		"%s() SRB_PROC_ACTIVE_STATUS_ADDR: %u pwr_status: 0x%08x\n",
		__func__, proc_pwr_status, *pwr_status);

	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_DED_MEM_PWR_STATUS_ADDR,
		&mem_pwr_status);
	if (rc) {
		dev_err(priv->dev,
			"%s SRB_DED_MEM_PWR_STATUS_ADDR read err = %0x\n",
			__func__, rc);
		goto exit;
	}
	*mem_status = mem_pwr_status &
			IAXXX_GET_PER_PROC_MEM_PWR_STATUS_MASK(proc_id);
	dev_info(priv->dev,
		"%s() SRB_DED_MEM_PWR_STATUS_ADDR: %u mem_status: 0x%08x\n",
		__func__, mem_pwr_status, *mem_status);

exit:
	return rc;
}

static int check_proc_power_status(struct iaxxx_priv *priv,
		uint32_t proc_id)
{
	uint8_t pwr_status = 0;
	uint8_t mem_status = 0;
	int retry_count = 5;
	int rc;

	do {
		rc = iaxxx_get_proc_pwr_status(priv, proc_id,
				&pwr_status, &mem_status);
		if (!rc && pwr_status == 0)
			break;
		if (rc || retry_count <= 0) {
			dev_err(priv->dev, "%s timedout in processor power down\n",
				__func__);
			return -ETIMEDOUT;
		}
		usleep_range(IAXXX_PROC_STATUS_WAIT_MIN,
				IAXXX_PROC_STATUS_WAIT_MAX);
	} while (retry_count--);

	return 0;
}

int iaxxx_power_up_core_mem(
	struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t status;
	int rc = 0;

	dev_dbg(priv->dev, "%s() proc_id:%u\n", __func__, proc_id);

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_STALL_ENABLE,
			priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s proc power ctrl PROC_STALL_ENABLE failed = %0x\n",
					__func__, rc);
		return rc;
	}

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_PWR_UP, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s proc power ctrl PROC_PWR_UP failed = %0x\n",
					__func__, rc);
		return rc;
	}

	rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_PWR_UP, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s mem power ctrl MEM_PWR_UP failed = %0x\n",
					__func__, rc);
		return rc;
	}

	rc = iaxxx_send_update_block_request(priv->dev,
				&status, IAXXX_BLOCK_0);
	if (rc) {
		dev_err(priv->dev, "Update blk failed before download %s(): %d\n",
				__func__, status);
		return rc;
	}

	rc = iaxxx_boot_core(priv, proc_id);
	if (rc) {
		dev_err(priv->dev, "boot_core (%d) fail :%d\n", proc_id, rc);
		return rc;
	}

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id,
				PROC_STALL_DISABLE, priv->regmap);
	if (rc) {
		dev_err(priv->dev, "%s proc power ctrl PROC_STALL_DIS failed = %0x\n",
					__func__, rc);
		return rc;
	}

	rc = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
	if (rc) {
		dev_err(priv->dev, "Update blk failed after download %s(): %d\n",
				__func__, status);
		return rc;
	}

	dev_info(priv->dev,
		"proc(%u) %s() Success\n", proc_id, __func__);
	return rc;
}

int iaxxx_power_down_core_mem(
	struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t status;
	int rc = 0;

	dev_dbg(priv->dev, "%s() proc_id:%u\n", __func__, proc_id);

	mutex_lock(&priv->proc_on_off_lock);

	rc = iaxxx_set_proc_pwr_ctrl(priv, proc_id, PROC_PWR_DOWN,
			priv->regmap_no_pm);
	if (rc) {
		dev_err(priv->dev, "%s proc power ctrl PROC_PWR_DOWN failed = %0x\n",
					__func__, rc);
		goto exit;
	}

	if (proc_id != IAXXX_SSP_ID) {
		rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap_no_pm,
			0, UPDATE_BLOCK_NO_LOCK_OPTION, &status);
		if (rc) {
			dev_err(priv->dev,
			"Update blk failed after proc pwr down %s(): %d\n",
			__func__, status);
			goto exit;
		}

		rc = check_proc_power_status(priv, proc_id);
		if (rc) {
			dev_err(priv->dev, "%s check_proc_power_status failed = %0x\n",
						__func__, rc);
			goto exit;
		}
	}

	rc = iaxxx_set_mem_pwr_ctrl(priv, proc_id, MEM_PWR_DOWN,
			priv->regmap_no_pm);
	if (rc) {
		dev_err(priv->dev, "%s mem power ctrl MEM_PWR_DOWN failed = %0x\n",
					__func__, rc);
		goto exit;
	}

	rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap_no_pm,
			0, UPDATE_BLOCK_NO_LOCK_OPTION, &status);
	if (rc)
		dev_err(priv->dev, "Update blk failed after mem pwr down%s(): %d\n",
				__func__, status);

exit:
	mutex_unlock(&priv->proc_on_off_lock);
	return rc;
}

int iaxxx_check_and_powerup_core(struct iaxxx_priv *priv, uint32_t proc_id)
{
	uint32_t proc_status;
	int rc = 0;

	if ((proc_id != IAXXX_HMD_ID) && (proc_id != IAXXX_DMX_ID) &&
			(proc_id != IAXXX_SSP_ID)) {
		dev_err(priv->dev, "Invalid proc id in %s\n", __func__);
		return -EINVAL;
	}

	if (!iaxxx_is_firmware_ready(priv)) {
		rc = -EIO;
		return rc;
	}

	/* Core power up would happen only in the resume path w.r.t pm
	 * getting the sync ensure that resume callback is called without
	 * holding the proc_on_off_lock
	 */
	rc = iaxxx_pm_get_sync(priv->dev);
	if (rc < 0) {
		dev_err(priv->dev, "%s failed to get pm_sync rc= 0x%x\n",
			__func__, rc);
		goto get_sync_err;
	}

	mutex_lock(&priv->proc_on_off_lock);

	/* Read core processor status */
	rc = regmap_read(priv->regmap,
			IAXXX_SRB_PROC_ACTIVE_STATUS_ADDR,
			&proc_status);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read proc status %s\n", __func__);
		goto exit;
	}

	proc_status &= IAXXX_GET_PROC_PWR_STATUS_MASK(proc_id);

	if (!proc_status)
		rc = iaxxx_power_up_core_mem(priv, proc_id);

	if (rc)
		dev_err(priv->dev, "Failed to power up core %d in %s\n",
				proc_id, __func__);

exit:
	mutex_unlock(&priv->proc_on_off_lock);
get_sync_err:
	iaxxx_pm_put_autosuspend(priv->dev);
	return rc;
}

int iaxxx_check_and_power_up_ssp(struct iaxxx_priv *priv)
{
	uint32_t str_status;
	int ret = 0;

	ret = regmap_read(priv->regmap, IAXXX_STR_HDR_STR_ST_ADDR, &str_status);
	if (ret) {
		dev_err(priv->dev, "Failed to read stream  status in %s\n",
				__func__);
		return ret;
	}

	/*this means its the first stream to get enabled*/
	if (!str_status) {
		ret = iaxxx_check_and_powerup_core(priv, IAXXX_SSP_ID);
		if (ret)
			dev_err(priv->dev, " Error in powering up SSP %s %d\n",
					__func__, ret);
	}
	return ret;
}

/*
 * iaxxx_get_max_spi_speed
 * Gets the Max SPI Speed supported in the optimal power mode
 * max_spi_speed   Maximum SPI speed
 */
int iaxxx_get_max_spi_speed(struct device *dev, uint32_t *max_spi_speed)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;

	rc = regmap_read(priv->regmap,
			IAXXX_PWR_MGMT_MAX_SPI_SPEED_ADDR,
			max_spi_speed);
	if (rc) {
		dev_err(dev,
		"%s() Fail to read PWR_MGMT_MAX_SPI_SPEED_ADDR rc = %d\n",
								__func__, rc);
	}

	dev_dbg(priv->dev, "%s() speed:%u\n", __func__, *max_spi_speed);

	return rc;
}

/*****************************************************************************
 * iaxxx_core_get_pwr_stats()
 * @brief get power transition count for each xclk and aclk values.
 *
 * @pwr_stats power statistics for mpll and apll values
 *
 * @ret n number of words read, ret in case of error
 ****************************************************************************/
int iaxxx_core_get_pwr_stats(struct device *dev,
			struct iaxxx_pwr_stats *pwr_stats)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t pwr_stats_addr = 0;
	uint32_t pwr_stats_size = 0;
	int ret = 0;
	uint32_t status = 0;

	ret = regmap_update_bits(priv->regmap,
				IAXXX_PWR_MGMT_PWR_MGMT_STATS_CTRL_ADDR,
				IAXXX_PWR_MGMT_PWR_MGMT_STATS_CTRL_MASK_VAL,
				IAXXX_PWR_MGMT_PWR_MGMT_STATS_CTRL_REQ_MASK);
	if (ret) {
		dev_err(priv->dev, "Failed to set Power Stats Ctrl Reg\n");
		goto exit;
	}

	ret = iaxxx_send_update_block_request(priv->dev,
						&status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(priv->dev, "Update block failed after pwr ctrl\n");
		goto exit;
	}

	ret = regmap_read(priv->regmap,
		IAXXX_PWR_MGMT_PWR_MGMT_STATS_PTR_ADDR, &pwr_stats_addr);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		goto exit;
	}

	ret = regmap_read(priv->regmap,
		IAXXX_PWR_MGMT_PWR_MGMT_STATS_SIZE_ADDR, &pwr_stats_size);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		goto exit;
	}

	if (pwr_stats_size != (sizeof(struct iaxxx_pwr_stats) >> 2)) {
		dev_err(dev,
			"%s() pwr_stats_size %d != struct size %lu\n",
			__func__, pwr_stats_size,
			sizeof(struct iaxxx_pwr_stats) >> 2);
		ret = -EINVAL;
		goto exit;
	}

	ret = iaxxx_btp_read(priv,
		pwr_stats_addr, pwr_stats,
		pwr_stats_size, IAXXX_HOST_0);
	if (ret < 0) {
		dev_err(priv->dev, "Not able to read pwr stats %d\n",
				ret);
		goto exit;
	}

	dev_dbg(priv->dev, "read pwr stats successfully in words %u\n",
			pwr_stats_size);
exit:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_get_pwr_stats);

/* iaxxx_set_osc_trim_period
 * Non zero values would enable trimming and set the period in seconds.
 * Zero would disable the feature. Ensure that 32.56 Khz ext clock is been
 * provided
 */
int iaxxx_set_osc_trim_period(struct iaxxx_priv *priv, int period)
{
	int rc = 0;
	uint32_t status;

	if (period == priv->int_osc_trim_period)
		goto exit;
	dev_dbg(priv->dev, "%s() period:%d\n", __func__, period);

	rc = regmap_update_bits(priv->regmap,
		IAXXX_SRB_SYS_POWER_CTRL_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_TRIM_OSC_FREQ_MASK |
		IAXXX_SRB_SYS_POWER_CTRL_TRIM_OSC_PERIOD_MASK,
		IAXXX_SRB_SYS_POWER_CTRL_TRIM_OSC_FREQ_MASK |
		(period << IAXXX_SRB_SYS_POWER_CTRL_TRIM_OSC_PERIOD_POS));

	if (rc) {
		dev_err(priv->dev, "%s Failed to set Osc Trim period\n",
				__func__);
		goto exit;
	}

	rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap,
			20,
			UPDATE_BLOCK_FIXED_WAIT_OPTION,
			&status);

	if (rc)
		dev_err(priv->dev, "Update block failed in %s\n", __func__);

	priv->int_osc_trim_period = period;
exit:
	return rc;
}
