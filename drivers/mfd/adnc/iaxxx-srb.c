/*
 * iaxxx-srb.c -- IAxxx System Register Block Requests
 *
 * Copyright 2016 Knowles Corporation
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
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-plugin-instance-header.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "iaxxx.h"

#define IAXXX_MAX_CORES	3

/* The reserved bits in Update block register used to
 * set values to check if those bits are still set
 * even after request_bit is cleared.
 */
#define IAXXX_SRB_SYS_BLK_UPDATE_RESERVED_BITS_MASK 0xf0000

/* No of times to retry in case of update block register
 * does not return expected value.
 */
#define UPDATE_BLOCK_FAIL_RETRIES 200

#define IAXXX_UPDATE_BLOCK_WITH_FIXED_WAIT	(true)
#define IAXXX_UPDATE_BLOCK_WITH_NORMAL_WAIT	(false)

#define IAXXX_UPDATE_BLOCK_NO_PM      (true)
#define IAXXX_UPDATE_BLOCK_WITH_PM    (false)

/* 1ms delay before read */
#define IAXXX_UPDATE_BLK_READ_DELAY		1000
#define IAXXX_UPDATE_BLK_READ_DELAY_RANGE	5
/* 2ms delay before read retry */
#define IAXXX_UPDATE_BLK_READ_RETRY_DELAY	2000

/**
 * iaxxx_regmap_wait_match - waits for register read value
 * to match the given value. This is a dual test to check
 * if necessary bits are cleared and necessary bits bit
 * still set. This is to fix cases when invalid 0 value
 * read from spi would be treated as update block
 * success.
 *
 * @priv: iaxxx private data
 * @regmap: regmap to use
 * @reg : the register to be monitored
 * @match: the value to be matched
 *
 * Returns 0 on success or -ETIMEDOUT if the bits didn't clear.
 */
static int iaxxx_regmap_wait_match(struct iaxxx_priv *priv,
				struct regmap *regmap, uint32_t reg,
				uint32_t match, uint32_t *status)
{
	int rc, retries;
	const int max_retries = UPDATE_BLOCK_FAIL_RETRIES;
	struct device *dev = priv->dev;

	/* 1ms of delay after write is required as per FW */
	usleep_range(IAXXX_UPDATE_BLK_READ_DELAY, IAXXX_UPDATE_BLK_READ_DELAY +
		     IAXXX_UPDATE_BLK_READ_DELAY_RANGE);

	for (retries = 0; retries < max_retries; ++retries) {
		rc = regmap_read(regmap, reg, status);
		if (rc) {
			/* We should not return here until we are done
			 * with our retries
			 */
			dev_err(dev, "regmap_read failed, rc = %d\n", rc);

			if (rc == -EIO)
				break;
		}

		/* Read value matches */
		if (*status == match)
			return 0;
		else if (!(*status & IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK) &&
			(*status & IAXXX_SRB_SYS_BLK_UPDATE_RES_MASK) &&
			(*status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK)) {
			/*
			 * If the BLOCK_UPDATE_RESULT is non-zero, then some
			 * update failed. The caller(s) will need to determine
			 * how to handle the failure.
			 */
			dev_err(dev, "%s status 0x%x\n", __func__, *status);
			return -ENXIO;
		}

		/*
		 * Ideally 1ms of delay before read is enough, but if bit is not
		 * cleared yet wait for 2ms before retry
		 */
		usleep_range(IAXXX_UPDATE_BLK_READ_RETRY_DELAY,
				IAXXX_UPDATE_BLK_READ_RETRY_DELAY +
				IAXXX_UPDATE_BLK_READ_DELAY_RANGE);
	}

	rc = -ETIMEDOUT;
	dev_err(dev, "%s:Update block timed out\n", __func__);

	return rc;
}

static inline int
iaxxx_regmap_clear_bits(struct iaxxx_priv *priv, uint32_t reg, uint32_t mask)
{
	return regmap_update_bits(priv->regmap, reg, mask, 0);
}

static int iaxxx_read_error_register(struct iaxxx_priv *priv,
		struct regmap *regmap)
{
	int rc;
	int i;

	rc = regmap_read(regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_0_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_0]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 0 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_0_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_0]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 0 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_1_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_1]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 1 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_1_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_1]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 1 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_2_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_2]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 2 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_2_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_2]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 2 addr failed %d\n", rc);
		goto out;
	}
	for (i = 0 ; i < IAXXX_MAX_CORES; i++)
		dev_info(priv->dev, "Block id %d : Err 0x%x Inst id 0x%x\n", i,
		priv->iaxxx_state->err.error[i],
		priv->iaxxx_state->err.plg_inst_id[i] == PLUGIN_INST_NONE ? 0
			: priv->iaxxx_state->err.plg_inst_id[i]);
out:
	return rc;
}


static int iaxxx_update_block_status_check_after_wait(struct device *dev,
						struct regmap *regmap)
{
	int rc = 0;
	uint32_t status;

	rc = regmap_read(regmap,
			IAXXX_SRB_SYS_BLK_UPDATE_ADDR, &status);
	if (rc) {
		dev_err(dev, "%s failed update block status read = %d\n",
			__func__, rc);
		goto update_block_check_err;
	}
	dev_dbg(dev, "%s update block status = 0x%x\n",
		__func__, status);
	/* Update Block Req bit not cleared is not expected */
	if (status & IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK) {
		dev_err(dev,
			"%s Critical error looks like fw in bad shape = 0x%x\n",
			__func__, status);
		rc = -EINVAL;
		goto update_block_check_err;
	}

	if (((status & IAXXX_SRB_SYS_BLK_UPDATE_RES_MASK) != 0xFF) &&
		((status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK) != 0x00)) {
		dev_err(dev, "%s update block failed with error= 0x%x\n",
			__func__, status);
		/* clear stale errors */
		regmap_write(regmap, IAXXX_SRB_SYS_BLK_UPDATE_ADDR, 0);
	}

update_block_check_err:
	return rc;
}

/**
 * iaxxx_update_block_request - sends Update Block Request & waits for response
 *
 * @priv     : iaxxx private data
 * @block_id : block id of the processor
 * @host_id  : id of the Host
 * @no_pm    : if main regmap or no_pm regmap
 * @fixed_wait: if true use fixed time to wait after update block otherwise
 *              use regular wait logic.
 * @wait_time_in_ms: wait time to use if fixed_wait is true.
 * @options  : options
 * @status   : pointer for the returned system status
 *
 * Core lock must be held by the caller.
 *
 * All requests are expected to finish within a few milliseconds and therefore
 * polling is used rather than using event interrupts to signal completion.
 */

static int iaxxx_update_block_request(struct iaxxx_priv *priv,
		int  block_id,
		int  host_id,
		struct regmap *regmap,
		uint32_t wait_time_in_ms,
		enum update_block_options_t options,
		uint32_t *status)
{
	int rc;
	int ret;
	struct device *dev = priv->dev;
	uint32_t sys_blk_update_addr;
	uint32_t reserved_bits_mask = 0;
	uint32_t result_bits_mask = 0;

	if (!status)
		return -EINVAL;

	dev_dbg(dev, "%s() block_id:%d\n", __func__, block_id);

	/* Choose update block register address based on block id
	 * and host_id
	 */
	switch (block_id) {
	case IAXXX_BLOCK_0:
		sys_blk_update_addr = (host_id == IAXXX_HOST_1) ?
				IAXXX_SRB_SYS_BLK_UPDATE_HOST_1_ADDR :
				IAXXX_SRB_SYS_BLK_UPDATE_ADDR;
		break;

	case IAXXX_BLOCK_1:
		sys_blk_update_addr = (host_id == IAXXX_HOST_1) ?
				IAXXX_SRB_SYS_BLK_UPDATE_1_HOST_1_ADDR :
				IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR;
		break;

	case IAXXX_BLOCK_2:
		sys_blk_update_addr = (host_id == IAXXX_HOST_1) ?
				IAXXX_SRB_SYS_BLK_UPDATE_2_HOST_1_ADDR :
				IAXXX_SRB_SYS_BLK_UPDATE_2_ADDR;
		break;

	default:
		return -EINVAL;
	}

	/* This check ensure that pm_resume is executed in normal use case
	 * scenarios except from the suspend path as regmap_no_pm is used in
	 * those functions.  This ensure that update_block_lock is not contended
	 * by both resume and suspend paths simultaneously
	 */
	if (regmap == priv->regmap) {
		rc = iaxxx_pm_get_sync(priv->dev);
		if (rc < 0) {
			dev_err(priv->dev,
				"%s failed to get pm_sync rc= 0x%x\n",
				__func__, rc);
			goto get_sync_err;
		}
	}

	/* To protect concurrent update blocks requests.
	 *
	 * If the option to not use update block locking
	 * don't lock the mutex.
	 */
	if (!(options & UPDATE_BLOCK_NO_LOCK_OPTION))
		mutex_lock(&priv->update_block_lock);

	/* Based on app-mode or sbl-mode choose what response to
	 * expect for update block status
	 *
	 * In SBL mode Reserved bits are not cleared but no RES value
	 * In App mode Reserved bits are cleared but RES returns 0xFF
	 * for no error.
	 *
	 */
	if (options & UPDATE_BLOCK_FIXED_WAIT_OPTION) {
		result_bits_mask	= 0;
		reserved_bits_mask	= 0;
	} else if (priv->is_application_mode)
		result_bits_mask = IAXXX_SRB_SYS_BLK_UPDATE_RES_MASK;
	else
		reserved_bits_mask =
				IAXXX_SRB_SYS_BLK_UPDATE_RESERVED_BITS_MASK;

	/* clear all stale errors */
	rc = regmap_write(regmap, sys_blk_update_addr, 0);
	/* Set the UPDATE_BLOCK_REQUEST bit and some reserved bits
	 * in the SRB_UPDATE_CTRL register
	 */
	rc = regmap_write(regmap, sys_blk_update_addr,
			       IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK |
			       reserved_bits_mask);
	if (rc) {
		dev_err(dev, "Failed to set UPDATE_BLOCK_REQUEST bit\n");
		goto out;
	}

	/* If fixed wait is needed after update block wait and return */
	if (options & UPDATE_BLOCK_FIXED_WAIT_OPTION) {
		msleep(wait_time_in_ms);

		/* If the option to check for SRB status after fixed
		 * wait is enabled, then check the status and
		 * return.
		 */
		if (options &
			UPDATE_BLOCK_STATUS_CHECK_AFTER_FIXED_WAIT_OPTION)
			rc = iaxxx_update_block_status_check_after_wait(dev,
									regmap);
		goto out;
	}

	/*
	 * Note that in application mode, events can be used instead of polling.
	 * Make a wait for event like task that either polls or waits for an
	 * incoming event and wakes us up when the block update has completed.
	 */

	/* Poll waiting for the UPDATE_BLOCK_REQUEST bit to clear
	 * and reserved bits to match the value written
	 */
	rc = iaxxx_regmap_wait_match(priv, regmap, sys_blk_update_addr,
			reserved_bits_mask | result_bits_mask, status);
	if (rc) {
		dev_err(dev, "Update Block Failed, rc = %d\n", rc);
		goto out;
	}

	/* Clear reserved bits set earlier in update block status */
	if (reserved_bits_mask)
		*status &= (~reserved_bits_mask);

out:
	/* We should not acquire lock during pm operations
	 * otherwise it leads to dead lock
	 */
	if (!(options & UPDATE_BLOCK_NO_LOCK_OPTION))
		mutex_unlock(&priv->update_block_lock);
	if (rc) {
		ret = iaxxx_read_error_register(priv, regmap);
		if (ret)
			dev_err(dev, "Read error register failed %d\n", ret);
	}
get_sync_err:
	if (regmap == priv->regmap)
		iaxxx_pm_put_autosuspend(priv->dev);

	return rc;
}

static int iaxxx_check_update_block_err_and_recover(struct device *dev,
		int rc, uint32_t status, uint32_t proc_id)
{
	uint32_t err = 0;

	if (rc == -ETIMEDOUT) {
		dev_err(dev, "%s:Update block timed out\n", __func__);
	} else if (rc == -ENXIO) {
		err = status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK;
		if (err == SYSRC_FAIL)
			dev_err(dev, "FW SYSRC_FAIL:0x%x\n", status);
		else if (err == SYSRC_ERR_MEM)
			dev_err(dev, "FW NOMEM err:0x%x\n", status);
		else if (err) {
			dev_err(dev, "FW general err:0x%x\n", status);
			return -EINVAL;
		}
	} else
		return rc;

	iaxxx_fw_crash(dev, IAXXX_FW_CRASH_UPDATE_BLOCK_REQ, proc_id);
	return rc;
}

/*===========================================================================
 * Bootloader
 *===========================================================================
 */

static int iaxxx_bootloader_request(struct iaxxx_priv *priv, u32 mask, u32 val)
{
	int rc;
	uint32_t status;
	struct device *dev = priv->dev;

	/* Set the request bit in BOOTLOADER_REQUEST register */
	rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_BOOT_REQ_ADDR, mask, val);
	if (rc) {
		dev_err(dev, "Failed to set BOOTLOADER_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/*
	 * Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register
	 * to trigger BOOTLOADER request. Don't check update block
	 * status to avoid spi-reads that could return invalid data.
	 *
	 */
	rc = regmap_write(priv->regmap_no_pm,
			IAXXX_SRB_SYS_BLK_UPDATE_ADDR,
			IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "Failed to set UPDATE_BLOCK_REQUEST bit\n");
		goto out;
	}

	/* Allow time for firmware to boot */
	msleep(200);

	/* The bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap_no_pm,
			IAXXX_SRB_BOOT_REQ_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read BOOTLOADR_REQUEST, rc = %d\n", rc);
		goto out;
	}

	if (WARN_ON(status & mask)) {
		/* Clear the request bit */
		regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_BOOT_REQ_ADDR, mask, 0);
	}

out:
	return rc;
}

int iaxxx_jump_to_request(struct iaxxx_priv *priv, uint32_t address)
{
	int rc;
	struct device *dev = priv->dev;

	/* Program BOOTLOADER_JUMP_ADDRESS with the start address */
	rc = regmap_write(priv->regmap_no_pm,
			IAXXX_SRB_BOOT_JUMP_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "Failed to set JUMP_ADDRESS, rc = %d\n", rc);
		goto out;
	}

	/* Send the request */
	rc = iaxxx_bootloader_request(priv,
				IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK,
				0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS);
	if (rc) {
		dev_err(dev, "JUMP_REQUEST failed, rc = %d\n", rc);
		goto out;
	}

out:
	return rc;
}

int iaxxx_calibrate_oscillator_request(struct iaxxx_priv *priv, uint32_t delay)
{
	return -ENOTSUPP;
}

/*===========================================================================
 * Checksum
 *===========================================================================
 */
int iaxxx_checksum_request(struct iaxxx_priv *priv, uint32_t address,
			uint32_t length, uint32_t *sum1, uint32_t *sum2,
			struct regmap *regmap)
{
	int rc;
	uint32_t status;
	uint32_t request = 0;
	struct device *dev = priv->dev;

	if (!sum1 || !sum2)
		return -EINVAL;

	/* Set the CHECKSUM_VALUE */
	rc = regmap_write(regmap,
			IAXXX_SRB_CHKSUM_VAL1_ADDR, *sum1);
	if (rc) {
		dev_err(dev, "Unable to clear CHECKSUM_VALUE1, rc = %d\n", rc);
		goto out;
	}

	rc = regmap_write(regmap,
			IAXXX_SRB_CHKSUM_VAL2_ADDR, *sum2);
	if (rc) {
		dev_err(dev, "Unable to clear CHECKSUM_VALUE2, rc = %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_ADDRESS with first memory address */
	rc = regmap_write(regmap,
			IAXXX_SRB_CHKSUM_BUFFER_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_ADDRESS, rc = %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_LENGTH with the buffer length (in bytes) */
	rc = regmap_write(regmap,
			  IAXXX_SRB_CHKSUM_LENGTH_ADDR, length);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_LENGTH, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Set the CHECKSUM_REQUEST bit in CHECKSUM_REQUEST */
	rc = regmap_update_bits(regmap, IAXXX_SRB_CHKSUM_ADDR,
				IAXXX_SRB_CHKSUM_REQ_MASK,
				0x1 << IAXXX_SRB_CHKSUM_REQ_POS);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, IAXXX_BLOCK_0,
			IAXXX_HOST_0, regmap, 0,
			UPDATE_BLOCK_NO_OPTIONS, &status);
	if (rc) {
		dev_err(dev, "CHECKSUM_REQUEST failed, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* The CHECKSUM_REQUEST bit should have been cleared by firmware */
	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_ADDR, &request);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_REQUEST, rc = %d\n", rc);
		goto out;
	}

	if (request & IAXXX_SRB_CHKSUM_REQ_MASK)
		goto out;

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Read the checksum from CHECKSUM_VALUE */
	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_VAL1_ADDR, sum1);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_VALUE1, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	rc = regmap_read(regmap, IAXXX_SRB_CHKSUM_VAL2_ADDR, sum2);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_VALUE2, rc = %d\n", rc);
		goto out;
	}

out:
	return rc;
}

/*===========================================================================
 * Script
 *===========================================================================
 */

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_send_update_block_request - sends Update Block Request to the device
 *
 * @dev    : MFD core device pointer
 * @status : pointer for the returned system status
 * @id     : Block ID/ Proc ID
 *
 * This call will be blocked until either the device responds to the request
 * or a timeout occurs while waiting for the response.
 */
int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
						int id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;

	if (!priv)
		return -EINVAL;

	rc = iaxxx_update_block_request(priv, id,
			IAXXX_HOST_0, priv->regmap,
			0, UPDATE_BLOCK_NO_OPTIONS,
			status);

	rc = iaxxx_check_update_block_err_and_recover(dev, rc, *status,
			IAXXX_BLOCK_ID_TO_PROC_ID(id));

	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_request);

/**
 * iaxxx_send_update_block_request_with_options -
 * sends update Block Request to the device based on options.
 *
 * @dev    : MFD core device pointer
 * @block_id : Block ID/ Proc ID
 * @host_id : Host ID
 * @regmap  : regmap to use
 * @wait_time_in_ms: wait time to use if fixed_wait flag is set
 * @options : options:
 *            UPDATE_BLOCK_FIXED_WAIT_OPTION - This option enables fixed wait
 *            time specified by wait_time_in_ms parameter. This delay would
 *            execute instead of polling for status after sending update block
 *            to firmware.
 *
 *            UPDATE_BLOCK_NO_LOCK_OPTION - This option disables mutex lock
 *            during whole update block operation. This option is used
 *            from power management context which can trigger another
 *            update block.
 *
 *            UPDATE_BLOCK_STATUS_CHECK_AFTER_FIXED_WAIT_OPTION - This option
 *            is used in combination with UPDATE_BLOCK_FIXED_WAIT_OPTION to
 *            do a status check after update block and fixed wait.
 *
 * @status: status returned on error.
 *
 */
int iaxxx_send_update_block_request_with_options(struct device *dev,
					int block_id,
					int host_id,
					struct regmap *regmap,
					uint32_t wait_time_in_ms,
					enum update_block_options_t options,
					uint32_t *status)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;

	if (!priv)
		return -EINVAL;

	rc = iaxxx_update_block_request(priv, block_id, host_id, regmap,
					wait_time_in_ms, options, status);

	rc = iaxxx_check_update_block_err_and_recover(dev, rc, *status,
			IAXXX_BLOCK_ID_TO_PROC_ID(block_id));

	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_request_with_options);

int iaxxx_send_update_block_hostid(struct device *dev,
		int host_id, int block_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;
	uint32_t status;

	dev_dbg(dev, "%s()\n", __func__);

	rc = iaxxx_update_block_request(priv, block_id,
					host_id, priv->regmap,
					0,
					UPDATE_BLOCK_NO_OPTIONS,
					&status);

	rc = iaxxx_check_update_block_err_and_recover(dev, rc, status,
			IAXXX_BLOCK_ID_TO_PROC_ID(block_id));
	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_hostid);

int iaxxx_poll_update_block_req_bit_clr(struct iaxxx_priv *priv,
			struct regmap *regmap)
{
	int rc = 0;
	uint32_t status;
	uint32_t retry_count = 10;

	if (regmap == priv->regmap) {
		rc = iaxxx_pm_get_sync(priv->dev);
		if (rc < 0) {
			dev_err(priv->dev,
				"%s failed to get pm_sync rc= 0x%x\n",
				__func__, rc);
			goto get_sync_err;
		}

		mutex_lock(&priv->update_block_lock);
	}

	/* Make sure update block bit is in cleared state */
	do {
		rc = regmap_read(regmap,
				IAXXX_SRB_SYS_BLK_UPDATE_ADDR,
				&status);
		if (rc) {
			pr_err("Failed to read SRB_SYS_BLK_UPDATE_ADDR, rc = %d\n",
				rc);
			goto update_block_clr_check_err;
		}
		if (status & IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK)
			usleep_range(1000, 1005);
		else
			break;
	} while (retry_count--);
	if (status & IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK) {
		pr_err("Update Block bit not cleared, rc = %d\n", rc);
		rc = -EBUSY;
	}

update_block_clr_check_err:
	if (regmap == priv->regmap)
		mutex_unlock(&priv->update_block_lock);
get_sync_err:
	if (regmap == priv->regmap)
		iaxxx_pm_put_autosuspend(priv->dev);
	return rc;
}
EXPORT_SYMBOL(iaxxx_poll_update_block_req_bit_clr);

int iaxxx_get_firmware_version(struct device *dev, char *ver, uint32_t len)
{
	return iaxxx_get_version_str(to_iaxxx_priv(dev),
		IAXXX_SRB_SYS_APP_VER_STR_ADDR, ver, len);
}
EXPORT_SYMBOL(iaxxx_get_firmware_version);

int iaxxx_get_application_ver_num(struct device *dev, uint32_t *app_ver_num)
{
	int rc;

	rc = regmap_read(to_iaxxx_priv(dev)->regmap,
				IAXXX_SRB_SYS_APP_VER_NUM_ADDR, app_ver_num);
	if (rc) {
		dev_err(dev, "Failed to read SRB_SYS_APP_VER_NUM, rc = %d\n",
									rc);
	}

	return rc;
}
EXPORT_SYMBOL(iaxxx_get_application_ver_num);

int iaxxx_get_rom_version(struct device *dev, char *ver, uint32_t len)
{
	return iaxxx_get_version_str(to_iaxxx_priv(dev),
		IAXXX_SRB_SYS_ROM_VER_STR_ADDR, ver, len);
}
EXPORT_SYMBOL(iaxxx_get_rom_version);

int iaxxx_get_rom_ver_num(struct device *dev, uint32_t *rom_ver_num)
{
	int rc;

	rc = regmap_read(to_iaxxx_priv(dev)->regmap,
				IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, rom_ver_num);
	if (rc) {
		dev_err(dev, "Failed to read SRB_SYS_ROM_VER_NUM, rc = %d\n",
									rc);
	}

	return rc;
}
EXPORT_SYMBOL(iaxxx_get_rom_ver_num);

int iaxxx_get_device_id(struct device *dev, uint32_t *device_id)
{
	int rc;

	rc = regmap_read(to_iaxxx_priv(dev)->regmap,
				IAXXX_SRB_SYS_DEVICE_ID_ADDR, device_id);
	if (rc) {
		dev_err(dev, "Failed to read SRB_SYS_DEVICE_ID, rc = %d\n",
									rc);
	}

	return rc;
}
EXPORT_SYMBOL(iaxxx_get_device_id);
