/*
 * iaxxx-debug.c  --  iaxxx debug support
 *
 * Copyright 2018 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/mfd/adnc/iaxxx-debug-intf.h>
#include <linux/circ_buf.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-tunnel-registers.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"

#define GET_INTF_PRIV(iaxxx) \
	((!iaxxx || !iaxxx->intf_priv) ? NULL : iaxxx->intf_priv)
#define IAXXX_I2C		0
#define IAXXX_SPI		1
#define IAXXX_MAX_LOG_SIZE	50

struct iaxxx_debug_data {
	struct iaxxx_priv *priv;
	struct iaxxx_cdev raw_cdev;
	struct iaxxx_cdev regdump_cdev;
	struct iaxxx_cdev crashdump_cdev;
};

static ssize_t raw_read(struct file *filp, char __user *buf,
			size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf = NULL;

	pr_debug("%s() called\n", __func__);

	if (!iaxxx) {
		pr_err("Invalid private pointer");
		rc = -EINVAL;
		goto raw_read_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("failed to copy user data of len: %d\n", (u32)count);
		rc = -ENOMEM;
		goto raw_read_err;
	}

	rc = iaxxx->raw_ops->read(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("failed to read data: %d", rc);
		rc = -EIO;
	} else {
		rc = copy_to_user(buf, kbuf, count);
		if (rc) {
			pr_err("failed to copy response to userspace %d", rc);
			rc = -EIO;
			goto raw_read_err;
		}

		rc = count;
	}

raw_read_err:
	kfree(kbuf);
	return rc;
}

static ssize_t raw_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf;

	pr_debug("%s() called\n", __func__);

	if (!iaxxx) {
		pr_err("Invalid private pointer");
		rc = -EINVAL;
		goto raw_write_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("failed to copy user data of len: %d\n", (u32)count);
		rc = -ENOMEM;
		goto raw_write_err;
	}

	rc = iaxxx->raw_ops->write(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("failed to write data: %d", rc);
		rc = -EIO;
	} else {
		rc = count;
	}

	kfree(kbuf);

raw_write_err:
	return rc;
}

static int get_srb_info(struct iaxxx_priv *iaxxx, struct iaxxx_srb_info *srb)
{
	int ret;

	ret = iaxxx->bulk_read(iaxxx->dev,
			IAXXX_SRB_REGS_ADDR, srb->reg_vals, IAXXX_SRB_REGS_NUM);
	if (ret < 0) {
		pr_err("Can't get SRB register (ret=%d)\n", ret);
	} else {
		srb->reg_start_addr = IAXXX_SRB_REGS_ADDR;
		srb->reg_num = IAXXX_SRB_REGS_NUM;
	}

	return ret;
}

static int get_arb_info(struct iaxxx_priv *iaxxx, struct iaxxx_arb_info *arb)
{
	int ret;
	int i;

	/* ARB */
	ret = iaxxx->bulk_read(iaxxx->dev, IAXXX_SRB_ARB_0_BASE_ADDR_ADDR,
					arb->reg_vals, IAXXX_ARB_REGS_NUM);
	if (ret < 0) {
		pr_err("Can't get ARB register (ret=%d)\n", ret);
		return ret;
	}

	arb->reg_start_addr = IAXXX_SRB_ARB_0_BASE_ADDR_ADDR;
	arb->reg_num = IAXXX_ARB_REGS_NUM;
	for (i = 0; i < IAXXX_ARB_BLOCK_NUM; ++i) {
		uint32_t addr = arb->reg_vals[2 * i + 0];
		uint32_t reg_num =
			arb->reg_vals[2 * i + 1] / sizeof(uint32_t);
		struct iaxxx_arb_block *block = &arb->blocks[i];

		block->reg_start_addr = addr;
		block->reg_num = reg_num;
		if (addr == 0 || reg_num == 0) {
			block->reg_start_addr = 0;
			block->reg_num = 0;
			continue;
		}
		block->reg_start_addr = addr;
		block->reg_num = reg_num;
		if (reg_num > IAXXX_MAX_REGS_NUM) {
			pr_err("Too large reg_num(%u)\n", reg_num);
			continue;
		}

		ret = iaxxx->bulk_read(iaxxx->dev,
					addr, block->reg_vals, reg_num);
		if (ret < 0) {
			pr_err("Can't get ARB Block %d (ret=%d)\n", i, ret);
			continue;
		}
	}

	return ret;
}

static int get_circ_buffer_info(struct iaxxx_priv *iaxxx,
			 struct iaxxx_circ_buffer_info *circ_buffer_info)
{
	int ret;
	uint32_t arb10_phy_addr;
	int i;
	static uint32_t reg_vals[IAXXX_MAX_CIRC_BUFS * 2];

	/* First get the address for the ARB10 block */
	ret = iaxxx->bulk_read(iaxxx->dev,
		IAXXX_SRB_ARB_10_BASE_ADDR_ADDR, &arb10_phy_addr, 1);
	if (ret < 0) {
		pr_err("Failed to find physical address for ARB10\n");
		return ret;
	}

	/* We need to read only NUM_CIRC_BUFS*2 words from ARB10 */
	ret = iaxxx->bulk_read(iaxxx->dev,
			arb10_phy_addr, reg_vals, IAXXX_MAX_CIRC_BUFS * 2);
	if (ret < 0) {
		pr_err(
		"Failed to read Circular buffer register information\n");
		return ret;
	}

	circ_buffer_info->buf_num = IAXXX_MAX_CIRC_BUFS;
	for (i = 0; i < circ_buffer_info->buf_num; ++i) {
		uint32_t phy_addr, words;
		struct iaxxx_circ_buffer *buf;

		/*
		 * Get the physical address and size for
		 * each of the circular buffers
		 */
		phy_addr = reg_vals[2 * i + 0];
		words = reg_vals[2 * i + 1] / sizeof(uint32_t);
		if (phy_addr == 0 || words == 0) {
			pr_err("Couldn't find physical address and size\n");
			continue;
		}
		if (words > IAXXX_MAX_REGS_NUM) {
			pr_err("Too large words(%u)\n", words);
			continue;
		}

		buf = circ_buffer_info->bufs + i;
		ret = iaxxx->bulk_read(iaxxx->dev,
					phy_addr, buf->reg_vals, words);
		if (ret < 0) {
			pr_err("Failed to get the circ buf(%d) data\n", i);
			buf->reg_start_addr = 0;
			buf->reg_num = 0;
		} else {
			buf->reg_start_addr = phy_addr;
			buf->reg_num = words;
		}
	}

	return ret;
}

static int get_registers_dump(struct iaxxx_priv *iaxxx,
			  struct iaxxx_registers_dump *info)
{
	int ret = 0;

	if (get_srb_info(iaxxx, &info->srb_info) < 0)
		ret = -EIO;

	if (get_arb_info(iaxxx, &info->arb_info) < 0)
		ret = -EIO;

	if (get_circ_buffer_info(iaxxx, &info->circ_buffer_info) < 0)
		ret = -EIO;

	return ret;
}

static int get_tunnel_info_dump(struct iaxxx_priv *iaxxx,
			struct iaxxx_tunnel_info_dump *dump)
{
	int ret;
	uint32_t id = dump->id;
	uint32_t *tmp_buf;

	if (id >= IAXXX_TUNNEL_MAX) {
		pr_err("%s() Invalid tunnel id %u\n", __func__, id);
		return -EINVAL;
	}

	tmp_buf = kvzalloc(max(IAXXX_TNL_HDR_REG_NUM,
				IAXXX_OUT_TNL_GRP_REG_NUM) *
				sizeof(uint32_t),
				GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_TNL_HDR_REGS_ADDR, tmp_buf, IAXXX_TNL_HDR_REG_NUM);
	if (ret) {
		pr_err(
		"%s() regmap_bulk_read IAXXX_TNL_HDR_REGS_ADDR failed %d\n",
								__func__, ret);
		goto exit;
	}

	dump->out_buf_size = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_ADDR)];
	dump->out_buf_addr = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_OUT_BUF_ADDR_ADDR)];
	dump->out_buf_head = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_OUT_BUF_HEAD_ADDR)];
	dump->out_buf_tail = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_OUT_BUF_TAIL_ADDR)];
	dump->out_buf_threshold = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
				IAXXX_TNL_HDR_TNL_OUT_BUF_THRESHOLD_ADDR)];
	dump->en = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_ENABLE_ADDR)];
	dump->st = tmp_buf[IAXXX_TNL_HDR_REG_OFFSET(
					IAXXX_TNL_HDR_TNL_ST_ADDR)];

	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_OUT_TNL_GRP_REGS(id), tmp_buf, IAXXX_OUT_TNL_GRP_REG_NUM);
	if (ret) {
		pr_err(
		"%s() regmap_bulk_read from IAXXX_IN_TNL_GRP_REGS failed %d\n",
								__func__, ret);
		goto exit;
	}

	dump->nframe_drops = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_ADDR)];
	dump->nsent_to_host = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_TNL_NSENT_TO_HOST_ADDR)];
	dump->nsent = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_TNL_NSENT_ADDR)];
	dump->nrecvd = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_TNL_NRECVD_ADDR)];
	dump->ctrl = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_TNL_CTRL_ADDR)];
	dump->format = tmp_buf[IAXXX_OUT_TNL_GRP_REG_OFFSET(
				IAXXX_OUT_TNL_GRP_CONNECT_ADDR)];

exit:
	kvfree(tmp_buf);
	return ret;
}

static int get_channel_info_dump(struct iaxxx_priv *iaxxx,
			struct iaxxx_channel_info_dump *dump)
{
	int ret;
	uint32_t id = dump->id;
	uint32_t *tmp_buf;

	if (id >= IAXXX_CHANNEL_MAX) {
		pr_err("%s() Invalid channel id %u\n", __func__, id);
		return -EINVAL;
	}

	tmp_buf = kvzalloc(max(IAXXX_CH_HDR_REG_NUM,
				IAXXX_IN_CH_GRP_REG_NUM) *
				sizeof(uint32_t),
				GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_CH_HDR_REGS_ADDR, tmp_buf, IAXXX_CH_HDR_REG_NUM);
	if (ret) {
		pr_err(
	"%s() regmap_bulk_read from IAXXX_CH_HDR_REGS_ADDR failed %d\n",
								__func__, ret);
		goto exit;
	}

	dump->st = tmp_buf[IAXXX_CH_HDR_REG_OFFSET(IAXXX_CH_HDR_CH_ST_ADDR)];
	dump->direction = tmp_buf[
		IAXXX_CH_HDR_REG_OFFSET(IAXXX_CH_HDR_CH_DIR_ADDR)];
	dump->gain = tmp_buf[
		IAXXX_CH_HDR_REG_OFFSET(IAXXX_CH_HDR_CH_GAIN_ADDR)];

	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_OUT_CH_GRP_REGS(id), tmp_buf, IAXXX_OUT_CH_GRP_REG_NUM);
	if (ret) {
		pr_err(
		"%s() regmap_bulk_read from IAXXX_OUT_CH_GRP_REGS failed %d\n",
								__func__, ret);
		goto exit;
	}

	dump->nsent = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_NSENT_ADDR)];
	dump->nrecvd = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_NRECVD_ADDR)];
	dump->endpoint_state = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
				IAXXX_OUT_CH_GRP_CH_ENDPOINT_STATE_ADDR)];
	dump->intr_cnt = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_INTR_CNT_ADDR)];
	dump->drop_cnt = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_DROP_CNT_ADDR)];
	dump->gain_ctrl = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR)];
	dump->gain_status = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_CH_GAIN_STATUS_ADDR)];
	dump->in_connect = tmp_buf[IAXXX_OUT_CH_GRP_REG_OFFSET(
					IAXXX_OUT_CH_GRP_IN_CONNECT_ADDR)];

exit:
	kvfree(tmp_buf);
	return ret;
}

static int get_stream_info_dump(struct iaxxx_priv *iaxxx,
			struct iaxxx_stream_info_dump *dump)
{
	int ret;
	uint32_t id = dump->id;
	uint32_t *tmp_buf;

	if (id >= IAXXX_STREAM_MAX) {
		pr_err("%s() Invalid stream id %u\n", __func__, id);
		return -EINVAL;
	}

	tmp_buf = kvzalloc(max(IAXXX_STR_HDR_REG_NUM,
				IAXXX_STR_GRP_REG_NUM) *
				sizeof(uint32_t),
				GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	/* Read the header */
	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_STR_HDR_REGS_ADDR, tmp_buf, IAXXX_STR_HDR_REG_NUM);
	if (ret) {
		pr_err(
	"%s() regmap_bulk_read from IAXXX_STR_HDR_REGS_ADDR failed %d\n",
								__func__, ret);
		goto exit;
	}

	/* Write values from buffer to structure */
	dump->en =
		tmp_buf[IAXXX_STR_HDR_REG_OFFSET(IAXXX_STR_HDR_STR_EN_ADDR)];
	dump->st =
		tmp_buf[IAXXX_STR_HDR_REG_OFFSET(IAXXX_STR_HDR_STR_ST_ADDR)];

	/* Read the group */
	ret = regmap_bulk_read(iaxxx->regmap,
		IAXXX_STR_GRP_REGS(id), tmp_buf, IAXXX_STR_GRP_REG_NUM);
	if (ret) {
		pr_err(
	"%s() regmap_bulk_read from IAXXX_STR_GRP_REGS(%u) failed %d\n",
							__func__, id, ret);
		goto exit;
	}

	dump->af_error_afs_fifo_overflow_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_AFS_FIFO_OVERFLOW_CNT_ADDR)];
	dump->af_error_afs_fifo_underflow_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_AFS_FIFO_UNDERFLOW_CNT_ADDR)];
	dump->af_error_tus_fifo_overflow_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_OVERFLOW_CNT_ADDR)];
	dump->af_error_tus_fifo_underflow_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_UNDERFLOW_CNT_ADDR)];
	dump->af_error_tus_fifo_coherency_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_COHERENCY_CNT_ADDR)];
	dump->af_error_deadline_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_DEADLINE_CNT_ADDR)];
	dump->af_error_phy_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_PHY_CNT_ADDR)];
	dump->af_error_timeout_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_TIMEOUT_CNT_ADDR)];
	dump->af_error_access_cnt = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(
			IAXXX_STR_GRP_STR_AF_ERR_ACCESS_CNT_ADDR)];

	dump->ctrl = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_CTRL_ADDR)];
	dump->status = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_STATUS_ADDR)];
	dump->format = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_FORMAT_ADDR)];
	dump->port = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_PORT_ADDR)];
	dump->channel = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_CHN_ADDR)];
	dump->sync = tmp_buf[
		IAXXX_STR_GRP_REG_OFFSET(IAXXX_STR_GRP_STR_SYNC_ADDR)];
exit:
	kvfree(tmp_buf);
	return ret;
}

static long raw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct iaxxx_priv *const iaxxx
		= (struct iaxxx_priv *)file->private_data;
	struct iaxxx_log_level_info log_level_info;
	struct iaxxx_log_mode_info log_mode_info;
	struct iaxxx_plgin_log_mode_info plgin_log_mode;
	struct iaxxx_plgin_log_state_info plgin_log_state;
	int ret = 0;
	u8 bus_config;
	bool mode;
	bool state;
	uint32_t log_level;

	pr_debug("%s() called\n", __func__);

	if (iaxxx == NULL) {
		pr_err("Invalid private pointer\n");
		return -EINVAL;
	}

	switch (cmd) {
	case IAXXX_BUS_CONFIG:
#if defined(CONFIG_MFD_IAXXX_SPI)
		bus_config = IAXXX_SPI;
#elif defined(CONFIG_MFD_IAXXX_I2C)
		bus_config = IAXXX_I2C;
#endif
		ret = copy_to_user((void __user *)arg, &bus_config,
				   sizeof(u8));
		if (ret) {
			pr_err("Failed to copy response to userspace %d\n",
									ret);
			ret = -EFAULT;
		}
		break;
	case IAXXX_IOCTL_GET_REGISTERS_DUMP: {
		struct iaxxx_registers_dump *dump;

		dump = kzalloc(sizeof(struct iaxxx_registers_dump), GFP_KERNEL);
		if (!dump)
			return -ENOMEM;

		mutex_lock(&iaxxx->debug_mutex);
		ret = get_registers_dump(iaxxx, dump);
		mutex_unlock(&iaxxx->debug_mutex);
		if (ret) {
			pr_err("Failed to get_registers_dump ret=%d\n", ret);
		} else {
			if (copy_to_user((void __user *)arg, dump,
					sizeof(struct iaxxx_registers_dump)))
				ret = -EFAULT;
		}
		kfree(dump);
		break;
	}
	case IAXXX_IOCTL_GET_TUNNEL_INFO_DUMP: {
		struct iaxxx_tunnel_info_dump dump;

		if (copy_from_user(&dump, (void __user *)arg, sizeof(dump)))
			return -EFAULT;
		ret = get_tunnel_info_dump(iaxxx, &dump);
		if (ret) {
			pr_err("Failed to get_tunnel_info_dump ret=%d\n", ret);
		} else {
			if (copy_to_user((void __user *)arg,
						&dump, sizeof(dump)))
				ret = -EFAULT;
		}
		break;
	}
	case IAXXX_IOCTL_GET_CHANNEL_INFO_DUMP: {
		struct iaxxx_channel_info_dump dump;

		if (copy_from_user(&dump, (void __user *)arg, sizeof(dump)))
			return -EFAULT;
		ret = get_channel_info_dump(iaxxx, &dump);
		if (ret) {
			pr_err(
			"Failed to get_channel_info_dump ret=%d\n", ret);
		} else {
			if (copy_to_user((void __user *)arg,
						&dump, sizeof(dump)))
				ret = -EFAULT;
		}
		break;
	}
	case IAXXX_IOCTL_GET_STREAM_INFO_DUMP: {
		struct iaxxx_stream_info_dump dump;

		if (copy_from_user(&dump, (void __user *)arg, sizeof(dump)))
			return -EFAULT;
		ret = get_stream_info_dump(iaxxx, &dump);
		if (ret) {
			pr_err("Failed to get_stream_info_dump ret=%d\n", ret);
		} else {
			if (copy_to_user((void __user *)arg,
						&dump, sizeof(dump)))
				ret = -EFAULT;
		}
		break;
	}
	case IAXXX_SET_DBG_LOG_LEVEL:
		if (copy_from_user(&log_level_info, (void __user *)arg,
						sizeof(log_level_info)))
			return -EFAULT;

		mutex_lock(&iaxxx->debug_mutex);
		ret = iaxxx_set_debug_log_level(iaxxx->dev,
					log_level_info.module_id,
					log_level_info.log_level);
		mutex_unlock(&iaxxx->debug_mutex);
		if (ret) {
			dev_err(iaxxx->dev, "%s(): failed %d\n", __func__, ret);
			return ret;
		}
		break;
	case IAXXX_GET_DBG_LOG_LEVEL:
		if (copy_from_user(&log_level_info, (void __user *)arg,
						sizeof(log_level_info)))
			return -EFAULT;

		mutex_lock(&iaxxx->debug_mutex);
		ret = iaxxx_get_debug_log_level(iaxxx->dev,
					log_level_info.module_id, &log_level);
		mutex_unlock(&iaxxx->debug_mutex);
		if (ret) {
			dev_err(iaxxx->dev, "%s(): failed %d\n", __func__, ret);
			return ret;
		}

		log_level_info.log_level = log_level;
		if (copy_to_user((void __user *)arg, &log_level_info,
						sizeof(log_level_info)))
			return -EFAULT;

		break;
	case IAXXX_SET_DBG_LOG_MODE:
		if (copy_from_user(&log_mode_info, (void __user *)arg,
						sizeof(log_mode_info)))
			return -EFAULT;

		mutex_lock(&iaxxx->debug_mutex);
		ret = iaxxx_set_debug_log_mode(iaxxx->dev,
			log_mode_info.mode, log_mode_info.proc_id);
		mutex_unlock(&iaxxx->debug_mutex);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}
		break;
	case IAXXX_GET_DBG_LOG_MODE:
		if (copy_from_user(&log_mode_info, (void __user *)arg,
				sizeof(log_mode_info)))
			return -EFAULT;

		mutex_lock(&iaxxx->debug_mutex);
		ret = iaxxx_get_debug_log_mode(iaxxx->dev,
			&mode, log_mode_info.proc_id);
		mutex_unlock(&iaxxx->debug_mutex);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}

		log_mode_info.mode = mode;
		if (copy_to_user((void __user *)arg, &log_mode_info,
						sizeof(log_mode_info)))
			return -EFAULT;

		break;
	case IAXXX_SET_PLUGIN_LOG_MODE:
		if (copy_from_user(&plgin_log_mode, (void __user *)arg,
						sizeof(plgin_log_mode)))
			return -EFAULT;

		ret = iaxxx_set_plugin_log_mode(iaxxx->dev,
				plgin_log_mode.mode, plgin_log_mode.inst_id,
				plgin_log_mode.block_id);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}
		break;
	case IAXXX_GET_PLUGIN_LOG_MODE:
		if (copy_from_user(&plgin_log_mode, (void __user *)arg,
				sizeof(plgin_log_mode)))
			return -EFAULT;

		ret = iaxxx_get_plugin_log_mode(iaxxx->dev,
			&mode, plgin_log_mode.inst_id, plgin_log_mode.block_id);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}

		plgin_log_mode.mode = mode;
		if (copy_to_user((void __user *)arg, &plgin_log_mode,
						sizeof(plgin_log_mode)))
			return -EFAULT;

		break;
	case IAXXX_SET_PLUGIN_LOG_STATE:
		if (copy_from_user(&plgin_log_state, (void __user *)arg,
						sizeof(plgin_log_state)))
			return -EFAULT;

		ret = iaxxx_set_update_plugin_log_state(iaxxx->dev,
				plgin_log_state.state, plgin_log_state.inst_id,
				plgin_log_state.block_id);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}
		break;
	case IAXXX_GET_PLUGIN_LOG_STATE:
		if (copy_from_user(&plgin_log_state, (void __user *)arg,
				sizeof(plgin_log_state)))
			return -EFAULT;

		ret = iaxxx_get_plugin_log_state(iaxxx->dev,
		&state, plgin_log_state.inst_id, plgin_log_state.block_id);
		if (ret) {
			dev_err(iaxxx->dev,
				"%s(): failed %d\n", __func__, ret);
			return ret;
		}

		plgin_log_state.state = state;
		if (copy_to_user((void __user *)arg, &plgin_log_state,
						sizeof(plgin_log_state)))
			return -EFAULT;

		break;
	default:
		pr_err("Invalid ioctl command received 0x%x\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long raw_compat_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	return raw_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int raw_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("raw device open called\n");

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode\n");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, raw_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data\n");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;

	return 0;
}

static int raw_release(struct inode *inode, struct file *filp)
{
	pr_debug("raw device release called\n");

	filp->private_data = NULL;
	return 0;
}

static ssize_t regdump_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	char *kbuf;
	int err;

	dev_dbg(iaxxx->dev, "%s() called\n", __func__);
	if (!iaxxx)
		return -EINVAL;
	kbuf = kvzalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;
	err = copy_from_user(kbuf, buf, count);
	if (err) {
		dev_err(iaxxx->dev, "%s() Copy error\n", __func__);
		kvfree(kbuf);
		return -EINVAL;
	}
	if (!strncmp(kbuf, "clear", (count - 1))) {
		spin_lock(&iaxxx->reg_dump->ring_lock);
		iaxxx->reg_dump->head = 0;
		iaxxx->reg_dump->tail = 0;
		spin_unlock(&iaxxx->reg_dump->ring_lock);
	} else
		dev_err(iaxxx->dev, "%s() Invalid command\n", __func__);

	kvfree(kbuf);
	return count;
}

static ssize_t regdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	struct iaxxx_reg_dump_priv *reg_dump;
	struct iaxxx_register_log log;
	char *kbuf;
	ssize_t bytes_written = 0;
	static uint32_t logs_to_read;
	static bool done;
	static uint32_t r_index;

	dev_dbg(iaxxx->dev, "%s() called\n", __func__);
	/* Return if no priv structure */
	if (!iaxxx)
		return -EINVAL;
	if (!iaxxx->reg_dump)
		return -EINVAL;

	/* Register dump read is complete */
	if (done) {
		done = false;
		return 0;
	}
	/* Allocate kernel buffer to read register dump */
	kbuf = kvzalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;
	reg_dump = iaxxx->reg_dump;
	spin_lock(&reg_dump->ring_lock);
	/* reading first time or last time read is complete */
	if (!logs_to_read) {
		logs_to_read = CIRC_CNT(reg_dump->head, reg_dump->tail,
				IAXXX_BUF_MAX_LEN);
		r_index = reg_dump->tail;
		bytes_written += scnprintf(kbuf + bytes_written, PAGE_SIZE,
				"R/W:\t[TimeStamp]\t0xAddress\t0xValue\n");
	}
	/*
	 * Until kernel buf full or all the logs read,
	 * (count - IAXXX_MAX_LOG_SIZE) check is to avoid
	 * buffer overflow
	 */
	while ((bytes_written < (count - IAXXX_MAX_LOG_SIZE))
					&& logs_to_read > 0) {
		log = reg_dump->log[r_index];
		if (log.op == IAXXX_READ)
			bytes_written += scnprintf(kbuf + bytes_written,
				PAGE_SIZE, "R:\t[%lu.%03lu]\t0x%08x\t0x%08x\n",
				log.timestamp.tv_sec,
				log.timestamp.tv_nsec / (1000*1000),
				iaxxx_conv_physical_to_virtual_register_address
				(iaxxx, log.addr),
				log.val);
		else
			bytes_written += scnprintf(kbuf + bytes_written,
				PAGE_SIZE, "W:\t[%lu.%03lu]\t0x%08x\t0x%08x\n",
				log.timestamp.tv_sec,
				log.timestamp.tv_nsec / (1000*1000),
				iaxxx_conv_physical_to_virtual_register_address
				(iaxxx, log.addr),
				log.val);
		/* Increment read index and align with ring buffer boundary */
		r_index++;
		r_index %= IAXXX_BUF_MAX_LEN;
		/* update to remaining logs to read */
		logs_to_read--;
	}
	spin_unlock(&reg_dump->ring_lock);
	/* Copy data to user buffer */
	if (copy_to_user(buf, kbuf, bytes_written)) {
		kvfree(kbuf);
		return -EFAULT;
	}
	/* If no more logs to read, mark read complete */
	if (!logs_to_read)
		done = true;
	kvfree(kbuf);
	return bytes_written;
}

static int regdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("%s()\n", __func__);

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode\n");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, regdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch register dump private data\n");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int regdump_release(struct inode *inode, struct file *filp)
{
	pr_debug("%s() called\n", __func__);

	filp->private_data = NULL;
	return 0;
}

static int crashdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("%s()\n", __func__);

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode\n");
		return -ENODEV;
	}
	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, crashdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch crash dump private data\n");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int crashdump_release(struct inode *inode, struct file *filp)
{

	filp->private_data = NULL;
	return 0;
}

static ssize_t crashdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)filp->private_data;
	ssize_t size;

	dev_dbg(priv->dev, "%s() called\n", __func__);

	if (!priv)
		return -EINVAL;

	if (!priv->crashlog)
		return -EINVAL;
	if (!priv->crashlog->log_buffer)
		return -EINVAL;
	mutex_lock(&priv->crashdump_lock);
	/* Read done */
	if (priv->crashlog->logs_read == priv->crashlog->log_buffer_size) {
		priv->crashlog->logs_read = 0;
		mutex_unlock(&priv->crashdump_lock);
		return 0;
	}

	if (priv->crashlog->logs_read + count > priv->crashlog->log_buffer_size)
		size = priv->crashlog->log_buffer_size
			- priv->crashlog->logs_read;
	else
		size = count;
	/* Copy data to user buffer */
	if (copy_to_user(buf, priv->crashlog->log_buffer
				+ priv->crashlog->logs_read, size)) {
		mutex_unlock(&priv->crashdump_lock);
		return -EFAULT;
	}
	priv->crashlog->logs_read += size;
	mutex_unlock(&priv->crashdump_lock);

	return size;
}

static const struct file_operations raw_fops = {
	.owner = THIS_MODULE,
	.open = raw_open,
	.read = raw_read,
	.write = raw_write,
	.unlocked_ioctl = raw_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = raw_compat_ioctl,
#endif
	.release = raw_release,
};

static const struct file_operations regdump_fops = {
	.owner = THIS_MODULE,
	.open = regdump_open,
	.read = regdump_read,
	.write = regdump_write,
	.release = regdump_release,
};

static const struct file_operations crashdump_fops = {
	.owner = THIS_MODULE,
	.open = crashdump_open,
	.read = crashdump_read,
	.release = crashdump_release,
};

int iaxxx_debug_init(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = NULL;
	int err;

	pr_debug("%s: initializing debug interface\n", __func__);

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer\n");
		return -EINVAL;
	}

	intf_priv = devm_kzalloc(priv->dev, sizeof(*intf_priv), GFP_KERNEL);
	if (!intf_priv)
		return -ENOMEM;

	priv->intf_priv = intf_priv;
	intf_priv->priv = priv;

	err = iaxxx_cdev_create(&intf_priv->raw_cdev, priv->dev,
		&raw_fops, intf_priv, IAXXX_CDEV_DEBUG);
	if (err) {
		pr_err("%s() error in creating the raw char device\n",
				__func__);
		err = -EIO;
		goto raw_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->regdump_cdev, priv->dev,
		&regdump_fops, intf_priv, IAXXX_CDEV_REGDUMP);
	if (err) {
		pr_err("error in creating the char device\n");
		err = -EIO;
		goto regdump_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->crashdump_cdev, priv->dev,
		&crashdump_fops, intf_priv, IAXXX_CDEV_CRASHDUMP);
	if (err) {
		pr_err("error in creating the char device\n");
		err = -EIO;
		goto crashdump_cdev_err;
	}

	priv->raw_ops = kvmalloc(sizeof(struct iaxxx_raw_bus_ops), GFP_KERNEL);
	if (!priv->raw_ops) {
		err = -ENOMEM;
		goto raw_mem_alloc_err;
	}
	return err;
raw_mem_alloc_err:
	iaxxx_cdev_destroy(&intf_priv->crashdump_cdev);
crashdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev);
regdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->raw_cdev);
raw_cdev_err:

	devm_kfree(priv->dev, intf_priv);
	priv->intf_priv = NULL;
	return err;
}
EXPORT_SYMBOL(iaxxx_debug_init);

int iaxxx_debug_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = NULL;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer\n");
		return -EINVAL;
	}
	intf_priv = (struct iaxxx_debug_data *) priv->intf_priv;
	iaxxx_cdev_destroy(&intf_priv->raw_cdev);
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev);
	iaxxx_cdev_destroy(&intf_priv->crashdump_cdev);
	devm_kfree(priv->dev, intf_priv);

	kvfree(priv->raw_ops);

	return 0;
}
EXPORT_SYMBOL(iaxxx_debug_exit);
