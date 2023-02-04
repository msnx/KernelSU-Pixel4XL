/*
 * iaxxx-stream-registers.h
 *
 * Copyright 2017 Knowles Corporation
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

#ifndef __IAXXX_STREAM_REGISTER_H__
#define __IAXXX_STREAM_REGISTER_H__

#include "iaxxx-register-defs-stream-header.h"
#include "iaxxx-register-defs-stream-group.h"

#define IAXXX_STR_GRP_REGS(I) \
	(IAXXX_STR_GRP_REGS_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_CTRL_REG(I) \
	(IAXXX_STR_GRP_STR_CTRL_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_STATUS_REG(I) \
	(IAXXX_STR_GRP_STR_STATUS_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_FORMAT_REG(I) \
	(IAXXX_STR_GRP_STR_FORMAT_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_PORT_REG(I) \
	(IAXXX_STR_GRP_STR_PORT_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_CHN_REG(I) \
	(IAXXX_STR_GRP_STR_CHN_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_SYNC_REG(I) \
	(IAXXX_STR_GRP_STR_SYNC_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_AFS_FIFO_OVERFLOW_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_AFS_FIFO_OVERFLOW_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_AFS_FIFO_UNDERFLOW_CNT_REG(I)   \
	(IAXXX_STR_GRP_STR_AF_ERR_AFS_FIFO_UNDERFLOW_CNT_ADDR +  \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_TUS_FIFO_OVERFLOW_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_OVERFLOW_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_TUS_FIFO_UNDERFLOW_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_UNDERFLOW_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_TUS_FIFO_COHERENCY_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_TUS_FIFO_COHERENCY_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_DEADLINE_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_DEADLINE_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_PHY_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_PHY_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_TIMEOUT_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_TIMEOUT_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_AF_ERROR_ACCESS_CNT_REG(I)    \
	(IAXXX_STR_GRP_STR_AF_ERR_ACCESS_CNT_ADDR +   \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_PHY_LOGICAL_CH_MAPPING_REG(I) \
	(IAXXX_STR_GRP_STR_PHY_LOGICAL_CH_MAPPING_ADDR + \
	(4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_HDR_REG_OFFSET(reg) \
	(((reg) - IAXXX_STR_HDR_REGS_ADDR) / sizeof(uint32_t))

#define IAXXX_STR_GRP_REG_OFFSET(reg) \
	(((reg) - IAXXX_STR_GRP_REGS_ADDR) / sizeof(uint32_t))

#endif /* __IAXXX_STREAM_REGISTER_H__ */
