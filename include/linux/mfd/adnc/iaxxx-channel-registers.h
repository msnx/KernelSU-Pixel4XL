/*
 * iaxxx-channel-registers.h
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

#ifndef __IAXXX_CHANNEL_REGISTER_H__
#define __IAXXX_CHANNEL_REGISTER_H__

#include "iaxxx-register-defs-channel-header.h"
#include "iaxxx-register-defs-in-channel-group.h"
#include "iaxxx-register-defs-out-channel-group.h"
#include "iaxxx-register-defs-in-endpoint-group.h"
#include "iaxxx-register-defs-out-endpoint-group.h"

/* Input Channels */
#define IAXXX_IN_CH_GRP_CH_CTRL_REG(I) \
	(IAXXX_IN_CH_GRP_CH_CTRL_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(I) \
	(IAXXX_IN_CH_GRP_CH_GAIN_CTRL_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_GAIN_STATUS_REG(I) \
	(IAXXX_IN_CH_GRP_CH_GAIN_STATUS_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_PORT_REG(I) \
	(IAXXX_IN_CH_GRP_CH_PORT_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_OUT_FMT_REG(I) \
	(IAXXX_IN_CH_GRP_OUT_FMT_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_PEAK_REG(I) \
	(IAXXX_IN_CH_GRP_CH_PEAK_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_RMS_REG(I) \
	(IAXXX_IN_CH_GRP_CH_RMS_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_MTR_SMPL_REG(I) \
	(IAXXX_IN_CH_GRP_CH_MTR_SMPL_ADDR + (4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_NSENT_REG(I) \
	(IAXXX_IN_CH_GRP_CH_NSENT_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_NRECVD_REG(I) \
	(IAXXX_IN_CH_GRP_CH_NRECVD_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_ENDPOINT_REG(I) \
	(IAXXX_IN_CH_GRP_CH_ENDPOINT_STATE_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_INTR_CNT_REG(I) \
	(IAXXX_IN_CH_GRP_CH_INTR_CNT_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

#define IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(I) \
	(IAXXX_IN_CH_GRP_CH_DROP_CNT_ADDR + \
	(4 * (I) * IAXXX_IN_CH_GRP_REG_NUM))

/* Output Channels */
#define IAXXX_OUT_CH_GRP_REGS(I) \
	(IAXXX_OUT_CH_GRP_REGS_ADDR + (4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_CTRL_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_CTRL_ADDR + (4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_GAIN_STATUS_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_GAIN_STATUS_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_PORT_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_PORT_ADDR + (4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_IN_CONNECT_REG(I) \
	(IAXXX_OUT_CH_GRP_IN_CONNECT_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_PEAK_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_PEAK_ADDR + (4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_RMS_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_RMS_ADDR + (4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_MTR_SMPL_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_MTR_SMPL_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_NSENT_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_NSENT_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_NRECVD_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_NRECVD_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_ENDPOINT_STATE_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_ENDPOINT_STATE_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_INTR_CNT_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_INTR_CNT_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_OUT_CH_GRP_CH_DROP_CNT_REG(I) \
	(IAXXX_OUT_CH_GRP_CH_DROP_CNT_ADDR + \
	(4 * (I) * IAXXX_OUT_CH_GRP_REG_NUM))

#define IAXXX_CH_HDR_REG_OFFSET(reg) \
	(((reg) - IAXXX_CH_HDR_REGS_ADDR) / sizeof(uint32_t))

#define IAXXX_OUT_CH_GRP_REG_OFFSET(reg) \
	(((reg) - IAXXX_OUT_CH_GRP_REGS_ADDR) / sizeof(uint32_t))

#endif /* __IAXXX_CHANNEL_REGISTER_H__ */
