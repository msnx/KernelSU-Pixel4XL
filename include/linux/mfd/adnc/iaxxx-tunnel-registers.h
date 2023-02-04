/*
 * iaxxx-tunnel-registers.h
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

#ifndef __IAXXX_TUNNEL_REGISTER_H__
#define __IAXXX_TUNNEL_REGISTER_H__

#include "iaxxx-register-defs-tunnel-header.h"
#include "iaxxx-register-defs-in-tunnel-group.h"
#include "iaxxx-register-defs-out-tunnel-group.h"

#define IAXXX_IN_TNL_GRP_REGS(I)  \
	(IAXXX_IN_TNL_GRP_REGS_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_CTRL_REG(I)  \
	(IAXXX_IN_TNL_GRP_TNL_CTRL_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_FORMAT_REG(I)   \
	(IAXXX_IN_TNL_GRP_FORMAT_ADDR +  \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(I)  \
	(IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_REG(I)  \
	(IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_NSENT_REG(I)  \
	(IAXXX_IN_TNL_GRP_TNL_NSENT_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_NRECVD_REG(I)  \
	(IAXXX_IN_TNL_GRP_TNL_NRECVD_ADDR + \
	(4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_REGS(I)  \
	(IAXXX_OUT_TNL_GRP_REGS_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_CTRL_REG(I)  \
	(IAXXX_OUT_TNL_GRP_TNL_CTRL_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_CONNECT_REG(I)   \
	(IAXXX_OUT_TNL_GRP_CONNECT_ADDR +  \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_REG(I)  \
	(IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_NSENT_TO_HOST_REG(I)  \
	(IAXXX_OUT_TNL_GRP_TNL_NSENT_TO_HOST_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_NSENT_REG(I)  \
	(IAXXX_OUT_TNL_GRP_TNL_NSENT_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_NRECVD_REG(I)  \
	(IAXXX_OUT_TNL_GRP_TNL_NRECVD_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_TNL_HDR_REG_OFFSET(reg) \
	(((reg) - IAXXX_TNL_HDR_REGS_ADDR) / sizeof(uint32_t))

#define IAXXX_OUT_TNL_GRP_REG_OFFSET(reg) \
	(((reg) - IAXXX_OUT_TNL_GRP_REGS_ADDR) / sizeof(uint32_t))

#endif /* __IAXXX_TUNNEL_REGISTER_H__ */
