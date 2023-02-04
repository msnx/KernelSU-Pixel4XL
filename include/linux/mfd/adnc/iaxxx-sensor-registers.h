/*
 * iaxxx-sensor-registers.h
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

#ifndef __IAXXX_SENSOR_REGISTER_H__
#define __IAXXX_SENSOR_REGISTER_H__

#include "iaxxx-register-defs-sensor-header.h"
#include "iaxxx-register-defs-sensor-group.h"

#define IAXXX_SENSOR_GRP_PARAM_ID_REG(I) \
	(IAXXX_SENSOR_GRP_PARAM_ID_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_PARAM_REG(I) \
	(IAXXX_SENSOR_GRP_PARAM_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_CONNECT_REG(I) \
	(IAXXX_SENSOR_GRP_CONNECT_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_SENSOR_NSENT_REG(I) \
	(IAXXX_SENSOR_GRP_SENSOR_NSENT_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_SENSOR_ENDPOINT_STATE_REG(I) \
	(IAXXX_SENSOR_GRP_SENSOR_ENDPOINT_STATE_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_SENSOR_DROP_CNT_REG(I) \
	(IAXXX_SENSOR_GRP_SENSOR_DROP_CNT_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_ADDR_REG(I) \
	(IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_ADDR_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))

#define IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_SIZE_REG(I) \
	(IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_SIZE_ADDR +	\
	(4 * (I) * IAXXX_SENSOR_GRP_REG_NUM))


#endif /* __IAXXX_SENSOR_REGISTER_H__ */
