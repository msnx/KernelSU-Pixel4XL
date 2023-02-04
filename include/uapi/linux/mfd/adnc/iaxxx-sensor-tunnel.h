/*
 * iaxxx-sensor-tunnel.h  --  IAXXX flicker sensor  header file
 *
 * Copyright 2019 Knowles, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_SENSOR_TUNNEL_H__
#define __IAXXX_SENSOR_TUNNEL_H__

/* IOCTL Magic character */
#define IAXXX_SENSOR_TUNNEL_IOCTL_MAGIC 'K'

/* Create IOCTL */
#define FLICKER_TUNNEL_SETUP _IOWR(IAXXX_SENSOR_TUNNEL_IOCTL_MAGIC, \
		0x022, struct tunlMsg)
#define FLICKER_TUNNEL_TERMINATE _IOWR(IAXXX_SENSOR_TUNNEL_IOCTL_MAGIC, \
		0x023, struct tunlMsg)
#define FLICKER_ROUTE_SETUP _IO(IAXXX_SENSOR_TUNNEL_IOCTL_MAGIC, 0x24)
#define FLICKER_ROUTE_TERMINATE _IO(IAXXX_SENSOR_TUNNEL_IOCTL_MAGIC, 25)

#endif /* __IAXXX_SENSOR_TUNNEL_H__ */
