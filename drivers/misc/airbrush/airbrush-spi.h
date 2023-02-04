/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Surendran K <surendran.k@samsung.com>
 *	    Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush SPI Slave driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_SPI_H
#define _AIRBRUSH_SPI_H

#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/of.h>


#define DONT_CARE	(0)
#define ONE_BYTE	(1 << 3)
#define TWO_BYTE	(1 << 4)
#define FOUR_BYTE	(1 << 5)
#define EIGHT_BYTE	(1 << 6)

enum airbrush_spi_command {
	AB_SPI_CMD_FSM_READ_SINGLE,
	AB_SPI_CMD_FSM_WRITE_SINGLE,
	AB_SPI_CMD_FSM_BURST_WRITE,
	AB_SPI_CMD_READ_SINGLE,
	AB_SPI_CMD_WRITE_SINGLE,
	AB_SPI_CMD_READ_RANGE,
	AB_SPI_CMD_WRITE_RANGE,
	AB_SPI_CMD_MEMSET_RANGE,
	AB_SPI_CMD_BIT_SET,
	AB_SPI_CMD_BIT_CLEAR,
	AB_SPI_CMD_BIT_CLEAR_SET,
	AB_SPI_CMD_READ_VER,
	AB_SPI_CMD_READ_ID,
	AB_SPI_CMD_READ_STATUS,
	AB_SPI_CMD_GO_BOOT,
	AB_SPI_CMD_MAX,
};

struct airbrush_spi_packet {
	uint32_t	command;
	uint8_t		granularity;
	uint64_t	base_address;
	uint16_t	data_length;
	uint32_t	*data;
};


int airbrush_spi_run_cmd(struct airbrush_spi_packet *pkt);

#endif /* _AIRBRUSH_SPI_H */
