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

#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "airbrush-spi.h"

#define AB_SPI_MAX_BURST_SIZE	0x2000
#define AB_SPI_RESPONSE_TIMEOUT	1000

//#define DEBUG
#define WAIT_FOR_RESPONSE()	send_dummy_byte()

struct spi_device *abc_spi_dev;

struct packet_header {
	uint32_t command;
	uint8_t granularity;
	uint64_t base_address;
	uint16_t length;
};

/* Internal data structure mainly for buffer allocation */
struct airbrush_spi_client {
	struct spi_device *spi_device;

	uint8_t *tx_buff_onetime;
	uint8_t *rx_buff_onetime;

	uint8_t *tx_buff_for_cmd;
	uint8_t *rx_buff_for_cmd;

	size_t buff_len_onetime;
	size_t buff_len_for_cmd;
};

int airbrush_spi_command_code[] = {
	[AB_SPI_CMD_FSM_READ_SINGLE]	= 0x02,
	[AB_SPI_CMD_FSM_WRITE_SINGLE]	= 0x03,
	[AB_SPI_CMD_FSM_BURST_WRITE]	= 0x04,
	[AB_SPI_CMD_READ_SINGLE]	= 0x03,
	[AB_SPI_CMD_WRITE_SINGLE]	= 0x02,
	[AB_SPI_CMD_READ_RANGE]		= 0x0B,
	[AB_SPI_CMD_WRITE_RANGE]	= 0x0A,
	[AB_SPI_CMD_MEMSET_RANGE]	= 0x1A,
	[AB_SPI_CMD_BIT_SET]		= 0xC1,
	[AB_SPI_CMD_BIT_CLEAR]		= 0xC0,
	[AB_SPI_CMD_BIT_CLEAR_SET]	= 0xC2,
	[AB_SPI_CMD_READ_VER]		= 0x9E,
	[AB_SPI_CMD_READ_ID]		= 0x9F,
	[AB_SPI_CMD_READ_STATUS]	= 0x01,
	[AB_SPI_CMD_GO_BOOT]		= 0xA0,
};

/* Table of command codes and acceptable granularity flags */
static int granularity_table[][2] = {
	{ AB_SPI_CMD_FSM_READ_SINGLE,	FOUR_BYTE},
	{ AB_SPI_CMD_FSM_WRITE_SINGLE,	FOUR_BYTE},
	{ AB_SPI_CMD_FSM_BURST_WRITE,	FOUR_BYTE},
	{ AB_SPI_CMD_READ_SINGLE,	FOUR_BYTE},
	{ AB_SPI_CMD_WRITE_SINGLE,	FOUR_BYTE},
	{ AB_SPI_CMD_READ_RANGE,	FOUR_BYTE},
	{ AB_SPI_CMD_WRITE_RANGE,	FOUR_BYTE},
	{ AB_SPI_CMD_MEMSET_RANGE,	FOUR_BYTE},
	{ AB_SPI_CMD_BIT_SET,		FOUR_BYTE},
	{ AB_SPI_CMD_BIT_CLEAR,		FOUR_BYTE | EIGHT_BYTE },
	{ AB_SPI_CMD_BIT_CLEAR_SET,	FOUR_BYTE | EIGHT_BYTE },
	{ AB_SPI_CMD_READ_VER,		DONT_CARE},
	{ AB_SPI_CMD_READ_ID,		DONT_CARE},
	{ AB_SPI_CMD_READ_STATUS,	DONT_CARE},
	{ AB_SPI_CMD_GO_BOOT,		DONT_CARE},
};

static void airbrush_spi_send(void *tx_data, void *rx_data, uint16_t _len)
{
#ifdef DEBUG
	int i;
#endif
	struct spi_transfer t = {
			.tx_buf = tx_data,
			.rx_buf = rx_data,
			.len = _len,
		};
	struct spi_message message;

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

#ifdef DEBUG
	for (i = 0 ; i < _len ; i++)
		printk("In %s : %x\n", __func__, *((uint8_t *)tx_data + i));
#endif
	spi_sync(abc_spi_dev, &message);
}

static int is_valid_packet(struct airbrush_spi_packet *pkt)
{
	if (pkt->command < AB_SPI_CMD_MAX)
		if (pkt->granularity & granularity_table[pkt->command][1])
			return 1;
	return 0;
}


static bool send_to_fsm(struct airbrush_spi_packet *pkt, char *tx_buff,
		char *rx_buff)
{
	uint16_t _itr;
	uint32_t command = airbrush_spi_command_code[pkt->command];
	uint32_t base_address = pkt->base_address;

	/* for Burst write first byte must be length */
	if (pkt->command == AB_SPI_CMD_FSM_BURST_WRITE) {

		/* In FSM for n-1 length n writes happen */
		if (pkt->data_length == 0)
			return false;
		pkt->data_length--;
		tx_buff[0] = *((uint8_t *)&pkt->data_length + 1);
		tx_buff[1] = *((uint8_t *)&pkt->data_length);
		pkt->data_length++; /* restore */

	} else {
		/* first 2 bytes has to be 0 for read and write single */
		tx_buff[0] = 0x0;
		tx_buff[1] = 0x0;
	}

	tx_buff[2] = *((uint8_t *)&command + 1);
	tx_buff[3] = *((uint8_t *)&command);

	/* Address phase */
	tx_buff[4] = *((uint8_t *)&base_address + 3);
	tx_buff[5] = *((uint8_t *)&base_address + 2);
	tx_buff[6] = *((uint8_t *)&base_address + 1);
	tx_buff[7] = *((uint8_t *)&base_address);

	if (pkt->command == AB_SPI_CMD_FSM_READ_SINGLE) {
		airbrush_spi_send(tx_buff, rx_buff, 8);
		return true;
	}
	/* Data phase */
	tx_buff[8] = *((uint8_t *)pkt->data + 3);
	tx_buff[9] = *((uint8_t *)pkt->data + 2);
	tx_buff[10] = *((uint8_t *)pkt->data + 1);
	tx_buff[11] = *((uint8_t *)pkt->data);

	airbrush_spi_send(tx_buff, rx_buff, 12);

	if (pkt->command == AB_SPI_CMD_FSM_WRITE_SINGLE)/* write is done */
		return true;

	/* send the remaining n-1 data */
	for (_itr = 1; _itr < pkt->data_length; _itr++) {
		tx_buff[0] = *((uint8_t *)pkt->data + 4 * _itr + 3);
		tx_buff[1] = *((uint8_t *)pkt->data + 4 * _itr + 2);
		tx_buff[2] = *((uint8_t *)pkt->data + 4 * _itr + 1);
		tx_buff[3] = *((uint8_t *)pkt->data + 4 * _itr);
#ifdef DEBUG
		printk("0x%x 0x%x 0x%x 0x%x\n",
			tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3]);
#endif
		airbrush_spi_send(tx_buff, rx_buff, 4);
	}

	return true;
}

/*
 * Send a byte with 0x00 and read one byte.
 * Returns read value.
 */
static uint8_t send_dummy_byte(void)
{
	struct airbrush_spi_client *client;
	uint8_t *tx_buff;
	uint8_t *rx_buff;

	client = spi_get_drvdata(abc_spi_dev);
	tx_buff = client->tx_buff_onetime;
	rx_buff = client->rx_buff_onetime;

	*tx_buff = 0x00;
	airbrush_spi_send(tx_buff, rx_buff, 1);
	return *rx_buff;
}

int airbrush_spi_run_cmd(struct airbrush_spi_packet *pkt)
{
	struct airbrush_spi_client *client;
	uint8_t *tx_buff;
	uint8_t *rx_buff;
	uint32_t *result = pkt->data;
	uint32_t timeout = AB_SPI_RESPONSE_TIMEOUT;

	client = spi_get_drvdata(abc_spi_dev);
	tx_buff = client->tx_buff_for_cmd;
	rx_buff = client->rx_buff_for_cmd;

	/* validate packet */
	if (!is_valid_packet(pkt))
		return -EINVAL;

	/* max size can be only of 32kb */
	if (pkt->command == AB_SPI_CMD_FSM_BURST_WRITE &&
			pkt->data_length > AB_SPI_MAX_BURST_SIZE)
		return -EINVAL;

	/*check if it FSM or non FSM */
	if (!((pkt->command == AB_SPI_CMD_FSM_READ_SINGLE) ||
	      (pkt->command == AB_SPI_CMD_FSM_WRITE_SINGLE) ||
	      (pkt->command == AB_SPI_CMD_FSM_BURST_WRITE)))
		return 0;

	if (send_to_fsm(pkt, tx_buff, rx_buff) == false)
		return -EIO;

	if (pkt->command == AB_SPI_CMD_FSM_READ_SINGLE) {

		while ((WAIT_FOR_RESPONSE() != 0x1) && (--timeout > 0))
			;

		if (timeout == 0) {
			pr_err("%s: Timeout waiting for SPI response\n",
					__func__);
			return -EINVAL;
		}

		/* read response data */
		rx_buff[0] = send_dummy_byte();
		rx_buff[1] = send_dummy_byte();
		rx_buff[2] = send_dummy_byte();
		rx_buff[3] = send_dummy_byte();
		if (pkt->data) {
			*((uint8_t *)result) =  rx_buff[3];
			*((uint8_t *)result + 1) =  rx_buff[2];
			*((uint8_t *)result + 2) =  rx_buff[1];
			*((uint8_t *)result + 3) =  rx_buff[0];
		}
	}

	return 0;
}
EXPORT_SYMBOL(airbrush_spi_run_cmd);

static const struct of_device_id airbrush_spi_of_match[] = {
	{ .compatible = "abc,airbrush-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, airbrush_spi_of_match);

/*
 * Allocate Resource-managed memory as SPI buffer.
 *
 * Returns 0 on success, -ENOMEM on failure.
 */
static int airbrush_spi_client_alloc_buf(struct airbrush_spi_client *client)
{
	client->buff_len_onetime = 1;  /* single read/write */
	client->buff_len_for_cmd = 16; /* sufficient for a cmd */

	/* Allocate for both tx and rx */
	client->tx_buff_onetime = devm_kmalloc(
					&client->spi_device->dev,
					2 * client->buff_len_onetime,
					GFP_KERNEL | GFP_DMA);
	if (client->tx_buff_onetime == NULL)
		return -ENOMEM;
	client->rx_buff_onetime = client->tx_buff_onetime +
					client->buff_len_onetime;


	/* Allocate for both tx and rx */
	client->tx_buff_for_cmd = devm_kmalloc(
					&client->spi_device->dev,
					2 * client->buff_len_for_cmd,
					GFP_KERNEL | GFP_DMA);
	if (client->tx_buff_for_cmd == NULL)
		return -ENOMEM;
	client->rx_buff_for_cmd = client->tx_buff_onetime +
					client->buff_len_for_cmd;

	return 0;
}

static int airbrush_spi_probe(struct spi_device *spi)
{
	struct airbrush_spi_client *client;
	int ret;

	abc_spi_dev = spi;

	client = devm_kzalloc(&abc_spi_dev->dev,
			      sizeof(struct airbrush_spi_client),
			      GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->spi_device = spi;

	ret = airbrush_spi_client_alloc_buf(client);
	if (ret)
		return ret;

	spi_set_drvdata(spi, client);

	return 0;
}

static int airbrush_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver airbrush_spi_driver = {
	.driver = {
		.name		= "abc,airbrush-spi",
		.of_match_table = airbrush_spi_of_match,
	},
	.probe	= airbrush_spi_probe,
	.remove = airbrush_spi_remove,
};

module_spi_driver(airbrush_spi_driver);

MODULE_DESCRIPTION("Airbrush SPI driver");
MODULE_LICENSE("GPL");
