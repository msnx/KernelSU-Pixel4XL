/*
 * iaxxx-spi.c -- SPI driver for Knowles IAxxx device
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "ia8508a-memory-map.h"
#include "iaxxx.h"


#define IAXXX_SYNC_RETRY	15

/* spi large reads are failing and currently
 * our code request max read as 8k bytes
 * which translates to 2048 words
 */
#define IAXXX_SPI_MAX_SIZE	((128*1024) + IAXXX_REG_LEN_WITH_PADDING)
#define IAXXX_SPI_PACKET_LEN	2048
#define IAXXX_REG_LEN		4
/* Padding is required to give time FW ready for data */
#define IAXXX_REG_PADDING	12
#define IAXXX_REG_LEN_WITH_PADDING (IAXXX_REG_LEN + IAXXX_REG_PADDING)

/* Burst size used when sending raw data to firmware */
#define IAXXX_SPI_BURST_SIZE  (32*1024)

/**
 * Description of driver private data
 *
 * @priv: IAxxx private data
 * @spi:  spi client pointer
 */
struct iaxxx_spi_priv {
	struct iaxxx_priv priv;	/* private data */
	struct spi_device *spi;
	struct mutex spi_mutex;
	uint8_t *rx_buf;
	uint8_t *tx_buf;
};

static inline struct iaxxx_spi_priv *to_spi_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_spi_priv, priv) : NULL;
}

/* Register address passed to this function is in BE order.
 * The array of data in valaddr can be in BE or CPU order based
 * on the flag passed.
 */
static void register_dump_log(struct device *dev,
			const uint32_t st_regaddr, const uint32_t *st_valaddr,
			const bool data_in_be32_fmt,
			const uint32_t no_words, bool op)
{
	int i;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t st_regaddr_cpuorder = be32_to_cpu(st_regaddr);

	if (priv->dump_log) {
		for (i = 0; i < no_words; i++)
			register_transac_log(dev,
				st_regaddr_cpuorder + (sizeof(uint32_t) * i),
				data_in_be32_fmt ? be32_to_cpu(st_valaddr[i]) :
				st_valaddr[i], op);
	}
}

static int iaxxx_spi_read(struct spi_device *spi, void *buf, int len)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(&spi->dev);
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);
	int rc;
	void *dmabuf;

	mutex_lock(&spi_priv->spi_mutex);
	memset(spi_priv->rx_buf, 0, IAXXX_SPI_MAX_SIZE);

	if (len > IAXXX_SPI_MAX_SIZE) {
		pr_err("%s() msg_len excess MAX!\n", __func__);
		rc = -EINVAL;
		goto err;
	}

	dmabuf = spi_priv->rx_buf;
	if (!dmabuf) {
		pr_err("%s: allocate DMA buffer failed\n", __func__);
		rc = -ENOMEM;
		goto err;
	}

	rc = spi_read(spi, dmabuf, len);
	memcpy(buf, dmabuf, len);
	if (rc < 0) {
		dev_err(&spi->dev, "spi_read() failed, rc = %d\n", rc);
		goto err;
	}

err:
	mutex_unlock(&spi_priv->spi_mutex);
	return rc;
}

static int iaxxx_spi_write(struct spi_device *spi, const void *buf, int len)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(&spi->dev);
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);
	int rc;
	void *dmabuf;

	mutex_lock(&spi_priv->spi_mutex);
	memset(spi_priv->tx_buf, 0, IAXXX_SPI_MAX_SIZE);

	if (len > IAXXX_SPI_MAX_SIZE) {
		pr_err("%s() msg_len excess MAX!\n", __func__);
		rc = -EINVAL;
		goto err;
	}

	dmabuf = spi_priv->tx_buf;
	if (!dmabuf) {
		pr_err("%s: allocate DMA buffer failed\n", __func__);
		rc = -ENOMEM;
		goto err;
	}

	memcpy(dmabuf, buf, len);
	rc = spi_write(spi, dmabuf, len);
	if (rc < 0) {
		dev_err(&spi->dev, "spi_write() failed, rc = %d\n", rc);
		goto err;
	}

err:
	mutex_unlock(&spi_priv->spi_mutex);
	return rc;
}

/* send command to firmware and read response */
static int iaxxx_spi_cmd(struct spi_device *spi, u32 cmd, u32 *resp)
{
	int ret;

	if (!spi) {
		pr_err("%s: NULL input pointer(s)\n", __func__);
		return -EINVAL;
	}

	ret = iaxxx_pm_get_sync(&spi->dev);
	if (ret < 0)
		goto out;

	cmd = cpu_to_be32(cmd);
	dev_dbg(&spi->dev, "iaxxx: cmd = 0x%08x\n", cmd);

	ret = iaxxx_spi_write(spi, &cmd, sizeof(cmd));
	if (ret) {
		dev_err(&spi->dev, "Failed to send command 0x%.08X\n", cmd);
		goto out;
	}

	if (resp) {
		usleep_range(4000, 4500);
		ret = iaxxx_spi_read(spi, resp, sizeof(*resp));
		if (ret) {
			dev_err(&spi->dev, "Failed to read command response\n");
			goto out;
		}
		*resp = be32_to_cpu(*resp);
	}

out:
	iaxxx_pm_put_autosuspend(&spi->dev);
	return ret;
}

static int iaxxx_copy_cpu_to_be32(struct device *dev,
				void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;
	uint32_t align_len = len;

	if (len % 4) {
		align_len = len + (len % 4);
		dev_warn(dev, "Not aligned 32bit boundary: %d aligned len %d\n",
				len, align_len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = cpu_to_be32(data);
	}

	return align_len;
}

/* This is a common function used by all public spi write functions.
 *
 * This function sends data over SPI to firmware with the option to
 * convert the data to BE32. The option says the data is NOT in Be32Fmt then
 * data would converted before writing. Similarly, if the option is set, the
 * register address is assumed to be in BE32 format.
 *
 */
static int iaxxx_spi_write_common(void *context,
				const void *reg, const size_t reg_len,
				const void *val, const size_t val_len,
				const bool data_in_be32_fmt,
				bool pm_needed)
{
	int rc;
	size_t reg_len_new;
	size_t val_len_new = val_len;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);
	struct spi_message m;
	struct spi_transfer t[1] = { };
	uint8_t *padding;
	uint32_t reg_addr = (*(uint32_t *)reg);

	pr_debug("%s() Register address %x\n", __func__, reg_addr);

	if (pm_needed) {
		rc = iaxxx_pm_get_sync(&spi->dev);
		if (rc < 0)
			goto get_sync_err;
	}

	mutex_lock(&spi_priv->spi_mutex);
	memset(spi_priv->tx_buf, 0, IAXXX_SPI_MAX_SIZE);

	if ((val_len + IAXXX_REG_LEN_WITH_PADDING) > IAXXX_SPI_MAX_SIZE) {
		rc = -EINVAL;
		pr_err("%s() val_len %d excess MAX!\n", __func__, (int)val_len);
		goto err;
	}

	padding = spi_priv->tx_buf;

	if (data_in_be32_fmt)
		reg_addr = be32_to_cpu(reg_addr);

	/* Device protocol requires address to be shifted by one bit */
	reg_addr = cpu_to_be32(reg_addr >> 1);

	spi_message_init(&m);

	reg_len_new = (reg_len > IAXXX_REG_LEN) ? IAXXX_REG_LEN_WITH_PADDING :
			IAXXX_REG_LEN;

	memcpy(padding, &reg_addr, IAXXX_REG_LEN);
	memcpy(padding + reg_len_new, val, val_len);
	if (!data_in_be32_fmt)
		val_len_new = iaxxx_copy_cpu_to_be32(dev, padding +
				reg_len_new, val_len);
	/* Register address */
	t[0].len = reg_len_new + val_len_new;
	t[0].tx_buf = padding;
	spi_message_add_tail(&t[0], &m);

	rc = spi_sync(spi, &m);

err:
	mutex_unlock(&spi_priv->spi_mutex);
get_sync_err:
	if (pm_needed)
		iaxxx_pm_put_autosuspend(&spi->dev);
	return rc;
}

/* This spi-write function is regmap interface function. It is called
 * from regmap which is configured to send data in Big-Endian.
 * Since SPI communication with firmware is Big-Endian, there is no
 * conversion needed before sending data.
 */
static int iaxxx_regmap_spi_gather_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	/* flag set to indicate data and reg-address are in BE32 format */
	return iaxxx_spi_write_common(context,
				 reg, reg_len,
				 val, val_len,
				 true, true);

}

/* This version of spi-write is called by regmap. Since regmap is configured
 * for Big-endian, the data received here will be in Big-endian format.
 * Since SPI communication with Firmware is Big-endian, no byte-order
 * conversion is needed before sending data.
 */
static int iaxxx_regmap_spi_write(void *context, const void *data, size_t count)
{
	const void *val = data + sizeof(uint32_t);
	size_t reg_len = sizeof(uint32_t);
	size_t val_len = count - reg_len;
	struct device *dev = context;
	uint32_t *writebuf = (uint32_t *)((uint8_t *)val + IAXXX_REG_PADDING);
	uint32_t reg_addr = *(uint32_t *)data;
	uint32_t words = (val_len - IAXXX_REG_PADDING) / sizeof(uint32_t);
	int rc;

	if (WARN_ON(count <= sizeof(uint32_t))
		|| WARN_ON(val_len <= IAXXX_REG_PADDING)) {
		pr_err("%s(), Error Input param put of range\n", __func__);
		return -EINVAL;
	}

	/* flag set to indicate data and reg-address are in BE32 format */
	rc = iaxxx_spi_write_common(context, data, reg_len,
			val, val_len, true, true);

	register_dump_log(dev, reg_addr, writebuf, true, words, IAXXX_WRITE);
	return rc;
}

/* This version of spi-write is used privately across the driver to write
 * directly to firmware's memory. Since data received here will be in
 * CPU byte order, the data would need to be converted to Big-endian
 * before sending to firmware over SPI.
 *
 * Also this function splits big buffers into burst size defined by
 * IAXXX_SPI_BURST_SIZE and sends to firmware due to limitations
 * in some SPI master drivers handling big buffers size.
 *
 */
static int iaxxx_spi_raw_write(void *context,
			 uint32_t reg_addr,
			 const void *val, size_t val_len)
{
	size_t reg_len = IAXXX_REG_LEN_WITH_PADDING;
	size_t val_index;
	size_t burst_size = val_len > IAXXX_SPI_BURST_SIZE ?
			IAXXX_SPI_BURST_SIZE : val_len;
	uint8_t *val_addr = (uint8_t *)val;
	int rc = 0;

	for (val_index = 0; val_index < val_len; val_index += burst_size) {
		/* Flag is false to indicate data and reg-address are
		 * not in BE32 (in CPU byte order)
		 */
		rc = iaxxx_spi_write_common(context,
			&reg_addr, reg_len,
			(val_addr + val_index),
			(val_len - val_index) < burst_size ?
			(val_len - val_index) : burst_size,
			false, true);
		if (rc)
			break;
		if (val_len-val_index < burst_size)
			reg_addr += val_len - val_index;
		else
			reg_addr += burst_size;
	}
	return rc;
}

/* This is the common function used by all public spi read functions.
 * This function assumes that the register address in BE32 format.
 * The data read in the buffer will be in BE32 format. It will be up to
 * the caller to convert the data if needed to CPU byte order.
 */
static int iaxxx_spi_read_common(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len, bool pm_needed)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);
	struct spi_message m;
	struct spi_transfer t[1] = { };
	uint32_t reg2[4] = {0};
	uint8_t *rx_buf;
	uint8_t *tx_buf;
	int rc = 0;
	uint32_t reg_addr = be32_to_cpu(*(uint32_t *)reg);
	size_t msg_len = max(sizeof(reg2),
			     val_len + IAXXX_REG_LEN_WITH_PADDING);

	if (pm_needed) {
		rc = iaxxx_pm_get_sync(&spi->dev);
		if (rc < 0)
			goto get_sync_err;
	}

	mutex_lock(&spi_priv->spi_mutex);
	memset(spi_priv->rx_buf, 0, IAXXX_SPI_MAX_SIZE);
	memset(spi_priv->tx_buf, 0, IAXXX_SPI_MAX_SIZE);

	if (msg_len > IAXXX_SPI_MAX_SIZE) {
		rc = -EINVAL;
		pr_err("%s() msg_len excess MAX!\n", __func__);
		goto err;
	}

	rx_buf = spi_priv->rx_buf;
	tx_buf = spi_priv->tx_buf;

	/* For reads, most significant bit is set after address shifted */
	reg_addr = cpu_to_be32(reg_addr >> 1) | 0x80;
	reg2[0] = reg_addr;
	memcpy(tx_buf, reg2, sizeof(reg2));

	spi_message_init(&m);

	/* Register address */
	t[0].len = msg_len;
	t[0].tx_buf = tx_buf;
	t[0].rx_buf = rx_buf;
	spi_message_add_tail(&t[0], &m);

	rc = spi_sync(spi, &m);
	if (rc)
		goto err;

	memcpy(val, rx_buf + IAXXX_REG_LEN_WITH_PADDING, val_len);

err:
	mutex_unlock(&spi_priv->spi_mutex);
get_sync_err:
	if (pm_needed)
		iaxxx_pm_put_autosuspend(&spi->dev);
	return rc;
}


/* This version of spi-read is called by regmap. Since SPI communication
 * with firmware is in Big-endian and regmap is also configured for
 * Big-endian, the data read here does not need to be altered.
 * Also the register address received here is also in Big-endian format.
 */
static int iaxxx_regmap_spi_read(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	struct device *dev = context;
	uint32_t words = val_len / sizeof(uint32_t);
	int rc = 0;
	uint32_t reg_addr = *(uint32_t *) reg;

	rc = iaxxx_spi_read_common(context, reg, reg_len, val, val_len, true);
	if (rc)
		goto reg_read_err;

	register_dump_log(dev, reg_addr, val, true, words, IAXXX_READ);

reg_read_err:
	return rc;
}

/* This spi-read function is raw data version. This function reads data
 * from firmware using SPI functions and pass it to caller as it is
 * without modifying any data.
 */
static int iaxxx_spi_bus_raw_read(struct iaxxx_priv *priv, void *buf, int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);
	struct spi_transfer t[1] = {};
	struct spi_message m;
	uint8_t *cbuf = (uint8_t *)buf;
	int rc;
	uint8_t *val;
	uint32_t val_len;
	uint8_t *preg_addr;

	rc = iaxxx_pm_get_sync(&spi->dev);
	if (rc < 0)
		goto get_sync_err;

	mutex_lock(&spi_priv->spi_mutex);
	memset(spi_priv->rx_buf, 0, IAXXX_SPI_MAX_SIZE);
	memset(spi_priv->tx_buf, 0, IAXXX_SPI_MAX_SIZE);

	if (len <= IAXXX_REG_LEN_WITH_PADDING) {
		dev_err(dev, "%s() len parameter invalid\n", __func__);
		rc = -EINVAL;
		goto err;
	}

	if (len > IAXXX_SPI_MAX_SIZE) {
		dev_err(dev, "%s() msg_len excess MAX!\n", __func__);
		rc = -EINVAL;
		goto err;
	}

	spi_message_init(&m);

	preg_addr = spi_priv->tx_buf;

	/* Create buffer to store read data */
	val_len = len - IAXXX_REG_LEN_WITH_PADDING;
	val = spi_priv->rx_buf;

	/* Fetch the Register address with padding */
	memcpy(preg_addr, cbuf, IAXXX_REG_LEN_WITH_PADDING);

	/* Add Register address write message */
	t[0].len = len;
	t[0].tx_buf = (void *)preg_addr;
	t[0].rx_buf = (void *)val;
	spi_message_add_tail(&t[0], &m);

	/* Transfer the message */
	rc = spi_sync(spi, &m);
	if (rc)
		goto err;
	/* Copy the read data into buffer after reagister address */
	memcpy((uint8_t *)(buf + IAXXX_REG_LEN_WITH_PADDING),
			val + IAXXX_REG_LEN_WITH_PADDING, val_len);

err:
	mutex_unlock(&spi_priv->spi_mutex);
get_sync_err:
	iaxxx_pm_put_autosuspend(&spi->dev);
	return rc;
}

/* This spi-write function is raw data version. The caller of this function
 * generates the whole data and this driver does not alter anything and just
 * uses SPI functions to pass it to firmware.
 */
static int iaxxx_spi_bus_raw_write(struct iaxxx_priv *priv, const void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	int rc;

	if (len <= IAXXX_REG_LEN_WITH_PADDING) {
		pr_err("%s() len parameter invalid\n", __func__);
		return -EINVAL;
	}

	rc = iaxxx_pm_get_sync(dev);
	if (rc < 0)
		goto get_sync_err;

	rc =  spi_write(spi, buf, len);

get_sync_err:
	iaxxx_pm_put_autosuspend(dev);
	return rc;
}

static int iaxxx_copy_be32_to_cpu(struct device *dev,
		void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;

	if (len % 4) {
		dev_warn(dev,
			"buffer not aligned to word boundary: %d", len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = be32_to_cpu(data);
	}
	return len;
}

/* This spi-read function is used across driver code for reading directly
 * from firmware's memory. Since SPI communication with firmware is in
 * Big-Endian format, the data read here needs to be converted to CPU
 * byte-order before passing it back. Also register address received here
 * is in CPU byte-order and should be converted to Big-endian before
 * sending for spi read.
 *
 * This function also reads from firmware in chunks defined by
 * IAXXX_SPI_PACKET_LEN since some SPI master drivers have limitation on max
 * amount of data that
 * can be read.
 */
static int iaxxx_spi_bulk_read(struct device *dev,
			uint32_t address, void *read_buf, size_t words)
{
	uint32_t reg_addr;
	size_t reg_len = sizeof(uint32_t) + 12;
	uint32_t words_to_read;
	int rc;
	int count = 0;

	while (words > 0) {
		words_to_read = (words < IAXXX_SPI_PACKET_LEN) ?
				words : IAXXX_SPI_PACKET_LEN;

		dev_dbg(dev, "%s(): words_to_read %d\n",
					__func__, words_to_read);

		/*
		 * The register address passed to iaxxx_spi_read_common
		 * should be in Big-endian format.
		 */
		reg_addr = cpu_to_be32(address);
		rc = iaxxx_spi_read_common(dev, &reg_addr, reg_len, read_buf,
				(words_to_read * 4), true);
		if (rc < 0) {
			dev_err(dev,
				"%s(): bulk read error %d\n",
				__func__, rc);
			return rc;
		}

		/* Data from iaxxx_spi_read_common function is in BE32 format
		 * so it should be converted back to CPU byte order.
		 */
		iaxxx_copy_be32_to_cpu(dev,
					read_buf, (words_to_read << 2));
		address += (words_to_read * sizeof(uint32_t));
		read_buf = ((char *)read_buf) +
				(words_to_read * sizeof(uint32_t));
		words -= words_to_read;
		count += words_to_read;
	}

	return count;
}

/* This version of spi-write is called by second regmap for no_pm calls.
 * Since regmap is configured for Big-endian, the data received here will be
 * in Big-endian format. Since SPI communication with Firmware is Big-endian,
 * no byte-order conversion is needed before sending data.
 *
 * This version does not trigger any power-management related calls.
 */
static int iaxxx_regmap_spi_write_no_pm(void *context,
		const void *data, size_t count)
{
	const void *val = data + sizeof(uint32_t);
	size_t reg_len = sizeof(uint32_t);
	size_t val_len = count - reg_len;
	struct device *dev = context;
	uint32_t *writebuf = (uint32_t *)((uint8_t *)val + IAXXX_REG_PADDING);
	uint32_t reg_addr = *(uint32_t *)data;
	uint32_t words = (val_len - IAXXX_REG_PADDING) / sizeof(uint32_t);
	int rc;

	if (WARN_ON(count <= sizeof(uint32_t))
		|| WARN_ON(val_len <= IAXXX_REG_PADDING)) {
		pr_err("%s(), Error Input param put of range\n", __func__);
		return -EINVAL;
	}

	/* flag set to indicate data and reg-address are in BE32 format */
	rc = iaxxx_spi_write_common(context, data, reg_len,
			val, val_len, true, false);

	register_dump_log(dev, reg_addr, writebuf, true, words, IAXXX_WRITE);
	return rc;
}

/* This version of spi-read is called by second regmap for no-pm calls.
 * Since SPI communication with firmware is in Big-endian and regmap
 * is also configured for Big-endian, the data read here does not
 * need to be altered.
 * Also the register address received here is also in Big-endian format.
 * This function would execute without calling any power-management
 * functions.
 */
static int iaxxx_regmap_spi_read_no_pm(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	struct device *dev = context;
	uint32_t words = val_len / sizeof(uint32_t);
	int rc = 0;
	uint32_t reg_addr = *(uint32_t *) reg;

	rc = iaxxx_spi_read_common(context, reg, reg_len, val, val_len, false);
	if (rc)
		goto reg_read_err;

	register_dump_log(dev, reg_addr, val, true, words, IAXXX_READ);

reg_read_err:
	return rc;
}


/* This spi-write function is regmap interface function for second
 * no_pm regmap. It is called from regmap which is configured to send
 * data in Big-Endian.
 * Since SPI communication with firmware is Big-Endian, there is no
 * conversion needed before sending data.
 *
 * This function does not trigger any power-management related calls.
 */
static int iaxxx_regmap_spi_gather_write_no_pm(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	/* flag set to indicate data and reg-address are in BE32 format
	 * and no pm.
	 */
	return iaxxx_spi_write_common(context,
				 reg, reg_len,
				 val, val_len,
				 true, false);
}

static int iaxxx_spi_populate_dt_data(struct device_node *node, u32 *tmp)
{
	int rc = -EINVAL;

	/* No node means  no dts provision */
	if (node == NULL) {
		pr_err("Invalid of node");
		return 0;
	}

	rc = of_property_read_u32(node, "adnc,spi-app-speed", tmp);
	if (rc < 0) {
		pr_err("Failed to read spi_app_speed, rc = %d\n", rc);
		return rc;
	}

	return rc;

}

static struct regmap_bus regmap_spi = {
	.write = iaxxx_regmap_spi_write,
	.gather_write = iaxxx_regmap_spi_gather_write,
	.read = iaxxx_regmap_spi_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static struct regmap_bus regmap_no_pm_spi = {
	.write = iaxxx_regmap_spi_write_no_pm,
	.gather_write = iaxxx_regmap_spi_gather_write_no_pm,
	.read = iaxxx_regmap_spi_read_no_pm,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};


/* Register map initialization */
static int iaxxx_spi_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev;
	struct regmap *regmap;
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);

	if (!spi_priv || !priv->regmap_config || !priv->regmap_no_pm_config) {
		pr_err("%s: NULL input pointer(s)\n", __func__);
		return -EINVAL;
	}

	dev = &spi_priv->spi->dev;

	/* spi controller requires 12 bytes of zero padding between
	 * address and data
	 */
	priv->regmap_config->pad_bits       = 96;
	priv->regmap_no_pm_config->pad_bits = 96;

	regmap = regmap_init(dev, &regmap_spi,
					spi_priv->spi, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	priv->regmap = regmap;

	regmap = regmap_init(dev, &regmap_no_pm_spi,
			spi_priv->spi, priv->regmap_no_pm_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}
	priv->regmap_no_pm = regmap;
	return 0;
}

static int iaxxx_spi_sync(struct iaxxx_spi_priv *spi_priv)
{
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0;
	uint32_t sync_response;
	struct device *dev = spi_priv->priv.dev;
	const uint32_t SBL_SYNC_CMD = 0x80000000;
	const uint32_t SBL_SYNC_CMD_RESPONSE = 0x8000FFFF;
	const uint32_t CMD_REGMAP_MODE = 0x80040000;

#ifdef DEBUG
	const uint32_t SBL_SYNC_CMD_2 = 0x80720080;
	const uint32_t SBL_SYNC_CMD_3 = 0x80735001;
	const uint32_t SBL_SYNC_CMD_4 = 0x80740000;
	const uint32_t SBL_SYNC_CMD_5 = 0x80750000;
#endif

	do {
		retry--;
		/* Populate device tree data and Reset chip to SBL */
		rc = iaxxx_device_reset(&spi_priv->priv);
		if (rc) {
			dev_err(dev, "%s device reset failed, err:%d\n",
				__func__, rc);
			break;
		}

		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD,
				&sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_RESPONSE) {
			dev_err(dev,
				"%s SYNC fail, err:%d response:0x%.08X\n",
				__func__, rc, sync_response);
			continue;
		}

		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);

		/* The additional SPI testing is enabled only
		 * with DEBUG flag
		 */
#ifdef DEBUG
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD_2,
				&sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_2) {
			dev_err(dev,
				"%s SYNC fail, err:%d response:0x%.08X\n",
				__func__, rc, sync_response);
			continue;
		}
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD_3,
				&sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_3) {
			dev_err(dev,
				"%s SYNC fail, err:%d response:0x%.08X\n",
				__func__, rc, sync_response);
			continue;
		}
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD_4,
				&sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_4) {
			dev_err(dev,
				"%s SYNC fail, err:%d response:0x%.08X\n",
				__func__, rc, sync_response);
			continue;
		}
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD_5,
				&sync_response);
		if (rc || sync_response != SBL_SYNC_CMD_5) {
			dev_err(dev,
				"%s SYNC fail, err:%d response:0x%.08X\n",
				__func__, rc, sync_response);
			continue;
		}
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
#endif
		dev_dbg(dev, "Putting device in regmap mode\n");

		/* Switch the device into regmap mode */
		rc = iaxxx_spi_cmd(spi_priv->spi, CMD_REGMAP_MODE, NULL);
		if (rc) {
			dev_err(dev,
				"%s REGMAP MODE CMD fail, err:%d\n",
				__func__, rc);
			continue;
		} else
			break;
	} while (retry);

	return rc;
}

static int iaxxx_spi_reset_cb(struct device *dev)
{
	struct iaxxx_spi_priv *spi_priv = dev ? dev_get_drvdata(dev) : NULL;
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi_priv || !spi)
		return rc;


	/* set SPI speed to 4.8Mhz */
	spi->max_speed_hz = IAXXX_SPI_SBL_SPEED;
	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s spi 4.8Mhz setup failed, error:%d\n",
				__func__, rc);

	rc = iaxxx_spi_sync(spi_priv);
	if (rc)
		return rc;

	usleep_range(1000, 2500);

	return rc;
}

static int iaxxx_spi_speed_setup(struct device *dev, u32 iaxxx_spi_speed)
{
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi)
		return rc;

	/* set SPI speed to 8Mhz */
	spi->max_speed_hz = iaxxx_spi_speed;
	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s spi speed:%d setup failed, error:%d\n",
				__func__, spi->max_speed_hz, rc);

	dev_dbg(dev, "success spi->max_speed_hz = %d\n", spi->max_speed_hz);
	return rc;
}

static int iaxxx_spi_probe(struct spi_device *spi)
{
	int rc = 0;
	struct iaxxx_spi_priv *spi_priv;
	struct device *dev = &spi->dev;
	struct device_node *node = dev->of_node;
	u32 tmp = 0;

	dev_dbg(dev, "%s:%d\n", __func__, __LINE__);

	/* Create driver private-data struct */
	spi_priv = devm_kzalloc(dev, sizeof(*spi_priv), GFP_KERNEL);
	if (!spi_priv)
		return -ENOMEM;

	mutex_init(&spi_priv->spi_mutex);

	spi_priv->rx_buf = devm_kzalloc(dev,
			IAXXX_SPI_MAX_SIZE, GFP_DMA | GFP_KERNEL);
	if (!spi_priv->rx_buf) {
		rc = -ENOMEM;
		dev_err(dev, "%s rx buf alloc fail\n", __func__);
		goto rx_mem_failed;
	}

	spi_priv->tx_buf = devm_kzalloc(dev,
			IAXXX_SPI_MAX_SIZE, GFP_DMA | GFP_KERNEL);
	if (!spi_priv->tx_buf) {
		rc = -ENOMEM;
		dev_err(dev, "%s tx buf alloc fail\n", __func__);
		goto tx_mem_failed;
	}

	spi_priv->priv.iaxxx_state = devm_kzalloc(dev,
			sizeof(struct iaxxx_system_state), GFP_KERNEL);
	if (!spi_priv->priv.iaxxx_state) {
		rc = -ENOMEM;
		goto state_mem_failed;
	}

	spi_priv->priv.crashlog = devm_kzalloc(dev,
			sizeof(struct iaxxx_crashlog), GFP_KERNEL);
	if (!spi_priv->priv.crashlog) {
		rc = -ENOMEM;
		goto crash_mem_failed;
	}

	spi_priv->spi = spi;
	spi_priv->priv.dev = dev;
	spi_priv->priv.regmap_init_bus = iaxxx_spi_regmap_init;
	spi_priv->priv.bulk_read = iaxxx_spi_bulk_read;
	spi_priv->priv.raw_write = iaxxx_spi_raw_write;
	spi_priv->priv.bus = IAXXX_SPI;

	spi_set_drvdata(spi, spi_priv);

	spi_priv->priv.spi_speed_setup = iaxxx_spi_speed_setup;
	spi_priv->priv.reset_cb = iaxxx_spi_reset_cb;
	rc = iaxxx_device_init(&spi_priv->priv);
	if (rc) {
		dev_err(dev, "%s device init failed, err:%d\n", __func__, rc);
		goto probe_failed;
	}
	spi_priv->priv.raw_ops->write = iaxxx_spi_bus_raw_write;
	spi_priv->priv.raw_ops->read = iaxxx_spi_bus_raw_read;
	spi_priv->priv.crash_count = 0;
	spi_priv->priv.cm4_crashed = false;

	/* Get app SPI Speed from DT*/
	rc = iaxxx_spi_populate_dt_data(node, &tmp);
	if (rc < 0) {
		dev_err(dev, "Failed to read spi_app_speed, rc = %d\n", rc);
		goto probe_failed;
	}
	spi_priv->priv.spi_app_speed = tmp;

	return rc;
probe_failed:
	devm_kfree(&spi->dev, spi_priv->priv.crashlog);
crash_mem_failed:
	devm_kfree(&spi->dev, spi_priv->priv.iaxxx_state);

state_mem_failed:
	devm_kfree(dev, spi_priv->tx_buf);
tx_mem_failed:
	devm_kfree(dev, spi_priv->rx_buf);
rx_mem_failed:
	mutex_destroy(&spi_priv->spi_mutex);
	devm_kfree(dev, spi_priv);
	spi_set_drvdata(spi, NULL);
	return rc;
}

static int iaxxx_spi_remove(struct spi_device *spi)
{
	struct iaxxx_spi_priv *spi_priv = spi_get_drvdata(spi);

	if (spi_priv) {
		mutex_destroy(&spi_priv->spi_mutex);
		iaxxx_device_exit(&spi_priv->priv);
		devm_kfree(&spi->dev, spi_priv->priv.iaxxx_state);
		kvfree(spi_priv->priv.crashlog->ssp_log_buffer);
		spi_priv->priv.crashlog->ssp_log_buffer = NULL;
		kvfree(spi_priv->priv.crashlog->log_buffer);
		devm_kfree(&spi->dev, spi_priv->priv.crashlog);
		devm_kfree(&spi->dev, spi_priv);
	}
	spi_set_drvdata(spi, NULL);

	return 0;
}

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
static const struct dev_pm_ops iaxxx_spi_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_core_suspend_rt, iaxxx_core_resume_rt, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_core_dev_suspend, iaxxx_core_dev_resume)
};
#endif

static const struct spi_device_id iaxxx_spi_id[] = {
	{ "iaxxx-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, iaxxx_spi_id);

static const struct of_device_id iaxxx_spi_dt_match[] = {
	{
		.compatible = "knowles,iaxxx-spi",
	},
	{}
};

static struct spi_driver iaxxx_spi_driver = {
	.driver = {
		.name = "iaxxx-spi",
		.owner = THIS_MODULE,
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
		.pm = &iaxxx_spi_pm_ops,
#endif
		.of_match_table = iaxxx_spi_dt_match,
	},
	.probe = iaxxx_spi_probe,
	.remove = iaxxx_spi_remove,
	.id_table = iaxxx_spi_id,
/*	.device_up = iaxxx_spi_up, */
/*	.device_down = iaxxx_spi_down, */
/*	.reset_device = iaxxx_spi_reset, */
};

module_spi_driver(iaxxx_spi_driver);

MODULE_DESCRIPTION("SPI support for Knowles IAxxx");
MODULE_LICENSE("GPL");
