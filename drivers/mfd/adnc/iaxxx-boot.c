/*
 * iaxxx-boot.c -- IAxxx boot firmware to application mode
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

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-btp.h"
#include <linux/mfd/adnc/iaxxx-core.h>

#define IAXXX_CHUNK_SIZE 8192
#define IAXXX_REDUCED_CHUNK_SIZE 4096

/* Firmware and hardware configuration files */
static const char *iaxxx_fw_filename = "RomeApp.bin";

static bool iaxxx_sect_valid_proc_mask(
			uint32_t sect_addr, uint32_t proc_mask)
{
	if (((sect_addr >= GLOBAL_DRAM_START) &&
		(sect_addr <= GLOBAL_DRAM_END) &&
		(proc_mask & GLBL_MEM_ID_MASK)) ||
		((sect_addr >= SSP_DMEM0_SYS_START) &&
		(sect_addr <= SSP_DMEM1_SYS_END) &&
		(proc_mask & SSP_ID_MASK)) ||
		((sect_addr >= HMD_DMEM_SYS_START) &&
		(sect_addr <= HMD_IMEM_SYS_END) &&
		(proc_mask & HMD_ID_MASK)) ||
		((sect_addr >= DMX_DMEM_SYS_START) &&
		(sect_addr <= DMX_IMEM_SYS_END) &&
		(proc_mask & DMX_ID_MASK)))
		return true;
	else
		return false;
}

void iaxxx_copy_le32_to_cpu(void *dst, const void *src, size_t nbytes)
{
	int i;
	int num_words = nbytes / sizeof(uint32_t);
	uint32_t *p_dst = dst;
	const uint32_t *p_src = src;

	if (WARN_ON(!src) || WARN_ON(!dst) || WARN_ON(nbytes & 0x3))
		return;

	for (i = 0; i < num_words; ++i, ++p_dst, ++p_src)
		*p_dst = le32_to_cpu(*p_src);
}

/**
 * iaxxx_download_section_chunks - downloads a firmware text or data section
 *
 * @priv    : iaxxx private data
 * @data    : the section data to be downloaded
 * @chunk_size: max chunk size downloaded
 * @section : pointer to the section data (section address, length, etc).
 * @regmap : regmap to use
 */
static int iaxxx_download_section_chunks(struct iaxxx_priv *priv,
			const uint8_t *data, uint32_t chunk_size,
			const struct firmware_section_header *section,
			struct regmap *regmap,
			bool btp)
{
	int rc, i = 0;
	struct device *dev = priv->dev;
	int rem_bytes = section->length % (chunk_size);
	int temp_len = section->length / (chunk_size);
	int chunk_word_size = chunk_size * 4;

	dev_err(dev, "Writing section at 0x%.08X, %d words(s)\n",
				section->start_address, section->length);

	/* Write the section data directly to the device memory */
	for (i = 0; i < temp_len; i++) {
		/* If BTP is enabled use btp_write instead of direct
		 * write to memory. And assume HOST0 always.
		 */
		if (btp)
			rc = iaxxx_btp_write(priv,
				(section->start_address +
				((i * chunk_word_size))),
				data + (i * chunk_word_size), chunk_size,
				IAXXX_HOST_0);
		else
			rc = regmap_bulk_write(regmap,
				(section->start_address +
				((i * chunk_word_size))),
				data + (i * chunk_word_size), chunk_size);
		if (rc) {
			dev_err(dev,
				"%s: regmap bulk write section chunk failed, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}
	if (rem_bytes) {
		/* If BTP is enabled use btp_write instead of direct
		 * write to memory. And assume HOST0 always.
		 */
		if (btp)
			rc = iaxxx_btp_write(priv,
				(section->start_address +
				((i * chunk_word_size))),
				data + (i * chunk_word_size), rem_bytes,
				IAXXX_HOST_0);
		else
			rc = regmap_bulk_write(regmap,
				(section->start_address +
				((i * chunk_word_size))),
				data + (i * chunk_word_size), rem_bytes);
		if (rc) {
			dev_err(dev,
				"%s: regmap bulk write rem_bytes failed, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}

	return 0;
}

/**
 * iaxxx_download_section - downloads a firmware text or data section
 * This version of the function used no_pm regmap and it is intended
 * for usage during booting firmware.
 *
 * @priv    : iaxxx private data
 * @data    : the section data to be downloaded
 * @section : pointer to the section data (section address, length, etc).
 * @regmap  : regmap to use
 */
int iaxxx_download_section(struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section,
				struct regmap *regmap,
				bool btp)
{
	int rc = 0;
	struct device *dev = priv->dev;
	uint32_t iaxxx_chunk_size = (priv->bus == IAXXX_I2C)
				? IAXXX_CHUNK_SIZE/64 : IAXXX_CHUNK_SIZE;

	rc = iaxxx_download_section_chunks(priv, data,
					iaxxx_chunk_size, section, regmap, btp);
	if (rc == -ENOMEM) {
		iaxxx_chunk_size = (priv->bus == IAXXX_I2C)
					? IAXXX_REDUCED_CHUNK_SIZE / 64 :
					IAXXX_REDUCED_CHUNK_SIZE;
		dev_err(dev,
			"retry section download with reduced chunk size\n");
		rc = iaxxx_download_section_chunks(priv, data,
						iaxxx_chunk_size, section,
						regmap, btp);
		if (rc == -ENOMEM) {
			dev_err(dev,
				"%s: failed: %d, with reduced chunk size: %d",
				__func__, rc, IAXXX_REDUCED_CHUNK_SIZE);
			return rc;
		}
	}
	if (rc)
		dev_err(dev,
			"%s: regmap bulk write section failed, rc=%d\n",
			__func__, rc);
	else
		dev_dbg(dev,
			"download section at 0x%pK, %d words(s) successful\n",
			section->start_address, section->length);

	return rc;
}

int iaxxx_verify_fw_header(struct device *dev,
			struct firmware_file_header *header)
{
	const uint32_t SIG1 = 0x49445541;	/* "IDUA */
	const uint32_t SIG2 = 0x45434E45;	/* "ECNE */

	/* Verify the file signature */
	if (header->signature[0] != SIG1 || header->signature[1] != SIG2) {
		dev_err(dev, "Bad firmware file signature\n");
		return -EINVAL;
	}

	/* The file must contain at least 1 section */
	if (!header->number_of_sections) {
		dev_err(dev, "Missing sections in firmware file\n");
		return -EINVAL;
	}

	return 0;
}

static int
iaxxx_download_firmware(struct iaxxx_priv *priv, const struct firmware *fw)
{
	int i, rc = 0;
	int retries = 0;
	const int max_retries = 5;
	const uint8_t *data;
	struct device *dev = priv->dev;

	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;

	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };

	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad Firmware image file (too small)\n");
		goto out;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad firmware image file\n");
		goto out;
	}

	/* Include file header fields as part of the checksum */
	CALC_FLETCHER16(header.number_of_sections, sum1, sum2);
	CALC_FLETCHER16(header.entry_point, sum1, sum2);

	if (header.number_of_sections <= 1) {
		dev_err(dev, "No sections available to download\n");
		rc = -EINVAL;
		goto out;
	}

	/* Download each memory section */
	for (i = 0; i < header.number_of_sections; ++i) {
		/* Load the next data section */
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);

		if (file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);

			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);

			do {
				/* Use no_pm regmap since this is during booting
				 * and access to default regmap is disallowed
				 */
				rc = iaxxx_download_section(
						priv, data, &file_section,
						priv->regmap_no_pm, false);

			} while (rc && ++retries < max_retries);

			if (rc) {
				dev_err(dev, "Failed to load firmware section\n");
				goto out;
			}

			/* Include checksum for this section
			 * Use no_pm regmap for this because this is called
			 * during booting and there is no access to default
			 * regmap.
			 */
			rc = iaxxx_checksum_request(priv,
					file_section.start_address,
					file_section.length, &sum1, &sum2,
					priv->regmap_no_pm);
			if (rc) {
				dev_err(dev, "Checksum request error\n");
				goto out;
			}
			dev_dbg(dev, "%s(): section(%d) Checksum 0x%04X%04x\n",
				__func__, i, sum2, sum1);
			/* Next section */
			data += file_section_bytes;
			WARN_ON((data - fw->data) > fw->size);
		}
	}

	/* If the last section length is 0, then verify the checksum */
	if (file_section.length == 0) {
		uint32_t checksum = (sum2 << 16) | sum1;

		dev_err(dev, "Expected checksum = 0x%.08X\n", checksum);

		if (checksum != file_section.start_address) {
			rc = -EINVAL;
			dev_err(dev,
				"%s(): Checksum mismatch 0x%.08X != 0x%.08X\n",
				__func__, checksum, file_section.start_address);
		}
	}

out:
	return rc;
}

static int iaxxx_download_per_core_fw(struct iaxxx_priv *priv,
		const struct firmware *fw, u32 proc_id)
{
	int i, rc = 0;
	int retries = 0;
	const int max_retries = 5;
	const uint8_t *data;
	struct device *dev = priv->dev;
	u32 proc_id_mask = 1 << proc_id;

	/* Checksum variable */
	uint32_t devicesum1 = 0xffff;
	uint32_t devicesum2 = 0xffff;

	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };

	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad Firmware image file (too small)\n");
		goto out;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad firmware image file\n");
		goto out;
	}

	if (header.number_of_sections <= 1) {
		dev_err(dev, "No sections available to download\n");
		rc = -EINVAL;
		goto out;
	}

	/* Download each memory section */
	for (i = 0; i < header.number_of_sections; ++i) {
		/* Load the next data section */
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);

		dev_dbg(dev, "file_section.start_address: 0x%pK\n",
				file_section.start_address);
		if (iaxxx_sect_valid_proc_mask(file_section.start_address,
			proc_id_mask) && file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);

			do {
				rc = iaxxx_download_section(priv, data,
							&file_section,
							priv->regmap,
							false);
			} while (rc && ++retries < max_retries);

			if (rc) {
				dev_err(dev, "Failed to load firmware section\n");
				goto out;
			}

			/* Include checksum for this section */
			rc = iaxxx_checksum_request(priv,
				file_section.start_address, file_section.length,
				&devicesum1, &devicesum2, priv->regmap);
			if (rc) {
				dev_err(dev, "Checksum request error\n");
				goto out;
			}
			dev_dbg(dev, "%s(): section(%d) Checksum 0x%04X%04x\n",
				__func__, i, devicesum2, devicesum1);
		}
		/* Next section */
		data += file_section.length * sizeof(u32);
		WARN_ON((data - fw->data) > fw->size);
	}

	dev_info(dev, "Proc (%d) download success\n", proc_id);
out:
	return rc;
}

int iaxxx_boot_core(struct iaxxx_priv *priv, u32 proc_id)
{
	const struct firmware *fw;
	struct device *dev = priv->dev;
	int rc;

	/* Request the firmware image */
	rc = request_firmware(&fw, iaxxx_fw_filename, dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found rc = %d\n",
						iaxxx_fw_filename, rc);
		return rc;
	}
	/* Download the firmware to device memory */
	rc = iaxxx_download_per_core_fw(priv, fw, proc_id);
	if (rc) {
		dev_err(dev,
			"core(%d) firmware download failed, rc = %d\n",
			proc_id, rc);
		goto out;
	}
	/* core is up and running */
	dev_dbg(dev,
		"Processor core(%d) Firmware is loaded\n", proc_id);

out:
	release_firmware(fw);
	return rc;
}

/**
 * iaxxx_wait_apps_ready - Wait for firmware application mode to be ready
 *
 * @priv: iaxxx private data
 *
 * Returns 0 on success or -ETIMEDOUT if the device wasn't ready within the
 * expected time.
 */
static int iaxxx_wait_apps_ready(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int i, rc = 0;
	uint32_t status;
	uint8_t mode;

	const unsigned int IAXXX_APPS_MODE_POLL_MSEC = 20;	/* 20 ms */
	const unsigned int IAXXX_APPS_MODE_WAIT_MSEC = 200;	/* 200 ms */
	const unsigned int count = \
		IAXXX_APPS_MODE_WAIT_MSEC / IAXXX_APPS_MODE_POLL_MSEC;

	/* wait for fw boot to app mode success event */
	rc = wait_event_timeout(priv->boot_wq, priv->boot_completed, HZ);
	if (!priv->boot_completed && rc == 0)
		dev_err(priv->dev,
			"Timedout in wait for boot completion event, "
			"try polling\n");
	else
		return 0;

	/* Apps mode is expected to be ready within 50 msecs */
	for (i = 0; i < count; ++i) {
		/* In absence of events, poll SYSTEM_STATUS for App Mode */
		rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
				&status);
		if (rc) {
			dev_err(dev,
				"Failed to read SYSTEM_STATUS, rc = %d\n", rc);
			return rc;
		}

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(dev, "System status 0x%pK, mode = %d\n",
						status, mode);

		if (mode == SYSTEM_STATUS_MODE_APPS)
			return 0;

		msleep(IAXXX_APPS_MODE_POLL_MSEC);
	}

	return -ETIMEDOUT;
}

/**
 * iaxxx_bootup - Boots the chip hardware
 *
 * @priv: iaxxx private data
 *
 * Downloads any necessary hardware configuration and RAM patches.
 */
int iaxxx_bootup(struct iaxxx_priv *priv)
{
	const struct firmware *fw;
	struct device *dev = priv->dev;
	const struct firmware_file_header *fw_header;
	int rc;
#ifdef DEBUG
	uint32_t reg;
#endif

	/* Request the firmware image */
	rc = request_firmware(&fw, iaxxx_fw_filename, dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found rc = %d\n",
						iaxxx_fw_filename, rc);
		return rc;
	}
	/* Download the firmware to device memory */
	rc = iaxxx_download_firmware(priv, fw);
	if (rc) {
		dev_err(dev, "Firmware download failed, rc = %d\n", rc);
		goto out;
	}

	fw_header = (const struct firmware_file_header *)fw->data;

#ifdef DEBUG
	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_DEVICE_ID_ADDR,
			&reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "Device ID before jump to ram: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR,
			&reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "ROM Version before jump to ram: 0x%.08X\n", reg);
#endif

	/* Boot device into application mode */
	rc = iaxxx_jump_to_request(priv, le32_to_cpu(fw_header->entry_point));
	if (rc) {
		dev_err(dev, "Failed to boot firmware, rc = %d\n", rc);
		goto out;
	}

	/* Enable the irq to receive boot complete event */
	if (gpio_is_valid(priv->event_gpio) && !priv->is_irq_enabled) {
		enable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = true;
	}

	/* Wait for the application mode to be up and running */
	rc = iaxxx_wait_apps_ready(priv);
	if (rc) {
		dev_err(dev, "Timed out waiting for apps mode, rc = %d\n", rc);
		goto out;
	}

	/* device running in application mode */
	dev_dbg(dev, "Firmware running in application mode\n");

out:
	release_firmware(fw);
	return rc;
}
