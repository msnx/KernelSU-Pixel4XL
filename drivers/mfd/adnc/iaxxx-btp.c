/*
 * iaxxx-btp.c -- Bulk transfer protocol
 *
 * Copyright 2019 Knowles Corporation
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
#include <linux/kernel.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-bulk-transfer.h>
#include "iaxxx.h"
#include "ia8508a-memory-map.h"

#define IAXXX_MAC_SIGN(v, p)	(((v) > (p)) ? 1 : 0)
#define IAXXX_MAC_OFFSET(v, p)	((abs((v) - (p)) >> 10) & 0x0FFF)
#define IAXXX_MAC_OFFSET_SIGN(offset, sign)	((((sign) << 16) & 0x10000) | \
		(offset))

#define IAXXX_BTP_MAX_BLOCK_SIZE	(0x2000)

struct proc_id_type {
	uint32_t start;
	uint32_t end;
	uint32_t proc_id;
	uint32_t type;
};

static const uint32_t mac_addr[][2] = IAXXX_MAC_REG_ADDR_MAPPING;
static const struct proc_id_type pid[] = IAXXX_PROC_ID_TYPE_MAPPING;


static int iaxxx_get_proc_id_and_type(uint32_t paddr, uint32_t *type,
		uint32_t *proc_id)
{
	int i;
	int ret = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(pid); i++) {
		if ((paddr >= pid[i].start) && (paddr < pid[i].end)) {
			*type = pid[i].type;
			*proc_id = pid[i].proc_id;
			return 0;
		}
	}
	return ret;
}

static uint32_t iaxxx_get_mac_addr(uint32_t proc_id, uint32_t type,
		int host_id)
{
	uint32_t block_id;
	uint32_t offset;

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);
	offset = type + (block_id * 2);
	return mac_addr[offset][host_id];
}


static int iaxxx_get_btp_size_addr(struct iaxxx_priv *priv,
		uint32_t proc_id,
		uint32_t type,
		int      host_id,
		uint32_t *btp_size,
		uint32_t *btp_addr)
{
	uint32_t ret;
	uint32_t block_id;
	uint32_t offset;

	ret = regmap_read(priv->regmap,
			IAXXX_BULK_TRANSFER_BULK_TRANSFER_SIZE_ADDR,
			btp_size);
	if (ret) {
		dev_err(priv->dev, "read failed %d\n", ret);
		goto exit;
	}

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);
	offset = type + (block_id * 2);

	ret = regmap_bulk_read(priv->regmap,
		(IAXXX_BULK_TRANSFER_BULK_TRANSFER_I_ADDR_BLOCK_0_ADDR +
		(offset * sizeof(uint32_t)) +
		(host_id * IAXXX_BULK_TRANSFER_REG_NUM * sizeof(uint32_t))),
		btp_addr, 1);
	if (ret) {
		dev_err(priv->dev, "bulk read failed %d\n", ret);
		goto exit;
	}

	dev_dbg(priv->dev, "BTP information Size 0x%x", *btp_size);
	dev_dbg(priv->dev, "BTP addr 0x%pK", *btp_addr);

	if (!(*btp_size) || *btp_size > IAXXX_BTP_MAX_BLOCK_SIZE) {
		dev_err(priv->dev, "Invalid BTP size 0x%x", *btp_size);
		ret = -EIO;
	}

exit:
	return ret;
}

/* This function checks for FW ROM range section overlap
 * case before starting burst writes. if range check falls into
 * any ROM address range then driver will split and send the data
 * other wise it sends data in one shot.
 *
 */
static int iaxxx_btp_write_words(struct iaxxx_priv *priv, uint32_t phy_addr,
		const void *data, uint32_t words, uint32_t proc_id,
		uint32_t type, int host_id)
{
	uint32_t addr_offset;
	uint32_t vaddr;
	uint32_t size, phy_val, read_val;
	int ret;
	const uint8_t *pdata = (const uint8_t *)data;
	uint32_t btp_size;
	uint32_t btp_addr, mac_addr;

	/* Due to FW limitation, use only HOST0's BTP addresses
	 * and MAC window to write data.
	 */
	host_id = 0;

	ret = iaxxx_get_btp_size_addr(priv, proc_id, type, host_id,
			&btp_size, &btp_addr);
	if (ret) {
		dev_err(priv->dev, "not able to read btp size/addresses %d\n",
				ret);
		return ret;
	}

	vaddr = btp_addr;

	dev_dbg(priv->dev, "proc-id:%u type=%u host-id=%d\n",
			proc_id, type, host_id);

	mac_addr = iaxxx_get_mac_addr(proc_id, type, host_id);

	/* Address has to be multiples of MAC window size,
	 * so calculate the offset and use it for first transaction
	 */
	addr_offset = phy_addr % IAXXX_MAC_WINDOW_SIZE;
	if ((addr_offset >> 2) > btp_size) {
		ret = -EINVAL;
		goto exit;
	}

	while (words) {
		/* Size of each transfer cannot exceed chunk size */
		size = ((words + (addr_offset >> 2)) > btp_size)
			? btp_size - (addr_offset >> 2) : words;
		/* Configure mac physical address */
		phy_val = IAXXX_MAC_OFFSET_SIGN(
			IAXXX_MAC_OFFSET(vaddr,	phy_addr - addr_offset),
			IAXXX_MAC_SIGN(vaddr, phy_addr - addr_offset));

		ret = regmap_write(priv->regmap, mac_addr, phy_val);
		if (ret) {
			dev_err(priv->dev, "Physical address config fail\n");
			goto exit;
		}

		ret = regmap_read(priv->regmap, mac_addr, &read_val);
		if (ret) {
			dev_err(priv->dev, "Mac address read fail\n");
			goto exit;
		}
		if (phy_val != read_val) {
			dev_err(priv->dev, "Mac address write didn't happen\n");
			ret = -EINVAL;
			goto exit;
		}

		ret = regmap_bulk_write(priv->regmap, vaddr + addr_offset,
			 (void *)pdata, size);
		if (ret) {
			dev_err(priv->dev, "Write failed %d\n", ret);
			goto exit;
		}

		words -= size;
		pdata += (size << 2);
		phy_addr += (size << 2);
		addr_offset = 0;
	}
exit:
	return ret;
}

static int iaxxx_btp_read_words(struct iaxxx_priv *priv, uint32_t phy_addr,
					void *data, uint32_t words,
					uint32_t proc_id, uint32_t type,
					int host_id)
{
	uint32_t addr_offset;
	uint32_t vaddr;
	uint32_t size, phy_val, read_val;
	int ret;
	uint32_t btp_size;
	uint32_t btp_addr, mac_addr;


	/* Due to FW limitation, use only HOST0's BTP addresses
	 * and MAC window to read data.
	 */
	host_id = 0;

	ret = iaxxx_get_btp_size_addr(priv, proc_id, type, host_id,
			&btp_size, &btp_addr);
	if (ret) {
		dev_err(priv->dev, "not able to get btp size/addresses %d\n",
			ret);
		return ret;
	}

	dev_dbg(priv->dev, "proc-id:%u type=%u host-id=%d\n",
			proc_id, type, host_id);

	vaddr = btp_addr;

	mac_addr = iaxxx_get_mac_addr(proc_id, type, host_id);

	/* Address has to be multiples of MAC window size,
	 * so calculate the offset and use it for first transaction
	 */
	addr_offset = phy_addr % IAXXX_MAC_WINDOW_SIZE;
	if ((addr_offset >> 2) > btp_size) {
		ret = -EINVAL;
		goto exit;
	}

	while (words) {
		/* Size of each transfer cannot exceed chunk size */
		size = ((words + (addr_offset >> 2)) > btp_size)
			? btp_size - (addr_offset >> 2) : words;
		/* Configure mac physical address */
		phy_val = IAXXX_MAC_OFFSET_SIGN(
			IAXXX_MAC_OFFSET(vaddr,	phy_addr - addr_offset),
			IAXXX_MAC_SIGN(vaddr, phy_addr - addr_offset));

		ret = regmap_write(priv->regmap, mac_addr, phy_val);
		if (ret) {
			dev_err(priv->dev, "Physical address config fail\n");
			goto exit;
		}
		ret = regmap_read(priv->regmap, mac_addr, &read_val);
		if (ret) {
			dev_err(priv->dev, "Mac address read fail\n");
			goto exit;
		}
		if (phy_val != read_val) {
			dev_err(priv->dev, "Mac address write didn't happen\n");
			ret = -EINVAL;
			goto exit;
		}

		/* Write data to virtual address */
		ret = priv->bulk_read
			(priv->dev, vaddr + addr_offset, data, size);
		if (ret != size) {
			ret = -EINVAL;
			goto exit;
		}

		words -= size;
		data += (size << 2);
		phy_addr += (size << 2);
		addr_offset = 0;
	}

	ret = 0;
exit:
	return ret;
}

int iaxxx_btp_write(struct iaxxx_priv *priv, uint32_t phy_addr,
		const void *pdata, uint32_t words, int host_id)
{
	int ret;
	uint32_t proc_id;
	uint32_t type;
	uint32_t phy_size_range1;
	uint32_t phy_addr_range2, phy_size_range2;
	uint32_t phy_addr_range1 = phy_addr;

	dev_dbg(priv->dev, "%s addr=%pK words=%u\n", __func__, phy_addr,
			words);

	mutex_lock(&priv->btp_lock);

	ret = iaxxx_get_proc_id_and_type(phy_addr, &type, &proc_id);
	if (ret) {
		dev_err(priv->dev, "not able to get block id and type %d\n",
			ret);
		goto exit;
	}

	rom_phy_address_range_check_and_update(&phy_addr_range1,
		words * sizeof(uint32_t), &phy_size_range1, &phy_addr_range2,
		&phy_size_range2);

	dev_dbg(priv->dev, "addr1=%pK size1=%u addr2=%pK size2=%u\n",
			phy_addr_range1, phy_size_range1,
			phy_addr_range2, phy_size_range2);

	ret = iaxxx_btp_write_words(priv, phy_addr_range1, pdata,
			phy_size_range1 / sizeof(uint32_t),
			proc_id, type, host_id);
	if (ret) {
		dev_err(priv->dev, "btp write fail %d\n", ret);
		goto exit;
	}

	/* If address has hole write data after the hole */
	if (phy_size_range2 != 0 && phy_addr_range2 != 0)
		ret = iaxxx_btp_write_words(priv, phy_addr_range2,
				((const uint8_t *)pdata) + phy_size_range1,
				(phy_size_range2 / sizeof(uint32_t)),
				proc_id, type, host_id);
	if (ret)
		dev_err(priv->dev, "btp write fail %d ret\n", ret);

exit:
	mutex_unlock(&priv->btp_lock);
	return ret;
}

int iaxxx_btp_read(struct iaxxx_priv *priv, uint32_t phy_addr,
		void *pdata, uint32_t words, int host_id)
{
	int ret;
	uint32_t proc_id;
	uint32_t type;
	uint32_t phy_size_range1;
	uint32_t phy_addr_range2, phy_size_range2;
	uint32_t phy_addr_range1 = phy_addr;

	dev_dbg(priv->dev, "%s addr=%pK words=%u\n", __func__, phy_addr,
			words);

	mutex_lock(&priv->btp_lock);

	ret = iaxxx_get_proc_id_and_type(phy_addr, &type, &proc_id);
	if (ret) {
		dev_err(priv->dev, "not able to get block id and type %d\n",
			ret);
		goto exit;
	}

	rom_phy_address_range_check_and_update(&phy_addr_range1,
		words * sizeof(uint32_t), &phy_size_range1, &phy_addr_range2,
		&phy_size_range2);

	dev_dbg(priv->dev, "addr1=%pK size1=%u addr2=%pK size2=%u\n",
			phy_addr_range1, phy_size_range1,
			phy_addr_range2, phy_size_range2);

	ret = iaxxx_btp_read_words(priv, phy_addr_range1, pdata,
			phy_size_range1 / sizeof(uint32_t),
			proc_id, type, host_id);
	if (ret) {
		dev_err(priv->dev, "btp read fail %d\n", ret);
		goto exit;
	}

	/* If address has hole write data after the hole */
	if (phy_size_range2 != 0 && phy_addr_range2 != 0)
		ret = iaxxx_btp_read_words(priv, phy_addr_range2,
				((uint8_t *)pdata) + phy_size_range1,
				(phy_size_range2 / sizeof(uint32_t)),
				proc_id, type, host_id);
	if (ret)
		dev_err(priv->dev, "btp read fail %d ret\n", ret);

exit:
	mutex_unlock(&priv->btp_lock);
	return ret;
}
