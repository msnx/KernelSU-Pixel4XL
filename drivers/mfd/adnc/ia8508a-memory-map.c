/*
 * ia8508a-memory-map.c -- IA platform specific memory map
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
#include "ia8508a-memory-map.h"

void rom_phy_address_range_check_and_update(uint32_t *phy_addr,
		uint32_t in_bytes, uint32_t *phy_size1,
		uint32_t *phy_addr2, uint32_t *phy_size2)
{
	uint32_t phy_size = in_bytes, i;
	uint32_t rom_addr[] = IAXXX_ROM_ADDR_MAPPING;
	int count = sizeof(rom_addr) / sizeof(rom_addr[1]);

	for (i = 0; i < count; i++) {
		if (IS_WITHIN_ROM_ADDRESS_RANGE(
			rom_addr[i], *phy_addr, in_bytes)) {
			phy_size = (rom_addr[i] - *phy_addr);
			break;
		}
	}
	*phy_size1 = phy_size;

	if (phy_size != in_bytes) {
		*phy_addr2 = *phy_addr + phy_size + MAX_ADDR_ROM_SIZE;
		*phy_size2 = in_bytes - phy_size;
		return;
	}
	for (i = 0; i < count; i++) {
		if (IS_PHY_START_ADDR_IN_ROM_ADDRESS_RANGE(
			rom_addr[i], *phy_addr)) {
			phy_size = (*phy_addr - rom_addr[i]);
			*phy_addr = rom_addr[i] + MAX_ADDR_ROM_SIZE + phy_size;
			break;
		}
	}
	*phy_size2 = 0;
	*phy_addr2 = 0;
	return;
}
