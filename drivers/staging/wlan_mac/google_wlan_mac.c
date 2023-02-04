/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/setup.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/cnss_utils.h>
#include <soc/qcom/icnss.h>

#define CDB_PATH "/chosen/cdt/cdb2"
#define WIFI_MAC "wlan_mac1"

static void set_wifi_mac(void)
{
	u8 mac[6] = {0};
	unsigned int size;
	unsigned char *mac_addr = NULL;
	struct device_node *node;
	unsigned int mac_found = 0;

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		pr_err("[WLAN] CDB Node not created under %s", CDB_PATH);
	} else {
		mac_addr = (unsigned char *)
				of_get_property(node, WIFI_MAC, &size);
	}

	/* In case Missing Provisioned MAC Address, exit with error */
	if (!mac_addr) {
		pr_err("[WLAN] Missing Provisioned MAC address\n");
		return;
	}

	/* Start decoding MAC Address
	 * Note that 2 formats are supported for now
	 * AA:BB:CC:DD:EE:FF (with separating colons) and
	 * AABBCCDDEEFF (without separating colons) */
	if (sscanf(mac_addr,
		   "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
		   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
		   &mac[5]) == 6) {
		mac_found = 1;
	} else if (sscanf(mac_addr,
			  "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
			  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
			  &mac[5]) == 6) {
		mac_found = 1;
	}

	/* Make sure Address decoding succeeds */
	if (!mac_found) {
		pr_err("[WLAN] Invalid format for Provisioned MAC Address\n");
		return;
	}

	/* Make sure Provisioned MAC Address is globally Administered */
	if (mac[0] & 2) {
		pr_err("[WLAN] Invalid Provisioned MAC Address\n");
		return;
	}

	/* Send provisioned MAC Address to the platform driver */
	if (cnss_utils_set_wlan_mac_address(mac, sizeof(mac)) != 0) {
		pr_err("[WLAN] set wlan mac address failed\n");
		return;
	}

	/* Now derive the derived mac address
	 * by flipping the locally administred bit */
	mac[0] = mac[0] | 2;

	if (cnss_utils_set_wlan_derived_mac_address(mac, sizeof(mac)) != 0) {
		pr_err("[WLAN] set wlan derived  mac address failed\n");
		return;
	}

	return;
}

static int __init wifi_mac_init(void)
{
	set_wifi_mac();

	return 0;
}

late_initcall(wifi_mac_init);

MODULE_AUTHOR("Google");
MODULE_DESCRIPTION("Google set wlan mac");
MODULE_LICENSE("GPL v2");

