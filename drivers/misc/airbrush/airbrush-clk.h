/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _AIRBRUSH_CLK_H_
#define _AIRBRUSH_CLK_H_

#include <linux/airbrush-sm-ctrl.h>
#include <linux/clk-provider.h>
#include <linux/of.h>

struct ab_clk_context {
	struct device *dev;

	struct mutex pcie_link_lock;
	bool pcie_link_ready; /* Guarded by pcie_link_lock */
	struct notifier_block pcie_link_blocking_nb;

	uint32_t last_ipu_val;
	uint32_t last_tpu_val;
};

#endif //_AIRBRUSH_CLK_H_
