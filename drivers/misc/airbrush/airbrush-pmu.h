/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Thiagu Ramalingam(thiagu.r@samsung.com)
 *	Raman Kumar Banka(raman.k2@samsung.com)
 *
 * Airbrush Power Management Unit Control driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_PMU_H_
#define _AIRBRUSH_PMU_H_

struct ab_pmu_context {
	struct device *dev;

	struct mutex pcie_link_lock;
	bool pcie_link_ready; /* Guarded by pcie_link_lock */
	struct notifier_block pcie_link_blocking_nb;
};

#endif //_AIRBRUSH_PMU_H_
