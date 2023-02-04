/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2019 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _AIRBRUSH_TMU_ISR_
#define _AIRBRUSH_TMU_ISR_

struct ab_tmu_isr;
struct device;

struct ab_tmu_isr *devm_ab_tmu_isr_request(struct device *dev);

#endif /* _AIRBRUSH_TMU_ISR_ */
