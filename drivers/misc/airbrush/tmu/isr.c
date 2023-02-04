// SPDX-License-Identifier: GPL-2.0
/*
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
#include "isr.h"

#include "drvdata.h"
#include "hw.h"
#include "sensor.h"

#include <linux/device.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

struct ab_tmu_isr {
	struct device *dev;
	int irq;
	struct notifier_block nb;
	struct work_struct work;
	spinlock_t sensor_irq_lock;
	u32 sensor_irq[AB_TMU_NUM_ALL_PROBE];
};

static void ab_tmu_isr_work(struct work_struct *work)
{
	struct ab_tmu_isr *isr = container_of(work, struct ab_tmu_isr, work);
	struct ab_tmu_drvdata *data = dev_get_drvdata(isr->dev);
	int i;
	unsigned long flags;
	u32 sensor_irq[AB_TMU_NUM_ALL_PROBE];

	spin_lock_irqsave(&isr->sensor_irq_lock, flags);
	memcpy(sensor_irq, isr->sensor_irq, sizeof(sensor_irq));
	memset(isr->sensor_irq, 0, sizeof(isr->sensor_irq));
	spin_unlock_irqrestore(&isr->sensor_irq_lock, flags);

	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		if (!sensor_irq[i])
			continue;
		ab_tmu_sensor_update(data->sensor[i]);
		ab_tmu_sensor_notify(data->sensor[i]);
		dev_dbg(isr->dev, "interrupt for sensor %d triggered: %x",
				i, sensor_irq[i]);
	}
}

static void ab_tmu_isr_handler(struct ab_tmu_isr *isr)
{
	struct ab_tmu_drvdata *data = dev_get_drvdata(isr->dev);
	struct ab_tmu_hw *hw = data->hw;
	int i;
	u32 val_irq;

	spin_lock(&isr->sensor_irq_lock);
	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		val_irq = ab_tmu_hw_read(hw, AB_TMU_INTPEND(i));
		isr->sensor_irq[i] |= val_irq;
		ab_tmu_hw_write(hw, AB_TMU_INTPEND(i), val_irq);
	}
	spin_unlock(&isr->sensor_irq_lock);

	schedule_work(&isr->work);
}

static int ab_tmu_isr_notify(struct notifier_block *nb,
		unsigned long irq, void *data)
{
	struct ab_tmu_isr *isr = container_of(nb, struct ab_tmu_isr, nb);
	u32 intnc_val = (u64)data;

	if (irq == ABC_MSI_AON_INTNC &&
			(intnc_val & (1 << (isr->irq - ABC_MSI_COUNT))))
		ab_tmu_isr_handler(isr);

	return 0;
}

static int ab_tmu_isr_init(struct ab_tmu_isr *isr, struct device *dev)
{
	int ret;

	isr->dev = dev;
	/* TODO give from parameter instead of hard coded value. */
	isr->irq = 36;

	INIT_WORK(&isr->work, ab_tmu_isr_work);

	spin_lock_init(&isr->sensor_irq_lock);
	memset(isr->sensor_irq, 0, sizeof(isr->sensor_irq));

	isr->nb.notifier_call = ab_tmu_isr_notify;
	ret = abc_reg_notifier_callback(&isr->nb);
	if (ret) {
		dev_err(dev, "Airbrush pcie is not ready\n");
		return -ENODEV;
	}

	return 0;
}

static void ab_tmu_isr_exit(struct ab_tmu_isr *isr)
{
	/* TODO unreg notifier callback */
}

static void ab_tmu_isr_release(struct device *dev, void *res)
{
	struct ab_tmu_isr *isr = res;

	ab_tmu_isr_exit(isr);
}

struct ab_tmu_isr *devm_ab_tmu_isr_request(struct device *dev)
{
	struct ab_tmu_isr *isr;
	int err;

	isr = devres_alloc(ab_tmu_isr_release, sizeof(struct ab_tmu_isr),
			GFP_KERNEL);
	if (!isr)
		return ERR_PTR(-ENOMEM);

	err = ab_tmu_isr_init(isr, dev);
	if (err < 0) {
		devres_free(isr);
		return ERR_PTR(err);
	}

	devres_add(dev, isr);
	return isr;
}
