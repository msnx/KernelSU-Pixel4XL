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
#include <linux/mfd/abc-pcie.h>
#include <linux/airbrush-sm-ctrl.h>
#include "airbrush-pmu.h"

/* IPU and TPU Status Timeout */
#define IPU_TPU_STATUS_TIMEOUT			1000

/* ABC PMU config regisetr offsets */
#define SYSREG_PMU_PMU_CONTROL				0x10ba0004
#define SYSREG_PMU_PMU_STATUS				0x10ba0000
#define CMU_IPU_IPU_CONTROLLER_OPTION			0x10240800
#define CMU_TPU_TPU_CONTROLLER_OPTION			0x10040800

#define CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK 0x10041000
#define CLK_CON_MUX_MOUT_IPU_AONCLK_PLLCLK 0x10241000

static struct ab_sm_pmu_ops pmu_ops;
static int ab_pmu_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct ab_pmu_context *pmu_ctx = container_of(nb,
			struct ab_pmu_context, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		mutex_lock(&pmu_ctx->pcie_link_lock);
		pmu_ctx->pcie_link_ready = true;
		mutex_unlock(&pmu_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	/*
	 * Handling ABC_PCIE_LINK_ERROR is the same with handling
	 * ABC_PCIE_LINK_PRE_DISABLE at this moment.
	 *
	 * If the handling is changed to access registers in the future,
	 * handling of the 2 flags need to be split.
	 */
	if (action & (ABC_PCIE_LINK_PRE_DISABLE | ABC_PCIE_LINK_ERROR)) {
		mutex_lock(&pmu_ctx->pcie_link_lock);
		pmu_ctx->pcie_link_ready = false;
		mutex_unlock(&pmu_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;  /* Don't care */
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_ipu_sleep(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* IPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, (0x1 << 29));

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~0x1;
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((val & 0x1) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU down status\n");
		return -EBUSY;
	}

	return 0;
}

static int ab_pmu_ipu_sleep_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab ipu entering sleep\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_ipu_sleep(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_tpu_sleep(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	ab_sm_start_ts(AB_SM_TS_TPU_PREP_SLEEP);
	/* TPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, (0x1 << 29));

	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~0x2;
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	ab_sm_record_ts(AB_SM_TS_TPU_PREP_SLEEP);

	ab_sm_start_ts(AB_SM_TS_TPU_POLL_SLEEP);
	do {
		/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((val & 0x2) && --timeout > 0);

	ab_sm_record_ts(AB_SM_TS_TPU_POLL_SLEEP);
	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for TPU down status\n");
		return -EBUSY;
	}

	return 0;
}

static int ab_pmu_tpu_sleep_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab tpu entering sleep\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_tpu_sleep(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_ipu_tpu_sleep(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* IPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, (0x1 << 29));

	/* TPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, (0x1 << 29));

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~0x3;
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((val & 0x3) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU/TPU down status\n");
		return -EBUSY;
	}

	return 0;
}

static int ab_pmu_ipu_tpu_sleep_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab tpu entering sleep\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_ipu_tpu_sleep(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_deep_sleep_handler(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* IPU_CONTROLLER_OPTION[29:29] ENABLE_PWR_MANAGEMENT */
	ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, (0x1 << 29));

	/* TPU_CONTROLLER_OPTION[29:29] ENABLE_PWR_MANAGEMENT */
	ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, (0x1 << 29));

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~0x3;
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((val & 0x3) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout setting IPU/TPU to sleep before deep sleep\n");
		return -EBUSY;
	}

	/* PMU_CONTROL[2:2] DEEP_SLEEP */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x1 << 2);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	timeout = IPU_TPU_STATUS_TIMEOUT;
	do {
		/* PMU_STATUS[2:2] DEEP_SLEEP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while (((val & 0x7) != 0x4) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for deep_sleep set status\n");
		return -EBUSY;
	}

	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~(0x1 << 2);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	return 0;
}

static int ab_pmu_deep_sleep_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab entering deep sleep\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_deep_sleep_handler(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_ipu_resume_handler(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x1 << 0);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((!(val & 0x1)) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU up status\n");
		return -EBUSY;
	}

	// Enable IPU_IO power (See b/127824210)
	ABC_WRITE(0x10200068, 0x00);
	ABC_WRITE(0x1020006C, 0x00);

	// Select IPU PLL as parent clock of IPU block
	ABC_WRITE(CLK_CON_MUX_MOUT_IPU_AONCLK_PLLCLK, 0x1);

	return 0;
}

static int ab_pmu_ipu_resume_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab ipu resuming\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_ipu_resume_handler(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_tpu_resume_handler(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x2 << 0);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	do {
		/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((!(val & 0x2)) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for TPU up status\n");
		return -EBUSY;
	}

	// Select TPU PLL as parent clock of TPU block
	ABC_WRITE(CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK, 0x1);

	return 0;
}

static int ab_pmu_tpu_resume_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab tpu resuming\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_tpu_resume_handler(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold pmu_ctx->pcie_link_lock */
static int __ab_pmu_ipu_tpu_resume_handler(struct ab_pmu_context *pmu_ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x3 << 0);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((!(val & 0x2) || !(val & 0x1)) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU/TPU up status\n");
		return -EBUSY;
	}

	// Enable IPU_IO power (See b/127824210)
	ABC_WRITE(0x10200068, 0x00);
	ABC_WRITE(0x1020006C, 0x00);

	// Select IPU/TPU PLL as parent clocks of IPU/TPU blocks
	ABC_WRITE(CLK_CON_MUX_MOUT_IPU_AONCLK_PLLCLK, 0x1);
	ABC_WRITE(CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK, 0x1);

	return 0;
}

static int ab_pmu_ipu_tpu_resume_handler(void *ctx)
{
	int ret;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab ipu and tpu resuming\n");

	mutex_lock(&pmu_ctx->pcie_link_lock);
	if (pmu_ctx->pcie_link_ready) {
		ret = __ab_pmu_ipu_tpu_resume_handler(pmu_ctx);
	} else {
		dev_err(pmu_ctx->dev,
				"%s: pcie link down during pmu request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&pmu_ctx->pcie_link_lock);

	return ret;
}

static struct ab_sm_pmu_ops pmu_ops = {
	.pmu_ipu_sleep = &ab_pmu_ipu_sleep_handler,
	.pmu_tpu_sleep = &ab_pmu_tpu_sleep_handler,
	.pmu_ipu_tpu_sleep = &ab_pmu_ipu_tpu_sleep_handler,
	.pmu_deep_sleep = &ab_pmu_deep_sleep_handler,
	.pmu_ipu_resume = &ab_pmu_ipu_resume_handler,
	.pmu_tpu_resume = &ab_pmu_tpu_resume_handler,
	.pmu_ipu_tpu_resume = &ab_pmu_ipu_tpu_resume_handler,
};

static int ab_pmu_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct ab_pmu_context *pmu_ctx;

	pmu_ctx = devm_kzalloc(dev, sizeof(struct ab_pmu_context), GFP_KERNEL);
	if (!pmu_ctx)
		return -ENOMEM;

	pmu_ctx->dev = dev;

	mutex_init(&pmu_ctx->pcie_link_lock);
	pmu_ctx->pcie_link_ready = true;
	pmu_ctx->pcie_link_blocking_nb.notifier_call =
			ab_pmu_pcie_link_listener;
	ret = abc_register_pcie_link_blocking_event(
			&pmu_ctx->pcie_link_blocking_nb);
	if (ret) {
		dev_err(dev,
				"failed to subscribe to PCIe blocking link event, ret %d\n",
				ret);
		return ret;
	}

	pmu_ops.ctx = pmu_ctx;
	ab_sm_register_pmu_ops(&pmu_ops);

	ab_pmu_ipu_resume_handler(pmu_ctx);
	ab_pmu_tpu_resume_handler(pmu_ctx);

	return 0;
}

static int ab_pmu_remove(struct platform_device *pdev)
{
	ab_sm_unregister_pmu_ops();
	return 0;
}

static const struct of_device_id ab_pmu_of_match[] = {
		{ .compatible = "abc,airbrush-pmu", },
		{ },
};

static struct platform_driver ab_pmu_driver = {
	.probe = ab_pmu_probe,
	.remove = ab_pmu_remove,
	.driver = {
		.name = "ab-pmu",
		.of_match_table = ab_pmu_of_match,
	},
};
module_platform_driver(ab_pmu_driver);
