/*
 * iaxxx-core.c -- IAxxx Multi-Function Device driver
 *
 * Copyright 2018 Knowles Corporation
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
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/circ_buf.h>
#include <linux/clk.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-af.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debug.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-sysfs.h"
#include "iaxxx-tunnel.h"
#include "iaxxx-debug.h"
#include "iaxxx-cdev.h"
#include "iaxxx-build-info.h"
#include "iaxxx-btp.h"
#include "ia8508a-memory-map.h"

#include "iaxxx-misc.h"

#define IAXXX_RESET_RETRIES		5		/* retry attempts */
#define IAXXX_RESET_HOLD_TIME		(20*1000)	/* 20 ms */
#define IAXXX_RESET_READY_DELAY		(20*1000)	/* 20 ms */
#define IAXXX_RESET_RANGE_INTERVAL	100		/* 100 us */

#define IAXXX_RESET_PWR_VLD_DELAY		(3*1000)
#define IAXXX_RESET_PWR_VLD_RANGE_INTERVAL	100

/* 10 retries if failed */
#define IAXXX_FW_RETRY_COUNT		10
#define IAXXX_FW_RETRY_DELAY_MS	100
#define IAXXX_BYTES_IN_A_WORD		4
#define IAXXX_INT_OSC_TRIM_MASK	0x20000
#define IAXXX_INT_OSC_TRIM_POS	17
#define IAXXX_INT_OSC_CALIBRATION_MASK	0x7F
#define IAXXX_INT_OSC_CALIBRATION_POS	25
#define WAKEUP_TIMEOUT			5000
#define SPI_WAIT_TIMEOUT		3000

#define iaxxx_ptr2priv(ptr, item) container_of(ptr, struct iaxxx_priv, item)

struct iaxxx_port_clk_settings clk;
static const u32 iaxxx_port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_PORTD_CLK_ADDR,
	IAXXX_IO_CTRL_PORTE_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 iaxxx_port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTD_FS_ADDR,
	IAXXX_IO_CTRL_PORTE_FS_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 iaxxx_port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
	IAXXX_IO_CTRL_PORTE_DI_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 iaxxx_port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_PORTD_DO_ADDR,
	IAXXX_IO_CTRL_PORTE_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

/* Firmware download error code */
enum {
	E_IAXXX_REGMAP_ERROR = -1,
	E_IAXXX_BOOTUP_ERROR = -2,
	E_IAXXX_RESET_SYNC_ERROR = -3,
	E_IAXXX_BTP_ERROR = -4,
};

enum {
	IAXXX_DEV_SUSPEND = 0,
	IAXXX_DEV_RESUME = 1,
	IAXXX_DEV_SUSPENDING = 2,
};

/*===========================================================================
 * MFD Driver
 *===========================================================================
 */

static struct mfd_cell iaxxx_devices[] = {
	{
		.name = "iaxxx-codec",
	},
	{
		.name = "iaxxx-odsp-celldrv",
	},
	{
		.name = "iaxxx-tunnel-celldrv",
	},
	{
		.name = "iaxxx-module-celldrv",
	},
};


const char *iaxxx_get_processor_name(uint32_t proc_id)
{
	switch (proc_id) {
	case IAXXX_CM4_ID:
		return "CM4";

	case IAXXX_HMD_ID:
		return "HMD";

	case IAXXX_DMX_ID:
		return "DMX";

	case IAXXX_SSP_ID:
		return "SSP";

	default:
		return "Invalid Proc-id";
	}
}

/* Returns formatted crash error signature in the
 * error string, based on error and proc_id.
 */
void iaxxx_crash_err2str(char *err_str, int err_str_maxsize,
		int error, uint32_t proc_id)
{
	switch (error) {
	case IAXXX_FW_CRASH_EVENT:
		snprintf(err_str, err_str_maxsize,
			"%s crash",
			iaxxx_get_processor_name(proc_id));
		break;
	case IAXXX_FW_CRASH_UPDATE_BLOCK_REQ:
		snprintf(err_str, err_str_maxsize,
			"Update %s BLK timeout",
			iaxxx_get_processor_name(proc_id));
		break;

	case IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF:
		snprintf(err_str, err_str_maxsize,
			"forced recovery after wrong tunnel buff params");
		break;
	case IAXXX_FW_CRASH_RESUME:
		snprintf(err_str, err_str_maxsize,
			"forced recovery after resume failure");
		break;
	case IAXXX_FW_CRASH_SUSPEND:
		snprintf(err_str, err_str_maxsize,
			"forced recovery after suspend failure");
		break;

	case IAXXX_FW_CRASH_SIMULATED:
		snprintf(err_str, err_str_maxsize,
			"simulated crash for testing");
		break;
	default:
		snprintf(err_str, err_str_maxsize,
			"unknown error");
		break;
	}
}

/**
 * iaxxx_send_uevent - Send uevent KOBJ_CHANGE
 *
 * @priv	: iaxxx private data
 * @type	  Type of event
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type)
{
	char *event[2] = {type, NULL};

	/* Send recovery event to HAL */
	return kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
}

static int iaxxx_fw_bootup_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int rc;

	/* Initialize "application" regmap */
	rc = iaxxx_application_regmap_init(priv);
	if (rc)
		goto err_regmap;
	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_switch_regmap(dev, priv->regmap, priv->dfs_node);
	if (rc)
		dev_err(dev, "%s: Failed to create debugfs entry\n", __func__);
	else
		dev_info(dev, "%s: done\n", __func__);
	return rc;

err_regmap:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	return rc;
}

/* Clear system state */
static void clear_system_state(struct iaxxx_priv *priv)
{
	iaxxx_clr_pkg_plg_list(priv);
	priv->iaxxx_state->power_state = IAXXX_NOCHANGE;
	memset(&priv->iaxxx_state->err, 0, sizeof(struct iaxxx_block_err));
}

static int iaxxx_fw_recovery_regmap_init(struct iaxxx_priv *priv)
{
	int rc = 0;

	rc = iaxxx_regmap_drop_regions(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s() Failed to drop regmap regions: %d\n",
			__func__, rc);
		goto regmap_recovery_failed;
	}
	regcache_cache_bypass(priv->regmap, false);

	/* Clear system state */
	clear_system_state(priv);

	/* Reset route status */
	iaxxx_core_set_route_status(priv, false);
	priv->is_application_mode = true;
	dev_info(priv->dev, "%s: Recovery done\n", __func__);
regmap_recovery_failed:
	return rc;
}


static int iaxxx_do_suspend(struct iaxxx_priv *priv,
	bool is_fw_crash)
{
	bool suspended = false;


	/* Send broadcast event to all device children for crash */
	if (is_fw_crash)
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_CRASH, NULL);

	/* Start FW recovery if pending and is allowed, otherwise remember */
	if (is_fw_crash)
		iaxxx_work(priv, fw_crash_work);

	if (suspended)
		iaxxx_send_uevent(priv, "ACTION=IAXXX_SUSPEND_EVENT");

	return 0;
}

static int iaxxx_do_resume(struct iaxxx_priv *priv)
{
	unsigned long *flags = &priv->flags;
	bool resumed = false;

	if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_STARTUP, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_STARTUP, NULL);

		/* Send firmware startup event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_STARTUP_EVENT");
		iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_DWNLD_SUCCESS");
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_RECOVERY, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RECOVERY, NULL);

		/* Send recovery event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_RECOVERY_EVENT");
	}

	/* Send resume event to user space */
	if (resumed)
		iaxxx_send_uevent(priv, "ACTION=IAXXX_RESUME_EVENT");

	return 0;
}

/**
 * iaxxx_gpio_free - free a gpio for a managed device
 * @dev: device to free the gpio for
 * @gpio: GPIO to free
 */
static inline void iaxxx_gpio_free(struct device *dev, unsigned int gpio)
{
	devm_gpio_free(dev, gpio);
	dev_dbg(dev, "%s(): %d\n", __func__, gpio);
}

/**
 * get_named_gpio - Gets a GPIO property from a device tree node
 */
static inline int get_named_gpio(struct device *dev, const char *name)
{
	int rc;

	rc = of_get_named_gpio(dev->of_node, name, 0);
	if (rc < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			name, dev->of_node->full_name, rc);
		return rc;
	}
	dev_dbg(dev, "%s: %s %d\n", __func__, name, rc);
	return rc;
}

/**
 * iaxxx_populate_dt_gpios - populates GPIO data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_gpios(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	rc = get_named_gpio(dev, "adnc,reset-gpio");
	if (rc < 0) {
		priv->reset_gpio = -EINVAL;
		dev_err(dev, "Failed to read reset-gpio, rc = %d\n", rc);
		return rc;
	}
	priv->reset_gpio = rc;

	rc = get_named_gpio(dev, "adnc,event-gpio");
	if (rc < 0) {
		priv->event_gpio = -EINVAL;
		dev_err(dev, "Failed to read event-gpio gpio, rc = %d\n", rc);
		return rc;
	}
	priv->event_gpio = rc;

	rc = get_named_gpio(dev, "adnc,pwr-vld-gpio");
	if (rc < 0) {
		priv->pwr_vld_gpio = -EINVAL;
		dev_err(dev, "Failed to read pwr-vld-gpio gpio, rc = %d\n", rc);
		return rc;
	}
	priv->pwr_vld_gpio = rc;

	return 0;
}

/**
 * iaxxx_populate_dt_sensor_port - populates sensor port index from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_sensor_port(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	int rc = 0;

	if (np == NULL) {
		dev_err(dev, "Invalid of node\n");
		rc = -EINVAL;
		return rc;
	}

	rc = of_property_read_u32(np, "adnc,sensor-port", &priv->sensor_port);
	if (rc < 0) {
		dev_err(dev, "Failed to read sensor-port, rc = %d\n", rc);
		priv->sensor_port = PDM_PORTB;
		return rc;
	}
	dev_info(dev, "sensor-port: %d\n", priv->sensor_port);

	return rc;
}

/**
 * iaxxx_populate_dt_regulator - populates regulator data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_regulator(struct iaxxx_priv *priv,
		struct regulator **vreg, const char *reg_name)
{
	int rc;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct regulator *reg;
	char prop_name[32];
	u32 volt_uV[2], load_uA;

	dev_dbg(dev, "%s()\n", __func__);

	if (!reg_name)
		return -EINVAL;

	/* check if the regulator consumer node exists */
	snprintf(prop_name, sizeof(prop_name), "%s-supply", reg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "%s() property %s-supply not found.\n",
			__func__, reg_name);
		return 0;
	}

	/* get regulator */
	reg = devm_regulator_get(dev, reg_name);
	if (IS_ERR(reg)) {
		rc = PTR_ERR(reg);
		dev_err(dev, "%s: Failed to get regulator %s. rc=%d\n",
			__func__, reg_name, rc);
		goto err;
	}

	/* Read and set min/max regulator voltage */
	snprintf(prop_name, sizeof(prop_name), "%s-voltage-uV", reg_name);
	if (!of_property_read_u32_array(np, prop_name, volt_uV, 2)) {
		rc = regulator_set_voltage(reg, volt_uV[0], volt_uV[1]);
		if (rc < 0) {
			dev_err(dev, "%s: Failed to set voltage %s. rc=%d\n",
				__func__, reg_name, rc);
			goto err_put;
		}
	}

	/* Read and set max current load for this consumer */
	snprintf(prop_name, sizeof(prop_name), "%s-maxload-uA", reg_name);
	if (!of_property_read_u32(np, prop_name, &load_uA)) {
		rc = regulator_set_load(reg, load_uA);
		if (rc < 0) {
			dev_err(dev, "%s: Failed to set load %s. rc=%d\n",
				__func__, reg_name, rc);
			goto err_put;
		}
	}

	rc = regulator_enable(reg);
	if (rc < 0) {
		dev_err(dev,
			"Failed to enable regulator %s: %d\n", reg_name, rc);
		goto err_put;
	}

	dev_dbg(dev, "%s: regulator %s ready\n",
		__func__, reg_name);

	*vreg = reg;
	return 0;

err_put:
	devm_regulator_put(reg);
err:
	*vreg = NULL;
	return rc;
}

static int iaxxx_config_regulators(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* check and config vdd-io */
	rc = iaxxx_populate_dt_regulator(
			priv, &priv->vdd_io, "adnc,vdd-io");
	if (rc) {
		dev_err(dev,
			"Failed to configure regulator vdd_io, rc %d\n", rc);
		return rc;
	}

	/* Sleep for vdd_io to be stable before enabling vdd_core */
	if (priv->vdd_io)
		usleep_range(1000, 2000);

	/* check and config vdd-core */
	rc = iaxxx_populate_dt_regulator(
			priv, &priv->vdd_core, "adnc,vdd-core");
	if (rc) {
		dev_err(dev,
			"Failed to configure regulator vdd_core, rc %d\n", rc);
		return rc;
	}

	return 0;
}

static void iaxxx_ext_clk_off_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work, struct iaxxx_priv,
						ext_clk_off_work.work);
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s: mclk_en %d", __func__, priv->mclk_en);

	if (priv->mclk_en) {
		clk_disable_unprepare(priv->ext_clk);
		priv->mclk_en = false;
	}
	dev_info(dev, "%s mclk_en %d is off\n", __func__, priv->mclk_en);
}

/**
 * iaxxx_ext_clk_ctl - external clock control callback function
 */
static int iaxxx_ext_clk_ctl(struct iaxxx_priv *priv, bool clock_en)
{
	int ret = 0;
	struct device *dev = priv->dev;
	int cancel = 0;

	dev_dbg(dev, "%s: clock_en = %d\n", __func__, clock_en);

	if (!priv->ext_clk_off_work_initialized) {
		INIT_DELAYED_WORK(&priv->ext_clk_off_work,
				iaxxx_ext_clk_off_work);
		priv->ext_clk_off_work_initialized = true;
	}

	cancel = cancel_delayed_work_sync(&priv->ext_clk_off_work);

	if (clock_en == priv->mclk_en)
		return 0;

	if (clock_en) {
		clk_set_rate(priv->ext_clk, 19200000);
		ret = clk_prepare_enable(priv->ext_clk);
		if (ret == 0)
			priv->mclk_en = clock_en;
		/* Wait for Ext_CLK is enabled and stable */
		usleep_range(20000, 20100);
	} else {
		__pm_wakeup_event(&priv->ws, WAKEUP_TIMEOUT);
		schedule_delayed_work(&priv->ext_clk_off_work, HZ * 3);
	}

	dev_info(dev, "%s: mclk_en %d, cancel %d\n", __func__, priv->mclk_en,
		cancel);
	return ret;
}

/**
 * iaxxx_populate_ext_clock_pdata - populate clock data from device tree
 */
static int iaxxx_populate_ext_clock_pdata(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	priv->ext_clk = devm_clk_get(dev, "iaxxx_clk");
	if (IS_ERR(priv->ext_clk) &&
		(PTR_ERR(priv->ext_clk) == -EPROBE_DEFER)) {
		dev_info(dev, "%s: iaxxx_clk is not ready yet %ld\n",
			__func__, PTR_ERR(priv->ext_clk));
		priv->iaxxx_mclk_cb = NULL;
		rc = -EPROBE_DEFER;
	} else if (IS_ERR(priv->ext_clk)) {
		dev_err(dev, "%s: Failed to get iaxxx mclk from pmic %ld\n",
			__func__, PTR_ERR(priv->ext_clk));
		priv->iaxxx_mclk_cb = NULL;
		rc = PTR_ERR(priv->ext_clk);
	} else {
		dev_info(dev, "%s: Got iaxxx_clk from pmic %ld\n",
			__func__, PTR_ERR(priv->ext_clk));

		priv->iaxxx_mclk_cb = iaxxx_ext_clk_ctl;
		/* Enable mclock here if internal OSC not ON by default */
		priv->iaxxx_mclk_cb(priv, 1);
	}
	return rc;
}

/**
 * iaxxx_populate_dt_pdata - populate platform data from device tree
 */
static int iaxxx_populate_dt_pdata(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;
	u32 tmp;

	dev_dbg(dev, "%s()\n", __func__);

	if (!dev->of_node) {
		dev_err(dev, "Missing device tree information\n");
		return -ENODEV;
	}

	rc = iaxxx_populate_dt_gpios(priv);
	if (rc) {
		dev_err(dev, "Failed to read GPIO data, rc = %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(dev->of_node, "adnc,oscillator-mode", &tmp);
	if (rc < 0) {
		dev_err(dev,
			"no adnc,oscillator-mode in DT node: %d\n", rc);
		/* set to default ext oscillator */
		tmp = 0;
	}
	priv->oscillator_mode = tmp;

	rc = iaxxx_populate_dt_sensor_port(priv);
	if (rc)
		dev_warn(dev, "Failed to read sensor port, rc = %d\n", rc);

	rc = iaxxx_populate_ext_clock_pdata(priv);
	return rc;
}

/**
 * iaxxx_gpio_init(): Requests the GPIO for the device
 */
static int iaxxx_gpio_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* GPIO: event (input used for interrupts) */
	/* TODD: interrupt is active HIGH so the GPIO needs pull-down */
	rc = devm_gpio_request_one(dev, priv->event_gpio,
						GPIOF_DIR_IN, "EVENT");
	if (rc < 0)
		goto err_missing_event_gpio;

	/* GPIO: reset (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->reset_gpio,
						GPIOF_OUT_INIT_HIGH, "RESET");
	if (rc < 0)
		goto err_missing_reset_gpio;

	/* GPIO: pwr_vld (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->pwr_vld_gpio,
						GPIOF_OUT_INIT_HIGH, "RESET");
	if (rc < 0)
		goto err_missing_pwr_vld_gpio;

	return 0;
err_missing_pwr_vld_gpio:
	iaxxx_gpio_free(dev, priv->reset_gpio);
err_missing_reset_gpio:
	iaxxx_gpio_free(dev, priv->event_gpio);
err_missing_event_gpio:
	return rc;
}
/**
 * iaxxx_get_efuse_boot_values(): Get the Efuse values.
 */
static int iaxxx_get_efuse_boot_values(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int rc = 0;
	uint32_t efuse_boot0_val, efuse_boot1_val;

	rc = regmap_read(priv->regmap, IAXXX_PWR_MGMT_EFUSE_BOOT_0_ADDR,
			&efuse_boot0_val);
	if (rc) {
		dev_err(dev, "%s: failed to read the efuse boot0 val\n",
			__func__);
		goto efuse_read_fail;
	}
	dev_info(dev, "Efuse Boot0 Val 0x%x\n", efuse_boot0_val);
	dev_info(dev,
		"Int OSC TRIM programmed: %d, osc calibration val: 0x%02x",
		(efuse_boot0_val & IAXXX_INT_OSC_TRIM_MASK)
		>> IAXXX_INT_OSC_TRIM_POS,
		(efuse_boot0_val >> IAXXX_INT_OSC_CALIBRATION_POS) &
			IAXXX_INT_OSC_CALIBRATION_MASK);
	dev_info(dev, "%s() chip Layout Rev(%s)", __func__,
		((efuse_boot0_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_0_LAYOUT_REV_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_0_LAYOUT_REV_POS) ?
		"B" : "A");

	/* store iaxxx_misc hwinfo data */
	rc = scnprintf(priv->hwinfo.revision, sizeof(priv->hwinfo.revision),
		"%s", ((efuse_boot0_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_0_LAYOUT_REV_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_0_LAYOUT_REV_POS) ?
		"B" : "A");
	if (rc) {
		dev_warn(dev, "%s: cannot recognize revision\n",
			__func__);
	}

	rc = regmap_read(priv->regmap, IAXXX_PWR_MGMT_EFUSE_BOOT_1_ADDR,
			&efuse_boot1_val);
	if (rc) {
		dev_err(dev, "%s: failed to read the efuse boot1 val\n",
			__func__);
		goto efuse_read_fail;
	}
	dev_info(dev, "Efuse Boot1 Val 0x%x\n", efuse_boot1_val);
	dev_info(dev,
		"LDO_BG_TRIM: %d, LDO_BG_TRIM_DATA val: 0x%02x",
		((efuse_boot1_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_BG_TRIM_PG_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_BG_TRIM_PG_POS),
		(efuse_boot1_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_BG_TRIM_DATA_MASK));
	dev_info(dev,
		"LDO_0_TRIM: %d, LDO_1_TRIM: %d",
		((efuse_boot1_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_0_TRIM_DATA_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_0_TRIM_DATA_POS),
		((efuse_boot1_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_1_TRIM_DATA_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_1_TRIM_DATA_POS));

	/* store iaxxx_misc hwinfo data */
	priv->hwinfo.trim_ldo_bg = ((efuse_boot1_val &
			IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_BG_TRIM_PG_MASK)
		>> IAXXX_PWR_MGMT_EFUSE_BOOT_1_LDO_BG_TRIM_PG_POS);

efuse_read_fail:
	return rc;
}

static void dump_to_log(struct device *dev,
		struct iaxxx_reg_dump_priv *reg_dump,
		struct iaxxx_register_log *log)
{
	spin_lock(&reg_dump->ring_lock);
	/* Check if Log buffer has space, if not increment the tail index to
	 * get buffer to overwrite oldest data
	 */
	if (!(CIRC_SPACE(reg_dump->head, reg_dump->tail, IAXXX_BUF_MAX_LEN))) {
		dev_dbg(dev, "%s() Register log buffer is full\n",
				__func__);
		reg_dump->tail++;
		reg_dump->tail %= IAXXX_BUF_MAX_LEN;
	}
	reg_dump->log[reg_dump->head] = *log;
	reg_dump->head++;
	/* Align with the Ring Buffer boundary */
	reg_dump->head %= IAXXX_BUF_MAX_LEN;
	spin_unlock(&reg_dump->ring_lock);
}

void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
		bool op)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_reg_dump_priv *reg_dump = priv->reg_dump;
	struct iaxxx_register_log log;

	if (!priv->reg_dump)
		return;

	log.val = val;
	log.addr = reg;
	log.op = op;
	get_monotonic_boottime(&log.timestamp);
	/* Add the log into circular buffer */
	dump_to_log(dev, reg_dump, &log);
}

int iaxxx_get_version_str(struct iaxxx_priv *priv, uint32_t reg, char *verbuf,
								uint32_t len)
{
	struct device *dev = priv->dev;
	int rc;
	uint32_t addr;
	uint32_t i = 0;
	const uint32_t size = len;

	/* Read the version string address */
	rc = regmap_read(priv->regmap, reg, &addr);
	if (rc) {
		dev_err(dev, "%s() String address read failed %d\n",
			__func__, rc);
		return rc;
	}
	dev_dbg(dev, "%s() String address 0x%x\n", __func__, addr);

	if (!len || (len % IAXXX_BYTES_IN_A_WORD)) {
		dev_err(dev, "%s() Invalid len %d\n", __func__, len);
		return -EINVAL;
	}

	/* Read the FW string from address read above */
	while (len > 0) {
		rc = iaxxx_btp_read(priv, addr + i, &verbuf[i], 1,
				IAXXX_HOST_0);
		if (rc < 0) {
			dev_err(dev, "String Read fail addr 0x%x:%d\n",
				addr + i, rc);
			return -EIO;
		}
		/* Reached NULL character, FW version string ends here */
		if ((!verbuf[i]) || (!verbuf[i + 1]) ||
				(!verbuf[i + 2]) || (!verbuf[i + 3])) {
			dev_dbg(dev, "%s() String ends here\n", __func__);
			i += IAXXX_BYTES_IN_A_WORD;
			break;
		}
		/* 4 characters read, go for next 4 bytes to read */
		len -= IAXXX_BYTES_IN_A_WORD;
		i += IAXXX_BYTES_IN_A_WORD;
	}

	verbuf[size - 1] = '\0';
	print_hex_dump(KERN_INFO, "Version: ", DUMP_PREFIX_OFFSET, 32, 4,
			(void *)verbuf, i, true);
	/*
	 * If not reached end of buffer and buffer is not empty,
	 * then the Firmware version string is valid.
	 */
	if (len > 0 && verbuf[0] != '\0')
		return 0;
	return -EIO;
}

/**
 * iaxxx_reset(): Reset IAxxx device
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s(): reset iaxxx\n", __func__);

	if (!gpio_is_valid(priv->reset_gpio)) {
		dev_err(dev, "%s(): reset_gpio(%d) is an invalid gpio.\n",
						__func__, priv->reset_gpio);
		return -EIO;
	}
	if (!gpio_is_valid(priv->pwr_vld_gpio)) {
		dev_err(dev, "%s(): pwr_vld_gpio(%d) is an invalid gpio.\n",
						__func__, priv->pwr_vld_gpio);
		return -EIO;
	}

	gpio_set_value(priv->pwr_vld_gpio, 0);

	gpio_set_value(priv->reset_gpio, 0);
	usleep_range(IAXXX_RESET_HOLD_TIME,
		IAXXX_RESET_HOLD_TIME + IAXXX_RESET_RANGE_INTERVAL);

	gpio_set_value(priv->reset_gpio, 1);
	usleep_range(IAXXX_RESET_PWR_VLD_DELAY, IAXXX_RESET_PWR_VLD_DELAY +
		     IAXXX_RESET_PWR_VLD_RANGE_INTERVAL);

	gpio_set_value(priv->pwr_vld_gpio, 1);
	usleep_range(IAXXX_RESET_READY_DELAY, IAXXX_RESET_READY_DELAY +
		IAXXX_RESET_RANGE_INTERVAL);

	return 0;
}

/**
 * iaxxx_reset_to_sbl - Boots the chip hardware
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
int iaxxx_reset_to_sbl(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* Reset the chip */
	rc = iaxxx_reset(priv);
	if (rc)
		dev_err(dev, "%s: device reset failed\n", __func__);

	return rc;
}

/**
 * iaxxx_event_isr - Interrupt / Event handler
 *
 * @irq	 : interrupt number
 * @data : iaxxx private data
 */
static irqreturn_t iaxxx_event_isr(int irq, void *data)
{
	int rc;
	bool is_startup;
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;

	/* If ISR is disabled, return as handled */
	if (priv->debug_isr_disable)
		return IRQ_HANDLED;

	dev_info(priv->dev, "%s: IRQ %d\n", __func__, irq);

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto out;
	}
	if (!test_and_set_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
				&priv->flags)) {
		/* On any event always assume chip is awake */
		wake_up(&priv->wakeup_wq);
		dev_dbg(priv->dev,
			"%s: FW is expected to be in wakeup state\n", __func__);
	}

	queue_work(priv->event_workq, &priv->event_work_struct);
out:
	return IRQ_HANDLED;
}

/**
 * iaxxx_irq_init - Initialize interrupt handling
 *
 * @priv : iaxxx private data
 */
static int iaxxx_irq_init(struct iaxxx_priv *priv)
{
	int rc;

	if (!gpio_is_valid(priv->event_gpio))
		return -ENXIO;

	rc = request_threaded_irq(gpio_to_irq(priv->event_gpio), NULL,
		iaxxx_event_isr, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		"iaxxx-event-irq", priv);
	if (rc)
		return rc;
	rc = enable_irq_wake(gpio_to_irq(priv->event_gpio));
	if (rc < 0)
		pr_err("%s: enable_irq_wake() failed on %d\n", __func__, rc);

	/* disable the irq until fw is loaded */
	disable_irq(gpio_to_irq(priv->event_gpio));
	priv->is_irq_enabled = false;

	return rc;
}

/**
 * iaxxx_irq_exit - Frees the event IRQ
 *
 * @priv	: iaxxx private data
 */
static void iaxxx_irq_exit(struct iaxxx_priv *priv)
{
	if (gpio_is_valid(priv->event_gpio))
		free_irq(gpio_to_irq(priv->event_gpio), priv);
}

static int iaxxx_regdump_init(struct iaxxx_priv *priv)
{
	dev_dbg(priv->dev, "%s()", __func__);

	priv->dump_log = true;
	priv->reg_dump = kvzalloc(sizeof(struct iaxxx_reg_dump_priv),
					GFP_KERNEL);
	if (!priv->reg_dump)
		return -ENOMEM;

	priv->reg_dump->log = kvzalloc(sizeof(struct iaxxx_register_log) *
					IAXXX_BUF_MAX_LEN,
					GFP_KERNEL);
	if (!priv->reg_dump->log) {
		kvfree(priv->reg_dump);
		priv->reg_dump = NULL;
		return -ENOMEM;
	}

	spin_lock_init(&priv->reg_dump->ring_lock);
	return 0;
}

void iaxxx_regdump_exit(struct iaxxx_priv *priv)
{
	kvfree(priv->reg_dump->log);
	priv->reg_dump->log = NULL;
	kvfree(priv->reg_dump);
	priv->reg_dump = NULL;
}

static int iaxxx_reset_check_sbl_mode(struct iaxxx_priv *priv)
{
	uint32_t status;
	int ret = 0;
	int mode = 0;
	int mode_retry = 5;

	if (priv->reset_cb)
		priv->reset_cb(priv->dev);

	do {
		/* Verify that the device is in bootloader mode */
		ret = regmap_read(priv->regmap_no_pm,
				IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (ret)
			dev_err(priv->dev,
				"regmap_read failed, ret = %d\n", ret);

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(priv->dev,
			"System Status: 0x%.08X mode: 0x%.08X\n",
			status, mode);

		/* Give some time to device before read */
		usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);
	} while (!mode && mode_retry--);

	priv->boot_completed = false;
	priv->is_application_mode = false;
	test_and_clear_bit(IAXXX_FLG_CHIP_WAKEUP_HOST0,
			   &priv->flags);
	if (!mode && !mode_retry) {
		dev_err(priv->dev,
			"SBL SYS MODE retry expired in crash dump\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/**
 * iaxxx_do_fw_update - reset and sync target and do firmware update
 *
 * @priv	: iaxxx private data
 * Return
 *       0                    Success
 *       E_IAXXX_REGMAP_ERROR error accessing regmap
 *       E_IAXXX_BOOTUP_ERROR target bootup failure
 *
 */
static int iaxxx_do_fw_update(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t reg, mem_elec_ctrl_val;
	struct device *dev = priv->dev;

	rc = iaxxx_reset_check_sbl_mode(priv);
	if (rc) {
		dev_err(dev, "SBL sysmode check failed, rc = %d\n", rc);
		return rc;
	}

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_DEVICE_ID_ADDR,
			&reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "Device ID: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR,
			&reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "ROM Version: 0x%.08X\n", reg);


	/* Electrical control for Memory
	 * Value 2(Normal-Read 2) for read and write margins for ROMs.
	 * Value 0(Slowest Read, Slowest Write) for read and write margins
	 * for SRAMs
	 * Value 8(Medium Leakage) for Retention Voltage for PD8 memories.
	 *
	 *  This fix is required for FW download on FF part or else we get
	 *  checksum error after downloading it. It fixes chip
	 *  stability issues due to clock changes and heating issues.
	 *
	 */
	mem_elec_ctrl_val =
		((2 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_MASK) |
		((0 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_MASK) |
		((8 << IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_MASK);

	rc = regmap_write(priv->regmap_no_pm, IAXXX_AO_MEM_ELEC_CTRL_ADDR,
			mem_elec_ctrl_val);
	if (rc) {
		dev_err(dev, "Electrical control register failed, rc = %d\n",
				rc);
		return E_IAXXX_REGMAP_ERROR;
	}

	/* Boot the device into application mode */
	rc = iaxxx_bootup(priv);
	if (rc) {
		dev_err(dev, "bootup failed\n");
		return E_IAXXX_BOOTUP_ERROR;
	}

	/* Call SPI speed setup callback if exists */
	if (priv->spi_speed_setup)
		priv->spi_speed_setup(dev, priv->spi_app_speed);

	return 0;
}

static void iaxxx_crashlog_header_read(struct iaxxx_priv *priv)
{
	int i;
	int j = 0;
	int ret;
	int memdump_index;
	uint32_t debuglog_reg_phyaddr =
			iaxxx_conv_virtual_to_physical_register_address(
				priv,
				IAXXX_DEBUG_BLOCK_0_DEBUGLOG_ADDR_ADDR);
	uint32_t crashlog_reg_phyaddr =
			iaxxx_conv_virtual_to_physical_register_address(
				priv,
				IAXXX_DEBUG_BLOCK_0_CRASHLOG_ADDR_ADDR);

	struct iaxxx_crashlog_header *crashlog_header = priv->crashlog_header;

	/* Reading the debug log address and size */
	for (i = IAXXX_DBGLOG_CM4; i <= IAXXX_DBGLOG_DMX; i++) {
		crashlog_header[i].log_type = i;
		ret = priv->bulk_read(priv->dev,
			debuglog_reg_phyaddr + i * 8,
			&crashlog_header[i].log_addr, 2);
		if (ret != 2) {
			dev_err(priv->dev, "Log %d address @%x fail %d\n", i,
					debuglog_reg_phyaddr, ret);
			crashlog_header[i].log_addr = 0;
			crashlog_header[i].log_size = 0;
		}
	}
	/* Reading the crash log address and size */
	for (i = IAXXX_CRASHLOG_CM4; i <= IAXXX_CRASHLOG_DMX; i++) {
		crashlog_header[i].log_type = i;
		ret = priv->bulk_read(priv->dev,
			crashlog_reg_phyaddr + j * 8,
			&crashlog_header[i].log_addr, 2);
		j++;
		if (ret != 2) {
			dev_err(priv->dev, "Log %d address fail %d\n", i, ret);
			crashlog_header[i].log_addr = 0;
			crashlog_header[i].log_size = 0;
		}
	}

	/* Reading SSP memory log addresses and size that is already read
	 * before chip reset
	 */
	for (i = IAXXX_MEMDUMP_SSP_RAM0; i <= IAXXX_MEMDUMP_SSP_ROM0; i++) {
		memdump_index = i - IAXXX_MEMDUMP_SSP_RAM0;
		crashlog_header[i].log_type =
			priv->crashlog->ssp_memdumps[memdump_index].log_type;
		crashlog_header[i].log_addr =
			priv->crashlog->ssp_memdumps[memdump_index].log_addr;
		crashlog_header[i].log_size =
			priv->crashlog->ssp_memdumps[memdump_index].log_size;
	}

	/* Reading crash reason string */
	if (i == IAXXX_CRASH_REASON_LOG) {
		crashlog_header[i].log_type = i;
		crashlog_header[i].log_addr = 0;
		crashlog_header[i].log_size = IAXXX_CRASH_REASON_STR_MAX_SIZE;
	}

	for (i = 0; i < IAXXX_MAX_LOG; i++)
		dev_info(priv->dev, "addr 0x%x size 0x%x\n",
				crashlog_header[i].log_addr,
				crashlog_header[i].log_size);
}

static int iaxxx_dump_crashlogs(struct iaxxx_priv *priv)
{
	uint32_t buf_size;
	uint32_t data_written = 0;
	int i;
	uint32_t log_addr;
	uint32_t log_size;
	int ret;
	struct iaxxx_crashlog_header *crashlog_header = priv->crashlog_header;

	/* If memory already allocated */
	kvfree(priv->crashlog->log_buffer);
	priv->crashlog->log_buffer = NULL;
	/* Calculate total crash log dump size */
	buf_size = sizeof(struct iaxxx_crashlog_header) * IAXXX_MAX_LOG;
	for (i = 0; i < IAXXX_MAX_LOG; i++)
		buf_size += crashlog_header[i].log_size;
	priv->crashlog->log_buffer_size = buf_size;
	/* Allocate the memory */
	priv->crashlog->log_buffer = kvzalloc(buf_size, GFP_KERNEL);
	if (!priv->crashlog->log_buffer)
		return -ENOMEM;

	/* Collect the crashlogs into log buffer */
	for (i = 0; i < IAXXX_MAX_LOG; i++) {
		/* Copy header information */
		memcpy(priv->crashlog->log_buffer + data_written,
				&crashlog_header[i],
				sizeof(struct iaxxx_crashlog_header));
		data_written += sizeof(struct iaxxx_crashlog_header);
		log_addr = crashlog_header[i].log_addr;
		log_size = crashlog_header[i].log_size;
		/* If size of the log is 0 */
		if (!log_size)
			continue;
		/* Read the logs
		 * If it is for SSP, read from buffer where SSP memory
		 * is dumped into before chip reset.
		 */
		if ((i >= IAXXX_MEMDUMP_SSP_RAM0) &&
			(i <= IAXXX_MEMDUMP_SSP_ROM0)) {

			memcpy(priv->crashlog->log_buffer + data_written,
				&priv->crashlog->ssp_log_buffer[log_addr],
				log_size);

		} else if (i == IAXXX_CRASH_REASON_LOG) {
			iaxxx_crash_err2str(
				priv->crashlog->log_buffer + data_written,
				log_size, priv->fw_crash_reasons,
				priv->fw_crash_proc_id);
		} else {
			ret = priv->bulk_read(priv->dev,
				log_addr,
				priv->crashlog->log_buffer + data_written,
				log_size / sizeof(uint32_t));
			if (ret != log_size / sizeof(uint32_t)) {
				dev_err(priv->dev,
					"Not able to read Debug logs %d\n",
					ret);
				return ret;
			}
		}
		data_written += log_size;
	}
	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);
	return 0;
}
static int iaxxx_set_ssp_memdumps_data(struct iaxxx_priv *priv,
				uint32_t log_type, uint32_t data_written)
{
	int ret = 0;
	int memdump_index = 0;
	uint32_t log_addr = 0;
	uint32_t log_size = 0;

	if (log_type == IAXXX_MEMDUMP_SSP_RAM0) {
		memdump_index =
			IAXXX_MEMDUMP_SSP_RAM0 - IAXXX_MEMDUMP_SSP_RAM0;
		log_size =
			IAXXX_SSP_DRAM0_SYS_END - IAXXX_SSP_DRAM0_SYS_START;

		log_addr = IAXXX_SSP_DRAM0_SYS_START;
	} else if (log_type == IAXXX_MEMDUMP_SSP_RAM1) {
		memdump_index =
			IAXXX_MEMDUMP_SSP_RAM1 - IAXXX_MEMDUMP_SSP_RAM0;
		log_size =
			IAXXX_SSP_DRAM1_SYS_END - IAXXX_SSP_DRAM1_SYS_START;

		log_addr = IAXXX_SSP_DRAM1_SYS_START;
	} else if (log_type == IAXXX_MEMDUMP_SSP_ROM0) {
		memdump_index =
			IAXXX_MEMDUMP_SSP_ROM0 - IAXXX_MEMDUMP_SSP_RAM0;
		log_size =
			IAXXX_SSP_DROM0_SYS_END - IAXXX_SSP_DROM0_SYS_START;

		log_addr = IAXXX_SSP_DROM0_SYS_START;
	} else {
		dev_err(priv->dev, "Unknown log_type %u", log_type);
		ret = -EINVAL;
		goto exit;
	}

	ret =  priv->bulk_read(priv->dev, log_addr,
			priv->crashlog->ssp_log_buffer + data_written,
			log_size / sizeof(uint32_t));

	if (ret != log_size / sizeof(uint32_t)) {
		dev_err(priv->dev,
			"Not able to read SSP DRAM0 dump %d\n", ret);
		ret = -EINVAL;
		goto exit;
	}

	priv->crashlog->ssp_memdumps[memdump_index].log_type =
							log_type;
	priv->crashlog->ssp_memdumps[memdump_index].log_addr =
							data_written;
	priv->crashlog->ssp_memdumps[memdump_index].log_size =
							log_size;
exit:
	return ret;
}
static int iaxxx_dump_ssplogs(struct iaxxx_priv *priv)
{
	int ret;
	uint32_t buf_size;
	uint32_t data_written = 0;

	priv->crashlog->ssp_log_size = 0;

	buf_size = (IAXXX_SSP_DRAM0_SYS_END - IAXXX_SSP_DRAM0_SYS_START) +
		(IAXXX_SSP_DRAM1_SYS_END - IAXXX_SSP_DRAM1_SYS_START) +
		(IAXXX_SSP_DROM0_SYS_END - IAXXX_SSP_DROM0_SYS_START);

	/* Allocate the memory */
	if (!priv->crashlog->ssp_log_buffer)
		priv->crashlog->ssp_log_buffer = kvzalloc(buf_size, GFP_KERNEL);
	if (!priv->crashlog->ssp_log_buffer) {
		dev_err(priv->dev, "%s Failed to allocate mem for SSP dump\n",
				__func__);
		return -ENOMEM;
	}

	ret = iaxxx_set_ssp_memdumps_data(priv,
			IAXXX_MEMDUMP_SSP_RAM0, data_written);

	if (ret < 0)
		return ret;

	data_written += ret * sizeof(uint32_t);

	ret = iaxxx_set_ssp_memdumps_data(priv,
			IAXXX_MEMDUMP_SSP_RAM1, data_written);

	if (ret < 0)
		return ret;

	data_written += ret * sizeof(uint32_t);

	ret = iaxxx_set_ssp_memdumps_data(priv,
			IAXXX_MEMDUMP_SSP_ROM0, data_written);

	if (ret < 0)
		return ret;

	data_written += ret * sizeof(uint32_t);

	priv->crashlog->ssp_log_size = buf_size;
	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);
	return 0;
}

/**
 * iaxxx_fw_update_work - worker thread to download firmware.
 *
 * @work : used to retrieve private structure
 *
 * Firmware download and switch to application mode of firmware.
 */
static void iaxxx_fw_update_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_update_work);
	struct device *dev = priv->dev;
	bool is_startup;
	int rc;
	uint32_t efuse_trim_value;

	rc = iaxxx_wait_dev_resume(dev);
	if (rc) {
		if (rc == -EAGAIN)
			iaxxx_work(priv, fw_update_work);
		else if (rc == -ETIME)
			dev_err(dev, "%s: wait resume fail\n", __func__);
		return;
	}

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);

	rc = iaxxx_do_fw_update(priv);
	if (rc == E_IAXXX_REGMAP_ERROR) {
		goto exit_fw_fail;
	} else if (rc == E_IAXXX_BOOTUP_ERROR) {

		/* If there's device reset cb, retry */
		if (!priv->reset_cb) {
			dev_err(dev, "%s: Chip failed to boot up\n", __func__);
			goto exit_fw_fail;
		}

		if (++priv->try_count < IAXXX_FW_RETRY_COUNT) {
			dev_err(dev, "%s: Bootup error. retrying... %d\n",
				__func__, priv->try_count);
			msleep(IAXXX_FW_RETRY_DELAY_MS);
			iaxxx_work(priv, fw_update_work);
			return;
		}

		dev_err(dev, "%s: %d retry failed! EXIT\n",
			__func__, IAXXX_FW_RETRY_COUNT);
		goto exit_fw_fail;
	}

	priv->try_count = 0;

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto exit_fw_fail;
		priv->boot_completed = true;
	}

	/* Send firmware ready event to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_READY_EVENT");

	set_bit(IAXXX_FLG_FW_READY, &priv->flags);
	priv->iaxxx_state->power_state = IAXXX_NORMAL_MODE;

	iaxxx_fw_notifier_call(dev, IAXXX_EV_APP_MODE, NULL);

	if (test_and_clear_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		set_bit(IAXXX_FLG_RESUME_BY_RECOVERY, &priv->flags);
	else {
		set_bit(IAXXX_FLG_RESUME_BY_STARTUP, &priv->flags);
		rc = mfd_add_devices(priv->dev, -1, iaxxx_devices,
			ARRAY_SIZE(iaxxx_devices), NULL, 0, NULL);
		if (rc) {
			dev_err(priv->dev, "Failed to add cell devices\n");
			goto exit_fw_fail;
		}
	}

	rc = iaxxx_get_efuse_boot_values(priv);
	if (rc) {
		dev_err(dev, "%s: failed to read the efuse values\n", __func__);
		goto exit_fw_fail;
	}

	rc = regmap_read(priv->regmap, IAXXX_PLUGIN_HDR_COUNT_ADDR,
			&priv->plugin_inst_count);
	if (rc) {
		dev_err(dev, "%s: failed to read the plugin count\n",
				__func__);
		goto exit_fw_fail;
	}

	/* Subscribing for FW crash event for HOST_0 (AP) */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
			IAXXX_CRASH_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc) {
		dev_err(dev, "%s: failed to subscribe for crash event\n",
				__func__);
		goto exit_fw_fail;
	}

	/* Subscribing for FW crash event for HOST_1 */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
				IAXXX_CRASH_EVENT_ID, IAXXX_SYSID_HOST_1, 0);
	if (rc) {
		dev_err(dev, "%s: failed to subscribe for crash event HOST_1\n",
			__func__);
		goto exit_fw_fail;
	}

	/* Disable control interface 1 */
	rc =  iaxxx_pm_set_optimal_power_mode_host1(dev, false);
	if (rc) {
		dev_err(priv->dev,
		"%s() disabling controle interface 1 Fail\n", __func__);
		goto exit_fw_fail;
	}

	/* switch to internal oscillator if dt entry
	 * is selected for internal oscillator mode
	 * during bootup or recovery.
	 */
	if (priv->oscillator_mode) {
		rc = iaxxx_set_mpll_source(priv, IAXXX_INT_OSC);
		if (rc) {
			dev_err(dev,
			"%s: failed to switch to internal oscillator mode\n",
			__func__);
			goto exit_fw_fail;
		}
	}

	/* Read debug/crash log address after fw boot */
	iaxxx_dump_ssplogs(priv);
	iaxxx_crashlog_header_read(priv);

	dev_info(dev, "%s: done\n", __func__);
	iaxxx_work(priv, runtime_work);
	regmap_read(priv->regmap, IAXXX_AO_EFUSE_BOOT_ADDR, &efuse_trim_value);
	if (!efuse_trim_value) {
		dev_err(dev, "efuse_trim_value programmed by host\n");
		rc = regmap_update_bits(priv->regmap, IAXXX_AO_OSC_CTRL_ADDR,
			IAXXX_AO_OSC_CTRL_ADJ_MASK, 0x68);
	}
	regmap_read(priv->regmap, IAXXX_AO_OSC_CTRL_ADDR, &efuse_trim_value);
	dev_err(dev, "IAXXX_AO_OSC_CTRL_ADDR: 0x%x\n", efuse_trim_value);
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	/* Subscribing for FW wakeup event */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
			IAXXX_HOST0_WAKEUP_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc) {
		dev_err(dev,
			"%s: failed to subscribe for wakeup event\n",
			__func__);
		goto exit_fw_fail;
	}

	if (priv->pm_ws_enable) {
		priv->pm_ws_enable = false;
		__pm_relax(&priv->pm_ws);
	}

	iaxxx_pm_enable(priv);
#endif

	return;

exit_fw_fail:

	/* Send firmware fail uevent to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_FAIL_EVENT");

	/* Clear try counter */
	priv->try_count = 0;

}

static void iaxxx_fw_crash_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_crash_work);
	int ret;
	uint32_t core_crashed = 0;
	struct device *dev = priv->dev;

	ret = iaxxx_wait_dev_resume(dev);
	if (ret) {
		if (ret == -EAGAIN)
			iaxxx_work(priv, fw_crash_work);
		else if (ret == -ETIME)
			dev_err(dev, "%s: wait resume fail\n", __func__);
		return;
	}

	priv->crash_count++;
	dev_info(priv->dev, "iaxxx %d time crashed\n",
			priv->crash_count);

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	/* Disable runtime pm*/
	if (pm_runtime_enabled(priv->dev))
		pm_runtime_disable(priv->dev);
#endif

	/* Clear event queue */
	if (gpio_is_valid(priv->event_gpio) && priv->is_irq_enabled) {
		disable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = false;
	}

	mutex_lock(&priv->event_queue_lock);
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);

	atomic_set(&priv->fli_route_status, 0);

	if (priv->cm4_crashed) {
		dev_info(priv->dev, "CM4 Core crashed\n");
		priv->cm4_crashed = false;
	} else {
		ret = regmap_read(priv->regmap_no_pm,
				IAXXX_SRB_PROCESSOR_CRASH_STATUS_ADDR,
				&core_crashed);
		/* Crash status read fails, means CM4 core crashed */
		if (ret) {
			dev_info(priv->dev,
				"Crash status read fail %s()\n", __func__);
			core_crashed = IAXXX_CM4_ID;
		}
		if (core_crashed == IAXXX_CM4_ID) {
			dev_info(priv->dev, "D4100S CM4 core crashed\n");
			priv->cm4_crashed = false;
		} else if (core_crashed == 1 << IAXXX_HMD_ID)
			dev_info(priv->dev, "D4100S HMD core crashed\n");
		else if (core_crashed == 1 << IAXXX_DMX_ID)
			dev_info(priv->dev, "D4100S DMX Core Crashed\n");
		else if (core_crashed == 1 << IAXXX_SSP_ID)
			dev_info(priv->dev, "D4100S SSP Core Crashed\n");
		else
			dev_info(priv->dev, "D4100S Update block failed\n");
	}

	mutex_lock(&priv->crashdump_lock);
	iaxxx_reset_check_sbl_mode(priv);
	iaxxx_dump_crashlogs(priv);
	mutex_unlock(&priv->crashdump_lock);
	/* Notify the user about crash and read crash dump log*/
	iaxxx_send_uevent(priv, "ACTION=IAXXX_CRASH_EVENT");
	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);
	iaxxx_work(priv, fw_update_work);
}

static void iaxxx_runtime_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, runtime_work);
	unsigned long *flags = &priv->flags;
	bool is_fw_crash, do_suspend;

	/* Suspend due FW crash pending or aborted recovery */
	is_fw_crash = test_bit(IAXXX_FLG_FW_CRASH, flags);

	do_suspend = is_fw_crash;

	if (do_suspend)
		iaxxx_do_suspend(priv, is_fw_crash);
	else
		iaxxx_do_resume(priv);
}

/**
 * iaxxx_fw_update_test_work - work thread for running firmware test
 *
 * @work : used to retrieve Transport Layer private structure
 * Set the regmap to sbl mode and firmware update
 * If firmware update successful, then set bootup_done
 *
 */
static void iaxxx_fw_update_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_fw_update_test_work);
	int rc;

	priv->test_result = false;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);

	if (!rc)
		priv->test_result = true;

	complete_all(&priv->bootup_done);
}

int iaxxx_fw_crash(struct device *dev, enum iaxxx_fw_crash_reasons reasons,
		uint32_t proc_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);


	dev_err(priv->dev, "FW Crash occurred: reason:%d proc_id:%u\n",
			reasons, proc_id);

	if (priv->debug_fwcrash_handling_disable) {
		dev_err(priv->dev, "FW Crash Handling Skipped!\n");
		return 0;
	}

	/* Avoid second times if currently is handled */
	if (test_and_set_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EBUSY;

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);

	/* The following 2 variables are accessed
	 * only in this crash/recovery path, hence
	 * protection of concurrency is not needed.
	 */
	priv->fw_crash_reasons = reasons;
	priv->fw_crash_proc_id = proc_id;

	iaxxx_work(priv, runtime_work);
	return 0;
}

int iaxxx_fw_reset(struct iaxxx_priv *priv)
{

	if (test_and_set_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EBUSY;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	/* Disable runtime pm*/
	if (pm_runtime_enabled(priv->dev))
		pm_runtime_disable(priv->dev);
#endif

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);
	/* Clear event queue */
	if (gpio_is_valid(priv->event_gpio) && priv->is_irq_enabled) {
		disable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = false;
	}

	iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_FW_RESET, NULL);
	mutex_lock(&priv->event_queue_lock);
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);
	atomic_set(&priv->fli_route_status, 0);
	iaxxx_reset_check_sbl_mode(priv);
	regcache_cache_bypass(priv->regmap, true);
	iaxxx_work(priv, fw_update_work);
	return 0;
}

/*
 * iaxxx_abort_fw_recovery - abort current FW loading works
 *
 * @priv - context structure of driver
 *
 * FW Recovery procedure take more then 4sec.  When entering privacy mode,
 * we block the notifier call chain for the entire suspend time. Current
 * FW load procedure must be aborted. And started again when privacy complete
 *
 * TODO: This function is currently not being use and should be
 * removed.
 *
 */
int iaxxx_abort_fw_recovery(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	/* Not need abort if device is in active state */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		return -EPERM;

	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EPERM;

	dev_info(dev, "Aborting FW recovery...\n");

	iaxxx_work_flush(priv, runtime_work);
	iaxxx_work_flush(priv, fw_crash_work);
	iaxxx_work_flush(priv, fw_update_work);

	dev_info(dev, "FW recovery aborted!\n");

	return 0;
}

/**
 * iaxxx_cmem_test_work - work thread for running memory test
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Wait for an event interrupt happening with timeout
 *
 */
static void iaxxx_cmem_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_cmem_test_work);
	int rc;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);
}

/**
 * iaxxx_test_init - initialize work items and mutex for test environment
 *
 * @priv	: iaxxx private data
 *
 */
static void iaxxx_test_init(struct iaxxx_priv *priv)
{
	INIT_WORK(&priv->dev_fw_update_test_work, iaxxx_fw_update_test_work);
	INIT_WORK(&priv->dev_cmem_test_work, iaxxx_cmem_test_work);
	mutex_init(&priv->test_mutex);
	init_completion(&priv->bootup_done);
	init_completion(&priv->cmem_done);
}

int iaxxx_core_suspend_rt(struct device *dev)
{
	int rc = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!iaxxx_is_firmware_ready(priv))
		/* return -EBUSY; */
		goto exit;
	if (!pm_runtime_enabled(dev) && !test_bit(IAXXX_FLG_FW_CRASH,
		&priv->flags)) {
		dev_err(dev, "RT suspend requested while PM is not enabled\n");
		rc = -EINVAL;
		goto exit;
	}
	/* Chip suspend/optimal power switch happens here
	 * and they shouldn't be done in case of fw_crash
	 */

	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		rc = iaxxx_suspend_chip(priv);

	if  (rc) {
		dev_err(dev, " Chip suspend failed in %s\n", __func__);
		priv->in_suspend = 0;
	}

exit:
	if (priv->pm_ws_enable) {
		priv->pm_ws_enable = false;
		__pm_relax(&priv->pm_ws);
	}
	return rc;
}

int iaxxx_core_resume_rt(struct device *dev)
{
	int rc = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!iaxxx_is_firmware_ready(priv))
		/*return -EBUSY;*/
		goto exit;

	/* Regmap access must be enabled before enable event interrupt */

	if (!priv->pm_ws_enable) {
		priv->pm_ws_enable = true;
		__pm_stay_awake(&priv->pm_ws);
	}

	rc = iaxxx_wakeup_chip(priv);
	if  (rc) {
		dev_err(dev, " Chip wakeup failed in %s\n", __func__);
		priv->in_resume = 0;
		goto exit;
	}

	if (!pm_runtime_enabled(dev)) {
		/* This can happen during crash recovery, it's not an error
		 * condition
		 */
		dev_info(dev, "Resume requested while PM is not enabled\n");
	}

	priv->in_resume = 0;
exit:
	if (rc && priv->pm_ws_enable) {
		priv->pm_ws_enable = false;
		__pm_relax(&priv->pm_ws);
	}
	return rc;
}

int iaxxx_core_dev_suspend(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	atomic_set(&priv->pm_resume, IAXXX_DEV_SUSPENDING);
	flush_work(&priv->event_work_struct);
	iaxxx_tunnel_kthread_suspend(priv);
	iaxxx_flush_kthread_worker(&priv->worker);
	atomic_set(&priv->pm_resume, IAXXX_DEV_SUSPEND);

	return 0;
}

int iaxxx_core_dev_resume(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	atomic_set(&priv->pm_resume, IAXXX_DEV_RESUME);
	wake_up(&priv->irq_wake);

	return 0;
}

int iaxxx_wait_dev_resume(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc;
	int ret = 0;

	mutex_lock(&priv->resume_mutex);

	__pm_wakeup_event(&priv->ws, WAKEUP_TIMEOUT);

	if (atomic_read(&priv->pm_resume) == IAXXX_DEV_SUSPEND) {
		rc = wait_event_timeout(priv->irq_wake,
			atomic_read(&priv->pm_resume),
			msecs_to_jiffies(SPI_WAIT_TIMEOUT));

		if (!rc && !atomic_read(&priv->pm_resume)) {
			dev_err(priv->dev,
			"Wait resume timeout!, rc = %d\n", rc);
			ret = -ETIME;
		}
	} else if (atomic_read(&priv->pm_resume) == IAXXX_DEV_SUSPENDING) {
		ret = -EAGAIN;
	} else if (atomic_read(&priv->pm_resume) != IAXXX_DEV_RESUME) {
		dev_err(dev, "%s: flag value invalid\n", __func__);
		ret = -EINVAL;
	}

	mutex_unlock(&priv->resume_mutex);

	return ret;
}


/**
 * iaxxx_device_power_init - init power
 *
 * @priv: iaxxx private data
 */
static int iaxxx_device_power_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* Initialize the platform data */
	rc = iaxxx_populate_dt_pdata(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize platform data: %d\n", rc);
		goto err_populate_pdata;
	}

	rc = iaxxx_config_regulators(priv);
	if (rc) {
		dev_err(dev,
			"Failed to configure regulators %d\n", rc);
		goto err_enable_regulator;
	}

	/* Initialize the GPIOs */
	rc = iaxxx_gpio_init(priv);
	if (rc) {
		dev_err(dev, "Failed to initialize GPIOs: %d\n", rc);
		goto err_gpio_init;
	}

	/* Initialize interrupts */
	rc = iaxxx_irq_init(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize interrupts: %d\n", rc);
		goto err_irq_init;
	}

	return 0;

err_irq_init:
err_gpio_init:
err_enable_regulator:
err_populate_pdata:
	return rc;
}

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_device_reset - called from probe to perform chip reset
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_reset(struct iaxxx_priv *priv)
{
	/* Pull the device out of reset. */
	return iaxxx_reset_to_sbl(priv);
}

int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_register(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_unregister(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	return srcu_notifier_call_chain(&priv->core_notifier_list, val, v);
}

static void iaxxx_mutex_init(struct iaxxx_priv *priv)
{
	/* Init mutexes */
	mutex_init(&priv->update_block_lock);
	mutex_init(&priv->event_work_lock);
	mutex_init(&priv->event_queue_lock);
	mutex_init(&priv->plugin_lock);
	mutex_init(&priv->module_lock);
	mutex_init(&priv->sensor_tunnel_dev_lock);
	mutex_init(&priv->crashdump_lock);
	mutex_init(&priv->iaxxx_state->plg_pkg_list_lock);
	mutex_init(&priv->event_lock);
	mutex_init(&priv->proc_on_off_lock);
	mutex_init(&priv->btp_lock);
	mutex_init(&priv->debug_mutex);
	mutex_init(&priv->resume_mutex);
}

static void iaxxx_mutex_destroy(struct iaxxx_priv *priv)
{
	/* Destroy mutexes */
	mutex_destroy(&priv->update_block_lock);
	mutex_destroy(&priv->event_work_lock);
	mutex_destroy(&priv->event_queue_lock);
	mutex_destroy(&priv->plugin_lock);
	mutex_destroy(&priv->module_lock);
	mutex_destroy(&priv->sensor_tunnel_dev_lock);
	mutex_destroy(&priv->crashdump_lock);
	mutex_destroy(&priv->iaxxx_state->plg_pkg_list_lock);
	mutex_destroy(&priv->event_lock);
	mutex_destroy(&priv->proc_on_off_lock);
	mutex_destroy(&priv->btp_lock);
	mutex_destroy(&priv->debug_mutex);
	mutex_destroy(&priv->resume_mutex);
}

/**
 * iaxxx_device_init - called from probe to perform device initialization
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_init(struct iaxxx_priv *priv)
{
	int rc;

	/*
	 * Make the device power up the chip first so that
	 * the knowles chip won't cause any i2c communication error
	 * for other devices on the same bus
	 */
	rc = iaxxx_device_power_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to power up device: %d\n", rc);
		goto err_power_init;
	}

	iaxxx_mutex_init(priv);
	INIT_LIST_HEAD(&priv->static_mem_blks_list);

	iaxxx_init_kthread_worker(&priv->worker);
	init_waitqueue_head(&priv->boot_wq);
	init_waitqueue_head(&priv->wakeup_wq);
	init_waitqueue_head(&priv->irq_wake);
	atomic_set(&priv->pm_resume, IAXXX_DEV_RESUME);
	wakeup_source_init(&priv->ws, "iaxxx-spi");
	wakeup_source_init(&priv->pm_ws, "iaxxx-pm");
	priv->pm_ws_enable = false;

	priv->thread = kthread_run(kthread_worker_fn, &priv->worker,
				   "iaxxx-core");
	if (IS_ERR(priv->thread)) {
		dev_err(priv->dev, "Can't create kthread worker: %ld\n",
			PTR_ERR(priv->thread));
		return PTR_ERR(priv->thread);
	}

	iaxxx_init_kthread_work(&priv->fw_update_work, iaxxx_fw_update_work);
	iaxxx_init_kthread_work(&priv->fw_crash_work, iaxxx_fw_crash_work);
	iaxxx_init_kthread_work(&priv->runtime_work, iaxxx_runtime_work);

	INIT_LIST_HEAD(&priv->iaxxx_state->plugin_head_list);
	INIT_LIST_HEAD(&priv->iaxxx_state->pkg_head_list);

	atomic_set(&priv->fli_route_status, 0);

	/* Initialize regmap for SBL */
	rc = iaxxx_regmap_init(priv);
	if (rc)
		goto err_regdump_init;

	/* Initialize the register dump */
	rc = iaxxx_regdump_init(priv);
	if (rc)
		goto err_regdump_init;


	/* Create sysfs */
	rc = iaxxx_init_sysfs(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s [ERROR] sysfs_create_group\n", __func__);
	}

	/* TODO: SYSTEM_ROM_VER_STR */

	/* Initialize test environment */
	iaxxx_test_init(priv);

	/* Initialize the cdev interface */
	rc = iaxxx_cdev_init();
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize cdev interface: %d\n", rc);
		goto err_debug_init;
	}

	/* Initialize the debug interface */
	rc = iaxxx_debug_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize debug interface: %d\n", rc);
		goto err_debug_init;
	}

	srcu_init_notifier_head(&priv->core_notifier_list);

	/* Init early stage for tunneling */
	rc = iaxxx_tunnel_dev_init_early(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);
		goto err_debug_init;
	}

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_add_regmap(priv->dev, priv->regmap, &priv->dfs_node);
	if (rc)
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);

	rc = iaxxx_event_init(priv);
	if (rc) {
		dev_err(priv->dev, "Failed to initialize the event\n");
		goto err_event_init;
	}

	 priv->crashlog_header = kvzalloc((sizeof(struct
			iaxxx_crashlog_header) * IAXXX_MAX_LOG), GFP_KERNEL);
	iaxxx_work(priv, fw_update_work);

	/* add misc device for hardware statistics */
	rc = iaxxx_misc_init(priv);
	if (rc)
		dev_err(priv->dev, "Failed to initialize iaxxx_misc device\n");

	return 0;

err_event_init:
	iaxxx_event_exit(priv);
err_debug_init:
	iaxxx_remove_sysfs(priv);
	iaxxx_regdump_exit(priv);
err_regdump_init:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
	}
	if (priv->regmap_no_pm)
		regmap_exit(priv->regmap_no_pm);

	iaxxx_mutex_destroy(priv);
err_power_init:
	return rc;
}

void iaxxx_device_exit(struct iaxxx_priv *priv)
{
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	pm_runtime_disable(priv->dev);
#endif
	if (test_and_clear_bit(IAXXX_FLG_STARTUP, &priv->flags))
		mfd_remove_devices(priv->dev);

	if (priv->vdd_io) {
		regulator_disable(priv->vdd_io);
		devm_regulator_put(priv->vdd_io);
	}

	if (priv->vdd_core) {
		regulator_disable(priv->vdd_core);
		devm_regulator_put(priv->vdd_core);
	}

	iaxxx_misc_exit(priv);

	/* Delete the work queue */
	flush_work(&priv->event_work_struct);
	destroy_workqueue(priv->event_workq);

	iaxxx_flush_kthread_worker(&priv->worker);
	kthread_stop(priv->thread);

	iaxxx_mutex_destroy(priv);

	iaxxx_remove_sysfs(priv);
	iaxxx_regdump_exit(priv);
	iaxxx_irq_exit(priv);
	iaxxx_debug_exit(priv);
	iaxxx_cdev_exit();
	iaxxx_event_exit(priv);

	if (priv->regmap_config->ranges) {
		devm_kfree(priv->dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = NULL;
	}

	if (priv->regmap_no_pm_config->ranges) {
		devm_kfree(priv->dev,
				(void *)priv->regmap_no_pm_config->ranges);
		priv->regmap_no_pm_config->ranges = NULL;
	}

	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	if (priv->crashlog_header) {
		kvfree(priv->crashlog_header);
		priv->crashlog_header = NULL;
	}

}
