/*
 * iaxxx-i2c.c -- Generic i2c driver for Knowles IAxxx device
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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include "iaxxx.h"

#if defined(CONFIG_ARCH_MSM)
#define IAXXX_I2C_LOAD_UA	30000
#define IAXXX_I2C_VTG_MIN_UV	1800000
#define IAXXX_I2C_VTG_MAX_UV	1800000
#endif

/**
 * Description of driver private data
 *
 * @priv: IAxxx private data
 * @i2c:  i2c client pointer
 */
struct iaxxx_i2c_priv {
	struct iaxxx_priv priv;	/* private data */
	struct i2c_client *i2c;

#if defined(CONFIG_ARCH_MSM)
	bool i2c_pull_up;
	struct regulator *vcc_i2c;
#endif
};

static inline struct iaxxx_i2c_priv *to_i2c_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_i2c_priv, priv) : NULL;
}

static int iaxxx_i2c_read(struct i2c_client *i2c, void *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf =  buf,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&i2c->dev, "i2c_transfer() failed, rc = %d\n", ret);
		return ret > 0 ? -EIO : ret;
	}

	return 0;
}

static int iaxxx_i2c_write(struct i2c_client *i2c, const void *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = len,
			.buf =  (void *)buf,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&i2c->dev, "i2c_transfer() failed, rc = %d\n", ret);
		return ret > 0 ? -EIO : ret;
	}

	return 0;
}

static int iaxxx_i2c_bus_raw_read(struct iaxxx_priv *priv, void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct i2c_client *i2c = to_i2c_client(dev);
	int rc;

	rc = iaxxx_i2c_read(i2c, buf, len);
	return rc;
}

static int iaxxx_i2c_bus_raw_write(struct iaxxx_priv *priv, const void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct i2c_client *i2c = to_i2c_client(dev);
	int rc;

	rc = iaxxx_i2c_write(i2c, buf, len);
	return rc;
}

static int iaxxx_i2c_cmd(struct i2c_client *i2c, u32 cmd, u32 *resp)
{
	int ret;

	dev_dbg(&i2c->dev, "iaxxx: cmd = 0x%08x\n", cmd);
	cmd = cpu_to_be32(cmd);
	dev_dbg(&i2c->dev, "iaxxx: cmd = 0x%08x\n", cmd);

	ret = iaxxx_i2c_write(i2c, &cmd, sizeof(cmd));
	if (ret) {
		dev_err(&i2c->dev, "Failed to send command 0x%.08X\n", cmd);
		return ret;
	}

	if (resp) {
		usleep_range(4000, 4500);
		ret = iaxxx_i2c_read(i2c, resp, sizeof(*resp));
		if (ret) {
			dev_err(&i2c->dev, "Failed to read command response\n");
			return ret;
		}
	}

	return 0;
}

/* Register map initialization */
static int iaxxx_i2c_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev;
	struct regmap *regmap;
	struct iaxxx_i2c_priv *i2c_priv = to_i2c_priv(priv);

	if (!i2c_priv || !priv->regmap_config) {
		pr_err("%s: NULL input pointer(s)\n", __func__);
		return -EINVAL;
	}

	regmap = regmap_init_i2c(i2c_priv->i2c, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev = &i2c_priv->i2c->dev;
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	priv->regmap = regmap;
	return 0;
}

#if defined(CONFIG_ARCH_MSM)
static int reg_set_optimum_mode_check(struct regulator *reg, int load_ua)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_ua) : 0;
}

static int iaxxx_i2c_power_on(struct iaxxx_i2c_priv *i2c_priv)
{
	int rc;
	struct device *dev = &i2c_priv->i2c->dev;

	if (i2c_priv->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(i2c_priv->vcc_i2c,
				IAXXX_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(i2c_priv->vcc_i2c);
		if (rc) {
			dev_err(dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}
	msleep(130);
	return 0;

error_reg_en_vcc_i2c:
	if (i2c_priv->i2c_pull_up)
		reg_set_optimum_mode_check(i2c_priv->vcc_i2c, 0);
error_reg_opt_i2c:
	return rc;

}

static int iaxxx_i2c_regulator_configure(struct iaxxx_i2c_priv *i2c_priv)
{
	int rc;
	struct device *dev = &i2c_priv->i2c->dev;

	pr_debug("Configuring i2c_pull_up");

	if (i2c_priv->i2c_pull_up) {
		i2c_priv->vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(i2c_priv->vcc_i2c)) {
			rc = PTR_ERR(i2c_priv->vcc_i2c);
			dev_err(dev, "Regulator get failed rc=%d\n", rc);
			goto error_get_vtg_i2c;
		}
		pr_debug("regulator_count_voltages %d\n",
				regulator_count_voltages(i2c_priv->vcc_i2c));

		if (regulator_count_voltages(i2c_priv->vcc_i2c) > 0) {
			rc = regulator_set_voltage(i2c_priv->vcc_i2c,
				IAXXX_I2C_VTG_MIN_UV, IAXXX_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}
	return 0;

error_set_vtg_i2c:
	regulator_put(i2c_priv->vcc_i2c);
error_get_vtg_i2c:
	return rc;
}

static void iaxxx_i2c_regulator_disable(struct iaxxx_i2c_priv *i2c_priv)
{
	if (i2c_priv->i2c_pull_up) {
		reg_set_optimum_mode_check(i2c_priv->vcc_i2c, 0);
		regulator_disable(i2c_priv->vcc_i2c);
		msleep(50);
	}
}

static int iaxxx_i2c_power_off(struct iaxxx_i2c_priv *i2c_priv)
{
	if (i2c_priv->i2c_pull_up) {
		if (regulator_count_voltages(i2c_priv->vcc_i2c) > 0)
			regulator_set_voltage(i2c_priv->vcc_i2c, 0,
						IAXXX_I2C_VTG_MAX_UV);
		regulator_put(i2c_priv->vcc_i2c);
	}
	return 0;
}
#endif

static int iaxxx_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	int rc = 0;
	struct iaxxx_i2c_priv *i2c_priv;
	struct device *dev = &i2c->dev;

	uint32_t sync_response;
	const uint32_t SBL_SYNC_CMD = 0x80000000;
	const uint32_t CMD_REGMAP_MODE = 0x80040000;
#if defined(CONFIG_ARCH_MSM)
	struct device_node *np = NULL;
#endif

	dev_dbg(dev, "%s:%d\n", __func__, __LINE__);

	/* Create driver private-data struct */
	i2c_priv = devm_kzalloc(dev, sizeof(*i2c_priv), GFP_KERNEL);
	if (!i2c_priv) {
		dev_err(dev, "%s() private data allocation failed\n", __func__);
		return -ENOMEM;
	}

	i2c_priv->i2c = i2c;
	i2c_priv->priv.dev = dev;
	i2c_priv->priv.regmap_init_bus = iaxxx_i2c_regmap_init;
	i2c_priv->priv.bus = IAXXX_I2C;

	i2c_set_clientdata(i2c, i2c_priv);

#if defined(CONFIG_ARCH_MSM)
	np = dev->of_node;

	if (np) {
		/* regulator info */
		i2c_priv->i2c_pull_up =
			of_property_read_bool(np, "adnc,i2c-pull-up");
		dev_dbg(dev, "i2c_pull_up %d\n", i2c_priv->i2c_pull_up);

		rc = iaxxx_i2c_regulator_configure(i2c_priv);
		if (rc) {
			dev_err(dev, "%s: config hw fail %d\n", __func__, rc);
			goto continue_init;
		}
		rc = iaxxx_i2c_power_on(i2c_priv);
		if (rc)
			dev_err(dev, "%s: Power On hardware Fail %d\n",
				__func__, rc);
	}

continue_init:
#endif

	/* Populate device tree data and reset to SBL */
	rc = iaxxx_device_reset(&i2c_priv->priv);
	if (rc) {
		dev_err(dev, "%s device reset failed, err:%d\n", __func__, rc);
		goto probe_failed;
	}

	/* Send a SYNC command */
	rc = iaxxx_i2c_cmd(i2c_priv->i2c, SBL_SYNC_CMD, &sync_response);
	if (rc) {
		dev_err(dev, "%s Failed to send SYNC, err:%d\n", __func__, rc);
		goto probe_failed;
	}
	dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);

	dev_dbg(dev, "Putting device in regmap mode\n");

	/* Switch the device into regmap mode */
	rc = iaxxx_i2c_cmd(i2c_priv->i2c, CMD_REGMAP_MODE, NULL);
	if (rc) {
		dev_err(dev, "%s REGMAP mode failed, err:%d\n", __func__, rc);
		goto probe_failed;
	}
	usleep_range(1000, 2500);

	rc = iaxxx_device_init(&i2c_priv->priv);
	if (rc) {
		dev_err(dev, "%s device init failed, err:%d\n", __func__, rc);
		goto probe_failed;
	}

	/* Raw read write callbacks */
	i2c_priv->priv.raw_ops->read = iaxxx_i2c_bus_raw_read;
	i2c_priv->priv.raw_ops->write = iaxxx_i2c_bus_raw_write;

	return rc;

probe_failed:
	devm_kfree(dev, i2c_priv);
	i2c_set_clientdata(i2c, NULL);
	return rc;
}

static int iaxxx_i2c_remove(struct i2c_client *i2c)
{
	struct iaxxx_i2c_priv *i2c_priv = i2c_get_clientdata(i2c);

	if (i2c_priv) {
		iaxxx_device_exit(&i2c_priv->priv);
#if defined(CONFIG_ARCH_MSM)
		iaxxx_i2c_power_off(i2c_priv);
		iaxxx_i2c_regulator_disable(i2c_priv);
#endif
		devm_kfree(&i2c->dev, i2c_priv);
	}
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id iaxxx_i2c_id[] = {
	{ "iaxxx-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iaxxx_i2c_id);

static const struct of_device_id iaxxx_i2c_dt_match[] = {
	{.compatible = "adnc,iaxxx-i2c"},
	{}
};

static struct i2c_driver iaxxx_i2c_driver = {
	.driver = {
		.name = "iaxxx-i2c",
		/* .pm = &iaxxx_pm_ops, */
		.of_match_table = iaxxx_i2c_dt_match,
	},
	.probe = iaxxx_i2c_probe,
	.remove = iaxxx_i2c_remove,
	.id_table = iaxxx_i2c_id,
};

module_i2c_driver(iaxxx_i2c_driver);

MODULE_DESCRIPTION("I2C support for Knowles IAxxx");
MODULE_LICENSE("GPL");
