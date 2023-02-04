/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * PMIC rails controller for airbrush state manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#include <linux/airbrush-sm-ctrl.h>

int ab_mark_pmic_rail(struct ab_state_context *sc,
			   enum block_name blk_name,
			   bool enable,
			   enum block_state to_block_substate_id)
{
	dev_dbg(sc->dev,
			"marking rails %d for block %u substate id %u\n",
			enable, blk_name, to_block_substate_id);

	switch (blk_name) {
	case BLK_IPU:
	case BLK_TPU:
		sc->smps1_state = enable;
		sc->ldo3_state = enable;
		break;
	case BLK_AON:
		sc->smps2_state = enable;
		sc->ldo4_state = enable;
		sc->ldo5_state = enable;
		break;
	case DRAM:
		sc->ldo2_state = enable;
		if (enable) {
			if (to_block_substate_id != BLOCK_STATE_0) {
				sc->ldo1_state = true;
				sc->smps3_state = true;
			}
		} else {
			if (to_block_substate_id == BLOCK_STATE_0) {
				sc->ldo1_state = false;
				sc->smps3_state = false;
			}
		}

		break;
	case BLK_MIF:
	case BLK_FSYS:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int ab_pmic_off(struct ab_state_context *sc)
{
	int ret1, ret2 = 0;

	dev_dbg(sc->dev, "Turning OFF PMIC rails\n");

	if (!sc->ldo2_state && regulator_is_enabled(sc->ldo2)) {
		dev_dbg(sc->dev, "Disable ldo2\n");
		ret1 = regulator_disable(sc->ldo2);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable LDO2, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;
	}

	if (!sc->ldo3_state && regulator_is_enabled(sc->ldo3)) {
		dev_dbg(sc->dev, "Disable ldo3\n");
		ret1 = regulator_disable(sc->ldo3);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable LDO3, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;
		usleep_range(2000, 3000);
	}

	if (!sc->smps1_state && regulator_is_enabled(sc->smps1)) {
		dev_dbg(sc->dev, "Disable smps1\n");
		ret1 = regulator_disable(sc->smps1);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable SMPS1, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;
	}

	if (!sc->ldo5_state && regulator_is_enabled(sc->ldo5)) {
		dev_dbg(sc->dev, "Disable ldo5\n");
		ret1 = regulator_disable(sc->ldo5);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable LDO5, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;

		/* NOTE: delay required by b/120785608 */
		if (!sc->ldo4_state || !sc->smps2_state)
			usleep_range(sc->ldo5_delay, sc->ldo5_delay + 1);
	}

	if (!sc->ldo4_state && regulator_is_enabled(sc->ldo4)) {
		dev_dbg(sc->dev, "Disable ldo4\n");
		ret1 = regulator_disable(sc->ldo4);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable LDO4, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;

		if (sc->ldo4_delay)
			usleep_range(sc->ldo4_delay, sc->ldo4_delay + 1);
	}

	if (!sc->smps3_state && regulator_is_enabled(sc->smps3)) {
		dev_dbg(sc->dev, "Disable smps3\n");
		ret1 = regulator_disable(sc->smps3);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable SMPS3, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;

		/*
		 * Delay added between SMPS3 and LDO1 for correctness per
		 * characerization results and sequecing requirements.
		 * See b/131924947.
		 */
		usleep_range(4000, 4100);
	}

	if (!sc->ldo1_state && regulator_is_enabled(sc->ldo1)) {
		dev_dbg(sc->dev, "Disable ldo1\n");
		ret1 = regulator_disable(sc->ldo1);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable LDO1, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;
	}

	if (!sc->smps2_state && regulator_is_enabled(sc->smps2)) {
		dev_dbg(sc->dev, "Disable smps2\n");
		ret1 = regulator_disable(sc->smps2);
		if (ret1 < 0)
			dev_err(sc->dev,
				"failed to disable SMPS2, ret %d\n", ret1);
		ret2 = ret2 ? ret2 : ret1;

		if (sc->smps2_delay)
			usleep_range(sc->smps2_delay, sc->smps2_delay + 1);
	}

	/* NOTE: delay required by b/120785608 */
	if (!sc->smps2_state &&
			!sc->ldo1_state &&
			!sc->smps3_state &&
			!sc->ldo4_state &&
			!sc->ldo5_state &&
			!sc->smps1_state &&
			!sc->ldo3_state &&
			!sc->ldo2_state)
		usleep_range(sc->s60_delay, sc->s60_delay + 1);

	return ret2;
}

int ab_pmic_on(struct ab_state_context *sc)
{
	dev_dbg(sc->dev, "Turning ON PMIC rails\n");

	if (sc->smps2_state && !regulator_is_enabled(sc->smps2)) {
		dev_dbg(sc->dev, "Enable smps2\n");
		if (regulator_enable(sc->smps2))
			goto fail_regulator_enable;
	}

	if (sc->ldo1_state && !regulator_is_enabled(sc->ldo1)) {
		dev_dbg(sc->dev, "Enable ldo1\n");
		if (regulator_enable(sc->ldo1))
			goto fail_regulator_enable;
	}

	if (sc->smps3 && !regulator_is_enabled(sc->smps3)) {
		dev_dbg(sc->dev, "Enable smps3\n");
		if (regulator_enable(sc->smps3))
			goto fail_regulator_enable;
	}

	if (sc->ldo4_state && !regulator_is_enabled(sc->ldo4)) {
		dev_dbg(sc->dev, "Enable ldo4\n");
		if (regulator_enable(sc->ldo4))
			goto fail_regulator_enable;
	}

	if (sc->ldo5_state && !regulator_is_enabled(sc->ldo5)) {
		dev_dbg(sc->dev, "Enable ldo5\n");
		if (regulator_enable(sc->ldo5))
			goto fail_regulator_enable;
	}

	if (sc->smps1_state && !regulator_is_enabled(sc->smps1)) {
		dev_dbg(sc->dev, "Enable smps1\n");
		if (regulator_enable(sc->smps1))
			goto fail_regulator_enable;
	}

	if (sc->ldo3_state && !regulator_is_enabled(sc->ldo3)) {
		dev_dbg(sc->dev, "Enable ldo3\n");
		if (regulator_enable(sc->ldo3))
			goto fail_regulator_enable;
	}

	if (sc->ldo2_state && !regulator_is_enabled(sc->ldo2)) {
		dev_dbg(sc->dev, "Enable ldo2\n");
		if (regulator_enable(sc->ldo2))
			goto fail_regulator_enable;
	}

	return 0;

fail_regulator_enable:
	if (regulator_is_enabled(sc->ldo3))
		regulator_disable(sc->ldo3);
	sc->ldo3_state = false;
	if (regulator_is_enabled(sc->smps1))
		regulator_disable(sc->smps1);
	sc->smps1_state = false;
	if (regulator_is_enabled(sc->ldo5))
		regulator_disable(sc->ldo5);
	sc->ldo5_state = false;
	if (regulator_is_enabled(sc->ldo4))
		regulator_disable(sc->ldo4);
	sc->ldo4_state = false;
	if (regulator_is_enabled(sc->smps3))
		regulator_disable(sc->smps3);
	sc->smps3_state = false;
	if (regulator_is_enabled(sc->ldo1))
		regulator_disable(sc->ldo1);
	sc->ldo1_state = false;
	if (regulator_is_enabled(sc->smps2))
		regulator_disable(sc->smps2);
	sc->smps2_state = false;
	dev_err(sc->dev, "PMIC power up failure\n");
	return -ENODEV;
}

static int ab_register_notifier(struct ab_state_context *sc)
{
	struct device *dev = sc->dev;
	int ret = 0;

	ret |= devm_regulator_register_notifier(sc->smps1, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->smps2, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->smps3, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->ldo1, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->ldo2, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->ldo3, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->ldo4, &sc->regulator_nb);
	ret |= devm_regulator_register_notifier(sc->ldo5, &sc->regulator_nb);
	if (ret) {
		dev_err(dev, "failed to register notifier block\n");
		return ret;
	}

	return 0;
}

/* Called during sm driver probe. */
int ab_get_pmic_resources(struct ab_state_context *sc)
{
	struct device *dev = sc->dev;

	if (!sc->soc_pwrgood) {
		sc->soc_pwrgood =
			devm_gpiod_get(dev, "soc-pwrgood", GPIOD_OUT_LOW);
		if (IS_ERR(sc->soc_pwrgood)) {
			dev_warn(dev,
				 "Could not get pmic_soc_pwrgood gpio (%ld), PMIC driver may not be probed yet\n",
				 PTR_ERR(sc->soc_pwrgood));
			return PTR_ERR(sc->soc_pwrgood);
		}
	}

	if (!sc->ddr_sr) {
		sc->ddr_sr =
			devm_gpiod_get(dev, "ddr-sr", GPIOD_OUT_LOW);
		if (IS_ERR(sc->ddr_sr)) {
			dev_err(dev, "Could not get pmic_ddr_sr gpio (%ld)\n",
					PTR_ERR(sc->ddr_sr));
			return PTR_ERR(sc->ddr_sr);
		}
	}

	if (!sc->ddr_iso) {
		sc->ddr_iso =
			devm_gpiod_get(dev, "ddr-iso", GPIOD_OUT_LOW);
		if (IS_ERR(sc->ddr_iso)) {
			dev_err(dev, "Could not get pmic_ddr_iso gpio (%ld)\n",
					PTR_ERR(sc->ddr_iso));
			return PTR_ERR(sc->ddr_iso);
		}
	}

	if (!sc->smps1) {
		sc->smps1 = devm_regulator_get(dev, "s2mpg01_smps1");
		if (IS_ERR(sc->smps1)) {
			dev_err(dev, "failed to get s2mpg01_smps1 supply (%ld)\n",
					PTR_ERR(sc->smps1));
			return PTR_ERR(sc->smps1);
		}
	}

	if (!sc->smps2) {
		sc->smps2 = devm_regulator_get(dev, "s2mpg01_smps2");
		if (IS_ERR(sc->smps2)) {
			dev_err(dev, "failed to get s2mpg01_smps2 supply (%ld)\n",
					PTR_ERR(sc->smps2));
			return PTR_ERR(sc->smps2);
		}
	}

	if (!sc->smps3) {
		sc->smps3 = devm_regulator_get(dev, "s2mpg01_smps3");
		if (IS_ERR(sc->smps3)) {
			dev_err(dev, "failed to get s2mpg01_smps3 supply (%ld)\n",
					PTR_ERR(sc->smps3));
			return PTR_ERR(sc->smps3);
		}
	}

	if (!sc->ldo1) {
		sc->ldo1 = devm_regulator_get(dev, "s2mpg01_ldo1");
		if (IS_ERR(sc->ldo1)) {
			dev_err(dev, "failed to get s2mpg01_ldo1 supply (%ld)\n",
					PTR_ERR(sc->ldo1));
			return PTR_ERR(sc->ldo1);
		}
	}

	if (!sc->ldo2) {
		sc->ldo2 = devm_regulator_get(dev, "s2mpg01_ldo2");
		if (IS_ERR(sc->ldo2)) {
			dev_err(dev, "failed to get s2mpg01_ldo2 supply (%ld)\n",
					PTR_ERR(sc->ldo2));
			return PTR_ERR(sc->ldo2);
		}
	}

	if (!sc->ldo3) {
		sc->ldo3 = devm_regulator_get(dev, "s2mpg01_ldo3");
		if (IS_ERR(sc->ldo3)) {
			dev_err(dev, "failed to get s2mpg01_ldo3 supply (%ld)\n",
					PTR_ERR(sc->ldo3));
			return PTR_ERR(sc->ldo3);
		}
	}

	if (!sc->ldo4) {
		sc->ldo4 = devm_regulator_get(dev, "s2mpg01_ldo4");
		if (IS_ERR(sc->ldo4)) {
			dev_err(dev, "failed to get s2mpg01_ldo4 supply (%ld)\n",
					PTR_ERR(sc->ldo4));
			return PTR_ERR(sc->ldo4);
		}
	}

	if (!sc->ldo5) {
		sc->ldo5 = devm_regulator_get(dev, "s2mpg01_ldo5");
		if (IS_ERR(sc->ldo5)) {
			dev_err(dev, "failed to get s2mpg01_ldo5 supply (%ld)\n",
					PTR_ERR(sc->ldo5));
			return PTR_ERR(sc->ldo5);
		}
	}

	return ab_register_notifier(sc);
}
