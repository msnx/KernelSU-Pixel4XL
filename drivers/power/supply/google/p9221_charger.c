/*
 * P9221 Wireless Charger Driver
 *
 * Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <dt-bindings/wc/p9221-wc.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include <linux/alarmtimer.h>
#include "p9221_charger.h"
#include "../google/logbuffer.h"

#define P9221_TX_TIMEOUT_MS		(20 * 1000)
#define P9221_DCIN_TIMEOUT_MS		(1 * 1000)
#define P9221_VRECT_TIMEOUT_MS		(2 * 1000)
#define P9221_ALIGN_TIMEOUT_MS		(2 * 1000)
#define P9221_ALIGN_DELAY_MS		100
#define P9221_NOTIFIER_DELAY_MS		100
#define P9221_DCIN_PON_DELAY_MS		250
#define P9221R5_ILIM_MAX_UA		(1600 * 1000)
#define P9221R5_OVER_CHECK_NUM		3
#define P9221_POWER_MITIGATE_DELAY_MS	(10 * 1000)
#define P9221_FOD_MAX_TIMES		3

#define OVC_LIMIT			1
#define OVC_THRESHOLD			1400000
#define OVC_BACKOFF_LIMIT		900000
#define OVC_BACKOFF_AMOUNT		100000

#define P9221_CHECK_NP_DELAY_MS		50
#define P9221_NEG_POWER_5W		(5 / 0.5)
#define P9221_NEG_POWER_10W		(10 / 0.5)
#define P9221_PTMC_EPP_TX_1912		0x32

#define WLC_ALIGNMENT_MAX		100
#define WLC_MFG_GOOGLE			0x72
#define WLC_CURRENT_FILTER_LENGTH	10
#define WLC_ALIGN_DEFAULT_SCALAR	4
#define WLC_ALIGN_IRQ_THRESHOLD		10
#define WLC_ALIGN_DEFAULT_HYSTERESIS	5000

#define RTX_BEN_DISABLED	0
#define RTX_BEN_ON		1
#define RTX_BEN_ENABLED		2

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger);
static void p9221_icl_ramp_start(struct p9221_charger_data *charger);
static bool p9221_has_dd(struct p9221_charger_data *charger);
static const char *p9221_get_tx_id_str(struct p9221_charger_data *charger);

static const u32 p9221_ov_set_lut[] = {
	17000000, 20000000, 15000000, 13000000,
	11000000, 11000000, 11000000, 11000000};

static char *align_status_str[] = {
	"...", "M2C", "OK", "-1"
};

static size_t p9221_hex_str(u8 *data, size_t len, char *buf, size_t max_buf,
			    bool msbfirst)
{
	int i;
	int blen = 0;
	u8 val;

	for (i = 0; i < len; i++) {
		if (msbfirst)
			val = data[len - 1 - i];
		else
			val = data[i];
		blen += scnprintf(buf + (i * 3), max_buf - (i * 3),
				  "%02x ", val);
	}
	return blen;
}

static int p9221_reg_read_n(struct p9221_charger_data *charger, u16 reg,
			    void *buf, size_t n)
{
	int ret;
	struct i2c_msg msg[2];
	u8 wbuf[2];

	msg[0].addr = charger->client->addr;
	msg[0].flags = charger->client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = charger->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = n;
	msg[1].buf = buf;

	mutex_lock(&charger->io_lock);
	ret = i2c_transfer(charger->client->adapter, msg, 2);
	mutex_unlock(&charger->io_lock);

	if (ret < 0) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : ret;

		dev_err(&charger->client->dev,
			"i2c read error, reg:%x, ret:%d (%d)\n",
			reg, ret, nret);
		return nret;
	}

	return (ret == 2) ? 0 : -EIO;
}

static int p9221_reg_read_16(struct p9221_charger_data *charger, u16 reg,
			     u16 *val)
{
	u8 buf[2];
	int ret;

	ret = p9221_reg_read_n(charger, reg, buf, 2);
	if (ret == 0)
		*val = (buf[1] << 8) | buf[0];
	return ret;
}

static int p9221_reg_read_8(struct p9221_charger_data *charger,
			    u16 reg, u8 *val)
{
	return p9221_reg_read_n(charger, reg, val, 1);
}

static bool p9221_reg_is_8_bit(struct p9221_charger_data *charger, u16 reg)
{
	switch (reg) {
	case P9221_CHIP_REVISION_REG:
	case P9221_CUSTOMER_ID_REG:
	case P9221_COM_REG:
	case P9221R5_VOUT_SET_REG:
	case P9221R5_ILIM_SET_REG:
	case P9221R5_CHARGE_STAT_REG:
	case P9221R5_EPT_REG:
	case P9221R5_SYSTEM_MODE_REG:
	case P9221R5_COM_CHAN_RESET_REG:
	case P9221R5_COM_CHAN_SEND_SIZE_REG:
	case P9221R5_COM_CHAN_SEND_IDX_REG:
	case P9221R5_COM_CHAN_RECV_SIZE_REG:
	case P9221R5_COM_CHAN_RECV_IDX_REG:
	case P9221R5_DEBUG_REG:
	case P9221R5_EPP_Q_FACTOR_REG:
	case P9221R5_EPP_TX_GUARANTEED_POWER_REG:
	case P9221R5_EPP_TX_POTENTIAL_POWER_REG:
	case P9221R5_EPP_TX_CAPABILITY_FLAGS_REG:
	case P9221R5_EPP_RENEGOTIATION_REG:
	case P9221R5_EPP_CUR_RPP_HEADER_REG:
	case P9221R5_EPP_CUR_NEGOTIATED_POWER_REG:
	case P9221R5_EPP_CUR_MAXIMUM_POWER_REG:
	case P9221R5_EPP_CUR_FSK_MODULATION_REG:
	case P9221R5_EPP_REQ_RPP_HEADER_REG:
	case P9221R5_EPP_REQ_NEGOTIATED_POWER_REG:
	case P9221R5_EPP_REQ_MAXIMUM_POWER_REG:
	case P9221R5_EPP_REQ_FSK_MODULATION_REG:
	case P9221R5_VRECT_TARGET_REG:
	case P9221R5_VRECT_KNEE_REG:
	case P9221R5_FOD_SECTION_REG:
	case P9221R5_VRECT_ADJ_REG:
	case P9221R5_ALIGN_X_ADC_REG:
	case P9221R5_ALIGN_Y_ADC_REG:
	case P9221R5_ASK_MODULATION_DEPTH_REG:
	case P9221R5_OVSET_REG:
	case P9221R5_EPP_TX_SPEC_REV_REG:
		return true;
	default:
		return false;
	}
}

/*
 * Cook the value according to the register (R5+)
 */
static int p9221_cook_reg(struct p9221_charger_data *charger, u16 reg,
			  u16 raw_data, u32 *val)
{
	/* Do the appropriate conversion */
	switch (reg) {
	/* The following raw values */
	case P9221R5_ALIGN_X_ADC_REG:
	case P9221R5_ALIGN_Y_ADC_REG:
		*val = raw_data;
		break;

	/* The following are 12-bit ADC raw values */
	case P9221R5_VOUT_ADC_REG:
	case P9221R5_IOUT_ADC_REG:
	case P9221R5_EXT_TEMP_REG:
		*val = raw_data & 0xFFF;
		break;

	/* the formula to translate it into milli-degreeC  */
	case P9221R5_DIE_TEMP_ADC_REG:
		*val = (raw_data * 10 / 107 - 247) * 10;
		break;

	/* The following are in 0.1 mill- and need to go to micro- */
	case P9221R5_VOUT_SET_REG:	/* 100mV -> uV */
		raw_data *= 100;
		/* Fall through */

	/* The following are in milli- and need to go to micro- */
	case P9221R5_IOUT_REG:		/* mA -> uA */
	case P9221R5_VRECT_REG:		/* mV -> uV */
	case P9221R5_VOUT_REG:		/* mV -> uV */
		/* Fall through */

	/* The following are in kilo- and need to go to their base */
	case P9221R5_OP_FREQ_REG:	/* kHz -> Hz */
	case P9221R5_TX_PINGFREQ_REG:	/* kHz -> Hz */
		*val = raw_data * 1000;
		break;

	case P9221R5_ILIM_SET_REG:
		/* 100mA -> uA, 200mA offset */
		*val = ((raw_data * 100) + 200) * 1000;
		break;

	case P9221R5_OVSET_REG:
		/* uV */
		raw_data &= P9221R5_OVSET_MASK;
		*val = p9221_ov_set_lut[raw_data];
		break;

	default:
		return -ENOENT;
	}

	return 0;
}

/*
 * Read the reg and return the cooked value.
 */
static int p9221_reg_read_cooked(struct p9221_charger_data *charger,
				 u16 reg, u32 *val)
{
	int ret = 0;
	u16 data = 0;

	if (p9221_reg_is_8_bit(charger, reg)) {
		u8 data8 = 0;
		ret = p9221_reg_read_8(charger, reg, &data8);
		data = data8;
	} else {
		ret = p9221_reg_read_16(charger, reg, &data);
	}
	if (ret)
		return ret;

	return p9221_cook_reg(charger, reg, data, val);
}

static int p9221_reg_write_n(struct p9221_charger_data *charger, u16 reg,
			     void *buf, size_t n)
{
	int ret;
	u8 *data;
	int datalen = 2 + n;

	data = kmalloc(datalen, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data[0] = reg >> 8;
	data[1] = reg & 0xFF;
	memcpy(&data[2], buf, n);

	mutex_lock(&charger->io_lock);
	ret = i2c_master_send(charger->client, data, datalen);
	mutex_unlock(&charger->io_lock);
	kfree(data);

	if (ret < datalen) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : -EIO;

		dev_err(&charger->client->dev,
			"%s: i2c write error, reg: 0x%x, n: %zd ret: %d (%d)\n",
			__func__, reg, n, ret, nret);
		return nret;
	}

	return 0;
}

static int p9221_reg_write_16(struct p9221_charger_data *charger, u16 reg,
			      u16 val)
{
	return p9221_reg_write_n(charger, reg, &val, 2);
}

static int p9221_reg_write_8(struct p9221_charger_data *charger, u16 reg, u8 val)
{
	return p9221_reg_write_n(charger, reg, &val, 1);
}

/*
 * Uncook the values and write to register
 */
static int p9221_reg_write_cooked(struct p9221_charger_data *charger,
				  u16 reg, u32 val)
{
	int ret = 0;
	u16 data;
	int i;

	/* Do the appropriate conversion */
	switch (reg) {
	case P9221R5_ILIM_SET_REG:
		/* uA -> 0.1A, offset 0.2A */
		if ((val < 200000) || (val > 1600000))
			return -EINVAL;
		data = (val / (100 * 1000)) - 2;
		break;

	case P9221R5_VOUT_SET_REG:
		/* uV -> 0.1V */
		val /= 1000;
		if (val < 3500 || val > charger->pdata->max_vout_mv)
			return -EINVAL;
		data = val / 100;
		break;

	case P9221R5_OVSET_REG:
		/* uV */
		for (i = 0; i < ARRAY_SIZE(p9221_ov_set_lut); i++) {
			if (val == p9221_ov_set_lut[i])
				break;
		}
		if (i == ARRAY_SIZE(p9221_ov_set_lut))
			return -EINVAL;
		data = i;
		break;
	default:
		return -ENOENT;
	}

	if (p9221_reg_is_8_bit(charger, reg))
		ret = p9221_reg_write_8(charger, reg, data);
	else
		ret = p9221_reg_write_16(charger, reg, data);

	return ret;
}

static bool p9221_is_epp(struct p9221_charger_data *charger)
{
	int ret;
	u32 vout_uv;
	uint8_t reg;

	if (charger->fake_force_epp > 0)
		return true;
	if (charger->force_bpp)
		return false;

	/*
	*  NOTE: mfg may be zero due to race condition during bringup. will
	*  check once more if mfg == 0.
	*/
	if (charger->mfg == 0) {
		ret = p9221_reg_read_16(charger, P9221R5_EPP_TX_MFG_CODE_REG,
					&charger->mfg);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", ret);
	}

	charger->is_mfg_google = charger->mfg == WLC_MFG_GOOGLE;

	ret = p9221_reg_read_8(charger, P9221R5_SYSTEM_MODE_REG, &reg);
	if (ret == 0)
		return (reg & P9221R5_SYSTEM_MODE_EXTENDED_MASK) > 0;

	dev_err(&charger->client->dev, "Could not read mode: %d\n",
		ret);

	/* Check based on power supply voltage */
	ret = p9221_reg_read_cooked(charger, P9221R5_VOUT_ADC_REG, &vout_uv);
	if (ret) {
		dev_err(&charger->client->dev, "Could read VOUT_ADC, %d\n",
			ret);
		goto out;
	}

	dev_info(&charger->client->dev, "Voltage is %duV\n", vout_uv);
	if (vout_uv > P9221_EPP_THRESHOLD_UV)
		return true;

out:
	/* Default to BPP otherwise */
	return false;
}

static void p9221_write_fod(struct p9221_charger_data *charger)
{
	bool epp = false;
	u8 *fod = NULL;
	int fod_count = charger->pdata->fod_num;
	int ret;
	int retries = 3;

	if (!charger->pdata->fod_num && !charger->pdata->fod_epp_num)
		goto no_fod;

	/* Default to BPP FOD */
	if (charger->pdata->fod_num)
		fod = charger->pdata->fod;

	if (p9221_is_epp(charger) && charger->pdata->fod_epp_num) {
		fod = charger->pdata->fod_epp;
		fod_count = charger->pdata->fod_epp_num;
		epp = true;
	}

	if (!fod)
		goto no_fod;

	while (retries) {
		char s[fod_count * 3 + 1];
		u8 fod_read[fod_count];

		dev_info(&charger->client->dev, "Writing %s FOD (n=%d reg=%02x try=%d)\n",
			 epp ? "EPP" : "BPP", fod_count, P9221R5_FOD_REG,
			 retries);

		ret = p9221_reg_write_n(charger, P9221R5_FOD_REG, fod,
					fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not write FOD: %d\n", ret);
			return;
		}

		/* Verify the FOD has been written properly */
		ret = p9221_reg_read_n(charger, P9221R5_FOD_REG, fod_read,
				       fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not read back FOD: %d\n", ret);
			return;
		}

		if (memcmp(fod, fod_read, fod_count) == 0)
			return;

		p9221_hex_str(fod_read, fod_count, s, sizeof(s), 0);
		dev_err(&charger->client->dev,
			"FOD verify error, read: %s\n", s);

		retries--;
		msleep(100);
	}

no_fod:
	dev_warn(&charger->client->dev, "FOD not set! bpp:%d epp:%d r:%d\n",
		 charger->pdata->fod_num, charger->pdata->fod_epp_num, retries);
}

static int p9221_set_cmd_reg(struct p9221_charger_data *charger, u8 cmd)
{
	u8 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = p9221_reg_read_8(charger, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&charger->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = p9221_reg_write_8(charger, P9221_COM_REG, cmd);
	if (ret)
		dev_err(&charger->client->dev,
			"Failed to set cmd reg %02x: %d\n", cmd, ret);

	return ret;
}

static int p9221_send_data(struct p9221_charger_data *charger)
{
	int ret;

	if (charger->tx_busy)
		return -EBUSY;

	if (!charger->tx_len || charger->tx_len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	charger->tx_busy = true;

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_n(charger, charger->addr_data_send_buf_start,
				charger->tx_buf, charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load tx %d\n", ret);
		goto error;
	}

	ret = p9221_reg_write_8(charger, P9221R5_COM_CHAN_SEND_SIZE_REG,
				charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load txsz %d\n", ret);
		goto error;
	}

	ret = p9221_set_cmd_reg(charger, P9221R5_COM_CCACTIVATE);
	if (ret)
		goto error;

	mutex_unlock(&charger->cmd_lock);
	return ret;

error:
	mutex_unlock(&charger->cmd_lock);
	charger->tx_busy = false;
	return ret;
}

static int p9221_send_csp(struct p9221_charger_data *charger, u8 stat)
{
	int ret;

	dev_info(&charger->client->dev, "Send CSP status=%d\n", stat);

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_CHARGE_STAT_REG, stat);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_SENDCSP);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

static int p9221_send_eop(struct p9221_charger_data *charger, u8 reason)
{
	int ret;

	dev_info(&charger->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_EPT_REG, reason);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_SENDEPT);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

static int p9221_send_ccreset(struct p9221_charger_data *charger)
{
	int ret;

	dev_info(&charger->client->dev, "Send CC reset\n");

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_8(charger, P9221R5_COM_CHAN_RESET_REG,
				P9221R5_COM_CHAN_CCRESET);
	if (ret == 0)
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_CCACTIVATE);

	mutex_unlock(&charger->cmd_lock);
	return ret;
}

struct p9221_prop_reg_map_entry p9221_prop_reg_map[] = {
	/* property			register			g, s */
	{POWER_SUPPLY_PROP_CURRENT_NOW,	P9221R5_IOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_NOW,	P9221R5_VOUT_REG,		1, 0},
	{POWER_SUPPLY_PROP_VOLTAGE_MAX, P9221R5_VOUT_SET_REG,		1, 1},
	{POWER_SUPPLY_PROP_TEMP,	P9221R5_DIE_TEMP_ADC_REG,	1, 0},
	{POWER_SUPPLY_PROP_CAPACITY,	0,				1, 1},
	{POWER_SUPPLY_PROP_ONLINE,	0,				1, 1},
	{POWER_SUPPLY_PROP_OPERATING_FREQ,
					P9221R5_OP_FREQ_REG,		1, 0},
};

static struct p9221_prop_reg_map_entry *p9221_get_map_entry(
				struct p9221_charger_data *charger,
				enum power_supply_property prop, bool set)
{
	int i;
	struct p9221_prop_reg_map_entry *p;
	int map_size;

	p = p9221_prop_reg_map;
	map_size = ARRAY_SIZE(p9221_prop_reg_map);

	for (i = 0; i < map_size; i++) {
		if (p->prop == prop) {
			if ((set && p->set) || (!set && p->get))
				return p;
		}
		p++;
	}
	return NULL;
}

static bool p9221_is_online(const struct p9221_charger_data *charger)
{
	return charger->online || charger->ben_state;
}

static int p9221_get_property_reg(struct p9221_charger_data *charger,
				  enum power_supply_property prop,
				  union power_supply_propval *val)
{
	int ret;
	struct p9221_prop_reg_map_entry *p;
	u32 data;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

	p = p9221_get_map_entry(charger, prop, false);
	if (p == NULL)
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	ret = p9221_reg_read_cooked(charger, p->reg, &data);
	if (ret)
		return ret;

	val->intval = data;
	return 0;
}

static int p9221_set_property_reg(struct p9221_charger_data *charger,
				  enum power_supply_property prop,
				  const union power_supply_propval *val)
{
	struct p9221_prop_reg_map_entry *p;

	p = p9221_get_map_entry(charger, prop, true);
	if (p == NULL)
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	return p9221_reg_write_cooked(charger, p->reg, val->intval);
}

static void p9221_abort_transfers(struct p9221_charger_data *charger)
{
	/* Abort all transfers */
	cancel_delayed_work(&charger->tx_work);
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = true;
	charger->rx_len = 0;
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
}

/*
 * Put the default ICL back to BPP, reset OCP voter
 * @pre charger && charger->dc_icl_votable && charger->client->dev
 */
static void p9221_vote_defaults(struct p9221_charger_data *charger)
{
	int ret;

	if (!charger->dc_icl_votable) {
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL - no votable\n");
		return;
	}

	ret = vote(charger->dc_icl_votable, P9221_WLC_VOTER, true,
			P9221_DC_ICL_BPP_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	ret = vote(charger->dc_icl_votable, P9221_OCP_VOTER, true,
			P9221_DC_ICL_EPP_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not reset OCP DC_ICL voter %d\n", ret);

	vote(charger->dc_icl_votable, P9382A_LOW_POWER_TX_VOTER, false, 0);
}

static void p9221_set_offline(struct p9221_charger_data *charger)
{
	if (charger->fod_cnt == 0) {
		/* already change to BPP or not doing power_mitigation */
		cancel_delayed_work(&charger->power_mitigation_work);
		charger->trigger_power_mitigation = false;
		charger->wait_for_online = false;
	} else {
		charger->wait_for_online = true;
		schedule_delayed_work(&charger->power_mitigation_work,
				      msecs_to_jiffies(
					  P9221_POWER_MITIGATE_DELAY_MS));
	}

	dev_info(&charger->client->dev, "Set offline\n");
	logbuffer_log(charger->log, "offline\n");

	charger->online = false;
	charger->force_bpp = false;
	charger->re_nego = false;
	charger->is_low_power_tx = false;

	/* Reset PP buf so we can get a new serial number next time around */
	charger->pp_buf_valid = false;
	memset(charger->pp_buf, 0, sizeof(charger->pp_buf));

	p9221_abort_transfers(charger);
	cancel_delayed_work(&charger->dcin_work);

	/* Reset alignment value when charger goes offline */
	cancel_delayed_work(&charger->align_work);
	charger->align = POWER_SUPPLY_ALIGN_ERROR;
	charger->align_count = 0;
	charger->alignment = -1;
	charger->alignment_capable = ALIGN_MFG_FAILED;
	charger->mfg = 0;
	charger->tx_id = 0;
	schedule_work(&charger->uevent_work);

	p9221_icl_ramp_reset(charger);
	del_timer(&charger->vrect_timer);

	p9221_vote_defaults(charger);
	if (charger->enabled &&
	    charger->pdata->qien_gpio >= 0) {
		if (charger->is_mfg_google == false)
			gpio_set_value(charger->pdata->qien_gpio, 1);

		mod_delayed_work(system_wq, &charger->dcin_pon_work,
				 msecs_to_jiffies(P9221_DCIN_PON_DELAY_MS));
	}
	charger->is_mfg_google = false;
}

static void p9221_tx_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, tx_work.work);

	dev_info(&charger->client->dev, "timeout waiting for tx complete\n");

	charger->tx_busy = false;
	charger->tx_done = true;
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
}

static void p9221_vrect_timer_handler(unsigned long data)
{
	struct p9221_charger_data *charger = (struct p9221_charger_data *)data;

	if (charger->align == POWER_SUPPLY_ALIGN_CHECKING) {
		schedule_work(&charger->uevent_work);
		charger->align = POWER_SUPPLY_ALIGN_MOVE;
		logbuffer_log(charger->log, "align: state: %s",
			      align_status_str[charger->align]);
	}
	dev_info(&charger->client->dev,
		 "timeout waiting for VRECT, online=%d\n", charger->online);
	logbuffer_log(charger->log,
		"vrect: timeout online=%d", charger->online);

	mod_timer(&charger->align_timer,
		  jiffies + msecs_to_jiffies(P9221_ALIGN_TIMEOUT_MS));

	pm_relax(charger->dev);
}

static void p9221_align_timer_handler(unsigned long data)
{
	struct p9221_charger_data *charger = (struct p9221_charger_data *)data;

	schedule_work(&charger->uevent_work);
	charger->align = POWER_SUPPLY_ALIGN_ERROR;
	logbuffer_log(charger->log, "align: timeout no IRQ");
}

static void p9221_dcin_pon_work(struct work_struct *work)
{
	int ret;
	union power_supply_propval prop;
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, dcin_pon_work.work);

	ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_DC_RESET, &prop);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Error getting charging status: %d\n", ret);
		return;
	}

	if (prop.intval == 0) {
		gpio_set_value(charger->pdata->qien_gpio, 0);
	} else {
		/* Signal DC_RESET when vout keeps on 1. */
		ret = power_supply_set_property(charger->dc_psy,
						POWER_SUPPLY_PROP_DC_RESET,
						&prop);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"unable to set DC_RESET, ret=%d", ret);

		schedule_delayed_work(&charger->dcin_pon_work,
			msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	}
}

static void p9221_dcin_work(struct work_struct *work)
{
	int res;
	u16 status_reg = 0;
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, dcin_work.work);

	res = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
	if (res != 0) {
		dev_info(&charger->client->dev,
			"timeout waiting for dc-in, online=%d\n",
			charger->online);
		logbuffer_log(charger->log,
			"dc_in: timeout online=%d", charger->online);

		if (charger->online)
			p9221_set_offline(charger);

		power_supply_changed(charger->wc_psy);
		pm_relax(charger->dev);

		return;
	}

	schedule_delayed_work(&charger->dcin_work,
			msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	logbuffer_log(charger->log, "dc_in: check online=%d status=%x",
			charger->online, status_reg);
}

static void force_set_fod(struct p9221_charger_data *charger)
{
	u16 i = 0;
	int ret = 0;

	dev_info(&charger->client->dev, "power_mitigate: write 0 to fod\n");

	for (i = 0; i < 0x10; i++)
		ret |= p9221_reg_write_8(charger, P9221R5_FOD_REG + i, 0);

	if (ret)
		dev_err(&charger->client->dev,
			"power_mitigate: fail to write register 0x%02x\n",
			P9221R5_FOD_REG + i);
}

static void p9221_power_mitigation_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, power_mitigation_work.work);

	charger->wait_for_online = false;

	if (!p9221_is_online(charger)) {
		dev_info(&charger->client->dev, "power_mitigate: offline\n");
		charger->fod_cnt = 0;
		charger->trigger_power_mitigation = false;
		power_supply_changed(charger->wc_psy);
		return;
	}

	if (!p9221_is_epp(charger)) {
		charger->fod_cnt = 0;
		dev_info(&charger->client->dev,
			 "power_mitigate: already BPP\n");
		return;
	}

	/* right now p9221_has_dd() implies charger->mfg==WLC_MFG_GOOGLE */
	if (charger->mfg != WLC_MFG_GOOGLE || !p9221_has_dd(charger)) {
		const char *txid = p9221_get_tx_id_str(charger);

		dev_info(&charger->client->dev,
			 "power_mitigate: not DD mfg=%x, id=%s\n",
			 charger->mfg, txid ? txid : "<>");
		return;
	}

	if (charger->fod_cnt < P9221_FOD_MAX_TIMES) {
		force_set_fod(charger);
		charger->fod_cnt++;
		dev_info(&charger->client->dev,
			 "power_mitigate: send FOD, cnt=%d\n",
			 charger->fod_cnt);
	} else {
		dev_info(&charger->client->dev,
			 "power_mitigate: power mitigation fail!\n");
		charger->fod_cnt = 0;
	}
}

static void p9221_init_align(struct p9221_charger_data *charger)
{
	/* Reset values used for alignment */
	charger->alignment_last = -1;
	charger->current_filtered = 0;
	charger->current_sample_cnt = 0;
	charger->mfg_check_count = 0;
	schedule_delayed_work(&charger->align_work,
			      msecs_to_jiffies(P9221_ALIGN_DELAY_MS));
}

static void p9221_align_work(struct work_struct *work)
{
	int res, align_buckets, i, wlc_freq_threshold, wlc_adj_freq;
	u16 current_now, current_filter_sample;
	u32 wlc_freq, current_scaling = 0;
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, align_work.work);

	if (charger->pdata->alignment_freq == NULL)
		return;

	charger->alignment = -1;

	if (!charger->online)
		return;

	/*
	 *  NOTE: mfg may be zero due to race condition during bringup. If the
	 *  mfg check continues to fail then mfg is not correct and we do not
	 *  reschedule align_work. Always reschedule if alignment_capable is 1.
	 *  Check 10 times if alignment_capble is still 0.
	 */
	if ((charger->mfg_check_count < 10) ||
	    (charger->alignment_capable == ALIGN_MFG_PASSED))
		schedule_delayed_work(&charger->align_work,
				      msecs_to_jiffies(P9221_ALIGN_DELAY_MS));

	if (charger->alignment_capable == ALIGN_MFG_CHECKING) {
		charger->mfg_check_count += 1;

		res = p9221_reg_read_16(charger,
					P9221R5_EPP_TX_MFG_CODE_REG,
					&charger->mfg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", res);
			return;
		}

		/* No mfg update. Will check again on next schedule */
		if (charger->mfg == 0)
			return;

		if ((charger->mfg != WLC_MFG_GOOGLE) ||
		    !p9221_is_epp(charger)) {
			logbuffer_log(charger->log,
				      "align: not align capable mfg: 0x%x",
				      charger->mfg);
			cancel_delayed_work(&charger->align_work);
			charger->alignment_capable = ALIGN_MFG_FAILED;
			return;
		}
		charger->alignment_capable = ALIGN_MFG_PASSED;
	}

	if (!p9221_has_dd(charger))
		return;

	if (charger->pdata->alignment_scalar == 0)
		goto no_scaling;

	res = p9221_reg_read_16(charger, P9221R5_IOUT_REG, &current_now);

	if (res != 0) {
		logbuffer_log(charger->log, "align: failed to read IOUT");
		current_now = 0;
	}

	current_filter_sample =
			charger->current_filtered / WLC_CURRENT_FILTER_LENGTH;

	if (charger->current_sample_cnt < WLC_CURRENT_FILTER_LENGTH)
		charger->current_sample_cnt++;
	else
		charger->current_filtered -= current_filter_sample;

	charger->current_filtered += (current_now / WLC_CURRENT_FILTER_LENGTH);
	dev_dbg(&charger->client->dev, "current = %umA, avg_current = %umA\n",
		current_now, charger->current_filtered);

	current_scaling = charger->pdata->alignment_scalar *
			  charger->current_filtered;

no_scaling:
	res = p9221_reg_read_cooked(charger, P9221R5_OP_FREQ_REG, &wlc_freq);

	if (res != 0) {
		logbuffer_log(charger->log, "align: failed to read op_freq");
		return;
	}

	align_buckets = charger->pdata->nb_alignment_freq - 1;

	charger->alignment = -1;
	wlc_adj_freq = wlc_freq + current_scaling;

	if (wlc_adj_freq < charger->pdata->alignment_freq[0]) {
		logbuffer_log(charger->log, "align: freq below range");
		return;
	}

	for (i = 0; i < align_buckets; i += 1) {
		if ((wlc_adj_freq > charger->pdata->alignment_freq[i]) &&
		    (wlc_adj_freq <= charger->pdata->alignment_freq[i + 1])) {
			charger->alignment = (WLC_ALIGNMENT_MAX * i) /
					     (align_buckets - 1);
			break;
		}
	}

	if (i >= align_buckets) {
		logbuffer_log(charger->log, "align: freq above range");
		return;
	}

	if (charger->alignment == charger->alignment_last)
		return;

	/*
	 *  Frequency needs to be higher than frequency + hysteresis before
	 *  increasing alignment score.
	 */
	wlc_freq_threshold = charger->pdata->alignment_freq[i] +
			     charger->pdata->alignment_hysteresis;

	if ((charger->alignment < charger->alignment_last) ||
	    (wlc_adj_freq >= wlc_freq_threshold)) {
		schedule_work(&charger->uevent_work);
		logbuffer_log(charger->log,
			      "align: alignment=%i. op_freq=%u. current_avg=%u",
			     charger->alignment, wlc_freq,
			     charger->current_filtered);
		charger->alignment_last = charger->alignment;
	}
}

static const char *p9221_get_tx_id_str(struct p9221_charger_data *charger)
{
	int ret;

	if (!p9221_is_online(charger))
		return NULL;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return NULL;
	}
	pm_runtime_put_sync(charger->dev);

	if (p9221_is_epp(charger)) {
		ret = p9221_reg_read_n(charger, P9221R5_PROP_TX_ID_REG,
				       &charger->tx_id, sizeof(charger->tx_id));
		if (ret)
			dev_err(&charger->client->dev,
				"Failed to read txid %d\n", ret);
	} else {
		/*
		 * If pp_buf_valid is true, we have received a serial
		 * number from the Tx, copy it to tx_id. (pp_buf_valid
		 * is left true here until we go offline as we may
		 * read this multiple times.)
		 */
		if (charger->pp_buf_valid &&
		    sizeof(charger->tx_id) <= P9221R5_MAX_PP_BUF_SIZE)
			memcpy(&charger->tx_id, &charger->pp_buf[1],
			       sizeof(charger->tx_id));
	}
	scnprintf(charger->tx_id_str, sizeof(charger->tx_id_str),
		  "%08x", charger->tx_id);
	return charger->tx_id_str;
}

/* txid is available sometime after connect */
static bool p9221_has_dd(struct p9221_charger_data *charger)
{
	u8 val;
	bool ret = false;

	if (p9221_get_tx_id_str(charger) != NULL) {
		val = (charger->tx_id & TXID_TYPE_MASK) >> TXID_TYPE_SHIFT;
		if (val == TXID_DD_TYPE)
			ret = true;
	}

	return ret;
}

static int p9221_get_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (charger->dc_psy) {
			ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_DC_RESET, val);
			if (ret < 0)
				val->intval = 1;
		} else {
			val->intval = 1;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE: /* TODO: verify this */
		if (charger->wait_for_online) {
			val->intval = 1;
			break;
		}

		val->intval = charger->online && charger->enabled;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = p9221_get_tx_id_str(charger);
		if (val->strval == NULL)
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Zero may be returned on transition to wireless "online", as
		 * last_capacity is reset to -1 until capacity is re-written
		 * from userspace, leading to a new csp packet being sent.
		 *
		 * b/80435107 for additional context
		 */
		if (charger->last_capacity > 0)
			val->intval = charger->last_capacity;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (!charger->dc_icl_votable)
			return -EAGAIN;

		ret = get_effective_result(charger->dc_icl_votable);
		if (ret < 0)
			break;

		val->intval = ret;

		/* success */
		ret = 0;
		break;
	default:
		ret = p9221_get_property_reg(charger, prop, val);
		break;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't get prop %d, ret=%d\n", prop, ret);
	return ret;
}

static int p9221_set_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      const union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	int ret = 0;
	bool changed = false;
	u32 threshold;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((val->intval < 0) || (val->intval > 1)) {
			ret = -EINVAL;
			break;
		}

		if (charger->enabled == val->intval)
			break;
		/*
		 * Asserting the enable line will automatically take bring
		 * us online if we are in field.  De-asserting the enable
		 * line will automatically take us offline if we are in field.
		 * This is due to the fact that DC in will change state
		 * appropriately when we change the state of this line.
		 */
		charger->enabled = val->intval;

		dev_warn(&charger->client->dev, "Set enable %d\n",
			 charger->enabled);

		if (charger->pdata->qien_gpio >= 0)
			gpio_set_value(charger->pdata->qien_gpio,
				       charger->enabled ? 0 : 1);

		changed = true;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (charger->last_capacity == val->intval)
			break;

		charger->last_capacity = val->intval;

		if (!charger->online)
			break;

		ret = p9221_send_csp(charger, charger->last_capacity);
		if (ret)
			dev_err(&charger->client->dev,
				"Could not send csp: %d\n", ret);

		threshold = (charger->mitigate_threshold > 0) ?
			    charger->mitigate_threshold :
			    charger->pdata->power_mitigate_threshold;

		if (!threshold)
			break;

		if ((charger->last_capacity > threshold) &&
		    !charger->trigger_power_mitigation) {
			charger->trigger_power_mitigation = true;
			ret = delayed_work_pending(
			      &charger->power_mitigation_work);
			if (!ret)
				schedule_delayed_work(
				    &charger->power_mitigation_work,
				    msecs_to_jiffies(
				    P9221_POWER_MITIGATE_DELAY_MS));
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			ret = -EINVAL;
			break;
		}

		if (!charger->dc_icl_votable) {
			ret = -EAGAIN;
			break;
		}

		ret = vote(charger->dc_icl_votable, P9221_USER_VOTER, true,
			   val->intval);

		changed = true;
		break;
	default:
		ret = p9221_set_property_reg(charger, prop, val);
		if (ret == 0)
			changed = true;
		break;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't set prop %d, ret=%d\n", prop, ret);

	if (changed)
		power_supply_changed(psy);

	return ret;
}

static int p9221_prop_is_writeable(struct power_supply *psy,
				   enum power_supply_property prop)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);

	if (p9221_get_map_entry(charger, prop, true))
		return 1;

	return 0;
}

static int p9221_notifier_cb(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct power_supply *psy = data;
	struct p9221_charger_data *charger =
		container_of(nb, struct p9221_charger_data, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		goto out;

	if (strcmp(psy->desc->name, "dc") == 0) {
		charger->dc_psy = psy;
		charger->check_dc = true;
	}

	if (!charger->check_dc)
		goto out;

	pm_stay_awake(charger->dev);

	if (!schedule_delayed_work(&charger->notifier_work,
				   msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS)))
		pm_relax(charger->dev);

out:
	return NOTIFY_OK;
}

static int p9221_clear_interrupts(struct p9221_charger_data *charger, u16 mask)
{
	int ret;

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_16(charger, P9221R5_INT_CLEAR_REG, mask);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	ret = p9221_set_cmd_reg(charger, P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to reset INT: %d\n", ret);
	}
out:
	mutex_unlock(&charger->cmd_lock);
	return ret;
}

/*
 * Enable interrupts on the P9221, note we don't really need to disable
 * interrupts since when the device goes out of field, the P9221 is reset.
 */
static int p9221_enable_interrupts(struct p9221_charger_data *charger)
{
	u16 mask;
	int ret;

	dev_dbg(&charger->client->dev, "Enable interrupts\n");

	mask = P9221R5_STAT_LIMIT_MASK | P9221R5_STAT_CC_MASK |
	       P9221_STAT_VRECT;

	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_VOUTCHANGED)
		mask |= P9221R5_STAT_VOUTCHANGED;
	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_MODECHANGED)
		mask |= P9221R5_STAT_MODECHANGED;

	ret = p9221_clear_interrupts(charger, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not clear interrupts: %d\n", ret);

	ret = p9221_reg_write_8(charger, P9221_INT_ENABLE_REG, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);
	return ret;
}

static int p9221_set_dc_icl(struct p9221_charger_data *charger)
{
	int icl;
	int ret;

	if (!charger->dc_icl_votable) {
		charger->dc_icl_votable = find_votable("DC_ICL");
		if (!charger->dc_icl_votable) {
			dev_err(&charger->client->dev,
				"Could not get votable: DC_ICL\n");
			return -ENODEV;
		}
	}

	/* Default to BPP ICL */
	icl = P9221_DC_ICL_BPP_UA;

	if (charger->is_low_power_tx)
		icl = P9221_DC_ICL_LOW_POWER_UA;

	if (charger->icl_ramp)
		icl = charger->icl_ramp_ua;

	if (charger->dc_icl_bpp)
		icl = charger->dc_icl_bpp;

	if (p9221_is_epp(charger))
		icl = P9221_DC_ICL_EPP_UA;

	if (charger->re_nego)
		icl = P9221_DC_ICL_EPP_UA;

	dev_info(&charger->client->dev, "Setting ICL %duA ramp=%d\n", icl,
		 charger->icl_ramp);
	if ((charger->icl_ramp) || (charger->re_nego))
		vote(charger->dc_icl_votable, P9221_DEFAULT_VOTER, true, icl);

	ret = vote(charger->dc_icl_votable, P9221_WLC_VOTER, true, icl);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	/* Increase the IOUT limit */
	ret = p9221_reg_write_cooked(charger, P9221R5_ILIM_SET_REG,
				     P9221R5_ILIM_MAX_UA);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not set rx_iout limit reg: %d\n", ret);

	return ret;
}

static enum alarmtimer_restart p9221_icl_ramp_alarm_cb(struct alarm *alarm,
						       ktime_t now)
{
	struct p9221_charger_data *charger =
			container_of(alarm, struct p9221_charger_data,
				     icl_ramp_alarm);

	/* should not schedule icl_ramp_work if charge on low power tx */
	if (charger->is_low_power_tx)
		return ALARMTIMER_NORESTART;

	dev_info(&charger->client->dev, "ICL ramp alarm, ramp=%d\n",
		 charger->icl_ramp);

	/* Alarm is in atomic context, schedule work to complete the task */
	pm_stay_awake(charger->dev);
	schedule_delayed_work(&charger->icl_ramp_work, msecs_to_jiffies(100));

	return ALARMTIMER_NORESTART;
}

static bool p9221_wait_for_negotiation(struct p9221_charger_data *charger)
{
	int ret, loops;
	u8 val = 0;

	ret = p9221_reg_write_8(charger, P9221R5_EPP_RENEGOTIATION_REG,
				P9221R5_EPP_NEGOTIATED_RNTXCAPABREQ);
	if (ret < 0)
		return false;

	ret = p9221_reg_write_8(charger, P9221R5_EPP_REQ_NEGOTIATED_POWER_REG,
				P9221R5_EPP_NEGOTIATED_POWER_10W);
	if (ret < 0)
		return false;

	/* Write 0x80 to 0x4E, check 0x4C reads back as 0x0000 */
	ret = p9221_set_cmd_reg(charger, P9221R5_COM_RENEGOTIATE);
	if (ret != 0)
		return false;

	/* 30 * 100 = 3 sec */
	for (loops = 30; loops; loops--) {
		ret = p9221_reg_read_8(charger, P9221R5_EPP_RENEGOTIATION_REG,
				       &val);
		if (ret < 0)
			break;

		if (val & P9221R5_EPP_NEGOTIATED_RNDONE) {
			logbuffer_log(charger->log,
				      "Run re-negotiation process\n");
			ret = p9221_reg_read_8(
				charger, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG,
				&val);
			if (val == P9221R5_EPP_NEGOTIATED_POWER_10W) {
				ret = p9221_reg_write_8(charger,
							P9221R5_VOUT_SET_REG,
							P9221R5_VOUT_SET_9V);
				if (ret < 0)
					break;
				logbuffer_log(charger->log,
					      "10W detetction is done.\n");
				charger->re_nego = true;
				return true;
			}
			logbuffer_log(charger->log,
				      "less than 10W detection.\n");
			break;
		}

		if (val & P9221R5_EPP_NEGOTIATED_RNERROR) {
			logbuffer_log(charger->log,
				      "Process is done and do nothing.\n");
			break;
		}

		msleep(100);
	}
	if (loops == 0)
		logbuffer_log(charger->log,
			      "re-negotiation loop check timout\n");

	return false;
}

static void p9221_icl_ramp_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, icl_ramp_work.work);
	int ret;
	u8 reg = 0;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		schedule_delayed_work(&charger->icl_ramp_work,
				      msecs_to_jiffies(100));
		dev_dbg(&charger->client->dev, "Ramp reschedule\n");
		return;
	}
	pm_runtime_put_sync(charger->dev);

	dev_info(&charger->client->dev, "ICL ramp work, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = true;
	ret = p9221_reg_read_8(charger, P9221R5_SYSTEM_MODE_REG, &reg);
	if ((ret == 0) && ((reg & P9221R5_SYSTEM_MODE_EXTENDED_MASK) > 0))
		charger->icl_ramp = !p9221_wait_for_negotiation(charger);
	p9221_set_dc_icl(charger);

	pm_relax(charger->dev);
}

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger)
{
	dev_info(&charger->client->dev, "ICL ramp reset, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = false;

	if (alarm_try_to_cancel(&charger->icl_ramp_alarm) < 0)
		dev_warn(&charger->client->dev, "Couldn't cancel icl_ramp_alarm\n");
	cancel_delayed_work(&charger->icl_ramp_work);
}

static void p9221_icl_ramp_start(struct p9221_charger_data *charger)
{
	const bool no_ramp = charger->pdata->icl_ramp_delay_ms == -1 ||
			     !charger->icl_ramp_ua;

	/* Only ramp on BPP at this time */
	if (p9221_is_epp(charger) || no_ramp)
		return;

	p9221_icl_ramp_reset(charger);

	dev_info(&charger->client->dev, "ICL ramp set alarm %dms, %dua, ramp=%d\n",
		 charger->pdata->icl_ramp_delay_ms, charger->icl_ramp_ua,
		 charger->icl_ramp);

	alarm_start_relative(&charger->icl_ramp_alarm,
			     ms_to_ktime(charger->pdata->icl_ramp_delay_ms));
}

static void p9221_set_online(struct p9221_charger_data *charger)
{
	int ret;
	u8 cid = 5;

	dev_info(&charger->client->dev, "Set online\n");

	charger->online = true;
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = false;
	charger->last_capacity = -1;

	ret = p9221_reg_read_8(charger, P9221_CUSTOMER_ID_REG, &cid);
	if (ret)
		dev_err(&charger->client->dev, "Could not get ID: %d\n", ret);
	else
		charger->cust_id = cid;

	dev_info(&charger->client->dev, "P9221 cid: %02x\n", charger->cust_id);

	ret = p9221_enable_interrupts(charger);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);

	/* NOTE: depends on _is_epp() which is not valid until DC_IN */
	p9221_write_fod(charger);

	cancel_delayed_work(&charger->dcin_pon_work);

	charger->alignment_capable = ALIGN_MFG_CHECKING;
	charger->align = POWER_SUPPLY_ALIGN_CENTERED;
	charger->alignment = -1;
	logbuffer_log(charger->log, "align: state: %s",
		      align_status_str[charger->align]);
	schedule_work(&charger->uevent_work);
}

static int p9221_has_dc_in(struct p9221_charger_data *charger)
{
	union power_supply_propval prop;
	int ret;

	if (!charger->dc_psy)
		return -EINVAL;

	ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_PRESENT, &prop);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Error getting charging status: %d\n", ret);
		return -EINVAL;
	}

	return prop.intval != 0;
}

static int p9221_set_bpp_vout(struct p9221_charger_data *charger)
{
	uint8_t val8;
	int ret, loops;
	const uint8_t vout_5000mv = 5 * 10; /* in 0.1V units */

	for (loops = 0; loops < 10; loops++) {
		ret = p9221_reg_write_8(charger,
					P9221R5_VOUT_SET_REG, vout_5000mv);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot set VOUT (%d)\n", ret);
			return ret;
		}

		ret = p9221_reg_read_8(charger, P9221R5_VOUT_SET_REG, &val8);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot read VOUT (%d)\n", ret);
			return ret;
		}

		if (val8 == vout_5000mv)
			return 0;

		msleep(10);
	}

	return -ETIMEDOUT;
}

/* return <0 on error, 0 on done, 1 on keep trying */
static int p9221_notifier_check_neg_power(struct p9221_charger_data *charger)
{
	u8 np8;
	int ret;
	u16 status_reg;

	ret = p9221_reg_read_8(charger, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG,
			       &np8);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"cannot read EPP_NEG_POWER (%d)\n", ret);
		return -EIO;
	}

	if (np8 >= P9221_NEG_POWER_10W) {
		u8 mfg8;

		ret = p9221_reg_read_8(charger, P9221R5_EPP_TX_MFG_CODE_REG,
				       &mfg8);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", ret);
			return -EIO;
		}


		/* EPP unless dealing with P9221_PTMC_EPP_TX_1912 */
		charger->force_bpp = (mfg8 == P9221_PTMC_EPP_TX_1912);
		dev_info(&charger->client->dev, "np=%x mfg=%x fb=%d\n",
			 np8, mfg8, charger->force_bpp);
		goto done;
	}

	ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
	if (ret) {
		dev_err(&charger->client->dev,
			"failed to read P9221_STATUS_REG reg: %d\n",
			ret);
		return ret;
	}

	/* VOUT for standard BPP comes much earlier that VOUT for EPP */
	if (!(status_reg & P9221_STAT_VOUT))
		return 1;

	/* normal BPP TX or EPP at less than 10W */
	charger->force_bpp = true;
	dev_info(&charger->client->dev,
			"np=%x normal BPP or EPP less than 10W (%d)\n",
			np8, ret);

done:
	if (charger->force_bpp) {
		ret = p9221_set_bpp_vout(charger);
		if (ret)
			dev_err(&charger->client->dev,
				"cannot change VOUT (%d)\n", ret);
	}

	return 0;
}

/* 2 P9221_NOTIFIER_DELAY_MS from VRECTON */
static void p9221_notifier_check_dc(struct p9221_charger_data *charger)
{
	int ret, dc_in;

	charger->check_dc = false;

	if (charger->check_np) {

		ret = p9221_notifier_check_neg_power(charger);
		if (ret > 0) {
			ret = schedule_delayed_work(&charger->notifier_work,
				msecs_to_jiffies(P9221_CHECK_NP_DELAY_MS));
			if (ret)
				return;

			dev_err(&charger->client->dev,
				"cannot reschedule check_np (%d)\n", ret);
		}

		/* done */
		charger->check_np = false;
	}

	dc_in = p9221_has_dc_in(charger);
	if (dc_in < 0)
		return;

	dev_info(&charger->client->dev, "dc status is %d\n", dc_in);

	/*
	 * We now have confirmation from DC_IN, kill the timer, charger->online
	 * will be set by this function.
	 */
	cancel_delayed_work(&charger->dcin_work);
	del_timer(&charger->vrect_timer);

	if (charger->log) {
		u32 vout_uv;

		ret = p9221_reg_read_cooked(charger, P9221R5_VOUT_REG,
					    &vout_uv);
		logbuffer_log(charger->log,
			      "check_dc: online=%d present=%d VOUT=%uuV (%d)",
			      charger->online, dc_in,
			      (ret == 0) ? vout_uv : 0, ret);
	}

	/*
	 * Always write FOD, check dc_icl, send CSP
	 */
	if (dc_in) {
		p9221_set_dc_icl(charger);
		p9221_write_fod(charger);
		if (charger->last_capacity > 0)
			p9221_send_csp(charger, charger->last_capacity);
		if (!charger->dc_icl_bpp)
			p9221_icl_ramp_start(charger);
	}

	/* We may have already gone online during check_det */
	if (charger->online == dc_in)
		goto out;

	if (dc_in)
		p9221_set_online(charger);
	else
		p9221_set_offline(charger);

out:
	dev_info(&charger->client->dev, "trigger wc changed on:%d in:%d\n",
		 charger->online, dc_in);
	power_supply_changed(charger->wc_psy);
}

/* P9221_NOTIFIER_DELAY_MS from VRECTON */
bool p9221_notifier_check_det(struct p9221_charger_data *charger)
{
	bool relax = true;

	del_timer(&charger->vrect_timer);

	if (charger->online && !charger->ben_state)
		goto done;

	dev_info(&charger->client->dev, "detected wlc, trigger wc changed\n");

	/* b/130637382 workaround for 2622,2225,2574,1912 */
	charger->check_np = true;
	/* will send out a FOD but is_epp() is still invalid */
	p9221_set_online(charger);
	power_supply_changed(charger->wc_psy);

	/* Check dc-in every seconds as long as we are in field. */
	dev_info(&charger->client->dev, "start dc-in timer\n");
	cancel_delayed_work_sync(&charger->dcin_work);
	schedule_delayed_work(&charger->dcin_work,
			      msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	relax = false;

done:
	charger->check_det = false;

	return relax;
}

static void p9221_notifier_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, notifier_work.work);
	bool relax = true;
	int ret;

	dev_info(&charger->client->dev, "Notifier work: on:%d ben:%d dc:%d np:%d det:%d\n",
		 charger->online,
		 charger->ben_state,
		 charger->check_dc, charger->check_np,
		 charger->check_det);

	if (charger->pdata->q_value != -1) {

		ret = p9221_reg_write_8(charger,
					P9221R5_EPP_Q_FACTOR_REG,
					charger->pdata->q_value);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"cannot write Q=%d (%d)\n",
				 charger->pdata->q_value, ret);
	}

	if (charger->pdata->epp_rp_value != -1) {

		ret = p9221_reg_write_8(charger,
					P9221R5_EPP_REQ_NEGOTIATED_POWER_REG,
					charger->pdata->epp_rp_value);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"cannot write to EPP_NEG_POWER=%d (%d)\n",
				 charger->pdata->epp_rp_value, ret);
	}

	if (charger->log) {
		u32 vrect_uv;

		ret = p9221_reg_read_cooked(charger, P9221R5_VRECT_REG,
					    &vrect_uv);
		logbuffer_log(charger->log,
			      "notifier: on:%d ben:%d dc:%d det:%d VRECT=%uuV (%d)",
			      charger->online,
			      charger->ben_state,
			      charger->check_dc, charger->check_det,
			      (ret == 0) ? vrect_uv : 0, ret);
	}

	if (charger->check_det)
		relax = p9221_notifier_check_det(charger);

	if (charger->check_dc)
		p9221_notifier_check_dc(charger);

	if (relax)
		pm_relax(charger->dev);
}

static ssize_t p9221_add_reg_buffer(struct p9221_charger_data *charger,
				    char *buf, size_t count, u16 reg, int width,
				    bool cooked, const char *name, char *fmt)
{
	u32 val;
	int ret;
	int added = 0;

	if (cooked)
		ret = p9221_reg_read_cooked(charger, reg, &val);
	else if (width == 16) {
		u16 val16 = 0;
		ret = p9221_reg_read_16(charger, reg, &val16);
		val = val16;
	} else {
		u8 val8 = 0;
		ret = p9221_reg_read_8(charger, reg, &val8);
		val = val8;
	}

	added += scnprintf(buf + count, PAGE_SIZE - count, "%s", name);
	count += added;
	if (ret)
		added += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else
		added += scnprintf(buf + count, PAGE_SIZE - count, fmt, val);

	return added;
}

static ssize_t p9221_show_version(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0;
	int i;
	int ret;
	u8 val8 = 0;

	if (!p9221_is_online(charger))
		return -ENODEV;

	count += p9221_add_reg_buffer(charger, buf, count, P9221_CHIP_ID_REG,
				      16, 0, "chip id    : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CHIP_REVISION_REG, 8, 0,
				      "chip rev   : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CUSTOMER_ID_REG, 8, 0,
				      "cust id    : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MAJOR_REV_REG, 16, 0,
				      "otp fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MINOR_REV_REG, 16, 0,
				      "otp fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "otp fw date: ");
	for (i = 0; i < P9221_OTP_FW_DATE_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\notp fw time: ");
	for (i = 0; i < P9221_OTP_FW_TIME_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_OTP_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MAJOR_REV_REG, 16, 0,
				      "\nram fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MINOR_REV_REG, 16, 0,
				      "ram fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "ram fw date: ");
	for (i = 0; i < P9221_SRAM_FW_DATE_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\nram fw time: ");
	for (i = 0; i < P9221_SRAM_FW_TIME_SIZE; i++) {
		ret = p9221_reg_read_8(charger,
				       P9221_SRAM_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(version, 0444, p9221_show_version, NULL);

static ssize_t p9221_show_status(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0;
	int ret;
	u8 tmp[P9221R5_NUM_FOD];
	uint32_t tx_id = 0;

	if (!p9221_is_online(charger))
		return -ENODEV;

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_STATUS_REG, 16, 0,
				      "status      : ", "%04x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_INT_REG, 16, 0,
				      "int         : ", "%04x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_INT_ENABLE_REG, 16, 0,
				      "int_enable  : ", "%04x\n");


	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_SYSTEM_MODE_REG, 8, 0,
				      "mode        : ", "%02x\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_VOUT_REG, 16, 1,
				      "vout        : ", "%d uV\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_VRECT_REG, 16, 1,
				      "vrect       : ", "%d uV\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_IOUT_REG, 16, 1,
				      "iout        : ", "%d uA\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ILIM_SET_REG, 16, 1,
				      "ilim        : ", "%d uA\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_OP_FREQ_REG, 16, 1,
				      "freq        : ", "%d hz\n");
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_busy     : %d\n", charger->tx_busy);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_done     : %d\n", charger->tx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_done     : %d\n", charger->rx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_len      : %d\n", charger->tx_len);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_len      : %d\n", charger->rx_len);
	p9221_reg_read_n(charger, P9221R5_PROP_TX_ID_REG, &tx_id,
			 sizeof(tx_id));
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_id       : %08x (%s)\n", tx_id,
			   p9221_get_tx_id_str(charger));

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ALIGN_X_ADC_REG, 8, 1,
				      "align_x     : ", "%d\n");

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221R5_ALIGN_Y_ADC_REG, 8, 1,
				      "align_y     : ", "%d\n");

	/* FOD Register */
	ret = p9221_reg_read_n(charger, P9221R5_FOD_REG, tmp, P9221R5_NUM_FOD);
	count += scnprintf(buf + count, PAGE_SIZE - count, "fod         : ");
	if (ret)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else {
		count += p9221_hex_str(tmp, P9221R5_NUM_FOD, buf + count, count,
				       false);
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	/* Device tree FOD entries */
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "dt fod      : (n=%d) ", charger->pdata->fod_num);
	count += p9221_hex_str(charger->pdata->fod, charger->pdata->fod_num,
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\ndt fod-epp  : (n=%d) ",
			   charger->pdata->fod_epp_num);
	count += p9221_hex_str(charger->pdata->fod_epp,
			       charger->pdata->fod_epp_num,
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\npp buf      : (v=%d) ", charger->pp_buf_valid);
	count += p9221_hex_str(charger->pp_buf, sizeof(charger->pp_buf),
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(status, 0444, p9221_show_status, NULL);

static ssize_t p9221_show_count(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%u\n", charger->count);
}

static ssize_t p9221_store_count(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u8 cnt;

	ret = kstrtou8(buf, 0, &cnt);
	if (ret < 0)
		return ret;
	charger->count = cnt;
	return count;
}

static DEVICE_ATTR(count, 0644, p9221_show_count, p9221_store_count);

static ssize_t p9221_show_icl_ramp_delay_ms(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 charger->pdata->icl_ramp_delay_ms);
}

static ssize_t p9221_store_icl_ramp_delay_ms(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ms;

	ret = kstrtou32(buf, 10, &ms);
	if (ret < 0)
		return ret;
	charger->pdata->icl_ramp_delay_ms = ms;
	return count;
}

static DEVICE_ATTR(icl_ramp_delay_ms, 0644,
		   p9221_show_icl_ramp_delay_ms,
		   p9221_store_icl_ramp_delay_ms);

static ssize_t p9221_show_icl_ramp_ua(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->icl_ramp_ua);
}

static ssize_t p9221_store_icl_ramp_ua(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;
	charger->icl_ramp_ua = ua;
	return count;
}

static DEVICE_ATTR(icl_ramp_ua, 0644,
		   p9221_show_icl_ramp_ua, p9221_store_icl_ramp_ua);

static ssize_t p9221_show_addr(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", charger->addr);
}

static ssize_t p9221_store_addr(struct device *dev,
			        struct device_attribute *attr,
			        const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 addr;

	ret = kstrtou16(buf, 16, &addr);
	if (ret < 0)
		return ret;
	charger->addr = addr;
	return count;
}

static DEVICE_ATTR(addr, 0644, p9221_show_addr, p9221_store_addr);

static ssize_t p9221_show_data(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int ret;
	int i;
	ssize_t len = 0;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	ret = p9221_reg_read_n(charger, charger->addr, reg, charger->count);
	if (ret)
		return ret;

	for (i = 0; i < charger->count; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02x: %02x\n",
				 charger->addr + i, reg[i]);
	}
	return len;
}

static ssize_t p9221_store_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int i = 0;
	int ret = 0;
	char *data;
	char *tmp_buf;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	tmp_buf = kmalloc(strlen(buf) + 1, GFP_KERNEL);
	data = tmp_buf;
	if (!data)
		return -ENOMEM;

	strcpy(data, buf);
	while(data && i < charger->count) {
		char *d = strsep(&data, " ");
		if (*d) {
			ret = kstrtou8(d, 16, &reg[i]);
			if (ret)
				break;
			i++;
		}
	}
	if ((i != charger->count) || ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = p9221_reg_write_n(charger, charger->addr, reg, charger->count);
	if (ret)
		goto out;
	ret = count;

out:
	kfree(tmp_buf);
	return ret;
}

static DEVICE_ATTR(data, 0644, p9221_show_data, p9221_store_data);

static ssize_t p9221_store_ccreset(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;

	ret = p9221_send_ccreset(charger);
	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR(ccreset, 0200, NULL, p9221_store_ccreset);

static ssize_t p9221_show_rxdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->rx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(rxdone, 0444, p9221_show_rxdone, NULL);

static ssize_t p9221_show_rxlen(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%hu\n", charger->rx_len);
}

static DEVICE_ATTR(rxlen, 0444, p9221_show_rxlen, NULL);

static ssize_t p9221_show_txdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txdone, 0444, p9221_show_txdone, NULL);

static ssize_t p9221_show_txbusy(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_busy ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txbusy, 0444, p9221_show_txbusy, NULL);

static ssize_t p9221_store_txlen(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 len;

	ret = kstrtou16(buf, 16, &len);
	if (ret < 0)
		return ret;
	if (ret > P9221R5_COM_CHAN_SEND_SIZE_REG)
		return -EINVAL;

	cancel_delayed_work_sync(&charger->tx_work);

	charger->tx_len = len;
	charger->tx_done = false;
	ret = p9221_send_data(charger);
	if (ret) {
		charger->tx_done = true;
		return ret;
	}

	schedule_delayed_work(&charger->tx_work,
			      msecs_to_jiffies(P9221_TX_TIMEOUT_MS));

	return count;
}

static DEVICE_ATTR(txlen, 0200, NULL, p9221_store_txlen);

static ssize_t p9221_show_force_epp(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->fake_force_epp ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static ssize_t p9221_force_epp(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 val;

	ret = kstrtou16(buf, 16, &val);
	if (ret < 0)
		return ret;

	charger->fake_force_epp = (val != 0);

	if (charger->pdata->slct_gpio >= 0)
		gpio_set_value(charger->pdata->slct_gpio,
			       charger->fake_force_epp ? 1 : 0);
	return count;
}

static DEVICE_ATTR(force_epp, 0600, p9221_show_force_epp, p9221_force_epp);

static ssize_t p9221_show_dc_icl_bpp(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->dc_icl_bpp);
}

static ssize_t p9221_set_dc_icl_bpp(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;

	charger->dc_icl_bpp = ua;

	if (charger->dc_icl_votable && !p9221_is_epp(charger))
		vote(charger->dc_icl_votable,
		     P9221_WLC_VOTER, true, charger->dc_icl_bpp);

	return count;
}

static DEVICE_ATTR(dc_icl_bpp, 0644,
		   p9221_show_dc_icl_bpp, p9221_set_dc_icl_bpp);

static ssize_t p9221_show_alignment(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->alignment == -1)
		p9221_init_align(charger);

	if ((charger->align != POWER_SUPPLY_ALIGN_CENTERED) ||
	    (charger->alignment == -1))
		return scnprintf(buf, PAGE_SIZE, "%s\n",
				 align_status_str[charger->align]);
	else
		return scnprintf(buf, PAGE_SIZE, "%d\n", charger->alignment);
}

static DEVICE_ATTR(alignment, 0444, p9221_show_alignment, NULL);


static ssize_t p9221_show_mitigate_threshold(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%u\n", charger->mitigate_threshold);
}

static ssize_t p9221_store_mitigate_threshold(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u8 threshold;

	ret = kstrtou8(buf, 0, &threshold);
	if (ret < 0)
		return ret;
	charger->mitigate_threshold = threshold;
	return count;
}

static DEVICE_ATTR(mitigate_threshold, 0644,
		   p9221_show_mitigate_threshold,
		   p9221_store_mitigate_threshold);

/* ------------------------------------------------------------------------ */

static ssize_t p9382_show_rtx_sw(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->switch_gpio < 0)
		return -ENODEV;

	value = gpio_get_value(charger->pdata->switch_gpio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t p9382_set_rtx_sw(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->pdata->switch_gpio < 0)
		return -ENODEV;

	/* TODO: better test on rX mode */
	if (charger->online) {
		dev_err(&charger->client->dev, "invalid rX state");
		return -EINVAL;
	}

	gpio_set_value(charger->pdata->switch_gpio, buf[0] != '0');

	return count;
}

static DEVICE_ATTR(rtx_sw, 0644, p9382_show_rtx_sw, p9382_set_rtx_sw);


static ssize_t p9382_show_rtx_boost(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->ben_state);
}

/* assume that we have 2 GPIO to turn on the boost */
static int p9382_rtx_enable(struct p9221_charger_data *charger, bool enable)
{
	if (charger->pdata->ben_gpio >= 0)
		gpio_set_value(charger->pdata->ben_gpio, enable);
	if (charger->pdata->switch_gpio >= 0)
		gpio_set_value(charger->pdata->switch_gpio, enable);

	return (charger->pdata->ben_gpio >= 0 ||
		charger->pdata->switch_gpio >= 0);
}

static int p9382_ben_cfg(struct p9221_charger_data *charger, int cfg)
{
	const int ben_gpio = charger->pdata->ben_gpio;
	const int switch_gpio = charger->pdata->switch_gpio;

	dev_info(&charger->client->dev, "ben_cfg: %d->%d (ben=%d, switch=%d)",
		 charger->ben_state, cfg, ben_gpio, switch_gpio);

	switch (cfg) {
	case RTX_BEN_DISABLED:
		if (charger->ben_state == RTX_BEN_ON)
			p9382_rtx_enable(charger, false);
		else if (ben_gpio >= 0)
			gpio_set_value(ben_gpio, 0);
		charger->ben_state = cfg;
		break;
	case RTX_BEN_ENABLED:
		charger->ben_state = cfg;
		if (ben_gpio >= 0)
			gpio_set_value(ben_gpio, 1);
		break;
	case RTX_BEN_ON:
		charger->ben_state = cfg;
		p9382_rtx_enable(charger, true);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t p9382_set_rtx_boost(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	const int state = buf[0] - '0';
	int ret;

	/* always ok to disable */
	if (state && charger->online && !charger->ben_state) {
		dev_err(&charger->client->dev, "invalid rX state");
		return -ENODEV;
	}

	/* 0 -> BEN_DISABLED, 1 -> BEN_ON */
	ret = p9382_ben_cfg(charger, state);
	if (ret < 0)
		count = ret;

	return count;
}

static DEVICE_ATTR(rtx_boost, 0644, p9382_show_rtx_boost, p9382_set_rtx_boost);


static ssize_t p9382_show_rtx(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->ben_state);
}

static int p9382_wait_for_mode(struct p9221_charger_data *charger, int mode)
{
	int loops, ret;
	uint8_t sys_mode;

	/* 30 * 100 = 3 sec */
	for (loops = 30 ; loops ; loops--) {
		ret = p9221_reg_read_8(charger, P9221R5_SYSTEM_MODE_REG,
					&sys_mode);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot read system_mode (%d)", ret);
			return -EIO;
		}

		if (sys_mode == mode)
			return 0;

		msleep(100);
	}

	return -ETIMEDOUT;
}

/* write 1 to enable boost & switch, write 0 to 0x34, wait for 0x4c==0x4
 * write 0 to write 0x80 to 0x4E, wait for 0x4c==0, disable boost & switch
 */
static ssize_t p9382_set_rtx(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;

	if (buf[0] == '0') {
		/* Write 0x80 to 0x4E, check 0x4C reads back as 0x0000 */
		ret = p9221_set_cmd_reg(charger, P9221R5_COM_RENEGOTIATE);
		if (ret == 0) {
			ret = p9382_wait_for_mode(charger, 0);
			if (ret < 0)
				pr_err("cannot exit rTX mode (%d)\n", ret);
		}

		ret = p9382_ben_cfg(charger, RTX_BEN_DISABLED);
		if (ret < 0)
			return ret;

	} else if (buf[0] == '1') {
		ret = p9382_ben_cfg(charger, RTX_BEN_ON);
		if (ret < 0)
			return ret;

		/* write 0x0000 to 0x34, check 0x4C reads back as 0x04 */
		ret = p9221_reg_write_16(charger, P9382A_STATUS_REG, 0);
		if (ret == 0)
			ret = p9382_wait_for_mode(charger, P9382A_MODE_TXMODE);

		if (ret < 0) {
			pr_err("cannot enter rTX mode (%d)\n", ret);
			return ret;
		}

	} else {
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(rtx, 0644, p9382_show_rtx, p9382_set_rtx);

/* ------------------------------------------------------------------------ */

static struct attribute *p9221_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_status.attr,
	&dev_attr_addr.attr,
	&dev_attr_count.attr,
	&dev_attr_data.attr,
	&dev_attr_ccreset.attr,
	&dev_attr_txbusy.attr,
	&dev_attr_txdone.attr,
	&dev_attr_txlen.attr,
	&dev_attr_rxlen.attr,
	&dev_attr_rxdone.attr,
	&dev_attr_icl_ramp_ua.attr,
	&dev_attr_icl_ramp_delay_ms.attr,
	&dev_attr_force_epp.attr,
	&dev_attr_dc_icl_bpp.attr,
	&dev_attr_rtx_sw.attr,
	&dev_attr_rtx_boost.attr,
	&dev_attr_rtx.attr,
	&dev_attr_alignment.attr,
	&dev_attr_mitigate_threshold.attr,
	NULL
};

static ssize_t p9221_rxdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->rx_buf[pos], size);
	charger->rx_done = false;
	return size;
}

static struct bin_attribute bin_attr_rxdata = {
	.attr = {
		.name = "rxdata",
		.mode = 0400,
	},
	.read = p9221_rxdata_read,
	.size = P9221R5_DATA_RECV_BUF_SIZE,
};

static ssize_t p9221_txdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->tx_buf[pos], size);
	return size;
}

static ssize_t p9221_txdata_write(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(&charger->tx_buf[pos], buf, size);
	return size;
}

static struct bin_attribute bin_attr_txdata = {
	.attr = {
		.name = "txdata",
		.mode = 0600,
	},
	.read = p9221_txdata_read,
	.write = p9221_txdata_write,
	.size  = P9221R5_DATA_SEND_BUF_SIZE,
};

static struct bin_attribute *p9221_bin_attributes[] = {
	&bin_attr_txdata,
	&bin_attr_rxdata,
	NULL,
};

static const struct attribute_group p9221_attr_group = {
	.attrs		= p9221_attributes,
	.bin_attrs	= p9221_bin_attributes,
};

static void print_current_samples(struct p9221_charger_data *charger,
					u32 *iout_val, int count)
{
	int i;
	char temp[P9221R5_OVER_CHECK_NUM * 9 + 1] = { 0 };

	for (i = 0; i < count ; i++)
		scnprintf(temp + i * 9, sizeof(temp) - i * 9,
			  "%08x ", iout_val[i]);

	dev_info(&charger->client->dev, "OVER IOUT_SAMPLES: %s\n", temp);
}

/*
 * Number of times to poll the status to see if the current limit condition
 * was transient or not.
 */
static void p9221_over_handle(struct p9221_charger_data *charger,
			      u16 irq_src)
{
	u8 reason = 0;
	int i;
	int ret;
	int ovc_count = 0;
	u32 iout_val[P9221R5_OVER_CHECK_NUM] = { 0 };

	dev_err(&charger->client->dev, "Received OVER INT: %02x\n", irq_src);

	if (irq_src & P9221R5_STAT_OVV) {
		reason = P9221_EOP_OVER_VOLT;
		goto send_eop;
	}

	if (irq_src & P9221R5_STAT_OVT) {
		reason = P9221_EOP_OVER_TEMP;
		goto send_eop;
	}

	if ((irq_src & P9221R5_STAT_UV) && !(irq_src & P9221R5_STAT_OVC))
		return;

	/* Overcurrent, reduce ICL and poll to absorb any transients */

	if (charger->dc_icl_votable) {
		int icl;

		icl = get_effective_result_locked(charger->dc_icl_votable);
		if (icl < 0) {
			dev_err(&charger->client->dev,
				"Failed to read ICL (%d)\n", icl);
		} else if (icl > OVC_BACKOFF_LIMIT) {
			icl -= OVC_BACKOFF_AMOUNT;

			ret = vote(charger->dc_icl_votable,
				   P9221_OCP_VOTER, true,
				   icl);
			dev_err(&charger->client->dev,
				"Reduced ICL to %d (%d)\n", icl, ret);
		}
	}

	reason = P9221_EOP_OVER_CURRENT;
	for (i = 0; i < P9221R5_OVER_CHECK_NUM; i++) {
		ret = p9221_clear_interrupts(charger,
					     irq_src & P9221R5_STAT_LIMIT_MASK);
		msleep(50);
		if (ret)
			continue;

		ret = p9221_reg_read_cooked(charger, P9221R5_IOUT_REG,
			&iout_val[i]);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read IOUT[%d]: %d\n", i, ret);
			continue;
		} else if (iout_val[i] > OVC_THRESHOLD) {
			ovc_count++;
		}

		ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &irq_src);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read status: %d\n", ret);
			continue;
		}

		if ((irq_src & P9221R5_STAT_OVC) == 0) {
			print_current_samples(charger, iout_val, i + 1);
			dev_info(&charger->client->dev,
				 "OVER condition %04x cleared after %d tries\n",
				 irq_src, i);
			return;
		}

		dev_err(&charger->client->dev,
			"OVER status is still %04x, retry\n", irq_src);
	}

	if (ovc_count < OVC_LIMIT) {
		print_current_samples(charger, iout_val,
				      P9221R5_OVER_CHECK_NUM);
		dev_info(&charger->client->dev,
			 "ovc_threshold=%d, ovc_count=%d, ovc_limit=%d\n",
			 OVC_THRESHOLD, ovc_count, OVC_LIMIT);
		return;
	}

send_eop:
	dev_err(&charger->client->dev,
		"OVER is %04x, sending EOP %d\n", irq_src, reason);

	ret = p9221_send_eop(charger, reason);
	if (ret)
		dev_err(&charger->client->dev,
			"Failed to send EOP %d: %d\n", reason, ret);
}

static bool p9221_dc_reset_needed(struct p9221_charger_data *charger,
				  u16 irq_src)
{

	/*
	 * It is suspected that p9221 misses to set the interrupt status
	 * register occasionally. Evaluate spurious interrupt case for
	 * dc reset as well.
	 */
	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_MODECHANGED &&
	    (irq_src & P9221R5_STAT_MODECHANGED || !irq_src)) {
		u8 mode_reg = 0;
		int res;

		res = p9221_reg_read_8(charger, P9221R5_SYSTEM_MODE_REG,
				       &mode_reg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"Failed to read P9221_SYSTEM_MODE_REG: %d\n",
				res);
			/*
			 * p9221_reg_read_n returns ENODEV for ENOTCONN as well.
			 * Signal dc_reset when register read fails with the
			 * above reasons.
			 */
			return res == -ENODEV;
		}

		dev_info(&charger->client->dev,
			 "P9221_SYSTEM_MODE_REG reg: %02x\n", mode_reg);
		return !(mode_reg & (P9221R5_MODE_EXTENDED |
				     P9221R5_MODE_WPCMODE));
	}

	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_VOUTCHANGED &&
	    irq_src & P9221R5_STAT_VOUTCHANGED) {
		u16 status_reg = 0;
		int res;

		res = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"Failed to read P9221_STATUS_REG: %d\n", res);
			return res == -ENODEV ? true : false;
		}

		dev_info(&charger->client->dev,
			 "P9221_STATUS_REG reg: %04x\n", status_reg);
		return !(status_reg & P9221_STAT_VOUT);
	}

	return false;
}

/* Handler for R5 and R7 chips */
static void p9221_irq_handler(struct p9221_charger_data *charger, u16 irq_src)
{
	int res;

	if (p9221_dc_reset_needed(charger, irq_src)) {
		union power_supply_propval val = {.intval = 1};

		if (!charger->dc_psy)
			charger->dc_psy = power_supply_get_by_name("dc");
		if (charger->dc_psy) {
			/* Signal DC_RESET when wireless removal is sensed. */
			res = power_supply_set_property(charger->dc_psy,
						POWER_SUPPLY_PROP_DC_RESET,
						&val);
		} else {
			res = -ENODEV;
		}

		if (res < 0)
			dev_err(&charger->client->dev,
				"unable to set DC_RESET, ret=%d",
				res);
	}

	if (irq_src & P9221R5_STAT_LIMIT_MASK)
		p9221_over_handle(charger, irq_src);

	/* Receive complete */
	if (irq_src & P9221R5_STAT_CCDATARCVD) {
		uint8_t rxlen = 0;

		res = p9221_reg_read_8(charger, P9221R5_COM_CHAN_RECV_SIZE_REG,
				       &rxlen);
		if (res) {
			dev_err(&charger->client->dev,
				"Failed to read len: %d\n", res);
			rxlen = 0;
		}
		if (rxlen) {
			res = p9221_reg_read_n(charger,
					charger->addr_data_recv_buf_start,
					charger->rx_buf, rxlen);
			if (res)
				dev_err(&charger->client->dev,
					"Failed to read len: %d\n", res);

			charger->rx_len = rxlen;
			charger->rx_done = true;
			sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
		}
	}

	/* Send complete */
	if (irq_src & P9221R5_STAT_CCSENDBUSY) {
		charger->tx_busy = false;
		charger->tx_done = true;
		cancel_delayed_work(&charger->tx_work);
		sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
		sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	}

	/* Proprietary packet */
	if (irq_src & P9221R5_STAT_PPRCVD) {
		u8 buff[sizeof(charger->pp_buf)];

		res = p9221_reg_read_n(charger,
				       charger->addr_data_recv_buf_start,
				       buff,
				       sizeof(buff));
		if (res) {
			dev_err(&charger->client->dev,
				"Failed to read PP len: %d\n", res);
		} else if (buff[0] == FAST_SERIAL_ID_HEADER) {
			const size_t maxsz = sizeof(charger->pp_buf) * 3 + 1;
			char s[maxsz];
			u8 tmp;

			memcpy(charger->pp_buf, buff, sizeof(charger->pp_buf));

			/* We only care about PP which come with 0x4F header */
			charger->pp_buf_valid = true;

			p9221_hex_str(charger->pp_buf, sizeof(charger->pp_buf),
				      s, maxsz, false);
			dev_info(&charger->client->dev,
				 "Received PP: %s\n", s);

			tmp = charger->pp_buf[4] & ACCESSORY_TYPE_MASK;
			charger->is_low_power_tx =
				(tmp == ACCESSORY_TYPE_LOW_POWER_TX);
			dev_info(&charger->client->dev,
				 "is_low_power_tx=%d\n",
				 charger->is_low_power_tx);
			if (charger->is_low_power_tx) {
				vote(charger->dc_icl_votable,
				     P9382A_LOW_POWER_TX_VOTER,
				     true, P9221_DC_ICL_LOW_POWER_UA);
				dev_info(&charger->client->dev,
					 "set ICL to %dmA",
					 P9221_DC_ICL_LOW_POWER_UA/1000);
			}
		}
	}

	/* CC Reset complete */
	if (irq_src & P9221R5_STAT_CCRESET)
		p9221_abort_transfers(charger);
}

static irqreturn_t p9221_irq_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;
	int ret;
	u16 irq_src = 0;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

	ret = p9221_reg_read_16(charger, P9221_INT_REG, &irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to read INT reg: %d\n", ret);
		goto out;
	}

	/* TODO: interrupt storm with irq_src = when in rTX mode */
	if (!charger->ben_state) {
		dev_info(&charger->client->dev, "INT: %04x\n", irq_src);
		logbuffer_log(charger->log, "INT=%04x on:%d",
			      irq_src, charger->online);
	}

	if (!irq_src)
		goto out;

	ret = p9221_clear_interrupts(charger, irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	/* todo interrupt handling for rx */
	if (charger->ben_state)
		goto out;

	if (irq_src & P9221_STAT_VRECT) {
		dev_info(&charger->client->dev,
			"Received VRECTON, online=%d\n", charger->online);
		if (!charger->online) {
			charger->check_det = true;
			pm_stay_awake(charger->dev);

			if (!schedule_delayed_work(&charger->notifier_work,
				msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS))) {
				pm_relax(charger->dev);
			}
		}
	}

	p9221_irq_handler(charger, irq_src);

out:
	return IRQ_HANDLED;
}

static irqreturn_t p9221_irq_det_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;

	logbuffer_log(charger->log, "irq_det: online=%d ben=%d",
		      charger->online, charger->ben_state);

	/* If we are already online, just ignore the interrupt. */
	if (p9221_is_online(charger))
		return IRQ_HANDLED;

	if (charger->align != POWER_SUPPLY_ALIGN_MOVE) {
		if (charger->align != POWER_SUPPLY_ALIGN_CHECKING)
			schedule_work(&charger->uevent_work);
		charger->align = POWER_SUPPLY_ALIGN_CHECKING;
		charger->align_count++;

		if (charger->align_count > WLC_ALIGN_IRQ_THRESHOLD) {
			schedule_work(&charger->uevent_work);
			charger->align = POWER_SUPPLY_ALIGN_MOVE;
		}
		logbuffer_log(charger->log, "align: state: %s",
			      align_status_str[charger->align]);
	}

	del_timer(&charger->align_timer);

	/*
	 * This interrupt will wake the device if it's suspended,
	 * but it is not reliable enough to trigger the charging indicator.
	 * Give ourselves 2 seconds for the VRECTON interrupt to appear
	 * before we put up the charging indicator.
	 */
	mod_timer(&charger->vrect_timer,
		  jiffies + msecs_to_jiffies(P9221_VRECT_TIMEOUT_MS));
	pm_stay_awake(charger->dev);

	return IRQ_HANDLED;
}

static void p9221_uevent_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, uevent_work);

	kobject_uevent(&charger->dev->kobj, KOBJ_CHANGE);
}

static int p9221_parse_dt(struct device *dev,
			  struct p9221_charger_platform_data *pdata)
{
	int ret = 0;
	u32 data;
	struct device_node *node = dev->of_node;

	/* Enable */
	ret = of_get_named_gpio(node, "idt,gpio_qien", 0);
	pdata->qien_gpio = ret;
	if (ret < 0)
		dev_warn(dev, "unable to read idt,gpio_qien from dt: %d\n",
			 ret);
	else
		dev_info(dev, "enable gpio:%d", pdata->qien_gpio);

	/* WLC_BPP_EPP_SLCT */
	ret = of_get_named_gpio(node, "idt,gpio_slct", 0);
	pdata->slct_gpio = ret;
	if (ret < 0) {
		dev_warn(dev, "unable to read idt,gpio_slct from dt: %d\n",
			 ret);
	} else {
		ret = of_property_read_u32(node, "idt,gpio_slct_value", &data);
		if (ret == 0)
			pdata->slct_value = (data != 0);

		dev_info(dev, "WLC_BPP_EPP_SLCT gpio:%d value=%d",
					pdata->slct_gpio, pdata->slct_value);
	}

	/* boost enable, power WLC IC from device */
	ret = of_get_named_gpio(node, "idt,gpio_ben", 0);
	pdata->ben_gpio = ret;
	if (ret >= 0)
		dev_info(dev, "ben gpio:%d\n", pdata->ben_gpio);

	ret = of_get_named_gpio(node, "idt,gpio_switch", 0);
	pdata->switch_gpio = ret;
	if (ret >= 0)
		dev_info(dev, "switch gpio:%d\n", pdata->switch_gpio);

	/* Main IRQ */
	ret = of_get_named_gpio(node, "idt,irq_gpio", 0);
	if (ret < 0) {
		dev_err(dev, "unable to read idt,irq_gpio from dt: %d\n", ret);
		return ret;
	}
	pdata->irq_gpio = ret;
	pdata->irq_int = gpio_to_irq(pdata->irq_gpio);
	dev_info(dev, "gpio:%d, gpio_irq:%d\n", pdata->irq_gpio,
		 pdata->irq_int);

	/* Optional Detect IRQ */
	ret = of_get_named_gpio(node, "idt,irq_det_gpio", 0);
	pdata->irq_det_gpio = ret;
	if (ret < 0) {
		dev_warn(dev, "unable to read idt,irq_det_gpio from dt: %d\n",
			 ret);
	} else {
		pdata->irq_det_int = gpio_to_irq(pdata->irq_det_gpio);
		dev_info(dev, "det gpio:%d, det gpio_irq:%d\n",
			 pdata->irq_det_gpio, pdata->irq_det_int);
	}

	/* Optional VOUT max */
	pdata->max_vout_mv = P9221_MAX_VOUT_SET_MV_DEFAULT;
	ret = of_property_read_u32(node, "max_vout_mv", &data);
	if (ret == 0) {
		if (data < 3500 || data > 29000)
			dev_err(dev, "max_vout_mv out of range %d\n", data);
		else
			pdata->max_vout_mv = data;
	}

	/* Optional FOD data */
	pdata->fod_num =
	    of_property_count_elems_of_size(node, "fod", sizeof(u8));
	if (pdata->fod_num <= 0) {
		dev_err(dev, "No dt fod provided (%d)\n", pdata->fod_num);
		pdata->fod_num = 0;
	} else {
		if (pdata->fod_num > P9221R5_NUM_FOD) {
			dev_err(dev,
			    "Incorrect num of FOD %d, using first %d\n",
			    pdata->fod_num, P9221R5_NUM_FOD);
			pdata->fod_num = P9221R5_NUM_FOD;
		}
		ret = of_property_read_u8_array(node, "fod", pdata->fod,
						pdata->fod_num);
		if (ret == 0) {
			char buf[pdata->fod_num * 3 + 1];

			p9221_hex_str(pdata->fod, pdata->fod_num, buf,
				      pdata->fod_num * 3 + 1, false);
			dev_info(dev, "dt fod: %s (%d)\n", buf, pdata->fod_num);
		}
	}

	pdata->fod_epp_num =
	    of_property_count_elems_of_size(node, "fod_epp", sizeof(u8));
	if (pdata->fod_epp_num <= 0) {
		dev_err(dev, "No dt fod epp provided (%d)\n",
			pdata->fod_epp_num);
		pdata->fod_epp_num = 0;
	} else {
		if (pdata->fod_epp_num > P9221R5_NUM_FOD) {
			dev_err(dev,
			    "Incorrect num of EPP FOD %d, using first %d\n",
			    pdata->fod_epp_num, P9221R5_NUM_FOD);
			pdata->fod_epp_num = P9221R5_NUM_FOD;
		}
		ret = of_property_read_u8_array(node, "fod_epp", pdata->fod_epp,
						pdata->fod_epp_num);
		if (ret == 0) {
			char buf[pdata->fod_epp_num * 3 + 1];

			p9221_hex_str(pdata->fod_epp, pdata->fod_epp_num, buf,
				      pdata->fod_epp_num * 3 + 1, false);
			dev_info(dev, "dt fod_epp: %s (%d)\n", buf,
				 pdata->fod_epp_num);
		}
	}

	ret = of_property_read_u32(node, "google,q_value", &data);
	if (ret < 0) {
		pdata->q_value = -1;
	} else {
		pdata->q_value = data;
		dev_info(dev, "dt q_value:%d\n", pdata->q_value);
	}

	ret = of_property_read_u32(node, "google,epp_rp_value", &data);
	if (ret < 0) {
		pdata->epp_rp_value = -1;
	} else {
		pdata->epp_rp_value = data;
		dev_info(dev, "dt epp_rp_value: %d\n", pdata->epp_rp_value);
	}

	ret = of_property_read_u32(node, "google,needs_dcin_reset", &data);
	if (ret < 0) {
		pdata->needs_dcin_reset = -1;
	} else {
		pdata->needs_dcin_reset = data;
		dev_info(dev, "dt needs_dcin_reset: %d\n",
			 pdata->needs_dcin_reset);
	}

	pdata->nb_alignment_freq =
			of_property_count_elems_of_size(node,
							"google,alignment_frequencies",
							sizeof(u32));
	dev_info(dev, "dt google,alignment_frequencies size = %d\n",
		 pdata->nb_alignment_freq);

	if (pdata->nb_alignment_freq > 0) {
		pdata->alignment_freq =
				devm_kmalloc_array(dev,
						   pdata->nb_alignment_freq,
						   sizeof(u32),
						   GFP_KERNEL);
		if (!pdata->alignment_freq) {
			dev_warn(dev,
				 "dt google,alignment_frequencies array not created");
		} else {
			ret = of_property_read_u32_array(node,
							 "google,alignment_frequencies",
							 pdata->alignment_freq,
							 pdata->nb_alignment_freq);
			if (ret) {
				dev_warn(dev,
					 "failed to read google,alignment_frequencies: %d\n",
					 ret);
				devm_kfree(dev, pdata->alignment_freq);
			}
		}
	}

	ret = of_property_read_u32(node, "google,alignment_scalar", &data);
	if (ret < 0)
		pdata->alignment_scalar = WLC_ALIGN_DEFAULT_SCALAR;
	else {
		pdata->alignment_scalar = data;
		if (pdata->alignment_scalar != WLC_ALIGN_DEFAULT_SCALAR)
			dev_info(dev, "google,alignment_scalar updated to: %d\n",
				 pdata->alignment_scalar);
	}

	ret = of_property_read_u32(node, "google,alignment_hysteresis", &data);
	if (ret < 0)
		pdata->alignment_hysteresis = WLC_ALIGN_DEFAULT_HYSTERESIS;
	else
		pdata->alignment_hysteresis = data;

	dev_info(dev, "google,alignment_hysteresis set to: %d\n",
				 pdata->alignment_hysteresis);

	ret = of_property_read_bool(node, "idt,ramp-disable");
	if (ret)
		pdata->icl_ramp_delay_ms = -1 ;

	ret = of_property_read_u32(node, "google,power_mitigate_threshold",
				   &data);
	if (ret < 0)
		pdata->power_mitigate_threshold = 0;
	else
		pdata->power_mitigate_threshold = data;

	return 0;
}

static enum power_supply_property p9221_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_OPERATING_FREQ,
};

static const struct power_supply_desc p9221_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = p9221_props,
	.num_properties = ARRAY_SIZE(p9221_props),
	.get_property = p9221_get_property,
	.set_property = p9221_set_property,
	.property_is_writeable = p9221_prop_is_writeable,
	.no_thermal = true,
};

/* return true when online */
static bool p9221_get_chip_id(struct p9221_charger_data *charger, u16 *chip_id)
{
	int ret;

	/* Test to see if the charger is online */
	ret = p9221_reg_read_16(charger, P9221_CHIP_ID_REG, chip_id);
	if (ret == 0) {
		dev_info(charger->dev, "Charger online id:%04x\n", *chip_id);
		return true;
	}

	/* off, try to power on the WLC chip */
	ret = p9382_rtx_enable(charger, true);
	if (ret == 0) {
		ret = p9221_reg_read_16(charger, P9221_CHIP_ID_REG, chip_id);
		p9382_rtx_enable(charger, false);

		if (ret == 0) {
			dev_info(charger->dev, "Charger rTX id:%04x\n",
				 *chip_id);
			return false;
		}
	}

	/* hack/fallback use the chip address. useful for prototypes that might
	 * not have the ability to power the chip.
	 */
	if (charger->client->addr == P9382A_I2C_ADDRESS) {
		dev_info(charger->dev, "Charger rTX address :%02x\n",
			 charger->client->addr);
		*chip_id = P9382A_CHIP_ID;
	}

	return false;
}

static int p9221_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *of_node = client->dev.of_node;
	struct p9221_charger_data *charger;
	struct p9221_charger_platform_data *pdata = client->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	u16 chip_id = 0;
	bool online;
	int ret;

	ret = i2c_check_functionality(client->adapter,
				      I2C_FUNC_SMBUS_BYTE_DATA |
				      I2C_FUNC_SMBUS_WORD_DATA |
				      I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret) {
		ret = i2c_get_functionality(client->adapter);
		dev_err(&client->dev, "I2C adapter not compatible %x\n", ret);
		return -ENOSYS;
	}

	if (of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate pdata\n");
			return -ENOMEM;
		}
		ret = p9221_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dt\n");
			return ret;
		}
	}

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (charger == NULL) {
		dev_err(&client->dev, "Failed to allocate charger\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, charger);
	charger->dev = &client->dev;
	charger->client = client;
	charger->pdata = pdata;
	charger->resume_complete = true;
	charger->align = POWER_SUPPLY_ALIGN_ERROR;
	charger->align_count = 0;
	charger->is_mfg_google = false;
	mutex_init(&charger->io_lock);
	mutex_init(&charger->cmd_lock);
	setup_timer(&charger->vrect_timer, p9221_vrect_timer_handler,
		    (unsigned long)charger);
	setup_timer(&charger->align_timer, p9221_align_timer_handler,
		    (unsigned long)charger);
	INIT_DELAYED_WORK(&charger->dcin_work, p9221_dcin_work);
	INIT_DELAYED_WORK(&charger->tx_work, p9221_tx_work);
	INIT_DELAYED_WORK(&charger->icl_ramp_work, p9221_icl_ramp_work);
	INIT_DELAYED_WORK(&charger->align_work, p9221_align_work);
	INIT_DELAYED_WORK(&charger->dcin_pon_work, p9221_dcin_pon_work);
	INIT_DELAYED_WORK(&charger->power_mitigation_work,
			  p9221_power_mitigation_work);
	INIT_WORK(&charger->uevent_work, p9221_uevent_work);
	alarm_init(&charger->icl_ramp_alarm, ALARM_BOOTTIME,
		   p9221_icl_ramp_alarm_cb);

	/* Default enable */
	charger->enabled = true;
	if (charger->pdata->qien_gpio >= 0)
		gpio_direction_output(charger->pdata->qien_gpio, 0);

	if (charger->pdata->slct_gpio >= 0)
		gpio_direction_output(charger->pdata->slct_gpio,
				      charger->pdata->slct_value);

	if (charger->pdata->ben_gpio >= 0)
		gpio_direction_output(charger->pdata->ben_gpio, 0);

	if (charger->pdata->switch_gpio >= 0)
		gpio_direction_output(charger->pdata->switch_gpio, 0);

	/* Default to R5+ */
	charger->cust_id = 5;

	psy_cfg.drv_data = charger;
	psy_cfg.of_node = charger->dev->of_node;
	charger->wc_psy = devm_power_supply_register(charger->dev,
						     &p9221_psy_desc,
						     &psy_cfg);
	if (IS_ERR(charger->wc_psy)) {
		dev_err(&client->dev, "Fail to register supply: %d\n", ret);
		return PTR_ERR(charger->wc_psy);
	}

	/*
	 * Find the DC_ICL votable, we use this to limit the current that
	 * is taken from the wireless charger.
	 */
	charger->dc_icl_votable = find_votable("DC_ICL");
	if (!charger->dc_icl_votable)
		dev_warn(&charger->client->dev, "Could not find DC_ICL votable\n");

	charger->re_nego = false;

	/* Ramping on BPP is optional */
	if (charger->pdata->icl_ramp_delay_ms != -1) {
		charger->icl_ramp_ua = P9221_DC_ICL_BPP_RAMP_DEFAULT_UA;
		charger->pdata->icl_ramp_delay_ms =
					P9221_DC_ICL_BPP_RAMP_DELAY_DEFAULT_MS;
	}

	charger->dc_icl_bpp = 0;

	/* valid chip_id [P9382A_CHIP_ID, P9221_CHIP_ID] or 0 */
	online = p9221_get_chip_id(charger, &chip_id);
	if (chip_id == P9382A_CHIP_ID) {
		charger->addr_data_send_buf_start = P9382A_DATA_SEND_BUF_START;
		charger->addr_data_recv_buf_start = P9382A_DATA_RECV_BUF_START;
	} else {
		charger->addr_data_send_buf_start = P9221R5_DATA_SEND_BUF_START;
		charger->addr_data_recv_buf_start = P9221R5_DATA_RECV_BUF_START;
	}

	if (online) {
		/* set charger->online=true, will ignore first VRECTON IRQ */
		p9221_set_online(charger);
	} else {
		/* disconnected, (likely err!=0) vote for BPP */
		p9221_vote_defaults(charger);
	}

	ret = devm_request_threaded_irq(
		&client->dev, charger->pdata->irq_int, NULL,
		p9221_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		"p9221-irq", charger);
	if (ret) {
		dev_err(&client->dev, "Failed to request IRQ\n");
		return ret;
	}
	device_init_wakeup(charger->dev, true);

	/*
	 * We will receive a VRECTON after enabling IRQ if the device is
	 * if the device is already in-field when the driver is probed.
	 */
	enable_irq_wake(charger->pdata->irq_int);

	if (gpio_is_valid(charger->pdata->irq_det_gpio)) {
		ret = devm_request_threaded_irq(
			&client->dev, charger->pdata->irq_det_int, NULL,
			p9221_irq_det_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "p9221-irq-det",
			charger);
		if (ret) {
			dev_err(&client->dev, "Failed to request IRQ_DET\n");
			return ret;
		}

		ret = devm_gpio_request_one(&client->dev,
					    charger->pdata->irq_det_gpio,
					    GPIOF_DIR_IN, "p9221-det-gpio");
		if (ret) {
			dev_err(&client->dev, "Failed to request GPIO_DET\n");
			return ret;
		}
		enable_irq_wake(charger->pdata->irq_det_int);
	}

	charger->last_capacity = -1;
	charger->count = 1;
	ret = sysfs_create_group(&charger->dev->kobj, &p9221_attr_group);
	if (ret) {
		dev_info(&client->dev, "sysfs_create_group failed\n");
	}

	/*
	 * Register notifier so we can detect changes on DC_IN
	 */
	INIT_DELAYED_WORK(&charger->notifier_work, p9221_notifier_work);
	charger->nb.notifier_call = p9221_notifier_cb;
	ret = power_supply_reg_notifier(&charger->nb);
	if (ret) {
		dev_err(&client->dev, "Fail to register notifier: %d\n", ret);
		return ret;
	}

	charger->log = debugfs_logbuffer_register("wireless");
	if (IS_ERR(charger->log)) {
		ret = PTR_ERR(charger->log);
		dev_err(charger->dev,
			"failed to obtain logbuffer instance, ret=%d\n", ret);
		charger->log = NULL;
	}

	dev_info(&client->dev, "p9221 Charger Driver Loaded\n");

	if (online) {
		charger->dc_psy = power_supply_get_by_name("dc");
		if (charger->dc_psy)
			power_supply_changed(charger->dc_psy);
	}

	return 0;
}

static int p9221_charger_remove(struct i2c_client *client)
{
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&charger->dcin_work);
	cancel_delayed_work_sync(&charger->tx_work);
	cancel_delayed_work_sync(&charger->icl_ramp_work);
	cancel_delayed_work_sync(&charger->dcin_pon_work);
	cancel_delayed_work_sync(&charger->align_work);
	cancel_delayed_work_sync(&charger->power_mitigation_work);
	cancel_work_sync(&charger->uevent_work);
	alarm_try_to_cancel(&charger->icl_ramp_alarm);
	del_timer_sync(&charger->vrect_timer);
	del_timer_sync(&charger->align_timer);
	device_init_wakeup(charger->dev, false);
	cancel_delayed_work_sync(&charger->notifier_work);
	power_supply_unreg_notifier(&charger->nb);
	mutex_destroy(&charger->io_lock);
	if (charger->log)
		debugfs_logbuffer_unregister(charger->log);
	return 0;
}

static const struct i2c_device_id p9221_charger_id_table[] = {
	{ "p9221", 0 },
	{ "p9382", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, p9221_charger_id_table);

#ifdef CONFIG_OF
static struct of_device_id p9221_charger_match_table[] = {
	{ .compatible = "idt,p9221",},
	{ .compatible = "idt,p9382",},
	{},
};
#else
#define p9221_charger_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int p9221_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	charger->resume_complete = false;
	pm_runtime_put_sync(charger->dev);

	return 0;
}

static int p9221_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	charger->resume_complete = true;
	pm_runtime_put_sync(charger->dev);

	return 0;
}
#endif
static const struct dev_pm_ops p9221_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(p9221_pm_suspend, p9221_pm_resume)
};

static struct i2c_driver p9221_charger_driver = {
	.driver = {
		.name		= "p9221",
		.owner		= THIS_MODULE,
		.of_match_table = p9221_charger_match_table,
		.pm		= &p9221_pm_ops,
	},
	.probe		= p9221_charger_probe,
	.remove		= p9221_charger_remove,
	.id_table	= p9221_charger_id_table,
};

static int __init p9221_charger_init(void)
{
	return i2c_add_driver(&p9221_charger_driver);
}

static void __exit p9221_charger_exit(void)
{
	i2c_del_driver(&p9221_charger_driver);
}

module_init(p9221_charger_init);
module_exit(p9221_charger_exit);

MODULE_DESCRIPTION("IDT P9221 Wireless Power Receiver Driver");
MODULE_AUTHOR("Patrick Tjin <pattjin@google.com>");
MODULE_LICENSE("GPL");
