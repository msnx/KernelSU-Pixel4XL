/*
 * Copyright (C) 2017-2018 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/module.h>

#include "s2mpg01-core.h"

/* used for register access attributes */
static u8 _reg_addr;

static int s2mpg01_adc_single_chan_wrapper(struct s2mpg01_core *ddata,
					   unsigned int channel,
					   int *measured_data)
{
	u8 raw_data = 0xFF;
	int scale = 0;
	int ret;

	ret = s2mpg01_read_adc_chan(ddata, channel, &raw_data);
	if (ret) {
		dev_err(ddata->dev,
			"%s: failed to read adc channel %u\n",
			__func__, channel);
		return ret;
	}

	switch (channel) {
	case S2MPG01_ADC_I_SMPS1_SUM:
		scale = S2MPG01_ADC_SCALE_I_SMPS1;
		break;
	case S2MPG01_ADC_I_SMPS2:
		scale = S2MPG01_ADC_SCALE_I_SMPS2;
		break;
	case S2MPG01_ADC_I_SMPS3:
		scale = S2MPG01_ADC_SCALE_I_SMPS3;
		break;
	case S2MPG01_ADC_V_SMPS1:
		scale = S2MPG01_ADC_SCALE_V_SMPS1;
		break;
	case S2MPG01_ADC_V_SMPS2:
		scale = S2MPG01_ADC_SCALE_V_SMPS2;
		break;
	case S2MPG01_ADC_V_SMPS3:
		scale = S2MPG01_ADC_SCALE_V_SMPS3;
		break;
	case S2MPG01_ADC_I_SMPS1_PH1:
		scale = S2MPG01_ADC_SCALE_I_SMPS1_PH1;
		break;
	case S2MPG01_ADC_I_SMPS1_PH2:
		scale = S2MPG01_ADC_SCALE_I_SMPS1_PH2;
		break;
	case S2MPG01_ADC_I_SMPS1_PH3:
		scale = S2MPG01_ADC_SCALE_I_SMPS1_PH3;
		break;
	case S2MPG01_ADC_VBAT:
		scale = S2MPG01_ADC_SCALE_VBAT;
		break;
	case S2MPG01_ADC_I_LDO1:
		scale = S2MPG01_ADC_SCALE_I_LDO1;
		break;
	case S2MPG01_ADC_I_LDO2:
		scale = S2MPG01_ADC_SCALE_I_LDO2;
		break;
	case S2MPG01_ADC_I_LDO3:
		scale = S2MPG01_ADC_SCALE_I_LDO3;
		break;
	case S2MPG01_ADC_I_LDO4:
		scale = S2MPG01_ADC_SCALE_I_LDO4;
		break;
	case S2MPG01_ADC_I_LDO5:
		scale = S2MPG01_ADC_SCALE_I_LDO5;
		break;
	case S2MPG01_ADC_V_LDO1:
		scale = S2MPG01_ADC_SCALE_V_LDO1;
		break;
	case S2MPG01_ADC_V_LDO2:
		scale = S2MPG01_ADC_SCALE_V_LDO2;
		break;
	case S2MPG01_ADC_V_LDO3:
		scale = S2MPG01_ADC_SCALE_V_LDO3;
		break;
	case S2MPG01_ADC_V_LDO4:
		scale = S2MPG01_ADC_SCALE_V_LDO4;
		break;
	case S2MPG01_ADC_V_LDO5:
		scale = S2MPG01_ADC_SCALE_V_LDO5;
		break;
	case S2MPG01_ADC_PTAT:
		/* temperature is a special case */
		*measured_data = PTAT_CODE_TO_TEMP(raw_data);
		return 0;
	default:
		dev_err(ddata->dev,
			"%s: unrecognized channel %u\n",
			__func__, channel);
		break;
	}

	*measured_data = raw_data * scale;
	return 0;
}

/* show smps1_volt in mV */
static ssize_t smps1_volt_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_SMPS1,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps1_volt);

/* show smps1_curr in mA */
static ssize_t smps1_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS1_SUM,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps1_curr);

/* show smps1_ph1_curr in mA */
static ssize_t smps1_ph1_curr_show(struct device *dev,
				   struct device_attribute *mattr,
				   char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS1_PH1,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps1_ph1_curr);

/* show smps1_ph2_curr in mA */
static ssize_t smps1_ph2_curr_show(struct device *dev,
				   struct device_attribute *mattr,
				   char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS1_PH2,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps1_ph2_curr);

/* show smps1_ph3_curr in mA */
static ssize_t smps1_ph3_curr_show(struct device *dev,
				   struct device_attribute *mattr,
				   char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS1_PH3,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps1_ph3_curr);

/* show smps2_volt in mV */
static ssize_t smps2_volt_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_SMPS2,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps2_volt);

/* show smps2_curr in mA */
static ssize_t smps2_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS2,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps2_curr);

/* show smps3_volt in mV */
static ssize_t smps3_volt_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_SMPS3,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps3_volt);

/* show smps3_curr in mA */
static ssize_t smps3_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_SMPS3,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(smps3_curr);

/* show ldo1_volt in mV */
static ssize_t ldo1_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_LDO1,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo1_volt);

/* show ldo1_curr in mA */
static ssize_t ldo1_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_LDO1,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo1_curr);

/* show ldo2_volt in mV */
static ssize_t ldo2_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_LDO2,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo2_volt);

/* show ldo2_curr in mA */
static ssize_t ldo2_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_LDO2,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo2_curr);

/* show ldo3_volt in mV */
static ssize_t ldo3_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_LDO3,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo3_volt);

/* show ldo3_curr in mA */
static ssize_t ldo3_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_LDO3,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo3_curr);

/* show ldo4_volt in mV */
static ssize_t ldo4_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_LDO4,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo4_volt);

/* show ldo4_curr in mA */
static ssize_t ldo4_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_LDO4,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo4_curr);

/* show ldo5_volt in mV */
static ssize_t ldo5_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_V_LDO5,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo5_volt);

/* show ldo5_curr in mA */
static ssize_t ldo5_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_I_LDO5,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mA\n", measured_data / 1000);
}
DEVICE_ATTR_RO(ldo5_curr);

/* show vbat in mV */
static ssize_t vbat_show(struct device *dev,
			 struct device_attribute *mattr,
			 char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_VBAT,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d mV\n", measured_data / 1000);
}
DEVICE_ATTR_RO(vbat);

/* show temperature in degC */
static ssize_t temperature_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int measured_data;
	int ret;

	ret = s2mpg01_adc_single_chan_wrapper(ddata,
					      S2MPG01_ADC_PTAT,
					      &measured_data);
	if (ret)
		return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

	return scnprintf(data, PAGE_SIZE, "%d degC\n", measured_data / 1000);
}
DEVICE_ATTR_RO(temperature);

/* shows power in mW */
static ssize_t total_power_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int ret;
	int i;
	long total_power = 0;
	int measured_volt, measured_curr;
	unsigned int iv_channels[][2] = {
		/* current_channel, voltage_channel */
		{ S2MPG01_ADC_I_SMPS1_SUM, S2MPG01_ADC_V_SMPS1 },
		{ S2MPG01_ADC_I_SMPS2, S2MPG01_ADC_V_SMPS2 },
		{ S2MPG01_ADC_I_SMPS3, S2MPG01_ADC_V_SMPS3 },
		{ S2MPG01_ADC_I_LDO1, S2MPG01_ADC_V_LDO1 },
		{ S2MPG01_ADC_I_LDO2, S2MPG01_ADC_V_LDO2 },
		{ S2MPG01_ADC_I_LDO3, S2MPG01_ADC_V_LDO3 },
		{ S2MPG01_ADC_I_LDO4, S2MPG01_ADC_V_LDO4 },
		{ S2MPG01_ADC_I_LDO5, S2MPG01_ADC_V_LDO5 },
	};

	for (i = 0; i < ARRAY_SIZE(iv_channels); ++i) {
		ret = s2mpg01_adc_single_chan_wrapper(ddata,
						      iv_channels[i][0],
						      &measured_curr);
		if (ret)
			return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

		ret = s2mpg01_adc_single_chan_wrapper(ddata,
						      iv_channels[i][1],
						      &measured_volt);
		if (ret)
			return scnprintf(data, PAGE_SIZE, "Failed: %d\n", ret);

		total_power += (long)measured_volt / 1000
				* (long)measured_curr / 1000;
	}

	return scnprintf(data, PAGE_SIZE, "%ld mW\n", total_power / 1000);
}
DEVICE_ATTR_RO(total_power);

static ssize_t boost_mode_store(struct device *dev,
				struct device_attribute *mattr,
				const char *buf,
				size_t count)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);

	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val == 0)
		s2mpg01_disable_boost(ddata);
	else
		s2mpg01_enable_boost(ddata);

	return count;
}

static ssize_t boost_mode_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	bool boost_mode_status;
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);

	boost_mode_status = s2mpg01_boost_mode_status(ddata);

	return scnprintf(data, PAGE_SIZE,
			 boost_mode_status ?
			 "Boost mode enabled\n" : "Boost mode disabled\n");
}
DEVICE_ATTR_RW(boost_mode);

static ssize_t dump_regs_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);

	s2mpg01_dump_regs(ddata);

	return scnprintf(data, PAGE_SIZE, "ok\n");
}
DEVICE_ATTR_RO(dump_regs);

static ssize_t toggle_pon_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);

	s2mpg01_toggle_pon(ddata);

	return scnprintf(data, PAGE_SIZE, "ok\n");
}
DEVICE_ATTR_RO(toggle_pon);

static ssize_t toggle_pon_oneway_show(struct device *dev,
				      struct device_attribute *mattr,
				      char *data)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	static bool is_on = true;

	s2mpg01_toggle_pon_oneway(ddata, !is_on);

	is_on = !is_on;

	return scnprintf(data, PAGE_SIZE, "set pon to %d\n", is_on);
}
DEVICE_ATTR_RO(toggle_pon_oneway);

static ssize_t reg_addr_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", _reg_addr);
}

static ssize_t reg_addr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if ((val < 0) || (val > 0xff))
		return -EINVAL;

	dev_dbg(ddata->dev, "%s: _reg_addr %d\n", __func__, val);

	_reg_addr = val;

	return count;
}
DEVICE_ATTR_RW(reg_addr);

static ssize_t reg_data_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	u8 reg_data;

	s2mpg01_read_byte(ddata, _reg_addr, &reg_data);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", reg_data);
}

static ssize_t reg_data_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	struct s2mpg01_core *ddata = dev_get_drvdata(dev);
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if ((val < 0) || (val > 0xff))
		return -EINVAL;

	dev_dbg(ddata->dev, "%s: reg 0x%02x, data 0x%02x\n", __func__,
		_reg_addr, val);

	s2mpg01_write_byte(ddata, _reg_addr, val);

	return count;
}
DEVICE_ATTR_RW(reg_data);

static struct attribute *s2mpg01_attrs[] = {
	&dev_attr_smps1_volt.attr,
	&dev_attr_smps1_curr.attr,
	&dev_attr_smps1_ph1_curr.attr,
	&dev_attr_smps1_ph2_curr.attr,
	&dev_attr_smps1_ph3_curr.attr,
	&dev_attr_smps2_volt.attr,
	&dev_attr_smps2_curr.attr,
	&dev_attr_smps3_volt.attr,
	&dev_attr_smps3_curr.attr,
	&dev_attr_ldo1_volt.attr,
	&dev_attr_ldo1_curr.attr,
	&dev_attr_ldo2_volt.attr,
	&dev_attr_ldo2_curr.attr,
	&dev_attr_ldo3_volt.attr,
	&dev_attr_ldo3_curr.attr,
	&dev_attr_ldo4_volt.attr,
	&dev_attr_ldo4_curr.attr,
	&dev_attr_ldo5_volt.attr,
	&dev_attr_ldo5_curr.attr,
	&dev_attr_vbat.attr,
	&dev_attr_temperature.attr,
	&dev_attr_total_power.attr,
	&dev_attr_boost_mode.attr,
	&dev_attr_dump_regs.attr,
	&dev_attr_toggle_pon.attr,
	&dev_attr_toggle_pon_oneway.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_data.attr,
	NULL
};

static const struct attribute_group s2mpg01_attr_group = {
	.attrs = s2mpg01_attrs,
};

void s2mpg01_config_sysfs(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &s2mpg01_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s: could not create sysfs attributes (%d)\n",
			__func__, ret);
	}
}
EXPORT_SYMBOL_GPL(s2mpg01_config_sysfs);

