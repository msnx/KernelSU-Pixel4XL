/*
 * rt5514.c  --  RT5514 ALSA SoC audio codec driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rl6231.h"
#include "rt5514.h"
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
#include "rt5514-spi.h"
#endif
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
#include <linux/codec-misc.h>
#endif

struct rt5514_priv *g_rt5514;
const struct reg_sequence *rt5514_i2c_patch;

static const struct reg_sequence const_rt5514p_i2c_patch[] = {
	{0xfafafafa, 0x00000001},
	{0x18002000, 0x000010ec},
	{0x18002004, 0x00808f81},
	{0x18002008, 0x00770000},
	{0x18002f08, 0x00000006},
	{0x18002f10, 0x00000000},
	{0x18002f10, 0x00000001},
	{0xfafafafa, 0x00000000},
	{0x18001104, 0x00000007},
	{0x18001108, 0x00000000},
	{0x1800110c, 0x00000000},
	{0x18001100, 0x0000031f},
	{0x18002000, 0x000010ec},
};

static const struct reg_sequence const_rt5514_i2c_patch[] = {
	{0x1800101c, 0x00000000},
	{0x18001100, 0x0000031f},
	{0x18001104, 0x00000007},
	{0x18001108, 0x00000000},
	{0x1800110c, 0x00000000},
	{0x18001110, 0x00000000},
	{0x18001118, 0x00000000},
	{0x18002f08, 0x00000006},
	{0x18002f00, 0x00055149},
	{0x18002f00, 0x0005514b},
	{0x18002f00, 0x00055149},
	{0xfafafafa, 0x00000001},
	{0x18002f10, 0x00000001},
	{0x18002f10, 0x00000000},
	{0x18002f10, 0x00000001},
	{0xfafafafa, 0x00000001},
	{0x18002000, 0x000010ec},
	{0xfafafafa, 0x00000000},
	{0x18001044, 0x00000000},
};

static const struct reg_sequence rt5514_patch[] = {
	{RT5514_DIG_IO_CTRL,		0x00000040},
	{RT5514_CLK_CTRL1,		0x380200c1},
	{RT5514_SRC_CTRL,		0x44000eee},
	{RT5514_ANA_CTRL_LDO10,		0x00028704},
	{RT5514_ANA_CTRL_ADCFED,	0x00000800},
	{RT5514_ASRC_IN_CTRL1,		0x00000003},
	{RT5514_DOWNFILTER0_CTRL1,	0x0002042f},
	{RT5514_DOWNFILTER0_CTRL2,	0x0002042f},
	{RT5514_DOWNFILTER0_CTRL3,	0x10000342},
	{RT5514_DOWNFILTER1_CTRL1,	0x0002042f},
	{RT5514_DOWNFILTER1_CTRL2,	0x0002042f},
	{RT5514_DOWNFILTER1_CTRL3,	0x10000342},
};

static const struct reg_default rt5514_reg[] = {
	{RT5514_RESET,			0x00000000},
	{RT5514_PWR_ANA1,		0x00808880},
	{RT5514_PWR_ANA2,		0x00220000},
	{RT5514_I2S_CTRL1,		0x00000330},
	{RT5514_I2S_CTRL2,		0x20000000},
	{RT5514_VAD_CTRL6,		0xc00007d2},
	{RT5514_EXT_VAD_CTRL,		0x80000080},
	{RT5514_DIG_IO_CTRL,		0x00000040},
	{RT5514_PAD_CTRL1,		0x00804000},
	{RT5514_DMIC_DATA_CTRL,		0x00000005},
	{RT5514_DIG_SOURCE_CTRL,	0x00000002},
	{RT5514_SRC_CTRL,		0x44000eee},
	{RT5514_DOWNFILTER2_CTRL1,	0x0000882f},
	{RT5514_PLL_SOURCE_CTRL,	0x00000004},
	{RT5514_CLK_CTRL1,		0x380200c1},
	{RT5514_CLK_CTRL2,		0x00000000},
	{RT5514_PLL3_CALIB_CTRL1,	0x00400200},
	{RT5514_PLL3_CALIB_CTRL5,	0x40220012},
	{RT5514_DELAY_BUF_CTRL1,	0x7fff006a},
	{RT5514_DELAY_BUF_CTRL3,	0x00000000},
	{RT5514_DOWNFILTER0_CTRL1,	0x0002042f},
	{RT5514_DOWNFILTER0_CTRL2,	0x0002042f},
	{RT5514_DOWNFILTER0_CTRL3,	0x10000342},
	{RT5514_DOWNFILTER1_CTRL1,	0x0002042f},
	{RT5514_DOWNFILTER1_CTRL2,	0x0002042f},
	{RT5514_DOWNFILTER1_CTRL3,	0x10000342},
	{RT5514_ANA_CTRL_LDO10,		0x00028704},
	{RT5514_ANA_CTRL_LDO18_16,	0x02000345},
	{RT5514_ANA_CTRL_ADC12,		0x0000a2a8},
	{RT5514_ANA_CTRL_ADC21,		0x00001180},
	{RT5514_ANA_CTRL_ADC22,		0x0000aaa8},
	{RT5514_ANA_CTRL_ADC23,		0x00151427},
	{RT5514_ANA_CTRL_MICBST,	0x00002000},
	{RT5514_ANA_CTRL_ADCFED,	0x00000800},
	{RT5514_ANA_CTRL_INBUF,		0x00000143},
	{RT5514_ANA_CTRL_VREF,		0x00008d50},
	{RT5514_ANA_CTRL_PLL3,		0x0000000e},
	{RT5514_ANA_CTRL_PLL2_1,	0x00000000},
	{RT5514_ANA_CTRL_PLL2_2,	0x00030220},
	{RT5514_ANA_CTRL_PLL1_1,	0x00000000},
	{RT5514_ANA_CTRL_PLL1_2,	0x00030220},
	{RT5514_DMIC_LP_CTRL,		0x00000000},
	{RT5514_MISC_CTRL_DSP,		0x00000000},
	{RT5514_DSP_CTRL1,		0x00055149},
	{RT5514_DSP_CTRL3,		0x00000006},
	{RT5514_DSP_CTRL4,		0x00000001},
	{RT5514_VENDOR_ID1,		0x00000001},
	{RT5514_VENDOR_ID2,		0x10ec5514},
};

int rt5514_set_gpio(int gpio, bool output)
{
	switch (gpio) {
	case 5:
		regmap_update_bits(rt5514_g_i2c_regmap, 0x18002070,
			1 << 8, 1 << 8);
		regmap_update_bits(rt5514_g_i2c_regmap, 0x18002074,
			1 << 21 | 1 << 22, output << 21 | 1 << 22);
		break;

	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rt5514_set_gpio);

static void rt5514_filter_power_reset(struct rt5514_priv *rt5514)
{
	rt5514_spi_request_switch(SPI_SWITCH_MASK_RESET, 1);

	/* Following register control will cause Filter component power reset,
	 * this will reset buffer pointer as well
	 */
	regmap_write(rt5514->i2c_regmap, 0x18001014, 1);

	/* Notify CHRE DSP buffer reset when following register is Zero */
	regmap_write(rt5514->i2c_regmap, RT5514_DSP_CHRE_INFORM, 0);

	rt5514_spi_request_switch(SPI_SWITCH_MASK_RESET, 0);
}

static void rt5514_enable_dsp_prepare(struct rt5514_priv *rt5514)
{
	if (rt5514->v_p) {
		regmap_write(rt5514->i2c_regmap, 0x18002004, 0x00808f81);
		regmap_write(rt5514->i2c_regmap, 0x18002008, 0x00770000);
		/* LDO_I_limit */
		regmap_write(rt5514->i2c_regmap, 0x18002200, 0x00028704);
		/* PIN config */
		regmap_write(rt5514->i2c_regmap, 0x18002070, 0x00000040);
		/* pll3=1.3M*31=40M */
		regmap_write(rt5514->i2c_regmap, 0x18002240, 0x0000001e);
		/* PLL3 source=RCOSC, fsi=rt_clk */
		regmap_write(rt5514->i2c_regmap, 0x18002100, 0x0000000b);
		/* DSP clk source = pll3, ENABLE DSP clk */
		regmap_write(rt5514->i2c_regmap, 0x18002f08, 0x00000005);
		/* Reduce DSP power */
		regmap_write(rt5514->i2c_regmap, 0x18001118, 0x00000001);
		/* Buffer data mono/stereo */
		regmap_write(rt5514->i2c_regmap, 0x18002fcc,
			rt5514->dsp_buffer_channel);
		/* DFLL reset */
		regmap_write(rt5514->i2c_regmap, 0x18002124, 0x00220012);
		/* DFLL, set m/n(1220) */
		regmap_write(rt5514->i2c_regmap, 0x18002110, 0x000104c4);
		/* DFLL,reset DFLL */
		regmap_write(rt5514->i2c_regmap, 0x18002124, 0x80220012);
		/* DFLL */
		regmap_write(rt5514->i2c_regmap, 0x18002124, 0xc0220012);
	} else {
		/* Reset */
		regmap_write(rt5514->i2c_regmap, 0x18002000, 0x000010ec);
		/* LDO_I_limit */
		regmap_write(rt5514->i2c_regmap, 0x18002200, 0x00028704);
		/* I2C bypass enable */
		regmap_write(rt5514->i2c_regmap, 0xfafafafa, 0x00000001);
		/* mini-core reset */
		regmap_write(rt5514->i2c_regmap, 0x18002f00, 0x0005514b);
		regmap_write(rt5514->i2c_regmap, 0x18002f00, 0x00055149);
		/* I2C bypass disable */
		regmap_write(rt5514->i2c_regmap, 0xfafafafa, 0x00000000);
		/* PIN config */
		regmap_write(rt5514->i2c_regmap, 0x18002070, 0x00000040);
		/* PLL3(QN)=RCOSC*(22+2) */
		regmap_write(rt5514->i2c_regmap, 0x18002240, 0x00000016);
		/* PLL3 source=RCOSC, fsi=rt_clk */
		regmap_write(rt5514->i2c_regmap, 0x18002100, 0x0000000b);
		/* Power on RCOSC, pll3 */
		regmap_write(rt5514->i2c_regmap, 0x18002004, 0x00808b81);
		/* DSP clk source = pll3, ENABLE DSP clk */
		regmap_write(rt5514->i2c_regmap, 0x18002f08, 0x00000005);
		/* Enable DSP clk auto switch */
		regmap_write(rt5514->i2c_regmap, 0x18001114, 0x00000001);
		/* Reduce DSP power */
		regmap_write(rt5514->i2c_regmap, 0x18001118, 0x00000001);
		/* Buffer data mono/stereo */
		regmap_write(rt5514->i2c_regmap, 0x18002fcc,
			rt5514->dsp_buffer_channel);
	}
}

static bool rt5514_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RT5514_VENDOR_ID1:
	case RT5514_VENDOR_ID2:
		return true;

	default:
		return false;
	}
}

static bool rt5514_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RT5514_RESET:
	case RT5514_PWR_ANA1:
	case RT5514_PWR_ANA2:
	case RT5514_I2S_CTRL1:
	case RT5514_I2S_CTRL2:
	case RT5514_VAD_CTRL6:
	case RT5514_EXT_VAD_CTRL:
	case RT5514_DIG_IO_CTRL:
	case RT5514_PAD_CTRL1:
	case RT5514_DMIC_DATA_CTRL:
	case RT5514_DIG_SOURCE_CTRL:
	case RT5514_SRC_CTRL:
	case RT5514_DOWNFILTER2_CTRL1:
	case RT5514_PLL_SOURCE_CTRL:
	case RT5514_CLK_CTRL1:
	case RT5514_CLK_CTRL2:
	case RT5514_PLL3_CALIB_CTRL1:
	case RT5514_PLL3_CALIB_CTRL5:
	case RT5514_DELAY_BUF_CTRL1:
	case RT5514_DELAY_BUF_CTRL3:
	case RT5514_DOWNFILTER0_CTRL1:
	case RT5514_DOWNFILTER0_CTRL2:
	case RT5514_DOWNFILTER0_CTRL3:
	case RT5514_DOWNFILTER1_CTRL1:
	case RT5514_DOWNFILTER1_CTRL2:
	case RT5514_DOWNFILTER1_CTRL3:
	case RT5514_ANA_CTRL_LDO10:
	case RT5514_ANA_CTRL_LDO18_16:
	case RT5514_ANA_CTRL_ADC12:
	case RT5514_ANA_CTRL_ADC21:
	case RT5514_ANA_CTRL_ADC22:
	case RT5514_ANA_CTRL_ADC23:
	case RT5514_ANA_CTRL_MICBST:
	case RT5514_ANA_CTRL_ADCFED:
	case RT5514_ANA_CTRL_INBUF:
	case RT5514_ANA_CTRL_VREF:
	case RT5514_ANA_CTRL_PLL3:
	case RT5514_ANA_CTRL_PLL1_1:
	case RT5514_ANA_CTRL_PLL1_2:
	case RT5514_DMIC_LP_CTRL:
	case RT5514_MISC_CTRL_DSP:
	case RT5514_DSP_CTRL1:
	case RT5514_DSP_CTRL3:
	case RT5514_DSP_CTRL4:
	case RT5514_VENDOR_ID1:
	case RT5514_VENDOR_ID2:
		return true;

	default:
		return false;
	}
}

/* {-3, 0, +3, +4.5, +7.5, +9.5, +12, +14, +17} dB */
static const DECLARE_TLV_DB_RANGE(bst_tlv,
	0, 2, TLV_DB_SCALE_ITEM(-300, 300, 0),
	3, 3, TLV_DB_SCALE_ITEM(450, 0, 0),
	4, 4, TLV_DB_SCALE_ITEM(750, 0, 0),
	5, 5, TLV_DB_SCALE_ITEM(950, 0, 0),
	6, 6, TLV_DB_SCALE_ITEM(1200, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(1400, 0, 0),
	8, 8, TLV_DB_SCALE_ITEM(1700, 0, 0)
);

static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1725, 75, 0);

static void rt5514_unmute_work(struct work_struct *work)
{
	struct rt5514_priv *rt5514 =
		container_of(work, struct rt5514_priv, unmute_work.work);

	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
}

static int rt5514_dsp_voice_wake_up_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->dsp_enabled;

	return 0;
}

static int rt5514_dsp_stream_flag_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int rt5514_dsp_stream_flag_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	switch (ucontrol->value.integer.value[0]) {
	case RT5514_DSP_STREAM_HOTWORD:
		regmap_write(rt5514->i2c_regmap, RT5514_HOTWORD_FLAG, 0x1);
		regmap_update_bits(rt5514->i2c_regmap, 0x18002e04, 0x1, 0x1);
		regmap_update_bits(rt5514->i2c_regmap, 0x18002e04, 0x1, 0x0);
		break;

	case RT5514_DSP_STREAM_MUSDET:
		regmap_write(rt5514->i2c_regmap, RT5514_MUSDET_FLAG, 0x1);
		regmap_update_bits(rt5514->i2c_regmap, 0x18002e04, 0x1, 0x1);
		regmap_update_bits(rt5514->i2c_regmap, 0x18002e04, 0x1, 0x0);
		break;

	default:
		break;
	}

	return 0;
}

static int rt5514_dsp_frame_flag_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	u8 buf[8];
	unsigned int value_spi, value_i2c;

	mutex_lock(&rt5514->stream_lock);
	if ((rt5514->load_default_sound_model) || (rt5514->need_reload)) {
		ucontrol->value.integer.value[0] = 0;
		mutex_unlock(&rt5514->stream_lock);
		return 0;
	}
	mutex_unlock(&rt5514->stream_lock);

	rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
	rt5514_spi_burst_read(RT5514_BUFFER_MUSIC_WP, (u8 *)&buf, sizeof(buf));
	rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
	value_spi = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
	if ((value_spi & 0xffe00000) != 0x4fe00000) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	msleep(20);

	regmap_read(rt5514->i2c_regmap, RT5514_BUFFER_MUSIC_WP, &value_i2c);
	if ((value_i2c & 0xffe00000) != 0x4fe00000) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	ucontrol->value.integer.value[0] = !!(value_spi - value_i2c);

	return 0;
}

static int rt5514_dsp_test_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->dsp_test;

	return 0;
}

static int rt5514_dsp_test_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	rt5514->dsp_test = ucontrol->value.integer.value[0];

	return 0;
}

static int rt5514_dsp_buf_ch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->dsp_buffer_channel;

	return 0;
}

static int rt5514_dsp_buf_ch_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	rt5514->dsp_buffer_channel = ucontrol->value.integer.value[0];

	return 0;
}

static int rt5514_spi_switch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->spi_switch;

	return 0;
}

static int rt5514_spi_switch_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	rt5514->spi_switch = ucontrol->value.integer.value[0];
	rt5514_set_gpio(RT5514_SPI_SWITCH_GPIO, rt5514->spi_switch);

	return 0;
}

static int rt5514_mic_delay_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = (long)rt5514->mic_delay;

	return 0;
}

static int rt5514_mic_delay_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	struct soc_multi_mixer_control *mc;

	mc = (struct soc_multi_mixer_control *)(kcontrol->private_value);
	if (ucontrol->value.integer.value[0] >= 0 &&
		ucontrol->value.integer.value[0] <= mc->max)
		rt5514->mic_delay =
			(unsigned long)ucontrol->value.integer.value[0];

	return 0;
}

static int rt5514_memcmp(struct rt5514_priv *rt5514, const void *cs,
		const void *ct, size_t count)
{
	struct snd_soc_codec *codec = rt5514->codec;
	const unsigned char *su1, *su2;
	int res = 0;

	for (su1 = cs, su2 = ct; 0 < count; ++su1, ++su2, count--)
		if ((res = *su1 - *su2) != 0) {
			dev_err(codec->dev, "[%02x][%02x]", *su1, *su2);
			break;
		}

	return res;
}

static const struct firmware *rt5514_request_firmware(
	struct rt5514_priv *rt5514, int index)
{
	struct snd_soc_codec *codec = rt5514->codec;

	if (!rt5514->fw[index])
		request_firmware(&rt5514->fw[index], rt5514->fw_name[index],
			codec->dev);

	return rt5514->fw[index];
}

static int rt5514_fw_validate2(struct rt5514_priv *rt5514, int index, int addr)
{
	const struct firmware *fw = NULL;
	struct snd_soc_codec *codec = rt5514->codec;
	int ret = 0;
	u8 *buf;

	fw = rt5514_request_firmware(rt5514, index);
	if (fw) {
		buf = kmalloc(((fw->size/8)+1)*8, GFP_KERNEL);

#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
		rt5514_spi_burst_read(addr, buf, ((fw->size/8)+1)*8);
#else
		dev_err(codec->dev,
			"There is no SPI driver for reading the firmware\n");
#endif
		if (index)
			ret = rt5514_memcmp(rt5514, buf, fw->data, fw->size);
		else
			ret = rt5514_memcmp(rt5514, buf + 8, fw->data + 8,
				fw->size - 8);

		kfree(buf);
		if (ret) {
			dev_err(codec->dev, "FW validate failed fw %d",
				index);
			return ret;
		}
	}

	return 0;
}

static int rt5514_fw_validate(struct rt5514_priv *rt5514, int index, int addr)
{
	struct snd_soc_codec *codec = rt5514->codec;
	int ret = 0;
	u8 *buf;

	switch (index) {
	case 2:
		if (rt5514->hotword_model_buf && rt5514->hotword_model_len) {
			buf = kmalloc(((rt5514->hotword_model_len/8)+1)*8,
					GFP_KERNEL);

#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			rt5514_spi_burst_read(addr, buf,
				((rt5514->hotword_model_len/8)+1)*8);
#else
			dev_err(codec->dev,
				"%d No SPI driver to load fw\n", __LINE__);
#endif
			ret = rt5514_memcmp(rt5514, buf,
					rt5514->hotword_model_buf,
					rt5514->hotword_model_len);
			kfree(buf);
			if (ret) {
				dev_err(codec->dev,
					"FW validate failed fw %d", index);
				return ret;
			}
		} else {
			rt5514_fw_validate2(rt5514, index, addr);
		}

		break;

	case 3:
		if (rt5514->musdet_model_buf && rt5514->musdet_model_len) {
			buf = kmalloc(((rt5514->musdet_model_len/8)+1)*8,
					GFP_KERNEL);

#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			rt5514_spi_burst_read(addr, buf,
				((rt5514->musdet_model_len/8)+1)*8);
#else
			dev_err(codec->dev,
				"%d No SPI driver to load fw\n", __LINE__);
#endif
			ret = rt5514_memcmp(rt5514, buf,
					rt5514->musdet_model_buf,
					rt5514->musdet_model_len);

			kfree(buf);
			if (ret) {
				dev_err(codec->dev,
					"FW validate failed fw %d",
					index);
				return ret;
			}

		} else {
			rt5514_fw_validate2(rt5514, index, addr);
		}

		break;

	default:
		rt5514_fw_validate2(rt5514, index, addr);
		break;
	}

	return 0;
}

static int rt5514_dsp_func_select(struct rt5514_priv *rt5514){
	switch (rt5514->dsp_enabled) {
	case 1:
		regmap_write(rt5514->i2c_regmap,
			RT5514_DSP_WOV_TYPE, RT5514_DSP_WOV_HOTWORD);
		break;

	case 2:
		regmap_write(rt5514->i2c_regmap,
			RT5514_DSP_WOV_TYPE, RT5514_DSP_WOV_MUSDET);
		break;

	case 3:
		regmap_write(rt5514->i2c_regmap,
			RT5514_DSP_WOV_TYPE, RT5514_DSP_WOV_BOTH);
		break;

	default:
		regmap_write(rt5514->i2c_regmap,
			RT5514_DSP_WOV_TYPE, RT5514_DSP_WOV_NON);
		break;
	}

	return 0;
}

static int rt5514_dsp_status_check(struct rt5514_priv *rt5514)
{
	struct snd_soc_codec *codec = rt5514->codec;
	unsigned int val = 0, i;

	regmap_read(rt5514->regmap, RT5514_VENDOR_ID2, &val);
	if (val != RT5514_DEVICE_ID) {
		dev_err(codec->dev,
			"Device with ID register %x is not rt5514\n", val);
		val = -ENODEV;
		goto reset;
	}

	for (i = 0; i < 10; i++) {
		regmap_read(rt5514->i2c_regmap, 0x18001014, &val);
		if (val == 0)
			break;
		else
			usleep_range(10000, 15000);
	}

reset:
	if (val) {
		dev_err(codec->dev, "DSP run failure, reset DSP\n");

		if (rt5514->gpiod_reset) {
			gpiod_set_value(rt5514->gpiod_reset, 0);
			usleep_range(1000, 2000);
			gpiod_set_value(rt5514->gpiod_reset, 1);
		} else {
			regmap_multi_reg_write(rt5514->i2c_regmap,
				rt5514_i2c_patch, rt5514->i2c_patch_size);
		}
		regcache_mark_dirty(rt5514->regmap);
		regcache_sync(rt5514->regmap);
	}

	return val;
}

static int rt5514_dsp_enable(struct rt5514_priv *rt5514,
		bool is_adc, bool is_watchdog)
{
	struct snd_soc_codec *codec = rt5514->codec;
	const struct firmware *fw = NULL;
	unsigned int val, i = 0;
	struct _dsp_fw_ver_st dsp_fw_ver;

	if (is_watchdog)
		goto watchdog;

	if (is_adc) {
		if (rt5514->dsp_enabled) {
			if (rt5514->dsp_adc_enabled) {
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_WOV_SENSOR);
			} else {
				if (rt5514->dsp_enabled < 5)
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_WOV);
				else
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_SUSPEND);
			}

			rt5514_filter_power_reset(rt5514);

			return 0;
		}
	} else {
		if (rt5514->dsp_adc_enabled ||
			(rt5514->dsp_enabled &&
				rt5514->dsp_enabled_last > 0)) {
			rt5514_dsp_func_select(rt5514);

			if (rt5514->dsp_enabled < 5) {
				if (rt5514->dsp_enabled_last != 5)
					return 0;

				if (rt5514->dsp_adc_enabled)
					regmap_write(
						rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_WOV_SENSOR);
				else
					regmap_write(
						rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_WOV);
				rt5514_filter_power_reset(rt5514);
			} else {
				if (rt5514->dsp_adc_enabled) {
					dev_warn(codec->dev,
						"DSP ADC is enabled\n");
					return 0;
				}

				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_SUSPEND);
				rt5514_filter_power_reset(rt5514);
			}

			return 0;
		}
	}

watchdog:

	dev_dbg(codec->dev, "dsp_enabled = %d, dsp_adc_enabled = %d\n",
		rt5514->dsp_enabled, rt5514->dsp_adc_enabled);

	if (rt5514->dsp_enabled || rt5514->dsp_adc_enabled) {
		regmap_multi_reg_write(rt5514->i2c_regmap,
			rt5514_i2c_patch, rt5514->i2c_patch_size);
		rt5514_enable_dsp_prepare(rt5514);
		rt5514_dsp_func_select(rt5514);

		fw = rt5514_request_firmware(rt5514, 0);
		if (fw) {
			memcpy(&rt5514->sound_model_addr, fw->data,
				sizeof(unsigned int) * 2);
			if (rt5514->sound_model_addr[0])
				rt5514->fw_addr[2] =
					rt5514->sound_model_addr[0];

			if (rt5514->v_p) {
				memcpy(&dsp_fw_ver, fw->data + 0x100,
					sizeof(struct _dsp_fw_ver_st));
				dev_info(codec->dev,
				"DSP Firmware Version: %d.%d.%d.%d\n",
				dsp_fw_ver.chip_id, dsp_fw_ver.feature_id,
				dsp_fw_ver.version, dsp_fw_ver.sub_version);
			}
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			rt5514_spi_burst_write(rt5514->fw_addr[0], fw->data,
				fw->size);
#else
			dev_err(codec->dev, "There is no SPI driver for"
				" loading the firmware\n");
#endif
		}

		fw = rt5514_request_firmware(rt5514, 1);
		if (fw) {
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			rt5514_spi_burst_write(rt5514->fw_addr[1], fw->data,
				fw->size);
#else
			dev_err(codec->dev, "There is no SPI driver for"
				" loading the firmware\n");
#endif
		}

		if (rt5514->hotword_model_buf && rt5514->hotword_model_len &&
			!rt5514->load_default_sound_model) {
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			int ret;

			ret = rt5514_spi_burst_write(rt5514->fw_addr[2],
				rt5514->hotword_model_buf,
				rt5514->hotword_model_len);
			if (ret) {
				dev_err(codec->dev,
					"Model load failed %d\n", ret);
				return ret;
			}

			if (rt5514->sound_model_addr[0]) {
				rt5514->fw_addr[3] = rt5514->fw_addr[2] +
					((rt5514->hotword_model_len/8)+1)*8;
				rt5514_spi_burst_write(rt5514->fw_addr[0],
					(const u8 *)&rt5514->fw_addr[2], 8);
			}
#else
			dev_err(codec->dev,
				"No SPI driver for loading firmware\n");
#endif
		} else {
			fw = rt5514_request_firmware(rt5514, 2);
			if (fw) {
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
				rt5514_spi_burst_write(rt5514->fw_addr[2],
					fw->data,
					fw->size);
#else
				dev_err(codec->dev,
					"No SPI driver to load fw\n");
#endif
				if (rt5514->sound_model_addr[0]) {
					rt5514->fw_addr[3] =
						rt5514->fw_addr[2] +
						((fw->size/8)+1)*8;
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
					rt5514_spi_burst_write(
						rt5514->fw_addr[0],
						(const u8 *)&rt5514->fw_addr[2],
						8);
#else
					dev_err(codec->dev,
						"No SPI driver to load fw\n");
#endif
				}
			}
		}

		if (rt5514->musdet_model_buf && rt5514->musdet_model_len &&
			!rt5514->load_default_sound_model) {
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
			int ret;

			ret = rt5514_spi_burst_write(rt5514->fw_addr[3],
				rt5514->musdet_model_buf,
				rt5514->musdet_model_len);
			if (ret) {
				dev_err(codec->dev,
					"Model load failed %d\n", ret);
				return ret;
			}
#else
			dev_err(codec->dev,
				"No SPI driver for loading firmware\n");
#endif
		} else {
			fw = rt5514_request_firmware(rt5514, 3);
			if (fw) {
#if IS_ENABLED(CONFIG_SND_SOC_RT5514_SPI)
				rt5514_spi_burst_write(rt5514->fw_addr[3],
					fw->data,
					fw->size);
#else
				dev_err(codec->dev,
					"No SPI driver to load fw\n");
#endif
			}
		}

		if (rt5514->dsp_test) {
			for (i = 0; i < 4; i++) {
				if (rt5514_fw_validate(rt5514, i,
					rt5514->fw_addr[i])) {
					rt5514->dsp_enabled = 0;
					regmap_multi_reg_write(
						rt5514->i2c_regmap,
						rt5514_i2c_patch,
						rt5514->i2c_patch_size);
					regcache_mark_dirty(rt5514->regmap);
					regcache_sync(rt5514->regmap);
					return 0;
				}
			}
		}

		if (rt5514->v_p) {
			/* dsp clk=mux_out (40M) */
			regmap_write(rt5514->i2c_regmap, 0x18002f08,
				0x0000000b);
		}

		/* DSP run */
		regmap_write(rt5514->i2c_regmap, 0x18002f00,
			0x00055148);

		usleep_range(10000, 10005);

		if (is_watchdog && rt5514->is_streaming) {

			if (rt5514->dsp_adc_enabled) {
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_WOV_I2S_SENSOR);
			} else {
				if (rt5514->dsp_enabled < 5)
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_WOV_I2S);
				else
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_I2S);
			}
			regmap_read(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
				&val);
			regmap_write(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
				val);
			regmap_read(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
				&val);
			regmap_write(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
				val);
			regmap_read(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
				&val);
			regmap_write(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
				val);
			regmap_read(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
				&val);
			regmap_write(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
				val);
			regmap_read(rt5514->regmap, RT5514_DOWNFILTER2_CTRL1,
				&val);
			regmap_write(rt5514->regmap, RT5514_DOWNFILTER2_CTRL1,
				val);

		} else {
			if (rt5514->dsp_adc_enabled) {
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_WOV_SENSOR);
			} else {
				if (rt5514->dsp_enabled < 5)
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_WOV);
				else
					regmap_write(rt5514->i2c_regmap,
						RT5514_DSP_FUNC,
						RT5514_DSP_FUNC_SUSPEND);
			}
		}
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
		codec_detect_status_notifier(WDSP_STAT_UP);
#endif

		rt5514_filter_power_reset(rt5514);
	} else {
		if (rt5514->gpiod_reset) {
			gpiod_set_value(rt5514->gpiod_reset, 0);
			usleep_range(1000, 2000);
			gpiod_set_value(rt5514->gpiod_reset, 1);
		} else {
			regmap_multi_reg_write(rt5514->i2c_regmap,
				rt5514_i2c_patch, rt5514->i2c_patch_size);
		}
		regcache_mark_dirty(rt5514->regmap);
		regcache_sync(rt5514->regmap);
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
		codec_detect_status_notifier(WDSP_STAT_DOWN);
#endif
	}

	return 0;
}

void rt5514_watchdog_handler(void)
{
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
	codec_detect_status_notifier(WDSP_STAT_CRASH);
#endif
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);

	if (g_rt5514->gpiod_reset) {
		gpiod_set_value(g_rt5514->gpiod_reset, 0);
		usleep_range(1000, 2000);
		gpiod_set_value(g_rt5514->gpiod_reset, 1);
	} else {
		regmap_multi_reg_write(rt5514_g_i2c_regmap,
			rt5514_i2c_patch, g_rt5514->i2c_patch_size);
	}
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
	rt5514_dsp_enable(g_rt5514, false, true);
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);

	usleep_range(125000, 125100);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(g_rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
}

static void rt5514_reload_firmware(struct rt5514_priv *rt5514)
{
	if (!rt5514 || (!rt5514->need_reload) || (rt5514->is_streaming))
		return;

	rt5514->dsp_enabled_last = rt5514->dsp_enabled;
	rt5514->dsp_enabled = 0;
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
	rt5514_dsp_enable(rt5514, false, false);
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);

	rt5514->dsp_enabled = rt5514->dsp_enabled_last;
	rt5514->dsp_enabled_last = 0;
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
	rt5514->load_default_sound_model = false;
	rt5514_dsp_enable(rt5514, false, false);
	if (rt5514_dsp_status_check(rt5514)) {
		rt5514->load_default_sound_model = true;
		rt5514_dsp_enable(rt5514, false, true);
	}
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);
	rt5514->need_reload = false;
}

static int rt5514_dsp_voice_wake_up_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	rt5514->dsp_req = ucontrol->value.integer.value[0];

	if (ucontrol->value.integer.value[0] == rt5514->dsp_enabled)
		return 0;

	mutex_lock(&rt5514->stream_lock);
	if (!rt5514->is_streaming) {
		rt5514->dsp_enabled_last = rt5514->dsp_enabled;
		rt5514->dsp_enabled = ucontrol->value.integer.value[0];

		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
		rt5514_dsp_enable(rt5514, false, false);
		if (rt5514_dsp_status_check(rt5514))
			rt5514_dsp_enable(rt5514, false, true);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);
	} else {
		rt5514->dsp_enabled = ucontrol->value.integer.value[0];

		dev_warn(component->dev, "%s: Unsupport : %d %d\n",
			__func__, rt5514->dsp_enabled,
			rt5514->dsp_adc_enabled);
	}
	mutex_unlock(&rt5514->stream_lock);

	return 0;
}

static int rt5514_dsp_adc_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	rt5514->adc_req = ucontrol->value.integer.value[0];

	if (ucontrol->value.integer.value[0] == rt5514->dsp_adc_enabled)
		return 0;

	mutex_lock(&rt5514->stream_lock);
	if (!rt5514->is_streaming) {
		rt5514->dsp_adc_enabled = ucontrol->value.integer.value[0];
		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
		rt5514_dsp_enable(rt5514, true, false);
		if (rt5514_dsp_status_check(rt5514))
			rt5514_dsp_enable(rt5514, false, true);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);
	} else {
		rt5514->dsp_adc_enabled = ucontrol->value.integer.value[0];

		dev_warn(component->dev, "%s: Unsupport : %d %d\n",
			__func__, rt5514->dsp_enabled,
			rt5514->dsp_adc_enabled);
	}
	mutex_unlock(&rt5514->stream_lock);

	return 0;
}

static int rt5514_dsp_adc_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->dsp_adc_enabled;

	return 0;
}

static int rt5514_dsp_func_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	regmap_write(rt5514->i2c_regmap, RT5514_DSP_FUNC,
		ucontrol->value.integer.value[0]);
	rt5514_filter_power_reset(rt5514);

	return 0;
}

static int rt5514_dsp_mod_enable_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	switch (ucontrol->value.integer.value[0]) {
	case RT5514_DSP_CHRE:
		dev_info(component->dev, "chre enable\n");
		rt5514_spi_request_switch(SPI_SWITCH_MASK_NO_CHRE, 0);
		break;
	default:
		break;
	}

	return 0;
}

static int rt5514_dsp_mod_disable_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	switch (ucontrol->value.integer.value[0]) {
	case RT5514_DSP_CHRE:
		dev_info(component->dev, "chre disable\n");
		rt5514_spi_request_switch(SPI_SWITCH_MASK_NO_CHRE, 1);
		break;
	default:
		break;
	}

	return 0;
}

static int rt5514_dsp_func_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int rt5514_hw_ver_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5514->v_p;

	return 0;
}

static int rt5514_hw_reset_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	if (rt5514->gpiod_reset) {
		dev_err(component->dev, "Receive Trigger Reset\n");
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
		codec_detect_status_notifier(WDSP_STAT_CRASH);
#endif
		gpiod_set_value(rt5514->gpiod_reset, 0);
		usleep_range(1000, 2000);
		gpiod_set_value(rt5514->gpiod_reset, 1);

		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);

		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
		rt5514_dsp_enable(rt5514, false, true);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);

		usleep_range(125000, 125100);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
			RT5514_AD_AD_MUTE, 0x0);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
			RT5514_AD_AD_MUTE, 0x0);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
			RT5514_AD_AD_MUTE, 0x0);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
			RT5514_AD_AD_MUTE, 0x0);
		rt5514->need_reload = false;
	}

	return 0;
}

static int rt5514_hw_reset_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = !!rt5514->gpiod_reset;

	return 0;
}

static int rt5514_hotword_model_put(struct snd_kcontrol *kcontrol,
		const unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	struct snd_soc_codec *codec = rt5514->codec;
	int ret = 0;

	if (rt5514->hotword_model_buf || rt5514->hotword_model_len < size) {
		if (rt5514->hotword_model_buf)
			devm_kfree(codec->dev, rt5514->hotword_model_buf);
		rt5514->hotword_model_buf = devm_kmalloc(codec->dev, size,
			GFP_KERNEL);
		if (!rt5514->hotword_model_buf) {
			ret = -ENOMEM;
			goto done;
		}
	}

	if (copy_from_user(rt5514->hotword_model_buf, bytes, size))
		ret = -EFAULT;
done:
	rt5514->hotword_model_len = (ret ? 0 : size);

	/* reload firmware */
	mutex_lock(&rt5514->stream_lock);
	rt5514->need_reload = true;
	rt5514_reload_firmware(rt5514);
	mutex_unlock(&rt5514->stream_lock);

	return ret;
}

static int rt5514_musdet_model_put(struct snd_kcontrol *kcontrol,
		const unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	struct snd_soc_codec *codec = rt5514->codec;
	int ret = 0;

	if (rt5514->musdet_model_buf || rt5514->musdet_model_len < size) {
		if (rt5514->musdet_model_buf)
			devm_kfree(codec->dev, rt5514->musdet_model_buf);
		rt5514->musdet_model_buf = devm_kmalloc(codec->dev, size,
			GFP_KERNEL);
		if (!rt5514->musdet_model_buf) {
			ret = -ENOMEM;
			goto done;
		}
	}

	if (copy_from_user(rt5514->musdet_model_buf, bytes, size))
		ret = -EFAULT;
done:
	rt5514->musdet_model_len = (ret ? 0 : size);

	/* reload firmware */
	mutex_lock(&rt5514->stream_lock);
	rt5514->need_reload = true;
	rt5514_reload_firmware(rt5514);
	mutex_unlock(&rt5514->stream_lock);

	return ret;
}

static int rt5514_ambient_payload_put(struct snd_kcontrol *kcontrol,
		const unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	int ret = 0;
	char payload[sizeof(struct _payload_st)];
	unsigned int payload_addr;

	if (copy_from_user(payload, bytes, size))
		return -EFAULT;

	/* AmbientHotwordType */
	regmap_write(rt5514->i2c_regmap, 0x18002fd0, payload[0]);
	regmap_write(rt5514->i2c_regmap, 0x18001014, 2);
	regmap_read(rt5514->i2c_regmap, 0x18002fd4, &payload_addr);
	regmap_read(rt5514->i2c_regmap, 0x18002fd8, &rt5514->payload.size);
	regmap_read(rt5514->i2c_regmap, 0x18002fdc, &rt5514->payload.status);

	if ((payload_addr & 0xffe00000) == 0x4fe00000) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
		rt5514_spi_burst_read(payload_addr, (u8 *)&rt5514->payload.data,
			AMBIENT_COMMON_MAX_PAYLOAD_BUFFER_SIZE);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
	}

	return ret;
}

static int rt5514_ambient_payload_get(struct snd_kcontrol *kcontrol,
		unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (size != sizeof(struct _payload_st))
		return -EINVAL;

	if (copy_to_user(bytes, &rt5514->payload, sizeof(struct _payload_st))) {
		dev_warn(component->dev, "%s(), copy_to_user fail\n", __func__);
		ret = -EFAULT;
	}

	return ret;
}

static int rt5514_ambient_process_payload_get(struct snd_kcontrol *kcontrol,
		unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	struct snd_soc_codec *codec = rt5514->codec;
	int ret = 0;
	unsigned int payload_addr;

	if (size != sizeof(struct _payload_st))
		return -EINVAL;

	regmap_read(rt5514->i2c_regmap, 0x18002fe0, &payload_addr);
	regmap_read(rt5514->i2c_regmap, 0x18002fe4, &rt5514->payload.size);

	if ((payload_addr & 0xffe00000) == 0x4fe00000) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
		rt5514_spi_burst_read(payload_addr, (u8 *)&rt5514->payload.data,
			AMBIENT_COMMON_MAX_PAYLOAD_BUFFER_SIZE);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
	}

	if (copy_to_user(bytes, &rt5514->payload, sizeof(struct _payload_st))) {
		dev_warn(codec->dev, "%s(), copy_to_user fail\n", __func__);
		ret = -EFAULT;
	}

	return ret;
}

static int rt5514_mem_test_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	u8 *buf1, *buf2;
	int ret;

	if (!rt5514->v_p || !rt5514->dsp_test) {
		ucontrol->value.integer.value[0] = 2;
		return 0;
	}

	regmap_multi_reg_write(rt5514->i2c_regmap,
		rt5514_i2c_patch, rt5514->i2c_patch_size);
	rt5514_enable_dsp_prepare(rt5514);

	buf1 = kmalloc(0xb8000, GFP_KERNEL);
	if (!buf1) {
		ucontrol->value.integer.value[0] = 3;
		return 0;
	}

	buf2 = kmalloc(0xb8000, GFP_KERNEL);
	if (!buf2) {
		ucontrol->value.integer.value[0] = 3;
		kfree(buf1);
		return 0;
	}

	dev_info(component->dev, "Test 1 IMEM 0\n");
	rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
	memset(buf1, 0, 0x18000);
	rt5514_spi_burst_write(0x4ff00000, buf1, 0x18000);
	rt5514_spi_burst_read(0x4ff00000, buf2, 0x18000);
	ret = rt5514_memcmp(rt5514, buf1, buf2, 0x18000);
	if (ret)
		goto failed;

	dev_info(component->dev, "Test 2 IMEM 1\n");
	memset(buf1, 0xff, 0x18000);
	rt5514_spi_burst_write(0x4ff00000, buf1, 0x18000);
	rt5514_spi_burst_read(0x4ff00000, buf2, 0x18000);
	ret = rt5514_memcmp(rt5514, buf1, buf2, 0x18000);
	if (ret)
		goto failed;

	dev_info(component->dev, "Test 3 DMEM 0\n");
	memset(buf1, 0, 0xb8000);
	rt5514_spi_burst_write(0x4fe00000, buf1, 0xb8000);
	rt5514_spi_burst_read(0x4fe00000, buf2, 0xb8000);
	ret = rt5514_memcmp(rt5514, buf1, buf2, 0xb8000);
	if (ret)
		goto failed;

	dev_info(component->dev, "Test 4 DMEM 1\n");
	memset(buf1, 0xff, 0xb8000);
	rt5514_spi_burst_write(0x4fe00000, buf1, 0xb8000);
	rt5514_spi_burst_read(0x4fe00000, buf2, 0xb8000);
	ret = rt5514_memcmp(rt5514, buf1, buf2, 0xb8000);

	dev_info(component->dev, "Test done\n");

failed:
	regmap_multi_reg_write(rt5514->i2c_regmap,
		rt5514_i2c_patch, rt5514->i2c_patch_size);
	rt5514_dsp_enable(rt5514, false, true);
	rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
	ucontrol->value.integer.value[0] = !!ret;

	kfree(buf1);
	kfree(buf2);

	return 0;
}

static int rt5514_ambient_hotword_version_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	unsigned int version;

	regmap_write(rt5514->i2c_regmap, 0x18002fd0, 0x1 << 28);
	regmap_write(rt5514->i2c_regmap, 0x18001014, 2);

	msleep(20);

	regmap_read(rt5514->i2c_regmap, 0x18002fd4, &version);
	ucontrol->value.integer.value[0] = version;

	return 0;
}

static int rt5514_firmware_version_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	const struct firmware *fw = NULL;
	struct _dsp_fw_ver_st dsp_fw_ver;
	struct _dsp_mem_st dsp_mem;

	if (!rt5514->v_p)
		return 0;

	fw = rt5514_request_firmware(rt5514, 0);
	if (fw) {
		memcpy(&dsp_fw_ver, fw->data + 0x100,
			sizeof(struct _dsp_fw_ver_st));
		dev_info(component->dev, "DSP Firmware Version: %d.%d.%d.%d\n",
			dsp_fw_ver.chip_id, dsp_fw_ver.feature_id,
			dsp_fw_ver.version, dsp_fw_ver.sub_version);
	}

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
		rt5514_spi_burst_read(rt5514->fw_addr[0] + 0x128,
			(u8 *)&dsp_mem, sizeof(struct _dsp_mem_st));
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
		dev_info(component->dev, "IRAM: %d DRAM: %d\n",
			dsp_mem.iram, dsp_mem.dram);
	}

	return 0;
}

static int rt5514_hotword_dsp_identifier_get(struct snd_kcontrol *kcontrol,
		unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5514_priv *rt5514 = snd_soc_component_get_drvdata(component);
	int ret = 0;
	unsigned int identifier_addr;
	char uuid[DSP_IDENTIFIER_SIZE];

	if (size != DSP_IDENTIFIER_SIZE)
		return -EINVAL;

	regmap_write(rt5514->i2c_regmap, 0x18002fd0, 0x2 << 28);
	regmap_write(rt5514->i2c_regmap, 0x18001014, 2);

	msleep(20);

	regmap_read(rt5514->i2c_regmap, 0x18002fd4, &identifier_addr);

	if ((identifier_addr & 0xffe00000) == 0x4fe00000) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 1);
		rt5514_spi_burst_read(identifier_addr, (u8 *)&uuid,
			DSP_IDENTIFIER_SIZE);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_CMD, 0);
	}

	if (copy_to_user(bytes, &uuid, DSP_IDENTIFIER_SIZE)) {
		dev_warn(component->dev, "%s(), copy_to_user fail\n", __func__);
		ret = -EFAULT;
	}

	return ret;
}

static const char * const dmic_divider_rate_txt[] = {
	"1.024K", "1.536K", "2.048K", "3.072K",
};

static SOC_ENUM_SINGLE_EXT_DECL(dmic_divider_rate, dmic_divider_rate_txt);

static const char * const rt5514_mem_test_txt[] = {
	"PASS", "FAIL", "NOT_SUPPORT", "OUT_OF_MEMORY",
};
static SOC_ENUM_SINGLE_EXT_DECL(rt5514_mem_test, rt5514_mem_test_txt);


static const struct snd_kcontrol_new rt5514_snd_controls[] = {
	SOC_DOUBLE_TLV("MIC Boost Volume", RT5514_ANA_CTRL_MICBST,
		RT5514_SEL_BSTL_SFT, RT5514_SEL_BSTR_SFT, 8, 0, bst_tlv),
	SOC_DOUBLE_R_TLV("ADC1 Capture Volume", RT5514_DOWNFILTER0_CTRL1,
		RT5514_DOWNFILTER0_CTRL2, RT5514_AD_GAIN_SFT, 63, 0,
		adc_vol_tlv),
	SOC_DOUBLE_R_TLV("ADC2 Capture Volume", RT5514_DOWNFILTER1_CTRL1,
		RT5514_DOWNFILTER1_CTRL2, RT5514_AD_GAIN_SFT, 63, 0,
		adc_vol_tlv),
	/*
	* Control "DSP Voice Wake Up"
	* 0 => Disable DSP
	* 1 => WOV Hotword
	* 2 => WOV Musdet
	* 3 => WOV Hotword & Musdet
	* 4 => WOV Buffer Only
	* 5 => Suspend DSP
	*/
	SOC_SINGLE_EXT("DSP Voice Wake Up", SND_SOC_NOPM, 0, 5, 0,
		rt5514_dsp_voice_wake_up_get, rt5514_dsp_voice_wake_up_put),

	SOC_SINGLE_EXT("DSP Model Enable", SND_SOC_NOPM, 0, 5, 0,
		rt5514_dsp_func_get, rt5514_dsp_mod_enable_put),
	SOC_SINGLE_EXT("DSP Model Disable", SND_SOC_NOPM, 0, 5, 0,
		rt5514_dsp_func_get, rt5514_dsp_mod_disable_put),
	SOC_SINGLE_EXT("DSP ADC", SND_SOC_NOPM, 0, 1, 0,
		rt5514_dsp_adc_get, rt5514_dsp_adc_put),
	SOC_SINGLE_EXT("DSP FUNC", SND_SOC_NOPM, 0, 5, 0,
		rt5514_dsp_func_get, rt5514_dsp_func_put),
	SND_SOC_BYTES_TLV("Hotword Model", 0xffff, NULL,
		rt5514_hotword_model_put),
	SND_SOC_BYTES_TLV("Musdet Model", 0xffff, NULL,
		rt5514_musdet_model_put),
	SOC_SINGLE_EXT("DSP Stream Flag", SND_SOC_NOPM, 0, 2, 0,
		rt5514_dsp_stream_flag_get, rt5514_dsp_stream_flag_put),
	SOC_SINGLE_EXT("DSP Frame Flag", SND_SOC_NOPM, 0, 1, 0,
		rt5514_dsp_frame_flag_get, NULL),
	SOC_SINGLE_EXT("DSP Test", SND_SOC_NOPM, 0, 1, 0,
		rt5514_dsp_test_get, rt5514_dsp_test_put),
	SOC_ENUM_EXT("Mem Test", rt5514_mem_test,
		rt5514_mem_test_get, NULL),
	/* 0 => Stereo ; 1 => Mono */
	SOC_SINGLE_EXT("DSP Buffer Channel", SND_SOC_NOPM, 0, 1, 0,
		rt5514_dsp_buf_ch_get, rt5514_dsp_buf_ch_put),
	SOC_SINGLE_EXT("HW Version", SND_SOC_NOPM, 0, 1, 0,
		rt5514_hw_ver_get, NULL),
	SOC_SINGLE_EXT("HW Reset", SND_SOC_NOPM, 0, 1, 0,
		rt5514_hw_reset_get, rt5514_hw_reset_set),
	SND_SOC_BYTES_TLV("Ambient Payload", sizeof(struct _payload_st),
		rt5514_ambient_payload_get, rt5514_ambient_payload_put),
	SND_SOC_BYTES_TLV("Ambient Process Payload", sizeof(struct _payload_st),
		rt5514_ambient_process_payload_get, NULL),
	SOC_SINGLE_EXT("Ambient Hotword Version", SND_SOC_NOPM,
		0, 0x7fffffff, 0,
		rt5514_ambient_hotword_version_get, NULL),
	SOC_SINGLE_EXT("SPI Switch", SND_SOC_NOPM, 0, 1, 0,
		rt5514_spi_switch_get, rt5514_spi_switch_put),
	SOC_SINGLE_EXT("Mic Delay ms", SND_SOC_NOPM, 0, 1000, 0,
		rt5514_mic_delay_get, rt5514_mic_delay_put),
	SOC_SINGLE_EXT("DSP Firmware Version", SND_SOC_NOPM, 0, 0x7fffffff,
		0, rt5514_firmware_version_get, NULL),
	SND_SOC_BYTES_TLV("DSP Identifier", DSP_IDENTIFIER_SIZE,
		rt5514_hotword_dsp_identifier_get, NULL),
};

/* ADC Mixer*/
static const struct snd_kcontrol_new rt5514_sto1_adc_l_mix[] = {
	SOC_DAPM_SINGLE("DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("BargeIn DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("ADC Switch", RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new rt5514_sto1_adc_r_mix[] = {
	SOC_DAPM_SINGLE("DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("BargeIn DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("ADC Switch", RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new rt5514_sto2_adc_l_mix[] = {
	SOC_DAPM_SINGLE("DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("BargeIn DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("ADC Switch", RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MIX_BIT, 1, 1),
};

static const struct snd_kcontrol_new rt5514_sto2_adc_r_mix[] = {
	SOC_DAPM_SINGLE("DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("BargeIn DMIC Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("ADC Switch", RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MIX_BIT, 1, 1),
};

/* DMIC Source */
static const char * const rt5514_dmic_src[] = {
	"DMIC1", "DMIC2"
};

static const SOC_ENUM_SINGLE_DECL(
	rt5514_stereo1_dmic_enum, RT5514_DIG_SOURCE_CTRL,
	RT5514_AD0_DMIC_INPUT_SEL_SFT, rt5514_dmic_src);

static const struct snd_kcontrol_new rt5514_sto1_dmic_mux =
	SOC_DAPM_ENUM("Stereo1 DMIC Source", rt5514_stereo1_dmic_enum);

static const SOC_ENUM_SINGLE_DECL(
	rt5514_stereo2_dmic_enum, RT5514_DIG_SOURCE_CTRL,
	RT5514_AD1_DMIC_INPUT_SEL_SFT, rt5514_dmic_src);

static const struct snd_kcontrol_new rt5514_sto2_dmic_mux =
	SOC_DAPM_ENUM("Stereo2 DMIC Source", rt5514_stereo2_dmic_enum);

/**
 * rt5514_calc_dmic_clk - Calculate the frequency divider parameter of dmic.
 *
 * @rate: base clock rate.
 *
 * Choose divider parameter that gives the highest possible DMIC frequency in
 * 1MHz - 3MHz range.
 */
static int rt5514_calc_dmic_clk(struct snd_soc_codec *codec, int rate)
{
	int div[] = {2, 3, 4, 8, 12, 16, 24, 32};
	int i;

	if (rate < 1000000 * div[0]) {
		pr_warn("Base clock rate %d is too low\n", rate);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(div); i++) {
		/* find divider that gives DMIC frequency below 3.072MHz */
		if (3072000 * div[i] >= rate)
			return i;
	}

	dev_warn(codec->dev, "Base clock rate %d is too high\n", rate);
	return -EINVAL;
}

static int rt5514_set_dmic_clk(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	int idx;

	idx = rt5514_calc_dmic_clk(codec, rt5514->sysclk);
	if (idx < 0)
		dev_err(codec->dev, "Failed to set DMIC clock\n");
	else
		regmap_update_bits(rt5514->regmap, RT5514_CLK_CTRL1,
			RT5514_CLK_DMIC_OUT_SEL_MASK,
			idx << RT5514_CLK_DMIC_OUT_SEL_SFT);

	if (rt5514->pdata.dmic_init_delay)
		msleep(rt5514->pdata.dmic_init_delay);

	return idx;
}

static int rt5514_is_sys_clk_from_pll1(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	if (rt5514->sysclk_src == RT5514_SCLK_S_PLL && !rt5514->v_p)
		return 1;
	else
		return 0;
}

static int rt5514_is_sys_clk_from_pll2(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	if (rt5514->sysclk_src == RT5514_SCLK_S_PLL && rt5514->v_p)
		return 1;
	else
		return 0;
}

static int rt5514_i2s_use_asrc(struct snd_soc_dapm_widget *source,
	struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	return (rt5514->sysclk > rt5514->lrck * 384);
}

static int rt5514_is_not_dsp_enabled(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	return !(rt5514->dsp_enabled | rt5514->dsp_adc_enabled);
}

static int rt5514_dmic_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	if (event & SND_SOC_DAPM_PRE_PMU && rt5514->mic_delay > 0) {
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
			RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
		/* un-mute all dmic path after power up */
		cancel_delayed_work_sync(&rt5514->unmute_work);
		schedule_delayed_work(&rt5514->unmute_work,
			msecs_to_jiffies(rt5514->mic_delay));
	}

	return 0;
}

static const struct snd_soc_dapm_widget rt5514_dapm_widgets[] = {
	/* Input Lines */
	SND_SOC_DAPM_INPUT("DMIC1L"),
	SND_SOC_DAPM_INPUT("DMIC1R"),
	SND_SOC_DAPM_INPUT("DMIC2L"),
	SND_SOC_DAPM_INPUT("DMIC2R"),

	SND_SOC_DAPM_INPUT("AMICL"),
	SND_SOC_DAPM_INPUT("AMICR"),

	SND_SOC_DAPM_PGA("DMIC1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("DMIC2", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("DMIC CLK", SND_SOC_NOPM, 0, 0,
		rt5514_set_dmic_clk, SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_SUPPLY("ADC CLK", RT5514_CLK_CTRL1,
		RT5514_CLK_AD_ANA1_EN_BIT, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("LDO18 IN", RT5514_PWR_ANA1,
		RT5514_POW_LDO18_IN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("LDO18 ADC", RT5514_PWR_ANA1,
		RT5514_POW_LDO18_ADC_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("LDO21", RT5514_PWR_ANA1, RT5514_POW_LDO21_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("BG LDO18 IN", RT5514_PWR_ANA1,
		RT5514_POW_BG_LDO18_IN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BG LDO21", RT5514_PWR_ANA1,
		RT5514_POW_BG_LDO21_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BG MBIAS", RT5514_PWR_ANA2,
		RT5514_POW_BG_MBIAS_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MBIAS", RT5514_PWR_ANA2, RT5514_POW_MBIAS_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("VREF2", RT5514_PWR_ANA2, RT5514_POW_VREF2_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("VREF1", RT5514_PWR_ANA2, RT5514_POW_VREF1_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC Power", SND_SOC_NOPM, 0, 0, NULL, 0),


	SND_SOC_DAPM_SUPPLY("LDO16L", RT5514_PWR_ANA2, RT5514_POWL_LDO16_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC1L", RT5514_PWR_ANA2, RT5514_POW_ADC1_L_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTL2", RT5514_PWR_ANA2, RT5514_POW2_BSTL_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTL", RT5514_PWR_ANA2, RT5514_POW_BSTL_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCFEDL", RT5514_PWR_ANA2, RT5514_POW_ADCFEDL_BIT,
		0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCL Power", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("LDO16R", RT5514_PWR_ANA2, RT5514_POWR_LDO16_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC1R", RT5514_PWR_ANA2, RT5514_POW_ADC1_R_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTR2", RT5514_PWR_ANA2, RT5514_POW2_BSTR_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTR", RT5514_PWR_ANA2, RT5514_POW_BSTR_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCFEDR", RT5514_PWR_ANA2, RT5514_POW_ADCFEDR_BIT,
		0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCR Power", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLL1 LDO ENABLE", RT5514_ANA_CTRL_PLL1_2,
		RT5514_EN_LDO_PLL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL1 LDO", RT5514_PWR_ANA2,
		RT5514_POW_PLL1_LDO_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL1", RT5514_PWR_ANA2, RT5514_POW_PLL1_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL2 LDO ENABLE", RT5514_ANA_CTRL_PLL2_2,
		RT5514_EN_LDO_PLL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL2 LDO", RT5514_PWR_ANA2,
		RT5514_POW_PLL2_LDO_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL2", RT5514_PWR_ANA2, RT5514_POW_PLL2_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ASRC AD1", 1, RT5514_CLK_CTRL2,
		RT5514_CLK_AD0_ASRC_EN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("ASRC AD2", 1, RT5514_CLK_CTRL2,
		RT5514_CLK_AD1_ASRC_EN_BIT, 0, NULL, 0),

	/* ADC Mux */
	SND_SOC_DAPM_MUX("Stereo1 DMIC Mux", SND_SOC_NOPM, 0, 0,
				&rt5514_sto1_dmic_mux),
	SND_SOC_DAPM_MUX("Stereo2 DMIC Mux", SND_SOC_NOPM, 0, 0,
				&rt5514_sto2_dmic_mux),

	/* ADC Mixer */
	SND_SOC_DAPM_SUPPLY("adc stereo1 filter", RT5514_CLK_CTRL1,
		RT5514_CLK_AD0_EN_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("adc stereo2 filter", RT5514_CLK_CTRL1,
		RT5514_CLK_AD1_EN_BIT, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("Sto1 ADC MIXL", SND_SOC_NOPM, 0, 0,
		rt5514_sto1_adc_l_mix, ARRAY_SIZE(rt5514_sto1_adc_l_mix)),
	SND_SOC_DAPM_MIXER("Sto1 ADC MIXR", SND_SOC_NOPM, 0, 0,
		rt5514_sto1_adc_r_mix, ARRAY_SIZE(rt5514_sto1_adc_r_mix)),
	SND_SOC_DAPM_MIXER("Sto2 ADC MIXL", SND_SOC_NOPM, 0, 0,
		rt5514_sto2_adc_l_mix, ARRAY_SIZE(rt5514_sto2_adc_l_mix)),
	SND_SOC_DAPM_MIXER("Sto2 ADC MIXR", SND_SOC_NOPM, 0, 0,
		rt5514_sto2_adc_r_mix, ARRAY_SIZE(rt5514_sto2_adc_r_mix)),

	SND_SOC_DAPM_ADC("Stereo1 ADC MIXL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("Stereo1 ADC MIXR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("Stereo2 ADC MIXL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("Stereo2 ADC MIXR", NULL, SND_SOC_NOPM, 0, 0),

	/* ADC PGA */
	SND_SOC_DAPM_PGA_E("Stereo1 ADC MIX", SND_SOC_NOPM, 0, 0, NULL, 0,
		rt5514_dmic_event, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_PGA_E("Stereo2 ADC MIX", SND_SOC_NOPM, 0, 0, NULL, 0,
		rt5514_dmic_event, SND_SOC_DAPM_PRE_PMU),

	/* Audio Interface */
	SND_SOC_DAPM_AIF_OUT("AIF1TX", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0)
};

static const struct snd_soc_dapm_route rt5514_dapm_routes[] = {
	{ "DMIC1", NULL, "DMIC1L" },
	{ "DMIC1", NULL, "DMIC1R" },
	{ "DMIC2", NULL, "DMIC2L" },
	{ "DMIC2", NULL, "DMIC2R" },

	{ "DMIC1L", NULL, "DMIC CLK", rt5514_is_not_dsp_enabled },
	{ "DMIC1R", NULL, "DMIC CLK", rt5514_is_not_dsp_enabled },
	{ "DMIC2L", NULL, "DMIC CLK", rt5514_is_not_dsp_enabled },
	{ "DMIC2R", NULL, "DMIC CLK", rt5514_is_not_dsp_enabled },

	{ "Stereo1 DMIC Mux", "DMIC1", "DMIC1" },
	{ "Stereo1 DMIC Mux", "DMIC2", "DMIC2" },

	{ "Sto1 ADC MIXL", "DMIC Switch", "Stereo1 DMIC Mux" },
	{ "Sto1 ADC MIXL", "BargeIn DMIC Switch", "Stereo1 DMIC Mux" },
	{ "Sto1 ADC MIXL", "ADC Switch", "AMICL" },
	{ "Sto1 ADC MIXR", "DMIC Switch", "Stereo1 DMIC Mux" },
	{ "Sto1 ADC MIXR", "BargeIn DMIC Switch", "Stereo1 DMIC Mux" },
	{ "Sto1 ADC MIXR", "ADC Switch", "AMICR" },

	{ "ADC Power", NULL, "LDO18 IN" },
	{ "ADC Power", NULL, "LDO18 ADC" },
	{ "ADC Power", NULL, "LDO21" },
	{ "ADC Power", NULL, "BG LDO18 IN" },
	{ "ADC Power", NULL, "BG LDO21" },
	{ "ADC Power", NULL, "BG MBIAS" },
	{ "ADC Power", NULL, "MBIAS" },
	{ "ADC Power", NULL, "VREF2" },
	{ "ADC Power", NULL, "VREF1" },

	{ "ADCL Power", NULL, "LDO16L" },
	{ "ADCL Power", NULL, "ADC1L" },
	{ "ADCL Power", NULL, "BSTL2" },
	{ "ADCL Power", NULL, "BSTL" },
	{ "ADCL Power", NULL, "ADCFEDL" },

	{ "ADCR Power", NULL, "LDO16R" },
	{ "ADCR Power", NULL, "ADC1R" },
	{ "ADCR Power", NULL, "BSTR2" },
	{ "ADCR Power", NULL, "BSTR" },
	{ "ADCR Power", NULL, "ADCFEDR" },

	{ "AMICL", NULL, "ADC CLK" },
	{ "AMICL", NULL, "ADC Power" },
	{ "AMICL", NULL, "ADCL Power" },
	{ "AMICR", NULL, "ADC CLK" },
	{ "AMICR", NULL, "ADC Power" },
	{ "AMICR", NULL, "ADCR Power" },

	{ "PLL1 LDO", NULL, "PLL1 LDO ENABLE" },
	{ "PLL1", NULL, "PLL1 LDO" },

	{ "PLL2 LDO", NULL, "PLL2 LDO ENABLE" },
	{ "PLL2", NULL, "PLL2 LDO" },

	{ "Stereo1 ADC MIXL", NULL, "Sto1 ADC MIXL" },
	{ "Stereo1 ADC MIXR", NULL, "Sto1 ADC MIXR" },

	{ "Stereo1 ADC MIX", NULL, "Stereo1 ADC MIXL" },
	{ "Stereo1 ADC MIX", NULL, "Stereo1 ADC MIXR" },
	{ "Stereo1 ADC MIX", NULL, "adc stereo1 filter",
		rt5514_is_not_dsp_enabled },
	{ "adc stereo1 filter", NULL, "PLL1", rt5514_is_sys_clk_from_pll1 },
	{ "adc stereo1 filter", NULL, "PLL2", rt5514_is_sys_clk_from_pll2 },
	{ "adc stereo1 filter", NULL, "ASRC AD1", rt5514_i2s_use_asrc },

	{ "Stereo2 DMIC Mux", "DMIC1", "DMIC1" },
	{ "Stereo2 DMIC Mux", "DMIC2", "DMIC2" },

	{ "Sto2 ADC MIXL", "DMIC Switch", "Stereo2 DMIC Mux" },
	{ "Sto2 ADC MIXL", "BargeIn DMIC Switch", "Stereo2 DMIC Mux" },
	{ "Sto2 ADC MIXL", "ADC Switch", "AMICL" },
	{ "Sto2 ADC MIXR", "DMIC Switch", "Stereo2 DMIC Mux" },
	{ "Sto2 ADC MIXR", "BargeIn DMIC Switch", "Stereo2 DMIC Mux" },
	{ "Sto2 ADC MIXR", "ADC Switch", "AMICR" },

	{ "Stereo2 ADC MIXL", NULL, "Sto2 ADC MIXL" },
	{ "Stereo2 ADC MIXR", NULL, "Sto2 ADC MIXR" },

	{ "Stereo2 ADC MIX", NULL, "Stereo2 ADC MIXL" },
	{ "Stereo2 ADC MIX", NULL, "Stereo2 ADC MIXR" },
	{ "Stereo2 ADC MIX", NULL, "adc stereo2 filter",
		rt5514_is_not_dsp_enabled },
	{ "adc stereo2 filter", NULL, "PLL1", rt5514_is_sys_clk_from_pll1 },
	{ "adc stereo2 filter", NULL, "PLL2", rt5514_is_sys_clk_from_pll2 },
	{ "adc stereo2 filter", NULL, "ASRC AD2", rt5514_i2s_use_asrc },

	{ "AIF1TX", NULL, "Stereo1 ADC MIX"},
	{ "AIF1TX", NULL, "Stereo2 ADC MIX"},
};

static int rt5514_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	int pre_div, bclk_ms, frame_size;
	unsigned int val_len = 0;

	mutex_lock(&rt5514->stream_lock);

	rt5514->dsp_enabled_last = rt5514->dsp_enabled;
	/* force enable hotword and music detect during recording */
	rt5514->dsp_enabled = 3;
	rt5514->dsp_adc_enabled = 1;
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 1);
	rt5514_dsp_enable(rt5514, false, false);
	if (rt5514_dsp_status_check(rt5514))
		rt5514_dsp_enable(rt5514, false, true);
	rt5514_spi_request_switch(SPI_SWITCH_MASK_LOAD, 0);

	rt5514->is_streaming = true;

	/* mute all dmic path to prevent pop */
	cancel_delayed_work_sync(&rt5514->unmute_work);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);

	/* send un-mute with delay */
	schedule_delayed_work(&rt5514->unmute_work,
		msecs_to_jiffies(UNMUTE_TIMEOUT_MS));

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled) {
		if (rt5514->dsp_adc_enabled) {
			regmap_write(rt5514->i2c_regmap, RT5514_DSP_FUNC,
				RT5514_DSP_FUNC_WOV_I2S_SENSOR);
		} else {
			if (rt5514->dsp_enabled < 5)
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_WOV_I2S);
			else
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_I2S);
		}

		rt5514_filter_power_reset(rt5514);

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			regmap_update_bits(rt5514->i2c_regmap, 0x18002010, 0x3,
				0x0);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
			regmap_update_bits(rt5514->i2c_regmap, 0x18002010, 0x3,
				0x2);
			break;

		default:
			mutex_unlock(&rt5514->stream_lock);
			return -EINVAL;
		}

		mutex_unlock(&rt5514->stream_lock);
		return 0;
	}
	mutex_unlock(&rt5514->stream_lock);

	rt5514->lrck = params_rate(params);
	pre_div = rl6231_get_clk_info(rt5514->sysclk, rt5514->lrck);
	if (pre_div < 0) {
		dev_err(codec->dev, "Unsupported clock setting\n");
		return -EINVAL;
	}

	frame_size = snd_soc_params_to_frame_size(params);
	if (frame_size < 0) {
		dev_err(codec->dev, "Unsupported frame size: %d\n", frame_size);
		return -EINVAL;
	}

	bclk_ms = frame_size > 32;
	rt5514->bclk = rt5514->lrck * (32 << bclk_ms);

	dev_dbg(dai->dev, "bclk is %dHz and lrck is %dHz\n",
		rt5514->bclk, rt5514->lrck);
	dev_dbg(dai->dev, "bclk_ms is %d and pre_div is %d for iis %d\n",
				bclk_ms, pre_div, dai->id);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val_len = RT5514_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val_len = RT5514_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		val_len = RT5514_I2S_DL_8;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(rt5514->regmap, RT5514_I2S_CTRL1, RT5514_I2S_DL_MASK,
		val_len);
	regmap_update_bits(rt5514->regmap, RT5514_CLK_CTRL1,
		RT5514_CLK_AD_ANA1_SEL_MASK,
		(pre_div + 1) << RT5514_CLK_AD_ANA1_SEL_SFT);
	regmap_update_bits(rt5514->regmap, RT5514_CLK_CTRL2,
		RT5514_CLK_SYS_DIV_OUT_MASK | RT5514_SEL_ADC_OSR_MASK,
		pre_div << RT5514_CLK_SYS_DIV_OUT_SFT |
		pre_div << RT5514_SEL_ADC_OSR_SFT);

	return 0;
}

static int rt5514_hw_free(struct snd_pcm_substream  *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&rt5514->stream_lock);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, RT5514_AD_AD_MUTE);

	rt5514->dsp_enabled = rt5514->dsp_req;
	rt5514->dsp_adc_enabled = rt5514->adc_req;
	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled) {
		rt5514_dsp_func_select(rt5514);

		if (rt5514->dsp_adc_enabled) {
			regmap_write(rt5514->i2c_regmap, RT5514_DSP_FUNC,
				RT5514_DSP_FUNC_WOV_SENSOR);
		} else {
			if (rt5514->dsp_enabled < 5)
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_WOV);
			else
				regmap_write(rt5514->i2c_regmap,
					RT5514_DSP_FUNC,
					RT5514_DSP_FUNC_SUSPEND);
		}

		rt5514_filter_power_reset(rt5514);

	}
	rt5514->is_streaming = false;
	if (rt5514->need_reload)
		rt5514_reload_firmware(rt5514);

	usleep_range(125000, 125100);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER0_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL1,
		RT5514_AD_AD_MUTE, 0x0);
	regmap_update_bits(rt5514->regmap, RT5514_DOWNFILTER1_CTRL2,
		RT5514_AD_AD_MUTE, 0x0);
	mutex_unlock(&rt5514->stream_lock);

	return 0;
}

static int rt5514_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val = 0;

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled)
		return 0;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;

	case SND_SOC_DAIFMT_NB_IF:
		reg_val |= RT5514_I2S_LR_INV;
		break;

	case SND_SOC_DAIFMT_IB_NF:
		reg_val |= RT5514_I2S_BP_INV;
		break;

	case SND_SOC_DAIFMT_IB_IF:
		reg_val |= RT5514_I2S_BP_INV | RT5514_I2S_LR_INV;
		break;

	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		reg_val |= RT5514_I2S_DF_LEFT;
		break;

	case SND_SOC_DAIFMT_DSP_A:
		reg_val |= RT5514_I2S_DF_PCM_A;
		break;

	case SND_SOC_DAIFMT_DSP_B:
		reg_val |= RT5514_I2S_DF_PCM_B;
		break;

	default:
		return -EINVAL;
	}

	regmap_update_bits(rt5514->regmap, RT5514_I2S_CTRL1,
		RT5514_I2S_DF_MASK | RT5514_I2S_BP_MASK | RT5514_I2S_LR_MASK,
		reg_val);

	return 0;
}

static int rt5514_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val = 0;

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled)
		return 0;

	if (freq == rt5514->sysclk && clk_id == rt5514->sysclk_src)
		return 0;

	switch (clk_id) {
	case RT5514_SCLK_S_MCLK:
		reg_val |= RT5514_CLK_SYS_PRE_SEL_MCLK;
		break;

	case RT5514_SCLK_S_PLL:
		reg_val |= RT5514_CLK_SYS_PRE_SEL_PLL;
		break;

	default:
		dev_err(codec->dev, "Invalid clock id (%d)\n", clk_id);
		return -EINVAL;
	}

	regmap_update_bits(rt5514->regmap, RT5514_CLK_CTRL2,
		RT5514_CLK_SYS_PRE_SEL_MASK, reg_val);

	rt5514->sysclk = freq;
	rt5514->sysclk_src = clk_id;

	dev_dbg(dai->dev, "Sysclk is %dHz and clock id is %d\n", freq, clk_id);

	return 0;
}

static int rt5514_set_dai_pll(struct snd_soc_dai *dai, int pll_id, int source,
			unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	struct rl6231_pll_code pll_code;
	int ret;

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled)
		return 0;

	if (!freq_in || !freq_out) {
		dev_dbg(codec->dev, "PLL disabled\n");

		rt5514->pll_in = 0;
		rt5514->pll_out = 0;
		regmap_update_bits(rt5514->regmap, RT5514_CLK_CTRL2,
			RT5514_CLK_SYS_PRE_SEL_MASK,
			RT5514_CLK_SYS_PRE_SEL_MCLK);

		return 0;
	}

	if (source == rt5514->pll_src && freq_in == rt5514->pll_in &&
	    freq_out == rt5514->pll_out)
		return 0;

	switch (source) {
	case RT5514_PLL_S_MCLK:
		if (rt5514->v_p)
			regmap_update_bits(rt5514->regmap,
				RT5514_PLL_SOURCE_CTRL,
				RT5514_PLL_2_SEL_MASK, RT5514_PLL_2_SEL_MCLK);
		else
			regmap_update_bits(rt5514->regmap,
				RT5514_PLL_SOURCE_CTRL,
				RT5514_PLL_1_SEL_MASK, RT5514_PLL_1_SEL_MCLK);
		break;

	case RT5514_PLL_S_BCLK:
		if (rt5514->v_p)
			regmap_update_bits(rt5514->regmap,
				RT5514_PLL_SOURCE_CTRL,
				RT5514_PLL_2_SEL_MASK, RT5514_PLL_2_SEL_SCLK);
		else
			regmap_update_bits(rt5514->regmap,
				RT5514_PLL_SOURCE_CTRL,
				RT5514_PLL_1_SEL_MASK, RT5514_PLL_1_SEL_SCLK);
		break;

	default:
		dev_err(codec->dev, "Unknown PLL source %d\n", source);
		return -EINVAL;
	}

	ret = rl6231_pll_calc(freq_in, freq_out, &pll_code);
	if (ret < 0) {
		dev_err(codec->dev, "Unsupport input clock %d\n", freq_in);
		return ret;
	}

	dev_dbg(codec->dev, "bypass=%d m=%d n=%d k=%d\n",
		pll_code.m_bp, (pll_code.m_bp ? 0 : pll_code.m_code),
		pll_code.n_code, pll_code.k_code);

	if (rt5514->v_p) {
		regmap_write(rt5514->regmap, RT5514_ANA_CTRL_PLL2_1,
			pll_code.k_code << RT5514_PLL_K_SFT |
			pll_code.n_code << RT5514_PLL_N_SFT |
			(pll_code.m_bp ? 0 : pll_code.m_code)
				<< RT5514_PLL_M_SFT);
		regmap_update_bits(rt5514->regmap, RT5514_ANA_CTRL_PLL2_2,
			RT5514_PLL_M_BP, pll_code.m_bp << RT5514_PLL_M_BP_SFT);
	} else {
		regmap_write(rt5514->regmap, RT5514_ANA_CTRL_PLL1_1,
			pll_code.k_code << RT5514_PLL_K_SFT |
			pll_code.n_code << RT5514_PLL_N_SFT |
			(pll_code.m_bp ? 0 : pll_code.m_code)
				<< RT5514_PLL_M_SFT);
		regmap_update_bits(rt5514->regmap, RT5514_ANA_CTRL_PLL1_2,
			RT5514_PLL_M_BP, pll_code.m_bp << RT5514_PLL_M_BP_SFT);
	}
	rt5514->pll_in = freq_in;
	rt5514->pll_out = freq_out;
	rt5514->pll_src = source;

	return 0;
}

static int rt5514_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	if (rt5514->dsp_enabled | rt5514->dsp_adc_enabled)
		return 0;

	if (rx_mask || tx_mask)
		val |= RT5514_TDM_MODE;

	switch (slots) {
	case 4:
		val |= RT5514_TDMSLOT_SEL_RX_4CH | RT5514_TDMSLOT_SEL_TX_4CH;
		break;

	case 6:
		val |= RT5514_TDMSLOT_SEL_RX_6CH | RT5514_TDMSLOT_SEL_TX_6CH;
		break;

	case 8:
		val |= RT5514_TDMSLOT_SEL_RX_8CH | RT5514_TDMSLOT_SEL_TX_8CH;
		break;

	case 2:
	default:
		break;
	}

	switch (slot_width) {
	case 20:
		val |= RT5514_CH_LEN_RX_20 | RT5514_CH_LEN_TX_20;
		break;

	case 24:
		val |= RT5514_CH_LEN_RX_24 | RT5514_CH_LEN_TX_24;
		break;

	case 25:
		val |= RT5514_TDM_MODE2;
		break;

	case 32:
		val |= RT5514_CH_LEN_RX_32 | RT5514_CH_LEN_TX_32;
		break;

	case 16:
	default:
		break;
	}

	regmap_update_bits(rt5514->regmap, RT5514_I2S_CTRL1, RT5514_TDM_MODE |
		RT5514_TDMSLOT_SEL_RX_MASK | RT5514_TDMSLOT_SEL_TX_MASK |
		RT5514_CH_LEN_RX_MASK | RT5514_CH_LEN_TX_MASK |
		RT5514_TDM_MODE2, val);

	return 0;
}

static int rt5514_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (IS_ERR(rt5514->mclk))
			break;

		if (snd_soc_codec_get_bias_level(codec) == SND_SOC_BIAS_ON) {
			clk_disable_unprepare(rt5514->mclk);
		} else {
			ret = clk_prepare_enable(rt5514->mclk);
			if (ret)
				return ret;
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	default:
		break;
	}

	return 0;
}

static int rt5514_probe(struct snd_soc_codec *codec)
{
	struct rt5514_priv *rt5514 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	rt5514->mclk = devm_clk_get(codec->dev, "mclk");
	if (PTR_ERR(rt5514->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	snd_soc_dapm_ignore_suspend(dapm, "DMIC1L");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1R");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC2L");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC2R");
	snd_soc_dapm_ignore_suspend(dapm, "AMICL");
	snd_soc_dapm_ignore_suspend(dapm, "AMICR");
	snd_soc_dapm_ignore_suspend(dapm, "AIF1 Capture");
	snd_soc_dapm_sync(dapm);

	rt5514->codec = codec;

	// setup watchdog handler for SPI driver
	rt5514_watchdog_handler_cb = rt5514_watchdog_handler;

	return 0;
}

static int rt5514_i2c_read(void *context, unsigned int reg, unsigned int *val)
{
	struct i2c_client *client = context;
	struct rt5514_priv *rt5514 = i2c_get_clientdata(client);

	regmap_read(rt5514->i2c_regmap, reg | RT5514_DSP_MAPPING, val);

	return 0;
}

static int rt5514_i2c_write(void *context, unsigned int reg, unsigned int val)
{
	struct i2c_client *client = context;
	struct rt5514_priv *rt5514 = i2c_get_clientdata(client);

	regmap_write(rt5514->i2c_regmap, reg | RT5514_DSP_MAPPING, val);

	return 0;
}

#define RT5514_STEREO_RATES SNDRV_PCM_RATE_8000_192000
#define RT5514_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)

static const struct snd_soc_dai_ops rt5514_aif_dai_ops = {
	.hw_params = rt5514_hw_params,
	.hw_free = rt5514_hw_free,
	.set_fmt = rt5514_set_dai_fmt,
	.set_sysclk = rt5514_set_dai_sysclk,
	.set_pll = rt5514_set_dai_pll,
	.set_tdm_slot = rt5514_set_tdm_slot,
};

struct snd_soc_dai_driver rt5514_dai[] = {
	{
		.name = "rt5514-aif1",
		.id = 0,
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = RT5514_STEREO_RATES,
			.formats = RT5514_FORMATS,
		},
		.ops = &rt5514_aif_dai_ops,
	}
};

static struct snd_soc_codec_driver soc_codec_dev_rt5514 = {
	.probe = rt5514_probe,
	.idle_bias_off = true,
	.set_bias_level = rt5514_set_bias_level,
	.component_driver = {
		.controls		= rt5514_snd_controls,
		.num_controls		= ARRAY_SIZE(rt5514_snd_controls),
		.dapm_widgets		= rt5514_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(rt5514_dapm_widgets),
		.dapm_routes		= rt5514_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(rt5514_dapm_routes),
	},
};

static const struct regmap_config rt5514_i2c_regmap = {
	.name = "i2c",
	.reg_bits = 32,
	.val_bits = 32,

	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config rt5514_regmap = {
	.reg_bits = 16,
	.val_bits = 32,

	.max_register = RT5514_VENDOR_ID2,
	.volatile_reg = rt5514_volatile_register,
	.readable_reg = rt5514_readable_register,
	.reg_read = rt5514_i2c_read,
	.reg_write = rt5514_i2c_write,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = rt5514_reg,
	.num_reg_defaults = ARRAY_SIZE(rt5514_reg),
	.use_single_rw = true,
};

static const struct i2c_device_id rt5514_i2c_id[] = {
	{ "rt5514", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5514_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id rt5514_of_match[] = {
	{ .compatible = "realtek,rt5514", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5514_of_match);
#endif

#ifdef CONFIG_ACPI
static struct acpi_device_id rt5514_acpi_match[] = {
	{ "10EC5514", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, rt5514_acpi_match);
#endif

static int rt5514_parse_dp(struct rt5514_priv *rt5514, struct device *dev)
{
	device_property_read_u32(dev, "realtek,dmic-init-delay",
		&rt5514->pdata.dmic_init_delay);

	return 0;
}

static __maybe_unused int rt5514_i2c_resume(struct device *dev)
{
	struct rt5514_priv *rt5514 = dev_get_drvdata(dev);
	unsigned int val;

	/*
	 * Add a bogus read to avoid rt5514's confusion after s2r in case it
	 * saw glitches on the i2c lines and thought the other side sent a
	 * start bit.
	 */
	regmap_read(rt5514->regmap, RT5514_VENDOR_ID2, &val);

	return 0;
}

static int rt5514_i2c_probe(struct i2c_client *i2c,
		    const struct i2c_device_id *id)
{
	struct rt5514_platform_data *pdata = dev_get_platdata(&i2c->dev);
	struct rt5514_priv *rt5514;
	int ret;
	unsigned int val = ~0;

	rt5514 = devm_kzalloc(&i2c->dev, sizeof(struct rt5514_priv),
				GFP_KERNEL);
	if (rt5514 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, rt5514);

	if (pdata)
		rt5514->pdata = *pdata;
	else
		rt5514_parse_dp(rt5514, &i2c->dev);

	rt5514->i2c_regmap = devm_regmap_init_i2c(i2c, &rt5514_i2c_regmap);
	if (IS_ERR(rt5514->i2c_regmap)) {
		ret = PTR_ERR(rt5514->i2c_regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	rt5514_g_i2c_regmap = rt5514->i2c_regmap;
	g_rt5514 = rt5514;

	rt5514->regmap = devm_regmap_init(&i2c->dev, NULL, i2c, &rt5514_regmap);
	if (IS_ERR(rt5514->regmap)) {
		ret = PTR_ERR(rt5514->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	rt5514->gpiod_reset = devm_gpiod_get_optional(&i2c->dev, "reset",
							GPIOD_OUT_HIGH);
	if (IS_ERR(rt5514->gpiod_reset)) {
		ret = PTR_ERR(rt5514->gpiod_reset);
		dev_err(&i2c->dev, "Failed to initialize gpiod: %d\n", ret);
		return ret;
	}

	/*
	 * The rt5514 can get confused if the i2c lines glitch together, as
	 * can happen at bootup as regulators are turned off and on.  If it's
	 * in this glitched state the first i2c read will fail, so we'll give
	 * it one change to retry.
	 */
	ret = regmap_read(rt5514->regmap, RT5514_VENDOR_ID2, &val);
	if (ret || val != RT5514_DEVICE_ID)
		ret = regmap_read(rt5514->regmap, RT5514_VENDOR_ID2, &val);
	if (ret || val != RT5514_DEVICE_ID) {
		dev_err(&i2c->dev,
			"Device with ID register %x is not rt5514\n", val);
		return -ENODEV;
	}

	regmap_read(rt5514->regmap, RT5514_VENDOR_ID1, &val);
	if (val == 0x80) {
		rt5514->v_p = true;
		rt5514->fw_name[0] = RT5514P_FIRMWARE1;
		rt5514->fw_name[1] = RT5514P_FIRMWARE2;
		rt5514->fw_name[2] = RT5514P_FIRMWARE3;
		rt5514->fw_name[3] = RT5514P_FIRMWARE4;
		rt5514->fw_addr[0] = 0x4fe00000;
		rt5514->fw_addr[1] = 0x4ff00000;
		rt5514->fw_addr[2] = 0x4fe98000;
		rt5514->fw_addr[3] = 0x4fea8000;
		rt5514_i2c_patch = const_rt5514p_i2c_patch;
		rt5514->i2c_patch_size = ARRAY_SIZE(const_rt5514p_i2c_patch);
	} else {
		rt5514->fw_name[0] = RT5514_FIRMWARE1;
		rt5514->fw_name[1] = RT5514_FIRMWARE2;
		rt5514->fw_name[2] = RT5514_FIRMWARE3;
		rt5514->fw_name[3] = RT5514_FIRMWARE4;
		rt5514->fw_addr[0] = 0x4ff60000;
		rt5514->fw_addr[1] = 0x4ffc0000;
		rt5514->fw_addr[2] = 0x4ffaa800;
		rt5514->fw_addr[3] = 0x4ffb4800;
		rt5514_i2c_patch = const_rt5514_i2c_patch;
		rt5514->i2c_patch_size = ARRAY_SIZE(const_rt5514_i2c_patch);
	}

	ret = regmap_multi_reg_write(rt5514->i2c_regmap, rt5514_i2c_patch,
				    rt5514->i2c_patch_size);
	if (ret != 0)
		dev_warn(&i2c->dev, "Failed to apply i2c_regmap patch: %d\n",
			ret);

	ret = regmap_register_patch(rt5514->regmap, rt5514_patch,
				    ARRAY_SIZE(rt5514_patch));
	if (ret != 0)
		dev_warn(&i2c->dev, "Failed to apply regmap patch: %d\n", ret);

	/* 0 => Stereo ; 1 => Mono */
	rt5514->dsp_buffer_channel = 1;

	rt5514->spi_switch = 0;

	rt5514->mic_delay = 0;

	rt5514_set_gpio(RT5514_SPI_SWITCH_GPIO, rt5514->spi_switch);

	INIT_DELAYED_WORK(&rt5514->unmute_work, rt5514_unmute_work);

	mutex_init(&rt5514->stream_lock);

	dev_info(&i2c->dev, "Register rt5514 success\n");

	return snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5514,
			rt5514_dai, ARRAY_SIZE(rt5514_dai));
}

static int rt5514_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);

	return 0;
}

static const struct dev_pm_ops rt5514_i2_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(NULL, rt5514_i2c_resume)
};

static struct i2c_driver rt5514_i2c_driver = {
	.driver = {
		.name = "rt5514",
		.acpi_match_table = ACPI_PTR(rt5514_acpi_match),
		.of_match_table = of_match_ptr(rt5514_of_match),
		.pm = &rt5514_i2_pm_ops,
	},
	.probe = rt5514_i2c_probe,
	.remove   = rt5514_i2c_remove,
	.id_table = rt5514_i2c_id,
};
module_i2c_driver(rt5514_i2c_driver);

MODULE_DESCRIPTION("ASoC RT5514 driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
