/*
 * rt5514-spi.c  --  RT5514 SPI driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "rt5514-spi.h"
#include "rt5514.h"
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
#include <linux/codec-misc.h>
#endif

#define COPY_WORK_DELAY_TIME_MS 100
#define WAKEUP_TIMEOUT	5000
#define MAX_STREAM_FLAG	3

void (*rt5514_watchdog_handler_cb)(void) = NULL;
EXPORT_SYMBOL_GPL(rt5514_watchdog_handler_cb);
struct regmap *rt5514_g_i2c_regmap;
EXPORT_SYMBOL_GPL(rt5514_g_i2c_regmap);

static struct spi_device *rt5514_spi;
static struct mutex spi_lock;
static struct mutex switch_lock;
static struct wakeup_source rt5514_spi_ws;
static struct wakeup_source rt5514_watchdog_ws;
static u32 spi_switch_mask;
static int handshake_gpio, handshake_ack_irq;
struct completion switch_ack;
static struct rt5514_dsp *rt5514_g_dsp;

struct rt5514_dsp {
	struct device *dev;
	struct snd_soc_component *component;
	struct delayed_work copy_work_0, copy_work_1, copy_work_2, start_work,
		adc_work, chre_chk_work;
	struct mutex dma_lock;
	struct snd_pcm_substream *substream[3];
	unsigned int buf_base[3], buf_limit[3], buf_rp[3], buf_rp_addr[3];
	unsigned int stream_flag[MAX_STREAM_FLAG];
	unsigned int hotword_ignore_ms, musdet_ignore_ms;
	size_t buf_size[3], get_size[3], dma_offset[3];
};

static const struct snd_pcm_hardware rt5514_spi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= 0x20000 / 8,
	.periods_min		= 8,
	.periods_max		= 8,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 0x20000,
};

static const char * const st_enable_text[] = {
	"ZERO", "ON"
};

static const struct soc_enum st_enable_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
		ARRAY_SIZE(st_enable_text), st_enable_text);

static const struct snd_kcontrol_new st1_mux[] = {
	SOC_DAPM_ENUM("SoundTrigger1 Enable", st_enable_enum)
};

static const struct snd_kcontrol_new st2_mux[] = {
	SOC_DAPM_ENUM("SoundTrigger2 Enable", st_enable_enum)
};

static struct snd_soc_dapm_widget rt5514_spi_dapm_widgets[] = {
	/* Stream widgets */
	SND_SOC_DAPM_AIF_OUT("AIF_SPI_FE",
			"SoundTrigger Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF_SPI_FE2",
			"SoundTrigger Capture 2", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF_SPI_FE3",
			"ADC Capture", 0, 0, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF_SPI_BE", "SPI Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF_SPI_BE2", "SPI Capture 2", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF_SPI_BE3", "SPI Capture 3", 0, 0, 0, 0),

	SND_SOC_DAPM_MICBIAS("LDO_SOURCE", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("SoundTrigger1 Enable", SND_SOC_NOPM, 0, 0, st1_mux),
	SND_SOC_DAPM_MUX("SoundTrigger2 Enable", SND_SOC_NOPM, 0, 0, st2_mux),

	SND_SOC_DAPM_INPUT("DSP_IN1"),
	SND_SOC_DAPM_INPUT("DSP_IN2"),
	SND_SOC_DAPM_INPUT("DSP_IN3"),
};

static const struct snd_soc_dapm_route intercon_common[] = {
	{"AIF_SPI_FE", NULL, "AIF_SPI_BE"},
	{"AIF_SPI_FE2", NULL, "AIF_SPI_BE2"},
	{"AIF_SPI_FE3", NULL, "AIF_SPI_BE3"},

	{"SoundTrigger1 Enable", "ON", "DSP_IN1"},
	{"SoundTrigger2 Enable", "ON", "DSP_IN2"},

	{"SoundTrigger Capture", NULL, "AIF_SPI_FE"},
	{"SoundTrigger Capture 2", NULL, "AIF_SPI_FE2"},
	{"ADC Capture", NULL, "AIF_SPI_FE3"},

	{"AIF_SPI_BE", NULL, "SoundTrigger1 Enable"},
	{"AIF_SPI_BE2", NULL, "SoundTrigger2 Enable"},
	{"AIF_SPI_BE3", NULL, "DSP_IN3"},

	{"AIF_SPI_BE", NULL, "SPI Capture"},
	{"AIF_SPI_BE2", NULL, "SPI Capture 2"},
	{"AIF_SPI_BE3", NULL, "SPI Capture 3"},
};

static int rt5514_spi_fe_dai_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm =
				 snd_soc_component_get_dapm(component);

	snd_soc_dapm_ignore_suspend(dapm, "SoundTrigger Capture");
	snd_soc_dapm_ignore_suspend(dapm, "SoundTrigger Capture 2");
	snd_soc_dapm_ignore_suspend(dapm, "ADC Capture");
	snd_soc_dapm_ignore_suspend(dapm, "SPI Capture");
	snd_soc_dapm_ignore_suspend(dapm, "SPI Capture 2");
	snd_soc_dapm_ignore_suspend(dapm, "SPI Capture 3");
	snd_soc_dapm_ignore_suspend(dapm, "DSP_IN1");
	snd_soc_dapm_ignore_suspend(dapm, "DSP_IN2");
	snd_soc_dapm_ignore_suspend(dapm, "DSP_IN3");

	return 0;
}

static struct snd_soc_dai_driver rt5514_spi_dai[] = {
	{
		.name = "rt5514-dsp-fe-dai1",
		.id = 0,
		.capture = {
			.stream_name = "SoundTrigger Capture",
			.aif_name = "AIF_SPI_FE",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "rt5514-dsp-fe-dai2",
		.id = 1,
		.capture = {
			.stream_name = "SoundTrigger Capture 2",
			.aif_name = "AIF_SPI_FE2",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "rt5514-dsp-fe-dai3",
		.id = 2,
		.capture = {
			.stream_name = "ADC Capture",
			.aif_name = "AIF_SPI_FE3",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "rt5514-dsp-be-dai1",
		.id = 3,
		.capture = {
			.stream_name = "SPI Capture",
			.aif_name = "AIF_SPI_BE",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "rt5514-dsp-be-dai2",
		.id = 4,
		.capture = {
			.stream_name = "SPI Capture 2",
			.aif_name = "AIF_SPI_BE2",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "rt5514-dsp-be-dai3",
		.id = 5,
		.capture = {
			.stream_name = "SPI Capture 3",
			.aif_name = "AIF_SPI_BE3",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_8000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static const unsigned int rt5514_regdump_table1[] = {
	0x18000000, 0x18000004, 0x18000008, 0x1800000c, 0x18000010,
	0x18000014, 0x18000020, 0x18000024, 0x18000028, 0x1800002c,
	0x18000030, 0x18000034, 0x18000038, 0x18000040, 0x18000044,
	0x18000050, 0x18000100, 0x18000104, 0x18000108, 0x1800010c,
	0x18000110, 0x18000114, 0x18000120, 0x18000124, 0x18000128,
	0x1800012c, 0x18000130, 0x18000134, 0x18000138, 0x18000140,
	0x18000144, 0x18000150, 0x18000200, 0x18000204, 0x18000208,
	0x1800020c, 0x18000210, 0x18000214, 0x18000220, 0x18000224,
	0x18000228, 0x1800022c, 0x18000230, 0x18000234, 0x18000238,
	0x18000240, 0x18000244, 0x18000250, 0x18000300, 0x18000304,
	0x18000308, 0x1800030c, 0x18000310, 0x18000314, 0x18000320,
	0x18000324, 0x18000328, 0x1800032c, 0x18000330, 0x18000334,
	0x18000338, 0x18000340, 0x18000344, 0x18000350, 0x18000400,
	0x18000404, 0x18000408, 0x1800040c, 0x18000410, 0x18000414,
	0x18000420, 0x18000424, 0x18000428, 0x1800042c, 0x18000430,
	0x18000434, 0x18000438, 0x18000450, 0x18000454, 0x18000500,
	0x18000504, 0x18000508, 0x1800050c, 0x18000510, 0x18000514,
	0x18000520, 0x18000524, 0x18000528, 0x1800052c, 0x18000530,
	0x18000534, 0x18000538, 0x18000550, 0x18000554, 0x18000600,
	0x18000604, 0x18000608, 0x1800060c, 0x18000610, 0x18000614,
	0x18000620, 0x18000624, 0x18000628, 0x1800062c, 0x18000630,
	0x18000634, 0x18000638, 0x18000650, 0x18000654, 0x18000700,
	0x18000704, 0x18000708, 0x1800070c, 0x18000710, 0x18000714,
	0x18000720, 0x18000724, 0x18000728, 0x1800072c, 0x18000730,
	0x18000734, 0x18000738, 0x18000750, 0x18000754, 0x18001000,
	0x18001004, 0x18001008, 0x1800100c, 0x18001010, 0x18001014,
	0x18001018, 0x1800101c, 0x18001020, 0x18001024, 0x18001024,
	0x18001028, 0x1800102c, 0x18001030, 0x18001034, 0x18001038,
	0x1800103c, 0x18001040, 0x18001044, 0x18001100, 0x18001104,
	0x18001108, 0x1800110c, 0x18001110, 0x18001114, 0x18001118,
	0x1800111c, 0x18001200, 0x18001204, 0x18001300, 0x18001304,
	0x18001308, 0x1800130c, 0x18001310, 0x18001310, 0x18001310,
};

static const unsigned int rt5514_regdump_table2[] = {
	0x18002000, 0x18002004, 0x18002008, 0x18002010, 0x18002014,
	0x18002018, 0x1800201c, 0x18002020, 0x18002024, 0x18002028,
	0x1800202c, 0x18002030, 0x18002034, 0x18002038, 0x1800203c,
	0x18002040, 0x18002044, 0x18002048, 0x1800204c, 0x18002050,
	0x18002054, 0x18002058, 0x1800205c, 0x18002060, 0x18002064,
	0x18002068, 0x1800206c, 0x18002070, 0x18002074, 0x18002078,
	0x1800207c, 0x18002080, 0x18002084, 0x18002088, 0x1800208c,
	0x18002090, 0x18002094, 0x180020a0, 0x180020a4, 0x180020ac,
	0x180020b0, 0x180020b4, 0x180020b8, 0x180020bc, 0x180020c0,
	0x180020c4, 0x180020d0, 0x180020d4, 0x180020d8, 0x18002100,
	0x18002104, 0x18002108, 0x18002110, 0x18002114, 0x18002118,
	0x1800211c, 0x18002120, 0x18002124, 0x18002128, 0x1800212c,
	0x18002140, 0x18002144, 0x18002148, 0x1800214c, 0x18002160,
	0x18002164, 0x18002168, 0x1800216c, 0x18002170, 0x18002174,
	0x18002180, 0x18002184, 0x18002190, 0x18002194, 0x18002198,
	0x180021a0, 0x180021a4, 0x180021a8, 0x18002200, 0x18002204,
	0x18002208, 0x1800220c, 0x18002210, 0x18002214, 0x18002218,
	0x1800221c, 0x18002220, 0x18002224, 0x18002228, 0x1800222c,
	0x18002230, 0x18002240, 0x18002250, 0x18002254, 0x18002258,
	0x18002260, 0x18002264, 0x18002268, 0x18002d00, 0x18002d04,
	0x18002d08, 0x18002e00, 0x18002e04, 0x18002f00, 0x18002f04,
	0x18002f08, 0x18002f10, 0x18002f14, 0x18002fa0, 0x18002fa4,
	0x18002fa8, 0x18002fac, 0x18002fb0, 0x18002fb4, 0x18002fb8,
	0x18002fbc, 0x18002fc0, 0x18002fc4, 0x18002fc8, 0x18002fcc,
	0x18002fd0, 0x18002fd4, 0x18002fd8, 0x18002fdc, 0x18002fe0,
	0x18002fe4, 0x18002fe8, 0x18002fec, 0x18002ff0, 0x18002ff4,
};

#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
int rt5514_codec_state(void)
{
	int ret;
	unsigned int val;

	ret = regmap_read(rt5514_g_i2c_regmap, 0x18002ff4, &val);
	if (ret || val != 0x10ec5514) {
		pr_err("Device with ID register %x is not rt5514\n", val);
		return CODEC_STATE_UNKNOWN;
	}

	return CODEC_STATE_ONLINE;
}

char *rt5514_codec_hwinfo(void)
{
	unsigned int val;

	regmap_read(rt5514_g_i2c_regmap, 0x18002ff0, &val);
	if (val == 0x80)
		return "rt5514p";
	else
		return "rt5514";
}
#endif


static bool rt5514_watchdog_dbg_info(struct rt5514_dsp *rt5514_dsp)
{
	RT5514_DBGBUF_MEM dbgbuf;
	unsigned int i, val[5], ret = 0;

	ret = regmap_read(rt5514_g_i2c_regmap, 0x18002f04, &val[0]);

	if (ret) {
		dev_err(rt5514_dsp->dev,
			"Failed to i2c read %d\n", ret);
		return true;
	}

	if (!(val[0] & 0x2))
		return false;

	/* check chip version: value 0x80 = ACL5514p, others = ALC5514 */
	regmap_read(rt5514_g_i2c_regmap, 0x18002ff0, &val[0]);
	if (val[0] == 0x80)
		val[1] = 0x4fe00000;
	else
		val[1] = 0x4ff60000;

	rt5514_spi_request_switch(SPI_SWITCH_MASK_WATCHDOG, 1);
	rt5514_spi_burst_read(val[1], (u8 *)&dbgbuf, RT5514_DBG_BUF_SIZE);
	rt5514_spi_request_switch(SPI_SWITCH_MASK_WATCHDOG, 0);

	dev_err(rt5514_dsp->dev, "[DSP Dump]");
	for (i = 0; i < RT5514_DBG_BUF_CNT; i++)
		dev_err(&rt5514_spi->dev, "[%02x][%06x][%08x]\n",
			dbgbuf.unit[i].id, dbgbuf.unit[i].ts,
			dbgbuf.unit[i].val);
	dev_err(rt5514_dsp->dev, "[%08x][%08x]\n",
		dbgbuf.reserve, dbgbuf.idx);

	dev_err(rt5514_dsp->dev, "[Reg Dump]");
	for (i = 0; i < ARRAY_SIZE(rt5514_regdump_table1); i+=5) {
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table1[i],
			&val[0]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table1[i + 1],
			&val[1]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table1[i + 2],
			&val[2]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table1[i + 3],
			&val[3]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table1[i + 4],
			&val[4]);
		dev_err(rt5514_dsp->dev, "[%08x][%08x][%08x][%08x][%08x]",
			val[0], val[1], val[2], val[3], val[4]);
	}

	dev_err(rt5514_dsp->dev,
		"==================================================");

	regmap_write(rt5514_g_i2c_regmap, 0xfafafafa, 0x00000001);
	for (i = 0; i < ARRAY_SIZE(rt5514_regdump_table2); i+=5) {
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table2[i],
			&val[0]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table2[i + 1],
			&val[1]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table2[i + 2],
			&val[2]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table2[i + 3],
			&val[3]);
		regmap_read(rt5514_g_i2c_regmap, rt5514_regdump_table2[i + 4],
			&val[4]);
		dev_err(rt5514_dsp->dev, "[%08x][%08x][%08x][%08x][%08x]",
			val[0], val[1], val[2], val[3], val[4]);
	}
	regmap_write(rt5514_g_i2c_regmap, 0xfafafafa, 0x00000000);

	return true;
}

bool rt5514_check_chre_read_done(void)
{
	unsigned int val;

	regmap_read(rt5514_g_i2c_regmap, RT5514_CHRE_READ, &val);
	if (val == 1)
		return true;
	else
		return false;
}

static void rt5514_chre_check_work(struct work_struct *work)
{
	rt5514_spi_request_switch(SPI_SWITCH_MASK_CHRE_READ, 0);
	pr_info("%s: stop chre check work", __func__);
}

void rt5514_spi_request_switch(u32 mask, bool is_require)
{
	u32 previous_mask = spi_switch_mask;
	int ret;

	mutex_lock(&switch_lock);

	if (is_require) {
		spi_switch_mask |= mask;
	} else {
		spi_switch_mask &= ~mask;
	}

	if (!previous_mask == !spi_switch_mask) {
		mutex_unlock(&switch_lock);
		return;
	}

	if (spi_switch_mask & SPI_SWITCH_MASK_NO_IRQ) {
		rt5514_set_gpio(RT5514_SPI_SWITCH_GPIO, 0);
		mutex_unlock(&switch_lock);
		return;
	}

	if (spi_switch_mask > 0) {
		pr_debug("%s: on (mask=%2x)", __func__, spi_switch_mask);
		/* Set handshake GPIO */
		gpio_set_value(handshake_gpio, 1);

		/* Enable ack IRQ and wait for completion */
		reinit_completion(&switch_ack);
		enable_irq(handshake_ack_irq);
		ret = wait_for_completion_timeout(&switch_ack,
						  msecs_to_jiffies(100));

		if (ret == 0) {
			disable_irq(handshake_ack_irq);
			pr_warn("%s: Timeout! Force switch", __func__);
		}

		/* Set switch pin back */
		rt5514_set_gpio(RT5514_SPI_SWITCH_GPIO, 0);

		/* Check LPI read done */
		if (rt5514_check_chre_read_done()) {
			pr_info("%s: register CHRE_READ is 1", __func__);
			regmap_write(rt5514_g_i2c_regmap, RT5514_CHRE_READ, 0);
			spi_switch_mask |= SPI_SWITCH_MASK_CHRE_READ;
			cancel_delayed_work_sync(&rt5514_g_dsp->chre_chk_work);
			schedule_delayed_work(&rt5514_g_dsp->chre_chk_work,
					      msecs_to_jiffies(850));
		}
	} else {
		pr_debug("%s: off (mask=%2x)", __func__, spi_switch_mask);
		/* Set switch pin to CHRE */
		rt5514_set_gpio(RT5514_SPI_SWITCH_GPIO, 1);
		/* Set handshake GPIO */
		gpio_set_value(handshake_gpio, 0);
	}
	mutex_unlock(&switch_lock);
}
EXPORT_SYMBOL_GPL(rt5514_spi_request_switch);

static void rt5514_spi_copy_work_0(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, copy_work_0.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes, truncated_bytes = 0;
	unsigned int cur_wp, remain_data;
	u8 buf[8];
	struct snd_soc_component *component = rt5514_dsp->component;

	pm_wakeup_event(rt5514_dsp->dev, WAKEUP_TIMEOUT);
	if (snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		dev_err(rt5514_dsp->dev, "%s: Request in suspend\n", __func__);
		return;
	}

	mutex_lock(&rt5514_dsp->dma_lock);

	rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 1);

	if (!rt5514_dsp->substream[0]) {
		dev_err(rt5514_dsp->dev, "No pcm0 substream\n");
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
		goto done;
	}

	runtime = rt5514_dsp->substream[0]->runtime;
	period_bytes = snd_pcm_lib_period_bytes(rt5514_dsp->substream[0]);
	if (!period_bytes) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_0,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	/* check if hw has space for one period_size */
	if (snd_pcm_capture_hw_avail(runtime) <= runtime->period_size) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_0,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	if (rt5514_dsp->buf_size[0] % period_bytes)
		rt5514_dsp->buf_size[0] =
			(rt5514_dsp->buf_size[0] / period_bytes) * period_bytes;

	if (rt5514_dsp->get_size[0] >= rt5514_dsp->buf_size[0]) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp_addr[0], (u8 *)&buf,
			sizeof(buf));
		cur_wp = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
		if ((cur_wp & 0xffe00000) != 0x4fe00000) {
			rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
			schedule_delayed_work(&rt5514_dsp->copy_work_0,
				msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
			goto done;
		}

		if (cur_wp >= rt5514_dsp->buf_rp[0])
			remain_data = (cur_wp - rt5514_dsp->buf_rp[0]);
		else
			remain_data =
				(rt5514_dsp->buf_limit[0] -
				rt5514_dsp->buf_rp[0]) +
				(cur_wp - rt5514_dsp->buf_base[0]);

		if (remain_data < period_bytes) {
			rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
			schedule_delayed_work(&rt5514_dsp->copy_work_0,
				msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
			goto done;
		}
	}

	if (rt5514_dsp->buf_rp[0] + period_bytes <= rt5514_dsp->buf_limit[0]) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[0],
			runtime->dma_area + rt5514_dsp->dma_offset[0],
			period_bytes);

		if (rt5514_dsp->buf_rp[0] + period_bytes ==
			rt5514_dsp->buf_limit[0])
			rt5514_dsp->buf_rp[0] = rt5514_dsp->buf_base[0];
		else
			rt5514_dsp->buf_rp[0] += period_bytes;
	} else {
		truncated_bytes =
			rt5514_dsp->buf_limit[0] - rt5514_dsp->buf_rp[0];
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[0],
			runtime->dma_area + rt5514_dsp->dma_offset[0],
			truncated_bytes);

		rt5514_spi_burst_read(rt5514_dsp->buf_base[0],
			runtime->dma_area + rt5514_dsp->dma_offset[0] +
			truncated_bytes, period_bytes - truncated_bytes);

		rt5514_dsp->buf_rp[0] = rt5514_dsp->buf_base[0] + period_bytes -
			truncated_bytes;
	}

	rt5514_dsp->get_size[0] += period_bytes;
	rt5514_dsp->dma_offset[0] += period_bytes;
	if (rt5514_dsp->dma_offset[0] >= runtime->dma_bytes)
		rt5514_dsp->dma_offset[0] = 0;

	snd_pcm_period_elapsed(rt5514_dsp->substream[0]);

	schedule_delayed_work(&rt5514_dsp->copy_work_0, msecs_to_jiffies(0));

done:
	mutex_unlock(&rt5514_dsp->dma_lock);
}

static void rt5514_spi_copy_work_1(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, copy_work_1.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes, truncated_bytes = 0;
	unsigned int cur_wp, remain_data;
	u8 buf[8];
	struct snd_soc_component *component = rt5514_dsp->component;

	pm_wakeup_event(rt5514_dsp->dev, WAKEUP_TIMEOUT);
	if (snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		dev_err(rt5514_dsp->dev, "%s: Request in suspend\n", __func__);
		return;
	}

	mutex_lock(&rt5514_dsp->dma_lock);

	rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 1);

	if (!rt5514_dsp->substream[1]) {
		dev_err(rt5514_dsp->dev, "No pcm1 substream\n");
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
		goto done;
	}

	runtime = rt5514_dsp->substream[1]->runtime;
	period_bytes = snd_pcm_lib_period_bytes(rt5514_dsp->substream[1]);
	if (!period_bytes) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_1,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	/* check if hw has space for one period_size */
	if (snd_pcm_capture_hw_avail(runtime) <= runtime->period_size) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_1,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	if (rt5514_dsp->buf_size[1] % period_bytes)
		rt5514_dsp->buf_size[1] =
			(rt5514_dsp->buf_size[1] / period_bytes) * period_bytes;

	if (rt5514_dsp->get_size[1] >= rt5514_dsp->buf_size[1]) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp_addr[1], (u8 *)&buf,
			sizeof(buf));
		cur_wp = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
		if ((cur_wp & 0xffe00000) != 0x4fe00000) {
			rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
			schedule_delayed_work(&rt5514_dsp->copy_work_1,
				msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
			goto done;
		}

		if (cur_wp >= rt5514_dsp->buf_rp[1])
			remain_data = (cur_wp - rt5514_dsp->buf_rp[1]);
		else
			remain_data =
				(rt5514_dsp->buf_limit[1] -
				rt5514_dsp->buf_rp[1]) +
				(cur_wp - rt5514_dsp->buf_base[1]);

		if (remain_data < period_bytes) {
			rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
			schedule_delayed_work(&rt5514_dsp->copy_work_1,
				msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
			goto done;
		}
	}

	if (rt5514_dsp->buf_rp[1] + period_bytes <= rt5514_dsp->buf_limit[1]) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[1],
			runtime->dma_area + rt5514_dsp->dma_offset[1],
			period_bytes);

		if (rt5514_dsp->buf_rp[1] + period_bytes ==
			rt5514_dsp->buf_limit[1])
			rt5514_dsp->buf_rp[1] = rt5514_dsp->buf_base[1];
		else
			rt5514_dsp->buf_rp[1] += period_bytes;
	} else {
		truncated_bytes =
			rt5514_dsp->buf_limit[1] - rt5514_dsp->buf_rp[1];
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[1],
			runtime->dma_area + rt5514_dsp->dma_offset[1],
			truncated_bytes);

		rt5514_spi_burst_read(rt5514_dsp->buf_base[1],
			runtime->dma_area + rt5514_dsp->dma_offset[1] +
			truncated_bytes, period_bytes - truncated_bytes);

		rt5514_dsp->buf_rp[1] = rt5514_dsp->buf_base[1] + period_bytes -
			truncated_bytes;
	}

	rt5514_dsp->get_size[1] += period_bytes;
	rt5514_dsp->dma_offset[1] += period_bytes;
	if (rt5514_dsp->dma_offset[1] >= runtime->dma_bytes)
		rt5514_dsp->dma_offset[1] = 0;

	snd_pcm_period_elapsed(rt5514_dsp->substream[1]);

	schedule_delayed_work(&rt5514_dsp->copy_work_1, msecs_to_jiffies(0));

done:
	mutex_unlock(&rt5514_dsp->dma_lock);
}

static void rt5514_spi_copy_work_2(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, copy_work_2.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes, truncated_bytes = 0;
	unsigned int cur_wp, remain_data;
	u8 buf[8];
	struct snd_soc_component *component = rt5514_dsp->component;

	pm_wakeup_event(rt5514_dsp->dev, WAKEUP_TIMEOUT);
	if (snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		dev_err(rt5514_dsp->dev, "%s: Request in suspend\n", __func__);
		return;
	}

	mutex_lock(&rt5514_dsp->dma_lock);

	rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 1);

	if (!rt5514_dsp->substream[2]) {
		dev_err(rt5514_dsp->dev, "No pcm2 substream\n");
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		goto done;
	}

	runtime = rt5514_dsp->substream[2]->runtime;
	period_bytes = snd_pcm_lib_period_bytes(rt5514_dsp->substream[2]);
	if (!period_bytes) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_2,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	/* check if hw has space for one period_size */
	if (snd_pcm_capture_hw_avail(runtime) <= runtime->period_size) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_2,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	if (rt5514_dsp->buf_size[2] % period_bytes)
		rt5514_dsp->buf_size[2] =
			(rt5514_dsp->buf_size[2] / period_bytes) * period_bytes;

	rt5514_spi_burst_read(rt5514_dsp->buf_rp_addr[2], (u8 *)&buf,
		sizeof(buf));
	cur_wp = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
	if ((cur_wp & 0xffe00000) != 0x4fe00000) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_2,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	if (cur_wp >= rt5514_dsp->buf_rp[2])
		remain_data = (cur_wp - rt5514_dsp->buf_rp[2]);
	else
		remain_data =
			(rt5514_dsp->buf_limit[2] - rt5514_dsp->buf_rp[2]) +
			(cur_wp - rt5514_dsp->buf_base[2]);

	if (remain_data < period_bytes) {
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		schedule_delayed_work(&rt5514_dsp->copy_work_2,
			msecs_to_jiffies(COPY_WORK_DELAY_TIME_MS));
		goto done;
	}

	if (rt5514_dsp->buf_rp[2] + period_bytes <= rt5514_dsp->buf_limit[2]) {
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[2],
			runtime->dma_area + rt5514_dsp->dma_offset[2],
			period_bytes);

		if (rt5514_dsp->buf_rp[2] + period_bytes ==
			rt5514_dsp->buf_limit[2])
			rt5514_dsp->buf_rp[2] = rt5514_dsp->buf_base[2];
		else
			rt5514_dsp->buf_rp[2] += period_bytes;
	} else {
		truncated_bytes =
			rt5514_dsp->buf_limit[2] - rt5514_dsp->buf_rp[2];
		rt5514_spi_burst_read(rt5514_dsp->buf_rp[2],
			runtime->dma_area + rt5514_dsp->dma_offset[2],
			truncated_bytes);

		rt5514_spi_burst_read(rt5514_dsp->buf_base[2],
			runtime->dma_area + rt5514_dsp->dma_offset[2] +
			truncated_bytes, period_bytes - truncated_bytes);

		rt5514_dsp->buf_rp[2] = rt5514_dsp->buf_base[2] + period_bytes -
			truncated_bytes;
	}

	rt5514_dsp->dma_offset[2] += period_bytes;
	if (rt5514_dsp->dma_offset[2] >= runtime->dma_bytes)
		rt5514_dsp->dma_offset[2] = 0;

	snd_pcm_period_elapsed(rt5514_dsp->substream[2]);

	schedule_delayed_work(&rt5514_dsp->copy_work_2, msecs_to_jiffies(0));

done:
	mutex_unlock(&rt5514_dsp->dma_lock);
}

static void rt5514_schedule_copy(struct rt5514_dsp *rt5514_dsp, bool is_adc)
{
	u8 buf[8];
	unsigned int base_addr, limit_addr, truncated_bytes,
			buf_ignore_size = 0;
	unsigned int hotword_flag, musdet_flag, stream_flag;
	int retry_cnt = 0;

	rt5514_spi_request_switch(SPI_SWITCH_MASK_COPY, 1);

	if (is_adc) {
		stream_flag = RT5514_DSP_STREAM_ADC;
		base_addr = RT5514_BUFFER_ADC_BASE;
		limit_addr = RT5514_BUFFER_ADC_LIMIT;
		rt5514_dsp->buf_rp_addr[2] = RT5514_BUFFER_ADC_WP;
		dev_info(rt5514_dsp->dev, "adc is streaming\n");
	} else {
		rt5514_spi_burst_read(RT5514_HOTWORD_FLAG, (u8 *)&buf,
			sizeof(buf));
		hotword_flag = buf[0] | buf[1] << 8 | buf[2] << 16 |
			buf[3] << 24;

		rt5514_spi_burst_read(RT5514_MUSDET_FLAG, (u8 *)&buf,
			sizeof(buf));
		musdet_flag =
			buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;

		if (hotword_flag == 1) {
			stream_flag = RT5514_DSP_STREAM_HOTWORD;
			base_addr = RT5514_BUFFER_VOICE_BASE;
			limit_addr = RT5514_BUFFER_VOICE_LIMIT;
			rt5514_dsp->buf_rp_addr[0] = RT5514_BUFFER_VOICE_WP;
			buf_ignore_size =
				rt5514_dsp->hotword_ignore_ms * 2 * 16;
			memset(buf, 0, sizeof(buf));
			rt5514_spi_burst_write(RT5514_HOTWORD_FLAG, buf, 8);
		} else if (musdet_flag == 1) {
			stream_flag = RT5514_DSP_STREAM_MUSDET;
			base_addr = RT5514_BUFFER_MUSIC_BASE;
			limit_addr = RT5514_BUFFER_MUSIC_LIMIT;
			rt5514_dsp->buf_rp_addr[1] = RT5514_BUFFER_MUSIC_WP;
			buf_ignore_size = rt5514_dsp->musdet_ignore_ms * 16;
			memset(buf, 0, sizeof(buf));
			rt5514_spi_burst_write(RT5514_MUSDET_FLAG, buf, 8);
		} else {
			goto end;
		}

		if (stream_flag == RT5514_DSP_STREAM_HOTWORD) {
			if (!rt5514_dsp->substream[0] ||
				rt5514_dsp->stream_flag[0]) {
				dev_err(rt5514_dsp->dev,
					"No pcm0 substream or it is streaming\n");
				goto end;
			} else {
				rt5514_dsp->stream_flag[0] = stream_flag;
				rt5514_dsp->get_size[0] = 0;
			}
		} else if (stream_flag == RT5514_DSP_STREAM_MUSDET) {
			if (!rt5514_dsp->substream[1] ||
				rt5514_dsp->stream_flag[1]) {
				dev_err(rt5514_dsp->dev,
					"No pcm1 substream or it is streaming\n");
				goto end;
			} else {
				rt5514_dsp->stream_flag[1] = stream_flag;
				rt5514_dsp->get_size[1] = 0;
			}
		} else {
			goto end;
		}
	}

	/**
	 * The address area x1800XXXX is the register address, and it cannot
	 * support spi burst read perfectly. So we use the spi burst read
	 * individually to make sure the data correctly.
	 */

	while (retry_cnt < RT5514_SPI_RETRY_CNT) {
		/* sleep 10 ms if need retry*/
		if (retry_cnt)
			usleep_range(10000, 10010);
		retry_cnt++;

		rt5514_spi_burst_read(base_addr, (u8 *)&buf, sizeof(buf));
		rt5514_dsp->buf_base[stream_flag - 1] =
			buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
		if ((rt5514_dsp->buf_base[stream_flag - 1] & 0xffe00000) !=
			0x4fe00000)
			continue;

		rt5514_spi_burst_read(limit_addr, (u8 *)&buf, sizeof(buf));
		rt5514_dsp->buf_limit[stream_flag - 1] =
			buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
		if ((rt5514_dsp->buf_limit[stream_flag - 1] & 0xffe00000) !=
			0x4fe00000)
			continue;

		if (rt5514_dsp->buf_limit[stream_flag - 1] % 8)
			rt5514_dsp->buf_limit[stream_flag - 1] =
			((rt5514_dsp->buf_limit[stream_flag - 1] / 8) + 1) * 8;

		rt5514_spi_burst_read(rt5514_dsp->buf_rp_addr[stream_flag - 1],
			(u8 *)&buf, sizeof(buf));
		rt5514_dsp->buf_rp[stream_flag - 1] =
			buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
		if ((rt5514_dsp->buf_rp[stream_flag - 1] & 0xffe00000) !=
			0x4fe00000)
			continue;
		else
			break;
	}

	if (retry_cnt == RT5514_SPI_RETRY_CNT) {
		pr_err("%s: Fail for address read", __func__);
		goto end;
	}

	rt5514_dsp->buf_rp[stream_flag - 1] += buf_ignore_size;

	if (rt5514_dsp->buf_rp[stream_flag - 1] >=
		rt5514_dsp->buf_limit[stream_flag - 1]) {
		truncated_bytes = rt5514_dsp->buf_rp[stream_flag - 1] -
			rt5514_dsp->buf_limit[stream_flag - 1];

		rt5514_dsp->buf_rp[stream_flag - 1] =
			rt5514_dsp->buf_base[stream_flag - 1] + truncated_bytes;
	}

	if (rt5514_dsp->buf_rp[stream_flag - 1] % 8)
		rt5514_dsp->buf_rp[stream_flag - 1] =
			(rt5514_dsp->buf_rp[stream_flag - 1] / 8) * 8;

	rt5514_dsp->buf_size[stream_flag - 1] =
		rt5514_dsp->buf_limit[stream_flag - 1] -
		rt5514_dsp->buf_base[stream_flag - 1] - buf_ignore_size;

	if (rt5514_dsp->buf_base[stream_flag - 1] &&
		rt5514_dsp->buf_limit[stream_flag - 1] &&
		rt5514_dsp->buf_rp[stream_flag - 1] &&
		rt5514_dsp->buf_size[stream_flag - 1]) {

		/* switch to off before next spi schedule start*/
		rt5514_spi_request_switch(SPI_SWITCH_MASK_COPY, 0);

		if (is_adc)
			schedule_delayed_work(&rt5514_dsp->copy_work_2,
				msecs_to_jiffies(0));
		else if (stream_flag == RT5514_DSP_STREAM_HOTWORD)
			schedule_delayed_work(&rt5514_dsp->copy_work_0,
				msecs_to_jiffies(0));
		else if (stream_flag == RT5514_DSP_STREAM_MUSDET)
			schedule_delayed_work(&rt5514_dsp->copy_work_1,
				msecs_to_jiffies(0));
		return;
	}
end:
	rt5514_spi_request_switch(SPI_SWITCH_MASK_COPY, 0);
}

static void rt5514_spi_start_work(struct work_struct *work) {
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, start_work.work);
	struct snd_soc_component *component = rt5514_dsp->component;

	__pm_stay_awake(&rt5514_watchdog_ws);
	if (!snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		if (rt5514_watchdog_dbg_info(rt5514_dsp)) {
			if (rt5514_watchdog_handler_cb)
				rt5514_watchdog_handler_cb();
			__pm_relax(&rt5514_watchdog_ws);
			return;
		}
	}
	__pm_relax(&rt5514_watchdog_ws);

	mutex_lock(&rt5514_dsp->dma_lock);
	if (!(rt5514_dsp->substream[0] && rt5514_dsp->substream[0]->pcm) &&
		!(rt5514_dsp->substream[1] && rt5514_dsp->substream[1]->pcm)) {
		mutex_unlock(&rt5514_dsp->dma_lock);
		return;
	}
	mutex_unlock(&rt5514_dsp->dma_lock);

	rt5514_schedule_copy(rt5514_dsp, false);
}

static void rt5514_spi_adc_start(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, adc_work.work);
	struct snd_soc_component *component = rt5514_dsp->component;

	mutex_lock(&rt5514_dsp->dma_lock);
	if (!(rt5514_dsp->substream[2] && rt5514_dsp->substream[2]->pcm)) {
		mutex_unlock(&rt5514_dsp->dma_lock);
		return;
	}
	mutex_unlock(&rt5514_dsp->dma_lock);

	if (!snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0))
		rt5514_schedule_copy(rt5514_dsp, true);
}

static irqreturn_t rt5514_spi_irq(int irq, void *data)
{
	struct rt5514_dsp *rt5514_dsp = data;

	pm_wakeup_event(rt5514_dsp->dev, WAKEUP_TIMEOUT);
	cancel_delayed_work_sync(&rt5514_dsp->start_work);
	schedule_delayed_work(&rt5514_dsp->start_work, msecs_to_jiffies(0));

	return IRQ_HANDLED;
}

static irqreturn_t rt5514_ack_irq(int irq, void *data)
{
	complete(&switch_ack);
	disable_irq_nosync(handshake_ack_irq);

	return IRQ_HANDLED;
}

/* PCM for streaming audio from the DSP buffer */
static int rt5514_spi_pcm_open(struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rt5514_spi_pcm_hardware);

	return 0;
}

static int rt5514_spi_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct rt5514_dsp *rt5514_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	if (cpu_dai->id > 2)
		return 0;

	mutex_lock(&rt5514_dsp->dma_lock);
	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
			params_buffer_bytes(hw_params));
	rt5514_dsp->substream[cpu_dai->id] = substream;
	rt5514_dsp->dma_offset[cpu_dai->id] = 0;

	if (cpu_dai->id == 2)
		schedule_delayed_work(&rt5514_dsp->adc_work,
			msecs_to_jiffies(0));

	mutex_unlock(&rt5514_dsp->dma_lock);

	return ret;
}

static int rt5514_spi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct rt5514_dsp *rt5514_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);

	if (cpu_dai->id > 2)
		return 0;

	mutex_lock(&rt5514_dsp->dma_lock);
	rt5514_dsp->substream[cpu_dai->id] = NULL;
	mutex_unlock(&rt5514_dsp->dma_lock);

	switch (cpu_dai->id) {
	case 1:
		cancel_delayed_work_sync(&rt5514_dsp->copy_work_1);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_1, 0);
		break;

	case 2:
		dev_info(rt5514_dsp->dev, "adc stream turns off\n");
		cancel_delayed_work_sync(&rt5514_dsp->copy_work_2);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_2, 0);
		break;

	default:
		cancel_delayed_work_sync(&rt5514_dsp->copy_work_0);
		rt5514_spi_request_switch(SPI_SWITCH_MASK_WORK_0, 0);
		break;
	}

	if (cpu_dai->id < MAX_STREAM_FLAG)
		rt5514_dsp->stream_flag[cpu_dai->id] = RT5514_DSP_NO_STREAM;

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static snd_pcm_uframes_t rt5514_spi_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_platform_get_drvdata(rtd->platform);

	return bytes_to_frames(runtime, rt5514_dsp->dma_offset[cpu_dai->id]);
}

static const struct snd_pcm_ops rt5514_spi_pcm_ops = {
	.open		= rt5514_spi_pcm_open,
	.hw_params	= rt5514_spi_hw_params,
	.hw_free	= rt5514_spi_hw_free,
	.pointer	= rt5514_spi_pcm_pointer,
	.mmap		= snd_pcm_lib_mmap_vmalloc,
	.page		= snd_pcm_lib_get_vmalloc_page,
};

static int rt5514_pcm_parse_dp(struct rt5514_dsp *rt5514_dsp,
	struct device *dev)
{
	unsigned int val = ~0;

	regmap_read(rt5514_g_i2c_regmap, 0x18002ff0, &val);

	if (val == 0x80) {
		device_property_read_u32(dev, "realtek,musdet-ignore-ms-5514p",
			&rt5514_dsp->musdet_ignore_ms);
		device_property_read_u32(dev, "realtek,hotword-ignore-ms-5514p",
			&rt5514_dsp->hotword_ignore_ms);
		device_property_read_u32(dev, "realtek,spi-max-frequency-5514p",
			&rt5514_spi->max_speed_hz);
		spi_setup(rt5514_spi);
	} else {
		device_property_read_u32(dev, "realtek,musdet-ignore-ms",
			&rt5514_dsp->musdet_ignore_ms);
		device_property_read_u32(dev, "realtek,hotword-ignore-ms",
			&rt5514_dsp->hotword_ignore_ms);
		device_property_read_u32(dev, "realtek,spi-max-frequency-5514",
			&rt5514_spi->max_speed_hz);
		spi_setup(rt5514_spi);
	}

	return 0;
}

static int rt5514_spi_pcm_probe(struct snd_soc_platform *platform)
{
	struct rt5514_dsp *rt5514_dsp;
	int ret;

	if (!rt5514_g_i2c_regmap)
		return -EPROBE_DEFER;

	rt5514_dsp = devm_kzalloc(platform->dev, sizeof(*rt5514_dsp),
			GFP_KERNEL);

	rt5514_g_dsp = rt5514_dsp;

	rt5514_pcm_parse_dp(rt5514_dsp, &rt5514_spi->dev);

	rt5514_dsp->dev = &rt5514_spi->dev;

	rt5514_g_dsp = rt5514_dsp;

	rt5514_dsp->component = &platform->component;
	mutex_init(&rt5514_dsp->dma_lock);
	INIT_DELAYED_WORK(&rt5514_dsp->copy_work_0, rt5514_spi_copy_work_0);
	INIT_DELAYED_WORK(&rt5514_dsp->copy_work_1, rt5514_spi_copy_work_1);
	INIT_DELAYED_WORK(&rt5514_dsp->copy_work_2, rt5514_spi_copy_work_2);
	INIT_DELAYED_WORK(&rt5514_dsp->start_work, rt5514_spi_start_work);
	INIT_DELAYED_WORK(&rt5514_dsp->adc_work, rt5514_spi_adc_start);
	INIT_DELAYED_WORK(&rt5514_dsp->chre_chk_work, rt5514_chre_check_work);
	snd_soc_platform_set_drvdata(platform, rt5514_dsp);

	if (rt5514_spi->irq) {
		ret = devm_request_threaded_irq(&rt5514_spi->dev,
			rt5514_spi->irq, NULL, rt5514_spi_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rt5514-spi",
			rt5514_dsp);
		if (ret)
			dev_err(&rt5514_spi->dev,
				"%s Failed to reguest IRQ: %d\n", __func__,
				ret);
	}

#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
	codec_detect_state_callback(rt5514_codec_state);
	codec_detect_number_callback(rt5514_codec_hwinfo);
#endif

	return 0;
}

static struct snd_soc_platform_driver rt5514_spi_platform = {
	.probe = rt5514_spi_pcm_probe,
	.ops = &rt5514_spi_pcm_ops,
};

static const struct snd_soc_component_driver rt5514_spi_dai_component = {
	.name		= "rt5514-spi-dai",
	.probe = rt5514_spi_fe_dai_probe,
	.dapm_widgets = rt5514_spi_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5514_spi_dapm_widgets),
	.dapm_routes = intercon_common,
	.num_dapm_routes = ARRAY_SIZE(intercon_common),
};

/**
 * rt5514_spi_burst_read - Read data from SPI by rt5514 address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_READ;
	int status;
	u8 *write_buf;
	u8 *read_buf;
	unsigned int i, end, offset = 0;
	struct spi_message message;
	struct spi_transfer x[3];
	struct snd_soc_component *component = rt5514_g_dsp->component;

	mutex_lock(&spi_lock);
	__pm_stay_awake(&rt5514_spi_ws);

	if (snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		dev_err(rt5514_g_dsp->dev, "%s: Request in suspend\n",
			__func__);
		__pm_relax(&rt5514_spi_ws);
		mutex_unlock(&spi_lock);
		return false;
	}

	write_buf = kzalloc(8, GFP_DMA | GFP_KERNEL);
	read_buf = kzalloc(RT5514_SPI_BUF_LEN, GFP_DMA | GFP_KERNEL);

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 5;
		x[0].tx_buf = write_buf;
		spi_message_add_tail(&x[0], &message);

		x[1].len = 4;
		x[1].tx_buf = write_buf;
		spi_message_add_tail(&x[1], &message);

		x[2].len = end;
		x[2].rx_buf = read_buf;
		spi_message_add_tail(&x[2], &message);

		status = spi_sync(rt5514_spi, &message);

		if (status) {
			kfree(read_buf);
			kfree(write_buf);
			__pm_relax(&rt5514_spi_ws);
			mutex_unlock(&spi_lock);
			return false;
		}

		memcpy(rxbuf + offset, read_buf, end);

		offset += RT5514_SPI_BUF_LEN;
	}

	for (i = 0; i < len; i += 8) {
		write_buf[0] = rxbuf[i + 0];
		write_buf[1] = rxbuf[i + 1];
		write_buf[2] = rxbuf[i + 2];
		write_buf[3] = rxbuf[i + 3];
		write_buf[4] = rxbuf[i + 4];
		write_buf[5] = rxbuf[i + 5];
		write_buf[6] = rxbuf[i + 6];
		write_buf[7] = rxbuf[i + 7];

		rxbuf[i + 0] = write_buf[7];
		rxbuf[i + 1] = write_buf[6];
		rxbuf[i + 2] = write_buf[5];
		rxbuf[i + 3] = write_buf[4];
		rxbuf[i + 4] = write_buf[3];
		rxbuf[i + 5] = write_buf[2];
		rxbuf[i + 6] = write_buf[1];
		rxbuf[i + 7] = write_buf[0];
	}

	kfree(read_buf);
	kfree(write_buf);

	__pm_relax(&rt5514_spi_ws);
	mutex_unlock(&spi_lock);

	return true;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_read);

/**
 * rt5514_spi_burst_write - Write data to SPI by rt5514 address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_WRITE;
	u8 *write_buf;
	unsigned int i, j, end, offset = 0;
	struct snd_soc_component *component = rt5514_g_dsp->component;

	write_buf = kzalloc(RT5514_SPI_BUF_LEN + 6, GFP_DMA | GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	mutex_lock(&spi_lock);
	__pm_stay_awake(&rt5514_spi_ws);

	if (snd_power_wait(component->card->snd_card, SNDRV_CTL_POWER_D0)) {
		dev_err(rt5514_g_dsp->dev, "%s: Request in suspend\n",
			__func__);
		kfree(write_buf);
		__pm_relax(&rt5514_spi_ws);
		mutex_unlock(&spi_lock);
		return -ETIMEDOUT;
	}

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			for (j = 0; j < 8; j++) {
				if ((offset + i + j) < len) {
					write_buf[i + 12 - j] =
						txbuf[offset + i + j];
				} else {
					break;
				}
			}
		}

		if (end % 8)
			end = (end / 8 + 1) * 8;

		write_buf[end + 5] = spi_cmd;

		spi_write(rt5514_spi, write_buf, end + 6);

		offset += RT5514_SPI_BUF_LEN;
	}

	kfree(write_buf);

	__pm_relax(&rt5514_spi_ws);
	mutex_unlock(&spi_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_write);

static int rt5514_spi_probe(struct spi_device *spi)
{
	int ret;
	struct device_node *np = spi->dev.of_node;
	u32 SPI_hosts_Ctl_enable = 0;

	rt5514_spi = spi;
	mutex_init(&spi_lock);
	mutex_init(&switch_lock);
	wakeup_source_init(&rt5514_spi_ws, "rt5514-spi");
	wakeup_source_init(&rt5514_watchdog_ws, "rt5514-watchdog");

	ret = devm_snd_soc_register_platform(&spi->dev, &rt5514_spi_platform);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register platform.\n");
		return ret;
	}

	ret = devm_snd_soc_register_component(&spi->dev,
					      &rt5514_spi_dai_component,
					      rt5514_spi_dai,
					      ARRAY_SIZE(rt5514_spi_dai));
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register component.\n");
		return ret;
	}

	spi_switch_mask = 0;

	device_init_wakeup(&spi->dev, true);

	handshake_gpio = of_get_named_gpio(np, "handshake-gpio", 0);

	if (!gpio_is_valid(handshake_gpio)) {
		dev_err(&rt5514_spi->dev, "Look %s property %s fail on %d\n",
			"handshake-gpio", np->full_name, handshake_gpio);
		goto no_handshake;
	} else {
		dev_info(&rt5514_spi->dev, "handshake gpio %d property %s\n",
			handshake_gpio, np->full_name);
	}

	ret = gpio_request(handshake_gpio, "handshake-gpio");

	if (ret) {
		dev_err(&rt5514_spi->dev,
			"%s Failed to reguest handshake GPIO: %d\n", __func__,
			ret);
		goto no_handshake;
	} else
		gpio_direction_output(handshake_gpio, 1);

	handshake_ack_irq =
		of_irq_get_byname(rt5514_spi->dev.of_node, "handshake-ack");

	if (handshake_ack_irq < 0) {
		dev_err(&rt5514_spi->dev, "failed to get spi switch ack irq\n");
		goto no_handshake;
	} else {
		init_completion(&switch_ack);
		ret = devm_request_threaded_irq(&rt5514_spi->dev,
						handshake_ack_irq, NULL,
						rt5514_ack_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"rt5514-switch-ack", NULL);
		if (ret) {
			dev_err(&rt5514_spi->dev,
				"%s Failed to reguest ack IRQ: %d\n", __func__,
				ret);
			goto no_handshake;
		}
	}

	device_property_read_u32(&spi->dev, "realtek,en-2hosts",
				&SPI_hosts_Ctl_enable);

	if (!SPI_hosts_Ctl_enable)
		goto no_handshake;

	dev_info(&spi->dev, " rt5514-spi register component success.\n");
	return 0;

no_handshake:
	rt5514_spi_request_switch(SPI_SWITCH_MASK_NO_IRQ, 1);
	dev_info(&spi->dev, " rt5514-spi init success without handshake\n");
	return 0;
}

static int rt5514_suspend(struct device *dev)
{
	int irq = to_spi_device(dev)->irq;

	if (device_may_wakeup(dev))
		enable_irq_wake(irq);

	return 0;
}

static int rt5514_resume(struct device *dev)
{
	int irq = to_spi_device(dev)->irq;

	if (device_may_wakeup(dev))
		disable_irq_wake(irq);

	return 0;
}

static const struct dev_pm_ops rt5514_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rt5514_suspend, rt5514_resume)
};

static const struct of_device_id rt5514_of_match[] = {
	{ .compatible = "realtek,rt5514", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5514_of_match);

static struct spi_driver rt5514_spi_driver = {
	.driver = {
		.name = "rt5514",
		.pm = &rt5514_pm_ops,
		.of_match_table = of_match_ptr(rt5514_of_match),
	},
	.probe = rt5514_spi_probe,
};
module_spi_driver(rt5514_spi_driver);

MODULE_DESCRIPTION("RT5514 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
