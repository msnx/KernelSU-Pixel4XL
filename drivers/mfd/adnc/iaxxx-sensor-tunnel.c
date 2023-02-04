/*
 * iaxxx-sensor-tunnel.c -- functions related to sensor control though tunnel
 *
 * Copyright 2018 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pad-ctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-gpio.h>
#include <linux/mfd/adnc/iaxxx-register-defs-sensor-header.h>
#include <linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/mfd/adnc/iaxxx-sensor-tunnel.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/uaccess.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-tunnel-priv.h"

#define ENCODING_Q23	0x17
#define ENCODING_AFLOAT	0x1
#define K8_10MS	0x140
#define RATE_8K	0x0
#define REDBOX_2_1	0x2
#define ASRC_ENABLE	0x0
#define RETRY_COUNT	5
#define I2S3_HL_DEVIDER_CLK_PERIOD	0x12
#define I2S3_NR_DIVIDER_VALUE	0x3000001
#define I2S3_CLK_CFG_VALUE		0x1017F6
#define I2S3_CLK_CTRL_VALUE		0x103
#define I2S3_FS_ALIGN_VALUE		0x0
#define CNR0_CIC_RX_2_3_VALUE	0x3c0000
#define CNR0_HB_CIC_RX_3_POS	7
#define CNR0_CIC_RX_6_7_VALUE	0x5c0000
#define CNR0_HB_CIC_RX_7_POS	15
#define BUSY_SLEEP_RANGE	2000
#define BUSY_SLEEP_RANGE_MAX	2005

#define IAXXX_IO_CTRL_PORT_CLK_ADDR(port_id) \
	(IAXXX_IO_CTRL_PORTA_CLK_ADDR + port_id * 0x10)

static unsigned int io_ctrl_cfg_value[] = {
	0,	/* PORTA_IOCTRL_CFG_VALUE */
	0x205,	/* PORTB_IOCTRL_CFG_VALUE */
	0,	/* PORTC_IOCTRL_CFG_VALUE */
	0x2005,	/* PORTD_IOCTRL_CFG_VALUE */
	0,	/* PORTE_IOCTRL_CFG_VALUE */
	0,	/* Dummy */
	0,	/* CDC_IOCTRL_CFG_VALUE */
};

enum {
	PDMI_CDC_PDM0 = 0,
	PDMI_CDC_PDM1,
	PDMI_CDC_PDM2,
	PDMI_PORTD_DI,
	PDMI_PORTC_FS,
	PDMI_PORTC_DI,
	PDMI_PORTB_FS,
	PDMI_PORTB_DI,
};

static unsigned int pdmi_port[] = {
	0,		/* PORTA */
	PDMI_PORTB_DI,	/* PORTB */
	0,		/* PORTC */
	PDMI_PORTD_DI,	/* PORTD */
	0,		/* PORTE */
	0,		/* dummy */
	0,		/* CDC */
};

static const u32 dmic_busy_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_BUSY_ADDR,
	0,
	0,
	IAXXX_CNR0_CDC0_ENABLE_BUSY_ADDR,
};

static const u32 dmic_enable_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_ADDR,
	0,
	0,
	IAXXX_CNR0_CDC0_ENABLE_ADDR,
};

/*
 *[TODO]This is a first drop for testing flicker sensor.
 * Because the scenario is under discussing, It will refine later
 * b/113305927 will trace the progress.
 */
static int sensor_tunnel_route_setup(struct iaxxx_priv *priv,
				bool enable)
{
	int ret = 0;
	u32 status;
	int try = RETRY_COUNT;
	unsigned int port_id, strm_id = 0x7, chn_id = 0xF;

	if (priv == NULL)
		return -EINVAL;

	port_id = priv->sensor_port;
	if (port_id != PDM_PORTB && port_id != PDM_PORTD) {
		pr_err("%s Invalid sensor port %d\n", __func__, port_id);
		return -EINVAL;
	}

	if (enable) { /* Set up the route */

		ret = iaxxx_check_and_power_up_ssp(priv);
		if (ret) {
			dev_err(priv->dev, "Failed to power up SSP core in %s\n",
					__func__);
			return ret;
		}

		regmap_update_bits(priv->regmap, IAXXX_SRB_PORTB_DDR_ADDR,
				IAXXX_SRB_PORTB_DDR_COMMF_2_MASK,
				(0x1 << IAXXX_SRB_PORTB_DDR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_SRB_PORTB_DR_ADDR,
				IAXXX_SRB_PORTB_DR_COMMF_2_MASK,
				(0x1 << IAXXX_SRB_PORTB_DR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port_id), (0x1 << port_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			ret = -EIO;
			goto exit;
		}

		if (port_id == PDM_PORTD) {
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTD_CLK_ADDR,
					IAXXX_PAD_CTRL_PORTD_CLK_RESET_VAL);
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTD_DI_ADDR,
					IAXXX_PAD_CTRL_PORTD_DI_RESET_VAL);
		} else if (port_id == PDM_PORTB) {
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTB_CLK_ADDR,
					IAXXX_PAD_CTRL_PORTB_CLK_RESET_VAL);
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTB_DI_ADDR,
					IAXXX_PAD_CTRL_PORTB_DI_RESET_VAL);
		}

		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << port_id), IAXXX_CNR0_I2S_ENABLE_LOW);
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_HL_ADDR(port_id), 0x0);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_HL_ADDR(port_id),
				I2S3_HL_DEVIDER_CLK_PERIOD);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_NR_ADDR(port_id), 0x0);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_NR_ADDR(port_id),
				I2S3_NR_DIVIDER_VALUE);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_GEN_CFG_ADDR(port_id),
				I2S3_CLK_CFG_VALUE);
		regmap_update_bits(priv->regmap,
				IAXXX_I2S_I2S_CLK_CTRL_ADDR(port_id),
				IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL,
				I2S3_CLK_CTRL_VALUE);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port_id),
				I2S3_FS_ALIGN_VALUE);
		regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR,
				(1 << port_id), (1 << port_id));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << port_id), (0x1 << port_id));
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_write(priv->regmap, IAXXX_IO_CTRL_PORT_CLK_ADDR(port_id),
				io_ctrl_cfg_value[port_id]);
		regmap_update_bits(priv->regmap,
				IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
				(0x1 << pdmi_port[port_id]),
				(0x1 << pdmi_port[port_id]));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			ret = -EIO;
			goto exit;
		}
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_HOS_ADDR,
				(0x1 << pdmi_port[port_id]),
				(0x1 << pdmi_port[port_id]));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_ADTL_CTRL_ADDR,
				IAXXX_CIC_ADTL_RX_MASK(pdmi_port[port_id]), 0);
		if (port_id == PDM_PORTD) {
			/* PORTD_DI: PDMI3 */
			pr_info("%s enable port D\n", __func__);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				(1 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_M_3_MASK,
				CNR0_CIC_RX_2_3_VALUE);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_CTRL_ADDR,
				(0x1 << pdmi_port[port_id]),
				(0x1 << pdmi_port[port_id]));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_HB_ADDR,
				IAXXX_CNR0_CIC_HB_CIC_RX_MASK(port_id),
				0x1 << CNR0_HB_CIC_RX_3_POS);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_MASK,
				(0x1 <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_IO_CTRL_PORTD_DI_ADDR,
				IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_MASK,
				(0x1 <<
				IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				(0 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS));
		} else if (port_id == PDM_PORTB) {
			/* PORTB_DI: PDMI7 */
			pr_info("%s enable port B PDMI7\n", __func__);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_6_7_ADDR,
				IAXXX_CNR0_CIC_RX_6_7_CLR_7_MASK,
				(1 << IAXXX_CNR0_CIC_RX_6_7_CLR_7_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_6_7_ADDR,
				IAXXX_CNR0_CIC_RX_6_7_M_7_MASK,
				CNR0_CIC_RX_6_7_VALUE);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_CTRL_ADDR,
				(0x1 << pdmi_port[port_id]),
				(0x1 << pdmi_port[port_id]));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_HB_ADDR,
				IAXXX_CNR0_CIC_HB_CIC_RX_MASK(
					pdmi_port[port_id]),
				0x1 << CNR0_HB_CIC_RX_7_POS);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_7_MASK,
				(0x1 <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_7_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_7_MASK,
				(0x1 <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_7_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_IO_CTRL_PORTB_DI_ADDR,
				IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK,
				(0x1 <<
				IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_6_7_ADDR,
				IAXXX_CNR0_CIC_RX_6_7_CLR_7_MASK,
				(0 << IAXXX_CNR0_CIC_RX_6_7_CLR_7_POS));
		}

		do {
			usleep_range(BUSY_SLEEP_RANGE, BUSY_SLEEP_RANGE_MAX);
			regmap_read(priv->regmap,
				dmic_busy_addr[port_id], &status);
		} while (status && (try-- != 0));

		regmap_update_bits(priv->regmap, dmic_enable_addr[port_id],
				(0x1 << pdmi_port[port_id]),
				(0x1 << pdmi_port[port_id]));

		try = RETRY_COUNT;
		do {
			usleep_range(BUSY_SLEEP_RANGE, BUSY_SLEEP_RANGE_MAX);
			regmap_read(priv->regmap,
				dmic_busy_addr[port_id], &status);
		} while (status && (try-- != 0));

		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			ret = -EIO;
			goto exit;
		}

		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_CTRL_REG(strm_id),
				IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK,
				ASRC_ENABLE <<
				IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_SYNC_REG(strm_id),
				IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK,
				strm_id);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK,
				K8_10MS <<
				IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_ENCODING_MASK,
				ENCODING_AFLOAT <<
				IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK,
				RATE_8K  <<
				IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS);
		if (port_id == PDM_PORTD) {
			regmap_update_bits(priv->regmap,
					IAXXX_STR_GRP_STR_PORT_REG(strm_id),
					IAXXX_STR_GRP_STR_PORT_PORT_MASK,
					IAXXX_SYSID_PDMI3 <<
					IAXXX_STR_GRP_STR_PORT_PORT_POS);
		} else if (port_id == PDM_PORTB) {
			regmap_update_bits(priv->regmap,
					IAXXX_STR_GRP_STR_PORT_REG(strm_id),
					IAXXX_STR_GRP_STR_PORT_PORT_MASK,
					IAXXX_SYSID_PDMI7 <<
					IAXXX_STR_GRP_STR_PORT_PORT_POS);
		}
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_PORT_REG(strm_id),
				IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_MASK,
				ENCODING_Q23  <<
				IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_CHN_REG(strm_id),
				(0x1 << chn_id), (0x1 << chn_id));
		regmap_update_bits(priv->regmap, IAXXX_STR_HDR_STR_EN_ADDR,
				(0x1 << strm_id), (0x1 << strm_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			ret = -EIO;
			goto exit;
		}
		/* Setup VSYNC Sensor route */
		regmap_update_bits(priv->regmap,
				IAXXX_SENSOR_HDR_SENSOR_ENABLE_ADDR,
				IAXXX_SENSOR_HDR_SENSOR_ENABLE_1_REG_MASK,
				(0x1
				 << IAXXX_SENSOR_HDR_SENSOR_ENABLE_1_REG_POS));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up VSYNC Sensor  route\n");
			ret = -EIO;
			goto exit;
		}

	} else {
		regmap_update_bits(priv->regmap, IAXXX_SRB_PORTB_DDR_ADDR,
				IAXXX_SRB_PORTB_DDR_COMMF_2_MASK,
				(0x0 << IAXXX_SRB_PORTB_DDR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_SRB_PORTB_DR_ADDR,
				IAXXX_SRB_PORTB_DR_COMMF_2_MASK,
				(0x0 << IAXXX_SRB_PORTB_DR_COMMF_2_POS));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			ret = -EIO;
			goto exit;
		}
		regmap_update_bits(priv->regmap, IAXXX_STR_HDR_STR_EN_ADDR,
				(0x1 << strm_id), 0x0);
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM  route\n");
			ret = -EIO;
			goto exit;
		}

		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_HOS_ADDR,
				(0x1 << pdmi_port[port_id]),
				(0x0 << pdmi_port[port_id]));

		if (port_id == PDM_PORTD) {
			pr_info("%s disable port D\n", __func__);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				0x1 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_MASK,
				(0x0 <<
				 IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_POS));
		} else if (port_id == PDM_PORTB) {
			pr_info("%s disable port B PDMI7\n", __func__);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_6_7_ADDR,
				IAXXX_CNR0_CIC_RX_6_7_CLR_7_MASK,
				0x1 << IAXXX_CNR0_CIC_RX_6_7_CLR_7_POS);
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_7_MASK,
				(0x0 <<
				 IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_7_POS));
			regmap_update_bits(priv->regmap,
				IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_7_MASK,
				(0x0 <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_S_7_POS));
		}
		regmap_update_bits(priv->regmap, dmic_enable_addr[port_id],
				(0x1 << pdmi_port[port_id]), 0x0);
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error tearing up PDM  route\n");
			ret = -EIO;
			goto exit;
		}
		regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR,
				(1 << port_id), 0x0);
		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << port_id), IAXXX_CNR0_I2S_ENABLE_LOW);
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port_id), (0x0 << port_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error tearing up PDM  route\n");
			ret = -EIO;
			goto exit;
		}

		regmap_update_bits(priv->regmap,
				IAXXX_SENSOR_HDR_SENSOR_ENABLE_ADDR,
				IAXXX_SENSOR_HDR_SENSOR_ENABLE_1_REG_MASK,
				(0x0
				 << IAXXX_SENSOR_HDR_SENSOR_ENABLE_1_REG_POS));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in tearing VSYNC Sensor  route\n");
			ret = -EIO;
			goto exit;
		}

		if (port_id == PDM_PORTD) {
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTD_CLK_ADDR,
					IAXXX_PAD_CTRL_PORTD_CLK_LOW_PWR);
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTD_DI_ADDR,
					IAXXX_PAD_CTRL_PORTD_DI_LOW_PWR);
		} else if (port_id == PDM_PORTB) {
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTB_CLK_ADDR,
					IAXXX_PAD_CTRL_PORTB_CLK_LOW_PWR);
			regmap_write(priv->regmap,
					IAXXX_PAD_CTRL_PORTB_DI_ADDR,
					IAXXX_PAD_CTRL_PORTB_DI_LOW_PWR);
		}

	}

exit:
	return ret;
}

static int sensor_tunnel_open(struct inode *inode, struct file *filp)
{
	int rc;

	rc = iaxxx_tunnel_open_common(inode, filp, TUNNEL_1);
	return rc;
}


static int sensor_tunnel_release(struct inode *inode, struct file *filp)
{
	int rc;

	rc = iaxxx_tunnel_release_common(inode, filp, TUNNEL_1);
	return rc;
}

static ssize_t sensor_tunnel_read(struct file *filp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	return iaxxx_tunnel_read(filp, buf, count);
}

static long sensor_tunnel_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct iaxxx_tunnel_client * const client =
			(struct iaxxx_tunnel_client *)filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct tunlMsg msg;
	int ret = 0;
	uint32_t status;

	if (!priv) {
		pr_err("Unable to fetch tunnel private data\n");
		return -EINVAL;
	}

	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}

	if (!iaxxx_is_firmware_ready(priv)) {
		dev_err(priv->dev, "%s FW  is not in App mode\n", __func__);
		return -EIO;
	}
	if (arg != 0 && _IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE) &&
		_IOC_SIZE(cmd) == sizeof(struct tunlMsg)) {
		if (copy_from_user(&msg, (void __user *)arg,
					sizeof(struct tunlMsg))) {
			pr_err("parameter copy from user failed\n");
			return -EFAULT;
		}

		/* validate the tunnel parameters */
		if (msg.tunlSrc > (IAXXX_OUT_TNL_GRP_CONNECT_SOURCE_ID_MASK >>
				IAXXX_OUT_TNL_GRP_CONNECT_SOURCE_ID_POS) ||
			msg.tunlEncode >
			(IAXXX_OUT_TNL_GRP_TNL_CTRL_OUTPUT_ENCODING_MASK >>
			IAXXX_OUT_TNL_GRP_TNL_CTRL_OUTPUT_ENCODING_POS) ||
			msg.tunlMode > (IAXXX_OUT_TNL_GRP_TNL_CTRL_MODE_MASK >>
					IAXXX_OUT_TNL_GRP_TNL_CTRL_MODE_POS)) {
			pr_err("invalid tunnel parameter received\n");
			return -EINVAL;
		}

		pr_debug("cmd: %x, TunlSrc: %x, tunlMode: %x, tunlEncode: %x",
			cmd, (uint32_t)msg.tunlSrc, (uint32_t)msg.tunlMode,
			(uint32_t)msg.tunlEncode);
	}

	switch (cmd) {

	case FLICKER_ROUTE_SETUP:
		status = atomic_add_unless(&priv->fli_route_status, 1, 1);

		if (status) {
			mutex_lock(&priv->sensor_tunnel_dev_lock);
			ret = sensor_tunnel_route_setup(priv, true);
			mutex_unlock(&priv->sensor_tunnel_dev_lock);

			if (ret) {
				pr_err("Unable to setup sensor route\n");
				return -EIO;
			}
		}
		break;

	case FLICKER_TUNNEL_SETUP:

		ret = iaxxx_tunnel_setup(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to setup tunnel");
			return	-EINVAL;
		}
		break;

	case FLICKER_TUNNEL_TERMINATE:

		ret = iaxxx_tunnel_term(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to terminate tunnel");
			ret = -EINVAL;
		}

		break;

	case FLICKER_ROUTE_TERMINATE:
		status = atomic_add_unless(&priv->fli_route_status, -1, 0);

		if (status) {
			mutex_lock(&priv->sensor_tunnel_dev_lock);
			ret = sensor_tunnel_route_setup(priv, false);
			mutex_unlock(&priv->sensor_tunnel_dev_lock);
			if (ret) {
				pr_err("Unable to setup sensor route\n");
				return -EIO;
			}
		}
		break;

	default:
		pr_err("Invalid ioctl command received %x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sensor_tunnel_compat_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	return sensor_tunnel_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations sensor_tunnel_fops = {
	.owner = THIS_MODULE,
	.open = sensor_tunnel_open,
	.read = sensor_tunnel_read,
	.unlocked_ioctl	= sensor_tunnel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sensor_tunnel_compat_ioctl,
#endif
	.release = sensor_tunnel_release,
};

int iaxxx_sensor_tunnel_init(struct iaxxx_priv *priv)
{
	int err;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (priv == NULL) {
		pr_err("priv is NULL in %s\n", __func__);
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;
	err = iaxxx_cdev_create(&t_intf_priv->tunnel_sensor_cdev,
				t_intf_priv->dev,
				&sensor_tunnel_fops, t_intf_priv,
				IAXXX_CDEV_SENSOR_TUNNEL_DEV);
	if (err) {
		pr_err("Error in creating the char device %s\n", __func__);
		return -EIO;
	}
	return 0;
}

int iaxxx_sensor_tunnel_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;
	iaxxx_cdev_destroy(&t_intf_priv->tunnel_sensor_cdev);
	return 0;
}
