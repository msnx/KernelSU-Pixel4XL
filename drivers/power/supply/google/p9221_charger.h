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
#ifndef __P9221_CHARGER_H__
#define __P9221_CHARGER_H__

#define P9221_WLC_VOTER				"WLC_VOTER"
#define P9221_USER_VOTER			"WLC_USER_VOTER"
#define P9221_OCP_VOTER				"OCP_VOTER"
#define P9382A_LOW_POWER_TX_VOTER		"LOW_POWER_TX_VOTER"
#define P9221_DC_ICL_BPP_UA			700000
#define P9221_DC_ICL_BPP_RAMP_DEFAULT_UA	900000
#define P9221_DC_ICL_BPP_RAMP_DELAY_DEFAULT_MS	(7 * 60 * 1000)  /* 7 mins */
#define P9221_DC_ICL_EPP_UA			1100000
#define P9221_DC_ICL_LOW_POWER_UA		600000
#define P9221_EPP_THRESHOLD_UV			7000000
#define P9221_MAX_VOUT_SET_MV_DEFAULT		9000

#define P9221_DEFAULT_VOTER			"DEFAULT_VOTER"

#define P9221R5_EPP_NEGOTIATED_POWER_10W	0x14
#define P9221R5_VOUT_SET_9V			0x5A
#define P9221R5_EPP_NEGOTIATED_RNTXCAPABREQ	BIT(0)
#define P9221R5_EPP_NEGOTIATED_RNDONE		BIT(1)
#define P9221R5_EPP_NEGOTIATED_RNERROR		BIT(2)

/*
 * P9221 common registers
 */
#define P9221_CHIP_ID_REG			0x00
#define P9221_CHIP_ID				0x9220
#define P9221_CHIP_REVISION_REG			0x02
#define P9221_CUSTOMER_ID_REG			0x03
#define P9221R5_CUSTOMER_ID_VAL			0x05
#define P9221_OTP_FW_MAJOR_REV_REG		0x04
#define P9221_OTP_FW_MINOR_REV_REG		0x06
#define P9221_OTP_FW_DATE_REG			0x08
#define P9221_OTP_FW_DATE_SIZE			12
#define P9221_OTP_FW_TIME_REG			0x14
#define P9221_OTP_FW_TIME_SIZE			8
#define P9221_SRAM_FW_MAJOR_REV_REG		0x1C
#define P9221_SRAM_FW_MINOR_REV_REG		0x1E
#define P9221_SRAM_FW_DATE_REG			0x20
#define P9221_SRAM_FW_DATE_SIZE			12
#define P9221_SRAM_FW_TIME_REG			0x2C
#define P9221_SRAM_FW_TIME_SIZE			8
#define P9221_STATUS_REG			0x34
#define P9221_INT_REG				0x36
#define P9221_INT_MASK				0xF7
#define P9221_INT_ENABLE_REG			0x38
#define P9221_COM_REG				0x4E

/*
 * P9221R5 unique registers
 */
#define P9221R5_INT_CLEAR_REG			0x3A
#define P9221R5_VOUT_SET_REG			0x3C
#define P9221R5_ILIM_SET_REG			0x3D
#define P9221R5_ILIM_SET_MAX			0x0E	/* 0x0E = 1.6A */
#define P9221R5_CHARGE_STAT_REG			0x3E
#define P9221R5_EPT_REG				0x3F
#define P9221R5_VRECT_REG			0x40
#define P9221R5_VOUT_REG			0x42
#define P9221R5_IOUT_REG			0x44
#define P9221R5_OP_FREQ_REG			0x48
#define P9221R5_SYSTEM_MODE_REG			0x4C
#define P9221R5_COM_CHAN_RESET_REG		0x50
#define P9221R5_COM_CHAN_SEND_SIZE_REG		0x58
#define P9221R5_COM_CHAN_SEND_IDX_REG		0x59
#define P9221R5_COM_CHAN_RECV_SIZE_REG		0x5A
#define P9221R5_COM_CHAN_RECV_IDX_REG		0x5B
#define P9221R5_VRECT_ADC_REG			0x60
#define P9221R5_VOUT_ADC_REG			0x62
#define P9221R5_VOUT_ADC_MASK			0xFFF
#define P9221R5_IOUT_ADC_REG			0x64
#define P9221R5_IOUT_ADC_MASK			0xFFF
#define P9221R5_DIE_TEMP_ADC_REG		0x66
#define P9221R5_DIE_TEMP_ADC_MASK		0xFFF
#define P9221R5_AC_PERIOD_REG			0x68
#define P9221R5_TX_PINGFREQ_REG			0x6A
#define P9221R5_EXT_TEMP_REG			0x6C
#define P9221R5_EXT_TEMP_MASK			0xFFF
#define P9221R5_FOD_REG				0x70
#define P9221R5_NUM_FOD				16
#define P9221R5_DEBUG_REG			0x80
#define P9221R5_EPP_Q_FACTOR_REG		0x83
#define P9221R5_EPP_TX_GUARANTEED_POWER_REG	0x84
#define P9221R5_EPP_TX_POTENTIAL_POWER_REG	0x85
#define P9221R5_EPP_TX_CAPABILITY_FLAGS_REG	0x86
#define P9221R5_EPP_RENEGOTIATION_REG		0x87
#define P9221R5_EPP_CUR_RPP_HEADER_REG		0x88
#define P9221R5_EPP_CUR_NEGOTIATED_POWER_REG	0x89
#define P9221R5_EPP_CUR_MAXIMUM_POWER_REG	0x8A
#define P9221R5_EPP_CUR_FSK_MODULATION_REG	0x8B
#define P9221R5_EPP_REQ_RPP_HEADER_REG		0x8C
#define P9221R5_EPP_REQ_NEGOTIATED_POWER_REG	0x8D
#define P9221R5_EPP_REQ_MAXIMUM_POWER_REG	0x8E
#define P9221R5_EPP_REQ_FSK_MODULATION_REG	0x8F
#define P9221R5_VRECT_TARGET_REG		0x90
#define P9221R5_VRECT_KNEE_REG			0x92
#define P9221R5_VRECT_CORRECTION_FACTOR_REG	0x93
#define P9221R5_VRECT_MAX_CORRECTION_FACTOR_REG	0x94
#define P9221R5_VRECT_MIN_CORRECTION_FACTOR_REG	0x96
#define P9221R5_FOD_SECTION_REG			0x99
#define P9221R5_VRECT_ADJ_REG			0x9E
#define P9221R5_ALIGN_X_ADC_REG			0xA0
#define P9221R5_ALIGN_Y_ADC_REG			0xA1
#define P9221R5_ASK_MODULATION_DEPTH_REG	0xA2
#define P9221R5_OVSET_REG			0xA3
#define P9221R5_OVSET_MASK			0x7
#define P9221R5_EPP_TX_SPEC_REV_REG		0xA9
#define P9221R5_EPP_TX_MFG_CODE_REG		0xAA
#define P9221R5_GP0_RESET_VOLT_REG		0xAC
#define P9221R5_GP1_RESET_VOLT_REG		0xAE
#define P9221R5_GP2_RESET_VOLT_REG		0xB0
#define P9221R5_GP3_RESET_VOLT_REG		0xB2
#define P9221R5_PROP_TX_ID_REG			0xB4
#define P9221R5_PROP_TX_ID_SIZE			4
#define P9221R5_DATA_SEND_BUF_START		0x100
#define P9221R5_DATA_SEND_BUF_SIZE		0x80
#define P9221R5_DATA_RECV_BUF_START		0x180
#define P9221R5_DATA_RECV_BUF_SIZE		0x80
#define P9221R5_MAX_PP_BUF_SIZE			16
#define P9221R5_LAST_REG			0x1FF

/*
 * System Mode Mask (R5+/0x4C)
 */
#define P9221R5_SYSTEM_MODE_EXTENDED_MASK	BIT(3)

/*
 * Com Channel Commands
 */
#define P9221R5_COM_CHAN_CCRESET		BIT(7)
#define P9221_COM_CHAN_RETRIES			5

/*
 * End of Power packet types
 */
#define P9221_EOP_UNKNOWN			0x00
#define P9221_EOP_EOC				0x01
#define P9221_EOP_INTERNAL_FAULT		0x02
#define P9221_EOP_OVER_TEMP			0x03
#define P9221_EOP_OVER_VOLT			0x04
#define P9221_EOP_OVER_CURRENT			0x05
#define P9221_EOP_BATT_FAIL			0x06
#define P9221_EOP_RECONFIG			0x07
#define P9221_EOP_NO_RESPONSE			0x08
#define P9221_EOP_NEGOTIATION_FAIL		0x0A
#define P9221_EOP_RESTART_POWER			0x0B

/*
 * Command flags
 */
#define P9221R5_COM_RENEGOTIATE			P9221_COM_RENEGOTIATE
#define P9221R5_COM_SWITCH2RAM			P9221_COM_SWITCH_TO_RAM_MASK
#define P9221R5_COM_CLRINT			P9221_COM_CLEAR_INT_MASK
#define P9221R5_COM_SENDCSP			P9221_COM_SEND_CHG_STAT_MASK
#define P9221R5_COM_SENDEPT			P9221_COM_SEND_EOP_MASK
#define P9221R5_COM_LDOTGL			P9221_COM_LDO_TOGGLE
#define P9221R5_COM_CCACTIVATE			BIT(0)

#define P9221_COM_RENEGOTIATE			BIT(7)
#define P9221_COM_SWITCH_TO_RAM_MASK		BIT(6)
#define P9221_COM_CLEAR_INT_MASK		BIT(5)
#define P9221_COM_SEND_CHG_STAT_MASK		BIT(4)
#define P9221_COM_SEND_EOP_MASK			BIT(3)
#define P9221_COM_LDO_TOGGLE			BIT(1)

/*
 * Interrupt/Status flags for P9221
 */
#define P9221_STAT_VOUT				BIT(7)
#define P9221_STAT_VRECT			BIT(6)
#define P9221_STAT_ACMISSING			BIT(5)
#define P9221_STAT_OV_TEMP			BIT(2)
#define P9221_STAT_OV_VOLT			BIT(1)
#define P9221_STAT_OV_CURRENT			BIT(0)
#define P9221_STAT_LIMIT_MASK			(P9221_STAT_OV_TEMP | \
						 P9221_STAT_OV_VOLT | \
						 P9221_STAT_OV_CURRENT)
/*
 * Interrupt/Status flags for P9221R5
 */
#define P9221R5_STAT_CCRESET			BIT(12)
#define P9221R5_STAT_CCERROR			BIT(11)
#define P9221R5_STAT_PPRCVD			BIT(10)
#define P9221R5_STAT_CCDATARCVD			BIT(9)
#define P9221R5_STAT_CCSENDBUSY			BIT(8)
#define P9221R5_STAT_VOUTCHANGED		BIT(7)
#define P9221R5_STAT_VRECTON			BIT(6)
#define P9221R5_STAT_MODECHANGED		BIT(5)
#define P9221R5_STAT_UV				BIT(3)
#define P9221R5_STAT_OVT			BIT(2)
#define P9221R5_STAT_OVV			BIT(1)
#define P9221R5_STAT_OVC			BIT(0)
#define P9221R5_STAT_MASK			0x1FFF
#define P9221R5_STAT_CC_MASK			(P9221R5_STAT_CCRESET | \
						 P9221R5_STAT_PPRCVD | \
						 P9221R5_STAT_CCERROR | \
						 P9221R5_STAT_CCDATARCVD | \
						 P9221R5_STAT_CCSENDBUSY)
#define P9221R5_STAT_LIMIT_MASK			(P9221R5_STAT_UV | \
						 P9221R5_STAT_OVV | \
						 P9221R5_STAT_OVT | \
						 P9221R5_STAT_OVC)

/*
 * P9221R5_SYSTEM_MODE_REG bits.
 */
#define P9221R5_MODE_RAMCODE			BIT(6)
#define P9221R5_MODE_EEPROMCODE			BIT(5)
#define P9221R5_MODE_EXTENDED			BIT(3)
#define P9221R5_MODE_WPCMODE			BIT(0)

/*
 * P9382 unique registers
 */
#define P9382A_I2C_ADDRESS			0x3b

#define P9382A_CHIP_ID				0x9382
#define P9382A_DATA_SEND_BUF_START		0x130
#define P9382A_DATA_RECV_BUF_START		0x1B0

#define P9382A_STATUS_REG			0x34

#define P9382A_MODE_TXMODE			BIT(2)

#define FAST_SERIAL_ID_HEADER			0x4F
#define ACCESSORY_TYPE_MASK			0x7
#define ACCESSORY_TYPE_LOW_POWER_TX		BIT(2)

#define TXID_TYPE_MASK			0xFF000000 /* bit[24-31] */
#define TXID_TYPE_SHIFT			24
#define TXID_DD_TYPE			0xE0

enum p9221_align_mfg_chk_state {
	ALIGN_MFG_FAILED = -1,
	ALIGN_MFG_CHECKING,
	ALIGN_MFG_PASSED,
};

struct p9221_charger_platform_data {
	int				irq_gpio;
	int				irq_int;
	int				irq_det_gpio;
	int				irq_det_int;
	int				qien_gpio;
	int				slct_gpio;
	int				slct_value;
	int				ben_gpio;
	int				switch_gpio;
	int				max_vout_mv;
	u8				fod[P9221R5_NUM_FOD];
	u8				fod_epp[P9221R5_NUM_FOD];
	int				fod_num;
	int				fod_epp_num;
	int 				q_value;
	int				epp_rp_value;
	int				needs_dcin_reset;
	int				nb_alignment_freq;
	int				*alignment_freq;
	u32				alignment_scalar;
	u32				alignment_hysteresis;
	u32				icl_ramp_delay_ms;
	u32				power_mitigate_threshold;
};

struct p9221_charger_data {
	struct i2c_client		*client;
	struct p9221_charger_platform_data *pdata;
	struct power_supply		*wc_psy;
	struct power_supply		*dc_psy;
	struct votable			*dc_icl_votable;
	struct notifier_block		nb;
	struct mutex			io_lock;
	struct mutex			cmd_lock;
	struct device			*dev;
	struct delayed_work		notifier_work;
	struct delayed_work		dcin_work;
	struct delayed_work		align_work;
	struct delayed_work		tx_work;
	struct delayed_work		icl_ramp_work;
	struct delayed_work		power_mitigation_work;
	struct work_struct		uevent_work;
	struct alarm			icl_ramp_alarm;
	struct timer_list		vrect_timer;
	struct timer_list		align_timer;
	struct bin_attribute		bin;
	struct logbuffer		*log;
	int				online;
	bool				enabled;
	u16				addr;
	u8				count;
	u8				cust_id;
	int				ben_state;
	u8				pp_buf[P9221R5_MAX_PP_BUF_SIZE];
	bool				pp_buf_valid;
	int				addr_data_recv_buf_start;
	u8				rx_buf[P9221R5_DATA_RECV_BUF_SIZE];
	u16				rx_len;
	bool				rx_done;
	int				addr_data_send_buf_start;
	u8				tx_buf[P9221R5_DATA_SEND_BUF_SIZE];
	u32				tx_id;
	u8				tx_id_str[(sizeof(u32) * 2) + 1];
	u16				tx_len;
	bool				tx_done;
	bool				tx_busy;
	bool				check_np;
	bool				check_dc;
	bool				check_det;
	int				last_capacity;
	bool				resume_complete;
	bool				icl_ramp;
	bool				re_nego;
	u32				icl_ramp_ua;
	bool				fake_force_epp;
	bool				force_bpp;
	u32				dc_icl_bpp;
	int				align;
	int				align_count;
	int				alignment;
	u8				alignment_str[(sizeof(u32) * 3) + 1];
	int				alignment_last;
	enum p9221_align_mfg_chk_state  alignment_capable;
	int				mfg_check_count;
	u16				mfg;
	int				alignment_time;
	u32				current_filtered;
	u32				current_sample_cnt;
	struct delayed_work		dcin_pon_work;
	bool				is_mfg_google;
	bool				is_low_power_tx;
	u32				mitigate_threshold;
	u32				fod_cnt;
	bool				trigger_power_mitigation;
	bool                            wait_for_online;
};

struct p9221_prop_reg_map_entry {
	enum power_supply_property	prop;
	u16				reg;
	bool				get;
	bool				set;
};

#define P9221_SHOW(name, reg, width, mask, format)			\
	static ssize_t p9221_show_##name(struct device *dev,		\
					struct device_attribute *attr,	\
					char *buf)			\
	{								\
		struct i2c_client *client = to_i2c_client(dev);		\
		struct p9221_charger_data *charger =			\
					i2c_get_clientdata(client);	\
		u##width val;						\
		int ret;						\
									\
		ret = p9221_reg_read_##width(charger, reg, &val);	\
		if (ret)						\
			return ret;					\
		val &= mask;						\
		return snprintf(buf, PAGE_SIZE, format, val);		\
	}

#endif /* __P9221_CHARGER_H__ */
