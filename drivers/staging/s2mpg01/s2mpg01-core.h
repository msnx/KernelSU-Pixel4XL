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

#ifndef S2MPG01_REGULATOR_H
#define S2MPG01_REGULATOR_H

/* registers */
#define S2MPG01_REG_CHIP_ID 0x00
#define S2MPG01_REG_INT1 0x01
#define S2MPG01_REG_INT2 0x02
#define S2MPG01_REG_INT3 0x03
#define S2MPG01_REG_INT4 0x04
#define S2MPG01_REG_INT1M 0x05
#define S2MPG01_REG_INT2M 0x06
#define S2MPG01_REG_INT3M 0x07
#define S2MPG01_REG_INT4M 0x08
#define S2MPG01_REG_STATUS1 0x09
#define S2MPG01_REG_STATUS2 0x0A
#define S2MPG01_REG_STATUS3 0x0B
#define S2MPG01_REG_CFG_PM 0x0C
#define S2MPG01_REG_BUCK1_CTRL 0x0D
#define S2MPG01_REG_BUCK1_OUT 0x0E
#define S2MPG01_REG_BUCK2_CTRL 0x0F
#define S2MPG01_REG_BUCK2_OUT 0x10
#define S2MPG01_REG_BUCK3_CTRL 0x11
#define S2MPG01_REG_BUCK3_OUT 0x12
#define S2MPG01_REG_LDO1_CTRL 0x13
#define S2MPG01_REG_LDO2_CTRL 0x14
#define S2MPG01_REG_LDO3_CTRL 0x15
#define S2MPG01_REG_LDO4_CTRL 0x16
#define S2MPG01_REG_LDO5_CTRL 0x17
#define S2MPG01_REG_BUCK1_OUT_DVS 0x18
#define S2MPG01_REG_LDO3_CTRL_DVS 0x19
#define S2MPG01_REG_BOOST_CTRL 0x1A
#define S2MPG01_REG_NORM_THRES 0x1B
#define S2MPG01_REG_BOOST_THRES 0x1C
#define S2MPG01_REG_ADC_CTRL 0x1D
#define S2MPG01_REG_ADC_CTRL2 0x1E
#define S2MPG01_REG_ADC_CTRL3 0x1F
#define S2MPG01_REG_ADC0DATA 0x20
#define S2MPG01_REG_ADC1DATA 0x21
#define S2MPG01_REG_ADC2DATA 0x22
#define S2MPG01_REG_ADC3DATA 0x23
#define S2MPG01_REG_MUXSEL0 0x24
#define S2MPG01_REG_MUXSEL1 0x25
#define S2MPG01_REG_MUXSEL2 0x26
#define S2MPG01_REG_MUXSEL3 0x27
#define S2MPG01_REG_ADC_THRES_CTRL1 0x28
#define S2MPG01_REG_ADC_THRES_CTRL2 0x29
#define S2MPG01_REG_ADC_THRES_CTRL3 0x2A
#define S2MPG01_REG_ADC_THRES_CTRL4 0x2B
#define S2MPG01_REG_ADC_THRES_CTRL5 0x2C
#define S2MPG01_REG_ADC_THRES_CTRL6 0x2D
#define S2MPG01_REG_ADC_THRES_CTRL7 0x2E
#define S2MPG01_REG_ADC_THRES_CTRL8 0x2F
#define S2MPG01_REG_ADC_DBNC_CTRL1 0x30
#define S2MPG01_REG_ADC_DBNC_CTRL2 0x31
#define S2MPG01_REG_ADC_CAL_OFS1 0x32
#define S2MPG01_REG_ADC_CAL_OFS2 0x33
#define S2MPG01_REG_ADC_MON_TEMP 0x34
#define S2MPG01_REG_ADC_MON_VBAT 0x35
#define S2MPG01_REG_GPIO_CTRL 0x36
#define S2MPG01_REG_GPIO_A 0x37
#define S2MPG01_REG_GPIO_Y 0x38
#define S2MPG01_REG_TIME_CTRL1 0x39
#define S2MPG01_REG_TIME_CTRL2 0x51

/* adc inputs */
#define S2MPG01_ADC_I_SMPS1_SUM  0x01
#define S2MPG01_ADC_I_SMPS2  0x02
#define S2MPG01_ADC_I_SMPS3  0x03
#define S2MPG01_ADC_V_SMPS1  0x04
#define S2MPG01_ADC_V_SMPS2  0x05
#define S2MPG01_ADC_V_SMPS3  0x06
#define S2MPG01_ADC_I_SMPS1_PH1  0x07
#define S2MPG01_ADC_I_SMPS1_PH2  0x08
#define S2MPG01_ADC_I_SMPS1_PH3  0x09
#define S2MPG01_ADC_PTAT            0xC
#define S2MPG01_ADC_VBAT            0xD
#define S2MPG01_ADC_I_LDO1 0x11
#define S2MPG01_ADC_I_LDO2 0x12
#define S2MPG01_ADC_I_LDO3 0x13
#define S2MPG01_ADC_I_LDO4 0x14
#define S2MPG01_ADC_I_LDO5 0x15
#define S2MPG01_ADC_V_LDO1 0x16
#define S2MPG01_ADC_V_LDO2 0x17
#define S2MPG01_ADC_V_LDO3 0x18
#define S2MPG01_ADC_V_LDO4 0x19
#define S2MPG01_ADC_V_LDO5 0x1A

/* full scale of 8-bit adc slots */
#define S2MPG01_ADC_SCALE_I_SMPS1	46875	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_SMPS1_PH1	15625	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_SMPS1_PH2	15625	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_SMPS1_PH3	15625	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_SMPS2	15625	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_SMPS3	15625	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_V_SMPS1	4688	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_SMPS2	4688	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_SMPS3	6250	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_I_LDO1	1172	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_LDO2	1758	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_LDO3	4688	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_LDO4	1758	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_I_LDO5	1172	/* in uA/lsb */
#define S2MPG01_ADC_SCALE_V_LDO1	9375	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_LDO2	4688	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_LDO3	4688	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_LDO4	4688	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_V_LDO5	9375	/* in uV/lsb */
#define S2MPG01_ADC_SCALE_VBAT		23438	/* in uV/lsb */

/* macros for PTAT-to-temperature conversion */
#define TEMP_TO_PTAT_CODE(X)  ((238260 - (X)) / 1167)
#define PTAT_CODE_TO_TEMP(X)  (238260 - ((X) * 1167))

/* interrupt bits begin from INT1; mask bits are the same */
#define S2MPG01_INT_PONR		0
#define S2MPG01_INT_PONF		1
#define S2MPG01_INT_ADC_CH0		2
#define S2MPG01_INT_ADC_CH1		3
#define S2MPG01_INT_ADC_CH2		4
#define S2MPG01_INT_ADC_CH3		5
#define S2MPG01_INT_ADC_DONE		6
#define S2MPG01_INT_WATCHDOG		7
#define S2MPG01_INT_T_ALARM		8
#define S2MPG01_INT_NORM		9
#define S2MPG01_INT_BOOST		10
#define S2MPG01_INT_SMPS1_UV		11
#define S2MPG01_INT_TINT_OUT		12
#define S2MPG01_INT_TSD			13
#define S2MPG01_INT_TH_TRIPR		14
#define S2MPG01_INT_TH_TRIPF		15
#define S2MPG01_INT_LDO1_OI		16
#define S2MPG01_INT_LDO2_OI		17
#define S2MPG01_INT_LDO3_OI		18
#define S2MPG01_INT_LDO4_OI		19
#define S2MPG01_INT_LDO5_OI		20
#define S2MPG01_INT_SMPS3_OI		21
#define S2MPG01_INT_SMPS2_OI		22
#define S2MPG01_INT_SMPS1_OI		23
#define S2MPG01_INT_LDO3_DVS_END	30
#define S2MPG01_INT_SMPS1_DVS_END	31

#define S2MPG01_INT_4_RSVD_BITS		0x3F

/* regulator names */
#define S2MPG01_REGLTR_NAME_SMPS1 "s2mpg01_smps1"
#define S2MPG01_REGLTR_NAME_SMPS2 "s2mpg01_smps2"
#define S2MPG01_REGLTR_NAME_SMPS3 "s2mpg01_smps3"
#define S2MPG01_REGLTR_NAME_LDO1  "s2mpg01_ldo1"
#define S2MPG01_REGLTR_NAME_LDO2  "s2mpg01_ldo2"
#define S2MPG01_REGLTR_NAME_LDO3  "s2mpg01_ldo3"
#define S2MPG01_REGLTR_NAME_LDO4  "s2mpg01_ldo4"
#define S2MPG01_REGLTR_NAME_LDO5  "s2mpg01_ldo5"
#define S2MPG01_REGLTR_NAME_BOOST_SMPS1 "s2mpg01_boost_smps1"
#define S2MPG01_REGLTR_NAME_BOOST_LDO3 "s2mpg01_boost_ldo3"

/* silicon versions */
#define S2MPG01_REV_ES 0xC

#define S2MPG01_NUM_GPIOS 4
#define S2MPG01_NUM_IRQ_REGS 4

/* regulator id enum */
enum s2mpg01_regulator_ids {
	S2MPG01_ID_SMPS1,
	S2MPG01_ID_SMPS2,
	S2MPG01_ID_SMPS3,
	S2MPG01_ID_LDO1,
	S2MPG01_ID_LDO2,
	S2MPG01_ID_LDO3,
	S2MPG01_ID_LDO4,
	S2MPG01_ID_LDO5,
	S2MPG01_ID_BOOST_SMPS1,
	S2MPG01_ID_BOOST_LDO3,
	S2MPG01_ID_ALL,
	S2MPG01_NUM_REGULATORS = S2MPG01_ID_ALL,
};

/* driver data structure */
struct s2mpg01_core {
	struct device *dev;
	struct regmap *regmap;
	struct s2mpg01_platform_data *pdata;
	u8 rev_id;

	/* kernel thread to notify regulator fail event and
	 * clear any pending interrupt status
	 */
	struct work_struct reset_work;

	/* delayed work to poll for ready after reset */
	struct delayed_work poll_ready_work;

	/* completion used for initialization */
	struct completion init_complete;

	/* flags used for serialization */
	unsigned long adc_conv_busy;

	/* completion used for signaling end of adc conversion */
	struct completion adc_conv_complete;
};

/* platform data structure */
struct s2mpg01_platform_data {
	int pon_gpio;
	int pmic_ready_gpio;
	int intb_gpio;
	unsigned int resetb_irq;
	unsigned int intb_irq;
};

int s2mpg01_read_byte(struct s2mpg01_core *ddata, u8 addr, u8 *data);
int s2mpg01_write_byte(struct s2mpg01_core *ddata, u8 addr, u8 data);
int s2mpg01_update_bits(struct s2mpg01_core *ddata, u8 addr, unsigned int mask,
			u8 data);
void s2mpg01_toggle_pon_oneway(struct s2mpg01_core *ddata, bool turn_on);
void s2mpg01_toggle_pon(struct s2mpg01_core *ddata);
int s2mpg01_dump_regs(struct s2mpg01_core *ddata);
int s2mpg01_read_adc_chan(struct s2mpg01_core *ddata, int chan_num,
			  u8 *chan_data);
int s2mpg01_enable_boost(struct s2mpg01_core *ddata);
int s2mpg01_disable_boost(struct s2mpg01_core *ddata);
bool s2mpg01_boost_mode_status(struct s2mpg01_core *ddata);

/* s2mpg01-sysfs.c */
void s2mpg01_config_sysfs(struct device *dev);

/* s2mpg01-regulator.c */
void s2mpg01_regulator_notify(enum s2mpg01_regulator_ids rid,
			      unsigned long event);

#endif /* S2MPG01_REGULATOR_H */
