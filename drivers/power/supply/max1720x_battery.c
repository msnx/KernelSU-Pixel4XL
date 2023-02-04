/*
 * Fuel gauge driver for Maxim 17201/17205
 *
 * Copyright (C) 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h> /* register_chrdev, unregister_chrdev */
#include <linux/module.h>
#include <linux/seq_file.h> /* seq_read, seq_lseek, single_release */
#include <google/google_bms.h>
#include <google/logbuffer.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define MAX17X0X_TPOR_MS 150

#define MAX1720X_TRECALL_MS 5
#define MAX1730X_TRECALL_MS 5
#define MAX1720X_TICLR_MS 500
#define MAX1720X_I2C_DRIVER_NAME "max1720x_fg_irq"
#define MAX1720X_N_OF_HISTORY_PAGES 203
#define MAX1730X_N_OF_HISTORY_PAGES 100
#define MAX1720X_DELAY_INIT_MS 1000
#define FULLCAPNOM_STABILIZE_CYCLES 5
#define CYCLE_BUCKET_SIZE 200
#define TEMP_BUCKET_SIZE 5		/* unit is 0.1 degree C */
#define NB_CYCLE_BUCKETS 4

#define HISTORY_DEVICENAME "maxfg_history"

#define BATTERY_DEFAULT_CYCLE_STABLE	0
#define BATTERY_DEFAULT_CYCLE_FADE	0
#define BATTERY_DEFAULT_CYCLE_BAND	10
#define BATTERY_MAX_CYCLE_BAND		20

#define MAX17201_FIXUP_UPDATE_DELAY_MS	10

#define MAX1720X_GAUGE_TYPE	0
#define MAX1730X_GAUGE_TYPE	1
#define MAX1730X_GAUGE_PASS1	0x404

#define MAX1730X_NVPRTTH1_CHARGING	0x0008
#define MAX1730X_NPROTCFG_PASS1		0x6EA3
#define MAX1730X_NPROTCFG_PASS2		0x0A04

enum max1720x_register {
	/* ModelGauge m5 Register */
	MAX1720X_STATUS = 0x00,
	MAX1720X_VALRTTH = 0x01,
	MAX1720X_TALRTTH = 0x02,
	MAX1720X_SALRTTH = 0x03,
	MAX1720X_ATRATE = 0x04,
	MAX1720X_REPCAP = 0x05,
	MAX1720X_REPSOC = 0x06,
	MAX1720X_AGE = 0x07,
	MAX1720X_TEMP = 0x08,
	MAX1720X_VCELL = 0x09,
	MAX1720X_CURRENT = 0x0A,
	MAX1720X_AVGCURRENT = 0x0B,
	MAX1720X_QRESIDUAL = 0x0C,
	MAX1720X_MIXSOC = 0x0D,
	MAX1720X_AVSOC = 0x0E,
	MAX1720X_MIXCAP = 0x0F,

	MAX1720X_FULLCAP = 0x10,
	MAX1720X_TTE = 0x11,
	MAX1720X_QRTABLE00 = 0x12,
	MAX1720X_FULLSOCTHR = 0x13,
	MAX1720X_RCELL = 0x14,
	MAX1720X_RFAST = 0x15,
	MAX1720X_AVGTA = 0x16,
	MAX1720X_CYCLES = 0x17,
	MAX1720X_DESIGNCAP = 0x18,
	MAX1720X_AVGVCELL = 0x19,
	MAX1720X_MAXMINTEMP = 0x1A,
	MAX1720X_MAXMINVOLT = 0x1B,
	MAX1720X_MAXMINCURR = 0x1C,
	MAX1720X_CONFIG = 0x1D,
	MAX1720X_ICHGTERM = 0x1E,
	MAX1720X_AVCAP = 0x1F,

	MAX1720X_TTF = 0x20,
	MAX1720X_DEVNAME = 0x21,
	MAX1720X_QRTABLE10 = 0x22,
	MAX1720X_FULLCAPNOM = 0x23,
	MAX1720X_AIN0 = 0x27,
	MAX1720X_LEARNCFG = 0x28,
	MAX1720X_FILTERCFG = 0x29,
	MAX1720X_RELAXCFG = 0x2A,
	MAX1720X_MISCCFG = 0x2B,
	MAX1720X_TGAIN = 0x2C,
	Max1720x_TOff = 0x2D,
	MAX1720X_CGAIN = 0x2E,
	MAX1720X_COFF = 0x2F,

	MAX1720X_QRTABLE20 = 0x32,
	MAX1720X_FULLCAPREP = 0x35,
	MAX1720X_IAVGEMPTY = 0x36,
	MAX1720X_RCOMP0 = 0x38,
	MAX1720X_TEMPCO = 0x39,
	MAX1720X_VEMPTY = 0x3A,
	MAX1720X_FSTAT = 0x3D,
	MAX1720X_TIMER = 0x3E,
	MAX1720X_SHDNTIMER = 0x3F,

	MAX1720X_QRTABLE30 = 0x42,
	MAX1720X_DQACC = 0x45,
	MAX1720X_DPACC = 0x46,
	MAX1720X_VFREMCAP = 0x4A,
	MAX1720X_QH = 0x4D,

	MAX1720X_STATUS2 = 0xB0,
	MAX1720X_IALRTTH = 0xB4,
	MAX1720X_VSHDNCFG = 0xB8,
	MAX1720X_AGEFORECAST = 0xB9,
	MAX1720X_HIBCFG = 0xBA,
	MAX1720X_CONFIG2 = 0xBB,
	MAX1720X_VRIPPLE = 0xBC,
	MAX1720X_PACKCFG = 0xBD,
	MAX1720X_TIMERH = 0xBE,

	MAX1720X_AVGCELL4 = 0xD1,
	MAX1720X_AVGCELL3 = 0xD2,
	MAX1720X_AVGCELL2 = 0xD3,
	MAX1720X_AVGCELL1 = 0xD4,
	MAX1720X_CELL4 = 0xD5,
	MAX1720X_CELL3 = 0xD6,
	MAX1720X_CELL2 = 0xD7,
	MAX1720X_CELL1 = 0xD8,
	MAX1720X_CELLX = 0xD9,
	MAX1720X_BATT = 0xDA,
	MAX1720X_ATQRESIDUAL = 0xDC,
	MAX1720X_ATTTE = 0xDD,
	MAX1720X_ATAVSOC = 0xDE,
	MAX1720X_ATAVCAP = 0xDF,

	/* Individual Registers */
	MAX1720X_COMMAND = 0x60,
	MAX1720X_COMMSTAT = 0x61,
	MAX1720X_LOCK = 0x7F,
	MAX1720X_ODSCTH = 0xF2,
	MAX1720X_ODSCCFG = 0xF3,
	MAX1720X_VFOCV = 0xFB,
	MAX1720X_ALARM = 0xFD,
	MAX1720X_VFSOC = 0xFF,
};

enum max1720x_status_bits {
	MAX1720X_STATUS_POR = BIT(1),
	MAX1720X_STATUS_IMN = BIT(2),
	MAX1720X_STATUS_BST = BIT(3),
	MAX1720X_STATUS_IMX = BIT(6),
	MAX1720X_STATUS_DSOCI = BIT(7),
	MAX1720X_STATUS_VMN = BIT(8),
	MAX1720X_STATUS_TMN = BIT(9),
	MAX1720X_STATUS_SMN = BIT(10),
	MAX1720X_STATUS_BI = BIT(11),
	MAX1720X_STATUS_VMX = BIT(12),
	MAX1720X_STATUS_TMX = BIT(13),
	MAX1720X_STATUS_SMX = BIT(14),
	MAX1720X_STATUS_BR = BIT(15),
};

enum max1720x_commstat_bits {
	MAX1720X_COMMSTAT_NVBUSY = BIT(1),
	MAX1720X_COMMSTAT_NVERROR = BIT(2),
};

enum max1720x_config_bits {
	MAX1720X_CONFIG_BER = BIT(0),
	MAX1720X_CONFIG_BEI = BIT(1),
	MAX1720X_CONFIG_AEN = BIT(2),
	MAX1720X_CONFIG_FTHRM = BIT(3),
	MAX1720X_CONFIG_ETHRM = BIT(4),
	MAX1720X_CONFIG_COMMSH = BIT(6),
	MAX1720X_CONFIG_SHDN = BIT(7),
	MAX1720X_CONFIG_TEX = BIT(8),
	MAX1720X_CONFIG_TEN = BIT(9),
	MAX1720X_CONFIG_AINSH = BIT(10),
	MAX1720X_CONFIG_ALRTP = BIT(11),
	MAX1720X_CONFIG_VS = BIT(12),
	MAX1720X_CONFIG_TS = BIT(13),
	MAX1720X_CONFIG_SS = BIT(14),
};

enum max1720x_nnvcfg0_bits {
	MAX1720X_NNVCFG0_ENSBS = BIT(0),
	MAX1720X_NNVCFG0_ENHCFG = BIT(1),
	MAX1720X_NNVCFG0_ENAF = BIT(2),
	MAX1720X_NNVCFG0_ENMC = BIT(3),
	MAX1720X_NNVCFG0_ENDC = BIT(4),
	MAX1720X_NNVCFG0_ENVE = BIT(5),
	MAX1720X_NNVCFG0_ENCG = BIT(6),
	MAX1720X_NNVCFG0_ENICT = BIT(7),
	MAX1720X_NNVCFG0_ENLCFG = BIT(8),
	MAX1720X_NNVCFG0_ENRCFG = BIT(9),
	MAX1720X_NNVCFG0_ENFCFG = BIT(10),
	MAX1720X_NNVCFG0_ENCFG = BIT(11),
	MAX1720X_NNVCFG0_ENX = BIT(14),
	MAX1720X_NNVCFG0_ENOCV = BIT(15),
};

enum max1720x_command_bits {
	MAX1720X_COMMAND_FUEL_GAUGE_RESET = 0x0001,
	MAX1720X_COMMAND_HARDWARE_RESET = 0x000F,
	MAX1720X_COMMAND_QUERY_REMAINING_UPDATES = 0xE2FA,
	MAX1720X_COMMAND_COPY_NV_BLOCK = 0xE904,
	MAX1720X_COMMAND_HISTORY_RECALL_WRITE_0 = 0xE2FB,
	MAX1720X_COMMAND_HISTORY_RECALL_WRITE_1 = 0xE2FC,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_0 = 0xE2FC,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_1 = 0xE2FD,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_2 = 0xE2FE,
	MAX1720X_READ_HISTORY_CMD_BASE = 0xE226,
};


/** Nonvolatile Register Memory Map */
enum max1720x_nvram {
	MAX1720X_NVRAM_START = 0x80,
	MAX1720X_NUSER18C = 0x8C,	/* reserved for QH capacity */
	MAX1720X_NUSER18D = 0x8D,	/* reserved for QH QH  */
	MAX1720X_NODSCTH = 0x8E,	/* CCLC */
	MAX1720X_NODSCCFG = 0x8F,	/* CCLC */
	MAX1720X_NLEARNCFG = 0x9F,	/* not referred here */
	MAX1720X_NMISCCFG = 0xB2,	/* CCLC */
	MAX1720X_NDESIGNCAP = 0xB3,
	MAX1720X_NHIBCFG = 0xB4,	/* CCLC */
	MAX1720X_NCONVGCFG = 0xB7,	/* convergence configuration */
	MAX1720X_NNVCFG0 = 0xB8,	/* 'NCG0' with NCG1 */
	MAX1720X_NVALRTTH = 0xC0,	/* Comp Update Count */
	MAX1720X_NTALRTTH = 0xC1,	/* Capacity Update Count */
	MAX1720X_NUSER1C4 = 0xC4,	/* CCLC */
	MAX1720X_NUSER1C5 = 0xC5,	/* CCLC */
	MAX1720X_NTTFCFG = 0xC7,	/* Average resistance */
	MAX1720X_NCGAIN = 0xC8,		/* ....  */
	MAX1720X_NMANFCTRNAME0 = 0xCC,	/* SNUM */
	MAX1720X_NMANFCTRNAME1 = 0xCD,	/* CCLC */
	MAX1720X_NMANFCTRNAME2 = 0xCE,	/* CCLC */
	MAX1720X_NRSENSE = 0xCF,	/* value of sense resistor */
	MAX1720X_NUSER1D0 = 0xD0,	/* SNUM */
	MAX1720X_NUSER1D1 = 0xD1,	/* SNUM */
	MAX1720X_NAGEFCCFG = 0xD2,
	MAX1720X_NUSER1D4 = 0xD4,	/* URST */
	MAX1720X_NMANFCTRDATE = 0xD6,	/* SNUM */
	MAX1720X_NFIRSTUSED = 0xD7,	/* CCLC */
	MAX1720X_NSERIALNUMBER0 = 0xD8,	/* SNUM */
	MAX1720X_NSERIALNUMBER1 = 0xD9,	/* SNUM */
	MAX1720X_NSERIALNUMBER2 = 0xDA,	/* SNUM */
	MAX1720X_NDEVICENAME0 = 0xDB,	/* SNUM */
	MAX1720X_NDEVICENAME1 = 0xDC,	/* SNUM */
	MAX1720X_NDEVICENAME2 = 0xDD,	/* SNUM */
	MAX1720X_NDEVICENAME3 = 0xDE,	/* SNUM */
	MAX1720X_NDEVICENAME4 = 0xDF,	/* CCLC */
	MAX1720X_NVRAM_END = MAX1720X_NDEVICENAME4,

	MAX1720X_HISTORY_START = 0xE0,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START = 0xE1,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_END = 0xE4,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END = 0xEA,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_START = 0xEB,
	MAX1720X_NVRAM_REMAINING_UPDATES = 0xED,
	MAX1720X_NVRAM_HISTORY_END = 0xEF,

	MAX17201_COMP_UPDATE_CNT = MAX1720X_NVALRTTH,
	MAX17201_DXACC_UPDATE_CNT = MAX1720X_NTALRTTH,
};

enum max1730x_nvram {
	MAX1730X_NVRAM_START 	= 0x80,
	MAX1730X_NMANFCTRNAME0	= 0xCC,
	MAX1730X_NMANFCTRNAME1	= 0xCD,
	MAX1730X_NVPRTTH1 	= 0xD0,
	MAX1730X_NDPLIMIT	= 0xE0,
	MAX1730X_NSCOCVLIM	= 0xE1,

	MAX1730X_NVRAM_END 	= 0xEF,
	MAX1730X_HISTORY_START 	= 0xF0,
	MAX1730X_HISTORY_WRITE_STATUS_START = 0xF2,
	MAX1730X_HISTORY_VALID_STATUS_END = 0xFB,
	MAX1730X_HISTORY_WRITE_STATUS_END = 0xFE,
	MAX1730X_HISTORY_END	= 0xFF,
};

enum max1730x_register {
	MAX1730X_MAXMINVOLT = 0x08,
	MAX1730X_MAXMINTEMP = 0x09,
	MAX1730X_MAXMINCURR = 0x0A,
	MAX1730X_CONFIG = 0x0B,
	MAX1730X_FULLCAPREP = 0x10,
	MAX1730X_VCELL = 0x1A,
	MAX1730X_TEMP = 0x1B,
	MAX1730X_CURRENT = 0x1C,
	MAX1730X_AVGCURRENT = 0x1D,
	MAX1730X_MIXCAP = 0x2B,
	MAX1730X_FULLCAP = 0x35,
	MAX1730X_LEARNCFG = 0xA1,
	MAX1730X_MAXPEAKPWR = 0xA4,
	MAX1730X_SUSPEAKPWR = 0xA5,
	MAX1730X_PACKRESISTANCE = 0xA6,
	MAX1730X_SYSRESISTANCE = 0xA7,
	MAX1730X_MINSYSVOLTAGE = 0xA8,
	MAX1730X_MPPCURRENT = 0xA9,
	MAX1730X_SPPCURRENT = 0xAA,
	MAX1730X_CONFIG2 = 0xAB,
	MAX1730X_IALRTTH = 0xAC,
	MAX1730X_MINVOLT = 0xAD,
	MAX1730X_MINCURR = 0xAE,
	MAX1730X_NVPRTTH1BAK = 0xD6,
	MAX1730X_NPROTCFG = 0xD7,

};

enum max1730x_command_bits {
	MAX1730X_COMMAND_FUEL_GAUGE_RESET = 0x8000,
	MAX1730X_READ_HISTORY_CMD_BASE = 0xE22E,
	MAX1730X_COMMAND_HISTORY_RECALL_WRITE_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_1 = 0xE29D,
};

enum max17xxx_register {
	MAX17XXX_COMMAND	= MAX1720X_COMMAND,
};

enum max17xxx_nvram {
	MAX17XXX_QHCA = MAX1720X_NUSER18C,
	MAX17XXX_QHQH = MAX1720X_NUSER18D,
};

enum max17xxx_command_bits {
	MAX17XXX_COMMAND_NV_RECALL 	  = 0xE001,
};

#define MAX1720X_HISTORY_PAGE_SIZE \
		(MAX1720X_NVRAM_HISTORY_END - MAX1720X_HISTORY_START + 1)

#define MAX1720X_N_OF_HISTORY_FLAGS_REG				\
	(MAX1720X_NVRAM_HISTORY_END -				\
		MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START + 1 + \
		MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END -	\
		MAX1720X_HISTORY_START + 1)

#define MAX1720X_N_OF_QRTABLES 4

#define MAX1730X_HISTORY_PAGE_SIZE \
	(MAX1730X_HISTORY_END - MAX1730X_HISTORY_START + 1)

#define MAX1730X_N_OF_HISTORY_FLAGS_REG \
	(MAX1730X_HISTORY_END - \
		MAX1730X_HISTORY_END + 1 + \
		MAX1730X_HISTORY_VALID_STATUS_END - \
		MAX1730X_HISTORY_START + 1)

enum max17x0x_reg_types {
	GBMS_ATOM_TYPE_MAP = 0,
	GBMS_ATOM_TYPE_REG = 1,
	GBMS_ATOM_TYPE_ZONE = 2,
	GBMS_ATOM_TYPE_SET = 3,
};

enum max17x0x_reg_tags {
	MAX17X0X_TAG_avgc,
	MAX17X0X_TAG_cnfg,
	MAX17X0X_TAG_mmdv,
	MAX17X0X_TAG_vcel,
	MAX17X0X_TAG_temp,
	MAX17X0X_TAG_curr,
	MAX17X0X_TAG_mcap,
	MAX17X0X_TAG_avgr,

	MAX17X0X_TAG_BCNT,
	MAX17X0X_TAG_SNUM,
	MAX17X0X_TAG_HSTY,
	MAX17X0X_TAG_BCEA,
	MAX17X0X_TAG_rset,
	MAX17X0X_TAG_BRES,
	MAX17X0X_TAG_RSOC,
};

struct max17x0x_reg {
	int type;
	int size;
	union {
		unsigned int base;
		unsigned int reg;
		const u16 *map16;
		const u8 *map;
	};
};

struct max17x0x_cache_data {
	struct max17x0x_reg atom;
	u16 *cache_data;
};
/* Capacity Estimation */
struct gbatt_capacity_estimation {
	struct mutex batt_ce_lock;
	struct delayed_work settle_timer;
	int cap_tsettle;
	int cap_filt_length;
	int estimate_state;
	bool cable_in;
	int delta_cc_sum;
	int delta_vfsoc_sum;
	int cap_filter_count;
	int start_cc;
	int start_vfsoc;
};

#define DEFAULT_CAP_SETTLE_INTERVAL	3
#define DEFAULT_CAP_FILTER_LENGTH	12

#define ESTIMATE_DONE		2
#define ESTIMATE_PENDING	1
#define ESTIMATE_NONE		0

#define CE_CAP_FILTER_COUNT	0
#define CE_DELTA_CC_SUM_REG	1
#define CE_DELTA_VFSOC_SUM_REG	2

#define CE_FILTER_COUNT_MAX	15


struct max1720x_history {
	loff_t history_index;
	int history_count;
	bool *page_status;
	int page_size;
	u16 *history;

	int comp_update_count;
	int dxacc_update_count;
};

struct max1720x_chip {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work init_work;
	struct delayed_work filtercfg_work;
	struct delayed_work dumpreg_work;
	struct i2c_client *primary;
	struct i2c_client *secondary;
	struct regmap *regmap_nvram;
	struct device_node *batt_node;
	struct iio_channel *iio_ch;

	u16 devname;
	struct max17x0x_cache_data nRAM_por;
	bool needs_reset;

	/* config */
	u16 reg_prop_capacity_raw;

	/* history */
	struct mutex history_lock;
	int hcmajor;
	struct cdev hcdev;
	struct class *hcclass;
	bool history_available;
	bool history_added;
	int history_page_size;
	int nb_history_pages;
	int nb_history_flag_reg;

	int fake_battery;
	/* for storage interface */
	struct max1720x_history history_storage;

	u16 RSense;
	u16 RConfig;
	int batt_id;
	int cycle_count;
	int cycle_count_offset;
	bool init_complete;
	bool resume_complete;
	u16 health_status;
	int health;
	int fake_capacity;
	int previous_qh;
	int current_capacity;
	int prev_charge_status;
	char serial_number[25];
	bool offmode_charger;
	s32 convgcfg_hysteresis;
	int nb_convgcfg;
	int curr_convgcfg_idx;
	s16 *temp_convgcfg;
	u16 *convgcfg_values;
	struct mutex convgcfg_lock;
	unsigned int debug_irq_none_cnt;
	bool shadow_override;
	int nb_empty_voltage;
	u16 *empty_voltage;

	/* fix to capacity estimation */
	int comp_update_count;
	int dxacc_update_count;
	u16 design_capacity;
	int cycle_band;
	int cycle_fade;
	int cycle_stable;
	int ini_rcomp0;
	int ini_tempco;
	int ini_filtercfg;
	/* Capacity Estimation */
	struct gbatt_capacity_estimation cap_estimate;
	struct logbuffer *ce_log;
	struct logbuffer *maxfg_log;
};

#define MAX1720_EMPTY_VOLTAGE(profile, temp, cycle) \
	profile->empty_voltage[temp * NB_CYCLE_BUCKETS + cycle]

/* when 1 use max17301 features */
static int max17xxx_gauge_type = -1;

static inline int max1720x_regmap_read(struct regmap *map,
				       unsigned int reg,
				       u16 *val,
				       const char *name)
{
	int rtn;
	unsigned int tmp;

	if (reg != MAX1720X_DEVNAME && max17xxx_gauge_type == -1)
		pr_warn("using default MAX1720X regmap\n");

	rtn = regmap_read(map, reg, &tmp);
	if (rtn)
		pr_err("Failed to read %s\n", name);
	else
		*val = tmp;

	return rtn;
}

#define REGMAP_READ(regmap, what, dst) \
	max1720x_regmap_read(regmap, what, dst, #what)

static inline int max1720x_regmap_write(struct regmap *map,
				       unsigned int reg,
				       u16 data,
				       const char *name)
{
	int rtn;

	if (reg != MAX1720X_DEVNAME && max17xxx_gauge_type == -1)
		pr_warn("using default MAX1720X regmap\n");

	rtn = regmap_write(map, reg, data);
	if (rtn)
		pr_err("Failed to write %s\n", name);

	return rtn;
}

#define REGMAP_WRITE(regmap, what, value) \
	max1720x_regmap_write(regmap, what, value, #what)

/* ------------------------------------------------------------------------- */

/* this is a map for u16 registers */
#define ATOM_INIT_MAP(...)			\
	.type = GBMS_ATOM_TYPE_MAP,		\
	.size = 2 * sizeof((u8[]){__VA_ARGS__}),\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_REG16(r)		\
	.type = GBMS_ATOM_TYPE_REG,	\
	.size = 2,			\
	.reg = r

#define ATOM_INIT_ZONE(start, sz)	\
	.type = GBMS_ATOM_TYPE_ZONE,	\
	.size = sz,			\
	.base = start

/* a set has no storage and cannot be used in load/store */
#define ATOM_INIT_SET(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_SET16(...)		\
	.type = GBMS_ATOM_TYPE_SET,	\
	.size = 0,			\
	.map16 = (u16[]){__VA_ARGS__}


/* the point of the '' constants is to avoid defines for tag names...
 * so please don't add them ;-)
 */
static const struct max17x0x_reg max1720x[] = {
	[MAX17X0X_TAG_avgc] = { ATOM_INIT_REG16(MAX1720X_AVGCURRENT)},
	[MAX17X0X_TAG_cnfg] = { ATOM_INIT_REG16(MAX1720X_CONFIG)},
	[MAX17X0X_TAG_mmdv] = { ATOM_INIT_REG16(MAX1720X_MAXMINVOLT)},
	[MAX17X0X_TAG_vcel] = { ATOM_INIT_REG16(MAX1720X_VCELL)},
	[MAX17X0X_TAG_temp] = { ATOM_INIT_REG16(MAX1720X_TEMP)},
	[MAX17X0X_TAG_curr] = { ATOM_INIT_REG16(MAX1720X_CURRENT)},
	[MAX17X0X_TAG_mcap] = { ATOM_INIT_REG16(MAX1720X_MIXCAP)},
	[MAX17X0X_TAG_avgr] = { ATOM_INIT_REG16(MAX1720X_NTTFCFG) },

	[MAX17X0X_TAG_BCNT] = { ATOM_INIT_MAP(0x8e, 0x8f, 0xb2, 0xb4, 0xc4,
					      0xc5, 0xcd, 0xce, 0xd7, 0xdf) },
	[MAX17X0X_TAG_SNUM] = { ATOM_INIT_MAP(0xcc, 0xd8, 0xd9, 0xda, 0xd6,
					      0xdb, 0xdc, 0xdd, 0xde, 0xd1,
					      0xd0) },
	[MAX17X0X_TAG_RSOC] = { ATOM_INIT_MAP(0xc2) },
	[MAX17X0X_TAG_HSTY] = { ATOM_INIT_SET(0xe0, 0xe1, 0xe4, 0xea, 0xeb,
					      0xed, 0xef) },
	[MAX17X0X_TAG_BCEA] = { ATOM_INIT_SET(MAX1720X_NUSER1D4,
					      MAX1720X_NAGEFCCFG,
					      MAX1720X_NMISCCFG) },
	[MAX17X0X_TAG_rset] = { ATOM_INIT_SET16(MAX1720X_CONFIG2,
					MAX1720X_COMMAND_FUEL_GAUGE_RESET,
					700)},
	[MAX17X0X_TAG_BRES] = { ATOM_INIT_SET(MAX1720X_NTTFCFG,
					      MAX1720X_NUSER1D4) },
};

/* see b/119416045 for layout */
static const struct max17x0x_reg max1730x[] = {
	[MAX17X0X_TAG_avgc] = { ATOM_INIT_REG16(MAX1730X_AVGCURRENT)},
	[MAX17X0X_TAG_cnfg] = { ATOM_INIT_REG16(MAX1730X_CONFIG)},
	[MAX17X0X_TAG_mmdv] = { ATOM_INIT_REG16(MAX1730X_MAXMINVOLT)},
	[MAX17X0X_TAG_vcel] = { ATOM_INIT_REG16(MAX1730X_VCELL)},
	[MAX17X0X_TAG_temp] = { ATOM_INIT_REG16(MAX1730X_TEMP)},
	[MAX17X0X_TAG_curr] = { ATOM_INIT_REG16(MAX1730X_CURRENT)},
	[MAX17X0X_TAG_mcap] = { ATOM_INIT_REG16(MAX1730X_MIXCAP)},
	[MAX17X0X_TAG_avgr] = { ATOM_INIT_REG16(MAX1730X_NMANFCTRNAME1) },

	[MAX17X0X_TAG_BCNT] = { ATOM_INIT_MAP(0x8e, 0x8f, 0x9d, 0x9e, 0x9f,
					      0xb2, 0xb4, 0xb6, 0xc7, 0xe2)},
	[MAX17X0X_TAG_SNUM] = { ATOM_INIT_MAP(0xce, 0xe6, 0xe7, 0xe8, 0xe9,
					      0xea, 0xeb, 0xec, 0xed, 0xee,
					      0xef) },
	[MAX17X0X_TAG_RSOC] = { ATOM_INIT_MAP(0xc2) },
	[MAX17X0X_TAG_HSTY] = { ATOM_INIT_SET(0xf0, 0xf2, 0xfb, 0xfe, 0xff) },
	[MAX17X0X_TAG_BCEA] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME0,
					      MAX1730X_NDPLIMIT,
					      MAX1730X_NSCOCVLIM) },
	[MAX17X0X_TAG_rset] = { ATOM_INIT_SET16(MAX1730X_CONFIG2,
					MAX1730X_COMMAND_FUEL_GAUGE_RESET,
					700)},
	[MAX17X0X_TAG_BRES] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME1,
					      MAX1730X_NMANFCTRNAME0) },
};

struct max17x0x_device_info {
	const struct max17x0x_reg *map;
	unsigned int max;
};

static const struct max17x0x_device_info max17x0x[] = {
	{ .map = max1720x, .max = ARRAY_SIZE(max1720x) },
	{ .map = max1730x, .max = ARRAY_SIZE(max1730x) },
};

const struct max17x0x_reg *max17x0x_find_by_index(int index)
{
	if (max17xxx_gauge_type == -1)
		return NULL;

	if (index < 0 || index >= max17x0x[max17xxx_gauge_type].max)
		return NULL;

	return &max17x0x[max17xxx_gauge_type].map[index];
}

const struct max17x0x_reg *max17x0x_find_by_tag(enum max17x0x_reg_tags tag)
{
	return max17x0x_find_by_index(tag);
}

static inline int max17x0x_reg_read(struct regmap *map,
				    enum max17x0x_reg_tags tag,
				    u16 *val)
{
	const struct max17x0x_reg *reg = max17x0x_find_by_tag(tag);
	unsigned int tmp;
	int rtn;

	if (!reg)
		return -EINVAL;

	rtn = regmap_read(map, reg->reg, &tmp);
	if (rtn)
		pr_err("Failed to read %x\n", reg->reg);
	else
		*val = tmp;

	return rtn;
}

static inline int max17x0x_reg_write(struct regmap *map,
				    enum max17x0x_reg_tags tag,
				    u16 val)
{
	const struct max17x0x_reg *reg = max17x0x_find_by_tag(tag);
	int rtn;

	if (!reg)
		return -EINVAL;

	rtn = regmap_write(map, reg->reg, val);
	if (rtn)
		pr_err("Failed to write %x\n", reg->reg);

	return rtn;
}

/* ------------------------------------------------------------------------- */

/* offset of the register in this atom.
 * NOTE: this is the byte offset regardless of the size of the register
 */
static int max17x0x_reg_offset_of(const struct max17x0x_reg *a,
				  unsigned int reg)
{
	int i;

	switch (a->type) {
	case GBMS_ATOM_TYPE_REG:
		return (reg == a->reg) ? 0 : -EINVAL;
	case GBMS_ATOM_TYPE_ZONE:
		if (reg >= a->base && reg < a->base + a->size)
			return (reg - a->base) * 2;
		break;
	case GBMS_ATOM_TYPE_MAP:
		for (i = 0 ; i < a->size ; i++)
			if (a->map[i] == reg)
				return i * 2;
		break;
	}

	return -ERANGE;
}

static int max17x0x_reg_store_sz(struct regmap *regmap,
				 const struct max17x0x_reg *a,
				 const void *data,
				 int size)
{
	int ret;

	if (size > a->size)
		size = a->size;

	if (a->type == GBMS_ATOM_TYPE_MAP) {
		int i;
		const u16 *b = (u16 *)data;

		if (size % 2)
			return -ERANGE;

		for (i = 0; i < size / 2 ; i++) {
			ret = regmap_write(regmap, a->map[i], b[i]);
			if (ret < 0)
				break;
		}
	} else if (a->type == GBMS_ATOM_TYPE_SET) {
		ret = -EINVAL;
	} else {
		ret = regmap_raw_write(regmap, a->base, data, size);
	}

	return ret;
}

static int max17x0x_reg_load_sz(struct regmap *regmap,
				const struct max17x0x_reg *a,
				void *data,
				int size)
{
	int ret;

	if (size > a->size)
		size = a->size;

	if (a->type == GBMS_ATOM_TYPE_MAP) {
		int i;
		unsigned int tmp;
		u16 *b = (u16 *)data;

		if (size % 2)
			return -ERANGE;

		for (i = 0; i < size / 2 ; i++) {
			ret = regmap_read(regmap,
					  (unsigned int)a->map[i],
					  &tmp);
			if (ret < 0)
				break;
			b[i] = tmp;
		}
	} else if (a->type == GBMS_ATOM_TYPE_SET) {
		ret = -EINVAL;
	} else {
		ret = regmap_raw_read(regmap, a->base, data, size);
	}

	return ret;
}

#define max17x0x_reg_store(map, a, data) \
	max17x0x_reg_store_sz(map, a, data, (a)->size)

#define max17x0x_reg_load(map, a, data) \
	max17x0x_reg_load_sz(map, a, data, (a)->size)

/* CACHE ----------------------------------------------------------------- */

static int max17x0x_cache_index_of(const struct max17x0x_cache_data *cache,
				   unsigned int reg)
{
	const int offset = max17x0x_reg_offset_of(&cache->atom, reg);

	return (offset < 0) ? offset : offset / 2;
}

#define max17x0x_cache_store(cache, regmap) \
	max17x0x_reg_store(regmap, &(cache)->atom, (cache)->cache_data)

#define max17x0x_cache_load(cache, regmap) \
	max17x0x_reg_load(regmap, &(cache)->atom, (cache)->cache_data)

#define max17x0x_cache_memcmp(src, dst) \
	memcmp((src)->cache_data, (dst)->cache_data, (src)->atom.size)

static void max17x0x_cache_free(struct max17x0x_cache_data *cache)
{
	kfree(cache->cache_data);
	cache->cache_data = NULL;
}

static int max17x0x_cache_dup(struct max17x0x_cache_data *dst,
			      const struct max17x0x_cache_data *src)
{
	memcpy(dst, src, sizeof(*dst));

	dst->cache_data = (u16 *)kmalloc(src->atom.size, GFP_KERNEL);
	if (!dst->cache_data)
		return -ENOMEM;

	memcpy(dst->cache_data, src->cache_data, src->atom.size);
	return 0;
}

static int max17x0x_cache_init(struct max17x0x_cache_data *cache,
				     u16 start, int end)
{
	const int count = end - start + 1; /* includes end */

	memset(cache, 0, sizeof(*cache));

	cache->cache_data = (u16 *)kmalloc_array(count, sizeof(u16),
						 GFP_KERNEL);
	if (!cache->cache_data)
		return -ENOMEM;

	cache->atom.type = GBMS_ATOM_TYPE_ZONE;
	cache->atom.size = count * sizeof(u16);
	cache->atom.base = start;

	return 0;
}

static int max17x0x_nvram_cache_init(struct max17x0x_cache_data *cache,
				     int gauge_type)
{
	int ret;

	if (gauge_type == MAX1730X_GAUGE_TYPE) {
		ret = max17x0x_cache_init(cache,
					  MAX1730X_NVRAM_START,
					  MAX1730X_NVRAM_END);
	} else {
		ret = max17x0x_cache_init(cache,
					  MAX1720X_NVRAM_START,
					  MAX1720X_NVRAM_END);
	}

	return ret;
}

/** ------------------------------------------------------------------------ */

/* TODO: factor with the one in google_bms.c */
static char *psy_status_str[] = {
	"Unknown", "Charging", "Discharging", "NotCharging", "Full"
};

bool max1720x_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX17XXX_COMMAND:
	case MAX1720X_COMMSTAT:
	case MAX1720X_LOCK:
	case MAX1720X_ODSCTH:
	case MAX1720X_ODSCCFG:
	case MAX1720X_VFOCV:
	case MAX1720X_VFSOC:
	case MAX1720X_ALARM:
	case 0x00 ... 0x4F:
	case 0xB0 ... 0xDF:
		return true;
	}

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		switch (reg) {
		case 0xA0 ... 0xAE:
		case MAX1730X_NVPRTTH1BAK:
		case 0xF0:
		case 0xF5:
			return true;
		}
	}

	return false;
}

static const struct regmap_config max1720x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_VFSOC,
	.readable_reg = max1720x_is_reg,
	.volatile_reg = max1720x_is_reg,

};

bool max1720x_is_nvram_reg(struct device *dev, unsigned int reg)
{
	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		return (reg >= MAX1730X_NVRAM_START &&
			reg <= MAX1730X_HISTORY_END);
	else
		return (reg >= MAX1720X_NVRAM_START &&
			reg <= MAX1720X_NVRAM_HISTORY_END);
}

static const struct regmap_config max1720x_regmap_nvram_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_NVRAM_HISTORY_END,
	.readable_reg = max1720x_is_nvram_reg,
	.volatile_reg = max1720x_is_nvram_reg,
};

static const struct regmap_config max1730x_regmap_nvram_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1730X_HISTORY_END,
	.readable_reg = max1720x_is_nvram_reg,
	.volatile_reg = max1720x_is_nvram_reg,
};

static inline int reg_to_percentage(u16 val)
{
	/* LSB: 1/256% */
	return val >> 8;
}

static inline int reg_to_twos_comp_int(u16 val)
{
	/* Convert u16 to twos complement  */
	return -(val & 0x8000) + (val & 0x7FFF);
}

static inline int reg_to_micro_amp_h(s16 val, u16 rsense)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 500000, rsense);
}

static inline s16 micro_amp_h_to_reg(int val, u16 rsense)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64)val * rsense, 500000);
}

static inline int reg_to_micro_volt(u16 val)
{
	/* LSB: 0.078125mV */
	return div_u64((u64) val * 78125, 1000);
}

static inline int reg_to_micro_amp(s16 val, u16 rsense)
{
	/* LSB: 1.5625μV/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 156250, rsense);
}

static inline int reg_to_deci_deg_cel(s16 val)
{
	/* LSB: 1/256°C */
	return div_s64((s64) val * 10, 256);
}

static inline int reg_to_resistance_micro_ohms(s16 val, u16 rsense)
{
	/* LSB: 1/4096 Ohm */
	return div_s64((s64) val * 1000 * rsense, 4096);
}

static inline int reg_to_cycles(s16 val)
{
	/* LSB: 16% of one cycle */
	return DIV_ROUND_CLOSEST((int) val * 16, 100);
}

static inline int reg_to_seconds(s16 val)
{
	/* LSB: 5.625 seconds */
	return DIV_ROUND_CLOSEST((int) val * 5625, 1000);
}

static inline int reg_to_vempty(u16 val)
{
	return ((val >> 7) & 0x1FF) * 10;
}

static inline int reg_to_vrecovery(u16 val)
{
	return (val & 0x7F) * 40;
}

static void max1730x_read_log_write_status(const struct max1720x_chip *chip,
					   u16 *buffer)
{
	u16 i;
	u16 data = 0;
	const struct max17x0x_reg *hsty;

	hsty = max17x0x_find_by_tag(MAX17X0X_TAG_HSTY);
	if (!hsty)
		return;

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1730X_COMMAND_HISTORY_RECALL_WRITE_0);
	msleep(MAX1730X_TRECALL_MS);
	for (i = hsty->map[1]; i <= hsty->map[3]; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

static void max1730x_read_log_valid_status(const struct max1720x_chip *chip,
					   u16 *buffer)
{
	u16 i;
	u16 data = 0;
	const struct max17x0x_reg *hsty;

	hsty = max17x0x_find_by_tag(MAX17X0X_TAG_HSTY);

	if (!hsty)
		return;

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1730X_COMMAND_HISTORY_RECALL_VALID_0);
	msleep(MAX1730X_TRECALL_MS);
	(void) REGMAP_READ(chip->regmap_nvram, hsty->map[4], &data);
	*buffer++ = data;

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1730X_COMMAND_HISTORY_RECALL_VALID_1);
	msleep(MAX1730X_TRECALL_MS);
	for (i = hsty->map[0]; i <= hsty->map[2]; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

static void max1720x_read_log_write_status(const struct max1720x_chip *chip,
					   u16 *buffer)
{
	int i;
	u16 data = 0;

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_WRITE_0);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START;
	     i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_WRITE_1);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_HISTORY_START;
	     i <= MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

static void max1720x_read_log_valid_status(const struct max1720x_chip *chip,
					   u16 *buffer)
{
	int i;
	u16 data = 0;

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_0);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_HISTORY_VALID_STATUS_START;
	     i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_1);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_HISTORY_START;
	     i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_2);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_HISTORY_START;
	     i <= MAX1720X_NVRAM_HISTORY_VALID_STATUS_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

/* @return the number of pages or negative for error */
static int get_battery_history_status(const struct max1720x_chip *chip,
				      bool *page_status)
{
	u16 *write_status, *valid_status;
	int i, addr_offset, bit_offset, nb_history_pages;
	int valid_history_entry_count = 0;

	write_status = kmalloc_array(chip->nb_history_flag_reg,
				     sizeof(u16), GFP_KERNEL);
	if (!write_status)
		return -ENOMEM;

	valid_status = kmalloc_array(chip->nb_history_flag_reg,
				     sizeof(u16), GFP_KERNEL);
	if (!valid_status) {
		kfree(write_status);
		return -ENOMEM;
	}

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		max1730x_read_log_write_status(chip, write_status);
		max1730x_read_log_valid_status(chip, valid_status);
		nb_history_pages = MAX1730X_N_OF_HISTORY_PAGES;
	} else {
		max1720x_read_log_write_status(chip, write_status);
		max1720x_read_log_valid_status(chip, valid_status);
		nb_history_pages = MAX1720X_N_OF_HISTORY_PAGES;
	}

	/* Figure out the pages with valid history entry */
	for (i = 0; i < nb_history_pages; i++) {
		addr_offset = i / 8;
		bit_offset = i % 8;
		page_status[i] =
		    ((write_status[addr_offset] & BIT(bit_offset)) ||
		     (write_status[addr_offset] & BIT(bit_offset + 8))) &&
		    ((valid_status[addr_offset] & BIT(bit_offset)) ||
		     (valid_status[addr_offset] & BIT(bit_offset + 8)));
		if (page_status[i])
			valid_history_entry_count++;
	}

	kfree(write_status);
	kfree(valid_status);

	return valid_history_entry_count;
}

static void get_battery_history(const struct max1720x_chip *chip,
				bool *page_status, u16 *history)
{
	int i, j, index = 0;
	u16 data = 0;
	const struct max17x0x_reg *hsty;
	u16 command_base = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		? MAX1730X_READ_HISTORY_CMD_BASE
		: MAX1720X_READ_HISTORY_CMD_BASE;

	hsty = max17x0x_find_by_tag(MAX17X0X_TAG_HSTY);
	if (!hsty)
		return;

	for (i = 0; i < chip->nb_history_pages; i++) {
		if (!page_status[i])
			continue;
		REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
			     command_base + i);
		msleep(MAX1720X_TRECALL_MS);
		for (j = 0; j < chip->history_page_size; j++) {
			(void) REGMAP_READ(chip->regmap_nvram,
					   (unsigned int)hsty->map[0] + j,
					   &data);
			history[index * chip->history_page_size + j] = data;
		}
		index++;
	}
}

static int format_battery_history_entry(char *temp, int size, u16 *line,
					int count)
{
	int length = 0, i;

	for (i = 0; i < count; i++) {
		length += scnprintf(temp + length,
			size - length, "%04x ",
			line[i]);
	}

	if (length > 0)
		temp[--length] = 0;
	return length;
}

/* @return number of valid entries */
static int max1720x_history_read(struct max1720x_history *hi,
				 const struct max1720x_chip *chip)
{
	memset(hi, 0, sizeof(*hi));

	hi->page_status = kcalloc(chip->nb_history_pages,
				sizeof(bool), GFP_KERNEL);
	if (!hi->page_status)
		return -ENOMEM;

	hi->history_count = get_battery_history_status(chip, hi->page_status);
	if (hi->history_count < 0) {
		goto error_exit;
	} else if (hi->history_count != 0) {
		const int size = hi->history_count * chip->history_page_size;

		hi->page_size = chip->history_page_size;
		hi->history = kmalloc_array(size, sizeof(u16), GFP_KERNEL);
		if (!hi->history) {
			hi->history_count = -ENOMEM;
			goto error_exit;
		}

		get_battery_history(chip, hi->page_status, hi->history);
	}

	return hi->history_count;

error_exit:
	kfree(hi->page_status);
	hi->page_status = NULL;
	return hi->history_count;

}

static void max1720x_history_free(struct max1720x_history *hi)
{
	kfree(hi->page_status);
	kfree(hi->history);

	hi->history = NULL;
	hi->page_status = NULL;
	hi->history_count = -1;
	hi->history_index = 0;
}

static enum power_supply_property max1720x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_RAW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_DELTA_CC_SUM,
	POWER_SUPPLY_PROP_DELTA_VFSOC_SUM,
	POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE,
	POWER_SUPPLY_PROP_RES_FILTER_COUNT,
	POWER_SUPPLY_PROP_RESISTANCE_AVG,
};

/* ------------------------------------------------------------------------- */

static ssize_t max1720x_get_offmode_charger(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%hhd\n", chip->offmode_charger);
}

static ssize_t max1720x_set_offmode_charger(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);

	if (kstrtobool(buf, &chip->offmode_charger))
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(offmode_charger, 0660,
		   max1720x_get_offmode_charger,
		   max1720x_set_offmode_charger);

/* lsb 1/256 */
static int max1720x_get_capacity_raw(struct max1720x_chip *chip, u16 *data)
{
	return REGMAP_READ(chip->regmap, chip->reg_prop_capacity_raw, data);
}

static int max1720x_get_battery_soc(struct max1720x_chip *chip)
{
	u16 data;
	int capacity, err;

	if (chip->fake_capacity >= 0 && chip->fake_capacity <= 100)
		return chip->fake_capacity;

	err = REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &data);
	if (err)
		return err;
	capacity = reg_to_percentage(data);

	if (capacity == 100 && chip->offmode_charger)
		chip->fake_capacity = 100;

	return capacity;
}

static int max1720x_get_battery_vfsoc(struct max1720x_chip *chip)
{
	u16 data;
	int capacity, err;

	err = REGMAP_READ(chip->regmap, MAX1720X_VFSOC, &data);
	if (err)
		return err;
	capacity = reg_to_percentage(data);

	return capacity;
}

static void max1720x_prime_battery_qh_capacity(struct max1720x_chip *chip,
					       int status)
{
	u16 data = 0;

	(void)max17x0x_reg_read(chip->regmap, MAX17X0X_TAG_mcap, &data);
	chip->current_capacity = data;

	REGMAP_WRITE(chip->regmap_nvram, MAX17XXX_QHCA, ~data);
	dev_info(chip->dev, "Capacity primed to %d on %s\n",
		 data, psy_status_str[status]);

	(void) REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	chip->previous_qh = reg_to_twos_comp_int(data);

	REGMAP_WRITE(chip->regmap_nvram, MAX17XXX_QHQH, data);
	dev_info(chip->dev, "QH primed to %d on %s\n",
		 data, psy_status_str[status]);
}

/* NOTE: the gauge doesn't know if we are current limited to */
static int max1720x_get_battery_status(struct max1720x_chip *chip)
{
	u16 data = 0;
	int current_now, current_avg, ichgterm, vfsoc, soc, fullsocthr;
	int status = POWER_SUPPLY_STATUS_UNKNOWN, err;

	err = max17x0x_reg_read(chip->regmap, MAX17X0X_TAG_curr, &data);
	if (err)
		return err;
	current_now = -reg_to_micro_amp(data, chip->RSense);

	err = max17x0x_reg_read(chip->regmap, MAX17X0X_TAG_avgc, &data);
	if (err)
		return err;
	current_avg = -reg_to_micro_amp(data, chip->RSense);

	err = REGMAP_READ(chip->regmap, MAX1720X_ICHGTERM, &data);
	if (err)
		return err;
	ichgterm = reg_to_micro_amp(data, chip->RSense);

	err = REGMAP_READ(chip->regmap, MAX1720X_FULLSOCTHR, &data);
	if (err)
		return err;
	fullsocthr = reg_to_percentage(data);

	soc = max1720x_get_battery_soc(chip);
	if (soc < 0)
		return -EIO;

	vfsoc = max1720x_get_battery_vfsoc(chip);
	if (vfsoc < 0)
		return -EIO;

	if (current_avg > -ichgterm && current_avg <= 0) {

		if (soc >= fullsocthr) {
			const bool needs_prime = (chip->prev_charge_status ==
						  POWER_SUPPLY_STATUS_CHARGING);

			status = POWER_SUPPLY_STATUS_FULL;
			if (needs_prime)
				max1720x_prime_battery_qh_capacity(chip,
								   status);
		} else {
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

	} else if (current_now >= -ichgterm)  {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_CHARGING;
		if (chip->prev_charge_status == POWER_SUPPLY_STATUS_DISCHARGING
		    && current_avg  < -ichgterm)
			max1720x_prime_battery_qh_capacity(chip, status);
	}

	if (status != chip->prev_charge_status)
		dev_info(chip->dev, "s=%d->%d c=%d avg_c=%d ichgt=%d vfsoc=%d soc=%d fullsocthr=%d\n",
				    chip->prev_charge_status,
				    status, current_now, current_avg,
				    ichgterm, vfsoc, soc, fullsocthr);

	chip->prev_charge_status = status;

	return status;
}

static int max1720x_get_battery_health(struct max1720x_chip *chip)
{
	/* For health report what ever was recently alerted and clear it */
	if (chip->health >= POWER_SUPPLY_HEALTH_UNKNOWN)
		return chip->health;

	if (chip->health_status & MAX1720X_STATUS_VMX) {
		chip->health_status &= ~MAX1720X_STATUS_VMX;
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}

	if (chip->health_status & MAX1720X_STATUS_TMN) {
		chip->health_status &= ~MAX1720X_STATUS_TMN;
		return POWER_SUPPLY_HEALTH_COLD;
	}

	if (chip->health_status & MAX1720X_STATUS_TMX) {
		chip->health_status &= ~MAX1720X_STATUS_TMX;
		return POWER_SUPPLY_HEALTH_HOT;
	}

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int max1720x_update_battery_qh_based_capacity(struct max1720x_chip *chip)
{
	u16 data;
	int current_qh, err = 0;

	err = REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	if (err)
		return err;
	current_qh = reg_to_twos_comp_int(data);

	/* QH value accumulates as battery charges */
	chip->current_capacity -= (chip->previous_qh - current_qh);
	chip->previous_qh = current_qh;

	return 0;
}

static void max1720x_restore_battery_qh_capacity(struct max1720x_chip *chip)
{
	u16 data = 0, nvram_capacity;
	int current_qh, nvram_qh;

	(void) REGMAP_READ(chip->regmap_nvram, MAX17XXX_QHCA, &data);
	nvram_capacity = ~data;

	(void) REGMAP_READ(chip->regmap_nvram, MAX17XXX_QHQH, &data);
	nvram_qh = reg_to_twos_comp_int(data);

	(void) REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	current_qh = reg_to_twos_comp_int(data);

	/* QH value accumulates as battery discharges */
	chip->current_capacity = (int) nvram_capacity - (nvram_qh - current_qh);
	dev_info(chip->dev, "Capacity restored to %d\n",
		 chip->current_capacity);
	chip->previous_qh = current_qh;
	dev_info(chip->dev, "QH value restored to %d\n",
		 chip->previous_qh);
}

static void max1720x_handle_update_nconvgcfg(struct max1720x_chip *chip,
					     int temp)
{
	int idx = -1, hysteresis_temp;

	if (chip->temp_convgcfg == NULL)
		return;

	if (temp <= chip->temp_convgcfg[0]) {
		idx = 0;
	} else if (temp > chip->temp_convgcfg[chip->nb_convgcfg - 1]) {
		idx = chip->nb_convgcfg - 1;
	} else {
		for (idx = 1 ; idx < chip->nb_convgcfg; idx++) {
			if (temp > chip->temp_convgcfg[idx - 1] &&
			    temp <= chip->temp_convgcfg[idx])
				break;
		}
	}
	mutex_lock(&chip->convgcfg_lock);
	/* We want to switch to higher slot only if above temp + hysteresis
	 * but when temperature drops, we want to change at the level
	 */
	hysteresis_temp = chip->temp_convgcfg[chip->curr_convgcfg_idx] +
			  chip->convgcfg_hysteresis;
	if ((idx != chip->curr_convgcfg_idx) &&
	    (chip->curr_convgcfg_idx == -1 || idx < chip->curr_convgcfg_idx ||
	     temp >= hysteresis_temp)) {
		REGMAP_WRITE(chip->regmap_nvram, MAX1720X_NCONVGCFG,
			     chip->convgcfg_values[idx]);
		chip->curr_convgcfg_idx = idx;
		dev_info(chip->dev, "updating nConvgcfg to 0x%04x as temp is %d (idx:%d)\n",
			 chip->convgcfg_values[idx], temp, idx);
	}
	mutex_unlock(&chip->convgcfg_lock);
}

#define MAXIM_CYCLE_COUNT_RESET 655
#define MAX17201_HIST_CYCLE_COUNT_OFFSET	0x4
#define MAX17201_HIST_TIME_OFFSET		0xf

static int max1720x_get_cycle_count_offset(struct max1720x_chip *chip)
{
	int offset = 0, i, history_count;
	struct max1720x_history hi;

	if (!chip->history_page_size)
		return 0;

	mutex_lock(&chip->history_lock);
	history_count = max1720x_history_read(&hi, chip);
	if (history_count < 0) {
		mutex_unlock(&chip->history_lock);
		return 0;
	}

	for (i = 0; i < history_count; i++) {
		u16 *entry = &hi.history[i * chip->history_page_size];

		if (entry[MAX17201_HIST_CYCLE_COUNT_OFFSET] == 0 &&
		    entry[MAX17201_HIST_TIME_OFFSET] != 0) {
			offset += MAXIM_CYCLE_COUNT_RESET;
			break;
		}
	}
	mutex_unlock(&chip->history_lock);

	dev_dbg(chip->dev, "history_count=%d page_size=%d i=%d offset=%d\n",
		history_count, chip->history_page_size, i, offset);

	max1720x_history_free(&hi);
	return offset;
}

static int max1720x_get_cycle_count(struct max1720x_chip *chip)
{
	int err, cycle_count;
	u16 temp;

	err = REGMAP_READ(chip->regmap, MAX1720X_CYCLES, &temp);
	if (err < 0)
		return err;

	cycle_count = reg_to_cycles(temp);
	if ((chip->cycle_count == -1) ||
	    ((cycle_count + chip->cycle_count_offset) < chip->cycle_count))
		chip->cycle_count_offset =
			max1720x_get_cycle_count_offset(chip);

	chip->cycle_count = cycle_count + chip->cycle_count_offset;

	return chip->cycle_count;
}

static void max1720x_handle_update_empty_voltage(struct max1720x_chip *chip,
						 int temp)
{
	int cycle, cycle_idx, temp_idx, chg_st, ret = 0;
	u16 empty_volt_cfg, reg, vempty = 0;

	if (chip->empty_voltage == NULL)
		return;

	chg_st = max1720x_get_battery_status(chip);
	if (chg_st < 0)
		return;

	cycle = max1720x_get_cycle_count(chip);
	if (cycle < 0)
		return;

	ret = REGMAP_READ(chip->regmap, MAX1720X_VEMPTY, &vempty);
	if (ret < 0)
		return;

	cycle_idx = cycle / CYCLE_BUCKET_SIZE;
	if (cycle_idx > (NB_CYCLE_BUCKETS - 1))
		cycle_idx = NB_CYCLE_BUCKETS - 1;

	if (temp < 0) {
		temp_idx = 0;
	} else {
		const int idx = temp / TEMP_BUCKET_SIZE + 1;
		const int temp_buckets = chip->nb_empty_voltage /
					 NB_CYCLE_BUCKETS;

		temp_idx = idx < (temp_buckets - 1) ? idx : (temp_buckets - 1);
	}

	empty_volt_cfg = MAX1720_EMPTY_VOLTAGE(chip, temp_idx, cycle_idx);
	reg = (empty_volt_cfg / 10) << 7 | (vempty & 0x7F);
	if ((reg > vempty) ||
	    (reg < vempty && chg_st != POWER_SUPPLY_STATUS_DISCHARGING)) {
		REGMAP_WRITE(chip->regmap, MAX1720X_VEMPTY, reg);

		pr_debug("updating empty_voltage to %d(0x%04X), temp:%d(%d), cycle:%d(%d)\n",
				empty_volt_cfg, reg,
				temp, temp_idx,
				cycle, cycle_idx);
	}
}

/* Capacity Estimation functions*/
static int batt_ce_regmap_read(struct regmap *map, u32 reg, u16 *data)
{
	int err = -EINVAL;
	u16 val;
	const struct max17x0x_reg *bcea;

	bcea = max17x0x_find_by_tag(MAX17X0X_TAG_BCEA);
	if (!bcea)
		return err;

	switch(reg) {
	case CE_DELTA_CC_SUM_REG:
	case CE_DELTA_VFSOC_SUM_REG:
		err = REGMAP_READ(map, bcea->map[reg], &val);
		if (err)
			return err;
		*data = val;
		break;
	case CE_CAP_FILTER_COUNT:
		err = REGMAP_READ(map, bcea->map[reg], &val);
		if (err)
			return err;
		val = val & 0x0F00;
		*data = val >> 8;
		break;
	default:
		break;
	}

	return err;
}

static int batt_ce_regmap_write(struct regmap *map, u32 reg, u16 data)
{
	int err = -EINVAL;
	u16 val;
	const struct max17x0x_reg *bcea;

	bcea = max17x0x_find_by_tag(MAX17X0X_TAG_BCEA);
	if (!bcea)
		return err;

	switch(reg) {
	case CE_DELTA_CC_SUM_REG:
	case CE_DELTA_VFSOC_SUM_REG:
		err = REGMAP_WRITE(map, bcea->map[reg], data);
		break;
	case CE_CAP_FILTER_COUNT:
		err = REGMAP_READ(map, bcea->map[reg], &val);
		if (err)
			return err;
		val = val & 0xF0FF;
		if (data > CE_FILTER_COUNT_MAX)
			val = val | 0x0F00;
		else
			val = val | (data << 8);
		err = REGMAP_WRITE(map, bcea->map[reg], val);
		break;
	default:
		break;
	}

	return err;
}

static void batt_ce_dump_data(const struct gbatt_capacity_estimation *cap_esti,
			      struct logbuffer *log)
{
	logbuffer_log(log, "cap_filter_count: %d"
			    " start_cc: %d"
			    " start_vfsoc: %d"
			    " delta_cc_sum: %d"
			    " delta_vfsoc_sum: %d"
			    " state: %d"
			    " cable: %d\n",
			    cap_esti->cap_filter_count,
			    cap_esti->start_cc,
			    cap_esti->start_vfsoc,
			    cap_esti->delta_cc_sum,
			    cap_esti->delta_vfsoc_sum,
			    cap_esti->estimate_state,
			    cap_esti->cable_in);
}

static int batt_ce_load_data(struct regmap *map,
			      struct gbatt_capacity_estimation *cap_esti)
{
	u16 data;

	if (max17xxx_gauge_type == -1)
		return -EINVAL;

	cap_esti->estimate_state = ESTIMATE_NONE;
	if (batt_ce_regmap_read(map, CE_DELTA_CC_SUM_REG, &data) == 0)
		cap_esti->delta_cc_sum = data;
	else
		cap_esti->delta_cc_sum = 0;

	if (batt_ce_regmap_read(map, CE_DELTA_VFSOC_SUM_REG, &data) == 0)
		cap_esti->delta_vfsoc_sum = data;
	else
		cap_esti->delta_vfsoc_sum = 0;

	if (batt_ce_regmap_read(map, CE_CAP_FILTER_COUNT, &data) == 0)
		cap_esti->cap_filter_count = data;
	else
		cap_esti->cap_filter_count = 0;
	return 0;
}

/* call holding &cap_esti->batt_ce_lock */
static void batt_ce_store_data(struct regmap *map,
			       struct gbatt_capacity_estimation *cap_esti)
{
	if (cap_esti->cap_filter_count <= CE_FILTER_COUNT_MAX) {
		batt_ce_regmap_write(map, CE_CAP_FILTER_COUNT,
				     cap_esti->cap_filter_count);
	}

	batt_ce_regmap_write(map, CE_DELTA_VFSOC_SUM_REG,
				  cap_esti->delta_vfsoc_sum);
	batt_ce_regmap_write(map, CE_DELTA_CC_SUM_REG,
				  cap_esti->delta_cc_sum);
}

/* call holding &cap_esti->batt_ce_lock */
static void batt_ce_stop_estimation(struct gbatt_capacity_estimation *cap_esti,
				   int reason)
{
	cap_esti->estimate_state = reason;
	cap_esti->start_vfsoc = 0;
	cap_esti->start_cc = 0;
}

static int batt_ce_full_estimate(struct gbatt_capacity_estimation *ce)
{
	return (ce->cap_filter_count > 0) && (ce->delta_vfsoc_sum > 0) ?
		ce->delta_cc_sum / ce->delta_vfsoc_sum : -1;
}

/* Measure the deltaCC, deltaVFSOC and CapacityFiltered */
static void batt_ce_capacityfiltered_work(struct work_struct *work)
{
	struct max1720x_chip *chip = container_of(work, struct max1720x_chip,
					    cap_estimate.settle_timer.work);
	struct gbatt_capacity_estimation *cap_esti = &chip->cap_estimate;
	int settle_cc = 0, settle_vfsoc = 0;
	int delta_cc = 0, delta_vfsoc = 0;
	int cc_sum = 0, vfsoc_sum = 0;
	bool valid_estimate = false;
	int rc = 0;
	int data;

	mutex_lock(&cap_esti->batt_ce_lock);

	/* race with disconnect */
	if (!cap_esti->cable_in ||
	    cap_esti->estimate_state != ESTIMATE_PENDING) {
		goto exit;
	}

	rc = max1720x_update_battery_qh_based_capacity(chip);
	if (rc < 0)
		goto ioerr;

	settle_cc = reg_to_micro_amp_h(chip->current_capacity, chip->RSense);

	data = max1720x_get_battery_vfsoc(chip);
	if (data < 0)
		goto ioerr;

	settle_vfsoc = data;
	settle_cc = settle_cc / 1000;
	delta_cc = settle_cc - cap_esti->start_cc;
	delta_vfsoc = settle_vfsoc - cap_esti->start_vfsoc;

	if ((delta_cc > 0) && (delta_vfsoc > 0)) {

		cc_sum = delta_cc + cap_esti->delta_cc_sum;
		vfsoc_sum = delta_vfsoc + cap_esti->delta_vfsoc_sum;

		if (cap_esti->cap_filter_count >= cap_esti->cap_filt_length) {
			const int filter_divisor = cap_esti->cap_filt_length;

			cc_sum -= cap_esti->delta_cc_sum/filter_divisor;
			vfsoc_sum -= cap_esti->delta_vfsoc_sum/filter_divisor;
		}

		cap_esti->cap_filter_count++;
		cap_esti->delta_cc_sum = cc_sum;
		cap_esti->delta_vfsoc_sum = vfsoc_sum;
		batt_ce_store_data(chip->regmap_nvram, &chip->cap_estimate);

		valid_estimate = true;
	}

ioerr:
	batt_ce_stop_estimation(cap_esti, ESTIMATE_DONE);

exit:
	logbuffer_log(chip->ce_log,
		"valid=%d settle[cc=%d, vfsoc=%d], delta[cc=%d,vfsoc=%d] ce[%d]=%d\n",
		valid_estimate,
		settle_cc, settle_vfsoc, delta_cc, delta_vfsoc,
		cap_esti->cap_filter_count,
		batt_ce_full_estimate(cap_esti));

	mutex_unlock(&cap_esti->batt_ce_lock);

	/* force to update uevent to framework side. */
	if (valid_estimate)
		power_supply_changed(chip->psy);
}

/* batt_ce_init(): estimate_state = ESTIMATE_NONE
 * batt_ce_start(): estimate_state = ESTIMATE_NONE -> ESTIMATE_PENDING
 * batt_ce_capacityfiltered_work(): ESTIMATE_PENDING->ESTIMATE_DONE
 */

static int batt_ce_start(struct gbatt_capacity_estimation *cap_esti,
			 int cap_tsettle_ms)
{
	mutex_lock(&cap_esti->batt_ce_lock);

	/* Still has cable and estimate is not pending or cancelled */
	if (!cap_esti->cable_in || cap_esti->estimate_state != ESTIMATE_NONE)
		goto done;

	pr_info("EOC: Start the settle timer\n");
	cap_esti->estimate_state = ESTIMATE_PENDING;
	schedule_delayed_work(&cap_esti->settle_timer,
		msecs_to_jiffies(cap_tsettle_ms));

done:
	mutex_unlock(&cap_esti->batt_ce_lock);
	return 0;
}

static int batt_ce_init(struct gbatt_capacity_estimation *cap_esti,
			struct max1720x_chip *chip)
{
	int rc, vfsoc;

	rc = max1720x_update_battery_qh_based_capacity(chip);
	if (rc < 0)
		return -EIO;

	vfsoc = max1720x_get_battery_vfsoc(chip);
	if (vfsoc < 0)
		return -EIO;

	cap_esti->start_vfsoc = vfsoc;
	cap_esti->start_cc = reg_to_micro_amp_h(chip->current_capacity,
						chip->RSense) / 1000;
	/* Capacity Estimation starts only when the state is NONE */
	cap_esti->estimate_state = ESTIMATE_NONE;
	return 0;
}

/* ------------------------------------------------------------------------- */
#define SEL_RES_AVG		0
#define SEL_RES_FILTER_COUNT	1
static int batt_res_registers(struct max1720x_chip *chip, bool bread,
			      int isel, int *data)
{
	int err = -EINVAL;
	const struct max17x0x_reg *bres;
	u16 res_filtered, res_filt_count, val;

	bres = max17x0x_find_by_tag(MAX17X0X_TAG_BRES);
	if (!bres)
		return err;

	switch (isel) {
	case SEL_RES_AVG:
		if (bread) {
			err = REGMAP_READ(chip->regmap_nvram, bres->map[0],
					  &res_filtered);
			if (err)
				return err;

			*data = res_filtered;
			return 0;
		}
		err = REGMAP_WRITE(chip->regmap_nvram, bres->map[0], *data);
		break;
	case SEL_RES_FILTER_COUNT:
		err = REGMAP_READ(chip->regmap_nvram, bres->map[1], &val);
		if (err)
			return err;

		if (bread) {
			res_filt_count = (val & 0xF000) >> 12;
			*data = res_filt_count;
			return 0;
		}
		res_filt_count = (val & 0x0FFF) | (*data << 12);
		err = REGMAP_WRITE(chip->regmap_nvram, bres->map[1],
				   res_filt_count);
		break;
	default:
		break;
	}

	return err;
}

static int max1720x_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)
					power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	u16 data = 0;
	int err = 0;
	int idata;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max1720x_get_battery_status(chip);
		if (val->intval < 0)
			return val->intval;

		/* Capacity estimation must run only once.
		 * NOTE: this is a getter with a side effect
		 */
		if (val->intval == POWER_SUPPLY_STATUS_FULL)
			batt_ce_start(&chip->cap_estimate,
				      chip->cap_estimate.cap_tsettle);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max1720x_get_battery_health(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		err = max1720x_get_capacity_raw(chip, &data);
		if (err == 0)
			val->intval = (int)data;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max1720x_get_battery_soc(chip);
		if (val->intval < 0)
			return val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		err = max1720x_update_battery_qh_based_capacity(chip);
		if (err < 0)
			return err;
		val->intval = reg_to_micro_amp_h(chip->current_capacity,
						 chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/*
		 * Snap charge_full to DESIGNCAP during early charge cycles to
		 * prevent large fluctuations in FULLCAPNOM. MAX1720X_CYCLES LSB
		 * is 16%
		 */
		err = max1720x_get_cycle_count(chip);
		if (err >= 0) {
			const int cycle_count = err;

			if (cycle_count <= FULLCAPNOM_STABILIZE_CYCLES)
				err = REGMAP_READ(map, MAX1720X_DESIGNCAP,
						  &data);
			else
				err = REGMAP_READ(map, MAX1720X_FULLCAPNOM,
						  &data);

			val->intval = reg_to_micro_amp_h(data, chip->RSense);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		err = REGMAP_READ(map, MAX1720X_DESIGNCAP, &data);
		val->intval = reg_to_micro_amp_h(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_avgc, &data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_curr, &data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = max1720x_get_cycle_count(chip);
		if (val->intval < 0)
			err = val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (max17xxx_gauge_type == -1) {
			val->intval = 0;
		} else if (chip->fake_battery != -1) {
			val->intval = chip->fake_battery;
		} else {
			err = REGMAP_READ(map, MAX1720X_STATUS, &data);
			/* BST is 0 when the battery is present */
			val->intval = (((u16) data) & MAX1720X_STATUS_BST)
							? 0 : 1;
		}
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = chip->batt_id;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		err = REGMAP_READ(map, MAX1720X_RCELL, &data);
		val->intval = reg_to_resistance_micro_ohms(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_temp, &data);
		val->intval = reg_to_deci_deg_cel(data);
		max1720x_handle_update_nconvgcfg(chip, val->intval);
		max1720x_handle_update_empty_voltage(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		err = REGMAP_READ(map, MAX1720X_TTE, &data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		err = REGMAP_READ(map, MAX1720X_TTF, &data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		err = REGMAP_READ(map, MAX1720X_AVGVCELL, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_mmdv, &data);
		/* LSB: 20mV */
		val->intval = ((data >> 8) & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_mmdv, &data);
		/* LSB: 20mV */
		val->intval = (data & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		err = max17x0x_reg_read(map, MAX17X0X_TAG_vcel, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		err = REGMAP_READ(map, MAX1720X_VFOCV, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = chip->serial_number;
		break;
	case POWER_SUPPLY_PROP_DELTA_CC_SUM:
		val->intval = chip->cap_estimate.delta_cc_sum;
		break;
	case POWER_SUPPLY_PROP_DELTA_VFSOC_SUM:
		val->intval = chip->cap_estimate.delta_vfsoc_sum;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE:
		val->intval = batt_ce_full_estimate(&chip->cap_estimate);
		if (val->intval > 0)
			val->intval *= 100000;
		break;
	case POWER_SUPPLY_PROP_RES_FILTER_COUNT:
		err = batt_res_registers(chip, true, SEL_RES_FILTER_COUNT,
					 &idata);
		val->intval = idata;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_AVG:
		err = batt_res_registers(chip, true, SEL_RES_AVG, &idata);
		val->intval = idata;
		break;
	default:
		return -EINVAL;
	}
	if (err < 0)
		return err;
	return 0;
}

/* ------------------------------------------------------------------------- */

/* 1 = success, 0 compare error, < 0 error */
static int max1720x_update_compare(struct regmap *map, int reg,
				 u16 data0, u16 data1)
{
	u16 data[2] = {data0, data1};
	int ret;

	ret = regmap_raw_write(map, reg, data, sizeof(data));
	if (ret < 0)
		return -EIO;

	msleep(2);

	ret = regmap_raw_read(map, reg, data, sizeof(data));
	if (ret < 0)
		return -EIO;

	return (data[0] == data0) && (data[1] == data1);
}

/* 0 not updated, 1 updated, doesn't return IO errors */
static int max1720x_update_capacity(struct regmap *map, u16 mixcap, u16 repcap,
				    u16 fullcaprep)
{
	u16 temp;
	int err;

	err = REGMAP_WRITE(map, MAX1720X_MIXCAP, mixcap);
	if (err == 0)
		err = REGMAP_READ(map, MAX1720X_MIXCAP, &temp);
	if (err < 0 || temp != mixcap)
		return 0;

	/* RepCap and FullCapRep must be updated together */
	err = REGMAP_WRITE(map, MAX1720X_REPCAP, repcap);
	if (err == 0)
		err = REGMAP_READ(map, MAX1720X_REPCAP, &temp);
	if (err < 0 || temp != repcap)
		return 0;

	err = REGMAP_WRITE(map, MAX1720X_FULLCAPREP, fullcaprep);
	if (err == 0)
		err = REGMAP_READ(map, MAX1720X_FULLCAPREP, &temp);
	if (err < 0 || temp != fullcaprep)
		return 0;

	return 1;
}

/* max 110% of design cap, max 60% of design cap for age model */
#define MAX1720x_CC_UPPER_BOUND	110
#define MAX1720x_CC_LOWER_BOUND	60

/* return fullcapnom if no changes */
static int max1720x_capacity_check(int fullcapnom, int cycle_count,
				   const struct max1720x_chip *chip)
{
	const int fcn = fullcapnom;
	int refcap, upper_bound, lower_bound;

	if (!chip->design_capacity || cycle_count < chip->cycle_stable)
		return fullcapnom;

	/* apply fade model to design capacity, bound to absolute min/max */
	refcap = chip->design_capacity;
	if (chip->cycle_fade) {
		const int base_capacity = chip->design_capacity;

		lower_bound = (base_capacity * MAX1720x_CC_LOWER_BOUND) / 100;
		upper_bound = (base_capacity * MAX1720x_CC_UPPER_BOUND) / 100;

		refcap -= (base_capacity * cycle_count) / chip->cycle_fade;
		if (refcap < lower_bound)
			refcap = lower_bound;
		else if (refcap > upper_bound)
			refcap = upper_bound;

		pr_debug("refcap@%d=%d abs_min=%d abs_max=%d\n",
			cycle_count, refcap, lower_bound, upper_bound);
	}

	/* bound FCN max to the target age. Will not operate on devices
	 * that underestimate capacity.
	 * NOTE: range decrease with cycle count
	 */
	upper_bound = (refcap * (100 + chip->cycle_band)) / 100;
	if (fullcapnom > upper_bound)
		fullcapnom = upper_bound;

	pr_debug("fullcapnom=%d->%d upper_bound=%d\n",
		 fcn, fullcapnom, upper_bound);

	return fullcapnom;
}

/**
 * dQACC @0x45 battery charge between relaxation points.
 * dPACC @0x46 change in battery state of charge between relaxation points.
 */
/* 1 changed, 0 no changes, < 0 error*/
static int max1720x_fixup_dxacc(int plugged, struct max1720x_chip *chip)
{
	u16 temp, fullcapnom, mixcap, repcap, fcrep;
	u16 vfsoc = 0, repsoc = 0, avsoc = 0;
	int cycle_count, capacity, new_capacity;
	int err, loops;
	int dpacc, dqacc;

	if (chip->design_capacity <= 0)
		return 0;

	err = REGMAP_READ(chip->regmap, MAX1720X_FULLCAPNOM, &fullcapnom);
	if (err < 0)
		return err;
	capacity = reg_to_micro_amp_h(fullcapnom, chip->RSense) / 1000;

	cycle_count = max1720x_get_cycle_count(chip);
	if (cycle_count < 0)
		return cycle_count;

	/* return the expected FCN, done if the same of th eold one */
	new_capacity = max1720x_capacity_check(capacity, cycle_count, chip);
	if (new_capacity == capacity)
		return 0;

	/* You can use a ratio of dPAcc = 0x190 ( = 25%) with dQACC with 64 mAh
	 * LSB. Can make dPACC larger (ex 0xC80, 200%) and give dQAcc a smaller
	 * LSB (FullCapNom >> 4, LSB = 8 mAh). The equation can be written a
	 * (DesignCap * scale) >> 4 when writing the age-compensated value.
	 */
	fcrep = micro_amp_h_to_reg(new_capacity * 1000, chip->RSense);
	dqacc = fcrep >> 4;
	dpacc = 0xc80;

	/* will not update if dqacc/dpacc is already in line */
	err = REGMAP_READ(chip->regmap, MAX1720X_DQACC, &temp);
	if (err == 0 && temp == dqacc) {
		err = REGMAP_READ(chip->regmap, MAX1720X_DPACC, &temp);
		if (err == 0 && temp == dpacc) {
			pr_debug("Fix capacity: same dqacc=0x%x dpacc=0x%x\n",
				 dqacc, dpacc);
			return 0;
		}
	}

	/* fast convergence, avoid ghost drain */
	err = REGMAP_READ(chip->regmap, MAX1720X_VFSOC, &vfsoc);
	if (err == 0)
		err = REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &repsoc);
	if (err < 0) {
		pr_warn("Fix capacity: fcn=%d new=%d vfsoc=0x%x repsoc=0x%x (%d)\n",
			fullcapnom, new_capacity, vfsoc, repsoc, err);
		return err;
	}

	/* b/146218573: use avsoc in special charging profile */
	if (chip->health == POWER_SUPPLY_HEALTH_OVERHEAT && plugged) {
		chip->reg_prop_capacity_raw = MAX1720X_AVSOC;
	} else if (chip->reg_prop_capacity_raw == MAX1720X_AVSOC) {
		err = REGMAP_READ(chip->regmap, MAX1720X_AVSOC, &avsoc);
		if (err == 0) {
			err = REGMAP_WRITE(chip->regmap, MAX1720X_REPSOC,
					   avsoc);
			if (err == 0)
				repsoc = avsoc;
			else
				pr_warn("Fail to write AvSOC to RepSOC\n");
		}
		chip->reg_prop_capacity_raw = MAX1720X_REPSOC;
	}

	/* vfsoc/repsoc perc, lsb = 1/256 */
	mixcap = (((u32)vfsoc) * fcrep) / 25600;
	repcap = (((u32)repsoc) * fcrep) / 25600;

	for (loops = 0; loops < 3; loops++) {
		err = max1720x_update_capacity(chip->regmap, mixcap, repcap,
					       fcrep);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	dev_info(chip->dev, "Fix capacity: fixing caps retries=%d (%d)\n",
		 loops, err);

	/* 3 loops suggested from vendor */
	for (loops = 0; loops < 3; loops++) {
		err = max1720x_update_compare(chip->regmap, MAX1720X_DQACC,
					      dqacc, dpacc);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	dev_info(chip->dev, "Fix capacity: %d->%d, vfsoc=0x%x repsoc=0x%x "
		"fcrep=0x%x mixcap=0x%x repcap=0x%x ddqacc=0x%x dpacc=0x%x "
		"retries=%d raw_reg:0x%x (%d)\n",
		 fullcapnom, new_capacity, vfsoc, repsoc, fcrep, mixcap, repcap,
		 dqacc, dpacc, loops, chip->reg_prop_capacity_raw, err);

	/* TODO:  b/144630261 fix Google Capacity */

	return (loops== 3) ? -ETIMEDOUT : err;
}

/* Tempco and rcomp0 must remain within the following limits to avoid capacity
 * drift. Note that tempco has a hi and low limit (one byte).
 * (RCOMP0 > INI_RCOMP0 * 1.4) or (RCOMP0 < 0.7 * INI_RCOMP0)
 * (TempCoHot >INI_TempCoHot * 1.4) or (TempCoHot < 0.7 * INI_TempCoHot)
 * (TempCoCold >INI_TempCoCold * 1.4) or (TempCoCold < 0.7 * INI_TempCoCold)
 */
#define MAXIM_RCOMP0_LIM_HI	140
#define MAXIM_RCOMP0_LIM_LO	70
#define MAXIM_TEMPCO_LIM_HI	140
#define MAXIM_TEMPCO_LIM_LO 	70

static u8 comp_check(int value, int scale, int lim_low, int lim_high)
{
	if ((value * scale) < lim_low) {
		value = lim_low / scale;
	} else if ((value * scale) > lim_high) {
		value = lim_high / scale;
		if (value > 0xff)
			value = 0xff;
	}

	return value;
}

/* 0 no changes, >0 changes */
static bool max1720x_comp_check(u16 *new_rcomp0, u16 *new_tempco,
				const struct max1720x_chip *chip)
{
	const u16 rcomp0 = *new_rcomp0;
	const int ini_rcomp0_lob = chip->ini_rcomp0 & 0xff;
	int rcomp0_lob = rcomp0 & 0xff;
	const u16 tempco = *new_tempco;
	const int ini_tc_lob = chip->ini_tempco & 0xff;
	const int ini_tc_hib = (chip->ini_tempco >> 8) & 0xff;
	int tc_hib = (tempco >> 8) & 0xff;
	int tc_lob = tempco & 0xff;

	rcomp0_lob = comp_check(rcomp0_lob, 100,
				ini_rcomp0_lob * MAXIM_RCOMP0_LIM_LO,
				ini_rcomp0_lob * MAXIM_RCOMP0_LIM_HI);

	pr_debug("rcomp0=%x rcomp0_lob=%x->%x min=%x max=%x\n",
		 rcomp0, (rcomp0 & 0xff), rcomp0_lob,
		 (ini_rcomp0_lob * MAXIM_RCOMP0_LIM_LO) / 100,
		 (ini_rcomp0_lob * MAXIM_RCOMP0_LIM_HI) / 100);

	/* always write 0 to the high byte */
	*new_rcomp0 = rcomp0_lob & 0xff;

	tc_lob = comp_check(tc_lob, 100, ini_tc_lob * MAXIM_TEMPCO_LIM_LO,
			    ini_tc_lob * MAXIM_TEMPCO_LIM_HI);
	tc_hib = comp_check(tc_hib, 100, ini_tc_hib * MAXIM_TEMPCO_LIM_LO,
			    ini_tc_hib * MAXIM_TEMPCO_LIM_HI);

	pr_debug("tempco=%x tempco_lob=%x->%x min=%x max=%x, tempco_hib=%x->%x min=%x max=%x\n",
		 tempco, tempco & 0xff, tc_lob,
		 (ini_tc_lob * MAXIM_TEMPCO_LIM_LO) / 100,
		 (ini_tc_lob * MAXIM_TEMPCO_LIM_HI) / 100,
		 (tempco >> 8) & 0xff, tc_hib,
		 (ini_tc_hib * MAXIM_TEMPCO_LIM_LO) / 100,
		 (ini_tc_hib * MAXIM_TEMPCO_LIM_HI) / 100);

	*new_tempco = (tc_hib << 8) | (tc_lob);

	return ((rcomp0 & 0xff) != *new_rcomp0) || (tempco != *new_tempco);
}

static int max1720x_fixup_comp(int plugged, const struct max1720x_chip *chip)
{
	u16 new_rcomp0, new_tempco, data[2] = { 0 };
	int err, loops;

	if (chip->ini_rcomp0 == -1 || chip->ini_tempco == -1)
		return 0;

	err = regmap_raw_read(chip->regmap, MAX1720X_RCOMP0, data,
			      sizeof(data));
	if (err < 0)
		return -EIO;

	new_rcomp0 = data[0];
	new_tempco = data[1];

	err = max1720x_comp_check(&new_rcomp0, &new_tempco, chip);
	pr_debug("rcomp0=0x%x tempco=0x%x (%d)\n", data[0], data[1], err);
	if (err <= 0)
		return err;

	/* 3 loops suggested from vendor */
	for (loops = 0; loops < 3; loops++) {

		err = max1720x_update_compare(chip->regmap, MAX1720X_RCOMP0,
					      new_rcomp0, new_tempco);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	dev_info(chip->dev,"Fix rcomp0=0x%x->0x%x tempco:0x%x->0x%x, retries=%d, (%d)\n",
		 data[0], new_rcomp0, data[1], new_tempco, loops, err);

	return (loops== 3) ? -ETIMEDOUT : err;
}

static void max1720x_fixup_capacity(int plugged, struct max1720x_chip *chip)
{
	u16 data;
	int ret;

	/* capacity outliers: fix rcomp0, tempco */
	ret = max1720x_fixup_comp(plugged, chip);
	if (ret > 0) {
		chip->comp_update_count += 1;

		data = chip->comp_update_count;
		ret = REGMAP_WRITE(chip->regmap_nvram, MAX17201_COMP_UPDATE_CNT,
				   data);
		if (ret < 0)
			dev_err(chip->dev, "update comp stats (%d)\n", ret);
	}

	/* capacity outliers: fix capacity */
	ret = max1720x_fixup_dxacc(plugged, chip);
	if (ret > 0) {
		chip->dxacc_update_count += 1;

		data = chip->dxacc_update_count;
		ret = REGMAP_WRITE(chip->regmap_nvram,
				   MAX17201_DXACC_UPDATE_CNT, data);
		if (ret < 0)
			dev_err(chip->dev, "update cap stats (%d)\n", ret);
	}

}

/*
 * FILTERCFG changed to 45 sec * 2^(MAXIM_FILTERCFG_PLUGGED - 7) on charger plug
 * event. If temperature and VFSOC are below the limits, then reset to INI
 * value. Otherwise, reschedule the work to reset to INI in 100 seconds.
 */
#define MAXIM_FILTERCFG_PLUGGED		9
#define MAXIM_FILTERCFG_MASK		0xFFF0
#define MAXIM_FILTERCFG_VFSOC_LIM	25
#define MAXIM_FILTERCFG_TEMP_LIM	(5 * 10)
#define FILTCFG_DLY_MS			(100 * 1000)

static void max1720x_dyn_filtercfg(int plugged, struct max1720x_chip *chip)
{
	u16 data, new_filtercfg;
	int temp, err, vfsoc;

	if (chip->ini_filtercfg == -1)
		return;

	if (plugged) {
		cancel_delayed_work(&chip->filtercfg_work);
		new_filtercfg = (chip->ini_filtercfg & MAXIM_FILTERCFG_MASK) |
				MAXIM_FILTERCFG_PLUGGED;
	} else {
		err = max17x0x_reg_read(chip->regmap, MAX17X0X_TAG_temp, &data);
		if (err < 0)
			return;
		temp = reg_to_deci_deg_cel(data);
		vfsoc = max1720x_get_battery_vfsoc(chip);

		if ((vfsoc < MAXIM_FILTERCFG_VFSOC_LIM) &&
		    (temp < MAXIM_FILTERCFG_TEMP_LIM))
			new_filtercfg = chip->ini_filtercfg;
		else {
			dev_info(chip->dev, "vfsoc=%d, temp=%d\n", vfsoc, temp);
			schedule_delayed_work(&chip->filtercfg_work,
					      msecs_to_jiffies(FILTCFG_DLY_MS));
			return;
		}
	}

	dev_info(chip->dev, "new filtercfg=0x%x on plug=%d\n", new_filtercfg,
		 plugged);

	err = REGMAP_WRITE(chip->regmap, MAX1720X_FILTERCFG, new_filtercfg);
	if (err < 0)
		dev_err(chip->dev, "filtercfg failed update (%d)\n", err);
}

static void max1720x_filtercfg_work(struct work_struct *work)
{
	struct max1720x_chip *chip =
		container_of(work, struct max1720x_chip, filtercfg_work.work);
	int err;

	err = REGMAP_WRITE(chip->regmap, MAX1720X_FILTERCFG,
			   chip->ini_filtercfg);
	if (err < 0)
		dev_err(chip->dev, "filtercfg failed update (%d)\n", err);

}

static int max1720x_set_overheat_capacity(struct max1720x_chip *chip)
{
	bool plugged = chip->cap_estimate.cable_in;
	bool overheat = chip->health == POWER_SUPPLY_HEALTH_OVERHEAT;
	bool use_avsoc = chip->reg_prop_capacity_raw == MAX1720X_AVSOC;
	u16 repsoc;
	int err;

	if (plugged && overheat && !use_avsoc) {
		err = REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &repsoc);
		if (err < 0)
			return err;

		err = REGMAP_WRITE(chip->regmap, MAX1720X_AVSOC, repsoc);
		if (err < 0)
			return err;

		chip->reg_prop_capacity_raw = MAX1720X_AVSOC;
	}
	return 0;
}

static int max1720x_set_battery_health(struct max1720x_chip *chip, int health)
{
	int rc = 0;

	if (health < POWER_SUPPLY_HEALTH_UNKNOWN ||
		health > POWER_SUPPLY_HEALTH_HOT)
		return -EINVAL;

	chip->health = health;

	if (health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		rc = max1720x_set_overheat_capacity(chip);
		if (rc < 0)
			dev_warn(chip->dev,
				"set_overheat_capacity fail rc=%d\n", rc);
	}

	return 0;
}

static int max1720x_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)
					power_supply_get_drvdata(psy);
	struct gbatt_capacity_estimation *ce = &chip->cap_estimate;
	int rc = 0;
	int idata;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_BATT_CE_CTRL:
		max1720x_fixup_capacity(val->intval, chip);
		max1720x_dyn_filtercfg(val->intval, chip);

		mutex_lock(&ce->batt_ce_lock);
		if (val->intval) {

			if (!ce->cable_in) {
				rc = batt_ce_init(ce, chip);
				ce->cable_in = (rc == 0);
			}

		} else if (ce->cable_in) {
			if (ce->estimate_state == ESTIMATE_PENDING)
				cancel_delayed_work_sync(&ce->settle_timer);
			/* race with batt_ce_capacityfiltered_work() */
			batt_ce_stop_estimation(ce, ESTIMATE_NONE);
			batt_ce_dump_data(ce, chip->ce_log);
			ce->cable_in = false;
		}
		mutex_unlock(&ce->batt_ce_lock);
		break;
	case POWER_SUPPLY_PROP_RES_FILTER_COUNT:
		idata = val->intval;
		rc = batt_res_registers(chip, false, SEL_RES_FILTER_COUNT,
					&idata);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_AVG:
		idata = val->intval;
		rc = batt_res_registers(chip, false, SEL_RES_AVG, &idata);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		rc = max1720x_set_battery_health(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0)
		return rc;

	return 0;
}

static int max1720x_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_BATT_CE_CTRL:
	case POWER_SUPPLY_PROP_RES_FILTER_COUNT:
	case POWER_SUPPLY_PROP_RESISTANCE_AVG:
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * A fuel gauge reset resets only the fuel gauge operation without resetting IC
 * hardware. This is useful for testing different configurations without writing
 * nonvolatile memory.
 * TODO: add a lock around fg_reset to prevent SW from accessing the gauge until
 * the delay for volatile register access (rset->map[2]) expires. Need a lock
 * only if using this after _init()
 */
static void max17x0x_fg_reset(struct max1720x_chip *chip)
{
	const struct max17x0x_reg *rset;
	bool done = false;
	int err;

	rset = max17x0x_find_by_tag(MAX17X0X_TAG_rset);
	if (!rset)
		return;

	dev_info(chip->dev, "FG_RESET addr=%x value=%x delay=%d\n",
			    rset->map16[0], rset->map16[1], rset->map16[2]);

	err = REGMAP_WRITE(chip->regmap, rset->map16[0], rset->map16[1]);
	if (err < 0) {
		dev_err(chip->dev, "FG_RESET error writing Config2 (%d)\n",
				   err);
	} else {
		int loops = 10; /* 10 * MAX17X0X_TPOR_MS = 1.5 secs */
		u16 cfg2 = 0;

		for ( ; loops ; loops--) {
			msleep(MAX17X0X_TPOR_MS);

			err = REGMAP_READ(chip->regmap, rset->map16[0], &cfg2);
			done = (err == 0) && !(cfg2 & rset->map16[1]);
			if (done) {
				msleep(rset->map16[2]);
				break;
			}
		}

		if (!done)
			dev_err(chip->dev, "FG_RESET error rst not clearing\n");
		else
			dev_info(chip->dev, "FG_RESET cleared in %dms\n",
				loops * MAX17X0X_TPOR_MS + rset->map16[2]);

	}

}

/*
 * A full reset restores the ICs to their power-up state the same as if power
 * had been cycled.
 */
static int max1720x_full_reset(struct max1720x_chip *chip)
{
	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		/* a full (hw) reset on max1730x cause charge and discharge FET
		 * to toggle and the device will lose power. Will need to
		 * connect the device to a charger to get max1730x firmware to
		 * start and max1730x to close the FETs. Never send a HW reset
		 * to a 1730x while in system...
		 */
		dev_warn(chip->dev, "ignore full reset of fuel gauge\n");
		return 0;
	}

	REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
		     MAX1720X_COMMAND_HARDWARE_RESET);

	msleep(MAX17X0X_TPOR_MS);

	return 0;
}

#define LOG_BUFFER_SIZE   256
#define EACH_DUMP_SIZE   8
static void max1720x_dumpreg_work(struct work_struct *work)
{
	struct max1720x_chip *chip =
		container_of(work, struct max1720x_chip, dumpreg_work.work);
	int len = 0;
	u16 data;
	unsigned int reg;
	char buf[LOG_BUFFER_SIZE];

	for (reg = 0x00; reg <= 0xFF; reg++) {
		if (!max1720x_is_reg(chip->dev, reg))
			continue;
		if (!REGMAP_READ(chip->regmap, reg, &data)) {
			len += scnprintf(&buf[len], sizeof(buf) - len,
					     "%02X:%04X ", reg, data);
			/* 14 is the size of time stamp */
			if (len >= (LOG_BUFFER_SIZE - EACH_DUMP_SIZE - 14)) {
				logbuffer_log(chip->maxfg_log, "%s", buf);
				len = 0;
			}
		}
	}

	if (len > 0)  /* log the last line */
		logbuffer_log(chip->maxfg_log, "%s", buf);
}

#define IRQ_STORM_TRIGGER_SECONDS		60
#define IRQ_STORM_TRIGGER_MAX_COUNTS		50
#define IRQ_LOG_PERIOD		600
static irqreturn_t max1720x_fg_irq_thread_fn(int irq, void *obj)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)obj;
	u16 fg_status, fg_status_clr, fg_alarm = 0;
	int err = 0, now_time = 0, interval_time, irq_cnt;
	static int icnt;
	static int stime;
	static int dump_time;
	bool storm = false;
	bool is_log = false;

	now_time = div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC);
	if (now_time < IRQ_STORM_TRIGGER_SECONDS) {
		icnt = 0;
		stime = now_time;
	}

	icnt++;

	if (!chip || irq != chip->primary->irq) {
		WARN_ON_ONCE(1);
		return IRQ_NONE;
	}

	if (max17xxx_gauge_type == -1)
		return IRQ_NONE;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return IRQ_HANDLED;
	}

	pm_runtime_put_sync(chip->dev);

	err = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &fg_status);
	if (err)
		return IRQ_NONE;

	interval_time = now_time - stime;
	if (interval_time  > IRQ_STORM_TRIGGER_SECONDS) {
		irq_cnt = icnt * 100;
		irq_cnt /= (interval_time * 100 / IRQ_STORM_TRIGGER_SECONDS);
		storm = irq_cnt > IRQ_STORM_TRIGGER_MAX_COUNTS;
		if (storm) {
			/* IRQ storm */
			err = REGMAP_READ(chip->regmap, MAX1720X_ALARM,
					  &fg_alarm);
			dev_warn(chip->dev,
				"sts:%04x, alarm:%04x, cnt:%d err=%d\n",
				fg_status, fg_alarm, icnt, err);
		} else {
			icnt = 0;
			stime = now_time;
		}
	}

	if (fg_status == 0) {
		chip->debug_irq_none_cnt++;
		pr_debug("spurius: fg_status=0 cnt=%d\n",
			chip->debug_irq_none_cnt);
		/* rate limit spurius interrupts */
		msleep(MAX1720X_TICLR_MS);
		return IRQ_HANDLED;
	}

	/* only used to report health */
	chip->health_status |= fg_status;
	/* write 0 to clear will loose interrupts when we don't write 1 to the
	 * bits that are not set. Just inverting fg_status cause an interrupt
	 * storm, only setting the bits marked as "host must clear" in the DS
	 * seems to work eg:
	 *
	 * fg_status_clr = fg_status
	 * fg_status_clr |= MAX1720X_STATUS_POR | MAX1720X_STATUS_DSOCI
	 *                | MAX1720X_STATUS_BI;
	 *
	 * If the above logic is sound, we probably need to set also the bits
	 * that config mark as "host must clear". Maxim to confirm.
	 */
	fg_status_clr = fg_status;

	if (fg_status & MAX1720X_STATUS_POR) {
		fg_status_clr &= ~MAX1720X_STATUS_POR;
		pr_debug("POR is set\n");
	}
	if (fg_status & MAX1720X_STATUS_IMN)
		pr_debug("IMN is set\n");

	if (fg_status & MAX1720X_STATUS_BST)
		pr_debug("BST is set\n");

	if (fg_status & MAX1720X_STATUS_IMX)
		pr_debug("IMX is set\n");

	if (fg_status & MAX1720X_STATUS_DSOCI) {
		fg_status_clr &= ~MAX1720X_STATUS_DSOCI;
		pr_debug("DSOCI is set\n");
	}
	if (fg_status & MAX1720X_STATUS_VMN) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status_clr &= ~MAX1720X_STATUS_VMN;
		pr_debug("VMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMN) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status_clr &= ~MAX1720X_STATUS_TMN;
		pr_debug("TMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMN) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status_clr &= ~MAX1720X_STATUS_SMN;
		pr_debug("SMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_BI)
		pr_debug("BI is set\n");

	if (fg_status & MAX1720X_STATUS_VMX) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status_clr &= ~MAX1720X_STATUS_VMX;
		pr_debug("VMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMX) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status_clr &= ~MAX1720X_STATUS_TMX;
		pr_debug("TMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMX) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status_clr &= ~MAX1720X_STATUS_SMX;
		pr_debug("SMX is set\n");
	}

	if (fg_status & MAX1720X_STATUS_BR)
		pr_debug("BR is set\n");

	err = REGMAP_WRITE(chip->regmap, MAX1720X_STATUS, fg_status_clr);
	is_log = (now_time - dump_time > IRQ_LOG_PERIOD) || dump_time == 0;
	if (err == 0) {
		u16 temp = 0;

		err = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &temp);
		if (!is_log) {
			/* do nothing */
		} else if (err != 0) {
			logbuffer_log(chip->maxfg_log,
				      "status=%#x, fg_status_clr=%#x failed to read status register (%d)",
				      fg_status, fg_status_clr, err);
		} else if (temp & ~fg_status_clr) {
			logbuffer_log(chip->maxfg_log,
				      "status=%#x, fg_status_clr=%#x, temp=%#x failed to clear status register",
				      fg_status, fg_status_clr, temp);

			schedule_delayed_work(&chip->dumpreg_work, 0);
			dump_time = now_time;
		}
	} else if (is_log) {
		logbuffer_log(chip->maxfg_log,
			      "status:%#x failed to write %#x to status register (%d)",
			      fg_status, fg_status_clr, err);
	}

	if (storm && (fg_status & MAX1720X_STATUS_DSOCI)) {
		pr_debug("Force power_supply_change in storm\n");
		storm = false;
	}

	if (chip->psy && !storm)
		power_supply_changed(chip->psy);

	/* oneshot w/o filter will unmask on return but gauge will take up
	 * to 351 ms to clear ALRM1.
	 */
	msleep(MAX1720X_TICLR_MS);

	return IRQ_HANDLED;
}

static int max1720x_read_batt_id(const struct max1720x_chip *chip, int *batt_id)
{
	bool defer;
	int rc = 0, temp_id;
	struct device_node *node = chip->dev->of_node;
	const char *batt_psy_name = NULL;

	*batt_id = 0;

	rc = of_property_read_u32(node, "maxim,force-batt-id", &temp_id);
	if (rc == 0) {
		dev_warn(chip->dev, "forcing battery RID %d\n", temp_id);
		*batt_id = temp_id;
		return 0;
	}

	rc = of_property_read_string(node, "maxim,bat-power-supply",
				     &batt_psy_name);
	if (rc == 0) {
		struct power_supply *batt_psy;
		union power_supply_propval val;

		batt_psy = power_supply_get_by_name(batt_psy_name);
		if (!batt_psy) {
			dev_warn(chip->dev, "failed to get \"%s\" power supply\n",
				batt_psy_name);
			return -EPROBE_DEFER;
		}

		/* POWER_SUPPLY_PROP_RESISTANCE_ID is in ohms */
		rc = power_supply_get_property(batt_psy,
					       POWER_SUPPLY_PROP_RESISTANCE_ID,
					       &val);
		if (rc == 0)
			temp_id = val.intval;
	} else {
		rc = gbms_storage_read(GBMS_TAG_BRID, &temp_id,
				       sizeof(temp_id));
	}

	defer = (rc == -EPROBE_DEFER) ||
		(rc == -EINVAL) ||
		((rc == 0) && (temp_id == -EINVAL));
	if (defer)
		return -EPROBE_DEFER;
	if (rc < 0) {
		dev_err(chip->dev, "failed to get batt-id rc=%d\n", rc);
		return -EINVAL;
	}

	*batt_id = temp_id;
	return 0;
}

/* TODO: fix detection of 17301 for non samples looking at FW version too */
static int max1720x_read_gauge_type(struct max1720x_chip *chip)
{
	int ret, gauge_type = -1;

	ret = REGMAP_READ(chip->regmap, MAX1720X_DEVNAME, &chip->devname);
	if (ret != 0) {
		dev_err(chip->dev, "cannot read device name %d\n", ret);
	} else {
		switch (chip->devname >> 4) {
		case 0x406:  /* max1730x pass2 silicon */
		case 0x405:  /* max1730x pass2 silicon initial samples */
			gauge_type = MAX1730X_GAUGE_TYPE;
			break;
		case 0x404:  /* max1730x sample */
			chip->shadow_override = false;
			gauge_type = MAX1730X_GAUGE_TYPE;
			break;
		default: /* default to max1720x */
			gauge_type = MAX1720X_GAUGE_TYPE;
			break;
		}
	}

	return gauge_type;
}

static int max1720x_handle_dt_batt_id(struct max1720x_chip *chip)
{
	int ret, batt_id;
	u32 batt_id_range = 20, batt_id_kohm;
	struct device_node *node = chip->dev->of_node;
	struct device_node *config_node, *child_node;

	ret = max1720x_read_batt_id(chip, &chip->batt_id);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (ret < 0 || chip->batt_id < 0)
		/* Don't fail probe on that, just ignore error */
		return 0;

	dev_info(chip->dev, "device battery RID: %d kohm\n", chip->batt_id);

	ret = of_property_read_u32(node, "maxim,batt-id-range-pct",
				   &batt_id_range);
	if (ret && ret == -EINVAL)
		dev_warn(chip->dev, "failed to read maxim,batt-id-range-pct\n");

	config_node = of_find_node_by_name(node, "maxim,config");
	if (!config_node) {
		dev_warn(chip->dev, "Failed to find maxim,config setting\n");
		return 0;
	}

	batt_id = chip->batt_id / 1000;
	for_each_child_of_node(config_node, child_node) {
		ret = of_property_read_u32(child_node, "maxim,batt-id-kohm",
					   &batt_id_kohm);
		if (!ret &&
		    (batt_id < (batt_id_kohm * (100 + batt_id_range) / 100)) &&
		    (batt_id > (batt_id_kohm * (100 - batt_id_range) / 100))) {
			chip->batt_node = child_node;
			break;
		}
	}
	if (!chip->batt_node)
		dev_warn(chip->dev, "No child node found matching ID\n");

	return 0;
}

static int max17x0x_apply_regval_shadow(struct max1720x_chip *chip,
					struct device_node *node,
					struct max17x0x_cache_data *nRAM,
					int nb)
{
	u16 *regs;
	int ret, i;
	const char *propname = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) ?
		 "maxim,n_regval_1730x" : "maxim,n_regval_1720x";

	if (!node || nb <= 0)
		return 0;

	if (nb & 1) {
		dev_warn(chip->dev, "%s %s u16 elems count is not even: %d\n",
			 node->name, propname, nb);
		return -EINVAL;
	}

	regs = (u16 *)kmalloc_array(nb, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, propname, regs, nb);
	if (ret) {
		dev_warn(chip->dev, "failed to read %s: %d\n", propname, ret);
		goto shadow_out;
	}

	for (i = 0; i < nb; i += 2) {
		const int idx = max17x0x_cache_index_of(nRAM, regs[i]);
		nRAM->cache_data[idx] = regs[i + 1];
	}

shadow_out:
	kfree(regs);
	return ret;
}

/* support for initial batch of ill configured max1720x packs */
static void max1720x_consistency_check(struct max17x0x_cache_data *cache)
{
	int nvcfg_idx = max17x0x_cache_index_of(cache, MAX1720X_NNVCFG0);
	int ncgain_idx = max17x0x_cache_index_of(cache, MAX1720X_NCGAIN);
	u16 *nRAM_updated = cache->cache_data;

	if ((nRAM_updated[nvcfg_idx] & MAX1720X_NNVCFG0_ENCG) &&
		((nRAM_updated[ncgain_idx] == 0) ||
		(nRAM_updated[ncgain_idx] == 0x0400)))
		nRAM_updated[ncgain_idx] = 0x4000;
}

static int max17x0x_read_dt_version(struct device_node *node, u8 *reg, u8 *val)
{
	int ret;
	const char *propname;
	u8 version[2];

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		propname = "maxim,n_regval_1730x_ver";
	} else {
		propname = "maxim,n_regval_1720x_ver";
	}

	ret = of_property_read_u8_array(node, propname,
					version,
					sizeof(version));
	if (ret < 0)
		return -ENODATA;

	*reg = version[0];
	*val = version[1];

	return 0;
}

static int max17x0x_read_dt_version_por(struct device_node *node,
					u8 *reg, u8 *val)
{
	int ret;
	const char *propname;
	u8 version[2];

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		propname = "maxim,n_regval_1730x_ver_por";
	} else {
		propname = "maxim,n_regval_1720x_ver_por";
	}

	ret = of_property_read_u8_array(node, propname,
					version,
					sizeof(version));
	if (ret < 0)
		return -ENODATA;

	*reg = version[0];
	*val = version[1];

	return 0;
}

static int max17x0x_handle_dt_shadow_config(struct max1720x_chip *chip)
{
	int ret, glob_cnt;
	const char *propname = NULL;
	struct max17x0x_cache_data nRAM_c;
	struct max17x0x_cache_data nRAM_u;
	int ver_idx = -1;
	u8 vreg, vval;

	ret = max1720x_handle_dt_batt_id(chip);
	if (ret)
		return ret;

	/* for devices that don't support max1720x_fg_reset() */
	if (!chip->shadow_override) {
		dev_info(chip->dev, "ignore shadow override\n");
		return 0;
	}

	if (max17xxx_gauge_type == -1)
		return -EINVAL;
	else if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		propname = "maxim,n_regval_1730x";
	else
		propname = "maxim,n_regval_1720x";

	ret = max17x0x_nvram_cache_init(&nRAM_c, max17xxx_gauge_type);
	if (ret < 0)
		return ret;

	ret = max17x0x_cache_load(&nRAM_c, chip->regmap_nvram);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read config from shadow RAM\n");
		goto error_out;
	}

	ret = max17x0x_cache_dup(&nRAM_u, &nRAM_c);
	if (ret < 0)
		goto error_out;


	/* apply overrides */
	if (chip->batt_node) {
		int batt_cnt;

		batt_cnt = of_property_count_elems_of_size(chip->batt_node,
							   propname,
							   sizeof(u16));
		max17x0x_apply_regval_shadow(chip, chip->batt_node,
					     &nRAM_u,
					     batt_cnt);
	}

	glob_cnt = of_property_count_elems_of_size(chip->dev->of_node,
						   propname,
						   sizeof(u16));
	max17x0x_apply_regval_shadow(chip, chip->dev->of_node,
				     &nRAM_u,
				     glob_cnt);

	if (max17xxx_gauge_type == MAX1720X_GAUGE_TYPE)
		max1720x_consistency_check(&nRAM_u);

	if (max17x0x_read_dt_version(chip->dev->of_node, &vreg, &vval) == 0) {
		/* Versioning enforced: reset the gauge (and overwrite
		 * version) only if the version in device tree is
		 * greater than the version in the gauge.
		 */
		ver_idx = max17x0x_cache_index_of(&nRAM_u, vreg);
		if (ver_idx < 0) {
			dev_err(chip->dev, "version register %x is not mapped\n",
					vreg);
		} else if ((nRAM_u.cache_data[ver_idx] & 0xff) < vval) {
			/* force version in dt, will write (and reset fg)
			 * only when less than the version in nRAM_c
			 */
			dev_info(chip->dev,
				"DT version updated %d -> %d\n",
				nRAM_u.cache_data[ver_idx] & 0xff,
				vval);

			nRAM_u.cache_data[ver_idx] &= 0xff00;
			nRAM_u.cache_data[ver_idx] |= vval;
			chip->needs_reset = true;
		}
	}

	if (max17x0x_cache_memcmp(&nRAM_c, &nRAM_u)) {
		bool fg_reset = false;

		if (ver_idx < 0) {
			/* Versioning not enforced: nConvgCfg take effect
			 * without resetting the gauge
			 */
			const int idx = max17x0x_cache_index_of(&nRAM_u,
							MAX1720X_NCONVGCFG);
			nRAM_c.cache_data[idx] = nRAM_u.cache_data[idx];
			fg_reset = max17x0x_cache_memcmp(&nRAM_u, &nRAM_c) != 0;
		}

		ret = max17x0x_cache_store(&nRAM_u, chip->regmap_nvram);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to write config from shadow RAM\n");
			goto error_out;
		}

		/* different reason for reset */
		if (fg_reset) {
			chip->needs_reset = true;
			dev_info(chip->dev,
				"DT config differs from shadow, resetting\n");
		}
	}

error_out:
	max17x0x_cache_free(&nRAM_c);
	max17x0x_cache_free(&nRAM_u);

	return ret;
}

static int max17x0x_apply_regval_register(struct max1720x_chip *chip,
					struct device_node *node)
{
	int cnt, ret = 0, idx, err;
	u16 *regs, data;
	const char *propname;

	propname = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) ?
		 "maxim,r_regval_1730x" : "maxim,r_regval_1720x";

	cnt =  of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (!node || cnt <= 0)
		return 0;

	if (cnt & 1) {
		dev_warn(chip->dev, "%s %s u16 elems count is not even: %d\n",
			 node->name, propname, cnt);
		return -EINVAL;
	}

	regs = (u16 *)kmalloc_array(cnt, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, propname, regs, cnt);
	if (ret) {
		dev_warn(chip->dev, "failed to read %s %s: %d\n",
			 node->name, propname, ret);
		goto register_out;
	}

	for (idx = 0; idx < cnt; idx += 2) {
		if (max1720x_is_reg(chip->dev, regs[idx])) {
			err = REGMAP_READ(chip->regmap, regs[idx], &data);
			if (!err && data != regs[idx + 1])
				REGMAP_WRITE(chip->regmap, regs[idx],
					     regs[idx + 1]);
		}
	}
register_out:
	kfree(regs);
	return ret;
}

static int max17x0x_handle_dt_register_config(struct max1720x_chip *chip)
{
	int ret = 0;

	if (chip->batt_node)
		ret = max17x0x_apply_regval_register(chip, chip->batt_node);

	if (ret)
		return ret;

	ret = max17x0x_apply_regval_register(chip, chip->dev->of_node);

	return ret;
}

static int max1720x_handle_dt_nconvgcfg(struct max1720x_chip *chip)
{
	int ret = 0, i;
	struct device_node *node = chip->dev->of_node;

	chip->curr_convgcfg_idx = -1;
	mutex_init(&chip->convgcfg_lock);

	ret = of_property_read_u32(node, "google,cap-tsettle",
				   (u32 *)&chip->cap_estimate.cap_tsettle);
	if (ret < 0)
		chip->cap_estimate.cap_tsettle = DEFAULT_CAP_SETTLE_INTERVAL;
	chip->cap_estimate.cap_tsettle =
				chip->cap_estimate.cap_tsettle * 60 * 1000;

	ret = of_property_read_u32(node, "google,cap-filt-length",
				   (u32 *)&chip->cap_estimate.cap_filt_length);
	if (ret < 0)
		chip->cap_estimate.cap_filt_length = DEFAULT_CAP_FILTER_LENGTH;

	chip->nb_convgcfg =
	    of_property_count_elems_of_size(node, "maxim,nconvgcfg-temp-limits",
					    sizeof(s16));
	if (!chip->nb_convgcfg)
		return 0;

	ret = of_property_read_s32(node, "maxim,nconvgcfg-temp-hysteresis",
				   &chip->convgcfg_hysteresis);
	if (ret < 0)
		chip->convgcfg_hysteresis = 10;
	else if (chip->convgcfg_hysteresis < 0)
			chip->convgcfg_hysteresis = 10;
	if (ret == 0)
		dev_info(chip->dev, "%s maxim,nconvgcfg-temp-hysteresis = %d\n",
			 node->name, chip->convgcfg_hysteresis);

	if (chip->nb_convgcfg != of_property_count_elems_of_size(node,
						  "maxim,nconvgcfg-values",
						  sizeof(u16))) {
		dev_warn(chip->dev, "%s maxim,nconvgcfg-values and maxim,nconvgcfg-temp-limits are missmatching number of elements\n",
			 node->name);
		return -EINVAL;
	}
	chip->temp_convgcfg = (s16 *)devm_kmalloc_array(chip->dev,
							chip->nb_convgcfg,
							sizeof(s16),
								GFP_KERNEL);
	if (!chip->temp_convgcfg)
		return -ENOMEM;

	chip->convgcfg_values = (u16 *)devm_kmalloc_array(chip->dev,
							  chip->nb_convgcfg,
							  sizeof(u16),
							  GFP_KERNEL);
	if (!chip->convgcfg_values) {
		devm_kfree(chip->dev, chip->temp_convgcfg);
		chip->temp_convgcfg = NULL;
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(node, "maxim,nconvgcfg-temp-limits",
					 (u16 *) chip->temp_convgcfg,
					 chip->nb_convgcfg);
	if (ret) {
		dev_warn(chip->dev, "failed to read maxim,nconvgcfg-temp-limits: %d\n",
			 ret);
		goto error;
	}

	ret = of_property_read_u16_array(node, "maxim,nconvgcfg-values",
					 chip->convgcfg_values,
					 chip->nb_convgcfg);
	if (ret) {
		dev_warn(chip->dev, "failed to read maxim,nconvgcfg-values: %d\n",
			 ret);
		goto error;
	}
	for (i = 1; i < chip->nb_convgcfg; i++) {
		if (chip->temp_convgcfg[i] < chip->temp_convgcfg[i-1]) {
			dev_warn(chip->dev, "nconvgcfg-temp-limits idx:%d < idx:%d\n",
				 i, i-1);
			goto error;
		}
		if ((chip->temp_convgcfg[i] - chip->temp_convgcfg[i-1])
		    <= chip->convgcfg_hysteresis) {
			dev_warn(chip->dev, "nconvgcfg-temp-hysteresis smaller than idx:%d, idx:%d\n",
				 i, i-1);
			goto error;
		}
	}

	chip->nb_empty_voltage = of_property_count_elems_of_size(node,
								 "maxim,empty-voltage",
								 sizeof(u16));
	if (chip->nb_empty_voltage > 0 &&
	    chip->nb_empty_voltage % NB_CYCLE_BUCKETS == 0) {
		chip->empty_voltage = (u16 *)devm_kmalloc_array(chip->dev,
							chip->nb_empty_voltage,
							sizeof(u16),
							GFP_KERNEL);
		if (!chip->empty_voltage)
			goto error;

		ret = of_property_read_u16_array(node, "maxim,empty-voltage",
						chip->empty_voltage,
						chip->nb_empty_voltage);
		if (ret) {
			dev_warn(chip->dev,
				 "failed to read maxim,empty-voltage: %d\n",
				 ret);
		}
	} else
		dev_warn(chip->dev,
			 "maxim,empty-voltage is missmatching the number of elements, nb = %d\n",
			 chip->nb_empty_voltage);
error:
	if (ret) {
		devm_kfree(chip->dev, chip->temp_convgcfg);
		devm_kfree(chip->dev, chip->convgcfg_values);
		chip->temp_convgcfg = NULL;
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static int get_irq_none_cnt(void *data, u64 *val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;

	*val = chip->debug_irq_none_cnt;
	return 0;
}

static int set_irq_none_cnt(void *data, u64 val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;

	if (val == 0)
		chip->debug_irq_none_cnt = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(irq_none_cnt_fops, get_irq_none_cnt,
	set_irq_none_cnt, "%llu\n");


static int debug_fg_reset(void *data, u64 val)
{
	max17x0x_fg_reset((struct max1720x_chip *)data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fg_reset_fops, NULL, debug_fg_reset, "%llu\n");

static int debug_ce_start(void *data, u64 val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;
	batt_ce_start(&chip->cap_estimate, val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ce_start_fops, NULL, debug_ce_start, "%llu\n");


#define BATTERY_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

/* dump with "cat /d/max1720x/nvram_por | xxd"
 * NOTE: for testing add a setter that initialize chip->nRAM_por (if not
 * initialized) and use _load() to read NVRAM.
 */
static ssize_t debug_get_nvram_por(struct file *filp,
				   char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)filp->private_data;
	int size;

	if (!chip || !chip->nRAM_por.cache_data)
		return -ENODATA;

	size = chip->nRAM_por.atom.size > count ?
			count : chip->nRAM_por.atom.size;

	return simple_read_from_buffer(buf, count, ppos,
		chip->nRAM_por.cache_data,
		size);
}

BATTERY_DEBUG_ATTRIBUTE(debug_nvram_por_fops, debug_get_nvram_por, NULL);



static int debug_fake_battery_set(void *data, u64 val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;

	chip->fake_battery = (int)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fake_battery_fops, NULL,
			debug_fake_battery_set, "%llu\n");


#endif

static int init_debugfs(struct max1720x_chip *chip)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *de;

	de = debugfs_create_dir("max1720x", 0);
	if (de) {
		debugfs_create_file("irq_none_cnt", 0644, de,
				    chip, &irq_none_cnt_fops);
		debugfs_create_file("nvram_por", 0440, de,
				    chip, &debug_nvram_por_fops);
		debugfs_create_file("fg_reset", 0400, de,
				    chip, &debug_fg_reset_fops);
		debugfs_create_file("ce_start", 0400, de,
				    chip, &debug_ce_start_fops);
		debugfs_create_file("fake_battery", 0400, de,
				    chip, &debug_fake_battery_fops);
	}
#endif
	return 0;
}

static u16 max1720x_read_rsense(const struct max1720x_chip *chip)
{
	u16 rsense = 0;
	u32 rsense_default = 0;
	int ret;

	(void)of_property_read_u32(chip->dev->of_node, "maxim,rsense-default",
		&rsense_default);

	ret = REGMAP_READ(chip->regmap_nvram, MAX1720X_NRSENSE, &rsense);
	if (rsense_default && (ret < 0 || rsense != rsense_default)) {
		rsense = rsense_default;
		dev_warn(chip->dev, "RSense, forcing %d micro Ohm (%d)\n",
			rsense * 10, ret);
	}

	return rsense;
}

/* The BCNT, QHQH and QHCA for 17202 modify these common registers
 * 0x18C, 0x18D, 0x18E, 0x18F, 0x1B2, 0x1B4 which will default to 0 after
 * the recovery procedure. The map changes the content of 0x1C4, 0x1C5 which
 * are not used (and should be 0 unless used by a future version of SW) as well
 * as 0x1CD, 0x1CE used for nManfctrName1/2 which might change. 0x1DF is the
 * INI checksum that will change with updates and cannot be used for this test.
 * The only register left is 0x1D7 (MAX1730X_NPROTCFG), that is the register
 * that the code will used to determine the corruption.
 */
static int max1730x_check_prot(struct max1720x_chip *chip, u16 devname)
{
	int needs_recall = 0;
	u16 nprotcfg;
	int ret;

	/* check protection registers */
	ret = REGMAP_READ(chip->regmap_nvram, MAX1730X_NPROTCFG, &nprotcfg);
	if (ret < 0)
		return -EIO;

	if (devname == MAX1730X_GAUGE_PASS1) {
		needs_recall = (nprotcfg != MAX1730X_NPROTCFG_PASS1);
	} else /* pass2 */ {
		needs_recall = (nprotcfg != MAX1730X_NPROTCFG_PASS2);
	}

	return needs_recall;
}

static int max17x0x_nvram_recall(struct max1720x_chip *chip)
{
	REGMAP_WRITE(chip->regmap,
			MAX17XXX_COMMAND,
			MAX17XXX_COMMAND_NV_RECALL);
	msleep(MAX17X0X_TPOR_MS);
	return 0;
}

static int max17x0x_fixups(struct max1720x_chip *chip)
{
	int ret = 0;

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		struct device_node *node = chip->dev->of_node;
		bool write_back = false, write_ndata = false;
		const u16 val = 0x280B;
		u16 ndata = 0, bak = 0;

		/* b/123026365 */
		if (of_property_read_bool(node, "maxim,enable-nv-check")) {
			const u16 devname = (chip->devname >> 4);
			int needs_recall;

			needs_recall = max1730x_check_prot(chip, devname);
			if (needs_recall < 0) {
				dev_err(chip->dev,
					"error reading fg NV configuration\n");
			} else if (needs_recall == 0) {
				/* all good.. */
			} else if (devname == MAX1730X_GAUGE_PASS1) {
				dev_err(chip->dev, "***********************************************\n");
				dev_err(chip->dev, "WARNING: need to restore FG NV configuration to\n");
				dev_err(chip->dev, "default values. THE DEVICE WILL LOOSE POWER.\n");
				dev_err(chip->dev, "*******************************************\n");
				msleep(MSEC_PER_SEC);
				REGMAP_WRITE(chip->regmap, MAX17XXX_COMMAND,
					MAX1720X_COMMAND_HARDWARE_RESET);
				msleep(MAX17X0X_TPOR_MS);
			} else {
				dev_err(chip->dev, "Restoring FG NV configuration to sane values\n");

				if (!chip->needs_reset) {
					ret = max17x0x_nvram_recall(chip);
					if (ret == 0)
						chip->needs_reset = true;
				}

				REGMAP_WRITE(chip->regmap_nvram,
						MAX1730X_NPROTCFG,
						MAX1730X_NPROTCFG_PASS2);
			}
		} else {
			dev_err(chip->dev, "nv-check disabled\n");
		}

		/* b/122605202 */
		ret = REGMAP_READ(chip->regmap_nvram,
						MAX1730X_NVPRTTH1, &ndata);
		if (ret == 0)
			ret = REGMAP_READ(chip->regmap,
						MAX1730X_NVPRTTH1BAK, &bak);
		if (ret < 0)
			return -EIO;

		if (ndata == MAX1730X_NVPRTTH1_CHARGING)
			write_back = (bak != val);
		else
			write_ndata = (ndata != val);

		if (write_back || write_ndata) {
			ret = REGMAP_WRITE(chip->regmap,
						MAX1730X_NVPRTTH1BAK, val);
			if (ret == 0)
				ret = REGMAP_WRITE(chip->regmap_nvram,
						MAX1730X_NVPRTTH1, val);
			if (ret == 0)
				ret = REGMAP_WRITE(chip->regmap,
						MAX1730X_NVPRTTH1BAK, val);
		}

		if (ret < 0) {
			dev_info(chip->dev,
				"failed to update 0x0D6=%x 0x1D0=%x to %x (%d)\n",
				bak, ndata, val, ret);
		} else if (write_back) {
			dev_info(chip->dev,
				"0x0D6=%x 0x1D0=%x updated to %x (%d)\n",
				bak, ndata, val, ret);
		} else if (write_ndata) {
			dev_info(chip->dev,
				"0x1D0=%x updated to %x (%d)\n",
				ndata, val, ret);
		}
	}

	return ret;
}

static int read_chip_property_u32(const struct max1720x_chip *chip,
				  char *property, u32 *data32)
{
	int ret;

	if (chip->batt_node) {
		ret = of_property_read_u32(chip->batt_node, property, data32);
		if (ret == 0)
			return ret;
	}

	return of_property_read_u32(chip->dev->of_node, property, data32);
}

/* capacity outliers */
static int max17201_init_fix_capacity(struct max1720x_chip *chip)
{
	u16 data = 0;
	u32 data32;
	int ret;

	ret = REGMAP_READ(chip->regmap_nvram, MAX17201_COMP_UPDATE_CNT, &data);
	if (ret < 0)
		return -EPROBE_DEFER;
	if (ret == 0)
		chip->comp_update_count = data;

	ret = REGMAP_READ(chip->regmap_nvram, MAX17201_DXACC_UPDATE_CNT,
			  &data);
	if (ret < 0)
		return -EPROBE_DEFER;
	if (ret == 0)
		chip->dxacc_update_count = data;

	/* device dependent values */
	ret = of_property_read_u32(chip->dev->of_node, "maxim,capacity-design",
				   &data32);
	if (ret < 0) {
		chip->design_capacity = -1;
	} else if (data32 == 0) {
		ret = REGMAP_READ(chip->regmap_nvram, MAX1720X_NDESIGNCAP,
				  &chip->design_capacity);
		if (ret < 0)
			return -EPROBE_DEFER;
	} else {
		chip->design_capacity = data32;
	}

	/* chemistry dependent codes:
	 * NOTE: ->batt_node is initialized in max1720x_handle_dt_shadow_config
	 */
	ret = read_chip_property_u32(chip, "maxim,capacity-rcomp0", &data32);
	if (ret < 0)
		chip->ini_rcomp0 = -1;
	else
		chip->ini_rcomp0 = data32;

	ret = read_chip_property_u32(chip, "maxim,capacity-tempco", &data32);
	if (ret < 0)
		chip->ini_tempco = -1;
	else
		chip->ini_tempco = data32;

	ret = of_property_read_u32(chip->dev->of_node, "maxim,capacity-stable",
				   &data32);
	if (ret < 0)
		chip->cycle_stable = BATTERY_DEFAULT_CYCLE_STABLE;
	else
		chip->cycle_stable = data32;

	ret = of_property_read_u32(chip->dev->of_node, "maxim,capacity-fade",
				   &data32);
	if (ret < 0)
		chip->cycle_fade = BATTERY_DEFAULT_CYCLE_FADE;
	else
		chip->cycle_fade = data32;

	ret = of_property_read_u32(chip->dev->of_node, "maxim,capacity-band",
				   &data32);
	if (ret < 0) {
		chip->cycle_band = BATTERY_DEFAULT_CYCLE_BAND;
	} else {
		chip->cycle_band = data32;
		if (chip->cycle_band > BATTERY_MAX_CYCLE_BAND)
			chip->cycle_band = BATTERY_MAX_CYCLE_BAND;
	}

	dev_info(chip->dev, "cnts=%d,%d dc=%d cap_sta=%d cap_fad=%d rcomp0=0x%x tempco=0x%x\n",
		chip->comp_update_count, chip->dxacc_update_count,
		chip->design_capacity, chip->cycle_stable, chip->cycle_fade,
		chip->ini_rcomp0, chip->ini_tempco);

	ret = read_chip_property_u32(chip, "maxim,capacity-filtercfg", &data32);
	if (ret < 0)
		chip->ini_filtercfg = -1;
	else
		chip->ini_filtercfg = data32;

	dev_info(chip->dev, "ini_filtercfg=0x%x\n", chip->ini_filtercfg);

	return 0;
}

static int max1720x_init_chip(struct max1720x_chip *chip)
{
	int ret;
	u8 vreg, vpor;
	u16 data = 0, tmp;
	bool force_recall = false;

	if (of_property_read_bool(chip->dev->of_node, "maxim,force-hard-reset"))
		max1720x_full_reset(chip);

	ret = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &data);
	if (ret < 0)
		return -EPROBE_DEFER;
	force_recall = (data & MAX1720X_STATUS_POR) != 0;
	if (force_recall)
		dev_err(chip->dev, "Recall: POR\n");

	/* TODO: disable with maxim,fix-ignore-zero-rsense */
	chip->RSense = max1720x_read_rsense(chip);
	if (chip->RSense == 0) {
		dev_err(chip->dev, "Recall: RSense value 0 micro Ohm\n");
		force_recall |= true;
	}

	/* read por version from dt (new function) */
	ret = max17x0x_read_dt_version_por(chip->dev->of_node, &vreg, &vpor);
	if (ret == 0) {
		ret = REGMAP_READ(chip->regmap_nvram, vreg, &tmp);
		if ((ret == 0) && (vpor == (tmp & 0x00ff))) {
			dev_err(chip->dev, "Recall: POR version %d\n", vpor);
			force_recall = true;
		}
	}

	/* b/129384855 fix mismatch between pack INI file and overrides */
	if (of_property_read_bool(chip->dev->of_node, "maxim,fix-vempty")) {
		ret = REGMAP_READ(chip->regmap, MAX1720X_VEMPTY, &data);
		if ((ret == 0) && (reg_to_vrecovery(data) == 0)) {
			dev_err(chip->dev, "Recall: zero vrecovery\n");
			force_recall = true;
		}
	}

	if (force_recall) {
		/* debug only */
		ret = max17x0x_nvram_cache_init(&chip->nRAM_por,
							max17xxx_gauge_type);
		if (ret == 0)
			ret = max17x0x_cache_load(&chip->nRAM_por,
							chip->regmap_nvram);
		if (ret < 0) {
			dev_err(chip->dev, "POR: Failed to backup config\n");
			return -EPROBE_DEFER;
		}

		dev_info(chip->dev, "Recall Battery NVRAM\n");
		ret = max17x0x_nvram_recall(chip);
		if (ret == 0)
			chip->needs_reset = true;
	}

	ret = max17x0x_fixups(chip);
	if (ret < 0)
		return ret;

	ret = max17x0x_handle_dt_shadow_config(chip);
	if (ret == -EPROBE_DEFER)
		return ret;

	ret = max17x0x_handle_dt_register_config(chip);
	if (ret == -EPROBE_DEFER)
		return ret;

	(void) max1720x_handle_dt_nconvgcfg(chip);

	/* recall, force & reset SW */
	if (chip->needs_reset) {
		max17x0x_fg_reset(chip);

		if (chip->RSense == 0)
			chip->RSense = max1720x_read_rsense(chip);
	}

	ret = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &data);
	if (!ret && data & MAX1720X_STATUS_BR) {
		dev_info(chip->dev, "Clearing Battery Removal bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_BR, 0x0);
	}
	if (!ret && data & MAX1720X_STATUS_BI) {
		dev_info(chip->dev, "Clearing Battery Insertion bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_BI, 0x0);
	}
	if (!ret && data & MAX1720X_STATUS_POR) {
		dev_info(chip->dev, "Clearing Power-On Reset bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_POR, 0x0);
	}

	dev_info(chip->dev, "RSense value %d micro Ohm\n", chip->RSense * 10);

	ret = max17x0x_reg_read(chip->regmap, MAX17X0X_TAG_cnfg,
				&chip->RConfig);
	if (ret)
		return -EPROBE_DEFER;

	dev_info(chip->dev, "Config: 0x%04x\n", chip->RConfig);

	ret = REGMAP_READ(chip->regmap, MAX1720X_ICHGTERM, &data);
	if (ret)
		return -EPROBE_DEFER;

	dev_info(chip->dev, "IChgTerm: %d\n",
		 reg_to_micro_amp(data, chip->RSense));

	ret = REGMAP_READ(chip->regmap, MAX1720X_VEMPTY, &data);
	if (ret)
		return -EPROBE_DEFER;

	dev_info(chip->dev, "VEmpty: VE=%dmV VR=%dmV\n",
		 reg_to_vempty(data), reg_to_vrecovery(data));

	/* Fix capacity drift b/134500876 */
	ret = max17201_init_fix_capacity(chip);
	if (ret < 0)
		dev_err(chip->dev, "Failed to initialize capacity fix (%d)\n",
			ret);

	/* Capacity data is stored as complement so it will not be zero. Using
	 * zero case to detect new un-primed pack
	 */
	ret = REGMAP_READ(chip->regmap_nvram, MAX17XXX_QHCA, &data);
	if (!ret && data == 0)
		max1720x_prime_battery_qh_capacity(chip,
						   POWER_SUPPLY_STATUS_UNKNOWN);
	else
		max1720x_restore_battery_qh_capacity(chip);

	return 0;
}

static int max1730x_decode_sn(char *serial_number,
			      unsigned int max,
			      const u16 *data)
{
	int tmp, count = 0;

	if (data[0] != 0x4257)	/* "BW": DSY */
		return -EINVAL;

	count += scnprintf(serial_number + count, max - count, "%02X%02X%02X",
			   data[3] & 0xFF,
			   data[4] & 0xFF,
			   data[5] & 0xFF);

	tmp = (((((data[1] >> 9) & 0x3f) + 1980) * 10000) +
		((data[1] >> 5) & 0xf) * 100 + (data[1] & 0x1F));
	count += scnprintf(serial_number + count, max - count, "%d",
			   tmp);

	count += scnprintf(serial_number + count, max - count, "%c%c",
			   data[0] >> 8, data[0] & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c%c%c%c",
			   data[7] >> 8,
			   data[8] >> 8,
			   data[9] >> 8,
			   data[9] & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c",
			   data[2] & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c%c",
			   data[6] >> 8, data[6] & 0xFF);
	return 0;
}

static int max1720x_decode_sn(char *serial_number,
			      unsigned int max,
			      const u16 *data)
{
	int tmp, count = 0, shift;
	char cell_vendor;

	if (data[0] == 0x5357) /* "SW": SWD */
		shift = 0;
	else if (data[0] == 0x4257) /* "BW": DSY */
		shift = 8;
	else
		return -EINVAL;

	count += scnprintf(serial_number + count, max - count, "%02X%02X%02X",
			   data[1] >> shift,
			   data[2] >> shift,
			   data[3] >> shift);

	tmp = (((((data[4] >> 9) & 0x3f) + 1980) * 10000) +
		((data[4] >> 5) & 0xf) * 100 + (data[4] & 0x1F));
	count += scnprintf(serial_number + count, max - count, "%d",
			   tmp);

	count += scnprintf(serial_number + count, max - count, "%c%c",
			   data[0] >> 8,
			   data[0] & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c%c%c",
			   data[5] >> shift,
			   data[6] >> shift,
			   data[7] >> shift);

	tmp = data[8];
	if (tmp >> 8 == 0)
		tmp = ('?' << 8) | (tmp & 0xFF);
	if ((tmp & 0xFF) == 0)
		tmp = (tmp & 0xFF00) | '?';
	count += scnprintf(serial_number + count, max - count, "%c%c",
			   tmp >> 8,
			   tmp & 0xFF);

	cell_vendor = (shift == 8) ? (data[9] >> 8) : (data[9] & 0xFF);
	count += scnprintf(serial_number + count, max - count, "%c",
			   cell_vendor);

	if (shift == 8) {
		count += scnprintf(serial_number + count, max - count, "%02X",
				   data[10] >> 8);
	} else {
		count += scnprintf(serial_number + count, max - count, "%c%c",
				   data[10] >> 8, data[10] & 0xFF);
	}

	return 0;
}

static struct power_supply_desc max1720x_psy_desc = {
	.name = "maxfg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = max1720x_get_property,
	.set_property = max1720x_set_property,
	.property_is_writeable = max1720x_property_is_writeable,
	.properties = max1720x_battery_props,
	.num_properties = ARRAY_SIZE(max1720x_battery_props),
};


static void *ct_seq_start(struct seq_file *s, loff_t *pos)
{
	struct max1720x_history *hi =
		(struct max1720x_history *)s->private;
	loff_t end_pos;

	end_pos = hi->history_count;
	if (hi->comp_update_count != -1 || hi->dxacc_update_count != -1)
		end_pos += 1;

	if (*pos >= end_pos)
		return NULL;
	hi->history_index = *pos;

	return &hi->history_index;
}

static void *ct_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	loff_t *spos = (loff_t *)v;
	struct max1720x_history *hi =
		(struct max1720x_history *)s->private;
	loff_t end_pos;

	end_pos = hi->history_count;
	if (hi->comp_update_count != -1 || hi->dxacc_update_count != -1)
		end_pos += 1;

	*pos = ++*spos;
	if (*pos >= end_pos)
		return NULL;

	return spos;
}

static void ct_seq_stop(struct seq_file *s, void *v)
{
	/* iterator in hi, no need to free */
}

static int ct_seq_show(struct seq_file *s, void *v)
{
	loff_t *spos = (loff_t *)v;
	struct max1720x_history *hi = (struct max1720x_history *)s->private;

	if (*spos == hi->history_count) {
		seq_printf(s, "%x %x\n", hi->comp_update_count,
			   hi->dxacc_update_count);
	} else {
		const size_t offset = *spos * hi->page_size;
		char temp[96];

		format_battery_history_entry(temp, sizeof(temp),
					     &hi->history[offset],
					     hi->page_size);
		seq_printf(s, "%s\n", temp);
	}

	return 0;
}

static const struct seq_operations ct_seq_ops = {
	.start = ct_seq_start,
	.next  = ct_seq_next,
	.stop  = ct_seq_stop,
	.show  = ct_seq_show
};

static int history_dev_open(struct inode *inode, struct file *file)
{
	struct max1720x_chip *chip =
		container_of(inode->i_cdev, struct max1720x_chip, hcdev);
	struct max1720x_history *hi;
	int history_count;

	hi = __seq_open_private(file, &ct_seq_ops, sizeof(*hi));
	if (!hi)
		return -ENOMEM;

	mutex_lock(&chip->history_lock);
	history_count = max1720x_history_read(hi, chip);
	if (history_count < 0) {
		mutex_unlock(&chip->history_lock);
		return history_count;
	} else if (history_count == 0) {
		dev_info(chip->dev, "No battery history has been recorded\n");
	}
	mutex_unlock(&chip->history_lock);

	hi->comp_update_count = hi->dxacc_update_count = -1;
	if (chip->design_capacity != -1)
		hi->dxacc_update_count = chip->dxacc_update_count;
	if (chip->ini_tempco != -1 && chip->ini_rcomp0 != -1)
		hi->comp_update_count = chip->comp_update_count;

	return 0;
}

static int history_dev_release(struct inode *inode, struct file *file)
{
	struct max1720x_history *hi =
		((struct seq_file *)file->private_data)->private;

	if (hi) {
		max1720x_history_free(hi);
		seq_release_private(inode, file);
	}

	return 0;
}

static const struct file_operations hdev_fops = {
	.open = history_dev_open,
	.owner = THIS_MODULE,
	.read = seq_read,
	.release = history_dev_release,
};

static void max1720x_cleanup_history(struct max1720x_chip *chip)
{
	if (chip->history_added)
		cdev_del(&chip->hcdev);
	if (chip->history_available)
		device_destroy(chip->hcclass, chip->hcmajor);
	if (chip->hcclass)
		class_destroy(chip->hcclass);
	if (chip->hcmajor != -1)
		unregister_chrdev_region(chip->hcmajor, 1);
}

static int max1720x_init_history_device(struct max1720x_chip *chip)
{
	struct device *hcdev;

	mutex_init(&chip->history_lock);

	chip->hcmajor = -1;

	/* cat /proc/devices */
	if (alloc_chrdev_region(&chip->hcmajor, 0, 1, HISTORY_DEVICENAME) < 0)
		goto no_history;
	/* ls /sys/class */
	chip->hcclass = class_create(THIS_MODULE, HISTORY_DEVICENAME);
	if (chip->hcclass == NULL)
		goto no_history;
	/* ls /dev/ */
	hcdev = device_create(chip->hcclass, NULL, chip->hcmajor, NULL,
		HISTORY_DEVICENAME);
	if (hcdev == NULL)
		goto no_history;

	chip->history_available = true;
	cdev_init(&chip->hcdev, &hdev_fops);
	if (cdev_add(&chip->hcdev, chip->hcmajor, 1) == -1)
		goto no_history;

	chip->history_added = true;
	return 0;

no_history:
	max1720x_cleanup_history(chip);
	return -ENODEV;
}

static int max1720x_init_history(struct max1720x_chip *chip, int gauge_type)
{
	if (gauge_type == -1)
		return -EINVAL;

	if (gauge_type == MAX1730X_GAUGE_TYPE) {
		chip->nb_history_pages = MAX1730X_N_OF_HISTORY_PAGES;
		chip->history_page_size = MAX1730X_HISTORY_PAGE_SIZE;
		chip->nb_history_flag_reg = MAX1730X_N_OF_HISTORY_FLAGS_REG;
	} else {
		chip->nb_history_pages = MAX1720X_N_OF_HISTORY_PAGES;
		chip->history_page_size = MAX1720X_HISTORY_PAGE_SIZE;
		chip->nb_history_flag_reg = MAX1720X_N_OF_HISTORY_FLAGS_REG;
	}

	return 0;
}

static int max17x0x_storage_info(gbms_tag_t tag, size_t *addr, size_t *count,
				 void *ptr)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)ptr;

	if (!chip->history_available)
		return -ENOENT;

	*count = chip->history_page_size * 2; /* storage is in byte */
	*addr = -1;
	return 0;
}

static int max17x0x_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)ptr;
	static gbms_tag_t keys[] = { GBMS_TAG_SNUM, GBMS_TAG_BCNT,
				     GBMS_TAG_RSOC };
	const int count = ARRAY_SIZE(keys);

	if (index >= 0 && index < count) {
		*tag = keys[index];
	} else if (chip->history_available && index == count) {
		*tag = GBMS_TAG_HIST;
	} else {
		return -ENOENT;
	}

	return 0;
}

/**
 * The standard device call this with !data && !size && index=0 on start and
 * !data && !size && index<0 on stop. The call on start free and reload the
 * history from the gauge potentially increasing the number of entries (note
 * clients will not see that until they call start). On close the code just
 * release the allocated memory and entries: this is not a problem for cliets
 * that might be open because the data will be reloaded on next access.
 * This might create some churn but it's ok since we should not have more than
 * one client for this.
 */
static int max17x0x_storage_history_read(void *buff, size_t size, int index,
					 struct max1720x_chip *chip)
{
	struct max1720x_history *hi = &chip->history_storage;

	/* (!buff || !size) -> free the memory
	 *	if index == INVALID -> return 0
	 *	if index < 0 -> return -EIVAL
	 *	if index >= 0 -> re-read history
	 */
	if (!buff || !size) {
		max1720x_history_free(hi);
		if (index == GBMS_STORAGE_INDEX_INVALID)
			return 0;
	}

	if (index < 0)
		return -EINVAL;

	/* read history if needed */
	if (hi->history_count < 0) {
		int ret;

		ret = max1720x_history_read(hi, chip);
		if (ret < 0)
			return ret;
	}

	/* index == 0 is ok here */
	if (index >= hi->history_count)
		return -ENODATA;

	/* !buff, !size to read iterator count */
	if (!size || !buff)
		return hi->history_count;

	memcpy(buff, &hi->history[index * chip->history_page_size], size);
	return size;
}

static int max17x0x_storage_read_data(gbms_tag_t tag, void *buff, size_t size,
				      int index, void *ptr)
{
	int ret;
	struct max1720x_chip *chip = (struct max1720x_chip *)ptr;

	switch (tag) {
	case GBMS_TAG_HIST:
		/* short reads are invalid */
		if (size && size != chip->history_page_size * 2)
			return -EINVAL;

		mutex_lock(&chip->history_lock);
		ret = max17x0x_storage_history_read(buff, size, index, chip);
		mutex_unlock(&chip->history_lock);
		return ret;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static int max17x0x_storage_read(gbms_tag_t tag, void *buff, size_t size,
				 void *ptr)
{
	int ret;
	const struct max17x0x_reg *reg;
	struct max1720x_chip *chip = (struct max1720x_chip *)ptr;

	switch (tag) {
	case GBMS_TAG_SNUM:
		reg = max17x0x_find_by_tag(MAX17X0X_TAG_SNUM);
		if (reg && reg->size > size)
			return -ERANGE;
	break;
	case GBMS_TAG_BCNT:
		reg = max17x0x_find_by_tag(MAX17X0X_TAG_BCNT);
		if (reg && reg->size != size)
			return -ERANGE;
	break;
	case GBMS_TAG_RSOC:
		reg = max17x0x_find_by_tag(MAX17X0X_TAG_RSOC);
		if (reg && reg->size != size)
			return -ERANGE;
	break;
	default:
		reg = NULL;
		break;
	}

	if (!reg)
		return -ENOENT;

	ret = max17x0x_reg_load(chip->regmap_nvram, reg, buff);
	if (ret == 0)
		ret = reg->size;

	return ret;
}

static int max17x0x_storage_write(gbms_tag_t tag, const void *buff, size_t size,
				  void *ptr)
{
	int ret;
	const struct max17x0x_reg *reg;
	struct max1720x_chip *chip = (struct max1720x_chip *)ptr;

	switch (tag) {
	case GBMS_TAG_BCNT:
		reg = max17x0x_find_by_tag(MAX17X0X_TAG_BCNT);
		if (reg && reg->size != size)
			return -ERANGE;
	break;
	case GBMS_TAG_RSOC:
		reg = max17x0x_find_by_tag(MAX17X0X_TAG_RSOC);
		if (reg && reg->size != size)
			return -ERANGE;
	break;
	default:
		reg = NULL;
		break;
	}

	if (!reg)
		return -ENOENT;

	ret = max17x0x_reg_store(chip->regmap_nvram, reg, buff);
	if (ret == 0)
		ret = reg->size;

	return ret;
}

static struct gbms_storage_desc max17x0x_storage_dsc = {
	.info = max17x0x_storage_info,
	.iter = max17x0x_storage_iter,
	.read = max17x0x_storage_read,
	.write = max17x0x_storage_write,
	.read_data = max17x0x_storage_read_data,
};

static void max17x0x_read_serial_number(struct max1720x_chip *chip)
{
	char buff[32];
	int ret;

	ret = gbms_storage_read(GBMS_TAG_SNUM, buff, sizeof(buff));
	if (ret < 0) {
		/* do nothing */
	} else if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		ret = max1730x_decode_sn(chip->serial_number,
					 sizeof(chip->serial_number),
					 (u16 *)buff);
	} else {
		ret = max1720x_decode_sn(chip->serial_number,
					 sizeof(chip->serial_number),
					 (u16 *)buff);
	}

	if (ret < 0)
		chip->serial_number[0] = '\0';

}

static void max1720x_init_work(struct work_struct *work)
{
	struct max1720x_chip *chip = container_of(work, struct max1720x_chip,
						  init_work.work);
	int ret = 0;

	if (max17xxx_gauge_type != -1) {

		if (chip->regmap_nvram) {
			ret = gbms_storage_register(&max17x0x_storage_dsc,
						    "maxfg", chip);
			if (ret == -EBUSY)
				ret = 0;
		}

		if (ret == 0)
			ret = max1720x_init_chip(chip);
		if (ret == -EPROBE_DEFER) {
			schedule_delayed_work(&chip->init_work,
				msecs_to_jiffies(MAX1720X_DELAY_INIT_MS));
			return;
		}
	}

	max17x0x_read_serial_number(chip);

	chip->prev_charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->fake_capacity = -EINVAL;
	chip->health = -EINVAL;
	init_debugfs(chip);

	/* Handle any IRQ that might have been set before init */
	max1720x_fg_irq_thread_fn(chip->primary->irq, chip);

	ret = max1720x_init_history(chip, max17xxx_gauge_type);
	if (ret == 0) {
		(void)max1720x_init_history_device(chip);
		/* enable workaround for cycle count overflow */
		if (max17xxx_gauge_type == MAX1720X_GAUGE_TYPE)
			chip->cycle_count = -1;
	}

	mutex_init(&chip->cap_estimate.batt_ce_lock);
	ret = batt_ce_load_data(chip->regmap_nvram, &chip->cap_estimate);
	if (ret == 0)
		batt_ce_dump_data(&chip->cap_estimate, chip->ce_log);

	chip->resume_complete = true;
	chip->init_complete = true;
}

static int max1720x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max1720x_chip *chip;
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };

	int ret = 0;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->fake_battery = -1;
	i2c_set_clientdata(client, chip);
	chip->primary = client;
	chip->secondary = i2c_new_secondary_device(client, "nvram", 0xb);
	if (chip->secondary == NULL) {
		dev_err(dev, "Failed to initialize secondary i2c device\n");
		return -EINVAL;
	}
	i2c_set_clientdata(chip->secondary, chip);
	chip->regmap = devm_regmap_init_i2c(client, &max1720x_regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	chip->shadow_override = true;
	if (of_property_read_bool(dev->of_node, "maxim,max1730x-compat")) {
		/* NOTE: NEED TO COME BEFORE REGISTER ACCESS */
		max17xxx_gauge_type = max1720x_read_gauge_type(chip);
		dev_warn(chip->dev, "device gauge_type: %d shadow_override=%d\n",
			max17xxx_gauge_type, chip->shadow_override);
	} else {
		max17xxx_gauge_type = MAX1720X_GAUGE_TYPE;
	}

	/* gauge not present default to 17201 */
	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		chip->regmap_nvram =
		    devm_regmap_init_i2c(chip->secondary,
					 &max1730x_regmap_nvram_cfg);
	else
		chip->regmap_nvram =
		    devm_regmap_init_i2c(chip->secondary,
					 &max1720x_regmap_nvram_cfg);

	if (IS_ERR(chip->regmap_nvram)) {
		dev_err(dev, "Failed to initialize nvram regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	/* NOTE: should probably not request the interrupt when the gauge is
	 * not present (i.e when max17xxx_gauge_type == -1)
	 */
	if (chip->primary->irq) {
		ret = request_threaded_irq(chip->primary->irq, NULL,
					   max1720x_fg_irq_thread_fn,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   MAX1720X_I2C_DRIVER_NAME, chip);
		if (ret != 0) {
			dev_err(chip->dev, "Unable to register IRQ handler\n");
			goto i2c_unregister;
		}

		enable_irq_wake(chip->primary->irq);
	}

	if (of_property_read_bool(dev->of_node, "maxim,psy-type-unknown"))
		max1720x_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;
	chip->psy = devm_power_supply_register(dev,
					       &max1720x_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(dev, "Couldn't register as power supply\n");
		ret = PTR_ERR(chip->psy);
		goto irq_unregister;
	}

	ret = device_create_file(&chip->psy->dev, &dev_attr_offmode_charger);
	if (ret) {
		dev_err(dev, "Failed to create offmode_charger attribute\n");
		goto psy_unregister;
	}

	chip->ce_log = debugfs_logbuffer_register("batt_ce");
	if (IS_ERR(chip->ce_log)) {
		ret = PTR_ERR(chip->ce_log);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", ret);
		chip->ce_log = NULL;
	}

	chip->maxfg_log = debugfs_logbuffer_register("maxfg");
	if (IS_ERR(chip->maxfg_log)) {
		ret = PTR_ERR(chip->maxfg_log);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", ret);
		chip->maxfg_log = NULL;
	}

	/* NOTE: set to AvSOC and filter on google battery side */
	chip->reg_prop_capacity_raw =  MAX1720X_REPSOC;
	INIT_DELAYED_WORK(&chip->cap_estimate.settle_timer,
			  batt_ce_capacityfiltered_work);
	INIT_DELAYED_WORK(&chip->init_work, max1720x_init_work);
	INIT_DELAYED_WORK(&chip->filtercfg_work, max1720x_filtercfg_work);
	INIT_DELAYED_WORK(&chip->dumpreg_work, max1720x_dumpreg_work);

	schedule_delayed_work(&chip->init_work, 0);

	return 0;

psy_unregister:
	power_supply_unregister(chip->psy);
irq_unregister:
	free_irq(chip->primary->irq, chip);
i2c_unregister:
	i2c_unregister_device(chip->secondary);

	return ret;
}

static int max1720x_remove(struct i2c_client *client)
{
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	if (chip->ce_log) {
		debugfs_logbuffer_unregister(chip->ce_log);
		chip->ce_log = NULL;
	}
	max1720x_cleanup_history(chip);
	cancel_delayed_work(&chip->init_work);
	iio_channel_release(chip->iio_ch);
	if (chip->primary->irq)
		free_irq(chip->primary->irq, chip);
	power_supply_unregister(chip->psy);
	i2c_unregister_device(chip->secondary);

	return 0;
}

static const struct of_device_id max1720x_of_match[] = {
	{ .compatible = "maxim,max1720x"},
	{},
};
MODULE_DEVICE_TABLE(of, max1720x_of_match);

static const struct i2c_device_id max1720x_id[] = {
	{"max1720x", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max1720x_id);

#ifdef CONFIG_PM_SLEEP
static int max1720x_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = false;
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int max1720x_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = true;
	pm_runtime_put_sync(chip->dev);

	return 0;
}
#endif

static const struct dev_pm_ops max1720x_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(max1720x_pm_suspend, max1720x_pm_resume)
};

static struct i2c_driver max1720x_i2c_driver = {
	.driver = {
		   .name = "max1720x",
		   .of_match_table = max1720x_of_match,
		   .pm = &max1720x_pm_ops,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.id_table = max1720x_id,
	.probe = max1720x_probe,
	.remove = max1720x_remove,
};

module_i2c_driver(max1720x_i2c_driver);
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_DESCRIPTION("MAX17x01/MAX17x05 Fuel Gauge");
MODULE_LICENSE("GPL");
