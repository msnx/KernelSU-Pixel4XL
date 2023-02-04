/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush State Manager Control driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_SM_CTRL_H
#define _AIRBRUSH_SM_CTRL_H

#include <linux/airbrush-sm-notifier.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#if IS_ENABLED(CONFIG_PCI_MSM)
#include <linux/msm_pcie.h>
#endif
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define NUM_BLOCKS 6

#define __GPIO_ENABLE	0x1
#define __GPIO_DISABLE	0x0

#define AB_SM_OSC_RATE 19200000

#define REGULATOR_STEP 6250

#define AB_SM_STATE_IN_RANGE(chip_state, state_range) \
	((chip_state / 100) == (state_range / 100))

enum block_name {
	BLK_IPU,
	BLK_TPU,
	DRAM,
	BLK_MIF,
	BLK_FSYS,
	BLK_AON,
};

enum states {
	off = 0,
	on = 1,
};

/*
 * Read a register from the address,addr of ABC device and put it in to_addr,
 * all reads are currently happening through PCIe
 */
#define ABC_READ(addr, to_addr) \
	abc_pcie_config_read((addr) & 0xffffff, 0x0, (to_addr))

/*
 * Write value to an ABC register address, addr
 * All writes are currently happening through PCIe
 */
#define ABC_WRITE(addr, value) \
	abc_pcie_config_write((addr) & 0xffffff, 0x0, (value))

/*Should be in ascending order (for comparisons)*/
enum logic_voltage {
	VOLTAGE_0_0,
	VOLTAGE_0_60,
	VOLTAGE_0_75,
	VOLTAGE_0_85,
};

enum ddr_state {
	DDR_ON,
	DDR_SLEEP,
	DDR_SUSPEND,
	DDR_OFF,
};

enum throttle_state {
	THROTTLE_NONE = 0,
	THROTTLE_TO_MID,
	THROTTLE_TO_LOW,
	THROTTLE_TO_MIN,
	THROTTLE_NOCOMPUTE,
};

enum chip_state {
	CHIP_STATE_0 = 0,
	CHIP_STATE_100 = 100,
	CHIP_STATE_200 = 200,
	CHIP_STATE_300 = 300,
	CHIP_STATE_301,
	CHIP_STATE_302,
	CHIP_STATE_303,
	CHIP_STATE_400 = 400,
	CHIP_STATE_401,
	CHIP_STATE_402,
	CHIP_STATE_403,
	CHIP_STATE_404,
	CHIP_STATE_405,
	CHIP_STATE_406,
	CHIP_STATE_407,
	CHIP_STATE_408,
	CHIP_STATE_409,
	CHIP_STATE_500 = 500,
	CHIP_STATE_501,
	CHIP_STATE_502,
	CHIP_STATE_503,
	CHIP_STATE_504,
	CHIP_STATE_505,
	CHIP_STATE_600 = 600,
	CHIP_STATE_601,
	CHIP_STATE_602,
	CHIP_STATE_603,
	CHIP_STATE_604,
	CHIP_STATE_605,
	CHIP_STATE_700 = 700,
	CHIP_STATE_701,
	CHIP_STATE_702,
	CHIP_STATE_703,
	CHIP_STATE_704,
	CHIP_STATE_705,
	CHIP_STATE_800 = 800,
	CHIP_STATE_801,
	CHIP_STATE_802,
	CHIP_STATE_803,
	CHIP_STATE_804,
	CHIP_STATE_805,
	CHIP_STATE_900 = 900,
};

#define CHIP_STATE_OFF				CHIP_STATE_0
#define CHIP_STATE_SUSPEND			CHIP_STATE_100
#define CHIP_STATE_DEEP_SLEEP		CHIP_STATE_200
#define CHIP_STATE_SLEEP			CHIP_STATE_300
#define CHIP_STATE_SLEEP_IPU_UP		CHIP_STATE_301
#define CHIP_STATE_SLEEP_TPU_UP		CHIP_STATE_302
#define CHIP_STATE_SLEEP_BOTH_UP	CHIP_STATE_303
#define CHIP_STATE_ACTIVE_MAX		CHIP_STATE_409
#define CHIP_STATE_SECURE_APP		CHIP_STATE_900

enum block_state {
	BLOCK_STATE_0 = 0,
	BLOCK_STATE_100 = 100,
	BLOCK_STATE_101,
	BLOCK_STATE_200 = 200,
	BLOCK_STATE_201,
	BLOCK_STATE_202,
	BLOCK_STATE_300 = 300,
	BLOCK_STATE_301,
	BLOCK_STATE_302,
	BLOCK_STATE_303,
	BLOCK_STATE_304,
	BLOCK_STATE_305,
};

enum stat_state {
	STAT_STATE_ACTIVE = 0,
	STAT_STATE_SLEEP,
	STAT_STATE_DEEP_SLEEP,
	STAT_STATE_SUSPEND,
	STAT_STATE_OFF,
	STAT_STATE_UNKNOWN,
	STAT_STATE_SIZE,
};

enum pmu_states {
	PMU_STATE_OFF = 0,
	PMU_STATE_DEEP_SLEEP,
	PMU_STATE_SLEEP,
	PMU_STATE_ON,
};

/**
 * struct block_property
 * stores the information of a soc block's operating state.
 *
 * @id: The block state id of the SOC block.
 * @state_name: the name of the corresponing block state
 * @substate_name: the name of the corresponding substate.
 * @voltage_rate_status: status of the voltage rail, (on/off)
 * @logic_voltage: the voltage provoded to the block in volts (mult by 100).
 * @clock_status: status of the clock tree which provides the clock
 * @clk_frequency: frequency of the clock in Hz
 * @num_powered_cores: number of cores that are powered up.
 * @num_computing_cores: Number of cores that are used for computation.
 * @num_powered_tiles: Number of powered tiles.
 * @data_rate: Rate of data transfer.
 */
struct block_property {
	enum block_state id;
	char *state_name;
	char *substate_name;
	enum pmu_states pmu;
	enum states rail_en;
	enum logic_voltage logic_voltage;
	enum states clk_status;
	u64 clk_frequency;
	u32 num_powered_cores;
	u32 num_computing_cores;
	u32 num_powered_tiles;
	u32 data_rate;
};

typedef int (*ab_sm_set_block_state_t)(
		const struct block_property *current_property,
		const struct block_property *desired_property,
		enum block_state block_substate_id, void *data);

/**
 * struct block - stores the information about a SOC block
 *
 * @name: name of the block
 * @current_state: current poperties of the block
 * @prop_table: table containing details of all the states
 * @nr_block_states: number of possible states for this block
 * @set_state: callback function for the block
 * @data: callback data
 */
struct block {
	enum block_name name;
	struct block_property *current_state;
	struct block_property *prop_table;
	u32 nr_block_states;
	ab_sm_set_block_state_t set_state;
	void *data; /*IP specific data*/
};

struct chip_to_block_map {
	enum chip_state chip_substate_id;
	enum block_state ipu_block_state_id;
	enum block_state tpu_block_state_id;
	enum block_state dram_block_state_id;
	enum block_state mif_block_state_id;
	enum block_state fsys_block_state_id;
	enum block_state aon_block_state_id;
};

enum ab_sm_time_stamps {
	/* Bootsequence */
	AB_SM_TS_BOOT_SEQ,
	AB_SM_TS_GET_RESOURCES,
	AB_SM_TS_ALT_BOOT,
	AB_SM_TS_PCIE_ENUM,
	AB_SM_TS_LVCC_INIT,
	AB_SM_TS_AB_READY_NOTIFY,
	AB_SM_TS_CLK_INIT,
	AB_SM_TS_DDR_INIT,
	AB_SM_TS_DDR_SETUP,
	AB_SM_TS_DDR_M0_INIT_INTERNAL,
	AB_SM_TS_DDR_INIT_INTERNAL,
	AB_SM_TS_DDR_TRAIN,

	/* PMIC On */
	AB_SM_TS_PMIC_ON,
	AB_SM_TS_LVCC,
	AB_SM_TS_AON_DRAM_INIT,

	/* IPU/TPU blk_set_state */
	AB_SM_TS_IPU_TPU,
	AB_SM_TS_IPU_TPU_PMU_RES,
	AB_SM_TS_IPU_TPU_CLK,

	/*IPU blk_set_state*/
	AB_SM_TS_IPU,
	AB_SM_TS_IPU_PMU_RES,
	AB_SM_TS_IPU_APB_FIX,
	AB_SM_TS_IPU_CLK,
	AB_SM_TS_IPU_PRE_RC_NOTIFY,
	AB_SM_TS_IPU_GET_CLK,
	AB_SM_TS_IPU_SET_OSCCLK,
	AB_SM_TS_IPU_SET_CLKRATE,
	AB_SM_TS_IPU_CLK_LOCK,
	AB_SM_TS_IPU_FINISH_SET_CLKRATE,
	AB_SM_TS_IPU_POST_RC_NOTIFY,
	AB_SM_TS_IPU_PMU_SLEEP,

	/* TPU blk_set_state */
	AB_SM_TS_TPU,
	AB_SM_TS_TPU_PMU_RES,
	AB_SM_TS_TPU_APB_FIX,
	AB_SM_TS_TPU_CLK,
	AB_SM_TS_TPU_PRE_RC_NOTIFY,
	AB_SM_TS_TPU_GET_CLK,
	AB_SM_TS_TPU_SET_OSCCLK,
	AB_SM_TS_TPU_SET_CLKRATE,
	AB_SM_TS_TPU_CLK_LOCK,
	AB_SM_TS_TPU_FINISH_SET_CLKRATE,
	AB_SM_TS_TPU_POST_RC_NOTIFY,
	AB_SM_TS_TPU_PMU_SLEEP,
	AB_SM_TS_TPU_PREP_SLEEP,
	AB_SM_TS_TPU_POLL_SLEEP,
	AB_SM_TS_PMU_DEEP_SLEEP,

	AB_SM_TS_IPU_TPU_PMU_SLEEP,

	/* DRAM blk_set_state */
	AB_SM_TS_DRAM,
	AB_SM_TS_DDR_CB,
	AB_SM_TS_DDR_SET_PLL,
	AB_SM_TS_DDR_SET_PLL_POLL,
	AB_SM_TS_DDR_FINISH_PLL,
	AB_SM_TS_DDR_EXIT_SR_POWER_DIS,
	AB_SM_TS_DDR_EXIT_SR_ENABLE_DLL,
	AB_SM_TS_DDR_EXIT_SR_POWER_EN,
	AB_SM_TS_DDR_EXIT_SR_MODE,
	AB_SM_TS_DDR_EXIT_SR_FINISH,

	AB_SM_TS_MIF,

	/* FSYS blk_set_state */
	AB_SM_TS_FSYS,
	AB_SM_TS_PCIE_CB,
	AB_SM_TS_PCIE_GET_LINKSPEED,
	AB_SM_TS_PCIE_GET_LINKSTATE,
	AB_SM_TS_PCIE_SET_LINKSPEED,
	AB_SM_TS_PCIE_SET_LINKSTATE,

	/* AON blk_set_state */
	AB_SM_TS_AON,
	AB_SM_TS_AON_CLK,

	/* PMIC Off*/
	AB_SM_TS_PMIC_OFF,

	AB_SM_TS_FULL,

	NUM_AB_SM_TS,
};

enum ab_chip_id {
	CHIP_ID_UNKNOWN = -1,
	CHIP_ID_A0 = 0,
	CHIP_ID_B0,
};

struct ab_sm_state_stat {
	u64 counter;		/* cumulative */
	ktime_t duration;	/* cumulative */
	ktime_t last_entry;
	ktime_t last_exit;
};

struct ab_sm_pmu_ops {
	void *ctx;

	int (*pmu_ipu_sleep)(void *ctx);
	int (*pmu_tpu_sleep)(void *ctx);
	int (*pmu_ipu_tpu_sleep)(void *ctx);
	int (*pmu_deep_sleep)(void *ctx);
	int (*pmu_ipu_resume)(void *ctx);
	int (*pmu_tpu_resume)(void *ctx);
	int (*pmu_ipu_tpu_resume)(void *ctx);
};

struct ab_sm_clk_ops {
	void *ctx;

	void (*init)(void *ctx);
	void (*clear_cache)(void *ctx, enum block_name name);

	int64_t (*ipu_set_rate)(void *ctx, u64 old_rate, u64 new_rate);
	int64_t (*tpu_set_rate)(void *ctx, u64 old_rate, u64 new_rate);
	int64_t (*ipu_tpu_set_rate)(void *ctx,
			u64 old_ipu_rate, u64 new_ipu_rate,
			u64 old_tpu_rate, u64 new_tpu_rate);
	int64_t (*aon_set_rate)(void *ctx, u64 old_rate, u64 new_rate);
	int64_t (*aon_set_pll_div)(void *ctx, uint32_t div);
};

struct ab_sm_dram_ops {
	void *ctx;

	int (*setup)(void *ctx, void *ab_state_ctx);
	int (*wait_for_m0_ddr_init)(void *ctx);
	int (*init)(void *ctx);
	int (*train_all)(void *ctx);
	int (*get_freq)(void *ctx, u64 *val);
	int (*set_freq)(void *ctx, u64 val);
	int (*suspend)(void *ctx);
	int (*resume)(void *ctx);
	int (*sref_enter)(void *ctx);
	int (*sref_exit)(void *ctx);
	int (*rw_test)(void *ctx, unsigned int read_write);
	int (*eye_margin)(void *ctx, unsigned int test_data);
	int (*eye_margin_plot)(void *ctx);
	int (*ppc_set_event)(void *ctx,
			     unsigned int counter_idx, unsigned int event);
	void (*ppc_ctrl)(void *ctx, int is_start);
};

struct ab_sm_mfd_ops {
	void *ctx;

	// TODO: Define mfd ops
	int (*enter_el2)(void *ctx);
	int (*exit_el2)(void *ctx);

	int (*get_chip_id)(void *ctx, enum ab_chip_id *val);
	int (*ab_ready)(void *ctx);
	int (*pcie_pre_disable)(void *ctx);
	int (*pcie_linkdown)(void *ctx);
};

struct ab_thermal;

struct ab_change_req {
	u32 new_state;
	int *ret_code;
	struct completion *comp;
};

struct ab_asv_info {
	bool fusing_done;
	int asv_version;
	u32 ipu_volt;
	u32 tpu_volt;
	u32 ipu_ro;
	u32 tpu_ro;
	u32 last_volt;
};

/**
 * struct ab_state_context - stores the context of airbrush soc
 */
struct ab_state_context {
	struct platform_device *pdev;
	struct device *dev;
	struct miscdevice misc_dev;

	struct task_struct *state_change_task;

	struct block blocks[NUM_BLOCKS];
	enum throttle_state throttle_state_id;
	enum chip_state dest_chip_substate_id;
	enum chip_state curr_chip_substate_id;
	struct chip_to_block_map *chip_state_table;
	u32 nr_chip_states;

	struct kfifo state_change_reqs;
	enum ab_chip_id chip_id;

	/* Synchronization structs */
	struct mutex set_state_lock;
	struct mutex state_transitioning_lock;
	struct completion request_state_change_comp;
	struct completion transition_comp;
	struct completion notify_comp;
	struct completion shutdown_comp;

	int change_ret;

	/* pins used in bootsequence */
	struct gpio_desc *soc_pwrgood;	/* output */
	struct gpio_desc *fw_patch_en;	/* output */
	struct gpio_desc *ab_ready;	/* input  */
	struct gpio_desc *ddr_sr;	/* output */
	struct gpio_desc *ddr_iso;	/* output */
	struct gpio_desc *ddr_train;	/* output */
	struct gpio_desc *cke_in;	/* output */
	struct gpio_desc *cke_in_sense;	/* output */

	unsigned int ab_ready_irq;	/* ab_ready_gpio irq */

	/*
	 * When alternate_boot == 1, M0 is paused upon power on or resume
	 * to wait for SPI commands.  The SPI commands are implementation
	 * specific.
	 */
	int alternate_boot;

	/* Check for OTP bypass */
	int otp_bypass;

	/* Apply DDRCKE_ISO workaround if specified. */
	int ddrcke_iso_clamp_wr;

	/* if set, skip pcie resume operation for debugging purpose */
	bool debug_skip_pcie_link_init;

	struct ab_asv_info asv_info;

	/* regulator descriptors */
	struct regulator *smps1;
	struct regulator *smps2;
	struct regulator *smps3;
	struct regulator *ldo1;
	struct regulator *ldo2;
	struct regulator *ldo3;
	struct regulator *ldo4;
	struct regulator *ldo5;

	bool smps1_state;
	bool smps2_state;
	bool smps3_state;
	bool ldo1_state;
	bool ldo2_state;
	bool ldo3_state;
	bool ldo4_state;
	bool ldo5_state;

	u64 smps2_delay;
	u64 ldo4_delay;
	u64 ldo5_delay;
	u64 s60_delay;

#if IS_ENABLED(CONFIG_PCI_MSM)
	struct msm_pcie_register_event pcie_link_event;
#endif

	/* EL2 notification structs */
	/* NOTE: el2_in_secure_context represents the true state of the EL2
	 * secure mode. When el2_in_secure_context is true, the PCIe link is
	 * mapped to the secure world and EL1 (the kernel) is not allowed
	 * to interact with it.
	 */
	bool el2_in_secure_context; /* GUARDED_BY state_transitioning_lock */
	struct delayed_work el2_notif_init;
	struct notifier_block el2_notif_nb;
	void *el2_notif_handle;
	struct mutex el2_notif_init_lock;
	bool sm_exiting; /* GUARDED_BY el2_notif_init_lock */

#define AB_SM_CLEANUP_NOT_IN_PROGRESS 0
#define AB_SM_CLEANUP_IN_PROGRESS 1
	atomic_t is_cleanup_in_progress;
	struct notifier_block regulator_nb; /* single notifier */
	struct work_struct shutdown_work; /* emergency shutdown work */

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
	struct dentry *d_entry;
#endif
	atomic_t clocks_registered;
	void *ddr_data; /* ddr private data */
	struct pci_dev *pcie_dev;
	bool cold_boot;

	atomic_t async_in_use;
	struct mutex async_fifo_lock;
	struct kfifo *async_entries;

	/* List of listeners that subscribes to clk rate change events. */
	struct blocking_notifier_head clk_subscribers;

	/*
	 * Similar to clk_subscribers, but is ordered in a specific way
	 * that is preferrable to dma driver.
	 */
	struct blocking_notifier_head clk_subscribers_dma;

	/* power state stats */
	struct ab_sm_state_stat state_stats[STAT_STATE_SIZE];

	// MFD child operations
	struct mutex		op_lock;
	struct ab_sm_pmu_ops	*pmu_ops;
	struct ab_sm_clk_ops	*clk_ops;
	struct ab_sm_dram_ops	*dram_ops;

	/* NOTE: separate lock needed for mfd ops, as enter/exit_el2
	 * will likely cause mfd to call unregister methods for other ops.
	 * Would deadlock if using op_lock.
	 */
	struct mutex		mfd_lock;
	struct ab_sm_mfd_ops	*mfd_ops;

	struct ab_thermal *thermal;

	/* Fired when a throttle to nocompute event
	 * is about to occur
	 */
	struct completion throttle_nocompute_event;
	/* Fired when all listeners are ready to continue
	 * with the throttle to nocompute event
	 */
	struct completion throttle_nocompute_ready;
	/* Number of listeners who must be ready for the
	 * nocompute event to continue
	 * Guarded by throttle_ready_lock
	 */
	u32 req_thermal_listeners;
	/* Current number of listeners waiting/ready for
	 * a nocompute event.
	 * Guarded by throttle_ready_lock
	 */
	u32 curr_thermal_listeners;
	struct mutex throttle_ready_lock;
	/* True when we are waiting for listeners to be
	 * ready for nocompute event
	 * Guarded by throttle_ready_lock
	 */
	bool throttle_nocomp_waiting;
	/* True if we are transitioning to a compute ready situation
	 * False if we are leaving a compute ready situation
	 * Guarded by throttle_ready_lock
	 */
	bool going_to_comp_ready;
	/* True if DRAM data survived throughout a nocompute phase
	 * False if DRAM data was cleared
	 * Guarded by throttle_ready_lock
	 */
	bool dram_survived_no_comp;

	int el2_mode; /* Guarded by state_transitioning_lock */
	/* State we return to upon exit of el2 mode */
	enum chip_state return_chip_substate_id;

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
	/* time stamps */
	bool ts_enabled;
	u64 state_trans_ts[NUM_AB_SM_TS];
	u64 state_start_ts[NUM_AB_SM_TS];
	u64 state_first_ts[NUM_AB_SM_TS];
#endif
	int clkout_idx;
	int clkout_blk_idx;
	int clkout_clk_idx;
};

struct ab_sm_misc_session {
	struct ab_state_context *sc;
	bool first_entry;
	struct kfifo async_entries;
};

#if IS_ENABLED(CONFIG_PCI_MSM)
/*
 *  Set up listener to pcie linkdown event.
 *  Call only once after enumeration is done.
 */
int ab_sm_setup_pcie_event(struct ab_state_context *sc);
#else
static inline int ab_sm_setup_pcie_event(struct ab_state_context *sc)
{
	return 0;
}
#endif /* CONFIG_PCI_MSM */

/*
 *  void ab_sm_register_blk_callback
 *  register block specific state change callback
 *
 *  name: name of the block for which this callback should be called.
 *  callback: set_state callback function.
 *  block_property: block_property structure passed to callback.
 *  data: the cookie that is passed back to the callback.
 */
void ab_sm_register_blk_callback(enum block_name name,
		ab_sm_set_block_state_t callback, void *data);

void ab_sm_register_pmu_ops(struct ab_sm_pmu_ops *ops);
void ab_sm_unregister_pmu_ops(void);

void ab_sm_register_clk_ops(struct ab_sm_clk_ops *ops);
void ab_sm_unregister_clk_ops(void);

void ab_sm_register_dram_ops(struct ab_sm_dram_ops *ops);
void ab_sm_unregister_dram_ops(void);

void ab_sm_register_mfd_ops(struct ab_sm_mfd_ops *ops);
void ab_sm_unregister_mfd_ops(void);

int ab_sm_init(struct platform_device *pdev);
void ab_sm_exit(struct platform_device *pdev);

int ab_sm_set_state(struct ab_state_context *sc, u32 to_chip_substate_id);
u32 ab_sm_get_state(struct ab_state_context *sc);

int ab_bootsequence(struct ab_state_context *ab_ctx,
		enum chip_state prev_state);

enum ab_chip_id ab_get_chip_id(struct ab_state_context *sc);
enum ab_chip_id ab_get_raw_chip_id(struct ab_state_context *sc);

const enum stat_state ab_chip_state_to_stat_state(enum chip_state id);

int ab_sm_enter_el2(struct ab_state_context *sc);
int ab_sm_exit_el2(struct ab_state_context *sc, u32 exit_flag);

void ab_enable_pgood(struct ab_state_context *ab_ctx);
void ab_disable_pgood(struct ab_state_context *ab_ctx);

void ab_gpio_enable_ddr_sr(struct ab_state_context *ab_ctx);
void ab_gpio_disable_ddr_sr(struct ab_state_context *ab_ctx);
int  ab_gpio_get_ddr_sr(struct ab_state_context *ab_ctx);

void ab_gpio_enable_ddr_iso(struct ab_state_context *ab_ctx);
void ab_gpio_disable_ddr_iso(struct ab_state_context *ab_ctx);
int  ab_gpio_get_ddr_iso(struct ab_state_context *ab_ctx);

void ab_gpio_enable_fw_patch(struct ab_state_context *ab_ctx);
void ab_gpio_disable_fw_patch(struct ab_state_context *ab_ctx);

int ab_sm_enumerate_pcie(struct ab_state_context *ab_ctx);
int ab_sm_enable_pcie(struct ab_state_context *ab_ctx);
int ab_sm_disable_pcie(struct ab_state_context *ab_ctx);

void ab_clkout_sel(struct ab_state_context *sc, unsigned int clkout_idx);
void ab_clkout_blksel(struct ab_state_context *sc, unsigned int blk_idx);
void ab_clkout_clksel(struct ab_state_context *sc, unsigned int clk_idx);
int ab_clkout_enable(struct ab_state_context *sc, unsigned int enable);
int ab_clkout_freq(struct ab_state_context *sc, u64 *val);

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
void ab_sm_create_debugfs(struct ab_state_context *sc);
void ab_sm_remove_debugfs(struct ab_state_context *sc);
#else
static inline void ab_sm_create_debugfs(struct ab_state_context *sc) {}
static inline void ab_sm_remove_debugfs(struct ab_state_context *sc) {}
#endif

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
void ab_sm_start_ts(int ts);
void ab_sm_record_ts(int ts);
void ab_sm_zero_ts(struct ab_state_context *sc);
void ab_sm_print_ts(struct ab_state_context *sc);
#else
static inline void ab_sm_start_ts(int ts) {}
static inline void ab_sm_record_ts(int ts) {}
static inline void ab_sm_zero_ts(struct ab_state_context *sc) {}
static inline void ab_sm_print_ts(struct ab_state_context *sc) {}
#endif

void ab_sm_create_sysfs(struct ab_state_context *sc);
void ab_sm_remove_sysfs(struct ab_state_context *sc);

void ab_lvcc_init(struct ab_asv_info *info);
void set_asv_version(struct ab_asv_info *info, int asv_version);
int ab_lvcc(struct ab_state_context *sc, int chip_state);

u32 get_ipu_ro(void);
u32 get_tpu_ro(void);
int chip_info_open(struct inode *inode, struct  file *file);

#endif /* _AIRBRUSH_SM_CTRL_H */
