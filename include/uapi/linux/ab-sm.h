/*
 * Driver interface for the AB State Manager
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __UAPI_AB_SM_H__
#define __UAPI_AB_SM_H__

#include <linux/ioctl.h>

#define AB_SM_FATAL_EL2_ERROR_FLAG	0x01
#define AB_SM_COMPUTE_READY_MASK	0x01
#define AB_SM_DRAM_INTACT_MASK		0x02

#define AB_SM_IOCTL_MAGIC	'a'

/* Enters secure state. Blocks until ready.
 * On success will return 0, otherwise will return error < 0.
 * -ETIMEDOUT: state change for thermal disabling timed out.
 * -EBUSY: already throttled to suspend due to system overheated.
 */
#define AB_SM_ENTER_EL2		_IO(AB_SM_IOCTL_MAGIC, 3)

/* Exits secure state and blocks until cleanup is complete.
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_EXIT_EL2		_IO(AB_SM_IOCTL_MAGIC, 4)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_IPU_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 5, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_TPU_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 6, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_DDR_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 7, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 8, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_AON_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 9, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_IPU_STATE	_IOW(AB_SM_IOCTL_MAGIC, 10, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_TPU_STATE	_IOW(AB_SM_IOCTL_MAGIC, 11, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_DDR_STATE	_IOW(AB_SM_IOCTL_MAGIC, 12, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_STATE	_IOW(AB_SM_IOCTL_MAGIC, 13, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new ipu state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_IPU_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 14, \
		struct new_block_props *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new tpu state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_TPU_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 15, \
		struct new_block_props *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new dram state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_DRAM_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 16, \
		struct new_block_props *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new mif state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_MIF_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 17, \
		struct new_block_props *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new fsys state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_FSYS_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 18, \
		struct new_block_props *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter struct new_block_props *:
 *	Pass new aon state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_AON_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 19, \
		struct new_block_props *)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with state value
 *	On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_GET_EL2_MODE	_IOR(AB_SM_IOCTL_MAGIC, 20, int *)

/* First call after open will immediately return the current
 * state. Subsequent calls will block until next state change,
 * then return the new state.
 *
 * Parameter int *:
 *	Pass pointer to integer to be filled in with mapped state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_MAPPED_ASYNC_NOTIFY	_IOR(AB_SM_IOCTL_MAGIC, 21, int *)

/*
 * Parameter int:
 *	Pass mapped state value to set
 * On success will return 0, otherwise will return error < 0.
 * Note: State changes are not allowed after calling AB_SM_ENTER_EL2
 *     until subsequently calling AB_SM_EXIT EL2.
 *     All calls to AB_SM_SET_STATE will fail with -ENODEV
 *     during that time.
 */
#define AB_SM_MAPPED_SET_STATE		_IOW(AB_SM_IOCTL_MAGIC, 22, int)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with mapped state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_MAPPED_GET_STATE		_IOR(AB_SM_IOCTL_MAGIC, 23, int *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *	Pass new L1ss entry delay to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_L1SS_DELAY  _IOW(AB_SM_IOCTL_MAGIC, 24, int)

/*
 * Call will block until there is a throttle to no-compute
 * event.
 *
 * Parameter int*:
 *  Bit [0:0] (compute_ready): On return if bit is set to 0,
 *     caller is expected to prepare for a no-compute scenario,
 *     and then call back into this ioctl. The throttle event will
 *     wait until all listeners call back into this ioctl, or a timeout occurs.
 *     On return if bit is set to 1, the no-compute scenario
 *     is over and normal operation can continue. Caller is expected to
 *     call back into this ioctl to signal it has received the message.
 *  Bit [1:1] (dram_intact): On return if compute_ready is set to 1, this
 *     bit signals whether the DRAM remained valid during the compute-not-ready
 *     state. If dram_intact bit is set to 1, DRAM data is still valid.
 *     Otherwise, DRAM data was lost.
 *     This bit should be ignored if compute_ready bit is 0.
 */
#define AB_SM_COMPUTE_READY_NOTIFY	_IOR(AB_SM_IOCTL_MAGIC, 25, int *)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *  Pass new throttle level to set
 *  THROTTLE_NONE	= 0
 *  THROTTLE_TO_MID	= 1
 *  THROTTLE_TO_LOW	= 2
 *  THROTTLE_TO_MIN = 3
 *  THROTTLE_NOCOMPUTE = 4
 *
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_THROTTLE_LEVEL		_IOW(AB_SM_IOCTL_MAGIC, 26, int)

/*
 * WARNING: Debug only, will be removed in production
 * Parameter int:
 *  Pass 0 to disable all throttling
 *  Pass 1 to enable all throttling
 *
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_ENABLE_THERMAL			_IOW(AB_SM_IOCTL_MAGIC, 27, int)

/* Exits secure state and blocks until cleanup is complete.
 * Parameter int:
 *  Bitwise flag param used to notify ABSM of any issues
 *  which occurred during EL2 mode.
 *   Bit  [0]    - AB_SM_FATAL_EL2_ERROR_FLAG - A reset occurred during EL2
 *   Bits [1:31] - Reserved
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_EXIT_EL2_WITH_FLAG		_IOW(AB_SM_IOCTL_MAGIC, 28, int)

/* Get the current thermal (aka throttle) level.
 * Parameter int *:
 *   Filled in with current thermal level, which is a value
 *   ranging from 0 - 4.
 *   Level 0: No throttling is present
 *   Level 1: Minimum throttle level, IPU/TPU clocks reduced
 *   Level 2: Medium throttle level, IPU/TPU clocks reduced further
 *   Level 3: High throttle level, IPU/TPU clocks reduced further
 *   Level 4: Maximum throttle level, IPU/TPU blocks off, AB placed in S100
 */
#define AB_SM_GET_THERMAL_LEVEL		_IOR(AB_SM_IOCTL_MAGIC, 29, int *)

#define AB_CHIP_ID_UNKNOWN	-1
#define AB_CHIP_ID_A0		0
#define AB_CHIP_ID_B0		1

#define UAPI_BLK_(num, pmu, rail, v, clk, freq, pwr, used, tiles, dr) \
	{					\
		UAPI_BLOCK_STATE_ ## num,	\
		(enum uapi_pmu_state)pmu,	\
		uapi_ ## rail,			\
		UAPI_VOLTAGE_ ## v,		\
		uapi_ ## clk,			\
		(__u64)(1000000. * freq),	\
		pwr,				\
		used,				\
		tiles,				\
		dr,				\
	}

/* Keep in sync with enum block_state in airbrush-sm-ctrl.h. */
#define AB_SM_UAPI_NUM_BLOCK_STATES	12
enum uapi_block_state {
	UAPI_BLOCK_STATE_0 = 0,
	UAPI_BLOCK_STATE_100 = 100,
	UAPI_BLOCK_STATE_101,
	UAPI_BLOCK_STATE_200 = 200,
	UAPI_BLOCK_STATE_201,
	UAPI_BLOCK_STATE_202,
	UAPI_BLOCK_STATE_300 = 300,
	UAPI_BLOCK_STATE_301,
	UAPI_BLOCK_STATE_302,
	UAPI_BLOCK_STATE_303,
	UAPI_BLOCK_STATE_304,
	UAPI_BLOCK_STATE_305,
};

/* Keep in sync with enum pmu_states in airbrush_sm_ctrl.h. */
enum uapi_pmu_state {
	UAPI_PMU_STATE_ON = 0,
	UAPI_PMU_STATE_SLEEP,
	UAPI_PMU_STATE_DEEP_SLEEP,
	UAPI_PMU_STATE_OFF
};

/* Keep in sync with enum states in airbrush-sm-ctrl.h. */
enum uapi_state {
	uapi_off = 0,
	uapi_on = 1,
};

/* Keep in sync with enum logic_voltage in airbrush-sm-ctrl.h. */
enum uapi_logic_voltage {
	UAPI_VOLTAGE_0_0,
	UAPI_VOLTAGE_0_60,
	UAPI_VOLTAGE_0_75,
	UAPI_VOLTAGE_0_85,
};

/**
 * Stores information of the soc block's operating state.
 * Similar to struct block_property in airbrush-sm-ctrl.h.
 */
struct uapi_block_properties {
	enum uapi_block_state id;
	enum uapi_pmu_state pmu;
	enum uapi_state rail_en;
	enum uapi_logic_voltage logic_voltage;
	enum uapi_state clk_status;
	__u64 clk_frequency;
	__u32 num_powered_cores;
	__u32 num_computing_cores;
	__u32 num_powered_tiles;
	__u32 data_rate;
};

struct new_block_props {
	struct uapi_block_properties table[AB_SM_UAPI_NUM_BLOCK_STATES];
};


#endif /* __UAPI_AB_SM_H__ */
