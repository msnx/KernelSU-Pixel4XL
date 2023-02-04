/*
 * Power management support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-power.h"
#include "ipu-regs.h"
#include "ipu-stp.h"

#define IO_POWER_RAMP_TIME 10 /* us */

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */

static inline void ipu_power_enable_core_idle_clock_gating(
		struct paintbox_data *pb)
{
	ipu_writel(pb->dev, ~pb->power.stp_active_mask &
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS_DEF,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);
	ipu_writel(pb->dev, ~pb->power.lbp_active_mask &
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS_DEF,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);
}

static void ipu_power_disable_core_idle_clock_gating(struct paintbox_data *pb)
{
	ipu_writel(pb->dev, CLK_GATE_CONTROL_STP_IDLE_GATE_DIS_DEF,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);
	ipu_writel(pb->dev, CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS_DEF,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);
}

void ipu_power_enable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	if (WARN_ON(--pb->power.bif_mmu_clock_idle_disable_ref_count < 0))
		pb->power.bif_mmu_clock_idle_disable_ref_count = 0;

	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(b/115407902):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val &= ~(CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK);
		ipu_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}
}

void ipu_power_disable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(b/115407902):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val |= CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_DMA_IDLE_GATE_DIS_MASK;
		ipu_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}

	pb->power.bif_mmu_clock_idle_disable_ref_count++;
}

/* The caller to this function must hold pb->lock */
void ipu_power_enable_cores(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask, active_core_mask;
	unsigned int active_cores;

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores <= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->stp.num_stps) - 1;
	active_core_mask = (1 << requested_cores) - 1;

	ipu_power_disable_core_idle_clock_gating(pb);
	ipu_power_disable_mmu_bif_idle_clock_gating(pb);

	/* IPU cores need to be enabled in sequence in pairs */
	for (active_cores = pb->power.active_core_count;
			active_cores < requested_cores; active_cores += 2) {
		uint32_t new_core_mask_n = (max_core_mask <<
				(active_cores + 1)) & max_core_mask;
		uint32_t new_core_pairs = ((active_cores + 1) / 2) + 1;

		/* Power on the odd core first.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		udelay(CORE_POWER_RAMP_TIME);

		new_core_mask_n = (max_core_mask << (active_cores + 2)) &
				max_core_mask;

		/* Power on the even core next.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		udelay(CORE_POWER_RAMP_TIME);

		/* We need to run the clock to the core pair that's being
		 * powered on briefly so that all the synchronizers clock
		 * through their data and all the Xs (or random values in the
		 * real HW) clear. Then we need to turn the clock back off so
		 * that we can meet timing on the RAM SD pin -- the setup & hold
		 * on the RAM's SD pin is significantly longer that 1 clock
		 * cycle.
		 */
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn clocks off on all active cores */
		ipu_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn on RAMs for the core */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);
		udelay(RAM_POWER_RAIL_RAMP_TIME);

		/* Restore clocks to all active core pairs. */
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		ipu_frc_ipu_clock_ungate(pb->dev);

		/* Disable core isolation for the requested core */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);
	}

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	ipu_power_enable_mmu_bif_idle_clock_gating(pb);

	/* Re-enable LBP and STP idle clock gating for active cores. */
	ipu_power_enable_core_idle_clock_gating(pb);

	pb->power.active_core_count = requested_cores;
}

/* The caller to this function must hold pb->lock */
void ipu_power_disable_cores(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	if (requested_cores < pb->power.min_active_core_count)
		requested_cores = pb->power.min_active_core_count;
#endif

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores >= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->stp.num_stps) - 1;

	ipu_power_disable_core_idle_clock_gating(pb);
	ipu_power_disable_mmu_bif_idle_clock_gating(pb);

	do {
		int new_active_cores = pb->power.active_core_count - 2;
		uint32_t new_core_mask_n = (max_core_mask << new_active_cores) &
				max_core_mask;
		uint32_t new_core_pairs = (new_active_cores + 1) / 2;

		/* Enable core isolation for the disabled cores */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);

		/* Turn off clocks to all cores during the RAM power transition.
		 */
		ipu_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off RAMs for the disabled core pairs */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);

		/* Need to briefly turn on the clocks to the cores being turned
		 * off to propagate the RAM SD pin change into the RAM, then
		 * need to turn the clocks off again, since the cores are being
		 * turned off.
		 */
		ipu_writel(pb->dev, new_core_pairs + 1,
				IPU_CSR_AON_OFFSET + IPU_CORE_PAIRS_EN);

		/* Turn off clocks to the disabled core pairs. */
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off the core pair */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		/* Make sure the PRE bits for core power are turned off. */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);

		pb->power.active_core_count = new_active_cores;
	} while (pb->power.active_core_count > requested_cores);

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	ipu_power_enable_mmu_bif_idle_clock_gating(pb);

	/* Re-enable LBP and STP idle clock gating for active cores. */
	ipu_power_enable_core_idle_clock_gating(pb);
}

int ipu_power_enable_cores_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret = 0;
	unsigned int highest_core_id;
	unsigned int highest_stp_id = 0;
	unsigned int highest_lbp_id = 0;
	unsigned int max_core;
	struct ipu_power_core_enable_request __user *user_req;
	struct ipu_power_core_enable_request req;
	uint64_t masked_stp_enable, masked_lbp_enable;
	struct paintbox_stp *stp;
	struct paintbox_lbp *lbp;

	user_req = (struct ipu_power_core_enable_request __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	/* The maximum number of cores is equal to the number of STPs. */
	max_core = pb->stp.num_stps;

	masked_stp_enable = session->stp_id_mask & req.stp_mask;
	if (masked_stp_enable & 0x1) {
		dev_err(pb->dev, "%s: invalid stp_id included in mask\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	pb->power.stp_active_mask |= masked_stp_enable >> 1;

	/* Mark every STP indicated in the masks as powered up and get
	 * highest STP ID.
	 */
	while (true) {
		if (masked_stp_enable & 0x1) {
			stp = &pb->stp.stps[ipu_stp_id_to_index(
					highest_stp_id)];
			stp->pm_enabled = true;
		}

		masked_stp_enable = masked_stp_enable >> 1;

		if (masked_stp_enable == 0)
			break;

		highest_stp_id++;
	}

	masked_lbp_enable = session->lbp_id_mask & req.lbp_mask;
	pb->power.lbp_active_mask |= masked_lbp_enable;

	/* Mark every LBP indicated in the masks as powered up and get
	 * highest LBP ID.
	 */
	while (true) {
		if (masked_lbp_enable & 0x1) {
			lbp = &pb->lbp.lbps[highest_lbp_id];
			lbp->pm_enabled = true;
		}

		masked_lbp_enable = masked_lbp_enable >> 1;

		if (masked_lbp_enable == 0)
			break;

		highest_lbp_id++;
	}

	/* LBP with id higher than maximum number of cores is powered by I/O
	 * block. If that id shows up, ignore it by setting the highest lbp id
	 * to the maximum number of core (highest core id managed by core pm).
	 */
	highest_lbp_id = (highest_lbp_id > max_core) ? max_core :
		highest_lbp_id;

	/* If highest_stp_id is 0, means there is no STP to power up.
	 * If highest_lbp_id is 0, either there is no lbp to power up or there
	 * only LBP indicated is LBP 0. Since LBP 0 is powered by I/O, there is
	 * nothing to be done in either case.
	 * If no STP nor LBP needs to be powered up, exit.
	 */
	if (highest_stp_id == 0 && highest_lbp_id == 0)
		goto exit;

	highest_core_id = (highest_stp_id > highest_lbp_id) ? highest_stp_id :
		highest_lbp_id;

	ipu_power_enable_cores(pb, (highest_core_id + 1) & ~1);

exit:
	mutex_unlock(&pb->lock);
	return ret;
}

int ipu_power_disable_cores_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret = 0;
	int stp_id = 0;
	int lbp_id = 0;
	struct ipu_power_core_disable_request __user *user_req;
	struct ipu_power_core_disable_request req;
	uint64_t masked_stp_disable, masked_lbp_disable;
	struct paintbox_stp *stp;
	struct paintbox_lbp *lbp;

	user_req = (struct ipu_power_core_disable_request __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	if (ipu_reset_is_requested(pb)) {
		mutex_unlock(&pb->lock);
		return -ECONNRESET;
	}

	masked_stp_disable = session->stp_id_mask & req.stp_mask;
	if (masked_stp_disable & 0x1) {
		dev_err(pb->dev, "%s: invalid stp_id included in mask\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	pb->power.stp_active_mask &= ~(masked_stp_disable >> 1);

	/* Mark every STP indicated in the masks as powered down. */
	while (masked_stp_disable) {
		if (masked_stp_disable & 0x1) {
			stp = &pb->stp.stps[ipu_stp_id_to_index(stp_id)];
			stp->pm_enabled = false;
		}

		masked_stp_disable = masked_stp_disable >> 1;
		stp_id++;
	}

	masked_lbp_disable = session->lbp_id_mask & req.lbp_mask;

	pb->power.lbp_active_mask &= ~masked_lbp_disable;

	/* Mark every LBP indicated in the masks as powered down. */
	while (masked_lbp_disable) {
		if (masked_lbp_disable & 0x1) {
			lbp = &pb->lbp.lbps[lbp_id];
			lbp->pm_enabled = false;
		}

		masked_lbp_disable = masked_lbp_disable >> 1;
		lbp_id++;
	}

	ipu_power_core_power_walk_down(pb);

exit:
	mutex_unlock(&pb->lock);
	return ret;
}

/* The caller to this function must hold pb->lock */
void ipu_power_core_power_walk_down(struct paintbox_data *pb)
{
	unsigned int requested_cores;

	/* Walk backwards starting from the active core count until we find an
	 * enabled STP or LBP.  This is the new requested core count.
	 */
	for (requested_cores = pb->power.active_core_count;
			requested_cores > 0; requested_cores--) {
		struct paintbox_stp *stp = &pb->stp.stps[ipu_stp_id_to_index(
				requested_cores)];
		struct paintbox_lbp *lbp = &pb->lbp.lbps[requested_cores];

		if (stp->pm_enabled || lbp->pm_enabled)
			break;
	}

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	ipu_power_disable_cores(pb, (requested_cores + 1) & ~1);
}

void ipu_power_init(struct paintbox_data *pb)
{
	pb->power.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;
}
