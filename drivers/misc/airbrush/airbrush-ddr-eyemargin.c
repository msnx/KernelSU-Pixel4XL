/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Eye Margin tool.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

static void ddrphy_set_read_offset(struct ab_ddr_context *ddr_ctx,
		int offset_phy)
{
	uint32_t ctrl_offsetr;

	if (offset_phy < 0)
		ctrl_offsetr = 0x100 | ((offset_phy * -1) & 0xff);
	else
		ctrl_offsetr = offset_phy;

	ddr_reg_clr_set(ddr_ctx, DPHY_OFFSETR_CON0,
			CTRL_OFFSETR0_MSK | CTRL_OFFSETR1_MSK,
			CTRL_OFFSETR0(ctrl_offsetr) |
			CTRL_OFFSETR1(ctrl_offsetr));

	ddr_reg_clr_set(ddr_ctx, DPHY2_OFFSETR_CON0,
			CTRL_OFFSETR0_MSK | CTRL_OFFSETR1_MSK,
			CTRL_OFFSETR0(ctrl_offsetr) |
			CTRL_OFFSETR1(ctrl_offsetr));

	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

static void ddrphy_set_write_offset(struct ab_ddr_context *ddr_ctx,
		int offset_phy)
{
	uint32_t ctrl_offsetw;

	if (offset_phy < 0)
		ctrl_offsetw = 0x100 | ((offset_phy * -1) & 0xff);
	else
		ctrl_offsetw = offset_phy;

	ddr_reg_clr_set(ddr_ctx, DPHY_OFFSETW_CON0,
			CTRL_OFFSETW0_MSK | CTRL_OFFSETW1_MSK,
			CTRL_OFFSETW0(ctrl_offsetw) |
			CTRL_OFFSETW1(ctrl_offsetw));

	ddr_reg_clr_set(ddr_ctx, DPHY2_OFFSETW_CON0,
			CTRL_OFFSETW0_MSK | CTRL_OFFSETW1_MSK,
			CTRL_OFFSETW0(ctrl_offsetw) |
			CTRL_OFFSETW1(ctrl_offsetw));

	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

void ddr_eye_print_termination_info(struct ab_ddr_context *ddr_ctx)
{
	unsigned int zq_con0, zq_con3, zq_con6;
	unsigned int dphy_mdll_con0, dphy_mdll_con1;
	unsigned int dphy2_mdll_con0, dphy2_mdll_con1;

	zq_con0 = ddr_reg_rd(ddr_ctx, DPHY_ZQ_CON0);
	zq_con3 = ddr_reg_rd(ddr_ctx, DPHY_ZQ_CON3);
	zq_con6 = ddr_reg_rd(ddr_ctx, DPHY_ZQ_CON6);
	pr_info("----------------------------------------------------------\n");
	pr_info("DataSlice_0 = DQ[7:0], DataSlice_1 = DQ[15:8]\n");
	pr_info("ControlSlice = CA[5:0]\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("ODT Information for DPHY\n");
	pr_info("	3'b100 : 60 Ohm Far end VSSQ termination\n");
	pr_info("	3'b010 : 120 Ohm Far end VSSQ termination\n");
	pr_info("	3'b001 : 240 Ohm Far end VSSQ termination\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("ODT Data Slice 0: ZQ_CON6[5:3](zq_ds0_term) 0x%x\n",
		(zq_con6 >> 3) & 0x7);
	pr_info("ODT Data Slice 0: ZQ_CON6[13:11](zq_ds1_term) 0x%x\n",
		(zq_con6 >> 11) & 0x7);
	pr_info("----------------------------------------------------------\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("Driver Strength Info for DPHY (Impedance output driver)\n");
	pr_info("	3'b100 : 48 Ohm, 3'b101 : 40 Ohm\n");
	pr_info("	3'b110 : 34 Ohm, 3'b111 : 30 Ohm\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("Pull-Down Data Slice 0: ZQ_CON3[2:0](zq_ds0_pdds) 0x%x\n",
		(zq_con3 >> 0) & 0x7);
	pr_info("Pull-Down Data Slice 1: ZQ_CON3[10:8](zq_ds1_pdds) 0x%x\n",
		(zq_con3 >> 8) & 0x7);
	pr_info("Pull-Down Control Slice: ZQ_CON0[30:28](zq_mode_pdds) 0x%x\n",
		(zq_con0 >> 28) & 0x7);
	pr_info("Pull-Up Data Slice 0: ZQ_CON3[5:3](zq_ds0_dds) 0x%x\n",
		(zq_con3 >> 3) & 0x7);
	pr_info("Pull-Up Data Slice 1: ZQ_CON3[13:11](zq_ds1_dds) 0x%x\n",
		(zq_con3 >> 11) & 0x7);
	pr_info("Pull-Up Control Slice: ZQ_CON0[26:24](zq_mode_dds) 0x%x\n",
		(zq_con0 >> 24) & 0x7);
	pr_info("----------------------------------------------------------\n");

	zq_con0 = ddr_reg_rd(ddr_ctx, DPHY2_ZQ_CON0);
	zq_con3 = ddr_reg_rd(ddr_ctx, DPHY2_ZQ_CON3);
	zq_con6 = ddr_reg_rd(ddr_ctx, DPHY2_ZQ_CON6);
	pr_info("----------------------------------------------------------\n");
	pr_info("ODT Information for DPHY2\n");
	pr_info("	3'b100 : 60 Ohm Far end VSSQ termination\n");
	pr_info("	3'b010 : 120 Ohm Far end VSSQ termination\n");
	pr_info("	3'b001 : 240 Ohm Far end VSSQ termination\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("ODT Data Slice 0: ZQ_CON6[5:3](zq_ds0_term) 0x%x\n",
		(zq_con6 >> 3) & 0x7);
	pr_info("ODT Data Slice 0: ZQ_CON6[13:11](zq_ds1_term) 0x%x\n",
		(zq_con6 >> 11) & 0x7);
	pr_info("----------------------------------------------------------\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("Driver Strength Info for DPHY2 (Impedance output driver)\n");
	pr_info("	3'b100 : 48 Ohm, 3'b101 : 40 Ohm\n");
	pr_info("	3'b110 : 34 Ohm, 3'b111 : 30 Ohm\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("Pull-Down Data Slice 0: ZQ_CON3[2:0](zq_ds0_pdds) 0x%x\n",
		(zq_con3 >> 0) & 0x7);
	pr_info("Pull-Down Data Slice 1: ZQ_CON3[10:8](zq_ds1_pdds) 0x%x\n",
		(zq_con3 >> 8) & 0x7);
	pr_info("Pull-Down Control Slice: ZQ_CON0[30:28](zq_mode_pdds) 0x%x\n",
		(zq_con0 >> 28) & 0x7);
	pr_info("Pull-Up Data Slice 0: ZQ_CON3[5:3](zq_ds0_dds) 0x%x\n",
		(zq_con3 >> 3) & 0x7);
	pr_info("Pull-Up Data Slice 1: ZQ_CON3[13:11](zq_ds1_dds) 0x%x\n",
		(zq_con3 >> 11) & 0x7);
	pr_info("Pull-Up Control Slice: ZQ_CON0[26:24](zq_mode_dds) 0x%x\n",
		(zq_con0 >> 24) & 0x7);
	pr_info("----------------------------------------------------------\n");

	pr_info("----------------------------------------------------------\n");
	pr_info("DRAM ODT[MR11] and Drive-Strength[MR3] details:\n");
	pr_info("----------------------------------------------------------\n");
	pr_info("MR11: 0x%x, MR3: 0x%x\n", 0x24, 0xf1);
	pr_info("----------------------------------------------------------\n");

	dphy_mdll_con0  = ddr_reg_rd(ddr_ctx, DPHY_MDLL_CON0);
	dphy_mdll_con1  = ddr_reg_rd(ddr_ctx, DPHY_MDLL_CON1);
	dphy2_mdll_con0 = ddr_reg_rd(ddr_ctx, DPHY2_MDLL_CON0);
	dphy2_mdll_con1 = ddr_reg_rd(ddr_ctx, DPHY2_MDLL_CON1);

	pr_info("----------------------------------------------------------\n");
	pr_info("DPHY_MDLL_CONx: [0x%x, 0x%x]\n",
		dphy_mdll_con0, dphy_mdll_con1);
	pr_info("ctrl_lock_value[9:0]: 0x%x, ctrl_locked: [%d]\n",
		(dphy_mdll_con1 >> 8) & 0x3ff,
		(dphy_mdll_con1 >> 18) & 0x1);

	pr_info("DPHY2_MDLL_CONx: [0x%x, 0x%x]\n",
		dphy2_mdll_con0, dphy2_mdll_con1);
	pr_info("ctrl_lock_value[9:0]: 0x%x, ctrl_locked: [%d]\n",
		(dphy2_mdll_con1 >> 8) & 0x3ff,
		(dphy2_mdll_con1 >> 18) & 0x1);
	pr_info("----------------------------------------------------------\n");
}

void ab_ddr_eye_margin_plot_read(struct ab_ddr_context *ddr_ctx)
{
	char (*read_eye)[MAX_RW_OFFSETS];
	ktime_t (*eye_time)[MAX_RW_OFFSETS];
	int vrefIdx, offsetIdx, maxOffset;

	read_eye = &ddr_ctx->eye_data->read_eye[0];
	eye_time = &ddr_ctx->eye_data->read_eye_time[0];
	maxOffset = ddr_ctx->eye_data->num_samples_read;

	/* plot the pass fail information for each vref & delay offset */
	for (vrefIdx = VREF_FROM; vrefIdx < PHY_VREF_LEVELS;
				  vrefIdx += VREF_STEP)
		pr_info("VREF_0x%02x : %s\n", ddr_get_phy_vref(vrefIdx),
					      read_eye[vrefIdx]);

	/* plot the time tracking information for read & compare */
	for (vrefIdx = VREF_FROM; vrefIdx < PHY_VREF_LEVELS;
				  vrefIdx += VREF_STEP) {
		pr_cont("VREF_0x%02x : ", ddr_get_phy_vref(vrefIdx));

		for (offsetIdx = 0; offsetIdx < maxOffset; offsetIdx++)
			pr_cont("%10llu ",
				ktime_to_us(eye_time[vrefIdx][offsetIdx]));

		pr_cont("\n");
	}
}

void ab_ddr_eye_margin_plot_write(struct ab_ddr_context *ddr_ctx)
{
	char (*write_eye)[MAX_RW_OFFSETS];
	ktime_t (*eye_time)[MAX_RW_OFFSETS];
	int vrefIdx, offsetIdx, maxOffset;

	write_eye = &ddr_ctx->eye_data->write_eye[0];
	eye_time = &ddr_ctx->eye_data->write_eye_time[0];
	maxOffset = ddr_ctx->eye_data->num_samples_write;

	/* plot the pass fail information for each vref & delay offset */
	for (vrefIdx = VREF_FROM; vrefIdx < DRAM_VREF_LEVELS;
				  vrefIdx += VREF_STEP)
		pr_info("VREF_0x%02x : %s\n", ddr_get_dram_vref(vrefIdx),
					      write_eye[vrefIdx]);

	/* plot the time tracking information for read & compare */
	for (vrefIdx = VREF_FROM; vrefIdx < DRAM_VREF_LEVELS;
				  vrefIdx += VREF_STEP) {
		pr_cont("VREF_0x%02x : ", ddr_get_dram_vref(vrefIdx));

		for (offsetIdx = 0; offsetIdx < maxOffset; offsetIdx++)
			pr_cont("%10llu ",
				ktime_to_us(eye_time[vrefIdx][offsetIdx]));

		pr_cont("\n");
	}
}

/* Caller must hold ddr_ctx->ddr_lock */
static void ddrphy_margin_eye_read(struct ab_ddr_context *ddr_ctx,
				    int samples, uint32_t eye_data)
{
	int vrefIdx;
	int maxOffset;
	int offsetIdx, result_idx;
	char (*read_eye)[MAX_RW_OFFSETS];

	read_eye = &ddr_ctx->eye_data->read_eye[0];
	maxOffset = samples;

	for (vrefIdx = VREF_FROM; vrefIdx < PHY_VREF_LEVELS;
						vrefIdx += VREF_STEP) {
		ddrphy_set_read_vref(ddr_ctx, ddr_get_phy_vref(vrefIdx),
				     ddr_get_phy_vref(vrefIdx), VREF_BYTE_ALL);
		pr_info("SET READ_VREF: 0x%02x\n", ddr_get_phy_vref(vrefIdx));

		result_idx = 0;
		for (offsetIdx = (maxOffset * -1); offsetIdx <= maxOffset;
							offsetIdx++) {
			ddrphy_set_read_offset(ddr_ctx, offsetIdx);

			if (!__ab_ddr_read_write_test(ddr_ctx, eye_data))
				read_eye[vrefIdx][result_idx] = 'o';
			else
				read_eye[vrefIdx][result_idx] = '.';

			ddr_ctx->eye_data->read_eye_time[vrefIdx][result_idx] =
				ktime_sub(ddr_ctx->et_read, ddr_ctx->st_read);

			if (offsetIdx == 0)
				read_eye[vrefIdx][result_idx] = '|';

			result_idx++;
		}

		ddr_ctx->eye_data->num_samples_read = result_idx;
		read_eye[vrefIdx][result_idx] = '\0';
	}

	ab_ddr_eye_margin_plot_read(ddr_ctx);
}

/* Caller must hold ddr_ctx->ddr_lock */
static void ddrphy_margin_eye_write(struct ab_ddr_context *ddr_ctx,
				     int samples, uint32_t eye_data)
{
	int vrefIdx;
	int maxOffset;
	int offsetIdx, result_idx;
	char (*write_eye)[MAX_RW_OFFSETS];

	write_eye = &ddr_ctx->eye_data->write_eye[0];
	maxOffset = samples;

	for (vrefIdx = VREF_FROM; vrefIdx < DRAM_VREF_LEVELS;
				  vrefIdx += VREF_STEP) {
		ddrphy_set_write_vref(ddr_ctx, ddr_get_dram_vref(vrefIdx),
				      VREF_BYTE_ALL);
		pr_info("SET WRITE_VREF: 0x%02x\n", ddr_get_dram_vref(vrefIdx));

		result_idx = 0;
		for (offsetIdx = (maxOffset * -1); offsetIdx <= maxOffset;
							offsetIdx++) {
			ddrphy_set_write_offset(ddr_ctx, offsetIdx);

			if (!__ab_ddr_read_write_test(ddr_ctx, eye_data))
				write_eye[vrefIdx][result_idx] = 'o';
			else
				write_eye[vrefIdx][result_idx] = '.';

			ddr_ctx->eye_data->write_eye_time[vrefIdx][result_idx] =
				ktime_sub(ddr_ctx->et_read, ddr_ctx->st_read);

			if (offsetIdx == 0)
				write_eye[vrefIdx][result_idx] = '|';

			result_idx++;
		}

		ddr_ctx->eye_data->num_samples_write = result_idx;
		write_eye[vrefIdx][result_idx] = '\0';
	}

	ab_ddr_eye_margin_plot_write(ddr_ctx);
}

int ab_ddr_eye_margin_plot(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr eye margin: error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	if (!ddr_ctx->eye_data) {
		pr_err("ddr eye margin: initial memory alloc failed\n");
		return -ENOMEM;
	}

	if (!(ddr_ctx->eye_data->num_samples_write &&
	      ddr_ctx->eye_data->num_samples_read)) {
		pr_err("ddr eye margin: error!! measure the eye before plot\n");
		return DDR_FAIL;
	}

	ddr_eye_print_termination_info(ddr_ctx);
	ab_ddr_eye_margin_plot_read(ddr_ctx);
	ab_ddr_eye_margin_plot_write(ddr_ctx);

	return 0;
}

int ab_ddr_eye_margin(void *ctx, unsigned int data)
{
	uint32_t read_vref_phy0, read_vref_phy1;
	uint32_t write_vref;
	uint32_t num_samples;
	uint32_t eye_data;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("[ddr eye margin] Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Allow the eye margin test only when ddr state is DDR_ON */
	if (ddr_ctx->ddr_state != DDR_ON) {
		pr_err("ddr_eye_margin: Invalid ddr state: %d\n",
			ddr_ctx->ddr_state);
		return DDR_FAIL;
	}

	if (!ddr_ctx->eye_data) {
		pr_err("ddr eye margin: initial memory alloc failed\n");
		return -ENOMEM;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	/* Read the MR14 register to get the VREF(DQ) information.
	 * Read the information while DDR is in self-refresh mode.
	 */
	write_vref = ddr_read_mr_reg(ddr_ctx, 14) & 0x3f;

	ddr_eye_print_termination_info(ddr_ctx);

	/* Disable the PHY control logic clock gating for s/w margin
	 * offset settings to work
	 */
	ddr_reg_clr(ddr_ctx, DPHY_LP_CON0, PCL_PD);
	ddr_reg_clr(ddr_ctx, DPHY2_LP_CON0, PCL_PD);

	read_vref_phy0 = ddr_reg_rd(ddr_ctx, DPHY_ZQ_CON9) & 0x3f;
	read_vref_phy1 = ddr_reg_rd(ddr_ctx, DPHY2_ZQ_CON9) & 0x3f;

	/* "data" provides the information about the type of test to be run
	 * (memtester/pcie_dma) for checking ddr data integrity
	 */
	eye_data = data;
	num_samples = DDR_TEST_EYE_MARGIN_SAMPLES(eye_data);
	if (num_samples < 50)
		num_samples = 50;

	pr_info("current read vref: 0x%02x, write vref: 0x%02x\n",
		(read_vref_phy0 + read_vref_phy1) / 2, write_vref);
	pr_info("RD_EYE: sweeping vref [0x%02x -> 0x%02x]\n",
		ddr_get_phy_vref(VREF_FROM),
		ddr_get_phy_vref(PHY_VREF_LEVELS - 1));

	/* sweep read vref to get the eye margin on READ */
	ddrphy_margin_eye_read(ddr_ctx, num_samples, eye_data);

	/* Restore the working Read Vref & Offset */
	ddrphy_set_read_vref(ddr_ctx, read_vref_phy0,
			read_vref_phy1, VREF_BYTE_ALL);
	ddrphy_set_read_offset(ddr_ctx, 0);

	pr_info("WR_EYE: sweeping vref 0x%02x -> 0x%02x\n",
		ddr_get_dram_vref(VREF_FROM),
		ddr_get_dram_vref(DRAM_VREF_LEVELS - 1));

	/* sweep write vref to get the eye margin on WRITE */
	ddrphy_margin_eye_write(ddr_ctx, num_samples, eye_data);

	/* Restore the working write Vref & Offset */
	ddrphy_set_write_vref(ddr_ctx, write_vref, VREF_BYTE_ALL);
	ddrphy_set_write_offset(ddr_ctx, 0);

	/* Enable the PHY control logic clock gating after eyemargin test */
	ddr_reg_set(ddr_ctx, DPHY_LP_CON0, PCL_PD);
	ddr_reg_set(ddr_ctx, DPHY2_LP_CON0, PCL_PD);

	mutex_unlock(&ddr_ctx->ddr_lock);
	return DDR_SUCCESS;
}
