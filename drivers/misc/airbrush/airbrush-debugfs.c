/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *	Raman Kumar Banka (raman.k2@samsung.com)
 *
 * Airbrush State Manager Control driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>

#include <linux/airbrush-sm-ctrl.h>
#include "airbrush-regs.h"
#include "airbrush-ddr.h"


static int mapped_chip_state_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;
	int ret;

	ret = ab_sm_set_state(sc, val);
	if (ret < 0)
		dev_err(sc->dev, "%s: State change failed, ret %d\n",
				__func__, ret);

	return ret;
}

static int mapped_chip_state_get(void *sc, u64 *val)
{
	*val = ab_sm_get_state((struct ab_state_context *)sc);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_mapped_chip_state, mapped_chip_state_get,
		mapped_chip_state_set, "%llu\n");

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
static int time_stamps_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->ts_enabled = !!val;

	return 0;
}

static int time_stamps_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = (uint64_t)sc->ts_enabled;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_time_stamps, time_stamps_get,
		time_stamps_set, "%llu\n");
#endif

static int ldo5_delay_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->ldo5_delay = val;
	return 0;
}

static int ldo5_delay_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = sc->ldo5_delay;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_ldo5_delay, ldo5_delay_get,
	ldo5_delay_set, "%llu\n");

static int s60_delay_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->s60_delay = val;
	return 0;
}

static int s60_delay_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = sc->s60_delay;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_s60_delay, s60_delay_get,
	s60_delay_set, "%llu\n");

static int ldo4_delay_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->ldo4_delay = val;
	return 0;
}

static int ldo4_delay_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = sc->ldo4_delay;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_ldo4_delay, ldo4_delay_get,
	ldo4_delay_set, "%llu\n");


static int smps2_delay_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->smps2_delay = val;
	return 0;
}

static int smps2_delay_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = sc->smps2_delay;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_smps2_delay, smps2_delay_get,
	smps2_delay_set, "%llu\n");

static int alternate_boot_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->alternate_boot = !!val;

	return 0;
}

static int alternate_boot_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = (uint64_t)sc->alternate_boot;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_alternate_boot, alternate_boot_get,
		alternate_boot_set, "%llu\n");

static int asv_version_set(void *data, u64 val)
{
	struct ab_state_context *sc = data;

	set_asv_version(&sc->asv_info, val);
	return 0;
}

static int asv_version_get(void *data, u64 *val)
{
	struct ab_state_context *sc = data;

	*val = sc->asv_info.asv_version;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_asv_version_override, asv_version_get,
		asv_version_set, "%llu\n");

/*
 * This is added for testing error handling case when pcie link init
 * is skipped.
 */
static int ab_sm_skip_pcie_link_init_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->debug_skip_pcie_link_init = !!val;

	return 0;
}

static int ab_sm_skip_pcie_link_init_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = sc->debug_skip_pcie_link_init;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ab_sm_skip_pcie_link_init_fops,
			 ab_sm_skip_pcie_link_init_get,
			 ab_sm_skip_pcie_link_init_set, "%llu\n");

static int id_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->id;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_id, id_get, NULL, "%llu\n");

static int vrail_status_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->rail_en;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_vrail_status,
				vrail_status_get, NULL, "%llu\n");

static int logic_voltage_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->logic_voltage;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_logic_voltage,
				logic_voltage_get, NULL, "%llu\n");

static int clk_status_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->clk_status;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_clk_status, clk_status_get, NULL, "%llu\n");

static int clk_frequency_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->clk_frequency;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_clk_frequency,
				clk_frequency_get, NULL, "%llu\n");

static int num_powered_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_cores,
				num_powered_cores_get, NULL, "%llu\n");

static int num_computing_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_computing_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_computing_cores,
				num_computing_cores_get, NULL, "%llu\n");

static int num_powered_tiles_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_tiles;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_tiles,
				num_powered_tiles_get, NULL, "%llu\n");

static int data_rate_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->data_rate;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_data_rate, data_rate_get, NULL, "%llu\n");

static const struct file_operations chip_info_fops = {
	.open = chip_info_open,
	.read = seq_read,
};

static int ab_debugfs_ddr_ctrl(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

	switch (val) {
	case 0:
		if (IS_HOST_DDR_INIT())
			sc->dram_ops->init(sc->dram_ops->ctx);
		break;
	case 1:
		sc->dram_ops->suspend(sc->dram_ops->ctx);
		break;
	case 2:
		sc->dram_ops->resume(sc->dram_ops->ctx);
		break;
	case 3:
		sc->dram_ops->sref_enter(sc->dram_ops->ctx);
		break;
	case 4:
		sc->dram_ops->sref_exit(sc->dram_ops->ctx);
		break;
	case 5:
		sc->dram_ops->setup(sc->dram_ops->ctx, sc);
		break;
	case 6:
		sc->dram_ops->rw_test(sc->dram_ops->ctx,
				      DDR_TEST_PCIE_DMA_READ_WRITE(512));
		break;
	case 1866:
		sc->dram_ops->set_freq(sc->dram_ops->ctx, DRAM_CLK_1866MHZ);
		break;
	case 1600:
		sc->dram_ops->set_freq(sc->dram_ops->ctx, DRAM_CLK_1600MHZ);
		break;
	case 1200:
		sc->dram_ops->set_freq(sc->dram_ops->ctx, DRAM_CLK_1200MHZ);
		break;
	case 933:
		sc->dram_ops->set_freq(sc->dram_ops->ctx, DRAM_CLK_933MHZ);
		break;
	case 800:
		sc->dram_ops->set_freq(sc->dram_ops->ctx, DRAM_CLK_800MHZ);
		break;
	default:
		pr_err("ERROR!! Invalid DDR Control\n");
		break;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ab_ddr_ctrl_fops, NULL, ab_debugfs_ddr_ctrl, "%lli\n");

static int ab_debugfs_ddr_test(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	return sc->dram_ops->rw_test(sc->dram_ops->ctx, (unsigned int)val);
}
DEFINE_SIMPLE_ATTRIBUTE(ab_ddr_test_fops, NULL, ab_debugfs_ddr_test, "%lli\n");

static int ab_debugfs_ddr_eye_margin(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->eye_margin(sc->dram_ops->ctx, (unsigned int)val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_eye_margin_fops, NULL,
				ab_debugfs_ddr_eye_margin, "%lli\n");

static int ab_debugfs_ddr_eye_margin_plot(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->eye_margin_plot(sc->dram_ops->ctx);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_eye_margin_plot_fops, NULL,
				ab_debugfs_ddr_eye_margin_plot, "%lli\n");

static int ab_debugfs_clkout_sel(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	ab_clkout_sel(sc, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_clkout_sel_fops, NULL,
				ab_debugfs_clkout_sel, "%lli\n");

static int ab_debugfs_clkout_blksel(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	ab_clkout_blksel(sc, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_clkout_blksel_fops, NULL,
				ab_debugfs_clkout_blksel, "%lli\n");

static int ab_debugfs_clkout_clksel(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	ab_clkout_clksel(sc, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_clkout_clksel_fops, NULL,
				ab_debugfs_clkout_clksel, "%lli\n");

static int ab_debugfs_clkout_enable(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	ab_clkout_enable(sc, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_clkout_enable_fops, NULL,
				ab_debugfs_clkout_enable, "%lli\n");

static int ab_debugfs_clkout_freq(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	ab_clkout_freq(sc, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_clkout_freq_fops, ab_debugfs_clkout_freq,
				NULL, "%lli\n");

static ssize_t ab_debugfs_read_prop_table(struct file *file, char __user *ubuf,
		size_t len, loff_t *ppos)
{
	struct block *blk = file->private_data;
	char *buf;
	int pos = 0;
	int i, ret;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Keep legend in sync with struct block_property. */
	pos += scnprintf(buf + pos, PAGE_SIZE - pos,
			"{id, state_name, substate_name, pmu, rail_en, logic_voltage, clk_status, clk_frequency, num_powered_cores, num_computing_cores, num_powered_tiles, data_rate}\n");

	for (i = 0; i < blk->nr_block_states; i++) {
		pos += scnprintf(buf + pos, PAGE_SIZE - pos,
			"{%d, %s, %s, %d, %d, %d, %d, %llu, %u, %u, %u, %u}\n",
			blk->prop_table[i].id,
			blk->prop_table[i].state_name,
			blk->prop_table[i].substate_name,
			blk->prop_table[i].pmu,
			blk->prop_table[i].rail_en,
			blk->prop_table[i].logic_voltage,
			blk->prop_table[i].clk_status,
			blk->prop_table[i].clk_frequency,
			blk->prop_table[i].num_powered_cores,
			blk->prop_table[i].num_computing_cores,
			blk->prop_table[i].num_powered_tiles,
			blk->prop_table[i].data_rate);
	}

	ret = simple_read_from_buffer(ubuf, len, ppos, buf, pos);
	kfree(buf);

	return ret;
}

static const struct file_operations fops_prop_table = {
	.open = simple_open,
	.read = ab_debugfs_read_prop_table,
};
static int ab_debugfs_ddr_ppc_event0(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->ppc_set_event(sc->dram_ops->ctx, 0, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_ppc_event0_fops, NULL,
				ab_debugfs_ddr_ppc_event0, "%lli\n");

static int ab_debugfs_ddr_ppc_event1(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->ppc_set_event(sc->dram_ops->ctx, 1, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_ppc_event1_fops, NULL,
				ab_debugfs_ddr_ppc_event1, "%lli\n");

static int ab_debugfs_ddr_ppc_event2(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->ppc_set_event(sc->dram_ops->ctx, 2, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_ppc_event2_fops, NULL,
				ab_debugfs_ddr_ppc_event2, "%lli\n");

static int ab_debugfs_ddr_ppc_event3(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->ppc_set_event(sc->dram_ops->ctx, 3, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_ppc_event3_fops, NULL,
				ab_debugfs_ddr_ppc_event3, "%lli\n");

static int ab_debugfs_ddr_ppc_ctrl(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	sc->dram_ops->ppc_ctrl(sc->dram_ops->ctx, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ab_ddr_ppc_ctrl_fops, NULL,
				ab_debugfs_ddr_ppc_ctrl, "%lli\n");

void create_block_debugfs(struct dentry *parent_dir, struct block *blk)
{
	debugfs_create_file("state_id", 0444, parent_dir,
			blk, &fops_id);

	debugfs_create_file("rail_en", 0444, parent_dir,
			blk, &fops_vrail_status);

	debugfs_create_file("logic_voltage", 0444, parent_dir,
			blk, &fops_logic_voltage);

	debugfs_create_file("clk_status", 0444, parent_dir,
			blk, &fops_clk_status);

	debugfs_create_file("clk_frequency", 0444, parent_dir,
			blk, &fops_clk_frequency);

	debugfs_create_file("num_powered_cores", 0444, parent_dir,
			blk, &fops_num_powered_cores);

	debugfs_create_file("num_computing_cores", 0444, parent_dir,
			blk, &fops_num_computing_cores);

	debugfs_create_file("num_powered_tiles", 0444, parent_dir,
			blk, &fops_num_powered_tiles);

	debugfs_create_file("data_rate", 0444, parent_dir,
			blk, &fops_data_rate);

	debugfs_create_file("prop_table", 0444, parent_dir,
			blk, &fops_prop_table);

	return;
}

void ab_sm_create_debugfs(struct ab_state_context *sc)
{
	struct dentry *d_chip, *d_block;

	sc->d_entry = debugfs_create_dir("airbrush", NULL);


	debugfs_create_file("ab_ddr_ctrl", 0200, sc->d_entry, sc,
				&ab_ddr_ctrl_fops);

	debugfs_create_file("ab_ddr_test", 0200, sc->d_entry, sc,
				&ab_ddr_test_fops);

	debugfs_create_file("ddr_eye_margin", 0200, sc->d_entry, sc,
				&ab_ddr_eye_margin_fops);

	debugfs_create_file("ddr_eye_margin_plot", 0200, sc->d_entry, sc,
				&ab_ddr_eye_margin_plot_fops);

	debugfs_create_file("clkout_sel", 0200, sc->d_entry, sc,
				&ab_clkout_sel_fops);

	debugfs_create_file("clkout_blksel", 0200, sc->d_entry, sc,
				&ab_clkout_blksel_fops);

	debugfs_create_file("clkout_clksel", 0200, sc->d_entry, sc,
				&ab_clkout_clksel_fops);

	debugfs_create_file("clkout_enable", 0200, sc->d_entry, sc,
				&ab_clkout_enable_fops);

	debugfs_create_file("clkout_freq", 0400, sc->d_entry, sc,
				&ab_clkout_freq_fops);

	debugfs_create_file("ddr_ppc_event0", 0200, sc->d_entry, sc,
				&ab_ddr_ppc_event0_fops);

	debugfs_create_file("ddr_ppc_event1", 0200, sc->d_entry, sc,
				&ab_ddr_ppc_event1_fops);

	debugfs_create_file("ddr_ppc_event2", 0200, sc->d_entry, sc,
				&ab_ddr_ppc_event2_fops);

	debugfs_create_file("ddr_ppc_event3", 0200, sc->d_entry, sc,
				&ab_ddr_ppc_event3_fops);

	debugfs_create_file("ddr_ppc_ctrl", 0200, sc->d_entry, sc,
				&ab_ddr_ppc_ctrl_fops);

	debugfs_create_file("chip_info", 0444, sc->d_entry, NULL,
				&chip_info_fops);

	d_chip = debugfs_create_dir("airbrush_sm", sc->d_entry);

	debugfs_create_file("mapped_chip_state", 0664, d_chip, sc,
				&fops_mapped_chip_state);

	debugfs_create_file("smps2_delay", 0664, d_chip, sc,
			&fops_smps2_delay);

	debugfs_create_file("ldo4_delay", 0664, d_chip, sc,
			&fops_ldo4_delay);

	debugfs_create_file("ldo5_delay", 0664, d_chip, sc,
			&fops_ldo5_delay);

	debugfs_create_file("s60_delay", 0664, d_chip, sc,
			&fops_s60_delay);

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
	debugfs_create_file("time_stamps", 0664, d_chip, sc,
			&fops_time_stamps);
#endif

	debugfs_create_file("asv_version", 0660, d_chip, sc,
				&fops_asv_version_override);

	debugfs_create_file("skip_pcie_link_init", 0660, d_chip, sc,
				&ab_sm_skip_pcie_link_init_fops);

	d_block = debugfs_create_dir("ipu", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[BLK_IPU]));

	d_block = debugfs_create_dir("tpu", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[BLK_TPU]));

	d_block = debugfs_create_dir("dram", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[DRAM]));

	d_block = debugfs_create_dir("mif", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[BLK_MIF]));

	d_block = debugfs_create_dir("fsys", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[BLK_FSYS]));

	d_block = debugfs_create_dir("aon", d_chip);
	create_block_debugfs(d_block, &(sc->blocks[BLK_AON]));

	return;
}

void ab_sm_remove_debugfs(struct ab_state_context *sc)
{
	debugfs_remove_recursive(sc->d_entry);
}
