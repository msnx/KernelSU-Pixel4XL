/*
 * JQS management debug support for the Paintbox programmable IPU
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/types.h>
#include <linux/pm_runtime.h>

#include "ipu-core-internal.h"
#include "ipu-core-jqs.h"
#include "ipu-core-jqs-debug.h"
#include "ipu-debug.h"
#include "ipu-regs.h"

static const char *ipu_jqs_reg_names[IO_JQS_NUM_REGS] = {
	REG_NAME_ENTRY64(SYS_JQS_DBL),
	REG_NAME_ENTRY64(JQS_SYS_DBL),
	REG_NAME_ENTRY64(SYS_JQS_GPR_0),
	REG_NAME_ENTRY64(SYS_JQS_GPR_1),
	REG_NAME_ENTRY64(SYS_JQS_GPR_2),
	REG_NAME_ENTRY64(SYS_JQS_GPR_3),
	REG_NAME_ENTRY64(SYS_JQS_GPR_4),
	REG_NAME_ENTRY64(SYS_JQS_GPR_5),
	REG_NAME_ENTRY64(SYS_JQS_GPR_6),
	REG_NAME_ENTRY64(SYS_JQS_GPR_7),
	REG_NAME_ENTRY64(JQS_SYS_GPR_0),
	REG_NAME_ENTRY64(JQS_SYS_GPR_1),
	REG_NAME_ENTRY64(JQS_SYS_GPR_2),
	REG_NAME_ENTRY64(JQS_SYS_GPR_3),
	REG_NAME_ENTRY64(JQS_SYS_GPR_4),
	REG_NAME_ENTRY64(JQS_SYS_GPR_5),
	REG_NAME_ENTRY64(JQS_SYS_GPR_6),
	REG_NAME_ENTRY64(JQS_SYS_GPR_7),
	REG_NAME_ENTRY64(SYS_JQS_IRQ_INTENT),
	REG_NAME_ENTRY64(JQS_SYS_STAT)
};

static int ipu_jqs_debug_read_registers(struct seq_file *s, void *unused)
{
	struct paintbox_bus *bus = s->private;
	unsigned int i;

	mutex_lock(&bus->jqs.lock);

	if (!ipu_core_jqs_is_ready(bus)) {
		dev_warn(bus->parent_dev,
				"%s: unable to enable JQS, link not ready\n",
				__func__);
		mutex_unlock(&bus->jqs.lock);
		return -ENETDOWN;
	}

	for (i = 0; i < IO_JQS_NUM_REGS; i++) {
		unsigned int offset;

		if (ipu_jqs_reg_names[i] == NULL) {
			continue;
		} else if (i == REG_INDEX64(JQS_SYS_DBL)) {
			seq_printf(s, "0x%04lx: %-*s0x%016llx\n", offset,
					REG_VALUE_COL_WIDTH,
					ipu_jqs_reg_names[i],
					bus->jqs.shadow_reg_jqs_sys_dbl);
		} else {
			offset = i * IPU_REG_WIDTH_BYTES + IPU_CSR_JQS_OFFSET;

			seq_printf(s, "0x%04lx: %-*s0x%016llx\n", offset,
					REG_VALUE_COL_WIDTH,
					ipu_jqs_reg_names[i],
					ipu_core_readq(bus, offset));
		}
	}

	mutex_unlock(&bus->jqs.lock);

	return 0;
}

static int ipu_jqs_debug_register_set(void *data, u64 val)
{
	struct ipu_bus_debug_register *reg = data;
	struct paintbox_bus *bus = reg->bus;

	mutex_lock(&bus->jqs.lock);

	if (!ipu_core_jqs_is_ready(bus)) {
		dev_warn(bus->parent_dev,
				"%s: unable to enable JQS, link not ready\n",
				__func__);
		mutex_unlock(&bus->jqs.lock);
		return -ENETDOWN;
	}

	ipu_core_writeq(bus, val, reg->offset);

	mutex_unlock(&bus->jqs.lock);

	return 0;
}

static int ipu_jqs_debug_register_get(void *data, u64 *val)
{
	struct ipu_bus_debug_register *reg = data;
	struct paintbox_bus *bus = reg->bus;

	mutex_lock(&bus->jqs.lock);

	if (!ipu_core_jqs_is_ready(bus)) {
		dev_warn(bus->parent_dev,
				"%s: unable to enable JQS, link not ready\n",
				__func__);
		mutex_unlock(&bus->jqs.lock);
		return -ENETDOWN;
	}

	*val = ipu_core_readq(bus, reg->offset);

	mutex_unlock(&bus->jqs.lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_jqs_debug_register_fops, ipu_jqs_debug_register_get,
		ipu_jqs_debug_register_set, "%llx\n");

static void ipu_jqs_debug_create_register_file(struct paintbox_bus *bus,
		struct ipu_bus_debug_register *reg, const char *name,
		unsigned int offset)
{
	reg->offset = IPU_CSR_JQS_OFFSET + offset;
	reg->bus = bus;

	reg->dentry = debugfs_create_file(name, 0640, bus->jqs.debug_dir, reg,
			&ipu_jqs_debug_register_fops);
	if (WARN_ON(IS_ERR(reg->dentry)))
		reg->dentry = NULL;
}

static int ipu_core_jqs_log_level_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	enum jqs_log_level requested_level = (enum jqs_log_level)val;
	int ret = 0;

	if (requested_level > JQS_LOG_LEVEL_INFO)
		return -ERANGE;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.log_level = requested_level;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_log_level_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = (uint64_t)bus->jqs.log_level;

	return 0;
}

static int ipu_core_jqs_trigger_level_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	enum jqs_log_level requested_level = (enum jqs_log_level)val;
	int ret = 0;

	if (requested_level > JQS_LOG_LEVEL_INFO)
		return -ERANGE;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.log_trigger_level = requested_level;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_trigger_level_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = (uint64_t)bus->jqs.log_trigger_level;

	return 0;
}

static int ipu_core_jqs_kernel_log_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	if (val)
		bus->jqs.log_sink_mask |= JQS_LOG_SINK_MESSAGE;
	else
		bus->jqs.log_sink_mask &= ~JQS_LOG_SINK_MESSAGE;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_kernel_log_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = !!(bus->jqs.log_sink_mask & JQS_LOG_SINK_MESSAGE);

	return 0;
}

static int ipu_core_jqs_uart_log_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	if (val)
		bus->jqs.log_sink_mask |= JQS_LOG_SINK_UART;
	else
		bus->jqs.log_sink_mask &= ~JQS_LOG_SINK_UART;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_uart_log_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = !!(bus->jqs.log_sink_mask & JQS_LOG_SINK_UART);

	return 0;
}

static int ipu_core_jqs_uart_baud_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	mutex_lock(&bus->jqs.lock);

	bus->jqs.uart_baud = (uint32_t)val;

	/* If the JQS is ready then commit the new log configuration */
	if (ipu_core_jqs_is_ready(bus))
		ret = ipu_core_jqs_send_set_log_info(bus);

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_uart_baud_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = bus->jqs.uart_baud;

	return 0;
}

static int ipu_core_jqs_fw_state_set(void *data, u64 val)
{
	struct paintbox_bus *bus = data;
	int ret = 0;

	if (val > JQS_FW_STATUS_SUSPENDED) {
		dev_err(bus->parent_dev, "%s: invalid value, val = %llu",
				__func__, val);
		return -EINVAL;
	}

	mutex_lock(&bus->jqs.lock);

	/* Store the value as the minimum state */
	bus->jqs.status_min = val;

	/* Check if a higher readiness state was requested. */
	while (val > bus->jqs.status && !ret) {
		switch (bus->jqs.status) {
		case JQS_FW_STATUS_INIT:
			ret = ipu_core_jqs_load_firmware(bus);
			break;
		case JQS_FW_STATUS_REQUESTED:
			ret = ipu_core_jqs_stage_firmware(bus);
			break;
		case JQS_FW_STATUS_STAGED:
			ret = ipu_core_jqs_enable_firmware(bus);
			break;
		case JQS_FW_STATUS_RUNNING:
			ipu_core_jqs_suspend_firmware(bus);
			break;
		case JQS_FW_STATUS_SUSPENDED:
			break;
		}
	}

	/* Check if a lower readiness state was requested. */
	while (val < bus->jqs.status) {
		switch (bus->jqs.status) {
		case JQS_FW_STATUS_SUSPENDED:
			ipu_core_jqs_enable_firmware(bus);
			break;
		case JQS_FW_STATUS_RUNNING:
			ipu_core_jqs_disable_firmware_requested(bus);
			break;
		case JQS_FW_STATUS_STAGED:
			ipu_core_jqs_unstage_firmware(bus);
			break;
		case JQS_FW_STATUS_REQUESTED:
			ipu_core_jqs_unload_firmware(bus);
			break;
		case JQS_FW_STATUS_INIT:
			break;
		}
	}

	mutex_unlock(&bus->jqs.lock);

	return ret;
}

static int ipu_core_jqs_fw_state_get(void *data, u64 *val)
{
	struct paintbox_bus *bus = data;

	*val = bus->jqs.status;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_log_level_fops, ipu_core_jqs_log_level_get,
			ipu_core_jqs_log_level_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_log_trigger_level_fops,
			ipu_core_jqs_trigger_level_get,
			ipu_core_jqs_trigger_level_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_kernel_log_fops,
			ipu_core_jqs_kernel_log_get,
			ipu_core_jqs_kernel_log_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_uart_log_fops,
			ipu_core_jqs_uart_log_get,
			ipu_core_jqs_uart_log_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_uart_baud_fops,
			ipu_core_jqs_uart_baud_get,
			ipu_core_jqs_uart_baud_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu_core_jqs_fw_state_fops, ipu_core_jqs_fw_state_get,
			ipu_core_jqs_fw_state_set, "%llu\n");

static int ipu_debug_jqs_reg_dump_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, ipu_jqs_debug_read_registers,
			inode->i_private,
			IO_JQS_NUM_REGS * REG_DEBUG_BUFFER_SIZE);
}

static const struct file_operations ipu_debug_jqs_reg_dump_fops = {
	.open = ipu_debug_jqs_reg_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ipu_core_jqs_debug_init(struct paintbox_bus *bus)
{
	unsigned int i;

	bus->jqs.debug_dir = debugfs_create_dir("jqs", bus->debug_root);
	if (IS_ERR(bus->jqs.debug_dir)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.debug_dir));
		return;
	}

	bus->jqs.fw_state_dentry = debugfs_create_file("fw_state", 0640,
			bus->jqs.debug_dir, bus, &ipu_core_jqs_fw_state_fops);
	if (IS_ERR(bus->jqs.fw_state_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.fw_state_dentry));
		return;
	}

	bus->jqs.log_level_dentry = debugfs_create_file("log_level", 0640,
			bus->jqs.debug_dir, bus, &ipu_core_jqs_log_level_fops);
	if (IS_ERR(bus->jqs.log_level_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.log_level_dentry));
		return;
	}

	bus->jqs.trigger_level_dentry = debugfs_create_file("log_trigger_level",
			0640, bus->jqs.debug_dir, bus,
			&ipu_core_jqs_log_trigger_level_fops);
	if (IS_ERR(bus->jqs.trigger_level_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.trigger_level_dentry));
		return;
	}

	bus->jqs.kernel_log_dentry = debugfs_create_file("kernel_log_en", 0640,
			bus->jqs.debug_dir, bus,
			&ipu_core_jqs_kernel_log_fops);
	if (IS_ERR(bus->jqs.kernel_log_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.kernel_log_dentry));
		return;
	}

	bus->jqs.uart_log_dentry = debugfs_create_file("uart_log_en", 0640,
			bus->jqs.debug_dir, bus, &ipu_core_jqs_uart_log_fops);
	if (IS_ERR(bus->jqs.uart_log_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.uart_log_dentry));
	}

	bus->jqs.uart_baud_dentry = debugfs_create_file("uart_baud", 0640,
			bus->jqs.debug_dir, bus, &ipu_core_jqs_uart_baud_fops);
	if (IS_ERR(bus->jqs.uart_baud_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->jqs.uart_baud_dentry));
	}

	bus->jqs.reg_dump = debugfs_create_file("regs", 0640,
			bus->jqs.debug_dir, bus, &ipu_debug_jqs_reg_dump_fops);
	if (WARN_ON(IS_ERR_OR_NULL(bus->jqs.reg_dump)))
		return;


	for (i = 0; i < IO_JQS_NUM_REGS; i++) {
		if (!ipu_jqs_reg_names[i])
			continue;

		ipu_jqs_debug_create_register_file(bus,
				&bus->jqs.debug_registers[i],
				ipu_jqs_reg_names[i], i * IPU_REG_WIDTH_BYTES);
	}
}

void ipu_core_jqs_debug_remove(struct paintbox_bus *bus)
{
	debugfs_remove_recursive(bus->jqs.debug_dir);
}


