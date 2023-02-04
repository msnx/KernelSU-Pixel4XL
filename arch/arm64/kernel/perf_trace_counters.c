// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2013-2014, 2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/tracepoint.h>
#include <linux/perf_event.h>
#include <linux/mutex.h>
#include <trace/events/sched.h>
#include <linux/slab.h>
#define CREATE_TRACE_POINTS
#include "perf_trace_counters.h"

static unsigned int tp_pid_state;

DEFINE_PER_CPU(u32, cntenset_val);
DEFINE_PER_CPU(u32[NUM_EVENTS], previous_cnts);
DEFINE_PER_CPU(u32, old_pid);
DEFINE_PER_CPU(u32, hotplug_flag);
DEFINE_PER_CPU(struct perf_event **, perf_events);

static DEFINE_MUTEX(perf_trace_lock);
#define INST_EV		0x08
#define CYC_EV		0x11
#define L2DM_EV		0x17

static const unsigned int perf_trace_event_id[NUM_EVENTS] = {
	CYC_EV,
	INST_EV,
	L2DM_EV
};

enum tp_pid_state_type {
	TP_DISABLED = 0,
	TP_ENABLED,
};

static struct perf_event_attr *alloc_attr(void)
{
	struct perf_event_attr *attr;

	attr = kzalloc(sizeof(struct perf_event_attr), GFP_KERNEL);
	if (!attr)
		return attr;

	attr->type = PERF_TYPE_RAW;
	attr->size = sizeof(struct perf_event_attr);
	attr->pinned = 1;

	return attr;
}

static int create_counters(int cpu)
{
	struct perf_event *pevent;
	struct perf_event_attr *attr;
	int err, i;

	/* Allocate an attribute for event initialization */
	attr = alloc_attr();
	if (!attr)
		return -ENOMEM;

	for (i = 0; i < NUM_EVENTS; i++) {
		attr->config = perf_trace_event_id[i];
		pevent = perf_event_create_kernel_counter(attr, cpu, NULL,
							  NULL, NULL);
		if (IS_ERR(pevent))
			goto err_out;

		per_cpu(perf_events, cpu)[i] = pevent;
	}

	kfree(attr);
	return 0;

err_out:
	err = PTR_ERR(pevent);
	kfree(attr);
	return err;
}

static void destroy_counters(int cpu)
{
	struct perf_event *pevent;
	int i;

	for (i = 0; i < NUM_EVENTS; i++) {
		pevent = per_cpu(perf_events, cpu)[i];
		if (pevent && !IS_ERR(pevent)) {
			perf_event_release_kernel(pevent);
			per_cpu(perf_events, cpu)[i] = NULL;
		}
	}
}

static int tracectr_cpu_hotplug_coming_up(unsigned int cpu)
{
	per_cpu(hotplug_flag, cpu) = 1;

	return 0;
}

static void setup_prev_cnts(u32 cpu)
{
	struct perf_event *pevent;
	int i;

	for (i = 0; i < NUM_EVENTS; i++) {
		pevent = per_cpu(perf_events, cpu)[i];
		pevent->pmu->read(pevent);
		per_cpu(previous_cnts[i], cpu) =
			(int32_t) local64_read(&pevent->count);
	}
}

static void tracectr_notifier(void *ignore, bool preempt,
			struct task_struct *prev, struct task_struct *next)
{
	u32 cnten_val;
	int current_pid;
	u32 cpu = task_cpu(next);

	if (tp_pid_state != TP_ENABLED)
		return;
	current_pid = next->pid;
	if (per_cpu(old_pid, cpu) != -1) {
		cnten_val = read_sysreg(pmcntenset_el0);
		per_cpu(cntenset_val, cpu) = cnten_val;
		/* Disable all the counters that were enabled */
		write_sysreg(cnten_val, pmcntenclr_el0);

		if (per_cpu(hotplug_flag, cpu) == 1) {
			per_cpu(hotplug_flag, cpu) = 0;
			setup_prev_cnts(cpu);
		} else {
			trace_sched_switch_with_ctrs(prev);
		}

		/* Enable all the counters that were disabled */
		write_sysreg(cnten_val, pmcntenset_el0);
	}
	per_cpu(old_pid, cpu) = current_pid;
}

/**
 * enable_tp_pid_locked - register the tracepoint & PMU counters
 *
 * This function requires caller to take perf_trace_lock.
 */
static int enable_tp_pid_locked(void)
{
	int cpu, ret = 0;
	cpumask_t mask;

	if (tp_pid_state == TP_ENABLED)
		return 0;

	cpumask_clear(&mask);

	for_each_possible_cpu(cpu) {
		ret = create_counters(cpu);
		if (ret) {
			pr_warn("Perf event init failed on CPU%d\n", cpu);
			goto err_out;
		}
		cpumask_set_cpu(cpu, &mask);
	}
	tp_pid_state = TP_ENABLED;
	register_trace_sched_switch(tracectr_notifier, NULL);
	trace_set_clr_event("perf_trace_counters",
			    "sched_switch_with_ctrs", 1);

	return ret;

err_out:
	pr_err("%s: failed, error: %d\n", __func__, ret);
	for_each_cpu(cpu, &mask)
		destroy_counters(cpu);

	return ret;
}

/**
 * disable_tp_pid_locked - unregister the tracepoint and free counters
 *
 * This function requires caller to take perf_trace_lock.
 */
static void disable_tp_pid_locked(void)
{
	int cpu;

	if (tp_pid_state == TP_DISABLED)
		return;

	tp_pid_state = TP_DISABLED;
	trace_set_clr_event("perf_trace_counters",
			    "sched_switch_with_ctrs", 0);
	unregister_trace_sched_switch(tracectr_notifier, NULL);

	for_each_possible_cpu(cpu)
		destroy_counters(cpu);
}

static ssize_t read_enabled_perftp_file_bool(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[2];

	buf[1] = '\n';
	if (tp_pid_state == TP_DISABLED)
		buf[0] = '0';
	else
		buf[0] = '1';
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t write_enabled_perftp_file_bool(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;

	buf[0] = 0;
	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	mutex_lock(&perf_trace_lock);

	switch (buf[0]) {
	case 'y':
	case 'Y':
	case '1':
		enable_tp_pid_locked();
		break;
	case 'n':
	case 'N':
	case '0':
		disable_tp_pid_locked();
		break;
	default:
		count = -EINVAL;
	}

	mutex_unlock(&perf_trace_lock);

	return count;
}

static const struct file_operations fops_perftp = {
	.read =		read_enabled_perftp_file_bool,
	.write =	write_enabled_perftp_file_bool,
	.llseek =	default_llseek,
};

static enum cpuhp_state hp_state;
static struct dentry *dfs_dir;

int __init init_tracecounters(void)
{
	struct dentry *file;
	unsigned int value = 1;
	int cpu, rc;
	cpumask_t mask;

	rc = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
		"tracectr_cpu_hotplug",
		tracectr_cpu_hotplug_coming_up,
		NULL);

	if (rc < 0) {
		pr_err("%s: failed, error: %d\n", __func__, rc);
		return rc;
	}

	hp_state = rc;

	dfs_dir = debugfs_create_dir("perf_debug_tp", NULL);
	if (!dfs_dir)
		return -ENOMEM;
	file = debugfs_create_file("enable", 0660, dfs_dir,
		&value, &fops_perftp);
	if (!file) {
		debugfs_remove_recursive(dfs_dir);
		return -ENOMEM;
	}

	cpumask_clear(&mask);

	for_each_possible_cpu(cpu) {
		per_cpu(old_pid, cpu) = -1;
		per_cpu(perf_events, cpu) = kcalloc(NUM_EVENTS,
					    sizeof(struct perf_event *),
					    GFP_KERNEL);
		if (!per_cpu(perf_events, cpu)) {
			pr_warn("Failed to allocate %d perf events for cpu %d\n",
				NUM_EVENTS, cpu);
			goto err_out;
		}
		cpumask_set_cpu(cpu, &mask);
	}

	return 0;

err_out:
	for_each_cpu(cpu, &mask)
		kfree(per_cpu(perf_events, cpu));
	debugfs_remove_recursive(dfs_dir);
	return -ENOMEM;
}

int __exit exit_tracecounters(void)
{
	int cpu;

	cpuhp_remove_state_nocalls(hp_state);
	for_each_possible_cpu(cpu)
		kfree(per_cpu(perf_events, cpu));
	debugfs_remove_recursive(dfs_dir);
	return 0;
}
late_initcall(init_tracecounters);
