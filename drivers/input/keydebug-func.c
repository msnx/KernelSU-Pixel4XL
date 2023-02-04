/* drivers/input/keydebug-func.c
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
 *
 */

#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>
#include <linux/time.h>
#include <linux/tick.h>
#include <linux/rtc.h>
#include <linux/threads.h>
#include <linux/nmi.h>
#include <asm/irq_regs.h>
#include <linux/keydebug-func.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>
#include <linux/sched/debug.h>

#define NUM_BUSY_TASK_CHECK 5

struct kernel_top_context {
	u64 *prev_tasktics_array;
	u64 *frame_tasktics_array;
	pid_t *curr_task_pid_array;
	pid_t top_task_pid_array[NUM_BUSY_TASK_CHECK];
	struct task_struct **task_ptr_array;
	struct kernel_cpustat curr_all_cpustat;
	struct kernel_cpustat prev_all_cpustat;
	u64 frame_cpustat_total;
	bool kernel_top_alloc_done;
};

static struct kernel_top_context ktop_cxt;
static DEFINE_MUTEX(kernel_top_mutex);

#ifdef arch_idle_time

static u64 get_idle_time(int cpu)
{
	u64 idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
	u64 idle, idle_usecs = -1ULL;

	if (cpu_online(cpu))
		idle_usecs = get_cpu_idle_time_us(cpu, NULL);

	if (idle_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = idle_usecs * NSEC_PER_USEC;

	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

#endif

static void get_all_cpustat(struct kernel_cpustat *cpu_stat)
{
	int cpu;

	if (!cpu_stat)
		return;

	memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

#ifdef CONFIG_SMP
	for_each_possible_cpu(cpu) {
		cpu_stat->cpustat[CPUTIME_USER] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
		cpu_stat->cpustat[CPUTIME_NICE] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
		cpu_stat->cpustat[CPUTIME_SYSTEM] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
		cpu_stat->cpustat[CPUTIME_IDLE] +=
			get_idle_time(cpu);
		cpu_stat->cpustat[CPUTIME_IOWAIT] +=
			get_iowait_time(cpu);
		cpu_stat->cpustat[CPUTIME_IRQ] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		cpu_stat->cpustat[CPUTIME_SOFTIRQ] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
		cpu_stat->cpustat[CPUTIME_STEAL] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
		cpu_stat->cpustat[CPUTIME_GUEST] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST];
		cpu_stat->cpustat[CPUTIME_GUEST_NICE] +=
			kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST_NICE];
	}
#endif
}

static void sort_top_tasks(u64 *frame_tasktics_array,
	pid_t *curr_task_pid_array, int task_count, pid_t *result)
{
	int i = 0, j = 0, k = 0;
	int pid_checked = 0;
	pid_t p = 0;

	for (i = 0; i < NUM_BUSY_TASK_CHECK; i++) {
		result[i] = 0;
		/* Find the task which has the largest cputime in this frame */
		if (i == 0) {
			for (j = 0; j < task_count; j++) {
				p = curr_task_pid_array[j];

				if (frame_tasktics_array[result[i]] <
					frame_tasktics_array[p])
					result[i] = p;
			}
		} else {
			for (j = 0; j < task_count; j++) {
				p = curr_task_pid_array[j];
				for (k = 0; k < i; k++) {
					if (result[k] == p) {
						pid_checked = 1;
						break;
					}
				}
				if (pid_checked) {
					pid_checked = 0;
					continue;
				}
				if (frame_tasktics_array[result[i]] <
					frame_tasktics_array[p])
					result[i] = p;
			}
		}
	}
}

static u64 cal_frame_cpustat_total(struct kernel_cpustat curr_all_cpustat,
	struct kernel_cpustat prev_all_cpustat)
{
	u64 user_time = 0, system_time = 0, io_time = 0;
	u64 irq_time = 0, idle_time = 0;

	user_time = ((curr_all_cpustat.cpustat[CPUTIME_USER] +
				curr_all_cpustat.cpustat[CPUTIME_NICE]) -
				(prev_all_cpustat.cpustat[CPUTIME_USER] +
				prev_all_cpustat.cpustat[CPUTIME_NICE]));
	system_time = (curr_all_cpustat.cpustat[CPUTIME_SYSTEM] -
				prev_all_cpustat.cpustat[CPUTIME_SYSTEM]);
	io_time = (curr_all_cpustat.cpustat[CPUTIME_IOWAIT] -
				prev_all_cpustat.cpustat[CPUTIME_IOWAIT]);
	irq_time = ((curr_all_cpustat.cpustat[CPUTIME_IRQ] +
				curr_all_cpustat.cpustat[CPUTIME_SOFTIRQ]) -
				(prev_all_cpustat.cpustat[CPUTIME_IRQ] +
				prev_all_cpustat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = ((curr_all_cpustat.cpustat[CPUTIME_IDLE] >
				prev_all_cpustat.cpustat[CPUTIME_IDLE]) ?
				curr_all_cpustat.cpustat[CPUTIME_IDLE] -
				prev_all_cpustat.cpustat[CPUTIME_IDLE] : 0);
	idle_time += ((curr_all_cpustat.cpustat[CPUTIME_STEAL] +
				curr_all_cpustat.cpustat[CPUTIME_GUEST]) -
				(prev_all_cpustat.cpustat[CPUTIME_STEAL] +
				prev_all_cpustat.cpustat[CPUTIME_GUEST]));

	return (user_time + system_time + io_time + irq_time + idle_time);
}

static void kernel_top_cal(void)
{
	int task_count = 0;
	struct task_struct *tsk;
	struct task_cputime cputime;
	struct kernel_top_context *cxt = &ktop_cxt;

	/* Calculate each tasks tics in this time frame*/
	rcu_read_lock();
	for_each_process(tsk) {
		thread_group_cputime(tsk, &cputime);
		if (tsk->pid < PID_MAX_DEFAULT) {
			u64 cur_tasktics = (cputime.utime + cputime.stime);

			cxt->frame_tasktics_array[tsk->pid] =
				cur_tasktics -
					cxt->prev_tasktics_array[tsk->pid];

			cxt->prev_tasktics_array[tsk->pid] = cur_tasktics;
			cxt->task_ptr_array[tsk->pid] = tsk;

			if (cxt->frame_tasktics_array[tsk->pid] > 0) {
				cxt->curr_task_pid_array[task_count] = tsk->pid;
				task_count++;
			}
		}
	}
	rcu_read_unlock();

	get_all_cpustat(&cxt->curr_all_cpustat);

	sort_top_tasks(cxt->frame_tasktics_array,
		cxt->curr_task_pid_array, task_count, cxt->top_task_pid_array);

	cxt->frame_cpustat_total =
		cal_frame_cpustat_total(cxt->curr_all_cpustat,
			cxt->prev_all_cpustat);
	memcpy(&cxt->prev_all_cpustat,
		&cxt->curr_all_cpustat, sizeof(struct kernel_cpustat));

}

static void kernel_top_show(void)
{
	pid_t top_n_pid = 0;
	int i;
	struct kernel_top_context *cxt = &ktop_cxt;

	pr_info("%s: CPU Usage     PID     Name\n", __func__);
	for (i = 0; i < NUM_BUSY_TASK_CHECK; i++) {
		if (cxt->frame_cpustat_total > 0) {
			top_n_pid = cxt->top_task_pid_array[i];
			pr_info("%s: %8llu%%%8d     %s%10llu\n", __func__,
				cxt->frame_tasktics_array[top_n_pid] * 100 /
					cxt->frame_cpustat_total,
				top_n_pid,
				cxt->task_ptr_array[top_n_pid]->comm,
			nsec_to_clock_t(cxt->frame_tasktics_array[top_n_pid]));
		}
	}

	memset(cxt->frame_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->task_ptr_array, 0,
		sizeof(struct task_struct *) * PID_MAX_DEFAULT);
	memset(cxt->curr_task_pid_array, 0, sizeof(pid_t) * PID_MAX_DEFAULT);
}

void kernel_top_monitor(void)
{
	struct timespec ts;
	struct rtc_time tm;
	struct kernel_top_context *cxt = &ktop_cxt;

	mutex_lock(&kernel_top_mutex);
	if (cxt->kernel_top_alloc_done == false)
		goto done;

	kernel_top_cal();
	kernel_top_show();

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	pr_info("%s: Kernel Top Statistic done"
			"(%02d-%02d %02d:%02d:%02d)\n", __func__,
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

done:
	mutex_unlock(&kernel_top_mutex);
}
EXPORT_SYMBOL_GPL(kernel_top_monitor);

void kernel_top_init(void)
{
	struct task_struct *tsk;
	struct task_cputime cputime;
	struct timespec ts;
	struct rtc_time tm;
	struct kernel_top_context *cxt = &ktop_cxt;

	mutex_lock(&kernel_top_mutex);
	if (cxt->kernel_top_alloc_done == false) {

		cxt->prev_tasktics_array =
			vmalloc(sizeof(u64) * PID_MAX_DEFAULT);
		if (cxt->prev_tasktics_array == NULL)
			goto err_alloc_prev_tasktics;
		cxt->frame_tasktics_array =
			vmalloc(sizeof(u64) * PID_MAX_DEFAULT);
		if (cxt->frame_tasktics_array == NULL)
			goto err_alloc_frame_tasktics;
		cxt->task_ptr_array =
			vmalloc(sizeof(struct task_struct *) * PID_MAX_DEFAULT);
		if (cxt->task_ptr_array == NULL)
			goto err_alloc_task_ptr;
		cxt->curr_task_pid_array =
			vmalloc(sizeof(pid_t) * PID_MAX_DEFAULT);
		if (cxt->curr_task_pid_array == NULL)
			goto err_alloc_curr_task_pid;

		cxt->kernel_top_alloc_done = true;
	}

	memset(cxt->prev_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->frame_tasktics_array, 0, sizeof(u64) * PID_MAX_DEFAULT);
	memset(cxt->task_ptr_array, 0,
		sizeof(struct task_struct *) * PID_MAX_DEFAULT);
	memset(cxt->curr_task_pid_array, 0, sizeof(pid_t) * PID_MAX_DEFAULT);

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	pr_info("%s: Kernel Top Statistic start"
			"(%02d-%02d %02d:%02d:%02d)\n", __func__,
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	get_all_cpustat(&cxt->curr_all_cpustat);
	memcpy(&cxt->prev_all_cpustat,
		&cxt->curr_all_cpustat, sizeof(struct kernel_cpustat));

	/* Calculate time in a process;
	 * the sum of user time (utime) and system time (stime)*/
	rcu_read_lock();
	for_each_process(tsk) {
		if (tsk->pid < PID_MAX_DEFAULT) {
			thread_group_cputime(tsk, &cputime);
			cxt->prev_tasktics_array[tsk->pid] =
				cputime.stime + cputime.utime;
		}
	}
	rcu_read_unlock();
	goto done;

err_alloc_curr_task_pid:
	vfree(cxt->curr_task_pid_array);
err_alloc_task_ptr:
	vfree(cxt->task_ptr_array);
err_alloc_frame_tasktics:
	vfree(cxt->frame_tasktics_array);
err_alloc_prev_tasktics:
	vfree(cxt->prev_tasktics_array);

	cxt->kernel_top_alloc_done = false;
	pr_info("%s: memory allocate failed", __func__);
done:
	mutex_unlock(&kernel_top_mutex);
}

void kernel_top_exit(void)
{
	struct kernel_top_context *cxt = &ktop_cxt;

	mutex_lock(&kernel_top_mutex);
	if (cxt->kernel_top_alloc_done) {
		vfree(cxt->curr_task_pid_array);
		vfree(cxt->task_ptr_array);
		vfree(cxt->frame_tasktics_array);
		vfree(cxt->prev_tasktics_array);
		memset(cxt, 0, sizeof(*cxt));
	}
	mutex_unlock(&kernel_top_mutex);
}

#ifdef CONFIG_SMP
static DEFINE_SPINLOCK(show_lock);

static void keydebug_showacpu(void *dummy)
{
	unsigned long flags;

	/* Idle CPUs have no interesting backtrace. */
	if (idle_cpu(smp_processor_id()))
		return;

	spin_lock_irqsave(&show_lock, flags);
	dump_stack();
	spin_unlock_irqrestore(&show_lock, flags);
}

void keydebug_showallcpus(void)
{
	if(!trigger_all_cpu_backtrace()) {
		struct pt_regs *regs = NULL;

		if (in_irq())
			regs = get_irq_regs();
		if (regs) {
			pr_info("CPU%d:\n", smp_processor_id());
			show_regs(regs);
		}
		dump_stack();
		smp_call_function(keydebug_showacpu, NULL, 0);
	}
}
#else
void keydebug_showallcpus(void)
{
}
#endif
