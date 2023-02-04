/*
 * drivers/power/reset/debug-reboot.c
 *
 * Utility module to test various debug reboot types.
 *
 * Copyright (C) 2019 Google, Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/time.h>
#include <linux/sched/debug.h>

#define PANIC_REBOOT_CMD "debug-reboot-panic"
#define WDOG_REBOOT_CMD "debug-reboot-watchdog"
#define HANG_REBOOT_CMD "debug-reboot-hang"
#define REBOOT_TIMER_TIMEOUT_MS 15000
static struct timer_list reboot_timer;

#ifdef CONFIG_DEBUG_REBOOT_DEFAULT_ON
static bool enable = 1;
#else
static bool enable;
#endif
module_param(enable, bool, 0644);
MODULE_PARM_DESC(enable, "Enable/disable debug reboot commands");

static bool stop_cpus = 1;
module_param(stop_cpus, bool, 0644);
MODULE_PARM_DESC(stop_cpus, "Stop other cpus during watchdog (default: Y)");

static void debug_reboot_panic(void)
{
	char *pointer = NULL;

	pr_info("debug-reboot: Trigger a panic\n");

	/* Trigger a NULL pointer dereference */
	*pointer = 'a';

	/* Should not reach here */
	pr_err("debug-reboot: Trigger panic failed!\n");
}

static void debug_reboot_hang(void)
{
	pr_info("debug-reboot: Hogging a CPU now\n");

	for(;;)
		;

	/* Should not reach here */
	pr_err("debug-reboot: Simulate reboot hang fail !\n");
}

static void debug_reboot_watchdog(void)
{
	pr_info("debug-reboot: Trigger a watchdog\n");

	if (stop_cpus) {
		/* Stop other CPUs to ensure only this core running */
		smp_send_stop();
	}

	/* Disable interrupts and loop forever */
	local_irq_disable();
	debug_reboot_hang();

	/* Should not reach here */
	pr_err("debug-reboot: Trigger watchdog failed!\n");
}

static void reboot_timeout(unsigned long unused)
{
	pr_info("debug-reboot: Reboot timer (%d seconds) timeout - Show All Blocked State\n"
		, (REBOOT_TIMER_TIMEOUT_MS/1000));

	show_state_filter(TASK_UNINTERRUPTIBLE);

	debug_reboot_panic();
}

static void add_reboot_timer(void)
{
	pr_info("debug-reboot: Create reboot monitor timer now\n");

	reboot_timer.function = reboot_timeout;
	reboot_timer.expires = jiffies +
		msecs_to_jiffies(REBOOT_TIMER_TIMEOUT_MS);
	add_timer(&reboot_timer);
}

static int debug_reboot_notify(struct notifier_block *nb,
				unsigned long code, void *cmd)
{
	/*add a timer to monitor reboot or shutdown process.*/
	add_reboot_timer();

	if (enable && code == SYS_RESTART) {
		if (cmd != NULL) {
			if (!strcmp((char *)cmd, PANIC_REBOOT_CMD)) {
				debug_reboot_panic();
				/* Should not return... */
			} else if (!strcmp((char *)cmd, WDOG_REBOOT_CMD)) {
				debug_reboot_watchdog();
				/* Should not return... */
			} else if (!strcmp((char *)cmd, HANG_REBOOT_CMD)) {
				debug_reboot_hang();
				/* Should not return... */
			}

		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block debug_reboot_nb = {
	.notifier_call = debug_reboot_notify,
	.priority = INT_MAX,
};

static int __init debug_reboot_init(void)
{
	init_timer(&reboot_timer);

	return register_reboot_notifier(&debug_reboot_nb);
}

static void __exit debug_reboot_exit(void)
{
	unregister_reboot_notifier(&debug_reboot_nb);

	del_timer_sync(&reboot_timer);
}

module_init(debug_reboot_init);
module_exit(debug_reboot_exit);

MODULE_DESCRIPTION("Module for testing debug reboots");
MODULE_AUTHOR("Jonglin Lee <jonglin@google.com>");
MODULE_LICENSE("GPL v2");
