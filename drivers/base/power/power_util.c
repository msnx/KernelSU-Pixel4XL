#include <linux/rtc.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/module.h>
#include "power_util.h"

#define MSM_POWER_UTIL_DELAY_TIME 5000

static struct workqueue_struct *power_pm_monitor_wq;
struct delayed_work dwork;

extern void pm_print_debug_wakeup_sources(void);
extern void msm_rpmh_master_stats_dump(void);
extern int msm_rpmstats_stats_dump(void);
static void power_pm_monitor_work_func(struct work_struct *work)
{
	struct timespec ts;
	struct rtc_time tm;
	int ret = 0;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	pr_info("[K][PM] power PM Statistic start (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	/* Show wakeup source */
	pm_print_debug_wakeup_sources();
	msm_rpmh_master_stats_dump();
	ret = msm_rpmstats_stats_dump();
	if (ret < 0)
		pr_err("Unabled to dump System sleep stats, ret=%d", ret);

	queue_delayed_work(power_pm_monitor_wq, &dwork,
				msecs_to_jiffies(MSM_POWER_UTIL_DELAY_TIME));
	pr_info("[K][PM] power PM Statistic done\n");
}

static int __init power_pm_monitor_init(void)
{
	power_pm_monitor_wq = NULL;
	/* Create private workqueue */
	power_pm_monitor_wq = create_workqueue("power_pm_monitor_wq");

	if (!power_pm_monitor_wq)
		return 1;

	pr_info("[K] Success to create power_pm_monitor_wq (0x%p).\n",
						power_pm_monitor_wq);

	INIT_DELAYED_WORK(&dwork, power_pm_monitor_work_func);
	queue_delayed_work(power_pm_monitor_wq, &dwork,
		msecs_to_jiffies(MSM_POWER_UTIL_DELAY_TIME));

	return 0;
}
late_initcall(power_pm_monitor_init);
