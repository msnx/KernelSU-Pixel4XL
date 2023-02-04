/*
 * Copyright 2018 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/thermal.h>
#include <linux/pm_wakeup.h>
#include "google_psy.h"

#define USB_OVERHEAT_MITIGATION_VOTER	"USB_OVERHEAT_MITIGATION_VOTER"

enum {
	USB_NO_LIMIT = 0,
	USB_OVERHEAT_THROTTLE = 1,
	USB_MAX_THROTTLE_STATE = USB_OVERHEAT_THROTTLE,
};

static bool mitigation_enabled = true;
module_param_named(
	enable, mitigation_enabled, bool, 0600
);

struct overheat_event_stats {
	int plug_temp;
	int max_temp;
	int trip_time;
	int hysteresis_time;
	int cleared_time;
};

struct overheat_info {
	struct device              *dev;
	struct power_supply        *usb_psy;
	struct votable             *usb_icl_votable;
	struct votable             *disable_power_role_switch;
	struct notifier_block      psy_nb;
	struct delayed_work        port_overheat_work;
	struct wakeup_source	   overheat_ws;
	struct overheat_event_stats stats;
	struct thermal_cooling_device *cooling_dev;

	bool usb_connected;
	bool accessory_connected;
	bool usb_replug;
	bool overheat_mitigation;
	bool overheat_work_running;

	int temp;
	int plug_temp;
	int max_temp;
	time_t plug_time;
	time_t trip_time;
	time_t hysteresis_time;

	int begin_temp;
	int clear_temp;
	int overheat_work_delay_ms;
	int polling_freq;
	int check_status;
	unsigned long throttle_state;
};

#define OVH_ATTR(_name)							\
static ssize_t _name##_show(struct device *dev,				\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct overheat_info *ovh_info = dev_get_drvdata(dev);		\
									\
	return scnprintf(buf, PAGE_SIZE, "%d\n", ovh_info->stats._name);\
}									\
static DEVICE_ATTR_RO(_name);

OVH_ATTR(max_temp);
OVH_ATTR(plug_temp);
OVH_ATTR(trip_time);
OVH_ATTR(hysteresis_time);
OVH_ATTR(cleared_time);

static struct attribute *ovh_attr[] = {
	&dev_attr_max_temp.attr,
	&dev_attr_plug_temp.attr,
	&dev_attr_hysteresis_time.attr,
	&dev_attr_trip_time.attr,
	&dev_attr_cleared_time.attr,
	NULL,
};

static const struct attribute_group ovh_attr_group = {
	.attrs = ovh_attr,
};

static inline time_t get_seconds_since_boot(void)
{
	return div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC);
}

static inline int get_dts_vars(struct overheat_info *ovh_info)
{
	struct device *dev = ovh_info->dev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "google,begin-mitigation-temp",
				   &ovh_info->begin_temp);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read begin-mitigation-temp, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,end-mitigation-temp",
				   &ovh_info->clear_temp);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read end-mitigation-temp, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,port-overheat-work-interval",
				   &ovh_info->overheat_work_delay_ms);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read port-overheat-work-interval, ret=%d\n",
			ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,polling-freq",
				   &ovh_info->polling_freq);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read polling-freq, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int suspend_usb(struct overheat_info *ovh_info)
{
	int ret;

	ovh_info->usb_replug = false;

	/* disable USB */
	ret = vote(ovh_info->disable_power_role_switch,
		   USB_OVERHEAT_MITIGATION_VOTER, true, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't vote for disable_power_role_switch ret=%d\n",
			ret);
		return ret;
	}

	/* suspend charging */
	ret = vote(ovh_info->usb_icl_votable,
		  USB_OVERHEAT_MITIGATION_VOTER, true, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	ovh_info->trip_time = get_seconds_since_boot();
	ovh_info->overheat_mitigation = true;
	return ret;
}

static int resume_usb(struct overheat_info *ovh_info)
{
	int ret;

	/* Fill out stats so userspace can read them. */
	ovh_info->stats.max_temp = ovh_info->max_temp;
	ovh_info->stats.plug_temp = ovh_info->plug_temp;
	ovh_info->stats.trip_time =
		(int) (ovh_info->trip_time - ovh_info->plug_time);
	ovh_info->stats.hysteresis_time =
		(int) (ovh_info->hysteresis_time - ovh_info->trip_time);
	ovh_info->stats.cleared_time =
		(int) (get_seconds_since_boot() - ovh_info->hysteresis_time);

	/* enable charging */
	ret = vote(ovh_info->usb_icl_votable,
		  USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't un-vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	/* enable USB */
	ret = vote(ovh_info->disable_power_role_switch,
		   USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't un-vote for disable_power_role_switch ret=%d\n",
			ret);
		return ret;
	}

	/* Notify userspace to read the stats. */
	kobject_uevent(&ovh_info->dev->kobj, KOBJ_CHANGE);

	ovh_info->max_temp = INT_MIN;
	ovh_info->plug_temp = INT_MIN;
	ovh_info->plug_time = 0;
	ovh_info->trip_time = 0;
	ovh_info->hysteresis_time = 0;
	ovh_info->overheat_mitigation = false;
	ovh_info->usb_replug = false;
	return ret;
}

/*
 * Update usb_connected, accessory_connected, usb_replug, and plug_temp
 * status in overheat_info struct.
 */
static int update_usb_status(struct overheat_info *ovh_info)
{
	int ret;
	bool prev_state = ovh_info->usb_connected ||
		ovh_info->accessory_connected;
	bool curr_state;
	int *check_status = &ovh_info->check_status;

	/* Port is too hot to safely check the connected status. */
	if (ovh_info->overheat_mitigation &&
	    ovh_info->temp > ovh_info->clear_temp)
		return -EBUSY;

	if (ovh_info->overheat_mitigation) {
		if (!ovh_info->hysteresis_time)
			ovh_info->hysteresis_time = get_seconds_since_boot();
		// Only check USB status every polling_freq instances
		*check_status = (*check_status + 1) % ovh_info->polling_freq;
		if (*check_status > 0)
			return 0;
		ret = vote(ovh_info->disable_power_role_switch,
			   USB_OVERHEAT_MITIGATION_VOTER, false, 0);
		if (ret < 0) {
			dev_err(ovh_info->dev,
				"Couldn't un-vote for disable_power_role_switch ret=%d\n",
				ret);
			return ret;
		}
		msleep(200);
	}

	dev_dbg(ovh_info->dev, "Updating USB connected status\n");

	/*
	 * Update USB present status to determine if USB has been disconnected.
	 * If we use USB online status to determine replug, we will need to
	 * extend the delay between re-enabling CC detection and checking the
	 * USB online status.
	 */
	ret = GPSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_PRESENT);
	if (ret < 0)
		return ret;
	ovh_info->usb_connected = ret;

	ret = GPSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE);
	if (ret < 0)
		return ret;
	ovh_info->accessory_connected = (ret == POWER_SUPPLY_TYPEC_SINK) ||
			(ret == POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE);

	curr_state = ovh_info->usb_connected || ovh_info->accessory_connected;
	if (curr_state && !prev_state) {
		ovh_info->plug_time = get_seconds_since_boot();
		ovh_info->plug_temp = ovh_info->temp;
	}

	if (ovh_info->overheat_mitigation) {
		ret = vote(ovh_info->disable_power_role_switch,
			   USB_OVERHEAT_MITIGATION_VOTER, true, 0);
		if (ret < 0) {
			dev_err(ovh_info->dev,
				"Couldn't vote for disable_power_role_switch ret=%d\n",
				ret);
			return ret;
		}
	}

	if (curr_state != prev_state)
		dev_info(ovh_info->dev,
			 "USB is %sconnected",
			 curr_state ? "" : "dis");

	// USB should be disconnected for two cycles before replug is acked
	if (ovh_info->overheat_mitigation && !curr_state && !prev_state)
		ovh_info->usb_replug = true;

	return 0;
}

static inline int get_usb_port_temp(struct overheat_info *ovh_info)
{
	int temp;

	temp = GPSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_TEMP);

	if (temp == -EINVAL || temp == -ENODATA)
		return temp;

	dev_info(ovh_info->dev, "Update USB port temp:%d\n", temp);
	if (temp > ovh_info->max_temp)
		ovh_info->max_temp = temp;

	ovh_info->temp = temp;
	return 0;
}

static int psy_changed(struct notifier_block *nb, unsigned long action,
		       void *data)
{
	struct power_supply *psy = data;
	struct overheat_info *ovh_info =
			container_of(nb, struct overheat_info, psy_nb);

	if ((action != PSY_EVENT_PROP_CHANGED) || (psy == NULL) ||
	    (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    !strcmp(psy->desc->name, "usb")) {
		dev_dbg(ovh_info->dev, "name=usb evt=%lu\n", action);
		if (!ovh_info->overheat_work_running)
			schedule_delayed_work(&ovh_info->port_overheat_work, 0);
	}
	return NOTIFY_OK;
}

static void port_overheat_work(struct work_struct *work)
{
	struct overheat_info *ovh_info =
			container_of(work, struct overheat_info,
				     port_overheat_work.work);
	int ret = 0;

	// Take a wake lock to ensure we poll the temp regularly
	if (!ovh_info->overheat_work_running)
		__pm_stay_awake(&ovh_info->overheat_ws);
	ovh_info->overheat_work_running = true;

	if (get_usb_port_temp(ovh_info) < 0)
		goto rerun;

	ret = update_usb_status(ovh_info);
	if (ret < 0)
		goto rerun;

	if (ovh_info->overheat_mitigation && (!mitigation_enabled ||
	    (ovh_info->temp < ovh_info->clear_temp && ovh_info->usb_replug))) {
		dev_err(ovh_info->dev, "Port overheat mitigated\n");
		resume_usb(ovh_info);
	} else if (!ovh_info->overheat_mitigation &&
		 mitigation_enabled && ovh_info->temp > ovh_info->begin_temp) {
		dev_err(ovh_info->dev, "Port overheat triggered\n");
		suspend_usb(ovh_info);
		goto rerun;
	}

	if (ovh_info->overheat_mitigation || ovh_info->throttle_state)
		goto rerun;
	// Do not run again, USB port isn't overheated
	ovh_info->overheat_work_running = false;
	__pm_relax(&ovh_info->overheat_ws);
	return;

rerun:
	schedule_delayed_work(
			&ovh_info->port_overheat_work,
			msecs_to_jiffies(ovh_info->overheat_work_delay_ms));
}

static int usb_get_cur_state(struct thermal_cooling_device *cooling_dev,
							unsigned long *state)
{
	struct overheat_info *ovh_info = cooling_dev->devdata;

	if (!ovh_info)
		return -EINVAL;

	*state = ovh_info->throttle_state;

	return 0;
}

static int usb_get_max_state(struct thermal_cooling_device *cooling_dev,
							unsigned long *state)
{
	*state = USB_MAX_THROTTLE_STATE;

	return 0;
}

static int usb_set_cur_state(struct thermal_cooling_device *cooling_dev,
							unsigned long state)
{
	struct overheat_info *ovh_info = cooling_dev->devdata;
	unsigned long current_state;

	if (!ovh_info)
		return -EINVAL;

	if (state > USB_MAX_THROTTLE_STATE)
		return -EINVAL;

	current_state = ovh_info->throttle_state;
	ovh_info->throttle_state = state;

	if (current_state != state) {
		dev_info(ovh_info->dev, "usb overheat throttle state=%lu\n",
			 state);
		mod_delayed_work(system_wq, &ovh_info->port_overheat_work, 0);
	}
	return 0;
}

static const struct thermal_cooling_device_ops usb_cooling_ops = {
	.get_max_state = usb_get_max_state,
	.get_cur_state = usb_get_cur_state,
	.set_cur_state = usb_set_cur_state,
};

static int ovh_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct overheat_info *ovh_info;
	struct power_supply  *usb_psy;
	struct votable       *usb_icl_votable;
	struct votable       *disable_power_role_switch;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy)
		return -EPROBE_DEFER;

	usb_icl_votable = find_votable("USB_ICL");
	if (usb_icl_votable == NULL) {
		pr_err("Couldn't find USB_ICL votable\n");
		return -EPROBE_DEFER;
	}

	disable_power_role_switch = find_votable("DISABLE_POWER_ROLE_SWITCH");
	if (disable_power_role_switch == NULL) {
		pr_err("Couldn't find DISABLE_POWER_ROLE_SWITCH votable\n");
		return -EPROBE_DEFER;
	}

	ovh_info = devm_kzalloc(&pdev->dev, sizeof(*ovh_info), GFP_KERNEL);
	if (!ovh_info)
		return -ENOMEM;

	ovh_info->dev = &pdev->dev;
	ovh_info->usb_icl_votable = usb_icl_votable;
	ovh_info->disable_power_role_switch = disable_power_role_switch;
	ovh_info->usb_psy = usb_psy;
	ovh_info->max_temp = INT_MIN;
	ovh_info->plug_temp = INT_MIN;

	ret = get_dts_vars(ovh_info);
	if (ret < 0)
		return -ENODEV;

	// initialize votables
	vote(ovh_info->usb_icl_votable,
	     USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	vote(ovh_info->disable_power_role_switch,
	     USB_OVERHEAT_MITIGATION_VOTER, false, 0);

	wakeup_source_init(&ovh_info->overheat_ws, "overheat_mitigation");
	INIT_DELAYED_WORK(&ovh_info->port_overheat_work, port_overheat_work);

	// register power supply change notifier to update usb metric data
	ovh_info->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&ovh_info->psy_nb);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Cannot register power supply notifer, ret=%d\n", ret);
		return ret;
	}

	/* Register cooling device */
	ovh_info->cooling_dev = thermal_of_cooling_device_register(
				dev_of_node(ovh_info->dev), "usb-port",
				ovh_info, &usb_cooling_ops);

	if (IS_ERR(ovh_info->cooling_dev)) {
		ret = PTR_ERR(ovh_info->cooling_dev);
		dev_err(ovh_info->dev, "%s: failed to register cooling device: %d\n",
				__func__, ret);
		return ret;
	}

	platform_set_drvdata(pdev, ovh_info);
	ret = sysfs_create_group(&ovh_info->dev->kobj, &ovh_attr_group);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Cannot create sysfs group, ret=%d\n", ret);
	}

	return 0;
}

static int ovh_remove(struct platform_device *pdev)
{
	struct overheat_info *ovh_info = platform_get_drvdata(pdev);
	if (ovh_info) {
		power_supply_unreg_notifier(&ovh_info->psy_nb);
		sysfs_remove_group(&ovh_info->dev->kobj, &ovh_attr_group);
		wakeup_source_trash(&ovh_info->overheat_ws);
	}
	return 0;
}

static const struct of_device_id match_table[] = {
	{
		.compatible = "google,overheat_mitigation",
	},
	{},
};

static struct platform_driver ovh_driver = {
	.driver = {
		.name = "google,overheat_mitigation",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe = ovh_probe,
	.remove = ovh_remove,
};

module_platform_driver(ovh_driver);
MODULE_DESCRIPTION("USB port overheat mitigation driver");
MODULE_AUTHOR("Maggie White <maggiewhite@google.com>");
MODULE_LICENSE("GPL v2");
