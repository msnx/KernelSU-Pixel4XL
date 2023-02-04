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

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include "google_bms.h"
#include "google_psy.h"
#include "qmath.h"
#include "logbuffer.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define ELAP_LIMIT_S 60


void ttf_log(const struct batt_ttf_stats *stats, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	logbuffer_vlog(stats->ttf_log, fmt, args);
	va_end(args);
}

/* actual adapter current capability for this charging event
 * NOTE: peformance for a tier are known only after entering the tier
 */
static int ttf_pwr_icl(const struct gbms_ce_tier_stats *ts,
		       const union gbms_ce_adapter_details *ad)
{
	int elap, amperage;

	elap = ts->time_fast + ts->time_taper;
	if (elap <= ELAP_LIMIT_S)
		amperage = ad->ad_amperage * 100;
	else
		amperage = ts->icl_sum / (elap + ts->time_other);

	return amperage;
}

/* NOTE: the current in taper might need to be accounted in a different way */
static int ttf_pwr_ibatt(const struct gbms_ce_tier_stats *ts)
{
	int avg_ibatt, elap, sign = 1;

	elap = ts->time_fast + ts->time_taper;
	/* averages are not reliable until after some time in tier */
	if (elap <= ELAP_LIMIT_S) {
		pr_debug("%s: limit=%d elap=%d (%d+%d) o=%d\n", __func__,
			 elap, ELAP_LIMIT_S, ts->time_fast, ts->time_taper,
			 ts->time_other);
		return 0;
	}

	/* actual, called only when avg_ibatt in tier indicates charging */
	avg_ibatt = ts->ibatt_sum / (elap + ts->time_other);
	if (avg_ibatt < 0)
		sign = -1;

	pr_debug("%s: elap=%d (%d+%d+%d) sum=%ld avg_ibatt=%d\n", __func__,
		 elap, ts->time_fast, ts->time_taper, ts->time_other,
		 ts->ibatt_sum, avg_ibatt * sign);

	return avg_ibatt * sign;
}

/* nominal voltage tier index for this soc */
static int ttf_pwr_vtier_idx(const struct batt_ttf_stats *stats, int soc)
{
	int i;

	for (i = 1; i < GBMS_STATS_TIER_COUNT; i++)
		if (soc < stats->tier_stats[i].soc_in >> 8)
			break;

	return i - 1;
}

/*
 * reference or current average current demand for a soc at max rate.
 * NOTE: always <= cc_max for reference temperature
 */
static int ttf_ref_cc(const struct batt_ttf_stats *stats, int soc)
{
	const struct ttf_soc_stats *sstat = NULL;
	int delta_cc;

	/* soc average current demand */
	if (stats->soc_stats.cc[soc + 1] && stats->soc_stats.cc[soc] &&
	    stats->soc_stats.elap[soc])
		sstat = &stats->soc_stats;
	else if (stats->soc_ref.cc[soc + 1] && stats->soc_ref.cc[soc] &&
		 stats->soc_ref.elap[soc])
		sstat = &stats->soc_ref;
	else
		return 0;

	delta_cc = (sstat->cc[soc + 1] - sstat->cc[soc]);

	pr_debug("%s %d: delta_cc=%d elap=%ld\n", __func__, soc,
		delta_cc, sstat->elap[soc]);

	return (delta_cc * 3600) / sstat->elap[soc];
}

/* assumes that health is active for any soc greater than CHG_HEALTH_REST_SOC */
static int ttf_pwr_health(const struct gbms_charging_event *ce_data,
			  int soc)
{
	return CHG_HEALTH_REST_IS_ACTIVE(&ce_data->ce_health) &&
	       soc >= CHG_HEALTH_REST_SOC(&ce_data->ce_health);
}

static int ttf_pwr_health_pause(const struct gbms_charging_event *ce_data,
			  int soc)
{
	return CHG_HEALTH_REST_IS_PAUSE(&ce_data->ce_health) &&
	       soc >= CHG_HEALTH_REST_SOC(&ce_data->ce_health);
}

/*
 * equivalent icl: minimum between actual input current limit and battery
 * everage current _while_in_tier. actual_icl will be lower in high current
 * tiers for bad cables, ibatt is affected by temperature tier and sysload.
 */
static int ttf_pwr_equiv_icl(const struct gbms_charging_event *ce_data,
			     int vbatt_idx, int soc)
{
	const struct gbms_chg_profile *profile = ce_data->chg_profile;
	const int aratio = (ce_data->adapter_details.ad_voltage * 10000) /
			   (profile->volt_limits[vbatt_idx] / 1000);
	const struct gbms_ce_tier_stats *tier_stats;
	const int efficiency = 95; /* TODO: use real efficiency */
	const u32 capacity_ma = profile->capacity_ma;
	const int rest_rate = ce_data->ce_health.rest_rate;
	int equiv_icl, act_icl, act_ibatt, health_ibatt = -1;

	/* Health collects in ce_data->health_stats vtier */
	if (ttf_pwr_health(ce_data, soc)) {
		health_ibatt = ce_data->ce_health.rest_cc_max / 1000;
		tier_stats = &ce_data->health_stats;
	} else if (ttf_pwr_health_pause(ce_data, soc)) {
		/* use ACTIVE current in PAUSE stat for ttf calculation */
		health_ibatt = (capacity_ma * rest_rate * 10) / 1000;
		/* use ACTIVE tier in PAUSE stat for ttf calculation */
		tier_stats = &ce_data->health_stats;
	} else {
		tier_stats = &ce_data->tier_stats[vbatt_idx];
	}

	/*
	 * actual adapter capabilities at adapter voltage for vtier
	 * NOTE: demand and cable might cause voltage and icl do droop
	 */
	act_icl = ttf_pwr_icl(tier_stats, &ce_data->adapter_details);
	if (act_icl <= 0) {
		pr_debug("%s: negative,null act_icl=%d\n", __func__, act_icl);
		return -EINVAL;
	}

	/* scale icl (at adapter voltage) to vtier */
	equiv_icl = (act_icl * aratio * 100) / efficiency;
	pr_debug("%s: act_icl=%d aratio=%d equiv_icl=%d\n",
		 __func__, act_icl, aratio, equiv_icl);

	/* actual ibatt in this tier: act_ibatt==0 when too early to tell */
	act_ibatt = ttf_pwr_ibatt(tier_stats);
	if (act_ibatt == 0 && health_ibatt > 0)
		act_ibatt = health_ibatt;
	if (act_ibatt < 0) {
		pr_debug("%s: discharging ibatt=%d\n", __func__, act_ibatt);
		return -EINVAL;
	}

	/* assume that can deliver equiv_icl when act_ibatt == 0 */
	if (act_ibatt > 0 && act_ibatt < equiv_icl) {
		pr_debug("%s: sysload ibatt=%d, reduce icl %d->%d\n",
			 __func__, act_ibatt, equiv_icl, act_ibatt);
		equiv_icl = act_ibatt;
	}

	pr_debug("%s: equiv_icl=%d\n", __func__, equiv_icl);
	return equiv_icl;
}

/*
 * time scaling factor for available power and SOC demand.
 * NOTE: usually called when soc < ssoc_in && soc > ce_data->last_soc
 * TODO: this is very inefficient
 */
static int ttf_pwr_ratio(const struct batt_ttf_stats *stats,
			 const struct gbms_charging_event *ce_data,
			 int soc)
{
	const struct gbms_chg_profile *profile = ce_data->chg_profile;
	int cc_max, vbatt_idx, temp_idx;
	int avg_cc, equiv_icl;
	int ratio;

	/* regular charging tier */
	vbatt_idx = ttf_pwr_vtier_idx(stats, soc);
	if (vbatt_idx < 0)
		return -EINVAL;

	/* TODO: compensate with average increase/decrease of temperature? */
	temp_idx = ce_data->tier_stats[vbatt_idx].temp_idx;
	if (temp_idx == -1) {
		int64_t t_avg;
		const int elap = ce_data->tier_stats[vbatt_idx].time_fast +
		 		 ce_data->tier_stats[vbatt_idx].time_taper +
		 		 ce_data->tier_stats[vbatt_idx].time_other;

		if (ce_data->tier_stats[vbatt_idx].temp_sum != 0 || elap == 0)
			t_avg = ce_data->tier_stats[vbatt_idx].temp_in;
		if (t_avg == 0)
			t_avg = 250;

		/* average temperature in tier for charge tier index */
		temp_idx = gbms_msc_temp_idx(profile, t_avg);
		pr_debug("%s %d: temp_idx=%d t_avg=%ld sum=%ld elap=%d\n",
			__func__, soc, temp_idx, t_avg,
			ce_data->tier_stats[vbatt_idx].temp_sum,
			elap);

		if (temp_idx < 0)
			return -EINVAL;
	}

	/* max tier demand for voltage tier at this temperature index */
	cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx) / 1000;
	/* statistical current demand for soc (<= cc_max) */
	avg_cc = ttf_ref_cc(stats, soc);
	if (avg_cc <= 0) {
		/* default to cc_max if we have no data */
		pr_debug("%s %d: demand use default avg_cc=%d->%d\n",
			__func__, soc, avg_cc, cc_max);
		avg_cc = cc_max;
	}

	/* statistical or reference max power demand for the tier at */
	pr_debug("%s %d:%d,%d: avg_cc=%d cc_max=%d\n",
		 __func__, soc, temp_idx, vbatt_idx,
		 avg_cc, cc_max);

	/* equivalent input current for adapter at vtier */
	equiv_icl = ttf_pwr_equiv_icl(ce_data, vbatt_idx, soc);
	if (equiv_icl <= 0) {
		pr_debug("%s %d: negative, null act_icl=%d\n",
			 __func__, soc, equiv_icl);
		return -EINVAL;
	}

	/* lower to cc_max if in HOT and COLD */
	if (cc_max < equiv_icl) {
		pr_debug("%s %d: reduce act_icl=%d to cc_max=%d\n",
			 __func__, soc, equiv_icl, cc_max);
		equiv_icl = cc_max;
	}

	/*
	 * This is the trick that makes everything work:
	 *   equiv_icl = min(act_icl, act_ibatt, cc_max)
	 *
	 * act_icl = adapter max or adapter actual icl (due to bad cable,
	 *           AC enabled or temperature shift) scaled to vtier
	 * act_ibatt = measured for
	 *   at reference temperature or actual < cc_max due to sysload
	 * cc_max = cc_max from profile (lower than ref for  HOT or COLD)
	 *
	 */

	/* ratio for elap time: it doesn't work if reference is not maximal */
	if (equiv_icl < avg_cc)
		ratio = (avg_cc * 100) / equiv_icl;
	else
		ratio = 100;

	pr_debug("%s %d: equiv_icl=%d, avg_cc=%d ratio=%d\n",
		 __func__, soc, equiv_icl, avg_cc, ratio);

	return ratio;
}

/* SOC estimates ---------------------------------------------------------  */

/* reference of current elap for a soc at max rate */
static int ttf_ref_elap(const struct batt_ttf_stats *stats, int soc)
{
	time_t elap;

	if (soc < 0 || soc >= 100)
		return 0;

	elap = stats->soc_stats.elap[soc];
	if (elap == 0)
		elap = stats->soc_ref.elap[soc];

	return elap;
}

/* elap time for a single soc% */
static int ttf_elap(time_t *estimate, const struct batt_ttf_stats *stats,
		    const struct gbms_charging_event *ce_data,
		    int soc)
{
	time_t elap;
	int ratio;

	/* cannot really return 0 elap unless the data is corrupted */
	elap = ttf_ref_elap(stats, soc);
	if (elap == 0) {
		pr_debug("%s %d: zero elap\n", __func__, soc);
		return -EINVAL;
	}

	ratio = ttf_pwr_ratio(stats, ce_data, soc);
	if (ratio < 0) {
		pr_debug("%s %d: negative ratio=%d\n", __func__, soc, ratio);
		return -EINVAL;
	}

	*estimate = elap * ratio;

	pr_debug("%s: soc=%d estimate=%ld elap=%ld ratio=%d\n",
		 __func__, soc, *estimate, elap, ratio);

	return 0;
}

/*
 * time to full from SOC% using the actual stats
 * NOTE: prediction is based stats and corrected with the ce_data
 * NOTE: usually called with soc > ce_data->last_soc
 */
int ttf_soc_estimate(time_t *res, const struct batt_ttf_stats *stats,
		     const struct gbms_charging_event *ce_data,
		     qnum_t soc, qnum_t last)
{
	const int ssoc_in = ce_data->charging_stats.ssoc_in;
	time_t elap, estimate = 0;
	int i = 0, ret, frac;

	if (last > qnum_rconst(100) || last < soc)
		return -EINVAL;

	if (last == soc) {
		*res = 0;
		return 0;
	}

	/* FIRST: 100 - first 2 digits of the fractional part of soc if any */
	frac = (int)qnum_nfracdgt(soc, 2);
	if (frac) {

		ret = ttf_elap(&elap, stats, ce_data, qnum_toint(soc));
		if (ret == 0)
			estimate += (elap * (100 - frac)) / 100;

		i += 1;
	}

	/* accumulate ttf_elap starting from i + 1 until end */
	for (i += qnum_toint(soc); i < qnum_toint(last); i++) {

		if (i >= ssoc_in && i < ce_data->last_soc) {
			/* use real data if within charging event */
			elap = ce_data->soc_stats.elap[i] * 100;
		} else {
			/* future (and soc before ssoc_in) */
			ret = ttf_elap(&elap, stats, ce_data, i);
			if (ret < 0)
				return ret;
		}

		estimate += elap;
	}

	/* LAST: first 2 digits of the fractional part of soc if any */
	frac = (int)qnum_nfracdgt(last, 2);
	if (frac) {
		ret = ttf_elap(&elap, stats, ce_data, qnum_toint(last));
		if (ret == 0)
			estimate += (elap * frac) / 100;
	}

	*res = estimate / 100;
	return 0;
}

int ttf_soc_cstr(char *buff, int size, const struct ttf_soc_stats *soc_stats,
		 int start, int end)
{
	int i, len = 0, split = 100;

	if (start < 0 || start >= GBMS_SOC_STATS_LEN ||
	    end < 0 || end >= GBMS_SOC_STATS_LEN ||
	    start > end)
		return 0;

	len += scnprintf(&buff[len], size - len, "\n");

	/* only one way to print data @ 100 */
	if (end == 100 && start != 100)
		end = 99;
	/* std newline every 10 entries */
	if (start == 0 && end == 99)
		split = 10;

	/* dump elap time as T: */
	for (i = start; i <= end; i++) {
		if (i % split == 0 || i == start) {
			len += scnprintf(&buff[len], size - len, "T");
			if (split == 10)
				len += scnprintf(&buff[len], size - len,
						"%d", i / 10);
			len += scnprintf(&buff[len], size - len, ":");
		}

		len += scnprintf(&buff[len], size - len, " %4ld",
				soc_stats->elap[i]);
		if (i != end && (i + 1) % split == 0)
			len += scnprintf(&buff[len], size - len, "\n");
	}

	len += scnprintf(&buff[len], size - len, "\n");

	/* dump coulumb count as C: */
	for (i = start; i <= end; i++) {
		if (i % split == 0 || i == start) {
			len += scnprintf(&buff[len], size - len, "C");
			if (split == 10)
				len += scnprintf(&buff[len], size - len,
						 "%d", i / 10);
			len += scnprintf(&buff[len], size - len, ":");
		}

		len += scnprintf(&buff[len], size - len, " %4d",
				soc_stats->cc[i]);
		if (i != end && (i + 1) % split == 0)
			len += scnprintf(&buff[len], size - len, "\n");
	}

	len += scnprintf(&buff[len], size - len, "\n");

	return len;
}

/* TODO: tune these values */

/* discard updates for adapters that have less than 80% of nominal */
#define TTF_SOC_QUAL_ELAP_RATIO_MAX	200
/* cap updates of cc to no more of +-*_CUR_ABS_MAX from previous */
#define TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX	60
/* cap udpdates to cc max to no more of +-20% of reference */
#define TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX	20

/* return the weight to apply to this change */
static time_t ttf_soc_qual_elap(const struct batt_ttf_stats *stats,
				const struct gbms_charging_event *ce_data,
				int i)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	const struct ttf_soc_stats *dst = &stats->soc_stats;
	const int limit = TTF_SOC_QUAL_ELAP_RATIO_MAX;
	const int max_elap = ((100 + TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX) *
			     stats->soc_ref.elap[i]) / 100;
	const int min_elap = ((100 - TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX) *
			     stats->soc_ref.elap[i]) / 100;
	time_t elap, elap_new, elap_cur;
	int ratio;

	if (!src->elap[i])
		return 0;

	/* weight the adapter, discard if ratio is too high (poor adapter) */
	ratio = ttf_pwr_ratio(stats, ce_data, i);
	if (ratio <= 0 || ratio > limit) {
		pr_debug("%d: ratio=%d limit=%d\n", i, ratio, limit);
		return 0;
	}

	elap_new = (src->elap[i] * 100) / ratio;
	elap_cur = dst->elap[i];
	if (!elap_cur)
		elap_cur = stats->soc_ref.elap[i];
	elap = (elap_cur + elap_new) / 2;

	/* bounds check to previous */
	if (elap > (elap_cur + TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX))
		elap = elap_cur + TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX;
	else if (elap < (elap_cur - TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX))
		elap = elap_cur - TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX;

	/* bounds check to reference */
	if (elap > max_elap)
		elap = max_elap;
	else if (elap < min_elap)
		elap = min_elap;

	pr_debug("%d: dst->elap=%ld, ref_elap=%ld, elap=%ld, src_elap=%ld ratio=%d, min=%d max=%d\n",
		i, dst->elap[i], stats->soc_ref.elap[i], elap, src->elap[i],
		ratio, min_elap, max_elap);

	return elap;
}

/* cap updates of cc to no more of +-*_CUR_ABS_MAX from previous */
#define TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX	40
/* cap udpdates to cc max to no more of +-20% of reference */
#define TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX	20

static int ttf_soc_qual_cc(const struct batt_ttf_stats *stats,
			   const struct gbms_charging_event *ce_data,
			   int i)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	const struct ttf_soc_stats *dst = &stats->soc_stats;
	const int max_cc = ((100 + TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX) *
			   stats->soc_ref.cc[i]) / 100;
	const int min_cc = ((100 - TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX) *
			   stats->soc_ref.cc[i]) / 100;
	int cc, cc_cur;

	if (!src->cc[i])
		return 0;

	cc_cur = dst->cc[i];
	if (cc_cur <= 0)
		cc_cur = stats->soc_ref.cc[i];

	cc = (cc_cur + src->cc[i]) / 2;

	/* bounds check to previous */
	if (cc > cc_cur + TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX)
		cc = cc_cur + TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX;
	else if (cc < cc_cur  -TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX)
		cc = cc_cur - TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX;

	/* bounds check to reference */
	if (cc > max_cc)
		cc = max_cc;
	else if (cc < min_cc)
		cc = min_cc;

	pr_info("%d: cc_cur=%d, ref_cc=%d src->cc=%d, cc=%d\n",
		i, cc_cur, stats->soc_ref.cc[i], src->cc[i], cc);

	return cc;
}

/* update soc_stats using the charging event
 * NOTE: first_soc and last_soc are inclusive, will skip socs that have no
 * elap and no cc.
 */
static void ttf_soc_update(struct batt_ttf_stats *stats,
			   const struct gbms_charging_event *ce_data,
			   int first_soc, int last_soc)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	int i;

	for (i = first_soc; i <= last_soc; i++) {
		time_t elap;
		int cc;

		/* need to have data on both */
		if (!src->elap[i] || !src->cc[i])
			continue;

		/* average the elap time at soc */
		elap = ttf_soc_qual_elap(stats, ce_data, i);
		if (elap)
			stats->soc_stats.elap[i] = elap;

		/* average the coulumb count at soc */
		cc = ttf_soc_qual_cc(stats, ce_data, i);
		if (cc)
			stats->soc_stats.cc[i] = cc;
	}
}

void ttf_soc_init(struct ttf_soc_stats *dst)
{
	memset(dst, 0, sizeof(*dst));
}

/* Tier estimates ---------------------------------------------------------  */

#define TTF_STATS_FMT "[%d,%d %d %ld]"

#define BATT_TTF_TS_VALID(ts) \
	(ts->cc_total != 0 && ts->avg_time != 0)

/* TODO: adjust for adapter capability */
static time_t ttf_tier_accumulate(const struct ttf_tier_stat *ts,
				  int vbatt_idx,
				  const struct batt_ttf_stats *stats)
{
	time_t estimate = 0;

	if (vbatt_idx >= GBMS_STATS_TIER_COUNT)
		return 0;

	for (; vbatt_idx < GBMS_STATS_TIER_COUNT; vbatt_idx++) {

		/* no data in this tier, sorry */
		if (!BATT_TTF_TS_VALID(ts))
			return -ENODATA;

		estimate += ts[vbatt_idx].avg_time;
	}

	return estimate;
}

/* */
static int ttf_tier_sscan(struct batt_ttf_stats *stats,
			  const char *buff,
			  size_t size)
{
	int i, j, t, cnt, len = 0;

	memset(&stats->tier_stats, 0, sizeof(*stats));

	cnt = sscanf(&buff[len], "%d:", &t);
	if (t != i)
		i = t - 1;
	while (buff[len] != '[' && len < size)
		len++;

	for (j = 0; j < GBMS_STATS_TIER_COUNT; j++) {
		cnt = sscanf(&buff[len], TTF_STATS_FMT,
			&stats->tier_stats[j].soc_in,
			&stats->tier_stats[j].cc_in,
			&stats->tier_stats[j].cc_total,
			&stats->tier_stats[j].avg_time);

		len += sizeof(TTF_STATS_FMT) - 1;
	}

	return 0;
}

int ttf_tier_cstr(char *buff, int size, const struct ttf_tier_stat *tier_stats)
{
	int len = 0;

	len += scnprintf(&buff[len], size - len,
			 TTF_STATS_FMT,
			 tier_stats->soc_in >> 8,
			 tier_stats->cc_in,
			 tier_stats->cc_total,
			 tier_stats->avg_time);
	return len;
}

/* average soc_in, cc_in, cc_total an and avg time for charge tier */
static void ttf_tier_update_stats(struct ttf_tier_stat *ttf_ts,
			    	  const struct gbms_ce_tier_stats *chg_ts,
				  bool force)
{
	int elap;

	if (!force) {
		if (chg_ts->cc_total == 0)
			return;

		/* TODO: check dsg, qualify with adapter? */
	}

	/* TODO: check with -1 */
	if (ttf_ts->soc_in == 0)
		ttf_ts->soc_in = chg_ts->soc_in;
	 ttf_ts->soc_in = (ttf_ts->soc_in + chg_ts->soc_in) / 2;

	/* TODO: check with -1 */
	if (ttf_ts->cc_in == 0)
		ttf_ts->cc_in = chg_ts->cc_in;
	 ttf_ts->cc_in = (ttf_ts->cc_in + chg_ts->cc_in) / 2;

	if (ttf_ts->cc_total == 0)
		 ttf_ts->cc_total = chg_ts->cc_total;
	 ttf_ts->cc_total = (ttf_ts->cc_total + chg_ts->cc_total) / 2;

	/* */
	elap = chg_ts->time_fast + chg_ts->time_taper + chg_ts->time_other;
	if (ttf_ts->avg_time == 0)
		ttf_ts->avg_time = elap;

	/* qualify time with ratio */
	ttf_ts->avg_time =(ttf_ts->avg_time + elap) / 2;
}

/* updated tier stats using the charging event
 * NOTE: the ce has data from 1+ charging voltage and temperature tiers */
static void ttf_tier_update(struct batt_ttf_stats *stats,
			    const struct gbms_charging_event *data,
			    bool force)
{
	int i;

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const bool last_tier = i == (GBMS_STATS_TIER_COUNT - 1);
		const struct gbms_ce_tier_stats *chg_ts = &data->tier_stats[i];
		const struct gbms_ce_stats *chg_s = &data->charging_stats;
		long elap;

		/* skip data that has a temperature switch */
		if (chg_ts->temp_idx == -1)
			continue;
		/* or entries that have no actual charging */
		elap = chg_ts->time_fast + chg_ts->time_taper;
		if (!elap)
			continue;
		/* update first tier stats only at low soc_in */
		if (!force && i == 0 && (chg_ts->soc_in >> 8) > 1)
			continue;
		/* update last tier stats only at full */
		if (!force && last_tier && ((chg_s->ssoc_out >> 8) != 100))
			continue;

		/*  */
		ttf_tier_update_stats(&stats->tier_stats[i], chg_ts, false);
	}

}

/* tier estimates only, */
int ttf_tier_estimate(time_t *res, const struct batt_ttf_stats *stats,
		      int temp_idx, int vbatt_idx,
		      int capacity, int full_capacity)
{
	time_t estimate = 0;
	const struct ttf_tier_stat *ts;

	/* tier estimates, only when in tier */
	if (vbatt_idx == -1 && temp_idx == -1)
		return -EINVAL;

	ts = &stats->tier_stats[vbatt_idx];
	if (!ts || !BATT_TTF_TS_VALID(ts))
		return -ENODATA;

	/* accumulate next tier */
	estimate = ttf_tier_accumulate(ts, vbatt_idx + 1, stats);
	if (estimate < 0)
		return -ENODATA;

	/* eyeball current tier
	 * estimate =
	 * 	(ts->cc_in + ts->cc_total - capacity) *
	 * 	rs->avg_time) / ts->cc_total
	 */

	/* TODO: adjust for crossing thermals? */

	*res = estimate;
	return 0;
}

/* ----------------------------------------------------------------------- */

/* QUAL DELTA >= 3 */
#define TTF_STATS_QUAL_DELTA_MIN	3
#define TTF_STATS_QUAL_DELTA		TTF_STATS_QUAL_DELTA_MIN

static int ttf_soc_cstr_elap(char *buff, int size,
			     const struct ttf_soc_stats *soc_stats,
			     int start, int end)
{
	int i, len = 0;

	len += scnprintf(&buff[len], size - len, "T%d:", start);
	for (i = start; i < end; i++)
		len += scnprintf(&buff[len], size - len, " %4ld",
				 soc_stats->elap[i]);

	return len;
}

static int ttf_soc_cstr_cc(char *buff, int size,
			   const struct ttf_soc_stats *soc_stats,
			   int start, int end)
{
	int i, len = 0;

	len += scnprintf(&buff[len], size - len, "C%d:", start);
	for (i = start; i < end; i++)
		len += scnprintf(&buff[len], size - len, " %4d",
				 soc_stats->cc[i]);

	return len;
}

/* update ttf tier and soc stats using the charging event.
 * call holding stats->lock
 */
void ttf_stats_update(struct batt_ttf_stats *stats,
		      struct gbms_charging_event *ce_data,
		      bool force)
{
	int first_soc = ce_data->charging_stats.ssoc_in;
	const int last_soc = ce_data->last_soc;
	const int delta_soc = last_soc - first_soc;
	const int limit = force ? TTF_STATS_QUAL_DELTA_MIN :
			  TTF_STATS_QUAL_DELTA;
	const int tmp_size = PAGE_SIZE;
	char *tmp;

	/* skip data short periods */
	if (delta_soc < limit) {
		ttf_log(stats, "no updates delta_soc=%d, limit=%d, force=%d",
			delta_soc, limit, force);
		return;
	}

	/* ignore first nozero and last entry because they are partial */
	for ( ; first_soc <= last_soc; first_soc++)
		if (ce_data->soc_stats.elap[first_soc] != 0)
			break;

	ttf_soc_update(stats, ce_data, first_soc + 1, last_soc - 1);
	ttf_tier_update(stats, ce_data, force);

	/* dump update stats to logbuffer */
	tmp = kzalloc(tmp_size, GFP_KERNEL);
	if (tmp) {
		const int split = 10;
		int i;

		for (i = first_soc + 1;i < last_soc - 1; i += split) {
			int end_soc = i + split;

			if (end_soc > last_soc - 1)
				end_soc = last_soc - 1;

			ttf_soc_cstr_elap(tmp, tmp_size, &stats->soc_stats,
					  i, end_soc);
			ttf_log(stats, "%s", tmp);
			ttf_soc_cstr_cc(tmp, tmp_size, &stats->soc_stats,
					  i,  end_soc);
			ttf_log(stats, "%s", tmp);
		}

		kfree(tmp);
	}
}

static int ttf_init_soc_parse_dt(struct ttf_adapter_stats *as,
				 struct device *device)
{
	int table_count;
	int ret;

	table_count = of_property_count_elems_of_size(device->of_node,
						      "google,ttf-soc-table",
						      sizeof(u32));
	if (table_count <= 0)
		return 0;
	if (table_count % 2)
		return -EINVAL;

	as->soc_table = devm_kzalloc(device, table_count * 2 * sizeof(u32),
				     GFP_KERNEL);
	if (!as->soc_table)
		return -ENOMEM;

	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-soc-table",
					as->soc_table, table_count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-soc-table %d\n", ret);
		return ret;
	}

	as->elap_table = &as->soc_table[table_count];
	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-elap-table",
					as->elap_table, table_count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-elap-table %d\n", ret);
		return ret;
	}

	as->table_count = table_count;
	return 0;
}

int ttf_stats_sscan(struct batt_ttf_stats *stats,
		    const char *buff,
		    size_t size)
{
	/* TODO: scan ttf_soc_* data as well */

	return ttf_tier_sscan(stats, buff, size);
}

static int ttf_as_default(struct ttf_adapter_stats *as, int i, int table_i)
{
	while (i > as->soc_table[table_i] && table_i < as->table_count)
		table_i++;

	return table_i;
}

static int ttf_init_tier_parse_dt(struct batt_ttf_stats *stats,
				  struct device *device)
{
	int i, count, ret;
	u32 tier_table[GBMS_STATS_TIER_COUNT];

	if (!device)
		return -ENODEV;

	count = of_property_count_elems_of_size(device->of_node,
						"google,ttf-tier-table",
						sizeof(u32));
	if (count != GBMS_STATS_TIER_COUNT)
		return -EINVAL;

	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-tier-table",
					tier_table, count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-tier-table %d\n", ret);
		return ret;
	}

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++)
		stats->tier_stats[i].soc_in = tier_table[i] << 8;

	return 0;
}

/* clone and clear the stats */
struct batt_ttf_stats *ttf_stats_dup(struct batt_ttf_stats *dst,
				     const struct batt_ttf_stats *src)
{
	memcpy(dst, src, sizeof(*dst));
	memset(&dst->soc_stats, 0, sizeof(dst->soc_stats));
	memset(&dst->tier_stats, 0, sizeof(dst->tier_stats));
	return dst;
}

/*
 * TODO: need to be adjusted to more complex scenarios (adaptive charging,
 * slow top off) that have more tiers, this implementation only fill 3
 * tiers.
 */
static void ttf_stats_init_tier(struct batt_ttf_stats *stats,
				int capacity_ma)
{

	/* TODO: use the soc stats to calculate cc_in */
	stats->tier_stats[0].cc_in = 0;
	stats->tier_stats[1].cc_in = (capacity_ma *
					(stats->tier_stats[1].soc_in >> 8)) /
					100;
	stats->tier_stats[2].cc_in = (capacity_ma *
					(stats->tier_stats[2].soc_in >> 8)) /
					100;

	/* TODO: use the soc stats to calculate cc_total */
	stats->tier_stats[0].cc_total = (capacity_ma *
					((stats->tier_stats[1].soc_in -
					stats->tier_stats[0].soc_in) >> 8)) /
					100;
	stats->tier_stats[1].cc_total = (capacity_ma *
					((stats->tier_stats[2].soc_in -
					stats->tier_stats[1].soc_in) >> 8)) /
					100;
	stats->tier_stats[2].cc_total = capacity_ma -
					stats->tier_stats[2].cc_in;
}

/*
 * must come after charge profile
 * TODO: tier statistics need the charge profile
 */
int ttf_stats_init(struct batt_ttf_stats *stats,
		   struct device *device,
		   int capacity_ma)
{
	u32 value;
	int i, soc, ret;
	struct ttf_adapter_stats as;

	memset(stats, 0, sizeof(*stats));
	stats->ttf_fake = -1;

	ret = of_property_read_u32(device->of_node, "google,ttf-adapter",
				   &value);
	if (ret < 0)
		value = 0;
	stats->ref_watts = value;

	ret = of_property_read_u32(device->of_node, "google,ttf-temp-idx",
				   &value);
	if (ret < 0)
		value = 2;
	stats->ref_temp_idx = value;

	/* initialize reference soc estimates */
	/* TODO: allocate as->soc_table witk kzalloc, free here */

	ret = ttf_init_soc_parse_dt(&as, device);
	if (ret == 0) {
		int table_i = 0;
		const int cc = (capacity_ma * 100) / GBMS_SOC_STATS_LEN;

		for (i = 0; i < GBMS_SOC_STATS_LEN; i++) {
			table_i = ttf_as_default(&as, i, table_i);

			stats->soc_ref.elap[i] = as.elap_table[table_i];

			/* assume same cc for each soc */
			stats->soc_ref.cc[i] = (cc * i) / 100;
		}

	}

	/* TODO: free as->soc_table here  */

	/* tier estimates, avg time is filled from ref SOC */
	ret = ttf_init_tier_parse_dt(stats, device);
	if (ret < 0) {
		stats->tier_stats[0].soc_in = 0;
		stats->tier_stats[1].soc_in = 66 << 8;
		stats->tier_stats[2].soc_in = 80 << 8;
	}

	ttf_stats_init_tier(stats, capacity_ma);

	/* compute ref average time */
	for (i = 0; i < 2; i++) {
		const int soc_in = stats->tier_stats[i].soc_in >> 8;
		const int soc_out = stats->tier_stats[i + 1].soc_in >> 8;

		for (soc = soc_in;soc < soc_out; soc++)
			stats->tier_stats[i].avg_time += stats->soc_ref.elap[soc];
	}

	for ( ;soc < 100; soc++)
		stats->tier_stats[i].avg_time += stats->soc_ref.elap[soc];

	return 0;
}


/* tier and soc details */
ssize_t ttf_dump_details(char *buf, int max_size,
			 const struct batt_ttf_stats *ttf_stats,
			 int last_soc)
{
	int i, len = 0;

	/* interleave tier with SOC data */
	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		int next_soc_in;

		len += scnprintf(&buf[len], max_size - len, "%d: ", i);
		len += ttf_tier_cstr(&buf[len], max_size - len,
				     &ttf_stats->tier_stats[i]);
		len += scnprintf(&buf[len], max_size - len, "\n");

		/* continue only first */
		if (ttf_stats->tier_stats[i].avg_time == 0)
			continue;

		if (i == GBMS_STATS_TIER_COUNT - 1) {
			next_soc_in = -1;
		} else {
			next_soc_in = ttf_stats->tier_stats[i + 1].soc_in >> 8;
			if (next_soc_in == 0)
				next_soc_in = -1;
		}

		if (next_soc_in == -1)
			next_soc_in = last_soc - 1;

		len += ttf_soc_cstr(&buf[len], max_size - len,
				    &ttf_stats->soc_stats,
				    ttf_stats->tier_stats[i].soc_in >> 8,
				    next_soc_in);
	}

	return len;
}
