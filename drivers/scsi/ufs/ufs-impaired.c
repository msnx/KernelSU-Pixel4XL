/*
 * UFS impaired storage emulation.
 *
 * Copyright (C) 2019, Google, LLC.
 *
 * Authors:
 *	Hyojun Kim <hyojun@google.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#include <linux/random.h>
#include "ufshcd.h"

#define UFS_IMPAIRED_SLEEP_US			(500)
#define UFS_IMPAIRED_ERROR_US			(10)

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL

/**
 *  Latency distribution model for small (<128KiB) read requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_small_read[] = {
	82, 86, 87, 92, 97, 100, 101, 103,
	105, 109, 111, 114, 119, 124, 129, 131,
	133, 135, 135, 136, 139, 141, 143, 146,
	147, 149, 152, 154, 157, 160, 164, 168,
	171, 175, 176, 179, 181, 183, 185, 187,
	190, 192, 195, 197, 200, 203, 205, 208,
	211, 213, 216, 218, 221, 224, 227, 230,
	234, 237, 240, 243, 246, 250, 253, 256,
	261, 264, 267, 272, 275, 280, 283, 286,
	289, 293, 296, 300, 304, 307, 312, 315,
	318, 323, 326, 329, 332, 337, 340, 344,
	348, 352, 358, 361, 366, 371, 375, 381,
	386, 391, 395, 399, 402, 407, 410, 414,
	420, 425, 430, 434, 439, 444, 450, 455,
	460, 466, 474, 480, 485, 491, 497, 500,
	503, 508, 512, 516, 520, 524, 530, 534,
	539, 544, 547, 549, 551, 554, 555, 559,
	563, 567, 570, 573, 577, 579, 584, 589,
	593, 595, 598, 602, 605, 609, 613, 616,
	620, 624, 627, 630, 634, 638, 643, 647,
	653, 657, 662, 667, 671, 675, 679, 683,
	689, 694, 700, 706, 712, 716, 721, 727,
	734, 741, 749, 756, 763, 769, 777, 783,
	792, 798, 806, 813, 822, 830, 841, 850,
	859, 867, 876, 885, 896, 906, 922, 935,
	955, 973, 987, 1010, 1024, 1039, 1057, 1075,
	1092, 1111, 1137, 1160, 1185, 1210, 1234, 1264,
	1293, 1323, 1376, 1417, 1483, 1536, 1612, 1725,
	1809, 1898, 2016, 2136, 2269, 2527, 2781, 2943,
	3101, 3365, 3752, 4385, 4844, 5484, 6336, 7677,
	8957, 10130, 10930, 11191, 11346, 11438, 11518, 11581,
	11624, 11672, 11711, 11757, 11803, 11900, 17332, 24887,
};

/**
 *  Latency distribution model for large (>=128KiB) read requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_large_read[] = {
	330, 364, 369, 371, 372, 374, 374, 375,
	377, 378, 379, 380, 380, 382, 387, 392,
	398, 403, 406, 411, 415, 418, 425, 428,
	439, 448, 453, 455, 460, 462, 465, 466,
	468, 470, 471, 473, 474, 476, 476, 477,
	477, 479, 479, 481, 481, 482, 482, 483,
	484, 484, 485, 485, 485, 487, 488, 489,
	490, 490, 492, 493, 493, 495, 496, 498,
	500, 501, 503, 506, 508, 509, 511, 512,
	514, 516, 517, 519, 520, 522, 524, 525,
	527, 528, 530, 531, 532, 533, 534, 535,
	538, 540, 543, 546, 549, 554, 556, 562,
	567, 570, 572, 575, 579, 582, 586, 589,
	595, 597, 605, 610, 615, 624, 630, 637,
	641, 654, 659, 665, 670, 680, 689, 700,
	710, 713, 720, 724, 731, 733, 737, 740,
	745, 751, 759, 763, 772, 778, 783, 790,
	796, 799, 812, 818, 821, 828, 833, 844,
	850, 858, 863, 874, 880, 884, 887, 890,
	892, 893, 896, 898, 900, 901, 901, 903,
	904, 908, 912, 916, 919, 925, 928, 930,
	935, 939, 944, 947, 950, 957, 966, 971,
	978, 983, 989, 1000, 1006, 1017, 1031, 1041,
	1056, 1071, 1091, 1110, 1127, 1148, 1176, 1205,
	1231, 1271, 1287, 1322, 1371, 1452, 1498, 1541,
	1601, 1631, 1725, 1835, 1895, 2008, 2115, 2187,
	2265, 2433, 2592, 2700, 2916, 3356, 4245, 5134,
	6602, 7214, 8466, 9078, 9587, 9974, 10210, 10489,
	10668, 10881, 11043, 11169, 11232, 11295, 11352, 11386,
	11432, 11470, 11535, 11582, 11626, 11654, 11685, 11700,
	11720, 11747, 11778, 11792, 11801, 11818, 11838, 11854,
	11873, 11886, 11925, 11975, 12055, 12146, 12334, 22860,
};
/**
 *  Latency distribution model for small (<128KiB) write requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_small_write[] = {
	29, 30, 30, 31, 31, 31, 31, 32,
	33, 33, 33, 34, 34, 35, 35, 35,
	35, 36, 36, 36, 36, 36, 36, 37,
	38, 38, 38, 38, 38, 38, 39, 39,
	39, 39, 40, 40, 40, 40, 41, 41,
	41, 41, 41, 43, 43, 43, 43, 43,
	44, 44, 44, 44, 45, 45, 46, 46,
	46, 47, 47, 47, 48, 49, 49, 50,
	50, 50, 51, 52, 52, 53, 54, 54,
	55, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 62, 63, 65, 65, 66, 68,
	70, 70, 72, 73, 74, 76, 78, 79,
	82, 84, 86, 88, 90, 93, 95, 97,
	101, 104, 108, 110, 114, 117, 122, 125,
	130, 135, 141, 144, 150, 156, 164, 169,
	179, 184, 195, 203, 212, 221, 230, 240,
	248, 254, 263, 276, 289, 292, 293, 294,
	296, 296, 297, 299, 301, 303, 310, 316,
	344, 361, 379, 393, 406, 420, 427, 434,
	439, 444, 447, 449, 450, 452, 453, 455,
	455, 458, 460, 463, 468, 473, 477, 487,
	495, 501, 513, 520, 528, 543, 559, 570,
	584, 600, 626, 648, 662, 684, 711, 738,
	780, 868, 938, 963, 979, 1006, 1042, 1068,
	1260, 1472, 1516, 1580, 1661, 1851, 1981, 2129,
	2287, 2395, 2500, 2618, 2776, 2923, 3041, 3181,
	3639, 3812, 4036, 4157, 4269, 4471, 5030, 5567,
	5684, 5877, 6524, 7319, 8355, 10354, 11655, 11894,
	11988, 12047, 12101, 12168, 12227, 12254, 12276, 12303,
	12332, 12356, 12381, 12413, 12434, 12467, 12501, 12521,
	12544, 12571, 12615, 12650, 12671, 12762, 13325, 13590,
	14783, 15871, 17882, 19427, 21068, 27529, 83074, 248830,
};

/**
 *  Latency distribution model for large (>=128KiB) write requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_large_write[] = {
	291, 481, 1226, 1497, 1503, 1570, 1712, 1834,
	1873, 2337, 2849, 3093, 3118, 3166, 3185, 3259,
	3297, 3397, 3501, 3787, 4090, 4441, 4622, 4746,
	4844, 4988, 5068, 5170, 5409, 5586, 6021, 6150,
	6306, 6382, 6481, 6564, 6650, 6828, 7096, 7462,
	7948, 8014, 8131, 8162, 8316, 8570, 9290, 9480,
	9619, 9737, 9801, 10357, 10901, 11134, 11244, 11306,
	11403, 11550, 11910, 12032, 12611, 12722, 12768, 12837,
	12875, 12991, 13165, 13357, 13633, 13849, 14087, 14207,
	14361, 14508, 14607, 14687, 15307, 15558, 15639, 15855,
	15992, 16088, 16345, 16762, 17297, 17399, 17456, 17641,
	17701, 17998, 18339, 18543, 18721, 18868, 18984, 19108,
	19182, 19632, 20091, 20386, 20495, 21096, 21604, 22056,
	22278, 22640, 23360, 23855, 24343, 25066, 25383, 25671,
	26307, 26769, 27191, 28283, 28976, 30133, 30601, 31160,
	32555, 33402, 33693, 34603, 34854, 35316, 35993, 37728,
	38967, 39335, 40294, 40894, 42723, 43572, 45818, 46702,
	47478, 49183, 50503, 53104, 54147, 55234, 56557, 59641,
	60248, 62231, 63775, 66050, 68360, 70005, 72997, 74604,
	76439, 77722, 80372, 83042, 86060, 90752, 93454, 103156,
	109817, 118608, 127492, 135836, 145875, 155638, 164700, 174263,
	183504, 193366, 195222, 195641, 195909, 196086, 196213, 196425,
	197304, 197694, 198115, 198639, 199147, 199757, 202389, 206022,
	207042, 207830, 207954, 208561, 208891, 209069, 209483, 210002,
	210468, 210678, 211172, 211454, 211650, 212053, 212276, 212391,
	212611, 212816, 212974, 213152, 213401, 213574, 213931, 214327,
	214579, 214715, 214839, 215011, 215651, 215992, 216284, 216459,
	216846, 217005, 217542, 217864, 218358, 218890, 219708, 220049,
	220675, 221644, 222866, 223864, 224662, 225728, 226457, 227112,
	228230, 229819, 231050, 233468, 234945, 236253, 237395, 237959,
	238911, 240856, 242108, 244934, 246439, 248019, 258456, 278881,
	317887, 404510, 452795, 480299, 537260, 556246, 567101, 575240,
};

#define MODEL_SIZE	(sizeof(impaired_latency_large_write) / sizeof(int))
#define REQSZ_THRESHOLD	(128 * 1024)

#endif

/**
 *  Impaired storage sysfs entries
 */
struct impaired_attr {
	struct device_attribute attr;
};

struct impaired_delay_attr {
	struct device_attribute attr;
	enum ufs_impaired_optype optype;
	enum ufs_impaired_delaytype delaytype;
	u32 maximum;
};

struct impaired_stat_attr {
	struct device_attribute attr;
	enum ufs_impaired_optype optype;
	enum ufs_impaired_stattype stattype;
};

static ssize_t
impaired_show_enabled(struct device *dev, struct device_attribute *_attr,
		char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", hba->impaired.enabled);
}

static int __impaired_thread_fn(void *param);

static ssize_t
impaired_store_enabled(struct device *dev, struct device_attribute *_attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags, value;

	if (kstrtol(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&hba->impaired_thread_mutex);
	if (value && !hba->impaired_thread)
		hba->impaired_thread =
			kthread_run(__impaired_thread_fn, hba, "impaired");
	mutex_unlock(&hba->impaired_thread_mutex);

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->impaired.enabled = (value) ? 1 : 0;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

static ssize_t
impaired_show_delay(struct device *dev, struct device_attribute *_attr,
		char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct impaired_delay_attr *attr = (struct impaired_delay_attr *)_attr;
	u32 value = hba->impaired.io[attr->optype].delay[attr->delaytype];

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	if (attr->delaytype == UFS_IMPAIRED_DELAYTYPE_MODEL) {
		return snprintf(buf, PAGE_SIZE,
				(value) ? "fragftl\n" : "none\n");
	}
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t
impaired_store_delay(struct device *dev, struct device_attribute *_attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct impaired_delay_attr *attr = (struct impaired_delay_attr *)_attr;
	unsigned long flags, value;

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	if (attr->delaytype == UFS_IMPAIRED_DELAYTYPE_MODEL) {
		if (strncmp(buf, "none", 4) == 0)
			value = 0;
		else if (strncmp(buf, "fragftl", 7) == 0)
			value = 1;
		else
			return -EINVAL;
	} else if (kstrtol(buf, 0, &value)) {
		return -EINVAL;
	}
#else
	if (kstrtol(buf, 0, &value))
		return -EINVAL;
#endif

	if (value > attr->maximum)
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->impaired.io[attr->optype].delay[attr->delaytype] = (u32)value;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

static ssize_t
impaired_show_stat(struct device *dev, struct device_attribute *_attr,
		char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct impaired_stat_attr *attr = (struct impaired_stat_attr *)_attr;

	return snprintf(buf, PAGE_SIZE, "%lld\n",
			hba->impaired.io[attr->optype].stat[attr->stattype]);
}

static ssize_t
impaired_store_stat(struct device *dev, struct device_attribute *_attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct impaired_stat_attr *attr = (struct impaired_stat_attr *)_attr;
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	memset(&hba->impaired.io[attr->optype].stat, 0,
			sizeof(hba->impaired.io[attr->optype].stat));
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

#define IMPAIRED_ATTR_RW(_name) 					\
	static struct impaired_attr impaired_##_name = {		\
		.attr = __ATTR(_name, 0644, impaired_show_##_name, 	\
				impaired_store_##_name),		\
	}

#define IMPAIRED_DELAY_ATTR(_name, _delay, _maximum)			\
	static struct impaired_delay_attr 				\
	impaired_read_##_name = { 					\
		.attr = __ATTR(read_##_name, 0644, impaired_show_delay,	\
				impaired_store_delay),			\
		.optype = UFS_IMPAIRED_OPTYPE_READ,			\
		.delaytype = _delay,					\
		.maximum = _maximum,					\
	};								\
	static struct impaired_delay_attr 				\
	impaired_write_##_name = { 					\
		.attr = __ATTR(write_##_name, 0644, impaired_show_delay,\
				impaired_store_delay),			\
		.optype = UFS_IMPAIRED_OPTYPE_WRITE,			\
		.delaytype = _delay,					\
		.maximum = _maximum,					\
	}

#define IMPAIRED_STAT_ATTR(_name, _stat)				\
	static struct impaired_stat_attr 				\
	impaired_read_##_name = { 					\
		.attr = __ATTR(read_##_name, 0644, impaired_show_stat,	\
				impaired_store_stat),			\
		.optype = UFS_IMPAIRED_OPTYPE_READ,			\
		.stattype = _stat,					\
	};								\
	static struct impaired_stat_attr 				\
	impaired_write_##_name = { 					\
		.attr = __ATTR(write_##_name, 0644, impaired_show_stat,	\
				impaired_store_stat),			\
		.optype = UFS_IMPAIRED_OPTYPE_WRITE,			\
		.stattype = _stat,					\
	}

/* Basic control */
IMPAIRED_ATTR_RW(enabled);

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
IMPAIRED_DELAY_ATTR(model, UFS_IMPAIRED_DELAYTYPE_MODEL, 1);
#endif

IMPAIRED_DELAY_ATTR(delay_us, UFS_IMPAIRED_DELAYTYPE_US, 1000000);
IMPAIRED_DELAY_ATTR(delay_percent, UFS_IMPAIRED_DELAYTYPE_PERCENT, 100000);

/* Advanced_control */
IMPAIRED_DELAY_ATTR(max_delayed_us, UFS_IMPAIRED_DELAYTYPE_MAX, 2000000);
IMPAIRED_DELAY_ATTR(skip_delay_cnt, UFS_IMPAIRED_DELAYTYPE_SKIP, 2000000);

/* Statistics */
IMPAIRED_STAT_ATTR(total_cnt, UFS_IMPAIRED_STATTYPE_TOTAL_CNT);
IMPAIRED_STAT_ATTR(total_original_us, UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US);
IMPAIRED_STAT_ATTR(total_delayed_us, UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US);

/* Delay related statistics */
IMPAIRED_STAT_ATTR(max_delay_us, UFS_IMPAIRED_STATTYPE_MAX_DELAY);
IMPAIRED_STAT_ATTR(max_delay_error_us, UFS_IMPAIRED_STATTYPE_MAX_ERROR);
IMPAIRED_STAT_ATTR(total_delay_error_us, \
		UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR);

static struct attribute *impaired_sysfs_attrs[] = {
	/* Basic delay control */
	&impaired_enabled.attr.attr,

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	&impaired_read_model.attr.attr,
#endif
	&impaired_read_delay_us.attr.attr,
	&impaired_read_delay_percent.attr.attr,

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	&impaired_write_model.attr.attr,
#endif
	&impaired_write_delay_us.attr.attr,
	&impaired_write_delay_percent.attr.attr,

	/* Advanced control */
	&impaired_read_max_delayed_us.attr.attr,
	&impaired_read_skip_delay_cnt.attr.attr,

	&impaired_write_max_delayed_us.attr.attr,
	&impaired_write_skip_delay_cnt.attr.attr,

	/* Statistics */
	&impaired_read_total_cnt.attr.attr,
	&impaired_read_total_original_us.attr.attr,
	&impaired_read_total_delayed_us.attr.attr,

	&impaired_write_total_cnt.attr.attr,
	&impaired_write_total_original_us.attr.attr,
	&impaired_write_total_delayed_us.attr.attr,

	/* Delay related statistics */
	&impaired_read_max_delay_us.attr.attr,
	&impaired_read_max_delay_error_us.attr.attr,
	&impaired_read_total_delay_error_us.attr.attr,

	&impaired_write_max_delay_us.attr.attr,
	&impaired_write_max_delay_error_us.attr.attr,
	&impaired_write_total_delay_error_us.attr.attr,

	NULL
};

static const struct attribute_group impaired_sysfs_group = {
	.name = "impaired",
	.attrs = impaired_sysfs_attrs,
};

void ufs_impaired_init_sysfs(struct ufs_hba *hba)
{
	if (sysfs_create_group(&hba->dev->kobj, &impaired_sysfs_group))
		dev_err(hba->dev, "Failed to create impaired storage sysfs group\n");
}

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
/**
 * __pick_target_latency_with_model
 *   - pick a latency for a UFS request by using latency distribution models.
 * @op: operation type
 * @req_size: request size in bytes
 */
static int __pick_target_latency_with_model(
		enum ufs_impaired_optype op, u32 req_size)
{
	unsigned int idx;
	int *model;

	/* Pick a model based on request type and size */
	if (op == UFS_IMPAIRED_OPTYPE_READ)
		model = (req_size < REQSZ_THRESHOLD) ?
			impaired_latency_small_read :
			impaired_latency_large_read;
	else
		model = (req_size < REQSZ_THRESHOLD) ?
			impaired_latency_small_write :
			impaired_latency_large_write;

	/* Generate random index and pick a target latency */
	prandom_bytes(&idx, sizeof(idx));
	return model[idx % MODEL_SIZE];
}
#endif

/**
 * __set_delay_in_lrb - pick a target delay and set lrbp->complete_time and
 *                      lrbp->target_compete_time fields.
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to add delay
 */
void __set_delay_in_lrb(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct request *rq = (cmd) ? cmd->request : NULL;
	s32 actual_us = ktime_us_delta(lrbp->complete_time_stamp,
				lrbp->issue_time_stamp);
	struct ufs_impaired_io *io;
	enum ufs_impaired_optype op;
	s32 target_us;

	/* Set default value: no delay */
	lrbp->complete_delay = 0;
	lrbp->target_complete_time = lrbp->complete_time_stamp;

	/* Return if delay is not needed */
	if (!hba->impaired.enabled ||
			!rq ||
			(rq->cmd_flags & REQ_PREFLUSH) ||
			req_op(rq) == REQ_OP_FLUSH ||
			req_op(rq) == REQ_OP_DISCARD ||
			!rq->bio)
		return;

	op = (rq_data_dir(rq) == READ) ?
		UFS_IMPAIRED_OPTYPE_READ : UFS_IMPAIRED_OPTYPE_WRITE;
	io = &hba->impaired.io[op];

	/* Apply skip logic: delay one request after N skipping */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_SKIP] > io->skip_count++)
		return;
	io->skip_count = 0;

	/* Set default target latency */
	target_us = actual_us;

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	/* If fragmented FTL emulation is on,
	   call __pick_target_latency_with_model() to pick a target latency */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_MODEL])
		target_us = __pick_target_latency_with_model(op,
				scsi_get_bytes(lrbp->cmd));
#endif

	/* Apply additional knobs */
	target_us = target_us * io->delay[UFS_IMPAIRED_DELAYTYPE_PERCENT] / 100;
	target_us += io->delay[UFS_IMPAIRED_DELAYTYPE_US];

	/* Apply maximum latency */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_MAX] &&
			target_us > io->delay[UFS_IMPAIRED_DELAYTYPE_MAX])
		target_us = io->delay[UFS_IMPAIRED_DELAYTYPE_MAX];

	/* Target latency cannot be smaller than the actual latency */
	if (target_us < actual_us)
		target_us = actual_us;

	/* Update delay related statistics */
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_CNT] += 1;
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US] += actual_us;

	/* Delayed time will be added later when the request is completed */
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US] += actual_us;

	/* Set delay fields */
	lrbp->complete_delay = (int)(target_us - actual_us);
	lrbp->target_complete_time = ktime_add_us(lrbp->complete_time_stamp,
			lrbp->complete_delay);

	/* Keep track of the maximum delay */
	if (io->stat[UFS_IMPAIRED_STATTYPE_MAX_DELAY] < lrbp->complete_delay)
		io->stat[UFS_IMPAIRED_STATTYPE_MAX_DELAY] =
			lrbp->complete_delay;
}


/**
 * __queue_lrb - put a lrb in to the delay list
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to be queued
 */
void __queue_lrb(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct list_head *parent;
	struct ufshcd_lrb *l;

	/* Find insertion point in the sorted list by target_complete_time */
	parent = &hba->impaired_list_head;
	list_for_each_entry(l, &hba->impaired_list_head, list) {
		if (ktime_compare(l->target_complete_time,
					lrbp->target_complete_time) > 0)
			break;
		parent = &l->list;
	}

	/* Insert the LRB at the right position */
	list_add(&lrbp->list, parent);

	/* Mark the LRB is being delayed */
	__set_bit(lrbp - hba->lrb, &hba->delayed_reqs);
}

/**
 * __update_statistics - update delay related statistics
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to update
 */
static void __update_statistics(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	enum ufs_impaired_optype op = (rq_data_dir(lrbp->cmd->request) == READ)
		? UFS_IMPAIRED_OPTYPE_READ : UFS_IMPAIRED_OPTYPE_WRITE;
	s64 delay_error;

	/* Final (actual) completion time with delay */
	ktime_t actual_completion_time = ktime_get();

	/* Update statistics */
	hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US] +=
		ktime_us_delta(actual_completion_time,
				lrbp->complete_time_stamp);

	/* Monitor how much more delayed than intended */
	delay_error = ktime_us_delta(actual_completion_time,
			lrbp->target_complete_time);
	hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR] +=
		delay_error;

	/* Keep track the maximum delay error */
	if (hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_MAX_ERROR]
			< delay_error)
		hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_MAX_ERROR] =
			delay_error;


	/* Update completion time for UFS driver stats */
	lrbp->complete_time_stamp = actual_completion_time;
}

/**
 * __impaired_thread_fn - kernel thread to complete requests
 * @param: kthread parameter (hba)
 */
static int __impaired_thread_fn(void *param)
{
	struct ufs_hba *hba = param;
	struct ufshcd_lrb *lrbp;
	unsigned long flags;
	s64 delay_us;

	spin_lock_irqsave(hba->host->host_lock, flags);
	while (!kthread_should_stop()) {
		if (list_empty(&hba->impaired_list_head)) {
			/* Go to sleep if the list is empty */
			spin_unlock_irqrestore(hba->host->host_lock, flags);

			usleep_range(UFS_IMPAIRED_SLEEP_US,
					UFS_IMPAIRED_SLEEP_US * 2);

			spin_lock_irqsave(hba->host->host_lock, flags);
			continue;
		}

		/* Get the first entry */
		lrbp = list_first_entry(&hba->impaired_list_head,
				struct ufshcd_lrb, list);

		/* Calculate needed delay time at this point */
		delay_us = ktime_us_delta(lrbp->target_complete_time,
				ktime_get());

		if (delay_us < UFS_IMPAIRED_ERROR_US) {
			/* Complete an LRB */
			list_del(&lrbp->list);
			__update_statistics(hba, lrbp);
			ufshcd_complete_lrb(hba, lrbp);
			wake_up(&hba->dev_cmd.tag_wq);
		} else {
			/* Do sleep */
			if (delay_us > UFS_IMPAIRED_SLEEP_US)
				delay_us = UFS_IMPAIRED_SLEEP_US;

			spin_unlock_irqrestore(hba->host->host_lock, flags);
			usleep_range(delay_us / 2, delay_us);
			spin_lock_irqsave(hba->host->host_lock, flags);
		}
	}

	/* Complete pending I/Os, if exist */
	list_for_each_entry(lrbp, &hba->impaired_list_head, list) {
		__update_statistics(hba, lrbp);
		ufshcd_complete_lrb(hba, lrbp);
	}

	/* Empty the queue */
	INIT_LIST_HEAD(&hba->impaired_list_head);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return 0;
}

/**
 * ufs_impaired_delay_completion - delay an LRB.
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to be queued
 */
void ufs_impaired_delay_completion(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	/* Pick delay and set in LRB */
	__set_delay_in_lrb(hba, lrbp);

	/* If delay is not needed, complete and return.
	   Otherwise, put the LRB into the delay queue */
	if (!lrbp->complete_delay)
		ufshcd_complete_lrb(hba, lrbp);
	else {
		__queue_lrb(hba, lrbp);
		wake_up_process(hba->impaired_thread);
	}
}

/**
 * ufs_impaired_init - Start impaired storage emulation service.
 * @hba: per adapter instance
 */
void ufs_impaired_init(struct ufs_hba *hba)
{
	/* Set default value to the sysfs entries */
	hba->impaired.io[UFS_IMPAIRED_OPTYPE_READ].delay[
		UFS_IMPAIRED_DELAYTYPE_PERCENT] = 100;
	hba->impaired.io[UFS_IMPAIRED_OPTYPE_WRITE].delay[
		UFS_IMPAIRED_DELAYTYPE_PERCENT] = 100;

	/* Initialize impaired list */
	INIT_LIST_HEAD(&hba->impaired_list_head);

	/* Initialize a mutext to protect thread creation */
	mutex_init(&hba->impaired_thread_mutex);
}

/**
 * ufs_impaired_exit - Stop impaired storage emulation service.
 * @hba: per adapter instance
 */
void ufs_impaired_exit(struct ufs_hba *hba)
{
	mutex_lock(&hba->impaired_thread_mutex);
	if (!hba->impaired_thread) {
		mutex_unlock(&hba->impaired_thread_mutex);
		return;
	}
	mutex_unlock(&hba->impaired_thread_mutex);

	/* Wake up impaired_thread to let suicide */
	wake_up_process(hba->impaired_thread);
	kthread_stop(hba->impaired_thread);
}

