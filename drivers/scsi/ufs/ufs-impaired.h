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

#ifndef _UFS_IMPAIRED_H
#define _UFS_IMPAIRED_H

enum ufs_impaired_optype {
	UFS_IMPAIRED_OPTYPE_READ = 0,
	UFS_IMPAIRED_OPTYPE_WRITE = 1,
	UFS_IMPAIRED_OPTYPE_COUNT = 2,
};

enum ufs_impaired_delaytype {
	UFS_IMPAIRED_DELAYTYPE_PERCENT = 0,
	UFS_IMPAIRED_DELAYTYPE_US = 1,
	UFS_IMPAIRED_DELAYTYPE_MAX = 2,
	UFS_IMPAIRED_DELAYTYPE_SKIP = 3,
#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	UFS_IMPAIRED_DELAYTYPE_MODEL = 4,
	UFS_IMPAIRED_DELAYTYPE_COUNT = 5,
#else
	UFS_IMPAIRED_DELAYTYPE_COUNT = 4,
#endif
};

enum ufs_impaired_stattype {
	UFS_IMPAIRED_STATTYPE_TOTAL_CNT = 0,
	UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US = 1,
	UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US = 2,
	UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR = 3,
	UFS_IMPAIRED_STATTYPE_MAX_DELAY = 4,
	UFS_IMPAIRED_STATTYPE_MAX_ERROR = 5,
	UFS_IMPAIRED_STATTYPE_COUNT = 6,
};

struct ufs_impaired_io {
	/* Delay control */
	u32 delay[UFS_IMPAIRED_DELAYTYPE_COUNT];

	/* Statistics */
	s64 stat[UFS_IMPAIRED_STATTYPE_COUNT];

	/* Skip count */
	u32 skip_count;
};

struct ufs_impaired_storage {
	u8 enabled;
#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	u8 emulate_fragftl;
#endif
	struct ufs_impaired_io io[UFS_IMPAIRED_OPTYPE_COUNT];
};

struct ufshcd_lrb;

extern void ufs_impaired_init(struct ufs_hba *hba);
extern void ufs_impaired_exit(struct ufs_hba *hba);
extern void ufs_impaired_init_sysfs(struct ufs_hba *hba);
extern void ufs_impaired_delay_completion(struct ufs_hba *hba,
		struct ufshcd_lrb *lrbp);

#endif /* End of Header */
