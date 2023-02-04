/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MODEM_SMEM_H
#define _MODEM_SMEM_H

#include <linux/types.h>

/* Modem smem driver version */
#define MODEM_SMEM_VERSION	0x0
#define MODEM_FTM_MAGIC		0x6846544D
#define MODEM_DSDS_MAGIC	0x44534453

/* Modem smem channel */
#define SMEM_ID_VENDOR0		134

struct modem_smem_type {
	uint32_t	version;
	uint32_t	reserved_1;
	uint32_t	major_id;
	uint32_t	minor_id;
	uint32_t	subtype;
	uint32_t	platform;
	uint32_t	reserved_2;
	uint32_t	ftm_magic;
	uint32_t	efs_magic;
	uint32_t	modem_flag;
	uint32_t	dsds_magic;
	uint32_t	reserved_3[5];
};

#define modem_smem_addr(smem, field) \
	({ \
		void __iomem *__p = (smem); \
		__p += offsetof(typeof(*(smem)), field); \
		__p; \
	})

#define modem_smem_set_u32(p, field, value) \
	writel_relaxed((value), modem_smem_addr((p), field))

#define modem_smem_copy(p, field, src) \
	memcpy_toio(modem_smem_addr((p), field), (src), sizeof((p)->field))

#endif /* end of _MODEM_SMEM_H */
