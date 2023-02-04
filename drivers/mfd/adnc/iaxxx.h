/*
 * iaxxx.h -- IAxxx MFD core internals
 *
 * Copyright 2016 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef __MFD_IAXXX_H__
#define __MFD_IAXXX_H__

#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/mfd/adnc/iaxxx-core.h>

struct iaxxx_priv;
struct regmap;
struct regmap_config;
struct iaxxx_event;
struct iaxxx_evt_queue;
#define IAXXX_READ	0
#define IAXXX_WRITE	1
#define IAXXX_BUF_MAX_LEN	16384
#define IAXXX_CM4_CTRL_MGR_SRC_ID 0x2610
#define IAXXX_CRASH_EVENT_ID 2
#define IAXXX_BOOT_COMPLETE_EVENT_ID 0
#define IAXXX_HOST0_WAKEUP_EVENT_ID 3
#define IAXXX_HOST1_WAKEUP_EVENT_ID 4
#define IAXXX_SPI_SBL_SPEED	4800000
#define PLUGIN_INST_NONE	0xFF

#define EVENT_ID_KW_ID                0
/* Knowles VT algo specific events */
#define EVENT_ID_START_FRAME          1
#define EVENT_ID_END_FRAME            2
#define EVENT_ID_TRUE_CONFIRMATAION   3
#define EVENT_ID_FALSE_ACCEPTANCE     4
#define EVENT_ID_MAX_CONFIDENCE_LEVEL 5

#define IAXXX_READ_DELAY        10	/* 10 us delay before SPI read */
#define IAXXX_READ_DELAY_RANGE  10	/* 10 us range */

/* Linux kthread APIs have changes in version 4.9 */
#if defined(init_kthread_worker)
#define iaxxx_work_flush(priv, work)    flush_kthread_work(&priv->work)
#define iaxxx_work(priv, work) queue_kthread_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) flush_kthread_worker(worker)
#define iaxxx_init_kthread_worker(worker)  init_kthread_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  init_kthread_work(work, fn)
#elif defined(kthread_init_worker)
#define iaxxx_work_flush(priv, work)    kthread_flush_work(&priv->work)
#define iaxxx_work(priv, work) kthread_queue_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) kthread_flush_worker(worker)
#define iaxxx_init_kthread_worker(worker)  kthread_init_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  kthread_init_work(work, fn)
#else
#error kthread functions not defined
#endif

/* Checksum Calculation */
#define CALC_FLETCHER16(DATA, SUM1, SUM2)	\
do {						\
	SUM1 += (DATA) & 0xffff;		\
	SUM2 += SUM1;				\
	SUM1 += ((DATA) >> 16) & 0xffff;	\
	SUM2 += SUM1;				\
	SUM1 = (SUM1 & 0xffff) + (SUM1 >> 16);	\
	SUM2 = (SUM2 & 0xffff) + (SUM2 >> 16);	\
} while (0)

/* System mode */
enum iaxxx_system_status_mode {
	SYSTEM_STATUS_MODE_RESET = 0,
	SYSTEM_STATUS_MODE_SBL,
	SYSTEM_STATUS_MODE_APPS,
};

enum {
	SYSRC_SUCCESS = 0,
	SYSRC_FAIL,
	SYSRC_ERR_PARM,
	SYSRC_ERR_MEM,
	SYSRC_ERR_BUSY,
	SYSRC_ERR_PEND,
	SYSRC_ERR_STATE,
	SYSRC_ERR_LIMIT,
	SYSRC_ERR_BOUNDARY,
	SYSRC_ERR_IO,
};

enum iaxxx_fw_crash_reasons {
	IAXXX_FW_CRASH_EVENT = 1,
	IAXXX_FW_CRASH_UPDATE_BLOCK_REQ = 2,
	IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF = 3,
	IAXXX_FW_CRASH_RESUME = 4,
	IAXXX_FW_CRASH_SUSPEND = 5,
	IAXXX_FW_CRASH_SIMULATED = 6,
};

struct iaxxx_register_log {
	u32 addr;
	u32 val;
	bool op;
	struct timespec timestamp;
};

struct iaxxx_reg_dump_priv {
	struct iaxxx_register_log *log;
	uint32_t head;
	uint32_t tail;
	spinlock_t ring_lock;
};

struct firmware_file_header {
	uint32_t signature[2];
	uint32_t reserved;
	uint32_t abort_code;
	uint32_t number_of_sections;
	uint32_t entry_point;
};

/* Used for both binary data and checksum sections */
/* Each data section is followed by a checksum section */
struct firmware_section_header {
	uint32_t length;	/* Section length in 32-bit words */
	uint32_t start_address;
};

struct iaxxx_raw_bus_ops {
	int (*read)(struct iaxxx_priv *priv, void *buf, int len);
	int (*write)(struct iaxxx_priv *priv, const void *buf, int len);
};


int iaxxx_device_reset(struct iaxxx_priv *priv);
int iaxxx_device_init(struct iaxxx_priv *priv);
void iaxxx_device_exit(struct iaxxx_priv *priv);

/* Checks if the device firmware is ready to accept requests from the host */
int iaxxx_get_device_status(struct iaxxx_priv *priv, bool *status);

/* Boots the device into application mode */
int iaxxx_bootup(struct iaxxx_priv *priv);

/* Register map */
int iaxxx_regmap_init(struct iaxxx_priv *priv);
int iaxxx_sbl_regmap_init(struct iaxxx_priv *priv);
int iaxxx_application_regmap_init(struct iaxxx_priv *priv);

/* Bootloader */
int iaxxx_jump_to_request(struct iaxxx_priv *priv, uint32_t address);
int iaxxx_calibrate_oscillator_request(struct iaxxx_priv *priv, uint32_t delay);

/* Checksum */
int iaxxx_checksum_request(struct iaxxx_priv *priv, uint32_t address,
			uint32_t length, uint32_t *sum1, uint32_t *sum2,
			struct regmap *regmap);

/* Event manager */
int iaxxx_event_handler(struct iaxxx_priv *priv, struct iaxxx_event *evt);

int iaxxx_event_init(struct iaxxx_priv *priv);
void iaxxx_event_exit(struct iaxxx_priv *priv);
int iaxxx_get_event_flush(struct iaxxx_priv *priv);
void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
								bool op);
int iaxxx_verify_fw_header(struct device *dev,
			struct firmware_file_header *header);
int iaxxx_download_section(struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section,
				struct regmap *regmap, bool btp);

void iaxxx_copy_le32_to_cpu(void *dst, const void *src, size_t nbytes);
int iaxxx_fw_crash(struct device *dev, enum iaxxx_fw_crash_reasons reasons,
		uint32_t proc_id);
int iaxxx_get_version_str(struct iaxxx_priv *priv, uint32_t reg, char *verbuf,
								uint32_t len);
int iaxxx_abort_fw_recovery(struct iaxxx_priv *priv);
int iaxxx_reset_to_sbl(struct iaxxx_priv *priv);
int iaxxx_regmap_drop_regions(struct iaxxx_priv *priv);
uint32_t iaxxx_conv_physical_to_virtual_register_address(
		struct iaxxx_priv *priv,
		const uint32_t phy_addr);
uint32_t iaxxx_conv_virtual_to_physical_register_address(
		struct iaxxx_priv *priv,
		const uint32_t virt_addr);

#endif /* __MFD_IAXXX_H__ */
