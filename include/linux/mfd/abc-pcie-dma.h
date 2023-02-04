/*
 * Android coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ABC_PCIE_DMA_H
#define __ABC_PCIE_DMA_H

#include <linux/mfd/abc-pcie.h>

/**
 * Multi-block transfer configuration
 */

/* linked list headers */
#define LL_DATA_ELEMENT         0x1
#define LL_IRQ_DATA_ELEMENT     0x9
#define LL_LINK_ELEMENT         0x5
#define LL_LAST_LINK_ELEMENT    0x7

enum abc_dma_dram_state_e {
	AB_DMA_DRAM_DOWN = 0,
	AB_DMA_DRAM_SUSPEND, /* DRAM in self-refresh mode */
	AB_DMA_DRAM_UP
};

#define dram_state_str(state) ((state) ? \
			((state & 0x2) ? "UP" : "SUSPEND") : "DOWN")

struct abc_pcie_dma;

struct abc_pcie_dma_uapi {
	struct miscdevice mdev;
	struct abc_pcie_dma *abc_dma;
};

struct abc_pcie_dma_session {
	struct list_head transfers;
	uint64_t next_xfer_id;
	struct mutex lock;
	struct abc_pcie_dma_uapi *uapi;
	struct kmem_cache *waiter_cache;
	struct list_head list_session;
};

struct abc_pcie_dma {
	struct platform_device *pdev;
	struct device *dma_dev;
	struct abc_pcie_dma_uapi uapi;
	bool pcie_link_up;
	/* marks that an error was found on the link and it's
	 * going down. all transactions that come until the
	 * link down process finishes will result in an error
	 */
	unsigned long link_poisoned;
	enum abc_dma_dram_state_e dram_state;
	struct abc_pcie_dma_ops pcie_notify_ops;
	struct notifier_block dram_nb;
	struct rw_semaphore state_transition_rwsem;
	struct list_head sessions; /* List of all sessions */
	int iatu;
	struct mutex iatu_mutex;
};

/***
 * DMA buffer types supported
 */
enum dma_buf_kind {
	DMA_BUFFER_KIND_USER = 0,
	DMA_BUFFER_KIND_DMA_BUF,
	DMA_BUFFER_KIND_VMALLOC,
	DMA_BUFFER_KIND_CMA
};

/**
 * DMA transfer method.
 */
typedef enum  {
	DMA_TRANSFER_USING_PIO = 0,
	DMA_TRANSFER_USING_SBLOCK,
	DMA_TRANSFER_USING_MBLOCK
} dma_xfer_method_t;

/**
 * DMA Kernel descriptor.
 * Used internally by the kernel to specify the transaction.
 * More cleanup can be done b/129093732
 */
struct abc_pcie_kernel_dma_desc {
	enum dma_buf_kind local_buf_kind; /* local buffer type (DMA/user) */
	union {
		void __user *local_buf; /* local buffer address */
		int local_dma_buf_fd; /* local DMA buffer file descriptor */
	};
	uint64_t local_dma_buf_off; /* offset within dma buf to xfer from/to */

	enum dma_buf_kind remote_buf_kind;
	union {
		uint64_t remote_buf; /* remote buffer virtual address */
		int remote_dma_buf_fd; /* remote DMA buffer file descriptor */
	};
	uint64_t remote_dma_buf_off; /* offset within dma buf to xfer from/to */

	uint64_t size; /* number of bytes to transfer */
	enum dma_data_direction dir; /* direction of the DMA transfer */
};

struct abc_pcie_sg_list {
	struct page **mypage;
	struct scatterlist *sc_list;
	int n_num;
	enum dma_data_direction dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg_table;
};

/**
 * Structure of a single linked list entry
 */
struct abc_pcie_dma_ll_element {
	uint32_t header;
	uint32_t size;
	uint32_t sar_low;
	uint32_t sar_high;
	uint32_t dar_low;
	uint32_t dar_high;
};

/**
 * abc_buf_desc: Container describing the DMA buffer.
 */
struct abc_buf_desc {
	enum dma_buf_kind buf_type;
	uint64_t offset;
	union {
		int fd;
		void __user *local_addr;
		int64_t remote_addr;
	};
	struct abc_pcie_sg_list *sgl;
};

struct abc_pcie_dma_mblk_desc {
	size_t size;
	dma_addr_t dma_paddr;
	struct dma_buf *ab_dram_dma_buf;
	struct bar_mapping mapping;
	int num_dma_channels;
	int entries_per_channel[ABC_DMA_MAX_CHAN];
	int channel_mask;
};

struct abc_dma_xfer;

struct abc_dma_wait_info {
	struct completion done;
	int error; /* 0==OK or ERROR_CODE */
	int active_waiters;
	struct abc_dma_xfer *xfer;
	uint64_t xfer_id; /* Used for debugging */
	uint32_t start_id; /* Used for debugging */
};

/**
 * DMA Transfer (kernel transfer descriptor)
 */
struct abc_dma_xfer {
	uint64_t id;
	enum dma_data_direction dir;
	size_t size; /* Size of transfer, not buffer */
	dma_xfer_method_t transfer_method;
	struct abc_buf_desc local_buf;
	struct abc_buf_desc remote_buf;
	bool pending; /* True if xfer in pending queue */
	bool synced; /* True if cpu synced */
	int error;
	bool poisoned; /* True if poisoned */

	struct abc_pcie_dma_session *session; /* This transfer's session */
	struct list_head list_transfers; /* Has session transfers */
	struct list_head list_pending; /* Has overall pending transfers */

	struct dma_element_t *sblk_desc;
	struct abc_pcie_dma_mblk_desc *mblk_desc;
	struct abc_dma_wait_info *wait_info;
};

/*******************************************************************************
 *
 *	APIs
 *
 ******************************************************************************/

/**
 * Initializes a session.
 * @session[out] New session.
 * @return 0 on success
 */
int abc_pcie_dma_open_session(struct abc_pcie_dma_session *session);

/**
 * Cleans up & closes a session.
 * @session[in] Existing session.
 */
void abc_pcie_dma_close_session(struct abc_pcie_dma_session *session);

/**
 * Issue a Synchronous DMA transfer
 * @session[in] Currently open session.
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_dma_xfer_sync(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc);

/**
 * Issue a Sessionless Synchronous DMA transfer
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_sessionless_dma_xfer_sync(
					struct abc_pcie_kernel_dma_desc *desc);

/**
 * Create and prepare a transfer.
 * @session[in] session to which this transfer belongs to.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @id[out] Newly assign ID for the created transfer.
 * @return 0 on success
 */
int abc_pcie_create_dma_xfer(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc,
				struct abc_dma_xfer **new_xfer, uint64_t *id);

/**
 * Create and prepare a transfer.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @return 0 on success
 */
int abc_pcie_create_sessionless_dma_xfer(struct abc_pcie_kernel_dma_desc *desc,
					struct abc_dma_xfer **new_xfer);

/**
 * Top-level API for Asynchronously starting DMA transfers
 * abc_pcie_create_dma_xfer() must be called on the transfer before
 * this function gets called.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_start_dma_xfer(struct abc_dma_xfer *xfer, uint32_t *start_id);

/**
 * API for waiting for a transfer to be done.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 * @timeout[in] Timeout usec, 0=zero wait, <0 = Infinity
 * @error[in] If any error encountered in the transaction itself it will be
 *            indicated here.
 */
int abc_pcie_wait_dma_xfer(struct abc_dma_xfer *xfer, int timeout, int *error,
				uint32_t *start_id);

/**
 * API for cleaning a transfer and removing it from the internal,
 * data structures. This api also de-allocates the transfer.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
void abc_pcie_clean_dma_xfer(struct abc_dma_xfer *xfer);


/* Internal calls between UAPI and DMA below. */
struct abc_dma_xfer *abc_pcie_dma_find_xfer(
			struct abc_pcie_dma_session *session, uint64_t id);
int init_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi);
void remove_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi);
int abc_pcie_start_dma_xfer_locked(struct abc_dma_xfer *xfer,
					uint32_t *start_id);
void abc_pcie_clean_dma_xfer_locked(struct abc_dma_xfer *xfer);
int abc_pcie_dma_get_user_wait(uint64_t id,
				struct abc_pcie_dma_session *session,
				struct abc_dma_wait_info **wait_info);
int abc_pcie_dma_do_wait(struct abc_pcie_dma_session *session,
			struct abc_dma_wait_info *wait_info,
			int timeout, int *error, uint32_t *start_id);
#endif /* __ABC_PCIE_DMA_H */
