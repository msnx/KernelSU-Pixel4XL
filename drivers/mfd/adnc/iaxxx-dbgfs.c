/*
 * iaxxx-dbgfs.c -- IAxxx Debug-fs support
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#define DEBUG
#define pr_fmt(fmt) "%s:%d: " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/regmap.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx-dbgfs.h"

#define ADDR_LEN	 9	/* 4 byte address + 1 space character */
#define CHARS_PER_ITEM   9	/* Format is 'XXXXXXXX ' */
#define MAX_LINE_LENGTH  (ADDR_LEN + (ITEMS_PER_LINE * CHARS_PER_ITEM) + 1)

static const unsigned int ITEMS_PER_LINE = 4;	/* 4 data items per line */

static const char *DFS_ROOT_NAME	= "iaxxx";
static const mode_t DFS_MODE = 0600;

/* Log buffer */
struct iaxxx_log_buffer {
	size_t  rpos;	/* Current 'read' position in buffer */
	size_t  wpos;	/* Current 'write' position in buffer */
	size_t  len;	/* Length of the buffer */
	char data[0];	/* Log buffer */
};

/* IAxxx specific data */
struct iaxxx_dbgfs_data {
	u32 cnt;
	u32 addr;
	struct dentry *dir;
	struct regmap *map;
	struct list_head node;
};

/* Transaction parameters */
struct iaxxx_trans {
	u32 offset;	/* Offset of last read register */
	size_t cnt;	/* Number of registers to read */
	bool raw_data;	/* Set to true for raw data dump */
	struct regmap *map;
	struct iaxxx_log_buffer *log; /* log buffer */
};

struct iaxxx_dbgfs {
	struct dentry *root;
	struct mutex  lock;
	struct list_head link; /* List of iaxxx_dbgfs_data nodes */
	struct debugfs_blob_wrapper help_msg;
};

static struct iaxxx_dbgfs dbgfs_data = {
	.lock = __MUTEX_INITIALIZER(dbgfs_data.lock),
	.link = LIST_HEAD_INIT(dbgfs_data.link),
	.help_msg = {
		.data =
		"IAxxx Debug-FS support\n"
		"\n"
		"Hierarchy schema:\n"
		"/sys/kernel/debug/iaxxx\n"
		"       /help             -- Static help text\n"
		"       /iaxxx-0          -- Directory for IAxxx device 0\n"
		"       /iaxxx-0/address  -- Starting register address for reads or writes\n"
		"       /iaxxx-0/count    -- Number of registers to read (only used for reads)\n"
		"       /iaxxx-0/data     -- Initiates the read (formatted output)\n"
		"       /iaxxx-0/data_raw -- Initiates the raw read or write\n"
		"       /iaxxx-n          -- Directory for IAxxx device n\n"
		"\n"
		"To perform register read or write transactions, you need to first write the\n"
		"address of the IAxxx device register to the 'address' file. For read\n"
		"transactions, the number of bytes to be read needs to be written to the\n"
		"'count' file.\n"
		"\n"
		"The 'address' file specifies the 32-bit address of a device register.\n"
		"\n"
		"Reading from the 'data' file will initiate a IAxxx read transaction starting\n"
		"from register 'address' for 'count' number of registers.\n"
		"\n"
		"Writing to the 'data' file will initiate a IAxxx write transaction starting\n"
		"from register 'address'. The number of registers written to will\n"
		"match the number of 32-bit words written to the 'data' file.\n"
		"\n"
		"Example: Read 4 registers starting at offset 0x5800016C\n"
		"\n"
		"echo 0x5800016C > address\n"
		"echo 4 > count\n"
		"cat data\n"
		"\n"
		"Example: Write 3 registers (event subscription) starting at offset 0x58000120\n"
		"\n"
		"echo 0x58000120 > address\n"
		"echo 0xABE7 0x30814169 0xCAB00D1E > data\n"
		"\n"
		"Note that the count file is not used for writes. Since 3 words are\n"
		"written to the 'data' file, then 3 words will be written to the device\n\n",
	},
};

static int iaxxx_dfs_open(struct iaxxx_dbgfs_data *node_data, struct file *file)
{
	struct iaxxx_log_buffer *log;
	struct iaxxx_trans *trans;

	size_t logbufsize = SZ_4K;

	if (!node_data) {
		pr_err("No IAxxx node data\n");
		return -EINVAL;
	}

	/* Per file "transaction" data */
	trans = kvzalloc(sizeof(*trans), GFP_KERNEL);
	if (!trans)
		return -ENOMEM;

	/* Allocate log buffer */
	log = kvzalloc(logbufsize, GFP_KERNEL);
	if (!log) {
		kvfree(trans);
		pr_err("Unable to allocate memory for log buffer\n");
		return -ENOMEM;
	}

	log->rpos = 0;
	log->wpos = 0;
	log->len = logbufsize - sizeof(*log);

	trans->log = log;
	trans->map = node_data->map;
	trans->cnt = node_data->cnt;
	trans->offset = node_data->addr;

	/* Register offsets need to be a multiple of 4. If not, correct it */
	if (trans->offset & 0x3) {
		trans->offset &= ~3;
		pr_err("Offset address needs to be a multiple of 4\n");
	}

	file->private_data = trans;
	return 0;
}


static int iaxxx_dfs_data_open(struct inode *inode, struct file *file)
{
	struct iaxxx_dbgfs_data *node_data = inode->i_private;

	return iaxxx_dfs_open(node_data, file);
}

static int iaxxx_dfs_raw_data_open(struct inode *inode, struct file *file)
{
	int rc;
	struct iaxxx_trans *trans;
	struct iaxxx_dbgfs_data *node_data = inode->i_private;

	rc = iaxxx_dfs_open(node_data, file);
	trans = file->private_data;
	trans->raw_data = true;
	return rc;
}

static int iaxxx_dfs_close(struct inode *inode, struct file *file)
{
	struct iaxxx_trans *trans = file->private_data;

	if (trans && trans->log) {
		file->private_data = NULL;
		kvfree(trans->log);
		kvfree(trans);
	}

	return 0;
}

/**
 * print_to_log: format a string and place into the log buffer
 * @log: The log buffer to place the result into.
 * @fmt: The format string to use.
 * @...: The arguments for the format string.
 *
 * The return value is the number of characters written to @log buffer
 * not including the trailing '\0'.
 */
static int print_to_log(struct iaxxx_log_buffer *log, const char *fmt, ...)
{
	va_list args;
	int cnt;
	char *buf = &log->data[log->wpos];
	size_t size = log->len - log->wpos;

	va_start(args, fmt);
	cnt = vscnprintf(buf, size, fmt, args);
	va_end(args);

	log->wpos += cnt;
	return cnt;
}

/**
 * print_register_to_log: read register data and write to log buffer
 * @trans:  pointer to transaction data.
 * @offset: register offset to read from.
 *
 * The return value is the number of characters written to @log buffer
 * not including the trailing '\0'.
 */

static int print_register_to_log(struct iaxxx_trans *trans, int offset)
{
	int cnt;
	uint32_t data;

	if (regmap_read(trans->map, offset, &data))
		cnt = print_to_log(trans->log, "???????? ");
	else
		cnt = print_to_log(trans->log, "%8.8X ", data);

	return cnt;
}

/**
 * write_next_line_to_log: Writes a single "line" of data into the log buffer
 * @trans:  pointer to transaction data.
 * @offset: register offset to start reading from.
 * @pcnt:   pointer to 'cnt' variable. Indicates the number of registers to read
 *
 * On a successful read, the pcnt is decremented by the number of registers
 * read. When the cnt reaches 0, all requested registers have been read.
 */
static int
write_next_line_to_log(struct iaxxx_trans *trans, int offset, size_t *pcnt)
{
	int i;
	struct iaxxx_log_buffer *log = trans->log;

	int cnt = 0;
	size_t padding = (offset / sizeof(uint32_t)) % ITEMS_PER_LINE;
	size_t items_to_read = min((size_t)(ITEMS_PER_LINE - padding), *pcnt);

	/* Buffer needs enough space for an entire line */
	if ((log->len - log->wpos) < MAX_LINE_LENGTH)
		goto done;

	/* Each line starts with the aligned offset (32-bit address) */
	cnt = print_to_log(log, "%8.8X: ", offset & ~0xf);
	if (cnt == 0)
		goto done;

	/* If the offset is "unaligned", add padding to right justify items */
	for (i = 0; i < padding; ++i) {
		cnt = print_to_log(log, "-------- ");
		if (cnt == 0)
			goto done;
	}

	/* Log the register data */
	for (i = 0; i < items_to_read; ++i, offset += sizeof(u32), --*pcnt) {
		cnt = print_register_to_log(trans, offset);
		if (cnt == 0)
			goto done;
	}

	/* If the last character was a space, then replace it with a newline */
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

done:
	return cnt;
}

/**
 * write_raw_data_to_log: Writes a single "line" of data into the log buffer
 * @trans:  pointer to transaction data.
 * @offset: register offset to start reading from.
 * @pcnt:   pointer to 'cnt' variable. Indicates the number of registers to read
 *
 * On a successful read, the pcnt is decremented by the number of data
 * bytes read across the IAxxx bus.  When the cnt reaches 0, all requested
 * bytes have been read.
 */
static int
write_raw_data_to_log(struct iaxxx_trans *trans, int offset, size_t *pcnt)
{
	struct iaxxx_log_buffer *log = trans->log;

	int i;
	int cnt = 0;
	size_t items_to_read = min((size_t)ITEMS_PER_LINE, *pcnt);

	/* Buffer needs enough space for an entire line */
	if ((log->len - log->wpos) < 80)
		goto done;

	*pcnt -= items_to_read;

	/* Log the data items */
	for (i = 0; i < items_to_read; ++i, offset += sizeof(u32)) {
		cnt = print_register_to_log(trans, offset);
		if (cnt == 0)
			goto done;
	}

	/* If the last character was a space, then replace it with a newline */
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

done:
	return cnt;
}

/**
 * get_log_data - reads data from the register map and saves to the log buffer
 * @trans: Pointer to IAxxx transaction data.
 *
 * Returns the number of "items" read or error code for read failures.
 */
static int get_log_data(struct iaxxx_trans *trans)
{
	int cnt;
	size_t last_cnt;
	size_t items_read;
	int total_items_read = 0;
	u32 offset = trans->offset;
	size_t item_cnt = trans->cnt;
	struct iaxxx_log_buffer *log = trans->log;
	int (*write_to_log)(struct iaxxx_trans *, int, size_t *);

	if (item_cnt == 0)
		return 0;

	if (trans->raw_data)
		write_to_log = write_raw_data_to_log;
	else
		write_to_log = write_next_line_to_log;

	/* Reset the log buffer 'pointers' */
	log->wpos = log->rpos = 0;

	/* Keep reading data until the log is full */
	do {
		last_cnt = item_cnt;
		cnt = write_to_log(trans, offset, &item_cnt);
		items_read = last_cnt - item_cnt;
		offset += (items_read * sizeof(uint32_t));
		total_items_read += items_read;
	} while (cnt && item_cnt > 0);

	/* Adjust the transaction offset and count */
	trans->cnt = item_cnt;
	trans->offset += (total_items_read * sizeof(uint32_t));

	return total_items_read;
}


/**
 * iaxxx_dfs_reg_write: write user's byte array (coded as string).
 * @file: file pointer
 * @buf: user data to be written.
 * @count: maximum space available in @buf
 * @ppos: starting position
 * @return number of user byte written, or negative error value
 */
static ssize_t iaxxx_dfs_reg_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	int pos;
	int cnt;
	int bytes_read;

	uint32_t data;
	uint32_t *values;
	size_t ret = 0;

	struct iaxxx_trans *trans = file->private_data;
	uint32_t offset = trans->offset;
	char *kbuf;

	/* Make a copy of the user data */
	kbuf = kvmalloc(count + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = copy_from_user(kbuf, buf, count);
	if (ret == count) {
		pr_err("failed to copy data from user\n");
		ret = -EFAULT;
		goto free_kbuf;
	}

	count -= ret;
	*ppos += count;
	kbuf[count] = '\0';

	/* First pass, count the number of values in the buffer */
	pos = cnt = 0;
	while (sscanf(kbuf + pos, "%i%n", &data, &bytes_read) == 1) {
		++cnt;
		pos += bytes_read;
	}

	if (!cnt) {
		pr_err("No valid register data in input buffer\n");
		goto free_kbuf;
	}

	/* Allocate buffer for the 'parsed' register data */
	values = kmalloc_array(cnt, sizeof(*values), GFP_KERNEL);
	if (!values)
		goto free_kbuf;

	/* Parse the data in the buffer.  It should be a string of numbers */
	pos = cnt = 0;
	while (sscanf(kbuf + pos, "%i%n", &data, &bytes_read) == 1) {
		pos += bytes_read;
		values[cnt++] = data;
	}

	if (WARN_ON(!cnt))
		goto free_values;

	/* Perform the register write(s) */
	ret = regmap_bulk_write(trans->map, offset, values, cnt);
	if (ret) {
		pr_err("Register write failed, err = %zu\n", ret);
	} else {
		ret = count;
		trans->offset += (cnt * sizeof(*values));
	}

free_values:
	kfree(values);

free_kbuf:
	kvfree(kbuf);
	return ret;
}

/**
 * iaxxx_dfs_reg_read: reads value(s) and fill user's buffer with register
 *  data (coded as string)
 * @file: file pointer
 * @buf: where to put the result
 * @count: maximum space available in @buf
 * @ppos: starting position
 * @return number of user bytes read, or negative error value
 */
static ssize_t iaxxx_dfs_reg_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct iaxxx_trans *trans = file->private_data;
	struct iaxxx_log_buffer *log = trans->log;
	size_t ret;
	size_t len;

	/* Is the the log buffer empty */
	if (log->rpos >= log->wpos) {
		if (get_log_data(trans) <= 0)
			return 0;
	}

	len = min(count, log->wpos - log->rpos);

	ret = copy_to_user(buf, &log->data[log->rpos], len);
	if (ret == len) {
		pr_err("failed to copy register values to user\n");
		return -EFAULT;
	}

	/* 'ret' is the number of bytes not copied */
	len -= ret;

	*ppos += len;
	log->rpos += len;
	return len;
}

static const struct file_operations iaxxx_dfs_reg_fops = {
	.open		= iaxxx_dfs_data_open,
	.release	= iaxxx_dfs_close,
	.read		= iaxxx_dfs_reg_read,
	.write		= iaxxx_dfs_reg_write,
};

static const struct file_operations iaxxx_dfs_raw_data_fops = {
	.open		= iaxxx_dfs_raw_data_open,
	.release	= iaxxx_dfs_close,
	.read		= iaxxx_dfs_reg_read,
	.write		= iaxxx_dfs_reg_write,
};

/**
 * iaxxx_dfs_create_fs: create debugfs file system.
 * @return pointer to root directory or NULL if failed to create fs
 */
static struct dentry *iaxxx_dfs_create_fs(void)
{
	struct dentry *root, *file;

	pr_debug("Creating IAxxx debugfs file-system\n");
	root = debugfs_create_dir(DFS_ROOT_NAME, NULL);
	if (IS_ERR(root)) {
		pr_err("Error creating top level directory err:%ld\n",
		       (long)root);
		if (PTR_ERR(root) == -ENODEV)
			pr_err("debugfs is not enabled in the kernel\n");
		return NULL;
	}

	dbgfs_data.help_msg.size = strlen(dbgfs_data.help_msg.data);

	file = debugfs_create_blob("help", 0444, root, &dbgfs_data.help_msg);
	if (!file) {
		pr_err("error creating help entry\n");
		goto err_remove_fs;
	}
	return root;

err_remove_fs:
	debugfs_remove_recursive(root);
	return NULL;
}

/**
 * iaxxx_dfs_get_root: return a pointer to the debugfs root directory.
 * @brief return a pointer to the existing directory, or if no root
 * directory exists then create one.
 *
 * @returns valid pointer on success or NULL
 */
struct dentry *iaxxx_dfs_get_root(void)
{
	if (dbgfs_data.root)
		return dbgfs_data.root;

	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return NULL;
	/* critical section */
	if (!dbgfs_data.root) { /* double checking idiom */
		dbgfs_data.root = iaxxx_dfs_create_fs();
	}
	mutex_unlock(&dbgfs_data.lock);
	return dbgfs_data.root;
}

/*
 * iaxxx_dfs_switch_regmap: adds new register map entry
 * @return zero on success
 */
int iaxxx_dfs_switch_regmap(struct device *dev,
				struct regmap *map, void *dfs_node)
{
	struct iaxxx_dbgfs_data *node_data = dfs_node;

	/* Switch map if necessary */
	if (node_data)
		node_data->map = map;

	return 0;
}

/*
 * iaxxx_dfs_add_regmap: adds new register map entry
 * @return zero on success
 */
int iaxxx_dfs_add_regmap(struct device *dev,
				struct regmap *map, void **dfs_node)
{
	struct dentry *dir;
	struct dentry *root;
	struct dentry *file;
	struct iaxxx_dbgfs_data *node_data;

	pr_debug("Adding register map debugfs for %s\n", dev_name(dev));
	root = iaxxx_dfs_get_root();
	if (!root)
		return -ENOENT;

	/* Allocate transaction data for the controller */
	node_data = kvzalloc(sizeof(*node_data), GFP_KERNEL);
	if (!node_data)
		return -ENOMEM;

	dir = debugfs_create_dir(dev->kobj.name, root);
	if (!dir) {
		pr_err("Error creating regmap entry for %s\n", dev_name(dev));
		goto err_create_dir_failed;
	}

	node_data->cnt = 1;
	node_data->dir = dir;
	node_data->map = map;

	/* Return node_data for future regmap change */
	if (dfs_node)
		*dfs_node = node_data;

	file = debugfs_create_u32("count", DFS_MODE, dir, &node_data->cnt);
	if (!file) {
		pr_err("error creating 'count' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_x32("address", DFS_MODE, dir, &node_data->addr);
	if (!file) {
		pr_err("error creating 'address' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data", DFS_MODE, dir, node_data,
						&iaxxx_dfs_reg_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data_raw", DFS_MODE, dir, node_data,
						&iaxxx_dfs_raw_data_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	list_add(&node_data->node, &dbgfs_data.link);
	return 0;

err_remove_fs:
	debugfs_remove_recursive(dir);
err_create_dir_failed:
	kvfree(node_data);
	return -ENOMEM;
}

/*
 * iaxxx_dfs_del_regmap: deletes IAxxx register map entry
 * @return zero on success
 */
int iaxxx_dfs_del_regmap(struct device *dev, struct regmap *map)
{
	int rc;
	struct list_head *pos, *tmp;
	struct iaxxx_dbgfs_data *node_data;

	pr_debug("Deleting register map debugfs for %s\n", dev_name(dev));

	rc = mutex_lock_interruptible(&dbgfs_data.lock);
	if (rc)
		return rc;

	list_for_each_safe(pos, tmp, &dbgfs_data.link) {
		node_data = list_entry(pos, struct iaxxx_dbgfs_data, node);

		if (node_data->map == map) {
			debugfs_remove_recursive(node_data->dir);
			list_del(pos);
			kvfree(node_data);
			rc = 0;
			goto done;
		}
	}
	rc = -EINVAL;
	pr_debug("Unknown register map for %s\n", dev_name(dev));

done:
	mutex_unlock(&dbgfs_data.lock);
	return rc;
}

static void iaxxx_dfs_delete_all(struct list_head *head)
{
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, head) {
		struct iaxxx_dbgfs_data *node_data;

		node_data = list_entry(pos, struct iaxxx_dbgfs_data, node);
		list_del(pos);
		kvfree(node_data);
	}
}

static void iaxxx_dfs_destroy(void)
{
	pr_debug("de-initializing iaxxx debugfs ...\n");
	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return;

	debugfs_remove_recursive(dbgfs_data.root);
	dbgfs_data.root = NULL;
	iaxxx_dfs_delete_all(&dbgfs_data.link);

	mutex_unlock(&dbgfs_data.lock);
}

module_exit(iaxxx_dfs_destroy);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:iaxxx_debug_fs");
