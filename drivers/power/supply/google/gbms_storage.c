/*
 * Copyright 2019 Google, Inc
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/genalloc.h>
#include <linux/hashtable.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h> /* register_chrdev, unregister_chrdev */
#include <linux/of.h>
#include <linux/module.h>
#include <linux/seq_file.h> /* seq_read, seq_lseek, single_release */
#include <linux/log2.h>
#include "google_bms.h"

struct gbms_storage_provider {
	const char *name;
	struct gbms_storage_desc *dsc;
	bool offline;
	void *ptr;
};

struct gbms_cache_entry {
	struct hlist_node hnode;
	void *provider;
	gbms_tag_t tag;
	size_t count;
	size_t addr;
};

#define GBMS_PROVIDER_NAME_MAX	32

#define GBMS_PROVIDERS_MAX	4
static spinlock_t providers_lock;
static bool gbms_storage_init_done;

static int gbms_providers_count;
static struct gbms_storage_provider gbms_providers[GBMS_PROVIDERS_MAX];
static struct dentry *rootdir;

/* 1 << 5 = 64 entries */
#define GBMS_HASHTABLE_SIZE	5
DECLARE_HASHTABLE(gbms_cache, GBMS_HASHTABLE_SIZE);
static struct gen_pool *gbms_cache_pool;
static void *gbms_cache_mem;

/* use this as a temporary buffer for converting a tag to a string */
typedef char gbms_tag_cstr_t[sizeof(gbms_tag_t) + 1];

static char *tag2cstr(gbms_tag_cstr_t buff, gbms_tag_t tag)
{
	const u32 tmp = cpu_to_le32(tag);

	buff[3] = tmp & 0xff;
	buff[2] = (tmp >> 8) & 0xff;
	buff[1] = (tmp >> 16) & 0xff;
	buff[0] = (tmp >> 24) & 0xff;
	buff[4] = 0;

	return buff;
}

static gbms_tag_t cstr2tag(gbms_tag_cstr_t buff)
{
	gbms_tag_t tag;

	tag = (u8)buff[0];
	tag = (tag << 8) + (u8)buff[1];
	tag = (tag << 8) + (u8)buff[2];
	tag = (tag << 8) + (u8)buff[3];

	return tag;
}

/* ------------------------------------------------------------------------- */

static inline u64 gbms_cache_hash(gbms_tag_t tag)
{
	return tag;
}

/* TODO: caching */
static struct gbms_cache_entry *gbms_cache_lookup(gbms_tag_t tag, size_t *addr)
{
	unsigned long flags;
	struct gbms_cache_entry *ce;
	const u64 hash = gbms_cache_hash(tag);

	spin_lock_irqsave(&providers_lock, flags);

	hash_for_each_possible(gbms_cache, ce, hnode, hash) {
		if (ce->tag == tag) {
			spin_unlock_irqrestore(&providers_lock, flags);
			return ce;
		}
	}

	spin_unlock_irqrestore(&providers_lock, flags);
	return NULL;
}

/* call only on a cache miss */
static struct gbms_cache_entry *gbms_cache_add(gbms_tag_t tag,
					struct gbms_storage_provider *slot)
{
	unsigned long flags;
	struct gbms_cache_entry *entry;

	if (!gbms_cache_pool || !slot)
		return 0;

	entry = (struct gbms_cache_entry *)
		gen_pool_alloc(gbms_cache_pool, sizeof(*entry));
	if (!entry)
		return NULL;

	/* cache provider */
	memset(entry, 0, sizeof(*entry));
	entry->provider = slot;
	entry->tag = tag;
	entry->addr = GBMS_STORAGE_ADDR_INVALID;

	/* cache location if available */
	if (slot->dsc->fetch && slot->dsc->store && slot->dsc->info) {
		size_t addr, count;
		int ret;

		ret = slot->dsc->info(tag, &addr, &count, slot->ptr);
		if (ret == 0) {
			entry->count = count;
			entry->addr = addr;
		}
	}

	spin_lock_irqsave(&providers_lock, flags);
	hash_add(gbms_cache, &entry->hnode, gbms_cache_hash(tag));
	spin_unlock_irqrestore(&providers_lock, flags);

	return entry;
}

/* ------------------------------------------------------------------------- */

/* TODO: check for duplicates in the tag
 */
static int gbms_storage_check_dupes(struct gbms_storage_provider *provider)
{
	return 0;
}

/* TODO: resolve references in the tag cache. Prefill the cache with the raw
 * mappings (TAG:<provider_name>:addr:size) for top-down organization.
 */
static int gbms_storage_resolve_refs(struct gbms_storage_provider *provider)
{
	/* enumerate the elements in cache, resolve references */
	return 0;
}

static int gbms_storage_find_slot(const char *name)
{
	int index;
	struct gbms_storage_provider *slot;

	for (index = 0; index < gbms_providers_count; index++) {
		slot = &gbms_providers[index];

		if (strncmp(slot->name, name, strlen(slot->name)) == 0) {
			if (!slot->dsc)
				break;

			return -EBUSY;
		}
	}

	if (index == GBMS_PROVIDERS_MAX)
		return -ENOMEM;

	return index;
}

int gbms_storage_register_internal(struct gbms_storage_desc *desc,
				   const char *name,
				   void *ptr)
{
	int index;
	unsigned long flags;
	int refs = 0, dupes = 0;
	struct gbms_storage_provider *slot;

	if (!name || strlen(name) >= GBMS_PROVIDER_NAME_MAX)
		return -EINVAL;

	spin_lock_irqsave(&providers_lock, flags);
	index = gbms_storage_find_slot(name);
	if (index < 0) {
		spin_unlock_irqrestore(&providers_lock, flags);
		return index;
	}

	slot = &gbms_providers[index];
	slot->name = name;
	slot->dsc = desc;
	slot->ptr = ptr;

	/* resolve refs and check dupes only on real providers */
	if (slot->dsc && desc) {
		/* will not check for self consistency */
		if (gbms_providers_count > 0)
			dupes = gbms_storage_check_dupes(slot);

		refs = gbms_storage_resolve_refs(slot);
	}

	pr_info("%s registered %s at %d, dupes=%d, refs=%d\n",
		(desc) ? "storage" : "ref",
		name,
		index,
		dupes, refs);

	if (index == gbms_providers_count)
		gbms_providers_count += 1;

#ifdef CONFIG_DEBUG_FS
	if (!IS_ERR_OR_NULL(rootdir) && name) {
		/* TODO: create debugfs entries for the providers */
	}
#endif
	spin_unlock_irqrestore(&providers_lock, flags);

	return 0;
}

int gbms_storage_register(struct gbms_storage_desc *desc, const char *name,
			  void *ptr)
{
	if (!desc)
		return -EINVAL;
	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;

	return gbms_storage_register_internal(desc, name, ptr);
}

/* ------------------------------------------------------------------------- */

static int gbms_cache_read(gbms_tag_t tag, void *data, size_t count)
{
	struct gbms_cache_entry *ce;
	struct gbms_storage_provider *slot;
	size_t addr = GBMS_STORAGE_ADDR_INVALID;
	int ret;

	/* the cache can only contain true providers */
	ce = gbms_cache_lookup(tag, &addr);
	if (!ce)
		return -ENOENT;

	slot = (struct gbms_storage_provider *)ce->provider;
	if (slot->offline)
		return -ENODEV;

	if (slot->dsc->fetch && addr != GBMS_STORAGE_ADDR_INVALID)
		ret = slot->dsc->fetch(data, addr, count, slot->ptr);
	else if (!slot->dsc->read)
		ret = -EINVAL;
	else
		ret = slot->dsc->read(tag, data, count, slot->ptr);

	return ret;
}

/* needs a lock on the provider */
int gbms_storage_read(gbms_tag_t tag, void *data, size_t count)
{
	int ret;
	bool late_inits = false;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;
	/* non-data transfers must have zero count and data */
	if (!data && count)
		return -EINVAL;

	ret = gbms_cache_read(tag, data, count);
	if (ret == -ENOENT) {
		const int max = gbms_providers_count;
		struct gbms_storage_desc *dsc;
		int i;

		for (i = 0, ret = -ENOENT; ret == -ENOENT && i < max; i++) {
			dsc = gbms_providers[i].dsc;
			if (!dsc) {
				late_inits = true;
			} else if (dsc->read) {
				/* -ENOENT = next, <0 err, >=0 #n bytes */
				ret = dsc->read(tag, data, count,
						gbms_providers[i].ptr);
				if (ret >= 0)
					gbms_cache_add(tag, &gbms_providers[i]);
			}
		}

	}

	if (late_inits && ret == -ENOENT)
		ret = -EPROBE_DEFER;

	return ret;
}

/* needs a lock on the provider */
int gbms_storage_read_data(gbms_tag_t tag, void *data, size_t count, int idx)
{
	struct gbms_storage_desc *dsc;
	const int max_count = gbms_providers_count;
	bool late_inits = false;
	int ret, i;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;
	if (!data && count)
		return -EINVAL;

	for (i = 0, ret = -ENOENT; ret == -ENOENT && i < max_count; i++) {
		if (gbms_providers[i].offline)
			continue;

		dsc = gbms_providers[i].dsc;
		if (!dsc) {
			late_inits = true;
		} else if (dsc->read_data) {
			/* -ENOENT = next, <0 err, >=0 #n bytes */
			ret = dsc->read_data(tag, data, count, idx,
					gbms_providers[i].ptr);

			/* TODO: cache the provider */
		}
	}

	if (late_inits && ret == -ENOENT)
		ret = -EPROBE_DEFER;

	return ret;
}

static int gbms_cache_write(gbms_tag_t tag, const void *data, size_t count)
{
	struct gbms_cache_entry *ce;
	struct gbms_storage_provider *slot;
	size_t addr = GBMS_STORAGE_ADDR_INVALID;
	int ret;

	ce = gbms_cache_lookup(tag, &addr);
	if (!ce)
		return -ENOENT;

	slot = (struct gbms_storage_provider *)ce->provider;
	if (slot->offline)
		return -ENODEV;

	if (slot->dsc->store && addr != GBMS_STORAGE_ADDR_INVALID)
		ret = slot->dsc->store(data, addr, count, slot->ptr);
	else if (!slot->dsc->write)
		ret = -EINVAL;
	else
		ret = slot->dsc->write(tag, data, count, slot->ptr);

	return ret;
}

/* needs a lock on the provider */
int gbms_storage_write(gbms_tag_t tag, const void *data, size_t count)
{
	int ret;
	bool late_inits = false;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;
	if (!data && count)
		return -EINVAL;

	ret = gbms_cache_write(tag, data, count);
	if (ret == -ENOENT) {
		const int max = gbms_providers_count;
		struct gbms_storage_desc *dsc;
		int i;

		for (i = 0, ret = -ENOENT; ret == -ENOENT && i < max; i++) {
			if (gbms_providers[i].offline)
				continue;

			dsc = gbms_providers[i].dsc;
			if (!dsc) {
				late_inits = true;
			} else if (dsc->write) {
				/* -ENOENT = next, <0 err, >=0 #n bytes */
				ret = dsc->write(tag, data, count,
						gbms_providers[i].ptr);
				if (ret >= 0)
					gbms_cache_add(tag, &gbms_providers[i]);
			}

		}
	}

	if (late_inits && ret == -ENOENT)
		ret = -EPROBE_DEFER;

	return ret;
}

/* needs a lock on the provider */
int gbms_storage_write_data(gbms_tag_t tag, const void *data, size_t count,
			    int idx)
{
	const int max_count = gbms_providers_count;
	struct gbms_storage_desc *dsc;
	bool late_inits = false;
	int ret, i;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;
	if (!data && count)
		return -EINVAL;

	for (i = 0, ret = -ENOENT; ret == -ENOENT && i < max_count; i++) {
		if (gbms_providers[i].offline)
			continue;

		dsc = gbms_providers[i].dsc;
		if (!dsc) {
			late_inits = true;
		} else if (dsc->write_data) {
			/* -ENOENT = next, <0 err, >=0 #n bytes */
			ret = dsc->write_data(tag, data, count, idx,
					gbms_providers[i].ptr);

			/* TODO: cache the provider */
		}

	}

	if (late_inits && ret == -ENOENT)
		ret = -EPROBE_DEFER;

	return ret;
}

static int gbms_storage_flush_provider(struct gbms_storage_provider *slot,
				       bool force)
{
	if (slot->offline)
		return 0;

	if (!slot->dsc || !slot->dsc->flush)
		return 0;

	return slot->dsc->flush(force, slot->ptr);
}

static int gbms_storage_flush_all_internal(bool force)
{
	int ret, i;
	bool success = true;
	struct gbms_storage_provider *slot;

	for (i = 0; i < gbms_providers_count ; i++) {
		slot = &gbms_providers[i];

		ret = gbms_storage_flush_provider(slot, force);
		if (ret < 0) {
			pr_err("flush of %s failed (%d)\n", slot->name, ret);
			success = false;
		}
	}

	return success ? 0 : -EIO;
}

int gbms_storage_flush(gbms_tag_t tag)
{
	unsigned long flags;
	int ret;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;

	spin_lock_irqsave(&providers_lock, flags);

	/* TODO: search for the provider */

	ret = gbms_storage_flush_all_internal(false);
	spin_unlock_irqrestore(&providers_lock, flags);

	return 0;
}

int gbms_storage_flush_all(void)
{
	unsigned long flags;
	int ret;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;

	spin_lock_irqsave(&providers_lock, flags);
	ret = gbms_storage_flush_all_internal(false);
	spin_unlock_irqrestore(&providers_lock, flags);

	return ret;
}

int gbms_storage_offline(const char *name, bool flush)
{
	unsigned long flags;
	int ret = 0, index;

	if (!gbms_storage_init_done)
		return -EPROBE_DEFER;

	spin_lock_irqsave(&providers_lock, flags);
	index = gbms_storage_find_slot(name);
	if (index < 0) {
		spin_unlock_irqrestore(&providers_lock, flags);
		return index;
	}

	if (flush)
		ret = gbms_storage_flush_provider(&gbms_providers[index],
						  false);
	if (ret == 0)
		gbms_providers[index].offline = true;

	spin_unlock_irqrestore(&providers_lock, flags);
	return ret;
}

/* ------------------------------------------------------------------------ */
#ifdef CONFIG_DEBUG_FS

static int gbms_storage_show_cache(struct seq_file *m, void *data)
{
	int bucket;
	gbms_tag_cstr_t tname;
	unsigned long flags;
	struct gbms_cache_entry *ce;
	struct gbms_storage_provider *slot;

	spin_lock_irqsave(&providers_lock, flags);

	hash_for_each(gbms_cache, bucket, ce, hnode) {

		slot = (struct gbms_storage_provider *)ce->provider;
		seq_printf(m, slot->offline ? " (%s): %s" : " %s: %s",
			   slot->name, tag2cstr(tname, ce->tag));

		if (ce->count != 0)
			seq_printf(m, "[%d:%d]", ce->addr, ce->count);
		seq_printf(m, "\n");
	}

	spin_unlock_irqrestore(&providers_lock, flags);
	return 0;
}

static int gbms_storage_cache_open(struct inode *inode, struct file *file)
{
	return single_open(file, gbms_storage_show_cache, inode->i_private);
}
static const struct file_operations gbms_cache_status_ops = {
	.owner		= THIS_MODULE,
	.open		= gbms_storage_cache_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void gbms_show_storage_provider(struct seq_file *m,
				       struct gbms_storage_provider *slot,
				       bool verbose)
{
	gbms_tag_cstr_t tname;
	gbms_tag_t tag;
	int ret = 0, i;

	for (i = 0 ; ret == 0; i++) {
		ret = slot->dsc->iter(i, &tag, slot->ptr);
		if (ret < 0)
			break;

		seq_printf(m, "%s ", tag2cstr(tname, tag));

		if (verbose && slot->dsc->info) {
			size_t addr, count;

			ret = slot->dsc->info(tag, &addr, &count, slot->ptr);
			if (ret < 0)
				continue;

			seq_printf(m, "[%d,%d] ", addr, count);
		}
	}
}

static int gbms_show_storage_clients(struct seq_file *m, void *data)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&providers_lock, flags);

	for (i = 0; i < gbms_providers_count; i++) {

		seq_printf(m, gbms_providers[i].offline ? "%d (%s):" : "%d %s:",
			   i, gbms_providers[i].name);

		if (!gbms_providers[i].dsc || !gbms_providers[i].dsc->iter)
			continue;

		gbms_show_storage_provider(m, &gbms_providers[i], false);

		seq_printf(m, "\n");
	}

	spin_unlock_irqrestore(&providers_lock, flags);

	return 0;
}

static int gbms_storage_clients_open(struct inode *inode, struct file *file)
{
	return single_open(file, gbms_show_storage_clients, inode->i_private);
}

static const struct file_operations gbms_providers_status_ops = {
	.owner		= THIS_MODULE,
	.open		= gbms_storage_clients_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define GBMS_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.owner	= THIS_MODULE,			\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

static ssize_t debug_set_offline(struct file *filp,
				 const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	char name[GBMS_PROVIDER_NAME_MAX];
	int ret;

	ret = simple_write_to_buffer(name, sizeof(name), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	ret = gbms_storage_offline(name, true);
	if (ret == 0)
		ret = count;

	return 0;

}

GBMS_DEBUG_ATTRIBUTE(gbms_providers_offline_ops, NULL, debug_set_offline);

static int debug_set_tag_size(void *data, u64 val)
{
	struct gbms_cache_entry *ce = data;

	ce->count = val;
	return 0;
}

static int debug_show_tag_size(void *data, u64 *val)
{
	struct gbms_cache_entry *ce = data;

	*val = ce->count;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_tag_size_ops, debug_show_tag_size,
			debug_set_tag_size, "%llu\n");

static ssize_t debug_read_tag_data(struct file *filp,
				   char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct gbms_cache_entry *ce = filp->private_data;
	char *buf;
	int ret;

	if (!ce->count)
		return -ENODATA;

	buf = kzalloc(sizeof(char) * ce->count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = gbms_storage_read(ce->tag, buf, ce->count);
	if (ret < 0)
		goto rtag_free_mem;

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ce->count);

rtag_free_mem:
	kfree(buf);

	return ret;
}

static ssize_t debug_write_tag_data(struct file *filp,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct gbms_cache_entry *ce = filp->private_data;
	char *buf;
	int ret;

	if (!ce->count)
		return -ENODATA;

	buf = kzalloc(sizeof(char) * ce->count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, ce->count, ppos, user_buf, count);
	if (!ret) {
		ret = -EFAULT;
		goto wtag_free_mem;
	}

	ret = gbms_storage_write(ce->tag, buf, ce->count);

wtag_free_mem:
	kfree(buf);

	return (ret < 0) ? ret : count;
}

GBMS_DEBUG_ATTRIBUTE(debug_tag_data_ops, debug_read_tag_data,
		     debug_write_tag_data);

static int gbms_find(struct gbms_storage_provider *slot, gbms_tag_t tag)
{
	int ret = 0, i;
	gbms_tag_t tmp;

	for (i = 0; ret == 0; i++) {
		ret = slot->dsc->iter(i, &tmp, slot->ptr);
		if (ret < 0)
			break;
		if (tag == tmp)
			return 1;
	}

	return 0;
}

static struct gbms_cache_entry *gbms_cache_preload_tag(gbms_tag_t tag)
{
	struct gbms_storage_provider *slot = NULL;
	int ret, i;

	for (i = 0; !slot && i < gbms_providers_count; i++) {
		struct gbms_storage_desc *dsc;

		dsc = gbms_providers[i].dsc;
		if (!dsc || !dsc->iter)
			continue;

		ret = gbms_find(&gbms_providers[i], tag);
		if (ret == 1)
			slot = &gbms_providers[i];
	}

	return gbms_cache_add(tag, slot);
}


/* [ugo]+TAG_NAME | -TAG_NAME | TAG_NAME */
static ssize_t debug_export_tag(struct file *filp,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	size_t addr = GBMS_STORAGE_ADDR_INVALID;
	struct gbms_cache_entry *ce;
	gbms_tag_cstr_t name = { 0 };
	struct dentry *de;
	gbms_tag_t tag;
	char temp[32];
	int ret;

	if (!rootdir)
		return -ENODEV;

	ret = simple_write_to_buffer(temp, sizeof(temp), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	if (temp[0] == '-') {
		/* remove tag */
		return -EINVAL;
	}

	memcpy(name, temp + (temp[0] == '+'), 4);
	tag = cstr2tag(name);

	ce = gbms_cache_lookup(tag, &addr);
	if (!ce) {
		ce = gbms_cache_preload_tag(tag);
		if (!ce)
			return -ENOMEM;
	}

	de = debugfs_create_dir(name, rootdir);
	if (!de) {
		pr_err("cannot create debufsentry for %s\n", name);
		return -ENODEV;
	}

	debugfs_create_file("size", 0400, de, ce, &debug_tag_size_ops);
	debugfs_create_file("data", 0600, de, ce, &debug_tag_data_ops);

	return count;
}

GBMS_DEBUG_ATTRIBUTE(gbms_providers_export_ops, NULL, debug_export_tag);

#endif

/* ------------------------------------------------------------------------ */

struct gbms_storage_device {
	struct gbms_cache_entry entry;
	loff_t index;
	loff_t count;

	struct mutex gdev_lock;
	int hcmajor;
	struct cdev hcdev;
	struct class *hcclass;
	bool available;
	bool added;

	void (*show_fn)(struct seq_file *s, const u8 *data, size_t count);
};

struct gbms_storage_device_seq {
	struct gbms_storage_device *gbms_device;
	u8 seq_show_buffer[];
};

static void *ct_seq_start(struct seq_file *s, loff_t *pos)
{
	int ret;
	struct gbms_storage_device_seq *gdev_seq =
		(struct gbms_storage_device_seq *)s->private;
	struct gbms_storage_device *gdev = gdev_seq->gbms_device;

	ret = gbms_storage_read_data(gdev->entry.tag, NULL, 0, 0);
	if (ret < 0) {
		gbms_tag_cstr_t buff;

		pr_err("cannot init %s iterator data (%d)\n",
		       tag2cstr(buff, gdev->entry.tag), ret);
		return NULL;
	}

	if (*pos >= ret)
		return NULL;

	gdev->count = ret;
	gdev->index = *pos;

	return &gdev->index;
}

static void *ct_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	loff_t *spos = (loff_t *)v;
	struct gbms_storage_device_seq *gdev_seq =
		(struct gbms_storage_device_seq *)s->private;
	struct gbms_storage_device *gdev = gdev_seq->gbms_device;

	*pos = ++*spos;
	if (*pos >= gdev->count)
		 spos = NULL;

	return spos;
}

static void ct_seq_stop(struct seq_file *s, void *v)
{
	int ret;
	struct gbms_storage_device_seq *gdev_seq =
		(struct gbms_storage_device_seq *)s->private;
	struct gbms_storage_device *gdev = gdev_seq->gbms_device;

	ret = gbms_storage_read_data(gdev->entry.tag, NULL, 0,
				     GBMS_STORAGE_INDEX_INVALID);
	if (ret < 0) {
		gbms_tag_cstr_t buff;

		pr_err("cannot free %s iterator data (%d)\n",
		       tag2cstr(buff, gdev->entry.tag), ret);
	}
}

static int ct_seq_show(struct seq_file *s, void *v)
{
	struct gbms_storage_device_seq *gdev_seq =
		(struct gbms_storage_device_seq *)s->private;
	struct gbms_storage_device *gdev = gdev_seq->gbms_device;
	loff_t *spos = (loff_t *)v;
	int ret;

	ret = gbms_storage_read_data(gdev->entry.tag, gdev_seq->seq_show_buffer,
				     gdev->entry.count, *spos);
	if (ret < 0)
		return ret;

	if (gdev->show_fn)
		gdev->show_fn(s, gdev_seq->seq_show_buffer, ret);

	return 0;
}

static const struct seq_operations ct_seq_ops = {
	.start = ct_seq_start,
	.next  = ct_seq_next,
	.stop  = ct_seq_stop,
	.show  = ct_seq_show
};

static int gbms_storage_dev_open(struct inode *inode, struct file *file)
{
	int ret;
	struct gbms_storage_device *gdev =
		container_of(inode->i_cdev, struct gbms_storage_device, hcdev);

	ret = seq_open(file, &ct_seq_ops);
	if (ret == 0) {
		struct seq_file *seq = file->private_data;
		struct gbms_storage_device_seq *gdev_seq;

		seq->private = kzalloc(sizeof(struct gbms_storage_device_seq) +
				       gdev->entry.count, GFP_KERNEL);
		if (!seq->private)
			return -ENOMEM;

		gdev_seq = (struct gbms_storage_device_seq *)seq->private;
		gdev_seq->gbms_device = gdev;
	}

	return ret;
}

static int gbms_storage_dev_release(struct inode *inode, struct file *file)
{
	struct seq_file *seq = file->private_data;

	kfree(seq->private);

	return seq_release(inode, file);
}

static const struct file_operations hdev_fops = {
	.open = gbms_storage_dev_open,
	.owner = THIS_MODULE,
	.read = seq_read,
	.release = gbms_storage_dev_release,
};

void gbms_storage_cleanup_device(struct gbms_storage_device *gdev)
{
	if (gdev->added)
		cdev_del(&gdev->hcdev);
	if (gdev->available)
		device_destroy(gdev->hcclass, gdev->hcmajor);
	if (gdev->hcclass)
		class_destroy(gdev->hcclass);
	if (gdev->hcmajor != -1)
		unregister_chrdev_region(gdev->hcmajor, 1);
	kfree(gdev);
}

static int gbms_storage_device_init(struct gbms_storage_device *gdev,
				    const char *name)
{
	struct device *hcdev;

	mutex_init(&gdev->gdev_lock);
	gdev->hcmajor = -1;

	/* cat /proc/devices */
	if (alloc_chrdev_region(&gdev->hcmajor, 0, 1, name) < 0)
		goto no_gdev;
	/* ls /sys/class */
	gdev->hcclass = class_create(THIS_MODULE, name);
	if (gdev->hcclass == NULL)
		goto no_gdev;
	/* ls /dev/ */
	hcdev = device_create(gdev->hcclass, NULL, gdev->hcmajor, NULL, name);
	if (hcdev == NULL)
		goto no_gdev;

	gdev->available = true;
	cdev_init(&gdev->hcdev, &hdev_fops);
	if (cdev_add(&gdev->hcdev, gdev->hcmajor, 1) == -1)
		goto no_gdev;

	gdev->added = true;
	return 0;

no_gdev:
	gbms_storage_cleanup_device(gdev);
	return -ENODEV;
}

static void ct_dev_show(struct seq_file *s, const u8 *d, size_t count)
{
	int i;
	u16 *data = (u16 *)d;

	for (i = 0; i < count / 2; i++)
		seq_printf(s, "%04x ", data[i]);
	seq_printf(s, "\n");
}

struct gbms_storage_device *gbms_storage_create_device(const char *name,
						       gbms_tag_t tag)
{
	int i, ret;
	struct gbms_storage_device *gdev;
	const int max_count = gbms_providers_count;

	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		return NULL;

	ret = gbms_storage_device_init(gdev, name);
	if (ret < 0)
		return NULL;

	for (ret = -ENOENT, i = 0; ret == -ENOENT && i < max_count; i++) {
		size_t addr, count;
		struct gbms_storage_desc *dsc;

		dsc = gbms_providers[i].dsc;
		if (!dsc || !dsc->info)
			continue;

		ret = dsc->info(tag, &addr, &count, gbms_providers[i].ptr);
		if (ret == 0) {
			gdev->entry.provider = &gbms_providers[i];
			gdev->entry.count = count;
			gdev->entry.addr = addr;
			gdev->entry.tag = tag;
		}
	}

	if  (!gdev->entry.provider) {
		gbms_storage_cleanup_device(gdev);
		return NULL;
	}

	/* TODO: caller to customize */
	gdev->show_fn = ct_dev_show;

	return gdev;
}

/* ------------------------------------------------------------------------ */

#define entry_size(x) (ilog2(x) + (((x) & ((x) - 1)) != 0))

static void gbms_storage_parse_provider_refs(struct device_node *node)
{
	const char *s;
	int i, ret, count;

	count = of_property_count_strings(node, "google,gbms-providers");
	if (count < 0)
		return;

	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(node,
						    "google,gbms-providers",
						    i, &s);
		if (ret < 0) {
			pr_err("cannot parse index %d\n", i);
			continue;
		}

		ret = gbms_storage_register_internal(NULL, s, NULL);
		if (ret < 0)
			pr_err("cannot add a reference to %s (%d)\n", s, ret);
	}
}

static int __init gbms_storage_init(void)
{
	struct device_node *node;
	const int pe_size = entry_size(sizeof(struct gbms_cache_entry));

	pr_info("initialize gbms_storage\n");

	spin_lock_init(&providers_lock);

	gbms_cache_pool = gen_pool_create(pe_size, -1);
	if (gbms_cache_pool) {
		size_t mem_size = (1 << GBMS_HASHTABLE_SIZE) * pe_size;

		gbms_cache_mem = kzalloc(mem_size, GFP_KERNEL);
		if (!gbms_cache_mem) {
			gen_pool_destroy(gbms_cache_pool);
			gbms_cache_pool = NULL;
		} else {
			gen_pool_add(gbms_cache_pool,
				     (unsigned long)gbms_cache_mem,
				     mem_size, -1);
			hash_init(gbms_cache);
		}
	}

	if (!gbms_cache_pool)
		pr_err("unable to create cache\n");

	node = of_find_node_by_name(NULL, "google_bms");
	if (node) {
		/* TODO: prefill cache with static entries for top-down.
		 * NOTE: providers for top-down tags make the late_init list
		 * as well
		 */

		gbms_storage_parse_provider_refs(node);
	}

	gbms_storage_init_done = true;
	pr_info("init done\n");

#ifdef CONFIG_DEBUG_FS
	rootdir = debugfs_create_dir("gbms_storage", NULL);
	if (!IS_ERR_OR_NULL(rootdir)) {
		debugfs_create_file("cache", S_IFREG | 0444, rootdir,
				    NULL, &gbms_cache_status_ops);
		debugfs_create_file("providers", S_IFREG | 0444, rootdir,
				    NULL, &gbms_providers_status_ops);
		debugfs_create_file("offline", S_IFREG | 0200, rootdir,
				    NULL, &gbms_providers_offline_ops);
		debugfs_create_file("export", S_IFREG | 0200, rootdir,
				    NULL, &gbms_providers_export_ops);
	}
#endif

	return 0;
}

static void __exit gbms_storage_exit(void)
{
	int ret;

#ifdef CONFIG_DEBUG_FS
	if (!IS_ERR_OR_NULL(rootdir))
		debugfs_remove(rootdir);
#endif

	ret = gbms_storage_flush_all_internal(true);
	if (ret < 0)
		pr_err("flush all failed");

	if (gbms_cache_pool) {
		gen_pool_destroy(gbms_cache_pool);
		kfree(gbms_cache_mem);
	}

	gbms_providers_count = 0;
}

module_init(gbms_storage_init);
module_exit(gbms_storage_exit);
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_DESCRIPTION("Google BMS Storage");
