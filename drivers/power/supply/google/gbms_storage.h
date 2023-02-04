/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright (C) 2018 Google Inc.
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

#ifndef __GBMS_STORAGE_H__
#define __GBMS_STORAGE_H__

/**
 * GBMS Storage API
 * The API provides functions to access to data stored in the persistent and
 * semi-persistent storage of a device in a cross-platform and
 * location-independent fashion. Clients in kernel and userspace use this
 * directly and indirectly to retrieve battery serial number, cell chemistry
 * type, cycle bin count, battery lifetime history and other battery related
 * data.
 */

#define GBMS_STORAGE_ADDR_INVALID	-1
#define GBMS_STORAGE_INDEX_INVALID	-1

/* Battery Google Part Number */
#define GBMS_BGPN_LEN	10
/* Battery manufacturer info length */
#define GBMS_MINF_LEN	32
/* Battery device info length */
#define GBMS_DINF_LEN	32
/* Battery cycle count bin length */
#define GBMS_CNTB_LEN	16
/* Battery consistent soc length */
#define GBMS_RSOC_LEN	2

/**
 * Tags are u32 constants: hardcoding as hex since characters constants of more
 * than one byte such as 'BGCE' are frown upon.
 */
typedef uint32_t gbms_tag_t;

enum gbms_tags {
	GBMS_TAG_BGCE = 0x42474345,
	GBMS_TAG_BCNT = 0x42434e54,
	GBMS_TAG_BRES = 0x42524553,
	GBMS_TAG_SNUM = 0x534e554d,
	GBMS_TAG_HIST = 0x48495354,
	GBMS_TAG_BRID = 0x42524944,
	GBMS_TAG_DSNM = 0x44534e4d,
	GBMS_TAG_MINF = 0x4d494e46,
	GBMS_TAG_DINF = 0x44494e46,
	GBMS_TAG_BGPN = 0x4247504e,
	GBMS_TAG_CNTB = 0x434e5442,
	GBMS_TAG_RSOC = 0x52534F43,
};

/**
 * struct gbms_storage_desc - callbacks for a GBMS storage provider.
 *
 * Fields not used should be initialized with NULL. The provider name and the
 * iter callback are optional but strongly recommended. The write, fetch, store
 * and flush callbacks are optional, descriptors with a non NULL write/store
 * callback should have a non NULL read/fetch callback.
 *
 * The iterator callback (iter) is used to list the tags stored in the provider
 * and can be used to detect duplicates. The list of tags exported from iter
 * can be expected to be static (i.e. tags can be enumerated once on
 * registration).
 *
 * The read and write callbacks transfer the data associated with a tag. The
 * calls must return -ENOENT when called with a tag that is not known to the
 * provider, a negative number on every other error or the number of bytes
 * read or written to the device. The tag lookup for the read and write
 * callbacks must be very efficient (i.e. consider implementation that use hash
 * or switch statements).
 *
 * Fetch and store callbacks are used to grant non-mediated access to a range
 * of consecutive addresses in storage space. The implementation must return a
 * negative number on error or the number of bytes transferred with the
 * operation. Support caching of the tag data location requires non NULL fetch
 * and not NULL info callbacks.
 *
 * The read_data and write_data callbacks transfer the data associated with an
 * enumerator. The calls must return -ENOENT when called with a tag that is not
 * known to the provider, a negative number on every other error or the number
 * of bytes read or written to the device during data transfers.
 *
 * Clients can only access keys that are available on a device (i.e. clients
 * cannot create new tags) and the API returns -ENOENT when trying to access a
 * tag that is not available on a device, -EGAIN while the storage is not fully
 * initialized.
 *
 * @iter: callback, return the tags known from this provider
 * @info: callback, return address and size for tags (used for caching)
 * @read: callback, read data from a tag
 * @write: callback, write data to a tag
 * @fetch: callback, read up to count data bytes from an address
 * @store: callback, write up to count data bytes to an address
 * @flush: callback, request a fush of data to permanent storage
 * @read_data: callback, read the elements of an enumerations
 * @write_data: callback, write to the elements of an enumeration
 */
struct gbms_storage_desc {
	int (*iter)(int index, gbms_tag_t *tag, void *ptr);
	int (*info)(gbms_tag_t tag, size_t *addr, size_t *size, void *ptr);
	int (*read)(gbms_tag_t tag, void *data, size_t count, void *ptr);
	int (*write)(gbms_tag_t tag, const void *data, size_t count, void *ptr);
	int (*fetch)(void *data, size_t addr, size_t count, void *ptr);
	int (*store)(const void *data, size_t addr, size_t count, void *ptr);
	int (*flush)(bool force, void *ptr);

	int (*read_data)(gbms_tag_t tag, void *data, size_t count, int idx,
			 void *ptr);
	int (*write_data)(gbms_tag_t tag, const void *data, size_t count,
			  int idx, void *ptr);
};

int gbms_storage_register(struct gbms_storage_desc *desc, const char *name,
			  void *ptr);
int gbms_storage_offline(const char *name, bool flush);

int gbms_storage_read(gbms_tag_t tag, void *data, size_t count);
int gbms_storage_write(gbms_tag_t tag, const void *data, size_t count);

int gbms_storage_read_data(gbms_tag_t tag, void *data, size_t count, int idx);
int gbms_storage_write_data(gbms_tag_t tag, const void *data, size_t count,
			    int idx);
int gbms_storage_flush(gbms_tag_t tag);
int gbms_storage_flush_all(void);

struct gbms_storage_device;

/* standard device implementation that read data from an enumeration */
struct gbms_storage_device *gbms_storage_create_device(const char *name,
						       gbms_tag_t tag);
void gbms_storage_cleanup_device(struct gbms_storage_device *gdev);

#endif /* __GBMS_STORAGE_H__ */