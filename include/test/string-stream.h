/* SPDX-License-Identifier: GPL-2.0 */
/*
 * C++ stream style string builder used in KUnit for building messages.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#ifndef _TEST_STRING_STREAM_H
#define _TEST_STRING_STREAM_H

#include <linux/types.h>
#include <stdarg.h>

struct string_stream_fragment {
	struct list_head node;
	char *fragment;
};

struct string_stream {
	size_t length;
	struct list_head fragments;

	int (*add)(struct string_stream *this, const char *fmt, ...);
	int (*vadd)(struct string_stream *this, const char *fmt, va_list args);
	char *(*get_string)(struct string_stream *this);
	void (*clear)(struct string_stream *this);
	bool (*is_empty)(struct string_stream *this);
};

struct string_stream *new_string_stream(void);

void destroy_string_stream(struct string_stream *stream);

#endif /* _TEST_STRING_STREAM_H */
