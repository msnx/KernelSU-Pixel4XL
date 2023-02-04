/* SPDX-License-Identifier: GPL-2.0 */
/*
 * C++ stream style string formatter and printer used in KUnit for outputting
 * KUnit messages.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#ifndef _TEST_TEST_STREAM_H
#define _TEST_TEST_STREAM_H

#include <linux/types.h>
#include <test/string-stream.h>

struct test;

/**
 * struct test_stream - a std::stream style string builder.
 * @set_level: sets the level that this string should be printed at.
 * @add: adds the formatted input to the internal buffer.
 * @commit: prints out the internal buffer to the user.
 * @clear: clears the internal buffer.
 *
 * A std::stream style string builder. Allows messages to be built up and
 * printed all at once.
 */
struct test_stream {
	void (*set_level)(struct test_stream *this, const char *level);
	void (*add)(struct test_stream *this, const char *fmt, ...);
	void (*append)(struct test_stream *this, struct test_stream *other);
	void (*commit)(struct test_stream *this);
	void (*clear)(struct test_stream *this);
	/* private: internal use only. */
	struct test *test;
	const char *level;
	struct string_stream *internal_stream;
};

/**
 * test_new_stream() - constructs a new &struct test_stream.
 * @test: The test context object.
 *
 * Constructs a new test managed &struct test_stream.
 */
struct test_stream *test_new_stream(struct test *test);

#endif /* _TEST_TEST_STREAM_H */
