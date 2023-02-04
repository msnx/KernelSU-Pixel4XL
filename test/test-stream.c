// SPDX-License-Identifier: GPL-2.0
/*
 * C++ stream style string formatter and printer used in KUnit for outputting
 * KUnit messages.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/test.h>
#include <test/test-stream.h>
#include <test/string-stream.h>

static void test_stream_set_level(struct test_stream *this,
				  const char *level)
{
	this->level = level;
}

static void test_stream_add(struct test_stream *this, const char *fmt, ...)
{
	va_list args;
	struct string_stream *stream = this->internal_stream;

	va_start(args, fmt);
	if (stream->vadd(stream, fmt, args) < 0)
		test_err(this->test, "Failed to allocate fragment: %s", fmt);

	va_end(args);
}

static void test_stream_append(struct test_stream *this,
			       struct test_stream *other)
{
	struct string_stream *other_stream = other->internal_stream;
	const char *other_content;

	other_content = other_stream->get_string(other_stream);

	if (!other_content) {
		test_err(this->test,
			 "Failed to get string from second argument for appending.");
		return;
	}

	this->add(this, other_content);
}

static void test_stream_clear(struct test_stream *this)
{
	this->internal_stream->clear(this->internal_stream);
}

static void test_stream_commit(struct test_stream *this)
{
	char *buf;
	struct string_stream_fragment *fragment;
	struct string_stream *stream = this->internal_stream;

	if (!this->level) {
		test_err(this->test,
			 "Stream was committed without a specified log level.");
		this->set_level(this, KERN_ERR);
	}

	buf = stream->get_string(stream);
	if (!buf) {
		test_err(this->test,
			 "Could not allocate buffer, dumping stream:");
		list_for_each_entry(fragment, &stream->fragments, node) {
			test_err(this->test, fragment->fragment);
		}
		goto cleanup;
	}

	test_printk(this->level, this->test, buf);
	kfree(buf);

cleanup:
	this->clear(this);
}

static int test_stream_init(struct test_resource *res, void *context)
{
	struct test_stream *stream;
	struct test *test = context;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;
	res->allocation = stream;
	stream->test = test;
	stream->level = NULL;
	stream->internal_stream = new_string_stream();

	if (!stream->internal_stream)
		return -ENOMEM;

	stream->set_level = test_stream_set_level;
	stream->add = test_stream_add;
	stream->append = test_stream_append;
	stream->commit = test_stream_commit;
	stream->clear = test_stream_clear;
	return 0;
}

static void test_stream_free(struct test_resource *res)
{
	struct test_stream *stream = res->allocation;

	if (!stream->internal_stream->is_empty(stream->internal_stream)) {
		test_err(stream->test,
			 "End of test case reached with uncommitted stream entries.");
		stream->commit(stream);
	}

	destroy_string_stream(stream->internal_stream);
	kfree(stream);
}

struct test_stream *test_new_stream(struct test *test)
{
	struct test_resource *res;

	res = test_alloc_resource(test,
				  test_stream_init,
				  test_stream_free,
				  test);

	if (res)
		return res->allocation;
	else
		return NULL;
}
