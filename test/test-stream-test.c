// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for struct test_stream.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/test.h>
#include <test/mock.h>
#include <test/test-stream.h>

#include "test-mock.h"

struct test_stream_test_context {
	struct MOCK(test)	*mock_test;
	struct test_stream	*stream;
};

static void test_stream_test_add(struct test *test)
{
	struct test_stream_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = NICE_MOCK(ctx->mock_test);
	struct test_stream *stream = ctx->stream;

	stream->add(stream, "Foo");
	stream->add(stream, " %s", "bar");
	stream->set_level(stream, KERN_INFO);

	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
					  any(test),
					  va_format_cmp(test,
							streq(test, "Foo bar"),
							any(test))));

	stream->commit(stream);
}

static void test_stream_test_append(struct test *test)
{
	struct test_stream_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = NICE_MOCK(ctx->mock_test);
	struct test_stream *stream = ctx->stream;
	struct test_stream *other_stream;

	stream->add(stream, "Foo");
	stream->set_level(stream, KERN_INFO);
	other_stream = test_new_stream(mock_get_trgt(mock_test));
	other_stream->add(other_stream, " %s", "bar");

	stream->append(stream, other_stream);
	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
					  any(test),
					  va_format_cmp(test,
							streq(test, "Foo bar"),
							any(test))));

	stream->commit(stream);
}

static void test_stream_error_message_when_no_level_set(struct test *test)
{
	struct test_stream_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = NICE_MOCK(ctx->mock_test);
	struct test_stream *stream = ctx->stream;
	struct test_stream *other_stream;

	stream->add(stream, "Foo bar");
	other_stream = test_new_stream(mock_get_trgt(mock_test));

	stream->append(stream, other_stream);
	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
				 any(test),
				 va_format_cmp(test,
					       streq(test,
						     "Stream was committed without a specified log level."),
					       any(test))));
	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
				any(test),
				va_format_cmp(test,
					streq(test, "Foo bar"),
					any(test))));
	stream->commit(stream);
}

static int test_stream_test_init(struct test *test)
{
	struct mock_struct_formatter_entry *entries;
	struct test_stream_test_context *ctx;

	ctx = test_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	test->priv = ctx;

	ctx->mock_test = CONSTRUCT_MOCK(test, test);
	if (!ctx->mock_test)
		return -EINVAL;

	ctx->stream = test_new_stream(mock_get_trgt(ctx->mock_test));
	if (!ctx->stream)
		return -ENOMEM;

	entries = test_kzalloc(test, sizeof(*entries) * 3, GFP_KERNEL);
	if (!entries) {
		test_warn(test,
			  "Could not allocate arg formatter for struct va_format");
		return 0;
	}

	INIT_MOCK_STRUCT_FORMATTER_ENTRY(&entries[0],
					 struct va_format,
					 fmt,
					 FORMATTER_FROM_TYPE(const char *));
	INIT_MOCK_STRUCT_FORMATTER_ENTRY(&entries[1],
					 struct va_format,
					 va,
					 unknown_formatter);
	INIT_MOCK_STRUCT_FORMATTER_ENTRY_LAST(&entries[2]);

	mock_register_formatter(mock_struct_formatter(test,
						      "struct va_format *",
						      entries));

	return 0;
}

static void test_stream_test_commits_any_uncommitted_when_cleanup(
		struct test *test)
{
	struct test_stream_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = NICE_MOCK(ctx->mock_test);
	struct test_stream *stream = ctx->stream;

	stream->add(stream, "Hello World");
	stream->set_level(stream, KERN_WARNING);

	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
				 any(test),
				 va_format_cmp(test,
					       streq(test,
						     "End of test case reached with uncommitted stream entries."),
					       any(test))));
	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test),
			any(test),
			va_format_cmp(test,
				streq(test, "Hello World"),
				any(test))));
	test_cleanup(mock_get_trgt(mock_test));
}

static void test_stream_test_exit(struct test *test)
{
	mock_unregister_formatter(mock_find_formatter("struct va_format *"));
}

static struct test_case test_stream_test_cases[] = {
	TEST_CASE(test_stream_test_add),
	TEST_CASE(test_stream_test_append),
	TEST_CASE(test_stream_test_commits_any_uncommitted_when_cleanup),
	TEST_CASE(test_stream_error_message_when_no_level_set),
	{},
};

static struct test_module test_stream_test_module = {
	.name = "test-stream-test",
	.init = test_stream_test_init,
	.exit = test_stream_test_exit,
	.test_cases = test_stream_test_cases,
};
module_test(test_stream_test_module);
