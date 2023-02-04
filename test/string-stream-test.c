// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for struct string_stream.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <linux/slab.h>
#include <test/test.h>
#include <test/string-stream.h>

static void string_stream_test_get_string(struct test *test)
{
	struct string_stream *stream = new_string_stream();
	char *output;

	stream->add(stream, "Foo");
	stream->add(stream, " %s", "bar");

	output = stream->get_string(stream);
	ASSERT_STREQ(test, output, "Foo bar");
	kfree(output);
	destroy_string_stream(stream);
}

static void string_stream_test_add_and_clear(struct test *test)
{
	struct string_stream *stream = new_string_stream();
	char *output;
	int i;

	for (i = 0; i < 10; i++)
		stream->add(stream, "A");

	output = stream->get_string(stream);
	ASSERT_STREQ(test, output, "AAAAAAAAAA");
	ASSERT_EQ(test, stream->length, 10);
	ASSERT_FALSE(test, stream->is_empty(stream));
	kfree(output);

	stream->clear(stream);

	output = stream->get_string(stream);
	ASSERT_STREQ(test, output, "");
	ASSERT_TRUE(test, stream->is_empty(stream));
	destroy_string_stream(stream);
}

static struct test_case string_stream_test_cases[] = {
	TEST_CASE(string_stream_test_get_string),
	TEST_CASE(string_stream_test_add_and_clear),
	{}
};

static struct test_module string_stream_test_module = {
	.name = "string-stream-test",
	.test_cases = string_stream_test_cases
};
module_test(string_stream_test_module);

