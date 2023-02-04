// SPDX-License-Identifier: GPL-2.0
/*
 * Example KUnit test to show how to use KUnit.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/test.h>
#include <test/mock.h>

struct example {
	int (*foo)(struct example *example, int num);
};

static int example_bar(struct example *example, int num)
{
	return example->foo(example, num);
}

DECLARE_STRUCT_CLASS_MOCK_PREREQS(example);

DEFINE_STRUCT_CLASS_MOCK(METHOD(foo), CLASS(example),
			 RETURNS(int),
			 PARAMS(struct example *, int));

static int example_init(struct MOCK(example) *mock_example)
{
	struct example *example = mock_get_trgt(mock_example);

	example->foo = foo;
	return 0;
}

DEFINE_STRUCT_CLASS_MOCK_INIT(example, example_init);

static void example_simple_test(struct test *test)
{
	EXPECT_EQ(test, 1, 1);
}

static void example_mock_test(struct test *test)
{
	struct MOCK(example) *mock_example = test->priv;
	struct example *example = mock_get_trgt(mock_example);
	struct mock_expectation *handle;

	handle = EXPECT_CALL(foo(mock_get_ctrl(mock_example), int_eq(test, 5)));
	handle->action = int_return(test, 2);

	EXPECT_EQ(test, 2, example_bar(example, 5));
}

static int example_test_init(struct test *test)
{
	test_info(test, "initializing");

	test->priv = CONSTRUCT_MOCK(example, test);
	if (!test->priv)
		return -EINVAL;

	return 0;
}

static struct test_case example_test_cases[] = {
	TEST_CASE(example_simple_test),
	TEST_CASE(example_mock_test),
	{},
};

static struct test_module example_test_module = {
	.name = "example",
	.init = example_test_init,
	.test_cases = example_test_cases,
};
module_test(example_test_module);
