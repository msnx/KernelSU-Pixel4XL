// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for core test infrastructure.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */
#include <test/test.h>

static void test_test_catches_segfault(struct test *test)
{
	void (*invalid_func)(void) = (void (*)(void)) SIZE_MAX;

	ASSERT_SIGSEGV(test, invalid_func());
}

static int test_test_init(struct test *test)
{
	return 0;
}

static void test_test_exit(struct test *test)
{
}

static struct test_case test_test_cases[] = {
	TEST_CASE(test_test_catches_segfault),
	{},
};

static struct test_module test_test_module = {
	.name = "test-test",
	.init = test_test_init,
	.exit = test_test_exit,
	.test_cases = test_test_cases,
};
module_test(test_test_module);
