// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit mock for struct test.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include "test-mock.h"

DEFINE_STRUCT_CLASS_MOCK_VOID_RETURN(METHOD(fail), CLASS(test),
				     PARAMS(struct test *,
					    struct test_stream *));

DEFINE_STRUCT_CLASS_MOCK_VOID_RETURN(METHOD(mock_vprintk), CLASS(test),
				     PARAMS(const struct test *,
					    const char *,
					    struct va_format *));

static int test_init(struct MOCK(test) *mock_test)
{
	struct test *trgt = mock_get_trgt(mock_test);
	int ret;

	ret = test_init_test(trgt, "MOCK(test)");
	trgt->fail = fail;
	mock_set_default_action(mock_get_ctrl(mock_test),
				"fail",
				fail,
				int_return(mock_get_test(mock_test), 0));
	trgt->vprintk = mock_vprintk;
	mock_set_default_action(mock_get_ctrl(mock_test),
				"mock_vprintk",
				mock_vprintk,
				int_return(mock_get_test(mock_test), 0));
	return ret;
}

DEFINE_STRUCT_CLASS_MOCK_INIT(test, test_init);
