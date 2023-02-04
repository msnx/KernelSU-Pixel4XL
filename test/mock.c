// SPDX-License-Identifier: GPL-2.0
/*
 * Mocking API for KUnit.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/mock.h>

static int mock_void_ptr_init(struct MOCK(void) *mock_void_ptr)
{
	mock_void_ptr->trgt = mock_void_ptr;

	return 0;
}

DEFINE_STRUCT_CLASS_MOCK_INIT(void, mock_void_ptr_init);

static bool mock_match_params(struct mock_matcher *matcher,
		       struct test_stream *stream,
		       const void **params,
		       int len)
{
	struct mock_param_matcher *param_matcher;
	bool ret = true, tmp;
	int i;

	BUG_ON(matcher->num != len);

	for (i = 0; i < matcher->num; i++) {
		param_matcher = matcher->matchers[i];
		stream->add(stream, "\t");
		tmp = param_matcher->match(param_matcher, stream, params[i]);
		ret = ret && tmp;
		stream->add(stream, "\n");
	}

	return ret;
}

static const void *mock_do_expect(struct mock *mock,
				  const char *method_name,
				  const void *method_ptr,
				  const char * const *type_names,
				  const void **params,
				  int len);

void mock_validate_expectations(struct mock *mock)
{
	struct mock_expectation *expectation, *expectation_safe;
	struct mock_method *method;
	struct test_stream *stream;
	int times_called;

	stream = test_new_stream(mock->test);
	list_for_each_entry(method, &mock->methods, node) {
		list_for_each_entry_safe(expectation, expectation_safe,
					 &method->expectations, node) {
			times_called = expectation->times_called;
			if (!(expectation->min_calls_expected <= times_called &&
			      times_called <= expectation->max_calls_expected)
			    ) {
				stream->add(stream,
					    "Expectation was not called the specified number of times:\n\t");
				stream->add(stream,
					    "Function: %s, min calls: %d, max calls: %d, actual calls: %d",
					    method->method_name,
					    expectation->min_calls_expected,
					    expectation->max_calls_expected,
					    times_called);
				mock->test->fail(mock->test, stream);
			}
			list_del(&expectation->node);
		}
	}
}

static void mock_validate_wrapper(struct test_post_condition *condition)
{
	struct mock *mock = container_of(condition, struct mock, parent);

	mock_validate_expectations(mock);
}

void mock_init_ctrl(struct test *test, struct mock *mock)
{
	mock->test = test;
	INIT_LIST_HEAD(&mock->methods);
	mock->do_expect = mock_do_expect;
	mock->type = DEFAULT_MOCK_TYPE;
	mock->parent.validate = mock_validate_wrapper;
	list_add_tail(&mock->parent.node, &test->post_conditions);
}

struct global_mock {
	struct mock ctrl;
	bool is_initialized;
};

static struct global_mock global_mock = {
	.is_initialized = false,
};

static int mock_init_global_mock(struct test_initcall *initcall,
				 struct test *test)
{
	BUG_ON(global_mock.is_initialized);

	mock_init_ctrl(test, &global_mock.ctrl);
	global_mock.is_initialized = true;

	return 0;
}

static void mock_exit_global_mock(struct test_initcall *initcall)
{
	BUG_ON(!global_mock.is_initialized);

	global_mock.ctrl.test = NULL;
	global_mock.is_initialized = false;
}

static struct test_initcall global_mock_initcall = {
	.init = mock_init_global_mock,
	.exit = mock_exit_global_mock,
};
test_register_initcall(global_mock_initcall);

struct mock *mock_get_global_mock(void)
{
	BUG_ON(!global_mock.is_initialized);

	return &global_mock.ctrl;
}

static struct mock_method *mock_lookup_method(struct mock *mock,
					      const void *method_ptr)
{
	struct mock_method *ret;

	list_for_each_entry(ret, &mock->methods, node) {
		if (ret->method_ptr == method_ptr)
			return ret;
	}

	return NULL;
}

static struct mock_method *mock_add_method(struct mock *mock,
					   const char *method_name,
					   const void *method_ptr)
{
	struct mock_method *method;

	method = test_kzalloc(mock->test, sizeof(*method), GFP_KERNEL);
	if (!method)
		return NULL;

	INIT_LIST_HEAD(&method->expectations);
	method->method_name = method_name;
	method->method_ptr = method_ptr;
	list_add_tail(&method->node, &mock->methods);

	return method;
}

static int mock_add_expectation(struct mock *mock,
				const char *method_name,
				const void *method_ptr,
				struct mock_expectation *expectation)
{
	struct mock_method *method;

	method = mock_lookup_method(mock, method_ptr);
	if (!method) {
		method = mock_add_method(mock, method_name, method_ptr);
		if (!method)
			return -ENOMEM;
	}

	list_add_tail(&expectation->node, &method->expectations);

	return 0;
}

struct mock_expectation *mock_add_matcher(struct mock *mock,
					  const char *method_name,
					  const void *method_ptr,
					  struct mock_param_matcher *matchers[],
					  int len)
{
	struct mock_expectation *expectation;
	struct mock_matcher *matcher;
	int ret;

	expectation = test_kzalloc(mock->test,
				   sizeof(*expectation),
				   GFP_KERNEL);
	if (!expectation)
		return NULL;

	matcher = test_kmalloc(mock->test, sizeof(*matcher), GFP_KERNEL);
	if (!matcher)
		return NULL;

	memcpy(&matcher->matchers, matchers, sizeof(*matchers) * len);
	matcher->num = len;

	expectation->matcher = matcher;
	expectation->max_calls_expected = 1;
	expectation->min_calls_expected = 1;

	ret = mock_add_expectation(mock, method_name, method_ptr, expectation);
	if (ret < 0)
		return NULL;

	return expectation;
}

int mock_set_default_action(struct mock *mock,
			    const char *method_name,
			    const void *method_ptr,
			    struct mock_action *action)
{
	struct mock_method *method;

	method = mock_lookup_method(mock, method_ptr);
	if (!method) {
		method = mock_add_method(mock, method_name, method_ptr);
		if (!method)
			return -ENOMEM;
	}

	method->default_action = action;

	return 0;
}

struct mock_param_formatter_repo {
	struct list_head formatters;
};

static struct mock_param_formatter_repo mock_param_formatter_repo = {
	.formatters = LIST_HEAD_INIT(mock_param_formatter_repo.formatters),
};

void mock_register_formatter(struct mock_param_formatter *formatter)
{
	list_add_tail(&formatter->node, &mock_param_formatter_repo.formatters);
}

void mock_unregister_formatter(struct mock_param_formatter *formatter)
{
	list_del(&formatter->node);
}

struct mock_param_formatter *mock_find_formatter(const char *type_name)
{
	struct mock_param_formatter *formatter;

	list_for_each_entry(formatter,
			    &mock_param_formatter_repo.formatters,
			    node) {
		if (!strcmp(type_name, formatter->type_name))
			return formatter;
	}

	return NULL;
}

static void mock_format_param(struct test_stream *stream,
			      const char *type_name,
			      const void *param)
{
	struct mock_param_formatter *formatter;

	formatter = mock_find_formatter(type_name);
	if (formatter)
		formatter->format(formatter, stream, param);
	else
		/*
		 * Cannot find formatter, so just print the pointer of the
		 * symbol.
		 */
		stream->add(stream, "<%pS>", param);
}

static void mock_add_method_declaration_to_stream(
		struct test_stream *stream,
		const char *function_name,
		const char * const *type_names,
		const void **params,
		int len)
{
	int i;

	stream->add(stream, "%s(", function_name);
	for (i = 0; i < len; i++) {
		mock_format_param(stream, type_names[i], params[i]);
		if (i < len - 1)
			stream->add(stream, ", ");
	}
	stream->add(stream, ")\n");
}

static struct test_stream *mock_initialize_failure_message(
		struct test *test,
		const char *function_name,
		const char * const *type_names,
		const void **params,
		int len)
{
	struct test_stream *stream;

	stream = test_new_stream(test);
	if (!stream)
		return NULL;

	stream->add(stream, "EXPECTATION FAILED: no expectation for call: ");
	mock_add_method_declaration_to_stream(stream,
					      function_name,
					      type_names,
					      params,
					      len);
	return stream;
}

static bool mock_is_expectation_retired(struct mock_expectation *expectation)
{
	return expectation->retire_on_saturation &&
			expectation->times_called ==
			expectation->max_calls_expected;
}

static void mock_add_method_expectation_error(struct test *test,
					      struct test_stream *stream,
					      char *message,
					      struct mock *mock,
					      struct mock_method *method,
					      const char * const *type_names,
					      const void **params,
					      int len)
{
	stream->clear(stream);
	stream->set_level(stream, KERN_WARNING);
	stream->add(stream, message);
	mock_add_method_declaration_to_stream(stream,
		method->method_name, type_names, params, len);
}

static struct mock_expectation *mock_apply_expectations(
		struct mock *mock,
		struct mock_method *method,
		const char * const *type_names,
		const void **params,
		int len)
{
	struct test *test = mock->test;
	struct mock_expectation *ret;
	struct test_stream *attempted_matching_stream;
	bool expectations_all_saturated = true;

	struct test_stream *stream = test_new_stream(test);

	if (list_empty(&method->expectations)) {
		mock_add_method_expectation_error(test, stream,
			"Method was called with no expectations declared: ",
			mock, method, type_names, params, len);
		if (is_strict_mock(mock))
			test->fail(test, stream);
		else if (is_naggy_mock(mock))
			stream->commit(stream);
		else
			stream->clear(stream);
		return NULL;
	}

	attempted_matching_stream = mock_initialize_failure_message(
			test,
			method->method_name,
			type_names,
			params,
			len);

	list_for_each_entry(ret, &method->expectations, node) {
		if (mock_is_expectation_retired(ret))
			continue;
		expectations_all_saturated = false;

		attempted_matching_stream->add(attempted_matching_stream,
			"Tried expectation: %s, but\n", ret->expectation_name);
		if (mock_match_params(ret->matcher,
			attempted_matching_stream, params, len)) {
			/*
			 * Matcher was found; we won't print, so clean up the
			 * log.
			 */
			attempted_matching_stream->clear(
					attempted_matching_stream);
			return ret;
		}
	}

	if (expectations_all_saturated && !is_nice_mock(mock)) {
		mock_add_method_expectation_error(test, stream,
			"Method was called with fully saturated expectations: ",
			mock, method, type_names, params, len);
	} else {
		mock_add_method_expectation_error(test, stream,
			"Method called that did not match any expectations: ",
			mock, method, type_names, params, len);
		stream->append(stream, attempted_matching_stream);
	}
	test->fail(test, stream);
	attempted_matching_stream->clear(attempted_matching_stream);
	return NULL;
}

static const void *mock_do_expect(struct mock *mock,
				  const char *method_name,
				  const void *method_ptr,
				  const char * const *param_types,
				  const void **params,
				  int len)
{
	struct mock_expectation *expectation;
	struct mock_method *method;
	struct mock_action *action;

	method = mock_lookup_method(mock, method_ptr);
	if (!method)
		return NULL;

	expectation = mock_apply_expectations(mock,
					      method,
					      param_types,
					      params,
					      len);
	if (!expectation) {
		action = method->default_action;
	} else {
		expectation->times_called++;
		if (expectation->action)
			action = expectation->action;
		else
			action = method->default_action;
	}
	if (!action)
		return NULL;

	return action->do_action(action, params, len);
}
