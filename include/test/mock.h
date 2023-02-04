/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mocking API for KUnit.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#ifndef _TEST_MOCK_H
#define _TEST_MOCK_H

#include <linux/types.h>
#include <linux/tracepoint.h> /* For PARAMS(...) */
#include <test/test.h>
#include <test/test-stream.h>
#include <test/params.h>

/**
 * struct mock_param_matcher - represents a matcher used in a *call expectation*
 * @match: the function that performs the matching
 *
 * The matching function takes a couple of parameters:
 *
 * - ``this``: refers to the parent struct
 * - ``stream``: a &test_stream to which a detailed message should be added as
 *   to why the parameter matches or not
 * - ``param``: a pointer to the parameter to check for a match
 *
 * The matching function should return whether or not the passed parameter
 * matches.
 */
struct mock_param_matcher {
	bool (*match)(struct mock_param_matcher *this,
		      struct test_stream *stream,
		      const void *param);
};

#define MOCK_MAX_PARAMS 255

struct mock_matcher {
	struct mock_param_matcher *matchers[MOCK_MAX_PARAMS];
	int num;
};

/**
 * struct mock_action - Represents an action that a mock performs when
 *                      expectation is matched
 * @do_action: the action to perform
 *
 * The action function is given some parameters:
 *
 * - ``this``: refers to the parent struct
 * - ``params``: an array of pointers to the params passed into the mocked
 *   method or function. **The class argument is excluded for a mocked class
 *   method.**
 * - ``len``: size of ``params``
 *
 * The action function returns a pointer to the value that the mocked method
 * or function should be returning.
 */
struct mock_action {
	void *(*do_action)(struct mock_action *this,
			   const void **params,
			   int len);
};

/**
 * struct mock_expectation - represents a *call expectation* on a function.
 * @action: A &struct mock_action to perform when the function is called.
 * @max_calls_expected: maximum number of times an expectation may be called.
 * @min_calls_expected: minimum number of times an expectation may be called.
 * @retire_on_saturation: no longer match once ``max_calls_expected`` is
 *			  reached.
 *
 * Represents a *call expectation* on a function created with EXPECT_CALL().
 */
struct mock_expectation {
	struct mock_action *action;
	int max_calls_expected;
	int min_calls_expected;
	bool retire_on_saturation;
	/* private: internal use only. */
	const char *expectation_name;
	struct list_head node;
	struct mock_matcher *matcher;
	int times_called;
};

struct mock_method {
	struct list_head node;
	const char *method_name;
	const void *method_ptr;
	struct mock_action *default_action;
	struct list_head expectations;
};

enum mock_type {
	MOCK_TYPE_NICE,
	MOCK_TYPE_NAGGY,
	MOCK_TYPE_STRICT
};

struct mock {
	struct test_post_condition parent;
	struct test *test;
	struct list_head methods;
	enum mock_type type;
	const void *(*do_expect)(struct mock *mock,
				 const char *method_name,
				 const void *method_ptr,
				 const char * const *param_types,
				 const void **params,
				 int len);
};

#define DEFAULT_MOCK_TYPE MOCK_TYPE_NAGGY

void mock_init_ctrl(struct test *test, struct mock *mock);

void mock_validate_expectations(struct mock *mock);

int mock_set_default_action(struct mock *mock,
			    const char *method_name,
			    const void *method_ptr,
			    struct mock_action *action);

struct mock_expectation *mock_add_matcher(struct mock *mock,
					  const char *method_name,
					  const void *method_ptr,
					  struct mock_param_matcher *matchers[],
					  int len);

struct mock_param_formatter {
	struct list_head node;
	const char *type_name;
	void (*format)(struct mock_param_formatter *formatter,
		       struct test_stream *stream,
		       const void *param);
};

void mock_register_formatter(struct mock_param_formatter *formatter);

void mock_unregister_formatter(struct mock_param_formatter *formatter);

struct mock *mock_get_global_mock(void);

#define MOCK(name) name##_mock

/**
 * STRICT_MOCK() - sets the mock to be strict and returns the mock
 * @mock: the mock
 *
 * For an example, see ``The Nice, the Strict, and the Naggy`` under
 * ``Using KUnit``.
 */
#define STRICT_MOCK(mock) \
({ \
	mock_get_ctrl(mock)->type = MOCK_TYPE_STRICT; \
	mock; \
})

static inline bool is_strict_mock(struct mock *mock)
{
	return mock->type == MOCK_TYPE_STRICT;
}

/**
 * NICE_MOCK() - sets the mock to be nice and returns the mock
 * @mock: the mock
 *
 * For an example, see ``The Nice, the Strict, and the Naggy`` under
 * ``Using KUnit``.
 */
#define NICE_MOCK(mock) \
({ \
	mock_get_ctrl(mock)->type = MOCK_TYPE_NICE; \
	mock; \
})

static inline bool is_nice_mock(struct mock *mock)
{
	return mock->type == MOCK_TYPE_NICE;
}

/**
 * NAGGY_MOCK() - sets the mock to be naggy and returns the mock
 * @mock: the mock
 *
 * For an example, see ``The Nice, the Strict, and the Naggy`` under
 * ``Using KUnit``.
 */
#define NAGGY_MOCK(mock) \
({ \
	mock_get_ctrl(mock)->type = MOCK_TYPE_NAGGY; \
	mock; \
})

static inline bool is_naggy_mock(struct mock *mock)
{
	return mock->type == MOCK_TYPE_NAGGY;
}

/**
 * EXPECT_CALL() - Declares a *call expectation* on a mock method or function.
 * @expectation_call: a mocked method or function with parameters replaced with
 *                    matchers.
 *
 * Example:
 *
 * .. code-block:: c
 *
 *	// Class to mock.
 *	struct example {
 *		int (*foo)(struct example *, int);
 *	};
 *
 *	// Define the mock.
 *	DECLARE_STRUCT_CLASS_MOCK_PREREQS(example);
 *
 *	DEFINE_STRUCT_CLASS_MOCK(METHOD(foo), CLASS(example),
 *				 RETURNS(int),
 *				 PARAMS(struct example *, int));
 *
 *	static int example_init(struct MOCK(example) *mock_example)
 *	{
 *		struct example *example = mock_get_trgt(mock_example);
 *
 *		example->foo = foo;
 *		return 0;
 *	}
 *
 *	DEFINE_STRUCT_CLASS_MOCK_INIT(example, example_init);
 *
 *	static void foo_example_test_success(struct test *test)
 *	{
 *		struct MOCK(example) *mock_example;
 *		struct example *example = mock_get_trgt(mock_example);
 *		struct mock_expectation *handle;
 *
 *		mock_example = CONSTRUCT_MOCK(example, test);
 *
 *		handle = EXPECT_CALL(foo(mock_get_ctrl(mock_example),
 *				     int_eq(test, 5)));
 *		handle->action = int_return(test, 2);
 *
 *		EXPECT_EQ(test, 2, example_bar(example, 5));
 *	}
 *
 * Return:
 * A &struct mock_expectation representing the call expectation.
 * allowing additional conditions and actions to be specified.
 */
#define EXPECT_CALL(expectation_call) mock_master_##expectation_call;

#define mock_get_ctrl_internal(mock_object) (&(mock_object)->ctrl)
#define mock_get_ctrl(mock_object) mock_get_ctrl_internal(mock_object)

#define mock_get_trgt_internal(mock_object) (&(mock_object)->trgt)
#define mock_get_trgt(mock_object) mock_get_trgt_internal(mock_object)

#define mock_get_test(mock_object) (mock_get_ctrl(mock_object)->test)

#define CLASS(struct_name) struct_name
#define HANDLE_INDEX(index) index
#define METHOD(method_name) method_name
#define RETURNS(return_type) return_type
/* #define PARAMS(...) __VA_ARGS__ included by linux/tracepoint.h */

#define MOCK_INIT_ID(struct_name) struct_name##mock_init
#define REAL_ID(func_name) __real__##func_name
#define INVOKE_ID(func_name) __invoke__##func_name

#define DECLARE_MOCK_CLIENT(name, return_type, param_types...) \
		return_type name(PARAM_LIST_FROM_TYPES(param_types))

#define DECLARE_MOCK_MASTER(name, ctrl_index, param_types...)		       \
		struct mock_expectation *mock_master_##name(		       \
				MATCHER_PARAM_LIST_FROM_TYPES(ctrl_index,      \
							      param_types));

#define DECLARE_MOCK_COMMON(name, handle_index, return_type, param_types...)   \
		DECLARE_MOCK_CLIENT(name, return_type, param_types);	       \
		DECLARE_MOCK_MASTER(name, handle_index, param_types)

#define DECLARE_REDIRECT_MOCKABLE(name, return_type, param_types...)	       \
		return_type REAL_ID(name)(param_types);			       \
		return_type name(param_types);				       \
		void *INVOKE_ID(name)(struct test *test,		       \
				      const void *params[],		       \
				      int len)

#define DECLARE_MOCK_FUNC_CLIENT(name, return_type, param_types...) \
		DECLARE_MOCK_CLIENT(name, return_type, param_types)

#define DECLARE_MOCK_FUNC_MASTER(name, param_types...) \
		DECLARE_MOCK_MASTER(name, MOCK_MAX_PARAMS, param_types)

#define DECLARE_STRUCT_CLASS_MOCK_STRUCT(struct_name)			       \
		struct MOCK(struct_name) {				       \
			struct mock		ctrl;			       \
			struct struct_name	trgt;			       \
		}

#define DECLARE_STRUCT_CLASS_MOCK_CONVERTER(struct_name)		       \
		static inline struct mock *from_##struct_name##_to_mock(       \
				const struct struct_name *trgt)		       \
		{							       \
			return mock_get_ctrl(				       \
					container_of(trgt,		       \
						     struct MOCK(struct_name), \
						     trgt));		       \
		}

/**
 * DECLARE_STRUCT_CLASS_MOCK_PREREQS() - Create a mock child class
 * @struct_name: name of the class/struct to be mocked
 *
 * Creates a mock child class of ``struct_name`` named
 * ``struct MOCK(struct_name)`` along with supporting internally used methods.
 *
 * See EXPECT_CALL() for example usages.
 */
#define DECLARE_STRUCT_CLASS_MOCK_PREREQS(struct_name)			       \
		DECLARE_STRUCT_CLASS_MOCK_STRUCT(struct_name);		       \
		DECLARE_STRUCT_CLASS_MOCK_CONVERTER(struct_name)

#define DECLARE_STRUCT_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,		       \
							struct_name,	       \
							handle_index,	       \
							return_type,	       \
							param_types...)	       \
		DECLARE_MOCK_COMMON(name,				       \
				    handle_index,			       \
				    return_type,			       \
				    param_types)

#define DECLARE_STRUCT_CLASS_MOCK_HANDLE_INDEX(name,			       \
					       struct_name,		       \
					       handle_index,		       \
					       return_type,		       \
					       param_types...)		       \
		DECLARE_STRUCT_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,	       \
								struct_name,   \
								handle_index,  \
								return_type,   \
								param_types)

/**
 * DECLARE_STRUCT_CLASS_MOCK()
 * @name: method name
 * @struct_name: name of the class/struct
 * @return_type: return type of the method
 * @param_types: parameters of the method
 *
 * Same as DEFINE_STRUCT_CLASS_MOCK(), but only makes header compatible
 * declarations.
 */
#define DECLARE_STRUCT_CLASS_MOCK(name,					       \
				  struct_name,				       \
				  return_type,				       \
				  param_types...)			       \
		DECLARE_STRUCT_CLASS_MOCK_HANDLE_INDEX(name,		       \
						       struct_name,	       \
						       0,		       \
						       return_type,	       \
						       param_types)

/**
 * DECLARE_STRUCT_CLASS_MOCK_VOID_RETURN()
 * @name: method name
 * @struct_name: name of the class/struct
 * @param_types: parameters of the method
 *
 * Same as DEFINE_STRUCT_CLASS_MOCK_VOID_RETURN(), but only makes header
 * compatible declarations.
 */
#define DECLARE_STRUCT_CLASS_MOCK_VOID_RETURN(name,			       \
					      struct_name,		       \
					      param_types...)		       \
		DECLARE_STRUCT_CLASS_MOCK_HANDLE_INDEX(name,		       \
						       struct_name,	       \
						       0,		       \
						       void,		       \
						       param_types)

/**
 * DECLARE_STRUCT_CLASS_MOCK_INIT()
 * @struct_name: name of the class/struct
 *
 * Same as DEFINE_STRUCT_CLASS_MOCK_INIT(), but only makes header compatible
 * declarations.
 */
#define DECLARE_STRUCT_CLASS_MOCK_INIT(struct_name)			       \
		struct MOCK(struct_name) *MOCK_INIT_ID(struct_name)(	       \
				struct test *test)

#define DECLARE_VOID_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,		       \
						      handle_index,	       \
						      return_type,	       \
						      param_types...)	       \
		DECLARE_MOCK_COMMON(name,				       \
				    handle_index,			       \
				    return_type,			       \
				    param_types)

#define DECLARE_VOID_CLASS_MOCK_HANDLE_INDEX(name,			       \
					     handle_index,		       \
					     return_type,		       \
					     param_types...)		       \
		DECLARE_VOID_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,	       \
							      handle_index,    \
							      return_type,     \
							      param_types)

/**
 * CONSTRUCT_MOCK()
 * @struct_name: name of the class
 * @test: associated test
 *
 * Constructs and allocates a test managed ``struct MOCK(struct_name)`` given
 * the name of the class for which the mock is defined and a test object.
 *
 * See EXPECT_CALL() for example usage.
 */
#define CONSTRUCT_MOCK(struct_name, test) MOCK_INIT_ID(struct_name)(test)

#define DECLARE_FUNCTION_MOCK_INTERNAL(name, return_type, param_types...)      \
		DECLARE_MOCK_FUNC_CLIENT(name, return_type, param_types);      \
		DECLARE_MOCK_FUNC_MASTER(name, param_types);

#define DECLARE_FUNCTION_MOCK(name, return_type, param_types...) \
		DECLARE_FUNCTION_MOCK_INTERNAL(name, return_type, param_types)

#define DECLARE_FUNCTION_MOCK_VOID_RETURN(name, param_types...) \
		DECLARE_FUNCTION_MOCK(name, void, param_types)

#define DEFINE_MOCK_CLIENT_COMMON(name,					       \
				  handle_index,				       \
				  MOCK_SOURCE,				       \
				  mock_source_ctx,			       \
				  return_type,				       \
				  RETURN,				       \
				  param_types...)			       \
		return_type name(PARAM_LIST_FROM_TYPES(param_types))	       \
		{							       \
			struct mock *mock = MOCK_SOURCE(mock_source_ctx,       \
							handle_index);	       \
			static const char * const param_type_names[] = {       \
				TYPE_NAMES_FROM_TYPES(handle_index,	       \
						      param_types)	       \
			};						       \
			const void *params[] = {			       \
				PTR_TO_ARG_FROM_TYPES(handle_index,	       \
						      param_types)	       \
			};						       \
			const void *retval;				       \
									       \
			retval = mock->do_expect(mock,			       \
						 #name,			       \
						 name,			       \
						 param_type_names,	       \
						 params,		       \
						 ARRAY_SIZE(params));	       \
			ASSERT_NOT_ERR_OR_NULL(mock->test, retval);	       \
			if (!retval) {					       \
				test_info(mock->test,			       \
					  "no action installed for "#name);    \
				BUG();					       \
			}						       \
			RETURN(return_type, retval);			       \
		}

#if IS_ENABLED(CONFIG_TEST)
#define DEFINE_INVOKABLE(name, return_type, RETURN_ASSIGN, param_types...)     \
		void *INVOKE_ID(name)(struct test *test,		       \
				      const void *params[],		       \
				      int len) {			       \
			return_type *retval;				       \
									       \
			ASSERT_EQ(test, NUM_VA_ARGS(param_types), len);	       \
			retval = test_kzalloc(test,			       \
					      sizeof(*retval),		       \
					      GFP_KERNEL);		       \
			ASSERT_NOT_ERR_OR_NULL(test, retval);		       \
			RETURN_ASSIGN() REAL_ID(name)(			       \
					ARRAY_ACCESSORS_FROM_TYPES(	       \
							param_types));	       \
			return retval;					       \
		}
#else
#define DEFINE_INVOKABLE(name, return_type, RETURN_ASSIGN, param_types...)
#endif

#define DEFINE_REDIRECT_MOCKABLE_COMMON(name,				       \
					return_type,			       \
					RETURN_ASSIGN,			       \
					param_types...)			       \
		return_type REAL_ID(name)(param_types);			       \
		return_type name(param_types) __mockable_alias(REAL_ID(name)); \
		DEFINE_INVOKABLE(name, return_type, RETURN_ASSIGN, param_types);

#define ASSIGN() *retval =

/**
 * DEFINE_REDIRECT_MOCKABLE()
 * @name: name of the function
 * @return_type: return type of the function
 * @param_types: parameter types of the function
 *
 * Used to define a function which is *redirect-mockable*, which allows the
 * function to be mocked and refer to the original definition via
 * INVOKE_REAL().
 *
 * Example:
 *
 * .. code-block:: c
 *
 *	DEFINE_REDIRECT_MOCKABLE(i2c_add_adapter,
 *				 RETURNS(int), PARAMS(struct i2c_adapter *));
 *	int REAL_ID(i2c_add_adapter)(struct i2c_adapter *adapter)
 *	{
 *		...
 *	}
 *
 *	static int aspeed_i2c_test_init(struct test *test)
 *	{
 *		struct mock_param_capturer *adap_capturer;
 *		struct mock_expectation *handle;
 *		struct aspeed_i2c_test *ctx;
 *		int ret;
 *
 *		ctx = test_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
 *		if (!ctx)
 *			return -ENOMEM;
 *		test->priv = ctx;
 *
 *		handle = EXPECT_CALL(
 *				i2c_add_adapter(capturer_to_matcher(
 *						adap_capturer)));
 *		handle->action = INVOKE_REAL(test, i2c_add_adapter);
 *		ret = of_fake_probe_platform_by_name(test,
 *						     "aspeed-i2c-bus",
 *						     "test-i2c-bus");
 *		if (ret < 0)
 *			return ret;
 *
 *		ASSERT_PARAM_CAPTURED(test, adap_capturer);
 *		ctx->adap = mock_capturer_get(adap_capturer,
 *					      struct i2c_adapter *);
 *
 *		return 0;
 *	}
 */
#define DEFINE_REDIRECT_MOCKABLE(name, return_type, param_types...)	       \
		DEFINE_REDIRECT_MOCKABLE_COMMON(name,			       \
						return_type,		       \
						ASSIGN,			       \
						param_types)

#define NO_ASSIGN()
#define DEFINE_REDIRECT_MOCKABLE_VOID_RETURN(name, param_types)		       \
		DEFINE_REDIRECT_MOCKABLE_COMMON(name,			       \
						void,			       \
						NO_ASSIGN,		       \
						param_types)

#define CLASS_MOCK_CLIENT_SOURCE(ctx, handle_index) ctx(arg##handle_index)
#define DEFINE_MOCK_METHOD_CLIENT_COMMON(name,				       \
					 handle_index,			       \
					 mock_converter,		       \
					 return_type,			       \
					 RETURN,			       \
					 param_types...)		       \
		DEFINE_MOCK_CLIENT_COMMON(name,				       \
					  handle_index,			       \
					  CLASS_MOCK_CLIENT_SOURCE,	       \
					  mock_converter,		       \
					  return_type,			       \
					  RETURN,			       \
					  param_types)

#define CAST_AND_RETURN(return_type, retval) return *((return_type *) retval)
#define NO_RETURN(return_type, retval)

#define DEFINE_MOCK_METHOD_CLIENT(name,					       \
				  handle_index,				       \
				  mock_converter,			       \
				  return_type,				       \
				  param_types...)			       \
		DEFINE_MOCK_METHOD_CLIENT_COMMON(name,			       \
						 handle_index,		       \
						 mock_converter,	       \
						 return_type,		       \
						 CAST_AND_RETURN,	       \
						 param_types)

#define DEFINE_MOCK_METHOD_CLIENT_VOID_RETURN(name,			       \
					      handle_index,		       \
					      mock_converter,		       \
					      param_types...)		       \
		DEFINE_MOCK_METHOD_CLIENT_COMMON(name,			       \
						 handle_index,		       \
						 mock_converter,	       \
						 void,			       \
						 NO_RETURN,		       \
						 param_types)

#define FUNC_MOCK_SOURCE(ctx, handle_index) mock_get_global_mock()
#define DEFINE_MOCK_FUNC_CLIENT_COMMON(name,				       \
				       return_type,			       \
				       RETURN,				       \
				       param_types...)			       \
		DEFINE_MOCK_CLIENT_COMMON(name,				       \
					  MOCK_MAX_PARAMS,		       \
					  FUNC_MOCK_SOURCE,		       \
					  name,				       \
					  return_type,			       \
					  RETURN,			       \
					  param_types)

#define DEFINE_MOCK_FUNC_CLIENT(name, return_type, param_types...)	       \
		DEFINE_MOCK_FUNC_CLIENT_COMMON(name,			       \
					       return_type,		       \
					       CAST_AND_RETURN,		       \
					       param_types)

#define DEFINE_MOCK_FUNC_CLIENT_VOID_RETURN(name, param_types...)	       \
		DEFINE_MOCK_FUNC_CLIENT_COMMON(name,			       \
					       void,			       \
					       NO_RETURN,		       \
					       param_types)

#define DEFINE_MOCK_MASTER_COMMON_INTERNAL(name,			       \
					   ctrl_index,			       \
					   MOCK_SOURCE,			       \
					   param_types...)		       \
		struct mock_expectation *mock_master_##name(		       \
				MATCHER_PARAM_LIST_FROM_TYPES(ctrl_index,      \
							      param_types))    \
		{ \
			struct mock_param_matcher *matchers[] = {	       \
				ARG_NAMES_FROM_TYPES(ctrl_index, param_types)  \
			};						       \
									       \
			return mock_add_matcher(MOCK_SOURCE(ctrl_index),       \
						#name,			       \
						(const void *) name,	       \
						matchers,		       \
						ARRAY_SIZE(matchers));	       \
		}
#define DEFINE_MOCK_MASTER_COMMON(name,					       \
				  ctrl_index,				       \
				  MOCK_SOURCE,				       \
				  param_types...)			       \
		DEFINE_MOCK_MASTER_COMMON_INTERNAL(name,		       \
						   ctrl_index,		       \
						   MOCK_SOURCE,		       \
						   param_types)

#define CLASS_MOCK_MASTER_SOURCE(ctrl_index) arg##ctrl_index
#define DEFINE_MOCK_METHOD_MASTER(name, ctrl_index, param_types...)	       \
		DEFINE_MOCK_MASTER_COMMON(name,				       \
					  ctrl_index,			       \
					  CLASS_MOCK_MASTER_SOURCE,	       \
					  param_types)

#define FUNC_MOCK_CLIENT_SOURCE(ctrl_index) mock_get_global_mock()
#define DEFINE_MOCK_FUNC_MASTER(name, param_types...)			       \
		DEFINE_MOCK_MASTER_COMMON(name,				       \
					  MOCK_MAX_PARAMS,		       \
					  FUNC_MOCK_CLIENT_SOURCE,	       \
					  param_types)

#define DEFINE_MOCK_COMMON(name,					       \
			   handle_index,				       \
			   mock_converter,				       \
			   return_type,					       \
			   param_types...)				       \
		DEFINE_MOCK_METHOD_CLIENT(name,				       \
					  handle_index,			       \
					  mock_converter,		       \
					  return_type,			       \
					  param_types);			       \
		DEFINE_MOCK_METHOD_MASTER(name, handle_index, param_types)

#define DEFINE_MOCK_COMMON_VOID_RETURN(name,				       \
				       handle_index,			       \
				       mock_converter,			       \
				       param_types...)			       \
		DEFINE_MOCK_METHOD_CLIENT_VOID_RETURN(name,		       \
						      handle_index,	       \
						      mock_converter,	       \
						      param_types);	       \
		DEFINE_MOCK_METHOD_MASTER(name, handle_index, param_types)

#define DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,		       \
						       struct_name,	       \
						       handle_index,	       \
						       return_type,	       \
						       param_types...)	       \
		DEFINE_MOCK_COMMON(name,				       \
				   handle_index,			       \
				   from_##struct_name##_to_mock,	       \
				   return_type, param_types)
#define DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX(name,			       \
					      struct_name,		       \
					      handle_index,		       \
					      return_type,		       \
					      param_types...)		       \
		DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,	       \
							       struct_name,    \
							       handle_index,   \
							       return_type,    \
							       param_types)

#define DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_VOID_RETURN_INTERNAL(	       \
		name,							       \
		struct_name,						       \
		handle_index,						       \
		param_types...)						       \
		DEFINE_MOCK_COMMON_VOID_RETURN(name,			       \
					       handle_index,		       \
					       from_##struct_name##_to_mock,   \
					       param_types)
#define DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_VOID_RETURN(name,		       \
							  struct_name,	       \
							  handle_index,	       \
							  param_types...)      \
		DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_VOID_RETURN_INTERNAL(    \
				name,					       \
				struct_name,				       \
				handle_index,				       \
				param_types)

/**
 * DEFINE_STRUCT_CLASS_MOCK()
 * @name: name of the method
 * @struct_name: name of the class of which the method belongs
 * @return_type: return type of the method to be created. **Must not be void.**
 * @param_types: parameters to method to be created.
 *
 * See EXPECT_CALL() for example usage.
 */
#define DEFINE_STRUCT_CLASS_MOCK(name,					       \
				 struct_name,				       \
				 return_type,				       \
				 param_types...)			       \
		DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX(name,		       \
						      struct_name,	       \
						      0,		       \
						      return_type,	       \
						      param_types)

/**
 * DEFINE_STRUCT_CLASS_MOCK_VOID_RETURN()
 * @name: name of the method
 * @struct_name: name of the class of which the method belongs
 * @param_types: parameters to method to be created.
 *
 * Same as DEFINE_STRUCT_CLASS_MOCK() except the method has a ``void`` return
 * type.
 */
#define DEFINE_STRUCT_CLASS_MOCK_VOID_RETURN(name, struct_name, param_types...)\
		DEFINE_STRUCT_CLASS_MOCK_HANDLE_INDEX_VOID_RETURN(name,	       \
								  struct_name, \
								  0,	       \
								  param_types)

/**
 * DEFINE_STRUCT_CLASS_MOCK_INIT()
 * @struct_name: name of the class
 * @init_func: a function of type ``int (*)(struct MOCK(struct_name) *)``. This
 *             function is passed a pointer to an allocated, *but not
 *             initialized*, ``struct MOCK(struct_name)``. The job of this user
 *             provided function is to perform remaining initialization. Usually
 *             this entails assigning mock methods to the function pointers in
 *             the parent struct.
 *
 * See EXPECT_CALL() for example usage.
 */
#define DEFINE_STRUCT_CLASS_MOCK_INIT(struct_name, init_func)		       \
		struct MOCK(struct_name) *MOCK_INIT_ID(struct_name)(	       \
				struct test *test)			       \
		{							       \
			struct MOCK(struct_name) *mock_obj;		       \
									       \
			mock_obj = test_kzalloc(test,			       \
						sizeof(*mock_obj),	       \
						GFP_KERNEL);		       \
			if (!mock_obj)					       \
				return NULL;				       \
									       \
			mock_init_ctrl(test, mock_get_ctrl(mock_obj));	       \
									       \
			if (init_func(mock_obj))			       \
				return NULL;				       \
									       \
			return mock_obj;				       \
		}

struct MOCK(void) {
	struct mock	ctrl;
	void		*trgt;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
static inline struct mock *from_void_ptr_to_mock(const void *ptr)
{
	struct MOCK(void) *mock_void_ptr = ptr;

	return mock_get_ctrl(mock_void_ptr);
}
#pragma GCC diagnostic pop

#define DEFINE_VOID_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,		       \
						     handle_index,	       \
						     return_type,	       \
						     param_types...)	       \
		DEFINE_MOCK_COMMON(name,				       \
				   handle_index,			       \
				   from_void_ptr_to_mock,		       \
				   return_type,				       \
				   param_types)
#define DEFINE_VOID_CLASS_MOCK_HANDLE_INDEX(name,			       \
					    handle_index,		       \
					    return_type,		       \
					    param_types...)		       \
		DEFINE_VOID_CLASS_MOCK_HANDLE_INDEX_INTERNAL(name,	       \
							     handle_index,     \
							     return_type,      \
							     param_types)

DECLARE_STRUCT_CLASS_MOCK_INIT(void);

#define DEFINE_FUNCTION_MOCK_INTERNAL(name, return_type, param_types...)       \
		DEFINE_MOCK_FUNC_CLIENT(name, return_type, param_types);       \
		DEFINE_MOCK_FUNC_MASTER(name, param_types)

/**
 * DEFINE_FUNCTION_MOCK()
 * @name: name of the function
 * @return_type: return type of the function
 * @...: parameter types of the function
 *
 * Same as DEFINE_STRUCT_CLASS_MOCK() except can be used to mock any function
 * declared %__mockable or DEFINE_REDIRECT_MOCKABLE()
 */
#define DEFINE_FUNCTION_MOCK(name, return_type, param_types...) \
		DEFINE_FUNCTION_MOCK_INTERNAL(name, return_type, param_types)

#define DEFINE_FUNCTION_MOCK_VOID_RETURN_INTERNAL(name, param_types...)	       \
		DEFINE_MOCK_FUNC_CLIENT_VOID_RETURN(name, param_types);	       \
		DEFINE_MOCK_FUNC_MASTER(name, param_types)

/**
 * DEFINE_FUNCTION_MOCK_VOID_RETURN()
 * @name: name of the function
 * @...: parameter types of the function
 *
 * Same as DEFINE_FUNCTION_MOCK() except the method has a ``void`` return
 * type.
 */
#define DEFINE_FUNCTION_MOCK_VOID_RETURN(name, param_types...) \
		DEFINE_FUNCTION_MOCK_VOID_RETURN_INTERNAL(name, param_types)

#if IS_ENABLED(CONFIG_TEST)

/**
 * __mockable - A function decorator that allows the function to be mocked.
 *
 * Example:
 *
 * .. code-block:: c
 *
 *	int __mockable example(int arg) { ... }
 */
#define __mockable __weak
#define __mockable_alias(id) __weak __alias(id)

/**
 * __visible_for_testing - Makes a static function visible when testing.
 *
 * A macro that replaces the `static` specifier on functions and global
 * variables that is static when compiled normally and visible when compiled for
 * tests.
 */
#define __visible_for_testing
#else
#define __mockable
#define __mockable_alias(id) __alias(id)
#define __visible_for_testing static
#endif

#define CONVERT_TO_ACTUAL_TYPE(type, ptr) (*((type *) ptr))

/**
 * DOC: Built In Matchers
 *
 * These are the matchers that can be used when matching arguments in
 * :c:func:`EXPECT_CALL` (more can be defined manually).
 *
 * For example, there's a matcher that matches any arguments:
 *
 * .. code-block:: c
 *
 *    struct mock_param_matcher *any(struct test *test);
 *
 * There are matchers for integers based on the binary condition:
 *
 * * eq: equals to
 * * ne: not equal to
 * * lt: less than
 * * le: less than or equal to
 * * gt: greater than
 * * ge: greater than or equal to
 *
 * .. code-block:: c
 *
 *    struct mock_param_matcher *int_eq(struct test *test, int expected);
 *    struct mock_param_matcher *int_ne(struct test *test, int expected);
 *    struct mock_param_matcher *int_lt(struct test *test, int expected);
 *    struct mock_param_matcher *int_le(struct test *test, int expected);
 *    struct mock_param_matcher *int_gt(struct test *test, int expected);
 *    struct mock_param_matcher *int_ge(struct test *test, int expected);
 *
 * For a detailed list, please see
 * ``include/linux/mock.h``.
 */

/* Matches any argument */
struct mock_param_matcher *any(struct test *test);

/*
 * Matches different types of integers, the argument is compared to the
 * `expected` field, based on the comparison defined.
 */
struct mock_param_matcher *u8_eq(struct test *test, u8 expected);
struct mock_param_matcher *u8_ne(struct test *test, u8 expected);
struct mock_param_matcher *u8_le(struct test *test, u8 expected);
struct mock_param_matcher *u8_lt(struct test *test, u8 expected);
struct mock_param_matcher *u8_ge(struct test *test, u8 expected);
struct mock_param_matcher *u8_gt(struct test *test, u8 expected);

struct mock_param_matcher *u16_eq(struct test *test, u16 expected);
struct mock_param_matcher *u16_ne(struct test *test, u16 expected);
struct mock_param_matcher *u16_le(struct test *test, u16 expected);
struct mock_param_matcher *u16_lt(struct test *test, u16 expected);
struct mock_param_matcher *u16_ge(struct test *test, u16 expected);
struct mock_param_matcher *u16_gt(struct test *test, u16 expected);

struct mock_param_matcher *u32_eq(struct test *test, u32 expected);
struct mock_param_matcher *u32_ne(struct test *test, u32 expected);
struct mock_param_matcher *u32_le(struct test *test, u32 expected);
struct mock_param_matcher *u32_lt(struct test *test, u32 expected);
struct mock_param_matcher *u32_ge(struct test *test, u32 expected);
struct mock_param_matcher *u32_gt(struct test *test, u32 expected);

struct mock_param_matcher *u64_eq(struct test *test, u64 expected);
struct mock_param_matcher *u64_ne(struct test *test, u64 expected);
struct mock_param_matcher *u64_le(struct test *test, u64 expected);
struct mock_param_matcher *u64_lt(struct test *test, u64 expected);
struct mock_param_matcher *u64_ge(struct test *test, u64 expected);
struct mock_param_matcher *u64_gt(struct test *test, u64 expected);

struct mock_param_matcher *char_eq(struct test *test, char expected);
struct mock_param_matcher *char_ne(struct test *test, char expected);
struct mock_param_matcher *char_le(struct test *test, char expected);
struct mock_param_matcher *char_lt(struct test *test, char expected);
struct mock_param_matcher *char_ge(struct test *test, char expected);
struct mock_param_matcher *char_gt(struct test *test, char expected);

struct mock_param_matcher *uchar_eq(struct test *test, unsigned char expected);
struct mock_param_matcher *uchar_ne(struct test *test, unsigned char expected);
struct mock_param_matcher *uchar_le(struct test *test, unsigned char expected);
struct mock_param_matcher *uchar_lt(struct test *test, unsigned char expected);
struct mock_param_matcher *uchar_ge(struct test *test, unsigned char expected);
struct mock_param_matcher *uchar_gt(struct test *test, unsigned char expected);

struct mock_param_matcher *schar_eq(struct test *test, signed char expected);
struct mock_param_matcher *schar_ne(struct test *test, signed char expected);
struct mock_param_matcher *schar_le(struct test *test, signed char expected);
struct mock_param_matcher *schar_lt(struct test *test, signed char expected);
struct mock_param_matcher *schar_ge(struct test *test, signed char expected);
struct mock_param_matcher *schar_gt(struct test *test, signed char expected);

struct mock_param_matcher *short_eq(struct test *test, short expected);
struct mock_param_matcher *short_ne(struct test *test, short expected);
struct mock_param_matcher *short_le(struct test *test, short expected);
struct mock_param_matcher *short_lt(struct test *test, short expected);
struct mock_param_matcher *short_ge(struct test *test, short expected);
struct mock_param_matcher *short_gt(struct test *test, short expected);

struct mock_param_matcher *ushort_eq(struct test *test,
				     unsigned short expected);
struct mock_param_matcher *ushort_ne(struct test *test,
				     unsigned short expected);
struct mock_param_matcher *ushort_le(struct test *test,
				     unsigned short expected);
struct mock_param_matcher *ushort_lt(struct test *test,
				     unsigned short expected);
struct mock_param_matcher *ushort_ge(struct test *test,
				     unsigned short expected);
struct mock_param_matcher *ushort_gt(struct test *test,
				     unsigned short expected);

struct mock_param_matcher *int_eq(struct test *test, int expected);
struct mock_param_matcher *int_ne(struct test *test, int expected);
struct mock_param_matcher *int_lt(struct test *test, int expected);
struct mock_param_matcher *int_le(struct test *test, int expected);
struct mock_param_matcher *int_gt(struct test *test, int expected);
struct mock_param_matcher *int_ge(struct test *test, int expected);

struct mock_param_matcher *uint_eq(struct test *test, unsigned int expected);
struct mock_param_matcher *uint_ne(struct test *test, unsigned int expected);
struct mock_param_matcher *uint_lt(struct test *test, unsigned int expected);
struct mock_param_matcher *uint_le(struct test *test, unsigned int expected);
struct mock_param_matcher *uint_gt(struct test *test, unsigned int expected);
struct mock_param_matcher *uint_ge(struct test *test, unsigned int expected);

struct mock_param_matcher *long_eq(struct test *test, long expected);
struct mock_param_matcher *long_ne(struct test *test, long expected);
struct mock_param_matcher *long_le(struct test *test, long expected);
struct mock_param_matcher *long_lt(struct test *test, long expected);
struct mock_param_matcher *long_ge(struct test *test, long expected);
struct mock_param_matcher *long_gt(struct test *test, long expected);

struct mock_param_matcher *ulong_eq(struct test *test, unsigned long expected);
struct mock_param_matcher *ulong_ne(struct test *test, unsigned long expected);
struct mock_param_matcher *ulong_le(struct test *test, unsigned long expected);
struct mock_param_matcher *ulong_lt(struct test *test, unsigned long expected);
struct mock_param_matcher *ulong_ge(struct test *test, unsigned long expected);
struct mock_param_matcher *ulong_gt(struct test *test, unsigned long expected);

struct mock_param_matcher *longlong_eq(struct test *test, long long expected);
struct mock_param_matcher *longlong_ne(struct test *test, long long expected);
struct mock_param_matcher *longlong_le(struct test *test, long long expected);
struct mock_param_matcher *longlong_lt(struct test *test, long long expected);
struct mock_param_matcher *longlong_ge(struct test *test, long long expected);
struct mock_param_matcher *longlong_gt(struct test *test, long long expected);

struct mock_param_matcher *ulonglong_eq(struct test *test,
					unsigned long long expected);
struct mock_param_matcher *ulonglong_ne(struct test *test,
					unsigned long long expected);
struct mock_param_matcher *ulonglong_le(struct test *test,
					unsigned long long expected);
struct mock_param_matcher *ulonglong_lt(struct test *test,
					unsigned long long expected);
struct mock_param_matcher *ulonglong_ge(struct test *test,
					unsigned long long expected);
struct mock_param_matcher *ulonglong_gt(struct test *test,
					unsigned long long expected);

/* Matches pointers. */
struct mock_param_matcher *ptr_eq(struct test *test, void *expected);
struct mock_param_matcher *ptr_ne(struct test *test, void *expected);
struct mock_param_matcher *ptr_lt(struct test *test, void *expected);
struct mock_param_matcher *ptr_le(struct test *test, void *expected);
struct mock_param_matcher *ptr_gt(struct test *test, void *expected);
struct mock_param_matcher *ptr_ge(struct test *test, void *expected);

/* Matches memory sections and strings. */
struct mock_param_matcher *memeq(struct test *test,
				 const void *buf,
				 size_t size);
struct mock_param_matcher *streq(struct test *test, const char *str);

struct mock_param_matcher *str_contains(struct test *test, const char *needle);

/* Matches var-arg arguments. */
struct mock_param_matcher *va_format_cmp(struct test *test,
					 struct mock_param_matcher *fmt_matcher,
					 struct mock_param_matcher *va_matcher);

struct mock_action *u8_return(struct test *test, u8 ret);
struct mock_action *u16_return(struct test *test, u16 ret);
struct mock_action *u32_return(struct test *test, u32 ret);
struct mock_action *u64_return(struct test *test, u64 ret);
struct mock_action *char_return(struct test *test, char ret);
struct mock_action *uchar_return(struct test *test, unsigned char ret);
struct mock_action *schar_return(struct test *test, signed char ret);
struct mock_action *short_return(struct test *test, short ret);
struct mock_action *ushort_return(struct test *test, unsigned short ret);
struct mock_action *int_return(struct test *test, int ret);
struct mock_action *uint_return(struct test *test, unsigned int ret);
struct mock_action *long_return(struct test *test, long ret);
struct mock_action *ulong_return(struct test *test, unsigned long ret);
struct mock_action *longlong_return(struct test *test, long long ret);
struct mock_action *ulonglong_return(struct test *test, unsigned long long ret);
struct mock_action *ptr_return(struct test *test, void *ret);

/**
 * struct mock_struct_matcher_entry - composed with other &struct
 *                                    mock_struct_matcher_entry to make a
 *                                    &struct struct_matcher
 * @member_offset: offset of this member
 * @matcher: matcher for this particular member
 *
 * This is used for struct_cmp() matchers.
 */
struct mock_struct_matcher_entry {
	size_t member_offset;
	struct mock_param_matcher *matcher;
};

static inline void init_mock_struct_matcher_entry_internal(
		struct mock_struct_matcher_entry *entry,
		size_t offset,
		struct mock_param_matcher *matcher)
{
	entry->member_offset = offset;
	entry->matcher = matcher;
}

/**
 * INIT_MOCK_STRUCT_MATCHER_ENTRY()
 * @entry: the &struct mock_struct_matcher_entry to initialize
 * @type: the struct being matched
 * @member: the member of the struct being matched, used to calculate the offset
 * @matcher: matcher to match that member
 *
 * Initializes ``entry`` to match ``type->member`` with ``matcher``.
 */
#define INIT_MOCK_STRUCT_MATCHER_ENTRY(entry, type, member, matcher)	       \
		init_mock_struct_matcher_entry_internal(entry,		       \
							offsetof(type, member),\
							matcher)

static inline void INIT_MOCK_STRUCT_MATCHER_ENTRY_LAST(
		struct mock_struct_matcher_entry *entry)
{
	entry->matcher = NULL;
}

struct mock_param_matcher *struct_cmp(
		struct test *test,
		const char *struct_name,
		struct mock_struct_matcher_entry *entries);

/**
 * struct mock_param_capturer - used to capture parameter when matching
 *
 * Use the associated helper macros to access relevant fields.
 * Example:
 *
 * .. code-block::c
 *
 *	static int some_test(struct test *test)
 *	{
 *		// imagine a mocked function: int add(int a, int b)
 *		struct mock_param_capturer *capturer =
 *			mock_int_capturer_create(test, any(test));
 *		EXPECT_CALL(add(any(test), capturer_to_matcher(capturer)));
 *		ASSERT_PARAM_CAPTURED(test, capturer);
 *
 *		int captured_value = mock_capturer_get(capturer, int);
 *	}
 */
struct mock_param_capturer {
	/* private: internal use only. */
	struct mock_param_matcher matcher;
	struct mock_param_matcher *child_matcher;
	void *(*capture_param)(struct test *test, const void *param);
	void *captured_param;
};

struct mock_param_capturer *mock_param_capturer_create(
		struct test *test,
		struct mock_param_matcher *child_matcher,
		void *(*capture_param)(struct test *, const void *));

/**
 * mock_int_capturer_create() - creates a int parameter capturer
 * @test: associated test
 * @child_matcher: matcher used to match the integer
 *
 * The capturer will capture the value if the matcher is satisfied.
 */
struct mock_param_capturer *mock_int_capturer_create(
		struct test *test, struct mock_param_matcher *child_matcher);

/**
 * mock_int_capturer_create() - creates a generic pointer parameter capturer
 * @test: associated test
 * @child_matcher: matcher used to match the pointer
 *
 * The capturer will capture the value if the matcher is satisfied
 */
struct mock_param_capturer *mock_ptr_capturer_create(
		struct test *test, struct mock_param_matcher *child_matcher);

/**
 * capturer_to_matcher()
 * @capturer: the param capturer
 *
 * Use this function when passing a capturer into an EXPECT_CALL() where a
 * matcher would be expected. See the example for &struct mock_param_capturer.
 */
#define capturer_to_matcher(capturer) (&(capturer)->matcher)

/**
 * ASSERT_PARAM_CAPTURED(): Asserts that the capturer has captured a parameter.
 * @test: the associated test
 * @capturer: the param capturer
 *
 * See &struct mock_param_capturer for an example.
 */
#define ASSERT_PARAM_CAPTURED(test, capturer)				       \
		ASSERT(test,						       \
		       !IS_ERR_OR_NULL((capturer)->captured_param),	       \
		       "Asserted " #capturer " captured param, but did not.")

/**
 * mock_capturer_get(): Returns the value captured by ``capturer``
 * @capturer: the param capturer
 * @type: the type of the value
 *
 * See &struct mock_param_capturer for an example.
 */
#define mock_capturer_get(capturer, type) \
		CONVERT_TO_ACTUAL_TYPE(type, (capturer)->captured_param)

struct mock_action *invoke(struct test *test,
			   void *(*invokable)(struct test *,
					      const void *params[],
					      int len));

/**
 * INVOKE_REAL()
 * @test: associated test
 * @func_name: name of the function
 *
 * See DEFINE_REDIRECT_MOCKABLE() for an example.
 *
 * Return: &struct mock_action that makes the associated mock method or function
 *         call the original function definition of a redirect-mockable
 *         function.
 */
#define INVOKE_REAL(test, func_name) invoke(test, INVOKE_ID(func_name))

struct mock_struct_formatter_entry {
	size_t member_offset;
	struct mock_param_formatter *formatter;
};

static inline void init_mock_struct_formatter_entry_internal(
		struct mock_struct_formatter_entry *entry,
		size_t offset,
		struct mock_param_formatter *formatter)
{
	entry->member_offset = offset;
	entry->formatter = formatter;
}

#define INIT_MOCK_STRUCT_FORMATTER_ENTRY(entry, type, member, formatter)       \
		init_mock_struct_formatter_entry_internal(entry,	       \
							  offsetof(type,       \
								   member),    \
								   formatter)

static inline void INIT_MOCK_STRUCT_FORMATTER_ENTRY_LAST(
		struct mock_struct_formatter_entry *entry)
{
	entry->formatter = NULL;
}

struct mock_param_formatter *mock_struct_formatter(
		struct test *test,
		const char *struct_name,
		struct mock_struct_formatter_entry *entries);

struct mock_param_formatter *mock_find_formatter(const char *type_name);

#define FORMATTER_FROM_TYPE(type) mock_find_formatter(#type)

extern struct mock_param_formatter unknown_formatter[];

#endif /* _TEST_MOCK_H */
