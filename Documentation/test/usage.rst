.. SPDX-License-Identifier: GPL-2.0

=============
Using KUnit
=============

The purpose of this document is to describe what KUnit is, how it works, how it
is intended to be used, and all the concepts and terminology that are needed to
understand it. This guide assumes a working knowledge of the Linux kernel and
some basic knowledge of testing.

For a high level introduction to KUnit, including setting up KUnit for your
project, see :doc:`start`.

Organization of this document
=================================

This document is organized into two main sections: Testing and Mocking. The
first covers what a unit test is and how to use KUnit to write them. The second
covers how to use KUnit to mock out dependencies and make it possible to unit
test code that was otherwise un-unit-testable.

Testing
==========

What is KUnit?
------------------

"K" is short for "kernel" so "KUnit" is the "(Linux) Kernel Unit Testing
Framework." KUnit is intended first and foremost for writing unit tests; it is
general enough that it can be used to write integration tests; however, this is
a secondary goal. KUnit has no ambition of being the only testing framework for
the kernel; for example, it does not intend to be an end-to-end testing
framework.

What is Unit Testing?
-------------------------

A *unit test* is a test that tests code at the smallest possible scope, a
*unit* of code. For the C programming language, defining a unit is easy: it is
the compilation unit; however, that does not mean compilation units cannot be
too large and should not be broken down to get more coverage or just because
they are hard to reason about.

A unit test should test all the publicly exposed functions in a unit; so that is
all the functions that are exported in either a *class* (defined below) or all
functions which are **not** static.

Writing Tests
-------------

Test Cases
~~~~~~~~~~

The fundamental unit in KUnit is the test case. A test case is a function with
the signature ``void (*)(struct test *test)``. It calls a function to be tested
and then sets *expectations* for what should happen. For example:

.. code-block:: c

	void example_test_success(struct test *test)
	{
	}

	void example_test_failure(struct test *test)
	{
		FAIL(test, "This test never passes.");
	}

In the above example ``example_test_success`` always passes because it does
nothing; no expectations are set, so all expectations pass. On the other hand
``example_test_failure`` always fails because it calls ``FAIL``, which is a
special expectation that logs a message and causes the test case to fail.

Preferably, a single test case will be pretty short, it should be pretty easy to
understand, and it should really only try to test at most a handful of very
closely relating things.

Expectations
~~~~~~~~~~~~
An *expectation* is a way to make an assertion about what a piece of code should
do in a test. An expectation is called like a function. A test is made by
setting expectations about the behavior of a piece of code under test; when one
or more of the expectations fail, the test case fails and information about the
failure is logged. For example:

.. code-block:: c

	void add_test_basic(struct test *test)
	{
		EXPECT_EQ(test, 1, add(1, 0));
		EXPECT_EQ(test, 2, add(1, 1));
		EXPECT_EQ(test, 0, add(-1, 1));
		EXPECT_EQ(test, INT_MAX, add(0, INT_MAX));
		EXPECT_EQ(test, -1, add(INT_MAX, INT_MIN));
	}

In the above example ``add_test_basic`` makes a number of assertions about the
behavior of a function called ``add``; the first parameter is always of type
``struct test *``, which contains information about the current test context; the
second parameter, in this case, is what the value is expected to be; the last
value is what the value actually is. If ``add`` passes all of these expectations,
the test case, ``add_test_basic`` will pass; if any one of these expectations
fail, the test case will fail.

To learn about more expectations supported by KUnit, see :doc:`api/test`.

Assertions
~~~~~~~~~~

KUnit also has the concept of an *assertion*. An assertion is just like an
expectation except the assertion immediately terminates the test case if it is
not satisfied.

For example:

.. code-block:: c

	static void mock_test_do_expect_default_return(struct test *test)
	{
		struct mock_test_context *ctx = test->priv;
		struct mock *mock = ctx->mock;
		int param0 = 5, param1 = -5;
		const char *two_param_types[] = {"int", "int"};
		const void *two_params[] = {&param0, &param1};
		const void *ret;

		ret = mock->do_expect(mock,
				      "test_printk", test_printk,
				      two_param_types, two_params,
				      ARRAY_SIZE(two_params));
		ASSERT_NOT_ERR_OR_NULL(test, ret);
		EXPECT_EQ(test, -4, *((int *) ret));
	}

In this example, the method under test should return a pointer to a value, so if
the pointer returned by the method is null or an errno, we don't want to bother
continuing the test since the following expectation could crash the test case
if the pointer is null. `ASSERT_NOT_ERR_OR_NULL(...)` allows us to bail out of
the test case if the appropriate conditions have not been satisfied to complete
the test.

Modules / Test Suites
~~~~~~~~~~~~~~~~~~~~~

Because test cases are supposed to test just a single function and then only
test a set of closely related concepts about that function, multiple test cases
are usually needed to fully test a unit. This is where the concept of a *test
suite* or a *module* comes in; it is just a collection of test cases for a unit
of code. In addition to specifying a set of test cases, a test suite also
supports specifying setup and tear down functions which allow functions to be
specified to run before and after each test case, respectively; this is useful
when the procedure to setup a unit for testing requires several steps and needs
to be done in every test case.

Example:

.. code-block:: c

	static struct test_case example_test_cases[] = {
		TEST_CASE(example_test_foo),
		TEST_CASE(example_test_bar),
		TEST_CASE(example_test_baz),
		{},
	};

	static struct test_module example_test_module[] = {
		.name = "example",
		.init = example_test_init,
		.exit = example_test_exit,
		.test_cases = example_test_cases,
	};
	module_test(example_test_module);

In the above example the test suite, ``example_test_module``, would run the test
cases ``example_test_foo``, ``example_test_bar``, and ``example_test_baz``, each
would have ``example_test_init`` called immediately before it and would have
``example_test_exit`` called immediately after it.
``module_test(example_test_module)`` registers the test suite with the KUnit
test framework.

.. note::
   A test case will only be run if it is associated with a test suite.

For a more information on these types of things see the :doc:`api/test`.

Mocking
=======

The most important aspect of unit testing that other forms of testing do not
provide is the ability to limit the amount of code under test to a single unit.
In practice, this is only possible by being able to control what code gets run
when the unit under test calls a function and this is usually accomplished
through some sort of indirection where a function is exposed as part of an API
such that the definition of that function can be changed without affecting the
rest of the code base. In the kernel this primarily comes from two constructs,
classes, structs that contain function pointers that are provided by the
implementer, and architecture specific functions which have definitions selected
at compile time.

Classes
-------

Classes are not a construct that is built into the C programming language;
however, it is an easily derived concept. Accordingly, pretty much every project
that does not use a standardized object oriented library (like GNOME's GObject)
has their own slightly different way of doing object oriented programming; the
Linux kernel is no exception.

The central concept in the kernel object oriented programming is the class. In
the kernel, a *class* is a struct that contains function pointers. This creates
a contract between *implementers* and *users* since it forces them to use the
same function signature without having to call the function directly. In order
for it to truly be a class, the function pointers must specify that a pointer to
the class, known as a *class handle*, be one of the parameters; this makes it
possible for the member functions (also known as *methods*) to have access to
member variables (more commonly known as *fields*) allowing the same
implementation to have multiple *instances*.

Typically a class can be *overridden* by *child classes* by embedding the
*parent class* in the child class. Then when a method provided by the child
class is called, the child implementation knows that the pointer passed to it is
of a parent contained within the child; because of this, the child can compute
the pointer to itself because the pointer to the parent is always a fixed offset
from the pointer to the child; this offset is the offset of the parent contained
in the child struct. For example:

.. code-block:: c

	struct shape {
		int (*area)(struct shape *this);
	};

	struct rectangle {
		struct shape parent;
		int length;
		int width;
	};

	int rectangle_area(struct shape *this)
	{
		struct rectangle *self = container_of(this, struct shape, parent);

		return self->length * self->width;
	};

	void rectangle_new(struct rectangle *self, int length, int width)
	{
		self->parent.area = rectangle_area;
		self->length = length;
		self->width = width;
	}

In this example (as in most kernel code) the operation of computing the pointer
to the child from the pointer to the parent is done by ``container_of``.

Mocking Classes
~~~~~~~~~~~~~~~

In order to unit test a piece of code that calls a method in a class, the
behavior of the method must be controllable, otherwise the test ceases to be a
unit test and becomes an integration test. KUnit allows classes to be *mocked*
which means that it generates subclasses whose behavior can be specified in a
test case. KUnit accomplishes this with two sets of macros: the mock generation
macros and the ``EXPECT_CALL`` macro.

For example, let's say you have a file named ``drivers/foo/example.c`` and it
contains the following code you would like to test:

.. code-block:: c

	int example_bar(struct example *example)
	{
		if (example->foo(example, 5))
			return -EIO;
		else
			return 0;
	}

For the purposes of this example, assume ``struct example`` is defined as such:

.. code-block:: c

	struct example {
		int (*foo)(struct example *, int);
	};

You would create a test file named ``drivers/foo/example-test.c`` and it would
contain the following code:

.. code-block:: c

	/* Define the mock. */

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

	/* Define the test cases. */

	static void foo_example_test_success(struct test *test)
	{
		struct MOCK(example) *mock_example = test->priv;
		struct example *example = mock_get_trgt(mock_example);
		struct mock_expectation *handle;

		handle = EXPECT_CALL(foo(mock_get_ctrl(mock_example), int_eq(test, 5)));
		handle->action = int_return(test, 0);

		EXPECT_EQ(test, 0, example_bar(example));
	}

	static void foo_example_test_failure(struct test *test)
	{
		struct MOCK(example) *mock_example = test->priv;
		struct example *example = mock_get_trgt(mock_example);
		struct mock_expectation *handle;

		handle = EXPECT_CALL(foo(mock_get_ctrl(mock_example), int_eq(test, 5)));
		handle->action = int_return(test, -EINVAL);
		EXPECT_EQ(test, -EINVAL, example_bar(example));
	}

	static int example_test_init(struct test *test)
	{
		test->priv = CONSTRUCT_MOCK(example, test);
		if (!test->priv)
			return -EINVAL;

		return 0;
	}

	static void example_test_exit(struct test *test)
	{
	}

	static struct test_case foo_example_test_cases[] = {
		TEST_CASE(foo_example_test_success),
		TEST_CASE(foo_example_test_failure),
		{},
	};

	static struct test_module foo_example_test_module = {
		.name = "example",
		.init = example_test_init,
		.exit = example_test_exit,
		.test_cases = foo_example_test_cases,
	};
	module_test(foo_example_test_module);

``foo_example_test_success`` uses the mock allocated in init. It asserts that
``example.foo`` will get called with ``5`` as a parameter with the ``int_eq``
parameter matcher. ``EXPECT_CALL`` the returns a handle that a user can use to
specify additional behavior on the mock; it must always specify a return value
using an *action*. Finally, it calls the function under test.

For more information on class mocking see :doc:`api/class-and-function-mocking`.

Mocking Arbitrary Functions
---------------------------

.. important::
   Always prefer class mocking over arbitrary function mocking where possible.
   Class mocking has a much more limited scope and provides more control.

Sometimes it is necessary to mock a function that does not use any class style
indirection. First and foremost, if you encounter this in your own code, please
rewrite it so that uses class style indirection discussed above, but if this is
in some code that is outside of your control you may use KUnit's function
mocking features.

KUnit provides macros to allow arbitrary functions to be overridden so that the
original definition is replaced with a mock stub. For most functions, all you
have to do is label the function ``__mockable``:

.. code-block:: c

	int __mockable example(int arg) {...}

If a function is ``__mockable`` and a mock is defined:

.. code-block:: c

	DEFINE_FUNCTION_MOCK(example, RETURNS(int), PARAMS(int));

When the function is called, the mock stub will actually be called.

.. note::
   There is no performance penalty or potential side effects from doing this.
   When not compiling for testing, ``__mockable`` compiles away.

.. note::
   ``__mockable`` does not work on inlined functions.

Redirect-mockable
~~~~~~~~~~~~~~~~~

Sometimes it is desirable to have a mock function that delegates to the original
definition in some or all circumstances. This is possible by making the function
*redirect-mockable*:

.. code-block:: c

	DEFINE_REDIRECT_MOCKABLE(i2c_add_adapter, RETURNS(int), PARAMS(struct i2c_adapter *));
	int REAL_ID(i2c_add_adapter)(struct i2c_adapter *adapter)
	{
		...
	}

This allows the function to be overridden by a mock as with ``__mockable``;
however, it associates the original definition of the function with alternate
symbol that KUnit can still reference. This makes it possible to mock the
function and then have the mock delegate to the original function definition
with the ``INVOKE_REAL(...)`` action:

.. code-block:: c

	static int aspeed_i2c_test_init(struct test *test)
	{
		struct mock_param_capturer *adap_capturer;
		struct mock_expectation *handle;
		struct aspeed_i2c_test *ctx;
		int ret;

		ctx = test_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
		if (!ctx)
			return -ENOMEM;
		test->priv = ctx;

		handle = EXPECT_CALL(
				i2c_add_adapter(capturer_to_matcher(adap_capturer)));
		handle->action = INVOKE_REAL(test, i2c_add_adapter);
		ret = of_fake_probe_platform_by_name(test,
						     "aspeed-i2c-bus",
						     "test-i2c-bus");
		if (ret < 0)
			return ret;

		ASSERT_PARAM_CAPTURED(test, adap_capturer);
		ctx->adap = mock_capturer_get(adap_capturer, struct i2c_adapter *);

		return 0;
	}

For more information on function mocking see
:doc:`api/class-and-function-mocking`.

Platform Mocking
----------------
The Linux kernel generally forbids normal code from accessing architecture
specific features. Instead, low level hardware features are usually abstracted
so that architecture specific code can live in the ``arch/`` directory and all
other code relies on APIs exposed by it.

KUnit provides a mock architecture that currently allows mocking basic IO memory
accessors and in the future will provide even more. A major use case for
platform mocking is unit testing platform drivers, so KUnit also provides
helpers for this as well.

In order to use platform mocking, ``CONFIG_PLATFORM_MOCK`` must be enabled in
your ``kunitconfig``.

For more information on platform mocking see :doc:`api/platform-mocking`.

Method Call Expectations
========================
Once we have classes and methods mocked, we can place more advanced
expectations. Previously, we could only place expectations on simple return
values. With the :c:func:`EXPECT_CALL` macro, which allows you to make
assertions that a certain mocked function is called with specific arguments
given some code to be run.

Basic Usage
-----------
Imagine we had some kind of dependency like this:

.. code-block:: c

	struct Printer {
		void (*print)(int arg);
	};

	// Printer's print
	void printer_print(int arg)
	{
		do_something_to_print_to_screen(arg);
	}

	struct Foo {
		struct Printer *internal_printer;
		void (*print_add_two)(struct Foo*, int);
	};

	// Foo's print_add_two:
	void foo_print_add_two(struct Foo *this, int arg)
	{
		internal_printer->print(arg + 2);
	}

and we wanted to test ``struct Foo``'s behaviour, that ``foo->print_add_two``
actually adds 2 to the argument passed. To properly unit test this, we create
mocks for all of ``struct Foo``'s dependencies, like ``struct Printer``.
We first setup stubs for ``MOCK(Printer)`` and its ``print`` function.

In the real code, we'd assign a real ``struct Printer`` to the
``internal_printer`` variable in our ``struct Foo`` object, but in the
test, we'd construct a ``struct Foo`` with our ``MOCK(Printer)``.

Finally, we can place expectations on the ``MOCK(Printer)``.

For example:

.. code-block:: c

	static int test_foo_add_two(struct test *test)
	{
		struct MOCK(Printer) *mock_printer = get_mocked_printer();
		struct Foo *foo = initialize_foo(mock_printer);

		// print() is a mocked method stub
		EXPECT_CALL(print(any(test), int_eq(12)));

		foo->print_add_two(foo, 10);
	}

Here, we expect that the printer's print function will be called (by default,
once), and that it will be called with the argument ``12``. Once we've placed
expectations, we can call the function we want to test to see that it behaves
as we expected.

Matchers
--------
Above, we see ``any`` and ``int_eq``, which are matchers. A matcher simply
asserts that the argument passed to that function call fulfills some condition.
In this case, ``any()`` matches any argument, and ``int_eq(12)`` asserts that
the argument passed to that function must equal 12. If we had called:
``foo->print_add_two(foo, 9)`` instead, the expectation would not have been
fulfilled. There are a variety of built-in matchers:
:doc:`api/class-and-function-mocking` has a section about these matchers.

.. note::
	:c:func:`EXPECT_CALL` only works with mocked functions and methods.
	Matchers may only be used within the function inside the
	:c:func:`EXPECT_CALL`.

Additional :c:func:`EXPECT_CALL` Properties
-------------------------------------------

The return value of :c:func:`EXPECT_CALL` is a
:c:func:`struct mock_expectation`. We can capture the value and add extra
properties to it as defined by the :c:func:`struct mock_expectation` interface.

Times Called
~~~~~~~~~~~~
In the previous example, if we wanted assert that the method is never called,
we could write:

.. code-block:: c

	...
	struct mock_expectation* handle = EXPECT_CALL(...);
	handle->min_calls_expected = 0;
	handle->max_calls_expected = 0;
	...

Both those fields are set to 1 by default and can be changed to assert a range
of times that the method or function is called.

Mocked Actions
~~~~~~~~~~~~~~
Because ``mock_printer`` is a mock, it doesn't actually perform any task. If
the function had some side effect that ``struct Foo`` requires to have been
done, such as modifying some state, we could mock that as well.

Each expectation has an associated :c:func:`struct mock_action` which can be
set with ``handle->action``. By default, there are two actions that mock return
values. Those can also be found in :doc:`api/class-and-function-mocking`.

Custom actions can be defined by simply creating a :c:func:`struct mock_action`
and assigning the appropriate function to ``do_action``. Mocked actions have
access to the parameters passed to the mocked function, as well as have the
ability to change / set the return value.


The Nice, the Strict, and the Naggy
===================================
KUnit has three different mock types that can be set on a mocked class: nice
mocks, strict mocks, and naggy mocks. These are set via the corresponding macros
:c:func:`NICE_MOCK`, :c:func:`STRICT_MOCK`, and :c:func:`NAGGY_MOCK`, with naggy
mocks being the default.

The type of mock simply dictates the behaviour the mock exhibits when
expectations are placed on it.

+-----------------------+------------+--------------------+--------------------+
|                       | **Nice**   | **Naggy (default)**| **Strict**         |
+-----------------------+------------+--------------------+--------------------+
| Method called with no | Do nothing | Prints warning for | Fails test, prints |
| expectations on it    |            | uninteresting call | warning            |
|                       |            |                    | uninteresting call |
+-----------------------+------------+--------------------+--------------------+
| Method called with no | Fails test, prints warnings, prints tried            |
| matching expectations | expectations                                         |
| on it                 |                                                      |
+-----------------------+------------------------------------------------------+
| Test ends with an     | Fail test, print warning                             |
| unfulfilled           |                                                      |
| expectation           |                                                      |
+-----------------------+------------------------------------------------------+

These macros take a ``MOCK(struct_name)`` and so should be used when retrieving
the mocked object. Following the example in :doc:`start`, there was this test
case:

.. code-block:: c

	static void misc_example_bar_test_success(struct test *test)
	{
		struct MOCK(misc_example) *mock_example = test->priv;
		struct misc_example *example = mock_get_trgt(mock_example);
		struct mock_expectation *handle;

		handle = EXPECT_CALL(misc_example_foo(mock_get_ctrl(mock_example),
						      int_eq(test, 5)));
		handle->action = int_return(test, 0);

		EXPECT_EQ(test, 0, misc_example_bar(example));
	}

If we wanted ``mock_example`` to be a nice mock instead, we would simply write:

.. code-block:: c

	struct MOCK(misc_example) *mock_example = NICE_MOCK(test->priv);
