.. SPDX-License-Identifier: GPL-2.0

===============
Getting Started
===============

Installing dependencies
=======================
KUnit has the same dependencies as the Linux kernel. As long as you can build
the kernel, you can run KUnit.

KUnit Wrapper
=============
Included with KUnit is a simple Python wrapper that helps format the output to
easily use and read KUnit output. It handles building and running the kernel, as
well as formatting the output.

The wrapper can be run with:

.. code-block:: bash

   ./tools/testing/kunit/kunit.py

Creating a kunitconfig
======================
The Python script is a thin wrapper around Kbuild as such, it needs to be
configured with a ``kunitconfig`` file. This file essentially contains the
regular Kernel config, with the specific test targets as well.

.. TODO(brendanhiggins@google.com): I guess we need to release a kunitconfig
   when we release KUnit. We need to create a repo and then link it here.

.. code-block:: bash

	PLACEHOLDER_COMMAND_TO_GET_RELEASED_KUNITCONFIG

You may want to add kunitconfig to your local gitignore.

Verifying KUnit Works
-------------------------

To make sure that everything is set up correctly, simply invoke the Python
wrapper from your kernel repo:

.. code-block:: bash

	./tools/testing/kunit/kunit.py

.. note::
   You may want to run ``make mrproper`` first.

If everything worked correctly, you should see the following:

.. code-block:: bash

	Generating .config ...
	Building KUnit Kernel ...
	Starting KUnit Kernel ...

followed by a list of tests that are run. All of them should be passing.

.. note::
   Because it is building a lot of sources for the first time, the ``Building
   kunit kernel`` step may take a while.

Writing your first test
==========================

In your kernel repo let's add some code that we can test. Create a file
``drivers/misc/example.h`` with the contents:

.. code-block:: c

	struct misc_example {
		int (*misc_example_foo)(struct misc_example *, int);
	};

	int misc_example_bar(struct misc_example *example);

create a file ``drivers/misc/example.c``:

.. code-block:: c

	#include <linux/errno.h>

	#include "example.h"

	int misc_example_bar(struct misc_example *example)
	{
		if (example->misc_example_foo(example, 5))
			return -EIO;
		else
			return 0;
	}

Now add the following lines to ``drivers/misc/Kconfig``:

.. code-block:: kconfig

	config MISC_EXAMPLE
		bool "My example"

and the following lines to ``drivers/misc/Makefile``:

.. code-block:: make

	obj-$(CONFIG_MISC_EXAMPLE) += example.o

Now we are ready to write the test. The test will be in
``drivers/misc/example-test.c``:

.. code-block:: c

	#include <linux/test.h>
	#include <linux/mock.h>
	#include "example.h"

	/* Define the mock. */

	DECLARE_STRUCT_CLASS_MOCK_PREREQS(misc_example);

	DEFINE_STRUCT_CLASS_MOCK(METHOD(misc_example_foo), CLASS(misc_example),
				 RETURNS(int),
				 PARAMS(struct misc_example *, int));

	static int misc_example_init(struct MOCK(misc_example) *mock_example)
	{
		struct misc_example *example = mock_get_trgt(mock_example);

		example->misc_example_foo = misc_example_foo;
		return 0;
	}

	DEFINE_STRUCT_CLASS_MOCK_INIT(misc_example, misc_example_init);

	/* Define the test cases. */

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

	static void misc_example_bar_test_failure(struct test *test)
	{
		struct MOCK(misc_example) *mock_example = test->priv;
		struct misc_example *example = mock_get_trgt(mock_example);
		struct mock_expectation *handle;

		handle = EXPECT_CALL(misc_example_foo(mock_get_ctrl(mock_example),
						      int_eq(test, 5)));
		handle->action = int_return(test, -EINVAL);

		EXPECT_EQ(test, -EINVAL, misc_example_bar(example));
	}

	static int misc_example_test_init(struct test *test)
	{
		test->priv = CONSTRUCT_MOCK(misc_example, test);
		if (!test->priv)
			return -EINVAL;

		return 0;
	}

	static void misc_example_test_exit(struct test *test)
	{
	}

	static struct test_case misc_example_test_cases[] = {
		TEST_CASE(misc_example_bar_test_success),
		TEST_CASE(misc_example_bar_test_failure),
		{},
	};

	static struct test_module misc_example_test_module = {
		.name = "misc-example",
		.init = misc_example_test_init,
		.exit = misc_example_test_exit,
		.test_cases = misc_example_test_cases,
	};
	module_test(misc_example_test_module);

Now add the following to ``drivers/misc/Kconfig``:

.. code-block:: kconfig

	config MISC_EXAMPLE_TEST
		bool "Test for my example"
		depends on MISC_EXAMPLE && TEST

and the following to ``drivers/misc/Makefile``:

.. code-block:: make

	obj-$(CONFIG_MISC_EXAMPLE_TEST) += example-test.o

Now add it to your ``kunitconfig``:

.. code-block:: none

	CONFIG_MISC_EXAMPLE=y
	CONFIG_MISC_EXAMPLE_TEST=y

Now you can run the test:

.. code-block:: bash

	./tools/testing/kunit/kunit.py

You should see the following failure:

.. code-block:: none

	...
	kunit misc-example: misc_example_bar_test_success passed
	kunit misc-example: EXPECTATION FAILED at drivers/misc/example-test.c:48
		Expected -22 == misc_example_bar(example), but
			-22 == -22
			misc_example_bar(example) == -5
	kunit misc-example: misc_example_bar_test_failure failed
	kunit misc-example: one or more tests failed

Congrats! You just wrote your first KUnit test!

Next Steps
=============
*   Check out the :doc:`usage` page for a more
    in-depth explanation of KUnit.
