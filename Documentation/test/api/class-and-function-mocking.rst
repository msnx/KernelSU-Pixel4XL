.. SPDX-License-Identifier: GPL-2.0

==========================
Class and Function Mocking
==========================

This file documents class and function mocking features.

.. note::
   If possible, prefer class mocking over arbitrary function mocking. Class
   mocking has a much more limited scope and provides more control.
   This file documents class mocking and most mocking features that do not
   depend on function or platform mocking.

Readability Macros
------------------
When defining and declaring mock stubs, use these readability macros.

.. code-block:: c

        #define CLASS(struct_name) struct_name
        #define HANDLE_INDEX(index) index
        #define METHOD(method_name) method_name
        #define RETURNS(return_type) return_type
        #define PARAMS(...) __VA_ARGS__

Consider a ``struct Foo`` with a member function
``int add(struct Foo*, int a, int b);``

When generating a mock stub with :c:func:`DEFINE_STRUCT_CLASS_MOCK`, which
takes a method name, struct name, return type, and method parameters, the
arguments should be passed in with the readability macros.

.. code-block:: c

        DEFINE_STRUCT_CLASS_MOCK(
                METHOD(add),
                CLASS(Foo),
                RETURNS(int),
                PARAMS(struct Foo *, int, int)
        );

For a more detailed example of this, take a look at the example in
:doc:`../start`

These macros should only be used in the context of the mock stub generators.


Built in Matchers
-----------------

.. kernel-doc:: include/test/mock.h
   :doc: Built In Matchers

Mock Returns
------------
These functions can be used to specify a value to be returned (``ret``) when a
mocked function is intercepted via :c:func:`EXPECT_CALL`.

.. code-block:: c

        struct mock_action *int_return(struct test *test, int ret);
        struct mock_action *u32_return(struct test *test, u32 ret);

API
---
.. kernel-doc:: include/test/mock.h
   :internal:
