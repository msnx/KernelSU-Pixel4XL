.. SPDX-License-Identifier: GPL-2.0

================
Platform Mocking
================

This file documents *platform mocking*, mocking libraries that mock out platform
specific features and aid in writing mocks for platform drivers and other low
level kernel code.

Enable Platform Mocking
-----------------------
``CONFIG_PLATFORM_MOCK`` needs to be added to the .config (or kunitconfig) to
enable platform mocking.

Mocked IO Functions
-------------------
The following functions have been mocked for convenience.

.. code-block:: c

	u8 readb(const volatile void __iomem *);
	u16 readw(const volatile void __iomem *);
	u32 readl(const volatile void __iomem *);
	u64 readq(const volatile void __iomem *);
	void writeb(u8, const volatile void __iomem *);
	void writew(u16, const volatile void __iomem *);
	void writel(u32, const volatile void __iomem *);
	void writeq(u64, const volatile void __iomem *);

.. note:: These functions do not have any non-mocked behaviour in UML.

API
---
.. kernel-doc:: include/linux/platform_device_mock.h
   :internal:
