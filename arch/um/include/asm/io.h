/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_UM_IO_H
#define _ASM_UM_IO_H

#include <linux/types.h>
#include <asm/byteorder.h>

#if IS_ENABLED(CONFIG_PLATFORM_MOCK)
#include <asm/io-mock-shared.h>
#endif

#define ioremap ioremap
static inline void __iomem *ioremap(phys_addr_t offset, size_t size)
{
	return (void __iomem *)(unsigned long)offset;
}
#define ioremap_nocache ioremap

#define iounmap iounmap
static inline void iounmap(void __iomem *addr)
{
}

/*
 * This is really for allowing drivers and subsystems which depend on an IOMMU
 * to be unit tested via KUnit.
 *
 * TODO(brendanhiggins): we should probably replace this with a single config
 * option.
 */
#if IS_ENABLED(CONFIG_MMU) && IS_ENABLED(CONFIG_TEST)
static inline void __iomem *ioremap_wc(phys_addr_t addr, size_t size)
{
	return (void __iomem *)(unsigned long) addr;
}

static inline void __iomem *ioremap_wt(phys_addr_t addr, size_t size)
{
	return (void __iomem *)(unsigned long) addr;
}
#endif

#include <asm-generic/io.h>

#endif
