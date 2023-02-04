/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_UM_IO_MOCK_SHARED_H
#define _ASM_UM_IO_MOCK_SHARED_H

#define readb readb
u8 readb(const volatile void __iomem *);

#define readw readw
u16 readw(const volatile void __iomem *);

#define readl readl
u32 readl(const volatile void __iomem *);

#ifdef CONFIG_64BIT
#define readq readq
u64 readq(const volatile void __iomem *);
#endif /* CONFIG_64BIT */

#define writeb writeb
void writeb(u8, const volatile void __iomem *);

#define writew writew
void writew(u16, const volatile void __iomem *);

#define writel writel
void writel(u32, const volatile void __iomem *);

#ifdef CONFIG_64BIT
#define writeq writeq
void writeq(u64, const volatile void __iomem *);
#endif /* CONFIG_64BIT */

#endif /* _ASM_UM_IO_MOCK_SHARED_H */
