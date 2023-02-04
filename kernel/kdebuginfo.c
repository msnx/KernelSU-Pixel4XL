/* Copyright 2018  Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/slab.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/bldr_debug_tools.h>
#include <linux/utsname.h>

/*
 * These will be re-linked against their real values
 * during the second link stage.
 */
extern const unsigned long kallsyms_addresses[] __weak;
extern const int kallsyms_offsets[] __weak;
extern const u8 kallsyms_names[] __weak;

/*
 * Tell the compiler that the count isn't in the small data section if the arch
 * has one (eg: FRV).
 */
extern const unsigned long kallsyms_num_syms
__attribute__((weak, section(".rodata")));

extern const unsigned long kallsyms_relative_base
__attribute__((weak, section(".rodata")));

extern const u8 kallsyms_token_table[] __weak;
extern const u16 kallsyms_token_index[] __weak;

extern const unsigned long kallsyms_markers[] __weak;

static phys_addr_t all_info_addr;

/*
 * Header structure must be byte-packed, since the table is provided to
 * bootloader.
 */
struct kernel_info {
	/* For kallsyms */
	u8 enabled_all;
	u8 enabled_base_relative;
	u8 enabled_absolute_percpu;
	u8 enabled_cfi_clang;
	u32 num_syms;
	u16 name_len;
	u16 bit_per_long;
	u16 module_name_len;
	u16 symbol_len;
	phys_addr_t _addresses_va;
	phys_addr_t _relative_va;
	phys_addr_t _stext_va;
	phys_addr_t _etext_va;
	phys_addr_t _sinittext_va;
	phys_addr_t _einittext_va;
	phys_addr_t _end_va;
	phys_addr_t _offsets_va;
	phys_addr_t _names_va;
	phys_addr_t _token_table_va;
	phys_addr_t _token_index_va;
	phys_addr_t _markers_va;

	/* For frame pointer */
	u32 thread_size;

	/* For virt_to_phys */
	u32 va_bits;
	u64 page_offset;
	u64 phys_offset;
	u64 swapper_pg_dir;
	u64 kimage_voffset;

	/* For linux banner */
	u8 last_uts_release[__NEW_UTS_LEN];

	/* Info of running build */
	char build_info[32];
} __packed;

struct kernel_all_info {
	u32 magic_number;
	u32 combined_checksum;
	struct kernel_info info;
} __packed;

static void update_all_info_toio(void __iomem *io_base,
		struct kernel_all_info *all_info)
{
	int index;
	struct kernel_info *info;
	u32 *checksum_info;

	all_info->magic_number = BOOT_DEBUG_MAGIC;
	all_info->combined_checksum = 0;

	info = &(all_info->info);
	checksum_info = (u32 *)info;
	for (index = 0; index < sizeof(*info)/sizeof(u32); index++)
		all_info->combined_checksum ^= checksum_info[index];

	memcpy_toio(io_base, all_info, sizeof(*all_info));
}

static void backup_kernel_info(void)
{
	struct device_node *np;
	struct resource res;
	struct kernel_all_info all_info;
	struct kernel_info *info;
	void __iomem *imem_base;
	int num_reg = 0;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "msm-imem-kernel_info");
	if (!np) {
		pr_warn("%s: msm-imem-kernel_info node does not exist\n",
				__func__);
		return;
	}

	ret = of_address_to_resource(np, num_reg, &res);
	if(ret) {
		pr_warn("%s: invalid argument, ret %d\n", __func__, ret);
		return;
	}

	if ((!res.start) ||
		(resource_size(&res) < sizeof(struct kernel_all_info))) {
		pr_warn("%s: unexpected resource start %llx and size %llx\n",
				__func__, res.start, resource_size(&res));
		return;
	}

	all_info_addr = res.start;

	imem_base= ioremap(res.start, resource_size(&res));
	if (!imem_base) {
		pr_warn("%s: msm imem kernel info imem offset mapping failed\n",
				__func__);
		return;
	}

	memset(&all_info, 0, sizeof(all_info));
	memset_io(imem_base, 0, resource_size(&res));
	info = &(all_info.info);
	info->enabled_all = IS_ENABLED(CONFIG_KALLSYMS_ALL);
	info->enabled_base_relative = IS_ENABLED(CONFIG_KALLSYMS_BASE_RELATIVE);
	info->enabled_absolute_percpu =
		  IS_ENABLED(CONFIG_KALLSYMS_ABSOLUTE_PERCPU);
	info->enabled_cfi_clang = IS_ENABLED(CONFIG_CFI_CLANG);
	info->num_syms = kallsyms_num_syms;
	info->name_len = KSYM_NAME_LEN;
	info->bit_per_long = BITS_PER_LONG;
	info->module_name_len = MODULE_NAME_LEN;
	info->symbol_len = KSYM_SYMBOL_LEN;
	info->_addresses_va = (phys_addr_t)kallsyms_addresses;
	info->_relative_va = (phys_addr_t)kallsyms_relative_base;
	info->_stext_va = (phys_addr_t)_stext;
	info->_etext_va = (phys_addr_t)_etext;
	info->_sinittext_va = (phys_addr_t)_sinittext;
	info->_einittext_va = (phys_addr_t)_einittext;
	info->_end_va = (phys_addr_t)_end;
	info->_offsets_va = (phys_addr_t)kallsyms_offsets;
	info->_names_va = (phys_addr_t)kallsyms_names;
	info->_token_table_va = (phys_addr_t)kallsyms_token_table;
	info->_token_index_va = (phys_addr_t)kallsyms_token_index;
	info->_markers_va = (phys_addr_t)kallsyms_markers;
	info->thread_size = THREAD_SIZE;
	info->va_bits = VA_BITS;
	info->page_offset = PAGE_OFFSET;
	info->phys_offset = PHYS_OFFSET;
	info->swapper_pg_dir = (u64)swapper_pg_dir;
	info->kimage_voffset = kimage_voffset;
	strlcpy(info->last_uts_release, init_utsname()->release,
			sizeof(info->last_uts_release));

	update_all_info_toio(imem_base, &all_info);
	iounmap(imem_base);
}

static int build_info_set(const char *str, const struct kernel_param *kp)
{
	void __iomem *imem_base;
	struct kernel_all_info all_info;
	const size_t build_info_size = sizeof(all_info.info.build_info);

	if (all_info_addr == 0)
		return -EPERM;

	imem_base = ioremap(all_info_addr, sizeof(all_info));
	if (!imem_base) {
		pr_err("%s: Failed to map all_info\n", __func__);
		return -EPERM;
	}

	memcpy_fromio(&all_info, imem_base, sizeof(all_info));
	memcpy(&all_info.info.build_info, str,
			min(build_info_size, strlen(str)));
	update_all_info_toio(imem_base, &all_info);
	iounmap(imem_base);

	if (strlen(str) > build_info_size) {
		pr_warn("%s: Build info buffer (len: %zd) can't hold entire "
				"string '%s'\n",
				__func__, build_info_size, str);
		return -ENOMEM;
	}

	return 0;
}

static const struct kernel_param_ops build_info_op = {
	.set = build_info_set,
};

module_param_cb(build_info, &build_info_op, NULL, 0200);
MODULE_PARM_DESC(build_info,
		"Write build info to field 'build_info' of kdebuginfo.");

static int __init kdebuginfo_init(void)
{
	/* Backup kernel information for bootloader */
	backup_kernel_info();

	return 0;
}
device_initcall(kdebuginfo_init);
