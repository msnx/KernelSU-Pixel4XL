/*
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *
 * Author: Shradha Todi <shradha.t@samsung.com>
 *
 * AB chip information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/mfd/abc-pcie.h>

#define LOTID_LBIT		0
#define LOTID_MASK		0x1FFFFF
#define LOTID_NO_OF_BITS	21

#define WAFER_LBIT		21
#define WAFER_MASK		0x1F
#define WAFER_NO_OF_BITS	5

#define XCOORD_LBIT		26
#define XCOORD_MASK		0xFF
#define XCOORD_NO_OF_BITS	8

#define YCOORD_LBIT		2
#define YCOORD_MASK		0xFF
#define YCOORD_NO_OF_BITS	8

#define PRO_LINE_LBIT		2
#define PRO_LINE_MASK		0x3

#define SUB_REV_LBIT		16
#define SUB_REV_MASK		0xF

#define MAIN_REV_LBIT		20
#define MAIN_REV_MASK		0xF

#define PKG_REV_LBIT		24
#define PKG_REV_MASK		0xFF

static char table[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A',
	'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
	'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};

static u32 reverse_bits(u32 n, u32 no_of_bits)
{
	u32 rev = 0;
	int i;

	for (i = 0; i < no_of_bits; i++) {
		rev <<= 1;
		if (n & 1)
			rev ^= 1;
		n >>= 1;
	}
	return rev;
}

static void to_base_36(u32 lotid, char *ans)
{
	u32 n = lotid;
	u32 i = 4;

	while (n > 0 && i >= 0) {
		ans[i--] = table[n % 36];
		n = n / 36;
	}
	ans[5] = '\0';
}

static int chip_info_show(struct seq_file *m, void *v)
{
	u32 chip0, chip1, chip2, chip3;
	u32 lotid, pro_line;
	char lot[6];
	int ret;


	ret = ABC_READ(0x10BB0004, &chip0);
	if (ret) {
		seq_puts(m, "AB Unavailable (powered down?)\n");
		return 0;
	}
	ABC_READ(0x10BB0008, &chip1);
	ABC_READ(0x10BB000C, &chip2);
	ABC_READ(0x10BB0010, &chip3);

	lotid = reverse_bits(((chip0 >> LOTID_LBIT) & LOTID_MASK),
				LOTID_NO_OF_BITS);
	pro_line = ((chip1 >> PRO_LINE_LBIT) & PRO_LINE_MASK);
	if (pro_line == 0x1)
		lot[0] = 'N';
	else if (pro_line == 0x2)
		lot[0] = 'S';
	to_base_36(lotid, lot);

	seq_puts(m, "==================================================\n");
	seq_printf(m, "ECID[127:96][95:64][63:32][31:00] = %08X_%08X_%08X_%08X\n",
			chip3, chip2, chip1, chip0);
	seq_puts(m, "--------------------------------------------------\n");
	seq_printf(m, "  Wafer Number: %d\n", reverse_bits(((chip0 >>
			WAFER_LBIT) & WAFER_MASK), WAFER_NO_OF_BITS));
	seq_printf(m, "  X Position: %d\n", reverse_bits((((chip0 >>
			XCOORD_LBIT) | (chip1 << 6)) &
			XCOORD_MASK), XCOORD_NO_OF_BITS));
	seq_printf(m, "  Y Position: %d\n", reverse_bits(((chip1 >>
			YCOORD_LBIT) & YCOORD_MASK), YCOORD_NO_OF_BITS));
	seq_printf(m, "  Main Revision: %d\n", ((chip3 >> MAIN_REV_LBIT) &
			MAIN_REV_MASK));
	seq_printf(m, "  Sub Revision: %d\n", ((chip3 >> SUB_REV_LBIT) &
			SUB_REV_MASK));
	seq_printf(m, "  PKG Revision: %d\n", ((chip3 >> PKG_REV_LBIT) &
			PKG_REV_MASK));
	seq_printf(m, "  Lot ID: %s\n", lot);
	seq_printf(m, "  IPU RO: %d\n", get_ipu_ro());
	seq_printf(m, "  TPU RO: %d\n", get_tpu_ro());
	seq_puts(m, "==================================================\n");

	return 0;
}

int chip_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, chip_info_show, NULL);
}

