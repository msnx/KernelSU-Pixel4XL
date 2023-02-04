/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __DT_POWER_DELIVERY_H
#define __DT_POWER_DELIVERY_H

/* Power delivery Power Data Object definitions */
#define PDO_TYPE_FIXED		0
#define PDO_TYPE_BATT		1
#define PDO_TYPE_VAR		2
#define PDO_TYPE_APDO		3

#define PDO_TYPE_SHIFT		30
#define PDO_TYPE_MASK		0x3

#define PDO_TYPE(t)	((t) << PDO_TYPE_SHIFT)

#define PDO_VOLT_MASK		0x3ff
#define PDO_CURR_MASK		0x3ff
#define PDO_PWR_MASK		0x3ff

#define PDO_FIXED_DUAL_ROLE	(1 << 29) /* Power role swap supported */
#define PDO_FIXED_SUSPEND	(1 << 28) /* USB Suspend supported (Source) */
#define PDO_FIXED_HIGHER_CAP	(1 << 28) /* Requires more than vSafe5V (Sink) */
#define PDO_FIXED_EXTPOWER	(1 << 27) /* Externally powered */
#define PDO_FIXED_USB_COMM	(1 << 26) /* USB communications capable */
#define PDO_FIXED_DATA_SWAP	(1 << 25) /* Data role swap supported */
#define PDO_FIXED_VOLT_SHIFT	10	/* 50mV units */
#define PDO_FIXED_CURR_SHIFT	0	/* 10mA units */

#define PDO_FIXED_VOLT(mv)	((((mv) / 50) & PDO_VOLT_MASK) << PDO_FIXED_VOLT_SHIFT)
#define PDO_FIXED_CURR(ma)	((((ma) / 10) & PDO_CURR_MASK) << PDO_FIXED_CURR_SHIFT)

#define PDO_FIXED(mv, ma, flags)			\
	(PDO_TYPE(PDO_TYPE_FIXED) | (flags) |		\
	 PDO_FIXED_VOLT(mv) | PDO_FIXED_CURR(ma))

#define VSAFE5V 5000 /* mv units */

#define PDO_BATT_MAX_VOLT_SHIFT	20	/* 50mV units */
#define PDO_BATT_MIN_VOLT_SHIFT	10	/* 50mV units */
#define PDO_BATT_MAX_PWR_SHIFT	0	/* 250mW units */

#define PDO_BATT_MIN_VOLT(mv) ((((mv) / 50) & PDO_VOLT_MASK) << PDO_BATT_MIN_VOLT_SHIFT)
#define PDO_BATT_MAX_VOLT(mv) ((((mv) / 50) & PDO_VOLT_MASK) << PDO_BATT_MAX_VOLT_SHIFT)
#define PDO_BATT_MAX_POWER(mw) ((((mw) / 250) & PDO_PWR_MASK) << PDO_BATT_MAX_PWR_SHIFT)

#define PDO_BATT(min_mv, max_mv, max_mw)			\
	(PDO_TYPE(PDO_TYPE_BATT) | PDO_BATT_MIN_VOLT(min_mv) |	\
	 PDO_BATT_MAX_VOLT(max_mv) | PDO_BATT_MAX_POWER(max_mw))

#define PDO_VAR_MAX_VOLT_SHIFT	20	/* 50mV units */
#define PDO_VAR_MIN_VOLT_SHIFT	10	/* 50mV units */
#define PDO_VAR_MAX_CURR_SHIFT	0	/* 10mA units */

#define PDO_VAR_MIN_VOLT(mv) ((((mv) / 50) & PDO_VOLT_MASK) << PDO_VAR_MIN_VOLT_SHIFT)
#define PDO_VAR_MAX_VOLT(mv) ((((mv) / 50) & PDO_VOLT_MASK) << PDO_VAR_MAX_VOLT_SHIFT)
#define PDO_VAR_MAX_CURR(ma) ((((ma) / 10) & PDO_CURR_MASK) << PDO_VAR_MAX_CURR_SHIFT)

#define PDO_VAR(min_mv, max_mv, max_ma)				\
	(PDO_TYPE(PDO_TYPE_VAR) | PDO_VAR_MIN_VOLT(min_mv) |	\
	 PDO_VAR_MAX_VOLT(max_mv) | PDO_VAR_MAX_CURR(max_ma))

#define APDO_TYPE_PPS		0

#define PDO_APDO_TYPE_SHIFT	28 /* Only valid value currently is 0x0 - PPS */
#define PDO_APDO_TYPE_MASK	0x3

#define PDO_APDO_TYPE(t)	((t) << PDO_APDO_TYPE_SHIFT)

#define PDO_PPS_APDO_MAX_VOLT_SHIFT	17	/* 100mV units */
#define PDO_PPS_APDO_MIN_VOLT_SHIFT	8	/* 100mV units */
#define PDO_PPS_APDO_MAX_CURR_SHIFT	0	/* 50mA units */

#define PDO_PPS_APDO_VOLT_MASK	0xff
#define PDO_PPS_APDO_CURR_MASK	0x7f

#define PDO_PPS_APDO_MIN_VOLT(mv)	\
	((((mv) / 100) & PDO_PPS_APDO_VOLT_MASK) << PDO_PPS_APDO_MIN_VOLT_SHIFT)
#define PDO_PPS_APDO_MAX_VOLT(mv)	\
	((((mv) / 100) & PDO_PPS_APDO_VOLT_MASK) << PDO_PPS_APDO_MAX_VOLT_SHIFT)
#define PDO_PPS_APDO_MAX_CURR(ma)	\
	((((ma) / 50) & PDO_PPS_APDO_CURR_MASK) << PDO_PPS_APDO_MAX_CURR_SHIFT)

#define PDO_PPS_APDO(min_mv, max_mv, max_ma)				\
	(PDO_TYPE(PDO_TYPE_APDO) | PDO_APDO_TYPE(APDO_TYPE_PPS) |	\
	 PDO_PPS_APDO_MIN_VOLT(min_mv) | PDO_PPS_APDO_MAX_VOLT(max_mv) |\
	 PDO_PPS_APDO_MAX_CURR(max_ma))


/* VDM definitions: These should be kept in sync with pd_vdo.h */

/*
 * SVDM Identity Header
 * --------------------
 * <31>     :: data capable as a USB host
 * <30>     :: data capable as a USB device
 * <29:27>  :: product type (UFP or Cable Plug)
 * <26>     :: modal operation supported (1b == yes)
 * <25:23>  :: product type (DFP) (SVDM version 2.0 only; Shall be set to zero
 *             if SVDM version is 1.0)
 * <22:16>  :: Reserved, Shall be set to zero
 * <15:0>   :: USB-IF assigned VID for this cable vendor
 */
#define IDH_PTYPE_UNDEF		0
#define IDH_PTYPE_HUB		1
#define IDH_PTYPE_PERIPH	2
#define IDH_PTYPE_HOST		2
#define IDH_PTYPE_PSD		3
#define IDH_PTYPE_PCABLE	3
#define IDH_PTYPE_BRICK		3
#define IDH_PTYPE_ACABLE	4
#define IDH_PTYPE_AMC		4
#define IDH_PTYPE_AMA		5
#define IDH_PTYPE_VPD		6

#define IDH_USB_HOST		(1 << 31)
#define IDH_USB_DEVICE		(1 << 30)
#define IDH_PT_UFP_PLUG_SHIFT	27
#define IDH_PT_UFP_PLUG_MASK	(0x7 << IDH_PT_UFP_PLUG_SHIFT)
#define IDH_MODAL_SUPP		(1 << 26)
#define IDH_PT_DFP_SHIFT	23
#define IDH_PT_DFP_MASK		(0x7 << IDH_PT_DFP_SHIFT)

#define IDH_PT_UFP_PLUG(type)	(((type) << IDH_PT_UFP_PLUG_SHIFT)	\
				 & IDH_PT_UFP_PLUG_MASK)
#define IDH_PT_DFP(type)	(((type) << IDH_PT_DFP_SHIFT) & IDH_PT_DFP_MASK)

#define VDO_IDH(flags, pt_ufp_plug, pt_dfp, vid)               \
	((flags) | IDH_PT_UFP_PLUG(pt_ufp_plug)                 \
	 | IDH_PT_DFP(pt_dfp) | ((vid) & 0xffff))

/*
 * Cert Stat VDO
 * -----------
 * <31:0>  : USB-IF assigned XID for this cable
 */
#define VDO_CERT(xid)		((xid) & 0xffffffff)

/*
 * Product VDO
 * -----------
 * <31:16> : USB Product ID
 * <15:0>  : USB bcdDevice
 */
#define VDO_PRODUCT(pid, bcd)	(((pid) & 0xffff) << 16 | ((bcd) & 0xffff))

#endif /* __DT_POWER_DELIVERY_H */
