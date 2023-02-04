/*
 * Airbrush PCIe Notifier Interface
 * Note: this is not to be confused with intnc_notifier.
 *
 * Copyright (C) 2018 Google, Inc.
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

#ifndef _ABC_PCIE_NOTIFIER_H
#define _ABC_PCIE_NOTIFIER_H

#include <linux/bitops.h>
#include <linux/notifier.h>

/**
 * Notifier types for Airbrush PCIe blocking events.
 * These types can be OR'ed together when being notified.
 *
 * PCIE_LINK_POST_ENABLE - called when PCIe link is enabled; also
 *     called after PCIe is unmapped from EL2 and back to EL1 control.
 *     This indicates that PCIe is now available.
 *     This also indicates a PCIE_LINK_ERROR condition is naturally
 *     cleared, if it occurred.
 *
 * PCIE_LINK_PRE_DISABLE - called when PCIe link will disable; also
 *     called before PCIe will be unmapped from EL1 and entering EL2.
 *     This indicates that PCIe is still available, but will become
 *     unavailable.
 *
 * PCIE_LINK_ENTER_EL2 - called before PCIe will be unmapped from EL1
 *     and entering EL2.
 *
 * PCIE_LINK_EXIT_EL2 - called after PCIe is unmapped from EL2 and
 *     back to EL1 control.
 *
 * PCIE_LINK_ERROR - called when an unexpected link error occurs.
 *     This indicates that PCIe is currently _unavailable_.
 *     Upon receiving this event, subscribers should reset their
 *     software states as needed, but should not access registers over
 *     PCIe.
 *     Examples of situations when this will be broadcast are PCIe link
 *     down or regulator fatal errors.
 */
#define ABC_PCIE_LINK_POST_ENABLE	BIT(0)
#define ABC_PCIE_LINK_PRE_DISABLE	BIT(1)
#define ABC_PCIE_LINK_ENTER_EL2		BIT(2)
#define ABC_PCIE_LINK_EXIT_EL2		BIT(3)
#define ABC_PCIE_LINK_ERROR			BIT(4)

int abc_register_pcie_link_blocking_event(struct notifier_block *nb);
int abc_unregister_pcie_link_blocking_event(struct notifier_block *nb);

/* Provider functions */
int abc_pcie_link_notify_blocking(unsigned long event);

#endif /* _ABC_PCIE_NOTIFIER_H */
