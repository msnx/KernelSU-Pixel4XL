/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ST21NFC_H
#define __ST21NFC_H

#define ST21NFC_MAGIC	0xEA

#define ST21NFC_NAME "st21nfc"
/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP	      _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_PULSE_RESET		_IOR(ST21NFC_MAGIC, 0x02, unsigned int)
#define ST21NFC_SET_POLARITY_RISING   _IOR(ST21NFC_MAGIC, 0x03, unsigned int)
#define ST21NFC_SET_POLARITY_HIGH     _IOR(ST21NFC_MAGIC, 0x05, unsigned int)
#define ST21NFC_GET_POLARITY	      _IOR(ST21NFC_MAGIC, 0x07, unsigned int)
#define ST21NFC_RECOVERY              _IOR(ST21NFC_MAGIC, 0x08, unsigned int)

#endif
