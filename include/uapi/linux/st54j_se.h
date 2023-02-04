/* SPDX-License-Identifier: GPL-2.0 */
/* include/uapi/linux/st54j_se.h
 * Copyright (C) 2018 ST Microelectronics S.A.
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _UAPI_ST54J_SE_H
#define _UAPI_ST54J_SE_H

#define ST54J_SE_MAGIC	0xE5
/*
 * ST54J_SE control via ioctl
  */
#define ST54J_SE_RESET            _IOR(ST54J_SE_MAGIC, 0x01, unsigned int)

#endif
