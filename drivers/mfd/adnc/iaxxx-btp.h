/*
 * iaxxx-btp.h -- IAxxx BTP read/write for memory read/write
 *
 * Copyright 2016 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#ifndef __IAXXX_BTP_H__
#define __IAXXX_BTP_H__
int iaxxx_btp_write(struct iaxxx_priv *priv, uint32_t phy_addr,
		const void *pdata, uint32_t words, int host_id);
int iaxxx_btp_read(struct iaxxx_priv *priv, uint32_t phy_addr,
		void *pdata, uint32_t words, int host_id);
#endif /* __IAXXX_BTP_H__ */
