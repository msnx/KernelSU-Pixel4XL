/*
 * FaceAuth coordinator driver
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

#ifndef _FACEAUTH_HYPX_H
#define _FACEAUTH_HYPX_H

#include <linux/faceauth.h>
#include <linux/faceauth_shared.h>
#include <linux/device.h>

int el2_faceauth_probe(struct device *dev);
int el2_faceauth_remove(struct device *dev);
int el2_faceauth_wait_pil_dma_over(void);
int el2_faceauth_init(struct device *dev, struct faceauth_init_data *data,
		      uint64_t verbosity_level);
int el2_faceauth_process(struct device *dev, struct faceauth_start_data *data,
			 bool is_secure_camera);
int el2_faceauth_get_process_result(struct device *dev,
				    struct faceauth_start_data *data);
int el2_faceauth_cleanup(struct device *dev);
int el2_faceauth_gather_debug_log(struct device *dev,
				  struct faceauth_debug_data *data);
int el2_gather_debug_data(struct device *dev, void *destination_buffer,
			  uint32_t buffer_size);

#endif /* _FACEAUTH_HYPX_H */
