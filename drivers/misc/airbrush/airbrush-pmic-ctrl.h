/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * PMIC rails controller for airbrush state manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#ifndef _AIRBRUSH_PMIC_CTRL_H_
#define _AIRBRUSH_PMIC_CTRL_H_

#define SMPS2_DEFAULT_DELAY	0
#define LDO4_DEFAULT_DELAY	8000
#define LDO5_DEFAULT_DELAY	0
#define S60_DEFAULT_DELAY	0

int ab_mark_pmic_rail(struct ab_state_context *sc,
			   enum block_name blk_name,
			   bool enable,
			   enum block_state to_block_substate_id);

int ab_pmic_on(struct ab_state_context *ab_ctx);
int ab_pmic_off(struct ab_state_context *ab_ctx);
int ab_get_pmic_resources(struct ab_state_context *ab_ctx);

#endif // _AIRBRUSH_PMIC_CTRL_H_
