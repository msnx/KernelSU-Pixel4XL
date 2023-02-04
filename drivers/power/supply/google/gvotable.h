/*
 * Copyright 2019 Google, Inc
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
 */

#ifndef __GOOGLE_GVOTABLE_H_
#define __GOOGLE_GVOTABLE_H_

#include <linux/types.h>
#include <linux/mutex.h>

#define VOTES_HISTORY_DEPTH  1
#define MAX_NAME_LEN        16
#define MAX_REASON_LEN      16
#define MAX_VOTE2STR_LEN    16

struct ballot;

struct election {
	uint32_t hash;
	bool has_name;
	char name[MAX_NAME_LEN];
	struct mutex lock;

	bool  result_is_valid;
	void  *result;
	char  reason[MAX_REASON_LEN];
	void  *data;

	void  (*callback)(const char *reason, void *vote, void *data);
	int   (*cmp)(void *, void *);

	uint32_t  num_voters;
	uint32_t  num_votes;

	bool   auto_callback;
	bool   has_default_vote;
	void   *default_vote;

	struct list_head votes;
};

struct election_slot {
	struct election *el;
	struct list_head list;
};

struct election *gvotable_create_election(const char *name,
					  int  (*cmp_fn)(void *, void *),
					  void (*callback_fn)(const char *,
							      void *,
							      void *),
					  void *data);

int gvotable_destroy_election(struct election *el);

int gvotable_list_elections(int (*vote2str)(char *, void *, int));

struct election *gvotable_election_get_handle(const char *name);

int gvotable_get_current_vote(struct election *el, void **vote);
int gvotable_get_current_data(struct election *el, void **data);
int gvotable_get_current_reason(struct election *el,
				char *reason,
				int max_reason_len);

int gvotable_set_default(struct election *el, void *default_val);
int gvotable_election_set_name(struct election *el, const char *name);
int gvotable_use_default(struct election *el, bool default_is_enabled);

int gvotable_dump_votes(struct election *el,
			int (*vote2str)(char *, void *, int));

int gvotable_cast_vote(struct election *el,
		       const char *reason,
		       void *vote,
		       bool enabled);

int gvotable_get_vote(struct election *el,
		      const char *reason,
		      void **vote);

int gvotable_comparator_max(void *a, void *b);
int gvotable_comparator_min(void *a, void *b);
int gvotable_comparator_any(void *a, void *b);

#endif /* __GOOGLE_GVOTABLE_H_*/
