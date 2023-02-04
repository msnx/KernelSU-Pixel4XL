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

#include "gvotable.h"

#include <linux/init.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stringhash.h>

static const char default_reason[] = "Default";

static DEFINE_MUTEX(gvotable_lock);
static LIST_HEAD(gvotables);

struct ballot {
	bool enabled;
	uint32_t reason_hash;
	char reason[MAX_REASON_LEN];

	uint32_t idx;
	void *vote[VOTES_HISTORY_DEPTH];

	uint32_t num_votes;

	struct list_head list;
};

/* This function compares l and r as integers */
int gvotable_comparator_max(void *l, void *r)
{
	if (l > r)
		return 1;
	else if (l < r)
		return (-1);
	else
		return 0;
}

/* This function compares l and r as integers */
int gvotable_comparator_min(void *a, void *b)
{
	return -gvotable_comparator_max(a, b);
}

int gvotable_comparator_any(void *a, void *b)
{
	return 1;
}

int gvotable_vote_to_str_uint(char *str, void *vote, int len)
{
	unsigned long val = (unsigned long) vote;

	return scnprintf(str, MAX_VOTE2STR_LEN, "%lu", val);
}

int gvotable_vote_to_str_uint_hex(char *str, void *vote, int len)
{
	unsigned long val = (unsigned long) vote;

	return scnprintf(str, MAX_VOTE2STR_LEN, "0x%lx", val);
}

/* GVotable internal hashing function */
static uint32_t gvotable_internal_hash(const char *str)
{
	return full_name_hash(NULL, str, strlen(str));
}

/* Internal function called when an election needs to run. This function
 * requires lock to be acquired because the callback needs to be called
 * without a locking on the election
 */
static void gvotable_internal_run_election(struct election *el)
{
	struct ballot *ballot;
	bool callback_required = false;
	void  *callback_result;
	char   callback_reason[MAX_REASON_LEN];

	mutex_lock(&el->lock);

	list_for_each_entry(ballot, &el->votes, list) {
		if (!ballot->enabled)
			continue;

		/* Update reason if needed */
		if (strncmp(el->reason, ballot->reason, MAX_REASON_LEN)) {
			strlcpy(el->reason, ballot->reason, MAX_REASON_LEN);
			callback_required = el->auto_callback;
		}

		/* Update result if needed */
		if (el->result != ballot->vote[ballot->idx]) {
			el->result = ballot->vote[ballot->idx];
			callback_required = el->auto_callback;
		}

		el->result_is_valid = true;
		goto process_callback;
	}

	/*  Could not find a vote. Do we have at least a default? */
	if (el->has_default_vote) {
		if (strncmp(el->reason, default_reason, MAX_REASON_LEN)) {
			strlcpy(el->reason, default_reason, MAX_REASON_LEN);
			callback_required = el->auto_callback;
		}

		if (el->result != el->default_vote) {
			el->result = el->default_vote;
			callback_required = el->auto_callback;
		}
		el->result_is_valid = true;
	} else {
		el->result_is_valid = false;
		callback_required = false;
	}

process_callback:
	/* Copy callback arguments from election */
	strlcpy(callback_reason, el->reason, MAX_REASON_LEN);
	callback_result = el->result;

	mutex_unlock(&el->lock);

	if (callback_required && el->callback)
		el->callback(callback_reason, callback_result, el->data);
}

static struct election_slot *gvotable_find_internal(const char *name)
{
	struct election_slot *slot;
	struct election *el;
	unsigned int hash;

	if (!name)
		return NULL;

	hash = gvotable_internal_hash(name);

	list_for_each_entry(slot, &gvotables, list) {
		el = slot->el;
		if ((hash == el->hash) && el->has_name &&
		    (strncmp(el->name, name, MAX_NAME_LEN) == 0))
			return slot;
	}

	return NULL;
}

static struct election_slot *gvotable_find_internal_ptr(struct election *el)
{
	struct election_slot *slot;

	list_for_each_entry(slot, &gvotables, list)
		if (slot->el == el)
			return slot;

	return NULL;
}

static void gvotable_add_internal(struct election_slot *slot)
{
	list_add(&slot->list, &gvotables);
}

static void gvotable_delete_internal(struct election_slot *slot)
{
	list_del(&slot->list);
	kfree(slot);
}

static struct ballot *gvotable_ballot_find_internal(struct election *el,
						    const char *reason)
{
	struct ballot *ballot;
	uint32_t reason_hash;

	reason_hash = gvotable_internal_hash(reason);

	list_for_each_entry(ballot, &el->votes, list) {
		if ((reason_hash == ballot->reason_hash) &&
		    (strncmp(ballot->reason, reason, MAX_REASON_LEN) == 0)) {
			return ballot;
		}
	}
	return NULL;
}

/* Creates an election */
struct election *gvotable_create_election(const char *name,
					  int  (*cmp_fn)(void *, void *),
					  void (*callback_fn)(const char*,
							      void *,
							      void *),
					  void *data)
{
	struct election_slot *slot, *tmp;

	if (!cmp_fn)
		return NULL;

	slot = kzalloc(sizeof(*slot), GFP_KERNEL);
	if (!slot)
		return NULL;

	slot->el = kzalloc(sizeof(*slot->el), GFP_KERNEL);
	if (!slot->el) {
		kfree(slot);
		return NULL;
	}

	mutex_init(&(slot->el->lock));
	INIT_LIST_HEAD(&slot->el->votes);
	slot->el->callback         = callback_fn;
	slot->el->cmp              = cmp_fn;
	slot->el->auto_callback    = true;
	slot->el->data             = data;

	if (name) {
		slot->el->has_name = true;
		slot->el->hash     = gvotable_internal_hash(name);
		strlcpy(slot->el->name, name, MAX_NAME_LEN);
	}

	mutex_lock(&gvotable_lock);

	if (name) {
		tmp = gvotable_find_internal(name);
		if (tmp) {
			kfree(slot->el);
			kfree(slot);
			return NULL;
		}
	}

	gvotable_add_internal(slot);
	mutex_unlock(&gvotable_lock);

	return slot->el;
}

/* Remove an election from votables */
int gvotable_destroy_election(struct election *el)
{
	/* Destroying an election involves removing all voters
	 * and removing the election (and all its links) from
	 * the election slot list
	 */
	struct ballot *tmp, *ballot;
	struct election_slot *slot;

	if (!el)
		return -EINVAL;

	mutex_lock(&el->lock);
	list_for_each_entry_safe(ballot, tmp, &el->votes, list) {
		kfree(ballot);
	}
	mutex_unlock(&el->lock);

	/* Find slots associated with this handle and remove them */
	mutex_lock(&gvotable_lock);
	slot = gvotable_find_internal_ptr(el);
	while (slot) {
		gvotable_delete_internal(slot);
		slot = gvotable_find_internal_ptr(el);
	}
	kfree(el);
	mutex_unlock(&gvotable_lock);

	return 0;
}

/* Returns a list of active elections */
int gvotable_list_elections(int (*vote2str)(char *, void *, int))
{
	void *vote;
	char reason[MAX_REASON_LEN];
	char str[MAX_VOTE2STR_LEN];
	const char *el_name;
	int rc;
	struct election_slot *slot;

	vote = NULL;

	mutex_lock(&gvotable_lock);

	if (list_empty(&gvotables)) {
		pr_info("No available elections.");
		goto unlock;
	}


	pr_info("Available Elections:");
	list_for_each_entry(slot, &gvotables, list) {
		struct election *el = slot->el;

		rc =  gvotable_get_current_vote(el, &vote);
		rc += gvotable_get_current_reason(el, reason, MAX_REASON_LEN);

		el_name = (el->has_name ? el->name : "<NULL>");

		if (!rc &&
		    (vote2str(str, vote, MAX_VOTE2STR_LEN) > 0))
			pr_info("\t [%s]:(vote %s, reason %s)", el_name,
								str,
								reason);
		else
			pr_info("\t [%s] ()", el_name);
	}

unlock:
	mutex_unlock(&gvotable_lock);
	return 0;
}

/* Get handle of a public election */
struct election *gvotable_election_get_handle(const char *name)
{
	struct election_slot *slot;

	if (!name)
		return NULL;

	mutex_lock(&gvotable_lock);
	/* Slot is set to NULL if not found */
	slot = gvotable_find_internal(name);
	mutex_unlock(&gvotable_lock);

	return slot->el;
}

/* Set name of an election (makes election available for lookup) */
int gvotable_election_set_name(struct election *el, const char *name)
{
	struct election_slot *slot;

	if (!el || !name)
		return -EINVAL;

	if (el->has_name)
		return -EEXIST;

	mutex_lock(&gvotable_lock);
	slot = gvotable_find_internal(name);

	if (slot) {
		mutex_unlock(&gvotable_lock);
		return -EEXIST;
	}

	el->has_name = true;
	el->hash     = gvotable_internal_hash(name);
	strlcpy(el->name, name, MAX_NAME_LEN);

	mutex_unlock(&gvotable_lock);
	return 0;
}

/* Allows to set a default value for a given election */
int gvotable_set_default(struct election *el, void *default_val)
{
	if (!el)
		return -EINVAL;

	mutex_lock(&el->lock);
	el->default_vote = default_val;
	mutex_unlock(&el->lock);
	return 0;
}

/* Enable or disable usage of a default value for a given election */
int gvotable_use_default(struct election *el, bool default_is_enabled)
{
	if (!el)
		return -EINVAL;

	mutex_lock(&el->lock);
	el->has_default_vote = default_is_enabled;
	mutex_unlock(&el->lock);

	gvotable_internal_run_election(el);
	return 0;
}

/* Retrieve current vote for an election */
int gvotable_get_current_vote(struct election *el, void **vote)
{
	if (!el || !vote)
		return -EINVAL;

	mutex_lock(&el->lock);

	if (!el->result_is_valid) {
		mutex_unlock(&el->lock);
		return -EAGAIN;
	}

	*vote = el->result;
	mutex_unlock(&el->lock);
	return 0;
}

/* Retrieve current data for an election */
int gvotable_get_current_data(struct election *el, void **data)
{
	if (!el || !data)
		return -EINVAL;

	mutex_lock(&el->lock);

	if (!el->result_is_valid) {
		mutex_unlock(&el->lock);
		return -EAGAIN;
	}

	*data = el->data;
	mutex_unlock(&el->lock);
	return 0;
}

/* Retrieve current reason for election result */
int gvotable_get_current_reason(struct election *el,
				char *reason,
				int max_reason_len)
{
	int len;

	if (!el || !reason)
		return -EINVAL;

	mutex_lock(&el->lock);
	if (!el->result_is_valid) {
		mutex_unlock(&el->lock);
		return -EAGAIN;
	}

	len = strlcpy(reason, el->reason, max_reason_len);
	mutex_unlock(&el->lock);
	return len;
}

/* Get vote associated with a specific reason */
int gvotable_get_vote(struct election *el, const char *reason, void **vote)
{
	struct ballot *ballot;

	if (!el || !reason || !vote)
		return -EINVAL;

	mutex_lock(&el->lock);
	ballot = gvotable_ballot_find_internal(el, reason);

	if (!ballot) {
		mutex_unlock(&el->lock);
		return -ENODEV;
	}

	*vote = ballot->vote[ballot->idx];
	mutex_unlock(&el->lock);
	return 0;
}

/* Cast a vote on an election */
int gvotable_cast_vote(struct election *el,
		       const char *reason,
		       void *vote,
		       bool enabled)
{
	struct ballot *ballot, *tmp;

	if (!el || !reason)
		return -EINVAL;

	/* Can't check if vote is null because 0 is a valid value
	 * if we overload with integers
	 */

	/* Do we have a vote for this reason? */
	if (!el || !reason)
		return -EINVAL;

	mutex_lock(&el->lock);
	ballot = gvotable_ballot_find_internal(el, reason);
	mutex_unlock(&el->lock);

	/* If exists, extract vote otherwise create */
	if (ballot)
		list_del(&ballot->list);
	else {
		ballot = kzalloc(sizeof(*ballot), GFP_KERNEL);

		if (!ballot)
			return -ENOMEM;

		ballot->reason_hash = gvotable_internal_hash(reason);
		strlcpy(ballot->reason, reason, MAX_REASON_LEN);
		el->num_voters++;
	}

	/* Cast vote */
	ballot->enabled = enabled;
	ballot->num_votes++;
	ballot->vote[ballot->idx] = vote;
	ballot->idx = (ballot->idx + 1) % VOTES_HISTORY_DEPTH;
	el->num_votes++;

	/* Position voter in place */
	if (list_empty(&el->votes))
		list_add(&ballot->list, &el->votes);
	else {
		struct ballot *prev_tmp;

		prev_tmp = list_first_entry(&el->votes, struct ballot, list);
		list_for_each_entry(tmp, &el->votes, list) {
			if (el->cmp(vote, tmp->vote[tmp->idx]) < 0) {
				list_add_tail(&ballot->list, &prev_tmp->list);
				goto run_elect;
			}
			prev_tmp = tmp;
		}
		list_add_tail(&ballot->list, &prev_tmp->list);
	}

run_elect:
	/* Current result might have changed */
	gvotable_internal_run_election(el);
	return 0;
}

/* List all votes associated with an election */
int gvotable_dump_votes(struct election *el,
			int (*vote2str)(char *, void *, int))
{
	struct ballot *ballot;
	char str[MAX_VOTE2STR_LEN];

	if (!el)
		return -EINVAL;

	mutex_lock(&el->lock);

	pr_info("%s: %d votes cast ", el->has_name ? el->name : "<NULL>",
				      el->num_votes);

	list_for_each_entry(ballot, &el->votes, list) {
		void *vote_tmp;

		vote_tmp = ballot->vote[ballot->idx];
		if (vote2str(str, vote_tmp, MAX_VOTE2STR_LEN) > 0)
			pr_info("\t%c %-16s %16s (%u votes)",
				ballot->enabled ? '*' : ' ',
				ballot->reason,
				str,
				ballot->num_votes);
	}

	if (el->has_default_vote &&
	    (vote2str(str, el->default_vote, MAX_VOTE2STR_LEN) > 0))
		pr_info("Default vote %s.", str);

	if (el->result_is_valid &&
	    (vote2str(str, el->result, MAX_VOTE2STR_LEN) > 0))
		pr_info("Result %s, Reason %s.", str, el->reason);
	else
		pr_info("No result");

	mutex_unlock(&el->lock);
	return 0;
}

static int __init gvotable_init(void)
{
	return 0;
}

static void __exit gvotable_exit(void)
{
	struct election_slot *slot, *tmp;

	list_for_each_entry_safe(slot, tmp, &gvotables, list) {
		pr_debug("Destroying %p\n", slot->el);
		gvotable_destroy_election(slot->el);
	}
	pr_info("Deinit completed\n\n");
}

module_init(gvotable_init);
module_exit(gvotable_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luigi Zevola <zevola@google.com>");
MODULE_DESCRIPTION("Election library for shared resources");
MODULE_VERSION("0.01");
