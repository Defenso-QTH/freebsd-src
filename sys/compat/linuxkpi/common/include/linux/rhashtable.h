/*-
 * Copyright (c) 2023 Bjoern A. Zeeb
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef	_LINUXKPI_LINUX_RHASHTABLE_H
#define	_LINUXKPI_LINUX_RHASHTABLE_H

#include <linux/kernel.h>	/* pr_debug */
#include <linux/err.h>
#include <linux/jhash.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

/*
 * A fixed size chained hash table with the rhashtable API subset that
 * ath11k uses.  Buckets are allocated once in rhashtable_init(), which may
 * sleep; insert, lookup and remove never allocate and only take the
 * table's own spinlock, so they can be called with other spinlocks held.
 * The table does not grow: it is sized from nelem_hint, and chains just
 * get longer past that.  Keys are compared with memcmp() over key_len;
 * the Linux hashfn/obj_hashfn/obj_cmpfn hooks are not supported.
 */

struct rhash_head {
	struct rhash_head	*next;
};

struct rhashtable_params {
	uint16_t	head_offset;
	uint16_t	key_len;
	uint16_t	key_offset;
	uint16_t	nelem_hint;
	bool		automatic_shrinking;
};

struct rhashtable {
	struct rhash_head	**buckets;
	unsigned int		mask;
	unsigned int		nelems;
	struct rhashtable_params p;
	spinlock_t		lock;
};

#define	LKPI_RHT_MIN_BUCKETS	16

static inline void *
lkpi_rht_obj(const struct rhashtable *ht, const struct rhash_head *he)
{
	return ((char *)__DECONST(struct rhash_head *, he) - ht->p.head_offset);
}

static inline const void *
lkpi_rht_key(const struct rhashtable *ht, const struct rhash_head *he)
{
	return ((const char *)lkpi_rht_obj(ht, he) + ht->p.key_offset);
}

static inline unsigned int
lkpi_rht_bucket(const struct rhashtable *ht, const void *key)
{
	return (jhash(key, ht->p.key_len, 0) & ht->mask);
}

/* Find the entry with this key; called with ht->lock held. */
static inline struct rhash_head *
lkpi_rht_find(const struct rhashtable *ht, const void *key)
{
	struct rhash_head *he;

	for (he = ht->buckets[lkpi_rht_bucket(ht, key)]; he != NULL;
	    he = he->next)
		if (memcmp(lkpi_rht_key(ht, he), key, ht->p.key_len) == 0)
			return (he);
	return (NULL);
}

static inline int
rhashtable_init(struct rhashtable *ht,
    const struct rhashtable_params *params)
{
	unsigned int n, want;

	if (params->key_len == 0)
		return (-EINVAL);
	/* Keep the load factor at or below 3/4 for the hinted size. */
	want = params->nelem_hint + params->nelem_hint / 3;
	for (n = LKPI_RHT_MIN_BUCKETS; n < want && n < (1u << 16); n <<= 1)
		;
	ht->buckets = kcalloc(n, sizeof(*ht->buckets), GFP_KERNEL);
	if (ht->buckets == NULL)
		return (-ENOMEM);
	ht->mask = n - 1;
	ht->nelems = 0;
	ht->p = *params;
	spin_lock_init(&ht->lock);
	return (0);
}

static inline void
rhashtable_destroy(struct rhashtable *ht)
{
	/* Like Linux, the objects still linked in belong to the caller. */
	kfree(ht->buckets);
	ht->buckets = NULL;
	ht->nelems = 0;
	spin_lock_destroy(&ht->lock);
}

static inline void *
rhashtable_lookup_fast(struct rhashtable *ht, const void *key,
    const struct rhashtable_params params __unused)
{
	struct rhash_head *he;
	void *obj;

	spin_lock(&ht->lock);
	he = lkpi_rht_find(ht, key);
	obj = (he != NULL) ? lkpi_rht_obj(ht, he) : NULL;
	spin_unlock(&ht->lock);
	return (obj);
}

/*
 * Insert obj unless an entry with the same key exists: return NULL when
 * inserted, the existing object otherwise.
 */
static inline void *
rhashtable_lookup_get_insert_fast(struct rhashtable *ht,
    struct rhash_head *obj, const struct rhashtable_params params __unused)
{
	struct rhash_head *he;
	const void *key;
	unsigned int b;
	void *ret;

	key = lkpi_rht_key(ht, obj);
	spin_lock(&ht->lock);
	he = lkpi_rht_find(ht, key);
	if (he != NULL) {
		ret = lkpi_rht_obj(ht, he);
	} else {
		b = lkpi_rht_bucket(ht, key);
		obj->next = ht->buckets[b];
		ht->buckets[b] = obj;
		ht->nelems++;
		ret = NULL;
	}
	spin_unlock(&ht->lock);
	return (ret);
}

/* Unlink this very object (not just one with its key). */
static inline int
rhashtable_remove_fast(struct rhashtable *ht,
    struct rhash_head *obj, const struct rhashtable_params params __unused)
{
	struct rhash_head **pp;
	int error;

	error = -ENOENT;
	spin_lock(&ht->lock);
	for (pp = &ht->buckets[lkpi_rht_bucket(ht, lkpi_rht_key(ht, obj))];
	    *pp != NULL; pp = &(*pp)->next) {
		if (*pp == obj) {
			*pp = obj->next;
			obj->next = NULL;
			ht->nelems--;
			error = 0;
			break;
		}
	}
	spin_unlock(&ht->lock);
	return (error);
}

#endif	/* _LINUXKPI_LINUX_RHASHTABLE_H */
