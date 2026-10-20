/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Quentin Thebault
 *
 * Test of the LinuxKPI rhashtable subset that ath11k uses: two tables
 * keyed like ath11k peers (an int id and a 6 byte MAC address), driven
 * with another spinlock held as ath11k does with base_lock.  Prints PASS
 * or FAIL on load.
 */
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <linux/kernel.h>
#include <linux/rhashtable.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

struct peer {
	int		 peer_id;
	struct rhash_head rhash_id;
	uint8_t		 addr[6];
	struct rhash_head rhash_addr;
};

#define	N	600
#define	CHECK(c) do { if (!(c)) { printf("rht_test: FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static int
rht_test_run(void)
{
	struct rhashtable_params pid = {
		.head_offset = offsetof(struct peer, rhash_id),
		.key_offset = offsetof(struct peer, peer_id),
		.key_len = sizeof(int), .automatic_shrinking = true,
		.nelem_hint = 512,
	};
	struct rhashtable_params paddr = {
		.head_offset = offsetof(struct peer, rhash_addr),
		.key_offset = offsetof(struct peer, addr),
		.key_len = 6, .automatic_shrinking = true, .nelem_hint = 512,
	};
	struct rhashtable *tid, *taddr;
	struct peer *p, dup;
	spinlock_t base_lock;
	int fails = 0, i, k;
	uint8_t a[6];

	p = kcalloc(N, sizeof(*p), GFP_KERNEL);
	tid = kzalloc(sizeof(*tid), GFP_KERNEL);
	taddr = kzalloc(sizeof(*taddr), GFP_KERNEL);
	spin_lock_init(&base_lock);
	CHECK(rhashtable_init(tid, &pid) == 0);
	CHECK(rhashtable_init(taddr, &paddr) == 0);

	spin_lock_bh(&base_lock);
	for (i = 0; i < N; i++) {
		p[i].peer_id = i * 7 + 1;
		p[i].addr[0] = 0x02; p[i].addr[4] = i >> 8; p[i].addr[5] = i;
		CHECK(rhashtable_lookup_get_insert_fast(tid, &p[i].rhash_id, pid) == NULL);
		CHECK(rhashtable_lookup_get_insert_fast(taddr, &p[i].rhash_addr, paddr) == NULL);
	}
	/* Duplicate keys return the existing object and do not insert. */
	dup = p[5];
	CHECK(rhashtable_lookup_get_insert_fast(tid, &dup.rhash_id, pid) == &p[5]);
	CHECK(rhashtable_lookup_get_insert_fast(taddr, &dup.rhash_addr, paddr) == &p[5]);
	CHECK(tid->nelems == N && taddr->nelems == N);
	for (i = 0; i < N; i++) {
		k = i * 7 + 1;
		CHECK(rhashtable_lookup_fast(tid, &k, pid) == &p[i]);
		CHECK(rhashtable_lookup_fast(taddr, p[i].addr, paddr) == &p[i]);
	}
	k = 2; CHECK(rhashtable_lookup_fast(tid, &k, pid) == NULL);
	memset(a, 0xff, 6); CHECK(rhashtable_lookup_fast(taddr, a, paddr) == NULL);
	/* Removing an object not in the table (same key) is -ENOENT. */
	CHECK(rhashtable_remove_fast(tid, &dup.rhash_id, pid) == -ENOENT);
	CHECK(rhashtable_lookup_fast(tid, &p[5].peer_id, pid) == &p[5]);
	for (i = 0; i < N; i += 2) {
		CHECK(rhashtable_remove_fast(tid, &p[i].rhash_id, pid) == 0);
		CHECK(rhashtable_remove_fast(taddr, &p[i].rhash_addr, paddr) == 0);
		CHECK(rhashtable_remove_fast(tid, &p[i].rhash_id, pid) == -ENOENT);
	}
	for (i = 0; i < N; i++) {
		CHECK(rhashtable_lookup_fast(tid, &p[i].peer_id, pid) == ((i & 1) ? &p[i] : NULL));
		CHECK(rhashtable_lookup_fast(taddr, p[i].addr, paddr) == ((i & 1) ? &p[i] : NULL));
	}
	CHECK(tid->nelems == N / 2 && taddr->nelems == N / 2);
	/* Re-insert after removal works. */
	CHECK(rhashtable_lookup_get_insert_fast(tid, &p[0].rhash_id, pid) == NULL);
	CHECK(rhashtable_lookup_fast(tid, &p[0].peer_id, pid) == &p[0]);
	spin_unlock_bh(&base_lock);

	printf("rht_test: buckets id %u addr %u\n", tid->mask + 1, taddr->mask + 1);
	rhashtable_destroy(tid);
	rhashtable_destroy(taddr);
	kfree(tid); kfree(taddr); kfree(p);
	spin_lock_destroy(&base_lock);
	printf("rht_test: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
	return (0);
}

static int
rht_test_modevent(module_t mod, int type, void *data)
{
	if (type == MOD_LOAD)
		return (rht_test_run());
	return (0);
}
static moduledata_t rht_test_mod = { "rht_test", rht_test_modevent, NULL };
DECLARE_MODULE(rht_test, rht_test_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(rht_test, linuxkpi, 1, 1, 1);
