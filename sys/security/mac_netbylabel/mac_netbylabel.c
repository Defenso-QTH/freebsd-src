/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Quentin Thebault
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 */

/*
 * MAC/netbylabel - restrict IP network access based on the veriexec label
 * of the binary a process is running.  Sibling of MAC/grantbylabel: veriexec
 * establishes the verified identity + label; this policy acts on it.
 *
 * A verified binary's veriexec label may carry "net/" tokens:
 *   net/allow  - permitted to use the IP network
 *   net/deny   - denied the IP network
 *
 * security.mac.netbylabel.enabled       master switch (default off)
 * security.mac.netbylabel.default_deny  0: block only net/deny (deny-list)
 *                                        1: block unless net/allow (allow-list)
 * security.mac.netbylabel.block_loopback 0: loopback exempt (off-host only)
 *                                         1: policy applies to loopback too
 *
 * Enforced at connect(2) and bind(2) for AF_INET/AF_INET6.
 */

#include <sys/cdefs.h>

#include "opt_mac.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/vnode.h>
#include <sys/mac.h>
#include <sys/sysctl.h>
#include <sys/socket.h>
#include <sys/systm.h>

#include <netinet/in.h>

#include <security/mac/mac_policy.h>
#include <security/mac_veriexec/mac_veriexec.h>

#define	MAC_NETBYLABEL_FULLNAME		"MAC/netbylabel"

/* net-policy tokens carried in the veriexec label, prefixed "net/" */
#define	NBL_PREFIX	"net/"
#define	NBL_ALLOW	(1U << 0)
#define	NBL_DENY	(1U << 1)
#define	NBL_EMPTY	(1U << 31)	/* computed, no net/ tokens present */

typedef uint32_t nbl_label_t;

static int mac_netbylabel_slot;
#define	SLOT(l)		((nbl_label_t)mac_label_get((l), mac_netbylabel_slot))
#define	SLOT_SET(l, v)	mac_label_set((l), mac_netbylabel_slot, (intptr_t)(v))

SYSCTL_NODE(_security_mac, OID_AUTO, netbylabel,
    CTLFLAG_RW | CTLFLAG_MPSAFE, 0, "MAC/netbylabel policy controls");

static int mac_netbylabel_enabled = 0;
SYSCTL_INT(_security_mac_netbylabel, OID_AUTO, enabled, CTLFLAG_RWTUN,
    &mac_netbylabel_enabled, 0, "Enforce the netbylabel policy");

static int mac_netbylabel_default_deny = 0;
SYSCTL_INT(_security_mac_netbylabel, OID_AUTO, default_deny, CTLFLAG_RWTUN,
    &mac_netbylabel_default_deny, 0,
    "Deny IP network unless the binary is labeled net/allow");

static int mac_netbylabel_block_loopback = 0;
SYSCTL_INT(_security_mac_netbylabel, OID_AUTO, block_loopback, CTLFLAG_RWTUN,
    &mac_netbylabel_block_loopback, 0,
    "Apply the policy to loopback addresses too");

/*
 * Parse a veriexec label string for "net/" tokens.  A token is recognised
 * only at the start of the label or immediately after a comma, matching the
 * convention used by mac_grantbylabel.
 */
static nbl_label_t
nbl_parse_label(const char *label)
{
	nbl_label_t nbl;
	const char *cp;

	if (label == NULL || *label == '\0')
		return (NBL_EMPTY);
	nbl = 0;
	for (cp = strstr(label, NBL_PREFIX); cp != NULL;
	    cp = strstr(cp, NBL_PREFIX)) {
		if (cp > label && cp[-1] != ',') {
			cp += sizeof(NBL_PREFIX) - 1;
			continue;
		}
		cp += sizeof(NBL_PREFIX) - 1;
		if (strncmp(cp, "allow", 5) == 0)
			nbl |= NBL_ALLOW;
		else if (strncmp(cp, "deny", 4) == 0)
			nbl |= NBL_DENY;
	}
	if (nbl == 0)
		nbl = NBL_EMPTY;
	return (nbl);
}

/*
 * Fetch and cache the net label of a vnode from veriexec.  Called at exec,
 * where the vnode is locked, so VOP_GETATTR is safe here.
 */
static nbl_label_t
nbl_get_vlabel(struct vnode *vp, struct ucred *cred)
{
	struct vattr va;
	const char *label;
	nbl_label_t nbl;
	int error;

	nbl = SLOT(vp->v_label);
	if (nbl != 0)
		return (nbl);
	error = VOP_GETATTR(vp, &va, cred);
	if (error != 0)
		return (NBL_EMPTY);
	label = mac_veriexec_metadata_get_file_label(va.va_fsid, va.va_fileid,
	    va.va_gen, 0);
	nbl = nbl_parse_label(label);
	SLOT_SET(vp->v_label, nbl);
	return (nbl);
}

/*
 * The net label of the current process is the cached label of its text vnode.
 * If it was never computed (process exec'd before this module loaded) we treat
 * it as having no tokens.
 */
static nbl_label_t
nbl_proc_label(void)
{
	struct proc *p = curproc;
	nbl_label_t nbl;

	if (p == NULL || (p->p_flag & (P_KPROC | P_SYSTEM)) != 0)
		return (NBL_EMPTY);
	if (p->p_textvp == NULL)
		return (NBL_EMPTY);
	nbl = SLOT(p->p_textvp->v_label);
	if (nbl == 0)
		nbl = NBL_EMPTY;
	return (nbl);
}

static bool
nbl_is_loopback(const struct sockaddr *sa)
{
	const struct sockaddr_in *sin;
	const struct sockaddr_in6 *sin6;

	switch (sa->sa_family) {
	case AF_INET:
		/* 127.0.0.0/8; avoid IN_LOOPBACK() (references in_loopback_mask) */
		sin = (const struct sockaddr_in *)sa;
		return ((ntohl(sin->sin_addr.s_addr) & 0xff000000) == 0x7f000000);
	case AF_INET6:
		sin6 = (const struct sockaddr_in6 *)sa;
		return (IN6_IS_ADDR_LOOPBACK(&sin6->sin6_addr));
	default:
		return (false);
	}
}

/*
 * Shared decision for connect and bind.  Returns 0 to allow, EACCES to deny.
 */
static int
nbl_check(const struct sockaddr *sa)
{
	nbl_label_t nbl;

	if (mac_netbylabel_enabled == 0)
		return (0);
	if (sa == NULL)
		return (0);
	if (sa->sa_family != AF_INET && sa->sa_family != AF_INET6)
		return (0);
	if (nbl_is_loopback(sa) && mac_netbylabel_block_loopback == 0)
		return (0);
	nbl = nbl_proc_label();
	if (mac_netbylabel_default_deny != 0)
		return ((nbl & NBL_ALLOW) != 0 ? 0 : EACCES);
	return ((nbl & NBL_DENY) != 0 ? EACCES : 0);
}

static int
mac_netbylabel_socket_check_connect(struct ucred *cred __unused,
    struct socket *so __unused, struct label *solabel __unused,
    struct sockaddr *sa)
{
	return (nbl_check(sa));
}

static int
mac_netbylabel_socket_check_bind(struct ucred *cred __unused,
    struct socket *so __unused, struct label *solabel __unused,
    struct sockaddr *sa)
{
	return (nbl_check(sa));
}

static int
mac_netbylabel_vnode_check_exec(struct ucred *cred, struct vnode *vp,
    struct label *label __unused, struct image_params *imgp __unused,
    struct label *execlabel __unused)
{
	if (SLOT(vp->v_label) == 0)
		(void)nbl_get_vlabel(vp, cred);
	return (0);
}

static void
mac_netbylabel_vnode_init_label(struct label *label)
{
	SLOT_SET(label, 0);
}

static void
mac_netbylabel_vnode_copy_label(struct label *src, struct label *dest)
{
	SLOT_SET(dest, SLOT(src));
}

static struct mac_policy_ops mac_netbylabel_ops =
{
	.mpo_socket_check_connect = mac_netbylabel_socket_check_connect,
	.mpo_socket_check_bind = mac_netbylabel_socket_check_bind,
	.mpo_vnode_check_exec = mac_netbylabel_vnode_check_exec,
	.mpo_vnode_init_label = mac_netbylabel_vnode_init_label,
	.mpo_vnode_copy_label = mac_netbylabel_vnode_copy_label,
};

MAC_POLICY_SET(&mac_netbylabel_ops, mac_netbylabel, MAC_NETBYLABEL_FULLNAME,
    MPC_LOADTIME_FLAG_UNLOADOK, &mac_netbylabel_slot);
MODULE_VERSION(mac_netbylabel, 1);
MODULE_DEPEND(mac_netbylabel, mac_veriexec, MAC_VERIEXEC_VERSION,
    MAC_VERIEXEC_VERSION, MAC_VERIEXEC_VERSION);
