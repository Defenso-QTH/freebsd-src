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

/*
 * QRTR (Qualcomm IPC Router) over MHI, and the Linux QMI client interface
 * on top of it, for drivers such as ath11k(4) that talk to their firmware's
 * QMI services.
 *
 * Linux implements this with an address family, a name service daemon
 * living in the kernel and kernel sockets.  Here the pieces are folded
 * together, with the same externally visible behaviour:
 *
 *  - the "IPCR" MHI channels of a controller form an endpoint, the link to
 *    one remote node (the firmware), as net/qrtr/mhi.c does;
 *  - control packets addressed to the control port are handled as the
 *    Linux name service does (net/qrtr/ns.c): HELLO is answered, servers
 *    announced by the remote are recorded and reported to the QMI handles
 *    whose lookups match;
 *  - each QMI handle owns one local port; data packets to that port are
 *    dispatched as drivers/soc/qcom/qmi_interface.c does: responses
 *    complete their transaction, other messages go to the handle's
 *    message handlers.
 *
 * Everything received is processed in order on one workqueue, so handlers
 * run in a sleepable context as on Linux.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/qrtr.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/socket.h>
#include <linux/workqueue.h>

#include "linux_qrtr.h"

#define	QRTR_PROTO_VER_1	1
#define	QRTR_PROTO_VER_2	3
#define	QRTR_FLAGS_CONFIRM_RX	(1 << 0)
#define	QRTR_PORT_CTRL_LEGACY	0xffff
#define	QRTR_MIN_EPH_SOCKET	0x4000
#define	QRTR_MAX_EPH_SOCKET	0x7fff

struct qrtr_hdr_v1 {
	__le32			version;
	__le32			type;
	__le32			src_node_id;
	__le32			src_port_id;
	__le32			confirm_rx;
	__le32			size;
	__le32			dst_node_id;
	__le32			dst_port_id;
} __packed;

struct qrtr_hdr_v2 {
	uint8_t			version;
	uint8_t			type;
	uint8_t			flags;
	uint8_t			optlen;
	__le32			size;
	__le16			src_node_id;
	__le16			src_port_id;
	__le16			dst_node_id;
	__le16			dst_port_id;
} __packed;

/* Decoded packet header. */
struct qrtr_cb {
	unsigned int		type;
	unsigned int		src_node;
	unsigned int		src_port;
	unsigned int		dst_node;
	unsigned int		dst_port;
	bool			confirm_rx;
};

#define	QRTR_EP_NIDS		4

/* The link to one remote node over the IPCR channels of an MHI device. */
struct qrtr_ep {
	struct list_head	link;
	struct mhi_device	*mhi_dev;
	/* Node ids reached through this endpoint, the first one directly. */
	unsigned int		nids[QRTR_EP_NIDS];
	int			nnids;
	bool			dead;
};

/* A server announced by a remote node. */
struct qrtr_server {
	struct list_head	link;
	unsigned int		service;
	unsigned int		instance;
	unsigned int		node;
	unsigned int		port;
};

/* A local port, bound to a QMI handle. */
struct lkpi_qrtr_port {
	struct list_head	link;
	unsigned int		port;
	struct qmi_handle	*qmi;
	struct sockaddr_qrtr	peer;		/* sq_family 0: not connected */
};

/* What a lookup asked for, see qmi_add_lookup(). */
struct qrtr_lookup {
	struct list_head	link;
	unsigned int		service;
	unsigned int		instance;	/* version | instance << 8 */
};

/* A received packet, or a lookup to evaluate, queued for the worker. */
struct qrtr_rx {
	struct list_head	link;
	struct qrtr_ep		*ep;		/* NULL: lookup scan */
	struct qmi_handle	*qmi;		/* lookup scan */
	struct qrtr_lookup	*lookup;	/* lookup scan */
	size_t			len;
	uint8_t			data[];
};

static MALLOC_DEFINE(M_QRTR, "qrtr", "QRTR receive queue");

static unsigned int qrtr_local_nid = 1;

int lkpi_qrtr_debug = 0;
SYSCTL_DECL(_compat_linuxkpi);
SYSCTL_INT(_compat_linuxkpi, OID_AUTO, qrtr_debug, CTLFLAG_RWTUN,
    &lkpi_qrtr_debug, 0, "Print QRTR and QMI debug messages");

#define	qrtr_dbg(fmt, ...) do {						\
	if (lkpi_qrtr_debug)						\
		printf("qrtr: " fmt, ##__VA_ARGS__);			\
} while (0)
/* Rare events worth having in any bug report. */
#define	qrtr_info(fmt, ...)	printf("qrtr: " fmt, ##__VA_ARGS__)

/*
 * qrtr_ep_lock protects the endpoints and the nodes they reach; it is
 * taken shared to send.  qrtr_ns_lock protects the servers, the ports and
 * their lookups; the worker holds it while it calls into QMI handles, so a
 * handle cannot go away under a callback.  A handle's sock_lock protects
 * its port's peer.  Order: qrtr_ns_lock, sock_lock, qrtr_ep_lock.
 */
static struct sx qrtr_ep_lock;
static struct sx qrtr_ns_lock;
static LIST_HEAD(qrtr_eps);
static LIST_HEAD(qrtr_servers);
static LIST_HEAD(qrtr_ports);
static unsigned int qrtr_next_port = QRTR_MIN_EPH_SOCKET;

/* Receive queue, filled from MHI completion context. */
static struct mtx qrtr_rx_lock;
static LIST_HEAD(qrtr_rxq);
static struct workqueue_struct *qrtr_wq;
static struct work_struct qrtr_rx_work;

/* -------------------------------------------------------------------- */

static bool
qrtr_ep_has_nid(const struct qrtr_ep *ep, unsigned int nid)
{
	int i;

	for (i = 0; i < ep->nnids; i++) {
		if (ep->nids[i] == nid)
			return (true);
	}
	return (false);
}

/* Record that nid is reachable through ep, as qrtr_node_assign() does. */
static void
qrtr_ep_assign(struct qrtr_ep *ep, unsigned int nid)
{
	if (nid == QRTR_NODE_BCAST || qrtr_ep_has_nid(ep, nid))
		return;
	if (ep->nnids == QRTR_EP_NIDS)
		return;
	ep->nids[ep->nnids++] = nid;
	qrtr_info("endpoint %p reaches node %u\n", ep, nid);
}

static void
qrtr_ul_free(struct mhi_device *mhi_dev __unused, struct mhi_result *res)
{
	kfree(res->buf_addr);
}

/*
 * Send one packet.  Data comes from a QMI handle port, control packets from
 * the control port.  A packet to QRTR_NODE_BCAST goes to every endpoint.
 */
static int
qrtr_send(unsigned int type, unsigned int src_node, unsigned int src_port,
    unsigned int dst_node, unsigned int dst_port, const void *data,
    size_t len)
{
	struct qrtr_hdr_v1 *hdr;
	struct qrtr_ep *ep;
	size_t total;
	void *buf;
	int error, sent;

	total = sizeof(*hdr) + ALIGN(len, 4);
	error = -EHOSTUNREACH;
	sent = 0;

	sx_slock(&qrtr_ep_lock);
	list_for_each_entry(ep, &qrtr_eps, link) {
		if (ep->dead)
			continue;
		if (dst_node != QRTR_NODE_BCAST && !qrtr_ep_has_nid(ep, dst_node))
			continue;

		buf = kzalloc(total, GFP_KERNEL);
		if (buf == NULL) {
			error = -ENOMEM;
			break;
		}
		hdr = buf;
		hdr->version = cpu_to_le32(QRTR_PROTO_VER_1);
		hdr->type = cpu_to_le32(type);
		hdr->src_node_id = cpu_to_le32(src_node);
		hdr->src_port_id = cpu_to_le32(src_port);
		hdr->confirm_rx = 0;
		hdr->size = cpu_to_le32(len);
		hdr->dst_node_id = cpu_to_le32(dst_node);
		hdr->dst_port_id = cpu_to_le32(dst_port);
		memcpy((uint8_t *)buf + sizeof(*hdr), data, len);

		qrtr_dbg("tx type %u %u:%x -> %u:%x len %zu\n", type, src_node,
		    src_port, dst_node, dst_port, len);
		error = mhi_queue_buf(ep->mhi_dev, DMA_TO_DEVICE, buf, total,
		    MHI_EOT);
		if (error != 0) {
			kfree(buf);
			break;
		}
		sent++;
		if (dst_node != QRTR_NODE_BCAST)
			break;
	}
	sx_sunlock(&qrtr_ep_lock);

	if (sent > 0 && dst_node == QRTR_NODE_BCAST)
		return (0);
	return (error);
}

static int
qrtr_send_ctrl(unsigned int dst_node, const struct qrtr_ctrl_pkt *pkt)
{
	return (qrtr_send(le32_to_cpu(pkt->cmd), qrtr_local_nid,
	    QRTR_PORT_CTRL, dst_node, QRTR_PORT_CTRL, pkt, sizeof(*pkt)));
}

/* Acknowledge a packet received with confirm_rx set. */
static void
qrtr_send_resume_tx(const struct qrtr_cb *cb)
{
	struct qrtr_ctrl_pkt pkt;

	/* From us, also when the packet was broadcast. */
	memset(&pkt, 0, sizeof(pkt));
	pkt.cmd = cpu_to_le32(QRTR_TYPE_RESUME_TX);
	pkt.client.node = cpu_to_le32(qrtr_local_nid);
	pkt.client.port = cpu_to_le32(cb->dst_port);
	(void)qrtr_send(QRTR_TYPE_RESUME_TX, qrtr_local_nid, cb->dst_port,
	    cb->src_node, cb->src_port, &pkt, sizeof(pkt));
}

/* -------------------------------------------------------------------- */
/* QMI handles: lookups and message dispatch (qmi_interface.c). */

static bool
qrtr_lookup_match(const struct qrtr_lookup *lookup,
    const struct qrtr_server *srv)
{
	if (lookup->service != 0 && lookup->service != srv->service)
		return (false);
	if (lookup->instance != 0 && lookup->instance != srv->instance)
		return (false);
	return (true);
}

static void
qmi_recv_new_server(struct qmi_handle *qmi, const struct qrtr_server *srv)
{
	struct qmi_service *svc;
	int ret;

	if (qmi->ops.new_server == NULL)
		return;

	svc = kzalloc(sizeof(*svc), GFP_KERNEL);
	if (svc == NULL)
		return;
	svc->service = srv->service;
	svc->version = srv->instance & 0xff;
	svc->instance = srv->instance >> 8;
	svc->node = srv->node;
	svc->port = srv->port;

	ret = qmi->ops.new_server(qmi, svc);
	if (ret < 0)
		kfree(svc);
	else
		list_add(&svc->list_node, &qmi->lookup_results);
}

static void
qmi_recv_del_server(struct qmi_handle *qmi, unsigned int node,
    unsigned int port)
{
	struct qmi_service *svc, *tmp;

	list_for_each_entry_safe(svc, tmp, &qmi->lookup_results, list_node) {
		if (node != -1U && svc->node != node)
			continue;
		if (port != -1U && svc->port != port)
			continue;
		if (qmi->ops.del_server != NULL)
			qmi->ops.del_server(qmi, svc);
		list_del(&svc->list_node);
		kfree(svc);
	}
}

static void
qmi_invoke_handler(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
    struct qmi_txn *txn, const void *buf, size_t len)
{
	const struct qmi_msg_handler *handler;
	const struct qmi_header *hdr = buf;
	void *dest;
	int ret;

	if (qmi->handlers == NULL)
		return;

	for (handler = qmi->handlers; handler->fn != NULL; handler++) {
		if (handler->type == hdr->type &&
		    handler->msg_id == le16_to_cpu(hdr->msg_id))
			break;
	}
	if (handler->fn == NULL) {
		qrtr_info("no handler for QMI type %u msg 0x%x\n", hdr->type,
		    le16_to_cpu(hdr->msg_id));
		return;
	}

	dest = kzalloc(handler->decoded_size, GFP_KERNEL);
	if (dest == NULL)
		return;

	ret = qmi_decode_message(buf, len, handler->ei, dest);
	if (ret < 0)
		pr_err("qmi: failed to decode message 0x%x: %d\n",
		    le16_to_cpu(hdr->msg_id), ret);
	else
		handler->fn(qmi, sq, txn, dest);

	kfree(dest);
}

static void
qmi_handle_message(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
    const void *buf, size_t len)
{
	const struct qmi_header *hdr;
	struct qmi_txn tmp_txn;
	struct qmi_txn *txn;
	int ret;

	if (len < sizeof(*hdr)) {
		pr_err("qmi: ignoring short packet\n");
		return;
	}
	hdr = buf;
	qrtr_dbg("qmi rx type %u txn %u msg 0x%x len %u\n", hdr->type,
	    le16_to_cpu(hdr->txn_id), le16_to_cpu(hdr->msg_id),
	    le16_to_cpu(hdr->msg_len));

	if (hdr->type == QMI_RESPONSE) {
		mutex_lock(&qmi->txn_lock);
		txn = idr_find(&qmi->txns, le16_to_cpu(hdr->txn_id));
		if (txn == NULL) {
			/* Unexpected, or its waiter gave up. */
			mutex_unlock(&qmi->txn_lock);
			qrtr_info("qmi: response 0x%x for no transaction %u "
			    "(late, after a timeout?)\n",
			    le16_to_cpu(hdr->msg_id), le16_to_cpu(hdr->txn_id));
			return;
		}
		mutex_lock(&txn->lock);
		mutex_unlock(&qmi->txn_lock);

		if (txn->dest != NULL && txn->ei != NULL) {
			ret = qmi_decode_message(buf, len, txn->ei, txn->dest);
			if (ret < 0)
				pr_err("qmi: failed to decode response 0x%x: "
				    "%d\n", le16_to_cpu(hdr->msg_id), ret);
			txn->result = ret;
			complete(&txn->completion);
		} else {
			qmi_invoke_handler(qmi, sq, txn, buf, len);
		}
		mutex_unlock(&txn->lock);
	} else {
		memset(&tmp_txn, 0, sizeof(tmp_txn));
		tmp_txn.id = le16_to_cpu(hdr->txn_id);
		qmi_invoke_handler(qmi, sq, &tmp_txn, buf, len);
	}
}

/* -------------------------------------------------------------------- */
/* Name service (ns.c), for remote servers only. */

/* Tell the handles whose lookups match; returns how many matched. */
static int
qrtr_ns_notify(const struct qrtr_server *srv, bool new)
{
	struct lkpi_qrtr_port *p;
	struct qrtr_lookup *lookup;
	int matched = 0;

	sx_assert(&qrtr_ns_lock, SA_XLOCKED);
	list_for_each_entry(p, &qrtr_ports, link) {
		list_for_each_entry(lookup, &p->qmi->lookups, link) {
			if (!qrtr_lookup_match(lookup, srv))
				continue;
			if (new)
				qmi_recv_new_server(p->qmi, srv);
			else
				qmi_recv_del_server(p->qmi, srv->node,
				    srv->port);
			matched++;
			break;
		}
	}
	return (matched);
}

static void
qrtr_ns_server_del(struct qrtr_server *srv)
{
	sx_assert(&qrtr_ns_lock, SA_XLOCKED);
	qrtr_info("del server service 0x%x instance 0x%x at %u:%x\n",
	    srv->service, srv->instance, srv->node, srv->port);
	list_del(&srv->link);
	(void)qrtr_ns_notify(srv, false);
	kfree(srv);
}

static void
qrtr_ns_new_server(unsigned int service, unsigned int instance,
    unsigned int node, unsigned int port)
{
	struct qrtr_server *srv, *tmp;

	if (service == 0 || port == 0)
		return;

	sx_xlock(&qrtr_ns_lock);
	/* A new server on the same port replaces the old one. */
	list_for_each_entry_safe(srv, tmp, &qrtr_servers, link) {
		if (srv->node == node && srv->port == port)
			qrtr_ns_server_del(srv);
	}
	srv = kzalloc(sizeof(*srv), GFP_KERNEL);
	if (srv != NULL) {
		srv->service = service;
		srv->instance = instance;
		srv->node = node;
		srv->port = port;
		list_add_tail(&srv->link, &qrtr_servers);
		qrtr_info("new server service 0x%x instance 0x%x at %u:%x\n",
		    service, instance, node, port);
		if (qrtr_ns_notify(srv, true) == 0)
			qrtr_info("no lookup matches service 0x%x instance "
			    "0x%x\n", service, instance);
	}
	sx_xunlock(&qrtr_ns_lock);
}

static void
qrtr_ns_del_server(unsigned int node, unsigned int port)
{
	struct qrtr_server *srv, *tmp;

	sx_xlock(&qrtr_ns_lock);
	list_for_each_entry_safe(srv, tmp, &qrtr_servers, link) {
		if (srv->node == node && (port == -1U || srv->port == port))
			qrtr_ns_server_del(srv);
	}
	sx_xunlock(&qrtr_ns_lock);
}

static void
qrtr_ns_bye(unsigned int node)
{
	struct lkpi_qrtr_port *p;

	qrtr_ns_del_server(node, -1U);
	sx_xlock(&qrtr_ns_lock);
	list_for_each_entry(p, &qrtr_ports, link) {
		if (p->qmi->ops.bye != NULL)
			p->qmi->ops.bye(p->qmi, node);
	}
	sx_xunlock(&qrtr_ns_lock);
}

static void
qrtr_ns_del_client(unsigned int node, unsigned int port)
{
	struct lkpi_qrtr_port *p;

	sx_xlock(&qrtr_ns_lock);
	list_for_each_entry(p, &qrtr_ports, link) {
		if (p->qmi->ops.del_client != NULL)
			p->qmi->ops.del_client(p->qmi, node, port);
	}
	sx_xunlock(&qrtr_ns_lock);
}

static void
qrtr_rx_ctrl(struct qrtr_ep *ep, const struct qrtr_cb *cb, const void *data,
    size_t size)
{
	struct qrtr_ctrl_pkt pkt, hello;

	memset(&pkt, 0, sizeof(pkt));
	memcpy(&pkt, data, MIN(size, sizeof(pkt)));

	switch (cb->type) {
	case QRTR_TYPE_HELLO:
		qrtr_info("hello from node %u\n", cb->src_node);
		/* Answer, as ns.c say_hello() does; we serve nothing. */
		memset(&hello, 0, sizeof(hello));
		hello.cmd = cpu_to_le32(QRTR_TYPE_HELLO);
		if (qrtr_send_ctrl(cb->src_node, &hello) != 0)
			pr_err("qrtr: failed to answer hello from node %u\n",
			    cb->src_node);
		break;
	case QRTR_TYPE_BYE:
		qrtr_info("bye from node %u\n", cb->src_node);
		qrtr_ns_bye(cb->src_node);
		break;
	case QRTR_TYPE_NEW_SERVER:
		qrtr_ns_new_server(le32_to_cpu(pkt.server.service),
		    le32_to_cpu(pkt.server.instance),
		    le32_to_cpu(pkt.server.node),
		    le32_to_cpu(pkt.server.port));
		break;
	case QRTR_TYPE_DEL_SERVER:
		qrtr_ns_del_server(le32_to_cpu(pkt.server.node),
		    le32_to_cpu(pkt.server.port));
		break;
	case QRTR_TYPE_DEL_CLIENT:
		qrtr_ns_del_client(le32_to_cpu(pkt.client.node),
		    le32_to_cpu(pkt.client.port));
		break;
	default:
		qrtr_dbg("ignoring control packet type %u from node %u\n",
		    cb->type, cb->src_node);
		break;
	}
}

static void
qrtr_rx_data(const struct qrtr_cb *cb, const void *data, size_t size)
{
	struct lkpi_qrtr_port *p;
	struct sockaddr_qrtr sq;

	sq.sq_family = AF_QIPCRTR;
	sq.sq_node = cb->src_node;
	sq.sq_port = cb->src_port;

	/* Held across the handler: release() waits for us to finish. */
	sx_xlock(&qrtr_ns_lock);
	list_for_each_entry(p, &qrtr_ports, link) {
		if (p->port == cb->dst_port)
			break;
	}
	if (&p->link == &qrtr_ports) {
		sx_xunlock(&qrtr_ns_lock);
		qrtr_info("no port %x for data from %u:%x\n", cb->dst_port,
		    cb->src_node, cb->src_port);
		return;
	}
	if (p->qmi->ops.msg_handler != NULL)
		p->qmi->ops.msg_handler(p->qmi, &sq, data, size);
	else
		qmi_handle_message(p->qmi, &sq, data, size);
	sx_xunlock(&qrtr_ns_lock);
}

/*
 * Parse one packet from an endpoint, with the checks of
 * qrtr_endpoint_post().
 */
static void
qrtr_rx_packet(struct qrtr_ep *ep, const uint8_t *data, size_t len)
{
	const struct qrtr_hdr_v1 *v1;
	const struct qrtr_hdr_v2 *v2;
	const struct qrtr_ctrl_pkt *pkt;
	struct qrtr_cb cb;
	size_t hdrlen, size;

	if (len == 0 || (len & 3) != 0) {
		pr_err("qrtr: bad packet length %zu\n", len);
		return;
	}

	switch (data[0]) {
	case QRTR_PROTO_VER_1:
		if (len < sizeof(*v1))
			goto bad;
		v1 = (const struct qrtr_hdr_v1 *)data;
		hdrlen = sizeof(*v1);
		cb.type = le32_to_cpu(v1->type);
		cb.src_node = le32_to_cpu(v1->src_node_id);
		cb.src_port = le32_to_cpu(v1->src_port_id);
		cb.confirm_rx = v1->confirm_rx != 0;
		cb.dst_node = le32_to_cpu(v1->dst_node_id);
		cb.dst_port = le32_to_cpu(v1->dst_port_id);
		size = le32_to_cpu(v1->size);
		break;
	case QRTR_PROTO_VER_2:
		if (len < sizeof(*v2))
			goto bad;
		v2 = (const struct qrtr_hdr_v2 *)data;
		hdrlen = sizeof(*v2) + v2->optlen;
		cb.type = v2->type;
		cb.confirm_rx = (v2->flags & QRTR_FLAGS_CONFIRM_RX) != 0;
		cb.src_node = le16_to_cpu(v2->src_node_id);
		cb.src_port = le16_to_cpu(v2->src_port_id);
		cb.dst_node = le16_to_cpu(v2->dst_node_id);
		cb.dst_port = le16_to_cpu(v2->dst_port_id);
		if (cb.src_port == (uint16_t)QRTR_PORT_CTRL)
			cb.src_port = QRTR_PORT_CTRL;
		if (cb.dst_port == (uint16_t)QRTR_PORT_CTRL)
			cb.dst_port = QRTR_PORT_CTRL;
		size = le32_to_cpu(v2->size);
		break;
	default:
		pr_err("qrtr: invalid version %u\n", data[0]);
		return;
	}

	if (cb.dst_port == QRTR_PORT_CTRL_LEGACY)
		cb.dst_port = QRTR_PORT_CTRL;

	if (size == 0 || len != ALIGN(size, 4) + hdrlen)
		goto bad;
	if ((cb.type == QRTR_TYPE_NEW_SERVER ||
	    cb.type == QRTR_TYPE_RESUME_TX) &&
	    size < sizeof(struct qrtr_ctrl_pkt))
		goto bad;
	if (cb.dst_port != QRTR_PORT_CTRL && cb.type != QRTR_TYPE_DATA &&
	    cb.type != QRTR_TYPE_RESUME_TX)
		goto bad;

	qrtr_dbg("rx type %u %u:%x -> %u:%x len %zu%s\n", cb.type,
	    cb.src_node, cb.src_port, cb.dst_node, cb.dst_port, size,
	    cb.confirm_rx ? " confirm" : "");

	sx_xlock(&qrtr_ep_lock);
	qrtr_ep_assign(ep, cb.src_node);
	if (cb.type == QRTR_TYPE_NEW_SERVER) {
		/* The remote node can bridge other nodes. */
		pkt = (const struct qrtr_ctrl_pkt *)(data + hdrlen);
		qrtr_ep_assign(ep, le32_to_cpu(pkt->server.node));
	}
	sx_xunlock(&qrtr_ep_lock);

	if (cb.type == QRTR_TYPE_RESUME_TX) {
		/* We never ask for confirmation, nothing waits on it. */
		return;
	}

	if (cb.dst_port == QRTR_PORT_CTRL)
		qrtr_rx_ctrl(ep, &cb, data + hdrlen, size);
	else
		qrtr_rx_data(&cb, data + hdrlen, size);

	if (cb.confirm_rx)
		qrtr_send_resume_tx(&cb);
	return;

bad:
	pr_err("qrtr: dropping malformed packet of %zu bytes\n", len);
}

/* Report the servers already known to a new lookup, as ns.c does. */
static void
qrtr_rx_lookup(struct qmi_handle *qmi, struct qrtr_lookup *lookup)
{
	struct lkpi_qrtr_port *p;
	struct qrtr_server *srv;

	sx_xlock(&qrtr_ns_lock);
	/* The handle may have been released since. */
	list_for_each_entry(p, &qrtr_ports, link) {
		if (p->qmi == qmi)
			break;
	}
	if (&p->link != &qrtr_ports) {
		list_for_each_entry(srv, &qrtr_servers, link) {
			if (qrtr_lookup_match(lookup, srv))
				qmi_recv_new_server(qmi, srv);
		}
	}
	sx_xunlock(&qrtr_ns_lock);
}

static void
qrtr_rx_work_fn(struct work_struct *work __unused)
{
	struct qrtr_rx *rx;

	for (;;) {
		mtx_lock(&qrtr_rx_lock);
		rx = list_first_entry_or_null(&qrtr_rxq, struct qrtr_rx, link);
		if (rx != NULL)
			list_del(&rx->link);
		mtx_unlock(&qrtr_rx_lock);
		if (rx == NULL)
			break;

		if (rx->ep != NULL)
			qrtr_rx_packet(rx->ep, rx->data, rx->len);
		else
			qrtr_rx_lookup(rx->qmi, rx->lookup);
		free(rx, M_QRTR);
	}
}

static void
qrtr_rx_enqueue(struct qrtr_rx *rx)
{
	mtx_lock(&qrtr_rx_lock);
	list_add_tail(&rx->link, &qrtr_rxq);
	mtx_unlock(&qrtr_rx_lock);
	queue_work(qrtr_wq, &qrtr_rx_work);
}

/* -------------------------------------------------------------------- */
/* The IPCR MHI client (net/qrtr/mhi.c). */

static void
qrtr_mhi_dl_cb(struct mhi_device *mhi_dev, struct mhi_result *res)
{
	struct qrtr_ep *ep;
	struct qrtr_rx *rx;
	int error;

	ep = dev_get_drvdata(&mhi_dev->dev);
	/* The channel was reset: the buffer comes back to be freed. */
	if (res->transaction_status == -ENOTCONN) {
		kfree(res->buf_addr);
		return;
	}
	if (ep == NULL || res->transaction_status != 0)
		return;

	/* Completion context: copy and hand over to the worker. */
	rx = malloc(sizeof(*rx) + res->bytes_xferd, M_QRTR, M_NOWAIT);
	if (rx != NULL) {
		rx->ep = ep;
		rx->qmi = NULL;
		rx->lookup = NULL;
		rx->len = res->bytes_xferd;
		memcpy(rx->data, res->buf_addr, res->bytes_xferd);
		qrtr_rx_enqueue(rx);
	} else {
		pr_err("qrtr: dropping %zu byte packet, no memory\n",
		    res->bytes_xferd);
	}

	/* Recycle the buffer. */
	error = mhi_queue_buf(mhi_dev, DMA_FROM_DEVICE, res->buf_addr,
	    mhi_dev->mhi_cntrl->buffer_len, MHI_EOT);
	if (error != 0) {
		dev_err(&mhi_dev->dev, "failed to recycle buffer: %d\n", error);
		kfree(res->buf_addr);
	}
}

static int
qrtr_mhi_queue_dl_buffers(struct mhi_device *mhi_dev)
{
	void *buf;
	int error, free_desc;

	free_desc = mhi_get_free_desc_count(mhi_dev, DMA_FROM_DEVICE);
	while (free_desc-- > 0) {
		buf = kmalloc(mhi_dev->mhi_cntrl->buffer_len, GFP_KERNEL);
		if (buf == NULL)
			return (-ENOMEM);
		error = mhi_queue_buf(mhi_dev, DMA_FROM_DEVICE, buf,
		    mhi_dev->mhi_cntrl->buffer_len, MHI_EOT);
		if (error != 0) {
			dev_err(&mhi_dev->dev, "failed to queue buffer: %d\n",
			    error);
			kfree(buf);
			return (error);
		}
	}
	return (0);
}

static int
qrtr_mhi_probe(struct mhi_device *mhi_dev, const struct mhi_device_id *id)
{
	struct qrtr_ep *ep;
	int error;

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (ep == NULL)
		return (-ENOMEM);
	ep->mhi_dev = mhi_dev;
	dev_set_drvdata(&mhi_dev->dev, ep);

	sx_xlock(&qrtr_ep_lock);
	list_add_tail(&ep->link, &qrtr_eps);
	sx_xunlock(&qrtr_ep_lock);

	/* Start the channels, then hand the receive buffers over. */
	error = mhi_prepare_for_transfer(mhi_dev);
	if (error == 0) {
		error = qrtr_mhi_queue_dl_buffers(mhi_dev);
		if (error != 0)
			mhi_unprepare_from_transfer(mhi_dev);
	}
	if (error != 0) {
		sx_xlock(&qrtr_ep_lock);
		list_del(&ep->link);
		sx_xunlock(&qrtr_ep_lock);
		dev_set_drvdata(&mhi_dev->dev, NULL);
		kfree(ep);
		return (error);
	}
	qrtr_info("endpoint %p up on %s\n", ep, dev_name(&mhi_dev->dev));

	return (0);
}

static void
qrtr_mhi_remove(struct mhi_device *mhi_dev)
{
	struct qrtr_ep *ep;
	struct qrtr_rx *rx, *tmp;
	unsigned int nids[QRTR_EP_NIDS];
	int i, nnids;

	ep = dev_get_drvdata(&mhi_dev->dev);
	if (ep == NULL)
		return;
	qrtr_info("endpoint %p going down\n", ep);

	sx_xlock(&qrtr_ep_lock);
	ep->dead = true;
	nnids = ep->nnids;
	memcpy(nids, ep->nids, sizeof(nids));
	sx_xunlock(&qrtr_ep_lock);

	mhi_unprepare_from_transfer(mhi_dev);

	/* Drop what this endpoint still has queued, then wait the worker. */
	mtx_lock(&qrtr_rx_lock);
	list_for_each_entry_safe(rx, tmp, &qrtr_rxq, link) {
		if (rx->ep == ep) {
			list_del(&rx->link);
			free(rx, M_QRTR);
		}
	}
	mtx_unlock(&qrtr_rx_lock);
	flush_workqueue(qrtr_wq);

	/* The remote nodes are gone: a local BYE, as qrtr does on unregister. */
	for (i = 0; i < nnids; i++)
		qrtr_ns_bye(nids[i]);

	sx_xlock(&qrtr_ep_lock);
	list_del(&ep->link);
	sx_xunlock(&qrtr_ep_lock);
	dev_set_drvdata(&mhi_dev->dev, NULL);
	kfree(ep);
}

static const struct mhi_device_id qrtr_mhi_ids[] = {
	{ .chan = "IPCR" },
	{}
};

static struct mhi_driver qrtr_mhi_driver = {
	.id_table = qrtr_mhi_ids,
	.probe = qrtr_mhi_probe,
	.remove = qrtr_mhi_remove,
	.ul_xfer_cb = qrtr_ul_free,
	.dl_xfer_cb = qrtr_mhi_dl_cb,
	.driver = {
		.name = "qcom_mhi_qrtr",
	},
};

/* -------------------------------------------------------------------- */
/* The QMI client interface. */

int
qmi_handle_init(struct qmi_handle *qmi, size_t recv_buf_size __unused,
    const struct qmi_ops *ops, const struct qmi_msg_handler *handlers)
{
	struct lkpi_qrtr_port *p, *q;
	unsigned int port, tries;

	INIT_LIST_HEAD(&qmi->lookups);
	INIT_LIST_HEAD(&qmi->lookup_results);
	INIT_LIST_HEAD(&qmi->services);
	idr_init(&qmi->txns);
	mutex_init(&qmi->sock_lock);
	mutex_init(&qmi->txn_lock);
	qmi->handlers = handlers;
	if (ops != NULL)
		qmi->ops = *ops;
	else
		memset(&qmi->ops, 0, sizeof(qmi->ops));

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL)
		return (-ENOMEM);
	p->qmi = qmi;

	sx_xlock(&qrtr_ns_lock);
	for (tries = 0; tries <= QRTR_MAX_EPH_SOCKET - QRTR_MIN_EPH_SOCKET;
	    tries++) {
		port = qrtr_next_port++;
		if (qrtr_next_port > QRTR_MAX_EPH_SOCKET)
			qrtr_next_port = QRTR_MIN_EPH_SOCKET;
		list_for_each_entry(q, &qrtr_ports, link) {
			if (q->port == port)
				break;
		}
		if (&q->link == &qrtr_ports)
			break;
	}
	if (tries > QRTR_MAX_EPH_SOCKET - QRTR_MIN_EPH_SOCKET) {
		sx_xunlock(&qrtr_ns_lock);
		kfree(p);
		return (-EADDRINUSE);
	}
	p->port = port;
	list_add_tail(&p->link, &qrtr_ports);
	sx_xunlock(&qrtr_ns_lock);

	qmi->sq.sq_family = AF_QIPCRTR;
	qmi->sq.sq_node = qrtr_local_nid;
	qmi->sq.sq_port = port;
	mutex_lock(&qmi->sock_lock);
	qmi->sock = p;
	mutex_unlock(&qmi->sock_lock);
	qrtr_dbg("qmi handle %p on port %x\n", qmi, port);

	return (0);
}
EXPORT_SYMBOL_GPL(qmi_handle_init);

void
qmi_handle_release(struct qmi_handle *qmi)
{
	struct lkpi_qrtr_port *p;
	struct qrtr_lookup *lookup, *ltmp;
	struct qmi_service *svc, *stmp;
	struct qrtr_rx *rx, *rtmp;
	struct qmi_txn *txn;
	int id;

	mutex_lock(&qmi->sock_lock);
	p = qmi->sock;
	qmi->sock = NULL;
	mutex_unlock(&qmi->sock_lock);
	if (p == NULL)
		return;

	sx_xlock(&qrtr_ns_lock);
	list_del(&p->link);
	sx_xunlock(&qrtr_ns_lock);

	/* Pending lookup scans for this handle, then a running one. */
	mtx_lock(&qrtr_rx_lock);
	list_for_each_entry_safe(rx, rtmp, &qrtr_rxq, link) {
		if (rx->ep == NULL && rx->qmi == qmi) {
			list_del(&rx->link);
			free(rx, M_QRTR);
		}
	}
	mtx_unlock(&qrtr_rx_lock);
	flush_workqueue(qrtr_wq);
	kfree(p);

	/* Wake whoever still waits on a transaction. */
	mutex_lock(&qmi->txn_lock);
	idr_for_each_entry(&qmi->txns, txn, id) {
		mutex_lock(&txn->lock);
		idr_remove(&qmi->txns, txn->id);
		txn->result = -ENETRESET;
		complete(&txn->completion);
		mutex_unlock(&txn->lock);
	}
	mutex_unlock(&qmi->txn_lock);
	idr_destroy(&qmi->txns);

	list_for_each_entry_safe(lookup, ltmp, &qmi->lookups, link) {
		list_del(&lookup->link);
		kfree(lookup);
	}
	list_for_each_entry_safe(svc, stmp, &qmi->lookup_results, list_node) {
		list_del(&svc->list_node);
		kfree(svc);
	}
	list_for_each_entry_safe(svc, stmp, &qmi->services, list_node) {
		list_del(&svc->list_node);
		kfree(svc);
	}
}
EXPORT_SYMBOL_GPL(qmi_handle_release);

int
qmi_add_lookup(struct qmi_handle *qmi, unsigned int service,
    unsigned int version, unsigned int instance)
{
	struct qrtr_lookup *lookup;
	struct qrtr_rx *rx;

	lookup = kzalloc(sizeof(*lookup), GFP_KERNEL);
	if (lookup == NULL)
		return (-ENOMEM);
	lookup->service = service;
	lookup->instance = version | instance << 8;

	rx = malloc(sizeof(*rx), M_QRTR, M_WAITOK | M_ZERO);

	sx_xlock(&qrtr_ns_lock);
	list_add_tail(&lookup->link, &qmi->lookups);
	sx_xunlock(&qrtr_ns_lock);
	qrtr_info("qmi handle %p looks up service 0x%x instance 0x%x\n", qmi,
	    service, lookup->instance);

	/* Servers already known are reported from the worker, as on Linux. */
	rx->qmi = qmi;
	rx->lookup = lookup;
	qrtr_rx_enqueue(rx);

	return (0);
}
EXPORT_SYMBOL_GPL(qmi_add_lookup);

int
lkpi_qrtr_connect(struct lkpi_qrtr_port *p, const struct sockaddr_unsized *sa,
    int salen, int flags __unused)
{
	const struct sockaddr_qrtr *sq;

	if (p == NULL)
		return (-EINVAL);
	if (salen < (int)sizeof(*sq))
		return (-EINVAL);
	sq = (const struct sockaddr_qrtr *)sa;
	if (sq->sq_family != AF_QIPCRTR)
		return (-EINVAL);

	mutex_lock(&p->qmi->sock_lock);
	p->peer = *sq;
	mutex_unlock(&p->qmi->sock_lock);

	return (0);
}

int
qmi_txn_init(struct qmi_handle *qmi, struct qmi_txn *txn,
    const struct qmi_elem_info *ei, void *c_struct)
{
	int ret;

	memset(txn, 0, sizeof(*txn));
	mutex_init(&txn->lock);
	init_completion(&txn->completion);
	txn->qmi = qmi;
	txn->ei = ei;
	txn->dest = c_struct;

	mutex_lock(&qmi->txn_lock);
	ret = idr_alloc_cyclic(&qmi->txns, txn, 0, U16_MAX, GFP_KERNEL);
	if (ret < 0)
		pr_err("qmi: failed to allocate transaction id\n");
	txn->id = ret;
	mutex_unlock(&qmi->txn_lock);

	return (ret);
}
EXPORT_SYMBOL_GPL(qmi_txn_init);

int
qmi_txn_wait(struct qmi_txn *txn, unsigned long timeout)
{
	struct qmi_handle *qmi = txn->qmi;
	unsigned long ret;

	ret = wait_for_completion_timeout(&txn->completion, timeout);

	mutex_lock(&qmi->txn_lock);
	mutex_lock(&txn->lock);
	idr_remove(&qmi->txns, txn->id);
	mutex_unlock(&txn->lock);
	mutex_unlock(&qmi->txn_lock);

	if (ret == 0)
		return (-ETIMEDOUT);
	return (txn->result);
}
EXPORT_SYMBOL_GPL(qmi_txn_wait);

void
qmi_txn_cancel(struct qmi_txn *txn)
{
	struct qmi_handle *qmi = txn->qmi;

	mutex_lock(&qmi->txn_lock);
	mutex_lock(&txn->lock);
	idr_remove(&qmi->txns, txn->id);
	mutex_unlock(&txn->lock);
	mutex_unlock(&qmi->txn_lock);
}
EXPORT_SYMBOL_GPL(qmi_txn_cancel);

static ssize_t
qmi_send_message(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
    struct qmi_txn *txn, int type, int msg_id, size_t len,
    const struct qmi_elem_info *ei, const void *c_struct)
{
	struct sockaddr_qrtr dst;
	void *msg;
	int ret;

	msg = qmi_encode_message(type, msg_id, &len, txn->id, ei, c_struct);
	if (IS_ERR(msg))
		return (PTR_ERR(msg));

	mutex_lock(&qmi->sock_lock);
	if (qmi->sock == NULL) {
		ret = -EPIPE;
		goto out;
	}
	if (sq != NULL) {
		dst = *sq;
	} else {
		dst = qmi->sock->peer;
		if (dst.sq_family != AF_QIPCRTR) {
			ret = -ENOTCONN;
			goto out;
		}
	}
	qrtr_dbg("qmi tx type %d txn %u msg 0x%x len %zu to %u:%x\n", type,
	    txn->id, msg_id, len, dst.sq_node, dst.sq_port);
	ret = qrtr_send(QRTR_TYPE_DATA, qrtr_local_nid, qmi->sq.sq_port,
	    dst.sq_node, dst.sq_port, msg, len);
	if (ret < 0)
		pr_err("qmi: failed to send message 0x%x: %d\n", msg_id, ret);
out:
	mutex_unlock(&qmi->sock_lock);
	kfree(msg);

	return (ret < 0 ? ret : 0);
}

ssize_t
qmi_send_request(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
    struct qmi_txn *txn, int msg_id, size_t len,
    const struct qmi_elem_info *ei, const void *c_struct)
{
	return (qmi_send_message(qmi, sq, txn, QMI_REQUEST, msg_id, len, ei,
	    c_struct));
}
EXPORT_SYMBOL_GPL(qmi_send_request);

/* -------------------------------------------------------------------- */

int
lkpi_qrtr_init(void)
{
	int error;

	sx_init(&qrtr_ep_lock, "qrtr ep");
	sx_init(&qrtr_ns_lock, "qrtr ns");
	mtx_init(&qrtr_rx_lock, "qrtr rx", NULL, MTX_DEF);
	INIT_WORK(&qrtr_rx_work, qrtr_rx_work_fn);
	qrtr_wq = alloc_ordered_workqueue("qrtr", 0);
	if (qrtr_wq == NULL) {
		error = -ENOMEM;
		goto fail;
	}
	error = mhi_driver_register(&qrtr_mhi_driver);
	if (error != 0)
		goto fail;
	return (0);

fail:
	if (qrtr_wq != NULL)
		destroy_workqueue(qrtr_wq);
	mtx_destroy(&qrtr_rx_lock);
	sx_destroy(&qrtr_ns_lock);
	sx_destroy(&qrtr_ep_lock);
	return (error);
}

void
lkpi_qrtr_uninit(void)
{
	struct qrtr_server *srv, *tmp;

	mhi_driver_unregister(&qrtr_mhi_driver);
	flush_workqueue(qrtr_wq);
	destroy_workqueue(qrtr_wq);
	list_for_each_entry_safe(srv, tmp, &qrtr_servers, link) {
		list_del(&srv->link);
		kfree(srv);
	}
	mtx_destroy(&qrtr_rx_lock);
	sx_destroy(&qrtr_ns_lock);
	sx_destroy(&qrtr_ep_lock);
}

bool
lkpi_qrtr_busy(void)
{
	bool busy;

	sx_slock(&qrtr_ns_lock);
	busy = !list_empty(&qrtr_ports);
	sx_sunlock(&qrtr_ns_lock);
	sx_slock(&qrtr_ep_lock);
	busy |= !list_empty(&qrtr_eps);
	sx_sunlock(&qrtr_ep_lock);
	return (busy);
}
