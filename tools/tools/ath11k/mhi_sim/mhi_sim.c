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
 * A software model of the device side of an ath11k-class MHI device, to
 * exercise the MHI host core, the MHI bus glue, QRTR and the QMI client
 * interface of linuxkpi_mhi without hardware.
 *
 * The model answers register accesses through the controller's read_reg
 * and write_reg callbacks and does the device's work on its own thread,
 * accessing host memory through the direct map as a bus master would:
 *  - PBL: BHI download of the SBL part of the image, checked byte for byte;
 *  - READY, then M0 on request, with the M0 state change and SBL EE events;
 *  - SBL: BHIe download of the full image through the vector table,
 *    checked byte for byte, then the AMSS EE event;
 *  - the command ring (channel start, stop, reset) and command completions;
 *  - transfers on the IPCR channels with transfer completion events;
 *  - a firmware QRTR node (7) that says hello, announces a WLFW-like QMI
 *    service once greeted, asks for a confirmation (RESUME_TX), and
 *    answers requests and follows a chosen one with an indication.
 * Interrupts are the test interrupts of linuxkpi_mhi.
 *
 * sysctl debug.mhisim.run=1 runs one power up, QMI exchange and power
 * down; the result is printed and left in debug.mhisim.result.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/pctrie.h>
#include <sys/priority.h>
#include <sys/proc.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/bus.h>
#include <machine/vmparam.h>

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/mhi.h>
#include <linux/pci.h>
#include <linux/qrtr.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/socket.h>

#include <contrib/dev/mhi/common.h>

/* From linuxkpi_mhi's mhi_freebsd.h. */
#define	MHI_FBSD_TEST_IRQ_BASE	0x40000000u
void	mhi_fbsd_test_irq_raise(unsigned int);

/*
 * LinuxKPI's private DMA state of a device, as in linux_pci.c, which only
 * sets it up for PCI devices it attaches.  Must match that file.
 */
struct linux_dma_priv {
	uint64_t	dma_mask;
	bus_dma_tag_t	dmat;
	uint64_t	dma_coherent_mask;
	bus_dma_tag_t	dmat_coherent;
	struct mtx	lock;
	struct pctrie	ptree;
};

static MALLOC_DEFINE(M_MHISIM, "mhisim", "MHI device model");

/* Register map of the model. */
#define	SIM_REGLEN	0x1000
#define	SIM_BHIOFF	0x100
#define	SIM_BHIEOFF	0x200
#define	SIM_CHDBOFF	0x400
#define	SIM_ERDBOFF	0x800
#define	SIM_NCH		128
#define	SIM_NER		8
#define	SIM_IRQS	3

/* The image: SBL part through BHI, all of it through BHIe. */
#define	SIM_SBL_SIZE	(512 * 1024)
#define	SIM_SEG_LEN	(512 * 1024)
#define	SIM_FW_SIZE	(3 * SIM_SEG_LEN + 1000)
#define	SIM_FW_NAME	"mhisim/amss.bin"

/* QRTR and QMI of the model's firmware node. */
#define	SIM_NODE	7
#define	SIM_PORT	1
#define	SIM_SERVICE	0x45
#define	SIM_VERSION	1
#define	SIM_INSTANCE	2
#define	SIM_REQ_ID	0x20
#define	SIM_IND_ID	0x21
#define	SIM_IND_VALUE	0xdeadbeefU

#define	QRTR_PROTO_VER_1	1

struct sim_hdr_v1 {
	__le32	version;
	__le32	type;
	__le32	src_node_id;
	__le32	src_port_id;
	__le32	confirm_rx;
	__le32	size;
	__le32	dst_node_id;
	__le32	dst_port_id;
} __packed;

struct sim_msg {
	STAILQ_ENTRY(sim_msg)	link;
	size_t			len;
	uint8_t			data[];
};

struct mhisim {
	struct mtx		lock;		/* registers and rings */
	uint32_t		regs[SIM_REGLEN / 4];
	uint8_t			*regbase;	/* what the host offsets from */
	device_t		bsddev;
	struct pci_dev		*pdev;
	struct mhi_controller	*mhi;
	int			irqs[SIM_IRQS];
	uint8_t			*fw;
	const struct firmware	*fwreg;

	struct taskqueue	*tq;
	struct task		bhi_task;
	struct task		bhie_task;
	struct task		m0_task;
	struct task		cmd_task;
	struct task		ch_task;
	struct task		reset_task;

	/* Device state. */
	enum mhi_ee_type	ee;
	uint64_t		crdb;
	uint64_t		cmd_rp;
	uint64_t		chdb[SIM_NCH];
	uint64_t		ch_rp[SIM_NCH];
	bool			ch_running[SIM_NCH];
	bool			hello_sent;
	STAILQ_HEAD(, sim_msg)	dlq;		/* to the host, on channel 21 */

	/* Observations. */
	int			errors;
	struct sbuf		*log;
	bool			bhi_ok, bhie_ok, greeted, resumed;
	bool			req_ok;
	int			host_port;
};

static struct mhisim *sim;
static struct sx sim_sx;
SX_SYSINIT(mhisim, &sim_sx, "mhisim run");
static char sim_result[1024] = "not run";

#define	SIMLOG(fmt, ...) do {						\
	printf("mhisim: " fmt "\n", ##__VA_ARGS__);			\
} while (0)
#define	SIMERR(fmt, ...) do {						\
	sim->errors++;							\
	printf("mhisim: ERROR " fmt "\n", ##__VA_ARGS__);		\
} while (0)

static uint8_t
sim_fw_byte(size_t i)
{
	return ((uint8_t)((i * 2654435761u) >> 13) ^ (uint8_t)(i >> 9));
}

static void *
sim_dmap(uint64_t busaddr)
{
	/* No IOMMU: bus addresses are physical. */
	return ((void *)PHYS_TO_DMAP(busaddr));
}

static uint32_t
sim_reg(uint32_t off)
{
	return (sim->regs[off / 4]);
}

static void
sim_set_reg(uint32_t off, uint32_t val)
{
	sim->regs[off / 4] = val;
}

static void
sim_set_state(enum mhi_state state, bool ready)
{
	uint32_t v;

	v = sim_reg(MHISTATUS);
	v &= ~(MHISTATUS_MHISTATE_MASK | MHISTATUS_READY_MASK);
	v |= FIELD_PREP(MHISTATUS_MHISTATE_MASK, state);
	if (ready)
		v |= MHISTATUS_READY_MASK;
	sim_set_reg(MHISTATUS, v);
}

static uint64_t
sim_reg64(uint32_t lo)
{
	return ((uint64_t)sim_reg(lo + 4) << 32 | sim_reg(lo));
}

/* Check that a DMA address the host gave us is inside its window. */
static void
sim_check_window(uint64_t addr, size_t len, const char *what)
{
	uint64_t lo, hi;

	lo = sim_reg64(MHICTRLBASE_LOWER);
	hi = sim_reg64(MHICTRLLIMIT_LOWER);
	if (hi != 0 && (addr < lo || addr + len - 1 > hi))
		SIMERR("%s at %#jx+%zu outside window %#jx-%#jx", what,
		    (uintmax_t)addr, len, (uintmax_t)lo, (uintmax_t)hi);
}

/* ------------------------------------------------------------------ */
/* Event rings. */

static struct mhi_event_ctxt *
sim_er_ctxt(int er)
{
	return ((struct mhi_event_ctxt *)sim_dmap(sim_reg64(ECABAP_LOWER)) +
	    er);
}

static struct mhi_chan_ctxt *
sim_ch_ctxt(int ch)
{
	return ((struct mhi_chan_ctxt *)sim_dmap(sim_reg64(CCABAP_LOWER)) +
	    ch);
}

static struct mhi_cmd_ctxt *
sim_cmd_ctxt(void)
{
	return ((struct mhi_cmd_ctxt *)sim_dmap(sim_reg64(CRCBAP_LOWER)));
}

/* Write one event at the device read pointer and interrupt.  Locked. */
static void
sim_post_event(int er, uint64_t ptr, uint32_t dw0, uint32_t dw1)
{
	struct mhi_event_ctxt *ctx;
	struct mhi_ring_element *el;
	uint64_t base, len, rp, wp, nrp;
	unsigned int msivec;

	mtx_assert(&sim->lock, MA_OWNED);
	ctx = sim_er_ctxt(er);
	base = le64_to_cpu(ctx->rbase);
	len = le64_to_cpu(ctx->rlen);
	rp = le64_to_cpu(ctx->rp);
	wp = le64_to_cpu(ctx->wp);
	msivec = le32_to_cpu(ctx->msivec);
	if (len == 0 || rp < base || rp >= base + len) {
		SIMERR("event ring %d not set up (base %#jx len %ju rp %#jx)",
		    er, (uintmax_t)base, (uintmax_t)len, (uintmax_t)rp);
		return;
	}
	if (rp == wp) {
		SIMERR("event ring %d full", er);
		return;
	}
	el = sim_dmap(rp);
	el->ptr = cpu_to_le64(ptr);
	el->dword[0] = cpu_to_le32(dw0);
	el->dword[1] = cpu_to_le32(dw1);
	nrp = rp + sizeof(*el);
	if (nrp == base + len)
		nrp = base;
	wmb();
	ctx->rp = cpu_to_le64(nrp);
	wmb();
	if (msivec >= SIM_IRQS) {
		SIMERR("event ring %d has msivec %u", er, msivec);
		return;
	}
	mhi_fbsd_test_irq_raise(MHI_FBSD_TEST_IRQ_BASE + msivec);
}

static void
sim_post_state(enum mhi_state state)
{
	sim_post_event(0, 0, FIELD_PREP(GENMASK(31, 24), state),
	    FIELD_PREP(GENMASK(23, 16), MHI_PKT_TYPE_STATE_CHANGE_EVENT));
}

static void
sim_post_ee(enum mhi_ee_type ee)
{
	sim_post_event(0, 0, FIELD_PREP(GENMASK(31, 24), ee),
	    FIELD_PREP(GENMASK(23, 16), MHI_PKT_TYPE_EE_EVENT));
}

/* ------------------------------------------------------------------ */
/* Boot: BHI, READY, M0, BHIe. */

static void
sim_bhi_task(void *arg __unused, int pending __unused)
{
	uint64_t addr;
	uint32_t size, txdb;
	const uint8_t *img;
	size_t i, bad;

	mtx_lock(&sim->lock);
	addr = (uint64_t)sim_reg(SIM_BHIOFF + BHI_IMGADDR_HIGH) << 32 |
	    sim_reg(SIM_BHIOFF + BHI_IMGADDR_LOW);
	size = sim_reg(SIM_BHIOFF + BHI_IMGSIZE);
	txdb = sim_reg(SIM_BHIOFF + BHI_IMGTXDB);
	mtx_unlock(&sim->lock);

	SIMLOG("BHI: image at %#jx size %u session %#x", (uintmax_t)addr,
	    size, txdb);
	bad = 0;
	if (size != SIM_SBL_SIZE) {
		SIMERR("BHI: size %u, expected %u", size, SIM_SBL_SIZE);
		bad = 1;
	} else {
		img = sim_dmap(addr);
		for (i = 0; i < size; i++) {
			if (img[i] != sim_fw_byte(i)) {
				bad++;
				break;
			}
		}
		if (bad)
			SIMERR("BHI: image differs at byte %zu", i);
	}
	sim->bhi_ok = (bad == 0);

	mtx_lock(&sim->lock);
	sim_set_reg(SIM_BHIOFF + BHI_STATUS,
	    FIELD_PREP(BHI_STATUS_MASK, bad ? BHI_STATUS_ERROR :
	    BHI_STATUS_SUCCESS));
	if (!bad) {
		sim->ee = MHI_EE_SBL;
		sim_set_reg(SIM_BHIOFF + BHI_EXECENV, MHI_EE_SBL);
	}
	mtx_unlock(&sim->lock);
	mhi_fbsd_test_irq_raise(MHI_FBSD_TEST_IRQ_BASE + 0);
	if (bad)
		return;

	/* The SBL comes up and reports READY. */
	pause("mhisim", hz / 50);
	mtx_lock(&sim->lock);
	sim_set_state(MHI_STATE_READY, true);
	mtx_unlock(&sim->lock);
}

static void
sim_m0_task(void *arg __unused, int pending __unused)
{
	mtx_lock(&sim->lock);
	sim->cmd_rp = le64_to_cpu(sim_cmd_ctxt()->rbase);
	sim_set_state(MHI_STATE_M0, true);
	sim_post_state(MHI_STATE_M0);
	if (sim->ee == MHI_EE_SBL)
		sim_post_ee(MHI_EE_SBL);
	mtx_unlock(&sim->lock);
	SIMLOG("M0 in %s", sim->ee == MHI_EE_SBL ? "SBL" : "AMSS");
}

static void
sim_bhie_task(void *arg __unused, int pending __unused)
{
	struct bhi_vec {
		__le64	dma_addr;
		__le64	size;
	} *vec;
	uint64_t addr;
	uint32_t vsize, seq;
	size_t off, i, n, j, len;
	const uint8_t *seg;
	bool bad;

	mtx_lock(&sim->lock);
	addr = (uint64_t)sim_reg(SIM_BHIEOFF + BHIE_TXVECADDR_HIGH_OFFS) <<
	    32 | sim_reg(SIM_BHIEOFF + BHIE_TXVECADDR_LOW_OFFS);
	vsize = sim_reg(SIM_BHIEOFF + BHIE_TXVECSIZE_OFFS);
	seq = FIELD_GET(BHIE_TXVECDB_SEQNUM_BMSK,
	    sim_reg(SIM_BHIEOFF + BHIE_TXVECDB_OFFS));
	mtx_unlock(&sim->lock);

	n = vsize / sizeof(*vec);
	SIMLOG("BHIe: vector table at %#jx, %zu entries, sequence %#x",
	    (uintmax_t)addr, n, seq);
	sim_check_window(addr, vsize, "BHIe vector table");
	vec = sim_dmap(addr);
	bad = false;
	off = 0;
	for (i = 0; i < n && !bad; i++) {
		len = le64_to_cpu(vec[i].size);
		sim_check_window(le64_to_cpu(vec[i].dma_addr), len,
		    "BHIe segment");
		seg = sim_dmap(le64_to_cpu(vec[i].dma_addr));
		for (j = 0; j < len && off + j < SIM_FW_SIZE; j++) {
			if (seg[j] != sim_fw_byte(off + j)) {
				SIMERR("BHIe: segment %zu differs at byte %zu",
				    i, j);
				bad = true;
				break;
			}
		}
		off += len;
	}
	if (!bad && off < SIM_FW_SIZE) {
		SIMERR("BHIe: %zu bytes, expected %u", off, SIM_FW_SIZE);
		bad = true;
	}
	sim->bhie_ok = !bad;

	mtx_lock(&sim->lock);
	sim_set_reg(SIM_BHIEOFF + BHIE_TXVECSTATUS_OFFS,
	    FIELD_PREP(BHIE_TXVECSTATUS_STATUS_BMSK, bad ?
	    BHIE_TXVECSTATUS_STATUS_ERROR :
	    BHIE_TXVECSTATUS_STATUS_XFER_COMPL) |
	    FIELD_PREP(BHIE_TXVECSTATUS_SEQNUM_BMSK, seq));
	mtx_unlock(&sim->lock);
	mhi_fbsd_test_irq_raise(MHI_FBSD_TEST_IRQ_BASE + 0);
	if (bad)
		return;

	/* The firmware starts. */
	pause("mhisim", hz / 50);
	mtx_lock(&sim->lock);
	sim->ee = MHI_EE_AMSS;
	sim_set_reg(SIM_BHIOFF + BHI_EXECENV, MHI_EE_AMSS);
	sim_post_ee(MHI_EE_AMSS);
	mtx_unlock(&sim->lock);
	SIMLOG("AMSS running");
}

static void
sim_reset_task(void *arg __unused, int pending __unused)
{
	int ch;

	mtx_lock(&sim->lock);
	sim_set_reg(MHICTRL, sim_reg(MHICTRL) & ~MHICTRL_RESET_MASK);
	sim_set_reg(SIM_BHIOFF + BHI_INTVEC, 0);
	for (ch = 0; ch < SIM_NCH; ch++)
		sim->ch_running[ch] = false;
	sim->hello_sent = false;
	/* Out of PBL, the device comes back READY. */
	sim_set_state(MHI_STATE_RESET, sim->ee != MHI_EE_PBL);
	mtx_unlock(&sim->lock);
	SIMLOG("MHI reset");
}

/* ------------------------------------------------------------------ */
/* Firmware: QRTR node and QMI service. */

static void sim_dl_deliver(void);

/* Queue a QRTR packet for the host.  Locked. */
static void
sim_qrtr_send(uint32_t type, uint32_t src_port, uint32_t dst_node,
    uint32_t dst_port, bool confirm_rx, const void *payload, size_t len)
{
	struct sim_hdr_v1 *hdr;
	struct sim_msg *m;
	size_t total;

	total = sizeof(*hdr) + roundup2(len, 4);
	m = malloc(sizeof(*m) + total, M_MHISIM, M_NOWAIT | M_ZERO);
	if (m == NULL) {
		SIMERR("no memory for a message");
		return;
	}
	m->len = total;
	hdr = (struct sim_hdr_v1 *)m->data;
	hdr->version = cpu_to_le32(QRTR_PROTO_VER_1);
	hdr->type = cpu_to_le32(type);
	hdr->src_node_id = cpu_to_le32(SIM_NODE);
	hdr->src_port_id = cpu_to_le32(src_port);
	hdr->confirm_rx = cpu_to_le32(confirm_rx ? 1 : 0);
	hdr->size = cpu_to_le32(len);
	hdr->dst_node_id = cpu_to_le32(dst_node);
	hdr->dst_port_id = cpu_to_le32(dst_port);
	memcpy(m->data + sizeof(*hdr), payload, len);
	STAILQ_INSERT_TAIL(&sim->dlq, m, link);
}

static void
sim_send_ctrl(uint32_t cmd, bool confirm_rx)
{
	struct qrtr_ctrl_pkt pkt;

	memset(&pkt, 0, sizeof(pkt));
	pkt.cmd = cpu_to_le32(cmd);
	if (cmd == QRTR_TYPE_NEW_SERVER) {
		pkt.server.service = cpu_to_le32(SIM_SERVICE);
		pkt.server.instance = cpu_to_le32(SIM_VERSION |
		    SIM_INSTANCE << 8);
		pkt.server.node = cpu_to_le32(SIM_NODE);
		pkt.server.port = cpu_to_le32(SIM_PORT);
	}
	sim_qrtr_send(cmd, QRTR_PORT_CTRL, QRTR_NODE_BCAST, QRTR_PORT_CTRL,
	    confirm_rx, &pkt, sizeof(pkt));
}

/* A QMI request to our service: check it, answer, maybe indicate. */
static void
sim_qmi_request(uint32_t src_port, const uint8_t *msg, size_t len)
{
	const struct qmi_header *qh;
	struct {
		struct qmi_header	h;
		uint8_t			tlv[7];
	} __packed resp;
	struct {
		struct qmi_header	h;
		uint8_t			tlv[7];
	} __packed ind;
	static const uint8_t expect[] = {
		/* TLV 0x10: u8 enable = 1; TLV 0x11: u32 cookie = 0x12345678 */
		0x10, 0x01, 0x00, 0x01,
		0x11, 0x04, 0x00, 0x78, 0x56, 0x34, 0x12,
	};
	uint16_t txn, id, mlen;

	if (len < sizeof(*qh)) {
		SIMERR("QMI: short message of %zu bytes", len);
		return;
	}
	qh = (const struct qmi_header *)msg;
	txn = le16_to_cpu(qh->txn_id);
	id = le16_to_cpu(qh->msg_id);
	mlen = le16_to_cpu(qh->msg_len);
	SIMLOG("QMI: request type %u txn %u msg %#x len %u from port %#x",
	    qh->type, txn, id, mlen, src_port);
	if (qh->type != QMI_REQUEST || sizeof(*qh) + mlen != len) {
		SIMERR("QMI: bad header (type %u len %u of %zu)", qh->type,
		    mlen, len);
		return;
	}
	if (id == SIM_REQ_ID) {
		if (mlen != sizeof(expect) ||
		    memcmp(msg + sizeof(*qh), expect, sizeof(expect)) != 0) {
			SIMERR("QMI: request payload not as encoded by Linux");
		} else {
			sim->req_ok = true;
		}
	}

	/* Response: TLV 0x02, result and error. */
	memset(&resp, 0, sizeof(resp));
	resp.h.type = QMI_RESPONSE;
	resp.h.txn_id = cpu_to_le16(txn);
	resp.h.msg_id = cpu_to_le16(id);
	resp.h.msg_len = cpu_to_le16(sizeof(resp.tlv));
	resp.tlv[0] = 0x02;
	resp.tlv[1] = 0x04;
	resp.tlv[2] = 0x00;
	/* result 0, error 0 */
	sim_qrtr_send(QRTR_TYPE_DATA, SIM_PORT, 1, src_port, false, &resp,
	    sizeof(resp));

	if (id != SIM_REQ_ID)
		return;
	/* Indication: TLV 0x10, u32. */
	memset(&ind, 0, sizeof(ind));
	ind.h.type = QMI_INDICATION;
	ind.h.txn_id = 0;
	ind.h.msg_id = cpu_to_le16(SIM_IND_ID);
	ind.h.msg_len = cpu_to_le16(sizeof(ind.tlv));
	ind.tlv[0] = 0x10;
	ind.tlv[1] = 0x04;
	ind.tlv[2] = 0x00;
	ind.tlv[3] = SIM_IND_VALUE & 0xff;
	ind.tlv[4] = (SIM_IND_VALUE >> 8) & 0xff;
	ind.tlv[5] = (SIM_IND_VALUE >> 16) & 0xff;
	ind.tlv[6] = (SIM_IND_VALUE >> 24) & 0xff;
	sim_qrtr_send(QRTR_TYPE_DATA, SIM_PORT, 1, src_port, true, &ind,
	    sizeof(ind));
}

/* A QRTR packet from the host.  Locked. */
static void
sim_qrtr_rx(const uint8_t *data, size_t len)
{
	const struct sim_hdr_v1 *hdr;
	const struct qrtr_ctrl_pkt *pkt;
	uint32_t type, size, src_node, src_port, dst_node, dst_port;

	if (len < sizeof(*hdr) || (len & 3) != 0) {
		SIMERR("QRTR: bad length %zu", len);
		return;
	}
	hdr = (const struct sim_hdr_v1 *)data;
	if (le32_to_cpu(hdr->version) != QRTR_PROTO_VER_1) {
		SIMERR("QRTR: version %u", le32_to_cpu(hdr->version));
		return;
	}
	type = le32_to_cpu(hdr->type);
	size = le32_to_cpu(hdr->size);
	src_node = le32_to_cpu(hdr->src_node_id);
	src_port = le32_to_cpu(hdr->src_port_id);
	dst_node = le32_to_cpu(hdr->dst_node_id);
	dst_port = le32_to_cpu(hdr->dst_port_id);
	if (sizeof(*hdr) + roundup2(size, 4) != len) {
		SIMERR("QRTR: size %u in %zu bytes", size, len);
		return;
	}
	SIMLOG("QRTR: rx type %u %u:%#x -> %u:%#x size %u", type, src_node,
	    src_port, dst_node, dst_port, size);
	if (src_node != 1)
		SIMERR("QRTR: host node %u, expected 1", src_node);
	pkt = (const struct qrtr_ctrl_pkt *)(data + sizeof(*hdr));

	switch (type) {
	case QRTR_TYPE_HELLO:
		if (dst_node != SIM_NODE || dst_port != QRTR_PORT_CTRL ||
		    src_port != QRTR_PORT_CTRL)
			SIMERR("QRTR: hello addressed %u:%#x from %#x",
			    dst_node, dst_port, src_port);
		sim->greeted = true;
		/* Announce the service, and ask for a confirmation. */
		sim_send_ctrl(QRTR_TYPE_NEW_SERVER, true);
		break;
	case QRTR_TYPE_RESUME_TX:
		if (size < sizeof(*pkt) ||
		    le32_to_cpu(pkt->client.node) != 1) {
			SIMERR("QRTR: bad resume_tx");
			break;
		}
		SIMLOG("QRTR: resume_tx for %u:%#x",
		    le32_to_cpu(pkt->client.node),
		    le32_to_cpu(pkt->client.port));
		sim->resumed = true;
		break;
	case QRTR_TYPE_DATA:
		if (dst_node != SIM_NODE || dst_port != SIM_PORT) {
			SIMERR("QRTR: data for %u:%#x", dst_node, dst_port);
			break;
		}
		sim->host_port = src_port;
		sim_qmi_request(src_port, data + sizeof(*hdr), size);
		break;
	case QRTR_TYPE_BYE:
	case QRTR_TYPE_DEL_CLIENT:
		break;
	default:
		SIMERR("QRTR: unexpected type %u", type);
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Command ring and channels. */

static void
sim_cmd_task(void *arg __unused, int pending __unused)
{
	struct mhi_cmd_ctxt *ctx;
	struct mhi_ring_element *el;
	struct mhi_chan_ctxt *cc;
	uint64_t base, len;
	uint32_t type, ch, code;

	mtx_lock(&sim->lock);
	ctx = sim_cmd_ctxt();
	base = le64_to_cpu(ctx->rbase);
	len = le64_to_cpu(ctx->rlen);
	while (sim->cmd_rp != sim->crdb) {
		if (sim->cmd_rp < base || sim->cmd_rp >= base + len) {
			SIMERR("command rp %#jx outside ring",
			    (uintmax_t)sim->cmd_rp);
			break;
		}
		el = sim_dmap(sim->cmd_rp);
		type = MHI_TRE_GET_CMD_TYPE(el);
		ch = MHI_TRE_GET_CMD_CHID(el);
		code = MHI_EV_CC_SUCCESS;
		cc = sim_ch_ctxt(ch);
		switch (type) {
		case MHI_PKT_TYPE_START_CHAN_CMD:
			SIMLOG("command: start channel %u", ch);
			sim->ch_rp[ch] = le64_to_cpu(cc->rbase);
			sim->chdb[ch] = sim->ch_rp[ch];
			sim->ch_running[ch] = true;
			cc->chcfg = cpu_to_le32((le32_to_cpu(cc->chcfg) &
			    ~CHAN_CTX_CHSTATE_MASK) | MHI_CH_STATE_RUNNING);
			sim_check_window(le64_to_cpu(cc->rbase),
			    le64_to_cpu(cc->rlen), "channel ring");
			break;
		case MHI_PKT_TYPE_STOP_CHAN_CMD:
			SIMLOG("command: stop channel %u", ch);
			sim->ch_running[ch] = false;
			cc->chcfg = cpu_to_le32((le32_to_cpu(cc->chcfg) &
			    ~CHAN_CTX_CHSTATE_MASK) | MHI_CH_STATE_STOP);
			break;
		case MHI_PKT_TYPE_RESET_CHAN_CMD:
			SIMLOG("command: reset channel %u", ch);
			sim->ch_running[ch] = false;
			cc->chcfg = cpu_to_le32((le32_to_cpu(cc->chcfg) &
			    ~CHAN_CTX_CHSTATE_MASK) | MHI_CH_STATE_DISABLED);
			break;
		default:
			SIMERR("command: type %#x", type);
			code = MHI_EV_CC_UNDEFINED_ERR;
			break;
		}
		sim_post_event(0, sim->cmd_rp, FIELD_PREP(GENMASK(31, 24), code),
		    FIELD_PREP(GENMASK(23, 16),
		    MHI_PKT_TYPE_CMD_COMPLETION_EVENT));
		sim->cmd_rp += sizeof(*el);
		if (sim->cmd_rp == base + len)
			sim->cmd_rp = base;
		ctx->rp = cpu_to_le64(sim->cmd_rp);
	}
	mtx_unlock(&sim->lock);
}

/* Fill the host's receive buffers on channel 21.  Locked. */
static void
sim_dl_deliver(void)
{
	struct mhi_chan_ctxt *cc;
	struct mhi_ring_element *el;
	struct sim_msg *m;
	uint64_t base, len, rp;
	uint32_t blen;
	int ch = 21;

	if (!sim->ch_running[ch])
		return;
	cc = sim_ch_ctxt(ch);
	base = le64_to_cpu(cc->rbase);
	len = le64_to_cpu(cc->rlen);
	while ((m = STAILQ_FIRST(&sim->dlq)) != NULL &&
	    sim->ch_rp[ch] != sim->chdb[ch]) {
		rp = sim->ch_rp[ch];
		el = sim_dmap(rp);
		blen = MHI_TRE_DATA_GET_LEN(el);
		if (blen < m->len) {
			SIMERR("DL buffer of %u bytes for %zu", blen, m->len);
			break;
		}
		sim_check_window(MHI_TRE_DATA_GET_PTR(el), m->len,
		    "DL buffer");
		memcpy(sim_dmap(MHI_TRE_DATA_GET_PTR(el)), m->data, m->len);
		wmb();
		STAILQ_REMOVE_HEAD(&sim->dlq, link);
		sim->ch_rp[ch] = rp + sizeof(*el) == base + len ? base :
		    rp + sizeof(*el);
		cc->rp = cpu_to_le64(sim->ch_rp[ch]);
		sim_post_event(le32_to_cpu(cc->erindex), rp,
		    FIELD_PREP(GENMASK(31, 24), MHI_EV_CC_EOT) |
		    FIELD_PREP(GENMASK(15, 0), m->len),
		    FIELD_PREP(GENMASK(31, 24), ch) |
		    FIELD_PREP(GENMASK(23, 16), MHI_PKT_TYPE_TX_EVENT));
		free(m, M_MHISIM);
	}
}

/* Consume what the host queued on channel 20. Locked. */
static void
sim_ul_consume(void)
{
	struct mhi_chan_ctxt *cc;
	struct mhi_ring_element *el;
	uint64_t base, len, rp;
	uint32_t blen;
	int ch = 20;

	if (!sim->ch_running[ch])
		return;
	cc = sim_ch_ctxt(ch);
	base = le64_to_cpu(cc->rbase);
	len = le64_to_cpu(cc->rlen);
	while (sim->ch_rp[ch] != sim->chdb[ch]) {
		rp = sim->ch_rp[ch];
		el = sim_dmap(rp);
		blen = MHI_TRE_DATA_GET_LEN(el);
		sim_check_window(MHI_TRE_DATA_GET_PTR(el), blen, "UL buffer");
		if (!MHI_TRE_DATA_GET_IEOT(el))
			SIMERR("UL TRE without IEOT");
		sim_qrtr_rx(sim_dmap(MHI_TRE_DATA_GET_PTR(el)), blen);
		sim->ch_rp[ch] = rp + sizeof(*el) == base + len ? base :
		    rp + sizeof(*el);
		cc->rp = cpu_to_le64(sim->ch_rp[ch]);
		sim_post_event(le32_to_cpu(cc->erindex), rp,
		    FIELD_PREP(GENMASK(31, 24), MHI_EV_CC_EOT) |
		    FIELD_PREP(GENMASK(15, 0), blen),
		    FIELD_PREP(GENMASK(31, 24), ch) |
		    FIELD_PREP(GENMASK(23, 16), MHI_PKT_TYPE_TX_EVENT));
	}
}

static void
sim_ch_task(void *arg __unused, int pending __unused)
{
	mtx_lock(&sim->lock);
	sim_ul_consume();
	/* The firmware greets the host once it can be heard. */
	if (sim->ch_running[21] && sim->chdb[21] != sim->ch_rp[21] &&
	    !sim->hello_sent) {
		sim->hello_sent = true;
		sim_send_ctrl(QRTR_TYPE_HELLO, false);
	}
	sim_dl_deliver();
	mtx_unlock(&sim->lock);
}

/* ------------------------------------------------------------------ */
/* Register access from the host. */

static int
sim_read_reg(struct mhi_controller *mhi __unused, void __iomem *addr,
    u32 *out)
{
	size_t off;

	off = (uint8_t *)addr - sim->regbase;
	if (off >= SIM_REGLEN || (off & 3) != 0) {
		SIMERR("read of register %#zx", off);
		*out = 0;
		return (0);
	}
	mtx_lock(&sim->lock);
	*out = sim_reg(off);
	mtx_unlock(&sim->lock);
	return (0);
}

static void
sim_write_reg(struct mhi_controller *mhi __unused, void __iomem *addr,
    u32 val)
{
	size_t off;
	int n;

	off = (uint8_t *)addr - sim->regbase;
	if (off >= SIM_REGLEN || (off & 3) != 0) {
		SIMERR("write of register %#zx", off);
		return;
	}
	mtx_lock(&sim->lock);
	switch (off) {
	case MHICTRL:
		sim_set_reg(off, val);
		if (val & MHICTRL_RESET_MASK) {
			taskqueue_enqueue(sim->tq, &sim->reset_task);
		} else if (FIELD_GET(MHICTRL_MHISTATE_MASK, val) ==
		    MHI_STATE_M0) {
			if ((sim_reg(MHISTATUS) & MHISTATUS_READY_MASK) == 0)
				SIMERR("M0 requested before READY");
			taskqueue_enqueue(sim->tq, &sim->m0_task);
		}
		break;
	case MHISTATUS:
	case MHIREGLEN:
	case CHDBOFF:
	case ERDBOFF:
	case BHIOFF:
	case BHIEOFF:
		SIMERR("write of read-only register %#zx", off);
		break;
	case CRDB_LOWER:
		sim->crdb = (uint64_t)sim_reg(CRDB_HIGHER) << 32 | val;
		sim_set_reg(off, val);
		taskqueue_enqueue(sim->tq, &sim->cmd_task);
		break;
	case SIM_BHIOFF + BHI_IMGTXDB:
		sim_set_reg(off, val);
		taskqueue_enqueue(sim->tq, &sim->bhi_task);
		break;
	case SIM_BHIEOFF + BHIE_TXVECDB_OFFS:
		sim_set_reg(off, val);
		taskqueue_enqueue(sim->tq, &sim->bhie_task);
		break;
	default:
		sim_set_reg(off, val);
		if (off >= SIM_CHDBOFF && off < SIM_CHDBOFF + 8 * SIM_NCH &&
		    ((off - SIM_CHDBOFF) & 7) == 0) {
			n = (off - SIM_CHDBOFF) / 8;
			sim->chdb[n] = (uint64_t)sim_reg(off + 4) << 32 | val;
			taskqueue_enqueue(sim->tq, &sim->ch_task);
		} else if (off >= SIM_ERDBOFF &&
		    off < SIM_ERDBOFF + 8 * SIM_NER) {
			/* Event ring doorbells: more room; nothing to do. */
		}
		break;
	}
	mtx_unlock(&sim->lock);
}

/* ------------------------------------------------------------------ */
/* The host side: a QMI client like ath11k's. */

struct sim_req {
	uint8_t		enable_valid;
	uint8_t		enable;
	uint8_t		cookie_valid;
	uint32_t	cookie;
};

static const struct qmi_elem_info sim_req_ei[] = {
	{
		.data_type = QMI_OPT_FLAG, .elem_len = 1,
		.elem_size = sizeof(uint8_t), .array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct sim_req, enable_valid),
	},
	{
		.data_type = QMI_UNSIGNED_1_BYTE, .elem_len = 1,
		.elem_size = sizeof(uint8_t), .array_type = NO_ARRAY,
		.tlv_type = 0x10, .offset = offsetof(struct sim_req, enable),
	},
	{
		.data_type = QMI_OPT_FLAG, .elem_len = 1,
		.elem_size = sizeof(uint8_t), .array_type = NO_ARRAY,
		.tlv_type = 0x11,
		.offset = offsetof(struct sim_req, cookie_valid),
	},
	{
		.data_type = QMI_UNSIGNED_4_BYTE, .elem_len = 1,
		.elem_size = sizeof(uint32_t), .array_type = NO_ARRAY,
		.tlv_type = 0x11, .offset = offsetof(struct sim_req, cookie),
	},
	{ .data_type = QMI_EOTI, .array_type = NO_ARRAY,
	  .tlv_type = QMI_COMMON_TLV_TYPE },
};

struct sim_resp {
	struct qmi_response_type_v01	resp;
};

static const struct qmi_elem_info sim_resp_ei[] = {
	{
		.data_type = QMI_STRUCT, .elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY, .tlv_type = 0x02,
		.offset = offsetof(struct sim_resp, resp),
		.ei_array = qmi_response_type_v01_ei,
	},
	{ .data_type = QMI_EOTI, .array_type = NO_ARRAY,
	  .tlv_type = QMI_COMMON_TLV_TYPE },
};

struct sim_ind {
	uint8_t		value_valid;
	uint32_t	value;
};

static const struct qmi_elem_info sim_ind_ei[] = {
	{
		.data_type = QMI_OPT_FLAG, .elem_len = 1,
		.elem_size = sizeof(uint8_t), .array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct sim_ind, value_valid),
	},
	{
		.data_type = QMI_UNSIGNED_4_BYTE, .elem_len = 1,
		.elem_size = sizeof(uint32_t), .array_type = NO_ARRAY,
		.tlv_type = 0x10, .offset = offsetof(struct sim_ind, value),
	},
	{ .data_type = QMI_EOTI, .array_type = NO_ARRAY,
	  .tlv_type = QMI_COMMON_TLV_TYPE },
};

struct sim_client {
	struct qmi_handle	handle;
	struct sockaddr_qrtr	sq;
	struct completion	arrived;
	struct completion	indicated;
	struct completion	gone;
	uint32_t		ind_value;
	bool			in_worker;
};

static int
sim_new_server(struct qmi_handle *qmi, struct qmi_service *svc)
{
	struct sim_client *c = container_of(qmi, struct sim_client, handle);
	int error;

	SIMLOG("client: new server service %#x version %u instance %u at "
	    "%u:%#x", svc->service, svc->version, svc->instance, svc->node,
	    svc->port);
	if (svc->service != SIM_SERVICE || svc->version != SIM_VERSION ||
	    svc->instance != SIM_INSTANCE || svc->node != SIM_NODE ||
	    svc->port != SIM_PORT)
		SIMERR("client: unexpected service");
	c->sq.sq_family = AF_QIPCRTR;
	c->sq.sq_node = svc->node;
	c->sq.sq_port = svc->port;
	/* As ath11k does, from the callback. */
	error = kernel_connect(qmi->sock, (struct sockaddr_unsized *)&c->sq,
	    sizeof(c->sq), 0);
	if (error != 0)
		SIMERR("client: kernel_connect %d", error);
	complete(&c->arrived);
	return (0);
}

static void
sim_del_server(struct qmi_handle *qmi, struct qmi_service *svc)
{
	struct sim_client *c = container_of(qmi, struct sim_client, handle);

	SIMLOG("client: server at %u:%#x gone", svc->node, svc->port);
	complete(&c->gone);
}

static void
sim_ind_cb(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
    struct qmi_txn *txn, const void *data)
{
	struct sim_client *c = container_of(qmi, struct sim_client, handle);
	const struct sim_ind *ind = data;

	SIMLOG("client: indication from %u:%#x valid %u value %#x",
	    sq->sq_node, sq->sq_port, ind->value_valid, ind->value);
	if (!ind->value_valid || ind->value != SIM_IND_VALUE)
		SIMERR("client: indication decoded wrong");
	c->ind_value = ind->value;
	complete(&c->indicated);
}

static const struct qmi_ops sim_ops = {
	.new_server = sim_new_server,
	.del_server = sim_del_server,
};

static const struct qmi_msg_handler sim_handlers[] = {
	{
		.type = QMI_INDICATION,
		.msg_id = SIM_IND_ID,
		.ei = sim_ind_ei,
		.decoded_size = sizeof(struct sim_ind),
		.fn = sim_ind_cb,
	},
	{}
};

/* Copy of ath11k's QCA6390/WCN6855 MHI configuration. */
static const struct mhi_channel_config sim_channels[] = {
	{
		.num = 20, .name = "IPCR", .num_elements = 64,
		.event_ring = 1, .dir = DMA_TO_DEVICE, .ee_mask = 0x4,
		.pollcfg = 0, .doorbell = MHI_DB_BRST_DISABLE,
		.lpm_notify = false, .offload_channel = false,
		.doorbell_mode_switch = false,
	},
	{
		.num = 21, .name = "IPCR", .num_elements = 64,
		.event_ring = 1, .dir = DMA_FROM_DEVICE, .ee_mask = 0x4,
		.pollcfg = 0, .doorbell = MHI_DB_BRST_DISABLE,
		.lpm_notify = false, .offload_channel = false,
		.doorbell_mode_switch = false,
	},
};

static struct mhi_event_config sim_events[] = {
	{
		.num_elements = 32, .irq_moderation_ms = 0, .irq = 1,
		.mode = MHI_DB_BRST_DISABLE, .data_type = MHI_ER_CTRL,
		.hardware_event = false, .client_managed = false,
		.offload_channel = false,
	},
	{
		.num_elements = 256, .irq_moderation_ms = 1, .irq = 2,
		.mode = MHI_DB_BRST_DISABLE, .priority = 1,
		.hardware_event = false, .client_managed = false,
		.offload_channel = false,
	},
};

static const struct mhi_controller_config sim_config = {
	.max_channels = 128,
	.timeout_ms = 2000,
	.use_bounce_buf = false,
	.buf_len = 8192,
	.num_channels = nitems(sim_channels),
	.ch_cfg = sim_channels,
	.num_events = nitems(sim_events),
	.event_cfg = sim_events,
};

static int
sim_runtime_get(struct mhi_controller *mhi __unused)
{
	return (0);
}

static void
sim_runtime_put(struct mhi_controller *mhi __unused)
{
}

static void
sim_status_cb(struct mhi_controller *mhi __unused, enum mhi_callback cb)
{
	SIMLOG("status callback %d", cb);
}

/* ------------------------------------------------------------------ */

static int
sim_setup(void)
{
	struct linux_dma_priv *priv;
	size_t i;
	int error;

	sim = malloc(sizeof(*sim), M_MHISIM, M_WAITOK | M_ZERO);
	mtx_init(&sim->lock, "mhisim state", NULL, MTX_DEF);
	STAILQ_INIT(&sim->dlq);
	sim->regbase = malloc(SIM_REGLEN, M_MHISIM, M_WAITOK | M_ZERO);

	sim->tq = taskqueue_create("mhisim", M_WAITOK,
	    taskqueue_thread_enqueue, &sim->tq);
	taskqueue_start_threads(&sim->tq, 1, PI_NET, "mhisim device");
	TASK_INIT(&sim->bhi_task, 0, sim_bhi_task, NULL);
	TASK_INIT(&sim->bhie_task, 0, sim_bhie_task, NULL);
	TASK_INIT(&sim->m0_task, 0, sim_m0_task, NULL);
	TASK_INIT(&sim->cmd_task, 0, sim_cmd_task, NULL);
	TASK_INIT(&sim->ch_task, 0, sim_ch_task, NULL);
	TASK_INIT(&sim->reset_task, 0, sim_reset_task, NULL);

	/* Power-on state: PBL, MHI RESET. */
	sim_set_reg(MHIREGLEN, SIM_REGLEN);
	sim_set_reg(MHIVER, 0x01000000);
	sim_set_reg(MHICFG, FIELD_PREP(MHICFG_NER_MASK, SIM_NER) |
	    FIELD_PREP(MHICFG_NCH_MASK, SIM_NCH - 1));
	sim_set_reg(CHDBOFF, SIM_CHDBOFF);
	sim_set_reg(ERDBOFF, SIM_ERDBOFF);
	sim_set_reg(BHIOFF, SIM_BHIOFF);
	sim_set_reg(BHIEOFF, SIM_BHIEOFF);
	sim_set_reg(SIM_BHIOFF + BHI_EXECENV, MHI_EE_PBL);
	sim_set_reg(SIM_BHIOFF + BHI_SERIALNU, 0x5ea1);
	sim_set_state(MHI_STATE_RESET, false);
	sim->ee = MHI_EE_PBL;

	/* The firmware image, through firmware(9) as ath11k's would be. */
	sim->fw = malloc(SIM_FW_SIZE, M_MHISIM, M_WAITOK);
	for (i = 0; i < SIM_FW_SIZE; i++)
		sim->fw[i] = sim_fw_byte(i);
	sim->fwreg = firmware_register(SIM_FW_NAME, sim->fw, SIM_FW_SIZE, 1,
	    NULL);
	if (sim->fwreg == NULL)
		return (EEXIST);

	/* A device for DMA and messages. */
	bus_topo_lock();
	sim->bsddev = device_add_child(root_bus, "mhisim", DEVICE_UNIT_ANY);
	bus_topo_unlock();
	if (sim->bsddev == NULL)
		return (ENXIO);
	sim->pdev = kzalloc(sizeof(*sim->pdev), GFP_KERNEL);
	sim->pdev->dev.bsddev = sim->bsddev;
	INIT_LIST_HEAD(&sim->pdev->dev.irqents);
	spin_lock_init(&sim->pdev->dev.devres_lock);
	INIT_LIST_HEAD(&sim->pdev->dev.devres_head);
	priv = malloc(sizeof(*priv), M_MHISIM, M_WAITOK | M_ZERO);
	mtx_init(&priv->lock, "lkpi-priv-dma", NULL, MTX_DEF);
	pctrie_init(&priv->ptree);
	sim->pdev->dev.dma_priv = priv;
	/* The masks ath11k uses. */
	error = -dma_set_mask(&sim->pdev->dev, DMA_BIT_MASK(36));
	if (error == 0)
		error = -dma_set_coherent_mask(&sim->pdev->dev,
		    DMA_BIT_MASK(32));
	return (error);
}

static void
sim_teardown(void)
{
	struct linux_dma_priv *priv;
	struct sim_msg *m;

	if (sim == NULL)
		return;
	if (sim->tq != NULL) {
		taskqueue_drain_all(sim->tq);
		taskqueue_free(sim->tq);
	}
	if (sim->pdev != NULL) {
		priv = sim->pdev->dev.dma_priv;
		if (priv != NULL) {
			if (priv->dmat != NULL)
				bus_dma_tag_destroy(priv->dmat);
			if (priv->dmat_coherent != NULL)
				bus_dma_tag_destroy(priv->dmat_coherent);
			mtx_destroy(&priv->lock);
			free(priv, M_MHISIM);
		}
		kfree(sim->pdev);
	}
	if (sim->bsddev != NULL) {
		bus_topo_lock();
		device_delete_child(root_bus, sim->bsddev);
		bus_topo_unlock();
	}
	if (sim->fwreg != NULL)
		firmware_unregister(SIM_FW_NAME);
	free(sim->fw, M_MHISIM);
	while ((m = STAILQ_FIRST(&sim->dlq)) != NULL) {
		STAILQ_REMOVE_HEAD(&sim->dlq, link);
		free(m, M_MHISIM);
	}
	free(sim->regbase, M_MHISIM);
	mtx_destroy(&sim->lock);
	free(sim, M_MHISIM);
	sim = NULL;
}

static int
sim_run(void)
{
	struct mhi_controller *mhi;
	struct sim_client *c;
	struct sim_req req;
	struct sim_resp resp;
	struct qmi_txn txn;
	int error, i, ret;
	bool registered, prepared, powered, handle;

	registered = prepared = powered = handle = false;
	c = NULL;
	error = sim_setup();
	if (error != 0) {
		SIMERR("setup failed: %d", error);
		goto out;
	}

	mhi = mhi_alloc_controller();
	sim->mhi = mhi;
	mhi->cntrl_dev = &sim->pdev->dev;
	mhi->regs = sim->regbase;
	mhi->reg_len = SIM_REGLEN;
	for (i = 0; i < SIM_IRQS; i++)
		sim->irqs[i] = MHI_FBSD_TEST_IRQ_BASE + i;
	mhi->irq = sim->irqs;
	mhi->nr_irqs = SIM_IRQS;
	mhi->fw_image = SIM_FW_NAME;
	mhi->iova_start = 0;
	mhi->iova_stop = DMA_BIT_MASK(36);
	mhi->sbl_size = SIM_SBL_SIZE;
	mhi->seg_len = SIM_SEG_LEN;
	mhi->fbc_download = true;
	mhi->runtime_get = sim_runtime_get;
	mhi->runtime_put = sim_runtime_put;
	mhi->status_cb = sim_status_cb;
	mhi->read_reg = sim_read_reg;
	mhi->write_reg = sim_write_reg;

	ret = mhi_register_controller(mhi, &sim_config);
	if (ret != 0) {
		SIMERR("mhi_register_controller: %d", ret);
		goto out;
	}
	registered = true;

	/* The client looks the service up before the device boots, as
	 * ath11k does. */
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	init_completion(&c->arrived);
	init_completion(&c->indicated);
	init_completion(&c->gone);
	ret = qmi_handle_init(&c->handle, 64, &sim_ops, sim_handlers);
	if (ret != 0) {
		SIMERR("qmi_handle_init: %d", ret);
		goto out;
	}
	handle = true;
	ret = qmi_add_lookup(&c->handle, SIM_SERVICE, SIM_VERSION,
	    SIM_INSTANCE);
	if (ret != 0) {
		SIMERR("qmi_add_lookup: %d", ret);
		goto out;
	}

	mhi->timeout_ms = 5000;
	ret = mhi_prepare_for_power_up(mhi);
	if (ret != 0) {
		SIMERR("mhi_prepare_for_power_up: %d", ret);
		goto out;
	}
	prepared = true;
	ret = mhi_sync_power_up(mhi);
	if (ret != 0) {
		SIMERR("mhi_sync_power_up: %d", ret);
		goto out;
	}
	powered = true;
	SIMLOG("host: mission mode reached");

	if (wait_for_completion_timeout(&c->arrived,
	    msecs_to_jiffies(5000)) == 0) {
		SIMERR("client: service never arrived");
		goto out;
	}

	memset(&req, 0, sizeof(req));
	req.enable_valid = 1;
	req.enable = 1;
	req.cookie_valid = 1;
	req.cookie = 0x12345678;
	memset(&resp, 0, sizeof(resp));
	ret = qmi_txn_init(&c->handle, &txn, sim_resp_ei, &resp);
	if (ret < 0) {
		SIMERR("qmi_txn_init: %d", ret);
		goto out;
	}
	/* NULL destination: the connected one, as ath11k sends. */
	ret = qmi_send_request(&c->handle, NULL, &txn, SIM_REQ_ID, 64,
	    sim_req_ei, &req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		SIMERR("qmi_send_request: %d", ret);
		goto out;
	}
	ret = qmi_txn_wait(&txn, msecs_to_jiffies(5000));
	if (ret < 0) {
		SIMERR("qmi_txn_wait: %d", ret);
		goto out;
	}
	if (resp.resp.result != QMI_RESULT_SUCCESS_V01)
		SIMERR("client: response result %u error %u",
		    resp.resp.result, resp.resp.error);
	else
		SIMLOG("client: response ok");
	if (wait_for_completion_timeout(&c->indicated,
	    msecs_to_jiffies(5000)) == 0)
		SIMERR("client: no indication");
	/* The confirmation asked with the indication. */
	for (i = 0; i < 50 && !sim->resumed; i++)
		pause("mhisim", hz / 50);
	if (!sim->resumed)
		SIMERR("QRTR: no resume_tx for confirm_rx");

out:
	if (powered || prepared) {
		if (powered)
			mhi_power_down(mhi, true);
		mhi_unprepare_after_power_down(mhi);
		/* The endpoint went away: the service must be gone. */
		if (powered && handle && wait_for_completion_timeout(&c->gone,
		    msecs_to_jiffies(2000)) == 0)
			SIMERR("client: del_server not reported");
	}
	if (handle)
		qmi_handle_release(&c->handle);
	kfree(c);
	if (registered) {
		mhi_unregister_controller(mhi);
		mhi_free_controller(mhi);
	}
	if (sim != NULL) {
		if (!sim->bhi_ok)
			SIMERR("BHI not done");
		if (!sim->bhie_ok)
			SIMERR("BHIe not done");
		if (!sim->greeted)
			SIMERR("QRTR: host never said hello");
		if (!sim->req_ok)
			SIMERR("QMI: request not received as expected");
		error = sim->errors;
	}
	snprintf(sim_result, sizeof(sim_result), "%s: %d error(s)",
	    error == 0 ? "PASS" : "FAIL", error);
	SIMLOG("%s", sim_result);
	sim_teardown();
	return (error);
}

static int
sim_sysctl_run(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	val = 0;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL || val == 0)
		return (error);
	sx_xlock(&sim_sx);
	error = sim_run();
	sx_xunlock(&sim_sx);
	return (error == 0 ? 0 : EIO);
}

static SYSCTL_NODE(_debug, OID_AUTO, mhisim, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "MHI device model");
SYSCTL_PROC(_debug_mhisim, OID_AUTO, run,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, sim_sysctl_run, "I",
    "Run the power up, QMI exchange and power down test");
SYSCTL_STRING(_debug_mhisim, OID_AUTO, result, CTLFLAG_RD, sim_result, 0,
    "Result of the last run");

static int
sim_modevent(module_t mod __unused, int type, void *data __unused)
{
	switch (type) {
	case MOD_LOAD:
	case MOD_UNLOAD:
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t sim_mod = { "mhi_sim", sim_modevent, NULL };
DECLARE_MODULE(mhi_sim, sim_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(mhi_sim, 1);
MODULE_DEPEND(mhi_sim, linuxkpi, 1, 1, 1);
MODULE_DEPEND(mhi_sim, linuxkpi_mhi, 1, 1, 1);
