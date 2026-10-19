/*	$OpenBSD: if_qwx_pci.c,v 1.41 2026/10/17 08:00:40 stsp Exp $	*/

/*
 * Copyright 2023 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Copyright (c) 2021-2022, Qualcomm Innovation Center, Inc.
 * Copyright (c) 2018-2021 The Linux Foundation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of [Owner Organization] nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define PCIE_TXVECDB				0x360
#define PCIE_TXVECSTATUS			0x368
#define PCIE_RXVECDB				0x394
#define PCIE_RXVECSTATUS			0x39C

#define MHI_CHAN_CTX_CHSTATE_MASK		GENMASK(7, 0)
#define   MHI_CHAN_CTX_CHSTATE_DISABLED		0
#define   MHI_CHAN_CTX_CHSTATE_ENABLED		1
#define   MHI_CHAN_CTX_CHSTATE_RUNNING		2
#define   MHI_CHAN_CTX_CHSTATE_SUSPENDED	3
#define   MHI_CHAN_CTX_CHSTATE_STOP		4
#define   MHI_CHAN_CTX_CHSTATE_ERROR		5
#define MHI_CHAN_CTX_BRSTMODE_MASK		GENMASK(9, 8)
#define MHI_CHAN_CTX_BRSTMODE_SHFT		8
#define   MHI_CHAN_CTX_BRSTMODE_DISABLE		2
#define   MHI_CHAN_CTX_BRSTMODE_ENABLE		3
#define MHI_CHAN_CTX_POLLCFG_MASK		GENMASK(15, 10)
#define MHI_CHAN_CTX_RESERVED_MASK		GENMASK(31, 16)

#define QWX_MHI_CONFIG_QCA6390_MAX_CHANNELS	128
#define QWX_MHI_CONFIG_QCA6390_TIMEOUT_MS	2000
#define QWX_MHI_CONFIG_QCA9074_MAX_CHANNELS	30

#define MHI_CHAN_TYPE_INVALID		0
#define MHI_CHAN_TYPE_OUTBOUND		1 /* to device */
#define MHI_CHAN_TYPE_INBOUND		2 /* from device */
#define MHI_CHAN_TYPE_INBOUND_COALESCED	3

#define MHI_EV_CTX_RESERVED_MASK	GENMASK(7, 0)
#define MHI_EV_CTX_INTMODC_MASK		GENMASK(15, 8)
#define MHI_EV_CTX_INTMODT_MASK		GENMASK(31, 16)
#define MHI_EV_CTX_INTMODT_SHFT		16

#define MHI_ER_TYPE_INVALID	0
#define MHI_ER_TYPE_VALID	1

#define MHI_ER_DATA	0
#define MHI_ER_CTRL	1

#define MHI_CH_STATE_DISABLED	0
#define MHI_CH_STATE_ENABLED	1
#define MHI_CH_STATE_RUNNING	2
#define MHI_CH_STATE_SUSPENDED	3
#define MHI_CH_STATE_STOP	4
#define MHI_CH_STATE_ERROR	5

#define QWX_NUM_EVENT_CTX	2

/* Event context. Shared with device. */
struct qwx_mhi_event_ctxt {
	uint32_t intmod;
	uint32_t ertype;
	uint32_t msivec;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

/* Channel context. Shared with device. */
struct qwx_mhi_chan_ctxt {
	uint32_t chcfg;
	uint32_t chtype;
	uint32_t erindex;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

/* Command context. Shared with device. */
struct qwx_mhi_cmd_ctxt {
	uint32_t reserved0;
	uint32_t reserved1;
	uint32_t reserved2;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

struct qwx_mhi_ring_element {
	uint64_t ptr;
	uint32_t dword[2];
};

struct qwx_xfer_data {
	bus_dmamap_t	map;
	struct mbuf	*m;
};

#define QWX_PCI_XFER_MAX_DATA_SIZE	0xffff
#define QWX_PCI_XFER_RING_MAX_ELEMENTS	64

struct qwx_pci_xfer_ring {
	struct qwx_dmamem	*dmamem;
	bus_size_t		size;
	uint32_t		mhi_chan_id;
	uint32_t		mhi_chan_state;
	uint32_t		mhi_chan_direction;
	uint32_t		mhi_chan_event_ring_index;
	uint32_t		db_addr;
	uint32_t		cmd_status;
	int			num_elements;
	int			queued;
	struct qwx_xfer_data	data[QWX_PCI_XFER_RING_MAX_ELEMENTS];
	uint64_t		rp;
	uint64_t		wp;
	struct qwx_mhi_chan_ctxt *chan_ctxt;
};


#define QWX_PCI_EVENT_RING_MAX_ELEMENTS	256

struct qwx_pci_event_ring {
	struct qwx_dmamem	*dmamem;
	bus_size_t		size;
	uint32_t		mhi_er_type;
	uint32_t		mhi_er_irq;
	uint32_t		mhi_er_irq_moderation_ms;
	uint32_t		db_addr;
	int			num_elements;
	uint64_t		rp;
	uint64_t		wp;
	struct qwx_mhi_event_ctxt *event_ctxt;
};

struct qwx_cmd_data {
	bus_dmamap_t	map;
	struct mbuf	*m;
};

#define QWX_PCI_CMD_RING_MAX_ELEMENTS	128

struct qwx_pci_cmd_ring {
	struct qwx_dmamem	*dmamem;
	bus_size_t		size;
	uint64_t		rp;
	uint64_t		wp;
	int			num_elements;
	int			queued;
};

struct qwx_pci_ops;
struct qwx_msi_config;

#define QWX_NUM_MSI_VEC	32

struct qwx_pci_softc {
	struct qwx_softc	sc_sc;
	pci_chipset_tag_t	sc_pc;
	pcitag_t		sc_tag;
	int			sc_cap_off;
	int			sc_msi_off;
	pcireg_t		sc_msi_cap;
	void			*sc_ih[QWX_NUM_MSI_VEC];
	char			sc_ivname[QWX_NUM_MSI_VEC][16];
	struct qwx_ext_irq_grp	ext_irq_grp[ATH11K_EXT_IRQ_GRP_NUM_MAX];
	int			mhi_irq[2];
	bus_space_tag_t		sc_st;
	bus_space_handle_t	sc_sh;
	bus_addr_t		sc_map;
	bus_size_t		sc_mapsize;

	pcireg_t		sc_lcsr;
	uint32_t		sc_flags;
#define ATH11K_PCI_ASPM_RESTORE	1

	uint32_t		register_window;
	const struct qwx_pci_ops *sc_pci_ops;

	uint32_t		 bhi_off;
	uint32_t		 bhi_ee;
	uint32_t		 bhie_off;
	uint32_t		 mhi_state;
	uint32_t		 max_chan;

	uint64_t		 wake_db;

	/*
	 * DMA memory for AMSS.bin firmware image.
	 * This memory must remain available to the device until
	 * the device is powered down.
	 */
	struct qwx_dmamem	*amss_data;
	struct qwx_dmamem	*amss_vec;

	struct qwx_dmamem	 *rddm_vec;
	struct qwx_dmamem	 *rddm_data;
	int			 rddm_triggered;
	struct task		 rddm_task;
#define	QWX_RDDM_DUMP_SIZE	0x420000

	struct qwx_dmamem	*chan_ctxt;
	struct qwx_dmamem	*event_ctxt;
	struct qwx_dmamem	*cmd_ctxt;


	struct qwx_pci_xfer_ring xfer_rings[2];
#define QWX_PCI_XFER_RING_IPCR_OUTBOUND		0
#define QWX_PCI_XFER_RING_IPCR_INBOUND		1
	struct qwx_pci_event_ring event_rings[QWX_NUM_EVENT_CTX];
	struct qwx_pci_cmd_ring cmd_ring;
};

int	qwx_pci_match(struct device *, void *, void *);
void	qwx_pci_attach(struct device *, struct device *, void *);
int	qwx_pci_detach(struct device *, int);
void	qwx_pci_attach_hook(struct device *);
void	qwx_pci_free_xfer_rings(struct qwx_pci_softc *);
int	qwx_pci_alloc_xfer_ring(struct qwx_softc *, struct qwx_pci_xfer_ring *,
	    uint32_t, uint32_t, uint32_t, size_t);
int	qwx_pci_alloc_xfer_rings_qca6390(struct qwx_pci_softc *);
int	qwx_pci_alloc_xfer_rings_qcn9074(struct qwx_pci_softc *);
void	qwx_pci_free_event_rings(struct qwx_pci_softc *);
int	qwx_pci_alloc_event_ring(struct qwx_softc *,
	    struct qwx_pci_event_ring *, uint32_t, uint32_t, uint32_t, size_t);
int	qwx_pci_alloc_event_rings(struct qwx_pci_softc *);
void	qwx_pci_free_cmd_ring(struct qwx_pci_softc *);
int	qwx_pci_init_cmd_ring(struct qwx_softc *, struct qwx_pci_cmd_ring *);
uint32_t qwx_pci_read(struct qwx_softc *, uint32_t);
void	qwx_pci_write(struct qwx_softc *, uint32_t, uint32_t);

void	qwx_pci_read_hw_version(struct qwx_softc *, uint32_t *, uint32_t *);
uint32_t qwx_pcic_read32(struct qwx_softc *, uint32_t);
void	 qwx_pcic_write32(struct qwx_softc *, uint32_t, uint32_t);

void	qwx_pcic_ext_irq_enable(struct qwx_softc *);
void	qwx_pcic_ext_irq_disable(struct qwx_softc *);
int	qwx_pcic_config_irq(struct qwx_softc *, struct pci_attach_args *);

int	qwx_pci_start(struct qwx_softc *);
void	qwx_pci_stop(struct qwx_softc *);
void	qwx_pci_aspm_disable(struct qwx_softc *);
void	qwx_pci_aspm_restore(struct qwx_softc *);
int	qwx_pci_power_up(struct qwx_softc *);
void	qwx_pci_power_down(struct qwx_softc *);

int	qwx_pci_bus_wake_up(struct qwx_softc *);
void	qwx_pci_bus_release(struct qwx_softc *);
void	qwx_pci_window_write32(struct qwx_softc *, uint32_t, uint32_t);
uint32_t qwx_pci_window_read32(struct qwx_softc *, uint32_t);

int	qwx_mhi_register(struct qwx_softc *);
void	qwx_mhi_unregister(struct qwx_softc *);
void	qwx_mhi_ring_doorbell(struct qwx_softc *sc, uint64_t, uint64_t);
void	qwx_mhi_device_wake(struct qwx_softc *);
void	qwx_mhi_device_zzz(struct qwx_softc *);
int	qwx_mhi_wake_db_clear_valid(struct qwx_softc *);
void	qwx_mhi_init_xfer_rings(struct qwx_pci_softc *);
void	qwx_mhi_init_event_rings(struct qwx_pci_softc *);
void	qwx_mhi_init_cmd_ring(struct qwx_pci_softc *);
void	qwx_mhi_init_dev_ctxt(struct qwx_pci_softc *);
int	qwx_mhi_send_cmd(struct qwx_pci_softc *psc, uint32_t, uint32_t);
void *	qwx_pci_xfer_ring_get_elem(struct qwx_pci_xfer_ring *, uint64_t);
struct qwx_xfer_data *qwx_pci_xfer_ring_get_data(struct qwx_pci_xfer_ring *,
	    uint64_t);
int	qwx_mhi_submit_xfer(struct qwx_softc *sc, struct mbuf *m);
int	qwx_mhi_start_channel(struct qwx_pci_softc *,
	    struct qwx_pci_xfer_ring *);
int	qwx_mhi_start_channels(struct qwx_pci_softc *);
int	qwx_mhi_start(struct qwx_pci_softc *);
void	qwx_mhi_stop(struct qwx_softc *);
int	qwx_mhi_reset_device(struct qwx_softc *, int);
void	qwx_mhi_clear_vector(struct qwx_softc *);
int	qwx_mhi_fw_load_handler(struct qwx_pci_softc *);
int	qwx_mhi_await_device_reset(struct qwx_softc *);
int	qwx_mhi_await_device_ready(struct qwx_softc *);
void	qwx_mhi_ready_state_transition(struct qwx_pci_softc *);
void	qwx_mhi_mission_mode_state_transition(struct qwx_pci_softc *);
void	qwx_mhi_low_power_mode_state_transition(struct qwx_pci_softc *);
void	qwx_mhi_set_state(struct qwx_softc *, uint32_t);
void	qwx_mhi_init_mmio(struct qwx_pci_softc *);
int	qwx_mhi_fw_load_bhi(struct qwx_pci_softc *, uint8_t *, size_t);
int	qwx_mhi_fw_load_bhie(struct qwx_pci_softc *, uint8_t *, size_t);
void	qwx_rddm_prepare(struct qwx_pci_softc *);
#ifdef QWX_DEBUG
void	qwx_rddm_task(void *);
#endif
void *	qwx_pci_event_ring_get_elem(struct qwx_pci_event_ring *, uint64_t);
void	qwx_pci_intr_ctrl_event_mhi(struct qwx_pci_softc *, uint32_t);
void	qwx_pci_intr_ctrl_event_ee(struct qwx_pci_softc *, uint32_t);
void	qwx_pci_intr_ctrl_event_cmd_complete(struct qwx_pci_softc *,
	    uint64_t, uint32_t);
int	qwx_pci_intr_ctrl_event(struct qwx_pci_softc *,
	    struct qwx_pci_event_ring *, int);
void	qwx_pci_intr_data_event_tx(struct qwx_pci_softc *,
	    struct qwx_mhi_ring_element *);
int	qwx_pci_intr_data_event(struct qwx_pci_softc *,
	    struct qwx_pci_event_ring *, int);
int	qwx_pci_intr_mhi_ctrl(void *);
int	qwx_pci_intr_mhi_data(void *);
int	qwx_pci_intr(void *);

struct qwx_pci_ops {

qwx_pci_free_xfer_rings(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int i;

	for (i = 0; i < nitems(psc->xfer_rings); i++) {
		struct qwx_pci_xfer_ring *ring = &psc->xfer_rings[i];
		if (ring->dmamem) {
			qwx_dmamem_free(sc->sc_dmat, ring->dmamem);
			ring->dmamem = NULL;
		}
		memset(ring, 0, sizeof(*ring));
	}
}

int
qwx_pci_alloc_xfer_ring(struct qwx_softc *sc, struct qwx_pci_xfer_ring *ring,
    uint32_t id, uint32_t direction, uint32_t event_ring_index,
    size_t num_elements)
{
	bus_size_t size;
	int i, err;

	memset(ring, 0, sizeof(*ring));

	size = sizeof(struct qwx_mhi_ring_element) * num_elements;
	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = qwx_dmamem_alloc(sc->sc_dmat, size, size);
	if (ring->dmamem == NULL)
		return ENOMEM;

	ring->size = size;
	ring->mhi_chan_id = id;
	ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
	ring->mhi_chan_direction = direction;
	ring->mhi_chan_event_ring_index = event_ring_index;
	ring->num_elements = num_elements;

	memset(ring->data, 0, sizeof(ring->data));
	for (i = 0; i < ring->num_elements; i++) {
		struct qwx_xfer_data *xfer = &ring->data[i];
		
		err = bus_dmamap_create(sc->sc_dmat, QWX_PCI_XFER_MAX_DATA_SIZE,
		    1, QWX_PCI_XFER_MAX_DATA_SIZE, 0, BUS_DMA_NOWAIT,
		    &xfer->map);
		if (err) {
			printf("%s: could not create xfer DMA map\n",
			    sc->sc_dev.dv_xname);
			goto fail;
		}

		if (direction == MHI_CHAN_TYPE_INBOUND) {
			struct mbuf *m;

			m = m_gethdr(M_DONTWAIT, MT_DATA);
			if (m == NULL) {
				err = ENOBUFS;
				goto fail;
			}

			MCLGETL(m, M_DONTWAIT, QWX_PCI_XFER_MAX_DATA_SIZE);
			if ((m->m_flags & M_EXT) == 0) {
				m_freem(m);
				err = ENOBUFS;
				goto fail;
			}

			m->m_len = m->m_pkthdr.len = QWX_PCI_XFER_MAX_DATA_SIZE;
			err = bus_dmamap_load_mbuf(sc->sc_dmat, xfer->map,
			    m, BUS_DMA_READ | BUS_DMA_NOWAIT);
			if (err) {
				printf("%s: can't map mbuf (error %d)\n",
				    sc->sc_dev.dv_xname, err);
				m_freem(m);
				goto fail;
			}

			bus_dmamap_sync(sc->sc_dmat, xfer->map, 0,
			    QWX_PCI_XFER_MAX_DATA_SIZE, BUS_DMASYNC_PREREAD);
			xfer->m = m;
		}
	}

	return 0;
fail:
	for (i = 0; i < ring->num_elements; i++) {
		struct qwx_xfer_data *xfer = &ring->data[i];

		if (xfer->map) {
			bus_dmamap_sync(sc->sc_dmat, xfer->map, 0,
			    xfer->map->dm_mapsize, BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->sc_dmat, xfer->map);
			bus_dmamap_destroy(sc->sc_dmat, xfer->map);
			xfer->map = NULL;
		}

		if (xfer->m) {
			m_freem(xfer->m);
			xfer->m = NULL;
		}
	}
	return 1;
}

int
qwx_pci_alloc_xfer_rings_qca6390(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret;

	ret = qwx_pci_alloc_xfer_ring(sc,
	    &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_OUTBOUND],
	    20, MHI_CHAN_TYPE_OUTBOUND, 1, 64);
	if (ret)
		goto fail;

	ret = qwx_pci_alloc_xfer_ring(sc,
	    &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_INBOUND],
	    21, MHI_CHAN_TYPE_INBOUND, 1, 64);
	if (ret)
		goto fail;

	return 0;
fail:
	qwx_pci_free_xfer_rings(psc);
	return ret;
}

int
qwx_pci_alloc_xfer_rings_qcn9074(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret;

	ret = qwx_pci_alloc_xfer_ring(sc,
	    &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_OUTBOUND],
	    20, MHI_CHAN_TYPE_OUTBOUND, 1, 32);
	if (ret)
		goto fail;

	ret = qwx_pci_alloc_xfer_ring(sc,
	    &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_INBOUND],
	    21, MHI_CHAN_TYPE_INBOUND, 1, 32);
	if (ret)
		goto fail;

	return 0;
fail:
	qwx_pci_free_xfer_rings(psc);
	return ret;
}

void
qwx_pci_free_event_rings(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int i;

	for (i = 0; i < nitems(psc->event_rings); i++) {
		struct qwx_pci_event_ring *ring = &psc->event_rings[i];
		if (ring->dmamem) {
			qwx_dmamem_free(sc->sc_dmat, ring->dmamem);
			ring->dmamem = NULL;
		}
		memset(ring, 0, sizeof(*ring));
	}
}

int
qwx_pci_alloc_event_ring(struct qwx_softc *sc, struct qwx_pci_event_ring *ring,
    uint32_t type, uint32_t irq, uint32_t intmod, size_t num_elements)
{
	bus_size_t size;

	memset(ring, 0, sizeof(*ring));

	size = sizeof(struct qwx_mhi_ring_element) * num_elements;
	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = qwx_dmamem_alloc(sc->sc_dmat, size, size);
	if (ring->dmamem == NULL)
		return ENOMEM;

	ring->size = size;
	ring->mhi_er_type = type;
	ring->mhi_er_irq = irq;
	ring->mhi_er_irq_moderation_ms = intmod;
	ring->num_elements = num_elements;
	return 0;
}

int
qwx_pci_alloc_event_rings(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret;

	ret = qwx_pci_alloc_event_ring(sc, &psc->event_rings[0],
	    MHI_ER_CTRL, psc->mhi_irq[MHI_ER_CTRL], 0, 32);
	if (ret)
		goto fail;

	ret = qwx_pci_alloc_event_ring(sc, &psc->event_rings[1],
	    MHI_ER_DATA, psc->mhi_irq[MHI_ER_DATA], 1, 256);
	if (ret)
		goto fail;

	return 0;
fail:
	qwx_pci_free_event_rings(psc);
	return ret;
}

void
qwx_pci_free_cmd_ring(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_pci_cmd_ring *ring = &psc->cmd_ring;

	if (ring->dmamem)
		qwx_dmamem_free(sc->sc_dmat, ring->dmamem);

	memset(ring, 0, sizeof(*ring));
}

int
qwx_pci_init_cmd_ring(struct qwx_softc *sc, struct qwx_pci_cmd_ring *ring)
{
	memset(ring, 0, sizeof(*ring));

	ring->num_elements = QWX_PCI_CMD_RING_MAX_ELEMENTS;
	ring->size = sizeof(struct qwx_mhi_ring_element) * ring->num_elements;

	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = qwx_dmamem_alloc(sc->sc_dmat, ring->size, ring->size);
	if (ring->dmamem == NULL)
		return ENOMEM;

	return 0;
}

uint32_t
qwx_pci_read(struct qwx_softc *sc, uint32_t addr)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;

	return (bus_space_read_4(psc->sc_st, psc->sc_sh, addr));
}

void
qwx_pci_write(struct qwx_softc *sc, uint32_t addr, uint32_t val)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;

	bus_space_write_4(psc->sc_st, psc->sc_sh, addr, val);
}

void

qwx_pci_power_up(struct qwx_softc *sc)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;
	int error;

	psc->register_window = 0;
	clear_bit(ATH11K_FLAG_DEVICE_INIT_DONE, sc->sc_flags);

	qwx_pci_sw_reset(sc, true);

	/* Disable ASPM during firmware download due to problems switching
	 * to AMSS state.
	 */
	qwx_pci_aspm_disable(sc);

	qwx_pci_msi_enable(sc);

	error = qwx_mhi_start(psc);
	if (error)
		return error;

	if (sc->hw_params.static_window_map)
		qwx_pci_select_static_window(sc);

	return 0;
}

/*
 * MHI
 */
int
qwx_mhi_register(struct qwx_softc *sc)
{
	DNPRINTF(QWX_D_MHI, "%s: STUB %s()\n", sc->sc_dev.dv_xname, __func__);
	return 0;
}

void
qwx_mhi_unregister(struct qwx_softc *sc)
{
	DNPRINTF(QWX_D_MHI, "%s: STUB %s()\n", sc->sc_dev.dv_xname, __func__);
}

// XXX MHI is GPLd - we provide a compatible bare-bones implementation
#define MHI_CFG				0x10
#define   MHI_CFG_NHWER_MASK		GENMASK(31, 24)
#define   MHI_CFG_NHWER_SHFT		24
#define   MHI_CFG_NER_MASK		GENMASK(23, 16)
#define   MHI_CFG_NER_SHFT		16
#define   MHI_CFG_NHWCH_MASK		GENMASK(15, 8)
#define   MHI_CFG_NHWCH_SHFT		8
#define   MHI_CFG_NCH_MASK		GENMASK(7, 0)
#define MHI_CHDBOFF			0x18
#define MHI_DEV_WAKE_DB			127
#define MHI_ERDBOFF			0x20
#define MHI_BHI_OFFSET			0x28
#define   MHI_BHI_IMGADDR_LOW			0x08
#define   MHI_BHI_IMGADDR_HIGH			0x0c
#define   MHI_BHI_IMGSIZE			0x10
#define   MHI_BHI_IMGTXDB			0x18
#define   MHI_BHI_INTVEC			0x20
#define   MHI_BHI_EXECENV			0x28
#define   MHI_BHI_STATUS			0x2c
#define	  MHI_BHI_SERIALNU			0x40
#define MHI_BHIE_OFFSET			0x2c
#define   MHI_BHIE_TXVECADDR_LOW_OFFS		0x2c
#define   MHI_BHIE_TXVECADDR_HIGH_OFFS		0x30
#define   MHI_BHIE_TXVECSIZE_OFFS		0x34
#define   MHI_BHIE_TXVECDB_OFFS			0x3c
#define   MHI_BHIE_TXVECSTATUS_OFFS		0x44
#define   MHI_BHIE_RXVECADDR_LOW_OFFS		0x60
#define   MHI_BHIE_RXVECSTATUS_OFFS		0x78
#define MHI_CTRL			0x38
#define    MHI_CTRL_READY_MASK			0x1
#define    MHI_CTRL_RESET_MASK			0x2
#define    MHI_CTRL_MHISTATE_MASK		GENMASK(15, 8)
#define    MHI_CTRL_MHISTATE_SHFT		8
#define MHI_STATUS			0x48
#define    MHI_STATUS_MHISTATE_MASK		GENMASK(15, 8)
#define    MHI_STATUS_MHISTATE_SHFT		8
#define        MHI_STATE_RESET			0x0
#define        MHI_STATE_READY			0x1
#define        MHI_STATE_M0			0x2
#define        MHI_STATE_M1			0x3
#define        MHI_STATE_M2			0x4
#define        MHI_STATE_M3			0x5
#define        MHI_STATE_M3_FAST		0x6
#define        MHI_STATE_BHI			0x7
#define        MHI_STATE_SYS_ERR		0xff
#define    MHI_STATUS_READY_MASK		0x1
#define    MHI_STATUS_SYSERR_MASK		0x4
#define MHI_CCABAP_LOWER		0x58
#define MHI_CCABAP_HIGHER		0x5c
#define MHI_ECABAP_LOWER		0x60
#define MHI_ECABAP_HIGHER		0x64
#define MHI_CRCBAP_LOWER		0x68
#define MHI_CRCBAP_HIGHER		0x6c
#define MHI_CRDB_LOWER			0x70
#define MHI_CRDB_HIGHER			0x74
#define MHI_CTRLBASE_LOWER		0x80
#define MHI_CTRLBASE_HIGHER		0x84
#define MHI_CTRLLIMIT_LOWER		0x88
#define MHI_CTRLLIMIT_HIGHER		0x8c
#define MHI_DATABASE_LOWER		0x98
#define MHI_DATABASE_HIGHER		0x9c
#define MHI_DATALIMIT_LOWER		0xa0
#define MHI_DATALIMIT_HIGHER		0xa4

#define MHI_EE_PBL	0x0	/* Primary Bootloader */
#define MHI_EE_SBL	0x1	/* Secondary Bootloader */
#define MHI_EE_AMSS	0x2	/* Modem, aka the primary runtime EE */
#define MHI_EE_RDDM	0x3	/* Ram dump download mode */
#define MHI_EE_WFW	0x4	/* WLAN firmware mode */
#define MHI_EE_PTHRU	0x5	/* Passthrough */
#define MHI_EE_EDL	0x6	/* Embedded downloader */
#define MHI_EE_FP	0x7	/* Flash Programmer Environment */

#define MHI_IN_PBL(e) (e == MHI_EE_PBL || e == MHI_EE_PTHRU || e == MHI_EE_EDL)
#define MHI_POWER_UP_CAPABLE(e) (MHI_IN_PBL(e) || e == MHI_EE_AMSS)
#define MHI_IN_MISSION_MODE(e) \
	(e == MHI_EE_AMSS || e == MHI_EE_WFW || e == MHI_EE_FP)

/* BHI register bits */
#define MHI_BHI_TXDB_SEQNUM_BMSK	GENMASK(29, 0)
#define MHI_BHI_TXDB_SEQNUM_SHFT	0
#define MHI_BHI_STATUS_MASK		GENMASK(31, 30)
#define MHI_BHI_STATUS_SHFT		30
#define MHI_BHI_STATUS_ERROR		0x03
#define MHI_BHI_STATUS_SUCCESS		0x02
#define MHI_BHI_STATUS_RESET		0x00

/* MHI BHIE registers */
#define MHI_BHIE_MSMSOCID_OFFS		0x00
#define MHI_BHIE_RXVECADDR_LOW_OFFS	0x60
#define MHI_BHIE_RXVECADDR_HIGH_OFFS	0x64
#define MHI_BHIE_RXVECSIZE_OFFS		0x68
#define MHI_BHIE_RXVECDB_OFFS		0x70
#define MHI_BHIE_RXVECSTATUS_OFFS	0x78

/* BHIE register bits */
#define MHI_BHIE_TXVECDB_SEQNUM_BMSK		GENMASK(29, 0)
#define MHI_BHIE_TXVECDB_SEQNUM_SHFT		0
#define MHI_BHIE_TXVECSTATUS_SEQNUM_BMSK	GENMASK(29, 0)
#define MHI_BHIE_TXVECSTATUS_SEQNUM_SHFT	0
#define MHI_BHIE_TXVECSTATUS_STATUS_BMSK	GENMASK(31, 30)
#define MHI_BHIE_TXVECSTATUS_STATUS_SHFT	30
#define MHI_BHIE_TXVECSTATUS_STATUS_RESET	0x00
#define MHI_BHIE_TXVECSTATUS_STATUS_XFER_COMPL	0x02
#define MHI_BHIE_TXVECSTATUS_STATUS_ERROR	0x03
#define MHI_BHIE_RXVECDB_SEQNUM_BMSK		GENMASK(29, 0)
#define MHI_BHIE_RXVECDB_SEQNUM_SHFT		0
#define MHI_BHIE_RXVECSTATUS_SEQNUM_BMSK	GENMASK(29, 0)
#define MHI_BHIE_RXVECSTATUS_SEQNUM_SHFT	0
#define MHI_BHIE_RXVECSTATUS_STATUS_BMSK	GENMASK(31, 30)
#define MHI_BHIE_RXVECSTATUS_STATUS_SHFT	30
#define MHI_BHIE_RXVECSTATUS_STATUS_RESET	0x00
#define MHI_BHIE_RXVECSTATUS_STATUS_XFER_COMPL	0x02
#define MHI_BHIE_RXVECSTATUS_STATUS_ERROR	0x03

#define MHI_EV_CC_INVALID	0x0
#define MHI_EV_CC_SUCCESS	0x1
#define MHI_EV_CC_EOT		0x2
#define MHI_EV_CC_OVERFLOW	0x3
#define MHI_EV_CC_EOB		0x4
#define MHI_EV_CC_OOB		0x5
#define MHI_EV_CC_DB_MODE	0x6
#define MHI_EV_CC_UNDEFINED_ERR	0x10
#define MHI_EV_CC_BAD_TRE	0x11

#define MHI_CMD_NOP		01
#define MHI_CMD_RESET_CHAN	16
#define MHI_CMD_STOP_CHAN	17
#define MHI_CMD_START_CHAN	18

#define MHI_TRE_CMD_CHID_MASK	GENMASK(31, 24)
#define MHI_TRE_CMD_CHID_SHFT	24
#define MHI_TRE_CMD_CMDID_MASK	GENMASK(23, 16)
#define MHI_TRE_CMD_CMDID_SHFT	16

#define MHI_TRE0_EV_LEN_MASK	GENMASK(15, 0)
#define MHI_TRE0_EV_LEN_SHFT	0
#define MHI_TRE0_EV_CODE_MASK	GENMASK(31, 24)
#define MHI_TRE0_EV_CODE_SHFT	24
#define MHI_TRE1_EV_TYPE_MASK	GENMASK(23, 16)
#define MHI_TRE1_EV_TYPE_SHFT	16
#define MHI_TRE1_EV_CHID_MASK	GENMASK(31, 24)
#define MHI_TRE1_EV_CHID_SHFT	24

#define MHI_TRE0_DATA_LEN_MASK	GENMASK(15, 0)
#define MHI_TRE0_DATA_LEN_SHFT	0
#define MHI_TRE1_DATA_CHAIN	(1 << 0)
#define MHI_TRE1_DATA_IEOB	(1 << 8)
#define MHI_TRE1_DATA_IEOT	(1 << 9)
#define MHI_TRE1_DATA_BEI	(1 << 10)
#define MHI_TRE1_DATA_TYPE_MASK		GENMASK(23, 16)
#define MHI_TRE1_DATA_TYPE_SHIFT	16
#define MHI_TRE1_DATA_TYPE_TRANSFER	0x2

#define MHI_PKT_TYPE_INVALID			0x00
#define MHI_PKT_TYPE_NOOP_CMD			0x01
#define MHI_PKT_TYPE_TRANSFER			0x02
#define MHI_PKT_TYPE_COALESCING			0x08
#define MHI_PKT_TYPE_RESET_CHAN_CMD		0x10
#define MHI_PKT_TYPE_STOP_CHAN_CMD		0x11
#define MHI_PKT_TYPE_START_CHAN_CMD		0x12
#define MHI_PKT_TYPE_STATE_CHANGE_EVENT		0x20
#define MHI_PKT_TYPE_CMD_COMPLETION_EVENT	0x21
#define MHI_PKT_TYPE_TX_EVENT			0x22
#define MHI_PKT_TYPE_RSC_TX_EVENT		0x28
#define MHI_PKT_TYPE_EE_EVENT			0x40
#define MHI_PKT_TYPE_TSYNC_EVENT		0x48
#define MHI_PKT_TYPE_BW_REQ_EVENT		0x50


#define MHI_DMA_VEC_CHUNK_SIZE			524288 /* 512 KB */
struct qwx_dma_vec_entry {
	uint64_t paddr;
	uint64_t size;
};

int
qwx_mhi_stop_channel(struct qwx_pci_softc *psc, struct qwx_pci_xfer_ring *ring)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret = 0;

	if (ring->mhi_chan_state != MHI_CH_STATE_ENABLED)
		return 0;

	DNPRINTF(QWX_D_MHI, "%s: stop MHI channel %d in state %d\n", __func__,
	    ring->mhi_chan_id, ring->mhi_chan_state);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->chan_ctxt), 0,
	    QWX_DMA_LEN(psc->chan_ctxt), BUS_DMASYNC_PREWRITE);

	ring->cmd_status = MHI_EV_CC_INVALID;
	if (qwx_mhi_send_cmd(psc, MHI_CMD_STOP_CHAN, ring->mhi_chan_id))
		return 1;

	while (ring->cmd_status != MHI_EV_CC_SUCCESS) {
		ret = tsleep_nsec(&ring->cmd_status, 0, "qwxcmd",
		    SEC_TO_NSEC(5));
		if (ret)
			break;
	}

	if (ret) {
		printf("%s: could not stop MHI channel %d in state %d: status 0x%x\n",
		    sc->sc_dev.dv_xname, ring->mhi_chan_id,
		    ring->mhi_chan_state, ring->cmd_status);
		return 1;
	}

	ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
	return 0;
}

void
qwx_mhi_stop_channels(struct qwx_pci_softc *psc)
{
	struct qwx_pci_xfer_ring *ring;

	qwx_mhi_device_wake(&psc->sc_sc);

	ring = &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_OUTBOUND];
	qwx_mhi_stop_channel(psc, ring);

	ring = &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_INBOUND];
	qwx_mhi_stop_channel(psc, ring);

	qwx_mhi_device_zzz(&psc->sc_sc);
}

void
qwx_mhi_flush_mhi_event_rings(struct qwx_pci_softc *psc)
{
	qwx_pci_intr_ctrl_event(psc, &psc->event_rings[0], 1);
	qwx_pci_intr_data_event(psc, &psc->event_rings[1], 1);
}

void
qwx_pci_power_down(struct qwx_softc *sc)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;
	uint32_t state;
	int i;

	/* Restore ASPM in case firmware bootup fails. */
	qwx_pci_aspm_restore(sc);

	qwx_pci_force_wake(sc);

	qwx_mhi_stop_channels(psc);
	qwx_mhi_flush_mhi_event_rings(psc);

	/*
	 * Ask firmware to transition to M3 before resetting the device
	 * so it can flush state cleanly.  Otherwise stale chip RAM from
	 * the previous boot causes the next firmware boot to silently
	 * drop WMI PDEV commands.
	 */
	state = (qwx_pci_read(sc, MHI_STATUS) & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;
	if (state == MHI_STATE_M0) {
		qwx_mhi_set_state(sc, MHI_STATE_M3);
		for (i = 0; i < 100; i++) {
			state = (qwx_pci_read(sc, MHI_STATUS) &
			    MHI_STATUS_MHISTATE_MASK) >>
			    MHI_STATUS_MHISTATE_SHFT;
			if (state == MHI_STATE_M3)
				break;
			DELAY(10 * 1000);
		}
		if (state != MHI_STATE_M3)
			printf("%s: MHI M3 transition timeout (state=0x%x)\n",
			    sc->sc_dev.dv_xname, state);
	}

	qwx_pci_msi_disable(sc);

	qwx_mhi_stop(sc);
	clear_bit(ATH11K_FLAG_DEVICE_INIT_DONE, sc->sc_flags);
	qwx_pci_sw_reset(sc, false);
}

void
qwx_mhi_ring_doorbell(struct qwx_softc *sc, uint64_t db_addr, uint64_t val)
{
	qwx_pci_write(sc, db_addr + 4, val >> 32);
	qwx_pci_write(sc, db_addr, val & 0xffffffff);
}

void
qwx_mhi_device_wake(struct qwx_softc *sc)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;

	/*
	 * Device wake is async only for now because we do not
	 * keep track of PM state in software.
	 */
	qwx_mhi_ring_doorbell(sc, psc->wake_db, 1);
}

void
qwx_mhi_device_zzz(struct qwx_softc *sc)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;

	qwx_mhi_ring_doorbell(sc, psc->wake_db, 0);
}

int
qwx_mhi_wake_db_clear_valid(struct qwx_softc *sc)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;

	return (psc->mhi_state == MHI_STATE_M0); /* TODO other states? */
}

void
qwx_mhi_init_xfer_rings(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int i;
	uint32_t chcfg;
	struct qwx_pci_xfer_ring *ring;
	struct qwx_mhi_chan_ctxt *cbase, *c;

	cbase = (struct qwx_mhi_chan_ctxt *)QWX_DMA_KVA(psc->chan_ctxt);
	for (i = 0; i < psc->max_chan; i++) {
		c = &cbase[i];
		chcfg = le32toh(c->chcfg);
		chcfg &= ~(MHI_CHAN_CTX_CHSTATE_MASK |
		    MHI_CHAN_CTX_BRSTMODE_MASK |
		    MHI_CHAN_CTX_POLLCFG_MASK);
		chcfg |= (MHI_CHAN_CTX_CHSTATE_DISABLED |
		    (MHI_CHAN_CTX_BRSTMODE_DISABLE <<
		    MHI_CHAN_CTX_BRSTMODE_SHFT));
		c->chcfg = htole32(chcfg);
		c->chtype = htole32(MHI_CHAN_TYPE_INVALID);
		c->erindex = 0;
	}

	for (i = 0; i < nitems(psc->xfer_rings); i++) {
		ring = &psc->xfer_rings[i];
		KASSERT(ring->mhi_chan_id < psc->max_chan);
		c = &cbase[ring->mhi_chan_id];
		c->chtype = htole32(ring->mhi_chan_direction);
		c->erindex = htole32(ring->mhi_chan_event_ring_index);
		ring->chan_ctxt = c;
	}

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->chan_ctxt), 0,
	    QWX_DMA_LEN(psc->chan_ctxt), BUS_DMASYNC_PREWRITE);
}

void
qwx_mhi_init_event_rings(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int i;
	uint32_t intmod;
	uint64_t paddr, len;
	struct qwx_pci_event_ring *ring;
	struct qwx_mhi_event_ctxt *c;

	c = (struct qwx_mhi_event_ctxt *)QWX_DMA_KVA(psc->event_ctxt);
	for (i = 0; i < nitems(psc->event_rings); i++, c++) {
		ring = &psc->event_rings[i];

		ring->event_ctxt = c;

		intmod = le32toh(c->intmod);
		intmod &= ~(MHI_EV_CTX_INTMODC_MASK | MHI_EV_CTX_INTMODT_MASK);
		intmod |= (ring->mhi_er_irq_moderation_ms <<
		    MHI_EV_CTX_INTMODT_SHFT) & MHI_EV_CTX_INTMODT_MASK;
		c->intmod = htole32(intmod);

		c->ertype = htole32(MHI_ER_TYPE_VALID);
		c->msivec = htole32(ring->mhi_er_irq);

		paddr = QWX_DMA_DVA(ring->dmamem);
		ring->rp = paddr;
		ring->wp = paddr + ring->size -
		    sizeof(struct qwx_mhi_ring_element);
		c->rbase = htole64(paddr);
		c->rp = htole64(ring->rp);
		c->wp = htole64(ring->wp);

		len = sizeof(struct qwx_mhi_ring_element) * ring->num_elements;
		c->rlen = htole64(len);
	}

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->event_ctxt), 0,
	    QWX_DMA_LEN(psc->event_ctxt), BUS_DMASYNC_PREWRITE);
}

void
qwx_mhi_init_cmd_ring(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_pci_cmd_ring *ring = &psc->cmd_ring;
	struct qwx_mhi_cmd_ctxt *c;
	uint64_t paddr, len;

	paddr = QWX_DMA_DVA(ring->dmamem);
	len = ring->size;

	ring->rp = ring->wp = paddr;
	ring->queued = 0;

	c = (struct qwx_mhi_cmd_ctxt *)QWX_DMA_KVA(psc->cmd_ctxt);
	c->rbase = htole64(paddr);
	c->rp = htole64(paddr);
	c->wp = htole64(paddr);
	c->rlen = htole64(len);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->cmd_ctxt), 0,
	    QWX_DMA_LEN(psc->cmd_ctxt), BUS_DMASYNC_PREWRITE);
}

void
qwx_mhi_init_dev_ctxt(struct qwx_pci_softc *psc)
{
	qwx_mhi_init_xfer_rings(psc);
	qwx_mhi_init_event_rings(psc);
	qwx_mhi_init_cmd_ring(psc);
}

void *
qwx_pci_cmd_ring_get_elem(struct qwx_pci_cmd_ring *ring, uint64_t ptr)
{
	uint64_t base = QWX_DMA_DVA(ring->dmamem), offset;

	if (ptr < base || ptr >= base + ring->size)
		return NULL;

	offset = ptr - base;
	if (offset >= ring->size)
		return NULL;

	return QWX_DMA_KVA(ring->dmamem) + offset;
}

int
qwx_mhi_cmd_ring_submit(struct qwx_pci_softc *psc,
    struct qwx_pci_cmd_ring *ring)
{
	struct qwx_softc *sc = &psc->sc_sc;
	uint64_t base = QWX_DMA_DVA(ring->dmamem);
	struct qwx_mhi_cmd_ctxt *c;

	if (ring->queued >= ring->num_elements)
		return 1;

	if (ring->wp + sizeof(struct qwx_mhi_ring_element) >= base + ring->size)
		ring->wp = base;
	else
		ring->wp += sizeof(struct qwx_mhi_ring_element);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->cmd_ctxt), 0,
	    QWX_DMA_LEN(psc->cmd_ctxt), BUS_DMASYNC_POSTREAD);

	c = (struct qwx_mhi_cmd_ctxt *)QWX_DMA_KVA(psc->cmd_ctxt);
	c->wp = htole64(ring->wp);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->cmd_ctxt), 0,
	    QWX_DMA_LEN(psc->cmd_ctxt), BUS_DMASYNC_PREWRITE);

	ring->queued++;
	qwx_mhi_ring_doorbell(sc, MHI_CRDB_LOWER, ring->wp);
	return 0;
}

int
qwx_mhi_send_cmd(struct qwx_pci_softc *psc, uint32_t cmd, uint32_t chan)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_pci_cmd_ring	*ring = &psc->cmd_ring;
	struct qwx_mhi_ring_element *e;

	if (ring->queued >= ring->num_elements) {
		printf("%s: command ring overflow\n", sc->sc_dev.dv_xname);
		return 1;
	}

	e = qwx_pci_cmd_ring_get_elem(ring, ring->wp);
	if (e == NULL)
		return 1;

	e->ptr = 0ULL;
	e->dword[0] = 0;
	e->dword[1] = htole32(
	    ((chan << MHI_TRE_CMD_CHID_SHFT) & MHI_TRE_CMD_CHID_MASK) |
	    ((cmd << MHI_TRE_CMD_CMDID_SHFT) & MHI_TRE_CMD_CMDID_MASK));

	return qwx_mhi_cmd_ring_submit(psc, ring);
}

void *
qwx_pci_xfer_ring_get_elem(struct qwx_pci_xfer_ring *ring, uint64_t wp)
{
	uint64_t base = QWX_DMA_DVA(ring->dmamem), offset;
	void *addr = QWX_DMA_KVA(ring->dmamem);

	if (wp < base)
		return NULL;

	offset = wp - base;
	if (offset >= ring->size)
		return NULL;

	return addr + offset;
}

struct qwx_xfer_data *
qwx_pci_xfer_ring_get_data(struct qwx_pci_xfer_ring *ring, uint64_t wp)
{
	uint64_t base = QWX_DMA_DVA(ring->dmamem), offset;

	if (wp < base)
		return NULL;

	offset = wp - base;
	if (offset >= ring->size)
		return NULL;

	return &ring->data[offset / sizeof(ring->data[0])];
}

int
qwx_mhi_submit_xfer(struct qwx_softc *sc, struct mbuf *m)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;
	struct qwx_pci_xfer_ring *ring;
	struct qwx_mhi_ring_element *e;
	struct qwx_xfer_data *xfer;
	uint64_t paddr, base;
	int err;

	ring = &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_OUTBOUND];

	if (ring->queued >= ring->num_elements)
		return 1;

	if (m->m_pkthdr.len > QWX_PCI_XFER_MAX_DATA_SIZE) {
		/* TODO: chunk xfers */
		printf("%s: xfer too large: %d bytes\n", __func__, m->m_pkthdr.len);
		return 1;

	}

	e = qwx_pci_xfer_ring_get_elem(ring, ring->wp);
	if (e == NULL)
		return 1;

	xfer = qwx_pci_xfer_ring_get_data(ring, ring->wp);
	if (xfer == NULL || xfer->m != NULL)
		return 1;

	err = bus_dmamap_load_mbuf(sc->sc_dmat, xfer->map, m,
	    BUS_DMA_NOWAIT | BUS_DMA_WRITE);
	if (err && err != EFBIG) {
		printf("%s: can't map mbuf (error %d)\n",
		    sc->sc_dev.dv_xname, err);
		return err;
	}
	if (err) {
		/* Too many DMA segments, linearize mbuf. */
		if (m_defrag(m, M_DONTWAIT))
			return ENOBUFS;
		err = bus_dmamap_load_mbuf(sc->sc_dmat, xfer->map, m,
		    BUS_DMA_NOWAIT | BUS_DMA_WRITE);
		if (err) {
			printf("%s: can't map mbuf (error %d)\n",
			    sc->sc_dev.dv_xname, err);
			return err;
		}
	}

	bus_dmamap_sync(sc->sc_dmat, xfer->map, 0, m->m_pkthdr.len,
	    BUS_DMASYNC_PREWRITE);

	xfer->m = m;
	paddr = xfer->map->dm_segs[0].ds_addr;

	e->ptr = htole64(paddr);
	e->dword[0] = htole32((m->m_pkthdr.len << MHI_TRE0_DATA_LEN_SHFT) &
	    MHI_TRE0_DATA_LEN_MASK);
	e->dword[1] = htole32(MHI_TRE1_DATA_IEOT |
	    MHI_TRE1_DATA_TYPE_TRANSFER << MHI_TRE1_DATA_TYPE_SHIFT);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(ring->dmamem),
	    0, QWX_DMA_LEN(ring->dmamem), BUS_DMASYNC_PREWRITE);

	base = QWX_DMA_DVA(ring->dmamem);
	if (ring->wp + sizeof(struct qwx_mhi_ring_element) >= base + ring->size)
		ring->wp = base;
	else
		ring->wp += sizeof(struct qwx_mhi_ring_element);
	ring->queued++;

	ring->chan_ctxt->wp = htole64(ring->wp);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->chan_ctxt), 0,
	    QWX_DMA_LEN(psc->chan_ctxt), BUS_DMASYNC_PREWRITE);

	qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	return 0;
}

int
qwx_mhi_start_channel(struct qwx_pci_softc *psc,
	struct qwx_pci_xfer_ring *ring)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_mhi_chan_ctxt *c;
	int ret = 0;
	uint32_t chcfg;
	uint64_t paddr, len;

	DNPRINTF(QWX_D_MHI, "%s: start MHI channel %d in state %d\n", __func__,
	    ring->mhi_chan_id, ring->mhi_chan_state);

	c = ring->chan_ctxt;

	chcfg = le32toh(c->chcfg);
	chcfg &= ~MHI_CHAN_CTX_CHSTATE_MASK;
	chcfg |= MHI_CHAN_CTX_CHSTATE_ENABLED;
	c->chcfg = htole32(chcfg);

	paddr = QWX_DMA_DVA(ring->dmamem);
	ring->rp = ring->wp = paddr;
	ring->queued = 0;
	c->rbase = htole64(paddr);
	c->rp = htole64(ring->rp);
	c->wp = htole64(ring->wp);
	len = sizeof(struct qwx_mhi_ring_element) * ring->num_elements;
	c->rlen = htole64(len);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->chan_ctxt), 0,
	    QWX_DMA_LEN(psc->chan_ctxt), BUS_DMASYNC_PREWRITE);

	ring->cmd_status = MHI_EV_CC_INVALID;
	if (qwx_mhi_send_cmd(psc, MHI_CMD_START_CHAN, ring->mhi_chan_id))
		return 1;

	while (ring->cmd_status != MHI_EV_CC_SUCCESS) {
		ret = tsleep_nsec(&ring->cmd_status, 0, "qwxcmd",
		    SEC_TO_NSEC(5));
		if (ret)
			break;
	}

	if (ret) {
		printf("%s: could not start MHI channel %d in state %d: status 0x%x\n",
		    sc->sc_dev.dv_xname, ring->mhi_chan_id,
		    ring->mhi_chan_state, ring->cmd_status);
		return 1;
	}

	if (ring->mhi_chan_direction == MHI_CHAN_TYPE_INBOUND) {
		uint64_t wp = QWX_DMA_DVA(ring->dmamem);
		int i;

		for (i = 0; i < ring->num_elements; i++) {
			struct qwx_mhi_ring_element *e;
			struct qwx_xfer_data *xfer;
			uint64_t paddr;

			e = qwx_pci_xfer_ring_get_elem(ring, wp);
			xfer = qwx_pci_xfer_ring_get_data(ring, wp);
			paddr = xfer->map->dm_segs[0].ds_addr;

			e->ptr = htole64(paddr);
			e->dword[0] = htole32((QWX_PCI_XFER_MAX_DATA_SIZE <<
			    MHI_TRE0_DATA_LEN_SHFT) &
			    MHI_TRE0_DATA_LEN_MASK);
			e->dword[1] = htole32(MHI_TRE1_DATA_IEOT |
			    MHI_TRE1_DATA_BEI |
			    MHI_TRE1_DATA_TYPE_TRANSFER <<
			    MHI_TRE1_DATA_TYPE_SHIFT);

			ring->wp = wp;
			wp += sizeof(*e);
		}

		bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(ring->dmamem), 0,
		    QWX_DMA_LEN(ring->dmamem), BUS_DMASYNC_PREWRITE);

		qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	}

	ring->mhi_chan_state = MHI_CH_STATE_ENABLED;
	return 0;
}

int
qwx_mhi_start_channels(struct qwx_pci_softc *psc)
{
	struct qwx_pci_xfer_ring *ring;
	int ret = 0;

	qwx_mhi_device_wake(&psc->sc_sc);

	ring = &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_OUTBOUND];
	if (qwx_mhi_start_channel(psc, ring)) {
		ret = 1;
		goto done;
	}

	ring = &psc->xfer_rings[QWX_PCI_XFER_RING_IPCR_INBOUND];
	if (qwx_mhi_start_channel(psc, ring))
		ret = 1;
done:
	qwx_mhi_device_zzz(&psc->sc_sc);
	return ret;
}

int
qwx_mhi_start(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	uint32_t off;
	uint32_t ee, state;
	int ret;

	qwx_mhi_init_dev_ctxt(psc);

	psc->bhi_off = qwx_pci_read(sc, MHI_BHI_OFFSET);
	DNPRINTF(QWX_D_MHI, "%s: BHI offset 0x%x\n", __func__, psc->bhi_off);

	psc->bhie_off = qwx_pci_read(sc, MHI_BHIE_OFFSET);
	DNPRINTF(QWX_D_MHI, "%s: BHIE offset 0x%x\n", __func__, psc->bhie_off);

	/* Clean BHIE RX registers */
	for (off = MHI_BHIE_RXVECADDR_LOW_OFFS;
	     off < (MHI_BHIE_RXVECSTATUS_OFFS - 4);
	     off += 4)
	     	qwx_pci_write(sc, psc->bhie_off + off, 0x0);

	qwx_rddm_prepare(psc);

	/* Program BHI INTVEC */
	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_INTVEC, 0x00);

	/*
	 * Get BHI execution environment and confirm that it is valid
	 * for power on.
	 */
	ee = qwx_pci_read(sc, psc->bhi_off + MHI_BHI_EXECENV);
	if (!MHI_POWER_UP_CAPABLE(ee)) {
		printf("%s: invalid EE for power on: 0x%x\n",
		     sc->sc_dev.dv_xname, ee);
		return 1;
	}

	/*
	 * Get MHI state of the device and reset it if it is in system
	 * error.
	 */
	state = qwx_pci_read(sc, MHI_STATUS);
	DNPRINTF(QWX_D_MHI, "%s: MHI power on with EE: 0x%x, status: 0x%x\n",
	     sc->sc_dev.dv_xname, ee, state);
	state = (state & MHI_STATUS_MHISTATE_MASK) >> MHI_STATUS_MHISTATE_SHFT;
	if (state == MHI_STATE_SYS_ERR) {
		if (qwx_mhi_reset_device(sc, 0))
			return 1;
		state = qwx_pci_read(sc, MHI_STATUS);
		DNPRINTF(QWX_D_MHI, "%s: MHI state after reset: 0x%x\n",
		    sc->sc_dev.dv_xname, state);
		state = (state & MHI_STATUS_MHISTATE_MASK) >>
		    MHI_STATUS_MHISTATE_SHFT;
		if (state == MHI_STATE_SYS_ERR) {
			printf("%s: MHI stuck in system error state\n",
			    sc->sc_dev.dv_xname);
			return 1;
		}
	}

	psc->bhi_ee = ee;
	psc->mhi_state = state;

#if notyet
	/* Enable IRQs */
	//  XXX todo?
#endif

	/* Transition to primary runtime. */
	if (MHI_IN_PBL(ee)) {
		ret = qwx_mhi_fw_load_handler(psc);
		if (ret)
			return ret;

		/* XXX without this delay starting the channels may fail */
		delay(1000);
		qwx_mhi_start_channels(psc);
	} else {
		/* XXX Handle partially initialized device...?!? */
		ee = qwx_pci_read(sc, psc->bhi_off + MHI_BHI_EXECENV);
		if (!MHI_IN_MISSION_MODE(ee)) {
			printf("%s: failed to power up MHI, ee=0x%x\n",
			    sc->sc_dev.dv_xname, ee);
			return EIO;
		}
	}

	return 0;
}

void
qwx_mhi_stop(struct qwx_softc *sc)
{
	qwx_mhi_reset_device(sc, 1);
}

int
qwx_mhi_reset_device(struct qwx_softc *sc, int force)
{
	struct qwx_pci_softc *psc = (struct qwx_pci_softc *)sc;
	uint32_t reg;
	int ret = 0;

	reg = qwx_pcic_read32(sc, MHI_STATUS);

	DNPRINTF(QWX_D_MHI, "%s: MHISTATUS 0x%x\n", sc->sc_dev.dv_xname, reg);
	/*
	 * Observed on QCA6390 that after SOC_GLOBAL_RESET, MHISTATUS
	 * has SYSERR bit set and thus need to set MHICTRL_RESET
	 * to clear SYSERR.
	 */
	if (force || (reg & MHI_STATUS_SYSERR_MASK)) {
		/* Trigger MHI Reset in device. */
		qwx_pcic_write32(sc, MHI_CTRL, MHI_CTRL_RESET_MASK);

		/* Wait for the reset bit to be cleared by the device. */
		ret = qwx_mhi_await_device_reset(sc);
		if (ret)
			return ret;

		if (psc->bhi_off == 0)
			psc->bhi_off = qwx_pci_read(sc, MHI_BHI_OFFSET);

		/* Device clear BHI INTVEC so re-program it. */
		qwx_pci_write(sc, psc->bhi_off + MHI_BHI_INTVEC, 0x00);
	}

	return 0;
}

static inline void
qwx_mhi_reset_txvecdb(struct qwx_softc *sc)
{
	qwx_pcic_write32(sc, PCIE_TXVECDB, 0);
}

static inline void
qwx_mhi_reset_txvecstatus(struct qwx_softc *sc)
{
	qwx_pcic_write32(sc, PCIE_TXVECSTATUS, 0);
}

static inline void
qwx_mhi_reset_rxvecdb(struct qwx_softc *sc)
{
	qwx_pcic_write32(sc, PCIE_RXVECDB, 0);
}

static inline void
qwx_mhi_reset_rxvecstatus(struct qwx_softc *sc)
{
	qwx_pcic_write32(sc, PCIE_RXVECSTATUS, 0);
}

void
qwx_mhi_clear_vector(struct qwx_softc *sc)
{
	qwx_mhi_reset_txvecdb(sc);
	qwx_mhi_reset_txvecstatus(sc);
	qwx_mhi_reset_rxvecdb(sc);
	qwx_mhi_reset_rxvecstatus(sc);
}

int
qwx_mhi_fw_load_handler(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret;
	char amss_path[PATH_MAX];
	u_char *data;
	size_t len;

	ret = snprintf(amss_path, sizeof(amss_path), "%s-%s-%s",
	    ATH11K_FW_DIR, sc->hw_params.fw.dir, ATH11K_AMSS_FILE);
	if (ret < 0 || ret >= sizeof(amss_path))
		return ENOSPC;

	if (sc->fw_img[QWX_FW_AMSS].data) {
		data = sc->fw_img[QWX_FW_AMSS].data;
		len = sc->fw_img[QWX_FW_AMSS].size;
	} else {
		ret = loadfirmware(amss_path, &data, &len);
		if (ret) {
			printf("%s: could not read %s (error %d)\n",
			    sc->sc_dev.dv_xname, amss_path, ret);
			return ret;
		}

		if (len < MHI_DMA_VEC_CHUNK_SIZE) {
			printf("%s: %s is too short, have only %zu bytes\n",
			    sc->sc_dev.dv_xname, amss_path, len);
			free(data, M_DEVBUF, len);
			return EINVAL;
		}

		sc->fw_img[QWX_FW_AMSS].data = data;
		sc->fw_img[QWX_FW_AMSS].size = len;
	}

	/* Second-stage boot loader sits in the first 512 KB of image. */
	ret = qwx_mhi_fw_load_bhi(psc, data, MHI_DMA_VEC_CHUNK_SIZE);
	if (ret != 0) {
		printf("%s: could not load firmware %s\n",
		    sc->sc_dev.dv_xname, amss_path);
		return ret;
	}

	/* Now load the full image. */
	ret = qwx_mhi_fw_load_bhie(psc, data, len);
	if (ret != 0) {
		printf("%s: could not load firmware %s\n",
		    sc->sc_dev.dv_xname, amss_path);
		return ret;
	}

	while (psc->bhi_ee < MHI_EE_AMSS) {
		ret = tsleep_nsec(&psc->bhi_ee, 0, "qwxamss",
		    SEC_TO_NSEC(5));
		if (ret)
			break;
	}
	if (ret != 0) {
		printf("%s: device failed to enter AMSS EE\n",
		    sc->sc_dev.dv_xname);
	}

	return ret;
}

int
qwx_mhi_await_device_reset(struct qwx_softc *sc)
{
	const uint32_t msecs = 24, retries = 2;
	uint32_t reg;
	int timeout;

	/* Poll for CTRL RESET to clear. */
	timeout = retries;
	while (timeout > 0) {
		reg = qwx_pci_read(sc, MHI_CTRL);
		DNPRINTF(QWX_D_MHI, "%s: MHI_CTRL is 0x%x\n", __func__, reg);
		if ((reg & MHI_CTRL_RESET_MASK) == 0)
			break;
		DELAY((msecs / retries) * 1000);
		timeout--;
	}
	if (timeout == 0) {
		DNPRINTF(QWX_D_MHI, "%s: MHI reset failed\n", __func__);
		return ETIMEDOUT;
	}

	return 0;
}

int
qwx_mhi_await_device_ready(struct qwx_softc *sc)
{
	uint32_t reg;
	int timeout;
	const uint32_t msecs = 2000, retries = 4;


	/* Poll for READY to be set. */
	timeout = retries;
	while (timeout > 0) {
		reg = qwx_pci_read(sc, MHI_STATUS);
		DNPRINTF(QWX_D_MHI, "%s: MHI_STATUS is 0x%x\n", __func__, reg);
		if (reg & MHI_STATUS_READY_MASK) {
			reg &= ~MHI_STATUS_READY_MASK;
			qwx_pci_write(sc, MHI_STATUS, reg);
			break;
		}
		DELAY((msecs / retries) * 1000);
		timeout--;
	}
	if (timeout == 0) {
		printf("%s: MHI not ready\n", sc->sc_dev.dv_xname);
		return ETIMEDOUT;
	}

	return 0;
}

void
qwx_mhi_ready_state_transition(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int ret, i;

	ret = qwx_mhi_await_device_reset(sc);
	if (ret)
		return;

	ret = qwx_mhi_await_device_ready(sc);
	if (ret)
		return;

	/* Set up memory-mapped IO for channels, events, etc. */
	qwx_mhi_init_mmio(psc);

	/* Notify event rings. */
	for (i = 0; i < nitems(psc->event_rings); i++) {
		struct qwx_pci_event_ring *ring = &psc->event_rings[i];
		qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	}

	/*
	 * Set the device into M0 state. The device will transition
	 * into M0 and the execution environment will switch to SBL.
	 */
	qwx_mhi_set_state(sc, MHI_STATE_M0);
}

void
qwx_mhi_mission_mode_state_transition(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	int i;

	qwx_mhi_device_wake(sc);

	/* Notify event rings. */
	for (i = 0; i < nitems(psc->event_rings); i++) {
		struct qwx_pci_event_ring *ring = &psc->event_rings[i];
		qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	}

	/* TODO: Notify transfer/command rings? */

	qwx_mhi_device_zzz(sc);
}

void
qwx_mhi_low_power_mode_state_transition(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;

	qwx_mhi_set_state(sc, MHI_STATE_M2);
}

void
qwx_mhi_set_state(struct qwx_softc *sc, uint32_t state)
{
	uint32_t reg;

	reg = qwx_pci_read(sc, MHI_CTRL);

	if (state != MHI_STATE_RESET) {
		reg &= ~MHI_CTRL_MHISTATE_MASK;
		reg |= (state << MHI_CTRL_MHISTATE_SHFT) & MHI_CTRL_MHISTATE_MASK;
	} else
		reg |= MHI_CTRL_RESET_MASK;

	qwx_pci_write(sc, MHI_CTRL, reg);
}

void
qwx_mhi_init_mmio(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	uint64_t paddr;
	uint32_t reg;
	int i;

	reg = qwx_pci_read(sc, MHI_CHDBOFF);

	/* Set device wake doorbell address. */
	psc->wake_db = reg + 8 * MHI_DEV_WAKE_DB;

	/* Set doorbell address for each transfer ring. */
	for (i = 0; i < nitems(psc->xfer_rings); i++) {
		struct qwx_pci_xfer_ring *ring = &psc->xfer_rings[i];
		ring->db_addr = reg + (8 * ring->mhi_chan_id);
	}

	reg = qwx_pci_read(sc, MHI_ERDBOFF);
	/* Set doorbell address for each event ring. */
	for (i = 0; i < nitems(psc->event_rings); i++) {
		struct qwx_pci_event_ring *ring = &psc->event_rings[i];
		ring->db_addr = reg + (8 * i);
	}

	paddr = QWX_DMA_DVA(psc->chan_ctxt);
	qwx_pci_write(sc, MHI_CCABAP_HIGHER, paddr >> 32);
	qwx_pci_write(sc, MHI_CCABAP_LOWER, paddr & 0xffffffff);

	paddr = QWX_DMA_DVA(psc->event_ctxt);
	qwx_pci_write(sc, MHI_ECABAP_HIGHER, paddr >> 32);
	qwx_pci_write(sc, MHI_ECABAP_LOWER, paddr & 0xffffffff);

	paddr = QWX_DMA_DVA(psc->cmd_ctxt);
	qwx_pci_write(sc, MHI_CRCBAP_HIGHER, paddr >> 32);
	qwx_pci_write(sc, MHI_CRCBAP_LOWER, paddr & 0xffffffff);

	/* Not (yet?) using fixed memory space from a device-tree. */
	qwx_pci_write(sc, MHI_CTRLBASE_HIGHER, 0);
	qwx_pci_write(sc, MHI_CTRLBASE_LOWER, 0);
	qwx_pci_write(sc, MHI_DATABASE_HIGHER, 0);
	qwx_pci_write(sc, MHI_DATABASE_LOWER, 0);
	qwx_pci_write(sc, MHI_CTRLLIMIT_HIGHER, 0x0);
	qwx_pci_write(sc, MHI_CTRLLIMIT_LOWER, 0xffffffff);
	qwx_pci_write(sc, MHI_DATALIMIT_HIGHER, 0x0);
	qwx_pci_write(sc, MHI_DATALIMIT_LOWER, 0xffffffff);

	reg = qwx_pci_read(sc, MHI_CFG);
	reg &= ~(MHI_CFG_NER_MASK | MHI_CFG_NHWER_MASK);
	reg |= QWX_NUM_EVENT_CTX << MHI_CFG_NER_SHFT;
	qwx_pci_write(sc, MHI_CFG, reg);
}

int
qwx_mhi_fw_load_bhi(struct qwx_pci_softc *psc, uint8_t *data, size_t len)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_dmamem *data_adm;
	uint32_t seq, reg, status = MHI_BHI_STATUS_RESET;
	uint64_t paddr;
	int ret;

	data_adm = qwx_dmamem_alloc(sc->sc_dmat, len, 0);
	if (data_adm == NULL) {
		printf("%s: could not allocate BHI DMA data buffer\n",
		    sc->sc_dev.dv_xname);
		return 1;
	}

	/* Copy firmware image to DMA memory. */
	memcpy(QWX_DMA_KVA(data_adm), data, len);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(data_adm), 0, len,
	    BUS_DMASYNC_PREWRITE);

	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_STATUS, 0);

	/* Set data physical address and length. */
	paddr = QWX_DMA_DVA(data_adm);
	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_IMGADDR_HIGH, paddr >> 32);
	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_IMGADDR_LOW,
	    paddr & 0xffffffff);
	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_IMGSIZE, len);
	
	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHI_TXDB_SEQNUM_BMSK);
	} while (seq == 0);
	qwx_pci_write(sc, psc->bhi_off + MHI_BHI_IMGTXDB, seq);

	/* Wait for completion. */
	ret = 0;
	while (status != MHI_BHI_STATUS_SUCCESS && psc->bhi_ee < MHI_EE_SBL) {
		ret = tsleep_nsec(&psc->bhi_ee, 0, "qwxbhi", SEC_TO_NSEC(5));
		if (ret)
			break;
		reg = qwx_pci_read(sc, psc->bhi_off + MHI_BHI_STATUS);
		status = (reg & MHI_BHI_STATUS_MASK) >> MHI_BHI_STATUS_SHFT;
	}

	if (ret) {
		printf("%s: BHI load timeout\n", sc->sc_dev.dv_xname);
		reg = qwx_pci_read(sc, psc->bhi_off + MHI_BHI_STATUS);
		status = (reg & MHI_BHI_STATUS_MASK) >> MHI_BHI_STATUS_SHFT;
		DNPRINTF(QWX_D_MHI, "%s: BHI status is 0x%x EE is 0x%x\n",
		    __func__, status, psc->bhi_ee);
	}

	qwx_dmamem_free(sc->sc_dmat, data_adm);
	return ret;
}

int
qwx_mhi_fw_load_bhie(struct qwx_pci_softc *psc, uint8_t *data, size_t len)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_dma_vec_entry *vec;
	uint32_t seq, reg, state = MHI_BHIE_TXVECSTATUS_STATUS_RESET;
	uint64_t paddr;
	const size_t chunk_size = MHI_DMA_VEC_CHUNK_SIZE;
	size_t nseg, remain, vec_size;
	int i, ret;

	nseg = howmany(len, chunk_size);
	if (nseg == 0) {
		printf("%s: BHIE data too short, have only %zu bytes\n",
		    sc->sc_dev.dv_xname, len);
		return 1;
	}

	if (psc->amss_data == NULL || QWX_DMA_LEN(psc->amss_data) < len) {
		if (psc->amss_data)
			qwx_dmamem_free(sc->sc_dmat, psc->amss_data);
		psc->amss_data = qwx_dmamem_alloc(sc->sc_dmat, len, 0);
		if (psc->amss_data == NULL) {
			printf("%s: could not allocate BHIE DMA data buffer\n",
			    sc->sc_dev.dv_xname);
			return 1;
		}
	}

	vec_size = nseg * sizeof(*vec);
	if (psc->amss_vec == NULL || QWX_DMA_LEN(psc->amss_vec) < vec_size) {
		if (psc->amss_vec)
			qwx_dmamem_free(sc->sc_dmat, psc->amss_vec);
		psc->amss_vec = qwx_dmamem_alloc(sc->sc_dmat, vec_size, 0);
		if (psc->amss_vec == NULL) {
			printf("%s: could not allocate BHIE DMA vec buffer\n",
			    sc->sc_dev.dv_xname);
			qwx_dmamem_free(sc->sc_dmat, psc->amss_data);
			psc->amss_data = NULL;
			return 1;
		}
	}

	/* Copy firmware image to DMA memory. */
	memcpy(QWX_DMA_KVA(psc->amss_data), data, len);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->amss_data), 0, len,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->amss_vec), 0, vec_size,
	    BUS_DMASYNC_PREWRITE);

	/* Create vector which controls chunk-wise DMA copy in hardware. */
	paddr = QWX_DMA_DVA(psc->amss_data);
	vec = QWX_DMA_KVA(psc->amss_vec);
	remain = len;
	for (i = 0; i < nseg; i++) {
		vec[i].paddr = paddr;
		if (remain >= chunk_size) {
			vec[i].size = chunk_size;
			remain -= chunk_size;
			paddr += chunk_size;
		} else
			vec[i].size = remain;
	}

	/* Set vector physical address and length. */
	paddr = QWX_DMA_DVA(psc->amss_vec);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_TXVECADDR_HIGH_OFFS,
	    paddr >> 32);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_TXVECADDR_LOW_OFFS,
	    paddr & 0xffffffff);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_TXVECSIZE_OFFS, vec_size);

	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHIE_TXVECSTATUS_SEQNUM_BMSK);
	} while (seq == 0);
	reg = qwx_pci_read(sc, psc->bhie_off + MHI_BHIE_TXVECDB_OFFS);
	reg &= ~MHI_BHIE_TXVECDB_SEQNUM_BMSK;
	reg |= seq << MHI_BHIE_TXVECDB_SEQNUM_SHFT;
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_TXVECDB_OFFS, reg);

	/* Wait for completion. */
	ret = 0;
	while (state != MHI_BHIE_TXVECSTATUS_STATUS_XFER_COMPL) {
		ret = tsleep_nsec(&psc->bhie_off, 0, "qwxbhie",
		    SEC_TO_NSEC(5));
		if (ret)
			break;
		reg = qwx_pci_read(sc,
		    psc->bhie_off + MHI_BHIE_TXVECSTATUS_OFFS);
		state = (reg & MHI_BHIE_TXVECSTATUS_STATUS_BMSK) >>
		    MHI_BHIE_TXVECSTATUS_STATUS_SHFT;
		DNPRINTF(QWX_D_MHI, "%s: txvec state is 0x%x\n", __func__,
		    state);
	}

	if (ret) {
		printf("%s: BHIE load timeout\n", sc->sc_dev.dv_xname);
		return ret;
	}
	return 0;
}

void
qwx_rddm_prepare(struct qwx_pci_softc *psc)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_dma_vec_entry *vec;
	struct qwx_dmamem *data_adm, *vec_adm;
	uint32_t seq, reg;
	uint64_t paddr;
	const size_t len = QWX_RDDM_DUMP_SIZE;
	const size_t chunk_size = MHI_DMA_VEC_CHUNK_SIZE;
	size_t nseg, remain, vec_size;
	int i;

	if (psc->rddm_data != NULL)
		return; /* already allocated */

	nseg = howmany(len, chunk_size);
	if (nseg == 0) {
		printf("%s: RDDM data too short, have only %zu bytes\n",
		    sc->sc_dev.dv_xname, len);
		return;
	}

	data_adm = qwx_dmamem_alloc(sc->sc_dmat, len, 0);
	if (data_adm == NULL) {
		printf("%s: could not allocate BHIE DMA data buffer\n",
		    sc->sc_dev.dv_xname);
		return;
	}

	vec_size = nseg * sizeof(*vec);
	vec_adm = qwx_dmamem_alloc(sc->sc_dmat, vec_size, 0);
	if (vec_adm == NULL) {
		printf("%s: could not allocate BHIE DMA vector buffer\n",
		    sc->sc_dev.dv_xname);
		qwx_dmamem_free(sc->sc_dmat, data_adm);
		return;
	}

	/* Create vector which controls chunk-wise DMA copy from hardware. */
	paddr = QWX_DMA_DVA(data_adm);
	vec = QWX_DMA_KVA(vec_adm);
	remain = len;
	for (i = 0; i < nseg; i++) {
		vec[i].paddr = paddr;
		if (remain >= chunk_size) {
			vec[i].size = chunk_size;
			remain -= chunk_size;
			paddr += chunk_size;
		} else
			vec[i].size = remain;
	}

	/* Set vector physical address and length. */
	paddr = QWX_DMA_DVA(vec_adm);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_RXVECADDR_HIGH_OFFS,
	    paddr >> 32);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_RXVECADDR_LOW_OFFS,
	    paddr & 0xffffffff);
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_RXVECSIZE_OFFS, vec_size);

	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHIE_RXVECSTATUS_SEQNUM_BMSK);
	} while (seq == 0);

	reg = qwx_pci_read(sc, psc->bhie_off + MHI_BHIE_RXVECDB_OFFS);
	reg &= ~MHI_BHIE_RXVECDB_SEQNUM_BMSK;
	reg |= seq << MHI_BHIE_RXVECDB_SEQNUM_SHFT;
	qwx_pci_write(sc, psc->bhie_off + MHI_BHIE_RXVECDB_OFFS, reg);

	psc->rddm_data = data_adm;
	psc->rddm_vec = vec_adm;
}

#ifdef QWX_DEBUG
void
qwx_rddm_task(void *arg)
{
	struct qwx_pci_softc *psc = arg;
	struct qwx_softc *sc = &psc->sc_sc;
	uint32_t reg, state = MHI_BHIE_RXVECSTATUS_STATUS_RESET;
	const size_t len = QWX_RDDM_DUMP_SIZE;
	int i, timeout;
	const uint32_t msecs = 100, retries = 20;
	uint8_t *rddm;
	struct nameidata nd;
	struct vnode *vp = NULL;
	struct iovec iov[3];
	struct uio uio;
	char path[PATH_MAX];
	int error = 0;

	if (psc->rddm_data == NULL) {
		DPRINTF("%s: RDDM not prepared\n", __func__);
		return;
	}

	/* Poll for completion */
	timeout = retries;
	while (timeout > 0 && state != MHI_BHIE_RXVECSTATUS_STATUS_XFER_COMPL) {
		reg = qwx_pci_read(sc,
		    psc->bhie_off + MHI_BHIE_RXVECSTATUS_OFFS);
		state = (reg & MHI_BHIE_RXVECSTATUS_STATUS_BMSK) >>
		    MHI_BHIE_RXVECSTATUS_STATUS_SHFT;
		DPRINTF("%s: txvec state is 0x%x\n", __func__, state);
		DELAY((msecs / retries) * 1000);
		timeout--;
	}

	if (timeout == 0) {
		DPRINTF("%s: RDDM dump failed\n", sc->sc_dev.dv_xname);
		return;
	}

	rddm = QWX_DMA_KVA(psc->rddm_data);
	DPRINTF("%s: RDDM snippet:\n", __func__);
	for (i = 0; i < MIN(64, len); i++) {
		DPRINTF("%s %.2x", i % 16 == 0 ? "\n" : "", rddm[i]);
	}
	DPRINTF("\n");

	DPRINTF("%s: sleeping for 30 seconds to allow userland to boot\n", __func__);
	tsleep_nsec(&psc->rddm_data, 0, "qwxrddm", SEC_TO_NSEC(30));

	snprintf(path, sizeof(path), "/root/%s-rddm.bin", sc->sc_dev.dv_xname);
	DPRINTF("%s: saving RDDM to %s\n", __func__, path);
	NDINIT(&nd, 0, 0, UIO_SYSSPACE, path, curproc);
	nd.ni_pledge = PLEDGE_CPATH | PLEDGE_WPATH;
	nd.ni_unveil = UNVEIL_CREATE | UNVEIL_WRITE;
	error = vn_open(&nd, FWRITE | O_CREAT | O_NOFOLLOW | O_TRUNC,
	    S_IRUSR | S_IWUSR);
	if (error) {
		DPRINTF("%s: vn_open: error %d\n", __func__, error);
		goto done;
	}
	vp = nd.ni_vp;
	VOP_UNLOCK(vp);

	iov[0].iov_base = (void *)rddm;
	iov[0].iov_len = len;
	iov[1].iov_len = 0;
	uio.uio_iov = &iov[0];
	uio.uio_offset = 0;
	uio.uio_segflg = UIO_SYSSPACE;
	uio.uio_rw = UIO_WRITE;
	uio.uio_resid = len;
	uio.uio_iovcnt = 1;
	uio.uio_procp = curproc;
	error = vget(vp, LK_EXCLUSIVE | LK_RETRY);
	if (error) {
		DPRINTF("%s: vget: error %d\n", __func__, error);
		goto done;
	}
	error = VOP_WRITE(vp, &uio, IO_UNIT|IO_APPEND, curproc->p_ucred);
	vput(vp);
	if (error)
		DPRINTF("%s: VOP_WRITE: error %d\n", __func__, error);
	#if 0
	error = vn_close(vp, FWRITE, curproc->p_ucred, curproc);
	if (error)
		DPRINTF("%s: vn_close: error %d\n", __func__, error);
	#endif
done:
	qwx_dmamem_free(sc->sc_dmat, psc->rddm_data);
	qwx_dmamem_free(sc->sc_dmat, psc->rddm_vec);
	psc->rddm_data = NULL;
	psc->rddm_vec = NULL;
	DPRINTF("%s: done, error %d\n", __func__, error);
}
#endif

void *
qwx_pci_event_ring_get_elem(struct qwx_pci_event_ring *ring, uint64_t rp)
{
	uint64_t base = QWX_DMA_DVA(ring->dmamem), offset;
	void *addr = QWX_DMA_KVA(ring->dmamem);

	if (rp < base)
		return NULL;

	offset = rp - base;
	if (offset >= ring->size)
		return NULL;

	return addr + offset;
}

void
qwx_mhi_state_change(struct qwx_pci_softc *psc, int ee, int mhi_state)
{
	struct qwx_softc *sc = &psc->sc_sc;
	uint32_t old_ee = psc->bhi_ee;
	uint32_t old_mhi_state = psc->mhi_state;

	if (ee != -1 && psc->bhi_ee != ee) {
		switch (ee) {
		case MHI_EE_PBL:
			DNPRINTF(QWX_D_MHI, "%s: new EE PBL\n",
			    sc->sc_dev.dv_xname);
			psc->bhi_ee = ee;
			break;
		case MHI_EE_SBL:
			psc->bhi_ee = ee;
			DNPRINTF(QWX_D_MHI, "%s: new EE SBL\n",
			    sc->sc_dev.dv_xname);
			break;
		case MHI_EE_AMSS:
			DNPRINTF(QWX_D_MHI, "%s: new EE AMSS\n",
			    sc->sc_dev.dv_xname);
			psc->bhi_ee = ee;
			/* Wake thread loading the full AMSS image. */
			wakeup(&psc->bhie_off);
			break;
		case MHI_EE_WFW:
			DNPRINTF(QWX_D_MHI, "%s: new EE WFW\n",
			    sc->sc_dev.dv_xname);
			psc->bhi_ee = ee;
			break;
		default:
			printf("%s: unhandled EE change to %x\n",
			    sc->sc_dev.dv_xname, ee);
			break;
		}
	}

	if (mhi_state != -1 && psc->mhi_state != mhi_state) {
		switch (mhi_state) {
		case -1:
			break;
		case MHI_STATE_RESET:
			DNPRINTF(QWX_D_MHI, "%s: new MHI state RESET\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_READY:
			DNPRINTF(QWX_D_MHI, "%s: new MHI state READY\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			qwx_mhi_ready_state_transition(psc);
			break;
		case MHI_STATE_M0:
			DNPRINTF(QWX_D_MHI, "%s: new MHI state M0\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			qwx_mhi_mission_mode_state_transition(psc);
			break;
		case MHI_STATE_M1:
			DNPRINTF(QWX_D_MHI, "%s: new MHI state M1\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			qwx_mhi_low_power_mode_state_transition(psc);
			break;
		case MHI_STATE_M3:
			DNPRINTF(QWX_D_MHI, "%s: new MHI state M3\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_SYS_ERR:
			DNPRINTF(QWX_D_MHI,
			    "%s: new MHI state SYS ERR\n",
			    sc->sc_dev.dv_xname);
			psc->mhi_state = mhi_state;
			break;
		default:
			printf("%s: unhandled MHI state change to %x\n",
			    sc->sc_dev.dv_xname, mhi_state);
			break;
		}
	}

	if (old_ee != psc->bhi_ee)
		wakeup(&psc->bhi_ee);
	if (old_mhi_state != psc->mhi_state)
		wakeup(&psc->mhi_state);
}

void
qwx_pci_intr_ctrl_event_mhi(struct qwx_pci_softc *psc, uint32_t mhi_state)
{
	DNPRINTF(QWX_D_MHI, "%s: MHI state change 0x%x -> 0x%x\n", __func__,
	    psc->mhi_state, mhi_state);

	if (psc->mhi_state != mhi_state)
		qwx_mhi_state_change(psc, -1, mhi_state);
}

void
qwx_pci_intr_ctrl_event_ee(struct qwx_pci_softc *psc, uint32_t ee)
{
	DNPRINTF(QWX_D_MHI, "%s: EE change 0x%x to 0x%x\n", __func__,
	    psc->bhi_ee, ee);

	if (psc->bhi_ee != ee)
		qwx_mhi_state_change(psc, ee, -1);
}

void
qwx_pci_intr_ctrl_event_cmd_complete(struct qwx_pci_softc *psc,
    uint64_t ptr, uint32_t cmd_status)
{
	struct qwx_pci_cmd_ring	*cmd_ring = &psc->cmd_ring;
	uint64_t base = QWX_DMA_DVA(cmd_ring->dmamem);
	struct qwx_pci_xfer_ring *xfer_ring = NULL;
	struct qwx_mhi_ring_element *e;
	uint32_t tre1, chid;
	size_t i;

	e = qwx_pci_cmd_ring_get_elem(cmd_ring, ptr);
	if (e == NULL)
		return;

	tre1 = le32toh(e->dword[1]);
	chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;

	for (i = 0; i < nitems(psc->xfer_rings); i++) {
		if (psc->xfer_rings[i].mhi_chan_id == chid) {
			xfer_ring = &psc->xfer_rings[i];
			break;
		}
	}
	if (xfer_ring == NULL) {
		printf("%s: no transfer ring found for command completion "
		    "on channel %u\n", __func__, chid);
		return;
	}

	xfer_ring->cmd_status = cmd_status;
	wakeup(&xfer_ring->cmd_status);

	if (cmd_ring->rp + sizeof(*e) >= base + cmd_ring->size)
		cmd_ring->rp = base;
	else
		cmd_ring->rp += sizeof(*e);
}

int
qwx_pci_intr_ctrl_event(struct qwx_pci_softc *psc,
    struct qwx_pci_event_ring *ring, int flush)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_mhi_event_ctxt *c;
	uint64_t rp, wp, base;
	struct qwx_mhi_ring_element *e;
	uint32_t tre0, tre1, type, code, chid, len;

	c = ring->event_ctxt;
	if (c == NULL) {
		/*
		 * Interrupts can trigger before mhi_init_event_rings()
		 * if the device is still active after a warm reboot.
		 */
		return 0;
	}

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->event_ctxt), 0,
	    QWX_DMA_LEN(psc->event_ctxt), BUS_DMASYNC_POSTREAD);

	rp = le64toh(c->rp);
	wp = le64toh(c->wp);

	DNPRINTF(QWX_D_MHI, "%s: kernel rp=0x%llx\n", __func__, ring->rp);
	DNPRINTF(QWX_D_MHI, "%s: device rp=0x%llx\n", __func__, rp);
	DNPRINTF(QWX_D_MHI, "%s: kernel wp=0x%llx\n", __func__, ring->wp);
	DNPRINTF(QWX_D_MHI, "%s: device wp=0x%llx\n", __func__, wp);

	base = QWX_DMA_DVA(ring->dmamem);
	if (ring->rp == rp || rp < base || rp >= base + ring->size)
		return 0;
	if (wp < base || wp >= base + ring->size)
		return 0;

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(ring->dmamem),
	    0, QWX_DMA_LEN(ring->dmamem), BUS_DMASYNC_POSTREAD);

	while (ring->rp != rp) {
		e = qwx_pci_event_ring_get_elem(ring, ring->rp);
		if (e == NULL)
			return 0;

		tre0 = le32toh(e->dword[0]);
		tre1 = le32toh(e->dword[1]);

		len = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
		code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
		type = (tre1 & MHI_TRE1_EV_TYPE_MASK) >> MHI_TRE1_EV_TYPE_SHFT;
		chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;
		DNPRINTF(QWX_D_MHI, "%s: len=%u code=0x%x type=0x%x chid=%d\n",
		    __func__, len, code, type, chid);

		if (!flush) {
			switch (type) {
			case MHI_PKT_TYPE_STATE_CHANGE_EVENT:
				qwx_pci_intr_ctrl_event_mhi(psc, code);
				break;
			case MHI_PKT_TYPE_EE_EVENT:
				qwx_pci_intr_ctrl_event_ee(psc, code);
				break;
			case MHI_PKT_TYPE_CMD_COMPLETION_EVENT:
				qwx_pci_intr_ctrl_event_cmd_complete(psc,
				    le64toh(e->ptr), code);
				break;
			default:
				printf("%s: unhandled event type 0x%x\n",
				    __func__, type);
				break;
			}
		}

		if (ring->rp + sizeof(*e) >= base + ring->size)
			ring->rp = base;
		else
			ring->rp += sizeof(*e);

		if (ring->wp + sizeof(*e) >= base + ring->size)
			ring->wp = base;
		else
			ring->wp += sizeof(*e);
	}

	c->wp = htole64(ring->wp);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->event_ctxt), 0,
	    QWX_DMA_LEN(psc->event_ctxt), BUS_DMASYNC_PREWRITE);

	qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	return 1;
}

void
qwx_pci_intr_data_event_tx(struct qwx_pci_softc *psc, struct qwx_mhi_ring_element *e)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_pci_xfer_ring *ring;
	struct qwx_xfer_data *xfer;
	uint64_t rp, evrp, base, paddr;
	uint32_t tre0, tre1, code, chid, evlen, len;
	int i;

	tre0 = le32toh(e->dword[0]);
	tre1 = le32toh(e->dword[1]);

	evlen = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
	code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
	chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;

	switch (code) {
	case MHI_EV_CC_EOT:
		for (i = 0; i < nitems(psc->xfer_rings); i++) {
			ring = &psc->xfer_rings[i];
			if (ring->mhi_chan_id == chid)
				break;
		}
		if (i == nitems(psc->xfer_rings)) {
			printf("%s: unhandled channel 0x%x\n",
			    __func__, chid);
			break;
		}
		base = QWX_DMA_DVA(ring->dmamem);
		/* PTR contains the entry that was last written */
		evrp = letoh64(e->ptr);
		rp = evrp;
		if (rp < base || rp >= base + ring->size) {
			printf("%s: invalid ptr 0x%llx\n",
			    __func__, rp);
			break;
		}
		/* Point rp to next empty slot */
		if (rp + sizeof(*e) >= base + ring->size)
			rp = base;
		else
			rp += sizeof(*e);
		/* Parse until next empty slot */
		while (ring->rp != rp) {
			DNPRINTF(QWX_D_MHI, "%s:%d: ring->rp 0x%llx "
			    "ring->wp 0x%llx rp 0x%llx\n", __func__,
			    __LINE__, ring->rp, ring->wp, rp);
			e = qwx_pci_xfer_ring_get_elem(ring, ring->rp);
			xfer = qwx_pci_xfer_ring_get_data(ring, ring->rp);

			if (ring->rp == evrp)
				len = evlen;
			else
				len = xfer->m->m_pkthdr.len;

			bus_dmamap_sync(sc->sc_dmat, xfer->map, 0,
			    xfer->m->m_pkthdr.len, BUS_DMASYNC_POSTREAD);
#ifdef QWX_DEBUG
			{
			int i;
			DNPRINTF(QWX_D_MHI, "%s: chan %u data (len %u): ",
			    __func__,
			    ring->mhi_chan_id, len);
			for (i = 0; i < MIN(32, len); i++) {
				DNPRINTF(QWX_D_MHI, "%02x ",
				    (unsigned char)mtod(xfer->m, caddr_t)[i]);
			}
			if (i < len)
				DNPRINTF(QWX_D_MHI, "...");
			DNPRINTF(QWX_D_MHI, "\n");
			}
#endif
			if (ring->mhi_chan_direction == MHI_CHAN_TYPE_INBOUND) {
				/* Save m_data as upper layers use m_adj(9) */
				void *o_data = xfer->m->m_data;

				/* Pass mbuf to upper layers */
				qwx_qrtr_recv_msg(sc, xfer->m);

				/* Reset RX mbuf instead of free/alloc */
				KASSERT(xfer->m->m_next == NULL);
				xfer->m->m_data = o_data;
				xfer->m->m_len = xfer->m->m_pkthdr.len =
				    QWX_PCI_XFER_MAX_DATA_SIZE;

				paddr = xfer->map->dm_segs[0].ds_addr;

				e->ptr = htole64(paddr);
				e->dword[0] = htole32((
				    QWX_PCI_XFER_MAX_DATA_SIZE <<
				    MHI_TRE0_DATA_LEN_SHFT) &
				    MHI_TRE0_DATA_LEN_MASK);
				e->dword[1] = htole32(MHI_TRE1_DATA_IEOT |
				    MHI_TRE1_DATA_BEI |
				    MHI_TRE1_DATA_TYPE_TRANSFER <<
				    MHI_TRE1_DATA_TYPE_SHIFT);

				if (ring->wp + sizeof(*e) >= base + ring->size)
					ring->wp = base;
				else
					ring->wp += sizeof(*e);
			} else {
				/* Unload and free TX mbuf */
				bus_dmamap_unload(sc->sc_dmat, xfer->map);
				m_freem(xfer->m);
				xfer->m = NULL;
				ring->queued--;
			}

			if (ring->rp + sizeof(*e) >= base + ring->size)
				ring->rp = base;
			else
				ring->rp += sizeof(*e);
		}

		if (ring->mhi_chan_direction == MHI_CHAN_TYPE_INBOUND) {
			ring->chan_ctxt->wp = htole64(ring->wp);

			bus_dmamap_sync(sc->sc_dmat,
			    QWX_DMA_MAP(psc->chan_ctxt), 0,
			    QWX_DMA_LEN(psc->chan_ctxt),
			    BUS_DMASYNC_PREWRITE);

			qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
		}
		break;
	default:
		printf("%s: unhandled event code 0x%x\n",
		    __func__, code);
	}
}

int
qwx_pci_intr_data_event(struct qwx_pci_softc *psc,
    struct qwx_pci_event_ring *ring, int flush)
{
	struct qwx_softc *sc = &psc->sc_sc;
	struct qwx_mhi_event_ctxt *c;
	uint64_t rp, wp, base;
	struct qwx_mhi_ring_element *e;
	uint32_t tre0, tre1, type, code, chid, len;

	c = ring->event_ctxt;
	if (c == NULL) {
		/*
		 * Interrupts can trigger before mhi_init_event_rings()
		 * if the device is still active after a warm reboot.
		 */
		return 0;
	}

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->event_ctxt), 0,
	    QWX_DMA_LEN(psc->event_ctxt), BUS_DMASYNC_POSTREAD);

	rp = le64toh(c->rp);
	wp = le64toh(c->wp);

	DNPRINTF(QWX_D_MHI, "%s: kernel rp=0x%llx\n", __func__, ring->rp);
	DNPRINTF(QWX_D_MHI, "%s: device rp=0x%llx\n", __func__, rp);
	DNPRINTF(QWX_D_MHI, "%s: kernel wp=0x%llx\n", __func__, ring->wp);
	DNPRINTF(QWX_D_MHI, "%s: device wp=0x%llx\n", __func__, wp);

	base = QWX_DMA_DVA(ring->dmamem);
	if (ring->rp == rp || rp < base || rp >= base + ring->size)
		return 0;

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(ring->dmamem),
	    0, QWX_DMA_LEN(ring->dmamem), BUS_DMASYNC_POSTREAD);

	while (ring->rp != rp) {
		e = qwx_pci_event_ring_get_elem(ring, ring->rp);
		if (e == NULL)
			return 0;

		tre0 = le32toh(e->dword[0]);
		tre1 = le32toh(e->dword[1]);

		len = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
		code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
		type = (tre1 & MHI_TRE1_EV_TYPE_MASK) >> MHI_TRE1_EV_TYPE_SHFT;
		chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;
		DNPRINTF(QWX_D_MHI, "%s: len=%u code=0x%x type=0x%x chid=%d\n",
		    __func__, len, code, type, chid);

		if (!flush) {
			switch (type) {
			case MHI_PKT_TYPE_TX_EVENT:
				qwx_pci_intr_data_event_tx(psc, e);
				break;
			default:
				printf("%s: unhandled event type 0x%x\n",
				    __func__, type);
				break;
			}
		}

		if (ring->rp + sizeof(*e) >= base + ring->size)
			ring->rp = base;
		else
			ring->rp += sizeof(*e);

		if (ring->wp + sizeof(*e) >= base + ring->size)
			ring->wp = base;
		else
			ring->wp += sizeof(*e);
	}

	c->wp = htole64(ring->wp);

	bus_dmamap_sync(sc->sc_dmat, QWX_DMA_MAP(psc->event_ctxt), 0,
	    QWX_DMA_LEN(psc->event_ctxt), BUS_DMASYNC_PREWRITE);

	qwx_mhi_ring_doorbell(sc, ring->db_addr, ring->wp);
	return 1;
}

int
qwx_pci_intr_mhi_ctrl(void *arg)
{
	struct qwx_pci_softc *psc = arg;

	if (qwx_pci_intr_ctrl_event(psc, &psc->event_rings[0], 0))
		return 1;

	return 0;
}

int
qwx_pci_intr_mhi_data(void *arg)
{
	struct qwx_pci_softc *psc = arg;

	if (qwx_pci_intr_data_event(psc, &psc->event_rings[1], 0))
		return 1;

	return 0;
}

int
qwx_pci_intr(void *arg)
{
	struct qwx_pci_softc *psc = arg;
	struct qwx_softc *sc = (void *)psc;
	uint32_t ee, state;
	int ret = 0;

	/*
	 * Interrupts can trigger before mhi_start() during boot if the device
	 * is still active after a warm reboot.
	 */
	if (psc->bhi_off == 0)
		psc->bhi_off = qwx_pci_read(sc, MHI_BHI_OFFSET);

	ee = qwx_pci_read(sc, psc->bhi_off + MHI_BHI_EXECENV);
	state = qwx_pci_read(sc, MHI_STATUS);
	state = (state & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;

	DNPRINTF(QWX_D_MHI,
	    "%s: BHI interrupt with EE: 0x%x -> 0x%x state: 0x%x -> 0x%x\n",
	     sc->sc_dev.dv_xname, psc->bhi_ee, ee, psc->mhi_state, state);

	if (ee == MHI_EE_RDDM) {
		/* Firmware crash, e.g. due to invalid DMA memory access. */
		psc->bhi_ee = ee;
#ifdef QWX_DEBUG
		if (!psc->rddm_triggered) {
			/* Write fw memory dump to root's home directory. */
			task_add(systq, &psc->rddm_task);
			psc->rddm_triggered = 1;
		}
#else
		printf("%s: fatal firmware error\n",
		   sc->sc_dev.dv_xname);
		if (!test_bit(ATH11K_FLAG_CRASH_FLUSH, sc->sc_flags) &&
		    (sc->sc_ic.ic_if.if_flags & (IFF_UP | IFF_RUNNING)) ==
		    (IFF_UP | IFF_RUNNING)) {
			/* Try to reset the device. */
			set_bit(ATH11K_FLAG_CRASH_FLUSH, sc->sc_flags);
			task_add(systq, &sc->init_task);
		}
#endif
		return 1;
	} else if (psc->bhi_ee == MHI_EE_PBL || psc->bhi_ee == MHI_EE_SBL) {
		int new_ee = -1, new_mhi_state = -1;

		if (psc->bhi_ee != ee)
			new_ee = ee;

		if (psc->mhi_state != state)
			new_mhi_state = state;

		if (new_ee != -1 || new_mhi_state != -1)
			qwx_mhi_state_change(psc, new_ee, new_mhi_state);

		ret = 1;
	}

	if (!test_bit(ATH11K_FLAG_MULTI_MSI_VECTORS, sc->sc_flags)) {
		int i;

		if (qwx_pci_intr_ctrl_event(psc, &psc->event_rings[0], 0))
			ret = 1;
		if (qwx_pci_intr_data_event(psc, &psc->event_rings[1], 0))
			ret = 1;

		for (i = 0; i < sc->hw_params.ce_count; i++) {
			struct qwx_ce_pipe *ce_pipe = &sc->ce.ce_pipe[i];

			if (qwx_ce_intr(ce_pipe))
				ret = 1;
		}

		if (test_bit(ATH11K_FLAG_EXT_IRQ_ENABLED, sc->sc_flags)) {
			for (i = 0; i < nitems(sc->ext_irq_grp); i++) {
				if (qwx_dp_service_srng(sc, i))
					ret = 1;
			}
		}
	}

	return ret;
}
