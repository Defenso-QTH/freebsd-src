/*
 * This file comes from OpenBSD's sys/dev/pci/if_qwx_pci.c revision 1.41
 * (the MHI host code of qwx(4)), turned into the MHI host core of the
 * LinuxKPI.
 */

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

/*
 * The MHI (Modem Host Interface) host core of the LinuxKPI: firmware
 * download (BHI, BHIe), the device state machine, the command, event and
 * transfer rings, and the client devices of the Linux MHI bus, for
 * ath11k(4) and QRTR (linux_qrtr.c).  The interface is in linux/mhi.h;
 * register and ring definitions in linux_mhi_reg.h; interrupts, the module
 * handler and the sysctls in linux_mhi_bus.c.
 *
 * Locking, in lock order:
 *  - lkpi_mhi_bus_lock (sx): the lists of controllers, drivers and client
 *    devices; held while drivers are probed and removed.
 *  - psc->mem_lock (sx): the DMA memory of a controller, for the state
 *    dump against mhi_unprepare_after_power_down().
 *  - an event ring's lock: processing of that ring.  Transfer callbacks
 *    run with it held (they must not sleep), so taking and dropping it
 *    waits for callbacks in progress.
 *  - a transfer ring's lock: the ring and its buffers; never held across
 *    a driver callback, so mhi_queue_buf() works from the callbacks.
 *  - psc->lock: device and host state (EE, MHI state, flags, device wake
 *    references, command status), and the command ring.
 * Register accesses go through the controller's read_reg and write_reg
 * callbacks and may happen with any of these held.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/libkern.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/sx.h>
#include <sys/taskqueue.h>

#include <linux/compat.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/slab.h>

#include "linux_mhi_reg.h"

#define	mhi_printf(psc, ...)	device_printf((psc)->dev->bsddev, __VA_ARGS__)
#define	DNPRINTF(psc, fmt, ...) do {					\
	if (mhi_fbsd_debug)						\
		mhi_printf((psc), fmt, ##__VA_ARGS__);			\
} while (0)

/* DMA memory shared with the device. */
struct lkpi_dmamem {
	void			*kva;
	dma_addr_t		 dva;
	size_t			 size;
};
#define	MHI_DMA_KVA(_adm)	((_adm)->kva)
#define	MHI_DMA_DVA(_adm)	((_adm)->dva)
#define	MHI_DMA_LEN(_adm)	((_adm)->size)

#define MHI_CMD_RING_MAX_ELEMENTS	128
/* Receive buffer size when the configuration does not give one. */
#define	MHI_XFER_MAX_DATA_SIZE		0xffff
#define	MHI_DEFAULT_TIMEOUT_MS		2000
#define	MHI_POLL_TICKS			MAX(1, hz / 100)	/* 10 ms */

struct lkpi_xfer_data {
	void			*buf;
	dma_addr_t		 dma;
	size_t			 len;
};

struct lkpi_mhi_softc;

struct lkpi_mhi_xfer_ring {
	struct mtx		lock;
	struct lkpi_dmamem	*dmamem;
	size_t			size;
	uint32_t		mhi_chan_id;
	uint32_t		mhi_chan_state;
	uint32_t		mhi_chan_direction;
	uint32_t		mhi_chan_event_ring_index;
	uint32_t		db_addr;
	uint32_t		cmd_status;
	int			num_elements;
	int			queued;
	bool			db_pending;	/* doorbell held back in M3 */
	struct lkpi_xfer_data	*data;
	uint64_t		rp;
	uint64_t		wp;
	struct mhi_chan_ctxt	*chan_ctxt;

	/* From the channel configuration. */
	const char		*name;
	enum dma_data_direction	dir;
	uint32_t		ee_mask;
	uint32_t		brstmode;
	uint32_t		pollcfg;

	struct mhi_device	*mhi_dev;	/* client device using it */
	struct lkpi_mhi_softc	*psc;
};

struct lkpi_mhi_event_ring {
	struct mtx		lock;
	struct lkpi_dmamem	*dmamem;
	size_t			size;
	uint32_t		mhi_er_type;
	uint32_t		mhi_er_irq;
	uint32_t		mhi_er_irq_moderation_ms;
	uint32_t		db_addr;
	int			num_elements;
	uint64_t		rp;
	uint64_t		wp;
	struct mhi_event_ctxt	*event_ctxt;
	bool			hw_event;
	u_long			nevents;	/* processed since power up */
	struct lkpi_mhi_vec	*vec;
};

struct lkpi_mhi_cmd_ring {
	struct lkpi_dmamem	*dmamem;
	size_t			size;
	uint64_t		rp;
	uint64_t		wp;
	int			num_elements;
	int			queued;
};

/* One interrupt vector; irq[0] also signals BHI and state changes. */
struct lkpi_mhi_vec {
	struct lkpi_mhi_softc	*psc;
	unsigned int		irq;
	bool			bhi;
	bool			requested;
	bool			enabled;
};

struct lkpi_mhi_softc {
	struct mhi_controller	*mhi;
	struct device		*dev;
	int			 index;
	struct list_head	 node;		/* lkpi_mhi_controllers */
	struct list_head	 devices;	/* client devices */
	struct mtx		 lock;
	struct sx		 mem_lock;

	uint32_t		 bhi_off;
	uint32_t		 bhi_ee;
	uint32_t		 bhie_off;
	uint32_t		 mhi_state;
	uint32_t		 max_chan;

	uint64_t		 wake_db;
	int			 wake_refcnt;

	uint32_t		 timeout_ms;
	bool			 prepared;
	bool			 powered;
	bool			 mission;
	bool			 suspended;
	bool			 sys_err;
	bool			 unreachable;
	bool			 fatal;

	/*
	 * DMA memory for AMSS.bin firmware image when it is not downloaded
	 * through BHIe (mhi->fbc_image otherwise).
	 * This memory must remain available to the device until
	 * the device is powered down.
	 */
	struct lkpi_dmamem	*amss_data;
	size_t			 fw_size;

	bool			 rddm_armed;

	struct lkpi_dmamem	*chan_ctxt;
	struct lkpi_dmamem	*event_ctxt;
	struct lkpi_dmamem	*cmd_ctxt;

	struct lkpi_mhi_xfer_ring *xfer_rings;
	int			 nxfer_rings;
	struct lkpi_mhi_xfer_ring **chan_map;	/* channel id -> ring */
	struct lkpi_mhi_event_ring *event_rings;
	int			 nevent_rings;
	struct lkpi_mhi_cmd_ring cmd_ring;

	struct lkpi_mhi_vec	*vecs;
	int			 nvecs;

	struct task		 status_task;
	uint32_t		 status_pending;
};

static struct sx lkpi_mhi_bus_lock;
SX_SYSINIT(lkpi_mhi_bus_lock, &lkpi_mhi_bus_lock, "mhi bus");
static LIST_HEAD(lkpi_mhi_controllers);
static LIST_HEAD(lkpi_mhi_drivers);

static void	lkpi_pci_free_xfer_rings(struct lkpi_mhi_softc *);
static int	lkpi_pci_alloc_xfer_rings(struct lkpi_mhi_softc *);
static void	lkpi_pci_free_event_rings(struct lkpi_mhi_softc *);
static int	lkpi_pci_alloc_event_rings(struct lkpi_mhi_softc *);
static void	lkpi_pci_free_cmd_ring(struct lkpi_mhi_softc *);
static int	lkpi_pci_init_cmd_ring(struct lkpi_mhi_softc *,
		    struct lkpi_mhi_cmd_ring *);
static uint32_t	lkpi_pci_read(struct lkpi_mhi_softc *, uint32_t);
static void	lkpi_pci_write(struct lkpi_mhi_softc *, uint32_t, uint32_t);

static void	lkpi_mhi_ring_doorbell(struct lkpi_mhi_softc *, uint64_t,
		    uint64_t);
static void	lkpi_mhi_device_wake(struct lkpi_mhi_softc *);
static void	lkpi_mhi_device_zzz(struct lkpi_mhi_softc *);
static int	lkpi_mhi_wake_db_clear_valid(struct lkpi_mhi_softc *);
static void	lkpi_mhi_init_xfer_rings(struct lkpi_mhi_softc *);
static void	lkpi_mhi_init_event_rings(struct lkpi_mhi_softc *);
static void	lkpi_mhi_init_cmd_ring(struct lkpi_mhi_softc *);
static void	lkpi_mhi_init_dev_ctxt(struct lkpi_mhi_softc *);
static int	lkpi_mhi_send_cmd(struct lkpi_mhi_softc *, uint32_t, uint32_t);
static void *	lkpi_pci_xfer_ring_get_elem(struct lkpi_mhi_xfer_ring *,
		    uint64_t);
static struct lkpi_xfer_data *lkpi_pci_xfer_ring_get_data(
		    struct lkpi_mhi_xfer_ring *, uint64_t);
static int	lkpi_mhi_start_channel(struct lkpi_mhi_softc *,
		    struct lkpi_mhi_xfer_ring *);
static int	lkpi_mhi_start(struct lkpi_mhi_softc *);
static void	lkpi_mhi_stop(struct lkpi_mhi_softc *);
static int	lkpi_mhi_reset_device(struct lkpi_mhi_softc *, int);
static int	lkpi_mhi_fw_load_handler(struct lkpi_mhi_softc *);
static int	lkpi_mhi_await_device_reset(struct lkpi_mhi_softc *);
static int	lkpi_mhi_await_device_ready(struct lkpi_mhi_softc *);
static int	lkpi_mhi_ready_state_transition(struct lkpi_mhi_softc *);
static void	lkpi_mhi_mission_mode_state_transition(struct lkpi_mhi_softc *);
static void	lkpi_mhi_low_power_mode_state_transition(
		    struct lkpi_mhi_softc *);
static void	lkpi_mhi_set_state(struct lkpi_mhi_softc *, uint32_t);
static void	lkpi_mhi_init_mmio(struct lkpi_mhi_softc *);
static int	lkpi_mhi_fw_load_bhi(struct lkpi_mhi_softc *);
static int	lkpi_mhi_fw_load_bhie(struct lkpi_mhi_softc *,
		    struct image_info *);
static void	lkpi_rddm_prepare(struct lkpi_mhi_softc *);
static void *	lkpi_pci_event_ring_get_elem(struct lkpi_mhi_event_ring *,
		    uint64_t);
static void	lkpi_mhi_state_change(struct lkpi_mhi_softc *, int, int);
static void	lkpi_pci_intr_ctrl_event_mhi(struct lkpi_mhi_softc *,
		    uint32_t);
static void	lkpi_pci_intr_ctrl_event_ee(struct lkpi_mhi_softc *, uint32_t);
static void	lkpi_pci_intr_ctrl_event_cmd_complete(struct lkpi_mhi_softc *,
		    uint64_t, uint32_t);
static int	lkpi_pci_intr_ctrl_event(struct lkpi_mhi_softc *,
		    struct lkpi_mhi_event_ring *, int);
static void	lkpi_pci_intr_data_event_tx(struct lkpi_mhi_softc *,
		    struct mhi_ring_element *);
static int	lkpi_pci_intr_data_event(struct lkpi_mhi_softc *,
		    struct lkpi_mhi_event_ring *, int);
static int	lkpi_pci_intr(struct lkpi_mhi_softc *);
static irqreturn_t lkpi_pci_intr_vec(int, void *);

static void	lkpi_mhi_dump_state(struct lkpi_mhi_softc *, const char *);
static int	lkpi_mhi_check_mission(struct lkpi_mhi_softc *, void *);
static void	lkpi_mhi_init_device(struct lkpi_mhi_softc *,
		    struct mhi_device *, const char *);
static void	lkpi_mhi_xfer_ring_flush(struct lkpi_mhi_softc *,
		    struct lkpi_mhi_xfer_ring *, bool);

/* ------------------------------------------------------------------ */
/* DMA memory and waits. */

static struct lkpi_dmamem *
lkpi_dmamem_alloc(struct lkpi_mhi_softc *psc, size_t size, size_t align)
{
	struct lkpi_dmamem *adm;
	size_t a;

	adm = kzalloc(sizeof(*adm), GFP_KERNEL);
	if (adm == NULL)
		return NULL;
	adm->size = size;
	adm->kva = dma_alloc_coherent(psc->dev, size, &adm->dva,
	    GFP_KERNEL | __GFP_ZERO);
	if (adm->kva == NULL) {
		kfree(adm);
		return NULL;
	}
	for (a = 1; a < align; a <<= 1)
		;
	if (align != 0 && (adm->dva & (a - 1)) != 0) {
		mhi_printf(psc, "DMA memory at %#jx not aligned to %zu\n",
		    (uintmax_t)adm->dva, a);
		dma_free_coherent(psc->dev, size, adm->kva, adm->dva);
		kfree(adm);
		return NULL;
	}
	return adm;
}

static void
lkpi_dmamem_free(struct lkpi_mhi_softc *psc, struct lkpi_dmamem *adm)
{
	if (adm == NULL)
		return;
	dma_free_coherent(psc->dev, adm->size, adm->kva, adm->dva);
	kfree(adm);
}

/* The MHI state field of MHI_STATUS. */
static uint32_t
lkpi_mhi_status_state(uint32_t reg)
{
	return ((reg & MHI_STATUS_MHISTATE_MASK) >> MHI_STATUS_MHISTATE_SHFT);
}

static int
lkpi_mhi_timeout_ticks(struct lkpi_mhi_softc *psc)
{
	return ((int)MAX(1, ((uint64_t)psc->timeout_ms * hz + 999) / 1000));
}

/*
 * Wait until check() reports done (> 0) or failure (< 0), for at most
 * timeout_ms.  check() runs with psc->lock held, on every wakeup of wchan
 * and every 10 ms; poll(), if given, runs before it without the lock.
 */
static int
lkpi_mhi_wait(struct lkpi_mhi_softc *psc, void *wchan, const char *wmesg,
    int (*check)(struct lkpi_mhi_softc *, void *), void *arg,
    void (*poll)(struct lkpi_mhi_softc *))
{
	int end, left, ret;

	end = ticks + lkpi_mhi_timeout_ticks(psc);
	for (;;) {
		if (poll != NULL)
			poll(psc);
		mtx_lock(&psc->lock);
		ret = check(psc, arg);
		if (ret != 0) {
			mtx_unlock(&psc->lock);
			break;
		}
		left = end - ticks;
		if (left <= 0) {
			mtx_unlock(&psc->lock);
			ret = -ETIMEDOUT;
			break;
		}
		mtx_sleep(wchan, &psc->lock, 0, wmesg,
		    MIN(left, MHI_POLL_TICKS));
		mtx_unlock(&psc->lock);
	}
	return (ret < 0 ? ret : 0);
}

/* Queue a status callback for the controller driver.  psc->lock held. */
static void
lkpi_mhi_queue_status(struct lkpi_mhi_softc *psc, enum mhi_callback cb)
{
	mtx_assert(&psc->lock, MA_OWNED);
	psc->status_pending |= 1u << cb;
	taskqueue_enqueue(taskqueue_thread, &psc->status_task);
}

static void
lkpi_mhi_status_task(void *arg, int pending __unused)
{
	static const enum mhi_callback order[] = {
		MHI_CB_SYS_ERROR, MHI_CB_EE_RDDM, MHI_CB_FATAL_ERROR,
	};
	struct lkpi_mhi_softc *psc = arg;
	uint32_t p;
	int i;

	linux_set_current(curthread);
	mtx_lock(&psc->lock);
	p = psc->status_pending;
	psc->status_pending = 0;
	mtx_unlock(&psc->lock);
	for (i = 0; i < nitems(order); i++) {
		if (p & (1u << order[i]))
			psc->mhi->status_cb(psc->mhi, order[i]);
	}
}

/* ------------------------------------------------------------------ */
/* Rings. */

static void
lkpi_pci_free_xfer_rings(struct lkpi_mhi_softc *psc)
{
	int i;

	for (i = 0; i < psc->nxfer_rings; i++) {
		struct lkpi_mhi_xfer_ring *ring = &psc->xfer_rings[i];
		if (ring->dmamem) {
			lkpi_dmamem_free(psc, ring->dmamem);
			ring->dmamem = NULL;
		}
		kfree(ring->data);
		ring->data = NULL;
		ring->chan_ctxt = NULL;
	}
}

static int
lkpi_pci_alloc_xfer_ring(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring)
{
	size_t size;

	size = sizeof(struct mhi_ring_element) * ring->num_elements;
	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = lkpi_dmamem_alloc(psc, size, size);
	if (ring->dmamem == NULL)
		return -ENOMEM;

	ring->size = size;
	ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
	ring->queued = 0;

	ring->data = kcalloc(ring->num_elements, sizeof(*ring->data),
	    GFP_KERNEL);
	if (ring->data == NULL) {
		lkpi_dmamem_free(psc, ring->dmamem);
		ring->dmamem = NULL;
		return -ENOMEM;
	}
	return 0;
}

static int
lkpi_pci_alloc_xfer_rings(struct lkpi_mhi_softc *psc)
{
	int i, ret;

	for (i = 0; i < psc->nxfer_rings; i++) {
		ret = lkpi_pci_alloc_xfer_ring(psc, &psc->xfer_rings[i]);
		if (ret)
			goto fail;
	}
	return 0;
fail:
	lkpi_pci_free_xfer_rings(psc);
	return ret;
}

static void
lkpi_pci_free_event_rings(struct lkpi_mhi_softc *psc)
{
	int i;

	for (i = 0; i < psc->nevent_rings; i++) {
		struct lkpi_mhi_event_ring *ring = &psc->event_rings[i];
		struct lkpi_dmamem *adm;

		mtx_lock(&ring->lock);
		adm = ring->dmamem;
		ring->dmamem = NULL;
		ring->event_ctxt = NULL;
		mtx_unlock(&ring->lock);
		lkpi_dmamem_free(psc, adm);
	}
}

static int
lkpi_pci_alloc_event_ring(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_event_ring *ring)
{
	size_t size;

	size = sizeof(struct mhi_ring_element) * ring->num_elements;
	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = lkpi_dmamem_alloc(psc, size, size);
	if (ring->dmamem == NULL)
		return -ENOMEM;

	ring->size = size;
	return 0;
}

static int
lkpi_pci_alloc_event_rings(struct lkpi_mhi_softc *psc)
{
	int i, ret;

	for (i = 0; i < psc->nevent_rings; i++) {
		ret = lkpi_pci_alloc_event_ring(psc, &psc->event_rings[i]);
		if (ret)
			goto fail;
	}
	return 0;
fail:
	lkpi_pci_free_event_rings(psc);
	return ret;
}

static void
lkpi_pci_free_cmd_ring(struct lkpi_mhi_softc *psc)
{
	struct lkpi_mhi_cmd_ring *ring = &psc->cmd_ring;

	if (ring->dmamem)
		lkpi_dmamem_free(psc, ring->dmamem);

	memset(ring, 0, sizeof(*ring));
}

static int
lkpi_pci_init_cmd_ring(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_cmd_ring *ring)
{
	memset(ring, 0, sizeof(*ring));

	ring->num_elements = MHI_CMD_RING_MAX_ELEMENTS;
	ring->size = sizeof(struct mhi_ring_element) * ring->num_elements;

	/* Hardware requires that rings are aligned to ring size. */
	ring->dmamem = lkpi_dmamem_alloc(psc, ring->size, ring->size);
	if (ring->dmamem == NULL)
		return -ENOMEM;

	return 0;
}

static uint32_t
lkpi_pci_read(struct lkpi_mhi_softc *psc, uint32_t addr)
{
	struct mhi_controller *mhi = psc->mhi;
	uint32_t val;

	if ((size_t)addr + 4 > mhi->reg_len) {
		mhi_printf(psc, "MHI register %#x beyond %#zx\n", addr,
		    mhi->reg_len);
		return (0xffffffff);
	}
	if (mhi->read_reg(mhi, (uint8_t *)mhi->regs + addr, &val) != 0)
		return (0xffffffff);
	return (val);
}

static void
lkpi_pci_write(struct lkpi_mhi_softc *psc, uint32_t addr, uint32_t val)
{
	struct mhi_controller *mhi = psc->mhi;

	if ((size_t)addr + 4 > mhi->reg_len) {
		mhi_printf(psc, "MHI register %#x beyond %#zx\n", addr,
		    mhi->reg_len);
		return;
	}
	mhi->write_reg(mhi, (uint8_t *)mhi->regs + addr, val);
}

/* ------------------------------------------------------------------ */
/* Firmware images. */

static void
lkpi_mhi_free_image(struct lkpi_mhi_softc *psc, struct image_info **imgp)
{
	struct image_info *img = *imgp;
	u32 i;

	if (img == NULL)
		return;
	for (i = 0; i < img->entries; i++) {
		if (img->mhi_buf[i].buf != NULL)
			dma_free_coherent(psc->dev, img->mhi_buf[i].len,
			    img->mhi_buf[i].buf, img->mhi_buf[i].dma_addr);
	}
	kfree(img->mhi_buf);
	kfree(img);
	*imgp = NULL;
}

/*
 * Allocate a BHIe image of len bytes in seg_len segments, followed by the
 * vector table describing them.
 */
static int
lkpi_mhi_alloc_image(struct lkpi_mhi_softc *psc, struct image_info **imgp,
    size_t len)
{
	struct mhi_controller *mhi = psc->mhi;
	struct image_info *img;
	struct mhi_dma_vec_entry *vec;
	struct mhi_buf *mb;
	const size_t chunk_size = mhi->seg_len != 0 ? mhi->seg_len :
	    MHI_DMA_VEC_CHUNK_SIZE;
	size_t nseg, remain;
	int i;

	nseg = howmany(len, chunk_size);
	if (nseg == 0) {
		mhi_printf(psc, "BHIE data too short, have only %zu bytes\n",
		    len);
		return -EINVAL;
	}

	img = kzalloc(sizeof(*img), GFP_KERNEL);
	if (img == NULL)
		return -ENOMEM;
	img->mhi_buf = kcalloc(nseg + 1, sizeof(*img->mhi_buf), GFP_KERNEL);
	if (img->mhi_buf == NULL) {
		kfree(img);
		return -ENOMEM;
	}
	img->entries = nseg + 1;

	remain = len;
	for (i = 0; i < nseg; i++) {
		mb = &img->mhi_buf[i];
		mb->len = MIN(remain, chunk_size);
		mb->buf = dma_alloc_coherent(psc->dev, mb->len, &mb->dma_addr,
		    GFP_KERNEL);
		if (mb->buf == NULL)
			goto fail;
		remain -= mb->len;
	}

	mb = &img->mhi_buf[nseg];
	mb->len = nseg * sizeof(*vec);
	mb->buf = dma_alloc_coherent(psc->dev, mb->len, &mb->dma_addr,
	    GFP_KERNEL | __GFP_ZERO);
	if (mb->buf == NULL)
		goto fail;

	/* Create vector which controls chunk-wise DMA copy in hardware. */
	vec = mb->buf;
	for (i = 0; i < nseg; i++) {
		vec[i].paddr = htole64(img->mhi_buf[i].dma_addr);
		vec[i].size = htole64(img->mhi_buf[i].len);
	}
	wmb();

	*imgp = img;
	return 0;
fail:
	mhi_printf(psc, "could not allocate BHIE DMA buffers\n");
	*imgp = img;
	lkpi_mhi_free_image(psc, imgp);
	return -ENOMEM;
}

/* Copy between a linear buffer and the data segments of an image. */
static void
lkpi_mhi_image_copy(struct image_info *img, void *data, size_t len,
    bool to_image)
{
	size_t n, off;
	u32 i;

	off = 0;
	for (i = 0; i + 1 < img->entries && off < len; i++) {
		n = MIN(len - off, img->mhi_buf[i].len);
		if (to_image)
			memcpy(img->mhi_buf[i].buf, (uint8_t *)data + off, n);
		else
			memcpy((uint8_t *)data + off, img->mhi_buf[i].buf, n);
		off += n;
	}
}

/*
 * Get the firmware image (fw_data, or fw_image through firmware(9)) into
 * DMA memory: the BHIe image if the full image is downloaded through BHIe,
 * one buffer for BHI otherwise.
 */
static int
lkpi_mhi_load_image(struct lkpi_mhi_softc *psc)
{
	struct mhi_controller *mhi = psc->mhi;
	const struct firmware *fw = NULL;
	const u8 *data;
	size_t len;
	int ret;

	if (mhi->fw_data != NULL && mhi->fw_sz > 0) {
		data = mhi->fw_data;
		len = mhi->fw_sz;
	} else if (mhi->fw_image != NULL) {
		ret = request_firmware(&fw, mhi->fw_image, psc->dev);
		if (ret) {
			mhi_printf(psc, "could not read %s (error %d)\n",
			    mhi->fw_image, ret);
			return ret;
		}
		data = fw->data;
		len = fw->size;
	} else {
		mhi_printf(psc, "no firmware image\n");
		return -EINVAL;
	}

	if (len < mhi->sbl_size) {
		mhi_printf(psc, "%s is too short, have only %zu bytes\n",
		    mhi->fw_image != NULL ? mhi->fw_image : "firmware", len);
		ret = -EINVAL;
		goto out;
	}

	if (mhi->fbc_download) {
		ret = lkpi_mhi_alloc_image(psc, &mhi->fbc_image, len);
		if (ret == 0)
			lkpi_mhi_image_copy(mhi->fbc_image, __DECONST(u8 *,
			    data), len, true);
	} else {
		psc->amss_data = lkpi_dmamem_alloc(psc, len, 0);
		if (psc->amss_data == NULL) {
			mhi_printf(psc,
			    "could not allocate BHI DMA data buffer\n");
			ret = -ENOMEM;
		} else {
			memcpy(MHI_DMA_KVA(psc->amss_data), data, len);
			ret = 0;
		}
	}
	if (ret == 0)
		psc->fw_size = len;
out:
	if (fw != NULL)
		release_firmware(fw);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Interrupts. */

static void
lkpi_mhi_free_irqs(struct lkpi_mhi_softc *psc)
{
	int i;

	for (i = 0; i < psc->nvecs; i++) {
		struct lkpi_mhi_vec *vec = &psc->vecs[i];

		if (!vec->requested)
			continue;
		mhi_fbsd_free_irq(vec->irq, vec);
		vec->requested = false;
		vec->enabled = false;
	}
}

static void
lkpi_mhi_disable_irqs(struct lkpi_mhi_softc *psc)
{
	int i;

	for (i = 0; i < psc->nvecs; i++) {
		struct lkpi_mhi_vec *vec = &psc->vecs[i];

		if (!vec->enabled)
			continue;
		/* Waits for a running handler. */
		mhi_fbsd_disable_irq(vec->irq);
		vec->enabled = false;
	}
}

static int
lkpi_mhi_request_irqs(struct lkpi_mhi_softc *psc)
{
	int i, ret;

	for (i = 0; i < psc->nvecs; i++) {
		struct lkpi_mhi_vec *vec = &psc->vecs[i];

		ret = mhi_fbsd_request_irq(vec->irq, lkpi_pci_intr_vec, NULL,
		    psc->mhi->irq_flags, "mhi", vec);
		if (ret) {
			mhi_printf(psc, "could not request irq %u: %d\n",
			    vec->irq, ret);
			lkpi_mhi_free_irqs(psc);
			return ret;
		}
		vec->requested = true;
		vec->enabled = true;
	}
	return 0;
}

/*
 * MHI
 */

int
linuxkpi_mhi_register_controller(struct mhi_controller *mhi,
    const struct mhi_controller_config *config)
{
	struct lkpi_mhi_softc *psc, *p;
	const struct mhi_channel_config *ch;
	const struct mhi_event_config *ev;
	struct lkpi_mhi_xfer_ring *ring;
	struct lkpi_mhi_event_ring *er;
	struct mhi_device *mdev;
	unsigned int irq;
	int i, j, index;

	if (mhi == NULL || config == NULL || mhi->cntrl_dev == NULL ||
	    mhi->regs == NULL || mhi->irq == NULL || mhi->nr_irqs <= 0 ||
	    mhi->runtime_get == NULL || mhi->runtime_put == NULL ||
	    mhi->status_cb == NULL || mhi->read_reg == NULL ||
	    mhi->write_reg == NULL)
		return -EINVAL;
	if (config->num_events == 0 || config->event_cfg == NULL ||
	    (config->num_channels != 0 && config->ch_cfg == NULL) ||
	    config->max_channels == 0 ||
	    config->max_channels >
	    (MHI_TRE_CMD_CHID_MASK >> MHI_TRE_CMD_CHID_SHFT) + 1)
		goto bad;
	for (i = 0; i < config->num_events; i++) {
		ev = &config->event_cfg[i];
		if (ev->irq >= mhi->nr_irqs || ev->num_elements < 2)
			goto bad;
	}
	for (i = 0; i < config->num_channels; i++) {
		ch = &config->ch_cfg[i];
		if (ch->name == NULL || ch->num >= config->max_channels ||
		    ch->event_ring >= config->num_events ||
		    ch->num_elements < 2 ||
		    (ch->dir != DMA_TO_DEVICE && ch->dir != DMA_FROM_DEVICE))
			goto bad;
		for (j = 0; j < i; j++) {
			if (config->ch_cfg[j].num == ch->num)
				goto bad;
		}
	}

	psc = kzalloc(sizeof(*psc), GFP_KERNEL);
	if (psc == NULL)
		return -ENOMEM;
	psc->mhi = mhi;
	psc->dev = mhi->cntrl_dev;
	psc->max_chan = config->max_channels;
	INIT_LIST_HEAD(&psc->devices);
	mtx_init(&psc->lock, "mhi state", NULL, MTX_DEF);
	sx_init(&psc->mem_lock, "mhi memory");
	TASK_INIT(&psc->status_task, 0, lkpi_mhi_status_task, psc);

	/* The interrupt vectors: irq[0], then those of the event rings. */
	psc->vecs = kcalloc(config->num_events + 1, sizeof(*psc->vecs),
	    GFP_KERNEL);
	psc->event_rings = kcalloc(config->num_events,
	    sizeof(*psc->event_rings), GFP_KERNEL);
	psc->xfer_rings = kcalloc(MAX(config->num_channels, 1),
	    sizeof(*psc->xfer_rings), GFP_KERNEL);
	psc->chan_map = kcalloc(psc->max_chan, sizeof(*psc->chan_map),
	    GFP_KERNEL);
	if (psc->vecs == NULL || psc->event_rings == NULL ||
	    psc->xfer_rings == NULL || psc->chan_map == NULL) {
		kfree(psc->vecs);
		kfree(psc->event_rings);
		kfree(psc->xfer_rings);
		kfree(psc->chan_map);
		sx_destroy(&psc->mem_lock);
		mtx_destroy(&psc->lock);
		kfree(psc);
		return -ENOMEM;
	}
	psc->vecs[0].psc = psc;
	psc->vecs[0].irq = mhi->irq[0];
	psc->vecs[0].bhi = true;
	psc->nvecs = 1;

	psc->nevent_rings = config->num_events;
	for (i = 0; i < psc->nevent_rings; i++) {
		ev = &config->event_cfg[i];
		er = &psc->event_rings[i];
		mtx_init(&er->lock, "mhi event ring", NULL, MTX_DEF);
		er->num_elements = ev->num_elements;
		er->mhi_er_type = ev->data_type;
		er->mhi_er_irq = ev->irq;
		er->mhi_er_irq_moderation_ms = ev->irq_moderation_ms;
		er->hw_event = ev->hardware_event;
		/* Vectors with the same interrupt number are one. */
		irq = mhi->irq[ev->irq];
		for (j = 0; j < psc->nvecs; j++) {
			if (psc->vecs[j].irq == irq)
				break;
		}
		if (j == psc->nvecs) {
			psc->vecs[j].psc = psc;
			psc->vecs[j].irq = irq;
			psc->nvecs++;
		}
		er->vec = &psc->vecs[j];
	}

	psc->nxfer_rings = config->num_channels;
	for (i = 0; i < psc->nxfer_rings; i++) {
		ch = &config->ch_cfg[i];
		ring = &psc->xfer_rings[i];
		mtx_init(&ring->lock, "mhi xfer ring", NULL, MTX_DEF);
		ring->psc = psc;
		ring->name = ch->name;
		ring->mhi_chan_id = ch->num;
		ring->dir = ch->dir;
		ring->mhi_chan_direction = ch->dir == DMA_TO_DEVICE ?
		    MHI_CHAN_TYPE_OUTBOUND : MHI_CHAN_TYPE_INBOUND;
		ring->mhi_chan_event_ring_index = ch->event_ring;
		ring->num_elements = ch->num_elements;
		ring->ee_mask = ch->ee_mask;
		ring->brstmode = ch->doorbell == MHI_DB_BRST_ENABLE ?
		    MHI_CHAN_CTX_BRSTMODE_ENABLE :
		    MHI_CHAN_CTX_BRSTMODE_DISABLE;
		ring->pollcfg = ch->pollcfg;
		ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
		psc->chan_map[ch->num] = ring;
	}

	if (mhi->timeout_ms == 0)
		mhi->timeout_ms = config->timeout_ms != 0 ?
		    config->timeout_ms : MHI_DEFAULT_TIMEOUT_MS;
	psc->timeout_ms = mhi->timeout_ms;
	mhi->buffer_len = config->buf_len != 0 ? config->buf_len :
	    MHI_XFER_MAX_DATA_SIZE;
	mhi->lkpi_priv = psc;

	sx_xlock(&lkpi_mhi_bus_lock);
	for (index = 0;; index++) {
		list_for_each_entry(p, &lkpi_mhi_controllers, node) {
			if (p->index == index)
				break;
		}
		if (&p->node == &lkpi_mhi_controllers)
			break;
	}
	psc->index = index;
	list_add_tail(&psc->node, &lkpi_mhi_controllers);
	sx_xunlock(&lkpi_mhi_bus_lock);

	/* The controller's own device. */
	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (mdev == NULL) {
		linuxkpi_mhi_unregister_controller(mhi);
		return -ENOMEM;
	}
	lkpi_mhi_init_device(psc, mdev, NULL);
	mdev->lkpi_is_controller = true;
	mhi->mhi_dev = mdev;
	return 0;
bad:
	printf("mhi: invalid controller configuration\n");
	return -EINVAL;
}

/* ------------------------------------------------------------------ */
/* Client devices and drivers. */

static void
lkpi_mhi_dev_release(struct kobject *kobj)
{
	struct mhi_device *mdev;

	mdev = container_of(kobj, struct mhi_device, dev.kobj);
	kfree(mdev);
}

static const struct kobj_type lkpi_mhi_dev_ktype = {
	.release = lkpi_mhi_dev_release,
};

static void
lkpi_mhi_init_device(struct lkpi_mhi_softc *psc, struct mhi_device *mdev,
    const char *name)
{
	kobject_init(&mdev->dev.kobj, &lkpi_mhi_dev_ktype);
	mdev->dev.parent = psc->dev;
	/* Messages go to the controller's device. */
	mdev->dev.bsddev = psc->dev->bsddev;
	INIT_LIST_HEAD(&mdev->dev.irqents);
	spin_lock_init(&mdev->dev.devres_lock);
	INIT_LIST_HEAD(&mdev->dev.devres_head);
	mdev->mhi_cntrl = psc->mhi;
	mdev->name = name;
	INIT_LIST_HEAD(&mdev->lkpi_node);
	if (name != NULL)
		kobject_set_name(&mdev->dev.kobj, "mhi%d_%s", psc->index, name);
	else
		kobject_set_name(&mdev->dev.kobj, "mhi%d", psc->index);
}

static const struct mhi_device_id *
lkpi_mhi_match(struct mhi_driver *drv, struct mhi_device *mdev)
{
	const struct mhi_device_id *id;

	if (drv->id_table == NULL)
		return (NULL);
	for (id = drv->id_table; id->chan[0] != '\0'; id++) {
		if (strcmp(id->chan, mdev->name) == 0)
			return (id);
	}
	return (NULL);
}

/* Bind drv to the unbound client device mdev if it matches.  Bus locked. */
static void
lkpi_mhi_probe_device(struct mhi_device *mdev, struct mhi_driver *drv)
{
	const struct mhi_device_id *id;
	int error;

	sx_assert(&lkpi_mhi_bus_lock, SA_XLOCKED);
	if (mdev->lkpi_drv != NULL || drv->probe == NULL)
		return;
	id = lkpi_mhi_match(drv, mdev);
	if (id == NULL)
		return;
	/* The transfer callbacks of the driver may run from here on. */
	mdev->lkpi_drv = drv;
	mdev->id = id;
	mdev->dev.driver = &drv->driver;
	error = drv->probe(mdev, id);
	if (error != 0) {
		device_printf(mdev->dev.bsddev, "%s: probe of %s failed: %d\n",
		    dev_name(&mdev->dev), drv->driver.name, error);
		mdev->lkpi_drv = NULL;
		mdev->id = NULL;
		mdev->dev.driver = NULL;
	} else {
		DNPRINTF(mdev->mhi_cntrl->lkpi_priv, "%s: bound to %s\n",
		    dev_name(&mdev->dev), drv->driver.name);
	}
}

/* Unbind the driver of a client device.  Bus locked. */
static void
lkpi_mhi_unbind_device(struct mhi_device *mdev)
{
	struct mhi_driver *drv = mdev->lkpi_drv;

	sx_assert(&lkpi_mhi_bus_lock, SA_XLOCKED);
	if (drv == NULL)
		return;
	if (drv->remove != NULL)
		drv->remove(mdev);
	mdev->lkpi_drv = NULL;
	mdev->id = NULL;
	mdev->dev.driver = NULL;
}

/*
 * Entering an execution environment: create a client device for each
 * channel name the channels allow there (or take back the ones kept by
 * mhi_power_down_keep_dev()), and bind the registered drivers to them.
 */
static void
lkpi_mhi_create_devices(struct lkpi_mhi_softc *psc)
{
	struct lkpi_mhi_xfer_ring *ring;
	struct mhi_device *mdev;
	struct mhi_driver *drv;
	uint32_t ee;
	int i;

	mtx_lock(&psc->lock);
	ee = psc->bhi_ee;
	mtx_unlock(&psc->lock);

	sx_xlock(&lkpi_mhi_bus_lock);
	for (i = 0; i < psc->nxfer_rings; i++) {
		ring = &psc->xfer_rings[i];
		if (ee >= 32 || (ring->ee_mask & (1u << ee)) == 0 ||
		    ring->mhi_dev != NULL)
			continue;
		list_for_each_entry(mdev, &psc->devices, lkpi_node) {
			if (strcmp(mdev->name, ring->name) == 0)
				break;
		}
		if (&mdev->lkpi_node == &psc->devices) {
			mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
			if (mdev == NULL) {
				mhi_printf(psc, "no memory for device %s\n",
				    ring->name);
				continue;
			}
			lkpi_mhi_init_device(psc, mdev, ring->name);
			list_add_tail(&mdev->lkpi_node, &psc->devices);
		}
		if (ring->dir == DMA_TO_DEVICE)
			mdev->lkpi_ul_chan = ring;
		else
			mdev->lkpi_dl_chan = ring;
		mtx_lock(&ring->lock);
		ring->mhi_dev = mdev;
		mtx_unlock(&ring->lock);
	}
	list_for_each_entry(mdev, &psc->devices, lkpi_node) {
		list_for_each_entry(drv, &lkpi_mhi_drivers, lkpi_node) {
			if (mdev->lkpi_drv != NULL)
				break;
			lkpi_mhi_probe_device(mdev, drv);
		}
	}
	sx_xunlock(&lkpi_mhi_bus_lock);
}

/*
 * Unbind the drivers of the client devices; unless keep, also destroy
 * the devices.
 */
static void
lkpi_mhi_destroy_devices(struct lkpi_mhi_softc *psc, bool keep)
{
	struct mhi_device *mdev, *tmp;
	struct lkpi_mhi_xfer_ring *ring;
	int i;

	sx_xlock(&lkpi_mhi_bus_lock);
	list_for_each_entry_safe(mdev, tmp, &psc->devices, lkpi_node) {
		lkpi_mhi_unbind_device(mdev);
		if (keep)
			continue;
		list_del(&mdev->lkpi_node);
		for (i = 0; i < psc->nxfer_rings; i++) {
			ring = &psc->xfer_rings[i];
			mtx_lock(&ring->lock);
			if (ring->mhi_dev == mdev)
				ring->mhi_dev = NULL;
			mtx_unlock(&ring->lock);
		}
		put_device(&mdev->dev);
	}
	sx_xunlock(&lkpi_mhi_bus_lock);
}

int
linuxkpi_mhi_driver_register(struct mhi_driver *drv)
{
	struct lkpi_mhi_softc *psc;
	struct mhi_device *mdev;

	if (drv == NULL || drv->id_table == NULL || drv->probe == NULL)
		return -EINVAL;
	sx_xlock(&lkpi_mhi_bus_lock);
	list_add_tail(&drv->lkpi_node, &lkpi_mhi_drivers);
	list_for_each_entry(psc, &lkpi_mhi_controllers, node) {
		list_for_each_entry(mdev, &psc->devices, lkpi_node)
			lkpi_mhi_probe_device(mdev, drv);
	}
	sx_xunlock(&lkpi_mhi_bus_lock);
	return 0;
}

void
linuxkpi_mhi_driver_unregister(struct mhi_driver *drv)
{
	struct lkpi_mhi_softc *psc;
	struct mhi_device *mdev;

	sx_xlock(&lkpi_mhi_bus_lock);
	list_for_each_entry(psc, &lkpi_mhi_controllers, node) {
		list_for_each_entry(mdev, &psc->devices, lkpi_node) {
			if (mdev->lkpi_drv == drv)
				lkpi_mhi_unbind_device(mdev);
		}
	}
	list_del(&drv->lkpi_node);
	sx_xunlock(&lkpi_mhi_bus_lock);
}

/* Tell the drivers of the client devices about a power state change. */
static void
lkpi_mhi_notify_clients(struct lkpi_mhi_softc *psc, enum mhi_callback cb)
{
	struct mhi_device *mdev;

	sx_slock(&lkpi_mhi_bus_lock);
	list_for_each_entry(mdev, &psc->devices, lkpi_node) {
		if (mdev->lkpi_drv != NULL && mdev->lkpi_drv->status_cb != NULL)
			mdev->lkpi_drv->status_cb(mdev, cb);
	}
	sx_sunlock(&lkpi_mhi_bus_lock);
}

/* ------------------------------------------------------------------ */
/* Controller and power. */

struct mhi_controller *
linuxkpi_mhi_alloc_controller(void)
{
	return (kzalloc(sizeof(struct mhi_controller), GFP_KERNEL));
}

void
linuxkpi_mhi_free_controller(struct mhi_controller *mhi)
{
	kfree(mhi);
}

static int
lkpi_mhi_stop_channel(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring);
static void lkpi_mhi_power_off(struct lkpi_mhi_softc *, bool);

static void
lkpi_mhi_free_mem(struct lkpi_mhi_softc *psc)
{
	struct mhi_controller *mhi = psc->mhi;

	sx_assert(&psc->mem_lock, SA_XLOCKED);
	lkpi_pci_free_xfer_rings(psc);
	lkpi_pci_free_event_rings(psc);
	lkpi_pci_free_cmd_ring(psc);
	lkpi_dmamem_free(psc, psc->chan_ctxt);
	psc->chan_ctxt = NULL;
	lkpi_dmamem_free(psc, psc->event_ctxt);
	psc->event_ctxt = NULL;
	lkpi_dmamem_free(psc, psc->cmd_ctxt);
	psc->cmd_ctxt = NULL;
	lkpi_dmamem_free(psc, psc->amss_data);
	psc->amss_data = NULL;
	lkpi_mhi_free_image(psc, &mhi->fbc_image);
	lkpi_mhi_free_image(psc, &mhi->rddm_image);
	psc->fw_size = 0;
	psc->prepared = false;
}

int
linuxkpi_mhi_prepare_for_power_up(struct mhi_controller *mhi)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	int ret;

	if (psc == NULL)
		return -EINVAL;
	if (psc->prepared)
		return 0;
	if (mhi->fbc_download && mhi->sbl_size == 0)
		return -EINVAL;

	sx_xlock(&psc->mem_lock);
	ret = -ENOMEM;
	psc->chan_ctxt = lkpi_dmamem_alloc(psc,
	    sizeof(struct mhi_chan_ctxt) * psc->max_chan, 0);
	psc->event_ctxt = lkpi_dmamem_alloc(psc,
	    sizeof(struct mhi_event_ctxt) * psc->nevent_rings, 0);
	psc->cmd_ctxt = lkpi_dmamem_alloc(psc,
	    sizeof(struct mhi_cmd_ctxt), 0);
	if (psc->chan_ctxt == NULL || psc->event_ctxt == NULL ||
	    psc->cmd_ctxt == NULL)
		goto fail;
	ret = lkpi_pci_init_cmd_ring(psc, &psc->cmd_ring);
	if (ret)
		goto fail;
	ret = lkpi_pci_alloc_event_rings(psc);
	if (ret)
		goto fail;
	ret = lkpi_pci_alloc_xfer_rings(psc);
	if (ret)
		goto fail;
	ret = lkpi_mhi_load_image(psc);
	if (ret)
		goto fail;
	if (mhi->rddm_size > 0) {
		ret = lkpi_mhi_alloc_image(psc, &mhi->rddm_image,
		    mhi->rddm_size);
		if (ret)
			goto fail;
	}
	psc->prepared = true;
	sx_xunlock(&psc->mem_lock);
	return 0;
fail:
	mhi_printf(psc, "could not allocate MHI memory: %d\n", ret);
	lkpi_mhi_free_mem(psc);
	sx_xunlock(&psc->mem_lock);
	return ret;
}

void
linuxkpi_mhi_unprepare_after_power_down(struct mhi_controller *mhi)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;

	if (psc == NULL)
		return;
	if (psc->powered) {
		mhi_printf(psc, "MHI memory released while powered\n");
		lkpi_mhi_power_off(psc, false);
	}
	sx_xlock(&psc->mem_lock);
	lkpi_mhi_free_mem(psc);
	sx_xunlock(&psc->mem_lock);
}

int
linuxkpi_mhi_sync_power_up(struct mhi_controller *mhi)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	int ret;

	if (psc == NULL || !psc->prepared)
		return -EINVAL;
	if (psc->powered)
		return -EBUSY;

	mtx_lock(&psc->lock);
	psc->timeout_ms = mhi->timeout_ms != 0 ? mhi->timeout_ms :
	    MHI_DEFAULT_TIMEOUT_MS;
	psc->powered = true;
	psc->mission = false;
	psc->suspended = false;
	psc->sys_err = false;
	psc->unreachable = false;
	psc->fatal = false;
	psc->rddm_armed = false;
	psc->wake_db = 0;
	psc->wake_refcnt = 0;
	psc->status_pending = 0;
	mhi->mhi_dev->lkpi_dev_wake = 0;
	mtx_unlock(&psc->lock);

	ret = lkpi_mhi_start(psc);
	if (ret == 0) {
		/* XXX without this delay starting the channels may fail */
		DELAY(1000);
		mtx_lock(&psc->lock);
		psc->mission = true;
		mtx_unlock(&psc->lock);
		DNPRINTF(psc, "%s: mission mode, EE %u\n", __func__,
		    psc->bhi_ee);
		mhi->status_cb(mhi, MHI_CB_EE_MISSION_MODE);
		lkpi_mhi_create_devices(psc);
		return 0;
	}

	mhi_printf(psc, "MHI power up failed: %d\n", ret);
	lkpi_mhi_dump_state(psc, "power up failed");
	lkpi_mhi_power_off(psc, true);
	return ret;
}

int
linuxkpi_mhi_prepare_for_transfer(struct mhi_device *mdev)
{
	struct lkpi_mhi_softc *psc = mdev->mhi_cntrl->lkpi_priv;
	int ret = 0;

	mtx_lock(&psc->lock);
	lkpi_mhi_device_wake(psc);
	mtx_unlock(&psc->lock);

	if (mdev->lkpi_ul_chan != NULL) {
		ret = lkpi_mhi_start_channel(psc, mdev->lkpi_ul_chan);
		if (ret)
			goto done;
	}
	if (mdev->lkpi_dl_chan != NULL) {
		ret = lkpi_mhi_start_channel(psc, mdev->lkpi_dl_chan);
		if (ret && mdev->lkpi_ul_chan != NULL)
			lkpi_mhi_stop_channel(psc, mdev->lkpi_ul_chan);
	}
done:
	mtx_lock(&psc->lock);
	lkpi_mhi_device_zzz(psc);
	mtx_unlock(&psc->lock);
	return ret;
}

/* Can commands still be sent to the device?  psc->lock held. */
static bool
lkpi_mhi_reachable(struct lkpi_mhi_softc *psc)
{
	return (psc->powered && !psc->sys_err && !psc->unreachable &&
	    !psc->suspended);
}

static int
lkpi_mhi_cmd_done(struct lkpi_mhi_softc *psc, void *arg)
{
	struct lkpi_mhi_xfer_ring *ring = arg;

	if (ring->cmd_status != MHI_EV_CC_INVALID)
		return 1;
	if (psc->sys_err || !psc->powered)
		return -EIO;
	return 0;
}

static void
lkpi_mhi_poll_cmd_ring(struct lkpi_mhi_softc *psc)
{
	int i;

	/*
	 * Command completions also when interrupts are off, as they are
	 * when the controller driver powers down with MSI disabled.
	 */
	for (i = 0; i < psc->nevent_rings; i++) {
		if (psc->event_rings[i].mhi_er_type == MHI_ER_CTRL)
			lkpi_pci_intr_ctrl_event(psc, &psc->event_rings[i], 0);
	}
}

/* Send a channel command and wait for its completion. */
static int
lkpi_mhi_chan_cmd(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring, uint32_t cmd)
{
	int ret;

	mtx_lock(&psc->lock);
	if (!lkpi_mhi_reachable(psc)) {
		mtx_unlock(&psc->lock);
		return -EIO;
	}
	ring->cmd_status = MHI_EV_CC_INVALID;
	if (lkpi_mhi_send_cmd(psc, cmd, ring->mhi_chan_id)) {
		mtx_unlock(&psc->lock);
		return -EIO;
	}
	mtx_unlock(&psc->lock);

	ret = lkpi_mhi_wait(psc, &ring->cmd_status, "mhicmd",
	    lkpi_mhi_cmd_done, ring, lkpi_mhi_poll_cmd_ring);
	if (ret == 0 && ring->cmd_status != MHI_EV_CC_SUCCESS)
		ret = -EIO;
	return ret;
}

/*
 * Reset a channel: hold off new buffers and completions, send the reset
 * command, and hand back the buffers still queued.
 */
static void
lkpi_mhi_reset_channel(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring)
{
	uint32_t state;
	int ret;

	mtx_lock(&ring->lock);
	state = ring->mhi_chan_state;
	ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
	mtx_unlock(&ring->lock);
	if (state != MHI_CH_STATE_ENABLED)
		return;

	DNPRINTF(psc, "%s: reset MHI channel %d\n", __func__,
	    ring->mhi_chan_id);
	mtx_lock(&psc->lock);
	if (!lkpi_mhi_reachable(psc)) {
		mtx_unlock(&psc->lock);
	} else {
		mtx_unlock(&psc->lock);
		ret = lkpi_mhi_chan_cmd(psc, ring, MHI_CMD_RESET_CHAN);
		if (ret) {
			mhi_printf(psc, "could not reset MHI channel %d: "
			    "status 0x%x, error %d\n", ring->mhi_chan_id,
			    ring->cmd_status, ret);
		}
	}
	lkpi_mhi_xfer_ring_flush(psc, ring, true);
}

void
linuxkpi_mhi_unprepare_from_transfer(struct mhi_device *mdev)
{
	struct lkpi_mhi_softc *psc = mdev->mhi_cntrl->lkpi_priv;

	mtx_lock(&psc->lock);
	lkpi_mhi_device_wake(psc);
	mtx_unlock(&psc->lock);

	if (mdev->lkpi_ul_chan != NULL)
		lkpi_mhi_reset_channel(psc, mdev->lkpi_ul_chan);
	if (mdev->lkpi_dl_chan != NULL)
		lkpi_mhi_reset_channel(psc, mdev->lkpi_dl_chan);

	mtx_lock(&psc->lock);
	lkpi_mhi_device_zzz(psc);
	mtx_unlock(&psc->lock);
}

/* ------------------------------------------------------------------ */

static int
lkpi_mhi_stop_channel(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring)
{
	int ret = 0;

	mtx_lock(&ring->lock);
	if (ring->mhi_chan_state != MHI_CH_STATE_ENABLED) {
		mtx_unlock(&ring->lock);
		return 0;
	}
	ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
	mtx_unlock(&ring->lock);

	DNPRINTF(psc, "%s: stop MHI channel %d\n", __func__,
	    ring->mhi_chan_id);

	ret = lkpi_mhi_chan_cmd(psc, ring, MHI_CMD_STOP_CHAN);
	if (ret) {
		mhi_printf(psc, "could not stop MHI channel %d: "
		    "status 0x%x\n", ring->mhi_chan_id, ring->cmd_status);
	}
	lkpi_mhi_xfer_ring_flush(psc, ring, false);
	return ret;
}

static void
lkpi_mhi_stop_channels(struct lkpi_mhi_softc *psc)
{
	int i;

	mtx_lock(&psc->lock);
	lkpi_mhi_device_wake(psc);
	mtx_unlock(&psc->lock);

	for (i = 0; i < psc->nxfer_rings; i++)
		lkpi_mhi_stop_channel(psc, &psc->xfer_rings[i]);

	mtx_lock(&psc->lock);
	lkpi_mhi_device_zzz(psc);
	mtx_unlock(&psc->lock);
}

static void
lkpi_mhi_flush_mhi_event_rings(struct lkpi_mhi_softc *psc)
{
	int i;

	for (i = 0; i < psc->nevent_rings; i++) {
		struct lkpi_mhi_event_ring *ring = &psc->event_rings[i];

		if (ring->mhi_er_type == MHI_ER_CTRL)
			lkpi_pci_intr_ctrl_event(psc, ring, 1);
		else
			lkpi_pci_intr_data_event(psc, ring, 1);
	}
}

static int
lkpi_mhi_check_m3(struct lkpi_mhi_softc *psc, void *arg __unused)
{
	uint32_t state;

	if (psc->mhi_state == MHI_STATE_M3)
		return 1;
	state = (lkpi_pci_read(psc, MHI_STATUS) & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;
	if (state == MHI_STATE_M3)
		return 1;
	if (psc->sys_err || state == MHI_STATE_SYS_ERR)
		return -EIO;
	return 0;
}

/*
 * Stop using the device: interrupts off, device reset if graceful,
 * interrupts released.  The DMA memory stays.
 */
static void
lkpi_mhi_power_off(struct lkpi_mhi_softc *psc, bool graceful)
{
	int i;

	lkpi_mhi_disable_irqs(psc);
	lkpi_mhi_flush_mhi_event_rings(psc);
	for (i = 0; i < psc->nxfer_rings; i++) {
		mtx_lock(&psc->xfer_rings[i].lock);
		psc->xfer_rings[i].mhi_chan_state = MHI_CH_STATE_DISABLED;
		mtx_unlock(&psc->xfer_rings[i].lock);
		lkpi_mhi_xfer_ring_flush(psc, &psc->xfer_rings[i], false);
	}
	if (graceful)
		lkpi_mhi_stop(psc);
	lkpi_mhi_free_irqs(psc);

	mtx_lock(&psc->lock);
	psc->powered = false;
	psc->mission = false;
	psc->suspended = false;
	psc->wake_refcnt = 0;
	psc->wake_db = 0;
	psc->mhi->mhi_dev->lkpi_dev_wake = 0;
	mtx_unlock(&psc->lock);
	taskqueue_drain(taskqueue_thread, &psc->status_task);
}

static void
lkpi_pci_power_down(struct lkpi_mhi_softc *psc, bool graceful, bool keep)
{
	uint32_t state;
	bool reachable;

	mtx_lock(&psc->lock);
	if (!graceful)
		psc->unreachable = true;
	mtx_unlock(&psc->lock);

	/* The client drivers go first; they reset their channels. */
	lkpi_mhi_destroy_devices(psc, keep);
	if (!psc->powered)
		return;

	mtx_lock(&psc->lock);
	psc->mission = false;
	reachable = lkpi_mhi_reachable(psc);
	mtx_unlock(&psc->lock);

	if (reachable) {
		lkpi_mhi_stop_channels(psc);

		/*
		 * Ask firmware to transition to M3 before resetting the device
		 * so it can flush state cleanly.  Otherwise stale chip RAM from
		 * the previous boot causes the next firmware boot to silently
		 * drop WMI PDEV commands.
		 */
		state = (lkpi_pci_read(psc, MHI_STATUS) &
		    MHI_STATUS_MHISTATE_MASK) >> MHI_STATUS_MHISTATE_SHFT;
		if (state == MHI_STATE_M0) {
			lkpi_mhi_set_state(psc, MHI_STATE_M3);
			if (lkpi_mhi_wait(psc, &psc->mhi_state, "mhim3",
			    lkpi_mhi_check_m3, NULL, NULL) != 0)
				mhi_printf(psc, "MHI M3 transition timeout "
				    "(state=0x%x)\n", lkpi_mhi_status_state(
				    lkpi_pci_read(psc, MHI_STATUS)));
		}
	}

	lkpi_mhi_power_off(psc, graceful && reachable);
}

void
linuxkpi_mhi_power_down(struct mhi_controller *mhi, bool graceful)
{
	if (mhi->lkpi_priv != NULL)
		lkpi_pci_power_down(mhi->lkpi_priv, graceful, false);
}

void
linuxkpi_mhi_power_down_keep_dev(struct mhi_controller *mhi, bool graceful)
{
	if (mhi->lkpi_priv != NULL)
		lkpi_pci_power_down(mhi->lkpi_priv, graceful, true);
}

void
linuxkpi_mhi_unregister_controller(struct mhi_controller *mhi)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	int i;

	if (psc == NULL)
		return;
	if (psc->powered)
		lkpi_pci_power_down(psc, false, false);
	else
		lkpi_mhi_destroy_devices(psc, false);
	if (psc->prepared)
		linuxkpi_mhi_unprepare_after_power_down(mhi);
	taskqueue_drain(taskqueue_thread, &psc->status_task);

	sx_xlock(&lkpi_mhi_bus_lock);
	list_del(&psc->node);
	sx_xunlock(&lkpi_mhi_bus_lock);

	if (mhi->mhi_dev != NULL)
		put_device(&mhi->mhi_dev->dev);
	mhi->mhi_dev = NULL;
	for (i = 0; i < psc->nxfer_rings; i++)
		mtx_destroy(&psc->xfer_rings[i].lock);
	for (i = 0; i < psc->nevent_rings; i++)
		mtx_destroy(&psc->event_rings[i].lock);
	kfree(psc->xfer_rings);
	kfree(psc->event_rings);
	kfree(psc->chan_map);
	kfree(psc->vecs);
	sx_destroy(&psc->mem_lock);
	mtx_destroy(&psc->lock);
	kfree(psc);
	mhi->lkpi_priv = NULL;
}

static void
lkpi_mhi_ring_doorbell(struct lkpi_mhi_softc *psc, uint64_t db_addr,
    uint64_t val)
{
	lkpi_pci_write(psc, db_addr + 4, val >> 32);
	lkpi_pci_write(psc, db_addr, val & 0xffffffff);
}

/*
 * Device wake: a reference count, the doorbell asserted while it is
 * non-zero.  psc->lock held.
 */
static void
lkpi_mhi_device_wake(struct lkpi_mhi_softc *psc)
{
	mtx_assert(&psc->lock, MA_OWNED);
	if (psc->wake_refcnt++ == 0 && psc->wake_db != 0)
		lkpi_mhi_ring_doorbell(psc, psc->wake_db, 1);
}

static void
lkpi_mhi_device_zzz(struct lkpi_mhi_softc *psc)
{
	mtx_assert(&psc->lock, MA_OWNED);
	if (psc->wake_refcnt == 0)
		return;
	if (--psc->wake_refcnt == 0 && psc->wake_db != 0 &&
	    lkpi_mhi_wake_db_clear_valid(psc))
		lkpi_mhi_ring_doorbell(psc, psc->wake_db, 0);
}

static int
lkpi_mhi_wake_db_clear_valid(struct lkpi_mhi_softc *psc)
{
	return (psc->mhi_state == MHI_STATE_M0); /* TODO other states? */
}

static void
lkpi_mhi_init_xfer_rings(struct lkpi_mhi_softc *psc)
{
	int i;
	uint32_t chcfg;
	struct lkpi_mhi_xfer_ring *ring;
	struct mhi_chan_ctxt *cbase, *c;

	cbase = (struct mhi_chan_ctxt *)MHI_DMA_KVA(psc->chan_ctxt);
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

	for (i = 0; i < psc->nxfer_rings; i++) {
		ring = &psc->xfer_rings[i];
		KASSERT(ring->mhi_chan_id < psc->max_chan,
		    ("MHI channel %u", ring->mhi_chan_id));
		c = &cbase[ring->mhi_chan_id];
		chcfg = le32toh(c->chcfg);
		chcfg &= ~(MHI_CHAN_CTX_BRSTMODE_MASK |
		    MHI_CHAN_CTX_POLLCFG_MASK);
		chcfg |= (ring->brstmode << MHI_CHAN_CTX_BRSTMODE_SHFT) &
		    MHI_CHAN_CTX_BRSTMODE_MASK;
		chcfg |= (ring->pollcfg << MHI_CHAN_CTX_POLLCFG_SHFT) &
		    MHI_CHAN_CTX_POLLCFG_MASK;
		c->chcfg = htole32(chcfg);
		c->chtype = htole32(ring->mhi_chan_direction);
		c->erindex = htole32(ring->mhi_chan_event_ring_index);
		mtx_lock(&ring->lock);
		ring->chan_ctxt = c;
		ring->mhi_chan_state = MHI_CH_STATE_DISABLED;
		ring->db_pending = false;
		mtx_unlock(&ring->lock);
	}

	wmb();
}

static void
lkpi_mhi_init_event_rings(struct lkpi_mhi_softc *psc)
{
	int i;
	uint32_t intmod;
	uint64_t paddr, len;
	struct lkpi_mhi_event_ring *ring;
	struct mhi_event_ctxt *c;

	c = (struct mhi_event_ctxt *)MHI_DMA_KVA(psc->event_ctxt);
	for (i = 0; i < psc->nevent_rings; i++, c++) {
		ring = &psc->event_rings[i];

		mtx_lock(&ring->lock);
		ring->event_ctxt = c;
		ring->nevents = 0;

		intmod = le32toh(c->intmod);
		intmod &= ~(MHI_EV_CTX_INTMODC_MASK | MHI_EV_CTX_INTMODT_MASK);
		intmod |= (ring->mhi_er_irq_moderation_ms <<
		    MHI_EV_CTX_INTMODT_SHFT) & MHI_EV_CTX_INTMODT_MASK;
		c->intmod = htole32(intmod);

		c->ertype = htole32(MHI_ER_TYPE_VALID);
		c->msivec = htole32(ring->mhi_er_irq);

		paddr = MHI_DMA_DVA(ring->dmamem);
		ring->rp = paddr;
		ring->wp = paddr + ring->size -
		    sizeof(struct mhi_ring_element);
		c->rbase = htole64(paddr);
		c->rp = htole64(ring->rp);
		c->wp = htole64(ring->wp);

		len = sizeof(struct mhi_ring_element) * ring->num_elements;
		c->rlen = htole64(len);
		mtx_unlock(&ring->lock);
	}

	wmb();
}

static void
lkpi_mhi_init_cmd_ring(struct lkpi_mhi_softc *psc)
{
	struct lkpi_mhi_cmd_ring *ring = &psc->cmd_ring;
	struct mhi_cmd_ctxt *c;
	uint64_t paddr, len;

	paddr = MHI_DMA_DVA(ring->dmamem);
	len = ring->size;

	mtx_lock(&psc->lock);
	ring->rp = ring->wp = paddr;
	ring->queued = 0;

	c = (struct mhi_cmd_ctxt *)MHI_DMA_KVA(psc->cmd_ctxt);
	c->rbase = htole64(paddr);
	c->rp = htole64(paddr);
	c->wp = htole64(paddr);
	c->rlen = htole64(len);
	mtx_unlock(&psc->lock);

	wmb();
}

static void
lkpi_mhi_init_dev_ctxt(struct lkpi_mhi_softc *psc)
{
	lkpi_mhi_init_xfer_rings(psc);
	lkpi_mhi_init_event_rings(psc);
	lkpi_mhi_init_cmd_ring(psc);
}

static void *
lkpi_pci_cmd_ring_get_elem(struct lkpi_mhi_cmd_ring *ring, uint64_t ptr)
{
	uint64_t base = MHI_DMA_DVA(ring->dmamem), offset;

	if (ptr < base || ptr >= base + ring->size)
		return NULL;

	offset = ptr - base;
	if (offset >= ring->size)
		return NULL;

	return (uint8_t *)MHI_DMA_KVA(ring->dmamem) + offset;
}

/* psc->lock held. */
static int
lkpi_mhi_cmd_ring_submit(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_cmd_ring *ring)
{
	uint64_t base = MHI_DMA_DVA(ring->dmamem);
	struct mhi_cmd_ctxt *c;

	if (ring->queued >= ring->num_elements - 1)
		return 1;

	if (ring->wp + sizeof(struct mhi_ring_element) >= base + ring->size)
		ring->wp = base;
	else
		ring->wp += sizeof(struct mhi_ring_element);

	wmb();

	c = (struct mhi_cmd_ctxt *)MHI_DMA_KVA(psc->cmd_ctxt);
	c->wp = htole64(ring->wp);

	wmb();

	ring->queued++;
	lkpi_mhi_ring_doorbell(psc, MHI_CRDB_LOWER, ring->wp);
	return 0;
}

/* psc->lock held. */
static int
lkpi_mhi_send_cmd(struct lkpi_mhi_softc *psc, uint32_t cmd, uint32_t chan)
{
	struct lkpi_mhi_cmd_ring *ring = &psc->cmd_ring;
	struct mhi_ring_element *e;

	mtx_assert(&psc->lock, MA_OWNED);
	if (ring->queued >= ring->num_elements - 1) {
		mhi_printf(psc, "command ring overflow\n");
		return 1;
	}

	e = lkpi_pci_cmd_ring_get_elem(ring, ring->wp);
	if (e == NULL)
		return 1;

	e->ptr = 0ULL;
	e->dword[0] = 0;
	e->dword[1] = htole32(
	    ((chan << MHI_TRE_CMD_CHID_SHFT) & MHI_TRE_CMD_CHID_MASK) |
	    ((cmd << MHI_TRE_CMD_CMDID_SHFT) & MHI_TRE_CMD_CMDID_MASK));

	return lkpi_mhi_cmd_ring_submit(psc, ring);
}

static void *
lkpi_pci_xfer_ring_get_elem(struct lkpi_mhi_xfer_ring *ring, uint64_t wp)
{
	uint64_t base = MHI_DMA_DVA(ring->dmamem), offset;
	void *addr = MHI_DMA_KVA(ring->dmamem);

	if (wp < base)
		return NULL;

	offset = wp - base;
	if (offset >= ring->size)
		return NULL;

	return (uint8_t *)addr + offset;
}

static struct lkpi_xfer_data *
lkpi_pci_xfer_ring_get_data(struct lkpi_mhi_xfer_ring *ring, uint64_t wp)
{
	uint64_t base = MHI_DMA_DVA(ring->dmamem), offset;

	if (wp < base)
		return NULL;

	offset = wp - base;
	if (offset >= ring->size)
		return NULL;

	return &ring->data[offset / sizeof(struct mhi_ring_element)];
}

int
linuxkpi_mhi_get_free_desc_count(struct mhi_device *mdev,
    enum dma_data_direction dir)
{
	struct lkpi_mhi_xfer_ring *ring;
	int n;

	ring = dir == DMA_TO_DEVICE ? mdev->lkpi_ul_chan : mdev->lkpi_dl_chan;
	if (ring == NULL)
		return 0;
	mtx_lock(&ring->lock);
	/* One element stays free: a full ring would look empty. */
	n = ring->dmamem != NULL ? ring->num_elements - 1 - ring->queued : 0;
	mtx_unlock(&ring->lock);
	return n;
}

int
linuxkpi_mhi_queue_buf(struct mhi_device *mdev, enum dma_data_direction dir,
    void *buf, size_t len, enum mhi_flags mflags)
{
	struct lkpi_mhi_softc *psc = mdev->mhi_cntrl->lkpi_priv;
	struct lkpi_mhi_xfer_ring *ring;
	struct mhi_ring_element *e;
	struct lkpi_xfer_data *xfer;
	uint64_t paddr, base;
	uint32_t tre1;

	ring = dir == DMA_TO_DEVICE ? mdev->lkpi_ul_chan : mdev->lkpi_dl_chan;
	if (ring == NULL)
		return -EINVAL;

	if (len > MHI_TRE0_DATA_LEN_MASK) {
		/* TODO: chunk xfers */
		mhi_printf(psc, "%s: xfer too large: %zu bytes\n", __func__,
		    len);
		return -EINVAL;
	}

	paddr = dma_map_single(psc->dev, buf, len, dir);
	if (dma_mapping_error(psc->dev, paddr)) {
		mhi_printf(psc, "can't map buffer of %zu bytes\n", len);
		return -ENOMEM;
	}

	mtx_lock(&ring->lock);
	if (ring->mhi_chan_state != MHI_CH_STATE_ENABLED ||
	    READ_ONCE(psc->sys_err)) {
		mtx_unlock(&ring->lock);
		dma_unmap_single(psc->dev, paddr, len, dir);
		return -EIO;
	}
	if (ring->queued >= ring->num_elements - 1) {
		mtx_unlock(&ring->lock);
		dma_unmap_single(psc->dev, paddr, len, dir);
		return -EAGAIN;
	}

	e = lkpi_pci_xfer_ring_get_elem(ring, ring->wp);
	xfer = lkpi_pci_xfer_ring_get_data(ring, ring->wp);
	if (e == NULL || xfer == NULL || xfer->buf != NULL) {
		mtx_unlock(&ring->lock);
		dma_unmap_single(psc->dev, paddr, len, dir);
		return -EIO;
	}

	xfer->buf = buf;
	xfer->dma = paddr;
	xfer->len = len;

	tre1 = MHI_TRE1_DATA_TYPE_TRANSFER << MHI_TRE1_DATA_TYPE_SHIFT;
	if (mflags & MHI_EOT)
		tre1 |= MHI_TRE1_DATA_IEOT;
	if (mflags & MHI_EOB)
		tre1 |= MHI_TRE1_DATA_IEOB;
	if (mflags & MHI_CHAIN)
		tre1 |= MHI_TRE1_DATA_CHAIN;
	e->ptr = htole64(paddr);
	e->dword[0] = htole32((len << MHI_TRE0_DATA_LEN_SHFT) &
	    MHI_TRE0_DATA_LEN_MASK);
	e->dword[1] = htole32(tre1);

	wmb();

	base = MHI_DMA_DVA(ring->dmamem);
	if (ring->wp + sizeof(struct mhi_ring_element) >= base + ring->size)
		ring->wp = base;
	else
		ring->wp += sizeof(struct mhi_ring_element);
	ring->queued++;

	ring->chan_ctxt->wp = htole64(ring->wp);

	wmb();

	/* In M3 the doorbell waits for the resume. */
	if (READ_ONCE(psc->suspended))
		ring->db_pending = true;
	else
		lkpi_mhi_ring_doorbell(psc, ring->db_addr, ring->wp);
	mtx_unlock(&ring->lock);
	return 0;
}

/*
 * Hand back the buffers still queued on a channel that no longer runs:
 * to the driver's callback with -ENOTCONN if notify, otherwise only
 * unmapped.
 */
static void
lkpi_mhi_xfer_ring_flush(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_xfer_ring *ring, bool notify)
{
	struct lkpi_mhi_event_ring *er;
	struct lkpi_xfer_data *xfer;
	struct mhi_result res;
	struct mhi_device *mdev;
	void (*cb)(struct mhi_device *, struct mhi_result *);
	uint64_t base;

	/* Wait for a completion handler still running a callback. */
	er = &psc->event_rings[ring->mhi_chan_event_ring_index];
	mtx_lock(&er->lock);
	mtx_unlock(&er->lock);

	mtx_lock(&ring->lock);
	if (ring->dmamem == NULL) {
		mtx_unlock(&ring->lock);
		return;
	}
	base = MHI_DMA_DVA(ring->dmamem);
	while (ring->queued > 0) {
		xfer = lkpi_pci_xfer_ring_get_data(ring, ring->rp);
		if (ring->rp + sizeof(struct mhi_ring_element) >=
		    base + ring->size)
			ring->rp = base;
		else
			ring->rp += sizeof(struct mhi_ring_element);
		ring->queued--;
		if (xfer == NULL || xfer->buf == NULL)
			continue;
		dma_unmap_single(psc->dev, xfer->dma, xfer->len, ring->dir);
		res.buf_addr = xfer->buf;
		res.bytes_xferd = 0;
		res.dir = ring->dir;
		res.transaction_status = -ENOTCONN;
		xfer->buf = NULL;
		mdev = ring->mhi_dev;
		cb = NULL;
		if (notify && mdev != NULL && mdev->lkpi_drv != NULL)
			cb = ring->dir == DMA_TO_DEVICE ?
			    mdev->lkpi_drv->ul_xfer_cb :
			    mdev->lkpi_drv->dl_xfer_cb;
		if (cb == NULL)
			continue;
		mtx_unlock(&ring->lock);
		cb(mdev, &res);
		mtx_lock(&ring->lock);
	}
	ring->wp = ring->rp;
	ring->db_pending = false;
	mtx_unlock(&ring->lock);
}

static int
lkpi_mhi_start_channel(struct lkpi_mhi_softc *psc,
	struct lkpi_mhi_xfer_ring *ring)
{
	struct mhi_chan_ctxt *c;
	int ret = 0;
	uint32_t chcfg;
	uint64_t paddr, len;

	DNPRINTF(psc, "%s: start MHI channel %d in state %d\n", __func__,
	    ring->mhi_chan_id, ring->mhi_chan_state);

	mtx_lock(&ring->lock);
	if (ring->dmamem == NULL || ring->chan_ctxt == NULL ||
	    ring->mhi_chan_state == MHI_CH_STATE_ENABLED) {
		mtx_unlock(&ring->lock);
		return -EINVAL;
	}
	c = ring->chan_ctxt;

	chcfg = le32toh(c->chcfg);
	chcfg &= ~MHI_CHAN_CTX_CHSTATE_MASK;
	chcfg |= MHI_CHAN_CTX_CHSTATE_ENABLED;
	c->chcfg = htole32(chcfg);

	paddr = MHI_DMA_DVA(ring->dmamem);
	ring->rp = ring->wp = paddr;
	ring->queued = 0;
	ring->db_pending = false;
	c->rbase = htole64(paddr);
	c->rp = htole64(ring->rp);
	c->wp = htole64(ring->wp);
	len = sizeof(struct mhi_ring_element) * ring->num_elements;
	c->rlen = htole64(len);
	mtx_unlock(&ring->lock);

	wmb();

	ret = lkpi_mhi_chan_cmd(psc, ring, MHI_CMD_START_CHAN);
	if (ret) {
		mhi_printf(psc, "could not start MHI channel %d in state "
		    "%d: status 0x%x\n", ring->mhi_chan_id,
		    ring->mhi_chan_state, ring->cmd_status);
		if (ret == -ETIMEDOUT)
			lkpi_mhi_dump_state(psc, "channel start timeout");
		return ret;
	}

	/*
	 * qwx filled an inbound ring with its own buffers here; the client
	 * driver queues its receive buffers with mhi_queue_buf().
	 */
	mtx_lock(&ring->lock);
	ring->mhi_chan_state = MHI_CH_STATE_ENABLED;
	mtx_unlock(&ring->lock);
	return 0;
}

/*
 * M0 reported by a state change event or, if arg is not NULL, by the
 * status register.
 */
static int
lkpi_mhi_check_m0(struct lkpi_mhi_softc *psc, void *arg)
{
	uint32_t state;

	if (psc->sys_err)
		return -EIO;
	if (psc->mhi_state == MHI_STATE_M0)
		return 1;
	state = (lkpi_pci_read(psc, MHI_STATUS) & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;
	if (state == MHI_STATE_SYS_ERR)
		return -EIO;
	return (state == MHI_STATE_M0 && arg != NULL);
}

static int
lkpi_mhi_start(struct lkpi_mhi_softc *psc)
{
	uint32_t off;
	uint32_t ee, state;
	int ret;

	lkpi_mhi_init_dev_ctxt(psc);

	psc->bhi_off = lkpi_pci_read(psc, MHI_BHI_OFFSET);
	DNPRINTF(psc, "%s: BHI offset 0x%x\n", __func__, psc->bhi_off);

	psc->bhie_off = lkpi_pci_read(psc, MHI_BHIE_OFFSET);
	DNPRINTF(psc, "%s: BHIE offset 0x%x\n", __func__, psc->bhie_off);

	if (psc->bhi_off == 0 || psc->bhi_off >= psc->mhi->reg_len ||
	    psc->bhie_off == 0 || psc->bhie_off >= psc->mhi->reg_len) {
		mhi_printf(psc, "invalid BHI/BHIE offsets 0x%x/0x%x\n",
		    psc->bhi_off, psc->bhie_off);
		return -EIO;
	}

	/* Clean BHIE RX registers */
	for (off = MHI_BHIE_RXVECADDR_LOW_OFFS;
	     off < (MHI_BHIE_RXVECSTATUS_OFFS - 4);
	     off += 4)
		lkpi_pci_write(psc, psc->bhie_off + off, 0x0);

	lkpi_rddm_prepare(psc);

	/* Program BHI INTVEC */
	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_INTVEC, 0x00);

	/*
	 * Get BHI execution environment and confirm that it is valid
	 * for power on.
	 */
	ee = lkpi_pci_read(psc, psc->bhi_off + MHI_BHI_EXECENV);
	if (!MHI_POWER_UP_CAPABLE(ee)) {
		mhi_printf(psc, "invalid EE for power on: 0x%x\n", ee);
		return -EIO;
	}

	/*
	 * Get MHI state of the device and reset it if it is in system
	 * error.
	 */
	state = lkpi_pci_read(psc, MHI_STATUS);
	DNPRINTF(psc, "%s: MHI power on with EE: 0x%x, status: 0x%x\n",
	     __func__, ee, state);
	state = (state & MHI_STATUS_MHISTATE_MASK) >> MHI_STATUS_MHISTATE_SHFT;
	if (state == MHI_STATE_SYS_ERR) {
		if (lkpi_mhi_reset_device(psc, 0))
			return -EIO;
		state = lkpi_pci_read(psc, MHI_STATUS);
		DNPRINTF(psc, "%s: MHI state after reset: 0x%x\n",
		    __func__, state);
		state = (state & MHI_STATUS_MHISTATE_MASK) >>
		    MHI_STATUS_MHISTATE_SHFT;
		if (state == MHI_STATE_SYS_ERR) {
			mhi_printf(psc, "MHI stuck in system error state\n");
			return -EIO;
		}
	}

	mtx_lock(&psc->lock);
	psc->bhi_ee = ee;
	psc->mhi_state = state;
	mtx_unlock(&psc->lock);

	/* Enable IRQs */
	ret = lkpi_mhi_request_irqs(psc);
	if (ret)
		return ret;

	/* Transition to primary runtime. */
	if (MHI_IN_PBL(ee)) {
		ret = lkpi_mhi_fw_load_handler(psc);
		if (ret)
			return ret;
	} else {
		/*
		 * The firmware runs already (the device kept it across a
		 * warm reboot): reset MHI and redo the READY/M0 handshake.
		 */
		ret = lkpi_mhi_reset_device(psc, 1);
		if (ret == 0)
			ret = lkpi_mhi_ready_state_transition(psc);
		if (ret == 0)
			ret = lkpi_mhi_wait(psc, &psc->mhi_state, "mhim0",
			    lkpi_mhi_check_m0, NULL, NULL);
		if (ret)
			return ret;
		ee = lkpi_pci_read(psc, psc->bhi_off + MHI_BHI_EXECENV);
		if (!MHI_IN_MISSION_MODE(ee)) {
			mhi_printf(psc, "failed to power up MHI, ee=0x%x\n",
			    ee);
			return -EIO;
		}
		mtx_lock(&psc->lock);
		psc->bhi_ee = ee;
		mtx_unlock(&psc->lock);
	}

	return 0;
}

static void
lkpi_mhi_stop(struct lkpi_mhi_softc *psc)
{
	lkpi_mhi_reset_device(psc, 1);
}

static int
lkpi_mhi_reset_device(struct lkpi_mhi_softc *psc, int force)
{
	uint32_t reg;
	int ret = 0;

	reg = lkpi_pci_read(psc, MHI_STATUS);

	DNPRINTF(psc, "%s: MHISTATUS 0x%x\n", __func__, reg);
	/*
	 * Observed on QCA6390 that after SOC_GLOBAL_RESET, MHISTATUS
	 * has SYSERR bit set and thus need to set MHICTRL_RESET
	 * to clear SYSERR.
	 */
	if (force || (reg & MHI_STATUS_SYSERR_MASK)) {
		/* Trigger MHI Reset in device. */
		lkpi_pci_write(psc, MHI_CTRL, MHI_CTRL_RESET_MASK);

		/* Wait for the reset bit to be cleared by the device. */
		ret = lkpi_mhi_await_device_reset(psc);
		if (ret)
			return ret;

		if (psc->bhi_off == 0)
			psc->bhi_off = lkpi_pci_read(psc, MHI_BHI_OFFSET);

		/* Device clear BHI INTVEC so re-program it. */
		lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_INTVEC, 0x00);

		mtx_lock(&psc->lock);
		psc->mhi_state = MHI_STATE_RESET;
		mtx_unlock(&psc->lock);
	}

	return 0;
}

static int
lkpi_mhi_fw_load_handler(struct lkpi_mhi_softc *psc)
{
	struct mhi_controller *mhi = psc->mhi;
	const char *name;
	int ret;

	name = mhi->fw_image != NULL ? mhi->fw_image : "firmware";

	/* Second-stage boot loader sits in the first sbl_size bytes. */
	ret = lkpi_mhi_fw_load_bhi(psc);
	if (ret != 0) {
		mhi_printf(psc, "could not load firmware %s\n", name);
		return ret;
	}

	/*
	 * The SBL reports READY: give it the contexts and move it to M0.
	 * (qwx did this from its interrupt handler.)
	 */
	ret = lkpi_mhi_ready_state_transition(psc);
	if (ret != 0)
		return ret;
	ret = lkpi_mhi_wait(psc, &psc->mhi_state, "mhim0",
	    lkpi_mhi_check_m0, NULL, NULL);
	if (ret != 0) {
		mhi_printf(psc, "device failed to enter M0\n");
		return ret;
	}

	if (mhi->fbc_download) {
		/* Now load the full image. */
		ret = lkpi_mhi_fw_load_bhie(psc, mhi->fbc_image);
		if (ret != 0) {
			mhi_printf(psc, "could not load firmware %s\n",
			    name);
			return ret;
		}
	}

	ret = lkpi_mhi_wait(psc, &psc->bhi_ee, "mhiamss",
	    lkpi_mhi_check_mission, NULL, NULL);
	if (ret != 0)
		mhi_printf(psc, "device failed to enter AMSS EE\n");

	return ret;
}

static int
lkpi_mhi_await_device_reset(struct lkpi_mhi_softc *psc)
{
	uint32_t reg;
	int end;

	/* Poll for CTRL RESET to clear. */
	end = ticks + lkpi_mhi_timeout_ticks(psc);
	for (;;) {
		reg = lkpi_pci_read(psc, MHI_CTRL);
		DNPRINTF(psc, "%s: MHI_CTRL is 0x%x\n", __func__, reg);
		if ((reg & MHI_CTRL_RESET_MASK) == 0)
			break;
		if (end - ticks <= 0) {
			mhi_printf(psc, "MHI reset failed\n");
			return -ETIMEDOUT;
		}
		pause("mhirst", MHI_POLL_TICKS);
	}

	return 0;
}

static int
lkpi_mhi_await_device_ready(struct lkpi_mhi_softc *psc)
{
	uint32_t reg;
	int end;

	/* Poll for READY to be set. */
	end = ticks + lkpi_mhi_timeout_ticks(psc);
	for (;;) {
		reg = lkpi_pci_read(psc, MHI_STATUS);
		DNPRINTF(psc, "%s: MHI_STATUS is 0x%x\n", __func__, reg);
		/* MHI_STATUS is read-only: qwx's write to clear READY went. */
		if (reg & MHI_STATUS_READY_MASK)
			break;
		if ((reg & MHI_STATUS_MHISTATE_MASK) >>
		    MHI_STATUS_MHISTATE_SHFT == MHI_STATE_SYS_ERR) {
			mhi_printf(psc, "MHI in system error, not ready\n");
			return -EIO;
		}
		if (end - ticks <= 0) {
			mhi_printf(psc, "MHI not ready\n");
			return -ETIMEDOUT;
		}
		pause("mhirdy", MHI_POLL_TICKS);
	}

	return 0;
}

static int
lkpi_mhi_ready_state_transition(struct lkpi_mhi_softc *psc)
{
	int ret, i;

	ret = lkpi_mhi_await_device_reset(psc);
	if (ret)
		return ret;

	ret = lkpi_mhi_await_device_ready(psc);
	if (ret)
		return ret;

	mtx_lock(&psc->lock);
	if (psc->mhi_state == MHI_STATE_RESET)
		psc->mhi_state = MHI_STATE_READY;
	mtx_unlock(&psc->lock);

	/* Set up memory-mapped IO for channels, events, etc. */
	lkpi_mhi_init_mmio(psc);

	/* Notify event rings. */
	for (i = 0; i < psc->nevent_rings; i++) {
		struct lkpi_mhi_event_ring *ring = &psc->event_rings[i];
		mtx_lock(&ring->lock);
		lkpi_mhi_ring_doorbell(psc, ring->db_addr, ring->wp);
		mtx_unlock(&ring->lock);
	}

	/*
	 * Set the device into M0 state. The device will transition
	 * into M0 and the execution environment will switch to SBL.
	 */
	lkpi_mhi_set_state(psc, MHI_STATE_M0);
	return 0;
}

/* psc->lock held. */
static void
lkpi_mhi_mission_mode_state_transition(struct lkpi_mhi_softc *psc)
{
	int i;

	lkpi_mhi_device_wake(psc);

	/* Notify event rings. */
	for (i = 0; i < psc->nevent_rings; i++) {
		struct lkpi_mhi_event_ring *ring = &psc->event_rings[i];
		lkpi_mhi_ring_doorbell(psc, ring->db_addr,
		    READ_ONCE(ring->wp));
	}

	/* TODO: Notify transfer/command rings? */

	lkpi_mhi_device_zzz(psc);
}

/* psc->lock held. */
static void
lkpi_mhi_low_power_mode_state_transition(struct lkpi_mhi_softc *psc)
{
	/* Not while someone keeps the device awake. */
	if (psc->wake_refcnt == 0)
		lkpi_mhi_set_state(psc, MHI_STATE_M2);
}

static void
lkpi_mhi_set_state(struct lkpi_mhi_softc *psc, uint32_t state)
{
	uint32_t reg;

	reg = lkpi_pci_read(psc, MHI_CTRL);

	if (state != MHI_STATE_RESET) {
		reg &= ~MHI_CTRL_MHISTATE_MASK;
		reg |= (state << MHI_CTRL_MHISTATE_SHFT) & MHI_CTRL_MHISTATE_MASK;
	} else
		reg |= MHI_CTRL_RESET_MASK;

	lkpi_pci_write(psc, MHI_CTRL, reg);
}

static void
lkpi_mhi_init_mmio(struct lkpi_mhi_softc *psc)
{
	struct mhi_controller *mhi = psc->mhi;
	uint64_t paddr;
	uint32_t reg;
	int i, nhwer;

	reg = lkpi_pci_read(psc, MHI_CHDBOFF);

	/* Set device wake doorbell address. */
	mtx_lock(&psc->lock);
	psc->wake_db = reg + 8 * MHI_DEV_WAKE_DB;
	mtx_unlock(&psc->lock);

	/* Set doorbell address for each transfer ring. */
	for (i = 0; i < psc->nxfer_rings; i++) {
		struct lkpi_mhi_xfer_ring *ring = &psc->xfer_rings[i];
		ring->db_addr = reg + (8 * ring->mhi_chan_id);
	}

	reg = lkpi_pci_read(psc, MHI_ERDBOFF);
	/* Set doorbell address for each event ring. */
	for (i = 0; i < psc->nevent_rings; i++) {
		struct lkpi_mhi_event_ring *ring = &psc->event_rings[i];
		ring->db_addr = reg + (8 * i);
	}

	paddr = MHI_DMA_DVA(psc->chan_ctxt);
	lkpi_pci_write(psc, MHI_CCABAP_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_CCABAP_LOWER, paddr & 0xffffffff);

	paddr = MHI_DMA_DVA(psc->event_ctxt);
	lkpi_pci_write(psc, MHI_ECABAP_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_ECABAP_LOWER, paddr & 0xffffffff);

	paddr = MHI_DMA_DVA(psc->cmd_ctxt);
	lkpi_pci_write(psc, MHI_CRCBAP_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_CRCBAP_LOWER, paddr & 0xffffffff);

	/* The device's DMA window. */
	paddr = mhi->iova_start;
	lkpi_pci_write(psc, MHI_CTRLBASE_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_CTRLBASE_LOWER, paddr & 0xffffffff);
	lkpi_pci_write(psc, MHI_DATABASE_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_DATABASE_LOWER, paddr & 0xffffffff);
	paddr = mhi->iova_stop;
	lkpi_pci_write(psc, MHI_CTRLLIMIT_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_CTRLLIMIT_LOWER, paddr & 0xffffffff);
	lkpi_pci_write(psc, MHI_DATALIMIT_HIGHER, paddr >> 32);
	lkpi_pci_write(psc, MHI_DATALIMIT_LOWER, paddr & 0xffffffff);

	nhwer = 0;
	for (i = 0; i < psc->nevent_rings; i++) {
		if (psc->event_rings[i].hw_event)
			nhwer++;
	}
	reg = lkpi_pci_read(psc, MHI_CFG);
	reg &= ~(MHI_CFG_NER_MASK | MHI_CFG_NHWER_MASK);
	reg |= (psc->nevent_rings << MHI_CFG_NER_SHFT) & MHI_CFG_NER_MASK;
	reg |= (nhwer << MHI_CFG_NHWER_SHFT) & MHI_CFG_NHWER_MASK;
	lkpi_pci_write(psc, MHI_CFG, reg);
}

struct lkpi_mhi_bhi_wait {
	uint32_t	reg;
	uint32_t	mask;
	uint32_t	shift;
	uint32_t	done;
	uint32_t	error;
	uint32_t	status;
};

static int
lkpi_mhi_check_bhi(struct lkpi_mhi_softc *psc, void *arg)
{
	struct lkpi_mhi_bhi_wait *w = arg;

	w->status = (lkpi_pci_read(psc, w->reg) & w->mask) >> w->shift;
	if (w->status == w->done)
		return 1;
	if (w->status == w->error || psc->sys_err)
		return -EIO;
	return 0;
}

static int
lkpi_mhi_fw_load_bhi(struct lkpi_mhi_softc *psc)
{
	struct mhi_controller *mhi = psc->mhi;
	struct lkpi_dmamem *data_adm;
	struct lkpi_mhi_bhi_wait w;
	uint32_t seq;
	uint64_t paddr;
	size_t len;
	int ret;

	len = mhi->fbc_download ? mhi->sbl_size : psc->fw_size;
	data_adm = lkpi_dmamem_alloc(psc, len, 0);
	if (data_adm == NULL) {
		mhi_printf(psc, "could not allocate BHI DMA data buffer\n");
		return -ENOMEM;
	}

	/* Copy firmware image to DMA memory. */
	if (mhi->fbc_download)
		lkpi_mhi_image_copy(mhi->fbc_image, MHI_DMA_KVA(data_adm), len,
		    false);
	else
		memcpy(MHI_DMA_KVA(data_adm), MHI_DMA_KVA(psc->amss_data), len);

	wmb();

	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_STATUS, 0);

	/* Set data physical address and length. */
	paddr = MHI_DMA_DVA(data_adm);
	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_IMGADDR_HIGH, paddr >> 32);
	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_IMGADDR_LOW,
	    paddr & 0xffffffff);
	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_IMGSIZE, len);

	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHI_TXDB_SEQNUM_BMSK);
	} while (seq == 0);
	lkpi_pci_write(psc, psc->bhi_off + MHI_BHI_IMGTXDB, seq);

	/* Wait for completion. */
	w.reg = psc->bhi_off + MHI_BHI_STATUS;
	w.mask = MHI_BHI_STATUS_MASK;
	w.shift = MHI_BHI_STATUS_SHFT;
	w.done = MHI_BHI_STATUS_SUCCESS;
	w.error = MHI_BHI_STATUS_ERROR;
	w.status = MHI_BHI_STATUS_RESET;
	ret = lkpi_mhi_wait(psc, &psc->bhi_ee, "mhibhi", lkpi_mhi_check_bhi,
	    &w, NULL);

	if (ret == -ETIMEDOUT)
		mhi_printf(psc, "BHI load timeout\n");
	else if (ret)
		mhi_printf(psc, "BHI load failed: status 0x%x\n", w.status);
	DNPRINTF(psc, "%s: BHI status is 0x%x EE is 0x%x\n",
	    __func__, w.status, psc->bhi_ee);

	lkpi_dmamem_free(psc, data_adm);
	return ret;
}

static int
lkpi_mhi_fw_load_bhie(struct lkpi_mhi_softc *psc, struct image_info *img)
{
	struct lkpi_mhi_bhi_wait w;
	struct mhi_buf *vec;
	uint32_t seq, reg;
	uint64_t paddr;
	int ret;

	if (img == NULL || img->entries < 2) {
		mhi_printf(psc, "no BHIE image\n");
		return -EINVAL;
	}
	vec = &img->mhi_buf[img->entries - 1];

	/* Set vector physical address and length. */
	paddr = vec->dma_addr;
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_TXVECADDR_HIGH_OFFS,
	    paddr >> 32);
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_TXVECADDR_LOW_OFFS,
	    paddr & 0xffffffff);
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_TXVECSIZE_OFFS, vec->len);

	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHIE_TXVECSTATUS_SEQNUM_BMSK);
	} while (seq == 0);
	reg = lkpi_pci_read(psc, psc->bhie_off + MHI_BHIE_TXVECDB_OFFS);
	reg &= ~MHI_BHIE_TXVECDB_SEQNUM_BMSK;
	reg |= seq << MHI_BHIE_TXVECDB_SEQNUM_SHFT;
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_TXVECDB_OFFS, reg);

	/* Wait for completion. */
	w.reg = psc->bhie_off + MHI_BHIE_TXVECSTATUS_OFFS;
	w.mask = MHI_BHIE_TXVECSTATUS_STATUS_BMSK;
	w.shift = MHI_BHIE_TXVECSTATUS_STATUS_SHFT;
	w.done = MHI_BHIE_TXVECSTATUS_STATUS_XFER_COMPL;
	w.error = MHI_BHIE_TXVECSTATUS_STATUS_ERROR;
	w.status = MHI_BHIE_TXVECSTATUS_STATUS_RESET;
	ret = lkpi_mhi_wait(psc, &psc->bhie_off, "mhibhie",
	    lkpi_mhi_check_bhi, &w, NULL);
	DNPRINTF(psc, "%s: txvec state is 0x%x\n", __func__, w.status);

	if (ret == -ETIMEDOUT)
		mhi_printf(psc, "BHIE load timeout\n");
	else if (ret)
		mhi_printf(psc, "BHIE load failed: status 0x%x\n", w.status);
	return ret;
}

/*
 * Give the device the BHIe receive vector of the crash dump image, for
 * the RDDM execution environment.  The image is allocated by
 * mhi_prepare_for_power_up().
 */
static void
lkpi_rddm_prepare(struct lkpi_mhi_softc *psc)
{
	struct image_info *img = psc->mhi->rddm_image;
	struct mhi_buf *vec;
	uint32_t seq, reg;
	uint64_t paddr;

	if (img == NULL || img->entries < 2)
		return;
	vec = &img->mhi_buf[img->entries - 1];

	/* Set vector physical address and length. */
	paddr = vec->dma_addr;
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_RXVECADDR_HIGH_OFFS,
	    paddr >> 32);
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_RXVECADDR_LOW_OFFS,
	    paddr & 0xffffffff);
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_RXVECSIZE_OFFS, vec->len);

	/* Set a random transaction sequence number. */
	do {
		seq = arc4random_uniform(MHI_BHIE_RXVECSTATUS_SEQNUM_BMSK);
	} while (seq == 0);

	reg = lkpi_pci_read(psc, psc->bhie_off + MHI_BHIE_RXVECDB_OFFS);
	reg &= ~MHI_BHIE_RXVECDB_SEQNUM_BMSK;
	reg |= seq << MHI_BHIE_RXVECDB_SEQNUM_SHFT;
	lkpi_pci_write(psc, psc->bhie_off + MHI_BHIE_RXVECDB_OFFS, reg);

	psc->rddm_armed = true;
}

int
linuxkpi_mhi_download_rddm_image(struct mhi_controller *mhi, bool in_panic)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	struct image_info *img;
	uint32_t ee, reg, state = MHI_BHIE_RXVECSTATUS_STATUS_RESET;
	uint8_t *rddm;
	int end;

	if (psc == NULL || (img = mhi->rddm_image) == NULL) {
		DNPRINTF(psc, "%s: RDDM not prepared\n", __func__);
		return -EINVAL;
	}
	if (psc->bhi_off == 0 || psc->bhie_off == 0)
		return -EIO;
	ee = lkpi_pci_read(psc, psc->bhi_off + MHI_BHI_EXECENV);
	if (ee != MHI_EE_RDDM) {
		mhi_printf(psc, "no RDDM download: device EE 0x%x\n", ee);
		return -EIO;
	}
	if (!psc->rddm_armed)
		lkpi_rddm_prepare(psc);

	/* Poll for completion */
	end = ticks + lkpi_mhi_timeout_ticks(psc);
	for (;;) {
		reg = lkpi_pci_read(psc,
		    psc->bhie_off + MHI_BHIE_RXVECSTATUS_OFFS);
		state = (reg & MHI_BHIE_RXVECSTATUS_STATUS_BMSK) >>
		    MHI_BHIE_RXVECSTATUS_STATUS_SHFT;
		DNPRINTF(psc, "%s: rxvec state is 0x%x\n", __func__, state);
		if (state == MHI_BHIE_RXVECSTATUS_STATUS_XFER_COMPL)
			break;
		if (state == MHI_BHIE_RXVECSTATUS_STATUS_ERROR ||
		    end - ticks <= 0) {
			mhi_printf(psc, "RDDM dump failed: rxvec status "
			    "0x%x\n", reg);
			return (state == MHI_BHIE_RXVECSTATUS_STATUS_ERROR ?
			    -EIO : -ETIMEDOUT);
		}
		if (in_panic)
			DELAY(10 * 1000);
		else
			pause("mhirddm", MHI_POLL_TICKS);
	}

	rmb();
	rddm = img->mhi_buf[0].buf;
	DNPRINTF(psc, "%s: RDDM done, %u segments, first bytes "
	    "%02x %02x %02x %02x %02x %02x %02x %02x\n", __func__,
	    img->entries - 1, rddm[0], rddm[1], rddm[2], rddm[3], rddm[4],
	    rddm[5], rddm[6], rddm[7]);
	return 0;
}

static void *
lkpi_pci_event_ring_get_elem(struct lkpi_mhi_event_ring *ring, uint64_t rp)
{
	uint64_t base = MHI_DMA_DVA(ring->dmamem), offset;
	void *addr = MHI_DMA_KVA(ring->dmamem);

	if (rp < base)
		return NULL;

	offset = rp - base;
	if (offset >= ring->size)
		return NULL;

	return (uint8_t *)addr + offset;
}

static int
lkpi_mhi_check_mission(struct lkpi_mhi_softc *psc, void *arg __unused)
{
	if (psc->sys_err || psc->bhi_ee == MHI_EE_RDDM)
		return -EIO;
	return (MHI_IN_MISSION_MODE(psc->bhi_ee));
}

/* psc->lock held. */
static void
lkpi_mhi_state_change(struct lkpi_mhi_softc *psc, int ee, int mhi_state)
{
	uint32_t old_ee = psc->bhi_ee;
	uint32_t old_mhi_state = psc->mhi_state;

	mtx_assert(&psc->lock, MA_OWNED);
	if (ee != -1 && psc->bhi_ee != ee) {
		switch (ee) {
		case MHI_EE_PBL:
			DNPRINTF(psc, "new EE PBL\n");
			psc->bhi_ee = ee;
			break;
		case MHI_EE_SBL:
			psc->bhi_ee = ee;
			DNPRINTF(psc, "new EE SBL\n");
			break;
		case MHI_EE_AMSS:
			DNPRINTF(psc, "new EE AMSS\n");
			psc->bhi_ee = ee;
			/* Wake thread loading the full AMSS image. */
			wakeup(&psc->bhie_off);
			break;
		case MHI_EE_WFW:
			DNPRINTF(psc, "new EE WFW\n");
			psc->bhi_ee = ee;
			break;
		case MHI_EE_RDDM:
			/* Firmware crash: the rings are of no use now. */
			mhi_printf(psc, "fatal firmware error: RDDM\n");
			psc->bhi_ee = ee;
			psc->sys_err = true;
			lkpi_mhi_queue_status(psc, MHI_CB_EE_RDDM);
			break;
		default:
			mhi_printf(psc, "unhandled EE change to %x\n", ee);
			break;
		}
	}

	if (mhi_state != -1 && psc->mhi_state != mhi_state) {
		switch (mhi_state) {
		case -1:
			break;
		case MHI_STATE_RESET:
			DNPRINTF(psc, "new MHI state RESET\n");
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_READY:
			/*
			 * The power up thread does the READY transition;
			 * the device may report READY without an interrupt.
			 */
			DNPRINTF(psc, "new MHI state READY\n");
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_M0:
			DNPRINTF(psc, "new MHI state M0\n");
			psc->mhi_state = mhi_state;
			lkpi_mhi_mission_mode_state_transition(psc);
			break;
		case MHI_STATE_M1:
			DNPRINTF(psc, "new MHI state M1\n");
			psc->mhi_state = mhi_state;
			lkpi_mhi_low_power_mode_state_transition(psc);
			break;
		case MHI_STATE_M2:
			DNPRINTF(psc, "new MHI state M2\n");
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_M3:
			DNPRINTF(psc, "new MHI state M3\n");
			psc->mhi_state = mhi_state;
			break;
		case MHI_STATE_SYS_ERR:
			DNPRINTF(psc, "new MHI state SYS ERR\n");
			psc->mhi_state = mhi_state;
			if (psc->powered && !psc->sys_err) {
				mhi_printf(psc, "device in system error\n");
				psc->sys_err = true;
				lkpi_mhi_queue_status(psc, MHI_CB_SYS_ERROR);
			}
			break;
		default:
			mhi_printf(psc, "unhandled MHI state change to %x\n",
			    mhi_state);
			break;
		}
	}

	if (old_ee != psc->bhi_ee)
		wakeup(&psc->bhi_ee);
	if (old_mhi_state != psc->mhi_state)
		wakeup(&psc->mhi_state);
}

static void
lkpi_pci_intr_ctrl_event_mhi(struct lkpi_mhi_softc *psc, uint32_t mhi_state)
{
	mtx_lock(&psc->lock);
	DNPRINTF(psc, "%s: MHI state change 0x%x -> 0x%x\n", __func__,
	    psc->mhi_state, mhi_state);

	if (psc->mhi_state != mhi_state)
		lkpi_mhi_state_change(psc, -1, mhi_state);
	mtx_unlock(&psc->lock);
}

static void
lkpi_pci_intr_ctrl_event_ee(struct lkpi_mhi_softc *psc, uint32_t ee)
{
	mtx_lock(&psc->lock);
	DNPRINTF(psc, "%s: EE change 0x%x to 0x%x\n", __func__,
	    psc->bhi_ee, ee);

	if (psc->bhi_ee != ee)
		lkpi_mhi_state_change(psc, ee, -1);
	mtx_unlock(&psc->lock);
}

static void
lkpi_pci_intr_ctrl_event_cmd_complete(struct lkpi_mhi_softc *psc,
    uint64_t ptr, uint32_t cmd_status)
{
	struct lkpi_mhi_cmd_ring *cmd_ring = &psc->cmd_ring;
	uint64_t base;
	struct lkpi_mhi_xfer_ring *xfer_ring = NULL;
	struct mhi_ring_element *e;
	uint32_t tre1, chid;

	mtx_lock(&psc->lock);
	if (cmd_ring->dmamem == NULL) {
		mtx_unlock(&psc->lock);
		return;
	}
	base = MHI_DMA_DVA(cmd_ring->dmamem);
	e = lkpi_pci_cmd_ring_get_elem(cmd_ring, ptr);
	if (e == NULL) {
		mtx_unlock(&psc->lock);
		return;
	}

	tre1 = le32toh(e->dword[1]);
	chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;

	if (chid < psc->max_chan)
		xfer_ring = psc->chan_map[chid];
	if (xfer_ring == NULL) {
		mhi_printf(psc, "%s: no transfer ring found for command "
		    "completion on channel %u\n", __func__, chid);
	} else {
		xfer_ring->cmd_status = cmd_status;
		wakeup(&xfer_ring->cmd_status);
	}

	/* One command is done: its ring element is free again. */
	if (cmd_ring->rp + sizeof(*e) >= base + cmd_ring->size)
		cmd_ring->rp = base;
	else
		cmd_ring->rp += sizeof(*e);
	if (cmd_ring->queued > 0)
		cmd_ring->queued--;
	mtx_unlock(&psc->lock);
}

static int
lkpi_pci_intr_ctrl_event(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_event_ring *ring, int flush)
{
	struct mhi_event_ctxt *c;
	uint64_t rp, wp, base;
	struct mhi_ring_element *e;
	uint32_t tre0, tre1, type, code, chid, len;

	mtx_lock(&ring->lock);
	c = ring->event_ctxt;
	if (c == NULL || ring->dmamem == NULL) {
		/*
		 * Interrupts can trigger before mhi_init_event_rings()
		 * if the device is still active after a warm reboot.
		 */
		mtx_unlock(&ring->lock);
		return 0;
	}

	rmb();

	rp = le64toh(c->rp);
	wp = le64toh(c->wp);

	DNPRINTF(psc, "%s: kernel rp=0x%jx device rp=0x%jx kernel wp=0x%jx "
	    "device wp=0x%jx\n", __func__, (uintmax_t)ring->rp, (uintmax_t)rp,
	    (uintmax_t)ring->wp, (uintmax_t)wp);

	base = MHI_DMA_DVA(ring->dmamem);
	if (ring->rp == rp || rp < base || rp >= base + ring->size) {
		mtx_unlock(&ring->lock);
		return 0;
	}
	if (wp < base || wp >= base + ring->size) {
		mtx_unlock(&ring->lock);
		return 0;
	}

	rmb();

	while (ring->rp != rp) {
		e = lkpi_pci_event_ring_get_elem(ring, ring->rp);
		if (e == NULL) {
			mtx_unlock(&ring->lock);
			return 0;
		}

		tre0 = le32toh(e->dword[0]);
		tre1 = le32toh(e->dword[1]);

		len = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
		code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
		type = (tre1 & MHI_TRE1_EV_TYPE_MASK) >> MHI_TRE1_EV_TYPE_SHFT;
		chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;
		DNPRINTF(psc, "%s: len=%u code=0x%x type=0x%x chid=%d\n",
		    __func__, len, code, type, chid);

		if (!flush) {
			switch (type) {
			case MHI_PKT_TYPE_STATE_CHANGE_EVENT:
				lkpi_pci_intr_ctrl_event_mhi(psc, code);
				break;
			case MHI_PKT_TYPE_EE_EVENT:
				lkpi_pci_intr_ctrl_event_ee(psc, code);
				break;
			case MHI_PKT_TYPE_CMD_COMPLETION_EVENT:
				lkpi_pci_intr_ctrl_event_cmd_complete(psc,
				    le64toh(e->ptr), code);
				break;
			case MHI_PKT_TYPE_TX_EVENT:
				lkpi_pci_intr_data_event_tx(psc, e);
				break;
			default:
				mhi_printf(psc, "%s: unhandled event type "
				    "0x%x\n", __func__, type);
				break;
			}
		}
		ring->nevents++;

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

	wmb();

	lkpi_mhi_ring_doorbell(psc, ring->db_addr, ring->wp);
	mtx_unlock(&ring->lock);
	return 1;
}

/*
 * A transfer completion event: hand the buffers up to the one it names
 * back to the client driver.  The event ring's lock is held.
 */
static void
lkpi_pci_intr_data_event_tx(struct lkpi_mhi_softc *psc,
    struct mhi_ring_element *e)
{
	struct lkpi_mhi_xfer_ring *ring = NULL;
	struct lkpi_xfer_data *xfer;
	struct mhi_device *mdev;
	struct mhi_result res;
	void (*cb)(struct mhi_device *, struct mhi_result *);
	uint64_t rp, evrp, base;
	uint32_t tre0, tre1, code, chid, evlen;
	int status;

	tre0 = le32toh(e->dword[0]);
	tre1 = le32toh(e->dword[1]);

	evlen = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
	code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
	chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;
	/* PTR contains the entry that was last written */
	evrp = le64toh(e->ptr);

	switch (code) {
	case MHI_EV_CC_EOT:
	case MHI_EV_CC_EOB:
	case MHI_EV_CC_OVERFLOW:
		status = code == MHI_EV_CC_OVERFLOW ? -EOVERFLOW : 0;
		break;
	default:
		mhi_printf(psc, "%s: event code 0x%x on channel %u\n",
		    __func__, code, chid);
		status = -EIO;
		break;
	}

	if (chid < psc->max_chan)
		ring = psc->chan_map[chid];
	if (ring == NULL) {
		mhi_printf(psc, "%s: unhandled channel 0x%x\n", __func__,
		    chid);
		return;
	}

	mtx_lock(&ring->lock);
	/* Reset or stopped: the buffers are handed back there. */
	if (ring->mhi_chan_state != MHI_CH_STATE_ENABLED ||
	    ring->dmamem == NULL) {
		mtx_unlock(&ring->lock);
		return;
	}
	base = MHI_DMA_DVA(ring->dmamem);
	rp = evrp;
	if (rp < base || rp >= base + ring->size) {
		mhi_printf(psc, "%s: invalid ptr 0x%jx\n", __func__,
		    (uintmax_t)rp);
		mtx_unlock(&ring->lock);
		return;
	}
	/* Point rp to next empty slot */
	if (rp + sizeof(*e) >= base + ring->size)
		rp = base;
	else
		rp += sizeof(*e);
	/* Parse until next empty slot */
	while (ring->rp != rp && ring->queued > 0) {
		DNPRINTF(psc, "%s:%d: ring->rp 0x%jx ring->wp 0x%jx rp 0x%jx\n",
		    __func__, __LINE__, (uintmax_t)ring->rp,
		    (uintmax_t)ring->wp, (uintmax_t)rp);
		xfer = lkpi_pci_xfer_ring_get_data(ring, ring->rp);

		res.buf_addr = xfer->buf;
		res.dir = ring->dir;
		if (ring->dir == DMA_FROM_DEVICE && ring->rp == evrp)
			res.bytes_xferd = MIN(evlen, xfer->len);
		else
			res.bytes_xferd = xfer->len;
		res.transaction_status = status;

		if (xfer->buf != NULL)
			dma_unmap_single(psc->dev, xfer->dma, xfer->len,
			    ring->dir);
		DNPRINTF(psc, "%s: chan %u data (len %zu)\n", __func__,
		    ring->mhi_chan_id, res.bytes_xferd);
		xfer->buf = NULL;
		ring->queued--;

		if (ring->rp + sizeof(*e) >= base + ring->size)
			ring->rp = base;
		else
			ring->rp += sizeof(*e);

		mdev = ring->mhi_dev;
		cb = NULL;
		if (res.buf_addr != NULL && mdev != NULL &&
		    mdev->lkpi_drv != NULL)
			cb = ring->dir == DMA_TO_DEVICE ?
			    mdev->lkpi_drv->ul_xfer_cb :
			    mdev->lkpi_drv->dl_xfer_cb;
		if (cb == NULL)
			continue;
		/* The callback may queue buffers on this ring. */
		mtx_unlock(&ring->lock);
		cb(mdev, &res);
		mtx_lock(&ring->lock);
		if (ring->mhi_chan_state != MHI_CH_STATE_ENABLED)
			break;
	}
	mtx_unlock(&ring->lock);
}

static int
lkpi_pci_intr_data_event(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_event_ring *ring, int flush)
{
	struct mhi_event_ctxt *c;
	uint64_t rp, wp, base;
	struct mhi_ring_element *e;
	uint32_t tre0, tre1, type, code, chid, len;

	mtx_lock(&ring->lock);
	c = ring->event_ctxt;
	if (c == NULL || ring->dmamem == NULL) {
		/*
		 * Interrupts can trigger before mhi_init_event_rings()
		 * if the device is still active after a warm reboot.
		 */
		mtx_unlock(&ring->lock);
		return 0;
	}

	rmb();

	rp = le64toh(c->rp);
	wp = le64toh(c->wp);

	DNPRINTF(psc, "%s: kernel rp=0x%jx device rp=0x%jx kernel wp=0x%jx "
	    "device wp=0x%jx\n", __func__, (uintmax_t)ring->rp, (uintmax_t)rp,
	    (uintmax_t)ring->wp, (uintmax_t)wp);

	base = MHI_DMA_DVA(ring->dmamem);
	if (ring->rp == rp || rp < base || rp >= base + ring->size) {
		mtx_unlock(&ring->lock);
		return 0;
	}

	rmb();

	while (ring->rp != rp) {
		e = lkpi_pci_event_ring_get_elem(ring, ring->rp);
		if (e == NULL) {
			mtx_unlock(&ring->lock);
			return 0;
		}

		tre0 = le32toh(e->dword[0]);
		tre1 = le32toh(e->dword[1]);

		len = (tre0 & MHI_TRE0_EV_LEN_MASK) >> MHI_TRE0_EV_LEN_SHFT;
		code = (tre0 & MHI_TRE0_EV_CODE_MASK) >> MHI_TRE0_EV_CODE_SHFT;
		type = (tre1 & MHI_TRE1_EV_TYPE_MASK) >> MHI_TRE1_EV_TYPE_SHFT;
		chid = (tre1 & MHI_TRE1_EV_CHID_MASK) >> MHI_TRE1_EV_CHID_SHFT;
		DNPRINTF(psc, "%s: len=%u code=0x%x type=0x%x chid=%d\n",
		    __func__, len, code, type, chid);

		if (!flush) {
			switch (type) {
			case MHI_PKT_TYPE_TX_EVENT:
				lkpi_pci_intr_data_event_tx(psc, e);
				break;
			default:
				mhi_printf(psc, "%s: unhandled event type "
				    "0x%x\n", __func__, type);
				break;
			}
		}
		ring->nevents++;

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

	wmb();

	lkpi_mhi_ring_doorbell(psc, ring->db_addr, ring->wp);
	mtx_unlock(&ring->lock);
	return 1;
}

static int
lkpi_pci_intr_mhi_ctrl(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_event_ring *ring)
{
	if (lkpi_pci_intr_ctrl_event(psc, ring, 0))
		return 1;

	return 0;
}

static int
lkpi_pci_intr_mhi_data(struct lkpi_mhi_softc *psc,
    struct lkpi_mhi_event_ring *ring)
{
	if (lkpi_pci_intr_data_event(psc, ring, 0))
		return 1;

	return 0;
}

/* The BHI and state change part of the interrupt (irq[0]). */
static int
lkpi_pci_intr(struct lkpi_mhi_softc *psc)
{
	uint32_t ee, state, reg;
	int ret = 0;

	/*
	 * Interrupts can trigger before mhi_start() during boot if the device
	 * is still active after a warm reboot.
	 */
	if (psc->bhi_off == 0)
		psc->bhi_off = lkpi_pci_read(psc, MHI_BHI_OFFSET);

	ee = lkpi_pci_read(psc, psc->bhi_off + MHI_BHI_EXECENV);
	reg = lkpi_pci_read(psc, MHI_STATUS);
	state = (reg & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;

	DNPRINTF(psc,
	    "BHI interrupt with EE: 0x%x -> 0x%x state: 0x%x -> 0x%x\n",
	     psc->bhi_ee, ee, psc->mhi_state, state);

	mtx_lock(&psc->lock);
	if (reg == 0xffffffff) {
		/* The device is gone from the bus. */
		if (psc->powered && !psc->fatal) {
			mhi_printf(psc, "MHI registers unreadable\n");
			psc->fatal = true;
			psc->sys_err = true;
			lkpi_mhi_queue_status(psc, MHI_CB_FATAL_ERROR);
		}
		ret = 1;
	} else if (ee == MHI_EE_RDDM) {
		/* Firmware crash, e.g. due to invalid DMA memory access. */
		lkpi_mhi_state_change(psc, ee, -1);
		ret = 1;
	} else if (state == MHI_STATE_SYS_ERR) {
		lkpi_mhi_state_change(psc, -1, state);
		ret = 1;
	} else if (psc->bhi_ee == MHI_EE_PBL || psc->bhi_ee == MHI_EE_SBL) {
		int new_ee = -1, new_mhi_state = -1;

		if (psc->bhi_ee != ee)
			new_ee = ee;

		if (psc->mhi_state != state)
			new_mhi_state = state;

		if (new_ee != -1 || new_mhi_state != -1)
			lkpi_mhi_state_change(psc, new_ee, new_mhi_state);

		ret = 1;
	}
	/* BHI and BHIe waiters look at their status registers. */
	wakeup(&psc->bhi_ee);
	wakeup(&psc->bhie_off);
	mtx_unlock(&psc->lock);

	return ret;
}

/* Interrupt handler of one vector: BHI, and the event rings using it. */
static irqreturn_t
lkpi_pci_intr_vec(int irq __unused, void *arg)
{
	struct lkpi_mhi_vec *vec = arg;
	struct lkpi_mhi_softc *psc = vec->psc;
	struct lkpi_mhi_event_ring *ring;
	int i, ret = 0;

	if (vec->bhi && lkpi_pci_intr(psc))
		ret = 1;

	for (i = 0; i < psc->nevent_rings; i++) {
		ring = &psc->event_rings[i];
		if (ring->vec != vec)
			continue;
		if (ring->mhi_er_type == MHI_ER_CTRL) {
			if (lkpi_pci_intr_mhi_ctrl(psc, ring))
				ret = 1;
		} else {
			if (lkpi_pci_intr_mhi_data(psc, ring))
				ret = 1;
		}
	}

	return (ret ? IRQ_HANDLED : IRQ_NONE);
}

/* ------------------------------------------------------------------ */
/* Device wake, suspend and resume. */

int
linuxkpi_mhi_device_get_sync(struct mhi_device *mdev)
{
	struct mhi_controller *mhi = mdev->mhi_cntrl;
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	int ret;

	ret = mhi->runtime_get(mhi);
	if (ret < 0)
		return ret;

	mtx_lock(&psc->lock);
	if (!psc->powered || psc->sys_err || psc->wake_db == 0 ||
	    psc->mhi_state == MHI_STATE_M3 || psc->suspended) {
		mtx_unlock(&psc->lock);
		mhi->runtime_put(mhi);
		return -EIO;
	}
	lkpi_mhi_device_wake(psc);
	mdev->lkpi_dev_wake++;
	mtx_unlock(&psc->lock);

	/* Out of M1/M2 the asserted doorbell brings the device to M0. */
	ret = lkpi_mhi_wait(psc, &psc->mhi_state, "mhiwake",
	    lkpi_mhi_check_m0, psc, NULL);
	if (ret) {
		mhi_printf(psc, "device did not wake up: %d\n", ret);
		mtx_lock(&psc->lock);
		mdev->lkpi_dev_wake--;
		lkpi_mhi_device_zzz(psc);
		mtx_unlock(&psc->lock);
		mhi->runtime_put(mhi);
		return ret;
	}
	return 0;
}

void
linuxkpi_mhi_device_put(struct mhi_device *mdev)
{
	struct mhi_controller *mhi = mdev->mhi_cntrl;
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	bool held;

	mtx_lock(&psc->lock);
	held = mdev->lkpi_dev_wake > 0;
	if (held) {
		mdev->lkpi_dev_wake--;
		lkpi_mhi_device_zzz(psc);
	}
	mtx_unlock(&psc->lock);
	if (held)
		mhi->runtime_put(mhi);
}

int
linuxkpi_mhi_pm_suspend(struct mhi_controller *mhi)
{
	struct lkpi_mhi_softc *psc = mhi->lkpi_priv;
	struct lkpi_mhi_xfer_ring *ring;
	bool busy;
	int i, ret;

	mtx_lock(&psc->lock);
	if (!psc->powered || !psc->mission || psc->sys_err ||
	    psc->suspended || psc->mhi_state != MHI_STATE_M0) {
		mtx_unlock(&psc->lock);
		return -EIO;
	}
	if (psc->wake_refcnt > 0) {
		mtx_unlock(&psc->lock);
		return -EBUSY;
	}
	/* From here on mhi_queue_buf() holds back the doorbells. */
	psc->suspended = true;
	mtx_unlock(&psc->lock);

	busy = false;
	for (i = 0; i < psc->nxfer_rings && !busy; i++) {
		ring = &psc->xfer_rings[i];
		mtx_lock(&ring->lock);
		busy = ring->dir == DMA_TO_DEVICE && ring->queued > 0;
		mtx_unlock(&ring->lock);
	}
	if (busy) {
		mtx_lock(&psc->lock);
		psc->suspended = false;
		mtx_unlock(&psc->lock);
		return -EBUSY;
	}

	lkpi_mhi_notify_clients(psc, MHI_CB_LPM_ENTER);

	lkpi_mhi_set_state(psc, MHI_STATE_M3);
	ret = lkpi_mhi_wait(psc, &psc->mhi_state, "mhim3", lkpi_mhi_check_m3,
	    NULL, NULL);
	if (ret) {
		mhi_printf(psc, "MHI M3 transition timeout (state=0x%x)\n",
		    lkpi_mhi_status_state(lkpi_pci_read(psc, MHI_STATUS)));
		/* Leave the device as it was. */
		lkpi_mhi_set_state(psc, MHI_STATE_M0);
		mtx_lock(&psc->lock);
		psc->suspended = false;
		mtx_unlock(&psc->lock);
		for (i = 0; i < psc->nxfer_rings; i++) {
			ring = &psc->xfer_rings[i];
			mtx_lock(&ring->lock);
			if (ring->db_pending) {
				ring->db_pending = false;
				lkpi_mhi_ring_doorbell(psc, ring->db_addr,
				    ring->wp);
			}
			mtx_unlock(&ring->lock);
		}
		lkpi_mhi_notify_clients(psc, MHI_CB_LPM_EXIT);
		return -EIO;
	}

	mtx_lock(&psc->lock);
	psc->mhi_state = MHI_STATE_M3;
	mtx_unlock(&psc->lock);
	DNPRINTF(psc, "%s: in M3\n", __func__);
	return 0;
}

static int
lkpi_mhi_pm_resume(struct lkpi_mhi_softc *psc, bool force)
{
	struct lkpi_mhi_xfer_ring *ring;
	uint32_t state;
	int i, ret;

	mtx_lock(&psc->lock);
	if (!psc->powered || psc->sys_err) {
		mtx_unlock(&psc->lock);
		return -EIO;
	}
	if (!force && psc->mhi_state != MHI_STATE_M3) {
		mtx_unlock(&psc->lock);
		return -EINVAL;
	}
	mtx_unlock(&psc->lock);

	state = (lkpi_pci_read(psc, MHI_STATUS) & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT;
	if (state != MHI_STATE_M0) {
		lkpi_mhi_set_state(psc, MHI_STATE_M0);
		ret = lkpi_mhi_wait(psc, &psc->mhi_state, "mhim0",
		    lkpi_mhi_check_m0, psc, NULL);
		if (ret) {
			mhi_printf(psc, "MHI M0 transition timeout "
			    "(state=0x%x)\n", lkpi_mhi_status_state(
			    lkpi_pci_read(psc, MHI_STATUS)));
			return ret;
		}
	}

	mtx_lock(&psc->lock);
	psc->mhi_state = MHI_STATE_M0;
	psc->suspended = false;
	mtx_unlock(&psc->lock);

	/* The doorbells of the work queued while suspended. */
	for (i = 0; i < psc->nxfer_rings; i++) {
		ring = &psc->xfer_rings[i];
		mtx_lock(&ring->lock);
		if (ring->db_pending) {
			ring->db_pending = false;
			if (ring->mhi_chan_state == MHI_CH_STATE_ENABLED)
				lkpi_mhi_ring_doorbell(psc, ring->db_addr,
				    ring->wp);
		}
		mtx_unlock(&ring->lock);
	}

	lkpi_mhi_notify_clients(psc, MHI_CB_LPM_EXIT);
	DNPRINTF(psc, "%s: in M0\n", __func__);
	return 0;
}

int
linuxkpi_mhi_pm_resume(struct mhi_controller *mhi)
{
	return (lkpi_mhi_pm_resume(mhi->lkpi_priv, false));
}

int
linuxkpi_mhi_pm_resume_force(struct mhi_controller *mhi)
{
	return (lkpi_mhi_pm_resume(mhi->lkpi_priv, true));
}

/* ------------------------------------------------------------------ */
/* State dump. */

static const char *
lkpi_mhi_state_str(uint32_t state)
{
	switch (state) {
	case MHI_STATE_RESET:	return "RESET";
	case MHI_STATE_READY:	return "READY";
	case MHI_STATE_M0:	return "M0";
	case MHI_STATE_M1:	return "M1";
	case MHI_STATE_M2:	return "M2";
	case MHI_STATE_M3:	return "M3";
	case MHI_STATE_M3_FAST:	return "M3_FAST";
	case MHI_STATE_BHI:	return "BHI";
	case MHI_STATE_SYS_ERR:	return "SYS_ERR";
	default:		return "?";
	}
}

static const char *
lkpi_mhi_ee_str(uint32_t ee)
{
	switch (ee) {
	case MHI_EE_PBL:	return "PBL";
	case MHI_EE_SBL:	return "SBL";
	case MHI_EE_AMSS:	return "AMSS";
	case MHI_EE_RDDM:	return "RDDM";
	case MHI_EE_WFW:	return "WFW";
	case MHI_EE_PTHRU:	return "PTHRU";
	case MHI_EE_EDL:	return "EDL";
	case MHI_EE_FP:		return "FP";
	default:		return "?";
	}
}

static const char *
lkpi_mhi_xfer_status_str(uint32_t status)
{
	switch (status) {
	case MHI_BHI_STATUS_RESET:	return "reset";
	case MHI_BHI_STATUS_SUCCESS:	return "done";
	case MHI_BHI_STATUS_ERROR:	return "ERROR";
	default:			return "?";
	}
}

/*
 * Print where the device is and what the host thinks, for power up
 * failures and compat.linuxkpi.mhi_dump.
 */
static void
lkpi_mhi_dump_state(struct lkpi_mhi_softc *psc, const char *why)
{
	struct mhi_controller *mhi = psc->mhi;
	struct lkpi_mhi_event_ring *er;
	struct lkpi_mhi_xfer_ring *ring;
	struct mhi_chan_ctxt *cc;
	uint32_t ctrl, status, cfg, bhi_off, bhie_off, ee, bhist, txst, rxst;
	uint64_t drp, hrp;
	int i;

	mhi_printf(psc, "MHI state dump (%s): mhi%d\n", why, psc->index);
	ctrl = lkpi_pci_read(psc, MHI_CTRL);
	status = lkpi_pci_read(psc, MHI_STATUS);
	cfg = lkpi_pci_read(psc, MHI_CFG);
	mhi_printf(psc, "  device: MHICTRL 0x%08x (state %s%s), MHISTATUS "
	    "0x%08x (state %s, ready %d, syserr %d), MHICFG 0x%08x\n", ctrl,
	    lkpi_mhi_state_str((ctrl & MHI_CTRL_MHISTATE_MASK) >>
	    MHI_CTRL_MHISTATE_SHFT), (ctrl & MHI_CTRL_RESET_MASK) ?
	    ", reset" : "", status,
	    lkpi_mhi_state_str((status & MHI_STATUS_MHISTATE_MASK) >>
	    MHI_STATUS_MHISTATE_SHFT), (status & MHI_STATUS_READY_MASK) != 0,
	    (status & MHI_STATUS_SYSERR_MASK) != 0, cfg);

	bhi_off = psc->bhi_off != 0 ? psc->bhi_off :
	    lkpi_pci_read(psc, MHI_BHI_OFFSET);
	bhie_off = psc->bhie_off != 0 ? psc->bhie_off :
	    lkpi_pci_read(psc, MHI_BHIE_OFFSET);
	if (bhi_off != 0 && bhi_off < mhi->reg_len &&
	    bhie_off != 0 && bhie_off < mhi->reg_len) {
		ee = lkpi_pci_read(psc, bhi_off + MHI_BHI_EXECENV);
		bhist = lkpi_pci_read(psc, bhi_off + MHI_BHI_STATUS);
		txst = lkpi_pci_read(psc, bhie_off + MHI_BHIE_TXVECSTATUS_OFFS);
		rxst = lkpi_pci_read(psc, bhie_off + MHI_BHIE_RXVECSTATUS_OFFS);
		mhi_printf(psc, "  device: EE %s (0x%x), BHI status 0x%08x "
		    "(%s), BHIe tx status 0x%08x (%s), rx status 0x%08x (%s)\n",
		    lkpi_mhi_ee_str(ee), ee, bhist, lkpi_mhi_xfer_status_str(
		    (bhist & MHI_BHI_STATUS_MASK) >> MHI_BHI_STATUS_SHFT),
		    txst, lkpi_mhi_xfer_status_str((txst &
		    MHI_BHIE_TXVECSTATUS_STATUS_BMSK) >>
		    MHI_BHIE_TXVECSTATUS_STATUS_SHFT), rxst,
		    lkpi_mhi_xfer_status_str((rxst &
		    MHI_BHIE_RXVECSTATUS_STATUS_BMSK) >>
		    MHI_BHIE_RXVECSTATUS_STATUS_SHFT));
	} else {
		mhi_printf(psc, "  device: BHI/BHIe offsets 0x%x/0x%x "
		    "invalid\n", bhi_off, bhie_off);
	}

	mhi_printf(psc, "  host: EE %s, state %s, prepared %d powered %d "
	    "mission %d suspended %d sys_err %d, wake refs %d, "
	    "timeout %u ms\n", lkpi_mhi_ee_str(psc->bhi_ee),
	    lkpi_mhi_state_str(psc->mhi_state), psc->prepared, psc->powered,
	    psc->mission, psc->suspended, psc->sys_err, psc->wake_refcnt,
	    psc->timeout_ms);
	for (i = 0; i < psc->nvecs; i++)
		mhi_printf(psc, "  irq %#x: %s%s%s\n", psc->vecs[i].irq,
		    psc->vecs[i].bhi ? "BHI/state, " : "",
		    psc->vecs[i].requested ? "requested" : "not requested",
		    psc->vecs[i].enabled ? ", enabled" : "");

	if (!psc->prepared || psc->event_ctxt == NULL)
		return;
	for (i = 0; i < psc->nevent_rings; i++) {
		er = &psc->event_rings[i];
		if (er->event_ctxt == NULL || er->dmamem == NULL)
			continue;
		rmb();
		drp = le64toh(er->event_ctxt->rp);
		hrp = er->rp;
		mhi_printf(psc, "  event ring %d (%s, irq %#x, msivec %u): "
		    "device rp 0x%jx, host rp 0x%jx wp 0x%jx, %lu events "
		    "processed\n", i, er->mhi_er_type == MHI_ER_CTRL ?
		    "ctrl" : "data", er->vec->irq, er->mhi_er_irq,
		    (uintmax_t)drp, (uintmax_t)hrp, (uintmax_t)er->wp,
		    er->nevents);
		if (drp != hrp) {
			mhi_printf(psc, "  event ring %d: the device wrote "
			    "%ju event(s) the host has not processed%s\n", i,
			    (uintmax_t)(((drp - hrp) + er->size) % er->size /
			    sizeof(struct mhi_ring_element)),
			    er->nevents == 0 ?
			    ", and none since power up: interrupt not "
			    "delivered?" : ": interrupt not delivered?");
		}
	}
	for (i = 0; i < psc->nxfer_rings; i++) {
		ring = &psc->xfer_rings[i];
		cc = ring->chan_ctxt;
		if (cc == NULL || ring->dmamem == NULL)
			continue;
		rmb();
		mhi_printf(psc, "  channel %u (%s, %s): host state %u, "
		    "%d queued, host rp 0x%jx wp 0x%jx, device state %u rp "
		    "0x%jx\n", ring->mhi_chan_id, ring->name,
		    ring->dir == DMA_TO_DEVICE ? "to device" : "from device",
		    ring->mhi_chan_state, ring->queued, (uintmax_t)ring->rp,
		    (uintmax_t)ring->wp, (uint32_t)(le32toh(cc->chcfg) &
		    MHI_CHAN_CTX_CHSTATE_MASK), (uintmax_t)le64toh(cc->rp));
	}
	if (psc->cmd_ctxt != NULL && psc->cmd_ring.dmamem != NULL)
		mhi_printf(psc, "  command ring: host rp 0x%jx wp 0x%jx, "
		    "%d queued, device rp 0x%jx\n",
		    (uintmax_t)psc->cmd_ring.rp, (uintmax_t)psc->cmd_ring.wp,
		    psc->cmd_ring.queued, (uintmax_t)le64toh(
		    ((struct mhi_cmd_ctxt *)MHI_DMA_KVA(psc->cmd_ctxt))->rp));
}

void
lkpi_mhi_dump_all(void)
{
	struct lkpi_mhi_softc *psc;

	sx_slock(&lkpi_mhi_bus_lock);
	list_for_each_entry(psc, &lkpi_mhi_controllers, node) {
		sx_slock(&psc->mem_lock);
		lkpi_mhi_dump_state(psc, "requested");
		sx_sunlock(&psc->mem_lock);
	}
	sx_sunlock(&lkpi_mhi_bus_lock);
}

bool
lkpi_mhi_busy(void)
{
	bool busy;

	sx_slock(&lkpi_mhi_bus_lock);
	busy = !list_empty(&lkpi_mhi_controllers);
	sx_sunlock(&lkpi_mhi_bus_lock);
	return (busy);
}
