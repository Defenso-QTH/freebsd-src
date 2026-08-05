/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Defenso
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
 * virtio-gpu device emulation backed by virglrenderer.
 *
 * Implements VIRTIO_GPU_F_VIRGL for 3D acceleration through the host GPU's
 * EGL render node.  Designed for headless use (RDP delivery of the guest
 * desktop); scanout flush is a no-op.
 *
 * Configuration:
 *   -s <slot>,virtio-gpu[,render=/dev/dri/renderD128]
 */

#include <sys/param.h>
#include <sys/event.h>
#include <sys/linker_set.h>
#include <sys/queue.h>
#include <sys/uio.h>

#include <assert.h>
#include <err.h>
#include <errno.h>
#include <pthread.h>
#include <pthread_np.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <dev/vmm/vmm_mem.h>	/* VM_VIRTIO_GPU_HOSTVIS devmem segid */
#include <machine/vmm.h>
#include <vmmapi.h>		/* vm_create_devmem / vm_mmap_memseg */

/*
 * virgl_renderer_export_fence() -- the only way to obtain a sync_file for the
 * guest's rendering -- sits behind this in virglrenderer.h, which says the APIs
 * it guards are "for development/testing purposes only, not in production".
 * Taken deliberately: without a fence the viewer samples buffers mid-render.
 * If it ever disappears the build breaks loudly rather than silently, and the
 * fallback is simply not to fence.
 */
#define	VIRGL_RENDERER_UNSTABLE_APIS 1
#include <virglrenderer.h>

/* virgl_protocol.h is not installed by the virglrenderer port; define here. */
struct virgl_box {
	uint32_t x, y, z;
	uint32_t w, h, d;
};

#include "bhyverun.h"
#include "config.h"
#include "debug.h"
#include "pci_emul.h"
#include "console.h"
#include "gpu_display.h"
#include "virtio.h"

/* Import the protocol structs from the kernel tree. */
#include <dev/virtio/gpu/virtio_gpu.h>

#define	VTGPU_RINGSZ		128
#define	VTGPU_CONTROLQ		0
#define	VTGPU_CURSORQ		1
#define	VTGPU_MAXQ		2

/* One output for now; the guest will see one logical display. */
#define	VTGPU_NUM_SCANOUTS	1

/* Announce virgl 3D support. */
#define	VTGPU_S_HOSTCAPS	(1ULL << VIRTIO_GPU_F_VIRGL)

/*
 * Default virtual display geometry; can be overridden via config key
 * "width" and "height".
 */
#define	VTGPU_DEFAULT_WIDTH	1920
#define	VTGPU_DEFAULT_HEIGHT	1080

/* Max iov entries we copy for an ATTACH_BACKING command. */
/*
 * Upper bound on mem entries accepted in one ATTACH_BACKING / CREATE_BLOB.
 * The real limit is how many entries the received command buffer actually
 * holds (checked per-command); this is only a sanity ceiling so a bogus
 * nr_entries cannot make us allocate absurdly.  It must be generous: a
 * 1280x720 BGRA surface is 900 pages, and a scattered 4K one is ~2000, so
 * the old value of 256 silently rejected ordinary compositor buffers.
 */
#define	VTGPU_MAX_BACKING	65536

/*
 * virtio-gpu is a modern-only device: its Linux driver refuses any device
 * that does not offer VIRTIO_F_VERSION_1, which the legacy virtio-pci
 * transport cannot express.  bhyve's shared vi_ layer is legacy-only, so
 * this device implements the modern virtio-pci transport directly (below).
 */
#define	VIRTIO_ID_GPU		16		/* virtio device type */
#define	VIRTIO_DEV_GPU		(0x1040 + VIRTIO_ID_GPU)  /* modern PCI id 0x1050 */

/*
 * In the FreeBSD virtio headers VIRTIO_F_VERSION_1 is a feature *mask*
 * (1<<32), not a bit number, so it cannot be used as a shift count.  Use
 * an explicit bit position when assembling the offered feature word.
 */
#define	VTGPU_F_VERSION_1_BIT	32
#ifndef	VIRTIO_MSI_NO_VECTOR
#define	VIRTIO_MSI_NO_VECTOR	0xffff
#endif

/* Features this device offers the guest (64-bit modern feature space). */
#define	VTGPU_MODERN_FEATURES	\
	((1ULL << VIRTIO_GPU_F_VIRGL) | (1ULL << VTGPU_F_VERSION_1_BIT))

/*
 * With venus=on we additionally negotiate CONTEXT_INIT (so the guest kernel
 * exposes context-init / venus contexts to userspace) and RESOURCE_BLOB (the
 * venus command ring is a guest-memory blob resource).  Without these two
 * bits the guest's venus ICD reports "no valid GPUs".  The feature numbers
 * come from <dev/virtio/gpu/virtio_gpu.h>.
 */
#define	VTGPU_VENUS_FEATURES	\
	(VTGPU_MODERN_FEATURES | \
	 (1ULL << VIRTIO_GPU_F_CONTEXT_INIT) | \
	 (1ULL << VIRTIO_GPU_F_RESOURCE_BLOB))

/*
 * Capset id for the Venus (Vulkan) renderer.  virtio_gpu.h only defines the
 * VIRGL/VIRGL2 capsets; VENUS is advertised as a third capset (index 2) when
 * the device is started with venus=on.
 */
#define	VIRTIO_GPU_CAPSET_VENUS		4

/* device_status bits (virtio 1.0 s2.1). */
#define	VTGPU_S_ACKNOWLEDGE	0x01
#define	VTGPU_S_DRIVER		0x02
#define	VTGPU_S_DRIVER_OK	0x04
#define	VTGPU_S_FEATURES_OK	0x08
#define	VTGPU_S_NEEDS_RESET	0x40
#define	VTGPU_S_FAILED		0x80

/* virtio_pci_cap.cfg_type values. */
#define	VTGPU_CAP_COMMON_CFG	1
#define	VTGPU_CAP_NOTIFY_CFG	2
#define	VTGPU_CAP_ISR_CFG	3
#define	VTGPU_CAP_DEVICE_CFG	4
#define	VTGPU_CAP_PCI_CFG	5
#define	VTGPU_CAP_SHARED_MEMORY_CFG 8	/* virtio shared-memory region cap */

/*
 * Host-visible memory window (venus).  A separate MEM64 BAR backed by a
 * devmem segment (VM_VIRTIO_GPU_HOSTVIS), advertised to the guest via a
 * virtio SHARED_MEMORY cap with shmid VIRTIO_GPU_SHM_ID_HOST_VISIBLE so the
 * guest kernel reports "+host_visible" and the venus ICD will attach.
 */
/* virgl_hw.h resource bind bits we care about here. */
#define	VTGPU_BIND_SCANOUT	(1u << 18)
#define	VTGPU_BIND_LINEAR	(1u << 22)

#define	VTGPU_HOSTVIS_BAR	2		/* MEM64 -> consumes BARs 2 and 3 */
/*
 * Size of the host-visible window, and therefore the size of the only
 * heap the guest can allocate mappable Vulkan memory from -- the guest
 * driver reads it off the BAR and refuses anything larger itself, which
 * is why exhausting it produces no host-side error at all.
 *
 * 256 MiB was chosen when vkcube was the only workload.  A real game
 * blows through it: CK3 asks for a 162 MiB texture alongside 67 and 84
 * MiB blobs, which cannot coexist in 256 MiB, and DXVK reports the
 * refusal as a failed CreateTexture2D.
 *
 * Enlarging it is close to free.  The BAR is backed by a devmem segment,
 * which vm_alloc_memseg() creates as an OBJT_SWAP object, so pages are
 * committed on first touch rather than up front -- and blob mappings
 * alias real host memory over the range anyway, so most of it is never
 * touched.  The cost is guest address space, and bhyve's 64-bit MMIO
 * window is 32 GB (PCI_EMUL_MEMSIZE64).
 */
#define	VTGPU_HOSTVIS_SZ	(4ULL << 30)	/* 4 GiB */

/*
 * Modern config BAR (BAR 4, MEM64) layout.  One 4 KiB page per structure
 * so the offsets are trivially aligned; the guest is told exactly where
 * each lives via the vendor-specific PCI capabilities.
 */
#define	VTGPU_MODERN_BAR	4
#define	VTGPU_MSIX_BAR		1
#define	VTGPU_OFF_COMMON	0x0000u
#define	VTGPU_OFF_ISR		0x1000u
#define	VTGPU_OFF_DEVICE	0x2000u
#define	VTGPU_OFF_NOTIFY	0x3000u
#define	VTGPU_REGION_LEN	0x1000u
#define	VTGPU_MODERN_BAR_SZ	0x4000u
#define	VTGPU_NOTIFY_MULT	4u	/* queue_notify_off[N] == N */

/* Register offsets within the common-config region (virtio_pci_common_cfg). */
#define	VTGPU_CC_DFSELECT	0x00	/* device_feature_select   (32) */
#define	VTGPU_CC_DF		0x04	/* device_feature      RO  (32) */
#define	VTGPU_CC_GFSELECT	0x08	/* guest_feature_select    (32) */
#define	VTGPU_CC_GF		0x0c	/* guest_feature           (32) */
#define	VTGPU_CC_MSIXCFG	0x10	/* msix_config             (16) */
#define	VTGPU_CC_NUMQ		0x12	/* num_queues          RO  (16) */
#define	VTGPU_CC_STATUS		0x14	/* device_status            (8) */
#define	VTGPU_CC_CFGGEN		0x15	/* config_generation   RO   (8) */
#define	VTGPU_CC_QSELECT	0x16	/* queue_select            (16) */
#define	VTGPU_CC_QSIZE		0x18	/* queue_size              (16) */
#define	VTGPU_CC_QMSIX		0x1a	/* queue_msix_vector       (16) */
#define	VTGPU_CC_QENABLE	0x1c	/* queue_enable            (16) */
#define	VTGPU_CC_QNOFF		0x1e	/* queue_notify_off    RO  (16) */
#define	VTGPU_CC_QDESCLO	0x20	/* queue_desc_lo           (32) */
#define	VTGPU_CC_QDESCHI	0x24	/* queue_desc_hi           (32) */
#define	VTGPU_CC_QAVAILLO	0x28	/* queue_avail_lo          (32) */
#define	VTGPU_CC_QAVAILHI	0x2c	/* queue_avail_hi          (32) */
#define	VTGPU_CC_QUSEDLO	0x30	/* queue_used_lo           (32) */
#define	VTGPU_CC_QUSEDHI	0x34	/* queue_used_hi           (32) */

static int pci_vtgpu_debug;
#define	DPRINTF(fmt, ...)					\
	do {							\
		if (pci_vtgpu_debug)				\
			fprintf(stderr, "vtgpu: " fmt "\n",	\
			    ##__VA_ARGS__);			\
	} while (0)

/*
 * A pending fenced command: holds the vq chain index and queue pointer
 * so that the write_fence callback can complete the response.
 */
struct vtgpu_fence {
	uint64_t		 vf_id;
	struct vqueue_info	*vf_vq;
	uint16_t		 vf_idx;
	uint32_t		 vf_resp_len;
	TAILQ_ENTRY(vtgpu_fence) vf_link;
};

/*
 * Fence ids we create ourselves start here.  vtgpu_write_fence() retires every
 * queued fence with an id <= the one that signalled, which assumes a single
 * monotonic space; injecting ours into it would retire the guest's fences early
 * and release their descriptors before their work had finished.  Keeping ours
 * in the top half lets write_fence recognise and ignore them.
 */
#define	VTGPU_OWN_FENCE_BASE	0x80000000u

/* kqueue ident for the queue-kick user event. */
#define	VTGPU_KQ_NOTIFY		1

/*
 * Backstop for the kqueue wait.  Measured 2026-08-01 under a real
 * workload: ~90% of fence waits are woken by an event, ~10% fall through
 * to this timeout.  That 10% is why the backstop must stay at 1ms.  It
 * was briefly 10ms, on the assumption that events would cover everything
 * -- which left the mean wait unchanged (0.9*0 + 0.1*10ms == the 1ms
 * every wait used to cost) while making the tail ten times worse.  A
 * 10ms stall is over half a frame at 60fps, so the spikes hurt pacing
 * more than the old uniform 1ms did.  At 1ms this path is never worse
 * than the condvar it replaced, and ~10x better on the 90% that are
 * woken by an event.  Do not raise it without re-measuring the miss rate.
 */
#define	VTGPU_KQ_BACKSTOP_MS	1

struct vtgpu_softc {
	struct virtio_softc	vsc_vs;
	struct virtio_gpu_config vsc_cfg;
	bool			vsc_venus;	/* advertise Venus (Vulkan) capset */
	struct vqueue_info	vsc_queues[VTGPU_MAXQ];
	pthread_mutex_t		vsc_mtx;

	/* worker thread */
	pthread_t		vsc_tid;
	pthread_cond_t		vsc_cnd;
	bool			vsc_running;
	/*
	 * virglrenderer init runs ON the worker thread (its host GL context
	 * is thread-bound; creating it on one thread and rendering on another
	 * makes glXMakeContextCurrent fail with BadAccess, which Xlib turns
	 * into a process abort).  The worker signals the result here and
	 * pci_vtgpu_init waits for it: 0 = pending, 1 = ok, -1 = failed.
	 */
	int			vsc_init_done;

	/* fence tracking (vtgpu_write_fence fires from the worker thread) */
	TAILQ_HEAD(, vtgpu_fence) vsc_fences;
	uint64_t		vsc_fence_next;

	struct vmctx		*vsc_ctx;
	uint32_t		vsc_width;
	uint32_t		vsc_height;
	/*
	 * Host DRM render node fd (e.g. /dev/dri/renderD128), opened at init.
	 * Handed to virglrenderer via the get_drm_fd callback so it brings up
	 * EGL on the GBM platform against this node — a fully headless GPU
	 * context that needs no Wayland/X display.  -1 if none was opened.
	 */
	int			vsc_drm_fd;
	/*
	 * Event-driven wake-up.  vsc_kq multiplexes the two independent
	 * sources that can make the worker runnable: virglrenderer fence
	 * progress (EVFILT_READ on vsc_poll_fd) and a guest queue kick
	 * (EVFILT_USER, triggered by vtgpu_qnotify).  Both are needed: a
	 * fence-blocked guest never rings a queue, and a queue kick can
	 * arrive with no fence outstanding.  Either being < 0 means the
	 * kqueue path is unavailable and we fall back to the condvar.
	 */
	int			vsc_kq;
	int			vsc_poll_fd;
	/*
	 * Scanout readback probe (scanout_probe=on).  Measures what it would
	 * cost to present the guest's scanout from the host: on RESOURCE_FLUSH
	 * of the scanout resource, read it back with transfer_read_iov into a
	 * host buffer and time it.  Nothing is displayed -- this only answers
	 * whether a host-side present is affordable before anything is built
	 * on top of it.
	 */
	/*
	 * External viewer (display=unix:/path).  NULL when unconfigured, which
	 * is the default: the device then behaves exactly as before.
	 */
	struct gpu_display	*vsc_display;
	bool			vsc_scanout_probe;
	bool			vsc_scanout_linear;
	/*
	 * Scanout resource ids already reported.  A page-flipping compositor
	 * rebinds the scanout every frame -- sway alternates two buffers -- so
	 * without this the probe would query virglrenderer and write two log
	 * lines sixty times a second, on the worker thread, in the guest's
	 * command path.  Each distinct buffer is worth describing once.
	 */
	uint32_t		vsc_seen_scanout[8];
	unsigned		vsc_seen_n;
	/* Set by SET_SCANOUT, consumed by the response path. */
	unsigned		vsc_scanout_seen_total;
	uint32_t		vsc_own_fence_next;
	unsigned		vsc_fence_reports;
	/*
	 * Context that created each scanout-capable resource.  A fence has to
	 * be created on the context that did the drawing, and SET_SCANOUT
	 * arrives on the control queue where ctx_id is not that context.
	 */
	struct {
		uint32_t	res_id;
		uint32_t	ctx_id;
	}			vsc_scanout_ctx[8];
	unsigned		vsc_scanout_ctx_n;
	uint32_t		vsc_scanout_res;	/* 0 = none bound */
	uint32_t		vsc_scanout_w;
	uint32_t		vsc_scanout_h;
	void			*vsc_scanout_buf;
	size_t			vsc_scanout_bufsz;
	uint64_t		vsc_ro_n;		/* readbacks timed */
	uint64_t		vsc_ro_ns;		/* cumulative ns */
	uint64_t		vsc_ro_min;
	uint64_t		vsc_ro_max;
	uint64_t		vsc_ro_fail;
	uint64_t		vsc_blob_hv;	/* host-visible blobs created */
	uint64_t		vsc_unref;	/* RESOURCE_UNREF commands seen */
	uint64_t		vsc_fwait;	/* fence waits entered */
	uint64_t		vsc_fwait_late;	/* ... that hit the backstop */

	/*
	 * Host-visible memory window (venus).  vsc_hostvis_base is the bhyve
	 * process mapping of the devmem segment; vsc_hostvis_gpa is the guest
	 * physical address the BAR is currently mapped at (0 = not mapped).
	 */
	void			*vsc_hostvis_base;
	uint64_t		vsc_hostvis_gpa;

	/*
	 * Active host-visible blob mappings (venus).  Each RESOURCE_MAP_BLOB
	 * aliases a virglrenderer-owned host mapping into the window at a guest
	 * offset; we track the guest range so RESOURCE_UNMAP_BLOB (which carries
	 * only the resource id) can tear it down.
	 */
/*
 * Concurrent host-visible blob mappings.  64 was chosen when only vkcube and
 * vulkaninfo had ever run; a real game holds far more mapped at once.  Slots
 * are released by RESOURCE_UNMAP_BLOB and RESOURCE_UNREF, so this bounds live
 * mappings rather than lifetime allocations -- but exhausting it drops guest
 * commands, so map_blob logs unconditionally when it does.
 */
#define	VTGPU_MAX_BLOB_MAPS	1024
	struct vtgpu_blob_map {
		uint32_t	res_id;
		uint64_t	gpa;	/* guest phys addr in the window */
		uint64_t	len;	/* page-rounded length */
		bool		used;
	}			vsc_blob_maps[VTGPU_MAX_BLOB_MAPS];

	/*
	 * Modern virtio-pci (virtio 1.0) transport state.  bhyve's shared
	 * vi_ layer is legacy-only; virtio-gpu requires VIRTIO_F_VERSION_1,
	 * so this device implements the modern transport itself (a MEM64 BAR
	 * carrying the common/notify/isr/device-config regions).  The ring
	 * processing (vq_getchain/vq_relchain/...) is transport-agnostic and
	 * reused unchanged; only queue setup and the register file differ.
	 */
	uint32_t		vsc_dev_feature_sel;	/* device_feature_select */
	uint32_t		vsc_drv_feature_sel;	/* guest_feature_select */
	uint64_t		vsc_drv_features;	/* features guest accepted */
	uint16_t		vsc_msix_cfg;		/* config MSI-X vector */
	uint8_t			vsc_status;		/* device_status */
	uint8_t			vsc_cfg_gen;		/* config_generation */
	uint16_t		vsc_qsel;		/* queue_select */
	/* per-queue modern ring addresses programmed by the guest */
	uint64_t		vsc_q_desc[VTGPU_MAXQ];
	uint64_t		vsc_q_avail[VTGPU_MAXQ];
	uint64_t		vsc_q_used[VTGPU_MAXQ];
	uint16_t		vsc_q_enable[VTGPU_MAXQ];
};

/* ----------------------------------------------------------------------- */
/* virglrenderer callbacks						   */
/* ----------------------------------------------------------------------- */

static void
vtgpu_write_fence(void *cookie, uint32_t fence_id)
{
	struct vtgpu_softc *sc = cookie;
	struct vtgpu_fence *vf, *tmp;

	/*
	 * Called from within virgl_renderer_poll(), which we invoke on the
	 * worker thread — so vsc_mtx is already held by the caller.
	 */
	if (fence_id >= VTGPU_OWN_FENCE_BASE)
		return;		/* ours; nothing is queued against it */
	TAILQ_FOREACH_SAFE(vf, &sc->vsc_fences, vf_link, tmp) {
		if ((uint32_t)vf->vf_id > fence_id)
			break;
		TAILQ_REMOVE(&sc->vsc_fences, vf, vf_link);
		vq_relchain(vf->vf_vq, vf->vf_idx, vf->vf_resp_len);
		vq_endchains(vf->vf_vq, 0);
		free(vf);
	}
}

/*
 * Supply the host render-node fd to virglrenderer (version-2 callback).
 * When this returns a valid fd, virglrenderer's EGL init uses the GBM
 * platform on that node instead of falling back to EGL_DEFAULT_DISPLAY,
 * so no window-system (Wayland/X) display is needed.  We dup on each call
 * because virglrenderer takes ownership of the fd it receives, and the
 * init fallback chain may query us more than once.
 */
static int
vtgpu_get_drm_fd(void *cookie)
{
	struct vtgpu_softc *sc = cookie;

	if (sc->vsc_drm_fd < 0)
		return (-1);
	return (dup(sc->vsc_drm_fd));
}

static struct virgl_renderer_callbacks vtgpu_virgl_cbs = {
	.version     = 2,
	.write_fence = vtgpu_write_fence,
	.get_drm_fd  = vtgpu_get_drm_fd,
};

/* ----------------------------------------------------------------------- */
/* Response helpers							   */
/* ----------------------------------------------------------------------- */

/*
 * Copy a response into the writable part of the descriptor chain and
 * release it.  If this command carried a fence, queue the release instead
 * and let vtgpu_write_fence() do it once the fence fires.
 */
static void
vtgpu_respond(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    void *resp, size_t resp_len, struct iovec *wiov, int nwiov)
{
	size_t copy = resp_len;
	int i;

	for (i = 0; i < nwiov && copy > 0; i++) {
		size_t n = copy < wiov[i].iov_len ? copy : wiov[i].iov_len;
		memcpy(wiov[i].iov_base, resp, n);
		resp = (char *)resp + n;
		copy -= n;
	}

	if (hdr->flags & VIRTIO_GPU_FLAG_FENCE) {
		struct vtgpu_fence *vf = calloc(1, sizeof(*vf));
		if (vf == NULL)
			errx(1, "vtgpu: out of memory for fence");
		vf->vf_id       = hdr->fence_id;
		vf->vf_vq       = vq;
		vf->vf_idx      = chain_idx;
		vf->vf_resp_len = (uint32_t)resp_len;
		virgl_renderer_create_fence((int)hdr->fence_id, hdr->ctx_id);
		TAILQ_INSERT_TAIL(&sc->vsc_fences, vf, vf_link);
	} else {
		vq_relchain(vq, chain_idx, (uint32_t)resp_len);
		vq_endchains(vq, 0);
	}
}

static inline void
vtgpu_resp_nodata(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    uint32_t type, struct iovec *wiov, int nwiov)
{
	struct virtio_gpu_ctrl_hdr resp = {
		.type   = type,
		.flags  = hdr->flags & VIRTIO_GPU_FLAG_FENCE,
		.fence_id = hdr->fence_id,
		.ctx_id = hdr->ctx_id,
	};
	vtgpu_respond(sc, vq, hdr, chain_idx, &resp, sizeof(resp), wiov, nwiov);
}

/* ----------------------------------------------------------------------- */
/* Command handlers							   */
/* ----------------------------------------------------------------------- */

static void
vtgpu_cmd_get_display_info(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    struct iovec *wiov, int nwiov)
{
	struct virtio_gpu_resp_display_info resp = {};
	resp.hdr.type     = VIRTIO_GPU_RESP_OK_DISPLAY_INFO;
	resp.hdr.flags    = hdr->flags & VIRTIO_GPU_FLAG_FENCE;
	resp.hdr.fence_id = hdr->fence_id;
	resp.hdr.ctx_id   = hdr->ctx_id;
	resp.pmodes[0].r.x      = 0;
	resp.pmodes[0].r.y      = 0;
	resp.pmodes[0].r.width  = sc->vsc_width;
	resp.pmodes[0].r.height = sc->vsc_height;
	resp.pmodes[0].enabled  = 1;
	vtgpu_respond(sc, vq, hdr, chain_idx, &resp, sizeof(resp), wiov, nwiov);
}

static void
vtgpu_cmd_resource_create_2d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_create_2d *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virgl_renderer_resource_create_args args = {
		.handle         = cmd->resource_id,
		.target         = 2,        /* PIPE_TEXTURE_2D */
		.format         = cmd->format,
		.bind           = (1 << 1), /* PIPE_BIND_SAMPLER_VIEW */
		.width          = cmd->width,
		.height         = cmd->height,
		.depth          = 1,
		.array_size     = 1,
		.last_level     = 0,
		.nr_samples     = 0,
		.flags          = 0,
	};
	int ret = virgl_renderer_resource_create(&args, NULL, 0);
	DPRINTF("create_2d id=%u fmt=%u %ux%u ctx=%u ret=%d",
	    cmd->resource_id, cmd->format, cmd->width, cmd->height,
	    hdr->ctx_id, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

static void
vtgpu_cmd_resource_create_3d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_create_3d *cmd,
    struct iovec *wiov, int nwiov)
{
	uint32_t bind = cmd->bind;

	/*
	 * A resource the guest will scan out is one we may have to hand to a
	 * host compositor as a dma_buf, and the host allocates render targets
	 * compressed: on RDNA3 that means DCC, whose parameters live in a
	 * format modifier virglrenderer does not report -- it returns
	 * DRM_FORMAT_MOD_INVALID whatever the guest does.  An importer that
	 * cannot know the layout reads the compressed blocks as pixels, which
	 * is exactly the periodic corruption observed.
	 *
	 * Asking for it linear costs some render bandwidth and removes the
	 * guesswork entirely.  Only worth doing when a viewer is attached, so
	 * it is opt-in.
	 */
	if (sc->vsc_scanout_linear && (bind & VTGPU_BIND_SCANOUT) != 0)
		bind |= VTGPU_BIND_LINEAR;

	struct virgl_renderer_resource_create_args args = {
		.handle         = cmd->resource_id,
		.target         = cmd->target,
		.format         = cmd->format,
		.bind           = bind,
		.width          = cmd->width,
		.height         = cmd->height,
		.depth          = cmd->depth,
		.array_size     = cmd->array_size,
		.last_level     = cmd->last_level,
		.nr_samples     = cmd->nr_samples,
		.flags          = cmd->flags,
	};
	int ret = virgl_renderer_resource_create(&args, NULL, 0);
	/*
	 * Scanout-capable resources are rare -- a couple per mode set -- so
	 * report them unconditionally: whether the guest asks for SCANOUT at
	 * all decides whether forcing linear can work.
	 */
	if ((cmd->bind & VTGPU_BIND_SCANOUT) != 0) {
		unsigned i;

		for (i = 0; i < sc->vsc_scanout_ctx_n; i++)
			if (sc->vsc_scanout_ctx[i].res_id == cmd->resource_id)
				break;
		if (i == sc->vsc_scanout_ctx_n &&
		    i < nitems(sc->vsc_scanout_ctx))
			sc->vsc_scanout_ctx_n++;
		if (i < nitems(sc->vsc_scanout_ctx)) {
			sc->vsc_scanout_ctx[i].res_id = cmd->resource_id;
			sc->vsc_scanout_ctx[i].ctx_id = hdr->ctx_id;
		}
	}
	if ((cmd->bind & VTGPU_BIND_SCANOUT) != 0)
		EPRINTLN("vtgpu: create_3d id=%u SCANOUT bind=0x%x->0x%x "
		    "fmt=%u %ux%u ret=%d", cmd->resource_id, cmd->bind, bind,
		    cmd->format, cmd->width, cmd->height, ret);
	DPRINTF("create_3d id=%u tgt=%u fmt=%u bind=0x%x %ux%ux%u "
	    "array=%u levels=%u samples=%u ctx=%u ret=%d",
	    cmd->resource_id, cmd->target, cmd->format, cmd->bind,
	    cmd->width, cmd->height, cmd->depth, cmd->array_size,
	    cmd->last_level, cmd->nr_samples, hdr->ctx_id, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

/* Defined below, next to the rest of the blob-map helpers. */
static struct vtgpu_blob_map *vtgpu_blob_map_find(struct vtgpu_softc *sc,
    uint32_t res_id);

static void
vtgpu_cmd_resource_unref(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_unref *cmd,
    struct iovec *wiov, int nwiov)
{
	struct vtgpu_blob_map *bm;

	/*
	 * A blob resource may be unref'd while still mapped: the guest is not
	 * obliged to send RESOURCE_UNMAP_BLOB first, and a guest that exits or
	 * crashes never will.  Tear the mapping down here before dropping the
	 * reference.
	 *
	 * Order matters.  virgl_renderer_resource_unref() can free the storage
	 * the blob is backed by, so the guest alias installed by map_blob must
	 * be removed BEFORE the unref -- otherwise the guest is left with a
	 * window onto freed host memory that it can still read and write.
	 * Releasing the tracking slot also matters on its own: without it the
	 * fixed-size table fills up and every later map_blob fails.
	 */
	sc->vsc_unref++;
	bm = vtgpu_blob_map_find(sc, cmd->resource_id);
	if (bm != NULL) {
		EPRINTLN("vtgpu: unref res=%u releasing blob map gpa=0x%lx "
		    "len=%lu", cmd->resource_id, (unsigned long)bm->gpa,
		    (unsigned long)bm->len);
		vm_munmap_blob(sc->vsc_ctx, bm->gpa, bm->len);
		memset(bm, 0, sizeof(*bm));
		virgl_renderer_resource_unmap(cmd->resource_id);
	}
	virgl_renderer_resource_unref(cmd->resource_id);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

/*
 * Hand the scanout buffer to an external viewer, if one is configured and the
 * resource can be exported.  Zero-copy: the viewer imports the dma_buf into
 * the host compositor, so the pixels are never read back, encoded or copied.
 *
 * When the resource cannot be exported -- FreeBSD RADV has historically
 * handed back OPAQUE fds rather than dma_bufs -- nothing is published and the
 * viewer simply sees no scanout.  The shm fallback (read back into a shared
 * segment and publish that instead) is deliberately not written yet: whether
 * it is needed depends on what has_dmabuf_export actually reports on this
 * hardware, and writing it before knowing would be guessing.
 */
/*
 * A sync_file for the guest's rendering into this scanout resource, or -1.
 *
 * The guest does not fence SET_SCANOUT, so there is no completion signal to
 * ride on; make one on the context that created the resource, which is the
 * context that drew into it.  Only vrend and drm contexts register an
 * exportable fd (virgl_fence_set_fd() is called from nowhere in venus), which
 * is fine while the compositor renders through GL -- if it ever moves to the
 * Vulkan renderer this returns -1 and the viewer goes back to not waiting.
 */
static int
vtgpu_scanout_fence(struct vtgpu_softc *sc, uint32_t res_id)
{
	uint32_t ctx_id = 0, fid;
	int fd = -1, cret, eret;

	for (unsigned i = 0; i < sc->vsc_scanout_ctx_n; i++)
		if (sc->vsc_scanout_ctx[i].res_id == res_id) {
			ctx_id = sc->vsc_scanout_ctx[i].ctx_id;
			break;
		}

	if (sc->vsc_own_fence_next < VTGPU_OWN_FENCE_BASE)
		sc->vsc_own_fence_next = VTGPU_OWN_FENCE_BASE;
	fid = sc->vsc_own_fence_next++;

	cret = ctx_id != 0 ?
	    virgl_renderer_create_fence((int)fid, ctx_id) : -1;
	eret = cret == 0 ? virgl_renderer_export_fence(fid, &fd) : -1;

	/*
	 * Report the first few attempts.  Every step here can fail quietly --
	 * no context recorded for the resource, no fence created, nothing
	 * exportable -- and each failure looks identical from the outside: the
	 * viewer simply does not wait, exactly as before the fence existed.
	 */
	if (sc->vsc_fence_reports < 3) {
		sc->vsc_fence_reports++;
		EPRINTLN("vtgpu: scanout fence res=%u ctx=%u create=%d "
		    "export=%d fd=%d", res_id, ctx_id, cret, eret, fd);
	}
	return (eret == 0 ? fd : -1);
}

static void
vtgpu_scanout_publish(struct vtgpu_softc *sc,
    const struct virtio_gpu_set_scanout *cmd,
    const struct virgl_renderer_resource_info_ext *info)
{
	struct gpu_display_scanout so;
	int dfd = -1, stride = 0, offset = 0;

	if (sc->vsc_display == NULL)
		return;

	/*
	 * A DRM compositor page-flips: sway alternates between two resources,
	 * rebinding the scanout every frame.  Export each buffer once and
	 * thereafter just say which one is on screen -- re-exporting would
	 * hand over a fresh dma_buf fd sixty times a second to describe
	 * memory the viewer already has.
	 */
	if (gpu_display_have_buffer(sc->vsc_display, cmd->resource_id)) {
		/*
		 * Defer to the fence when the guest supplied one: it marks the
		 * point at which rendering into this buffer is complete.
		 * Publishing on command arrival instead lets the viewer sample
		 * a half-drawn frame, which is what made a fast renderer show
		 * two frames at once while a slow one looked fine.
		 */
		gpu_display_frame(sc->vsc_display, cmd->resource_id,
		    vtgpu_scanout_fence(sc, cmd->resource_id), cmd->r.x,
		    cmd->r.y, cmd->r.width, cmd->r.height);
		return;
	}

	if (!info->has_dmabuf_export)
		return;

	if (virgl_renderer_get_fd_for_texture2(info->base.tex_id, &dfd,
	    &stride, &offset) != 0 || dfd < 0) {
		EPRINTLN("vtgpu: scanout res=%u dmabuf export failed despite "
		    "has_dmabuf_export", cmd->resource_id);
		return;
	}

	memset(&so, 0, sizeof(so));
	so.buffer_id = cmd->resource_id;
	so.transport = GPU_DISPLAY_XPORT_DMABUF;
	so.width = cmd->r.width;
	so.height = cmd->r.height;
	so.stride = stride != 0 ? (uint32_t)stride : info->base.stride;
	so.drm_fourcc = (uint32_t)info->base.drm_fourcc;
	so.planes = (uint32_t)info->planes;
	/*
	 * The texture need not start at byte 0 of the dma_buf; dropping this
	 * makes every row read from the wrong place.
	 */
	so.offset = (uint32_t)offset;
	so.modifier = info->modifiers;
	EPRINTLN("vtgpu: scanout res=%u published fourcc=0x%08x stride=%u "
	    "offset=%d planes=%d modifier=0x%016jx", cmd->resource_id,
	    so.drm_fourcc, so.stride, offset, info->planes,
	    (uintmax_t)info->modifiers);
	gpu_display_scanout(sc->vsc_display, &so, dfd);	/* consumes dfd */
	gpu_display_frame(sc->vsc_display, cmd->resource_id,
	    vtgpu_scanout_fence(sc, cmd->resource_id), cmd->r.x, cmd->r.y,
	    cmd->r.width, cmd->r.height);
}

static void
vtgpu_cmd_set_scanout(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_set_scanout *cmd, struct iovec *wiov, int nwiov)
{
	/*
	 * Record what the guest bound so the rest of the device knows which
	 * resource is the display and how big it is.  Needed whenever a viewer
	 * is attached or the readback probe is running; resource_id 0 unbinds.
	 *
	 * A page-flipping compositor comes through here every frame, so the
	 * expensive part -- the virglrenderer query, the log line, sizing the
	 * readback buffer -- is done once per distinct buffer, not per flip.
	 */
	if ((sc->vsc_scanout_probe || sc->vsc_display != NULL) && cmd != NULL) {
		bool seen = false;

		/*
		 * Whether the guest fences these decides whether the publish
		 * can be deferred to completion at all.  Report the first few
		 * with their flags: if FENCE is never set, deferring is a
		 * no-op and the ordering has to come from somewhere else.
		 */
		if (sc->vsc_scanout_seen_total++ < 3)
			EPRINTLN("vtgpu: set_scanout #%u res=%u flags=0x%x "
			    "fenced=%s fence_id=%ju",
			    sc->vsc_scanout_seen_total, cmd->resource_id,
			    hdr->flags,
			    (hdr->flags & VIRTIO_GPU_FLAG_FENCE) ? "YES" : "no",
			    (uintmax_t)hdr->fence_id);

		for (unsigned i = 0; i < sc->vsc_seen_n; i++)
			if (sc->vsc_seen_scanout[i] == cmd->resource_id) {
				seen = true;
				break;
			}
		if (!seen && cmd->resource_id != 0 &&
		    sc->vsc_seen_n < nitems(sc->vsc_seen_scanout))
			sc->vsc_seen_scanout[sc->vsc_seen_n++] =
			    cmd->resource_id;

		sc->vsc_scanout_res = cmd->resource_id;
		sc->vsc_scanout_w = cmd->r.width;
		sc->vsc_scanout_h = cmd->r.height;
		if (cmd->resource_id != 0 && !seen) {
			size_t need = (size_t)cmd->r.width * cmd->r.height * 4;

			if (sc->vsc_scanout_probe &&
			    need > sc->vsc_scanout_bufsz) {
				free(sc->vsc_scanout_buf);
				sc->vsc_scanout_buf = malloc(need);
				sc->vsc_scanout_bufsz =
				    sc->vsc_scanout_buf ? need : 0;
			}
			struct virgl_renderer_resource_info_ext info;
			int iret;

			EPRINTLN("vtgpu: scanout %u bound to res=%u %ux%u "
			    "(%zu KiB readback)", cmd->scanout_id,
			    cmd->resource_id, cmd->r.width, cmd->r.height,
			    need / 1024);

			/*
			 * Can this resource be handed to the host compositor
			 * as a dma_buf instead of being read back?  If so the
			 * readback above is a cost a zero-copy present would
			 * never pay, and the timing below is measuring the
			 * wrong design.  FreeBSD RADV has historically had no
			 * dma_buf export -- it returns OPAQUE fds -- so ask
			 * rather than assume.
			 */
			memset(&info, 0, sizeof(info));
			info.version = VIRGL_RENDERER_RESOURCE_INFO_EXT_VERSION;
			iret = virgl_renderer_resource_get_info_ext(
			    (int)cmd->resource_id, &info);
			if (iret != 0) {
				EPRINTLN("vtgpu: scanout res=%u get_info_ext "
				    "failed ret=%d", cmd->resource_id, iret);
			} else {
				vtgpu_scanout_publish(sc, cmd, &info);
				EPRINTLN("vtgpu: scanout res=%u dmabuf_export=%s "
				    "fourcc=0x%08x stride=%u planes=%d "
				    "modifier=0x%016jx fmt=%u %ux%u",
				    cmd->resource_id,
				    info.has_dmabuf_export ? "YES" : "no",
				    (unsigned)info.base.drm_fourcc,
				    info.base.stride, info.planes,
				    (uintmax_t)info.modifiers,
				    info.base.virgl_format,
				    info.base.width, info.base.height);
			}
		}
	}
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_resource_flush(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_flush *cmd, struct iovec *wiov, int nwiov)
{
	/*
	 * Headless: nothing is presented.  Under scanout_probe, read the
	 * scanout resource back into a host buffer and time it -- that read
	 * is the per-frame cost a host-side present would pay, and it is the
	 * one number worth having before building a display path on it.
	 */
	if (sc->vsc_scanout_probe && cmd != NULL &&
	    cmd->resource_id != 0 &&
	    cmd->resource_id == sc->vsc_scanout_res &&
	    sc->vsc_scanout_buf != NULL) {
		struct virgl_box box = {
			.x = 0, .y = 0, .z = 0,
			.w = sc->vsc_scanout_w, .h = sc->vsc_scanout_h, .d = 1,
		};
		struct iovec iov = {
			.iov_base = sc->vsc_scanout_buf,
			.iov_len = (size_t)sc->vsc_scanout_w *
			    sc->vsc_scanout_h * 4,
		};
		struct timespec t0, t1;
		uint64_t ns;
		int ret;

		clock_gettime(CLOCK_MONOTONIC, &t0);
		ret = virgl_renderer_transfer_read_iov(cmd->resource_id, 0, 0,
		    sc->vsc_scanout_w * 4, 0, &box, 0, &iov, 1);
		clock_gettime(CLOCK_MONOTONIC, &t1);

		if (ret != 0) {
			if (sc->vsc_ro_fail++ == 0)
				EPRINTLN("vtgpu: scanout readback res=%u "
				    "FAILED ret=%d (reported once)",
				    cmd->resource_id, ret);
		} else {
			ns = (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000000ULL +
			    (t1.tv_nsec - t0.tv_nsec);
			if (sc->vsc_ro_n == 0 || ns < sc->vsc_ro_min)
				sc->vsc_ro_min = ns;
			if (ns > sc->vsc_ro_max)
				sc->vsc_ro_max = ns;
			sc->vsc_ro_ns += ns;
			if ((++sc->vsc_ro_n % 60) == 0)
				EPRINTLN("vtgpu: scanout readback %ux%u n=%ju "
				    "avg=%juus min=%juus max=%juus "
				    "(avg caps ~%ju fps)",
				    sc->vsc_scanout_w, sc->vsc_scanout_h,
				    (uintmax_t)sc->vsc_ro_n,
				    (uintmax_t)(sc->vsc_ro_ns /
					sc->vsc_ro_n / 1000),
				    (uintmax_t)(sc->vsc_ro_min / 1000),
				    (uintmax_t)(sc->vsc_ro_max / 1000),
				    (uintmax_t)(sc->vsc_ro_ns ?
					1000000000ULL /
					(sc->vsc_ro_ns / sc->vsc_ro_n) : 0));
		}
	}
	/*
	 * Still forwarded for guests that draw into one resource and flush it
	 * (the 2D/fbdev path), but a page-flipping compositor never gets here
	 * -- for those the flip is SET_SCANOUT and the frame is reported from
	 * vtgpu_scanout_publish() instead.
	 */
	if (sc->vsc_display != NULL && cmd != NULL &&
	    cmd->resource_id == sc->vsc_scanout_res)
		gpu_display_frame(sc->vsc_display, cmd->resource_id,
		    vtgpu_scanout_fence(sc, cmd->resource_id), cmd->r.x,
		    cmd->r.y, cmd->r.width, cmd->r.height);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_transfer_to_host_2d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_transfer_to_host_2d *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virgl_box box = {
		.x = cmd->r.x,  .y = cmd->r.y,  .z = 0,
		.w = cmd->r.width, .h = cmd->r.height, .d = 1,
	};
	int ret = virgl_renderer_transfer_write_iov(cmd->resource_id, 0,
	    0, 0, 0, &box, cmd->offset, NULL, 0);
	DPRINTF("xfer_2d id=%u box=%u,%u %ux%u off=%lu ret=%d",
	    cmd->resource_id, cmd->r.x, cmd->r.y, cmd->r.width, cmd->r.height,
	    (unsigned long)cmd->offset, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

static void
vtgpu_cmd_resource_attach_backing(struct vtgpu_softc *sc,
    struct vqueue_info *vq, const struct virtio_gpu_ctrl_hdr *hdr,
    uint16_t chain_idx,
    const struct virtio_gpu_resource_attach_backing *cmd,
    const struct virtio_gpu_mem_entry *entries, uint32_t max_entries,
    struct iovec *wiov, int nwiov)
{
	uint32_t n = cmd->nr_entries;
	struct iovec *iovs;
	uint32_t i;

	/*
	 * Validate nr_entries against what the command buffer actually
	 * contains before indexing entries[]; the guest controls the count.
	 * Rejections are logged unconditionally: silently dropping the
	 * backing leaves the resource dataless and every later use fails with
	 * virglrenderer "illegal resource", which is very hard to trace back
	 * to here.
	 */
	if (n > max_entries || n > VTGPU_MAX_BACKING) {
		EPRINTLN("vtgpu: attach_backing id=%u REJECTED nr_entries=%u "
		    "(buffer holds %u, ceiling %u)", cmd->resource_id, n,
		    max_entries, VTGPU_MAX_BACKING);
		vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
		    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
		return;
	}
	iovs = calloc(n, sizeof(*iovs));
	if (iovs == NULL) {
		EPRINTLN("vtgpu: attach_backing id=%u FAILED - out of memory "
		    "for %u iovecs", cmd->resource_id, n);
		vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
		    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY, wiov, nwiov);
		return;
	}
	for (i = 0; i < n; i++) {
		iovs[i].iov_base = paddr_guest2host(sc->vsc_ctx,
		    entries[i].addr, entries[i].length);
		iovs[i].iov_len  = entries[i].length;
		if (iovs[i].iov_base == NULL) {
			EPRINTLN("vtgpu: attach_backing id=%u FAILED - entry %u "
			    "gpa=0x%lx len=%u is not guest memory",
			    cmd->resource_id, i,
			    (unsigned long)entries[i].addr, entries[i].length);
			free(iovs);
			vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
			    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
			return;
		}
	}
	virgl_renderer_resource_attach_iov(cmd->resource_id, iovs, (int)n);
	DPRINTF("attach_backing id=%u nr_entries=%u n=%u gpa=0x%lx len=%u",
	    cmd->resource_id, cmd->nr_entries, n,
	    (unsigned long)entries[0].addr, entries[0].length);
	/* virglrenderer takes ownership of iovs; do not free here. */
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_resource_detach_backing(struct vtgpu_softc *sc,
    struct vqueue_info *vq, const struct virtio_gpu_ctrl_hdr *hdr,
    uint16_t chain_idx, const struct virtio_gpu_resource_detach_backing *cmd,
    struct iovec *wiov, int nwiov)
{
	struct iovec *iovs = NULL;
	int niov = 0;
	virgl_renderer_resource_detach_iov(cmd->resource_id, &iovs, &niov);
	free(iovs);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

/*
 * RESOURCE_CREATE_BLOB.  Used by venus: the command ring and other shared
 * buffers are guest-memory blobs (BLOB_MEM_GUEST / HOST3D_GUEST) whose pages
 * the guest supplies as mem entries — we translate them to host iovecs and
 * hand them to virglrenderer, which (with USE_EXTERNAL_BLOB) shares them with
 * the render server.  Purely host-allocated blobs (HOST3D, nr_entries == 0)
 * would need the host-visible PCI window, which is not implemented yet.
 */
static void
vtgpu_cmd_resource_create_blob(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_create_blob *cmd,
    const struct virtio_gpu_mem_entry *entries, uint32_t max_entries,
    struct iovec *wiov, int nwiov)
{
	struct virgl_renderer_resource_create_blob_args args;
	struct iovec *iovs = NULL;
	uint32_t n = cmd->nr_entries;
	uint32_t i;
	int ret;

	if (n > max_entries || n > VTGPU_MAX_BACKING) {
		EPRINTLN("vtgpu: create_blob id=%u REJECTED nr_entries=%u "
		    "(buffer holds %u, ceiling %u)", cmd->resource_id, n,
		    max_entries, VTGPU_MAX_BACKING);
		vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
		    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
		return;
	}
	if (n > 0) {
		iovs = calloc(n, sizeof(*iovs));
		if (iovs == NULL) {
			EPRINTLN("vtgpu: create_blob id=%u FAILED - out of "
			    "memory for %u iovecs", cmd->resource_id, n);
			vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
			    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY, wiov, nwiov);
			return;
		}
		for (i = 0; i < n; i++) {
			iovs[i].iov_base = paddr_guest2host(sc->vsc_ctx,
			    entries[i].addr, entries[i].length);
			iovs[i].iov_len  = entries[i].length;
			if (iovs[i].iov_base == NULL) {
				EPRINTLN("vtgpu: create_blob id=%u FAILED - "
				    "entry %u gpa=0x%lx len=%u is not guest "
				    "memory", cmd->resource_id, i,
				    (unsigned long)entries[i].addr,
				    entries[i].length);
				free(iovs);
				vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
				    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
				return;
			}
		}
	}

	memset(&args, 0, sizeof(args));
	args.res_handle = cmd->resource_id;
	args.ctx_id     = hdr->ctx_id;
	args.blob_mem   = cmd->blob_mem;
	args.blob_flags = cmd->blob_flags;
	args.blob_id    = cmd->blob_id;
	args.size       = cmd->size;
	args.iovecs     = iovs;
	args.num_iovs   = n;

	ret = virgl_renderer_resource_create_blob(&args);
	DPRINTF("create_blob id=%u mem=%u flags=0x%x blob_id=%lu size=%lu "
	    "nr=%u ctx=%u ret=%d", cmd->resource_id, cmd->blob_mem,
	    cmd->blob_flags, (unsigned long)cmd->blob_id,
	    (unsigned long)cmd->size, n, hdr->ctx_id, ret);
	if (ret != 0)
		EPRINTLN("vtgpu: create_blob id=%u FAILED ret=%d (mem=%u "
		    "flags=0x%x blob_id=%lu size=%lu nr=%u ctx=%u)",
		    cmd->resource_id, ret, cmd->blob_mem, cmd->blob_flags,
		    (unsigned long)cmd->blob_id, (unsigned long)cmd->size, n,
		    hdr->ctx_id);
	/*
	 * Host-visible blobs (BLOB_MEM_HOST3D == 2) are few and expensive --
	 * CK3 asks for them 256MB at a time -- so log the successful ones too,
	 * unconditionally.  venus sets its per-VkDeviceMemory "exported" flag
	 * before the export completes and never clears it, so a partial
	 * failure poisons that memory permanently and every later attempt
	 * reports "mem has been exported".  Without a record of the export
	 * that succeeded, those later failures cannot be traced back to the
	 * one that actually broke, and the success path is otherwise visible
	 * only under debug=on, which costs 53% and changes the timing.
	 */
	if (ret == 0 && cmd->blob_mem == 2) {
		EPRINTLN("vtgpu: create_blob id=%u OK blob_id=%lu size=%lu "
		    "flags=0x%x ctx=%u", cmd->resource_id,
		    (unsigned long)cmd->blob_id, (unsigned long)cmd->size,
		    cmd->blob_flags, hdr->ctx_id);
		/*
		 * Whether these are ever reclaimed cannot be read off the
		 * unref lines above: those only fire for a blob that still
		 * had a mapping, so one unmapped first, or never mapped at
		 * all, is released silently.  Report the totals instead --
		 * if unref stays near zero while created climbs, resources
		 * really are accumulating.
		 */
		if ((++sc->vsc_blob_hv & 0x3f) == 0)
			EPRINTLN("vtgpu: host-visible blobs created=%ju "
			    "unref-cmds=%ju", (uintmax_t)sc->vsc_blob_hv,
			    (uintmax_t)sc->vsc_unref);
	}
	/*
	 * Attaching here is a leftover from the removed mvisor mode, whose
	 * guest driver never sent CTX_ATTACH_RESOURCE.  It is harmless -- for
	 * a context-created blob proxy_context_attach_resource finds the
	 * resource already present and returns -- but the standard Linux guest
	 * sends the attach itself, so keep it only until that is confirmed
	 * across all guest drivers we care about.
	 */
	if (ret == 0 && hdr->ctx_id != 0)
		virgl_renderer_ctx_attach_resource(hdr->ctx_id,
		    (int)cmd->resource_id);
	/* virglrenderer copies the (const) iovec array; free our copy. */
	free(iovs);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    ret ? VIRTIO_GPU_RESP_ERR_UNSPEC : VIRTIO_GPU_RESP_OK_NODATA,
	    wiov, nwiov);
}

#define	VTGPU_PAGE_ROUND(x)	(((x) + PAGE_SIZE - 1) & ~((uint64_t)PAGE_SIZE - 1))

static struct vtgpu_blob_map *
vtgpu_blob_map_find(struct vtgpu_softc *sc, uint32_t res_id)
{
	int i;

	for (i = 0; i < VTGPU_MAX_BLOB_MAPS; i++)
		if (sc->vsc_blob_maps[i].used &&
		    sc->vsc_blob_maps[i].res_id == res_id)
			return (&sc->vsc_blob_maps[i]);
	return (NULL);
}

/*
 * RESOURCE_MAP_BLOB.  A host-visible venus blob (host-allocated VkDeviceMemory
 * or the coherent shm ring) is placed into the guest's host-visible BAR window
 * at cmd->offset so the guest can mmap it.  We export the blob's fd from
 * virglrenderer, mmap it in bhyve to obtain a host VA, and alias that VA into
 * the guest window via VM_MMAP_BLOB — zero-copy and coherent with the render
 * server.  The bhyve mapping and fd are retained until UNMAP_BLOB.
 */
static void
vtgpu_cmd_resource_map_blob(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_map_blob *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virtio_gpu_resp_map_info resp = {};
	struct vtgpu_blob_map *bm;
	uint32_t map_info = 0;
	uint64_t gpa, len, map_size = 0;
	void *hva = NULL;
	int i, ret;

	if (sc->vsc_hostvis_gpa == 0) {
		EPRINTLN("vtgpu: map_blob res=%u REJECTED - host-visible BAR "
		    "not mapped by the guest", cmd->resource_id);
		goto err;
	}
	/* Find a free tracking slot. */
	bm = NULL;
	for (i = 0; i < VTGPU_MAX_BLOB_MAPS; i++)
		if (!sc->vsc_blob_maps[i].used) {
			bm = &sc->vsc_blob_maps[i];
			break;
		}
	if (bm == NULL) {
		EPRINTLN("vtgpu: map_blob res=%u REJECTED - all %d blob-map "
		    "slots in use (leak, or raise VTGPU_MAX_BLOB_MAPS)",
		    cmd->resource_id, VTGPU_MAX_BLOB_MAPS);
		goto err;
	}

	/*
	 * Let virglrenderer map the blob into our address space: it uses the
	 * resource's true size (res->map_size, not fstat -- device-memory blob
	 * fds are not regular files) and picks the right method for shm, dmabuf
	 * or opaque device memory.  We then alias the returned host VA into the
	 * guest window; virglrenderer owns the mapping until resource_unmap.
	 */
	ret = virgl_renderer_resource_map(cmd->resource_id, &hva, &map_size);
	if (ret != 0 || hva == NULL) {
		EPRINTLN("vtgpu: map_blob res=%u FAILED resource_map ret=%d "
		    "hva=%p", cmd->resource_id, ret, hva);
		goto err;
	}
	/*
	 * Diagnostic: report the resource's fd type and the mapped VA/size.
	 * "no entry" from vm_mmap_blob means this hva is not a real mapping in
	 * our address space -- distinguishes a DMABUF we failed to mmap from an
	 * OPAQUE handle whose resource_map returns a render-server-side pointer.
	 */
	{
		uint32_t dbg_fd_type = 0;
		int dbg_fd = -1;
		int er = virgl_renderer_resource_export_blob(cmd->resource_id,
		    &dbg_fd_type, &dbg_fd);
		DPRINTF("map_blob res=%u DIAG hva=%p map_size=%lu | "
		    "export_blob ret=%d fd_type=%u fd=%d",
		    cmd->resource_id, hva, (unsigned long)map_size,
		    er, dbg_fd_type, dbg_fd);
		if (dbg_fd >= 0)
			close(dbg_fd);
	}
	len = VTGPU_PAGE_ROUND(map_size);
	if (len == 0 || cmd->offset + len < cmd->offset ||
	    cmd->offset + len > VTGPU_HOSTVIS_SZ) {
		DPRINTF("map_blob res=%u bad off=0x%lx size=0x%lx (win=0x%lx)",
		    cmd->resource_id, (unsigned long)cmd->offset,
		    (unsigned long)map_size, (unsigned long)VTGPU_HOSTVIS_SZ);
		virgl_renderer_resource_unmap(cmd->resource_id);
		goto err;
	}
	gpa = sc->vsc_hostvis_gpa + cmd->offset;

	if (vm_mmap_blob(sc->vsc_ctx, gpa, hva, len,
	    PROT_READ | PROT_WRITE) != 0) {
		DPRINTF("map_blob res=%u vm_mmap_blob(gpa=0x%lx len=0x%lx) "
		    "failed: %s", cmd->resource_id, (unsigned long)gpa,
		    (unsigned long)len, strerror(errno));
		virgl_renderer_resource_unmap(cmd->resource_id);
		goto err;
	}

	bm->res_id = cmd->resource_id;
	bm->gpa    = gpa;
	bm->len    = len;
	bm->used   = true;

	virgl_renderer_resource_get_map_info(cmd->resource_id, &map_info);
	DPRINTF("map_blob res=%u off=0x%lx gpa=0x%lx len=0x%lx map_info=%u ok",
	    cmd->resource_id, (unsigned long)cmd->offset, (unsigned long)gpa,
	    (unsigned long)len, map_info);

	resp.hdr.type     = VIRTIO_GPU_RESP_OK_MAP_INFO;
	resp.hdr.flags    = hdr->flags & VIRTIO_GPU_FLAG_FENCE;
	resp.hdr.fence_id = hdr->fence_id;
	resp.hdr.ctx_id   = hdr->ctx_id;
	resp.map_info     = map_info;
	vtgpu_respond(sc, vq, hdr, chain_idx, &resp, sizeof(resp), wiov, nwiov);
	return;

err:
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
}

/*
 * RESOURCE_UNMAP_BLOB.  Reverse of map_blob: drop the guest alias, then let
 * virglrenderer release its host mapping.  The command carries only the
 * resource id, so we look up the guest range we recorded at map time.
 */
static void
vtgpu_cmd_resource_unmap_blob(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_resource_unmap_blob *cmd,
    struct iovec *wiov, int nwiov)
{
	struct vtgpu_blob_map *bm;

	bm = vtgpu_blob_map_find(sc, cmd->resource_id);
	if (bm != NULL) {
		vm_munmap_blob(sc->vsc_ctx, bm->gpa, bm->len);
		memset(bm, 0, sizeof(*bm));
	}
	virgl_renderer_resource_unmap(cmd->resource_id);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_get_capset_info(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_get_capset_info *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virtio_gpu_resp_capset_info resp = {};
	resp.hdr.type     = VIRTIO_GPU_RESP_OK_CAPSET_INFO;
	resp.hdr.flags    = hdr->flags & VIRTIO_GPU_FLAG_FENCE;
	resp.hdr.fence_id = hdr->fence_id;
	resp.hdr.ctx_id   = hdr->ctx_id;

	/*
	 * We expose index 0 = VIRGL, index 1 = VIRGL2, and — when venus is
	 * enabled — index 2 = VENUS.  The guest probes these by capset_index,
	 * not by ID.
	 */
	uint32_t capset_id;
	switch (cmd->capset_index) {
	case 0:
		capset_id = VIRTIO_GPU_CAPSET_VIRGL;
		break;
	case 2:
		if (sc->vsc_venus) {
			capset_id = VIRTIO_GPU_CAPSET_VENUS;
			break;
		}
		/* FALLTHROUGH */
	case 1:
	default:
		capset_id = VIRTIO_GPU_CAPSET_VIRGL2;
		break;
	}
	uint32_t max_ver = 0, max_size = 0;
	virgl_renderer_get_cap_set(capset_id, &max_ver, &max_size);
	DPRINTF("capset_info idx=%u -> id=%u max_ver=%u max_size=%u",
	    cmd->capset_index, capset_id, max_ver, max_size);

	resp.capset_id          = capset_id;
	resp.capset_max_version = max_ver;
	resp.capset_max_size    = max_size;
	vtgpu_respond(sc, vq, hdr, chain_idx, &resp, sizeof(resp), wiov, nwiov);
}

static void
vtgpu_cmd_get_capset(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_get_capset *cmd,
    struct iovec *wiov, int nwiov)
{
	uint32_t max_ver = 0, max_size = 0;
	virgl_renderer_get_cap_set(cmd->capset_id, &max_ver, &max_size);
	DPRINTF("get_capset id=%u ver=%u -> max_ver=%u max_size=%u",
	    cmd->capset_id, cmd->capset_version, max_ver, max_size);

	if (max_size == 0) {
		EPRINTLN("vtgpu: get_capset id=%u ver=%u REJECTED - unknown "
		    "capset", cmd->capset_id, cmd->capset_version);
		vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
		    VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, wiov, nwiov);
		return;
	}

	size_t total = sizeof(struct virtio_gpu_resp_capset) + max_size;
	struct virtio_gpu_resp_capset *resp = calloc(1, total);
	if (resp == NULL) {
		EPRINTLN("vtgpu: get_capset id=%u FAILED - out of memory for "
		    "%zu bytes", cmd->capset_id, total);
		vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
		    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY, wiov, nwiov);
		return;
	}
	resp->hdr.type     = VIRTIO_GPU_RESP_OK_CAPSET;
	resp->hdr.flags    = hdr->flags & VIRTIO_GPU_FLAG_FENCE;
	resp->hdr.fence_id = hdr->fence_id;
	resp->hdr.ctx_id   = hdr->ctx_id;
	virgl_renderer_fill_caps(cmd->capset_id, cmd->capset_version,
	    resp->capset_data);
	vtgpu_respond(sc, vq, hdr, chain_idx, resp, total, wiov, nwiov);
	free(resp);
}

static void
vtgpu_cmd_ctx_create(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_ctx_create *cmd,
    struct iovec *wiov, int nwiov)
{
	int ret;

	if (cmd->context_init != 0) {
		/*
		 * Context-init: the low byte of context_init selects the
		 * capset (VIRGL / VIRGL2 / VENUS).  virglrenderer's context
		 * flags use the same capset-id encoding in their low byte, so
		 * pass context_init straight through.  This is the path a
		 * Venus (Vulkan) guest takes to create a vkr context.
		 */
		ret = virgl_renderer_context_create_with_flags(hdr->ctx_id,
		    cmd->context_init, cmd->nlen, cmd->debug_name);
		DPRINTF("ctx_create id=%u context_init=0x%x (capset=%u) "
		    "nlen=%u ret=%d", hdr->ctx_id, cmd->context_init,
		    cmd->context_init & 0xff, cmd->nlen, ret);
	} else {
		ret = virgl_renderer_context_create(hdr->ctx_id,
		    cmd->nlen, cmd->debug_name);
		DPRINTF("ctx_create id=%u (legacy, no context_init) ret=%d",
		    hdr->ctx_id, ret);
	}
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

static void
vtgpu_cmd_ctx_destroy(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    struct iovec *wiov, int nwiov)
{
	virgl_renderer_context_destroy(hdr->ctx_id);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_ctx_attach_resource(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_ctx_resource *cmd,
    struct iovec *wiov, int nwiov)
{
	DPRINTF("ctx_attach_res ctx=%u res=%u", hdr->ctx_id, cmd->resource_id);
	virgl_renderer_ctx_attach_resource(hdr->ctx_id, (int)cmd->resource_id);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_ctx_detach_resource(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_ctx_resource *cmd,
    struct iovec *wiov, int nwiov)
{
	DPRINTF("ctx_detach_res ctx=%u res=%u", hdr->ctx_id, cmd->resource_id);
	virgl_renderer_ctx_detach_resource(hdr->ctx_id, (int)cmd->resource_id);
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx,
	    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
}

static void
vtgpu_cmd_transfer_to_host_3d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_transfer_host_3d *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virgl_box box = {
		.x = cmd->box.x, .y = cmd->box.y, .z = cmd->box.z,
		.w = cmd->box.w, .h = cmd->box.h, .d = cmd->box.d,
	};
	int ret = virgl_renderer_transfer_write_iov(cmd->resource_id,
	    hdr->ctx_id, (int)cmd->level, cmd->stride, cmd->layer_stride,
	    &box, cmd->offset, NULL, 0);
	DPRINTF("xfer_to_3d id=%u ctx=%u lvl=%u stride=%u lstride=%u "
	    "box=%u,%u,%u %ux%ux%u off=%lu ret=%d",
	    cmd->resource_id, hdr->ctx_id, cmd->level, cmd->stride,
	    cmd->layer_stride, cmd->box.x, cmd->box.y, cmd->box.z,
	    cmd->box.w, cmd->box.h, cmd->box.d,
	    (unsigned long)cmd->offset, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

static void
vtgpu_cmd_transfer_from_host_3d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_transfer_host_3d *cmd,
    struct iovec *wiov, int nwiov)
{
	struct virgl_box box = {
		.x = cmd->box.x, .y = cmd->box.y, .z = cmd->box.z,
		.w = cmd->box.w, .h = cmd->box.h, .d = cmd->box.d,
	};
	int ret = virgl_renderer_transfer_read_iov(cmd->resource_id,
	    hdr->ctx_id, (int)cmd->level, cmd->stride, cmd->layer_stride,
	    &box, cmd->offset, NULL, 0);
	DPRINTF("xfer_from_3d id=%u ctx=%u lvl=%u stride=%u lstride=%u "
	    "box=%u,%u,%u %ux%ux%u off=%lu ret=%d",
	    cmd->resource_id, hdr->ctx_id, cmd->level, cmd->stride,
	    cmd->layer_stride, cmd->box.x, cmd->box.y, cmd->box.z,
	    cmd->box.w, cmd->box.h, cmd->box.d,
	    (unsigned long)cmd->offset, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

static void
vtgpu_cmd_submit_3d(struct vtgpu_softc *sc, struct vqueue_info *vq,
    const struct virtio_gpu_ctrl_hdr *hdr, uint16_t chain_idx,
    const struct virtio_gpu_cmd_submit *cmd,
    const void *buf, struct iovec *wiov, int nwiov)
{
	int ret = virgl_renderer_submit_cmd((void *)(uintptr_t)buf,
	    (int)hdr->ctx_id, cmd->size / 4);
	DPRINTF("submit_3d ctx=%u size=%uB (%u dwords) ret=%d",
	    hdr->ctx_id, cmd->size, cmd->size / 4, ret);
	uint32_t type = ret ? VIRTIO_GPU_RESP_ERR_UNSPEC
	                    : VIRTIO_GPU_RESP_OK_NODATA;
	vtgpu_resp_nodata(sc, vq, hdr, chain_idx, type, wiov, nwiov);
}

/* ----------------------------------------------------------------------- */
/* Control queue processing						   */
/* ----------------------------------------------------------------------- */

/*
 * Linearise a chain of readable iovecs into a freshly malloc'd buffer.
 * Returns the buffer and its length via *out / *out_len.  Caller frees.
 */
static int
vtgpu_flatten_riov(struct iovec *riov, int nriov, void **out, size_t *out_len)
{
	size_t total = 0;
	int i;
	char *buf, *p;

	for (i = 0; i < nriov; i++)
		total += riov[i].iov_len;
	buf = malloc(total);
	if (buf == NULL)
		return (ENOMEM);
	p = buf;
	for (i = 0; i < nriov; i++) {
		memcpy(p, riov[i].iov_base, riov[i].iov_len);
		p += riov[i].iov_len;
	}
	*out = buf;
	*out_len = total;
	return (0);
}

#define	VTGPU_MAXIOV	(VTGPU_RINGSZ * 2)

static void
vtgpu_process_controlq(struct vtgpu_softc *sc, int qidx)
{
	struct vqueue_info *vq = &sc->vsc_queues[qidx];
	struct iovec iov[VTGPU_MAXIOV];
	struct vi_req req;
	int n;

	while (vq_has_descs(vq)) {
		n = vq_getchain(vq, iov, VTGPU_MAXIOV, &req);
		if (n < 0)
			errx(1, "vtgpu: vq_getchain error");

		/*
		 * Readable descriptors precede writable ones.  At minimum we
		 * need the command header in the readable region.
		 */
		if (req.readable == 0 || req.readable > n) {
			vq_relchain(vq, req.idx, 0);
			vq_endchains(vq, 0);
			continue;
		}

		/* Flatten the readable side into a contiguous buffer. */
		void *cmdbuf;
		size_t cmdlen;
		if (vtgpu_flatten_riov(iov, req.readable, &cmdbuf, &cmdlen)) {
			vq_relchain(vq, req.idx, 0);
			vq_endchains(vq, 0);
			continue;
		}

		struct iovec *wiov = &iov[req.readable];
		int nwiov = n - req.readable;

		const struct virtio_gpu_ctrl_hdr *hdr = cmdbuf;

		DPRINTF("cmd type=0x%x ctx=%u fence_id=%lu flags=0x%x",
		    hdr->type, hdr->ctx_id, (unsigned long)hdr->fence_id,
		    hdr->flags);

		switch (hdr->type) {
		case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
			vtgpu_cmd_get_display_info(sc, vq, hdr, req.idx,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
			vtgpu_cmd_resource_create_2d(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_create_2d *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_CREATE_3D:
			vtgpu_cmd_resource_create_3d(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_create_3d *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_UNREF:
			vtgpu_cmd_resource_unref(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_unref *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_SET_SCANOUT:
		case VIRTIO_GPU_CMD_SET_SCANOUT_BLOB:
			vtgpu_cmd_set_scanout(sc, vq, hdr, req.idx,
			    cmdlen >= sizeof(struct virtio_gpu_set_scanout) ?
			    (const struct virtio_gpu_set_scanout *)hdr : NULL,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
			vtgpu_cmd_resource_flush(sc, vq, hdr, req.idx,
			    cmdlen >= sizeof(struct virtio_gpu_resource_flush) ?
			    (const struct virtio_gpu_resource_flush *)hdr :
			    NULL, wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
			vtgpu_cmd_transfer_to_host_2d(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_transfer_to_host_2d *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING: {
			const struct virtio_gpu_resource_attach_backing *ab =
			    (const void *)hdr;
			const struct virtio_gpu_mem_entry *ents =
			    (const void *)(ab + 1);
			uint32_t maxe = cmdlen > sizeof(*ab) ?
			    (cmdlen - sizeof(*ab)) / sizeof(*ents) : 0;
			vtgpu_cmd_resource_attach_backing(sc, vq, hdr, req.idx,
			    ab, ents, maxe, wiov, nwiov);
			break;
		}

		case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING:
			vtgpu_cmd_resource_detach_backing(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_detach_backing *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB: {
			const struct virtio_gpu_resource_create_blob *cb =
			    (const void *)hdr;
			const struct virtio_gpu_mem_entry *ents =
			    (const void *)(cb + 1);
			uint32_t maxe = cmdlen > sizeof(*cb) ?
			    (cmdlen - sizeof(*cb)) / sizeof(*ents) : 0;
			vtgpu_cmd_resource_create_blob(sc, vq, hdr, req.idx,
			    cb, ents, maxe, wiov, nwiov);
			break;
		}

		case VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB:
			vtgpu_cmd_resource_map_blob(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_map_blob *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB:
			vtgpu_cmd_resource_unmap_blob(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_resource_unmap_blob *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_GET_CAPSET_INFO:
			vtgpu_cmd_get_capset_info(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_get_capset_info *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_GET_CAPSET:
			vtgpu_cmd_get_capset(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_get_capset *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_CTX_CREATE:
			vtgpu_cmd_ctx_create(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_ctx_create *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_CTX_DESTROY:
			vtgpu_cmd_ctx_destroy(sc, vq, hdr, req.idx,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE:
			vtgpu_cmd_ctx_attach_resource(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_ctx_resource *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE:
			vtgpu_cmd_ctx_detach_resource(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_ctx_resource *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D:
			vtgpu_cmd_transfer_to_host_3d(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_transfer_host_3d *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D:
			vtgpu_cmd_transfer_from_host_3d(sc, vq, hdr, req.idx,
			    (const struct virtio_gpu_transfer_host_3d *)hdr,
			    wiov, nwiov);
			break;

		case VIRTIO_GPU_CMD_SUBMIT_3D: {
			const struct virtio_gpu_cmd_submit *sub =
			    (const void *)hdr;
			const void *payload = sub + 1;
			vtgpu_cmd_submit_3d(sc, vq, hdr, req.idx,
			    sub, payload, wiov, nwiov);
			break;
		}

		case VIRTIO_GPU_CMD_UPDATE_CURSOR:
		case VIRTIO_GPU_CMD_MOVE_CURSOR:
			/* Software cursor; ignore. */
			vtgpu_resp_nodata(sc, vq, hdr, req.idx,
			    VIRTIO_GPU_RESP_OK_NODATA, wiov, nwiov);
			break;

		default:
			DPRINTF("unknown cmd 0x%x", hdr->type);
			vtgpu_resp_nodata(sc, vq, hdr, req.idx,
			    VIRTIO_GPU_RESP_ERR_UNSPEC, wiov, nwiov);
			break;
		}

		free(cmdbuf);

		/* Drain any fences that have completed. */
		virgl_renderer_poll();
	}
}

static void
vtgpu_process_cursorq(struct vtgpu_softc *sc)
{
	struct vqueue_info *vq = &sc->vsc_queues[VTGPU_CURSORQ];
	struct iovec iov[VTGPU_MAXIOV];
	struct vi_req req;
	int n;

	while (vq_has_descs(vq)) {
		n = vq_getchain(vq, iov, VTGPU_MAXIOV, &req);
		if (n < 0)
			errx(1, "vtgpu: cursorq vq_getchain error");
		/* Just consume and acknowledge. */
		vq_relchain(vq, req.idx, 0);
		vq_endchains(vq, 0);
	}
}

/* ----------------------------------------------------------------------- */
/* Worker thread							   */
/* ----------------------------------------------------------------------- */

static int vtgpu_virgl_init(struct vtgpu_softc *sc);

/*
 * Build the kqueue the worker waits on.  Any failure here is non-fatal:
 * vsc_kq stays -1 and the worker uses the condvar path, which is correct
 * but wakes on a fixed timeout while fences are outstanding.  Notably
 * this is the expected outcome if Capsicum denies CAP_EVENT on
 * virglrenderer's fd, so it must degrade rather than abort.
 */
static void
vtgpu_kq_setup(struct vtgpu_softc *sc)
{
	struct kevent kev[2];
	int n = 0;

	sc->vsc_poll_fd = virgl_renderer_get_poll_fd();
	sc->vsc_kq = kqueue();
	if (sc->vsc_kq < 0) {
		EPRINTLN("vtgpu: kqueue() failed (%s), using timed wait",
		    strerror(errno));
		return;
	}

	/*
	 * EV_CLEAR on both: the user event must re-arm per trigger rather
	 * than stay permanently ready, and the fence fd must not spin us if
	 * virgl_renderer_poll() leaves it readable.
	 */
	EV_SET(&kev[n++], VTGPU_KQ_NOTIFY, EVFILT_USER, EV_ADD | EV_CLEAR,
	    0, 0, NULL);
	if (sc->vsc_poll_fd >= 0)
		EV_SET(&kev[n++], sc->vsc_poll_fd, EVFILT_READ,
		    EV_ADD | EV_CLEAR, 0, 0, NULL);

	if (kevent(sc->vsc_kq, kev, n, NULL, 0, NULL) < 0) {
		EPRINTLN("vtgpu: kevent register failed (%s), using timed wait",
		    strerror(errno));
		close(sc->vsc_kq);
		sc->vsc_kq = -1;
		return;
	}
	EPRINTLN("vtgpu: event-driven wait active (fence fd=%d%s)",
	    sc->vsc_poll_fd,
	    sc->vsc_poll_fd < 0 ? ", queue kicks only" : "");
}

static void *
vtgpu_worker(void *arg)
{
	struct vtgpu_softc *sc = arg;
	int rc;

	/*
	 * Initialize virglrenderer HERE, on the worker thread, so its host
	 * GL context is both created and used on the same thread.  Then hand
	 * the result back to pci_vtgpu_init() so it can fail device creation
	 * if virgl is unavailable.
	 */
	rc = vtgpu_virgl_init(sc);

	pthread_mutex_lock(&sc->vsc_mtx);
	sc->vsc_init_done = (rc == 0) ? 1 : -1;
	pthread_cond_signal(&sc->vsc_cnd);
	if (rc != 0) {
		pthread_mutex_unlock(&sc->vsc_mtx);
		return (NULL);
	}
	vtgpu_kq_setup(sc);

	while (sc->vsc_running) {
		while (sc->vsc_running &&
		    !vq_has_descs(&sc->vsc_queues[VTGPU_CONTROLQ]) &&
		    !vq_has_descs(&sc->vsc_queues[VTGPU_CURSORQ])) {
			if (!TAILQ_EMPTY(&sc->vsc_fences)) {
				/*
				 * Fenced commands are awaiting GPU completion.
				 * The guest may be blocked on one of those
				 * fences and will not ring a queue again, so we
				 * must not sleep indefinitely: wait for fence
				 * progress until the list drains (write_fence
				 * releases each chain as its fence fires).
				 * Without this the guest hangs forever on the
				 * first fenced submit/transfer.
				 */
				sc->vsc_fwait++;
				if (sc->vsc_kq >= 0) {
					struct kevent ev[2];
					struct timespec ts = {
						.tv_sec = 0,
						.tv_nsec =
						    VTGPU_KQ_BACKSTOP_MS *
						    1000000L
					};
					int nev;

					/*
					 * kevent() blocks, so the mutex must be
					 * dropped around it.  A queue kick in
					 * that window is not lost: the user
					 * event stays pending and returns
					 * immediately below.
					 */
					pthread_mutex_unlock(&sc->vsc_mtx);
					nev = kevent(sc->vsc_kq, NULL, 0, ev,
					    nitems(ev), &ts);
					pthread_mutex_lock(&sc->vsc_mtx);
					if (nev == 0)
						sc->vsc_fwait_late++;
				} else {
					struct timespec ts;

					clock_gettime(CLOCK_REALTIME, &ts);
					ts.tv_nsec += 1000000;	/* 1 ms */
					if (ts.tv_nsec >= 1000000000L) {
						ts.tv_sec++;
						ts.tv_nsec -= 1000000000L;
					}
					pthread_cond_timedwait(&sc->vsc_cnd,
					    &sc->vsc_mtx, &ts);
					sc->vsc_fwait_late++;
				}
				virgl_renderer_poll();

				/*
				 * Periodic, and rare enough to be free: tells
				 * us whether this path is hot at all and how
				 * often we fall through to the backstop rather
				 * than being woken by an event.
				 */
				if ((sc->vsc_fwait & 0xffff) == 0)
					EPRINTLN("vtgpu: fence waits=%ju "
					    "backstop=%ju", (uintmax_t)
					    sc->vsc_fwait, (uintmax_t)
					    sc->vsc_fwait_late);
			} else {
				pthread_cond_wait(&sc->vsc_cnd, &sc->vsc_mtx);
			}
		}

		if (!sc->vsc_running)
			break;

		vtgpu_process_controlq(sc, VTGPU_CONTROLQ);
		vtgpu_process_cursorq(sc);
	}
	pthread_mutex_unlock(&sc->vsc_mtx);
	return (NULL);
}

/* ----------------------------------------------------------------------- */
/* virtio glue								   */
/* ----------------------------------------------------------------------- */

static void
vtgpu_qnotify(void *arg, struct vqueue_info *vq __unused)
{
	struct vtgpu_softc *sc = arg;

	/*
	 * Trigger before signalling: an EVFILT_USER trigger is sticky, so
	 * one racing with the worker's descriptor check is still pending
	 * when it calls kevent() and wakes it immediately.  A condvar
	 * signal delivered with no waiter is simply lost, which is why the
	 * kqueue path -- not a bare poll() on the fence fd -- is what makes
	 * an event-driven wait correct here.
	 */
	if (sc->vsc_kq >= 0) {
		struct kevent kev;

		EV_SET(&kev, VTGPU_KQ_NOTIFY, EVFILT_USER, 0, NOTE_TRIGGER,
		    0, NULL);
		(void)kevent(sc->vsc_kq, &kev, 1, NULL, 0, NULL);
	}
	pthread_mutex_lock(&sc->vsc_mtx);
	pthread_cond_signal(&sc->vsc_cnd);
	pthread_mutex_unlock(&sc->vsc_mtx);
}

static void
vtgpu_reset(void *arg)
{
	struct vtgpu_softc *sc = arg;

	vi_reset_dev(&sc->vsc_vs);
}

static int
vtgpu_cfgread(void *arg, int offset, int size, uint32_t *retval)
{
	struct vtgpu_softc *sc = arg;
	const uint8_t *base = (const uint8_t *)&sc->vsc_cfg;
	size_t cfgsize = sizeof(sc->vsc_cfg);

	*retval = 0;
	if (offset < 0 || (size_t)offset + size > cfgsize)
		return (0);		/* out-of-range read reads as zero */
	memcpy(retval, base + offset, size);
	return (0);
}

static int
vtgpu_cfgwrite(void *arg __unused, int offset __unused, int size __unused,
    uint32_t val __unused)
{
	/* Only events_clear is writable; ignore for now. */
	return (0);
}

static struct virtio_consts vtgpu_vconsts = {
	.vc_name	= "vtgpu",
	.vc_nvq		= VTGPU_MAXQ,
	.vc_cfgsize	= sizeof(struct virtio_gpu_config),
	.vc_reset	= vtgpu_reset,
	.vc_qnotify	= vtgpu_qnotify,
	.vc_cfgread	= vtgpu_cfgread,
	.vc_cfgwrite	= vtgpu_cfgwrite,
	.vc_hv_caps	= VTGPU_S_HOSTCAPS,
};

/* ----------------------------------------------------------------------- */
/* Modern virtio-pci transport (virtio 1.0)				   */
/* ----------------------------------------------------------------------- */

#ifndef	PCIY_VENDOR
#define	PCIY_VENDOR	0x09
#endif

#define	VTGPU_SETLO(x, v)	((x) = ((x) & ~0xffffffffULL) | (uint32_t)(v))
#define	VTGPU_SETHI(x, v)	\
	((x) = ((x) & 0xffffffffULL) | ((uint64_t)(uint32_t)(v) << 32))

/* Append one vendor-specific virtio PCI capability (struct virtio_pci_cap). */
static void
vtgpu_add_cap(struct pci_devinst *pi, uint8_t cfg_type, uint8_t bar,
    uint32_t offset, uint32_t length, uint32_t notify_mult, bool is_notify)
{
	uint8_t buf[20];
	int len = is_notify ? 20 : 16;

	memset(buf, 0, sizeof(buf));
	buf[0] = PCIY_VENDOR;		/* cap_vndr */
	buf[1] = 0;			/* cap_next (filled by add_capability) */
	buf[2] = (uint8_t)len;		/* cap_len */
	buf[3] = cfg_type;		/* cfg_type */
	buf[4] = bar;			/* bar */
	/* buf[5..7] padding */
	memcpy(&buf[8], &offset, 4);
	memcpy(&buf[12], &length, 4);
	if (is_notify)
		memcpy(&buf[16], &notify_mult, 4);
	(void)pci_emul_add_capability(pi, buf, len);
}

/*
 * Add a virtio SHARED_MEMORY region capability (struct virtio_pci_cap64):
 * the 16-byte base cap with the shmid in the id byte, plus 64-bit offset and
 * length split into lo/hi halves.
 */
static void
vtgpu_add_shm_cap(struct pci_devinst *pi, uint8_t bar, uint8_t shmid,
    uint64_t offset, uint64_t length)
{
	uint8_t buf[24];
	uint32_t v;

	memset(buf, 0, sizeof(buf));
	buf[0] = PCIY_VENDOR;			/* cap_vndr */
	buf[1] = 0;				/* cap_next (filled in later) */
	buf[2] = 24;				/* cap_len (cap64) */
	buf[3] = VTGPU_CAP_SHARED_MEMORY_CFG;	/* cfg_type */
	buf[4] = bar;				/* bar */
	buf[5] = shmid;				/* id (shmid) */
	v = (uint32_t)offset;         memcpy(&buf[8],  &v, 4);	/* offset lo */
	v = (uint32_t)length;         memcpy(&buf[12], &v, 4);	/* length lo */
	v = (uint32_t)(offset >> 32); memcpy(&buf[16], &v, 4);	/* offset hi */
	v = (uint32_t)(length >> 32); memcpy(&buf[20], &v, 4);	/* length hi */
	(void)pci_emul_add_capability(pi, buf, 24);
}

/*
 * Allocate and advertise the host-visible memory window (venus only).  The
 * BAR is backed by a devmem segment so it is both guest-mappable (via
 * vm_mmap_memseg in the baraddr handler) and reachable from bhyve.
 */
static int
vtgpu_hostvis_setup(struct vtgpu_softc *sc, struct pci_devinst *pi)
{
	if (pci_emul_alloc_bar(pi, VTGPU_HOSTVIS_BAR, PCIBAR_MEM64,
	    VTGPU_HOSTVIS_SZ) != 0)
		return (1);
	sc->vsc_hostvis_base = vm_create_devmem(pi->pi_vmctx,
	    VM_VIRTIO_GPU_HOSTVIS, "vtgpu-hostvis", VTGPU_HOSTVIS_SZ);
	if (sc->vsc_hostvis_base == NULL ||
	    sc->vsc_hostvis_base == MAP_FAILED) {
		EPRINTLN("vtgpu: vm_create_devmem(hostvis) failed");
		return (1);
	}
	vtgpu_add_shm_cap(pi, VTGPU_HOSTVIS_BAR,
	    VIRTIO_GPU_SHM_ID_HOST_VISIBLE, 0, VTGPU_HOSTVIS_SZ);
	return (0);
}

/*
 * Map/unmap the host-visible window into guest physical space when the guest
 * programs (or clears) the BAR — same pattern as pci_fbuf.
 */
static void
vtgpu_baraddr(struct pci_devinst *pi, int baridx, int enabled,
    uint64_t address)
{
	struct vtgpu_softc *sc = pi->pi_arg;

	if (baridx != VTGPU_HOSTVIS_BAR)
		return;
	if (!enabled) {
		if (sc->vsc_hostvis_gpa != 0)
			(void)vm_munmap_memseg(pi->pi_vmctx,
			    sc->vsc_hostvis_gpa, VTGPU_HOSTVIS_SZ);
		sc->vsc_hostvis_gpa = 0;
		return;
	}
	if (vm_mmap_memseg(pi->pi_vmctx, address, VM_VIRTIO_GPU_HOSTVIS, 0,
	    VTGPU_HOSTVIS_SZ, PROT_READ | PROT_WRITE) != 0) {
		EPRINTLN("vtgpu: vm_mmap_memseg(hostvis) failed");
		return;
	}
	sc->vsc_hostvis_gpa = address;
}

/* Reset all modern transport + virtqueue state (device_status written 0). */
static void
vtgpu_modern_reset(struct vtgpu_softc *sc)
{
	struct vqueue_info *vq;
	int i;

	for (i = 0; i < VTGPU_MAXQ; i++) {
		vq = &sc->vsc_queues[i];
		vq->vq_flags = 0;
		vq->vq_last_avail = 0;
		vq->vq_next_used = 0;
		vq->vq_save_used = 0;
		vq->vq_msix_idx = VIRTIO_MSI_NO_VECTOR;
		sc->vsc_q_desc[i] = 0;
		sc->vsc_q_avail[i] = 0;
		sc->vsc_q_used[i] = 0;
		sc->vsc_q_enable[i] = 0;
	}
	sc->vsc_dev_feature_sel = 0;
	sc->vsc_drv_feature_sel = 0;
	sc->vsc_drv_features = 0;
	sc->vsc_msix_cfg = VIRTIO_MSI_NO_VECTOR;
	sc->vsc_status = 0;
	sc->vsc_qsel = 0;
	sc->vsc_vs.vs_msix_cfg_idx = VIRTIO_MSI_NO_VECTOR;
	sc->vsc_vs.vs_isr = 0;
}

/* Map a queue's three separately-addressed rings once the guest enables it. */
static void
vtgpu_modern_vq_map(struct vtgpu_softc *sc, int q)
{
	struct vqueue_info *vq = &sc->vsc_queues[q];
	struct vmctx *ctx = sc->vsc_vs.vs_pi->pi_vmctx;
	uint16_t qsz = vq->vq_qsize;

	/* desc[qsz]; avail = flags+idx+ring[qsz]+used_event; used similarly. */
	vq->vq_desc = paddr_guest2host(ctx, sc->vsc_q_desc[q],
	    (size_t)qsz * 16);
	vq->vq_avail = paddr_guest2host(ctx, sc->vsc_q_avail[q],
	    (size_t)6 + 2 * qsz);
	vq->vq_used = paddr_guest2host(ctx, sc->vsc_q_used[q],
	    (size_t)6 + 8 * qsz);

	vq->vq_flags = VQ_ALLOC;
	vq->vq_last_avail = 0;
	vq->vq_next_used = 0;
	vq->vq_save_used = 0;
}

static uint64_t
vtgpu_common_read(struct vtgpu_softc *sc, uint64_t off)
{
	uint16_t q = sc->vsc_qsel;
	bool qok = (q < VTGPU_MAXQ);
	uint64_t feat = sc->vsc_venus ? VTGPU_VENUS_FEATURES :
	    VTGPU_MODERN_FEATURES;

	switch (off) {
	case VTGPU_CC_DFSELECT:	return sc->vsc_dev_feature_sel;
	case VTGPU_CC_DF:
		if (sc->vsc_dev_feature_sel == 0)
			return (uint32_t)feat;
		if (sc->vsc_dev_feature_sel == 1)
			return (uint32_t)(feat >> 32);
		return 0;
	case VTGPU_CC_GFSELECT:	return sc->vsc_drv_feature_sel;
	case VTGPU_CC_GF:
		if (sc->vsc_drv_feature_sel == 0)
			return (uint32_t)sc->vsc_drv_features;
		if (sc->vsc_drv_feature_sel == 1)
			return (uint32_t)(sc->vsc_drv_features >> 32);
		return 0;
	case VTGPU_CC_MSIXCFG:	return sc->vsc_msix_cfg;
	case VTGPU_CC_NUMQ:	return VTGPU_MAXQ;
	case VTGPU_CC_STATUS:	return sc->vsc_status;
	case VTGPU_CC_CFGGEN:	return sc->vsc_cfg_gen;
	case VTGPU_CC_QSELECT:	return sc->vsc_qsel;
	case VTGPU_CC_QSIZE:	return qok ? sc->vsc_queues[q].vq_qsize : 0;
	case VTGPU_CC_QMSIX:	return qok ? sc->vsc_queues[q].vq_msix_idx :
				    VIRTIO_MSI_NO_VECTOR;
	case VTGPU_CC_QENABLE:	return qok ? sc->vsc_q_enable[q] : 0;
	case VTGPU_CC_QNOFF:	return q;	/* notify_off == queue index */
	case VTGPU_CC_QDESCLO:	return qok ? (uint32_t)sc->vsc_q_desc[q] : 0;
	case VTGPU_CC_QDESCHI:	return qok ? (uint32_t)(sc->vsc_q_desc[q] >> 32) : 0;
	case VTGPU_CC_QAVAILLO:	return qok ? (uint32_t)sc->vsc_q_avail[q] : 0;
	case VTGPU_CC_QAVAILHI:	return qok ? (uint32_t)(sc->vsc_q_avail[q] >> 32) : 0;
	case VTGPU_CC_QUSEDLO:	return qok ? (uint32_t)sc->vsc_q_used[q] : 0;
	case VTGPU_CC_QUSEDHI:	return qok ? (uint32_t)(sc->vsc_q_used[q] >> 32) : 0;
	}
	return 0;
}

static void
vtgpu_common_write(struct vtgpu_softc *sc, uint64_t off, uint64_t val)
{
	uint16_t q = sc->vsc_qsel;
	bool qok = (q < VTGPU_MAXQ);

	switch (off) {
	case VTGPU_CC_DFSELECT:	sc->vsc_dev_feature_sel = (uint32_t)val; break;
	case VTGPU_CC_GFSELECT:	sc->vsc_drv_feature_sel = (uint32_t)val; break;
	case VTGPU_CC_GF:
		if (sc->vsc_drv_feature_sel == 0)
			VTGPU_SETLO(sc->vsc_drv_features, val);
		else if (sc->vsc_drv_feature_sel == 1)
			VTGPU_SETHI(sc->vsc_drv_features, val);
		break;
	case VTGPU_CC_MSIXCFG:
		sc->vsc_msix_cfg = (uint16_t)val;
		sc->vsc_vs.vs_msix_cfg_idx = (uint16_t)val;
		break;
	case VTGPU_CC_STATUS:
		if (val == 0)
			vtgpu_modern_reset(sc);
		else
			sc->vsc_status = (uint8_t)val;
		break;
	case VTGPU_CC_QSELECT:	sc->vsc_qsel = (uint16_t)val; break;
	case VTGPU_CC_QSIZE:
		if (qok) sc->vsc_queues[q].vq_qsize = (uint16_t)val;
		break;
	case VTGPU_CC_QMSIX:
		if (qok) sc->vsc_queues[q].vq_msix_idx = (uint16_t)val;
		break;
	case VTGPU_CC_QENABLE:
		if (qok && val == 1) {
			sc->vsc_q_enable[q] = 1;
			vtgpu_modern_vq_map(sc, q);
		}
		break;
	case VTGPU_CC_QDESCLO:	if (qok) VTGPU_SETLO(sc->vsc_q_desc[q], val); break;
	case VTGPU_CC_QDESCHI:	if (qok) VTGPU_SETHI(sc->vsc_q_desc[q], val); break;
	case VTGPU_CC_QAVAILLO:	if (qok) VTGPU_SETLO(sc->vsc_q_avail[q], val); break;
	case VTGPU_CC_QAVAILHI:	if (qok) VTGPU_SETHI(sc->vsc_q_avail[q], val); break;
	case VTGPU_CC_QUSEDLO:	if (qok) VTGPU_SETLO(sc->vsc_q_used[q], val); break;
	case VTGPU_CC_QUSEDHI:	if (qok) VTGPU_SETHI(sc->vsc_q_used[q], val); break;
	}
}

static uint64_t
vtgpu_modern_barread(struct pci_devinst *pi, int baridx, uint64_t offset,
    int size)
{
	struct vtgpu_softc *sc = pi->pi_arg;
	struct virtio_softc *vs = &sc->vsc_vs;
	uint32_t v = 0;

	if (baridx == pci_msix_table_bar(pi))
		return (pci_emul_msix_tread(pi, offset, size));
	if (baridx != VTGPU_MODERN_BAR)
		return (0);

	if (offset >= VTGPU_OFF_COMMON && offset < VTGPU_OFF_ISR)
		return (vtgpu_common_read(sc, offset - VTGPU_OFF_COMMON));
	if (offset >= VTGPU_OFF_ISR && offset < VTGPU_OFF_DEVICE) {
		/* Reading the ISR returns and clears the pending bits. */
		VS_LOCK(vs);
		v = vs->vs_isr;
		vs->vs_isr = 0;
		VS_UNLOCK(vs);
		return (v);
	}
	if (offset >= VTGPU_OFF_DEVICE && offset < VTGPU_OFF_NOTIFY) {
		(void)vtgpu_cfgread(sc, offset - VTGPU_OFF_DEVICE, size, &v);
		return (v);
	}
	return (0);
}

static void
vtgpu_modern_barwrite(struct pci_devinst *pi, int baridx, uint64_t offset,
    int size, uint64_t value)
{
	struct vtgpu_softc *sc = pi->pi_arg;
	struct virtio_softc *vs = &sc->vsc_vs;
	uint16_t q;

	if (baridx == pci_msix_table_bar(pi)) {
		pci_emul_msix_twrite(pi, offset, size, value);
		return;
	}
	if (baridx != VTGPU_MODERN_BAR)
		return;

	if (offset >= VTGPU_OFF_COMMON && offset < VTGPU_OFF_ISR) {
		VS_LOCK(vs);
		vtgpu_common_write(sc, offset - VTGPU_OFF_COMMON, value);
		VS_UNLOCK(vs);
		return;
	}
	if (offset >= VTGPU_OFF_NOTIFY &&
	    offset < VTGPU_OFF_NOTIFY + VTGPU_REGION_LEN) {
		q = (uint16_t)((offset - VTGPU_OFF_NOTIFY) / VTGPU_NOTIFY_MULT);
		if (q < VTGPU_MAXQ)
			vtgpu_qnotify(sc, &sc->vsc_queues[q]);
		return;
	}
	if (offset >= VTGPU_OFF_DEVICE && offset < VTGPU_OFF_NOTIFY) {
		(void)vtgpu_cfgwrite(sc, offset - VTGPU_OFF_DEVICE, size,
		    (uint32_t)value);
		return;
	}
}

/* Allocate the modern config BAR and advertise its regions via caps. */
static int
vtgpu_modern_setup(struct pci_devinst *pi)
{
	if (pci_emul_alloc_bar(pi, VTGPU_MODERN_BAR, PCIBAR_MEM64,
	    VTGPU_MODERN_BAR_SZ) != 0)
		return (1);

	vtgpu_add_cap(pi, VTGPU_CAP_COMMON_CFG, VTGPU_MODERN_BAR,
	    VTGPU_OFF_COMMON, VTGPU_REGION_LEN, 0, false);
	vtgpu_add_cap(pi, VTGPU_CAP_ISR_CFG, VTGPU_MODERN_BAR,
	    VTGPU_OFF_ISR, VTGPU_REGION_LEN, 0, false);
	vtgpu_add_cap(pi, VTGPU_CAP_DEVICE_CFG, VTGPU_MODERN_BAR,
	    VTGPU_OFF_DEVICE, VTGPU_REGION_LEN, 0, false);
	vtgpu_add_cap(pi, VTGPU_CAP_NOTIFY_CFG, VTGPU_MODERN_BAR,
	    VTGPU_OFF_NOTIFY, VTGPU_REGION_LEN, VTGPU_NOTIFY_MULT, true);

	return (0);
}

/* ----------------------------------------------------------------------- */
/* PCI device init							   */
/* ----------------------------------------------------------------------- */

/*
 * Probe for a Wayland compositor socket in the standard locations.
 * This is only a last-resort EGL backend: the preferred paths are the
 * render-node (GBM) and surfaceless platforms set up in vtgpu_virgl_init,
 * both of which are fully headless.  We fall back to a compositor only if
 * no usable render node is available.  With WAYLAND_DISPLAY set, mesa's EGL
 * uses the Wayland platform and virglrenderer creates a surfaceless context
 * against it without producing any actual on-screen output.
 */
static void
vtgpu_probe_wayland(void)
{
	const char *xdg;
	char path[1024];
	static const char *names[] = { "wayland-0", "wayland-1", NULL };

	/* Absolute-path sockets in /tmp (common on FreeBSD) */
	for (int i = 0; names[i] != NULL; i++) {
		snprintf(path, sizeof(path), "/tmp/%s", names[i]);
		if (access(path, W_OK) == 0) {
			setenv("WAYLAND_DISPLAY", path, 1);
			return;
		}
	}

	/* XDG_RUNTIME_DIR sockets (standard Wayland location) */
	xdg = getenv("XDG_RUNTIME_DIR");
	if (xdg != NULL) {
		for (int i = 0; names[i] != NULL; i++) {
			snprintf(path, sizeof(path), "%s/%s", xdg, names[i]);
			if (access(path, W_OK) == 0) {
				setenv("WAYLAND_DISPLAY", path, 1);
				return;
			}
		}
	}
}

/*
 * Initialize virglrenderer, trying host GL backends in order.  Called on
 * the worker thread so the host GL context it creates is owned by the same
 * thread that will render with it (a context made current on a different
 * thread than it was created on triggers glXMakeContextCurrent BadAccess).
 * Returns 0 on success.  The environment (WAYLAND_DISPLAY) and the render
 * node fd (sc->vsc_drm_fd) were set up by pci_vtgpu_init before the worker
 * started.
 *
 * Order of preference is headless-first: with a render-node fd the initial
 * EGL attempts come up on the GBM platform (no display server); if that
 * fails we try surfaceless EGL; only then do we fall back to attaching to a
 * Wayland/X compositor.  So on a host with a usable render node no
 * compositor needs to be running at all.
 */
static int
vtgpu_virgl_init(struct vtgpu_softc *sc)
{
	int flags;
	/*
	 * When venus is enabled, OR the Venus + render-server flags into every
	 * backend attempt.  RENDER_SERVER makes virglrenderer bring up the
	 * out-of-process virgl_render_server (which it fork/execs itself, since
	 * we do not provide a get_server_fd callback) and proxy Vulkan command
	 * streams to it; VENUS selects the vkr renderer and a GBM-compatible
	 * resource layout for GL<->Vulkan interop.
	 */
	int venus = sc->vsc_venus ?
	    (VIRGL_RENDERER_VENUS | VIRGL_RENDERER_RENDER_SERVER |
	     VIRGL_RENDERER_USE_EXTERNAL_BLOB) : 0;

	/*
	 * Headless via the render node (get_drm_fd -> GBM platform).
	 *
	 * Prefer desktop GL (USE_EGL alone) over GLES: on this host desktop
	 * GL exposes the full GL 4.6 / GLSL 460 profile, whereas the GLES
	 * backend caps the guest at a GL 4.3-equivalent profile.  Some guests
	 * (notably Blender) require the higher profile, and the GLES profile
	 * can also mis-render text/OSD.  GLES stays as a fallback for hosts
	 * where desktop GL is unavailable.
	 */
	flags = VIRGL_RENDERER_USE_EGL;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}
	flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_GLES;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}

	/* Headless via surfaceless EGL (no window system, no render-node fd). */
	flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_SURFACELESS;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}
	flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_GLES |
	    VIRGL_RENDERER_USE_SURFACELESS;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}

	/* Probe for a running Wayland compositor and retry EGL. */
	if (getenv("WAYLAND_DISPLAY") == NULL && getenv("DISPLAY") == NULL)
		vtgpu_probe_wayland();
	flags = VIRGL_RENDERER_USE_EGL;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}
	flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_GLES;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}

	flags = VIRGL_RENDERER_USE_GLX;
	if (virgl_renderer_init(sc, flags | venus, &vtgpu_virgl_cbs) == 0) {
		/*
		 * Unconditional: this fires once per VM start, costs nothing,
		 * and is the first thing wanted when the guest shows no
		 * acceleration.  Hiding it behind debug=on (which must stay
		 * off for performance) leaves no way to tell whether virgl
		 * came up at all.
		 */
		EPRINTLN("vtgpu: virgl init ok (flags=0x%x venus=0x%x)",
		    flags, venus);
		return (0);
	}

	EPRINTLN("vtgpu: virgl init FAILED - every backend rejected "
	    "(no GL/EGL?  check the render node and virglrenderer)");
	return (1);
}

static int
pci_vtgpu_init(struct pci_devinst *pi, nvlist_t *nvl)
{
	struct vtgpu_softc *sc;
	const char *render_node, *wayland_display;
	int rc;

	sc = calloc(1, sizeof(*sc));
	if (sc == NULL)
		return (1);

	sc->vsc_ctx    = pi->pi_vmctx;
	sc->vsc_width  = VTGPU_DEFAULT_WIDTH;
	sc->vsc_height = VTGPU_DEFAULT_HEIGHT;
	sc->vsc_drm_fd = -1;
	sc->vsc_kq = -1;
	sc->vsc_poll_fd = -1;
	TAILQ_INIT(&sc->vsc_fences);

	render_node     = NULL;
	wayland_display = NULL;

	/* Optional config overrides. */
	if (nvl != NULL) {
		const char *w = get_config_value_node(nvl, "width");
		const char *h = get_config_value_node(nvl, "height");
		if (w) sc->vsc_width  = (uint32_t)atoi(w);
		if (h) sc->vsc_height = (uint32_t)atoi(h);
		render_node     = get_config_value_node(nvl, "render");
		wayland_display = get_config_value_node(nvl, "wayland");
		sc->vsc_venus   = get_config_bool_node_default(nvl, "venus",
		    false);
		sc->vsc_scanout_probe = get_config_bool_node_default(nvl,
		    "scanout_probe", false);
		sc->vsc_scanout_linear = get_config_bool_node_default(nvl,
		    "scanout_linear", false);
		{
			const char *disp = get_config_value_node(nvl, "display");

			/*
			 * display=unix:/path publishes the scanout to an
			 * external viewer.  Unset (the default) leaves
			 * vsc_display NULL and every path below inert.
			 */
			if (disp != NULL && strncmp(disp, "unix:", 5) == 0) {
				sc->vsc_display = gpu_display_init(disp + 5);
				/*
				 * The USB tablet drops every event unless a
				 * graphics context exists: umouse_event()
				 * returns early when console_get_image() is
				 * NULL, and scales the absolute coordinates it
				 * reports against the image's dimensions.  The
				 * fbuf device is what normally creates one, so
				 * without graphics=yes the pointer is dead
				 * while the PS/2 keyboard keeps working.
				 *
				 * Create one ourselves if nothing has, sized
				 * to the scanout so the scaling comes out
				 * right.  Only when the viewer is configured,
				 * and only if fbuf has not already done it, so
				 * that setups not using this option are
				 * untouched.
				 */
				if (sc->vsc_display != NULL &&
				    console_get_image() == NULL)
					console_init((int)sc->vsc_width,
					    (int)sc->vsc_height, NULL);
			} else if (disp != NULL) {
				EPRINTLN("vtgpu: display=%s not understood, "
				    "expected unix:/path", disp);
			}
		}
		pci_vtgpu_debug = get_config_bool_node_default(nvl, "debug",
		    false);
	}

	/* Environment overrides. */
	if (render_node == NULL)
		render_node = getenv("VTGPU_RENDER_NODE");
	if (wayland_display != NULL)
		setenv("WAYLAND_DISPLAY", wayland_display, 1);

	/*
	 * Open the host DRM render node and hand it to virglrenderer (via the
	 * get_drm_fd callback) so it initializes EGL headlessly on the GBM
	 * platform — no Wayland/X compositor required.  Default to renderD128;
	 * a specific node can be selected with render=/dev/dri/renderDNNN.
	 * If this fails we leave the fd at -1 and virgl_init falls back to the
	 * surfaceless / Wayland / GLX chain.
	 */
	if (render_node == NULL)
		render_node = "/dev/dri/renderD128";
	sc->vsc_drm_fd = open(render_node, O_RDWR | O_CLOEXEC);
	if (sc->vsc_drm_fd < 0)
		EPRINTLN("vtgpu: open(%s) failed, falling back to "
		    "surfaceless/compositor EGL", render_node);

	/*
	 * virglrenderer is initialized later, on the worker thread
	 * (vtgpu_worker -> vtgpu_virgl_init), because its host GL context is
	 * thread-bound and must live on the same thread that renders.  The
	 * environment set up above steers which backend that init selects.
	 */

	/* Fill in the device config space.  Base layout advertises 2 capsets
	 * (VIRGL + VIRGL2); with venus=on a third (VENUS) is added. */
	sc->vsc_cfg.num_scanouts = VTGPU_NUM_SCANOUTS;
	sc->vsc_cfg.num_capsets  = sc->vsc_venus ? 3 : 2;

	pthread_mutex_init(&sc->vsc_mtx, NULL);
	pthread_cond_init(&sc->vsc_cnd, NULL);

	vi_softc_linkup(&sc->vsc_vs, &vtgpu_vconsts, sc, pi, sc->vsc_queues);
	sc->vsc_vs.vs_mtx = &sc->vsc_mtx;

	/* Set up queues. */
	sc->vsc_queues[VTGPU_CONTROLQ].vq_qsize = VTGPU_RINGSZ;
	sc->vsc_queues[VTGPU_CURSORQ].vq_qsize  = VTGPU_RINGSZ;

	/*
	 * MSI-X on BAR 1 (one vector per queue + config).  We reuse the
	 * shared helper only for the MSI-X/MSI capability plumbing; the
	 * transport itself (below) is modern, implemented in this file.
	 */
	if (vi_intr_init(&sc->vsc_vs, VTGPU_MSIX_BAR, true) != 0) {
		warnx("vtgpu: vi_intr_init failed");
		free(sc);
		return (1);
	}

	/* Modern virtio-pci config BAR (BAR 4) + describing capabilities. */
	if (vtgpu_modern_setup(pi) != 0) {
		warnx("vtgpu: modern transport setup failed");
		free(sc);
		return (1);
	}
	vtgpu_modern_reset(sc);

	/*
	 * Host-visible memory window (venus only): a second MEM64 BAR + a
	 * SHARED_MEMORY cap so the guest reports +host_visible and the venus
	 * ICD attaches.  Requires the VM_VIRTIO_GPU_HOSTVIS devmem segid in the
	 * (rebuilt) kernel.
	 */
	if (sc->vsc_venus && vtgpu_hostvis_setup(sc, pi) != 0) {
		warnx("vtgpu: host-visible window setup failed");
		free(sc);
		return (1);
	}

	/*
	 * PCI identity.  Device ID 0x1040+type routes the guest down the
	 * modern virtio-pci probe path; revision >= 1 marks it
	 * non-transitional.
	 */
	pci_set_cfgdata16(pi, PCIR_DEVICE, VIRTIO_DEV_GPU);
	pci_set_cfgdata16(pi, PCIR_VENDOR, VIRTIO_VENDOR);
	pci_set_cfgdata8(pi, PCIR_REVID, 1);
	pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_DISPLAY);
	pci_set_cfgdata8(pi, PCIR_SUBCLASS, PCIS_DISPLAY_OTHER);
	pci_set_cfgdata16(pi, PCIR_SUBDEV_0, VIRTIO_ID_GPU);
	pci_set_cfgdata16(pi, PCIR_SUBVEND_0, VIRTIO_VENDOR);

	/*
	 * Start the worker.  It initializes virglrenderer on its own thread
	 * (so the GL context is owned by the rendering thread) and reports
	 * the result via vsc_init_done; wait for it so we can fail device
	 * creation cleanly if virgl is unavailable.
	 */
	sc->vsc_running = true;
	sc->vsc_init_done = 0;
	if (pthread_create(&sc->vsc_tid, NULL, vtgpu_worker, sc) != 0) {
		warnx("vtgpu: pthread_create failed");
		free(sc);
		return (1);
	}
	pthread_set_name_np(sc->vsc_tid, "vtgpu:work");

	pthread_mutex_lock(&sc->vsc_mtx);
	while (sc->vsc_init_done == 0)
		pthread_cond_wait(&sc->vsc_cnd, &sc->vsc_mtx);
	rc = sc->vsc_init_done;
	pthread_mutex_unlock(&sc->vsc_mtx);

	if (rc < 0) {
		warnx("vtgpu: virgl_renderer_init failed (tried EGL+GLES, "
		    "EGL, Wayland EGL, GLX); from a TTY pass "
		    "wayland=<socket-path> or set WAYLAND_DISPLAY");
		sc->vsc_running = false;
		pthread_join(sc->vsc_tid, NULL);
		free(sc);
		return (1);
	}

	return (0);
}

static const struct pci_devemu pci_de_vtgpu = {
	.pe_emu		= "virtio-gpu",
	.pe_init	= pci_vtgpu_init,
	.pe_barwrite	= vtgpu_modern_barwrite,
	.pe_barread	= vtgpu_modern_barread,
	.pe_baraddr	= vtgpu_baraddr,
};
PCI_EMUL_SET(pci_de_vtgpu);
