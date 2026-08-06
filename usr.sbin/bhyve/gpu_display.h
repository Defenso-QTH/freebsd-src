/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Defenso
 */

/*
 * Protocol between bhyve's virtio-gpu device and an external viewer.
 *
 * The guest renders on the host GPU already: with venus its images live in
 * host GPU memory the whole time.  Rather than read the scanout back, encode
 * it and ship it over VNC -- which costs the guest a full framebuffer copy
 * and a software encode, on CPUs that measurement showed were already the
 * bottleneck -- hand the buffer to a viewer on the host and let the host
 * compositor present it.
 *
 * The viewer is a separate process, not linked into bhyve: the VMM stays free
 * of Wayland and EGL dependencies, and the viewer can be jailed with nothing
 * but this socket.  Buffers are passed as file descriptors over SCM_RIGHTS.
 *
 * Transport is SOCK_STREAM with explicit length framing, deliberately not
 * SOCK_SEQPACKET: FreeBSD's AF_UNIX SOCK_SEQPACKET does not preserve message
 * boundaries the way Linux's does, which is the same trap that broke
 * virglrenderer's render-server protocol on FreeBSD.
 *
 * Every message begins with gpu_display_hdr.  len counts the header, so a
 * reader can skip a message whose type it does not know.
 */

#ifndef _GPU_DISPLAY_H_
#define	_GPU_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

#define	GPU_DISPLAY_VERSION	2
/*
 * Large enough for a cursor image inline: 64x64 is the usual hardware cursor
 * size and 128x128 is not unheard of, at 4 bytes a pixel.  Everything else is
 * tiny.  Sending the pixels in the message rather than over a shared buffer
 * keeps the cursor path free of fd lifetime rules for something updated a few
 * times a second at most.
 */
#define	GPU_DISPLAY_MAX_MSG	(72 * 1024)

enum gpu_display_msg_type {
	/* host -> viewer */
	GPU_DISPLAY_MSG_HELLO	= 1,	/* first message on connect */
	GPU_DISPLAY_MSG_SCANOUT	= 2,	/* buffer handed over; carries an fd */
	GPU_DISPLAY_MSG_FRAME	= 3,	/* contents changed; damage rect */
	GPU_DISPLAY_MSG_UNBIND	= 4,	/* guest dropped the scanout */
	GPU_DISPLAY_MSG_CURSOR	= 5,	/* hardware cursor image, or hide */

	/* viewer -> host */
	GPU_DISPLAY_MSG_KEY	= 128,
	GPU_DISPLAY_MSG_PTR	= 129,
};

/*
 * How the buffer named by the passed fd should be interpreted.
 *
 * DMABUF is the point of the exercise: the viewer imports it with
 * zwp_linux_dmabuf_v1 and nothing is ever copied.  It is only available when
 * virglrenderer reports has_dmabuf_export for the resource, which on FreeBSD
 * is not a given -- RADV has historically handed back OPAQUE fds instead.
 *
 * SHM is the fallback for when it cannot: bhyve reads the scanout back into a
 * shared memory segment and the viewer maps it.  One copy instead of none,
 * but the guest still does no capture and no encode, which is where the cost
 * actually was.
 */
enum gpu_display_transport {
	GPU_DISPLAY_XPORT_DMABUF	= 1,
	GPU_DISPLAY_XPORT_SHM		= 2,
};

struct gpu_display_hdr {
	uint32_t	type;		/* enum gpu_display_msg_type */
	uint32_t	len;		/* total bytes, including this header */
};

struct gpu_display_hello {
	struct gpu_display_hdr	hdr;
	uint32_t		version;
	uint32_t		pad;
};

/* Carries exactly one fd via SCM_RIGHTS. */
struct gpu_display_scanout {
	struct gpu_display_hdr	hdr;
	uint32_t		buffer_id;	/* names this buffer for FRAME */
	uint32_t		transport;	/* enum gpu_display_transport */
	uint32_t		width;
	uint32_t		height;
	uint32_t		stride;
	uint32_t		drm_fourcc;	/* DMABUF only */
	uint32_t		planes;		/* DMABUF only */
	uint32_t		offset;		/* DMABUF: byte offset of plane 0 */
	uint64_t		modifier;	/* DMABUF only */
	uint64_t		size;		/* SHM only: bytes to map */
};

/*
 * A compositor page-flips between a small set of buffers -- sway uses two --
 * so each is exported once and named, and a flip is just a reference to one
 * of them.  Re-exporting a dma_buf per frame would pass a new fd sixty times
 * a second for no reason.
 */
/*
 * has_fence means an fd rides with this message via SCM_RIGHTS: a sync_file
 * for the guest's rendering into this buffer.  The viewer must wait on it
 * before sampling.
 *
 * Without it the viewer can read a buffer the guest is still drawing into and
 * show parts of two frames at once -- visible as a doubled image on anything
 * rendering fast enough to lose the race.  The guest does not fence
 * SET_SCANOUT itself, so the fence is created here, on the context that owns
 * the resource.
 */
struct gpu_display_frame {
	struct gpu_display_hdr	hdr;
	uint32_t		buffer_id;	/* which buffer is now scanned out */
	uint32_t		has_fence;	/* an fd accompanies this message */
	uint32_t		x;
	uint32_t		y;
	uint32_t		w;
	uint32_t		h;
};

/*
 * Input is fed straight into bhyve's existing console layer, the same entry
 * points rfb.c uses, so the guest's keyboard and tablet need no changes and
 * nothing in console.c is modified -- we are simply a second producer.
 *
 * A Wayland viewer has evdev keycodes natively, which is what ps2kbd_event()
 * ultimately wants, so send those and leave keysym 0 unless the viewer has a
 * reason to translate.
 */
struct gpu_display_key {
	struct gpu_display_hdr	hdr;
	uint32_t		down;
	uint32_t		keysym;		/* X11 keysym, or 0 */
	uint32_t		keycode;	/* evdev keycode, or 0 */
	uint32_t		pad;
};

/* Coordinates are absolute in guest pixels; the USB tablet is an absolute
 * device, so the viewer scales from its window rather than sending motion. */
struct gpu_display_ptr {
	struct gpu_display_hdr	hdr;
	uint32_t		button;		/* mask, as console_ptr_event */
	int32_t			x;
	int32_t			y;
	uint32_t		pad;
};

/*
 * The guest's hardware cursor.
 *
 * A guest that uses the cursor plane never draws its pointer into the scanout,
 * so without this the viewer shows no cursor at all -- the host compositor's
 * own cursor is drawn over the surface and looks like one, which makes the
 * absence easy to miss.
 *
 * No position is carried.  The viewer maps its pointer onto the guest one to
 * one, so the host compositor already knows where the cursor is; handing it
 * the image and letting it draw at the real pointer position is both simpler
 * and lower latency than compositing at a position the guest reports after
 * the fact.  MOVE_CURSOR therefore needs no message at all.
 *
 * hidden is set when the guest asks for no cursor (resource 0), in which case
 * no pixels follow.
 */
struct gpu_display_cursor {
	struct gpu_display_hdr	hdr;
	uint32_t		width;
	uint32_t		height;
	uint32_t		hot_x;
	uint32_t		hot_y;
	uint32_t		hidden;
	uint32_t		pad;
	/* width * height * 4 bytes of ARGB8888 follow, unless hidden */
};

struct gpu_display;

/*
 * path is a filesystem path for the listening AF_UNIX socket.  Returns NULL
 * and leaves the device fully functional if the socket cannot be created:
 * the viewer is optional and its absence must never keep a guest from
 * booting.
 */
struct gpu_display *gpu_display_init(const char *path);

/*
 * Publish the guest's hardware cursor.  pixels is width*height ARGB8888, or
 * NULL to hide the cursor.  The data is copied; the caller keeps ownership.
 */
void	gpu_display_cursor(struct gpu_display *gd, uint32_t width,
	    uint32_t height, uint32_t hot_x, uint32_t hot_y,
	    const void *pixels);

/*
 * Hand the current scanout buffer to the viewer.  fd is consumed (closed)
 * regardless of outcome.  Safe to call with no viewer connected.
 */
void	gpu_display_scanout(struct gpu_display *gd,
	    const struct gpu_display_scanout *info, int fd);

/*
 * Tell the viewer which buffer is now on screen.  For a DRM page-flipping
 * compositor this is driven by SET_SCANOUT, not RESOURCE_FLUSH: the flip IS
 * the scanout change, and RESOURCE_FLUSH never arrives at all.
 *
 * Called from the virtio-gpu worker while it processes guest commands, so it
 * never blocks -- a viewer that is not keeping up loses frames rather than
 * stalling the guest.
 */
/*
 * fence_fd, if >= 0, is a sync_file the viewer must wait on before sampling
 * the buffer; it is consumed (closed) either way.
 */
void	gpu_display_frame(struct gpu_display *gd, uint32_t buffer_id,
	    int fence_fd, uint32_t x, uint32_t y, uint32_t w, uint32_t h);

/* Has this buffer already been exported to the viewer? */
bool	gpu_display_have_buffer(struct gpu_display *gd, uint32_t buffer_id);

void	gpu_display_unbind(struct gpu_display *gd);

#endif /* _GPU_DISPLAY_H_ */
