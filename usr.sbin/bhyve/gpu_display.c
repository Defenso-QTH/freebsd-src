/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Defenso
 */

/*
 * Host side of the virtio-gpu external viewer protocol; see gpu_display.h for
 * why it exists and what goes over the wire.
 *
 * Threading: gpu_display_scanout() and gpu_display_frame() are called from the
 * virtio-gpu worker thread while it processes guest commands.  The listen and
 * input paths run on the mevent thread.  A single mutex covers the connection
 * state, and the sends are non-blocking so a slow or wedged viewer can never
 * stall the guest -- frames are dropped instead.
 *
 * Only one viewer is served at a time; a second connection replaces the first,
 * which keeps reconnect handling trivial.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config.h"
#include "console.h"
#include "debug.h"
#include "gpu_display.h"
#include "mevent.h"

struct gpu_display {
	pthread_mutex_t	mtx;
	int		lfd;		/* listening socket */
	int		cfd;		/* connected viewer, -1 if none */
	struct mevent	*levp;
	struct mevent	*cevp;
	char		*path;

	/*
	 * Buffers already handed to the viewer.  A compositor cycles through a
	 * couple of them, so they are exported once and thereafter referenced
	 * by id; the fds are kept so a viewer connecting mid-session can be
	 * given the whole set rather than waiting for a mode change.
	 */
#define	GPU_DISPLAY_MAX_BUFS	8
	struct {
		uint32_t		id;
		bool			used;
		struct gpu_display_scanout info;
		int			fd;
	}		bufs[GPU_DISPLAY_MAX_BUFS];

	/*
	 * The cursor last set by the guest, kept so a viewer connecting after
	 * the guest set it is not left with no pointer until the guest happens
	 * to change it -- which for a desktop that set its cursor at startup
	 * may be never.
	 */
	struct gpu_display_cursor	cursor;
	void				*cursor_pixels;
	size_t				cursor_bytes;
	bool				cursor_valid;

	uint64_t	frames_sent;
	uint64_t	frames_dropped;
	uint64_t	keys_in;
	uint64_t	ptrs_in;

	/* Partial input message accumulator. */
	uint8_t		inbuf[GPU_DISPLAY_MAX_MSG];
	size_t		inlen;
};

/*
 * Write a message, optionally with one fd attached.  Returns false if the
 * viewer could not take it; the caller decides whether that is fatal.
 *
 * The socket is non-blocking.  A short write would desynchronise the stream,
 * so it is treated as a lost viewer rather than retried: dropping the
 * connection is recoverable (it reconnects), a corrupt stream is not.
 */
static bool
gd_send(struct gpu_display *gd, const void *msg, size_t len, int fd)
{
	struct msghdr mh;
	struct iovec iov;
	union {
		struct cmsghdr hdr;
		unsigned char buf[CMSG_SPACE(sizeof(int))];
	} cmsgbuf;
	ssize_t n;

	if (gd->cfd < 0)
		return (false);

	memset(&mh, 0, sizeof(mh));
	iov.iov_base = __DECONST(void *, msg);
	iov.iov_len = len;
	mh.msg_iov = &iov;
	mh.msg_iovlen = 1;

	if (fd >= 0) {
		memset(&cmsgbuf, 0, sizeof(cmsgbuf));
		mh.msg_control = cmsgbuf.buf;
		mh.msg_controllen = sizeof(cmsgbuf.buf);
		cmsgbuf.hdr.cmsg_len = CMSG_LEN(sizeof(int));
		cmsgbuf.hdr.cmsg_level = SOL_SOCKET;
		cmsgbuf.hdr.cmsg_type = SCM_RIGHTS;
		memcpy(CMSG_DATA(&cmsgbuf.hdr), &fd, sizeof(fd));
	}

	n = sendmsg(gd->cfd, &mh, MSG_NOSIGNAL);
	return (n == (ssize_t)len);
}

/* Caller holds gd->mtx. */
static void
gd_drop_client(struct gpu_display *gd)
{

	if (gd->cfd < 0)
		return;
	if (gd->cevp != NULL) {
		mevent_delete(gd->cevp);
		gd->cevp = NULL;
	}
	close(gd->cfd);
	gd->cfd = -1;
	gd->inlen = 0;
	EPRINTLN("gpu_display: viewer disconnected (frames sent=%ju "
	    "dropped=%ju)", (uintmax_t)gd->frames_sent,
	    (uintmax_t)gd->frames_dropped);
}

/*
 * One complete message from the viewer.  Input is handed to the same console
 * entry points rfb.c uses, so it reaches whatever keyboard and pointer the
 * guest was configured with and nothing in console.c changes.
 */
static void
gd_handle_input(struct gpu_display *gd, const struct gpu_display_hdr *hdr,
    size_t len)
{

	switch (hdr->type) {
	case GPU_DISPLAY_MSG_KEY: {
		const struct gpu_display_key *k = (const void *)hdr;

		if (len < sizeof(*k))
			return;
		/*
		 * Report the first of each kind, then rarely.  Whether events
		 * arrive here at all is the question that splits "the viewer
		 * is not sending" from "the guest is not receiving", and the
		 * two need opposite fixes.
		 */
		if (gd->keys_in++ == 0 || (gd->keys_in & 0xff) == 0)
			EPRINTLN("gpu_display: key #%ju code=0x%x sym=0x%x %s",
			    (uintmax_t)gd->keys_in, k->keycode, k->keysym,
			    k->down ? "down" : "up");
		console_key_event((int)k->down, k->keysym, k->keycode);
		break;
	}
	case GPU_DISPLAY_MSG_PTR: {
		const struct gpu_display_ptr *p = (const void *)hdr;

		if (len < sizeof(*p))
			return;
		if (gd->ptrs_in++ == 0 || (gd->ptrs_in & 0x3ff) == 0)
			EPRINTLN("gpu_display: ptr #%ju buttons=0x%x %d,%d",
			    (uintmax_t)gd->ptrs_in, p->button, p->x, p->y);
		console_ptr_event((uint8_t)p->button, p->x, p->y);
		break;
	}
	default:
		/* Unknown types are skipped by length, not fatal. */
		break;
	}
}

static void
gd_client_readable(int fd, enum ev_type ev __unused, void *arg)
{
	struct gpu_display *gd = arg;
	ssize_t n;

	pthread_mutex_lock(&gd->mtx);
	if (fd != gd->cfd) {
		pthread_mutex_unlock(&gd->mtx);
		return;
	}

	n = read(fd, gd->inbuf + gd->inlen, sizeof(gd->inbuf) - gd->inlen);
	if (n <= 0) {
		if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
			pthread_mutex_unlock(&gd->mtx);
			return;
		}
		gd_drop_client(gd);
		pthread_mutex_unlock(&gd->mtx);
		return;
	}
	gd->inlen += (size_t)n;

	/* Drain every complete message the buffer now holds. */
	for (;;) {
		struct gpu_display_hdr hdr;

		if (gd->inlen < sizeof(hdr))
			break;
		memcpy(&hdr, gd->inbuf, sizeof(hdr));
		if (hdr.len < sizeof(hdr) || hdr.len > sizeof(gd->inbuf)) {
			EPRINTLN("gpu_display: viewer sent bad length %u, "
			    "dropping", hdr.len);
			gd_drop_client(gd);
			break;
		}
		if (gd->inlen < hdr.len)
			break;
		gd_handle_input(gd, (const struct gpu_display_hdr *)gd->inbuf,
		    hdr.len);
		memmove(gd->inbuf, gd->inbuf + hdr.len, gd->inlen - hdr.len);
		gd->inlen -= hdr.len;
	}
	pthread_mutex_unlock(&gd->mtx);
}

static void gd_send_cursor_locked(struct gpu_display *gd);

static void
gd_accept(int fd, enum ev_type ev __unused, void *arg)
{
	struct gpu_display *gd = arg;
	struct gpu_display_hello hello;
	int cfd;

	cfd = accept(fd, NULL, NULL);
	if (cfd < 0)
		return;

	pthread_mutex_lock(&gd->mtx);
	/* A new viewer replaces the old one. */
	gd_drop_client(gd);

	if (fcntl(cfd, F_SETFL, O_NONBLOCK) != 0) {
		close(cfd);
		pthread_mutex_unlock(&gd->mtx);
		return;
	}
	gd->cfd = cfd;

	memset(&hello, 0, sizeof(hello));
	hello.hdr.type = GPU_DISPLAY_MSG_HELLO;
	hello.hdr.len = sizeof(hello);
	hello.version = GPU_DISPLAY_VERSION;
	if (!gd_send(gd, &hello, sizeof(hello), -1)) {
		gd_drop_client(gd);
		pthread_mutex_unlock(&gd->mtx);
		return;
	}

	gd->cevp = mevent_add(cfd, EVF_READ, gd_client_readable, gd);
	if (gd->cevp == NULL) {
		gd_drop_client(gd);
		pthread_mutex_unlock(&gd->mtx);
		return;
	}

	/*
	 * Same reasoning for the cursor: a desktop sets it once at startup, so
	 * a viewer attaching later would otherwise have no pointer for the
	 * rest of the session.
	 */
	gd_send_cursor_locked(gd);

	/*
	 * Replay every known buffer so a viewer connecting mid-session can
	 * present the next flip immediately instead of waiting for the guest
	 * to allocate a new one, which it may never do.
	 */
	for (size_t i = 0; i < GPU_DISPLAY_MAX_BUFS; i++) {
		int dup_fd;

		if (!gd->bufs[i].used || gd->bufs[i].fd < 0)
			continue;
		dup_fd = dup(gd->bufs[i].fd);
		if (dup_fd < 0)
			continue;
		if (!gd_send(gd, &gd->bufs[i].info, sizeof(gd->bufs[i].info),
		    dup_fd))
			gd_drop_client(gd);
		close(dup_fd);
	}
	EPRINTLN("gpu_display: viewer connected on %s", gd->path);
	pthread_mutex_unlock(&gd->mtx);
}

struct gpu_display *
gpu_display_init(const char *path)
{
	struct gpu_display *gd;
	struct sockaddr_un sun;

	if (path == NULL || strlen(path) >= sizeof(sun.sun_path)) {
		EPRINTLN("gpu_display: bad socket path");
		return (NULL);
	}

	gd = calloc(1, sizeof(*gd));
	if (gd == NULL)
		return (NULL);
	gd->cfd = -1;
	for (size_t i = 0; i < GPU_DISPLAY_MAX_BUFS; i++)
		gd->bufs[i].fd = -1;
	pthread_mutex_init(&gd->mtx, NULL);

	gd->lfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (gd->lfd < 0) {
		EPRINTLN("gpu_display: socket failed: %s", strerror(errno));
		goto fail;
	}

	memset(&sun, 0, sizeof(sun));
	sun.sun_family = AF_UNIX;
	strlcpy(sun.sun_path, path, sizeof(sun.sun_path));
	(void)unlink(path);
	if (bind(gd->lfd, (struct sockaddr *)&sun, sizeof(sun)) != 0) {
		EPRINTLN("gpu_display: bind %s failed: %s", path,
		    strerror(errno));
		goto fail;
	}
	if (listen(gd->lfd, 1) != 0) {
		EPRINTLN("gpu_display: listen failed: %s", strerror(errno));
		goto fail;
	}

	gd->path = strdup(path);
	gd->levp = mevent_add(gd->lfd, EVF_READ, gd_accept, gd);
	if (gd->levp == NULL) {
		EPRINTLN("gpu_display: mevent_add failed");
		goto fail;
	}

	EPRINTLN("gpu_display: listening on %s", path);
	return (gd);

fail:
	if (gd->lfd >= 0)
		close(gd->lfd);
	free(gd->path);
	free(gd);
	return (NULL);
}

bool
gpu_display_have_buffer(struct gpu_display *gd, uint32_t buffer_id)
{
	bool found = false;

	if (gd == NULL)
		return (true);	/* nothing to publish to */

	pthread_mutex_lock(&gd->mtx);
	for (size_t i = 0; i < GPU_DISPLAY_MAX_BUFS; i++) {
		if (gd->bufs[i].used && gd->bufs[i].id == buffer_id) {
			found = true;
			break;
		}
	}
	pthread_mutex_unlock(&gd->mtx);
	return (found);
}

void
gpu_display_scanout(struct gpu_display *gd, const struct gpu_display_scanout *info,
    int fd)
{
	struct gpu_display_scanout msg;
	size_t slot;

	if (gd == NULL) {
		if (fd >= 0)
			close(fd);
		return;
	}

	msg = *info;
	msg.hdr.type = GPU_DISPLAY_MSG_SCANOUT;
	msg.hdr.len = sizeof(msg);

	pthread_mutex_lock(&gd->mtx);

	/* Reuse the slot for this id, else take a free one, else the oldest. */
	slot = GPU_DISPLAY_MAX_BUFS;
	for (size_t i = 0; i < GPU_DISPLAY_MAX_BUFS; i++) {
		if (gd->bufs[i].used && gd->bufs[i].id == msg.buffer_id) {
			slot = i;
			break;
		}
		if (!gd->bufs[i].used && slot == GPU_DISPLAY_MAX_BUFS)
			slot = i;
	}
	if (slot == GPU_DISPLAY_MAX_BUFS)
		slot = 0;

	if (gd->bufs[slot].used && gd->bufs[slot].fd >= 0)
		close(gd->bufs[slot].fd);
	gd->bufs[slot].id = msg.buffer_id;
	gd->bufs[slot].info = msg;
	gd->bufs[slot].fd = fd >= 0 ? dup(fd) : -1;
	gd->bufs[slot].used = true;

	if (gd->cfd >= 0 && !gd_send(gd, &msg, sizeof(msg), fd))
		gd_drop_client(gd);
	pthread_mutex_unlock(&gd->mtx);

	if (fd >= 0)
		close(fd);
}

void
gpu_display_frame(struct gpu_display *gd, uint32_t buffer_id, int fence_fd,
    uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
	struct gpu_display_frame msg;

	if (gd == NULL) {
		if (fence_fd >= 0)
			close(fence_fd);
		return;
	}

	memset(&msg, 0, sizeof(msg));
	msg.hdr.type = GPU_DISPLAY_MSG_FRAME;
	msg.hdr.len = sizeof(msg);
	msg.buffer_id = buffer_id;
	msg.has_fence = fence_fd >= 0;
	msg.x = x;
	msg.y = y;
	msg.w = w;
	msg.h = h;

	pthread_mutex_lock(&gd->mtx);
	if (gd->cfd < 0) {
		pthread_mutex_unlock(&gd->mtx);
		if (fence_fd >= 0)
			close(fence_fd);
		return;
	}
	/*
	 * Called from the guest command path.  A viewer that is not draining
	 * must not become the guest's problem, so a frame that will not fit
	 * in the socket buffer is dropped and counted, not retried.
	 */
	if (gd_send(gd, &msg, sizeof(msg), fence_fd))
		gd->frames_sent++;
	else if (errno == EAGAIN || errno == EWOULDBLOCK)
		gd->frames_dropped++;
	else
		gd_drop_client(gd);
	pthread_mutex_unlock(&gd->mtx);
	if (fence_fd >= 0)
		close(fence_fd);
}

/* Caller must hold gd->mtx.  Declared above gd_accept. */
static void
gd_send_cursor_locked(struct gpu_display *gd)
{
	uint8_t buf[GPU_DISPLAY_MAX_MSG];
	size_t len = sizeof(struct gpu_display_cursor);

	if (!gd->cursor_valid || gd->cfd < 0)
		return;
	if (!gd->cursor.hidden)
		len += gd->cursor_bytes;
	if (len > sizeof(buf))
		return;			/* refused at set time; belt and braces */

	memcpy(buf, &gd->cursor, sizeof(gd->cursor));
	((struct gpu_display_cursor *)buf)->hdr.len = (uint32_t)len;
	if (!gd->cursor.hidden && gd->cursor_pixels != NULL)
		memcpy(buf + sizeof(gd->cursor), gd->cursor_pixels,
		    gd->cursor_bytes);

	if (!gd_send(gd, buf, len, -1) &&
	    errno != EAGAIN && errno != EWOULDBLOCK)
		gd_drop_client(gd);
}

void
gpu_display_cursor(struct gpu_display *gd, uint32_t width, uint32_t height,
    uint32_t hot_x, uint32_t hot_y, const void *pixels)
{
	size_t bytes;

	if (gd == NULL)
		return;

	bytes = (size_t)width * height * 4;
	if (pixels != NULL &&
	    sizeof(struct gpu_display_cursor) + bytes > GPU_DISPLAY_MAX_MSG) {
		EPRINTLN("gpu_display: cursor %ux%u too large, ignored",
		    width, height);
		return;
	}

	pthread_mutex_lock(&gd->mtx);

	memset(&gd->cursor, 0, sizeof(gd->cursor));
	gd->cursor.hdr.type = GPU_DISPLAY_MSG_CURSOR;
	gd->cursor.width = width;
	gd->cursor.height = height;
	gd->cursor.hot_x = hot_x;
	gd->cursor.hot_y = hot_y;
	gd->cursor.hidden = pixels == NULL;

	if (pixels != NULL) {
		if (bytes > gd->cursor_bytes || gd->cursor_pixels == NULL) {
			void *n = realloc(gd->cursor_pixels, bytes);

			if (n == NULL) {
				pthread_mutex_unlock(&gd->mtx);
				return;
			}
			gd->cursor_pixels = n;
		}
		memcpy(gd->cursor_pixels, pixels, bytes);
		gd->cursor_bytes = bytes;
	}
	gd->cursor_valid = true;

	gd_send_cursor_locked(gd);
	pthread_mutex_unlock(&gd->mtx);
}

void
gpu_display_unbind(struct gpu_display *gd)
{
	struct gpu_display_hdr hdr;

	if (gd == NULL)
		return;

	hdr.type = GPU_DISPLAY_MSG_UNBIND;
	hdr.len = sizeof(hdr);

	pthread_mutex_lock(&gd->mtx);
	for (size_t i = 0; i < GPU_DISPLAY_MAX_BUFS; i++) {
		if (gd->bufs[i].used && gd->bufs[i].fd >= 0)
			close(gd->bufs[i].fd);
		gd->bufs[i].used = false;
		gd->bufs[i].fd = -1;
	}
	if (gd->cfd >= 0 && !gd_send(gd, &hdr, sizeof(hdr), -1))
		gd_drop_client(gd);
	pthread_mutex_unlock(&gd->mtx);
}
