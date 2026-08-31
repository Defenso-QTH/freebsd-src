/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Hans Rosenfeld
 * Author: Hans Rosenfeld <rosenfeld@grumpf.hope-2000.org>
 */

#include <sys/types.h>
#include <sys/endian.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <malloc_np.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "config.h"
#include "tpm_device.h"
#include "tpm_emul.h"

struct tpm_swtpm {
	int fd;
	int ctrl_fd;
};

/*
 * swtpm's control channel, as much of it as is needed to reinitialise the
 * TPM.  See include/swtpm/tpm_ioctl.h in the swtpm distribution.
 *
 * A command is a big-endian uint32 followed by that command's request body;
 * the reply is the response body alone.  CMD_INIT takes a uint32 of flags and
 * answers with a uint32 TPM result code.
 */
#define	SWTPM_CMD_INIT		2

struct swtpm_init_req {
	uint32_t cmd;
	uint32_t init_flags;
} __packed;

/*
 * Connect a second socket to swtpm's control channel and tell it to
 * reinitialise the TPM.
 *
 * swtpm answers CMD_INIT by shutting the TPM down and starting it again,
 * which runs the startup its --flags selected: with startup-clear the PCRs
 * come up cleared, exactly as they would on a cold boot of real hardware.
 *
 * That matters because bhyve exits when the guest reboots and is then started
 * again against the same long-lived swtpm.  Without this the TPM still holds
 * the previous boot's PCR values, firmware measures on top of them, and a
 * guest that sealed anything against those PCRs -- BitLocker with secure boot
 * is the common case -- cannot unseal it and asks for a recovery key on every
 * reboot.  The alternative is to restart swtpm alongside every boot, which
 * needs the two lifecycles kept in step from the outside.
 *
 * Resetting the emulated TPM this way is what the control channel is for,
 * and is how other VMMs drive it; the protocol itself is swtpm's, defined
 * in the header cited above.
 */
static int
tpm_swtpm_ctrl_init(struct tpm_swtpm *tpm, const char *ctrl_path)
{
	struct sockaddr_un ctrl_addr;
	struct swtpm_init_req req;
	uint32_t res;
	ssize_t len;

	tpm->ctrl_fd = socket(PF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (tpm->ctrl_fd < 0) {
		warnx("%s: unable to open tpm control socket", __func__);
		return (ENOENT);
	}

	bzero(&ctrl_addr, sizeof (ctrl_addr));
	ctrl_addr.sun_family = AF_UNIX;
	strlcpy(ctrl_addr.sun_path, ctrl_path, sizeof (ctrl_addr.sun_path) - 1);

	if (connect(tpm->ctrl_fd, (struct sockaddr *)&ctrl_addr,
	    sizeof (ctrl_addr)) == -1) {
		warnx("%s: unable to connect to tpm control socket \"%s\"",
		    __func__, ctrl_path);
		return (ENOENT);
	}

	req.cmd = htobe32(SWTPM_CMD_INIT);
	req.init_flags = htobe32(0);

	len = send(tpm->ctrl_fd, &req, sizeof (req), MSG_NOSIGNAL);
	if (len != (ssize_t)sizeof (req)) {
		warnx("%s: failed to send CMD_INIT (sent %zd of %zu)", __func__,
		    len, sizeof (req));
		return (EFAULT);
	}

	len = recv(tpm->ctrl_fd, &res, sizeof (res), MSG_WAITALL);
	if (len != (ssize_t)sizeof (res)) {
		warnx("%s: no reply to CMD_INIT (read %zd of %zu)", __func__,
		    len, sizeof (res));
		return (EFAULT);
	}

	if (be32toh(res) != 0) {
		warnx("%s: swtpm failed to initialise the TPM: 0x%x", __func__,
		    be32toh(res));
		return (EIO);
	}

	return (0);
}

struct tpm_resp_hdr {
	uint16_t tag;
	uint32_t len;
	uint32_t errcode;
} __packed;

static int
tpm_swtpm_init(void **sc, nvlist_t *nvl)
{
	struct tpm_swtpm *tpm;
	const char *path, *ctrl_path;
	struct sockaddr_un tpm_addr;
	int error;

	tpm = calloc(1, sizeof (struct tpm_swtpm));
	if (tpm == NULL) {
		warnx("%s: failed to allocate tpm_swtpm", __func__);
		return (ENOMEM);
	}
	tpm->ctrl_fd = -1;

	path = get_config_value_node(nvl, "path");
	if (path == NULL) {
		warnx("%s: no socket path specified", __func__);
		return (ENOENT);
	}

	tpm->fd = socket(PF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (tpm->fd < 0) {
		warnx("%s: unable to open tpm socket", __func__);
		return (ENOENT);
	}

	bzero(&tpm_addr, sizeof (tpm_addr));
	tpm_addr.sun_family = AF_UNIX;
	strlcpy(tpm_addr.sun_path, path, sizeof (tpm_addr.sun_path) - 1);

	if (connect(tpm->fd, (struct sockaddr *)&tpm_addr, sizeof (tpm_addr)) ==
	    -1) {
		warnx("%s: unable to connect to tpm socket \"%s\"", __func__,
		    path);
		return (ENOENT);
	}

	/*
	 * Optional: without a control socket the TPM is used exactly as it is
	 * found, which is the behaviour of every existing configuration.
	 */
	ctrl_path = get_config_value_node(nvl, "ctrl_path");
	if (ctrl_path != NULL) {
		error = tpm_swtpm_ctrl_init(tpm, ctrl_path);
		if (error != 0)
			return (error);
	}

	*sc = tpm;

	return (0);
}

static int
tpm_swtpm_execute_cmd(void *sc, void *cmd, uint32_t cmd_size, void *rsp,
    uint32_t rsp_size)
{
	struct tpm_swtpm *tpm;
	ssize_t len;

	if (rsp_size < (ssize_t)sizeof(struct tpm_resp_hdr)) {
		warn("%s: rsp_size of %u is too small", __func__, rsp_size);
		return (EINVAL);
	}

	tpm = sc;

	len = send(tpm->fd, cmd, cmd_size, MSG_NOSIGNAL|MSG_DONTWAIT);
	if (len == -1)
		err(1, "%s: cmd send failed, is swtpm running?", __func__);
	if (len != cmd_size) {
		warn("%s: cmd write failed (bytes written: %zd / %d)", __func__,
		    len, cmd_size);
		return (EFAULT);
	}

	len = recv(tpm->fd, rsp, rsp_size, 0);
	if (len == -1)
		err(1, "%s: rsp recv failed, is swtpm running?", __func__);
	if (len < (ssize_t)sizeof(struct tpm_resp_hdr)) {
		warn("%s: rsp read failed (bytes read: %zd / %d)", __func__,
		    len, rsp_size);
		return (EFAULT);
	}

	return (0);
}

static void
tpm_swtpm_deinit(void *sc)
{
	struct tpm_swtpm *tpm;

	tpm = sc;
	if (tpm == NULL)
		return;

	if (tpm->fd >= 0)
		close(tpm->fd);
	if (tpm->ctrl_fd >= 0)
		close(tpm->ctrl_fd);

	free(tpm);
}

static const struct tpm_emul tpm_emul_swtpm = {
	.name = "swtpm",
	.init = tpm_swtpm_init,
	.deinit = tpm_swtpm_deinit,
	.execute_cmd = tpm_swtpm_execute_cmd,
};
TPM_EMUL_SET(tpm_emul_swtpm);
