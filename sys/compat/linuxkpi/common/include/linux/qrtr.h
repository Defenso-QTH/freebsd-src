/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022-2023 Bjoern A. Zeeb
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


#ifndef	_LINUXKPI_LINUX_QRTR_H
#define	_LINUXKPI_LINUX_QRTR_H

/* Qualcomm IPC Router (QRTR) */

#include <sys/socket.h>

#define	QRTR_NODE_BCAST		0xffffffffu
#define	QRTR_PORT_CTRL		0xfffffffeu

struct sockaddr_qrtr {
	sa_family_t		sq_family;
	uint32_t		sq_node;
	uint32_t		sq_port;
};

enum qrtr_pkt_type {
	QRTR_TYPE_DATA		= 1,
	QRTR_TYPE_HELLO		= 2,
	QRTR_TYPE_BYE		= 3,
	QRTR_TYPE_NEW_SERVER	= 4,
	QRTR_TYPE_DEL_SERVER	= 5,
	QRTR_TYPE_DEL_CLIENT	= 6,
	QRTR_TYPE_RESUME_TX	= 7,
	QRTR_TYPE_EXIT		= 8,
	QRTR_TYPE_PING		= 9,
	QRTR_TYPE_NEW_LOOKUP	= 10,
	QRTR_TYPE_DEL_LOOKUP	= 11,
};

struct qrtr_ctrl_pkt {
	__le32			cmd;

	union {
		struct {
			__le32	service;
			__le32	instance;
			__le32	node;
			__le32	port;
		} server;

		struct {
			__le32	node;
			__le32	port;
		} client;
	};
} __packed;

#endif	/* _LINUXKPI_LINUX_QRTR_H */
