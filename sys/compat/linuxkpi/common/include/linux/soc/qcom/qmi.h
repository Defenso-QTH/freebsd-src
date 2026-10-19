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


#ifndef	_LINUXKPI_LINUX_SOC_QCOM_QMI_H
#define	_LINUXKPI_LINUX_SOC_QCOM_QMI_H

/*
 * QMI (Qualcomm MSM Interface) client helpers, on top of the QRTR router
 * over MHI in linux_qrtr.c.  The interface follows Linux; there are no
 * sockets underneath, a handle is bound to a QRTR port directly.
 */

#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/qrtr.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct qmi_header {
	uint8_t					type;
	__le16					txn_id;
	__le16					msg_id;
	__le16					msg_len;
} __packed;

#define	QMI_REQUEST				0
#define	QMI_RESPONSE				2
#define	QMI_INDICATION				4

#define	QMI_COMMON_TLV_TYPE			0

enum qmi_elem_type {
	QMI_EOTI,
	QMI_OPT_FLAG,
	QMI_DATA_LEN,
	QMI_UNSIGNED_1_BYTE,
	QMI_UNSIGNED_2_BYTE,
	QMI_UNSIGNED_4_BYTE,
	QMI_UNSIGNED_8_BYTE,
	QMI_SIGNED_2_BYTE_ENUM,
	QMI_SIGNED_4_BYTE_ENUM,
	QMI_STRUCT,
	QMI_STRING,
};

enum qmi_array_type {
	NO_ARRAY,
	STATIC_ARRAY,
	VAR_LEN_ARRAY,
};

struct qmi_elem_info {
	enum qmi_elem_type			data_type;
	uint32_t				elem_len;
	uint32_t				elem_size;
	enum qmi_array_type			array_type;
	uint8_t					tlv_type;
	uint32_t				offset;
	const struct qmi_elem_info		*ei_array;
};

#define	QMI_RESULT_SUCCESS_V01			0
#define	QMI_RESULT_FAILURE_V01			1

#define	QMI_ERR_NONE_V01			0
#define	QMI_ERR_MALFORMED_MSG_V01		1
#define	QMI_ERR_NO_MEMORY_V01			2
#define	QMI_ERR_INTERNAL_V01			3
#define	QMI_ERR_CLIENT_IDS_EXHAUSTED_V01	5
#define	QMI_ERR_INVALID_ID_V01			41
#define	QMI_ERR_ENCODING_V01			58
#define	QMI_ERR_DISABLED_V01			69
#define	QMI_ERR_INCOMPATIBLE_STATE_V01		90
#define	QMI_ERR_NOT_SUPPORTED_V01		94

struct qmi_response_type_v01 {
	uint16_t				result;
	uint16_t				error;
};

extern const struct qmi_elem_info qmi_response_type_v01_ei[];

struct qmi_service {
	unsigned int				service;
	unsigned int				version;
	unsigned int				instance;

	unsigned int				node;
	unsigned int				port;

	void					*priv;
	struct list_head			list_node;
};

struct qmi_handle;
struct lkpi_qrtr_port;

struct qmi_ops {
	int	(*new_server)(struct qmi_handle *, struct qmi_service *);
	void	(*del_server)(struct qmi_handle *, struct qmi_service *);
	void	(*net_reset)(struct qmi_handle *);
	void	(*msg_handler)(struct qmi_handle *, struct sockaddr_qrtr *,
		    const void *, size_t);
	void	(*bye)(struct qmi_handle *, unsigned int);
	void	(*del_client)(struct qmi_handle *, unsigned int, unsigned int);
};

struct qmi_txn {
	struct qmi_handle			*qmi;

	uint16_t				id;

	struct mutex				lock;
	struct completion			completion;
	int					result;

	const struct qmi_elem_info		*ei;
	void					*dest;
};

struct qmi_msg_handler {
	unsigned int				type;
	unsigned int				msg_id;

	const struct qmi_elem_info		*ei;

	size_t					decoded_size;
	void	(*fn)(struct qmi_handle *, struct sockaddr_qrtr *,
		    struct qmi_txn *, const void *);
};

struct qmi_handle {
	/* The QRTR port of the handle; passed to kernel_connect(). */
	struct lkpi_qrtr_port			*sock;
	struct mutex				sock_lock;

	struct sockaddr_qrtr			sq;

	struct list_head			lookups;
	struct list_head			lookup_results;
	struct list_head			services;

	struct qmi_ops				ops;

	struct idr				txns;
	struct mutex				txn_lock;

	const struct qmi_msg_handler		*handlers;
};

int	qmi_add_lookup(struct qmi_handle *, unsigned int, unsigned int,
	    unsigned int);

int	qmi_handle_init(struct qmi_handle *, size_t, const struct qmi_ops *,
	    const struct qmi_msg_handler *);
void	qmi_handle_release(struct qmi_handle *);

ssize_t	qmi_send_request(struct qmi_handle *, struct sockaddr_qrtr *,
	    struct qmi_txn *, int, size_t, const struct qmi_elem_info *,
	    const void *);

void	*qmi_encode_message(int, unsigned int, size_t *, unsigned int,
	    const struct qmi_elem_info *, const void *);
int	qmi_decode_message(const void *, size_t, const struct qmi_elem_info *,
	    void *);

int	qmi_txn_init(struct qmi_handle *, struct qmi_txn *,
	    const struct qmi_elem_info *, void *);
int	qmi_txn_wait(struct qmi_txn *, unsigned long);
void	qmi_txn_cancel(struct qmi_txn *);

#endif	/* _LINUXKPI_LINUX_SOC_QCOM_QMI_H */
