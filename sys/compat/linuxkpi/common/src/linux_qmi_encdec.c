/*
 * The QMI message encoder and decoder below come from OpenBSD's qwx(4)
 * driver, sys/dev/ic/qwx.c revision 1.146, adapted to the LinuxKPI QMI
 * interface (linux/soc/qcom/qmi.h).
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

#include <sys/param.h>
#include <sys/endian.h>

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/soc/qcom/qmi.h>

#define	QMI_NUM_DATA_TYPES	(QMI_STRING + 1)

static const char *qmi_data_type_name[QMI_NUM_DATA_TYPES] = {
	"EOTI",
	"OPT_FLAG",
	"DATA_LEN",
	"UNSIGNED_1_BYTE",
	"UNSIGNED_2_BYTE",
	"UNSIGNED_4_BYTE",
	"UNSIGNED_8_BYTE",
	"SIGNED_2_BYTE_ENUM",
	"SIGNED_4_BYTE_ENUM",
	"STRUCT",
	"STRING"
};

const struct qmi_elem_info qmi_response_type_v01_ei[] = {
	{
		.data_type	= QMI_SIGNED_2_BYTE_ENUM,
		.elem_len	= 1,
		.elem_size	= sizeof(uint16_t),
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct qmi_response_type_v01, result),
		.ei_array	= NULL,
	},
	{
		.data_type	= QMI_SIGNED_2_BYTE_ENUM,
		.elem_len	= 1,
		.elem_size	= sizeof(uint16_t),
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct qmi_response_type_v01, error),
		.ei_array	= NULL,
	},
	{
		.data_type	= QMI_EOTI,
		.elem_len	= 0,
		.elem_size	= 0,
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= 0,
		.ei_array	= NULL,
	},
};

/*
 * Size of the C structure described by an element info table, as far as
 * the table describes it.  The size of the C structure is not passed in
 * by callers, so this is used to bound accesses to the structure instead.
 * DATA_LEN members are stored as 32-bit values in the C structure.
 */
static size_t
qmi_struct_size(const struct qmi_elem_info *ei)
{
	size_t size = 0, end;

	for (; ei->data_type != QMI_EOTI; ei++) {
		if (ei->data_type == QMI_DATA_LEN)
			end = (size_t)ei->offset + sizeof(uint32_t);
		else
			end = (size_t)ei->offset +
			    (size_t)ei->elem_len * ei->elem_size;
		if (end > size)
			size = end;
	}

	return size;
}

static const struct qmi_elem_info *
qmi_decode_get_elem(const struct qmi_elem_info *ei, uint8_t elem_type)
{
	while (ei->data_type != QMI_EOTI && ei->tlv_type != elem_type)
		ei++;

	return ei;
}

static size_t
qmi_decode_min_elem_size(const struct qmi_elem_info *ei, int nested)
{
	size_t min_size = 0;
	uint32_t nelem = 1;

	/* Variable length arrays may be empty; static arrays may not. */
	if (ei->array_type == VAR_LEN_ARRAY)
		return 0;
	if (ei->array_type == STATIC_ARRAY && ei->data_type != QMI_STRING)
		nelem = ei->elem_len;

	switch (ei->data_type) {
	case QMI_EOTI:
	case QMI_OPT_FLAG:
		break;
	case QMI_DATA_LEN:
		if (ei->elem_size == 1)
			min_size += sizeof(uint8_t);
		else
			min_size += sizeof(uint16_t);
		break;
	case QMI_UNSIGNED_1_BYTE:
	case QMI_UNSIGNED_2_BYTE:
	case QMI_UNSIGNED_4_BYTE:
	case QMI_UNSIGNED_8_BYTE:
	case QMI_SIGNED_2_BYTE_ENUM:
	case QMI_SIGNED_4_BYTE_ENUM:
		min_size += nelem * ei->elem_size;
		break;
	case QMI_STRUCT:
		if (nested > 2) {
			printf("%s: QMI struct element 0x%x with "
			    "data type %s (0x%x) is nested too "
			    "deeply\n", __func__,
			    ei->tlv_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type);
			break;
		}
		if (ei->ei_array == NULL)
			break;
		ei = ei->ei_array;
		while (ei->data_type != QMI_EOTI) {
			min_size += nelem * qmi_decode_min_elem_size(ei,
			    nested + 1);
			ei++;
		}
		break;
	case QMI_STRING:
		/* Strings nested in structs use an in-band length field. */
		if (nested) {
			if (ei->elem_len <= 0xff)
				min_size += sizeof(uint8_t);
			else
				min_size += sizeof(uint16_t);
		}
		break;
	default:
		printf("%s: unhandled data type 0x%x\n", __func__,
		    ei->data_type);
		break;
	}

	return min_size;
}

static int
qmi_decode_tlv_hdr(const struct qmi_elem_info **next_ei,
    uint16_t *actual_size, size_t output_len, const struct qmi_elem_info *ei0,
    const uint8_t *input, size_t input_len)
{
	const uint8_t *p = input;
	size_t remain = input_len;
	uint8_t elem_type;
	uint16_t elem_size = 0;
	const struct qmi_elem_info *ei;

	*next_ei = NULL;
	*actual_size = 0;

	if (remain < 3) {
		printf("%s: QMI message TLV header too short\n", __func__);
		return -1;
	}
	elem_type = *p;
	p++;
	remain--;

	/*
	 * By relying on TLV type information we can skip over EIs which
	 * describe optional elements that have not been encoded.
	 * Such elements will be left at their default value (zero) in
	 * the decoded output struct.
	 * XXX We currently allow elements to appear in any order and
	 * we do not detect duplicates.
	 */
	ei = qmi_decode_get_elem(ei0, elem_type);

	/* The TLV length field is always 16 bits wide. */
	elem_size = (p[0] | (p[1] << 8));
	p += 2;
	remain -= 2;

	if (remain < elem_size) {
		printf("%s: QMI message too short\n", __func__);
		return -1;
	}

	*next_ei = ei;
	*actual_size = elem_size;

	if (ei->data_type == QMI_EOTI) {
		/*
		 * Types below 0x10 are reserved for mandatory elements;
		 * a message holding one we do not know cannot be decoded.
		 */
		if (elem_type < 0x10) {
			printf("%s: unrecognized mandatory QMI element "
			    "type 0x%x\n", __func__, elem_type);
			return -1;
		}
		/* Unrecognized optional QMI element type; will be skipped. */
		return 0;
	}

	/*
	 * Is this an optional element which has been encoded?
	 * If so, use info about this optional element for verification.
	 */
	if (ei->data_type == QMI_OPT_FLAG)
		ei++;

	switch (ei->data_type) {
	case QMI_UNSIGNED_1_BYTE:
	case QMI_UNSIGNED_2_BYTE:
	case QMI_UNSIGNED_4_BYTE:
	case QMI_UNSIGNED_8_BYTE:
	case QMI_SIGNED_2_BYTE_ENUM:
	case QMI_SIGNED_4_BYTE_ENUM:
		if (ei->array_type == VAR_LEN_ARRAY ||
		    elem_size != qmi_decode_min_elem_size(ei, 0)) {
			printf("%s: QMI message element 0x%x "
			    "data type %s (0x%x) with bad size: %u\n",
			    __func__, elem_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type, elem_size);
			return -1;
		}
		break;
	case QMI_DATA_LEN:
		break;
	case QMI_STRING:
	case QMI_STRUCT:
		if (elem_size < qmi_decode_min_elem_size(ei, 0)) {
			printf("%s: QMI message element 0x%x "
			    "data type %s (0x%x) with bad size: %u\n",
			    __func__, elem_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type, elem_size);
			return -1;
		}
		break;
	default:
		printf("%s: unexpected QMI message element "
		    "data type 0x%x\n", __func__, ei->data_type);
		return -1;
	}

	if (ei->offset + ei->elem_size > output_len) {
		printf("%s: QMI message element type 0x%x too large: %u\n",
		    __func__, elem_type, ei->elem_size);
		return -1;
	}

	return 0;
}

static int
qmi_decode_byte(void *output, const struct qmi_elem_info *ei,
    const void *input)
{
	if (ei->elem_size != sizeof(uint8_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	memcpy(output, input, ei->elem_size);
	return 0;
}

static int
qmi_decode_word(void *output, const struct qmi_elem_info *ei,
    const void *input)
{
	uint16_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	memcpy(&val, input, sizeof(val));
	val = le16toh(val);
	memcpy(output, &val, sizeof(val));
	return 0;
}

static int
qmi_decode_dword(void *output, const struct qmi_elem_info *ei,
    const void *input)
{
	uint32_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	memcpy(&val, input, sizeof(val));
	val = le32toh(val);
	memcpy(output, &val, sizeof(val));
	return 0;
}

static int
qmi_decode_qword(void *output, const struct qmi_elem_info *ei,
    const void *input)
{
	uint64_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	memcpy(&val, input, sizeof(val));
	val = le64toh(val);
	memcpy(output, &val, sizeof(val));
	return 0;
}

static int
qmi_decode_datalen(size_t *used, uint32_t *datalen,
    void *output, size_t output_len, const struct qmi_elem_info *ei,
    const uint8_t *input, size_t input_len)
{
	const uint8_t *p = input;
	size_t remain = input_len;

	*datalen = 0;

	if (remain < ei->elem_size) {
		printf("%s: QMI message too short: remain=%zu elem_size=%u\n",
		    __func__, remain, ei->elem_size);
		return -1;
	}

	switch (ei->elem_size) {
	case sizeof(uint8_t):
		*datalen = p[0];
		break;
	case sizeof(uint16_t):
		*datalen = p[0] | (p[1] << 8);
		break;
	default:
		printf("%s: bad datalen element size %u\n",
		    __func__, ei->elem_size);
		return -1;
	}
	*used = ei->elem_size;

	if (ei->offset + sizeof(*datalen) > output_len) {
		printf("%s: QMI message element type 0x%x too large\n",
		    __func__, ei->tlv_type);
		return -1;
	}
	memcpy((uint8_t *)output + ei->offset, datalen, sizeof(*datalen));
	return 0;
}

static int
qmi_decode_string(size_t *used_total,
    void *output, size_t output_len, const struct qmi_elem_info *ei,
    const uint8_t *input, size_t input_len, uint16_t elem_size, int nested)
{
	const uint8_t *p = input;
	uint8_t *s;
	uint16_t len;
	size_t remain = input_len;

	*used_total = 0;

	if (nested) {
		/* Strings nested in structs use an in-band length field. */
		if (ei->elem_len <= 0xff) {
			if (remain == 0) {
				printf("%s: QMI string length header exceeds "
				    "input buffer size\n", __func__);
				return -1;
			}
			len = p[0];
			p++;
			(*used_total)++;
			remain--;
		} else {
			if (remain < 2) {
				printf("%s: QMI string length header exceeds "
				    "input buffer size\n", __func__);
				return -1;
			}
			len = p[0] | (p[1] << 8);
			p += 2;
			*used_total += 2;
			remain -= 2;
		}
	} else
		len = elem_size;

	/* elem_len includes the terminating NUL. */
	if (len >= ei->elem_len) {
		printf("%s: QMI string element of length %u exceeds "
		    "maximum length %u\n", __func__, len, ei->elem_len - 1);
		return -1;
	}
	if (len > remain) {
		printf("%s: QMI string element of length %u exceeds "
		    "input buffer size %zu\n", __func__, len, remain);
		return -1;
	}
	if (len >= output_len) {
		printf("%s: QMI string element of length %u exceeds "
		    "output buffer size %zu\n", __func__, len, output_len);
		return -1;
	}

	memcpy(output, p, len);

	s = output;
	s[len] = '\0';

	*used_total += len;
	return 0;
}

static int
qmi_decode_struct(size_t *used_total,
    void *output, size_t output_len,
    const struct qmi_elem_info *struct_ei,
    const uint8_t *input, size_t input_len,
    int nested)
{
	const struct qmi_elem_info *ei = struct_ei->ei_array;
	const uint8_t *p = input;
	size_t remain = input_len;
	size_t used = 0;

	*used_total = 0;

	if (ei == NULL) {
		printf("%s: QMI struct element 0x%x without element info\n",
		    __func__, struct_ei->tlv_type);
		return -1;
	}

	while (ei->data_type != QMI_EOTI) {
		uint32_t nelem = 1, i;

		if (ei->data_type == QMI_DATA_LEN) {
			uint32_t datalen;

			used = 0;
			if (qmi_decode_datalen(&used, &datalen,
			    output, output_len, ei, p, remain))
				return -1;
			p += used;
			remain -= used;
			*used_total += used;
			ei++;
			if (ei->data_type == QMI_EOTI ||
			    ei->array_type != VAR_LEN_ARRAY) {
				printf("%s: data len not for a var array\n",
				    __func__);
				return -1;
			}
			if (datalen > ei->elem_len) {
				printf("%s: QMI array of %u elements exceeds "
				    "maximum of %u\n", __func__, datalen,
				    ei->elem_len);
				return -1;
			}
			nelem = datalen;
		} else if (ei->array_type == STATIC_ARRAY &&
		    ei->data_type != QMI_STRING) {
			nelem = ei->elem_len;
		} else if (ei->array_type == VAR_LEN_ARRAY) {
			printf("%s: var array without data len\n", __func__);
			return -1;
		}

		if (ei->offset + (size_t)nelem * ei->elem_size > output_len) {
			printf("%s: QMI message struct member element "
			    "type 0x%x too large: %u\n", __func__,
			    ei->tlv_type, ei->elem_size);
			return -1;
		}

		for (i = 0; i < nelem; i++) {
			size_t outoff = ei->offset + (ei->elem_size * i);
			uint8_t *out = (uint8_t *)output + outoff;

			switch (ei->data_type) {
			case QMI_UNSIGNED_1_BYTE:
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
			case QMI_UNSIGNED_8_BYTE:
				if (remain < ei->elem_size) {
					printf("%s: QMI message too short\n",
					    __func__);
					return -1;
				}
				break;
			default:
				break;
			}

			switch (ei->data_type) {
			case QMI_UNSIGNED_1_BYTE:
				if (qmi_decode_byte(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				*used_total += ei->elem_size;
				break;
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
				if (qmi_decode_word(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				*used_total += ei->elem_size;
				break;
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
				if (qmi_decode_dword(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				*used_total += ei->elem_size;
				break;
			case QMI_UNSIGNED_8_BYTE:
				if (qmi_decode_qword(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				*used_total += ei->elem_size;
				break;
			case QMI_STRUCT:
				if (nested > 2) {
					printf("%s: QMI struct element data "
					    "type 0x%x is nested too deeply\n",
					    __func__, ei->data_type);
					return -1;
				}
				used = 0;
				if (qmi_decode_struct(&used,
				    out, ei->elem_size,
				    ei, p, remain, nested + 1))
					return -1;
				remain -= used;
				p += used;
				*used_total += used;
				break;
			case QMI_STRING:
				used = 0;
				if (qmi_decode_string(&used,
				    out, output_len - outoff,
				    ei, p, remain, 0, 1))
					return -1;
				remain -= used;
				p += used;
				*used_total += used;
				break;
			default:
				printf("%s: unhandled QMI struct element "
				    "data type 0x%x\n", __func__,
				    ei->data_type);
				return -1;
			}
		}

		ei++;
	}

	return 0;
}

static int
qmi_decode_msg(void *output, size_t output_len,
    const struct qmi_elem_info *ei0, const uint8_t *input, uint16_t input_len)
{
	const uint8_t *p = input;
	size_t remain = input_len, used;
	const struct qmi_elem_info *ei = ei0;

	if (output_len > 0)
		memset(output, 0, output_len);

	/*
	 * Elements are looked up by TLV type in the whole table, so keep
	 * going until all input has been consumed.
	 */
	while (remain > 0) {
		uint32_t nelem = 1, i;
		uint16_t datalen;

		if (qmi_decode_tlv_hdr(&ei, &datalen, output_len,
		    ei0, p, remain))
			return -1;

		/* Skip unrecognized elements. */
		if (ei->data_type == QMI_EOTI) {
			p += 3 + datalen;
			remain -= 3 + datalen;
			ei = ei0;
			continue;
		}

		/* Set 'valid' flag for optional fields in output struct. */
		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t *pvalid;

			if (ei->offset + ei->elem_size > output_len) {
				printf("%s: QMI message element type 0x%x "
				    "too large: %u\n", __func__,
				    ei->tlv_type, ei->elem_size);
				return -1;
			}

			pvalid = (uint8_t *)output + ei->offset;
			*pvalid = 1;

			ei++;
		}

		p += 3;
		remain -= 3;

		/*
		 * From here on, 'datalen' counts the bytes of this
		 * element which have not been decoded yet.  The TLV
		 * header check made sure that all of them are present.
		 */
		if (ei->data_type == QMI_DATA_LEN) {
			const struct qmi_elem_info *datalen_ei = ei;
			uint8_t elem_type = ei->tlv_type;

			ei++;

			/* Related EIs must have the same type. */
			if (ei->tlv_type != elem_type ||
			    ei->data_type == QMI_EOTI ||
			    ei->array_type != VAR_LEN_ARRAY) {
				printf("%s: unexpected element type 0x%x; "
				    "expected 0x%x\n", __func__,
				    ei->tlv_type, elem_type);
				return -1;
			}

			if (datalen == 0) {
				ei = ei0;
				continue;
			}

			/*
			 * For variable length arrays a one- or two-byte
			 * value follows the header, indicating the number
			 * of elements in the array.
			 */
			used = 0;
			if (qmi_decode_datalen(&used, &nelem,
			    output, output_len, datalen_ei, p, datalen))
				return -1;
			p += used;
			remain -= used;
			/*
			 * Previous datalen value included the total
			 * amount of bytes following the DATALEN TLV
			 * header.
			 */
			datalen -= used;

			if (nelem > ei->elem_len) {
				printf("%s: QMI array of %u elements exceeds "
				    "maximum of %u\n", __func__, nelem,
				    ei->elem_len);
				return -1;
			}
		} else if (ei->array_type == STATIC_ARRAY &&
		    ei->data_type != QMI_STRING) {
			nelem = ei->elem_len;
		}

		if (output_len < ei->offset + (size_t)nelem * ei->elem_size) {
			printf("%s: QMI output buffer too short: "
			    "nelem=%u ei->elem_size=%u\n", __func__,
			    nelem, ei->elem_size);
			return -1;
		}

		for (i = 0; i < nelem; i++) {
			size_t outoff;
			uint8_t *out;

			outoff = ei->offset + (ei->elem_size * i);
			out = (uint8_t *)output + outoff;
			switch (ei->data_type) {
			case QMI_STRUCT:
				used = 0;
				if (qmi_decode_struct(&used,
				    out, ei->elem_size,
				    ei, p, datalen, 0))
					return -1;
				remain -= used;
				p += used;
				datalen -= used;
				break;
			case QMI_STRING:
				used = 0;
				if (nelem != 1 || qmi_decode_string(&used,
				    out, output_len - outoff,
				    ei, p, datalen, datalen, 0))
					return -1;
				remain -= used;
				p += used;
				datalen -= used;
				break;
			case QMI_UNSIGNED_1_BYTE:
				if (datalen < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qmi_decode_byte(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				datalen -= ei->elem_size;
				break;
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
				if (datalen < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qmi_decode_word(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				datalen -= ei->elem_size;
				break;
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
				if (datalen < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qmi_decode_dword(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				datalen -= ei->elem_size;
				break;
			case QMI_UNSIGNED_8_BYTE:
				if (datalen < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qmi_decode_qword(out, ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				datalen -= ei->elem_size;
				break;
			default:
				printf("%s: unhandled QMI message element "
				    "data type 0x%x\n", __func__,
				    ei->data_type);
				return -1;
			}
		}

		/* The element must have used all of its TLV's bytes. */
		if (datalen != 0) {
			printf("%s: QMI message element 0x%x has %u "
			    "trailing bytes\n", __func__, ei->tlv_type,
			    datalen);
			return -1;
		}

		ei = ei0;
	}

	return 0;
}

static int
qmi_encode_datalen(uint8_t *p, uint32_t *datalen,
    const struct qmi_elem_info *ei, const void *input)
{
	memcpy(datalen, (const uint8_t *)input + ei->offset, sizeof(uint32_t));

	if (ei->elem_size == sizeof(uint8_t)) {
		if (*datalen > 0xff) {
			printf("%s: data len %u too large\n", __func__,
			    *datalen);
			return -1;
		}
		if (p != NULL)
			p[0] = (*datalen & 0xff);
	} else if (ei->elem_size == sizeof(uint16_t)) {
		if (*datalen > 0xffff) {
			printf("%s: data len %u too large\n", __func__,
			    *datalen);
			return -1;
		}
		if (p != NULL) {
			p[0] = (*datalen & 0xff);
			p[1] = (*datalen >> 8) & 0xff;
		}
	} else {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	return 0;
}

static int
qmi_encode_byte(uint8_t *p, const struct qmi_elem_info *ei, const void *input,
    int i)
{
	if (ei->elem_size != sizeof(uint8_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(p, (const uint8_t *)input + ei->offset + (i * ei->elem_size),
	    ei->elem_size);
	return 0;
}

static int
qmi_encode_word(uint8_t *p, const struct qmi_elem_info *ei, const void *input,
    int i)
{
	uint16_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, (const uint8_t *)input + ei->offset + (i * ei->elem_size),
	    ei->elem_size);
	val = htole16(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

static int
qmi_encode_dword(uint8_t *p, const struct qmi_elem_info *ei, const void *input,
    int i)
{
	uint32_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, (const uint8_t *)input + ei->offset + (i * ei->elem_size),
	    ei->elem_size);
	val = htole32(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

static int
qmi_encode_qword(uint8_t *p, const struct qmi_elem_info *ei, const void *input,
    int i)
{
	uint64_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, (const uint8_t *)input + ei->offset + (i * ei->elem_size),
	    ei->elem_size);
	val = htole64(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

/*
 * Encode a string.  Strings nested in structs are preceded by an in-band
 * length field; a string of its own TLV takes the length from the TLV
 * header.  If p is NULL, only compute the encoded length.  Otherwise,
 * out_len is the space available at p.
 */
static int
qmi_encode_string(uint8_t *p, size_t out_len, size_t *encoded_len,
    const struct qmi_elem_info *string_ei, const void *input, int nested)
{
	size_t len, hdrlen = 0;

	*encoded_len = 0;

	/* elem_len is the size of the char array, including the NUL. */
	len = strnlen(input, string_ei->elem_len);
	if (len >= string_ei->elem_len) {
		printf("%s: QMI string is not NUL-terminated\n", __func__);
		return -1;
	}

	if (nested)
		hdrlen = (string_ei->elem_len <= 0xff) ? 1 : 2;

	if (p) {
		if (hdrlen + len > out_len) {
			printf("%s: QMI message buffer too short\n", __func__);
			return -1;
		}
		if (hdrlen == 1)
			p[0] = len & 0xff;
		else if (hdrlen == 2) {
			p[0] = len & 0xff;
			p[1] = (len >> 8) & 0xff;
		}
		memcpy(p + hdrlen, input, len);
	}

	*encoded_len = hdrlen + len;
	return 0;
}

/*
 * Encode a struct, which has no TLV headers of its own.  input_len is
 * the size of the C struct at input.  If p is NULL, only compute the
 * encoded length.  Otherwise, out_len is the space available at p.
 */
static int
qmi_encode_struct(uint8_t *p, size_t out_len, size_t *encoded_len,
    const struct qmi_elem_info *struct_ei, const void *input, size_t input_len,
    int nested)
{
	const struct qmi_elem_info *ei = struct_ei->ei_array;
	size_t remain = out_len;

	*encoded_len = 0;

	if (ei == NULL) {
		printf("%s: QMI struct element 0x%x without element info\n",
		    __func__, struct_ei->tlv_type);
		return -1;
	}
	if (nested > 2) {
		printf("%s: QMI struct element is nested too deeply\n",
		    __func__);
		return -1;
	}

	while (ei->data_type != QMI_EOTI) {
		uint32_t nelem = 1, i;

		if (ei->offset + sizeof(uint8_t) > input_len) {
			printf("%s: bad input buffer offset\n", __func__);
			return -1;
		}

		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			memcpy(&do_encode, (const uint8_t *)input + ei->offset,
			    sizeof(uint8_t));
			ei++; /* Advance to element we might have to encode. */
			if (ei->data_type == QMI_OPT_FLAG ||
			    ei->data_type == QMI_EOTI) {
				printf("%s: bad optional flag element\n",
				    __func__);
				return -1;
			}
			if (!do_encode) {
				/* The element will not be encoded. Skip it. */
				tlv_type = ei->tlv_type;
				while (ei->data_type != QMI_EOTI &&
				    ei->tlv_type == tlv_type)
					ei++;
				continue;
			}
		}

		if (ei->data_type == QMI_DATA_LEN) {
			uint32_t datalen = 0;

			if (ei->offset + sizeof(uint32_t) > input_len) {
				printf("%s: bad input buffer offset\n",
				    __func__);
				return -1;
			}
			if (p != NULL && ei->elem_size > remain) {
				printf("%s: QMI message buffer too short\n",
				    __func__);
				return -1;
			}
			if (qmi_encode_datalen(p, &datalen, ei, input))
				return -1;
			remain -= (p != NULL) ? ei->elem_size : 0;
			if (p != NULL)
				p += ei->elem_size;
			*encoded_len += ei->elem_size;
			ei++;
			if (ei->data_type == QMI_EOTI ||
			    ei->array_type != VAR_LEN_ARRAY) {
				printf("%s: data len not for a var array\n",
				    __func__);
				return -1;
			}
			if (datalen > ei->elem_len) {
				printf("%s: QMI array of %u elements exceeds "
				    "maximum of %u\n", __func__, datalen,
				    ei->elem_len);
				return -1;
			}
			nelem = datalen;
		} else if (ei->array_type == STATIC_ARRAY &&
		    ei->data_type != QMI_STRING) {
			nelem = ei->elem_len;
		} else if (ei->array_type == VAR_LEN_ARRAY) {
			printf("%s: var array without data len\n", __func__);
			return -1;
		}

		if (ei->data_type == QMI_STRING) {
			if (ei->offset + (size_t)ei->elem_len > input_len) {
				printf("%s: bad input buffer offset\n",
				    __func__);
				return -1;
			}
		} else if (ei->offset + (size_t)nelem * ei->elem_size >
		    input_len) {
			printf("%s: bad input buffer offset\n", __func__);
			return -1;
		}

		for (i = 0; i < nelem; i++) {
			size_t used = 0;
			size_t inoff = ei->offset + (i * ei->elem_size);

			switch (ei->data_type) {
			case QMI_UNSIGNED_1_BYTE:
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
			case QMI_UNSIGNED_8_BYTE:
				if (p != NULL && ei->elem_size > remain) {
					printf("%s: QMI message buffer too "
					    "short\n", __func__);
					return -1;
				}
				used = ei->elem_size;
				break;
			default:
				break;
			}

			switch (ei->data_type) {
			case QMI_UNSIGNED_1_BYTE:
				if (qmi_encode_byte(p, ei, input, i))
					return -1;
				break;
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
				if (qmi_encode_word(p, ei, input, i))
					return -1;
				break;
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
				if (qmi_encode_dword(p, ei, input, i))
					return -1;
				break;
			case QMI_UNSIGNED_8_BYTE:
				if (qmi_encode_qword(p, ei, input, i))
					return -1;
				break;
			case QMI_STRUCT:
				if (qmi_encode_struct(p, remain, &used, ei,
				    (const uint8_t *)input + inoff,
				    ei->elem_size, nested + 1))
					return -1;
				break;
			case QMI_STRING:
				if (qmi_encode_string(p, remain, &used, ei,
				    (const uint8_t *)input + inoff, 1))
					return -1;
				break;
			default:
				printf("%s: unhandled QMI struct element "
				    "type %d\n", __func__, ei->data_type);
				return -1;
			}

			if (p != NULL) {
				remain -= used;
				p += used;
			}
			*encoded_len += used;
		}
		ei++;
	}

	return 0;
}

/*
 * Encode the value of the top-level TLV element described by ei,
 * which has been advanced past a QMI_OPT_FLAG, if any.  Returns the
 * element info following the element in *next_ei.  If p is NULL, only
 * compute the encoded length.  Otherwise, out_len is the space at p.
 */
static int
qmi_encode_tlv_value(uint8_t *p, size_t out_len, size_t *encoded_len,
    const struct qmi_elem_info **next_ei, const struct qmi_elem_info *ei,
    const void *input, size_t input_len)
{
	size_t remain = out_len;
	uint32_t datalen = 0;
	uint32_t nelem = 1, i;

	*encoded_len = 0;

	if (ei->data_type == QMI_DATA_LEN) {
		if (ei->offset + sizeof(uint32_t) > input_len) {
			printf("%s: bad input buffer offset\n", __func__);
			return -1;
		}
		if (p != NULL && ei->elem_size > remain) {
			printf("%s: QMI message buffer too short\n", __func__);
			return -1;
		}
		if (qmi_encode_datalen(p, &datalen, ei, input))
			return -1;
		if (p != NULL) {
			p += ei->elem_size;
			remain -= ei->elem_size;
		}
		*encoded_len += ei->elem_size;
		ei++;
		if (ei->data_type == QMI_EOTI ||
		    ei->array_type != VAR_LEN_ARRAY ||
		    ei->tlv_type != (ei - 1)->tlv_type) {
			printf("%s: data len not for a var array\n",
			    __func__);
			return -1;
		}
		if (datalen > ei->elem_len) {
			printf("%s: QMI array of %u elements exceeds "
			    "maximum of %u\n", __func__, datalen, ei->elem_len);
			return -1;
		}
		nelem = datalen;
	} else if (ei->array_type == STATIC_ARRAY &&
	    ei->data_type != QMI_STRING) {
		nelem = ei->elem_len;
	} else if (ei->array_type == VAR_LEN_ARRAY) {
		printf("%s: var array without data len\n", __func__);
		return -1;
	}

	if (ei->data_type == QMI_STRING) {
		if (ei->offset + (size_t)ei->elem_len > input_len) {
			printf("%s: bad input buffer offset\n", __func__);
			return -1;
		}
	} else if (ei->offset + (size_t)nelem * ei->elem_size > input_len) {
		printf("%s: bad input buffer offset at element 0x%x "
		    "data type 0x%x\n",
		    __func__, ei->tlv_type, ei->data_type);
		return -1;
	}

	for (i = 0; i < nelem; i++) {
		size_t used = 0;
		size_t inoff = ei->offset + (i * ei->elem_size);

		switch (ei->data_type) {
		case QMI_UNSIGNED_1_BYTE:
		case QMI_UNSIGNED_2_BYTE:
		case QMI_SIGNED_2_BYTE_ENUM:
		case QMI_UNSIGNED_4_BYTE:
		case QMI_SIGNED_4_BYTE_ENUM:
		case QMI_UNSIGNED_8_BYTE:
			if (p != NULL && ei->elem_size > remain) {
				printf("%s: QMI message buffer too short\n",
				    __func__);
				return -1;
			}
			used = ei->elem_size;
			break;
		default:
			break;
		}

		switch (ei->data_type) {
		case QMI_UNSIGNED_1_BYTE:
			if (qmi_encode_byte(p, ei, input, i))
				return -1;
			break;
		case QMI_UNSIGNED_2_BYTE:
		case QMI_SIGNED_2_BYTE_ENUM:
			if (qmi_encode_word(p, ei, input, i))
				return -1;
			break;
		case QMI_UNSIGNED_4_BYTE:
		case QMI_SIGNED_4_BYTE_ENUM:
			if (qmi_encode_dword(p, ei, input, i))
				return -1;
			break;
		case QMI_UNSIGNED_8_BYTE:
			if (qmi_encode_qword(p, ei, input, i))
				return -1;
			break;
		case QMI_STRUCT:
			if (qmi_encode_struct(p, remain, &used, ei,
			    (const uint8_t *)input + inoff, ei->elem_size, 0))
				return -1;
			break;
		case QMI_STRING:
			if (qmi_encode_string(p, remain, &used, ei,
			    (const uint8_t *)input + inoff, 0))
				return -1;
			break;
		default:
			printf("%s: unhandled QMI message element type %d\n",
			    __func__, ei->data_type);
			return -1;
		}

		if (p != NULL) {
			remain -= used;
			p += used;
		}
		*encoded_len += used;
	}

	if (*encoded_len > 0xffff) {
		printf("%s: QMI element 0x%x too large: %zu\n", __func__,
		    ei->tlv_type, *encoded_len);
		return -1;
	}

	*next_ei = ei + 1;
	return 0;
}

static int
qmi_encode_msg(uint8_t **encoded_msg, size_t *encoded_len, int type,
    uint16_t txn_id, uint16_t msg_id, size_t msg_len,
    const struct qmi_elem_info *ei, const void *input, size_t input_len)
{
	const struct qmi_elem_info *ei0 = ei;
	struct qmi_header hdr;
	size_t remain;
	uint8_t *p, *op;
	int error = -EINVAL;

	*encoded_msg = NULL;
	*encoded_len = 0;

	/* First pass: Determine length of encoded message. */
	while (ei->data_type != QMI_EOTI) {
		size_t value_len = 0;

		/*
		 * OPT_FLAG determines whether the next element
		 * should be considered for encoding.
		 */
		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			if (ei->offset + sizeof(uint8_t) > input_len) {
				printf("%s: bad input buffer offset\n",
				    __func__);
				goto err;
			}
			memcpy(&do_encode, (const uint8_t *)input + ei->offset,
			    sizeof(uint8_t));
			ei++; /* Advance to element we might have to encode. */
			if (ei->data_type == QMI_OPT_FLAG ||
			    ei->data_type == QMI_EOTI) {
				printf("%s: bad optional element\n", __func__);
				goto err;
			}
			if (!do_encode) {
				/* The element will not be encoded. Skip it. */
				tlv_type = ei->tlv_type;
				while (ei->data_type != QMI_EOTI &&
				    ei->tlv_type == tlv_type)
					ei++;
				continue;
			}
		}

		if (qmi_encode_tlv_value(NULL, 0, &value_len, &ei, ei,
		    input, input_len))
			goto err;
		*encoded_len += 3 + value_len; /* type, length, value */
	}

	if (*encoded_len > msg_len) {
		printf("%s: QMI message 0x%x of %zu bytes exceeds "
		    "maximum length %zu\n", __func__, msg_id,
		    *encoded_len, msg_len);
		error = -EMSGSIZE;
		goto err;
	}

	*encoded_len += sizeof(hdr);
	*encoded_msg = kzalloc(*encoded_len, GFP_KERNEL);
	if (*encoded_msg == NULL) {
		*encoded_len = 0;
		return -ENOMEM;
	}

	hdr.type = type;
	hdr.txn_id = htole16(txn_id);
	hdr.msg_id = htole16(msg_id);
	hdr.msg_len = htole16(*encoded_len - sizeof(hdr));
	memcpy(*encoded_msg, &hdr, sizeof(hdr));

	/* Second pass: Encode the message. */
	ei = ei0;
	p = *encoded_msg + sizeof(hdr);
	remain = *encoded_len - sizeof(hdr);
	while (ei->data_type != QMI_EOTI) {
		const struct qmi_elem_info *tlv_ei;
		size_t value_len = 0;

		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			memcpy(&do_encode, (const uint8_t *)input + ei->offset,
			    sizeof(uint8_t));
			ei++; /* Advance to element we might have to encode. */
			if (ei->data_type == QMI_OPT_FLAG ||
			    ei->data_type == QMI_EOTI) {
				printf("%s: bad optional flag element\n",
				    __func__);
				goto err;
			}
			if (!do_encode) {
				/* The element will not be encoded. Skip it. */
				tlv_type = ei->tlv_type;
				while (ei->data_type != QMI_EOTI &&
				    ei->tlv_type == tlv_type)
					ei++;
				continue;
			}
		}

		if (3 > remain) {
			printf("%s: QMI message buffer too short\n", __func__);
			goto err;
		}

		/* 3 bytes of type-length-value header, remember for later */
		op = p;
		p += 3;
		remain -= 3;
		tlv_ei = ei;

		if (qmi_encode_tlv_value(p, remain, &value_len, &ei, ei,
		    input, input_len))
			goto err;
		p += value_len;
		remain -= value_len;

		op[0] = tlv_ei->tlv_type;
		op[1] = value_len & 0xff;
		op[2] = (value_len >> 8) & 0xff;
	}

	if (remain != 0) {
		printf("%s: QMI message length changed while encoding\n",
		    __func__);
		goto err;
	}

	return 0;
err:
	kfree(*encoded_msg);
	*encoded_msg = NULL;
	*encoded_len = 0;
	return error;
}

void *
qmi_encode_message(int type, unsigned int msg_id, size_t *len,
    unsigned int txn_id, const struct qmi_elem_info *ei, const void *c_struct)
{
	uint8_t *msg;
	size_t encoded_len, input_len;
	int error;

	if (len == NULL || ei == NULL)
		return ERR_PTR(-EINVAL);
	input_len = qmi_struct_size(ei);
	if (c_struct == NULL && input_len != 0)
		return ERR_PTR(-EINVAL);

	error = qmi_encode_msg(&msg, &encoded_len, type, txn_id, msg_id,
	    *len, ei, c_struct, input_len);
	if (error != 0)
		return ERR_PTR(error);

	*len = encoded_len;
	return msg;
}

int
qmi_decode_message(const void *buf, size_t len,
    const struct qmi_elem_info *ei, void *c_struct)
{
	struct qmi_header hdr;
	size_t output_len;
	uint16_t msg_len;

	if (buf == NULL || ei == NULL)
		return -EINVAL;
	output_len = qmi_struct_size(ei);
	if (c_struct == NULL && output_len != 0)
		return -EINVAL;

	if (len < sizeof(hdr)) {
		printf("%s: QMI message too short\n", __func__);
		return -EINVAL;
	}
	memcpy(&hdr, buf, sizeof(hdr));
	msg_len = le16toh(hdr.msg_len);
	if (sizeof(hdr) + msg_len != len) {
		printf("%s: bad QMI message length %u\n", __func__, msg_len);
		return -EINVAL;
	}

	if (qmi_decode_msg(c_struct, output_len, ei,
	    (const uint8_t *)buf + sizeof(hdr), msg_len))
		return -EINVAL;

	return 0;
}
