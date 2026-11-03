/*	$OpenBSD: qwx.c,v 1.146 2026/10/02 11:24:18 stsp Exp $	*/

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

const char *qmi_data_type_name[QMI_NUM_DATA_TYPES] = {
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

const struct qmi_elem_info *
qwx_qmi_decode_get_elem(const struct qmi_elem_info *ei, uint8_t elem_type)
{
	while (ei->data_type != QMI_EOTI && ei->tlv_type != elem_type)
		ei++;

	DNPRINTF(QWX_D_QMI, "%s: found elem 0x%x data type 0x%x\n", __func__,
	    ei->tlv_type, ei->data_type);
	return ei;
}

size_t
qwx_qmi_decode_min_elem_size(const struct qmi_elem_info *ei, int nested)
{
	size_t min_size = 0;

	switch (ei->data_type) {
	case QMI_EOTI:
	case QMI_OPT_FLAG:
		break;
	case QMI_DATA_LEN:
		if (ei->elem_len == 1)
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
		min_size += ei->elem_len * ei->elem_size;
		break;
	case QMI_STRUCT:
		if (nested > 2) {
			printf("%s: QMI struct element 0x%x with "
			    "data type %s (0x%x) is nested too "
			    "deeply\n", __func__,
			    ei->tlv_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type);
		}
		ei = ei->ei_array;
		while (ei->data_type != QMI_EOTI) {
			min_size += qwx_qmi_decode_min_elem_size(ei,
			    nested + 1);
			ei++;
		}
		break;
	case QMI_STRING:
		min_size += 1;
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

int
qwx_qmi_decode_tlv_hdr(struct qwx_softc *sc,
    const struct qmi_elem_info **next_ei, uint16_t *actual_size,
    size_t output_len, const struct qmi_elem_info *ei0,
    uint8_t *input, size_t input_len)
{
	uint8_t *p = input;
	size_t remain = input_len;
	uint8_t elem_type;
	uint16_t elem_size = 0;
	const struct qmi_elem_info *ei;

	*next_ei = NULL;
	*actual_size = 0;

	if (remain < 3) {
		printf("%s: QMI message TLV header too short\n",
		   sc->sc_dev.dv_xname);
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
	ei = qwx_qmi_decode_get_elem(ei0, elem_type);

	DNPRINTF(QWX_D_QMI,
	    "%s: decoding element 0x%x with data type %s (0x%x)\n",
	    __func__, elem_type, qmi_data_type_name[ei->data_type],
	    ei->data_type);

	if (remain < 2) {
		printf("%s: QMI message too short\n", sc->sc_dev.dv_xname);
		return -1;
	}

	if (ei->data_type == QMI_DATA_LEN && ei->elem_len == 1) {
		elem_size = p[0];
		p++;
		remain--;
	} else {
		elem_size = (p[0] | (p[1] << 8));
		p += 2;
		remain -= 2;
	}

	*next_ei = ei;
	*actual_size = elem_size;

	if (ei->data_type == QMI_EOTI) {
		DNPRINTF(QWX_D_QMI,
		    "%s: unrecognized QMI element type 0x%x size %u\n",
		    sc->sc_dev.dv_xname, elem_type, elem_size);
		return 0;
	}

	/*
	 * Is this an optional element which has been encoded?
	 * If so, use info about this optional element for verification.
	 */
	if (ei->data_type == QMI_OPT_FLAG)
		ei++;

	DNPRINTF(QWX_D_QMI, "%s: ei->size %u, actual size %u\n", __func__,
	    ei->elem_size, *actual_size);

	switch (ei->data_type) {
	case QMI_UNSIGNED_1_BYTE:
	case QMI_UNSIGNED_2_BYTE:
	case QMI_UNSIGNED_4_BYTE:
	case QMI_UNSIGNED_8_BYTE:
	case QMI_SIGNED_2_BYTE_ENUM:
	case QMI_SIGNED_4_BYTE_ENUM:
		if (elem_size != ei->elem_size) {
			printf("%s: QMI message element 0x%x "
			    "data type %s (0x%x) with bad size: %u\n",
			    sc->sc_dev.dv_xname, elem_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type, elem_size);
			return -1;
		}
		break;
	case QMI_DATA_LEN:
		break;
	case QMI_STRING:
	case QMI_STRUCT:
		if (elem_size < qwx_qmi_decode_min_elem_size(ei, 0)) {
			printf("%s: QMI message element 0x%x "
			    "data type %s (0x%x) with bad size: %u\n",
			    sc->sc_dev.dv_xname, elem_type,
			    qmi_data_type_name[ei->data_type],
			    ei->data_type, elem_size);
			return -1;
		}
		break;
	default:
		printf("%s: unexpected QMI message element "
		    "data type 0x%x\n", sc->sc_dev.dv_xname,
		    ei->data_type);
		return -1;
	}

	if (remain < elem_size) {
		printf("%s: QMI message too short\n", sc->sc_dev.dv_xname);
		return -1;
	}

	if (ei->offset + ei->elem_size > output_len) {
		printf("%s: QMI message element type 0x%x too large: %u\n",
		    sc->sc_dev.dv_xname, elem_type, ei->elem_size);
		return -1;
	}

	return 0;
}

int
qwx_qmi_decode_byte(void *output, const struct qmi_elem_info *ei, void *input)
{
	if (ei->elem_size != sizeof(uint8_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	DNPRINTF(QWX_D_QMI, "%s: element 0x%x data type 0x%x size %u\n",
	    __func__, ei->tlv_type, ei->data_type, ei->elem_size);
	memcpy(output, input, ei->elem_size);
	return 0;
}

int
qwx_qmi_decode_word(void *output, const struct qmi_elem_info *ei, void *input)
{
	if (ei->elem_size != sizeof(uint16_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	DNPRINTF(QWX_D_QMI, "%s: element 0x%x data type 0x%x size %u\n",
	    __func__, ei->tlv_type, ei->data_type, ei->elem_size);
	memcpy(output, input, ei->elem_size);
	return 0;
}

int
qwx_qmi_decode_dword(void *output, const struct qmi_elem_info *ei, void *input)
{
	if (ei->elem_size != sizeof(uint32_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	DNPRINTF(QWX_D_QMI, "%s: element 0x%x data type 0x%x size %u\n",
	    __func__, ei->tlv_type, ei->data_type, ei->elem_size);
	memcpy(output, input, ei->elem_size);
	return 0;
}

int
qwx_qmi_decode_qword(void *output, const struct qmi_elem_info *ei, void *input)
{
	if (ei->elem_size != sizeof(uint64_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	DNPRINTF(QWX_D_QMI, "%s: element 0x%x data type 0x%x size %u\n",
	    __func__, ei->tlv_type, ei->data_type, ei->elem_size);
	memcpy(output, input, ei->elem_size);
	return 0;
}

int
qwx_qmi_decode_datalen(struct qwx_softc *sc, size_t *used, uint32_t *datalen,
    void *output, size_t output_len, const struct qmi_elem_info *ei,
    uint8_t *input, uint16_t input_len)
{
	uint8_t *p = input;
	size_t remain = input_len;

	*datalen = 0;

	DNPRINTF(QWX_D_QMI, "%s: input: ", __func__);
	for (int i = 0; i < input_len; i++) {
		DNPRINTF(QWX_D_QMI, " %02x", input[i]);
	}
	DNPRINTF(QWX_D_QMI, "\n");

	if (remain < ei->elem_size) {
		printf("%s: QMI message too short: remain=%zu elem_size=%u\n", __func__, remain, ei->elem_size);
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
		    sc->sc_dev.dv_xname, ei->elem_size);
		return -1;
		
	}
	*used = ei->elem_size;

	if (ei->offset + sizeof(*datalen) > output_len) {
		printf("%s: QMI message element type 0x%x too large\n",
		    sc->sc_dev.dv_xname, ei->tlv_type);
		return -1;
	}
	memcpy(output + ei->offset, datalen, sizeof(*datalen));
	return 0;
}

int
qwx_qmi_decode_string(struct qwx_softc *sc, size_t *used_total,
    void *output, size_t output_len, const struct qmi_elem_info *ei,
    uint8_t *input, uint16_t input_len, uint16_t elem_size, int nested)
{
	uint8_t *p = input;
	uint16_t len;
	size_t remain = input_len;

	*used_total = 0;

	DNPRINTF(QWX_D_QMI, "%s: input: ", __func__);
	for (int i = 0; i < input_len; i++) {
		DNPRINTF(QWX_D_QMI, " %02x", input[i]);
	}
	DNPRINTF(QWX_D_QMI, "\n");

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

	if (len > ei->elem_len) {
		printf("%s: QMI string element of length %u exceeds "
		    "maximum length %u\n", __func__, len, ei->elem_len);
		return -1;
	}
	if (len > remain) {
		printf("%s: QMI string element of length %u exceeds "
		    "input buffer size %zu\n", __func__, len, remain);
		return -1;
	}
	if (len > output_len) {
		printf("%s: QMI string element of length %u exceeds "
		    "output buffer size %zu\n", __func__, len, output_len);
		return -1;
	}

	memcpy(output, p, len);

	p = output;
	p[len] = '\0';
	DNPRINTF(QWX_D_QMI, "%s: string (len %u): %s\n", __func__, len, p);

	*used_total += len;
	return 0;
}

int
qwx_qmi_decode_struct(struct qwx_softc *sc, size_t *used_total,
    void *output, size_t output_len,
    const struct qmi_elem_info *struct_ei,
    uint8_t *input, uint16_t input_len,
    int nested)
{
	const struct qmi_elem_info *ei = struct_ei->ei_array;
	uint32_t min_size;
	uint8_t *p = input;
	size_t remain = input_len;
	size_t used = 0;

	*used_total = 0;

	DNPRINTF(QWX_D_QMI, "%s: input: ", __func__);
	for (int i = 0; i < input_len; i++) {
		DNPRINTF(QWX_D_QMI, " %02x", input[i]);
	}
	DNPRINTF(QWX_D_QMI, "\n");

	min_size = qwx_qmi_decode_min_elem_size(struct_ei, 0);
	DNPRINTF(QWX_D_QMI, "%s: minimum struct size: %u\n", __func__, min_size);
	while (*used_total < min_size && ei->data_type != QMI_EOTI) {
		if (remain == 0) {
			printf("%s: QMI message too short\n", __func__);
			return -1;
		}

		if (ei->data_type == QMI_DATA_LEN) {
			uint32_t datalen;

			used = 0;
			if (qwx_qmi_decode_datalen(sc, &used, &datalen,
			    output, output_len, ei, p, remain))
				return -1;
			DNPRINTF(QWX_D_QMI, "%s: datalen %u used %zu bytes\n",
			    __func__, datalen, used);
			p += used;
			remain -= used;
			*used_total += used;
			if (remain < datalen) {
				printf("%s: QMI message too short\n", __func__);
				return -1;
			}
			ei++;
			DNPRINTF(QWX_D_QMI, "%s: datalen is for data_type=0x%x "
			    "tlv_type=0x%x elem_size=%u(0x%x) remain=%zu\n",
			    __func__, ei->data_type, ei->tlv_type,
			    ei->elem_size, ei->elem_size, remain);
			if (datalen == 0) {
				ei++;
				DNPRINTF(QWX_D_QMI,
				    "%s: skipped to data_type=0x%x "
				    "tlv_type=0x%x elem_size=%u(0x%x) "
				    "remain=%zu\n", __func__,
				    ei->data_type, ei->tlv_type,
				    ei->elem_size, ei->elem_size, remain);
				continue;
			}
		} else {
			if (remain < ei->elem_size) {
				printf("%s: QMI message too short\n",
				    __func__);
				return -1;
			}
		}

		if (ei->offset + ei->elem_size > output_len) {
			printf("%s: QMI message struct member element "
			    "type 0x%x too large: %u\n", sc->sc_dev.dv_xname,
			    ei->tlv_type, ei->elem_size);
			return -1;
		}

		DNPRINTF(QWX_D_QMI,
		    "%s: decoding struct member element 0x%x with "
		    "data type %s (0x%x) size=%u(0x%x) remain=%zu\n", __func__,
		    ei->tlv_type, qmi_data_type_name[ei->data_type],
		    ei->data_type, ei->elem_size, ei->elem_size, remain);
		switch (ei->data_type) {
		case QMI_UNSIGNED_1_BYTE:
			if (qwx_qmi_decode_byte(output + ei->offset, ei, p))
				return -1;
			remain -= ei->elem_size;
			p += ei->elem_size;
			*used_total += ei->elem_size;
			break;
		case QMI_UNSIGNED_2_BYTE:
		case QMI_SIGNED_2_BYTE_ENUM:
			if (qwx_qmi_decode_word(output + ei->offset, ei, p))
				return -1;
			remain -= ei->elem_size;
			p += ei->elem_size;
			*used_total += ei->elem_size;
			break;
		case QMI_UNSIGNED_4_BYTE:
		case QMI_SIGNED_4_BYTE_ENUM:
			if (qwx_qmi_decode_dword(output + ei->offset, ei, p))
				return -1;
			remain -= ei->elem_size;
			p += ei->elem_size;
			*used_total += ei->elem_size;
			break;
		case QMI_UNSIGNED_8_BYTE:
			if (qwx_qmi_decode_qword(output + ei->offset, ei, p))
				return -1;
			remain -= ei->elem_size;
			p += ei->elem_size;
			*used_total += ei->elem_size;
			break;
		case QMI_STRUCT:
			if (nested > 2) {
				printf("%s: QMI struct element data type 0x%x "
				    "is nested too deeply\n",
				    sc->sc_dev.dv_xname, ei->data_type);
				return -1;
			}
			used = 0;
			if (qwx_qmi_decode_struct(sc, &used,
			    output + ei->offset, output_len - ei->offset,
			    ei, p, remain, nested + 1))
				return -1;
			remain -= used;
			p += used;
			*used_total += used;
			break;
		case QMI_STRING:
			used = 0;
			if (qwx_qmi_decode_string(sc, &used,
			    output + ei->offset, output_len - ei->offset,
			    ei, p, remain, 0, 1))
				return -1;
			remain -= used;
			p += used;
			*used_total += used;
			break;
		default:
			printf("%s: unhandled QMI struct element "
			    "data type 0x%x\n", sc->sc_dev.dv_xname,
			    ei->data_type);
			return -1;
		}

		ei++;
		DNPRINTF(QWX_D_QMI, "%s: next ei 0x%x ei->data_type=0x%x\n",
		    __func__, ei->tlv_type, ei->data_type);
	}

	DNPRINTF(QWX_D_QMI, "%s: used_total=%zu ei->data_type=0x%x\n",
	    __func__, *used_total, ei->data_type);

	return 0;
}

int
qwx_qmi_decode_msg(struct qwx_softc *sc, void *output, size_t output_len,
    const struct qmi_elem_info *ei0, uint8_t *input, uint16_t input_len)
{
	uint8_t *p = input;
	size_t remain = input_len, used;
	const struct qmi_elem_info *ei = ei0;

	memset(output, 0, output_len);

	DNPRINTF(QWX_D_QMI, "%s: input: ", __func__);
	for (int i = 0; i < input_len; i++) {
		DNPRINTF(QWX_D_QMI, " %02x", input[i]);
	}
	DNPRINTF(QWX_D_QMI, "\n");

	while (remain > 0 && ei->data_type != QMI_EOTI) {
		uint32_t nelem = 1, i;
		uint16_t datalen;

		if (qwx_qmi_decode_tlv_hdr(sc, &ei, &datalen, output_len,
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
				    "too large: %u\n", sc->sc_dev.dv_xname,
				    ei->tlv_type, ei->elem_size);
			}

			pvalid = (uint8_t *)output + ei->offset;
			*pvalid = 1;

			ei++;
		}

		p += 3;
		remain -= 3;

		if (ei->data_type == QMI_DATA_LEN) {
			const struct qmi_elem_info *datalen_ei = ei;
			uint8_t elem_type = ei->tlv_type;

			/*
			 * Size info in TLV header indicates the
			 * total length of element data that follows.
			 */
			if (remain < datalen) {
				printf("%s:%d QMI message too short\n",
				    __func__, __LINE__);
				return -1;
			}

			ei++;
			DNPRINTF(QWX_D_QMI,
			    "%s: next ei data_type=0x%x tlv_type=0x%x "
			    "dst elem_size=%u(0x%x) src total size=%u "
			    "remain=%zu\n", __func__, ei->data_type,
			    ei->tlv_type, ei->elem_size, ei->elem_size,
			    datalen, remain);

			/* Related EIs must have the same type. */
			if (ei->tlv_type != elem_type) {
				printf("%s: unexpected element type 0x%x; "
				    "expected 0x%x\n", __func__,
				    ei->tlv_type, elem_type);
				return -1;
			}

			if (datalen == 0) {
				if (ei->data_type != QMI_EOTI)
					ei++;
				continue;
			}

			/*
			 * For variable length arrays a one- or two-byte
			 * value follows the header, indicating the number
			 * of elements in the array.
			 */
			if (ei->array_type == VAR_LEN_ARRAY) {
				DNPRINTF(QWX_D_QMI,
				    "%s: variable length array\n", __func__);
				used = 0;
				if (qwx_qmi_decode_datalen(sc, &used, &nelem,
				    output, output_len, datalen_ei, p, remain))
					return -1;
				p += used;
				remain -= used;
				/*
				 * Previous datalen value included the total
				 * amount of bytes following the DATALEN TLV
				 * header.
				 */
				datalen -= used;

				if (nelem == 0) {
					if (ei->data_type != QMI_EOTI)
						ei++;
					continue;
				}

				DNPRINTF(QWX_D_QMI,
				    "%s: datalen %u used %zu bytes\n",
				    __func__, nelem, used);

				DNPRINTF(QWX_D_QMI,
				    "%s: decoding %u array elements with "
				    "src size %u dest size %u\n", __func__,
				    nelem, datalen / nelem, ei->elem_size);
			}
		}

		if (remain < datalen) {
			printf("%s:%d QMI message too short: remain=%zu, "
			    "datalen=%u\n", __func__, __LINE__, remain,
			    datalen);
			return -1;
		}
		if (output_len < nelem * ei->elem_size) {
			printf("%s: QMI output buffer too short: remain=%zu "
			    "nelem=%u ei->elem_size=%u\n", __func__, remain,
			    nelem, ei->elem_size);
			return -1;
		}

		for (i = 0; i < nelem && remain > 0; i++) {
			size_t outoff;

			outoff = ei->offset + (ei->elem_size * i);
			switch (ei->data_type) {
			case QMI_STRUCT:
				used = 0;
				if (qwx_qmi_decode_struct(sc, &used,
				    output + outoff, output_len - outoff,
				    ei, p, remain, 0))
					return -1;
				remain -= used;
				p += used;
				if (used != datalen) {
					DNPRINTF(QWX_D_QMI,
					    "%s struct used only %zu bytes "
					    "of %u input bytes\n", __func__,
					    used, datalen);
				} else {
					DNPRINTF(QWX_D_QMI,
					    "%s: struct used %zu bytes "
					    "of input\n", __func__, used);
				}
				break;
			case QMI_STRING:
				used = 0;
				if (qwx_qmi_decode_string(sc, &used,
				    output + outoff, output_len - outoff,
				    ei, p, remain, datalen, 0))
					return -1;
				remain -= used;
				p += used;
				if (used != datalen) {
					DNPRINTF(QWX_D_QMI,
					    "%s: string used only %zu bytes "
					    "of %u input bytes\n", __func__,
					    used, datalen);
				} else {
					DNPRINTF(QWX_D_QMI,
					    "%s: string used %zu bytes "
					    "of input\n", __func__, used);
				}
				break;
			case QMI_UNSIGNED_1_BYTE:
				if (remain < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qwx_qmi_decode_byte(output + outoff,
				    ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
				if (remain < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qwx_qmi_decode_word(output + outoff,
				    ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
				if (remain < ei->elem_size) {
					printf("%s: QMI message too "
					    "short\n", __func__);
					return -1;
				}
				if (qwx_qmi_decode_dword(output + outoff,
				    ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_8_BYTE:
				if (remain < ei->elem_size) {
					printf("%s: QMI message too "
					    "short 4\n", __func__);
					return -1;
				}
				if (qwx_qmi_decode_qword(output + outoff,
				    ei, p))
					return -1;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			default:
				printf("%s: unhandled QMI message element "
				    "data type 0x%x\n",
				    sc->sc_dev.dv_xname, ei->data_type);
				return -1;
			}
		}

		ei++;
		DNPRINTF(QWX_D_QMI,
		    "%s: next ei 0x%x ei->data_type=0x%x remain=%zu\n",
		    __func__, ei->tlv_type, ei->data_type, remain);

		DNPRINTF(QWX_D_QMI, "%s: remaining input: ", __func__);
		for (int i = 0; i < remain; i++)
			DNPRINTF(QWX_D_QMI, " %02x", p[i]);
		DNPRINTF(QWX_D_QMI, "\n");
	}

	return 0;
}

int
qwx_qmi_encode_datalen(uint8_t *p, uint32_t *datalen,
    const struct qmi_elem_info *ei, void *input)
{
	memcpy(datalen, input + ei->offset, sizeof(uint32_t));

	if (ei->elem_size == sizeof(uint8_t)) {
		p[0] = (*datalen & 0xff);
	} else if (ei->elem_size == sizeof(uint16_t)) {
		p[0] = (*datalen & 0xff);
		p[1] = (*datalen >> 8) & 0xff;
	} else {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	return 0;
}

int
qwx_qmi_encode_byte(uint8_t *p, const struct qmi_elem_info *ei, void *input,
    int i)
{
	if (ei->elem_size != sizeof(uint8_t)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(p, input + ei->offset + (i * ei->elem_size), ei->elem_size);
	return 0;
}

int
qwx_qmi_encode_word(uint8_t *p, const struct qmi_elem_info *ei, void *input,
    int i)
{
	uint16_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, input + ei->offset + (i * ei->elem_size), ei->elem_size);
	val = htole16(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

int
qwx_qmi_encode_dword(uint8_t *p, const struct qmi_elem_info *ei, void *input,
    int i)
{
	uint32_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, input + ei->offset + (i * ei->elem_size), ei->elem_size);
	val = htole32(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

int
qwx_qmi_encode_qword(uint8_t *p, const struct qmi_elem_info *ei, void *input,
    int i)
{
	uint64_t val;

	if (ei->elem_size != sizeof(val)) {
		printf("%s: bad element size\n", __func__);
		return -1;
	}

	if (p == NULL)
		return 0;

	memcpy(&val, input + ei->offset + (i * ei->elem_size), ei->elem_size);
	val = htole64(val);
	memcpy(p, &val, sizeof(val));
	return 0;
}

int
qwx_qmi_encode_struct(uint8_t *p, size_t *encoded_len,
    const struct qmi_elem_info *struct_ei, void *input, size_t input_len)
{
	const struct qmi_elem_info *ei = struct_ei->ei_array;
	size_t remain = input_len;

	*encoded_len = 0;

	while (ei->data_type != QMI_EOTI) {
		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			memcpy(&do_encode, input + ei->offset, sizeof(uint8_t));
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

		if (ei->elem_size > remain) {
			printf("%s: QMI message buffer too short\n", __func__);
			return -1;
		}

		switch (ei->data_type) {
		case QMI_UNSIGNED_1_BYTE:
			if (qwx_qmi_encode_byte(p, ei, input, 0))
				return -1;
			break;
		case QMI_UNSIGNED_2_BYTE:
			if (qwx_qmi_encode_word(p, ei, input, 0))
				return -1;
			break;
		case QMI_UNSIGNED_4_BYTE:
		case QMI_SIGNED_4_BYTE_ENUM:
			if (qwx_qmi_encode_dword(p, ei, input, 0))
				return -1;
			break;
		case QMI_UNSIGNED_8_BYTE:
			if (qwx_qmi_encode_qword(p, ei, input, 0))
				return -1;
			break;
		default:
			printf("%s: unhandled QMI struct element type %d\n",
			    __func__, ei->data_type);
			return -1;
		}

		remain -= ei->elem_size;
		if (p != NULL)
			p += ei->elem_size;
		*encoded_len += ei->elem_size;
		ei++;
	}

	return 0;
}

int
qwx_qmi_encode_string(uint8_t *p, size_t *encoded_len,
    const struct qmi_elem_info *string_ei, void *input, size_t input_len)
{
	*encoded_len = strnlen(input, input_len);
	if (*encoded_len > string_ei->elem_len) {
		printf("%s: QMI message buffer too short\n", __func__);
		return -1;
	}

	if (p)
		memcpy(p, input, *encoded_len);

	return 0;
}

int
qwx_qmi_encode_msg(uint8_t **encoded_msg, size_t *encoded_len, int type,
    uint16_t *txn_id, uint16_t msg_id, size_t msg_len,
    const struct qmi_elem_info *ei, void *input, size_t input_len)
{
	const struct qmi_elem_info *ei0 = ei;
	struct qmi_header hdr;
	size_t remain;
	uint8_t *p, *op;

	*encoded_msg = NULL;
	*encoded_len = 0;

	/* First pass: Determine length of encoded message. */
	while (ei->data_type != QMI_EOTI) {
		int nelem = 1, i;

		if (ei->offset + ei->elem_size > input_len) {
			printf("%s: bad input buffer offset at element 0x%x "
			    "data type 0x%x\n",
			    __func__, ei->tlv_type, ei->data_type);
			goto err;
		}

		/*
		 * OPT_FLAG determines whether the next element
		 * should be considered for encoding.
		 */
		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			memcpy(&do_encode, input + ei->offset, sizeof(uint8_t));
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

		*encoded_len += 3; /* type, length */
		if (ei->data_type == QMI_DATA_LEN) {
			uint32_t datalen = 0;
			uint8_t dummy[2];

			if (qwx_qmi_encode_datalen(dummy, &datalen, ei, input))
				goto err;
			*encoded_len += ei->elem_size;
			ei++;
			if (ei->array_type != VAR_LEN_ARRAY) {
				printf("%s: data len not for a var array\n",
				    __func__);
				goto err;
			}
			nelem = datalen;
			if (ei->data_type == QMI_STRUCT) {
				for (i = 0; i < nelem; i++) {
					size_t encoded_struct_len = 0;
					size_t inoff = ei->offset + (i * ei->elem_size);

					if (qwx_qmi_encode_struct(NULL,
					    &encoded_struct_len, ei,
					    input + inoff, input_len - inoff))
						goto err;

					*encoded_len += encoded_struct_len;
				}
			} else
				*encoded_len += nelem * ei->elem_size;
			ei++;
		} else if (ei->data_type == QMI_STRING) {
			size_t encoded_string_len = 0;
			size_t inoff = ei->offset;

			if (qwx_qmi_encode_string(NULL,
			    &encoded_string_len, ei,
			    input + inoff, input_len - inoff))
				goto err;
			*encoded_len += encoded_string_len;
			ei++;
		} else {
			*encoded_len += ei->elem_size;
			ei++;
		}
	}

	*encoded_len += sizeof(hdr);
	*encoded_msg = malloc(*encoded_len, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (*encoded_msg == NULL)
		return ENOMEM;

	hdr.type = type;
	hdr.txn_id = htole16(*txn_id);
	hdr.msg_id = htole16(msg_id);
	hdr.msg_len = htole16(*encoded_len - sizeof(hdr));
	memcpy(*encoded_msg, &hdr, sizeof(hdr));

	/* Second pass: Encode the message. */
	ei = ei0;
	p = *encoded_msg + sizeof(hdr);
	remain = *encoded_len - sizeof(hdr);
	while (ei->data_type != QMI_EOTI) {
		uint32_t datalen = 0;
		int nelem = 1, i;

		if (ei->data_type == QMI_OPT_FLAG) {
			uint8_t do_encode, tlv_type;

			memcpy(&do_encode, input + ei->offset, sizeof(uint8_t));
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

		if (ei->elem_size + 3 > remain) {
			printf("%s: QMI message buffer too short\n", __func__);
			goto err;
		}

		/* 3 bytes of type-length-value header, remember for later */
		op = p;
		p += 3;

		if (ei->data_type == QMI_DATA_LEN) {
			if (qwx_qmi_encode_datalen(p, &datalen, ei, input))
				goto err;
			p += ei->elem_size;
			ei++;
			if (ei->array_type == VAR_LEN_ARRAY)
				nelem = datalen;
		}

		for (i = 0; i < nelem; i++) {
			size_t encoded_struct_len = 0;
			size_t encoded_string_len = 0;
			size_t inoff = ei->offset + (i * ei->elem_size);

			switch (ei->data_type) {
			case QMI_UNSIGNED_1_BYTE:
				if (qwx_qmi_encode_byte(p, ei, input, i))
					goto err;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_2_BYTE:
			case QMI_SIGNED_2_BYTE_ENUM:
				if (qwx_qmi_encode_word(p, ei, input, i))
					goto err;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_4_BYTE:
			case QMI_SIGNED_4_BYTE_ENUM:
				if (qwx_qmi_encode_dword(p, ei, input, i))
					goto err;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_UNSIGNED_8_BYTE:
				if (qwx_qmi_encode_qword(p, ei, input, i))
					goto err;
				remain -= ei->elem_size;
				p += ei->elem_size;
				break;
			case QMI_STRUCT:
				if (qwx_qmi_encode_struct(p,
				    &encoded_struct_len, ei,
				    input + inoff, input_len - inoff))
					goto err;
				remain -= encoded_struct_len;
				p += encoded_struct_len;
				break;
			case QMI_STRING:
				if (qwx_qmi_encode_string(p,
				    &encoded_string_len, ei,
				    input + inoff, input_len - inoff))
					goto err;
				remain -= encoded_string_len;
				p += encoded_string_len;
				break;
			default:
				printf("%s: unhandled QMI message element type %d\n",
				    __func__, ei->data_type);
				goto err;
			}
		}

		op[0] = ei->tlv_type;
		op[1] = (p - (op + 3)) & 0xff;
		op[2] = ((p - (op + 3)) >> 8) & 0xff;

		ei++;
	}

	if (0) {
		int i;
		DNPRINTF(QWX_D_QMI,
		   "%s: message type 0x%x txnid 0x%x msgid 0x%x "
		    "msglen %zu encoded:", __func__,
		    type, *txn_id, msg_id, *encoded_len - sizeof(hdr));
		for (i = 0; i < *encoded_len; i++) {
			DNPRINTF(QWX_D_QMI, "%s %.2x", i % 16 == 0 ? "\n" : "",
			    (*encoded_msg)[i]);
		}
		if (i % 16)
			DNPRINTF(QWX_D_QMI, "\n");
	}

	(*txn_id)++; /* wrap-around is fine */
	return 0;
err:
	free(*encoded_msg, M_DEVBUF, *encoded_len);
	*encoded_msg = NULL;
	*encoded_len = 0;
	return -1;
}
