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
 * MHI register, context and ring element definitions, taken from the MHI
 * host code of OpenBSD's qwx(4) (if_qwx_pci.c 1.41).  Shared by the MHI
 * host core (linux_mhi.c) and the device model
 * (tools/tools/ath11k/mhi_sim).
 */

#ifndef	_LINUXKPI_SRC_LINUX_MHI_REG_H_
#define	_LINUXKPI_SRC_LINUX_MHI_REG_H_

#include <linux/types.h>
#include <linux/bits.h>
#include <linux/bitfield.h>

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
#define MHI_CHAN_CTX_POLLCFG_SHFT		10
#define MHI_CHAN_CTX_RESERVED_MASK		GENMASK(31, 16)

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

#define MHI_CH_STATE_DISABLED	0
#define MHI_CH_STATE_ENABLED	1
#define MHI_CH_STATE_RUNNING	2
#define MHI_CH_STATE_SUSPENDED	3
#define MHI_CH_STATE_STOP	4
#define MHI_CH_STATE_ERROR	5

/* Event context. Shared with device. */
struct mhi_event_ctxt {
	uint32_t intmod;
	uint32_t ertype;
	uint32_t msivec;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

/* Channel context. Shared with device. */
struct mhi_chan_ctxt {
	uint32_t chcfg;
	uint32_t chtype;
	uint32_t erindex;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

/* Command context. Shared with device. */
struct mhi_cmd_ctxt {
	uint32_t reserved0;
	uint32_t reserved1;
	uint32_t reserved2;

	uint64_t rbase;
	uint64_t rlen;
	uint64_t rp;
	uint64_t wp;
} __packed;

struct mhi_ring_element {
	uint64_t ptr;
	uint32_t dword[2];
};

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

/*
 * Not from qwx: the device model also implements a register length, a
 * version and a BHI error code register.  The qwx code does not name
 * them and neither the core nor the state dump accesses them, so these
 * are offsets of the model only (placed where qwx defines nothing), not
 * known hardware offsets.
 */
#define MHI_REGLEN			0x00
#define MHI_VER				0x04
#define   MHI_BHI_ERRCODE			0x30

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
/* One entry of a BHIe vector table. Shared with device. */
struct mhi_dma_vec_entry {
	uint64_t paddr;
	uint64_t size;
};

/*
 * Interface of linux_mhi_bus.c (FreeBSD glue): interrupts, which also
 * serve the test interrupts of the device model, numbered from
 * MHI_FBSD_TEST_IRQ_BASE up.
 */
#define	MHI_FBSD_TEST_IRQ_BASE	0x40000000u

#ifdef _KERNEL
#include <linux/interrupt.h>

extern int mhi_fbsd_debug;

int	mhi_fbsd_request_irq(unsigned int, irq_handler_t, irq_handler_t,
	    unsigned long, const char *, void *);
void	mhi_fbsd_free_irq(unsigned int, void *);
int	mhi_fbsd_enable_irq(unsigned int);
void	mhi_fbsd_disable_irq(unsigned int);
void	mhi_fbsd_test_irq_raise(unsigned int);

/* From linux_mhi.c, for the module handler and the sysctls. */
bool	lkpi_mhi_busy(void);
void	lkpi_mhi_dump_all(void);
#endif

#endif	/* _LINUXKPI_SRC_LINUX_MHI_REG_H_ */
