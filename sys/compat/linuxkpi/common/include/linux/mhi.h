/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022-2023 Bjoern A. Zeeb
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

#ifndef	_LINUXKPI_LINUX_MHI_H
#define	_LINUXKPI_LINUX_MHI_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/mod_devicetable.h>

/* Modem Host Interface (MHI) */

/*
 * Doorbell modes of a channel or event ring: the values of the burst mode
 * field of a channel context.
 */
enum mhi_db_brst_mode {
	MHI_DB_BRST_DISABLE	= 2,
	MHI_DB_BRST_ENABLE	= 3,
};

/* What an event ring carries. */
enum mhi_er_data_type {
	MHI_ER_DATA		= 0,
	MHI_ER_CTRL		= 1,
};

enum mhi_callback {
	MHI_CB_SYS_ERROR,
	MHI_CB_BW_REQ,
	MHI_CB_EE_MISSION_MODE,
	MHI_CB_EE_RDDM,
	MHI_CB_FATAL_ERROR,
	MHI_CB_IDLE,
	MHI_CB_LPM_ENTER,
	MHI_CB_LPM_EXIT,
	MHI_CB_PENDING_DATA,
};

/* Flags of mhi_queue_buf(). */
enum mhi_flags {
	MHI_EOB			= (1 << 0),	/* interrupt, end of block */
	MHI_EOT			= (1 << 1),	/* interrupt, end of transfer */
	MHI_CHAIN		= (1 << 2),	/* chained to the next one */
};

struct mhi_channel_config {
	const char		*name;
	u32			num;
	u32			num_elements;
	u32			event_ring;
	enum dma_data_direction	dir;
	u32			ee_mask;
	u32			pollcfg;
	enum mhi_db_brst_mode	doorbell;
	bool			lpm_notify;
	bool			offload_channel;
	bool			doorbell_mode_switch;
	bool			auto_queue;
};

struct mhi_event_config {
	u32			num_elements;
	u32			irq_moderation_ms;
	u32			irq;
	u32			mode;
	enum mhi_er_data_type	data_type;
	u32			priority;
	bool			hardware_event;
	bool			client_managed;
	bool			offload_channel;
};

struct mhi_controller_config {
	const struct mhi_channel_config	*ch_cfg;
	const struct mhi_event_config	*event_cfg;

	u32				max_channels;
	u32				num_channels;
	u32				num_events;
	u32				buf_len;
	bool				use_bounce_buf;

	uint32_t			timeout_ms;
};

/* One DMA buffer of an image. */
struct mhi_buf {
	void				*buf;
	dma_addr_t			dma_addr;
	size_t				len;
};

/*
 * A BHIe image (firmware or crash dump): the data segments, then the
 * vector table describing them as the last entry.
 */
struct image_info {
	struct mhi_buf			*mhi_buf;
	u32				entries;
};

struct mhi_controller;
struct mhi_driver;
struct lkpi_mhi_xfer_ring;

/*
 * The controller's own device (mhi_controller->mhi_dev), or a client
 * device: the channels of one name, for example "IPCR".
 */
struct mhi_device {
	struct device			dev;
	struct mhi_controller		*mhi_cntrl;
	const struct mhi_device_id	*id;
	const char			*name;

	/* Private to the MHI core. */
	struct mhi_driver		*lkpi_drv;
	struct lkpi_mhi_xfer_ring	*lkpi_ul_chan;
	struct lkpi_mhi_xfer_ring	*lkpi_dl_chan;
	struct list_head		lkpi_node;
	int				lkpi_dev_wake;
	bool				lkpi_is_controller;
};

struct mhi_result {
	void				*buf_addr;
	size_t				bytes_xferd;
	enum dma_data_direction		dir;
	int				transaction_status;
};

struct mhi_driver {
	const struct mhi_device_id	*id_table;
	int				(*probe)(struct mhi_device *,
					    const struct mhi_device_id *);
	void				(*remove)(struct mhi_device *);
	void				(*ul_xfer_cb)(struct mhi_device *,
					    struct mhi_result *);
	void				(*dl_xfer_cb)(struct mhi_device *,
					    struct mhi_result *);
	void				(*status_cb)(struct mhi_device *,
					    enum mhi_callback);
	struct device_driver		driver;

	/* Private to the MHI core. */
	struct list_head		lkpi_node;
};

struct lkpi_mhi_softc;

struct mhi_controller {
	struct device			*cntrl_dev;
	struct mhi_device		*mhi_dev;
	void				*regs;
	int				*irq;
	const char			*fw_image;
	const u8			*fw_data;
	size_t				fw_sz;

	bool				fbc_download;
	size_t				rddm_size;
	size_t				sbl_size;
	size_t				seg_len;
	size_t				reg_len;
	int				nr_irqs;
	unsigned long			irq_flags;
	uint32_t			timeout_ms;

	dma_addr_t			iova_start;
	dma_addr_t			iova_stop;

	/* Provided by the MHI core. */
	size_t				buffer_len;
	struct image_info		*fbc_image;
	struct image_info		*rddm_image;

	int				(*runtime_get)(struct mhi_controller *);
	void				(*runtime_put)(struct mhi_controller *);
	void				(*status_cb)(struct mhi_controller *, enum mhi_callback);
	int				(*read_reg)(struct mhi_controller *, void __iomem *, uint32_t *);
	void				(*write_reg)(struct mhi_controller *, void __iomem *, uint32_t);

	/* Private to the MHI core. */
	struct lkpi_mhi_softc		*lkpi_priv;
};

/* -------------------------------------------------------------------------- */

struct mhi_controller *linuxkpi_mhi_alloc_controller(void);
void linuxkpi_mhi_free_controller(struct mhi_controller *);
int linuxkpi_mhi_register_controller(struct mhi_controller *,
    const struct mhi_controller_config *);
void linuxkpi_mhi_unregister_controller(struct mhi_controller *);

int linuxkpi_mhi_prepare_for_power_up(struct mhi_controller *);
void linuxkpi_mhi_unprepare_after_power_down(struct mhi_controller *);
int linuxkpi_mhi_sync_power_up(struct mhi_controller *);
void linuxkpi_mhi_power_down(struct mhi_controller *, bool);
void linuxkpi_mhi_power_down_keep_dev(struct mhi_controller *, bool);

int linuxkpi_mhi_driver_register(struct mhi_driver *);
void linuxkpi_mhi_driver_unregister(struct mhi_driver *);
int linuxkpi_mhi_prepare_for_transfer(struct mhi_device *);
void linuxkpi_mhi_unprepare_from_transfer(struct mhi_device *);
int linuxkpi_mhi_queue_buf(struct mhi_device *, enum dma_data_direction,
    void *, size_t, enum mhi_flags);
int linuxkpi_mhi_get_free_desc_count(struct mhi_device *,
    enum dma_data_direction);

int linuxkpi_mhi_device_get_sync(struct mhi_device *);
void linuxkpi_mhi_device_put(struct mhi_device *);
int linuxkpi_mhi_pm_suspend(struct mhi_controller *);
int linuxkpi_mhi_pm_resume(struct mhi_controller *);
int linuxkpi_mhi_pm_resume_force(struct mhi_controller *);
int linuxkpi_mhi_download_rddm_image(struct mhi_controller *, bool);

/* -------------------------------------------------------------------------- */

static inline struct mhi_controller *
mhi_alloc_controller(void)
{

	/* Keep allocations internal to our implementation. */
	return (linuxkpi_mhi_alloc_controller());
}

static inline void
mhi_free_controller(struct mhi_controller *mhi_ctrl)
{

	linuxkpi_mhi_free_controller(mhi_ctrl);
}

static inline int
mhi_register_controller(struct mhi_controller *mhi_ctrl,
    const struct mhi_controller_config *cfg)
{

	return (linuxkpi_mhi_register_controller(mhi_ctrl, cfg));
}

static inline void
mhi_unregister_controller(struct mhi_controller *mhi_ctrl)
{

	linuxkpi_mhi_unregister_controller(mhi_ctrl);
}

/* -------------------------------------------------------------------------- */

static __inline int
mhi_device_get_sync(struct mhi_device *mhi_dev)
{

	return (linuxkpi_mhi_device_get_sync(mhi_dev));
}

static __inline void
mhi_device_put(struct mhi_device *mhi_dev)
{

	linuxkpi_mhi_device_put(mhi_dev);
}

/* -------------------------------------------------------------------------- */

static __inline int
mhi_prepare_for_power_up(struct mhi_controller *mhi_ctrl)
{

	return (linuxkpi_mhi_prepare_for_power_up(mhi_ctrl));
}

static __inline int
mhi_sync_power_up(struct mhi_controller *mhi_ctrl)
{

	return (linuxkpi_mhi_sync_power_up(mhi_ctrl));
}

static __inline void
mhi_power_down(struct mhi_controller *mhi_ctrl, bool graceful)
{

	linuxkpi_mhi_power_down(mhi_ctrl, graceful);
}

static __inline void
mhi_power_down_keep_dev(struct mhi_controller *mhi_ctrl, bool graceful)
{

	linuxkpi_mhi_power_down_keep_dev(mhi_ctrl, graceful);
}

static __inline void
mhi_unprepare_after_power_down(struct mhi_controller *mhi_ctrl)
{

	linuxkpi_mhi_unprepare_after_power_down(mhi_ctrl);
}

/* -------------------------------------------------------------------------- */

static __inline int
mhi_driver_register(struct mhi_driver *mhi_drv)
{

	return (linuxkpi_mhi_driver_register(mhi_drv));
}

static __inline void
mhi_driver_unregister(struct mhi_driver *mhi_drv)
{

	linuxkpi_mhi_driver_unregister(mhi_drv);
}

static __inline int
mhi_prepare_for_transfer(struct mhi_device *mhi_dev)
{

	return (linuxkpi_mhi_prepare_for_transfer(mhi_dev));
}

static __inline void
mhi_unprepare_from_transfer(struct mhi_device *mhi_dev)
{

	linuxkpi_mhi_unprepare_from_transfer(mhi_dev);
}

static __inline int
mhi_queue_buf(struct mhi_device *mhi_dev, enum dma_data_direction dir,
    void *buf, size_t len, enum mhi_flags mflags)
{

	return (linuxkpi_mhi_queue_buf(mhi_dev, dir, buf, len, mflags));
}

static __inline int
mhi_get_free_desc_count(struct mhi_device *mhi_dev,
    enum dma_data_direction dir)
{

	return (linuxkpi_mhi_get_free_desc_count(mhi_dev, dir));
}

/* -------------------------------------------------------------------------- */

static __inline int
mhi_pm_suspend(struct mhi_controller *mhi_ctrl)
{

	return (linuxkpi_mhi_pm_suspend(mhi_ctrl));
}

static __inline int
mhi_pm_resume(struct mhi_controller *mhi_ctrl)
{

	return (linuxkpi_mhi_pm_resume(mhi_ctrl));
}

static __inline int
mhi_pm_resume_force(struct mhi_controller *mhi_ctrl)
{

	return (linuxkpi_mhi_pm_resume_force(mhi_ctrl));
}

/* -------------------------------------------------------------------------- */

static __inline int
mhi_download_rddm_image(struct mhi_controller *mhi_ctrl, bool in_panic)
{

	return (linuxkpi_mhi_download_rddm_image(mhi_ctrl, in_panic));
}

#endif	/* _LINUXKPI_LINUX_MHI_H */
