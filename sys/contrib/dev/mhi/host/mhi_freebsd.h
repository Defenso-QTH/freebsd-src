/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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

/*
 * FreeBSD glue for the Linux MHI host core.  Included at the end of
 * internal.h, after all Linux headers, so the macros below take over the
 * LinuxKPI versions in the MHI sources only.
 *
 * LinuxKPI has no bus_type and its device_add() builds a newbus child from
 * a class, so the MHI bus keeps its own small device and driver lists:
 * mhi_fbsd_device_add() matches a new device against the registered MHI
 * drivers and probes it, mhi_fbsd_driver_register() does the same for the
 * devices already present.
 */

#ifndef _MHI_FREEBSD_H_
#define _MHI_FREEBSD_H_

struct kobj_uevent_env;

struct bus_type {
	const char	*name;
	const char	*dev_name;
	int		(*match)(struct device *, const struct device_driver *);
	int		(*uevent)(const struct device *, struct kobj_uevent_env *);
	int		(*probe)(struct device *);
	void		(*remove)(struct device *);
	const struct attribute_group **dev_groups;
};

/* Debug output, off unless compat.linuxkpi.mhi_debug is set. */
extern int mhi_fbsd_debug;
#undef dev_dbg
#define	dev_dbg(_dev, _fmt, ...) do {					\
	if (mhi_fbsd_debug)						\
		dev_info((_dev), _fmt, ##__VA_ARGS__);			\
} while (0)

void	mhi_fbsd_device_initialize(struct device *);
int	mhi_fbsd_device_add(struct device *);
void	mhi_fbsd_device_del(struct device *);
int	mhi_fbsd_device_for_each_child(struct device *, void *,
	    int (*)(struct device *, void *));
int	mhi_fbsd_driver_register(struct device_driver *);
void	mhi_fbsd_driver_unregister(struct device_driver *);

/*
 * Interrupts.  Numbers from MHI_FBSD_TEST_IRQ_BASE up belong to a test
 * controller that raises them with mhi_fbsd_test_irq_raise() (see
 * tools/tools/ath11k/mhi_sim); all others are LinuxKPI interrupts.
 */
#define	MHI_FBSD_TEST_IRQ_BASE		0x40000000u

int	mhi_fbsd_request_irq(unsigned int, irq_handler_t, irq_handler_t,
	    unsigned long, const char *, void *);
void	mhi_fbsd_free_irq(unsigned int, void *);
int	mhi_fbsd_enable_irq(unsigned int);
void	mhi_fbsd_disable_irq(unsigned int);
void	mhi_fbsd_test_irq_raise(unsigned int);

/*
 * Print the device and host MHI state, for boot and runtime failures.
 * Also on demand for every controller with compat.linuxkpi.mhi_dump=1.
 */
void	mhi_fbsd_dump(struct mhi_controller *, const char *);

#define	request_threaded_irq(_irq, _h, _th, _f, _n, _a)			\
	mhi_fbsd_request_irq((_irq), (_h), (_th), (_f), (_n), (_a))
#define	request_irq(_irq, _h, _f, _n, _a)				\
	mhi_fbsd_request_irq((_irq), (_h), NULL, (_f), (_n), (_a))
#define	free_irq(_irq, _a)		mhi_fbsd_free_irq((_irq), (_a))
#define	enable_irq(_irq)		mhi_fbsd_enable_irq(_irq)
#define	disable_irq(_irq)		mhi_fbsd_disable_irq(_irq)

#define	device_initialize(_dev)		mhi_fbsd_device_initialize(_dev)
#define	device_add(_dev)		mhi_fbsd_device_add(_dev)
#define	device_del(_dev)		mhi_fbsd_device_del(_dev)
#define	device_for_each_child(_dev, _data, _fn)				\
	mhi_fbsd_device_for_each_child((_dev), (_data), (_fn))
#define	driver_register(_drv)		mhi_fbsd_driver_register(_drv)
#define	driver_unregister(_drv)		mhi_fbsd_driver_unregister(_drv)
#define	bus_register(_bus)		0
#define	bus_unregister(_bus)		do { } while (0)

#endif /* _MHI_FREEBSD_H_ */
