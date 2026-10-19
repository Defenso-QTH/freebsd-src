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
 * The MHI bus on FreeBSD: device and driver lists standing in for the Linux
 * driver core, see mhi_freebsd.h.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <linux/device.h>
#include <linux/mhi.h>
#include <linux/slab.h>

#include "internal.h"
#include "linux_qrtr.h"

static MALLOC_DEFINE(M_MHIBUS, "mhibus", "MHI bus device and driver lists");

int mhi_fbsd_debug = 0;
SYSCTL_DECL(_compat_linuxkpi);
SYSCTL_INT(_compat_linuxkpi, OID_AUTO, mhi_debug, CTLFLAG_RWTUN,
    &mhi_fbsd_debug, 0, "Print MHI bus debug messages");

struct mhi_fbsd_dev {
	TAILQ_ENTRY(mhi_fbsd_dev)	link;
	struct device			*dev;
};

struct mhi_fbsd_drv {
	TAILQ_ENTRY(mhi_fbsd_drv)	link;
	struct device_driver		*drv;
};

static TAILQ_HEAD(, mhi_fbsd_dev) mhi_fbsd_devs =
    TAILQ_HEAD_INITIALIZER(mhi_fbsd_devs);
static TAILQ_HEAD(, mhi_fbsd_drv) mhi_fbsd_drvs =
    TAILQ_HEAD_INITIALIZER(mhi_fbsd_drvs);
static struct sx mhi_fbsd_lock;
SX_SYSINIT(mhi_fbsd_lock, &mhi_fbsd_lock, "mhi bus");

void
mhi_fbsd_device_initialize(struct device *dev)
{
	kobject_init(&dev->kobj, &linux_dev_ktype);
	spin_lock_init(&dev->devres_lock);
	INIT_LIST_HEAD(&dev->devres_head);
	INIT_LIST_HEAD(&dev->irqents);
}

/* Bind an unbound device to a driver if they match.  Lock held. */
static void
mhi_fbsd_try_bind(struct device *dev, struct device_driver *drv)
{
	int error;

	sx_assert(&mhi_fbsd_lock, SA_XLOCKED);
	if (dev->driver != NULL)
		return;
	if (!mhi_bus_type.match(dev, drv))
		return;
	dev->driver = drv;
	error = mhi_bus_type.probe(dev);
	if (error != 0) {
		dev_err(dev, "probe of %s failed: %d\n", drv->name, error);
		dev->driver = NULL;
	}
}

int
mhi_fbsd_device_add(struct device *dev)
{
	struct mhi_fbsd_dev *node;
	struct mhi_fbsd_drv *dnode;
	struct device *parent;

	/*
	 * Messages about this device go to the newbus device the MHI
	 * controller hangs off.
	 */
	for (parent = dev->parent; parent != NULL; parent = parent->parent) {
		if (parent->bsddev != NULL) {
			dev->bsddev = parent->bsddev;
			break;
		}
	}
	if (dev->bsddev == NULL)
		return (-ENODEV);

	node = malloc(sizeof(*node), M_MHIBUS, M_WAITOK | M_ZERO);
	node->dev = dev;
	sx_xlock(&mhi_fbsd_lock);
	TAILQ_INSERT_TAIL(&mhi_fbsd_devs, node, link);
	TAILQ_FOREACH(dnode, &mhi_fbsd_drvs, link) {
		mhi_fbsd_try_bind(dev, dnode->drv);
		if (dev->driver != NULL)
			break;
	}
	sx_xunlock(&mhi_fbsd_lock);

	return (0);
}

void
mhi_fbsd_device_del(struct device *dev)
{
	struct mhi_fbsd_dev *node;

	sx_xlock(&mhi_fbsd_lock);
	TAILQ_FOREACH(node, &mhi_fbsd_devs, link) {
		if (node->dev == dev)
			break;
	}
	if (node == NULL) {
		sx_xunlock(&mhi_fbsd_lock);
		return;
	}
	if (dev->driver != NULL) {
		mhi_bus_type.remove(dev);
		dev->driver = NULL;
	}
	TAILQ_REMOVE(&mhi_fbsd_devs, node, link);
	sx_xunlock(&mhi_fbsd_lock);
	free(node, M_MHIBUS);
	/* dev->bsddev is left alone: it belongs to the controller. */
}

/*
 * Call fn on every child of parent.  fn may delete the child, so it is
 * called without the lock on a referenced snapshot of the children.
 */
int
mhi_fbsd_device_for_each_child(struct device *parent, void *data,
    int (*fn)(struct device *, void *))
{
	struct mhi_fbsd_dev *node;
	struct device **children;
	int count, error, i;

	sx_xlock(&mhi_fbsd_lock);
	count = 0;
	TAILQ_FOREACH(node, &mhi_fbsd_devs, link) {
		if (node->dev->parent == parent)
			count++;
	}
	if (count == 0) {
		sx_xunlock(&mhi_fbsd_lock);
		return (0);
	}
	children = malloc(count * sizeof(*children), M_MHIBUS, M_WAITOK);
	i = 0;
	TAILQ_FOREACH(node, &mhi_fbsd_devs, link) {
		if (node->dev->parent == parent)
			children[i++] = get_device(node->dev);
	}
	sx_xunlock(&mhi_fbsd_lock);

	error = 0;
	for (i = 0; i < count; i++) {
		if (error == 0)
			error = fn(children[i], data);
		put_device(children[i]);
	}
	free(children, M_MHIBUS);

	return (error);
}

int
mhi_fbsd_driver_register(struct device_driver *drv)
{
	struct mhi_fbsd_drv *dnode;
	struct mhi_fbsd_dev *node;

	dnode = malloc(sizeof(*dnode), M_MHIBUS, M_WAITOK | M_ZERO);
	dnode->drv = drv;
	sx_xlock(&mhi_fbsd_lock);
	TAILQ_INSERT_TAIL(&mhi_fbsd_drvs, dnode, link);
	TAILQ_FOREACH(node, &mhi_fbsd_devs, link)
		mhi_fbsd_try_bind(node->dev, drv);
	sx_xunlock(&mhi_fbsd_lock);

	return (0);
}

void
mhi_fbsd_driver_unregister(struct device_driver *drv)
{
	struct mhi_fbsd_drv *dnode;
	struct mhi_fbsd_dev *node;

	sx_xlock(&mhi_fbsd_lock);
	TAILQ_FOREACH(node, &mhi_fbsd_devs, link) {
		if (node->dev->driver == drv) {
			mhi_bus_type.remove(node->dev);
			node->dev->driver = NULL;
		}
	}
	TAILQ_FOREACH(dnode, &mhi_fbsd_drvs, link) {
		if (dnode->drv == drv)
			break;
	}
	if (dnode != NULL)
		TAILQ_REMOVE(&mhi_fbsd_drvs, dnode, link);
	sx_xunlock(&mhi_fbsd_lock);
	free(dnode, M_MHIBUS);
}

/*
 * QRTR, the MHI client the IPCR channels bind to, and the QMI interface
 * live in this module too (linux_qrtr.c).
 */
static int
mhi_fbsd_modevent(module_t mod __unused, int type, void *data __unused)
{
	int error;

	switch (type) {
	case MOD_LOAD:
		error = lkpi_qrtr_init();
		return (-error);
	case MOD_UNLOAD:
		if (lkpi_qrtr_busy())
			return (EBUSY);
		sx_xlock(&mhi_fbsd_lock);
		if (!TAILQ_EMPTY(&mhi_fbsd_devs)) {
			sx_xunlock(&mhi_fbsd_lock);
			return (EBUSY);
		}
		sx_xunlock(&mhi_fbsd_lock);
		lkpi_qrtr_uninit();
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t mhi_fbsd_mod = {
	"linuxkpi_mhi",
	mhi_fbsd_modevent,
	NULL
};
DECLARE_MODULE(linuxkpi_mhi, mhi_fbsd_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(linuxkpi_mhi, 1);
MODULE_DEPEND(linuxkpi_mhi, linuxkpi, 1, 1, 1);
