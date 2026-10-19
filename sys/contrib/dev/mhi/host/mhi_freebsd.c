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
#include <sys/mutex.h>
#include <sys/priority.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <linux/compat.h>
#include <linux/device.h>
#include <linux/interrupt.h>
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
 * Test interrupts, for a controller without hardware behind it.  They keep
 * the semantics LinuxKPI gives MSI interrupts: disabling only counts, an
 * interrupt raised while disabled is replayed on enable, and invocations
 * of a handler do not overlap.  Handlers run on a taskqueue thread.
 */
#define	MHI_FBSD_TEST_IRQS	8

struct mhi_fbsd_test_irq {
	unsigned int	irq;
	irq_handler_t	handler;
	irq_handler_t	thread_handler;
	void		*arg;
	int		depth;
	bool		pending;
	bool		busy;
	struct task	task;
};

static struct mhi_fbsd_test_irq mhi_fbsd_test_irqs[MHI_FBSD_TEST_IRQS];
static struct mtx mhi_fbsd_test_irq_lock;
MTX_SYSINIT(mhi_fbsd_test_irq_lock, &mhi_fbsd_test_irq_lock, "mhi test irq list",
    MTX_DEF);
static struct taskqueue *mhi_fbsd_test_tq;

static struct mhi_fbsd_test_irq *
mhi_fbsd_test_irq_find(unsigned int irq)
{
	int i;

	for (i = 0; i < MHI_FBSD_TEST_IRQS; i++) {
		if (mhi_fbsd_test_irqs[i].handler != NULL &&
		    mhi_fbsd_test_irqs[i].irq == irq)
			return (&mhi_fbsd_test_irqs[i]);
	}
	return (NULL);
}

static void
mhi_fbsd_test_irq_task(void *arg, int pending __unused)
{
	struct mhi_fbsd_test_irq *ti = arg;
	irq_handler_t handler, thread_handler;
	void *harg;

	if (linux_set_current_flags(curthread, M_NOWAIT))
		return;
	mtx_lock(&mhi_fbsd_test_irq_lock);
	while (ti->handler != NULL && ti->pending && ti->depth == 0 &&
	    !ti->busy) {
		ti->pending = false;
		ti->busy = true;
		handler = ti->handler;
		thread_handler = ti->thread_handler;
		harg = ti->arg;
		mtx_unlock(&mhi_fbsd_test_irq_lock);
		if (handler(ti->irq, harg) == IRQ_WAKE_THREAD &&
		    thread_handler != NULL)
			thread_handler(ti->irq, harg);
		mtx_lock(&mhi_fbsd_test_irq_lock);
		ti->busy = false;
		wakeup(ti);
	}
	mtx_unlock(&mhi_fbsd_test_irq_lock);
}

void
mhi_fbsd_test_irq_raise(unsigned int irq)
{
	struct mhi_fbsd_test_irq *ti;

	mtx_lock(&mhi_fbsd_test_irq_lock);
	ti = mhi_fbsd_test_irq_find(irq);
	if (ti != NULL) {
		ti->pending = true;
		if (ti->depth == 0)
			taskqueue_enqueue(mhi_fbsd_test_tq, &ti->task);
	}
	mtx_unlock(&mhi_fbsd_test_irq_lock);
}

int
mhi_fbsd_request_irq(unsigned int irq, irq_handler_t handler,
    irq_handler_t thread_handler, unsigned long flags, const char *name,
    void *arg)
{
	struct mhi_fbsd_test_irq *ti;
	int i;

	if (irq < MHI_FBSD_TEST_IRQ_BASE)
		return (lkpi_request_irq(NULL, irq, handler, thread_handler,
		    flags, name, arg));

	mtx_lock(&mhi_fbsd_test_irq_lock);
	if (mhi_fbsd_test_tq == NULL) {
		mtx_unlock(&mhi_fbsd_test_irq_lock);
		return (-ENXIO);
	}
	for (i = 0; i < MHI_FBSD_TEST_IRQS; i++) {
		if (mhi_fbsd_test_irqs[i].handler == NULL)
			break;
	}
	if (i == MHI_FBSD_TEST_IRQS || mhi_fbsd_test_irq_find(irq) != NULL) {
		mtx_unlock(&mhi_fbsd_test_irq_lock);
		return (-EBUSY);
	}
	ti = &mhi_fbsd_test_irqs[i];
	ti->irq = irq;
	ti->handler = handler;
	ti->thread_handler = thread_handler;
	ti->arg = arg;
	ti->depth = 0;
	ti->pending = false;
	ti->busy = false;
	TASK_INIT(&ti->task, 0, mhi_fbsd_test_irq_task, ti);
	mtx_unlock(&mhi_fbsd_test_irq_lock);
	return (0);
}

void
mhi_fbsd_free_irq(unsigned int irq, void *arg)
{
	struct mhi_fbsd_test_irq *ti;

	if (irq < MHI_FBSD_TEST_IRQ_BASE) {
		lkpi_free_irq(irq, arg);
		return;
	}
	mtx_lock(&mhi_fbsd_test_irq_lock);
	ti = mhi_fbsd_test_irq_find(irq);
	if (ti != NULL)
		ti->depth++;
	mtx_unlock(&mhi_fbsd_test_irq_lock);
	if (ti == NULL)
		return;
	taskqueue_drain(mhi_fbsd_test_tq, &ti->task);
	mtx_lock(&mhi_fbsd_test_irq_lock);
	ti->handler = NULL;
	mtx_unlock(&mhi_fbsd_test_irq_lock);
}

int
mhi_fbsd_enable_irq(unsigned int irq)
{
	struct mhi_fbsd_test_irq *ti;

	if (irq < MHI_FBSD_TEST_IRQ_BASE)
		return (lkpi_enable_irq(irq));
	mtx_lock(&mhi_fbsd_test_irq_lock);
	ti = mhi_fbsd_test_irq_find(irq);
	if (ti != NULL && ti->depth > 0 && --ti->depth == 0 && ti->pending)
		taskqueue_enqueue(mhi_fbsd_test_tq, &ti->task);
	mtx_unlock(&mhi_fbsd_test_irq_lock);
	return (0);
}

void
mhi_fbsd_disable_irq(unsigned int irq)
{
	struct mhi_fbsd_test_irq *ti;

	if (irq < MHI_FBSD_TEST_IRQ_BASE) {
		lkpi_disable_irq(irq);
		return;
	}
	mtx_lock(&mhi_fbsd_test_irq_lock);
	ti = mhi_fbsd_test_irq_find(irq);
	if (ti != NULL) {
		ti->depth++;
		while (ti->busy && taskqueue_member(mhi_fbsd_test_tq,
		    curthread) == 0)
			mtx_sleep(ti, &mhi_fbsd_test_irq_lock, 0, "mhitirq", 0);
	}
	mtx_unlock(&mhi_fbsd_test_irq_lock);
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
		mhi_fbsd_test_tq = taskqueue_create("mhi test irq", M_WAITOK,
		    taskqueue_thread_enqueue, &mhi_fbsd_test_tq);
		taskqueue_start_threads(&mhi_fbsd_test_tq, 1, PI_NET,
		    "mhi test irq");
		error = lkpi_qrtr_init();
		if (error != 0) {
			taskqueue_free(mhi_fbsd_test_tq);
			mhi_fbsd_test_tq = NULL;
		}
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
		taskqueue_free(mhi_fbsd_test_tq);
		mhi_fbsd_test_tq = NULL;
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
