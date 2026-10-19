/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2015 Mellanox Technologies, Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/priority.h>
#include <sys/proc.h>
#include <sys/taskqueue.h>

/*
 * MSI and MSI-X interrupts follow the Linux "lazy" disable model:
 * disable_irq() and disable_irq_nosync() only raise a depth counter, an
 * interrupt that arrives while the depth is non-zero is recorded as
 * pending, and enable_irq() replays it once the depth drops back to zero.
 * Handler invocations of one interrupt never run concurrently, an
 * invocation that collides with a running one is folded into it.  This is
 * what lets drivers call disable_irq_nosync() from their own handler and
 * enable_irq() later from a tasklet, which tearing the handler down cannot
 * support.  Legacy INTx interrupts keep the teardown behaviour, as their
 * level-triggered sources would otherwise keep firing while disabled.
 */
struct irq_ent {
	struct list_head	links;
	struct device	*dev;
	struct resource	*res;
	void		*arg;
	irqreturn_t	(*handler)(int, void *);
	irqreturn_t	(*thread_handler)(int, void *);
	void		*tag;
	unsigned int	irq;
	/* Lazy disable state, MSI and MSI-X only. */
	bool		lazy;
	bool		pending;
	struct thread	*busy_td;
	int		depth;
	struct mtx	mtx;
	struct task	replay_task;
};

static struct taskqueue *lkpi_irq_tq;

static void
lkpi_irq_tq_init(void *arg __unused)
{
	lkpi_irq_tq = taskqueue_create("lkpi_irq", M_WAITOK,
	    taskqueue_thread_enqueue, &lkpi_irq_tq);
	taskqueue_start_threads(&lkpi_irq_tq, 1, PI_NET, "lkpi_irq");
}
SYSINIT(lkpi_irq_tq, SI_SUB_DRIVERS, SI_ORDER_FIRST, lkpi_irq_tq_init, NULL);

static void
lkpi_irq_tq_uninit(void *arg __unused)
{
	taskqueue_free(lkpi_irq_tq);
}
SYSUNINIT(lkpi_irq_tq, SI_SUB_DRIVERS, SI_ORDER_FIRST, lkpi_irq_tq_uninit, NULL);

static inline int
lkpi_irq_rid(struct device *dev, unsigned int irq)
{
	/* check for MSI- or MSIX- interrupt */
	if (irq >= dev->irq_start && irq < dev->irq_end)
		return (irq - dev->irq_start + 1);
	else
		return (0);
}

static inline struct irq_ent *
lkpi_irq_ent(struct device *dev, unsigned int irq)
{
	struct irq_ent *irqe;

	list_for_each_entry(irqe, &dev->irqents, links)
		if (irqe->irq == irq)
			return (irqe);

	return (NULL);
}

static void
lkpi_irq_call(struct irq_ent *irqe)
{
	if (irqe->handler(irqe->irq, irqe->arg) == IRQ_WAKE_THREAD &&
	    irqe->thread_handler != NULL) {
		THREAD_SLEEPING_OK();
		irqe->thread_handler(irqe->irq, irqe->arg);
		THREAD_NO_SLEEPING();
	}
}

/*
 * Run the handler unless the interrupt is disabled or the handler is
 * already running, in which case the interrupt is left pending for
 * enable_irq() or for the running invocation to pick up.
 */
static void
lkpi_irq_run(struct irq_ent *irqe)
{
	mtx_lock(&irqe->mtx);
	for (;;) {
		if (irqe->depth > 0 || irqe->busy_td != NULL) {
			irqe->pending = true;
			break;
		}
		irqe->pending = false;
		irqe->busy_td = curthread;
		mtx_unlock(&irqe->mtx);
		lkpi_irq_call(irqe);
		mtx_lock(&irqe->mtx);
		irqe->busy_td = NULL;
		wakeup(irqe);
		if (!irqe->pending)
			break;
	}
	mtx_unlock(&irqe->mtx);
}

static void
lkpi_irq_handler(void *ent)
{
	struct irq_ent *irqe;

	if (linux_set_current_flags(curthread, M_NOWAIT))
		return;

	irqe = ent;
	if (irqe->lazy)
		lkpi_irq_run(irqe);
	else
		lkpi_irq_call(irqe);
}

static void
lkpi_irq_replay(void *ent, int pending __unused)
{
	struct irq_ent *irqe;

	if (linux_set_current_flags(curthread, M_NOWAIT))
		return;

	irqe = ent;
	THREAD_NO_SLEEPING();
	lkpi_irq_run(irqe);
	THREAD_SLEEPING_OK();
}

static inline void
lkpi_irq_release(struct device *dev, struct irq_ent *irqe)
{
	if (irqe->tag != NULL)
		bus_teardown_intr(dev->bsddev, irqe->res, irqe->tag);
	if (irqe->lazy) {
		/* The handler is gone; stop replays and let any finish. */
		mtx_lock(&irqe->mtx);
		irqe->depth++;
		mtx_unlock(&irqe->mtx);
		taskqueue_drain(lkpi_irq_tq, &irqe->replay_task);
		mtx_destroy(&irqe->mtx);
	}
	if (irqe->res != NULL)
		bus_release_resource(dev->bsddev, SYS_RES_IRQ,
		    rman_get_rid(irqe->res), irqe->res);
	list_del(&irqe->links);
}

static void
lkpi_devm_irq_release(struct device *dev, void *p)
{
	struct irq_ent *irqe;

	if (dev == NULL || p == NULL)
		return;

	irqe = p;
	lkpi_irq_release(dev, irqe);
}

int
lkpi_request_irq(struct device *xdev, unsigned int irq,
    irq_handler_t handler, irq_handler_t thread_handler,
    unsigned long flags, const char *name, void *arg)
{
	struct resource *res;
	struct irq_ent *irqe;
	struct device *dev;
	unsigned resflags;
	int error;
	int rid;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return -ENXIO;
	if (xdev != NULL && xdev != dev)
		return -ENXIO;
	rid = lkpi_irq_rid(dev, irq);
	resflags = RF_ACTIVE;
	if ((flags & IRQF_SHARED) != 0)
		resflags |= RF_SHAREABLE;
	res = bus_alloc_resource_any(dev->bsddev, SYS_RES_IRQ, &rid, resflags);
	if (res == NULL)
		return (-ENXIO);
	if (xdev != NULL)
		irqe = lkpi_devres_alloc(lkpi_devm_irq_release, sizeof(*irqe),
		    GFP_KERNEL | __GFP_ZERO);
	else
		irqe = kzalloc(sizeof(*irqe), GFP_KERNEL);
	irqe->dev = dev;
	irqe->res = res;
	irqe->arg = arg;
	irqe->handler = handler;
	irqe->thread_handler = thread_handler;
	irqe->irq = irq;
	irqe->lazy = (rid != 0);
	if (irqe->lazy) {
		mtx_init(&irqe->mtx, "lkpi_irq", NULL, MTX_DEF);
		TASK_INIT(&irqe->replay_task, 0, lkpi_irq_replay, irqe);
	}

	error = bus_setup_intr(dev->bsddev, res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, lkpi_irq_handler, irqe, &irqe->tag);
	if (error)
		goto errout;
	list_add(&irqe->links, &dev->irqents);
	if (xdev != NULL)
		devres_add(xdev, irqe);

	return 0;

errout:
	if (irqe->lazy)
		mtx_destroy(&irqe->mtx);
	bus_release_resource(dev->bsddev, SYS_RES_IRQ, rid, irqe->res);
	if (xdev != NULL)
		devres_free(irqe);
	else
		kfree(irqe);
	return (-error);
}

int
lkpi_enable_irq(unsigned int irq)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return -EINVAL;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return -EINVAL;
	if (irqe->lazy) {
		mtx_lock(&irqe->mtx);
		if (irqe->depth == 0) {
			mtx_unlock(&irqe->mtx);
			pr_warn("%s: unbalanced enable for irq %u\n",
			    __func__, irq);
			return (0);
		}
		irqe->depth--;
		/* A running handler picks a pending interrupt up itself. */
		if (irqe->depth == 0 && irqe->pending &&
		    irqe->busy_td == NULL)
			taskqueue_enqueue(lkpi_irq_tq, &irqe->replay_task);
		mtx_unlock(&irqe->mtx);
		return (0);
	}
	if (irqe->tag != NULL)
		return -EINVAL;
	return -bus_setup_intr(dev->bsddev, irqe->res, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, lkpi_irq_handler, irqe, &irqe->tag);
}

static void
lkpi_disable_irq_common(unsigned int irq, bool sync)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	if (irqe->lazy) {
		mtx_lock(&irqe->mtx);
		irqe->depth++;
		/*
		 * Wait for a running handler to finish, unless we are that
		 * handler, as Linux disable_irq() does.
		 */
		while (sync && irqe->busy_td != NULL &&
		    irqe->busy_td != curthread)
			mtx_sleep(irqe, &irqe->mtx, 0, "lkpiirq", 0);
		mtx_unlock(&irqe->mtx);
		return;
	}
	if (irqe->tag != NULL)
		bus_teardown_intr(dev->bsddev, irqe->res, irqe->tag);
	irqe->tag = NULL;
}

void
lkpi_disable_irq(unsigned int irq)
{
	lkpi_disable_irq_common(irq, true);
}

void
lkpi_disable_irq_nosync(unsigned int irq)
{
	lkpi_disable_irq_common(irq, false);
}

int
lkpi_bind_irq_to_cpu(unsigned int irq, int cpu_id)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return (-ENOENT);

	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return (-ENOENT);

	return (-bus_bind_intr(dev->bsddev, irqe->res, cpu_id));
}

void
lkpi_free_irq(unsigned int irq, void *device __unused)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	lkpi_irq_release(dev, irqe);
	kfree(irqe);
}

void
lkpi_devm_free_irq(struct device *xdev, unsigned int irq, void *p __unused)
{
	struct device *dev;
	struct irq_ent *irqe;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		return;
	if (xdev != dev)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	lkpi_irq_release(dev, irqe);
	lkpi_devres_unlink(dev, irqe);
	lkpi_devres_free(irqe);
	return;
}
