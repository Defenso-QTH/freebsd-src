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
 * FreeBSD glue of the LinuxKPI MHI host core (linux_mhi.c): interrupts,
 * including the test interrupts of the device model, the debug and state
 * dump sysctls, and the module handler.  The client devices and drivers of
 * the MHI bus are in linux_mhi.c.
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

#include "linux_mhi_reg.h"
#include "linux_qrtr.h"

int mhi_fbsd_debug = 0;
SYSCTL_DECL(_compat_linuxkpi);
SYSCTL_INT(_compat_linuxkpi, OID_AUTO, mhi_debug, CTLFLAG_RWTUN,
    &mhi_fbsd_debug, 0, "Print MHI bus debug messages");

static int
mhi_fbsd_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	val = 0;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val == 1)
		lkpi_mhi_dump_all();
	return (0);
}
SYSCTL_PROC(_compat_linuxkpi, OID_AUTO, mhi_dump,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0, mhi_fbsd_sysctl_dump,
    "I", "Write 1 to print the state of every MHI controller");

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
		/* A controller still registered may have client devices. */
		if (lkpi_mhi_busy() || lkpi_qrtr_busy())
			return (EBUSY);
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
