/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2011 NetApp, Inc.
 * All rights reserved.
 */

#include <sys/types.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/sx.h>
#include <sys/systm.h>

#include <machine/vmm.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <dev/vmm/vmm_dev.h>
#include <dev/vmm/vmm_mem.h>
#include <dev/vmm/vmm_vm.h>

static void vm_free_memmap(struct vm *vm, int ident);

int
vm_mem_init(struct vm_mem *mem, vm_offset_t lo, vm_offset_t hi)
{
	mem->mem_vmspace = vmmops_vmspace_alloc(lo, hi);
	if (mem->mem_vmspace == NULL)
		return (ENOMEM);
	sx_init(&mem->mem_segs_lock, "vm_mem_segs");
	return (0);
}

static bool
sysmem_mapping(struct vm_mem *mem, int idx)
{
	if (mem->mem_maps[idx].len != 0 &&
	    mem->mem_segs[mem->mem_maps[idx].segid].sysmem)
		return (true);
	else
		return (false);
}

bool
vm_memseg_sysmem(struct vm *vm, int ident)
{
	struct vm_mem *mem;

	mem = vm_mem(vm);
	vm_assert_memseg_locked(vm);

	if (ident < 0 || ident >= VM_MAX_MEMSEGS)
		return (false);

	return (mem->mem_segs[ident].sysmem);
}

void
vm_mem_cleanup(struct vm *vm)
{
	struct vm_mem *mem;

	mem = vm_mem(vm);

	/*
	 * System memory is removed from the guest address space only when
	 * the VM is destroyed. This is because the mapping remains the same
	 * across VM reset.
	 *
	 * Device memory can be relocated by the guest (e.g. using PCI BARs)
	 * so those mappings are removed on a VM reset.
	 */
	for (int i = 0; i < VM_MAX_MEMMAPS; i++) {
		if (!sysmem_mapping(mem, i))
			vm_free_memmap(vm, i);
	}
}

void
vm_mem_destroy(struct vm *vm)
{
	struct vm_mem *mem;

	mem = vm_mem(vm);
	vm_assert_memseg_xlocked(vm);

	for (int i = 0; i < VM_MAX_MEMMAPS; i++) {
		if (sysmem_mapping(mem, i))
			vm_free_memmap(vm, i);
	}

	for (int i = 0; i < VM_MAX_MEMSEGS; i++)
		vm_free_memseg(vm, i);

	vmmops_vmspace_free(mem->mem_vmspace);

	sx_xunlock(&mem->mem_segs_lock);
	sx_destroy(&mem->mem_segs_lock);
}

struct vmspace *
vm_vmspace(struct vm *vm)
{
	struct vm_mem *mem;

	mem = vm_mem(vm);
	return (mem->mem_vmspace);
}

void
vm_slock_memsegs(struct vm *vm)
{
	sx_slock(&vm_mem(vm)->mem_segs_lock);
}

void
vm_xlock_memsegs(struct vm *vm)
{
	sx_xlock(&vm_mem(vm)->mem_segs_lock);
}

void
vm_unlock_memsegs(struct vm *vm)
{
	sx_unlock(&vm_mem(vm)->mem_segs_lock);
}

void
vm_assert_memseg_locked(struct vm *vm)
{
	sx_assert(&vm_mem(vm)->mem_segs_lock, SX_LOCKED);
}

void
vm_assert_memseg_xlocked(struct vm *vm)
{
	sx_assert(&vm_mem(vm)->mem_segs_lock, SX_XLOCKED);
}

/*
 * Return 'true' if 'gpa' is allocated in the guest address space.
 *
 * This function is called in the context of a running vcpu which acts as
 * an implicit lock on 'vm->mem_maps[]'.
 */
bool
vm_mem_allocated(struct vcpu *vcpu, vm_paddr_t gpa)
{
	struct vm *vm = vcpu_vm(vcpu);
	struct vm_mem_map *mm;
	int i;

#ifdef INVARIANTS
	int hostcpu, state;
	state = vcpu_get_state(vcpu, &hostcpu);
	KASSERT(state == VCPU_RUNNING && hostcpu == curcpu,
	    ("%s: invalid vcpu state %d/%d", __func__, state, hostcpu));
#endif

	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		mm = &vm_mem(vm)->mem_maps[i];
		if (mm->len != 0 && gpa >= mm->gpa && gpa < mm->gpa + mm->len)
			return (true);		/* 'gpa' is sysmem or devmem */
	}

	return (false);
}

int
vm_alloc_memseg(struct vm *vm, int ident, size_t len, bool sysmem,
    struct domainset *obj_domainset)
{
	struct vm_mem_seg *seg;
	struct vm_mem *mem;
	vm_object_t obj;

	mem = vm_mem(vm);
	vm_assert_memseg_xlocked(vm);

	if (ident < 0 || ident >= VM_MAX_MEMSEGS)
		return (EINVAL);

	if (len == 0 || (len & PAGE_MASK))
		return (EINVAL);

	seg = &mem->mem_segs[ident];
	if (seg->object != NULL) {
		if (seg->len == len && seg->sysmem == sysmem)
			return (EEXIST);
		else
			return (EINVAL);
	}

	/*
	 * When given an impossible policy, signal an
	 * error to the user.
	 */
	if (obj_domainset != NULL && domainset_empty_vm(obj_domainset))
		return (EINVAL);
	obj = vm_object_allocate(OBJT_SWAP, len >> PAGE_SHIFT);
	if (obj == NULL)
		return (ENOMEM);

	seg->len = len;
	seg->object = obj;
	if (obj_domainset != NULL)
		seg->object->domain.dr_policy = obj_domainset;
	seg->sysmem = sysmem;

	return (0);
}

int
vm_get_memseg(struct vm *vm, int ident, size_t *len, bool *sysmem,
    vm_object_t *objptr)
{
	struct vm_mem *mem;
	struct vm_mem_seg *seg;

	mem = vm_mem(vm);

	vm_assert_memseg_locked(vm);

	if (ident < 0 || ident >= VM_MAX_MEMSEGS)
		return (EINVAL);

	seg = &mem->mem_segs[ident];
	if (len)
		*len = seg->len;
	if (sysmem)
		*sysmem = seg->sysmem;
	if (objptr)
		*objptr = seg->object;
	return (0);
}

void
vm_free_memseg(struct vm *vm, int ident)
{
	struct vm_mem_seg *seg;

	KASSERT(ident >= 0 && ident < VM_MAX_MEMSEGS,
	    ("%s: invalid memseg ident %d", __func__, ident));

	seg = &vm_mem(vm)->mem_segs[ident];
	if (seg->object != NULL) {
		vm_object_deallocate(seg->object);
		bzero(seg, sizeof(struct vm_mem_seg));
	}
}

int
vm_mmap_memseg(struct vm *vm, vm_paddr_t gpa, int segid, vm_ooffset_t first,
    size_t len, int prot, int flags)
{
	struct vm_mem *mem;
	struct vm_mem_seg *seg;
	struct vm_mem_map *m, *map;
	struct vm_map *vmmap;
	vm_ooffset_t last;
	int i, error;

	if (prot == 0 || (prot & ~(VM_PROT_ALL)) != 0)
		return (EINVAL);

	if (flags & ~VM_MEMMAP_F_WIRED)
		return (EINVAL);

	if (segid < 0 || segid >= VM_MAX_MEMSEGS)
		return (EINVAL);

	mem = vm_mem(vm);
	seg = &mem->mem_segs[segid];
	if (seg->object == NULL)
		return (EINVAL);

	if (first + len < first || gpa + len < gpa)
		return (EINVAL);
	last = first + len;
	if (first >= last || last > seg->len)
		return (EINVAL);

	if ((gpa | first | last) & PAGE_MASK)
		return (EINVAL);

	map = NULL;
	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		m = &mem->mem_maps[i];
		if (m->len == 0) {
			map = m;
			break;
		}
	}
	if (map == NULL)
		return (ENOSPC);

	vmmap = &mem->mem_vmspace->vm_map;
	vm_map_lock(vmmap);
	error = vm_map_insert(vmmap, seg->object, first, gpa, gpa + len,
	    prot, prot, 0);
	vm_map_unlock(vmmap);
	if (error != KERN_SUCCESS)
		return (vm_mmap_to_errno(error));
	vm_object_reference(seg->object);

	if (flags & VM_MEMMAP_F_WIRED) {
		error = vm_map_wire(vmmap, gpa, gpa + len,
		    VM_MAP_WIRE_USER | VM_MAP_WIRE_NOHOLES);
		if (error != KERN_SUCCESS) {
			vm_map_remove(vmmap, gpa, gpa + len);
			return (error == KERN_RESOURCE_SHORTAGE ? ENOMEM :
			    EFAULT);
		}
	}

	map->gpa = gpa;
	map->len = len;
	map->segoff = first;
	map->segid = segid;
	map->prot = prot;
	map->flags = flags;
	return (0);
}

int
vm_munmap_memseg(struct vm *vm, vm_paddr_t gpa, size_t len)
{
	struct vm_mem *mem;
	struct vm_mem_map *m;
	int i;

	mem = vm_mem(vm);
	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		m = &mem->mem_maps[i];
#ifdef VM_MEMMAP_F_IOMMU
		if ((m->flags & VM_MEMMAP_F_IOMMU) != 0)
			continue;
#endif
		if (m->gpa == gpa && m->len == len) {
			vm_free_memmap(vm, i);
			return (0);
		}
	}

	return (EINVAL);
}

/*
 * Alias a range [hva, hva+len) of the calling (VMM) process's address space
 * into the guest's physical address space at 'gpa'.  Used by the bhyve
 * virtio-gpu device to expose a host-visible venus blob (which virglrenderer
 * has mmap'd at 'hva') to the guest inside the device's host-visible BAR
 * window -- zero-copy, coherent, like vm_mmap_memseg but backed by whatever
 * VM object currently backs 'hva' (an shm/dmabuf mapping) instead of a memseg.
 *
 * Any existing guest mapping over [gpa, gpa+len) is replaced.
 */
int
vm_mmap_blob(struct vm *vm, vm_paddr_t gpa, uintptr_t hva, size_t len, int prot)
{
	struct vmspace *uvmspace;
	vm_map_t umap, gmap;
	vm_map_entry_t entry;
	vm_object_t obj;
	vm_ooffset_t obj_off;
	int error;

	if (prot == 0 || (prot & ~(VM_PROT_ALL)) != 0)
		return (EINVAL);
	if (len == 0 || (gpa | hva | len) & PAGE_MASK)
		return (EINVAL);
	if (hva + len < hva || gpa + len < gpa)
		return (EINVAL);

	/*
	 * Find the VM object backing the host mapping and take a reference on
	 * it that we will hand to the guest map entry.  The whole range must
	 * fall inside a single map entry (virglrenderer maps a blob as one
	 * contiguous mmap, so this holds).
	 */
	uvmspace = curproc->p_vmspace;
	umap = &uvmspace->vm_map;
	vm_map_lock_read(umap);
	if (!vm_map_lookup_entry(umap, hva, &entry) ||
	    entry->end < hva + len ||
	    (entry->eflags & (MAP_ENTRY_IS_SUB_MAP | MAP_ENTRY_GUARD)) != 0) {
		vm_map_unlock_read(umap);
		return (EINVAL);
	}
	obj = entry->object.vm_object;
	if (obj == NULL) {
		vm_map_unlock_read(umap);
		return (EINVAL);
	}
	obj_off = entry->offset + (hva - entry->start);
	vm_object_reference(obj);
	vm_map_unlock_read(umap);

	/*
	 * Drop any prior guest mapping over the target range, then insert the
	 * host object.  Callers hold the memseg xlock with all vcpus frozen, so
	 * the window between remove and insert cannot race an EPT fault.  On
	 * failure release the reference we took above.
	 */
	gmap = &vm_vmspace(vm)->vm_map;
	(void)vm_map_remove(gmap, gpa, gpa + len);
	vm_map_lock(gmap);
	error = vm_map_insert(gmap, obj, obj_off, gpa, gpa + len, prot, prot, 0);
	vm_map_unlock(gmap);
	if (error != KERN_SUCCESS) {
		vm_object_deallocate(obj);
		return (vm_mmap_to_errno(error));
	}

	return (0);
}

int
vm_munmap_blob(struct vm *vm, vm_paddr_t gpa, size_t len)
{
	int error;

	if (len == 0 || (gpa | len) & PAGE_MASK)
		return (EINVAL);
	if (gpa + len < gpa)
		return (EINVAL);

	error = vm_map_remove(&vm_vmspace(vm)->vm_map, gpa, gpa + len);

	return (error == KERN_SUCCESS ? 0 : EINVAL);
}

int
vm_mmap_getnext(struct vm *vm, vm_paddr_t *gpa, int *segid,
    vm_ooffset_t *segoff, size_t *len, int *prot, int *flags)
{
	struct vm_mem *mem;
	struct vm_mem_map *mm, *mmnext;
	int i;

	mem = vm_mem(vm);

	mmnext = NULL;
	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		mm = &mem->mem_maps[i];
		if (mm->len == 0 || mm->gpa < *gpa)
			continue;
		if (mmnext == NULL || mm->gpa < mmnext->gpa)
			mmnext = mm;
	}

	if (mmnext != NULL) {
		*gpa = mmnext->gpa;
		if (segid)
			*segid = mmnext->segid;
		if (segoff)
			*segoff = mmnext->segoff;
		if (len)
			*len = mmnext->len;
		if (prot)
			*prot = mmnext->prot;
		if (flags)
			*flags = mmnext->flags;
		return (0);
	} else {
		return (ENOENT);
	}
}

static void
vm_free_memmap(struct vm *vm, int ident)
{
	struct vm_mem_map *mm;
	int error __diagused;

	mm = &vm_mem(vm)->mem_maps[ident];
	if (mm->len) {
		error = vm_map_remove(&vm_vmspace(vm)->vm_map, mm->gpa,
		    mm->gpa + mm->len);
		KASSERT(error == KERN_SUCCESS, ("%s: vm_map_remove error %d",
		    __func__, error));
		bzero(mm, sizeof(struct vm_mem_map));
	}
}

vm_paddr_t
vmm_sysmem_maxaddr(struct vm *vm)
{
	struct vm_mem *mem;
	struct vm_mem_map *mm;
	vm_paddr_t maxaddr;
	int i;

	mem = vm_mem(vm);
	maxaddr = 0;
	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		mm = &mem->mem_maps[i];
		if (sysmem_mapping(mem, i)) {
			if (maxaddr < mm->gpa + mm->len)
				maxaddr = mm->gpa + mm->len;
		}
	}
	return (maxaddr);
}

static void *
_vm_gpa_hold(struct vm *vm, vm_paddr_t gpa, size_t len, int reqprot,
    void **cookie)
{
	struct vm_mem_map *mm;
	vm_page_t m;
	int i, count, pageoff;

	pageoff = gpa & PAGE_MASK;
	if (len > PAGE_SIZE - pageoff)
		panic("vm_gpa_hold: invalid gpa/len: 0x%016lx/%lu", gpa, len);

	count = 0;
	for (i = 0; i < VM_MAX_MEMMAPS; i++) {
		mm = &vm_mem(vm)->mem_maps[i];
		if (gpa >= mm->gpa && gpa < mm->gpa + mm->len) {
			count = vm_fault_quick_hold_pages(
			    &vm_vmspace(vm)->vm_map, trunc_page(gpa),
			    PAGE_SIZE, reqprot, &m, 1);
			break;
		}
	}

	if (count == 1) {
		*cookie = m;
		return ((void *)(PHYS_TO_DMAP(VM_PAGE_TO_PHYS(m)) + pageoff));
	} else {
		*cookie = NULL;
		return (NULL);
	}
}

void *
vm_gpa_hold(struct vcpu *vcpu, vm_paddr_t gpa, size_t len, int reqprot,
    void **cookie)
{
#ifdef INVARIANTS
	/*
	 * The current vcpu should be frozen to ensure 'vm_memmap[]'
	 * stability.
	 */
	int state = vcpu_get_state(vcpu, NULL);
	KASSERT(state == VCPU_FROZEN, ("%s: invalid vcpu state %d",
	    __func__, state));
#endif
	return (_vm_gpa_hold(vcpu_vm(vcpu), gpa, len, reqprot, cookie));
}

void *
vm_gpa_hold_global(struct vm *vm, vm_paddr_t gpa, size_t len, int reqprot,
    void **cookie)
{
	vm_assert_memseg_locked(vm);
	return (_vm_gpa_hold(vm, gpa, len, reqprot, cookie));
}

void
vm_gpa_release(void *cookie)
{
	vm_page_t m = cookie;

	vm_page_unwire(m, PQ_ACTIVE);
}
