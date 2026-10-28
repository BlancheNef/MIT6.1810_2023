// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

int page_ref_count[(PHYSTOP - KERNBASE) / PGSIZE];

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  char *p = (char*)PGROUNDUP((uint64)end);
  for(; p + PGSIZE <= (char*)PHYSTOP; p += PGSIZE) {
    page_ref_count[((uint64)p - KERNBASE) >> PGSHIFT] = 0;
    kfree(p);
  }
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  acquire(&kmem.lock);
  if(page_ref_count[((uint64)pa - KERNBASE) >> PGSHIFT] > 1) {
    page_ref_count[((uint64)pa - KERNBASE) >> PGSHIFT]--;
    release(&kmem.lock);
    return;
  }
  release(&kmem.lock);

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  acquire(&kmem.lock);
  r->next = kmem.freelist;
  kmem.freelist = r;
  page_ref_count[((uint64)pa - KERNBASE) >> PGSHIFT] = 0;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r) {
    page_ref_count[((uint64)r - KERNBASE) >> PGSHIFT] = 1;
    kmem.freelist = r->next;
  }
  release(&kmem.lock);

  if(r)
    memset((char*)r, 5, PGSIZE); // fill with junk
  return (void*)r;
}

void 
add_page_ref_count(uint64 pa, int ref_count)
{
  acquire(&kmem.lock);
  page_ref_count[(pa - KERNBASE) >> PGSHIFT] += ref_count;
  release(&kmem.lock);
}

int
get_page_ref_count(uint64 pa) {
  int count = 0;
  acquire(&kmem.lock);
  count = page_ref_count[(pa - KERNBASE) >> PGSHIFT];
  release(&kmem.lock);
  return count;
}
