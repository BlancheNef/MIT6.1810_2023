#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h"
#include "fcntl.h"

int on_demand_page(struct proc* p, uint64 va){
  if(va > MAXVA) {
    return -1;
  }

  int index = p->vma_head;
  while(index != -1) {
    if(p->vma[index].start <= va &&  va < p->vma[index].end) {
      break;
    }
    index = p->vma[index].next;
  }

  if(index == -1) {
    return -1; // error
  }

  uint64 pa = (uint64)kalloc();
  if(pa == 0) {
    return -1;
  }

  va = PGROUNDDOWN(va);
  int off = va - p->vma[index].off_start;
  ilock(p->vma[index].file->ip);
  // readi may not return PGSIZE because off + PGSIZE > file->size
  int read_size = readi(p->vma[index].file->ip, 0, pa, off, PGSIZE);
  iunlock(p->vma[index].file->ip);

  // padding zero
  memset((void*)(pa+read_size), 0, PGSIZE - read_size);

  int perm = 0;
  if(p->vma[index].prot & PROT_READ) {
    perm |= PTE_R;
  }
  if(p->vma[index].prot & PROT_WRITE) {
    perm |= PTE_W;
  }
  if(p->vma[index].prot & PROT_EXEC) {
    perm |= PTE_X;
  }
  perm |= PTE_V;
  perm |= PTE_U;
  
  if(mappages(p->pagetable, va, PGSIZE, pa, perm) != 0){
    return -1;
  }

  // printf("pid: %d, map %p to %p\n", p->pid, va, pa);
  
  return 0;
}

int write_page_fault(struct proc* p, uint64 va)
{
  if (va >= MAXVA) {
    return -1;
  }
  va = PGROUNDDOWN(va);
  pte_t* pte = walk(p->pagetable, va, 0);
  if(pte == 0 || (*pte & PTE_V) == 0) {
    // need page in!
    return on_demand_page(p, va);
  } else {
    return handle_copy_on_write(va, p->pagetable);
  }
}

void freevma(struct proc* p, int index, uint64 start, uint64 end){
    int n = PGROUNDUP(end - start) / PGSIZE;
    for(int i=0; i<n; i++){
      uint64 va = start+i*PGSIZE;
      pte_t* pte = walk(p->pagetable, va, 0);
      if(*pte & PTE_V){
        if(p->vma[index].flag & MAP_SHARED) {
          if(*pte & PTE_D) {
            uint64 pa = PTE2PA(*pte);
            int off = va - p->vma[index].off_start;
            begin_op();
            ilock(p->vma[index].file->ip);
            writei(p->vma[index].file->ip, 0, pa, off, PGSIZE);
            iunlock(p->vma[index].file->ip);
            end_op();
            *pte &= ~PTE_D;
          }
        }
        uvmunmap(p->pagetable, start+i*PGSIZE, 1, 1);
        // printf("pid: %d, unmap %p\n", p->pid, start+i*PGSIZE);
      }
    }
}

void uvmfreevma(struct proc* p)
{
  int index = p->vma_head;
  while(index != -1) {
    uint64 start = p->vma[index].start;
    uint64 end = p->vma[index].end;
    freevma(p, index, start, end);
    fileclose(p->vma[index].file);
    p->vma[index].valid = 0;
    index = p->vma[index].next;
  }
  p->vma_head = -1;
}

void fork_vma(struct proc* parent, struct proc* child) {
  int index = parent->vma_head;
  child->vma_head = parent->vma_head;
  while (index != -1)
  {
    child->vma[index].start = parent->vma[index].start;
    child->vma[index].end = parent->vma[index].end;
    child->vma[index].flag = parent->vma[index].flag;
    child->vma[index].prot = parent->vma[index].prot;
    child->vma[index].prev = parent->vma[index].prev;
    child->vma[index].next = parent->vma[index].next;
    child->vma[index].file = parent->vma[index].file;
    child->vma[index].off_start = parent->vma[index].off_start;
    filedup(parent->vma[index].file);

    uint64 start = parent->vma[index].start;
    uint64 end = parent->vma[index].end;
    int n = (end - start) / PGSIZE;
    for(int i=0; i<n; i++){
      uint64 va = start+i*PGSIZE;
      pte_t* pte = walk(parent->pagetable, va, 0);
      if(pte != 0 && (*pte & PTE_V)){
        uint64 pa = PTE2PA(*pte);
        // 1. W = 1 -> W = 0, C = 1.
        // 2. W = 0, C = 0 -> W = 0, C = 0.
        // 3. W = 0, C = 1 -> W = 0, C = 1. 
        if((*pte & PTE_W)) {
          *pte &= ~PTE_W;
          *pte |= PTE_C;
        }
        add_page_ref_count(pa, 1);
        mappages(child->pagetable, va, PGSIZE, (uint64)pa, PTE_FLAGS(*pte) & ~PTE_D); // clear Dirty bit
        // printf("atfork, pid: %d, map %p to %p\n", child->pid, va, pa);
      }
    }
    child->vma[index].valid = 1;
    index = parent->vma[index].next;
  }
}

uint64
sys_mmap(void)
{
  int len, prot, flag, fd;
  argint(1, &len);
  argint(2, &prot);
  argint(3, &flag);
  argint(4, &fd);

  struct proc *p = myproc();
  struct file* f =  p-> ofile[fd];

  // check that mmap doesn't allow read/write mapping of a file opened read-only.
  // should be able to map file opened read-only with private writable
  if(f->writable == 0 && (prot & PROT_WRITE)) {
    if(!(flag & MAP_PRIVATE))
      return 0xffffffffffffffff;
  }

  len = PGROUNDUP(len);

  int free_slot = -1;
  int index = p->vma_head;
  if(index == -1) {
    free_slot = 0;
    p->vma[0].start = MAXVA - 2 * PGSIZE - len;
    p->vma[0].end =  MAXVA - 2 * PGSIZE;
    p->vma[0].prev = -1;
    p->vma[0].next = -1;
    p->vma_head = 0;
  } else {
    for(int i=0;i<16;i++){
        if(p->vma[i].valid == 0){
            free_slot = i;
            break;
        }
    }

    while(p->vma[index].next != -1){
        if(p->vma[index].start - p->vma[p->vma[index].next].end >= len){
            break;
        } else {
            index = p->vma[index].next;
        }
    }

    p->vma[free_slot].start = p->vma[index].start - len;
    p->vma[free_slot].end =  p->vma[index].start;
    p->vma[free_slot].prev = index;
    p->vma[free_slot].next = p->vma[index].next;
    if(p->vma[index].next != -1) {
      p->vma[p->vma[index].next].prev = free_slot;
    }
    p->vma[index].next = free_slot;

  }

  p->vma[free_slot].file = f;
  p->vma[free_slot].prot = prot;
  p->vma[free_slot].flag = flag;
  p->vma[free_slot].off_start = p->vma[free_slot].start;

  p->vma[free_slot].valid = 1; 

   // dup the file
  filedup(f);

  return p->vma[free_slot].start;
}

uint64
sys_munmap(void)
{
  uint64 addr;
  int len;
  argaddr(0, &addr);
  argint(1, &len);

  if(addr % PGSIZE != 0){
    return -1;
  }

  struct proc *p = myproc();

  int index = p->vma_head;
  while (index != -1)
  {
    if(addr == p->vma[index].start || addr+len == p->vma[index].end) {
      break;
    }
    index = p->vma[index].next;
  }
  
  if(index == -1){
    return -1;
  }

  freevma(p, index, addr, addr+len);

  if(addr == p->vma[index].start && addr+len == p->vma[index].end) {
    fileclose(p->vma[index].file);
    p->vma[index].valid = 0;
    if(p->vma_head == index) {
      if(p->vma[index].next != -1) {
        p->vma[p->vma[index].next].prev = -1;
      }
      p->vma_head = p->vma[index].next;
    } else if (p->vma[index].next == -1) {
      // tail but not head
      p->vma[p->vma[index].prev].next = -1;
    } else {
      p->vma[p->vma[index].prev].next = p->vma[index].next;
      p->vma[p->vma[index].next].prev = p->vma[index].prev;
    }
  } else if (addr == p->vma[index].start) {
    p->vma[index].start += len; 
  } else if (addr+len == p->vma[index].end) {
    p->vma[index].end -= len;
  } else {
    return -1;
  }

  return 0;
}
