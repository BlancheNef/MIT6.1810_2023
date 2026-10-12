// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define PRIME 13

struct {
  struct spinlock lock;
  struct buf buf[NBUF];
} bcache;

struct {
  struct buf *data_list; 
  struct spinlock lock;
} hash_buckets[PRIME];

void double_link_add_to_head(struct buf **list, struct buf *b) {
  if ((*list) == 0) {
    b->next = b;
    b->prev = b;
  } else {
    b->next = (*list);
    b->prev = (*list)->prev;
    (*list)->prev->next = b;
    (*list)->prev = b;
  }
  *list = b;
}

void double_link_lift_to_head(struct buf **list, struct buf *b) {
  // if only one node in list, that node must be b
  if(*list == b) return;

  b->prev->next = b->next;
  b->next->prev = b->prev;
  double_link_add_to_head(list, b);
} 

void double_link_rm(struct buf **list, struct buf *b){
  if(b->next == b) {
    // only one node
    (*list) = 0; 
    b->prev = b->next = 0;
    return;
  }

  b->prev->next = b->next;
  b->next->prev = b->prev;
  if((*list) == b) {
    (*list) = b->next;
  }
  b->prev = b->next = 0;
}

void
binit(void)
{
  struct buf *b;

  initlock(&bcache.lock, "bcache");

  for(int i=0; i<NBUF; i++) {
    b = &bcache.buf[i];
    initsleeplock(&b->lock, "buffer");
    b->bucket = -1;
  }
  // initialized the bucket lock
  for(int i=0; i<PRIME; i++) {
    initlock(&hash_buckets[i].lock, "bcache_hash");
  }
}

int last_index = 0;
// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  // Is the block already cached?
  int bucket = (dev + blockno) % PRIME;
  acquire(&hash_buckets[bucket].lock);
  b = hash_buckets[bucket].data_list;
  if(b != 0) {
    while(b != 0){
      if (b->dev == dev && b->blockno == blockno){
        // found
        b->refcnt++;
        release(&hash_buckets[bucket].lock);
        acquiresleep(&b->lock);
        return b;
      }
      b = b->next;
      if(b == hash_buckets[bucket].data_list) break;
    }
  }

  release(&hash_buckets[bucket].lock);
  acquire(&bcache.lock);
  // check again
  acquire(&hash_buckets[bucket].lock);
  b = hash_buckets[bucket].data_list;
  if(b != 0) {
    while(b != 0){
      if (b->dev == dev && b->blockno == blockno){
        // found
        b->refcnt++;
        release(&bcache.lock);
        release(&hash_buckets[bucket].lock);
        acquiresleep(&b->lock);
        return b;
      }
      b = b->next;
      if(b == hash_buckets[bucket].data_list) break;
    }
  }
  release(&hash_buckets[bucket].lock);

  int i = 0;
  for(;i<NBUF;i++) {
    b = &bcache.buf[(last_index + i) % NBUF];
    if(b->bucket == -1)
      break;

    acquire(&hash_buckets[b->bucket].lock);
    if(b->refcnt == 0){
      double_link_rm(&hash_buckets[b->bucket].data_list, b);
      release(&hash_buckets[b->bucket].lock);
      break;
    }
    release(&hash_buckets[b->bucket].lock);
  }

  if (i == NBUF) 
    panic("bget: no buffers");

  last_index = (last_index + i + 1) % NBUF;


  acquire(&hash_buckets[bucket].lock);
  b->dev = dev;
  b->blockno = blockno;
  b->valid = 0;
  b->refcnt = 1;
  b->bucket = bucket;
  double_link_add_to_head(&hash_buckets[bucket].data_list, b);

  release(&bcache.lock);
  release(&hash_buckets[bucket].lock);
  acquiresleep(&b->lock);
  return b;
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int bucket = (b->dev + b->blockno) % PRIME;
  acquire(&hash_buckets[bucket].lock);
  b->refcnt--;
  release(&hash_buckets[bucket].lock);
}

void
bpin(struct buf *b) {
  int bucket = (b->dev + b->blockno) % PRIME;
  acquire(&hash_buckets[bucket].lock);
  b->refcnt++;
  release(&hash_buckets[bucket].lock);
}

void
bunpin(struct buf *b) {
  int bucket = (b->dev + b->blockno) % PRIME;
  acquire(&hash_buckets[bucket].lock);
  b->refcnt--;
  release(&hash_buckets[bucket].lock);
}