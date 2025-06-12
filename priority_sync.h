#ifndef __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__
#define __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__

#include <list.h>
#include "threads/thread.h"
#include "threads/synch.h"  // 이 줄을 꼭 넣자

struct vehicle_info; 

/* Priority Semaphore */
struct priority_semaphore {
    unsigned value;
    struct list waiters;
};

struct waiter {
    struct list_elem elem;
    struct thread *t;
    struct vehicle_info *vi;
};

void priority_sema_init(struct priority_semaphore *ps, unsigned value);
void priority_sema_down(struct priority_semaphore *ps, struct vehicle_info *vi);
void priority_sema_up(struct priority_semaphore *ps);

/* Priority Lock */
struct priority_lock {
    struct thread *holder;
    struct priority_semaphore semaphore;
};

void priority_lock_init(struct priority_lock *lock);
void priority_lock_acquire(struct priority_lock *lock, struct vehicle_info *vi);
void priority_lock_release(struct priority_lock *lock);

/* Priority Condition */
struct priority_condition {
    struct list waiters;
};

void priority_cond_init(struct priority_condition *cond);
void priority_cond_wait(struct priority_condition *cond, struct priority_lock *lock, struct vehicle_info *vi);
void priority_cond_signal(struct priority_condition *cond);
void priority_cond_broadcast(struct priority_condition *cond);

void lock_acquire_priority(struct lock *lock, struct vehicle_info *vi);
void lock_release_priority(struct lock *lock);

#endif /* __PROJECTS_CROSSROADS_PRIORITY_SYNC_H__ */
