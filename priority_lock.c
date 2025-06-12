#include "projects/crossroads/priority_sync.h"
#include "threads/thread.h"
#include <stdio.h>

void priority_lock_init(struct priority_lock *lock) {
    lock->holder = NULL;
    priority_sema_init(&lock->semaphore, 1);
}

void priority_lock_acquire(struct priority_lock *lock, struct vehicle_info *vi) {
    priority_sema_down(&lock->semaphore, vi);
    lock->holder = thread_current();
}

void priority_lock_release(struct priority_lock *lock) {
    lock->holder = NULL;
    priority_sema_up(&lock->semaphore);
}
