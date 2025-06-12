#include "projects/crossroads/priority_sync.h"
#include "threads/thread.h"
#include <stdio.h>
#include <stdlib.h>
#include "projects/crossroads/vehicle.h"

bool ambulance_first(const struct list_elem *a, const struct list_elem *b, void *aux);

void priority_cond_init(struct priority_condition *cond) {
    list_init(&cond->waiters);
}

void priority_cond_wait(struct priority_condition *cond,
                        struct priority_lock *lock,
                        struct vehicle_info *vi) {
    struct waiter *w = malloc(sizeof(struct waiter));
    w->t = thread_current();
    w->vi = vi;
    list_insert_ordered(&cond->waiters, &w->elem, ambulance_first, NULL);
    priority_lock_release(lock);
    thread_block();
    priority_lock_acquire(lock, vi);
}

void priority_cond_signal(struct priority_condition *cond) {
    if (!list_empty(&cond->waiters)) {
        struct list_elem *e = list_pop_front(&cond->waiters);
        struct waiter *w = list_entry(e, struct waiter, elem);
        thread_unblock(w->t);
        free(w);
    }
}

void priority_cond_broadcast(struct priority_condition *cond) {
    while (!list_empty(&cond->waiters)) {
        priority_cond_signal(cond);
    }
}
